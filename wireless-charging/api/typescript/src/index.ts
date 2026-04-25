/**
 * WIA-COMM-008: Wireless Charging - TypeScript SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Communication Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

import type {
  ChargingStandard,
  TransmitterCoil,
  ReceiverCoil,
  PowerTransferConfig,
  PowerTransferResult,
  FODConfig,
  FODResult,
  AlignmentStatus,
  ThermalStatus,
  EMFMeasurement,
  EMFStandard,
  ChargingSession,
  WirelessChargingConfig,
  WirelessChargingEvent,
  EventHandler,
  ImpedanceMatchingConfig,
  ResonanceTuningConfig,
  EfficiencyOptimizationResult,
  MultiDeviceChargerConfig,
  MultiDeviceStatus,
  ChargingDevice,
  EVChargingSession,
  GroundAssembly,
  VehicleAssembly,
} from './types';

// ============================================================================
// Physical Constants
// ============================================================================

const CONSTANTS = {
  /** Permeability of free space (H/m) */
  MU_0: 4 * Math.PI * 1e-7,

  /** Permittivity of free space (F/m) */
  EPSILON_0: 8.854187817e-12,

  /** Speed of light (m/s) */
  SPEED_OF_LIGHT: 299792458,
} as const;

// ============================================================================
// Wireless Charging SDK
// ============================================================================

export class WirelessChargingSDK {
  private config: WirelessChargingConfig;
  private eventHandlers: Map<string, EventHandler[]> = new Map();
  private activeSessions: Map<string, ChargingSession> = new Map();

  constructor(config: WirelessChargingConfig) {
    this.config = config;
  }

  // ==========================================================================
  // Power Transfer Calculations
  // ==========================================================================

  /**
   * Calculate coupling coefficient between two coils
   */
  calculateCouplingCoefficient(
    transmitter: TransmitterCoil,
    receiver: ReceiverCoil,
    distance: number,
    lateralOffset: { x: number; y: number } = { x: 0, y: 0 }
  ): number {
    // Convert to meters
    const d = distance / 1000;
    const r1 = (transmitter.outerDiameter / 2) / 1000;
    const r2 = (receiver.outerDiameter / 2) / 1000;

    // Calculate offset magnitude
    const offsetMag = Math.sqrt(lateralOffset.x ** 2 + lateralOffset.y ** 2) / 1000;

    // Simplified coupling coefficient formula for circular coils
    // k ≈ (r1 * r2)^2 / [(r1 * r2)^2 + (d + offset)^2]^(3/2)
    const effectiveDistance = Math.sqrt(d ** 2 + offsetMag ** 2);
    const numerator = (r1 * r2) ** 2;
    const denominator = (numerator + effectiveDistance ** 2) ** 1.5;

    return numerator / denominator;
  }

  /**
   * Calculate mutual inductance between coils
   */
  calculateMutualInductance(
    transmitter: TransmitterCoil,
    receiver: ReceiverCoil,
    couplingCoefficient: number
  ): number {
    // M = k * sqrt(L1 * L2)
    // Convert inductances from μH to H
    const L1 = transmitter.inductance * 1e-6;
    const L2 = receiver.inductance * 1e-6;

    return couplingCoefficient * Math.sqrt(L1 * L2);
  }

  /**
   * Calculate power transfer
   */
  calculatePowerTransfer(config: PowerTransferConfig): PowerTransferResult {
    const { transmitter, receiver, distance, lateralOffset, inputPower } = config;

    // Calculate coupling
    const k = this.calculateCouplingCoefficient(transmitter, receiver, distance, lateralOffset);
    const M = this.calculateMutualInductance(transmitter, receiver, k);

    // Calculate transferred power (simplified)
    const Q1 = transmitter.qualityFactor;
    const Q2 = receiver.qualityFactor;
    const powerTransferFactor = (k ** 2 * Q1 * Q2) / (1 + k ** 2 * Q1 * Q2);

    const transferredPower = inputPower * powerTransferFactor;

    // Calculate losses
    const coilLosses = inputPower * (1 - powerTransferFactor) * 0.7; // 70% of losses in coils
    const rectifierLosses = transferredPower * 0.05; // ~5% rectifier loss
    const totalLosses = inputPower - transferredPower + rectifierLosses;

    const outputPower = transferredPower - rectifierLosses;
    const efficiency = outputPower / inputPower;

    // Estimate output voltage and current (simplified)
    const outputVoltage = Math.sqrt(outputPower * receiver.resistance);
    const outputCurrent = outputPower / outputVoltage;

    return {
      couplingCoefficient: k,
      mutualInductance: M,
      transferredPower,
      efficiency,
      outputVoltage,
      outputCurrent,
      coilLosses,
      rectifierLosses,
      totalLosses,
    };
  }

  /**
   * Calculate charging efficiency
   */
  calculateChargingEfficiency(params: {
    inputPower: number;
    outputPower: number;
    distance: number;
    frequency: number;
  }): { percentage: number; losses: number; rating: string } {
    const efficiency = (params.outputPower / params.inputPower) * 100;
    const losses = params.inputPower - params.outputPower;

    let rating: string;
    if (efficiency >= 85) rating = 'excellent';
    else if (efficiency >= 75) rating = 'good';
    else if (efficiency >= 65) rating = 'fair';
    else rating = 'poor';

    return { percentage: efficiency, losses, rating };
  }

  // ==========================================================================
  // Coil Design
  // ==========================================================================

  /**
   * Design transmitter coil
   */
  designTransmitterCoil(params: {
    targetPower: number;
    frequency: number;
    diameter: number;
    turns: number;
  }): TransmitterCoil {
    const { targetPower, frequency, diameter, turns } = params;

    // Calculate inductance (simplified flat spiral coil formula)
    const radius = (diameter / 2) / 1000; // Convert to meters
    const innerRadius = radius * 0.5; // Assume 50% fill
    const width = radius - innerRadius;

    const inductance =
      (CONSTANTS.MU_0 * turns ** 2 * radius ** 2) / (8 * radius + 11 * width) * 1e6; // in μH

    // Estimate resistance (simplified)
    const wireLength = 2 * Math.PI * radius * turns;
    const wireGauge = 20; // AWG 20
    const resistancePerMeter = 0.033; // Ω/m for AWG 20
    const resistance = wireLength * resistancePerMeter;

    // Calculate quality factor
    const qualityFactor = (2 * Math.PI * frequency * inductance * 1e-6) / resistance;

    // Calculate max current
    const maxCurrent = Math.sqrt(targetPower / resistance);

    const coil: TransmitterCoil = {
      role: 'transmitter',
      outerDiameter: diameter,
      innerDiameter: diameter * 0.5,
      turns,
      wireGauge,
      wireType: 'litz',
      litzStrands: 100,
      coreMaterial: 'ferrite',
      ferriteThickness: 0.5,
      inductance,
      resistance,
      qualityFactor,
      frequency,
      maxCurrent,
      maxPower: targetPower,
    };

    return coil;
  }

  /**
   * Design receiver coil
   */
  designReceiverCoil(params: {
    targetPower: number;
    frequency: number;
    diameter: number;
    turns: number;
  }): ReceiverCoil {
    const { targetPower, frequency, diameter, turns } = params;

    // Similar calculation to transmitter
    const transmitter = this.designTransmitterCoil(params);

    const coil: ReceiverCoil = {
      role: 'receiver',
      outerDiameter: transmitter.outerDiameter,
      innerDiameter: transmitter.innerDiameter,
      turns,
      wireGauge: transmitter.wireGauge,
      wireType: transmitter.wireType,
      litzStrands: transmitter.litzStrands,
      coreMaterial: transmitter.coreMaterial,
      ferriteThickness: transmitter.ferriteThickness,
      inductance: transmitter.inductance,
      resistance: transmitter.resistance,
      qualityFactor: transmitter.qualityFactor,
      frequency,
      maxVoltage: Math.sqrt(targetPower * transmitter.resistance),
      rectificationType: 'synchronous',
    };

    return coil;
  }

  /**
   * Design Qi-compatible charger
   */
  designQiCharger(params: {
    powerLevel: number;
    frequency: number;
    coilDiameter: number;
    voltage: number;
  }) {
    const { powerLevel, frequency, coilDiameter, voltage } = params;

    // Determine Qi profile
    let profile: 'BPP' | 'EPP' | 'MP';
    if (powerLevel <= 5) profile = 'BPP';
    else if (powerLevel <= 15) profile = 'EPP';
    else profile = 'MP';

    // Calculate turns needed
    const turns = Math.ceil(20 * (powerLevel / 15)); // Scale turns with power

    // Design coil
    const coil = this.designTransmitterCoil({
      targetPower: powerLevel,
      frequency,
      diameter: coilDiameter,
      turns,
    });

    // Estimate performance
    const efficiency = 75 + (coil.qualityFactor / 100) * 10; // 75-85% typical
    const maxDistance = coilDiameter * 0.2; // ~20% of diameter

    return {
      profile,
      coilTurns: turns,
      coilInductance: coil.inductance,
      coilResistance: coil.resistance,
      qualityFactor: coil.qualityFactor,
      efficiency,
      maxDistance,
      operatingFrequency: frequency,
    };
  }

  // ==========================================================================
  // Foreign Object Detection
  // ==========================================================================

  /**
   * Detect foreign objects
   */
  detectForeignObject(
    config: FODConfig,
    currentQ: number,
    currentPower: { input: number; output: number }
  ): FODResult {
    const { baselineQ, qThreshold, powerLossThreshold, method } = config;

    let detected = false;
    let confidence = 0;
    const qChange = ((baselineQ - currentQ) / baselineQ) * 100;
    const powerLoss = currentPower.input - currentPower.output;
    const frequencyShift = 0; // Simplified - would measure actual frequency shift

    // Q-factor method
    if (method === 'q-factor' || method === 'multi-method') {
      if (Math.abs(qChange) > qThreshold) {
        detected = true;
        confidence = Math.min(Math.abs(qChange) / qThreshold, 1);
      }
    }

    // Power loss method
    if (method === 'power-loss' || method === 'multi-method') {
      const normalLoss = currentPower.input * 0.25; // Expected 25% loss
      const excessLoss = powerLoss - normalLoss;

      if (excessLoss > powerLossThreshold) {
        detected = true;
        confidence = Math.max(confidence, Math.min(excessLoss / powerLossThreshold, 1));
      }
    }

    // Determine action
    let action: FODResult['action'];
    if (!detected) {
      action = 'continue';
    } else if (confidence < 0.5) {
      action = 'reduce-power';
    } else if (confidence < 0.8) {
      action = 'alert-user';
    } else {
      action = 'stop-charging';
    }

    return {
      detected,
      confidence,
      measuredQ: currentQ,
      qChange,
      powerLoss,
      frequencyShift,
      action,
    };
  }

  // ==========================================================================
  // Alignment and Positioning
  // ==========================================================================

  /**
   * Optimize alignment
   */
  optimizeAlignment(params: {
    offsetX: number;
    offsetY: number;
    angularOffset?: number;
  }): AlignmentStatus {
    const { offsetX, offsetY, angularOffset = 0 } = params;

    const offsetMagnitude = Math.sqrt(offsetX ** 2 + offsetY ** 2);

    // Calculate alignment score (0-100)
    const maxAllowedOffset = 20; // mm
    const score = Math.max(0, 100 - (offsetMagnitude / maxAllowedOffset) * 100);

    // Determine quality
    let quality: AlignmentStatus['quality'];
    if (score >= 90) quality = 'excellent';
    else if (score >= 70) quality = 'good';
    else if (score >= 50) quality = 'fair';
    else quality = 'poor';

    // Generate recommendation
    let direction: 'up' | 'down' | 'left' | 'right' | undefined;
    if (Math.abs(offsetX) > Math.abs(offsetY)) {
      direction = offsetX > 0 ? 'left' : 'right';
    } else if (offsetMagnitude > 1) {
      direction = offsetY > 0 ? 'down' : 'up';
    }

    // Generate feedback
    const feedback = {
      type: 'multi-modal' as const,
      visual: {
        ledColor: (quality === 'excellent' || quality === 'good'
          ? 'green'
          : quality === 'fair'
          ? 'yellow'
          : 'red') as 'red' | 'yellow' | 'green',
        displayMessage: `Alignment: ${quality} (${score.toFixed(0)}%)`,
      },
      auditory: {
        beepFrequency: 1 + score / 25, // 1-5 Hz
        tonePitch: 200 + score * 8, // 200-1000 Hz
      },
      haptic: {
        vibrationIntensity: score,
        pulseRate: 1 + score / 20, // 1-6 Hz
      },
    };

    return {
      score,
      quality,
      lateralOffset: { x: offsetX, y: offsetY },
      angularOffset,
      recommendation: {
        direction,
        distance: offsetMagnitude,
      },
      feedback,
    };
  }

  // ==========================================================================
  // EMF Safety
  // ==========================================================================

  /**
   * Calculate EMF exposure
   */
  calculateEMF(params: {
    power: number;
    distance: number;
    frequency: number;
    coilRadius: number;
    coilTurns: number;
  }): EMFMeasurement {
    const { power, distance, frequency, coilRadius, coilTurns } = params;

    // Convert to SI units
    const d = distance / 1000; // mm to m
    const r = (coilRadius / 2) / 1000; // mm to m

    // Calculate current in coil
    const resistance = 1; // Assume 1Ω for calculation
    const current = Math.sqrt(power / resistance);

    // Magnetic field at distance d (on-axis)
    // B ≈ (μ₀ * N * I * r²) / [2 * (r² + d²)^(3/2)]
    const numerator = CONSTANTS.MU_0 * coilTurns * current * r ** 2;
    const denominator = 2 * (r ** 2 + d ** 2) ** 1.5;
    const magneticField = (numerator / denominator) * 1e6; // Convert to μT

    // Simplified electric field calculation
    const electricField = magneticField * 0.01; // Rough approximation

    // Check compliance with ICNIRP 2010 (for 110 kHz)
    const standard: EMFStandard = 'ICNIRP-2010';
    const magneticLimit = 6.25; // μT
    const compliant = magneticField <= magneticLimit;
    const safetyMargin = ((magneticLimit - magneticField) / magneticLimit) * 100;

    return {
      location: { x: 0, y: 0, z: distance },
      magneticField,
      electricField,
      distance,
      compliant,
      standard,
      safetyMargin: Math.max(0, safetyMargin),
    };
  }

  // ==========================================================================
  // Efficiency Optimization
  // ==========================================================================

  /**
   * Optimize charging efficiency
   */
  optimizeEfficiency(config: PowerTransferConfig): EfficiencyOptimizationResult {
    const originalResult = this.calculatePowerTransfer(config);
    const originalEfficiency = originalResult.efficiency;

    const methodsApplied: EfficiencyOptimizationResult['methodsApplied'] = [];
    let optimizedEfficiency = originalEfficiency;

    // 1. Coil alignment optimization
    if (config.lateralOffset) {
      const offsetMag = Math.sqrt(
        config.lateralOffset.x ** 2 + config.lateralOffset.y ** 2
      );
      if (offsetMag > 5) {
        // More than 5mm offset
        optimizedEfficiency += 0.05; // ~5% gain from better alignment
        methodsApplied.push('coil-alignment');
      }
    }

    // 2. Resonance tuning
    optimizedEfficiency += 0.03; // ~3% gain from tuning
    methodsApplied.push('resonance-tuning');

    // 3. High-Q coils (if Q < 100)
    if (config.transmitter.qualityFactor < 100) {
      optimizedEfficiency += 0.04; // ~4% gain from better coils
      methodsApplied.push('high-q-coils');
    }

    // 4. Active rectification
    optimizedEfficiency += 0.03; // ~3% gain from sync rectification
    methodsApplied.push('active-rectification');

    const recommendations: string[] = [];
    if (!methodsApplied.includes('coil-alignment')) {
      recommendations.push('Alignment is good, no adjustment needed');
    }
    if (config.transmitter.qualityFactor < 100) {
      recommendations.push('Consider using litz wire to improve Q factor');
    }
    recommendations.push('Implement active rectification for best efficiency');

    return {
      originalEfficiency,
      optimizedEfficiency: Math.min(optimizedEfficiency, 0.95), // Cap at 95%
      efficiencyGain: optimizedEfficiency - originalEfficiency,
      methodsApplied,
      recommendations,
    };
  }

  // ==========================================================================
  // Session Management
  // ==========================================================================

  /**
   * Start charging session
   */
  startSession(params: {
    standard: ChargingStandard;
    deviceInfo?: Partial<ChargingSession['device']>;
  }): ChargingSession {
    const session: ChargingSession = {
      id: this.generateSessionId(),
      startTime: new Date(),
      currentPhase: 'selection',
      standard: params.standard,
      device: params.deviceInfo || {},
      powerStats: {
        averagePower: 0,
        peakPower: 0,
        totalEnergy: 0,
        averageEfficiency: 0,
      },
      alignmentStats: {
        averageScore: 0,
        minScore: 100,
        maxScore: 0,
      },
      thermalStats: {
        maxTemperature: 0,
        averageTemperature: 0,
      },
      fodEvents: [],
      status: 'active',
    };

    this.activeSessions.set(session.id, session);
    this.emitEvent({ type: 'session-started', session });

    return session;
  }

  /**
   * End charging session
   */
  endSession(sessionId: string): ChargingSession | null {
    const session = this.activeSessions.get(sessionId);
    if (!session) return null;

    session.endTime = new Date();
    session.status = 'completed';
    session.currentPhase = 'complete';

    this.activeSessions.delete(sessionId);
    this.emitEvent({ type: 'session-ended', session });

    return session;
  }

  // ==========================================================================
  // Event Handling
  // ==========================================================================

  /**
   * Register event handler
   */
  on<T extends WirelessChargingEvent>(
    eventType: T['type'],
    handler: EventHandler<T>
  ): void {
    if (!this.eventHandlers.has(eventType)) {
      this.eventHandlers.set(eventType, []);
    }
    this.eventHandlers.get(eventType)!.push(handler as EventHandler);
  }

  /**
   * Unregister event handler
   */
  off<T extends WirelessChargingEvent>(
    eventType: T['type'],
    handler: EventHandler<T>
  ): void {
    const handlers = this.eventHandlers.get(eventType);
    if (handlers) {
      const index = handlers.indexOf(handler as EventHandler);
      if (index > -1) {
        handlers.splice(index, 1);
      }
    }
  }

  /**
   * Emit event
   */
  private emitEvent(event: WirelessChargingEvent): void {
    const handlers = this.eventHandlers.get(event.type);
    if (handlers) {
      handlers.forEach((handler) => handler(event));
    }
  }

  // ==========================================================================
  // Utility Methods
  // ==========================================================================

  /**
   * Generate unique session ID
   */
  private generateSessionId(): string {
    return `wia-comm-008-${Date.now()}-${Math.random().toString(36).substring(2, 9)}`;
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return '1.0.0';
  }

  /**
   * Get SDK configuration
   */
  getConfig(): WirelessChargingConfig {
    return { ...this.config };
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Calculate photon energy for wireless charging frequency
 */
export function calculatePhotonEnergy(frequency: number): {
  energy: number;
  wavelength: number;
} {
  const h = 6.62607015e-34; // Planck's constant (J·s)
  const energy = h * frequency;
  const wavelength = CONSTANTS.SPEED_OF_LIGHT / frequency;

  return { energy, wavelength };
}

/**
 * Convert power units
 */
export function convertPower(value: number, from: 'W' | 'mW' | 'kW', to: 'W' | 'mW' | 'kW'): number {
  const toWatts = { W: 1, mW: 0.001, kW: 1000 };
  const watts = value * toWatts[from];
  return watts / toWatts[to];
}

/**
 * Convert frequency units
 */
export function convertFrequency(
  value: number,
  from: 'Hz' | 'kHz' | 'MHz',
  to: 'Hz' | 'kHz' | 'MHz'
): number {
  const toHz = { Hz: 1, kHz: 1000, MHz: 1000000 };
  const hz = value * toHz[from];
  return hz / toHz[to];
}

// ============================================================================
// Exports
// ============================================================================

export * from './types';
export { CONSTANTS };

// Default export
export default WirelessChargingSDK;

// 弘益人間 (홍익인간) · Benefit All Humanity
