/**
 * WIA-COMM-009: Wireless Power Transfer SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Communication Standards Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides computational tools for wireless power transfer including:
 * - Inductive power transfer (IPT) for near-field charging
 * - Capacitive power transfer (CPT) through electric fields
 * - Resonant wireless power for mid-range applications
 * - Microwave power beaming for long-range transmission
 * - Laser power beaming for space and terrestrial applications
 * - EV dynamic charging systems
 * - Space solar power calculations
 */

import {
  WPTTechnology,
  InductiveWPTConfig,
  InductivePowerTransfer,
  CapacitiveWPTConfig,
  CapacitivePowerTransfer,
  ResonantWPTConfig,
  ResonantPowerTransfer,
  MicrowaveWPTConfig,
  MicrowaveBeamTransmission,
  LaserPowerBeamingConfig,
  LaserPowerTransmission,
  EVDynamicChargingConfig,
  EVDynamicChargingSession,
  SpaceSolarPowerConfig,
  SpaceSolarPowerTransmission,
  EfficiencyAnalysis,
  ForeignObjectDetection,
  Position3D,
  GeoLocation,
  WPT_CONSTANTS,
  ISM_BANDS,
  WirelessPowerErrorCode,
  WirelessPowerError,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-COMM-009 Wireless Power Transfer SDK
 */
export class WirelessPowerSDK {
  private version = '1.0.0';

  constructor() {}

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Create inductive WPT system
   */
  createInductiveWPT(config: InductiveWPTConfig): InductiveWPT {
    return new InductiveWPT(config);
  }

  /**
   * Create capacitive WPT system
   */
  createCapacitiveWPT(config: CapacitiveWPTConfig): CapacitiveWPT {
    return new CapacitiveWPT(config);
  }

  /**
   * Create resonant WPT system
   */
  createResonantWPT(config: ResonantWPTConfig): ResonantWPT {
    return new ResonantWPT(config);
  }

  /**
   * Create microwave WPT system
   */
  createMicrowaveWPT(config: MicrowaveWPTConfig): MicrowaveWPT {
    return new MicrowaveWPT(config);
  }

  /**
   * Create laser power beaming system
   */
  createLaserPowerBeaming(config: LaserPowerBeamingConfig): LaserPowerBeaming {
    return new LaserPowerBeaming(config);
  }

  /**
   * Create EV dynamic charging system
   */
  createEVDynamicCharging(config: EVDynamicChargingConfig): EVDynamicCharging {
    return new EVDynamicCharging(config);
  }

  /**
   * Create space solar power system
   */
  createSpaceSolarPower(config: SpaceSolarPowerConfig): SpaceSolarPower {
    return new SpaceSolarPower(config);
  }
}

// ============================================================================
// Inductive Power Transfer (IPT)
// ============================================================================

/**
 * Inductive WPT implementation (Qi, AirFuel, SAE J2954)
 */
export class InductiveWPT {
  private config: InductiveWPTConfig;
  private activeSessions: Map<string, InductivePowerTransfer> = new Map();

  constructor(config: InductiveWPTConfig) {
    this.config = config;
  }

  /**
   * Start power transfer session
   */
  async startTransfer(params: {
    deviceId: string;
    requestedPower: number;
    alignment: Position3D;
  }): Promise<InductivePowerTransfer> {
    const { deviceId, requestedPower, alignment } = params;

    // Check foreign objects
    const fod = await this.detectForeignObjects();
    if (fod.detected && fod.objectType === 'metallic') {
      throw new WirelessPowerError(
        WirelessPowerErrorCode.FOREIGN_OBJECT_DETECTED,
        'Foreign metallic object detected on charging surface'
      );
    }

    // Calculate coupling coefficient
    const coupling = this.calculateCoupling(alignment.z);

    // Check if coupling is sufficient
    if (coupling < 0.2) {
      throw new WirelessPowerError(
        WirelessPowerErrorCode.LOW_COUPLING,
        `Coupling coefficient ${coupling.toFixed(3)} too low (min 0.2)`,
        { coupling, distance: alignment.z }
      );
    }

    // Calculate efficiency
    const q1 = this.config.transmitterCoil.quality;
    const q2 = this.config.receiverCoil?.quality || 150;
    const efficiency = coupling * coupling * q1 * q2 / (1 + coupling * coupling * q1 * q2);

    // Power delivered
    const powerTransmitted = Math.min(requestedPower / efficiency, this.config.maxPower);
    const powerDelivered = powerTransmitted * efficiency;

    // Alignment quality
    const lateralOffset = Math.sqrt(alignment.x ** 2 + alignment.y ** 2);
    let alignmentQuality: 'excellent' | 'good' | 'fair' | 'poor';
    if (lateralOffset < 0.005 && coupling > 0.6) alignmentQuality = 'excellent';
    else if (lateralOffset < 0.01 && coupling > 0.4) alignmentQuality = 'good';
    else if (lateralOffset < 0.02 && coupling > 0.3) alignmentQuality = 'fair';
    else alignmentQuality = 'poor';

    const session: InductivePowerTransfer = {
      sessionId: `IPT-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
      timestamp: new Date(),
      deviceId,
      powerTransmitted,
      powerDelivered,
      efficiency,
      coupling,
      distance: alignment.z,
      alignment,
      alignmentQuality,
      temperature: 25, // Initial temperature
      foreignObjects: false,
      livingObjectDetected: false,
      status: 'charging',
    };

    this.activeSessions.set(session.sessionId, session);
    return session;
  }

  /**
   * Stop power transfer
   */
  async stopTransfer(sessionId: string): Promise<void> {
    const session = this.activeSessions.get(sessionId);
    if (session) {
      session.status = 'complete';
      this.activeSessions.delete(sessionId);
    }
  }

  /**
   * Detect foreign objects using Q-factor method
   */
  async detectForeignObjects(): Promise<ForeignObjectDetection> {
    if (!this.config.foreignObjectDetection) {
      return {
        detected: false,
        method: 'Q-factor',
        confidence: 0,
        action: 'continue',
      };
    }

    // Simulate Q-factor measurement
    const qMeasured = this.config.transmitterCoil.quality * (0.95 + Math.random() * 0.1);
    const qExpected = this.config.transmitterCoil.quality;
    const deltaQ = Math.abs(qMeasured - qExpected) / qExpected;

    const detected = deltaQ > 0.05; // 5% threshold

    return {
      detected,
      method: 'Q-factor',
      objectType: detected ? 'metallic' : undefined,
      confidence: detected ? 0.8 : 0.2,
      action: detected ? 'stop' : 'continue',
    };
  }

  /**
   * Calculate coupling coefficient from distance
   */
  private calculateCoupling(distance: number): number {
    // Simplified model: k = exp(-distance / characteristic_distance)
    const r1 = this.config.transmitterCoil.diameter / 2;
    const r2 = this.config.receiverCoil?.diameter || r1 * 0.8;
    const characteristicDistance = Math.sqrt(r1 * r2);

    return Math.exp(-distance / characteristicDistance) * 0.9;
  }

  /**
   * Get configuration
   */
  getConfig(): InductiveWPTConfig {
    return { ...this.config };
  }
}

// ============================================================================
// Capacitive Power Transfer (CPT)
// ============================================================================

/**
 * Capacitive WPT implementation
 */
export class CapacitiveWPT {
  private config: CapacitiveWPTConfig;

  constructor(config: CapacitiveWPTConfig) {
    this.config = config;
  }

  /**
   * Calculate capacitance between plates
   */
  calculateCapacitance(distance: number): number {
    const area =
      this.config.plateDimensions.length * this.config.plateDimensions.width;
    const epsilon0 = WPT_CONSTANTS.EPSILON_0;
    const epsilonR = this.config.dielectric.permittivity;

    // C = ε₀ × εᵣ × A / d
    return (epsilon0 * epsilonR * area) / distance;
  }

  /**
   * Start power transfer
   */
  async startTransfer(params: { distance: number }): Promise<CapacitivePowerTransfer> {
    const capacitance = this.calculateCapacitance(params.distance);

    // Calculate impedance: Z = 1 / (2πfC)
    const impedance = 1 / (2 * Math.PI * this.config.frequency * capacitance);

    // Power transferred (simplified)
    const voltage = this.config.voltage;
    const current = voltage / impedance;
    const powerTransmitted = voltage * current;

    // Efficiency (simplified, decreases with distance)
    const efficiency = Math.exp(-params.distance / 0.01) * 0.85; // 85% max

    const powerDelivered = powerTransmitted * efficiency;

    // Electric field strength
    const electricField = voltage / params.distance;

    // Check safety
    if (electricField > this.config.maxElectricField) {
      throw new WirelessPowerError(
        WirelessPowerErrorCode.SAR_VIOLATION,
        `Electric field ${electricField.toFixed(0)} V/m exceeds limit`
      );
    }

    return {
      sessionId: `CPT-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
      timestamp: new Date(),
      powerTransmitted,
      powerDelivered,
      efficiency,
      capacitance,
      electricField,
      distance: params.distance,
      status: 'active',
    };
  }

  getConfig(): CapacitiveWPTConfig {
    return { ...this.config };
  }
}

// ============================================================================
// Resonant Wireless Power
// ============================================================================

/**
 * Resonant WPT implementation (WiTricity-style)
 */
export class ResonantWPT {
  private config: ResonantWPTConfig;

  constructor(config: ResonantWPTConfig) {
    this.config = config;
  }

  /**
   * Calculate transfer parameters
   */
  async calculateTransfer(params: {
    distance: number;
    alignment: Position3D;
    requestedPower: number;
  }): Promise<ResonantPowerTransfer> {
    const { distance, alignment, requestedPower } = params;

    // Calculate mutual inductance
    const mutualInductance = this.calculateMutualInductance(distance, alignment);

    // Calculate coupling coefficient
    const L1 = this.config.transmitterCoil.inductance;
    const L2 = this.config.receiverCoil.inductance;
    const coupling = mutualInductance / Math.sqrt(L1 * L2);

    // Calculate efficiency
    const Q1 = this.config.transmitterCoil.quality;
    const Q2 = this.config.receiverCoil.quality;

    // Resonant coupling efficiency
    const efficiency = (coupling ** 2 * Q1 * Q2) / ((1 + coupling ** 2 * Q1 * Q2) ** 2 / 4);

    // Power calculation
    const powerTransmitted = Math.min(requestedPower / efficiency, this.config.maxPower);
    const powerDelivered = powerTransmitted * efficiency;
    const heatDissipation = powerTransmitted - powerDelivered;

    return {
      sessionId: `RES-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
      timestamp: new Date(),
      powerTransmitted,
      powerDelivered,
      efficiency,
      powerFactor: 0.95,
      coupling,
      mutualInductance,
      distance,
      alignment,
      resonantFrequency: this.config.frequency,
      frequencyOffset: 0,
      qFactor: {
        transmitter: Q1,
        receiver: Q2,
      },
      temperature: 25 + heatDissipation / 100,
      heatDissipation,
      status: 'active',
    };
  }

  /**
   * Calculate mutual inductance
   */
  private calculateMutualInductance(distance: number, alignment: Position3D): number {
    const r1 = this.config.transmitterCoil.diameter / 2;
    const r2 = this.config.receiverCoil.diameter / 2;

    // Neumann formula approximation for coaxial coils
    // M ≈ μ₀ × π × r1² × r2² × N1 × N2 / (2 × (r1² + d²)^(3/2))
    const mu0 = WPT_CONSTANTS.MU_0;
    const N1 = this.config.transmitterCoil.turns;
    const N2 = this.config.receiverCoil.turns;

    const lateralOffset = Math.sqrt(alignment.x ** 2 + alignment.y ** 2);
    const effectiveDistance = Math.sqrt(distance ** 2 + lateralOffset ** 2);

    const M =
      (mu0 * Math.PI * r1 ** 2 * r2 ** 2 * N1 * N2) /
      (2 * (r1 ** 2 + effectiveDistance ** 2) ** 1.5);

    return M;
  }

  getConfig(): ResonantWPTConfig {
    return { ...this.config };
  }
}

// ============================================================================
// Microwave Power Transfer
// ============================================================================

/**
 * Microwave WPT implementation (rectenna systems)
 */
export class MicrowaveWPT {
  private config: MicrowaveWPTConfig;

  constructor(config: MicrowaveWPTConfig) {
    this.config = config;
  }

  /**
   * Transmit microwave power beam
   */
  async transmit(params: {
    targetLocation?: GeoLocation;
    distance: number;
    powerRequired: number;
  }): Promise<MicrowaveBeamTransmission> {
    const { distance, powerRequired } = params;

    // Calculate wavelength
    const wavelength = WPT_CONSTANTS.SPEED_OF_LIGHT / this.config.frequency;

    // Friis transmission equation
    const Gt = Math.pow(10, this.config.transmitAntenna.gain / 10); // Linear gain
    const Gr = Math.pow(10, this.config.receiverAntenna.gain / 10);

    // Path loss: (λ / 4πd)²
    const pathLossFactor = (wavelength / (4 * Math.PI * distance)) ** 2;

    // Received power: Pr = Pt × Gt × Gr × (λ/4πd)²
    const receivedPowerRF = this.config.transmitPower * Gt * Gr * pathLossFactor;

    // Rectenna conversion
    const receivedPower = receivedPowerRF * this.config.rectennaEfficiency;

    // Atmospheric loss (simplified, 0.1 dB/km at 2.45 GHz)
    const atmosphericLossDb = 0.1 * (distance / 1000);

    // Path loss in dB
    const pathLossDb = -10 * Math.log10(pathLossFactor);

    // Total loss
    const totalLossDb = pathLossDb + atmosphericLossDb;

    // Efficiency
    const efficiency = receivedPower / this.config.transmitPower;

    // Power density at receiver
    const beamArea = Math.PI * (distance * Math.tan((this.config.transmitAntenna.beamwidth * Math.PI) / 180 / 2)) ** 2;
    const powerDensityPeak = this.config.transmitPower * Gt / (4 * Math.PI * distance ** 2);
    const powerDensityAvg = this.config.transmitPower / beamArea;

    // Check safety
    if (powerDensityPeak > this.config.maxPowerDensity) {
      throw new WirelessPowerError(
        WirelessPowerErrorCode.SAR_VIOLATION,
        `Power density ${powerDensityPeak.toFixed(2)} W/m² exceeds limit`
      );
    }

    // Rectenna DC output
    const rectennaVoltage = Math.sqrt(receivedPower * 50); // Assuming 50 Ω load
    const rectennaCurrent = receivedPower / rectennaVoltage;

    return {
      transmissionId: `MW-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
      timestamp: new Date(),
      transmitPower: this.config.transmitPower,
      receivedPower,
      efficiency,
      frequency: this.config.frequency,
      beamDirection: { azimuth: 0, elevation: 30 },
      beamwidth: this.config.transmitAntenna.beamwidth,
      powerDensity: {
        peak: powerDensityPeak,
        average: powerDensityAvg,
      },
      distance,
      pathLoss: pathLossDb,
      atmosphericLoss: atmosphericLossDb,
      totalLoss: totalLossDb,
      rectennaVoltage,
      rectennaCurrent,
      status: 'transmitting',
    };
  }

  getConfig(): MicrowaveWPTConfig {
    return { ...this.config };
  }
}

// ============================================================================
// Laser Power Beaming
// ============================================================================

/**
 * Laser power beaming implementation
 */
export class LaserPowerBeaming {
  private config: LaserPowerBeamingConfig;

  constructor(config: LaserPowerBeamingConfig) {
    this.config = config;
  }

  /**
   * Transmit laser power
   */
  async transmit(params: {
    distance: number;
    atmosphericTransmission?: number;
  }): Promise<LaserPowerTransmission> {
    const { distance, atmosphericTransmission = 0.8 } = params;

    // Calculate spot size at target
    const spotSize = this.config.beamDiameter + 2 * distance * this.config.divergence;

    // Optical power at target
    const opticalPower = this.config.laserPower * atmosphericTransmission;

    // Irradiance at target
    const spotArea = Math.PI * (spotSize / 2) ** 2;
    const irradiance = opticalPower / spotArea;

    // Check safety
    if (irradiance > this.config.maxIrradiance) {
      throw new WirelessPowerError(
        WirelessPowerErrorCode.SAR_VIOLATION,
        `Irradiance ${irradiance.toFixed(0)} W/m² exceeds limit`
      );
    }

    // PV cell receives power
    const pvReceivedPower = Math.min(irradiance * this.config.pvArea, opticalPower);

    // PV conversion to electrical
    const deliveredPower = pvReceivedPower * this.config.pvEfficiency;

    // Efficiencies
    const opticalEfficiency = opticalPower / this.config.laserPower;
    const conversionEfficiency = deliveredPower / pvReceivedPower;
    const totalEfficiency = deliveredPower / this.config.laserPower;

    // PV electrical output
    const pvVoltage = this.config.pvVoltage;
    const pvCurrent = deliveredPower / pvVoltage;
    const pvTemperature = 25 + (pvReceivedPower - deliveredPower) / 10; // Simplified heating

    return {
      transmissionId: `LAS-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
      timestamp: new Date(),
      laserPower: this.config.laserPower,
      deliveredPower,
      efficiency: {
        optical: opticalEfficiency,
        conversion: conversionEfficiency,
        total: totalEfficiency,
      },
      wavelength: this.config.wavelength,
      beamDiameter: this.config.beamDiameter,
      spotSize,
      irradiance,
      distance,
      atmosphericTransmission,
      alignment: { lateral: 0, angular: 0 },
      pvVoltage,
      pvCurrent,
      pvTemperature,
      status: 'active',
    };
  }

  getConfig(): LaserPowerBeamingConfig {
    return { ...this.config };
  }
}

// ============================================================================
// EV Dynamic Charging
// ============================================================================

/**
 * EV dynamic (in-road) wireless charging
 */
export class EVDynamicCharging {
  private config: EVDynamicChargingConfig;

  constructor(config: EVDynamicChargingConfig) {
    this.config = config;
  }

  /**
   * Simulate dynamic charging session
   */
  async simulateCharging(params: {
    vehicleId: string;
    speed: number; // m/s
    batteryLevel: number; // %
    duration: number; // seconds
  }): Promise<EVDynamicChargingSession> {
    const { vehicleId, speed, batteryLevel, duration } = params;

    // Calculate how many segments the vehicle passes through
    const distanceTraveled = speed * duration;
    const segmentsPassed = Math.floor(distanceTraveled / this.config.coilSpacing);

    // Coupling coefficient (assuming good alignment)
    const coupling = 0.25; // Typical for 20 cm air gap

    // Efficiency calculation
    const Q1 = this.config.roadCoil.quality;
    const Q2 = this.config.vehicleCoil.quality;
    const efficiency = (coupling ** 2 * Q1 * Q2) / (1 + coupling ** 2 * Q1 * Q2);

    // Power received (accounting for duty cycle as vehicle moves)
    const dutyCycle = this.config.segmentLength / this.config.coilSpacing;
    const powerReceived = this.config.maxPower * efficiency * dutyCycle;

    // Energy transferred
    const energyTransferred = powerReceived * duration; // Joules

    return {
      sessionId: `EV-DYN-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
      vehicleId,
      timestamp: new Date(),
      speed,
      position: { latitude: 37.7749, longitude: -122.4194 },
      batteryLevel,
      powerReceived,
      energyTransferred,
      efficiency,
      activeSegments: Array.from({ length: this.config.segments }, (_, i) => i),
      coupling,
      alignment: { lateral: 0.05, longitudinal: 0 },
      durationSeconds: duration,
      status: 'charging',
    };
  }

  getConfig(): EVDynamicChargingConfig {
    return { ...this.config };
  }
}

// ============================================================================
// Space Solar Power
// ============================================================================

/**
 * Space-based solar power satellite system
 */
export class SpaceSolarPower {
  private config: SpaceSolarPowerConfig;

  constructor(config: SpaceSolarPowerConfig) {
    this.config = config;
  }

  /**
   * Calculate power transmission from space
   */
  async calculateTransmission(params: {
    sunAngle: number; // degrees
    atmosphericConditions?: 'clear' | 'cloudy' | 'rain' | 'storm';
  }): Promise<SpaceSolarPowerTransmission> {
    const { sunAngle, atmosphericConditions = 'clear' } = params;

    // Solar power collected
    const solarConstant = 1361; // W/m² in space
    const solarPowerCollected =
      solarConstant *
      this.config.solarPanelArea *
      this.config.solarPanelEfficiency *
      Math.cos((sunAngle * Math.PI) / 180);

    // Power available for transmission
    const transmittedPower = Math.min(
      solarPowerCollected * 0.9, // 90% conversion to RF/laser
      this.config.transmitPower
    );

    // Atmospheric conditions affect transmission
    let atmosphericLossDb: number;
    switch (atmosphericConditions) {
      case 'clear':
        atmosphericLossDb = 0.5;
        break;
      case 'cloudy':
        atmosphericLossDb = 2.0;
        break;
      case 'rain':
        atmosphericLossDb = 5.0;
        break;
      case 'storm':
        atmosphericLossDb = 10.0;
        break;
    }

    const atmosphericLossFactor = Math.pow(10, -atmosphericLossDb / 10);

    // Link efficiency (Friis for microwave, or geometric for laser)
    let linkEfficiency: number;
    if (this.config.transmissionTechnology === 'microwave') {
      const wavelength = WPT_CONSTANTS.SPEED_OF_LIGHT / this.config.frequency!;
      const pathLoss =
        (wavelength / (4 * Math.PI * this.config.slantRange)) ** 2;
      linkEfficiency = pathLoss * atmosphericLossFactor;
    } else {
      // Laser: geometric spreading + atmospheric loss
      const beamArea =
        Math.PI * (this.config.slantRange * 0.001) ** 2; // Simplified
      linkEfficiency = (this.config.receiverAperture / beamArea) * atmosphericLossFactor;
    }

    // Received RF/optical power at ground
    const receivedRFPower = transmittedPower * linkEfficiency;

    // Grid power (after rectenna/PV conversion)
    const gridPower = receivedRFPower * this.config.receiverEfficiency;

    // Total efficiency
    const solarToRF = transmittedPower / solarPowerCollected;
    const rfToDC = gridPower / receivedRFPower;
    const totalEfficiency = gridPower / solarPowerCollected;

    return {
      transmissionId: `SSP-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
      timestamp: new Date(),
      solarPowerCollected,
      transmittedPower,
      receivedPower: receivedRFPower,
      gridPower,
      efficiency: {
        solarToRF,
        linkEfficiency,
        rfToDC,
        total: totalEfficiency,
      },
      sunAngle,
      distance: this.config.slantRange,
      atmosphericConditions,
      linkMargin: 10 * Math.log10(receivedRFPower / (gridPower / this.config.receiverEfficiency)), // dB
      status: 'transmitting',
    };
  }

  getConfig(): SpaceSolarPowerConfig {
    return { ...this.config };
  }
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Calculate WPT efficiency for any technology
 */
export function calculateEfficiency(params: {
  technology: WPTTechnology;
  distance: number;
  frequency: number;
  power: number;
  coupling?: number;
  qFactor?: number;
}): EfficiencyAnalysis {
  const { technology, distance, frequency, power, coupling = 0.5, qFactor = 100 } = params;

  let totalEfficiency: number;
  let conductionLoss = 0;
  let radiationLoss = 0;
  let dielectricLoss = 0;
  let mismatchLoss = 0;

  switch (technology) {
    case 'inductive':
      // Near-field, high efficiency at short range
      totalEfficiency = coupling ** 2 * qFactor / (1 + coupling ** 2 * qFactor);
      conductionLoss = power * 0.05;
      radiationLoss = power * 0.02;
      break;

    case 'resonant':
      // Mid-range, efficiency depends on Q and coupling
      totalEfficiency = (coupling ** 2 * qFactor ** 2) / ((1 + coupling ** 2 * qFactor ** 2) ** 2 / 4);
      conductionLoss = power * 0.08;
      radiationLoss = power * 0.05;
      break;

    case 'microwave':
      // Far-field, path loss dominant
      const wavelength = WPT_CONSTANTS.SPEED_OF_LIGHT / frequency;
      const pathLoss = (wavelength / (4 * Math.PI * distance)) ** 2;
      totalEfficiency = pathLoss * 0.7; // 70% rectenna efficiency
      radiationLoss = power * (1 - pathLoss);
      break;

    case 'laser':
      // Optical, atmospheric loss + PV efficiency
      const atmosphericTransmission = Math.exp(-distance / 10000); // Simplified
      totalEfficiency = atmosphericTransmission * 0.4; // 40% PV efficiency
      radiationLoss = power * (1 - atmosphericTransmission);
      break;

    default:
      totalEfficiency = 0.5;
  }

  const totalLoss = conductionLoss + radiationLoss + dielectricLoss + mismatchLoss;

  return {
    technology,
    distance,
    frequency,
    power,
    inverterEfficiency: 0.95,
    coilEfficiency: technology === 'inductive' || technology === 'resonant' ? 0.98 : undefined,
    couplingEfficiency: coupling,
    rectifierEfficiency: technology === 'microwave' || technology === 'laser' ? 0.85 : undefined,
    totalEfficiency,
    losses: {
      conduction: conductionLoss,
      radiation: radiationLoss,
      dielectric: dielectricLoss,
      mismatch: mismatchLoss,
      total: totalLoss,
    },
    heatDissipation: totalLoss,
    coolingRequired: totalLoss > 100, // > 100W needs cooling
  };
}

// ============================================================================
// Export Everything
// ============================================================================

export {
  WirelessPowerSDK,
  InductiveWPT,
  CapacitiveWPT,
  ResonantWPT,
  MicrowaveWPT,
  LaserPowerBeaming,
  EVDynamicCharging,
  SpaceSolarPower,
  calculateEfficiency,
};

// Re-export types
export * from './types';

/**
 * 弘익人間 (홍익인간) · Benefit All Humanity
 */
