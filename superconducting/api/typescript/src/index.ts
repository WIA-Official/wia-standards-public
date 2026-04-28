/**
 * WIA-QUA-007: Superconducting - TypeScript SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Quantum Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

import {
  SuperconductingMaterial,
  SuperconductingState,
  BCSParameters,
  JosephsonJunction,
  SuperconductingQubit,
  QubitMeasurement,
  GateOperation,
  SQUIDMagnetometer,
  SQUIDMeasurement,
  MaglevController,
  MaglevStatus,
  CryogenicSystem,
  Vector3,
  ComplexNumber,
  SUPERCONDUCTING_CONSTANTS,
  Result,
  ProgressCallback,
  SuperconductorType,
  SuperconductorFamily,
  PairingSymmetry,
  QubitType,
  QuantumGate,
  SQUIDType,
  MaglevType,
} from './types';

// Export all types
export * from './types';

// ============================================================================
// Superconducting Material Class
// ============================================================================

/**
 * Superconducting material implementation
 */
export class SuperconductingMaterialImpl {
  constructor(private material: SuperconductingMaterial) {}

  /**
   * Get current state at given conditions
   */
  getState(conditions: {
    temperature: number;
    magneticField: number;
    currentDensity: number;
  }): SuperconductingState {
    const { temperature, magneticField, currentDensity } = conditions;

    // Determine if superconducting
    const isBelowTc = temperature < this.material.criticalTemperature;
    const bc = this.getCriticalField(temperature);
    const isBelowBc = magneticField < bc;
    const jc = this.getCriticalCurrent(temperature, magneticField);
    const isBelowJc = currentDensity < jc;

    let phase: 'normal' | 'superconducting' | 'mixed-state' | 'transition';
    let resistance: number;

    if (!isBelowTc || !isBelowBc || !isBelowJc) {
      phase = 'normal';
      resistance = 1e-6; // Normal state resistance
    } else if (this.material.type === 'type-II') {
      const bc1 = this.getBc1(temperature);
      const bc2 = this.getBc2(temperature);
      if (magneticField > bc1 && magneticField < bc2) {
        phase = 'mixed-state';
        resistance = 1e-12; // Very small but non-zero
      } else {
        phase = 'superconducting';
        resistance = 0;
      }
    } else {
      phase = 'superconducting';
      resistance = 0;
    }

    const energyGap = this.calculateEnergyGap(temperature);

    return {
      phase,
      temperature,
      magneticField,
      currentDensity,
      resistance,
      criticalCurrent: jc,
      energyGap,
      margin: {
        temperature: ((this.material.criticalTemperature - temperature) / this.material.criticalTemperature) * 100,
        field: ((bc - magneticField) / bc) * 100,
        current: ((jc - currentDensity) / jc) * 100,
      },
    };
  }

  /**
   * Calculate energy gap at temperature
   */
  calculateEnergyGap(temperature: number): number {
    if (temperature >= this.material.criticalTemperature) {
      return 0;
    }

    const tc = this.material.criticalTemperature;
    const delta0 = SUPERCONDUCTING_CONSTANTS.BCS_GAP_RATIO * SUPERCONDUCTING_CONSTANTS.KB_EV * tc;

    if (temperature === 0) {
      return delta0;
    }

    const ratio = tc / temperature - 1;
    const delta = delta0 * Math.tanh(1.74 * Math.sqrt(ratio));

    return delta;
  }

  /**
   * Calculate critical field at temperature
   */
  getCriticalField(temperature: number): number {
    if (temperature >= this.material.criticalTemperature) {
      return 0;
    }

    const tRatio = temperature / this.material.criticalTemperature;
    const bc0 = typeof this.material.criticalField === 'number'
      ? this.material.criticalField
      : this.material.criticalField.bcThermodynamic || this.material.criticalField.bc2 || 0;

    return bc0 * (1 - tRatio * tRatio);
  }

  /**
   * Get lower critical field (Type II)
   */
  private getBc1(temperature: number): number {
    if (this.material.type !== 'type-II' && this.material.type !== 'type-II-HTSC') {
      return 0;
    }

    if (typeof this.material.criticalField === 'object' && this.material.criticalField.bc1) {
      const tRatio = temperature / this.material.criticalTemperature;
      return this.material.criticalField.bc1 * (1 - tRatio * tRatio);
    }

    return 0;
  }

  /**
   * Get upper critical field (Type II)
   */
  private getBc2(temperature: number): number {
    if (this.material.type !== 'type-II' && this.material.type !== 'type-II-HTSC') {
      return this.getCriticalField(temperature);
    }

    if (typeof this.material.criticalField === 'object' && this.material.criticalField.bc2) {
      const tRatio = temperature / this.material.criticalTemperature;
      return this.material.criticalField.bc2 * (1 - tRatio * tRatio);
    }

    return typeof this.material.criticalField === 'number'
      ? this.material.criticalField
      : 0;
  }

  /**
   * Calculate critical current density
   */
  getCriticalCurrent(temperature: number, magneticField: number): number {
    if (temperature >= this.material.criticalTemperature) {
      return 0;
    }

    // Simplified model: Jc decreases with both T and B
    const tFactor = 1 - Math.pow(temperature / this.material.criticalTemperature, 2);
    const bc = this.getCriticalField(temperature);
    const bFactor = bc > 0 ? (1 - magneticField / bc) : 0;

    return this.material.criticalCurrent * tFactor * Math.max(0, bFactor);
  }

  /**
   * Calculate coherence length
   */
  calculateCoherenceLength(temperature: number): number {
    if (this.material.coherenceLength) {
      if (temperature === 0) {
        return this.material.coherenceLength;
      }

      const delta0 = this.calculateEnergyGap(0);
      const deltaT = this.calculateEnergyGap(temperature);

      if (deltaT === 0) {
        return 0;
      }

      return this.material.coherenceLength * Math.sqrt(delta0 / deltaT);
    }

    // Estimate from BCS theory if not provided
    if (this.material.fermiVelocity) {
      const delta = this.calculateEnergyGap(temperature);
      if (delta === 0) return 0;

      const deltaJoules = delta * SUPERCONDUCTING_CONSTANTS.ELEMENTARY_CHARGE / 1000; // meV to J
      return (SUPERCONDUCTING_CONSTANTS.HBAR * this.material.fermiVelocity) / (Math.PI * deltaJoules) * 1e9; // to nm
    }

    return 0;
  }

  /**
   * Calculate penetration depth
   */
  calculatePenetrationDepth(temperature: number): number {
    if (temperature >= this.material.criticalTemperature) {
      return Infinity;
    }

    if (this.material.penetrationDepth) {
      const lambda0 = this.material.penetrationDepth;
      const tRatio = temperature / this.material.criticalTemperature;
      return lambda0 / Math.sqrt(1 - Math.pow(tRatio, 4));
    }

    return 0;
  }
}

// ============================================================================
// BCS Theory Implementation
// ============================================================================

/**
 * BCS theory calculator
 */
export class BCSTheory {
  constructor(
    private config: {
      material: SuperconductingMaterial;
      temperature: number;
    }
  ) {}

  /**
   * Calculate all BCS parameters
   */
  calculateParameters(): BCSParameters {
    const material = new SuperconductingMaterialImpl(this.config.material);

    return {
      material: this.config.material,
      temperature: this.config.temperature,
      energyGap: material.calculateEnergyGap(this.config.temperature),
      coherenceLength: material.calculateCoherenceLength(this.config.temperature),
      penetrationDepth: material.calculatePenetrationDepth(this.config.temperature),
      criticalField: material.getCriticalField(this.config.temperature),
    };
  }

  /**
   * Calculate energy gap
   */
  calculateEnergyGap(): number {
    const material = new SuperconductingMaterialImpl(this.config.material);
    return material.calculateEnergyGap(this.config.temperature);
  }

  /**
   * Calculate coherence length
   */
  calculateCoherenceLength(): number {
    const material = new SuperconductingMaterialImpl(this.config.material);
    return material.calculateCoherenceLength(this.config.temperature);
  }

  /**
   * Calculate penetration depth
   */
  calculatePenetrationDepth(): number {
    const material = new SuperconductingMaterialImpl(this.config.material);
    return material.calculatePenetrationDepth(this.config.temperature);
  }
}

// ============================================================================
// Josephson Junction Implementation
// ============================================================================

/**
 * Josephson junction simulator
 */
export class JosephsonJunctionSimulator {
  constructor(private junction: JosephsonJunction) {}

  /**
   * Calculate DC Josephson current
   */
  calculateDCCurrent(phaseDifference: number): number {
    return this.junction.criticalCurrent * Math.sin(phaseDifference);
  }

  /**
   * Calculate AC Josephson frequency
   */
  calculateACFrequency(voltage: number): number {
    // f = 2eV/h
    return SUPERCONDUCTING_CONSTANTS.JOSEPHSON_CONSTANT * voltage;
  }

  /**
   * Calculate IcRn product
   */
  calculateIcRn(): number {
    return this.junction.criticalCurrent * this.junction.normalResistance;
  }

  /**
   * Calculate Josephson energy
   */
  calculateJosephsonEnergy(): number {
    // EJ = (Φ₀ / 2π) × Ic
    return (SUPERCONDUCTING_CONSTANTS.FLUX_QUANTUM / (2 * Math.PI)) * this.junction.criticalCurrent;
  }

  /**
   * Calculate charging energy
   */
  calculateChargingEnergy(): number {
    // EC = e² / 2C
    const e = SUPERCONDUCTING_CONSTANTS.ELEMENTARY_CHARGE;
    return (e * e) / (2 * this.junction.capacitance);
  }

  /**
   * Simulate junction voltage
   */
  simulate(current: number): { voltage: number; power: number } {
    if (Math.abs(current) <= this.junction.criticalCurrent) {
      // Superconducting state
      return { voltage: 0, power: 0 };
    } else {
      // Resistive state
      const voltage = this.junction.normalResistance * current;
      const power = voltage * current;
      return { voltage, power };
    }
  }
}

// ============================================================================
// Superconducting Qubit Implementation
// ============================================================================

/**
 * Superconducting qubit controller
 */
export class SuperconductingQubitController {
  private state: ComplexNumber = { real: 1, imag: 0 }; // |0⟩
  private initialized: boolean = false;

  constructor(private qubit: SuperconductingQubit) {}

  /**
   * Initialize qubit to ground state
   */
  async initialize(): Promise<void> {
    this.state = { real: 1, imag: 0 }; // |0⟩
    this.initialized = true;
    // Simulate initialization time
    await this.delay(this.qubit.t1 / 10);
  }

  /**
   * Apply quantum gate
   */
  async applyGate(gate: QuantumGate, params?: { angle?: number }): Promise<void> {
    if (!this.initialized) {
      throw new Error('Qubit not initialized');
    }

    // Simulate gate time based on typical durations
    const gateDuration = this.getGateDuration(gate);
    await this.delay(gateDuration);

    // Apply gate operation (simplified)
    this.state = this.applyGateMatrix(gate, this.state, params);

    // Apply decoherence (simplified)
    this.applyDecoherence(gateDuration);
  }

  /**
   * Measure qubit
   */
  async measure(): Promise<QubitMeasurement> {
    if (!this.initialized) {
      throw new Error('Qubit not initialized');
    }

    // Measurement duration
    const duration = 1e-6; // 1 μs typical
    await this.delay(duration);

    // Probability of measuring |1⟩
    const p1 = this.state.imag * this.state.imag;

    // Measure with readout fidelity
    const readoutFidelity = 0.99;
    const measured = Math.random() < p1 ? 1 : 0;
    const actualState = Math.random() < readoutFidelity ? measured : 1 - measured;

    // Collapse state
    this.state = actualState === 0
      ? { real: 1, imag: 0 }
      : { real: 0, imag: 1 };

    return {
      state: actualState as 0 | 1,
      fidelity: readoutFidelity,
      snr: 20, // dB
      duration,
      excitedPopulation: p1,
    };
  }

  /**
   * Get qubit frequency
   */
  getFrequency(): number {
    return this.qubit.frequency;
  }

  /**
   * Get coherence times
   */
  getCoherenceTimes(): { t1: number; t2: number } {
    return {
      t1: this.qubit.t1,
      t2: this.qubit.t2,
    };
  }

  private applyGateMatrix(gate: QuantumGate, state: ComplexNumber, params?: { angle?: number }): ComplexNumber {
    // Simplified gate implementation
    switch (gate) {
      case 'I': // Identity
        return state;

      case 'X': // Pauli-X (NOT)
        return { real: state.imag, imag: state.real };

      case 'H': // Hadamard
        return {
          real: (state.real + state.imag) / Math.sqrt(2),
          imag: (state.real - state.imag) / Math.sqrt(2),
        };

      case 'Rx': // Rotation around X
        const angle = params?.angle || Math.PI / 2;
        return {
          real: Math.cos(angle / 2) * state.real - Math.sin(angle / 2) * state.imag,
          imag: Math.cos(angle / 2) * state.imag - Math.sin(angle / 2) * state.real,
        };

      default:
        return state;
    }
  }

  private applyDecoherence(duration: number): void {
    // Simplified decoherence model
    const t1Decay = Math.exp(-duration / this.qubit.t1);
    const t2Decay = Math.exp(-duration / this.qubit.t2);

    this.state.real *= t1Decay;
    this.state.imag *= t2Decay;

    // Renormalize
    const norm = Math.sqrt(this.state.real ** 2 + this.state.imag ** 2);
    if (norm > 0) {
      this.state.real /= norm;
      this.state.imag /= norm;
    }
  }

  private getGateDuration(gate: QuantumGate): number {
    // Typical gate durations in seconds
    switch (gate) {
      case 'I':
        return 0;
      case 'X':
      case 'Y':
      case 'Z':
        return 40e-9; // 40 ns
      case 'H':
        return 40e-9;
      case 'Rx':
      case 'Ry':
      case 'Rz':
        return 20e-9; // 20 ns for π/2
      default:
        return 40e-9;
    }
  }

  private delay(seconds: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, seconds * 1000));
  }
}

// ============================================================================
// SQUID Magnetometer Implementation
// ============================================================================

/**
 * SQUID magnetometer controller
 */
export class SQUIDMagnetometerController {
  constructor(private squid: SQUIDMagnetometer) {}

  /**
   * Measure magnetic field
   */
  async measureField(config: {
    duration: number; // milliseconds
    bandwidth: number; // Hz
    averaging: number;
  }): Promise<SQUIDMeasurement> {
    // Simulate measurement
    await this.delay(config.duration);

    // Simulated field (in reality, this would come from actual measurement)
    const field = this.simulateFieldMeasurement();

    // Calculate noise based on sensitivity and bandwidth
    const noise = this.squid.sensitivity * Math.sqrt(config.bandwidth);

    // SNR calculation
    const snr = 20 * Math.log10(Math.abs(field) / noise);

    return {
      field,
      noise,
      snr,
      duration: config.duration / 1000,
      flux: field * this.squid.loopArea / SUPERCONDUCTING_CONSTANTS.FLUX_QUANTUM,
    };
  }

  /**
   * Get sensitivity
   */
  getSensitivity(): number {
    return this.squid.sensitivity;
  }

  /**
   * Calibrate SQUID
   */
  async calibrate(): Promise<void> {
    // Simulate calibration procedure
    await this.delay(1000);
    console.log('SQUID calibrated');
  }

  private simulateFieldMeasurement(): number {
    // Return a simulated field measurement (e.g., Earth's magnetic field ~50 μT)
    return 50e-6 + (Math.random() - 0.5) * 1e-9; // 50 μT ± 0.5 nT
  }

  private delay(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}

// ============================================================================
// Maglev Controller Implementation
// ============================================================================

/**
 * Maglev system controller
 */
export class MaglevControllerImpl {
  private currentHeight: number = 0;
  private currentSpeed: number = 0;
  private isLevitating: boolean = false;

  constructor(private controller: MaglevController) {}

  /**
   * Engage levitation
   */
  async levitate(config: {
    targetHeight: number;
    stabilityThreshold: number;
  }): Promise<void> {
    console.log(`Engaging levitation to ${config.targetHeight * 100} cm...`);

    // Simulate levitation process
    const steps = 20;
    for (let i = 0; i <= steps; i++) {
      this.currentHeight = (i / steps) * config.targetHeight;
      await this.delay(100);
    }

    this.isLevitating = true;
    this.currentHeight = config.targetHeight;
    console.log('Levitation engaged');
  }

  /**
   * Land vehicle
   */
  async land(): Promise<void> {
    console.log('Landing...');

    const steps = 20;
    const startHeight = this.currentHeight;

    for (let i = steps; i >= 0; i--) {
      this.currentHeight = (i / steps) * startHeight;
      await this.delay(100);
    }

    this.isLevitating = false;
    this.currentHeight = 0;
    console.log('Landed');
  }

  /**
   * Accelerate
   */
  async accelerate(targetSpeed: number): Promise<void> {
    if (!this.isLevitating) {
      throw new Error('Cannot accelerate while not levitating');
    }

    const steps = 30;
    const speedIncrement = (targetSpeed - this.currentSpeed) / steps;

    for (let i = 0; i < steps; i++) {
      this.currentSpeed += speedIncrement;
      await this.delay(100);
    }

    this.currentSpeed = targetSpeed;
  }

  /**
   * Get current status
   */
  getStatus(): MaglevStatus {
    return {
      height: this.currentHeight,
      stability: 0.001, // meters RMS
      speed: this.currentSpeed,
      power: this.calculatePower(),
      temperature: 77, // Kelvin (if using YBCO with LN2)
      status: this.isLevitating ? 'levitating' : 'grounded',
    };
  }

  private calculatePower(): number {
    // Power = cooling power + propulsion power
    const coolingPower = this.controller.coolingPower;
    const propulsionPower = this.isLevitating
      ? 0.5 * this.controller.vehicleMass * this.currentSpeed * this.currentSpeed / 1000
      : 0;

    return coolingPower + propulsionPower;
  }

  private delay(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}

// ============================================================================
// Cryogenic System Implementation
// ============================================================================

/**
 * Cryogenic system controller
 */
export class CryogenicSystemController {
  private currentTemperature: number = 300; // Start at room temperature

  constructor(private system: CryogenicSystem) {}

  /**
   * Cool down to target temperature
   */
  async cooldown(progress?: ProgressCallback): Promise<void> {
    const steps = 100;
    const tempDrop = (this.currentTemperature - this.system.targetTemperature) / steps;
    const timePerStep = (this.system.cooldownTime * 3600 * 1000) / steps; // to milliseconds

    for (let i = 0; i < steps; i++) {
      this.currentTemperature -= tempDrop;
      await this.delay(timePerStep / 100); // Sped up for demo

      if (progress) {
        progress({
          current: i + 1,
          total: steps,
          percentage: ((i + 1) / steps) * 100,
          message: `Cooling: ${this.currentTemperature.toFixed(1)} K`,
        });
      }
    }

    this.currentTemperature = this.system.targetTemperature;
  }

  /**
   * Warm up to room temperature
   */
  async warmup(progress?: ProgressCallback): Promise<void> {
    const steps = 50;
    const tempRise = (300 - this.currentTemperature) / steps;
    const timePerStep = 100; // ms

    for (let i = 0; i < steps; i++) {
      this.currentTemperature += tempRise;
      await this.delay(timePerStep);

      if (progress) {
        progress({
          current: i + 1,
          total: steps,
          percentage: ((i + 1) / steps) * 100,
          message: `Warming: ${this.currentTemperature.toFixed(1)} K`,
        });
      }
    }

    this.currentTemperature = 300;
  }

  /**
   * Get current temperature
   */
  getTemperature(): number {
    return this.currentTemperature;
  }

  /**
   * Check if at target temperature
   */
  isAtTarget(): boolean {
    return Math.abs(this.currentTemperature - this.system.targetTemperature) < 0.1;
  }

  private delay(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Create a superconducting material from common presets
 */
export function createMaterial(name: string): Result<SuperconductingMaterial> {
  const materials: Record<string, SuperconductingMaterial> = {
    'NbTi': {
      name: 'NbTi',
      formula: 'Nb-47wt%Ti',
      type: 'type-II',
      family: 'alloy',
      pairingSymmetry: 's-wave',
      criticalTemperature: 9.2,
      criticalField: { bc1: 0.02, bc2: 15 },
      criticalCurrent: 3e9,
      cooperPairDensity: 1e28,
      penetrationDepth: 200,
      coherenceLength: 4,
      glParameter: 50,
      discoveryYear: 1962,
    },
    'YBCO': {
      name: 'YBCO',
      formula: 'YBa2Cu3O7',
      type: 'type-II-HTSC',
      family: 'cuprate',
      pairingSymmetry: 'd-wave',
      criticalTemperature: 92,
      criticalField: { bc1: 0.05, bc2: 120 },
      criticalCurrent: 1e10,
      cooperPairDensity: 5e27,
      penetrationDepth: 150,
      coherenceLength: 1.5,
      glParameter: 100,
      discoveryYear: 1987,
    },
    'Aluminum': {
      name: 'Aluminum',
      formula: 'Al',
      type: 'type-I',
      family: 'element',
      pairingSymmetry: 's-wave',
      criticalTemperature: 1.2,
      criticalField: 0.01,
      criticalCurrent: 1e8,
      cooperPairDensity: 1e29,
      penetrationDepth: 16,
      coherenceLength: 1600,
      glParameter: 0.01,
      discoveryYear: 1933,
    },
  };

  const material = materials[name];
  if (!material) {
    return {
      success: false,
      error: new Error(`Material '${name}' not found`),
    };
  }

  return { success: true, data: material };
}

// ============================================================================
// Main Exports
// ============================================================================

export {
  SuperconductingMaterialImpl as SuperconductingMaterial,
  JosephsonJunctionSimulator as JosephsonJunction,
  SuperconductingQubitController as SuperconductingQubit,
  SQUIDMagnetometerController as SQUIDMagnetometer,
  MaglevControllerImpl as MaglevController,
  CryogenicSystemController as CryogenicSystem,
  BCSTheory,
};

/**
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */
