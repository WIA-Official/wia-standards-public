/**
 * WIA-QUA-007: Superconducting - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Quantum Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Physics Types
// ============================================================================

/**
 * Three-dimensional vector
 */
export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

/**
 * Complex number representation
 */
export interface ComplexNumber {
  real: number;
  imag: number;
}

/**
 * Complex function (simplified representation)
 */
export type ComplexFunction = (r: Vector3) => ComplexNumber;

// ============================================================================
// Superconductor Classification
// ============================================================================

/**
 * Superconductor type
 */
export type SuperconductorType =
  | 'type-I'
  | 'type-II'
  | 'type-II-HTSC'
  | 'unconventional';

/**
 * Superconductor family
 */
export type SuperconductorFamily =
  | 'element'
  | 'alloy'
  | 'cuprate'
  | 'iron-pnictide'
  | 'heavy-fermion'
  | 'organic'
  | 'magnesium-diboride'
  | 'topological';

/**
 * Pairing symmetry
 */
export type PairingSymmetry = 's-wave' | 'p-wave' | 'd-wave' | 'mixed';

/**
 * Superconducting state
 */
export type SuperconductingState =
  | 'normal'
  | 'superconducting'
  | 'mixed-state'
  | 'transition';

// ============================================================================
// Material Properties
// ============================================================================

/**
 * Superconducting material specification
 */
export interface SuperconductingMaterial {
  /** Material name */
  name: string;

  /** Chemical formula */
  formula?: string;

  /** Superconductor type */
  type: SuperconductorType;

  /** Material family */
  family: SuperconductorFamily;

  /** Pairing symmetry */
  pairingSymmetry: PairingSymmetry;

  /** Critical temperature (Kelvin) */
  criticalTemperature: number;

  /** Critical magnetic field (Tesla) */
  criticalField: number | {
    bc1?: number; // Lower critical field (Type II)
    bc2?: number; // Upper critical field (Type II)
    bcThermodynamic?: number; // Thermodynamic critical field
  };

  /** Critical current density (A/m²) */
  criticalCurrent: number;

  /** Cooper pair density (pairs/m³) */
  cooperPairDensity: number;

  /** London penetration depth (nanometers) */
  penetrationDepth?: number;

  /** Coherence length (nanometers) */
  coherenceLength?: number;

  /** Ginzburg-Landau parameter */
  glParameter?: number;

  /** Energy gap at T=0 (meV) */
  energyGap?: number;

  /** Fermi velocity (m/s) */
  fermiVelocity?: number;

  /** Discovery year */
  discoveryYear?: number;
}

/**
 * Operating state of superconductor
 */
export interface SuperconductingState {
  /** Current phase */
  phase: SuperconductingState;

  /** Temperature (Kelvin) */
  temperature: number;

  /** Magnetic field (Tesla) */
  magneticField: number;

  /** Current density (A/m²) */
  currentDensity: number;

  /** Resistance (Ohms) */
  resistance: number;

  /** Critical current at these conditions (A) */
  criticalCurrent: number;

  /** Energy gap (meV) */
  energyGap: number;

  /** Margin to critical values */
  margin: {
    temperature: number; // % to Tc
    field: number; // % to Bc
    current: number; // % to Jc
  };
}

// ============================================================================
// Cooper Pairs
// ============================================================================

/**
 * Cooper pair specification
 */
export interface CooperPair {
  /** Electron 1 properties */
  electron1: {
    spin: 'up' | 'down';
    momentum: Vector3;
    energy: number;
  };

  /** Electron 2 properties */
  electron2: {
    spin: 'up' | 'down';
    momentum: Vector3;
    energy: number;
  };

  /** Total spin (always 0 for singlet) */
  totalSpin: 0;

  /** Total momentum */
  totalMomentum: Vector3;

  /** Binding energy (meV) */
  bindingEnergy: number;

  /** Coherence length (nanometers) */
  coherenceLength: number;

  /** Wavefunction */
  wavefunction: ComplexFunction;

  /** Phase (radians) */
  phase: number;
}

// ============================================================================
// BCS Theory
// ============================================================================

/**
 * BCS theory parameters
 */
export interface BCSParameters {
  /** Material being analyzed */
  material: SuperconductingMaterial;

  /** Operating temperature (Kelvin) */
  temperature: number;

  /** Energy gap at operating temperature (meV) */
  energyGap: number;

  /** Coherence length at operating temperature (nm) */
  coherenceLength: number;

  /** Penetration depth at operating temperature (nm) */
  penetrationDepth: number;

  /** Critical field at operating temperature (Tesla) */
  criticalField: number;

  /** Density of states at Fermi level */
  densityOfStates?: number;

  /** Electron-phonon coupling constant */
  couplingConstant?: number;

  /** Debye temperature (Kelvin) */
  debyeTemperature?: number;
}

// ============================================================================
// Josephson Junctions
// ============================================================================

/**
 * Josephson junction type
 */
export type JunctionType =
  | 'SIS' // Superconductor-Insulator-Superconductor
  | 'SNS' // Superconductor-Normal-Superconductor
  | 'SInsS' // Superconductor-Insulator-Superconductor
  | 'dayem-bridge' // Superconducting weak link
  | 'variable-thickness'; // Variable thickness bridge

/**
 * Josephson junction specification
 */
export interface JosephsonJunction {
  /** Junction type */
  type: JunctionType;

  /** Junction area (m²) */
  area: number;

  /** Barrier thickness (nanometers) */
  barrierThickness: number;

  /** Critical current (Amperes) */
  criticalCurrent: number;

  /** Normal resistance (Ohms) */
  normalResistance: number;

  /** Capacitance (Farads) */
  capacitance: number;

  /** Josephson energy (GHz) */
  josephsonEnergy: number;

  /** Charging energy (GHz) */
  chargingEnergy: number;

  /** IcRn product (mV) */
  icRnProduct: number;

  /** Superconductors */
  superconductors: {
    material1: string;
    material2: string;
  };

  /** Barrier material */
  barrierMaterial: string;
}

/**
 * Josephson effect parameters
 */
export interface JosephsonEffect {
  /** DC Josephson effect */
  dc: {
    /** Supercurrent (A) */
    current: number;
    /** Phase difference (radians) */
    phaseDifference: number;
  };

  /** AC Josephson effect */
  ac: {
    /** Applied voltage (V) */
    voltage: number;
    /** Josephson frequency (Hz) */
    frequency: number; // f = 2eV/h
  };
}

/**
 * RCSJ (Resistively and Capacitively Shunted Junction) model
 */
export interface RCSJModel {
  /** Critical current (A) */
  criticalCurrent: number;

  /** Shunt resistance (Ω) */
  resistance: number;

  /** Shunt capacitance (F) */
  capacitance: number;

  /** Plasma frequency (Hz) */
  plasmaFrequency: number;

  /** Characteristic frequency (Hz) */
  characteristicFrequency: number;

  /** Stewart-McCumber parameter */
  stewartMcCumberParameter: number;

  /** Damping regime */
  dampingRegime: 'underdamped' | 'overdamped' | 'critically-damped';
}

// ============================================================================
// Superconducting Qubits
// ============================================================================

/**
 * Qubit type
 */
export type QubitType = 'transmon' | 'flux' | 'phase' | 'fluxonium' | 'xmon';

/**
 * Quantum gate type
 */
export type QuantumGate =
  | 'I' | 'X' | 'Y' | 'Z'
  | 'H' | 'S' | 'T'
  | 'Rx' | 'Ry' | 'Rz'
  | 'CNOT' | 'CZ' | 'iSWAP' | 'CPHASE';

/**
 * Superconducting qubit specification
 */
export interface SuperconductingQubit {
  /** Qubit type */
  type: QubitType;

  /** Qubit frequency (Hz) */
  frequency: number;

  /** Anharmonicity (Hz) */
  anharmonicity: number;

  /** Energy relaxation time T1 (seconds) */
  t1: number;

  /** Phase coherence time T2 (seconds) */
  t2: number;

  /** Echo coherence time T2* (seconds) */
  t2Echo?: number;

  /** Josephson junction(s) */
  junctions: JosephsonJunction[];

  /** Junction area (m²) */
  junctionArea: number;

  /** Capacitance (F) */
  capacitance: number;

  /** Josephson energy (GHz) */
  josephsonEnergy: number;

  /** Charging energy (GHz) */
  chargingEnergy: number;

  /** EJ/EC ratio */
  ejEcRatio?: number;

  /** Readout resonator */
  readoutResonator?: {
    frequency: number; // Hz
    quality: number; // Q factor
    coupling: number; // κ (Hz)
  };
}

/**
 * Qubit measurement result
 */
export interface QubitMeasurement {
  /** Measured state (0 or 1) */
  state: 0 | 1;

  /** Measurement fidelity */
  fidelity: number;

  /** Readout SNR (dB) */
  snr: number;

  /** Measurement duration (seconds) */
  duration: number;

  /** Population in excited state */
  excitedPopulation?: number;
}

/**
 * Gate operation specification
 */
export interface GateOperation {
  /** Gate type */
  gate: QuantumGate;

  /** Gate duration (seconds) */
  duration: number;

  /** Gate fidelity */
  fidelity: number;

  /** Control amplitude */
  amplitude?: number;

  /** Control phase */
  phase?: number;

  /** Rotation angle (for parametric gates) */
  angle?: number;

  /** Pulse shape */
  pulseShape?: 'gaussian' | 'DRAG' | 'square' | 'cosine';
}

// ============================================================================
// SQUID Devices
// ============================================================================

/**
 * SQUID type
 */
export type SQUIDType = 'dc-SQUID' | 'rf-SQUID';

/**
 * SQUID magnetometer specification
 */
export interface SQUIDMagnetometer {
  /** SQUID type */
  type: SQUIDType;

  /** Magnetic field sensitivity (Tesla/√Hz) */
  sensitivity: number;

  /** Noise floor (Tesla) */
  noiseFloor: number;

  /** Bandwidth (Hz) */
  bandwidth: number;

  /** Josephson junctions */
  junctions: JosephsonJunction[];

  /** SQUID loop inductance (H) */
  inductance: number;

  /** Loop area (m²) */
  loopArea: number;

  /** Critical current (A) */
  criticalCurrent: number;

  /** Flux quantum (Wb) */
  fluxQuantum: 2.067833848e-15;

  /** Flux resolution (Φ₀) */
  fluxResolution: number;
}

/**
 * SQUID measurement result
 */
export interface SQUIDMeasurement {
  /** Measured magnetic field (Tesla) */
  field: number;

  /** Measurement noise (Tesla/√Hz) */
  noise: number;

  /** Signal-to-noise ratio (dB) */
  snr: number;

  /** Measurement duration (seconds) */
  duration: number;

  /** Flux through SQUID (Φ₀ units) */
  flux?: number;
}

/**
 * Flux-locked loop configuration
 */
export interface FluxLockedLoop {
  /** Feedback gain (V/Φ₀) */
  gain: number;

  /** Feedback bandwidth (Hz) */
  bandwidth: number;

  /** Lock range (Φ₀ units) */
  lockRange: number;

  /** Modulation frequency (Hz) */
  modulationFrequency: number;

  /** Modulation amplitude (Φ₀ units) */
  modulationAmplitude: number;

  /** Dynamic range (Φ₀ units) */
  dynamicRange: number;

  /** Slew rate (Φ₀/s) */
  slewRate: number;
}

// ============================================================================
// Superconducting Magnets
// ============================================================================

/**
 * Magnet application type
 */
export type MagnetApplication =
  | 'MRI'
  | 'NMR'
  | 'particle-accelerator'
  | 'fusion'
  | 'research';

/**
 * Superconducting magnet specification
 */
export interface SuperconductingMagnet {
  /** Application type */
  application: MagnetApplication;

  /** Magnetic field strength (Tesla) */
  fieldStrength: number;

  /** Field homogeneity (ppm) */
  homogeneity: number;

  /** Bore diameter (meters) */
  boreDiameter: number;

  /** Superconductor material */
  conductor: string;

  /** Operating current (A) */
  operatingCurrent: number;

  /** Stored energy (Joules) */
  storedEnergy: number;

  /** Inductance (H) */
  inductance: number;

  /** Wire/tape specifications */
  wire: {
    material: string;
    diameter: number; // meters
    length: number; // meters
    criticalCurrent: number; // A
  };

  /** Cryogenic system */
  cryogenics: CryogenicSystem;

  /** Quench protection */
  quenchProtection: QuenchProtection;
}

// ============================================================================
// Maglev Systems
// ============================================================================

/**
 * Maglev suspension type
 */
export type MaglevType =
  | 'electromagnetic-suspension'
  | 'electrodynamic-suspension'
  | 'HTSC-levitation';

/**
 * Maglev controller specification
 */
export interface MaglevController {
  /** Suspension type */
  type: MaglevType;

  /** Vehicle mass (kg) */
  vehicleMass: number;

  /** Target levitation height (meters) */
  levitationHeight: number;

  /** Track length (meters) */
  trackLength: number;

  /** Maximum speed (m/s) */
  maxSpeed: number;

  /** Superconductor type */
  superconductor: string;

  /** Cooling system power (watts) */
  coolingPower: number;

  /** Magnetic field (Tesla) */
  magneticField?: number;

  /** Gap sensors */
  gapSensors?: number;

  /** Control bandwidth (Hz) */
  controlBandwidth?: number;
}

/**
 * Maglev status
 */
export interface MaglevStatus {
  /** Current height (meters) */
  height: number;

  /** Height stability (meters RMS) */
  stability: number;

  /** Current speed (m/s) */
  speed: number;

  /** Power consumption (watts) */
  power: number;

  /** Superconductor temperature (Kelvin) */
  temperature: number;

  /** System status */
  status: 'levitating' | 'grounded' | 'transition' | 'error';
}

// ============================================================================
// Cryogenic Systems
// ============================================================================

/**
 * Cryogenic cooling method
 */
export type CoolingMethod =
  | 'liquid-nitrogen'
  | 'liquid-helium'
  | 'cryocooler'
  | 'dilution-refrigerator'
  | 'superfluid-helium'
  | 'pulse-tube';

/**
 * Cryogenic system specification
 */
export interface CryogenicSystem {
  /** Target temperature (Kelvin) */
  targetTemperature: number;

  /** Cooling method */
  method: CoolingMethod;

  /** Cooling power at target temperature (watts) */
  coolingPower: number;

  /** Heat load (watts) */
  heatLoad: number;

  /** Cooldown time (hours) */
  cooldownTime: number;

  /** Hold time (hours, for closed systems) */
  holdTime?: number;

  /** Cryogen consumption */
  cryogenConsumption?: {
    liquidNitrogen?: number; // L/day
    liquidHelium?: number; // L/day
  };

  /** Cryocooler specifications */
  cryocooler?: {
    type: string;
    stages: number;
    inputPower: number; // watts
    efficiency: number; // % of Carnot
  };
}

/**
 * Temperature sensor
 */
export interface TemperatureSensor {
  /** Sensor type */
  type: 'diode' | 'RTD' | 'thermocouple' | 'RuO2';

  /** Location */
  location: string;

  /** Current reading (Kelvin) */
  temperature: number;

  /** Accuracy (Kelvin) */
  accuracy: number;

  /** Range (Kelvin) */
  range: {
    min: number;
    max: number;
  };
}

// ============================================================================
// Quench Protection
// ============================================================================

/**
 * Quench detection and protection
 */
export interface QuenchProtection {
  /** Voltage monitoring */
  voltageThreshold: number; // V

  /** Temperature monitoring */
  temperatureSensors: TemperatureSensor[];
  temperatureThreshold: number; // K

  /** Quench heaters */
  heaters: {
    count: number;
    power: number; // watts each
    coverage: number; // % of magnet
  };

  /** Energy dump resistor */
  dumpResistor: number; // Ω

  /** Fast discharge capability */
  fastDischarge: boolean;

  /** Detection time (seconds) */
  detectionTime: number;

  /** Protection time (seconds) */
  protectionTime: number;

  /** Maximum hot spot temperature (K) */
  maxHotSpotTemp: number;
}

// ============================================================================
// Power Transmission
// ============================================================================

/**
 * Superconducting power cable
 */
export interface SuperconductingPowerCable {
  /** Cable design */
  design: 'coaxial' | 'tri-axial' | 'warm-dielectric';

  /** HTS tape specifications */
  tape: {
    material: string;
    width: number; // mm
    thickness: number; // μm
    criticalCurrent: number; // A at 77K
  };

  /** Cable length (meters) */
  length: number;

  /** Cable diameter (meters) */
  diameter: number;

  /** Power capacity (watts) */
  capacity: number;

  /** Operating current (A) */
  current: number;

  /** Operating voltage (V) */
  voltage: number;

  /** Cooling system */
  cooling: {
    cryogen: string;
    flowRate: number; // L/min
    refrigerationPower: number; // watts
  };

  /** Losses */
  losses: {
    conductor: number; // W/m
    dielectric: number; // W/m
    thermal: number; // W/m
    total: number; // W/m
  };
}

// ============================================================================
// Testing & Quality Metrics
// ============================================================================

/**
 * Superconductor quality metrics
 */
export interface QualityMetrics {
  /** Tc uniformity (% variation) */
  tcUniformity: number;

  /** Jc reproducibility (% variation) */
  jcReproducibility: number;

  /** Surface roughness (nanometers RMS) */
  surfaceRoughness: number;

  /** Defect density (defects/cm²) */
  defectDensity: number;

  /** Manufacturing yield (%) */
  manufacturingYield: number;

  // For qubits
  qubitMetrics?: {
    t1: number; // μs
    t2: number; // μs
    gateFidelity: number; // 0-1
  };

  // For SQUIDs
  squidMetrics?: {
    sensitivity: number; // T/√Hz
    noise: number; // T/√Hz
  };
}

/**
 * Test configuration
 */
export interface TestConfiguration {
  /** Test type */
  testType: 'tc' | 'jc' | 'bc' | 'coherence' | 'sensitivity';

  /** Temperature range (Kelvin) */
  temperatureRange: {
    min: number;
    max: number;
    step: number;
  };

  /** Magnetic field range (Tesla) */
  fieldRange?: {
    min: number;
    max: number;
    step: number;
  };

  /** Current range (A) */
  currentRange?: {
    min: number;
    max: number;
    step: number;
  };

  /** Measurement accuracy */
  accuracy: {
    temperature: number; // K
    field: number; // T
    current: number; // A
    voltage: number; // V
  };
}

// ============================================================================
// Safety
// ============================================================================

/**
 * Magnetic field safety zones
 */
export interface MagneticFieldSafety {
  /** Field strength (Tesla) */
  fieldStrength: number;

  /** Safety zones */
  zones: {
    publicAccess: number; // T (typically 0.5 mT)
    occupationalExposure: number; // T (typically 2 mT)
    controlledAccess: number; // T (typically 0.5 T)
    pacemakerLimit: number; // T (typically 0.5 mT)
  };

  /** Projectile risk threshold (T) */
  projectileRiskThreshold: number;

  /** Warning signage locations */
  signageLocations: string[];
}

/**
 * Cryogenic safety parameters
 */
export interface CryogenicSafety {
  /** Oxygen level monitoring */
  oxygenMonitoring: {
    enabled: boolean;
    alarmLevel: number; // % O2 (19.5% minimum)
    sensors: number;
  };

  /** Personal protective equipment */
  ppe: string[];

  /** Ventilation requirements */
  ventilation: {
    airChangesPerHour: number;
    emergencyExhaust: boolean;
  };

  /** Pressure relief */
  pressureRelief: {
    burstDisks: boolean;
    reliefValves: boolean;
    ventingCapacity: number; // m³/s
  };
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Physical constants for superconductivity
 */
export const SUPERCONDUCTING_CONSTANTS = {
  /** Flux quantum (Wb) */
  FLUX_QUANTUM: 2.067833848e-15,

  /** Elementary charge (C) */
  ELEMENTARY_CHARGE: 1.602176634e-19,

  /** Planck constant (J·s) */
  PLANCK_CONSTANT: 6.62607015e-34,

  /** Reduced Planck constant (J·s) */
  HBAR: 1.054571817e-34,

  /** Boltzmann constant (J/K) */
  BOLTZMANN_CONSTANT: 1.380649e-23,

  /** Boltzmann constant (eV/K) */
  KB_EV: 8.617333262e-5,

  /** Electron mass (kg) */
  ELECTRON_MASS: 9.1093837015e-31,

  /** Permeability of free space (H/m) */
  MU_0: 4 * Math.PI * 1e-7,

  /** BCS energy gap ratio */
  BCS_GAP_RATIO: 1.764,

  /** Josephson constant (Hz/V) */
  JOSEPHSON_CONSTANT: 483597.8484e9,
} as const;

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Result type for operations
 */
export type Result<T, E = Error> =
  | { success: true; data: T }
  | { success: false; error: E };

/**
 * Async result type
 */
export type AsyncResult<T, E = Error> = Promise<Result<T, E>>;

/**
 * Progress callback
 */
export type ProgressCallback = (progress: {
  current: number;
  total: number;
  percentage: number;
  message: string;
}) => void;

// ============================================================================
// Export All
// ============================================================================

export type {
  // Materials
  SuperconductingMaterial,
  SuperconductingState,

  // Cooper Pairs & BCS
  CooperPair,
  BCSParameters,

  // Josephson Junctions
  JosephsonJunction,
  JosephsonEffect,
  RCSJModel,

  // Qubits
  SuperconductingQubit,
  QubitMeasurement,
  GateOperation,

  // SQUIDs
  SQUIDMagnetometer,
  SQUIDMeasurement,
  FluxLockedLoop,

  // Magnets
  SuperconductingMagnet,

  // Maglev
  MaglevController,
  MaglevStatus,

  // Cryogenics
  CryogenicSystem,
  TemperatureSensor,

  // Protection
  QuenchProtection,

  // Power
  SuperconductingPowerCable,

  // Testing
  QualityMetrics,
  TestConfiguration,

  // Safety
  MagneticFieldSafety,
  CryogenicSafety,
};

export { SUPERCONDUCTING_CONSTANTS };

/**
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */
