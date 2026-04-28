/**
 * WIA-QUA-019: Room-Temperature Superconductor - TypeScript Type Definitions
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
 * Temperature scale
 */
export type TemperatureScale = 'kelvin' | 'celsius' | 'fahrenheit';

/**
 * Pressure units
 */
export type PressureUnit = 'pascal' | 'gpa' | 'bar' | 'atm';

// ============================================================================
// Material Classification
// ============================================================================

/**
 * Room-temperature superconductor material class
 */
export type RTSMaterialClass =
  | 'hydrogen-rich-hydride'
  | 'lk-99-type'
  | 'cuprate-htsc'
  | 'nickelate'
  | 'organic'
  | 'graphene-based'
  | 'theoretical';

/**
 * Pressure regime
 */
export type PressureRegime =
  | 'ambient' // < 1 GPa
  | 'low' // 1-10 GPa
  | 'moderate' // 10-50 GPa
  | 'high' // 50-150 GPa
  | 'ultra-high'; // > 150 GPa

/**
 * Superconducting state at room temperature
 */
export type RoomTempState =
  | 'confirmed-superconducting'
  | 'likely-superconducting'
  | 'controversial'
  | 'not-superconducting'
  | 'untested';

/**
 * Pairing mechanism
 */
export type PairingMechanism =
  | 'electron-phonon'
  | 'excitonic'
  | 'spin-fluctuation'
  | 'plasmon'
  | 'topological'
  | 'unknown';

// ============================================================================
// Hydrogen-Rich Hydride Properties
// ============================================================================

/**
 * Hydrogen-rich hydride compound
 */
export interface HydrideCompound {
  /** Chemical name */
  name: string;

  /** Chemical formula */
  formula: string;

  /** Crystal structure */
  latticeStructure: string;

  /** Critical temperature (Kelvin) */
  criticalTemperature: number;

  /** Pressure at which Tc is measured (Pascals) */
  criticalPressure: number;

  /** Hydrogen atomic percentage */
  hydrogenContent: number;

  /** Synthesis method */
  synthesisMethod: 'diamond-anvil-cell' | 'large-volume-press' | 'theoretical';

  /** Discovery year */
  discoveryYear?: number;

  /** Confirmation status */
  status: 'confirmed' | 'unconfirmed' | 'theoretical';

  /** Maximum critical field (Tesla) */
  criticalField?: number;

  /** Critical current density (A/m²) */
  criticalCurrentDensity?: number;
}

/**
 * Hydride synthesis parameters
 */
export interface HydrideSynthesisParams {
  /** Precursor materials */
  precursors: {
    metalElement: string;
    hydrogenPressure: number; // Pascals
    additionalElements?: string[];
  };

  /** Target pressure */
  targetPressure: number; // Pascals (GPa range)

  /** Synthesis temperature */
  temperature: number; // Kelvin

  /** Heating duration */
  duration: number; // seconds

  /** Laser heating */
  laserHeating?: {
    power: number; // Watts
    spotSize: number; // meters
    wavelength: number; // meters
  };

  /** Annealing */
  annealing?: {
    temperature: number; // Kelvin
    duration: number; // seconds
    coolingRate: number; // K/s
  };
}

// ============================================================================
// LK-99 Type Materials
// ============================================================================

/**
 * LK-99 material properties
 */
export interface LK99Material {
  /** Base composition */
  composition: string; // e.g., "Pb₁₀₋ₓCuₓ(PO₄)₆O"

  /** Copper doping level */
  copperDoping: number; // x value (0 to 1)

  /** Synthesis temperature (Kelvin) */
  synthesisTemp: number;

  /** Annealing time (hours) */
  annealingTime: number;

  /** Claimed critical temperature (Kelvin) */
  claimedTc?: number;

  /** Validation status */
  validationStatus: 'confirmed' | 'unconfirmed' | 'refuted' | 'inconclusive';

  /** Crystal phase */
  phase?: string;
}

/**
 * LK-99 synthesis configuration
 */
export interface LK99SynthesisConfig {
  /** Starting materials (grams) */
  materials: {
    leadOxide: number;
    leadSulfate: number;
    copperPhosphide: number;
  };

  /** Reaction conditions */
  reaction: {
    temperature: number; // Kelvin
    atmosphere: 'air' | 'vacuum' | 'argon' | 'nitrogen';
    duration: number; // hours
    heatingRate: number; // K/min
    coolingRate: number; // K/min
  };

  /** Annealing step */
  annealing?: {
    temperature: number; // Kelvin
    duration: number; // hours
    atmosphere: 'air' | 'vacuum' | 'inert';
  };

  /** Post-processing */
  postProcessing: {
    grinding: boolean;
    pelletizing?: number; // pressure in Pascals
    secondHeat?: boolean;
  };
}

// ============================================================================
// High-Pressure Synthesis
// ============================================================================

/**
 * Diamond anvil cell configuration
 */
export interface DiamondAnvilCell {
  /** Diamond anvils */
  diamonds: {
    type: 'Type-IIa' | 'Type-Ia' | 'boron-doped';
    culetSize: number; // meters (typically 10-100 microns)
    culetShape: 'flat' | 'beveled' | 'double-beveled';
    coating?: 'boron-doped-diamond';
  };

  /** Gasket material */
  gasket: {
    material: 'rhenium' | 'tungsten' | 'steel' | 'cu-be';
    thickness: number; // meters
    holeSize: number; // meters (sample chamber)
    preIndentation: number; // meters
  };

  /** Pressure generation */
  pressure: {
    target: number; // Pascals
    rampRate: number; // Pa/s
    calibration: 'ruby-fluorescence' | 'diamond-edge' | 'raman';
  };

  /** Heating methods */
  heating?: {
    method: 'laser' | 'resistive' | 'none';
    targetTemp: number; // Kelvin
    duration: number; // seconds
  };
}

/**
 * High-pressure synthesis result
 */
export interface HighPressureSynthesisResult {
  /** Success flag */
  success: boolean;

  /** Final pressure achieved (Pascals) */
  pressure: number;

  /** Final temperature (Kelvin) */
  temperature: number;

  /** Phase identification */
  phase: string;

  /** Sample quality */
  quality: 'excellent' | 'good' | 'fair' | 'poor';

  /** Errors or warnings */
  messages: string[];
}

// ============================================================================
// Characterization Methods
// ============================================================================

/**
 * Four-point resistance measurement
 */
export interface ResistanceMeasurement {
  /** Measurement method */
  method: 'four-point-probe' | 'two-point' | 'contactless';

  /** Temperature range (Kelvin) */
  temperatureRange: [number, number];

  /** Number of temperature points */
  temperaturePoints: number;

  /** Cooling/heating rate (K/min) */
  rampRate: number;

  /** Measurement current (Amperes) */
  measurementCurrent: number;

  /** AC frequency (Hz) for AC measurements */
  frequency?: number;

  /** Expected transition */
  expectedTransition: {
    tc: number; // Kelvin
    transitionWidth: number; // Kelvin
    residualResistance: number; // Ohms
    normalStateResistance: number; // Ohms
  };
}

/**
 * Resistance vs temperature measurement result
 */
export interface ResistanceVsTemperature {
  /** Temperature array (Kelvin) */
  temperature: number[];

  /** Resistance array (Ohms) */
  resistance: number[];

  /** Critical temperature (Kelvin) */
  tc: number;

  /** Transition width (Kelvin) */
  transitionWidth: number;

  /** Zero-resistance achieved */
  zeroResistance: boolean;

  /** Resistance ratio R(Tc)/R(300K) */
  resistanceRatio: number;
}

/**
 * Magnetic susceptibility measurement
 */
export interface MagneticMeasurement {
  /** Measurement instrument */
  instrument: 'SQUID' | 'VSM' | 'Hall-probe' | 'magnetometer';

  /** Applied magnetic field (Tesla) */
  appliedField: number;

  /** Field orientation */
  fieldOrientation: 'parallel' | 'perpendicular';

  /** Temperature range (Kelvin) */
  temperatureRange: [number, number];

  /** Zero-field-cooled measurement */
  zfc: boolean;

  /** Field-cooled measurement */
  fc: boolean;

  /** Expected susceptibility */
  expectedSusceptibility: number; // χ < -0.9 for superconductor
}

/**
 * Meissner effect test result
 */
export interface MeissnerTestResult {
  /** Magnetic susceptibility (dimensionless) */
  susceptibility: number;

  /** Levitation observed */
  levitating: boolean;

  /** Magnetic field expulsion fraction (0-1) */
  fieldExpulsion: number;

  /** Meissner fraction (0-1, 1 = perfect) */
  meissnerFraction: number;

  /** Temperature at test (Kelvin) */
  temperature: number;

  /** Applied field (Tesla) */
  appliedField: number;
}

/**
 * Critical current density measurement
 */
export interface CriticalCurrentTest {
  /** Test temperature (Kelvin) */
  temperature: number;

  /** Magnetic field (Tesla) */
  magneticField: number;

  /** Current range (A/m²) */
  currentRange: [number, number];

  /** Ramp rate (A/m²/s) */
  rampRate: number;

  /** Voltage criterion (V/m) */
  voltageCriterion: number;

  /** Measured critical current density (A/m²) */
  jc: number;

  /** Irreversibility field (Tesla) */
  irreversibilityField?: number;
}

/**
 * Spectroscopic analysis configuration
 */
export interface SpectroscopicAnalysis {
  /** X-ray diffraction */
  xrd?: {
    wavelength: number; // meters (Cu Kα: 1.5406 Å)
    rangeStart: number; // degrees 2θ
    rangeEnd: number; // degrees 2θ
    stepSize: number; // degrees
    phaseIdentification: string[];
    latticeParameters: number[]; // Angstroms
  };

  /** Raman spectroscopy */
  raman?: {
    laserWavelength: number; // meters (typically 532 nm)
    rangeStart: number; // cm⁻¹
    rangeEnd: number; // cm⁻¹
    peakAssignments: { [wavenumber: number]: string };
  };

  /** Photoemission spectroscopy (ARPES) */
  arpes?: {
    photonEnergy: number; // eV
    energyResolution: number; // meV
    gapMeasurement: number; // meV
  };
}

// ============================================================================
// Room-Temperature Superconductor System
// ============================================================================

/**
 * Room-temperature superconductor specification
 */
export interface RoomTempSuperconductor {
  /** Material properties */
  material: HydrideCompound | LK99Material;

  /** Operating pressure (Pascals) */
  operatingPressure: number;

  /** Target critical temperature (Kelvin, must be ≥ 300K) */
  targetTc: number;

  /** Pairing mechanism */
  pairingMechanism?: PairingMechanism;

  /** Critical current density at 300K (A/m²) */
  jc300K?: number;

  /** Critical field at 300K (Tesla) */
  bc300K?: number;

  /** Coherence length (meters) */
  coherenceLength?: number;

  /** Penetration depth (meters) */
  penetrationDepth?: number;
}

/**
 * Room-temperature superconductivity validation result
 */
export interface RTSValidationResult {
  /** Is room-temp capable (Tc ≥ 300K) */
  roomTempCapable: boolean;

  /** Measured critical temperature (Kelvin) */
  tc: number;

  /** Zero resistance confirmed */
  zeroResistance: boolean;

  /** Meissner effect observed */
  meissnerEffect: boolean;

  /** Critical current measured */
  criticalCurrent: boolean;

  /** Reproducibility across samples */
  reproducible: boolean;

  /** Independent verification */
  independentlyVerified: boolean;

  /** Overall confidence (0-1) */
  confidence: number;

  /** Validation tier (1-3, lower is more rigorous) */
  validationTier: 1 | 2 | 3;

  /** Detailed messages */
  messages: string[];
}

// ============================================================================
// Applications
// ============================================================================

/**
 * Superconducting power cable specification
 */
export interface SuperconductingCable {
  /** Cable geometry */
  geometry: {
    type: 'coaxial' | 'triaxial' | 'single-core';
    innerDiameter: number; // meters
    length: number; // meters
    layers: 'single' | 'multi-layer';
  };

  /** Superconductor material */
  material: RoomTempSuperconductor;

  /** Cooling system (minimal for room-temp SC) */
  cooling: {
    method: 'passive-air' | 'water-cooling' | 'none';
    targetTemp: number; // Kelvin (slightly below Tc for margin)
    powerCooling: number; // Watts
  };

  /** Electrical parameters */
  electrical: {
    voltage: number; // Volts
    current: number; // Amperes
    power: number; // Watts
    frequency: number; // Hz (0 for DC)
  };

  /** Performance metrics */
  efficiency: {
    transmission: number; // 0-1 (>0.999 for superconductor)
    losses: number; // Watts
    savings: number; // USD/year
  };
}

/**
 * Room-temperature maglev system
 */
export interface RoomTempMaglev {
  /** Superconducting system */
  superconductor: {
    material: RoomTempSuperconductor;
    operatingTemp: number; // Kelvin (300K for room-temp)
    configuration: 'bulk' | 'coated-conductor';
    levitationForce: number; // Newtons per meter
  };

  /** Track magnet system */
  track: {
    type: 'permanent-magnet-array' | 'electromagnet';
    field: number; // Tesla
    spacing: number; // meters
  };

  /** Vehicle parameters */
  vehicle: {
    mass: number; // kg
    levitationHeight: number; // meters
    maxSpeed: number; // m/s
    passengers: number;
  };

  /** Energy and propulsion */
  propulsion: {
    type: 'linear-motor';
    efficiency: number; // 0-1
    energyPerKm: number; // kWh/passenger/km
  };
}

/**
 * Room-temperature quantum computer
 */
export interface RoomTempQuantumComputer {
  /** Qubit system */
  qubits: {
    type: 'room-temp-superconducting-qubit';
    operatingTemp: number; // Kelvin (300K)
    coherenceT1: number; // seconds
    coherenceT2: number; // seconds
    gateTime: number; // seconds
    gateFidelity: number; // 0-1
  };

  /** System scaling */
  scaling: {
    qubitCount: number;
    connectivity: 'linear' | 'planar' | 'all-to-all';
    footprint: string; // e.g., "desktop-scale"
  };

  /** Infrastructure (no dilution fridge needed!) */
  infrastructure: {
    cooling: 'none' | 'air-conditioning';
    cost: number; // USD
    size: string; // e.g., "desktop", "room-scale"
    power: number; // Watts
  };

  /** Applications */
  applications: string[];
}

/**
 * Room-temperature MRI system
 */
export interface RoomTempMRI {
  /** Superconducting magnet */
  magnet: {
    material: RoomTempSuperconductor;
    fieldStrength: number; // Tesla
    homogeneity: number; // ppm over imaging volume
    stability: number; // per hour
    operatingTemp: number; // Kelvin (300K)
  };

  /** No cryogenics! */
  cooling: {
    cryogens: 'none';
    cooling: 'passive' | 'air-cooled';
    maintenance: 'minimal';
  };

  /** System specifications */
  system: {
    size: 'portable' | 'ambulance-mounted' | 'clinic';
    weight: number; // kg
    power: number; // Watts
    cost: number; // USD
  };

  /** Deployment scenarios */
  deployment: string[];
}

/**
 * Application simulation result
 */
export interface ApplicationSimulationResult {
  /** Application type */
  application: string;

  /** Power loss (Watts) */
  powerLoss: number;

  /** Conventional system loss for comparison (Watts) */
  conventionalLoss: number;

  /** Efficiency gain (0-1) */
  efficiencyGain: number;

  /** Annual cost savings (USD) */
  annualSavings: number;

  /** Environmental impact (CO₂ reduction, kg/year) */
  co2Reduction: number;

  /** Performance metrics */
  metrics: { [key: string]: number | string };
}

// ============================================================================
// Physical Constants (Room-Temperature Specific)
// ============================================================================

/**
 * Physical constants for room-temperature superconductivity
 */
export interface RTSPhysicalConstants {
  /** Room temperature threshold (Kelvin) */
  ROOM_TEMP_MIN: 300;

  /** Preferred target Tc (Kelvin) */
  ROOM_TEMP_PREFERRED: 350;

  /** Ideal target Tc (Kelvin) */
  ROOM_TEMP_IDEAL: 400;

  /** Boltzmann constant (J/K) */
  KB: 1.380649e-23;

  /** Boltzmann constant (eV/K) */
  KB_EV: 8.617333262e-5;

  /** Planck constant (J·s) */
  H: 6.62607015e-34;

  /** Reduced Planck constant (J·s) */
  HBAR: 1.054571817e-34;

  /** Elementary charge (C) */
  E: 1.602176634e-19;

  /** Electron mass (kg) */
  ME: 9.1093837015e-31;

  /** BCS gap ratio */
  BCS_GAP_RATIO: 1.764;

  /** Flux quantum (Wb) */
  PHI_0: 2.067833848e-15;

  /** Permeability of free space (H/m) */
  MU_0: 1.2566370614e-6;

  /** Standard atmospheric pressure (Pa) */
  ATM: 101325;

  /** 1 GPa in Pascals */
  GPA: 1e9;
}

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Temperature conversion result
 */
export interface TemperatureConversion {
  kelvin: number;
  celsius: number;
  fahrenheit: number;
}

/**
 * Pressure conversion result
 */
export interface PressureConversion {
  pascal: number;
  gpa: number;
  bar: number;
  atm: number;
}

/**
 * Material database entry
 */
export interface MaterialDatabaseEntry {
  /** Material identifier */
  id: string;

  /** Material class */
  class: RTSMaterialClass;

  /** Properties */
  properties: HydrideCompound | LK99Material;

  /** References */
  references: string[];

  /** Discovery date */
  discoveryDate?: string;

  /** Confirmation status */
  confirmationStatus: RoomTempState;
}

/**
 * Characterization suite configuration
 */
export interface CharacterizationSuite {
  /** Sample to characterize */
  sample: RoomTempSuperconductor;

  /** List of tests to perform */
  tests: (
    | 'resistance-vs-temperature'
    | 'meissner-effect'
    | 'critical-field'
    | 'critical-current'
    | 'hall-effect'
    | 'specific-heat'
    | 'raman-spectroscopy'
    | 'xray-diffraction'
    | 'arpes'
  )[];
}

/**
 * Comprehensive characterization report
 */
export interface CharacterizationReport {
  /** Total tests performed */
  totalTests: number;

  /** Tests passed */
  passedTests: string[];

  /** Tests failed */
  failedTests: string[];

  /** Overall confidence (0-1) */
  confidence: number;

  /** Is superconducting */
  isSuperconducting: boolean;

  /** Is room-temperature superconducting */
  isRoomTempSuperconducting: boolean;

  /** Detailed results */
  results: {
    resistance?: ResistanceVsTemperature;
    meissner?: MeissnerTestResult;
    criticalCurrent?: CriticalCurrentTest;
    spectroscopy?: SpectroscopicAnalysis;
  };

  /** Recommendations */
  recommendations: string[];
}

/**
 * Research roadmap
 */
export interface ResearchRoadmap {
  shortTerm: {
    goals: string[];
    funding: string;
    expectedBreakthrough: string;
  };
  mediumTerm: {
    goals: string[];
    funding: string;
    expectedBreakthrough: string;
  };
  longTerm: {
    goals: string[];
    funding: string;
    expectedBreakthrough: string;
  };
}

/**
 * Export all types
 */
export type {
  Vector3,
  ComplexNumber,
  TemperatureScale,
  PressureUnit,
  RTSMaterialClass,
  PressureRegime,
  RoomTempState,
  PairingMechanism,
};
