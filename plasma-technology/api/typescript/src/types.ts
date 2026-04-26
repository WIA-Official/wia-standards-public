/**
 * WIA-QUA-008: Plasma Technology - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Plasma Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Plasma Types
// ============================================================================

/**
 * Plasma state classification
 */
export type PlasmaType = 'thermal' | 'non-thermal' | 'fusion' | 'low-temperature' | 'high-temperature';

/**
 * Plasma generation method
 */
export type PlasmaGenerationMethod = 'rf-ccp' | 'rf-icp' | 'dc-glow' | 'dc-arc' | 'microwave' | 'ecr' | 'dbd' | 'plasma-jet';

/**
 * Basic plasma parameters
 */
export interface PlasmaParameters {
  /** Electron density (m⁻³) */
  electronDensity: number;

  /** Ion density (m⁻³) */
  ionDensity?: number;

  /** Neutral density (m⁻³) */
  neutralDensity?: number;

  /** Electron temperature (K) */
  electronTemperature: number;

  /** Ion temperature (K) */
  ionTemperature?: number;

  /** Gas temperature (K) */
  gasTemperature?: number;

  /** Debye length (m) */
  debyeLength?: number;

  /** Plasma frequency (Hz) */
  plasmaFrequency?: number;

  /** Ionization degree (0-1) */
  ionizationDegree?: number;
}

/**
 * Calculated plasma properties
 */
export interface PlasmaProperties extends PlasmaParameters {
  /** Debye length (m) */
  debyeLength: number;

  /** Electron plasma frequency (rad/s) */
  plasmaFrequency: number;

  /** Plasma parameter (particles in Debye sphere) */
  plasmaParameter: number;

  /** Ionization degree (0-1) */
  ionizationDegree: number;

  /** Plasma type classification */
  plasmaType: PlasmaType;

  /** Quasi-neutrality check */
  isQuasiNeutral: boolean;

  /** Collision frequency (s⁻¹) */
  collisionFrequency?: number;
}

// ============================================================================
// Plasma Generation
// ============================================================================

/**
 * RF plasma configuration
 */
export interface RFPlasmaConfig {
  /** RF frequency (Hz) */
  frequency: number;

  /** RF power (W) */
  power: number;

  /** Gas pressure (Pa) */
  pressure: number;

  /** Gas type */
  gas: string;

  /** Coupling type */
  coupling: 'capacitive' | 'inductive';

  /** Electrode area (m²) */
  electrodeArea?: number;

  /** Chamber volume (m³) */
  volume?: number;
}

/**
 * DC plasma configuration
 */
export interface DCPlasmaConfig {
  /** Discharge voltage (V) */
  voltage: number;

  /** Discharge current (A) */
  current: number;

  /** Gas pressure (Pa) */
  pressure: number;

  /** Gap distance (m) */
  gapDistance: number;

  /** Cathode area (m²) */
  cathodeArea: number;

  /** Discharge type */
  dischargeType: 'glow' | 'arc';
}

/**
 * Microwave plasma configuration
 */
export interface MicrowavePlasmaConfig {
  /** Microwave frequency (Hz) */
  frequency: number;

  /** Microwave power (W) */
  power: number;

  /** Gas pressure (Pa) */
  pressure: number;

  /** Waveguide mode */
  waveguideMode: string;

  /** Magnetic field (T) for ECR */
  magneticField?: number;
}

/**
 * Plasma generation result
 */
export interface PlasmaGenerationResult {
  /** Generation method used */
  method: PlasmaGenerationMethod;

  /** Achieved plasma parameters */
  parameters: PlasmaProperties;

  /** Power absorbed by plasma (W) */
  absorbedPower: number;

  /** Plasma density uniformity (0-1) */
  uniformity: number;

  /** Generation efficiency (0-1) */
  efficiency: number;

  /** Stability status */
  stable: boolean;

  /** Warnings */
  warnings: string[];
}

// ============================================================================
// Fusion Plasma
// ============================================================================

/**
 * Fusion reactor type
 */
export type FusionReactorType = 'tokamak' | 'stellarator' | 'icf' | 'frc' | 'z-pinch';

/**
 * Tokamak configuration
 */
export interface TokamakConfig {
  /** Major radius (m) */
  majorRadius: number;

  /** Minor radius (m) */
  minorRadius: number;

  /** Plasma current (A) */
  plasmaCurrent: number;

  /** Toroidal magnetic field (T) */
  toroidalField: number;

  /** Plasma density (m⁻³) */
  plasmaDensity: number;

  /** Ion temperature (K) */
  ionTemperature: number;

  /** Elongation */
  elongation?: number;

  /** Triangularity */
  triangularity?: number;
}

/**
 * Stellarator configuration
 */
export interface StellatorConfig {
  /** Major radius (m) */
  majorRadius: number;

  /** Average minor radius (m) */
  minorRadius: number;

  /** Plasma density (m⁻³) */
  plasmaDensity: number;

  /** Ion temperature (K) */
  ionTemperature: number;

  /** Magnetic field (T) */
  magneticField: number;

  /** Rotational transform */
  rotationalTransform: number;
}

/**
 * Fusion performance metrics
 */
export interface FusionPerformance {
  /** Fusion power (W) */
  fusionPower: number;

  /** Fusion reaction rate (reactions/m³/s) */
  reactionRate: number;

  /** Energy confinement time (s) */
  confinementTime: number;

  /** Triple product n·T·τ (keV·s/m³) */
  tripleProduct: number;

  /** Q factor (fusion power / heating power) */
  qFactor: number;

  /** Safety factor q */
  safetyFactor: number;

  /** Plasma beta */
  beta: number;

  /** Lawson criterion satisfied */
  lawsonSatisfied: boolean;

  /** Ignition achieved */
  ignition: boolean;
}

// ============================================================================
// Plasma Processing
// ============================================================================

/**
 * Etching process parameters
 */
export interface EtchingParams {
  /** Plasma power (W) */
  power: number;

  /** Gas flow rate (sccm) */
  flowRate: number;

  /** Pressure (Pa) */
  pressure: number;

  /** Bias voltage (V) */
  biasVoltage?: number;

  /** Process gas composition */
  gasComposition: Record<string, number>;

  /** Substrate temperature (K) */
  substrateTemperature?: number;
}

/**
 * Etching result
 */
export interface EtchingResult {
  /** Etch rate (nm/min) */
  etchRate: number;

  /** Selectivity (material/mask) */
  selectivity: number;

  /** Aspect ratio (depth/width) */
  aspectRatio: number;

  /** Anisotropy (0=isotropic, 1=perfectly anisotropic) */
  anisotropy: number;

  /** Uniformity across wafer (%) */
  uniformity: number;

  /** Surface roughness (nm RMS) */
  roughness: number;
}

/**
 * PECVD deposition parameters
 */
export interface PECVDParams {
  /** Plasma power (W) */
  power: number;

  /** Precursor gas flow (sccm) */
  precursorFlow: number;

  /** Carrier gas flow (sccm) */
  carrierFlow?: number;

  /** Pressure (Pa) */
  pressure: number;

  /** Substrate temperature (K) */
  substrateTemperature: number;

  /** RF frequency (Hz) */
  frequency?: number;
}

/**
 * PECVD deposition result
 */
export interface PECVDResult {
  /** Deposition rate (nm/min) */
  depositionRate: number;

  /** Film thickness (nm) */
  thickness: number;

  /** Refractive index */
  refractiveIndex: number;

  /** Film density (g/cm³) */
  density: number;

  /** Stress (MPa, positive=tensile, negative=compressive) */
  stress: number;

  /** Uniformity (%) */
  uniformity: number;
}

// ============================================================================
// Plasma Medicine
// ============================================================================

/**
 * Medical plasma application type
 */
export type MedicalPlasmaType = 'wound-healing' | 'sterilization' | 'cancer-therapy' | 'dental' | 'skin-treatment';

/**
 * Cold atmospheric plasma (CAP) parameters
 */
export interface CAPParameters {
  /** Application type */
  applicationType: MedicalPlasmaType;

  /** Treatment time (s) */
  treatmentTime: number;

  /** Distance to tissue (mm) */
  distance: number;

  /** Plasma power (W) */
  power: number;

  /** Gas flow rate (slpm) */
  gasFlowRate: number;

  /** Working gas */
  gas: string;

  /** Pulsed mode */
  pulsed?: boolean;

  /** Pulse frequency (Hz) */
  pulseFrequency?: number;
}

/**
 * Reactive species concentrations
 */
export interface ReactiveSpecies {
  /** OH radicals (molecules/cm³) */
  OH?: number;

  /** Ozone O₃ (ppm) */
  O3?: number;

  /** Hydrogen peroxide H₂O₂ (ppm) */
  H2O2?: number;

  /** Nitric oxide NO (ppm) */
  NO?: number;

  /** Superoxide O₂⁻ (molecules/cm³) */
  O2_minus?: number;
}

/**
 * Medical plasma treatment result
 */
export interface MedicalPlasmaResult {
  /** Reactive species generated */
  reactiveSpecies: ReactiveSpecies;

  /** UV dose (mJ/cm²) */
  uvDose: number;

  /** Tissue temperature increase (°C) */
  temperatureIncrease: number;

  /** Bacterial reduction (log) */
  bacterialReduction?: number;

  /** Safety assessment */
  safetyStatus: 'safe' | 'warning' | 'unsafe';

  /** Warnings */
  warnings: string[];
}

// ============================================================================
// Plasma Propulsion
// ============================================================================

/**
 * Thruster type
 */
export type ThrusterType = 'ion' | 'hall' | 'vasimr' | 'ppt' | 'mpd';

/**
 * Ion thruster configuration
 */
export interface IonThrusterConfig {
  /** Propellant (e.g., 'xenon', 'krypton', 'argon') */
  propellant: string;

  /** Beam power (W) */
  beamPower: number;

  /** Acceleration voltage (V) */
  accelerationVoltage: number;

  /** Mass flow rate (mg/s) */
  massFlowRate: number;

  /** Beam current (A) */
  beamCurrent: number;
}

/**
 * Hall effect thruster configuration
 */
export interface HallThrusterConfig {
  /** Propellant */
  propellant: string;

  /** Discharge power (W) */
  dischargePower: number;

  /** Discharge voltage (V) */
  dischargeVoltage: number;

  /** Mass flow rate (mg/s) */
  massFlowRate: number;

  /** Magnetic field (T) */
  magneticField: number;
}

/**
 * Thruster performance
 */
export interface ThrusterPerformance {
  /** Thrust (N) */
  thrust: number;

  /** Specific impulse (s) */
  specificImpulse: number;

  /** Efficiency (0-1) */
  efficiency: number;

  /** Exhaust velocity (m/s) */
  exhaustVelocity: number;

  /** Power-to-thrust ratio (W/mN) */
  powerToThrustRatio: number;

  /** Propellant consumption rate (kg/day) */
  propellantConsumption: number;
}

// ============================================================================
// Plasma Diagnostics
// ============================================================================

/**
 * Langmuir probe measurement
 */
export interface LangmuirProbe {
  /** Voltage sweep range [min, max] (V) */
  voltageRange: [number, number];

  /** Current measurements (A) */
  currentData: number[];

  /** Voltage points (V) */
  voltageData: number[];

  /** Probe area (m²) */
  probeArea: number;
}

/**
 * Langmuir probe analysis result
 */
export interface LangmuirProbeResult {
  /** Electron density (m⁻³) */
  electronDensity: number;

  /** Electron temperature (eV) */
  electronTemperatureEV: number;

  /** Electron temperature (K) */
  electronTemperatureK: number;

  /** Plasma potential (V) */
  plasmaPotential: number;

  /** Floating potential (V) */
  floatingPotential: number;

  /** Electron saturation current (A) */
  electronSaturationCurrent: number;

  /** Ion saturation current (A) */
  ionSaturationCurrent: number;
}

/**
 * Optical emission spectroscopy data
 */
export interface OESData {
  /** Wavelengths (nm) */
  wavelengths: number[];

  /** Intensities (arbitrary units) */
  intensities: number[];

  /** Spectral resolution (nm) */
  resolution: number;

  /** Integration time (ms) */
  integrationTime: number;
}

/**
 * OES analysis result
 */
export interface OESResult {
  /** Identified spectral lines */
  spectralLines: Array<{
    wavelength: number;
    species: string;
    transition: string;
    intensity: number;
  }>;

  /** Electron temperature (eV, from line ratio) */
  electronTemperature?: number;

  /** Gas temperature (K, from Doppler broadening) */
  gasTemperature?: number;

  /** Relative species densities */
  speciesDensities?: Record<string, number>;
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Physical constants for plasma calculations
 */
export const PLASMA_CONSTANTS = {
  /** Elementary charge (C) */
  ELEMENTARY_CHARGE: 1.602176634e-19,

  /** Electron mass (kg) */
  ELECTRON_MASS: 9.1093837015e-31,

  /** Proton mass (kg) */
  PROTON_MASS: 1.67262192369e-27,

  /** Boltzmann constant (J/K) */
  BOLTZMANN: 1.380649e-23,

  /** Vacuum permittivity (F/m) */
  EPSILON_0: 8.8541878128e-12,

  /** Vacuum permeability (H/m) */
  MU_0: 1.25663706212e-6,

  /** Speed of light (m/s) */
  SPEED_OF_LIGHT: 299792458,

  /** Planck constant (J·s) */
  PLANCK: 6.62607015e-34,

  /** Avogadro number (mol⁻¹) */
  AVOGADRO: 6.02214076e23,

  /** Standard gravity (m/s²) */
  STANDARD_GRAVITY: 9.80665,

  /** Atomic mass unit (kg) */
  AMU: 1.66053906660e-27,

  /** eV to Joules conversion */
  EV_TO_JOULES: 1.602176634e-19,

  /** eV to Kelvin conversion (k = kT/e) */
  EV_TO_KELVIN: 11604.52,
} as const;

/**
 * Common gas properties
 */
export const GAS_PROPERTIES = {
  argon: { mass: 39.948, ionizationEnergy: 15.76 }, // AMU, eV
  helium: { mass: 4.0026, ionizationEnergy: 24.59 },
  neon: { mass: 20.180, ionizationEnergy: 21.56 },
  krypton: { mass: 83.798, ionizationEnergy: 14.00 },
  xenon: { mass: 131.29, ionizationEnergy: 12.13 },
  oxygen: { mass: 31.999, ionizationEnergy: 12.07 },
  nitrogen: { mass: 28.014, ionizationEnergy: 14.53 },
  hydrogen: { mass: 2.016, ionizationEnergy: 13.60 },
} as const;

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Result type for operations that can fail
 */
export type Result<T, E = Error> =
  | { success: true; data: T }
  | { success: false; error: E };

/**
 * Async result type
 */
export type AsyncResult<T, E = Error> = Promise<Result<T, E>>;

/**
 * Plasma simulation result
 */
export interface SimulationResult {
  /** Simulation ID */
  id: string;

  /** Simulation type */
  type: 'generation' | 'fusion' | 'processing' | 'propulsion' | 'medical';

  /** Input parameters */
  input: Record<string, unknown>;

  /** Output results */
  output: Record<string, unknown>;

  /** Simulation duration (ms) */
  duration: number;

  /** Success status */
  success: boolean;

  /** Warnings */
  warnings: string[];

  /** Error message if failed */
  error?: string;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-QUA-008 error codes
 */
export enum PlasmaErrorCode {
  INVALID_PARAMETERS = 'P001',
  DENSITY_TOO_LOW = 'P002',
  TEMPERATURE_OUT_OF_RANGE = 'P003',
  POWER_INSUFFICIENT = 'P004',
  PRESSURE_OUT_OF_RANGE = 'P005',
  INSTABILITY_DETECTED = 'P006',
  BREAKDOWN_FAILED = 'P007',
  CONFINEMENT_LOSS = 'P008',
  SAFETY_LIMIT_EXCEEDED = 'P009',
  EQUIPMENT_FAILURE = 'P010',
}

/**
 * Plasma technology error
 */
export class PlasmaError extends Error {
  constructor(
    public code: PlasmaErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'PlasmaError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  PlasmaType,
  PlasmaGenerationMethod,
  PlasmaParameters,
  PlasmaProperties,
  RFPlasmaConfig,
  DCPlasmaConfig,
  MicrowavePlasmaConfig,
  PlasmaGenerationResult,
  FusionReactorType,
  TokamakConfig,
  StellatorConfig,
  FusionPerformance,
  EtchingParams,
  EtchingResult,
  PECVDParams,
  PECVDResult,
  MedicalPlasmaType,
  CAPParameters,
  ReactiveSpecies,
  MedicalPlasmaResult,
  ThrusterType,
  IonThrusterConfig,
  HallThrusterConfig,
  ThrusterPerformance,
  LangmuirProbe,
  LangmuirProbeResult,
  OESData,
  OESResult,
  SimulationResult,
};

export { PLASMA_CONSTANTS, GAS_PROPERTIES, PlasmaErrorCode, PlasmaError };

**弘益人間 (홍익인간) · Benefit All Humanity**
