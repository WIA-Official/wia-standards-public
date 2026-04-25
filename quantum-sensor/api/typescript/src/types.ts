/**
 * WIA-QUA-004: Quantum Sensor - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Quantum Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
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
 * Geographic location
 */
export interface GeoLocation {
  latitude: number;
  longitude: number;
  altitude: number;
  geoid?: string;
}

/**
 * Measurement quality indicators
 */
export interface QualityMetrics {
  snr: number;
  validity: boolean;
  flags: string[];
  chiSquared?: number;
  residualsRms?: number;
}

/**
 * Uncertainty budget
 */
export interface Uncertainty {
  typeA: number;          // Statistical (1σ)
  typeB: {
    instrumental: number;
    environmental: number;
    model: number;
    [key: string]: number;
  };
  combined: number;       // √(A² + ΣBᵢ²)
  expanded: number;       // k × combined
  coverageFactor: number; // Typically 2 (95% confidence)
}

// ============================================================================
// Sensor Configuration
// ============================================================================

/**
 * Sensor types supported by this standard
 */
export type SensorType =
  | 'atomic-clock'
  | 'magnetometer'
  | 'gravimeter'
  | 'accelerometer'
  | 'gyroscope'
  | 'imaging'
  | 'radar';

/**
 * Base sensor configuration
 */
export interface SensorConfig {
  sensorType: SensorType;
  model: string;
  serialNumber: string;
  calibrationDate: Date | string;
  operatingMode: string;
  manufacturer?: string;
  firmwareVersion?: string;
}

/**
 * Sensor initialization parameters
 */
export interface SensorInit {
  config: SensorConfig;
  warmupTime?: number;      // seconds
  selfTest?: boolean;
  environmentalCheck?: boolean;
}

// ============================================================================
// Atomic Clock Types
// ============================================================================

/**
 * Atom types for atomic clocks
 */
export type AtomType =
  | 'cesium-133'
  | 'rubidium-87'
  | 'strontium-87'
  | 'ytterbium-171'
  | 'hydrogen';

/**
 * Transition types
 */
export type TransitionType = 'microwave' | 'optical';

/**
 * Atomic clock configuration
 */
export interface AtomicClockConfig extends SensorConfig {
  sensorType: 'atomic-clock';
  atomType: AtomType;
  transitionFrequency: number;  // Hz
  transitionType: TransitionType;
  accuracy: number;             // Fractional frequency uncertainty
  stability: number;            // Allan deviation at 1 second
  warmupTime: number;           // seconds
  temperature: number;          // Kelvin
  magneticShielding: boolean;
  vibrationIsolation: boolean;
}

/**
 * Atomic clock measurement result
 */
export interface AtomicClockMeasurement {
  measurementId: string;
  timestamp: Date | string;
  atomType: AtomType;
  frequency: number;
  uncertainty: number;
  fractionalAccuracy: number;
  allanDeviation: {
    '1s': number;
    '100s': number;
    '10000s': number;
    [key: string]: number;
  };
  environmental: {
    temperature: number;
    magneticField: number;
    pressure: number;
  };
  utc?: string;
  tai?: string;
}

// ============================================================================
// Magnetometer Types
// ============================================================================

/**
 * Magnetometer types
 */
export type MagnetometerType =
  | 'SQUID'
  | 'OPM'           // Optically Pumped
  | 'NV-center'     // Diamond NV-center
  | 'spin-exchange'
  | 'fluxgate';

/**
 * SQUID types
 */
export type SQUIDType = 'DC-SQUID' | 'RF-SQUID';

/**
 * Magnetometer configuration
 */
export interface MagnetometerConfig extends SensorConfig {
  sensorType: 'magnetometer';
  type: MagnetometerType;
  squidType?: SQUIDType;
  sensitivity: number;          // Tesla or Tesla/√Hz
  bandwidth: [number, number];  // [low, high] Hz
  dynamicRange: number;         // Tesla
  coolingRequired: boolean;
  operatingTemperature: number; // Kelvin
  powerConsumption: number;     // Watts
  inputCoil?: {
    inductance: number;
    coupling: number;
  };
}

/**
 * Magnetic field vector
 */
export interface MagneticField {
  x: number;
  y: number;
  z: number;
  magnitude: number;
  unit: 'Tesla' | 'Gauss' | 'nT';
  direction?: {
    azimuth: number;    // degrees
    elevation: number;  // degrees
  };
}

/**
 * Magnetometer measurement result
 */
export interface MagnetometerMeasurement {
  measurementId: string;
  timestamp: Date | string;
  sensorType: MagnetometerType;
  fieldVector: MagneticField;
  uncertainty: number;
  bandwidth: number;
  integrationTime: number;      // milliseconds
  environmental: {
    temperature: number;
    shieldingFactor?: number;
  };
  quality: QualityMetrics;
}

// ============================================================================
// Gravimeter Types
// ============================================================================

/**
 * Gravimeter configuration types
 */
export type GravimeterConfiguration =
  | 'fountain'
  | 'drop'
  | 'catapult'
  | 'continuous';

/**
 * Gravimeter configuration
 */
export interface GravimeterConfig extends SensorConfig {
  sensorType: 'gravimeter';
  atomType: 'rubidium-87' | 'cesium-133';
  configuration: GravimeterConfiguration;
  dropHeight: number;           // meters
  dropTime: number;             // seconds
  measurementRate: number;      // Hz
  sensitivity: number;          // fractional (e.g., 1e-9)
  absoluteAccuracy: number;     // fractional
  spatialResolution: number;    // meters
  dynamicRange: [number, number]; // [min, max] × g
  operatingTemp: number;        // Kelvin
  powerConsumption: number;     // Watts
}

/**
 * Gravity gradient
 */
export interface GravityGradient {
  dg_dz: number;    // Vertical gradient (1/s²)
  dg_dx: number;    // East-West gradient
  dg_dy: number;    // North-South gradient
}

/**
 * Gravimeter measurement result
 */
export interface GravimeterMeasurement {
  measurementId: string;
  timestamp: Date | string;
  gravity: {
    value: number;
    uncertainty: number;
    unit: 'm/s²';
  };
  location: GeoLocation;
  gradient?: GravityGradient;
  tides?: {
    correction: number;
    model: string;
  };
  quality: QualityMetrics;
}

// ============================================================================
// Accelerometer & Gyroscope Types
// ============================================================================

/**
 * Accelerometer configuration
 */
export interface AccelerometerConfig extends SensorConfig {
  sensorType: 'accelerometer';
  type: 'atom-interferometer' | 'optomechanical' | 'mems-quantum';
  axes: 1 | 3;
  sensitivity: number;          // g/√Hz
  bandwidth: [number, number];  // Hz
  dynamicRange: [number, number]; // [min, max] in g
  biasStability: number;        // g
  noiseDensity: number;         // g/√Hz
}

/**
 * Gyroscope configuration
 */
export interface GyroscopeConfig extends SensorConfig {
  sensorType: 'gyroscope';
  type: 'atom-interferometer' | 'nuclear-spin' | 'sagnac';
  axes: 1 | 3;
  sensitivity: number;          // rad/s/√Hz
  biasStability: number;        // rad/s
  scaleFactorStability: number; // fractional
  enclosedArea?: number;        // m² (for Sagnac)
}

/**
 * Inertial measurement unit (IMU) combining accelerometers and gyroscopes
 */
export interface QuantumIMU {
  accelerometers: {
    x: { bias: number; noiseDensity: number };
    y: { bias: number; noiseDensity: number };
    z: { bias: number; noiseDensity: number };
  };
  gyroscopes: {
    x: { bias: number; noiseDensity: number };
    y: { bias: number; noiseDensity: number };
    z: { bias: number; noiseDensity: number };
  };
  updateRate: number;           // Hz
  alignmentAccuracy: number;    // radians
  powerConsumption: number;     // Watts
}

/**
 * Inertial measurement result
 */
export interface InertialMeasurement {
  measurementId: string;
  timestamp: Date | string;
  acceleration: Vector3;        // m/s²
  angularVelocity: Vector3;     // rad/s
  uncertainty: {
    acceleration: Vector3;
    angularVelocity: Vector3;
  };
  quality: QualityMetrics;
}

// ============================================================================
// Quantum Imaging Types
// ============================================================================

/**
 * Quantum imaging modes
 */
export type ImagingMode =
  | 'ghost'
  | 'sub-shot-noise'
  | 'squeezed'
  | 'quantum-illumination';

/**
 * Image resolution
 */
export interface ImageResolution {
  x: number;          // pixels
  y: number;          // pixels
  pixelSize: number;  // meters
}

/**
 * Quantum imaging configuration
 */
export interface QuantumImagingConfig extends SensorConfig {
  sensorType: 'imaging';
  mode: ImagingMode;
  resolution: ImageResolution;
  wavelength: number;         // meters
  photonBudget: number;       // photons per measurement
  squeezingDb?: number;       // dB of squeezing
  entanglementSource?: string;
}

/**
 * Quantum imaging result
 */
export interface QuantumImageMeasurement {
  imageId: string;
  type: ImagingMode;
  resolution: ImageResolution;
  photonCount: number;
  integrationTime: number;    // milliseconds
  squeezingDb?: number;
  snrEnhancement: number;
  wavelength: number;
  data: string;               // base64 encoded image
  metadata?: Record<string, any>;
}

// ============================================================================
// Quantum Radar Types
// ============================================================================

/**
 * Radar target information
 */
export interface RadarTarget {
  detected: boolean;
  range: number;              // meters
  velocity: number;           // m/s
  azimuth: number;            // degrees
  elevation: number;          // degrees
  radarCrossSection: number;  // m²
  confidence?: number;        // 0-1
}

/**
 * Quantum radar configuration
 */
export interface QuantumRadarConfig extends SensorConfig {
  sensorType: 'radar';
  type: 'microwave' | 'lidar';
  frequency: number;          // Hz
  power: number;              // Watts
  bandwidth: number;          // Hz
  entanglementVisibility: number; // 0-1
  detectionRange: number;     // meters
  resolutionRange: number;    // meters
  resolutionVelocity: number; // m/s
}

/**
 * Quantum radar measurement result
 */
export interface QuantumRadarMeasurement {
  detectionId: string;
  timestamp: Date | string;
  target: RadarTarget;
  quantumCorrelation: number; // 0-1
  snr: number;                // dB
  falseAlarmRate: number;     // probability
  entanglementVisibility: number; // 0-1
  quality: QualityMetrics;
}

// ============================================================================
// Measurement Request/Response
// ============================================================================

/**
 * Generic measurement request
 */
export interface MeasurementRequest {
  sensorId: string;
  integrationTime: number;      // milliseconds
  repetitions?: number;
  bandwidth?: [number, number]; // [low, high] Hz
  outputFormat?: 'raw' | 'calibrated' | 'processed';
  metadata?: Record<string, any>;
}

/**
 * Generic measurement result
 */
export interface MeasurementResult {
  measurementId: string;
  sensorId: string;
  sensorType: SensorType;
  timestamp: Date | string;
  value: number | number[] | Vector3 | MagneticField;
  uncertainty: number | Uncertainty;
  unit: string;
  quality: QualityMetrics;
  metadata?: Record<string, any>;
}

/**
 * Time series measurement data
 */
export interface TimeSeriesData {
  sensorId: string;
  sensorType: SensorType;
  startTime: Date | string;
  sampleRate: number;         // Hz
  unit: string;
  data: number[];
  timestamps: string[];
  uncertainties: number[];
  quality?: QualityMetrics;
}

// ============================================================================
// Calibration Types
// ============================================================================

/**
 * Calibration procedure
 */
export interface CalibrationProcedure {
  procedureId: string;
  sensorType: SensorType;
  method: string;
  referenceStandard: string;
  steps: CalibrationStep[];
}

/**
 * Calibration step
 */
export interface CalibrationStep {
  stepNumber: number;
  description: string;
  expectedValue?: number;
  tolerance?: number;
  duration?: number;          // seconds
}

/**
 * Calibration result
 */
export interface CalibrationResult {
  calibrationId: string;
  sensorId: string;
  timestamp: Date | string;
  procedure: string;
  traceability: string;
  results: {
    parameter: string;
    measuredValue: number;
    referenceValue: number;
    deviation: number;
    pass: boolean;
  }[];
  uncertainty: Uncertainty;
  nextCalibrationDue: Date | string;
  certifiedBy?: string;
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Physical constants used in quantum sensing
 */
export const QUANTUM_CONSTANTS = {
  /** Planck constant (J·s) */
  PLANCK_CONSTANT: 6.62607015e-34,

  /** Reduced Planck constant (J·s) */
  REDUCED_PLANCK: 1.054571817e-34,

  /** Elementary charge (C) */
  ELEMENTARY_CHARGE: 1.602176634e-19,

  /** Magnetic flux quantum (Wb) */
  FLUX_QUANTUM: 2.067833848e-15,

  /** Cesium-133 hyperfine transition frequency (Hz) */
  CESIUM_FREQUENCY: 9192631770,

  /** Standard gravity (m/s²) */
  STANDARD_GRAVITY: 9.80665,

  /** Speed of light (m/s) */
  SPEED_OF_LIGHT: 299792458,

  /** Boltzmann constant (J/K) */
  BOLTZMANN: 1.380649e-23,

  /** Bohr magneton (J/T) */
  BOHR_MAGNETON: 9.2740100783e-24,
} as const;

/**
 * Gyromagnetic ratios for common atoms (Hz/T)
 */
export const GYROMAGNETIC_RATIOS = {
  'cesium-133': 3.5e9,
  'rubidium-87': 7.0e9,
  'hydrogen': 42.576e6,
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-QUA-004 error codes
 */
export enum QuantumSensorErrorCode {
  SENSOR_NOT_INITIALIZED = 'QS001',
  CALIBRATION_EXPIRED = 'QS002',
  ENVIRONMENTAL_OUT_OF_RANGE = 'QS003',
  LOW_SNR = 'QS004',
  COHERENCE_LOSS = 'QS005',
  HARDWARE_MALFUNCTION = 'QS006',
  INVALID_CONFIGURATION = 'QS007',
  MEASUREMENT_TIMEOUT = 'QS008',
  DATA_CORRUPTION = 'QS009',
}

/**
 * Quantum sensor error
 */
export class QuantumSensorError extends Error {
  constructor(
    public code: QuantumSensorErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'QuantumSensorError';
  }
}

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

// ============================================================================
// Export All Types
// ============================================================================

export type {
  // Core
  Vector3,
  GeoLocation,
  QualityMetrics,
  Uncertainty,

  // Sensor config
  SensorType,
  SensorConfig,
  SensorInit,

  // Atomic clocks
  AtomType,
  TransitionType,
  AtomicClockConfig,
  AtomicClockMeasurement,

  // Magnetometers
  MagnetometerType,
  SQUIDType,
  MagnetometerConfig,
  MagneticField,
  MagnetometerMeasurement,

  // Gravimeters
  GravimeterConfiguration,
  GravimeterConfig,
  GravityGradient,
  GravimeterMeasurement,

  // Accelerometers & Gyroscopes
  AccelerometerConfig,
  GyroscopeConfig,
  QuantumIMU,
  InertialMeasurement,

  // Imaging
  ImagingMode,
  ImageResolution,
  QuantumImagingConfig,
  QuantumImageMeasurement,

  // Radar
  RadarTarget,
  QuantumRadarConfig,
  QuantumRadarMeasurement,

  // Measurement
  MeasurementRequest,
  MeasurementResult,
  TimeSeriesData,

  // Calibration
  CalibrationProcedure,
  CalibrationStep,
  CalibrationResult,
};

export {
  QUANTUM_CONSTANTS,
  GYROMAGNETIC_RATIOS,
  QuantumSensorErrorCode,
  QuantumSensorError,
};

/**
 * 弘익人間 (홍익인간) · Benefit All Humanity
 */
