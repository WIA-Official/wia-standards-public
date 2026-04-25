/**
 * WIA-BIO-011: Biosensor - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Biotechnology Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Biosensor Types
// ============================================================================

/**
 * Biosensor types based on transduction mechanism
 */
export type BiosensorType =
  | 'electrochemical'
  | 'optical'
  | 'piezoelectric'
  | 'thermal'
  | 'impedance'
  | 'calorimetric';

/**
 * Electrochemical sensor subtypes
 */
export type ElectrochemicalType =
  | 'amperometric'
  | 'potentiometric'
  | 'voltammetric'
  | 'conductometric'
  | 'impedimetric';

/**
 * Optical sensor subtypes
 */
export type OpticalType =
  | 'absorbance'
  | 'fluorescence'
  | 'luminescence'
  | 'spr'
  | 'fret'
  | 'colorimetric';

/**
 * Biorecognition element types
 */
export type BiorecognitionElement =
  | 'enzyme'
  | 'antibody'
  | 'aptamer'
  | 'dna'
  | 'rna'
  | 'whole-cell'
  | 'bacteriophage'
  | 'molecularly-imprinted-polymer';

/**
 * Common enzymes for biosensors
 */
export type EnzymeType =
  | 'glucose-oxidase'
  | 'lactate-oxidase'
  | 'cholesterol-oxidase'
  | 'urease'
  | 'acetylcholinesterase'
  | 'horseradish-peroxidase'
  | 'alkaline-phosphatase';

/**
 * Target analyte categories
 */
export type AnalyteCategory =
  | 'metabolite'
  | 'protein'
  | 'nucleic-acid'
  | 'pathogen'
  | 'toxin'
  | 'drug'
  | 'ion'
  | 'cell';

// ============================================================================
// Biosensor Configuration
// ============================================================================

/**
 * Biosensor configuration
 */
export interface BiosensorConfig {
  /** Sensor type */
  type: BiosensorType;

  /** Electrochemical subtype (if applicable) */
  electrochemicalType?: ElectrochemicalType;

  /** Optical subtype (if applicable) */
  opticalType?: OpticalType;

  /** Biorecognition element */
  biorecognitionElement: BiorecognitionElement | EnzymeType | string;

  /** Target analyte */
  targetAnalyte: string;

  /** Analyte category */
  analyteCategory?: AnalyteCategory;

  /** Working electrode material (electrochemical) */
  workingElectrode?: 'platinum' | 'gold' | 'carbon' | 'silver' | 'glassy-carbon';

  /** Reference electrode (electrochemical) */
  referenceElectrode?: 'ag-agcl' | 'sce' | 'calomel' | 'pseudo-reference';

  /** Counter electrode (electrochemical) */
  counterElectrode?: 'platinum' | 'gold' | 'carbon';

  /** Applied potential (V) for amperometric sensors */
  appliedPotential?: number;

  /** Wavelength (nm) for optical sensors */
  wavelength?: {
    excitation?: number;
    emission?: number;
  };

  /** Sample type */
  sampleType?: 'blood' | 'serum' | 'plasma' | 'urine' | 'saliva' | 'water' | 'food' | 'environmental';

  /** Temperature control */
  temperature?: number; // °C

  /** pH optimization */
  pH?: number;
}

/**
 * Biosensor instance
 */
export interface Biosensor {
  /** Unique sensor identifier */
  id: string;

  /** Sensor configuration */
  config: BiosensorConfig;

  /** Calibration data */
  calibration?: CalibrationData;

  /** Performance metrics */
  performance?: PerformanceMetrics;

  /** Sensor status */
  status: 'active' | 'calibrating' | 'measuring' | 'idle' | 'error' | 'expired';

  /** Creation timestamp */
  created: Date;

  /** Last measurement timestamp */
  lastMeasurement?: Date;

  /** Number of measurements performed */
  measurementCount: number;

  /** Sensor lifetime */
  lifetime?: {
    /** Expiration date */
    expirationDate: Date;
    /** Maximum measurements */
    maxMeasurements: number;
    /** Storage conditions */
    storageConditions: string;
  };
}

// ============================================================================
// Calibration
// ============================================================================

/**
 * Calibration data point
 */
export interface CalibrationPoint {
  /** Standard concentration */
  concentration: number;

  /** Measured signal */
  signal: number;

  /** Signal unit */
  signalUnit: string;

  /** Concentration unit */
  concentrationUnit: string;

  /** Replicate measurements */
  replicates?: number[];

  /** Standard deviation */
  stdDev?: number;
}

/**
 * Calibration data
 */
export interface CalibrationData {
  /** Calibration ID */
  id: string;

  /** Calibration points */
  points: CalibrationPoint[];

  /** Calibration type */
  type: 'linear' | 'polynomial' | 'logarithmic' | '4pl' | 'michaelis-menten';

  /** Regression parameters */
  regression: {
    /** Slope (linear) or coefficients (polynomial) */
    slope?: number;
    /** Intercept */
    intercept?: number;
    /** Polynomial coefficients [a, b, c, ...] */
    coefficients?: number[];
    /** 4PL parameters */
    fourPL?: {
      a: number; // min asymptote
      d: number; // max asymptote
      c: number; // IC50
      b: number; // Hill slope
    };
  };

  /** Coefficient of determination */
  rSquared: number;

  /** Sensitivity (slope of calibration curve) */
  sensitivity: number;

  /** Sensitivity unit */
  sensitivityUnit: string;

  /** Linear range */
  linearRange: {
    min: number;
    max: number;
    unit: string;
  };

  /** Calibration date */
  calibratedAt: Date;

  /** Calibration validity */
  validUntil?: Date;

  /** Calibration performed by */
  operator?: string;

  /** Quality control */
  qualityControl?: {
    passed: boolean;
    residuals: number[];
    maxResidual: number;
  };
}

/**
 * Calibration request
 */
export interface CalibrationRequest {
  /** Sensor ID */
  sensorId: string;

  /** Standard concentrations */
  standardConcentrations: number[];

  /** Measured signals */
  measuredSignals: number[];

  /** Concentration unit */
  concentrationUnit?: string;

  /** Signal unit */
  signalUnit?: string;

  /** Calibration type */
  calibrationType?: CalibrationData['type'];

  /** Force recalibration */
  forceRecalibration?: boolean;
}

// ============================================================================
// Performance Metrics
// ============================================================================

/**
 * Performance metrics
 */
export interface PerformanceMetrics {
  /** Limit of Detection (LOD) */
  lod: number;

  /** Limit of Quantification (LOQ) */
  loq: number;

  /** Dynamic range (orders of magnitude) */
  dynamicRange: number;

  /** Concentration unit */
  concentrationUnit: string;

  /** Response time (seconds) */
  responseTime: number;

  /** Selectivity coefficients */
  selectivity?: {
    [interferent: string]: number;
  };

  /** Accuracy (%) */
  accuracy?: number;

  /** Precision (CV %) */
  precision?: number;

  /** Stability metrics */
  stability?: {
    /** Shelf life (days) */
    shelfLife: number;
    /** Operational stability (measurements) */
    operationalStability: number;
    /** Long-term drift (%/month) */
    drift: number;
  };

  /** Measurement date */
  measuredAt: Date;
}

/**
 * LOD calculation parameters
 */
export interface LODCalculation {
  /** Blank measurements */
  blankSignals: number[];

  /** Sensitivity from calibration */
  sensitivity: number;

  /** Method */
  method?: 'iupac' | 'epa' | '3-sigma';

  /** Confidence level */
  confidenceLevel?: number; // 0.95, 0.99
}

/**
 * LOD calculation result
 */
export interface LODResult {
  /** Limit of Detection */
  lod: number;

  /** Limit of Quantification */
  loq: number;

  /** Blank mean signal */
  blankMean: number;

  /** Blank standard deviation */
  blankStdDev: number;

  /** Signal-to-noise ratio at LOD */
  snrAtLOD: number;

  /** Unit */
  unit: string;

  /** Calculation method */
  method: string;
}

// ============================================================================
// Measurements
// ============================================================================

/**
 * Measurement request
 */
export interface MeasurementRequest {
  /** Sensor ID */
  sensorId: string;

  /** Measured signal */
  signal: number;

  /** Signal unit */
  signalUnit?: string;

  /** Temperature (°C) */
  temperature?: number;

  /** pH */
  pH?: number;

  /** Sample ID */
  sampleId?: string;

  /** Sample type */
  sampleType?: string;

  /** Use specific calibration */
  calibration?: CalibrationData;

  /** Replicate number */
  replicateNumber?: number;
}

/**
 * Measurement result
 */
export interface MeasurementResult {
  /** Measurement ID */
  id: string;

  /** Sensor ID */
  sensorId: string;

  /** Measured concentration */
  concentration: number;

  /** Concentration unit */
  concentrationUnit: string;

  /** Raw signal */
  signal: number;

  /** Signal unit */
  signalUnit: string;

  /** Corrected signal (baseline, temperature) */
  correctedSignal?: number;

  /** Confidence/uncertainty */
  confidence: number;

  /** Standard error */
  standardError?: number;

  /** Quality flags */
  qualityFlags: QualityFlag[];

  /** Within linear range? */
  withinRange: boolean;

  /** Temperature at measurement */
  temperature?: number;

  /** pH at measurement */
  pH?: number;

  /** Timestamp */
  timestamp: Date;

  /** Sample information */
  sample?: {
    id: string;
    type: string;
    metadata?: Record<string, unknown>;
  };

  /** Operator */
  operator?: string;
}

/**
 * Quality flags
 */
export type QualityFlag =
  | 'ok'
  | 'below-lod'
  | 'below-loq'
  | 'above-range'
  | 'drift-detected'
  | 'temperature-out-of-range'
  | 'calibration-expired'
  | 'sensor-expired'
  | 'high-noise'
  | 'interference-detected';

/**
 * Batch measurement
 */
export interface BatchMeasurement {
  /** Batch ID */
  id: string;

  /** Sensor ID */
  sensorId: string;

  /** Measurements */
  measurements: MeasurementResult[];

  /** Quality control results */
  qualityControl?: QualityControlResult[];

  /** Batch statistics */
  statistics?: {
    mean: number;
    median: number;
    stdDev: number;
    cv: number;
    min: number;
    max: number;
  };

  /** Started at */
  startedAt: Date;

  /** Completed at */
  completedAt: Date;
}

// ============================================================================
// Quality Control
// ============================================================================

/**
 * Quality control sample
 */
export interface QualityControlSample {
  /** QC level */
  level: 'blank' | 'low' | 'medium' | 'high';

  /** Target concentration */
  targetConcentration: number;

  /** Acceptable range */
  acceptableRange: {
    min: number;
    max: number;
  };

  /** Historical mean */
  historicalMean?: number;

  /** Historical standard deviation */
  historicalStdDev?: number;

  /** Lot number */
  lotNumber?: string;

  /** Expiration date */
  expirationDate?: Date;
}

/**
 * Quality control result
 */
export interface QualityControlResult {
  /** QC sample */
  sample: QualityControlSample;

  /** Measured concentration */
  measuredConcentration: number;

  /** Within acceptable range? */
  passed: boolean;

  /** Deviation from target (%) */
  deviation: number;

  /** Z-score (if historical data available) */
  zScore?: number;

  /** Timestamp */
  timestamp: Date;

  /** Corrective action required */
  correctiveAction?: string;
}

// ============================================================================
// Signal Processing
// ============================================================================

/**
 * Signal processing parameters
 */
export interface SignalProcessingParams {
  /** Raw signal */
  rawSignal: number;

  /** Baseline correction */
  baseline?: number;

  /** Temperature correction */
  temperatureCorrection?: {
    temperature: number;
    referenceTemperature: number;
    coefficient: number; // %/°C
  };

  /** Noise filtering */
  filtering?: {
    method: 'moving-average' | 'savitzky-golay' | 'low-pass' | 'median';
    windowSize?: number;
    polynomialOrder?: number;
  };

  /** Drift compensation */
  driftCompensation?: {
    driftRate: number; // signal/hour
    timeSinceCalibration: number; // hours
  };
}

/**
 * Processed signal result
 */
export interface ProcessedSignal {
  /** Original signal */
  original: number;

  /** Baseline-corrected */
  baselineCorrected?: number;

  /** Temperature-corrected */
  temperatureCorrected?: number;

  /** Filtered signal */
  filtered?: number;

  /** Drift-compensated */
  driftCompensated?: number;

  /** Final processed signal */
  final: number;

  /** Processing steps applied */
  stepsApplied: string[];

  /** Signal-to-noise ratio */
  snr?: number;
}

// ============================================================================
// Validation
// ============================================================================

/**
 * Method validation parameters
 */
export interface ValidationParameters {
  /** Sensor ID */
  sensorId: string;

  /** Linearity test */
  linearity?: {
    concentrations: number[];
    signals: number[];
    requiredR2: number; // typically 0.99
  };

  /** Accuracy test */
  accuracy?: {
    spikedSamples: Array<{
      expectedConcentration: number;
      measuredConcentration: number;
    }>;
    requiredRecovery: [number, number]; // e.g., [90, 110] %
  };

  /** Precision test */
  precision?: {
    replicates: number[];
    requiredCV: number; // e.g., 5 %
  };

  /** Selectivity test */
  selectivity?: {
    interferents: Array<{
      name: string;
      concentration: number;
      signal: number;
    }>;
    targetSignal: number;
    requiredSelectivity: number; // e.g., 100:1
  };

  /** Stability test */
  stability?: {
    measurements: Array<{
      time: number; // hours
      signal: number;
    }>;
    requiredStability: number; // % signal retained
  };
}

/**
 * Validation result
 */
export interface ValidationResult {
  /** Sensor ID */
  sensorId: string;

  /** Overall validation status */
  validated: boolean;

  /** Linearity result */
  linearity?: {
    passed: boolean;
    rSquared: number;
    requiredR2: number;
  };

  /** Accuracy result */
  accuracy?: {
    passed: boolean;
    recovery: number; // %
    requiredRange: [number, number];
  };

  /** Precision result */
  precision?: {
    passed: boolean;
    cv: number; // %
    requiredCV: number;
  };

  /** Selectivity result */
  selectivity?: {
    passed: boolean;
    coefficients: { [interferent: string]: number };
    requiredSelectivity: number;
  };

  /** Stability result */
  stability?: {
    passed: boolean;
    retentionRate: number; // %
    halfLife?: number; // hours
  };

  /** Validation date */
  validatedAt: Date;

  /** Validated by */
  operator?: string;

  /** Comments */
  comments?: string;
}

// ============================================================================
// Point-of-Care
// ============================================================================

/**
 * Point-of-care test result
 */
export interface POCTestResult {
  /** Test ID */
  id: string;

  /** Test type */
  testType: 'glucose' | 'cardiac-marker' | 'infectious-disease' | 'pregnancy' | 'drug-test' | 'other';

  /** Analyte */
  analyte: string;

  /** Result value */
  value: number;

  /** Unit */
  unit: string;

  /** Clinical interpretation */
  interpretation: 'normal' | 'abnormal' | 'borderline' | 'positive' | 'negative' | 'indeterminate';

  /** Reference range */
  referenceRange?: {
    min: number;
    max: number;
    unit: string;
  };

  /** Clinical cutoff */
  cutoff?: number;

  /** Test timestamp */
  timestamp: Date;

  /** Patient information */
  patient?: {
    id: string;
    age?: number;
    sex?: 'M' | 'F' | 'Other';
  };

  /** Quality control */
  qualityControl: {
    passed: boolean;
    flags: QualityFlag[];
  };

  /** Operator */
  operator?: string;

  /** Device information */
  device?: {
    serialNumber: string;
    lotNumber: string;
    expirationDate: Date;
  };
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Physical and biochemical constants
 */
export const BIOSENSOR_CONSTANTS = {
  /** Faraday constant (C/mol) */
  FARADAY_CONSTANT: 96485,

  /** Gas constant (J/mol·K) */
  GAS_CONSTANT: 8.314,

  /** Standard temperature (K) */
  STANDARD_TEMPERATURE: 298.15, // 25°C

  /** Nernst slope at 25°C (mV/decade) */
  NERNST_SLOPE: 59.16,

  /** Typical temperature coefficient for enzymes (%/°C) */
  ENZYME_TEMP_COEFFICIENT: 3.0,

  /** Typical pH optimum for glucose oxidase */
  GOX_PH_OPTIMUM: 7.0,

  /** Typical Km for glucose oxidase (mM) */
  GOX_KM: 33.0,

  /** Standard calibration points */
  STANDARD_CALIBRATION_POINTS: 5,

  /** Minimum R² for calibration */
  MIN_R_SQUARED: 0.99,

  /** LOD multiplier (IUPAC) */
  LOD_MULTIPLIER: 3.3,

  /** LOQ multiplier */
  LOQ_MULTIPLIER: 10.0,

  /** Maximum acceptable CV (%) */
  MAX_CV_PERCENT: 5.0,

  /** Minimum acceptable recovery (%) */
  MIN_RECOVERY: 90.0,

  /** Maximum acceptable recovery (%) */
  MAX_RECOVERY: 110.0,
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
 * Time series data point
 */
export interface TimeSeriesPoint {
  /** Timestamp */
  time: Date;

  /** Value */
  value: number;

  /** Unit */
  unit: string;

  /** Quality flag */
  quality?: QualityFlag;
}

/**
 * Time series data
 */
export interface TimeSeries {
  /** Series ID */
  id: string;

  /** Sensor ID */
  sensorId: string;

  /** Analyte */
  analyte: string;

  /** Data points */
  data: TimeSeriesPoint[];

  /** Metadata */
  metadata?: {
    patientId?: string;
    studyId?: string;
    [key: string]: unknown;
  };
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-BIO-011 error codes
 */
export enum BiosensorErrorCode {
  SENSOR_NOT_FOUND = 'BIO001',
  INVALID_CONFIGURATION = 'BIO002',
  CALIBRATION_FAILED = 'BIO003',
  CALIBRATION_EXPIRED = 'BIO004',
  MEASUREMENT_OUT_OF_RANGE = 'BIO005',
  BELOW_LOD = 'BIO006',
  SENSOR_EXPIRED = 'BIO007',
  TEMPERATURE_OUT_OF_RANGE = 'BIO008',
  QC_FAILED = 'BIO009',
  INSUFFICIENT_CALIBRATION_POINTS = 'BIO010',
  POOR_LINEARITY = 'BIO011',
  INVALID_SIGNAL = 'BIO012',
  SENSOR_DEGRADED = 'BIO013',
}

/**
 * Biosensor error
 */
export class BiosensorError extends Error {
  constructor(
    public code: BiosensorErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'BiosensorError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Sensor types
  BiosensorType,
  ElectrochemicalType,
  OpticalType,
  BiorecognitionElement,
  EnzymeType,
  AnalyteCategory,

  // Configuration
  BiosensorConfig,
  Biosensor,

  // Calibration
  CalibrationPoint,
  CalibrationData,
  CalibrationRequest,

  // Performance
  PerformanceMetrics,
  LODCalculation,
  LODResult,

  // Measurements
  MeasurementRequest,
  MeasurementResult,
  QualityFlag,
  BatchMeasurement,

  // Quality Control
  QualityControlSample,
  QualityControlResult,

  // Signal Processing
  SignalProcessingParams,
  ProcessedSignal,

  // Validation
  ValidationParameters,
  ValidationResult,

  // Point-of-Care
  POCTestResult,

  // Time Series
  TimeSeriesPoint,
  TimeSeries,
};

export { BIOSENSOR_CONSTANTS, BiosensorErrorCode, BiosensorError };
