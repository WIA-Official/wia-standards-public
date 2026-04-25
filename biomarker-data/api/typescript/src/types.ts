/**
 * WIA-BIO-004: Biomarker Data - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Biomarker Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Biomarker Types
// ============================================================================

/**
 * Biomarker classification types
 */
export type BiomarkerType = 'protein' | 'genetic' | 'metabolic' | 'imaging' | 'cellular';

/**
 * Measurement methods
 */
export type MeasurementMethod =
  | 'ELISA'
  | 'mass_spectrometry'
  | 'PCR'
  | 'sequencing'
  | 'immunoassay'
  | 'clinical_chemistry'
  | 'flow_cytometry'
  | 'imaging'
  | 'other';

/**
 * Sample types
 */
export type SampleType =
  | 'blood_serum'
  | 'blood_plasma'
  | 'whole_blood'
  | 'urine'
  | 'tissue_biopsy'
  | 'cerebrospinal_fluid'
  | 'saliva'
  | 'stool';

/**
 * Disease status
 */
export type DiseaseStatus = 'diseased' | 'healthy' | 'at_risk' | 'unknown';

/**
 * Clinical utility levels
 */
export type ClinicalUtility = 'excellent' | 'good' | 'fair' | 'poor' | 'insufficient';

/**
 * Biomarker definition
 */
export interface Biomarker {
  /** Unique biomarker identifier */
  id: string;

  /** Biomarker name */
  name: string;

  /** Biomarker type classification */
  type: BiomarkerType;

  /** Description */
  description?: string;

  /** Associated genes (for genetic biomarkers) */
  genes?: string[];

  /** Associated proteins (for protein biomarkers) */
  proteins?: string[];

  /** Associated metabolites (for metabolic biomarkers) */
  metabolites?: string[];

  /** Clinical indications */
  indications: string[];

  /** Reference range */
  referenceRange?: ReferenceRange;
}

/**
 * Reference range for biomarker
 */
export interface ReferenceRange {
  /** Minimum value */
  min: number;

  /** Maximum value */
  max: number;

  /** Unit of measurement */
  unit: string;

  /** Median or typical value */
  median?: number;

  /** Age group (if applicable) */
  ageGroup?: string;

  /** Sex (if applicable) */
  sex?: 'male' | 'female' | 'all';

  /** Population (if applicable) */
  population?: string;
}

// ============================================================================
// Measurement Data
// ============================================================================

/**
 * Biomarker measurement
 */
export interface BiomarkerMeasurement {
  /** Measurement ID */
  id: string;

  /** Biomarker ID */
  biomarkerID: string;

  /** Measured value */
  value: number;

  /** Unit of measurement */
  unit: string;

  /** Patient/sample ID */
  patientID?: string;

  /** Disease status */
  diseaseStatus?: boolean | DiseaseStatus;

  /** Measurement method */
  method?: MeasurementMethod;

  /** Sample type */
  sampleType?: SampleType;

  /** Collection date */
  dateCollected?: Date | string;

  /** Measurement date */
  dateMeasured?: Date | string;

  /** Laboratory ID */
  laboratoryID?: string;

  /** Quality control flags */
  qualityFlags?: QualityFlags;

  /** Additional metadata */
  metadata?: Record<string, any>;
}

/**
 * Quality control flags
 */
export interface QualityFlags {
  /** Hemolyzed sample */
  hemolyzed?: boolean;

  /** Lipemic sample */
  lipemic?: boolean;

  /** Icteric sample */
  icteric?: boolean;

  /** Clotted sample */
  clotted?: boolean;

  /** Diluted sample */
  diluted?: boolean;

  /** Out of range */
  outOfRange?: boolean;

  /** Other quality issues */
  other?: string[];
}

/**
 * Patient sample information
 */
export interface PatientSample {
  /** Patient ID (anonymized) */
  patientID: string;

  /** Age in years */
  age?: number;

  /** Sex */
  sex?: 'male' | 'female' | 'other';

  /** Ethnicity */
  ethnicity?: string;

  /** Fasting status */
  fastingStatus?: boolean;

  /** Current medications */
  medications?: string[];

  /** Medical history */
  medicalHistory?: string[];

  /** Disease status */
  diseaseStatus?: DiseaseStatus;

  /** Disease stage (if applicable) */
  diseaseStage?: string;

  /** Sample collection details */
  sampleDetails?: {
    type: SampleType;
    collectionTime: Date | string;
    collectionMethod?: string;
    storageConditions?: string;
  };
}

/**
 * Laboratory result
 */
export interface LabResult {
  /** Result ID */
  id: string;

  /** Biomarker measurements */
  measurements: BiomarkerMeasurement[];

  /** Patient information */
  patient: PatientSample;

  /** Clinical context */
  clinicalContext?: {
    indication: string;
    orderingPhysician?: string;
    facility?: string;
    urgency?: 'routine' | 'urgent' | 'stat';
  };

  /** Quality assurance */
  qualityAssurance?: {
    internalQC: boolean;
    externalQC: boolean;
    calibrationStatus: 'valid' | 'expired' | 'unknown';
    instrumentID?: string;
  };
}

// ============================================================================
// Validation and Performance
// ============================================================================

/**
 * ROC analysis request
 */
export interface ROCRequest {
  /** True labels (1 = diseased, 0 = healthy) */
  trueLabels: number[];

  /** Predictions (biomarker values or probabilities) */
  predictions: number[];

  /** Optional specific thresholds to evaluate */
  thresholds?: number[];

  /** Calculate confidence intervals */
  calculateCI?: boolean;

  /** Confidence level (default: 0.95) */
  confidenceLevel?: number;
}

/**
 * ROC point on curve
 */
export interface ROCPoint {
  /** Threshold value */
  threshold: number;

  /** Sensitivity (True Positive Rate) */
  sensitivity: number;

  /** Specificity */
  specificity: number;

  /** False Positive Rate */
  fpr: number;

  /** True Positive Rate */
  tpr: number;

  /** Youden Index */
  youdenIndex: number;
}

/**
 * ROC analysis result
 */
export interface ROCResult {
  /** Area Under Curve */
  auc: number;

  /** AUC 95% confidence interval */
  aucCI95?: [number, number];

  /** ROC curve points */
  rocCurve: ROCPoint[];

  /** Optimal threshold by Youden Index */
  optimalThreshold: {
    value: number;
    sensitivity: number;
    specificity: number;
    youdenIndex: number;
  };

  /** Number of diseased */
  nDiseased: number;

  /** Number of healthy */
  nHealthy: number;
}

/**
 * Diagnostic performance metrics
 */
export interface DiagnosticPerformance {
  /** Sensitivity (%) */
  sensitivity: number;

  /** Sensitivity 95% CI */
  sensitivityCI95?: [number, number];

  /** Specificity (%) */
  specificity: number;

  /** Specificity 95% CI */
  specificityCI95?: [number, number];

  /** Positive Predictive Value (%) */
  ppv: number;

  /** Negative Predictive Value (%) */
  npv: number;

  /** Accuracy (%) */
  accuracy: number;

  /** Positive Likelihood Ratio */
  lrPositive: number;

  /** Negative Likelihood Ratio */
  lrNegative: number;

  /** Confusion matrix */
  confusionMatrix: {
    tp: number; // True Positives
    tn: number; // True Negatives
    fp: number; // False Positives
    fn: number; // False Negatives
  };

  /** Prevalence in study population */
  prevalence?: number;
}

/**
 * Biomarker validation parameters
 */
export interface BiomarkerValidation {
  /** Biomarker type */
  biomarkerType: BiomarkerType;

  /** Measurements with disease status */
  measurements: Array<{
    value: number;
    unit: string;
    diseaseStatus: boolean;
    patientID?: string;
  }>;

  /** Reference range */
  referenceRange: {
    min: number;
    max: number;
    unit: string;
  };

  /** Optional cutoff threshold */
  cutoffThreshold?: number;

  /** Disease prevalence (for PPV/NPV calculation) */
  prevalence?: number;
}

/**
 * Validation result
 */
export interface ValidationResult {
  /** Is validation successful */
  isValid: boolean;

  /** Diagnostic performance */
  performance: DiagnosticPerformance;

  /** ROC analysis */
  rocAnalysis?: ROCResult;

  /** Recommended cutoff */
  recommendedCutoff: number;

  /** Clinical utility assessment */
  clinicalUtility: ClinicalUtility;

  /** Validation warnings */
  warnings: string[];

  /** Validation errors */
  errors: string[];

  /** Sample size adequacy */
  sampleSizeAdequate: boolean;
}

// ============================================================================
// Clinical Correlation
// ============================================================================

/**
 * Clinical correlation analysis
 */
export interface ClinicalCorrelation {
  /** Correlation ID */
  id: string;

  /** Biomarker ID */
  biomarkerID: string;

  /** Clinical outcome */
  outcome: ClinicalOutcome;

  /** Correlation coefficient */
  correlationCoefficient: number;

  /** P-value */
  pValue: number;

  /** Correlation type */
  correlationType: 'pearson' | 'spearman' | 'kendall';

  /** Sample size */
  sampleSize: number;

  /** Is statistically significant */
  isSignificant: boolean;
}

/**
 * Clinical outcome
 */
export interface ClinicalOutcome {
  /** Outcome type */
  type: 'survival' | 'progression' | 'response' | 'recurrence' | 'severity' | 'other';

  /** Outcome name */
  name: string;

  /** Measurement */
  measurement?: 'continuous' | 'binary' | 'ordinal' | 'time-to-event';

  /** Values */
  values?: number[];

  /** Time points (for longitudinal data) */
  timePoints?: Date[] | string[];
}

/**
 * Prognostic analysis
 */
export interface PrognosticAnalysis {
  /** Biomarker ID */
  biomarkerID: string;

  /** Hazard ratio */
  hazardRatio: number;

  /** HR 95% confidence interval */
  hazardRatioCI95: [number, number];

  /** P-value */
  pValue: number;

  /** C-index (concordance index) */
  cIndex?: number;

  /** Survival curves */
  survivalCurves?: {
    high: number[]; // Survival probabilities for high biomarker
    low: number[];  // Survival probabilities for low biomarker
    timePoints: number[]; // Time points
  };

  /** Median survival */
  medianSurvival?: {
    high: number; // Median survival for high biomarker
    low: number;  // Median survival for low biomarker
    unit: string; // Time unit (days, months, years)
  };
}

/**
 * Treatment response analysis
 */
export interface TreatmentResponse {
  /** Patient ID */
  patientID: string;

  /** Biomarker measurements over time */
  longitudinalData: Array<{
    timePoint: Date | string;
    value: number;
    unit: string;
  }>;

  /** Response classification */
  response: 'complete' | 'partial' | 'stable' | 'progressive';

  /** Percent change from baseline */
  percentChange: number;

  /** Time to response */
  timeToResponse?: number;

  /** Duration of response */
  durationOfResponse?: number;
}

// ============================================================================
// Data Exchange and Reporting
// ============================================================================

/**
 * Biomarker report
 */
export interface BiomarkerReport {
  /** Report ID */
  id: string;

  /** Report type */
  type: 'validation' | 'clinical_utility' | 'performance' | 'comprehensive';

  /** Biomarker information */
  biomarker: Biomarker;

  /** Study population */
  population: {
    totalParticipants: number;
    cases: number;
    controls: number;
    demographics?: {
      ageRange?: [number, number];
      sexDistribution?: { male: number; female: number };
      ethnicity?: Record<string, number>;
    };
  };

  /** Diagnostic performance */
  diagnosticPerformance?: DiagnosticPerformance;

  /** ROC analysis */
  rocAnalysis?: ROCResult;

  /** Clinical correlations */
  clinicalCorrelations?: ClinicalCorrelation[];

  /** Prognostic value */
  prognosticAnalysis?: PrognosticAnalysis;

  /** Recommendations */
  recommendations: string[];

  /** Study metadata */
  metadata: {
    studyID?: string;
    studyName?: string;
    principal_investigator?: string;
    institution?: string;
    dateGenerated: Date | string;
    version: string;
  };
}

/**
 * Validation study
 */
export interface ValidationStudy {
  /** Study ID */
  studyID: string;

  /** Study type */
  studyType: 'discovery' | 'validation' | 'clinical_utility' | 'multi_center';

  /** Biomarker being validated */
  biomarker: Biomarker;

  /** Study population */
  population: PatientSample[];

  /** Measurements */
  measurements: BiomarkerMeasurement[];

  /** Reference standard */
  referenceStandard: {
    method: string;
    sensitivity: number;
    specificity: number;
    description?: string;
  };

  /** Validation results */
  results: ValidationResult;

  /** Cross-validation */
  crossValidation?: {
    method: string;
    folds?: number;
    iterations?: number;
    averageAUC: number;
    aucStdDev: number;
  };

  /** External validation */
  externalValidation?: {
    cohorts: Array<{
      name: string;
      n: number;
      auc: number;
      sensitivity: number;
      specificity: number;
    }>;
  };
}

// ============================================================================
// Statistical Analysis
// ============================================================================

/**
 * Statistical test result
 */
export interface StatisticalTest {
  /** Test name */
  testName: string;

  /** Test statistic */
  statistic: number;

  /** P-value */
  pValue: number;

  /** Degrees of freedom */
  degreesOfFreedom?: number;

  /** Confidence interval */
  confidenceInterval?: [number, number];

  /** Is significant at α=0.05 */
  isSignificant: boolean;

  /** Effect size */
  effectSize?: number;
}

/**
 * Sensitivity analysis parameters
 */
export interface SensitivityAnalysisParams {
  /** Biomarker measurements */
  measurements: BiomarkerMeasurement[];

  /** Range of thresholds to test */
  thresholdRange?: {
    min: number;
    max: number;
    step: number;
  };

  /** Disease prevalence scenarios */
  prevalenceScenarios?: number[];
}

/**
 * Sensitivity analysis result
 */
export interface SensitivityAnalysisResult {
  /** Threshold analysis */
  thresholdAnalysis: Array<{
    threshold: number;
    sensitivity: number;
    specificity: number;
    ppv: number;
    npv: number;
    accuracy: number;
  }>;

  /** Prevalence impact */
  prevalenceImpact?: Array<{
    prevalence: number;
    ppv: number;
    npv: number;
  }>;

  /** Optimal thresholds by different criteria */
  optimalThresholds: {
    maxYouden: number;
    maxAccuracy: number;
    sensitivity90: number; // Threshold for 90% sensitivity
    specificity90: number; // Threshold for 90% specificity
  };
}

// ============================================================================
// Prediction and Outcome
// ============================================================================

/**
 * Prediction model
 */
export interface PredictionModel {
  /** Model ID */
  id: string;

  /** Model name */
  name: string;

  /** Model type */
  type: 'logistic_regression' | 'random_forest' | 'neural_network' | 'svm' | 'other';

  /** Biomarkers included */
  biomarkers: string[];

  /** Model coefficients/parameters */
  parameters: Record<string, number>;

  /** Performance metrics */
  performance: {
    trainingAUC: number;
    validationAUC: number;
    testAUC?: number;
    calibration?: number;
  };

  /** Feature importance */
  featureImportance?: Record<string, number>;
}

/**
 * Outcome prediction request
 */
export interface OutcomePrediction {
  /** Prediction model */
  model: PredictionModel;

  /** Patient biomarker values */
  biomarkerValues: Record<string, number>;

  /** Additional clinical variables */
  clinicalVariables?: Record<string, any>;
}

/**
 * Prediction result
 */
export interface PredictionResult {
  /** Predicted outcome */
  prediction: 'positive' | 'negative';

  /** Probability/score */
  probability: number;

  /** Risk category */
  riskCategory: 'low' | 'medium' | 'high';

  /** Confidence interval */
  confidenceInterval?: [number, number];

  /** Contributing factors */
  contributingFactors?: Array<{
    factor: string;
    contribution: number;
  }>;
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Biomarker data constants
 */
export const BIOMARKER_CONSTANTS = {
  /** Minimum sample size for validation */
  MIN_VALIDATION_SAMPLES: 100,

  /** Minimum AUC for clinical utility */
  MIN_AUC_CLINICAL: 0.70,

  /** Target sensitivity for screening tests */
  TARGET_SENSITIVITY_SCREENING: 0.90,

  /** Target specificity for confirmatory tests */
  TARGET_SPECIFICITY_CONFIRMATORY: 0.95,

  /** Maximum acceptable CV for biomarker assay */
  MAX_CV_DIAGNOSTIC: 15.0,

  /** Maximum acceptable CV for screening */
  MAX_CV_SCREENING: 20.0,

  /** Minimum statistical power */
  MIN_STATISTICAL_POWER: 0.80,

  /** Default confidence level */
  DEFAULT_CONFIDENCE_LEVEL: 0.95,

  /** Significance threshold */
  ALPHA: 0.05,

  /** Clinical utility thresholds */
  CLINICAL_UTILITY_THRESHOLDS: {
    excellent: 0.90,
    good: 0.80,
    fair: 0.70,
    poor: 0.60,
  },
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-BIO-004 error codes
 */
export enum BiomarkerErrorCode {
  INSUFFICIENT_SAMPLES = 'B001',
  INVALID_MEASUREMENTS = 'B002',
  MISSING_REFERENCE = 'B003',
  POOR_PERFORMANCE = 'B004',
  FAILED_VALIDATION = 'B005',
  LOW_STATISTICAL_POWER = 'B006',
  INVALID_THRESHOLD = 'B007',
  DATA_QUALITY_ISSUE = 'B008',
  MISSING_CLINICAL_DATA = 'B009',
  CALCULATION_ERROR = 'B010',
}

/**
 * Biomarker data error
 */
export class BiomarkerError extends Error {
  constructor(
    public code: BiomarkerErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'BiomarkerError';
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
// Export All
// ============================================================================

export type {
  // Core types
  Biomarker,
  BiomarkerType,
  MeasurementMethod,
  SampleType,
  DiseaseStatus,
  ClinicalUtility,
  ReferenceRange,

  // Measurements
  BiomarkerMeasurement,
  QualityFlags,
  PatientSample,
  LabResult,

  // Validation
  ROCRequest,
  ROCPoint,
  ROCResult,
  DiagnosticPerformance,
  BiomarkerValidation,
  ValidationResult,

  // Clinical correlation
  ClinicalCorrelation,
  ClinicalOutcome,
  PrognosticAnalysis,
  TreatmentResponse,

  // Reporting
  BiomarkerReport,
  ValidationStudy,

  // Statistical
  StatisticalTest,
  SensitivityAnalysisParams,
  SensitivityAnalysisResult,

  // Prediction
  PredictionModel,
  OutcomePrediction,
  PredictionResult,
};

export { BIOMARKER_CONSTANTS, BiomarkerErrorCode, BiomarkerError };
