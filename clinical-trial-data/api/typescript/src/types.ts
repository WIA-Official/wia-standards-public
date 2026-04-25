/**
 * WIA-BIO-010: Clinical Trial Data - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Biotechnology Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Clinical Trial Types
// ============================================================================

/**
 * Clinical trial phases
 */
export type TrialPhase = 'I' | 'II' | 'III' | 'IV';

/**
 * Trial status
 */
export type TrialStatus =
  | 'planned'
  | 'recruiting'
  | 'active'
  | 'suspended'
  | 'completed'
  | 'terminated';

/**
 * Clinical trial registration
 */
export interface ClinicalTrial {
  /** Unique trial identifier (e.g., NCT number) */
  id: string;

  /** Trial title */
  title: string;

  /** Clinical trial phase */
  phase: TrialPhase;

  /** Medical indication */
  indication: string;

  /** Sponsor organization */
  sponsor: string;

  /** Principal investigator */
  principalInvestigator: Investigator;

  /** Primary endpoint */
  primaryEndpoint: Endpoint;

  /** Secondary endpoints */
  secondaryEndpoints?: Endpoint[];

  /** Sample size information */
  sampleSize: SampleSizeInfo;

  /** Trial start date */
  startDate: Date | string;

  /** Estimated completion date */
  estimatedCompletionDate: Date | string;

  /** Current trial status */
  status: TrialStatus;

  /** Trial protocol version */
  protocolVersion?: string;

  /** Registration date */
  registrationDate: Date | string;

  /** Trial duration in weeks */
  duration?: number;
}

/**
 * Investigator information
 */
export interface Investigator {
  /** Investigator name */
  name: string;

  /** Institution */
  institution: string;

  /** Contact email */
  email?: string;

  /** Credentials */
  credentials?: string;
}

/**
 * Trial endpoint definition
 */
export interface Endpoint {
  /** Endpoint parameter */
  parameter: string;

  /** Measurement metric */
  metric: string;

  /** Assessment timepoint */
  timepoint: string;

  /** Target value or threshold */
  target?: number;

  /** Unit of measurement */
  unit?: string;
}

/**
 * Sample size information
 */
export interface SampleSizeInfo {
  /** Planned sample size */
  planned: number;

  /** Currently enrolled */
  enrolled: number;

  /** Randomized subjects */
  randomized: number;

  /** Completed subjects */
  completed: number;

  /** Withdrawn subjects */
  withdrawn?: number;

  /** Dropout rate */
  dropoutRate?: number;
}

// ============================================================================
// Patient Data Types
// ============================================================================

/**
 * Patient demographics
 */
export interface Demographics {
  /** Age in years */
  age: number;

  /** Sex */
  sex: 'M' | 'F' | 'Other';

  /** Race */
  race: string;

  /** Ethnicity */
  ethnicity: string;

  /** Body weight in kg */
  weight?: number;

  /** Height in cm */
  height?: number;

  /** BMI */
  bmi?: number;
}

/**
 * Patient enrollment and randomization
 */
export interface PatientEnrollment {
  /** Unique subject identifier */
  subjectId: string;

  /** Trial ID */
  trialId: string;

  /** Demographics */
  demographics: Demographics;

  /** Enrollment date */
  enrollmentDate: Date | string;

  /** Informed consent date */
  consentDate: Date | string;

  /** Randomization information */
  randomization?: Randomization;

  /** Enrollment status */
  status: 'enrolled' | 'randomized' | 'active' | 'withdrawn' | 'completed';
}

/**
 * Treatment randomization
 */
export interface Randomization {
  /** Randomization date */
  date: Date | string;

  /** Assigned treatment */
  treatment: string;

  /** Randomization number */
  randomizationNumber?: string;

  /** Stratification factors */
  stratification?: Record<string, string>;

  /** Treatment arm */
  arm?: string;
}

/**
 * Patient visit data
 */
export interface PatientVisit {
  /** Visit identifier */
  visitId: string;

  /** Visit name */
  visitName: string;

  /** Visit date */
  visitDate: Date | string;

  /** Subject identifier */
  subjectId: string;

  /** Visit window (days from baseline) */
  visitWindow?: number;

  /** Actual visit day */
  visitDay?: number;

  /** Visit status */
  status: 'scheduled' | 'completed' | 'missed' | 'unscheduled';

  /** Clinical measurements */
  measurements?: ClinicalMeasurements;

  /** Laboratory results */
  laboratoryResults?: LabResults;

  /** Vital signs */
  vitalSigns?: VitalSigns;
}

/**
 * Clinical measurements
 */
export interface ClinicalMeasurements {
  /** HbA1c percentage */
  hba1c?: number;

  /** Fasting glucose (mg/dL) */
  fasting_glucose?: number;

  /** Body weight (kg) */
  weight?: number;

  /** Custom measurements */
  [key: string]: number | undefined;
}

/**
 * Laboratory results
 */
export interface LabResults {
  /** Hematology */
  hematology?: {
    wbc?: number; // White blood cells (10³/μL)
    rbc?: number; // Red blood cells (10⁶/μL)
    hemoglobin?: number; // g/dL
    hematocrit?: number; // %
    platelets?: number; // 10³/μL
  };

  /** Chemistry */
  chemistry?: {
    glucose?: number; // mg/dL
    creatinine?: number; // mg/dL
    bun?: number; // mg/dL
    alt?: number; // U/L
    ast?: number; // U/L
    albumin?: number; // g/dL
  };

  /** Lipid panel */
  lipids?: {
    totalCholesterol?: number; // mg/dL
    ldl?: number; // mg/dL
    hdl?: number; // mg/dL
    triglycerides?: number; // mg/dL
  };

  /** Custom lab tests */
  [key: string]: any;
}

/**
 * Vital signs
 */
export interface VitalSigns {
  /** Systolic blood pressure (mmHg) */
  systolicBP?: number;

  /** Diastolic blood pressure (mmHg) */
  diastolicBP?: number;

  /** Heart rate (bpm) */
  heartRate?: number;

  /** Temperature (°C) */
  temperature?: number;

  /** Respiratory rate (breaths/min) */
  respiratoryRate?: number;

  /** Oxygen saturation (%) */
  oxygenSaturation?: number;
}

// ============================================================================
// Adverse Events
// ============================================================================

/**
 * Adverse event severity
 */
export type AESeverity = 'mild' | 'moderate' | 'severe' | 'life-threatening' | 'death';

/**
 * Causality assessment
 */
export type Causality = 'certain' | 'probable' | 'possible' | 'unlikely' | 'unassessable';

/**
 * Adverse event outcome
 */
export type AEOutcome =
  | 'recovered'
  | 'recovering'
  | 'not-recovered'
  | 'recovered-with-sequelae'
  | 'fatal'
  | 'unknown';

/**
 * Adverse event
 */
export interface AdverseEvent {
  /** Unique AE identifier */
  aeId: string;

  /** Subject identifier */
  subjectId: string;

  /** Trial identifier */
  trialId: string;

  /** Adverse event term */
  term: string;

  /** MedDRA coding */
  meddraCode?: MedDRACode;

  /** Start date */
  startDate: Date | string;

  /** End date */
  endDate?: Date | string;

  /** Ongoing flag */
  ongoing: boolean;

  /** Severity */
  severity: AESeverity;

  /** Seriousness criteria */
  seriousness: SeriousnessCriteria;

  /** Causality to study treatment */
  causality: Causality;

  /** Action taken */
  action?: 'none' | 'dose-reduced' | 'dose-interrupted' | 'drug-withdrawn' | 'other';

  /** Outcome */
  outcome: AEOutcome;

  /** Treatment for AE */
  treatment?: string;

  /** CTCAE grade (1-5) */
  ctcaeGrade?: 1 | 2 | 3 | 4 | 5;
}

/**
 * MedDRA coding
 */
export interface MedDRACode {
  /** Preferred Term (PT) code */
  pt: string;

  /** Preferred Term text */
  ptTerm: string;

  /** System Organ Class (SOC) code */
  soc: string;

  /** System Organ Class text */
  socTerm: string;

  /** High Level Term (HLT) */
  hlt?: string;

  /** High Level Group Term (HLGT) */
  hlgt?: string;
}

/**
 * SAE seriousness criteria
 */
export interface SeriousnessCriteria {
  /** Is serious adverse event */
  serious: boolean;

  /** Results in death */
  death: boolean;

  /** Life-threatening */
  lifeThreatening: boolean;

  /** Requires or prolongs hospitalization */
  hospitalization: boolean;

  /** Persistent or significant disability */
  disability: boolean;

  /** Congenital anomaly */
  congenitalAnomaly: boolean;

  /** Other medically important condition */
  other: boolean;
}

// ============================================================================
// Statistical Analysis
// ============================================================================

/**
 * Statistical test type
 */
export type TestType = 'one-tailed' | 'two-tailed';

/**
 * Sample size calculation parameters
 */
export interface SampleSizeCalculation {
  /** Significance level (typically 0.05) */
  alpha: number;

  /** Statistical power (typically 0.80 or 0.90) */
  power: number;

  /** Effect size */
  effectSize: number;

  /** Standard deviation (for continuous outcomes) */
  standardDeviation?: number;

  /** Event rate in control group (for binary outcomes) */
  controlRate?: number;

  /** Event rate in treatment group (for binary outcomes) */
  treatmentRate?: number;

  /** Test type */
  testType: TestType;

  /** Expected dropout rate */
  dropoutRate?: number;
}

/**
 * Sample size result
 */
export interface SampleSizeResult {
  /** Required sample size per group */
  sampleSizePerGroup: number;

  /** Total sample size */
  totalSampleSize: number;

  /** Adjusted for dropout */
  adjustedSampleSize?: number;

  /** Statistical power achieved */
  power: number;

  /** Effect size */
  effectSize: number;

  /** Calculation method */
  method: string;
}

/**
 * Statistical power calculation
 */
export interface PowerCalculation {
  /** Sample size per group */
  sampleSize: number;

  /** Significance level */
  alpha: number;

  /** Effect size */
  effectSize: number;

  /** Standard deviation */
  standardDeviation?: number;

  /** Test type */
  testType: TestType;
}

/**
 * Power calculation result
 */
export interface PowerResult {
  /** Calculated statistical power */
  power: number;

  /** Sample size used */
  sampleSize: number;

  /** Effect size */
  effectSize: number;

  /** Alpha level */
  alpha: number;
}

/**
 * Hypothesis test result
 */
export interface HypothesisTestResult {
  /** Test statistic value */
  testStatistic: number;

  /** P-value */
  pValue: number;

  /** Degrees of freedom */
  degreesOfFreedom?: number;

  /** Confidence interval (95% by default) */
  confidenceInterval: [number, number];

  /** Reject null hypothesis */
  significant: boolean;

  /** Test name */
  testName: string;

  /** Effect estimate */
  effectEstimate?: number;

  /** Standard error */
  standardError?: number;
}

/**
 * Descriptive statistics
 */
export interface DescriptiveStats {
  /** Sample size */
  n: number;

  /** Mean */
  mean: number;

  /** Standard deviation */
  sd: number;

  /** Standard error */
  se: number;

  /** Median */
  median: number;

  /** Minimum */
  min: number;

  /** Maximum */
  max: number;

  /** 25th percentile */
  q1: number;

  /** 75th percentile */
  q3: number;

  /** 95% confidence interval */
  ci95: [number, number];
}

// ============================================================================
// Data Collection
// ============================================================================

/**
 * CDISC SDTM domain
 */
export type SDTMDomain =
  | 'DM' // Demographics
  | 'AE' // Adverse Events
  | 'VS' // Vital Signs
  | 'LB' // Laboratory
  | 'CM' // Concomitant Medications
  | 'EX' // Exposure
  | 'DS' // Disposition
  | 'MH' // Medical History
  | 'PE' // Physical Examination
  | 'QS'; // Questionnaires

/**
 * SDTM record
 */
export interface SDTMRecord {
  /** Study identifier */
  STUDYID: string;

  /** Domain abbreviation */
  DOMAIN: SDTMDomain;

  /** Unique subject identifier */
  USUBJID: string;

  /** Sequence number */
  SEQ: number;

  /** Additional domain-specific fields */
  [key: string]: any;
}

/**
 * Data collection request
 */
export interface DataCollectionRequest {
  /** Trial ID */
  trialId: string;

  /** Patient ID */
  patientId: string;

  /** Visit identifier */
  visit: string;

  /** Clinical measurements */
  measurements?: Record<string, number>;

  /** Laboratory results */
  laboratoryResults?: LabResults;

  /** Vital signs */
  vitalSigns?: VitalSigns;

  /** Adverse events */
  adverseEvents?: AdverseEvent[];
}

/**
 * Data collection response
 */
export interface DataCollectionResponse {
  /** Success flag */
  success: boolean;

  /** Data record ID */
  dataId?: string;

  /** Validation errors */
  validationErrors: string[];

  /** Warnings */
  warnings: string[];

  /** Timestamp */
  timestamp: Date;
}

// ============================================================================
// Regulatory Submission
// ============================================================================

/**
 * Regulatory agency
 */
export type RegulatoryAgency = 'FDA' | 'EMA' | 'PMDA' | 'NMPA' | 'MHRA' | 'OTHER';

/**
 * Submission format
 */
export type SubmissionFormat = 'CDISC' | 'eCTD' | 'NeeS' | 'SEND';

/**
 * Regulatory submission
 */
export interface RegulatorySubmission {
  /** Submission ID */
  id: string;

  /** Trial ID */
  trialId: string;

  /** Target agency */
  agency: RegulatoryAgency;

  /** Submission format */
  format: SubmissionFormat;

  /** Submission type */
  type: 'IND' | 'NDA' | 'BLA' | 'CTA' | 'MAA' | 'OTHER';

  /** Submission date */
  submissionDate: Date | string;

  /** Included datasets */
  datasets: string[];

  /** Status */
  status: 'draft' | 'submitted' | 'under-review' | 'approved' | 'rejected';

  /** Review comments */
  comments?: string[];
}

// ============================================================================
// Physical Constants & Thresholds
// ============================================================================

/**
 * Clinical trial constants
 */
export const CLINICAL_CONSTANTS = {
  /** Standard significance level */
  ALPHA: 0.05,

  /** Standard statistical power */
  POWER: 0.80,

  /** Critical value for two-sided α=0.05 */
  Z_ALPHA_2: 1.96,

  /** Critical value for 80% power */
  Z_BETA: 0.84,

  /** Critical value for 90% power */
  Z_BETA_90: 1.28,

  /** Typical dropout rate */
  DROPOUT_RATE: 0.20,

  /** SAE reporting timeline (days) */
  SAE_REPORTING_DAYS: 15,

  /** Fatal SAE reporting timeline (days) */
  FATAL_SAE_REPORTING_DAYS: 7,
} as const;

/**
 * Normal ranges for vital signs
 */
export const VITAL_SIGNS_RANGES = {
  systolicBP: { min: 90, max: 140, unit: 'mmHg' },
  diastolicBP: { min: 60, max: 90, unit: 'mmHg' },
  heartRate: { min: 60, max: 100, unit: 'bpm' },
  temperature: { min: 36.1, max: 37.2, unit: '°C' },
  respiratoryRate: { min: 12, max: 20, unit: 'breaths/min' },
  oxygenSaturation: { min: 95, max: 100, unit: '%' },
} as const;

/**
 * Normal ranges for common laboratory tests
 */
export const LAB_RANGES = {
  hba1c: { min: 4.0, max: 5.6, unit: '%', target: '<7.0' },
  glucose_fasting: { min: 70, max: 100, unit: 'mg/dL', target: '<126' },
  creatinine: { min: 0.6, max: 1.2, unit: 'mg/dL' },
  alt: { min: 7, max: 56, unit: 'U/L' },
  ast: { min: 10, max: 40, unit: 'U/L' },
  totalCholesterol: { min: 0, max: 200, unit: 'mg/dL', target: '<200' },
  ldl: { min: 0, max: 100, unit: 'mg/dL', target: '<100' },
  hdl: { min: 40, max: 200, unit: 'mg/dL', target: '>60' },
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

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-BIO-010 error codes
 */
export enum ClinicalTrialErrorCode {
  INVALID_TRIAL_ID = 'B001',
  PATIENT_NOT_ENROLLED = 'B002',
  VALIDATION_FAILURE = 'B003',
  SAE_NOT_REPORTED = 'B004',
  STATISTICAL_VIOLATION = 'B005',
  MISSING_CONSENT = 'B006',
  DATA_OUT_OF_RANGE = 'B007',
  PROTOCOL_DEVIATION = 'B008',
  REGULATORY_NONCOMPLIANCE = 'B009',
  EDC_SYSTEM_ERROR = 'B010',
}

/**
 * Clinical trial error
 */
export class ClinicalTrialError extends Error {
  constructor(
    public code: ClinicalTrialErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'ClinicalTrialError';
  }
}

// ============================================================================
// Export All Types
// ============================================================================

export type {
  // Core types
  ClinicalTrial,
  Investigator,
  Endpoint,
  SampleSizeInfo,

  // Patient data
  Demographics,
  PatientEnrollment,
  Randomization,
  PatientVisit,
  ClinicalMeasurements,
  LabResults,
  VitalSigns,

  // Adverse events
  AdverseEvent,
  MedDRACode,
  SeriousnessCriteria,

  // Statistics
  SampleSizeCalculation,
  SampleSizeResult,
  PowerCalculation,
  PowerResult,
  HypothesisTestResult,
  DescriptiveStats,

  // Data collection
  SDTMRecord,
  DataCollectionRequest,
  DataCollectionResponse,

  // Regulatory
  RegulatorySubmission,
};

export {
  CLINICAL_CONSTANTS,
  VITAL_SIGNS_RANGES,
  LAB_RANGES,
  ClinicalTrialErrorCode,
  ClinicalTrialError,
};
