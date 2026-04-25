/**
 * WIA-BIO-003: Gene Therapy - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Biotechnology Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Therapy Types
// ============================================================================

/**
 * Viral vector types used in gene therapy
 */
export type VectorType =
  | 'AAV1'
  | 'AAV2'
  | 'AAV5'
  | 'AAV8'
  | 'AAV9'
  | 'AAVrh10'
  | 'lentivirus'
  | 'adenovirus'
  | 'LNP'
  | 'electroporation'
  | 'custom';

/**
 * Target tissue types
 */
export type TargetTissue =
  | 'liver'
  | 'muscle'
  | 'CNS'
  | 'heart'
  | 'lung'
  | 'retina'
  | 'blood'
  | 'bone-marrow'
  | 'other';

/**
 * Delivery routes
 */
export type DeliveryRoute =
  | 'IV'
  | 'intramuscular'
  | 'intrathecal'
  | 'subretinal'
  | 'intravitreal'
  | 'intra-articular'
  | 'ex-vivo';

/**
 * Gene therapy vector configuration
 */
export interface GeneTherapyVector {
  /** Vector identifier */
  id: string;

  /** Vector type (AAV, lentivirus, etc.) */
  type: VectorType;

  /** Serotype for AAV vectors */
  serotype?: string;

  /** Therapeutic gene symbol */
  gene: string;

  /** Gene name */
  geneName: string;

  /** Promoter type */
  promoter: string;

  /** Vector genome size in kb */
  genomeSize: number;

  /** Production titer in vg/mL */
  titer: number;

  /** Tropism (tissue targeting) */
  tropism: TargetTissue[];

  /** Expected transduction efficiency (%) */
  expectedEfficiency: number;

  /** Immunogenicity level */
  immunogenicity: 'low' | 'moderate' | 'high';

  /** Expression duration */
  duration: 'transient' | 'long-term' | 'permanent';
}

/**
 * Therapeutic gene target
 */
export interface TargetGene {
  /** Gene symbol (e.g., F8, SMN1) */
  symbol: string;

  /** Full gene name */
  name: string;

  /** Chromosome location */
  chromosome: string;

  /** Coding sequence length */
  cdnaLength: number;

  /** Associated disease(s) */
  diseases: string[];

  /** Normal expression level (copies/cell or protein concentration) */
  normalExpression?: number;

  /** Expression unit */
  expressionUnit?: string;
}

// ============================================================================
// Dosage and Administration
// ============================================================================

/**
 * Dosage calculation parameters
 */
export interface DosageParameters {
  /** Patient weight in kg */
  patientWeight: number;

  /** Target tissue for delivery */
  targetTissue: TargetTissue;

  /** Vector type */
  vectorType: VectorType;

  /** Therapeutic gene */
  therapeuticGene: string;

  /** AAV serotype (if applicable) */
  serotype?: string;

  /** Target cell count (optional override) */
  targetCellCount?: number;

  /** Desired vg per cell (optional override) */
  vgPerCell?: number;

  /** Route of administration */
  route?: DeliveryRoute;
}

/**
 * Calculated dosage result
 */
export interface DosageResult {
  /** Total viral genomes required */
  viralGenomes: number;

  /** Dose in vg/kg */
  vgPerKg: number;

  /** Injection volume in mL */
  volumeMl: number;

  /** Expected transduction efficiency (%) */
  expectedEfficiency: number;

  /** Feasibility assessment */
  feasibility: 'standard' | 'high-dose' | 'experimental' | 'not-recommended';

  /** Dose level classification */
  doseLevel: 'low' | 'medium' | 'high' | 'MTD';

  /** Recommended administration protocol */
  protocol: AdministrationProtocol;

  /** Safety warnings */
  warnings: string[];
}

/**
 * Administration protocol
 */
export interface AdministrationProtocol {
  /** Route of administration */
  route: DeliveryRoute;

  /** Infusion/injection duration */
  duration: string;

  /** Pre-medications */
  premedications: string[];

  /** Hospital observation period */
  observationPeriod: string;

  /** Special instructions */
  instructions: string[];
}

// ============================================================================
// Safety and Monitoring
// ============================================================================

/**
 * Safety assessment parameters
 */
export interface SafetyAssessment {
  /** Vector dose in vg/kg */
  vectorDose: number;

  /** Immune system status */
  immuneStatus: 'normal' | 'suppressed' | 'autoimmune' | 'compromised';

  /** Pre-existing neutralizing antibodies */
  preexistingAntibodies: boolean;

  /** NAb titer (if tested) */
  nabTiter?: number;

  /** Liver function tests */
  liverFunction: LiverFunction;

  /** Kidney function */
  kidneyFunction?: KidneyFunction;

  /** Patient age in years */
  age: number;

  /** Previous gene therapy exposure */
  previousTherapy?: boolean;

  /** Concurrent medications */
  medications?: string[];
}

/**
 * Liver function parameters
 */
export interface LiverFunction {
  /** Alanine aminotransferase (U/L) */
  ALT: number;

  /** Aspartate aminotransferase (U/L) */
  AST: number;

  /** Total bilirubin (mg/dL) */
  totalBilirubin: number;

  /** Alkaline phosphatase (U/L) */
  ALP?: number;

  /** Albumin (g/dL) */
  albumin?: number;
}

/**
 * Kidney function parameters
 */
export interface KidneyFunction {
  /** Serum creatinine (mg/dL) */
  creatinine: number;

  /** Estimated GFR (mL/min/1.73m²) */
  eGFR: number;

  /** Blood urea nitrogen (mg/dL) */
  BUN?: number;
}

/**
 * Safety result
 */
export interface SafetyResult {
  /** Overall safety score (0-100) */
  score: number;

  /** Risk level */
  riskLevel: 'low' | 'medium' | 'high' | 'extreme';

  /** Specific safety concerns */
  warnings: string[];

  /** Recommendations for mitigation */
  recommendations: string[];

  /** Monitoring plan */
  monitoringPlan: MonitoringPlan;

  /** Predicted adverse events */
  predictedAdverseEvents: AdverseEvent[];

  /** Eligibility for treatment */
  eligible: boolean;

  /** Reasons for ineligibility (if applicable) */
  ineligibilityReasons?: string[];
}

/**
 * Monitoring plan
 */
export interface MonitoringPlan {
  /** Monitoring frequency */
  frequency: string;

  /** Biomarkers to monitor */
  biomarkers: string[];

  /** Laboratory tests */
  labTests: LabTest[];

  /** Imaging studies */
  imaging?: string[];

  /** Duration of monitoring */
  duration: string;

  /** Follow-up schedule */
  schedule: FollowUpSchedule[];
}

/**
 * Laboratory test
 */
export interface LabTest {
  /** Test name */
  name: string;

  /** Test frequency */
  frequency: string;

  /** Normal range */
  normalRange: string;

  /** Action threshold */
  actionThreshold?: string;
}

/**
 * Follow-up schedule
 */
export interface FollowUpSchedule {
  /** Timepoint (e.g., "Day 1", "Week 4") */
  timepoint: string;

  /** Assessments to perform */
  assessments: string[];

  /** Is this a critical timepoint? */
  critical: boolean;
}

/**
 * Adverse event
 */
export interface AdverseEvent {
  /** Event type */
  type: string;

  /** Probability (0-1) */
  probability: number;

  /** Severity grade (1-5) */
  grade: number;

  /** Expected onset */
  onset: string;

  /** Duration */
  duration?: string;

  /** Management strategy */
  management: string[];
}

// ============================================================================
// Expression and Efficacy
// ============================================================================

/**
 * Gene expression monitoring parameters
 */
export interface ExpressionMonitoring {
  /** Therapeutic gene */
  gene: string;

  /** Measurement timepoints (days post-treatment) */
  timePoints: number[];

  /** Measurement method */
  method: 'qPCR' | 'ELISA' | 'Western-blot' | 'activity-assay' | 'flow-cytometry';

  /** Baseline value (pre-treatment) */
  baselineValue?: number;

  /** Target therapeutic level */
  targetLevel?: number;

  /** Measurement unit */
  unit: string;
}

/**
 * Expression result
 */
export interface ExpressionResult {
  /** Individual measurements */
  measurements: ExpressionMeasurement[];

  /** Overall trend */
  trend: 'increasing' | 'stable' | 'decreasing' | 'variable';

  /** Is expression in therapeutic range? */
  therapeuticRange: boolean;

  /** Peak expression */
  peakExpression: {
    value: number;
    timepoint: number;
    percentOfNormal: number;
  };

  /** Current expression level */
  currentLevel: {
    value: number;
    percentOfNormal: number;
    therapeutic: boolean;
  };

  /** Predicted future expression */
  prediction?: ExpressionPrediction;
}

/**
 * Individual expression measurement
 */
export interface ExpressionMeasurement {
  /** Timepoint in days */
  timepoint: number;

  /** Measured value */
  value: number;

  /** Measurement unit */
  unit: string;

  /** Percent of normal expression */
  percentOfNormal: number;

  /** Standard deviation */
  standardDeviation?: number;

  /** Quality control status */
  qcStatus: 'pass' | 'fail' | 'warning';
}

/**
 * Expression level prediction
 */
export interface ExpressionPrediction {
  /** Predicted value at future timepoint */
  predictedValue: number;

  /** Prediction timepoint (days) */
  timepoint: number;

  /** Confidence interval (95%) */
  confidenceInterval: {
    lower: number;
    upper: number;
  };

  /** Model used for prediction */
  model: string;
}

/**
 * Transduction efficiency measurement
 */
export interface TransductionResult {
  /** Total cells analyzed */
  totalCells: number;

  /** Successfully transduced cells */
  transducedCells: number;

  /** Transduction efficiency (%) */
  efficiency: number;

  /** Vector copies per cell */
  vectorCopiesPerCell: number;

  /** Tissue/cell type */
  cellType: string;

  /** Measurement method */
  method: string;

  /** Time post-administration */
  timepoint: number;
}

// ============================================================================
// CRISPR/Gene Editing
// ============================================================================

/**
 * CRISPR delivery system
 */
export interface CRISPRDelivery {
  /** Cas protein type */
  casProtein: 'Cas9' | 'Cas12a' | 'base-editor' | 'prime-editor';

  /** Guide RNA sequence (20 nt) */
  guideRNA: string;

  /** PAM sequence */
  pamSequence: string;

  /** Donor template (for HDR) */
  donorTemplate?: string;

  /** Delivery method */
  deliveryMethod: 'AAV' | 'LNP' | 'electroporation' | 'RNP';

  /** Expected on-target efficiency (%) */
  onTargetEfficiency: number;

  /** Predicted off-target sites */
  offTargetSites: OffTargetSite[];
}

/**
 * Off-target genomic site
 */
export interface OffTargetSite {
  /** Chromosome location */
  chromosome: string;

  /** Genomic position */
  position: number;

  /** Off-target sequence */
  sequence: string;

  /** Number of mismatches */
  mismatches: number;

  /** Off-target score */
  score: number;

  /** Predicted cleavage frequency */
  cleavageFrequency: number;

  /** Gene affected (if any) */
  affectedGene?: string;

  /** Severity if cleaved */
  severity: 'low' | 'medium' | 'high' | 'critical';
}

/**
 * Gene editing result
 */
export interface EditingResult {
  /** On-target editing efficiency (%) */
  onTargetEfficiency: number;

  /** Off-target editing rate (%) */
  offTargetRate: number;

  /** Indel frequency (%) */
  indelFrequency: number;

  /** Precise edit frequency (HDR, %) */
  preciseEditFrequency?: number;

  /** Quality score (0-100) */
  qualityScore: number;

  /** Detected off-target events */
  offTargetEvents: OffTargetEvent[];

  /** Karyotype status */
  karyotypeNormal: boolean;
}

/**
 * Off-target editing event
 */
export interface OffTargetEvent {
  /** Genomic location */
  location: string;

  /** Type of modification */
  modificationType: 'insertion' | 'deletion' | 'substitution';

  /** Size of modification (bp) */
  size: number;

  /** Allele frequency (%) */
  alleleFrequency: number;

  /** Functional consequence */
  consequence: string;
}

// ============================================================================
// Clinical Protocol
// ============================================================================

/**
 * Clinical trial protocol
 */
export interface ClinicalProtocol {
  /** Protocol identifier */
  protocolId: string;

  /** Trial phase */
  phase: 'Phase-I' | 'Phase-I/II' | 'Phase-II' | 'Phase-III';

  /** Disease/condition */
  condition: string;

  /** Therapeutic approach */
  therapy: GeneTherapyVector;

  /** Inclusion criteria */
  inclusionCriteria: string[];

  /** Exclusion criteria */
  exclusionCriteria: string[];

  /** Dosing regimen */
  dosingRegimen: DosingRegimen;

  /** Primary endpoints */
  primaryEndpoints: string[];

  /** Secondary endpoints */
  secondaryEndpoints: string[];

  /** Safety monitoring */
  safetyMonitoring: MonitoringPlan;

  /** Estimated duration */
  duration: string;

  /** Sample size */
  sampleSize: number;
}

/**
 * Dosing regimen
 */
export interface DosingRegimen {
  /** Dose levels to test */
  doseLevels: number[];

  /** Dose escalation design */
  escalationDesign: '3+3' | 'BOIN' | 'mTPI' | 'fixed';

  /** Patients per cohort */
  patientsPerCohort: number;

  /** DLT observation period */
  dltObservationPeriod: string;

  /** Maximum tolerated dose target */
  mtdTarget?: number;
}

/**
 * Patient consent information
 */
export interface PatientConsent {
  /** Patient identifier */
  patientId: string;

  /** Consent date */
  consentDate: Date;

  /** Consent version */
  consentVersion: string;

  /** Key risks acknowledged */
  acknowledgedRisks: string[];

  /** Long-term monitoring agreement */
  longTermMonitoring: boolean;

  /** Contraception agreement (if applicable) */
  contraceptionAgreement?: boolean;

  /** Data sharing consent */
  dataSharingConsent: boolean;

  /** Genetic privacy consent */
  geneticPrivacyConsent: boolean;

  /** Witness signature */
  witnessedBy?: string;
}

// ============================================================================
// Physical and Biological Constants
// ============================================================================

/**
 * Gene therapy constants
 */
export const GENE_THERAPY_CONSTANTS = {
  /** Typical AAV capsid size (nm) */
  AAV_CAPSID_SIZE: 25,

  /** AAV genome capacity (kb) */
  AAV_MAX_GENOME: 4.7,

  /** Lentivirus genome capacity (kb) */
  LENTIVIRUS_MAX_GENOME: 8.0,

  /** Standard dose ranges (vg/kg) */
  DOSE_RANGES: {
    LOW: 1e12,
    MEDIUM: 1e13,
    HIGH: 1e14,
    MTD: 2e14,
  },

  /** Transduction efficiency by tissue (%) */
  TRANSDUCTION_EFFICIENCY: {
    liver_AAV8: 80,
    liver_AAV9: 70,
    muscle_AAV1: 60,
    muscle_AAV9: 50,
    CNS_AAV9: 40,
    retina_AAV2: 70,
  },

  /** Safety thresholds */
  SAFETY_THRESHOLDS: {
    NAB_TITER_MAX: 5,
    ALT_ULN: 40, // U/L
    AST_ULN: 40, // U/L
    BILIRUBIN_MAX: 1.2, // mg/dL
    OFF_TARGET_RATE_MAX: 0.1, // %
  },

  /** Expression kinetics (days) */
  EXPRESSION_KINETICS: {
    AAV_ONSET: 7,
    AAV_PEAK: 28,
    AAV_PLATEAU: 84,
    LENTIVIRUS_ONSET: 3,
    LENTIVIRUS_PEAK: 14,
  },

  /** Monitoring periods (days) */
  MONITORING_PERIODS: {
    ACUTE: 7,
    SUBACUTE: 28,
    INTERMEDIATE: 180,
    LONG_TERM: 365,
  },
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
 * Gene therapy simulation result
 */
export interface SimulationResult {
  /** Simulation identifier */
  id: string;

  /** Patient parameters */
  patient: PatientProfile;

  /** Therapy administered */
  therapy: GeneTherapyVector;

  /** Dosage calculation */
  dosage: DosageResult;

  /** Safety assessment */
  safety: SafetyResult;

  /** Predicted expression */
  expression: ExpressionResult;

  /** Predicted outcomes */
  outcomes: TherapeuticOutcome;

  /** Simulation duration (ms) */
  duration: number;

  /** Success status */
  success: boolean;

  /** Error message if failed */
  error?: string;
}

/**
 * Patient profile
 */
export interface PatientProfile {
  /** Patient identifier */
  id: string;

  /** Age in years */
  age: number;

  /** Weight in kg */
  weight: number;

  /** Diagnosis */
  diagnosis: string;

  /** Genotype */
  genotype?: string;

  /** Baseline disease severity */
  baselineSeverity: 'mild' | 'moderate' | 'severe';

  /** Previous treatments */
  previousTreatments?: string[];

  /** Comorbidities */
  comorbidities?: string[];
}

/**
 * Therapeutic outcome prediction
 */
export interface TherapeuticOutcome {
  /** Predicted success probability (0-1) */
  successProbability: number;

  /** Expected clinical benefit */
  clinicalBenefit: 'none' | 'minimal' | 'moderate' | 'substantial' | 'cure';

  /** Time to therapeutic effect (days) */
  timeToEffect: number;

  /** Duration of effect */
  durationOfEffect: string;

  /** Quality of life improvement (0-100) */
  qolImprovement: number;

  /** Predicted biomarker changes */
  biomarkerChanges: {
    biomarker: string;
    baselineValue: number;
    predictedValue: number;
    unit: string;
  }[];
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-BIO-003 error codes
 */
export enum GeneTherapyErrorCode {
  DOSE_EXCEEDS_MTD = 'B001',
  HIGH_NAB_TITER = 'B002',
  LIVER_DYSFUNCTION = 'B003',
  HIGH_OFF_TARGET = 'B004',
  IMMUNOGENICITY_ALERT = 'B005',
  VECTOR_QUALITY_FAILURE = 'B006',
  INVALID_PARAMETERS = 'B007',
  PATIENT_INELIGIBLE = 'B008',
  CONSENT_REQUIRED = 'B009',
  REGULATORY_VIOLATION = 'B010',
}

/**
 * Gene therapy error
 */
export class GeneTherapyError extends Error {
  constructor(
    public code: GeneTherapyErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'GeneTherapyError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core types
  VectorType,
  TargetTissue,
  DeliveryRoute,
  GeneTherapyVector,
  TargetGene,

  // Dosage
  DosageParameters,
  DosageResult,
  AdministrationProtocol,

  // Safety
  SafetyAssessment,
  LiverFunction,
  KidneyFunction,
  SafetyResult,
  MonitoringPlan,
  LabTest,
  FollowUpSchedule,
  AdverseEvent,

  // Expression
  ExpressionMonitoring,
  ExpressionResult,
  ExpressionMeasurement,
  ExpressionPrediction,
  TransductionResult,

  // CRISPR
  CRISPRDelivery,
  OffTargetSite,
  EditingResult,
  OffTargetEvent,

  // Clinical
  ClinicalProtocol,
  DosingRegimen,
  PatientConsent,
  PatientProfile,
  TherapeuticOutcome,

  // Simulation
  SimulationResult,
};

export { GENE_THERAPY_CONSTANTS, GeneTherapyErrorCode, GeneTherapyError };
