/**
 * WIA-TIME-028: Temporal Medical Care - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Temporal Medicine Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Medical Types
// ============================================================================

/**
 * Patient identification and basic information
 */
export interface PatientInfo {
  /** Unique patient identifier */
  patientId: string;

  /** Patient name (optional for privacy) */
  name?: string;

  /** Chronological age in years */
  chronologicalAge: number;

  /** Biological age in years (if known) */
  biologicalAge?: number;

  /** Gender */
  gender?: 'male' | 'female' | 'other' | 'unknown';

  /** Blood type */
  bloodType?: string;

  /** Medical history summary */
  medicalHistory?: string[];

  /** Temporal travel history */
  temporalHistory?: TemporalExposure[];

  /** Allergies and contraindications */
  allergies?: string[];

  /** Current medications */
  medications?: Medication[];
}

/**
 * Temporal exposure record
 */
export interface TemporalExposure {
  /** Exposure ID */
  id: string;

  /** Date of temporal displacement */
  date: Date;

  /** Temporal displacement in seconds */
  displacement: number;

  /** Method of travel */
  method: 'wormhole' | 'ctc' | 'field' | 'alcubierre' | 'other';

  /** Duration of displacement in seconds */
  duration: number;

  /** Post-travel symptoms */
  symptoms?: string[];

  /** Medical treatment received */
  treatmentReceived?: string[];

  /** Recovery time in days */
  recoveryTime?: number;
}

/**
 * Medication record
 */
export interface Medication {
  /** Medication name */
  name: string;

  /** Dosage */
  dosage: string;

  /** Frequency */
  frequency: string;

  /** Route of administration */
  route: 'oral' | 'IV' | 'IM' | 'SC' | 'topical' | 'other';

  /** Start date */
  startDate: Date;

  /** End date (if applicable) */
  endDate?: Date;

  /** Reason for medication */
  indication?: string;
}

// ============================================================================
// Vital Signs
// ============================================================================

/**
 * Standard vital signs
 */
export interface VitalSigns {
  /** Heart rate in beats per minute */
  heartRate: number;

  /** Blood pressure */
  bloodPressure: {
    systolic: number;
    diastolic: number;
  };

  /** Body temperature in Celsius */
  temperature: number;

  /** Respiratory rate in breaths per minute */
  respiratoryRate?: number;

  /** Oxygen saturation percentage */
  oxygenSaturation?: number;

  /** Timestamp of measurement */
  timestamp?: Date;
}

/**
 * Temporal-specific vital signs
 */
export interface TemporalVitalSigns extends VitalSigns {
  /** Temporal Stress Index (0-100) */
  temporalStressIndex: number;

  /** Cellular Age Indicator */
  cellularAgeIndicator: number;

  /** Chronological Alignment Score (0-1) */
  chronologicalAlignment: number;

  /** Memory Coherence Index (0-1) */
  memoryCoherence: number;

  /** Temporal Antibody Count per μL */
  temporalAntibodyCount?: number;
}

// ============================================================================
// Temporal Sickness Syndrome
// ============================================================================

/**
 * TSS diagnosis parameters
 */
export interface TSSParameters {
  /** Patient information */
  patientId: string;

  /** Temporal displacement in seconds */
  displacement: number;

  /** Mass of patient in kg */
  mass?: number;

  /** Velocity factor (0-1) */
  velocityFactor?: number;

  /** Biological age */
  biologicalAge?: number;

  /** Recovery coefficient (0-1) */
  recoveryCoefficient?: number;

  /** Current symptoms */
  symptoms: string[];

  /** Vital signs */
  vitalSigns: VitalSigns;

  /** Previous temporal exposure */
  previousExposures?: number;
}

/**
 * TSS diagnosis result
 */
export interface TSSDiagnosis {
  /** Patient ID */
  patientId: string;

  /** Diagnosis timestamp */
  timestamp: Date;

  /** Condition identified */
  condition: 'TSS' | 'CAD' | 'TA' | 'CSS' | 'PTD' | 'none' | 'multiple';

  /** Severity score (0-100) */
  severity: number;

  /** Severity level */
  severityLevel: 'mild' | 'moderate' | 'severe' | 'critical';

  /** Symptoms present */
  symptoms: string[];

  /** Clinical findings */
  findings: string[];

  /** Recommended treatment */
  recommendedTreatment: TreatmentPlan;

  /** Prognosis */
  prognosis: 'excellent' | 'good' | 'fair' | 'poor' | 'critical';

  /** Expected recovery time in days */
  expectedRecovery: number;

  /** Follow-up required */
  requiresFollowUp: boolean;

  /** Hospitalization required */
  requiresHospitalization: boolean;

  /** Additional notes */
  notes?: string;
}

// ============================================================================
// Cellular Age Management
// ============================================================================

/**
 * Cellular age analysis parameters
 */
export interface CellularAgeParameters {
  /** Patient ID */
  patientId: string;

  /** Chronological age in years */
  chronologicalAge: number;

  /** Telomere length in base pairs */
  telomereLength: number;

  /** Epigenetic age in years (if measured) */
  epigeneticAge?: number;

  /** Percentage of senescent cells */
  senescentCellPercentage?: number;

  /** Mitochondrial function (0-100%) */
  mitochondrialFunction?: number;

  /** Temporal displacement history */
  displacementHistory: number[];

  /** Advanced glycation end-products level */
  ageLevel?: number;

  /** Oxidative stress markers */
  oxidativeStress?: number;
}

/**
 * Cellular age analysis result
 */
export interface CellularAgeAnalysis {
  /** Patient ID */
  patientId: string;

  /** Analysis timestamp */
  timestamp: Date;

  /** Calculated biological age in years */
  biologicalAge: number;

  /** Chronological age in years */
  chronologicalAge: number;

  /** Age delta (biological - chronological) */
  ageDelta: number;

  /** Age delta as percentage */
  ageDeltaPercentage: number;

  /** Severity classification */
  severity: 'normal' | 'mild' | 'moderate' | 'severe' | 'critical';

  /** Requires treatment? */
  requiresTreatment: boolean;

  /** Biomarker values */
  biomarkers: {
    telomereLength: number;
    telomereStatus: 'normal' | 'short' | 'very-short' | 'long';
    epigeneticAge?: number;
    senescentCells?: number;
    mitochondrialHealth?: number;
  };

  /** Contributing factors */
  contributingFactors: string[];

  /** Treatment recommendation */
  treatmentRecommendation?: TreatmentPlan;

  /** Prognosis */
  prognosis: string;
}

// ============================================================================
// Memory Disorders
// ============================================================================

/**
 * Memory assessment parameters
 */
export interface MemoryAssessmentParameters {
  /** Patient ID */
  patientId: string;

  /** Total expected memories for age */
  expectedMemories: number;

  /** Number of intact, accessible memories */
  intactMemories: number;

  /** Memory fragmentation events */
  fragmentationEvents: number;

  /** Timeline coherence score (0-1) */
  timelineCoherence: number;

  /** Temporal exposure count */
  temporalExposures: number;

  /** Cognitive test scores */
  cognitiveScores?: {
    recall: number;
    recognition: number;
    sequencing: number;
    orientation: number;
  };

  /** Reported symptoms */
  symptoms: string[];
}

/**
 * Memory coherence result
 */
export interface MemoryCoherenceResult {
  /** Patient ID */
  patientId: string;

  /** Assessment timestamp */
  timestamp: Date;

  /** Memory Coherence Index (0-1) */
  memoryCoherenceIndex: number;

  /** Coherence level */
  coherenceLevel: 'normal' | 'mild-impairment' | 'moderate-impairment' | 'severe-impairment';

  /** Type of memory disorder */
  disorderType?: 'anterograde' | 'retrograde' | 'timeline-confusion' | 'fragmentation' | 'mixed';

  /** Affected memory domains */
  affectedDomains: string[];

  /** Intact memory percentage */
  intactPercentage: number;

  /** Fragmentation rate */
  fragmentationRate: number;

  /** Timeline coherence score */
  timelineCoherence: number;

  /** Treatment recommended */
  treatmentRecommended: boolean;

  /** Treatment plan */
  treatmentPlan?: TreatmentPlan;

  /** Prognosis */
  prognosis: string;
}

// ============================================================================
// Chronological Stress Syndrome
// ============================================================================

/**
 * CSS assessment parameters
 */
export interface CSSParameters {
  /** Patient ID */
  patientId: string;

  /** Psychological distress level (0-10) */
  psychologicalDistress: number;

  /** Total temporal exposure time in seconds */
  totalExposureTime: number;

  /** Number of paradoxical or stressful events */
  paradoxicalEvents: number;

  /** Social support score (0-10) */
  socialSupport: number;

  /** Resilience factor (0-10) */
  resilience: number;

  /** Reported symptoms */
  symptoms: string[];

  /** Sleep quality (0-10) */
  sleepQuality?: number;

  /** Anxiety level (0-10) */
  anxietyLevel?: number;

  /** Depression indicators */
  depressionIndicators?: string[];
}

/**
 * CSS assessment result
 */
export interface CSSAssessment {
  /** Patient ID */
  patientId: string;

  /** Assessment timestamp */
  timestamp: Date;

  /** CSS Score */
  cssScore: number;

  /** Severity level */
  severity: 'mild' | 'moderate' | 'severe' | 'critical';

  /** Primary symptoms */
  primarySymptoms: string[];

  /** Emotional impact */
  emotionalImpact: {
    anxiety: number;
    depression: number;
    emotionalInstability: number;
  };

  /** Cognitive impact */
  cognitiveImpact: {
    concentration: number;
    decisionMaking: number;
    temporalOrientation: number;
  };

  /** Behavioral changes */
  behavioralChanges: string[];

  /** Requires intervention */
  requiresIntervention: boolean;

  /** Treatment plan */
  treatmentPlan?: TreatmentPlan;

  /** Suicide risk assessment */
  suicideRisk?: 'none' | 'low' | 'moderate' | 'high';

  /** Follow-up schedule */
  followUpSchedule: string;
}

// ============================================================================
// Treatment Plans
// ============================================================================

/**
 * Comprehensive treatment plan
 */
export interface TreatmentPlan {
  /** Plan ID */
  id: string;

  /** Patient ID */
  patientId: string;

  /** Primary diagnosis */
  diagnosis: string;

  /** Treatment start date */
  startDate: Date;

  /** Expected end date */
  expectedEndDate?: Date;

  /** Treatment goals */
  goals: string[];

  /** Pharmacological interventions */
  medications?: MedicationPrescription[];

  /** Non-pharmacological interventions */
  therapies?: TherapyIntervention[];

  /** Monitoring requirements */
  monitoring: MonitoringRequirement[];

  /** Follow-up schedule */
  followUpSchedule: FollowUpSchedule[];

  /** Special instructions */
  specialInstructions?: string[];

  /** Emergency contact plan */
  emergencyPlan?: EmergencyPlan;

  /** Success criteria */
  successCriteria: string[];

  /** Treatment team */
  team?: {
    primaryPhysician: string;
    specialists?: string[];
    nurses?: string[];
    therapists?: string[];
  };
}

/**
 * Medication prescription
 */
export interface MedicationPrescription {
  /** Medication name */
  medication: string;

  /** Generic name */
  genericName?: string;

  /** Dosage */
  dosage: string;

  /** Frequency */
  frequency: string;

  /** Route of administration */
  route: 'oral' | 'IV' | 'IM' | 'SC' | 'topical' | 'inhaled' | 'other';

  /** Duration in days */
  duration: number;

  /** Indication */
  indication: string;

  /** Special instructions */
  instructions?: string[];

  /** Potential side effects */
  sideEffects?: string[];

  /** Monitoring required */
  monitoringRequired?: boolean;
}

/**
 * Therapy intervention
 */
export interface TherapyIntervention {
  /** Therapy type */
  type: 'physical' | 'occupational' | 'psychological' | 'cognitive' | 'cellular' | 'other';

  /** Specific therapy name */
  name: string;

  /** Description */
  description: string;

  /** Frequency (sessions per week) */
  frequency: number;

  /** Session duration in minutes */
  sessionDuration: number;

  /** Total sessions planned */
  totalSessions: number;

  /** Provider */
  provider?: string;

  /** Location */
  location?: string;

  /** Goals */
  goals: string[];
}

/**
 * Monitoring requirement
 */
export interface MonitoringRequirement {
  /** What to monitor */
  parameter: string;

  /** Frequency */
  frequency: string;

  /** Method */
  method: string;

  /** Normal range */
  normalRange?: string;

  /** Alert thresholds */
  alertThresholds?: {
    low?: number;
    high?: number;
  };

  /** Action if abnormal */
  actionIfAbnormal: string;
}

/**
 * Follow-up schedule
 */
export interface FollowUpSchedule {
  /** Date/time of follow-up */
  date: Date;

  /** Type of follow-up */
  type: 'in-person' | 'telemedicine' | 'phone' | 'lab-only';

  /** Purpose */
  purpose: string;

  /** Tests/assessments to perform */
  assessments: string[];

  /** Provider */
  provider?: string;

  /** Completed? */
  completed?: boolean;

  /** Notes from visit */
  notes?: string;
}

/**
 * Emergency plan
 */
export interface EmergencyPlan {
  /** Emergency contacts */
  contacts: {
    name: string;
    relationship: string;
    phone: string;
    email?: string;
  }[];

  /** Emergency symptoms to watch for */
  warningSymptoms: string[];

  /** Actions to take */
  immediateActions: string[];

  /** When to call 911 */
  call911If: string[];

  /** Hospital preference */
  preferredHospital?: string;

  /** Special considerations */
  specialConsiderations?: string[];
}

// ============================================================================
// Medical Equipment
// ============================================================================

/**
 * Temporal Vital Signs Monitor reading
 */
export interface TVSMReading {
  /** Reading ID */
  id: string;

  /** Patient ID */
  patientId: string;

  /** Timestamp */
  timestamp: Date;

  /** Standard vital signs */
  vitalSigns: VitalSigns;

  /** Temporal vital signs */
  temporalVitals: TemporalVitalSigns;

  /** Device ID */
  deviceId: string;

  /** Data quality indicator (0-1) */
  dataQuality: number;

  /** Alerts generated */
  alerts?: Alert[];
}

/**
 * Medical alert
 */
export interface Alert {
  /** Alert ID */
  id: string;

  /** Alert level */
  level: 'info' | 'warning' | 'critical' | 'emergency';

  /** Alert type */
  type: string;

  /** Message */
  message: string;

  /** Timestamp */
  timestamp: Date;

  /** Parameter that triggered alert */
  parameter: string;

  /** Value that triggered alert */
  value: number | string;

  /** Threshold exceeded */
  threshold?: number | string;

  /** Acknowledged? */
  acknowledged?: boolean;

  /** Action taken */
  actionTaken?: string;
}

/**
 * Temporal MRI scan result
 */
export interface TMRIScan {
  /** Scan ID */
  id: string;

  /** Patient ID */
  patientId: string;

  /** Scan date */
  scanDate: Date;

  /** Scan type */
  scanType: 'structural' | 'temporal-state' | 'cellular-age' | 'neural-pathway' | 'comprehensive';

  /** Field strength */
  fieldStrength: '3T' | '7T';

  /** Scan quality */
  quality: 'excellent' | 'good' | 'adequate' | 'poor';

  /** Findings */
  findings: string[];

  /** Temporal abnormalities detected */
  temporalAbnormalities: boolean;

  /** Cellular age map available */
  cellularAgeMap?: boolean;

  /** Images */
  images?: {
    url: string;
    type: string;
    description: string;
  }[];

  /** Radiologist report */
  radiologistReport?: string;

  /** Clinical significance */
  clinicalSignificance: string;
}

// ============================================================================
// Laboratory Results
// ============================================================================

/**
 * Temporal laboratory panel
 */
export interface TemporalLabPanel {
  /** Panel ID */
  id: string;

  /** Patient ID */
  patientId: string;

  /** Collection date */
  collectionDate: Date;

  /** Results date */
  resultsDate: Date;

  /** Complete Blood Count */
  cbc?: {
    wbc: number;
    rbc: number;
    hemoglobin: number;
    hematocrit: number;
    platelets: number;
  };

  /** Temporal Antibody Panel */
  temporalAntibodies?: {
    count: number;
    types: string[];
    status: 'normal' | 'elevated' | 'low';
  };

  /** Cellular Age Analysis */
  cellularAge?: {
    telomereLength: number;
    epigeneticAge: number;
    senescentCellPercentage: number;
  };

  /** Chronological Alignment Index */
  chronologicalAlignment?: number;

  /** DNA Temporal Strain */
  dnaTemporalStrain?: {
    strainLevel: number;
    damage: 'none' | 'minimal' | 'moderate' | 'severe';
  };

  /** Other results */
  otherResults?: Record<string, any>;

  /** Abnormal flags */
  abnormalFlags: string[];

  /** Clinical interpretation */
  interpretation: string;
}

// ============================================================================
// Health Records
// ============================================================================

/**
 * Temporal health record
 */
export interface TemporalHealthRecord {
  /** Record ID */
  id: string;

  /** Patient information */
  patient: PatientInfo;

  /** Created date */
  createdDate: Date;

  /** Last updated */
  lastUpdated: Date;

  /** Temporal exposure history */
  exposureHistory: TemporalExposure[];

  /** Medical conditions */
  conditions: {
    condition: string;
    diagnosisDate: Date;
    status: 'active' | 'resolved' | 'managed';
    notes?: string;
  }[];

  /** All diagnoses */
  diagnoses: (TSSDiagnosis | CellularAgeAnalysis | MemoryCoherenceResult | CSSAssessment)[];

  /** Treatment history */
  treatments: TreatmentPlan[];

  /** Laboratory results */
  labResults: TemporalLabPanel[];

  /** Imaging studies */
  imaging: TMRIScan[];

  /** Vital signs history */
  vitalSigns: TVSMReading[];

  /** Medications */
  medications: Medication[];

  /** Allergies */
  allergies: string[];

  /** Emergency contacts */
  emergencyContacts: EmergencyPlan['contacts'];

  /** Access permissions */
  accessPermissions?: {
    providerId: string;
    level: 'read' | 'write' | 'full';
    grantedDate: Date;
  }[];
}

// ============================================================================
// Statistical and Predictive Types
// ============================================================================

/**
 * Risk assessment for temporal travel
 */
export interface TemporalRiskAssessment {
  /** Patient ID */
  patientId: string;

  /** Assessment date */
  assessmentDate: Date;

  /** Overall risk level */
  overallRisk: 'low' | 'moderate' | 'high' | 'very-high' | 'contraindicated';

  /** Risk factors */
  riskFactors: {
    factor: string;
    severity: 'low' | 'moderate' | 'high';
    description: string;
  }[];

  /** TSS risk */
  tssRisk: number;

  /** CAD risk */
  cadRisk: number;

  /** Memory disorder risk */
  memoryRisk: number;

  /** CSS risk */
  cssRisk: number;

  /** Recommendations */
  recommendations: string[];

  /** Precautions */
  precautions: string[];

  /** Clearance status */
  clearanceStatus: 'cleared' | 'cleared-with-conditions' | 'not-cleared';

  /** Special monitoring required */
  specialMonitoring?: string[];
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Medical constants and thresholds
 */
export const MEDICAL_CONSTANTS = {
  /** Normal Temporal Stress Index range */
  TSI_NORMAL: [0, 30],
  TSI_CRITICAL: 70,

  /** Normal Cellular Age Delta */
  AGE_DELTA_NORMAL: 0.05, // ±5%
  AGE_DELTA_TREATMENT: 0.15, // ±15%
  AGE_DELTA_CRITICAL: 0.30, // ±30%

  /** Memory Coherence Index thresholds */
  MCI_NORMAL: 0.95,
  MCI_MILD: 0.80,
  MCI_MODERATE: 0.60,

  /** Telomere length (base pairs) */
  TELOMERE_NORMAL_MIN: 8000,
  TELOMERE_NORMAL_MAX: 15000,
  TELOMERE_CRITICAL_MIN: 5000,
  TELOMERE_CRITICAL_MAX: 20000,

  /** Temporal Antibody Count (per μL) */
  ANTIBODY_NORMAL_MIN: 500,
  ANTIBODY_NORMAL_MAX: 2000,
  ANTIBODY_CRITICAL: 200,

  /** CSS Score thresholds */
  CSS_MILD: 30,
  CSS_MODERATE: 60,
  CSS_SEVERE: 90,

  /** Chronological Alignment normal range */
  ALIGNMENT_NORMAL: 0.90,
  ALIGNMENT_CRITICAL: 0.70,
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-TIME-028 error codes
 */
export enum MedicalErrorCode {
  INVALID_PATIENT_DATA = 'M001',
  DIAGNOSTIC_FAILURE = 'M002',
  TREATMENT_CONTRAINDICATED = 'M003',
  INSUFFICIENT_DATA = 'M004',
  EQUIPMENT_MALFUNCTION = 'M005',
  CRITICAL_CONDITION = 'M006',
  EMERGENCY_PROTOCOL_ACTIVATED = 'M007',
  ACCESS_DENIED = 'M008',
  DATA_INTEGRITY_ERROR = 'M009',
  TEMPORAL_EXPOSURE_EXCEEDED = 'M010',
}

/**
 * Temporal medical error
 */
export class TemporalMedicalError extends Error {
  constructor(
    public code: MedicalErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'TemporalMedicalError';
  }
}

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Result type for medical operations
 */
export type MedicalResult<T, E = Error> =
  | { success: true; data: T }
  | { success: false; error: E };

/**
 * Async medical result
 */
export type AsyncMedicalResult<T, E = Error> = Promise<MedicalResult<T, E>>;

// ============================================================================
// Export All
// ============================================================================

export type {
  // Patient
  PatientInfo,
  TemporalExposure,
  Medication,

  // Vital Signs
  VitalSigns,
  TemporalVitalSigns,

  // TSS
  TSSParameters,
  TSSDiagnosis,

  // Cellular Age
  CellularAgeParameters,
  CellularAgeAnalysis,

  // Memory
  MemoryAssessmentParameters,
  MemoryCoherenceResult,

  // CSS
  CSSParameters,
  CSSAssessment,

  // Treatment
  TreatmentPlan,
  MedicationPrescription,
  TherapyIntervention,
  MonitoringRequirement,
  FollowUpSchedule,
  EmergencyPlan,

  // Equipment
  TVSMReading,
  Alert,
  TMRIScan,

  // Laboratory
  TemporalLabPanel,

  // Records
  TemporalHealthRecord,

  // Risk
  TemporalRiskAssessment,
};

export { MEDICAL_CONSTANTS, MedicalErrorCode, TemporalMedicalError };
