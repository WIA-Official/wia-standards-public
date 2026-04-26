/**
 * WIA-MED-015: Clinical Decision Support Standard - TypeScript Type Definitions
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

export type Timestamp = string;
export type PatientID = string;
export type AlertID = string;
export type MedicationID = string;

// ============================================================================
// Enums
// ============================================================================

export enum AlertSeverity {
  CRITICAL = 'critical',
  HIGH = 'high',
  MEDIUM = 'medium',
  LOW = 'low',
  INFO = 'info'
}

export enum AlertType {
  DRUG_INTERACTION = 'drug_interaction',
  ALLERGY = 'allergy',
  CONTRAINDICATION = 'contraindication',
  DOSAGE_WARNING = 'dosage_warning',
  DUPLICATE_THERAPY = 'duplicate_therapy',
  RENAL_ADJUSTMENT = 'renal_adjustment',
  HEPATIC_ADJUSTMENT = 'hepatic_adjustment',
  PREGNANCY_WARNING = 'pregnancy_warning'
}

export enum Gender {
  MALE = 'male',
  FEMALE = 'female',
  OTHER = 'other'
}

export enum DecisionType {
  DIAGNOSTIC = 'diagnostic',
  THERAPEUTIC = 'therapeutic',
  PREVENTIVE = 'preventive',
  MONITORING = 'monitoring'
}

// ============================================================================
// Patient Information
// ============================================================================

export interface PatientInfo {
  patient_id: PatientID;
  age: number;
  gender: Gender;
  weight_kg?: number;
  height_cm?: number;
  allergies?: string[];
  conditions?: string[];
  renal_function?: {
    creatinine_mg_dl: number;
    gfr_ml_min: number;
  };
  pregnancy_status?: boolean;
}

// ============================================================================
// Medication
// ============================================================================

export interface Medication {
  medication_id?: MedicationID;
  name: string;
  generic_name?: string;
  dose: string;
  dose_value?: number;
  dose_unit?: string;
  route: string;
  frequency: string;
  start_date?: Timestamp;
  end_date?: Timestamp;
  prescriber?: string;
}

// ============================================================================
// Drug Interaction
// ============================================================================

export interface DrugInteraction {
  interaction_id: string;
  severity: AlertSeverity;
  drugs: string[];
  description: string;
  mechanism?: string;
  clinical_effect?: string;
  management?: string[];
  references?: string[];
}

// ============================================================================
// Clinical Alert
// ============================================================================

export interface ClinicalAlert {
  alert_id: AlertID;
  type: AlertType;
  severity: AlertSeverity;
  title: string;
  message: string;
  recommendations?: string[];
  evidence_level?: 'A' | 'B' | 'C' | 'D';
  source?: string;
  timestamp: Timestamp;
  acknowledged?: boolean;
  dismissed?: boolean;
}

// ============================================================================
// Diagnostic Suggestion
// ============================================================================

export interface DiagnosticSuggestion {
  diagnosis: string;
  icd_code?: string;
  probability: number;
  supporting_evidence: string[];
  recommended_tests?: string[];
  differential_diagnoses?: string[];
}

// ============================================================================
// Treatment Recommendation
// ============================================================================

export interface TreatmentRecommendation {
  treatment: string;
  indication: string;
  evidence_level: 'A' | 'B' | 'C' | 'D';
  guideline_source?: string;
  alternatives?: string[];
  contraindications?: string[];
  monitoring_required?: string[];
}

// ============================================================================
// Risk Assessment
// ============================================================================

export interface RiskAssessment {
  risk_type: string;
  score: number;
  risk_level: 'very_low' | 'low' | 'moderate' | 'high' | 'very_high';
  factors: Array<{
    factor: string;
    weight: number;
    present: boolean;
  }>;
  recommendations: string[];
  timeframe?: string;
}

// ============================================================================
// API Requests
// ============================================================================

export interface InteractionCheckRequest {
  patient: PatientInfo;
  medications: Medication[];
  new_medication?: Medication;
}

export interface DiagnosticSuggestionRequest {
  patient: PatientInfo;
  symptoms: string[];
  vital_signs?: {
    temperature_c?: number;
    heart_rate_bpm?: number;
    blood_pressure_systolic?: number;
    blood_pressure_diastolic?: number;
    respiratory_rate?: number;
    oxygen_saturation?: number;
  };
  lab_results?: Record<string, any>;
}

export interface TreatmentRecommendationRequest {
  patient: PatientInfo;
  diagnosis: string;
  severity?: string;
  comorbidities?: string[];
}

export interface RiskAssessmentRequest {
  patient: PatientInfo;
  risk_type: string;
  clinical_data: Record<string, any>;
}

// ============================================================================
// API Responses
// ============================================================================

export interface APIResponse<T = any> {
  status: number;
  success: boolean;
  message: string;
  data?: T;
  error?: {
    code: string;
    message: string;
    details?: any;
  };
  timestamp: Timestamp;
}

export interface InteractionCheckResponse {
  interactions: DrugInteraction[];
  alerts: ClinicalAlert[];
  safe_to_prescribe: boolean;
  requires_monitoring: boolean;
}

export interface DiagnosticSuggestionResponse {
  suggestions: DiagnosticSuggestion[];
  recommended_tests: string[];
  urgency_level: 'emergency' | 'urgent' | 'routine';
}

export interface TreatmentRecommendationResponse {
  recommendations: TreatmentRecommendation[];
  guidelines_applied: string[];
  patient_specific_considerations: string[];
}

export interface RiskAssessmentResponse {
  assessments: RiskAssessment[];
  overall_risk: 'low' | 'moderate' | 'high';
  interventions_recommended: string[];
}

// ============================================================================
// Clinical Guidelines
// ============================================================================

export interface ClinicalGuideline {
  guideline_id: string;
  title: string;
  organization: string;
  version: string;
  published_date: Timestamp;
  evidence_level: 'A' | 'B' | 'C' | 'D';
  recommendations: string[];
  applicability_criteria?: string[];
}

// ============================================================================
// Order Entry
// ============================================================================

export interface MedicationOrder {
  order_id: string;
  patient_id: PatientID;
  medication: Medication;
  indication: string;
  prescriber_id: string;
  status: 'pending' | 'verified' | 'dispensed' | 'administered' | 'cancelled';
  alerts_generated: ClinicalAlert[];
  override_reason?: string;
  timestamp: Timestamp;
}

// ============================================================================
// Audit Log
// ============================================================================

export interface AuditLog {
  log_id: string;
  event_type: string;
  user_id: string;
  patient_id?: PatientID;
  alert_id?: AlertID;
  action: string;
  timestamp: Timestamp;
  details?: Record<string, any>;
}
