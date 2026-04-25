/**
 * WIA AI Diagnosis Standard - TypeScript Type Definitions
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

export type Timestamp = string;
export type DiseaseType = 'pneumonia' | 'cancer' | 'diabetes' | 'cardiac' | 'neurological' | 'dermatological' | 'ophthalmological' | 'musculoskeletal';
export type ModelType = 'cnn' | 'transformer' | 'ensemble' | 'decision_tree' | 'svm' | 'random_forest' | 'xgboost';

// ============================================================================
// Diagnosis Types
// ============================================================================

export interface DiagnosisResult {
  format: 'WIA-AI-DIAGNOSIS-v1.0';
  timestamp: Timestamp;
  diagnosis_id: string;
  patient_id: string;
  disease_type: DiseaseType;
  confidence: number;
  model_version: string;
  predictions: Prediction[];
  explanation?: Explanation;
  recommendations?: string[];
  urgency: UrgencyLevel;
  follow_up: FollowUp;
}

export interface Prediction {
  label: string;
  confidence: number;
  icd_code?: string;
  description?: string;
}

export interface Explanation {
  type: ExplanationType;
  summary: string;
  details: ExplanationDetail[];
  visualization_url?: string;
}

export enum ExplanationType {
  GRADCAM = 'gradcam',
  SHAP = 'shap',
  LIME = 'lime',
  ATTENTION = 'attention',
  FEATURE_IMPORTANCE = 'feature_importance',
}

export interface ExplanationDetail {
  feature: string;
  importance: number;
  contribution: 'positive' | 'negative';
  description?: string;
}

export enum UrgencyLevel {
  ROUTINE = 'routine',
  SOON = 'soon',
  URGENT = 'urgent',
  EMERGENCY = 'emergency',
}

export interface FollowUp {
  recommended: boolean;
  timeframe?: string;
  specialist?: string;
  tests?: string[];
}

// ============================================================================
// Model Types
// ============================================================================

export interface AIModel {
  model_id: string;
  name: string;
  model_type: ModelType;
  version: string;
  accuracy: number;
  sensitivity: number;
  specificity: number;
  auc_roc: number;
  training_date: Timestamp;
  approved: boolean;
  certifications: ModelCertification[];
  supported_inputs: InputType[];
  target_diseases: DiseaseType[];
  performance_metrics: PerformanceMetrics;
}

export interface ModelCertification {
  authority: string;
  certification_id: string;
  issue_date: Timestamp;
  expiry_date: Timestamp;
  scope: string;
}

export enum InputType {
  XRAY = 'xray',
  CT = 'ct',
  MRI = 'mri',
  ULTRASOUND = 'ultrasound',
  DERMOSCOPY = 'dermoscopy',
  FUNDUS = 'fundus',
  ECG = 'ecg',
  LAB_RESULTS = 'lab_results',
  VITAL_SIGNS = 'vital_signs',
  SYMPTOMS = 'symptoms',
}

export interface PerformanceMetrics {
  accuracy: number;
  precision: number;
  recall: number;
  f1_score: number;
  auc_roc: number;
  confusion_matrix: number[][];
  validation_dataset: string;
  validation_size: number;
}

// ============================================================================
// Inference Types
// ============================================================================

export interface InferenceRequest {
  request_id?: string;
  patient_id: string;
  input_data: InputData;
  model_id: string;
  return_explanation: boolean;
  priority?: Priority;
  context?: ClinicalContext;
}

export interface InputData {
  images?: ImageInput[];
  vital_signs?: VitalSigns;
  lab_results?: LabResults;
  symptoms?: Symptom[];
  medical_history?: MedicalHistory;
}

export interface ImageInput {
  modality: InputType;
  data: string;
  format: 'base64' | 'url';
  metadata?: ImageMetadata;
}

export interface ImageMetadata {
  acquisition_date?: Timestamp;
  device?: string;
  body_part?: string;
  view?: string;
}

export interface VitalSigns {
  heart_rate?: number;
  blood_pressure_systolic?: number;
  blood_pressure_diastolic?: number;
  temperature?: number;
  respiratory_rate?: number;
  oxygen_saturation?: number;
  weight?: number;
  height?: number;
}

export interface LabResults {
  [key: string]: LabValue;
}

export interface LabValue {
  value: number;
  unit: string;
  reference_min?: number;
  reference_max?: number;
  flag?: 'normal' | 'low' | 'high' | 'critical';
}

export interface Symptom {
  name: string;
  duration_days?: number;
  severity?: 'mild' | 'moderate' | 'severe';
  location?: string;
}

export interface MedicalHistory {
  conditions: string[];
  medications: string[];
  allergies: string[];
  surgeries: string[];
  family_history: string[];
}

export interface ClinicalContext {
  chief_complaint?: string;
  clinical_notes?: string;
  ordering_physician?: string;
  department?: string;
}

export enum Priority {
  LOW = 'low',
  NORMAL = 'normal',
  HIGH = 'high',
  STAT = 'stat',
}

export interface InferenceResponse {
  inference_id: string;
  request_id: string;
  diagnosis_result: DiagnosisResult;
  processing_time_ms: number;
  model_used: string;
  cost?: number;
  audit_trail: AuditEntry[];
}

// ============================================================================
// Audit Types
// ============================================================================

export interface AuditEntry {
  timestamp: Timestamp;
  action: AuditAction;
  actor: string;
  details: string;
}

export enum AuditAction {
  REQUEST_RECEIVED = 'request_received',
  PREPROCESSING = 'preprocessing',
  INFERENCE_STARTED = 'inference_started',
  INFERENCE_COMPLETED = 'inference_completed',
  RESULT_REVIEWED = 'result_reviewed',
  RESULT_APPROVED = 'result_approved',
  RESULT_REJECTED = 'result_rejected',
}

// ============================================================================
// API Types
// ============================================================================

export interface APIResponse<T> {
  success: boolean;
  data?: T;
  error?: APIError;
  timestamp: Timestamp;
  request_id: string;
}

export interface APIError {
  code: string;
  message: string;
  details?: Record<string, unknown>;
}

export interface PaginationParams {
  page: number;
  page_size: number;
  sort_by?: string;
  sort_order?: 'asc' | 'desc';
}

export interface PaginatedResponse<T> {
  items: T[];
  total: number;
  page: number;
  page_size: number;
  total_pages: number;
}

export interface SDKConfig {
  api_key: string;
  base_url?: string;
  timeout_ms?: number;
  debug?: boolean;
}
