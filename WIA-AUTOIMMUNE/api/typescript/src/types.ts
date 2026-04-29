/**
 * WIA-AUTOIMMUNE TypeScript Type Definitions
 * Treg-Microbiome Axis Autoimmune Disease Standard
 *
 * @version 1.0.0
 * @license MIT
 * 弘益人間 (홍익인간) - Benefit All Humanity
 */

// ============================================================================
// Core Types
// ============================================================================

export type DiseaseType = 'RA' | 'SLE' | 'MS' | 'T1D' | 'IBD' | 'PSO' | 'HT' | 'GD';

export type ActivityLevel = 'remission' | 'low' | 'moderate' | 'high' | 'severe';

export type RemissionType = 'clinical' | 'biochemical' | 'deep' | 'drug-free';

export type TreatmentType = 'low_dose_il2' | 'fmt' | 'car_treg' | 'dietary' | 'immunosuppressant' | 'biologic';

export type EvidenceLevel = 'A' | 'B' | 'C' | 'D';

export type FunctionalStatus = 'normal' | 'mildly_impaired' | 'moderately_impaired' | 'severely_impaired';

export type RiskLevel = 'low' | 'moderate' | 'high' | 'very_high';

export type SampleType = 'stool' | 'biopsy' | 'swab';

export type SequencingMethod = '16S' | 'shotgun' | 'metatranscriptomics';

// ============================================================================
// Value Objects
// ============================================================================

export interface MeasurementValue {
  value: number;
  unit: string;
}

export interface RangedMeasurement extends MeasurementValue {
  reference_range?: {
    low: number;
    high: number;
  };
  status?: 'low' | 'normal' | 'high';
}

export interface Autoantibody {
  name: string;
  code: string;
  value: number;
  unit: string;
  positive: boolean;
  titer?: string;
}

// ============================================================================
// Treg Status
// ============================================================================

export interface TregStatus {
  count: RangedMeasurement;
  foxp3_expression: number; // 0-100%
  suppressive_function: number; // 0-100%
  cd25_expression?: number;
  ctla4_expression?: number;
  grail_activity?: number; // 0-100%
  stability_score?: number; // 0-100%
}

export interface TregStatusResponse {
  patient_id: string;
  assessment_date: string;
  treg: TregStatus;
  th17_treg_ratio: number;
  functional_status: FunctionalStatus;
  clinical_interpretation: string;
}

// ============================================================================
// Inflammatory Markers
// ============================================================================

export interface InflammatoryCytokines {
  il6?: number; // pg/mL
  il17?: number; // pg/mL
  tnf_alpha?: number; // pg/mL
  ifn_gamma?: number; // pg/mL
  il1_beta?: number; // pg/mL
  il23?: number; // pg/mL
}

export interface AntiInflammatoryCytokines {
  il10?: number; // pg/mL
  tgf_beta?: number; // ng/mL
  il35?: number; // pg/mL
}

export interface ImmuneMarkers {
  treg: TregStatus;
  th17_treg_ratio: number;
  autoantibodies: Autoantibody[];
  inflammatory_cytokines: InflammatoryCytokines;
  anti_inflammatory_cytokines?: AntiInflammatoryCytokines;
}

// ============================================================================
// Microbiome
// ============================================================================

export interface DiversityIndex {
  shannon: number;
  simpson: number;
  chao1?: number;
  status?: 'low' | 'normal' | 'high';
}

export interface Genus {
  name: string;
  abundance: number;
}

export interface SCFAProducers {
  butyrate_producers: {
    abundance: number;
    genera: Genus[];
  };
  propionate_producers?: {
    abundance: number;
    genera?: Genus[];
  };
  acetate_producers?: {
    abundance: number;
    genera?: Genus[];
  };
}

export interface SCFALevels {
  butyrate: number; // μmol/g
  propionate: number; // μmol/g
  acetate: number; // μmol/g
}

export interface LeakyGutMarkers {
  zonulin?: number; // ng/mL
  lps?: number; // EU/mL
  i_fabp?: number; // pg/mL
  lactulose_mannitol_ratio?: number;
}

export interface Pathobiont {
  name: string;
  abundance: number;
  pathogenic_potential: 'low' | 'moderate' | 'high';
}

export interface Microbiome {
  sample_date: string;
  sample_type: SampleType;
  sequencing_method: SequencingMethod;
  diversity_index: DiversityIndex;
  scfa_producers: SCFAProducers;
  scfa_levels?: SCFALevels;
  dysbiosis_score: number; // 0-100
  leaky_gut_markers?: LeakyGutMarkers;
  pathobionts?: Pathobiont[];
}

export interface MicrobiomeResponse {
  patient_id: string;
  sample_date: string;
  diversity: DiversityIndex;
  scfa_production: {
    butyrate: RangedMeasurement;
    propionate?: RangedMeasurement;
    acetate?: RangedMeasurement;
    overall_status: string;
  };
  dysbiosis: {
    score: number;
    severity: string;
    key_findings: string[];
  };
  leaky_gut?: {
    zonulin: number;
    lps: number;
    status: string;
  };
  treg_modulators?: Record<string, unknown>;
}

// ============================================================================
// Disease Activity
// ============================================================================

export interface DiseaseScores {
  das28?: number; // RA
  das28_crp?: number;
  sledai?: number; // SLE
  sledai_2k?: number;
  edss?: number; // MS
  msfc?: number;
  pasi?: number; // Psoriasis
  mayo_score?: number; // UC
  cdai?: number; // Crohn's
  hba1c?: number; // T1D
}

export interface FlareStatus {
  is_flaring: boolean;
  flare_severity?: 'mild' | 'moderate' | 'severe';
  days_since_last_flare?: number;
  flare_frequency?: number; // per year
}

export interface RemissionStatus {
  in_remission: boolean;
  remission_type?: RemissionType | null;
  duration_months?: number;
}

export interface DiseaseActivity {
  disease_type: DiseaseType;
  activity_level: ActivityLevel;
  scores: DiseaseScores;
  flare_status?: FlareStatus;
  remission?: RemissionStatus;
}

export interface DiseaseActivityResponse {
  patient_id: string;
  disease_type: DiseaseType;
  current_activity: DiseaseActivity;
  history?: DiseaseActivity[];
}

// ============================================================================
// Autoimmune Profile (Root Entity)
// ============================================================================

export interface AutoimmuneProfile {
  '@context': string[];
  type: string[];
  id: string;
  patient_id: string;
  disease_type: DiseaseType;
  assessment_date: string;
  immune_markers: ImmuneMarkers;
  microbiome?: Microbiome;
  disease_activity: DiseaseActivity;
}

// ============================================================================
// Treatment
// ============================================================================

export interface TreatmentRecommendation {
  treatment_type: TreatmentType;
  name: string;
  rationale: string;
  evidence_level: EvidenceLevel;
  priority: number;
}

export interface DietaryIntervention {
  intervention: string;
  target: string;
  expected_benefit: string;
}

export interface MonitoringPlan {
  frequency: string;
  markers_to_track: string[];
}

export interface TreatmentRecommendations {
  primary_recommendations: TreatmentRecommendation[];
  dietary_interventions?: DietaryIntervention[];
  monitoring_plan?: MonitoringPlan;
}

// ============================================================================
// Prediction
// ============================================================================

export interface ContributingFactor {
  factor: string;
  contribution: number;
  trend: 'increasing' | 'stable' | 'decreasing';
}

export interface FlarePrediction {
  patient_id: string;
  prediction_date: string;
  flare_probability: number; // 0-100
  risk_level: RiskLevel;
  contributing_factors: ContributingFactor[];
  preventive_actions: string[];
}

export interface RemissionAssessment {
  remission_probability: number;
  current_status: string;
  factors_supporting: string[];
  factors_against: string[];
  time_to_remission_estimate?: string;
}

// ============================================================================
// API Request/Response Types
// ============================================================================

export interface AssessmentRequest {
  patient_id: string;
  disease_type: DiseaseType;
  immune_panel?: {
    treg_count?: number;
    foxp3_expression?: number;
    suppressive_function?: number;
    th17_count?: number;
    autoantibodies?: Partial<Autoantibody>[];
    cytokines?: Partial<InflammatoryCytokines>;
  };
  microbiome_sample_id?: string;
  clinical_observations?: Record<string, unknown>;
}

export interface AssessmentResponse {
  assessment_id: string;
  patient_id: string;
  created_at: string;
  profile: AutoimmuneProfile;
  recommendations: TreatmentRecommendations;
  risk_scores: {
    flare_risk: number;
    progression_risk: number;
    remission_likelihood: number;
  };
}

export interface TreatmentRecommendationRequest {
  patient_id: string;
  current_profile_id?: string;
  treatment_history?: Array<{
    treatment: string;
    start_date: string;
    end_date?: string;
    response: 'none' | 'partial' | 'good' | 'excellent';
  }>;
  preferences?: {
    avoid_immunosuppressants?: boolean;
    open_to_experimental?: boolean;
    dietary_restrictions?: string[];
  };
}

export interface FlarePredictionRequest {
  patient_id: string;
  include_microbiome?: boolean;
  prediction_horizon_days?: number;
}

export interface RemissionAssessmentRequest {
  patient_id: string;
  current_treatment?: string[];
  target_remission_type?: RemissionType;
}

export interface MicrobiomeSampleRequest {
  sample_date: string;
  sample_type: SampleType;
  sequencing_method: SequencingMethod;
  raw_data_location?: string;
  lab_id?: string;
}

export interface SampleSubmissionResponse {
  sample_id: string;
  status: 'received' | 'processing' | 'completed' | 'failed';
  estimated_completion?: string;
}

// ============================================================================
// Webhook Events
// ============================================================================

export type WebhookEventType =
  | 'assessment.completed'
  | 'flare.predicted'
  | 'remission.achieved'
  | 'microbiome.analyzed'
  | 'treg.critical';

export interface WebhookPayload<T = unknown> {
  event: WebhookEventType;
  timestamp: string;
  data: T;
}

export interface FlareWarningWebhookData {
  patient_id: string;
  flare_probability: number;
  risk_level: RiskLevel;
  recommended_actions: string[];
}

// ============================================================================
// TMAS Score (Treg-Microbiome Axis Score)
// ============================================================================

export interface TMASScore {
  total: number; // 0-100
  treg_component: number;
  microbiome_component: number;
  activity_component: number;
  interpretation: 'excellent' | 'good' | 'impaired' | 'poor' | 'critical';
  calculated_at: string;
}

// ============================================================================
// Client Configuration
// ============================================================================

export type Environment = 'production' | 'staging' | 'development';

export interface ClientConfig {
  apiKey: string;
  clientId?: string;
  environment?: Environment;
  baseUrl?: string;
  timeout?: number;
  retries?: number;
}

// ============================================================================
// Error Types
// ============================================================================

export interface WiaAutoImmuneError {
  code: string;
  message: string;
  details?: unknown;
}

export const ErrorCodes = {
  INVALID_PATIENT_ID: 'AI-001',
  INVALID_DISEASE_TYPE: 'AI-002',
  INCOMPLETE_IMMUNE_PANEL: 'AI-003',
  MICROBIOME_SAMPLE_NOT_FOUND: 'AI-004',
  ASSESSMENT_CONFLICT: 'AI-005',
} as const;

export type ErrorCode = typeof ErrorCodes[keyof typeof ErrorCodes];
