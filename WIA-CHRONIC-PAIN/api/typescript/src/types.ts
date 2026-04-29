/**
 * WIA-CHRONIC-PAIN TypeScript Type Definitions
 * Neuroplasticity Reversal Chronic Pain Standard
 *
 * @version 1.0.0
 * @license MIT
 * 弘益人間 (홍익인간) - Benefit All Humanity
 */

// ============================================================================
// Core Types
// ============================================================================

export type PainType = 'nociceptive' | 'neuropathic' | 'nociplastic' | 'mixed';

export type SensitizationLevel = 'none' | 'mild' | 'moderate' | 'severe';

export type Laterality = 'left' | 'right' | 'bilateral' | 'midline';

export type DiurnalPattern = 'constant' | 'morning_worse' | 'evening_worse' | 'night_worse' | 'variable';

export type PainQuality = 'burning' | 'shooting' | 'stabbing' | 'aching' | 'throbbing' | 'tingling' | 'numbness' | 'electric' | 'pressure' | 'cramping';

export type TreatmentCategory = 'neuromodulation' | 'behavioral' | 'pharmacological' | 'physical' | 'interventional';

export type NeuromodulationType = 'rtms' | 'tdcs' | 'tens' | 'scs' | 'dbs' | 'focused_ultrasound';

export type MedicationClass = 'opioid' | 'nsaid' | 'antidepressant' | 'anticonvulsant' | 'muscle_relaxant' | 'topical' | 'other';

export type TreatmentStatus = 'not_started' | 'in_progress' | 'completed' | 'maintenance';

export type TreatmentResponse = 'none' | 'partial' | 'good' | 'excellent';

export type OpioidTaperStatus = 'not_applicable' | 'stable' | 'tapering' | 'completed';

export type RiskLevel = 'low' | 'moderate' | 'high' | 'very_high';

export type CPMStatus = 'intact' | 'reduced' | 'absent' | 'paradoxical';

export type TemporalSummationLevel = 'normal' | 'enhanced' | 'severely_enhanced';

export type CatastrophizingLevel = 'low' | 'moderate' | 'high' | 'clinical';

export type DepressionSeverity = 'minimal' | 'mild' | 'moderate' | 'moderately_severe' | 'severe';

export type AnxietySeverity = 'minimal' | 'mild' | 'moderate' | 'severe';

// ============================================================================
// Pain Assessment
// ============================================================================

export interface NRSScore {
  current: number; // 0-10
  average_7days?: number;
  worst_7days?: number;
  best_7days?: number;
}

export interface VASScore {
  current: number; // 0-100
  average_7days?: number;
  worst_7days?: number;
  best_7days?: number;
}

export interface McGillScore {
  sensory?: number; // 0-42
  affective?: number; // 0-14
  evaluative?: number; // 0-5
  miscellaneous?: number; // 0-17
  total_ppi?: number; // 0-78
  descriptors?: string[];
}

export interface BriefPainInventory {
  pain_severity: number; // 0-10
  pain_interference: number; // 0-10
  general_activity?: number;
  mood?: number;
  walking?: number;
  work?: number;
  relations?: number;
  sleep?: number;
  enjoyment?: number;
}

export interface PainLocation {
  body_region: string;
  icd_code?: string;
  laterality?: Laterality;
  referred?: boolean;
  intensity?: number; // 0-10
}

export interface PainAssessment {
  nrs?: NRSScore;
  vas?: VASScore;
  mcgill?: McGillScore;
  brief_pain_inventory?: BriefPainInventory;
  duration_months: number;
  onset_date?: string;
  locations: PainLocation[];
  pain_quality: PainQuality[];
  aggravating_factors?: string[];
  relieving_factors?: string[];
  diurnal_pattern?: DiurnalPattern;
}

// ============================================================================
// Central Sensitization
// ============================================================================

export interface CSIScore {
  part_a: number; // 0-100
  part_b?: number; // Number of conditions
  interpretation: SensitizationLevel | 'subclinical' | 'extreme';
}

export interface ThermalDetection {
  cold_detection?: number; // °C
  warm_detection?: number;
  cold_pain?: number;
  heat_pain?: number;
}

export interface MechanicalDetection {
  mdt?: number; // Mechanical detection threshold (mN)
  mpt?: number; // Mechanical pain threshold (mN)
  mps?: number; // Mechanical pain sensitivity
  dma?: number; // Dynamic mechanical allodynia
}

export interface PressurePain {
  ppt?: number; // Pressure pain threshold (kPa)
  ppt_remote?: number; // Remote PPT
}

export interface QSTProfile {
  thermal_detection?: ThermalDetection;
  mechanical_detection?: MechanicalDetection;
  pressure_pain?: PressurePain;
  vibration?: number;
  z_scores?: Record<string, number>;
}

export interface TemporalSummation {
  present: boolean;
  ratio?: number; // Pain rating 10th/1st stimulus
  interpretation?: TemporalSummationLevel;
}

export interface ConditionedPainModulation {
  cpm_effect: number; // Percentage change
  status: CPMStatus;
  conditioning_stimulus?: string;
  test_stimulus?: string;
}

export interface Allodynia {
  mechanical?: boolean;
  thermal_cold?: boolean;
  thermal_heat?: boolean;
}

export interface Hyperalgesia {
  primary?: boolean; // At injury site
  secondary?: boolean; // Beyond injury site
  widespread?: boolean; // Generalized
}

export interface CentralSensitization {
  csi_score: CSIScore;
  qst_profile?: QSTProfile;
  temporal_summation?: TemporalSummation;
  conditioned_pain_modulation?: ConditionedPainModulation;
  allodynia?: Allodynia;
  hyperalgesia?: Hyperalgesia;
  neuroplasticity_index: number; // 0-100, higher = more maladaptive
}

// ============================================================================
// Neuroimaging
// ============================================================================

export interface RegionChange {
  volume_change: number; // % change from normative
  status: 'normal' | 'reduced' | 'increased';
}

export interface GrayMatterChanges {
  acc?: RegionChange; // Anterior Cingulate Cortex
  insula?: RegionChange;
  pfc?: RegionChange & { dlpfc_volume?: number; mpfc_volume?: number };
  thalamus?: RegionChange;
  pag?: RegionChange; // Periaqueductal Gray
  s1_s2?: { reorganization?: boolean; expansion?: number };
}

export interface FunctionalConnectivity {
  dmn_salience?: number; // Default Mode - Salience network
  pain_matrix_connectivity?: number;
  acc_pag_connectivity?: number; // Descending modulation
}

export interface DefaultModeNetwork {
  activity_at_rest?: number;
  deactivation_during_pain?: number;
  status?: 'normal' | 'disrupted';
}

export interface SpectralAnalysis {
  theta_power?: number;
  alpha_power?: number;
  beta_power?: number;
  gamma_power?: number;
  thalamo_cortical_dysrhythmia?: boolean;
}

export interface Neuroimaging {
  scan_date?: string;
  modality?: 'MRI' | 'fMRI' | 'PET' | 'EEG' | 'MEG';
  gray_matter_changes?: GrayMatterChanges;
  functional_connectivity?: FunctionalConnectivity;
  default_mode_network?: DefaultModeNetwork;
  spectral_analysis?: SpectralAnalysis;
}

// ============================================================================
// Psychosocial
// ============================================================================

export interface PainCatastrophizing {
  total: number; // 0-52
  rumination?: number; // 0-16
  magnification?: number; // 0-12
  helplessness?: number; // 0-24
  level: CatastrophizingLevel;
}

export interface Kinesiophobia {
  total: number; // 17-68
  activity_avoidance?: number;
  harm_beliefs?: number;
  level: 'low' | 'moderate' | 'high';
}

export interface FearAvoidance {
  fabq_physical?: number; // 0-24
  fabq_work?: number; // 0-42
}

export interface Depression {
  phq9_score: number; // 0-27
  severity: DepressionSeverity;
}

export interface Anxiety {
  gad7_score: number; // 0-21
  severity: AnxietySeverity;
}

export interface Sleep {
  psqi_score?: number; // 0-21
  insomnia_severity?: number; // 0-28
  pain_disrupted_sleep?: boolean;
}

export interface QualityOfLife {
  sf36_physical?: number; // 0-100
  sf36_mental?: number; // 0-100
  eq5d_index?: number; // -0.5 to 1.0
}

export interface SelfEfficacy {
  pain_self_efficacy: number; // 0-60
  level: 'low' | 'moderate' | 'high';
}

export interface Psychosocial {
  pain_catastrophizing: PainCatastrophizing;
  kinesiophobia?: Kinesiophobia;
  fear_avoidance?: FearAvoidance;
  depression?: Depression;
  anxiety?: Anxiety;
  sleep?: Sleep;
  quality_of_life?: QualityOfLife;
  self_efficacy?: SelfEfficacy;
  social_support?: {
    perceived_support?: number;
    functional_status?: 'working' | 'disabled' | 'retired' | 'limited_work';
  };
}

// ============================================================================
// Treatment
// ============================================================================

export interface Medication {
  name: string;
  class: MedicationClass;
  dose: string;
  frequency: string;
  effectiveness?: number; // 0-10
  side_effects?: string[];
}

export interface OpioidUse {
  current_use: boolean;
  mme_daily?: number; // Morphine Milligram Equivalents
  duration_months?: number;
  opioid_risk_tool?: number; // 0-26
  tapering_status?: OpioidTaperStatus;
}

export interface Intervention {
  procedure: string;
  date: string;
  location?: string;
  response: TreatmentResponse;
  duration_relief_weeks?: number;
}

export interface NeuromodulationSession {
  type: NeuromodulationType;
  target_region?: string;
  parameters?: Record<string, unknown>;
  sessions_completed: number;
  response?: TreatmentResponse;
}

export interface BehavioralTherapy {
  cbt?: {
    sessions_completed: number;
    current_status: TreatmentStatus;
  };
  act?: {
    sessions_completed: number;
    current_status: TreatmentStatus;
  };
  mindfulness?: {
    type?: 'mbsr' | 'mbct' | 'general_meditation';
    practice_minutes_week?: number;
  };
  pain_neuroscience_education?: {
    completed: boolean;
    understanding_score?: number; // 0-100
  };
}

export interface PhysicalTherapy {
  physical_therapy?: {
    sessions_completed: number;
    frequency_per_week?: number;
    focus?: string[];
  };
  exercise_program?: {
    type: string[];
    frequency_per_week: number;
    duration_minutes: number;
    adherence_percentage?: number;
  };
  graded_exposure?: {
    in_progress: boolean;
    current_level?: number;
  };
}

export interface Treatment {
  current_medications: Medication[];
  opioid_use?: OpioidUse;
  interventional?: Intervention[];
  neuromodulation?: NeuromodulationSession[];
  behavioral?: BehavioralTherapy;
  physical?: PhysicalTherapy;
  response_history?: Array<{
    treatment: string;
    start_date: string;
    end_date?: string;
    pain_change_percent: number;
    function_change_percent?: number;
    reason_discontinued?: string;
  }>;
}

// ============================================================================
// Chronic Pain Profile (Root Entity)
// ============================================================================

export interface ChronicPainProfile {
  '@context': string[];
  type: string[];
  id: string;
  patient_id: string;
  pain_type: PainType;
  assessment_date: string;
  pain_assessment: PainAssessment;
  central_sensitization?: CentralSensitization;
  neuroimaging?: Neuroimaging;
  psychosocial?: Psychosocial;
  treatment?: Treatment;
}

// ============================================================================
// API Request/Response Types
// ============================================================================

export interface AssessmentRequest {
  patient_id: string;
  pain_type: PainType;
  pain_scores?: {
    nrs_current?: number;
    vas_current?: number;
    bpi_severity?: number;
    bpi_interference?: number;
  };
  duration_months?: number;
  locations?: Partial<PainLocation>[];
  sensitization_data?: {
    csi_score?: number;
    temporal_summation_present?: boolean;
  };
  psychosocial_data?: Partial<Psychosocial>;
}

export interface AssessmentResponse {
  assessment_id: string;
  patient_id: string;
  created_at: string;
  profile: ChronicPainProfile;
  neuroplasticity_index: number;
  recommendations: TreatmentRecommendations;
  risk_scores: {
    chronification_risk: number;
    opioid_risk: number;
    disability_risk: number;
  };
}

export interface SensitizationResponse {
  patient_id: string;
  assessment_date: string;
  csi_score: CSIScore;
  temporal_summation?: TemporalSummation;
  cpm?: ConditionedPainModulation;
  neuroplasticity_index: number;
  sensitization_level: SensitizationLevel;
  reversal_potential: number;
}

export interface NeuromodulationPlanRequest {
  patient_id: string;
  pain_profile_id?: string;
  target_symptoms?: string[];
  previous_treatments?: Array<{ type: string; response: TreatmentResponse }>;
  contraindications?: string[];
  preferences?: {
    home_based?: boolean;
    clinic_based?: boolean;
    max_sessions_per_week?: number;
  };
}

export interface NeuromodulationPlan {
  patient_id: string;
  plan_id: string;
  primary_modality: {
    type: NeuromodulationType;
    target_region: string;
    parameters: Record<string, unknown>;
    frequency: string;
    duration_weeks: number;
    expected_sessions: number;
  };
  adjunct_modalities?: Array<{
    type: NeuromodulationType;
    parameters: Record<string, unknown>;
  }>;
  expected_outcomes: {
    pain_reduction_percent: number;
    function_improvement_percent: number;
  };
  evidence_level: string;
  monitoring_schedule: Array<{ week: number; assessments: string[] }>;
}

export interface TreatmentRecommendation {
  category: TreatmentCategory;
  treatment: string;
  rationale: string;
  evidence_level: string;
  priority: number;
}

export interface TreatmentRecommendations {
  primary_recommendations: TreatmentRecommendation[];
  opioid_sparing_strategies?: string[];
  behavioral_interventions?: Array<{
    type: string;
    sessions_recommended: number;
    priority: number;
  }>;
  exercise_prescription?: {
    type: string[];
    frequency: string;
    intensity: string;
    duration: string;
  };
  monitoring_plan?: {
    frequency: string;
    measures: string[];
  };
}

export interface NeuroplasticityIndex {
  patient_id: string;
  current_index: number; // 0-100
  components: {
    central_sensitization: number;
    gray_matter_changes: number;
    functional_connectivity: number;
    psychosocial_factors: number;
  };
  trend: 'improving' | 'stable' | 'worsening';
  reversal_potential: number;
  reversal_timeline_months?: number;
  history?: Array<{ date: string; index: number }>;
}

export interface OpioidRiskResponse {
  patient_id: string;
  current_mme: number;
  opioid_risk_tool_score: number;
  risk_level: RiskLevel;
  tapering_readiness: {
    score: number;
    barriers: string[];
    facilitators: string[];
  };
  recommendations: string[];
}

export interface TaperPlan {
  patient_id: string;
  current_mme: number;
  target_mme: number;
  duration_weeks: number;
  schedule: Array<{
    week: number;
    mme: number;
    percentage_reduction: number;
  }>;
  support_interventions: string[];
  monitoring_schedule: Array<{ week: number; assessments: string[] }>;
}

export interface ChronificationPrediction {
  patient_id: string;
  prediction_date: string;
  chronification_probability: number; // 0-100
  risk_level: RiskLevel;
  contributing_factors: Array<{
    factor: string;
    contribution: number;
  }>;
  preventive_actions: string[];
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

export interface WiaChronicPainError {
  code: string;
  message: string;
  details?: unknown;
}

export const ErrorCodes = {
  INVALID_PATIENT_ID: 'CP-001',
  INVALID_PAIN_TYPE: 'CP-002',
  PAIN_SCORES_OUT_OF_RANGE: 'CP-003',
  INVALID_DURATION: 'CP-004',
  MISSING_ASSESSMENT_DATA: 'CP-005',
  NEUROMODULATION_CONTRAINDICATION: 'CP-006',
} as const;

export type ErrorCode = typeof ErrorCodes[keyof typeof ErrorCodes];
