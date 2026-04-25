/**
 * WIA-CRYO-010 TypeScript Type Definitions
 * Version: 2.0.0
 *
 * 弘益人間 (Hongik Ingan) - Benefit All Humanity
 */

// ==================== Core Types ====================

export type ExperimentType = 'cell' | 'tissue' | 'organ' | 'embryo';

export type CryopreservationMethod =
  | 'slow_freeze'
  | 'vitrification'
  | 'directional'
  | 'ultra_rapid';

export type ViabilityAssessmentMethod =
  | 'trypan_blue'
  | 'flow_cytometry'
  | 'MTT'
  | 'MTS'
  | 'ATP'
  | 'live_dead_staining';

export type ConcentrationUnit = 'percent_v/v' | 'percent_w/v' | 'molar';

export type TemperatureUnit = 'celsius' | 'kelvin';

// ==================== Researcher ====================

export interface Researcher {
  name: string;
  orcid: string; // Format: 0000-0000-0000-0000
  institution: string;
  email: string;
}

// ==================== Sample ====================

export interface Sample {
  type: string;
  source: string;
  quantity: number;
  unit: string;
  passage?: number;
  culture_conditions?: {
    medium?: string;
    temperature?: number;
    co2?: number;
  };
}

// ==================== Temperature Profile ====================

export interface TemperatureDataPoint {
  time: number; // minutes from start
  temperature: number;
  chamber_temperature?: number;
  sample_temperature: number;
}

export interface TemperatureStatistics {
  mean_cooling_rate: number; // degrees per minute
  max_cooling_rate?: number;
  min_cooling_rate?: number;
  temperature_uniformity: number; // 0-1 scale
}

export interface TemperatureProfile {
  unit: TemperatureUnit;
  sampling_rate: number; // measurements per minute
  data_points: TemperatureDataPoint[];
  statistics: TemperatureStatistics;
}

// ==================== Cryoprotectant ====================

export interface CryoprotectantComponent {
  name: string;
  concentration: number;
  unit: ConcentrationUnit;
  grade?: string;
  manufacturer?: string;
  lot_number?: string;
}

export interface Cryoprotectant {
  components: CryoprotectantComponent[];
  osmolality?: number; // mOsm/kg
  pH?: number;
  preparation_date: string; // ISO8601
  expiration_date?: string; // ISO8601
  storage_temperature?: number;
}

// ==================== Equipment ====================

export interface Equipment {
  model: string;
  manufacturer: string;
  serial_number?: string;
  calibration_date: string; // ISO8601
  next_calibration?: string; // ISO8601
  settings?: Record<string, any>;
}

// ==================== Viability ====================

export interface ViabilityMeasurements {
  total_cells: number;
  viable_cells: number;
  dead_cells?: number;
  viability_percentage: number; // 0-100
  membrane_integrity?: number;
  metabolic_activity?: number;
}

export interface ViabilityStatistics {
  mean: number;
  standard_deviation: number;
  standard_error?: number;
  confidence_interval_95?: [number, number];
}

export interface ViabilityAssessment {
  method: ViabilityAssessmentMethod;
  timepoint: number; // hours post-thaw
  biological_replicates: number;
  technical_replicates: number;
  measurements: ViabilityMeasurements;
  statistics: ViabilityStatistics;
  flow_cytometry_data?: {
    live_dead_marker: string;
    positive_cells: number;
    negative_cells: number;
  };
}

// ==================== Protocol ====================

export interface Protocol {
  method: CryopreservationMethod;
  temperature_profile: TemperatureProfile;
  cryoprotectant: Cryoprotectant;
  equipment_settings: Equipment;
  loading_protocol?: {
    method: 'stepwise' | 'single_step';
    duration?: number;
  };
}

// ==================== Quality Control ====================

export interface QualityControl {
  equipment_calibration: string; // ISO8601 date
  contamination_check?: 'negative' | 'positive';
  protocol_deviations?: Array<{
    description: string;
    impact: 'none' | 'minor' | 'major';
    corrective_action?: string;
  }>;
}

// ==================== Metadata ====================

export interface Metadata {
  standard_version: string; // e.g., "WIA-CRYO-010 v2.0"
  data_created: string; // ISO8601
  data_modified?: string; // ISO8601
  created_by: string;
  modified_by?: string;
  institution: string;
  funding?: {
    agency: string;
    grant_number: string;
    project_title?: string;
  };
  ethical_approval?: {
    irb_number: string;
    approval_date: string;
    expiration_date?: string;
  };
  data_classification: 'public' | 'restricted' | 'confidential';
  license: string; // e.g., "CC-BY-4.0"
  related_publications?: string[]; // DOIs
  keywords?: string[];
}

// ==================== Main Experiment Data ====================

export interface ExperimentData {
  standard: string; // "WIA-CRYO-010"
  version: string; // "2.0"
  experiment: {
    id: string; // Format: CRYO-YYYY-NNN
    type: ExperimentType;
    title: string;
    date: string; // ISO8601
    researcher: Researcher;
    location?: {
      facility?: string;
      room?: string;
      equipment?: string;
    };
  };
  sample: Sample;
  protocol: Protocol;
  measurements: {
    pre_freeze_viability: number; // 0-100
    post_thaw_viability: number; // 0-100
    recovery_time?: number;
    functional_tests?: Array<{
      test_name: string;
      timepoint: number;
      method: string;
      results: Record<string, any>;
    }>;
  };
  quality_control: QualityControl;
  metadata?: Metadata;
}

// ==================== Clinical Trial Extensions ====================

export interface ClinicalPatient {
  study_id: string; // NO PHI!
  enrollment_date: string;
  consent_version: string;
  consent_date: string;
  age_range: string; // e.g., "30-35"
  sex: 'M' | 'F' | 'O';
  ethnicity_category?: string;
  bmi_range?: string;
  region?: string;
  facility?: string;
}

export interface ClinicalTrial {
  study_id: string;
  trial_phase: string;
  sponsor: string;
  irb_approval: string;
  patient: ClinicalPatient;
  data_security: {
    encryption: string;
    access_log: boolean;
    audit_trail: boolean;
    phi_removed: boolean;
  };
}

export interface ClinicalExperimentData extends ExperimentData {
  clinical_trial: ClinicalTrial;
}

// ==================== API Response Types ====================

export interface ValidationResult {
  valid: boolean;
  errors?: Array<{
    field: string;
    message: string;
    severity: 'error' | 'warning';
  }>;
  warnings?: Array<{
    field: string;
    message: string;
  }>;
}

export interface SubmitResponse {
  success: boolean;
  experiment_id: string;
  validation: ValidationResult;
  doi?: string;
  url?: string;
}

export interface SearchQuery {
  cell_type?: string;
  cryopreservation_method?: CryopreservationMethod;
  viability_range?: [number, number];
  date_range?: [string, string];
  institution?: string;
  researcher_orcid?: string;
  page?: number;
  per_page?: number;
}

export interface SearchResult {
  total_results: number;
  page: number;
  per_page: number;
  results: Array<{
    id: string;
    title: string;
    authors: string[];
    institution: string;
    date: string;
    viability: number;
    doi?: string;
  }>;
}

// ==================== Client Configuration ====================

export interface ClientConfig {
  apiKey?: string;
  endpoint?: string;
  version?: string;
  timeout?: number;
  retries?: number;
}

// ==================== Export All ====================

export type {
  ExperimentData as default,
};
