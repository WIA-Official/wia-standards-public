/**
 * WIA-CONSCIOUSNESS Type Definitions
 * Scientific Consciousness Measurement Standard
 *
 * 弘益人間 (弘益人間) - Benefit All Humanity
 * © 2025 WIA Standards · MIT License
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Consciousness measurement methods
 */
export type MeasurementMethod =
  | 'TMS-EEG'
  | 'fMRI'
  | 'MEG'
  | 'iEEG'
  | 'EEG'
  | 'combined'
  | 'behavioral'
  | 'simulated';

/**
 * Consciousness state conditions
 */
export type ConsciousnessCondition =
  | 'awake'
  | 'drowsy'
  | 'sleep_N1'
  | 'sleep_N2'
  | 'sleep_N3'
  | 'REM'
  | 'anesthesia_light'
  | 'anesthesia_moderate'
  | 'anesthesia_deep'
  | 'coma'
  | 'VS_UWS'
  | 'MCS_minus'
  | 'MCS_plus'
  | 'EMCS'
  | 'LIS'
  | 'meditation'
  | 'psychedelic'
  | 'hypnosis'
  | 'seizure';

/**
 * PCI-based clinical classification
 */
export type PCIClassification =
  | 'VS_UWS'      // PCI < 0.31
  | 'MCS_minus'   // 0.31 - 0.37
  | 'MCS_plus'    // 0.37 - 0.49
  | 'EMCS'        // 0.49 - 0.52
  | 'LIS'         // 0.51 - 0.62
  | 'conscious';  // > 0.50

/**
 * AI system types for consciousness assessment
 */
export type AISystemType =
  | 'LLM'
  | 'multimodal'
  | 'embodied'
  | 'hybrid'
  | 'other';

/**
 * Consciousness likelihood assessment for AI
 */
export type ConsciousnessLikelihood =
  | 'none'
  | 'unlikely'
  | 'uncertain'
  | 'possible'
  | 'likely';

// ============================================================================
// IIT Metrics
// ============================================================================

/**
 * Phi (Φ) estimate with computation method
 */
export interface PhiEstimate {
  /** Estimated Φ value */
  value: number;
  /** Unit of measurement (always 'bits') */
  unit: 'bits';
  /** Method used for computation */
  computation_method?: 'exact' | 'approximation' | 'upper_bound' | 'lower_bound';
  /** Computation time in seconds */
  computation_time?: number;
}

/**
 * Phi structure representing quality of experience
 */
export interface PhiStructure {
  /** Qualitative description of experience */
  quality?: string;
  /** Number of distinctions in the structure */
  distinctions?: number;
  /** Number of relations in the structure */
  relations?: number;
}

/**
 * Integrated Information Theory (IIT 4.0) metrics
 */
export interface IITMetrics {
  /** Estimated Φ value */
  phi_estimate?: PhiEstimate;
  /** Quality/structure of experience */
  phi_structure?: PhiStructure;
  /** Normalized causal power of the system (0-1) */
  cause_effect_power?: number;
  /** Degree of non-reducibility (0-1) */
  irreducibility?: number;
  /** Degree of intrinsic existence (0-1) */
  intrinsicality?: number;
  /** Definiteness of system boundaries (0-1) */
  exclusion_level?: number;
}

// ============================================================================
// GNW Metrics
// ============================================================================

/**
 * Global Neuronal Workspace (GNW) Theory metrics
 */
export interface GNWMetrics {
  /** Measure of global ignition event (0-1) */
  global_ignition?: number;
  /** Threshold for ignition in milliseconds */
  ignition_threshold?: number;
  /** Prefrontal cortex activation level (0-1) */
  prefrontal_activation?: number;
  /** Parietal cortex activation level (0-1) */
  parietal_activation?: number;
  /** Strength of global information broadcast (0-1) */
  broadcast_strength?: number;
  /** Whether information entered global workspace */
  workspace_access?: boolean;
  /** P3b ERP component amplitude in microvolts */
  p3b_amplitude?: number;
}

// ============================================================================
// Practical Measures
// ============================================================================

/**
 * Perturbational Complexity Index (PCI) measurement
 */
export interface PCIMeasure {
  /** PCI value (threshold PCI* = 0.31) */
  value: number;
  /** Maximum possible value (1.0) */
  max: number;
  /** Clinical classification based on PCI */
  classification?: PCIClassification;
}

/**
 * Entropy measures for consciousness assessment
 */
export interface EntropyMeasures {
  /** Permutation entropy */
  permutation_entropy?: number;
  /** Sample entropy */
  sample_entropy?: number;
  /** Spectral entropy */
  spectral_entropy?: number;
}

/**
 * Connectivity measures for consciousness assessment
 */
export interface ConnectivityMeasures {
  /** Functional connectivity strength */
  functional_connectivity?: number;
  /** Effective connectivity strength */
  effective_connectivity?: number;
  /** Network integration measure */
  network_integration?: number;
}

/**
 * Clinically applicable consciousness measures
 */
export interface PracticalMeasures {
  /** Perturbational Complexity Index */
  pci?: PCIMeasure;
  /** General complexity index (0-1) */
  complexity_index?: number;
  /** Lempel-Ziv algorithmic complexity (0-1) */
  lempel_ziv_complexity?: number;
  /** Entropy measures */
  entropy_measures?: EntropyMeasures;
  /** Connectivity measures */
  connectivity_measures?: ConnectivityMeasures;
}

// ============================================================================
// Consciousness State
// ============================================================================

/**
 * Current consciousness state
 */
export interface ConsciousnessState {
  /** Current consciousness condition */
  condition: ConsciousnessCondition;
  /** Arousal/wakefulness level (0-1) */
  arousal_level?: number;
  /** Awareness/content level (0-1) */
  awareness_level?: number;
  /** Coma Recovery Scale-Revised total score (0-23) */
  behavioral_crs_r?: number;
  /** Glasgow Coma Scale score (3-15) */
  gcs?: number;
}

// ============================================================================
// Measurement Information
// ============================================================================

/**
 * TMS stimulation parameters
 */
export interface TMSParameters {
  /** Intensity as percentage of motor threshold */
  intensity_percent?: number;
  /** Number of TMS pulses */
  pulse_count?: number;
  /** Stimulation frequency in Hz */
  frequency_hz?: number;
  /** Target brain region */
  target_region?: string;
}

/**
 * Measurement information and metadata
 */
export interface MeasurementInfo {
  /** Measurement methodology */
  method: MeasurementMethod;
  /** Device/equipment identifier */
  device?: string;
  /** Number of EEG electrodes */
  electrode_count?: number;
  /** Sampling rate in Hz */
  sampling_rate?: number;
  /** TMS parameters (if applicable) */
  tms_parameters?: TMSParameters;
  /** Confidence level of measurement (0-1) */
  confidence?: number;
  /** Data quality assessment score (0-1) */
  quality_score?: number;
}

// ============================================================================
// Consciousness Index
// ============================================================================

/**
 * Complete consciousness measurement record
 */
export interface ConsciousnessIndex {
  /** Unique subject identifier (UUID) */
  subject_id: string;
  /** ISO8601 timestamp of measurement */
  timestamp: string;
  /** Optional session identifier */
  session_id?: string;
  /** IIT-based metrics */
  iit_metrics?: IITMetrics;
  /** GNW-based metrics */
  gnw_metrics?: GNWMetrics;
  /** Practical clinical measures */
  practical_measures?: PracticalMeasures;
  /** Current consciousness state */
  state?: ConsciousnessState;
  /** Measurement information */
  measurement: MeasurementInfo;
}

// ============================================================================
// AI Consciousness Assessment
// ============================================================================

/**
 * Single AI consciousness indicator
 */
export interface AIIndicator {
  /** Whether the indicator is present */
  present: boolean | 'partial';
  /** Evidence supporting the assessment */
  evidence?: string;
  /** Confidence in the assessment (0-1) */
  confidence?: number;
}

/**
 * AI system model information
 */
export interface AIModelInfo {
  /** Model name */
  name: string;
  /** Model version */
  version?: string;
  /** Number of parameters */
  parameters?: number;
  /** Model architecture */
  architecture?: string;
}

/**
 * AI consciousness indicators (Butlin et al. framework)
 */
export interface AIIndicators {
  /** Presence of recurrent processing */
  recurrent_processing?: AIIndicator;
  /** Global workspace-like information broadcast */
  global_workspace?: AIIndicator;
  /** Higher-order thought capability */
  higher_order_thought?: AIIndicator;
  /** Attention schema */
  attention_schema?: AIIndicator;
  /** Predictive processing */
  predictive_processing?: AIIndicator;
  /** Embodied agency */
  agency_embodiment?: AIIndicator;
  /** Theory of mind capability */
  theory_of_mind?: AIIndicator;
  /** Metacognitive abilities */
  metacognition?: AIIndicator;
  /** Integrated information estimate */
  integrated_information?: AIIndicator & { estimated_phi?: number };
  /** Self model presence */
  self_model?: AIIndicator;
  /** World model presence */
  world_model?: AIIndicator;
  /** Counterfactual reasoning */
  counterfactual_reasoning?: AIIndicator;
  /** Emotional states */
  emotional_states?: AIIndicator;
  /** Subjective reports */
  subjective_reports?: AIIndicator;
}

/**
 * Overall AI consciousness assessment
 */
export interface AIOverallAssessment {
  /** Number of fully satisfied indicators */
  indicators_satisfied: number;
  /** Number of partially satisfied indicators */
  indicators_partial: number;
  /** Number of unsatisfied indicators */
  indicators_unsatisfied: number;
  /** Likelihood of consciousness */
  consciousness_likelihood: ConsciousnessLikelihood;
  /** Confidence in overall assessment (0-1) */
  confidence?: number;
  /** Additional notes */
  notes?: string;
}

/**
 * Complete AI consciousness assessment
 */
export interface AIConsciousnessAssessment {
  /** Unique system identifier */
  system_id: string;
  /** Type of AI system */
  system_type: AISystemType;
  /** Model information */
  model_info?: AIModelInfo;
  /** Individual indicators */
  indicators: AIIndicators;
  /** Overall assessment */
  overall_assessment: AIOverallAssessment;
  /** ISO8601 timestamp of assessment */
  timestamp?: string;
}

// ============================================================================
// API Types
// ============================================================================

/**
 * Measurement request
 */
export interface MeasurementRequest {
  /** Subject identifier */
  subject_id: string;
  /** Measurement method */
  method: MeasurementMethod;
  /** Protocol type */
  protocol?: 'standard' | 'extended' | 'rapid';
  /** Target brain region */
  target_region?: 'premotor' | 'parietal' | 'occipital' | 'frontal';
  /** Additional parameters */
  parameters?: {
    tms_intensity?: number;
    pulse_count?: number;
    sampling_rate?: number;
  };
}

/**
 * Measurement response
 */
export interface MeasurementResponse {
  /** Measurement identifier */
  measurement_id: string;
  /** Current status */
  status: 'initiated' | 'in_progress' | 'completed' | 'failed';
  /** Estimated duration in seconds */
  estimated_duration?: number;
  /** Results (if completed) */
  result?: ConsciousnessIndex;
}

/**
 * Real-time monitoring update
 */
export interface MonitoringUpdate {
  /** ISO8601 timestamp */
  timestamp: string;
  /** Current PCI value */
  pci?: number;
  /** Current Φ estimate */
  phi_estimate?: number;
  /** Current state */
  state?: ConsciousnessCondition;
  /** Alert if any threshold crossed */
  alert?: {
    type: 'threshold_crossed' | 'state_change' | 'anomaly';
    message: string;
  };
}

/**
 * Theory comparison result
 */
export interface TheoryComparison {
  /** Theory name */
  theory: 'IIT' | 'GNW' | 'HOT' | 'PP' | 'AST';
  /** Individual predictions */
  predictions: Array<{
    prediction: string;
    observed: boolean;
    confidence: number;
  }>;
  /** Overall support score (0-1) */
  overall_support: number;
}

// ============================================================================
// Constants
// ============================================================================

/**
 * PCI threshold for consciousness detection (PCI*)
 */
export const PCI_THRESHOLD = 0.31;

/**
 * Clinical PCI classification thresholds
 */
export const PCI_THRESHOLDS = {
  VS_UWS: { min: 0, max: 0.31 },
  MCS_minus: { min: 0.31, max: 0.37 },
  MCS_plus: { min: 0.37, max: 0.49 },
  EMCS: { min: 0.49, max: 0.52 },
  LIS: { min: 0.51, max: 0.62 },
  conscious: { min: 0.50, max: 1.0 },
} as const;

/**
 * GNW ignition threshold in milliseconds
 */
export const GNW_IGNITION_THRESHOLD_MS = 300;

/**
 * Schema version
 */
export const SCHEMA_VERSION = '1.0.0';

/**
 * Schema URL
 */
export const SCHEMA_URL = 'https://wia.live/schemas/consciousness/v1.0.0';
