/**
 * WIA-PLASTIC-ENZYME SDK Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @copyright 2025 WIA - World Certification Industry Association
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Enzyme classification types
 */
export type EnzymeClassification =
  | 'PETase'
  | 'MHETase'
  | 'BHETase'
  | 'Cutinase'
  | 'Lipase'
  | 'Esterase';

/**
 * Plastic type codes
 */
export type PlasticType =
  | 'PET'
  | 'PBAT'
  | 'PLA'
  | 'PCL'
  | 'PHA'
  | 'PBS';

/**
 * Plastic form for input material
 */
export type PlasticForm =
  | 'flakes'
  | 'powder'
  | 'film'
  | 'fiber'
  | 'mixed';

/**
 * Quality grade for output products
 */
export type QualityGrade =
  | 'food-contact'
  | 'bottle-grade'
  | 'textile-grade'
  | 'industrial';

/**
 * Certification status
 */
export type CertificationStatus =
  | 'pending'
  | 'WIA-certified'
  | 'rejected';

/**
 * Source domain for enzyme origin
 */
export type SourceDomain =
  | 'bacteria'
  | 'archaea'
  | 'fungi'
  | 'engineered';

/**
 * Environment type for enzyme discovery
 */
export type SourceEnvironment =
  | 'marine'
  | 'soil'
  | 'geothermal'
  | 'wastewater'
  | 'laboratory';

// ============================================================================
// Measurement Types
// ============================================================================

/**
 * Generic measurement with value and unit
 */
export interface Measurement<T = number> {
  value: T;
  unit: string;
}

/**
 * Range definition with min and max
 */
export interface Range {
  min: number;
  max: number;
}

/**
 * Optimal value with acceptable range
 */
export interface OptimalValue {
  optimal: number;
  range: [number, number];
}

// ============================================================================
// Enzyme Types
// ============================================================================

/**
 * Enzyme source information
 */
export interface EnzymeSource {
  organism: string;
  domain?: SourceDomain;
  environment?: SourceEnvironment;
  engineered?: boolean;
  parent_enzyme?: string;
  mutations?: string[];
}

/**
 * Enzyme kinetic parameters
 */
export interface EnzymeKinetics {
  km?: Measurement;
  kcat?: Measurement;
  kcat_km?: Measurement;
}

/**
 * Optimal reaction conditions for enzyme
 */
export interface OptimalConditions {
  temperature_c?: OptimalValue;
  ph?: OptimalValue;
  substrate_loading_percent?: number;
}

/**
 * Enzyme stability parameters
 */
export interface EnzymeStability {
  half_life_hours?: number;
  thermostability_tm_c?: number;
  storage_conditions?: string;
}

/**
 * Enzyme products information
 */
export interface EnzymeProducts {
  primary: ('TPA' | 'MHET' | 'BHET' | 'EG' | 'oligomers')[];
  toxicity_class?: 'non-toxic' | 'low' | 'moderate' | 'high';
}

/**
 * Complete enzyme profile
 */
export interface EnzymeProfile {
  enzyme_id: string;
  name: string;
  classification: EnzymeClassification;
  source: EnzymeSource;
  target_plastics?: PlasticType[];
  kinetics?: EnzymeKinetics;
  optimal_conditions?: OptimalConditions;
  stability?: EnzymeStability;
  products?: EnzymeProducts;
  sequence?: string;
  structure_pdb?: string;
  reference?: string;
  metadata?: Metadata;
}

/**
 * Enzyme matching result
 */
export interface EnzymeMatch {
  enzyme_id: string;
  name: string;
  match_score: number;
  predicted_efficiency: number;
  optimal_concentration_mg_g: number;
  estimated_time_hours: number;
}

/**
 * Enzyme cocktail suggestion
 */
export interface CocktailSuggestion {
  primary: string;
  secondary: string;
  ratio: string;
  synergy_bonus: string;
}

// ============================================================================
// Process Types
// ============================================================================

/**
 * Plastic input for degradation process
 */
export interface PlasticInput {
  type: PlasticType;
  form?: PlasticForm;
  weight_kg: number;
  crystallinity_percent?: number;
  contamination_level?: 'none' | 'low' | 'medium' | 'high';
  source?: string;
}

/**
 * Enzyme cocktail component
 */
export interface EnzymeCocktailComponent {
  enzyme_id: string;
  concentration_mg_g: number;
}

/**
 * Reaction conditions
 */
export interface ReactionConditions {
  temperature_c: number;
  ph: number;
  duration_hours: number;
  agitation_rpm?: number;
  volume_liters?: number;
}

/**
 * Process output results
 */
export interface ProcessOutput {
  degradation_percent: number;
  tpa_yield_kg?: number;
  tpa_purity_percent?: number;
  eg_yield_kg?: number;
  eg_purity_percent?: number;
  residue_kg?: number;
}

/**
 * Complete degradation process record
 */
export interface DegradationProcess {
  process_id: string;
  facility_id?: string;
  timestamp?: string;
  plastic_input: PlasticInput;
  enzyme_cocktail: EnzymeCocktailComponent[];
  conditions: ReactionConditions;
  output?: ProcessOutput;
  quality_grade?: QualityGrade;
  certification_status?: CertificationStatus;
}

// ============================================================================
// Prediction Types
// ============================================================================

/**
 * Prediction with uncertainty
 */
export interface PredictionValue {
  mean: number;
  std: number;
  ci_95?: [number, number];
}

/**
 * Degradation predictions
 */
export interface DegradationPredictions {
  degradation_percent: PredictionValue;
  tpa_yield_kg: PredictionValue;
  eg_yield_kg: PredictionValue;
  residue_kg?: PredictionValue;
}

/**
 * Time course data point
 */
export interface TimeCoursePoint {
  hour: number;
  degradation: number;
}

/**
 * Complete prediction response
 */
export interface PredictionResponse {
  predictions: DegradationPredictions;
  time_course: TimeCoursePoint[];
  model_version?: string;
}

// ============================================================================
// Optimization Types
// ============================================================================

/**
 * Optimization constraints
 */
export interface OptimizationConstraints {
  max_temperature_c?: number;
  max_time_hours?: number;
  target_efficiency?: number;
}

/**
 * Optimization goal
 */
export type OptimizationGoal = 'cost' | 'speed' | 'efficiency';

/**
 * Optimization request
 */
export interface OptimizationRequest {
  plastic: PlasticInput;
  constraints?: OptimizationConstraints;
  optimization_goal?: OptimizationGoal;
}

/**
 * Optimal conditions result
 */
export interface OptimalConditionsResult {
  temperature_c: number;
  ph: number;
  enzyme_loading_mg_g: number;
  substrate_loading_percent: number;
}

/**
 * Cocktail component with ratio
 */
export interface CocktailComponent {
  enzyme_id: string;
  ratio: number;
}

/**
 * Predicted outcomes
 */
export interface PredictedOutcomes {
  efficiency_percent: number;
  time_hours: number;
  tpa_yield_kg: number;
  cost_per_kg?: number;
}

/**
 * Optimization response
 */
export interface OptimizationResponse {
  optimal_conditions: OptimalConditionsResult;
  enzyme_cocktail: CocktailComponent[];
  predicted_outcomes: PredictedOutcomes;
}

// ============================================================================
// Quality Types
// ============================================================================

/**
 * Monomer analysis results
 */
export interface MonomerAnalysis {
  concentration_mM: number;
  purity_percent: number;
  contaminants?: Record<string, number>;
  color_hazen?: number;
  method?: string;
}

/**
 * Safety assessment results
 */
export interface SafetyAssessment {
  heavy_metals_compliant: boolean;
  microbial_count_cfu_g?: number;
  endotoxin_eu_ml?: number;
  residual_enzyme_activity?: string;
}

/**
 * Grade qualification results
 */
export interface GradeQualification {
  food_contact_eligible: boolean;
  bottle_grade_eligible: boolean;
  textile_grade_eligible: boolean;
  certification_ids?: string[];
}

/**
 * Complete quality metrics
 */
export interface QualityMetrics {
  batch_id: string;
  measurement_date: string;
  monomer_analysis: {
    tpa?: MonomerAnalysis;
    eg?: MonomerAnalysis;
  };
  safety_assessment?: SafetyAssessment;
  grade_qualification?: GradeQualification;
}

// ============================================================================
// API Types
// ============================================================================

/**
 * Metadata for records
 */
export interface Metadata {
  created_at?: string;
  updated_at?: string;
  wia_version?: string;
  created_by?: string;
}

/**
 * Paginated list response
 */
export interface PaginatedResponse<T> {
  total: number;
  page: number;
  per_page: number;
  data: T[];
}

/**
 * API error response
 */
export interface ApiError {
  code: string;
  message: string;
  details?: Record<string, unknown>;
  request_id?: string;
  documentation_url?: string;
}

/**
 * Client configuration
 */
export interface ClientConfig {
  apiKey: string;
  baseUrl?: string;
  environment?: 'production' | 'sandbox';
  timeout?: number;
}

// ============================================================================
// Plastic Identification Types
// ============================================================================

/**
 * Plastic identification request
 */
export interface PlasticIdentifyRequest {
  method: 'ftir_spectrum' | 'nir_spectrum' | 'image';
  data: {
    wavenumbers?: number[];
    absorbance?: number[];
    image_base64?: string;
  };
}

/**
 * Plastic identification response
 */
export interface PlasticIdentifyResponse {
  plastic_type: PlasticType;
  confidence: number;
  crystallinity_estimate?: number;
  contaminants_detected?: string[];
  recommended_pretreatment?: string;
}

// ============================================================================
// Enzyme Match Types
// ============================================================================

/**
 * Enzyme match request parameters
 */
export interface EnzymeMatchParams {
  temperature?: number;
  ph?: number;
  target_efficiency?: number;
}

/**
 * Enzyme match response
 */
export interface EnzymeMatchResponse {
  recommendations: EnzymeMatch[];
  cocktail_suggestion?: CocktailSuggestion;
}

// ============================================================================
// Enzyme Library Types
// ============================================================================

/**
 * Enzyme library query parameters
 */
export interface EnzymeLibraryParams {
  classification?: EnzymeClassification;
  min_temperature?: number;
  max_temperature?: number;
  engineered?: boolean;
  page?: number;
  per_page?: number;
}

/**
 * Enzyme library entry (summary)
 */
export interface EnzymeLibraryEntry {
  enzyme_id: string;
  name: string;
  classification: EnzymeClassification;
  source_organism: string;
  optimal_temperature: number;
  kcat_km?: number;
  reference?: string;
}

/**
 * Enzyme library response
 */
export interface EnzymeLibraryResponse {
  total: number;
  page: number;
  per_page: number;
  enzymes: EnzymeLibraryEntry[];
}
