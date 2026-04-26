/**
 * WIA-AI-009 Explainable AI Standard - TypeScript Type Definitions
 * 弘益人間 (Benefit All Humanity)
 */

export const STANDARD_VERSION = "1.0";
export const STANDARD_ID = "WIA-AI-009";

// ============================================================================
// Core Explanation Types
// ============================================================================

export type ExplanationType = "local" | "global" | "counterfactual";

export type ExplanationMethod =
  | "shap"
  | "lime"
  | "attention"
  | "integrated_gradients"
  | "grad_cam"
  | "counterfactual"
  | "custom";

export type TargetAudience =
  | "technical"
  | "business"
  | "enduser"
  | "regulatory";

export type Granularity = "low" | "medium" | "high";

// ============================================================================
// Model Information
// ============================================================================

export interface ModelInfo {
  model_id: string;
  model_type: string;
  model_version: string;
  framework?: "tensorflow" | "pytorch" | "sklearn" | "xgboost" | "custom";
  training_date?: string;
}

// ============================================================================
// Prediction
// ============================================================================

export interface Prediction {
  value: number | string;
  class?: string;
  probability?: number;
  confidence?: "low" | "medium" | "high";
}

// ============================================================================
// Feature Attribution
// ============================================================================

export interface FeatureAttribution {
  feature_name: string;
  attribution_value: number;
  feature_value: any;
  confidence_interval?: [number, number];
  rank?: number;
  direction: "positive" | "negative" | "neutral";
}

export interface AttributionMap {
  [featureName: string]: number;
}

// ============================================================================
// Trust Metrics
// ============================================================================

export interface TrustMetrics {
  fidelity: number;
  consistency?: number;
  stability?: number;
  completeness?: number;
  comprehensibility_score?: number;
}

export interface FairnessMetrics {
  fidelity_parity?: number;
  stability_parity?: number;
  protected_attribute_weight?: number;
}

export interface QualityMetrics extends TrustMetrics {
  fairness_metrics?: FairnessMetrics;
  validation?: {
    ablation_correlation?: number;
    perturbation_alignment?: number;
    cross_method_agreement?: number;
  };
}

// ============================================================================
// Explanation Data Structures
// ============================================================================

export interface LocalExplanation {
  base_value: number;
  feature_attributions: AttributionMap;
  feature_values: Record<string, any>;
  top_features?: FeatureAttribution[];
}

export interface GlobalExplanation {
  feature_importance: AttributionMap;
  total_instances_analyzed: number;
  aggregation_method: string;
  feature_interactions?: Record<string, number>;
}

export interface CounterfactualExplanation {
  original_prediction: string | number;
  counterfactual_prediction: string | number;
  required_changes: Record<string, {
    current: any;
    required: any;
    change: any;
  }>;
  minimal_change_set: string[];
  actionability_score: number;
}

export type ExplanationData =
  | LocalExplanation
  | GlobalExplanation
  | CounterfactualExplanation;

// ============================================================================
// Explanation Request
// ============================================================================

export interface ExplanationConfig {
  methods?: ExplanationMethod[];
  granularity?: Granularity;
  num_features?: number;
  include_counterfactuals?: boolean;
  target_audience?: TargetAudience;
}

export interface QualityRequirements {
  min_fidelity?: number;
  min_stability?: number;
  max_computation_time_ms?: number;
}

export interface OutputFormat {
  format?: "json" | "html" | "pdf";
  include_visualizations?: boolean;
  language?: "en" | "ko" | "es" | "fr" | "de";
}

export interface ExplanationRequest {
  protocol: "XAI-REQUEST-v1";
  request_id: string;
  timestamp: string;
  model_identifier: ModelInfo;
  input_data: Record<string, any>;
  explanation_config: ExplanationConfig;
  quality_requirements?: QualityRequirements;
  output_format?: OutputFormat;
}

// ============================================================================
// Explanation Response
// ============================================================================

export interface ExplanationMetadata {
  computation_time_ms: number;
  model_version_used: string;
  explainer_versions?: Record<string, string>;
  cache_hit?: boolean;
}

export interface Explanation {
  standard: string;
  version: string;
  explanation_id: string;
  timestamp: string;
  explanation_type: ExplanationType;
  method: ExplanationMethod;
  model_info: ModelInfo;
  input: Record<string, any>;
  prediction: Prediction;
  explanation: ExplanationData;
  metadata: ExplanationMetadata;
  quality_metrics?: QualityMetrics;
  warnings?: string[];
}

export type ResponseStatus = "success" | "partial" | "failed";

export interface ExplanationResponse {
  protocol: "XAI-RESPONSE-v1";
  request_id: string;
  response_id: string;
  timestamp: string;
  status: ResponseStatus;
  prediction: Prediction;
  explanations: Explanation[];
  metadata: ExplanationMetadata;
  warnings?: string[];
  errors?: ExplanationError[];
}

// ============================================================================
// Error Handling
// ============================================================================

export type ErrorCode =
  | "INSUFFICIENT_SAMPLES"
  | "TIMEOUT"
  | "MODEL_INCOMPATIBLE"
  | "CONVERGENCE_FAILED"
  | "MEMORY_EXCEEDED"
  | "INVALID_INPUT"
  | "QUALITY_THRESHOLD_NOT_MET";

export interface ExplanationError {
  code: ErrorCode;
  message: string;
  details: Record<string, any>;
  fallback_available: boolean;
  retry_recommended: boolean;
}

// ============================================================================
// Explainer Configuration
// ============================================================================

export interface SHAPConfig {
  variant?: "TreeSHAP" | "KernelSHAP" | "LinearSHAP" | "DeepSHAP";
  background_data?: any[];
  n_samples?: number;
  check_additivity?: boolean;
}

export interface LIMEConfig {
  kernel_width?: number;
  n_samples?: number;
  n_features?: number;
  categorical_features?: string[];
}

export interface AttentionConfig {
  layer?: number;
  head?: number | "all";
  aggregation?: "mean" | "max" | "sum";
}

export interface IntegratedGradientsConfig {
  baseline?: "zeros" | "mean" | "black" | any[];
  n_steps?: number;
  method?: "riemann_trapezoidal" | "riemann_left" | "riemann_right";
}

export type ExplainerConfig =
  | SHAPConfig
  | LIMEConfig
  | AttentionConfig
  | IntegratedGradientsConfig;

// ============================================================================
// Validation Results
// ============================================================================

export interface AblationResult {
  passed: boolean;
  correlation: number;
  details: Array<{
    feature: string;
    expected_change: number;
    actual_change: number;
    ratio: number;
  }>;
}

export interface ValidationReport {
  fidelity: { value: number; passed: boolean };
  consistency?: { value: number; passed: boolean };
  stability?: { value: number; passed: boolean };
  completeness?: { value: number; passed: boolean };
  ablation_test?: AblationResult;
  cross_method_agreement?: { value: number; passed: boolean };
  fairness_parity?: { value: number; passed: boolean };
  overall_passed: boolean;
  recommendations: string[];
}

// ============================================================================
// Batch Operations
// ============================================================================

export interface BatchExplanationRequest {
  protocol: "XAI-BATCH-REQUEST-v1";
  batch_id: string;
  instances: Record<string, any>[];
  explanation_config: ExplanationConfig;
  async?: boolean;
}

export interface BatchExplanationResponse {
  protocol: "XAI-BATCH-RESPONSE-v1";
  batch_id: string;
  status: "processing" | "completed" | "failed";
  progress?: { completed: number; total: number };
  explanations?: Explanation[];
  estimated_completion?: string;
  results_url?: string;
}

// ============================================================================
// Visualization
// ============================================================================

export type VisualizationType =
  | "force_plot"
  | "bar_chart"
  | "heatmap"
  | "scatter"
  | "attention_map"
  | "dependence_plot";

export interface VisualizationConfig {
  type: VisualizationType;
  color_scheme?: "diverging" | "sequential" | "categorical";
  highlight_features?: string[];
  width?: number;
  height?: number;
}

export interface VisualizationData {
  recommended_type: VisualizationType;
  config: VisualizationConfig;
  data: any;
}

// ============================================================================
// Explainer Interface
// ============================================================================

export interface IExplainer {
  explain(
    input: Record<string, any>,
    config?: ExplainerConfig
  ): Promise<Explanation>;

  explainBatch(
    inputs: Record<string, any>[],
    config?: ExplainerConfig
  ): Promise<Explanation[]>;

  validate(
    testSet: Record<string, any>[],
    config?: ExplainerConfig
  ): Promise<ValidationReport>;

  getConfig(): ExplainerConfig;
  setConfig(config: ExplainerConfig): void;
}

// ============================================================================
// Model Interface
// ============================================================================

export interface IModel {
  predict(input: Record<string, any>): Promise<number | string>;
  predictBatch(inputs: Record<string, any>[]): Promise<(number | string)[]>;
  getInfo(): ModelInfo;
}
