/**
 * WIA-AI-008 AI Model Exchange Standard - TypeScript Types
 * 弘益人間 - Benefit All Humanity
 *
 * @module @wia/ai-model-exchange
 * @version 1.0.0
 * @license Apache-2.0
 */

/**
 * Task types supported by WIA-AI-008
 */
export enum TaskType {
  ImageClassification = 'image-classification',
  ObjectDetection = 'object-detection',
  SemanticSegmentation = 'semantic-segmentation',
  InstanceSegmentation = 'instance-segmentation',
  TextClassification = 'text-classification',
  NamedEntityRecognition = 'named-entity-recognition',
  QuestionAnswering = 'question-answering',
  TextGeneration = 'text-generation',
  Translation = 'translation',
  SpeechRecognition = 'speech-recognition',
  SpeechSynthesis = 'speech-synthesis',
  Recommendation = 'recommendation',
  TimeSeriesForecasting = 'time-series-forecasting',
  AnomalyDetection = 'anomaly-detection',
  Custom = 'custom',
}

/**
 * Supported model formats
 */
export enum ModelFormat {
  ONNX = 'onnx',
  PyTorchStateDict = 'pytorch-state-dict',
  TorchScript = 'torchscript',
  TensorFlowSavedModel = 'tensorflow-saved-model',
  HDF5 = 'hdf5',
  TensorFlowLite = 'tflite',
  CoreML = 'coreml',
}

/**
 * Model lifecycle stages
 */
export enum ModelStage {
  Development = 'development',
  Staging = 'staging',
  Production = 'production',
  Archived = 'archived',
}

/**
 * Model identity information
 */
export interface ModelIdentity {
  /** Model name (unique identifier) */
  name: string;
  /** Semantic version (MAJOR.MINOR.PATCH) */
  version: string;
  /** Unique ID (UUID v4) */
  unique_id: string;
  /** Creation timestamp (ISO 8601) */
  created_at: string;
  /** Last update timestamp (ISO 8601) */
  updated_at: string;
}

/**
 * Model details
 */
export interface ModelDetails {
  /** Model architecture (e.g., "ResNet-50", "BERT-base") */
  architecture: string;
  /** Task type */
  task_type: TaskType;
  /** Framework used for training */
  framework: string;
  /** Framework version */
  framework_version: string;
  /** Optional model description */
  description?: string;
}

/**
 * Training hyperparameters
 */
export interface Hyperparameters {
  learning_rate?: number;
  batch_size?: number;
  epochs?: number;
  optimizer?: string;
  [key: string]: any;
}

/**
 * Training information
 */
export interface TrainingInfo {
  /** Dataset name/identifier */
  dataset: string;
  /** Dataset version */
  dataset_version?: string;
  /** Training date (ISO 8601) */
  training_date: string;
  /** Hyperparameters used */
  hyperparameters: Hyperparameters;
  /** Preprocessing configuration */
  preprocessing?: Record<string, any>;
}

/**
 * Performance metrics
 */
export interface PerformanceMetrics {
  /** Primary metric (e.g., accuracy, F1, mAP) */
  primary_metric: {
    name: string;
    value: number;
  };
  /** Additional metrics */
  additional_metrics?: Record<string, number>;
  /** Evaluation dataset */
  evaluation_dataset: string;
}

/**
 * Input specification
 */
export interface InputSpec {
  /** Input name */
  name: string;
  /** Input shape (e.g., [1, 3, 224, 224]) */
  shape: (number | string)[];
  /** Data type (e.g., "float32", "int64") */
  dtype: string;
  /** Optional description */
  description?: string;
  /** Data format (e.g., "NCHW", "NHWC") */
  format?: string;
  /** Value range [min, max] */
  range?: [number, number];
}

/**
 * Output specification
 */
export interface OutputSpec {
  /** Output name */
  name: string;
  /** Output shape */
  shape: (number | string)[];
  /** Data type */
  dtype: string;
  /** Optional description */
  description?: string;
}

/**
 * Input/Output specifications
 */
export interface IOSpec {
  /** List of inputs */
  inputs: InputSpec[];
  /** List of outputs */
  outputs: OutputSpec[];
}

/**
 * Deployment configuration
 */
export interface DeploymentConfig {
  /** Target platforms */
  target_platforms: string[];
  /** Minimum memory requirement (MB) */
  min_memory_mb: number;
  /** Minimum compute requirement */
  min_compute: string;
  /** Expected inference latency (ms) */
  inference_latency_ms?: number;
}

/**
 * Governance information
 */
export interface GovernanceInfo {
  /** Model owner */
  owner: string;
  /** License (SPDX identifier) */
  license: string;
  /** Intended use description */
  intended_use: string;
  /** Known limitations */
  limitations: string;
  /** Ethical considerations */
  ethical_considerations?: string;
  /** Whether bias analysis was performed */
  bias_analysis: boolean;
  /** Whether privacy compliant */
  privacy_compliant: boolean;
}

/**
 * Provenance information
 */
export interface ProvenanceInfo {
  /** Base model (if fine-tuned) */
  base_model?: string;
  /** Derived from (parent model) */
  derived_from?: string;
  /** Git commit hash */
  git_commit?: string;
  /** Experiment ID (MLflow, W&B, etc.) */
  experiment_id?: string;
}

/**
 * WIA standard metadata
 */
export interface WIAStandard {
  /** Standard ID */
  standard_id: 'WIA-AI-008';
  /** Standard version */
  version: string;
  /** Philosophy */
  philosophy: '弘益人間 - Benefit All Humanity';
}

/**
 * Complete model card metadata
 */
export interface ModelCard {
  model_identity: ModelIdentity;
  model_details: ModelDetails;
  training_info: TrainingInfo;
  performance_metrics: PerformanceMetrics;
  input_output_spec: IOSpec;
  deployment: DeploymentConfig;
  governance: GovernanceInfo;
  provenance: ProvenanceInfo;
  wia_standard: WIAStandard;
}

/**
 * Model version information
 */
export interface ModelVersion {
  /** Version string */
  version: string;
  /** Creation timestamp */
  created_at: string;
  /** Model file size (bytes) */
  size_bytes: number;
  /** SHA256 checksum */
  checksum: string;
  /** Model metadata */
  metadata: ModelCard;
  /** Lifecycle stage */
  stage: ModelStage;
  /** Download URL */
  download_url?: string;
}

/**
 * Model summary for listing
 */
export interface ModelSummary {
  /** Model name */
  name: string;
  /** Latest version */
  latest_version: string;
  /** Task type */
  task_type: TaskType;
  /** Framework */
  framework: string;
  /** Creation date */
  created_at: string;
  /** Last update date */
  updated_at: string;
  /** Number of downloads */
  downloads?: number;
  /** Number of stars/favorites */
  stars?: number;
}

/**
 * Model registration request
 */
export interface ModelRegistrationRequest {
  /** Model name */
  name: string;
  /** Model description */
  description: string;
  /** Model owner */
  owner: string;
  /** License */
  license: string;
  /** Initial version */
  version: string;
  /** Model metadata */
  metadata: ModelCard;
}

/**
 * Conversion options
 */
export interface ConversionOptions {
  /** Source format */
  source_format: ModelFormat;
  /** Target format */
  target_format: ModelFormat;
  /** ONNX opset version */
  opset_version?: number;
  /** Optimization level */
  optimization_level?: 'none' | 'basic' | 'standard' | 'aggressive';
  /** Quantization */
  quantization?: 'none' | 'fp16' | 'int8' | 'int4';
}

/**
 * Conversion result
 */
export interface ConversionResult {
  /** Success status */
  success: boolean;
  /** Output path */
  output_path?: string;
  /** Conversion metadata */
  metadata: Record<string, any>;
  /** Warnings */
  warnings: string[];
  /** Errors */
  errors: string[];
}

/**
 * Validation result
 */
export interface ValidationResult {
  /** Whether validation passed */
  passed: boolean;
  /** Maximum numerical difference */
  max_difference: number;
  /** Mean numerical difference */
  mean_difference: number;
  /** Number of test cases */
  test_cases: number;
  /** List of failures */
  failures: string[];
}

/**
 * Registry client configuration
 */
export interface RegistryConfig {
  /** Registry URL */
  registry_url: string;
  /** API key for authentication */
  api_key?: string;
  /** OAuth token */
  token?: string;
  /** Request timeout (ms) */
  timeout?: number;
  /** Custom headers */
  headers?: Record<string, string>;
}

/**
 * Search parameters
 */
export interface SearchParams {
  /** Search query */
  query?: string;
  /** Filter by task type */
  task_type?: TaskType;
  /** Filter by framework */
  framework?: string;
  /** Filter by tags */
  tags?: string[];
  /** Result limit */
  limit?: number;
  /** Result offset */
  offset?: number;
  /** Sort by field */
  sort_by?: 'downloads' | 'stars' | 'created_at' | 'updated_at';
  /** Sort order */
  sort_order?: 'asc' | 'desc';
}

/**
 * Search results
 */
export interface SearchResults {
  /** List of matching models */
  results: ModelSummary[];
  /** Total number of results */
  total: number;
  /** Applied limit */
  limit: number;
  /** Applied offset */
  offset: number;
  /** Search query */
  query?: string;
}

/**
 * Health check response
 */
export interface HealthCheckResponse {
  /** Overall status */
  status: 'healthy' | 'degraded' | 'unhealthy';
  /** Model information */
  model: {
    name: string;
    version: string;
    loaded: boolean;
    memory_mb: number;
  };
  /** Server information */
  server: {
    uptime_seconds: number;
    requests_total: number;
    requests_per_second: number;
    avg_latency_ms: number;
  };
  /** Timestamp */
  timestamp: string;
}

/**
 * Inference request
 */
export interface InferenceRequest {
  /** Input data */
  inputs: Record<string, any>;
  /** Model name */
  model_name: string;
  /** Model version (or alias like "latest") */
  model_version?: string;
}

/**
 * Inference response
 */
export interface InferenceResponse {
  /** Output data */
  outputs: Record<string, any>;
  /** Model information */
  model: {
    name: string;
    version: string;
  };
  /** Performance metrics */
  performance: {
    latency_ms: number;
    preprocessing_ms?: number;
    inference_ms?: number;
    postprocessing_ms?: number;
  };
  /** Request ID */
  request_id: string;
}
