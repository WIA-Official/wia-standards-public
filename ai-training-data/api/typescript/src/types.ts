/**
 * WIA-AI-007 AI Training Data Standard - TypeScript Type Definitions
 * 弘益人間 (Benefit All Humanity)
 *
 * @version 1.0.0
 * @license MIT
 * @copyright 2025 SmileStory Inc. / WIA
 */

export type DataType = 'image' | 'text' | 'audio' | 'video' | 'tabular' | 'multimodal';
export type DataFormat = 'json' | 'parquet' | 'tfrecord' | 'hdf5' | 'arrow' | 'csv';
export type LicenseType = 'CC0' | 'CC-BY' | 'CC-BY-SA' | 'MIT' | 'Apache-2.0' | 'proprietary';
export type SourceType = 'original' | 'derived' | 'synthetic';

/**
 * Core dataset metadata following WIA-AI-007 specification
 */
export interface DatasetMetadata {
  'wia-ai-007': string;
  dataset: {
    id: string;
    name: string;
    version: string;
    created: string;
    updated: string;
    description: string;
    type: DataType;
    format: DataFormat;
    size: {
      samples: number;
      bytes: number;
      files: number;
    };
    splits: {
      train: number;
      validation: number;
      test: number;
    };
    philosophy: string;
  };
  schema: DatasetSchema;
  provenance: ProvenanceInfo;
  quality: QualityMetrics;
  license: LicenseInfo;
  ethics: EthicsInfo;
  statistics: StatisticsInfo;
}

/**
 * Dataset schema definition
 */
export interface DatasetSchema {
  features: Record<string, FeatureDefinition>;
  labels: Record<string, LabelDefinition>;
}

export interface FeatureDefinition {
  type: 'binary' | 'integer' | 'float' | 'string' | 'categorical';
  description: string;
  required: boolean;
  constraints?: {
    min?: number;
    max?: number;
    pattern?: string;
    enum?: string[];
  };
  format?: string;
  dimensions?: number[];
  dtype?: string;
}

export interface LabelDefinition {
  type: 'categorical' | 'continuous' | 'multi-label';
  classes?: string[];
  description: string;
  encoding?: 'one-hot' | 'label' | 'ordinal';
}

/**
 * Data provenance and lineage
 */
export interface ProvenanceInfo {
  source: {
    type: SourceType;
    origin: string;
    collection_date: string;
    collection_method: string;
    contributors: string[];
  };
  transformations: Transformation[];
  parent_datasets: string[];
  derivation?: string;
}

export interface Transformation {
  timestamp: string;
  operation: string;
  parameters: Record<string, any>;
  tool: string;
}

/**
 * Quality metrics
 */
export interface QualityMetrics {
  completeness: QualityScore;
  consistency: QualityScore;
  accuracy: QualityScore;
  uniqueness: QualityScore;
  timeliness?: QualityScore;
  overall_score: number;
}

export interface QualityScore {
  score: number;
  [key: string]: any;
}

/**
 * License and usage information
 */
export interface LicenseInfo {
  type: LicenseType;
  url: string;
  attribution: string;
  commercial_use: boolean;
  modifications_allowed: boolean;
  redistribution_allowed: boolean;
  restrictions?: string[];
}

/**
 * Ethics and compliance
 */
export interface EthicsInfo {
  irb_approval?: string;
  consent_obtained: boolean;
  anonymization: string;
  bias_assessment: string;
  intended_use: string[];
  prohibited_use: string[];
}

/**
 * Statistical information
 */
export interface StatisticsInfo {
  label_distribution: Record<string, number>;
  demographic_distribution?: Record<string, any>;
  feature_statistics?: Record<string, any>;
}

/**
 * Dataset version information
 */
export interface VersionInfo {
  version: string;
  date: string;
  changes: string[];
  breaking_changes: boolean;
}

/**
 * API Client configuration
 */
export interface ClientConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  retries?: number;
}

/**
 * Dataset creation options
 */
export interface CreateDatasetOptions {
  name: string;
  type: DataType;
  format: DataFormat;
  description: string;
  philosophy?: string;
}

/**
 * Dataset upload options
 */
export interface UploadOptions {
  files: string | string[];
  labels?: string;
  metadata?: string;
  batch_size?: number;
}

/**
 * Quality check options
 */
export interface QualityCheckOptions {
  checks: Array<'completeness' | 'consistency' | 'validity' | 'uniqueness'>;
  thresholds?: Record<string, number>;
}

/**
 * Quality check result
 */
export interface QualityCheckResult {
  overallScore: number;
  checks: Record<string, { score: number; passed: boolean }>;
  issues?: QualityIssue[];
}

export interface QualityIssue {
  severity: 'error' | 'warning' | 'info';
  description: string;
  field?: string;
  count?: number;
}

/**
 * Download options
 */
export interface DownloadOptions {
  version?: string;
  format?: DataFormat;
  outputPath: string;
  split?: 'train' | 'validation' | 'test' | 'all';
}

/**
 * Bias detection options
 */
export interface BiasDetectionOptions {
  protectedAttributes: string[];
  fairnessMetrics: Array<'demographic_parity' | 'equal_opportunity' | 'equalized_odds'>;
  threshold?: number;
}

/**
 * Bias detection result
 */
export interface BiasDetectionResult {
  biasDetected: boolean;
  metrics: Record<string, number>;
  affectedGroups: string[];
  recommendations: string[];
}

/**
 * Anonymization options
 */
export interface AnonymizationOptions {
  method: 'k-anonymity' | 'l-diversity' | 't-closeness' | 'differential-privacy';
  k?: number;
  l?: number;
  epsilon?: number;
  delta?: number;
  quasiIdentifiers?: string[];
}

/**
 * Export options
 */
export interface ExportOptions {
  framework: 'pytorch' | 'tensorflow' | 'jax' | 'scikit-learn';
  format: DataFormat;
  splits?: { train: number; val: number; test: number };
  outputPath: string;
}

/**
 * Pipeline stage
 */
export interface PipelineStage {
  name: string;
  type: 'ingestion' | 'validation' | 'transformation' | 'augmentation' | 'quality-check' | 'export';
  config: Record<string, any>;
}

/**
 * Pipeline definition
 */
export interface Pipeline {
  name: string;
  version: string;
  stages: PipelineStage[];
  philosophy: string;
}

/**
 * Pipeline execution result
 */
export interface PipelineResult {
  status: 'success' | 'failed' | 'partial';
  samplesProcessed: number;
  qualityScore: number;
  errors?: string[];
}

/**
 * Event types for webhooks
 */
export type WebhookEvent =
  | 'dataset.created'
  | 'dataset.updated'
  | 'dataset.deleted'
  | 'quality.below_threshold'
  | 'bias.detected'
  | 'upload.completed'
  | 'transform.completed';

/**
 * Webhook payload
 */
export interface WebhookPayload {
  event: WebhookEvent;
  timestamp: string;
  data: Record<string, any>;
}

/**
 * Error response
 */
export interface ErrorResponse {
  error: {
    code: string;
    message: string;
    details?: Record<string, any>;
  };
}
