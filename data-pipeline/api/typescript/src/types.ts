/**
 * WIA Data Pipeline Standard - TypeScript Type Definitions
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Pipeline Types
// ============================================================================

export interface WIADataPipeline {
  standard: 'WIA-DATA-PIPELINE';
  version: string;
  pipeline: PipelineMetadata;
  stages: PipelineStage[];
  connections: StageConnection[];
  triggers: PipelineTrigger[];
  configuration: PipelineConfiguration;
  monitoring?: MonitoringConfig;
  extensions?: Record<string, unknown>;
}

export interface PipelineMetadata {
  id: string;
  name: string;
  description?: string;
  owner: string;
  team?: string;
  createdAt: string;
  updatedAt?: string;
  status: PipelineStatus;
  tags?: string[];
}

export type PipelineStatus =
  | 'draft'
  | 'active'
  | 'paused'
  | 'deprecated'
  | 'archived';

// ============================================================================
// Stage Types
// ============================================================================

export interface PipelineStage {
  id: string;
  name: string;
  type: StageType;
  description?: string;
  configuration: StageConfiguration;
  inputs: StageIO[];
  outputs: StageIO[];
  dependencies?: string[];
  retry?: RetryPolicy;
  timeout?: number;
}

export type StageType =
  | 'source'
  | 'sink'
  | 'transform'
  | 'filter'
  | 'aggregate'
  | 'join'
  | 'split'
  | 'merge'
  | 'custom';

export interface StageConfiguration {
  executor: ExecutorConfig;
  parameters: Record<string, unknown>;
  resources?: ResourceRequirements;
  parallelism?: number;
}

export interface ExecutorConfig {
  type: ExecutorType;
  image?: string;
  version?: string;
  entrypoint?: string;
  args?: string[];
}

export type ExecutorType =
  | 'spark'
  | 'flink'
  | 'beam'
  | 'airflow'
  | 'dbt'
  | 'python'
  | 'sql'
  | 'container'
  | 'serverless';

export interface ResourceRequirements {
  cpu?: string;
  memory?: string;
  disk?: string;
  gpu?: number;
}

export interface StageIO {
  id: string;
  name: string;
  format: DataFormat;
  schema?: SchemaDefinition;
  location?: DataLocation;
}

export type DataFormat =
  | 'json'
  | 'parquet'
  | 'avro'
  | 'csv'
  | 'orc'
  | 'delta'
  | 'iceberg'
  | 'protobuf'
  | 'xml';

export interface SchemaDefinition {
  type: 'inline' | 'reference';
  inline?: SchemaField[];
  reference?: string;
}

export interface SchemaField {
  name: string;
  type: string;
  nullable?: boolean;
  description?: string;
  metadata?: Record<string, unknown>;
}

export interface DataLocation {
  type: LocationType;
  uri: string;
  credentials?: string;
  options?: Record<string, unknown>;
}

export type LocationType =
  | 's3'
  | 'gcs'
  | 'azure-blob'
  | 'hdfs'
  | 'kafka'
  | 'kinesis'
  | 'pubsub'
  | 'jdbc'
  | 'http'
  | 'file';

export interface RetryPolicy {
  maxAttempts: number;
  backoffMultiplier?: number;
  initialDelay?: number;
  maxDelay?: number;
  retryableErrors?: string[];
}

// ============================================================================
// Connection Types
// ============================================================================

export interface StageConnection {
  id: string;
  source: ConnectionEndpoint;
  target: ConnectionEndpoint;
  bufferConfig?: BufferConfiguration;
  dataValidation?: DataValidation;
}

export interface ConnectionEndpoint {
  stageId: string;
  outputId?: string;
  inputId?: string;
}

export interface BufferConfiguration {
  type: 'memory' | 'disk' | 'queue';
  size?: number;
  overflow?: 'drop' | 'backpressure' | 'spill';
}

export interface DataValidation {
  enabled: boolean;
  rules?: ValidationRule[];
  onFailure: 'stop' | 'skip' | 'quarantine';
}

export interface ValidationRule {
  name: string;
  type: 'schema' | 'business' | 'quality';
  expression: string;
  severity: 'error' | 'warning' | 'info';
}

// ============================================================================
// Trigger Types
// ============================================================================

export interface PipelineTrigger {
  id: string;
  name: string;
  type: TriggerType;
  enabled: boolean;
  configuration: TriggerConfiguration;
  conditions?: TriggerCondition[];
}

export type TriggerType =
  | 'schedule'
  | 'event'
  | 'file-arrival'
  | 'api'
  | 'manual'
  | 'dependency';

export type TriggerConfiguration =
  | ScheduleTriggerConfig
  | EventTriggerConfig
  | FileArrivalTriggerConfig
  | APITriggerConfig
  | DependencyTriggerConfig;

export interface ScheduleTriggerConfig {
  type: 'schedule';
  cron: string;
  timezone?: string;
  catchUp?: boolean;
}

export interface EventTriggerConfig {
  type: 'event';
  source: string;
  eventTypes: string[];
  filter?: string;
}

export interface FileArrivalTriggerConfig {
  type: 'file-arrival';
  location: DataLocation;
  pattern?: string;
  minFiles?: number;
  maxWait?: number;
}

export interface APITriggerConfig {
  type: 'api';
  endpoint?: string;
  authentication?: string;
}

export interface DependencyTriggerConfig {
  type: 'dependency';
  pipelineIds: string[];
  condition: 'all-success' | 'any-success' | 'all-complete';
}

export interface TriggerCondition {
  type: 'time' | 'data' | 'custom';
  expression: string;
}

// ============================================================================
// Configuration Types
// ============================================================================

export interface PipelineConfiguration {
  executionMode: ExecutionMode;
  errorHandling: ErrorHandlingConfig;
  logging: LoggingConfig;
  secrets?: SecretsConfig;
  variables?: Record<string, string>;
}

export type ExecutionMode =
  | 'batch'
  | 'streaming'
  | 'micro-batch'
  | 'hybrid';

export interface ErrorHandlingConfig {
  strategy: 'fail-fast' | 'continue' | 'retry';
  deadLetterQueue?: DataLocation;
  alerting?: AlertingConfig;
}

export interface AlertingConfig {
  channels: AlertChannel[];
  conditions: AlertCondition[];
}

export interface AlertChannel {
  type: 'email' | 'slack' | 'pagerduty' | 'webhook';
  target: string;
  severity?: string[];
}

export interface AlertCondition {
  type: 'failure' | 'latency' | 'data-quality' | 'custom';
  threshold?: number;
  expression?: string;
}

export interface LoggingConfig {
  level: 'debug' | 'info' | 'warn' | 'error';
  destination: DataLocation;
  format: 'json' | 'text';
  retention?: number;
}

export interface SecretsConfig {
  provider: 'vault' | 'aws-secrets' | 'gcp-secrets' | 'azure-keyvault' | 'env';
  path?: string;
}

// ============================================================================
// Monitoring Types
// ============================================================================

export interface MonitoringConfig {
  metrics: MetricsConfig;
  healthChecks?: HealthCheckConfig[];
  sla?: SLAConfig;
}

export interface MetricsConfig {
  enabled: boolean;
  destination?: DataLocation;
  interval?: number;
  customMetrics?: CustomMetric[];
}

export interface CustomMetric {
  name: string;
  type: 'counter' | 'gauge' | 'histogram';
  labels?: string[];
  expression: string;
}

export interface HealthCheckConfig {
  name: string;
  type: 'http' | 'tcp' | 'command';
  target: string;
  interval: number;
  timeout: number;
}

export interface SLAConfig {
  latency?: SLAThreshold;
  availability?: SLAThreshold;
  dataQuality?: SLAThreshold;
}

export interface SLAThreshold {
  target: number;
  warning: number;
  critical: number;
}

// ============================================================================
// Execution Types
// ============================================================================

export interface PipelineRun {
  id: string;
  pipelineId: string;
  status: RunStatus;
  triggeredBy: string;
  triggeredAt: string;
  startedAt?: string;
  completedAt?: string;
  stageRuns: StageRun[];
  metrics?: RunMetrics;
  error?: RunError;
}

export type RunStatus =
  | 'pending'
  | 'running'
  | 'success'
  | 'failed'
  | 'cancelled'
  | 'timeout';

export interface StageRun {
  stageId: string;
  status: RunStatus;
  startedAt?: string;
  completedAt?: string;
  metrics?: StageMetrics;
  logs?: string;
  error?: RunError;
}

export interface StageMetrics {
  recordsRead?: number;
  recordsWritten?: number;
  bytesRead?: number;
  bytesWritten?: number;
  duration?: number;
  customMetrics?: Record<string, number>;
}

export interface RunMetrics {
  totalDuration?: number;
  totalRecordsProcessed?: number;
  totalBytesProcessed?: number;
  resourceUsage?: ResourceUsage;
}

export interface ResourceUsage {
  cpuSeconds?: number;
  memoryGBSeconds?: number;
  networkBytes?: number;
}

export interface RunError {
  code: string;
  message: string;
  stageId?: string;
  stackTrace?: string;
  timestamp: string;
}

// ============================================================================
// API Types
// ============================================================================

export interface APIConfig {
  baseURL: string;
  apiKey?: string;
  timeout?: number;
}

export interface PipelineResponse {
  id: string;
  name: string;
  status: PipelineStatus;
  stageCount: number;
  lastRunAt?: string;
  lastRunStatus?: RunStatus;
  createdAt: string;
  updatedAt?: string;
  links: {
    self: string;
    runs?: string;
    stages?: string;
  };
}

export interface RunRequest {
  pipelineId: string;
  variables?: Record<string, string>;
  stageOverrides?: StageOverride[];
  dryRun?: boolean;
}

export interface StageOverride {
  stageId: string;
  parameters?: Record<string, unknown>;
  skip?: boolean;
}

// ============================================================================
// Utility Types
// ============================================================================

export interface APIError {
  code: string;
  message: string;
  details?: Record<string, unknown>;
  timestamp: string;
  requestId?: string;
}

export interface ValidationResult {
  valid: boolean;
  errors?: ValidationError[];
}

export interface ValidationError {
  path: string;
  message: string;
  value?: unknown;
}

export interface PaginatedResponse<T> {
  data: T[];
  pagination: {
    total: number;
    limit: number;
    offset: number;
    hasMore: boolean;
  };
  links: {
    first?: string;
    prev?: string;
    self: string;
    next?: string;
    last?: string;
  };
}
