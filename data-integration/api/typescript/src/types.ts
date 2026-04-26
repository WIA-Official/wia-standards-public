/**
 * WIA Data Integration Standard - TypeScript Type Definitions
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Data Integration Types
// ============================================================================

export interface WIADataIntegrationProject {
  standard: 'WIA-DATA-INTEGRATION';
  version: string;
  metadata: ProjectMetadata;
  sources: SourceConfiguration;
  targets: TargetConfiguration;
  pipelines: PipelineConfiguration;
  transformations: TransformationConfig;
  orchestration: OrchestrationConfig;
  quality: QualityConfig;
  monitoring: MonitoringConfig;
  extensions?: Record<string, unknown>;
}

export interface ProjectMetadata {
  id: string;
  name: string;
  description?: string;
  organization: Organization;
  environment: 'development' | 'staging' | 'production';
  createdAt: string;
  updatedAt?: string;
  status: ProjectStatus;
}

export interface Organization {
  name: string;
  team: string;
  contact: ContactInfo;
}

export interface ContactInfo {
  name: string;
  email: string;
  role: string;
}

export type ProjectStatus = 'active' | 'paused' | 'maintenance' | 'deprecated';

// ============================================================================
// Source Configuration Types
// ============================================================================

export interface SourceConfiguration {
  connectors: SourceConnector[];
  discovery: SourceDiscovery;
  schemas: SchemaManagement;
}

export interface SourceConnector {
  id: string;
  name: string;
  type: ConnectorType;
  config: ConnectionConfig;
  credentials: CredentialRef;
  capabilities: SourceCapabilities;
  status: 'active' | 'inactive' | 'error';
}

export type ConnectorType = 'database' | 'file' | 'api' | 'stream' | 'queue' | 'saas' | 'cloud-storage' | 'custom';

export interface ConnectionConfig {
  host?: string;
  port?: number;
  database?: string;
  schema?: string;
  ssl?: boolean;
  poolSize?: number;
  timeout?: number;
  options?: Record<string, unknown>;
}

export interface CredentialRef {
  type: 'secret-manager' | 'vault' | 'env' | 'config';
  reference: string;
  rotation?: RotationConfig;
}

export interface RotationConfig {
  enabled: boolean;
  frequency: string;
  notification: string[];
}

export interface SourceCapabilities {
  fullLoad: boolean;
  incrementalLoad: boolean;
  cdc: boolean;
  parallelRead: boolean;
  pushdown: boolean;
  watermark: WatermarkConfig;
}

export interface WatermarkConfig {
  supported: boolean;
  columns?: string[];
  type?: 'timestamp' | 'version' | 'sequence';
}

export interface SourceDiscovery {
  enabled: boolean;
  frequency: string;
  autoSchema: boolean;
  filters: DiscoveryFilter[];
}

export interface DiscoveryFilter {
  type: 'include' | 'exclude';
  pattern: string;
  scope: 'schema' | 'table' | 'column';
}

export interface SchemaManagement {
  evolution: SchemaEvolution;
  validation: SchemaValidation;
  versioning: boolean;
}

export interface SchemaEvolution {
  strategy: 'strict' | 'flexible' | 'auto';
  allowedChanges: ('add-column' | 'remove-column' | 'type-change' | 'rename')[];
  notification: boolean;
}

export interface SchemaValidation {
  enabled: boolean;
  rules: ValidationRule[];
  action: 'warn' | 'fail' | 'skip';
}

export interface ValidationRule {
  id: string;
  type: 'type' | 'nullable' | 'format' | 'custom';
  expression: string;
  severity: 'error' | 'warning';
}

// ============================================================================
// Target Configuration Types
// ============================================================================

export interface TargetConfiguration {
  destinations: TargetDestination[];
  writeConfig: WriteConfiguration;
  partitioning: PartitionStrategy[];
}

export interface TargetDestination {
  id: string;
  name: string;
  type: ConnectorType;
  config: ConnectionConfig;
  credentials: CredentialRef;
  capabilities: TargetCapabilities;
}

export interface TargetCapabilities {
  upsert: boolean;
  delete: boolean;
  truncate: boolean;
  createTable: boolean;
  parallelWrite: boolean;
  transactions: boolean;
}

export interface WriteConfiguration {
  mode: 'append' | 'overwrite' | 'upsert' | 'merge';
  batchSize: number;
  parallelism: number;
  errorHandling: ErrorHandling;
  commit: CommitStrategy;
}

export interface ErrorHandling {
  strategy: 'fail' | 'skip' | 'retry' | 'quarantine';
  maxErrors?: number;
  retryAttempts?: number;
  retryDelay?: string;
  deadLetterQueue?: string;
}

export interface CommitStrategy {
  type: 'per-batch' | 'per-partition' | 'end-of-job';
  checkpointing: boolean;
  checkpointInterval?: string;
}

export interface PartitionStrategy {
  id: string;
  name: string;
  columns: string[];
  type: 'date' | 'hash' | 'list' | 'range';
  granularity?: 'hour' | 'day' | 'month' | 'year';
  pruning: boolean;
}

// ============================================================================
// Pipeline Configuration Types
// ============================================================================

export interface PipelineConfiguration {
  pipelines: Pipeline[];
  templates: PipelineTemplate[];
  dependencies: DependencyGraph;
}

export interface Pipeline {
  id: string;
  name: string;
  description?: string;
  source: string;
  target: string;
  stages: PipelineStage[];
  schedule?: ScheduleConfig;
  config: PipelineConfig;
  status: 'active' | 'paused' | 'failed' | 'draft';
}

export interface PipelineStage {
  id: string;
  name: string;
  type: StageType;
  order: number;
  config: Record<string, unknown>;
  inputs: string[];
  outputs: string[];
  errorHandler?: string;
}

export type StageType = 'extract' | 'transform' | 'load' | 'validate' | 'filter' | 'aggregate' | 'join' | 'lookup' | 'split' | 'merge' | 'custom';

export interface ScheduleConfig {
  type: 'cron' | 'interval' | 'event' | 'manual';
  expression?: string;
  timezone?: string;
  startDate?: string;
  endDate?: string;
  dependencies?: string[];
}

export interface PipelineConfig {
  parallelism: number;
  timeout: string;
  retryPolicy: RetryPolicy;
  resources: ResourceConfig;
  isolation: IsolationLevel;
}

export interface RetryPolicy {
  maxAttempts: number;
  backoff: 'linear' | 'exponential' | 'fixed';
  initialDelay: string;
  maxDelay: string;
}

export interface ResourceConfig {
  cpu: string;
  memory: string;
  executors?: number;
  spot?: boolean;
}

export type IsolationLevel = 'read-committed' | 'repeatable-read' | 'serializable';

export interface PipelineTemplate {
  id: string;
  name: string;
  description: string;
  stages: PipelineStage[];
  parameters: TemplateParameter[];
}

export interface TemplateParameter {
  name: string;
  type: 'string' | 'number' | 'boolean' | 'list';
  required: boolean;
  default?: unknown;
  description?: string;
}

export interface DependencyGraph {
  nodes: DependencyNode[];
  edges: DependencyEdge[];
}

export interface DependencyNode {
  id: string;
  type: 'pipeline' | 'dataset' | 'external';
  name: string;
}

export interface DependencyEdge {
  source: string;
  target: string;
  type: 'data' | 'schedule' | 'trigger';
}

// ============================================================================
// Transformation Types
// ============================================================================

export interface TransformationConfig {
  engine: TransformationEngine;
  library: TransformationLibrary;
  mapping: MappingConfig;
}

export interface TransformationEngine {
  type: 'spark' | 'flink' | 'dbt' | 'sql' | 'python' | 'custom';
  version?: string;
  config: Record<string, unknown>;
}

export interface TransformationLibrary {
  builtin: BuiltinTransform[];
  custom: CustomTransform[];
  udfs: UserDefinedFunction[];
}

export interface BuiltinTransform {
  id: string;
  name: string;
  category: 'string' | 'numeric' | 'date' | 'conversion' | 'aggregation' | 'window';
  parameters: TransformParameter[];
}

export interface TransformParameter {
  name: string;
  type: string;
  required: boolean;
  default?: unknown;
}

export interface CustomTransform {
  id: string;
  name: string;
  language: 'sql' | 'python' | 'scala';
  code: string;
  inputSchema: SchemaDefinition;
  outputSchema: SchemaDefinition;
}

export interface SchemaDefinition {
  columns: ColumnDef[];
}

export interface ColumnDef {
  name: string;
  type: string;
  nullable: boolean;
}

export interface UserDefinedFunction {
  id: string;
  name: string;
  language: 'python' | 'java' | 'scala';
  returnType: string;
  parameters: { name: string; type: string }[];
  implementation: string;
}

export interface MappingConfig {
  autoMapping: boolean;
  matchingStrategy: 'exact' | 'case-insensitive' | 'fuzzy';
  defaultTransforms: DefaultTransform[];
}

export interface DefaultTransform {
  sourceType: string;
  targetType: string;
  transform: string;
}

export interface FieldMapping {
  source: string;
  target: string;
  transform?: string;
  default?: unknown;
  nullable: boolean;
}

// ============================================================================
// Orchestration Types
// ============================================================================

export interface OrchestrationConfig {
  platform: OrchestrationPlatform;
  workflows: Workflow[];
  triggers: TriggerConfig[];
  notifications: NotificationConfig;
}

export interface OrchestrationPlatform {
  type: 'airflow' | 'prefect' | 'dagster' | 'argo' | 'step-functions' | 'custom';
  endpoint?: string;
  authentication?: string;
}

export interface Workflow {
  id: string;
  name: string;
  description?: string;
  tasks: WorkflowTask[];
  schedule?: ScheduleConfig;
  sla?: SLAConfig;
  status: 'active' | 'paused' | 'archived';
}

export interface WorkflowTask {
  id: string;
  name: string;
  type: 'pipeline' | 'sensor' | 'operator' | 'branch' | 'join';
  config: Record<string, unknown>;
  dependencies: string[];
  retries?: number;
  timeout?: string;
}

export interface SLAConfig {
  duration: string;
  alertChannels: string[];
  escalation: string;
}

export interface TriggerConfig {
  id: string;
  type: 'schedule' | 'file' | 'api' | 'event' | 'sensor';
  config: Record<string, unknown>;
  targets: string[];
  enabled: boolean;
}

export interface NotificationConfig {
  channels: NotificationChannel[];
  rules: NotificationRule[];
}

export interface NotificationChannel {
  id: string;
  type: 'email' | 'slack' | 'pagerduty' | 'webhook';
  config: Record<string, unknown>;
}

export interface NotificationRule {
  event: 'success' | 'failure' | 'sla-breach' | 'start' | 'retry';
  channels: string[];
  template?: string;
}

// ============================================================================
// Quality & Monitoring Types
// ============================================================================

export interface QualityConfig {
  checks: QualityCheck[];
  profiling: ProfilingConfig;
  anomalyDetection: AnomalyConfig;
}

export interface QualityCheck {
  id: string;
  name: string;
  type: 'completeness' | 'uniqueness' | 'validity' | 'consistency' | 'freshness' | 'custom';
  expression: string;
  threshold: number;
  severity: 'warning' | 'error' | 'critical';
  action: 'log' | 'alert' | 'fail';
}

export interface ProfilingConfig {
  enabled: boolean;
  frequency: 'per-run' | 'daily' | 'weekly';
  metrics: ('cardinality' | 'nulls' | 'distribution' | 'patterns')[];
  sampling: number;
}

export interface AnomalyConfig {
  enabled: boolean;
  metrics: string[];
  sensitivity: 'low' | 'medium' | 'high';
  baseline: string;
}

export interface MonitoringConfig {
  metrics: MetricsConfig;
  logging: LoggingConfig;
  alerting: AlertingConfig;
  lineage: LineageTracking;
}

export interface MetricsConfig {
  enabled: boolean;
  provider: string;
  interval: string;
  dimensions: string[];
  customMetrics: CustomMetric[];
}

export interface CustomMetric {
  name: string;
  type: 'counter' | 'gauge' | 'histogram';
  labels: string[];
}

export interface LoggingConfig {
  level: 'debug' | 'info' | 'warn' | 'error';
  format: 'json' | 'text';
  destination: string;
  retention: string;
}

export interface AlertingConfig {
  enabled: boolean;
  rules: AlertRule[];
  channels: string[];
}

export interface AlertRule {
  id: string;
  name: string;
  condition: string;
  severity: 'low' | 'medium' | 'high' | 'critical';
  cooldown: string;
}

export interface LineageTracking {
  enabled: boolean;
  granularity: 'pipeline' | 'table' | 'column';
  storage: string;
}

// ============================================================================
// Execution Types
// ============================================================================

export interface PipelineRun {
  id: string;
  pipelineId: string;
  status: 'pending' | 'running' | 'completed' | 'failed' | 'cancelled';
  startedAt: string;
  completedAt?: string;
  triggeredBy: string;
  stages: StageExecution[];
  metrics: RunMetrics;
  errors?: ExecutionError[];
}

export interface StageExecution {
  stageId: string;
  status: 'pending' | 'running' | 'completed' | 'failed' | 'skipped';
  startedAt?: string;
  completedAt?: string;
  recordsRead?: number;
  recordsWritten?: number;
  error?: string;
}

export interface RunMetrics {
  recordsProcessed: number;
  bytesProcessed: number;
  duration: number;
  cpuTime: number;
  peakMemory: number;
}

export interface ExecutionError {
  stage: string;
  message: string;
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

export interface ProjectResponse {
  id: string;
  name: string;
  status: ProjectStatus;
  createdAt: string;
  updatedAt?: string;
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
}
