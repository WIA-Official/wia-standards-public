/**
 * WIA Data Lake Standard - TypeScript Type Definitions
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Data Lake Types
// ============================================================================

export interface WIADataLakeProject {
  standard: 'WIA-DATA-LAKE';
  version: string;
  metadata: ProjectMetadata;
  architecture: LakeArchitecture;
  storage: StorageConfiguration;
  ingestion: IngestionConfig;
  processing: ProcessingConfig;
  catalog: CatalogConfig;
  governance: GovernanceConfig;
  access: AccessConfig;
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
  cloudProvider: 'aws' | 'azure' | 'gcp' | 'hybrid' | 'on-premises';
  region: string;
  contact: ContactInfo;
}

export interface ContactInfo {
  name: string;
  email: string;
  role: string;
}

export type ProjectStatus = 'active' | 'provisioning' | 'maintenance' | 'archived';

// ============================================================================
// Lake Architecture Types
// ============================================================================

export interface LakeArchitecture {
  pattern: ArchitecturePattern;
  zones: DataZone[];
  layers: DataLayer[];
  medallion?: MedallionArchitecture;
}

export interface ArchitecturePattern {
  type: 'traditional' | 'lakehouse' | 'data-mesh' | 'hybrid';
  description: string;
  principles: string[];
}

export interface DataZone {
  id: string;
  name: string;
  purpose: 'landing' | 'raw' | 'curated' | 'consumption' | 'sandbox';
  storage: string;
  retention: string;
  access: string[];
  encryption: boolean;
}

export interface DataLayer {
  id: string;
  name: string;
  type: 'bronze' | 'silver' | 'gold' | 'platinum';
  zone: string;
  format: DataFormat[];
  transformations: string[];
  sla: LayerSLA;
}

export interface DataFormat {
  format: 'parquet' | 'delta' | 'iceberg' | 'hudi' | 'orc' | 'avro' | 'json' | 'csv';
  compression?: 'snappy' | 'gzip' | 'zstd' | 'lz4';
  partitioning?: PartitionScheme;
}

export interface PartitionScheme {
  columns: string[];
  strategy: 'date' | 'hash' | 'range' | 'list';
  granularity?: 'hour' | 'day' | 'month' | 'year';
}

export interface LayerSLA {
  freshness: string;
  availability: number;
  quality: number;
}

export interface MedallionArchitecture {
  enabled: boolean;
  bronze: MedallionLayer;
  silver: MedallionLayer;
  gold: MedallionLayer;
}

export interface MedallionLayer {
  zone: string;
  format: string;
  retention: string;
  transformations: string[];
}

// ============================================================================
// Storage Configuration Types
// ============================================================================

export interface StorageConfiguration {
  primary: StorageSystem;
  secondary?: StorageSystem;
  tiering: StorageTiering;
  lifecycle: LifecyclePolicy[];
}

export interface StorageSystem {
  type: 's3' | 'adls' | 'gcs' | 'hdfs' | 'minio';
  bucket: string;
  region?: string;
  endpoint?: string;
  credentials: CredentialRef;
  encryption: StorageEncryption;
  versioning: boolean;
}

export interface CredentialRef {
  type: 'iam' | 'service-account' | 'access-key' | 'managed-identity';
  reference: string;
}

export interface StorageEncryption {
  enabled: boolean;
  type: 'sse-s3' | 'sse-kms' | 'sse-c' | 'client-side';
  keyId?: string;
}

export interface StorageTiering {
  enabled: boolean;
  tiers: StorageTier[];
  automation: boolean;
}

export interface StorageTier {
  name: string;
  class: 'hot' | 'warm' | 'cold' | 'archive';
  transitionAfter: string;
  cost: number;
}

export interface LifecyclePolicy {
  id: string;
  name: string;
  prefix?: string;
  tags?: Record<string, string>;
  transitions: LifecycleTransition[];
  expiration?: string;
}

export interface LifecycleTransition {
  daysAfterCreation: number;
  storageClass: string;
}

// ============================================================================
// Ingestion Configuration Types
// ============================================================================

export interface IngestionConfig {
  sources: IngestionSource[];
  patterns: IngestionPattern[];
  streaming: StreamingConfig;
  batch: BatchConfig;
}

export interface IngestionSource {
  id: string;
  name: string;
  type: SourceType;
  connection: ConnectionConfig;
  schema?: SchemaConfig;
  schedule?: string;
  status: 'active' | 'paused' | 'error';
}

export type SourceType = 'database' | 'api' | 'file' | 'stream' | 'queue' | 'iot' | 'saas';

export interface ConnectionConfig {
  endpoint: string;
  credentials: CredentialRef;
  options?: Record<string, unknown>;
}

export interface SchemaConfig {
  format: 'auto' | 'explicit' | 'schema-registry';
  registry?: string;
  evolution: 'strict' | 'backward' | 'forward' | 'full';
}

export interface IngestionPattern {
  id: string;
  name: string;
  type: 'full' | 'incremental' | 'cdc' | 'streaming';
  config: Record<string, unknown>;
}

export interface StreamingConfig {
  enabled: boolean;
  platform: 'kafka' | 'kinesis' | 'pubsub' | 'eventhub';
  topics: StreamTopic[];
  processing: StreamProcessing;
}

export interface StreamTopic {
  name: string;
  partitions: number;
  retention: string;
  schema?: string;
}

export interface StreamProcessing {
  engine: 'spark-streaming' | 'flink' | 'kafka-streams' | 'dataflow';
  checkpointing: boolean;
  watermark: string;
  lateDataHandling: 'drop' | 'allow' | 'sideOutput';
}

export interface BatchConfig {
  scheduler: 'airflow' | 'prefect' | 'dagster' | 'cron';
  defaultSchedule: string;
  parallelism: number;
  retryPolicy: RetryPolicy;
}

export interface RetryPolicy {
  maxAttempts: number;
  backoff: 'linear' | 'exponential';
  initialDelay: string;
}

// ============================================================================
// Processing Configuration Types
// ============================================================================

export interface ProcessingConfig {
  engines: ProcessingEngine[];
  pipelines: DataPipeline[];
  transformations: TransformationLibrary;
  compute: ComputeConfig;
}

export interface ProcessingEngine {
  id: string;
  type: 'spark' | 'dbt' | 'flink' | 'presto' | 'trino' | 'duckdb';
  version: string;
  cluster?: ClusterConfig;
  serverless?: boolean;
}

export interface ClusterConfig {
  name: string;
  size: 'small' | 'medium' | 'large' | 'xlarge';
  minNodes: number;
  maxNodes: number;
  autoScaling: boolean;
  spotInstances: boolean;
}

export interface DataPipeline {
  id: string;
  name: string;
  description?: string;
  source: PipelineEndpoint;
  target: PipelineEndpoint;
  stages: PipelineStage[];
  schedule?: string;
  dependencies?: string[];
  status: 'active' | 'paused' | 'failed';
}

export interface PipelineEndpoint {
  zone: string;
  layer: string;
  table: string;
}

export interface PipelineStage {
  id: string;
  name: string;
  type: 'extract' | 'transform' | 'validate' | 'load' | 'quality';
  engine: string;
  config: Record<string, unknown>;
  order: number;
}

export interface TransformationLibrary {
  builtin: BuiltinTransform[];
  custom: CustomTransform[];
  macros: TransformMacro[];
}

export interface BuiltinTransform {
  name: string;
  category: 'cleaning' | 'enrichment' | 'aggregation' | 'modeling';
  parameters: TransformParam[];
}

export interface TransformParam {
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
  tests?: string[];
}

export interface TransformMacro {
  name: string;
  definition: string;
  parameters: string[];
}

export interface ComputeConfig {
  default: string;
  autoscaling: AutoscalingConfig;
  scheduling: SchedulingConfig;
}

export interface AutoscalingConfig {
  enabled: boolean;
  minCapacity: number;
  maxCapacity: number;
  targetUtilization: number;
  cooldown: string;
}

export interface SchedulingConfig {
  fairScheduler: boolean;
  queues: ResourceQueue[];
  priorities: { pipeline: string; priority: number }[];
}

export interface ResourceQueue {
  name: string;
  capacity: number;
  maxCapacity: number;
}

// ============================================================================
// Catalog & Governance Types
// ============================================================================

export interface CatalogConfig {
  platform: CatalogPlatform;
  discovery: DiscoveryConfig;
  metadata: MetadataConfig;
  search: SearchConfig;
}

export interface CatalogPlatform {
  type: 'unity-catalog' | 'glue' | 'hive' | 'datahub' | 'openmetadata' | 'custom';
  endpoint?: string;
  sync: boolean;
}

export interface DiscoveryConfig {
  autoDiscovery: boolean;
  frequency: string;
  profiling: boolean;
  classification: boolean;
}

export interface MetadataConfig {
  businessMetadata: boolean;
  technicalMetadata: boolean;
  operationalMetadata: boolean;
  customAttributes: CustomAttribute[];
}

export interface CustomAttribute {
  name: string;
  type: string;
  required: boolean;
  scope: ('table' | 'column' | 'pipeline')[];
}

export interface SearchConfig {
  fullText: boolean;
  facets: string[];
  suggestions: boolean;
}

export interface GovernanceConfig {
  policies: GovernancePolicy[];
  classification: ClassificationScheme;
  lineage: LineageConfig;
  quality: QualityConfig;
}

export interface GovernancePolicy {
  id: string;
  name: string;
  type: 'access' | 'retention' | 'privacy' | 'quality';
  scope: string[];
  rules: PolicyRule[];
  enforcement: 'strict' | 'advisory';
}

export interface PolicyRule {
  condition: string;
  action: string;
  priority: number;
}

export interface ClassificationScheme {
  levels: ClassificationLevel[];
  autoClassification: boolean;
  piiDetection: boolean;
}

export interface ClassificationLevel {
  name: string;
  description: string;
  handling: string[];
  retention: string;
}

export interface LineageConfig {
  enabled: boolean;
  granularity: 'table' | 'column';
  sources: ('spark' | 'dbt' | 'airflow' | 'custom')[];
  visualization: boolean;
}

export interface QualityConfig {
  dimensions: QualityDimension[];
  rules: QualityRule[];
  monitoring: QualityMonitoring;
}

export interface QualityDimension {
  name: string;
  weight: number;
  metrics: string[];
}

export interface QualityRule {
  id: string;
  name: string;
  type: 'completeness' | 'uniqueness' | 'validity' | 'consistency' | 'freshness';
  expression: string;
  threshold: number;
  severity: 'warning' | 'error' | 'critical';
}

export interface QualityMonitoring {
  enabled: boolean;
  frequency: string;
  alerting: boolean;
  trending: boolean;
}

// ============================================================================
// Access & Security Types
// ============================================================================

export interface AccessConfig {
  model: AccessModel;
  authentication: AuthenticationConfig;
  authorization: AuthorizationConfig;
  audit: AuditConfig;
}

export interface AccessModel {
  type: 'rbac' | 'abac' | 'tag-based';
  inheritance: boolean;
  defaultDeny: boolean;
}

export interface AuthenticationConfig {
  providers: AuthProvider[];
  sso: boolean;
  mfa: boolean;
}

export interface AuthProvider {
  type: 'iam' | 'ldap' | 'oauth' | 'saml';
  config: Record<string, unknown>;
}

export interface AuthorizationConfig {
  roles: AccessRole[];
  policies: AccessPolicy[];
  rowLevelSecurity: boolean;
  columnMasking: ColumnMasking;
}

export interface AccessRole {
  id: string;
  name: string;
  permissions: Permission[];
  scope: string[];
}

export interface Permission {
  resource: string;
  actions: ('read' | 'write' | 'delete' | 'admin')[];
}

export interface AccessPolicy {
  id: string;
  name: string;
  principals: string[];
  resources: string[];
  actions: string[];
  conditions?: PolicyCondition[];
}

export interface PolicyCondition {
  attribute: string;
  operator: string;
  value: unknown;
}

export interface ColumnMasking {
  enabled: boolean;
  rules: MaskingRule[];
}

export interface MaskingRule {
  column: string;
  method: 'full' | 'partial' | 'hash' | 'tokenize';
  roles: string[];
}

export interface AuditConfig {
  enabled: boolean;
  events: ('read' | 'write' | 'schema-change' | 'access-grant')[];
  retention: string;
  destination: string;
}

// ============================================================================
// Monitoring Configuration Types
// ============================================================================

export interface MonitoringConfig {
  metrics: MetricsConfig;
  logging: LoggingConfig;
  alerting: AlertingConfig;
  costManagement: CostConfig;
}

export interface MetricsConfig {
  enabled: boolean;
  provider: 'cloudwatch' | 'prometheus' | 'datadog' | 'custom';
  metrics: LakeMetric[];
  dashboards: Dashboard[];
}

export interface LakeMetric {
  name: string;
  type: 'gauge' | 'counter' | 'histogram';
  dimensions: string[];
  unit: string;
}

export interface Dashboard {
  id: string;
  name: string;
  widgets: Widget[];
  refresh: string;
}

export interface Widget {
  type: 'chart' | 'table' | 'gauge' | 'text';
  metrics: string[];
  config: Record<string, unknown>;
}

export interface LoggingConfig {
  level: 'debug' | 'info' | 'warn' | 'error';
  centralized: boolean;
  destination: string;
  retention: string;
}

export interface AlertingConfig {
  enabled: boolean;
  rules: AlertRule[];
  channels: AlertChannel[];
}

export interface AlertRule {
  id: string;
  name: string;
  condition: string;
  severity: 'low' | 'medium' | 'high' | 'critical';
  actions: string[];
}

export interface AlertChannel {
  id: string;
  type: 'email' | 'slack' | 'pagerduty' | 'webhook';
  config: Record<string, unknown>;
}

export interface CostConfig {
  tracking: boolean;
  budgets: Budget[];
  optimization: CostOptimization;
}

export interface Budget {
  name: string;
  amount: number;
  period: 'monthly' | 'quarterly' | 'yearly';
  alerts: { threshold: number; contacts: string[] }[];
}

export interface CostOptimization {
  enabled: boolean;
  recommendations: boolean;
  autoImplement: boolean;
}

// ============================================================================
// Table & Dataset Types
// ============================================================================

export interface DataTable {
  id: string;
  name: string;
  zone: string;
  layer: string;
  format: string;
  schema: TableSchema;
  partitioning?: PartitionScheme;
  statistics: TableStatistics;
  metadata: TableMetadata;
}

export interface TableSchema {
  columns: ColumnDefinition[];
  primaryKey?: string[];
}

export interface ColumnDefinition {
  name: string;
  type: string;
  nullable: boolean;
  description?: string;
  tags?: string[];
}

export interface TableStatistics {
  rowCount: number;
  sizeBytes: number;
  files: number;
  lastModified: string;
}

export interface TableMetadata {
  owner: string;
  description?: string;
  tags: string[];
  classification: string;
  lineage?: LineageInfo;
}

export interface LineageInfo {
  upstream: string[];
  downstream: string[];
  transformations: string[];
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
