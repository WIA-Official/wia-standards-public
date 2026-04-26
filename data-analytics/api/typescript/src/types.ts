/**
 * WIA Data Analytics Standard - TypeScript Type Definitions
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Data Analytics Types
// ============================================================================

export interface WIADataAnalyticsProject {
  standard: 'WIA-DATA-ANALYTICS';
  version: string;
  metadata: ProjectMetadata;
  dataSources: DataSourceConfig;
  processing: DataProcessing;
  analytics: AnalyticsCapabilities;
  visualization: VisualizationConfig;
  reporting: ReportingConfig;
  governance: DataGovernance;
  mlIntegration: MLIntegration;
  extensions?: Record<string, unknown>;
}

export interface ProjectMetadata {
  id: string;
  name: string;
  description?: string;
  organization: Organization;
  domain: AnalyticsDomain;
  createdAt: string;
  updatedAt?: string;
  status: ProjectStatus;
}

export interface Organization {
  name: string;
  department?: string;
  industry: string;
  contact: ContactInfo;
}

export interface ContactInfo {
  name: string;
  email: string;
  role: string;
}

export type AnalyticsDomain = 'business-intelligence' | 'data-science' | 'operational' | 'customer' | 'financial' | 'marketing' | 'hr' | 'supply-chain';
export type ProjectStatus = 'active' | 'development' | 'staging' | 'archived';

// ============================================================================
// Data Source Types
// ============================================================================

export interface DataSourceConfig {
  sources: DataSource[];
  connections: ConnectionConfig[];
  ingestion: IngestionConfig;
  catalog: DataCatalogEntry[];
}

export interface DataSource {
  id: string;
  name: string;
  type: DataSourceType;
  connection: string;
  schema?: string;
  tables?: string[];
  refreshSchedule?: string;
  status: 'active' | 'inactive' | 'error';
}

export type DataSourceType = 'database' | 'api' | 'file' | 'stream' | 'data-lake' | 'data-warehouse' | 'saas' | 'iot';

export interface ConnectionConfig {
  id: string;
  name: string;
  type: ConnectionType;
  host?: string;
  port?: number;
  database?: string;
  credentials: CredentialReference;
  ssl: boolean;
  poolSize?: number;
  timeout?: number;
}

export type ConnectionType = 'postgresql' | 'mysql' | 'mongodb' | 'snowflake' | 'bigquery' | 'redshift' | 'databricks' | 's3' | 'azure-blob' | 'gcs' | 'kafka' | 'rest-api';

export interface CredentialReference {
  type: 'secret-manager' | 'vault' | 'env' | 'oauth';
  reference: string;
}

export interface IngestionConfig {
  mode: 'batch' | 'streaming' | 'hybrid';
  batchSchedule?: CronSchedule;
  streamingConfig?: StreamingConfig;
  validation: ValidationRules;
  errorHandling: ErrorHandlingConfig;
}

export interface CronSchedule {
  expression: string;
  timezone: string;
  enabled: boolean;
}

export interface StreamingConfig {
  platform: 'kafka' | 'kinesis' | 'pubsub' | 'eventhub';
  topics: string[];
  consumerGroup: string;
  offsetReset: 'earliest' | 'latest';
  batchSize: number;
  processingTime: string;
}

export interface ValidationRules {
  schemaValidation: boolean;
  nullChecks: boolean;
  typeChecks: boolean;
  customRules: ValidationRule[];
}

export interface ValidationRule {
  id: string;
  field: string;
  type: 'range' | 'regex' | 'enum' | 'custom';
  expression: string;
  severity: 'error' | 'warning';
}

export interface ErrorHandlingConfig {
  strategy: 'skip' | 'fail' | 'quarantine' | 'retry';
  maxRetries?: number;
  deadLetterQueue?: string;
  alerting: boolean;
}

export interface DataCatalogEntry {
  id: string;
  name: string;
  source: string;
  schema: SchemaDefinition;
  description?: string;
  owner: string;
  tags: string[];
  lineage?: DataLineage;
  quality: QualityMetrics;
}

export interface SchemaDefinition {
  columns: ColumnDefinition[];
  primaryKey?: string[];
  partitioning?: PartitionConfig;
}

export interface ColumnDefinition {
  name: string;
  type: DataType;
  nullable: boolean;
  description?: string;
  pii?: boolean;
  sensitive?: boolean;
}

export type DataType = 'string' | 'integer' | 'float' | 'boolean' | 'date' | 'timestamp' | 'json' | 'array' | 'binary';

export interface PartitionConfig {
  columns: string[];
  type: 'date' | 'range' | 'hash';
  granularity?: 'hour' | 'day' | 'month' | 'year';
}

export interface DataLineage {
  upstream: string[];
  downstream: string[];
  transformations: string[];
}

export interface QualityMetrics {
  completeness: number;
  accuracy: number;
  freshness: string;
  lastChecked: string;
}

// ============================================================================
// Data Processing Types
// ============================================================================

export interface DataProcessing {
  pipelines: Pipeline[];
  transformations: TransformationLibrary;
  scheduling: SchedulingConfig;
  orchestration: OrchestrationConfig;
}

export interface Pipeline {
  id: string;
  name: string;
  description?: string;
  stages: PipelineStage[];
  triggers: PipelineTrigger[];
  config: PipelineConfig;
  status: 'active' | 'paused' | 'failed' | 'draft';
}

export interface PipelineStage {
  id: string;
  name: string;
  type: StageType;
  inputs: string[];
  outputs: string[];
  transformation?: string;
  config: Record<string, unknown>;
  dependencies: string[];
}

export type StageType = 'extract' | 'transform' | 'load' | 'validate' | 'aggregate' | 'join' | 'filter' | 'enrich' | 'ml-inference';

export interface PipelineTrigger {
  type: 'schedule' | 'event' | 'api' | 'dependency';
  config: Record<string, unknown>;
}

export interface PipelineConfig {
  parallelism: number;
  retryPolicy: RetryPolicy;
  timeout: string;
  resources: ResourceAllocation;
}

export interface RetryPolicy {
  maxAttempts: number;
  backoff: 'linear' | 'exponential';
  initialDelay: string;
}

export interface ResourceAllocation {
  cpu: string;
  memory: string;
  executors?: number;
}

export interface TransformationLibrary {
  custom: CustomTransformation[];
  sql: SQLTransformation[];
  python: PythonTransformation[];
}

export interface CustomTransformation {
  id: string;
  name: string;
  type: 'map' | 'filter' | 'aggregate' | 'window' | 'join';
  definition: string;
  inputSchema: SchemaDefinition;
  outputSchema: SchemaDefinition;
}

export interface SQLTransformation {
  id: string;
  name: string;
  query: string;
  parameters?: Record<string, unknown>;
}

export interface PythonTransformation {
  id: string;
  name: string;
  module: string;
  function: string;
  dependencies: string[];
}

export interface SchedulingConfig {
  scheduler: 'airflow' | 'prefect' | 'dagster' | 'custom';
  defaultTimezone: string;
  concurrencyLimit: number;
}

export interface OrchestrationConfig {
  platform: 'kubernetes' | 'spark' | 'flink' | 'dbt' | 'custom';
  cluster?: string;
  namespace?: string;
  serviceAccount?: string;
}

// ============================================================================
// Analytics Capabilities Types
// ============================================================================

export interface AnalyticsCapabilities {
  descriptive: DescriptiveAnalytics;
  diagnostic: DiagnosticAnalytics;
  predictive: PredictiveAnalytics;
  prescriptive: PrescriptiveAnalytics;
  realtime: RealtimeAnalytics;
}

export interface DescriptiveAnalytics {
  kpis: KPIDefinition[];
  aggregations: AggregationConfig[];
  dimensions: DimensionConfig[];
  measures: MeasureConfig[];
}

export interface KPIDefinition {
  id: string;
  name: string;
  description?: string;
  formula: string;
  unit: string;
  target?: number;
  thresholds: { level: string; min?: number; max?: number }[];
}

export interface AggregationConfig {
  id: string;
  name: string;
  source: string;
  groupBy: string[];
  metrics: MetricAggregation[];
  filters?: FilterCondition[];
  window?: WindowConfig;
}

export interface MetricAggregation {
  name: string;
  column: string;
  function: 'sum' | 'avg' | 'count' | 'min' | 'max' | 'median' | 'percentile' | 'stddev';
  percentile?: number;
}

export interface FilterCondition {
  column: string;
  operator: 'eq' | 'ne' | 'gt' | 'gte' | 'lt' | 'lte' | 'in' | 'between' | 'like';
  value: unknown;
}

export interface WindowConfig {
  type: 'tumbling' | 'sliding' | 'session';
  size: string;
  slide?: string;
}

export interface DimensionConfig {
  id: string;
  name: string;
  column: string;
  type: 'categorical' | 'temporal' | 'hierarchical';
  hierarchy?: string[];
}

export interface MeasureConfig {
  id: string;
  name: string;
  column: string;
  aggregation: string;
  format?: string;
}

export interface DiagnosticAnalytics {
  drillDown: DrillDownConfig[];
  correlation: CorrelationAnalysis[];
  segmentation: SegmentationConfig[];
  anomalyDetection: AnomalyConfig;
}

export interface DrillDownConfig {
  id: string;
  name: string;
  dimensions: string[];
  levels: string[];
}

export interface CorrelationAnalysis {
  id: string;
  name: string;
  variables: string[];
  method: 'pearson' | 'spearman' | 'kendall';
  threshold: number;
}

export interface SegmentationConfig {
  id: string;
  name: string;
  method: 'manual' | 'clustering' | 'decision-tree';
  features: string[];
  segments?: Segment[];
}

export interface Segment {
  id: string;
  name: string;
  criteria: FilterCondition[];
  size?: number;
}

export interface AnomalyConfig {
  enabled: boolean;
  methods: ('zscore' | 'iqr' | 'isolation-forest' | 'dbscan')[];
  sensitivity: 'low' | 'medium' | 'high';
  metrics: string[];
}

export interface PredictiveAnalytics {
  models: PredictiveModel[];
  forecasting: ForecastConfig[];
  propensity: PropensityModel[];
}

export interface PredictiveModel {
  id: string;
  name: string;
  type: 'regression' | 'classification' | 'clustering' | 'timeseries';
  algorithm: string;
  features: string[];
  target: string;
  performance: ModelPerformance;
  status: 'training' | 'deployed' | 'deprecated';
}

export interface ModelPerformance {
  metric: string;
  value: number;
  validationDate: string;
}

export interface ForecastConfig {
  id: string;
  name: string;
  metric: string;
  horizon: number;
  granularity: 'hour' | 'day' | 'week' | 'month';
  model: string;
  confidence: number;
}

export interface PropensityModel {
  id: string;
  name: string;
  outcome: string;
  features: string[];
  segments?: string[];
}

export interface PrescriptiveAnalytics {
  optimization: OptimizationConfig[];
  recommendations: RecommendationEngine[];
  simulations: SimulationConfig[];
}

export interface OptimizationConfig {
  id: string;
  name: string;
  objective: 'maximize' | 'minimize';
  target: string;
  constraints: Constraint[];
  variables: OptVariable[];
}

export interface Constraint {
  expression: string;
  type: 'equality' | 'inequality';
}

export interface OptVariable {
  name: string;
  type: 'continuous' | 'integer' | 'binary';
  bounds?: { min: number; max: number };
}

export interface RecommendationEngine {
  id: string;
  name: string;
  type: 'collaborative' | 'content-based' | 'hybrid';
  items: string;
  users: string;
  features?: string[];
}

export interface SimulationConfig {
  id: string;
  name: string;
  type: 'monte-carlo' | 'what-if' | 'scenario';
  variables: string[];
  iterations?: number;
}

export interface RealtimeAnalytics {
  enabled: boolean;
  streaming: StreamAnalyticsConfig;
  alerts: AlertConfig[];
  dashboardRefresh: string;
}

export interface StreamAnalyticsConfig {
  platform: 'kafka-streams' | 'flink' | 'spark-streaming' | 'ksql';
  latency: string;
  windows: WindowConfig[];
}

export interface AlertConfig {
  id: string;
  name: string;
  metric: string;
  condition: FilterCondition;
  severity: 'info' | 'warning' | 'critical';
  channels: string[];
}

// ============================================================================
// Visualization & Reporting Types
// ============================================================================

export interface VisualizationConfig {
  platform: VisualizationPlatform;
  dashboards: Dashboard[];
  themes: ThemeConfig;
  embedding: EmbeddingConfig;
}

export interface VisualizationPlatform {
  type: 'tableau' | 'powerbi' | 'looker' | 'superset' | 'metabase' | 'custom';
  version?: string;
  server?: string;
}

export interface Dashboard {
  id: string;
  name: string;
  description?: string;
  layout: LayoutConfig;
  widgets: Widget[];
  filters: DashboardFilter[];
  sharing: SharingConfig;
  refreshInterval?: string;
}

export interface LayoutConfig {
  type: 'grid' | 'freeform' | 'responsive';
  columns?: number;
  rows?: number;
}

export interface Widget {
  id: string;
  type: WidgetType;
  title: string;
  position: { x: number; y: number; width: number; height: number };
  dataSource: string;
  config: Record<string, unknown>;
  interactions?: WidgetInteraction[];
}

export type WidgetType = 'chart' | 'table' | 'kpi' | 'map' | 'text' | 'filter' | 'image';

export interface WidgetInteraction {
  type: 'click' | 'hover' | 'select';
  action: 'filter' | 'drilldown' | 'link' | 'popup';
  target?: string;
}

export interface DashboardFilter {
  id: string;
  name: string;
  type: 'dropdown' | 'date-range' | 'slider' | 'search';
  dimension: string;
  defaultValue?: unknown;
  cascading?: string[];
}

export interface SharingConfig {
  visibility: 'private' | 'team' | 'organization' | 'public';
  roles: string[];
  embedding: boolean;
}

export interface ThemeConfig {
  primary: string;
  secondary: string;
  palette: string[];
  fonts: { heading: string; body: string };
}

export interface EmbeddingConfig {
  enabled: boolean;
  authentication: 'token' | 'sso' | 'public';
  allowedDomains: string[];
}

export interface ReportingConfig {
  templates: ReportTemplate[];
  scheduling: ReportSchedule[];
  distribution: DistributionConfig;
  formats: ('pdf' | 'excel' | 'csv' | 'html')[];
}

export interface ReportTemplate {
  id: string;
  name: string;
  type: 'standard' | 'executive' | 'detailed' | 'custom';
  sections: ReportSection[];
  parameters: ReportParameter[];
}

export interface ReportSection {
  id: string;
  title: string;
  type: 'summary' | 'chart' | 'table' | 'narrative';
  content: string;
}

export interface ReportParameter {
  id: string;
  name: string;
  type: 'date' | 'string' | 'number' | 'list';
  required: boolean;
  defaultValue?: unknown;
}

export interface ReportSchedule {
  id: string;
  report: string;
  frequency: 'daily' | 'weekly' | 'monthly' | 'quarterly';
  day?: number;
  time: string;
  timezone: string;
  recipients: string[];
  format: string;
}

export interface DistributionConfig {
  channels: ('email' | 'slack' | 'teams' | 'sftp' | 'api')[];
  emailConfig?: { from: string; replyTo?: string };
  slackConfig?: { webhook: string };
}

// ============================================================================
// Governance & ML Integration Types
// ============================================================================

export interface DataGovernance {
  policies: GovernancePolicy[];
  accessControl: AccessControlConfig;
  privacy: PrivacyConfig;
  audit: AuditConfig;
}

export interface GovernancePolicy {
  id: string;
  name: string;
  type: 'retention' | 'access' | 'quality' | 'privacy';
  rules: PolicyRule[];
  enforcement: 'strict' | 'advisory';
}

export interface PolicyRule {
  id: string;
  condition: string;
  action: string;
  priority: number;
}

export interface AccessControlConfig {
  model: 'rbac' | 'abac';
  roles: AccessRole[];
  dataPermissions: DataPermission[];
}

export interface AccessRole {
  id: string;
  name: string;
  permissions: string[];
  dataAccess: string[];
}

export interface DataPermission {
  resource: string;
  operations: ('read' | 'write' | 'delete' | 'export')[];
  conditions?: FilterCondition[];
}

export interface PrivacyConfig {
  piiFields: string[];
  anonymization: AnonymizationConfig;
  consent: ConsentConfig;
}

export interface AnonymizationConfig {
  methods: { field: string; method: 'hash' | 'mask' | 'generalize' | 'suppress' }[];
  kAnonymity?: number;
}

export interface ConsentConfig {
  required: boolean;
  purposes: string[];
  tracking: boolean;
}

export interface AuditConfig {
  enabled: boolean;
  events: string[];
  retention: string;
  storage: string;
}

export interface MLIntegration {
  platforms: MLPlatform[];
  featureStore: FeatureStoreConfig;
  modelRegistry: ModelRegistryConfig;
  serving: ModelServingConfig;
}

export interface MLPlatform {
  type: 'mlflow' | 'sagemaker' | 'vertex' | 'databricks-ml' | 'custom';
  endpoint: string;
  authentication: string;
}

export interface FeatureStoreConfig {
  enabled: boolean;
  platform: 'feast' | 'tecton' | 'sagemaker' | 'vertex' | 'custom';
  features: FeatureSet[];
}

export interface FeatureSet {
  id: string;
  name: string;
  entity: string;
  features: FeatureDefinition[];
  ttl?: string;
}

export interface FeatureDefinition {
  name: string;
  type: DataType;
  source: string;
  transformation?: string;
}

export interface ModelRegistryConfig {
  platform: string;
  versioning: boolean;
  lifecycle: string[];
}

export interface ModelServingConfig {
  platform: 'kubernetes' | 'sagemaker' | 'vertex' | 'seldon' | 'custom';
  scaling: { min: number; max: number };
  monitoring: boolean;
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
