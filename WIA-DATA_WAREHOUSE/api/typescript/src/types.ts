/**
 * WIA Data Warehouse Standard - TypeScript Type Definitions
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Data Warehouse Types
// ============================================================================

export interface WIADataWarehouse {
  standard: 'WIA-DATA-WAREHOUSE';
  version: string;
  warehouse: WarehouseMetadata;
  schemas: SchemaDefinition[];
  tables: TableDefinition[];
  relationships: RelationshipDefinition[];
  policies: DataPolicy[];
  performance?: PerformanceConfiguration;
  extensions?: Record<string, unknown>;
}

export interface WarehouseMetadata {
  id: string;
  name: string;
  description?: string;
  type: WarehouseType;
  provider: WarehouseProvider;
  region: string;
  createdAt: string;
  updatedAt?: string;
  status: WarehouseStatus;
  tags?: string[];
}

export type WarehouseType =
  | 'enterprise'
  | 'cloud-native'
  | 'lakehouse'
  | 'federated'
  | 'hybrid';

export type WarehouseProvider =
  | 'snowflake'
  | 'databricks'
  | 'bigquery'
  | 'redshift'
  | 'synapse'
  | 'teradata'
  | 'postgresql'
  | 'clickhouse'
  | 'custom';

export type WarehouseStatus =
  | 'provisioning'
  | 'active'
  | 'suspended'
  | 'scaling'
  | 'maintenance'
  | 'terminated';

// ============================================================================
// Schema Types
// ============================================================================

export interface SchemaDefinition {
  id: string;
  name: string;
  type: SchemaType;
  description?: string;
  owner?: string;
  managedBy?: 'dbt' | 'terraform' | 'manual' | 'custom';
  metadata?: SchemaMetadata;
}

export type SchemaType =
  | 'raw'
  | 'staging'
  | 'integration'
  | 'mart'
  | 'archive'
  | 'sandbox';

export interface SchemaMetadata {
  domain?: string;
  classification?: string;
  retentionDays?: number;
  accessLevel?: 'public' | 'internal' | 'restricted' | 'confidential';
}

// ============================================================================
// Table Types
// ============================================================================

export interface TableDefinition {
  id: string;
  schemaId: string;
  name: string;
  type: TableType;
  description?: string;
  columns: ColumnDefinition[];
  primaryKey?: string[];
  clustering?: ClusteringConfig;
  partitioning?: PartitioningConfig;
  materialization?: MaterializationConfig;
  metadata?: TableMetadata;
}

export type TableType =
  | 'table'
  | 'view'
  | 'materialized-view'
  | 'external'
  | 'temporary'
  | 'transient';

export interface ColumnDefinition {
  name: string;
  type: DataType;
  nullable: boolean;
  defaultValue?: unknown;
  description?: string;
  businessName?: string;
  sensitivity?: DataSensitivity;
  constraints?: ColumnConstraint[];
  metadata?: ColumnMetadata;
}

export type DataType =
  | 'string'
  | 'varchar'
  | 'char'
  | 'integer'
  | 'bigint'
  | 'smallint'
  | 'decimal'
  | 'float'
  | 'double'
  | 'boolean'
  | 'date'
  | 'timestamp'
  | 'timestamptz'
  | 'time'
  | 'binary'
  | 'json'
  | 'array'
  | 'map'
  | 'struct';

export type DataSensitivity =
  | 'none'
  | 'internal'
  | 'confidential'
  | 'pii'
  | 'phi'
  | 'pci';

export interface ColumnConstraint {
  type: 'not-null' | 'unique' | 'check' | 'foreign-key';
  expression?: string;
  references?: ForeignKeyReference;
}

export interface ForeignKeyReference {
  table: string;
  column: string;
  onDelete?: 'cascade' | 'set-null' | 'restrict' | 'no-action';
  onUpdate?: 'cascade' | 'set-null' | 'restrict' | 'no-action';
}

export interface ColumnMetadata {
  sourceSystem?: string;
  sourceColumn?: string;
  transformation?: string;
  tags?: string[];
}

export interface ClusteringConfig {
  columns: string[];
  auto?: boolean;
  depth?: number;
}

export interface PartitioningConfig {
  type: 'range' | 'list' | 'hash' | 'time';
  column: string;
  granularity?: 'hour' | 'day' | 'month' | 'year';
  retention?: number;
}

export interface MaterializationConfig {
  type: 'table' | 'view' | 'incremental' | 'ephemeral';
  strategy?: 'append' | 'merge' | 'delete+insert' | 'upsert';
  uniqueKey?: string[];
  refreshInterval?: number;
}

export interface TableMetadata {
  owner?: string;
  steward?: string;
  domain?: string;
  sla?: SLAConfig;
  documentation?: string;
  sourceQuery?: string;
  tests?: TableTest[];
}

export interface SLAConfig {
  availability: number;
  freshness: FreshnessConfig;
  quality?: QualityConfig;
}

export interface FreshnessConfig {
  maxDelay: number;
  unit: 'minutes' | 'hours' | 'days';
  warningThreshold?: number;
}

export interface QualityConfig {
  completeness: number;
  accuracy: number;
  consistency: number;
}

export interface TableTest {
  name: string;
  type: 'unique' | 'not_null' | 'accepted_values' | 'relationships' | 'custom';
  column?: string;
  config?: Record<string, unknown>;
}

// ============================================================================
// Relationship Types
// ============================================================================

export interface RelationshipDefinition {
  id: string;
  name: string;
  type: RelationshipType;
  source: RelationshipEndpoint;
  target: RelationshipEndpoint;
  cardinality: Cardinality;
  enforced?: boolean;
  description?: string;
}

export type RelationshipType =
  | 'foreign-key'
  | 'logical'
  | 'derived';

export interface RelationshipEndpoint {
  tableId: string;
  columns: string[];
}

export type Cardinality =
  | 'one-to-one'
  | 'one-to-many'
  | 'many-to-one'
  | 'many-to-many';

// ============================================================================
// Data Policy Types
// ============================================================================

export interface DataPolicy {
  id: string;
  name: string;
  type: PolicyType;
  description?: string;
  scope: PolicyScope;
  rules: PolicyRule[];
  enabled: boolean;
}

export type PolicyType =
  | 'access-control'
  | 'masking'
  | 'retention'
  | 'audit'
  | 'encryption';

export interface PolicyScope {
  schemas?: string[];
  tables?: string[];
  columns?: string[];
  roles?: string[];
  users?: string[];
}

export type PolicyRule =
  | AccessControlRule
  | MaskingRule
  | RetentionRule
  | AuditRule
  | EncryptionRule;

export interface AccessControlRule {
  type: 'access-control';
  action: 'allow' | 'deny';
  operations: ('select' | 'insert' | 'update' | 'delete')[];
  condition?: string;
}

export interface MaskingRule {
  type: 'masking';
  method: 'full' | 'partial' | 'hash' | 'null' | 'custom';
  pattern?: string;
  preserveFormat?: boolean;
}

export interface RetentionRule {
  type: 'retention';
  period: number;
  unit: 'days' | 'months' | 'years';
  action: 'delete' | 'archive' | 'anonymize';
}

export interface AuditRule {
  type: 'audit';
  events: ('read' | 'write' | 'delete' | 'schema-change')[];
  destination: string;
  includeData?: boolean;
}

export interface EncryptionRule {
  type: 'encryption';
  algorithm: 'aes-256' | 'aes-128' | 'custom';
  keyManagement: 'kms' | 'vault' | 'internal';
  keyId?: string;
}

// ============================================================================
// Performance Configuration
// ============================================================================

export interface PerformanceConfiguration {
  compute: ComputeConfig;
  storage: StorageConfig;
  caching?: CachingConfig;
  optimization?: OptimizationConfig;
}

export interface ComputeConfig {
  size: ComputeSize;
  autoScale?: AutoScaleConfig;
  concurrency?: number;
  timeout?: number;
}

export type ComputeSize =
  | 'xs'
  | 'small'
  | 'medium'
  | 'large'
  | 'xl'
  | '2xl'
  | '3xl'
  | '4xl';

export interface AutoScaleConfig {
  enabled: boolean;
  minSize: ComputeSize;
  maxSize: ComputeSize;
  scaleUpThreshold?: number;
  scaleDownThreshold?: number;
  cooldownSeconds?: number;
}

export interface StorageConfig {
  type: 'managed' | 'external' | 'hybrid';
  format?: 'parquet' | 'orc' | 'delta' | 'iceberg';
  compression?: 'snappy' | 'gzip' | 'zstd' | 'lz4';
  lifecycle?: StorageLifecycle[];
}

export interface StorageLifecycle {
  name: string;
  condition: string;
  action: 'archive' | 'delete' | 'transition';
  target?: string;
}

export interface CachingConfig {
  enabled: boolean;
  type: 'result' | 'data' | 'both';
  ttl?: number;
  maxSize?: string;
}

export interface OptimizationConfig {
  autoVacuum?: boolean;
  autoAnalyze?: boolean;
  queryPruning?: boolean;
  resultCaching?: boolean;
  spillToDisk?: boolean;
}

// ============================================================================
// Query and Analytics Types
// ============================================================================

export interface QueryDefinition {
  id: string;
  name: string;
  sql: string;
  parameters?: QueryParameter[];
  schedule?: QuerySchedule;
  destination?: QueryDestination;
}

export interface QueryParameter {
  name: string;
  type: DataType;
  defaultValue?: unknown;
  required?: boolean;
}

export interface QuerySchedule {
  cron: string;
  timezone?: string;
  enabled: boolean;
}

export interface QueryDestination {
  type: 'table' | 'file' | 'notification';
  target: string;
  mode?: 'append' | 'overwrite' | 'merge';
}

export interface QueryResult {
  queryId: string;
  status: 'running' | 'completed' | 'failed' | 'cancelled';
  rowCount?: number;
  bytesProcessed?: number;
  duration?: number;
  columns?: { name: string; type: string }[];
  data?: unknown[][];
  error?: QueryError;
}

export interface QueryError {
  code: string;
  message: string;
  line?: number;
  position?: number;
}

// ============================================================================
// API Types
// ============================================================================

export interface APIConfig {
  baseURL: string;
  apiKey?: string;
  timeout?: number;
}

export interface WarehouseResponse {
  id: string;
  name: string;
  type: WarehouseType;
  provider: WarehouseProvider;
  status: WarehouseStatus;
  region: string;
  createdAt: string;
  links: {
    self: string;
    schemas?: string;
    tables?: string;
  };
}

export interface TableResponse {
  id: string;
  name: string;
  schemaName: string;
  type: TableType;
  rowCount?: number;
  sizeBytes?: number;
  lastModified?: string;
  links: {
    self: string;
    columns?: string;
    data?: string;
  };
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
