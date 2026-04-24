/**
 * WIA Big Data Standard - Type Definitions
 *
 * @packageDocumentation
 * @module wia-big-data
 */

/**
 * Processing mode
 */
export enum ProcessingMode {
  Batch = 'batch',
  Stream = 'stream',
  MicroBatch = 'micro_batch',
  Lambda = 'lambda',
  Kappa = 'kappa'
}

/**
 * Data format
 */
export enum DataFormat {
  JSON = 'json',
  Avro = 'avro',
  Parquet = 'parquet',
  ORC = 'orc',
  CSV = 'csv',
  Protobuf = 'protobuf',
  Arrow = 'arrow'
}

/**
 * Storage tier
 */
export enum StorageTier {
  Hot = 'hot',
  Warm = 'warm',
  Cold = 'cold',
  Archive = 'archive'
}

/**
 * Data quality level
 */
export enum DataQualityLevel {
  Raw = 'raw',
  Cleaned = 'cleaned',
  Validated = 'validated',
  Curated = 'curated',
  Production = 'production'
}

/**
 * Compression type
 */
export enum CompressionType {
  None = 'none',
  Gzip = 'gzip',
  Snappy = 'snappy',
  LZ4 = 'lz4',
  Zstd = 'zstd',
  Brotli = 'brotli'
}

/**
 * Data source configuration
 */
export interface DataSource {
  /** Source ID */
  id: string;
  /** Source name */
  name: string;
  /** Source type */
  type: 'kafka' | 'kinesis' | 's3' | 'hdfs' | 'database' | 'api' | 'file';
  /** Connection string */
  connectionString: string;
  /** Format */
  format: DataFormat;
  /** Schema */
  schema?: Schema;
  /** Partitioning */
  partitioning?: PartitionSpec;
  /** Metadata */
  metadata?: Record<string, unknown>;
}

/**
 * Schema definition
 */
export interface Schema {
  /** Schema ID */
  id: string;
  /** Schema version */
  version: string;
  /** Fields */
  fields: SchemaField[];
  /** Primary key */
  primaryKey?: string[];
  /** Timestamp field */
  timestampField?: string;
}

/**
 * Schema field
 */
export interface SchemaField {
  /** Field name */
  name: string;
  /** Data type */
  type: 'string' | 'integer' | 'long' | 'float' | 'double' | 'boolean' | 'timestamp' | 'date' | 'binary' | 'array' | 'map' | 'struct';
  /** Is nullable */
  nullable: boolean;
  /** Description */
  description?: string;
  /** Nested fields (for struct/array/map) */
  nestedFields?: SchemaField[];
  /** Default value */
  defaultValue?: unknown;
}

/**
 * Partition specification
 */
export interface PartitionSpec {
  /** Partition columns */
  columns: string[];
  /** Partition type */
  type: 'identity' | 'year' | 'month' | 'day' | 'hour' | 'bucket' | 'truncate';
  /** Number of buckets */
  numBuckets?: number;
}

/**
 * Data pipeline
 */
export interface Pipeline {
  /** Pipeline ID */
  id: string;
  /** Pipeline name */
  name: string;
  /** Description */
  description?: string;
  /** Processing mode */
  mode: ProcessingMode;
  /** Source */
  source: DataSource;
  /** Transformations */
  transformations: Transformation[];
  /** Sink */
  sink: DataSink;
  /** Schedule */
  schedule?: Schedule;
  /** Retry policy */
  retryPolicy?: RetryPolicy;
  /** State */
  state: PipelineState;
}

/**
 * Pipeline state
 */
export enum PipelineState {
  Created = 'created',
  Running = 'running',
  Paused = 'paused',
  Failed = 'failed',
  Completed = 'completed',
  Stopped = 'stopped'
}

/**
 * Transformation
 */
export interface Transformation {
  /** Transformation ID */
  id: string;
  /** Type */
  type: TransformationType;
  /** Configuration */
  config: Record<string, unknown>;
  /** Input columns */
  inputColumns?: string[];
  /** Output columns */
  outputColumns?: string[];
}

/**
 * Transformation type
 */
export enum TransformationType {
  Filter = 'filter',
  Map = 'map',
  FlatMap = 'flatmap',
  Aggregate = 'aggregate',
  Join = 'join',
  Window = 'window',
  Deduplicate = 'deduplicate',
  Enrich = 'enrich',
  Validate = 'validate',
  Custom = 'custom'
}

/**
 * Data sink
 */
export interface DataSink {
  /** Sink ID */
  id: string;
  /** Sink name */
  name: string;
  /** Sink type */
  type: 'kafka' | 'kinesis' | 's3' | 'hdfs' | 'database' | 'elasticsearch' | 'api';
  /** Connection string */
  connectionString: string;
  /** Format */
  format: DataFormat;
  /** Compression */
  compression?: CompressionType;
  /** Write mode */
  writeMode: 'append' | 'overwrite' | 'upsert' | 'merge';
}

/**
 * Schedule
 */
export interface Schedule {
  /** Schedule type */
  type: 'cron' | 'interval' | 'event';
  /** Cron expression */
  cronExpression?: string;
  /** Interval in seconds */
  intervalSeconds?: number;
  /** Event trigger */
  eventTrigger?: string;
  /** Timezone */
  timezone?: string;
}

/**
 * Retry policy
 */
export interface RetryPolicy {
  /** Max retries */
  maxRetries: number;
  /** Initial delay in ms */
  initialDelayMs: number;
  /** Max delay in ms */
  maxDelayMs: number;
  /** Backoff multiplier */
  backoffMultiplier: number;
}

/**
 * Data quality rule
 */
export interface DataQualityRule {
  /** Rule ID */
  id: string;
  /** Rule name */
  name: string;
  /** Rule type */
  type: 'completeness' | 'uniqueness' | 'validity' | 'consistency' | 'timeliness' | 'custom';
  /** Column */
  column?: string;
  /** Expression */
  expression: string;
  /** Threshold */
  threshold: number;
  /** Severity */
  severity: 'warning' | 'error' | 'critical';
}

/**
 * Data quality result
 */
export interface DataQualityResult {
  /** Rule ID */
  ruleId: string;
  /** Passed */
  passed: boolean;
  /** Score (0-100) */
  score: number;
  /** Records checked */
  recordsChecked: number;
  /** Records failed */
  recordsFailed: number;
  /** Timestamp */
  timestamp: Date;
}

/**
 * Job execution
 */
export interface JobExecution {
  /** Execution ID */
  id: string;
  /** Pipeline ID */
  pipelineId: string;
  /** Start time */
  startTime: Date;
  /** End time */
  endTime?: Date;
  /** Status */
  status: 'running' | 'success' | 'failed' | 'cancelled';
  /** Records processed */
  recordsProcessed: number;
  /** Bytes processed */
  bytesProcessed: number;
  /** Error message */
  errorMessage?: string;
  /** Metrics */
  metrics: ExecutionMetrics;
}

/**
 * Execution metrics
 */
export interface ExecutionMetrics {
  /** Duration in ms */
  durationMs: number;
  /** Records per second */
  recordsPerSecond: number;
  /** Bytes per second */
  bytesPerSecond: number;
  /** CPU usage */
  cpuUsage?: number;
  /** Memory usage */
  memoryUsage?: number;
}

/**
 * Data catalog entry
 */
export interface CatalogEntry {
  /** Entry ID */
  id: string;
  /** Dataset name */
  name: string;
  /** Description */
  description?: string;
  /** Schema */
  schema: Schema;
  /** Location */
  location: string;
  /** Format */
  format: DataFormat;
  /** Owner */
  owner: string;
  /** Tags */
  tags: string[];
  /** Quality level */
  qualityLevel: DataQualityLevel;
  /** Storage tier */
  storageTier: StorageTier;
  /** Lineage */
  lineage?: DataLineage;
  /** Created at */
  createdAt: Date;
  /** Updated at */
  updatedAt: Date;
}

/**
 * Data lineage
 */
export interface DataLineage {
  /** Upstream sources */
  upstream: LineageNode[];
  /** Downstream targets */
  downstream: LineageNode[];
  /** Transformations applied */
  transformations: string[];
}

/**
 * Lineage node
 */
export interface LineageNode {
  /** Node ID */
  id: string;
  /** Node name */
  name: string;
  /** Node type */
  type: 'dataset' | 'pipeline' | 'external';
}

/**
 * SDK configuration
 */
export interface BigDataConfig {
  /** Cluster endpoint */
  clusterEndpoint: string;
  /** API key */
  apiKey?: string;
  /** Default format */
  defaultFormat: DataFormat;
  /** Default compression */
  defaultCompression: CompressionType;
  /** Enable lineage tracking */
  lineageEnabled: boolean;
  /** Enable data quality checks */
  qualityChecksEnabled: boolean;
}

/**
 * Certification level
 */
export enum CertificationLevel {
  Bronze = 'bronze',
  Silver = 'silver',
  Gold = 'gold'
}

/**
 * Compliance report
 */
export interface ComplianceReport {
  standard: 'WIA-BIG-DATA';
  testDate: string;
  config: BigDataConfig;
  targetLevel: CertificationLevel;
  tests: TestResult[];
  passed: boolean;
  achievedLevel?: CertificationLevel;
}

/**
 * Test result
 */
export interface TestResult {
  testName: string;
  passed: boolean;
  notes?: string;
}
