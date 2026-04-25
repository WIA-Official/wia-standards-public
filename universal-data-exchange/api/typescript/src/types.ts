/**
 * WIA-CORE-003: Universal Data Exchange - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Core Standards Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Enums
// ============================================================================

/**
 * Supported data formats for universal exchange
 */
export enum DataFormat {
  JSON = 'json',
  JSON_LD = 'jsonld',
  XML = 'xml',
  YAML = 'yaml',
  PROTOBUF = 'protobuf',
  MSGPACK = 'msgpack',
  CBOR = 'cbor',
  AVRO = 'avro',
}

/**
 * Character encoding types
 */
export enum Encoding {
  UTF8 = 'utf-8',
  UTF16 = 'utf-16',
  ASCII = 'ascii',
  BINARY = 'binary',
}

/**
 * Compression algorithms
 */
export enum CompressionAlgorithm {
  NONE = 'none',
  GZIP = 'gzip',
  BROTLI = 'brotli',
  ZSTD = 'zstd',
  LZ4 = 'lz4',
}

/**
 * Hash algorithms for integrity verification
 */
export enum HashAlgorithm {
  SHA256 = 'sha256',
  SHA512 = 'sha512',
  SHA3_256 = 'sha3-256',
  BLAKE3 = 'blake3',
}

/**
 * Encryption algorithms
 */
export enum EncryptionAlgorithm {
  NONE = 'none',
  AES_256_GCM = 'aes-256-gcm',
  CHACHA20_POLY1305 = 'chacha20-poly1305',
}

/**
 * Transformation step types
 */
export enum TransformationType {
  VALIDATOR = 'validator',
  MAPPER = 'mapper',
  CONVERTER = 'converter',
  ENRICHER = 'enricher',
  FILTER = 'filter',
  CUSTOM = 'custom',
}

// ============================================================================
// Universal Data Envelope (UDE)
// ============================================================================

/**
 * Source system information
 */
export interface SourceInfo {
  /** Source system identifier */
  system?: string;

  /** Source endpoint URL */
  endpoint?: string;

  /** Distributed tracing ID */
  trace?: string;

  /** Source timestamp */
  timestamp?: string;

  /** Source version */
  version?: string;
}

/**
 * Encryption metadata
 */
export interface EncryptionMeta {
  /** Encryption algorithm used */
  algorithm: EncryptionAlgorithm;

  /** Key identifier (not the key itself) */
  keyId: string;

  /** Initialization vector (base64) */
  iv?: string;

  /** Authentication tag (base64, for AEAD) */
  authTag?: string;
}

/**
 * Metadata describing the data envelope
 */
export interface EnvelopeMeta {
  /** Unique envelope identifier (UUID v4) */
  id: string;

  /** UDE format version (SemVer) */
  version: string;

  /** Schema identifier (URI) */
  schema: string;

  /** Schema version (SemVer) */
  schemaVersion: string;

  /** Creation timestamp (ISO 8601) */
  timestamp: string;

  /** Data format */
  format: DataFormat;

  /** Character encoding */
  encoding: Encoding;

  /** Compression algorithm */
  compression?: CompressionAlgorithm;

  /** Payload size in bytes (uncompressed) */
  size: number;

  /** Checksum in format "algorithm:hash" */
  checksum: string;

  /** Source system information */
  source?: SourceInfo;

  /** Encryption metadata */
  encryption?: EncryptionMeta;

  /** Custom extension fields */
  extensions?: Record<string, unknown>;
}

/**
 * Integrity verification information
 */
export interface IntegrityInfo {
  /** Hash algorithm */
  algorithm: HashAlgorithm;

  /** Cryptographic hash (hex) */
  hash: string;

  /** Digital signature (base64, optional) */
  signature?: string;

  /** Signer identifier (optional) */
  signerId?: string;

  /** Signature timestamp (ISO 8601, optional) */
  signedAt?: string;
}

/**
 * Universal Data Envelope structure
 */
export interface DataEnvelope<T = unknown> {
  /** Envelope metadata */
  meta: EnvelopeMeta;

  /** Data payload */
  data: T;

  /** Integrity verification */
  integrity: IntegrityInfo;
}

// ============================================================================
// Schema Management
// ============================================================================

/**
 * JSON Schema definition (subset)
 */
export interface SchemaDefinition {
  /** Schema draft version */
  $schema: string;

  /** Schema unique identifier */
  $id: string;

  /** Schema title */
  title?: string;

  /** Schema description */
  description?: string;

  /** Schema type */
  type: string;

  /** Schema properties */
  properties?: Record<string, unknown>;

  /** Required fields */
  required?: string[];

  /** Additional properties allowed */
  additionalProperties?: boolean;

  /** Schema version (SemVer) */
  version?: string;
}

/**
 * Schema registry entry
 */
export interface SchemaRegistryEntry {
  /** Schema identifier */
  id: string;

  /** Schema version */
  version: string;

  /** Schema definition */
  schema: SchemaDefinition;

  /** Creation timestamp */
  createdAt: string;

  /** Last update timestamp */
  updatedAt: string;

  /** Deprecation status */
  deprecated?: boolean;

  /** Successor version (if deprecated) */
  successor?: string;
}

/**
 * Schema compatibility result
 */
export interface SchemaCompatibility {
  /** Source schema version */
  sourceVersion: string;

  /** Target schema version */
  targetVersion: string;

  /** Compatibility score (0-1) */
  score: number;

  /** Is compatible (score >= threshold) */
  compatible: boolean;

  /** Matching fields */
  matchingFields: string[];

  /** Missing fields in target */
  missingFields: string[];

  /** Extra fields in target */
  extraFields: string[];

  /** Type conflicts */
  typeConflicts: Array<{
    field: string;
    sourceType: string;
    targetType: string;
  }>;
}

// ============================================================================
// Transformation Pipeline
// ============================================================================

/**
 * Transformation step configuration
 */
export interface TransformationStep {
  /** Step name */
  name: string;

  /** Step type */
  type: TransformationType;

  /** Schema for validation steps */
  schema?: string;

  /** Transform expressions for mapper steps */
  transform?: Record<string, string>;

  /** Fields to add for enricher steps */
  fields?: Record<string, unknown>;

  /** Filter expression */
  filter?: string;

  /** Custom transformation function */
  customFn?: (data: unknown) => unknown | Promise<unknown>;

  /** Continue on error */
  continueOnError?: boolean;
}

/**
 * Transformation pipeline configuration
 */
export interface TransformationPipelineConfig {
  /** Pipeline identifier */
  id: string;

  /** Pipeline version */
  version: string;

  /** Pipeline description */
  description?: string;

  /** Transformation steps */
  steps: TransformationStep[];

  /** Pipeline metadata */
  metadata?: Record<string, unknown>;
}

/**
 * Transformation result
 */
export interface TransformationResult<T = unknown> {
  /** Was transformation successful */
  success: boolean;

  /** Transformed data (if successful) */
  data?: T;

  /** Error message (if failed) */
  error?: string;

  /** Steps executed */
  stepsExecuted: number;

  /** Steps failed */
  stepsFailed: number;

  /** Execution time in milliseconds */
  executionTime: number;

  /** Warnings */
  warnings?: string[];
}

// ============================================================================
// Validation
// ============================================================================

/**
 * Validation error
 */
export interface ValidationError {
  /** Field path (JSON Pointer) */
  field: string;

  /** Error message */
  message: string;

  /** Error code */
  code: string;

  /** Expected value/type */
  expected?: unknown;

  /** Actual value/type */
  actual?: unknown;
}

/**
 * Validation result
 */
export interface ValidationResult {
  /** Is valid */
  valid: boolean;

  /** Validation errors */
  errors: ValidationError[];

  /** Validation warnings */
  warnings?: ValidationError[];

  /** Schema used for validation */
  schema?: string;
}

// ============================================================================
// Configuration
// ============================================================================

/**
 * Universal Data Exchange SDK configuration
 */
export interface UDEConfig {
  /** Default UDE version */
  defaultVersion?: string;

  /** Default data format */
  defaultFormat?: DataFormat;

  /** Default encoding */
  defaultEncoding?: Encoding;

  /** Default compression */
  defaultCompression?: CompressionAlgorithm;

  /** Default hash algorithm */
  defaultHashAlgorithm?: HashAlgorithm;

  /** Validate on envelope creation */
  validateOnCreate?: boolean;

  /** Validate on envelope reception */
  validateOnReceive?: boolean;

  /** Strict mode (reject invalid data) */
  strictMode?: boolean;

  /** Schema registry URL */
  schemaRegistryUrl?: string;

  /** Schema cache size (LRU) */
  schemaCacheSize?: number;

  /** Enable compression */
  enableCompression?: boolean;

  /** Compression threshold (bytes) */
  compressionThreshold?: number;

  /** Max envelope size (bytes) */
  maxEnvelopeSize?: number;

  /** Enable integrity verification */
  enableIntegrityCheck?: boolean;

  /** Enable digital signatures */
  enableSignatures?: boolean;

  /** Private key for signatures */
  privateKey?: string;

  /** Public key for signature verification */
  publicKey?: string;

  /** Custom extensions */
  extensions?: Record<string, unknown>;
}

// ============================================================================
// Protocol Bindings
// ============================================================================

/**
 * HTTP transport options
 */
export interface HttpTransportOptions {
  /** Base URL */
  baseUrl: string;

  /** HTTP headers */
  headers?: Record<string, string>;

  /** Request timeout (ms) */
  timeout?: number;

  /** Retry attempts */
  retries?: number;

  /** Enable compression */
  compression?: boolean;
}

/**
 * WebSocket transport options
 */
export interface WebSocketTransportOptions {
  /** WebSocket URL */
  url: string;

  /** Protocol */
  protocol?: string;

  /** Reconnect on disconnect */
  autoReconnect?: boolean;

  /** Reconnect interval (ms) */
  reconnectInterval?: number;

  /** Max reconnect attempts */
  maxReconnectAttempts?: number;
}

/**
 * Transport protocol types
 */
export enum TransportProtocol {
  HTTP = 'http',
  HTTPS = 'https',
  WEBSOCKET = 'websocket',
  GRPC = 'grpc',
  MQTT = 'mqtt',
  AMQP = 'amqp',
}

// ============================================================================
// Streaming
// ============================================================================

/**
 * Stream options
 */
export interface StreamOptions {
  /** Chunk size in bytes */
  chunkSize?: number;

  /** High water mark */
  highWaterMark?: number;

  /** Enable backpressure handling */
  backpressure?: boolean;
}

/**
 * Stream chunk
 */
export interface StreamChunk<T = unknown> {
  /** Chunk sequence number */
  sequence: number;

  /** Total chunks */
  total: number;

  /** Chunk data */
  data: T;

  /** Is last chunk */
  isLast: boolean;

  /** Chunk checksum */
  checksum?: string;
}

// ============================================================================
// Metrics and Monitoring
// ============================================================================

/**
 * Performance metrics
 */
export interface PerformanceMetrics {
  /** Total envelopes processed */
  totalEnvelopes: number;

  /** Total bytes processed */
  totalBytes: number;

  /** Average throughput (msg/sec) */
  avgThroughput: number;

  /** Average latency (ms) */
  avgLatency: number;

  /** P50 latency (ms) */
  p50Latency: number;

  /** P95 latency (ms) */
  p95Latency: number;

  /** P99 latency (ms) */
  p99Latency: number;

  /** Error rate (0-1) */
  errorRate: number;

  /** Compression ratio (0-1) */
  compressionRatio?: number;

  /** Cache hit rate (0-1) */
  cacheHitRate?: number;
}

/**
 * Error tracking
 */
export interface ErrorInfo {
  /** Error code */
  code: string;

  /** Error message */
  message: string;

  /** Error timestamp */
  timestamp: string;

  /** Stack trace */
  stack?: string;

  /** Additional context */
  context?: Record<string, unknown>;
}

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Async operation result
 */
export type Result<T, E = Error> =
  | { success: true; data: T }
  | { success: false; error: E };

/**
 * Deep partial type
 */
export type DeepPartial<T> = {
  [P in keyof T]?: T[P] extends object ? DeepPartial<T[P]> : T[P];
};

/**
 * Deep readonly type
 */
export type DeepReadonly<T> = {
  readonly [P in keyof T]: T[P] extends object ? DeepReadonly<T[P]> : T[P];
};

/**
 * Extract envelope data type
 */
export type EnvelopeData<E> = E extends DataEnvelope<infer T> ? T : never;

// ============================================================================
// Constants
// ============================================================================

/**
 * Default configuration values
 */
export const DEFAULT_CONFIG: Required<UDEConfig> = {
  defaultVersion: '1.0.0',
  defaultFormat: DataFormat.JSON,
  defaultEncoding: Encoding.UTF8,
  defaultCompression: CompressionAlgorithm.NONE,
  defaultHashAlgorithm: HashAlgorithm.SHA256,
  validateOnCreate: true,
  validateOnReceive: true,
  strictMode: false,
  schemaRegistryUrl: 'https://registry.wiastandards.com',
  schemaCacheSize: 1000,
  enableCompression: true,
  compressionThreshold: 1024, // 1KB
  maxEnvelopeSize: 100 * 1024 * 1024, // 100MB
  enableIntegrityCheck: true,
  enableSignatures: false,
  privateKey: '',
  publicKey: '',
  extensions: {},
};

/**
 * Maximum supported envelope size (100MB)
 */
export const MAX_ENVELOPE_SIZE = 100 * 1024 * 1024;

/**
 * Minimum compatibility score for automatic transformation
 */
export const MIN_COMPATIBILITY_SCORE = 0.7;

/**
 * Schema cache TTL in milliseconds (1 hour)
 */
export const SCHEMA_CACHE_TTL = 60 * 60 * 1000;
