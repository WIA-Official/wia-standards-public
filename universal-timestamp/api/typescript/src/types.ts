/**
 * WIA-CORE-009: Universal Timestamp - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Core Standards Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Precision level for timestamps
 */
export enum PrecisionLevel {
  /** Second precision (1s) */
  SECOND = 0,
  /** Millisecond precision (0.001s) */
  MILLISECOND = 1,
  /** Microsecond precision (0.000001s) */
  MICROSECOND = 2,
  /** Nanosecond precision (0.000000001s) */
  NANOSECOND = 3,
  /** Planck-level precision (5.391×10⁻⁴⁴s) */
  PLANCK = 4,
}

/**
 * Timezone representation
 */
export type Timezone =
  | 'UTC'
  | string // IANA timezone (e.g., "America/New_York")
  | `${'+' | '-'}${string}:${string}`; // UTC offset (e.g., "+09:00")

/**
 * Temporal context for time-travel scenarios
 */
export interface TemporalContext {
  /** Original timeline timestamp (epoch) */
  origin: number;

  /** Temporal displacement in seconds (negative for past) */
  displacement: number;

  /** Worldline identifier */
  worldline: string;

  /** Timeline branch identifier */
  timeline: string;

  /** Reference WIA-TIME standard */
  reference: string;

  /** Optional Planck-level precision */
  planckValue?: number;
}

/**
 * Cryptographic signature for timestamp
 */
export interface TimestampSignature {
  /** Signature algorithm */
  algorithm: 'ed25519' | 'rsa' | 'ecdsa' | 'dilithium' | 'sphincs+';

  /** Signature value (base64) */
  value: string;

  /** Hash algorithm */
  hashAlgorithm: 'sha256' | 'sha3' | 'blake3';

  /** Hash value (hex) */
  hashValue: string;

  /** Optional nonce for uniqueness */
  nonce?: string;

  /** Public key for verification (optional) */
  publicKey?: string;
}

/**
 * Universal Timestamp
 */
export interface UniversalTimestamp {
  /** Unix epoch (seconds since 1970-01-01T00:00:00Z) */
  epoch: number;

  /** Nanosecond precision (0-999999999) */
  precision: number;

  /** Precision level */
  precisionLevel: PrecisionLevel;

  /** Timezone */
  timezone: Timezone;

  /** Temporal context (optional, for time-travel) */
  temporalContext?: TemporalContext;

  /** Cryptographic signature (optional) */
  signature?: TimestampSignature;

  /** WIA format string */
  wiaFormat: string;

  /** ISO 8601 format */
  iso8601: string;

  /** RFC 3339 format */
  rfc3339: string;

  /** JavaScript Date object */
  date: Date;

  /** Unix timestamp with milliseconds */
  milliseconds: number;

  /** Unix timestamp with microseconds */
  microseconds: number;

  /** Unix timestamp with nanoseconds */
  nanoseconds: number;
}

// ============================================================================
// Creation Options
// ============================================================================

/**
 * Options for creating a timestamp
 */
export interface CreateTimestampOptions {
  /** Date to create timestamp from (default: now) */
  date?: Date;

  /** Precision level (default: NANOSECOND) */
  precision?: PrecisionLevel;

  /** Timezone (default: UTC) */
  timezone?: Timezone;

  /** Temporal context for time-travel */
  temporalContext?: TemporalContext;

  /** Sign the timestamp */
  sign?: boolean;

  /** Private key for signing */
  privateKey?: CryptoKey | string;

  /** Signature algorithm */
  signatureAlgorithm?: TimestampSignature['algorithm'];

  /** Hash algorithm */
  hashAlgorithm?: TimestampSignature['hashAlgorithm'];

  /** Nonce for signature uniqueness */
  nonce?: string;
}

// ============================================================================
// Parsing & Validation
// ============================================================================

/**
 * Result of timestamp parsing
 */
export interface ParseResult {
  /** Was parsing successful? */
  success: boolean;

  /** Parsed timestamp (if successful) */
  timestamp?: UniversalTimestamp;

  /** Error message (if failed) */
  error?: string;

  /** Warnings (non-blocking) */
  warnings?: string[];
}

/**
 * Validation result
 */
export interface ValidationResult {
  /** Is timestamp valid? */
  isValid: boolean;

  /** Validation errors (blocking) */
  errors: string[];

  /** Validation warnings (non-blocking) */
  warnings: string[];

  /** Detailed checks */
  checks: ValidationCheck[];
}

/**
 * Individual validation check
 */
export interface ValidationCheck {
  /** Check name */
  name: string;

  /** Check status */
  status: 'pass' | 'fail' | 'warning';

  /** Check message */
  message: string;

  /** Expected value */
  expected?: unknown;

  /** Actual value */
  actual?: unknown;
}

// ============================================================================
// Conversion & Formatting
// ============================================================================

/**
 * Output format options
 */
export type OutputFormat =
  | 'wia' // WIA-TS format (canonical)
  | 'iso' // ISO 8601
  | 'rfc3339' // RFC 3339
  | 'unix' // Unix epoch
  | 'unix-ms' // Unix epoch with milliseconds
  | 'unix-us' // Unix epoch with microseconds
  | 'unix-ns' // Unix epoch with nanoseconds
  | 'human' // Human-readable
  | 'json' // JSON object
  | 'binary'; // Binary encoding

/**
 * Timezone conversion options
 */
export interface ConversionOptions {
  /** Target timezone */
  targetTimezone: Timezone;

  /** Preserve precision level */
  preservePrecision?: boolean;

  /** Preserve temporal context */
  preserveTemporalContext?: boolean;

  /** Preserve signature */
  preserveSignature?: boolean;
}

/**
 * Human-readable format options
 */
export interface HumanFormatOptions {
  /** Include date */
  includeDate?: boolean;

  /** Include time */
  includeTime?: boolean;

  /** Include timezone */
  includeTimezone?: boolean;

  /** Include precision */
  includePrecision?: boolean;

  /** Locale for formatting */
  locale?: string;

  /** Date style */
  dateStyle?: 'full' | 'long' | 'medium' | 'short';

  /** Time style */
  timeStyle?: 'full' | 'long' | 'medium' | 'short';
}

// ============================================================================
// Comparison & Operations
// ============================================================================

/**
 * Comparison result
 */
export type ComparisonResult = -1 | 0 | 1;

/**
 * Timestamp difference
 */
export interface TimestampDifference {
  /** Difference in seconds */
  seconds: number;

  /** Difference in milliseconds */
  milliseconds: number;

  /** Difference in microseconds */
  microseconds: number;

  /** Difference in nanoseconds */
  nanoseconds: number;

  /** Human-readable duration */
  human: string;

  /** Absolute difference */
  absolute: TimestampDifference;
}

/**
 * Duration components
 */
export interface Duration {
  /** Years */
  years?: number;

  /** Months */
  months?: number;

  /** Days */
  days?: number;

  /** Hours */
  hours?: number;

  /** Minutes */
  minutes?: number;

  /** Seconds */
  seconds?: number;

  /** Milliseconds */
  milliseconds?: number;

  /** Microseconds */
  microseconds?: number;

  /** Nanoseconds */
  nanoseconds?: number;
}

// ============================================================================
// Serialization
// ============================================================================

/**
 * JSON serialization format
 */
export interface TimestampJSON {
  /** Format identifier */
  format: 'WIA-TS';

  /** Version */
  version: '1.0';

  /** Epoch */
  epoch: number;

  /** Precision */
  precision: number;

  /** Precision level */
  precisionLevel: PrecisionLevel;

  /** Timezone */
  timezone: Timezone;

  /** Temporal context */
  temporalContext?: TemporalContext;

  /** Signature */
  signature?: TimestampSignature;
}

/**
 * Binary encoding options
 */
export interface BinaryEncodingOptions {
  /** Include temporal context */
  includeTemporalContext?: boolean;

  /** Include signature */
  includeSignature?: boolean;

  /** Compression */
  compress?: boolean;
}

/**
 * URL-safe encoding options
 */
export interface URLSafeOptions {
  /** Base format */
  baseFormat?: 'base64' | 'base64url' | 'hex';

  /** Include temporal context */
  includeTemporalContext?: boolean;

  /** Include signature */
  includeSignature?: boolean;
}

// ============================================================================
// Cryptography
// ============================================================================

/**
 * Signing options
 */
export interface SigningOptions {
  /** Private key */
  privateKey: CryptoKey | string;

  /** Algorithm */
  algorithm: TimestampSignature['algorithm'];

  /** Hash algorithm */
  hashAlgorithm: TimestampSignature['hashAlgorithm'];

  /** Nonce */
  nonce?: string;
}

/**
 * Verification options
 */
export interface VerificationOptions {
  /** Public key */
  publicKey: CryptoKey | string;

  /** Expected algorithm */
  expectedAlgorithm?: TimestampSignature['algorithm'];

  /** Verify hash */
  verifyHash?: boolean;

  /** Strict mode (fail on warnings) */
  strict?: boolean;
}

/**
 * Verification result
 */
export interface VerificationResult {
  /** Is signature valid? */
  isValid: boolean;

  /** Signer public key */
  signer?: string;

  /** Signature algorithm */
  algorithm?: string;

  /** Verification timestamp */
  verifiedAt: Date;

  /** Errors */
  errors: string[];

  /** Warnings */
  warnings: string[];
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Physical and computational constants
 */
export const TIMESTAMP_CONSTANTS = {
  /** Unix epoch start */
  UNIX_EPOCH: new Date('1970-01-01T00:00:00Z'),

  /** Minimum valid epoch (1900-01-01) */
  MIN_EPOCH: -2208988800,

  /** Maximum valid epoch (2100-01-01) */
  MAX_EPOCH: 4102444800,

  /** Nanoseconds per second */
  NANOS_PER_SECOND: 1_000_000_000,

  /** Microseconds per second */
  MICROS_PER_SECOND: 1_000_000,

  /** Milliseconds per second */
  MILLIS_PER_SECOND: 1_000,

  /** Planck time in seconds */
  PLANCK_TIME: 5.391247e-44,

  /** Speed of light in m/s */
  SPEED_OF_LIGHT: 299_792_458,

  /** Supported precision digits */
  PRECISION_DIGITS: {
    [PrecisionLevel.SECOND]: 0,
    [PrecisionLevel.MILLISECOND]: 3,
    [PrecisionLevel.MICROSECOND]: 6,
    [PrecisionLevel.NANOSECOND]: 9,
    [PrecisionLevel.PLANCK]: -1, // variable
  },

  /** WIA-TS format prefix */
  FORMAT_PREFIX: 'WIA-TS:',

  /** Version */
  VERSION: '1.0',
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-CORE-009 error codes
 */
export enum TimestampErrorCode {
  INVALID_FORMAT = 'TS001',
  INVALID_EPOCH = 'TS002',
  INVALID_PRECISION = 'TS003',
  INVALID_TIMEZONE = 'TS004',
  INVALID_TEMPORAL_CONTEXT = 'TS005',
  INVALID_SIGNATURE = 'TS006',
  PARSE_ERROR = 'TS007',
  VALIDATION_ERROR = 'TS008',
  CONVERSION_ERROR = 'TS009',
  SIGNATURE_ERROR = 'TS010',
  UNSUPPORTED_OPERATION = 'TS011',
}

/**
 * Timestamp error
 */
export class TimestampError extends Error {
  constructor(
    public code: TimestampErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'TimestampError';
  }
}

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Result type for operations that can fail
 */
export type Result<T, E = Error> =
  | { success: true; data: T }
  | { success: false; error: E };

/**
 * Async result type
 */
export type AsyncResult<T, E = Error> = Promise<Result<T, E>>;

/**
 * Partial timestamp (for parsing incomplete timestamps)
 */
export interface PartialTimestamp {
  epoch?: number;
  precision?: number;
  precisionLevel?: PrecisionLevel;
  timezone?: Timezone;
  temporalContext?: Partial<TemporalContext>;
  signature?: Partial<TimestampSignature>;
}

/**
 * Timestamp range
 */
export interface TimestampRange {
  /** Start timestamp */
  start: UniversalTimestamp;

  /** End timestamp */
  end: UniversalTimestamp;

  /** Duration */
  duration: TimestampDifference;
}

/**
 * Timestamp iterator options
 */
export interface IteratorOptions {
  /** Start timestamp */
  start: UniversalTimestamp;

  /** End timestamp */
  end: UniversalTimestamp;

  /** Step size */
  step: Duration;

  /** Include end */
  includeEnd?: boolean;
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core
  UniversalTimestamp,
  Timezone,
  TemporalContext,
  TimestampSignature,

  // Creation
  CreateTimestampOptions,

  // Parsing & Validation
  ParseResult,
  ValidationResult,
  ValidationCheck,

  // Conversion
  OutputFormat,
  ConversionOptions,
  HumanFormatOptions,

  // Comparison
  ComparisonResult,
  TimestampDifference,
  Duration,

  // Serialization
  TimestampJSON,
  BinaryEncodingOptions,
  URLSafeOptions,

  // Cryptography
  SigningOptions,
  VerificationOptions,
  VerificationResult,

  // Utility
  PartialTimestamp,
  TimestampRange,
  IteratorOptions,
};

export { PrecisionLevel, TIMESTAMP_CONSTANTS, TimestampErrorCode, TimestampError };
