/**
 * WIA-TIME-014: Data Time Transport - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Time Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Data Types
// ============================================================================

/**
 * UUID v7 (timestamp-ordered)
 */
export type UUID = string;

/**
 * ISO 8601 timestamp with nanosecond precision
 */
export type Timestamp = string | Date;

/**
 * Timeline identifier
 */
export type TimelineID = string; // Format: TL-01-PRIME-ALPHA-0000000001

/**
 * Hex-encoded hash
 */
export type HashHex = string;

/**
 * Base64-encoded data
 */
export type Base64 = string;

// ============================================================================
// Temporal Data Packet
// ============================================================================

/**
 * Temporal Data Packet flags
 */
export interface TDPFlags {
  /** Is data compressed? */
  compressed: boolean;

  /** Is data encrypted? */
  encrypted: boolean;

  /** Requires acknowledgment? */
  requiresAck: boolean;

  /** High priority? */
  highPriority: boolean;

  /** Contains quantum entangled data? */
  quantumEntangled: boolean;

  /** Includes error correction? */
  errorCorrection: boolean;
}

/**
 * Temporal Data Packet header
 */
export interface TDPHeader {
  /** Magic number (0x54445054 - "TDPT") */
  magicNumber: number;

  /** Protocol version */
  version: {
    major: number;
    minor: number;
    patch: number;
  };

  /** Control flags */
  flags: TDPFlags;

  /** Origin timestamp (nanoseconds) */
  originTime: bigint;

  /** Destination timestamp (nanoseconds) */
  destinationTime: bigint;

  /** Origin timeline ID */
  originTimeline: TimelineID;

  /** Destination timeline ID */
  destinationTimeline: TimelineID;

  /** Payload size in bytes */
  payloadSize: bigint;

  /** Compression algorithm */
  compression: CompressionAlgorithm;
}

/**
 * Compression algorithms
 */
export enum CompressionAlgorithm {
  NONE = 0,
  LZ4 = 1,
  ZSTD = 2,
  BROTLI = 3,
  LZMA2 = 4,
  CUSTOM = 255,
}

/**
 * 3D spatial coordinates
 */
export interface Coordinates {
  x: number; // meters
  y: number; // meters
  z: number; // meters
}

/**
 * Temporal Data Packet metadata
 */
export interface TDPMetadata {
  /** Unique packet ID */
  id: UUID;

  /** Creation timestamp */
  created: Timestamp;

  /** Origin information */
  origin: {
    time: Timestamp;
    timeline: TimelineID;
    coordinates: Coordinates;
    referenceFrame: string;
  };

  /** Destination information */
  destination: {
    time: Timestamp;
    timeline: TimelineID;
    coordinates: Coordinates;
    referenceFrame: string;
  };

  /** Content information */
  content: {
    type: string; // MIME type
    encoding: string;
    compression: string;
    encryption: string | null;
  };

  /** Integrity information */
  integrity: {
    algorithm: string;
    hash: HashHex;
    errorCorrection: string;
  };

  /** Delivery priority */
  priority: Priority;

  /** Time-to-live in seconds */
  ttl: number;

  /** Redundancy level (1-10) */
  redundancy: number;
}

/**
 * Message priority levels
 */
export enum Priority {
  CRITICAL = 0,
  HIGH = 1,
  NORMAL = 2,
  LOW = 3,
  BACKGROUND = 4,
}

/**
 * Temporal signature
 */
export interface TemporalSignature {
  /** Cryptographic signature */
  signature: Base64;

  /** Timestamp of signing */
  timestamp: Timestamp;

  /** Quantum entropy used */
  entropy: Base64;

  /** Proof-of-work nonce */
  powNonce: bigint;

  /** Public key */
  publicKey: Base64;

  /** Signature algorithm */
  algorithm: 'Ed25519' | 'ECDSA' | 'RSA' | 'Dilithium';
}

/**
 * Complete Temporal Data Packet
 */
export interface TemporalDataPacket {
  /** Packet header */
  header: TDPHeader;

  /** Packet metadata */
  metadata: TDPMetadata;

  /** Payload data */
  payload: Buffer;

  /** Checksum (SHA-512) */
  checksum: HashHex;

  /** Temporal signature */
  signature: TemporalSignature;
}

// ============================================================================
// Time Capsule
// ============================================================================

/**
 * Time capsule states
 */
export enum CapsuleState {
  CREATED = 'created',
  SEALED = 'sealed',
  IN_TRANSIT = 'in_transit',
  ANCHORED = 'anchored',
  READY = 'ready',
  OPENED = 'opened',
  ARCHIVED = 'archived',
}

/**
 * Access control policy
 */
export interface AccessControl {
  /** Owner ID */
  owner: string;

  /** Authorized users */
  authorized: string[];

  /** Required signatures (threshold) */
  threshold: number;

  /** Public access allowed? */
  publicAccess: boolean;

  /** Access conditions */
  conditions?: {
    timeRange?: [Timestamp, Timestamp];
    timeline?: TimelineID;
    location?: Coordinates;
  };
}

/**
 * Time-locked encryption configuration
 */
export interface TimeLockConfig {
  /** Encryption type */
  type: 'time-lock-puzzle' | 'witness-encryption' | 'blockchain' | 'quantum';

  /** Encryption algorithm */
  algorithm: 'AES-256-GCM' | 'ChaCha20-Poly1305';

  /** Unlock time */
  unlockTime: Timestamp;

  /** Time-lock specific parameters */
  parameters: Record<string, unknown>;

  /** Backup key custodians */
  backupKeys?: {
    custodian: string;
    encryptedShare: Base64;
  }[];

  /** Threshold for backup keys */
  backupThreshold?: number;
}

/**
 * Time capsule configuration
 */
export interface TimeCapsule {
  /** Unique capsule ID */
  id: UUID;

  /** Capsule version */
  version: string;

  /** Creation time */
  created: Timestamp;

  /** Scheduled delivery time */
  deliveryTime: Timestamp;

  /** Target timeline */
  timeline: TimelineID;

  /** Current state */
  state: CapsuleState;

  /** Access control policy */
  accessControl: AccessControl;

  /** Encryption configuration */
  encryption: TimeLockConfig | null;

  /** Data packets */
  data: TemporalDataPacket[];

  /** Redundancy level (1-10) */
  redundancy: number;

  /** Temporal anchor coordinates */
  anchor?: {
    time: Timestamp;
    coordinates: Coordinates;
    stability: number; // 0-1
  };

  /** Metadata */
  metadata: {
    title?: string;
    description?: string;
    tags?: string[];
    creator?: string;
    size: number; // bytes
  };

  /** Integrity information */
  integrity: {
    hashes: {
      primary: HashHex; // SHA-512
      secondary: HashHex; // BLAKE3
      quantumResistant: HashHex; // SPHINCS+
    };
    lastVerified: Timestamp;
    verificationCount: number;
  };
}

/**
 * Parameters for creating time capsule
 */
export interface TimeCapsuleParams {
  /** Data to store */
  data: unknown;

  /** Delivery time */
  deliveryTime: Timestamp;

  /** Target timeline (default: current) */
  timeline?: TimelineID;

  /** Encryption configuration */
  encryption?: TimeLockConfig;

  /** Access control */
  accessControl?: Partial<AccessControl>;

  /** Redundancy level (1-10, default: 3) */
  redundancy?: number;

  /** Metadata */
  metadata?: {
    title?: string;
    description?: string;
    tags?: string[];
  };

  /** Compression algorithm */
  compression?: CompressionAlgorithm;
}

/**
 * Time capsule filter for listing
 */
export interface CapsuleFilter {
  /** Filter by state */
  state?: CapsuleState | CapsuleState[];

  /** Filter by time range */
  timeRange?: [Timestamp, Timestamp];

  /** Filter by timeline */
  timeline?: TimelineID;

  /** Filter by owner */
  owner?: string;

  /** Filter by tags */
  tags?: string[];

  /** Limit results */
  limit?: number;

  /** Offset for pagination */
  offset?: number;
}

/**
 * Extracted capsule data
 */
export interface CapsuleData {
  /** Capsule ID */
  capsuleId: UUID;

  /** Extracted data */
  data: unknown;

  /** Metadata */
  metadata: TimeCapsule['metadata'];

  /** Extraction time */
  extractedAt: Timestamp;

  /** Integrity status */
  integrity: {
    valid: boolean;
    score: number; // 0-1
    issues: string[];
  };
}

// ============================================================================
// Cross-Timeline Messaging
// ============================================================================

/**
 * Timeline types
 */
export enum TimelineType {
  PRIME = 'PRIME',
  BRANCH = 'BRANCH',
  ALTERNATE = 'ALTERNATE',
  PARALLEL = 'PARALLEL',
}

/**
 * Timeline information
 */
export interface TimelineInfo {
  /** Timeline ID */
  id: TimelineID;

  /** Timeline type */
  type: TimelineType;

  /** Branch identifier */
  branch: string;

  /** Sequence number */
  sequence: number;

  /** Parent timeline */
  parent?: TimelineID;

  /** Branch point time */
  branchPoint?: Timestamp;

  /** Timeline name/description */
  name?: string;

  /** Current time in timeline */
  currentTime: Timestamp;

  /** Timeline status */
  status: 'active' | 'merged' | 'abandoned' | 'unknown';

  /** Synchronization status */
  sync: {
    lastSync: Timestamp;
    offset: number; // milliseconds
    drift: number; // ms/day
  };
}

/**
 * Message delivery modes
 */
export enum DeliveryMode {
  BEST_EFFORT = 'best_effort',
  AT_LEAST_ONCE = 'at_least_once',
  EXACTLY_ONCE = 'exactly_once',
  IN_ORDER = 'in_order',
  ATOMIC = 'atomic',
}

/**
 * Message parameters
 */
export interface MessageParams {
  /** Message content */
  content: unknown;

  /** Target timeline */
  targetTimeline: TimelineID;

  /** Target time */
  targetTime: Timestamp;

  /** Target recipient */
  recipient?: string;

  /** Message priority */
  priority?: Priority;

  /** Delivery mode */
  deliveryMode?: DeliveryMode;

  /** Maximum delivery attempts */
  maxAttempts?: number;

  /** Delivery timeout in seconds */
  timeout?: number;

  /** Require quantum verification? */
  quantumVerification?: boolean;
}

/**
 * Message receipt
 */
export interface MessageReceipt {
  /** Message ID */
  messageId: UUID;

  /** Delivery status */
  status: 'pending' | 'sent' | 'delivered' | 'failed';

  /** Sent timestamp */
  sentAt: Timestamp;

  /** Delivered timestamp */
  deliveredAt?: Timestamp;

  /** Receipt signature */
  signature: TemporalSignature;

  /** Chain of custody */
  custody: {
    node: string;
    timestamp: Timestamp;
  }[];

  /** Error message if failed */
  error?: string;
}

/**
 * Message
 */
export interface Message {
  /** Message ID */
  id: UUID;

  /** Protocol version */
  protocol: string; // "TMP/1.0"

  /** Sender information */
  from: {
    timeline: TimelineID;
    time: Timestamp;
    sender: string;
  };

  /** Recipient information */
  to: {
    timeline: TimelineID;
    time: Timestamp;
    recipient: string;
  };

  /** Message content */
  content: {
    type: string; // MIME type
    data: Base64;
    size: number;
  };

  /** Priority */
  priority: Priority;

  /** Delivery configuration */
  delivery: {
    mode: DeliveryMode;
    attempts: number;
    timeout: number;
  };

  /** Verification */
  verification: {
    signature: Base64;
    algorithm: string;
    quantumEntangled: boolean;
  };

  /** Receipt timestamp */
  receivedAt?: Timestamp;
}

/**
 * Message filter for receiving
 */
export interface MessageFilter {
  /** Filter by sender */
  sender?: string;

  /** Filter by timeline */
  timeline?: TimelineID;

  /** Filter by time range */
  timeRange?: [Timestamp, Timestamp];

  /** Filter by priority */
  priority?: Priority | Priority[];

  /** Include read messages? */
  includeRead?: boolean;

  /** Limit results */
  limit?: number;

  /** Offset for pagination */
  offset?: number;
}

// ============================================================================
// Data Integrity
// ============================================================================

/**
 * Integrity check result
 */
export interface IntegrityReport {
  /** Overall valid? */
  valid: boolean;

  /** Integrity score (0-1) */
  score: number;

  /** Hash verifications */
  hashes: {
    primary: { algorithm: string; valid: boolean; expected: HashHex; actual: HashHex };
    secondary: { algorithm: string; valid: boolean; expected: HashHex; actual: HashHex };
    quantumResistant?: { algorithm: string; valid: boolean; expected: HashHex; actual: HashHex };
  };

  /** Signature verification */
  signature: {
    valid: boolean;
    algorithm: string;
    publicKey: Base64;
  };

  /** Temporal signature verification */
  temporalSignature: {
    valid: boolean;
    timestamp: Timestamp;
    powVerified: boolean;
  };

  /** Error correction status */
  errorCorrection?: {
    errors: number;
    corrected: number;
    correctable: boolean;
  };

  /** Issues found */
  issues: {
    severity: 'info' | 'warning' | 'error' | 'critical';
    message: string;
  }[];

  /** Recommendations */
  recommendations: string[];

  /** Check timestamp */
  checkedAt: Timestamp;
}

/**
 * Encoding options
 */
export interface EncodeOptions {
  /** Compression algorithm */
  compression?: CompressionAlgorithm;

  /** Compression level (1-9) */
  compressionLevel?: number;

  /** Apply encryption? */
  encryption?: {
    algorithm: 'AES-256-GCM' | 'ChaCha20-Poly1305';
    key: Buffer;
  };

  /** Add error correction? */
  errorCorrection?: boolean;

  /** Target timeline */
  targetTimeline?: TimelineID;

  /** Target time */
  targetTime?: Timestamp;

  /** Priority */
  priority?: Priority;
}

// ============================================================================
// Bandwidth Management
// ============================================================================

/**
 * Bandwidth status
 */
export interface BandwidthStatus {
  /** Current utilization (0-1) */
  utilization: number;

  /** Available bandwidth (bytes/s²) */
  available: number;

  /** Total capacity (bytes/s²) */
  capacity: number;

  /** Current queue size (bytes) */
  queueSize: number;

  /** Estimated delay (seconds) */
  delay: number;

  /** Per-priority allocation */
  allocation: {
    critical: number;
    high: number;
    normal: number;
    low: number;
    background: number;
  };

  /** Current congestion level */
  congestion: 'none' | 'low' | 'medium' | 'high' | 'severe';
}

/**
 * Bandwidth reservation
 */
export interface Reservation {
  /** Reservation ID */
  id: UUID;

  /** Reserved bandwidth (bytes/s²) */
  bandwidth: number;

  /** Reservation duration (seconds) */
  duration: number;

  /** Start time */
  startTime: Timestamp;

  /** End time */
  endTime: Timestamp;

  /** Reservation status */
  status: 'active' | 'expired' | 'cancelled';
}

// ============================================================================
// Timeline Operations
// ============================================================================

/**
 * Timeline synchronization result
 */
export interface SyncResult {
  /** Timeline ID */
  timeline: TimelineID;

  /** Sync successful? */
  success: boolean;

  /** Time offset (milliseconds) */
  offset: number;

  /** Round-trip delay (milliseconds) */
  delay: number;

  /** Clock drift (ms/day) */
  drift: number;

  /** Sync timestamp */
  syncedAt: Timestamp;

  /** Next recommended sync */
  nextSync: Timestamp;

  /** Error if failed */
  error?: string;
}

// ============================================================================
// Credentials and Authentication
// ============================================================================

/**
 * Authentication credentials
 */
export interface Credentials {
  /** User/entity ID */
  id: string;

  /** Authentication type */
  type: 'key' | 'signature' | 'token' | 'biometric' | 'multi-factor';

  /** Credential data */
  data: {
    privateKey?: Buffer;
    token?: string;
    signature?: Base64;
    factors?: unknown[];
  };

  /** Expiration time */
  expiresAt?: Timestamp;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * Error codes
 */
export enum DataTimeTransportErrorCode {
  // Time Capsule Errors
  CAPSULE_NOT_FOUND = 'DT001',
  CAPSULE_LOCKED = 'DT002',
  CAPSULE_CORRUPTED = 'DT003',
  CAPSULE_EXPIRED = 'DT004',

  // Message Errors
  MESSAGE_TOO_LARGE = 'DT010',
  MESSAGE_SEND_FAILED = 'DT011',
  MESSAGE_DELIVERY_FAILED = 'DT012',
  INVALID_TIMELINE = 'DT013',

  // Encoding Errors
  ENCODING_FAILED = 'DT020',
  DECODING_FAILED = 'DT021',
  COMPRESSION_ERROR = 'DT022',

  // Integrity Errors
  INTEGRITY_CHECK_FAILED = 'DT030',
  SIGNATURE_INVALID = 'DT031',
  HASH_MISMATCH = 'DT032',
  TEMPORAL_SIGNATURE_INVALID = 'DT033',

  // Bandwidth Errors
  BANDWIDTH_EXCEEDED = 'DT040',
  BANDWIDTH_UNAVAILABLE = 'DT041',
  RESERVATION_FAILED = 'DT042',

  // Timeline Errors
  TIMELINE_NOT_FOUND = 'DT050',
  TIMELINE_SYNC_FAILED = 'DT051',
  TIMELINE_DIVERGENCE = 'DT052',

  // Encryption Errors
  ENCRYPTION_FAILED = 'DT060',
  DECRYPTION_FAILED = 'DT061',
  KEY_NOT_FOUND = 'DT062',
  TIME_LOCK_ACTIVE = 'DT063',

  // General Errors
  INVALID_PARAMETERS = 'DT100',
  INSUFFICIENT_ENERGY = 'DT101',
  NETWORK_ERROR = 'DT102',
  TIMEOUT = 'DT103',
}

/**
 * Data Time Transport error
 */
export class DataTimeTransportError extends Error {
  constructor(
    public code: DataTimeTransportErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'DataTimeTransportError';
  }
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Protocol constants
 */
export const CONSTANTS = {
  /** Protocol version */
  PROTOCOL_VERSION: '1.0.0',

  /** Magic number for TDP */
  TDP_MAGIC: 0x54445054, // "TDPT"

  /** Maximum packet size (1 PB) */
  MAX_PACKET_SIZE: 1_000_000_000_000_000, // 10^15 bytes

  /** Maximum temporal range (±1000 years) */
  MAX_TEMPORAL_RANGE: 1000 * 365.25 * 24 * 3600, // seconds

  /** Default redundancy level */
  DEFAULT_REDUNDANCY: 3,

  /** Minimum integrity score */
  MIN_INTEGRITY_SCORE: 0.95,

  /** Default TTL (1 year) */
  DEFAULT_TTL: 365.25 * 24 * 3600, // seconds

  /** Bandwidth limits */
  BANDWIDTH: {
    PER_SENDER_DAILY: 1_000_000_000, // 1 GB
    PER_TIMELINE_DAILY: 1_000_000_000_000, // 1 TB
    GLOBAL_DAILY: 100_000_000_000_000, // 100 TB
  },

  /** Retry configuration */
  RETRY: {
    BASE_DELAY: 1.0, // seconds
    MAX_DELAY: 3600, // 1 hour
    MAX_ATTEMPTS: 10,
  },

  /** Temporal sync interval */
  SYNC_INTERVAL: 60, // seconds
} as const;

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Result type for operations that can fail
 */
export type Result<T, E = DataTimeTransportError> =
  | { success: true; data: T }
  | { success: false; error: E };

/**
 * Async result type
 */
export type AsyncResult<T, E = DataTimeTransportError> = Promise<Result<T, E>>;

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core types
  UUID,
  Timestamp,
  TimelineID,
  HashHex,
  Base64,
  Coordinates,

  // TDP
  TDPFlags,
  TDPHeader,
  TDPMetadata,
  TemporalDataPacket,
  TemporalSignature,

  // Time Capsule
  TimeCapsule,
  TimeCapsuleParams,
  CapsuleFilter,
  CapsuleData,
  AccessControl,
  TimeLockConfig,

  // Messaging
  Message,
  MessageParams,
  MessageReceipt,
  MessageFilter,
  TimelineInfo,

  // Integrity
  IntegrityReport,
  EncodeOptions,

  // Bandwidth
  BandwidthStatus,
  Reservation,

  // Timeline
  SyncResult,

  // Auth
  Credentials,
};

export {
  // Enums
  CompressionAlgorithm,
  Priority,
  CapsuleState,
  TimelineType,
  DeliveryMode,
  DataTimeTransportErrorCode,

  // Classes
  DataTimeTransportError,

  // Constants
  CONSTANTS,
};
