/**
 * WIA-TIME-011: Historical Integrity - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Time Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Three-dimensional vector for spatial coordinates
 */
export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

/**
 * Event category determining protection level
 */
export enum EventCategory {
  CRITICAL = 'critical',      // Immutable, highest importance
  PROTECTED = 'protected',    // Requires consensus for changes
  STANDARD = 'standard',      // Normal verification
  TRIVIAL = 'trivial',        // Minimal verification
}

/**
 * Types of evidence supporting historical events
 */
export enum EvidenceType {
  PHYSICAL_ARTIFACT = 'physical',
  DOCUMENT = 'document',
  WITNESS_TESTIMONY = 'witness',
  SENSOR_DATA = 'sensor',
  PHOTOGRAPHIC = 'photo',
  VIDEO_RECORDING = 'video',
  AUDIO_RECORDING = 'audio',
  DIGITAL_TRAIL = 'digital',
  BLOCKCHAIN_RECORD = 'blockchain',
}

// ============================================================================
// Historical Events
// ============================================================================

/**
 * Evidence supporting a historical event
 */
export interface Evidence {
  /** Unique evidence identifier */
  id: string;

  /** Type of evidence */
  type: EvidenceType;

  /** Evidence source/origin */
  source: string;

  /** When evidence was created/collected */
  timestamp: Date;

  /** Evidence content (type-dependent) */
  content: any;

  /** Cryptographic hash of evidence */
  hash: string;

  /** Digital signatures verifying authenticity */
  signatures: Signature[];

  /** Reliability score (0-1) */
  reliability?: number;

  /** Additional metadata */
  metadata: Record<string, any>;
}

/**
 * Digital signature for authentication
 */
export interface Signature {
  /** Signer identifier */
  signer: string;

  /** Signature algorithm */
  algorithm: 'SHA3-256' | 'CRYSTALS-Dilithium' | 'EdDSA' | 'RSA' | 'ECDSA';

  /** Signature value (hex string) */
  value: string;

  /** When signature was created */
  timestamp: Date;

  /** Public key or certificate */
  publicKey?: string;

  /** Is signature valid? */
  isValid?: boolean;
}

/**
 * A historical event in spacetime
 */
export interface HistoricalEvent {
  /** Unique event identifier */
  id: string;

  /** Event occurrence time */
  timestamp: Date;

  /** Spatial location in meters */
  location: Vector3;

  /** Event description */
  description: string;

  /** Entities involved in event */
  participants: string[];

  /** Supporting evidence */
  evidence: Evidence[];

  /** Event category/importance */
  category: EventCategory;

  /** Event hash for verification */
  hash?: string;

  /** Causal dependencies (event IDs) */
  causes?: string[];

  /** Causal effects (event IDs) */
  effects?: string[];

  /** Can this event be modified? */
  mutable: boolean;

  /** Additional metadata */
  metadata: Record<string, any>;
}

// ============================================================================
// Verification
// ============================================================================

/**
 * Event verification request parameters
 */
export interface EventVerificationParams {
  /** Event to verify */
  event: HistoricalEvent;

  /** Expected hash (if known) */
  expectedHash?: string;

  /** Verify evidence chain? */
  verifyEvidence?: boolean;

  /** Check temporal consistency? */
  checkConsistency?: boolean;

  /** Required confidence threshold (0-1) */
  minConfidence?: number;
}

/**
 * Status of signature verification
 */
export interface SignatureStatus {
  /** Signature being verified */
  signature: Signature;

  /** Is signature valid? */
  isValid: boolean;

  /** Verification timestamp */
  verifiedAt: Date;

  /** Error message if invalid */
  error?: string;
}

/**
 * Result of event verification
 */
export interface VerificationResult {
  /** Is event verified? */
  isValid: boolean;

  /** Confidence score (0-1) */
  confidence: number;

  /** Does hash match expected? */
  hashMatch: boolean;

  /** Computed event hash */
  computedHash: string;

  /** Is evidence valid? */
  evidenceValid: boolean;

  /** Is temporally consistent? */
  temporalConsistency: boolean;

  /** Signature verification results */
  signatures: SignatureStatus[];

  /** Non-blocking warnings */
  warnings: string[];

  /** Blocking errors */
  errors: string[];

  /** Verification timestamp */
  timestamp: Date;

  /** Additional verification details */
  details?: Record<string, any>;
}

// ============================================================================
// Timeline
// ============================================================================

/**
 * A timeline containing historical events
 */
export interface Timeline {
  /** Unique timeline identifier */
  id: string;

  /** Timeline name/description */
  name: string;

  /** Events in this timeline */
  events: HistoricalEvent[];

  /** Checkpoints for this timeline */
  checkpoints: Checkpoint[];

  /** Parent timeline (if branched) */
  parentTimeline?: string;

  /** Branch point from parent */
  branchPoint?: Date;

  /** Integrity score (0-1) */
  integrity: number;

  /** Current timeline fingerprint */
  fingerprint: string;

  /** Timeline creation time */
  created: Date;

  /** Last update time */
  updated: Date;

  /** Timeline metadata */
  metadata: Record<string, any>;
}

/**
 * Timeline integrity check parameters
 */
export interface IntegrityCheckParams {
  /** Timeline to check */
  timeline: Timeline;

  /** Start of date range */
  startDate?: Date;

  /** End of date range */
  endDate?: Date;

  /** Include checkpoints in verification */
  verifyCheckpoints?: boolean;

  /** Minimum acceptable integrity score */
  minIntegrityScore?: number;

  /** Detect tampering? */
  detectTampering?: boolean;
}

/**
 * Integrity violation detected
 */
export interface IntegrityViolation {
  /** Violation type */
  type: 'hash_mismatch' | 'missing_event' | 'orphaned_event' | 'duplicate' | 'temporal_inconsistency' | 'broken_chain' | 'signature_invalid';

  /** Severity level */
  severity: 'info' | 'warning' | 'error' | 'critical';

  /** Description of violation */
  description: string;

  /** Affected event(s) */
  affectedEvents: string[];

  /** Detection timestamp */
  detectedAt: Date;

  /** Suggested remediation */
  remediation?: string;
}

/**
 * Timeline integrity report
 */
export interface IntegrityReport {
  /** Timeline identifier */
  timelineId: string;

  /** Is timeline integrity intact? */
  isIntact: boolean;

  /** Integrity score (0-1) */
  integrityScore: number;

  /** Total events checked */
  totalEvents: number;

  /** Successfully verified events */
  verifiedEvents: number;

  /** Failed verifications */
  failedEvents: number;

  /** Detected violations */
  violations: IntegrityViolation[];

  /** Warnings */
  warnings: string[];

  /** Check timestamp */
  timestamp: Date;

  /** Time taken for check (ms) */
  duration: number;
}

// ============================================================================
// Tampering Detection
// ============================================================================

/**
 * Date range for temporal queries
 */
export interface DateRange {
  start: Date;
  end: Date;
}

/**
 * Tampering detection parameters
 */
export interface TamperingDetectionParams {
  /** Timeline to analyze */
  timeline: Timeline;

  /** Period to check */
  period?: DateRange;

  /** Use ML anomaly detection? */
  useMLDetection?: boolean;

  /** Sensitivity (0-1, higher = more sensitive) */
  sensitivity?: number;

  /** Check evidence chains? */
  verifyEvidence?: boolean;
}

/**
 * Detected tampering incident
 */
export interface TamperingIncident {
  /** Incident identifier */
  id: string;

  /** Incident type */
  type: 'direct_modification' | 'checkpoint_poisoning' | 'evidence_fabrication' | 'fingerprint_spoofing' | 'timeline_forgery' | 'timestamp_manipulation';

  /** Severity assessment */
  severity: 'low' | 'medium' | 'high' | 'critical';

  /** Confidence in detection (0-1) */
  confidence: number;

  /** When tampering was detected */
  detectedAt: Date;

  /** Estimated tampering time */
  estimatedTime?: Date;

  /** Affected events */
  affectedEvents: string[];

  /** Attack vector description */
  description: string;

  /** Recommended actions */
  recommendations: string[];

  /** Supporting evidence */
  evidence: any[];
}

/**
 * Tampering detection report
 */
export interface TamperingReport {
  /** Timeline identifier */
  timelineId: string;

  /** Was tampering detected? */
  tamperingDetected: boolean;

  /** Detected incidents */
  incidents: TamperingIncident[];

  /** Anomaly score (0-1) */
  anomalyScore: number;

  /** Suspicious patterns found */
  suspiciousPatterns: string[];

  /** Report timestamp */
  timestamp: Date;

  /** Analysis duration (ms) */
  duration: number;
}

// ============================================================================
// Evidence Chain
// ============================================================================

/**
 * Link in evidence chain
 */
export interface EvidenceChainLink {
  /** Evidence hash */
  evidenceHash: string;

  /** Previous link hash */
  prevLinkHash: string;

  /** Evidence timestamp */
  timestamp: Date;

  /** Verification result */
  verification: VerificationResult;

  /** Link index in chain */
  index: number;
}

/**
 * Chain of evidence supporting an event
 */
export interface EvidenceChain {
  /** Chain identifier */
  id: string;

  /** Event this chain supports */
  eventId: string;

  /** Evidence items */
  evidence: Evidence[];

  /** Chain links */
  links: EvidenceChainLink[];

  /** Chain root hash */
  chainHash: string;

  /** Overall reliability (0-1) */
  reliability: number;

  /** Is chain valid? */
  isValid: boolean;

  /** Chain creation time */
  created: Date;
}

/**
 * Evidence chain validation result
 */
export interface ChainValidation {
  /** Is chain valid? */
  isValid: boolean;

  /** Overall reliability score (0-1) */
  reliability: number;

  /** Per-link validation */
  linkValidations: {
    index: number;
    isValid: boolean;
    errors: string[];
  }[];

  /** Chain integrity check */
  integrityCheck: {
    passed: boolean;
    expectedHash: string;
    actualHash: string;
  };

  /** Validation timestamp */
  timestamp: Date;
}

// ============================================================================
// Checkpoints
// ============================================================================

/**
 * Historical checkpoint metadata
 */
export interface CheckpointMetadata {
  /** Checkpoint creator */
  creator: string;

  /** Reason for checkpoint */
  reason: string;

  /** Event date range covered */
  eventRange: [Date, Date];

  /** Additional metadata */
  [key: string]: any;
}

/**
 * Historical checkpoint snapshot
 */
export interface Checkpoint {
  /** Unique checkpoint identifier */
  id: string;

  /** Timeline this checkpoint belongs to */
  timelineId: string;

  /** Checkpoint creation time */
  timestamp: Date;

  /** Optional checkpoint name */
  name?: string;

  /** Timeline state fingerprint */
  fingerprint: string;

  /** Number of events at checkpoint */
  eventCount: number;

  /** Integrity score at checkpoint */
  integrityScore: number;

  /** Merkle tree root of events */
  merkleRoot: string;

  /** Checkpoint metadata */
  metadata: CheckpointMetadata;

  /** Verification signatures */
  signatures: Signature[];

  /** Previous checkpoint ID */
  previousCheckpoint?: string;

  /** Is checkpoint validated? */
  isValidated?: boolean;
}

/**
 * Checkpoint creation parameters
 */
export interface CheckpointCreationParams {
  /** Timeline to checkpoint */
  timeline: Timeline;

  /** Optional checkpoint name */
  name?: string;

  /** Reason for checkpoint */
  reason?: string;

  /** Events to include (default: all) */
  includeEvents?: HistoricalEvent[];

  /** Collect signatures? */
  collectSignatures?: boolean;
}

/**
 * Checkpoint validation result
 */
export interface CheckpointValidation {
  /** Is checkpoint valid? */
  isValid: boolean;

  /** Signatures valid? */
  signaturesValid: boolean;

  /** Merkle root matches? */
  merkleRootMatch: boolean;

  /** Fingerprint matches? */
  fingerprintMatch: boolean;

  /** Detected discrepancies */
  discrepancies: string[];

  /** Validation timestamp */
  timestamp: Date;
}

/**
 * Rollback operation result
 */
export interface RollbackResult {
  /** Was rollback successful? */
  success: boolean;

  /** Checkpoint used for rollback */
  checkpointId: string;

  /** Number of events removed */
  eventsRemoved: number;

  /** Events that were removed */
  removedEvents: HistoricalEvent[];

  /** Backup identifier */
  backupId: string;

  /** New timeline fingerprint */
  newFingerprint: string;

  /** Rollback timestamp */
  timestamp: Date;

  /** Error message if failed */
  error?: string;
}

// ============================================================================
// Fingerprinting
// ============================================================================

/**
 * Timeline fingerprint
 */
export interface TimelineFingerprint {
  /** Fingerprint value (64-byte hex) */
  value: string;

  /** Timeline identifier */
  timelineId: string;

  /** Fingerprint timestamp */
  timestamp: Date;

  /** Merkle tree root */
  merkleRoot: string;

  /** Number of events */
  eventCount: number;

  /** Metadata hash */
  metadataHash: string;

  /** Algorithm version */
  version: string;
}

/**
 * Fingerprint comparison result
 */
export interface FingerprintComparison {
  /** Are fingerprints identical? */
  identical: boolean;

  /** Similarity score (0-1) */
  similarity: number;

  /** Comparison classification */
  classification: 'identical' | 'highly_similar' | 'partially_similar' | 'different';

  /** Common prefix length */
  commonPrefixLength: number;

  /** Divergence point (if known) */
  divergencePoint?: Date;

  /** Comparison details */
  details: string;
}

/**
 * Timeline identification result
 */
export interface TimelineIdentification {
  /** Exact matches found */
  exactMatches: string[];

  /** Similar timelines */
  similarTimelines: {
    timelineId: string;
    similarity: number;
    divergencePoint?: Date;
    divergenceEvents?: number;
  }[];

  /** Identification confidence (0-1) */
  confidence: number;

  /** Identification timestamp */
  timestamp: Date;
}

/**
 * Fingerprint evolution entry
 */
export interface FingerprintEvolution {
  /** Checkpoint timestamp */
  timestamp: Date;

  /** Fingerprint at this time */
  fingerprint: string;

  /** Changes since last checkpoint */
  changes: {
    eventsAdded: number;
    eventsModified: number;
    eventsRemoved: number;
  };

  /** Integrity score */
  integrityScore: number;
}

// ============================================================================
// Immutability
// ============================================================================

/**
 * Immutability seal for protected events
 */
export interface ImmutabilitySeal {
  /** Seal identifier */
  id: string;

  /** Event hash */
  eventHash: string;

  /** Seal creation time */
  timestamp: Date;

  /** Event category */
  category: EventCategory;

  /** Required signatures */
  signatures: Signature[];

  /** Hardware security module seal */
  hardwareSeal?: string;

  /** Is seal intact? */
  isIntact?: boolean;
}

/**
 * Modification request
 */
export interface ModificationRequest {
  /** Request identifier */
  id: string;

  /** Event to modify */
  eventId: string;

  /** Proposed changes */
  proposedChanges: Partial<HistoricalEvent>;

  /** Justification for modification */
  justification: string;

  /** Requestor identifier */
  requestor: string;

  /** Request timestamp */
  timestamp: Date;

  /** Required approvals */
  requiredApprovals: number;

  /** Collected approvals */
  approvals: Signature[];

  /** Request status */
  status: 'pending' | 'approved' | 'rejected' | 'expired';

  /** Cooling period end (for protected events) */
  coolingPeriodEnd?: Date;
}

/**
 * Modification approval result
 */
export interface ModificationApproval {
  /** Was modification approved? */
  approved: boolean;

  /** Request identifier */
  requestId: string;

  /** Approval timestamp */
  timestamp: Date;

  /** Approver signatures */
  signatures: Signature[];

  /** Reason if rejected */
  rejectionReason?: string;

  /** When modification can proceed */
  effectiveDate?: Date;
}

// ============================================================================
// Preservation
// ============================================================================

/**
 * Storage degradation status
 */
export interface DegradationStatus {
  /** Storage identifier */
  storageId: string;

  /** Degradation score (0-100%) */
  degradationScore: number;

  /** Errors found */
  errorsFound: number;

  /** Total bits checked */
  totalBits: number;

  /** Last check timestamp */
  lastChecked: Date;

  /** Status classification */
  status: 'healthy' | 'degrading' | 'critical' | 'failed';

  /** Recommended actions */
  recommendations: string[];
}

/**
 * Recovery operation result
 */
export interface RecoveryResult {
  /** Was recovery successful? */
  success: boolean;

  /** Record identifier */
  recordId: string;

  /** Recovery method used */
  method: 'primary' | 'redundant_copy' | 'error_correction' | 'checkpoint_reconstruction' | 'forensic';

  /** Recovered data integrity */
  integrityCheck: {
    passed: boolean;
    checksumValid: boolean;
  };

  /** Recovery timestamp */
  timestamp: Date;

  /** Error message if failed */
  error?: string;
}

/**
 * Preservation encoding info
 */
export interface PreservationEncoding {
  /** Encoding version */
  version: string;

  /** Compression algorithm */
  compression: 'quantum-resistant' | 'none';

  /** Error correction scheme */
  errorCorrection: 'reed-solomon' | 'ldpc' | 'turbo';

  /** Encryption algorithm */
  encryption: 'post-quantum' | 'AES-256' | 'none';

  /** Number of redundant copies */
  redundancy: number;

  /** Storage medium */
  medium: 'diamond' | 'dna' | 'optical' | 'magnetic' | 'ssd';

  /** Expected lifetime (years) */
  expectedLifetime: number;
}

// ============================================================================
// Configuration
// ============================================================================

/**
 * Historical integrity configuration
 */
export interface IntegrityConfig {
  /** Minimum integrity score threshold */
  minIntegrityScore: number;

  /** Enable real-time tampering detection */
  enableTamperingDetection: boolean;

  /** Monitoring interval (seconds) */
  monitoringInterval: number;

  /** Checkpoint frequency by category */
  checkpointFrequency: {
    critical: number;    // seconds
    standard: number;
    archival: number;
  };

  /** Evidence verification enabled */
  verifyEvidence: boolean;

  /** ML anomaly detection enabled */
  useMLDetection: boolean;

  /** Quantum-resistant cryptography */
  useQuantumResistant: boolean;

  /** Storage redundancy level */
  redundancyLevel: number;

  /** Audit logging enabled */
  enableAuditLogging: boolean;
}

// ============================================================================
// API Response Types
// ============================================================================

/**
 * Success result wrapper
 */
export interface SuccessResult<T> {
  success: true;
  data: T;
  timestamp: Date;
}

/**
 * Error result wrapper
 */
export interface ErrorResult {
  success: false;
  error: {
    code: string;
    message: string;
    details?: any;
  };
  timestamp: Date;
}

/**
 * Generic result type
 */
export type Result<T> = SuccessResult<T> | ErrorResult;

/**
 * Async result type
 */
export type AsyncResult<T> = Promise<Result<T>>;

// ============================================================================
// Error Codes
// ============================================================================

/**
 * WIA-TIME-011 error codes
 */
export enum IntegrityErrorCode {
  VERIFICATION_FAILED = 'I001',
  HASH_MISMATCH = 'I002',
  TAMPERING_DETECTED = 'I003',
  CHECKPOINT_INVALID = 'I004',
  EVIDENCE_INVALID = 'I005',
  SIGNATURE_INVALID = 'I006',
  IMMUTABILITY_VIOLATION = 'I007',
  TIMELINE_CORRUPTED = 'I008',
  FINGERPRINT_MISMATCH = 'I009',
  STORAGE_FAILURE = 'I010',
  CONSENSUS_FAILED = 'I011',
  INVALID_PARAMETERS = 'I012',
}

/**
 * Historical integrity error
 */
export class IntegrityError extends Error {
  constructor(
    public code: IntegrityErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'IntegrityError';
  }
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Evidence type reliability weights
 */
export const EVIDENCE_RELIABILITY_WEIGHTS = {
  [EvidenceType.PHYSICAL_ARTIFACT]: 1.0,
  [EvidenceType.DOCUMENT]: 0.9,
  [EvidenceType.BLOCKCHAIN_RECORD]: 0.95,
  [EvidenceType.SENSOR_DATA]: 0.8,
  [EvidenceType.VIDEO_RECORDING]: 0.7,
  [EvidenceType.PHOTOGRAPHIC]: 0.7,
  [EvidenceType.WITNESS_TESTIMONY]: 0.5,
  [EvidenceType.AUDIO_RECORDING]: 0.6,
  [EvidenceType.DIGITAL_TRAIL]: 0.6,
} as const;

/**
 * Event category weights for integrity calculation
 */
export const EVENT_CATEGORY_WEIGHTS = {
  [EventCategory.CRITICAL]: 10.0,
  [EventCategory.PROTECTED]: 5.0,
  [EventCategory.STANDARD]: 1.0,
  [EventCategory.TRIVIAL]: 0.1,
} as const;

/**
 * Default integrity configuration
 */
export const DEFAULT_INTEGRITY_CONFIG: IntegrityConfig = {
  minIntegrityScore: 0.99,
  enableTamperingDetection: true,
  monitoringInterval: 60,
  checkpointFrequency: {
    critical: 3600,      // 1 hour
    standard: 86400,     // 1 day
    archival: 604800,    // 1 week
  },
  verifyEvidence: true,
  useMLDetection: true,
  useQuantumResistant: true,
  redundancyLevel: 5,
  enableAuditLogging: true,
};

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core
  Vector3,
  Signature,

  // Events
  Evidence,
  HistoricalEvent,

  // Verification
  EventVerificationParams,
  SignatureStatus,
  VerificationResult,

  // Timeline
  Timeline,
  IntegrityCheckParams,
  IntegrityViolation,
  IntegrityReport,

  // Tampering
  DateRange,
  TamperingDetectionParams,
  TamperingIncident,
  TamperingReport,

  // Evidence Chain
  EvidenceChainLink,
  EvidenceChain,
  ChainValidation,

  // Checkpoints
  CheckpointMetadata,
  Checkpoint,
  CheckpointCreationParams,
  CheckpointValidation,
  RollbackResult,

  // Fingerprinting
  TimelineFingerprint,
  FingerprintComparison,
  TimelineIdentification,
  FingerprintEvolution,

  // Immutability
  ImmutabilitySeal,
  ModificationRequest,
  ModificationApproval,

  // Preservation
  DegradationStatus,
  RecoveryResult,
  PreservationEncoding,

  // Configuration
  IntegrityConfig,

  // Results
  SuccessResult,
  ErrorResult,
  Result,
  AsyncResult,
};

export {
  EventCategory,
  EvidenceType,
  IntegrityErrorCode,
  IntegrityError,
  EVIDENCE_RELIABILITY_WEIGHTS,
  EVENT_CATEGORY_WEIGHTS,
  DEFAULT_INTEGRITY_CONFIG,
};
