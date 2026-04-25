/**
 * WIA-TIME-035: Temporal Information Security - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Security Working Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Security Types
// ============================================================================

/**
 * Encryption algorithms for temporal data
 */
export enum EncryptionAlgorithm {
  TE_128 = 'TE-128',
  TE_256 = 'TE-256',
  TE_512 = 'TE-512',
  QTE_256 = 'QTE-256',
  QTE_512 = 'QTE-512',
}

/**
 * Security classification levels
 */
export enum SecurityClassification {
  PUBLIC = 'public',
  INTERNAL = 'internal',
  CONFIDENTIAL = 'confidential',
  TEMPORAL_SECRET = 'temporal-secret',
  PARADOX_RISK = 'paradox-risk',
}

/**
 * Timeline identifier
 */
export interface TimelineIdentifier {
  /** Unique timeline ID */
  id: string;

  /** Timeline name */
  name: string;

  /** Origin timestamp */
  origin: Date;

  /** Branch point (if branched timeline) */
  branchPoint?: Date;

  /** Parent timeline ID (if branched) */
  parentTimelineId?: string;

  /** Causality chain hash */
  causalityChain: string;
}

// ============================================================================
// Temporal Encryption
// ============================================================================

/**
 * Encryption configuration
 */
export interface EncryptionConfig {
  /** Encryption algorithm */
  algorithm: EncryptionAlgorithm;

  /** Key size in bits */
  keySize: number;

  /** Quantum resistant */
  quantumResistant: boolean;

  /** Timeline binding enabled */
  timelineBinding: boolean;

  /** Mode of operation */
  mode?: 'CTR' | 'GCM' | 'CBC';

  /** Additional parameters */
  parameters?: Record<string, unknown>;
}

/**
 * Encrypted data structure
 */
export interface EncryptedData {
  /** Ciphertext */
  ciphertext: string; // Base64 encoded

  /** Algorithm used */
  algorithm: EncryptionAlgorithm;

  /** Initialization vector */
  iv: string; // Base64 encoded

  /** Authentication tag (for authenticated encryption) */
  authTag?: string; // Base64 encoded

  /** Key ID used for encryption */
  keyId: string;

  /** Timeline ID */
  timelineId: string;

  /** Temporal context */
  temporalContext: TemporalContext;

  /** Metadata */
  metadata?: {
    encrypted: Date;
    classification: SecurityClassification;
    dataType?: string;
  };
}

/**
 * Decryption result
 */
export interface DecryptionResult {
  /** Decrypted plaintext */
  plaintext: string | Buffer;

  /** Success status */
  success: boolean;

  /** Timeline verified */
  timelineVerified: boolean;

  /** Causality verified */
  causalityVerified: boolean;

  /** Warning messages */
  warnings?: string[];

  /** Metadata */
  metadata?: {
    decrypted: Date;
    algorithm: EncryptionAlgorithm;
    keyId: string;
  };
}

/**
 * Temporal context for encryption
 */
export interface TemporalContext {
  /** Timeline ID */
  timelineId: string;

  /** Timestamp */
  timestamp: Date;

  /** Causality proof */
  causalityProof: string;

  /** Quantum state signature (if applicable) */
  quantumState?: string;

  /** Additional context */
  additionalContext?: Record<string, unknown>;
}

// ============================================================================
// Time-Lock Mechanisms
// ============================================================================

/**
 * Time-lock types
 */
export enum TimeLockType {
  CHRONOLOGICAL = 'chronological',
  CONDITIONAL = 'conditional',
  PUZZLE = 'puzzle',
  MULTISIG = 'multisig',
}

/**
 * Time-lock configuration
 */
export interface TimeLock {
  /** Time-lock type */
  type: TimeLockType;

  /** Unlock date (for chronological) */
  unlockDate?: Date;

  /** Unlock condition (for conditional) */
  unlockCondition?: UnlockCondition;

  /** Puzzle parameters (for puzzle) */
  puzzleParams?: PuzzleParameters;

  /** Multi-signature parameters (for multisig) */
  multisigParams?: MultisigParameters;

  /** Allow past access */
  allowPastAccess: boolean;

  /** Allow future access */
  allowFutureAccess: boolean;

  /** Timeline restrictions */
  timelineRestrictions?: string[];
}

/**
 * Unlock condition for conditional time-locks
 */
export interface UnlockCondition {
  /** Condition type */
  type: 'event' | 'timeline-state' | 'causality' | 'custom';

  /** Event identifier (for event-based) */
  eventId?: string;

  /** Timeline state requirement */
  timelineState?: {
    timelineId: string;
    requiredState: string;
  };

  /** Causality requirement */
  causalityRequirement?: {
    chainHash: string;
    minDepth: number;
  };

  /** Custom condition */
  customCondition?: {
    oracleUrl: string;
    parameters: Record<string, unknown>;
  };

  /** Verification method */
  verificationMethod: string;
}

/**
 * Puzzle parameters for puzzle time-locks
 */
export interface PuzzleParameters {
  /** Number of operations required */
  operations: number;

  /** Operations per second (estimated) */
  opsPerSecond: number;

  /** Puzzle difficulty */
  difficulty: number;

  /** Puzzle seed */
  seed: string;

  /** Verifiable Delay Function parameters */
  vdfParams?: {
    securityParameter: number;
    timeDelay: number;
  };
}

/**
 * Multi-signature parameters
 */
export interface MultisigParameters {
  /** Required signatures (k-of-n) */
  threshold: number;

  /** Total participants */
  totalParticipants: number;

  /** Participant public keys */
  participantKeys: string[];

  /** Timeframe constraints per participant */
  timeframes?: Array<{
    participantId: string;
    validFrom: Date;
    validUntil: Date;
  }>;
}

/**
 * Time-locked data
 */
export interface TimeLockedData {
  /** Encrypted data */
  encryptedData: EncryptedData;

  /** Time-lock configuration */
  timeLock: TimeLock;

  /** Lock ID */
  lockId: string;

  /** Creation timestamp */
  created: Date;

  /** Status */
  status: 'locked' | 'unlocked' | 'expired' | 'failed';

  /** Unlock attempts */
  unlockAttempts?: number;
}

// ============================================================================
// Temporal Keys
// ============================================================================

/**
 * Key types
 */
export enum KeyType {
  MASTER_TEMPORAL_KEY = 'MTK',
  TIMELINE_SPECIFIC_KEY = 'TSK',
  SESSION_KEY = 'SK',
  DATA_ENCRYPTION_KEY = 'DEK',
  EMERGENCY_RECOVERY_KEY = 'ERK',
}

/**
 * Cryptographic key
 */
export interface TemporalKey {
  /** Key ID */
  id: string;

  /** Key type */
  type: KeyType;

  /** Key material (encrypted) */
  keyMaterial: string;

  /** Algorithm */
  algorithm: EncryptionAlgorithm;

  /** Key size */
  keySize: number;

  /** Creation date */
  created: Date;

  /** Expiration date */
  expires?: Date;

  /** Timeline binding */
  timelineId?: string;

  /** Quantum resistant */
  quantumResistant: boolean;

  /** Status */
  status: 'active' | 'expired' | 'revoked' | 'compromised';

  /** Metadata */
  metadata?: {
    createdBy?: string;
    purpose?: string;
    tags?: string[];
  };
}

/**
 * Key generation parameters
 */
export interface KeyGenerationParams {
  /** Key type */
  keyType: KeyType;

  /** Key size in bits */
  keySize: 256 | 512;

  /** Algorithm */
  algorithm: EncryptionAlgorithm;

  /** Quantum resistant */
  quantumResistant: boolean;

  /** Timeline binding */
  timelineBinding?: {
    timelineId: string;
    bindingStrength: 'weak' | 'strong' | 'absolute';
  };

  /** Rotation period (in days) */
  rotationPeriod?: number;

  /** Backup/recovery options */
  backup?: {
    enabled: boolean;
    sharesThreshold?: number;
    sharesTotalCount?: number;
  };
}

/**
 * Key rotation result
 */
export interface KeyRotationResult {
  /** New key ID */
  newKeyId: string;

  /** Old key ID */
  oldKeyId: string;

  /** Rotation timestamp */
  rotatedAt: Date;

  /** Overlap period start */
  overlapStart: Date;

  /** Overlap period end */
  overlapEnd: Date;

  /** Re-encryption required */
  reencryptionRequired: boolean;

  /** Affected data count */
  affectedDataCount?: number;
}

// ============================================================================
// Secure Communication
// ============================================================================

/**
 * Secure temporal channel
 */
export interface SecureChannel {
  /** Channel ID */
  id: string;

  /** Source timeline */
  sourceTimeline: TimelineIdentifier;

  /** Target timeline */
  targetTimeline: TimelineIdentifier;

  /** Encryption algorithm */
  encryption: EncryptionAlgorithm;

  /** Authentication method */
  authentication: 'single-factor' | 'multi-factor' | 'quantum-signature';

  /** Channel status */
  status: 'establishing' | 'active' | 'closed' | 'error';

  /** Session key */
  sessionKey?: TemporalKey;

  /** Created timestamp */
  created: Date;

  /** Last activity */
  lastActivity?: Date;

  /** Security properties */
  securityProperties: {
    confidentiality: boolean;
    integrity: boolean;
    authenticity: boolean;
    forwardSecrecy: boolean;
    causalityPreservation: boolean;
  };
}

/**
 * Temporal message
 */
export interface TemporalMessage {
  /** Message ID */
  id: string;

  /** Sender timeline */
  sender: TimelineIdentifier;

  /** Recipient timeline */
  recipient: TimelineIdentifier;

  /** Encrypted payload */
  payload: EncryptedData;

  /** Message timestamp */
  timestamp: Date;

  /** Causality chain */
  causalityChain: string;

  /** Digital signature */
  signature: string;

  /** Message priority */
  priority?: 'low' | 'normal' | 'high' | 'critical';

  /** Delivery confirmation required */
  confirmationRequired?: boolean;
}

/**
 * Message sending result
 */
export interface MessageSendResult {
  /** Success status */
  success: boolean;

  /** Message ID */
  messageId: string;

  /** Sent timestamp */
  sentAt: Date;

  /** Delivery status */
  deliveryStatus: 'sent' | 'delivered' | 'failed' | 'pending';

  /** Error message (if failed) */
  error?: string;

  /** Causality verified */
  causalityVerified: boolean;
}

// ============================================================================
// Security Auditing
// ============================================================================

/**
 * Security event types
 */
export enum SecurityEventType {
  ENCRYPTION = 'encryption',
  DECRYPTION = 'decryption',
  KEY_GENERATION = 'key_generation',
  KEY_ROTATION = 'key_rotation',
  KEY_REVOCATION = 'key_revocation',
  ACCESS_GRANTED = 'access_granted',
  ACCESS_DENIED = 'access_denied',
  AUTHENTICATION = 'authentication',
  CHANNEL_CREATED = 'channel_created',
  MESSAGE_SENT = 'message_sent',
  SECURITY_VIOLATION = 'security_violation',
  THREAT_DETECTED = 'threat_detected',
  ANOMALY_DETECTED = 'anomaly_detected',
}

/**
 * Security audit event
 */
export interface SecurityAuditEvent {
  /** Event ID */
  id: string;

  /** Timestamp */
  timestamp: Date;

  /** Timeline ID */
  timelineId: string;

  /** Event type */
  eventType: SecurityEventType;

  /** Actor (user/system) */
  actor: {
    id: string;
    type: 'user' | 'system' | 'service';
    timelineOrigin?: string;
  };

  /** Action details */
  action: {
    operation: string;
    targetResource?: string;
    parameters?: Record<string, unknown>;
  };

  /** Result */
  result: 'success' | 'failure' | 'partial';

  /** Security context */
  securityContext: {
    threatScore: number; // 0-1
    anomalyDetected: boolean;
    policyViolations: string[];
  };

  /** Causality proof */
  causalityProof: string;

  /** Digital signature */
  signature: string;
}

/**
 * Security audit query
 */
export interface AuditQuery {
  /** Timeframe */
  timeframe: {
    start: Date;
    end: Date;
  };

  /** Timeline filters */
  timelines?: string[];

  /** Event type filters */
  eventTypes?: SecurityEventType[];

  /** Actor filters */
  actors?: string[];

  /** Result filters */
  results?: ('success' | 'failure' | 'partial')[];

  /** Minimum threat score */
  minThreatScore?: number;

  /** Detail level */
  detailLevel?: 'summary' | 'standard' | 'full';

  /** Maximum results */
  limit?: number;

  /** Offset for pagination */
  offset?: number;
}

/**
 * Audit report
 */
export interface AuditReport {
  /** Report ID */
  id: string;

  /** Generation timestamp */
  generated: Date;

  /** Query parameters */
  query: AuditQuery;

  /** Summary statistics */
  summary: {
    totalEvents: number;
    timelinesAnalyzed: number;
    securityIncidents: number;
    threatsDetected: number;
    averageThreatScore: number;
  };

  /** Events */
  events: SecurityAuditEvent[];

  /** Timeline analysis */
  timelineAnalysis: Array<{
    timelineId: string;
    eventCount: number;
    threatScore: number;
    violations: number;
  }>;

  /** Threats detected */
  threats: ThreatDetection[];

  /** Recommendations */
  recommendations: string[];

  /** Security score (0-100) */
  securityScore: number;

  /** Compliance status */
  complianceStatus: {
    compliant: boolean;
    violations: string[];
    lastAssessment: Date;
  };
}

// ============================================================================
// Threat Detection
// ============================================================================

/**
 * Threat severity levels
 */
export enum ThreatSeverity {
  LOW = 'low',
  MEDIUM = 'medium',
  HIGH = 'high',
  CRITICAL = 'critical',
  CATASTROPHIC = 'catastrophic',
}

/**
 * Threat detection
 */
export interface ThreatDetection {
  /** Threat ID */
  id: string;

  /** Detection timestamp */
  detected: Date;

  /** Threat type */
  type:
    | 'temporal-eavesdropping'
    | 'time-travel-interception'
    | 'historical-data-theft'
    | 'future-information-leak'
    | 'timeline-manipulation'
    | 'paradox-exploitation'
    | 'quantum-attack'
    | 'insider-threat'
    | 'brute-force'
    | 'replay-attack'
    | 'other';

  /** Severity */
  severity: ThreatSeverity;

  /** Timeline(s) affected */
  affectedTimelines: string[];

  /** Threat description */
  description: string;

  /** Indicators */
  indicators: string[];

  /** Confidence score (0-1) */
  confidence: number;

  /** Status */
  status: 'detected' | 'investigating' | 'contained' | 'resolved' | 'false-positive';

  /** Automated response taken */
  automatedResponse?: string[];

  /** Recommended actions */
  recommendedActions: string[];

  /** Related events */
  relatedEvents?: string[];
}

/**
 * Threat assessment
 */
export interface ThreatAssessment {
  /** Assessment ID */
  id: string;

  /** Assessment timestamp */
  timestamp: Date;

  /** Overall threat level */
  overallThreatLevel: ThreatSeverity;

  /** Active threats */
  activeThreats: ThreatDetection[];

  /** Timeline security scores */
  timelineSecurityScores: Map<string, number>;

  /** Vulnerabilities identified */
  vulnerabilities: Array<{
    id: string;
    description: string;
    severity: ThreatSeverity;
    affectedComponents: string[];
    mitigation: string;
  }>;

  /** Risk score (0-100) */
  riskScore: number;

  /** Trends */
  trends: {
    threatTrend: 'increasing' | 'stable' | 'decreasing';
    incidentFrequency: number;
    avgResponseTime: number; // seconds
  };
}

// ============================================================================
// Information Leak Detection
// ============================================================================

/**
 * Leak detection result
 */
export interface LeakDetectionResult {
  /** Detection ID */
  id: string;

  /** Detection timestamp */
  timestamp: Date;

  /** Leak detected */
  leakDetected: boolean;

  /** Leak type */
  leakType?:
    | 'cross-timeline'
    | 'temporal-side-channel'
    | 'unauthorized-access'
    | 'data-exfiltration'
    | 'encryption-bypass';

  /** Source timeline */
  sourceTimeline?: string;

  /** Destination timeline */
  destinationTimeline?: string;

  /** Leaked data classification */
  dataClassification?: SecurityClassification;

  /** Leak volume (bytes) */
  leakVolume?: number;

  /** Confidence score (0-1) */
  confidence: number;

  /** Indicators */
  indicators: string[];

  /** Impact assessment */
  impact: {
    severity: ThreatSeverity;
    affectedSystems: string[];
    potentialDamage: string;
  };

  /** Remediation status */
  remediation: {
    automated: boolean;
    actions: string[];
    status: 'pending' | 'in-progress' | 'completed';
  };
}

// ============================================================================
// Access Control
// ============================================================================

/**
 * Access control policy
 */
export interface AccessControlPolicy {
  /** Policy ID */
  id: string;

  /** Policy name */
  name: string;

  /** Source timeline(s) */
  sourceTimelines: string[];

  /** Target timeline(s) */
  targetTimelines: string[];

  /** Allowed operations */
  allowedOperations: ('read' | 'write' | 'execute' | 'delete')[];

  /** Required authentication */
  requiredAuthentication: 'none' | 'single-factor' | 'multi-factor' | 'quantum';

  /** Required authorization level */
  requiredAuthorizationLevel: number; // 0-10

  /** Temporal constraints */
  temporalConstraints?: {
    allowPastAccess: boolean;
    allowFutureAccess: boolean;
    timeRanges?: Array<{
      start: Date;
      end: Date;
    }>;
  };

  /** Data classification constraints */
  dataClassificationLimit?: SecurityClassification;

  /** Active */
  active: boolean;
}

/**
 * Access request
 */
export interface AccessRequest {
  /** Request ID */
  id: string;

  /** Requester */
  requester: {
    id: string;
    timelineId: string;
    authenticationLevel: 'none' | 'single-factor' | 'multi-factor' | 'quantum';
    authorizationLevel: number;
  };

  /** Resource */
  resource: {
    id: string;
    type: string;
    timelineId: string;
    classification: SecurityClassification;
  };

  /** Requested operation */
  operation: 'read' | 'write' | 'execute' | 'delete';

  /** Request timestamp */
  timestamp: Date;

  /** Causality proof */
  causalityProof: string;
}

/**
 * Access decision
 */
export interface AccessDecision {
  /** Decision ID */
  id: string;

  /** Request ID */
  requestId: string;

  /** Decision */
  decision: 'allow' | 'deny' | 'conditional';

  /** Reason */
  reason: string;

  /** Applied policy */
  appliedPolicy?: string;

  /** Conditions (if conditional) */
  conditions?: string[];

  /** Decision timestamp */
  timestamp: Date;

  /** Valid until */
  validUntil?: Date;

  /** Logged */
  logged: boolean;
}

// ============================================================================
// Security Configuration
// ============================================================================

/**
 * Security configuration
 */
export interface SecurityConfiguration {
  /** Encryption settings */
  encryption: {
    defaultAlgorithm: EncryptionAlgorithm;
    minimumKeySize: number;
    quantumResistantRequired: boolean;
    timelineBindingRequired: boolean;
  };

  /** Key management */
  keyManagement: {
    rotationPeriod: number; // days
    masterKeyBackup: boolean;
    hsmRequired: boolean;
    emergencyRecoveryEnabled: boolean;
  };

  /** Monitoring */
  monitoring: {
    realTimeMonitoring: boolean;
    anomalyDetection: boolean;
    threatIntelligence: boolean;
    alertThreshold: ThreatSeverity;
  };

  /** Audit */
  audit: {
    enabled: boolean;
    logLevel: 'minimal' | 'standard' | 'detailed' | 'comprehensive';
    retentionPeriod: number; // days
    immutableLogs: boolean;
  };

  /** Access control */
  accessControl: {
    defaultPolicy: 'deny' | 'allow';
    multiFactorRequired: boolean;
    timelineIsolationEnabled: boolean;
  };

  /** Compliance */
  compliance: {
    standard: 'WIA-TIME-035';
    certificationLevel: 1 | 2 | 3;
    assessmentFrequency: number; // days
  };
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
 * Temporal coordinates
 */
export interface TemporalCoordinates {
  /** Timeline ID */
  timelineId: string;

  /** Timestamp */
  timestamp: Date;

  /** Location (optional) */
  location?: {
    latitude: number;
    longitude: number;
    altitude?: number;
  };
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-TIME-035 error codes
 */
export enum SecurityErrorCode {
  ENCRYPTION_FAILED = 'SEC001',
  DECRYPTION_FAILED = 'SEC002',
  INVALID_KEY = 'SEC003',
  KEY_EXPIRED = 'SEC004',
  TIMELINE_MISMATCH = 'SEC005',
  CAUSALITY_VIOLATION = 'SEC006',
  ACCESS_DENIED = 'SEC007',
  AUTHENTICATION_FAILED = 'SEC008',
  TIMELOCK_NOT_READY = 'SEC009',
  THREAT_DETECTED = 'SEC010',
  LEAK_DETECTED = 'SEC011',
  CONFIGURATION_ERROR = 'SEC012',
}

/**
 * Temporal security error
 */
export class TemporalSecurityError extends Error {
  constructor(
    public code: SecurityErrorCode,
    message: string,
    public timelineId?: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'TemporalSecurityError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  TimelineIdentifier,
  EncryptionConfig,
  EncryptedData,
  DecryptionResult,
  TemporalContext,
  TimeLock,
  UnlockCondition,
  PuzzleParameters,
  MultisigParameters,
  TimeLockedData,
  TemporalKey,
  KeyGenerationParams,
  KeyRotationResult,
  SecureChannel,
  TemporalMessage,
  MessageSendResult,
  SecurityAuditEvent,
  AuditQuery,
  AuditReport,
  ThreatDetection,
  ThreatAssessment,
  LeakDetectionResult,
  AccessControlPolicy,
  AccessRequest,
  AccessDecision,
  SecurityConfiguration,
  TemporalCoordinates,
};

export {
  EncryptionAlgorithm,
  SecurityClassification,
  KeyType,
  TimeLockType,
  SecurityEventType,
  ThreatSeverity,
  SecurityErrorCode,
  TemporalSecurityError,
};
