/**
 * WIA-TIME-025: Temporal Verification - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Time Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Geometry Types
// ============================================================================

/**
 * Three-dimensional spatial coordinates
 */
export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

/**
 * Spacetime coordinates (4D)
 */
export interface SpacetimeCoordinates {
  /** Spatial coordinates in meters */
  position: Vector3;

  /** Temporal coordinate */
  time: Date | string;

  /** Timeline identifier */
  timeline?: string;

  /** Reference frame identifier */
  referenceFrame?: string;
}

// ============================================================================
// Signature Types
// ============================================================================

/**
 * Supported signature algorithms
 */
export type SignatureAlgorithm =
  | 'ECDSA-TEMPORAL-SHA3'
  | 'RSA-TEMPORAL-4096'
  | 'EdDSA-25519-TEMPORAL'
  | 'SPHINCS+-TEMPORAL'
  | 'DILITHIUM-TEMPORAL';

/**
 * Quantum temporal nonce
 */
export interface QuantumNonce {
  /** Base64 encoded quantum random value */
  value: string;

  /** Generation timestamp */
  generatedAt: Date;

  /** Bits of entropy (≥256) */
  entropy: number;

  /** Quantum random source identifier */
  quantumSource: string;

  /** Verification hash */
  hash?: string;
}

/**
 * Temporal signature
 */
export interface TemporalSignature {
  /** Signature algorithm used */
  algorithm: SignatureAlgorithm;

  /** Public key (hex encoded) */
  publicKey: string;

  /** Signature data (hex encoded) */
  signature: string;

  /** Quantum temporal nonce */
  temporalNonce: QuantumNonce;

  /** Signing timestamp */
  timestamp: Date;

  /** Certificate chain */
  certificates: Certificate[];

  /** Additional metadata */
  metadata?: Record<string, unknown>;
}

/**
 * Digital certificate
 */
export interface Certificate {
  /** Certificate issuer */
  issuer: string;

  /** Certificate subject */
  subject: string;

  /** Public key */
  publicKey: string;

  /** Valid from date */
  validFrom: Date;

  /** Valid until date */
  validUntil: Date;

  /** Certificate signature */
  signature: string;

  /** Serial number */
  serialNumber: string;

  /** Revoked status */
  revoked?: boolean;
}

// ============================================================================
// Biometric Types
// ============================================================================

/**
 * Biometric hash
 */
export interface BiometricHash {
  /** Hash algorithm */
  algorithm: 'SHA3-512' | 'BLAKE3';

  /** Hash value (hex encoded) */
  hash: string;

  /** Salt (hex encoded) */
  salt: string;

  /** Match confidence (0-1) */
  confidence?: number;
}

/**
 * Biometric data (hashed, never raw)
 */
export interface BiometricData {
  /** Fingerprint hash */
  fingerprint?: BiometricHash;

  /** Retina scan hash */
  retina?: BiometricHash;

  /** DNA profile hash */
  dna?: BiometricHash;

  /** Facial recognition hash */
  facial?: BiometricHash;

  /** Voice print hash */
  voice?: BiometricHash;

  /** Temporal age at verification */
  temporalAge: number;

  /** Travel history hash */
  temporalHistory?: string[];
}

// ============================================================================
// Energy Types
// ============================================================================

/**
 * Energy signature
 */
export interface EnergySignature {
  /** Total energy in joules */
  total: number;

  /** Energy profile over time */
  profile: number[];

  /** Peak energy points */
  peaks: number[];

  /** SHA3-512 hash of profile */
  hash: string;

  /** Quantum fingerprint */
  quantum: {
    /** Entanglement strength (0-1) */
    entanglement: number;

    /** Quantum coherence (0-1) */
    coherence: number;

    /** Decoherence rate (1/s) */
    decoherence: number;
  };
}

// ============================================================================
// Journey Types
// ============================================================================

/**
 * Temporal trajectory
 */
export interface Trajectory {
  /** Waypoints along trajectory */
  waypoints: SpacetimeCoordinates[];

  /** Trajectory type */
  type: 'direct' | 'curved' | 'multi-hop';

  /** Total path length (meters) */
  spatialDistance: number;

  /** Temporal distance (seconds) */
  temporalDistance: number;

  /** Trajectory hash */
  hash: string;
}

/**
 * Traveler information
 */
export interface TravelerInfo {
  /** Unique traveler identifier */
  id: string;

  /** Public key for signatures */
  publicKey: string;

  /** Biometric data (hashed) */
  biometric: BiometricData;

  /** Traveler certificate */
  certificate?: Certificate;

  /** Registration date */
  registeredAt?: Date;

  /** Verified status */
  verified?: boolean;
}

/**
 * Journey record
 */
export interface JourneyRecord {
  /** Unique journey identifier */
  journeyId: string;

  /** Record version */
  version: string;

  /** Departure information */
  departure: {
    /** Departure time */
    time: Date;

    /** Spatial location */
    location: Vector3;

    /** Timeline identifier */
    timeline: string;
  };

  /** Arrival information */
  arrival: {
    /** Arrival time */
    time: Date;

    /** Spatial location */
    location: Vector3;

    /** Timeline identifier */
    timeline: string;
  };

  /** Traveler information */
  traveler: TravelerInfo;

  /** Journey duration (objective time, seconds) */
  duration: number;

  /** Energy signature */
  energy: EnergySignature;

  /** Temporal trajectory */
  trajectory: Trajectory;

  /** Journey logs */
  logs: JourneyLog[];

  /** Witness accounts */
  witnesses: WitnessAccount[];

  /** Beacon readings */
  beacons: BeaconReading[];

  /** Journey signature */
  signature: TemporalSignature;

  /** Counter-signatures */
  counterSignatures?: string[];

  /** Blockchain anchors */
  blockchainAnchors?: BlockchainAnchor[];

  /** Additional metadata */
  metadata?: Record<string, unknown>;
}

// ============================================================================
// Log Types
// ============================================================================

/**
 * Journey log event type
 */
export type LogEventType =
  | 'DEPARTURE'
  | 'ARRIVAL'
  | 'WAYPOINT'
  | 'OBSERVATION'
  | 'INTERACTION'
  | 'ANOMALY'
  | 'EMERGENCY';

/**
 * Beacon reading
 */
export interface BeaconReading {
  /** Beacon identifier */
  beaconId: string;

  /** Signal strength (0-1) */
  signalStrength: number;

  /** Time delay (seconds) */
  timeDelay: number;

  /** Reading timestamp */
  timestamp: Date;

  /** Calculated position */
  position?: Vector3;
}

/**
 * Journey log entry
 */
export interface JourneyLog {
  /** Log entry identifier */
  logId: string;

  /** Sequence number */
  sequence: number;

  /** Event timestamp */
  timestamp: Date;

  /** Event information */
  event: {
    /** Event type */
    type: LogEventType;

    /** Event description */
    description: string;

    /** Event location */
    location: Vector3;

    /** Timeline */
    timeline: string;

    /** Event-specific data */
    data: Record<string, any>;
  };

  /** SHA3-512 hash of this entry */
  hash: string;

  /** Hash of previous entry (blockchain style) */
  previousHash: string;

  /** Entry signature */
  signature: string;

  /** Witness identifiers */
  witnesses?: string[];

  /** Beacon readings at event time */
  beaconReadings?: BeaconReading[];

  /** Photograph IPFS hashes */
  photographs?: string[];
}

/**
 * Witness account
 */
export interface WitnessAccount {
  /** Witness identifier */
  witnessId: string;

  /** Witness name */
  name?: string;

  /** Witnessed event */
  event: {
    /** Event description */
    description: string;

    /** Event timestamp */
    timestamp: Date;

    /** Event location */
    location: Vector3;
  };

  /** Witness statement */
  statement: string;

  /** Witness signature */
  signature: string;

  /** Verification status */
  verified?: boolean;
}

// ============================================================================
// Blockchain Types
// ============================================================================

/**
 * Blockchain network
 */
export type BlockchainNetwork = 'ethereum' | 'hyperledger' | 'arweave' | 'ipfs';

/**
 * Blockchain anchor
 */
export interface BlockchainAnchor {
  /** Blockchain network */
  network: BlockchainNetwork;

  /** Transaction hash */
  txHash: string;

  /** Block number */
  blockNumber: number;

  /** Block timestamp */
  timestamp: Date;

  /** Merkle root of anchored data */
  merkleRoot: string;

  /** Journey identifier */
  journeyId: string;

  /** Traveler identifier */
  travelerId: string;

  /** Number of confirmations */
  confirmations: number;

  /** Verified status */
  verified: boolean;
}

// ============================================================================
// Event Types
// ============================================================================

/**
 * Historical event
 */
export interface HistoricalEvent {
  /** Event identifier */
  eventId: string;

  /** Event timestamp */
  timestamp: Date;

  /** Event location */
  location: Vector3;

  /** Event description */
  description: string;

  /** Event participants */
  participants: string[];

  /** Evidence supporting event */
  evidence: Evidence[];

  /** Event signature */
  signature: {
    /** Energy signature at event */
    energy: number;

    /** Number of participants */
    participants: number;

    /** Event duration (seconds) */
    duration: number;

    /** Historical impact (0-1) */
    impact: number;
  };

  /** Timeline identifier */
  timeline?: string;
}

/**
 * Evidence for event
 */
export interface Evidence {
  /** Evidence type */
  type: 'photograph' | 'video' | 'audio' | 'document' | 'physical' | 'witness';

  /** Evidence description */
  description: string;

  /** Evidence hash (IPFS or SHA3) */
  hash: string;

  /** Collection timestamp */
  collectedAt: Date;

  /** Collector identifier */
  collectedBy?: string;

  /** Verification status */
  verified?: boolean;
}

// ============================================================================
// Verification Types
// ============================================================================

/**
 * Verification confidence level
 */
export type ConfidenceLevel =
  | 'absolute'
  | 'very_high'
  | 'high'
  | 'moderate'
  | 'low'
  | 'very_low';

/**
 * Component verification result
 */
export interface ComponentResult {
  /** Component score (0-1) */
  score: number;

  /** Component weight */
  weight: number;

  /** Pass/fail status */
  passed: boolean;

  /** Result details */
  details: string;

  /** Sub-component scores */
  subComponents?: Record<string, number>;

  /** Detected anomalies */
  anomalies?: Anomaly[];
}

/**
 * Anomaly severity
 */
export type AnomalySeverity = 'CRITICAL' | 'HIGH' | 'MEDIUM' | 'LOW';

/**
 * Anomaly type
 */
export type AnomalyType =
  | 'SIGNATURE_MISMATCH'
  | 'TEMPORAL_ANOMALY'
  | 'IDENTITY_CONFLICT'
  | 'LOG_TAMPERING'
  | 'TIMELINE_INCONSISTENCY'
  | 'ENERGY_MISMATCH'
  | 'CERTIFICATE_INVALID'
  | 'BLOCKCHAIN_MISMATCH'
  | 'PARADOX_DETECTED';

/**
 * Anomaly detection result
 */
export interface Anomaly {
  /** Anomaly severity */
  severity: AnomalySeverity;

  /** Anomaly type */
  type: AnomalyType;

  /** Description */
  description: string;

  /** Detection timestamp */
  detectedAt: Date;

  /** Component where detected */
  component: string;

  /** Additional data */
  data?: Record<string, unknown>;
}

/**
 * Verification result
 */
export interface VerificationResult {
  /** Overall verification status */
  verified: boolean;

  /** Overall score (0-100) */
  score: number;

  /** Confidence level */
  confidence: number;

  /** Confidence level category */
  confidenceLevel: ConfidenceLevel;

  /** Component results */
  components: {
    /** Journey verification */
    journey: ComponentResult;

    /** Identity verification */
    identity: ComponentResult;

    /** Log verification */
    logs: ComponentResult;

    /** Signature verification */
    signature: ComponentResult;

    /** Consistency checking */
    consistency: ComponentResult;
  };

  /** Detected anomalies */
  anomalies: Anomaly[];

  /** Verification timestamp */
  timestamp: Date;

  /** Unique verification ID */
  verificationId: string;

  /** Verifier identity */
  verifier?: string;

  /** Failure reasons (if not verified) */
  reasons?: string[];
}

/**
 * Journey validation parameters
 */
export interface JourneyValidationParams {
  /** Journey record to validate */
  record: JourneyRecord;

  /** Strict mode (stricter thresholds) */
  strictMode?: boolean;

  /** Require blockchain anchoring */
  requireBlockchain?: boolean;

  /** Minimum confidence threshold */
  minimumConfidence?: number;

  /** Verification level */
  level?: VerificationLevel;

  /** Custom weights for components */
  weights?: {
    journey?: number;
    identity?: number;
    logs?: number;
    signature?: number;
    consistency?: number;
  };
}

// ============================================================================
// Audit Types
// ============================================================================

/**
 * Verification level
 */
export type VerificationLevel = 'basic' | 'standard' | 'comprehensive' | 'forensic';

/**
 * Audit trail
 */
export interface AuditTrail {
  /** Audit identifier */
  auditId: string;

  /** Verification identifier */
  verificationId: string;

  /** Journey identifier */
  journeyId: string;

  /** Traveler identifier */
  travelerId: string;

  /** Creation timestamp */
  createdAt: Date;

  /** Completion timestamp */
  completedAt?: Date;

  /** Component results */
  components: {
    journey: ComponentResult;
    identity: ComponentResult;
    logs: ComponentResult;
    signature: ComponentResult;
    consistency: ComponentResult;
  };

  /** Overall score (0-100) */
  overallScore: number;

  /** Confidence (0-1) */
  confidence: number;

  /** Verified status */
  verified: boolean;

  /** Detected anomalies */
  anomalies: Anomaly[];

  /** Witness accounts */
  witnesses: WitnessAccount[];

  /** Photograph hashes */
  photographs: string[];

  /** Beacon readings */
  beaconReadings: BeaconReading[];

  /** Blockchain anchors */
  blockchainAnchors: BlockchainAnchor[];

  /** Certificate ID (if generated) */
  certificateId?: string;

  /** Certificate URL */
  certificateUrl?: string;

  /** Auditor identity */
  auditor: string;

  /** Audit level */
  auditLevel: VerificationLevel;

  /** Additional notes */
  notes?: string;
}

/**
 * Forensic report
 */
export interface ForensicReport {
  /** Report identifier */
  reportId: string;

  /** Journey identifier */
  journeyId: string;

  /** Report start timestamp */
  startedAt: Date;

  /** Report completion timestamp */
  completedAt?: Date;

  /** Signature analysis */
  signatureAnalysis: {
    algorithm: SignatureAlgorithm;
    keyStrength: number;
    temporalBinding: number;
    quantumResistance: number;
    certificateChain: boolean;
  };

  /** Energy forensics */
  energyForensics: {
    profile: number[];
    anomalies: Anomaly[];
    source: string;
    efficiency: number;
  };

  /** Timeline forensics */
  timelineForensics: {
    consistency: number;
    alterations: Anomaly[];
    paradoxes: Anomaly[];
    butterflyEffects: number;
  };

  /** Identity forensics */
  identityForensics: {
    biometric: number;
    continuity: number;
    duplicates: boolean;
    history: TemporalHistory[];
  };

  /** Log forensics */
  logForensics: {
    integrity: number;
    tampering: Anomaly[];
    witnesses: number;
    correlation: number;
  };

  /** Overall assessment */
  assessment: {
    verified: boolean;
    confidence: number;
    riskLevel: 'low' | 'medium' | 'high' | 'critical';
    recommendation: string;
  };
}

/**
 * Temporal history entry
 */
export interface TemporalHistory {
  /** Journey identifier */
  journeyId: string;

  /** Timestamp */
  timestamp: Date;

  /** Traveler age at journey */
  age: number;

  /** Objective time */
  objectiveTime: Date;

  /** Location */
  location: Vector3;

  /** Timeline */
  timeline: string;
}

// ============================================================================
// Certificate Types
// ============================================================================

/**
 * Verification certificate
 */
export interface VerificationCertificate {
  /** Certificate identifier */
  certificateId: string;

  /** Certificate version */
  version: string;

  /** Journey identifier */
  journeyId: string;

  /** Traveler identifier */
  travelerId: string;

  /** Verification identifier */
  verificationId: string;

  /** Verification date */
  verificationDate: Date;

  /** Verified by */
  verifiedBy: string;

  /** Verification status */
  verified: boolean;

  /** Verification score (0-100) */
  score: number;

  /** Confidence (0-1) */
  confidence: number;

  /** Verification level */
  level: VerificationLevel;

  /** Journey summary */
  journey: {
    from: { time: Date; location: Vector3 };
    to: { time: Date; location: Vector3 };
    duration: number;
  };

  /** Component scores */
  components: {
    temporal: number;
    identity: number;
    logs: number;
    signature: number;
    consistency: number;
  };

  /** Blockchain proof */
  blockchain?: {
    network: string;
    txHash: string;
    blockNumber: number;
  };

  /** Valid from */
  validFrom: Date;

  /** Valid until */
  validUntil: Date;

  /** Issuer */
  issuer: string;

  /** Issuer signature */
  issuerSignature: string;

  /** QR code (base64) */
  qrCode: string;

  /** Public verification URL */
  verificationUrl: string;
}

// ============================================================================
// Consistency Types
// ============================================================================

/**
 * Timeline consistency result
 */
export interface ConsistencyResult {
  /** Consistency score (0-1) */
  score: number;

  /** Consistent status */
  consistent: boolean;

  /** Primary timeline event */
  primary: HistoricalEvent;

  /** Alternate timeline events */
  alternates: {
    timeline: string;
    consistency: number;
    event: HistoricalEvent;
  }[];

  /** Divergence analysis */
  divergence: {
    magnitude: number;
    timeline: string;
    cause?: string;
  }[];

  /** Failure reason (if inconsistent) */
  reason?: string;
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

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-TIME-025 error codes
 */
export enum VerificationErrorCode {
  SIGNATURE_INVALID = 'V001',
  IDENTITY_MISMATCH = 'V002',
  LOG_CORRUPTED = 'V003',
  TIMELINE_INCONSISTENT = 'V004',
  ENERGY_ANOMALY = 'V005',
  CERTIFICATE_EXPIRED = 'V006',
  BLOCKCHAIN_MISMATCH = 'V007',
  INSUFFICIENT_DATA = 'V008',
  PARADOX_DETECTED = 'V009',
  VERIFICATION_FAILED = 'V010',
}

/**
 * Verification error class
 */
export class VerificationError extends Error {
  constructor(
    public code: VerificationErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'VerificationError';
  }
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Verification constants
 */
export const VERIFICATION_CONSTANTS = {
  /** Default component weights */
  DEFAULT_WEIGHTS: {
    journey: 0.30,
    identity: 0.25,
    logs: 0.20,
    signature: 0.15,
    consistency: 0.10,
  },

  /** Confidence thresholds */
  CONFIDENCE_THRESHOLDS: {
    absolute: 0.98,
    very_high: 0.95,
    high: 0.90,
    moderate: 0.80,
    low: 0.70,
  },

  /** Minimum scores for pass */
  MINIMUM_SCORES: {
    basic: 0.70,
    standard: 0.80,
    comprehensive: 0.90,
    forensic: 0.95,
  },

  /** Certificate validity (seconds) */
  CERTIFICATE_VALIDITY: 315360000, // 10 years

  /** Maximum temporal nonce age (seconds) */
  MAX_NONCE_AGE: 31536000, // 1 year

  /** Minimum quantum entropy (bits) */
  MIN_QUANTUM_ENTROPY: 256,

  /** Energy tolerance */
  ENERGY_TOLERANCE: 0.10, // ±10%

  /** Temporal coordinate tolerance (seconds) */
  TEMPORAL_TOLERANCE: 1.0,

  /** Spatial coordinate tolerance (meters) */
  SPATIAL_TOLERANCE: 10.0,
} as const;

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core types
  Vector3,
  SpacetimeCoordinates,

  // Signature
  SignatureAlgorithm,
  QuantumNonce,
  TemporalSignature,
  Certificate,

  // Biometric
  BiometricHash,
  BiometricData,

  // Energy
  EnergySignature,

  // Journey
  Trajectory,
  TravelerInfo,
  JourneyRecord,

  // Logs
  LogEventType,
  BeaconReading,
  JourneyLog,
  WitnessAccount,

  // Blockchain
  BlockchainNetwork,
  BlockchainAnchor,

  // Events
  HistoricalEvent,
  Evidence,

  // Verification
  ConfidenceLevel,
  ComponentResult,
  AnomalySeverity,
  AnomalyType,
  Anomaly,
  VerificationResult,
  JourneyValidationParams,

  // Audit
  VerificationLevel,
  AuditTrail,
  ForensicReport,
  TemporalHistory,

  // Certificate
  VerificationCertificate,

  // Consistency
  ConsistencyResult,
};

export { VERIFICATION_CONSTANTS, VerificationErrorCode, VerificationError };
