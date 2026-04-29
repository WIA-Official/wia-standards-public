/**
 * WIA-TIME-033: Historical Archive - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Time Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Geometry and Time Types
// ============================================================================

/**
 * Geographic coordinates
 */
export interface Coordinates {
  lat: number;
  lon: number;
  alt?: number;
}

/**
 * Location information
 */
export interface Location {
  coordinates?: Coordinates;
  place?: string;
  accuracy?: number;
  planet?: string;
}

/**
 * Date range with precision
 */
export interface DateRange {
  start: Date | string;
  end?: Date | string;
  precision?: 'year' | 'month' | 'day' | 'hour' | 'minute' | 'second' | 'decade' | 'century';
}

// ============================================================================
// Timeline Types
// ============================================================================

/**
 * Timeline identifier
 */
export type TimelineID = string;

/**
 * Timeline metadata
 */
export interface Timeline {
  /** Unique timeline identifier */
  id: TimelineID;

  /** Timeline name */
  name: string;

  /** Parent timeline (if branch) */
  parent?: TimelineID;

  /** Creation timestamp */
  created: Date;

  /** Timeline status */
  status: 'active' | 'preserved' | 'merged' | 'archived';

  /** Total records in timeline */
  recordCount: number;

  /** Divergence point (if branch) */
  divergencePoint?: {
    date: Date;
    event: string;
    parentTimeline: TimelineID;
  };

  /** Timeline metadata */
  metadata?: Record<string, unknown>;
}

/**
 * Timeline version info
 */
export interface TimelineVersion {
  /** Timeline ID */
  timeline: TimelineID;

  /** Version number */
  version: string;

  /** Last modified */
  lastModified: Date;

  /** Total records */
  recordCount: number;

  /** Integrity hash */
  hash: string;
}

// ============================================================================
// Historical Record Types
// ============================================================================

/**
 * Event type categories
 */
export type EventType =
  | 'political'
  | 'military'
  | 'scientific'
  | 'cultural'
  | 'natural'
  | 'economic'
  | 'technological'
  | 'social'
  | 'other';

/**
 * Event importance level
 */
export type ImportanceLevel = 'critical' | 'high' | 'medium' | 'low';

/**
 * Event participant
 */
export interface Participant {
  name: string;
  role: string;
  verified?: boolean;
}

/**
 * Historical event data
 */
export interface Event {
  /** Event type */
  type: EventType;

  /** Event title */
  title: string;

  /** Detailed description */
  description: string;

  /** Event date/time */
  date: DateRange;

  /** Event location */
  location?: Location;

  /** Participants */
  participants?: Participant[];

  /** Event category */
  category?: string[];

  /** Impact assessment */
  impact?: {
    geographic?: 'global' | 'continental' | 'national' | 'regional' | 'local';
    temporal?: 'permanent' | 'long-term' | 'medium-term' | 'short-term';
    population?: number;
    economicImpact?: string;
    culturalSignificance?: number;
  };
}

/**
 * Evidence source types
 */
export type EvidenceType = 'document' | 'artifact' | 'testimony' | 'media' | 'scientific';

/**
 * Evidence source
 */
export interface EvidenceSource {
  /** Source type */
  type: EvidenceType;

  /** Source identifier */
  id: string;

  /** Content hash */
  hash: string;

  /** Storage location (IPFS, URL, etc.) */
  location: string;

  /** Confidence level (0-1) */
  confidence: number;

  /** Description */
  description?: string;
}

/**
 * Evidence collection
 */
export interface Evidence {
  /** Primary evidence sources */
  primary: string[];

  /** Secondary evidence sources */
  secondary: string[];

  /** Detailed sources */
  sources: EvidenceSource[];
}

/**
 * Verification method types
 */
export type VerificationMethod = 'multi-witness' | 'cryptographic' | 'scientific' | 'consensus';

/**
 * Verification data
 */
export interface Verification {
  /** Verification method used */
  method: VerificationMethod;

  /** Verifier identifiers */
  verifiers: string[];

  /** Overall confidence (0-1) */
  confidence: number;

  /** Verification timestamp */
  timestamp: Date;

  /** Digital signature */
  signature?: string;

  /** Verification notes */
  notes?: string;
}

/**
 * Record metadata
 */
export interface Metadata {
  /** Category tags */
  category?: string[];

  /** Importance level */
  importance?: ImportanceLevel;

  /** Historical period */
  period?: string;

  /** Era designation */
  era?: string;

  /** Century */
  century?: number;

  /** Decade */
  decade?: number;

  /** Geographic region */
  continent?: string;
  country?: string;
  region?: string;
  city?: string;

  /** Cultural context */
  culture?: string[];
  civilization?: string;
  language?: string;
  religion?: string[];

  /** Discovery info */
  discovered?: Date;
  discoverer?: string;
  recorded?: Date;
  recorder?: string;

  /** Keywords and tags */
  keywords?: string[];
  tags?: string[];

  /** Related records */
  related?: Array<{
    id: string;
    relationship: string;
  }>;

  /** Bibliography */
  bibliography?: string[];
  citations?: number;
  disputes?: string[];

  /** Custom metadata */
  custom?: Record<string, unknown>;
}

/**
 * Integrity data
 */
export interface IntegrityData {
  /** Record content hash */
  hash: string;

  /** Previous record hash (blockchain link) */
  previousHash: string;

  /** Block height in chain */
  blockHeight: number;

  /** Merkle tree root */
  merkleRoot?: string;

  /** Tamper-evident seal */
  sealed?: boolean;
}

/**
 * Complete historical record
 */
export interface HistoricalRecord {
  /** Unique record identifier */
  id: string;

  /** Timeline this record belongs to */
  timeline: TimelineID;

  /** Record version */
  version: string;

  /** Record creation timestamp */
  created: Date;

  /** Last modified timestamp */
  modified?: Date;

  /** Event data */
  event: Event;

  /** Evidence */
  evidence: Evidence;

  /** Verification status */
  verification: Verification;

  /** Metadata */
  metadata: Metadata;

  /** Integrity chain data */
  integrity: IntegrityData;

  /** Record status */
  status?: 'verified' | 'unverified' | 'disputed' | 'archived';
}

// ============================================================================
// Archive Types
// ============================================================================

/**
 * Archive storage configuration
 */
export interface ArchiveStorage {
  /** Storage type */
  type: 'distributed' | 'centralized' | 'hybrid';

  /** Storage nodes */
  nodes?: string[];

  /** Replication factor */
  replication: number;

  /** Encryption enabled */
  encryption?: boolean;

  /** Compression enabled */
  compression?: boolean;
}

/**
 * Archive configuration
 */
export interface ArchiveConfig {
  /** Archive identifier */
  archiveId: string;

  /** Default timeline */
  timeline?: TimelineID;

  /** Storage configuration */
  storage: ArchiveStorage;

  /** Access control */
  accessControl?: {
    enabled: boolean;
    defaultLevel: AccessLevel;
  };

  /** Metadata */
  metadata?: Record<string, unknown>;
}

/**
 * Archive entry (record with index info)
 */
export interface ArchiveEntry {
  /** The historical record */
  record: HistoricalRecord;

  /** Index position */
  index: number;

  /** Storage location */
  storageLocation: string[];

  /** Replication status */
  replicationStatus: {
    completed: number;
    required: number;
    nodes: string[];
  };
}

// ============================================================================
// Query Types
// ============================================================================

/**
 * Archive query parameters
 */
export interface ArchiveQuery {
  /** Timeline to query */
  timeline?: TimelineID;

  /** Date range filter */
  dateRange?: {
    start: Date | string;
    end?: Date | string;
  };

  /** Event type filter */
  eventType?: EventType | EventType[];

  /** Location filter */
  location?: {
    type?: 'circle' | 'rectangle' | 'polygon';
    center?: Coordinates;
    radius?: number;
    bounds?: {
      northeast: Coordinates;
      southwest: Coordinates;
    };
    points?: Coordinates[];
  };

  /** Importance filter */
  importance?: ImportanceLevel | ImportanceLevel[];

  /** Keyword search */
  keywords?: string[];

  /** Full-text search */
  text?: string;

  /** Verification status */
  verified?: boolean;

  /** Pagination */
  limit?: number;
  offset?: number;

  /** Sorting */
  sortBy?: 'date' | 'importance' | 'relevance';
  sortOrder?: 'asc' | 'desc';
}

/**
 * Query result
 */
export interface QueryResult {
  /** Total matching records */
  total: number;

  /** Returned records */
  returned: number;

  /** Current page */
  page: number;

  /** Records */
  records: HistoricalRecord[];

  /** Query execution time (ms) */
  executionTime?: number;
}

// ============================================================================
// Timeline Reconciliation Types
// ============================================================================

/**
 * Field difference in a record
 */
export interface FieldDifference {
  field: string;
  timeline1Value: unknown;
  timeline2Value: unknown;
}

/**
 * Modified record comparison
 */
export interface ModifiedRecord {
  timeline1: HistoricalRecord;
  timeline2: HistoricalRecord;
  changes: FieldDifference[];
}

/**
 * Timeline differences
 */
export interface TimelineDifference {
  /** Events added in timeline2 */
  eventsAdded: HistoricalRecord[];

  /** Events removed in timeline2 */
  eventsRemoved: HistoricalRecord[];

  /** Events modified between timelines */
  eventsModified: ModifiedRecord[];
}

/**
 * Divergence statistics
 */
export interface DivergenceStatistics {
  totalDifferences: number;
  significantDifferences: number;
  minorDifferences: number;
  divergenceSeverity: 'minor' | 'moderate' | 'major' | 'catastrophic';
}

/**
 * Reconciliation result
 */
export interface ReconciliationResult {
  /** Divergence point */
  divergencePoint: {
    date: Date;
    recordId: string;
    confidence: number;
  };

  /** Differences between timelines */
  differences: TimelineDifference;

  /** Statistics */
  statistics: DivergenceStatistics;

  /** Merged timeline (if applicable) */
  mergedTimeline?: Timeline;

  /** Reconciliation metadata */
  metadata?: Record<string, unknown>;
}

/**
 * Reconciliation strategy
 */
export type ReconciliationStrategy =
  | 'preserve-both'
  | 'merge'
  | 'prefer-primary'
  | 'consensus';

/**
 * Reconciliation parameters
 */
export interface ReconciliationParams {
  timeline1: TimelineID;
  timeline2: TimelineID;
  divergencePoint?: Date;
  strategy: ReconciliationStrategy;
  timelines?: TimelineID[];
}

// ============================================================================
// Alteration Types
// ============================================================================

/**
 * Alteration type
 */
export type AlterationType = 'event-prevention' | 'event-addition' | 'event-modification';

/**
 * Alteration severity
 */
export type AlterationSeverity = 'minor' | 'moderate' | 'major' | 'catastrophic';

/**
 * Timeline alteration log
 */
export interface AlterationLog {
  /** Alteration identifier */
  id: string;

  /** Timeline affected */
  timeline: TimelineID;

  /** Detection timestamp */
  detectedAt: Date;

  /** Divergence point */
  divergencePoint: {
    date: Date;
    event: string;
    originalOutcome?: string;
    alteredOutcome?: string;
  };

  /** Alteration type */
  alterationType: AlterationType;

  /** Severity assessment */
  severity: AlterationSeverity;

  /** Number of affected records */
  affectedRecords: number;

  /** Temporal energy signature */
  temporalEnergy?: number;

  /** Detection source */
  source: {
    method: string;
    detected: string;
    confidence: number;
  };

  /** Preservation info */
  preservation: {
    originalTimelineId: TimelineID;
    alteredTimelineId: TimelineID;
    preservationMethod: string;
    storageNodes: string[];
  };
}

// ============================================================================
// Access Control Types
// ============================================================================

/**
 * Access level
 */
export type AccessLevel = 'public' | 'researcher' | 'curator' | 'verifier' | 'admin';

/**
 * Permission
 */
export interface Permission {
  resource: string;
  actions: string[];
}

/**
 * Authentication credentials
 */
export interface AuthCredentials {
  userId: string;
  apiKey?: string;
  institution?: string;
  publicKey?: string;
}

/**
 * Authentication request
 */
export interface AuthRequest {
  credentials: AuthCredentials;
  accessLevel: AccessLevel;
  purpose: string;
  requestedPermissions?: string[];
}

/**
 * Authentication response
 */
export interface AuthResponse {
  sessionToken: string;
  expiresAt: Date;
  permissions: Permission[];
  rateLimit: {
    requestsPerHour: number;
    recordsPerDay: number;
  };
}

/**
 * Access log entry
 */
export interface AccessLog {
  timestamp: Date;
  userId: string;
  action: string;
  resource: string;
  parameters?: Record<string, unknown>;
  recordsAccessed?: number;
  ipAddress?: string;
  userAgent?: string;
}

// ============================================================================
// Research Access Types
// ============================================================================

/**
 * Research session
 */
export interface ResearchSession {
  /** Session identifier */
  sessionId: string;

  /** Researcher */
  researcher: {
    userId: string;
    name: string;
    institution?: string;
    credentials?: string[];
  };

  /** Research purpose */
  purpose: string;

  /** Access level */
  accessLevel: AccessLevel;

  /** Session start */
  startedAt: Date;

  /** Session end */
  expiresAt: Date;

  /** Records accessed */
  recordsAccessed: string[];

  /** Query history */
  queries: ArchiveQuery[];
}

/**
 * Archive export request
 */
export interface ExportRequest {
  /** Timeline to export */
  timeline: TimelineID;

  /** Date range */
  dateRange?: DateRange;

  /** Export format */
  format: 'json' | 'xml' | 'csv' | 'rdf';

  /** Include evidence */
  includeEvidence?: boolean;

  /** Include metadata */
  includeMetadata?: boolean;

  /** Compression */
  compress?: boolean;

  /** Requester info */
  requester: AuthCredentials;
}

/**
 * Export result
 */
export interface ExportResult {
  /** Export identifier */
  exportId: string;

  /** Download URL */
  downloadUrl: string;

  /** File size (bytes) */
  fileSize: number;

  /** Record count */
  recordCount: number;

  /** Created timestamp */
  createdAt: Date;

  /** Expires at */
  expiresAt: Date;
}

// ============================================================================
// Verification Types
// ============================================================================

/**
 * Verification request
 */
export interface VerificationRequest {
  /** Record to verify */
  recordId: string;

  /** Timeline */
  timeline: TimelineID;

  /** Verification method */
  method: VerificationMethod;

  /** Verifier credentials */
  verifier: AuthCredentials;

  /** Supporting evidence */
  evidence?: EvidenceSource[];

  /** Notes */
  notes?: string;
}

/**
 * Verification result
 */
export interface VerificationResult {
  /** Verification ID */
  verificationId: string;

  /** Record ID */
  recordId: string;

  /** Verification status */
  verified: boolean;

  /** Confidence score (0-1) */
  confidence: number;

  /** Verification timestamp */
  timestamp: Date;

  /** Verifier */
  verifier: string;

  /** Details */
  details?: string;
}

/**
 * Integrity check result
 */
export interface IntegrityCheck {
  /** Record ID */
  recordId: string;

  /** Integrity valid */
  valid: boolean;

  /** Hash matches */
  hashValid: boolean;

  /** Chain intact */
  chainIntact: boolean;

  /** Signature valid */
  signatureValid?: boolean;

  /** Issues found */
  issues?: string[];

  /** Check timestamp */
  timestamp: Date;
}

// ============================================================================
// Physical Constants and Configuration
// ============================================================================

/**
 * Archive constants
 */
export const ARCHIVE_CONSTANTS = {
  /** Default storage format */
  STORAGE_FORMAT: 'json-ld',

  /** Hash algorithm */
  HASH_ALGORITHM: 'SHA3-512',

  /** Encryption algorithm */
  ENCRYPTION_ALGORITHM: 'AES-256-GCM',

  /** Minimum replication factor */
  MIN_REPLICATION: 3,

  /** Maximum replication factor */
  MAX_REPLICATION: 7,

  /** Default compression algorithm */
  COMPRESSION: 'brotli',

  /** Default compression level (1-11) */
  COMPRESSION_LEVEL: 11,

  /** Verification confidence threshold */
  VERIFICATION_THRESHOLD: 0.95,

  /** Query limits */
  QUERY_LIMITS: {
    PUBLIC: { requestsPerHour: 100, recordsPerDay: 1000 },
    RESEARCHER: { requestsPerHour: 1000, recordsPerDay: 10000 },
    CURATOR: { requestsPerHour: 5000, recordsPerDay: 50000 },
    ADMIN: { requestsPerHour: 10000, recordsPerDay: 100000 },
  },

  /** Session timeout (milliseconds) */
  SESSION_TIMEOUT: 3600000, // 1 hour
} as const;

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
 * Archive error codes
 */
export enum ArchiveErrorCode {
  RECORD_NOT_FOUND = 'A001',
  TIMELINE_NOT_FOUND = 'A002',
  VERIFICATION_FAILED = 'A003',
  INTEGRITY_VIOLATION = 'A004',
  ACCESS_DENIED = 'A005',
  INVALID_QUERY = 'A006',
  STORAGE_ERROR = 'A007',
  REPLICATION_FAILED = 'A008',
  RECONCILIATION_ERROR = 'A009',
  ALTERATION_DETECTED = 'A010',
}

/**
 * Archive error class
 */
export class ArchiveError extends Error {
  constructor(
    public code: ArchiveErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'ArchiveError';
  }
}

// ============================================================================
// Export All Types
// ============================================================================

export type {
  Coordinates,
  Location,
  DateRange,
  TimelineID,
  Timeline,
  TimelineVersion,
  EventType,
  ImportanceLevel,
  Participant,
  Event,
  EvidenceType,
  EvidenceSource,
  Evidence,
  VerificationMethod,
  Verification,
  Metadata,
  IntegrityData,
  HistoricalRecord,
  ArchiveStorage,
  ArchiveConfig,
  ArchiveEntry,
  ArchiveQuery,
  QueryResult,
  FieldDifference,
  ModifiedRecord,
  TimelineDifference,
  DivergenceStatistics,
  ReconciliationResult,
  ReconciliationStrategy,
  ReconciliationParams,
  AlterationType,
  AlterationSeverity,
  AlterationLog,
  AccessLevel,
  Permission,
  AuthCredentials,
  AuthRequest,
  AuthResponse,
  AccessLog,
  ResearchSession,
  ExportRequest,
  ExportResult,
  VerificationRequest,
  VerificationResult,
  IntegrityCheck,
};

export { ARCHIVE_CONSTANTS, ArchiveErrorCode, ArchiveError };
