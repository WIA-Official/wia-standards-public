/**
 * WIA-TIME-006: Universal Time Database - TypeScript Type Definitions
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
 * Three-dimensional spatial vector
 */
export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

/**
 * Universal Temporal Coordinate Plus (UTC+)
 */
export interface UniversalTemporalCoordinate {
  /** Timestamp in nanoseconds since epoch (2000-01-01T00:00:00.000000000Z) */
  timestamp: bigint;

  /** Universe identifier */
  universeId: string;

  /** Timeline branch identifier */
  timelineId: string;

  /** Spatial coordinates (meters, ECI system) */
  position: Vector3;

  /** Branch point reference (optional) */
  branchPoint?: string;
}

/**
 * Temporal event stored in the database
 */
export interface TemporalEvent {
  /** Unique event identifier */
  eventId: string;

  /** Universal temporal coordinate */
  coordinate: UniversalTemporalCoordinate;

  /** Event type/classification */
  eventType: string;

  /** Event data payload */
  data: Record<string, unknown>;

  /** Causality chain (IDs of causal predecessors) */
  causalityChain: string[];

  /** Event significance (0-1, higher = more important) */
  significance: number;

  /** Whether this event can be modified */
  mutable: boolean;

  /** Event metadata */
  metadata?: {
    /** When this event was inserted into DB */
    insertedAt?: Date;

    /** Who/what created this event */
    creator?: string;

    /** Additional tags */
    tags?: string[];

    /** Custom metadata */
    [key: string]: unknown;
  };
}

/**
 * Timeline representation
 */
export interface Timeline {
  /** Timeline identifier */
  timelineId: string;

  /** Universe this timeline belongs to */
  universeId: string;

  /** Parent timeline (if this is a branch) */
  parentTimeline?: string;

  /** Point where this timeline branched from parent */
  branchPoint?: bigint;

  /** Divergence factor from parent timeline (0-1) */
  divergenceFactor: number;

  /** Timeline integrity score (0-1) */
  integrityScore: number;

  /** Number of events in this timeline */
  eventCount: number;

  /** When this timeline was created */
  createdAt: Date;

  /** Timeline description */
  description?: string;

  /** Timeline status */
  status: 'active' | 'frozen' | 'archived' | 'merged';

  /** Timeline metadata */
  metadata?: Record<string, unknown>;
}

/**
 * Universe representation
 */
export interface Universe {
  /** Universe identifier */
  universeId: string;

  /** Universe type */
  type: 'prime' | 'parallel' | 'quantum' | 'simulated' | 'custom';

  /** Number of timelines in this universe */
  timelineCount: number;

  /** Physical constants (may differ from our universe) */
  physicalConstants?: {
    speedOfLight?: number;
    gravitationalConstant?: number;
    planckConstant?: number;
    [key: string]: number | undefined;
  };

  /** Universe creation timestamp */
  createdAt: Date;

  /** Universe description */
  description?: string;
}

// ============================================================================
// Query Types
// ============================================================================

/**
 * Time range for queries
 */
export interface TimeRange {
  /** Start timestamp (inclusive) */
  start: Date | bigint | string;

  /** End timestamp (inclusive) */
  end: Date | bigint | string;

  /** Tolerance for time matching (nanoseconds) */
  tolerance?: bigint;
}

/**
 * Spatial region for queries
 */
export interface SpatialRegion {
  /** Bounding box query */
  box?: {
    min: Vector3;
    max: Vector3;
  };

  /** Spherical query */
  sphere?: {
    center: Vector3;
    radius: number;
  };

  /** Point query with tolerance */
  point?: {
    position: Vector3;
    tolerance: number;
  };
}

/**
 * Query parameters for event queries
 */
export interface QueryParams {
  /** Timeline(s) to query */
  timeline?: string | string[];

  /** Universe to query */
  universe?: string;

  /** Time range filter */
  timeRange?: TimeRange;

  /** Spatial filter */
  spatial?: SpatialRegion;

  /** Event type filter */
  eventType?: string | string[];

  /** Minimum significance filter */
  minSignificance?: number;

  /** Only mutable events */
  mutableOnly?: boolean;

  /** Causality filters */
  causality?: {
    /** Events that cause this event ID */
    causes?: string;

    /** Events caused by this event ID */
    causedBy?: string;

    /** Events in causal chain between two events */
    between?: [string, string];
  };

  /** Ordering */
  orderBy?: {
    field: 'timestamp' | 'significance' | 'eventId';
    direction: 'asc' | 'desc';
  };

  /** Pagination */
  limit?: number;
  offset?: number;
  cursor?: string;
}

/**
 * Query result
 */
export interface QueryResult {
  /** Matching events */
  events: TemporalEvent[];

  /** Total count (may be more than returned) */
  totalCount: number;

  /** Next page cursor */
  nextCursor?: string;

  /** Query execution time (ms) */
  executionTime: number;

  /** Query metadata */
  metadata?: {
    scannedEvents?: number;
    indexUsed?: string[];
    [key: string]: unknown;
  };
}

/**
 * Fluent query builder
 */
export interface QueryBuilder {
  /** Filter by universe */
  universe(universeId: string): QueryBuilder;

  /** Filter by timeline(s) */
  timeline(...timelineIds: string[]): QueryBuilder;
  timelines(timelineIds: string[]): QueryBuilder;

  /** Filter by time range */
  between(start: Date | string, end: Date | string): QueryBuilder;
  at(timestamp: Date | string, tolerance?: number): QueryBuilder;
  before(timestamp: Date | string): QueryBuilder;
  after(timestamp: Date | string): QueryBuilder;

  /** Filter by spatial region */
  within(region: SpatialRegion): QueryBuilder;
  near(position: Vector3, radius: number): QueryBuilder;

  /** Filter by event type */
  type(eventType: string): QueryBuilder;
  types(...eventTypes: string[]): QueryBuilder;

  /** Filter by significance */
  significanceGreaterThan(value: number): QueryBuilder;
  significanceLessThan(value: number): QueryBuilder;

  /** Filter by causality */
  causes(eventId: string): QueryBuilder;
  causedBy(eventId: string): QueryBuilder;
  causalChainBetween(start: string, end: string): QueryBuilder;

  /** Generic where clause */
  where(field: string, operator: string, value: unknown): QueryBuilder;

  /** Ordering */
  orderBy(field: string, direction?: 'asc' | 'desc'): QueryBuilder;

  /** Pagination */
  limit(count: number): QueryBuilder;
  offset(count: number): QueryBuilder;

  /** Execute query */
  execute(): Promise<QueryResult>;

  /** Execute and return first result */
  first(): Promise<TemporalEvent | null>;

  /** Execute and return count */
  count(): Promise<number>;
}

// ============================================================================
// Timeline Operations
// ============================================================================

/**
 * Parameters for creating a timeline branch
 */
export interface BranchParams {
  /** Source timeline to branch from */
  from: string;

  /** Name of new timeline */
  name: string;

  /** Point in time where branch occurs */
  branchPoint: Date | bigint | string;

  /** Type of divergence */
  divergenceType: 'quantum-split' | 'deliberate' | 'natural' | 'simulation';

  /** Description of branch */
  description?: string;

  /** Initial divergence factor */
  initialDivergence?: number;
}

/**
 * Result of branch creation
 */
export interface BranchResult {
  /** New timeline ID */
  timelineId: string;

  /** Branch point timestamp */
  branchPoint: bigint;

  /** Number of events copied */
  eventsCopied: number;

  /** Creation timestamp */
  createdAt: Date;
}

/**
 * Parameters for merging timelines
 */
export interface MergeParams {
  /** Source timeline */
  source: string;

  /** Target timeline to merge into */
  target: string;

  /** Merge strategy */
  strategy: 'three-way-merge' | 'fast-forward' | 'squash' | 'rebase';

  /** Conflict resolution strategy */
  conflictResolution: 'manual' | 'source-wins' | 'target-wins' | 'merge';

  /** Merge message */
  message?: string;
}

/**
 * Result of timeline merge
 */
export interface MergeResult {
  /** Success status */
  success: boolean;

  /** Number of events merged */
  eventsMerged: number;

  /** Conflicts detected */
  conflicts: MergeConflict[];

  /** Merge commit ID */
  commitId?: string;

  /** Error message if failed */
  error?: string;
}

/**
 * Merge conflict
 */
export interface MergeConflict {
  /** Event ID with conflict */
  eventId: string;

  /** Source event data */
  sourceData: Record<string, unknown>;

  /** Target event data */
  targetData: Record<string, unknown>;

  /** Conflict type */
  type: 'data-mismatch' | 'causality-violation' | 'temporal-paradox';

  /** Suggested resolution */
  suggestion?: 'keep-source' | 'keep-target' | 'merge-both' | 'manual';
}

/**
 * Timeline commit
 */
export interface TimelineCommit {
  /** Commit ID */
  commitId: string;

  /** Timeline ID */
  timelineId: string;

  /** Parent commit ID */
  parentCommit?: string;

  /** Commit timestamp */
  timestamp: bigint;

  /** Events added in this commit */
  eventsAdded: string[];

  /** Events modified in this commit */
  eventsModified: string[];

  /** Events deleted in this commit */
  eventsDeleted: string[];

  /** Commit author */
  author: string;

  /** Commit message */
  message: string;

  /** Commit checksum */
  checksum: string;
}

// ============================================================================
// Synchronization
// ============================================================================

/**
 * Synchronization parameters
 */
export interface SyncParams {
  /** Source universe:timeline */
  source: {
    universe: string;
    timeline: string;
  };

  /** Target universe:timeline */
  target: {
    universe: string;
    timeline: string;
  };

  /** Sync mode */
  mode: 'full' | 'incremental' | 'differential' | 'selective';

  /** Time range to sync (for selective sync) */
  timeRange?: TimeRange;

  /** Event filter (for selective sync) */
  filter?: Partial<QueryParams>;

  /** Conflict resolution */
  conflictResolution?: 'last-write-wins' | 'merge' | 'vector-clock' | 'manual';

  /** Use quantum entanglement for instant sync */
  useQuantumSync?: boolean;
}

/**
 * Synchronization result
 */
export interface SyncResult {
  /** Sync operation ID */
  syncId: string;

  /** Success status */
  success: boolean;

  /** Events synchronized */
  eventsSynced: number;

  /** Bytes transferred */
  bytesTransferred: number;

  /** Sync duration (ms) */
  duration: number;

  /** Conflicts encountered */
  conflicts: SyncConflict[];

  /** Sync checksum */
  checksum: string;

  /** Error message if failed */
  error?: string;
}

/**
 * Synchronization conflict
 */
export interface SyncConflict {
  /** Event ID with conflict */
  eventId: string;

  /** Source version */
  sourceVersion: TemporalEvent;

  /** Target version */
  targetVersion: TemporalEvent;

  /** Conflict resolution applied */
  resolution: 'source' | 'target' | 'merged' | 'pending';

  /** Resolved event (if resolved) */
  resolvedEvent?: TemporalEvent;
}

// ============================================================================
// Causality
// ============================================================================

/**
 * Parameters for causality chain query
 */
export interface CausalChainParams {
  /** Starting event ID */
  eventId: string;

  /** Direction to traverse */
  direction: 'forward' | 'backward' | 'both';

  /** Maximum depth to traverse */
  maxDepth?: number;

  /** Include indirect causality */
  includeIndirect?: boolean;
}

/**
 * Causal relationship
 */
export interface CausalRelationship {
  /** Cause event ID */
  cause: string;

  /** Effect event ID */
  effect: string;

  /** Causal strength (0-1) */
  strength: number;

  /** Time delta (effect - cause) in nanoseconds */
  timeDelta: bigint;

  /** Relationship type */
  type: 'direct' | 'indirect' | 'quantum-entangled';
}

/**
 * Causality graph
 */
export interface CausalityGraph {
  /** Events in graph */
  events: TemporalEvent[];

  /** Relationships between events */
  relationships: CausalRelationship[];

  /** Graph metadata */
  metadata: {
    /** Total nodes */
    nodeCount: number;

    /** Total edges */
    edgeCount: number;

    /** Maximum depth */
    maxDepth: number;

    /** Graph is acyclic */
    isAcyclic: boolean;
  };
}

// ============================================================================
// Analytics
// ============================================================================

/**
 * Timeline divergence analysis
 */
export interface DivergenceAnalysis {
  /** Timeline A */
  timelineA: string;

  /** Timeline B */
  timelineB: string;

  /** Divergence factor (0-1) */
  divergenceFactor: number;

  /** Branch point timestamp */
  branchPoint: bigint;

  /** Common events */
  commonEvents: number;

  /** Events only in A */
  eventsOnlyInA: number;

  /** Events only in B */
  eventsOnlyInB: number;

  /** Key divergence points */
  keyDivergences: Array<{
    timestamp: bigint;
    eventA?: TemporalEvent;
    eventB?: TemporalEvent;
    description: string;
  }>;
}

/**
 * Temporal density analysis
 */
export interface TemporalDensity {
  /** Timeline ID */
  timelineId: string;

  /** Time range analyzed */
  timeRange: TimeRange;

  /** Events per unit time */
  eventsPerSecond: number;

  /** Density histogram */
  histogram: Array<{
    timestamp: bigint;
    eventCount: number;
  }>;

  /** Peak density */
  peakDensity: {
    timestamp: bigint;
    eventCount: number;
  };
}

/**
 * Event significance distribution
 */
export interface SignificanceDistribution {
  /** Timeline ID */
  timelineId: string;

  /** Total events analyzed */
  totalEvents: number;

  /** Distribution buckets */
  buckets: Array<{
    /** Significance range [min, max) */
    range: [number, number];

    /** Event count in this bucket */
    count: number;

    /** Percentage of total */
    percentage: number;
  }>;

  /** Average significance */
  average: number;

  /** Median significance */
  median: number;
}

// ============================================================================
// Database Configuration
// ============================================================================

/**
 * Database configuration
 */
export interface DBConfig {
  /** Universe to connect to */
  universe: string;

  /** Default timeline */
  timeline: string;

  /** Storage configuration */
  storage: {
    /** Storage type */
    type: 'local' | 'distributed' | 'quantum-distributed';

    /** Storage nodes */
    nodes?: string[];

    /** Replication factor */
    replicationFactor?: number;

    /** Consistency level */
    consistency?: 'strong' | 'causal' | 'timeline' | 'eventual';
  };

  /** Connection settings */
  connection?: {
    /** Connection timeout (ms) */
    timeout?: number;

    /** Max retries */
    maxRetries?: number;

    /** Connection pool size */
    poolSize?: number;
  };

  /** Authentication */
  auth?: {
    /** Auth method */
    method: 'jwt' | 'oauth' | 'quantum-key' | 'mfa';

    /** Credentials */
    credentials: Record<string, string>;
  };

  /** Cache settings */
  cache?: {
    /** Enable caching */
    enabled: boolean;

    /** Cache size (MB) */
    size?: number;

    /** TTL (seconds) */
    ttl?: number;
  };
}

/**
 * Database statistics
 */
export interface DBStats {
  /** Total events across all timelines */
  totalEvents: bigint;

  /** Total timelines */
  totalTimelines: number;

  /** Total universes */
  totalUniverses: number;

  /** Storage used (bytes) */
  storageUsed: bigint;

  /** Index size (bytes) */
  indexSize: bigint;

  /** Compression ratio */
  compressionRatio: number;

  /** Performance stats */
  performance: {
    /** Queries per second */
    queriesPerSecond: number;

    /** Inserts per second */
    insertsPerSecond: number;

    /** Average query latency (ms) */
    avgQueryLatency: number;

    /** Average insert latency (ms) */
    avgInsertLatency: number;
  };
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-TIME-006 error codes
 */
export enum TimeDBErrorCode {
  // Connection errors
  CONNECTION_FAILED = 'TDB001',
  AUTHENTICATION_FAILED = 'TDB002',
  TIMEOUT = 'TDB003',

  // Data errors
  EVENT_NOT_FOUND = 'TDB101',
  TIMELINE_NOT_FOUND = 'TDB102',
  UNIVERSE_NOT_FOUND = 'TDB103',
  INVALID_COORDINATE = 'TDB104',
  INVALID_QUERY = 'TDB105',

  // Consistency errors
  CAUSALITY_VIOLATION = 'TDB201',
  TEMPORAL_PARADOX = 'TDB202',
  INTEGRITY_ERROR = 'TDB203',
  CONFLICT_DETECTED = 'TDB204',

  // Operation errors
  BRANCH_FAILED = 'TDB301',
  MERGE_FAILED = 'TDB302',
  SYNC_FAILED = 'TDB303',
  REPLICATION_FAILED = 'TDB304',

  // Resource errors
  STORAGE_FULL = 'TDB401',
  QUOTA_EXCEEDED = 'TDB402',
  RATE_LIMITED = 'TDB403',
}

/**
 * Time database error
 */
export class TimeDBError extends Error {
  constructor(
    public code: TimeDBErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'TimeDBError';
  }
}

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Result type for operations
 */
export type Result<T, E = Error> =
  | { success: true; data: T }
  | { success: false; error: E };

/**
 * Async result type
 */
export type AsyncResult<T, E = Error> = Promise<Result<T, E>>;

/**
 * Paginated result
 */
export interface PaginatedResult<T> {
  /** Result items */
  items: T[];

  /** Total count */
  totalCount: number;

  /** Current page */
  page: number;

  /** Items per page */
  pageSize: number;

  /** Has more pages */
  hasMore: boolean;

  /** Next page cursor */
  nextCursor?: string;
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Default configuration values
 */
export const DEFAULT_CONFIG = {
  /** Default replication factor */
  REPLICATION_FACTOR: 3,

  /** Default consistency level */
  CONSISTENCY_LEVEL: 'causal' as const,

  /** Default query limit */
  QUERY_LIMIT: 1000,

  /** Default connection timeout (ms) */
  CONNECTION_TIMEOUT: 30000,

  /** Default cache TTL (seconds) */
  CACHE_TTL: 3600,

  /** Default time tolerance (nanoseconds) */
  TIME_TOLERANCE: BigInt(1000000), // 1ms
} as const;

/**
 * Database limits
 */
export const DB_LIMITS = {
  /** Max events per query */
  MAX_QUERY_EVENTS: 1000000,

  /** Max timeline branches */
  MAX_BRANCHES: 10000,

  /** Max causality depth */
  MAX_CAUSALITY_DEPTH: 100,

  /** Max sync batch size */
  MAX_SYNC_BATCH: 10000000,
} as const;

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core types
  Vector3,
  UniversalTemporalCoordinate,
  TemporalEvent,
  Timeline,
  Universe,

  // Query types
  TimeRange,
  SpatialRegion,
  QueryParams,
  QueryResult,
  QueryBuilder,

  // Timeline operations
  BranchParams,
  BranchResult,
  MergeParams,
  MergeResult,
  MergeConflict,
  TimelineCommit,

  // Synchronization
  SyncParams,
  SyncResult,
  SyncConflict,

  // Causality
  CausalChainParams,
  CausalRelationship,
  CausalityGraph,

  // Analytics
  DivergenceAnalysis,
  TemporalDensity,
  SignificanceDistribution,

  // Configuration
  DBConfig,
  DBStats,

  // Utility types
  Result,
  AsyncResult,
  PaginatedResult,
};

export { TimeDBErrorCode, TimeDBError, DEFAULT_CONFIG, DB_LIMITS };
