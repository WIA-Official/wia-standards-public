/**
 * WIA-TIME-019: Timeline Synchronization - TypeScript Type Definitions
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
 * Universal Time Reference (UTR)
 */
export interface UniversalTimeReference {
  /** Reference timestamp in nanoseconds since epoch (2000-01-01T00:00:00.000000000Z) */
  referenceTime: bigint;

  /** Timeline phase vector (complex number representing timeline position) */
  phaseVector: {
    /** Amplitude (timeline "distance" from reference) */
    amplitude: number;

    /** Phase angle in radians (temporal offset) */
    phase: number;
  };

  /** Drift correction factor (ns/s) */
  driftCorrection: number;

  /** Synchronization confidence score (0.0-1.0) */
  confidence: number;

  /** When this UTR was established */
  establishedAt: Date;

  /** Source timeline for reference */
  referenceTimeline: string;
}

/**
 * Timeline clock state
 */
export interface ClockState {
  /** Timeline identifier */
  timelineId: string;

  /** Clock identifier */
  clockId: string;

  /** Local time in nanoseconds */
  localTime: bigint;

  /** Reference time in nanoseconds */
  referenceTime: bigint;

  /** Clock offset (local - reference) in nanoseconds */
  offset: bigint;

  /** Drift rate in nanoseconds per second */
  driftRate: number;

  /** Drift acceleration in nanoseconds per second squared */
  driftAcceleration: number;

  /** Timestamp of last sync */
  lastSync: bigint;

  /** Sync quality score (0.0-1.0) */
  syncQuality: number;

  /** Synchronization method used */
  syncMethod: 'cristian' | 'berkeley' | 'ntp' | 'quantum' | 'custom';

  /** Total number of syncs performed */
  totalSyncs: bigint;

  /** Number of failed sync attempts */
  failedSyncs: bigint;

  /** Average offset over recent syncs */
  averageOffset: number;

  /** Clock metadata */
  metadata?: Record<string, unknown>;
}

/**
 * Temporal drift measurement
 */
export interface TemporalDrift {
  /** Timeline experiencing drift */
  timelineId: string;

  /** Reference timeline */
  referenceTimeline: string;

  /** Drift rate (nanoseconds per second) */
  rate: number;

  /** Total accumulated drift (nanoseconds) */
  total: bigint;

  /** Drift direction */
  direction: 'ahead' | 'behind';

  /** Confidence in measurement (0.0-1.0) */
  confidence: number;

  /** When drift was measured */
  measuredAt: Date;

  /** Measurement duration */
  measurementDuration: number;

  /** Drift trend */
  trend: 'increasing' | 'decreasing' | 'stable';
}

/**
 * Timeline divergence information
 */
export interface TimelineDivergence {
  /** First timeline */
  timelineA: string;

  /** Second timeline */
  timelineB: string;

  /** Divergence detected */
  detected: boolean;

  /** Point where divergence began (nanoseconds) */
  divergencePoint: bigint;

  /** Magnitude of divergence (0.0-1.0) */
  magnitude: number;

  /** Type of divergence */
  type: 'quantum-split' | 'deliberate' | 'natural' | 'induced' | 'paradox' | 'simulation' | 'error';

  /** Severity level */
  severity: 'negligible' | 'minor' | 'moderate' | 'major' | 'critical' | 'catastrophic';

  /** Detailed metrics */
  metrics: {
    /** Event-based divergence (0.0-1.0) */
    eventDivergence: number;

    /** Data-based divergence (0.0-1.0) */
    dataDivergence: number;

    /** Causal divergence (0.0-1.0) */
    causalDivergence: number;

    /** Temporal divergence (nanoseconds) */
    temporalDivergence: bigint;
  };

  /** Recommended action */
  recommendation: 'ignore' | 'monitor' | 'sync' | 'merge' | 'isolate' | 'investigate';

  /** When divergence was detected */
  detectedAt: Date;
}

// ============================================================================
// Synchronization Types
// ============================================================================

/**
 * Synchronization configuration
 */
export interface SyncConfig {
  /** Reference timeline for synchronization */
  referenceTimeline: string;

  /** Synchronization mode */
  syncMode: 'manual' | 'periodic' | 'continuous' | 'event-driven';

  /** Time precision level */
  precision: 'second' | 'millisecond' | 'microsecond' | 'nanosecond' | 'picosecond';

  /** Drift tolerance (nanoseconds) */
  driftTolerance: number;

  /** Sync interval for periodic mode (milliseconds) */
  syncInterval?: number;

  /** Conflict resolution strategy */
  conflictStrategy: 'manual' | 'auto' | 'source-wins' | 'target-wins' | 'merge' | 'crdt';

  /** Use quantum entanglement for instant sync */
  useQuantumSync: boolean;

  /** Preserve causal consistency */
  preserveCausality: boolean;

  /** Maximum retry attempts */
  maxRetries: number;

  /** Operation timeout (milliseconds) */
  timeout: number;

  /** Custom sync algorithm */
  customAlgorithm?: string;
}

/**
 * Synchronization request parameters
 */
export interface SyncRequest {
  /** Timeline to synchronize */
  timelineId: string;

  /** Synchronization strategy */
  strategy: 'clock-sync' | 'event-sync' | 'state-sync' | 'full-sync';

  /** Apply drift correction */
  correctDrift: boolean;

  /** Merge timelines on conflict */
  mergeOnConflict: boolean;

  /** Time range to sync (optional) */
  timeRange?: {
    start: Date | bigint | string;
    end: Date | bigint | string;
  };

  /** Event filter for selective sync */
  eventFilter?: {
    types?: string[];
    minSignificance?: number;
  };

  /** Priority level */
  priority: 'low' | 'normal' | 'high' | 'critical';

  /** Request metadata */
  metadata?: Record<string, unknown>;
}

/**
 * Synchronization result
 */
export interface SyncResult {
  /** Sync operation ID */
  syncId: string;

  /** Sync status */
  status: 'success' | 'partial' | 'failed' | 'timeout';

  /** Timeline synchronized */
  timelineId: string;

  /** Drift corrected (nanoseconds) */
  driftCorrected: bigint;

  /** Clock offset after sync (nanoseconds) */
  offset: bigint;

  /** Sync accuracy (0.0-1.0) */
  accuracy: number;

  /** Sync quality (0.0-1.0) */
  quality: number;

  /** Round-trip time (nanoseconds) */
  roundTripTime: bigint;

  /** Algorithm used */
  algorithm: string;

  /** Number of samples */
  samples: number;

  /** Events synchronized */
  eventsSynced: number;

  /** Conflicts encountered */
  conflicts: SyncConflict[];

  /** Sync duration (milliseconds) */
  duration: number;

  /** When sync completed */
  completedAt: Date;

  /** Error message if failed */
  error?: string;

  /** Result metadata */
  metadata?: Record<string, unknown>;
}

/**
 * Synchronization conflict
 */
export interface SyncConflict {
  /** Conflict ID */
  conflictId: string;

  /** Event or data ID with conflict */
  itemId: string;

  /** Conflict type */
  type: 'data' | 'causality' | 'temporal' | 'structural';

  /** Severity */
  severity: 'minor' | 'major' | 'critical';

  /** Source timeline value */
  sourceValue: unknown;

  /** Target timeline value */
  targetValue: unknown;

  /** Common ancestor value (if available) */
  ancestorValue?: unknown;

  /** Resolution applied */
  resolution: 'source' | 'target' | 'merged' | 'manual' | 'pending';

  /** Resolved value */
  resolvedValue?: unknown;

  /** Can be auto-resolved */
  autoResolvable: boolean;

  /** Suggested resolution */
  suggestion?: string;

  /** Conflict description */
  description: string;
}

// ============================================================================
// Drift Correction Types
// ============================================================================

/**
 * Drift correction parameters
 */
export interface DriftCorrectionParams {
  /** Timeline to correct */
  timelineId: string;

  /** Correction method */
  method: 'immediate' | 'gradual' | 'proportional' | 'pid';

  /** Maximum single adjustment (nanoseconds) */
  maxAdjustment: bigint;

  /** Correction duration for gradual method (milliseconds) */
  duration?: number;

  /** PID controller parameters */
  pidParams?: {
    kp: number; // Proportional gain
    ki: number; // Integral gain
    kd: number; // Derivative gain
  };

  /** Verify after correction */
  verify: boolean;
}

/**
 * Drift correction result
 */
export interface DriftCorrectionResult {
  /** Correction ID */
  correctionId: string;

  /** Status */
  status: 'success' | 'partial' | 'failed';

  /** Timeline corrected */
  timelineId: string;

  /** Adjustment applied (nanoseconds) */
  adjustment: bigint;

  /** New drift rate (ns/s) */
  newDriftRate: number;

  /** Drift before correction (nanoseconds) */
  driftBefore: bigint;

  /** Drift after correction (nanoseconds) */
  driftAfter: bigint;

  /** Improvement percentage */
  improvement: number;

  /** Correction duration (milliseconds) */
  duration: number;

  /** When correction completed */
  completedAt: Date;

  /** Error message if failed */
  error?: string;
}

// ============================================================================
// Timeline Merge Types
// ============================================================================

/**
 * Timeline merge parameters
 */
export interface MergeParams {
  /** Source timeline */
  source: string;

  /** Target timeline */
  target: string;

  /** Merge strategy */
  strategy: 'three-way' | 'fast-forward' | 'squash' | 'rebase';

  /** Conflict resolution strategy */
  conflictResolution: 'manual' | 'auto' | 'source-wins' | 'target-wins' | 'merge' | 'crdt';

  /** Find common ancestor */
  findCommonAncestor: boolean;

  /** Preserve source branch */
  preserveSource: boolean;

  /** Preserve history */
  preserveHistory: boolean;

  /** Require clean state */
  requireCleanState: boolean;

  /** Merge message */
  message?: string;

  /** Merge metadata */
  metadata?: Record<string, unknown>;
}

/**
 * Timeline merge result
 */
export interface MergeResult {
  /** Merge operation ID */
  mergeId: string;

  /** Success status */
  success: boolean;

  /** Source timeline */
  source: string;

  /** Target timeline */
  target: string;

  /** Strategy used */
  strategy: string;

  /** Events merged */
  eventsMerged: number;

  /** Conflicts detected */
  conflictsDetected: number;

  /** Conflicts resolved */
  conflictsResolved: number;

  /** Unresolved conflicts */
  unresolvedConflicts: MergeConflict[];

  /** Common ancestor (if found) */
  commonAncestor?: string;

  /** Merge commit ID */
  commitId?: string;

  /** Merge duration (milliseconds) */
  duration: number;

  /** When merge completed */
  completedAt: Date;

  /** Error message if failed */
  error?: string;

  /** Merge quality score (0.0-1.0) */
  quality: number;
}

/**
 * Merge conflict
 */
export interface MergeConflict {
  /** Conflict ID */
  conflictId: string;

  /** Event ID with conflict */
  eventId: string;

  /** Source timeline */
  sourceTimeline: string;

  /** Target timeline */
  targetTimeline: string;

  /** Source event data */
  sourceData: Record<string, unknown>;

  /** Target event data */
  targetData: Record<string, unknown>;

  /** Ancestor event data (if available) */
  ancestorData?: Record<string, unknown>;

  /** Conflict type */
  type: 'data' | 'causality' | 'temporal' | 'structural';

  /** Severity */
  severity: 'minor' | 'major' | 'critical';

  /** Resolution */
  resolution: 'source' | 'target' | 'merged' | 'manual' | 'pending';

  /** Resolved data */
  resolvedData?: Record<string, unknown>;

  /** Auto-resolvable */
  autoResolvable: boolean;

  /** Suggested resolution */
  suggestion?: 'keep-source' | 'keep-target' | 'merge-both' | 'manual';

  /** Conflict description */
  description: string;
}

// ============================================================================
// Monitoring Types
// ============================================================================

/**
 * Continuous sync monitoring parameters
 */
export interface MonitoringParams {
  /** Timelines to monitor */
  timelines: string[];

  /** Check interval (milliseconds) */
  interval: number;

  /** Drift alert threshold (nanoseconds) */
  driftThreshold?: number;

  /** Divergence alert threshold (0.0-1.0) */
  divergenceThreshold?: number;

  /** Alert on drift */
  alertOnDrift: boolean;

  /** Alert on divergence */
  alertOnDivergence: boolean;

  /** Auto-correct minor drift */
  autoCorrectDrift: boolean;

  /** Auto-sync on divergence */
  autoSyncOnDivergence: boolean;

  /** Notification channels */
  notificationChannels?: string[];
}

/**
 * Drift event
 */
export interface DriftEvent {
  /** Event ID */
  eventId: string;

  /** Timeline with drift */
  timeline: string;

  /** Drift magnitude (nanoseconds) */
  magnitude: bigint;

  /** Drift direction */
  direction: 'ahead' | 'behind';

  /** Drift rate (ns/s) */
  rate: number;

  /** Exceeds threshold */
  exceedsThreshold: boolean;

  /** When detected */
  detectedAt: Date;

  /** Recommended action */
  recommendation: 'ignore' | 'monitor' | 'correct' | 'investigate';
}

/**
 * Divergence event
 */
export interface DivergenceEvent {
  /** Event ID */
  eventId: string;

  /** First timeline */
  timelineA: string;

  /** Second timeline */
  timelineB: string;

  /** Divergence point (nanoseconds) */
  divergencePoint: bigint;

  /** Divergence magnitude (0.0-1.0) */
  magnitude: number;

  /** Divergence type */
  type: string;

  /** Severity */
  severity: string;

  /** When detected */
  detectedAt: Date;

  /** Recommended action */
  recommendation: string;
}

/**
 * Sync statistics
 */
export interface SyncStats {
  /** Timeline ID */
  timelineId: string;

  /** Total syncs performed */
  totalSyncs: bigint;

  /** Successful syncs */
  successfulSyncs: bigint;

  /** Failed syncs */
  failedSyncs: bigint;

  /** Success rate (0.0-1.0) */
  successRate: number;

  /** Average sync duration (milliseconds) */
  averageDuration: number;

  /** Average drift correction (nanoseconds) */
  averageDriftCorrection: bigint;

  /** Average sync quality (0.0-1.0) */
  averageQuality: number;

  /** Last sync time */
  lastSyncAt?: Date;

  /** Next scheduled sync */
  nextSyncAt?: Date;

  /** Current drift (nanoseconds) */
  currentDrift?: bigint;

  /** Current sync quality */
  currentQuality?: number;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-TIME-019 error codes
 */
export enum SyncErrorCode {
  // Connection errors
  CONNECTION_FAILED = 'SYNC001',
  TIMELINE_UNREACHABLE = 'SYNC002',
  TIMEOUT = 'SYNC003',
  AUTHENTICATION_FAILED = 'SYNC004',

  // Sync errors
  SYNC_FAILED = 'SYNC101',
  DRIFT_CORRECTION_FAILED = 'SYNC102',
  CLOCK_UNSTABLE = 'SYNC103',
  PRECISION_INSUFFICIENT = 'SYNC104',

  // Divergence errors
  DIVERGENCE_DETECTED = 'SYNC201',
  DIVERGENCE_CRITICAL = 'SYNC202',
  TIMELINE_INCOMPATIBLE = 'SYNC203',

  // Merge errors
  MERGE_FAILED = 'SYNC301',
  MERGE_CONFLICT = 'SYNC302',
  NO_COMMON_ANCESTOR = 'SYNC303',
  STATE_NOT_CLEAN = 'SYNC304',

  // Consistency errors
  CAUSALITY_VIOLATION = 'SYNC401',
  TEMPORAL_PARADOX = 'SYNC402',
  CONSISTENCY_ERROR = 'SYNC403',

  // Resource errors
  RESOURCE_EXHAUSTED = 'SYNC501',
  RATE_LIMITED = 'SYNC502',
  QUOTA_EXCEEDED = 'SYNC503',
}

/**
 * Timeline sync error
 */
export class TimelineSyncError extends Error {
  constructor(
    public code: SyncErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'TimelineSyncError';
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
 * Event handler type
 */
export type EventHandler<T> = (event: T) => void | Promise<void>;

/**
 * Sync event types
 */
export type SyncEventType = 'drift' | 'divergence' | 'sync' | 'merge' | 'conflict' | 'error';

/**
 * Sync event
 */
export interface SyncEvent {
  /** Event type */
  type: SyncEventType;

  /** Timeline ID */
  timelineId: string;

  /** Event payload */
  payload: unknown;

  /** Event timestamp */
  timestamp: Date;
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Default configuration values
 */
export const DEFAULT_SYNC_CONFIG: Partial<SyncConfig> = {
  syncMode: 'manual',
  precision: 'microsecond',
  driftTolerance: 1000, // 1 microsecond
  syncInterval: 60000, // 1 minute
  conflictStrategy: 'manual',
  useQuantumSync: false,
  preserveCausality: true,
  maxRetries: 3,
  timeout: 30000, // 30 seconds
} as const;

/**
 * Precision levels in nanoseconds
 */
export const PRECISION_LEVELS = {
  second: 1_000_000_000,
  millisecond: 1_000_000,
  microsecond: 1_000,
  nanosecond: 1,
  picosecond: 0.001,
} as const;

/**
 * Divergence thresholds
 */
export const DIVERGENCE_THRESHOLDS = {
  negligible: 0.01,
  minor: 0.1,
  moderate: 0.25,
  major: 0.5,
  critical: 0.75,
  catastrophic: 0.95,
} as const;

/**
 * Sync limits
 */
export const SYNC_LIMITS = {
  /** Maximum timelines in single sync operation */
  MAX_TIMELINES: 1000,

  /** Maximum events per sync */
  MAX_EVENTS: 1_000_000,

  /** Maximum merge conflicts */
  MAX_CONFLICTS: 10_000,

  /** Maximum sync duration (ms) */
  MAX_DURATION: 300_000, // 5 minutes
} as const;

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core types
  UniversalTimeReference,
  ClockState,
  TemporalDrift,
  TimelineDivergence,

  // Sync types
  SyncConfig,
  SyncRequest,
  SyncResult,
  SyncConflict,

  // Drift correction
  DriftCorrectionParams,
  DriftCorrectionResult,

  // Merge types
  MergeParams,
  MergeResult,
  MergeConflict,

  // Monitoring
  MonitoringParams,
  DriftEvent,
  DivergenceEvent,
  SyncStats,

  // Utility types
  Result,
  AsyncResult,
  EventHandler,
  SyncEventType,
  SyncEvent,
};

export {
  SyncErrorCode,
  TimelineSyncError,
  DEFAULT_SYNC_CONFIG,
  PRECISION_LEVELS,
  DIVERGENCE_THRESHOLDS,
  SYNC_LIMITS,
};
