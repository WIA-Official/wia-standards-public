/**
 * WIA-TIME-010: Paradox Prevention - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Time Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Paradox Types
// ============================================================================

/**
 * Types of temporal paradoxes
 */
export enum ParadoxType {
  GRANDFATHER = 'GRANDFATHER',
  BOOTSTRAP = 'BOOTSTRAP',
  PREDESTINATION = 'PREDESTINATION',
  ONTOLOGICAL = 'ONTOLOGICAL',
  POLCHINSKI = 'POLCHINSKI',
  INFORMATION = 'INFORMATION',
  CAUSAL_VIOLATION = 'CAUSAL_VIOLATION',
  TIMELINE_CORRUPTION = 'TIMELINE_CORRUPTION',
}

/**
 * Paradox severity levels (0-4)
 */
export enum ParadoxSeverity {
  SAFE = 0,
  MINOR = 1,
  MODERATE = 2,
  SEVERE = 3,
  CRITICAL = 4,
}

/**
 * Paradox detection result
 */
export interface ParadoxDetection {
  /** Was a paradox detected? */
  paradoxDetected: boolean;

  /** Type of paradox */
  type: ParadoxType | null;

  /** Severity level (0-4) */
  severity: ParadoxSeverity;

  /** Description of the paradox */
  description: string;

  /** Affected entities */
  affectedEntities: Entity[];

  /** Causal chain involved */
  causalChain: CausalChain;

  /** Recommended action */
  recommendation: RecommendedAction;

  /** Prevention strategy */
  preventionStrategy: PreventionStrategy | null;

  /** Detection timestamp */
  detectedAt: Date;

  /** Detailed analysis */
  analysis: ParadoxAnalysis;
}

/**
 * Recommended actions based on paradox severity
 */
export enum RecommendedAction {
  PROCEED = 'PROCEED',
  PROCEED_WITH_MONITORING = 'PROCEED_WITH_MONITORING',
  PROCEED_WITH_CAUTION = 'PROCEED_WITH_CAUTION',
  PREVENT_ACTION = 'PREVENT_ACTION',
  EMERGENCY_ABORT = 'EMERGENCY_ABORT',
  EMERGENCY_ROLLBACK = 'EMERGENCY_ROLLBACK',
}

/**
 * Detailed paradox analysis
 */
export interface ParadoxAnalysis {
  /** Base severity from paradox type */
  baseSeverity: number;

  /** Impact score from various factors */
  impactScore: number;

  /** Individual impact factors */
  factors: {
    travelerExistence: number;
    timelineStability: number;
    affectedCount: number;
    temporalScope: number;
    causalityViolation: number;
  };

  /** Probability of paradox occurrence (0-1) */
  probability: number;

  /** Is the paradox self-consistent? */
  selfConsistent: boolean;

  /** Novikov compatibility */
  novikovCompatible: boolean;
}

// ============================================================================
// Timeline and Entity Types
// ============================================================================

/**
 * Entity in timeline
 */
export interface Entity {
  /** Unique entity identifier */
  id: string;

  /** Entity name */
  name?: string;

  /** Entity type */
  type: 'person' | 'object' | 'event' | 'information' | 'other';

  /** Birth/creation date */
  birthDate?: Date;

  /** Death/destruction date (if applicable) */
  deathDate?: Date;

  /** Ancestors (for persons) */
  ancestors?: string[];

  /** Creators (for objects/information) */
  creators?: string[];

  /** Current timeline location */
  timeline: string;

  /** Metadata */
  metadata?: Record<string, unknown>;
}

/**
 * Action to be performed
 */
export interface Action {
  /** Action identifier */
  id: string;

  /** Action type */
  type: 'observe' | 'modify' | 'create' | 'destroy' | 'interact' | 'communicate';

  /** Target entity */
  target?: Entity | string;

  /** Target time */
  targetTime: Date;

  /** Actor performing the action */
  actor: Entity | string;

  /** Action description */
  description: string;

  /** Expected effects */
  expectedEffects?: Effect[];

  /** Is this action reversible? */
  reversible: boolean;
}

/**
 * Effect of an action
 */
export interface Effect {
  /** Effect identifier */
  id: string;

  /** Affected entity */
  entity: Entity | string;

  /** Effect type */
  type: 'creation' | 'destruction' | 'modification' | 'state_change';

  /** Effect time */
  time: Date;

  /** Effect description */
  description: string;

  /** Magnitude (0-1) */
  magnitude: number;
}

/**
 * Timeline representation
 */
export interface Timeline {
  /** Timeline identifier */
  id: string;

  /** Timeline name */
  name?: string;

  /** Events in this timeline */
  events: TimelineEvent[];

  /** Entities in this timeline */
  entities: Entity[];

  /** Parent timeline (if branched) */
  parentTimeline?: string;

  /** Branch point (if branched) */
  branchPoint?: Date;

  /** Branch reason */
  branchReason?: string;

  /** Timeline probability (MWI) */
  probability: number;

  /** Timeline integrity score (0-1) */
  integrity: number;

  /** Is timeline stable? */
  stable: boolean;

  /** Physical state */
  physicalState?: Record<string, unknown>;

  /** Creation timestamp */
  created: Date;

  /** Last modified */
  updated: Date;
}

/**
 * Event in timeline
 */
export interface TimelineEvent {
  /** Event identifier */
  id: string;

  /** Event time */
  time: Date;

  /** Event description */
  description: string;

  /** Event type */
  type: 'major' | 'minor' | 'critical' | 'protected';

  /** Affected entities */
  affected: string[];

  /** Causal predecessors */
  causes: string[];

  /** Causal successors */
  effects: string[];

  /** Is this event mutable? */
  mutable: boolean;

  /** Protection level (0-10) */
  protectionLevel: number;
}

/**
 * Causal chain connecting events
 */
export interface CausalChain {
  /** Chain identifier */
  id: string;

  /** Events in chain */
  events: TimelineEvent[];

  /** Chain length */
  length: number;

  /** Time span (seconds) */
  timeSpan: number;

  /** Is chain closed (forms loop)? */
  isClosed: boolean;

  /** Is chain consistent? */
  isConsistent: boolean;

  /** Contradiction points */
  contradictions: Contradiction[];
}

/**
 * Contradiction in timeline
 */
export interface Contradiction {
  /** Contradiction identifier */
  id: string;

  /** Type of contradiction */
  type: 'existence' | 'causality' | 'information' | 'entropy';

  /** Description */
  description: string;

  /** Involved events */
  events: string[];

  /** Severity */
  severity: 'info' | 'warning' | 'error' | 'critical';

  /** Possible resolutions */
  resolutions: string[];
}

// ============================================================================
// Prevention Strategies
// ============================================================================

/**
 * Paradox prevention strategy
 */
export interface PreventionStrategy {
  /** Strategy name */
  name: string;

  /** Strategy type */
  type: 'novikov' | 'observer_mode' | 'protected_event' | 'timeline_branch' | 'block_action';

  /** Strategy description */
  description: string;

  /** Implementation details */
  implementation: string;

  /** Success probability (0-1) */
  successProbability: number;

  /** Side effects */
  sideEffects: string[];
}

/**
 * Novikov self-consistency check
 */
export interface NovikovCheck {
  /** Is action consistent? */
  isConsistent: boolean;

  /** Action probability (0-1) */
  probability: number;

  /** Detected contradictions */
  contradictions: Contradiction[];

  /** Violations of consistency */
  violations: string[];

  /** Recommendations */
  recommendations: string[];
}

/**
 * Protected event configuration
 */
export interface ProtectedEvent {
  /** Event to protect */
  event: TimelineEvent | string;

  /** Protection level (0-10) */
  protectionLevel: number;

  /** Justification for protection */
  justification: string;

  /** Is event locked? */
  locked: boolean;

  /** Modification attempts log */
  modificationAttempts: ModificationAttempt[];
}

/**
 * Modification attempt record
 */
export interface ModificationAttempt {
  /** Attempt timestamp */
  timestamp: Date;

  /** Actor attempting modification */
  actor: Entity | string;

  /** Attempted modification */
  modification: Action;

  /** Impact assessment */
  impact: number;

  /** Was attempt denied? */
  denied: boolean;

  /** Denial reason */
  reason?: string;
}

/**
 * Observer mode configuration
 */
export interface ObserverMode {
  /** Is observer mode active? */
  active: boolean;

  /** Traveler in observer mode */
  traveler: Entity | string;

  /** Target observation time */
  targetTime: Date;

  /** Allowed actions */
  allowedActions: string[];

  /** Blocked actions */
  blockedActions: string[];

  /** Observation log */
  observationLog: ObservationEntry[];

  /** Cloaking settings */
  cloaking: {
    visual: boolean;
    audio: boolean;
    physical: boolean;
    electromagnetic: boolean;
  };
}

/**
 * Observation entry
 */
export interface ObservationEntry {
  /** Observation timestamp */
  timestamp: Date;

  /** What was observed */
  observation: string;

  /** Observer */
  observer: string;

  /** Location */
  location?: {
    x: number;
    y: number;
    z: number;
  };

  /** Metadata */
  metadata?: Record<string, unknown>;
}

// ============================================================================
// Timeline Branching
// ============================================================================

/**
 * Timeline branch configuration
 */
export interface TimelineBranch {
  /** Branch identifier */
  id: string;

  /** Parent timeline */
  parentId: string;

  /** Branch point in time */
  branchPoint: Date;

  /** Reason for branching */
  reason: string;

  /** Branch probability (MWI) */
  probability: number;

  /** Divergence metric from parent */
  divergence: number;

  /** Branch creation time */
  created: Date;

  /** Is branch stable? */
  stable: boolean;
}

/**
 * Branch merge request
 */
export interface BranchMergeRequest {
  /** First branch to merge */
  branchA: string;

  /** Second branch to merge */
  branchB: string;

  /** Merge strategy */
  strategy: 'union' | 'intersection' | 'priority_a' | 'priority_b';

  /** Conflict resolution */
  conflictResolution: 'abort' | 'branch' | 'manual';
}

/**
 * Branch merge result
 */
export interface BranchMergeResult {
  /** Was merge successful? */
  merged: boolean;

  /** Merged timeline ID */
  timelineId?: string;

  /** Parent branches */
  parentBranches: string[];

  /** Conflicts encountered */
  conflicts: Contradiction[];

  /** Resolution method used */
  resolution?: string;

  /** Error message if failed */
  error?: string;
}

/**
 * Timeline divergence metrics
 */
export interface TimelineDivergence {
  /** Divergence score (0-1) */
  divergence: number;

  /** Severity classification */
  severity: 'negligible' | 'minor' | 'moderate' | 'severe' | 'critical';

  /** Number of major changes */
  majorChanges: number;

  /** Number of minor changes */
  minorChanges: number;

  /** Is timeline stable? */
  stable: boolean;

  /** Breakdown by category */
  breakdown: {
    events: number;
    entities: number;
    physicalLaws: number;
  };
}

// ============================================================================
// Snapshot and Rollback
// ============================================================================

/**
 * Timeline snapshot
 */
export interface TimelineSnapshot {
  /** Snapshot identifier */
  id: string;

  /** Timeline being snapshotted */
  timelineId: string;

  /** Snapshot label */
  label: string;

  /** Snapshot timestamp */
  timestamp: Date;

  /** Number of events */
  eventCount: number;

  /** Number of entities */
  entityCount: number;

  /** Checksum for integrity */
  checksum: string;

  /** Has integrity been verified? */
  verified: boolean;

  /** Snapshot size in bytes */
  sizeBytes: number;

  /** Snapshot data (compressed) */
  data?: unknown;
}

/**
 * Rollback request
 */
export interface RollbackRequest {
  /** Target snapshot to restore */
  targetSnapshot: string;

  /** Preserve memories of affected entities? */
  preserveMemories: boolean;

  /** Create backup before rollback? */
  createBackup: boolean;

  /** Operator authorization */
  verification: {
    operator: string;
    authorization: string;
  };
}

/**
 * Rollback result
 */
export interface RollbackResult {
  /** Was rollback successful? */
  success: boolean;

  /** Snapshot restored */
  snapshotId: string;

  /** Time restored to */
  rollbackTime: Date;

  /** Backup ID if created */
  backupId?: string;

  /** Number of affected entities */
  affectedEntities: number;

  /** Number of events removed */
  eventsRemoved: number;

  /** Were memories preserved? */
  memoriesPreserved: boolean;

  /** Error message if failed */
  error?: string;
}

/**
 * Selective rollback request
 */
export interface SelectiveRollbackRequest {
  /** Event IDs to rollback */
  eventIds: string[];

  /** Target time to rollback to */
  targetTime: Date;

  /** Include cascading effects? */
  includeCascading: boolean;
}

// ============================================================================
// Monitoring and Detection
// ============================================================================

/**
 * Paradox monitoring configuration
 */
export interface MonitoringConfig {
  /** Monitoring interval (milliseconds) */
  intervalMs: number;

  /** Enable real-time detection */
  realTimeDetection: boolean;

  /** Detection algorithms to use */
  algorithms: DetectionAlgorithm[];

  /** Severity threshold for alerts */
  alertThreshold: ParadoxSeverity;

  /** Auto-abort on critical? */
  autoAbort: boolean;

  /** Auto-rollback on critical? */
  autoRollback: boolean;
}

/**
 * Detection algorithm types
 */
export enum DetectionAlgorithm {
  CAUSAL_CHAIN = 'CAUSAL_CHAIN',
  EXISTENCE_VERIFICATION = 'EXISTENCE_VERIFICATION',
  GRANDFATHER_CHECK = 'GRANDFATHER_CHECK',
  TIMELINE_DIVERGENCE = 'TIMELINE_DIVERGENCE',
  BOOTSTRAP_DETECTION = 'BOOTSTRAP_DETECTION',
  ENTROPY_ANALYSIS = 'ENTROPY_ANALYSIS',
}

/**
 * Monitoring status
 */
export interface MonitoringStatus {
  /** Is monitoring active? */
  active: boolean;

  /** Current operation being monitored */
  operation?: string;

  /** Paradoxes detected */
  paradoxesDetected: number;

  /** Last check timestamp */
  lastCheck: Date;

  /** Current severity level */
  currentSeverity: ParadoxSeverity;

  /** Active alerts */
  activeAlerts: Alert[];
}

/**
 * Alert for paradox detection
 */
export interface Alert {
  /** Alert identifier */
  id: string;

  /** Alert severity */
  severity: ParadoxSeverity;

  /** Alert message */
  message: string;

  /** Related paradox */
  paradox?: ParadoxDetection;

  /** Alert timestamp */
  timestamp: Date;

  /** Has alert been acknowledged? */
  acknowledged: boolean;
}

// ============================================================================
// Causal Loop Management
// ============================================================================

/**
 * Causal loop configuration
 */
export interface CausalLoop {
  /** Loop identifier */
  id: string;

  /** Events in loop */
  events: TimelineEvent[];

  /** Loop period */
  period: number;

  /** Is loop stable? */
  stable: boolean;

  /** Stability metric (0-1) */
  stability: number;

  /** Entropy change */
  entropyChange: number;

  /** Information conservation */
  informationConserved: boolean;

  /** External perturbations */
  perturbations: Perturbation[];
}

/**
 * External perturbation to causal loop
 */
export interface Perturbation {
  /** Perturbation identifier */
  id: string;

  /** Source of perturbation */
  source: string;

  /** Magnitude (0-1) */
  magnitude: number;

  /** Impact on loop stability */
  impact: number;

  /** Timestamp */
  timestamp: Date;
}

/**
 * Loop stabilization result
 */
export interface LoopStabilization {
  /** Is loop stable? */
  stable: boolean;

  /** Stability metric */
  stability: number;

  /** Entropy change */
  entropyChange: number;

  /** Information conserved? */
  informationConserved: boolean;

  /** Detected perturbations */
  perturbations: Perturbation[];

  /** Recommendations */
  recommendations: string[];

  /** Error if unstable */
  error?: string;
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
 * Paradox prevention operation
 */
export interface ParadoxPreventionOperation {
  /** Operation identifier */
  id: string;

  /** Operation type */
  type: 'detection' | 'prevention' | 'rollback' | 'branch' | 'monitoring';

  /** Operation status */
  status: 'pending' | 'in-progress' | 'completed' | 'failed' | 'aborted';

  /** Start time */
  startTime: Date;

  /** End time */
  endTime?: Date;

  /** Associated timeline */
  timeline: string;

  /** Result */
  result?: unknown;

  /** Error if failed */
  error?: string;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-TIME-010 error codes
 */
export enum ParadoxErrorCode {
  PARADOX_DETECTED = 'P001',
  CRITICAL_PARADOX = 'P002',
  TIMELINE_CORRUPTED = 'P003',
  ROLLBACK_FAILED = 'P004',
  SNAPSHOT_CORRUPTED = 'P005',
  BRANCH_FAILED = 'P006',
  DETECTION_FAILED = 'P007',
  INVALID_ACTION = 'P008',
  PROTECTED_EVENT = 'P009',
  NOVIKOV_VIOLATION = 'P010',
}

/**
 * Paradox prevention error
 */
export class ParadoxPreventionError extends Error {
  constructor(
    public code: ParadoxErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'ParadoxPreventionError';
  }
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Paradox prevention constants
 */
export const PARADOX_CONSTANTS = {
  /** Minimum timeline stability threshold */
  MIN_STABILITY: 0.95,

  /** Maximum divergence allowed before branching */
  MAX_DIVERGENCE: 0.30,

  /** Default monitoring interval (ms) */
  DEFAULT_MONITOR_INTERVAL: 100,

  /** Critical severity auto-abort threshold */
  AUTO_ABORT_THRESHOLD: ParadoxSeverity.CRITICAL,

  /** Snapshot verification required */
  REQUIRE_SNAPSHOT_VERIFICATION: true,

  /** Maximum rollback attempts */
  MAX_ROLLBACK_ATTEMPTS: 3,

  /** Protected event min level */
  PROTECTED_EVENT_MIN_LEVEL: 7,

  /** Default observer mode cloaking */
  DEFAULT_CLOAKING: {
    visual: true,
    audio: true,
    physical: true,
    electromagnetic: true,
  },
} as const;

// ============================================================================
// Export All Types
// ============================================================================

export type {
  ParadoxDetection,
  ParadoxAnalysis,
  Entity,
  Action,
  Effect,
  Timeline,
  TimelineEvent,
  CausalChain,
  Contradiction,
  PreventionStrategy,
  NovikovCheck,
  ProtectedEvent,
  ModificationAttempt,
  ObserverMode,
  ObservationEntry,
  TimelineBranch,
  BranchMergeRequest,
  BranchMergeResult,
  TimelineDivergence,
  TimelineSnapshot,
  RollbackRequest,
  RollbackResult,
  SelectiveRollbackRequest,
  MonitoringConfig,
  MonitoringStatus,
  Alert,
  CausalLoop,
  Perturbation,
  LoopStabilization,
  ParadoxPreventionOperation,
};

export {
  ParadoxType,
  ParadoxSeverity,
  RecommendedAction,
  DetectionAlgorithm,
  ParadoxErrorCode,
  ParadoxPreventionError,
  PARADOX_CONSTANTS,
};
