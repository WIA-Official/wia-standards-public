/**
 * WIA-TIME-009: Causality Protection - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Time Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Timeline Types
// ============================================================================

/**
 * Timeline representation
 */
export interface Timeline {
  /** Unique timeline identifier */
  id: string;

  /** Timeline name/description */
  name?: string;

  /** Events in this timeline */
  events: TimelineEvent[];

  /** Timeline branch point (if branched from another) */
  branchPoint?: Date;

  /** Parent timeline ID (if branched) */
  parentTimeline?: string;

  /** Child timeline IDs (if branched) */
  childTimelines?: string[];

  /** Timeline integrity score (0-1) */
  integrity: number;

  /** Is timeline active? */
  active: boolean;

  /** Is timeline deprecated? */
  deprecated?: boolean;

  /** Successor timeline (if deprecated) */
  successor?: string;

  /** Creation timestamp */
  created: Date;

  /** Last update timestamp */
  updated: Date;

  /** Metadata */
  metadata?: Record<string, unknown>;
}

/**
 * Event in a timeline
 */
export interface TimelineEvent {
  /** Event unique identifier */
  id: string;

  /** Event timestamp */
  time: Date;

  /** Event description */
  description: string;

  /** Event type/category */
  type?: string;

  /** Spatial location */
  location?: {
    x: number;
    y: number;
    z: number;
  };

  /** Events that cause this event */
  causes: string[];

  /** Events caused by this event */
  effects: string[];

  /** Can this event be modified? */
  mutable: boolean;

  /** Is this event protected? */
  protected?: boolean;

  /** Protection level (1-5) */
  protectionLevel?: number;

  /** Event properties */
  properties?: Record<string, unknown>;

  /** Causal weight (importance in causal chain) */
  causalWeight?: number;

  /** Probability (for quantum branches) */
  probability?: number;
}

/**
 * Temporal action to be validated
 */
export interface TemporalAction {
  /** Action identifier */
  id: string;

  /** Action type */
  type:
    | 'create_event'
    | 'modify_event'
    | 'delete_event'
    | 'create_causal_link'
    | 'remove_causal_link'
    | 'branch_timeline'
    | 'merge_timelines'
    | 'other';

  /** Target timeline */
  timeline: string;

  /** Target event (if applicable) */
  targetEvent?: string;

  /** Action parameters */
  parameters: Record<string, unknown>;

  /** Actor/initiator */
  actor?: string;

  /** Timestamp of action */
  timestamp: Date;

  /** Authorization level */
  authLevel?: number;
}

// ============================================================================
// Consistency Types
// ============================================================================

/**
 * Novikov self-consistency check result
 */
export interface ConsistencyResult {
  /** Is action consistent with timeline? */
  isConsistent: boolean;

  /** Consistency score (0-1) */
  score: number;

  /** Is action allowed? */
  allowed: boolean;

  /** Detected violations */
  violations: Violation[];

  /** Warnings (non-blocking issues) */
  warnings: string[];

  /** Recommendations */
  recommendations: string[];

  /** Probability of success if allowed (0-1) */
  probability: number;

  /** Alternative consistent actions */
  alternatives?: TemporalAction[];

  /** Computation time in milliseconds */
  computationTime: number;
}

/**
 * Causality violation
 */
export interface Violation {
  /** Violation unique ID */
  id: string;

  /** Violation type */
  type:
    | 'TEMPORAL_ORDER_VIOLATION'
    | 'CAUSAL_CHAIN_BREAK'
    | 'CONTRADICTORY_EVENTS'
    | 'PROBABILITY_VIOLATION'
    | 'CONSERVATION_VIOLATION'
    | 'HISTORICAL_DIVERGENCE'
    | 'GRANDFATHER_PARADOX'
    | 'BOOTSTRAP_PARADOX'
    | 'PREDESTINATION_PARADOX'
    | 'CTC_FORMATION'
    | 'OTHER';

  /** Severity level */
  severity: 'LOW' | 'MEDIUM' | 'HIGH' | 'CRITICAL';

  /** Severity score (0-20) */
  severityScore: number;

  /** Description */
  description: string;

  /** Involved events */
  events: string[];

  /** Affected causal chains */
  causalChains?: string[];

  /** Detection timestamp */
  detected: Date;

  /** Suggested corrections */
  corrections?: CorrectionStrategy[];

  /** Can be auto-corrected? */
  autoCorrectible: boolean;

  /** Additional details */
  details?: Record<string, unknown>;
}

/**
 * Correction strategy for violations
 */
export interface CorrectionStrategy {
  /** Strategy type */
  type:
    | 'EVENT_REMOVAL'
    | 'EVENT_MODIFICATION'
    | 'CAUSAL_REPAIR'
    | 'TIMELINE_BRANCHING'
    | 'TEMPORAL_SMOOTHING'
    | 'ROLLBACK';

  /** Strategy description */
  description: string;

  /** Confidence in success (0-1) */
  confidence: number;

  /** Impact assessment */
  impact: {
    eventsAffected: number;
    causalChainsAffected: number;
    integrityChange: number;
  };

  /** Parameters for strategy */
  parameters: Record<string, unknown>;

  /** Estimated execution time (ms) */
  estimatedTime: number;
}

// ============================================================================
// Causal Loop Types
// ============================================================================

/**
 * Causal loop structure
 */
export interface CausalLoop {
  /** Loop unique identifier */
  id: string;

  /** Events in the loop */
  events: string[];

  /** Loop classification */
  type: 'BOOTSTRAP' | 'PREDESTINATION' | 'SIMPLE' | 'COMPLEX';

  /** Loop length (number of events) */
  length: number;

  /** Total temporal span (seconds) */
  temporalSpan: number;

  /** Loop stability score (0-1) */
  stability: number;

  /** Is loop self-consistent? */
  selfConsistent: boolean;

  /** Entropy change per cycle */
  entropyChange: number;

  /** Information content (bits) */
  informationContent: number;

  /** Risk assessment */
  risk: 'LOW' | 'MEDIUM' | 'HIGH' | 'CRITICAL';

  /** Recommended action */
  recommendation: 'ALLOW' | 'ALLOW_WITH_MONITORING' | 'BLOCK' | 'CORRECT';

  /** Detection timestamp */
  detected: Date;
}

/**
 * Loop detection parameters
 */
export interface LoopDetectionParams {
  /** Timeline to analyze */
  timeline: Timeline;

  /** Maximum search depth */
  maxDepth: number;

  /** Detect bootstrap paradoxes? */
  detectBootstrap: boolean;

  /** Detect predestination paradoxes? */
  detectPredestination: boolean;

  /** Minimum loop length to report */
  minLoopLength?: number;

  /** Maximum loop length to search */
  maxLoopLength?: number;
}

// ============================================================================
// Timeline Integrity Types
// ============================================================================

/**
 * Timeline integrity score breakdown
 */
export interface IntegrityScore {
  /** Overall integrity (0-1) */
  overall: number;

  /** Component scores */
  components: {
    /** Causal consistency (0-1) */
    causalConsistency: number;

    /** Event coherence (0-1) */
    eventCoherence: number;

    /** Temporal continuity (0-1) */
    temporalContinuity: number;

    /** Historical consistency (0-1) */
    historicalConsistency: number;
  };

  /** Integrity level */
  level: 'PERFECT' | 'EXCELLENT' | 'GOOD' | 'FAIR' | 'POOR' | 'CRITICAL';

  /** Number of violations found */
  violationCount: number;

  /** Number of warnings */
  warningCount: number;

  /** Calculation timestamp */
  timestamp: Date;

  /** Detailed breakdown */
  details?: {
    consistentCausalPairs: number;
    totalCausalPairs: number;
    coherentEvents: number;
    totalEvents: number;
    timelineGaps: number;
    preservedHistoricalEvents: number;
    criticalEvents: number;
  };
}

/**
 * Monitoring session
 */
export interface MonitorSession {
  /** Session unique ID */
  id: string;

  /** Timeline being monitored */
  timeline: string;

  /** Start time */
  started: Date;

  /** Is session active? */
  active: boolean;

  /** Monitoring interval (seconds) */
  interval: number;

  /** Alert threshold (0-1) */
  alertThreshold: number;

  /** Number of integrity checks performed */
  checksPerformed: number;

  /** Number of alerts triggered */
  alertsTriggered: number;

  /** Latest integrity score */
  latestIntegrity?: IntegrityScore;

  /** Stop monitoring session */
  stop: () => void;
}

/**
 * Monitoring options
 */
export interface MonitorOptions {
  /** Monitoring interval in seconds */
  interval?: number;

  /** Alert threshold (trigger if integrity below this) */
  alertThreshold?: number;

  /** Enable auto-correction? */
  autoCorrection?: boolean;

  /** Alert callbacks */
  onAlert?: (alert: ViolationAlert) => void;

  /** Integrity update callbacks */
  onIntegrityUpdate?: (integrity: IntegrityScore) => void;

  /** Violation detected callback */
  onViolation?: (violation: Violation) => void;
}

/**
 * Violation alert
 */
export interface ViolationAlert {
  /** Alert ID */
  id: string;

  /** Alert timestamp */
  timestamp: Date;

  /** Timeline */
  timeline: string;

  /** Violation that triggered alert */
  violation: Violation;

  /** Current integrity score */
  integrity: IntegrityScore;

  /** Recommended actions */
  recommendedActions: string[];

  /** Auto-correction applied? */
  autoCorrectionApplied: boolean;

  /** Alert severity */
  severity: 'LOW' | 'MEDIUM' | 'HIGH' | 'CRITICAL';
}

// ============================================================================
// Event Dependency Graph Types
// ============================================================================

/**
 * Event dependency graph
 */
export interface EventGraph {
  /** Graph identifier */
  id: string;

  /** Timeline this graph represents */
  timeline: string;

  /** Graph nodes (events) */
  nodes: GraphNode[];

  /** Graph edges (causal relationships) */
  edges: GraphEdge[];

  /** Graph statistics */
  stats: GraphStatistics;

  /** Is graph acyclic? */
  isDAG: boolean;

  /** Detected cycles */
  cycles: string[][];

  /** Critical events (articulation points) */
  criticalEvents: string[];

  /** Generation timestamp */
  generated: Date;
}

/**
 * Graph node (event)
 */
export interface GraphNode {
  /** Node ID (event ID) */
  id: string;

  /** Event data */
  event: TimelineEvent;

  /** In-degree (number of causes) */
  inDegree: number;

  /** Out-degree (number of effects) */
  outDegree: number;

  /** Betweenness centrality */
  betweenness?: number;

  /** Degree centrality */
  degreeCentrality?: number;

  /** Causal influence score */
  causalInfluence?: number;

  /** Node position (for visualization) */
  position?: {
    x: number;
    y: number;
  };
}

/**
 * Graph edge (causal relationship)
 */
export interface GraphEdge {
  /** Edge ID */
  id: string;

  /** Source event (cause) */
  source: string;

  /** Target event (effect) */
  target: string;

  /** Causal strength (0-1) */
  strength: number;

  /** Time delay (seconds) */
  delay: number;

  /** Certainty of causation (0-1) */
  certainty: number;

  /** Edge type */
  type?: 'DIRECT' | 'INDIRECT' | 'INFERRED';
}

/**
 * Graph statistics
 */
export interface GraphStatistics {
  /** Number of nodes */
  nodeCount: number;

  /** Number of edges */
  edgeCount: number;

  /** Graph density (0-1) */
  density: number;

  /** Number of strongly connected components */
  stronglyConnectedComponents: number;

  /** Longest causal chain length */
  longestChainLength: number;

  /** Average causal chain length */
  averageChainLength: number;

  /** Number of isolated events */
  isolatedEvents: number;

  /** Number of weak edges (certainty < threshold) */
  weakEdges: number;
}

/**
 * Graph analysis result
 */
export interface GraphAnalysis {
  /** Graph being analyzed */
  graph: EventGraph;

  /** Analysis type performed */
  analysisType: string;

  /** Key findings */
  findings: string[];

  /** Critical events identified */
  criticalEvents: {
    eventId: string;
    reason: string;
    severity: 'LOW' | 'MEDIUM' | 'HIGH';
  }[];

  /** Recommendations */
  recommendations: string[];

  /** Analysis timestamp */
  timestamp: Date;
}

// ============================================================================
// Protection Types
// ============================================================================

/**
 * Protection status of an event
 */
export interface ProtectionStatus {
  /** Is event protected? */
  protected: boolean;

  /** Protection level (1-5, null if not protected) */
  level: number | null;

  /** Protection reason */
  reason?: string;

  /** Can modifications be authorized? */
  authorizationPossible: boolean;

  /** Required authorization level */
  requiredAuth?: number;

  /** Protected by which rule/registry */
  protectedBy?: string;
}

/**
 * Protected event registry entry
 */
export interface ProtectedEvent {
  /** Event identifier pattern */
  eventId: string;

  /** Protection level (1-5) */
  level: number;

  /** Protection reason */
  reason: string;

  /** Can be overridden with authorization? */
  allowOverride: boolean;

  /** Required authorization level for override */
  overrideAuthLevel?: number;

  /** Added to registry timestamp */
  added: Date;

  /** Added by */
  addedBy?: string;
}

/**
 * Protection rule (pattern-based)
 */
export interface ProtectionRule {
  /** Rule identifier */
  id: string;

  /** Rule name */
  name: string;

  /** Event matching pattern */
  pattern: {
    timeRange?: [Date, Date];
    type?: string;
    description?: RegExp;
    location?: {
      center: { x: number; y: number; z: number };
      radius: number;
    };
  };

  /** Protection level applied by this rule */
  protectionLevel: number;

  /** Rule priority (higher = checked first) */
  priority: number;

  /** Is rule active? */
  active: boolean;
}

// ============================================================================
// Correction Types
// ============================================================================

/**
 * Correction result
 */
export interface CorrectionResult {
  /** Was correction successful? */
  success: boolean;

  /** Corrected timeline */
  timeline?: Timeline;

  /** Corrections applied */
  correctionsApplied: AppliedCorrection[];

  /** Violations remaining */
  remainingViolations: Violation[];

  /** Integrity before correction */
  integrityBefore: number;

  /** Integrity after correction */
  integrityAfter: number;

  /** Integrity change */
  integrityDelta: number;

  /** Correction timestamp */
  timestamp: Date;

  /** Total correction time (ms) */
  duration: number;

  /** Error message if failed */
  error?: string;
}

/**
 * Applied correction
 */
export interface AppliedCorrection {
  /** Correction ID */
  id: string;

  /** Violation being corrected */
  violation: string;

  /** Strategy used */
  strategy: CorrectionStrategy;

  /** Was this correction successful? */
  success: boolean;

  /** Events modified */
  eventsModified: string[];

  /** Events created */
  eventsCreated: string[];

  /** Events removed */
  eventsRemoved: string[];

  /** Error if failed */
  error?: string;
}

// ============================================================================
// Chronology Protection Types
// ============================================================================

/**
 * CTC detection result
 */
export interface CTCDetection {
  /** Was a CTC detected? */
  detected: boolean;

  /** CTC type */
  type?: 'GODEL' | 'TIPLER' | 'KERR' | 'WORMHOLE' | 'FIELD' | 'OTHER';

  /** CTC worldline */
  worldline?: {
    events: string[];
    period: number;
  };

  /** Quantum stress-energy divergence */
  stressTensorDivergence?: number;

  /** Risk level */
  risk?: 'THEORETICAL' | 'HIGH' | 'CRITICAL';

  /** Should formation be prevented? */
  preventFormation: boolean;

  /** Prevention actions recommended */
  preventionActions?: string[];
}

/**
 * Chronology protection status
 */
export interface ChronologyProtection {
  /** Is protection active? */
  active: boolean;

  /** Protection strength (0-1) */
  strength: number;

  /** Monitoring frequency (Hz) */
  monitoringFrequency: number;

  /** Energy threshold for intervention */
  energyThreshold: number;

  /** Number of CTCs prevented */
  ctcsPrevented: number;

  /** Last check timestamp */
  lastCheck: Date;
}

// ============================================================================
// Configuration Types
// ============================================================================

/**
 * Causality protector configuration
 */
export interface CausalityProtectorConfig {
  /** Enforcement level */
  enforcementLevel?: 'advisory' | 'standard' | 'strict' | 'maximum';

  /** Enable auto-correction? */
  autoCorrection?: boolean;

  /** Alert threshold (0-1) */
  alertThreshold?: number;

  /** Monitoring interval (seconds) */
  monitoringInterval?: number;

  /** Maximum correction attempts per violation */
  maxCorrectionAttempts?: number;

  /** Enable timeline branching for corrections? */
  enableTimelineBranching?: boolean;

  /** Protected event enforcement? */
  protectedEventEnforcement?: boolean;

  /** Loop detection depth */
  loopDetectionDepth?: number;

  /** Integrity score weights */
  integrityWeights?: {
    causalConsistency: number;
    eventCoherence: number;
    temporalContinuity: number;
    historicalConsistency: number;
  };

  /** Chronology protection config */
  chronologyProtection?: {
    enabled: boolean;
    energyThreshold: number;
    monitoringFrequency: number;
  };
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Causality protection constants
 */
export const CAUSALITY_CONSTANTS = {
  /** Minimum integrity threshold for normal operations */
  MIN_INTEGRITY: 0.95,

  /** Critical integrity threshold (emergency isolation) */
  CRITICAL_INTEGRITY: 0.5,

  /** Default monitoring interval (seconds) */
  DEFAULT_MONITORING_INTERVAL: 0.1,

  /** Default alert threshold */
  DEFAULT_ALERT_THRESHOLD: 0.95,

  /** Maximum loop detection depth */
  MAX_LOOP_DEPTH: 1000,

  /** Maximum causal chain length */
  MAX_CAUSAL_CHAIN_LENGTH: 1000,

  /** Maximum correction attempts */
  MAX_CORRECTION_ATTEMPTS: 10,

  /** Default entropy change threshold for bootstrap loops */
  BOOTSTRAP_ENTROPY_THRESHOLD: 1e-10,

  /** Planck time (for CTC monitoring) */
  PLANCK_TIME: 5.391247e-44,

  /** Planck energy threshold */
  PLANCK_ENERGY: 1.956e9, // Joules
} as const;

/**
 * Protection level descriptions
 */
export const PROTECTION_LEVELS = {
  1: 'Absolute - Cannot be modified under any circumstances',
  2: 'Very High - Requires extreme authorization',
  3: 'High - Requires authorization',
  4: 'Medium - Warnings issued, monitoring enabled',
  5: 'Low - Monitoring only',
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
 * Causality protection error codes
 */
export enum CausalityErrorCode {
  NOVIKOV_VIOLATION = 'CP001',
  CHRONOLOGY_PROTECTION_TRIGGERED = 'CP002',
  CAUSAL_LOOP_DETECTED = 'CP003',
  INTEGRITY_BELOW_THRESHOLD = 'CP004',
  PROTECTED_EVENT_MODIFICATION = 'CP005',
  TEMPORAL_ORDER_VIOLATION = 'CP006',
  CAUSAL_CHAIN_BREAK = 'CP007',
  AUTO_CORRECTION_FAILED = 'CP008',
  TIMELINE_ISOLATION_REQUIRED = 'CP009',
  HISTORICAL_INCONSISTENCY = 'CP010',
}

/**
 * Causality protection error
 */
export class CausalityProtectionError extends Error {
  constructor(
    public code: CausalityErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'CausalityProtectionError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  Timeline,
  TimelineEvent,
  TemporalAction,
  ConsistencyResult,
  Violation,
  CorrectionStrategy,
  CausalLoop,
  LoopDetectionParams,
  IntegrityScore,
  MonitorSession,
  MonitorOptions,
  ViolationAlert,
  EventGraph,
  GraphNode,
  GraphEdge,
  GraphStatistics,
  GraphAnalysis,
  ProtectionStatus,
  ProtectedEvent,
  ProtectionRule,
  CorrectionResult,
  AppliedCorrection,
  CTCDetection,
  ChronologyProtection,
  CausalityProtectorConfig,
};

export {
  CAUSALITY_CONSTANTS,
  PROTECTION_LEVELS,
  CausalityErrorCode,
  CausalityProtectionError,
};
