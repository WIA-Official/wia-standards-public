/**
 * WIA-TIME-005: Timeline Anchor Standard - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @description Complete type definitions for Timeline Anchor operations
 *
 * 弘益人間 (Benefit All Humanity)
 * © 2025 SmileStory Inc. / WIA
 */

// ============================================================================
// Core Coordinate Types
// ============================================================================

/**
 * 5-dimensional spacetime coordinates
 */
export interface AnchorCoordinates {
  /** Temporal coordinate (Unix timestamp in milliseconds) */
  t: number;

  /** Spatial X coordinate (meters from origin) */
  x: number;

  /** Spatial Y coordinate (meters from origin) */
  y: number;

  /** Spatial Z coordinate (meters from origin, altitude) */
  z: number;

  /** Dimensional index (timeline/dimension identifier) */
  d: number;

  /** Quantum phase (0-1) */
  q: number;
}

/**
 * Spatial coordinates only
 */
export interface SpatialCoordinates {
  x: number;
  y: number;
  z: number;
}

// ============================================================================
// Anchor Types
// ============================================================================

/**
 * Anchor type classification
 */
export type AnchorType = 'primary' | 'secondary' | 'waypoint' | 'emergency';

/**
 * Anchor operational status
 */
export type AnchorStatus =
  | 'active'
  | 'standby'
  | 'maintenance'
  | 'degraded'
  | 'failed'
  | 'decommissioned';

/**
 * Beacon configuration
 */
export interface BeaconConfig {
  /** Primary frequency in Hz (recommended: 432 Hz) */
  frequency: number;

  /** Harmonic frequencies */
  harmonics?: number[];

  /** Temporal range in years */
  range: number;

  /** Spatial range in kilometers */
  spatialRange?: number;

  /** Signal type */
  signalType: 'quantum-entangled' | 'electromagnetic' | 'gravitational';

  /** Transmission power in Gigawatts */
  power?: number;
}

/**
 * Complete Timeline Anchor definition
 */
export interface TimelineAnchor {
  /** Unique anchor identifier */
  id: string;

  /** Human-readable name */
  name: string;

  /** Anchor type */
  type: AnchorType;

  /** 5D spacetime coordinates */
  coordinates: AnchorCoordinates;

  /** Temporal flux strength in Gigawatts (1.21 - 10.0) */
  strength: number;

  /** Return beacon configuration */
  beacon: BeaconConfig;

  /** Current operational status */
  status: AnchorStatus;

  /** Overall health percentage (0-100) */
  health: number;

  /** Creation timestamp (Unix ms) */
  created: number;

  /** Last maintenance timestamp (Unix ms) */
  lastMaintenance?: number;

  /** Next scheduled maintenance (Unix ms) */
  nextMaintenance?: number;

  /** Additional metadata */
  metadata?: Record<string, any>;
}

/**
 * Anchor point (simplified anchor reference)
 */
export interface AnchorPoint {
  id: string;
  name: string;
  coordinates: AnchorCoordinates;
  health: number;
}

// ============================================================================
// Beacon Types
// ============================================================================

/**
 * Beacon signal data
 */
export interface BeaconSignal {
  /** Source anchor ID */
  anchorId: string;

  /** Signal timestamp */
  timestamp: number;

  /** Sequence number */
  sequenceNumber: number;

  /** Current position */
  position: AnchorCoordinates;

  /** Navigation data */
  navigation: NavigationData;

  /** Health status */
  health: HealthStatus;

  /** Quantum state */
  quantum: QuantumState;

  /** Digital signature */
  signature: string;
}

/**
 * Navigation data from beacon
 */
export interface NavigationData {
  /** Waypoint coordinates */
  waypoints: AnchorCoordinates[];

  /** Total distance in spacetime units */
  totalDistance: number;

  /** Estimated travel time in seconds */
  estimatedTime: number;

  /** Encoded return path */
  returnPath: string;
}

/**
 * Temporal beacon for navigation
 */
export interface TemporalBeacon {
  /** Beacon identifier */
  id: string;

  /** Associated anchor ID */
  anchorId: string;

  /** Beacon configuration */
  config: BeaconConfig;

  /** Current signal strength (0-100) */
  signalStrength: number;

  /** Last signal timestamp */
  lastSignal: number;

  /** Beacon operational status */
  status: 'active' | 'standby' | 'offline';
}

// ============================================================================
// Health & Status Types
// ============================================================================

/**
 * Component health metrics
 */
export interface ComponentHealth {
  /** Quantum lock health (0-100%) */
  quantumLock: number;

  /** Temporal flux health (0-100%) */
  temporalFlux: number;

  /** Beacon signal health (0-100%) */
  beaconSignal: number;

  /** Spatial stability (0-100%) */
  spatialStability: number;

  /** Dimensional anchor health (0-100%) */
  dimensionalAnchor: number;
}

/**
 * Degradation analysis
 */
export interface DegradationMetrics {
  /** Degradation rate (percent per year) */
  rate: number;

  /** Is degradation accelerating? */
  accelerating: boolean;

  /** Estimated failure timestamp (Unix ms) */
  estimatedFailure: number | null;

  /** Time until next maintenance recommended (seconds) */
  timeUntilMaintenance: number;
}

/**
 * Maintenance information
 */
export interface MaintenanceInfo {
  /** Last service timestamp (Unix ms) */
  lastService: number | null;

  /** Next scheduled service (Unix ms) */
  nextService: number;

  /** Maintenance urgency level */
  urgency: 'none' | 'routine' | 'soon' | 'urgent' | 'critical';

  /** Recommended actions */
  recommendations: string[];
}

/**
 * Complete anchor health status
 */
export interface AnchorHealth {
  /** Anchor identifier */
  anchorId: string;

  /** Health check timestamp */
  timestamp: number;

  /** Overall health (0-100%) */
  overall: number;

  /** Component-level health */
  components: ComponentHealth;

  /** Degradation metrics */
  degradation: DegradationMetrics;

  /** Maintenance information */
  maintenance: MaintenanceInfo;

  /** Active warnings */
  warnings: string[];

  /** Active errors */
  errors: string[];
}

/**
 * Health status (simplified)
 */
export interface HealthStatus {
  overall: number;
  quantumLock: number;
  temporalFlux: number;
  beaconSignal: number;
  spatialStability: number;
}

/**
 * Quantum state information
 */
export interface QuantumState {
  /** Entanglement identifier */
  entanglementId: string;

  /** Quantum fidelity (0-1) */
  fidelity: number;

  /** Number of entangled pairs */
  pairCount: number;

  /** Quantum coherence (0-1) */
  coherence: number;

  /** Last refresh timestamp */
  lastRefresh?: number;
}

// ============================================================================
// Drift Types
// ============================================================================

/**
 * Drift vector components
 */
export interface DriftVector {
  /** Temporal drift (temporal units) */
  temporal: number;

  /** Spatial drift */
  spatial: SpatialCoordinates;

  /** Dimensional drift */
  dimensional: number;
}

/**
 * Temporal drift measurement
 */
export interface TemporalDrift {
  /** Anchor identifier */
  anchorId: string;

  /** Measurement timestamp */
  measurementTime: number;

  /** Drift components */
  drift: DriftVector;

  /** Overall drift magnitude */
  magnitude: number;

  /** 5D drift direction vector */
  direction: number[];

  /** Drift velocity (units per second) */
  velocity: number;

  /** Drift acceleration (units per second²) */
  acceleration: number;

  /** Is drift within acceptable limits? */
  acceptable: boolean;
}

/**
 * Drift correction strategy
 */
export interface DriftCorrectionStrategy {
  /** Correction method */
  method: 'gradual' | 'immediate' | 'emergency';

  /** Correction duration in seconds */
  duration: number;

  /** Number of correction steps */
  steps: number;

  /** Power required in Gigawatts */
  powerRequired: number;
}

/**
 * Drift correction plan
 */
export interface DriftCorrection {
  /** Anchor identifier */
  anchorId: string;

  /** Detected drift */
  detectedDrift: TemporalDrift;

  /** Correction strategy */
  correctionStrategy: DriftCorrectionStrategy;

  /** Correction vector to apply */
  correctionVector: DriftVector;

  /** Verification parameters */
  verification: {
    /** Expected result coordinates */
    expectedResult: AnchorCoordinates;

    /** Acceptable tolerance band */
    toleranceBand: number;

    /** Verification delay in milliseconds */
    verificationDelay: number;
  };

  /** Correction status */
  status: 'planned' | 'in-progress' | 'completed' | 'failed';

  /** Start time (Unix ms) */
  startTime?: number;

  /** Completion time (Unix ms) */
  completionTime?: number;
}

// ============================================================================
// Anchor Chain Types
// ============================================================================

/**
 * Chain topology type
 */
export type ChainTopology = 'linear' | 'branching' | 'mesh';

/**
 * Anchor chain definition
 */
export interface AnchorChain {
  /** Unique chain identifier */
  chainId: string;

  /** Chain name */
  name: string;

  /** Anchor configuration */
  anchors: {
    /** Primary anchor path */
    primary: AnchorPoint[];

    /** Backup anchor paths */
    backup: AnchorPoint[];
  };

  /** Chain topology */
  topology: ChainTopology;

  /** Chain properties */
  properties: {
    /** Total chain length in spacetime units */
    totalLength: number;

    /** Total number of anchors */
    anchorCount: number;

    /** Number of waypoint anchors */
    waypointCount: number;

    /** Redundancy level (1-5) */
    redundancyLevel: number;
  };

  /** Chain health metrics */
  health: {
    /** Overall chain integrity (0-100%) */
    overallIntegrity: number;

    /** Weakest link anchor ID */
    weakestLink: string | null;

    /** Estimated lifespan in years */
    estimatedLifespan: number;
  };

  /** Creation timestamp */
  created: number;

  /** Metadata */
  metadata?: Record<string, any>;
}

/**
 * Chain navigation state
 */
export interface ChainNavigation {
  /** Chain identifier */
  chainId: string;

  /** Current waypoint index */
  currentPosition: number;

  /** Navigation status */
  navigation: {
    /** Next waypoint */
    nextWaypoint: AnchorPoint | null;

    /** Previous waypoint */
    previousWaypoint: AnchorPoint | null;

    /** Distance to next waypoint */
    distanceToNext: number;

    /** Progress percentage (0-100) */
    progressPercent: number;
  };

  /** Route information */
  route: {
    /** Planned path */
    plannedPath: AnchorPoint[];

    /** Alternate paths */
    alternatePaths: AnchorPoint[][];

    /** Emergency exit path */
    emergencyExit: AnchorPoint[];
  };
}

/**
 * Chain options for creation
 */
export interface ChainOptions {
  /** Waypoint spacing in years */
  spacing: number;

  /** Chain topology */
  topology: ChainTopology;

  /** Redundancy level (1-5) */
  redundancyLevel: number;

  /** Waypoint anchor strength */
  waypointStrength?: number;

  /** Enable auto-failover */
  autoFailover?: boolean;
}

// ============================================================================
// Emergency Types
// ============================================================================

/**
 * Emergency priority levels
 */
export type EmergencyPriority = 'critical' | 'high' | 'medium';

/**
 * Emergency deployment method
 */
export type DeploymentMethod = 'instant' | 'rapid' | 'standard';

/**
 * Emergency anchor configuration
 */
export interface EmergencyAnchorConfig {
  /** Emergency priority */
  priority: EmergencyPriority;

  /** Deployment configuration */
  deployment: {
    /** Deployment method */
    method: DeploymentMethod;

    /** Deployment location ('auto' for automatic) */
    location: AnchorCoordinates | 'auto';

    /** Anchor strength in Gigawatts */
    strength: number;
  };

  /** Lifespan requirements */
  lifespan: {
    /** Minimum lifespan in seconds */
    minimum: number;

    /** Target lifespan in hours */
    target: number;

    /** Power budget in Gigawatts */
    powerBudget: number;
  };

  /** Notification configuration */
  notification: {
    /** User IDs to alert */
    alerts: string[];

    /** Broadcast to all users */
    broadcast: boolean;

    /** Escalate to management */
    escalation: boolean;
  };
}

/**
 * Emergency event
 */
export interface EmergencyEvent {
  /** Event identifier */
  id: string;

  /** Event timestamp */
  timestamp: number;

  /** Event type */
  type: 'anchor-failure' | 'timeline-collapse' | 'paradox' | 'drift-critical' | 'manual';

  /** Event severity (1-10) */
  severity: number;

  /** Associated anchor ID */
  anchorId?: string;

  /** Event description */
  description: string;

  /** Event data */
  data: Record<string, any>;

  /** Response status */
  status: 'detected' | 'responding' | 'resolved' | 'failed';
}

// ============================================================================
// Stability Types
// ============================================================================

/**
 * Stability report
 */
export interface StabilityReport {
  /** Report timestamp */
  timestamp: number;

  /** Anchor identifier */
  anchorId: string;

  /** Stability metrics */
  metrics: {
    /** Temporal coherence (0-100%) */
    temporalCoherence: number;

    /** Spatial coherence (0-100%) */
    spatialCoherence: number;

    /** Dimensional stability (0-100%) */
    dimensionalStability: number;

    /** Overall health (0-100%) */
    overallHealth: number;
  };

  /** Active warnings */
  warnings: string[];

  /** Active errors */
  errors: string[];

  /** Next maintenance window */
  nextMaintenanceWindow: number;
}

// ============================================================================
// API Request/Response Types
// ============================================================================

/**
 * Request to create a new anchor
 */
export interface CreateAnchorRequest {
  /** Anchor name */
  name: string;

  /** Anchor coordinates */
  coordinates: AnchorCoordinates;

  /** Temporal flux strength (1.21 - 10.0 GW) */
  strength: number;

  /** Anchor type */
  type: AnchorType;

  /** Beacon configuration */
  beacon: BeaconConfig;

  /** Additional metadata */
  metadata?: Record<string, any>;
}

/**
 * Response from anchor creation
 */
export interface CreateAnchorResponse {
  /** Created anchor */
  anchor: TimelineAnchor;

  /** Deployment time in milliseconds */
  deploymentTime: number;

  /** Success message */
  message: string;
}

/**
 * Request to update an anchor
 */
export interface UpdateAnchorRequest {
  /** New strength value */
  strength?: number;

  /** Updated beacon config */
  beacon?: Partial<BeaconConfig>;

  /** Updated metadata */
  metadata?: Record<string, any>;
}

/**
 * Request to correct drift
 */
export interface CorrectDriftRequest {
  /** Correction strategy */
  strategy: 'gradual' | 'immediate' | 'emergency';

  /** Target coordinates (optional, defaults to baseline) */
  targetCoordinates?: AnchorCoordinates;
}

/**
 * Request to create anchor chain
 */
export interface CreateChainRequest {
  /** Chain name */
  name: string;

  /** Origin coordinates */
  origin: AnchorCoordinates;

  /** Destination coordinates */
  destination: AnchorCoordinates;

  /** Chain options */
  options: ChainOptions;
}

/**
 * Request to deploy emergency anchor
 */
export interface DeployEmergencyAnchorRequest {
  /** Emergency configuration */
  config: EmergencyAnchorConfig;

  /** Reason for deployment */
  reason: string;
}

// ============================================================================
// SDK Configuration Types
// ============================================================================

/**
 * SDK environment
 */
export type SDKEnvironment = 'production' | 'staging' | 'development';

/**
 * SDK configuration
 */
export interface SDKConfig {
  /** API key for authentication */
  apiKey: string;

  /** API environment */
  environment: SDKEnvironment;

  /** Base URL (optional, auto-determined from environment) */
  baseUrl?: string;

  /** Request timeout in milliseconds */
  timeout?: number;

  /** Enable debug logging */
  debug?: boolean;

  /** Retry configuration */
  retry?: {
    /** Maximum retry attempts */
    maxAttempts: number;

    /** Initial retry delay in milliseconds */
    initialDelay: number;

    /** Maximum retry delay in milliseconds */
    maxDelay: number;
  };
}

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Pagination parameters
 */
export interface PaginationParams {
  /** Page number (1-indexed) */
  page?: number;

  /** Items per page */
  limit?: number;

  /** Sort field */
  sortBy?: string;

  /** Sort order */
  sortOrder?: 'asc' | 'desc';
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  /** Response data */
  data: T[];

  /** Pagination metadata */
  pagination: {
    /** Current page */
    page: number;

    /** Items per page */
    limit: number;

    /** Total items */
    total: number;

    /** Total pages */
    totalPages: number;

    /** Has next page */
    hasNext: boolean;

    /** Has previous page */
    hasPrevious: boolean;
  };
}

/**
 * API error response
 */
export interface APIError {
  /** Error code */
  code: string;

  /** Error message */
  message: string;

  /** Error details */
  details?: Record<string, any>;

  /** Timestamp */
  timestamp: number;
}

/**
 * Success response wrapper
 */
export interface APIResponse<T> {
  /** Success indicator */
  success: boolean;

  /** Response data */
  data?: T;

  /** Error information */
  error?: APIError;

  /** Response metadata */
  metadata?: Record<string, any>;
}

// ============================================================================
// Event Types
// ============================================================================

/**
 * Event listener callback type
 */
export type EventCallback<T> = (data: T) => void;

/**
 * WebSocket event types
 */
export type WebSocketEventType =
  | 'health-update'
  | 'drift-alert'
  | 'emergency'
  | 'beacon-signal'
  | 'maintenance-required'
  | 'anchor-degraded'
  | 'chain-update';

/**
 * Event subscription
 */
export interface EventSubscription {
  /** Subscription ID */
  id: string;

  /** Event type */
  eventType: WebSocketEventType;

  /** Anchor ID (if anchor-specific) */
  anchorId?: string;

  /** Callback function */
  callback: EventCallback<any>;

  /** Subscription timestamp */
  subscribedAt: number;
}

// ============================================================================
// Export all types
// ============================================================================

export default {
  // Re-export for convenience
  AnchorCoordinates,
  SpatialCoordinates,
  TimelineAnchor,
  AnchorPoint,
  BeaconConfig,
  BeaconSignal,
  TemporalBeacon,
  AnchorHealth,
  TemporalDrift,
  DriftCorrection,
  AnchorChain,
  EmergencyAnchorConfig,
  StabilityReport,
  SDKConfig,
};
