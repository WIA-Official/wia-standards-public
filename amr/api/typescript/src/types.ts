/**
 * WIA-AMR Standard Type Definitions
 * @module @wia/amr-sdk/types
 * @version 1.0.0
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * 2D/3D Position with orientation
 */
export interface Position {
  /** X coordinate in meters */
  x: number;
  /** Y coordinate in meters */
  y: number;
  /** Z coordinate in meters (optional for 2D) */
  z?: number;
  /** Orientation in radians (-π to π) */
  theta: number;
  /** Reference map identifier */
  mapId?: string;
  /** Whether position is reliably initialized */
  positionInitialized?: boolean;
  /** Confidence score (0-1) */
  localizationScore?: number;
}

/**
 * Linear and angular velocity
 */
export interface Velocity {
  /** Linear velocity in m/s */
  linear?: number;
  /** Angular velocity in rad/s */
  angular?: number;
  /** X component of velocity */
  vx?: number;
  /** Y component of velocity */
  vy?: number;
  /** Rotational velocity in rad/s */
  omega?: number;
}

/**
 * Battery state information
 */
export interface BatteryState {
  /** Battery level percentage (0-100) */
  level: number;
  /** Battery voltage in volts */
  voltage?: number;
  /** Current draw in amperes */
  current?: number;
  /** Temperature in Celsius */
  temperature?: number;
  /** Whether battery is charging */
  charging?: boolean;
  /** Battery health percentage */
  health?: number;
  /** Estimated time to empty in seconds */
  timeToEmpty?: number;
  /** Estimated time to full in seconds */
  timeToFull?: number;
}

/**
 * 3D Vector
 */
export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

/**
 * Quaternion for 3D orientation
 */
export interface Quaternion {
  x: number;
  y: number;
  z: number;
  w: number;
}

// ============================================================================
// Robot State
// ============================================================================

/**
 * Robot operating states
 */
export type OperatingState =
  | 'IDLE'
  | 'NAVIGATING'
  | 'WAITING'
  | 'CHARGING'
  | 'PAUSED'
  | 'ERROR'
  | 'EMERGENCY_STOP'
  | 'MANUAL';

/**
 * Safety states
 */
export type SafetyState =
  | 'SAFE'
  | 'WARNING'
  | 'PROTECTIVE_STOP'
  | 'EMERGENCY_STOP';

/**
 * Error severity levels
 */
export type ErrorLevel = 'WARNING' | 'ERROR' | 'FATAL';

/**
 * Robot error information
 */
export interface RobotError {
  /** Error identifier */
  errorId: string;
  /** Error type code */
  errorType: string;
  /** Severity level */
  errorLevel: ErrorLevel;
  /** Human-readable description */
  errorDescription?: string;
  /** Resolution hint */
  errorHint?: string;
  /** When error occurred */
  timestamp?: string;
}

/**
 * Load/payload on robot
 */
export interface Load {
  /** Load identifier */
  loadId?: string;
  /** Type of load */
  loadType?: string;
  /** Weight in kg */
  weight?: number;
  /** Dimensions in meters */
  dimensions?: {
    length: number;
    width: number;
    height: number;
  };
}

/**
 * Complete robot state
 */
export interface RobotState {
  /** Unique robot identifier */
  robotId: string;
  /** Manufacturer name */
  manufacturer?: string;
  /** Serial number */
  serialNumber?: string;
  /** Robot model */
  model?: string;
  /** Software version */
  softwareVersion?: string;
  /** Current position */
  position: Position;
  /** Current velocity */
  velocity?: Velocity;
  /** Battery state */
  battery?: BatteryState;
  /** Operating state */
  operatingState: OperatingState;
  /** Safety state */
  safetyState?: SafetyState;
  /** Whether robot is moving */
  driving?: boolean;
  /** Whether robot is paused */
  paused?: boolean;
  /** Current task ID */
  currentTaskId?: string;
  /** Last completed task ID */
  lastCompletedTaskId?: string;
  /** Current loads */
  loads?: Load[];
  /** Active errors */
  errors?: RobotError[];
  /** State timestamp (ISO 8601) */
  timestamp: string;
  /** Vendor extensions */
  extensions?: Record<string, unknown>;
}

// ============================================================================
// Task Types
// ============================================================================

/**
 * Task types
 */
export type TaskType =
  | 'NAVIGATE'
  | 'PICK'
  | 'PLACE'
  | 'DOCK'
  | 'UNDOCK'
  | 'WAIT'
  | 'CHARGE'
  | 'CUSTOM';

/**
 * Task status
 */
export type TaskStatus =
  | 'PENDING'
  | 'ASSIGNED'
  | 'IN_PROGRESS'
  | 'COMPLETED'
  | 'FAILED'
  | 'CANCELLED';

/**
 * Action types
 */
export type ActionType =
  | 'LIFT'
  | 'DROP'
  | 'BEEP'
  | 'WAIT'
  | 'SCAN'
  | 'OPEN_DOOR'
  | 'CLOSE_DOOR'
  | 'CUSTOM';

/**
 * Blocking type for actions
 */
export type BlockingType = 'HARD' | 'SOFT' | 'NONE';

/**
 * Action to perform at destination
 */
export interface Action {
  /** Action identifier */
  actionId?: string;
  /** Action type */
  actionType: ActionType;
  /** Action parameters */
  parameters?: Record<string, unknown>;
  /** Blocking type */
  blockingType?: BlockingType;
}

/**
 * Task constraints
 */
export interface TaskConstraints {
  /** Deadline (ISO 8601) */
  deadline?: string;
  /** Maximum duration in seconds */
  maxDuration?: number;
  /** Allowed zone IDs */
  allowedZones?: string[];
  /** Forbidden zone IDs */
  forbiddenZones?: string[];
  /** Required robot capabilities */
  requiredCapabilities?: string[];
  /** Maximum speed in m/s */
  maxSpeed?: number;
}

/**
 * External system reference
 */
export interface ExternalRef {
  /** Source system name */
  system: string;
  /** Order/task ID in source system */
  orderId?: string;
  /** Line item ID */
  lineItemId?: string;
}

/**
 * Task definition
 */
export interface Task {
  /** Unique task identifier */
  taskId: string;
  /** Task type */
  taskType: TaskType;
  /** Priority (0-100, higher = more urgent) */
  priority?: number;
  /** Assigned robot ID */
  robotId?: string;
  /** Source location */
  source?: Position;
  /** Destination location */
  destination?: Position;
  /** Actions at destination */
  actions?: Action[];
  /** Task constraints */
  constraints?: TaskConstraints;
  /** Current status */
  status?: TaskStatus;
  /** Completion percentage */
  progress?: number;
  /** External reference */
  externalRef?: ExternalRef;
  /** Task-specific payload */
  payload?: Record<string, unknown>;
  /** Creation time */
  createdAt?: string;
  /** Assignment time */
  assignedAt?: string;
  /** Start time */
  startedAt?: string;
  /** Completion time */
  completedAt?: string;
}

// ============================================================================
// Command Types
// ============================================================================

/**
 * Command types
 */
export type CommandType =
  | 'NAVIGATE'
  | 'PAUSE'
  | 'RESUME'
  | 'STOP'
  | 'EMERGENCY_STOP'
  | 'DOCK'
  | 'UNDOCK';

/**
 * Command status
 */
export type CommandStatus =
  | 'PENDING'
  | 'ACCEPTED'
  | 'REJECTED'
  | 'EXECUTING'
  | 'COMPLETED'
  | 'FAILED';

/**
 * Robot command
 */
export interface Command {
  /** Command identifier */
  commandId: string;
  /** Command type */
  commandType: CommandType;
  /** Target destination */
  destination?: Position;
  /** Command parameters */
  parameters?: Record<string, unknown>;
  /** Command timestamp */
  timestamp: string;
}

/**
 * Command acknowledgment
 */
export interface CommandAck {
  /** Command identifier */
  commandId: string;
  /** Acknowledgment status */
  status: CommandStatus;
  /** Response timestamp */
  timestamp: string;
  /** Estimated duration in seconds */
  estimatedDuration?: number;
  /** Error code if rejected/failed */
  errorCode?: string;
  /** Error message */
  errorMessage?: string;
}

// ============================================================================
// Map Types
// ============================================================================

/**
 * Map node
 */
export interface MapNode {
  /** Node identifier */
  nodeId: string;
  /** Node position */
  position: Position;
  /** Node name */
  name?: string;
  /** Node type */
  nodeType?: string;
  /** Associated actions */
  actions?: Action[];
}

/**
 * Map edge
 */
export interface MapEdge {
  /** Edge identifier */
  edgeId: string;
  /** Start node ID */
  startNodeId: string;
  /** End node ID */
  endNodeId: string;
  /** Maximum speed on edge */
  maxSpeed?: number;
  /** Edge type */
  edgeType?: string;
  /** Is bidirectional */
  bidirectional?: boolean;
}

/**
 * Zone type
 */
export type ZoneType =
  | 'CHARGING'
  | 'LOADING'
  | 'UNLOADING'
  | 'FORBIDDEN'
  | 'SPEED_LIMIT'
  | 'CUSTOM';

/**
 * Map zone
 */
export interface MapZone {
  /** Zone identifier */
  zoneId: string;
  /** Zone name */
  name?: string;
  /** Zone type */
  zoneType: ZoneType;
  /** Polygon vertices */
  polygon: Position[];
  /** Speed limit in m/s */
  speedLimit?: number;
}

/**
 * Environment map
 */
export interface Map {
  /** Map identifier */
  mapId: string;
  /** Map name */
  name?: string;
  /** Map version */
  version?: string;
  /** Map origin in world coordinates */
  origin?: Position;
  /** Grid resolution in m/cell */
  resolution?: number;
  /** Navigation nodes */
  nodes?: MapNode[];
  /** Navigation edges */
  edges?: MapEdge[];
  /** Zones */
  zones?: MapZone[];
}

// ============================================================================
// Telemetry Types
// ============================================================================

/**
 * LiDAR sensor data
 */
export interface LidarData {
  /** Range measurements in meters */
  ranges: number[];
  /** Minimum angle in radians */
  angleMin: number;
  /** Maximum angle in radians */
  angleMax: number;
  /** Angle increment in radians */
  angleIncrement: number;
  /** Minimum range in meters */
  rangeMin?: number;
  /** Maximum range in meters */
  rangeMax?: number;
}

/**
 * IMU sensor data
 */
export interface ImuData {
  /** Orientation quaternion */
  orientation?: Quaternion;
  /** Angular velocity */
  angularVelocity?: Vector3;
  /** Linear acceleration */
  linearAcceleration?: Vector3;
}

/**
 * Odometry data
 */
export interface OdometryData {
  /** Pose */
  pose?: Position;
  /** Twist (velocity) */
  twist?: {
    linear?: Vector3;
    angular?: Vector3;
  };
}

/**
 * System metrics
 */
export interface SystemMetrics {
  /** CPU usage percentage */
  cpuUsage?: number;
  /** Memory usage percentage */
  memoryUsage?: number;
  /** Disk usage percentage */
  diskUsage?: number;
  /** Network latency in ms */
  networkLatency?: number;
  /** Uptime in seconds */
  uptime?: number;
}

/**
 * Telemetry data
 */
export interface Telemetry {
  /** Robot identifier */
  robotId: string;
  /** Telemetry timestamp */
  timestamp: string;
  /** Sensor data */
  sensors?: {
    lidar?: LidarData;
    imu?: ImuData;
    odometry?: OdometryData;
  };
  /** System metrics */
  system?: SystemMetrics;
}

// ============================================================================
// Event Types
// ============================================================================

/**
 * Event types
 */
export type EventType =
  | 'task.created'
  | 'task.assigned'
  | 'task.started'
  | 'task.completed'
  | 'task.failed'
  | 'task.cancelled'
  | 'robot.state.changed'
  | 'robot.position.updated'
  | 'robot.error'
  | 'robot.battery.low'
  | 'inventory.moved'
  | 'zone.entered'
  | 'zone.exited';

/**
 * Event source information
 */
export interface EventSource {
  /** Source system name */
  system: string;
  /** Instance identifier */
  instance?: string;
  /** Version */
  version?: string;
}

/**
 * WIA-AMR Event
 */
export interface WiaEvent<T = unknown> {
  /** Unique event identifier */
  eventId: string;
  /** Event type */
  eventType: EventType;
  /** Event timestamp */
  timestamp: string;
  /** Event source */
  source: EventSource;
  /** Correlation ID for tracking */
  correlationId?: string;
  /** Causation ID */
  causationId?: string;
  /** Event data payload */
  data: T;
  /** Additional metadata */
  metadata?: Record<string, unknown>;
}

// ============================================================================
// API Types
// ============================================================================

/**
 * Pagination information
 */
export interface Pagination {
  /** Total count */
  total: number;
  /** Items per page */
  limit: number;
  /** Current offset */
  offset: number;
  /** Has more pages */
  hasMore: boolean;
  /** Navigation links */
  links?: {
    first?: string;
    prev?: string;
    next?: string;
    last?: string;
  };
}

/**
 * API response wrapper
 */
export interface ApiResponse<T> {
  /** Response data */
  data: T;
  /** Pagination (for list responses) */
  pagination?: Pagination;
}

/**
 * API error
 */
export interface ApiError {
  /** Error code */
  code: string;
  /** Error message */
  message: string;
  /** Additional details */
  details?: Record<string, unknown>;
  /** Request ID */
  requestId?: string;
  /** Error timestamp */
  timestamp: string;
}

/**
 * API error response
 */
export interface ApiErrorResponse {
  error: ApiError;
}

// ============================================================================
// Fleet Types
// ============================================================================

/**
 * Fleet summary
 */
export interface FleetSummary {
  /** Timestamp */
  timestamp: string;
  /** Fleet statistics */
  fleet: {
    totalRobots: number;
    activeRobots: number;
    chargingRobots: number;
    errorRobots: number;
    idleRobots: number;
    utilizationRate: number;
  };
  /** Task statistics */
  tasks: {
    pending: number;
    inProgress: number;
    completedToday: number;
    failedToday: number;
    averageCompletionTime: number;
    throughputPerHour: number;
  };
  /** Performance metrics */
  performance: {
    totalDistanceKm: number;
    energyConsumedKwh: number;
    averageSpeed: number;
  };
  /** Alerts summary */
  alerts: {
    critical: number;
    warning: number;
    info: number;
  };
}
