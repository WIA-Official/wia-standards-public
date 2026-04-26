/**
 * WIA-DEF-003: Military Robot - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Defense & Security Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Three-dimensional vector for position and velocity
 */
export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

/**
 * Geographic coordinates
 */
export interface GeoCoordinate {
  /** Latitude in degrees */
  latitude: number;

  /** Longitude in degrees */
  longitude: number;

  /** Altitude in meters (optional) */
  altitude?: number;
}

/**
 * Robot position in space
 */
export interface RobotPosition {
  /** Geographic coordinates */
  location: GeoCoordinate;

  /** Orientation in degrees (0-360, North = 0) */
  heading: number;

  /** Pitch angle in degrees */
  pitch?: number;

  /** Roll angle in degrees */
  roll?: number;

  /** Timestamp of position */
  timestamp: Date;
}

// ============================================================================
// Robot Classification
// ============================================================================

/**
 * Robot weight class
 */
export enum WeightClass {
  Micro = 'micro',           // 0.5-5 kg
  Light = 'light',           // 5-25 kg
  Medium = 'medium',         // 25-150 kg
  Heavy = 'heavy',           // 150-500 kg
  SuperHeavy = 'super-heavy' // 500+ kg
}

/**
 * Robot functional type
 */
export enum RobotType {
  Reconnaissance = 'reconnaissance',
  EOD = 'eod',                      // Explosive Ordnance Disposal
  Logistics = 'logistics',
  CombatSupport = 'combat-support'
}

/**
 * Locomotion type
 */
export enum LocomotionType {
  Wheeled = 'wheeled',
  Tracked = 'tracked',
  Legged = 'legged',
  Hybrid = 'hybrid'
}

/**
 * Autonomy level (SAE-inspired)
 */
export enum AutonomyLevel {
  Manual = 0,                    // Full human control
  Teleoperation = 1,             // Remote control with assistance
  SupervisedAutonomy = 2,        // Supervised autonomous operation
  HighAutonomy = 3,              // High autonomy with human approval
  FullAutonomy = 4               // Full autonomy with human monitoring
}

// ============================================================================
// Robot Configuration
// ============================================================================

/**
 * Manipulator configuration
 */
export interface Manipulator {
  /** Degrees of freedom */
  dof: number;

  /** Maximum reach in meters */
  reach: number;

  /** Payload capacity in kg */
  payload: number;

  /** Has force feedback */
  hasForceFeedback: boolean;

  /** End effector type */
  endEffector: 'gripper' | 'multi-finger' | 'magnetic' | 'vacuum' | 'tool';

  /** Available tools */
  tools?: ('disruptor' | 'xray' | 'cutter' | 'hook' | 'excavator')[];

  /** Position accuracy in mm */
  accuracy?: number;
}

/**
 * Sensor suite configuration
 */
export interface SensorSuite {
  /** Camera types */
  cameras: ('RGB' | 'thermal' | 'night-vision' | 'stereo' | '360-panoramic')[];

  /** Has LIDAR */
  hasLIDAR: boolean;

  /** Has RADAR */
  hasRADAR?: boolean;

  /** Has ultrasonic sensors */
  hasUltrasonic?: boolean;

  /** Has X-ray capability */
  hasXRay?: boolean;

  /** Environmental sensors */
  environmental?: {
    chemical?: boolean;
    biological?: boolean;
    radiation?: boolean;
    explosive?: boolean;
  };

  /** Navigation sensors */
  navigation: {
    gps: boolean;
    imu: boolean;
    magnetometer?: boolean;
    rtk?: boolean; // Real-Time Kinematic GPS
  };
}

/**
 * Mobility specifications
 */
export interface MobilitySpecs {
  /** Locomotion type */
  type: LocomotionType;

  /** Maximum speed in m/s */
  maxSpeed: number;

  /** Maximum slope angle in degrees */
  maxSlope: number;

  /** Maximum obstacle height in meters */
  maxObstacleHeight: number;

  /** Maximum trench width in meters */
  maxTrenchWidth?: number;

  /** Can climb stairs */
  canClimbStairs: boolean;

  /** Turning radius in meters */
  turningRadius: number;

  /** Ground pressure in kPa */
  groundPressure?: number;
}

/**
 * Power system configuration
 */
export interface PowerSystem {
  /** Battery type */
  batteryType: 'Li-ion' | 'LiPo' | 'LiFePO4';

  /** Battery capacity in Wh */
  capacity: number;

  /** Voltage in volts */
  voltage: number;

  /** Typical endurance in hours */
  endurance: number;

  /** Charging time in hours */
  chargingTime: number;

  /** Supports hot-swap */
  hotSwap: boolean;

  /** Power consumption (W) */
  consumption: {
    idle: number;
    cruising: number;
    peak: number;
  };
}

/**
 * Communication system configuration
 */
export interface CommunicationSystem {
  /** Radio frequencies in GHz */
  frequencies: number[];

  /** Maximum range in meters */
  maxRange: number;

  /** Bandwidth in Mbps */
  bandwidth: number;

  /** Encryption standard */
  encryption: 'AES-128' | 'AES-256';

  /** Supports mesh networking */
  meshCapable: boolean;

  /** Anti-jamming capable */
  antiJam?: boolean;

  /** Maximum latency in ms */
  maxLatency: number;
}

// ============================================================================
// Military Robot
// ============================================================================

/**
 * Complete military robot definition
 */
export interface MilitaryRobot {
  /** Unique robot identifier */
  id: string;

  /** Robot name */
  name?: string;

  /** Robot type */
  type: RobotType;

  /** Weight class */
  weightClass: WeightClass;

  /** Operational weight in kg */
  weight: number;

  /** Autonomy level */
  autonomyLevel: AutonomyLevel;

  /** Mobility specifications */
  mobility: MobilitySpecs;

  /** Sensor suite */
  sensors: SensorSuite;

  /** Power system */
  power: PowerSystem;

  /** Communication system */
  communication: CommunicationSystem;

  /** Manipulator (if equipped) */
  manipulator?: Manipulator;

  /** Payload capacity in kg (for logistics robots) */
  payloadCapacity?: number;

  /** Dimensions in meters */
  dimensions: {
    length: number;
    width: number;
    height: number;
  };

  /** Current status */
  status: RobotStatus;

  /** Current position */
  position?: RobotPosition;

  /** Battery level (0-100) */
  batteryLevel?: number;

  /** Manufacturer */
  manufacturer?: string;

  /** Model designation */
  model?: string;

  /** Certification level */
  certification?: 'bronze' | 'silver' | 'gold' | 'platinum';
}

/**
 * Robot operational status
 */
export enum RobotStatus {
  Idle = 'idle',
  Ready = 'ready',
  Mission = 'mission',
  Charging = 'charging',
  Maintenance = 'maintenance',
  Error = 'error',
  Emergency = 'emergency'
}

// ============================================================================
// Mission Planning
// ============================================================================

/**
 * Mission type
 */
export enum MissionType {
  Reconnaissance = 'reconnaissance',
  EOD = 'eod',
  Logistics = 'logistics',
  Patrol = 'patrol',
  Search = 'search',
  Transport = 'transport',
  Support = 'support'
}

/**
 * Waypoint definition
 */
export interface Waypoint {
  /** Waypoint identifier */
  id: string;

  /** Geographic location */
  location: GeoCoordinate;

  /** Action at waypoint */
  action?: 'stop' | 'scan' | 'wait' | 'pickup' | 'dropoff' | 'observe';

  /** Duration in seconds (for wait action) */
  duration?: number;

  /** Speed limit approaching waypoint (m/s) */
  speedLimit?: number;

  /** Heading to maintain at waypoint (degrees) */
  heading?: number;
}

/**
 * Mission zone definition
 */
export interface MissionZone {
  /** Zone identifier */
  id: string;

  /** Zone type */
  type: 'operation' | 'no-go' | 'restricted' | 'safe';

  /** Zone boundary (polygon vertices) */
  boundary: GeoCoordinate[];

  /** Description */
  description?: string;
}

/**
 * Mission parameters
 */
export interface MissionParams {
  /** Mission identifier */
  id: string;

  /** Mission type */
  type: MissionType;

  /** Robot assigned to mission */
  robotId: string;

  /** Waypoints */
  waypoints: Waypoint[];

  /** Mission zones */
  zones?: MissionZone[];

  /** Start time */
  startTime?: Date;

  /** Maximum duration in seconds */
  maxDuration?: number;

  /** Required autonomy level */
  requiredAutonomy: AutonomyLevel;

  /** Human operator ID */
  operatorId: string;

  /** Rules of engagement */
  roe?: RulesOfEngagement;

  /** Priority (1-10, 10 = highest) */
  priority?: number;

  /** Mission status */
  status: MissionStatus;
}

/**
 * Mission status
 */
export enum MissionStatus {
  Pending = 'pending',
  Approved = 'approved',
  InProgress = 'in-progress',
  Paused = 'paused',
  Completed = 'completed',
  Aborted = 'aborted',
  Failed = 'failed'
}

/**
 * Rules of engagement
 */
export interface RulesOfEngagement {
  /** Human authorization required for critical actions */
  requireHumanAuth: boolean;

  /** Actions requiring authorization */
  authRequired?: ('force' | 'civilian-area' | 'infrastructure')[];

  /** Maximum force level */
  maxForce: 'none' | 'non-lethal' | 'lethal';

  /** Civilian proximity rules */
  civilianProximity?: {
    minDistance: number; // meters
    action: 'stop' | 'slow' | 'alert';
  };

  /** Communication check-in interval in seconds */
  checkInInterval?: number;
}

// ============================================================================
// Mission Execution
// ============================================================================

/**
 * Mission validation result
 */
export interface MissionValidation {
  /** Is mission valid */
  isValid: boolean;

  /** Validation errors */
  errors: string[];

  /** Validation warnings */
  warnings: string[];

  /** Safety checks */
  safetyChecks: SafetyCheck[];

  /** Estimated mission duration in seconds */
  estimatedDuration?: number;

  /** Estimated energy consumption in Wh */
  estimatedEnergy?: number;

  /** Battery sufficient */
  batterySufficient?: boolean;

  /** Risk level */
  riskLevel: 'low' | 'medium' | 'high' | 'extreme';

  /** Recommendations */
  recommendations: string[];
}

/**
 * Safety check result
 */
export interface SafetyCheck {
  /** Check name */
  name: string;

  /** Check status */
  status: 'pass' | 'fail' | 'warning';

  /** Description */
  description: string;

  /** Measured value */
  value?: number | string | boolean;

  /** Expected value */
  expected?: number | string | boolean;

  /** Corrective action */
  correctiveAction?: string;
}

/**
 * Mission progress
 */
export interface MissionProgress {
  /** Mission ID */
  missionId: string;

  /** Current status */
  status: MissionStatus;

  /** Current waypoint index */
  currentWaypoint: number;

  /** Total waypoints */
  totalWaypoints: number;

  /** Progress percentage (0-100) */
  progress: number;

  /** Distance traveled in meters */
  distanceTraveled: number;

  /** Time elapsed in seconds */
  timeElapsed: number;

  /** Estimated time remaining in seconds */
  timeRemaining?: number;

  /** Current robot position */
  currentPosition: RobotPosition;

  /** Battery level (0-100) */
  batteryLevel: number;

  /** Events during mission */
  events: MissionEvent[];
}

/**
 * Mission event
 */
export interface MissionEvent {
  /** Event ID */
  id: string;

  /** Timestamp */
  timestamp: Date;

  /** Event type */
  type: 'start' | 'waypoint-reached' | 'obstacle' | 'warning' | 'error' | 'abort' | 'complete';

  /** Event description */
  description: string;

  /** Location where event occurred */
  location?: GeoCoordinate;

  /** Severity */
  severity: 'info' | 'warning' | 'error' | 'critical';

  /** Additional data */
  data?: Record<string, unknown>;
}

// ============================================================================
// Efficiency & Performance
// ============================================================================

/**
 * Efficiency calculation parameters
 */
export interface EfficiencyParams {
  /** Robot type */
  robotType: RobotType;

  /** Payload weight in kg */
  payload: number;

  /** Distance to travel in meters */
  distance: number;

  /** Terrain type */
  terrain: 'paved' | 'dirt' | 'sand' | 'mud' | 'snow' | 'rubble';

  /** Average speed in m/s */
  speed?: number;

  /** Include manipulator usage */
  manipulatorUsage?: number; // percentage of mission time
}

/**
 * Efficiency calculation result
 */
export interface EfficiencyResult {
  /** Energy consumption in Wh */
  energyConsumption: number;

  /** Mission duration in seconds */
  duration: number;

  /** Energy efficiency (payload-km per Wh) */
  efficiency: number;

  /** Battery cycles required */
  batteryCycles: number;

  /** Cost estimate (arbitrary units) */
  costEstimate?: number;

  /** Comparison to human equivalent */
  humanEquivalent?: {
    personnel: number; // number of humans needed
    timeRatio: number; // time comparison (robot time / human time)
    riskReduction: number; // percentage risk reduction
  };
}

// ============================================================================
// Multi-Robot Coordination
// ============================================================================

/**
 * Robot team configuration
 */
export interface RobotTeam {
  /** Team identifier */
  id: string;

  /** Team name */
  name: string;

  /** Robot IDs in team */
  robotIds: string[];

  /** Team leader (robot ID or human operator ID) */
  leader: string;

  /** Communication topology */
  topology: 'centralized' | 'decentralized' | 'hybrid';

  /** Coordination strategy */
  strategy: 'leader-follower' | 'swarm' | 'task-allocation';

  /** Team status */
  status: 'forming' | 'ready' | 'deployed' | 'returning' | 'disbanded';
}

/**
 * Team task allocation
 */
export interface TaskAllocation {
  /** Task identifier */
  taskId: string;

  /** Assigned robot ID */
  robotId: string;

  /** Task priority (1-10) */
  priority: number;

  /** Task status */
  status: 'pending' | 'assigned' | 'in-progress' | 'completed' | 'failed';

  /** Task dependencies */
  dependencies?: string[]; // Task IDs that must complete first
}

// ============================================================================
// Telemetry & Diagnostics
// ============================================================================

/**
 * Robot telemetry data
 */
export interface RobotTelemetry {
  /** Robot ID */
  robotId: string;

  /** Timestamp */
  timestamp: Date;

  /** Position */
  position: RobotPosition;

  /** Velocity in m/s */
  velocity: number;

  /** Battery level (0-100) */
  batteryLevel: number;

  /** Battery voltage */
  batteryVoltage: number;

  /** Motor currents (amps) */
  motorCurrents?: number[];

  /** Motor temperatures (°C) */
  motorTemperatures?: number[];

  /** CPU usage (0-100) */
  cpuUsage?: number;

  /** Memory usage (0-100) */
  memoryUsage?: number;

  /** Signal strength (dBm) */
  signalStrength: number;

  /** Communication latency (ms) */
  latency: number;

  /** Active sensors */
  activeSensors: string[];

  /** Manipulator position (if equipped) */
  manipulatorPosition?: {
    joint1: number;
    joint2: number;
    joint3: number;
    joint4: number;
    joint5?: number;
    joint6?: number;
    joint7?: number;
  };

  /** Status flags */
  flags: {
    emergencyStop: boolean;
    communicationLoss: boolean;
    lowBattery: boolean;
    sensorFault: boolean;
    motorFault: boolean;
    tipOver: boolean;
  };
}

/**
 * Diagnostic report
 */
export interface DiagnosticReport {
  /** Robot ID */
  robotId: string;

  /** Report timestamp */
  timestamp: Date;

  /** Overall health (0-100) */
  health: number;

  /** System status */
  systemStatus: 'healthy' | 'degraded' | 'critical' | 'failed';

  /** Component statuses */
  components: {
    mobility: ComponentStatus;
    sensors: ComponentStatus;
    communication: ComponentStatus;
    power: ComponentStatus;
    manipulator?: ComponentStatus;
  };

  /** Faults detected */
  faults: Fault[];

  /** Maintenance recommendations */
  maintenance: string[];

  /** Next service due (hours of operation) */
  nextServiceHours?: number;
}

/**
 * Component status
 */
export interface ComponentStatus {
  /** Component name */
  name: string;

  /** Status */
  status: 'operational' | 'degraded' | 'failed';

  /** Health score (0-100) */
  health: number;

  /** Operating hours */
  hours?: number;

  /** Last service date */
  lastService?: Date;

  /** Issues */
  issues?: string[];
}

/**
 * Fault definition
 */
export interface Fault {
  /** Fault code */
  code: string;

  /** Severity */
  severity: 'info' | 'warning' | 'error' | 'critical';

  /** Description */
  description: string;

  /** First occurrence */
  firstOccurrence: Date;

  /** Occurrence count */
  count: number;

  /** Recommended action */
  action: string;

  /** Is fault cleared */
  cleared: boolean;
}

// ============================================================================
// Error Handling
// ============================================================================

/**
 * WIA-DEF-003 error codes
 */
export enum DefenseErrorCode {
  INVALID_ROBOT = 'D001',
  MISSION_VALIDATION_FAILED = 'D002',
  INSUFFICIENT_BATTERY = 'D003',
  COMMUNICATION_LOSS = 'D004',
  SENSOR_FAILURE = 'D005',
  NAVIGATION_ERROR = 'D006',
  MANIPULATOR_FAULT = 'D007',
  AUTONOMY_VIOLATION = 'D008',
  SAFETY_VIOLATION = 'D009',
  UNAUTHORIZED = 'D010',
}

/**
 * Military robot error
 */
export class MilitaryRobotError extends Error {
  constructor(
    public code: DefenseErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'MilitaryRobotError';
  }
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Physical and operational constants
 */
export const DEFENSE_CONSTANTS = {
  /** Maximum recommended mission duration (hours) */
  MAX_MISSION_DURATION: 24,

  /** Minimum battery reserve percentage */
  MIN_BATTERY_RESERVE: 20,

  /** Maximum communication range (meters) */
  MAX_COMM_RANGE: 50000,

  /** Standard communication latency threshold (ms) */
  MAX_LATENCY_TELEOPERATION: 200,

  /** Emergency stop distance at max speed (meters) */
  EMERGENCY_STOP_DISTANCE: 1,

  /** Communication loss timeout (seconds) */
  COMM_LOSS_TIMEOUT: 30,

  /** Low battery threshold (percentage) */
  LOW_BATTERY_THRESHOLD: 30,

  /** Critical battery threshold (percentage) */
  CRITICAL_BATTERY_THRESHOLD: 10,

  /** Default waypoint tolerance (meters) */
  WAYPOINT_TOLERANCE: 2,

  /** Obstacle detection range (meters) */
  OBSTACLE_DETECTION_RANGE: 20,

  /** Maximum tip-over angle (degrees) */
  MAX_TIP_ANGLE: 45,
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
// Export All
// ============================================================================

export type {
  Vector3,
  GeoCoordinate,
  RobotPosition,
  Manipulator,
  SensorSuite,
  MobilitySpecs,
  PowerSystem,
  CommunicationSystem,
  MilitaryRobot,
  Waypoint,
  MissionZone,
  MissionParams,
  RulesOfEngagement,
  MissionValidation,
  SafetyCheck,
  MissionProgress,
  MissionEvent,
  EfficiencyParams,
  EfficiencyResult,
  RobotTeam,
  TaskAllocation,
  RobotTelemetry,
  DiagnosticReport,
  ComponentStatus,
  Fault,
};

export {
  WeightClass,
  RobotType,
  LocomotionType,
  AutonomyLevel,
  RobotStatus,
  MissionType,
  MissionStatus,
  DefenseErrorCode,
  MilitaryRobotError,
  DEFENSE_CONSTANTS,
};
