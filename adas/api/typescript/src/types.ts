/**
 * WIA-AUTO-002: ADAS - Advanced Driver Assistance System - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Automotive Safety Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * 3D position vector
 */
export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

/**
 * 2D position vector
 */
export interface Vector2 {
  x: number;
  y: number;
}

/**
 * Object dimensions
 */
export interface Dimensions {
  length: number; // meters
  width: number; // meters
  height: number; // meters
}

/**
 * Timestamp in microseconds
 */
export type Timestamp = number;

// ============================================================================
// Sensor Types
// ============================================================================

/**
 * LiDAR sensor configuration
 */
export interface LiDARConfig {
  enabled: boolean;
  range: number; // meters
  fov: number; // degrees
  frequency: number; // Hz
  resolution: number; // degrees
  wavelength?: 905 | 1550; // nanometers
}

/**
 * LiDAR point
 */
export interface LiDARPoint {
  x: number; // meters
  y: number; // meters
  z: number; // meters
  intensity: number; // 0-255
  timestamp: Timestamp;
  ring?: number; // laser ring ID
}

/**
 * LiDAR point cloud
 */
export interface LiDARPointCloud {
  points: LiDARPoint[];
  timestamp: Timestamp;
  frameId: string;
}

/**
 * Radar sensor configuration
 */
export interface RadarConfig {
  enabled: boolean;
  range: number; // meters
  fov: number; // degrees
  frequency: number; // GHz
  rangeResolution: number; // meters
  velocityResolution: number; // m/s
}

/**
 * Radar detection target
 */
export interface RadarTarget {
  range: number; // meters
  azimuth: number; // radians
  elevation: number; // radians
  rangeRate: number; // m/s (radial velocity)
  rcs: number; // dBsm (Radar Cross Section)
  snr: number; // dB (Signal-to-Noise Ratio)
  timestamp: Timestamp;
}

/**
 * Radar detection list
 */
export interface RadarDetection {
  targets: RadarTarget[];
  timestamp: Timestamp;
  sensorId: string;
}

/**
 * Camera sensor configuration
 */
export interface CameraConfig {
  enabled: boolean;
  resolution: string; // e.g., "1920x1080"
  fov: number; // degrees
  fps: number; // frames per second
  exposureMode?: 'auto' | 'manual';
  hdr?: boolean;
}

/**
 * Camera intrinsic parameters
 */
export interface CameraIntrinsics {
  fx: number; // focal length x
  fy: number; // focal length y
  cx: number; // principal point x
  cy: number; // principal point y
  k1?: number; // radial distortion
  k2?: number; // radial distortion
  p1?: number; // tangential distortion
  p2?: number; // tangential distortion
}

/**
 * Camera image
 */
export interface CameraImage {
  width: number;
  height: number;
  encoding: 'rgb8' | 'bgr8' | 'mono8' | 'rgba8';
  data: Uint8Array | ArrayBuffer;
  timestamp: Timestamp;
  cameraId: string;
  intrinsics?: CameraIntrinsics;
}

/**
 * Ultrasonic sensor configuration
 */
export interface UltrasonicConfig {
  enabled: boolean;
  range: number; // meters
  beamWidth: number; // degrees
  frequency: number; // kHz
}

/**
 * Ultrasonic measurement
 */
export interface UltrasonicMeasurement {
  distance: number; // meters
  sensorId: string;
  timestamp: Timestamp;
  valid: boolean;
}

/**
 * All sensor configurations
 */
export interface SensorConfiguration {
  lidar?: LiDARConfig;
  radar?: RadarConfig;
  camera?: CameraConfig;
  ultrasonic?: UltrasonicConfig;
}

/**
 * Sensor data bundle
 */
export interface SensorData {
  lidar?: LiDARPointCloud;
  radar?: RadarDetection;
  camera?: CameraImage;
  ultrasonic?: UltrasonicMeasurement[];
  timestamp: Timestamp;
}

// ============================================================================
// Object Detection Types
// ============================================================================

/**
 * Object classification
 */
export enum ObjectClass {
  UNKNOWN = 0,
  CAR = 1,
  TRUCK = 2,
  BUS = 3,
  MOTORCYCLE = 4,
  BICYCLE = 5,
  PEDESTRIAN = 6,
  ANIMAL = 7,
  TRAFFIC_SIGN = 8,
  TRAFFIC_LIGHT = 9,
  ROAD_BARRIER = 10,
}

/**
 * Object class names
 */
export const ObjectClassName: Record<ObjectClass, string> = {
  [ObjectClass.UNKNOWN]: 'unknown',
  [ObjectClass.CAR]: 'car',
  [ObjectClass.TRUCK]: 'truck',
  [ObjectClass.BUS]: 'bus',
  [ObjectClass.MOTORCYCLE]: 'motorcycle',
  [ObjectClass.BICYCLE]: 'bicycle',
  [ObjectClass.PEDESTRIAN]: 'pedestrian',
  [ObjectClass.ANIMAL]: 'animal',
  [ObjectClass.TRAFFIC_SIGN]: 'traffic_sign',
  [ObjectClass.TRAFFIC_LIGHT]: 'traffic_light',
  [ObjectClass.ROAD_BARRIER]: 'road_barrier',
};

/**
 * Detected object
 */
export interface DetectedObject {
  /** Unique object ID */
  id: number;

  /** Object classification */
  class: ObjectClass;

  /** Classification confidence (0-1) */
  classConfidence: number;

  /** Position in vehicle coordinate system */
  position: Vector3;

  /** Velocity vector */
  velocity: Vector3;

  /** Acceleration vector */
  acceleration: Vector3;

  /** Object dimensions */
  dimensions: Dimensions;

  /** Heading angle in radians */
  heading: number;

  /** State covariance matrix (6x6) */
  covariance?: number[][];

  /** Tracking age in seconds */
  trackingAge: number;

  /** Predicted trajectory */
  prediction?: TrajectoryPrediction;
}

/**
 * Trajectory prediction
 */
export interface TrajectoryPrediction {
  /** Prediction horizon in seconds */
  horizon: number;

  /** Predicted waypoints */
  trajectory: Array<{
    t: number; // time in seconds
    x: number; // position x in meters
    y: number; // position y in meters
  }>;

  /** Confidence in prediction (0-1) */
  confidence: number;
}

/**
 * Object list
 */
export interface ObjectList {
  objects: DetectedObject[];
  timestamp: Timestamp;
  frameId: string;
}

// ============================================================================
// Lane Detection Types
// ============================================================================

/**
 * Lane marking type
 */
export enum LaneMarkingType {
  UNKNOWN = 'unknown',
  SOLID = 'solid',
  DASHED = 'dashed',
  DOUBLE_SOLID = 'double_solid',
  SOLID_DASHED = 'solid_dashed',
  DASHED_SOLID = 'dashed_solid',
  BOTTS_DOTS = 'botts_dots',
}

/**
 * Lane marking color
 */
export enum LaneMarkingColor {
  UNKNOWN = 'unknown',
  WHITE = 'white',
  YELLOW = 'yellow',
  BLUE = 'blue',
  RED = 'red',
}

/**
 * Lane boundary
 */
export interface LaneBoundary {
  /** Polynomial coefficients: x(y) = c0 + c1*y + c2*y^2 + c3*y^3 */
  coefficients: number[];

  /** Marking type */
  type: LaneMarkingType;

  /** Marking color */
  color: LaneMarkingColor;

  /** Detection confidence (0-1) */
  confidence: number;

  /** Valid range in y direction */
  validRange: {
    yMin: number; // meters
    yMax: number; // meters
  };
}

/**
 * Lane information
 */
export interface LaneInfo {
  /** Left boundary of ego lane */
  egoLeft?: LaneBoundary;

  /** Right boundary of ego lane */
  egoRight?: LaneBoundary;

  /** Left adjacent lane boundary */
  adjacentLeft?: LaneBoundary;

  /** Right adjacent lane boundary */
  adjacentRight?: LaneBoundary;

  /** Ego vehicle position in lane */
  egoPosition: {
    /** Lateral offset from lane center (negative = left) */
    lateralOffset: number; // meters

    /** Heading angle relative to lane */
    headingAngle: number; // radians
  };

  timestamp: Timestamp;
}

// ============================================================================
// ADAS Feature Types
// ============================================================================

/**
 * SAE automation level
 */
export enum SAELevel {
  LEVEL_0 = 0, // No Automation
  LEVEL_1 = 1, // Driver Assistance
  LEVEL_2 = 2, // Partial Automation
  LEVEL_3 = 3, // Conditional Automation
  LEVEL_4 = 4, // High Automation
  LEVEL_5 = 5, // Full Automation
}

/**
 * Road condition
 */
export enum RoadCondition {
  DRY = 'dry',
  WET = 'wet',
  SNOW = 'snow',
  ICE = 'ice',
  UNKNOWN = 'unknown',
}

/**
 * Collision risk level
 */
export enum CollisionRisk {
  NONE = 'none',
  LOW = 'low',
  MEDIUM = 'medium',
  HIGH = 'high',
  CRITICAL = 'critical',
}

/**
 * Time-to-Collision result
 */
export interface TTCResult {
  /** Time to collision in seconds */
  ttc: number;

  /** Distance to object */
  distance: number;

  /** Relative velocity */
  relativeVelocity: number;

  /** Target object ID */
  objectId: number;

  /** Risk level */
  risk: CollisionRisk;
}

/**
 * Collision avoidance result
 */
export interface CollisionAvoidanceResult {
  /** Should activate emergency braking? */
  shouldBrake: boolean;

  /** Recommended braking force (0-1) */
  brakingForce: number;

  /** TTC calculation */
  ttc: TTCResult;

  /** Safe following distance */
  safeDistance: number;

  /** Warning level */
  warningLevel: 'none' | 'visual' | 'audible' | 'haptic' | 'emergency';
}

/**
 * Lane departure status
 */
export interface LaneDepartureStatus {
  /** Is vehicle departing lane? */
  isDeparting: boolean;

  /** Time to lane crossing in seconds */
  tlc: number;

  /** Departure direction */
  direction: 'left' | 'right' | 'none';

  /** Warning level */
  warningLevel: 'none' | 'visual' | 'haptic' | 'audible';
}

/**
 * Adaptive Cruise Control state
 */
export enum ACCState {
  OFF = 'off',
  STANDBY = 'standby',
  CRUISE = 'cruise',
  FOLLOWING = 'following',
  APPROACHING = 'approaching',
  STANDSTILL = 'standstill',
  OVERRIDE = 'override',
  FAULT = 'fault',
}

/**
 * ACC configuration
 */
export interface ACCConfig {
  /** Set speed in km/h */
  setSpeed: number;

  /** Time gap in seconds (1.0 - 2.5) */
  timeGap: number;

  /** Maximum speed in km/h */
  maxSpeed: number;

  /** Minimum speed in km/h (for Stop&Go) */
  minSpeed: number;

  /** Enable Stop&Go functionality */
  stopAndGo: boolean;
}

/**
 * ACC status
 */
export interface ACCStatus {
  /** Current state */
  state: ACCState;

  /** Target object (if following) */
  targetObject?: {
    id: number;
    distance: number; // meters
    relativeVelocity: number; // m/s
  };

  /** Commanded acceleration */
  commandedAcceleration: number; // m/s²

  /** Current ego velocity */
  egoVelocity: number; // m/s

  /** Time gap to target */
  actualTimeGap?: number; // seconds

  timestamp: Timestamp;
}

/**
 * Lane Keeping Assist state
 */
export enum LKAState {
  OFF = 'off',
  STANDBY = 'standby',
  ACTIVE = 'active',
  INTERVENTION = 'intervention',
  FAULT = 'fault',
}

/**
 * LKA status
 */
export interface LKAStatus {
  /** Current state */
  state: LKAState;

  /** Commanded steering angle */
  commandedSteering: number; // radians

  /** Lateral offset from center */
  lateralOffset: number; // meters

  /** Heading error */
  headingError: number; // radians

  /** Lane confidence */
  laneConfidence: number; // 0-1

  timestamp: Timestamp;
}

// ============================================================================
// ADAS System Configuration
// ============================================================================

/**
 * ADAS system configuration
 */
export interface ADASConfig {
  /** Sensor configuration */
  sensors: SensorConfiguration;

  /** SAE automation level */
  safetyLevel: SAELevel;

  /** Enable specific features */
  features?: {
    acc?: boolean;
    lka?: boolean;
    aeb?: boolean;
    fcw?: boolean;
    ldw?: boolean;
    bsa?: boolean; // Blind Spot Assist
    psa?: boolean; // Parking Assist
  };

  /** Detection parameters */
  detection?: {
    maxRange?: number; // meters
    minConfidence?: number; // 0-1
    trackingTimeout?: number; // seconds
  };

  /** Safety parameters */
  safety?: {
    reactionTime?: number; // seconds
    maxDeceleration?: number; // m/s²
    maxJerk?: number; // m/s³
  };
}

/**
 * ADAS system state
 */
export interface ADASState {
  /** System initialization status */
  initialized: boolean;

  /** System health */
  health: 'healthy' | 'degraded' | 'fault';

  /** Active warnings */
  activeWarnings: string[];

  /** Feature states */
  features: {
    acc?: ACCStatus;
    lka?: LKAStatus;
  };

  /** Last update timestamp */
  timestamp: Timestamp;
}

// ============================================================================
// Processing Results
// ============================================================================

/**
 * Sensor fusion result
 */
export interface SensorFusionResult {
  /** Fused object list */
  objects: DetectedObject[];

  /** Lane information */
  lanes: LaneInfo;

  /** Timestamp */
  timestamp: Timestamp;

  /** Processing time in milliseconds */
  processingTime: number;
}

/**
 * ADAS processing result
 */
export interface ADASProcessingResult {
  /** Detected objects */
  objects: DetectedObject[];

  /** Lane information */
  lanes: LaneInfo;

  /** Collision avoidance */
  collision?: CollisionAvoidanceResult;

  /** Lane departure */
  laneDeparture?: LaneDepartureStatus;

  /** ACC status */
  acc?: ACCStatus;

  /** LKA status */
  lka?: LKAStatus;

  /** System state */
  systemState: ADASState;

  /** Timestamp */
  timestamp: Timestamp;
}

/**
 * Safe distance calculation parameters
 */
export interface SafeDistanceParams {
  /** Vehicle velocity in m/s */
  velocity: number;

  /** Road condition */
  roadCondition: RoadCondition;

  /** Reaction time in seconds */
  reactionTime?: number;

  /** Safety margin multiplier */
  safetyMargin?: number;
}

/**
 * Safe distance result
 */
export interface SafeDistanceResult {
  /** Total safe distance */
  totalDistance: number; // meters

  /** Reaction distance */
  reactionDistance: number; // meters

  /** Braking distance */
  brakingDistance: number; // meters

  /** Friction coefficient used */
  frictionCoefficient: number;

  /** Maximum deceleration */
  maxDeceleration: number; // m/s²
}

// ============================================================================
// Validation and Safety
// ============================================================================

/**
 * System validation result
 */
export interface ValidationResult {
  /** Is system ready? */
  isValid: boolean;

  /** Validation errors */
  errors: string[];

  /** Validation warnings */
  warnings: string[];

  /** Sensor health */
  sensorHealth: {
    lidar?: 'healthy' | 'degraded' | 'fault';
    radar?: 'healthy' | 'degraded' | 'fault';
    camera?: 'healthy' | 'degraded' | 'fault';
    ultrasonic?: 'healthy' | 'degraded' | 'fault';
  };

  /** Recommended actions */
  recommendations: string[];
}

/**
 * Safety check result
 */
export interface SafetyCheck {
  /** Check name */
  name: string;

  /** Status */
  status: 'pass' | 'fail' | 'warning';

  /** Description */
  description: string;

  /** Measured value */
  value?: number;

  /** Expected value */
  expected?: number;

  /** Corrective action */
  correctiveAction?: string;
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Physical constants for ADAS calculations
 */
export const ADAS_CONSTANTS = {
  /** Speed of light (m/s) */
  SPEED_OF_LIGHT: 299792458,

  /** Gravitational acceleration (m/s²) */
  GRAVITY: 9.81,

  /** Standard reaction time (seconds) */
  REACTION_TIME: 1.5,

  /** Friction coefficients */
  FRICTION: {
    DRY_ASPHALT: 0.85,
    WET_ASPHALT: 0.6,
    SNOW: 0.25,
    ICE: 0.12,
  },

  /** Safety margins */
  SAFETY: {
    MIN_DISTANCE: 2.0, // meters
    MIN_TIME_GAP: 1.0, // seconds
    MAX_DECELERATION: 10.0, // m/s²
    MAX_JERK: 10.0, // m/s³
  },

  /** Sensor ranges (meters) */
  SENSOR_RANGE: {
    LIDAR: 200,
    RADAR: 150,
    CAMERA: 100,
    ULTRASONIC: 5,
  },

  /** Detection thresholds */
  DETECTION: {
    MIN_CONFIDENCE: 0.7,
    MIN_POINTS: 10, // minimum points per cluster
    MAX_TRACK_AGE: 5.0, // seconds
  },

  /** TTC thresholds (seconds) */
  TTC_THRESHOLDS: {
    MONITOR: 4.0,
    WARN: 2.5,
    ALERT: 1.5,
    BRAKE: 0.8,
  },
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * ADAS error codes
 */
export enum ADASErrorCode {
  SENSOR_FAILURE = 'A001',
  CALIBRATION_ERROR = 'A002',
  PROCESSING_TIMEOUT = 'A003',
  INVALID_INPUT = 'A004',
  INSUFFICIENT_DATA = 'A005',
  SYSTEM_OVERLOAD = 'A006',
  FUSION_FAILURE = 'A007',
  SAFETY_VIOLATION = 'A008',
}

/**
 * ADAS error
 */
export class ADASError extends Error {
  constructor(
    public code: ADASErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'ADASError';
  }
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
// Export All
// ============================================================================

export type {
  // Core
  Vector3,
  Vector2,
  Dimensions,
  Timestamp,

  // Sensors
  LiDARConfig,
  LiDARPoint,
  LiDARPointCloud,
  RadarConfig,
  RadarTarget,
  RadarDetection,
  CameraConfig,
  CameraIntrinsics,
  CameraImage,
  UltrasonicConfig,
  UltrasonicMeasurement,
  SensorConfiguration,
  SensorData,

  // Objects
  DetectedObject,
  TrajectoryPrediction,
  ObjectList,

  // Lanes
  LaneBoundary,
  LaneInfo,

  // ADAS Features
  TTCResult,
  CollisionAvoidanceResult,
  LaneDepartureStatus,
  ACCConfig,
  ACCStatus,
  LKAStatus,

  // System
  ADASConfig,
  ADASState,
  SensorFusionResult,
  ADASProcessingResult,
  SafeDistanceParams,
  SafeDistanceResult,

  // Validation
  ValidationResult,
  SafetyCheck,
};

export {
  ObjectClass,
  ObjectClassName,
  LaneMarkingType,
  LaneMarkingColor,
  SAELevel,
  RoadCondition,
  CollisionRisk,
  ACCState,
  LKAState,
  ADAS_CONSTANTS,
  ADASErrorCode,
  ADASError,
};
