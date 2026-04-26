/**
 * WIA-AUTO-015: Autonomous Ship - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Maritime Autonomy Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Maritime Types
// ============================================================================

/**
 * Geographic position (WGS 84)
 */
export interface Position {
  /** Latitude in decimal degrees (-90 to 90) */
  latitude: number;

  /** Longitude in decimal degrees (-180 to 180) */
  longitude: number;

  /** Altitude/elevation in meters (optional, default: 0 for sea level) */
  altitude?: number;

  /** Position accuracy in meters */
  accuracy?: number;

  /** Timestamp of position fix */
  timestamp?: Date;
}

/**
 * Ship motion parameters
 */
export interface ShipMotion {
  /** True heading in degrees (0-360) */
  heading: number;

  /** Course over ground in degrees (0-360) */
  courseOverGround: number;

  /** Speed over ground in knots */
  speedOverGround: number;

  /** Speed through water in knots */
  speedThroughWater: number;

  /** Rate of turn in degrees per minute */
  rateOfTurn: number;
}

/**
 * Ship dimensions
 */
export interface ShipDimensions {
  /** Overall length in meters */
  length: number;

  /** Beam (width) in meters */
  beam: number;

  /** Draft in meters */
  draft: number;

  /** Height above waterline in meters */
  height?: number;

  /** Displacement tonnage */
  displacement?: number;

  /** Gross tonnage */
  grossTonnage?: number;
}

/**
 * Ship identification
 */
export interface ShipIdentification {
  /** IMO number (7-digit) */
  imo: string;

  /** MMSI number (9-digit) */
  mmsi: string;

  /** Call sign */
  callSign: string;

  /** Ship name */
  name?: string;

  /** Ship type */
  type?: ShipType;

  /** Flag state */
  flag?: string;
}

/**
 * Ship types
 */
export type ShipType =
  | 'cargo'
  | 'tanker'
  | 'passenger'
  | 'fishing'
  | 'tug'
  | 'pilot'
  | 'search-rescue'
  | 'military'
  | 'pleasure'
  | 'other';

// ============================================================================
// IMO MASS Autonomy Levels
// ============================================================================

/**
 * IMO MASS autonomy levels (0-4)
 */
export enum AutonomyLevel {
  /** Manual operation - traditional ship */
  MANUAL = 0,

  /** On-board decision support */
  DECISION_SUPPORT = 1,

  /** Remote control with seafarers on board */
  REMOTE_WITH_CREW = 2,

  /** Remote control without seafarers */
  REMOTE_WITHOUT_CREW = 3,

  /** Fully autonomous */
  FULLY_AUTONOMOUS = 4,
}

/**
 * Autonomy level transition request
 */
export interface AutonomyLevelTransition {
  /** Current autonomy level */
  currentLevel: AutonomyLevel;

  /** Requested autonomy level */
  requestedLevel: AutonomyLevel;

  /** Reason for transition */
  reason: string;

  /** Environmental conditions */
  conditions?: {
    weather: WeatherConditions;
    traffic: TrafficDensity;
    systemHealth: SystemHealth;
  };

  /** Approval status */
  approved?: boolean;

  /** Approval timestamp */
  approvalTime?: Date;
}

/**
 * Traffic density levels
 */
export type TrafficDensity = 'low' | 'medium' | 'high' | 'very-high';

/**
 * System health status
 */
export type SystemHealth = 'nominal' | 'degraded' | 'critical' | 'failed';

// ============================================================================
// Navigation
// ============================================================================

/**
 * Navigation waypoint
 */
export interface Waypoint {
  /** Waypoint identifier */
  id: string;

  /** Geographic position */
  position: Position;

  /** Planned speed at waypoint (knots) */
  speed: number;

  /** Turn radius at waypoint (nautical miles) */
  turnRadius?: number;

  /** Estimated time of arrival */
  eta?: Date;

  /** Waypoint type */
  type?: 'waypoint' | 'tsss' | 'pilot-boarding' | 'anchorage' | 'berth';
}

/**
 * Route plan
 */
export interface Route {
  /** Route identifier */
  id: string;

  /** Route name/description */
  name?: string;

  /** Origin port/position */
  origin: Position;

  /** Destination port/position */
  destination: Position;

  /** Ordered list of waypoints */
  waypoints: Waypoint[];

  /** Total route distance (nautical miles) */
  totalDistance: number;

  /** Estimated fuel consumption (metric tons) */
  estimatedFuel: number;

  /** Estimated duration (hours) */
  estimatedDuration: number;

  /** Route optimization parameters */
  optimization?: RouteOptimization;

  /** Route constraints */
  constraints?: RouteConstraints;

  /** Creation timestamp */
  created: Date;

  /** Last updated timestamp */
  updated?: Date;
}

/**
 * Route optimization parameters
 */
export interface RouteOptimization {
  /** Optimization objective */
  objective: 'fuel-efficiency' | 'time' | 'safety' | 'emission' | 'balanced';

  /** Weather routing enabled */
  weatherRouted: boolean;

  /** Avoid traffic separation schemes */
  avoidTSS?: boolean;

  /** Prefer great circle routes */
  preferGreatCircle?: boolean;
}

/**
 * Route constraints
 */
export interface RouteConstraints {
  /** Maximum wave height (meters) */
  maxWaveHeight?: number;

  /** Maximum wind speed (knots) */
  maxWindSpeed?: number;

  /** Minimum water depth (meters) */
  minWaterDepth?: number;

  /** Emission Control Area compliance */
  ecaCompliant?: boolean;

  /** Traffic Separation Scheme compliance */
  tsssCompliance?: boolean;

  /** Piracy high-risk areas avoidance */
  avoidPiracyAreas?: boolean;
}

/**
 * Cross-track error
 */
export interface CrossTrackError {
  /** Distance from planned track (nautical miles) */
  distance: number;

  /** Direction: positive = right, negative = left */
  direction: 'port' | 'starboard';

  /** Recommended course correction (degrees) */
  correction: number;
}

// ============================================================================
// Collision Avoidance
// ============================================================================

/**
 * Ship state for collision calculations
 */
export interface ShipState {
  /** Ship position */
  position: Position;

  /** True heading (degrees) */
  heading: number;

  /** Speed over ground (knots) */
  speed: number;

  /** Ship dimensions */
  dimensions?: ShipDimensions;
}

/**
 * Collision assessment
 */
export interface CollisionAssessment {
  /** Closest Point of Approach (nautical miles) */
  cpa: number;

  /** Time to Closest Point of Approach (minutes) */
  tcpa: number;

  /** Collision risk index (0-10) */
  riskIndex: number;

  /** Risk level */
  riskLevel: 'low' | 'medium' | 'high' | 'critical';

  /** COLREG situation */
  colregSituation: ColregSituation;

  /** Give-way vessel */
  giveWayVessel: 'own' | 'target' | 'both' | 'none';

  /** Stand-on vessel */
  standOnVessel: 'own' | 'target' | 'both' | 'none';

  /** Relative bearing to target (degrees) */
  relativeBearing: number;

  /** Relative speed (knots) */
  relativeSpeed: number;
}

/**
 * COLREG situation types
 */
export type ColregSituation =
  | 'head-on'
  | 'crossing'
  | 'overtaking'
  | 'safe-passing'
  | 'restricted-visibility'
  | 'unknown';

/**
 * Collision warning
 */
export interface CollisionWarning {
  /** Warning identifier */
  id: string;

  /** Warning timestamp */
  timestamp: Date;

  /** Severity level */
  severity: 'info' | 'warning' | 'critical' | 'emergency';

  /** Target vessel information */
  target: {
    mmsi?: string;
    name?: string;
    type?: ShipType;
    position: Position;
    heading: number;
    speed: number;
  };

  /** Collision assessment */
  assessment: CollisionAssessment;

  /** Recommended action */
  recommendedAction: AvoidanceManeuver;

  /** Action executed */
  actionExecuted?: boolean;

  /** Action execution time */
  executionTime?: Date;
}

/**
 * Avoidance maneuver
 */
export interface AvoidanceManeuver {
  /** Maneuver type */
  type: 'alter-course' | 'reduce-speed' | 'stop' | 'combined';

  /** New heading (if altering course) */
  newHeading?: number;

  /** Course change magnitude (degrees) */
  courseChange?: number;

  /** New speed (if reducing speed) */
  newSpeed?: number;

  /** Speed change magnitude (knots) */
  speedChange?: number;

  /** Reason for maneuver */
  reason: string;

  /** COLREG rule applied */
  colregRule?: string;

  /** Execution time */
  executionTime: Date;

  /** Duration of maneuver (seconds) */
  duration?: number;
}

/**
 * Ship domain (safety zone)
 */
export interface ShipDomain {
  /** Forward distance (nautical miles) */
  forward: number;

  /** Aft distance (nautical miles) */
  aft: number;

  /** Starboard distance (nautical miles) */
  starboard: number;

  /** Port distance (nautical miles) */
  port: number;

  /** Domain type */
  type: 'fujii' | 'goodwin' | 'custom';
}

// ============================================================================
// Sensors
// ============================================================================

/**
 * Sensor types
 */
export type SensorType =
  | 'gnss'
  | 'radar-x'
  | 'radar-s'
  | 'lidar'
  | 'camera-visual'
  | 'camera-thermal'
  | 'ais'
  | 'anemometer'
  | 'barometer'
  | 'gyro'
  | 'depth-sounder';

/**
 * Sensor status
 */
export interface SensorStatus {
  /** Sensor type */
  type: SensorType;

  /** Operational status */
  status: 'operational' | 'degraded' | 'failed' | 'offline';

  /** Health percentage (0-100) */
  health: number;

  /** Last update timestamp */
  lastUpdate: Date;

  /** Error message if failed */
  error?: string;

  /** Sensor-specific data */
  data?: Record<string, unknown>;
}

/**
 * GNSS sensor data
 */
export interface GNSSData {
  /** Position fix */
  position: Position;

  /** Fix quality */
  fixQuality: 'no-fix' | 'gps' | 'dgps' | 'rtk' | 'rtk-float';

  /** Number of satellites */
  satellites: number;

  /** Horizontal dilution of precision */
  hdop: number;

  /** Altitude above sea level (meters) */
  altitude: number;

  /** Constellations used */
  constellations: ('gps' | 'glonass' | 'galileo' | 'beidou')[];
}

/**
 * Radar target
 */
export interface RadarTarget {
  /** Target identifier */
  id: string;

  /** Relative position from own ship */
  position: Position;

  /** Distance (nautical miles) */
  distance: number;

  /** Bearing (degrees) */
  bearing: number;

  /** Course over ground (degrees) */
  course: number;

  /** Speed over ground (knots) */
  speed: number;

  /** Closest Point of Approach (nautical miles) */
  cpa: number;

  /** Time to CPA (minutes) */
  tcpa: number;

  /** Target classification */
  classification?: 'ship' | 'land' | 'buoy' | 'unknown';

  /** Tracking quality (0-1) */
  trackingQuality: number;
}

/**
 * AIS target data
 */
export interface AISTarget {
  /** MMSI number */
  mmsi: string;

  /** Ship name */
  name?: string;

  /** Call sign */
  callSign?: string;

  /** IMO number */
  imo?: string;

  /** Ship type */
  type?: ShipType;

  /** Position */
  position: Position;

  /** Course over ground (degrees) */
  cog: number;

  /** Speed over ground (knots) */
  sog: number;

  /** True heading (degrees) */
  heading: number;

  /** Navigation status */
  navStatus:
    | 'under-way-engine'
    | 'at-anchor'
    | 'not-under-command'
    | 'restricted-maneuverability'
    | 'moored'
    | 'aground';

  /** Destination */
  destination?: string;

  /** ETA */
  eta?: Date;

  /** Ship dimensions */
  dimensions?: ShipDimensions;

  /** Last update time */
  timestamp: Date;
}

/**
 * LiDAR point cloud data
 */
export interface LiDARData {
  /** Number of points */
  pointCount: number;

  /** Detected objects */
  objects: DetectedObject[];

  /** Scan timestamp */
  timestamp: Date;

  /** Range (meters) */
  range: number;

  /** Field of view (degrees) */
  fieldOfView: number;
}

/**
 * Detected object from sensors
 */
export interface DetectedObject {
  /** Object identifier */
  id: string;

  /** Object type */
  type: 'vessel' | 'buoy' | 'structure' | 'debris' | 'unknown';

  /** Position relative to ship */
  relativePosition: {
    distance: number; // meters
    bearing: number; // degrees
    elevation: number; // degrees
  };

  /** Object dimensions (if known) */
  dimensions?: {
    length: number;
    width: number;
    height: number;
  };

  /** Confidence level (0-1) */
  confidence: number;

  /** Tracking status */
  tracked: boolean;
}

// ============================================================================
// Weather
// ============================================================================

/**
 * Weather conditions
 */
export interface WeatherConditions {
  /** Wind data */
  wind: {
    speed: number; // knots
    direction: number; // degrees
    gusts: number; // knots
  };

  /** Wave data */
  waves: {
    height: number; // meters (significant wave height)
    period: number; // seconds
    direction: number; // degrees
  };

  /** Atmospheric pressure */
  pressure: {
    value: number; // hPa
    trend: 'rising' | 'falling' | 'steady';
  };

  /** Visibility */
  visibility: number; // nautical miles

  /** Precipitation */
  precipitation?: 'none' | 'light' | 'moderate' | 'heavy';

  /** Temperature */
  temperature: {
    air: number; // Celsius
    water: number; // Celsius
  };

  /** Humidity */
  humidity: number; // percentage

  /** Sea state (Beaufort scale 0-12) */
  seaState: number;

  /** Forecast timestamp */
  timestamp: Date;
}

// ============================================================================
// System Status
// ============================================================================

/**
 * Complete system status
 */
export interface SystemStatus {
  /** Overall health */
  overallHealth: SystemHealth;

  /** Current autonomy level */
  autonomyLevel: AutonomyLevel;

  /** Timestamp */
  timestamp: Date;

  /** Navigation subsystem */
  navigation: {
    status: SystemHealth;
    gnss: GNSSData;
    imu: {
      status: SystemHealth;
      drift: number; // degrees per hour
    };
    ecdis: {
      status: SystemHealth;
      chartCoverage: number; // percentage
    };
  };

  /** Propulsion subsystem */
  propulsion: {
    status: SystemHealth;
    mainEngine: {
      rpm: number;
      load: number; // percentage
      temperature: number; // Celsius
    };
    fuel: {
      level: number; // percentage
      consumptionRate: number; // tons per day
    };
  };

  /** Sensor subsystem */
  sensors: SensorStatus[];

  /** Communication subsystem */
  communication: {
    satellite: {
      status: SystemHealth;
      bandwidth: number; // Mbps
      latency: number; // milliseconds
    };
    cellular?: {
      status: SystemHealth;
      signalStrength: number; // dBm
    };
    vhf: {
      status: SystemHealth;
    };
  };

  /** Active alerts */
  alerts: SystemAlert[];
}

/**
 * System alert
 */
export interface SystemAlert {
  /** Alert identifier */
  id: string;

  /** Severity */
  severity: 'info' | 'warning' | 'critical' | 'emergency';

  /** Subsystem */
  subsystem: string;

  /** Alert message */
  message: string;

  /** Timestamp */
  timestamp: Date;

  /** Acknowledged */
  acknowledged?: boolean;

  /** Recommended action */
  recommendedAction?: string;
}

// ============================================================================
// Remote Operations
// ============================================================================

/**
 * Remote command
 */
export interface RemoteCommand {
  /** Command identifier */
  id: string;

  /** Command type */
  type:
    | 'set-course'
    | 'set-speed'
    | 'alter-route'
    | 'emergency-stop'
    | 'return-to-port'
    | 'change-autonomy-level';

  /** Command parameters */
  parameters: Record<string, unknown>;

  /** Operator identifier */
  operatorId: string;

  /** Command timestamp */
  timestamp: Date;

  /** Execution status */
  status: 'pending' | 'executing' | 'completed' | 'failed' | 'cancelled';

  /** Execution result */
  result?: {
    success: boolean;
    message?: string;
    data?: Record<string, unknown>;
  };
}

/**
 * Shore control operator
 */
export interface ShoreOperator {
  /** Operator identifier */
  id: string;

  /** Operator name */
  name: string;

  /** Certification level */
  certification: string;

  /** Active vessels under control */
  activeVessels: string[]; // IMO numbers

  /** Status */
  status: 'active' | 'on-break' | 'offline';

  /** Login timestamp */
  loginTime?: Date;
}

// ============================================================================
// Configuration
// ============================================================================

/**
 * Ship configuration
 */
export interface ShipConfig {
  /** Ship identification */
  identification: ShipIdentification;

  /** Ship dimensions */
  dimensions: ShipDimensions;

  /** Maximum autonomy level authorized */
  maxAutonomyLevel: AutonomyLevel;

  /** Performance characteristics */
  performance: {
    maxSpeed: number; // knots
    cruisingSpeed: number; // knots
    minSteerageSpeed: number; // knots
    turnRadius: number; // nautical miles at max speed
  };

  /** Safety margins */
  safetyMargins: {
    collisionDistance: number; // nautical miles
    underKeelClearance: number; // meters or percentage of draft
    crossTrackError: number; // nautical miles
  };

  /** Sensor configuration */
  sensors: {
    available: SensorType[];
    redundancy: Record<SensorType, number>;
  };
}

/**
 * Navigation parameters
 */
export interface NavigationParams {
  /** Destination position */
  destination: Position;

  /** Destination port name */
  destinationPort?: string;

  /** Maximum speed (knots) */
  maxSpeed?: number;

  /** Estimated time of arrival */
  eta?: Date;

  /** Route optimization */
  optimization?: RouteOptimization;

  /** Route constraints */
  constraints?: RouteConstraints;

  /** Safety margin (nautical miles) */
  safetyMargin?: number;
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Maritime navigation constants
 */
export const MARITIME_CONSTANTS = {
  /** Earth radius in nautical miles */
  EARTH_RADIUS_NM: 3440.065,

  /** Speed of sound in seawater (m/s) */
  SOUND_SPEED_SEAWATER: 1500,

  /** Safe passing distance (nautical miles) */
  SAFE_PASSING_DISTANCE: 2.0,

  /** COLREG visibility range (nautical miles) */
  COLREG_VISIBILITY_RANGE: 12.0,

  /** Standard gravity (m/s²) */
  GRAVITY: 9.80665,

  /** Seawater density (kg/m³) */
  SEAWATER_DENSITY: 1025,

  /** Knots to m/s conversion */
  KNOTS_TO_MPS: 0.514444,

  /** Nautical mile to meters */
  NM_TO_METERS: 1852,
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-AUTO-015 error codes
 */
export enum AutoShipErrorCode {
  // Navigation errors
  ROUTE_PLANNING_FAILED = 'A001',
  NAVIGATION_FAILURE = 'A002',
  POSITION_INVALID = 'A003',
  WAYPOINT_UNREACHABLE = 'A004',

  // Collision avoidance errors
  COLLISION_IMMINENT = 'A101',
  AVOIDANCE_FAILED = 'A102',
  COLREG_VIOLATION = 'A103',

  // Sensor errors
  SENSOR_FAILURE = 'A201',
  GNSS_UNAVAILABLE = 'A202',
  RADAR_FAILURE = 'A203',
  AIS_OFFLINE = 'A204',

  // Communication errors
  COMMUNICATION_LOST = 'A301',
  SHORE_DISCONNECTED = 'A302',
  COMMAND_TIMEOUT = 'A303',

  // System errors
  PROPULSION_FAILURE = 'A401',
  STEERING_FAILURE = 'A402',
  POWER_FAILURE = 'A403',
  AUTONOMY_DEGRADED = 'A404',

  // Security errors
  UNAUTHORIZED_ACCESS = 'A501',
  GNSS_SPOOFING = 'A502',
  CYBER_ATTACK = 'A503',
}

/**
 * Autonomous ship error
 */
export class AutonomousShipError extends Error {
  constructor(
    public code: AutoShipErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'AutonomousShipError';
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

/**
 * Time range for queries
 */
export interface TimeRange {
  /** Start time */
  start: Date;

  /** End time */
  end: Date;
}

// ============================================================================
// Export All Types
// ============================================================================

export type {
  // Core types
  Position,
  ShipMotion,
  ShipDimensions,
  ShipIdentification,
  ShipType,

  // Autonomy
  AutonomyLevelTransition,
  TrafficDensity,
  SystemHealth,

  // Navigation
  Waypoint,
  Route,
  RouteOptimization,
  RouteConstraints,
  CrossTrackError,

  // Collision avoidance
  ShipState,
  CollisionAssessment,
  ColregSituation,
  CollisionWarning,
  AvoidanceManeuver,
  ShipDomain,

  // Sensors
  SensorType,
  SensorStatus,
  GNSSData,
  RadarTarget,
  AISTarget,
  LiDARData,
  DetectedObject,

  // Weather
  WeatherConditions,

  // System
  SystemStatus,
  SystemAlert,

  // Remote operations
  RemoteCommand,
  ShoreOperator,

  // Configuration
  ShipConfig,
  NavigationParams,

  // Utilities
  TimeRange,
};

export { AutonomyLevel, AutoShipErrorCode, AutonomousShipError, MARITIME_CONSTANTS };
