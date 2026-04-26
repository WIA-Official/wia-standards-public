/**
 * WIA-AUTO-017: Delivery Drone - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Autonomous Vehicle Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Geographic Types
// ============================================================================

/**
 * Geographic coordinate with altitude
 */
export interface GeoCoordinate {
  /** Latitude in decimal degrees (-90 to 90) */
  latitude: number;

  /** Longitude in decimal degrees (-180 to 180) */
  longitude: number;

  /** Altitude above mean sea level in meters */
  altitude_msl?: number;

  /** Altitude above ground level in meters */
  altitude_agl?: number;
}

/**
 * Short-form coordinate (lat, lng)
 */
export interface LatLng {
  lat: number;
  lng: number;
  alt?: number;
}

/**
 * 3D position with velocity
 */
export interface Position3D {
  /** X coordinate (East, meters) */
  x: number;

  /** Y coordinate (North, meters) */
  y: number;

  /** Z coordinate (Up, meters) */
  z: number;

  /** Velocity vector (optional) */
  velocity?: {
    vx: number;
    vy: number;
    vz: number;
  };

  /** Heading in degrees (0-360, 0=North) */
  heading?: number;
}

/**
 * Physical dimensions
 */
export interface Dimensions {
  /** Length in centimeters */
  length: number;

  /** Width in centimeters */
  width: number;

  /** Height in centimeters */
  height: number;

  /** Unit of measurement */
  unit?: 'cm' | 'm' | 'in';
}

// ============================================================================
// Drone Specifications
// ============================================================================

/**
 * Drone classification by weight
 */
export type DroneClass = 'micro' | 'light' | 'medium' | 'heavy';

/**
 * Propulsion system type
 */
export type PropulsionType = 'multi-rotor' | 'fixed-wing' | 'hybrid-vtol' | 'tilt-rotor';

/**
 * Power system type
 */
export type PowerSystem = 'electric-battery' | 'hybrid-gas-electric' | 'hydrogen-fuel-cell' | 'solar-assisted';

/**
 * Drone specifications
 */
export interface DroneSpecification {
  /** Unique drone identifier */
  drone_id: string;

  /** Drone model name */
  model: string;

  /** Manufacturer */
  manufacturer: string;

  /** Drone classification */
  class: DroneClass;

  /** Maximum takeoff weight in kg */
  mtow: number;

  /** Drone dry weight in kg */
  weight: number;

  /** Maximum payload capacity in kg */
  max_payload: number;

  /** Propulsion type */
  propulsion: PropulsionType;

  /** Number of rotors/motors */
  motor_count: number;

  /** Power system */
  power_system: PowerSystem;

  /** Battery capacity in mAh */
  battery_capacity: number;

  /** Battery voltage in V */
  battery_voltage: number;

  /** Maximum range in meters */
  max_range: number;

  /** Maximum flight time in minutes */
  max_flight_time: number;

  /** Maximum speed in m/s */
  max_speed: number;

  /** Cruising speed in m/s */
  cruise_speed: number;

  /** Maximum altitude in meters AGL */
  max_altitude: number;

  /** Hover accuracy in meters */
  hover_accuracy: number;

  /** Wind resistance in m/s */
  wind_resistance: number;

  /** Operating temperature range */
  temperature_range: {
    min: number;
    max: number;
  };

  /** Sensors equipped */
  sensors: string[];

  /** Certifications */
  certifications: string[];
}

// ============================================================================
// Package and Payload
// ============================================================================

/**
 * Delivery package information
 */
export interface DeliveryPackage {
  /** Package identifier */
  package_id: string;

  /** Weight in kilograms */
  weight: number;

  /** Dimensions */
  dimensions: Dimensions;

  /** Is package fragile? */
  fragile: boolean;

  /** Is temperature sensitive? */
  temperature_sensitive: boolean;

  /** Required temperature range (if sensitive) */
  temperature_range?: {
    min: number;
    max: number;
  };

  /** Package value in currency */
  value?: number;

  /** Currency code */
  currency?: string;

  /** Is insured? */
  insurance?: boolean;

  /** Special handling instructions */
  special_instructions?: string;

  /** Tracking code */
  tracking_code?: string;

  /** Contents description */
  contents?: string;
}

/**
 * Payload validation result
 */
export interface PayloadValidation {
  /** Is payload valid for drone? */
  is_valid: boolean;

  /** Validation errors */
  errors: string[];

  /** Validation warnings */
  warnings: string[];

  /** Estimated range reduction percentage */
  range_reduction: number;

  /** Estimated flight time reduction percentage */
  flight_time_reduction: number;

  /** Maximum safe range with payload */
  max_range: number;

  /** Center of gravity offset */
  cg_offset?: {
    x: number;
    y: number;
    z: number;
  };
}

// ============================================================================
// Location and Waypoint
// ============================================================================

/**
 * Delivery location details
 */
export interface DeliveryLocation {
  /** Geographic coordinates */
  location: LatLng;

  /** Street address */
  address: string;

  /** Address line 2 (apt, suite, etc.) */
  address_line2?: string;

  /** City */
  city?: string;

  /** State/Province */
  state?: string;

  /** Postal code */
  postal_code?: string;

  /** Country code */
  country?: string;

  /** Contact information */
  contact?: {
    name: string;
    phone: string;
    email?: string;
  };

  /** Landing zone type */
  landing_zone?: 'yard' | 'rooftop' | 'parking-lot' | 'designated-pad' | 'street';

  /** Access notes */
  access_notes?: string;

  /** Geofence restrictions */
  restrictions?: string[];
}

/**
 * Flight waypoint
 */
export interface Waypoint {
  /** Waypoint identifier */
  waypoint_id: string;

  /** Position */
  position: GeoCoordinate;

  /** Desired speed at waypoint (m/s) */
  speed?: number;

  /** Desired heading at waypoint (degrees) */
  heading?: number;

  /** Actions to perform at waypoint */
  actions?: WaypointAction[];

  /** Acceptance radius in meters */
  acceptance_radius?: number;

  /** Waypoint type */
  type?: 'transit' | 'hover' | 'land' | 'takeoff' | 'photo';
}

/**
 * Action to perform at waypoint
 */
export interface WaypointAction {
  /** Action type */
  type: 'hover' | 'take_photo' | 'take_video' | 'release_payload' | 'wait' | 'scan';

  /** Action parameters */
  params?: Record<string, unknown>;

  /** Duration in seconds (for time-based actions) */
  duration?: number;
}

// ============================================================================
// Flight Planning
// ============================================================================

/**
 * Flight plan
 */
export interface FlightPlan {
  /** Flight plan identifier */
  plan_id: string;

  /** Departure location and time */
  departure: {
    location: GeoCoordinate;
    time: Date | string;
  };

  /** Arrival location and time */
  arrival: {
    location: GeoCoordinate;
    time: Date | string;
  };

  /** Waypoints along route */
  waypoints: Waypoint[];

  /** Maximum altitude for flight */
  max_altitude: number;

  /** Maximum speed for flight */
  max_speed: number;

  /** Total distance in meters */
  total_distance: number;

  /** Estimated flight time in seconds */
  estimated_time: number;

  /** Emergency landing zones */
  emergency_zones?: GeoCoordinate[];

  /** No-fly zones to avoid */
  avoid_zones?: GeoCoordinate[];
}

/**
 * Route planning parameters
 */
export interface RoutePlanningParams {
  /** Origin coordinates */
  origin: LatLng;

  /** Destination coordinates */
  destination: LatLng;

  /** Payload information */
  payload?: DeliveryPackage;

  /** Weather conditions */
  weather?: WeatherConditions;

  /** Avoid no-fly zones? */
  avoid_no_fly_zones?: boolean;

  /** Optimize for */
  optimize?: 'time' | 'energy' | 'safety' | 'distance';

  /** Maximum altitude constraint */
  max_altitude?: number;

  /** Required intermediate waypoints */
  via_points?: LatLng[];
}

/**
 * Weather conditions
 */
export interface WeatherConditions {
  /** Wind speed in m/s */
  wind_speed: number;

  /** Wind direction in degrees (0=North) */
  wind_direction: number;

  /** Temperature in Celsius */
  temperature: number;

  /** Humidity percentage */
  humidity?: number;

  /** Visibility in meters */
  visibility?: number;

  /** Precipitation level */
  precipitation?: 'none' | 'light' | 'moderate' | 'heavy';

  /** Weather condition */
  condition?: 'clear' | 'partly-cloudy' | 'cloudy' | 'rain' | 'snow' | 'fog';
}

// ============================================================================
// Mission Management
// ============================================================================

/**
 * Delivery mission priority
 */
export type MissionPriority = 'express' | 'standard' | 'economy';

/**
 * Mission status
 */
export type MissionStatus =
  | 'pending'
  | 'scheduled'
  | 'preparing'
  | 'ready'
  | 'in_flight'
  | 'picking_up'
  | 'delivering'
  | 'completed'
  | 'cancelled'
  | 'failed';

/**
 * Delivery mission
 */
export interface DeliveryMission {
  /** Mission identifier */
  mission_id: string;

  /** Operator identifier */
  operator_id: string;

  /** Drone identifier */
  drone_id?: string;

  /** Mission status */
  status: MissionStatus;

  /** Pickup location */
  pickup: DeliveryLocation;

  /** Dropoff location */
  dropoff: DeliveryLocation;

  /** Package information */
  package: DeliveryPackage;

  /** Mission priority */
  priority: MissionPriority;

  /** Scheduled pickup time */
  scheduled_pickup?: Date | string;

  /** Scheduled delivery time */
  scheduled_delivery?: Date | string;

  /** Actual pickup time */
  actual_pickup?: Date | string;

  /** Actual delivery time */
  actual_delivery?: Date | string;

  /** Flight plan */
  flight_plan?: FlightPlan;

  /** Current position (if in flight) */
  current_position?: GeoCoordinate;

  /** Progress percentage (0-100) */
  progress?: number;

  /** Estimated time of arrival */
  eta?: Date | string;

  /** Battery remaining percentage */
  battery_remaining?: number;

  /** Tracking URL */
  tracking_url?: string;

  /** Special instructions */
  special_instructions?: string[];

  /** Mission notes */
  notes?: string;
}

/**
 * Mission creation parameters
 */
export interface CreateMissionParams {
  pickup: DeliveryLocation;
  dropoff: DeliveryLocation;
  package: DeliveryPackage;
  priority: MissionPriority;
  scheduled_time?: Date | string;
  special_instructions?: string[];
}

/**
 * Mission result
 */
export interface MissionResult {
  mission_id: string;
  status: MissionStatus;
  estimated_pickup: Date | string;
  estimated_delivery: Date | string;
  drone_assigned: string;
  tracking_url: string;
  estimated_cost?: number;
}

// ============================================================================
// Flight Control
// ============================================================================

/**
 * Flight mode
 */
export type FlightMode =
  | 'manual'
  | 'assisted'
  | 'altitude-hold'
  | 'position-hold'
  | 'waypoint'
  | 'return-to-home'
  | 'land';

/**
 * Flight controller state
 */
export interface FlightControllerState {
  /** Current flight mode */
  mode: FlightMode;

  /** Is armed? */
  armed: boolean;

  /** Current position */
  position: GeoCoordinate;

  /** Velocity vector */
  velocity: {
    vx: number;
    vy: number;
    vz: number;
  };

  /** Attitude (roll, pitch, yaw in degrees) */
  attitude: {
    roll: number;
    pitch: number;
    yaw: number;
  };

  /** Angular rates (deg/s) */
  rates?: {
    roll_rate: number;
    pitch_rate: number;
    yaw_rate: number;
  };

  /** Battery state of charge (0-100) */
  battery_soc: number;

  /** GPS satellite count */
  gps_satellites: number;

  /** GPS fix type */
  gps_fix: 'none' | '2d' | '3d' | 'dgps' | 'rtk';

  /** Home position */
  home_position?: GeoCoordinate;

  /** Timestamp */
  timestamp: Date | string;
}

/**
 * Control command
 */
export interface ControlCommand {
  /** Command type */
  type: 'takeoff' | 'land' | 'goto' | 'hover' | 'rtl' | 'emergency-stop';

  /** Target position (for goto) */
  target?: GeoCoordinate;

  /** Target altitude (for takeoff) */
  altitude?: number;

  /** Target speed */
  speed?: number;

  /** Command timestamp */
  timestamp: Date | string;
}

// ============================================================================
// Battery Management
// ============================================================================

/**
 * Battery information
 */
export interface BatteryInfo {
  /** Battery identifier */
  battery_id: string;

  /** Voltage in volts */
  voltage: number;

  /** Current in amperes */
  current: number;

  /** Capacity in mAh */
  capacity: number;

  /** Remaining capacity in mAh */
  remaining_capacity: number;

  /** State of charge (0-100) */
  soc: number;

  /** Temperature in Celsius */
  temperature: number;

  /** Cycle count */
  cycles: number;

  /** Health percentage (0-100) */
  health: number;

  /** Time to empty in minutes */
  time_to_empty?: number;

  /** Is charging? */
  charging: boolean;

  /** Battery status */
  status: 'normal' | 'low' | 'critical' | 'charging' | 'error';
}

/**
 * Battery estimation parameters
 */
export interface BatteryEstimationParams {
  /** Battery capacity in mAh */
  capacity: number;

  /** Current state of charge (0-100) */
  current_soc: number;

  /** Payload weight in kg */
  payload_weight: number;

  /** Distance to travel in meters */
  distance: number;

  /** Average speed in m/s */
  speed?: number;

  /** Weather conditions */
  weather?: WeatherConditions;
}

/**
 * Battery estimation result
 */
export interface BatteryEstimation {
  /** Can complete mission? */
  sufficient: boolean;

  /** Estimated SOC at destination */
  estimated_soc_at_destination: number;

  /** Estimated SOC at return home */
  estimated_soc_at_return: number;

  /** Maximum range with current battery */
  max_range: number;

  /** Recommended action */
  recommendation: 'proceed' | 'charge-first' | 'reduce-payload' | 'impossible';

  /** Energy required in Wh */
  energy_required: number;

  /** Energy available in Wh */
  energy_available: number;

  /** Safety margin percentage */
  safety_margin: number;
}

// ============================================================================
// UTM Integration
// ============================================================================

/**
 * UTM operation
 */
export interface UTMOperation {
  /** Operation identifier */
  operation_id: string;

  /** Operator identifier */
  operator_id: string;

  /** Drone identifier */
  drone_id: string;

  /** Flight plan */
  flight_plan: FlightPlan;

  /** Operation state */
  state: 'planned' | 'accepted' | 'activated' | 'nonconforming' | 'contingent' | 'ended';

  /** UTM approval status */
  approval_status: 'pending' | 'approved' | 'denied' | 'expired';

  /** Approval timestamp */
  approval_time?: Date | string;

  /** Expiration time */
  expiration_time?: Date | string;
}

/**
 * Position report for UTM
 */
export interface UTMPositionReport {
  /** Drone identifier */
  drone_id: string;

  /** Report timestamp */
  timestamp: Date | string;

  /** Position */
  position: GeoCoordinate;

  /** Velocity */
  velocity: {
    vx: number;
    vy: number;
    vz: number;
  };

  /** Heading in degrees */
  heading: number;

  /** Battery percentage */
  battery: number;

  /** Operation status */
  status: 'nominal' | 'off-nominal' | 'contingent' | 'emergency';
}

/**
 * Conflict alert from UTM
 */
export interface ConflictAlert {
  /** Alert identifier */
  alert_id: string;

  /** Conflicting operation ID */
  conflicting_operation: string;

  /** Time to conflict in seconds */
  time_to_conflict: number;

  /** Minimum separation at conflict */
  min_separation: {
    horizontal: number;
    vertical: number;
  };

  /** Severity */
  severity: 'info' | 'warning' | 'critical';

  /** Recommended action */
  recommended_action?: 'alter-course' | 'change-altitude' | 'return-home' | 'land';
}

// ============================================================================
// Safety and Emergency
// ============================================================================

/**
 * Emergency type
 */
export type EmergencyType =
  | 'motor-failure'
  | 'battery-critical'
  | 'gps-loss'
  | 'communication-loss'
  | 'collision-detected'
  | 'geofence-breach'
  | 'weather-deterioration'
  | 'manual-trigger';

/**
 * Emergency event
 */
export interface EmergencyEvent {
  /** Event identifier */
  event_id: string;

  /** Emergency type */
  type: EmergencyType;

  /** Event timestamp */
  timestamp: Date | string;

  /** Position when emergency occurred */
  position: GeoCoordinate;

  /** Severity */
  severity: 'low' | 'medium' | 'high' | 'critical';

  /** Description */
  description: string;

  /** Actions taken */
  actions_taken: string[];

  /** Current status */
  status: 'active' | 'resolved' | 'escalated';

  /** Resolution */
  resolution?: string;
}

/**
 * Geofence zone
 */
export interface GeofenceZone {
  /** Zone identifier */
  zone_id: string;

  /** Zone name */
  name: string;

  /** Center point */
  center: LatLng;

  /** Radius in meters */
  radius: number;

  /** Altitude range */
  altitude_range?: {
    min: number;
    max: number;
  };

  /** Priority level */
  priority: 'critical' | 'high' | 'medium' | 'low';

  /** Zone type */
  type: 'no-fly' | 'restricted' | 'warning' | 'safe-landing';

  /** Is active? */
  active: boolean;

  /** Start time (for temporal restrictions) */
  start_time?: Date | string;

  /** End time (for temporal restrictions) */
  end_time?: Date | string;
}

/**
 * Safety check result
 */
export interface SafetyCheck {
  /** Check name */
  name: string;

  /** Check status */
  status: 'pass' | 'fail' | 'warning' | 'info';

  /** Check description */
  description: string;

  /** Measured value */
  value?: number | string;

  /** Expected value */
  expected?: number | string;

  /** Is blocking? */
  blocking: boolean;

  /** Corrective action */
  corrective_action?: string;
}

/**
 * Pre-flight check results
 */
export interface PreFlightCheckResults {
  /** Overall status */
  overall_status: 'pass' | 'fail' | 'warning';

  /** Individual checks */
  checks: SafetyCheck[];

  /** Can proceed with flight? */
  flight_approved: boolean;

  /** Timestamp of checks */
  timestamp: Date | string;
}

// ============================================================================
// Telemetry and Logging
// ============================================================================

/**
 * Telemetry data point
 */
export interface TelemetryData {
  /** Timestamp */
  timestamp: Date | string;

  /** Position */
  position: GeoCoordinate;

  /** Velocity */
  velocity: {
    vx: number;
    vy: number;
    vz: number;
  };

  /** Attitude */
  attitude: {
    roll: number;
    pitch: number;
    yaw: number;
  };

  /** Battery state */
  battery: {
    voltage: number;
    current: number;
    soc: number;
  };

  /** GPS info */
  gps: {
    satellites: number;
    fix_type: string;
    hdop: number;
  };

  /** Motor states */
  motors?: number[];

  /** Sensor data */
  sensors?: Record<string, unknown>;
}

/**
 * Flight log
 */
export interface FlightLog {
  /** Log identifier */
  log_id: string;

  /** Flight identifier */
  flight_id: string;

  /** Drone identifier */
  drone_id: string;

  /** Operator identifier */
  operator_id: string;

  /** Start time */
  start_time: Date | string;

  /** End time */
  end_time: Date | string;

  /** Duration in seconds */
  duration: number;

  /** Total distance in meters */
  distance: number;

  /** Maximum altitude reached */
  max_altitude: number;

  /** Maximum speed reached */
  max_speed: number;

  /** Battery consumed percentage */
  battery_consumed: number;

  /** Waypoints completed */
  waypoints_completed: number;

  /** Total waypoints */
  total_waypoints: number;

  /** Incidents during flight */
  incidents: EmergencyEvent[];

  /** Telemetry file path */
  telemetry_file?: string;

  /** Video file path */
  video_file?: string;
}

// ============================================================================
// Physical Constants and Calculations
// ============================================================================

/**
 * Physical constants for drone calculations
 */
export const DRONE_CONSTANTS = {
  /** Gravitational acceleration (m/s²) */
  GRAVITY: 9.81,

  /** Air density at sea level (kg/m³) */
  AIR_DENSITY_SEA_LEVEL: 1.225,

  /** Speed of sound at sea level (m/s) */
  SPEED_OF_SOUND: 343,

  /** Standard atmospheric pressure (Pa) */
  ATMOSPHERIC_PRESSURE: 101325,

  /** Typical rotor thrust coefficient */
  THRUST_COEFFICIENT: 0.012,

  /** Typical drag coefficient for multi-rotor */
  DRAG_COEFFICIENT: 1.2,

  /** Battery reserve margin (20-30%) */
  BATTERY_RESERVE: 0.25,

  /** Typical motor efficiency */
  MOTOR_EFFICIENCY: 0.8,

  /** Minimum GPS satellites for flight */
  MIN_GPS_SATELLITES: 8,

  /** Minimum battery SOC for takeoff */
  MIN_TAKEOFF_SOC: 30,

  /** Critical battery SOC */
  CRITICAL_BATTERY_SOC: 10,
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
 * WIA-AUTO-017 error codes
 */
export enum DroneErrorCode {
  PAYLOAD_TOO_HEAVY = 'D001',
  BATTERY_INSUFFICIENT = 'D002',
  WEATHER_UNSAFE = 'D003',
  GEOFENCE_VIOLATION = 'D004',
  GPS_UNAVAILABLE = 'D005',
  MOTOR_FAILURE = 'D006',
  COMMUNICATION_LOST = 'D007',
  INVALID_WAYPOINT = 'D008',
  UTM_DENIED = 'D009',
  EMERGENCY = 'D010',
}

/**
 * Drone error
 */
export class DroneError extends Error {
  constructor(
    public code: DroneErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'DroneError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Geographic
  GeoCoordinate,
  LatLng,
  Position3D,
  Dimensions,

  // Drone
  DroneSpecification,
  DroneClass,
  PropulsionType,
  PowerSystem,

  // Package
  DeliveryPackage,
  PayloadValidation,

  // Location
  DeliveryLocation,
  Waypoint,
  WaypointAction,

  // Planning
  FlightPlan,
  RoutePlanningParams,
  WeatherConditions,

  // Mission
  DeliveryMission,
  CreateMissionParams,
  MissionResult,
  MissionPriority,
  MissionStatus,

  // Control
  FlightMode,
  FlightControllerState,
  ControlCommand,

  // Battery
  BatteryInfo,
  BatteryEstimationParams,
  BatteryEstimation,

  // UTM
  UTMOperation,
  UTMPositionReport,
  ConflictAlert,

  // Safety
  EmergencyType,
  EmergencyEvent,
  GeofenceZone,
  SafetyCheck,
  PreFlightCheckResults,

  // Telemetry
  TelemetryData,
  FlightLog,
};

export { DRONE_CONSTANTS, DroneErrorCode, DroneError };
