/**
 * WIA-COMM-003: V2X Communication - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Communication Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Geographic position with WGS84 coordinates
 */
export interface Position {
  /** Latitude in degrees */
  latitude: number;

  /** Longitude in degrees */
  longitude: number;

  /** Altitude in meters above sea level */
  altitude?: number;

  /** Position accuracy in meters */
  accuracy?: number;
}

/**
 * 3D acceleration vector
 */
export interface Acceleration {
  /** Longitudinal acceleration in m/s² (forward/backward) */
  longitudinal: number;

  /** Lateral acceleration in m/s² (left/right) */
  lateral: number;

  /** Vertical acceleration in m/s² (up/down) */
  vertical: number;

  /** Yaw rate in degrees/second */
  yawRate?: number;
}

/**
 * Vehicle dimensions
 */
export interface VehicleDimensions {
  /** Width in meters */
  width: number;

  /** Length in meters */
  length: number;

  /** Height in meters */
  height?: number;
}

/**
 * Brake status information
 */
export interface BrakeStatus {
  /** Wheel brakes engaged (bit mask) */
  wheelBrakes: string;

  /** Traction control status */
  tractionControl: 'on' | 'off' | 'engaged' | 'unavailable';

  /** Anti-lock braking system status */
  abs: 'on' | 'off' | 'engaged' | 'unavailable';

  /** Electronic stability control status */
  stabilityControl: 'on' | 'off' | 'engaged' | 'unavailable';

  /** Brake boost status */
  brakeBoost: 'on' | 'off' | 'unavailable';

  /** Auxiliary brakes (engine brake, exhaust brake) */
  auxBrakes: 'on' | 'off' | 'unavailable';
}

// ============================================================================
// V2X Technology
// ============================================================================

/**
 * V2X communication technology
 */
export type V2XTechnology = 'DSRC' | 'C-V2X-LTE' | 'C-V2X-5G' | 'HYBRID';

/**
 * Communication mode
 */
export type CommunicationMode = 'broadcast' | 'unicast' | 'multicast' | 'geocast';

/**
 * V2X message types
 */
export enum MessageType {
  /** Basic Safety Message (SAE J2735) */
  BSM = 'BSM',

  /** Cooperative Awareness Message (ETSI) */
  CAM = 'CAM',

  /** Decentralized Environmental Notification Message */
  DENM = 'DENM',

  /** Signal Phase and Timing */
  SPAT = 'SPaT',

  /** Map Data */
  MAP = 'MAP',

  /** Personal Safety Message (Pedestrian) */
  PSM = 'PSM',

  /** Signal Request Message */
  SRM = 'SRM',

  /** Signal Status Message */
  SSM = 'SSM',

  /** Traveler Information Message */
  TIM = 'TIM',

  /** Road Side Alert */
  RSA = 'RSA'
}

/**
 * Security level for V2X communication
 */
export type SecurityLevel = 'none' | 'basic' | 'standard' | 'high' | 'maximum';

/**
 * Privacy level
 */
export type PrivacyLevel = 'none' | 'low' | 'medium' | 'high' | 'maximum';

// ============================================================================
// Basic Safety Message (BSM)
// ============================================================================

/**
 * Basic Safety Message - Part I (Core Data)
 */
export interface BSMPartI {
  /** Message type */
  messageType: MessageType.BSM;

  /** Message count (0-127, wraps around) */
  msgCount: number;

  /** Temporary vehicle ID */
  id: string;

  /** Timestamp in milliseconds within the minute */
  timestamp: number;

  /** Vehicle position */
  position: Position & {
    /** Position accuracy details */
    accuracy: {
      semiMajor: number;
      semiMinor: number;
      orientation: number;
    };
  };

  /** Speed in m/s */
  speed: number;

  /** Heading in degrees (0-359.9875) */
  heading: number;

  /** Acceleration data */
  acceleration: Acceleration;

  /** Steering wheel angle in degrees */
  steeringAngle?: number;

  /** Brake system status */
  brakeStatus: BrakeStatus;

  /** Vehicle size */
  vehicleSize: VehicleDimensions;
}

/**
 * Basic Safety Message - Part II (Optional Data)
 */
export interface BSMPartII {
  /** Path history (previous positions) */
  pathHistory?: Position[];

  /** Path prediction (intended trajectory) */
  pathPrediction?: Position[];

  /** Vehicle classification */
  vehicleClass?: VehicleClass;

  /** Exterior lights */
  lights?: {
    lowBeams: boolean;
    highBeams: boolean;
    leftTurn: boolean;
    rightTurn: boolean;
    hazard: boolean;
    fog: boolean;
  };

  /** Weather information */
  weather?: WeatherInfo;

  /** Road surface conditions */
  roadSurface?: RoadSurfaceCondition;
}

/**
 * Complete BSM message
 */
export interface BSM extends BSMPartI {
  partII?: BSMPartII;
}

// ============================================================================
// Cooperative Awareness Message (CAM)
// ============================================================================

/**
 * Cooperative Awareness Message (ETSI)
 */
export interface CAM {
  /** Message type */
  messageType: MessageType.CAM;

  /** Protocol version */
  protocolVersion: number;

  /** Station ID */
  stationID: number;

  /** Generation timestamp */
  generationTime: number;

  /** Basic container */
  basicContainer: {
    stationType: StationType;
    referencePosition: Position;
  };

  /** High frequency container */
  highFrequencyContainer: {
    heading: number;
    speed: number;
    driveDirection: 'forward' | 'backward';
    vehicleLength: number;
    vehicleWidth: number;
    longitudinalAcceleration: number;
    curvature?: number;
    yawRate?: number;
  };

  /** Low frequency container (optional) */
  lowFrequencyContainer?: {
    vehicleRole: VehicleRole;
    exteriorLights?: number;
    pathHistory?: Position[];
  };
}

// ============================================================================
// Decentralized Environmental Notification Message (DENM)
// ============================================================================

/**
 * Decentralized Environmental Notification Message
 */
export interface DENM {
  /** Message type */
  messageType: MessageType.DENM;

  /** Protocol version */
  protocolVersion: number;

  /** Station ID */
  stationID: number;

  /** Management container */
  management: {
    actionID: {
      originatingStationID: number;
      sequenceNumber: number;
    };
    detectionTime: number;
    referenceTime: number;
    eventPosition: Position;
    relevanceDistance: RelevanceDistance;
    relevanceTrafficDirection: RelevanceTrafficDirection;
    validityDuration?: number;
    transmissionInterval?: number;
  };

  /** Situation container */
  situation?: {
    eventType: {
      causeCode: CauseCode;
      subCauseCode?: number;
    };
    severity?: Severity;
    linkedCause?: {
      causeCode: CauseCode;
      subCauseCode?: number;
    };
  };

  /** Location container */
  location?: {
    eventSpeed?: number;
    eventPositionHeading?: number;
    traces?: Position[][];
    roadType?: RoadType;
  };

  /** Alacarte container */
  alacarte?: {
    lanePosition?: number;
    temperature?: number;
    positioningSolution?: PositioningSolution;
  };
}

// ============================================================================
// Signal Phase and Timing (SPaT)
// ============================================================================

/**
 * Signal Phase and Timing Message
 */
export interface SPaTMessage {
  /** Message type */
  messageType: MessageType.SPAT;

  /** Timestamp */
  timestamp: number;

  /** Intersection ID */
  intersectionID: number;

  /** Intersection status */
  status: IntersectionStatus;

  /** Signal states */
  states: SignalState[];
}

/**
 * Signal state for a movement/phase
 */
export interface SignalState {
  /** Movement name/description */
  movementName?: string;

  /** Signal group ID */
  signalGroup: number;

  /** Current state */
  state: SignalPhase;

  /** Timing information */
  timing: {
    minEndTime: number;
    maxEndTime?: number;
    likelyTime?: number;
    confidence?: number;
    nextTime?: number;
  };

  /** Maneuver assist data */
  maneuverAssist?: {
    connectionID: number;
    queueLength?: number;
    availableStorageLength?: number;
    waitOnStop?: boolean;
    pedBicycleDetect?: boolean;
  };
}

// ============================================================================
// MAP Message
// ============================================================================

/**
 * MAP Data Message
 */
export interface MAPMessage {
  /** Message type */
  messageType: MessageType.MAP;

  /** Timestamp */
  timestamp: number;

  /** Intersection ID */
  intersectionID: number;

  /** Reference point (intersection center) */
  referencePoint: Position;

  /** Lane set */
  laneSet: Lane[];

  /** Speed limits */
  speedLimits?: SpeedLimit[];
}

/**
 * Lane definition
 */
export interface Lane {
  /** Lane ID */
  laneID: number;

  /** Lane attributes */
  laneAttributes: {
    directionalUse: 'ingressPath' | 'egressPath';
    sharedWith: 'none' | 'pedestrians' | 'cyclists' | 'buses';
    laneType: 'vehicle' | 'crosswalk' | 'bikeLane' | 'sidewalk';
  };

  /** Allowed maneuvers */
  maneuvers: Maneuver[];

  /** Node list (lane geometry) */
  nodeList: {
    nodes: Array<{
      delta: {
        dx: number;
        dy: number;
        dz?: number;
      };
    }>;
  };

  /** Connections to other lanes */
  connectsTo?: Array<{
    connectingLane: number;
    signalGroup?: number;
    connectionID?: number;
  }>;

  /** Speed limits for this lane */
  speedLimits?: SpeedLimit[];
}

// ============================================================================
// Personal Safety Message (PSM)
// ============================================================================

/**
 * Personal Safety Message (Pedestrian)
 */
export interface PSM {
  /** Message type */
  messageType: MessageType.PSM;

  /** Device ID */
  deviceID: string;

  /** Device type */
  deviceType: 'smartphone' | 'wearable' | 'dedicated';

  /** Position */
  position: Position;

  /** Speed in m/s */
  speed: number;

  /** Heading in degrees */
  heading: number;

  /** User type */
  userType: 'pedestrian' | 'cyclist' | 'wheelchair' | 'stroller';

  /** Cluster information */
  cluster?: {
    clusterSize: number;
    clusterRadius: number;
  };

  /** Timestamp */
  timestamp: number;

  /** Crossing state */
  crossing?: {
    status: 'waiting' | 'crossing' | 'finished';
    intention: 'cross' | 'not-cross' | 'unknown';
  };
}

// ============================================================================
// V2X Communication Configuration
// ============================================================================

/**
 * V2X communication configuration
 */
export interface V2XConfig {
  /** Vehicle ID */
  vehicleId: string;

  /** Communication technology */
  technology: V2XTechnology;

  /** Operating frequency in MHz */
  frequency?: number;

  /** Transmit power in dBm */
  transmitPower?: number;

  /** Security level */
  securityLevel?: SecurityLevel;

  /** Privacy level */
  privacyLevel?: PrivacyLevel;

  /** Message rates (Hz) */
  messageRates?: {
    bsm?: number;
    cam?: number;
    denm?: number;
  };

  /** Maximum communication range in meters */
  maxRange?: number;
}

// ============================================================================
// V2V Messages
// ============================================================================

/**
 * V2V message (generic)
 */
export interface V2VMessage {
  /** Message type */
  type: MessageType;

  /** Sender vehicle ID */
  senderId: string;

  /** Position */
  position: Position;

  /** Speed in km/h */
  speed: number;

  /** Heading in degrees */
  heading: number;

  /** Acceleration */
  acceleration: Acceleration;

  /** Timestamp */
  timestamp: number;

  /** Message-specific data */
  data?: any;
}

/**
 * V2V collision warning
 */
export interface CollisionWarning {
  /** Warning level */
  level: 'LOW' | 'MEDIUM' | 'HIGH' | 'CRITICAL';

  /** Target vehicle ID */
  targetVehicleId: string;

  /** Time to collision in seconds */
  ttc: number;

  /** Distance to collision point in meters */
  distance: number;

  /** Recommended action */
  action: 'MONITOR' | 'WARN' | 'BRAKE' | 'EMERGENCY_BRAKE';

  /** Required deceleration in m/s² */
  requiredDeceleration?: number;
}

// ============================================================================
// V2I Messages
// ============================================================================

/**
 * V2I message (generic)
 */
export interface V2IMessage {
  /** Message type */
  type: MessageType;

  /** Infrastructure ID (RSU ID) */
  infrastructureId: string;

  /** Position of infrastructure */
  position: Position;

  /** Timestamp */
  timestamp: number;

  /** Message-specific data */
  data?: any;
}

/**
 * RSU (Road Side Unit) information
 */
export interface RSUInfo {
  /** RSU ID */
  id: string;

  /** RSU location */
  position: Position;

  /** RSU type */
  type: 'intersection' | 'highway' | 'parking' | 'toll' | 'general';

  /** Coverage area radius in meters */
  coverageRadius: number;

  /** Supported message types */
  supportedMessages: MessageType[];

  /** Status */
  status: 'active' | 'inactive' | 'maintenance';
}

// ============================================================================
// V2P Messages
// ============================================================================

/**
 * Pedestrian detection result
 */
export interface PedestrianDetection {
  /** Pedestrian ID */
  pedestrianId: string;

  /** Position */
  position: Position;

  /** Speed in m/s */
  speed: number;

  /** Heading in degrees */
  heading: number;

  /** Distance from vehicle in meters */
  distance: number;

  /** Risk level */
  risk: 'NONE' | 'LOW' | 'MEDIUM' | 'HIGH' | 'CRITICAL';

  /** Time to collision (if applicable) */
  ttc?: number;

  /** Recommended action */
  action?: 'MONITOR' | 'WARN_AND_SLOW' | 'EMERGENCY_BRAKE';
}

// ============================================================================
// V2N Messages
// ============================================================================

/**
 * V2N telemetry data
 */
export interface V2NTelemetry {
  /** Vehicle ID */
  vehicleId: string;

  /** Timestamp */
  timestamp: number;

  /** Position */
  position: Position;

  /** Speed in km/h */
  speed: number;

  /** Heading in degrees */
  heading: number;

  /** Road conditions */
  roadConditions?: {
    friction: number;
    wetness: 'dry' | 'damp' | 'wet' | 'flooded';
    visibility: 'good' | 'moderate' | 'poor' | 'zero';
  };

  /** Traffic conditions */
  trafficConditions?: {
    density: 'free' | 'light' | 'moderate' | 'heavy' | 'congested';
    averageSpeed: number;
    incidents: string[];
  };

  /** Vehicle health */
  vehicleHealth?: {
    battery?: number;
    fuel?: number;
    engineHealth?: 'good' | 'warning' | 'critical';
    diagnosticCodes?: string[];
  };
}

/**
 * Traffic update from cloud
 */
export interface TrafficUpdate {
  /** Update timestamp */
  timestamp: number;

  /** Geographic region */
  region: {
    bounds: {
      north: number;
      south: number;
      east: number;
      west: number;
    };
  };

  /** Incidents */
  incidents: TrafficIncident[];

  /** Congestion information */
  congestion: CongestionInfo[];

  /** Road closures */
  closures?: RoadClosure[];
}

/**
 * Traffic incident
 */
export interface TrafficIncident {
  /** Incident ID */
  id: string;

  /** Incident type */
  type: 'accident' | 'breakdown' | 'hazard' | 'weather' | 'construction';

  /** Location */
  location: Position;

  /** Severity */
  severity: 'minor' | 'moderate' | 'major' | 'severe';

  /** Lanes affected */
  lanesAffected?: number[];

  /** Estimated clearance time */
  estimatedClearance?: string;

  /** Description */
  description?: string;
}

// ============================================================================
// Platooning
// ============================================================================

/**
 * Platoon information
 */
export interface Platoon {
  /** Platoon ID */
  platoonId: string;

  /** Leader vehicle ID */
  leader: string;

  /** Member vehicle IDs (in order) */
  members: string[];

  /** Formation type */
  formation: 'line' | 'column';

  /** Target speed in km/h */
  targetSpeed: number;

  /** Inter-vehicle spacing in meters */
  spacing: number;

  /** Maximum members allowed */
  maxMembers: number;

  /** Current status */
  status: 'FORMING' | 'ACTIVE' | 'DISSOLVING';
}

/**
 * Platoon join request
 */
export interface PlatoonJoinRequest {
  /** Requesting vehicle ID */
  vehicleId: string;

  /** Target platoon ID */
  platoonId: string;

  /** Vehicle capabilities */
  capabilities: {
    cacc: boolean;
    v2v: boolean;
    maxSpeed: number;
    vehicleType: VehicleClass;
  };
}

/**
 * Platoon join response
 */
export interface PlatoonJoinResponse {
  /** Status */
  status: 'APPROVED' | 'REJECTED' | 'PENDING';

  /** Reason (if rejected) */
  reason?: string;

  /** Assigned position in platoon */
  position?: number;

  /** Target spacing */
  targetSpacing?: number;

  /** Target speed */
  targetSpeed?: number;

  /** Formation point */
  formationPoint?: Position;
}

/**
 * Platoon maneuver command
 */
export interface PlatoonManeuver {
  /** Platoon ID */
  platoonId: string;

  /** Maneuver type */
  type: 'LANE_CHANGE' | 'SPEED_CHANGE' | 'SPLIT' | 'MERGE' | 'DISSOLVE';

  /** Execution time */
  executionTime: number;

  /** Maneuver parameters */
  parameters: {
    targetLane?: number;
    targetSpeed?: number;
    splitPosition?: number;
    mergePlatoon?: string;
  };
}

// ============================================================================
// Enumerations and Constants
// ============================================================================

/**
 * Station type (ETSI)
 */
export enum StationType {
  UNKNOWN = 0,
  PEDESTRIAN = 1,
  CYCLIST = 2,
  MOPED = 3,
  MOTORCYCLE = 4,
  PASSENGER_CAR = 5,
  BUS = 6,
  LIGHT_TRUCK = 7,
  HEAVY_TRUCK = 8,
  TRAILER = 9,
  SPECIAL_VEHICLE = 10,
  TRAM = 11,
  RSU = 15
}

/**
 * Vehicle class
 */
export enum VehicleClass {
  MOTORCYCLE = 'motorcycle',
  PASSENGER_CAR = 'passengerCar',
  LIGHT_TRUCK = 'lightTruck',
  HEAVY_TRUCK = 'heavyTruck',
  BUS = 'bus',
  TRAILER = 'trailer',
  EMERGENCY = 'emergency',
  CONSTRUCTION = 'construction'
}

/**
 * Vehicle role
 */
export enum VehicleRole {
  DEFAULT = 'default',
  PUBLIC_TRANSPORT = 'publicTransport',
  SPECIAL_TRANSPORT = 'specialTransport',
  DANGEROUS_GOODS = 'dangerousGoods',
  ROAD_WORK = 'roadWork',
  RESCUE = 'rescue',
  EMERGENCY = 'emergency',
  SAFETY_CAR = 'safetyCar'
}

/**
 * Cause code (ETSI)
 */
export enum CauseCode {
  RESERVED = 0,
  TRAFFIC_CONDITION = 1,
  ACCIDENT = 2,
  ROADWORKS = 3,
  ADVERSE_WEATHER = 6,
  IMPASSABILITY = 7,
  HAZARDOUS_LOCATION = 9,
  ANIMAL_ON_ROAD = 10,
  HUMAN_PRESENCE = 11,
  WRONG_WAY_DRIVING = 14,
  RESCUE_AND_RECOVERY = 15,
  EMERGENCY_VEHICLE = 95,
  COLLISION_RISK = 97,
  SIGNAL_VIOLATION = 98,
  DANGEROUS_SITUATION = 99
}

/**
 * Severity level
 */
export enum Severity {
  UNAVAILABLE = 'unavailable',
  LOWEST = 'lowest',
  LOW = 'low',
  MEDIUM = 'medium',
  HIGH = 'high',
  HIGHEST = 'highest',
  DANGER = 'danger'
}

/**
 * Signal phase
 */
export enum SignalPhase {
  UNAVAILABLE = 'unavailable',
  DARK = 'dark',
  STOP_THEN_PROCEED = 'stop-Then-Proceed',
  STOP_AND_REMAIN = 'stop-And-Remain',
  PRE_MOVEMENT = 'pre-Movement',
  PERMISSIVE_MOVEMENT = 'permissive-Movement-Allowed',
  PROTECTED_MOVEMENT = 'protected-Movement-Allowed',
  PERMISSIVE_CLEARANCE = 'permissive-clearance',
  PROTECTED_CLEARANCE = 'protected-clearance',
  CAUTION_CONFLICTING = 'caution-Conflicting-Traffic'
}

/**
 * Intersection status
 */
export type IntersectionStatus = 'signalOn' | 'signalOff' | 'failure' | 'preempt';

/**
 * Maneuver type
 */
export type Maneuver =
  | 'straight'
  | 'left'
  | 'right'
  | 'uTurn'
  | 'leftTurnOnRed'
  | 'rightTurnOnRed';

/**
 * Relevance distance
 */
export type RelevanceDistance =
  | 'lessThan50m'
  | 'lessThan100m'
  | 'lessThan200m'
  | 'lessThan500m'
  | 'lessThan1000m'
  | 'lessThan5km'
  | 'lessThan10km'
  | 'over10km';

/**
 * Relevance traffic direction
 */
export type RelevanceTrafficDirection =
  | 'allTrafficDirections'
  | 'upstreamTraffic'
  | 'downstreamTraffic'
  | 'oppositeTraffic';

/**
 * Road type
 */
export type RoadType =
  | 'urban'
  | 'suburban'
  | 'highway'
  | 'rural'
  | 'motorway';

/**
 * Positioning solution
 */
export type PositioningSolution =
  | 'noPositioningSolution'
  | 'sGNSS'
  | 'dGNSS'
  | 'sGNSSplusDR'
  | 'dGNSSplusDR'
  | 'dR';

/**
 * Weather information
 */
export interface WeatherInfo {
  temperature?: number; // Celsius
  precipitation?: 'none' | 'light' | 'moderate' | 'heavy';
  visibility?: number; // meters
  windSpeed?: number; // m/s
  windDirection?: number; // degrees
}

/**
 * Road surface condition
 */
export type RoadSurfaceCondition =
  | 'dry'
  | 'wet'
  | 'icy'
  | 'snowy'
  | 'slippery';

/**
 * Speed limit
 */
export interface SpeedLimit {
  /** Speed limit type */
  type: 'vehicleMaxSpeed' | 'vehicleMinSpeed' | 'vehicleNightMaxSpeed';

  /** Speed in m/s */
  speed: number;
}

/**
 * Congestion information
 */
export interface CongestionInfo {
  /** Road ID */
  roadID: string;

  /** Road segment */
  segment: string;

  /** Average speed in km/h */
  speed: number;

  /** Expected delay in minutes */
  delay: number;
}

/**
 * Road closure
 */
export interface RoadClosure {
  /** Closure ID */
  id: string;

  /** Road ID */
  roadID: string;

  /** Start position */
  startPosition: Position;

  /** End position */
  endPosition: Position;

  /** Start time */
  startTime: string;

  /** End time */
  endTime?: string;

  /** Reason */
  reason: string;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * V2X error codes
 */
export enum V2XErrorCode {
  INITIALIZATION_FAILED = 'V2X001',
  RADIO_ERROR = 'V2X002',
  GNSS_ERROR = 'V2X003',
  CERTIFICATE_ERROR = 'V2X004',
  MESSAGE_PARSE_ERROR = 'V2X005',
  SECURITY_VIOLATION = 'V2X006',
  COMMUNICATION_TIMEOUT = 'V2X007',
  INVALID_MESSAGE = 'V2X008',
  MISBEHAVIOR_DETECTED = 'V2X009',
  PLATOON_ERROR = 'V2X010'
}

/**
 * V2X error
 */
export class V2XError extends Error {
  constructor(
    public code: V2XErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'V2XError';
  }
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * V2X communication constants
 */
export const V2X_CONSTANTS = {
  /** DSRC frequency in MHz */
  DSRC_FREQUENCY: 5900,

  /** Maximum BSM transmission rate in Hz */
  MAX_BSM_RATE: 10,

  /** Minimum BSM transmission rate in Hz */
  MIN_BSM_RATE: 1,

  /** Maximum communication range in meters (DSRC) */
  MAX_RANGE_DSRC: 1000,

  /** Maximum communication range in meters (C-V2X) */
  MAX_RANGE_CV2X: 1500,

  /** Target latency for safety messages in milliseconds */
  TARGET_SAFETY_LATENCY: 10,

  /** Maximum acceptable latency for safety messages in milliseconds */
  MAX_SAFETY_LATENCY: 50,

  /** Minimum packet delivery ratio for safety messages */
  MIN_PDR_SAFETY: 0.999,

  /** Platoon minimum spacing in meters */
  PLATOON_MIN_SPACING: 5,

  /** Platoon maximum spacing in meters */
  PLATOON_MAX_SPACING: 15,

  /** Collision warning TTC threshold in seconds */
  COLLISION_WARNING_TTC: 2.5,

  /** Critical collision TTC threshold in seconds */
  CRITICAL_COLLISION_TTC: 1.5,

  /** Certificate validity period in days */
  CERTIFICATE_VALIDITY: 7
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

/**
 * V2X event handler
 */
export type V2XEventHandler<T = any> = (data: T) => void;

/**
 * Message filter predicate
 */
export type MessageFilter<T> = (message: T) => boolean;

// ============================================================================
// Export All Types
// ============================================================================

export type {
  Position,
  Acceleration,
  VehicleDimensions,
  BrakeStatus,
  BSM,
  BSMPartI,
  BSMPartII,
  CAM,
  DENM,
  SPaTMessage,
  SignalState,
  MAPMessage,
  Lane,
  PSM,
  V2XConfig,
  V2VMessage,
  CollisionWarning,
  V2IMessage,
  RSUInfo,
  PedestrianDetection,
  V2NTelemetry,
  TrafficUpdate,
  TrafficIncident,
  Platoon,
  PlatoonJoinRequest,
  PlatoonJoinResponse,
  PlatoonManeuver,
  WeatherInfo,
  SpeedLimit,
  CongestionInfo,
  RoadClosure
};

export {
  MessageType,
  StationType,
  VehicleClass,
  VehicleRole,
  CauseCode,
  Severity,
  SignalPhase,
  V2XErrorCode,
  V2XError,
  V2X_CONSTANTS
};
