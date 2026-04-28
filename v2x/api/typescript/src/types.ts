/**
 * WIA-AUTO-003: V2X - Vehicle-to-Everything Communication - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Automotive Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Geographic and Position Types
// ============================================================================

/**
 * Geographic position with latitude, longitude, and altitude
 */
export interface GeoPosition {
  /** Latitude in decimal degrees (-90 to +90) */
  lat: number;

  /** Longitude in decimal degrees (-180 to +180) */
  lon: number;

  /** Altitude/elevation in meters above sea level */
  alt: number;

  /** Position accuracy in meters (optional) */
  accuracy?: PositionalAccuracy;
}

/**
 * Position accuracy (semi-major/minor axis)
 */
export interface PositionalAccuracy {
  /** Semi-major axis in meters */
  semiMajor: number;

  /** Semi-minor axis in meters */
  semiMinor: number;

  /** Orientation in degrees (0-359) */
  orientation: number;
}

/**
 * Three-dimensional vector (acceleration, velocity, etc.)
 */
export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

/**
 * Two-dimensional position (relative or absolute)
 */
export interface Position2D {
  x: number;
  y: number;
}

// ============================================================================
// V2X Communication Types
// ============================================================================

/**
 * V2X communication technology
 */
export enum V2XTechnology {
  DSRC = 'DSRC',
  C_V2X = 'C-V2X',
  LTE_V2X = 'LTE-V2X',
  NR_V2X = '5G-V2X',
  HYBRID = 'Hybrid'
}

/**
 * V2X communication mode
 */
export enum CommunicationMode {
  V2V = 'V2V',           // Vehicle-to-Vehicle
  V2I = 'V2I',           // Vehicle-to-Infrastructure
  V2P = 'V2P',           // Vehicle-to-Pedestrian
  V2N = 'V2N',           // Vehicle-to-Network
  V2C = 'V2C'            // Vehicle-to-Cloud
}

/**
 * V2X message type identifiers (SAE J2735)
 */
export enum MessageType {
  BSM = 20,              // Basic Safety Message
  MAP = 18,              // Map Data
  SPAT = 19,             // Signal Phase and Timing
  TIM = 31,              // Traveler Information Message
  PSM = 32,              // Personal Safety Message
  DENM = 1,              // Decentralized Environmental Notification (ETSI)
  CAM = 2,               // Cooperative Awareness Message (ETSI)
  PVD = 26,              // Probe Vehicle Data
  RSA = 27,              // Road Side Alert
  SSM = 30,              // Signal Status Message
  EVA = 6                // Emergency Vehicle Alert
}

/**
 * V2X communication configuration
 */
export interface V2XConfig {
  /** Unique vehicle identifier */
  vehicleId: string;

  /** Communication technology */
  technology: V2XTechnology;

  /** Operating frequency in MHz */
  frequency: number;

  /** Transmission power in dBm */
  transmitPower?: number;

  /** BSM broadcast rate in Hz */
  bsmRate?: number;

  /** Security level */
  securityLevel: 'low' | 'medium' | 'high';

  /** Enable message encryption */
  encryption?: boolean;

  /** Maximum transmission range in meters */
  maxRange?: number;
}

/**
 * Connection status
 */
export interface ConnectionStatus {
  /** Is V2X communication active? */
  active: boolean;

  /** Current technology in use */
  technology: V2XTechnology;

  /** Signal strength (RSSI) in dBm */
  signalStrength?: number;

  /** Number of nearby vehicles detected */
  nearbyVehicles: number;

  /** Number of RSUs in range */
  nearbyRSUs: number;

  /** Channel load (0-100%) */
  channelLoad?: number;

  /** Packet delivery ratio (0-1) */
  pdr?: number;

  /** Average latency in milliseconds */
  latency?: number;

  /** Last communication timestamp */
  lastUpdate: number;
}

// ============================================================================
// Basic Safety Message (BSM) / Cooperative Awareness Message (CAM)
// ============================================================================

/**
 * Basic Safety Message - Core data (Part I)
 */
export interface BasicSafetyMessage {
  /** Message type (always 20 for BSM) */
  messageId: MessageType.BSM;

  /** Message count (0-127, wraps around) */
  msgCnt: number;

  /** Temporary vehicle ID (for privacy) */
  id: string;

  /** Time mark (milliseconds within minute) */
  secMark: number;

  /** Vehicle position */
  position: GeoPosition;

  /** Speed in m/s */
  speed: number;

  /** Heading in degrees (0-359.9875) */
  heading: number;

  /** Steering wheel angle in degrees */
  steeringAngle?: number;

  /** 4-way acceleration set */
  acceleration: {
    /** Longitudinal acceleration (m/s²) */
    long: number;
    /** Lateral acceleration (m/s²) */
    lat: number;
    /** Vertical acceleration (m/s²) */
    vert: number;
    /** Yaw rate (degrees/s) */
    yaw: number;
  };

  /** Brake system status */
  brakes: BrakeStatus;

  /** Vehicle size */
  vehicleSize: VehicleSize;

  /** Transmission state */
  transmission?: TransmissionState;

  /** Extended data (Part II) */
  partII?: BSMPartII;

  /** Message timestamp */
  timestamp: number;

  /** Sender's certificate (for security) */
  certificate?: string;

  /** Message signature (for security) */
  signature?: string;
}

/**
 * BSM Part II - Optional extended data
 */
export interface BSMPartII {
  /** Path history (recent positions) */
  pathHistory?: PathHistory[];

  /** Path prediction (future trajectory) */
  pathPrediction?: PathPrediction[];

  /** Vehicle classification */
  vehicleClass?: VehicleType;

  /** Exterior lights status */
  lights?: LightStatus;

  /** Special vehicle status (emergency, transit, etc.) */
  specialVehicle?: SpecialVehicleStatus;

  /** Weather report from vehicle sensors */
  weather?: WeatherReport;

  /** Safety extensions (ABS, traction control, etc.) */
  safetyExtensions?: SafetyExtensions;
}

/**
 * Brake system status
 */
export interface BrakeStatus {
  /** Brake pedal applied */
  brakeApplied: boolean;

  /** Anti-lock braking system active */
  abs?: boolean;

  /** Traction control active */
  tractionControl?: boolean;

  /** Stability control active */
  stabilityControl?: boolean;

  /** Brake boost active */
  brakeBoost?: boolean;

  /** Auxiliary brakes active */
  auxiliaryBrakes?: boolean;
}

/**
 * Vehicle size dimensions
 */
export interface VehicleSize {
  /** Vehicle width in centimeters */
  width: number;

  /** Vehicle length in centimeters */
  length: number;

  /** Vehicle height in centimeters (optional) */
  height?: number;
}

/**
 * Transmission state
 */
export enum TransmissionState {
  NEUTRAL = 'neutral',
  PARK = 'park',
  FORWARD_GEARS = 'forwardGears',
  REVERSE_GEARS = 'reverseGears',
  UNAVAILABLE = 'unavailable'
}

/**
 * Vehicle type classification
 */
export enum VehicleType {
  PASSENGER_CAR = 'passengerCar',
  MOTORCYCLE = 'motorcycle',
  BUS = 'bus',
  LIGHT_TRUCK = 'lightTruck',
  HEAVY_TRUCK = 'heavyTruck',
  TRAILER = 'trailer',
  SPECIAL_VEHICLE = 'specialVehicle',
  TRAM = 'tram',
  EMERGENCY = 'emergency',
  UNKNOWN = 'unknown'
}

/**
 * Path history waypoint
 */
export interface PathHistory {
  /** Relative position offset from current */
  offset: Position2D;

  /** Time elapsed since this position (ms) */
  timeOffset: number;

  /** Speed at this position (m/s) */
  speed?: number;

  /** Heading at this position (degrees) */
  heading?: number;
}

/**
 * Path prediction waypoint
 */
export interface PathPrediction {
  /** Predicted position offset */
  offset: Position2D;

  /** Time until this position (ms) */
  timeOffset: number;

  /** Confidence (0-1) */
  confidence: number;
}

/**
 * Exterior lights status
 */
export interface LightStatus {
  /** Headlights on */
  headlights?: boolean;

  /** High beams on */
  highBeams?: boolean;

  /** Left turn signal */
  leftTurnSignal?: boolean;

  /** Right turn signal */
  rightTurnSignal?: boolean;

  /** Hazard lights on */
  hazardLights?: boolean;

  /** Brake lights on */
  brakeLights?: boolean;

  /** Fog lights on */
  fogLights?: boolean;

  /** Parking lights on */
  parkingLights?: boolean;

  /** Daytime running lights on */
  drl?: boolean;
}

/**
 * Special vehicle status
 */
export interface SpecialVehicleStatus {
  /** Vehicle type */
  type: 'emergency' | 'transit' | 'maintenance' | 'police' | 'fire' | 'ambulance';

  /** Emergency lights active */
  emergencyLights?: boolean;

  /** Siren active */
  siren?: boolean;

  /** Requesting signal priority */
  priorityRequest?: boolean;

  /** Response type (for emergency vehicles) */
  responseType?: 'code1' | 'code2' | 'code3';
}

/**
 * Weather report from vehicle sensors
 */
export interface WeatherReport {
  /** Outside temperature in Celsius */
  temperature?: number;

  /** Barometric pressure in kPa */
  pressure?: number;

  /** Windshield wipers status */
  wipers?: 'off' | 'intermittent' | 'low' | 'high';

  /** Rain detected */
  rain?: boolean;

  /** Ambient light level (0-100%) */
  ambientLight?: number;
}

/**
 * Safety system extensions
 */
export interface SafetyExtensions {
  /** ABS engaged */
  abs?: boolean;

  /** Traction control engaged */
  tractionControl?: boolean;

  /** Stability control engaged */
  stabilityControl?: boolean;

  /** Airbag deployed */
  airbagDeployed?: boolean;

  /** Pre-collision system active */
  preCollisionSystem?: boolean;
}

// ============================================================================
// Signal Phase and Timing (SPaT)
// ============================================================================

/**
 * Signal Phase and Timing message
 */
export interface SignalPhaseAndTiming {
  /** Message type (always 19 for SPaT) */
  messageId: MessageType.SPAT;

  /** Timestamp */
  timestamp: number;

  /** List of intersection signal states */
  intersections: IntersectionState[];
}

/**
 * Intersection signal state
 */
export interface IntersectionState {
  /** Intersection identifier */
  id: string;

  /** Geographic position of intersection */
  position?: GeoPosition;

  /** Signal groups (per movement/lane) */
  signalGroups: SignalGroup[];

  /** Pedestrian signals */
  pedestrianSignals?: PedestrianSignal[];

  /** Intersection status */
  status?: IntersectionStatus;
}

/**
 * Signal group (per movement)
 */
export interface SignalGroup {
  /** Signal group ID */
  id: number;

  /** Movement name (e.g., "North-South Green") */
  movementName?: string;

  /** Current signal phase */
  state: SignalPhase;

  /** Time remaining in current phase (seconds) */
  timeRemaining: number;

  /** Next signal phase */
  nextState?: SignalPhase;

  /** Time until next phase (seconds) */
  timeToNextState?: number;

  /** Confidence level (0-1) */
  confidence?: number;
}

/**
 * Signal phase state
 */
export enum SignalPhase {
  DARK = 'dark',
  STOP_AND_REMAIN = 'stop-and-remain',                    // Red
  STOP_THEN_PROCEED = 'stop-then-proceed',                // Stop sign
  PRE_MOVEMENT = 'pre-movement',                          // Red + Yellow
  PERMISSIVE_MOVEMENT_ALLOWED = 'permissive-movement-allowed',  // Green
  PROTECTED_MOVEMENT_ALLOWED = 'protected-movement-allowed',    // Green arrow
  PERMISSIVE_CLEARANCE = 'permissive-clearance',          // Yellow
  PROTECTED_CLEARANCE = 'protected-clearance',            // Yellow arrow
  CAUTION_CONFLICTING_TRAFFIC = 'caution-conflicting-traffic'   // Flashing red/yellow
}

/**
 * Pedestrian signal
 */
export interface PedestrianSignal {
  /** Signal ID */
  id: number;

  /** Current state */
  state: 'walk' | 'dont-walk' | 'flashing-dont-walk';

  /** Time remaining (seconds) */
  timeRemaining: number;
}

/**
 * Intersection status flags
 */
export interface IntersectionStatus {
  /** Manual control active */
  manualControl?: boolean;

  /** Traffic signal failure */
  failure?: boolean;

  /** Preemption active (emergency vehicle) */
  preemption?: boolean;

  /** Signal priority active (transit) */
  priorityActive?: boolean;

  /** Fixed timing mode */
  fixedTiming?: boolean;
}

// ============================================================================
// Map Data (MAP)
// ============================================================================

/**
 * Map data message
 */
export interface MapData {
  /** Message type (always 18 for MAP) */
  messageId: MessageType.MAP;

  /** Timestamp */
  timestamp: number;

  /** List of intersections */
  intersections?: IntersectionGeometry[];

  /** List of road segments */
  roadSegments?: RoadSegment[];
}

/**
 * Intersection geometry
 */
export interface IntersectionGeometry {
  /** Intersection ID */
  id: string;

  /** Reference position (center of intersection) */
  refPoint: GeoPosition;

  /** Intersection name */
  name?: string;

  /** Approach lanes */
  approaches: LaneGeometry[];

  /** Speed limits */
  speedLimits?: SpeedLimit[];
}

/**
 * Lane geometry
 */
export interface LaneGeometry {
  /** Lane ID */
  id: number;

  /** Lane type */
  type: LaneType;

  /** Lane width in cm */
  width: number;

  /** Node list (sequence of positions) */
  nodes: LaneNode[];

  /** Connected lanes */
  connections?: LaneConnection[];

  /** Lane attributes */
  attributes?: LaneAttributes;
}

/**
 * Lane type
 */
export enum LaneType {
  VEHICLE = 'vehicle',
  BICYCLE = 'bicycle',
  PEDESTRIAN = 'pedestrian',
  PARKING = 'parking',
  BUS = 'bus',
  HOV = 'hov',
  TURN_LANE = 'turnLane',
  EXIT_LANE = 'exitLane',
  MERGE_LANE = 'mergeLane'
}

/**
 * Lane node (waypoint)
 */
export interface LaneNode {
  /** Offset from reference point */
  offset: Position2D;

  /** Lane width at this point (cm) */
  width?: number;

  /** Elevation (cm) */
  elevation?: number;
}

/**
 * Lane connection (ingress to egress)
 */
export interface LaneConnection {
  /** Connected lane ID */
  laneId: number;

  /** Connection type */
  maneuver?: 'straight' | 'left' | 'right' | 'u-turn';

  /** Signal group controlling this connection */
  signalGroup?: number;
}

/**
 * Lane attributes
 */
export interface LaneAttributes {
  /** Directional use */
  direction?: 'ingress' | 'egress' | 'bidirectional';

  /** Shared with other users */
  sharedWith?: ('pedestrian' | 'bicycle' | 'bus')[];

  /** Lane purpose */
  purpose?: 'through' | 'left-turn' | 'right-turn' | 'parking';
}

/**
 * Speed limit
 */
export interface SpeedLimit {
  /** Speed limit type */
  type: 'vehicleMaxSpeed' | 'vehicleMinSpeed' | 'truckMaxSpeed';

  /** Speed in m/s */
  speed: number;
}

/**
 * Road segment
 */
export interface RoadSegment {
  /** Segment ID */
  id: string;

  /** Segment name */
  name?: string;

  /** Reference position */
  refPoint: GeoPosition;

  /** Lanes in this segment */
  lanes: LaneGeometry[];

  /** Speed limits */
  speedLimits?: SpeedLimit[];
}

// ============================================================================
// Traveler Information Message (TIM)
// ============================================================================

/**
 * Traveler Information Message
 */
export interface TravelerInformationMessage {
  /** Message type (always 31 for TIM) */
  messageId: MessageType.TIM;

  /** Message ID */
  msgId: string;

  /** Timestamp */
  timestamp: number;

  /** Data frames */
  dataFrames: TIMDataFrame[];
}

/**
 * TIM data frame
 */
export interface TIMDataFrame {
  /** Frame type */
  frameType: 'advisory' | 'roadSignage' | 'commercialSignage';

  /** Message content */
  content: TIMContent;

  /** Geographic region affected */
  region?: GeographicRegion;

  /** Priority (1-8, higher is more important) */
  priority?: number;

  /** Start time (optional) */
  startTime?: number;

  /** Duration in seconds (optional) */
  duration?: number;
}

/**
 * TIM content
 */
export interface TIMContent {
  /** Work zone information */
  workZone?: WorkZoneInfo;

  /** Road conditions */
  roadConditions?: RoadCondition[];

  /** Incident information */
  incident?: IncidentInfo;

  /** Parking information */
  parking?: ParkingInfo;

  /** Generic advisory text */
  text?: string;
}

/**
 * Work zone information
 */
export interface WorkZoneInfo {
  /** Location description */
  location: string;

  /** Start time */
  startTime?: number;

  /** End time */
  endTime?: number;

  /** Lane closures */
  laneClosures?: number[];

  /** Reduced speed limit (m/s) */
  speedLimit?: number;

  /** Workers present */
  workersPresent?: boolean;
}

/**
 * Road condition
 */
export interface RoadCondition {
  /** Condition type */
  type: 'icy' | 'wet' | 'flooded' | 'debris' | 'fog' | 'snow' | 'construction';

  /** Severity */
  severity: 'low' | 'medium' | 'high';

  /** Description */
  description?: string;
}

/**
 * Incident information
 */
export interface IncidentInfo {
  /** Incident type */
  type: 'accident' | 'disabled-vehicle' | 'spill' | 'congestion' | 'other';

  /** Severity */
  severity: 'minor' | 'moderate' | 'major';

  /** Location description */
  location: string;

  /** Lanes affected */
  lanesAffected?: number[];

  /** Expected clearance time */
  clearanceTime?: number;
}

/**
 * Parking information
 */
export interface ParkingInfo {
  /** Parking facility ID */
  facilityId: string;

  /** Available spaces */
  availableSpaces: number;

  /** Total capacity */
  totalCapacity: number;

  /** Occupancy percentage */
  occupancy: number;

  /** Parking rates */
  rates?: string;
}

/**
 * Geographic region
 */
export interface GeographicRegion {
  /** Region type */
  type: 'circle' | 'rectangle' | 'polygon';

  /** Center point (for circle) */
  center?: GeoPosition;

  /** Radius in meters (for circle) */
  radius?: number;

  /** Bounding box (for rectangle) */
  bounds?: {
    min: GeoPosition;
    max: GeoPosition;
  };

  /** Vertices (for polygon) */
  vertices?: GeoPosition[];
}

// ============================================================================
// Personal Safety Message (PSM)
// ============================================================================

/**
 * Personal Safety Message (for pedestrians, cyclists)
 */
export interface PersonalSafetyMessage {
  /** Message type (always 32 for PSM) */
  messageId: MessageType.PSM;

  /** User ID (temporary, for privacy) */
  id: string;

  /** Timestamp */
  timestamp: number;

  /** User type */
  userType: VulnerableUserType;

  /** Position */
  position: GeoPosition;

  /** Speed in m/s */
  speed: number;

  /** Heading in degrees */
  heading: number;

  /** Movement prediction */
  prediction?: PathPrediction[];

  /** Device type */
  deviceType?: 'smartphone' | 'wearable' | 'dedicated-device';

  /** Cluster size (number of people in group) */
  clusterSize?: number;

  /** Activity type */
  activity?: 'walking' | 'running' | 'cycling' | 'standing' | 'sitting';
}

/**
 * Vulnerable road user type
 */
export enum VulnerableUserType {
  PEDESTRIAN = 'pedestrian',
  CYCLIST = 'cyclist',
  MOTORCYCLIST = 'motorcyclist',
  WHEELCHAIR = 'wheelchair',
  SKATER = 'skater',
  SCOOTER = 'scooter',
  ANIMAL = 'animal',
  OTHER = 'other'
}

// ============================================================================
// Decentralized Environmental Notification Message (DENM) - ETSI
// ============================================================================

/**
 * Decentralized Environmental Notification Message
 */
export interface EnvironmentalNotificationMessage {
  /** Message type (always 1 for DENM) */
  messageId: MessageType.DENM;

  /** Station ID (sender) */
  stationId: string;

  /** Timestamp */
  timestamp: number;

  /** Event type */
  eventType: DENMEventType;

  /** Event position */
  position: GeoPosition;

  /** Severity */
  severity: 'low' | 'medium' | 'high' | 'critical';

  /** Detection time */
  detectionTime: number;

  /** Validity duration (seconds) */
  validityDuration: number;

  /** Event description */
  description?: string;

  /** Speed advice (m/s) */
  speedAdvice?: number;

  /** Affected lanes */
  affectedLanes?: number[];

  /** Cause code */
  causeCode?: number;

  /** Sub-cause code */
  subCauseCode?: number;
}

/**
 * DENM event type
 */
export enum DENMEventType {
  TRAFFIC_JAM = 'trafficJam',
  ACCIDENT = 'accident',
  ROADWORKS = 'roadworks',
  ADVERSE_WEATHER = 'adverseWeather',
  HAZARDOUS_LOCATION = 'hazardousLocation',
  EMERGENCY_VEHICLE = 'emergencyVehicle',
  SLOW_VEHICLE = 'slowVehicle',
  STATIONARY_VEHICLE = 'stationaryVehicle',
  WRONG_WAY_DRIVING = 'wrongWayDriving',
  POST_CRASH = 'postCrash',
  EMERGENCY_BRAKE = 'emergencyBrake',
  VEHICLE_BREAKDOWN = 'vehicleBreakdown'
}

// ============================================================================
// Security and Certificates
// ============================================================================

/**
 * Security certificate information
 */
export interface CertificateInfo {
  /** Certificate ID */
  certificateId: string;

  /** Certificate type */
  type: 'enrollment' | 'pseudonym' | 'authorization';

  /** Public key */
  publicKey: string;

  /** Issuer (CA) */
  issuer: string;

  /** Valid from timestamp */
  validFrom: number;

  /** Valid until timestamp */
  validUntil: number;

  /** Permissions (message types allowed) */
  permissions: MessageType[];

  /** Geographic scope (optional) */
  geographicScope?: GeographicRegion;

  /** Certificate status */
  status: 'active' | 'expired' | 'revoked';
}

/**
 * Message security wrapper
 */
export interface SecuredMessage<T> {
  /** Protocol version */
  version: number;

  /** Message content */
  payload: T;

  /** Signer certificate */
  certificate: CertificateInfo;

  /** Digital signature */
  signature: string;

  /** Signature algorithm */
  signatureAlgorithm: 'ECDSA-256' | 'ECDSA-384';

  /** Generation time */
  generationTime: number;

  /** Encryption (optional) */
  encryption?: {
    algorithm: 'AES-128-CCM';
    recipients: string[];
  };
}

/**
 * Certificate verification result
 */
export interface VerificationResult {
  /** Is message valid? */
  isValid: boolean;

  /** Certificate is valid and trusted */
  certificateValid: boolean;

  /** Signature verification passed */
  signatureValid: boolean;

  /** Certificate not revoked */
  notRevoked: boolean;

  /** Message plausibility check passed */
  plausibilityCheck: boolean;

  /** Verification errors */
  errors?: string[];

  /** Warnings */
  warnings?: string[];

  /** Verification timestamp */
  timestamp: number;
}

// ============================================================================
// Application-Level Types
// ============================================================================

/**
 * Collision warning
 */
export interface CollisionWarning {
  /** Warning ID */
  id: string;

  /** Severity level */
  severity: 'low' | 'medium' | 'high' | 'critical';

  /** Time to collision in seconds */
  timeToCollision: number;

  /** Distance to collision point in meters */
  distance: number;

  /** Relative speed (m/s, positive = closing) */
  relativeSpeed: number;

  /** Collision type */
  collisionType: 'frontal' | 'rear' | 'side' | 'intersection' | 'pedestrian';

  /** Other vehicle/object information */
  otherVehicle?: NearbyVehicle;

  /** Recommended action */
  recommendedAction: 'brake' | 'steer-left' | 'steer-right' | 'accelerate' | 'none';

  /** Warning timestamp */
  timestamp: number;
}

/**
 * Nearby vehicle information
 */
export interface NearbyVehicle {
  /** Vehicle ID (temporary/pseudonym) */
  vehicleId: string;

  /** Absolute position */
  position: GeoPosition;

  /** Relative position (to ego vehicle) */
  relativePosition: Vector3;

  /** Speed (m/s) */
  speed: number;

  /** Heading (degrees) */
  heading: number;

  /** Vehicle type */
  vehicleType: VehicleType;

  /** Vehicle size */
  size?: VehicleSize;

  /** Last BSM received timestamp */
  lastUpdate: number;

  /** Confidence level (0-1) */
  confidence: number;

  /** Distance from ego vehicle (meters) */
  distance: number;

  /** Bearing from ego vehicle (degrees) */
  bearing: number;
}

/**
 * Traffic signal information
 */
export interface TrafficSignalInfo {
  /** Intersection ID */
  intersectionId: string;

  /** Current signal phase */
  currentPhase: SignalPhase;

  /** Time remaining in current phase (seconds) */
  timeRemaining: number;

  /** Next phase */
  nextPhase?: SignalPhase;

  /** Distance to intersection (meters) */
  distance: number;

  /** Recommended speed for green wave (m/s) */
  recommendedSpeed?: number;

  /** Will arrive on red light? */
  willArriveOnRed: boolean;

  /** Signal group ID */
  signalGroupId?: number;
}

/**
 * Platoon information
 */
export interface PlatoonInfo {
  /** Platoon ID */
  platoonId: string;

  /** Lead vehicle ID */
  leadVehicle: string;

  /** Member vehicle IDs */
  members: string[];

  /** Target speed (m/s) */
  targetSpeed: number;

  /** Target inter-vehicle spacing (meters) */
  targetSpacing: number;

  /** Ego vehicle position in platoon (0 = leader) */
  position: number;

  /** Platoon status */
  status: 'forming' | 'active' | 'dissolving';

  /** Total platoon length (meters) */
  length: number;
}

/**
 * Vehicle state (ego vehicle)
 */
export interface VehicleState {
  /** Current position */
  position: GeoPosition;

  /** Speed (m/s) */
  speed: number;

  /** Heading (degrees) */
  heading: number;

  /** Acceleration */
  acceleration: Vector3;

  /** Steering angle (degrees) */
  steeringAngle: number;

  /** Brake status */
  brakes: BrakeStatus;

  /** Lights status */
  lights: LightStatus;

  /** Transmission state */
  transmission: TransmissionState;

  /** Vehicle type */
  vehicleType: VehicleType;

  /** Timestamp */
  timestamp: number;
}

/**
 * Hazard warning
 */
export interface HazardWarning {
  /** Hazard ID */
  id: string;

  /** Hazard type */
  type: DENMEventType;

  /** Hazard location */
  position: GeoPosition;

  /** Severity */
  severity: 'low' | 'medium' | 'high' | 'critical';

  /** Distance from ego vehicle (meters) */
  distance: number;

  /** Description */
  description: string;

  /** Speed advice (m/s) */
  speedAdvice?: number;

  /** Detection time */
  detectionTime: number;

  /** Validity duration (seconds) */
  validityDuration: number;

  /** Source vehicle ID */
  sourceId?: string;
}

// ============================================================================
// Communication Statistics and Monitoring
// ============================================================================

/**
 * V2X communication statistics
 */
export interface CommunicationStats {
  /** Total messages sent */
  messagesSent: number;

  /** Total messages received */
  messagesReceived: number;

  /** Messages sent by type */
  sentByType: Record<MessageType, number>;

  /** Messages received by type */
  receivedByType: Record<MessageType, number>;

  /** Average latency (ms) */
  averageLatency: number;

  /** Packet delivery ratio (0-1) */
  packetDeliveryRatio: number;

  /** Channel utilization (0-1) */
  channelUtilization: number;

  /** Signature verification success rate (0-1) */
  verificationSuccessRate: number;

  /** Number of detected misbehaving vehicles */
  misbehavingVehicles: number;

  /** Statistics collection start time */
  startTime: number;

  /** Statistics last update time */
  lastUpdate: number;
}

/**
 * Misbehavior report
 */
export interface MisbehaviorReport {
  /** Report ID */
  id: string;

  /** Suspected vehicle ID */
  suspectedVehicleId: string;

  /** Misbehavior type */
  type: 'invalid-signature' | 'position-implausible' | 'speed-implausible' | 'duplicate-messages' | 'jamming' | 'other';

  /** Description */
  description: string;

  /** Evidence (message samples) */
  evidence?: any[];

  /** Reporter vehicle ID */
  reporterId: string;

  /** Report timestamp */
  timestamp: number;

  /** Confidence (0-1) */
  confidence: number;
}

// ============================================================================
// Constants
// ============================================================================

/**
 * V2X physical constants and limits
 */
export const V2X_CONSTANTS = {
  /** DSRC frequency in MHz */
  DSRC_FREQUENCY: 5900,

  /** Maximum DSRC range in meters */
  DSRC_MAX_RANGE: 1000,

  /** Maximum C-V2X range in meters */
  CV2X_MAX_RANGE: 1500,

  /** Maximum 5G-V2X range in meters */
  NR_V2X_MAX_RANGE: 2000,

  /** Target latency for safety messages (ms) */
  SAFETY_MESSAGE_LATENCY: 100,

  /** BSM transmission rate (Hz) */
  BSM_RATE: 10,

  /** SPaT transmission rate (Hz) */
  SPAT_RATE: 10,

  /** Maximum message size (bytes) */
  MAX_MESSAGE_SIZE: 1500,

  /** BSM typical size (bytes) */
  BSM_SIZE: 200,

  /** Certificate validity (days) */
  CERTIFICATE_VALIDITY: 7,

  /** Number of pseudonym certificates */
  PSEUDONYM_CERT_COUNT: 20,

  /** Certificate rotation interval (minutes) */
  CERT_ROTATION_INTERVAL: 5,

  /** Speed of light (m/s) */
  SPEED_OF_LIGHT: 299792458,

  /** Earth radius (meters) */
  EARTH_RADIUS: 6371000,
} as const;

// ============================================================================
// Export All
// ============================================================================

export type {
  // Geo types
  GeoPosition,
  PositionalAccuracy,
  Vector3,
  Position2D,

  // Communication types
  V2XConfig,
  ConnectionStatus,

  // Messages
  BasicSafetyMessage,
  BSMPartII,
  SignalPhaseAndTiming,
  MapData,
  TravelerInformationMessage,
  PersonalSafetyMessage,
  EnvironmentalNotificationMessage,

  // Message components
  BrakeStatus,
  VehicleSize,
  PathHistory,
  PathPrediction,
  LightStatus,
  SpecialVehicleStatus,
  WeatherReport,
  SafetyExtensions,
  IntersectionState,
  SignalGroup,
  PedestrianSignal,
  IntersectionStatus,
  IntersectionGeometry,
  LaneGeometry,
  LaneNode,
  LaneConnection,
  LaneAttributes,
  SpeedLimit,
  RoadSegment,
  TIMDataFrame,
  TIMContent,
  WorkZoneInfo,
  RoadCondition,
  IncidentInfo,
  ParkingInfo,
  GeographicRegion,

  // Security
  CertificateInfo,
  SecuredMessage,
  VerificationResult,

  // Application
  CollisionWarning,
  NearbyVehicle,
  TrafficSignalInfo,
  PlatoonInfo,
  VehicleState,
  HazardWarning,
  CommunicationStats,
  MisbehaviorReport,
};

export {
  V2XTechnology,
  CommunicationMode,
  MessageType,
  TransmissionState,
  VehicleType,
  SignalPhase,
  LaneType,
  VulnerableUserType,
  DENMEventType,
  V2X_CONSTANTS,
};
