/**
 * WIA-AUTO-018: Railway System - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Railway Engineering Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Railway Types
// ============================================================================

/**
 * Geographic position with accuracy
 */
export interface Position {
  latitude: number;
  longitude: number;
  altitude?: number;
  accuracy?: number;
}

/**
 * Track identification
 */
export interface TrackLocation {
  /** Track identifier */
  trackId: string;

  /** Milepost or kilometer post */
  milepost: number;

  /** Position on track */
  position?: Position;

  /** Direction of travel */
  direction?: 'forward' | 'reverse';
}

/**
 * Speed and movement information
 */
export interface Movement {
  /** Current speed in km/h */
  speed: number;

  /** Acceleration in m/s² */
  acceleration?: number;

  /** Jerk (rate of acceleration change) in m/s³ */
  jerk?: number;

  /** Heading in degrees (0-360) */
  heading?: number;

  /** Direction */
  direction: 'forward' | 'reverse' | 'stopped';
}

// ============================================================================
// Train Types
// ============================================================================

/**
 * Train identification and classification
 */
export interface Train {
  /** Unique train identifier */
  trainId: string;

  /** Line or service identifier */
  line: string;

  /** Train operator */
  operator: string;

  /** Train type */
  type: 'electric_multiple_unit' | 'diesel_multiple_unit' | 'locomotive_hauled' |
        'high_speed' | 'metro' | 'light_rail' | 'freight';

  /** Number of cars/carriages */
  formation: string;

  /** Total train length in meters */
  length?: number;

  /** Total train mass in tonnes */
  mass?: number;

  /** Passenger capacity */
  capacity?: {
    seated: number;
    standing: number;
    total: number;
  };
}

/**
 * Complete train status
 */
export interface TrainStatus {
  /** Train information */
  train: Train;

  /** Current position */
  position: Position & TrackLocation;

  /** Movement information */
  movement: Movement;

  /** Operational status */
  status: {
    operationalMode: 'manual' | 'ATP' | 'ATO_supervised' | 'ATO_driverless' | 'ATO_unattended';
    doors: 'open' | 'closed_locked' | 'closing' | 'opening' | 'fault';
    brakes: 'released' | 'applied' | 'emergency' | 'parking';
    pantograph?: 'raised' | 'lowered';
    traction: 'enabled' | 'disabled' | 'coasting';
    serviceStatus: 'in_service' | 'out_of_service' | 'testing' | 'maintenance';
  };

  /** Safety system status */
  safety: SafetyStatus;

  /** Next station and ETA */
  schedule?: {
    nextStation: string;
    etaSeconds: number;
    delaySeconds: number;
  };

  /** Timestamp of status */
  timestamp: Date | string;
}

/**
 * Safety system status
 */
export interface SafetyStatus {
  /** ATP (Automatic Train Protection) active */
  atpActive: boolean;

  /** Movement authority endpoint */
  movementAuthorityEnd?: number;

  /** Current permitted speed in km/h */
  permittedSpeed: number;

  /** Target speed in km/h */
  targetSpeed: number;

  /** Vigilance/deadman system status */
  vigilanceStatus?: 'acknowledged' | 'warning' | 'intervention';

  /** Emergency brake status */
  emergencyBrake: boolean;

  /** System faults */
  faults: string[];
}

// ============================================================================
// Signaling Types
// ============================================================================

/**
 * Signal aspect types
 */
export type SignalAspect =
  | 'RED'           // Stop
  | 'YELLOW'        // Approach
  | 'DOUBLE_YELLOW' // Preliminary caution
  | 'GREEN'         // Proceed
  | 'LUNAR_WHITE'   // Shunting permitted
  | 'FLASHING_YELLOW' // Advance approach
  | 'DARK';         // Signal not in use

/**
 * Signal status
 */
export interface Signal {
  /** Unique signal identifier */
  signalId: string;

  /** Signal location */
  location: Position & TrackLocation;

  /** Current aspect */
  aspect: SignalAspect;

  /** Aspect code (single character) */
  aspectCode: string;

  /** Speed limit at this signal (km/h) */
  speedLimit: number;

  /** Distance to next signal (meters) */
  distanceToNext?: number;

  /** Next signal's aspect */
  nextSignalAspect?: SignalAspect;

  /** Timestamp of last update */
  updatedAt: Date | string;
}

/**
 * Route information
 */
export interface Route {
  /** Route identifier */
  routeId: string;

  /** Route type */
  routeType: 'main_line' | 'siding' | 'crossover' | 'platform' | 'yard';

  /** Points (switch) positions */
  pointsPosition: 'normal' | 'reverse' | 'mixed';

  /** Overlap distance (safety distance beyond signal) */
  overlapDistance: number;

  /** Route is locked */
  locked: boolean;

  /** Route reserved for train */
  reservedFor?: string;
}

/**
 * Movement Authority
 */
export interface MovementAuthority {
  /** End location (milepost or position) */
  endLocation: number;

  /** Speed at end location (usually 0 for stop) */
  endSpeed: number;

  /** Speed profile along the route */
  profile: SpeedProfile[];

  /** Gradient profile */
  gradientProfile?: GradientProfile[];

  /** Validity timeout */
  validUntil?: Date | string;

  /** Reason for authority */
  reason?: string;
}

/**
 * Speed profile point
 */
export interface SpeedProfile {
  /** Location (milepost) */
  location: number;

  /** Speed limit at this location (km/h) */
  speed: number;

  /** Reason for speed limit */
  reason?: 'signal' | 'temporary' | 'permanent' | 'curve' | 'gradient' | 'station';
}

/**
 * Gradient profile point
 */
export interface GradientProfile {
  /** Location (milepost) */
  location: number;

  /** Gradient in per-mille (‰) */
  gradient: number;
}

// ============================================================================
// Train Control Types
// ============================================================================

/**
 * ETCS Level
 */
export type ETCSLevel = '0' | '1' | '2' | '3' | 'NTC' | 'STM';

/**
 * ETCS Mode
 */
export type ETCSMode =
  | 'Full Supervision'
  | 'On Sight'
  | 'Staff Responsible'
  | 'Shunting'
  | 'Unfitted'
  | 'Standby'
  | 'Reversing'
  | 'Limited Supervision';

/**
 * ETCS status
 */
export interface ETCSStatus {
  /** Current ETCS level */
  level: ETCSLevel;

  /** Current ETCS mode */
  mode: ETCSMode;

  /** Connection to RBC (Radio Block Center) */
  rbcConnection: boolean;

  /** RBC identifier */
  rbcId?: string;

  /** Movement authority */
  movementAuthority: MovementAuthority;

  /** Braking curve parameters */
  brakingCurve: BrakingCurve;
}

/**
 * Braking curve
 */
export interface BrakingCurve {
  /** Service brake intervention speed curve */
  serviceBrake: CurvePoint[];

  /** Emergency brake intervention speed curve */
  emergencyBrake: CurvePoint[];

  /** Permitted speed curve */
  permitted: CurvePoint[];

  /** Warning curve */
  warning: CurvePoint[];
}

/**
 * Curve point (distance vs speed)
 */
export interface CurvePoint {
  /** Distance from current position (meters) */
  distance: number;

  /** Speed at this distance (km/h) */
  speed: number;
}

/**
 * CBTC (Communications-Based Train Control) status
 */
export interface CBTCStatus {
  /** Connection to zone controller */
  zoneControllerConnection: boolean;

  /** Zone controller ID */
  zoneControllerId?: string;

  /** Grade of Automation */
  goA: 0 | 1 | 2 | 3 | 4;

  /** Leading train information */
  leadingTrain?: {
    trainId: string;
    distance: number;
    speed: number;
  };

  /** Safe separation distance */
  safeSeparation: number;

  /** Movement authority (moving block) */
  movementAuthority: MovementAuthority;
}

// ============================================================================
// Station and Platform Types
// ============================================================================

/**
 * Station information
 */
export interface Station {
  /** Unique station identifier */
  stationId: string;

  /** Station name */
  stationName: string;

  /** Station code (short identifier) */
  stationCode: string;

  /** Station location */
  location: Position;

  /** Number of platforms */
  platformCount: number;

  /** Station type */
  type: 'terminal' | 'through' | 'junction' | 'halt';

  /** Facilities available */
  facilities?: {
    ticketOffice: boolean;
    waitingRoom: boolean;
    accessibility: boolean;
    parkAndRide: boolean;
    bikeParking: boolean;
    retail: boolean;
  };
}

/**
 * Platform information
 */
export interface Platform {
  /** Platform identifier */
  platformId: string;

  /** Platform number/name */
  platformNumber: string;

  /** Platform section (for long platforms) */
  platformSection?: string;

  /** Platform length in meters */
  length: number;

  /** Platform height in mm */
  height: number;

  /** Track ID */
  trackId: string;

  /** Platform screen doors installed */
  psdInstalled: boolean;

  /** Platform location */
  location: Position;
}

/**
 * Platform Screen Door (PSD) status
 */
export interface PSDStatus {
  /** PSD installed */
  installed: boolean;

  /** Current status */
  status: 'open' | 'closed_locked' | 'opening' | 'closing' | 'fault' | 'emergency_open';

  /** Last operation timestamp */
  lastOperation: Date | string;

  /** All doors operational */
  operational: boolean;

  /** Individual door status (optional) */
  doors?: {
    doorNumber: number;
    status: 'open' | 'closed_locked' | 'fault';
    obstructionDetected: boolean;
  }[];
}

/**
 * Platform arrival/departure
 */
export interface PlatformEvent {
  /** Station information */
  station: Station;

  /** Platform information */
  platform: Platform;

  /** Train information */
  train: Train;

  /** Schedule information */
  schedule: {
    scheduledArrival: Date | string;
    estimatedArrival: Date | string;
    actualArrival?: Date | string;
    scheduledDeparture: Date | string;
    estimatedDeparture: Date | string;
    actualDeparture?: Date | string;
    delaySeconds: number;
    status: 'on_time' | 'delayed' | 'cancelled' | 'early';
  };

  /** Platform alignment */
  alignment: {
    aligned: boolean;
    stoppingPosition: string;
    accuracyCm: number;
  };

  /** Platform screen doors */
  psd?: PSDStatus;

  /** Passenger flow */
  passengers?: {
    alighting: number;
    boarding: number;
    waitingPlatform: number;
    loadFactorBefore: number;
    loadFactorAfter: number;
  };
}

// ============================================================================
// Braking and Dynamics
// ============================================================================

/**
 * Braking parameters
 */
export interface BrakingParameters {
  /** Initial velocity in km/h */
  initialVelocity: number;

  /** Deceleration rate in m/s² */
  decelerationRate: number;

  /** Reaction time in seconds */
  reactionTime: number;

  /** Safety margin in meters */
  safetyMargin: number;

  /** Gradient (‰ per thousand) */
  gradient?: number;

  /** Adhesion coefficient (0.2-0.4) */
  adhesion?: number;
}

/**
 * Braking distance calculation result
 */
export interface BrakingDistance {
  /** Total braking distance in meters */
  totalDistance: number;

  /** Reaction distance in meters */
  reactionDistance: number;

  /** Actual braking distance in meters */
  brakingDistance: number;

  /** Safety margin in meters */
  safetyMargin: number;

  /** Gradient effect in meters */
  gradientEffect?: number;

  /** Time to stop in seconds */
  timeToStop: number;

  /** Initial velocity in m/s */
  initialVelocityMs: number;
}

/**
 * Headway (train separation) calculation
 */
export interface HeadwayCalculation {
  /** Train speed in km/h */
  trainSpeed: number;

  /** Braking distance in meters */
  brakingDistance: number;

  /** Train length in meters */
  trainLength: number;

  /** Safety distance in meters */
  safetyDistance: number;

  /** Calculated minimum headway in meters */
  headway: number;

  /** Headway in seconds */
  headwaySeconds: number;

  /** Line capacity in trains per hour */
  capacity: number;
}

/**
 * Traction and resistance parameters
 */
export interface TractionParameters {
  /** Train mass in tonnes */
  mass: number;

  /** Speed in km/h */
  speed: number;

  /** Rolling resistance coefficient */
  rollingResistance?: number;

  /** Aerodynamic drag coefficient */
  dragCoefficient?: number;

  /** Frontal area in m² */
  frontalArea?: number;

  /** Gradient in ‰ */
  gradient?: number;

  /** Curve radius in meters */
  curveRadius?: number;
}

/**
 * Resistance calculation result
 */
export interface ResistanceForces {
  /** Total resistance in kN */
  totalResistance: number;

  /** Rolling resistance in kN */
  rollingResistance: number;

  /** Aerodynamic resistance in kN */
  aerodynamicResistance: number;

  /** Grade resistance in kN */
  gradeResistance: number;

  /** Curve resistance in kN */
  curveResistance: number;
}

// ============================================================================
// Communication Types
// ============================================================================

/**
 * GSM-R call information
 */
export interface GSMRCall {
  /** Call ID */
  callId: string;

  /** Caller number */
  caller: string;

  /** Called number */
  called: string;

  /** Call type */
  callType: 'point_to_point' | 'group_call' | 'broadcast' | 'emergency';

  /** Priority level (0-3) */
  priority: 0 | 1 | 2 | 3;

  /** Call status */
  status: 'connecting' | 'active' | 'on_hold' | 'ended';

  /** Call start time */
  startTime: Date | string;

  /** Call duration in seconds */
  duration?: number;
}

/**
 * Radio network status
 */
export interface RadioNetworkStatus {
  /** Network type */
  networkType: 'GSM-R' | 'LTE-R' | 'TETRA' | '5G-R';

  /** Connection status */
  connected: boolean;

  /** Signal strength (dBm) */
  signalStrength: number;

  /** Signal quality (0-100%) */
  signalQuality: number;

  /** Current cell ID */
  cellId?: string;

  /** Data rate (kbps) */
  dataRate?: number;

  /** Latency (ms) */
  latency?: number;
}

// ============================================================================
// Passenger Information Types
// ============================================================================

/**
 * Real-time departure information
 */
export interface DepartureInfo {
  /** Train information */
  trainId: string;
  line: string;
  destination: string;

  /** Timing */
  scheduledDeparture: Date | string;
  estimatedDeparture: Date | string;
  delaySeconds: number;

  /** Platform */
  platform: string;

  /** Status */
  status: 'on_time' | 'delayed' | 'cancelled' | 'boarding' | 'departed';

  /** Real-time tracking available */
  realTime: boolean;

  /** Train formation/length */
  formation?: string;

  /** Accessibility information */
  accessibility?: {
    wheelchairAccessible: boolean;
    audioVisualAnnouncements: boolean;
  };
}

/**
 * Station departures board
 */
export interface DeparturesBoard {
  /** Station information */
  stationId: string;
  stationName: string;

  /** Board timestamp */
  timestamp: Date | string;

  /** List of departures */
  departures: DepartureInfo[];

  /** Service alerts */
  alerts?: ServiceAlert[];
}

/**
 * Service alert
 */
export interface ServiceAlert {
  /** Alert ID */
  alertId: string;

  /** Alert type */
  type: 'delay' | 'cancellation' | 'platform_change' | 'service_disruption' | 'information';

  /** Severity */
  severity: 'info' | 'warning' | 'severe';

  /** Affected lines */
  affectedLines: string[];

  /** Affected stations */
  affectedStations?: string[];

  /** Alert message */
  message: string;

  /** Valid from */
  validFrom: Date | string;

  /** Valid until */
  validUntil?: Date | string;
}

/**
 * Passenger counting data
 */
export interface PassengerCount {
  /** Train ID */
  trainId: string;

  /** Total passengers on train */
  totalPassengers: number;

  /** Total capacity */
  capacity: number;

  /** Load factor (percentage) */
  loadFactor: number;

  /** Per-car breakdown */
  cars: {
    carNumber: number;
    passengers: number;
    capacity: number;
    loadFactor: number;
  }[];

  /** Timestamp */
  timestamp: Date | string;
}

// ============================================================================
// Safety and Validation Types
// ============================================================================

/**
 * Signal aspect validation
 */
export interface SignalValidation {
  /** Is the signal aspect valid for current conditions */
  isValid: boolean;

  /** Required action */
  requiredAction: 'proceed' | 'reduce_speed' | 'prepare_to_stop' | 'stop' | 'shunt';

  /** Permitted speed */
  permittedSpeed: number;

  /** Distance to next signal */
  distanceToNext: number;

  /** Warnings */
  warnings: string[];

  /** Errors */
  errors: string[];
}

/**
 * Safety Integrity Level
 */
export type SafetyIntegrityLevel = 'SIL-0' | 'SIL-1' | 'SIL-2' | 'SIL-3' | 'SIL-4';

/**
 * Safety check result
 */
export interface SafetyCheck {
  /** Check name */
  name: string;

  /** SIL level */
  silLevel: SafetyIntegrityLevel;

  /** Check passed */
  passed: boolean;

  /** Check result */
  result: 'pass' | 'fail' | 'warning';

  /** Description */
  description: string;

  /** Measured value */
  measuredValue?: number;

  /** Expected value */
  expectedValue?: number;

  /** Tolerance */
  tolerance?: number;

  /** Corrective action if failed */
  correctiveAction?: string;
}

/**
 * Emergency event
 */
export interface EmergencyEvent {
  /** Event ID */
  eventId: string;

  /** Event type */
  type: 'emergency_brake' | 'passenger_emergency' | 'collision' | 'derailment' |
        'fire' | 'medical' | 'security' | 'equipment_failure';

  /** Severity */
  severity: 'minor' | 'major' | 'critical' | 'catastrophic';

  /** Train involved */
  trainId: string;

  /** Location */
  location: Position & TrackLocation;

  /** Timestamp */
  timestamp: Date | string;

  /** Description */
  description: string;

  /** Response status */
  responseStatus: 'reported' | 'responding' | 'on_scene' | 'resolved';

  /** Emergency services notified */
  emergencyServices: {
    police: boolean;
    fire: boolean;
    ambulance: boolean;
    railway: boolean;
  };
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Railway physical constants
 */
export const RAILWAY_CONSTANTS = {
  /** Standard track gauge in mm */
  STANDARD_GAUGE: 1435,

  /** Broad gauge in mm (India, Spain, Portugal) */
  BROAD_GAUGE: 1668,

  /** Russian gauge in mm */
  RUSSIAN_GAUGE: 1520,

  /** Maximum service deceleration in m/s² */
  MAX_SERVICE_DECELERATION: 1.2,

  /** Typical service deceleration in m/s² */
  SERVICE_DECELERATION: 0.7,

  /** Emergency deceleration in m/s² */
  EMERGENCY_DECELERATION: 1.5,

  /** Standard platform height in mm */
  PLATFORM_HEIGHT_STANDARD: 1100,

  /** Low platform height in mm */
  PLATFORM_HEIGHT_LOW: 550,

  /** GSM-R uplink frequency in MHz */
  GSMR_UPLINK_START: 876,
  GSMR_UPLINK_END: 880,

  /** GSM-R downlink frequency in MHz */
  GSMR_DOWNLINK_START: 921,
  GSMR_DOWNLINK_END: 925,

  /** Gravitational acceleration in m/s² */
  GRAVITY: 9.81,

  /** Air density at sea level in kg/m³ */
  AIR_DENSITY: 1.225,

  /** Rolling resistance coefficient range */
  ROLLING_RESISTANCE_MIN: 0.0015,
  ROLLING_RESISTANCE_MAX: 0.003,

  /** Adhesion coefficient range */
  ADHESION_DRY: 0.35,
  ADHESION_WET: 0.25,
  ADHESION_ICE: 0.10,
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
 * Time range
 */
export interface TimeRange {
  from: Date | string;
  to: Date | string;
}

/**
 * Distance range
 */
export interface DistanceRange {
  from: number;
  to: number;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-AUTO-018 error codes
 */
export enum RailwayErrorCode {
  INVALID_SIGNAL = 'R001',
  ATP_VIOLATION = 'R002',
  OVERSPEED = 'R003',
  COMMUNICATION_LOSS = 'R004',
  EQUIPMENT_FAILURE = 'R005',
  ROUTE_CONFLICT = 'R006',
  PLATFORM_MISALIGNMENT = 'R007',
  DOOR_OBSTRUCTION = 'R008',
  EMERGENCY_ACTIVATED = 'R009',
  SAFETY_SYSTEM_FAULT = 'R010',
}

/**
 * Railway error
 */
export class RailwayError extends Error {
  constructor(
    public code: RailwayErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'RailwayError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core types
  Position,
  TrackLocation,
  Movement,

  // Train
  Train,
  TrainStatus,
  SafetyStatus,

  // Signaling
  Signal,
  SignalAspect,
  Route,
  MovementAuthority,
  SpeedProfile,
  GradientProfile,

  // Train Control
  ETCSLevel,
  ETCSMode,
  ETCSStatus,
  BrakingCurve,
  CurvePoint,
  CBTCStatus,

  // Station/Platform
  Station,
  Platform,
  PSDStatus,
  PlatformEvent,

  // Dynamics
  BrakingParameters,
  BrakingDistance,
  HeadwayCalculation,
  TractionParameters,
  ResistanceForces,

  // Communication
  GSMRCall,
  RadioNetworkStatus,

  // Passenger Info
  DepartureInfo,
  DeparturesBoard,
  ServiceAlert,
  PassengerCount,

  // Safety
  SignalValidation,
  SafetyIntegrityLevel,
  SafetyCheck,
  EmergencyEvent,

  // Utility
  TimeRange,
  DistanceRange,
};

export { RAILWAY_CONSTANTS, RailwayErrorCode, RailwayError };
