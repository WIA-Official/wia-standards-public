/**
 * WIA-AUTO-011: Intelligent Transportation System - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Mobility Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Geographic Types
// ============================================================================

/**
 * Geographic coordinate
 */
export interface GeoCoordinate {
  lat: number;
  lon: number;
  altitude?: number;
}

/**
 * Geographic area/region
 */
export interface GeoRegion {
  center: GeoCoordinate;
  radius?: number; // meters
  bounds?: {
    north: number;
    south: number;
    east: number;
    west: number;
  };
}

/**
 * Road location identifier
 */
export interface RoadLocation {
  roadId: string;
  direction?: 'NB' | 'SB' | 'EB' | 'WB' | 'BOTH';
  milepost?: number;
  laneId?: number;
  coordinate?: GeoCoordinate;
  description?: string;
}

// ============================================================================
// Traffic Flow Types
// ============================================================================

/**
 * Traffic flow parameters
 */
export interface TrafficFlow {
  /** Flow rate in vehicles per hour */
  flow: number;

  /** Density in vehicles per kilometer */
  density: number;

  /** Average speed in km/h */
  speed: number;

  /** Occupancy percentage (0-100) */
  occupancy?: number;

  /** Number of lanes */
  lanes?: number;

  /** Timestamp of measurement */
  timestamp: Date;
}

/**
 * Traffic flow calculation parameters
 */
export interface TrafficFlowParams {
  /** Density in vehicles per kilometer */
  density: number;

  /** Speed in km/h (optional, will calculate if not provided) */
  speed?: number;

  /** Free-flow speed in km/h (for calculations) */
  freeFlowSpeed?: number;

  /** Jam density in vehicles/km (for calculations) */
  jamDensity?: number;
}

/**
 * Traffic flow result
 */
export interface TrafficFlowResult {
  /** Flow rate in vehicles per hour */
  flow: number;

  /** Density in vehicles per kilometer */
  density: number;

  /** Speed in km/h */
  speed: number;

  /** Level of Service (A-F) */
  levelOfService: LevelOfService;

  /** Critical density */
  criticalDensity: number;

  /** Maximum flow (capacity) */
  maxFlow: number;

  /** Flow regime */
  regime: 'free-flow' | 'stable' | 'unstable' | 'congested';
}

/**
 * Level of Service classification
 */
export type LevelOfService = 'A' | 'B' | 'C' | 'D' | 'E' | 'F';

/**
 * LOS criteria
 */
export interface LOSCriteria {
  levelOfService: LevelOfService;
  densityRange: [number, number]; // veh/km/lane
  speedRatio: [number, number]; // % of free-flow
  description: string;
}

// ============================================================================
// Traffic Signal Types
// ============================================================================

/**
 * Signal phase definition
 */
export interface SignalPhase {
  /** Phase identifier */
  phaseId: number;

  /** Green time in seconds */
  greenTime: number;

  /** Yellow time in seconds */
  yellowTime: number;

  /** Red time in seconds */
  redTime: number;

  /** Minimum green time */
  minGreen: number;

  /** Maximum green time */
  maxGreen: number;

  /** Associated movements */
  movements?: string[];
}

/**
 * Signal timing plan
 */
export interface SignalTiming {
  /** Intersection identifier */
  intersectionId: string;

  /** Cycle length in seconds */
  cycleLength: number;

  /** Offset from master clock (seconds) */
  offset: number;

  /** Coordination mode */
  coordination: 'free' | 'coordinated' | 'adaptive';

  /** All phases */
  phases: SignalPhase[];

  /** Total lost time per cycle */
  lostTime?: number;

  /** Effective start time */
  effectiveTime?: Date;
}

/**
 * Traffic approach for signal optimization
 */
export interface TrafficApproach {
  /** Approach identifier */
  approachId: string;

  /** Traffic flow in vehicles per hour */
  flow: number;

  /** Saturation flow in vehicles per hour */
  saturationFlow: number;

  /** Lost time in seconds */
  lostTime: number;

  /** Critical lane group */
  isCritical?: boolean;
}

/**
 * Signal optimization parameters
 */
export interface SignalOptimizationParams {
  /** List of approaches */
  approaches: TrafficApproach[];

  /** Desired cycle length ('auto' or specific value in seconds) */
  cycleLength: number | 'auto';

  /** Minimum cycle length */
  minCycleLength?: number;

  /** Maximum cycle length */
  maxCycleLength?: number;

  /** Optimization objective */
  objective?: 'minimize-delay' | 'maximize-throughput' | 'balance';
}

/**
 * Signal optimization result
 */
export interface SignalOptimizationResult {
  /** Optimal cycle length */
  cycleLength: number;

  /** Phase timings */
  phases: SignalPhase[];

  /** Total delay (vehicle-seconds) */
  totalDelay: number;

  /** Average delay per vehicle (seconds) */
  averageDelay: number;

  /** Level of Service */
  levelOfService: LevelOfService;

  /** Volume to capacity ratio */
  volumeCapacityRatio: number;

  /** Recommendations */
  recommendations: string[];
}

/**
 * Signal status
 */
export interface SignalStatus {
  /** Intersection identifier */
  intersectionId: string;

  /** Current phase */
  currentPhase: number;

  /** Time remaining in current phase (seconds) */
  timeInPhase: number;

  /** Next phase */
  nextPhase: number;

  /** Operating mode */
  mode: 'manual' | 'free' | 'coordinated' | 'adaptive' | 'preemption' | 'flash';

  /** Timestamp */
  timestamp: Date;

  /** Health status */
  health: 'ok' | 'warning' | 'error' | 'offline';
}

// ============================================================================
// Congestion Types
// ============================================================================

/**
 * Congestion level
 */
export type CongestionLevel = 'none' | 'light' | 'moderate' | 'heavy' | 'severe';

/**
 * Congestion prediction parameters
 */
export interface CongestionPredictionParams {
  /** Location to predict */
  location: GeoCoordinate | RoadLocation;

  /** Prediction time window in minutes */
  timeWindow: number;

  /** Include historical data */
  historicalData?: boolean;

  /** Current weather conditions */
  weather?: WeatherCondition;

  /** Special events */
  events?: SpecialEvent[];
}

/**
 * Congestion prediction result
 */
export interface CongestionPrediction {
  /** Predicted time */
  time: Date;

  /** Predicted speed (km/h) */
  speed: number;

  /** Predicted flow (veh/h) */
  flow: number;

  /** Congestion level */
  congestionLevel: CongestionLevel;

  /** Congestion index (%) */
  congestionIndex: number;

  /** Estimated delay (minutes) */
  estimatedDelay: number;

  /** Prediction confidence (0-1) */
  confidence: number;

  /** Contributing factors */
  factors?: string[];
}

/**
 * Weather condition
 */
export interface WeatherCondition {
  condition: 'clear' | 'rain' | 'snow' | 'fog' | 'wind' | 'ice';
  temperature: number; // Celsius
  visibility: number; // meters
  precipitation?: number; // mm/h
}

/**
 * Special event
 */
export interface SpecialEvent {
  eventId: string;
  type: 'sports' | 'concert' | 'conference' | 'construction' | 'other';
  location: GeoCoordinate;
  startTime: Date;
  endTime: Date;
  expectedAttendance?: number;
  trafficImpact: 'low' | 'medium' | 'high';
}

// ============================================================================
// Vehicle Detection Types
// ============================================================================

/**
 * Vehicle classification
 */
export type VehicleClass = 'motorcycle' | 'car' | 'van' | 'truck' | 'semi' | 'bus' | 'other';

/**
 * Detected vehicle
 */
export interface DetectedVehicle {
  /** Vehicle identifier (if trackable) */
  vehicleId?: string;

  /** Vehicle classification */
  class: VehicleClass;

  /** Speed in km/h */
  speed: number;

  /** Length in meters */
  length?: number;

  /** Lane number */
  lane: number;

  /** Detection timestamp */
  timestamp: Date;

  /** Confidence (0-1) */
  confidence: number;
}

/**
 * Vehicle detection parameters
 */
export interface VehicleDetectionParams {
  /** Detection location */
  location: RoadLocation;

  /** Detection technology */
  technology: 'loop' | 'radar' | 'lidar' | 'video' | 'acoustic';

  /** Detection zone length (meters) */
  zoneLength: number;

  /** Classification enabled */
  classification?: boolean;

  /** Speed measurement enabled */
  speedMeasurement?: boolean;
}

/**
 * Vehicle detection result
 */
export interface VehicleDetectionResult {
  /** Total vehicle count */
  count: number;

  /** Vehicles by class */
  byClass: Record<VehicleClass, number>;

  /** Average speed */
  averageSpeed: number;

  /** Occupancy percentage */
  occupancy: number;

  /** Detection period */
  period: {
    start: Date;
    end: Date;
  };

  /** Individual detections */
  detections?: DetectedVehicle[];
}

// ============================================================================
// Emergency Vehicle Preemption Types
// ============================================================================

/**
 * Emergency vehicle type
 */
export type EmergencyVehicleType = 'fire' | 'ambulance' | 'police' | 'other';

/**
 * Emergency vehicle
 */
export interface EmergencyVehicle {
  /** Vehicle identifier */
  vehicleId: string;

  /** Vehicle type */
  type: EmergencyVehicleType;

  /** Current location */
  location: GeoCoordinate;

  /** Heading (degrees, 0=North) */
  heading: number;

  /** Speed (km/h) */
  speed: number;

  /** Destination */
  destination?: GeoCoordinate;

  /** Route waypoints */
  route?: string[];

  /** Priority level (1-5, 5=highest) */
  priority: number;

  /** Timestamp */
  timestamp: Date;
}

/**
 * Preemption request
 */
export interface PreemptionRequest {
  /** Request identifier */
  requestId: string;

  /** Emergency vehicle */
  vehicle: EmergencyVehicle;

  /** Affected intersections */
  intersections: string[];

  /** Requested path */
  path: string[];

  /** Estimated time of arrival at each intersection */
  eta: Date[];

  /** Request status */
  status: 'pending' | 'active' | 'completed' | 'cancelled';
}

/**
 * Preemption response
 */
export interface PreemptionResponse {
  /** Request identifier */
  requestId: string;

  /** Granted or denied */
  granted: boolean;

  /** Affected intersections and their timings */
  intersectionTimings: Array<{
    intersectionId: string;
    clearanceTime: number;
    greenTime: number;
    recoveryTime: number;
  }>;

  /** Estimated time savings (seconds) */
  timeSavings: number;

  /** Conflicts detected */
  conflicts?: string[];

  /** Timestamp */
  timestamp: Date;
}

// ============================================================================
// Incident Management Types
// ============================================================================

/**
 * Incident type
 */
export type IncidentType =
  | 'accident'
  | 'breakdown'
  | 'debris'
  | 'construction'
  | 'weather'
  | 'special-event'
  | 'other';

/**
 * Incident severity
 */
export type IncidentSeverity = 'minor' | 'moderate' | 'major' | 'critical';

/**
 * Traffic incident
 */
export interface TrafficIncident {
  /** Incident identifier */
  incidentId: string;

  /** Incident type */
  type: IncidentType;

  /** Severity level */
  severity: IncidentSeverity;

  /** Location */
  location: RoadLocation;

  /** Start time */
  startTime: Date;

  /** Estimated end time */
  estimatedEndTime?: Date;

  /** Actual end time (if resolved) */
  endTime?: Date;

  /** Impact */
  impact: {
    lanesBlocked: number;
    totalLanes: number;
    estimatedDuration: number; // minutes
    delay: number; // minutes
  };

  /** Description */
  description: string;

  /** Response units */
  responseUnits?: string[];

  /** Status */
  status: 'active' | 'clearing' | 'cleared';
}

// ============================================================================
// Toll Collection Types
// ============================================================================

/**
 * Toll transaction
 */
export interface TollTransaction {
  /** Transaction identifier */
  transactionId: string;

  /** Vehicle identifier (license plate, RFID, etc.) */
  vehicleId: string;

  /** Vehicle class */
  vehicleClass: VehicleClass;

  /** Entry point */
  entryPoint: RoadLocation;

  /** Exit point */
  exitPoint: RoadLocation;

  /** Entry time */
  entryTime: Date;

  /** Exit time */
  exitTime: Date;

  /** Distance traveled (km) */
  distance: number;

  /** Base toll amount */
  baseToll: number;

  /** Congestion surcharge */
  congestionSurcharge: number;

  /** Total toll amount */
  totalToll: number;

  /** Payment method */
  paymentMethod: 'RFID' | 'ANPR' | 'cash' | 'mobile';

  /** Transaction status */
  status: 'success' | 'pending' | 'failed' | 'violation';
}

/**
 * Dynamic toll pricing parameters
 */
export interface DynamicTollPricing {
  /** Base toll rate */
  baseRate: number;

  /** Current demand level (0-1) */
  demandLevel: number;

  /** Road capacity */
  capacity: number;

  /** Congestion index */
  congestionIndex: number;

  /** Time of day factor */
  timeOfDayFactor: number;

  /** Vehicle class multiplier */
  vehicleClassMultiplier: number;
}

/**
 * Toll calculation result
 */
export interface TollCalculation {
  /** Base toll */
  baseToll: number;

  /** Surcharges */
  surcharges: {
    congestion: number;
    timeOfDay: number;
    vehicleClass: number;
  };

  /** Total toll */
  totalToll: number;

  /** Calculation breakdown */
  breakdown: string[];
}

// ============================================================================
// Traveler Information Types
// ============================================================================

/**
 * Route segment
 */
export interface RouteSegment {
  /** Segment identifier */
  segmentId: string;

  /** Road name */
  roadName: string;

  /** Segment start */
  start: GeoCoordinate;

  /** Segment end */
  end: GeoCoordinate;

  /** Distance (km) */
  distance: number;

  /** Current speed (km/h) */
  currentSpeed: number;

  /** Free-flow speed (km/h) */
  freeFlowSpeed: number;

  /** Travel time (minutes) */
  travelTime: number;

  /** Congestion level */
  congestionLevel: CongestionLevel;
}

/**
 * Route information
 */
export interface RouteInfo {
  /** Route identifier */
  routeId: string;

  /** Route name/description */
  name: string;

  /** Origin */
  origin: GeoCoordinate;

  /** Destination */
  destination: GeoCoordinate;

  /** Route segments */
  segments: RouteSegment[];

  /** Total distance (km) */
  totalDistance: number;

  /** Current travel time (minutes) */
  currentTravelTime: number;

  /** Free-flow travel time (minutes) */
  freeFlowTravelTime: number;

  /** Congestion index */
  congestionIndex: number;

  /** Incidents on route */
  incidents?: TrafficIncident[];

  /** Alternative routes available */
  hasAlternatives: boolean;
}

/**
 * Travel time prediction
 */
export interface TravelTimePrediction {
  /** Route identifier */
  routeId: string;

  /** Departure time */
  departureTime: Date;

  /** Predicted arrival time */
  arrivalTime: Date;

  /** Predicted travel time (minutes) */
  travelTime: number;

  /** Confidence interval */
  confidence: {
    low: number; // minutes
    high: number; // minutes
  };

  /** Prediction confidence (0-1) */
  accuracy: number;
}

// ============================================================================
// System Management Types
// ============================================================================

/**
 * ITS Manager configuration
 */
export interface ITSManagerConfig {
  /** Region/area name */
  region: string;

  /** Update interval (milliseconds) */
  updateInterval: number;

  /** Enable real-time monitoring */
  enableMonitoring?: boolean;

  /** Enable signal optimization */
  enableSignalOptimization?: boolean;

  /** Enable incident detection */
  enableIncidentDetection?: boolean;

  /** API endpoints */
  endpoints?: {
    traffic?: string;
    signals?: string;
    incidents?: string;
  };
}

/**
 * Traffic conditions summary
 */
export interface TrafficConditions {
  /** Region identifier */
  region: string;

  /** Average speed (km/h) */
  averageSpeed: number;

  /** Overall congestion level */
  congestionLevel: CongestionLevel;

  /** Number of active incidents */
  activeIncidents: number;

  /** Total monitored segments */
  totalSegments: number;

  /** Segments by LOS */
  segmentsByLOS: Record<LevelOfService, number>;

  /** Last update time */
  lastUpdate: Date;
}

/**
 * Network optimization result
 */
export interface NetworkOptimization {
  /** Number of signals optimized */
  signalsOptimized: number;

  /** Estimated improvement (%) */
  improvement: number;

  /** Total delay reduction (vehicle-hours) */
  delayReduction: number;

  /** Emissions reduction (kg CO2) */
  emissionsReduction?: number;

  /** Optimization timestamp */
  timestamp: Date;

  /** Recommendations */
  recommendations: string[];
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * ITS physical constants and typical values
 */
export const ITS_CONSTANTS = {
  /** Typical free-flow speed on highways (km/h) */
  HIGHWAY_FREE_FLOW_SPEED: 100,

  /** Typical free-flow speed on arterials (km/h) */
  ARTERIAL_FREE_FLOW_SPEED: 60,

  /** Typical jam density (vehicles/km/lane) */
  JAM_DENSITY: 160,

  /** Typical saturation flow rate (vehicles/hour/lane) */
  SATURATION_FLOW: 1900,

  /** Average vehicle length (meters) */
  AVERAGE_VEHICLE_LENGTH: 5.5,

  /** Minimum headway (seconds) */
  MINIMUM_HEADWAY: 1.5,

  /** Driver reaction time (seconds) */
  REACTION_TIME: 1.0,

  /** Typical yellow time (seconds) */
  YELLOW_TIME: 4,

  /** Minimum green time (seconds) */
  MIN_GREEN_TIME: 7,

  /** Maximum green time (seconds) */
  MAX_GREEN_TIME: 120,

  /** Minimum cycle length (seconds) */
  MIN_CYCLE_LENGTH: 40,

  /** Maximum cycle length (seconds) */
  MAX_CYCLE_LENGTH: 180,

  /** Emergency vehicle detection range (meters) */
  PREEMPTION_DETECTION_RANGE: 800,
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
 * Time series data point
 */
export interface TimeSeriesPoint<T> {
  timestamp: Date;
  value: T;
}

/**
 * Time series data
 */
export interface TimeSeries<T> {
  data: TimeSeriesPoint<T>[];
  interval: number; // seconds
  aggregation?: 'raw' | 'average' | 'sum' | 'max' | 'min';
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-AUTO-011 error codes
 */
export enum ITSErrorCode {
  INVALID_PARAMETERS = 'ITS001',
  LOCATION_NOT_FOUND = 'ITS002',
  INSUFFICIENT_DATA = 'ITS003',
  SIGNAL_TIMEOUT = 'ITS004',
  OPTIMIZATION_FAILED = 'ITS005',
  PREDICTION_ERROR = 'ITS006',
  DETECTION_FAILURE = 'ITS007',
  COMMUNICATION_ERROR = 'ITS008',
  SYSTEM_OVERLOAD = 'ITS009',
  UNAUTHORIZED = 'ITS010',
}

/**
 * ITS error
 */
export class ITSError extends Error {
  constructor(
    public code: ITSErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'ITSError';
  }
}

// ============================================================================
// Export All Types
// ============================================================================

export type {
  GeoCoordinate,
  GeoRegion,
  RoadLocation,
  TrafficFlow,
  TrafficFlowParams,
  TrafficFlowResult,
  LOSCriteria,
  SignalPhase,
  SignalTiming,
  TrafficApproach,
  SignalOptimizationParams,
  SignalOptimizationResult,
  SignalStatus,
  CongestionPredictionParams,
  CongestionPrediction,
  WeatherCondition,
  SpecialEvent,
  DetectedVehicle,
  VehicleDetectionParams,
  VehicleDetectionResult,
  EmergencyVehicle,
  PreemptionRequest,
  PreemptionResponse,
  TrafficIncident,
  TollTransaction,
  DynamicTollPricing,
  TollCalculation,
  RouteSegment,
  RouteInfo,
  TravelTimePrediction,
  ITSManagerConfig,
  TrafficConditions,
  NetworkOptimization,
  TimeSeriesPoint,
  TimeSeries,
};

export { ITS_CONSTANTS, ITSErrorCode, ITSError };
