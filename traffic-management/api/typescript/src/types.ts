/**
 * WIA-AUTO-012: Traffic Management - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Automotive & Mobility Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Traffic Flow Types
// ============================================================================

/**
 * Geographic location
 */
export interface Location {
  roadId: string;
  milepost?: number;
  latitude: number;
  longitude: number;
  direction?: 'northbound' | 'southbound' | 'eastbound' | 'westbound';
}

/**
 * Traffic flow measurements
 */
export interface TrafficFlow {
  /** Flow rate in vehicles per hour */
  flow: number;

  /** Average speed in km/h */
  speed: number;

  /** Density in vehicles per km */
  density: number;

  /** Lane occupancy (0-1) */
  occupancy?: number;

  /** Number of lanes */
  laneCount: number;

  /** Timestamp of measurement */
  timestamp: Date;

  /** Location of measurement */
  location: Location;

  /** Data quality indicator */
  quality?: 'excellent' | 'good' | 'fair' | 'poor';

  /** Data source */
  source?: 'loop-detector' | 'radar' | 'camera' | 'probe-vehicle' | 'manual';
}

/**
 * Traffic flow parameters
 */
export interface FlowParameters {
  /** Free-flow speed in km/h */
  freeFlowSpeed: number;

  /** Jam density in vehicles/km */
  jamDensity: number;

  /** Critical density in vehicles/km */
  criticalDensity?: number;

  /** Capacity in vehicles/h/lane */
  capacity?: number;

  /** Saturation flow in vehicles/h/lane */
  saturationFlow?: number;
}

/**
 * Traffic regime classification
 */
export type TrafficRegime = 'free-flow' | 'capacity' | 'congested' | 'jammed';

/**
 * Level of Service (LOS) classification
 */
export type LevelOfService = 'A' | 'B' | 'C' | 'D' | 'E' | 'F';

/**
 * Traffic flow analysis result
 */
export interface FlowAnalysis {
  /** Current flow rate */
  flow: number;

  /** Current speed */
  speed: number;

  /** Current density */
  density: number;

  /** Traffic regime */
  regime: TrafficRegime;

  /** Level of service */
  levelOfService: LevelOfService;

  /** Capacity of roadway */
  capacity: number;

  /** Volume to capacity ratio */
  volumeToCapacity: number;

  /** Is congested? */
  isCongested: boolean;
}

// ============================================================================
// Signal Timing Types
// ============================================================================

/**
 * Traffic signal phase
 */
export interface SignalPhase {
  /** Phase identifier */
  phaseId: number;

  /** Phase name/description */
  name: string;

  /** Minimum green time (seconds) */
  minGreen: number;

  /** Maximum green time (seconds) */
  maxGreen: number;

  /** Actual green time (seconds) */
  green: number;

  /** Yellow interval (seconds) */
  yellow: number;

  /** All-red clearance (seconds) */
  allRed: number;

  /** Pedestrian walk time (seconds) */
  pedestrianWalk?: number;

  /** Pedestrian clearance time (seconds) */
  pedestrianClearance?: number;

  /** Traffic volume for this phase (veh/h) */
  volume?: number;

  /** Saturation flow (veh/h) */
  saturationFlow?: number;
}

/**
 * Traffic signal timing plan
 */
export interface SignalTimingPlan {
  /** Intersection identifier */
  intersectionId: string;

  /** Plan identifier */
  planId: string;

  /** Cycle time (seconds) */
  cycleTime: number;

  /** Offset from reference point (seconds) */
  offset: number;

  /** All phases in the plan */
  phases: SignalPhase[];

  /** Coordination settings */
  coordination?: {
    system: 'fixed-time' | 'actuated' | 'adaptive' | 'SCOOT' | 'SCATS';
    bandwidth?: number;
    progressionSpeed?: number;
  };

  /** Total lost time per cycle */
  lostTime: number;

  /** Plan activation schedule */
  schedule?: {
    startTime: string;
    endTime: string;
    daysOfWeek: number[];
  };
}

/**
 * Signal optimization request
 */
export interface SignalOptimizationRequest {
  /** Phases to optimize */
  phases: Array<{
    name: string;
    volume: number;
    saturationFlow: number;
  }>;

  /** Lost time per phase (seconds) */
  lostTime: number;

  /** Target average delay (seconds) */
  targetDelay?: number;

  /** Minimum cycle time constraint (seconds) */
  minCycleTime?: number;

  /** Maximum cycle time constraint (seconds) */
  maxCycleTime?: number;
}

/**
 * Signal optimization result
 */
export interface SignalOptimizationResult {
  /** Optimal cycle time (seconds) */
  cycleTime: number;

  /** Green times for each phase (seconds) */
  greenTimes: number[];

  /** Average delay for each phase (seconds) */
  delays: number[];

  /** Overall average delay (seconds) */
  averageDelay: number;

  /** Level of service */
  levelOfService: LevelOfService;

  /** Volume to capacity ratio */
  volumeToCapacity: number;

  /** Critical flow ratio sum */
  criticalFlowRatio: number;

  /** Is over-saturated? */
  isOverSaturated: boolean;

  /** Recommendations */
  recommendations: string[];
}

// ============================================================================
// Congestion Detection Types
// ============================================================================

/**
 * Congestion severity levels
 */
export type CongestionSeverity = 'none' | 'mild' | 'moderate' | 'severe' | 'gridlock';

/**
 * Congestion detection request
 */
export interface CongestionDetectionRequest {
  /** Current speed (km/h) */
  speed: number;

  /** Current density (veh/km) */
  density: number;

  /** Free-flow speed (km/h) */
  freeFlowSpeed: number;

  /** Jam density (veh/km) */
  jamDensity?: number;

  /** Historical average speed (km/h) */
  historicalSpeed?: number;
}

/**
 * Congestion detection result
 */
export interface CongestionDetectionResult {
  /** Is currently congested? */
  isCongested: boolean;

  /** Severity level */
  severity: CongestionSeverity;

  /** Travel time index */
  travelTimeIndex: number;

  /** Speed reduction percentage */
  speedReduction: number;

  /** Estimated delay (seconds per vehicle) */
  estimatedDelay: number;

  /** Recommended actions */
  recommendedActions: string[];

  /** Congestion score (0-100) */
  congestionScore: number;
}

/**
 * Congestion metrics
 */
export interface CongestionMetrics {
  /** Total delay (vehicle-hours) */
  totalDelay: number;

  /** Vehicle-hours of delay */
  vehicleHoursDelay: number;

  /** Queue length (meters) */
  queueLength: number;

  /** Number of vehicles in queue */
  vehiclesInQueue: number;

  /** Congestion duration (seconds) */
  duration: number;

  /** Affected roadway length (km) */
  affectedLength: number;
}

// ============================================================================
// Incident Detection Types
// ============================================================================

/**
 * Incident types
 */
export type IncidentType =
  | 'collision'
  | 'breakdown'
  | 'debris'
  | 'weather'
  | 'construction'
  | 'special-event'
  | 'other';

/**
 * Incident severity
 */
export type IncidentSeverity = 'minor' | 'moderate' | 'major' | 'critical';

/**
 * Incident report
 */
export interface Incident {
  /** Unique incident identifier */
  incidentId: string;

  /** Incident timestamp */
  timestamp: Date;

  /** Incident location */
  location: Location & {
    lane?: number;
    direction?: string;
  };

  /** Type of incident */
  type: IncidentType;

  /** Severity level */
  severity: IncidentSeverity;

  /** Number of lanes blocked */
  lanesBlocked: number;

  /** Capacity reduction factor (0-1) */
  capacityReduction: number;

  /** Estimated clearance time */
  estimatedClearance?: Date;

  /** Impact assessment */
  impact?: {
    queueLength: number;
    delay: number;
    affectedVehicles: number;
    estimatedDuration: number;
  };

  /** Response information */
  response?: {
    detectionTime: Date;
    responseTime: Date;
    dispatchedUnits: string[];
    status: 'detected' | 'responding' | 'on-scene' | 'clearing' | 'cleared';
  };

  /** Additional description */
  description?: string;

  /** Is verified? */
  verified: boolean;
}

/**
 * Incident detection parameters
 */
export interface IncidentDetectionParams {
  /** Current speed (km/h) */
  speed: number;

  /** Previous speed (km/h) */
  previousSpeed: number;

  /** Current density (veh/km) */
  density: number;

  /** Previous density (veh/km) */
  previousDensity: number;

  /** Current occupancy (0-1) */
  occupancy?: number;

  /** Speed threshold for detection */
  speedThreshold?: number;

  /** Density threshold for detection */
  densityThreshold?: number;
}

/**
 * Incident detection result
 */
export interface IncidentDetectionResult {
  /** Is incident detected? */
  incidentDetected: boolean;

  /** Confidence level (0-1) */
  confidence: number;

  /** Detection algorithm used */
  algorithm: 'california' | 'mcmaster' | 'ml-based' | 'manual';

  /** Estimated incident type */
  estimatedType?: IncidentType;

  /** Estimated severity */
  estimatedSeverity?: IncidentSeverity;

  /** Detection timestamp */
  detectionTime: Date;

  /** Additional details */
  details?: string;
}

// ============================================================================
// Traffic Prediction Types
// ============================================================================

/**
 * Traffic prediction request
 */
export interface TrafficPredictionRequest {
  /** Location identifier */
  locationId: string;

  /** Prediction horizon (minutes) */
  horizon: number;

  /** Historical flow data */
  historicalData?: number[];

  /** External factors */
  externalFactors?: {
    weather?: 'clear' | 'rain' | 'snow' | 'fog';
    temperature?: number;
    dayOfWeek?: 'monday' | 'tuesday' | 'wednesday' | 'thursday' | 'friday' | 'saturday' | 'sunday';
    hour?: number;
    isHoliday?: boolean;
    specialEvent?: string;
  };

  /** Prediction model to use */
  model?: 'historical-average' | 'exponential-smoothing' | 'arima' | 'neural-network' | 'kalman-filter';
}

/**
 * Single prediction point
 */
export interface PredictionPoint {
  /** Prediction timestamp */
  timestamp: Date;

  /** Predicted flow (veh/h) */
  flow: number;

  /** Predicted speed (km/h) */
  speed: number;

  /** Predicted density (veh/km) */
  density?: number;

  /** Confidence level (0-1) */
  confidence: number;

  /** Prediction interval (lower bound) */
  lowerBound?: number;

  /** Prediction interval (upper bound) */
  upperBound?: number;
}

/**
 * Traffic prediction result
 */
export interface TrafficPredictionResult {
  /** Unique prediction identifier */
  predictionId: string;

  /** Prediction generation timestamp */
  timestamp: Date;

  /** Location */
  locationId: string;

  /** Prediction horizon (minutes) */
  horizon: number;

  /** Array of predictions */
  predictions: PredictionPoint[];

  /** Model used */
  model: string;

  /** Model accuracy metrics */
  accuracy: {
    mape: number;  // Mean Absolute Percentage Error
    rmse: number;  // Root Mean Square Error
    mae?: number;  // Mean Absolute Error
  };

  /** Model metadata */
  metadata?: {
    trainingSamples: number;
    lastUpdated: Date;
    version: string;
  };
}

// ============================================================================
// Intersection Analysis Types
// ============================================================================

/**
 * Intersection approach
 */
export interface IntersectionApproach {
  /** Approach name */
  name: string;

  /** Traffic volume (veh/h) */
  volume: number;

  /** Saturation flow (veh/h) */
  saturationFlow: number;

  /** Green time (seconds) */
  greenTime?: number;

  /** Phase identifier */
  phase: string | number;

  /** Number of lanes */
  lanes: number;

  /** Movement types */
  movements?: Array<'through' | 'left' | 'right'>;
}

/**
 * Intersection analysis result
 */
export interface IntersectionAnalysis {
  /** Intersection identifier */
  intersectionId: string;

  /** Analysis timestamp */
  timestamp: Date;

  /** Cycle time (seconds) */
  cycleTime: number;

  /** Approaches analyzed */
  approaches: IntersectionApproach[];

  /** Overall level of service */
  levelOfService: LevelOfService;

  /** Average delay (seconds) */
  averageDelay: number;

  /** Maximum approach delay (seconds) */
  maxDelay: number;

  /** Total capacity (veh/h) */
  capacity: number;

  /** Volume to capacity ratio */
  volumeToCapacity: number;

  /** Queue lengths by approach (vehicles) */
  queueLengths: number[];

  /** Performance metrics */
  metrics: {
    throughput: number;
    efficiency: number;
    reliability: number;
  };

  /** Improvement recommendations */
  recommendations: string[];
}

// ============================================================================
// Ramp Metering Types
// ============================================================================

/**
 * Ramp metering parameters
 */
export interface RampMeteringParams {
  /** Mainline flow (veh/h) */
  mainlineFlow: number;

  /** Mainline capacity (veh/h) */
  mainlineCapacity: number;

  /** On-ramp demand (veh/h) */
  rampDemand: number;

  /** Current occupancy (0-1) */
  occupancy: number;

  /** Target occupancy (0-1) */
  targetOccupancy?: number;

  /** Algorithm */
  algorithm?: 'fixed-rate' | 'alinea' | 'predictive';
}

/**
 * Ramp metering result
 */
export interface RampMeteringResult {
  /** Metering rate (veh/h) */
  meteringRate: number;

  /** Cycle time (seconds) */
  cycleTime: number;

  /** Green time (seconds) */
  greenTime: number;

  /** Queue on ramp (vehicles) */
  rampQueue: number;

  /** Storage capacity utilized (%) */
  storageUtilization: number;

  /** Mainline benefit (veh-h saved) */
  mainlineBenefit: number;

  /** Ramp delay (veh-h) */
  rampDelay: number;

  /** Overall benefit (veh-h) */
  netBenefit: number;
}

// ============================================================================
// Physical Constants and Defaults
// ============================================================================

/**
 * Traffic management constants
 */
export const TRAFFIC_CONSTANTS = {
  /** Typical free-flow speed for highways (km/h) */
  HIGHWAY_FREE_FLOW_SPEED: 100,

  /** Typical free-flow speed for urban roads (km/h) */
  URBAN_FREE_FLOW_SPEED: 60,

  /** Typical jam density (veh/km) */
  JAM_DENSITY: 180,

  /** Typical critical density (veh/km) */
  CRITICAL_DENSITY: 30,

  /** Typical saturation flow (veh/h/lane) */
  SATURATION_FLOW: 1900,

  /** Typical lost time per phase (seconds) */
  LOST_TIME_PER_PHASE: 4,

  /** Minimum green time (seconds) */
  MIN_GREEN_TIME: 7,

  /** Maximum green time (seconds) */
  MAX_GREEN_TIME: 90,

  /** Typical yellow interval (seconds) */
  YELLOW_INTERVAL: 4,

  /** Typical all-red clearance (seconds) */
  ALL_RED_CLEARANCE: 2,

  /** Pedestrian walking speed (m/s) */
  PEDESTRIAN_SPEED: 1.2,

  /** Minimum pedestrian crossing time (seconds) */
  MIN_PEDESTRIAN_CROSSING: 7,

  /** Speed threshold for congestion (fraction of free-flow) */
  CONGESTION_SPEED_THRESHOLD: 0.3,

  /** Density threshold for congestion (fraction of jam density) */
  CONGESTION_DENSITY_THRESHOLD: 0.7,

  /** Target occupancy for ramp metering */
  TARGET_OCCUPANCY: 0.28,

  /** ALINEA regulator parameter */
  ALINEA_KR: 70,
} as const;

/**
 * Level of Service delay thresholds (seconds per vehicle)
 */
export const LOS_THRESHOLDS = {
  A: 10,
  B: 20,
  C: 35,
  D: 55,
  E: 80,
  F: Infinity,
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
export interface TimeSeriesPoint {
  timestamp: Date;
  value: number;
  quality?: 'good' | 'fair' | 'poor';
}

/**
 * Time series data
 */
export interface TimeSeries {
  locationId: string;
  metric: 'flow' | 'speed' | 'density' | 'occupancy';
  data: TimeSeriesPoint[];
  aggregation?: 'raw' | '1min' | '5min' | '15min' | '1hour';
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-AUTO-012 error codes
 */
export enum TrafficErrorCode {
  INVALID_PARAMETERS = 'TM001',
  OVER_SATURATED = 'TM002',
  INSUFFICIENT_DATA = 'TM003',
  CALIBRATION_REQUIRED = 'TM004',
  SIGNAL_CONFLICT = 'TM005',
  PREDICTION_FAILED = 'TM006',
  INTEGRATION_ERROR = 'TM007',
  DATA_QUALITY_LOW = 'TM008',
}

/**
 * Traffic management error
 */
export class TrafficManagementError extends Error {
  constructor(
    public code: TrafficErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'TrafficManagementError';
  }
}

// ============================================================================
// Export All Types
// ============================================================================

export type {
  Location,
  TrafficFlow,
  FlowParameters,
  FlowAnalysis,
  SignalPhase,
  SignalTimingPlan,
  SignalOptimizationRequest,
  SignalOptimizationResult,
  CongestionDetectionRequest,
  CongestionDetectionResult,
  CongestionMetrics,
  Incident,
  IncidentDetectionParams,
  IncidentDetectionResult,
  TrafficPredictionRequest,
  PredictionPoint,
  TrafficPredictionResult,
  IntersectionApproach,
  IntersectionAnalysis,
  RampMeteringParams,
  RampMeteringResult,
  TimeSeries,
  TimeSeriesPoint,
};

export {
  TRAFFIC_CONSTANTS,
  LOS_THRESHOLDS,
  TrafficErrorCode,
  TrafficManagementError,
};
