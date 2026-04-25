/**
 * WIA-CITY-017: Traffic Simulation Standard - TypeScript Type Definitions
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Basic Types
// ============================================================================

/**
 * ISO 8601 timestamp string
 */
export type Timestamp = string;

/**
 * Geographic coordinates
 */
export interface Coordinates {
  latitude: number;
  longitude: number;
  elevation?: number;
}

/**
 * Location information
 */
export interface Location {
  address?: string;
  district?: string;
  coordinates: Coordinates;
}

// ============================================================================
// Network Types
// ============================================================================

/**
 * Link (road segment) type
 */
export enum LinkType {
  FREEWAY = 'freeway',
  ARTERIAL = 'arterial',
  COLLECTOR = 'collector',
  LOCAL = 'local',
  RAMP = 'ramp',
}

/**
 * Road link
 */
export interface RoadLink {
  linkId: string;
  name: string;
  fromNode: string;
  toNode: string;

  geometry: {
    length: number;                     // m
    lanes: number;
    width: number;                      // m per lane
    coordinates: Coordinates[];         // polyline
  };

  characteristics: {
    linkType: LinkType;
    speedLimit: number;                 // km/h
    capacity: number;                   // veh/h per lane
    freeFlowSpeed: number;              // km/h
    laneCapacity: number;               // veh/h/lane
  };

  restrictions?: {
    noTrucks?: boolean;
    noBuses?: boolean;
    onewayDirection?: 'forward' | 'backward';
    timeRestrictions?: TimeRestriction[];
  };

  metadata?: {
    surfaceType?: 'asphalt' | 'concrete' | 'gravel';
    condition?: 'good' | 'fair' | 'poor';
    lastMaintenance?: Timestamp;
  };
}

/**
 * Traffic state on a link
 */
export interface TrafficState {
  linkId: string;
  timestamp: Timestamp;

  flow: {
    volume: number;                     // veh/h
    speed: number;                      // km/h
    density: number;                    // veh/km
    occupancy: number;                  // % (0-100)
  };

  byLane?: {
    lane: number;
    volume: number;
    speed: number;
    occupancy: number;
  }[];

  quality: {
    los: 'A' | 'B' | 'C' | 'D' | 'E' | 'F';
    vcRatio: number;                    // volume/capacity
  };
}

/**
 * Node (intersection) type
 */
export enum NodeType {
  SIGNALIZED = 'signalized',
  UNSIGNALIZED = 'unsignalized',
  ROUNDABOUT = 'roundabout',
  MERGE = 'merge',
  DIVERGE = 'diverge',
}

/**
 * Traffic node
 */
export interface TrafficNode {
  nodeId: string;
  name: string;
  nodeType: NodeType;
  coordinates: Coordinates;

  connections: {
    incomingLinks: string[];
    outgoingLinks: string[];
  };

  control?: {
    signalId?: string;
    priority?: 'main' | 'minor';
    yieldRules?: YieldRule[];
  };
}

/**
 * Yield rule
 */
export interface YieldRule {
  fromLink: string;
  toLink: string;
  yieldTo: string[];                    // link IDs that have priority
}

/**
 * Time restriction
 */
export interface TimeRestriction {
  type: 'no_parking' | 'bus_only' | 'hov_only' | 'closed';
  startTime: string;                    // HH:MM
  endTime: string;                      // HH:MM
  daysOfWeek: number[];                 // 0=Sunday, 6=Saturday
}

// ============================================================================
// Vehicle Types
// ============================================================================

/**
 * Vehicle type
 */
export enum VehicleType {
  CAR = 'car',
  BUS = 'bus',
  TRUCK = 'truck',
  MOTORCYCLE = 'motorcycle',
  BICYCLE = 'bicycle',
  EMERGENCY = 'emergency',
}

/**
 * Vehicle characteristics
 */
export interface VehicleCharacteristics {
  type: VehicleType;
  length: number;                       // m
  width: number;                        // m
  height?: number;                      // m
  pcu: number;                          // Passenger Car Unit equivalent

  performance: {
    maxSpeed: number;                   // km/h
    maxAcceleration: number;            // m/s²
    maxDeceleration: number;            // m/s²
    comfortableDecel: number;           // m/s²
  };

  driver: {
    desiredSpeed: number;               // km/h
    reactionTime: number;               // s
    aggressiveness: number;             // 0-1 (0=cautious, 1=aggressive)
    compliance: number;                 // 0-1 (0=non-compliant, 1=compliant)
  };
}

/**
 * Vehicle instance
 */
export interface Vehicle {
  vehicleId: string;
  characteristics: VehicleCharacteristics;

  trip: {
    origin: string;                     // node ID
    destination: string;                // node ID
    departureTime: Timestamp;
    arrivalTime?: Timestamp;
    route: string[];                    // link IDs
  };

  currentState: {
    linkId: string;
    laneIndex: number;                  // 0-based
    position: number;                   // m from link start
    speed: number;                      // km/h
    acceleration: number;               // m/s²
    heading: number;                    // degrees (0=North)
  };

  history?: VehicleHistoryPoint[];
}

/**
 * Vehicle history point
 */
export interface VehicleHistoryPoint {
  timestamp: Timestamp;
  linkId: string;
  position: number;
  speed: number;
  acceleration: number;
}

// ============================================================================
// Traffic Flow Models
// ============================================================================

/**
 * Car-following model type
 */
export enum CarFollowingModel {
  WIEDEMANN = 'wiedemann',
  GIPPS = 'gipps',
  IDM = 'idm',                          // Intelligent Driver Model
  KRAUSS = 'krauss',
}

/**
 * Car-following parameters
 */
export interface CarFollowingParameters {
  model: CarFollowingModel;

  // Common parameters
  minGap: number;                       // m
  timeHeadway: number;                  // s

  // IDM specific
  idm?: {
    delta: number;                      // acceleration exponent
    a: number;                          // max acceleration (m/s²)
    b: number;                          // comfortable deceleration (m/s²)
  };

  // Wiedemann specific
  wiedemann?: {
    cc0: number;                        // standstill distance
    cc1: number;                        // headway time
    cc2: number;                        // following variation
  };
}

/**
 * Lane-changing model type
 */
export enum LaneChangingModel {
  MOBIL = 'mobil',
  GIPPS = 'gipps',
  SIMPLE = 'simple',
}

/**
 * Lane-changing parameters
 */
export interface LaneChangingParameters {
  model: LaneChangingModel;

  // MOBIL parameters
  mobil?: {
    politeness: number;                 // 0-1
    threshold: number;                  // m/s²
    safeDeceleration: number;           // m/s²
  };

  // Look-ahead distance
  anticipationDistance: number;         // m

  // Mandatory vs discretionary
  mandatoryDistance: number;            // m (distance before turn)
}

// ============================================================================
// Traffic Signals
// ============================================================================

/**
 * Traffic signal phase
 */
export interface SignalPhase {
  phaseId: number;
  name?: string;

  movements: Movement[];

  timing: {
    greenTime: number;                  // s
    yellowTime: number;                 // s
    allRedTime: number;                 // s
    minGreenTime?: number;              // s (for actuated)
    maxGreenTime?: number;              // s (for actuated)
  };

  detectors?: string[];                 // detector IDs
}

/**
 * Movement (from-to at intersection)
 */
export interface Movement {
  fromLink: string;
  toLink: string;
  type: 'through' | 'left' | 'right' | 'uturn';
  protected: boolean;                   // true if protected, false if permitted
}

/**
 * Traffic signal timing plan
 */
export interface SignalTimingPlan {
  planId: string;
  name: string;
  signalId: string;

  cycleLength: number;                  // s
  offset: number;                       // s

  phases: SignalPhase[];

  coordination?: {
    referencePhase: number;
    coordinatedSpeed: number;           // km/h
    bandwidth: number;                  // s
  };

  activeTime?: {
    startTime: string;                  // HH:MM
    endTime: string;                    // HH:MM
    daysOfWeek: number[];
  };
}

/**
 * Traffic signal
 */
export interface TrafficSignal {
  signalId: string;
  nodeId: string;
  location: Coordinates;

  controlType: 'fixed' | 'actuated' | 'adaptive';

  timingPlans: SignalTimingPlan[];
  currentPlan?: string;

  detectors?: Detector[];

  status: {
    operational: boolean;
    currentPhase: number;
    timeInPhase: number;                // s
    lastUpdate: Timestamp;
  };
}

/**
 * Detector
 */
export interface Detector {
  detectorId: string;
  type: 'loop' | 'video' | 'radar' | 'bluetooth';

  location: {
    linkId: string;
    position: number;                   // m from link start
    lane?: number;
  };

  measurements: {
    volume: number;                     // veh/h
    speed: number;                      // km/h
    occupancy: number;                  // %
    timestamp: Timestamp;
  };

  status: {
    operational: boolean;
    lastMaintenance?: Timestamp;
  };
}

// ============================================================================
// Pedestrians
// ============================================================================

/**
 * Pedestrian
 */
export interface Pedestrian {
  pedestrianId: string;

  characteristics: {
    age: number;
    mobility: 'normal' | 'elderly' | 'disabled';
    desiredSpeed: number;               // m/s
  };

  trip: {
    origin: Coordinates;
    destination: Coordinates;
    departureTime: Timestamp;
  };

  currentState: {
    position: Coordinates;
    speed: number;                      // m/s
    heading: number;                    // degrees
  };
}

/**
 * Crosswalk
 */
export interface Crosswalk {
  crosswalkId: string;
  nodeId: string;

  geometry: {
    width: number;                      // m
    length: number;                     // m
    coordinates: Coordinates[];
  };

  signal?: {
    walkTime: number;                   // s
    flashingDontWalkTime: number;       // s
    clearanceInterval: number;          // s
  };

  pedestrianVolume?: {
    volume: number;                     // ped/h
    timestamp: Timestamp;
  };
}

// ============================================================================
// Public Transit
// ============================================================================

/**
 * Transit route
 */
export interface TransitRoute {
  routeId: string;
  routeName: string;
  routeType: 'bus' | 'subway' | 'tram' | 'rail';

  stops: TransitStop[];

  schedule: {
    headway: number;                    // s
    firstDeparture: string;             // HH:MM
    lastDeparture: string;              // HH:MM
  };

  path: string[];                       // link IDs
}

/**
 * Transit stop
 */
export interface TransitStop {
  stopId: string;
  stopName: string;
  location: Coordinates;

  linkId: string;
  position: number;                     // m from link start

  capacity?: number;                    // waiting passengers

  facilities?: {
    shelter: boolean;
    bench: boolean;
    realTimeInfo: boolean;
  };
}

/**
 * Transit vehicle
 */
export interface TransitVehicle {
  vehicleId: string;
  routeId: string;

  capacity: {
    seated: number;
    standing: number;
    total: number;
    wheelchair: number;
  };

  currentState: {
    linkId: string;
    position: number;
    speed: number;
    passengers: number;
    nextStop: string;
    delay: number;                      // s (negative = early)
  };

  schedule: {
    plannedStops: ScheduledStop[];
    actualStops: ActualStop[];
  };
}

/**
 * Scheduled stop
 */
export interface ScheduledStop {
  stopId: string;
  arrivalTime: string;                  // HH:MM:SS
  departureTime: string;                // HH:MM:SS
  dwellTime: number;                    // s
}

/**
 * Actual stop
 */
export interface ActualStop {
  stopId: string;
  arrivalTime: Timestamp;
  departureTime?: Timestamp;
  boarding: number;
  alighting: number;
  delay: number;                        // s
}

// ============================================================================
// Origin-Destination
// ============================================================================

/**
 * Traffic zone
 */
export interface TrafficZone {
  zoneId: string;
  zoneName: string;

  area: {
    polygon: Coordinates[];
    centroid: Coordinates;
  };

  landUse?: {
    residential: number;                // %
    commercial: number;                 // %
    industrial: number;                 // %
    mixed: number;                      // %
  };

  population?: number;
  employment?: number;
}

/**
 * OD matrix
 */
export interface ODMatrix {
  matrixId: string;
  name: string;

  timeOfDay: {
    startTime: string;                  // HH:MM
    endTime: string;                    // HH:MM
  };

  zones: string[];                      // zone IDs

  trips: {
    origin: string;
    destination: string;
    trips: number;                      // vehicles or persons
    mode?: 'car' | 'transit' | 'walk' | 'bike';
  }[];

  metadata?: {
    source: string;
    date: string;
    scalingFactor?: number;
  };
}

// ============================================================================
// Simulation
// ============================================================================

/**
 * Simulation configuration
 */
export interface SimulationConfig {
  simulationId: string;
  name: string;
  description?: string;

  network: {
    networkId: string;
    boundingBox?: {
      minLat: number;
      minLon: number;
      maxLat: number;
      maxLon: number;
    };
  };

  timeSettings: {
    startTime: Timestamp;
    endTime: Timestamp;
    timeStep: number;                   // s (0.1-1.0)
    warmupPeriod: number;               // s
  };

  demand: {
    odMatrixId?: string;
    demandFile?: string;
    scalingFactor: number;
  };

  models: {
    carFollowing: CarFollowingParameters;
    laneChanging: LaneChangingParameters;
    routeChoice: 'static' | 'dynamic' | 'c2x';
  };

  output: {
    interval: number;                   // s
    metrics: string[];
    aggregationLevel: 'vehicle' | 'link' | 'zone' | 'network';
  };

  seed?: number;                        // random seed
}

/**
 * Simulation state
 */
export interface SimulationState {
  simulationId: string;
  status: 'queued' | 'running' | 'completed' | 'failed' | 'cancelled';

  progress: {
    currentTime: Timestamp;
    percentComplete: number;
    vehiclesActive: number;
    vehiclesCompleted: number;
  };

  startedAt?: Timestamp;
  completedAt?: Timestamp;

  error?: {
    code: string;
    message: string;
  };
}

/**
 * Simulation results
 */
export interface SimulationResults {
  simulationId: string;

  networkPerformance: {
    totalVMT: number;                   // vehicle-miles traveled
    totalVHT: number;                   // vehicle-hours traveled
    avgSpeed: number;                   // km/h
    avgDelay: number;                   // s/veh
    totalDelay: number;                 // veh-hours
    avgLOS: 'A' | 'B' | 'C' | 'D' | 'E' | 'F';
  };

  linkPerformance: LinkPerformance[];

  emissions?: {
    totalCO2: number;                   // kg
    totalNOx: number;                   // kg
    totalPM25: number;                  // kg
    fuelConsumed: number;               // liters
  };

  incidents?: IncidentImpact[];
}

/**
 * Link performance
 */
export interface LinkPerformance {
  linkId: string;

  avgSpeed: number;                     // km/h
  avgVolume: number;                    // veh/h
  avgDensity: number;                   // veh/km
  maxQueue: number;                     // vehicles
  avgDelay: number;                     // s/veh

  vcRatio: number;
  los: 'A' | 'B' | 'C' | 'D' | 'E' | 'F';

  timeSeries?: {
    timestamp: Timestamp;
    speed: number;
    volume: number;
    density: number;
  }[];
}

// ============================================================================
// Incidents
// ============================================================================

/**
 * Incident type
 */
export enum IncidentType {
  ACCIDENT = 'accident',
  BREAKDOWN = 'breakdown',
  CONSTRUCTION = 'construction',
  SPECIAL_EVENT = 'special_event',
  WEATHER = 'weather',
  DEBRIS = 'debris',
}

/**
 * Traffic incident
 */
export interface TrafficIncident {
  incidentId: string;
  type: IncidentType;

  location: {
    linkId: string;
    position: number;                   // m from link start
    coordinates: Coordinates;
  };

  impact: {
    lanesBlocked: number[];             // lane indices
    capacityReduction: number;          // %
    speedReduction?: number;            // %
  };

  time: {
    startTime: Timestamp;
    endTime?: Timestamp;
    duration?: number;                  // s (if known)
  };

  severity: 'minor' | 'moderate' | 'major' | 'severe';

  description?: string;
}

/**
 * Incident impact analysis
 */
export interface IncidentImpact {
  incidentId: string;

  spatialImpact: {
    queueLength: number;                // m
    affectedLinks: string[];
    detourRoutes: DetourRoute[];
  };

  temporalImpact: {
    incidentDuration: number;           // s
    recoveryTime: number;               // s
    totalDuration: number;              // s
  };

  metrics: {
    totalDelayHours: number;
    avgSpeedReduction: number;          // %
    affectedVehicles: number;
    economicCost?: number;              // currency
  };
}

/**
 * Detour route
 */
export interface DetourRoute {
  routeId: string;
  links: string[];
  distance: number;                     // m
  travelTime: number;                   // s
  vehiclesUsingRoute: number;
}

// ============================================================================
// Traffic Forecasting
// ============================================================================

/**
 * Traffic forecast
 */
export interface TrafficForecast {
  forecastId: string;
  linkId: string;

  horizon: 'short' | 'medium' | 'long';  // 5min-1h, 1h-24h, 1yr-20yr

  predictions: Prediction[];

  confidence: {
    level: number;                      // 0-1
    interval: {
      lower: number;
      upper: number;
    };
  };

  metadata: {
    model: string;
    accuracy: ModelAccuracy;
    createdAt: Timestamp;
  };
}

/**
 * Prediction
 */
export interface Prediction {
  timestamp: Timestamp;
  speed: number;                        // km/h
  volume: number;                       // veh/h
  density?: number;                     // veh/km
  congestionLevel: 'free' | 'moderate' | 'congested' | 'jammed';
}

/**
 * Model accuracy
 */
export interface ModelAccuracy {
  mae: number;                          // Mean Absolute Error
  rmse: number;                         // Root Mean Squared Error
  mape: number;                         // Mean Absolute Percentage Error
  r2: number;                           // R-squared
}

// ============================================================================
// Congestion
// ============================================================================

/**
 * Congestion data
 */
export interface CongestionData {
  linkId: string;
  timestamp: Timestamp;

  indices: {
    tci: number;                        // Traffic Congestion Index
    tti: number;                        // Travel Time Index
    pti: number;                        // Planning Time Index
  };

  level: 'free' | 'slow' | 'congested' | 'jammed';

  characteristics: {
    type: 'recurrent' | 'non-recurrent';
    cause?: string;
    bottleneckIntensity?: number;
  };
}

// ============================================================================
// API Types
// ============================================================================

/**
 * Generic API response
 */
export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
    details?: any;
  };
  metadata?: {
    timestamp: Timestamp;
    version: string;
  };
}

/**
 * Pagination parameters
 */
export interface PaginationParams {
  page: number;
  limit: number;
  sortBy?: string;
  sortOrder?: 'asc' | 'desc';
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  items: T[];
  total: number;
  page: number;
  limit: number;
  hasMore: boolean;
}

/**
 * Date range filter
 */
export interface DateRangeFilter {
  startDate: Timestamp;
  endDate: Timestamp;
}

// ============================================================================
// Export all types
// ============================================================================

export type {
  Timestamp,
  Coordinates,
  Location,
};
