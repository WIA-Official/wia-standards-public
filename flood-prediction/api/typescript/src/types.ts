/**
 * WIA-ENE-033: Flood Prediction Standard - TypeScript Type Definitions
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
}

/**
 * Location information
 */
export interface Location {
  regionId: string;
  name: string;
  coordinates: Coordinates;
  address?: string;
  boundary?: GeoJSON;
}

/**
 * GeoJSON geometry (simplified)
 */
export interface GeoJSON {
  type: string;
  coordinates: any;
  properties?: Record<string, any>;
}

// ============================================================================
// Risk Levels
// ============================================================================

/**
 * Flood risk levels (0-4)
 */
export enum RiskLevel {
  LEVEL_0 = 'LEVEL-0',   // 정상 (Normal)
  LEVEL_1 = 'LEVEL-1',   // 관심 (Attention)
  LEVEL_2 = 'LEVEL-2',   // 주의 (Caution)
  LEVEL_3 = 'LEVEL-3',   // 경계 (Warning)
  LEVEL_4 = 'LEVEL-4',   // 심각 (Severe)
}

/**
 * Alert severity
 */
export enum AlertSeverity {
  MINOR = 'minor',
  MODERATE = 'moderate',
  SEVERE = 'severe',
  EXTREME = 'extreme',
}

/**
 * Alert urgency
 */
export enum AlertUrgency {
  IMMEDIATE = 'immediate',
  EXPECTED = 'expected',
  FUTURE = 'future',
}

/**
 * Alert certainty
 */
export enum AlertCertainty {
  OBSERVED = 'observed',
  LIKELY = 'likely',
  POSSIBLE = 'possible',
}

// ============================================================================
// Precipitation Data
// ============================================================================

/**
 * Observed precipitation data
 */
export interface ObservedPrecipitation {
  current: number;      // mm/h (current intensity)
  last1h: number;       // mm (1-hour accumulation)
  last3h: number;       // mm (3-hour accumulation)
  last6h: number;       // mm (6-hour accumulation)
  last24h: number;      // mm (24-hour accumulation)
}

/**
 * Forecast precipitation data
 */
export interface ForecastPrecipitation {
  next1h: number;
  next3h: number;
  next6h: number;
  next12h: number;
  next24h: number;
  hourly: number[];     // hourly forecast (mm/h)
}

/**
 * Radar data
 */
export interface RadarData {
  imageUrl: string;
  coverage: number;     // km
  lastUpdate: Timestamp;
}

/**
 * Precipitation statistics
 */
export interface PrecipitationStatistics {
  returnPeriod: number;  // years
  percentile: number;    // %
  isExtreme: boolean;
}

/**
 * Precipitation data
 */
export interface PrecipitationData {
  observed: ObservedPrecipitation;
  forecast: ForecastPrecipitation;
  radar?: RadarData;
  statistics: PrecipitationStatistics;
}

// ============================================================================
// River Level Data
// ============================================================================

/**
 * River water level status
 */
export enum RiverLevelStatus {
  NORMAL = 'normal',
  ATTENTION = 'attention',
  WARNING = 'warning',
  DANGER = 'danger',
}

/**
 * River level trend direction
 */
export enum TrendDirection {
  RISING = 'rising',
  STABLE = 'stable',
  FALLING = 'falling',
}

/**
 * Current river level
 */
export interface CurrentRiverLevel {
  waterLevel: number;    // m
  flowRate: number;      // m³/s
  velocity: number;      // m/s
  timestamp: Timestamp;
}

/**
 * Reference water levels
 */
export interface ReferenceLevels {
  normal: number;        // m
  attention: number;     // m
  warning: number;       // m
  danger: number;        // m
  bankHeight: number;    // m
}

/**
 * River level forecast
 */
export interface RiverLevelForecast {
  peak: number;          // m
  peakTime: Timestamp;
  hourly: number[];      // hourly forecast (m)
}

/**
 * River level trend
 */
export interface RiverLevelTrend {
  rateOfRise: number;    // m/h
  direction: TrendDirection;
  acceleration: number;  // m/h²
}

/**
 * River status
 */
export interface RiverStatus {
  condition: RiverLevelStatus;
  overflowRisk: number;           // 0-100%
  estimatedTimeToOverflow?: number; // hours
}

/**
 * River level data
 */
export interface RiverLevelData {
  riverId: string;
  name: string;
  stationId: string;
  location: Coordinates;
  current: CurrentRiverLevel;
  levels: ReferenceLevels;
  forecast: RiverLevelForecast;
  trend: RiverLevelTrend;
  status: RiverStatus;
}

// ============================================================================
// Dam/Reservoir Data
// ============================================================================

/**
 * Dam types
 */
export enum DamType {
  MULTI_PURPOSE = 'multi-purpose',
  FLOOD_CONTROL = 'flood-control',
  HYDROPOWER = 'hydropower',
}

/**
 * Gate status
 */
export enum GateStatus {
  OPEN = 'open',
  CLOSED = 'closed',
  PARTIAL = 'partial',
}

/**
 * Dam water level
 */
export interface DamWaterLevel {
  current: number;       // m
  normal: number;        // m (normal high water level)
  flood: number;         // m (flood control level)
  design: number;        // m (design flood level)
}

/**
 * Dam storage
 */
export interface DamStorage {
  current: number;       // million m³
  total: number;         // million m³
  effective: number;     // million m³
  percentage: number;    // %
}

/**
 * Dam flow
 */
export interface DamFlow {
  inflow: number;        // m³/s
  outflow: number;       // m³/s
  discharge: number;     // m³/s (spillway discharge)
  powerGeneration: number; // m³/s
}

/**
 * Dam discharge plan
 */
export interface DischargePlan {
  scheduled: boolean;
  startTime?: Timestamp;
  duration?: number;     // hours
  maxDischarge?: number; // m³/s
  affectedArea?: string[];
}

/**
 * Dam operational status
 */
export interface DamOperationalStatus {
  gateStatus: GateStatus;
  gateOpening: number;   // % (gate opening percentage)
  emergencyMode: boolean;
}

/**
 * Dam/Reservoir status data
 */
export interface DamStatusData {
  damId: string;
  name: string;
  type: DamType;
  location: Coordinates;
  waterLevel: DamWaterLevel;
  storage: DamStorage;
  flow: DamFlow;
  dischargePlan: DischargePlan;
  operational: DamOperationalStatus;
}

// ============================================================================
// Drainage Data
// ============================================================================

/**
 * Drainage facility types
 */
export enum DrainageFacilityType {
  PUMP_STATION = 'pump-station',
  SEWER = 'sewer',
  DETENTION_BASIN = 'detention-basin',
}

/**
 * Operational status
 */
export enum OperationalStatus {
  ACTIVE = 'active',
  STANDBY = 'standby',
  MAINTENANCE = 'maintenance',
  FAILURE = 'failure',
}

/**
 * Drainage capacity
 */
export interface DrainageCapacity {
  design: number;        // m³/s
  current: number;       // m³/s
  utilization: number;   // %
}

/**
 * Pump information
 */
export interface PumpInfo {
  total: number;
  active: number;
  standby: number;
  failed: number;
  efficiency: number;    // %
}

/**
 * Basin water level
 */
export interface BasinWaterLevel {
  current: number;       // m
  capacity: number;      // m
  available: number;     // m
}

/**
 * Operational information
 */
export interface DrainageOperational {
  status: OperationalStatus;
  lastMaintenance: Timestamp;
  nextMaintenance: Timestamp;
  alerts: string[];
}

/**
 * Drainage performance
 */
export interface DrainagePerformance {
  flowRate: number;      // m³/s
  energyUsage: number;   // kWh
  reliability: number;   // %
}

/**
 * Drainage facility data
 */
export interface DrainageData {
  facilityId: string;
  name: string;
  type: DrainageFacilityType;
  location: Coordinates;
  capacity: DrainageCapacity;
  pumps?: PumpInfo;
  waterLevel?: BasinWaterLevel;
  operational: DrainageOperational;
  performance: DrainagePerformance;
}

// ============================================================================
// Flood Prediction
// ============================================================================

/**
 * Time horizon for prediction
 */
export type TimeHorizon = '1h' | '6h' | '24h' | '72h';

/**
 * Spatial resolution
 */
export type SpatialResolution = 'coarse' | 'medium' | 'fine';

/**
 * Flood prediction request
 */
export interface FloodPredictionRequest {
  location: Location;
  timeHorizon: TimeHorizon;
  resolution?: SpatialResolution;
  includePrecipitation?: boolean;
  includeRiverLevels?: boolean;
  includeDamStatus?: boolean;
  includeDrainage?: boolean;
  includeTopography?: boolean;
  runSimulation?: boolean;
  simulationScenarios?: string[];
}

/**
 * Inundation area
 */
export interface InundationArea {
  estimatedArea: number;      // km²
  maxDepth: number;           // m
  averageDepth: number;       // m
  affectedPopulation: number;
  affectedBuildings: number;
  geoJson: GeoJSON;
}

/**
 * Contributing factors
 */
export interface ContributingFactors {
  precipitation: PrecipitationData;
  riverLevels: RiverLevelData[];
  damStatus: DamStatusData[];
  drainage: DrainageData;
  soilSaturation?: number;    // %
}

/**
 * Scenario result
 */
export interface ScenarioResult {
  scenarioName: string;
  probability: number;
  inundationArea: InundationArea;
  peakTime: Timestamp;
}

/**
 * Flood prediction result
 */
export interface FloodPredictionResult {
  predictionId: string;
  timestamp: Timestamp;
  location: Location;
  riskLevel: RiskLevel;
  probability: number;        // 0-100%
  confidence: number;         // 0-100%
  peakTime: Timestamp;
  inundationArea: InundationArea;
  contributingFactors: ContributingFactors;
  scenarios?: {
    bestCase: ScenarioResult;
    worstCase: ScenarioResult;
    mostLikely: ScenarioResult;
  };
  recommendations: string[];
  evacuationRequired: boolean;
  affectedAreas: string[];
}

// ============================================================================
// Flood Alerts
// ============================================================================

/**
 * Flood event type
 */
export enum FloodEventType {
  RIVER_FLOOD = 'river-flood',
  URBAN_FLOOD = 'urban-flood',
  COASTAL_FLOOD = 'coastal-flood',
  FLASH_FLOOD = 'flash-flood',
  DAM_DISCHARGE = 'dam-discharge',
}

/**
 * Alert issuer
 */
export interface AlertIssuer {
  organizationId: string;
  name: string;
  authorityLevel: string;
}

/**
 * Alert information
 */
export interface AlertInfo {
  level: RiskLevel;
  severity: AlertSeverity;
  urgency: AlertUrgency;
  certainty: AlertCertainty;
}

/**
 * Affected areas
 */
export interface AffectedAreas {
  regionIds: string[];
  geoJson: GeoJSON;
  population: number;
  buildings: number;
}

/**
 * Event information
 */
export interface EventInfo {
  type: FloodEventType;
  onset: Timestamp;
  expiry: Timestamp;
  peakTime: Timestamp;
}

/**
 * Instructions
 */
export interface Instructions {
  ko: string;
  en: string;
  actions: string[];
  evacuationRequired: boolean;
  evacuationZones: string[];
}

/**
 * Contact information
 */
export interface ContactInfo {
  emergency: string;
  information: string;
  website: string;
}

/**
 * Flood alert
 */
export interface FloodAlert {
  alertId: string;
  timestamp: Timestamp;
  issuer: AlertIssuer;
  alert: AlertInfo;
  affectedAreas: AffectedAreas;
  event: EventInfo;
  instructions: Instructions;
  shelters?: EvacuationShelter[];
  contact: ContactInfo;
}

// ============================================================================
// Evacuation
// ============================================================================

/**
 * Shelter type
 */
export enum ShelterType {
  SCHOOL = 'school',
  COMMUNITY_CENTER = 'community-center',
  SPORTS_FACILITY = 'sports-facility',
  TEMPORARY = 'temporary',
}

/**
 * Shelter capacity
 */
export interface ShelterCapacity {
  total: number;
  current: number;
  available: number;
}

/**
 * Shelter facilities
 */
export interface ShelterFacilities {
  restrooms: boolean;
  kitchen: boolean;
  medicalRoom: boolean;
  powerSupply: boolean;
  water: boolean;
  heating: boolean;
  cooling: boolean;
  wifi: boolean;
}

/**
 * Accessibility
 */
export interface Accessibility {
  wheelchairAccessible: boolean;
  elevatorAvailable: boolean;
  parkingAvailable: boolean;
  publicTransport: string[];
}

/**
 * Shelter supplies
 */
export interface ShelterSupplies {
  food: number;          // servings
  water: number;         // L
  blankets: number;
  firstAidKits: number;
  emergencyLights: number;
}

/**
 * Shelter status
 */
export interface ShelterStatus {
  operational: boolean;
  lastInspection: Timestamp;
  contact: string;
  openingHours: string;
}

/**
 * Evacuation shelter
 */
export interface EvacuationShelter {
  shelterId: string;
  name: string;
  type: ShelterType;
  location: Coordinates;
  address: string;
  capacity: ShelterCapacity;
  facilities: ShelterFacilities;
  accessibility: Accessibility;
  supplies: ShelterSupplies;
  status: ShelterStatus;
}

/**
 * Route status
 */
export enum RouteStatus {
  CLEAR = 'clear',
  CAUTION = 'caution',
  BLOCKED = 'blocked',
}

/**
 * Path information
 */
export interface PathInfo {
  coordinates: Coordinates[];
  distance: number;      // m
  estimatedTime: number; // minutes
  safetyScore: number;   // 0-100
}

/**
 * Flooded segment
 */
export interface FloodedSegment {
  start: Coordinates;
  end: Coordinates;
  depth: number;         // m
}

/**
 * Route hazards
 */
export interface RouteHazards {
  floodedSegments: FloodedSegment[];
  blockedRoads: string[];
  highRiskAreas: string[];
}

/**
 * Alternative route
 */
export interface AlternativeRoute {
  routeId: string;
  distance: number;
  time: number;
  safetyScore: number;
}

/**
 * Route update
 */
export interface RouteUpdate {
  timestamp: Timestamp;
  status: RouteStatus;
  message: string;
}

/**
 * Evacuation route
 */
export interface EvacuationRoute {
  routeId: string;
  origin: Coordinates;
  destination: EvacuationShelter;
  path: PathInfo;
  hazards: RouteHazards;
  alternatives: AlternativeRoute[];
  updates: RouteUpdate[];
}

// ============================================================================
// Monitoring
// ============================================================================

/**
 * Region status
 */
export interface RegionStatus {
  regionId: string;
  name: string;
  riskLevel: RiskLevel;
  status: string;
  population: number;
  evacuationProgress: number; // %
}

/**
 * Sensor status
 */
export interface SensorStatus {
  total: number;
  active: number;
  offline: number;
  maintenance: number;
}

/**
 * System status
 */
export interface SystemStatus {
  uptime: number;        // %
  latency: number;       // ms
  dataQuality: number;   // %
  errors: string[];
}

/**
 * Monitoring dashboard
 */
export interface MonitoringDashboard {
  timestamp: Timestamp;
  refreshInterval: number; // seconds
  overview: {
    activeAlerts: number;
    highRiskAreas: number;
    evacuatedPopulation: number;
    operationalShelters: number;
  };
  regions: RegionStatus[];
  sensors: SensorStatus;
  system: SystemStatus;
}

// ============================================================================
// Performance & Reporting
// ============================================================================

/**
 * Prediction performance
 */
export interface PredictionPerformance {
  leadTime: number;      // hours
  accuracy: number;      // %
  falseAlarms: number;
  missedEvents: number;
}

/**
 * Response performance
 */
export interface ResponsePerformance {
  alertsSent: number;
  alertDeliveryTime: number;   // minutes
  evacuatedPopulation: number;
  evacuationCompletionTime: number; // hours
  sheltersOpened: number;
}

/**
 * Impact assessment
 */
export interface ImpactAssessment {
  casualties: number;
  injuries: number;
  affectedPopulation: number;
  damagedBuildings: number;
  economicLoss: number;  // KRW
  infrastructureDamage: string[];
}

/**
 * Lessons learned
 */
export interface LessonsLearned {
  successes: string[];
  failures: string[];
  recommendations: string[];
}

/**
 * Flood response report
 */
export interface FloodResponseReport {
  reportId: string;
  period: {
    start: Timestamp;
    end: Timestamp;
  };
  event: {
    eventId: string;
    type: string;
    severity: string;
    duration: number;    // hours
  };
  prediction: PredictionPerformance;
  response: ResponsePerformance;
  impact: ImpactAssessment;
  lessons: LessonsLearned;
}

// ============================================================================
// KPIs
// ============================================================================

/**
 * Prediction KPIs
 */
export interface PredictionKPIs {
  accuracy: {
    overall: number;
    shortTerm: number;
    mediumTerm: number;
    longTerm: number;
  };
  leadTime: {
    average: number;
    minimum: number;
    maximum: number;
  };
  reliability: {
    falseAlarmRate: number;
    missedEventRate: number;
    confidenceScore: number;
  };
}

/**
 * Response KPIs
 */
export interface ResponseKPIs {
  alerting: {
    averageDeliveryTime: number;
    deliverySuccess: number;
    publicAwareness: number;
  };
  evacuation: {
    completionRate: number;
    averageTime: number;
    shelterUtilization: number;
  };
  damageReduction: {
    casualties: number;
    propertyLoss: number;
    comparedToPrevious: number;
  };
}

/**
 * KPI dashboard
 */
export interface KPIDashboard {
  periodId: string;
  timestamp: Timestamp;
  prediction: PredictionKPIs;
  response: ResponseKPIs;
  overallScore?: number; // 0-100
}

// ============================================================================
// API Request/Response Types
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
  GeoJSON,
};
