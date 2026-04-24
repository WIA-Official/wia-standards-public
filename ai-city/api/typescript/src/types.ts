/**
 * WIA AI City Standard - Type Definitions
 *
 * @packageDocumentation
 * @module wia-ai-city
 */

/**
 * Urban domain types
 */
export enum UrbanDomain {
  Traffic = 'traffic',
  Energy = 'energy',
  Water = 'water',
  Waste = 'waste',
  PublicSafety = 'public_safety',
  Environment = 'environment',
  Health = 'health',
  Emergency = 'emergency',
  Parking = 'parking',
  PublicTransport = 'public_transport'
}

/**
 * Sensor type
 */
export enum SensorType {
  Camera = 'camera',
  TrafficLoop = 'traffic_loop',
  AirQuality = 'air_quality',
  NoiseLevel = 'noise_level',
  Weather = 'weather',
  WaterFlow = 'water_flow',
  EnergyMeter = 'energy_meter',
  Occupancy = 'occupancy',
  Motion = 'motion',
  Environmental = 'environmental'
}

/**
 * AI model type
 */
export enum AIModelType {
  ObjectDetection = 'object_detection',
  TrafficPrediction = 'traffic_prediction',
  EnergyForecasting = 'energy_forecasting',
  AnomalyDetection = 'anomaly_detection',
  Optimization = 'optimization',
  NLP = 'nlp',
  ComputerVision = 'computer_vision',
  TimeSeriesAnalysis = 'time_series_analysis'
}

/**
 * Alert severity levels
 */
export enum AlertSeverity {
  Info = 'info',
  Warning = 'warning',
  Critical = 'critical',
  Emergency = 'emergency'
}

/**
 * Geographic coordinates
 */
export interface GeoCoordinate {
  /** Latitude */
  latitude: number;
  /** Longitude */
  longitude: number;
  /** Altitude in meters */
  altitude?: number;
}

/**
 * Geographic bounding box
 */
export interface GeoBoundingBox {
  /** North-west corner */
  northWest: GeoCoordinate;
  /** South-east corner */
  southEast: GeoCoordinate;
}

/**
 * Geographic zone
 */
export interface Zone {
  /** Zone identifier */
  id: string;
  /** Zone name */
  name: string;
  /** Zone type */
  type: 'district' | 'neighborhood' | 'block' | 'intersection' | 'custom';
  /** Bounding box */
  bounds: GeoBoundingBox;
  /** Polygon coordinates */
  polygon?: GeoCoordinate[];
  /** Parent zone ID */
  parentZone?: string;
  /** Zone properties */
  properties: Record<string, unknown>;
}

/**
 * IoT sensor definition
 */
export interface Sensor {
  /** Sensor identifier */
  id: string;
  /** Sensor name */
  name: string;
  /** Sensor type */
  type: SensorType;
  /** Location */
  location: GeoCoordinate;
  /** Zone ID */
  zoneId: string;
  /** Measurement unit */
  unit: string;
  /** Sampling interval in seconds */
  samplingInterval: number;
  /** Is sensor active */
  active: boolean;
  /** Last reading timestamp */
  lastReadingAt?: Date;
  /** Sensor metadata */
  metadata?: Record<string, unknown>;
}

/**
 * Sensor reading
 */
export interface SensorReading {
  /** Reading identifier */
  id: string;
  /** Sensor ID */
  sensorId: string;
  /** Reading value */
  value: number;
  /** Measurement unit */
  unit: string;
  /** Reading timestamp */
  timestamp: Date;
  /** Quality score (0-1) */
  quality: number;
  /** Is anomaly */
  isAnomaly: boolean;
}

/**
 * Traffic data
 */
export interface TrafficData {
  /** Road segment ID */
  segmentId: string;
  /** Current flow (vehicles/hour) */
  flow: number;
  /** Average speed (km/h) */
  averageSpeed: number;
  /** Occupancy percentage */
  occupancy: number;
  /** Congestion level (0-10) */
  congestionLevel: number;
  /** Travel time (seconds) */
  travelTime: number;
  /** Timestamp */
  timestamp: Date;
}

/**
 * Traffic prediction
 */
export interface TrafficPrediction {
  /** Segment ID */
  segmentId: string;
  /** Predicted flow */
  predictedFlow: number;
  /** Predicted congestion level */
  predictedCongestion: number;
  /** Prediction confidence */
  confidence: number;
  /** Prediction horizon (minutes) */
  horizonMinutes: number;
  /** Prediction timestamp */
  timestamp: Date;
}

/**
 * Energy consumption data
 */
export interface EnergyData {
  /** Zone or building ID */
  entityId: string;
  /** Entity type */
  entityType: 'zone' | 'building' | 'streetlight' | 'infrastructure';
  /** Consumption in kWh */
  consumption: number;
  /** Peak demand in kW */
  peakDemand: number;
  /** Renewable percentage */
  renewablePercentage: number;
  /** Cost */
  cost: number;
  /** Currency */
  currency: string;
  /** Period start */
  periodStart: Date;
  /** Period end */
  periodEnd: Date;
}

/**
 * Air quality data
 */
export interface AirQualityData {
  /** Zone ID */
  zoneId: string;
  /** AQI (Air Quality Index) */
  aqi: number;
  /** PM2.5 in μg/m³ */
  pm25: number;
  /** PM10 in μg/m³ */
  pm10: number;
  /** O3 in ppb */
  ozone: number;
  /** NO2 in ppb */
  no2: number;
  /** CO in ppm */
  co: number;
  /** SO2 in ppb */
  so2: number;
  /** Timestamp */
  timestamp: Date;
  /** Health recommendation */
  healthRecommendation: string;
}

/**
 * Public safety incident
 */
export interface SafetyIncident {
  /** Incident ID */
  id: string;
  /** Incident type */
  type: 'accident' | 'crime' | 'fire' | 'medical' | 'hazard' | 'other';
  /** Severity */
  severity: AlertSeverity;
  /** Location */
  location: GeoCoordinate;
  /** Zone ID */
  zoneId: string;
  /** Description */
  description: string;
  /** Reported at */
  reportedAt: Date;
  /** Resolved at */
  resolvedAt?: Date;
  /** Responders */
  responders?: string[];
  /** Status */
  status: 'open' | 'responding' | 'resolved' | 'closed';
}

/**
 * AI model configuration
 */
export interface AIModelConfig {
  /** Model identifier */
  id: string;
  /** Model name */
  name: string;
  /** Model type */
  type: AIModelType;
  /** Target domain */
  domain: UrbanDomain;
  /** Model version */
  version: string;
  /** Input features */
  inputFeatures: string[];
  /** Output fields */
  outputs: string[];
  /** Inference endpoint */
  endpoint?: string;
  /** Batch size */
  batchSize: number;
  /** Is model active */
  active: boolean;
}

/**
 * Prediction request
 */
export interface PredictionRequest {
  /** Request ID */
  requestId: string;
  /** Model ID */
  modelId: string;
  /** Input data */
  input: Record<string, unknown>;
  /** Prediction horizon */
  horizon?: string;
  /** Request timestamp */
  timestamp: Date;
}

/**
 * Prediction response
 */
export interface PredictionResponse {
  /** Request ID */
  requestId: string;
  /** Model ID */
  modelId: string;
  /** Predictions */
  predictions: unknown[];
  /** Confidence scores */
  confidence: number[];
  /** Processing time in ms */
  processingTimeMs: number;
  /** Response timestamp */
  timestamp: Date;
}

/**
 * Alert definition
 */
export interface Alert {
  /** Alert ID */
  id: string;
  /** Alert type */
  type: string;
  /** Domain */
  domain: UrbanDomain;
  /** Severity */
  severity: AlertSeverity;
  /** Title */
  title: string;
  /** Message */
  message: string;
  /** Affected zones */
  affectedZones: string[];
  /** Created at */
  createdAt: Date;
  /** Acknowledged at */
  acknowledgedAt?: Date;
  /** Acknowledged by */
  acknowledgedBy?: string;
  /** Is active */
  active: boolean;
}

/**
 * Dashboard widget configuration
 */
export interface DashboardWidget {
  /** Widget ID */
  id: string;
  /** Widget type */
  type: 'chart' | 'map' | 'gauge' | 'table' | 'stat' | 'alert_list';
  /** Title */
  title: string;
  /** Data source */
  dataSource: string;
  /** Refresh interval in seconds */
  refreshInterval: number;
  /** Widget configuration */
  config: Record<string, unknown>;
}

/**
 * City dashboard configuration
 */
export interface DashboardConfig {
  /** Dashboard ID */
  id: string;
  /** Dashboard name */
  name: string;
  /** Widgets */
  widgets: DashboardWidget[];
  /** Layout */
  layout: 'grid' | 'freeform';
  /** Refresh interval in seconds */
  refreshInterval: number;
}

/**
 * Optimization goal
 */
export interface OptimizationGoal {
  /** Goal ID */
  id: string;
  /** Goal type */
  type: 'minimize' | 'maximize' | 'target';
  /** Metric name */
  metric: string;
  /** Target value */
  targetValue?: number;
  /** Weight (for multi-objective) */
  weight: number;
  /** Constraints */
  constraints?: Constraint[];
}

/**
 * Constraint definition
 */
export interface Constraint {
  /** Constraint name */
  name: string;
  /** Constraint expression */
  expression: string;
  /** Constraint type */
  type: 'hard' | 'soft';
}

/**
 * Optimization result
 */
export interface OptimizationResult {
  /** Goal ID */
  goalId: string;
  /** Recommended actions */
  actions: OptimizationAction[];
  /** Expected improvement */
  expectedImprovement: number;
  /** Confidence */
  confidence: number;
  /** Timestamp */
  timestamp: Date;
}

/**
 * Optimization action
 */
export interface OptimizationAction {
  /** Action ID */
  id: string;
  /** Action type */
  type: string;
  /** Target system */
  targetSystem: string;
  /** Parameters */
  parameters: Record<string, unknown>;
  /** Priority */
  priority: number;
}

/**
 * SDK configuration
 */
export interface AICityConfig {
  /** City identifier */
  cityId: string;
  /** City name */
  cityName: string;
  /** API endpoint */
  apiEndpoint: string;
  /** API key */
  apiKey?: string;
  /** WebSocket endpoint */
  wsEndpoint?: string;
  /** Enabled domains */
  enabledDomains: UrbanDomain[];
  /** Data retention days */
  dataRetentionDays: number;
}

/**
 * Certification level
 */
export enum CertificationLevel {
  Bronze = 'bronze',
  Silver = 'silver',
  Gold = 'gold'
}

/**
 * Compliance report
 */
export interface ComplianceReport {
  /** Standard */
  standard: 'WIA-AI-CITY';
  /** Test date */
  testDate: string;
  /** City configuration */
  config: AICityConfig;
  /** Target level */
  targetLevel: CertificationLevel;
  /** Test results */
  tests: TestResult[];
  /** Overall pass */
  passed: boolean;
  /** Achieved level */
  achievedLevel?: CertificationLevel;
}

/**
 * Test result
 */
export interface TestResult {
  /** Test name */
  testName: string;
  /** Passed */
  passed: boolean;
  /** Notes */
  notes?: string;
}
