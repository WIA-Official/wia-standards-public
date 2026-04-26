/**
 * WIA-ENE-032: Forest Fire Detection Standard - TypeScript Type Definitions
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
  latitude: number;
  longitude: number;
  elevation: number;
  address: string;
  forestType: string;
  administrativeArea: string;
}

/**
 * GeoJSON Polygon for fire perimeter
 */
export interface GeoJSONPolygon {
  type: 'Polygon';
  coordinates: number[][][];
}

// ============================================================================
// Fire Danger Rating
// ============================================================================

/**
 * Fire danger levels (1-5)
 */
export enum FireDangerLevel {
  LOW = 1,           // 낮음 (녹색)
  MODERATE = 2,      // 보통 (황색)
  HIGH = 3,          // 높음 (주황)
  VERY_HIGH = 4,     // 매우 높음 (적색)
  EXTREME = 5,       // 극도 (보라)
}

/**
 * Fire danger calculation inputs
 */
export interface FireDangerCalculation {
  // Weather conditions (40% weight)
  temperature: number;        // °C
  humidity: number;           // %
  windSpeed: number;          // m/s
  precipitation: number;      // mm (last 7 days)

  // Fuel conditions (30% weight)
  deadFuelMoisture: number;   // %
  liveFuelMoisture: number;   // %
  fuelLoad: number;           // ton/ha

  // Terrain conditions (20% weight)
  slope: number;              // degrees
  aspect: number;             // degrees (0-360)
  elevation: number;          // meters

  // Human activity (10% weight)
  humanActivity: number;      // 0-1
  proximity: number;          // km to populated area
}

// ============================================================================
// Satellite Detection
// ============================================================================

/**
 * Satellite detection methods
 */
export enum SatelliteType {
  MODIS_TERRA = 'MODIS-Terra',
  MODIS_AQUA = 'MODIS-Aqua',
  VIIRS_NPP = 'VIIRS-Suomi-NPP',
  VIIRS_NOAA20 = 'VIIRS-NOAA-20',
  VIIRS_NOAA21 = 'VIIRS-NOAA-21',
  SENTINEL_2 = 'Sentinel-2',
  SENTINEL_3 = 'Sentinel-3',
}

/**
 * Detection method
 */
export enum DetectionMethod {
  SATELLITE_MODIS = 'MODIS',
  SATELLITE_VIIRS = 'VIIRS',
  SATELLITE_SENTINEL = 'Sentinel',
  THERMAL_CAMERA = 'thermal_camera',
  SMOKE_DETECTOR = 'smoke_detector',
  DRONE = 'drone',
  HUMAN_REPORT = 'human_report',
  AI_VIDEO = 'ai_video',
}

/**
 * Satellite metadata
 */
export interface SatelliteMetadata {
  satellite: SatelliteType | string;
  sensor: string;
  resolution: number;         // meters
  quality: 'low' | 'medium' | 'high';
  validated: boolean;
}

// ============================================================================
// Fire Characteristics
// ============================================================================

/**
 * Fire type classification
 */
export enum FireType {
  SURFACE = 'surface',        // 지표화
  CROWN = 'crown',            // 수관화
  GROUND = 'ground',          // 지중화
  SMOLDERING = 'smoldering',  // 잔불
  SPOT = 'spot',              // 비화
}

/**
 * Fire characteristics
 */
export interface FireCharacteristics {
  frp: number;                // Fire Radiative Power (MW)
  brightness: number;         // Brightness temperature (K)
  area: number;               // Burned area (m²)
  perimeter: number;          // Fire perimeter (m)
  fireLineIntensity: number;  // kW/m
  rateOfSpread: number;       // m/min
  fireType?: FireType;
}

// ============================================================================
// Weather Conditions
// ============================================================================

/**
 * Weather conditions
 */
export interface WeatherConditions {
  temperature: number;        // °C
  humidity: number;           // %
  windSpeed: number;          // m/s
  windDirection: number;      // degrees (0-360)
  precipitation24h: number;   // mm
  pressure?: number;          // hPa
  solarRadiation?: number;    // W/m²
}

// ============================================================================
// Risk Assessment
// ============================================================================

/**
 * Threat level
 */
export enum ThreatLevel {
  LOW = 'low',
  MODERATE = 'moderate',
  HIGH = 'high',
  CRITICAL = 'critical',
}

/**
 * Risk assessment
 */
export interface RiskAssessment {
  dangerLevel: FireDangerLevel;
  fwi: number;                // Fire Weather Index
  threatToLife: ThreatLevel;
  threatToProperty: ThreatLevel;
  evacuationRequired: boolean;
}

// ============================================================================
// Fire Detection Event
// ============================================================================

/**
 * Fire detection event
 */
export interface FireDetectionEvent {
  eventId: string;
  timestamp: Timestamp;
  detectionMethod: DetectionMethod;
  confidenceLevel: number;    // 0-100

  location: Location;
  fireCharacteristics: FireCharacteristics;
  weatherConditions: WeatherConditions;
  riskAssessment: RiskAssessment;
  metadata: SatelliteMetadata | Record<string, any>;
}

// ============================================================================
// Fire Spread Prediction
// ============================================================================

/**
 * Spread prediction
 */
export interface SpreadPrediction {
  currentArea: number;        // m²
  predicted6h: number;        // m²
  predicted12h: number;       // m²
  predicted24h: number;       // m²
  confidence: number;         // 0-100
}

/**
 * Asset type
 */
export enum AssetType {
  RESIDENTIAL = 'residential',
  COMMERCIAL = 'commercial',
  INFRASTRUCTURE = 'infrastructure',
  CULTURAL = 'cultural',
  ECOLOGICAL = 'ecological',
}

/**
 * Priority level
 */
export enum PriorityLevel {
  LOW = 'low',
  MEDIUM = 'medium',
  HIGH = 'high',
  CRITICAL = 'critical',
}

/**
 * Threatened asset
 */
export interface ThreatenedAsset {
  type: AssetType;
  name: string;
  population?: number;
  distance: number;           // km
  arrivalTime: Timestamp;
  priority: PriorityLevel;
}

/**
 * Prediction model information
 */
export interface PredictionModel {
  name: string;
  version: string;
  accuracy: number;           // 0-100
}

/**
 * Fire spread prediction
 */
export interface FireSpreadPrediction {
  predictionId: string;
  fireEventId: string;
  timestamp: Timestamp;
  forecastHorizon: number;    // hours

  spreadPrediction: SpreadPrediction;
  firePerimeter: GeoJSONPolygon;
  threatenedAssets: ThreatenedAsset[];
  model: PredictionModel;
}

// ============================================================================
// Thermal Detection
// ============================================================================

/**
 * Thermal camera detection
 */
export interface ThermalDetection {
  cameraId: string;
  location: Coordinates;
  detectionTime: Timestamp;
  temperature: number;        // °C
  imageUrl?: string;
  confidence: number;         // 0-100
}

/**
 * Drone thermal detection
 */
export interface DroneThermalDetection {
  droneId: string;
  location: Coordinates;
  altitude: number;           // meters
  detectionTime: Timestamp;
  thermalImageUrl: string;
  visibleImageUrl?: string;
  hotspots: Coordinates[];
}

// ============================================================================
// Smoke Detection
// ============================================================================

/**
 * Smoke detection method
 */
export enum SmokeDetectionMethod {
  OPTICAL = 'optical',
  IONIZATION = 'ionization',
  AI_VIDEO = 'ai_video',
}

/**
 * Smoke detection event
 */
export interface SmokeDetectionEvent {
  detectorId: string;
  location: Coordinates;
  detectionTime: Timestamp;
  method: SmokeDetectionMethod;
  particleDensity?: number;   // mg/m³
  confidence: number;         // 0-100
  imageUrl?: string;
}

// ============================================================================
// Fuel Moisture
// ============================================================================

/**
 * Fuel moisture measurement
 */
export interface FuelMoisture {
  measurementId: string;
  location: Coordinates;
  timestamp: Timestamp;
  deadFuelMoisture: number;   // % (DFMC)
  liveFuelMoisture: number;   // % (LFMC)
  method: 'direct' | 'modeled';
}

// ============================================================================
// Evacuation Zone
// ============================================================================

/**
 * Evacuation zone status
 */
export enum EvacuationStatus {
  GREEN = 'green',            // Safe (>12h)
  YELLOW = 'yellow',          // Alert (6-12h)
  ORANGE = 'orange',          // Prepare (2-6h)
  RED = 'red',                // Evacuate (0-2h)
}

/**
 * Evacuation zone
 */
export interface EvacuationZone {
  zoneId: string;
  name: string;
  boundary: GeoJSONPolygon;
  status: EvacuationStatus;
  population: number;
  estimatedArrivalTime?: Timestamp;
  evacuationRoutes: string[];
}

// ============================================================================
// Firefighting Resources
// ============================================================================

/**
 * Resource type
 */
export enum ResourceType {
  HELICOPTER = 'helicopter',
  FIRE_TRUCK = 'fire_truck',
  PERSONNEL = 'personnel',
  EQUIPMENT = 'equipment',
}

/**
 * Resource status
 */
export enum ResourceStatus {
  AVAILABLE = 'available',
  EN_ROUTE = 'en_route',
  DEPLOYED = 'deployed',
  UNAVAILABLE = 'unavailable',
}

/**
 * Resource capacity
 */
export interface ResourceCapacity {
  waterTank?: number;         // liters
  currentLoad?: number;       // liters
  personnel?: number;
  equipment?: string[];
}

/**
 * Firefighting resource
 */
export interface FirefightingResource {
  resourceId: string;
  type: ResourceType;
  status: ResourceStatus;
  location: Coordinates;
  capacity: ResourceCapacity;
  eta?: Timestamp;
}

// ============================================================================
// Alert System
// ============================================================================

/**
 * Alert level
 */
export enum AlertLevel {
  LEVEL_1 = 1,                // 관심 (Interest)
  LEVEL_2 = 2,                // 주의 (Attention)
  LEVEL_3 = 3,                // 경계 (Alert)
  LEVEL_4 = 4,                // 심각 (Critical)
}

/**
 * Alert channel
 */
export enum AlertChannel {
  EMAIL = 'email',
  SMS = 'sms',
  PHONE = 'phone',
  CBS = 'cbs',                // Cell Broadcast Service
  APP = 'app',
  SIREN = 'siren',
}

/**
 * Alert recipient
 */
export interface AlertRecipient {
  recipientId: string;
  name: string;
  channels: AlertChannel[];
  language: string;
}

/**
 * Fire alert
 */
export interface FireAlert {
  alertId: string;
  fireEventId: string;
  timestamp: Timestamp;
  level: AlertLevel;
  recipients: AlertRecipient[];
  message: string;
  deliveryStatus: Record<string, 'pending' | 'sent' | 'failed'>;
}

// ============================================================================
// API Response Types
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
  };
  metadata?: {
    timestamp: Timestamp;
    requestId: string;
  };
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  success: boolean;
  data: T[];
  pagination: {
    total: number;
    page: number;
    pageSize: number;
    totalPages: number;
  };
}

/**
 * Pagination parameters
 */
export interface PaginationParams {
  page?: number;
  pageSize?: number;
  sortBy?: string;
  sortOrder?: 'asc' | 'desc';
}

/**
 * Date range filter
 */
export interface DateRangeFilter {
  startDate: Timestamp;
  endDate: Timestamp;
}

// ============================================================================
// Certification
// ============================================================================

/**
 * Certification level
 */
export enum CertificationLevel {
  TIER_1 = 'tier_1',          // Basic
  TIER_2 = 'tier_2',          // Advanced
  TIER_3 = 'tier_3',          // Professional
}

/**
 * Certification status
 */
export interface CertificationStatus {
  level: CertificationLevel;
  issueDate: Timestamp;
  expiryDate: Timestamp;
  uptime: number;             // %
  avgDetectionTime: number;   // minutes
  falsePositiveRate: number;  // %
  missedDetectionRate: number; // %
}

// ============================================================================
// Export all types
// ============================================================================

export type {
  // Basic
  Timestamp,
  Coordinates,
  Location,
  GeoJSONPolygon,

  // Fire Danger
  FireDangerCalculation,

  // Satellite
  SatelliteMetadata,

  // Fire
  FireCharacteristics,
  WeatherConditions,
  RiskAssessment,
  FireDetectionEvent,

  // Prediction
  SpreadPrediction,
  ThreatenedAsset,
  PredictionModel,
  FireSpreadPrediction,

  // Detection
  ThermalDetection,
  DroneThermalDetection,
  SmokeDetectionEvent,
  FuelMoisture,

  // Evacuation
  EvacuationZone,

  // Resources
  ResourceCapacity,
  FirefightingResource,

  // Alerts
  AlertRecipient,
  FireAlert,

  // API
  ApiResponse,
  PaginatedResponse,
  PaginationParams,
  DateRangeFilter,

  // Certification
  CertificationStatus,
};

export {
  // Enums
  FireDangerLevel,
  SatelliteType,
  DetectionMethod,
  FireType,
  ThreatLevel,
  AssetType,
  PriorityLevel,
  SmokeDetectionMethod,
  EvacuationStatus,
  ResourceType,
  ResourceStatus,
  AlertLevel,
  AlertChannel,
  CertificationLevel,
};
