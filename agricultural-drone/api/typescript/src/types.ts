/**
 * WIA-AGRI-003 Agricultural Drone Standard - TypeScript Type Definitions
 * Version: 1.0.0
 * Philosophy: 弘益人間 (Benefit All Humanity)
 */

/**
 * WIA Agricultural Drone Standard Version
 */
export type WIAVersion = '1.0' | '1.1' | '2.0';

/**
 * Drone Types
 */
export type DroneType = 'multirotor' | 'fixed_wing' | 'hybrid' | 'vtol';

/**
 * Mission Types
 */
export type MissionType =
  | 'survey'
  | 'spraying'
  | 'monitoring'
  | 'inspection'
  | 'mapping'
  | 'thermal_imaging';

/**
 * Drone Status
 */
export type DroneStatus =
  | 'idle'
  | 'in_flight'
  | 'charging'
  | 'maintenance'
  | 'error';

/**
 * Flight Mode
 */
export type FlightMode = 'manual' | 'assisted' | 'autonomous' | 'rtl' | 'hold';

/**
 * Payload Types
 */
export type PayloadType =
  | 'rgb_camera'
  | 'multispectral'
  | 'thermal'
  | 'lidar'
  | 'sprayer'
  | 'seeder';

/**
 * Agricultural Drone Configuration
 */
export interface AgriculturalDroneConfig {
  droneId: string;
  name: string;
  type: DroneType;
  manufacturer?: string;
  model?: string;
  serialNumber?: string;
  specifications: DroneSpecifications;
  payloads: PayloadInfo[];
  homeLocation: Location;
  owner: Owner;
  metadata?: Record<string, any>;
}

/**
 * Drone Specifications
 */
export interface DroneSpecifications {
  maxFlightTime: number; // minutes
  maxSpeed: number; // m/s
  maxAltitude: number; // meters
  maxPayload: number; // kg
  wingspan?: number; // meters (for fixed wing)
  rotorCount?: number; // for multirotors
  batteryCapacity: number; // mAh
  weight: number; // kg
  gpsAccuracy?: number; // meters
}

/**
 * Payload Information
 */
export interface PayloadInfo {
  payloadId: string;
  type: PayloadType;
  model?: string;
  resolution?: string;
  bands?: string[]; // For multispectral cameras
  capacity?: number; // For sprayers/seeders
  weight: number; // kg
  mountPosition?: string;
}

/**
 * Geographic Location
 */
export interface Location {
  latitude: number;
  longitude: number;
  altitude?: number;
  address?: string;
  country?: string;
}

/**
 * Owner Information
 */
export interface Owner {
  id: string;
  name: string;
  email?: string;
  phone?: string;
  organization?: string;
  license?: string;
}

/**
 * Flight Mission
 */
export interface FlightMission {
  missionId: string;
  droneId: string;
  name: string;
  type: MissionType;
  fieldId?: string;
  plannedDate: string;
  status: 'planned' | 'in_progress' | 'completed' | 'cancelled' | 'failed';
  flightPlan: FlightPlan;
  payload: PayloadType;
  estimatedDuration?: number; // minutes
  actualDuration?: number; // minutes
  weather?: WeatherConditions;
  operator?: string;
  metadata?: Record<string, any>;
}

/**
 * Flight Plan
 */
export interface FlightPlan {
  waypoints: Waypoint[];
  altitude: number; // meters AGL
  speed: number; // m/s
  overlap?: number; // percentage
  sideOverlap?: number; // percentage
  flightPattern?: 'grid' | 'circular' | 'perimeter' | 'custom';
  area?: GeoJSON;
  homeLocation: Location;
}

/**
 * Waypoint
 */
export interface Waypoint {
  sequence: number;
  location: Location;
  action?: WaypointAction;
  speed?: number;
  heading?: number;
}

/**
 * Waypoint Action
 */
export interface WaypointAction {
  type: 'photo' | 'video_start' | 'video_stop' | 'spray_on' | 'spray_off' | 'hover';
  duration?: number; // seconds
  parameters?: Record<string, any>;
}

/**
 * GeoJSON Polygon
 */
export interface GeoJSON {
  type: 'Polygon' | 'MultiPolygon';
  coordinates: number[][][] | number[][][][];
}

/**
 * Flight Telemetry
 */
export interface FlightTelemetry {
  timestamp: string;
  droneId: string;
  missionId?: string;
  location: Location;
  heading: number; // degrees
  speed: number; // m/s
  altitude: number; // meters
  battery: number; // percentage
  signal: number; // percentage
  flightMode: FlightMode;
  satellites?: number;
  errors?: string[];
}

/**
 * Weather Conditions
 */
export interface WeatherConditions {
  temperature: number; // Celsius
  humidity: number; // percentage
  windSpeed: number; // m/s
  windDirection: number; // degrees
  precipitation?: number; // mm
  visibility?: number; // meters
  cloudCover?: number; // percentage
}

/**
 * Image Capture
 */
export interface ImageCapture {
  imageId: string;
  missionId: string;
  droneId: string;
  timestamp: string;
  location: Location;
  altitude: number;
  heading: number;
  payloadType: PayloadType;
  imageUrl?: string;
  thumbnailUrl?: string;
  format?: string;
  size?: number; // bytes
  metadata?: ImageMetadata;
}

/**
 * Image Metadata
 */
export interface ImageMetadata {
  width?: number;
  height?: number;
  bands?: string[];
  groundSampleDistance?: number; // cm/pixel
  exposureTime?: string;
  iso?: number;
  focalLength?: number;
  gimbalPitch?: number;
  gimbalRoll?: number;
  gimbalYaw?: number;
}

/**
 * Spray Operation
 */
export interface SprayOperation {
  operationId: string;
  missionId: string;
  droneId: string;
  fieldId?: string;
  timestamp: string;
  product: string;
  totalVolume: number; // liters
  area: number; // hectares
  applicationRate: number; // l/ha
  coverage: GeoJSON;
  nozzleSettings?: NozzleSettings;
  environmentalConditions: WeatherConditions;
}

/**
 * Nozzle Settings
 */
export interface NozzleSettings {
  nozzleType?: string;
  flowRate: number; // ml/min
  pressure: number; // bar
  dropletSize?: number; // microns
  swathWidth: number; // meters
}

/**
 * Drone Maintenance Record
 */
export interface MaintenanceRecord {
  recordId: string;
  droneId: string;
  date: string;
  type: 'inspection' | 'repair' | 'replacement' | 'calibration' | 'software_update';
  description: string;
  technician?: string;
  partsReplaced?: string[];
  cost?: number;
  nextMaintenanceDate?: string;
}

/**
 * Flight Log
 */
export interface FlightLog {
  logId: string;
  missionId: string;
  droneId: string;
  startTime: string;
  endTime: string;
  duration: number; // minutes
  distance: number; // meters
  maxAltitude: number;
  maxSpeed: number;
  batteryUsed: number; // percentage
  telemetryPoints: FlightTelemetry[];
  issues?: string[];
  summary?: string;
}

/**
 * Orthomosaic Map
 */
export interface OrthomosaicMap {
  mapId: string;
  missionId: string;
  fieldId?: string;
  createdDate: string;
  imageCount: number;
  area: number; // hectares
  resolution: number; // cm/pixel
  bands: string[];
  fileUrl?: string;
  thumbnailUrl?: string;
  format?: string;
  fileSize?: number; // MB
  processingTime?: number; // minutes
}

/**
 * Anomaly Detection Result
 */
export interface AnomalyDetection {
  detectionId: string;
  missionId: string;
  imageId?: string;
  timestamp: string;
  location: Location;
  anomalyType: 'pest' | 'disease' | 'weed' | 'water_stress' | 'nutrient_deficiency';
  confidence: number; // 0-100
  severity?: 'low' | 'medium' | 'high';
  area?: number; // square meters
  imageUrl?: string;
  recommendations?: string[];
}

/**
 * Battery Information
 */
export interface BatteryInfo {
  batteryId: string;
  droneId: string;
  capacity: number; // mAh
  voltage: number; // V
  currentCharge: number; // percentage
  cycles: number;
  health: number; // percentage
  temperature?: number; // Celsius
  lastCharged?: string;
}

/**
 * SDK Configuration
 */
export interface SDKConfig {
  baseURL?: string;
  apiKey?: string;
  droneId?: string;
  timeout?: number;
}

/**
 * List Query Parameters
 */
export interface ListParams {
  limit?: number;
  offset?: number;
  startDate?: string;
  endDate?: string;
  status?: string;
  type?: string;
}

/**
 * Error Response
 */
export interface ErrorResponse {
  error: {
    code: string;
    message: string;
    details?: any;
  };
}
