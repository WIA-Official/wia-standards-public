/**
 * WIA-AGRI-004 Soil Sensor Standard - TypeScript Type Definitions
 * Version: 1.0.0
 * Philosophy: 弘益人間 (Benefit All Humanity)
 */

/**
 * WIA Soil Sensor Standard Version
 */
export type WIAVersion = '1.0' | '1.1' | '2.0';

/**
 * Sensor Types
 */
export type SensorType =
  | 'moisture'
  | 'temperature'
  | 'ph'
  | 'ec'
  | 'npk'
  | 'organic_matter'
  | 'salinity'
  | 'oxygen';

/**
 * Sensor Status
 */
export type SensorStatus = 'active' | 'inactive' | 'maintenance' | 'error' | 'calibration';

/**
 * Measurement Quality
 */
export type MeasurementQuality = 'excellent' | 'good' | 'fair' | 'poor';

/**
 * Soil Sensor Configuration
 */
export interface SoilSensorConfig {
  sensorId: string;
  name: string;
  type: SensorType;
  manufacturer?: string;
  model?: string;
  serialNumber?: string;
  specifications: SensorSpecifications;
  installation: InstallationInfo;
  status: SensorStatus;
  calibration?: CalibrationInfo;
  metadata?: Record<string, any>;
}

/**
 * Sensor Specifications
 */
export interface SensorSpecifications {
  measurementRange: {
    min: number;
    max: number;
    unit: string;
  };
  accuracy: number;
  resolution: number;
  responseTime?: number; // seconds
  powerRequirement?: string;
  communicationProtocol?: string;
  batteryLife?: number; // days
  operatingTemperature?: {
    min: number;
    max: number;
  };
}

/**
 * Installation Information
 */
export interface InstallationInfo {
  fieldId?: string;
  location: Location;
  depth: number; // cm
  installationDate: string;
  soilType?: string;
  zone?: string;
  installedBy?: string;
  notes?: string;
}

/**
 * Geographic Location
 */
export interface Location {
  latitude: number;
  longitude: number;
  altitude?: number;
  accuracy?: number; // meters
}

/**
 * Calibration Information
 */
export interface CalibrationInfo {
  lastCalibrationDate: string;
  nextCalibrationDate?: string;
  calibratedBy?: string;
  calibrationValues?: Record<string, number>;
  certificateUrl?: string;
}

/**
 * Soil Measurement
 */
export interface SoilMeasurement {
  measurementId: string;
  sensorId: string;
  timestamp: string;
  sensorType: SensorType;
  value: number;
  unit: string;
  quality: MeasurementQuality;
  temperature?: number; // Sensor temperature
  batteryLevel?: number; // Percentage
  signalStrength?: number; // RSSI or percentage
  metadata?: Record<string, any>;
}

/**
 * Aggregated Soil Data
 */
export interface AggregatedSoilData {
  sensorId: string;
  fieldId?: string;
  period: string; // ISO 8601 duration
  startTime: string;
  endTime: string;
  measurements: {
    [key in SensorType]?: SensorStatistics;
  };
}

/**
 * Sensor Statistics
 */
export interface SensorStatistics {
  count: number;
  average: number;
  min: number;
  max: number;
  stdDev?: number;
  unit: string;
  trend?: 'increasing' | 'decreasing' | 'stable';
}

/**
 * Soil Profile
 */
export interface SoilProfile {
  profileId: string;
  location: Location;
  samplingDate: string;
  layers: SoilLayer[];
  metadata?: Record<string, any>;
}

/**
 * Soil Layer
 */
export interface SoilLayer {
  depth: {
    from: number; // cm
    to: number; // cm
  };
  moisture?: number;
  temperature?: number;
  ph?: number;
  ec?: number; // Electrical Conductivity
  organicMatter?: number; // Percentage
  nutrients?: NutrientLevels;
  texture?: string;
  color?: string;
}

/**
 * Nutrient Levels
 */
export interface NutrientLevels {
  nitrogen?: number; // ppm or mg/kg
  phosphorus?: number;
  potassium?: number;
  calcium?: number;
  magnesium?: number;
  sulfur?: number;
  iron?: number;
  zinc?: number;
  copper?: number;
  manganese?: number;
  boron?: number;
}

/**
 * Alert Configuration
 */
export interface AlertConfig {
  alertId: string;
  sensorId: string;
  name: string;
  enabled: boolean;
  condition: AlertCondition;
  notification: NotificationConfig;
  createdAt: string;
  updatedAt?: string;
}

/**
 * Alert Condition
 */
export interface AlertCondition {
  metric: SensorType;
  operator: 'gt' | 'lt' | 'eq' | 'gte' | 'lte' | 'between' | 'outside';
  threshold: number | [number, number];
  duration?: number; // minutes - alert only if condition persists
}

/**
 * Notification Configuration
 */
export interface NotificationConfig {
  channels: ('email' | 'sms' | 'push' | 'webhook')[];
  recipients?: string[];
  webhookUrl?: string;
  messageTemplate?: string;
}

/**
 * Sensor Alert
 */
export interface SensorAlert {
  alertId: string;
  configId: string;
  sensorId: string;
  timestamp: string;
  metric: SensorType;
  value: number;
  threshold: number | [number, number];
  message: string;
  acknowledged?: boolean;
  acknowledgedAt?: string;
  acknowledgedBy?: string;
}

/**
 * Sensor Network
 */
export interface SensorNetwork {
  networkId: string;
  name: string;
  fieldId?: string;
  sensors: string[]; // Sensor IDs
  gateway?: GatewayInfo;
  topology?: 'star' | 'mesh' | 'hybrid';
  createdAt: string;
  metadata?: Record<string, any>;
}

/**
 * Gateway Information
 */
export interface GatewayInfo {
  gatewayId: string;
  model?: string;
  location?: Location;
  firmwareVersion?: string;
  status: 'online' | 'offline';
  connectedSensors?: number;
}

/**
 * Irrigation Recommendation
 */
export interface IrrigationRecommendation {
  recommendationId: string;
  sensorId: string;
  timestamp: string;
  currentMoisture: number;
  targetMoisture: number;
  recommendedAmount: number; // mm or liters
  priority: 'low' | 'medium' | 'high' | 'critical';
  reason: string;
  validUntil?: string;
}

/**
 * Sensor Maintenance
 */
export interface SensorMaintenance {
  maintenanceId: string;
  sensorId: string;
  date: string;
  type: 'cleaning' | 'calibration' | 'battery_replacement' | 'repair' | 'inspection';
  description: string;
  performedBy?: string;
  nextMaintenanceDate?: string;
  cost?: number;
}

/**
 * Data Export Request
 */
export interface DataExportRequest {
  sensorIds: string[];
  startDate: string;
  endDate: string;
  format: 'csv' | 'json' | 'excel';
  metrics?: SensorType[];
  includeMetadata?: boolean;
}

/**
 * Data Export Response
 */
export interface DataExportResponse {
  exportId: string;
  status: 'processing' | 'completed' | 'failed';
  downloadUrl?: string;
  expiresAt?: string;
  recordCount?: number;
  fileSize?: number; // bytes
}

/**
 * SDK Configuration
 */
export interface SDKConfig {
  baseURL?: string;
  apiKey?: string;
  sensorId?: string;
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
  fieldId?: string;
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
