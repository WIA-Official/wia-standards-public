/**
 * WIA-AGRI-001 Smart Farm Standard - TypeScript Type Definitions
 * Version: 1.0.0
 * Philosophy: 弘益人間 (Benefit All Humanity)
 */

/**
 * WIA Smart Farm Standard Version
 */
export type WIAVersion = '1.0' | '1.1' | '2.0';

/**
 * Farm Types
 */
export type FarmType =
  | 'crop'
  | 'livestock'
  | 'mixed'
  | 'aquaculture'
  | 'greenhouse'
  | 'vertical'
  | 'organic'
  | 'conventional';

/**
 * Sensor Types
 */
export type SensorType =
  | 'temperature'
  | 'humidity'
  | 'soil_moisture'
  | 'light'
  | 'co2'
  | 'ph'
  | 'ec'
  | 'camera'
  | 'weather';

/**
 * Device Status
 */
export type DeviceStatus = 'online' | 'offline' | 'maintenance' | 'error';

/**
 * Alert Severity
 */
export type AlertSeverity = 'info' | 'warning' | 'critical' | 'emergency';

/**
 * Automation Rule Status
 */
export type RuleStatus = 'active' | 'inactive' | 'paused';

/**
 * Smart Farm Configuration
 */
export interface SmartFarmConfig {
  farmId: string;
  name: string;
  type: FarmType;
  location: Location;
  area: number; // in square meters
  timezone: string;
  owner: FarmOwner;
  metadata?: Record<string, any>;
}

/**
 * Geographic Location
 */
export interface Location {
  latitude: number;
  longitude: number;
  altitude?: number;
  address?: string;
  region?: string;
  country: string;
}

/**
 * Farm Owner Information
 */
export interface FarmOwner {
  id: string;
  name: string;
  email?: string;
  phone?: string;
  organization?: string;
}

/**
 * Smart Farm Device
 */
export interface SmartFarmDevice {
  deviceId: string;
  name: string;
  type: string;
  sensorType?: SensorType;
  status: DeviceStatus;
  location: DeviceLocation;
  firmware?: string;
  lastSeen?: string;
  metadata?: Record<string, any>;
}

/**
 * Device Location within Farm
 */
export interface DeviceLocation {
  zone?: string;
  section?: string;
  coordinates?: {
    x: number;
    y: number;
    z?: number;
  };
}

/**
 * Sensor Reading
 */
export interface SensorReading {
  deviceId: string;
  timestamp: string;
  sensorType: SensorType;
  value: number;
  unit: string;
  quality?: number; // 0-100
  metadata?: Record<string, any>;
}

/**
 * Environmental Data
 */
export interface EnvironmentalData {
  farmId: string;
  timestamp: string;
  temperature?: number; // Celsius
  humidity?: number; // Percentage
  soilMoisture?: number; // Percentage
  lightIntensity?: number; // Lux
  co2Level?: number; // ppm
  phLevel?: number;
  ecLevel?: number; // Electrical Conductivity
  rainfall?: number; // mm
  windSpeed?: number; // m/s
}

/**
 * Farm Alert
 */
export interface FarmAlert {
  alertId: string;
  farmId: string;
  deviceId?: string;
  severity: AlertSeverity;
  type: string;
  message: string;
  timestamp: string;
  acknowledged?: boolean;
  acknowledgedAt?: string;
  acknowledgedBy?: string;
  resolved?: boolean;
  resolvedAt?: string;
}

/**
 * Automation Rule
 */
export interface AutomationRule {
  ruleId: string;
  name: string;
  farmId: string;
  status: RuleStatus;
  condition: RuleCondition;
  action: RuleAction;
  schedule?: Schedule;
  createdAt: string;
  updatedAt?: string;
}

/**
 * Automation Rule Condition
 */
export interface RuleCondition {
  type: 'sensor' | 'time' | 'composite';
  sensorType?: SensorType;
  operator?: 'gt' | 'lt' | 'eq' | 'gte' | 'lte' | 'between';
  value?: number | [number, number];
  timeRange?: TimeRange;
  conditions?: RuleCondition[]; // For composite conditions
  logic?: 'and' | 'or';
}

/**
 * Automation Rule Action
 */
export interface RuleAction {
  type: 'device_control' | 'notification' | 'data_log' | 'composite';
  deviceId?: string;
  command?: string;
  parameters?: Record<string, any>;
  notificationChannel?: 'email' | 'sms' | 'push' | 'webhook';
  notificationTarget?: string;
  actions?: RuleAction[]; // For composite actions
}

/**
 * Time Range
 */
export interface TimeRange {
  start: string; // ISO 8601 or time like "08:00"
  end: string;
}

/**
 * Schedule
 */
export interface Schedule {
  type: 'once' | 'daily' | 'weekly' | 'monthly' | 'cron';
  startDate?: string;
  endDate?: string;
  daysOfWeek?: number[]; // 0-6, Sunday = 0
  cronExpression?: string;
}

/**
 * Device Control Command
 */
export interface DeviceControlCommand {
  deviceId: string;
  command: string;
  parameters?: Record<string, any>;
  timestamp?: string;
  userId?: string;
}

/**
 * Device Control Response
 */
export interface DeviceControlResponse {
  deviceId: string;
  command: string;
  status: 'success' | 'failed' | 'pending';
  message?: string;
  timestamp: string;
}

/**
 * Farm Analytics
 */
export interface FarmAnalytics {
  farmId: string;
  period: string;
  metrics: FarmMetrics;
  trends?: TrendData[];
  predictions?: PredictionData[];
}

/**
 * Farm Metrics
 */
export interface FarmMetrics {
  averageTemperature?: number;
  averageHumidity?: number;
  averageSoilMoisture?: number;
  totalRainfall?: number;
  activeDevices: number;
  totalDevices: number;
  alertsCount: number;
  criticalAlertsCount: number;
}

/**
 * Trend Data
 */
export interface TrendData {
  metric: string;
  values: number[];
  timestamps: string[];
  trend: 'up' | 'down' | 'stable';
  changePercentage?: number;
}

/**
 * Prediction Data
 */
export interface PredictionData {
  metric: string;
  predictedValue: number;
  confidence: number; // 0-100
  timeHorizon: string; // e.g., "24h", "7d"
  timestamp: string;
}

/**
 * Farm Zone
 */
export interface FarmZone {
  zoneId: string;
  farmId: string;
  name: string;
  type: string;
  area?: number;
  coordinates?: Polygon;
  metadata?: Record<string, any>;
}

/**
 * Polygon for zone boundaries
 */
export interface Polygon {
  type: 'Polygon';
  coordinates: [number, number][][]; // GeoJSON format
}

/**
 * Harvest Data
 */
export interface HarvestData {
  harvestId: string;
  farmId: string;
  zoneId?: string;
  cropType: string;
  quantity: number;
  unit: string;
  quality?: string;
  harvestDate: string;
  notes?: string;
}

/**
 * SDK Configuration
 */
export interface SDKConfig {
  baseURL?: string;
  apiKey?: string;
  farmId?: string;
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
