/**
 * WIA-AGRI-006 Smart Irrigation Standard - TypeScript Type Definitions
 * Version: 1.0.0
 * Philosophy: 弘益人間 (Benefit All Humanity)
 */

/**
 * WIA Smart Irrigation Standard Version
 */
export type WIAVersion = '1.0' | '1.1' | '2.0';

/**
 * Irrigation Methods
 */
export type IrrigationMethod =
  | 'drip'
  | 'sprinkler'
  | 'center_pivot'
  | 'surface'
  | 'subsurface'
  | 'micro_sprinkler';

/**
 * System Status
 */
export type SystemStatus = 'idle' | 'running' | 'scheduled' | 'error' | 'maintenance';

/**
 * Schedule Type
 */
export type ScheduleType = 'time_based' | 'sensor_based' | 'et_based' | 'manual';

/**
 * Water Source
 */
export type WaterSource = 'well' | 'reservoir' | 'river' | 'municipal' | 'recycled';

/**
 * Smart Irrigation Configuration
 */
export interface SmartIrrigationConfig {
  systemId: string;
  name: string;
  fieldId?: string;
  location: Location;
  method: IrrigationMethod;
  waterSource: WaterSource;
  coverage: number; // hectares
  flowRate: number; // liters/minute
  pressure: number; // bar
  zones: IrrigationZone[];
  controllers: Controller[];
  status: SystemStatus;
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
}

/**
 * Irrigation Zone
 */
export interface IrrigationZone {
  zoneId: string;
  name: string;
  area: number; // hectares
  cropType?: string;
  soilType?: string;
  emitters?: number;
  emitterFlow?: number; // liters/hour per emitter
  status: 'active' | 'inactive' | 'error';
  valveId?: string;
  metadata?: Record<string, any>;
}

/**
 * Irrigation Controller
 */
export interface Controller {
  controllerId: string;
  name: string;
  type: string;
  firmware?: string;
  ipAddress?: string;
  zones: string[]; // Zone IDs
  status: 'online' | 'offline' | 'error';
  lastSeen?: string;
}

/**
 * Irrigation Schedule
 */
export interface IrrigationSchedule {
  scheduleId: string;
  systemId: string;
  zoneId?: string;
  name: string;
  type: ScheduleType;
  enabled: boolean;
  startDate?: string;
  endDate?: string;
  timing: ScheduleTiming;
  duration?: number; // minutes
  waterAmount?: number; // mm or liters
  conditions?: ScheduleCondition[];
  metadata?: Record<string, any>;
}

/**
 * Schedule Timing
 */
export interface ScheduleTiming {
  type: 'daily' | 'weekly' | 'interval' | 'cron';
  time?: string; // HH:mm
  daysOfWeek?: number[]; // 0-6
  intervalDays?: number;
  cronExpression?: string;
}

/**
 * Schedule Condition
 */
export interface ScheduleCondition {
  type: 'moisture' | 'temperature' | 'rainfall' | 'evapotranspiration';
  operator: 'gt' | 'lt' | 'gte' | 'lte' | 'eq';
  value: number;
  sensor?: string;
}

/**
 * Irrigation Event
 */
export interface IrrigationEvent {
  eventId: string;
  systemId: string;
  zoneId?: string;
  startTime: string;
  endTime?: string;
  duration?: number; // minutes
  waterUsed?: number; // liters or m³
  method: 'scheduled' | 'manual' | 'automatic';
  triggeredBy?: string;
  status: 'active' | 'completed' | 'cancelled' | 'failed';
  notes?: string;
  metadata?: Record<string, any>;
}

/**
 * Water Meter Reading
 */
export interface WaterMeterReading {
  readingId: string;
  systemId: string;
  timestamp: string;
  totalVolume: number; // m³ or liters
  flowRate: number; // liters/minute
  pressure?: number; // bar
  quality?: WaterQuality;
}

/**
 * Water Quality
 */
export interface WaterQuality {
  ph?: number;
  ec?: number; // Electrical Conductivity
  temperature?: number;
  turbidity?: number;
  tds?: number; // Total Dissolved Solids
  salinity?: number;
}

/**
 * Soil Moisture Data
 */
export interface SoilMoistureData {
  dataId: string;
  zoneId: string;
  timestamp: string;
  moisture: number; // percentage or volumetric
  depth: number; // cm
  temperature?: number;
  sensorId?: string;
}

/**
 * Evapotranspiration Data
 */
export interface EvapotranspirationData {
  dataId: string;
  location: Location;
  date: string;
  eto?: number; // Reference ET in mm
  etc?: number; // Crop ET in mm
  kc?: number; // Crop coefficient
  method?: string; // e.g., "Penman-Monteith"
  weatherData?: WeatherData;
}

/**
 * Weather Data
 */
export interface WeatherData {
  temperature: number;
  humidity: number;
  windSpeed: number;
  solarRadiation?: number;
  rainfall?: number;
}

/**
 * Irrigation Recommendation
 */
export interface IrrigationRecommendation {
  recommendationId: string;
  systemId: string;
  zoneId?: string;
  timestamp: string;
  recommendedAmount: number; // mm or liters
  recommendedDuration?: number; // minutes
  priority: 'low' | 'medium' | 'high' | 'critical';
  reason: string;
  factors: RecommendationFactor[];
  validUntil?: string;
}

/**
 * Recommendation Factor
 */
export interface RecommendationFactor {
  factor: string;
  value: number;
  impact: 'positive' | 'negative';
  weight?: number;
}

/**
 * Valve Control
 */
export interface ValveControl {
  valveId: string;
  name: string;
  zoneId: string;
  status: 'open' | 'closed' | 'partial';
  position?: number; // 0-100 percentage
  type: 'solenoid' | 'motorized' | 'manual';
  lastAction?: string;
  flowRate?: number;
}

/**
 * Pump Control
 */
export interface PumpControl {
  pumpId: string;
  name: string;
  status: 'on' | 'off' | 'standby';
  flowRate?: number; // liters/minute
  pressure?: number; // bar
  power?: number; // watts
  frequency?: number; // Hz for VFD pumps
  runtime?: number; // hours
  efficiency?: number; // percentage
}

/**
 * Fertigation Record
 */
export interface FertigationRecord {
  recordId: string;
  systemId: string;
  zoneId?: string;
  timestamp: string;
  fertilizer: string;
  concentration: number; // ppm or percentage
  volume: number; // liters
  duration: number; // minutes
  ec?: number;
  ph?: number;
}

/**
 * System Maintenance
 */
export interface SystemMaintenance {
  maintenanceId: string;
  systemId: string;
  date: string;
  type: 'inspection' | 'cleaning' | 'repair' | 'filter_change' | 'calibration';
  description: string;
  components?: string[];
  technician?: string;
  cost?: number;
  nextMaintenance?: string;
}

/**
 * Water Usage Analytics
 */
export interface WaterUsageAnalytics {
  systemId: string;
  period: string;
  totalWaterUsed: number; // m³
  averageDaily: number;
  byZone: Record<string, number>;
  efficiency?: number; // percentage
  cost?: number;
  savings?: number;
  trends?: TrendData[];
}

/**
 * Trend Data
 */
export interface TrendData {
  metric: string;
  values: number[];
  timestamps: string[];
  trend: 'increasing' | 'stable' | 'decreasing';
}

/**
 * Alert Configuration
 */
export interface AlertConfig {
  alertId: string;
  systemId: string;
  name: string;
  type: 'leak' | 'low_pressure' | 'high_pressure' | 'low_flow' | 'valve_error';
  threshold?: number;
  enabled: boolean;
  notification: NotificationConfig;
}

/**
 * Notification Configuration
 */
export interface NotificationConfig {
  channels: ('email' | 'sms' | 'push' | 'webhook')[];
  recipients?: string[];
  webhookUrl?: string;
}

/**
 * System Alert
 */
export interface SystemAlert {
  alertId: string;
  systemId: string;
  type: string;
  severity: 'info' | 'warning' | 'critical';
  message: string;
  timestamp: string;
  acknowledged?: boolean;
  resolvedAt?: string;
}

/**
 * SDK Configuration
 */
export interface SDKConfig {
  baseURL?: string;
  apiKey?: string;
  systemId?: string;
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
  zoneId?: string;
  status?: string;
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
