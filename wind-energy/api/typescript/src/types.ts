/**
 * WIA-ENE-006 Wind Energy TypeScript SDK - Type Definitions
 *
 * @module @wia/ene-006
 * @version 1.0.0
 * @license MIT
 * @copyright 2025 SmileStory Inc. / WIA
 * @philosophy 弘益人間 (홍익인간) - Benefit All Humanity
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Wind turbine operating states
 */
export enum TurbineState {
  STANDBY = 'standby',
  STARTING = 'starting',
  RUNNING = 'running',
  STOPPING = 'stopping',
  FAULT = 'fault',
  MAINTENANCE = 'maintenance',
  EMERGENCY_STOP = 'emergency_stop'
}

/**
 * Alert severity levels
 */
export enum AlertSeverity {
  INFO = 'info',
  WARNING = 'warning',
  CRITICAL = 'critical'
}

/**
 * Turbine types
 */
export enum TurbineType {
  HAWT = 'horizontal_axis',      // Horizontal Axis Wind Turbine
  VAWT = 'vertical_axis',         // Vertical Axis Wind Turbine
  OFFSHORE = 'offshore',
  FLOATING = 'floating',
  SMALL = 'small'                 // < 100 kW
}

/**
 * Generator types
 */
export enum GeneratorType {
  DFIG = 'doubly_fed_induction',
  PMSG = 'permanent_magnet_synchronous',
  SCIG = 'squirrel_cage_induction'
}

// ============================================================================
// Configuration Types
// ============================================================================

/**
 * Turbine configuration
 */
export interface TurbineConfig {
  /** Unique turbine identifier */
  turbineId: string;

  /** Turbine type */
  type: TurbineType;

  /** Manufacturer name */
  manufacturer: string;

  /** Model designation */
  model: string;

  /** Rated power (kW) */
  ratedPower: number;

  /** Rotor diameter (meters) */
  rotorDiameter: number;

  /** Hub height (meters) */
  hubHeight: number;

  /** Cut-in wind speed (m/s) */
  cutInSpeed: number;

  /** Rated wind speed (m/s) */
  ratedSpeed: number;

  /** Cut-out wind speed (m/s) */
  cutOutSpeed: number;

  /** Generator type */
  generatorType: GeneratorType;

  /** Installation date */
  installationDate: string;

  /** Geographic location */
  location: GeographicLocation;
}

/**
 * Geographic location
 */
export interface GeographicLocation {
  latitude: number;
  longitude: number;
  elevation: number;    // meters above sea level
  timezone: string;     // IANA timezone identifier
}

// ============================================================================
// Operational Data Types
// ============================================================================

/**
 * Real-time turbine data packet
 */
export interface TurbineDataPacket {
  /** ISO 8601 timestamp */
  timestamp: string;

  /** Turbine identifier */
  turbineId: string;

  /** Meteorological data */
  meteorological: MeteorologicalData;

  /** Power and performance */
  power: PowerData;

  /** Mechanical status */
  mechanical: MechanicalData;

  /** Health and status */
  status: StatusData;

  /** Philosophy marker */
  philosophy: '弘益人間';
}

/**
 * Meteorological data
 */
export interface MeteorologicalData {
  /** Wind speed (m/s) */
  windSpeed: number;

  /** Wind direction (degrees, 0-360) */
  windDirection: number;

  /** Ambient temperature (Celsius) */
  temperature: number;

  /** Atmospheric pressure (hPa) */
  pressure: number;

  /** Relative humidity (%) */
  humidity?: number;

  /** Wind speed at nacelle (m/s) */
  nacelleWindSpeed?: number;
}

/**
 * Power data
 */
export interface PowerData {
  /** Active power (kW) */
  activePower: number;

  /** Reactive power (kVAR) */
  reactivePower: number;

  /** Power factor (-1 to 1) */
  powerFactor: number;

  /** Grid frequency (Hz) */
  frequency: number;

  /** Grid voltage (V) */
  voltage: number;

  /** Current (A) */
  current: number;

  /** Daily energy production (kWh) */
  dailyEnergy: number;

  /** Total lifetime energy (MWh) */
  lifetimeEnergy: number;
}

/**
 * Mechanical data
 */
export interface MechanicalData {
  /** Rotor speed (RPM) */
  rotorSpeed: number;

  /** Generator speed (RPM) */
  generatorSpeed: number;

  /** Blade 1 pitch angle (degrees) */
  pitchAngle1: number;

  /** Blade 2 pitch angle (degrees) */
  pitchAngle2: number;

  /** Blade 3 pitch angle (degrees) */
  pitchAngle3: number;

  /** Yaw angle (degrees) */
  yawAngle: number;

  /** Gearbox oil temperature (Celsius) */
  gearboxTemp: number;

  /** Generator winding temperature (Celsius) */
  generatorTemp: number;

  /** Main bearing temperature (Celsius) */
  bearingTemp: number;
}

/**
 * Status data
 */
export interface StatusData {
  /** Operating state */
  state: TurbineState;

  /** Active alerts */
  alerts: Alert[];

  /** Availability (percentage) */
  availability: number;

  /** Operating hours */
  operatingHours: number;

  /** Grid connection status */
  gridConnected: boolean;

  /** Remote control enabled */
  remoteControlEnabled: boolean;
}

/**
 * Alert information
 */
export interface Alert {
  /** Alert code */
  code: string;

  /** Severity level */
  severity: AlertSeverity;

  /** Alert message */
  message: string;

  /** Timestamp when alert was raised */
  timestamp: string;

  /** Acknowledged flag */
  acknowledged: boolean;
}

// ============================================================================
// Wind Farm Types
// ============================================================================

/**
 * Wind farm configuration
 */
export interface WindFarmConfig {
  /** Unique wind farm identifier */
  windFarmId: string;

  /** Wind farm name */
  name: string;

  /** Total rated capacity (MW) */
  totalCapacity: number;

  /** Number of turbines */
  turbineCount: number;

  /** List of turbines */
  turbines: TurbineConfig[];

  /** Geographic location (center point) */
  location: GeographicLocation;

  /** Commissioning date */
  commissioningDate: string;
}

/**
 * Wind farm status
 */
export interface WindFarmStatus {
  /** Wind farm identifier */
  windFarmId: string;

  /** Timestamp */
  timestamp: string;

  /** Aggregate metrics */
  metrics: WindFarmMetrics;

  /** Individual turbine statuses */
  turbines: Map<string, TurbineDataPacket>;

  /** Active wind farm-level alerts */
  alerts: Alert[];
}

/**
 * Wind farm aggregate metrics
 */
export interface WindFarmMetrics {
  /** Total power output (kW) */
  totalPower: number;

  /** Number of turbines running */
  turbinesRunning: number;

  /** Number of turbines available */
  turbinesAvailable: number;

  /** Overall availability (%) */
  availability: number;

  /** Average wind speed (m/s) */
  avgWindSpeed: number;

  /** System efficiency (%) */
  systemEfficiency: number;

  /** Capacity factor (%) */
  capacityFactor: number;

  /** Daily energy production (MWh) */
  dailyEnergy: number;
}

// ============================================================================
// Control Types
// ============================================================================

/**
 * Turbine control command
 */
export interface ControlCommand {
  /** Target turbine ID */
  turbineId: string;

  /** Command type */
  command: TurbineCommand;

  /** Command parameters */
  parameters?: Record<string, any>;

  /** Timestamp */
  timestamp: string;

  /** Requesting user/system */
  requestedBy: string;
}

/**
 * Turbine commands
 */
export enum TurbineCommand {
  START = 'start',
  STOP = 'stop',
  EMERGENCY_STOP = 'emergency_stop',
  RESET = 'reset',
  SET_POWER_SETPOINT = 'set_power_setpoint',
  SET_PITCH_ANGLE = 'set_pitch_angle',
  ENABLE_CURTAILMENT = 'enable_curtailment',
  DISABLE_CURTAILMENT = 'disable_curtailment'
}

/**
 * Control response
 */
export interface ControlResponse {
  /** Success flag */
  success: boolean;

  /** Response message */
  message: string;

  /** Response timestamp */
  timestamp: string;

  /** Execution time (ms) */
  executionTime: number;
}

// ============================================================================
// Analytics Types
// ============================================================================

/**
 * Performance analysis request
 */
export interface PerformanceAnalysisRequest {
  /** Turbine or wind farm ID */
  targetId: string;

  /** Start time (ISO 8601) */
  startTime: string;

  /** End time (ISO 8601) */
  endTime: string;

  /** Metrics to analyze */
  metrics: string[];

  /** Aggregation interval (minutes) */
  interval?: number;
}

/**
 * Performance analysis result
 */
export interface PerformanceAnalysisResult {
  /** Target ID */
  targetId: string;

  /** Analysis period */
  period: {
    start: string;
    end: string;
  };

  /** Computed metrics */
  metrics: Record<string, number>;

  /** Time-series data */
  timeSeries: TimeSeriesData[];

  /** Recommendations */
  recommendations: string[];
}

/**
 * Time-series data point
 */
export interface TimeSeriesData {
  timestamp: string;
  values: Record<string, number>;
}

// ============================================================================
// Forecasting Types
// ============================================================================

/**
 * Production forecast request
 */
export interface ForecastRequest {
  /** Wind farm or turbine ID */
  targetId: string;

  /** Forecast horizon (hours) */
  horizon: number;

  /** Forecast model (optional) */
  model?: string;
}

/**
 * Production forecast result
 */
export interface ForecastResult {
  /** Target ID */
  targetId: string;

  /** Forecast generated timestamp */
  generatedAt: string;

  /** Forecast horizon (hours) */
  horizon: number;

  /** Hourly forecasts */
  forecasts: HourlyForecast[];

  /** Model used */
  model: string;

  /** Confidence level */
  confidence: number;
}

/**
 * Hourly forecast data
 */
export interface HourlyForecast {
  /** Hour offset from current time */
  hour: number;

  /** Timestamp */
  timestamp: string;

  /** Predicted power output (kW) */
  power: number;

  /** Predicted wind speed (m/s) */
  windSpeed: number;

  /** Uncertainty range */
  uncertainty: {
    lower: number;  // kW
    upper: number;  // kW
  };
}

// ============================================================================
// Export Configuration
// ============================================================================

/**
 * Data export configuration
 */
export interface ExportConfig {
  /** Export format */
  format: 'json' | 'csv' | 'xml' | 'excel';

  /** Start time */
  startTime: string;

  /** End time */
  endTime: string;

  /** Fields to include */
  fields: string[];

  /** Compression */
  compress?: boolean;
}

/**
 * Export result
 */
export interface ExportResult {
  /** Export ID */
  exportId: string;

  /** Download URL */
  downloadUrl: string;

  /** File size (bytes) */
  fileSize: number;

  /** Expiration time */
  expiresAt: string;
}

// ============================================================================
// Client Configuration
// ============================================================================

/**
 * WIA-ENE-006 Client configuration
 */
export interface ClientConfig {
  /** API endpoint */
  endpoint: string;

  /** API key */
  apiKey: string;

  /** Timeout (ms) */
  timeout?: number;

  /** Retry configuration */
  retry?: {
    maxRetries: number;
    retryDelay: number;
  };

  /** Enable logging */
  logging?: boolean;
}
