/**
 * WIA Air Quality Standard - Type Definitions
 *
 * @packageDocumentation
 * @module wia-air-quality
 */

/**
 * Pollutant types
 */
export enum Pollutant {
  PM25 = 'pm25',
  PM10 = 'pm10',
  Ozone = 'o3',
  NitrogenDioxide = 'no2',
  SulfurDioxide = 'so2',
  CarbonMonoxide = 'co',
  Lead = 'pb',
  VOC = 'voc',
  Ammonia = 'nh3',
  Methane = 'ch4'
}

/**
 * AQI standard
 */
export enum AQIStandard {
  US_EPA = 'us_epa',
  EU_CAQI = 'eu_caqi',
  China_MEE = 'china_mee',
  India_CPCB = 'india_cpcb',
  Korea_ME = 'korea_me',
  Japan_MOE = 'japan_moe',
  WHO = 'who'
}

/**
 * AQI category
 */
export enum AQICategory {
  Good = 'good',
  Moderate = 'moderate',
  UnhealthySensitive = 'unhealthy_sensitive',
  Unhealthy = 'unhealthy',
  VeryUnhealthy = 'very_unhealthy',
  Hazardous = 'hazardous'
}

/**
 * Sensor type
 */
export enum SensorType {
  Electrochemical = 'electrochemical',
  Optical = 'optical',
  MetalOxide = 'metal_oxide',
  PID = 'pid',
  NDIR = 'ndir',
  LaserScattering = 'laser_scattering',
  BetaAttenuation = 'beta_attenuation',
  TEOM = 'teom'
}

/**
 * Data quality level
 */
export enum DataQuality {
  Research = 'research',
  Regulatory = 'regulatory',
  Indicative = 'indicative',
  Consumer = 'consumer'
}

/**
 * Geographic location
 */
export interface GeoLocation {
  /** Latitude */
  latitude: number;
  /** Longitude */
  longitude: number;
  /** Altitude in meters */
  altitude?: number;
  /** Location name */
  name?: string;
}

/**
 * Pollutant concentration
 */
export interface PollutantConcentration {
  /** Pollutant type */
  pollutant: Pollutant;
  /** Concentration value */
  value: number;
  /** Unit */
  unit: 'ug/m3' | 'ppm' | 'ppb' | 'mg/m3';
  /** Averaging period in minutes */
  averagingPeriod: number;
  /** Measurement timestamp */
  timestamp: Date;
  /** Data quality flag */
  quality: DataQuality;
}

/**
 * AQI breakpoint
 */
export interface AQIBreakpoint {
  /** Pollutant */
  pollutant: Pollutant;
  /** AQI low */
  aqiLow: number;
  /** AQI high */
  aqiHigh: number;
  /** Concentration low */
  concLow: number;
  /** Concentration high */
  concHigh: number;
}

/**
 * AQI result
 */
export interface AQIResult {
  /** Overall AQI */
  aqi: number;
  /** Category */
  category: AQICategory;
  /** Dominant pollutant */
  dominantPollutant: Pollutant;
  /** Individual pollutant AQIs */
  pollutantAQIs: PollutantAQI[];
  /** AQI standard used */
  standard: AQIStandard;
  /** Calculation timestamp */
  timestamp: Date;
}

/**
 * Individual pollutant AQI
 */
export interface PollutantAQI {
  /** Pollutant */
  pollutant: Pollutant;
  /** AQI value */
  aqi: number;
  /** Concentration */
  concentration: number;
  /** Unit */
  unit: string;
}

/**
 * Air quality sensor
 */
export interface AQSensor {
  /** Sensor ID */
  id: string;
  /** Sensor name */
  name: string;
  /** Sensor type */
  type: SensorType;
  /** Pollutants measured */
  pollutants: Pollutant[];
  /** Location */
  location: GeoLocation;
  /** Data quality level */
  dataQuality: DataQuality;
  /** Sampling interval in seconds */
  samplingInterval: number;
  /** Accuracy specification */
  accuracy?: SensorAccuracy;
  /** Calibration info */
  calibration?: CalibrationInfo;
  /** Is sensor active */
  active: boolean;
  /** Last reading timestamp */
  lastReadingAt?: Date;
}

/**
 * Sensor accuracy
 */
export interface SensorAccuracy {
  /** Absolute error */
  absoluteError?: number;
  /** Relative error in percent */
  relativeError?: number;
  /** Detection limit */
  detectionLimit: number;
  /** Measurement range */
  range: [number, number];
}

/**
 * Calibration info
 */
export interface CalibrationInfo {
  /** Last calibration date */
  lastCalibrationDate: Date;
  /** Next calibration date */
  nextCalibrationDate: Date;
  /** Calibration method */
  method: 'zero_span' | 'multipoint' | 'reference' | 'factory';
  /** Calibration status */
  status: 'valid' | 'expired' | 'pending';
}

/**
 * Air quality reading
 */
export interface AQReading {
  /** Reading ID */
  id: string;
  /** Sensor ID */
  sensorId: string;
  /** Location */
  location: GeoLocation;
  /** Pollutant concentrations */
  concentrations: PollutantConcentration[];
  /** Meteorological data */
  meteorology?: MeteorologicalData;
  /** Reading timestamp */
  timestamp: Date;
  /** Is valid reading */
  valid: boolean;
  /** Validation flags */
  flags?: string[];
}

/**
 * Meteorological data
 */
export interface MeteorologicalData {
  /** Temperature in Celsius */
  temperature: number;
  /** Relative humidity in percent */
  humidity: number;
  /** Pressure in hPa */
  pressure: number;
  /** Wind speed in m/s */
  windSpeed?: number;
  /** Wind direction in degrees */
  windDirection?: number;
  /** Rainfall in mm */
  rainfall?: number;
  /** UV index */
  uvIndex?: number;
}

/**
 * Air quality forecast
 */
export interface AQForecast {
  /** Forecast ID */
  id: string;
  /** Location */
  location: GeoLocation;
  /** Forecast hours */
  forecasts: HourlyForecast[];
  /** Model used */
  model: string;
  /** Generated at */
  generatedAt: Date;
  /** Valid until */
  validUntil: Date;
}

/**
 * Hourly forecast
 */
export interface HourlyForecast {
  /** Hour */
  hour: Date;
  /** Predicted AQI */
  aqi: number;
  /** Category */
  category: AQICategory;
  /** Predicted concentrations */
  concentrations: PollutantConcentration[];
  /** Confidence (0-1) */
  confidence: number;
}

/**
 * Health recommendation
 */
export interface HealthRecommendation {
  /** AQI category */
  category: AQICategory;
  /** General public message */
  generalMessage: string;
  /** Sensitive groups message */
  sensitiveGroupsMessage: string;
  /** Outdoor activity guidance */
  outdoorActivity: 'unrestricted' | 'moderate' | 'reduce' | 'avoid' | 'stay_indoor';
  /** Mask recommendation */
  maskRecommendation: 'not_needed' | 'optional' | 'recommended' | 'required';
}

/**
 * Alert definition
 */
export interface AQAlert {
  /** Alert ID */
  id: string;
  /** Alert type */
  type: 'threshold' | 'forecast' | 'trend' | 'health';
  /** Severity */
  severity: 'info' | 'warning' | 'alert' | 'emergency';
  /** Pollutant */
  pollutant?: Pollutant;
  /** Threshold value */
  threshold?: number;
  /** Current value */
  currentValue?: number;
  /** Affected area */
  affectedArea: GeoLocation;
  /** Radius in km */
  radiusKm: number;
  /** Message */
  message: string;
  /** Created at */
  createdAt: Date;
  /** Expires at */
  expiresAt: Date;
  /** Is active */
  active: boolean;
}

/**
 * Station definition
 */
export interface MonitoringStation {
  /** Station ID */
  id: string;
  /** Station name */
  name: string;
  /** Location */
  location: GeoLocation;
  /** Station type */
  type: 'fixed' | 'mobile' | 'personal' | 'reference';
  /** Sensors */
  sensors: AQSensor[];
  /** Operating agency */
  agency?: string;
  /** Data quality level */
  dataQuality: DataQuality;
  /** Is station active */
  active: boolean;
}

/**
 * Historical query
 */
export interface HistoricalQuery {
  /** Station ID */
  stationId?: string;
  /** Location */
  location?: GeoLocation;
  /** Radius in km */
  radiusKm?: number;
  /** Start time */
  startTime: Date;
  /** End time */
  endTime: Date;
  /** Pollutants */
  pollutants?: Pollutant[];
  /** Aggregation */
  aggregation?: 'hourly' | 'daily' | 'monthly';
}

/**
 * Statistics
 */
export interface AQStatistics {
  /** Station ID */
  stationId: string;
  /** Pollutant */
  pollutant: Pollutant;
  /** Period start */
  periodStart: Date;
  /** Period end */
  periodEnd: Date;
  /** Mean */
  mean: number;
  /** Median */
  median: number;
  /** Min */
  min: number;
  /** Max */
  max: number;
  /** Standard deviation */
  stdDev: number;
  /** 95th percentile */
  percentile95: number;
  /** Valid data percentage */
  dataCompleteness: number;
}

/**
 * SDK configuration
 */
export interface AirQualityConfig {
  /** AQI standard to use */
  aqiStandard: AQIStandard;
  /** Data refresh interval in seconds */
  refreshInterval: number;
  /** Enable alerts */
  alertsEnabled: boolean;
  /** Alert thresholds */
  alertThresholds?: AlertThreshold[];
  /** API endpoint */
  apiEndpoint?: string;
  /** API key */
  apiKey?: string;
}

/**
 * Alert threshold
 */
export interface AlertThreshold {
  /** Pollutant */
  pollutant: Pollutant;
  /** Warning threshold */
  warningThreshold: number;
  /** Alert threshold */
  alertThreshold: number;
  /** Emergency threshold */
  emergencyThreshold: number;
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
  standard: 'WIA-AIR-QUALITY';
  /** Test date */
  testDate: string;
  /** Configuration */
  config: AirQualityConfig;
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
