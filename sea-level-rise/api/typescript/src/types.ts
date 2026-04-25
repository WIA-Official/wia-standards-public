/**
 * WIA-ENV-001 Sea Level Rise Monitoring SDK Type Definitions
 *
 * Comprehensive type definitions for coastal monitoring, sea level tracking,
 * and climate change impact assessment systems.
 *
 * @packageDocumentation
 * @module @wia/sea-level-rise
 * @version 1.0.0
 */

// ============================================================================
// Core Identifier Types
// ============================================================================

/** Unique identifier for monitoring stations */
export type StationId = string;

/** Unique identifier for measurement records */
export type MeasurementId = string;

/** Geographic coordinate tuple [latitude, longitude] */
export type Coordinates = [number, number];

/** ISO 8601 formatted timestamp string */
export type ISOTimestamp = string;

// ============================================================================
// Measurement Unit Types
// ============================================================================

/** Sea level measurement units */
export type SeaLevelUnit = 'mm' | 'cm' | 'm' | 'ft' | 'in';

/** Temperature units for water temperature measurements */
export type TemperatureUnit = 'celsius' | 'fahrenheit' | 'kelvin';

/** Pressure units for atmospheric measurements */
export type PressureUnit = 'hPa' | 'mbar' | 'inHg' | 'mmHg';

// ============================================================================
// Station Configuration Types
// ============================================================================

/**
 * Monitoring station configuration and metadata
 */
export interface MonitoringStation {
  /** Unique station identifier */
  id: StationId;

  /** Human-readable station name */
  name: string;

  /** Geographic coordinates */
  location: StationLocation;

  /** Station operational status */
  status: StationStatus;

  /** Installed sensors and equipment */
  equipment: StationEquipment;

  /** Station metadata */
  metadata: StationMetadata;

  /** Data collection configuration */
  dataConfig: DataCollectionConfig;
}

/**
 * Geographic location details for a monitoring station
 */
export interface StationLocation {
  /** Latitude in decimal degrees */
  latitude: number;

  /** Longitude in decimal degrees */
  longitude: number;

  /** Elevation above mean sea level in meters */
  elevation: number;

  /** Country code (ISO 3166-1 alpha-2) */
  countryCode: string;

  /** Region or state name */
  region: string;

  /** Coastal zone classification */
  coastalZone: CoastalZoneType;

  /** Tidal datum reference */
  tidalDatum: TidalDatum;
}

/** Coastal zone classification types */
export type CoastalZoneType =
  | 'beach'
  | 'cliff'
  | 'delta'
  | 'estuary'
  | 'lagoon'
  | 'mangrove'
  | 'marsh'
  | 'reef'
  | 'rocky_shore';

/** Standard tidal datum references */
export type TidalDatum =
  | 'MHHW'  // Mean Higher High Water
  | 'MHW'   // Mean High Water
  | 'MTL'   // Mean Tide Level
  | 'MSL'   // Mean Sea Level
  | 'MLW'   // Mean Low Water
  | 'MLLW' // Mean Lower Low Water
  | 'NAVD88' // North American Vertical Datum 1988
  | 'EGM96'; // Earth Gravitational Model 1996

/** Station operational status */
export type StationStatus =
  | 'active'
  | 'inactive'
  | 'maintenance'
  | 'calibrating'
  | 'offline'
  | 'decommissioned';

/**
 * Equipment installed at a monitoring station
 */
export interface StationEquipment {
  /** Tide gauge specifications */
  tideGauge: TideGaugeSpec;

  /** GPS/GNSS receiver for vertical land motion */
  gnssReceiver?: GNSSReceiverSpec;

  /** Weather sensors */
  weatherSensors?: WeatherSensorSpec[];

  /** Wave measurement equipment */
  waveMonitor?: WaveMonitorSpec;

  /** Communication equipment */
  communication: CommunicationSpec;
}

/**
 * Tide gauge sensor specifications
 */
export interface TideGaugeSpec {
  /** Gauge type */
  type: 'acoustic' | 'pressure' | 'radar' | 'float' | 'bubbler';

  /** Manufacturer name */
  manufacturer: string;

  /** Model identifier */
  model: string;

  /** Measurement accuracy in mm */
  accuracyMm: number;

  /** Measurement range in meters */
  rangeMeter: number;

  /** Sampling rate in Hz */
  samplingRateHz: number;

  /** Last calibration date */
  lastCalibration: ISOTimestamp;

  /** Next scheduled calibration */
  nextCalibration: ISOTimestamp;
}

/**
 * GNSS receiver specifications for vertical land motion measurement
 */
export interface GNSSReceiverSpec {
  /** Receiver manufacturer */
  manufacturer: string;

  /** Receiver model */
  model: string;

  /** Supported satellite systems */
  constellations: ('GPS' | 'GLONASS' | 'Galileo' | 'BeiDou')[];

  /** Vertical accuracy in mm */
  verticalAccuracyMm: number;

  /** Data logging interval in seconds */
  loggingIntervalSec: number;
}

/**
 * Weather sensor specifications
 */
export interface WeatherSensorSpec {
  /** Sensor type */
  type: 'temperature' | 'pressure' | 'humidity' | 'wind' | 'precipitation';

  /** Manufacturer */
  manufacturer: string;

  /** Model */
  model: string;

  /** Measurement accuracy */
  accuracy: number;

  /** Measurement unit */
  unit: string;
}

/**
 * Wave monitoring equipment specifications
 */
export interface WaveMonitorSpec {
  /** Monitor type */
  type: 'buoy' | 'radar' | 'pressure_array' | 'acoustic';

  /** Significant wave height accuracy in cm */
  heightAccuracyCm: number;

  /** Wave period accuracy in seconds */
  periodAccuracySec: number;

  /** Direction accuracy in degrees */
  directionAccuracyDeg: number;
}

/**
 * Communication equipment specifications
 */
export interface CommunicationSpec {
  /** Primary communication method */
  primary: 'satellite' | 'cellular' | 'radio' | 'fiber';

  /** Backup communication method */
  backup?: 'satellite' | 'cellular' | 'radio';

  /** Data transmission interval in minutes */
  transmissionIntervalMin: number;

  /** Real-time streaming capability */
  realTimeStreaming: boolean;
}

/**
 * Station metadata information
 */
export interface StationMetadata {
  /** Station installation date */
  installationDate: ISOTimestamp;

  /** Operating organization */
  operator: string;

  /** Data provider/owner */
  dataProvider: string;

  /** Station documentation URL */
  documentationUrl?: string;

  /** Historical data availability start date */
  dataAvailableFrom: ISOTimestamp;

  /** Quality control level */
  qcLevel: QualityControlLevel;
}

/** Quality control levels for data */
export type QualityControlLevel =
  | 'raw'
  | 'preliminary'
  | 'quality_controlled'
  | 'research_quality'
  | 'verified';

/**
 * Data collection configuration
 */
export interface DataCollectionConfig {
  /** Primary measurement interval in seconds */
  measurementIntervalSec: number;

  /** Data averaging period in minutes */
  averagingPeriodMin: number;

  /** Outlier detection enabled */
  outlierDetection: boolean;

  /** Spike detection threshold in mm */
  spikeThresholdMm: number;

  /** Gap filling method */
  gapFillingMethod: 'linear' | 'spline' | 'nearest' | 'none';
}

// ============================================================================
// Measurement Data Types
// ============================================================================

/**
 * Sea level measurement record
 */
export interface SeaLevelMeasurement {
  /** Unique measurement identifier */
  id: MeasurementId;

  /** Station that recorded the measurement */
  stationId: StationId;

  /** Measurement timestamp */
  timestamp: ISOTimestamp;

  /** Sea level reading */
  seaLevel: SeaLevelReading;

  /** Associated meteorological data */
  meteorology?: MeteorologicalData;

  /** Wave conditions */
  waves?: WaveData;

  /** Quality flags */
  qualityFlags: QualityFlags;

  /** Processing information */
  processing: ProcessingInfo;
}

/**
 * Sea level reading values
 */
export interface SeaLevelReading {
  /** Observed water level relative to datum in mm */
  observedMm: number;

  /** Predicted astronomical tide in mm */
  predictedTideMm: number;

  /** Non-tidal residual (surge) in mm */
  residualMm: number;

  /** Reference datum used */
  datum: TidalDatum;

  /** Measurement uncertainty in mm */
  uncertaintyMm: number;
}

/**
 * Meteorological conditions at time of measurement
 */
export interface MeteorologicalData {
  /** Atmospheric pressure in hPa */
  pressureHPa: number;

  /** Air temperature in Celsius */
  airTempC: number;

  /** Water temperature in Celsius */
  waterTempC: number;

  /** Wind speed in m/s */
  windSpeedMs: number;

  /** Wind direction in degrees from north */
  windDirectionDeg: number;

  /** Relative humidity percentage */
  humidityPercent: number;
}

/**
 * Wave condition data
 */
export interface WaveData {
  /** Significant wave height in meters */
  significantHeightM: number;

  /** Peak wave period in seconds */
  peakPeriodSec: number;

  /** Mean wave direction in degrees */
  meanDirectionDeg: number;

  /** Wave energy density */
  energyDensity: number;
}

/**
 * Quality control flags for measurements
 */
export interface QualityFlags {
  /** Overall quality flag */
  overall: 'good' | 'suspect' | 'bad' | 'missing';

  /** Specific quality issues detected */
  issues: QualityIssue[];

  /** Manual review status */
  reviewed: boolean;

  /** Reviewer notes */
  reviewNotes?: string;
}

/** Types of quality issues */
export type QualityIssue =
  | 'spike'
  | 'flat_line'
  | 'out_of_range'
  | 'sensor_drift'
  | 'data_gap'
  | 'timing_error'
  | 'calibration_needed';

/**
 * Data processing information
 */
export interface ProcessingInfo {
  /** Processing algorithm version */
  algorithmVersion: string;

  /** Processing timestamp */
  processedAt: ISOTimestamp;

  /** Applied corrections */
  corrections: AppliedCorrection[];

  /** Processing level */
  level: 'L0' | 'L1' | 'L2' | 'L3';
}

/**
 * Correction applied during processing
 */
export interface AppliedCorrection {
  /** Correction type */
  type: 'atmospheric' | 'temperature' | 'drift' | 'datum' | 'vlm';

  /** Correction value applied */
  valueMm: number;

  /** Correction method used */
  method: string;
}

// ============================================================================
// Analysis and Trend Types
// ============================================================================

/**
 * Sea level trend analysis results
 */
export interface SeaLevelTrend {
  /** Station identifier */
  stationId: StationId;

  /** Analysis period */
  period: AnalysisPeriod;

  /** Linear trend rate in mm/year */
  linearTrendMmPerYear: number;

  /** Trend uncertainty (95% CI) in mm/year */
  trendUncertainty: number;

  /** Acceleration in mm/year² */
  accelerationMmPerYear2: number;

  /** Vertical land motion rate in mm/year */
  vlmRateMmPerYear: number;

  /** Relative sea level rise (observed - VLM) */
  relativeRiseMmPerYear: number;

  /** Statistical significance */
  significance: TrendSignificance;
}

/**
 * Analysis period definition
 */
export interface AnalysisPeriod {
  /** Start date */
  startDate: ISOTimestamp;

  /** End date */
  endDate: ISOTimestamp;

  /** Number of valid data points */
  dataPoints: number;

  /** Data completeness percentage */
  completenessPercent: number;
}

/**
 * Statistical significance of trend
 */
export interface TrendSignificance {
  /** P-value */
  pValue: number;

  /** Is statistically significant at 95% confidence */
  isSignificant95: boolean;

  /** R-squared value */
  rSquared: number;
}

// ============================================================================
// Projection and Scenario Types
// ============================================================================

/**
 * Sea level rise projection
 */
export interface SeaLevelProjection {
  /** Target year for projection */
  targetYear: number;

  /** Emission scenario */
  scenario: EmissionScenario;

  /** Projected values */
  projection: ProjectionValues;

  /** Uncertainty ranges */
  uncertainty: ProjectionUncertainty;

  /** Contributing factors */
  contributions: ProjectionContributions;
}

/** IPCC emission scenarios */
export type EmissionScenario =
  | 'SSP1-1.9'
  | 'SSP1-2.6'
  | 'SSP2-4.5'
  | 'SSP3-7.0'
  | 'SSP5-8.5';

/**
 * Projected sea level values
 */
export interface ProjectionValues {
  /** Median projection in mm relative to baseline */
  medianMm: number;

  /** 17th percentile (likely range lower bound) */
  p17Mm: number;

  /** 83rd percentile (likely range upper bound) */
  p83Mm: number;

  /** 5th percentile (very likely range lower) */
  p5Mm: number;

  /** 95th percentile (very likely range upper) */
  p95Mm: number;
}

/**
 * Projection uncertainty breakdown
 */
export interface ProjectionUncertainty {
  /** Ice sheet contribution uncertainty */
  iceSheetsMm: number;

  /** Thermal expansion uncertainty */
  thermalExpansionMm: number;

  /** Glacier contribution uncertainty */
  glaciersMm: number;

  /** Regional variability uncertainty */
  regionalMm: number;
}

/**
 * Breakdown of sea level rise contributions
 */
export interface ProjectionContributions {
  /** Thermal expansion contribution in mm */
  thermalExpansionMm: number;

  /** Greenland ice sheet contribution in mm */
  greenlandMm: number;

  /** Antarctic ice sheet contribution in mm */
  antarcticMm: number;

  /** Glaciers and ice caps contribution in mm */
  glaciersMm: number;

  /** Land water storage changes in mm */
  landWaterMm: number;

  /** Glacial isostatic adjustment in mm */
  giaMm: number;
}

// ============================================================================
// Alert and Notification Types
// ============================================================================

/**
 * Sea level alert configuration
 */
export interface AlertConfig {
  /** Alert identifier */
  id: string;

  /** Alert name */
  name: string;

  /** Station to monitor */
  stationId: StationId;

  /** Alert trigger conditions */
  triggers: AlertTrigger[];

  /** Notification settings */
  notifications: NotificationConfig;

  /** Alert enabled status */
  enabled: boolean;
}

/**
 * Alert trigger condition
 */
export interface AlertTrigger {
  /** Trigger type */
  type: 'threshold' | 'rate_of_change' | 'anomaly';

  /** Threshold value in mm (for threshold type) */
  thresholdMm?: number;

  /** Rate threshold in mm/hour (for rate_of_change type) */
  rateMmPerHour?: number;

  /** Standard deviations (for anomaly type) */
  standardDeviations?: number;

  /** Comparison operator */
  operator: 'gt' | 'gte' | 'lt' | 'lte' | 'eq';

  /** Duration in minutes before triggering */
  durationMin: number;
}

/**
 * Notification configuration
 */
export interface NotificationConfig {
  /** Email recipients */
  emailRecipients: string[];

  /** SMS recipients */
  smsRecipients: string[];

  /** Webhook URLs */
  webhookUrls: string[];

  /** Notification cooldown period in minutes */
  cooldownMin: number;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * Base error class for sea level rise SDK
 */
export class WIASeaLevelError extends Error {
  constructor(
    message: string,
    public code: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'WIASeaLevelError';
  }
}

/**
 * Error thrown when station is not found
 */
export class StationNotFoundError extends WIASeaLevelError {
  constructor(stationId: StationId) {
    super(
      `Monitoring station not found: ${stationId}`,
      'STATION_NOT_FOUND',
      { stationId }
    );
    this.name = 'StationNotFoundError';
  }
}

/**
 * Error thrown when measurement data is invalid
 */
export class InvalidMeasurementError extends WIASeaLevelError {
  constructor(field: string, reason: string) {
    super(
      `Invalid measurement data: ${field} - ${reason}`,
      'INVALID_MEASUREMENT',
      { field, reason }
    );
    this.name = 'InvalidMeasurementError';
  }
}

/**
 * Error thrown when data quality check fails
 */
export class QualityCheckFailedError extends WIASeaLevelError {
  constructor(issues: QualityIssue[]) {
    super(
      `Quality check failed: ${issues.join(', ')}`,
      'QUALITY_CHECK_FAILED',
      { issues }
    );
    this.name = 'QualityCheckFailedError';
  }
}
