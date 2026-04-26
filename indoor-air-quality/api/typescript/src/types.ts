/**
 * WIA-ENE-027: Indoor Air Quality Standard - TypeScript Type Definitions
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
  lat: number;
  lon: number;
}

/**
 * Location information
 */
export interface Location {
  lat: number;
  lon: number;
  address?: string;
  city?: string;
  country?: string;
  postalCode?: string;
  floor?: number;
  room?: string;
}

// ============================================================================
// Air Quality Parameters
// ============================================================================

/**
 * Particulate matter (PM) measurements
 */
export interface ParticulateMatter {
  pm25_ugm3: number;        // PM2.5 (μg/m³)
  pm10_ugm3: number;        // PM10 (μg/m³)
  pm1_ugm3?: number;        // PM1.0 (μg/m³)
  timestamp: Timestamp;
  qc_flag: QCFlag;
}

/**
 * Gas concentrations
 */
export interface GasConcentrations {
  co2_ppm: number;          // 이산화탄소 (ppm)
  co_ppm?: number;          // 일산화탄소 (ppm)
  no2_ppb?: number;         // 이산화질소 (ppb)
  o3_ppb?: number;          // 오존 (ppb)
  so2_ppb?: number;         // 이산화황 (ppb)
  timestamp: Timestamp;
  qc_flag: QCFlag;
}

/**
 * Volatile Organic Compounds (VOCs)
 */
export interface VOCMeasurements {
  tvoc_ppb: number;         // 총휘발성유기화합물 (ppb)
  formaldehyde_ppb?: number; // 포름알데히드 (ppb)
  benzene_ppb?: number;     // 벤젠 (ppb)
  toluene_ppb?: number;     // 톨루엔 (ppb)
  xylene_ppb?: number;      // 자일렌 (ppb)
  timestamp: Timestamp;
  qc_flag: QCFlag;
}

/**
 * Radon measurement
 */
export interface RadonMeasurement {
  radon_bqm3: number;       // 라돈 (Bq/m³)
  radon_pCiL?: number;      // 라돈 (pCi/L)
  averagingPeriod: '24h' | '7d' | '30d';
  timestamp: Timestamp;
  qc_flag: QCFlag;
}

/**
 * Environmental conditions
 */
export interface EnvironmentalConditions {
  temperature_c: number;    // 온도 (°C)
  humidity_percent: number; // 상대습도 (%)
  pressure_hPa?: number;    // 기압 (hPa)
  timestamp: Timestamp;
}

/**
 * Quality control flag
 */
export enum QCFlag {
  VALID = 'VALID',
  SUSPECT = 'SUSPECT',
  INVALID = 'INVALID',
  CALIBRATING = 'CALIBRATING',
  OFFLINE = 'OFFLINE',
}

// ============================================================================
// Ventilation & HVAC
// ============================================================================

/**
 * Ventilation metrics
 */
export interface VentilationMetrics {
  airChangeRate_ACH: number;    // 시간당 환기율 (ACH: Air Changes per Hour)
  freshAirFlow_cfm?: number;    // 외기 공급량 (CFM: Cubic Feet per Minute)
  freshAirFlow_m3h?: number;    // 외기 공급량 (m³/h)
  ventilationType: VentilationType;
  timestamp: Timestamp;
}

/**
 * Ventilation type
 */
export enum VentilationType {
  NATURAL = 'NATURAL',           // 자연환기
  MECHANICAL = 'MECHANICAL',     // 기계환기
  MIXED = 'MIXED',               // 혼합환기
  NONE = 'NONE',                 // 환기 없음
}

/**
 * HVAC system status
 */
export interface HVACStatus {
  systemId: string;
  operatingMode: HVACMode;
  fanSpeed: FanSpeed;
  filterStatus: FilterStatus;
  setTemperature_c?: number;
  actualTemperature_c?: number;
  energyConsumption_kWh?: number;
  lastMaintenance?: Timestamp;
  nextMaintenance?: Timestamp;
  timestamp: Timestamp;
}

/**
 * HVAC operating mode
 */
export enum HVACMode {
  OFF = 'OFF',
  HEATING = 'HEATING',
  COOLING = 'COOLING',
  AUTO = 'AUTO',
  VENTILATION = 'VENTILATION',
  ECO = 'ECO',
}

/**
 * Fan speed
 */
export enum FanSpeed {
  OFF = 'OFF',
  LOW = 'LOW',
  MEDIUM = 'MEDIUM',
  HIGH = 'HIGH',
  AUTO = 'AUTO',
}

/**
 * Filter status
 */
export interface FilterStatus {
  lastReplaced: Timestamp;
  lifeRemaining_percent: number;
  needsReplacement: boolean;
  filterType?: string;
}

// ============================================================================
// Indoor Space Information
// ============================================================================

/**
 * Space type
 */
export enum SpaceType {
  RESIDENTIAL = 'RESIDENTIAL',           // 주거
  OFFICE = 'OFFICE',                     // 사무실
  CLASSROOM = 'CLASSROOM',               // 교실
  HOSPITAL = 'HOSPITAL',                 // 병원
  LABORATORY = 'LABORATORY',             // 실험실
  INDUSTRIAL = 'INDUSTRIAL',             // 산업시설
  COMMERCIAL = 'COMMERCIAL',             // 상업시설
  GYM = 'GYM',                          // 체육관
  RESTAURANT = 'RESTAURANT',             // 식당
  OTHER = 'OTHER',
}

/**
 * Indoor space information
 */
export interface IndoorSpace {
  spaceId: string;
  name: string;
  type: SpaceType;
  location: Location;
  floorArea_m2: number;
  ceilingHeight_m: number;
  volume_m3: number;
  occupancy: OccupancyInfo;
  buildingYear?: number;
  renovationYear?: number;
}

/**
 * Occupancy information
 */
export interface OccupancyInfo {
  maxOccupancy: number;
  currentOccupancy?: number;
  occupancySchedule?: OccupancySchedule[];
  averageOccupancy?: number;
}

/**
 * Occupancy schedule
 */
export interface OccupancySchedule {
  dayOfWeek: number;  // 0 = Sunday, 6 = Saturday
  startTime: string;  // HH:MM format
  endTime: string;    // HH:MM format
  expectedOccupancy: number;
}

// ============================================================================
// Sensors & Devices
// ============================================================================

/**
 * Sensor information
 */
export interface SensorInfo {
  sensorId: string;
  type: SensorType;
  manufacturer: string;
  model: string;
  serialNumber?: string;
  installationDate: Timestamp;
  lastCalibration?: Timestamp;
  calibrationInterval_days?: number;
  accuracy?: string;
  location: Location;
  status: SensorStatus;
}

/**
 * Sensor type
 */
export enum SensorType {
  PM_SENSOR = 'PM_SENSOR',               // 미세먼지 센서
  CO2_SENSOR = 'CO2_SENSOR',             // CO2 센서
  VOC_SENSOR = 'VOC_SENSOR',             // VOC 센서
  RADON_SENSOR = 'RADON_SENSOR',         // 라돈 센서
  TEMPERATURE_HUMIDITY = 'TEMPERATURE_HUMIDITY',  // 온습도 센서
  MULTI_SENSOR = 'MULTI_SENSOR',         // 복합 센서
  AIR_QUALITY_MONITOR = 'AIR_QUALITY_MONITOR',  // 공기질 모니터
}

/**
 * Sensor status
 */
export enum SensorStatus {
  ACTIVE = 'ACTIVE',
  INACTIVE = 'INACTIVE',
  MAINTENANCE = 'MAINTENANCE',
  ERROR = 'ERROR',
  CALIBRATING = 'CALIBRATING',
}

// ============================================================================
// Air Quality Reading
// ============================================================================

/**
 * Complete air quality reading
 */
export interface AirQualityReading {
  readingId: string;
  spaceId: string;
  timestamp: Timestamp;

  // Measurements
  particulateMatter?: ParticulateMatter;
  gasConcentrations?: GasConcentrations;
  vocMeasurements?: VOCMeasurements;
  radonMeasurement?: RadonMeasurement;
  environmental: EnvironmentalConditions;
  ventilation?: VentilationMetrics;

  // Sensor info
  sensorId: string;

  // Calculated indices
  iaq?: IndoorAirQualityIndex;

  // Metadata
  dataQuality: DataQuality;
}

/**
 * Indoor Air Quality Index
 */
export interface IndoorAirQualityIndex {
  overall: number;        // 0-500 scale
  category: IAQCategory;
  dominantPollutant?: string;
  healthAdvisory: string;
  color: string;          // Hex color code
}

/**
 * IAQ Category
 */
export enum IAQCategory {
  EXCELLENT = 'EXCELLENT',       // 0-50
  GOOD = 'GOOD',                 // 51-100
  MODERATE = 'MODERATE',         // 101-150
  POOR = 'POOR',                 // 151-200
  VERY_POOR = 'VERY_POOR',       // 201-300
  HAZARDOUS = 'HAZARDOUS',       // 301+
}

/**
 * Data quality metrics
 */
export interface DataQuality {
  completeness_percent: number;
  validityScore: number;
  lastCalibration?: Timestamp;
  uncertaintyEstimate?: number;
}

// ============================================================================
// Health Impact
// ============================================================================

/**
 * Health risk assessment
 */
export interface HealthRiskAssessment {
  assessmentId: string;
  spaceId: string;
  period: {
    start: Timestamp;
    end: Timestamp;
  };

  // Risk levels
  respiratoryRisk: RiskLevel;
  cardiovascularRisk: RiskLevel;
  allergyRisk: RiskLevel;
  overallRisk: RiskLevel;

  // Sensitive groups
  sensitiveGroupsWarning: boolean;
  childrenWarning: boolean;
  elderlyWarning: boolean;
  asthmaWarning: boolean;

  // Recommendations
  recommendations: Recommendation[];

  timestamp: Timestamp;
}

/**
 * Risk level
 */
export enum RiskLevel {
  LOW = 'LOW',
  MODERATE = 'MODERATE',
  HIGH = 'HIGH',
  VERY_HIGH = 'VERY_HIGH',
}

/**
 * Recommendation
 */
export interface Recommendation {
  priority: 'HIGH' | 'MEDIUM' | 'LOW';
  category: RecommendationCategory;
  action: string;
  expectedImpact?: string;
  estimatedCost?: string;
}

/**
 * Recommendation category
 */
export enum RecommendationCategory {
  VENTILATION = 'VENTILATION',         // 환기
  HVAC_MAINTENANCE = 'HVAC_MAINTENANCE',  // HVAC 유지보수
  SOURCE_CONTROL = 'SOURCE_CONTROL',   // 오염원 제거
  AIR_PURIFICATION = 'AIR_PURIFICATION',  // 공기정화
  OCCUPANCY = 'OCCUPANCY',             // 재실 관리
  MONITORING = 'MONITORING',           // 모니터링
}

// ============================================================================
// Standards & Compliance
// ============================================================================

/**
 * Air quality standards
 */
export interface AirQualityStandards {
  standardName: string;
  region: string;
  limits: PollutantLimit[];
  effectiveDate: Timestamp;
}

/**
 * Pollutant limit
 */
export interface PollutantLimit {
  pollutant: string;
  limit: number;
  unit: string;
  averagingPeriod: string;
  exceedanceAllowed?: number;
}

/**
 * Compliance status
 */
export interface ComplianceStatus {
  spaceId: string;
  standard: string;
  compliant: boolean;
  violations: Violation[];
  lastAssessment: Timestamp;
}

/**
 * Violation record
 */
export interface Violation {
  pollutant: string;
  timestamp: Timestamp;
  measuredValue: number;
  limitValue: number;
  exceedancePercent: number;
  duration_minutes: number;
}

// ============================================================================
// Alerts & Notifications
// ============================================================================

/**
 * Air quality alert
 */
export interface AirQualityAlert {
  alertId: string;
  spaceId: string;
  severity: AlertSeverity;
  alertType: AlertType;
  pollutant?: string;
  currentValue?: number;
  thresholdValue?: number;
  message: string;
  timestamp: Timestamp;
  acknowledged: boolean;
  resolvedAt?: Timestamp;
}

/**
 * Alert severity
 */
export enum AlertSeverity {
  INFO = 'INFO',
  WARNING = 'WARNING',
  CRITICAL = 'CRITICAL',
  EMERGENCY = 'EMERGENCY',
}

/**
 * Alert type
 */
export enum AlertType {
  THRESHOLD_EXCEEDED = 'THRESHOLD_EXCEEDED',
  SENSOR_MALFUNCTION = 'SENSOR_MALFUNCTION',
  POOR_VENTILATION = 'POOR_VENTILATION',
  HIGH_OCCUPANCY = 'HIGH_OCCUPANCY',
  FILTER_REPLACEMENT = 'FILTER_REPLACEMENT',
  CALIBRATION_DUE = 'CALIBRATION_DUE',
}

// ============================================================================
// Historical Data & Analytics
// ============================================================================

/**
 * Time series data
 */
export interface TimeSeriesData {
  spaceId: string;
  parameter: string;
  period: {
    start: Timestamp;
    end: Timestamp;
  };
  resolution: '1min' | '5min' | '15min' | '1hour' | '1day';
  dataPoints: DataPoint[];
}

/**
 * Data point
 */
export interface DataPoint {
  timestamp: Timestamp;
  value: number;
  qc_flag: QCFlag;
}

/**
 * Statistical summary
 */
export interface StatisticalSummary {
  spaceId: string;
  parameter: string;
  period: {
    start: Timestamp;
    end: Timestamp;
  };
  statistics: {
    mean: number;
    median: number;
    min: number;
    max: number;
    stdDev: number;
    percentile_95: number;
    percentile_99: number;
  };
  exceedances?: number;
  complianceRate_percent?: number;
}

// ============================================================================
// API Request/Response Types
// ============================================================================

/**
 * Space registration request
 */
export interface SpaceRegistrationRequest {
  space: IndoorSpace;
  sensors: SensorInfo[];
  hvacSystems?: HVACStatus[];
}

/**
 * Space registration response
 */
export interface SpaceRegistrationResponse {
  spaceId: string;
  registrationDate: Timestamp;
  dashboardUrl: string;
  apiAccessToken?: string;
}

/**
 * Reading submission request
 */
export interface ReadingSubmissionRequest {
  readings: AirQualityReading[];
}

/**
 * Reading submission response
 */
export interface ReadingSubmissionResponse {
  accepted: number;
  rejected: number;
  errors?: string[];
}

/**
 * Query parameters for historical data
 */
export interface HistoricalDataQuery {
  spaceId: string;
  parameters: string[];
  startDate: Timestamp;
  endDate: Timestamp;
  resolution?: '1min' | '5min' | '15min' | '1hour' | '1day';
  aggregation?: 'mean' | 'median' | 'min' | 'max';
}

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

// ============================================================================
// Reports
// ============================================================================

/**
 * Report type
 */
export enum ReportType {
  DAILY = 'DAILY',
  WEEKLY = 'WEEKLY',
  MONTHLY = 'MONTHLY',
  QUARTERLY = 'QUARTERLY',
  ANNUAL = 'ANNUAL',
  CUSTOM = 'CUSTOM',
}

/**
 * Air quality report
 */
export interface AirQualityReport {
  reportId: string;
  reportType: ReportType;
  spaceId: string;
  period: {
    start: Timestamp;
    end: Timestamp;
  };

  // Summary statistics
  summary: {
    averageIAQ: number;
    complianceRate_percent: number;
    alertsGenerated: number;
    criticalAlerts: number;
  };

  // Parameter summaries
  parameterSummaries: StatisticalSummary[];

  // Health assessment
  healthRisk?: HealthRiskAssessment;

  // Compliance
  compliance: ComplianceStatus;

  // Recommendations
  recommendations: Recommendation[];

  generatedAt: Timestamp;
  generatedBy?: string;
}

// ============================================================================
// Export all types
// ============================================================================

export type {
  Timestamp,
  Coordinates,
  Location,
};
