/**
 * WIA-ENE-035: Environmental Sensor Network Standard - TypeScript Type Definitions
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
  altitude?: number;  // meters
}

/**
 * Location information
 */
export interface Location {
  sensorId: string;
  siteName: string;
  coordinates: Coordinates;
  zone?: string;          // 구역 (도시, 산업단지 등)
  description?: string;
}

// ============================================================================
// Enumerations
// ============================================================================

/**
 * Sensor types
 */
export enum SensorType {
  // Air quality sensors
  AIR_QUALITY = 'AIR_QUALITY',                     // 대기질 센서
  PM_SENSOR = 'PM_SENSOR',                         // 미세먼지 센서
  GAS_SENSOR = 'GAS_SENSOR',                       // 가스 센서
  VOC_SENSOR = 'VOC_SENSOR',                       // 휘발성 유기화합물 센서

  // Water quality sensors
  WATER_QUALITY = 'WATER_QUALITY',                 // 수질 센서
  PH_SENSOR = 'PH_SENSOR',                         // pH 센서
  TURBIDITY_SENSOR = 'TURBIDITY_SENSOR',           // 탁도 센서
  DISSOLVED_OXYGEN = 'DISSOLVED_OXYGEN',           // 용존산소 센서
  CONDUCTIVITY_SENSOR = 'CONDUCTIVITY_SENSOR',     // 전기전도도 센서

  // Soil sensors
  SOIL_MOISTURE = 'SOIL_MOISTURE',                 // 토양 수분 센서
  SOIL_TEMPERATURE = 'SOIL_TEMPERATURE',           // 토양 온도 센서
  SOIL_PH = 'SOIL_PH',                             // 토양 pH 센서
  SOIL_EC = 'SOIL_EC',                             // 토양 전기전도도 센서
  SOIL_NPK = 'SOIL_NPK',                           // 토양 NPK 센서

  // Noise sensors
  NOISE_LEVEL = 'NOISE_LEVEL',                     // 소음 센서
  VIBRATION = 'VIBRATION',                         // 진동 센서

  // Radiation sensors
  RADIATION_ALPHA = 'RADIATION_ALPHA',             // 알파선 센서
  RADIATION_BETA = 'RADIATION_BETA',               // 베타선 센서
  RADIATION_GAMMA = 'RADIATION_GAMMA',             // 감마선 센서
  RADIATION_NEUTRON = 'RADIATION_NEUTRON',         // 중성자선 센서

  // Environmental sensors
  TEMPERATURE = 'TEMPERATURE',                     // 온도 센서
  HUMIDITY = 'HUMIDITY',                           // 습도 센서
  PRESSURE = 'PRESSURE',                           // 기압 센서
  LIGHT = 'LIGHT',                                 // 조도 센서
  UV = 'UV',                                       // 자외선 센서
  RAIN_GAUGE = 'RAIN_GAUGE',                       // 우량계
  WIND_SPEED = 'WIND_SPEED',                       // 풍속계
  WIND_DIRECTION = 'WIND_DIRECTION',               // 풍향계
}

/**
 * IoT communication protocols
 */
export enum IoTProtocol {
  // LPWAN protocols
  LORAWAN = 'LORAWAN',           // LoRaWAN
  SIGFOX = 'SIGFOX',             // Sigfox
  NB_IOT = 'NB_IOT',             // NB-IoT
  LTE_M = 'LTE_M',               // LTE-M

  // Short-range protocols
  ZIGBEE = 'ZIGBEE',             // ZigBee
  BLUETOOTH_LE = 'BLUETOOTH_LE', // Bluetooth Low Energy
  WIFI = 'WIFI',                 // Wi-Fi

  // Wired protocols
  ETHERNET = 'ETHERNET',         // Ethernet
  MODBUS = 'MODBUS',             // Modbus
  RS485 = 'RS485',               // RS-485

  // Application protocols
  MQTT = 'MQTT',                 // MQTT
  COAP = 'COAP',                 // CoAP
  HTTP = 'HTTP',                 // HTTP/HTTPS
  WEBSOCKET = 'WEBSOCKET',       // WebSocket
}

/**
 * Sensor status
 */
export enum SensorStatus {
  ONLINE = 'ONLINE',             // 정상 작동
  OFFLINE = 'OFFLINE',           // 오프라인
  MAINTENANCE = 'MAINTENANCE',   // 유지보수 중
  ERROR = 'ERROR',               // 오류
  CALIBRATING = 'CALIBRATING',   // 보정 중
  LOW_BATTERY = 'LOW_BATTERY',   // 배터리 부족
}

/**
 * Data quality level
 */
export enum DataQuality {
  EXCELLENT = 'EXCELLENT',       // 우수 (오차 < 5%)
  GOOD = 'GOOD',                 // 양호 (오차 < 10%)
  FAIR = 'FAIR',                 // 보통 (오차 < 20%)
  POOR = 'POOR',                 // 불량 (오차 > 20%)
  INVALID = 'INVALID',           // 유효하지 않음
}

/**
 * Alert severity
 */
export enum AlertSeverity {
  INFO = 'INFO',                 // 정보
  WARNING = 'WARNING',           // 경고
  CRITICAL = 'CRITICAL',         // 위험
  EMERGENCY = 'EMERGENCY',       // 긴급
}

/**
 * Calibration method
 */
export enum CalibrationType {
  FACTORY = 'FACTORY',           // 공장 보정
  FIELD = 'FIELD',               // 현장 보정
  AUTOMATIC = 'AUTOMATIC',       // 자동 보정
  MANUAL = 'MANUAL',             // 수동 보정
  REFERENCE = 'REFERENCE',       // 기준 센서 대조
}

// ============================================================================
// Sensor Configuration
// ============================================================================

/**
 * Sensor specifications
 */
export interface SensorSpecs {
  manufacturer: string;
  model: string;
  serialNumber: string;
  firmwareVersion: string;
  measuringRange: {
    min: number;
    max: number;
    unit: string;
  };
  accuracy: number;              // percentage
  resolution: number;
  responseTime: number;          // seconds
  operatingTemperature: {
    min: number;                 // °C
    max: number;                 // °C
  };
  ipRating?: string;             // IP rating (e.g., 'IP67')
  certifications?: string[];     // 인증 (CE, FCC, KCC 등)
}

/**
 * Power configuration
 */
export interface PowerConfig {
  powerSource: 'battery' | 'solar' | 'grid' | 'hybrid';
  batteryType?: string;
  batteryCapacity?: number;      // mAh
  batteryLevel?: number;         // percentage
  solarPanelWattage?: number;    // watts
  powerConsumption: number;      // mW
  expectedBatteryLife?: number;  // days
}

/**
 * Communication configuration
 */
export interface CommunicationConfig {
  protocol: IoTProtocol;
  frequency?: number;            // MHz
  bandwidth?: number;            // kHz
  transmitPower?: number;        // dBm
  dataRate?: number;             // bps
  networkId?: string;
  deviceEUI?: string;            // LoRaWAN DevEUI
  appKey?: string;               // LoRaWAN AppKey
  signalStrength?: number;       // RSSI (dBm)
  linkQuality?: number;          // LQI (0-255)
}

/**
 * Sampling configuration
 */
export interface SamplingConfig {
  samplingInterval: number;      // seconds
  transmissionInterval: number;  // seconds
  averagingPeriod?: number;      // seconds
  samplesPerTransmission: number;
  enabledParameters: string[];   // 측정 항목 목록
}

/**
 * Sensor device configuration
 */
export interface SensorDevice {
  sensorId: string;
  sensorType: SensorType;
  location: Location;
  specs: SensorSpecs;
  power: PowerConfig;
  communication: CommunicationConfig;
  sampling: SamplingConfig;
  status: SensorStatus;
  installationDate: Timestamp;
  lastMaintenanceDate?: Timestamp;
  nextMaintenanceDate?: Timestamp;
  notes?: string;
}

// ============================================================================
// Sensor Readings
// ============================================================================

/**
 * Air quality reading
 */
export interface AirQualityReading {
  pm1_0?: number;                // μg/m³
  pm2_5?: number;                // μg/m³
  pm10?: number;                 // μg/m³
  co?: number;                   // ppm
  co2?: number;                  // ppm
  no2?: number;                  // ppb
  so2?: number;                  // ppb
  o3?: number;                   // ppb
  voc?: number;                  // ppb
  aqi?: number;                  // Air Quality Index
}

/**
 * Water quality reading
 */
export interface WaterQualityReading {
  ph?: number;
  turbidity?: number;            // NTU
  dissolvedOxygen?: number;      // mg/L
  conductivity?: number;         // μS/cm
  temperature?: number;          // °C
  tds?: number;                  // mg/L (Total Dissolved Solids)
  orp?: number;                  // mV (Oxidation-Reduction Potential)
  salinity?: number;             // ppt
  bod?: number;                  // mg/L (Biochemical Oxygen Demand)
  cod?: number;                  // mg/L (Chemical Oxygen Demand)
}

/**
 * Soil sensor reading
 */
export interface SoilReading {
  moisture?: number;             // percentage
  temperature?: number;          // °C
  ph?: number;
  ec?: number;                   // μS/cm (Electrical Conductivity)
  nitrogen?: number;             // mg/kg
  phosphorus?: number;           // mg/kg
  potassium?: number;            // mg/kg
  organicMatter?: number;        // percentage
}

/**
 * Noise reading
 */
export interface NoiseReading {
  decibels: number;              // dB
  frequency?: number;            // Hz
  leq?: number;                  // dB (Equivalent continuous sound level)
  lmax?: number;                 // dB (Maximum sound level)
  lmin?: number;                 // dB (Minimum sound level)
  l10?: number;                  // dB (10th percentile)
  l50?: number;                  // dB (50th percentile)
  l90?: number;                  // dB (90th percentile)
}

/**
 * Radiation reading
 */
export interface RadiationReading {
  alpha?: number;                // cpm or Bq
  beta?: number;                 // cpm or Bq
  gamma?: number;                // μSv/h or mR/h
  neutron?: number;              // counts
  totalDose?: number;            // μSv
  doseRate?: number;             // μSv/h
}

/**
 * Environmental reading
 */
export interface EnvironmentalReading {
  temperature?: number;          // °C
  humidity?: number;             // percentage
  pressure?: number;             // hPa
  lightIntensity?: number;       // lux
  uvIndex?: number;
  rainfall?: number;             // mm
  windSpeed?: number;            // m/s
  windDirection?: number;        // degrees (0-360)
}

/**
 * Generic sensor reading
 */
export interface SensorReading {
  readingId: string;
  sensorId: string;
  sensorType: SensorType;
  timestamp: Timestamp;

  // Specific readings (based on sensor type)
  airQuality?: AirQualityReading;
  waterQuality?: WaterQualityReading;
  soil?: SoilReading;
  noise?: NoiseReading;
  radiation?: RadiationReading;
  environmental?: EnvironmentalReading;

  // Metadata
  dataQuality: DataQuality;
  batteryLevel?: number;         // percentage
  signalStrength?: number;       // RSSI (dBm)
  flags?: string[];              // 경고 플래그
  notes?: string;
}

// ============================================================================
// Calibration
// ============================================================================

/**
 * Calibration point
 */
export interface CalibrationPoint {
  referenceValue: number;
  measuredValue: number;
  unit: string;
  deviation: number;             // percentage
}

/**
 * Calibration record
 */
export interface CalibrationRecord {
  calibrationId: string;
  sensorId: string;
  calibrationType: CalibrationType;
  calibrationDate: Timestamp;
  performedBy: string;

  // Calibration data
  calibrationPoints: CalibrationPoint[];
  preCalibrationDrift?: number;  // percentage
  postCalibrationAccuracy: number; // percentage

  // Reference equipment
  referenceEquipment?: {
    manufacturer: string;
    model: string;
    serialNumber: string;
    certificationDate: Timestamp;
  };

  // Results
  passed: boolean;
  nextCalibrationDue: Timestamp;
  notes?: string;
}

// ============================================================================
// Maintenance
// ============================================================================

/**
 * Maintenance task type
 */
export enum MaintenanceType {
  INSPECTION = 'INSPECTION',               // 점검
  CLEANING = 'CLEANING',                   // 청소
  CALIBRATION = 'CALIBRATION',             // 보정
  BATTERY_REPLACEMENT = 'BATTERY_REPLACEMENT', // 배터리 교체
  FIRMWARE_UPDATE = 'FIRMWARE_UPDATE',     // 펌웨어 업데이트
  REPAIR = 'REPAIR',                       // 수리
  RELOCATION = 'RELOCATION',               // 위치 변경
  DECOMMISSION = 'DECOMMISSION',           // 폐기
}

/**
 * Maintenance record
 */
export interface MaintenanceRecord {
  maintenanceId: string;
  sensorId: string;
  maintenanceType: MaintenanceType;
  scheduledDate: Timestamp;
  performedDate?: Timestamp;
  performedBy?: string;

  // Details
  description: string;
  partsReplaced?: string[];
  costsIncurred?: number;

  // Results
  status: 'scheduled' | 'in-progress' | 'completed' | 'cancelled';
  preMaintenanceStatus?: SensorStatus;
  postMaintenanceStatus?: SensorStatus;
  issues?: string[];
  recommendations?: string[];
  notes?: string;
}

// ============================================================================
// Alerts & Thresholds
// ============================================================================

/**
 * Alert threshold
 */
export interface AlertThreshold {
  parameter: string;             // e.g., 'pm2_5', 'temperature'
  unit: string;
  warningMin?: number;
  warningMax?: number;
  criticalMin?: number;
  criticalMax?: number;
  emergencyMin?: number;
  emergencyMax?: number;
}

/**
 * Alert configuration
 */
export interface AlertConfig {
  alertId: string;
  sensorId: string;
  enabled: boolean;
  thresholds: AlertThreshold[];
  notificationChannels: ('email' | 'sms' | 'push' | 'webhook')[];
  recipients: string[];
  cooldownPeriod?: number;       // seconds (prevent alert spam)
}

/**
 * Alert event
 */
export interface AlertEvent {
  eventId: string;
  sensorId: string;
  timestamp: Timestamp;
  severity: AlertSeverity;
  parameter: string;
  value: number;
  threshold: number;
  message: string;
  acknowledged: boolean;
  acknowledgedBy?: string;
  acknowledgedAt?: Timestamp;
  resolved: boolean;
  resolvedAt?: Timestamp;
  actions?: string[];            // 조치 사항
}

// ============================================================================
// Data Aggregation
// ============================================================================

/**
 * Time series data point
 */
export interface TimeSeriesPoint {
  timestamp: Timestamp;
  value: number;
}

/**
 * Statistical aggregation
 */
export interface StatisticalSummary {
  count: number;
  mean: number;
  median: number;
  min: number;
  max: number;
  stdDev: number;
  p10?: number;                  // 10th percentile
  p50?: number;                  // 50th percentile
  p90?: number;                  // 90th percentile
  p95?: number;                  // 95th percentile
  p99?: number;                  // 99th percentile
}

/**
 * Aggregated sensor data
 */
export interface AggregatedData {
  aggregationId: string;
  sensorId: string;
  parameter: string;
  unit: string;
  period: {
    startDate: Timestamp;
    endDate: Timestamp;
  };
  interval: 'minute' | 'hour' | 'day' | 'week' | 'month';
  timeSeries: TimeSeriesPoint[];
  statistics: StatisticalSummary;
}

// ============================================================================
// Network Management
// ============================================================================

/**
 * Sensor network
 */
export interface SensorNetwork {
  networkId: string;
  networkName: string;
  description: string;
  organization: string;

  // Network configuration
  protocol: IoTProtocol;
  gateway?: {
    gatewayId: string;
    location: Coordinates;
    ipAddress?: string;
    status: 'online' | 'offline';
  };

  // Sensors in network
  sensors: string[];             // sensor IDs
  totalSensors: number;
  activeSensors: number;
  offlineSensors: number;

  // Coverage area
  coverageArea?: {
    type: 'Polygon';
    coordinates: number[][][];   // GeoJSON format
  };

  // Network health
  networkHealth: {
    uptime: number;              // percentage
    avgSignalStrength: number;   // RSSI (dBm)
    packetLossRate: number;      // percentage
    avgLatency: number;          // ms
  };

  createdAt: Timestamp;
  updatedAt: Timestamp;
}

/**
 * Gateway device
 */
export interface Gateway {
  gatewayId: string;
  location: Location;
  protocol: IoTProtocol;
  ipAddress?: string;
  macAddress?: string;

  // Connectivity
  connectionType: 'ethernet' | 'cellular' | 'wifi';
  uplink: {
    bandwidth: number;           // Mbps
    latency: number;             // ms
  };

  // Status
  status: 'online' | 'offline' | 'maintenance';
  connectedSensors: number;
  maxSensors: number;

  // Edge computing
  edgeComputing?: {
    enabled: boolean;
    processing: ('filtering' | 'aggregation' | 'anomaly-detection' | 'ml-inference')[];
    cpuUsage?: number;           // percentage
    memoryUsage?: number;        // percentage
    storageUsage?: number;       // percentage
  };

  lastSeen: Timestamp;
}

// ============================================================================
// Edge Computing
// ============================================================================

/**
 * Edge processing rule
 */
export interface EdgeProcessingRule {
  ruleId: string;
  ruleName: string;
  enabled: boolean;

  // Trigger conditions
  sensorTypes: SensorType[];
  conditions: {
    parameter: string;
    operator: '>' | '<' | '==' | '!=' | '>=' | '<=';
    value: number;
  }[];

  // Actions
  actions: {
    type: 'filter' | 'aggregate' | 'alert' | 'store' | 'forward';
    config: any;
  }[];

  // Processing
  priority: number;
  executionOrder: number;
}

/**
 * Anomaly detection result
 */
export interface AnomalyDetection {
  detectionId: string;
  sensorId: string;
  timestamp: Timestamp;
  parameter: string;
  expectedValue: number;
  actualValue: number;
  deviation: number;             // percentage
  anomalyScore: number;          // 0-100
  confidence: number;            // 0-100
  possibleCauses?: string[];
  recommendations?: string[];
}

// ============================================================================
// Data Quality
// ============================================================================

/**
 * Quality control check
 */
export interface QualityCheck {
  checkType: 'range' | 'rate-of-change' | 'stuck-value' | 'spike' | 'missing-data';
  passed: boolean;
  details?: string;
}

/**
 * Data validation result
 */
export interface DataValidation {
  validationId: string;
  readingId: string;
  timestamp: Timestamp;

  // Checks performed
  checks: QualityCheck[];
  overallQuality: DataQuality;

  // Issues found
  issues: string[];
  correctedValues?: {
    parameter: string;
    originalValue: number;
    correctedValue: number;
    method: string;
  }[];

  validated: boolean;
  validatedBy?: 'automatic' | 'manual';
  notes?: string;
}

// ============================================================================
// Reporting
// ============================================================================

/**
 * Sensor network report
 */
export interface NetworkReport {
  reportId: string;
  networkId: string;
  reportType: 'daily' | 'weekly' | 'monthly' | 'annual' | 'custom';
  period: {
    startDate: Timestamp;
    endDate: Timestamp;
  };

  // Summary
  summary: {
    totalSensors: number;
    activePercentage: number;
    dataPointsCollected: number;
    alertsTriggered: number;
    maintenanceActivities: number;
  };

  // Performance metrics
  performance: {
    networkUptime: number;       // percentage
    dataQuality: number;         // percentage
    avgResponseTime: number;     // seconds
    packetLossRate: number;      // percentage
  };

  // Environmental insights
  insights?: {
    parameter: string;
    trend: 'increasing' | 'decreasing' | 'stable';
    changeRate?: number;         // percentage
    exceedances?: number;        // threshold violations
  }[];

  generatedBy: string;
  generatedAt: Timestamp;
}

// ============================================================================
// KPIs (Key Performance Indicators)
// ============================================================================

/**
 * Network performance KPIs
 */
export interface NetworkKPIs {
  networkUptime: number;         // percentage
  sensorAvailability: number;    // percentage
  dataCompleteness: number;      // percentage
  dataAccuracy: number;          // percentage
  avgResponseTime: number;       // seconds
  packetLossRate: number;        // percentage
}

/**
 * Environmental KPIs
 */
export interface EnvironmentalKPIs {
  airQualityIndex?: number;
  waterQualityIndex?: number;
  noiseLevel?: number;           // dB
  exceedanceCount: number;       // threshold violations
  complianceRate: number;        // percentage
}

/**
 * Operational KPIs
 */
export interface OperationalKPIs {
  maintenanceCompletionRate: number;  // percentage
  calibrationCompletionRate: number;  // percentage
  avgRepairTime: number;              // hours
  costPerSensor: number;              // currency
  alertResponseTime: number;          // minutes
}

/**
 * KPI dashboard
 */
export interface KPIDashboard {
  dashboardId: string;
  networkId: string;
  timestamp: Timestamp;

  network: NetworkKPIs;
  environmental: EnvironmentalKPIs;
  operational: OperationalKPIs;

  overallScore?: number;         // 0-100
}

// ============================================================================
// API Request/Response Types
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
    details?: any;
  };
  metadata?: {
    timestamp: Timestamp;
    version: string;
  };
}

/**
 * Pagination parameters
 */
export interface PaginationParams {
  page: number;
  limit: number;
  sortBy?: string;
  sortOrder?: 'asc' | 'desc';
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  items: T[];
  total: number;
  page: number;
  limit: number;
  hasMore: boolean;
}

/**
 * Date range filter
 */
export interface DateRangeFilter {
  startDate: Timestamp;
  endDate: Timestamp;
}

/**
 * Sensor filter
 */
export interface SensorFilter {
  sensorIds?: string[];
  sensorTypes?: SensorType[];
  status?: SensorStatus[];
  location?: {
    boundingBox?: {
      north: number;
      south: number;
      east: number;
      west: number;
    };
    radius?: {
      center: Coordinates;
      radiusKm: number;
    };
  };
}

// ============================================================================
// Export all types
// ============================================================================

export type {
  Timestamp,
  Coordinates,
  Location,
};
