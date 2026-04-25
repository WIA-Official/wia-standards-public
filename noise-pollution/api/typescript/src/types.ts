/**
 * WIA-ENE-028: Noise Pollution Standard - TypeScript Type Definitions
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
}

/**
 * Location information
 */
export interface Location {
  address: string;
  coordinates: Coordinates;
  elevation?: number;          // 고도 (m)
  zoneType?: string;           // 지역 유형
}

// ============================================================================
// Noise Measurement Types
// ============================================================================

/**
 * Decibel weighting types
 */
export enum WeightingType {
  A = 'A',                     // A-가중 (인간 청각)
  C = 'C',                     // C-가중 (저주파 포함)
  Z = 'Z',                     // Z-가중 (무가중, 전 주파수)
}

/**
 * Time period types
 */
export enum TimePeriod {
  DAY = 'day',                 // 주간 (07:00-18:00)
  EVENING = 'evening',         // 저녁 (18:00-22:00)
  NIGHT = 'night',             // 야간 (22:00-07:00)
}

/**
 * dBA noise levels
 */
export interface NoiseLevel_dBA {
  leq: number;                 // 등가소음도 (dBA)
  lmax: number;                // 최대소음도 (dBA)
  lmin: number;                // 최소소음도 (dBA)
  l10: number;                 // 10% 초과 레벨
  l50: number;                 // 50% 초과 레벨 (중앙값)
  l90: number;                 // 90% 초과 레벨 (배경소음)
}

/**
 * dBC noise levels
 */
export interface NoiseLevel_dBC {
  leq: number;                 // C-가중 등가소음도
  lmax: number;                // C-가중 최대소음도
}

/**
 * dBZ noise levels
 */
export interface NoiseLevel_dBZ {
  leq: number;                 // 무가중 등가소음도
  lmax: number;                // 무가중 최대소음도
}

/**
 * Combined noise levels
 */
export interface NoiseLevels {
  dBA: NoiseLevel_dBA;
  dBC: NoiseLevel_dBC;
  dBZ?: NoiseLevel_dBZ;
}

/**
 * Octave band measurement
 */
export interface OctaveBand {
  freq: number;                // 중심 주파수 (Hz)
  level: number;               // 음압 레벨 (dB)
}

/**
 * Frequency analysis data
 */
export interface FrequencyAnalysis {
  octaveBand: OctaveBand[];                    // 옥타브 밴드
  oneThirdOctave?: OctaveBand[];               // 1/3 옥타브 밴드
}

/**
 * Environmental conditions during measurement
 */
export interface EnvironmentalConditions {
  temperature: number;         // 온도 (°C)
  humidity: number;            // 습도 (%)
  windSpeed: number;           // 풍속 (m/s)
  windDirection: number;       // 풍향 (도, 0-360)
  precipitation: boolean;      // 강수 여부
}

/**
 * Measurement metadata
 */
export interface MeasurementMetadata {
  instrumentId: string;        // 측정 장비 ID
  calibrationDate: Timestamp;  // 교정 일자
  dataQuality: number;         // 데이터 품질 (0-100)
  flags: string[];             // 플래그 (예: 'rain', 'wind', 'malfunction')
}

/**
 * Complete noise measurement data
 */
export interface NoiseMeasurement {
  measurementId: string;
  timestamp: Timestamp;
  stationId: string;
  location: Location;
  levels: NoiseLevels;
  frequencyAnalysis?: FrequencyAnalysis;
  environmental: EnvironmentalConditions;
  metadata: MeasurementMetadata;
}

// ============================================================================
// Noise Source Classification
// ============================================================================

/**
 * Noise source categories
 */
export enum NoiseSourceCategory {
  TRAFFIC_ROAD = 'traffic_road',               // 도로 교통
  TRAFFIC_RAIL = 'traffic_rail',               // 철도
  TRAFFIC_AIR = 'traffic_air',                 // 항공기
  INDUSTRIAL = 'industrial',                   // 산업 시설
  CONSTRUCTION = 'construction',               // 건설 공사
  RESIDENTIAL = 'residential',                 // 생활 소음
  COMMERCIAL = 'commercial',                   // 상업 시설
  RECREATIONAL = 'recreational',               // 여가 시설
  OTHER = 'other',                            // 기타
}

/**
 * Noise pattern types
 */
export enum NoisePattern {
  CONTINUOUS = 'continuous',                   // 연속적
  INTERMITTENT = 'intermittent',               // 간헐적
  IMPULSE = 'impulse',                        // 임펄스 (순간적)
}

/**
 * Frequency range
 */
export interface FrequencyRange {
  low: number;                 // 저주파 (Hz)
  high: number;                // 고주파 (Hz)
}

/**
 * Noise characteristics
 */
export interface NoiseCharacteristics {
  typical_dBA: number;         // 전형적 소음도 (dBA)
  peak_dBA: number;            // 최고 소음도 (dBA)
  frequency_range: FrequencyRange;
  pattern: NoisePattern;
}

/**
 * Operating hours
 */
export interface OperatingHours {
  weekday: {
    start: string;             // HH:MM format
    end: string;               // HH:MM format
  };
  weekend: {
    start: string;
    end: string;
  };
  exceptions: string[];        // 예외 사항
}

/**
 * Mitigation measure
 */
export interface MitigationMeasure {
  type: string;                // 저감 조치 유형
  effectiveness: number;       // 효과 (dB 감소)
  installedDate: Timestamp;    // 설치 일자
  cost?: number;               // 비용 (원)
}

/**
 * Compliance information
 */
export interface ComplianceInfo {
  limitApplicable: number;     // 적용 기준 (dBA)
  currentLevel: number;        // 현재 레벨 (dBA)
  isCompliant: boolean;        // 준수 여부
  lastInspection: Timestamp;   // 최근 검사일
  violations?: string[];       // 위반 사항
}

/**
 * Noise source information
 */
export interface NoiseSource {
  sourceId: string;
  name: string;
  category: NoiseSourceCategory;
  location: Location & { area?: number };  // 영향 범위 (m²)
  characteristics: NoiseCharacteristics;
  operatingHours: OperatingHours;
  mitigationMeasures: MitigationMeasure[];
  compliance: ComplianceInfo;
}

// ============================================================================
// Monitoring Station
// ============================================================================

/**
 * Station type
 */
export enum StationType {
  PERMANENT = 'permanent',     // 상설
  TEMPORARY = 'temporary',     // 임시
  MOBILE = 'mobile',          // 이동식
}

/**
 * Instrument class (IEC 61672-1)
 */
export enum InstrumentClass {
  CLASS_0 = 0,                 // ±0.7 dB (실험실 표준)
  CLASS_1 = 1,                 // ±1.0 dB (정밀 측정)
  CLASS_2 = 2,                 // ±1.5 dB (일반 측정)
  CLASS_3 = 3,                 // ±2.5 dB (간이 측정)
}

/**
 * Equipment information
 */
export interface Equipment {
  instrumentId: string;
  manufacturer: string;        // 제조사
  model: string;              // 모델명
  class: InstrumentClass;     // 정확도 등급
  calibrationDue: Timestamp;  // 다음 교정일
  installDate: Timestamp;     // 설치 일자
  serialNumber?: string;      // 일련번호
}

/**
 * Operational status
 */
export enum OperationalStatus {
  ACTIVE = 'active',
  INACTIVE = 'inactive',
  MAINTENANCE = 'maintenance',
  MALFUNCTION = 'malfunction',
}

/**
 * Data transmission mode
 */
export enum DataTransmission {
  REALTIME = 'realtime',      // 실시간
  BATCH = 'batch',            // 배치
}

/**
 * Power source
 */
export enum PowerSource {
  GRID = 'grid',              // 전력망
  SOLAR = 'solar',            // 태양광
  BATTERY = 'battery',        // 배터리
  HYBRID = 'hybrid',          // 혼합
}

/**
 * Operation information
 */
export interface OperationInfo {
  status: OperationalStatus;
  startDate: Timestamp;
  measurementInterval: number;        // 측정 주기 (초)
  dataTransmission: DataTransmission;
  powerSource: PowerSource;
  uptime?: number;                    // 가동률 (%)
}

/**
 * Communication protocol
 */
export enum CommunicationProtocol {
  MQTT = 'mqtt',
  HTTP = 'http',
  WEBSOCKET = 'websocket',
  MOBILE_4G = '4g',
  MOBILE_5G = '5g',
  WIFI = 'wifi',
}

/**
 * Communication information
 */
export interface CommunicationInfo {
  protocol: CommunicationProtocol;
  endpoint: string;            // 데이터 전송 엔드포인트
  lastTransmission: Timestamp; // 최근 전송 시각
  signalStrength?: number;     // 신호 강도 (%)
}

/**
 * Surroundings information
 */
export interface Surroundings {
  nearestRoad: number;         // 최근접 도로 거리 (m)
  nearestBuilding: number;     // 최근접 건물 거리 (m)
  landUse: string;            // 토지 이용
  populationDensity: number;   // 인구 밀도 (명/km²)
  description?: string;        // 설명
}

/**
 * Monitoring station
 */
export interface MonitoringStation {
  stationId: string;
  name: string;
  type: StationType;
  location: Location;
  equipment: Equipment;
  operation: OperationInfo;
  communication: CommunicationInfo;
  surroundings: Surroundings;
}

// ============================================================================
// Time Period Analysis
// ============================================================================

/**
 * Period noise data
 */
export interface PeriodNoiseData {
  leq: number;                 // 등가소음도
  lmax: number;                // 최대소음도
  exceedances: number;         // 기준 초과 횟수
}

/**
 * Time period noise levels
 */
export interface TimePeriodNoise {
  date: Timestamp;
  location: string;
  periods: {
    day: PeriodNoiseData;      // 주간 (07:00-18:00)
    evening: PeriodNoiseData;  // 저녁 (18:00-22:00)
    night: PeriodNoiseData;    // 야간 (22:00-07:00)
  };
  lden: number;                // 주야간 등가소음도
  lnight: number;              // 야간 소음도
  compliance: {
    dayLimit: number;
    eveningLimit: number;
    nightLimit: number;
    isCompliant: boolean;
    violations: string[];
  };
}

// ============================================================================
// Health Impact Assessment
// ============================================================================

/**
 * Exposed population data
 */
export interface ExposedPopulation {
  above55dB: number;           // 55 dB 이상 노출 인구
  above65dB: number;           // 65 dB 이상 노출 인구
  above75dB: number;           // 75 dB 이상 노출 인구
}

/**
 * Exposure assessment
 */
export interface ExposureAssessment {
  population: number;          // 총 인구
  exposedPopulation: ExposedPopulation;
  averageExposure_dBA: number; // 평균 노출 소음도
}

/**
 * Health effect data
 */
export interface HealthEffectData {
  affectedPopulation: number;
  percentage: number;
}

/**
 * Health effects
 */
export interface HealthEffects {
  sleepDisturbance: HealthEffectData;
  annoyance: {
    highlyAnnoyed: number;
    percentage: number;
  };
  cardiovascular: {
    attributableCases: number;   // 귀인 가능 사례 수
    relativeRisk: number;        // 상대 위험도
  };
  cognitiveImpairment: {
    affectedChildren: number;
    learningDelay: number;       // 학습 지연 (%)
  };
}

/**
 * Economic impact
 */
export interface EconomicImpact {
  healthcareCost: number;      // 의료 비용 (원)
  productivityLoss: number;    // 생산성 손실 (원)
  propertyValueLoss: number;   // 부동산 가치 하락 (원)
  totalCost: number;          // 총 사회적 비용 (원)
}

/**
 * Health impact assessment
 */
export interface HealthImpactAssessment {
  assessmentId: string;
  period: {
    start: Timestamp;
    end: Timestamp;
  };
  region: string;
  exposure: ExposureAssessment;
  healthEffects: HealthEffects;
  economicImpact: EconomicImpact;
  recommendations: string[];
}

// ============================================================================
// Noise Regulations
// ============================================================================

/**
 * Zone type for regulations
 */
export enum RegulationZoneType {
  RESIDENTIAL = 'residential',                 // 주거지역
  ROADSIDE = 'roadside',                      // 도로변지역
  COMMERCIAL_INDUSTRIAL = 'commercial_industrial',  // 상업·공업지역
  GREEN = 'green',                            // 녹지지역
  SCHOOL_HOSPITAL = 'school_hospital',        // 학교·병원
}

/**
 * Noise limit for a time period
 */
export interface NoiseLimit {
  day: number;                 // 주간 기준 (dBA)
  evening: number;             // 저녁 기준 (dBA)
  night: number;               // 야간 기준 (dBA)
}

/**
 * Regulation standard
 */
export interface RegulationStandard {
  zoneType: RegulationZoneType;
  limits: NoiseLimit;
  authority: string;           // 규제 기관
  effectiveDate: Timestamp;    // 시행일
  reference?: string;          // 법규 참조
}

// ============================================================================
// Quiet Zones
// ============================================================================

/**
 * Quiet facility type
 */
export enum QuietFacilityType {
  SCHOOL = 'school',
  HOSPITAL = 'hospital',
  LIBRARY = 'library',
  KINDERGARTEN = 'kindergarten',
  ELDERLY_CARE = 'elderly_care',
}

/**
 * Quiet zone protection area
 */
export interface QuietZone {
  zoneId: string;
  facilityType: QuietFacilityType;
  name: string;
  location: Location;
  protectionRadius: number;    // 보호 거리 (m)
  limits: NoiseLimit;
  designatedDate: Timestamp;
  restrictions: string[];      // 제한 사항
}

// ============================================================================
// Noise Reduction Measures
// ============================================================================

/**
 * Mitigation strategy type
 */
export enum MitigationStrategyType {
  SOURCE = 'source',           // 발생원 대책
  PATH = 'path',              // 전파 경로 대책
  RECEIVER = 'receiver',      // 수음점 대책
}

/**
 * Barrier type
 */
export enum BarrierType {
  ABSORPTIVE = 'absorptive',   // 흡음형
  REFLECTIVE = 'reflective',   // 반사형
  TRANSPARENT = 'transparent', // 투명형
  COMPOSITE = 'composite',     // 복합형
}

/**
 * Noise barrier
 */
export interface NoiseBarrier {
  barrierId: string;
  type: BarrierType;
  location: Location;
  length: number;              // 길이 (m)
  height: number;              // 높이 (m)
  material: string;            // 재료
  transmission_loss: number;   // 투과 손실 (dB)
  installedDate: Timestamp;
  cost: number;               // 비용 (원)
}

/**
 * Mitigation strategy
 */
export interface MitigationStrategy {
  strategyId: string;
  type: MitigationStrategyType;
  description: string;
  expectedReduction_dB: number;  // 예상 저감 효과 (dB)
  cost: number;
  implementationDate?: Timestamp;
  effectiveness?: number;          // 실제 효과 (%)
}

// ============================================================================
// KPIs and Analytics
// ============================================================================

/**
 * Environmental KPIs
 */
export interface EnvironmentalKPIs {
  exceedanceReduction: number;     // 기준 초과율 감소 (%)
  averageNoiseReduction: number;   // 평균 소음도 감소 (dB)
  nightNoiseImprovement: number;   // 야간 소음 개선 (dB)
  quietAreaExpansion: number;      // 조용한 구역 확대 (km²)
}

/**
 * Health KPIs
 */
export interface HealthKPIs {
  exposedPopulationReduction: number;  // 고소음 노출 인구 감소 (%)
  sleepDisturbanceReduction: number;   // 수면 방해 감소 (%)
  annoyanceReduction: number;          // 불쾌감 감소 (%)
  healthCostSavings: number;           // 건강 비용 절감 (원)
}

/**
 * Management KPIs
 */
export interface ManagementKPIs {
  stationUptime: number;           // 측정소 가동률 (%)
  dataQuality: number;             // 데이터 품질 (%)
  complaintResolutionRate: number; // 민원 처리율 (%)
  complianceRate: number;          // 규제 준수율 (%)
}

/**
 * KPI dashboard
 */
export interface KPIDashboard {
  periodId: string;
  timestamp: Timestamp;
  region: string;
  environmental: EnvironmentalKPIs;
  health: HealthKPIs;
  management: ManagementKPIs;
  overallScore?: number;           // 0-100
}

// ============================================================================
// Realtime Data Stream
// ============================================================================

/**
 * Realtime noise update
 */
export interface RealtimeNoiseUpdate {
  stationId: string;
  timestamp: Timestamp;
  leq_dBA: number;
  lmax_dBA: number;
  status: 'normal' | 'warning' | 'alert';
}

/**
 * Alert type
 */
export enum AlertType {
  THRESHOLD_EXCEEDED = 'threshold_exceeded',
  SUDDEN_INCREASE = 'sudden_increase',
  EQUIPMENT_MALFUNCTION = 'equipment_malfunction',
  DATA_ANOMALY = 'data_anomaly',
}

/**
 * Noise alert
 */
export interface NoiseAlert {
  alertId: string;
  type: AlertType;
  stationId: string;
  timestamp: Timestamp;
  severity: 'low' | 'medium' | 'high' | 'critical';
  message: string;
  currentLevel_dBA: number;
  thresholdLevel_dBA?: number;
  acknowledged: boolean;
}

// ============================================================================
// Complaint Management
// ============================================================================

/**
 * Complaint status
 */
export enum ComplaintStatus {
  SUBMITTED = 'submitted',
  IN_REVIEW = 'in_review',
  INVESTIGATING = 'investigating',
  RESOLVED = 'resolved',
  CLOSED = 'closed',
}

/**
 * Noise complaint
 */
export interface NoiseComplaint {
  complaintId: string;
  submittedDate: Timestamp;
  location: Location;
  noiseSource: string;
  description: string;
  status: ComplaintStatus;
  assignedTo?: string;
  resolution?: string;
  resolvedDate?: Timestamp;
  followUp?: string;
}

// ============================================================================
// API Types
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
 * Noise heatmap data point
 */
export interface NoiseHeatmapPoint {
  coordinates: Coordinates;
  leq_dBA: number;
  category: 'low' | 'moderate' | 'high' | 'very_high';
}

/**
 * Statistical summary
 */
export interface StatisticalSummary {
  mean: number;
  median: number;
  min: number;
  max: number;
  stdDev: number;
  p95: number;                 // 95th percentile
}

// ============================================================================
// Certification
// ============================================================================

/**
 * Certification level
 */
export enum CertificationLevel {
  BRONZE = 'bronze',
  SILVER = 'silver',
  GOLD = 'gold',
  PLATINUM = 'platinum',
}

/**
 * Certification
 */
export interface Certification {
  certificateId: string;
  stationId: string;
  level: CertificationLevel;
  issueDate: Timestamp;
  expiryDate: Timestamp;
  verifiedBy: string;
  requirements: {
    instrumentClass: InstrumentClass;
    dataQuality: number;
    uptime: number;
  };
  status: 'active' | 'expired' | 'suspended';
}

// ============================================================================
// Export all types
// ============================================================================

export type {
  Timestamp,
  Coordinates,
  Location,
};
