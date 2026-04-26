/**
 * WIA-ENE-029: Light Pollution Standard - TypeScript Type Definitions
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
  zone?: DarkSkyZone;
  region?: string;
  postalCode?: string;
}

// ============================================================================
// Light Pollution Measurement Types
// ============================================================================

/**
 * Luminance measurement (cd/m²)
 * 휘도 - 발광 표면의 밝기
 */
export interface Luminance {
  value: number;           // cd/m² (candela per square meter)
  unit: 'cd/m²';
  timestamp: Timestamp;
  location: Coordinates;
}

/**
 * Illuminance measurement (lux)
 * 조도 - 표면에 도달하는 빛의 양
 */
export interface Illuminance {
  value: number;           // lux
  unit: 'lux';
  timestamp: Timestamp;
  location: Coordinates;
  vertical?: number;       // 수직 조도 (lux)
  horizontal?: number;     // 수평 조도 (lux)
}

/**
 * Sky brightness measurement
 * 하늘 밝기 (천문학적 측정)
 */
export interface SkyBrightness {
  value: number;                    // mag/arcsec² (magnitudes per square arcsecond)
  unit: 'mag/arcsec²';
  timestamp: Timestamp;
  location: Coordinates;
  bortle: BortleScale;              // Bortle 어두운 하늘 척도
  sqm?: number;                     // Sky Quality Meter 측정값 (mag/arcsec²)
}

/**
 * Bortle Dark Sky Scale (1-9)
 * 1 = Excellent dark-sky site (가장 어두운 밤하늘)
 * 9 = Inner-city sky (도심 하늘)
 */
export enum BortleScale {
  EXCELLENT_DARK_SKY = 1,           // 우수한 어두운 하늘
  TYPICAL_DARK_SKY = 2,             // 일반적인 어두운 하늘
  RURAL_SKY = 3,                    // 시골 하늘
  RURAL_SUBURBAN_TRANSITION = 4,    // 시골-교외 경계
  SUBURBAN_SKY = 5,                 // 교외 하늘
  BRIGHT_SUBURBAN_SKY = 6,          // 밝은 교외 하늘
  SUBURBAN_URBAN_TRANSITION = 7,    // 교외-도심 경계
  CITY_SKY = 8,                     // 도시 하늘
  INNER_CITY_SKY = 9,               // 도심 하늘
}

/**
 * Color temperature measurement (Kelvin)
 * 색온도 - 빛의 색상 특성
 */
export interface ColorTemperature {
  value: number;           // K (Kelvin)
  unit: 'K';
  timestamp: Timestamp;
  classification: LightColor;
}

/**
 * Light color classification
 */
export enum LightColor {
  WARM_WHITE = 'warm_white',        // < 3000K (따뜻한 백색)
  NEUTRAL_WHITE = 'neutral_white',  // 3000-4000K (중성 백색)
  COOL_WHITE = 'cool_white',        // 4000-5000K (차가운 백색)
  DAYLIGHT = 'daylight',            // 5000-6500K (주광색)
  COOL_DAYLIGHT = 'cool_daylight',  // > 6500K (차가운 주광색)
}

/**
 * Blue light emission measurement
 * 청색광 방출량 (생체 리듬에 영향)
 */
export interface BlueLightEmission {
  percentage: number;               // 전체 스펙트럼 중 청색광 비율 (%)
  wavelength: number;               // 파장 (nm) - 일반적으로 400-500nm
  melanopicLux?: number;            // 멜라놉신 가중 조도
  circadianStimulus?: number;       // 일주기 자극 지수 (0-1)
  timestamp: Timestamp;
}

// ============================================================================
// Light Pollution Types
// ============================================================================

/**
 * Sky glow - 하늘 빛공해
 * 인공 조명이 대기에 산란되어 야간 하늘을 밝게 만드는 현상
 */
export interface SkyGlow {
  intensity: number;                // 강도 (0-100)
  skyBrightness: SkyBrightness;
  zenithLuminance: Luminance;       // 천정 휘도
  horizontalIlluminance: Illuminance; // 수평 조도
  affectedArea: number;             // 영향 받는 지역 (km²)
  visibleStars: number;             // 육안으로 보이는 별의 수
}

/**
 * Light trespass - 빛 침입
 * 원하지 않는 곳으로 빛이 침투하는 현상
 */
export interface LightTrespass {
  sourceLocation: Coordinates;
  targetLocation: Coordinates;
  illuminance: Illuminance;
  angle: number;                    // 입사각 (degrees)
  distance: number;                 // 거리 (m)
  severity: PollutionSeverity;
  affectedArea: 'window' | 'bedroom' | 'property' | 'wildlife_habitat';
}

/**
 * Glare - 눈부심
 * 과도한 밝기나 명암 대비로 인한 시각적 불편함
 */
export interface Glare {
  type: GlareType;
  luminance: Luminance;
  backgroundLuminance: number;      // 배경 휘도 (cd/m²)
  contrastRatio: number;            // 대비율
  viewingAngle: number;             // 시야각 (degrees)
  severity: PollutionSeverity;
  impactArea: 'road' | 'pedestrian' | 'residential' | 'workplace';
}

/**
 * Glare types
 */
export enum GlareType {
  DISABILITY_GLARE = 'disability_glare',     // 능력 저하 눈부심 (시야 감소)
  DISCOMFORT_GLARE = 'discomfort_glare',     // 불쾌 눈부심
  BLINDING_GLARE = 'blinding_glare',         // 실명 눈부심
}

/**
 * Pollution severity levels
 */
export enum PollutionSeverity {
  NONE = 'none',              // 없음
  MINIMAL = 'minimal',        // 최소
  LOW = 'low',                // 낮음
  MODERATE = 'moderate',      // 보통
  HIGH = 'high',              // 높음
  SEVERE = 'severe',          // 심각
  EXTREME = 'extreme',        // 극심
}

// ============================================================================
// Dark Sky Zones
// ============================================================================

/**
 * Dark Sky Zone classification
 * 어두운 하늘 구역 분류
 */
export enum DarkSkyZone {
  E1_INTRINSICALLY_DARK = 'E1',     // 본질적으로 어두운 구역 (국립공원, 천문대)
  E2_DARK = 'E2',                   // 어두운 구역 (농촌 지역)
  E3_RURAL = 'E3',                  // 시골 구역
  E4_SUBURBAN = 'E4',               // 교외 구역
  E5_URBAN = 'E5',                  // 도시 구역
}

/**
 * Lighting limits by zone
 * 구역별 조명 제한 기준
 */
export interface ZoneLightingLimits {
  zone: DarkSkyZone;
  maxVerticalIlluminance: number;   // lux
  maxUpwardLightRatio: number;      // % (0-100)
  maxColorTemperature: number;      // K
  curfewRequired: boolean;
  curfewHours: CurfewHours | null;
}

/**
 * Curfew hours
 * 소등 시간
 */
export interface CurfewHours {
  start: string;                    // HH:MM format
  end: string;                      // HH:MM format
  daysOfWeek?: number[];            // 0=Sunday, 1=Monday, ..., 6=Saturday
  exceptions?: string[];            // Special dates (YYYY-MM-DD)
}

// ============================================================================
// Environmental & Health Impacts
// ============================================================================

/**
 * Wildlife impact assessment
 * 야생동물 영향 평가
 */
export interface WildlifeImpact {
  affectedSpecies: string[];        // 영향 받는 종
  impactType: WildlifeImpactType[];
  severity: PollutionSeverity;
  location: Location;
  seasonalFactors: string[];        // 계절적 요인
  mitigationMeasures: string[];     // 완화 조치
}

/**
 * Types of wildlife impacts
 */
export enum WildlifeImpactType {
  NAVIGATION_DISRUPTION = 'navigation_disruption',         // 방향 감각 교란
  BREEDING_DISRUPTION = 'breeding_disruption',             // 번식 방해
  FEEDING_DISRUPTION = 'feeding_disruption',               // 섭식 방해
  MIGRATION_DISRUPTION = 'migration_disruption',           // 이동 경로 방해
  PREDATION_INCREASE = 'predation_increase',               // 포식 증가
  HABITAT_AVOIDANCE = 'habitat_avoidance',                 // 서식지 회피
  CIRCADIAN_DISRUPTION = 'circadian_disruption',           // 생체 리듬 교란
}

/**
 * Circadian rhythm effects
 * 일주기 리듬 영향
 */
export interface CircadianRhythmEffect {
  melanopicLux: number;             // 멜라놉신 가중 조도
  exposureDuration: number;         // 노출 시간 (minutes)
  exposureTime: string;             // 노출 시각 (HH:MM)
  melatoninSuppression: number;     // 멜라토닌 억제율 (%)
  sleepDisruptionRisk: 'low' | 'moderate' | 'high' | 'severe';
  healthImpact: HealthImpact;
}

/**
 * Health impacts
 */
export interface HealthImpact {
  sleepQuality: 'excellent' | 'good' | 'fair' | 'poor' | 'very_poor';
  melatoninLevel: 'normal' | 'reduced' | 'suppressed';
  riskFactors: HealthRiskFactor[];
  recommendations: string[];
}

/**
 * Health risk factors
 */
export enum HealthRiskFactor {
  SLEEP_DISRUPTION = 'sleep_disruption',               // 수면 장애
  CIRCADIAN_MISALIGNMENT = 'circadian_misalignment',   // 일주기 리듬 불일치
  INCREASED_CANCER_RISK = 'increased_cancer_risk',     // 암 위험 증가
  METABOLIC_DISORDERS = 'metabolic_disorders',         // 대사 장애
  MOOD_DISORDERS = 'mood_disorders',                   // 기분 장애
  COGNITIVE_IMPAIRMENT = 'cognitive_impairment',       // 인지 기능 저하
}

// ============================================================================
// Measurement & Monitoring
// ============================================================================

/**
 * Light pollution measurement event
 */
export interface LightPollutionMeasurement {
  measurementId: string;
  timestamp: Timestamp;
  location: Location;

  // 측정 데이터
  illuminance?: Illuminance;
  luminance?: Luminance;
  skyBrightness?: SkyBrightness;
  colorTemperature?: ColorTemperature;
  blueLightEmission?: BlueLightEmission;

  // 오염 유형
  skyGlow?: SkyGlow;
  lightTrespass?: LightTrespass;
  glare?: Glare;

  // 영향 평가
  wildlifeImpact?: WildlifeImpact;
  circadianEffect?: CircadianRhythmEffect;

  // 메타데이터
  metadata: MeasurementMetadata;
}

/**
 * Measurement metadata
 */
export interface MeasurementMetadata {
  deviceId?: string;
  deviceType: string;               // SQM, luxmeter, spectrometer 등
  calibrationDate?: Timestamp;
  operator?: string;
  weather: WeatherConditions;
  dataQuality: number;              // 0-100
  verified: boolean;
}

/**
 * Weather conditions
 */
export interface WeatherConditions {
  cloudCover: number;               // 구름 양 (%)
  visibility: number;               // 가시거리 (km)
  moonPhase: number;                // 달의 위상 (0-1, 0=신월, 1=만월)
  precipitation: boolean;           // 강수 여부
  temperature: number;              // 온도 (°C)
  humidity: number;                 // 습도 (%)
}

// ============================================================================
// Lighting Fixtures & Compliance
// ============================================================================

/**
 * Outdoor lighting fixture
 */
export interface LightingFixture {
  fixtureId: string;
  location: Location;
  type: FixtureType;

  // 기술 사양
  specifications: {
    wattage: number;                // 전력 (W)
    lumens: number;                 // 광속 (lm)
    colorTemperature: number;       // 색온도 (K)
    beamAngle: number;              // 빔 각도 (degrees)
    cutoffAngle: number;            // 차단 각도 (degrees)
    shielding: ShieldingType;
    upwardLightRatio: number;       // 상향광 비율 (%)
  };

  // 제어
  controls: {
    dimming: boolean;               // 조광 가능 여부
    motion: boolean;                // 동작 감지
    timer: boolean;                 // 타이머
    adaptiveLighting: boolean;      // 적응형 조명
  };

  // 규정 준수
  compliance: ComplianceStatus;
  installDate: Timestamp;
  lastInspection?: Timestamp;
}

/**
 * Fixture types
 */
export enum FixtureType {
  STREET_LIGHT = 'street_light',
  AREA_LIGHT = 'area_light',
  WALL_PACK = 'wall_pack',
  FLOOD_LIGHT = 'flood_light',
  DECORATIVE = 'decorative',
  SPORTS_FIELD = 'sports_field',
  BILLBOARD = 'billboard',
  ARCHITECTURAL = 'architectural',
}

/**
 * Shielding types
 */
export enum ShieldingType {
  FULLY_SHIELDED = 'fully_shielded',           // 완전 차폐 (0° 이상 상향광 없음)
  FULLY_CUTOFF = 'fully_cutoff',               // 완전 차단 (2.5% 이하 상향광)
  CUTOFF = 'cutoff',                           // 차단 (5% 이하 상향광)
  SEMI_CUTOFF = 'semi_cutoff',                 // 반차단 (20% 이하 상향광)
  NON_CUTOFF = 'non_cutoff',                   // 비차단 (상향광 제한 없음)
}

/**
 * Compliance status
 */
export interface ComplianceStatus {
  compliant: boolean;
  zone: DarkSkyZone;
  violations: ComplianceViolation[];
  certifications: string[];         // IDA, DarkSky 등 인증
  lastAudit?: Timestamp;
}

/**
 * Compliance violations
 */
export interface ComplianceViolation {
  violationType: ViolationType;
  severity: PollutionSeverity;
  description: string;
  detectedDate: Timestamp;
  resolved: boolean;
  resolutionDate?: Timestamp;
}

/**
 * Violation types
 */
export enum ViolationType {
  EXCESSIVE_ILLUMINANCE = 'excessive_illuminance',
  EXCESSIVE_UPWARD_LIGHT = 'excessive_upward_light',
  EXCESSIVE_COLOR_TEMP = 'excessive_color_temp',
  LIGHT_TRESPASS = 'light_trespass',
  GLARE = 'glare',
  CURFEW_VIOLATION = 'curfew_violation',
  IMPROPER_SHIELDING = 'improper_shielding',
}

// ============================================================================
// Alerts & Monitoring
// ============================================================================

/**
 * Light pollution alert
 */
export interface LightPollutionAlert {
  alertId: string;
  timestamp: Timestamp;
  location: Location;
  alertType: AlertType;
  severity: PollutionSeverity;
  measurement: LightPollutionMeasurement;
  affectedPopulation?: number;
  recommendations: string[];
  status: 'active' | 'acknowledged' | 'resolved';
}

/**
 * Alert types
 */
export enum AlertType {
  EXCESSIVE_SKY_GLOW = 'excessive_sky_glow',
  LIGHT_TRESPASS_COMPLAINT = 'light_trespass_complaint',
  GLARE_HAZARD = 'glare_hazard',
  WILDLIFE_THREAT = 'wildlife_threat',
  CURFEW_VIOLATION = 'curfew_violation',
  FIXTURE_MALFUNCTION = 'fixture_malfunction',
  COMPLIANCE_VIOLATION = 'compliance_violation',
}

// ============================================================================
// Analytics & Reporting
// ============================================================================

/**
 * Light pollution statistics
 */
export interface LightPollutionStatistics {
  periodId: string;
  startDate: Timestamp;
  endDate: Timestamp;
  region: string;

  // 측정 통계
  measurements: {
    total: number;
    averageIlluminance: number;           // lux
    averageSkyBrightness: number;         // mag/arcsec²
    averageColorTemperature: number;      // K
    complianceRate: number;               // %
  };

  // 구역별 분석
  byZone: {
    zone: DarkSkyZone;
    averageBortle: number;
    violationCount: number;
    complianceRate: number;
  }[];

  // 영향 평가
  impacts: {
    wildlifeAffected: number;             // 영향 받은 종의 수
    humanHealthRisk: 'low' | 'moderate' | 'high';
    energyWaste: number;                  // kWh
    economicImpact: number;               // 원
  };

  // 개선 현황
  improvements: {
    fixturesUpgraded: number;
    energySaved: number;                  // kWh
    lightReduced: number;                 // %
    darkSkyRecovered: number;             // km²
  };
}

/**
 * Performance report
 */
export interface PerformanceReport {
  reportId: string;
  reportType: ReportType;
  period: {
    start: Timestamp;
    end: Timestamp;
  };
  summary: LightPollutionStatistics;
  recommendations: string[];
  generatedBy?: string;
  generatedAt?: Timestamp;
}

/**
 * Report types
 */
export enum ReportType {
  DAILY = 'daily',
  WEEKLY = 'weekly',
  MONTHLY = 'monthly',
  QUARTERLY = 'quarterly',
  ANNUAL = 'annual',
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

// ============================================================================
// Export all types
// ============================================================================

export type {
  Timestamp,
  Coordinates,
  Location,
};
