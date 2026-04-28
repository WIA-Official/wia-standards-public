/**
 * WIA-ENE-039: Resource Depletion Response Standard - TypeScript Type Definitions
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
  address?: string;
  coordinates: Coordinates;
  country: string;
  region?: string;
}

// ============================================================================
// Resource Classification
// ============================================================================

/**
 * Resource category codes (RD-01 to RD-08)
 */
export enum ResourceCategoryCode {
  FOSSIL_FUELS = 'RD-01',        // 화석 연료
  METALS = 'RD-02',              // 금속
  NON_METALLIC = 'RD-03',        // 비금속 광물
  WATER = 'RD-04',               // 수자원
  SOIL = 'RD-05',                // 토양
  BIOLOGICAL = 'RD-06',          // 생물 자원
  STRATEGIC = 'RD-07',           // 전략 자원
  OTHER = 'RD-08',               // 기타 자원
}

/**
 * Depletion risk level
 */
export enum DepletionRiskLevel {
  SAFE = 1,          // 안전: R/P > 100년
  CAUTION = 2,       // 주의: 50년 < R/P < 100년
  WARNING = 3,       // 경고: 25년 < R/P < 50년
  DANGER = 4,        // 위험: 10년 < R/P < 25년
  CRITICAL = 5,      // 심각: R/P < 10년
}

/**
 * Resource renewability
 */
export enum Renewability {
  NON_RENEWABLE = 'non_renewable',           // 비재생
  RECYCLABLE = 'recyclable',                 // 재활용 가능
  RENEWABLE_SLOW = 'renewable_slow',         // 재생 속도 느림
  RENEWABLE_FAST = 'renewable_fast',         // 재생 가능
  LIMITED_RECYCLING = 'limited_recycling',   // 제한적 재활용
}

// ============================================================================
// Resource Status
// ============================================================================

/**
 * Reserve information
 */
export interface ReservesInfo {
  proven: number;              // 확인 매장량 (톤)
  probable: number;            // 가능 매장량 (톤)
  potential: number;           // 잠재 매장량 (톤)
  lastUpdated: Timestamp;
  confidence: number;          // 신뢰도 (0-100%)
  evaluationMethod?: string;
}

/**
 * Production information
 */
export interface ProductionInfo {
  annual: number;              // 연간 생산량 (톤/년)
  trend: number;               // 증감률 (%)
  topProducers: {
    country: string;
    quantity: number;
    percentage: number;
  }[];
  averageGrade?: number;       // 평균 품위 (%)
  recoveryRate?: number;       // 회수율 (%)
}

/**
 * Consumption information
 */
export interface ConsumptionInfo {
  annual: number;              // 연간 소비량 (톤/년)
  perCapita: number;           // 1인당 소비량 (kg/년)
  trend: number;               // 증감률 (%)
  topConsumers: {
    country: string;
    quantity: number;
    percentage: number;
  }[];
  sectorBreakdown?: {
    sector: string;            // 산업, 운송, 주거 등
    quantity: number;
    percentage: number;
  }[];
}

/**
 * Depletion metrics
 */
export interface DepletionMetrics {
  rpRatio: number;             // R/P 비율 (년)
  depletionRate: number;       // 고갈 속도 (%/년)
  riskLevel: DepletionRiskLevel;
  estimatedExhaustion: string; // 예상 고갈 시점 (YYYY)
  cumulativeDepletion: number; // 누적 고갈량 (%)
}

/**
 * Circularity metrics
 */
export interface CircularityMetrics {
  recyclingRate: number;       // 재활용률 (%)
  recoveryRate: number;        // 회수율 (%)
  circularityIndex: number;    // 순환 지수 (0-100)
  reuseRate?: number;          // 재사용률 (%)
  downcyclingRate?: number;    // 다운사이클링률 (%)
}

/**
 * Data metadata
 */
export interface DataMetadata {
  dataSource: string;
  quality: number;             // 데이터 품질 (0-100)
  verified: boolean;
  verifiedBy?: string;
  lastVerified: Timestamp;
  updateFrequency?: string;
}

/**
 * Comprehensive resource status
 */
export interface ResourceStatus {
  resourceId: string;
  resourceCode: ResourceCategoryCode;
  name: string;
  scientificName?: string;
  renewability: Renewability;

  reserves: ReservesInfo;
  production: ProductionInfo;
  consumption: ConsumptionInfo;
  depletion: DepletionMetrics;
  circularity: CircularityMetrics;
  metadata: DataMetadata;

  priceInfo?: {
    current: number;
    currency: string;
    trend: number;             // %
    volatility: number;        // % (연간 표준편차)
  };

  geopoliticalRisk?: {
    supplyConcentration: number; // HHI 지수
    conflictZoneProduction: number; // %
    tradeDependency: number;   // %
  };
}

// ============================================================================
// Depletion Forecasting
// ============================================================================

/**
 * Forecast model types
 */
export enum ForecastModelType {
  LINEAR = 'linear',
  EXPONENTIAL = 'exponential',
  LOGISTIC = 'logistic',
  MONTE_CARLO = 'monte_carlo',
  HUBBERT = 'hubbert',
  SYSTEM_DYNAMICS = 'system_dynamics',
}

/**
 * Scenario assumptions
 */
export interface ScenarioAssumptions {
  productionGrowth: number;    // 생산 증가율 (%/년)
  consumptionGrowth: number;   // 소비 증가율 (%/년)
  recyclingTarget: number;     // 재활용 목표 (%)
  substitutionRate: number;    // 대체율 (%/년)
  efficiencyGain: number;      // 효율성 향상 (%/년)
  newDiscoveries?: number;     // 신규 발견 (톤)
}

/**
 * Forecast result point
 */
export interface ForecastPoint {
  year: number;
  reserves: number;
  production: number;
  consumption: number;
  rpRatio: number;
  recycled: number;
  price?: number;
}

/**
 * Forecast scenario
 */
export interface ForecastScenario {
  name: string;                // BAU, Low, Medium, High
  description?: string;
  assumptions: ScenarioAssumptions;
  results: ForecastPoint[];
  exhaustionYear: number | null; // null = 고갈 없음
  confidence: number;          // 신뢰 구간 (%)
  probability?: number;        // 시나리오 발생 확률 (%)
}

/**
 * Sensitivity analysis
 */
export interface SensitivityAnalysis {
  parameter: string;
  baseValue: number;
  variation: number;           // ± %
  impact: number;              // R/P 비율 변화 (년)
  elasticity: number;          // 탄력성
}

/**
 * Depletion forecast
 */
export interface DepletionForecast {
  forecastId: string;
  resourceId: string;
  modelType: ForecastModelType;
  scenarios: ForecastScenario[];
  sensitivity: SensitivityAnalysis[];
  generatedAt: Timestamp;
  validUntil: Timestamp;
  generatedBy?: string;
}

// ============================================================================
// Mitigation Strategies
// ============================================================================

/**
 * Strategy types
 */
export enum StrategyType {
  EFFICIENCY = 'efficiency',           // 효율성 개선
  RECYCLING = 'recycling',             // 재활용
  SUBSTITUTION = 'substitution',       // 대체 개발
  CONSERVATION = 'conservation',       // 보존
  EXPLORATION = 'exploration',         // 탐사
  STRATEGIC_RESERVE = 'strategic_reserve', // 비축
}

/**
 * Implementation phase
 */
export interface ImplementationPhase {
  phase: string;
  startDate: Timestamp;
  endDate: Timestamp;
  milestones: {
    date: Timestamp;
    description: string;
    status: 'planned' | 'in_progress' | 'completed' | 'delayed';
  }[];
  budget?: number;
  responsible?: string;
}

/**
 * Expected impact
 */
export interface ExpectedImpact {
  resourceSaved: number;       // 절감 자원량 (톤/년)
  lifetimeExtension: number;   // 수명 연장 (년)
  costReduction: number;       // 비용 절감 (원/년)
  emissionsReduced: number;    // 배출 감축 (톤 CO2eq)
  jobsCreated?: number;        // 일자리 창출 (명)
}

/**
 * Economics of strategy
 */
export interface StrategyEconomics {
  investmentRequired: number;  // 필요 투자액 (원)
  operatingCost: number;       // 운영 비용 (원/년)
  paybackPeriod: number;       // 회수 기간 (년)
  roi: number;                 // 투자 수익률 (%)
  npv?: number;                // 순현재가치 (원)
  irr?: number;                // 내부수익률 (%)
}

/**
 * Mitigation strategy
 */
export interface MitigationStrategy {
  strategyId: string;
  resourceId: string;
  strategyType: StrategyType;

  details: {
    name: string;
    description: string;
    targetYear: number;
    targetMetrics: {
      metric: string;
      baseline: number;
      target: number;
      unit: string;
    }[];
  };

  implementation: ImplementationPhase[];
  expectedImpact: ExpectedImpact;
  economics: StrategyEconomics;

  status: 'proposed' | 'approved' | 'active' | 'completed' | 'cancelled' | 'paused';
  approvedBy?: string;
  startDate?: Timestamp;
  completionDate?: Timestamp;
}

// ============================================================================
// Early Warning System
// ============================================================================

/**
 * Indicator types
 */
export enum IndicatorType {
  DEPLETION_RATE = 'depletion_rate',
  PRICE_VOLATILITY = 'price_volatility',
  SUPPLY_RISK = 'supply_risk',
  GEOPOLITICAL = 'geopolitical',
  DEMAND_SURGE = 'demand_surge',
  RECYCLING_GAP = 'recycling_gap',
  TECHNOLOGICAL = 'technological',
}

/**
 * Alert level
 */
export enum AlertLevel {
  NORMAL = 'normal',
  CAUTION = 'caution',
  WARNING = 'warning',
  CRITICAL = 'critical',
}

/**
 * Trend direction
 */
export enum TrendDirection {
  IMPROVING = 'improving',
  STABLE = 'stable',
  DETERIORATING = 'deteriorating',
}

/**
 * Early warning indicator
 */
export interface EarlyWarningIndicator {
  indicatorId: string;
  resourceId: string;

  indicator: {
    name: string;
    type: IndicatorType;
    value: number;
    unit: string;
    threshold: {
      warning: number;
      critical: number;
    };
  };

  alert: {
    level: AlertLevel;
    triggered: boolean;
    triggerDate?: Timestamp;
    message: string;
    recommendations?: string[];
  };

  trend: {
    direction: TrendDirection;
    velocity: number;            // 변화 속도
    forecast: {
      thirtyDays: number;
      ninetyDays: number;
      oneYear: number;
    };
  };

  timestamp: Timestamp;
  nextUpdate?: Timestamp;
}

// ============================================================================
// Strategic Reserves
// ============================================================================

/**
 * Reserve status
 */
export enum ReserveStatus {
  NORMAL = 'normal',
  LOW = 'low',
  CRITICAL = 'critical',
  DEPLETED = 'depleted',
}

/**
 * Release criteria
 */
export interface ReleaseCriteria {
  trigger: string;
  condition: string;
  threshold: number;
  approvalRequired: boolean;
  maximumRelease?: number;     // 최대 방출량 (톤)
}

/**
 * Strategic reserve
 */
export interface StrategicReserve {
  reserveId: string;
  resourceId: string;
  name: string;

  inventory: {
    quantity: number;            // 현재 재고 (톤)
    target: number;              // 목표 재고 (톤)
    capacity: number;            // 저장 용량 (톤)
    daysOfSupply: number;        // 공급 일수
    lastRestocked: Timestamp;
  };

  status: ReserveStatus;

  releaseCriteria: ReleaseCriteria[];

  location: {
    facilities: {
      facilityId: string;
      location: Location;
      quantity: number;
      capacity: number;
      security: 'high' | 'maximum';
    }[];
  };

  governance: {
    authority: string;
    policies: string[];
    lastReview: Timestamp;
  };
}

// ============================================================================
// Substitution & Alternatives
// ============================================================================

/**
 * Substitution feasibility
 */
export enum SubstitutionFeasibility {
  HIGH = 'high',                // 쉽게 대체 가능
  MEDIUM = 'medium',            // 일부 대체 가능
  LOW = 'low',                  // 대체 어려움
  NONE = 'none',                // 대체 불가
}

/**
 * Alternative material
 */
export interface AlternativeMaterial {
  materialId: string;
  name: string;
  description: string;

  substitutionFor: {
    resourceId: string;
    applications: string[];      // 적용 분야
    feasibility: SubstitutionFeasibility;
    performanceRatio: number;    // 성능 비율 (%)
  };

  availability: {
    currentProduction: number;   // 현재 생산량 (톤/년)
    potentialProduction: number; // 잠재 생산량 (톤/년)
    scalability: 'low' | 'medium' | 'high';
  };

  economics: {
    costRatio: number;           // 비용 비율 (%)
    marketReadiness: 'research' | 'pilot' | 'commercial';
    adoptionRate: number;        // 채택률 (%/년)
  };

  environmental: {
    carbonFootprint: number;     // 탄소 발자국 (kg CO2eq/kg)
    waterUsage: number;          // 물 사용량 (L/kg)
    toxicity: 'low' | 'medium' | 'high';
  };
}

// ============================================================================
// Monitoring & Reporting
// ============================================================================

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

/**
 * Monitoring frequency
 */
export enum MonitoringFrequency {
  REALTIME = 'realtime',
  HOURLY = 'hourly',
  DAILY = 'daily',
  WEEKLY = 'weekly',
  MONTHLY = 'monthly',
  QUARTERLY = 'quarterly',
  ANNUALLY = 'annually',
}

/**
 * Resource report
 */
export interface ResourceReport {
  reportId: string;
  reportType: ReportType;
  period: {
    start: Timestamp;
    end: Timestamp;
  };

  summary: {
    resourcesCovered: number;
    criticalResources: number;
    averageRPRatio: number;
    totalRecyclingRate: number;
    circularityIndex: number;
  };

  details: {
    resourceId: string;
    status: ResourceStatus;
    forecast: DepletionForecast;
    alerts: EarlyWarningIndicator[];
    strategies: MitigationStrategy[];
  }[];

  recommendations: string[];
  generatedBy?: string;
  generatedAt: Timestamp;
}

// ============================================================================
// KPIs (Key Performance Indicators)
// ============================================================================

/**
 * Depletion prevention KPIs
 */
export interface DepletionPreventionKPIs {
  averageRPRatio: number;          // 평균 R/P 비율 (년)
  depletionRateReduction: number;  // 고갈 속도 감소율 (%)
  circularityRate: number;         // 순환율 (%)
  averageRecyclingRate: number;    // 평균 재활용률 (%)
  efficiencyImprovement: number;   // 효율성 개선 (%)
  resourcesSaved: number;          // 절감 자원량 (톤/년)
}

/**
 * Economic KPIs
 */
export interface EconomicKPIs {
  resourceProductivity: number;    // 자원 생산성 (GDP/톤)
  materialIntensity: number;       // 물질 강도 (톤/GDP)
  circularEconomyGDP: number;      // 순환 경제 GDP 기여 (%)
  costSavings: number;             // 비용 절감 (원/년)
  investmentROI: number;           // 투자 수익률 (%)
}

/**
 * Social KPIs
 */
export interface SocialKPIs {
  resourceSecurityIndex: number;   // 자원 안보 지수 (0-100)
  intergenerationEquity: number;   // 세대 간 형평성 (0-100)
  accessIndex: number;             // 접근성 지수 (0-100)
  jobsCreated: number;             // 일자리 창출 (명)
  publicAwareness: number;         // 대중 인식 (0-100)
}

/**
 * Comprehensive KPI dashboard
 */
export interface KPIDashboard {
  periodId: string;
  timestamp: Timestamp;
  depletionPrevention: DepletionPreventionKPIs;
  economic: EconomicKPIs;
  social: SocialKPIs;
  overallScore?: number;           // 종합 점수 (0-100)
}

// ============================================================================
// Digital Twin
// ============================================================================

/**
 * Real-time data
 */
export interface RealTimeData {
  production: number;
  consumption: number;
  price: number;
  inventory: number;
  recyclingRate: number;
  timestamp: Timestamp;
}

/**
 * Simulation scenario
 */
export interface SimulationScenario {
  scenarioId: string;
  name: string;
  parameters: Record<string, number>;
  results: ForecastPoint[];
  optimizationScore?: number;
}

/**
 * Optimization recommendation
 */
export interface OptimizationRecommendation {
  objective: string;
  constraints: string[];
  recommendation: string;
  expectedBenefit: number;
  implementationCost: number;
  priority: 'low' | 'medium' | 'high';
}

/**
 * Resource digital twin
 */
export interface ResourceDigitalTwin {
  twinId: string;
  resourceId: string;
  lastUpdated: Timestamp;

  realTimeData: RealTimeData;
  historicalData: RealTimeData[];

  simulation: SimulationScenario[];
  optimization: OptimizationRecommendation[];

  predictiveAnalytics: {
    nextWeek: ForecastPoint;
    nextMonth: ForecastPoint;
    nextYear: ForecastPoint;
  };
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
