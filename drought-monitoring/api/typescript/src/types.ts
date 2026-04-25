/**
 * WIA-ENE-034: Drought Monitoring Standard - TypeScript Type Definitions
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
  elevation?: number;
}

/**
 * GeoJSON geometry (simplified)
 */
export type GeoJSON = {
  type: 'Polygon' | 'MultiPolygon' | 'Point' | 'LineString';
  coordinates: number[] | number[][] | number[][][];
};

// ============================================================================
// Drought Classification
// ============================================================================

/**
 * Drought severity levels based on international standards
 */
export enum DroughtSeverity {
  NORMAL = 'normal',                       // 정상 (No Drought)
  ABNORMALLY_DRY = 'abnormally_dry',       // 이상 건조 (D0)
  MODERATE_DROUGHT = 'moderate_drought',   // 보통 가뭄 (D1)
  SEVERE_DROUGHT = 'severe_drought',       // 심한 가뭄 (D2)
  EXTREME_DROUGHT = 'extreme_drought',     // 극심한 가뭄 (D3)
  EXCEPTIONAL_DROUGHT = 'exceptional_drought', // 예외적 가뭄 (D4)
}

/**
 * Drought type classification
 */
export enum DroughtType {
  METEOROLOGICAL = 'meteorological',       // 기상학적 가뭄
  AGRICULTURAL = 'agricultural',           // 농업 가뭄
  HYDROLOGICAL = 'hydrological',           // 수문학적 가뭄
  SOCIOECONOMIC = 'socioeconomic',         // 사회경제적 가뭄
}

/**
 * Alert level for drought warnings
 */
export enum AlertLevel {
  NORMAL = 'normal',                       // 정상
  WATCH = 'watch',                         // 주의
  WARNING = 'warning',                     // 경고
  EMERGENCY = 'emergency',                 // 비상
}

// ============================================================================
// Drought Severity Indices
// ============================================================================

/**
 * Standardized Precipitation Index (SPI)
 * Time scales: 1, 3, 6, 12, 24 months
 */
export interface SPIData {
  monitoringId: string;
  location: Coordinates;
  timestamp: Timestamp;

  // SPI 값 (다양한 시간 척도)
  spi1Month: number;          // -3.0 ~ +3.0 (1개월)
  spi3Month: number;          // -3.0 ~ +3.0 (3개월)
  spi6Month: number;          // -3.0 ~ +3.0 (6개월)
  spi12Month: number;         // -3.0 ~ +3.0 (12개월)
  spi24Month: number;         // -3.0 ~ +3.0 (24개월)

  // 해석
  interpretation: {
    severity: DroughtSeverity;
    description: string;
    probability: number;      // % (발생 확률)
  };

  // 계산 메타데이터
  metadata: {
    baselinePeriod: string;   // e.g., "1981-2010"
    distribution: 'gamma' | 'pearson';
    dataQuality: number;      // 0-100
  };
}

/**
 * Palmer Drought Severity Index (PDSI)
 * Range: -10 (dry) to +10 (wet)
 */
export interface PDSIData {
  monitoringId: string;
  location: Coordinates;
  timestamp: Timestamp;

  // PDSI 값
  pdsi: number;               // -10 ~ +10

  // 관련 지수
  phdi: number;               // Palmer Hydrological Drought Index
  zIndex: number;             // Palmer Z-Index
  pmdi: number;               // Palmer Modified Drought Index

  // 입력 데이터
  inputs: {
    precipitation: number;    // mm
    temperature: number;      // °C
    potentialET: number;      // mm
    actualET: number;         // mm
    soilMoisture: number;     // mm
  };

  // 물 수지
  waterBalance: {
    surplus: number;          // mm
    deficit: number;          // mm
    runoff: number;           // mm
    recharge: number;         // mm
  };

  // 해석
  interpretation: {
    severity: DroughtSeverity;
    description: string;
    trend: 'worsening' | 'stable' | 'improving';
  };
}

/**
 * Standardized Precipitation Evapotranspiration Index (SPEI)
 * Combines precipitation and temperature (PET)
 */
export interface SPEIData {
  monitoringId: string;
  location: Coordinates;
  timestamp: Timestamp;

  // SPEI 값 (다양한 시간 척도)
  spei1Month: number;         // -3.0 ~ +3.0
  spei3Month: number;         // -3.0 ~ +3.0
  spei6Month: number;         // -3.0 ~ +3.0
  spei12Month: number;        // -3.0 ~ +3.0
  spei24Month: number;        // -3.0 ~ +3.0

  // 입력 변수
  inputs: {
    precipitation: number;    // mm
    potentialET: number;      // mm
    climateWaterBalance: number; // mm (P - PET)
  };

  // 해석
  interpretation: {
    severity: DroughtSeverity;
    description: string;
    droughtType: DroughtType;
  };

  // 메타데이터
  metadata: {
    petMethod: 'penman-monteith' | 'thornthwaite' | 'hargreaves';
    baselinePeriod: string;
  };
}

// ============================================================================
// Soil Moisture Monitoring
// ============================================================================

/**
 * Soil moisture data from sensors or satellite
 */
export interface SoilMoistureData {
  monitoringId: string;
  location: Coordinates;
  timestamp: Timestamp;

  // 깊이별 토양 수분 (부피 함수비, %)
  depthProfiles: {
    depth0to10cm: number;     // % VWC
    depth10to30cm: number;    // % VWC
    depth30to60cm: number;    // % VWC
    depth60to100cm: number;   // % VWC
  };

  // 토양 특성
  soilProperties: {
    soilType: string;
    fieldCapacity: number;    // % VWC
    wiltingPoint: number;     // % VWC
    saturation: number;       // % VWC
  };

  // 파생 지표
  derivedIndices: {
    plantAvailableWater: number;  // mm
    soilMoistureDeficit: number;  // mm
    relativeSoilMoisture: number; // % (현재/field capacity)
    soilMoistureAnomaly: number;  // % (현재 - 평년)
  };

  // 측정 방법
  source: {
    type: 'in-situ' | 'satellite' | 'model';
    sensor?: string;            // TDR, capacitance, etc.
    satellite?: string;         // SMAP, SMOS, Sentinel-1
    resolution?: number;        // km (위성인 경우)
  };

  // 가뭄 상태
  droughtStatus: {
    severity: DroughtSeverity;
    daysBelow30Percent: number;
    trend: 'declining' | 'stable' | 'increasing';
  };
}

// ============================================================================
// Groundwater Monitoring
// ============================================================================

/**
 * Groundwater level monitoring
 */
export interface GroundwaterData {
  monitoringId: string;
  wellId: string;
  location: Coordinates;
  timestamp: Timestamp;

  // 지하수위
  waterLevel: {
    depthToWater: number;         // m (지표로부터)
    elevation: number;            // m (해수면 기준)
    aquiferType: 'confined' | 'unconfined' | 'perched';
  };

  // 수위 변화
  levelChange: {
    dailyChange: number;          // m/day
    weeklyChange: number;         // m/week
    monthlyChange: number;        // m/month
    yearlyChange: number;         // m/year
    deviationFromNormal: number;  // m
  };

  // 수질 (선택사항)
  waterQuality?: {
    electricalConductivity: number; // μS/cm
    totalDissolvedSolids: number;   // mg/L
    temperature: number;             // °C
    ph: number;
  };

  // 대수층 특성
  aquiferProperties: {
    thickness: number;            // m
    transmissivity: number;       // m²/day
    storageCoefficient: number;   // dimensionless
    rechargeRate: number;         // mm/year
  };

  // 가뭄 영향
  droughtImpact: {
    severity: DroughtSeverity;
    percentileRank: number;       // 0-100 (역사적 비교)
    recoveryTime: number;         // days (예상)
  };
}

// ============================================================================
// Reservoir and Surface Water Storage
// ============================================================================

/**
 * Reservoir storage monitoring
 */
export interface ReservoirStorageData {
  monitoringId: string;
  reservoirId: string;
  reservoirName: string;
  location: Coordinates;
  timestamp: Timestamp;

  // 저수량
  storage: {
    currentVolume: number;        // million m³
    capacity: number;             // million m³
    percentFull: number;          // %
    deadStorage: number;          // million m³
    activeStorage: number;        // million m³
  };

  // 수위
  waterLevel: {
    currentLevel: number;         // m (해수면 기준)
    normalLevel: number;          // m
    fullSupplyLevel: number;      // m
    minOperatingLevel: number;    // m
  };

  // 저수율 통계
  statistics: {
    percentileRank: number;       // 0-100 (역사적)
    averageForDate: number;       // % (평년 동기)
    deviation: number;            // % (평년 대비)
    lowestEverForDate: number;    // %
    trend30Days: 'increasing' | 'stable' | 'decreasing';
  };

  // 유입/방류
  flow: {
    inflow: number;               // m³/s
    outflow: number;              // m³/s
    netChange: number;            // m³/day
    inflowAnomaly: number;        // % (평년 대비)
  };

  // 용수 공급
  waterSupply: {
    domesticSupply: number;       // m³/day
    irrigationSupply: number;     // m³/day
    industrialSupply: number;     // m³/day
    daysOfSupply: number;         // days (현재 소비 기준)
  };

  // 가뭄 상태
  droughtStatus: {
    severity: DroughtSeverity;
    alertLevel: AlertLevel;
    restrictionsInPlace: boolean;
  };
}

// ============================================================================
// Precipitation Monitoring
// ============================================================================

/**
 * Precipitation deficit analysis
 */
export interface PrecipitationDeficitData {
  monitoringId: string;
  location: Coordinates;
  timestamp: Timestamp;

  // 강수량 데이터
  precipitation: {
    current30Days: number;        // mm
    current90Days: number;        // mm
    current6Months: number;       // mm
    current12Months: number;      // mm
    waterYear: number;            // mm
  };

  // 평년값
  normal: {
    current30Days: number;        // mm
    current90Days: number;        // mm
    current6Months: number;       // mm
    current12Months: number;      // mm
    waterYear: number;            // mm
  };

  // 결핍량
  deficit: {
    absolute30Days: number;       // mm
    absolute90Days: number;       // mm
    absolute6Months: number;      // mm
    absolute12Months: number;     // mm
    absoluteWaterYear: number;    // mm
  };

  // 비율
  percentOfNormal: {
    current30Days: number;        // %
    current90Days: number;        // %
    current6Months: number;       // %
    current12Months: number;      // %
    waterYear: number;            // %
  };

  // 연속 무강우일
  dryDays: {
    consecutiveDays: number;
    longestThisSeason: number;
    averageThisSeason: number;
  };

  // 가뭄 영향
  impact: {
    severity: DroughtSeverity;
    droughtOnset: Timestamp;
    durationDays: number;
  };
}

// ============================================================================
// Vegetation Stress Monitoring
// ============================================================================

/**
 * Vegetation stress from remote sensing
 */
export interface VegetationStressData {
  monitoringId: string;
  location: Coordinates;
  timestamp: Timestamp;

  // 식생 지수
  indices: {
    ndvi: number;                 // -1 ~ +1 (Normalized Difference Vegetation Index)
    evi: number;                  // -1 ~ +1 (Enhanced Vegetation Index)
    vci: number;                  // 0-100 (Vegetation Condition Index)
    tci: number;                  // 0-100 (Temperature Condition Index)
    vhi: number;                  // 0-100 (Vegetation Health Index)
    ndwi: number;                 // -1 ~ +1 (Normalized Difference Water Index)
  };

  // 이상치 분석
  anomaly: {
    ndviAnomaly: number;          // (current - normal)
    eviAnomaly: number;           // (current - normal)
    percentileRank: number;       // 0-100
  };

  // 위성 데이터
  satellite: {
    sensor: string;               // MODIS, Sentinel-2, Landsat
    resolution: number;           // m
    acquisitionDate: Timestamp;
    cloudCover: number;           // %
  };

  // 식생 유형
  landCover: {
    type: string;                 // cropland, grassland, forest, etc.
    cropsGrown?: string[];        // 재배 작물
  };

  // 스트레스 평가
  stress: {
    severity: DroughtSeverity;
    stressLevel: number;          // 0-100
    affectedArea: number;         // km²
    trend: 'worsening' | 'stable' | 'recovering';
  };
}

// ============================================================================
// Agricultural Impact Assessment
// ============================================================================

/**
 * Agricultural drought impact
 */
export interface AgriculturalImpactData {
  monitoringId: string;
  location: Coordinates;
  timestamp: Timestamp;

  // 작물 정보
  cropInfo: {
    cropType: string;
    cropStage: string;            // planting, vegetative, reproductive, maturity
    plantingDate: Timestamp;
    expectedHarvestDate: Timestamp;
  };

  // 작물 상태
  cropCondition: {
    healthStatus: 'excellent' | 'good' | 'fair' | 'poor' | 'very_poor';
    stressLevel: number;          // 0-100
    canopyCover: number;          // %
    leafAreaIndex: number;        // m²/m²
    biomass: number;              // kg/ha
  };

  // 수분 스트레스
  waterStress: {
    cropWaterStressIndex: number; // 0-1
    soilMoistureAdequacy: number; // %
    irrigationRequired: number;   // mm
    ETcropDeficit: number;        // mm
  };

  // 수량 영향
  yieldImpact: {
    predictedYield: number;       // tons/ha
    normalYield: number;          // tons/ha
    yieldLoss: number;            // %
    economicLoss: number;         // USD/ha
  };

  // 관개
  irrigation: {
    method: string;               // drip, sprinkler, flood, none
    waterApplied: number;         // mm
    irrigationEfficiency: number; // %
    waterSource: 'groundwater' | 'surface_water' | 'reclaimed' | 'none';
  };

  // 가뭄 영향
  droughtSeverity: DroughtSeverity;

  // 권고사항
  recommendations: string[];
}

// ============================================================================
// Water Restrictions and Management
// ============================================================================

/**
 * Water use restrictions
 */
export interface WaterRestrictionData {
  restrictionId: string;
  jurisdiction: string;
  location: {
    region: string;
    country: string;
    boundary: GeoJSON;
  };

  // 제한 단계
  restrictionLevel: {
    stage: number;                // 1-4
    name: string;                 // e.g., "Stage 2 - Moderate"
    alertLevel: AlertLevel;
    effectiveDate: Timestamp;
    expectedDuration: number;     // days
  };

  // 제한 사항
  restrictions: {
    outdoor: {
      landscapeIrrigation: {
        allowed: boolean;
        daysAllowed?: string[];   // ["Mon", "Thu"]
        timeAllowed?: string;     // "before 10am or after 6pm"
        reductionTarget: number;  // %
      };
      carWashing: boolean;
      fountains: boolean;
      poolFilling: boolean;
    };
    indoor: {
      targetReduction: number;    // %
      prohibitedUses: string[];
    };
    agricultural: {
      irrigationCurtailment: number; // %
      priority: string;           // "junior rights curtailed first"
    };
    industrial: {
      reductionTarget: number;    // %
      recyclingRequired: boolean;
    };
  };

  // 위반 및 집행
  enforcement: {
    finesEnabled: boolean;
    firstOffenseFine: number;     // USD
    subsequentOffenseFine: number; // USD
    violationCount: number;
  };

  // 절수 목표
  conservationGoals: {
    dailyTarget: number;          // liters/person/day
    reductionFromBaseline: number; // %
    actualReduction: number;      // %
  };

  // 대중 홍보
  publicCommunication: {
    lastUpdate: Timestamp;
    website: string;
    hotline: string;
    socialMedia: string[];
  };
}

// ============================================================================
// Drought Declaration and Emergency Response
// ============================================================================

/**
 * Official drought declaration
 */
export interface DroughtDeclarationData {
  declarationId: string;
  timestamp: Timestamp;

  // 관할 구역
  jurisdiction: {
    level: 'national' | 'state' | 'county' | 'municipal';
    name: string;
    population: number;
    affectedArea: number;         // km²
    boundary: GeoJSON;
  };

  // 선언 내용
  declaration: {
    droughtType: DroughtType;
    severity: DroughtSeverity;
    alertLevel: AlertLevel;
    declaredDate: Timestamp;
    expectedEndDate?: Timestamp;
    status: 'active' | 'ended' | 'suspended';
  };

  // 트리거 조건 (선언 근거)
  triggers: {
    spiValue?: number;
    pdsiValue?: number;
    reservoirStorage?: number;    // %
    groundwaterLevel?: number;    // m below normal
    precipitationDeficit?: number; // %
    durationDays: number;
  };

  // 영향 평가
  impacts: {
    waterSupply: {
      affected: boolean;
      supplyReduction: number;    // %
      affectedPopulation: number;
    };
    agriculture: {
      affected: boolean;
      cropLossArea: number;       // ha
      livestockAffected: number;
      economicLoss: number;       // USD
    };
    environment: {
      affectedEcosystems: string[];
      wildlifeLoss: string;
      forestFireRisk: 'low' | 'moderate' | 'high' | 'extreme';
    };
    economy: {
      totalEconomicLoss: number;  // USD
      sectorsAffected: string[];
    };
  };

  // 대응 조치
  response: {
    emergencyFunding: number;     // USD
    waterRestrictionStage: number;
    droughtReliefPrograms: string[];
    agriculturalAssistance: boolean;
    publicAwareness: boolean;
  };

  // 복구 계획
  recovery: {
    planInPlace: boolean;
    estimatedRecoveryTime: number; // months
    requiredPrecipitation: number;  // mm
    monitoringFrequency: string;
  };

  // 문서
  documents: {
    officialNotice: string;       // URL
    impactAssessment: string;     // URL
    responseplan: string;         // URL
  };
}

// ============================================================================
// Integrated Drought Monitoring
// ============================================================================

/**
 * Comprehensive drought monitoring report
 */
export interface DroughtMonitoringReport {
  reportId: string;
  location: Coordinates;
  region: string;
  timestamp: Timestamp;

  // 종합 가뭄 지수
  compositeDroughtIndex: {
    value: number;                // 0-100
    severity: DroughtSeverity;
    confidence: number;           // 0-100
  };

  // 구성 요소
  components: {
    spi?: SPIData;
    pdsi?: PDSIData;
    spei?: SPEIData;
    soilMoisture?: SoilMoistureData;
    groundwater?: GroundwaterData;
    reservoirStorage?: ReservoirStorageData;
    precipitationDeficit?: PrecipitationDeficitData;
    vegetationStress?: VegetationStressData;
    agriculturalImpact?: AgriculturalImpactData;
  };

  // 추세 분석
  trend: {
    direction: 'worsening' | 'stable' | 'improving';
    changeRate: number;           // per week
    forecast7Day: DroughtSeverity;
    forecast30Day: DroughtSeverity;
  };

  // 영향 범위
  spatialExtent: {
    affectedArea: number;         // km²
    affectedPopulation: number;
    criticalInfrastructure: string[];
  };

  // 권고사항
  recommendations: {
    waterConservation: string[];
    agriculturalActions: string[];
    publicHealth: string[];
    ecosystemProtection: string[];
  };

  // 데이터 품질
  dataQuality: {
    completeness: number;         // 0-100
    spatialCoverage: number;      // %
    temporalResolution: string;   // daily, weekly, etc.
    lastUpdate: Timestamp;
  };
}

// ============================================================================
// Forecasting and Early Warning
// ============================================================================

/**
 * Drought forecast and early warning
 */
export interface DroughtForecastData {
  forecastId: string;
  location: Coordinates;
  issuedDate: Timestamp;
  validPeriod: {
    startDate: Timestamp;
    endDate: Timestamp;
  };

  // 예측 모델
  model: {
    name: string;
    version: string;
    inputData: string[];
    confidence: number;           // 0-100
  };

  // 기상 예측
  weatherForecast: {
    precipitation: {
      forecast30Day: number;      // mm
      forecast90Day: number;      // mm
      percentOfNormal30Day: number; // %
      percentOfNormal90Day: number; // %
    };
    temperature: {
      forecast30Day: number;      // °C
      forecast90Day: number;      // °C
      anomaly: number;            // °C above normal
    };
  };

  // 가뭄 예측
  droughtPrediction: {
    onset: {
      probability: number;        // %
      expectedDate?: Timestamp;
      triggeredBy: string[];
    };
    persistence: {
      probability: number;        // %
      expectedDuration: number;   // days
    };
    intensification: {
      probability: number;        // %
      peakSeverity: DroughtSeverity;
      peakDate?: Timestamp;
    };
    termination: {
      probability: number;        // %
      expectedDate?: Timestamp;
      requiredPrecipitation: number; // mm
    };
  };

  // 영향 예측
  anticipatedImpacts: {
    waterSupply: {
      supplyReduction: number;    // %
      affectedPopulation: number;
    };
    agriculture: {
      yieldLoss: number;          // %
      affectedCrops: string[];
    };
    environment: {
      wildfirerisk: 'low' | 'moderate' | 'high' | 'extreme';
      ecosystemsAtRisk: string[];
    };
  };

  // 조기 경보
  earlyWarning: {
    alertLevel: AlertLevel;
    leadTime: number;             // days
    recommendedActions: string[];
    preparednessMeasures: string[];
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
  GeoJSON,
};
