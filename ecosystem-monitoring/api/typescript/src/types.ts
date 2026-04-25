/**
 * WIA-ENE-031: Ecosystem Monitoring Standard - TypeScript Type Definitions
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
// Ecosystem Classification
// ============================================================================

/**
 * Ecosystem types
 */
export enum EcosystemType {
  TEMPERATE_FOREST = 'temperate_forest',         // 온대림
  TROPICAL_FOREST = 'tropical_forest',           // 열대림
  BOREAL_FOREST = 'boreal_forest',               // 한대림
  MANGROVE = 'mangrove',                         // 맹그로브
  WETLAND = 'wetland',                           // 습지
  TIDAL_FLAT = 'tidal_flat',                     // 갯벌
  CORAL_REEF = 'coral_reef',                     // 산호초
  SEAGRASS_BED = 'seagrass_bed',                 // 해초대
  GRASSLAND = 'grassland',                       // 초원
  SAVANNA = 'savanna',                           // 사바나
  ALPINE_GRASSLAND = 'alpine_grassland',         // 고산 초원
  URBAN = 'urban',                               // 도시
  AGRICULTURAL = 'agricultural',                 // 농업
  RIPARIAN = 'riparian',                         // 습윤
  POLAR = 'polar',                               // 극지
}

/**
 * Ecosystem code (ECO-01 to ECO-10)
 */
export enum EcosystemCode {
  ECO_01 = 'ECO-01',  // 온대 활엽수림
  ECO_02 = 'ECO-02',  // 열대우림
  ECO_03 = 'ECO-03',  // 맹그로브
  ECO_04 = 'ECO-04',  // 갯벌
  ECO_05 = 'ECO-05',  // 산호초
  ECO_06 = 'ECO-06',  // 고산 초원
  ECO_07 = 'ECO-07',  // 도시 숲
  ECO_08 = 'ECO-08',  // 논 생태계
  ECO_09 = 'ECO-09',  // 하천 습지
  ECO_10 = 'ECO-10',  // 복원 생태계
}

/**
 * Health grade
 */
export enum HealthGrade {
  A = 'A',  // 우수
  B = 'B',  // 양호
  C = 'C',  // 보통
  D = 'D',  // 불량
  E = 'E',  // 매우 불량
}

// ============================================================================
// Ecosystem Basic Information
// ============================================================================

export interface EcosystemBasicInfo {
  // 식별 정보
  ecosystemId: string;
  ecosystemCode: EcosystemCode;
  name: string;
  type: EcosystemType;

  // 위치 정보
  location: {
    coordinates: Coordinates;
    area: number;                     // ha
    boundary: GeoJSON;
    region: string;
    country: string;
  };

  // 기후 정보
  climate: {
    zone: string;                     // 기후대
    meanAnnualTemp: number;           // °C
    meanAnnualPrecip: number;         // mm
    growingSeason: number;            // days
  };

  // 관리 정보
  management: {
    protectionStatus: string;
    manager: string;
    establishedDate: Timestamp;
    managementPlan?: string;
  };

  // 메타데이터
  metadata: {
    createdAt: Timestamp;
    updatedAt: Timestamp;
    dataQuality: number;              // 0-100
    verified: boolean;
  };
}

// ============================================================================
// Biodiversity Monitoring
// ============================================================================

export interface SpeciesData {
  scientificName: string;
  commonName?: string;
  abundance?: number;
  biomassDensity?: number;            // kg/ha
}

export interface DominantSpecies extends SpeciesData {
  abundance: number;
  biomassDensity: number;
}

export interface KeySpecies extends SpeciesData {
  populationSize: number;
  populationTrend: 'increasing' | 'stable' | 'decreasing';
}

export interface EndangeredSpecies extends SpeciesData {
  conservationStatus: string;         // IUCN Red List
  populationSize?: number;
  populationTrend?: 'increasing' | 'stable' | 'decreasing';
}

export interface BiodiversityData {
  monitoringId: string;
  ecosystemId: string;
  timestamp: Timestamp;

  // 종 다양성
  species: {
    flora: {
      totalSpecies: number;
      endemicSpecies: number;
      endangeredSpecies: number;
      invasiveSpecies: number;
      dominantSpecies: DominantSpecies[];
    };
    fauna: {
      totalSpecies: number;
      endemicSpecies: number;
      endangeredSpecies: number;
      keySpecies: KeySpecies[];
    };
    microorganisms?: {
      bacterialDiversity: number;
      fungalDiversity: number;
      soilMicrobiome?: {
        shannon: number;
        simpson: number;
      };
    };
  };

  // 다양성 지수
  indices: {
    shannonIndex: number;
    simpsonIndex: number;
    evenness: number;
    richness: number;
  };

  // 조사 방법
  method: {
    surveyType: string;
    samplingArea: number;             // m²
    samplingEffort: number;           // person-hours
    observers: string[];
  };
}

// ============================================================================
// Vegetation Health
// ============================================================================

export interface RemoteSensingIndex {
  mean: number;
  stdDev: number;
  min?: number;
  max?: number;
}

export interface VegetationHealthData {
  monitoringId: string;
  ecosystemId: string;
  timestamp: Timestamp;

  // 원격탐사 지수
  remoteSensingIndices: {
    ndvi: RemoteSensingIndex;         // -1 to 1
    evi: RemoteSensingIndex;
    lai: RemoteSensingIndex;          // m²/m²
    fpar: number;                     // 0-1
    gpp: number;                      // gC/m²/day
    npp: number;                      // gC/m²/day
  };

  // 현장 측정
  fieldMeasurements: {
    canopyCover: number;              // %
    canopyHeight: number;             // m
    basalArea: number;                // m²/ha
    stemDensity: number;              // stems/ha
    dbh: {
      mean: number;                   // cm
      distribution: {
        class: string;
        count: number;
      }[];
    };
  };

  // 생리적 지표
  physiologicalIndicators?: {
    chlorophyllContent: number;       // mg/g
    leafWaterPotential: number;       // MPa
    photosynthesisRate: number;       // μmol CO2/m²/s
    stomatalConductance: number;      // mmol H2O/m²/s
  };

  // 스트레스 지표
  stress: {
    droughtStress: number;            // 0-100
    heatStress: number;               // 0-100
    pestInfestation: number;          // %
    diseaseIncidence: number;         // %
  };
}

// ============================================================================
// Soil Health
// ============================================================================

export interface SoilHealthData {
  monitoringId: string;
  ecosystemId: string;
  timestamp: Timestamp;
  sampleLocation: {
    latitude: number;
    longitude: number;
    depth: number;                    // cm
  };

  // 물리적 특성
  physical: {
    texture: string;
    bulkDensity: number;              // g/cm³
    porosity: number;                 // %
    waterContent: number;             // %
    infiltrationRate: number;         // cm/hr
    aggregateStability: number;       // %
  };

  // 화학적 특성
  chemical: {
    ph: number;
    organicCarbon: number;            // %
    organicMatter: number;            // %
    totalNitrogen: number;            // %
    availablePhosphorus: number;      // mg/kg
    cec: number;                      // cmol/kg
    nutrients: {
      potassium: number;              // mg/kg
      calcium: number;
      magnesium: number;
      sulfur: number;
    };
    heavyMetals?: {
      lead: number;                   // mg/kg
      cadmium: number;
      arsenic: number;
      mercury: number;
    };
  };

  // 생물학적 특성
  biological: {
    microbialBiomass: number;         // mg C/kg
    enzymeActivity: {
      dehydrogenase: number;          // μg TPF/g/day
      urease: number;                 // μg NH4-N/g/hr
      phosphatase: number;            // μg PNP/g/hr
    };
    respiration: number;              // mg CO2/kg/day
    nematodeDiversity: number;
    earthwormDensity: number;         // individuals/m²
  };

  // 탄소 저장
  carbonStorage: {
    organicCarbon: number;            // Mg C/ha
    inorganicCarbon: number;          // Mg C/ha
    totalCarbon: number;              // Mg C/ha
    sequestrationRate: number;        // Mg C/ha/year
  };
}

// ============================================================================
// Water Quality
// ============================================================================

export interface WaterQualityData {
  monitoringId: string;
  ecosystemId: string;
  timestamp: Timestamp;
  sampleLocation: {
    waterBodyType: 'stream' | 'river' | 'lake' | 'wetland' | 'groundwater';
    latitude: number;
    longitude: number;
    depth?: number;                   // m
  };

  // 물리적 특성
  physical: {
    temperature: number;              // °C
    transparency: number;             // cm
    turbidity: number;                // NTU
    color: number;                    // Pt-Co
    conductivity: number;             // μS/cm
  };

  // 화학적 특성
  chemical: {
    ph: number;
    dissolvedOxygen: number;          // mg/L
    bod: number;                      // mg/L
    cod: number;                      // mg/L
    toc: number;                      // mg/L
    nutrients: {
      totalNitrogen: number;          // mg/L
      ammonium: number;
      nitrate: number;
      totalPhosphorus: number;
      phosphate: number;
    };
    ions: {
      chloride: number;               // mg/L
      sulfate: number;
      calcium: number;
      magnesium: number;
    };
  };

  // 생물학적 특성
  biological: {
    chlorophyllA: number;             // μg/L
    phytoplanktonDensity: number;     // cells/mL
    zooplanktonDensity: number;       // individuals/L
    bacterialCount: number;           // CFU/mL
    coliformBacteria: number;         // MPN/100mL
    benthicIndex?: number;
  };

  // 수문 특성
  hydrology?: {
    streamflow: number;               // m³/s
    waterLevel: number;               // m
    precipitation: number;            // mm
    evapotranspiration: number;       // mm
  };

  // 수질 등급
  qualityGrade: 'excellent' | 'good' | 'moderate' | 'poor' | 'very_poor';
}

// ============================================================================
// Carbon Sequestration
// ============================================================================

export interface CarbonSequestrationData {
  monitoringId: string;
  ecosystemId: string;
  timestamp: Timestamp;

  // 바이오매스 탄소
  biomassCarbon: {
    abovegroundBiomass: number;       // Mg/ha
    belowgroundBiomass: number;       // Mg/ha
    totalBiomass: number;             // Mg/ha
    carbonContent: number;            // %
    carbonStock: number;              // Mg C/ha
  };

  // 토양 탄소
  soilCarbon: {
    depth0to30cm: number;             // Mg C/ha
    depth30to100cm: number;           // Mg C/ha
    totalSoilCarbon: number;          // Mg C/ha
  };

  // 낙엽층 탄소
  litterCarbon: number;               // Mg C/ha

  // 총 탄소 저장
  totalCarbonStock: number;           // Mg C/ha

  // 탄소 흡수 속도
  sequestrationRate: {
    annual: number;                   // Mg C/ha/year
    perHectare: number;               // Mg C/ha/year
    totalAnnual: number;              // Mg C/year
  };

  // 탄소 플럭스
  carbonFlux: {
    gpp: number;                      // gC/m²/day
    npp: number;                      // gC/m²/day
    nep: number;                      // gC/m²/day
    nee: number;                      // gC/m²/day
    soilRespiration: number;          // gC/m²/day
  };

  // CO2 등가
  co2Equivalent: {
    sequestered: number;              // Mg CO2e/year
    avoided: number;                  // Mg CO2e/year
    total: number;                    // Mg CO2e/year
  };

  // 측정 방법
  methodology: {
    biomassMethod: string;
    soilSamplingMethod: string;
    fluxMeasurementMethod: string;
    uncertaintyRange: number;         // %
  };
}

// ============================================================================
// Hydrological Data
// ============================================================================

export interface HydrologicalData {
  ecosystemId: string;
  timestamp: Timestamp;

  // 강수
  precipitation: {
    rainfall: number;                 // mm
    snowfall: number;                 // mm water equivalent
    intensity: number;                // mm/hr
    duration: number;                 // hr
  };

  // 증발산
  evapotranspiration: {
    actual: number;                   // mm
    potential: number;                // mm
    evaporation: number;              // mm
    transpiration: number;            // mm
  };

  // 지표수
  surfaceWater: {
    streamflow: number;               // m³/s
    waterLevel: number;               // m
    inundatedArea: number;            // ha
    waterStorage: number;             // m³
  };

  // 지하수
  groundwater: {
    waterTableDepth: number;          // m
    rechargeRate: number;             // mm/year
    dischargeRate: number;            // m³/day
  };

  // 토양 수분
  soilMoisture: {
    depth0to10cm: number;             // %
    depth10to30cm: number;            // %
    depth30to100cm: number;           // %
    fieldCapacity: number;            // %
    wiltingPoint: number;             // %
  };

  // 물 순환 서비스
  ecosystemServices: {
    waterRetention: number;           // m³/year
    floodMitigation: number;          // m³
    waterPurification: number;        // 등급
  };
}

// ============================================================================
// Nutrient Cycling
// ============================================================================

export interface NitrogenCycleData {
  ecosystemId: string;
  timestamp: Timestamp;

  // 질소 저장고
  pools: {
    atmosphericN2: number;            // kg N/ha
    soilOrganicN: number;             // kg N/ha
    soilInorganicN: number;           // kg N/ha
    plantBiomassN: number;            // kg N/ha
    microbialBiomassN: number;        // kg N/ha
  };

  // 질소 플럭스
  fluxes: {
    atmosphericDeposition: number;    // kg N/ha/year
    biologicalFixation: number;       // kg N/ha/year
    mineralization: number;           // kg N/ha/year
    immobilization: number;           // kg N/ha/year
    nitrification: number;            // kg N/ha/year
    denitrification: number;          // kg N/ha/year
    plantUptake: number;              // kg N/ha/year
    leaching: number;                 // kg N/ha/year
    volatilization: number;           // kg N/ha/year
  };

  // 질소 수지
  budget: {
    inputs: number;                   // kg N/ha/year
    outputs: number;                  // kg N/ha/year
    netBalance: number;               // kg N/ha/year
  };

  // 질소 이용 효율
  efficiency: {
    nue: number;                      // %
    retentionRate: number;            // %
  };
}

export interface PhosphorusCycleData {
  ecosystemId: string;
  timestamp: Timestamp;

  // 인 저장고
  pools: {
    soilOrganicP: number;             // kg P/ha
    soilInorganicP: number;           // kg P/ha
    availableP: number;               // kg P/ha
    plantBiomassP: number;            // kg P/ha
    litterP: number;                  // kg P/ha
  };

  // 인 플럭스
  fluxes: {
    weathering: number;               // kg P/ha/year
    mineralization: number;           // kg P/ha/year
    immobilization: number;           // kg P/ha/year
    plantUptake: number;              // kg P/ha/year
    litterfall: number;               // kg P/ha/year
    leaching: number;                 // kg P/ha/year
    runoff: number;                   // kg P/ha/year
  };

  // 인 제한 여부
  limitation: {
    isLimiting: boolean;
    npRatio: number;
    criticalNPRatio: number;
  };
}

// ============================================================================
// Satellite Imagery
// ============================================================================

export interface SatelliteImageryData {
  imageId: string;
  ecosystemId: string;

  // 이미지 메타데이터
  metadata: {
    satellite: string;                // Sentinel-2, Landsat-8
    sensor: string;                   // MSI, OLI
    acquisitionDate: Timestamp;
    cloudCover: number;               // %
    sunElevation: number;             // °
    processingLevel: string;          // L1C, L2A
  };

  // 공간 정보
  spatial: {
    resolution: number;               // m
    extent: {
      minLat: number;
      maxLat: number;
      minLon: number;
      maxLon: number;
    };
    crs: string;                      // EPSG:4326
  };

  // 파생 지수
  indices: {
    ndvi: RemoteSensingIndex;
    evi: number;
    savi: number;
    ndwi: number;
    ndmi: number;
    nbr: number;
  };

  // 분류 결과
  classification?: {
    landCoverType: string;
    confidence: number;               // %
    changeDetection?: {
      changeType: string;
      changeMagnitude: number;
      previousDate: Timestamp;
    };
  };

  // 바이오매스 추정
  biomassEstimate?: {
    abovegroundBiomass: number;       // Mg/ha
    canopyHeight: number;             // m
    uncertainty: number;              // %
  };
}

// ============================================================================
// IoT Sensors
// ============================================================================

export interface WeatherSensorStation {
  stationId: string;
  ecosystemId: string;
  location: Coordinates;

  sensors: {
    temperature: {
      model: string;
      range: string;
      accuracy: string;
      interval: number;               // 분
    };
    humidity: {
      model: string;
      range: string;
      accuracy: string;
    };
    precipitation: {
      model: string;
      resolution: string;
      accuracy: string;
    };
    solarRadiation: {
      model: string;
      range: string;
      accuracy: string;
    };
    windSpeed: {
      model: string;
      range: string;
      accuracy: string;
    };
    windDirection: {
      model: string;
      range: string;
      accuracy: string;
    };
    barometricPressure: {
      model: string;
      range: string;
      accuracy: string;
    };
  };

  dataTransmission: {
    protocol: 'LoRaWAN' | '4G' | 'Satellite';
    frequency: number;                // 분
    powerSource: 'solar' | 'battery' | 'grid';
    batteryLevel?: number;            // %
  };

  measurements: {
    timestamp: Timestamp;
    temperature: number;              // °C
    humidity: number;                 // %
    precipitation: number;            // mm
    solarRadiation: number;           // W/m²
    windSpeed: number;                // m/s
    windDirection: number;            // °
    pressure: number;                 // hPa
  };
}

export interface SoilSensorArray {
  arrayId: string;
  ecosystemId: string;
  location: { latitude: number; longitude: number; };

  sensors: {
    moisture: {
      depths: number[];               // cm
      model: string;
      type: 'capacitance' | 'TDR' | 'tensiometer';
      accuracy: string;
    };
    temperature: {
      depths: number[];               // cm
      accuracy: string;
    };
    conductivity: {
      depths: number[];               // cm
      unit: 'dS/m';
    };
  };

  measurements: {
    timestamp: Timestamp;
    soilMoisture: {
      depth10cm: number;              // % VWC
      depth30cm: number;
      depth60cm: number;
      depth100cm: number;
    };
    soilTemperature: {
      depth10cm: number;              // °C
      depth30cm: number;
      depth60cm: number;
    };
    soilConductivity: {
      depth10cm: number;              // dS/m
      depth30cm: number;
    };
  };
}

export interface WaterQualitySensor {
  sensorId: string;
  ecosystemId: string;
  waterBodyType: 'stream' | 'lake' | 'wetland';
  location: { latitude: number; longitude: number; depth: number; };

  sensors: {
    multiparameter: {
      model: string;
      parameters: string[];
    };
  };

  measurements: {
    timestamp: Timestamp;
    dissolvedOxygen: number;          // mg/L
    oxygenSaturation: number;         // %
    ph: number;
    temperature: number;              // °C
    conductivity: number;             // μS/cm
    turbidity: number;                // NTU
    chlorophyllA?: number;            // μg/L
    blueGreenAlgae?: number;          // cells/mL
  };

  maintenance: {
    lastCalibration: Timestamp;
    nextCalibration: Timestamp;
    lastCleaning: Timestamp;
    sensorHealth: 'good' | 'fair' | 'poor';
  };
}

export interface AutomatedCO2Chamber {
  chamberId: string;
  ecosystemId: string;
  location: { latitude: number; longitude: number; };

  configuration: {
    chamberType: 'soil' | 'ecosystem';
    area: number;                     // m²
    volume: number;                   // L
    closureTime: number;              // seconds
    measurementFrequency: number;     // per day
  };

  sensors: {
    co2Analyzer: {
      model: string;
      range: string;
      accuracy: string;
    };
    temperature: {
      airTemp: boolean;
      soilTemp: boolean;
    };
    soilMoisture: boolean;
    par: boolean;
  };

  measurements: {
    timestamp: Timestamp;
    co2Flux: number;                  // μmol CO2/m²/s
    ch4Flux?: number;                 // nmol CH4/m²/s
    n2oFlux?: number;                 // nmol N2O/m²/s
    airTemp: number;                  // °C
    soilTemp: number;                 // °C
    soilMoisture: number;             // %
    par: number;                      // μmol/m²/s
  };
}

// ============================================================================
// Restoration Projects
// ============================================================================

export interface RestorationProject {
  projectId: string;
  ecosystemId: string;

  // 기본 정보
  basicInfo: {
    name: string;
    location: {
      address: string;
      coordinates: Coordinates;
      area: number;                   // ha
    };
    type: string;
    objective: string;
    startDate: Timestamp;
    plannedEndDate: Timestamp;
    actualEndDate?: Timestamp;
  };

  // 초기 상태
  baseline: {
    assessmentDate: Timestamp;
    degradationType: string;
    degradationSeverity: 'low' | 'medium' | 'high' | 'severe';
    initialConditions: {
      vegetationCover: number;        // %
      speciesRichness: number;
      soilHealth: number;             // 0-100
      waterQuality: number;           // 0-100
      carbonStock: number;            // Mg C/ha
    };
    threats: string[];
    photos: string[];
  };

  // 복원 계획
  plan: {
    targets: {
      vegetationCoverTarget: number;  // %
      speciesRichnessTarget: number;
      carbonStockTarget: number;      // Mg C/ha
      timeframe: number;              // years
    };
    activities: {
      activityId: string;
      activityType: string;
      description: string;
      scheduledDate: Timestamp;
      completedDate?: Timestamp;
      status: 'planned' | 'ongoing' | 'completed' | 'cancelled';
      budget: number;                 // 원
      actualCost?: number;            // 원
    }[];
    species: {
      scientificName: string;
      commonName: string;
      quantity: number;
      survivorRate?: number;          // %
      spacing: string;
      source: string;
    }[];
  };

  // 모니터링 데이터
  monitoringRecords: {
    recordId: string;
    date: Timestamp;
    vegetation: {
      cover: number;                  // %
      height: number;                 // m
      survivorRate: number;           // %
      speciesRichness: number;
      invasiveSpeciesCover: number;   // %
    };
    soil: {
      organicMatter: number;          // %
      ph: number;
      bulkDensity: number;            // g/cm³
    };
    carbonStock: number;              // Mg C/ha
    biodiversity: {
      birdSpecies: number;
      mammalSpecies: number;
      insectFamilies: number;
    };
    threats: {
      erosion: 'none' | 'low' | 'medium' | 'high';
      invasiveSpecies: 'none' | 'low' | 'medium' | 'high';
      disease: 'none' | 'low' | 'medium' | 'high';
      browsing: 'none' | 'low' | 'medium' | 'high';
    };
    photos: string[];
    notes: string;
  }[];

  // 성과 평가
  performance: {
    currentStatus: 'on_track' | 'needs_attention' | 'off_track' | 'success';
    achievementRate: number;          // %
    successCriteria: {
      criterionName: string;
      targetValue: number;
      currentValue: number;
      achieved: boolean;
    }[];
    lessonsLearned: string[];
    recommendations: string[];
  };

  // 이해관계자
  stakeholders: {
    organization: string;
    role: string;
    contactPerson: string;
    email: string;
    phone: string;
  }[];
}

// ============================================================================
// Protected Areas
// ============================================================================

export interface ProtectedArea {
  protectedAreaId: string;
  ecosystemId: string;

  // 기본 정보
  designation: {
    name: string;
    category: string;                 // IUCN 카테고리
    designation: string;
    internationalDesignation?: string;
    establishedDate: Timestamp;
    legalBasis: string;
  };

  // 공간 정보
  spatial: {
    totalArea: number;                // ha
    coreZoneArea: number;             // ha
    bufferZoneArea: number;           // ha
    transitionZoneArea: number;       // ha
    boundary: GeoJSON;
    zonation: {
      zoneId: string;
      zoneName: string;
      zoneType: 'core' | 'buffer' | 'transition' | 'restoration';
      area: number;                   // ha
      boundary: GeoJSON;
      allowedActivities: string[];
      restrictions: string[];
    }[];
  };

  // 생물다양성 가치
  biodiversityValue: {
    endemicSpecies: EndangeredSpecies[];
    endangeredSpecies: EndangeredSpecies[];
    keyBiodiversityArea: boolean;
    importanceLevel: 'global' | 'regional' | 'national' | 'local';
  };

  // 관리 계획
  managementPlan: {
    planId: string;
    planPeriod: {
      startYear: number;
      endYear: number;
    };
    vision: string;
    objectives: string[];
    strategies: string[];
    zonationStrategy: string;
    programs: {
      programId: string;
      programName: string;
      activities: {
        activityName: string;
        schedule: string;
        budget: number;               // 원
        responsible: string;
      }[];
    }[];
  };

  // 위협 평가
  threats: {
    threatType: string;
    severity: 'low' | 'medium' | 'high' | 'critical';
    scope: 'localized' | 'widespread';
    trend: 'increasing' | 'stable' | 'decreasing';
    description: string;
    mitigationMeasures: string[];
  }[];

  // 방문자 관리
  visitorManagement: {
    annualVisitors: number;
    maxCapacity: number;
    permitRequired: boolean;
    entryFee: number;                 // 원
    facilities: {
      facilityType: string;
      location: Coordinates;
      capacity: number;
      condition: 'excellent' | 'good' | 'fair' | 'poor';
    }[];
    interpretationPrograms: {
      programName: string;
      targetAudience: string;
      frequency: string;
      participants: number;
    }[];
  };

  // 집행 및 순찰
  enforcement: {
    patrolRecords: {
      patrolId: string;
      date: Timestamp;
      route: GeoJSON;
      duration: number;               // hours
      rangers: string[];
      observations: string[];
      violations?: {
        type: string;
        location: Coordinates;
        action: string;
      }[];
    }[];
    violations: {
      violationType: string;
      count: number;
      trend: 'increasing' | 'stable' | 'decreasing';
    }[];
  };

  // 관리 효과성 평가
  managementEffectiveness: {
    assessmentFramework: string;      // METT, RAPPAM
    assessmentDate: Timestamp;
    scores: {
      context: number;                // 0-100
      planning: number;               // 0-100
      inputs: number;                 // 0-100
      processes: number;              // 0-100
      outputs: number;                // 0-100
      outcomes: number;               // 0-100
    };
    overallScore: number;             // 0-100
    overallRating: 'poor' | 'fair' | 'good' | 'very_good' | 'excellent';
    strengths: string[];
    weaknesses: string[];
    recommendations: string[];
  };
}

// ============================================================================
// Ecosystem Health Index
// ============================================================================

export interface EcosystemHealthIndex {
  ecosystemId: string;
  timestamp: Timestamp;

  // 하위 지수
  subIndices: {
    biodiversityIndex: number;        // 0-100
    vegetationIndex: number;          // 0-100
    soilIndex: number;                // 0-100
    waterIndex: number;               // 0-100
    carbonIndex: number;              // 0-100
  };

  // 종합 지수
  overallEHI: number;                 // 0-100
  healthGrade: HealthGrade;

  // 추세
  trend: {
    direction: 'improving' | 'stable' | 'declining';
    changeRate: number;               // %
    comparedTo: string;
  };

  // 위협 요소
  threats: {
    type: string;
    severity: 'low' | 'medium' | 'high' | 'critical';
    description: string;
  }[];

  // 권고사항
  recommendations: string[];
}

// ============================================================================
// Carbon Credit
// ============================================================================

export interface CarbonCreditCalculation {
  projectId: string;
  ecosystemId: string;
  methodology: string;                // VCS, Gold Standard, CDM

  // 기준선
  baseline: {
    carbonStock: number;              // Mg C
    year: number;
  };

  // 프로젝트 시나리오
  project: {
    carbonStock: number;              // Mg C
    sequestrationRate: number;        // Mg C/year
    monitoringPeriod: number;         // years
  };

  // 탄소 크레딧
  credits: {
    totalReductions: number;          // Mg CO2e
    annualReductions: number;         // Mg CO2e/year
    verifiedCredits: number;          // tCO2e
    vintage: number;
  };

  // 누출 및 영속성
  adjustments: {
    leakage: number;                  // %
    permanenceDiscount: number;       // %
    uncertaintyDeduction: number;     // %
  };

  // 최종 크레딧
  netCredits: number;                 // tCO2e

  verification: {
    verifier: string;
    verificationDate: Timestamp;
    certificationBody: string;
    certificationNumber: string;
  };
}

// ============================================================================
// KPI Dashboard
// ============================================================================

export interface EnvironmentalKPIs {
  ecosystemHealthIndex: number;       // 0-100
  biodiversityIndex: number;          // 0-100
  vegetationNDVI: number;             // 0-1
  soilHealthIndex: number;            // 0-100
  waterQualityGrade: string;
}

export interface CarbonKPIs {
  totalCarbonStock: number;           // Mg C/ha
  carbonSequestrationRate: number;    // Mg C/ha/year
  co2Equivalent: number;              // Mg CO2e/year
}

export interface EcosystemServicesKPIs {
  waterRetention: number;             // m³/ha/year
  floodMitigation: number;            // m³
  waterPurification: number;          // 등급
  biodiversitySupport: number;        // 종 수
}

export interface KPIDashboard {
  ecosystemId: string;
  timestamp: Timestamp;
  environmental: EnvironmentalKPIs;
  carbon: CarbonKPIs;
  ecosystemServices: EcosystemServicesKPIs;
  overallScore?: number;              // 0-100
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
