/**
 * WIA-ENE-XXX: Wetland Conservation Standard - TypeScript Type Definitions
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
  elevation?: number;  // meters above sea level
}

/**
 * GeoJSON polygon for wetland boundaries
 */
export interface WetlandBoundary {
  type: 'Polygon' | 'MultiPolygon';
  coordinates: number[][][];
}

// ============================================================================
// Wetland Classification
// ============================================================================

/**
 * Wetland types based on Ramsar Classification
 */
export enum WetlandType {
  // Inland wetlands
  MARSH = 'MARSH',                           // 습지 (초본류)
  SWAMP = 'SWAMP',                           // 늪지 (목본류)
  BOG = 'BOG',                               // 이탄지 (산성)
  FEN = 'FEN',                               // 습원 (알칼리성)
  PEATLAND = 'PEATLAND',                     // 토탄지
  FLOODPLAIN = 'FLOODPLAIN',                 // 범람원
  RIPARIAN = 'RIPARIAN',                     // 하천 습지
  LAKE = 'LAKE',                             // 호수
  POND = 'POND',                             // 연못
  SEASONAL_POOL = 'SEASONAL_POOL',           // 계절 습지

  // Coastal wetlands
  TIDAL_MARSH = 'TIDAL_MARSH',               // 조간대 습지
  SALT_MARSH = 'SALT_MARSH',                 // 염습지
  MANGROVE = 'MANGROVE',                     // 맹그로브
  MUDFLAT = 'MUDFLAT',                       // 갯벌
  ESTUARY = 'ESTUARY',                       // 하구
  LAGOON = 'LAGOON',                         // 석호

  // Human-made wetlands
  RICE_PADDY = 'RICE_PADDY',                 // 논
  RESERVOIR = 'RESERVOIR',                   // 저수지
  AQUACULTURE_POND = 'AQUACULTURE_POND',     // 양식장
  TREATMENT_WETLAND = 'TREATMENT_WETLAND',   // 처리 습지
}

/**
 * Hydrologic regime (water presence pattern)
 */
export enum HydrologicRegime {
  PERMANENTLY_FLOODED = 'PERMANENTLY_FLOODED',         // 영구 침수
  SEMI_PERMANENTLY_FLOODED = 'SEMI_PERMANENTLY_FLOODED', // 반영구 침수
  SEASONALLY_FLOODED = 'SEASONALLY_FLOODED',           // 계절적 침수
  TEMPORARILY_FLOODED = 'TEMPORARILY_FLOODED',         // 임시 침수
  INTERMITTENTLY_FLOODED = 'INTERMITTENTLY_FLOODED',   // 간헐적 침수
  SATURATED = 'SATURATED',                             // 포화
}

/**
 * Ramsar wetland designation
 */
export enum RamsarStatus {
  DESIGNATED = 'DESIGNATED',                 // 람사르 습지
  CANDIDATE = 'CANDIDATE',                   // 후보지
  NOT_DESIGNATED = 'NOT_DESIGNATED',         // 미지정
}

/**
 * Protection status
 */
export enum ProtectionStatus {
  NATIONAL_PARK = 'NATIONAL_PARK',           // 국립공원
  WILDLIFE_REFUGE = 'WILDLIFE_REFUGE',       // 야생동물보호구역
  NATURE_RESERVE = 'NATURE_RESERVE',         // 자연보호구역
  RAMSAR_SITE = 'RAMSAR_SITE',               // 람사르 습지
  PROTECTED_AREA = 'PROTECTED_AREA',         // 보호구역
  UNPROTECTED = 'UNPROTECTED',               // 미보호
}

// ============================================================================
// Wetland Basic Information
// ============================================================================

/**
 * Wetland site information
 */
export interface WetlandSite {
  siteId: string;
  name: string;
  wetlandType: WetlandType;

  // Location
  location: {
    coordinates: Coordinates;
    boundary: WetlandBoundary;
    area: number;                            // hectares
    country: string;
    region: string;
    watershed?: string;
  };

  // Classification
  classification: {
    hydrologicRegime: HydrologicRegime;
    ramsarStatus: RamsarStatus;
    protectionStatus: ProtectionStatus;
    ramsar_code?: string;                    // If Ramsar site
  };

  // Management
  management: {
    managingAuthority: string;
    establishedDate?: Timestamp;
    managementPlan?: string;
    zonation?: string[];
  };

  // Metadata
  metadata: {
    createdAt: Timestamp;
    updatedAt: Timestamp;
    dataQuality: number;                     // 0-100
    verified: boolean;
  };
}

// ============================================================================
// Water Level & Hydrology
// ============================================================================

/**
 * Water level measurement
 */
export interface WaterLevelMeasurement {
  measurementId: string;
  siteId: string;
  timestamp: Timestamp;

  // Water level data
  waterLevel: number;                        // meters
  waterDepth?: number;                       // meters
  surfaceArea?: number;                      // hectares
  volume?: number;                           // cubic meters

  // Flow data (for flowing wetlands)
  discharge?: number;                        // cubic meters per second
  velocity?: number;                         // meters per second

  // Measurement method
  method: 'STAFF_GAUGE' | 'PRESSURE_TRANSDUCER' | 'SATELLITE' | 'DRONE' | 'MANUAL';
  instrumentId?: string;
  accuracy?: number;                         // meters
}

/**
 * Water quality parameters
 */
export interface WaterQuality {
  measurementId: string;
  siteId: string;
  timestamp: Timestamp;
  location: Coordinates;

  // Physical parameters
  physical: {
    temperature: number;                     // °C
    turbidity: number;                       // NTU
    transparency?: number;                   // Secchi depth in meters
    color?: string;
    odor?: string;
  };

  // Chemical parameters
  chemical: {
    pH: number;
    dissolvedOxygen: number;                 // mg/L
    conductivity: number;                    // μS/cm
    salinity?: number;                       // ppt
    totalDissolvedSolids?: number;           // mg/L
    biochemicalOxygenDemand?: number;        // BOD, mg/L
    chemicalOxygenDemand?: number;           // COD, mg/L
  };

  // Nutrients
  nutrients: {
    totalNitrogen?: number;                  // mg/L
    nitrateNitrogen?: number;                // mg/L
    ammoniumNitrogen?: number;               // mg/L
    totalPhosphorus?: number;                // mg/L
    phosphate?: number;                      // mg/L
  };

  // Heavy metals (if applicable)
  heavyMetals?: {
    lead?: number;                           // μg/L
    mercury?: number;                        // μg/L
    cadmium?: number;                        // μg/L
    chromium?: number;                       // μg/L
    arsenic?: number;                        // μg/L
  };

  // Assessment
  qualityGrade: 'EXCELLENT' | 'GOOD' | 'FAIR' | 'POOR' | 'VERY_POOR';
  eutrophicationLevel?: 'OLIGOTROPHIC' | 'MESOTROPHIC' | 'EUTROPHIC' | 'HYPEREUTROPHIC';
}

/**
 * Hydrologic monitoring data
 */
export interface HydrologicData {
  monitoringId: string;
  siteId: string;
  period: {
    startDate: Timestamp;
    endDate: Timestamp;
  };

  // Water budget components
  waterBudget: {
    precipitation: number;                   // mm
    surfaceInflow: number;                   // cubic meters
    groundwaterInflow: number;               // cubic meters
    surfaceOutflow: number;                  // cubic meters
    groundwaterOutflow: number;              // cubic meters
    evapotranspiration: number;              // mm
    storageChange: number;                   // cubic meters
  };

  // Patterns
  floodingFrequency?: number;                // events per year
  dryingFrequency?: number;                  // events per year
  hydroperiod?: number;                      // days flooded per year
}

// ============================================================================
// Vegetation Monitoring
// ============================================================================

/**
 * Wetland vegetation types
 */
export enum VegetationType {
  EMERGENT = 'EMERGENT',                     // 정수식물 (갈대, 부들)
  SUBMERGENT = 'SUBMERGENT',                 // 침수식물 (말즘, 나자스말)
  FLOATING = 'FLOATING',                     // 부유식물 (부레옥잠)
  FLOATING_LEAVED = 'FLOATING_LEAVED',       // 부엽식물 (수련, 연꽃)
  TREE = 'TREE',                             // 습지 교목
  SHRUB = 'SHRUB',                           // 습지 관목
  MOSS = 'MOSS',                             // 이끼류
  ALGAE = 'ALGAE',                           // 조류
}

/**
 * Vegetation survey data
 */
export interface VegetationSurvey {
  surveyId: string;
  siteId: string;
  timestamp: Timestamp;
  location: Coordinates;

  // Survey method
  method: {
    surveyType: 'QUADRAT' | 'TRANSECT' | 'RELEVÉ' | 'AERIAL' | 'REMOTE_SENSING';
    plotSize?: number;                       // square meters
    replicates?: number;
  };

  // Community composition
  community: {
    dominantSpecies: string[];
    totalCover: number;                      // percentage
    speciesRichness: number;
    shannonDiversity?: number;
  };

  // Species records
  species: {
    scientificName: string;
    vegetationType: VegetationType;
    cover: number;                           // percentage
    frequency?: number;                      // 0-1
    abundance?: number;
    biomass?: number;                        // kg/m²
    height?: number;                         // meters
    phenology?: 'VEGETATIVE' | 'FLOWERING' | 'FRUITING' | 'SENESCENT';
  }[];

  // Invasive species
  invasiveSpecies: {
    scientificName: string;
    cover: number;                           // percentage
    invasiveness: 'LOW' | 'MODERATE' | 'HIGH' | 'SEVERE';
    controlMeasures?: string;
  }[];

  // Structural metrics
  structure: {
    canopyHeight?: number;                   // meters
    canopyCover?: number;                    // percentage
    litterDepth?: number;                    // cm
    standingDeadBiomass?: number;            // kg/m²
  };
}

// ============================================================================
// Species Inventory
// ============================================================================

/**
 * Wildlife observation in wetland
 */
export interface WetlandSpeciesObservation {
  observationId: string;
  siteId: string;
  timestamp: Timestamp;
  location: Coordinates;

  // Species information
  species: {
    scientificName: string;
    commonName: string;
    taxonomicGroup: 'BIRD' | 'MAMMAL' | 'REPTILE' | 'AMPHIBIAN' | 'FISH' | 'INVERTEBRATE';
    conservationStatus?: string;             // IUCN category
  };

  // Observation details
  observation: {
    count: number;
    lifeStage?: 'EGG' | 'LARVA' | 'JUVENILE' | 'ADULT';
    behavior?: string;
    habitat?: string;
    evidence?: 'VISUAL' | 'AUDIO' | 'TRACK' | 'NEST' | 'SCAT' | 'CAMERA_TRAP';
  };

  // Observer information
  observer: {
    observerId: string;
    expertise: 'EXPERT' | 'PROFESSIONAL' | 'AMATEUR' | 'CITIZEN_SCIENTIST';
    verified: boolean;
  };

  // Environmental context
  context?: {
    waterLevel?: number;
    temperature?: number;
    weather?: string;
  };
}

/**
 * Avian community assessment (birds are key wetland indicators)
 */
export interface AvianAssessment {
  assessmentId: string;
  siteId: string;
  timestamp: Timestamp;

  // Survey details
  survey: {
    method: 'POINT_COUNT' | 'TRANSECT' | 'AREA_SEARCH' | 'MIST_NET' | 'CAMERA';
    duration: number;                        // minutes
    distance?: number;                       // meters
    observers: number;
  };

  // Results
  results: {
    totalSpecies: number;
    totalIndividuals: number;
    waterbirds: number;
    migrants: number;
    residents: number;
    breeders: number;
  };

  // Key species
  keySpecies: {
    scientificName: string;
    count: number;
    status: 'BREEDING' | 'FEEDING' | 'RESTING' | 'MIGRATING';
  }[];

  // Diversity indices
  diversity: {
    shannonIndex: number;
    simpsonIndex: number;
    evenness: number;
  };
}

// ============================================================================
// Habitat Assessment
// ============================================================================

/**
 * Wetland condition indicators
 */
export interface WetlandCondition {
  assessmentId: string;
  siteId: string;
  assessmentDate: Timestamp;

  // Hydrologic condition
  hydrology: {
    waterRegimeIntegrity: number;            // 0-100
    surfaceWaterConnectivity: number;        // 0-100
    groundwaterConnectivity: number;         // 0-100
    floodingRegimeNatural: boolean;
  };

  // Vegetation condition
  vegetation: {
    nativeVegetationCover: number;           // percentage
    invasiveSpeciesCover: number;            // percentage
    structuralDiversity: number;             // 0-100
    vegetationIntegrity: number;             // 0-100
  };

  // Soil condition
  soil: {
    organicMatterContent: number;            // percentage
    soilType: string;
    hydricSoilIndicators: boolean;
    soilErosion: 'NONE' | 'LOW' | 'MODERATE' | 'HIGH' | 'SEVERE';
  };

  // Habitat quality
  habitat: {
    habitatDiversity: number;                // 0-100
    refugiaAvailability: number;             // 0-100
    connectivityScore: number;               // 0-100
    bufferZoneWidth: number;                 // meters
  };

  // Disturbances
  disturbances: {
    type: string;                            // e.g., 'drainage', 'pollution', 'development'
    severity: 'LOW' | 'MODERATE' | 'HIGH' | 'SEVERE';
    extent: number;                          // percentage of area
    trend: 'INCREASING' | 'STABLE' | 'DECREASING';
  }[];

  // Overall condition
  overallCondition: {
    score: number;                           // 0-100
    grade: 'EXCELLENT' | 'GOOD' | 'FAIR' | 'POOR' | 'DEGRADED';
    trend: 'IMPROVING' | 'STABLE' | 'DECLINING';
  };
}

/**
 * Functional assessment
 */
export interface FunctionalAssessment {
  assessmentId: string;
  siteId: string;
  assessmentDate: Timestamp;

  // Hydrologic functions
  hydrologicFunctions: {
    floodStorage: number;                    // 0-100
    floodFlowAlteration: number;             // 0-100
    groundwaterRecharge: number;             // 0-100
    streamflowMaintenance: number;           // 0-100
  };

  // Biogeochemical functions
  biogeochemicalFunctions: {
    nutrientCycling: number;                 // 0-100
    carbonSequestration: number;             // 0-100
    sedimentRetention: number;               // 0-100
    toxicantRetention: number;               // 0-100
  };

  // Biological functions
  biologicalFunctions: {
    habitatProvision: number;                // 0-100
    biodiversitySupport: number;             // 0-100
    foodWebSupport: number;                  // 0-100
    connectivityMaintenance: number;         // 0-100
  };

  // Overall functional capacity
  functionalCapacity: number;                // 0-100
}

// ============================================================================
// Ecosystem Services
// ============================================================================

/**
 * Ecosystem service types
 */
export enum EcosystemServiceType {
  // Provisioning services
  FRESHWATER_SUPPLY = 'FRESHWATER_SUPPLY',
  FOOD_PRODUCTION = 'FOOD_PRODUCTION',
  FIBER_PRODUCTION = 'FIBER_PRODUCTION',
  GENETIC_RESOURCES = 'GENETIC_RESOURCES',

  // Regulating services
  FLOOD_REGULATION = 'FLOOD_REGULATION',
  WATER_PURIFICATION = 'WATER_PURIFICATION',
  CLIMATE_REGULATION = 'CLIMATE_REGULATION',
  EROSION_CONTROL = 'EROSION_CONTROL',
  POLLINATION = 'POLLINATION',

  // Supporting services
  NUTRIENT_CYCLING = 'NUTRIENT_CYCLING',
  SOIL_FORMATION = 'SOIL_FORMATION',
  PRIMARY_PRODUCTION = 'PRIMARY_PRODUCTION',
  HABITAT_PROVISION = 'HABITAT_PROVISION',

  // Cultural services
  RECREATION = 'RECREATION',
  EDUCATION = 'EDUCATION',
  AESTHETIC_VALUE = 'AESTHETIC_VALUE',
  SPIRITUAL_VALUE = 'SPIRITUAL_VALUE',
  CULTURAL_HERITAGE = 'CULTURAL_HERITAGE',
}

/**
 * Ecosystem service valuation
 */
export interface EcosystemServiceValuation {
  valuationId: string;
  siteId: string;
  valuationDate: Timestamp;

  // Service quantification
  services: {
    serviceType: EcosystemServiceType;
    quantification: {
      metric: string;                        // e.g., 'cubic meters/year', 'kg N removed/year'
      value: number;
      unit: string;
    };
    beneficiaries?: {
      population: number;
      communities: string[];
    };
  }[];

  // Economic valuation (optional)
  economicValue?: {
    serviceType: EcosystemServiceType;
    monetaryValue: number;                   // USD per year
    valuationMethod: string;
    confidence: 'LOW' | 'MEDIUM' | 'HIGH';
  }[];

  // Total value
  totalAnnualValue?: {
    ecological: number;                      // composite score 0-100
    economic?: number;                       // USD per year
    social: number;                          // composite score 0-100
  };
}

// ============================================================================
// Restoration & Management
// ============================================================================

/**
 * Restoration project types
 */
export enum RestorationType {
  HYDROLOGIC_RESTORATION = 'HYDROLOGIC_RESTORATION',     // 수문 복원
  VEGETATION_RESTORATION = 'VEGETATION_RESTORATION',     // 식생 복원
  INVASIVE_REMOVAL = 'INVASIVE_REMOVAL',                 // 외래종 제거
  SOIL_RESTORATION = 'SOIL_RESTORATION',                 // 토양 복원
  HABITAT_CREATION = 'HABITAT_CREATION',                 // 서식지 조성
  CONNECTIVITY_RESTORATION = 'CONNECTIVITY_RESTORATION', // 연결성 복원
  POLLUTION_REMEDIATION = 'POLLUTION_REMEDIATION',       // 오염 정화
  COMPREHENSIVE = 'COMPREHENSIVE',                        // 종합 복원
}

/**
 * Restoration project
 */
export interface RestorationProject {
  projectId: string;
  siteId: string;
  projectName: string;
  restorationType: RestorationType;

  // Project timeline
  timeline: {
    plannedStart: Timestamp;
    plannedEnd: Timestamp;
    actualStart?: Timestamp;
    actualEnd?: Timestamp;
    status: 'PLANNED' | 'ONGOING' | 'COMPLETED' | 'MONITORING' | 'FAILED';
  };

  // Objectives
  objectives: {
    goal: string;
    targetMetrics: {
      metric: string;
      baselineValue: number;
      targetValue: number;
      unit: string;
    }[];
  };

  // Actions
  actions: {
    action: string;
    description: string;
    area?: number;                           // hectares
    implementationDate?: Timestamp;
    cost?: number;                           // USD
    responsibleParty: string;
  }[];

  // Monitoring plan
  monitoring: {
    parameters: string[];
    frequency: string;
    duration: number;                        // years
    methods: string[];
  };

  // Outcomes (for completed projects)
  outcomes?: {
    metric: string;
    achievedValue: number;
    targetValue: number;
    successRate: number;                     // percentage
  }[];

  // Lessons learned
  lessonsLearned?: string[];
}

/**
 * Management action
 */
export interface ManagementAction {
  actionId: string;
  siteId: string;
  actionType: 'MONITORING' | 'MAINTENANCE' | 'CONTROL' | 'ENHANCEMENT' | 'RESEARCH';

  action: {
    description: string;
    implementationDate: Timestamp;
    implementedBy: string;
    cost?: number;                           // USD
    area?: number;                           // hectares affected
  };

  effectiveness?: {
    evaluationDate: Timestamp;
    effectivenessScore: number;              // 0-100
    notes: string;
  };
}

// ============================================================================
// Threats & Pressures
// ============================================================================

/**
 * Threat to wetland
 */
export interface WetlandThreat {
  threatId: string;
  siteId: string;
  identifiedDate: Timestamp;

  threat: {
    category: 'HABITAT_LOSS' | 'POLLUTION' | 'INVASIVE_SPECIES' | 'OVEREXPLOITATION' |
              'CLIMATE_CHANGE' | 'HYDROLOGIC_ALTERATION' | 'DEVELOPMENT' | 'AGRICULTURE';
    description: string;
    source?: string;
  };

  assessment: {
    severity: 'LOW' | 'MODERATE' | 'HIGH' | 'CRITICAL';
    scope: number;                           // percentage of area affected
    timeframe: 'PAST' | 'ONGOING' | 'IMMINENT' | 'FUTURE';
    trend: 'INCREASING' | 'STABLE' | 'DECREASING' | 'UNKNOWN';
    reversibility: 'REVERSIBLE' | 'PARTIALLY_REVERSIBLE' | 'IRREVERSIBLE';
  };

  mitigation?: {
    measures: string[];
    effectiveness: number;                   // 0-100
    cost?: number;                           // USD
    responsibleParty?: string;
  };
}

// ============================================================================
// Monitoring & Reporting
// ============================================================================

/**
 * Monitoring program
 */
export interface MonitoringProgram {
  programId: string;
  siteId: string;
  programName: string;

  schedule: {
    startDate: Timestamp;
    endDate?: Timestamp;
    frequency: 'CONTINUOUS' | 'DAILY' | 'WEEKLY' | 'MONTHLY' | 'QUARTERLY' | 'ANNUAL';
    parameters: string[];
  };

  protocols: {
    parameter: string;
    method: string;
    standardReference?: string;
    qualityAssurance: string[];
  }[];

  responsibleParty: string;
  status: 'ACTIVE' | 'SUSPENDED' | 'COMPLETED';
}

/**
 * Wetland health report
 */
export interface WetlandHealthReport {
  reportId: string;
  siteId: string;
  reportingPeriod: {
    startDate: Timestamp;
    endDate: Timestamp;
  };

  summary: {
    overallHealth: number;                   // 0-100
    healthGrade: 'EXCELLENT' | 'GOOD' | 'FAIR' | 'POOR' | 'DEGRADED';
    trend: 'IMPROVING' | 'STABLE' | 'DECLINING';
  };

  indicators: {
    hydrology: number;                       // 0-100
    waterQuality: number;                    // 0-100
    vegetation: number;                      // 0-100
    wildlife: number;                        // 0-100
    soils: number;                           // 0-100
  };

  keyFindings: string[];
  recommendations: string[];
  generatedBy: string;
  generatedAt: Timestamp;
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
 * Location filter
 */
export interface LocationFilter {
  siteName?: string;
  wetlandType?: WetlandType;
  country?: string;
  region?: string;
  boundingBox?: {
    north: number;
    south: number;
    east: number;
    west: number;
  };
}

// ============================================================================
// Export all types
// ============================================================================

export type {
  Timestamp,
  Coordinates,
  WetlandBoundary,
};
