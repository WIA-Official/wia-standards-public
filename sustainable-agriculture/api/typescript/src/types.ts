/**
 * WIA-ENE-016: Sustainable Agriculture Standard - TypeScript Type Definitions
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
  farmName: string;
  coordinates: Coordinates;
  address: string;
  region: string;
  country: string;
  zone?: string;  // climate zone
}

// ============================================================================
// Enumerations
// ============================================================================

/**
 * Farm types
 */
export enum FarmType {
  ORGANIC = 'ORGANIC',                     // 유기농
  CONVENTIONAL = 'CONVENTIONAL',           // 관행농업
  BIODYNAMIC = 'BIODYNAMIC',               // 바이오다이나믹
  PERMACULTURE = 'PERMACULTURE',           // 퍼머컬쳐
  REGENERATIVE = 'REGENERATIVE',           // 재생 농업
  HYDROPONIC = 'HYDROPONIC',               // 수경재배
  AQUAPONIC = 'AQUAPONIC',                 // 아쿠아포닉스
  VERTICAL = 'VERTICAL',                   // 수직농장
  MIXED = 'MIXED',                         // 혼합농업
}

/**
 * Crop categories
 */
export enum CropCategory {
  CEREAL = 'CEREAL',                       // 곡물
  VEGETABLE = 'VEGETABLE',                 // 채소
  FRUIT = 'FRUIT',                         // 과일
  LEGUME = 'LEGUME',                       // 콩과식물
  ROOT = 'ROOT',                           // 뿌리채소
  HERB = 'HERB',                           // 허브
  COVER_CROP = 'COVER_CROP',               // 피복작물
  GREEN_MANURE = 'GREEN_MANURE',           // 녹비작물
}

/**
 * Irrigation methods
 */
export enum IrrigationType {
  DRIP = 'DRIP',                           // 점적관개
  SPRINKLER = 'SPRINKLER',                 // 스프링클러
  SURFACE = 'SURFACE',                     // 표면관개
  SUBSURFACE = 'SUBSURFACE',               // 지하관개
  RAINWATER = 'RAINWATER',                 // 빗물수확
  SMART = 'SMART',                         // 스마트 관개
}

/**
 * Soil health indicators
 */
export enum SoilHealthIndicator {
  EXCELLENT = 'EXCELLENT',     // 우수
  GOOD = 'GOOD',               // 양호
  FAIR = 'FAIR',               // 보통
  POOR = 'POOR',               // 불량
  CRITICAL = 'CRITICAL',       // 위험
}

/**
 * Certification standards
 */
export enum CertificationStandard {
  USDA_ORGANIC = 'USDA_ORGANIC',
  EU_ORGANIC = 'EU_ORGANIC',
  JAS_ORGANIC = 'JAS_ORGANIC',
  IFOAM = 'IFOAM',
  DEMETER = 'DEMETER',
  RAINFOREST_ALLIANCE = 'RAINFOREST_ALLIANCE',
  FAIR_TRADE = 'FAIR_TRADE',
  GAP = 'GAP',                             // Good Agricultural Practice
  GLOBALG_A_P = 'GLOBALG_A_P',
  CARBON_NEUTRAL = 'CARBON_NEUTRAL',
}

// ============================================================================
// Farm Information
// ============================================================================

/**
 * Farm profile
 */
export interface FarmProfile {
  farmId: string;
  name: string;
  owner: string;
  location: Location;
  farmType: FarmType;
  totalArea: number;                       // hectares
  cultivatedArea: number;                  // hectares
  establishedDate: Timestamp;
  contactInfo: {
    email: string;
    phone: string;
    website?: string;
  };
  certifications: CertificationInfo[];
}

/**
 * Certification information
 */
export interface CertificationInfo {
  certificationId: string;
  standard: CertificationStandard;
  certifyingBody: string;
  issueDate: Timestamp;
  expiryDate: Timestamp;
  certificateNumber: string;
  scope: string[];                         // certified crops/products
  status: 'active' | 'pending' | 'expired' | 'suspended';
  auditHistory: AuditRecord[];
}

/**
 * Audit record
 */
export interface AuditRecord {
  auditId: string;
  auditDate: Timestamp;
  auditor: string;
  auditType: 'initial' | 'annual' | 'surveillance' | 'unannounced';
  findings: string[];
  nonCompliances: string[];
  correctiveActions: string[];
  nextAuditDate?: Timestamp;
}

// ============================================================================
// Crop Management
// ============================================================================

/**
 * Crop information
 */
export interface CropInfo {
  cropId: string;
  scientificName: string;
  commonName: string;
  variety: string;
  category: CropCategory;
  growingSeasonDays: number;
  waterRequirement: 'low' | 'medium' | 'high';
  nitrogenFixing: boolean;
  companionPlants?: string[];
  antagonisticPlants?: string[];
}

/**
 * Crop rotation plan
 */
export interface CropRotationPlan {
  planId: string;
  farmId: string;
  planName: string;
  startDate: Timestamp;
  endDate: Timestamp;
  rotationCycle: number;                   // years
  fields: FieldRotation[];
  objectives: string[];                    // e.g., 'soil fertility', 'pest control'
  notes?: string;
}

/**
 * Field rotation schedule
 */
export interface FieldRotation {
  fieldId: string;
  fieldName: string;
  areaSize: number;                        // hectares
  seasons: SeasonCrop[];
}

/**
 * Season crop assignment
 */
export interface SeasonCrop {
  season: string;                          // e.g., 'Spring 2024', 'Fall 2024'
  crop: CropInfo;
  plantingDate: Timestamp;
  harvestDate: Timestamp;
  expectedYield: number;                   // kg/ha
  actualYield?: number;                    // kg/ha
}

/**
 * Planting record
 */
export interface PlantingRecord {
  recordId: string;
  fieldId: string;
  crop: CropInfo;
  plantingDate: Timestamp;
  plantingMethod: string;                  // e.g., 'direct seeding', 'transplanting'
  seedSource: string;
  seedVariety: string;
  density: number;                         // plants/ha
  rowSpacing?: number;                     // cm
  plantSpacing?: number;                   // cm
  organicSeed: boolean;
}

// ============================================================================
// Water Management
// ============================================================================

/**
 * Water source
 */
export interface WaterSource {
  sourceId: string;
  sourceType: 'well' | 'river' | 'reservoir' | 'municipal' | 'rainwater';
  capacity: number;                        // cubic meters
  quality: WaterQuality;
  sustainable: boolean;
  permitRequired: boolean;
  permitNumber?: string;
}

/**
 * Water quality
 */
export interface WaterQuality {
  pH: number;
  EC: number;                              // electrical conductivity (μS/cm)
  TDS: number;                             // total dissolved solids (ppm)
  hardness?: number;                       // mg/L CaCO3
  nitrate?: number;                        // mg/L
  contamination?: string[];
  testDate: Timestamp;
  certified: boolean;
}

/**
 * Irrigation system
 */
export interface IrrigationSystem {
  systemId: string;
  type: IrrigationType;
  coverage: number;                        // hectares
  efficiency: number;                      // percentage
  waterSource: WaterSource;
  schedulingMethod: 'manual' | 'timer' | 'sensor' | 'AI';
  sensors?: {
    soilMoisture: boolean;
    weather: boolean;
    flowMeter: boolean;
  };
}

/**
 * Water usage record
 */
export interface WaterUsageRecord {
  recordId: string;
  farmId: string;
  date: Timestamp;
  fieldId: string;
  volume: number;                          // cubic meters
  irrigationType: IrrigationType;
  duration: number;                        // minutes
  efficiency: number;                      // percentage
  cost?: number;
}

/**
 * Water conservation metrics
 */
export interface WaterConservationMetrics {
  totalWaterUsed: number;                  // cubic meters
  waterPerHectare: number;                 // cubic meters/ha
  waterPerYield: number;                   // cubic meters/kg
  rainwaterHarvested: number;              // cubic meters
  waterSavings: number;                    // percentage vs conventional
  efficiencyScore: number;                 // 0-100
}

// ============================================================================
// Soil Management
// ============================================================================

/**
 * Soil test results
 */
export interface SoilTestResults {
  testId: string;
  fieldId: string;
  sampleDate: Timestamp;
  laboratory: string;

  // Physical properties
  texture: string;                         // e.g., 'sandy loam', 'clay'
  structure: string;
  depth: number;                           // cm

  // Chemical properties
  pH: number;
  organicMatter: number;                   // percentage
  nitrogen: number;                        // ppm
  phosphorus: number;                      // ppm
  potassium: number;                       // ppm
  calcium: number;                         // ppm
  magnesium: number;                       // ppm
  sulfur: number;                          // ppm
  CEC: number;                             // cation exchange capacity (meq/100g)

  // Biological properties
  microbialBiomass?: number;               // mg C/kg soil
  earthwormCount?: number;                 // per m²

  // Heavy metals
  lead?: number;                           // ppm
  cadmium?: number;                        // ppm
  mercury?: number;                        // ppm

  // Overall assessment
  healthIndicator: SoilHealthIndicator;
  recommendations: string[];
}

/**
 * Soil amendment
 */
export interface SoilAmendment {
  amendmentId: string;
  type: string;                            // e.g., 'compost', 'biochar', 'lime'
  organic: boolean;
  applicationDate: Timestamp;
  fieldId: string;
  amount: number;                          // kg/ha
  purpose: string[];                       // e.g., ['pH adjustment', 'nutrient']
  source: string;
  certified?: boolean;
  NPKRatio?: string;                       // e.g., '10-10-10'
}

/**
 * Soil conservation practice
 */
export interface SoilConservationPractice {
  practiceId: string;
  practiceType: string;                    // e.g., 'cover cropping', 'no-till', 'contour farming'
  implementationDate: Timestamp;
  areaApplied: number;                     // hectares
  objectives: string[];
  benefits: {
    erosionReduction: number;              // percentage
    carbonSequestration: number;           // tons CO2/ha/year
    waterRetention: number;                // percentage improvement
  };
  ongoing: boolean;
}

// ============================================================================
// Biodiversity & Ecosystem
// ============================================================================

/**
 * Biodiversity assessment
 */
export interface BiodiversityAssessment {
  assessmentId: string;
  farmId: string;
  assessmentDate: Timestamp;

  // Flora
  plantSpeciesCount: number;
  nativePlants: number;
  beneficialPlants: number;
  invasiveSpecies: string[];

  // Fauna
  pollinatorSpecies: number;
  birdSpecies: number;
  beneficialInsects: string[];
  pestInsects: string[];

  // Ecosystem services
  pollination: 'high' | 'medium' | 'low';
  pestControl: 'high' | 'medium' | 'low';
  soilFormation: 'high' | 'medium' | 'low';

  // Habitat
  hedgerows: number;                       // meters
  wetlands: number;                        // hectares
  forestArea: number;                      // hectares
  wildlifeCorridor: boolean;

  biodiversityScore: number;               // 0-100
  recommendations: string[];
}

/**
 * Beneficial organism
 */
export interface BeneficialOrganism {
  organismId: string;
  scientificName: string;
  commonName: string;
  type: 'pollinator' | 'predator' | 'decomposer' | 'nitrogen-fixer';
  benefits: string[];
  habitat: string[];
  conservationStatus?: string;
}

// ============================================================================
// Carbon & Climate
// ============================================================================

/**
 * Carbon footprint assessment
 */
export interface CarbonFootprint {
  assessmentId: string;
  farmId: string;
  period: {
    startDate: Timestamp;
    endDate: Timestamp;
  };

  // Emissions (tons CO2e)
  emissions: {
    fertilizers: number;
    machinery: number;
    irrigation: number;
    livestock?: number;
    transport: number;
    other: number;
    total: number;
  };

  // Sequestration (tons CO2e)
  sequestration: {
    soilCarbon: number;
    biomass: number;
    trees: number;
    total: number;
  };

  // Net carbon
  netCarbon: number;                       // negative = carbon positive
  carbonPerHectare: number;                // tons CO2e/ha
  carbonPerYield: number;                  // tons CO2e/kg

  // Comparison
  baselineYear?: string;
  reduction?: number;                      // percentage

  carbonNeutral: boolean;
  offset?: number;                         // tons CO2e
  methodology: string;
}

/**
 * Climate adaptation strategy
 */
export interface ClimateAdaptation {
  strategyId: string;
  farmId: string;
  risks: {
    drought: 'low' | 'medium' | 'high';
    flood: 'low' | 'medium' | 'high';
    heatWaves: 'low' | 'medium' | 'high';
    frost: 'low' | 'medium' | 'high';
    storms: 'low' | 'medium' | 'high';
  };
  adaptations: {
    drought: string[];
    flood: string[];
    temperature: string[];
  };
  implementation: {
    measure: string;
    status: 'planned' | 'in-progress' | 'completed';
    effectiveness?: number;                // 0-100
  }[];
}

// ============================================================================
// Pest & Disease Management
// ============================================================================

/**
 * Integrated pest management (IPM)
 */
export interface IPMRecord {
  recordId: string;
  farmId: string;
  date: Timestamp;
  fieldId: string;
  pest: {
    name: string;
    type: 'insect' | 'disease' | 'weed' | 'rodent';
    severity: 'low' | 'medium' | 'high';
  };
  monitoring: {
    method: string;                        // e.g., 'scouting', 'trap', 'sensor'
    threshold: number;
    observed: number;
  };
  action: {
    type: 'cultural' | 'biological' | 'mechanical' | 'chemical';
    method: string;
    product?: string;
    organic: boolean;
    effectiveness: number;                 // 0-100
  };
  nonChemicalFirst: boolean;
}

// ============================================================================
// Production & Yield
// ============================================================================

/**
 * Harvest record
 */
export interface HarvestRecord {
  recordId: string;
  farmId: string;
  fieldId: string;
  crop: CropInfo;
  harvestDate: Timestamp;
  quantity: number;                        // kg
  yieldPerHectare: number;                 // kg/ha
  quality: 'A' | 'B' | 'C' | 'reject';
  organic: boolean;
  marketValue?: number;
  destination: string;                     // e.g., 'fresh market', 'processing'
}

/**
 * Production metrics
 */
export interface ProductionMetrics {
  farmId: string;
  period: {
    startDate: Timestamp;
    endDate: Timestamp;
  };
  totalYield: number;                      // kg
  yieldPerHectare: number;                 // kg/ha
  organicProportion: number;               // percentage
  cropDiversity: number;                   // number of different crops
  revenue: number;
  profitability: number;                   // percentage
}

// ============================================================================
// Sustainability Metrics
// ============================================================================

/**
 * Sustainability KPIs
 */
export interface SustainabilityKPIs {
  environmental: {
    waterEfficiency: number;               // 0-100
    soilHealth: number;                    // 0-100
    biodiversity: number;                  // 0-100
    carbonFootprint: number;               // tons CO2e
    pesticidesReduction: number;           // percentage vs conventional
    energyUse: number;                     // kWh/ha
  };
  social: {
    fairWages: boolean;
    workerSafety: number;                  // 0-100
    communityEngagement: number;           // 0-100
    knowledgeSharing: number;              // 0-100
  };
  economic: {
    profitability: number;                 // percentage
    costReduction: number;                 // percentage
    marketAccess: number;                  // 0-100
    premiumPrice: number;                  // percentage
  };
  overallScore: number;                    // 0-100
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
// Event Types
// ============================================================================

/**
 * Event types for real-time updates
 */
export enum EventType {
  FARM_CREATED = 'FARM_CREATED',
  CERTIFICATION_UPDATED = 'CERTIFICATION_UPDATED',
  CROP_PLANTED = 'CROP_PLANTED',
  HARVEST_COMPLETED = 'HARVEST_COMPLETED',
  WATER_USAGE_RECORDED = 'WATER_USAGE_RECORDED',
  SOIL_TEST_COMPLETED = 'SOIL_TEST_COMPLETED',
  PEST_DETECTED = 'PEST_DETECTED',
  CARBON_ASSESSED = 'CARBON_ASSESSED',
  ALERT = 'ALERT',
}

/**
 * Event payload
 */
export interface EventPayload {
  eventId: string;
  type: EventType;
  timestamp: Timestamp;
  farmId: string;
  data: any;
  severity?: 'info' | 'warning' | 'critical';
}

// ============================================================================
// Export all types
// ============================================================================

export type {
  Timestamp,
  Coordinates,
  Location,
};
