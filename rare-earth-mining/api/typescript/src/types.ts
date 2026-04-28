/**
 * WIA-ENE-041: Rare Earth Mining Standard - TypeScript Type Definitions
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
  elevation_m?: number;
}

/**
 * Location information
 */
export interface Location {
  lat: number;
  lon: number;
  elevation_m?: number;
  address?: string;
  city?: string;
  state?: string;
  country: string;
  postalCode?: string;
}

// ============================================================================
// Rare Earth Elements
// ============================================================================

/**
 * Rare Earth Elements (17 total)
 */
export enum RareEarthElement {
  // Light REE (LREE)
  LANTHANUM = 'La',        // 란타넘 (57)
  CERIUM = 'Ce',           // 세륨 (58)
  PRASEODYMIUM = 'Pr',     // 프라세오디뮴 (59)
  NEODYMIUM = 'Nd',        // 네오디뮴 (60)
  PROMETHIUM = 'Pm',       // 프로메튬 (61) - 인공
  SAMARIUM = 'Sm',         // 사마륨 (62)
  EUROPIUM = 'Eu',         // 유로퓸 (63)

  // Heavy REE (HREE)
  GADOLINIUM = 'Gd',       // 가돌리늄 (64)
  TERBIUM = 'Tb',          // 테르븀 (65)
  DYSPROSIUM = 'Dy',       // 디스프로슘 (66)
  HOLMIUM = 'Ho',          // 홀뮴 (67)
  ERBIUM = 'Er',           // 어븀 (68)
  THULIUM = 'Tm',          // 툴륨 (69)
  YTTERBIUM = 'Yb',        // 이터븀 (70)
  LUTETIUM = 'Lu',         // 루테튬 (71)

  // Associated
  SCANDIUM = 'Sc',         // 스칸듐 (21)
  YTTRIUM = 'Y',           // 이트륨 (39)
}

/**
 * REE category
 */
export enum REECategory {
  LREE = 'LREE',  // Light REE (경희토류)
  HREE = 'HREE',  // Heavy REE (중희토류)
  ASSOCIATED = 'ASSOCIATED',  // Sc, Y
}

/**
 * Strategic importance level
 */
export enum StrategicImportance {
  CRITICAL = 'CRITICAL',      // 매우 중요 (Nd, Dy, Tb, Eu, Y)
  HIGH = 'HIGH',              // 중요 (Pr, Sm, Gd)
  MODERATE = 'MODERATE',      // 보통 (La, Ce, Er)
}

/**
 * REE element information
 */
export interface REEElementInfo {
  element: RareEarthElement;
  atomicNumber: number;
  category: REECategory;
  strategicImportance: StrategicImportance;
  commonCompounds: string[];  // e.g., ["Nd2O3", "NdFeB"]
  primaryUses: string[];      // e.g., ["Permanent magnets", "Lasers"]
}

// ============================================================================
// Ore Types
// ============================================================================

/**
 * Types of rare earth ore deposits
 */
export enum OreType {
  BASTNASITE = 'BASTNASITE',              // 바스트네사이트 - 탄산염
  MONAZITE = 'MONAZITE',                  // 모나자이트 - 인산염
  XENOTIME = 'XENOTIME',                  // 제노타임 - 중희토류
  ION_ADSORPTION_CLAY = 'ION_ADSORPTION_CLAY',  // 이온흡착형 점토
  LOPARITE = 'LOPARITE',                  // 로파라이트
  APATITE = 'APATITE',                    // 인회석
}

/**
 * Ore deposit information
 */
export interface OreDeposit {
  oreType: OreType;
  mineralFormula?: string;  // e.g., "(Ce,La)CO₃F"
  treoGrade_percent: number;  // Total REO grade
  dominantREE: RareEarthElement[];  // Main REE in deposit
  thoriumContent_ppm?: number;  // Radioactive thorium content
  uraniumContent_ppm?: number;  // Radioactive uranium content
}

// ============================================================================
// Mining Methods
// ============================================================================

/**
 * Mining method types
 */
export enum MiningMethod {
  OPEN_PIT = 'OPEN_PIT',                  // 노천 채굴
  UNDERGROUND = 'UNDERGROUND',            // 지하 채굴
  IN_SITU_LEACHING = 'IN_SITU_LEACHING', // 원위치 침출 (ISL)
  PLACER = 'PLACER',                      // 사광 채굴
  HEAP_LEACHING = 'HEAP_LEACHING',        // 힙 침출
}

/**
 * Processing method types
 */
export enum ProcessingMethod {
  // Extraction
  FLOTATION = 'FLOTATION',                // 부유 선광
  MAGNETIC_SEPARATION = 'MAGNETIC_SEPARATION',  // 자력 선별
  GRAVITY_SEPARATION = 'GRAVITY_SEPARATION',    // 비중 선별

  // Leaching
  ACID_LEACHING = 'ACID_LEACHING',        // 산 침출
  ALKALI_LEACHING = 'ALKALI_LEACHING',    // 알칼리 침출
  ROASTING = 'ROASTING',                  // 배소

  // Separation
  SOLVENT_EXTRACTION = 'SOLVENT_EXTRACTION',  // 용매 추출 (SX)
  ION_EXCHANGE = 'ION_EXCHANGE',          // 이온 교환
  PRECIPITATION = 'PRECIPITATION',         // 침전

  // Reduction
  ELECTROLYSIS = 'ELECTROLYSIS',          // 전기분해
  METALLOTHERMIC = 'METALLOTHERMIC',      // 금속 열환원
}

// ============================================================================
// Mine Information
// ============================================================================

/**
 * Mine type
 */
export enum MineType {
  OPEN_PIT = 'OPEN_PIT',
  UNDERGROUND = 'UNDERGROUND',
  IN_SITU = 'IN_SITU',
  PLACER = 'PLACER',
}

/**
 * Mine operational status
 */
export enum MineStatus {
  EXPLORATION = 'EXPLORATION',      // 탐사 중
  DEVELOPMENT = 'DEVELOPMENT',      // 개발 중
  OPERATIONAL = 'OPERATIONAL',      // 가동 중
  MAINTENANCE = 'MAINTENANCE',      // 정비 중
  SUSPENDED = 'SUSPENDED',          // 일시 중단
  CLOSED = 'CLOSED',                // 폐광
  RESTORATION = 'RESTORATION',      // 복원 중
}

/**
 * Mine registration information
 */
export interface MineInfo {
  mineId: string;
  name: string;
  operator: string;
  location: Location;
  mineType: MineType;
  miningMethod: MiningMethod;
  oreType: OreType;
  status: MineStatus;
  capacity_tons_per_year?: number;
  treoGrade_percent: number;
  operationStartDate?: Timestamp;
  expectedLifespan_years?: number;
  license?: string;
  certifications?: Certification[];
}

/**
 * REE composition in ore/product
 */
export interface REEComposition {
  // Light REE (as oxides, %)
  La2O3_percent?: number;
  CeO2_percent?: number;
  Pr6O11_percent?: number;
  Nd2O3_percent?: number;
  Sm2O3_percent?: number;
  Eu2O3_percent?: number;

  // Heavy REE (as oxides, %)
  Gd2O3_percent?: number;
  Tb4O7_percent?: number;
  Dy2O3_percent?: number;
  Ho2O3_percent?: number;
  Er2O3_percent?: number;
  Tm2O3_percent?: number;
  Yb2O3_percent?: number;
  Lu2O3_percent?: number;

  // Associated
  Sc2O3_percent?: number;
  Y2O3_percent?: number;

  // Total
  TREO_percent: number;  // Total Rare Earth Oxide
}

// ============================================================================
// Radioactive Materials
// ============================================================================

/**
 * Radioactive material types
 */
export enum RadioactiveMaterial {
  THORIUM_232 = 'Th-232',
  URANIUM_238 = 'U-238',
  URANIUM_235 = 'U-235',
  RADIUM_226 = 'Ra-226',
  RADON_222 = 'Rn-222',
}

/**
 * Radioactive waste classification
 */
export enum RadioactiveWasteLevel {
  LOW_LEVEL = 'LOW_LEVEL',              // <100 Bq/g
  INTERMEDIATE_LEVEL = 'INTERMEDIATE_LEVEL',  // 100-10,000 Bq/g
  HIGH_LEVEL = 'HIGH_LEVEL',            // >10,000 Bq/g
}

/**
 * Radioactive material information
 */
export interface RadioactiveInfo {
  thorium_ppm: number;
  uranium_ppm: number;
  totalRadioactivity_Bq_g: number;
  wasteLevel: RadioactiveWasteLevel;
  halfLife_years?: number;
  radiationType?: string;  // e.g., "alpha", "beta", "gamma"
  shielding?: string;      // Required shielding material
  storageRequirements?: string;
}

// ============================================================================
// Environmental Impact
// ============================================================================

/**
 * Environmental impact metrics
 */
export interface EnvironmentalImpact {
  // Land
  landDisturbance_hectares?: number;
  topsoilRemoved_tons?: number;
  restorationRequired: boolean;
  restorationPlan?: string;
  restorationFund_USD?: number;

  // Water
  waterUsage_m3_per_ton: number;
  waterRecyclingRate_percent?: number;
  wastewaterGeneration_m3?: number;
  wastewaterTreatment?: string;

  // Air
  co2Emissions_tons?: number;
  dustGeneration_kg?: number;
  so2Emissions_kg?: number;
  noxEmissions_kg?: number;

  // Waste
  tailings_tons?: number;
  radioactiveWaste_kg?: number;
  hazardousWaste_kg?: number;

  // Energy
  energyConsumption_MWh?: number;
  energySource?: string;  // e.g., "grid", "diesel", "renewable"
}

// ============================================================================
// Production Batch
// ============================================================================

/**
 * Processing stage types
 */
export enum ProcessingStage {
  EXTRACTION = 'EXTRACTION',
  BENEFICIATION = 'BENEFICIATION',
  ROASTING = 'ROASTING',
  LEACHING = 'LEACHING',
  SOLVENT_EXTRACTION = 'SOLVENT_EXTRACTION',
  PRECIPITATION = 'PRECIPITATION',
  CALCINATION = 'CALCINATION',
  REDUCTION = 'REDUCTION',
  REFINING = 'REFINING',
}

/**
 * Processing stage record
 */
export interface ProcessingStageRecord {
  stage: ProcessingStage;
  method: ProcessingMethod;
  date: Timestamp;
  facilityId?: string;

  // Input
  input_tons?: number;
  inputGrade_percent?: number;

  // Output
  output_tons?: number;
  outputGrade_percent?: number;

  // Efficiency
  recovery_percent?: number;

  // Process parameters
  temperature_C?: number;
  pressure_bar?: number;
  duration_hours?: number;
  reagents?: Array<{
    name: string;
    quantity: number;
    unit: string;
  }>;

  // Quality
  purity_percent?: number;
  impurities?: Record<string, number>;
}

/**
 * REE product information
 */
export interface REEProduct {
  element: RareEarthElement;
  compound: string;  // e.g., "Nd2O3", "Nd metal"
  form: 'OXIDE' | 'METAL' | 'ALLOY' | 'COMPOUND';
  quantity_kg: number;
  purity_percent: number;
  grade?: string;  // e.g., "99.5%", "99.99%", "4N", "5N"
  pricePerKg_USD?: number;
  totalValue_USD?: number;
  certificationNumber?: string;
}

/**
 * Production batch record
 */
export interface ProductionBatch {
  batchId: string;
  productionDate: Timestamp;
  mineId: string;
  facilityId: string;
  stages: ProcessingStageRecord[];
  composition: REEComposition;
  products: REEProduct[];
  waste: {
    tailings_tons?: number;
    radioactiveWaste_kg?: number;
    radioactivity_Bq?: number;
    wastewater_m3?: number;
    treatment?: string;
  };
  environmentalImpact: EnvironmentalImpact;
  qualityControl?: {
    inspector: string;
    inspectionDate: Timestamp;
    passed: boolean;
    testResults?: Record<string, any>;
  };
}

// ============================================================================
// Supply Chain
// ============================================================================

/**
 * Supply chain stage
 */
export enum SupplyChainStage {
  MINING = 'MINING',
  BENEFICIATION = 'BENEFICIATION',
  PROCESSING = 'PROCESSING',
  SEPARATION = 'SEPARATION',
  REFINING = 'REFINING',
  MANUFACTURING = 'MANUFACTURING',
  DISTRIBUTION = 'DISTRIBUTION',
  END_USE = 'END_USE',
}

/**
 * Supply chain event
 */
export interface SupplyChainEvent {
  stage: SupplyChainStage;
  timestamp: Timestamp;
  facility: string;
  location: Location;
  operator?: string;
  certifications?: Certification[];
  notes?: string;
}

/**
 * Supply chain tracking
 */
export interface SupplyChainTracking {
  batchId: string;
  origin: {
    mine: string;
    country: string;
    extractionDate: Timestamp;
    gps: Coordinates;
  };
  currentLocation: {
    facility: string;
    address?: string;
    timestamp: Timestamp;
  };
  supplyChain: SupplyChainEvent[];
  products: REEProduct[];
  radioactiveSafety: {
    level: RadioactiveWasteLevel;
    radiation_Bq_g: number;
    certified: boolean;
    certificationDate?: Timestamp;
  };
  verified: boolean;
  verificationDate?: Timestamp;
}

// ============================================================================
// Pricing
// ============================================================================

/**
 * REE price information
 */
export interface REEPrice {
  element: RareEarthElement;
  compound: string;  // e.g., "Nd2O3", "Nd metal"
  price_USD_kg: number;
  currency?: string;
  exchange?: string;  // e.g., "Shanghai REE Exchange"
  timestamp: Timestamp;
  change_1d_percent?: number;
  change_7d_percent?: number;
  change_30d_percent?: number;
  volume_kg?: number;
  high_52week?: number;
  low_52week?: number;
}

/**
 * REE market index
 */
export interface REEMarketIndex {
  indexName: string;  // e.g., "WIA_REE_Index"
  value: number;
  timestamp: Timestamp;
  change_1d_percent: number;
  change_7d_percent: number;
  change_30d_percent: number;
  components?: Array<{
    element: RareEarthElement;
    weight_percent: number;
  }>;
}

// ============================================================================
// Recycling (Urban Mining)
// ============================================================================

/**
 * Source of recycled REE
 */
export enum RecyclingSource {
  HARD_DISK_DRIVES = 'HARD_DISK_DRIVES',      // HDD magnets (Nd, Dy)
  FLUORESCENT_LAMPS = 'FLUORESCENT_LAMPS',    // Phosphors (Y, Eu, Tb)
  BATTERIES = 'BATTERIES',                     // NiMH batteries (La)
  CATALYSTS = 'CATALYSTS',                     // Auto catalysts (La, Ce)
  MAGNETS = 'MAGNETS',                         // NdFeB magnets
  POLISHING_POWDER = 'POLISHING_POWDER',       // CeO2
  PHOSPHORS = 'PHOSPHORS',                     // LED, display phosphors
}

/**
 * Recycling batch information
 */
export interface RecyclingBatch {
  batchId: string;
  source: RecyclingSource;
  sourceDescription?: string;
  collectionDate: Timestamp;
  quantity_kg: number;
  estimatedREEContent_kg: number;
  processingMethod: ProcessingMethod;
  recoveredREE: REEProduct[];
  recoveryRate_percent: number;
  environmentalBenefit: {
    co2Avoided_kg: number;
    energySaved_kWh: number;
    wasteAvoided_kg: number;
  };
}

// ============================================================================
// Certifications
// ============================================================================

/**
 * Industry certifications
 */
export enum Certification {
  ISO_9001 = 'ISO9001',           // Quality Management
  ISO_14001 = 'ISO14001',         // Environmental Management
  ISO_45001 = 'ISO45001',         // Occupational Health & Safety
  IATF_16949 = 'IATF16949',       // Automotive Quality
  OHSAS_18001 = 'OHSAS18001',     // Occupational Health & Safety (legacy)
  IAEA_BSS = 'IAEA_BSS',          // Radiation Safety
  RMI = 'RMI',                    // Responsible Minerals Initiative
  EMAS = 'EMAS',                  // EU Eco-Management
}

// ============================================================================
// Blockchain Tracking
// ============================================================================

/**
 * Blockchain record type
 */
export enum BlockchainRecordType {
  MINE_REGISTRATION = 'MINE_REGISTRATION',
  BATCH_PRODUCTION = 'BATCH_PRODUCTION',
  PROCESSING = 'PROCESSING',
  SHIPMENT = 'SHIPMENT',
  CERTIFICATION = 'CERTIFICATION',
  RECYCLING = 'RECYCLING',
}

/**
 * Blockchain tracking record
 */
export interface BlockchainRecord {
  txHash: string;
  network: string;  // e.g., "WIA-BLOCKCHAIN"
  timestamp: Timestamp;
  recordType: BlockchainRecordType;
  data: {
    batchId?: string;
    mineId?: string;
    status?: string;
    [key: string]: any;
  };
  immutable: boolean;
  verified: boolean;
  verificationAuthority?: string;
}

// ============================================================================
// API Types
// ============================================================================

/**
 * Mine registration request
 */
export interface MineRegistrationRequest {
  mine: MineInfo;
  composition: REEComposition;
  radioactive: RadioactiveInfo;
  environmental: EnvironmentalImpact;
}

/**
 * Mine registration response
 */
export interface MineRegistrationResponse {
  mineId: string;
  registrationNumber: string;
  certificationUrl: string;
  trackingUrl: string;
  qrCode?: string;
}

/**
 * Batch submission request
 */
export interface BatchSubmissionRequest {
  batchId: string;
  mineId: string;
  facilityId: string;
  stages: ProcessingStageRecord[];
  composition: REEComposition;
  products: REEProduct[];
  waste: ProductionBatch['waste'];
  environmentalImpact: EnvironmentalImpact;
}

/**
 * Batch submission response
 */
export interface BatchSubmissionResponse {
  batchId: string;
  blockchainTx: string;
  certificateUrl: string;
  qrCode: string;
  estimatedValue_USD: number;
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
    requestId?: string;
  };
}

// ============================================================================
// Statistics & Reporting
// ============================================================================

/**
 * Production statistics
 */
export interface ProductionStatistics {
  period: {
    start: Timestamp;
    end: Timestamp;
  };
  mineId?: string;
  region?: string;
  country?: string;

  // Production volumes
  totalProduction_tons: number;
  byElement: Record<RareEarthElement, number>;  // kg per element

  // Economic
  totalValue_USD: number;
  averagePrice_USD_kg: number;

  // Environmental
  environmentalImpact: EnvironmentalImpact;

  // Safety
  radioactiveWasteGenerated_kg: number;
  radiationExposure_mSv?: number;  // Average worker exposure
  safetyIncidents?: number;
}

/**
 * Market report
 */
export interface MarketReport {
  reportDate: Timestamp;
  prices: REEPrice[];
  indices: REEMarketIndex[];
  supplyDemand: {
    element: RareEarthElement;
    supply_tons: number;
    demand_tons: number;
    balance_tons: number;  // positive = surplus, negative = deficit
    priceForcast_USD_kg?: number;
  }[];
  marketTrends?: string[];
}

// ============================================================================
// Export all types
// ============================================================================

export type {
  Timestamp,
  Coordinates,
  Location,
};
