/**
 * WIA-ENE-030: Biodiversity Index Standard - TypeScript Type Definitions
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
  siteName: string;
  coordinates: Coordinates;
  habitat: HabitatType;
  ecosystem: EcosystemType;
  protectionStatus?: string;
}

// ============================================================================
// Enumerations
// ============================================================================

/**
 * Habitat types
 */
export enum HabitatType {
  // Terrestrial
  MONTANE_FOREST = 'MONTANE_FOREST',           // 산지 산림
  LOWLAND_FOREST = 'LOWLAND_FOREST',           // 저지대 산림
  GRASSLAND = 'GRASSLAND',                     // 초원
  DESERT = 'DESERT',                           // 사막
  TUNDRA = 'TUNDRA',                           // 툰드라
  URBAN = 'URBAN',                             // 도시
  AGRICULTURAL = 'AGRICULTURAL',               // 농업지

  // Freshwater
  RIVER = 'RIVER',                             // 하천
  LAKE = 'LAKE',                               // 호수
  WETLAND = 'WETLAND',                         // 습지
  STREAM = 'STREAM',                           // 개울

  // Marine
  CORAL_REEF = 'CORAL_REEF',                   // 산호초
  COASTAL = 'COASTAL',                         // 연안
  PELAGIC = 'PELAGIC',                         // 대양
  DEEP_SEA = 'DEEP_SEA',                       // 심해
  MANGROVE = 'MANGROVE',                       // 맹그로브
}

/**
 * Ecosystem types
 */
export enum EcosystemType {
  TERRESTRIAL = 'TERRESTRIAL',     // 육상
  FRESHWATER = 'FRESHWATER',       // 담수
  MARINE = 'MARINE',               // 해양
  COASTAL = 'COASTAL',             // 연안
}

/**
 * Taxonomic ranks
 */
export enum TaxonomicRank {
  KINGDOM = 'KINGDOM',       // 계
  PHYLUM = 'PHYLUM',         // 문
  CLASS = 'CLASS',           // 강
  ORDER = 'ORDER',           // 목
  FAMILY = 'FAMILY',         // 과
  GENUS = 'GENUS',           // 속
  SPECIES = 'SPECIES',       // 종
  SUBSPECIES = 'SUBSPECIES', // 아종
}

/**
 * IUCN Red List categories
 */
export enum IUCNCategory {
  EX = 'EX',   // Extinct (절멸)
  EW = 'EW',   // Extinct in the Wild (야생절멸)
  CR = 'CR',   // Critically Endangered (위급)
  EN = 'EN',   // Endangered (위기)
  VU = 'VU',   // Vulnerable (취약)
  NT = 'NT',   // Near Threatened (준위협)
  LC = 'LC',   // Least Concern (관심대상)
  DD = 'DD',   // Data Deficient (정보부족)
  NE = 'NE',   // Not Evaluated (평가되지 않음)
}

/**
 * Survey types
 */
export enum SurveyType {
  LINE_TRANSECT = 'LINE_TRANSECT',               // 선조사법
  QUADRAT = 'QUADRAT',                           // 방형구법
  POINT_COUNT = 'POINT_COUNT',                   // 정점 조사
  CAMERA_TRAP = 'CAMERA_TRAP',                   // 무인카메라
  ACOUSTIC_MONITORING = 'ACOUSTIC_MONITORING',   // 음향 모니터링
  DNA_BARCODING = 'DNA_BARCODING',               // DNA 바코딩
  EDNA = 'EDNA',                                 // 환경 DNA
  VISUAL_ENCOUNTER = 'VISUAL_ENCOUNTER',         // 시각 조사
  MIST_NETTING = 'MIST_NETTING',                 // 그물 포획
  PITFALL_TRAP = 'PITFALL_TRAP',                 // 함정 트랩
}

// ============================================================================
// Species & Taxonomy
// ============================================================================

/**
 * Taxonomic classification
 */
export interface Taxonomy {
  kingdom: string;
  phylum: string;
  class: string;
  order: string;
  family: string;
  genus: string;
  species: string;
  subspecies?: string;
}

/**
 * Species information
 */
export interface SpeciesInfo {
  scientificName: string;                // 학명
  commonName: string;                    // 일반명 (한글)
  taxonomicRank: TaxonomicRank;
  taxonomy: Taxonomy;
  iucnStatus?: IUCNCategory;
  isEndemic: boolean;                    // 고유종 여부
  nativeRange?: string;                  // 자생 범위
}

// ============================================================================
// Observations
// ============================================================================

/**
 * Individual organism information
 */
export interface IndividualInfo {
  count: number;
  lifeStage?: 'egg' | 'larva' | 'juvenile' | 'adult' | 'unknown';
  sex?: 'male' | 'female' | 'hermaphrodite' | 'unknown';
  behavior?: string;
  reproductiveStatus?: 'breeding' | 'non-breeding' | 'unknown';
  health?: 'healthy' | 'injured' | 'sick' | 'dead';
}

/**
 * Survey method information
 */
export interface SurveyMethod {
  surveyType: SurveyType;
  samplingEffort: number;                // hours or person-hours
  detectionMethod: string;
  identificationMethod: string;
  distance?: number;                     // meters from observer
}

/**
 * Evidence for observation
 */
export interface ObservationEvidence {
  photos?: string[];                     // URLs
  audio?: string[];                      // URLs
  video?: string[];                      // URLs
  dnaSequence?: string;                  // DNA barcode sequence
  voucherSpecimen?: string;              // Specimen ID
}

/**
 * Data quality metadata
 */
export interface DataMetadata {
  dataQuality: number;                   // 0-100
  verified: boolean;
  verifiedBy?: string;
  verificationDate?: Timestamp;
  identificationConfidence?: number;     // 0-100
  coordinateUncertainty?: number;        // meters
}

/**
 * Species observation record
 */
export interface SpeciesObservation {
  observationId: string;
  timestamp: Timestamp;
  observerId: string;
  location: Location;
  species: SpeciesInfo;
  individual: IndividualInfo;
  method: SurveyMethod;
  evidence: ObservationEvidence;
  metadata: DataMetadata;
  notes?: string;
}

// ============================================================================
// Biodiversity Surveys
// ============================================================================

/**
 * Survey period
 */
export interface SurveyPeriod {
  startDate: Timestamp;
  endDate: Timestamp;
  season?: 'spring' | 'summer' | 'fall' | 'winter';
}

/**
 * Survey area with geographic boundary
 */
export interface SurveyArea {
  siteName: string;
  boundary: {
    type: 'Polygon';
    coordinates: number[][][];           // GeoJSON format
  };
  areaSize: number;                      // hectares
  protectionStatus?: string;
  managingOrganization?: string;
}

/**
 * Survey team
 */
export interface SurveyTeam {
  leader: string;
  members: string[];
  volunteers?: string[];
  organization: string;
}

/**
 * Survey targets
 */
export interface SurveyTargets {
  taxonomicGroups: string[];             // e.g., ['Aves', 'Mammalia']
  focusSpecies?: string[];               // Scientific names
  habitatTypes: HabitatType[];
}

/**
 * Survey results summary
 */
export interface SurveyResults {
  totalSpecies: number;
  totalIndividuals: number;
  endemicSpecies: number;
  endangeredSpecies: number;
  observations: SpeciesObservation[];
}

/**
 * Biodiversity survey
 */
export interface BiodiversitySurvey {
  surveyId: string;
  projectName: string;
  period: SurveyPeriod;
  area: SurveyArea;
  team: SurveyTeam;
  targets: SurveyTargets;
  results: SurveyResults;
  status: 'planned' | 'ongoing' | 'completed' | 'cancelled';
}

// ============================================================================
// Diversity Indices
// ============================================================================

/**
 * Species composition
 */
export interface SpeciesComposition {
  scientificName: string;
  count: number;
  frequency: number;                     // occurrence frequency
  relativeAbundance: number;             // percentage
  dominance: number;
}

/**
 * Rarefaction curve data
 */
export interface RarefactionCurve {
  sampleSize: number[];
  expectedSpecies: number[];
  standardDeviation?: number[];
}

/**
 * Diversity statistics
 */
export interface DiversityStatistics {
  rareSpecies: number;                   // species with count = 1
  commonSpecies: number;
  dominantSpecies: string[];             // top 3 species
  rarefactionCurve?: RarefactionCurve;
}

/**
 * Diversity indices
 */
export interface DiversityIndices {
  indexId: string;
  surveyId: string;
  calculationDate: Timestamp;

  // Basic metrics
  speciesRichness: number;               // S
  totalIndividuals: number;              // N

  // Diversity indices
  shannonIndex: number;                  // H' = -Σ(pi × ln(pi))
  shannonEquitability: number;           // J' = H' / ln(S)
  simpsonIndex: number;                  // D = Σ(pi²)
  simpsonDiversity: number;              // 1-D
  bergerParkerIndex: number;             // max(pi)

  // Additional indices
  margalefRichness?: number;             // (S-1) / ln(N)
  pielouEquitability?: number;           // H' / H'max
  fisherAlpha?: number;                  // Fisher's alpha

  // Species composition
  speciesComposition: SpeciesComposition[];

  // Statistics
  statistics: DiversityStatistics;
}

// ============================================================================
// Habitat Assessment
// ============================================================================

/**
 * Physical characteristics
 */
export interface PhysicalCharacteristics {
  habitatType: HabitatType;
  areaSize: number;                      // hectares
  elevation: number;                     // meters
  slope: number;                         // degrees
  aspect: 'N' | 'NE' | 'E' | 'SE' | 'S' | 'SW' | 'W' | 'NW';
  soilType?: string;
}

/**
 * Environmental conditions
 */
export interface EnvironmentalConditions {
  temperature: {
    mean: number;                        // °C
    min: number;
    max: number;
  };
  precipitation: number;                 // mm/year
  humidity: number;                      // percentage
  lightIntensity?: number;               // lux
  waterQuality?: {
    pH: number;
    dissolvedOxygen: number;             // mg/L
    turbidity: number;                   // NTU
    conductivity?: number;               // μS/cm
  };
}

/**
 * Vegetation structure
 */
export interface VegetationStructure {
  canopyCover: number;                   // percentage
  treeHeight: number;                    // meters
  understory: string;
  invasiveSpecies: string[];
  plantDiversity?: number;
}

/**
 * Disturbance event
 */
export interface Disturbance {
  type: string;                          // e.g., 'fire', 'logging', 'flood'
  severity: 'low' | 'medium' | 'high';
  frequency: string;
  lastOccurrence?: Timestamp;
  extent?: number;                       // percentage of area
}

/**
 * Threat to habitat
 */
export interface Threat {
  threat: string;
  impact: 'low' | 'medium' | 'high' | 'critical';
  scope: number;                         // percentage of area affected
  timeframe: 'past' | 'ongoing' | 'future';
  mitigation?: string;
}

/**
 * Habitat quality assessment
 */
export interface HabitatQuality {
  overallGrade: 'A' | 'B' | 'C' | 'D' | 'E';
  integrityScore: number;                // 0-100
  conditionScore: number;                // 0-100
  habitatSuitabilityIndex?: number;      // 0-1 (HSI)
  managementNeeds: string[];
}

/**
 * Habitat assessment
 */
export interface HabitatAssessment {
  assessmentId: string;
  siteName: string;
  assessmentDate: Timestamp;
  physical: PhysicalCharacteristics;
  environmental: EnvironmentalConditions;
  vegetation: VegetationStructure;
  disturbances: Disturbance[];
  threats: Threat[];
  quality: HabitatQuality;
}

// ============================================================================
// DNA Barcoding & eDNA
// ============================================================================

/**
 * Gene regions for DNA barcoding
 */
export enum GeneRegion {
  COI = 'COI',       // Cytochrome c oxidase I (animals)
  RBCL = 'rbcL',     // Ribulose-1,5-bisphosphate carboxylase (plants)
  MATK = 'matK',     // Maturase K (plants)
  ITS = 'ITS',       // Internal transcribed spacer (fungi)
  ITS2 = 'ITS2',     // ITS2 region
  SIXTEEN_S = '16S', // 16S rRNA (bacteria)
  EIGHTEEN_S = '18S',// 18S rRNA (eukaryotes)
}

/**
 * DNA sequence match
 */
export interface DNAMatch {
  scientificName: string;
  similarity: number;                    // percentage
  database: 'BOLD' | 'GenBank' | 'NCBI' | 'EMBL';
  accessionNumber?: string;
}

/**
 * DNA barcoding result
 */
export interface DNABarcoding {
  sampleId: string;
  geneRegion: GeneRegion;
  sequence: string;                      // nucleotide sequence
  sequenceLength: number;                // base pairs
  quality: number;                       // Phred score
  matchedSpecies: DNAMatch[];
  topMatch?: DNAMatch;
  barcodeGap?: number;                   // genetic distance
}

/**
 * eDNA sample information
 */
export interface EDNASample {
  sampleId: string;
  collectionDate: Timestamp;
  location: Location;
  sampleType: 'water' | 'soil' | 'air' | 'sediment';
  volume: number;                        // liters or grams
  filterSize?: number;                   // micrometers
  preservationMethod: string;
  storageConditions: string;
}

/**
 * eDNA metabarcoding result
 */
export interface EDNAMetabarcoding {
  analysisId: string;
  sampleId: string;
  geneRegion: GeneRegion;
  totalReads: number;
  speciesDetected: {
    scientificName: string;
    readCount: number;
    relativeAbundance: number;           // percentage
    confidence: number;                  // 0-100
  }[];
  diversityIndices?: Partial<DiversityIndices>;
}

// ============================================================================
// Monitoring & Reporting
// ============================================================================

/**
 * Monitoring schedule
 */
export interface MonitoringSchedule {
  scheduleId: string;
  siteName: string;
  frequency: 'weekly' | 'monthly' | 'quarterly' | 'seasonal' | 'annual';
  taxonomicGroups: string[];
  startDate: Timestamp;
  endDate?: Timestamp;
  responsibleParty: string;
}

/**
 * Trend analysis
 */
export interface TrendAnalysis {
  indicator: string;
  timeSeriesData: {
    date: Timestamp;
    value: number;
  }[];
  trend: 'increasing' | 'decreasing' | 'stable' | 'fluctuating';
  changeRate?: number;                   // percentage per year
  significance?: number;                 // p-value
}

/**
 * Biodiversity report
 */
export interface BiodiversityReport {
  reportId: string;
  reportType: 'quarterly' | 'annual' | 'comprehensive';
  period: SurveyPeriod;
  region: string;
  summary: {
    totalSurveys: number;
    totalObservations: number;
    speciesRecorded: number;
    newSpecies: number;
    endangeredSpecies: number;
  };
  diversityTrends: TrendAnalysis[];
  keyFindings: string[];
  recommendations: string[];
  generatedBy: string;
  generatedAt: Timestamp;
}

// ============================================================================
// KPIs (Key Performance Indicators)
// ============================================================================

/**
 * Biodiversity KPIs
 */
export interface BiodiversityKPIs {
  speciesRichness: number;
  shannonIndex: number;
  endangeredSpeciesCount: number;
  endemicSpeciesRatio: number;           // percentage
  habitatQualityScore: number;           // 0-100
  invasiveSpeciesRatio: number;          // percentage
}

/**
 * Survey performance KPIs
 */
export interface SurveyPerformanceKPIs {
  surveyCompletionRate: number;          // percentage
  dataQualityScore: number;              // 0-100
  expertVerificationRate: number;        // percentage
  citizenScienceParticipation: number;   // number of contributors
}

/**
 * Conservation performance KPIs
 */
export interface ConservationKPIs {
  habitatRestorationArea: number;        // hectares
  protectedAreaIncrease: number;         // percentage
  speciesRecoveryCount: number;
  ecologicalCorridors: number;
}

/**
 * KPI dashboard
 */
export interface KPIDashboard {
  dashboardId: string;
  timestamp: Timestamp;
  region: string;
  biodiversity: BiodiversityKPIs;
  surveyPerformance: SurveyPerformanceKPIs;
  conservation: ConservationKPIs;
  overallScore?: number;                 // 0-100
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
 * Species filter
 */
export interface SpeciesFilter {
  scientificName?: string;
  commonName?: string;
  taxonomicGroup?: string;
  iucnCategory?: IUCNCategory;
  isEndemic?: boolean;
}

/**
 * Location filter
 */
export interface LocationFilter {
  siteName?: string;
  habitatType?: HabitatType;
  ecosystem?: EcosystemType;
  boundingBox?: {
    north: number;
    south: number;
    east: number;
    west: number;
  };
}

// ============================================================================
// Certification
// ============================================================================

/**
 * Observer certification levels
 */
export enum CertificationLevel {
  VOLUNTEER = 'VOLUNTEER',       // 자원봉사자
  SURVEYOR = 'SURVEYOR',         // 조사원
  EXPERT = 'EXPERT',             // 전문가
  TAXONOMIST = 'TAXONOMIST',     // 분류학자
}

/**
 * Observer certification
 */
export interface ObserverCertification {
  certificateId: string;
  observerId: string;
  level: CertificationLevel;
  specialization?: string[];             // e.g., ['Aves', 'Lepidoptera']
  issueDate: Timestamp;
  expiryDate: Timestamp;
  trainingHours: number;
  status: 'active' | 'expired' | 'suspended';
}

/**
 * Data quality tier
 */
export enum DataQualityTier {
  GOLD = 'GOLD',       // 전문가 검증 + 증거
  SILVER = 'SILVER',   // 전문가 검증 또는 증거
  BRONZE = 'BRONZE',   // 관찰 기록만
}

// ============================================================================
// Export all types
// ============================================================================

export type {
  Timestamp,
  Coordinates,
  Location,
};
