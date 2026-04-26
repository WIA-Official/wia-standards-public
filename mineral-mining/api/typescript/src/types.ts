/**
 * WIA-ENE-038: Sustainable Mineral Mining Standard - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license CC BY 4.0
 * @description Type definitions for sustainable mineral mining operations, production monitoring,
 *              environmental management, community relations, and supply chain traceability
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 */

// ==================== Mine Type Classification ====================

/**
 * Mine type classification
 */
export type MineType =
  | 'OPEN_PIT' // Surface mining - open pit
  | 'UNDERGROUND' // Underground mining
  | 'SOLUTION' // Solution mining (ISL, heap leach)
  | 'MOUNTAINTOP_REMOVAL' // Mountaintop removal
  | 'PLACER' // Placer mining (alluvial)
  | 'DREDGING'; // Marine/river dredging

/**
 * Mine status
 */
export type MineStatus =
  | 'exploration' // Exploration phase
  | 'development' // Development/construction
  | 'operating' // Active operations
  | 'care_maintenance' // Care and maintenance
  | 'closure' // Closure in progress
  | 'post_closure' // Post-closure monitoring
  | 'abandoned'; // Abandoned (legacy)

// ==================== Mineral Classification ====================

/**
 * Mineral type classification
 */
export type MineralType =
  // Base metals
  | 'COPPER'
  | 'IRON'
  | 'ALUMINUM'
  | 'ZINC'
  | 'LEAD'
  | 'NICKEL'
  | 'TIN'
  // Precious metals
  | 'GOLD'
  | 'SILVER'
  | 'PLATINUM'
  | 'PALLADIUM'
  // Battery metals
  | 'LITHIUM'
  | 'COBALT'
  | 'GRAPHITE'
  | 'MANGANESE'
  // Rare earth elements
  | 'NEODYMIUM'
  | 'DYSPROSIUM'
  | 'PRASEODYMIUM'
  | 'RARE_EARTH_MIXED'
  // Industrial minerals
  | 'PHOSPHATE'
  | 'POTASH'
  | 'LIMESTONE'
  | 'GYPSUM'
  | 'SALT'
  // Other
  | 'URANIUM'
  | 'COAL'
  | 'DIAMOND'
  | 'OTHER';

/**
 * Mineral category
 */
export type MineralCategory =
  | 'base_metals'
  | 'precious_metals'
  | 'battery_metals'
  | 'rare_earth_elements'
  | 'industrial_minerals'
  | 'energy_minerals'
  | 'gemstones';

// ==================== Core Data Structures ====================

/**
 * Geographic coordinates
 */
export interface Coordinates {
  latitude: number; // Decimal degrees
  longitude: number; // Decimal degrees
  datum?: 'WGS84' | 'NAD83' | 'NAD27';
}

/**
 * Address information
 */
export interface Address {
  region?: string; // State/Province/Region
  country: string; // ISO 3166-1 alpha-2 code
  postalCode?: string;
  district?: string;
}

/**
 * Measurement with unit
 */
export interface Measurement {
  value: number;
  unit: string;
}

/**
 * Volume measurement
 */
export interface Volume {
  value: number;
  unit: 'tonnes' | 'kilotonnes' | 'megatonnes' | 'm3' | 'bbl';
}

/**
 * Length/depth measurement
 */
export interface Length {
  value: number;
  unit: 'meters' | 'feet' | 'kilometers';
}

/**
 * Percentage
 */
export interface Percentage {
  value: number;
  unit: 'percent' | 'fraction';
}

/**
 * Concentration
 */
export interface Concentration {
  value: number;
  unit: 'ppm' | 'ppb' | 'mg/L' | 'ug/L' | 'g/t' | 'oz/ton';
}

// ==================== Operator Information ====================

/**
 * Contact information
 */
export interface ContactInfo {
  name: string;
  role?: string;
  phone: string;
  email: string;
  emergencyContact?: string;
}

/**
 * Operator information
 */
export interface Operator {
  operatorId: string;
  operatorName: string;
  license: string;
  contact: ContactInfo;
  operatorType?: 'major' | 'mid_tier' | 'junior' | 'artisanal' | 'state_owned';
  certifications?: string[]; // e.g., IRMA, RMI, Fairtrade
}

// ==================== Mine Location ====================

/**
 * Mine location details
 */
export interface MineLocation {
  address: Address;
  coordinates: Coordinates;
  elevation?: Length;
  landOwnership?: 'state' | 'private' | 'communal' | 'indigenous' | 'mixed';
  indigenousLand?: boolean;
  protectedArea?: boolean; // Near protected areas
}

// ==================== Mineral Resources ====================

/**
 * Resource/Reserve category
 */
export type ResourceCategory =
  | 'inferred'
  | 'indicated'
  | 'measured'
  | 'probable_reserves'
  | 'proven_reserves';

/**
 * Mineral resource estimate
 */
export interface MineralResources {
  primaryMineral: MineralType;
  secondaryMinerals?: MineralType[];
  estimatedReserves?: {
    value: number;
    unit: 'tonnes' | 'kilotonnes' | 'megatonnes';
    category?: ResourceCategory;
  };
  averageGrade?: {
    [mineral: string]: Percentage;
  };
  resourceStatement?: {
    date: string; // ISO 8601 date
    competentPerson: string;
    standard: 'JORC' | 'NI43-101' | 'SAMREC' | 'PERC';
  };
}

// ==================== Mining Operations ====================

/**
 * Mining method
 */
export type MiningMethod =
  // Open pit methods
  | 'OPEN_PIT_TRUCK_SHOVEL'
  | 'OPEN_PIT_DRAGLINE'
  | 'OPEN_PIT_BUCKET_WHEEL'
  // Underground methods
  | 'UNDERGROUND_ROOM_PILLAR'
  | 'UNDERGROUND_LONGWALL'
  | 'UNDERGROUND_BLOCK_CAVING'
  | 'UNDERGROUND_SUBLEVEL_STOPING'
  | 'UNDERGROUND_CUT_AND_FILL'
  // Solution methods
  | 'IN_SITU_LEACHING'
  | 'HEAP_LEACHING'
  // Other
  | 'PLACER_MINING'
  | 'DREDGING';

/**
 * Processing method
 */
export type ProcessingMethod =
  | 'CRUSHING_GRINDING'
  | 'FLOTATION'
  | 'GRAVITY_SEPARATION'
  | 'MAGNETIC_SEPARATION'
  | 'LEACHING'
  | 'SOLVENT_EXTRACTION'
  | 'ELECTROWINNING'
  | 'SMELTING'
  | 'ROASTING';

/**
 * Mining operations data
 */
export interface MiningOperations {
  miningMethod: MiningMethod;
  processingMethods: ProcessingMethod[];
  operatingSchedule: {
    hoursPerDay: number;
    daysPerWeek: number;
    weeksPerYear: number;
  };
  equipment?: {
    haul_trucks?: number;
    excavators?: number;
    drills?: number;
    loaders?: number;
  };
  workforce?: {
    total: number;
    direct: number;
    contractors: number;
    localEmployment?: Percentage;
  };
  mineLife?: {
    startDate: string; // ISO 8601 date
    plannedClosureDate?: string; // ISO 8601 date
    estimatedYearsRemaining?: number;
  };
}

// ==================== Production Data ====================

/**
 * Ore production
 */
export interface OreProduction {
  extracted: Volume; // Total ore mined
  processed: Volume; // Ore processed through mill
  wasteRock?: Volume; // Waste rock removed
  stripRatio?: number; // Waste:Ore ratio
}

/**
 * Mineral production
 */
export interface MineralProduction {
  mineralType: MineralType;
  grade: Percentage; // Average grade
  produced: Volume; // Actual mineral produced
  quality?: {
    purity?: Percentage;
    contaminants?: Record<string, Concentration>;
  };
}

/**
 * Daily production data
 */
export interface DailyProduction {
  ore: OreProduction;
  minerals: MineralProduction[];
  recovery?: Percentage; // Overall recovery rate
  uptime?: {
    value: number;
    unit: 'hours' | 'percent';
  };
}

/**
 * Production record
 */
export interface ProductionRecord {
  recordId: string;
  mineId: string;
  timestamp: string; // ISO 8601 datetime
  dailyProduction: DailyProduction;
  comments?: string;
}

/**
 * Cumulative production
 */
export interface CumulativeProduction {
  minerals: {
    [mineralType: string]: Volume;
  };
  ore: {
    total: Volume;
    processed: Volume;
  };
  asOfDate: string; // ISO 8601 date
}

// ==================== Environmental Management ====================

/**
 * Water management
 */
export interface WaterManagement {
  freshwater: {
    value: number;
    unit: 'm3/day' | 'm3/year';
  };
  recycled: {
    value: number;
    unit: 'm3/day' | 'm3/year';
  };
  recyclingRate: Percentage;
  discharged?: {
    value: number;
    unit: 'm3/day' | 'm3/year';
  };
  waterQuality?: {
    pH?: number;
    tds?: Concentration; // Total dissolved solids
    heavyMetals?: Record<string, Concentration>;
  };
  waterSource?: 'surface_water' | 'groundwater' | 'municipal' | 'desalination' | 'mixed';
}

/**
 * Tailings facility type
 */
export type TailingsFacilityType =
  | 'TSF' // Tailings Storage Facility (wet)
  | 'DRY_STACK' // Dry stacking
  | 'PASTE' // Paste tailings
  | 'FILTERED' // Filtered tailings
  | 'UNDERWATER' // Submarine tailings disposal
  | 'BACKFILL'; // Underground backfill

/**
 * Tailings management
 */
export interface TailingsManagement {
  facilityType: TailingsFacilityType;
  volume: Volume; // Current volume stored
  capacity?: Volume; // Maximum capacity
  dryStacking: boolean;
  stabilityMonitoring: boolean;
  seismicRating?: number; // Earthquake magnitude design
  damHeight?: Length;
  damType?: 'upstream' | 'downstream' | 'centerline';
  monitoring?: {
    piezometers?: number;
    seismicSensors?: number;
    inclinometers?: number;
    inspectionFrequency: 'daily' | 'weekly' | 'monthly';
  };
  emergencyResponse?: {
    planInPlace: boolean;
    lastDrill?: string; // ISO 8601 date
  };
}

/**
 * Air quality data
 */
export interface AirQualityMonitoring {
  particulateMatter: {
    pm10?: Concentration;
    pm25?: Concentration;
  };
  sulfurDioxide?: Concentration;
  nitrogenOxides?: Concentration;
  monitoringStations?: number;
  exceedances?: number; // Number of times limits exceeded
}

/**
 * Greenhouse gas emissions
 */
export interface GreenhouseGasEmissions {
  scope1?: {
    value: number;
    unit: 'tonnes_CO2e/year';
  };
  scope2?: {
    value: number;
    unit: 'tonnes_CO2e/year';
  };
  scope3?: {
    value: number;
    unit: 'tonnes_CO2e/year';
  };
  total?: {
    value: number;
    unit: 'tonnes_CO2e/year';
  };
  intensity?: {
    value: number;
    unit: 'tonnes_CO2e/tonne_product';
  };
}

/**
 * Emissions data
 */
export interface EmissionsData {
  greenhouseGas: GreenhouseGasEmissions;
  particulateMatter?: AirQualityMonitoring['particulateMatter'];
  reportingYear?: number;
  verificationStatus?: 'unverified' | 'third_party_verified';
}

/**
 * Biodiversity protection
 */
export interface BiodiversityProtection {
  baselineAssessment?: {
    date: string; // ISO 8601 date
    endangered_species?: number;
    habitat_types?: string[];
  };
  protectionMeasures?: string[];
  offsetPrograms?: {
    area: {
      value: number;
      unit: 'hectares' | 'acres';
    };
    location?: string;
  };
  monitoringProgram?: boolean;
}

/**
 * Land reclamation
 */
export interface LandReclamation {
  totalDisturbedArea: {
    value: number;
    unit: 'hectares' | 'acres';
  };
  reclaimedArea?: {
    value: number;
    unit: 'hectares' | 'acres';
  };
  reclamationPlan: {
    approved: boolean;
    approvalDate?: string; // ISO 8601 date
    financialAssurance?: {
      value: number;
      currency: 'USD' | 'EUR' | 'CAD' | 'AUD' | 'ZAR';
    };
  };
  revegetation?: {
    nativeSpecies: boolean;
    survivalRate?: Percentage;
    monitoringYears: number;
  };
  soilReplacement?: {
    topsoilStockpiled: Volume;
    replacementDepth?: Length;
  };
}

/**
 * Environmental management
 */
export interface EnvironmentalManagement {
  waterUsage: WaterManagement;
  emissionsData: EmissionsData;
  tailingsManagement: TailingsManagement;
  landReclamation?: LandReclamation;
  biodiversity?: BiodiversityProtection;
  airQuality?: AirQualityMonitoring;
  wasteManagement?: {
    hazardous?: Volume;
    nonHazardous?: Volume;
    recyclingRate?: Percentage;
  };
}

// ==================== Safety Management ====================

/**
 * Safety incident
 */
export interface SafetyIncident {
  incidentId: string;
  date: string; // ISO 8601 datetime
  incidentType:
    | 'fatality'
    | 'lost_time_injury'
    | 'restricted_work_injury'
    | 'medical_treatment'
    | 'first_aid'
    | 'near_miss'
    | 'environmental_incident';
  severity: 'minor' | 'moderate' | 'serious' | 'catastrophic';
  description: string;
  injuries?: number;
  fatalities?: number;
  rootCause?: string;
  correctiveActions?: string[];
}

/**
 * Safety metrics
 */
export interface SafetyMetrics {
  ltifr?: number; // Lost Time Injury Frequency Rate (per million hours)
  trifr?: number; // Total Recordable Injury Frequency Rate
  fatalities?: number;
  incidentHistory: SafetyIncident[];
  safetyTraining?: {
    hoursPerEmployee: number;
    certifications?: string[];
  };
}

// ==================== Community Relations ====================

/**
 * Community engagement
 */
export interface CommunityEngagement {
  consultations: {
    date: string; // ISO 8601 date
    participants: number;
    topics: string[];
    outcomes?: string;
  }[];
  grievanceMechanism?: {
    inPlace: boolean;
    grievancesReceived?: number;
    grievancesResolved?: number;
    averageResolutionDays?: number;
  };
  culturalHeritage?: {
    sitesIdentified?: number;
    protectionMeasures?: string[];
  };
}

/**
 * Social impact
 */
export interface SocialImpact {
  localEmployment?: {
    total: number;
    percentage: Percentage;
  };
  localProcurement?: {
    value: number;
    currency: string;
    percentage: Percentage;
  };
  communityInvestment?: {
    annual: number;
    currency: string;
    programs?: string[];
  };
  resettlement?: {
    households: number;
    compensationProvided: boolean;
    livelihoodRestoration: boolean;
  };
  humanRights?: {
    dueDiligenceCompleted: boolean;
    lastAssessment?: string; // ISO 8601 date
    issues?: string[];
  };
}

/**
 * Community relations
 */
export interface CommunityRelations {
  engagement: CommunityEngagement;
  socialImpact: SocialImpact;
  indigenousRelations?: {
    fpicObtained?: boolean; // Free, Prior and Informed Consent
    benefitSharingAgreement?: boolean;
    traditionalLandRights?: boolean;
  };
}

// ==================== Supply Chain Traceability ====================

/**
 * Conflict-free certification
 */
export interface ConflictFreeCertification {
  certified: boolean;
  standard: 'OECD' | 'RMI' | 'LBMA' | 'Fairtrade' | 'other';
  auditDate?: string; // ISO 8601 date
  auditor?: string;
  certificateNumber?: string;
  validUntil?: string; // ISO 8601 date
}

/**
 * Supply chain tracking
 */
export interface SupplyChainTracking {
  shipmentId: string;
  mineralType: MineralType;
  quantity: Volume;
  origin: {
    mineId: string;
    mineName: string;
    country: string;
  };
  destination: {
    country: string;
    facility: string;
    company?: string;
  };
  transportMethod?: 'truck' | 'rail' | 'ship' | 'air' | 'pipeline';
  departureDate?: string; // ISO 8601 date
  arrivalDate?: string; // ISO 8601 date
  certifications: string[];
  conflictFree: boolean;
  blockchainHash?: string; // Blockchain transaction hash
  chainOfCustody?: {
    links: {
      timestamp: string;
      entity: string;
      location: string;
      verified: boolean;
    }[];
  };
}

/**
 * Supply chain data
 */
export interface SupplyChainData {
  traceabilitySystem: 'manual' | 'blockchain' | 'rfid' | 'gps' | 'integrated';
  conflictFreeCertification: ConflictFreeCertification;
  shipments: SupplyChainTracking[];
  downstreamCustomers?: string[];
}

// ==================== ESG Metrics ====================

/**
 * Environmental performance
 */
export interface EnvironmentalPerformance {
  carbonIntensity: {
    value: number;
    unit: 'tonnes_CO2e/tonne_product';
  };
  waterIntensity: {
    value: number;
    unit: 'm3/tonne_product';
  };
  energyIntensity?: {
    value: number;
    unit: 'GJ/tonne_product' | 'kWh/tonne_product';
  };
  wasteIntensity?: {
    value: number;
    unit: 'tonnes_waste/tonne_product';
  };
  landDisturbance: {
    value: number;
    unit: 'hectares';
  };
  landReclaimed: {
    value: number;
    unit: 'hectares';
  };
}

/**
 * Social performance
 */
export interface SocialPerformance {
  ltifr: number;
  fatalities: number;
  localEmploymentRate: Percentage;
  womenInWorkforce?: Percentage;
  communityInvestment: {
    value: number;
    currency: string;
  };
  grievancesResolved: Percentage;
  humanRightsCompliance: boolean;
}

/**
 * Governance performance
 */
export interface GovernancePerformance {
  boardIndependence?: Percentage;
  antiCorruptionPolicy: boolean;
  whistleblowerProtection: boolean;
  transparencyReporting: boolean;
  taxTransparency?: boolean;
  certifications: string[];
  regulatoryViolations?: number;
  finesAndPenalties?: {
    value: number;
    currency: string;
  };
}

/**
 * ESG metrics
 */
export interface ESGMetrics {
  reportingYear: number;
  environmental: EnvironmentalPerformance;
  social: SocialPerformance;
  governance: GovernancePerformance;
  overallESGScore?: number; // 0-100
  ratingAgency?: string; // e.g., MSCI, Sustainalytics
}

// ==================== Regulatory Compliance ====================

/**
 * Permit type
 */
export type PermitType =
  | 'mining_license'
  | 'environmental_permit'
  | 'water_permit'
  | 'explosives_permit'
  | 'export_license'
  | 'land_use_permit';

/**
 * Regulatory permit
 */
export interface Permit {
  permitType: PermitType;
  permitNumber: string;
  authority: string;
  issueDate: string; // ISO 8601 date
  expiryDate: string; // ISO 8601 date
  conditions?: string[];
  status: 'active' | 'expired' | 'suspended' | 'revoked' | 'pending';
}

/**
 * Regulatory information
 */
export interface RegulatoryInfo {
  permits: Permit[];
  inspections?: {
    lastInspection: string; // ISO 8601 date
    nextInspection?: string; // ISO 8601 date
    violations?: number;
  };
  reportingRequirements?: {
    production: 'monthly' | 'quarterly' | 'annual';
    environmental: 'monthly' | 'quarterly' | 'annual';
    safety: 'immediate' | 'monthly' | 'annual';
  };
}

// ==================== Main Mine Structure ====================

/**
 * Complete mine data structure
 */
export interface Mine {
  mineId: string;
  timestamp: string; // ISO 8601 datetime
  mineType: MineType;
  mineName: string;
  status: MineStatus;
  operator: Operator;
  location: MineLocation;
  mineralResources: MineralResources;
  miningOperations?: MiningOperations;
  production?: {
    firstProduction?: string; // ISO 8601 date
    currentRate?: DailyProduction;
    cumulativeProduction?: CumulativeProduction;
  };
  environmental?: EnvironmentalManagement;
  safety?: SafetyMetrics;
  communityRelations?: CommunityRelations;
  supplyChain?: SupplyChainData;
  esgMetrics?: ESGMetrics;
  regulatory?: RegulatoryInfo;
  metadata?: {
    createdBy?: string;
    lastModified?: string;
    version?: string;
    dataSource?: string;
    [key: string]: any;
  };
}

// ==================== API Request/Response Types ====================

/**
 * Create mine request
 */
export interface CreateMineRequest {
  mineType: MineType;
  mineName: string;
  operatorId: string;
  location: MineLocation;
  mineralResources: MineralResources;
}

/**
 * Create mine response
 */
export interface CreateMineResponse {
  mineId: string;
  timestamp: string;
  status: 'success' | 'error';
  message?: string;
}

/**
 * Update mine status request
 */
export interface UpdateMineStatusRequest {
  mineId: string;
  status: MineStatus;
  effectiveDate: string; // ISO 8601 date
  reason?: string;
}

/**
 * Query parameters for mines
 */
export interface MineQueryParams {
  operatorId?: string;
  mineType?: MineType;
  status?: MineStatus;
  country?: string;
  mineralType?: MineralType;
  page?: number;
  limit?: number;
}

/**
 * Production data submission request
 */
export interface SubmitProductionRequest {
  mineId: string;
  date: string; // ISO 8601 date
  production: DailyProduction;
}

/**
 * Production query parameters
 */
export interface ProductionQueryParams {
  mineId?: string;
  startDate?: string; // ISO 8601 date
  endDate?: string; // ISO 8601 date
  aggregation?: 'daily' | 'monthly' | 'yearly';
  page?: number;
  limit?: number;
}

/**
 * Environmental data submission
 */
export interface SubmitEnvironmentalDataRequest {
  mineId: string;
  reportingPeriod: {
    startDate: string; // ISO 8601 date
    endDate: string; // ISO 8601 date
  };
  waterUsage?: WaterManagement;
  emissionsData?: EmissionsData;
  tailingsManagement?: TailingsManagement;
  airQuality?: AirQualityMonitoring;
}

/**
 * Community engagement submission
 */
export interface SubmitCommunityEngagementRequest {
  mineId: string;
  engagement: CommunityEngagement;
  socialImpact?: Partial<SocialImpact>;
}

/**
 * Supply chain registration
 */
export interface RegisterSupplyChainRequest {
  mineId: string;
  shipmentId: string;
  mineralType: MineralType;
  quantity: Volume;
  destination: {
    country: string;
    facility: string;
    company?: string;
  };
  certifications: string[];
  conflictFree: boolean;
  blockchainHash?: string;
}

/**
 * ESG report submission
 */
export interface SubmitESGReportRequest {
  mineId: string;
  reportingYear: number;
  esgMetrics: ESGMetrics;
}

/**
 * Compliance status
 */
export interface ComplianceStatus {
  mineId: string;
  asOfDate: string; // ISO 8601 date
  permits: {
    total: number;
    active: number;
    expired: number;
    expiringSoon: number; // Within 90 days
  };
  environmentalCompliance: boolean;
  safetyCompliance: boolean;
  overallCompliance: 'compliant' | 'non_compliant' | 'conditional';
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  data: T[];
  pagination: {
    page: number;
    limit: number;
    total: number;
    totalPages: number;
  };
}

/**
 * Operator dashboard summary
 */
export interface OperatorDashboard {
  operatorId: string;
  operatorName: string;
  timestamp: string; // ISO 8601 datetime
  summary: {
    totalMines: number;
    operatingMines: number;
    closureMines: number;
  };
  production: {
    monthly: {
      [mineralType: string]: Volume;
    };
    yearly: {
      [mineralType: string]: Volume;
    };
  };
  environmental: {
    totalEmissions: {
      value: number;
      unit: 'tonnes_CO2e/year';
    };
    waterRecyclingRate: Percentage;
    landReclaimed: {
      value: number;
      unit: 'hectares';
    };
  };
  social: {
    ltifr: number;
    localEmploymentRate: Percentage;
    communityInvestment: {
      value: number;
      currency: string;
    };
  };
  esg: {
    overallScore?: number;
    environmental?: number;
    social?: number;
    governance?: number;
  };
}

// ==================== Webhook Types ====================

/**
 * Webhook event types
 */
export type WebhookEventType =
  | 'mine.created'
  | 'mine.status_changed'
  | 'production.threshold'
  | 'tailings.alert'
  | 'safety.incident'
  | 'environmental.violation'
  | 'permit.expiring'
  | 'supply_chain.verified';

/**
 * Webhook payload
 */
export interface WebhookPayload<T = any> {
  event: WebhookEventType;
  timestamp: string; // ISO 8601 datetime
  data: T;
  signature?: string; // HMAC signature for verification
}

// ==================== Client Configuration ====================

/**
 * API client configuration
 */
export interface ClientConfig {
  apiKey: string;
  endpoint?: string; // Default: https://api.wia.org/ene-038/v1
  timeout?: number; // milliseconds
  retries?: number;
  operatorId?: string;
}

/**
 * API error response
 */
export interface APIError {
  code: string;
  message: string;
  details?: any;
  timestamp: string; // ISO 8601 datetime
  requestId?: string;
}

// ==================== Export All ====================

export default {
  // Types are exported individually above
};
