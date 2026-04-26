/**
 * WIA-ENE-025: E-Waste Management Standard - TypeScript Type Definitions
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
}

/**
 * Location information
 */
export interface Location {
  lat: number;
  lon: number;
  address?: string;
  city?: string;
  country?: string;
  postalCode?: string;
}

// ============================================================================
// Device Categories
// ============================================================================

/**
 * E-waste categories (WEEE Directive)
 */
export enum DeviceCategory {
  LARGE_HOUSEHOLD = 'LARGE_HOUSEHOLD',           // 대형 가전
  IT_EQUIPMENT = 'IT_EQUIPMENT',                 // IT 및 통신장비
  SMALL_HOUSEHOLD = 'SMALL_HOUSEHOLD',           // 소형 가전
  DISPLAY_EQUIPMENT = 'DISPLAY_EQUIPMENT',       // 디스플레이 장비
  LIGHTING = 'LIGHTING',                         // 조명
  TOYS_LEISURE = 'TOYS_LEISURE',                 // 장난감 및 레저
  MEDICAL = 'MEDICAL',                           // 의료기기
}

/**
 * Specific device types
 */
export enum DeviceType {
  // IT Equipment
  SMARTPHONE = 'SMARTPHONE',
  TABLET = 'TABLET',
  LAPTOP = 'LAPTOP',
  DESKTOP = 'DESKTOP',
  SERVER = 'SERVER',
  NETWORK_EQUIPMENT = 'NETWORK_EQUIPMENT',
  PRINTER = 'PRINTER',
  SCANNER = 'SCANNER',

  // Large Household
  REFRIGERATOR = 'REFRIGERATOR',
  WASHING_MACHINE = 'WASHING_MACHINE',
  AIR_CONDITIONER = 'AIR_CONDITIONER',
  DISHWASHER = 'DISHWASHER',
  MICROWAVE = 'MICROWAVE',

  // Display
  LCD_TV = 'LCD_TV',
  LED_MONITOR = 'LED_MONITOR',
  OLED_DISPLAY = 'OLED_DISPLAY',
  CRT_MONITOR = 'CRT_MONITOR',
  CRT_TV = 'CRT_TV',

  // Others
  OTHER = 'OTHER',
}

/**
 * Device condition
 */
export enum DeviceCondition {
  FUNCTIONAL = 'FUNCTIONAL',               // 작동 가능
  PARTIALLY_FUNCTIONAL = 'PARTIALLY_FUNCTIONAL',  // 부분 작동
  NON_FUNCTIONAL = 'NON_FUNCTIONAL',       // 작동 불가
  DAMAGED = 'DAMAGED',                     // 손상
  UNKNOWN = 'UNKNOWN',                     // 알 수 없음
}

// ============================================================================
// Hazard Classification
// ============================================================================

/**
 * Hazard level classification
 */
export enum HazardLevel {
  LEVEL_1 = 'LEVEL_1',  // 낮음: 일반 소비자 전자제품
  LEVEL_2 = 'LEVEL_2',  // 중간: 특정 유해물질 함유
  LEVEL_3 = 'LEVEL_3',  // 높음: 복합 유해물질
}

/**
 * Hazardous substances
 */
export enum HazardousSubstance {
  LEAD = 'LEAD',                    // 납 (Pb)
  MERCURY = 'MERCURY',              // 수은 (Hg)
  CADMIUM = 'CADMIUM',              // 카드뮴 (Cd)
  HEXAVALENT_CHROMIUM = 'HEXAVALENT_CHROMIUM',  // 육가크롬 (Cr6+)
  PBB = 'PBB',                      // 폴리브롬화 비페닐
  PBDE = 'PBDE',                    // 폴리브롬화 디페닐 에테르
  ARSENIC = 'ARSENIC',              // 비소 (As)
  LITHIUM_BATTERY = 'LITHIUM_BATTERY',  // 리튬 배터리
  NICKEL_CADMIUM_BATTERY = 'NICKEL_CADMIUM_BATTERY',  // 니켈-카드뮴 배터리
  REFRIGERANT = 'REFRIGERANT',      // 냉매
}

/**
 * Risk level for hazardous materials
 */
export enum RiskLevel {
  LOW = 'LOW',
  MEDIUM = 'MEDIUM',
  HIGH = 'HIGH',
  CRITICAL = 'CRITICAL',
}

/**
 * Hazardous material information
 */
export interface HazardousMaterial {
  substance: HazardousSubstance;
  quantity_g?: number;
  concentration_ppm?: number;
  riskLevel: RiskLevel;
  location?: string;  // Component location
  handlingInstructions?: string;
}

// ============================================================================
// Device Information
// ============================================================================

/**
 * Device information
 */
export interface DeviceInfo {
  category: DeviceCategory;
  type: DeviceType;
  brand: string;
  model: string;
  serialNumber?: string;
  manufactureYear?: number;
  weight_kg: number;
  condition: DeviceCondition;
}

/**
 * Owner information
 */
export interface OwnerInfo {
  type: 'INDIVIDUAL' | 'BUSINESS' | 'GOVERNMENT' | 'INSTITUTION';
  country: string;
  postalCode?: string;
  anonymousId?: string;  // Privacy-preserving identifier
  contactEmail?: string;
  contactPhone?: string;
}

/**
 * Estimated material value
 */
export interface EstimatedValue {
  gold_g?: number;
  silver_g?: number;
  copper_g?: number;
  aluminum_g?: number;
  palladium_g?: number;
  platinum_g?: number;
  rareEarthElements_g?: number;
  totalValue_USD?: number;
}

// ============================================================================
// Collection
// ============================================================================

/**
 * Collection point information
 */
export interface CollectionPoint {
  facilityId: string;
  name: string;
  location: Location;
  collectionDate: Timestamp;
  operatingHours?: string;
  contactPhone?: string;
}

/**
 * Device registration/entry
 */
export interface DeviceEntry {
  deviceId: string;
  registrationDate: Timestamp;
  device: DeviceInfo;
  owner: OwnerInfo;
  collectionPoint: CollectionPoint;
  hazardousMaterials?: HazardousMaterial[];
  estimatedValue?: EstimatedValue;
  photos?: string[];  // URLs to device photos
  notes?: string;
}

// ============================================================================
// Transportation
// ============================================================================

/**
 * Packaging type
 */
export enum PackagingType {
  PALLET = 'PALLET',
  BOX = 'BOX',
  CONTAINER = 'CONTAINER',
  BULK = 'BULK',
  SPECIALIZED = 'SPECIALIZED',
}

/**
 * Transport manifest
 */
export interface TransportManifest {
  manifestId: string;
  transportDate: Timestamp;
  origin: Location;
  destination: Location;
  carrier: string;
  vehicleId?: string;
  driverName?: string;
  packaging: PackagingType;
  deviceIds: string[];
  totalWeight_kg: number;
  hazardousMaterialsDeclaration?: boolean;
  unNumber?: string;  // UN number for hazardous materials
}

// ============================================================================
// Processing Facility
// ============================================================================

/**
 * Facility certifications
 */
export enum Certification {
  R2 = 'R2',                        // Responsible Recycling
  E_STEWARDS = 'E_STEWARDS',        // e-Stewards
  ISO14001 = 'ISO14001',            // Environmental Management
  ISO9001 = 'ISO9001',              // Quality Management
  OHSAS18001 = 'OHSAS18001',        // Occupational Health & Safety
  WEEELABEX = 'WEEELABEX',          // WEEE Lab Excellence
}

/**
 * Facility information
 */
export interface FacilityInfo {
  facilityId: string;
  name: string;
  license: string;
  location: Location;
  certifications: Certification[];
  capacity_tons_per_day?: number;
  contactEmail?: string;
  contactPhone?: string;
}

// ============================================================================
// Processing Steps
// ============================================================================

/**
 * Processing step types
 */
export enum ProcessingStep {
  DATA_DESTRUCTION = 'DATA_DESTRUCTION',
  BATTERY_REMOVAL = 'BATTERY_REMOVAL',
  HAZMAT_REMOVAL = 'HAZMAT_REMOVAL',
  MANUAL_DISMANTLING = 'MANUAL_DISMANTLING',
  AUTOMATED_DISMANTLING = 'AUTOMATED_DISMANTLING',
  PCB_EXTRACTION = 'PCB_EXTRACTION',
  CABLE_SEPARATION = 'CABLE_SEPARATION',
  PLASTIC_RECOVERY = 'PLASTIC_RECOVERY',
  METAL_SEPARATION = 'METAL_SEPARATION',
  SHREDDING = 'SHREDDING',
  MAGNETIC_SEPARATION = 'MAGNETIC_SEPARATION',
  EDDY_CURRENT_SEPARATION = 'EDDY_CURRENT_SEPARATION',
  OPTICAL_SORTING = 'OPTICAL_SORTING',
  PRECIOUS_METAL_RECOVERY = 'PRECIOUS_METAL_RECOVERY',
}

/**
 * Data destruction method
 */
export enum DataDestructionMethod {
  DOD_5220_22_M = 'DOD_5220.22-M',     // US DoD standard
  NIST_800_88 = 'NIST_800-88',         // NIST standard
  GUTMANN = 'GUTMANN',                 // Gutmann method
  PHYSICAL_DESTRUCTION = 'PHYSICAL_DESTRUCTION',
  DEGAUSSING = 'DEGAUSSING',
}

/**
 * Processing step record
 */
export interface ProcessingStepRecord {
  step: ProcessingStep;
  timestamp: Timestamp;
  method?: string;
  verified?: boolean;
  verifier?: string;
  destination?: string;
  weight_g?: number;
  notes?: string;

  // Specific to data destruction
  dataDestructionMethod?: DataDestructionMethod;
  certificateNumber?: string;

  // Specific to battery/hazmat
  battery_type?: string;
  capacity_mAh?: number;
  hazmat_type?: HazardousSubstance;
}

// ============================================================================
// Materials Recovery
// ============================================================================

/**
 * Recovered materials
 */
export interface MaterialsRecovered {
  // Precious metals (milligrams)
  gold_mg?: number;
  silver_mg?: number;
  platinum_mg?: number;
  palladium_mg?: number;

  // Base metals (grams)
  copper_g?: number;
  aluminum_g?: number;
  iron_g?: number;
  nickel_g?: number;

  // Rare earth elements (grams)
  neodymium_g?: number;
  dysprosium_g?: number;
  indium_g?: number;
  tantalum_g?: number;

  // Other materials (grams)
  plastics_g?: number;
  glass_g?: number;
  circuitBoards_g?: number;

  // Total recovery rate
  recoveryRate_percent?: number;
}

/**
 * Recovery efficiency metrics
 */
export interface RecoveryEfficiency {
  gold_percent?: number;      // Percentage of estimated gold recovered
  silver_percent?: number;
  copper_percent?: number;
  overall_percent?: number;
}

// ============================================================================
// Environmental Impact
// ============================================================================

/**
 * Environmental impact metrics
 */
export interface EnvironmentalImpact {
  co2Avoided_kg: number;
  energySaved_kWh: number;
  waterSaved_L?: number;
  landfillAvoided_kg?: number;
  treesEquivalent?: number;
}

// ============================================================================
// Processing Record
// ============================================================================

/**
 * Complete processing record
 */
export interface ProcessingRecord {
  processingId: string;
  deviceId: string;
  facility: FacilityInfo;
  processDate: Timestamp;
  steps: ProcessingStepRecord[];
  materialsRecovered: MaterialsRecovered;
  recoveryEfficiency?: RecoveryEfficiency;
  environmentalImpact: EnvironmentalImpact;
  qualityControl?: {
    inspector: string;
    inspectionDate: Timestamp;
    passed: boolean;
    notes?: string;
  };
}

// ============================================================================
// Blockchain Tracking
// ============================================================================

/**
 * Blockchain record type
 */
export enum BlockchainRecordType {
  DEVICE_REGISTRATION = 'DEVICE_REGISTRATION',
  COLLECTION = 'COLLECTION',
  TRANSPORT = 'TRANSPORT',
  PROCESSING = 'PROCESSING',
  CERTIFICATION = 'CERTIFICATION',
}

/**
 * Blockchain tracking record
 */
export interface BlockchainRecord {
  txHash: string;
  network: string;
  timestamp: Timestamp;
  recordType: BlockchainRecordType;
  data: {
    deviceId: string;
    status?: string;
    certification?: Certification;
    hashPrevious?: string;
    [key: string]: any;
  };
  immutable: boolean;
  verified: boolean;
}

// ============================================================================
// Tracking & Status
// ============================================================================

/**
 * Device status
 */
export enum DeviceStatus {
  REGISTERED = 'REGISTERED',
  COLLECTED = 'COLLECTED',
  IN_TRANSIT = 'IN_TRANSIT',
  RECEIVED = 'RECEIVED',
  PROCESSING = 'PROCESSING',
  PROCESSED = 'PROCESSED',
  CERTIFIED = 'CERTIFIED',
}

/**
 * Timeline event
 */
export interface TimelineEvent {
  stage: DeviceStatus;
  timestamp: Timestamp;
  location?: string;
  facility?: string;
  carrier?: string;
  notes?: string;
}

/**
 * Device tracking information
 */
export interface DeviceTracking {
  deviceId: string;
  currentStatus: DeviceStatus;
  timeline: TimelineEvent[];
  environmentalImpact?: EnvironmentalImpact;
  materialsRecovered?: MaterialsRecovered;
  certificate?: {
    certificateId: string;
    certificateUrl: string;
    issueDate: Timestamp;
  };
}

// ============================================================================
// Recycling Grade
// ============================================================================

/**
 * Recycling value grade
 */
export enum RecyclingGrade {
  GRADE_A = 'GRADE_A',  // 고가치: 귀금속 고함량
  GRADE_B = 'GRADE_B',  // 중간가치: 일반 금속 주도
  GRADE_C = 'GRADE_C',  // 저가치: 플라스틱 주도
}

// ============================================================================
// API Types
// ============================================================================

/**
 * Device registration request
 */
export interface DeviceRegistrationRequest {
  device: DeviceInfo;
  owner: OwnerInfo;
  collectionPoint: Omit<CollectionPoint, 'collectionDate'>;
  hazardousMaterials?: HazardousMaterial[];
  photos?: string[];
}

/**
 * Device registration response
 */
export interface DeviceRegistrationResponse {
  deviceId: string;
  qrCode: string;
  trackingUrl: string;
  estimatedValue?: EstimatedValue;
  recyclingGrade?: RecyclingGrade;
}

/**
 * Processing submission request
 */
export interface ProcessingSubmissionRequest {
  deviceId: string;
  facility: FacilityInfo;
  steps: ProcessingStepRecord[];
  materialsRecovered: MaterialsRecovered;
  environmentalImpact: EnvironmentalImpact;
}

/**
 * Processing submission response
 */
export interface ProcessingSubmissionResponse {
  processingId: string;
  blockchainTx: string;
  certificateUrl: string;
  recoveryEfficiency: RecoveryEfficiency;
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
  };
}

// ============================================================================
// Statistics & Reporting
// ============================================================================

/**
 * Facility metrics
 */
export interface FacilityMetrics {
  facilityId: string;
  timestamp: Timestamp;
  metrics: {
    devicesProcessedToday: number;
    currentThroughput: number;  // units/hour
    goldRecoveredToday_g: number;
    silverRecoveredToday_g: number;
    copperRecoveredToday_kg: number;
    co2AvoidedToday_kg: number;
    energySavedToday_kWh: number;
  };
  operationalStatus: 'NORMAL' | 'MAINTENANCE' | 'ALERT' | 'OFFLINE';
  alerts?: string[];
}

/**
 * Regional statistics
 */
export interface RegionalStatistics {
  region: string;
  period: {
    start: Timestamp;
    end: Timestamp;
  };
  totalDevicesProcessed: number;
  totalWeight_tons: number;
  materialsRecovered: MaterialsRecovered;
  environmentalImpact: EnvironmentalImpact;
  recyclingRate_percent: number;
  topDeviceCategories: Array<{
    category: DeviceCategory;
    count: number;
    percentage: number;
  }>;
}

// ============================================================================
// MRV (Monitoring, Reporting, Verification)
// ============================================================================

/**
 * MRV report type
 */
export enum MRVReportType {
  DAILY = 'DAILY',
  WEEKLY = 'WEEKLY',
  MONTHLY = 'MONTHLY',
  QUARTERLY = 'QUARTERLY',
  ANNUAL = 'ANNUAL',
}

/**
 * MRV report
 */
export interface MRVReport {
  reportId: string;
  reportType: MRVReportType;
  period: {
    start: Timestamp;
    end: Timestamp;
  };
  facility?: FacilityInfo;
  region?: string;
  summary: {
    devicesProcessed: number;
    totalWeight_tons: number;
    recyclingRate_percent: number;
    co2Avoided_tons: number;
  };
  materialsRecovered: MaterialsRecovered;
  compliance: {
    r2Compliant: boolean;
    eStewardsCompliant: boolean;
    roHSCompliant: boolean;
    weeeCompliant: boolean;
  };
  verified: boolean;
  verifier?: string;
  verificationDate?: Timestamp;
}

// ============================================================================
// Export all types
// ============================================================================

export type {
  Timestamp,
  Coordinates,
  Location,
};
