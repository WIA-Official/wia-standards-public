/**
 * WIA-ENE-022: Waste Management Standard - TypeScript Type Definitions
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
  facilityType?: string;
  region?: string;
  postalCode?: string;
}

// ============================================================================
// Waste Classification
// ============================================================================

/**
 * Waste category codes (WM-01 to WM-10)
 */
export enum WasteCategoryCode {
  PAPER = 'WM-01',           // 종이류
  PLASTIC = 'WM-02',         // 플라스틱
  GLASS = 'WM-03',           // 유리
  METAL = 'WM-04',           // 금속
  FOOD = 'WM-05',            // 음식물
  TEXTILE = 'WM-06',         // 섬유
  WOOD = 'WM-07',            // 목재
  ELECTRONICS = 'WM-08',     // 전자제품
  HAZARDOUS = 'WM-09',       // 유해물질
  OTHER = 'WM-10',           // 기타
}

/**
 * Hazard classification
 */
export enum HazardClass {
  GENERAL = 1,      // 일반 - 무해
  CAUTION = 2,      // 주의 - 적절한 처리 필요
  DANGEROUS = 3,    // 위험 - 특수 처리 필요
  HIGH_RISK = 4,    // 고위험 - 엄격한 통제 필요
}

/**
 * Waste composition material
 */
export interface WasteComposition {
  material: string;
  percentage: number;  // 0-100
}

/**
 * Waste information
 */
export interface WasteInfo {
  categoryCode: WasteCategoryCode;
  name: string;
  quantity: number;        // kg
  volume: number;          // L
  hazardClass: HazardClass;
  composition: WasteComposition[];
}

// ============================================================================
// Waste Generation
// ============================================================================

/**
 * Treatment plan for waste
 */
export interface TreatmentPlan {
  plannedMethod: TreatmentMethod;
  destinationId: string;
  scheduledDate: Timestamp;
  estimatedCost?: number;
}

/**
 * Treatment methods
 */
export enum TreatmentMethod {
  RECYCLING = 'recycling',           // 재활용
  COMPOSTING = 'composting',         // 퇴비화
  INCINERATION = 'incineration',     // 소각
  LANDFILL = 'landfill',             // 매립
  ENERGY_RECOVERY = 'energy_recovery', // 에너지 회수
  SPECIAL_TREATMENT = 'special_treatment', // 특수 처리
}

/**
 * Data quality metadata
 */
export interface DataMetadata {
  source: string;
  quality: number;         // 0-100
  verified: boolean;
  verifiedBy?: string;
  verificationDate?: Timestamp;
}

/**
 * Waste generation event
 */
export interface WasteGenerationEvent {
  eventId: string;
  timestamp: Timestamp;
  generatorId: string;
  location: Location;
  waste: WasteInfo;
  treatment: TreatmentPlan;
  metadata: DataMetadata;
}

// ============================================================================
// Collection & Transportation
// ============================================================================

/**
 * Vehicle types for waste collection
 */
export enum VehicleType {
  COMPACTOR = 'compactor',           // 압축차
  FLATBED = 'flatbed',               // 평판차
  CONTAINER = 'container',           // 컨테이너차
  REFRIGERATED = 'refrigerated',     // 냉동차
  SPECIALIZED = 'specialized',       // 특수차량
}

/**
 * Fuel types
 */
export enum FuelType {
  DIESEL = 'diesel',
  GASOLINE = 'gasoline',
  CNG = 'cng',                       // 압축천연가스
  ELECTRIC = 'electric',
  HYBRID = 'hybrid',
  HYDROGEN = 'hydrogen',
}

/**
 * Vehicle information
 */
export interface VehicleInfo {
  type: VehicleType;
  capacity: number;        // kg
  fuelType: FuelType;
  emissions: number;       // CO2 kg
  plateNumber?: string;
  model?: string;
}

/**
 * Collection route information
 */
export interface CollectionRoute {
  distance: number;        // km
  stops: number;
  efficiency: number;      // 0-100
  estimatedTime?: number;  // minutes
  actualTime?: number;     // minutes
}

/**
 * Collected waste item
 */
export interface CollectedWasteItem {
  eventId: string;
  collectedQuantity: number;  // kg
  binId?: string;
  collectionTime: Timestamp;
  condition?: string;
}

/**
 * Collection information
 */
export interface CollectionInfo {
  startTime: Timestamp;
  endTime: Timestamp;
  wasteItems: CollectedWasteItem[];
  totalWeight: number;     // kg
  totalVolume: number;     // L
  efficiency?: number;     // 0-100
}

/**
 * Waste collection event
 */
export interface WasteCollectionEvent {
  collectionId: string;
  routeId: string;
  vehicleId: string;
  collection: CollectionInfo;
  vehicle: VehicleInfo;
  route: CollectionRoute;
  crew?: string[];
  notes?: string;
}

// ============================================================================
// Smart Collection (IoT)
// ============================================================================

/**
 * Smart bin sensor data
 */
export interface SensorData {
  fillLevel: number;       // % (0-100)
  weight: number;          // kg
  temperature: number;     // °C
  humidity?: number;       // %
  lastUpdated: Timestamp;
}

/**
 * Collection priority
 */
export enum CollectionPriority {
  LOW = 'low',
  MEDIUM = 'medium',
  HIGH = 'high',
  URGENT = 'urgent',
}

/**
 * Smart waste bin with IoT sensors
 */
export interface SmartBin {
  binId: string;
  location: Location;
  categoryCode: WasteCategoryCode;
  sensorData: SensorData;
  needsCollection: boolean;
  priority: CollectionPriority;
  estimatedFullTime?: Timestamp;
  batteryLevel?: number;   // %
}

// ============================================================================
// Treatment Facilities
// ============================================================================

/**
 * Facility types
 */
export enum FacilityType {
  RECYCLING = 'recycling',           // 재활용
  INCINERATION = 'incineration',     // 소각
  LANDFILL = 'landfill',             // 매립
  COMPOSTING = 'composting',         // 퇴비화
  MRF = 'mrf',                       // 선별시설 (Material Recovery Facility)
  TRANSFER_STATION = 'transfer_station', // 중계시설
}

/**
 * Operational status
 */
export enum OperationalStatus {
  ACTIVE = 'active',
  MAINTENANCE = 'maintenance',
  CLOSED = 'closed',
  PLANNED = 'planned',
}

/**
 * Facility capacity information
 */
export interface FacilityCapacity {
  daily: number;           // 톤/일
  annual: number;          // 톤/년
  current: number;         // % utilization
  remaining: number;       // 톤 (for landfills)
}

/**
 * Permit information
 */
export interface PermitInfo {
  permitNumber: string;
  issueDate: Timestamp;
  expiryDate: Timestamp;
  certifications: string[];
  issuingAuthority?: string;
}

/**
 * Emissions data point
 */
export interface EmissionData {
  pollutant: string;       // e.g., 'PM', 'NOx', 'SOx', 'CO'
  value: number;
  unit: string;            // e.g., 'mg/Sm³', 'ppm'
  limit: number;
  timestamp?: Timestamp;
}

/**
 * Energy recovery information
 */
export interface EnergyRecovery {
  type: 'electricity' | 'heat' | 'both';
  electricityOutput?: number;  // kWh
  heatOutput?: number;         // MJ
  efficiency?: number;         // %
}

/**
 * Environmental performance data
 */
export interface EnvironmentalPerformance {
  emissionsData: EmissionData[];
  energyRecovery?: EnergyRecovery;
  waterUsage?: number;         // m³
  noiseLevel?: number;         // dB
}

/**
 * Waste treatment facility
 */
export interface WasteTreatmentFacility {
  facilityId: string;
  name: string;
  type: FacilityType;
  location: Location;
  operationalStatus: OperationalStatus;
  capacity: FacilityCapacity;
  permits: PermitInfo;
  environmental: EnvironmentalPerformance;
  contact?: {
    phone: string;
    email: string;
    manager: string;
  };
}

// ============================================================================
// Recycling Performance
// ============================================================================

/**
 * Category-specific statistics
 */
export interface CategoryStatistics {
  categoryCode: WasteCategoryCode;
  generated: number;       // 톤
  recycled: number;        // 톤
  rate: number;            // % (0-100)
  revenue?: number;        // 원
}

/**
 * Waste statistics
 */
export interface WasteStatistics {
  totalGenerated: number;  // 톤
  totalRecycled: number;   // 톤
  recyclingRate: number;   // %
  diversionRate: number;   // %
  byCategory: CategoryStatistics[];
}

/**
 * Environmental impact
 */
export interface EnvironmentalImpact {
  co2Reduced: number;      // 톤 CO2
  energySaved: number;     // MWh
  waterSaved: number;      // m³
  landfillAvoided: number; // 톤
  treesEquivalent?: number; // 나무 그루 수
}

/**
 * Economic effects
 */
export interface EconomicEffects {
  revenue: number;         // 원
  cost: number;            // 원
  netBenefit: number;      // 원
  jobsCreated: number;     // 명
  costPerTon?: number;     // 원/톤
}

/**
 * Recycling performance report
 */
export interface RecyclingPerformance {
  periodId: string;
  startDate: Timestamp;
  endDate: Timestamp;
  region: string;
  statistics: WasteStatistics;
  impact: EnvironmentalImpact;
  economics: EconomicEffects;
  goals?: {
    targetRecyclingRate: number;
    achieved: boolean;
  };
}

// ============================================================================
// Monitoring & Reporting
// ============================================================================

/**
 * Monitoring parameter status
 */
export enum MonitoringStatus {
  NORMAL = 'normal',
  WARNING = 'warning',
  ALERT = 'alert',
  ERROR = 'error',
}

/**
 * Single measurement
 */
export interface Measurement {
  parameter: string;
  value: number;
  unit: string;
  timestamp: Timestamp;
  status: MonitoringStatus;
  threshold?: number;
}

/**
 * Monitoring data from IoT sensors
 */
export interface MonitoringData {
  deviceId: string;
  sensorType: string;
  measurements: Measurement[];
  location: Coordinates;
  deviceStatus?: string;
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
  summary: {
    totalWasteGenerated: number;
    totalWasteRecycled: number;
    recyclingRate: number;
    co2Reduction: number;
  };
  details: RecyclingPerformance;
  recommendations?: string[];
  generatedBy?: string;
  generatedAt?: Timestamp;
}

// ============================================================================
// Collection Schedule
// ============================================================================

/**
 * Collection frequency
 */
export enum CollectionFrequency {
  DAILY = 'daily',
  EVERY_OTHER_DAY = 'every_other_day',
  WEEKLY = 'weekly',
  BIWEEKLY = 'biweekly',
  MONTHLY = 'monthly',
  ON_DEMAND = 'on_demand',
}

/**
 * Time slot for collection
 */
export interface TimeSlot {
  startTime: string;       // HH:MM format
  endTime: string;         // HH:MM format
}

/**
 * Collection schedule
 */
export interface CollectionSchedule {
  scheduleId: string;
  region: string;
  wasteCategory: WasteCategoryCode;
  frequency: CollectionFrequency;
  timeSlot: TimeSlot;
  daysOfWeek?: number[];   // 0=Sunday, 1=Monday, ..., 6=Saturday
  startDate: Timestamp;
  endDate?: Timestamp;
  specialInstructions?: string;
}

// ============================================================================
// KPIs (Key Performance Indicators)
// ============================================================================

/**
 * Environmental KPIs
 */
export interface EnvironmentalKPIs {
  recyclingRate: number;           // %
  landfillReductionRate: number;   // %
  co2Reduction: number;            // 톤 CO2eq
  energyRecoveryRate: number;      // %
  collectionEfficiency: number;    // %
}

/**
 * Operational KPIs
 */
export interface OperationalKPIs {
  collectionCompletionRate: number;  // %
  facilityUptime: number;            // %
  safetyIncidents: number;           // 건수
  complaintResolutionRate: number;   // %
  responseTime: number;              // 시간
}

/**
 * Economic KPIs
 */
export interface EconomicKPIs {
  recyclingRevenue: number;          // 원
  costPerTon: number;                // 원/톤
  energySalesRevenue: number;        // 원
  roi: number;                       // %
  costReduction: number;             // %
}

/**
 * Comprehensive KPI dashboard
 */
export interface KPIDashboard {
  periodId: string;
  timestamp: Timestamp;
  environmental: EnvironmentalKPIs;
  operational: OperationalKPIs;
  economic: EconomicKPIs;
  overallScore?: number;             // 0-100
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
// Certification
// ============================================================================

/**
 * Certification levels
 */
export enum CertificationLevel {
  BRONZE = 'bronze',
  SILVER = 'silver',
  GOLD = 'gold',
  PLATINUM = 'platinum',
}

/**
 * Certification requirements
 */
export interface CertificationRequirements {
  level: CertificationLevel;
  minimumRecyclingRate: number;
  energyRecoveryRequired: boolean;
  carbonNeutralRequired: boolean;
  otherRequirements: string[];
}

/**
 * Certification record
 */
export interface Certification {
  certificateId: string;
  facilityId: string;
  level: CertificationLevel;
  issueDate: Timestamp;
  expiryDate: Timestamp;
  verifiedBy: string;
  performance: {
    recyclingRate: number;
    energyRecovery: boolean;
    carbonNeutral: boolean;
  };
  status: 'active' | 'expired' | 'suspended';
}

// ============================================================================
// Export all types
// ============================================================================

export type {
  Timestamp,
  Coordinates,
  Location,
};
