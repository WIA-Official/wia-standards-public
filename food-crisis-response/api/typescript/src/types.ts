/**
 * WIA-AGRI-031 Food Crisis Response Standard - TypeScript Type Definitions
 * @module @wia/food-crisis-response/types
 */

// ============================================================================
// Crisis Types
// ============================================================================

export interface FoodCrisis {
  crisisId: string;
  name: string;
  type: CrisisType;
  severity: CrisisSeverity;
  region: Region;
  startDate: string;
  endDate?: string;
  affectedPopulation: number;
  status: CrisisStatus;
  causes: string[];
  impactAssessment: ImpactAssessment;
}

export type CrisisType = 'drought' | 'flood' | 'conflict' | 'economic' | 'pandemic' | 'pest_outbreak' | 'earthquake' | 'cyclone' | 'multiple';
export type CrisisSeverity = 'phase_1' | 'phase_2' | 'phase_3' | 'phase_4' | 'phase_5'; // IPC phases
export type CrisisStatus = 'emerging' | 'active' | 'escalating' | 'stabilizing' | 'resolved';

export interface Region {
  regionId: string;
  name: string;
  country: string;
  coordinates: { latitude: number; longitude: number };
  population: number;
}

export interface ImpactAssessment {
  timestamp: number;
  livelihoodsLost: number;
  displacedPersons: number;
  foodInsecurePopulation: number;
  acuteMalnutrition: number;
  mortalityRate: number; // per 10,000 per day
  infrastructureDamage: InfrastructureDamage;
  economicLoss: number; // USD
}

export interface InfrastructureDamage {
  marketsDestroyed: number;
  roadsBlocked: number;
  storageDestroyed: number;
  waterSystemsDamaged: number;
  farmlandAffected: number; // hectares
}

// ============================================================================
// Response Operations
// ============================================================================

export interface EmergencyResponse {
  responseId: string;
  crisisId: string;
  name: string;
  type: ResponseType;
  leadAgency: string;
  partners: string[];
  targetPopulation: number;
  startDate: string;
  plannedDuration: number; // days
  budget: number; // USD
  status: 'planned' | 'active' | 'suspended' | 'completed';
  operations: Operation[];
  logistics: LogisticsInfo;
}

export type ResponseType = 'emergency_food' | 'cash_transfer' | 'vouchers' | 'nutrition' | 'livelihood' | 'water_sanitation' | 'multi_sector';

export interface Operation {
  operationId: string;
  name: string;
  type: string;
  location: Region;
  beneficiaries: number;
  startDate: string;
  endDate?: string;
  status: 'planned' | 'ongoing' | 'completed' | 'cancelled';
  resources: ResourceRequirement[];
  outputs: OperationOutput[];
}

export interface ResourceRequirement {
  resourceType: string;
  quantity: number;
  unit: string;
  allocated: number;
  gap: number;
  priority: 'critical' | 'high' | 'medium' | 'low';
}

export interface OperationOutput {
  outputId: string;
  description: string;
  target: number;
  achieved: number;
  unit: string;
  verificationDate?: string;
}

// ============================================================================
// Logistics & Distribution
// ============================================================================

export interface LogisticsInfo {
  hubs: LogisticsHub[];
  routes: DistributionRoute[];
  fleet: Vehicle[];
  warehouses: Warehouse[];
  lastMileCapacity: number; // households per day
}

export interface LogisticsHub {
  hubId: string;
  name: string;
  location: Region;
  type: 'main' | 'forward' | 'mobile';
  capacity: number; // tons
  currentStock: number; // tons
  throughput: number; // tons per day
}

export interface DistributionRoute {
  routeId: string;
  from: string;
  to: string;
  distance: number; // km
  travelTime: number; // hours
  condition: 'excellent' | 'good' | 'poor' | 'impassable';
  capacity: number; // tons per day
  securityRisk: 'low' | 'medium' | 'high';
}

export interface Vehicle {
  vehicleId: string;
  type: 'truck' | 'boat' | 'aircraft' | 'helicopter' | 'motorcycle';
  capacity: number; // tons or cubic meters
  status: 'available' | 'in_transit' | 'maintenance' | 'damaged';
  currentLocation?: string;
  fuelLevel?: number; // percentage
}

export interface Warehouse {
  warehouseId: string;
  name: string;
  location: Region;
  capacity: number; // cubic meters
  utilization: number; // percentage
  commodities: WarehouseStock[];
  securityLevel: 'secure' | 'at_risk' | 'compromised';
}

export interface WarehouseStock {
  commodity: string;
  quantity: number;
  unit: string;
  expiryDate?: string;
  condition: 'good' | 'fair' | 'damaged';
}

// ============================================================================
// Distribution & Beneficiaries
// ============================================================================

export interface Distribution {
  distributionId: string;
  operationId: string;
  date: string;
  location: DistributionPoint;
  commodities: CommodityDistribution[];
  beneficiaries: number;
  rationType: 'full' | 'half' | 'emergency';
  verificationMethod: string;
  issues: DistributionIssue[];
  status: 'planned' | 'ongoing' | 'completed' | 'cancelled';
}

export interface DistributionPoint {
  pointId: string;
  name: string;
  coordinates: { latitude: number; longitude: number };
  capacity: number; // beneficiaries per day
  facilities: string[];
  accessibility: 'easy' | 'moderate' | 'difficult';
}

export interface CommodityDistribution {
  commodity: string;
  quantityPlanned: number;
  quantityDistributed: number;
  unit: string;
  perBeneficiary: number;
}

export interface DistributionIssue {
  issueType: 'shortage' | 'quality' | 'access' | 'security' | 'verification' | 'other';
  description: string;
  severity: 'minor' | 'major' | 'critical';
  resolved: boolean;
}

export interface Beneficiary {
  beneficiaryId: string;
  householdId: string;
  name: string;
  age: number;
  gender: 'male' | 'female' | 'other';
  vulnerabilityCategory?: string[];
  registrationDate: string;
  distributionsReceived: number;
  lastDistribution?: string;
}

// ============================================================================
// Nutrition Programs
// ============================================================================

export interface NutritionProgram {
  programId: string;
  type: 'supplementary_feeding' | 'therapeutic_feeding' | 'blanket_feeding' | 'micronutrient';
  targetGroup: 'children_u5' | 'pregnant_women' | 'lactating_mothers' | 'elderly' | 'all';
  admissionCriteria: string;
  discharge Criteria: string;
  treatmentProtocol: TreatmentProtocol;
  facilities: NutritionFacility[];
  beneficiaries: number;
  cureRate?: number; // percentage
  defaultRate?: number; // percentage
}

export interface TreatmentProtocol {
  productType: string;
  dailyRation: number; // grams
  duration: number; // weeks
  accompaniments: string[];
  monitoringFrequency: string;
}

export interface NutritionFacility {
  facilityId: string;
  name: string;
  location: Region;
  type: 'outpatient' | 'inpatient' | 'mobile';
  capacity: number;
  currentCaseload: number;
  staff: number;
  supplies: NutritionSupply[];
}

export interface NutritionSupply {
  product: string;
  stock: number; // kg
  monthsOfStock: number;
  expiryDate: string;
}

// ============================================================================
// Assessment & Monitoring
// ============================================================================

export interface RapidAssessment {
  assessmentId: string;
  crisisId: string;
  date: string;
  assessors: string[];
  methodology: string;
  findings: AssessmentFindings;
  recommendations: string[];
  urgencyRating: 'immediate' | 'urgent' | 'planned';
}

export interface AssessmentFindings {
  affectedPopulation: number;
  foodSecurityPhase: CrisisSeverity;
  acuteMalnutritionPrevalence: number; // percentage
  mortalityRate: number; // per 10,000 per day
  crudeMortalityRate?: number;
  under5MortalityRate?: number;
  mainConcerns: string[];
  priorityNeeds: PriorityNeed[];
}

export interface PriorityNeed {
  category: 'food' | 'water' | 'shelter' | 'health' | 'protection' | 'livelihood';
  severity: 'critical' | 'high' | 'medium';
  affectedPopulation: number;
  timeframe: 'immediate' | 'short_term' | 'medium_term';
}

export interface MonitoringReport {
  reportId: string;
  responseId: string;
  period: { start: string; end: string };
  indicators: PerformanceIndicator[];
  challenges: string[];
  adaptations: string[];
  nextSteps: string[];
}

export interface PerformanceIndicator {
  indicator: string;
  target: number;
  achieved: number;
  unit: string;
  status: 'on_track' | 'behind' | 'ahead' | 'at_risk';
}

// ============================================================================
// Coordination
// ============================================================================

export interface CoordinationMeeting {
  meetingId: string;
  date: string;
  type: 'cluster' | 'inter_agency' | 'donor' | 'government';
  participants: string[];
  agenda: string[];
  decisions: Decision[];
  actionPoints: ActionPoint[];
}

export interface Decision {
  decisionId: string;
  description: string;
  rationale: string;
  responsible: string[];
  implementationDate?: string;
}

export interface ActionPoint {
  actionId: string;
  description: string;
  responsible: string;
  deadline: string;
  status: 'pending' | 'in_progress' | 'completed' | 'overdue';
  priority: 'high' | 'medium' | 'low';
}

// ============================================================================
// Funding & Resources
// ============================================================================

export interface FundingAppeal {
  appealId: string;
  crisisId: string;
  name: string;
  amount: number; // USD
  currency: string;
  startDate: string;
  endDate: string;
  sectors: FundingSector[];
  donors: Donor[];
  totalPledged: number;
  totalReceived: number;
  fundingGap: number;
}

export interface FundingSector {
  sector: string;
  requirement: number;
  received: number;
  percentFunded: number;
}

export interface Donor {
  donorId: string;
  name: string;
  type: 'government' | 'multilateral' | 'foundation' | 'private' | 'individual';
  pledged: number;
  disbursed: number;
  conditions?: string[];
}

// ============================================================================
// Client Configuration
// ============================================================================

export interface ClientConfig {
  apiKey: string;
  apiSecret?: string;
  baseURL?: string;
  timeout?: number;
  retryAttempts?: number;
}

// ============================================================================
// API Response Types
// ============================================================================

export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: { code: string; message: string; details?: any };
  timestamp: number;
}

export interface PaginatedResponse<T> {
  data: T[];
  total: number;
  page: number;
  pageSize: number;
  hasMore: boolean;
}

// ============================================================================
// Export all types
// ============================================================================

export * from './types';
