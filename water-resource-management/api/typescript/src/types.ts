/**
 * WIA Water Resource Management Standard - TypeScript Types
 *
 * Comprehensive types for sustainable water resource conservation and management
 *
 * @module @wia/water-resource-management/types
 * @version 1.0.0
 */

// ============================================================================
// Core Types
// ============================================================================

export type WaterResourceId = string;
export type WatershedId = string;
export type ReservoirId = string;
export type AquiferId = string;
export type WaterRightId = string;

export type WaterQuality = 'excellent' | 'good' | 'fair' | 'poor' | 'critical';
export type WaterSource = 'surface' | 'groundwater' | 'precipitation' | 'recycled' | 'desalinated';
export type AllocationPriority = 'domestic' | 'agricultural' | 'industrial' | 'environmental' | 'recreational';
export type ConservationStatus = 'normal' | 'advisory' | 'watch' | 'warning' | 'emergency';

// ============================================================================
// Watershed Types
// ============================================================================

export interface WatershedData {
  id: WatershedId;
  name: string;
  area: number; // km²
  location: GeoLocation;
  tributaries: string[];
  landUse: LandUseDistribution;
  precipitation: PrecipitationData;
  runoff: RunoffData;
  waterQuality: WaterQualityMetrics;
  ecosystemHealth: EcosystemHealth;
  metadata?: Record<string, unknown>;
}

export interface GeoLocation {
  latitude: number;
  longitude: number;
  elevation?: number;
  boundary?: GeoCoordinate[];
}

export interface GeoCoordinate {
  lat: number;
  lng: number;
}

export interface LandUseDistribution {
  forest: number; // percentage
  agriculture: number;
  urban: number;
  wetland: number;
  barren: number;
  other: number;
}

export interface PrecipitationData {
  annual: number; // mm
  monthly: number[];
  historical: HistoricalData;
  forecast: ForecastData;
}

export interface RunoffData {
  surfaceRunoff: number; // m³/s
  baseFlow: number;
  totalFlow: number;
  peak: number;
  averageAnnual: number;
}

export interface WaterQualityMetrics {
  pH: number;
  dissolvedOxygen: number; // mg/L
  turbidity: number; // NTU
  temperature: number; // °C
  conductivity: number; // μS/cm
  nutrients: NutrientLevels;
  contaminants: ContaminantLevels;
  overallQuality: WaterQuality;
}

export interface NutrientLevels {
  nitrogen: number; // mg/L
  phosphorus: number;
  potassium: number;
}

export interface ContaminantLevels {
  heavyMetals: Record<string, number>;
  pesticides: Record<string, number>;
  bacteria: number; // CFU/100mL
}

export interface EcosystemHealth {
  biodiversityIndex: number;
  habitatQuality: number;
  speciesRichness: number;
  indicators: string[];
}

// ============================================================================
// Water Allocation Types
// ============================================================================

export interface WaterAllocation {
  id: string;
  source: WaterSource;
  totalAvailable: number; // m³
  allocated: AllocationBreakdown;
  reserved: number; // m³ for environmental flow
  restrictions: AllocationRestriction[];
  timestamp: Date;
}

export interface AllocationBreakdown {
  domestic: number; // m³
  agricultural: number;
  industrial: number;
  environmental: number;
  recreational: number;
}

export interface AllocationRestriction {
  type: 'temporary' | 'permanent' | 'seasonal';
  sector: AllocationPriority;
  reduction: number; // percentage
  reason: string;
  startDate: Date;
  endDate?: Date;
}

// ============================================================================
// Reservoir Types
// ============================================================================

export interface ReservoirData {
  id: ReservoirId;
  name: string;
  location: GeoLocation;
  capacity: ReservoirCapacity;
  currentLevel: WaterLevel;
  inflow: number; // m³/s
  outflow: number; // m³/s
  storage: number; // m³
  waterQuality: WaterQualityMetrics;
  operations: ReservoirOperations;
  purpose: ReservoirPurpose[];
}

export interface ReservoirCapacity {
  total: number; // m³
  active: number;
  dead: number;
  flood: number;
  conservation: number;
}

export interface WaterLevel {
  elevation: number; // meters
  volume: number; // m³
  percentage: number; // % of capacity
  status: 'full' | 'normal' | 'low' | 'critical' | 'empty';
}

export interface ReservoirOperations {
  releaseSchedule: ReleaseSchedule[];
  maintenanceWindows: MaintenanceWindow[];
  emergencyProtocols: EmergencyProtocol[];
}

export interface ReleaseSchedule {
  startTime: Date;
  endTime: Date;
  rate: number; // m³/s
  purpose: string;
}

export interface MaintenanceWindow {
  startDate: Date;
  endDate: Date;
  type: string;
  impactLevel: 'low' | 'medium' | 'high';
}

export interface EmergencyProtocol {
  trigger: string;
  action: string;
  priority: number;
}

export type ReservoirPurpose =
  | 'water-supply'
  | 'flood-control'
  | 'hydropower'
  | 'irrigation'
  | 'recreation'
  | 'environmental';

// ============================================================================
// Groundwater Types
// ============================================================================

export interface GroundwaterData {
  id: AquiferId;
  name: string;
  type: AquiferType;
  location: GeoLocation;
  depth: DepthData;
  waterTable: WaterTableData;
  recharge: RechargeData;
  extraction: ExtractionData;
  quality: WaterQualityMetrics;
  sustainabilityIndex: number;
}

export type AquiferType = 'confined' | 'unconfined' | 'perched' | 'karst';

export interface DepthData {
  top: number; // meters below surface
  bottom: number;
  thickness: number;
}

export interface WaterTableData {
  currentDepth: number; // meters below surface
  historicalDepth: HistoricalData;
  seasonalVariation: number;
  trend: 'rising' | 'stable' | 'declining';
}

export interface RechargeData {
  natural: number; // m³/year
  artificial: number;
  total: number;
  sources: string[];
}

export interface ExtractionData {
  permitted: number; // m³/year
  actual: number;
  wells: WellData[];
  sustainability: number; // percentage
}

export interface WellData {
  id: string;
  location: GeoLocation;
  depth: number;
  capacity: number; // m³/day
  status: 'active' | 'inactive' | 'maintenance';
}

// ============================================================================
// Irrigation Types
// ============================================================================

export interface IrrigationSystem {
  id: string;
  name: string;
  type: IrrigationType;
  area: number; // hectares
  efficiency: number; // percentage
  waterSource: WaterSource[];
  schedule: IrrigationSchedule;
  monitoring: IrrigationMonitoring;
}

export type IrrigationType =
  | 'drip'
  | 'sprinkler'
  | 'surface'
  | 'subsurface'
  | 'center-pivot'
  | 'furrow';

export interface IrrigationSchedule {
  frequency: string;
  duration: number; // minutes
  waterVolume: number; // m³
  seasonalAdjustment: SeasonalAdjustment[];
}

export interface SeasonalAdjustment {
  season: 'spring' | 'summer' | 'fall' | 'winter';
  factor: number; // multiplier
}

export interface IrrigationMonitoring {
  soilMoisture: number; // percentage
  evapotranspiration: number; // mm/day
  cropWaterRequirement: number; // mm/day
  waterApplied: number; // mm
}

// ============================================================================
// Water Rights Types
// ============================================================================

export interface WaterRight {
  id: WaterRightId;
  holder: string;
  type: WaterRightType;
  source: WaterSource;
  priority: number;
  allocation: number; // m³/year
  conditions: RightCondition[];
  validFrom: Date;
  validUntil?: Date;
  status: 'active' | 'suspended' | 'revoked' | 'expired';
}

export type WaterRightType =
  | 'riparian'
  | 'appropriative'
  | 'groundwater'
  | 'storage'
  | 'transfer';

export interface RightCondition {
  type: string;
  description: string;
  compliance: boolean;
}

// ============================================================================
// Demand Forecasting Types
// ============================================================================

export interface DemandForecast {
  id: string;
  region: string;
  timeframe: TimeframeData;
  sectors: SectorDemand[];
  totalDemand: number; // m³
  confidence: number; // percentage
  scenarios: ForecastScenario[];
}

export interface TimeframeData {
  startDate: Date;
  endDate: Date;
  granularity: 'daily' | 'weekly' | 'monthly' | 'yearly';
}

export interface SectorDemand {
  sector: AllocationPriority;
  baseline: number; // m³
  forecast: number;
  growth: number; // percentage
  factors: string[];
}

export interface ForecastScenario {
  name: string;
  probability: number;
  demand: number; // m³
  assumptions: string[];
}

// ============================================================================
// Conservation Types
// ============================================================================

export interface ConservationPlan {
  id: string;
  name: string;
  status: ConservationStatus;
  measures: ConservationMeasure[];
  targets: ConservationTarget[];
  effectiveness: number; // percentage
  implementation: ImplementationPlan;
}

export interface ConservationMeasure {
  id: string;
  type: string;
  description: string;
  expectedSavings: number; // m³/year
  cost: number;
  priority: 'low' | 'medium' | 'high' | 'critical';
  status: 'planned' | 'active' | 'completed';
}

export interface ConservationTarget {
  metric: string;
  current: number;
  target: number;
  deadline: Date;
  progress: number; // percentage
}

export interface ImplementationPlan {
  phases: Phase[];
  timeline: Timeline;
  stakeholders: Stakeholder[];
  budget: number;
}

export interface Phase {
  name: string;
  startDate: Date;
  endDate: Date;
  deliverables: string[];
  status: 'pending' | 'active' | 'completed' | 'delayed';
}

export interface Timeline {
  start: Date;
  end: Date;
  milestones: Milestone[];
}

export interface Milestone {
  name: string;
  date: Date;
  achieved: boolean;
}

export interface Stakeholder {
  name: string;
  role: string;
  responsibilities: string[];
}

// ============================================================================
// Supporting Types
// ============================================================================

export interface HistoricalData {
  values: number[];
  timestamps: Date[];
  average: number;
  minimum: number;
  maximum: number;
}

export interface ForecastData {
  values: number[];
  dates: Date[];
  confidence: number[];
}

// ============================================================================
// Event Types
// ============================================================================

export interface WaterResourceEvent {
  type: WaterResourceEventType;
  timestamp: Date;
  data: unknown;
  severity: 'info' | 'warning' | 'critical';
  source: string;
}

export type WaterResourceEventType =
  | 'watershed-update'
  | 'allocation-change'
  | 'reservoir-level-change'
  | 'groundwater-alert'
  | 'conservation-status-change'
  | 'water-quality-alert'
  | 'demand-spike'
  | 'supply-shortage';

export type WaterResourceEventHandler = (event: WaterResourceEvent) => void;
