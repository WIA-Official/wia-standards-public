/**
 * WIA-AGRI-030 Polar Agriculture Standard - TypeScript Type Definitions
 * @version 1.0.0
 * @standard WIA-AGRI-030
 */

// ============================================================================
// Core Types
// ============================================================================

export type CropType = 'cold-hardy-greens' | 'root-vegetables' | 'berries' | 'mushrooms' | 'algae' | 'hydroponics' | 'other';
export type FacilityType = 'greenhouse' | 'indoor-farm' | 'geodesic-dome' | 'underground' | 'hybrid';
export type HeatingSource = 'geothermal' | 'solar' | 'wind' | 'biomass' | 'waste-heat' | 'mixed';
export type LightingType = 'natural' | 'led-supplemental' | 'full-spectrum-led' | 'hybrid';
export type ClimateZone = 'arctic' | 'subarctic' | 'alpine' | 'antarctic' | 'polar-desert';
export type PermafrostStatus = 'stable' | 'thawing' | 'unstable' | 'degraded';

// ============================================================================
// Polar Farm Profile
// ============================================================================

export interface PolarFarm {
  farmId: string;
  name: string;
  location: LocationInfo;
  climateZone: ClimateZone;
  facility: FacilityInfo;
  environmental: EnvironmentalConditions;
  crops: CropInfo[];
  energyManagement: EnergyManagement;
  permafrost: PermafrostMonitoring;
  sustainability: SustainabilityMetrics;
  createdAt: string; // ISO 8601 timestamp
  updatedAt: string; // ISO 8601 timestamp
  version: string;
  standard: string;
}

export interface LocationInfo {
  latitude: number;
  longitude: number;
  elevation: number;
  region: string;
  country: string;
  timezone: string;
  polarNight: PolarNightInfo;
  midnightSun: MidnightSunInfo;
}

export interface PolarNightInfo {
  startDate: string; // MM-DD
  endDate: string; // MM-DD
  durationDays: number;
}

export interface MidnightSunInfo {
  startDate: string; // MM-DD
  endDate: string; // MM-DD
  durationDays: number;
}

// ============================================================================
// Facility Information
// ============================================================================

export interface FacilityInfo {
  type: FacilityType;
  area: Measurement;
  insulation: InsulationInfo;
  heating: HeatingSystem;
  lighting: LightingSystem;
  ventilation: VentilationSystem;
  automation: AutomationLevel;
  construction: ConstructionDetails;
}

export interface Measurement {
  value: number;
  unit: string;
}

export interface InsulationInfo {
  type: string;
  rValue: number;
  thickness: Measurement;
  material: string[];
  heatLoss: Measurement; // watts per square meter
}

export interface HeatingSystem {
  primary: HeatingSource;
  backup: HeatingSource[];
  capacity: Measurement; // kW
  efficiency: number; // percentage
  temperature: TemperatureControl;
  zoneBased: boolean;
}

export interface TemperatureControl {
  target: number; // celsius
  range: { min: number; max: number };
  nightSetback: number; // celsius
  precision: number; // +/- celsius
}

export interface LightingSystem {
  type: LightingType;
  spectrum: string[];
  intensity: Measurement; // lumens or PAR
  photoperiod: PhotoperiodControl;
  coverage: number; // percentage of growing area
  efficiency: number; // lumens per watt
}

export interface PhotoperiodControl {
  dayLength: number; // hours
  dawn: DimmingProfile;
  dusk: DimmingProfile;
  adjustable: boolean;
  seasonal: boolean;
}

export interface DimmingProfile {
  duration: number; // minutes
  curve: 'linear' | 'logarithmic' | 'stepped';
}

export interface VentilationSystem {
  type: string;
  airExchangeRate: number; // per hour
  heatRecovery: boolean;
  heatRecoveryEfficiency?: number; // percentage
  co2Control: boolean;
  filtration: boolean;
}

export interface AutomationLevel {
  level: 'manual' | 'semi-automated' | 'fully-automated' | 'ai-controlled';
  sensors: SensorType[];
  actuators: string[];
  monitoring: boolean;
  remoteControl: boolean;
}

export type SensorType = 'temperature' | 'humidity' | 'light' | 'co2' | 'soil-moisture' | 'ph' | 'ec';

export interface ConstructionDetails {
  material: string[];
  foundation: string;
  permafrostProtection: boolean;
  windResistance: Measurement; // km/h
  snowLoadCapacity: Measurement; // kg/m²
  buildYear: number;
}

// ============================================================================
// Environmental Conditions
// ============================================================================

export interface EnvironmentalConditions {
  outdoor: OutdoorConditions;
  indoor: IndoorConditions;
  extremes: ExtremeConditions;
}

export interface OutdoorConditions {
  temperature: TemperatureStats;
  daylight: DaylightStats;
  snowCover: SnowCoverInfo;
  permafrost: PermafrostInfo;
  wind: WindStats;
}

export interface TemperatureStats {
  current: number;
  daily: { min: number; max: number; avg: number };
  monthly: { min: number; max: number; avg: number };
  annual: { min: number; max: number; avg: number };
  unit: string;
}

export interface DaylightStats {
  current: number; // hours
  sunrise?: string; // HH:MM or null during polar night
  sunset?: string; // HH:MM or null during polar night
  solarElevation: number; // degrees
  seasonalPattern: string;
}

export interface SnowCoverInfo {
  depth: Measurement;
  density: number; // kg/m³
  lastSnowfall: string; // ISO 8601 date
  accumulationRate: Measurement; // cm per week
}

export interface PermafrostInfo {
  depth: Measurement;
  temperature: number; // celsius
  activeLayerThickness: Measurement;
  status: PermafrostStatus;
}

export interface WindStats {
  speed: { current: number; avg: number; max: number };
  direction: string;
  gustSpeed: number;
  unit: string;
  windChill: number; // celsius
}

export interface IndoorConditions {
  temperature: number;
  humidity: number; // percentage
  co2: number; // ppm
  light: Measurement; // PAR or lumens
  vpdNeed: number; // kPa (Vapor Pressure Deficit)
  optimal: boolean;
}

export interface ExtremeConditions {
  recordLowTemp: number;
  recordHighWind: number;
  maxSnowLoad: number;
  polarNightAdaptation: string[];
  emergencyProtocols: string[];
}

// ============================================================================
// Crop Information
// ============================================================================

export interface CropInfo {
  cropId: string;
  type: CropType;
  variety: string;
  area: Measurement;
  plantingDate: string; // ISO 8601 date
  expectedHarvestDate: string; // ISO 8601 date
  growthStage: GrowthStage;
  health: CropHealth;
  coldAdaptation: ColdAdaptationMetrics;
  yield: YieldData;
  lightRequirement: LightRequirement;
}

export interface GrowthStage {
  stage: string;
  daysAfterPlanting: number;
  progressPercentage: number;
  expectedDuration: number; // days
}

export interface CropHealth {
  overallScore: number; // 0-100
  vigor: number; // 0-100
  stressLevel: number; // 0-100
  diseases: string[];
  pests: string[];
  deficiencies: string[];
}

export interface ColdAdaptationMetrics {
  frostTolerance: number; // minimum temperature in celsius
  lightEfficiency: number; // 0-100
  growthRateRatio: number; // compared to optimal conditions
  qualityScore: number; // 0-100
}

export interface YieldData {
  currentEstimate: Measurement;
  targetYield: Measurement;
  progress: number; // percentage
  quality: 'premium' | 'standard' | 'below-standard';
}

export interface LightRequirement {
  minimumPAR: number;
  optimalPAR: number;
  dayLength: number; // hours
  supplementalLighting: boolean;
}

// ============================================================================
// Energy Management
// ============================================================================

export interface EnergyManagement {
  sources: EnergySource[];
  consumption: EnergyConsumption;
  efficiency: EnergyEfficiency;
  storage: EnergyStorage;
  costAnalysis: EnergyCostAnalysis;
}

export interface EnergySource {
  sourceId: string;
  type: HeatingSource;
  capacity: Measurement;
  currentOutput: Measurement;
  availability: number; // percentage
  renewable: boolean;
  carbonFootprint: Measurement; // kg CO2 per kWh
}

export interface EnergyConsumption {
  total: Measurement; // kWh per day
  heating: Measurement;
  lighting: Measurement;
  ventilation: Measurement;
  automation: Measurement;
  other: Measurement;
  trend: 'increasing' | 'stable' | 'decreasing';
}

export interface EnergyEfficiency {
  heatingEfficiency: number; // percentage
  lightingEfficiency: number; // lumens per watt
  overallPUE: number; // Power Usage Effectiveness
  improvements: string[];
}

export interface EnergyStorage {
  type: 'battery' | 'thermal' | 'none';
  capacity?: Measurement; // kWh
  currentLevel?: number; // percentage
  backup: boolean;
}

export interface EnergyCostAnalysis {
  totalCost: number;
  costPerKWh: number;
  costPerKgYield: number;
  currency: string;
  savings: number; // compared to grid-only
}

// ============================================================================
// Permafrost Monitoring
// ============================================================================

export interface PermafrostMonitoring {
  status: PermafrostStatus;
  measurements: PermafrostMeasurement[];
  impacts: PermafrostImpact[];
  mitigation: MitigationStrategy[];
  alerts: PermafrostAlert[];
}

export interface PermafrostMeasurement {
  measurementId: string;
  timestamp: string; // ISO 8601 timestamp
  depth: Measurement;
  temperature: number; // celsius
  activeLayerThickness: Measurement;
  moistureContent: number; // percentage
  settlement: Measurement; // cm
}

export interface PermafrostImpact {
  type: string;
  severity: 'low' | 'medium' | 'high' | 'critical';
  description: string;
  affectedArea: Measurement;
}

export interface MitigationStrategy {
  strategyId: string;
  name: string;
  description: string;
  implementation: string;
  effectiveness: number; // percentage
  cost: number;
  currency: string;
}

export interface PermafrostAlert {
  alertId: string;
  type: string;
  severity: 'low' | 'medium' | 'high' | 'critical';
  message: string;
  timestamp: string; // ISO 8601 timestamp
  acknowledged: boolean;
}

// ============================================================================
// Sustainability Metrics
// ============================================================================

export interface SustainabilityMetrics {
  carbonFootprint: Measurement;
  energyIntensity: Measurement; // kWh per kg yield
  waterUsage: Measurement;
  wasteReduction: number; // percentage
  localFoodSecurity: FoodSecurityMetrics;
  economicViability: EconomicMetrics;
}

export interface FoodSecurityMetrics {
  communityServed: number; // population
  freshProduceCoverage: number; // percentage of local demand
  yearRoundProduction: boolean;
  nutritionalValue: string;
}

export interface EconomicMetrics {
  revenue: number;
  costs: number;
  profit: number;
  currency: string;
  roi: number; // percentage
  paybackPeriod: number; // years
}

// ============================================================================
// API Client Configuration
// ============================================================================

export interface ClientConfig {
  apiKey?: string;
  baseURL: string;
  timeout?: number;
  retryAttempts?: number;
}

export interface AuthTokens {
  accessToken: string;
  refreshToken?: string;
  expiresAt: number;
}

// ============================================================================
// Error Types
// ============================================================================

export class WIAPolarAgricultureError extends Error {
  constructor(
    message: string,
    public code: string,
    public statusCode?: number
  ) {
    super(message);
    this.name = 'WIAPolarAgricultureError';
  }
}
