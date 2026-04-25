/**
 * WIA-AGRI-029 Desert Agriculture Standard - TypeScript Type Definitions
 * @version 1.0.0
 * @standard WIA-AGRI-029
 */

// ============================================================================
// Core Types
// ============================================================================

export type CropType = 'date-palm' | 'cactus' | 'drought-resistant-grain' | 'desert-melon' | 'succulents' | 'other';
export type IrrigationType = 'drip' | 'subsurface' | 'micro-spray' | 'fog' | 'condensation' | 'solar-distillation';
export type WaterSource = 'groundwater' | 'desalinated' | 'recycled' | 'atmospheric' | 'fog-harvest' | 'mixed';
export type SoilType = 'sandy' | 'rocky' | 'saline' | 'clay' | 'loess' | 'mixed';
export type DesertificationLevel = 'low' | 'moderate' | 'high' | 'severe' | 'extreme';
export type AdaptationStatus = 'excellent' | 'good' | 'fair' | 'poor' | 'failed';

// ============================================================================
// Desert Farm Profile
// ============================================================================

export interface DesertFarm {
  farmId: string;
  name: string;
  location: LocationInfo;
  area: Measurement;
  climate: ClimateProfile;
  soilProfile: SoilProfile;
  waterManagement: WaterManagement;
  crops: CropInfo[];
  infrastructure: Infrastructure;
  sustainability: SustainabilityMetrics;
  challenges: EnvironmentalChallenge[];
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
}

export interface Measurement {
  value: number;
  unit: string;
}

export interface ClimateProfile {
  averageTemperature: TemperatureRange;
  extremeTemperature: TemperatureRange;
  annualRainfall: Measurement;
  humidity: HumidityProfile;
  solarRadiation: SolarProfile;
  windPatterns: WindProfile;
  desertificationType: string;
}

export interface TemperatureRange {
  min: number;
  max: number;
  average: number;
  unit: string;
}

export interface HumidityProfile {
  average: number; // percentage
  morning: number;
  afternoon: number;
  evening: number;
}

export interface SolarProfile {
  averageDailyRadiation: Measurement;
  peakRadiation: Measurement;
  sunshineHours: number;
  uvIndex: number;
}

export interface WindProfile {
  averageSpeed: Measurement;
  prevailingDirection: string;
  gustSpeed: Measurement;
  sandstormFrequency: number; // per year
}

// ============================================================================
// Soil Profile
// ============================================================================

export interface SoilProfile {
  type: SoilType;
  composition: SoilComposition;
  nutrients: NutrientProfile;
  salinity: SalinityLevel;
  organicMatter: Measurement;
  pH: number;
  waterRetention: number; // percentage
  infiltrationRate: Measurement;
  desertificationLevel: DesertificationLevel;
}

export interface SoilComposition {
  sand: number; // percentage
  silt: number; // percentage
  clay: number; // percentage
  gravel: number; // percentage
  rock: number; // percentage
}

export interface NutrientProfile {
  nitrogen: Measurement;
  phosphorus: Measurement;
  potassium: Measurement;
  calcium?: Measurement;
  magnesium?: Measurement;
  micronutrients?: MicronutrientLevel[];
}

export interface MicronutrientLevel {
  nutrient: string;
  level: Measurement;
  adequate: boolean;
}

export interface SalinityLevel {
  ec: number; // electrical conductivity (dS/m)
  classification: 'non-saline' | 'slightly-saline' | 'moderately-saline' | 'highly-saline' | 'extremely-saline';
  managementRequired: boolean;
}

// ============================================================================
// Water Management
// ============================================================================

export interface WaterManagement {
  sources: WaterSourceInfo[];
  irrigationSystem: IrrigationSystem;
  efficiency: WaterEfficiency;
  conservation: ConservationMeasures;
  quality: WaterQuality;
  usage: WaterUsageMetrics;
}

export interface WaterSourceInfo {
  sourceId: string;
  type: WaterSource;
  capacity: Measurement;
  quality: WaterQuality;
  cost: number;
  currency: string;
  reliability: number; // 0-100
  seasonal: boolean;
}

export interface IrrigationSystem {
  type: IrrigationType;
  coverage: number; // percentage of farm area
  automation: AutomationLevel;
  scheduling: IrrigationSchedule;
  sensors: SensorInfo[];
  efficiency: number; // percentage
}

export interface AutomationLevel {
  level: 'manual' | 'semi-automated' | 'fully-automated' | 'ai-controlled';
  features: string[];
  lastCalibrated: string; // ISO 8601 date
}

export interface IrrigationSchedule {
  frequency: string;
  duration: number; // minutes per session
  timeOfDay: string[];
  seasonal: boolean;
  demandBased: boolean;
}

export interface SensorInfo {
  sensorId: string;
  type: string;
  location: LocationInfo;
  status: 'active' | 'inactive' | 'maintenance';
  lastReading: string; // ISO 8601 timestamp
}

export interface WaterEfficiency {
  applicationEfficiency: number; // percentage
  distributionEfficiency: number; // percentage
  overallEfficiency: number; // percentage
  waterSavings: Measurement;
  benchmarkComparison: number; // percentage vs traditional
}

export interface ConservationMeasures {
  mulching: boolean;
  shadingNets: boolean;
  windbreaks: boolean;
  moistureBarriers: boolean;
  rainwaterHarvesting: boolean;
  grayWaterRecycling: boolean;
  condensationCollection: boolean;
  practices: string[];
}

export interface WaterQuality {
  pH: number;
  salinity: number;
  tds: number; // total dissolved solids (ppm)
  hardness: Measurement;
  contaminants: ContaminantLevel[];
  suitability: 'excellent' | 'good' | 'fair' | 'poor' | 'unsuitable';
}

export interface ContaminantLevel {
  contaminant: string;
  level: number;
  unit: string;
  limit: number;
  withinLimit: boolean;
}

export interface WaterUsageMetrics {
  dailyAverage: Measurement;
  weeklyTotal: Measurement;
  monthlyTotal: Measurement;
  perCropType: CropWaterUsage[];
  trend: 'increasing' | 'stable' | 'decreasing';
}

export interface CropWaterUsage {
  cropType: string;
  waterUsed: Measurement;
  efficiency: number; // percentage
  yieldPerLiter: number;
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
  adaptation: AdaptationMetrics;
  yield: YieldData;
  management: CropManagement;
}

export interface GrowthStage {
  stage: string;
  daysAfterPlanting: number;
  progressPercentage: number;
  expectedDuration: number; // days
  challenges: string[];
}

export interface CropHealth {
  overallScore: number; // 0-100
  vigor: number; // 0-100
  stressLevel: number; // 0-100
  diseases: DiseaseInfo[];
  pests: PestInfo[];
  environmentalStress: StressFactors;
}

export interface DiseaseInfo {
  disease: string;
  severity: 'low' | 'medium' | 'high';
  affectedArea: number; // percentage
  treatment: string;
  controlled: boolean;
}

export interface PestInfo {
  pest: string;
  population: 'low' | 'medium' | 'high';
  affectedArea: number; // percentage
  control: string;
  effectiveness: number; // percentage
}

export interface StressFactors {
  heat: number; // 0-100
  drought: number; // 0-100
  salinity: number; // 0-100
  wind: number; // 0-100
  sandDamage: number; // 0-100
}

export interface AdaptationMetrics {
  status: AdaptationStatus;
  heatTolerance: number; // 0-100
  droughtResistance: number; // 0-100
  salinityTolerance: number; // 0-100
  windResistance: number; // 0-100
  survivalRate: number; // percentage
}

export interface YieldData {
  currentEstimate: Measurement;
  targetYield: Measurement;
  progress: number; // percentage
  quality: 'premium' | 'standard' | 'below-standard';
  comparisonToConventional: number; // percentage
}

export interface CropManagement {
  fertilization: FertilizationPlan;
  pestControl: PestControlPlan;
  shading: ShadingStrategy;
  protection: ProtectionMeasures;
}

export interface FertilizationPlan {
  type: 'organic' | 'synthetic' | 'mixed';
  schedule: string;
  nutrients: NutrientApplication[];
  cost: number;
  currency: string;
}

export interface NutrientApplication {
  nutrient: string;
  amount: Measurement;
  frequency: string;
  method: string;
}

export interface PestControlPlan {
  strategy: 'biological' | 'chemical' | 'integrated';
  interventions: Intervention[];
  monitoring: string;
}

export interface Intervention {
  type: string;
  target: string;
  application: string;
  frequency: string;
  effectiveness: number; // percentage
}

export interface ShadingStrategy {
  type: 'natural' | 'artificial' | 'mixed' | 'none';
  coverage: number; // percentage
  schedule: string;
  effectiveness: number; // percentage
}

export interface ProtectionMeasures {
  windbreaks: boolean;
  sandFences: boolean;
  shelterbelts: boolean;
  greenhouseCovers: boolean;
  antiEvaporationScreens: boolean;
  measures: string[];
}

// ============================================================================
// Infrastructure
// ============================================================================

export interface Infrastructure {
  greenhouses: GreenhouseInfo[];
  shadingStructures: ShadingStructure[];
  waterStorage: WaterStorageInfo[];
  powerSystems: PowerSystem[];
  monitoring: MonitoringSystem;
}

export interface GreenhouseInfo {
  greenhouseId: string;
  type: string;
  area: Measurement;
  coolingSystem: string;
  ventilation: string;
  automation: boolean;
  cropsGrown: string[];
}

export interface ShadingStructure {
  structureId: string;
  type: string;
  coverage: Measurement;
  material: string;
  adjustable: boolean;
}

export interface WaterStorageInfo {
  storageId: string;
  type: 'tank' | 'reservoir' | 'pond' | 'underground';
  capacity: Measurement;
  currentLevel: number; // percentage
  material: string;
  evaporationPrevention: string[];
}

export interface PowerSystem {
  systemId: string;
  type: 'solar' | 'wind' | 'grid' | 'hybrid';
  capacity: Measurement;
  currentProduction: Measurement;
  efficiency: number; // percentage
  storage: boolean;
}

export interface MonitoringSystem {
  sensors: SensorDeployment[];
  dataCollection: DataCollectionInfo;
  analytics: AnalyticsCapability;
  alerts: AlertSystem;
}

export interface SensorDeployment {
  sensorType: string;
  count: number;
  coverage: string;
  dataFrequency: string;
}

export interface DataCollectionInfo {
  frequency: string;
  storage: string;
  retention: number; // days
  realtime: boolean;
}

export interface AnalyticsCapability {
  aiEnabled: boolean;
  predictiveModels: string[];
  dashboards: boolean;
  reporting: string;
}

export interface AlertSystem {
  enabled: boolean;
  types: string[];
  channels: string[];
  responseTime: number; // minutes
}

// ============================================================================
// Sustainability Metrics
// ============================================================================

export interface SustainabilityMetrics {
  waterFootprint: Measurement;
  carbonFootprint: Measurement;
  energyUse: EnergyMetrics;
  soilRegeneration: SoilRegenerationMetrics;
  biodiversity: BiodiversityMetrics;
  economicViability: EconomicMetrics;
}

export interface EnergyMetrics {
  totalConsumption: Measurement;
  renewablePercentage: number;
  fossilFuelUse: Measurement;
  efficiency: number; // percentage
}

export interface SoilRegenerationMetrics {
  organicMatterIncrease: number; // percentage per year
  erosionPrevention: number; // percentage
  carbonSequestration: Measurement;
  healthImprovement: number; // 0-100
}

export interface BiodiversityMetrics {
  speciesCount: number;
  habitatCreated: Measurement;
  pollinatorSupport: boolean;
  ecosystemServices: string[];
}

export interface EconomicMetrics {
  revenue: number;
  costs: number;
  profit: number;
  currency: string;
  roi: number; // percentage
  breakEvenAchieved: boolean;
}

// ============================================================================
// Environmental Challenges
// ============================================================================

export interface EnvironmentalChallenge {
  challengeId: string;
  type: string;
  severity: 'low' | 'medium' | 'high' | 'critical';
  impact: string;
  mitigation: MitigationStrategy;
  status: 'identified' | 'mitigating' | 'controlled' | 'resolved';
}

export interface MitigationStrategy {
  approach: string;
  actions: string[];
  timeline: string;
  cost: number;
  currency: string;
  effectiveness: number; // percentage
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

export class WIADesertAgricultureError extends Error {
  constructor(
    message: string,
    public code: string,
    public statusCode?: number
  ) {
    super(message);
    this.name = 'WIADesertAgricultureError';
  }
}
