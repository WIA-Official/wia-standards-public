/**
 * WIA-AGRI-035 Space Agriculture Standard - TypeScript Type Definitions
 * @module @wia/space-agriculture/types
 */

// ============================================================================
// Space Farming System Types
// ============================================================================

export interface SpaceFarm {
  farmId: string;
  name: string;
  location: SpaceLocation;
  type: FarmType;
  volume: number; // cubic meters
  crops: CropModule[];
  lifeSupportIntegration: boolean;
  status: 'operational' | 'maintenance' | 'experimental' | 'offline';
  crew: number;
  autonomyLevel: number; // 0-100 percentage
}

export type FarmType = 'space_station' | 'lunar_base' | 'mars_habitat' | 'orbital_greenhouse' | 'asteroid_farm';

export interface SpaceLocation {
  facility: string;
  orbit?: 'LEO' | 'GEO' | 'lunar' | 'mars' | 'deep_space';
  coordinates?: { latitude: number; longitude: number; altitude: number };
  gravityLevel: number; // percentage of Earth gravity
  radiationLevel: number; // mSv/year
}

export interface CropModule {
  moduleId: string;
  name: string;
  crop: string;
  variety: string;
  growthMethod: GrowthMethod;
  area: number; // square meters
  capacity: number; // plants
  currentPlants: number;
  harvestCycle: number; // days
  status: 'seeding' | 'growing' | 'harvest_ready' | 'harvested';
}

export type GrowthMethod = 'aeroponics' | 'hydroponics' | 'soil_simulant' | 'aquaponics' | 'biofilm';

// ============================================================================
// Environmental Control
// ============================================================================

export interface EnvironmentalControl {
  moduleId: string;
  timestamp: number;
  atmosphere: AtmosphericConditions;
  lighting: LightingSystem;
  temperature: TemperatureControl;
  humidity: number; // percentage
  airflow: number; // m/s
  pressure: number; // kPa
}

export interface AtmosphericConditions {
  o2Level: number; // percentage
  co2Level: number; // ppm
  n2Level: number; // percentage
  totalPressure: number; // kPa
  contaminants: Contaminant[];
  airQualityIndex: number; // 0-100
}

export interface Contaminant {
  substance: string;
  concentration: number;
  unit: string;
  threshold: number;
  status: 'safe' | 'warning' | 'critical';
}

export interface LightingSystem {
  type: 'led' | 'solar' | 'hybrid';
  spectrum: SpectrumConfig;
  ppfd: number; // μmol/m²/s
  photoperiod: number; // hours
  powerConsumption: number; // watts
  efficiency: number; // μmol/J
}

export interface SpectrumConfig {
  red: number; // percentage
  blue: number;
  green: number;
  farRed: number;
  uv: number;
  infrared: number;
}

export interface TemperatureControl {
  ambient: number; // Celsius
  rootZone: number; // Celsius
  canopy: number; // Celsius
  target: number; // Celsius
  tolerance: number; // +/- Celsius
}

// ============================================================================
// Resource Management
// ============================================================================

export interface ResourceBudget {
  farmId: string;
  period: { start: string; end: string };
  water: WaterBudget;
  power: PowerBudget;
  nutrients: NutrientBudget;
  atmosphere: AtmosphereBudget;
  waste: WasteManagement;
}

export interface WaterBudget {
  totalAvailable: number; // liters
  consumedCrops: number;
  recycled: number;
  transpired: number;
  systemLosses: number;
  recyclingRate: number; // percentage
  closureRate: number; // percentage
}

export interface PowerBudget {
  totalAvailable: number; // kWh
  lighting: number;
  climate: number;
  pumps: number;
  monitoring: number;
  automation: number;
  peakDemand: number; // kW
  solarGeneration?: number; // kWh
}

export interface NutrientBudget {
  totalMass: number; // kg
  consumed: number;
  recycled: number;
  resupplyNeeded: number;
  elements: Record<string, number>; // element -> kg
}

export interface AtmosphereBudget {
  o2Produced: number; // kg
  o2Consumed: number;
  co2Consumed: number;
  co2Produced: number;
  waterVapor: number;
  closureRate: number; // percentage
}

export interface WasteManagement {
  biomassWaste: number; // kg
  inedibleBiomass: number;
  composted: number;
  recycledNutrients: number;
  methaneProduced?: number; // liters
  wasteToEnergy: number; // kWh
}

// ============================================================================
// Crop Production
// ============================================================================

export interface SpaceHarvest {
  harvestId: string;
  moduleId: string;
  crop: string;
  harvestDate: string;
  edibleBiomass: number; // kg
  inedibleBiomass: number; // kg
  totalBiomass: number; // kg
  harvestIndex: number; // edible/total
  quality: QualityAssessment;
  nutritionalValue: NutritionalAnalysis;
  crewFeedback?: CrewFeedback;
}

export interface QualityAssessment {
  appearance: number; // 0-10
  taste: number; // 0-10
  texture: number; // 0-10
  freshness: number; // 0-10
  contamination: boolean;
  shelfLife: number; // days
  safetyApproved: boolean;
}

export interface NutritionalAnalysis {
  calories: number; // kcal per 100g
  protein: number; // g
  carbohydrates: number;
  fats: number;
  fiber: number;
  vitamins: Record<string, number>;
  minerals: Record<string, number>;
  antioxidants: number; // mg GAE/100g
  waterContent: number; // percentage
}

export interface CrewFeedback {
  rating: number; // 1-5
  comments: string;
  preferences: string[];
  improvementSuggestions: string[];
}

// ============================================================================
// Life Support Integration
// ============================================================================

export interface LifeSupportMetrics {
  farmId: string;
  timestamp: number;
  oxygenContribution: number; // kg/day
  co2Removal: number; // kg/day
  waterPurification: number; // liters/day
  foodProduction: number; // kcal/day
  psychologicalBenefit: number; // 0-100 score
  crewTimeInvestment: number; // hours/week
}

export interface BioregenerativeRatio {
  foodClosure: number; // percentage
  waterClosure: number;
  atmosphereClosure: number;
  overallClosure: number;
  targetClosure: number;
  gap: number;
}

// ============================================================================
// Automation & Monitoring
// ============================================================================

export interface AutomationSystem {
  systemId: string;
  moduleId: string;
  sensors: Sensor[];
  actuators: Actuator[];
  aiController: AIController;
  robotics: RoboticSystem[];
  autonomyLevel: number; // percentage
  crewInterventionRequired: number; // hours/week
}

export interface Sensor {
  sensorId: string;
  type: string;
  location: string;
  currentValue: number;
  unit: string;
  calibrationDate: string;
  status: 'operational' | 'degraded' | 'failed';
  redundancy: boolean;
}

export interface Actuator {
  actuatorId: string;
  type: 'pump' | 'valve' | 'fan' | 'heater' | 'light' | 'other';
  status: 'active' | 'idle' | 'maintenance' | 'failed';
  powerDraw: number; // watts
  cyclesCompleted: number;
  reliability: number; // percentage
}

export interface AIController {
  modelId: string;
  version: string;
  capabilities: string[];
  decisionsMade: number;
  successRate: number; // percentage
  lastUpdate: string;
  humanOverrides: number;
}

export interface RoboticSystem {
  robotId: string;
  type: 'harvester' | 'pruner' | 'pollinator' | 'monitor' | 'maintenance';
  status: 'active' | 'charging' | 'maintenance' | 'offline';
  tasksCompleted: number;
  batteryLevel: number; // percentage
  autonomy: number; // percentage
}

// ============================================================================
// Research & Experiments
// ============================================================================

export interface SpaceAgExperiment {
  experimentId: string;
  title: string;
  principalInvestigator: string;
  objective: string;
  hypothesis: string;
  variables: ExperimentVariable[];
  duration: number; // days
  crops: string[];
  microgravityEffect: boolean;
  radiationEffect: boolean;
  status: 'planned' | 'active' | 'completed' | 'terminated';
  results?: ExperimentResults;
}

export interface ExperimentVariable {
  name: string;
  type: 'independent' | 'dependent' | 'controlled';
  values: number[];
  unit: string;
}

export interface ExperimentResults {
  growthRate: number;
  yieldComparison: number; // percentage vs Earth control
  morphologicalChanges: string[];
  nutritionalChanges: Record<string, number>;
  genomeExpression: string[];
  conclusions: string[];
  publications: string[];
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

export * from './types';
