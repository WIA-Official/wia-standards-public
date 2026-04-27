/**
 * WIA-AGRI-032 Seaweed Farming Standard - TypeScript Type Definitions
 * @version 1.0.0
 * @standard WIA-AGRI-032
 */

export type SeaweedType = 'kelp' | 'nori' | 'wakame' | 'dulse' | 'spirulina' | 'sea-lettuce' | 'other';
export type FarmingMethod = 'longline' | 'raft' | 'bottom-culture' | 'tank-culture' | 'integrated';
export type GrowthStage = 'spore' | 'juvenile' | 'vegetative' | 'mature' | 'reproductive';
export type QualityGrade = 'premium' | 'grade-a' | 'grade-b' | 'industrial' | 'below-standard';

export interface SeaweedFarm {
  farmId: string;
  name: string;
  location: OceanLocation;
  farmingMethod: FarmingMethod;
  species: SeaweedSpecies[];
  waterConditions: WaterConditions;
  cultivation: CultivationSystem;
  harvest: HarvestData;
  processing: ProcessingFacility;
  sustainability: SustainabilityMetrics;
  createdAt: string;
  updatedAt: string;
  version: string;
  standard: string;
}

export interface OceanLocation {
  latitude: number;
  longitude: number;
  waterBody: string;
  depth: Measurement;
  coastalDistance: Measurement;
}

export interface Measurement {
  value: number;
  unit: string;
}

export interface SeaweedSpecies {
  speciesId: string;
  name: string;
  scientificName: string;
  type: SeaweedType;
  cultivationArea: Measurement;
  growthStage: GrowthStage;
  biomass: Measurement;
  growthRate: Measurement;
  harvestCycle: number; // days
  yield: YieldMetrics;
  quality: QualityMetrics;
}

export interface YieldMetrics {
  projectedYield: Measurement;
  actualYield: Measurement;
  yieldPerArea: Measurement;
  quality: QualityGrade;
}

export interface QualityMetrics {
  color: string;
  texture: string;
  thickness: Measurement;
  moisture: number; // percentage
  contamination: ContaminationTest[];
  nutritionalValue: NutritionalProfile;
}

export interface ContaminationTest {
  contaminant: string;
  level: number;
  unit: string;
  limit: number;
  safe: boolean;
}

export interface NutritionalProfile {
  protein: number; // percentage
  carbohydrates: number;
  fiber: number;
  minerals: MineralContent[];
  vitamins: VitaminContent[];
}

export interface MineralContent {
  mineral: string;
  amount: Measurement;
}

export interface VitaminContent {
  vitamin: string;
  amount: Measurement;
}

export interface WaterConditions {
  temperature: Measurement;
  salinity: Measurement;
  pH: number;
  nutrients: NutrientLevels;
  clarity: Measurement;
  current: CurrentProfile;
}

export interface NutrientLevels {
  nitrogen: Measurement;
  phosphorus: Measurement;
  iron: Measurement;
  silicate?: Measurement;
}

export interface CurrentProfile {
  speed: Measurement;
  direction: string;
  tidal: boolean;
}

export interface CultivationSystem {
  infrastructure: Infrastructure[];
  seeding: SeedingProgram;
  monitoring: MonitoringSystem;
  maintenance: MaintenanceSchedule;
}

export interface Infrastructure {
  type: string;
  quantity: number;
  material: string;
  length?: Measurement;
  capacity: Measurement;
  condition: string;
}

export interface SeedingProgram {
  method: string;
  density: Measurement;
  schedule: string;
  sources: string[];
  geneticDiversity: boolean;
}

export interface MonitoringSystem {
  sensors: SensorInfo[];
  inspectionFrequency: string;
  dataLogging: boolean;
  remoteSensing: boolean;
}

export interface SensorInfo {
  sensorType: string;
  count: number;
  updateFrequency: string;
}

export interface MaintenanceSchedule {
  lastMaintenance: string;
  nextMaintenance: string;
  tasks: string[];
  frequency: string;
}

export interface HarvestData {
  lastHarvest: string;
  nextPlannedHarvest: string;
  harvestMethod: string;
  totalHarvested: Measurement;
  quality: QualityDistribution[];
  postHarvestHandling: string[];
}

export interface QualityDistribution {
  grade: QualityGrade;
  percentage: number;
  biomass: Measurement;
}

export interface ProcessingFacility {
  hasProcessing: boolean;
  capacity?: Measurement;
  processes: ProcessingMethod[];
  products: SeaweedProduct[];
  certification: string[];
}

export interface ProcessingMethod {
  method: string;
  capacity: Measurement;
  efficiency: number; // percentage
  energyUse: Measurement;
}

export interface SeaweedProduct {
  productId: string;
  name: string;
  category: string;
  applications: string[];
  marketValue: number;
  currency: string;
}

export interface SustainabilityMetrics {
  carbonSequestration: Measurement;
  nutrientBioextraction: NutrientRemoval;
  biodiversityImpact: string;
  ecosystemServices: string[];
  certification: Certification[];
  economicMetrics: EconomicMetrics;
}

export interface NutrientRemoval {
  nitrogen: Measurement;
  phosphorus: Measurement;
  waterQualityImprovement: string;
}

export interface Certification {
  certificationId: string;
  type: string;
  issuingBody: string;
  issueDate: string;
  expiryDate: string;
  status: string;
}

export interface EconomicMetrics {
  revenue: number;
  costs: number;
  profit: number;
  currency: string;
  pricePerKg: number;
}

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

export class WIASeaweedFarmingError extends Error {
  constructor(
    message: string,
    public code: string,
    public statusCode?: number
  ) {
    super(message);
    this.name = 'WIASeaweedFarmingError';
  }
}
