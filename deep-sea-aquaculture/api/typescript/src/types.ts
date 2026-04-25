/**
 * WIA-AGRI-031 Deep-Sea Aquaculture Standard - TypeScript Type Definitions
 * @version 1.0.0
 * @standard WIA-AGRI-031
 */

export type SpeciesType = 'fish' | 'crustacean' | 'mollusk' | 'echinoderm' | 'cephalopod' | 'algae' | 'other';
export type FarmType = 'cage' | 'pen' | 'raft' | 'longline' | 'bottom-culture' | 'integrated';
export type WaterQuality = 'excellent' | 'good' | 'fair' | 'poor' | 'critical';
export type HealthStatus = 'healthy' | 'stressed' | 'diseased' | 'critical' | 'deceased';

export interface DeepSeaFarm {
  farmId: string;
  name: string;
  location: OceanLocation;
  depth: DepthProfile;
  farmType: FarmType;
  species: SpeciesInfo[];
  waterConditions: WaterConditions;
  equipment: FarmEquipment;
  monitoring: MonitoringSystem;
  harvest: HarvestMetrics;
  sustainability: SustainabilityMetrics;
  createdAt: string;
  updatedAt: string;
  version: string;
  standard: string;
}

export interface OceanLocation {
  latitude: number;
  longitude: number;
  oceanZone: string;
  eezCountry: string;
  distanceFromShore: Measurement;
}

export interface Measurement {
  value: number;
  unit: string;
}

export interface DepthProfile {
  minimum: number;
  maximum: number;
  average: number;
  unit: string;
  zone: 'epipelagic' | 'mesopelagic' | 'bathypelagic' | 'abyssopelagic';
}

export interface SpeciesInfo {
  speciesId: string;
  name: string;
  scientificName: string;
  type: SpeciesType;
  quantity: number;
  biomass: Measurement;
  growth: GrowthMetrics;
  health: HealthMetrics;
  feeding: FeedingProgram;
}

export interface GrowthMetrics {
  averageWeight: Measurement;
  growthRate: Measurement; // per day
  mortality: number; // percentage
  targetWeight: Measurement;
  daysToHarvest: number;
}

export interface HealthMetrics {
  status: HealthStatus;
  diseases: DiseaseInfo[];
  parasites: ParasiteInfo[];
  stressIndicators: StressIndicator[];
  veterinaryChecks: VeterinaryCheck[];
}

export interface DiseaseInfo {
  disease: string;
  prevalence: number; // percentage
  severity: 'low' | 'medium' | 'high';
  treatment: string;
  status: 'active' | 'controlled' | 'resolved';
}

export interface ParasiteInfo {
  parasite: string;
  loadLevel: 'low' | 'medium' | 'high';
  treatment: string;
  controlled: boolean;
}

export interface StressIndicator {
  indicator: string;
  level: number; // 0-100
  cause: string;
  mitigation: string;
}

export interface VeterinaryCheck {
  checkId: string;
  date: string;
  veterinarian: string;
  findings: string[];
  recommendations: string[];
}

export interface FeedingProgram {
  feedType: string;
  composition: NutrientComposition;
  dailyRation: Measurement;
  frequency: number; // times per day
  fcr: number; // Feed Conversion Ratio
  feedingMethod: 'manual' | 'automated' | 'demand';
}

export interface NutrientComposition {
  protein: number; // percentage
  fat: number;
  carbohydrates: number;
  vitamins: string[];
  minerals: string[];
}

export interface WaterConditions {
  temperature: Measurement;
  salinity: Measurement;
  dissolvedOxygen: Measurement;
  pH: number;
  turbidity: Measurement;
  current: CurrentProfile;
  quality: WaterQuality;
  pollutants: PollutantLevel[];
}

export interface CurrentProfile {
  speed: Measurement;
  direction: string;
  tidal: boolean;
  variability: 'low' | 'medium' | 'high';
}

export interface PollutantLevel {
  pollutant: string;
  concentration: Measurement;
  limit: number;
  withinLimit: boolean;
}

export interface FarmEquipment {
  cages: CageInfo[];
  feedingSystems: FeedingSystem[];
  monitoringDevices: MonitoringDevice[];
  harvestEquipment: string[];
  maintenance: MaintenanceSchedule;
}

export interface CageInfo {
  cageId: string;
  type: string;
  volume: Measurement;
  material: string;
  depthRange: { min: number; max: number };
  capacity: number;
  currentStock: number;
  condition: 'excellent' | 'good' | 'fair' | 'poor';
}

export interface FeedingSystem {
  systemId: string;
  type: string;
  automated: boolean;
  capacity: Measurement;
  coverage: string[];
  efficiency: number; // percentage
}

export interface MonitoringDevice {
  deviceId: string;
  type: string;
  parameters: string[];
  frequency: string;
  lastMaintenance: string;
  status: 'active' | 'inactive' | 'maintenance';
}

export interface MaintenanceSchedule {
  lastInspection: string;
  nextInspection: string;
  frequency: string;
  issues: MaintenanceIssue[];
}

export interface MaintenanceIssue {
  issueId: string;
  type: string;
  severity: 'low' | 'medium' | 'high' | 'critical';
  reported: string;
  status: 'open' | 'in-progress' | 'resolved';
}

export interface MonitoringSystem {
  realtime: boolean;
  sensors: SensorDeployment[];
  dataCollection: DataCollectionInfo;
  alerts: AlertSystem;
}

export interface SensorDeployment {
  sensorType: string;
  count: number;
  locations: string[];
  updateFrequency: string;
}

export interface DataCollectionInfo {
  frequency: string;
  storage: string;
  retention: number; // days
  cloudBackup: boolean;
}

export interface AlertSystem {
  enabled: boolean;
  thresholds: AlertThreshold[];
  notifications: string[];
  responseProtocol: string;
}

export interface AlertThreshold {
  parameter: string;
  min?: number;
  max?: number;
  unit: string;
}

export interface HarvestMetrics {
  totalHarvested: Measurement;
  quality: QualityGrade[];
  timing: HarvestTiming;
  yield: YieldAnalysis;
}

export interface QualityGrade {
  grade: string;
  percentage: number;
  criteria: string;
}

export interface HarvestTiming {
  lastHarvest: string;
  nextPlannedHarvest: string;
  seasonalPattern: string;
}

export interface YieldAnalysis {
  yieldPerCage: Measurement;
  survivalRate: number; // percentage
  marketReadyPercentage: number;
  projectedAnnualYield: Measurement;
}

export interface SustainabilityMetrics {
  environmentalImpact: EnvironmentalImpact;
  biosecurity: BiosecurityMeasures;
  certification: Certification[];
  economicViability: EconomicMetrics;
}

export interface EnvironmentalImpact {
  carbonFootprint: Measurement;
  nutrientDischarge: Measurement;
  ecosystemImpact: string;
  biodiversityEffect: string;
  mitigation: string[];
}

export interface BiosecurityMeasures {
  protocols: string[];
  quarantine: boolean;
  geneticManagement: boolean;
  escapeProtection: number; // percentage effectiveness
  diseaseScreening: boolean;
}

export interface Certification {
  certificationId: string;
  type: string;
  issuingBody: string;
  issueDate: string;
  expiryDate: string;
  status: 'active' | 'expired' | 'suspended';
}

export interface EconomicMetrics {
  revenue: number;
  operatingCosts: number;
  profit: number;
  currency: string;
  roi: number; // percentage
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

export class WIADeepSeaAquacultureError extends Error {
  constructor(
    message: string,
    public code: string,
    public statusCode?: number
  ) {
    super(message);
    this.name = 'WIADeepSeaAquacultureError';
  }
}
