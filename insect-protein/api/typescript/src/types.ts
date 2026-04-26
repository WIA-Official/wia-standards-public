/**
 * WIA-AGRI-025 Insect Protein Standard - TypeScript Type Definitions
 * @version 1.0.0
 * @standard WIA-AGRI-025
 */

export type InsectSpecies = 'black-soldier-fly' | 'crickets' | 'mealworms' | 'silkworms' | 'grasshoppers' | 'other';
export type ProductionStage = 'eggs' | 'larvae' | 'pupae' | 'adults' | 'processed';
export type ProcessingMethod = 'drying' | 'grinding' | 'extraction' | 'whole' | 'fractionation';
export type ProductType = 'whole-insect' | 'protein-powder' | 'oil' | 'meal' | 'chitin' | 'fertilizer';
export type FeedType = 'organic-waste' | 'agricultural-residue' | 'food-waste' | 'formulated-feed';

export interface InsectFarm {
  farmId: string;
  name: string;
  location: LocationInfo;
  species: SpeciesInfo[];
  production: ProductionSystem;
  processing: ProcessingFacility;
  quality: QualityControl;
  sustainability: SustainabilityMetrics;
  compliance: RegulatoryCompliance;
  createdAt: string;
  updatedAt: string;
  version: string;
  standard: string;
}

export interface LocationInfo {
  address: string;
  city: string;
  country: string;
  coordinates: { latitude: number; longitude: number };
  facilityType: 'indoor' | 'outdoor' | 'hybrid';
}

export interface SpeciesInfo {
  speciesId: string;
  species: InsectSpecies;
  strain: string;
  quantity: number;
  biomass: Measurement;
  stage: ProductionStage;
  growth: GrowthMetrics;
  health: HealthStatus;
  genetics: GeneticInfo;
}

export interface Measurement {
  value: number;
  unit: string;
}

export interface GrowthMetrics {
  averageWeight: Measurement;
  growthRate: Measurement; // per day
  mortality: number; // percentage
  developmentTime: number; // days
  feedConversionRatio: number;
}

export interface HealthStatus {
  overall: 'healthy' | 'stressed' | 'diseased';
  diseases: string[];
  parasites: string[];
  preventiveMeasures: string[];
}

export interface GeneticInfo {
  source: string;
  geneticallyModified: boolean;
  lineage: string;
  traits: string[];
}

export interface ProductionSystem {
  capacity: Measurement;
  currentProduction: Measurement;
  rearing: RearingSystem;
  feeding: FeedingProgram;
  environment: EnvironmentalControl;
  automation: AutomationLevel;
}

export interface RearingSystem {
  type: string;
  containers: ContainerInfo[];
  density: Measurement; // per square meter
  temperature: TemperatureControl;
  humidity: HumidityControl;
  ventilation: VentilationSystem;
}

export interface ContainerInfo {
  containerId: string;
  type: string;
  capacity: number;
  currentPopulation: number;
  stage: ProductionStage;
  condition: string;
}

export interface TemperatureControl {
  target: number; // celsius
  range: { min: number; max: number };
  controlMethod: string;
}

export interface HumidityControl {
  target: number; // percentage
  range: { min: number; max: number };
  controlMethod: string;
}

export interface VentilationSystem {
  airExchangeRate: number; // per hour
  co2Control: boolean;
  filtration: boolean;
}

export interface FeedingProgram {
  feedType: FeedType;
  composition: FeedComposition;
  dailyRation: Measurement;
  feedingFrequency: string;
  feedConversionRatio: number;
  wasteUtilization: number; // percentage
}

export interface FeedComposition {
  moisture: number; // percentage
  protein: number;
  carbohydrates: number;
  fat: number;
  fiber: number;
  minerals: string[];
}

export interface EnvironmentalControl {
  temperature: number; // celsius
  humidity: number; // percentage
  lighting: LightingSystem;
  airQuality: AirQualityMetrics;
  biosecurity: BiosecurityMeasures;
}

export interface LightingSystem {
  type: string;
  photoperiod: number; // hours
  intensity: Measurement;
  automated: boolean;
}

export interface AirQualityMetrics {
  co2: number; // ppm
  ammonia: number; // ppm
  dust: Measurement;
  acceptable: boolean;
}

export interface BiosecurityMeasures {
  protocols: string[];
  quarantine: boolean;
  sanitization: string;
  pestControl: string;
}

export interface AutomationLevel {
  level: 'manual' | 'semi-automated' | 'fully-automated';
  features: string[];
  sensors: SensorInfo[];
  monitoring: boolean;
}

export interface SensorInfo {
  sensorType: string;
  count: number;
  updateFrequency: string;
}

export interface ProcessingFacility {
  hasProcessing: boolean;
  capacity?: Measurement;
  methods: ProcessingLine[];
  products: InsectProduct[];
  packaging: PackagingInfo;
}

export interface ProcessingLine {
  lineId: string;
  method: ProcessingMethod;
  capacity: Measurement; // per day
  temperature?: number;
  duration?: number; // minutes
  yield: number; // percentage
}

export interface InsectProduct {
  productId: string;
  name: string;
  type: ProductType;
  composition: NutritionalComposition;
  applications: string[];
  packaging: string;
  shelfLife: number; // days
  certifications: string[];
}

export interface NutritionalComposition {
  protein: number; // percentage
  fat: number;
  carbohydrates: number;
  fiber: number;
  minerals: MineralContent[];
  vitamins: VitaminContent[];
  aminoAcids: AminoAcidProfile[];
}

export interface MineralContent {
  mineral: string;
  amount: Measurement;
}

export interface VitaminContent {
  vitamin: string;
  amount: Measurement;
}

export interface AminoAcidProfile {
  aminoAcid: string;
  amount: number; // g per 100g
  essential: boolean;
}

export interface PackagingInfo {
  materials: string[];
  sustainable: boolean;
  sizes: string[];
  labeling: LabelingInfo;
}

export interface LabelingInfo {
  nutritionalFacts: boolean;
  allergenWarnings: string[];
  certificationMarks: string[];
  tracabilityCode: boolean;
}

export interface QualityControl {
  microbiological: MicrobiologicalTesting;
  chemical: ChemicalTesting;
  nutritional: NutritionalTesting;
  sensory: SensoryEvaluation;
  certification: QualityCertification[];
}

export interface MicrobiologicalTesting {
  totalPlateCount: number;
  coliforms: number;
  salmonella: boolean;
  listeria: boolean;
  yeastMold: number;
  testFrequency: string;
}

export interface ChemicalTesting {
  heavyMetals: ContaminantTest[];
  pesticides: ContaminantTest[];
  antibiotics: ContaminantTest[];
  testFrequency: string;
}

export interface ContaminantTest {
  contaminant: string;
  detected: boolean;
  level?: number;
  unit?: string;
  limit: number;
  pass: boolean;
}

export interface NutritionalTesting {
  protein: number; // percentage
  fat: number;
  moisture: number;
  ash: number;
  testFrequency: string;
  certified: boolean;
}

export interface SensoryEvaluation {
  appearance: string;
  color: string;
  odor: string;
  texture: string;
  overallScore: number; // 0-100
}

export interface QualityCertification {
  certificationId: string;
  type: string;
  issuingBody: string;
  issueDate: string;
  expiryDate: string;
  scope: string[];
}

export interface SustainabilityMetrics {
  resourceEfficiency: ResourceEfficiency;
  wasteManagement: WasteManagement;
  carbonFootprint: Measurement;
  circularEconomy: CircularEconomyMetrics;
  socialImpact: SocialImpact;
}

export interface ResourceEfficiency {
  feedConversionRatio: number;
  waterUsage: Measurement; // per kg protein
  landUse: Measurement; // per kg protein
  energyUse: Measurement; // per kg protein
  comparisonToLivestock: ComparisonMetrics;
}

export interface ComparisonMetrics {
  feedReduction: number; // percentage
  waterReduction: number;
  landReduction: number;
  greenhouseGasReduction: number;
}

export interface WasteManagement {
  organicWasteProcessed: Measurement;
  wasteConversionRate: number; // percentage
  byproducts: Byproduct[];
  zerowaste: boolean;
}

export interface Byproduct {
  name: string;
  type: string;
  quantity: Measurement;
  application: string;
  value: number;
  currency: string;
}

export interface CircularEconomyMetrics {
  wasteAsResource: boolean;
  byproductUtilization: number; // percentage
  closedLoopSystems: string[];
  partnerships: string[];
}

export interface SocialImpact {
  employment: number;
  communityBenefit: string[];
  foodSecurity: string;
  education: string[];
}

export interface RegulatoryCompliance {
  foodSafety: FoodSafetyCompliance;
  animalWelfare: AnimalWelfare;
  environmental: EnvironmentalCompliance;
  traceability: TraceabilitySystem;
}

export interface FoodSafetyCompliance {
  haccp: boolean;
  gmp: boolean;
  iso22000: boolean;
  localRegulations: string[];
  audits: AuditRecord[];
}

export interface AuditRecord {
  auditId: string;
  type: string;
  date: string;
  auditor: string;
  result: string;
  findings: string[];
}

export interface AnimalWelfare {
  standards: string[];
  humaneHarvesting: boolean;
  stressMinimization: string[];
  monitoring: boolean;
}

export interface EnvironmentalCompliance {
  permits: string[];
  emissions: EmissionControl;
  wasteDisposal: string;
  audits: string[];
}

export interface EmissionControl {
  airEmissions: Measurement;
  waterDischarge: Measurement;
  controlled: boolean;
  monitoring: boolean;
}

export interface TraceabilitySystem {
  batchTracking: boolean;
  blockchainEnabled: boolean;
  qrCodes: boolean;
  recordRetention: number; // years
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

export class WIAInsectProteinError extends Error {
  constructor(
    message: string,
    public code: string,
    public statusCode?: number
  ) {
    super(message);
    this.name = 'WIAInsectProteinError';
  }
}
