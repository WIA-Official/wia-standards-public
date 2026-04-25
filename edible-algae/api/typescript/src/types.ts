/**
 * WIA-AGRI-033 Edible Algae Standard - TypeScript Type Definitions
 * @module @wia/edible-algae/types
 */

// ============================================================================
// Cultivation Types
// ============================================================================

export interface AlgaeCulture {
  cultureId: string;
  species: AlgaeSpecies;
  strain: string;
  cultivationType: CultivationType;
  facility: CultivationFacility;
  volume: number; // liters
  density: number; // cells/mL
  growthPhase: GrowthPhase;
  startDate: string;
  harvestDate?: string;
  status: 'inoculation' | 'exponential' | 'stationary' | 'harvest_ready' | 'harvested';
}

export type AlgaeSpecies = 'spirulina' | 'chlorella' | 'nannochloropsis' | 'dunaliella' | 'haematococcus' | 'kelp' | 'nori' | 'wakame' | 'other';
export type CultivationType = 'photobioreactor' | 'open_pond' | 'offshore_farm' | 'vertical_system' | 'hybrid';
export type GrowthPhase = 'lag' | 'exponential' | 'stationary' | 'decline';

export interface CultivationFacility {
  facilityId: string;
  name: string;
  location: { latitude: number; longitude: number };
  type: CultivationType;
  totalCapacity: number; // liters or hectares
  utilizationRate: number; // percentage
  waterSource: WaterSource[];
  energySource: EnergySource[];
}

export type WaterSource = 'freshwater' | 'seawater' | 'brackish' | 'wastewater' | 'recycled';
export type EnergySource = 'solar' | 'grid' | 'wind' | 'geothermal' | 'hybrid';

// ============================================================================
// Growth Monitoring
// ============================================================================

export interface GrowthMetrics {
  timestamp: number;
  cultureId: string;
  density: number; // cells/mL or g/L
  pH: number;
  temperature: number; // Celsius
  dissolvedOxygen: number; // mg/L
  co2Level: number; // percentage
  lightIntensity: number; // μmol/m²/s
  nutrientLevels: NutrientLevels;
  contamination: boolean;
  contaminants?: string[];
}

export interface NutrientLevels {
  nitrogen: number; // mg/L
  phosphorus: number; // mg/L
  potassium: number; // mg/L
  iron: number; // mg/L
  trace_elements?: Record<string, number>;
}

export interface GrowthRate {
  specificGrowthRate: number; // per day
  doublingTime: number; // hours
  productivity: number; // g/L/day
  photosynthetic Efficiency: number; // percentage
}

// ============================================================================
// Harvest & Processing
// ============================================================================

export interface Harvest {
  harvestId: string;
  cultureId: string;
  date: string;
  volume: number; // liters
  biomassYield: number; // kg
  dryWeight: number; // kg
  moisture Content: number; // percentage
  quality: QualityMetrics;
  processingMethod: ProcessingMethod;
}

export type ProcessingMethod = 'spray_drying' | 'freeze_drying' | 'sun_drying' | 'centrifugation' | 'filtration' | 'extraction';

export interface QualityMetrics {
  proteinContent: number; // percentage
  lipidContent: number; // percentage
  carbohydrateContent: number; // percentage
  pigmentContent: Record<string, number>; // mg/g
  heavyMetals: Record<string, number>; // ppm
  microbialCount: number; // CFU/g
  contaminants: string[];
  certified: boolean;
  certifications?: string[];
}

// ============================================================================
// Products & Applications
// ============================================================================

export interface AlgaeProduct {
  productId: string;
  name: string;
  species: AlgaeSpecies;
  form: ProductForm;
  application: ProductApplication[];
  composition: NutritionalProfile;
  packaging: PackagingInfo;
  shelfLife: number; // days
  price: number; // USD per kg
  certifications: string[];
}

export type ProductForm = 'powder' | 'tablet' | 'capsule' | 'liquid' | 'paste' | 'fresh' | 'dried_sheets';
export type ProductApplication = 'dietary_supplement' | 'food_ingredient' | 'animal_feed' | 'cosmetics' | 'biofuel' | 'fertilizer' | 'pharmaceutical';

export interface NutritionalProfile {
  protein: number; // g per 100g
  carbohydrates: number;
  fats: number;
  fiber: number;
  omega3: number;
  omega6: number;
  vitamins: Record<string, number>;
  minerals: Record<string, number>;
  antioxidants: Record<string, number>;
  caloriesPerHundredGrams: number;
}

export interface PackagingInfo {
  material: string;
  size: number;
  unit: string;
  recyclable: boolean;
  biodegradable: boolean;
}

// ============================================================================
// Environmental & Sustainability
// ============================================================================

export interface SustainabilityMetrics {
  facilityId: string;
  period: { start: string; end: string };
  co2Captured: number; // kg
  oxygenProduced: number; // kg
  waterUsage: number; // liters
  waterRecycled: number; // liters
  energyConsumption: number; // kWh
  renewableEnergyPercent: number;
  nitrogenRecovered: number; // kg
  phosphorusRecovered: number; // kg
  carbonFootprint: number; // kg CO2e
}

export interface BioremedationData {
  wastewaterTreated: number; // liters
  nitrogenRemoved: number; // kg
  phosphorusRemoved: number; // kg
  heavyMetalsRemoved: Record<string, number>;
  co2Sequestered: number; // kg
  efficiency: number; // percentage
}

// ============================================================================
// Research & Development
// ============================================================================

export interface StrainImprovement {
  projectId: string;
  baseStrain: string;
  targetTraits: string[];
  method: 'selective_breeding' | 'mutagenesis' | 'genetic_engineering' | 'hybrid';
  stage: 'research' | 'development' | 'testing' | 'commercialization';
  improvements: Record<string, number>; // trait -> percentage improvement
  regulatoryStatus?: string;
}

export interface Experiment {
  experimentId: string;
  title: string;
  objective: string;
  variables: ExperimentalVariable[];
  controlGroup: string;
  treatmentGroups: string[];
  duration: number; // days
  results?: ExperimentResults;
  status: 'planned' | 'ongoing' | 'completed' | 'cancelled';
}

export interface ExperimentalVariable {
  name: string;
  type: 'light' | 'temperature' | 'nutrient' | 'co2' | 'pH' | 'salinity' | 'other';
  levels: number[];
  unit: string;
}

export interface ExperimentResults {
  biomassProduction: number[];
  growthRate: number[];
  proteinContent: number[];
  statisticalSignificance: boolean;
  conclusions: string[];
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
