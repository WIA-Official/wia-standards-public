/**
 * WIA-AGRI-034 Single Cell Protein Standard - TypeScript Type Definitions
 * @module @wia/single-cell-protein/types
 */

// ============================================================================
// Production Types
// ============================================================================

export interface SCPProduction {
  productionId: string;
  organism: OrganismType;
  strain: string;
  substrate: Substrate;
  bioreactor: Bioreactor;
  batchNumber: string;
  startDate: string;
  harvestDate?: string;
  volume: number; // liters
  biomassYield: number; // kg
  status: ProductionStatus;
}

export type OrganismType = 'bacteria' | 'yeast' | 'fungi' | 'microalgae';
export type ProductionStatus = 'inoculation' | 'fermentation' | 'harvest' | 'processing' | 'completed';

export interface Substrate {
  type: SubstrateType;
  source: string;
  carbonSource: string;
  nitrogenSource: string;
  cost: number; // USD per ton
  sustainability: number; // 0-100 score
}

export type SubstrateType = 'methane' | 'methanol' | 'glucose' | 'cellulose' | 'co2' | 'industrial_waste' | 'agricultural_waste';

export interface Bioreactor {
  reactorId: string;
  type: ReactorType;
  volume: number; // liters
  workingVolume: number; // liters
  temperature: number; // Celsius
  pH: number;
  dissolvedOxygen: number; // percentage
  agitationSpeed: number; // RPM
  aeration: number; // vvm
  pressure: number; // bar
}

export type ReactorType = 'stirred_tank' | 'airlift' | 'bubble_column' | 'packed_bed' | 'fluidized_bed';

// ============================================================================
// Process Monitoring
// ============================================================================

export interface ProcessMetrics {
  timestamp: number;
  productionId: string;
  biomassDensity: number; // g/L
  specificGrowthRate: number; // per hour
  yieldCoefficient: number; // g biomass/g substrate
  productivityRate: number; // g/L/h
  oxygenUptakeRate: number; // mmol/L/h
  carbonDioxideEvolutionRate: number; // mmol/L/h
  respiratoryQuotient: number;
  viability: number; // percentage
}

export interface FermentationParameters {
  temperature: number;
  pH: number;
  dissolvedOxygen: number;
  redoxPotential: number; // mV
  foamLevel: number; // percentage
  substrateConcentration: number; // g/L
  productConcentration: number; // g/L
  metaboliteConcentrations: Record<string, number>;
}

// ============================================================================
// Downstream Processing
// ============================================================================

export interface DownstreamProcess {
  processId: string;
  productionId: string;
  steps: ProcessingStep[];
  totalDuration: number; // hours
  recoveryRate: number; // percentage
  purity: number; // percentage
  finalProduct: FinalProduct;
}

export interface ProcessingStep {
  stepId: string;
  name: string;
  type: ProcessingType;
  duration: number; // hours
  efficiency: number; // percentage
  energyConsumption: number; // kWh
  waterUsage: number; // liters
  wasteGenerated: number; // kg
}

export type ProcessingType = 'cell_disruption' | 'separation' | 'purification' | 'drying' | 'formulation' | 'packaging';

export interface FinalProduct {
  productId: string;
  name: string;
  form: ProductForm;
  proteinContent: number; // percentage
  aminoAcidProfile: AminoAcidProfile;
  nutritionalValue: NutritionalProfile;
  yield: number; // kg
  quality: QualityMetrics;
}

export type ProductForm = 'powder' | 'granules' | 'flakes' | 'concentrate' | 'isolate';

// ============================================================================
// Nutrition & Quality
// ============================================================================

export interface AminoAcidProfile {
  essentialAminoAcids: {
    leucine: number;
    isoleucine: number;
    valine: number;
    lysine: number;
    methionine: number;
    phenylalanine: number;
    threonine: number;
    tryptophan: number;
    histidine: number;
  };
  nonEssentialAminoAcids: Record<string, number>;
  totalProtein: number; // g per 100g
  digestibility: number; // percentage
  biologicalValue: number; // 0-100
}

export interface NutritionalProfile {
  protein: number; // g per 100g
  fat: number;
  carbohydrates: number;
  fiber: number;
  ash: number;
  moisture: number;
  vitamins: Record<string, number>;
  minerals: Record<string, number>;
  energyKcal: number;
}

export interface QualityMetrics {
  microbiologicalSafety: MicrobiologicalTests;
  heavyMetals: Record<string, number>; // ppm
  toxins: Record<string, number>; // ppb
  allergens: string[];
  gmoStatus: boolean;
  certifications: string[];
  shelfLife: number; // days
  storageConditions: string;
}

export interface MicrobiologicalTests {
  totalPlateCount: number; // CFU/g
  yeastMold: number; // CFU/g
  coliforms: number; // CFU/g
  salmonella: boolean;
  listeria: boolean;
  staphylococcus: number; // CFU/g
}

// ============================================================================
// Applications
// ============================================================================

export interface SCPApplication {
  applicationId: string;
  name: string;
  sector: ApplicationSector;
  productForm: ProductForm;
  targetMarket: string;
  specifications: ProductSpecifications;
  pricing: PricingInfo;
  regulations: RegulatoryInfo[];
}

export type ApplicationSector = 'animal_feed' | 'aquaculture' | 'human_nutrition' | 'pet_food' | 'functional_foods' | 'medical_nutrition';

export interface ProductSpecifications {
  minProteinContent: number;
  maxMoisture: number;
  particleSize: string;
  bulkDensity: number; // kg/m³
  solubility: string;
  color: string;
  odor: string;
}

export interface PricingInfo {
  pricePerKg: number; // USD
  currency: string;
  minimumOrder: number; // kg
  bulkDiscounts: { quantity: number; discount: number }[];
}

export interface RegulatoryInfo {
  region: string;
  status: 'approved' | 'pending' | 'not_approved';
  approvalDate?: string;
  restrictions?: string[];
  labelingRequirements: string[];
}

// ============================================================================
// Sustainability
// ============================================================================

export interface LifeCycleAssessment {
  productionId: string;
  functionalUnit: string;
  systemBoundary: string;
  impacts: EnvironmentalImpacts;
  comparisons: ComparisonData[];
}

export interface EnvironmentalImpacts {
  globalWarmingPotential: number; // kg CO2e
  waterConsumption: number; // liters
  landUse: number; // m²
  acidificationPotential: number; // kg SO2e
  eutrophicationPotential: number; // kg PO4e
  energyConsumption: number; // MJ
  renewableEnergyPercent: number;
}

export interface ComparisonData {
  product: string;
  category: string;
  metric: string;
  scpValue: number;
  comparisonValue: number;
  unit: string;
  difference: number; // percentage
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
