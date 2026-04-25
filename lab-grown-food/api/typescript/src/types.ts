/**
 * WIA-AGRI-027 Lab-Grown Food Standard - TypeScript Type Definitions
 * @version 1.0.0
 * @standard WIA-AGRI-027
 */

// ============================================================================
// Core Types
// ============================================================================

export type ProductType = 'meat' | 'seafood' | 'dairy' | 'egg' | 'fat' | 'other';
export type ProductionStage = 'cell-harvesting' | 'cell-culture' | 'tissue-engineering' | 'maturation' | 'processing' | 'packaging';
export type CellType = 'stem-cell' | 'myocyte' | 'adipocyte' | 'fibroblast' | 'other';
export type QualityStatus = 'excellent' | 'good' | 'acceptable' | 'below-standard' | 'failed';
export type CertificationStatus = 'certified' | 'pending' | 'expired' | 'revoked';
export type SafetyLevel = 'safe' | 'warning' | 'unsafe';

// ============================================================================
// Product Profile
// ============================================================================

export interface LabGrownProduct {
  productId: string;
  name: string;
  productType: ProductType;
  cellLine: CellLineInfo;
  productionBatch: string;
  manufacturer: ManufacturerInfo;
  productionDate: string; // ISO 8601 date
  expiryDate: string; // ISO 8601 date
  nutritionalInfo: NutritionalInfo;
  qualityMetrics: QualityMetrics;
  certifications: Certification[];
  traceability: TraceabilityRecord;
  environmentalImpact: EnvironmentalImpact;
  createdAt: string; // ISO 8601 timestamp
  updatedAt: string; // ISO 8601 timestamp
  version: string;
  standard: string;
}

export interface CellLineInfo {
  cellLineId: string;
  cellType: CellType;
  sourceOrganism: string;
  harvestDate: string;
  geneticModifications?: GeneticModification[];
  passageNumber: number;
  viability: number; // percentage
  validated: boolean;
}

export interface GeneticModification {
  modificationId: string;
  type: string;
  purpose: string;
  approvalStatus: string;
  regulatoryApproval?: string[];
}

export interface ManufacturerInfo {
  manufacturerId: string;
  name: string;
  location: LocationInfo;
  contactEmail: string;
  contactPhone: string;
  facilityLicense: string;
  certifications: string[];
}

export interface LocationInfo {
  address: string;
  city: string;
  state?: string;
  country: string;
  postalCode: string;
  coordinates?: GeoCoordinates;
}

export interface GeoCoordinates {
  latitude: number;
  longitude: number;
}

// ============================================================================
// Nutritional Information
// ============================================================================

export interface NutritionalInfo {
  servingSize: Measurement;
  calories: number;
  macronutrients: Macronutrients;
  micronutrients: Micronutrient[];
  vitamins: Vitamin[];
  minerals: Mineral[];
  allergens?: string[];
  additives?: Additive[];
}

export interface Measurement {
  value: number;
  unit: string;
}

export interface Macronutrients {
  protein: Measurement;
  fat: Measurement;
  carbohydrates: Measurement;
  fiber?: Measurement;
  saturatedFat?: Measurement;
  transFat?: Measurement;
  cholesterol?: Measurement;
}

export interface Micronutrient {
  name: string;
  amount: Measurement;
  dailyValuePercentage?: number;
}

export interface Vitamin {
  name: string;
  amount: Measurement;
  dailyValuePercentage?: number;
}

export interface Mineral {
  name: string;
  amount: Measurement;
  dailyValuePercentage?: number;
}

export interface Additive {
  name: string;
  purpose: string;
  amount: Measurement;
  eNumber?: string;
  approved: boolean;
}

// ============================================================================
// Quality Metrics
// ============================================================================

export interface QualityMetrics {
  overallQuality: QualityStatus;
  cellViability: number; // percentage
  proteinContent: Measurement;
  fatContent: Measurement;
  moistureContent: Measurement;
  texture: TextureMetrics;
  flavor: FlavorProfile;
  color: ColorMetrics;
  microbiologicalSafety: MicrobiologicalSafety;
  chemicalSafety: ChemicalSafety;
  testedAt: string; // ISO 8601 timestamp
  laboratory: string;
}

export interface TextureMetrics {
  firmness: number; // 1-10
  tenderness: number; // 1-10
  juiciness: number; // 1-10
  chewiness: number; // 1-10
  assessment: string;
}

export interface FlavorProfile {
  sweetness: number; // 1-10
  saltiness: number; // 1-10
  umami: number; // 1-10
  bitterness: number; // 1-10
  overall: string;
  panelScore?: number;
}

export interface ColorMetrics {
  lValue: number; // lightness
  aValue: number; // red-green
  bValue: number; // yellow-blue
  visualAssessment: string;
}

export interface MicrobiologicalSafety {
  totalPlateCount: number;
  coliforms: number;
  salmonella: boolean;
  listeria: boolean;
  ecoli: boolean;
  yeastMold: number;
  safetyLevel: SafetyLevel;
}

export interface ChemicalSafety {
  antibiotics: AntibioticTest[];
  hormones: HormoneTest[];
  heavyMetals: HeavyMetalTest[];
  pesticides: PesticideTest[];
  safetyLevel: SafetyLevel;
}

export interface AntibioticTest {
  antibiotic: string;
  detected: boolean;
  level?: number;
  unit?: string;
  limit: number;
}

export interface HormoneTest {
  hormone: string;
  detected: boolean;
  level?: number;
  unit?: string;
  limit: number;
}

export interface HeavyMetalTest {
  metal: string;
  level: number;
  unit: string;
  limit: number;
  withinLimit: boolean;
}

export interface PesticideTest {
  pesticide: string;
  detected: boolean;
  level?: number;
  unit?: string;
  limit: number;
}

// ============================================================================
// Production Data
// ============================================================================

export interface ProductionRecord {
  recordId: string;
  productId: string;
  batchNumber: string;
  stage: ProductionStage;
  startTime: string; // ISO 8601 timestamp
  endTime: string; // ISO 8601 timestamp
  bioreactor: BioreactorInfo;
  cultureMedia: CultureMediaInfo;
  growthParameters: GrowthParameters;
  yield: ProductionYield;
  energyUsage: EnergyUsage;
  waterUsage: WaterUsage;
  notes?: string;
}

export interface BioreactorInfo {
  bioreactorId: string;
  type: string;
  capacity: Measurement;
  temperature: Measurement;
  pH: number;
  oxygenLevel: number; // percentage
  agitationSpeed?: number; // RPM
}

export interface CultureMediaInfo {
  mediaType: string;
  composition: MediaComponent[];
  volume: Measurement;
  cost: number;
  currency: string;
}

export interface MediaComponent {
  name: string;
  concentration: Measurement;
  purpose: string;
}

export interface GrowthParameters {
  cellDensity: Measurement;
  doublingTime: Measurement;
  viability: number; // percentage
  metabolicRate: number;
  CO2Production: Measurement;
  lactateAccumulation?: Measurement;
}

export interface ProductionYield {
  biomass: Measurement;
  protein: Measurement;
  efficiency: number; // percentage
  targetMet: boolean;
}

export interface EnergyUsage {
  total: Measurement;
  heating: Measurement;
  cooling: Measurement;
  agitation: Measurement;
  lighting?: Measurement;
  cost: number;
  currency: string;
}

export interface WaterUsage {
  total: Measurement;
  fresh: Measurement;
  recycled?: Measurement;
  discharged?: Measurement;
  cost: number;
  currency: string;
}

// ============================================================================
// Certification & Compliance
// ============================================================================

export interface Certification {
  certificationId: string;
  type: string;
  issuingBody: string;
  issueDate: string; // ISO 8601 date
  expiryDate: string; // ISO 8601 date
  status: CertificationStatus;
  scope: string[];
  documentUrl?: string;
}

export interface TraceabilityRecord {
  traceabilityId: string;
  blockchain?: BlockchainInfo;
  sourceCell: CellSource;
  productionHistory: ProductionRecord[];
  distributionChain: DistributionRecord[];
  qualityChecks: QualityCheckRecord[];
}

export interface BlockchainInfo {
  network: string;
  contractAddress: string;
  tokenId?: string;
  transactionHash: string;
}

export interface CellSource {
  donorOrganism: string;
  donorAge?: string;
  harvestLocation: LocationInfo;
  harvestDate: string;
  harvestMethod: string;
  ethicalApproval: boolean;
}

export interface DistributionRecord {
  recordId: string;
  from: string;
  to: string;
  timestamp: string;
  temperature: Measurement;
  location: LocationInfo;
  status: string;
}

export interface QualityCheckRecord {
  checkId: string;
  timestamp: string;
  inspector: string;
  checkType: string;
  result: QualityStatus;
  notes?: string;
}

// ============================================================================
// Environmental Impact
// ============================================================================

export interface EnvironmentalImpact {
  carbonFootprint: Measurement;
  waterFootprint: Measurement;
  landUse: Measurement;
  comparisonToConventional: ComparisonMetrics;
  sustainability: SustainabilityMetrics;
}

export interface ComparisonMetrics {
  carbonReduction: number; // percentage
  waterReduction: number; // percentage
  landReduction: number; // percentage
  animalWelfareScore: number; // 1-100
}

export interface SustainabilityMetrics {
  renewableEnergy: number; // percentage
  wasteRecycling: number; // percentage
  byproductUtilization: number; // percentage
  overallScore: number; // 1-100
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

export class WIALabGrownFoodError extends Error {
  constructor(
    message: string,
    public code: string,
    public statusCode?: number
  ) {
    super(message);
    this.name = 'WIALabGrownFoodError';
  }
}
