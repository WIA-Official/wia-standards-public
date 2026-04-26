/**
 * WIA-AGRI-029 Food Security Standard - TypeScript Type Definitions
 * @module @wia/food-security/types
 */

// ============================================================================
// Food Security Assessment Types
// ============================================================================

export interface FoodSecurityAssessment {
  assessmentId: string;
  region: Region;
  timestamp: number;
  overallScore: number; // 0-100
  securityLevel: SecurityLevel;
  indicators: SecurityIndicators;
  population: PopulationData;
  recommendations: string[];
}

export type SecurityLevel = 'secure' | 'moderately_secure' | 'moderately_insecure' | 'severely_insecure' | 'famine';

export interface Region {
  regionId: string;
  name: string;
  country: string;
  coordinates: {
    latitude: number;
    longitude: number;
  };
  area: number; // km²
  population: number;
  urbanRural: 'urban' | 'rural' | 'mixed';
}

// ============================================================================
// Security Indicators
// ============================================================================

export interface SecurityIndicators {
  availability: AvailabilityIndicators;
  access: AccessIndicators;
  utilization: UtilizationIndicators;
  stability: StabilityIndicators;
}

export interface AvailabilityIndicators {
  localProduction: number; // tons/year
  imports: number; // tons/year
  exports: number; // tons/year
  stocks: number; // tons
  productionPerCapita: number; // kg/person/year
  cropDiversityIndex: number; // 0-1
}

export interface AccessIndicators {
  averageIncome: number; // USD/month
  foodPriceIndex: number; // baseline=100
  marketAccessScore: number; // 0-100
  infrastructureScore: number; // 0-100
  affordabilityRatio: number; // food cost / income
  socialSafetyNetCoverage: number; // percentage
}

export interface UtilizationIndicators {
  nutritionAdequacy: number; // percentage
  dietDiversity: number; // 0-1
  waterQualityIndex: number; // 0-100
  sanitationAccess: number; // percentage
  healthServiceAccess: number; // percentage
  childMalnutritionRate: number; // percentage
}

export interface StabilityIndicators {
  priceVolatility: number; // coefficient of variation
  productionVariability: number; // coefficient of variation
  climateVulnerability: number; // 0-1
  politicalStability: number; // 0-100
  disasterRiskIndex: number; // 0-1
}

// ============================================================================
// Population & Demographics
// ============================================================================

export interface PopulationData {
  total: number;
  underFive: number;
  children: number; // 5-18 years
  adults: number; // 18-65 years
  elderly: number; // 65+ years
  vulnerableGroups: VulnerableGroups;
}

export interface VulnerableGroups {
  pregnantWomen: number;
  lactatingMothers: number;
  disabled: number;
  homeless: number;
  refugees: number;
  internationallyDisplaced: number;
}

// ============================================================================
// Food Supply Chain Types
// ============================================================================

export interface SupplyChain {
  chainId: string;
  name: string;
  commodities: string[];
  stages: SupplyChainStage[];
  efficiency: number; // 0-100
  lossRate: number; // percentage
  carbonFootprint: number; // kg CO2e/kg food
}

export interface SupplyChainStage {
  stageId: string;
  name: string;
  type: 'production' | 'processing' | 'storage' | 'transportation' | 'distribution' | 'retail';
  location: Region;
  capacity: number;
  utilization: number; // percentage
  lossRate: number; // percentage
  quality: QualityMetrics;
}

export interface QualityMetrics {
  grade: 'A' | 'B' | 'C' | 'D' | 'rejected';
  freshness: number; // 0-100
  contamination: boolean;
  certifications: string[];
  shelfLife: number; // days remaining
}

// ============================================================================
// Food Stock & Inventory
// ============================================================================

export interface FoodStock {
  stockId: string;
  facility: StorageFacility;
  commodity: FoodCommodity;
  quantity: number; // kg or liters
  quality: QualityMetrics;
  expiryDate?: string;
  value: number; // USD
  lastUpdated: number;
}

export interface StorageFacility {
  facilityId: string;
  name: string;
  location: Region;
  type: 'warehouse' | 'silo' | 'cold_storage' | 'distribution_center';
  capacity: number; // cubic meters
  currentUtilization: number; // percentage
  temperature?: number; // Celsius
  humidity?: number; // percentage
  conditions: 'excellent' | 'good' | 'fair' | 'poor';
}

export interface FoodCommodity {
  commodityId: string;
  name: string;
  category: FoodCategory;
  unit: 'kg' | 'liters' | 'tons';
  caloriesPerKg: number;
  proteinPerKg: number; // grams
  shelfLife: number; // days
  storageRequirements: string[];
}

export type FoodCategory = 'cereals' | 'pulses' | 'vegetables' | 'fruits' | 'meat' | 'dairy' | 'oils' | 'other';

// ============================================================================
// Market & Price Data
// ============================================================================

export interface MarketPrice {
  priceId: string;
  commodity: string;
  market: Market;
  price: number; // USD per kg
  currency: string;
  timestamp: number;
  priceChange: number; // percentage
  availability: 'abundant' | 'adequate' | 'limited' | 'scarce';
}

export interface Market {
  marketId: string;
  name: string;
  location: Region;
  type: 'wholesale' | 'retail' | 'farmers_market' | 'commodity_exchange';
  operatingDays: string[];
  commoditiesTraded: string[];
}

export interface PriceAnalysis {
  commodity: string;
  period: { start: number; end: number };
  averagePrice: number;
  minPrice: number;
  maxPrice: number;
  volatility: number;
  trend: 'increasing' | 'stable' | 'decreasing';
  seasonalityDetected: boolean;
}

// ============================================================================
// Nutrition & Dietary Data
// ============================================================================

export interface NutritionalRequirement {
  populationGroup: string;
  dailyCalories: number; // kcal
  protein: number; // grams
  fat: number; // grams
  carbohydrates: number; // grams
  vitamins: Record<string, number>;
  minerals: Record<string, number>;
}

export interface DietaryIntake {
  personId?: string;
  householdId?: string;
  date: string;
  calories: number;
  protein: number;
  foodGroups: Record<FoodCategory, number>; // servings
  adequacy: number; // percentage of requirements met
  diversityScore: number; // 0-1
}

export interface FoodAssistance {
  programId: string;
  name: string;
  type: 'food_distribution' | 'cash_transfer' | 'vouchers' | 'school_feeding' | 'emergency_relief';
  region: Region;
  beneficiaries: number;
  rationSize: number; // calories per person per day
  frequency: 'daily' | 'weekly' | 'monthly';
  startDate: string;
  endDate?: string;
  budget: number; // USD
  funders: string[];
}

// ============================================================================
// Early Warning & Monitoring
// ============================================================================

export interface EarlyWarningAlert {
  alertId: string;
  timestamp: number;
  region: Region;
  severity: 'watch' | 'warning' | 'emergency' | 'catastrophic';
  type: 'drought' | 'flood' | 'pest' | 'disease' | 'conflict' | 'economic' | 'pandemic';
  affectedPopulation: number;
  foodGap: number; // tons
  timeframe: string;
  actionRequired: string[];
  status: 'active' | 'resolved' | 'escalated';
}

export interface MonitoringIndicator {
  indicatorId: string;
  name: string;
  category: string;
  value: number;
  threshold: { warning: number; critical: number };
  status: 'normal' | 'warning' | 'critical';
  trend: 'improving' | 'stable' | 'deteriorating';
  lastUpdated: number;
}

// ============================================================================
// Intervention & Response
// ============================================================================

export interface Intervention {
  interventionId: string;
  name: string;
  type: 'agricultural' | 'humanitarian' | 'economic' | 'infrastructure' | 'policy';
  region: Region;
  targetPopulation: number;
  objectives: string[];
  activities: Activity[];
  budget: number; // USD
  startDate: string;
  endDate: string;
  status: 'planned' | 'ongoing' | 'completed' | 'suspended';
  impact: InterventionImpact;
}

export interface Activity {
  activityId: string;
  name: string;
  description: string;
  responsible: string;
  startDate: string;
  endDate: string;
  status: 'pending' | 'in_progress' | 'completed';
  budget: number;
  outputs: string[];
}

export interface InterventionImpact {
  beneficiariesReached: number;
  foodProvided: number; // tons
  livelihoodsSupported: number;
  malnutritionReduced: number; // percentage
  incomeIncreased: number; // percentage
  costEffectiveness: number; // USD per beneficiary
}

// ============================================================================
// Climate & Environmental Factors
// ============================================================================

export interface ClimateData {
  regionId: string;
  timestamp: number;
  rainfall: number; // mm
  temperature: number; // Celsius
  humidity: number; // percentage
  soilMoisture: number; // percentage
  vegetationIndex: number; // NDVI 0-1
  droughtIndex: number; // 0-1
  floodRisk: number; // 0-1
}

export interface SeasonalForecast {
  region: Region;
  season: string;
  year: number;
  rainfallForecast: 'below_normal' | 'normal' | 'above_normal';
  temperatureForecast: 'below_normal' | 'normal' | 'above_normal';
  cropProductionForecast: number; // tons
  confidence: number; // 0-1
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
  error?: {
    code: string;
    message: string;
    details?: any;
  };
  timestamp: number;
}

export interface PaginatedResponse<T> {
  data: T[];
  total: number;
  page: number;
  pageSize: number;
  hasMore: boolean;
}

// ============================================================================
// Export all types
// ============================================================================

export * from './types';
