/**
 * WIA-AGRI-030 Urban Farming Standard - TypeScript Type Definitions
 * @module @wia/urban-farming/types
 */

// ============================================================================
// Urban Farm Types
// ============================================================================

export interface UrbanFarm {
  farmId: string;
  name: string;
  type: FarmType;
  location: UrbanLocation;
  area: number; // square meters
  growingMethod: GrowingMethod[];
  crops: string[];
  status: 'active' | 'inactive' | 'seasonal';
  certification?: string[];
  ownershipType: 'private' | 'community' | 'municipal' | 'cooperative';
}

export type FarmType = 'rooftop' | 'vertical' | 'community_garden' | 'greenhouse' | 'indoor' | 'balcony' | 'backyard' | 'aquaponics';
export type GrowingMethod = 'soil' | 'hydroponics' | 'aeroponics' | 'aquaponics' | 'container';

export interface UrbanLocation {
  address: string;
  city: string;
  district: string;
  coordinates: { latitude: number; longitude: number };
  zoning: string;
  accessibility: 'public' | 'private' | 'restricted';
}

// ============================================================================
// Production & Yield
// ============================================================================

export interface Production {
  productionId: string;
  farmId: string;
  crop: string;
  variety: string;
  plantingDate: string;
  harvestDate?: string;
  areaUsed: number; // square meters
  yieldAmount: number; // kg
  yieldPerSqm: number; // kg/m²
  quality: 'premium' | 'standard' | 'below_standard';
  destination: 'sale' | 'donation' | 'personal_use' | 'seed_saving';
}

export interface CropSchedule {
  scheduleId: string;
  farmId: string;
  crop: string;
  plantingWindow: { start: string; end: string };
  expectedHarvest: string;
  seasonalRotation: string[];
  companionPlants?: string[];
}

// ============================================================================
// Community & Education
// ============================================================================

export interface CommunityProgram {
  programId: string;
  farmId: string;
  name: string;
  type: 'education' | 'volunteer' | 'workshop' | 'therapy' | 'social_enterprise';
  participants: number;
  schedule: string;
  objectives: string[];
  impact: ProgramImpact;
}

export interface ProgramImpact {
  peopleServed: number;
  hoursContributed: number;
  foodProduced: number; // kg
  foodDistributed: number; // kg
  educationSessions: number;
  communityEngagement: number; // 0-100 score
}

export interface Workshop {
  workshopId: string;
  title: string;
  date: string;
  duration: number; // hours
  instructor: string;
  topics: string[];
  maxParticipants: number;
  registered: number;
  location: string;
}

// ============================================================================
// Resource Management
// ============================================================================

export interface WaterManagement {
  farmId: string;
  source: WaterSource[];
  dailyUsage: number; // liters
  rainwaterHarvesting: boolean;
  greyWaterRecycling: boolean;
  irrigationSystem: IrrigationType[];
  waterEfficiency: number; // liters per kg produced
}

export type WaterSource = 'municipal' | 'well' | 'rainwater' | 'greywater' | 'recycled';
export type IrrigationType = 'drip' | 'sprinkler' | 'manual' | 'automated' | 'hydroponics';

export interface SoilHealth {
  farmId: string;
  timestamp: number;
  pH: number;
  organicMatter: number; // percentage
  nitrogen: number; // ppm
  phosphorus: number; // ppm
  potassium: number; // ppm
  microbialActivity: 'low' | 'medium' | 'high';
  compostUsage: number; // kg/month
}

export interface EnergyUsage {
  farmId: string;
  period: { start: string; end: string };
  consumption: number; // kWh
  solarGeneration?: number; // kWh
  gridUsage: number; // kWh
  carbonFootprint: number; // kg CO2e
  efficiency: number; // kWh per kg produced
}

// ============================================================================
// Market & Distribution
// ============================================================================

export interface MarketOutlet {
  outletId: string;
  name: string;
  type: 'farmers_market' | 'csa' | 'restaurant' | 'grocery_store' | 'food_bank' | 'direct_sales';
  location: string;
  distance: number; // km from farm
  frequency: string;
  avgSales: number; // USD per week
}

export interface CSAProgram {
  programId: string;
  farmId: string;
  name: string;
  members: number;
  shareSize: 'small' | 'medium' | 'large' | 'family';
  weeklyBoxValue: number; // USD
  season: { start: string; end: string };
  deliveryMethod: 'pickup' | 'delivery' | 'both';
  pickupLocations?: string[];
}

export interface Sale {
  saleId: string;
  farmId: string;
  date: string;
  items: SaleItem[];
  totalAmount: number;
  customer: string;
  paymentMethod: string;
}

export interface SaleItem {
  product: string;
  quantity: number;
  unit: string;
  pricePerUnit: number;
  total: number;
}

// ============================================================================
// Sustainability & Environmental Impact
// ============================================================================

export interface SustainabilityMetrics {
  farmId: string;
  period: { start: string; end: string };
  carbonSequestration: number; // kg CO2
  wasteReduction: number; // kg
  biodiversityScore: number; // 0-100
  localProcurement: number; // percentage
  organicPractices: boolean;
  pesticideUse: number; // kg
  syntheticFertilizerUse: number; // kg
}

export interface Biodiversity {
  farmId: string;
  plantSpecies: number;
  pollinatorFriendly: boolean;
  nativeSpecies: number;
  habitatProvided: string[];
  pestManagement: 'ipm' | 'organic' | 'conventional';
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
