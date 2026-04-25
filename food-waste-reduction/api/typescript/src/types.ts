/**
 * WIA-AGRI-032 Food Waste Reduction Standard - TypeScript Type Definitions
 * @module @wia/food-waste-reduction/types
 */

// ============================================================================
// Waste Tracking Types
// ============================================================================

export interface FoodWaste {
  wasteId: string;
  source: WasteSource;
  category: FoodCategory;
  items: WasteItem[];
  totalWeight: number; // kg
  timestamp: number;
  reason: WasteReason;
  preventable: boolean;
  disposalMethod: DisposalMethod;
  cost: number; // USD
}

export type WasteSource = 'farm' | 'processing' | 'distribution' | 'retail' | 'foodservice' | 'household';
export type FoodCategory = 'fruits' | 'vegetables' | 'grains' | 'dairy' | 'meat' | 'prepared_food' | 'bakery' | 'other';
export type WasteReason = 'spoilage' | 'overproduction' | 'quality_standards' | 'expired' | 'damaged' | 'oversupply' | 'cosmetic' | 'edible_trim';
export type DisposalMethod = 'landfill' | 'composting' | 'anaerobic_digestion' | 'animal_feed' | 'donation' | 'industrial_use';

export interface WasteItem {
  product: string;
  quantity: number;
  unit: string;
  expiryDate?: string;
  value: number; // USD
}

// ============================================================================
// Prevention Programs
// ============================================================================

export interface PreventionProgram {
  programId: string;
  name: string;
  type: 'donation' | 'surplus_marketplace' | 'composting' | 'upcycling' | 'education' | 'smart_packaging';
  organization: string;
  coverage: Coverage;
  status: 'active' | 'pilot' | 'planned' | 'inactive';
  metrics: ProgramMetrics;
}

export interface Coverage {
  regions: string[];
  sources: WasteSource[];
  population: number;
}

export interface ProgramMetrics {
  wasteReduced: number; // kg/month
  foodRecovered: number; // kg/month
  co2Saved: number; // kg CO2e
  costSavings: number; // USD
  participantsReached: number;
}

// ============================================================================
// Donation & Recovery
// ============================================================================

export interface FoodDonation {
  donationId: string;
  donor: Organization;
  recipient: Organization;
  items: DonationItem[];
  totalWeight: number; // kg
  pickupDate: string;
  deliveryDate: string;
  status: 'scheduled' | 'picked_up' | 'delivered' | 'cancelled';
  taxDeduction?: number; // USD
}

export interface DonationItem {
  product: string;
  quantity: number;
  unit: string;
  expiryDate: string;
  condition: 'excellent' | 'good' | 'fair';
  value: number;
}

export interface Organization {
  orgId: string;
  name: string;
  type: 'retailer' | 'restaurant' | 'food_bank' | 'shelter' | 'school' | 'nonprofit';
  address: string;
  contactPerson: string;
  phone: string;
  certification?: string[];
}

// ============================================================================
// Smart Storage & Inventory
// ============================================================================

export interface SmartStorage {
  storageId: string;
  location: string;
  capacity: number; // cubic meters
  inventory: InventoryItem[];
  temperature: number; // Celsius
  humidity: number; // percentage
  monitoringEnabled: boolean;
  alerts: StorageAlert[];
}

export interface InventoryItem {
  itemId: string;
  product: string;
  quantity: number;
  unit: string;
  receivedDate: string;
  expiryDate: string;
  daysToExpiry: number;
  location: string;
  condition: 'fresh' | 'aging' | 'near_expiry' | 'expired';
}

export interface StorageAlert {
  alertId: string;
  timestamp: number;
  type: 'near_expiry' | 'temperature' | 'humidity' | 'overstock';
  severity: 'info' | 'warning' | 'critical';
  message: string;
  itemsAffected: string[];
  actionRecommended: string;
}

// ============================================================================
// Waste Analytics
// ============================================================================

export interface WasteAnalytics {
  period: { start: string; end: string };
  totalWaste: number; // kg
  wasteByCategory: Record<FoodCategory, number>;
  wasteBySource: Record<WasteSource, number>;
  wasteByReason: Record<WasteReason, number>;
  preventableWaste: number; // kg
  wasteValue: number; // USD
  environmentalImpact: EnvironmentalImpact;
  trends: WasteTrend[];
}

export interface EnvironmentalImpact {
  co2Emissions: number; // kg CO2e
  waterWasted: number; // liters
  landUsed: number; // hectares
  energyWasted: number; // kWh
}

export interface WasteTrend {
  period: string;
  waste: number;
  changePercentage: number;
  forecast?: number;
}

// ============================================================================
// Upcycling & Valorization
// ============================================================================

export interface UpcyclingProject {
  projectId: string;
  name: string;
  inputMaterial: string;
  outputProduct: string;
  process: string;
  inputVolume: number; // kg/month
  outputVolume: number; // kg/month
  conversionRate: number; // percentage
  revenue: number; // USD/month
  status: 'active' | 'pilot' | 'planned';
}

export interface Compost {
  batchId: string;
  inputMaterials: CompostInput[];
  startDate: string;
  estimatedCompletion: string;
  volume: number; // cubic meters
  temperature: number; // Celsius
  moistureLevel: number; // percentage
  turnFrequency: string;
  quality: 'premium' | 'standard' | 'below_standard';
}

export interface CompostInput {
  material: string;
  weight: number; // kg
  carbonNitrogenRatio: number;
  type: 'green' | 'brown';
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
