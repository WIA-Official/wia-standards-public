/**
 * WIA-ENE-007: Hydrogen Energy Standard - TypeScript SDK
 * Type Definitions
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 * © 2025 SmileStory Inc. / WIA
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Hydrogen purity grade classification
 */
export enum HydrogenGrade {
  COMMERCIAL = 'COMMERCIAL',           // 98.0-99.0% purity
  HIGH_PURITY = 'HIGH_PURITY',         // 99.95-99.995% purity
  ULTRA_HIGH_PURITY = 'ULTRA_HIGH_PURITY', // 99.999%+ purity
  RESEARCH_GRADE = 'RESEARCH_GRADE'    // 99.9999% purity
}

/**
 * Hydrogen color classification based on production method
 */
export enum HydrogenColor {
  GREEN = 'GREEN',       // Renewable energy electrolysis
  BLUE = 'BLUE',         // Natural gas with CCS
  GRAY = 'GRAY',         // Natural gas without CCS
  TURQUOISE = 'TURQUOISE', // Methane pyrolysis
  PINK = 'PINK',         // Nuclear-powered electrolysis
  YELLOW = 'YELLOW',     // Solar-powered electrolysis
  WHITE = 'WHITE'        // Natural geological hydrogen
}

/**
 * Production technology types
 */
export enum ProductionTechnology {
  ALKALINE_ELECTROLYSIS = 'ALKALINE_ELECTROLYSIS',
  PEM_ELECTROLYSIS = 'PEM_ELECTROLYSIS',
  SOEC_ELECTROLYSIS = 'SOEC_ELECTROLYSIS',
  AEM_ELECTROLYSIS = 'AEM_ELECTROLYSIS',
  STEAM_METHANE_REFORMING = 'STEAM_METHANE_REFORMING',
  BIOMASS_GASIFICATION = 'BIOMASS_GASIFICATION',
  PHOTOELECTROCHEMICAL = 'PHOTOELECTROCHEMICAL'
}

// ============================================================================
// Production System Types
// ============================================================================

/**
 * Electrolyzer configuration and status
 */
export interface Electrolyzer {
  id: string;
  technology: ProductionTechnology;
  ratedCapacity: number; // kg H₂/day
  ratedPower: number; // kW
  efficiency: number; // percentage (HHV basis)
  operatingHours: number;
  status: 'OPERATIONAL' | 'MAINTENANCE' | 'OFFLINE' | 'STARTING' | 'STOPPING';
  currentLoad: number; // percentage of rated capacity
  stackTemperature: number; // °C
  stackVoltage: number; // V
  stackCurrent: number; // A
  productionRate: number; // kg H₂/hour current
}

/**
 * Production system performance metrics
 */
export interface ProductionMetrics {
  hydrogenProduced: number; // kg
  energyConsumed: number; // kWh
  waterConsumed: number; // liters
  oxygenProduced: number; // kg
  efficiency: number; // percentage
  purity: number; // percentage
  carbonIntensity: number; // kg CO₂/kg H₂
  timestamp: Date;
}

// ============================================================================
// Storage System Types
// ============================================================================

/**
 * Storage technology types
 */
export enum StorageTechnology {
  COMPRESSED_GAS = 'COMPRESSED_GAS',
  LIQUID_HYDROGEN = 'LIQUID_HYDROGEN',
  METAL_HYDRIDE = 'METAL_HYDRIDE',
  LOHC = 'LOHC', // Liquid Organic Hydrogen Carriers
  UNDERGROUND = 'UNDERGROUND'
}

/**
 * Storage vessel configuration and status
 */
export interface StorageVessel {
  id: string;
  technology: StorageTechnology;
  capacity: number; // kg H₂
  currentInventory: number; // kg H₂
  pressure: number; // bar
  temperature: number; // °C
  fillLevel: number; // percentage
  status: 'AVAILABLE' | 'FILLING' | 'DISPENSING' | 'MAINTENANCE' | 'OFFLINE';
  lastInspectionDate: Date;
  nextInspectionDate: Date;
}

/**
 * Storage system aggregate metrics
 */
export interface StorageMetrics {
  totalCapacity: number; // kg H₂
  totalInventory: number; // kg H₂
  utilizationRate: number; // percentage
  boilOffRate: number; // percentage per day (for LH₂)
  vesselCount: number;
  timestamp: Date;
}

// ============================================================================
// Distribution System Types
// ============================================================================

/**
 * Distribution methods
 */
export enum DistributionMethod {
  PIPELINE = 'PIPELINE',
  TUBE_TRAILER = 'TUBE_TRAILER',
  LIQUID_TRUCK = 'LIQUID_TRUCK',
  RAIL = 'RAIL'
}

/**
 * Refueling station configuration
 */
export interface RefuelingStation {
  id: string;
  name: string;
  location: GeoLocation;
  pressureLevels: number[]; // bar (e.g., [350, 700])
  dailyCapacity: number; // kg H₂/day
  currentInventory: number; // kg H₂
  dispensingRate: number; // kg/hour
  status: 'OPERATIONAL' | 'MAINTENANCE' | 'OFFLINE';
  totalDispensed: number; // kg H₂ (lifetime)
}

/**
 * Geographic location
 */
export interface GeoLocation {
  latitude: number;
  longitude: number;
  address?: string;
  city?: string;
  country?: string;
}

// ============================================================================
// Quality and Compliance Types
// ============================================================================

/**
 * Hydrogen quality parameters per ISO 14687
 */
export interface HydrogenQuality {
  totalNonHydrogen: number; // mol/mol (max 2%)
  waterContent: number; // μmol/mol (max 5)
  totalHydrocarbons: number; // μmol/mol (max 2)
  oxygen: number; // μmol/mol (max 5)
  carbonMonoxide: number; // μmol/mol (max 0.2)
  carbonDioxide: number; // μmol/mol (max 2)
  totalSulfur: number; // μmol/mol (max 0.004)
  ammonia: number; // μmol/mol (max 0.1)
  compliantISO14687: boolean;
  sampleDate: Date;
  certificateNumber?: string;
}

/**
 * Green hydrogen certification
 */
export interface GreenCertification {
  certified: boolean;
  certificationBody: string;
  certificateNumber: string;
  issueDate: Date;
  expiryDate: Date;
  carbonIntensity: number; // kg CO₂/kg H₂
  renewableEnergyPercentage: number; // percentage
  productionMethod: ProductionTechnology;
}

// ============================================================================
// Safety and Monitoring Types
// ============================================================================

/**
 * Safety sensor reading
 */
export interface SafetySensor {
  id: string;
  type: 'HYDROGEN_DETECTOR' | 'PRESSURE' | 'TEMPERATURE' | 'FLAME' | 'SMOKE';
  location: string;
  value: number;
  unit: string;
  threshold: number;
  status: 'NORMAL' | 'WARNING' | 'ALARM' | 'FAULT';
  lastCalibration: Date;
  nextCalibration: Date;
}

/**
 * Safety incident record
 */
export interface SafetyIncident {
  id: string;
  date: Date;
  severity: 'MINOR' | 'MODERATE' | 'MAJOR' | 'CRITICAL';
  type: 'LEAK' | 'FIRE' | 'EXPLOSION' | 'EXPOSURE' | 'EQUIPMENT_FAILURE' | 'OTHER';
  description: string;
  rootCause?: string;
  correctiveActions: string[];
  status: 'OPEN' | 'INVESTIGATING' | 'RESOLVED' | 'CLOSED';
}

// ============================================================================
// Economic and Performance Types
// ============================================================================

/**
 * Cost breakdown structure
 */
export interface CostMetrics {
  electricityCost: number; // $ per kg H₂
  waterCost: number; // $ per kg H₂
  maintenanceCost: number; // $ per kg H₂
  laborCost: number; // $ per kg H₂
  otherCosts: number; // $ per kg H₂
  totalCost: number; // $ per kg H₂ (LCOH)
  period: string; // e.g., "2025-01"
}

/**
 * Key Performance Indicators
 */
export interface PerformanceKPIs {
  availability: number; // percentage
  capacityFactor: number; // percentage
  specificEnergyConsumption: number; // kWh per kg H₂
  purityComplianceRate: number; // percentage
  unplannedDowntime: number; // hours
  safetyIncidentRate: number; // TRIR
  period: string;
}

// ============================================================================
// API Request/Response Types
// ============================================================================

/**
 * Generic API response wrapper
 */
export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
    details?: any;
  };
  timestamp: Date;
}

/**
 * Paginated list response
 */
export interface PaginatedResponse<T> {
  items: T[];
  total: number;
  page: number;
  pageSize: number;
  hasMore: boolean;
}

/**
 * Time series data point
 */
export interface TimeSeriesDataPoint {
  timestamp: Date;
  value: number;
  unit?: string;
}

/**
 * Query parameters for time series data
 */
export interface TimeSeriesQuery {
  startDate: Date;
  endDate: Date;
  interval?: 'MINUTE' | 'HOUR' | 'DAY' | 'WEEK' | 'MONTH';
  aggregation?: 'AVG' | 'SUM' | 'MIN' | 'MAX' | 'COUNT';
}

// ============================================================================
// System Configuration Types
// ============================================================================

/**
 * Facility configuration
 */
export interface FacilityConfig {
  facilityId: string;
  name: string;
  location: GeoLocation;
  commissioningDate: Date;
  totalCapacity: number; // kg H₂/day
  electrolyzers: Electrolyzer[];
  storage: StorageVessel[];
  refuelingStations?: RefuelingStation[];
  primaryUse: 'INDUSTRIAL' | 'MOBILITY' | 'POWER_GENERATION' | 'MIXED';
}

/**
 * SDK configuration options
 */
export interface SDKConfig {
  apiKey: string;
  baseUrl?: string;
  timeout?: number; // milliseconds
  retryAttempts?: number;
  logLevel?: 'DEBUG' | 'INFO' | 'WARN' | 'ERROR';
}

// ============================================================================
// Export all types
// ============================================================================

export type {
  Electrolyzer,
  ProductionMetrics,
  StorageVessel,
  StorageMetrics,
  RefuelingStation,
  GeoLocation,
  HydrogenQuality,
  GreenCertification,
  SafetySensor,
  SafetyIncident,
  CostMetrics,
  PerformanceKPIs,
  ApiResponse,
  PaginatedResponse,
  TimeSeriesDataPoint,
  TimeSeriesQuery,
  FacilityConfig,
  SDKConfig
};
