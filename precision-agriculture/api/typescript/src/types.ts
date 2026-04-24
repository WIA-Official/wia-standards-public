/**
 * WIA-AGRI-002 Precision Agriculture Standard - TypeScript Type Definitions
 * Version: 1.0.0
 * Philosophy: 弘益人間 (Benefit All Humanity)
 */

/**
 * WIA Precision Agriculture Standard Version
 */
export type WIAVersion = '1.0' | '1.1' | '2.0';

/**
 * Field Management Types
 */
export type FieldType = 'crop' | 'pasture' | 'orchard' | 'vineyard' | 'fallow';

/**
 * Crop Types
 */
export type CropType =
  | 'corn'
  | 'wheat'
  | 'rice'
  | 'soybean'
  | 'potato'
  | 'tomato'
  | 'cotton'
  | 'sugarcane'
  | 'other';

/**
 * Application Types
 */
export type ApplicationType =
  | 'fertilizer'
  | 'pesticide'
  | 'herbicide'
  | 'irrigation'
  | 'seeding';

/**
 * Sampling Method
 */
export type SamplingMethod = 'grid' | 'zone' | 'random' | 'management_zone';

/**
 * Data Quality
 */
export type DataQuality = 'high' | 'medium' | 'low';

/**
 * Prescription Map Status
 */
export type PrescriptionStatus = 'draft' | 'approved' | 'applied' | 'archived';

/**
 * Precision Agriculture Configuration
 */
export interface PrecisionAgricultureConfig {
  farmId: string;
  name: string;
  location: Location;
  totalArea: number; // hectares
  operator: Operator;
  metadata?: Record<string, any>;
}

/**
 * Geographic Location
 */
export interface Location {
  latitude: number;
  longitude: number;
  altitude?: number;
  address?: string;
  region?: string;
  country: string;
}

/**
 * Farm Operator
 */
export interface Operator {
  id: string;
  name: string;
  email?: string;
  phone?: string;
  certifications?: string[];
}

/**
 * Field Information
 */
export interface FieldData {
  fieldId: string;
  farmId: string;
  name: string;
  type: FieldType;
  area: number; // hectares
  boundary: GeoJSON;
  currentCrop?: CropType;
  soilType?: string;
  slope?: number; // percentage
  drainage?: string;
  metadata?: Record<string, any>;
}

/**
 * GeoJSON Geometry
 */
export interface GeoJSON {
  type: 'Polygon' | 'MultiPolygon';
  coordinates: number[][][] | number[][][][];
}

/**
 * Soil Sample
 */
export interface SoilSample {
  sampleId: string;
  fieldId: string;
  location: {
    latitude: number;
    longitude: number;
  };
  depth: number; // cm
  samplingDate: string;
  method: SamplingMethod;
  analysis: SoilAnalysis;
  metadata?: Record<string, any>;
}

/**
 * Soil Analysis Results
 */
export interface SoilAnalysis {
  ph?: number;
  organicMatter?: number; // percentage
  nitrogen?: number; // ppm
  phosphorus?: number; // ppm
  potassium?: number; // ppm
  calcium?: number; // ppm
  magnesium?: number; // ppm
  sulfur?: number; // ppm
  cec?: number; // Cation Exchange Capacity
  texture?: string;
  moisture?: number; // percentage
}

/**
 * Variable Rate Application (VRA) Map
 */
export interface VRAMap {
  mapId: string;
  fieldId: string;
  name: string;
  applicationType: ApplicationType;
  product?: string;
  createdDate: string;
  status: PrescriptionStatus;
  zones: ApplicationZone[];
  totalAmount?: number;
  unit: string;
  metadata?: Record<string, any>;
}

/**
 * Application Zone
 */
export interface ApplicationZone {
  zoneId: string;
  geometry: GeoJSON;
  rate: number;
  area: number; // hectares
  color?: string;
  notes?: string;
}

/**
 * Yield Data Point
 */
export interface YieldDataPoint {
  timestamp: string;
  location: {
    latitude: number;
    longitude: number;
  };
  yield: number; // kg or bushels
  moisture?: number; // percentage
  elevation?: number;
  speed?: number; // km/h
  heading?: number; // degrees
  swathWidth?: number; // meters
}

/**
 * Yield Map
 */
export interface YieldMap {
  mapId: string;
  fieldId: string;
  cropType: CropType;
  harvestDate: string;
  dataPoints: YieldDataPoint[];
  statistics: YieldStatistics;
  interpolated?: boolean;
  metadata?: Record<string, any>;
}

/**
 * Yield Statistics
 */
export interface YieldStatistics {
  averageYield: number;
  minYield: number;
  maxYield: number;
  totalYield: number;
  standardDeviation?: number;
  coefficientVariation?: number;
  unit: string;
}

/**
 * NDVI (Normalized Difference Vegetation Index) Data
 */
export interface NDVIData {
  dataId: string;
  fieldId: string;
  captureDate: string;
  source: string; // satellite, drone, etc.
  resolution: number; // meters
  cloudCover?: number; // percentage
  values: NDVIPoint[];
  statistics: NDVIStatistics;
  metadata?: Record<string, any>;
}

/**
 * NDVI Data Point
 */
export interface NDVIPoint {
  location: {
    latitude: number;
    longitude: number;
  };
  ndvi: number; // -1 to 1
  quality?: DataQuality;
}

/**
 * NDVI Statistics
 */
export interface NDVIStatistics {
  mean: number;
  min: number;
  max: number;
  stdDev: number;
  healthyArea?: number; // percentage
  stressedArea?: number; // percentage
}

/**
 * Equipment Data
 */
export interface EquipmentData {
  equipmentId: string;
  type: string;
  make?: string;
  model?: string;
  serialNumber?: string;
  capabilities: string[];
  lastMaintenance?: string;
  status: 'active' | 'maintenance' | 'retired';
  metadata?: Record<string, any>;
}

/**
 * Operation Record
 */
export interface OperationRecord {
  operationId: string;
  fieldId: string;
  operationType: ApplicationType | 'tillage' | 'harvest' | 'planting';
  date: string;
  equipmentId?: string;
  operator?: string;
  product?: string;
  rate?: number;
  unit?: string;
  area?: number;
  duration?: number; // minutes
  cost?: number;
  notes?: string;
  metadata?: Record<string, any>;
}

/**
 * Management Zone
 */
export interface ManagementZone {
  zoneId: string;
  fieldId: string;
  name: string;
  geometry: GeoJSON;
  area: number;
  classification: string;
  characteristics: ZoneCharacteristics;
  recommendations?: Record<string, any>;
  metadata?: Record<string, any>;
}

/**
 * Zone Characteristics
 */
export interface ZoneCharacteristics {
  soilType?: string;
  productivity?: 'high' | 'medium' | 'low';
  drainageClass?: string;
  slope?: number;
  elevation?: number;
  historicalYield?: number;
}

/**
 * Weather Data
 */
export interface WeatherData {
  timestamp: string;
  location: Location;
  temperature?: number; // Celsius
  humidity?: number; // percentage
  precipitation?: number; // mm
  windSpeed?: number; // m/s
  windDirection?: number; // degrees
  solarRadiation?: number; // W/m²
  evapotranspiration?: number; // mm
}

/**
 * Prescription Recommendation
 */
export interface PrescriptionRecommendation {
  recommendationId: string;
  fieldId: string;
  applicationType: ApplicationType;
  generatedDate: string;
  basedOn: string[]; // e.g., ['soil_samples', 'yield_maps', 'ndvi']
  recommendations: ApplicationZone[];
  confidence?: number; // 0-100
  notes?: string;
}

/**
 * Analytics Report
 */
export interface AnalyticsReport {
  reportId: string;
  farmId: string;
  fieldId?: string;
  reportType: string;
  period: string;
  generatedDate: string;
  metrics: Record<string, any>;
  insights: string[];
  recommendations?: string[];
}

/**
 * SDK Configuration
 */
export interface SDKConfig {
  baseURL?: string;
  apiKey?: string;
  farmId?: string;
  timeout?: number;
}

/**
 * List Query Parameters
 */
export interface ListParams {
  limit?: number;
  offset?: number;
  startDate?: string;
  endDate?: string;
  fieldId?: string;
  cropType?: string;
}

/**
 * Error Response
 */
export interface ErrorResponse {
  error: {
    code: string;
    message: string;
    details?: any;
  };
}
