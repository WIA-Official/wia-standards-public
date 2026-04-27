/**
 * WIA-AGRI-007 Yield Prediction Standard - TypeScript Type Definitions
 * Version: 1.0.0
 * Philosophy: 弘益人間 (Benefit All Humanity)
 */

/**
 * WIA Yield Prediction Standard Version
 */
export type WIAVersion = '1.0' | '1.1' | '2.0';

/**
 * Crop Types
 */
export type CropType =
  | 'corn'
  | 'wheat'
  | 'rice'
  | 'soybean'
  | 'potato'
  | 'cotton'
  | 'sugarcane'
  | 'vegetables'
  | 'fruits';

/**
 * Prediction Model Types
 */
export type ModelType =
  | 'statistical'
  | 'machine_learning'
  | 'deep_learning'
  | 'ensemble'
  | 'hybrid';

/**
 * Prediction Status
 */
export type PredictionStatus = 'pending' | 'processing' | 'completed' | 'failed';

/**
 * Yield Prediction Configuration
 */
export interface YieldPredictionConfig {
  predictionId: string;
  fieldId: string;
  cropType: CropType;
  variety?: string;
  plantingDate: string;
  expectedHarvestDate: string;
  area: number; // hectares
  model: PredictionModel;
  status: PredictionStatus;
  metadata?: Record<string, any>;
}

/**
 * Prediction Model
 */
export interface PredictionModel {
  modelId: string;
  name: string;
  type: ModelType;
  version: string;
  accuracy?: number; // percentage
  trainedOn?: string; // Date
  features: string[];
  algorithm?: string;
}

/**
 * Yield Prediction Result
 */
export interface YieldPredictionResult {
  predictionId: string;
  fieldId: string;
  cropType: CropType;
  predictionDate: string;
  harvestDate: string;
  predictedYield: number;
  unit: string; // e.g., 'tons/ha', 'bushels/acre'
  confidence: number; // 0-100
  confidenceInterval?: {
    lower: number;
    upper: number;
  };
  factors: YieldFactor[];
  spatialPrediction?: SpatialYieldData;
  modelUsed: string;
}

/**
 * Yield Factor
 */
export interface YieldFactor {
  name: string;
  category: 'weather' | 'soil' | 'crop' | 'management' | 'historical';
  value: number | string;
  impact: number; // -100 to 100
  importance: number; // 0-100
  description?: string;
}

/**
 * Spatial Yield Data
 */
export interface SpatialYieldData {
  resolution: number; // meters
  zones: YieldZone[];
  averageYield: number;
  variability: number; // CV percentage
}

/**
 * Yield Zone
 */
export interface YieldZone {
  zoneId: string;
  geometry: GeoJSON;
  predictedYield: number;
  classification: 'low' | 'medium' | 'high';
  area: number;
}

/**
 * GeoJSON Geometry
 */
export interface GeoJSON {
  type: 'Polygon' | 'MultiPolygon';
  coordinates: number[][][] | number[][][][];
}

/**
 * Historical Yield Data
 */
export interface HistoricalYieldData {
  recordId: string;
  fieldId: string;
  cropType: CropType;
  year: number;
  harvestDate: string;
  actualYield: number;
  unit: string;
  quality?: string;
  area?: number;
  notes?: string;
}

/**
 * Weather Input Data
 */
export interface WeatherInputData {
  dataId: string;
  location: Location;
  startDate: string;
  endDate: string;
  temperature: TemperatureData;
  precipitation: PrecipitationData;
  solarRadiation?: number[]; // Daily values
  humidity?: number[]; // Daily values
  windSpeed?: number[]; // Daily values
}

/**
 * Temperature Data
 */
export interface TemperatureData {
  daily: {
    min: number[];
    max: number[];
    average: number[];
  };
  gdd?: number; // Growing Degree Days
  extremes?: {
    frost: number;
    heatStress: number;
  };
}

/**
 * Precipitation Data
 */
export interface PrecipitationData {
  daily: number[];
  total: number;
  distribution: 'poor' | 'fair' | 'good' | 'excellent';
  droughtDays?: number;
  excessiveDays?: number;
}

/**
 * Location
 */
export interface Location {
  latitude: number;
  longitude: number;
  altitude?: number;
}

/**
 * Soil Input Data
 */
export interface SoilInputData {
  dataId: string;
  fieldId: string;
  samplingDate: string;
  soilType: string;
  texture?: string;
  organicMatter?: number;
  ph?: number;
  nutrients: NutrientData;
  moisture?: MoistureData;
  depth?: number; // cm
}

/**
 * Nutrient Data
 */
export interface NutrientData {
  nitrogen?: number;
  phosphorus?: number;
  potassium?: number;
  calcium?: number;
  magnesium?: number;
  sulfur?: number;
  micronutrients?: Record<string, number>;
}

/**
 * Moisture Data
 */
export interface MoistureData {
  current?: number;
  fieldCapacity?: number;
  wiltingPoint?: number;
  available?: number;
}

/**
 * Crop Management Data
 */
export interface CropManagementData {
  dataId: string;
  fieldId: string;
  plantingDate: string;
  plantingRate?: number;
  rowSpacing?: number;
  fertilization: FertilizationRecord[];
  irrigation: IrrigationRecord[];
  pestManagement: PestManagementRecord[];
  tillage?: TillageRecord[];
}

/**
 * Fertilization Record
 */
export interface FertilizationRecord {
  date: string;
  product: string;
  rate: number;
  unit: string;
  method?: string;
}

/**
 * Irrigation Record
 */
export interface IrrigationRecord {
  date: string;
  amount: number; // mm
  method?: string;
}

/**
 * Pest Management Record
 */
export interface PestManagementRecord {
  date: string;
  type: 'pesticide' | 'fungicide' | 'herbicide';
  product: string;
  targetPest?: string;
}

/**
 * Tillage Record
 */
export interface TillageRecord {
  date: string;
  type: string;
  depth?: number; // cm
}

/**
 * Remote Sensing Data
 */
export interface RemoteSensingData {
  dataId: string;
  fieldId: string;
  captureDate: string;
  source: 'satellite' | 'drone' | 'aerial';
  indices: VegetationIndices;
  biomass?: number;
  canopyCover?: number;
  resolution?: number; // meters
}

/**
 * Vegetation Indices
 */
export interface VegetationIndices {
  ndvi?: number;
  evi?: number;
  ndre?: number;
  gndvi?: number;
  lai?: number; // Leaf Area Index
}

/**
 * Prediction Request
 */
export interface PredictionRequest {
  fieldId: string;
  cropType: CropType;
  variety?: string;
  plantingDate: string;
  expectedHarvestDate: string;
  area: number;
  modelId?: string;
  includeFactors?: boolean;
  includeSpatial?: boolean;
  historicalData?: HistoricalYieldData[];
  weatherData?: WeatherInputData;
  soilData?: SoilInputData;
  managementData?: CropManagementData;
  remoteSensingData?: RemoteSensingData[];
}

/**
 * Model Training Request
 */
export interface ModelTrainingRequest {
  modelName: string;
  modelType: ModelType;
  cropType: CropType;
  trainingData: TrainingDataset[];
  features: string[];
  hyperparameters?: Record<string, any>;
}

/**
 * Training Dataset
 */
export interface TrainingDataset {
  fieldId: string;
  year: number;
  actualYield: number;
  features: Record<string, number | string>;
}

/**
 * Model Validation Result
 */
export interface ModelValidationResult {
  modelId: string;
  validationDate: string;
  metrics: ValidationMetrics;
  testSize: number;
  crossValidationFolds?: number;
}

/**
 * Validation Metrics
 */
export interface ValidationMetrics {
  accuracy: number; // percentage
  rmse: number; // Root Mean Square Error
  mae: number; // Mean Absolute Error
  r2: number; // R-squared
  mape?: number; // Mean Absolute Percentage Error
}

/**
 * Yield Comparison
 */
export interface YieldComparison {
  comparisonId: string;
  fieldId: string;
  cropType: CropType;
  year: number;
  predictedYield: number;
  actualYield: number;
  difference: number;
  percentageError: number;
  factors?: string[];
}

/**
 * Analytics Report
 */
export interface AnalyticsReport {
  reportId: string;
  generatedDate: string;
  period: string;
  fieldId?: string;
  summary: AnalyticsSummary;
  trends: TrendAnalysis[];
  recommendations?: string[];
}

/**
 * Analytics Summary
 */
export interface AnalyticsSummary {
  totalPredictions: number;
  averageAccuracy: number;
  averagePredictedYield: number;
  averageActualYield?: number;
  yieldTrend: 'increasing' | 'stable' | 'decreasing';
}

/**
 * Trend Analysis
 */
export interface TrendAnalysis {
  metric: string;
  values: number[];
  years: number[];
  trend: 'increasing' | 'stable' | 'decreasing';
  correlation?: number;
}

/**
 * SDK Configuration
 */
export interface SDKConfig {
  baseURL?: string;
  apiKey?: string;
  fieldId?: string;
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
  cropType?: string;
  status?: string;
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
