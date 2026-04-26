/**
 * WIA-AGRI-005 Crop Monitoring Standard - TypeScript Type Definitions
 * Version: 1.0.0
 * Philosophy: 弘益人間 (Benefit All Humanity)
 */

/**
 * WIA Crop Monitoring Standard Version
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
  | 'tomato'
  | 'cotton'
  | 'vegetables'
  | 'fruits'
  | 'other';

/**
 * Growth Stages
 */
export type GrowthStage =
  | 'germination'
  | 'vegetative'
  | 'flowering'
  | 'fruiting'
  | 'maturity'
  | 'harvest';

/**
 * Health Status
 */
export type HealthStatus = 'healthy' | 'stressed' | 'diseased' | 'critical';

/**
 * Issue Types
 */
export type IssueType = 'pest' | 'disease' | 'weed' | 'nutrient' | 'water' | 'weather';

/**
 * Monitoring Method
 */
export type MonitoringMethod =
  | 'visual_inspection'
  | 'sensor'
  | 'drone_imagery'
  | 'satellite'
  | 'ai_analysis';

/**
 * Crop Monitoring Configuration
 */
export interface CropMonitoringConfig {
  cropId: string;
  fieldId: string;
  cropType: CropType;
  variety?: string;
  plantingDate: string;
  expectedHarvestDate?: string;
  area: number; // hectares
  plantPopulation?: number;
  rowSpacing?: number; // cm
  metadata?: Record<string, any>;
}

/**
 * Growth Observation
 */
export interface GrowthObservation {
  observationId: string;
  cropId: string;
  timestamp: string;
  stage: GrowthStage;
  height?: number; // cm
  canopyCover?: number; // percentage
  leafCount?: number;
  biomass?: number; // kg/m²
  healthStatus: HealthStatus;
  notes?: string;
  images?: string[];
  method: MonitoringMethod;
  observer?: string;
}

/**
 * Health Assessment
 */
export interface HealthAssessment {
  assessmentId: string;
  cropId: string;
  timestamp: string;
  overallHealth: HealthStatus;
  ndvi?: number;
  chlorophyllContent?: number;
  leafAreaIndex?: number;
  stressIndicators: StressIndicator[];
  recommendations?: string[];
  confidence?: number; // 0-100
}

/**
 * Stress Indicator
 */
export interface StressIndicator {
  type: 'water' | 'nutrient' | 'heat' | 'cold' | 'light' | 'salinity';
  severity: 'low' | 'medium' | 'high';
  affectedArea?: number; // percentage
  symptoms?: string[];
}

/**
 * Pest Detection
 */
export interface PestDetection {
  detectionId: string;
  cropId: string;
  timestamp: string;
  pestType: string;
  commonName?: string;
  scientificName?: string;
  severity: 'low' | 'medium' | 'high' | 'severe';
  affectedArea: number; // percentage or m²
  location?: Location;
  lifeCycle?: string;
  images?: string[];
  controlMeasures?: string[];
  confidence?: number;
}

/**
 * Disease Detection
 */
export interface DiseaseDetection {
  detectionId: string;
  cropId: string;
  timestamp: string;
  diseaseType: string;
  commonName?: string;
  scientificName?: string;
  pathogen?: 'fungal' | 'bacterial' | 'viral' | 'nematode';
  severity: 'low' | 'medium' | 'high' | 'severe';
  affectedArea: number; // percentage or m²
  location?: Location;
  symptoms: string[];
  images?: string[];
  treatment?: string[];
  confidence?: number;
}

/**
 * Weed Detection
 */
export interface WeedDetection {
  detectionId: string;
  cropId: string;
  timestamp: string;
  weedType: string;
  density: 'low' | 'medium' | 'high';
  coverage: number; // percentage
  location?: Location;
  images?: string[];
  controlRecommendations?: string[];
}

/**
 * Geographic Location
 */
export interface Location {
  latitude: number;
  longitude: number;
  altitude?: number;
  zone?: string;
}

/**
 * Phenology Stage
 */
export interface PhenologyStage {
  stageId: string;
  cropId: string;
  stage: GrowthStage;
  bbchCode?: string; // BBCH scale
  startDate: string;
  endDate?: string;
  duration?: number; // days
  notes?: string;
}

/**
 * Scouting Report
 */
export interface ScoutingReport {
  reportId: string;
  cropId: string;
  date: string;
  scout: string;
  duration?: number; // minutes
  areasInspected: string[];
  findings: ScoutingFinding[];
  overallAssessment: string;
  recommendations?: string[];
  images?: string[];
}

/**
 * Scouting Finding
 */
export interface ScoutingFinding {
  type: IssueType;
  description: string;
  severity: 'low' | 'medium' | 'high';
  location?: string;
  actionRequired?: boolean;
  estimatedImpact?: string;
}

/**
 * Imagery Data
 */
export interface ImageryData {
  imageId: string;
  cropId: string;
  captureDate: string;
  source: 'drone' | 'satellite' | 'ground' | 'mobile';
  imageType: 'rgb' | 'multispectral' | 'thermal' | 'hyperspectral';
  resolution: number; // cm/pixel
  coverage: number; // hectares
  bands?: string[];
  indices?: VegetationIndices;
  imageUrl?: string;
  thumbnailUrl?: string;
  metadata?: Record<string, any>;
}

/**
 * Vegetation Indices
 */
export interface VegetationIndices {
  ndvi?: number; // Normalized Difference Vegetation Index
  ndre?: number; // Normalized Difference Red Edge
  gndvi?: number; // Green NDVI
  savi?: number; // Soil Adjusted Vegetation Index
  evi?: number; // Enhanced Vegetation Index
  msavi?: number; // Modified SAVI
}

/**
 * Yield Estimate
 */
export interface YieldEstimate {
  estimateId: string;
  cropId: string;
  date: string;
  estimatedYield: number;
  unit: string; // e.g., 'tons/ha', 'bushels/acre'
  confidence: number; // 0-100
  method: string;
  factors?: YieldFactor[];
  harvestDate?: string;
}

/**
 * Yield Factor
 */
export interface YieldFactor {
  factor: string;
  impact: 'positive' | 'negative' | 'neutral';
  magnitude?: number; // percentage
  description?: string;
}

/**
 * Treatment Record
 */
export interface TreatmentRecord {
  treatmentId: string;
  cropId: string;
  date: string;
  type: 'pesticide' | 'fungicide' | 'herbicide' | 'fertilizer' | 'biological';
  product: string;
  activeIngredient?: string;
  dosage: number;
  unit: string;
  applicationMethod?: string;
  area?: number; // hectares
  targetIssue?: string;
  operator?: string;
  cost?: number;
  phi?: number; // Pre-Harvest Interval in days
}

/**
 * Weather Impact
 */
export interface WeatherImpact {
  impactId: string;
  cropId: string;
  date: string;
  eventType: 'drought' | 'flood' | 'frost' | 'hail' | 'wind' | 'heat';
  severity: 'minor' | 'moderate' | 'severe' | 'devastating';
  affectedArea?: number; // percentage
  estimatedLoss?: number; // percentage
  description: string;
  images?: string[];
}

/**
 * Analytics Report
 */
export interface AnalyticsReport {
  reportId: string;
  cropId: string;
  period: string;
  generatedDate: string;
  metrics: CropMetrics;
  trends: TrendData[];
  alerts: AlertSummary[];
}

/**
 * Crop Metrics
 */
export interface CropMetrics {
  averageHealth: number; // 0-100
  growthRate?: number;
  stressLevel?: number; // 0-100
  pestPressure?: number; // 0-100
  diseasePressure?: number; // 0-100
  weedPressure?: number; // 0-100
  projectedYield?: number;
}

/**
 * Trend Data
 */
export interface TrendData {
  metric: string;
  values: number[];
  dates: string[];
  trend: 'improving' | 'stable' | 'declining';
  changeRate?: number;
}

/**
 * Alert Summary
 */
export interface AlertSummary {
  type: IssueType;
  count: number;
  severity: 'low' | 'medium' | 'high';
  requiresAction: boolean;
}

/**
 * SDK Configuration
 */
export interface SDKConfig {
  baseURL?: string;
  apiKey?: string;
  cropId?: string;
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
