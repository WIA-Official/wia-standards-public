/**
 * WIA Pest Detection Standard - TypeScript Types
 * @version 1.0.0
 * @license MIT
 */

/**
 * Detection methods
 */
export type DetectionMethod =
  | 'visual_inspection'
  | 'image_ai'
  | 'trap_count'
  | 'sensor_array'
  | 'lab_analysis';

/**
 * Pest types
 */
export type PestType = 'insect' | 'fungal' | 'bacterial' | 'viral' | 'nematode' | 'weed';

/**
 * Life stages
 */
export type LifeStage = 'egg' | 'larva' | 'nymph' | 'adult' | 'spore' | 'vegetative';

/**
 * Severity levels
 */
export type SeverityLevel = 'low' | 'medium' | 'high' | 'critical';

/**
 * Pathogen types
 */
export type PathogenType = 'fungal' | 'bacterial' | 'viral' | 'phytoplasma';

/**
 * Symptom locations
 */
export type SymptomLocation = 'leaf' | 'stem' | 'root' | 'fruit' | 'flower';

/**
 * Progression stages
 */
export type ProgressionStage = 'early' | 'intermediate' | 'advanced';

/**
 * Trap types
 */
export type TrapType = 'pheromone' | 'light' | 'sticky' | 'pitfall';

/**
 * Spatial distribution patterns
 */
export type DistributionPattern = 'random' | 'clustered' | 'edge_effect' | 'uniform';

/**
 * Image formats
 */
export type ImageFormat = 'jpg' | 'png' | 'tiff' | 'raw';

/**
 * Lighting conditions
 */
export type LightingCondition = 'natural' | 'artificial' | 'mixed';

/**
 * AI model types
 */
export type AIModelType = 'cnn' | 'yolo' | 'rcnn' | 'transformer';

/**
 * Coordinates
 */
export interface Coordinates {
  latitude: number;
  longitude: number;
  elevation?: number;
}

/**
 * Location information
 */
export interface Location {
  field_id: string;
  coordinates: Coordinates;
  area_ha: number;
  region: string;
}

/**
 * Common name translations
 */
export interface CommonName {
  en: string;
  ko: string;
  local?: string;
}

/**
 * Pest identification
 */
export interface PestIdentification {
  pest_type: PestType;
  species_id: string;
  common_name: CommonName;
  life_stage: LifeStage;
  confidence_score: number;
}

/**
 * Severity assessment
 */
export interface SeverityAssessment {
  level: SeverityLevel;
  infestation_percent: number;
  population_density: number;
  damage_score: number;
  economic_threshold_exceeded: boolean;
}

/**
 * Crop information
 */
export interface CropInfo {
  crop_type: string;
  variety: string;
  growth_stage: string;
  planting_date: string;
}

/**
 * Detection metadata
 */
export interface DetectionMetadata {
  inspector_id: string;
  equipment_used: string;
  images_analyzed: number;
  samples_collected: number;
}

/**
 * Pest detection record
 */
export interface PestDetection {
  detection_id: string;
  timestamp: string;
  detection_method: DetectionMethod;
  location: Location;
  pest_identification: PestIdentification;
  severity_assessment: SeverityAssessment;
  crop_information: CropInfo;
  detection_metadata: DetectionMetadata;
}

/**
 * Disease name translations
 */
export interface DiseaseName {
  scientific: string;
  common_en: string;
  common_ko: string;
}

/**
 * Pathogen information
 */
export interface Pathogen {
  type: PathogenType;
  species: string;
  strain?: string;
}

/**
 * Disease symptom
 */
export interface Symptom {
  location: SymptomLocation;
  appearance: string;
  color_pattern: string;
  texture: string;
  progression_stage: ProgressionStage;
}

/**
 * Diagnostic markers
 */
export interface DiagnosticMarkers {
  visual_cues: string[];
  microscopic_features: string[];
  molecular_markers: string[];
}

/**
 * Temperature range
 */
export interface TemperatureRange {
  min: number;
  max: number;
}

/**
 * Humidity range
 */
export interface HumidityRange {
  min: number;
  max: number;
}

/**
 * Environmental triggers
 */
export interface EnvironmentalTriggers {
  temperature_range_c: TemperatureRange;
  humidity_range_percent: HumidityRange;
  favorable_conditions: string;
}

/**
 * Disease symptom catalog
 */
export interface DiseaseSymptomCatalog {
  symptom_id: string;
  disease_id: string;
  disease_name: DiseaseName;
  pathogen: Pathogen;
  symptoms: Symptom[];
  diagnostic_markers: DiagnosticMarkers;
  environmental_triggers: EnvironmentalTriggers;
}

/**
 * Monitoring period
 */
export interface MonitoringPeriod {
  start_date: string;
  end_date: string;
}

/**
 * Population metrics
 */
export interface PopulationMetrics {
  average_density: number;
  peak_density: number;
  growth_rate: number;
  generation_time_days: number;
}

/**
 * Trap reading
 */
export interface TrapReading {
  date: string;
  count: number;
  species_breakdown: Record<string, number>;
}

/**
 * Trap data
 */
export interface TrapData {
  trap_id: string;
  trap_type: TrapType;
  deployment_date: string;
  readings: TrapReading[];
}

/**
 * Hotspot location
 */
export interface Hotspot {
  coordinates: {
    lat: number;
    lon: number;
  };
  density: number;
  radius_m: number;
}

/**
 * Spatial distribution
 */
export interface SpatialDistribution {
  pattern: DistributionPattern;
  hotspots: Hotspot[];
}

/**
 * Pest population data
 */
export interface PestPopulation {
  population_id: string;
  location_id: string;
  pest_species: string;
  monitoring_period: MonitoringPeriod;
  population_metrics: PopulationMetrics;
  trap_data: TrapData[];
  spatial_distribution: SpatialDistribution;
}

/**
 * Image metadata
 */
export interface ImageMetadata {
  resolution: string;
  format: ImageFormat;
  camera_model: string;
  focal_length_mm: number;
}

/**
 * Capture conditions
 */
export interface CaptureConditions {
  lighting: LightingCondition;
  weather: string;
  time_of_day: string;
}

/**
 * Bounding box
 */
export interface BoundingBox {
  x: number;
  y: number;
  width: number;
  height: number;
}

/**
 * AI detection result
 */
export interface AIDetectionResult {
  class: string;
  confidence: number;
  bounding_box: BoundingBox;
  severity_estimate: SeverityLevel;
}

/**
 * AI analysis
 */
export interface AIAnalysis {
  model_version: string;
  model_type: AIModelType;
  processing_time_ms: number;
  detections: AIDetectionResult[];
  total_detections: number;
  average_confidence: number;
}

/**
 * Ground truth verification
 */
export interface GroundTruth {
  verified: boolean;
  actual_pest: string;
  verification_method: string;
  verified_by: string;
}

/**
 * Image recognition data
 */
export interface ImageRecognition {
  image_id: string;
  capture_timestamp: string;
  image_metadata: ImageMetadata;
  capture_conditions: CaptureConditions;
  ai_analysis: AIAnalysis;
  ground_truth?: GroundTruth;
}

/**
 * Treatment recommendation
 */
export interface TreatmentRecommendation {
  treatment_id: string;
  pest_id: string;
  recommended_action: string;
  chemical_treatment?: string;
  biological_control?: string;
  cultural_practice?: string;
  timing: string;
  efficacy_rate: number;
}

/**
 * API Response wrapper
 */
export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: string;
  timestamp: string;
}

/**
 * Query parameters
 */
export interface PestDetectionQuery {
  field_id?: string;
  pest_type?: PestType;
  severity?: SeverityLevel;
  start_date?: string;
  end_date?: string;
  limit?: number;
  offset?: number;
}
