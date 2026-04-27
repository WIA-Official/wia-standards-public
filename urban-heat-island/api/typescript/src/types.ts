/**
 * WIA Urban Heat Island Standard - TypeScript Type Definitions
 *
 * Comprehensive types for urban temperature management and heat mitigation
 *
 * @module @wia/urban-heat-island/types
 * @version 1.0.0
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Geographic coordinate
 */
export interface Coordinate {
  latitude: number;
  longitude: number;
  elevation?: number;
}

/**
 * Temperature reading with metadata
 */
export interface TemperatureReading {
  value: number;
  unit: 'celsius' | 'fahrenheit' | 'kelvin';
  timestamp: Date;
  location: Coordinate;
  source: 'sensor' | 'satellite' | 'model' | 'interpolated';
  reliability: number; // 0-1
}

/**
 * Temperature map grid cell
 */
export interface TemperatureGridCell {
  coordinate: Coordinate;
  temperature: TemperatureReading;
  surfaceMaterial: SurfaceMaterial;
  albedo: number;
  vegetationIndex: number;
  buildingDensity: number;
}

/**
 * Temperature map configuration
 */
export interface TemperatureMap {
  id: string;
  name: string;
  bounds: {
    north: number;
    south: number;
    east: number;
    west: number;
  };
  resolution: number; // meters
  grid: TemperatureGridCell[][];
  timestamp: Date;
  metadata: Record<string, any>;
}

// ============================================================================
// Heat Hotspot Types
// ============================================================================

/**
 * Heat hotspot severity classification
 */
export type HotspotSeverity = 'low' | 'moderate' | 'high' | 'extreme';

/**
 * Heat hotspot detection result
 */
export interface HeatHotspot {
  id: string;
  location: Coordinate;
  area: number; // square meters
  averageTemperature: number;
  peakTemperature: number;
  severity: HotspotSeverity;
  affectedPopulation: number;
  contributingFactors: HotspotFactor[];
  detectedAt: Date;
}

/**
 * Contributing factors to heat hotspots
 */
export interface HotspotFactor {
  type: 'surface-material' | 'lack-of-vegetation' | 'building-density' |
        'traffic' | 'industrial-activity' | 'lack-of-airflow';
  contribution: number; // 0-1 (percentage)
  description: string;
}

/**
 * Hotspot detection configuration
 */
export interface HotspotDetectionConfig {
  temperatureThreshold: number;
  minimumArea: number;
  clusteringDistance: number;
  severityThresholds: {
    low: number;
    moderate: number;
    high: number;
    extreme: number;
  };
}

// ============================================================================
// Surface Material Types
// ============================================================================

/**
 * Surface material categories
 */
export type SurfaceMaterialType =
  | 'asphalt'
  | 'concrete'
  | 'brick'
  | 'metal'
  | 'glass'
  | 'vegetation'
  | 'water'
  | 'soil'
  | 'gravel'
  | 'cool-pavement';

/**
 * Surface material properties
 */
export interface SurfaceMaterial {
  type: SurfaceMaterialType;
  albedo: number; // 0-1 (reflectivity)
  emissivity: number; // 0-1 (heat radiation)
  thermalCapacity: number; // J/(kg·K)
  coverage: number; // percentage of area
}

/**
 * Albedo measurement
 */
export interface AlbedoData {
  value: number; // 0-1
  location: Coordinate;
  surfaceType: SurfaceMaterialType;
  measurementMethod: 'satellite' | 'ground-sensor' | 'estimated';
  timestamp: Date;
}

// ============================================================================
// Vegetation Index Types
// ============================================================================

/**
 * Vegetation index types
 */
export type VegetationIndexType = 'NDVI' | 'EVI' | 'LAI' | 'SAVI';

/**
 * Vegetation index measurement
 */
export interface VegetationIndex {
  type: VegetationIndexType;
  value: number; // -1 to 1 for NDVI, varies for others
  location: Coordinate;
  timestamp: Date;
  source: 'satellite' | 'drone' | 'ground-survey';
}

/**
 * Vegetation coverage analysis
 */
export interface VegetationCoverage {
  totalArea: number; // square meters
  vegetatedArea: number;
  coveragePercentage: number;
  treeCanopyPercentage: number;
  grassPercentage: number;
  coolingEffect: number; // estimated temperature reduction in celsius
}

// ============================================================================
// Cooling Strategy Types
// ============================================================================

/**
 * Cooling strategy categories
 */
export type CoolingStrategyType =
  | 'green-infrastructure'
  | 'cool-pavement'
  | 'white-roof'
  | 'water-feature'
  | 'urban-forest'
  | 'permeable-surface'
  | 'shade-structure'
  | 'ventilation-corridor';

/**
 * Cooling strategy definition
 */
export interface CoolingStrategy {
  id: string;
  name: string;
  type: CoolingStrategyType;
  description: string;
  targetArea: {
    location: Coordinate;
    radius: number; // meters
  };
  estimatedCoolingEffect: number; // celsius
  implementationCost: number;
  maintenanceCost: number; // annual
  lifespan: number; // years
  cobenefits: string[];
}

/**
 * Cooling strategy evaluation result
 */
export interface StrategyEvaluation {
  strategy: CoolingStrategy;
  effectivenessScore: number; // 0-100
  costEffectiveness: number; // cooling per dollar
  implementationFeasibility: number; // 0-1
  communityImpact: number; // 0-1
  environmentalBenefit: number; // 0-1
  recommendationLevel: 'low' | 'medium' | 'high' | 'critical';
}

/**
 * Mitigation plan
 */
export interface MitigationPlan {
  id: string;
  name: string;
  targetHotspots: string[]; // hotspot IDs
  strategies: CoolingStrategy[];
  totalCost: number;
  estimatedCoolingEffect: number;
  timeline: {
    startDate: Date;
    phases: MitigationPhase[];
  };
  priorityLevel: number; // 1-5
}

/**
 * Mitigation plan phase
 */
export interface MitigationPhase {
  name: string;
  duration: number; // months
  strategies: string[]; // strategy IDs
  cost: number;
  expectedOutcome: string;
}

// ============================================================================
// Thermal Comfort Types
// ============================================================================

/**
 * Thermal comfort index types
 */
export type ThermalComfortIndex = 'UTCI' | 'PET' | 'PMV' | 'SET';

/**
 * Thermal comfort calculation input
 */
export interface ThermalComfortInput {
  airTemperature: number; // celsius
  relativeHumidity: number; // percentage
  windSpeed: number; // m/s
  solarRadiation: number; // W/m²
  location: Coordinate;
  timestamp: Date;
}

/**
 * Thermal comfort result
 */
export interface ThermalComfortResult {
  index: ThermalComfortIndex;
  value: number;
  stressLevel: 'no-stress' | 'slight-stress' | 'moderate-stress' |
                'strong-stress' | 'extreme-stress';
  recommendation: string;
  location: Coordinate;
  calculatedAt: Date;
}

// ============================================================================
// Monitoring and Analytics Types
// ============================================================================

/**
 * Temperature trend analysis
 */
export interface TemperatureTrend {
  location: Coordinate;
  period: {
    start: Date;
    end: Date;
  };
  averageTemperature: number;
  maxTemperature: number;
  minTemperature: number;
  trend: 'increasing' | 'decreasing' | 'stable';
  rateOfChange: number; // celsius per year
}

/**
 * Urban heat island intensity
 */
export interface UHIIntensity {
  urbanTemperature: number;
  ruralReferenceTemperature: number;
  intensity: number; // difference
  timestamp: Date;
  affectedArea: number; // square kilometers
}

// ============================================================================
// Event and Notification Types
// ============================================================================

/**
 * Heat event types
 */
export type HeatEventType =
  | 'hotspot-detected'
  | 'temperature-threshold-exceeded'
  | 'strategy-completed'
  | 'mitigation-milestone'
  | 'alert-issued';

/**
 * Heat event
 */
export interface HeatEvent {
  id: string;
  type: HeatEventType;
  severity: 'info' | 'warning' | 'critical';
  location?: Coordinate;
  data: Record<string, any>;
  timestamp: Date;
  message: string;
}

/**
 * Event handler function
 */
export type HeatEventHandler = (event: HeatEvent) => void | Promise<void>;

// ============================================================================
// Configuration Types
// ============================================================================

/**
 * Urban heat island simulator configuration
 */
export interface UHIConfig {
  region: {
    name: string;
    bounds: TemperatureMap['bounds'];
  };
  monitoring: {
    updateInterval: number; // seconds
    temperatureUnit: 'celsius' | 'fahrenheit' | 'kelvin';
  };
  detection: HotspotDetectionConfig;
  analysis: {
    enableTrendAnalysis: boolean;
    historicalDataRetention: number; // days
  };
}
