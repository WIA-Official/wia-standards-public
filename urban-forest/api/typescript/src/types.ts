/**
 * WIA Urban Forest Standard - TypeScript Type Definitions
 * Version: 1.0.0
 *
 * This file contains comprehensive type definitions for the WIA Urban Forest
 * Standard, covering city green space management and urban tree inventory.
 */

// ============================================================================
// Base Types
// ============================================================================

export type WIAVersion = '1.0';

export type ValidationStatus =
  | 'unvalidated'
  | 'in_review'
  | 'validated'
  | 'expert_verified'
  | 'questionable'
  | 'invalid';

export type QualityFlag =
  | 'excellent'
  | 'good'
  | 'fair'
  | 'poor'
  | 'critical'
  | 'missing';

export interface Location {
  latitude: number;
  longitude: number;
  elevation?: number;
  datum?: string;
  precision?: number;
  address?: string;
  district?: string;
  city: string;
}

export interface Inspector {
  id: string;
  name?: string;
  organization?: string;
  email?: string;
  certification?: string;
  certification_date?: string;
}

export interface Quality {
  validation_status: ValidationStatus;
  quality_flags?: QualityFlag[];
  confidence_level?: number;
  last_verified?: string;
}

export interface BaseRecord {
  wia_version: WIAVersion;
  schema_type: string;
  record_id: string;
  timestamp: string; // ISO 8601
  location: Location;
  inspector: Inspector;
  quality: Quality;
}

// ============================================================================
// Tree Inventory Types
// ============================================================================

export type TreeStatus =
  | 'alive_healthy'
  | 'alive_stressed'
  | 'alive_declining'
  | 'dead_standing'
  | 'dead_removed'
  | 'stump'
  | 'vacant_site';

export type TreeCondition =
  | 'excellent'
  | 'good'
  | 'fair'
  | 'poor'
  | 'critical'
  | 'dead';

export type TreeForm =
  | 'single_stem'
  | 'multi_stem'
  | 'clump'
  | 'espalier'
  | 'pollarded';

export type LandUse =
  | 'park'
  | 'street'
  | 'residential'
  | 'commercial'
  | 'industrial'
  | 'institutional'
  | 'natural_area';

export type SiteType =
  | 'planting_strip'
  | 'tree_pit'
  | 'median'
  | 'open_lawn'
  | 'forest'
  | 'landscape_bed';

export interface TreeSpecies {
  scientific_name: string;
  common_name: string;
  genus: string;
  species: string;
  cultivar?: string;
  family?: string;
  native_status: 'native' | 'exotic' | 'naturalized';
  hardiness_zone?: string;
  growth_rate?: 'slow' | 'medium' | 'fast';
  mature_height_m?: number;
  mature_spread_m?: number;
}

export interface TreeMeasurements {
  diameter_at_breast_height_cm: number;
  height_m?: number;
  crown_width_m?: number;
  crown_height_m?: number;
  trunk_circumference_cm?: number;
  number_of_stems?: number;
}

export interface TreeHealth {
  overall_condition: TreeCondition;
  vigor_rating?: number; // 0-100
  canopy_density_percent?: number;
  dieback_percent?: number;
  leaf_discoloration?: boolean;
  pest_presence?: boolean;
  disease_present?: boolean;
  structural_defects?: string[];
}

export interface TreeSite {
  site_type: SiteType;
  land_use: LandUse;
  soil_type?: string;
  soil_ph?: number;
  drainage_class?: 'excellent' | 'good' | 'moderate' | 'poor' | 'very_poor';
  available_growing_space_m2?: number;
  sidewalk_damage?: boolean;
  overhead_utility?: boolean;
  underground_utility?: boolean;
  distance_to_building_m?: number;
}

export interface TreeInventoryRecord extends BaseRecord {
  schema_type: 'tree-inventory';
  tree_id: string;
  species: TreeSpecies;
  tree_status: TreeStatus;
  tree_form: TreeForm;
  measurements: TreeMeasurements;
  health: TreeHealth;
  site: TreeSite;
  planting_date?: string;
  last_inspection_date: string;
  next_inspection_date?: string;
  maintenance_priority?: 'low' | 'medium' | 'high' | 'urgent';
  notes?: string;
  photos?: string[];
}

// ============================================================================
// Canopy Coverage Types
// ============================================================================

export type AnalysisMethod =
  | 'lidar'
  | 'aerial_imagery'
  | 'satellite_imagery'
  | 'ground_survey'
  | 'i_tree'
  | 'ndvi';

export interface CanopyMetrics {
  total_area_m2: number;
  canopy_coverage_m2: number;
  canopy_percentage: number;
  tree_count?: number;
  average_tree_density_per_hectare?: number;
  largest_contiguous_patch_m2?: number;
  fragmentation_index?: number;
}

export interface CanopyDistribution {
  residential_percent?: number;
  commercial_percent?: number;
  industrial_percent?: number;
  parks_percent?: number;
  streets_percent?: number;
  institutional_percent?: number;
}

export interface CanopyAnalysis extends BaseRecord {
  schema_type: 'canopy-analysis';
  analysis_id: string;
  area_name: string;
  boundary_geojson?: any;
  analysis_method: AnalysisMethod;
  analysis_date: string;
  imagery_date?: string;
  resolution_m?: number;
  metrics: CanopyMetrics;
  distribution?: CanopyDistribution;
  canopy_change_since_previous?: {
    previous_analysis_date: string;
    area_change_m2: number;
    percentage_change: number;
  };
}

// ============================================================================
// Ecosystem Service Types
// ============================================================================

export interface CarbonSequestration {
  annual_co2_sequestered_kg: number;
  total_carbon_stored_kg: number;
  calculation_method: string;
}

export interface AirQualityImprovement {
  annual_pollutants_removed_kg: {
    pm25?: number;
    pm10?: number;
    no2?: number;
    so2?: number;
    o3?: number;
    co?: number;
  };
  monetary_value_usd?: number;
}

export interface StormwaterManagement {
  annual_runoff_avoided_m3: number;
  annual_runoff_avoided_liters: number;
  monetary_value_usd?: number;
}

export interface EnergyConservation {
  annual_cooling_savings_kwh?: number;
  annual_heating_savings_kwh?: number;
  total_energy_savings_kwh?: number;
  monetary_value_usd?: number;
}

export interface BiodiversitySupport {
  habitat_quality_score?: number; // 0-100
  native_species_count?: number;
  structural_diversity_index?: number;
  wildlife_observation_count?: number;
}

export interface PropertyValueIncrease {
  estimated_value_increase_usd?: number;
  proximity_premium_percent?: number;
}

export interface EcosystemServices extends BaseRecord {
  schema_type: 'ecosystem-services';
  assessment_id: string;
  area_name: string;
  assessment_period: {
    start_date: string;
    end_date: string;
  };
  tree_count: number;
  total_canopy_m2: number;
  carbon: CarbonSequestration;
  air_quality: AirQualityImprovement;
  stormwater: StormwaterManagement;
  energy?: EnergyConservation;
  biodiversity?: BiodiversitySupport;
  property_value?: PropertyValueIncrease;
  total_annual_value_usd: number;
  calculation_tool?: string;
}

// ============================================================================
// Maintenance Schedule Types
// ============================================================================

export type MaintenanceType =
  | 'pruning'
  | 'removal'
  | 'planting'
  | 'watering'
  | 'mulching'
  | 'fertilization'
  | 'pest_treatment'
  | 'disease_treatment'
  | 'cabling_bracing'
  | 'root_management'
  | 'soil_amendment'
  | 'inspection';

export type MaintenancePriority =
  | 'emergency'
  | 'high'
  | 'medium'
  | 'low'
  | 'routine';

export type MaintenanceStatus =
  | 'scheduled'
  | 'in_progress'
  | 'completed'
  | 'cancelled'
  | 'deferred';

export interface MaintenanceTask {
  task_id: string;
  tree_id?: string;
  tree_ids?: string[];
  maintenance_type: MaintenanceType;
  priority: MaintenancePriority;
  status: MaintenanceStatus;
  scheduled_date: string;
  completion_date?: string;
  assigned_to?: string;
  crew_size?: number;
  estimated_hours?: number;
  actual_hours?: number;
  cost_estimate_usd?: number;
  actual_cost_usd?: number;
  description: string;
  notes?: string;
  before_photos?: string[];
  after_photos?: string[];
}

export interface MaintenanceSchedule extends BaseRecord {
  schema_type: 'maintenance-schedule';
  schedule_id: string;
  area_name?: string;
  tasks: MaintenanceTask[];
  schedule_period: {
    start_date: string;
    end_date: string;
  };
  total_estimated_cost_usd?: number;
  total_actual_cost_usd?: number;
}

// ============================================================================
// Planting Plan Types
// ============================================================================

export type PlantingSeasonRecommendation =
  | 'spring'
  | 'summer'
  | 'fall'
  | 'winter'
  | 'year_round';

export interface PlantingSite {
  site_id: string;
  location: Location;
  site_type: SiteType;
  available_space_m2: number;
  soil_conditions: {
    type?: string;
    ph?: number;
    drainage_class?: string;
    compaction_level?: 'low' | 'medium' | 'high';
  };
  site_constraints: {
    overhead_clearance_m?: number;
    underground_utilities?: boolean;
    sidewalk_present?: boolean;
    parking_present?: boolean;
    building_proximity_m?: number;
  };
  light_conditions?: 'full_sun' | 'partial_shade' | 'full_shade';
  current_status: 'vacant' | 'occupied' | 'planned';
}

export interface SpeciesRecommendation {
  species: TreeSpecies;
  suitability_score: number; // 0-100
  quantity_recommended: number;
  rationale: string;
  considerations?: string[];
}

export interface PlantingPlan extends BaseRecord {
  schema_type: 'planting-plan';
  plan_id: string;
  plan_name: string;
  target_area: string;
  planning_date: string;
  implementation_start_date?: string;
  implementation_end_date?: string;
  sites: PlantingSite[];
  species_recommendations: SpeciesRecommendation[];
  total_trees_planned: number;
  total_estimated_cost_usd?: number;
  canopy_increase_goal_percent?: number;
  diversity_goals?: {
    max_species_percent?: number;
    min_species_count?: number;
    native_species_percent_target?: number;
  };
  season_recommendation: PlantingSeasonRecommendation;
  funding_source?: string;
  community_engagement?: boolean;
}

// ============================================================================
// Health Assessment Types
// ============================================================================

export type RiskRating =
  | 'extreme'
  | 'high'
  | 'moderate'
  | 'low'
  | 'negligible';

export interface PestInfestation {
  pest_name: string;
  scientific_name?: string;
  severity: 'light' | 'moderate' | 'severe';
  affected_area_percent: number;
  treatment_recommended?: string;
}

export interface DiseaseIncidence {
  disease_name: string;
  pathogen?: string;
  severity: 'light' | 'moderate' | 'severe';
  affected_area_percent: number;
  treatment_recommended?: string;
}

export interface StructuralIssue {
  issue_type: string;
  severity: 'minor' | 'moderate' | 'major' | 'critical';
  location_on_tree: string;
  risk_to_public: RiskRating;
  mitigation_required?: string;
}

export interface HealthAssessment extends BaseRecord {
  schema_type: 'health-assessment';
  assessment_id: string;
  tree_id: string;
  assessment_date: string;
  assessor_certification?: string;
  overall_health_rating: number; // 0-100
  vitality_indicators: {
    leaf_size_normal?: boolean;
    leaf_color_normal?: boolean;
    seasonal_timing_normal?: boolean;
    growth_rate_normal?: boolean;
  };
  pests?: PestInfestation[];
  diseases?: DiseaseIncidence[];
  structural_issues?: StructuralIssue[];
  environmental_stress?: {
    drought_stress?: boolean;
    heat_stress?: boolean;
    soil_compaction?: boolean;
    salt_damage?: boolean;
    mechanical_damage?: boolean;
  };
  overall_risk_rating: RiskRating;
  recommended_actions: string[];
  follow_up_date?: string;
}

// ============================================================================
// Carbon Sequestration Types
// ============================================================================

export interface TreeCarbonData {
  tree_id: string;
  species: string;
  dbh_cm: number;
  height_m?: number;
  age_years?: number;
  carbon_storage_kg: number;
  annual_sequestration_kg_per_year: number;
  calculation_method: string;
}

export interface CarbonCalculation extends BaseRecord {
  schema_type: 'carbon-calculation';
  calculation_id: string;
  area_name: string;
  calculation_date: string;
  tree_data: TreeCarbonData[];
  total_trees: number;
  total_carbon_stored_kg: number;
  total_carbon_stored_tons: number;
  annual_sequestration_kg: number;
  annual_sequestration_tons: number;
  co2_equivalent_kg: number;
  co2_equivalent_tons: number;
  monetary_value_usd?: number;
  carbon_price_per_ton_usd?: number;
  methodology: string;
  reference?: string;
}

// ============================================================================
// API Types
// ============================================================================

export interface APIResponse<T> {
  status: 'success' | 'error';
  api_version: string;
  request_id: string;
  timestamp: string;
  query?: Record<string, any>;
  pagination?: Pagination;
  data?: T[];
  error_code?: string;
  message?: string;
  details?: any;
}

export interface Pagination {
  total_records: number;
  returned_records: number;
  page: number;
  total_pages: number;
  next_page?: string;
  previous_page?: string;
}

export interface QueryOptions {
  city?: string;
  district?: string;
  species?: string;
  condition?: TreeCondition;
  status?: TreeStatus;
  start_date?: string;
  end_date?: string;
  bbox?: [number, number, number, number]; // [minLon, minLat, maxLon, maxLat]
  limit?: number;
  offset?: number;
  format?: 'json' | 'csv' | 'geojson' | 'shapefile';
  sort_by?: string;
  sort_order?: 'asc' | 'desc';
}

export interface ClientConfig {
  apiKey?: string;
  baseURL?: string;
  timeout?: number;
  city?: string;
}

// ============================================================================
// Event Types
// ============================================================================

export type EventType =
  | 'tree_planted'
  | 'tree_removed'
  | 'tree_pruned'
  | 'health_assessment_completed'
  | 'maintenance_scheduled'
  | 'maintenance_completed'
  | 'canopy_analysis_completed'
  | 'risk_alert'
  | 'pest_outbreak'
  | 'disease_outbreak';

export interface UrbanForestEvent {
  event_id: string;
  event_type: EventType;
  timestamp: string;
  tree_id?: string;
  location?: Location;
  description: string;
  severity?: 'info' | 'warning' | 'critical';
  data?: Record<string, any>;
}

// ============================================================================
// Analytics Types
// ============================================================================

export interface UrbanForestMetrics {
  total_trees: number;
  total_canopy_hectares: number;
  canopy_coverage_percent: number;
  trees_per_capita?: number;
  average_tree_age_years?: number;
  species_diversity_index?: number;
  most_common_species: Array<{
    species: string;
    count: number;
    percentage: number;
  }>;
  health_distribution: {
    excellent: number;
    good: number;
    fair: number;
    poor: number;
    critical: number;
  };
  annual_carbon_sequestration_tons: number;
  total_carbon_storage_tons: number;
  total_ecosystem_services_value_usd: number;
}

// ============================================================================
// Union Types
// ============================================================================

export type UrbanForestRecord =
  | TreeInventoryRecord
  | CanopyAnalysis
  | EcosystemServices
  | MaintenanceSchedule
  | PlantingPlan
  | HealthAssessment
  | CarbonCalculation;

// ============================================================================
// Validation Types
// ============================================================================

export interface ValidationResult {
  valid: boolean;
  errors: ValidationError[];
  warnings?: ValidationWarning[];
}

export interface ValidationError {
  field: string;
  message: string;
  value?: any;
  severity: 'error';
}

export interface ValidationWarning {
  field: string;
  message: string;
  value?: any;
  severity: 'warning';
}
