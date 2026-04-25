/**
 * WIA Desertification Prevention Standard - TypeScript Type Definitions
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

export interface WIADesertificationPrevention {
  standard: 'WIA-DESERTIFICATION-PREVENTION';
  version: string;
  project: ProjectMetadata;
  region: RegionDefinition;
  assessment: LandAssessment;
  interventions: Intervention[];
  monitoring: MonitoringPlan;
  stakeholders?: Stakeholder[];
  extensions?: Record<string, unknown>;
}

export interface ProjectMetadata {
  id: string;
  name: string;
  description?: string;
  organization: string;
  startDate: string;
  endDate?: string;
  status: ProjectStatus;
  funding?: FundingInfo;
  tags?: string[];
}

export type ProjectStatus = 'planning' | 'active' | 'monitoring' | 'completed' | 'suspended';

export interface FundingInfo {
  source: string;
  amount: number;
  currency: string;
  disbursed?: number;
}

export interface RegionDefinition {
  id: string;
  name: string;
  country: string;
  coordinates: GeoCoordinates;
  boundary?: GeoPolygon;
  area: AreaMeasurement;
  climate: ClimateInfo;
  soil: SoilInfo;
}

export interface GeoCoordinates {
  latitude: number;
  longitude: number;
  altitude?: number;
}

export interface GeoPolygon {
  type: 'Polygon';
  coordinates: number[][][];
}

export interface AreaMeasurement {
  value: number;
  unit: 'hectares' | 'km2' | 'acres';
}

export interface ClimateInfo {
  type: ClimateType;
  avgTemperature: number;
  annualRainfall: number;
  rainySeasonMonths?: number[];
  droughtFrequency?: DroughtFrequency;
}

export type ClimateType = 'arid' | 'semi-arid' | 'dry-subhumid' | 'hyper-arid';
export type DroughtFrequency = 'rare' | 'occasional' | 'frequent' | 'chronic';

export interface SoilInfo {
  type: SoilType;
  degradationLevel: DegradationLevel;
  organicMatter: number;
  salinity?: SalinityLevel;
  erosionType?: ErosionType[];
}

export type SoilType = 'sandy' | 'loamy' | 'clay' | 'silty' | 'rocky' | 'mixed';
export type DegradationLevel = 'none' | 'light' | 'moderate' | 'severe' | 'very-severe';
export type SalinityLevel = 'none' | 'slight' | 'moderate' | 'strong' | 'extreme';
export type ErosionType = 'water' | 'wind' | 'chemical' | 'physical';

// ============================================================================
// Assessment Types
// ============================================================================

export interface LandAssessment {
  date: string;
  assessor: string;
  methodology: string;
  desertificationRisk: RiskLevel;
  vegetationCover: VegetationAssessment;
  waterResources: WaterAssessment;
  biodiversity: BiodiversityAssessment;
  humanImpact: HumanImpactAssessment;
  overallScore: number;
}

export type RiskLevel = 'low' | 'moderate' | 'high' | 'very-high' | 'critical';

export interface VegetationAssessment {
  coverPercentage: number;
  dominantSpecies: string[];
  healthStatus: 'healthy' | 'stressed' | 'degraded' | 'dead';
  nativeSpeciesRatio: number;
  invasiveSpecies?: string[];
}

export interface WaterAssessment {
  availability: 'abundant' | 'adequate' | 'limited' | 'scarce' | 'none';
  sources: WaterSource[];
  groundwaterLevel?: number;
  quality?: WaterQuality;
}

export interface WaterSource {
  type: 'river' | 'well' | 'spring' | 'rainwater' | 'aquifer';
  name?: string;
  seasonal?: boolean;
  reliability: 'reliable' | 'seasonal' | 'unpredictable';
}

export type WaterQuality = 'excellent' | 'good' | 'fair' | 'poor' | 'unfit';

export interface BiodiversityAssessment {
  speciesRichness: number;
  endemicSpecies: number;
  threatenedSpecies: number;
  ecosystemHealth: 'intact' | 'modified' | 'degraded' | 'collapsed';
}

export interface HumanImpactAssessment {
  populationDensity: number;
  landUse: LandUseType[];
  overgrazing: boolean;
  deforestation: boolean;
  unsustainableFarming: boolean;
  livelihoodsAtRisk: number;
}

export type LandUseType = 'agriculture' | 'pastoral' | 'forestry' | 'urban' | 'industrial' | 'conservation';

// ============================================================================
// Intervention Types
// ============================================================================

export interface Intervention {
  id: string;
  type: InterventionType;
  name: string;
  description?: string;
  startDate: string;
  endDate?: string;
  status: InterventionStatus;
  location: GeoCoordinates;
  area?: AreaMeasurement;
  techniques: Technique[];
  resources: Resource[];
  expectedOutcomes: Outcome[];
  actualOutcomes?: Outcome[];
}

export type InterventionType =
  | 'afforestation'
  | 'reforestation'
  | 'soil-conservation'
  | 'water-harvesting'
  | 'sustainable-agriculture'
  | 'grazing-management'
  | 'sand-dune-fixation'
  | 'windbreak'
  | 'irrigation-improvement'
  | 'community-education';

export type InterventionStatus = 'planned' | 'in-progress' | 'completed' | 'failed' | 'abandoned';

export interface Technique {
  name: string;
  type: TechniqueType;
  description?: string;
  materials?: string[];
  laborIntensity: 'low' | 'medium' | 'high';
}

export type TechniqueType =
  | 'planting'
  | 'terracing'
  | 'mulching'
  | 'composting'
  | 'check-dam'
  | 'contour-plowing'
  | 'agroforestry'
  | 'rotational-grazing';

export interface Resource {
  type: 'financial' | 'human' | 'equipment' | 'seeds' | 'water';
  quantity: number;
  unit: string;
  cost?: number;
}

export interface Outcome {
  indicator: string;
  baseline: number;
  target: number;
  actual?: number;
  unit: string;
  achievedDate?: string;
}

// ============================================================================
// Monitoring Types
// ============================================================================

export interface MonitoringPlan {
  frequency: MonitoringFrequency;
  indicators: MonitoringIndicator[];
  methods: MonitoringMethod[];
  reports: ReportSchedule[];
  alerts?: AlertConfig[];
}

export type MonitoringFrequency = 'daily' | 'weekly' | 'monthly' | 'quarterly' | 'annually';

export interface MonitoringIndicator {
  id: string;
  name: string;
  category: IndicatorCategory;
  unit: string;
  baselineValue: number;
  targetValue: number;
  thresholds: { warning: number; critical: number };
}

export type IndicatorCategory = 'vegetation' | 'soil' | 'water' | 'biodiversity' | 'climate' | 'socioeconomic';

export interface MonitoringMethod {
  type: 'ground-survey' | 'remote-sensing' | 'community-reporting' | 'automated-sensor';
  description: string;
  equipment?: string[];
  dataFormat?: string;
}

export interface ReportSchedule {
  type: 'progress' | 'annual' | 'impact';
  frequency: string;
  recipients: string[];
  format: 'pdf' | 'html' | 'json';
}

export interface AlertConfig {
  indicator: string;
  condition: string;
  channels: string[];
  severity: 'info' | 'warning' | 'critical';
}

export interface Stakeholder {
  id: string;
  name: string;
  type: StakeholderType;
  role: string;
  contact?: string;
  contribution?: string;
}

export type StakeholderType = 'government' | 'ngo' | 'community' | 'private' | 'international' | 'academic';

// ============================================================================
// API Types
// ============================================================================

export interface APIConfig {
  baseURL: string;
  apiKey?: string;
  timeout?: number;
}

export interface ProjectResponse {
  id: string;
  name: string;
  status: ProjectStatus;
  region: string;
  area: AreaMeasurement;
  interventionCount: number;
  createdAt: string;
  links: { self: string };
}

export interface APIError {
  code: string;
  message: string;
  details?: Record<string, unknown>;
  timestamp: string;
}

export interface ValidationResult {
  valid: boolean;
  errors?: { path: string; message: string }[];
}

export interface PaginatedResponse<T> {
  data: T[];
  pagination: { total: number; limit: number; offset: number; hasMore: boolean };
}
