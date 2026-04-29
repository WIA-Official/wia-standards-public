/**
 * WIA Ecosystem Restoration Standard - TypeScript Type Definitions
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

export interface WIAEcosystemRestoration {
  standard: 'WIA-ECOSYSTEM-RESTORATION';
  version: string;
  project: RestorationProject;
  site: RestorationSite;
  species: SpeciesData[];
  interventions: RestorationIntervention[];
  monitoring: MonitoringProgram;
  biodiversity: BiodiversityMetrics;
  stakeholders: Stakeholder[];
  funding: FundingInfo;
  extensions?: Record<string, unknown>;
}

export interface RestorationProject {
  id: string;
  name: string;
  type: ProjectType;
  status: ProjectStatus;
  objectives: RestorationObjective[];
  timeline: ProjectTimeline;
  lead: OrganizationInfo;
  certifications: Certification[];
  createdAt: string;
}

export type ProjectType = 'forest' | 'wetland' | 'grassland' | 'coral-reef' | 'mangrove' | 'riparian' | 'urban' | 'mixed';
export type ProjectStatus = 'planning' | 'preparation' | 'implementation' | 'monitoring' | 'maintenance' | 'completed';

export interface RestorationObjective {
  id: string;
  type: ObjectiveType;
  target: string;
  metric: string;
  baseline: number;
  goal: number;
  deadline: string;
  progress: number;
}

export type ObjectiveType = 'biodiversity' | 'carbon-sequestration' | 'water-quality' | 'soil-health' | 'habitat-connectivity' | 'community-benefit';

export interface ProjectTimeline {
  startDate: string;
  expectedEndDate: string;
  phases: ProjectPhase[];
  milestones: Milestone[];
}

export interface ProjectPhase {
  name: string;
  startDate: string;
  endDate: string;
  status: 'pending' | 'active' | 'completed';
  activities: string[];
}

export interface Milestone {
  id: string;
  name: string;
  date: string;
  achieved: boolean;
  verification?: string;
}

export interface OrganizationInfo {
  id: string;
  name: string;
  type: 'ngo' | 'government' | 'private' | 'academic' | 'community';
  contact: { email: string; phone?: string };
}

export interface Certification {
  type: string;
  issuer: string;
  validUntil: string;
  standard: string;
}

// ============================================================================
// Site Types
// ============================================================================

export interface RestorationSite {
  id: string;
  name: string;
  location: GeoLocation;
  area: AreaInfo;
  ecosystem: EcosystemInfo;
  landUse: LandUseHistory;
  degradation: DegradationAssessment;
  climate: ClimateInfo;
}

export interface GeoLocation {
  latitude: number;
  longitude: number;
  elevation?: number;
  country: string;
  region?: string;
  coordinates?: number[][];
}

export interface AreaInfo {
  total: number;
  restored: number;
  unit: 'hectares' | 'acres' | 'km2';
  zones: RestorationZone[];
}

export interface RestorationZone {
  id: string;
  name: string;
  area: number;
  type: string;
  priority: 'high' | 'medium' | 'low';
  status: string;
}

export interface EcosystemInfo {
  type: string;
  biome: string;
  ecoregion?: string;
  referenceEcosystem?: string;
  keySpecies: string[];
}

export interface LandUseHistory {
  previous: string;
  duration: number;
  impacts: string[];
  degradationCauses: string[];
}

export interface DegradationAssessment {
  level: 'severe' | 'moderate' | 'light' | 'minimal';
  factors: DegradationFactor[];
  recoveryPotential: 'high' | 'medium' | 'low';
}

export interface DegradationFactor {
  type: string;
  severity: number;
  area: number;
  reversible: boolean;
}

export interface ClimateInfo {
  zone: string;
  annualRainfall: number;
  temperature: { min: number; max: number; average: number };
  growingSeason: { start: string; end: string };
}

// ============================================================================
// Species Types
// ============================================================================

export interface SpeciesData {
  id: string;
  scientificName: string;
  commonName: string;
  type: SpeciesType;
  status: ConservationStatus;
  role: EcologicalRole;
  native: boolean;
  population: PopulationData;
  habitat: HabitatRequirements;
}

export type SpeciesType = 'plant' | 'mammal' | 'bird' | 'reptile' | 'amphibian' | 'fish' | 'invertebrate' | 'fungi';
export type ConservationStatus = 'LC' | 'NT' | 'VU' | 'EN' | 'CR' | 'EW' | 'EX' | 'DD';
export type EcologicalRole = 'keystone' | 'indicator' | 'engineer' | 'pollinator' | 'seed-disperser' | 'predator' | 'prey';

export interface PopulationData {
  baseline: number;
  current: number;
  target: number;
  trend: 'increasing' | 'stable' | 'decreasing' | 'unknown';
  surveys: PopulationSurvey[];
}

export interface PopulationSurvey {
  date: string;
  count: number;
  method: string;
  confidence: number;
}

export interface HabitatRequirements {
  preferred: string[];
  vegetation: string[];
  waterNeeds: 'high' | 'medium' | 'low';
  territory?: number;
}

// ============================================================================
// Intervention Types
// ============================================================================

export interface RestorationIntervention {
  id: string;
  type: InterventionType;
  name: string;
  description: string;
  area: number;
  timeline: { start: string; end: string };
  methods: InterventionMethod[];
  resources: ResourceRequirement[];
  outcomes: InterventionOutcome[];
  status: 'planned' | 'in-progress' | 'completed' | 'cancelled';
}

export type InterventionType = 'planting' | 'seeding' | 'invasive-removal' | 'erosion-control' | 'water-management' | 'fire-management' | 'wildlife-habitat' | 'soil-amendment';

export interface InterventionMethod {
  technique: string;
  equipment: string[];
  materials: MaterialInfo[];
  laborHours: number;
}

export interface MaterialInfo {
  name: string;
  quantity: number;
  unit: string;
  source: 'local' | 'regional' | 'imported';
}

export interface ResourceRequirement {
  type: 'funding' | 'labor' | 'equipment' | 'materials' | 'expertise';
  description: string;
  quantity: number;
  unit: string;
  cost?: number;
}

export interface InterventionOutcome {
  metric: string;
  baseline: number;
  target: number;
  achieved: number;
  verified: boolean;
}

// ============================================================================
// Monitoring Types
// ============================================================================

export interface MonitoringProgram {
  id: string;
  frequency: 'daily' | 'weekly' | 'monthly' | 'quarterly' | 'annually';
  indicators: MonitoringIndicator[];
  protocols: MonitoringProtocol[];
  data: MonitoringRecord[];
  reporting: ReportingConfig;
}

export interface MonitoringIndicator {
  id: string;
  name: string;
  type: 'ecological' | 'social' | 'economic';
  metric: string;
  unit: string;
  target: number;
  threshold: { warning: number; critical: number };
}

export interface MonitoringProtocol {
  id: string;
  name: string;
  method: string;
  equipment: string[];
  frequency: string;
  trained: string[];
}

export interface MonitoringRecord {
  id: string;
  date: string;
  indicator: string;
  value: number;
  location?: GeoLocation;
  observer: string;
  notes?: string;
  photos?: string[];
}

export interface ReportingConfig {
  frequency: string;
  recipients: string[];
  format: 'pdf' | 'excel' | 'online-dashboard';
  public: boolean;
}

// ============================================================================
// Biodiversity Types
// ============================================================================

export interface BiodiversityMetrics {
  speciesRichness: number;
  shannonIndex: number;
  simpsonIndex: number;
  evenness: number;
  functionalDiversity: number;
  trendAnalysis: TrendAnalysis;
  comparisons: BiodiversityComparison[];
}

export interface TrendAnalysis {
  period: string;
  direction: 'improving' | 'stable' | 'declining';
  rate: number;
  confidence: number;
}

export interface BiodiversityComparison {
  reference: string;
  similarity: number;
  missingSpecies: string[];
  additionalSpecies: string[];
}

// ============================================================================
// Stakeholder Types
// ============================================================================

export interface Stakeholder {
  id: string;
  name: string;
  type: StakeholderType;
  role: string;
  involvement: InvolvementLevel;
  contact: { email: string; phone?: string };
  contributions: Contribution[];
}

export type StakeholderType = 'community' | 'indigenous' | 'government' | 'ngo' | 'academic' | 'private' | 'donor';
export type InvolvementLevel = 'lead' | 'partner' | 'consultant' | 'beneficiary' | 'observer';

export interface Contribution {
  type: 'funding' | 'labor' | 'land' | 'expertise' | 'materials';
  description: string;
  value?: number;
  period: string;
}

// ============================================================================
// Funding Types
// ============================================================================

export interface FundingInfo {
  totalBudget: number;
  secured: number;
  currency: string;
  sources: FundingSource[];
  expenditure: Expenditure[];
  carbonCredits?: CarbonCreditInfo;
}

export interface FundingSource {
  name: string;
  type: 'grant' | 'donation' | 'government' | 'carbon-credits' | 'impact-investment';
  amount: number;
  status: 'pledged' | 'committed' | 'received';
  conditions?: string[];
}

export interface Expenditure {
  category: string;
  planned: number;
  actual: number;
  period: string;
}

export interface CarbonCreditInfo {
  registry: string;
  methodology: string;
  estimatedCredits: number;
  verifiedCredits: number;
  pricePerTon: number;
}

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
  type: ProjectType;
  status: ProjectStatus;
  area: number;
  speciesCount: number;
  links: { self: string };
}

export interface ValidationResult {
  valid: boolean;
  errors?: { path: string; message: string }[];
}

export interface PaginatedResponse<T> {
  data: T[];
  pagination: { total: number; limit: number; offset: number; hasMore: boolean };
}
