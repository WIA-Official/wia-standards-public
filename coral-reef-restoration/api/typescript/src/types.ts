/**
 * WIA Coral Reef Restoration Standard - TypeScript Type Definitions
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Coral Reef Types
// ============================================================================

export interface WIACoralReefProject {
  standard: 'WIA-CORAL-REEF-RESTORATION';
  version: string;
  metadata: ProjectMetadata;
  siteAssessment: SiteAssessment;
  restoration: RestorationPlan;
  coralNursery: CoralNursery;
  transplantation: TransplantationPlan;
  monitoring: MonitoringProgram;
  communityEngagement: CommunityProgram;
  extensions?: Record<string, unknown>;
}

export interface ProjectMetadata {
  id: string;
  name: string;
  description?: string;
  type: ProjectType;
  location: MarineLocation;
  timeframe: ProjectTimeframe;
  leadOrganization: Organization;
  partners?: Organization[];
  funding: FundingInfo;
  status: ProjectStatus;
}

export type ProjectType =
  | 'active-restoration'
  | 'passive-restoration'
  | 'hybrid'
  | 'research'
  | 'monitoring-only';

export interface MarineLocation {
  name: string;
  country: string;
  region: string;
  marineProtectedArea?: string;
  coordinates: Coordinates;
  reefArea: ReefArea;
  depth: DepthRange;
  jurisdiction?: string;
}

export interface Coordinates {
  latitude: number;
  longitude: number;
}

export interface ReefArea {
  total: number;
  targetRestoration: number;
  unit: 'hectares' | 'km2' | 'm2';
  polygon?: GeoJSON;
}

export interface GeoJSON {
  type: 'Polygon' | 'MultiPolygon';
  coordinates: number[][][];
}

export interface DepthRange {
  min: number;
  max: number;
  unit: 'meters' | 'feet';
  optimalDepth?: number;
}

export interface ProjectTimeframe {
  startDate: string;
  endDate?: string;
  phases: ProjectPhase[];
  monitoringPeriod: number;
}

export interface ProjectPhase {
  id: string;
  name: string;
  start: string;
  end: string;
  objectives: string[];
  status: PhaseStatus;
}

export type PhaseStatus =
  | 'planning'
  | 'preparation'
  | 'implementation'
  | 'monitoring'
  | 'completed';

export interface Organization {
  name: string;
  type: OrganizationType;
  country: string;
  role?: string;
  contact?: ContactInfo;
  expertise?: string[];
}

export type OrganizationType =
  | 'government'
  | 'ngo'
  | 'research-institution'
  | 'university'
  | 'private-sector'
  | 'community-based';

export interface ContactInfo {
  name: string;
  email: string;
  phone?: string;
}

export interface FundingInfo {
  total: Money;
  sources: FundingSource[];
  secured: number;
  gap?: Money;
}

export interface Money {
  amount: number;
  currency: string;
}

export interface FundingSource {
  name: string;
  type: 'government' | 'foundation' | 'corporate' | 'crowdfunding' | 'carbon-credit';
  amount: Money;
  status: 'committed' | 'pledged' | 'received';
}

export type ProjectStatus =
  | 'planning'
  | 'approved'
  | 'implementation'
  | 'monitoring'
  | 'completed'
  | 'suspended';

// ============================================================================
// Site Assessment
// ============================================================================

export interface SiteAssessment {
  baseline: BaselineSurvey;
  waterQuality: WaterQualityAssessment;
  threats: ThreatAssessment;
  suitability: SuitabilityAnalysis;
  biodiversity: BiodiversityAssessment;
}

export interface BaselineSurvey {
  date: string;
  methodology: string;
  coralCover: PercentageCover;
  reefHealth: ReefHealthIndex;
  speciesInventory: SpeciesRecord[];
  substrate: SubstrateAnalysis;
  photos?: SurveyPhoto[];
}

export interface PercentageCover {
  live: number;
  dead: number;
  algae: number;
  sand: number;
  rubble: number;
  other: number;
}

export interface ReefHealthIndex {
  score: number;
  category: 'excellent' | 'good' | 'fair' | 'poor' | 'critical';
  indicators: HealthIndicator[];
}

export interface HealthIndicator {
  name: string;
  value: number;
  unit: string;
  status: 'healthy' | 'stressed' | 'degraded';
}

export interface SpeciesRecord {
  scientificName: string;
  commonName?: string;
  category: 'coral' | 'fish' | 'invertebrate' | 'algae';
  abundance: 'rare' | 'uncommon' | 'common' | 'abundant';
  status?: 'native' | 'endemic' | 'invasive';
  conservation?: string;
}

export interface SubstrateAnalysis {
  type: SubstrateType;
  stability: 'stable' | 'moderate' | 'unstable';
  suitability: 'excellent' | 'good' | 'fair' | 'poor';
  recommendations?: string[];
}

export type SubstrateType =
  | 'natural-reef'
  | 'rock'
  | 'sand'
  | 'rubble'
  | 'artificial';

export interface SurveyPhoto {
  id: string;
  location: Coordinates;
  depth: number;
  timestamp: string;
  description?: string;
  url?: string;
}

export interface WaterQualityAssessment {
  parameters: WaterParameter[];
  seasonalVariation?: SeasonalData;
  pollutionSources?: PollutionSource[];
  overall: 'excellent' | 'good' | 'moderate' | 'poor';
}

export interface WaterParameter {
  name: string;
  value: number;
  unit: string;
  optimalRange: { min: number; max: number };
  status: 'optimal' | 'acceptable' | 'concerning' | 'critical';
}

export interface SeasonalData {
  wetSeason: Record<string, number>;
  drySeason: Record<string, number>;
  monsoon?: Record<string, number>;
}

export interface PollutionSource {
  type: 'agricultural' | 'urban' | 'industrial' | 'marine' | 'natural';
  description: string;
  severity: 'low' | 'moderate' | 'high';
  mitigation?: string;
}

export interface ThreatAssessment {
  threats: Threat[];
  overallRisk: 'low' | 'moderate' | 'high' | 'critical';
  climateProjections: ClimateProjection[];
}

export interface Threat {
  type: ThreatType;
  severity: 'low' | 'moderate' | 'high' | 'critical';
  trend: 'increasing' | 'stable' | 'decreasing';
  description: string;
  mitigation?: string[];
}

export type ThreatType =
  | 'climate-change'
  | 'ocean-acidification'
  | 'bleaching'
  | 'disease'
  | 'pollution'
  | 'overfishing'
  | 'destructive-fishing'
  | 'coastal-development'
  | 'tourism'
  | 'invasive-species'
  | 'storm-damage';

export interface ClimateProjection {
  scenario: string;
  timeHorizon: string;
  temperatureChange: number;
  seaLevelRise: number;
  acidificationChange: number;
  implications: string[];
}

export interface SuitabilityAnalysis {
  score: number;
  category: 'highly-suitable' | 'suitable' | 'marginal' | 'unsuitable';
  factors: SuitabilityFactor[];
  recommendations: string[];
}

export interface SuitabilityFactor {
  name: string;
  score: number;
  weight: number;
  notes?: string;
}

export interface BiodiversityAssessment {
  coralDiversity: DiversityIndex;
  fishDiversity: DiversityIndex;
  invertebrateCount: number;
  keyStoneSpecies: string[];
  endangeredSpecies: string[];
  ecologicalFunction: string;
}

export interface DiversityIndex {
  speciesRichness: number;
  shannonIndex?: number;
  simpsonIndex?: number;
  evenness?: number;
}

// ============================================================================
// Restoration Plan
// ============================================================================

export interface RestorationPlan {
  objectives: RestorationObjective[];
  methods: RestorationMethod[];
  targets: RestorationTarget[];
  timeline: ActivityTimeline;
  resources: ResourceRequirements;
  risks: Risk[];
}

export interface RestorationObjective {
  id: string;
  description: string;
  type: 'ecological' | 'social' | 'economic';
  indicator: string;
  baseline: number;
  target: number;
  timeline: string;
}

export interface RestorationMethod {
  id: string;
  name: string;
  type: MethodType;
  description: string;
  applicability: string;
  successRate?: number;
  cost: CostEstimate;
}

export type MethodType =
  | 'coral-gardening'
  | 'larval-propagation'
  | 'micro-fragmentation'
  | 'substrate-stabilization'
  | 'artificial-reef'
  | 'assisted-gene-flow'
  | 'threat-removal'
  | 'hybrid';

export interface CostEstimate {
  perUnit: Money;
  unit: string;
  total?: Money;
}

export interface RestorationTarget {
  metric: string;
  current: number;
  target: number;
  unit: string;
  deadline: string;
  priority: 'high' | 'medium' | 'low';
}

export interface ActivityTimeline {
  activities: Activity[];
  milestones: Milestone[];
  dependencies: Dependency[];
}

export interface Activity {
  id: string;
  name: string;
  phase: string;
  start: string;
  end: string;
  responsible: string;
  status: 'pending' | 'in-progress' | 'completed' | 'delayed';
}

export interface Milestone {
  id: string;
  name: string;
  date: string;
  criteria: string;
  achieved: boolean;
}

export interface Dependency {
  activity: string;
  dependsOn: string;
  type: 'finish-start' | 'start-start';
}

export interface ResourceRequirements {
  personnel: PersonnelNeeds;
  equipment: EquipmentNeeds;
  materials: MaterialNeeds;
  budget: BudgetBreakdown;
}

export interface PersonnelNeeds {
  roles: StaffRole[];
  volunteers: VolunteerNeeds;
  training: TrainingNeeds;
}

export interface StaffRole {
  title: string;
  count: number;
  qualifications: string[];
  duration: string;
}

export interface VolunteerNeeds {
  count: number;
  roles: string[];
  training: string;
}

export interface TrainingNeeds {
  topics: string[];
  duration: string;
  certification?: string;
}

export interface EquipmentNeeds {
  diving: string[];
  scientific: string[];
  restoration: string[];
  boats: BoatRequirement[];
}

export interface BoatRequirement {
  type: string;
  count: number;
  specifications: string;
}

export interface MaterialNeeds {
  substrates: SubstrateMaterial[];
  nurseryMaterials: string[];
  markers: string[];
  other: string[];
}

export interface SubstrateMaterial {
  type: string;
  quantity: number;
  unit: string;
  specifications?: string;
}

export interface BudgetBreakdown {
  total: Money;
  categories: BudgetCategory[];
  contingency: number;
}

export interface BudgetCategory {
  name: string;
  amount: Money;
  percentage: number;
}

export interface Risk {
  id: string;
  description: string;
  category: 'environmental' | 'technical' | 'financial' | 'social';
  probability: 'low' | 'medium' | 'high';
  impact: 'low' | 'medium' | 'high';
  mitigation: string;
  contingency?: string;
}

// ============================================================================
// Coral Nursery
// ============================================================================

export interface CoralNursery {
  type: NurseryType;
  locations: NurseryLocation[];
  species: NurserySpecies[];
  capacity: NurseryCapacity;
  maintenance: MaintenanceProtocol;
  production: ProductionMetrics;
}

export type NurseryType =
  | 'in-situ-rope'
  | 'in-situ-table'
  | 'in-situ-tree'
  | 'ex-situ-land'
  | 'ex-situ-floating'
  | 'hybrid';

export interface NurseryLocation {
  id: string;
  name: string;
  coordinates: Coordinates;
  depth: number;
  type: NurseryType;
  structures: NurseryStructure[];
  status: 'operational' | 'developing' | 'inactive';
}

export interface NurseryStructure {
  id: string;
  type: 'rope' | 'table' | 'tree' | 'tank' | 'raceway';
  dimensions: { length: number; width: number; height?: number };
  capacity: number;
  currentStock: number;
}

export interface NurserySpecies {
  scientificName: string;
  commonName?: string;
  type: CoralType;
  fragments: number;
  colonies: number;
  source: FragmentSource;
  growthRate: number;
  survivalRate: number;
  geneticDiversity?: number;
}

export type CoralType =
  | 'branching'
  | 'massive'
  | 'plating'
  | 'encrusting'
  | 'foliose'
  | 'columnar';

export interface FragmentSource {
  type: 'wild-collection' | 'nursery-propagation' | 'rescue' | 'aquaculture';
  location?: string;
  collectionDate?: string;
  permit?: string;
}

export interface NurseryCapacity {
  current: number;
  maximum: number;
  annualProduction: number;
  expansionPlanned?: boolean;
}

export interface MaintenanceProtocol {
  cleaning: MaintenanceTask;
  monitoring: MaintenanceTask;
  predatorControl: MaintenanceTask;
  diseaseManagement: DiseaseProtocol;
}

export interface MaintenanceTask {
  frequency: string;
  procedure: string;
  responsible: string;
  equipment?: string[];
}

export interface DiseaseProtocol {
  monitoring: string;
  identification: string[];
  treatment: string[];
  quarantine: string;
}

export interface ProductionMetrics {
  fragmentsProduced: number;
  survivalRate: number;
  growthRate: number;
  readyForOutplanting: number;
  outplantedToDate: number;
}

// ============================================================================
// Transplantation
// ============================================================================

export interface TransplantationPlan {
  sites: TransplantSite[];
  methods: TransplantMethod[];
  schedule: TransplantSchedule;
  protocols: TransplantProtocol;
  postCare: PostTransplantCare;
}

export interface TransplantSite {
  id: string;
  name: string;
  location: Coordinates;
  area: number;
  depth: DepthRange;
  substrate: SubstrateType;
  targetDensity: number;
  currentDensity?: number;
  status: 'planned' | 'in-progress' | 'completed' | 'monitoring';
}

export interface TransplantMethod {
  name: string;
  description: string;
  suitableFor: CoralType[];
  substrate: SubstrateType[];
  attachment: AttachmentMethod;
  successRate: number;
}

export interface AttachmentMethod {
  type: 'epoxy' | 'cement' | 'cable-tie' | 'wire' | 'natural' | 'plug';
  material?: string;
  curingTime?: string;
}

export interface TransplantSchedule {
  events: TransplantEvent[];
  constraints: ScheduleConstraint[];
  optimalConditions: OptimalConditions;
}

export interface TransplantEvent {
  id: string;
  date: string;
  site: string;
  species: string[];
  quantity: number;
  team: string;
  status: 'scheduled' | 'completed' | 'cancelled' | 'rescheduled';
}

export interface ScheduleConstraint {
  type: 'weather' | 'tide' | 'spawning' | 'logistics';
  description: string;
  avoidDates?: string[];
}

export interface OptimalConditions {
  temperature: { min: number; max: number };
  visibility: number;
  currentSpeed: number;
  tidalPhase?: string;
  moonPhase?: string;
}

export interface TransplantProtocol {
  preTransplant: ProtocolStep[];
  transport: TransportProtocol;
  attachment: ProtocolStep[];
  documentation: DocumentationRequirements;
}

export interface ProtocolStep {
  step: number;
  action: string;
  duration?: string;
  notes?: string;
}

export interface TransportProtocol {
  container: string;
  temperature: { min: number; max: number };
  maxDuration: string;
  handling: string[];
}

export interface DocumentationRequirements {
  preTransplant: string[];
  duringTransplant: string[];
  postTransplant: string[];
}

export interface PostTransplantCare {
  monitoring: PostTransplantMonitoring;
  maintenance: string[];
  interventions: InterventionProtocol[];
}

export interface PostTransplantMonitoring {
  frequency: string;
  duration: string;
  parameters: string[];
  photography: boolean;
}

export interface InterventionProtocol {
  trigger: string;
  action: string;
  responsible: string;
}

// ============================================================================
// Monitoring & Community
// ============================================================================

export interface MonitoringProgram {
  objectives: string[];
  indicators: MonitoringIndicator[];
  methods: MonitoringMethod[];
  schedule: MonitoringSchedule;
  dataManagement: DataManagement;
  reporting: ReportingSchedule;
}

export interface MonitoringIndicator {
  id: string;
  name: string;
  category: 'ecological' | 'restoration' | 'socioeconomic';
  unit: string;
  baseline?: number;
  target?: number;
  frequency: string;
  method: string;
}

export interface MonitoringMethod {
  name: string;
  type: 'transect' | 'quadrat' | 'photo' | 'video' | 'point-intercept' | 'fish-count';
  protocol: string;
  equipment: string[];
  personnel: number;
}

export interface MonitoringSchedule {
  regular: { frequency: string; duration: string };
  postEvent: string[];
  adaptive: string;
}

export interface DataManagement {
  platform: string;
  storage: string;
  sharing: string;
  quality: string;
  backup: string;
}

export interface ReportingSchedule {
  internal: { frequency: string; recipients: string[] };
  external: { frequency: string; recipients: string[] };
  public: { frequency: string; channels: string[] };
}

export interface CommunityProgram {
  engagement: EngagementActivities;
  education: EducationProgram;
  livelihoods: LivelihoodSupport;
  governance: CommunityGovernance;
}

export interface EngagementActivities {
  activities: CommunityActivity[];
  participationTarget: number;
  stakeholders: string[];
}

export interface CommunityActivity {
  name: string;
  type: 'meeting' | 'workshop' | 'field-visit' | 'volunteer' | 'event';
  frequency: string;
  participants: number;
  objectives: string[];
}

export interface EducationProgram {
  schools: SchoolProgram;
  publicAwareness: AwarenessProgram;
  training: TrainingProgram;
}

export interface SchoolProgram {
  schools: number;
  students: number;
  curriculum: string[];
  activities: string[];
}

export interface AwarenessProgram {
  campaigns: string[];
  materials: string[];
  reach: number;
  channels: string[];
}

export interface TrainingProgram {
  topics: string[];
  participants: number;
  certification?: boolean;
  outcomes: string[];
}

export interface LivelihoodSupport {
  opportunities: LivelihoodOpportunity[];
  beneficiaries: number;
  sustainableUse: string[];
}

export interface LivelihoodOpportunity {
  type: 'tourism' | 'fisheries' | 'handicrafts' | 'research' | 'restoration';
  description: string;
  participants: number;
  income?: Money;
}

export interface CommunityGovernance {
  structure: string;
  decisionMaking: string;
  conflictResolution: string;
  compliance: string;
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
  status: ProjectStatus;
  createdAt: string;
  updatedAt?: string;
}

export interface ValidationResult {
  valid: boolean;
  errors?: ValidationError[];
}

export interface ValidationError {
  path: string;
  message: string;
  value?: unknown;
}

export interface PaginatedResponse<T> {
  data: T[];
  pagination: {
    total: number;
    limit: number;
    offset: number;
    hasMore: boolean;
  };
}
