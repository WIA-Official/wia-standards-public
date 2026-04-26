/**
 * WIA Climate Refugee Standard - TypeScript Type Definitions
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Climate Refugee Types
// ============================================================================

export interface WIAClimateRefugeeProject {
  standard: 'WIA-CLIMATE-REFUGEE';
  version: string;
  metadata: ProjectMetadata;
  assessment: ClimateImpactAssessment;
  population: AffectedPopulation;
  displacement: DisplacementPlan;
  protection: ProtectionMeasures;
  assistance: AssistanceProgram;
  resettlement: ResettlementPlan;
  monitoring: MonitoringFramework;
  extensions?: Record<string, unknown>;
}

export interface ProjectMetadata {
  id: string;
  name: string;
  description?: string;
  type: ProjectType;
  scope: ProjectScope;
  region: GeographicRegion;
  timeframe: TimeFrame;
  leadOrganization: Organization;
  partners?: Organization[];
  status: ProjectStatus;
}

export type ProjectType =
  | 'emergency-response'
  | 'planned-relocation'
  | 'protection-program'
  | 'adaptation-support'
  | 'research'
  | 'policy-advocacy';

export type ProjectScope =
  | 'local'
  | 'national'
  | 'regional'
  | 'international';

export interface GeographicRegion {
  name: string;
  country: string;
  subRegions?: string[];
  coordinates?: Coordinates;
  affectedArea?: AffectedArea;
}

export interface Coordinates {
  latitude: number;
  longitude: number;
}

export interface AffectedArea {
  value: number;
  unit: 'km2' | 'hectares';
  boundaries?: GeoJSON;
}

export interface GeoJSON {
  type: 'Polygon' | 'MultiPolygon';
  coordinates: number[][][];
}

export interface TimeFrame {
  start: string;
  end?: string;
  phases?: Phase[];
}

export interface Phase {
  id: string;
  name: string;
  start: string;
  end: string;
  status: 'planned' | 'active' | 'completed';
}

export interface Organization {
  name: string;
  type: OrganizationType;
  country: string;
  role?: string;
  contact?: ContactInfo;
}

export type OrganizationType =
  | 'government'
  | 'un-agency'
  | 'ngo'
  | 'ingo'
  | 'academic'
  | 'community-based';

export interface ContactInfo {
  name: string;
  email: string;
  phone?: string;
  position?: string;
}

export type ProjectStatus =
  | 'planning'
  | 'assessment'
  | 'implementation'
  | 'monitoring'
  | 'completed'
  | 'suspended';

// ============================================================================
// Climate Impact Assessment
// ============================================================================

export interface ClimateImpactAssessment {
  hazards: ClimateHazard[];
  vulnerabilities: Vulnerability[];
  impacts: Impact[];
  projections: ClimateProjection[];
  riskLevel: RiskLevel;
}

export interface ClimateHazard {
  id: string;
  type: HazardType;
  severity: 'low' | 'medium' | 'high' | 'extreme';
  frequency: HazardFrequency;
  onset: 'sudden' | 'slow';
  description: string;
  historicalData?: HistoricalEvent[];
}

export type HazardType =
  | 'sea-level-rise'
  | 'flooding'
  | 'drought'
  | 'desertification'
  | 'extreme-heat'
  | 'cyclone'
  | 'wildfire'
  | 'landslide'
  | 'erosion'
  | 'salinization'
  | 'glacier-melt';

export interface HazardFrequency {
  value: number;
  unit: 'per-year' | 'per-decade' | 'years-between';
  trend: 'increasing' | 'stable' | 'decreasing';
}

export interface HistoricalEvent {
  date: string;
  description: string;
  affectedPeople?: number;
  displacement?: number;
  damages?: number;
}

export interface Vulnerability {
  id: string;
  type: VulnerabilityType;
  level: 'low' | 'medium' | 'high' | 'critical';
  factors: VulnerabilityFactor[];
  affectedGroups: string[];
}

export type VulnerabilityType =
  | 'environmental'
  | 'social'
  | 'economic'
  | 'infrastructural'
  | 'institutional';

export interface VulnerabilityFactor {
  name: string;
  description: string;
  indicator?: string;
  value?: number;
}

export interface Impact {
  id: string;
  type: ImpactType;
  severity: 'minor' | 'moderate' | 'severe' | 'catastrophic';
  description: string;
  affectedPopulation?: number;
  economicLoss?: Money;
  timeframe: 'immediate' | 'short-term' | 'medium-term' | 'long-term';
}

export type ImpactType =
  | 'displacement'
  | 'livelihood-loss'
  | 'food-insecurity'
  | 'water-scarcity'
  | 'health'
  | 'infrastructure-damage'
  | 'ecosystem-loss'
  | 'conflict';

export interface Money {
  amount: number;
  currency: string;
}

export interface ClimateProjection {
  scenario: string;
  timeHorizon: string;
  projectedChange: string;
  confidence: 'low' | 'medium' | 'high';
  source: string;
  implications: string[];
}

export type RiskLevel = 'minimal' | 'low' | 'moderate' | 'high' | 'very-high' | 'extreme';

// ============================================================================
// Affected Population
// ============================================================================

export interface AffectedPopulation {
  total: number;
  categories: PopulationCategory[];
  vulnerableGroups: VulnerableGroup[];
  demographics: Demographics;
  registration?: RegistrationSystem;
}

export interface PopulationCategory {
  name: string;
  count: number;
  percentage: number;
  description?: string;
}

export interface VulnerableGroup {
  type: VulnerableGroupType;
  count: number;
  specificNeeds: string[];
  priorityLevel: 'high' | 'medium' | 'normal';
}

export type VulnerableGroupType =
  | 'children'
  | 'elderly'
  | 'women'
  | 'disabled'
  | 'chronic-illness'
  | 'unaccompanied-minors'
  | 'single-headed-households'
  | 'indigenous'
  | 'stateless';

export interface Demographics {
  ageDistribution: AgeGroup[];
  genderRatio: { male: number; female: number; other?: number };
  householdSize: number;
  urbanRural: { urban: number; rural: number };
}

export interface AgeGroup {
  range: string;
  count: number;
  percentage: number;
}

export interface RegistrationSystem {
  type: 'biometric' | 'paper-based' | 'digital' | 'hybrid';
  coverage: number;
  lastUpdated: string;
  dataProtection: string;
}

// ============================================================================
// Displacement Plan
// ============================================================================

export interface DisplacementPlan {
  type: DisplacementType;
  triggers: DisplacementTrigger[];
  scenarios: DisplacementScenario[];
  routes: EvacuationRoute[];
  temporaryShelter: ShelterPlan;
  returnPlan?: ReturnPlan;
}

export type DisplacementType =
  | 'voluntary'
  | 'forced'
  | 'preventive'
  | 'planned-relocation';

export interface DisplacementTrigger {
  type: string;
  threshold: string;
  indicator: string;
  monitoringFrequency: string;
}

export interface DisplacementScenario {
  id: string;
  name: string;
  probability: 'low' | 'medium' | 'high';
  affectedPopulation: number;
  timeline: string;
  response: string[];
}

export interface EvacuationRoute {
  id: string;
  name: string;
  origin: string;
  destination: string;
  distance: number;
  transportMode: TransportMode[];
  capacity: number;
  status: 'primary' | 'alternative' | 'emergency';
}

export type TransportMode =
  | 'foot'
  | 'vehicle'
  | 'boat'
  | 'air'
  | 'public-transport';

export interface ShelterPlan {
  sites: ShelterSite[];
  totalCapacity: number;
  standards: string;
  duration: string;
}

export interface ShelterSite {
  id: string;
  name: string;
  location: Coordinates;
  capacity: number;
  type: 'camp' | 'collective-center' | 'host-community' | 'emergency';
  facilities: string[];
  status: 'available' | 'active' | 'full' | 'closed';
}

export interface ReturnPlan {
  conditions: string[];
  timeline?: string;
  support: string[];
  monitoring: string;
}

// ============================================================================
// Protection & Assistance
// ============================================================================

export interface ProtectionMeasures {
  legalFramework: LegalFramework;
  rights: RightsProtection[];
  mechanisms: ProtectionMechanism[];
  documentation: DocumentationServices;
  safetyMeasures: SafetyMeasure[];
}

export interface LegalFramework {
  nationalLaws: string[];
  internationalAgreements: string[];
  gaps: string[];
  recommendations: string[];
}

export interface RightsProtection {
  right: string;
  status: 'protected' | 'at-risk' | 'violated';
  measures: string[];
  responsible: string;
}

export interface ProtectionMechanism {
  type: string;
  description: string;
  coverage: number;
  effectiveness: 'high' | 'medium' | 'low';
}

export interface DocumentationServices {
  types: DocumentType[];
  accessPoints: number;
  processingTime: string;
  challenges: string[];
}

export type DocumentType =
  | 'identity'
  | 'registration'
  | 'travel'
  | 'land-tenure'
  | 'birth-certificate';

export interface SafetyMeasure {
  type: string;
  target: string;
  implementation: string;
  monitoring: string;
}

export interface AssistanceProgram {
  sectors: AssistanceSector[];
  delivery: DeliveryMechanism[];
  targeting: TargetingCriteria;
  budget: Budget;
  timeline: string;
}

export interface AssistanceSector {
  name: SectorType;
  activities: Activity[];
  beneficiaries: number;
  coverage: number;
  partners: string[];
}

export type SectorType =
  | 'food'
  | 'shelter'
  | 'health'
  | 'education'
  | 'water-sanitation'
  | 'livelihoods'
  | 'protection'
  | 'cash-assistance';

export interface Activity {
  id: string;
  name: string;
  description: string;
  startDate: string;
  endDate?: string;
  status: 'planned' | 'ongoing' | 'completed';
  output?: string;
}

export interface DeliveryMechanism {
  type: 'in-kind' | 'cash' | 'voucher' | 'service';
  modality: string;
  frequency: string;
  value?: Money;
}

export interface TargetingCriteria {
  approach: 'geographic' | 'vulnerability' | 'categorical' | 'community-based';
  criteria: string[];
  verification: string;
  appeals: string;
}

export interface Budget {
  total: Money;
  breakdown: BudgetItem[];
  funded: number;
  gap: Money;
  sources: FundingSource[];
}

export interface BudgetItem {
  category: string;
  amount: Money;
  percentage: number;
}

export interface FundingSource {
  name: string;
  type: 'government' | 'un' | 'bilateral' | 'private' | 'foundation';
  amount: Money;
  status: 'committed' | 'pledged' | 'received';
}

// ============================================================================
// Resettlement & Monitoring
// ============================================================================

export interface ResettlementPlan {
  type: ResettlementType;
  sites: ResettlementSite[];
  criteria: SelectionCriteria;
  support: ResettlementSupport;
  integration: IntegrationPlan;
  timeline: ResettlementTimeline;
}

export type ResettlementType =
  | 'internal'
  | 'cross-border'
  | 'third-country'
  | 'return';

export interface ResettlementSite {
  id: string;
  name: string;
  location: GeographicRegion;
  capacity: number;
  infrastructure: string[];
  services: string[];
  climate: ClimateAssessment;
  status: 'proposed' | 'approved' | 'developing' | 'operational';
}

export interface ClimateAssessment {
  riskLevel: RiskLevel;
  projectedChanges: string[];
  adaptationMeasures: string[];
}

export interface SelectionCriteria {
  priorityGroups: string[];
  eligibility: string[];
  process: string;
  timeline: string;
}

export interface ResettlementSupport {
  package: SupportPackage;
  duration: string;
  phaseOut: string;
}

export interface SupportPackage {
  housing: string;
  livelihood: string;
  integration: string;
  value?: Money;
}

export interface IntegrationPlan {
  hostCommunity: HostCommunityEngagement;
  services: ServiceAccess[];
  livelihoods: LivelihoodSupport;
  social: SocialIntegration;
}

export interface HostCommunityEngagement {
  consultations: string;
  benefits: string[];
  conflictPrevention: string;
}

export interface ServiceAccess {
  service: string;
  access: 'full' | 'partial' | 'limited';
  barriers?: string[];
  solutions?: string[];
}

export interface LivelihoodSupport {
  programs: string[];
  training: string[];
  employment: string;
  entrepreneurship: string;
}

export interface SocialIntegration {
  language: string;
  cultural: string;
  community: string;
  timeline: string;
}

export interface ResettlementTimeline {
  phases: ResettlementPhase[];
  milestones: Milestone[];
}

export interface ResettlementPhase {
  name: string;
  start: string;
  end: string;
  activities: string[];
  targets: number;
}

export interface Milestone {
  date: string;
  description: string;
  status: 'pending' | 'achieved' | 'delayed';
}

export interface MonitoringFramework {
  indicators: Indicator[];
  dataCollection: DataCollection;
  reporting: ReportingSchedule;
  evaluation: EvaluationPlan;
}

export interface Indicator {
  id: string;
  name: string;
  category: string;
  baseline: number;
  target: number;
  current?: number;
  unit: string;
  frequency: string;
  source: string;
}

export interface DataCollection {
  methods: string[];
  frequency: string;
  responsibility: string;
  tools: string[];
}

export interface ReportingSchedule {
  internal: string;
  external: string;
  formats: string[];
  recipients: string[];
}

export interface EvaluationPlan {
  type: 'formative' | 'summative' | 'both';
  frequency: string;
  methodology: string;
  independence: boolean;
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
