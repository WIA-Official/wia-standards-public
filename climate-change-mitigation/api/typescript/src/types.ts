/**
 * WIA Climate Change Mitigation Standard - TypeScript Type Definitions
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Mitigation Types
// ============================================================================

export interface WIAMitigationProject {
  standard: 'WIA-CLIMATE-CHANGE-MITIGATION';
  version: string;
  metadata: ProjectMetadata;
  baseline: BaselineAssessment;
  interventions: MitigationIntervention[];
  monitoring: MonitoringPlan;
  verification: VerificationPlan;
  reporting: ReportingConfiguration;
  extensions?: Record<string, unknown>;
}

export interface ProjectMetadata {
  id: string;
  name: string;
  description?: string;
  type: MitigationProjectType;
  sector: EmissionSector;
  location: ProjectLocation;
  timeframe: ProjectTimeframe;
  developer: Organization;
  stakeholders?: Stakeholder[];
  status: ProjectStatus;
}

export type MitigationProjectType =
  | 'renewable-energy'
  | 'energy-efficiency'
  | 'carbon-capture'
  | 'afforestation'
  | 'reforestation'
  | 'avoided-deforestation'
  | 'methane-capture'
  | 'industrial-process'
  | 'transportation'
  | 'waste-management'
  | 'agriculture'
  | 'blue-carbon';

export type EmissionSector =
  | 'energy'
  | 'industry'
  | 'transport'
  | 'buildings'
  | 'agriculture'
  | 'forestry'
  | 'waste'
  | 'cross-sector';

export interface ProjectLocation {
  country: string;
  region?: string;
  coordinates?: Coordinates;
  area?: ProjectArea;
  jurisdiction?: string;
}

export interface Coordinates {
  latitude: number;
  longitude: number;
}

export interface ProjectArea {
  value: number;
  unit: 'hectares' | 'km2' | 'acres';
  boundaries?: GeoJSON;
}

export interface GeoJSON {
  type: 'Polygon' | 'MultiPolygon';
  coordinates: number[][][];
}

export interface ProjectTimeframe {
  startDate: string;
  endDate?: string;
  creditingPeriod: CreditingPeriod;
}

export interface CreditingPeriod {
  start: string;
  end: string;
  renewable: boolean;
  renewalTerms?: string;
}

export interface Organization {
  name: string;
  type: 'company' | 'government' | 'ngo' | 'community';
  country: string;
  registrationNumber?: string;
  contact?: ContactInfo;
}

export interface ContactInfo {
  name: string;
  email: string;
  phone?: string;
}

export interface Stakeholder {
  name: string;
  role: StakeholderRole;
  organization?: string;
}

export type StakeholderRole =
  | 'developer'
  | 'validator'
  | 'verifier'
  | 'financier'
  | 'buyer'
  | 'local-community'
  | 'government';

export type ProjectStatus =
  | 'concept'
  | 'design'
  | 'validation'
  | 'registered'
  | 'operational'
  | 'verification'
  | 'completed'
  | 'suspended';

// ============================================================================
// Baseline Assessment
// ============================================================================

export interface BaselineAssessment {
  scenario: BaselineScenario;
  emissions: EmissionInventory;
  methodology: Methodology;
  additionality: AdditionalityAssessment;
}

export interface BaselineScenario {
  description: string;
  assumptions: Assumption[];
  dataYear: number;
  projectionMethod: string;
}

export interface Assumption {
  id: string;
  description: string;
  value?: string;
  source?: string;
  validated: boolean;
}

export interface EmissionInventory {
  scope1: EmissionSource[];
  scope2?: EmissionSource[];
  scope3?: EmissionSource[];
  total: EmissionTotal;
}

export interface EmissionSource {
  id: string;
  name: string;
  category: string;
  gas: GreenhouseGas;
  quantity: number;
  unit: EmissionUnit;
  emissionFactor?: EmissionFactor;
  uncertainty?: number;
}

export type GreenhouseGas =
  | 'CO2'
  | 'CH4'
  | 'N2O'
  | 'HFCs'
  | 'PFCs'
  | 'SF6'
  | 'NF3';

export type EmissionUnit =
  | 'tCO2e'
  | 'tCO2'
  | 'tCH4'
  | 'tN2O'
  | 'kg'
  | 'tonnes';

export interface EmissionFactor {
  value: number;
  unit: string;
  source: string;
  year: number;
}

export interface EmissionTotal {
  value: number;
  unit: 'tCO2e';
  breakdown: Record<string, number>;
}

export interface Methodology {
  name: string;
  version: string;
  registry: string;
  applicability: string[];
  baselineApproach: string;
  quantificationMethod: string;
}

export interface AdditionalityAssessment {
  type: AdditionalityType;
  barriers?: Barrier[];
  commonPractice?: CommonPracticeAnalysis;
  investment?: InvestmentAnalysis;
  conclusion: string;
}

export type AdditionalityType =
  | 'barrier-analysis'
  | 'common-practice'
  | 'investment-analysis'
  | 'combined';

export interface Barrier {
  type: 'financial' | 'technological' | 'institutional' | 'social';
  description: string;
  evidence: string;
}

export interface CommonPracticeAnalysis {
  region: string;
  penetrationRate: number;
  threshold: number;
  isAdditional: boolean;
}

export interface InvestmentAnalysis {
  benchmark: string;
  projectIRR: number;
  benchmarkIRR: number;
  isAdditional: boolean;
}

// ============================================================================
// Interventions
// ============================================================================

export interface MitigationIntervention {
  id: string;
  name: string;
  type: InterventionType;
  technology?: TechnologyDetails;
  implementation: ImplementationPlan;
  expectedReduction: EmissionReduction;
  cobenefits?: Cobenefit[];
  risks: Risk[];
}

export type InterventionType =
  | 'technology-deployment'
  | 'fuel-switching'
  | 'process-improvement'
  | 'carbon-sequestration'
  | 'emission-avoidance'
  | 'demand-reduction';

export interface TechnologyDetails {
  name: string;
  category: string;
  capacity?: number;
  capacityUnit?: string;
  efficiency?: number;
  lifetime: number;
  manufacturer?: string;
}

export interface ImplementationPlan {
  phases: Phase[];
  milestones: Milestone[];
  budget: Budget;
  timeline: Timeline;
}

export interface Phase {
  id: string;
  name: string;
  description: string;
  start: string;
  end: string;
  deliverables: string[];
}

export interface Milestone {
  id: string;
  name: string;
  date: string;
  criteria: string;
  status: 'pending' | 'achieved' | 'delayed';
}

export interface Budget {
  total: Money;
  breakdown: BudgetItem[];
  funding: FundingSource[];
}

export interface Money {
  amount: number;
  currency: string;
}

export interface BudgetItem {
  category: string;
  amount: Money;
  description?: string;
}

export interface FundingSource {
  name: string;
  type: 'equity' | 'debt' | 'grant' | 'carbon-finance';
  amount: Money;
  terms?: string;
}

export interface Timeline {
  start: string;
  end: string;
  phases: string[];
}

export interface EmissionReduction {
  annual: number;
  total: number;
  unit: 'tCO2e';
  uncertainty: number;
  methodology: string;
}

export interface Cobenefit {
  type: CobenfitType;
  description: string;
  indicator?: string;
  measurement?: string;
  sdgAlignment?: number[];
}

export type CobenfitType =
  | 'biodiversity'
  | 'water'
  | 'air-quality'
  | 'employment'
  | 'health'
  | 'energy-access'
  | 'gender-equality'
  | 'community-development';

export interface Risk {
  id: string;
  category: RiskCategory;
  description: string;
  probability: 'low' | 'medium' | 'high';
  impact: 'low' | 'medium' | 'high';
  mitigation: string;
}

export type RiskCategory =
  | 'technical'
  | 'financial'
  | 'regulatory'
  | 'operational'
  | 'market'
  | 'environmental'
  | 'social';

// ============================================================================
// Monitoring & Verification
// ============================================================================

export interface MonitoringPlan {
  parameters: MonitoringParameter[];
  procedures: MonitoringProcedure[];
  dataManagement: DataManagement;
  qualityControl: QualityControl;
}

export interface MonitoringParameter {
  id: string;
  name: string;
  unit: string;
  frequency: MonitoringFrequency;
  method: string;
  equipment?: string;
  responsible: string;
  dataSource: string;
}

export type MonitoringFrequency =
  | 'continuous'
  | 'hourly'
  | 'daily'
  | 'weekly'
  | 'monthly'
  | 'quarterly'
  | 'annually';

export interface MonitoringProcedure {
  id: string;
  parameter: string;
  steps: string[];
  equipment: string[];
  calibration?: CalibrationRequirement;
}

export interface CalibrationRequirement {
  frequency: string;
  method: string;
  tolerance: number;
  records: string;
}

export interface DataManagement {
  storage: string;
  backup: string;
  retention: number;
  security: string;
  access: AccessControl[];
}

export interface AccessControl {
  role: string;
  permissions: string[];
}

export interface QualityControl {
  procedures: QCProcedure[];
  audits: AuditSchedule;
}

export interface QCProcedure {
  type: string;
  frequency: string;
  responsible: string;
  documentation: string;
}

export interface AuditSchedule {
  internal: { frequency: string; scope: string };
  external?: { frequency: string; auditor: string };
}

export interface VerificationPlan {
  verifier: VerifierInfo;
  schedule: VerificationSchedule;
  scope: VerificationScope;
  standards: string[];
}

export interface VerifierInfo {
  name: string;
  accreditation: string;
  country: string;
  experience?: string;
}

export interface VerificationSchedule {
  frequency: 'annual' | 'biennial' | 'periodic';
  firstVerification: string;
  reportingDeadline: string;
}

export interface VerificationScope {
  boundaries: string;
  gases: GreenhouseGas[];
  sources: string[];
  sinks?: string[];
}

// ============================================================================
// Reporting & API Types
// ============================================================================

export interface ReportingConfiguration {
  templates: ReportTemplate[];
  schedule: ReportingSchedule;
  recipients: Recipient[];
}

export interface ReportTemplate {
  id: string;
  name: string;
  type: ReportType;
  format: 'pdf' | 'xlsx' | 'json';
  sections: string[];
}

export type ReportType =
  | 'monitoring'
  | 'verification'
  | 'annual'
  | 'issuance'
  | 'stakeholder';

export interface ReportingSchedule {
  monitoringReports: string;
  verificationReports: string;
  annualReports: string;
}

export interface Recipient {
  name: string;
  email: string;
  reports: string[];
}

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

export interface CreditIssuance {
  id: string;
  projectId: string;
  vintage: number;
  quantity: number;
  unit: 'tCO2e';
  serialNumbers: string;
  status: 'pending' | 'issued' | 'retired' | 'cancelled';
  issuanceDate?: string;
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
