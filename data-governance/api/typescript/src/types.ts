/**
 * WIA Data Governance Standard - TypeScript Type Definitions
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Data Governance Types
// ============================================================================

export interface WIADataGovernanceProject {
  standard: 'WIA-DATA-GOVERNANCE';
  version: string;
  metadata: ProjectMetadata;
  framework: GovernanceFramework;
  dataAssets: DataAssetManagement;
  policies: PolicyManagement;
  stewardship: StewardshipProgram;
  quality: DataQualityProgram;
  privacy: PrivacyManagement;
  compliance: ComplianceManagement;
  metrics: MetricsProgram;
  extensions?: Record<string, unknown>;
}

export interface ProjectMetadata {
  id: string;
  name: string;
  description?: string;
  organization: Organization;
  scope: GovernanceScope;
  createdAt: string;
  updatedAt?: string;
  status: ProjectStatus;
}

export interface Organization {
  name: string;
  industry: string;
  size: 'small' | 'medium' | 'large' | 'enterprise';
  regions: string[];
  contact: ContactInfo;
}

export interface ContactInfo {
  name: string;
  email: string;
  role: string;
}

export interface GovernanceScope {
  domains: string[];
  systems: string[];
  dataTypes: string[];
  exclusions?: string[];
}

export type ProjectStatus = 'active' | 'planning' | 'implementation' | 'review';

// ============================================================================
// Governance Framework Types
// ============================================================================

export interface GovernanceFramework {
  model: GovernanceModel;
  structure: OrganizationalStructure;
  processes: GovernanceProcess[];
  decisionRights: DecisionRights;
  maturity: MaturityAssessment;
}

export interface GovernanceModel {
  type: 'centralized' | 'federated' | 'hybrid';
  principles: GovernancePrinciple[];
  objectives: string[];
  charter: string;
}

export interface GovernancePrinciple {
  id: string;
  name: string;
  description: string;
  rationale: string;
  implications: string[];
}

export interface OrganizationalStructure {
  council: GovernanceCouncil;
  committees: Committee[];
  roles: GovernanceRole[];
  raci: RACIMatrix;
}

export interface GovernanceCouncil {
  name: string;
  charter: string;
  members: CouncilMember[];
  meetingFrequency: string;
  decisionProcess: string;
}

export interface CouncilMember {
  name: string;
  title: string;
  role: string;
  votingRights: boolean;
}

export interface Committee {
  id: string;
  name: string;
  purpose: string;
  members: string[];
  reportingTo: string;
  meetingFrequency: string;
}

export interface GovernanceRole {
  id: string;
  name: string;
  type: 'executive' | 'steward' | 'custodian' | 'owner' | 'user';
  responsibilities: string[];
  authorities: string[];
  qualifications: string[];
}

export interface RACIMatrix {
  activities: RACIActivity[];
  roles: string[];
}

export interface RACIActivity {
  activity: string;
  assignments: { role: string; type: 'R' | 'A' | 'C' | 'I' }[];
}

export interface GovernanceProcess {
  id: string;
  name: string;
  description: string;
  owner: string;
  steps: ProcessStep[];
  inputs: string[];
  outputs: string[];
  metrics: string[];
}

export interface ProcessStep {
  order: number;
  name: string;
  description: string;
  responsible: string;
  tools?: string[];
  sla?: string;
}

export interface DecisionRights {
  categories: DecisionCategory[];
  escalation: EscalationPath;
  delegation: DelegationRules;
}

export interface DecisionCategory {
  category: string;
  level: 'operational' | 'tactical' | 'strategic';
  authority: string;
  criteria: string[];
}

export interface EscalationPath {
  levels: EscalationLevel[];
  timeframes: { severity: string; maxTime: string }[];
}

export interface EscalationLevel {
  level: number;
  authority: string;
  criteria: string;
}

export interface DelegationRules {
  allowed: boolean;
  conditions: string[];
  documentation: boolean;
}

export interface MaturityAssessment {
  currentLevel: 1 | 2 | 3 | 4 | 5;
  targetLevel: 1 | 2 | 3 | 4 | 5;
  dimensions: MaturityDimension[];
  roadmap: MaturityRoadmap;
}

export interface MaturityDimension {
  name: string;
  currentScore: number;
  targetScore: number;
  gaps: string[];
}

export interface MaturityRoadmap {
  phases: RoadmapPhase[];
  timeline: string;
}

export interface RoadmapPhase {
  phase: number;
  name: string;
  objectives: string[];
  initiatives: string[];
  duration: string;
}

// ============================================================================
// Data Asset Management Types
// ============================================================================

export interface DataAssetManagement {
  inventory: AssetInventory;
  classification: ClassificationScheme;
  lifecycle: DataLifecycle;
  lineage: LineageConfig;
}

export interface AssetInventory {
  sources: DataSource[];
  assets: DataAsset[];
  cataloging: CatalogingConfig;
}

export interface DataSource {
  id: string;
  name: string;
  type: 'database' | 'file' | 'api' | 'stream' | 'external';
  system: string;
  owner: string;
  criticality: 'low' | 'medium' | 'high' | 'critical';
}

export interface DataAsset {
  id: string;
  name: string;
  description?: string;
  source: string;
  domain: string;
  owner: string;
  steward: string;
  classification: string;
  sensitivity: string;
  tags: string[];
  metadata: Record<string, unknown>;
}

export interface CatalogingConfig {
  automation: boolean;
  frequency: string;
  attributes: string[];
  validation: boolean;
}

export interface ClassificationScheme {
  levels: ClassificationLevel[];
  criteria: ClassificationCriterion[];
  defaultLevel: string;
  reviewFrequency: string;
}

export interface ClassificationLevel {
  id: string;
  name: string;
  description: string;
  handling: HandlingRequirements;
  examples: string[];
}

export interface HandlingRequirements {
  storage: string[];
  transmission: string[];
  access: string[];
  retention: string;
  disposal: string;
}

export interface ClassificationCriterion {
  id: string;
  name: string;
  questions: string[];
  scoring: { answer: string; points: number }[];
}

export interface DataLifecycle {
  stages: LifecycleStage[];
  retention: RetentionPolicy[];
  archival: ArchivalPolicy;
  disposal: DisposalPolicy;
}

export interface LifecycleStage {
  name: string;
  description: string;
  duration?: string;
  requirements: string[];
  nextStage?: string;
}

export interface RetentionPolicy {
  id: string;
  name: string;
  classification: string;
  period: string;
  legalBasis: string;
  exceptions: string[];
}

export interface ArchivalPolicy {
  criteria: string[];
  format: string;
  location: string;
  accessibility: string;
}

export interface DisposalPolicy {
  methods: DisposalMethod[];
  approval: string;
  verification: boolean;
  documentation: boolean;
}

export interface DisposalMethod {
  classification: string;
  method: 'delete' | 'overwrite' | 'crypto-erase' | 'physical';
  verification: string;
}

export interface LineageConfig {
  enabled: boolean;
  granularity: 'system' | 'table' | 'column';
  sources: string[];
  visualization: boolean;
}

// ============================================================================
// Policy Management Types
// ============================================================================

export interface PolicyManagement {
  framework: PolicyFramework;
  policies: Policy[];
  standards: Standard[];
  procedures: Procedure[];
  guidelines: Guideline[];
}

export interface PolicyFramework {
  hierarchy: PolicyHierarchy;
  lifecycle: PolicyLifecycle;
  enforcement: EnforcementConfig;
}

export interface PolicyHierarchy {
  levels: { level: number; type: string; approver: string }[];
  inheritance: boolean;
}

export interface PolicyLifecycle {
  stages: ('draft' | 'review' | 'approved' | 'published' | 'retired')[];
  reviewFrequency: string;
  versionControl: boolean;
}

export interface EnforcementConfig {
  methods: ('technical' | 'procedural' | 'audit')[];
  exceptions: ExceptionProcess;
  violations: ViolationHandling;
}

export interface ExceptionProcess {
  allowed: boolean;
  approvers: string[];
  documentation: string[];
  maxDuration: string;
}

export interface ViolationHandling {
  detection: string[];
  notification: string[];
  remediation: string;
  consequences: string[];
}

export interface Policy {
  id: string;
  name: string;
  type: 'access' | 'usage' | 'quality' | 'privacy' | 'retention' | 'security';
  scope: string[];
  statement: string;
  rationale: string;
  owner: string;
  approver: string;
  effectiveDate: string;
  reviewDate: string;
  version: string;
  status: 'draft' | 'active' | 'retired';
}

export interface Standard {
  id: string;
  name: string;
  policy: string;
  requirements: Requirement[];
  compliance: ComplianceCriteria;
}

export interface Requirement {
  id: string;
  description: string;
  mandatory: boolean;
  evidence: string[];
}

export interface ComplianceCriteria {
  metrics: string[];
  threshold: number;
  assessment: string;
}

export interface Procedure {
  id: string;
  name: string;
  standard: string;
  steps: ProcedureStep[];
  roles: string[];
  tools: string[];
}

export interface ProcedureStep {
  order: number;
  action: string;
  responsible: string;
  inputs?: string[];
  outputs?: string[];
}

export interface Guideline {
  id: string;
  name: string;
  context: string;
  recommendations: string[];
  examples: string[];
}

// ============================================================================
// Stewardship & Quality Types
// ============================================================================

export interface StewardshipProgram {
  model: StewardshipModel;
  stewards: DataSteward[];
  responsibilities: StewardResponsibility[];
  training: TrainingProgram;
}

export interface StewardshipModel {
  type: 'domain' | 'enterprise' | 'hybrid';
  reporting: string;
  authority: string[];
}

export interface DataSteward {
  id: string;
  name: string;
  domain: string;
  assets: string[];
  backup?: string;
  contact: ContactInfo;
}

export interface StewardResponsibility {
  area: string;
  tasks: string[];
  frequency: string;
  metrics: string[];
}

export interface TrainingProgram {
  courses: TrainingCourse[];
  certification: boolean;
  refresher: string;
}

export interface TrainingCourse {
  id: string;
  name: string;
  type: 'mandatory' | 'optional';
  duration: string;
  audience: string[];
}

export interface DataQualityProgram {
  dimensions: QualityDimension[];
  rules: QualityRule[];
  monitoring: QualityMonitoring;
  improvement: ImprovementProcess;
}

export interface QualityDimension {
  id: string;
  name: string;
  description: string;
  metrics: QualityMetric[];
  weight: number;
}

export interface QualityMetric {
  id: string;
  name: string;
  formula: string;
  target: number;
  threshold: number;
}

export interface QualityRule {
  id: string;
  name: string;
  dimension: string;
  type: 'completeness' | 'accuracy' | 'consistency' | 'timeliness' | 'validity' | 'uniqueness';
  definition: string;
  scope: string[];
  severity: 'low' | 'medium' | 'high' | 'critical';
}

export interface QualityMonitoring {
  frequency: string;
  automation: boolean;
  dashboards: string[];
  alerts: QualityAlert[];
}

export interface QualityAlert {
  condition: string;
  severity: string;
  notification: string[];
}

export interface ImprovementProcess {
  identification: string[];
  prioritization: string;
  remediation: string;
  tracking: boolean;
}

// ============================================================================
// Privacy & Compliance Types
// ============================================================================

export interface PrivacyManagement {
  principles: PrivacyPrinciple[];
  rights: DataSubjectRights;
  consent: ConsentManagement;
  impact: PrivacyImpact;
  incidents: IncidentManagement;
}

export interface PrivacyPrinciple {
  id: string;
  name: string;
  description: string;
  implementation: string[];
}

export interface DataSubjectRights {
  rights: SubjectRight[];
  processes: RightsProcess[];
  sla: string;
}

export interface SubjectRight {
  right: 'access' | 'rectification' | 'erasure' | 'portability' | 'objection' | 'restriction';
  applicable: boolean;
  process: string;
}

export interface RightsProcess {
  right: string;
  steps: string[];
  timeline: string;
  verification: string;
}

export interface ConsentManagement {
  purposes: ConsentPurpose[];
  collection: CollectionMethod[];
  withdrawal: WithdrawalProcess;
  records: ConsentRecords;
}

export interface ConsentPurpose {
  id: string;
  name: string;
  description: string;
  legalBasis: string;
  required: boolean;
}

export interface CollectionMethod {
  method: string;
  format: string;
  storage: string;
}

export interface WithdrawalProcess {
  methods: string[];
  timeline: string;
  impact: string;
}

export interface ConsentRecords {
  retention: string;
  attributes: string[];
  audit: boolean;
}

export interface PrivacyImpact {
  triggers: string[];
  process: PIAProcess;
  documentation: string;
}

export interface PIAProcess {
  steps: string[];
  stakeholders: string[];
  approval: string;
}

export interface IncidentManagement {
  classification: IncidentClassification[];
  response: IncidentResponse;
  notification: NotificationRequirements;
}

export interface IncidentClassification {
  severity: string;
  criteria: string[];
  escalation: string;
}

export interface IncidentResponse {
  team: string[];
  procedures: string[];
  timeline: string;
}

export interface NotificationRequirements {
  authorities: { authority: string; timeline: string }[];
  subjects: { criteria: string; timeline: string; content: string[] };
}

export interface ComplianceManagement {
  regulations: Regulation[];
  controls: Control[];
  audits: AuditProgram;
  reporting: ComplianceReporting;
}

export interface Regulation {
  id: string;
  name: string;
  jurisdiction: string;
  requirements: string[];
  applicability: string[];
}

export interface Control {
  id: string;
  name: string;
  type: 'preventive' | 'detective' | 'corrective';
  regulation: string[];
  implementation: string;
  evidence: string[];
  status: 'implemented' | 'partial' | 'planned';
}

export interface AuditProgram {
  internal: { frequency: string; scope: string[] };
  external: { frequency: string; auditor: string };
  findings: FindingManagement;
}

export interface FindingManagement {
  classification: string[];
  remediation: string;
  tracking: boolean;
}

export interface ComplianceReporting {
  frequency: string;
  recipients: string[];
  content: string[];
}

// ============================================================================
// Metrics Program Types
// ============================================================================

export interface MetricsProgram {
  kpis: KPI[];
  dashboards: Dashboard[];
  reporting: MetricsReporting;
  benchmarking: BenchmarkingConfig;
}

export interface KPI {
  id: string;
  name: string;
  category: string;
  formula: string;
  target: number;
  frequency: string;
  owner: string;
}

export interface Dashboard {
  id: string;
  name: string;
  audience: string[];
  widgets: Widget[];
  refresh: string;
}

export interface Widget {
  type: 'chart' | 'gauge' | 'table' | 'scorecard';
  metric: string;
  visualization: Record<string, unknown>;
}

export interface MetricsReporting {
  scheduled: ScheduledReport[];
  adhoc: boolean;
  distribution: string[];
}

export interface ScheduledReport {
  name: string;
  frequency: string;
  content: string[];
  recipients: string[];
}

export interface BenchmarkingConfig {
  enabled: boolean;
  sources: string[];
  frequency: string;
  dimensions: string[];
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
