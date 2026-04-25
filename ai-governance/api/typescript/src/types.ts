/**
 * WIA-AI-020: AI Governance Standard - TypeScript Type Definitions
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Core Types
// ============================================================================

export enum RiskLevel {
  CRITICAL = 'CRITICAL',
  HIGH = 'HIGH',
  MEDIUM = 'MEDIUM',
  LOW = 'LOW'
}

export enum RiskCategory {
  BIAS_AND_FAIRNESS = 'BIAS_AND_FAIRNESS',
  PRIVACY_AND_DATA_PROTECTION = 'PRIVACY_AND_DATA_PROTECTION',
  SAFETY_AND_RELIABILITY = 'SAFETY_AND_RELIABILITY',
  TRANSPARENCY = 'TRANSPARENCY',
  SECURITY = 'SECURITY',
  MODEL_DRIFT = 'MODEL_DRIFT',
  THIRD_PARTY = 'THIRD_PARTY',
  COMPLIANCE = 'COMPLIANCE'
}

export enum AISystemRiskClassification {
  UNACCEPTABLE = 'UNACCEPTABLE',
  HIGH_RISK = 'HIGH_RISK',
  LIMITED_RISK = 'LIMITED_RISK',
  MINIMAL_RISK = 'MINIMAL_RISK'
}

export enum ComplianceStatus {
  COMPLIANT = 'COMPLIANT',
  PARTIALLY_COMPLIANT = 'PARTIALLY_COMPLIANT',
  NON_COMPLIANT = 'NON_COMPLIANT',
  NOT_APPLICABLE = 'NOT_APPLICABLE'
}

export enum GovernancePhase {
  PHASE_1 = 'PHASE_1',
  PHASE_2 = 'PHASE_2',
  PHASE_3 = 'PHASE_3',
  PHASE_4 = 'PHASE_4'
}

// ============================================================================
// AI System Types
// ============================================================================

export interface AISystem {
  id: string;
  name: string;
  description: string;
  version: string;
  owner: string;
  intendedUse: string;
  riskClassification: AISystemRiskClassification;
  status: 'DEVELOPMENT' | 'TESTING' | 'DEPLOYED' | 'DEPRECATED';
  createdAt: Date;
  updatedAt: Date;
  metadata?: Record<string, any>;
}

export interface ModelInformation {
  modelType: string;
  framework: string;
  architecture: string;
  hyperparameters: Record<string, any>;
  trainingData: DataSource;
  performanceMetrics: PerformanceMetrics;
  limitations: string[];
  intendedUse: string[];
}

export interface DataSource {
  id: string;
  name: string;
  description: string;
  dataType: string;
  size: number;
  sources: string[];
  collectionDate: Date;
  preprocessing: string[];
  knownBiases: string[];
  limitations: string[];
}

export interface PerformanceMetrics {
  accuracy: number;
  precision: number;
  recall: number;
  f1Score: number;
  auc?: number;
  customMetrics?: Record<string, number>;
}

// ============================================================================
// Risk Assessment Types
// ============================================================================

export interface RiskAssessment {
  id: string;
  systemId: string;
  category: RiskCategory;
  title: string;
  description: string;
  affectedStakeholders: string[];
  likelihood: number; // 1-5
  impact: number; // 1-5
  detectability: number; // 1-5
  riskScore: number;
  riskLevel: RiskLevel;
  currentControls: string[];
  mitigationPlan: MitigationPlan;
  residualRisk: number;
  owner: string;
  status: 'IDENTIFIED' | 'ASSESSED' | 'MITIGATED' | 'ACCEPTED';
  assessmentDate: Date;
  reviewDate?: Date;
}

export interface MitigationPlan {
  strategies: MitigationStrategy[];
  timeline: string;
  resourcesRequired: Resource[];
  successMetrics: Metric[];
  owner: string;
  status: 'PLANNED' | 'IN_PROGRESS' | 'IMPLEMENTED' | 'VERIFIED';
}

export interface MitigationStrategy {
  type: 'PREVENTIVE' | 'DETECTIVE' | 'CORRECTIVE';
  description: string;
  implementation: string;
  costBenefit: number;
}

export interface Resource {
  type: 'PERSONNEL' | 'TOOLS' | 'DATA' | 'INFRASTRUCTURE';
  description: string;
  cost: number;
}

export interface Metric {
  name: string;
  target: string | number;
  current: string | number;
  unit?: string;
}

// ============================================================================
// Ethics & Impact Assessment Types
// ============================================================================

export interface ImpactAssessment {
  id: string;
  systemId: string;
  systemDescription: string;
  intendedUse: string;
  affectedStakeholders: Stakeholder[];
  benefits: Impact[];
  harms: Impact[];
  fairnessEvaluation: FairnessEvaluation;
  privacyImpact: PrivacyImpact;
  transparencyAssessment: TransparencyAssessment;
  accountabilityMechanisms: AccountabilityMechanism[];
  mitigationStrategies: string[];
  monitoringPlans: string[];
  assessor: string;
  assessmentDate: Date;
  reviewDate?: Date;
  approvalStatus: 'PENDING' | 'APPROVED' | 'REJECTED' | 'REQUIRES_MODIFICATION';
}

export interface Stakeholder {
  group: string;
  description: string;
  influenceLevel: 'HIGH' | 'MEDIUM' | 'LOW';
  interestLevel: 'HIGH' | 'MEDIUM' | 'LOW';
  concerns: string[];
}

export interface Impact {
  description: string;
  severity: 'HIGH' | 'MEDIUM' | 'LOW';
  likelihood: 'HIGH' | 'MEDIUM' | 'LOW';
  affectedGroups: string[];
}

export interface FairnessEvaluation {
  protectedAttributes: string[];
  fairnessMetrics: FairnessMetric[];
  biasTestingResults: BiasTestingResult[];
  mitigationApproaches: string[];
}

export interface FairnessMetric {
  name: string;
  value: number;
  threshold: number;
  compliant: boolean;
  description: string;
}

export interface BiasTestingResult {
  attribute: string;
  metric: string;
  value: number;
  threshold: number;
  passed: boolean;
  details: string;
}

export interface PrivacyImpact {
  personalDataProcessed: boolean;
  dataTypes: string[];
  lawfulBasis: string;
  dataMinimization: boolean;
  retentionPeriod: string;
  securityMeasures: string[];
  individualRights: string[];
}

export interface TransparencyAssessment {
  explainabilityRequired: boolean;
  explanationMethods: string[];
  disclosureToUsers: boolean;
  documentationComplete: boolean;
  auditTrailAvailable: boolean;
}

export interface AccountabilityMechanism {
  mechanism: string;
  description: string;
  responsible: string;
}

// ============================================================================
// Policy & Compliance Types
// ============================================================================

export interface Policy {
  id: string;
  name: string;
  version: string;
  effectiveDate: Date;
  reviewDate: Date;
  owner: string;
  purpose: string;
  scope: string;
  principles: string[];
  requirements: PolicyRequirement[];
  relatedDocuments: string[];
  approvalStatus: 'DRAFT' | 'REVIEW' | 'APPROVED' | 'DEPRECATED';
}

export interface PolicyRequirement {
  id: string;
  description: string;
  mandatory: boolean; // true = MUST/SHALL, false = SHOULD/MAY
  automated: boolean;
  checkFunction?: string;
}

export interface ComplianceCheck {
  id: string;
  systemId: string;
  regulation: string;
  requirements: ComplianceRequirement[];
  overallStatus: ComplianceStatus;
  findings: ComplianceFinding[];
  assessor: string;
  assessmentDate: Date;
  nextReviewDate: Date;
}

export interface ComplianceRequirement {
  requirementId: string;
  description: string;
  status: ComplianceStatus;
  evidence: string[];
  gaps: string[];
}

export interface ComplianceFinding {
  severity: 'CRITICAL' | 'HIGH' | 'MEDIUM' | 'LOW';
  requirement: string;
  finding: string;
  recommendation: string;
  remediation: string;
  owner: string;
  dueDate: Date;
}

// ============================================================================
// Governance Framework Types
// ============================================================================

export interface GovernanceFramework {
  organizationId: string;
  phase: GovernancePhase;
  strategicLayer: StrategicLayer;
  tacticalLayer: TacticalLayer;
  operationalLayer: OperationalLayer;
  maturityScore: number;
  lastAssessmentDate: Date;
}

export interface StrategicLayer {
  aiVision: string;
  governancePrinciples: string[];
  executiveOversight: OversightStructure;
  stakeholderStrategy: StakeholderEngagement;
}

export interface TacticalLayer {
  policies: Policy[];
  riskFramework: RiskFramework;
  complianceProgram: ComplianceProgram;
  ethicsBoard: EthicsBoard;
  performanceMetrics: KPI[];
}

export interface OperationalLayer {
  developmentGuidelines: DevelopmentStandard[];
  testingProtocols: TestingProcedure[];
  deploymentControls: DeploymentGate[];
  monitoringSystem: MonitoringSystem;
  incidentResponse: IncidentProcedure;
}

export interface OversightStructure {
  boardOversight: boolean;
  chiefAIOfficer: string;
  oversightCommittee: Committee;
}

export interface Committee {
  name: string;
  members: string[];
  meetingFrequency: string;
  responsibilities: string[];
}

export interface EthicsBoard extends Committee {
  charter: string;
  reviewProcess: ReviewProcess;
  decisionAuthority: string[];
}

export interface ReviewProcess {
  submissionCriteria: string[];
  evaluationCriteria: string[];
  decisionTimeline: string;
  appealProcess: string;
}

export interface StakeholderEngagement {
  stakeholders: Stakeholder[];
  engagementStrategy: string;
  feedbackMechanisms: string[];
}

export interface RiskFramework {
  methodology: string;
  riskCategories: RiskCategory[];
  assessmentFrequency: string;
  escalationCriteria: string[];
}

export interface ComplianceProgram {
  applicableRegulations: string[];
  complianceChecks: ComplianceCheck[];
  auditSchedule: string;
  reportingFrequency: string;
}

export interface DevelopmentStandard {
  name: string;
  description: string;
  requirements: string[];
  tools: string[];
}

export interface TestingProcedure {
  name: string;
  description: string;
  steps: string[];
  passCriteria: string[];
}

export interface DeploymentGate {
  gateName: string;
  criteria: string[];
  approvers: string[];
  mandatory: boolean;
}

export interface MonitoringSystem {
  metrics: Metric[];
  alertThresholds: AlertThreshold[];
  reportingFrequency: string;
}

export interface AlertThreshold {
  metric: string;
  threshold: number;
  severity: 'CRITICAL' | 'HIGH' | 'MEDIUM' | 'LOW';
  action: string;
}

export interface IncidentProcedure {
  detectionMethods: string[];
  classificationCriteria: string[];
  responseSteps: string[];
  escalationPath: string[];
}

export interface KPI {
  name: string;
  description: string;
  target: number;
  current: number;
  trend: 'IMPROVING' | 'STABLE' | 'DECLINING';
}

// ============================================================================
// Audit & Reporting Types
// ============================================================================

export interface AuditReport {
  id: string;
  auditType: 'INTERNAL' | 'EXTERNAL' | 'REGULATORY';
  scope: string[];
  auditor: string;
  auditDate: Date;
  findings: AuditFinding[];
  overallAssessment: string;
  recommendations: string[];
  nextAuditDate: Date;
}

export interface AuditFinding {
  severity: 'CRITICAL' | 'HIGH' | 'MEDIUM' | 'LOW';
  area: string;
  finding: string;
  evidence: string;
  recommendation: string;
  remediation: RemediationPlan;
}

export interface RemediationPlan {
  actions: string[];
  owner: string;
  timeline: string;
  status: 'PLANNED' | 'IN_PROGRESS' | 'COMPLETED';
}

// ============================================================================
// Utility Types
// ============================================================================

export interface ValidationResult {
  valid: boolean;
  errors: ValidationError[];
  warnings: ValidationWarning[];
}

export interface ValidationError {
  field: string;
  message: string;
  code: string;
}

export interface ValidationWarning {
  field: string;
  message: string;
  recommendation: string;
}

/**
 * 弘益人間 (Benefit All Humanity)
 *
 * All types and interfaces in this module support the development of
 * AI governance systems that prioritize human welfare and societal benefit.
 */
