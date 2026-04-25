/**
 * WIA Security Audit Standard - Type Definitions
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * Comprehensive security audit, compliance, and vulnerability management types
 *
 * @package @wia/security-audit
 * @version 1.0.0
 */

// ============================================================================
// Severity and Risk Levels
// ============================================================================

export type SeverityLevel = 'critical' | 'high' | 'medium' | 'low' | 'info';
export type RiskLevel = 'critical' | 'high' | 'medium' | 'low' | 'negligible';
export type ComplianceStatus = 'compliant' | 'non-compliant' | 'partially-compliant' | 'not-applicable';
export type FindingStatus = 'open' | 'in-progress' | 'resolved' | 'accepted' | 'false-positive' | 'wont-fix';
export type AuditStatus = 'planned' | 'in-progress' | 'review' | 'completed' | 'cancelled';

// ============================================================================
// Vulnerability Types
// ============================================================================

export interface Vulnerability {
  id: string;
  title: string;
  description: string;
  severity: SeverityLevel;
  cvssScore?: number;
  cvssVector?: string;
  cveId?: string;
  cweId?: string;
  affectedAssets: string[];
  discoveredAt: Date;
  discoveredBy: string;
  category: VulnerabilityCategory;
  exploitability: ExploitabilityLevel;
  impact: ImpactAssessment;
  references: string[];
  tags: string[];
}

export type VulnerabilityCategory =
  | 'injection'
  | 'authentication'
  | 'authorization'
  | 'cryptography'
  | 'configuration'
  | 'sensitive-data'
  | 'xxe'
  | 'access-control'
  | 'security-misconfiguration'
  | 'xss'
  | 'deserialization'
  | 'logging-monitoring'
  | 'ssrf'
  | 'other';

export type ExploitabilityLevel = 'easy' | 'medium' | 'hard' | 'theoretical';

export interface ImpactAssessment {
  confidentiality: 'high' | 'medium' | 'low' | 'none';
  integrity: 'high' | 'medium' | 'low' | 'none';
  availability: 'high' | 'medium' | 'low' | 'none';
  scope: 'unchanged' | 'changed';
  businessImpact: string;
}

// ============================================================================
// Security Finding Types
// ============================================================================

export interface SecurityFinding {
  id: string;
  auditId: string;
  type: FindingType;
  title: string;
  description: string;
  severity: SeverityLevel;
  status: FindingStatus;
  category: string;
  location: FindingLocation;
  evidence: Evidence[];
  recommendation: string;
  remediation?: RemediationPlan;
  affectedComponents: string[];
  discoveredAt: Date;
  discoveredBy: string;
  assignedTo?: string;
  dueDate?: Date;
  resolvedAt?: Date;
  metadata: Record<string, any>;
}

export type FindingType =
  | 'vulnerability'
  | 'compliance-violation'
  | 'best-practice'
  | 'configuration-issue'
  | 'policy-violation'
  | 'security-weakness';

export interface FindingLocation {
  type: 'code' | 'configuration' | 'infrastructure' | 'process' | 'documentation';
  path?: string;
  lineNumber?: number;
  component?: string;
  service?: string;
  environment?: string;
}

// ============================================================================
// Evidence and Artifact Types
// ============================================================================

export interface Evidence {
  id: string;
  type: EvidenceType;
  title: string;
  description: string;
  content?: string;
  fileUrl?: string;
  screenshot?: string;
  collectedAt: Date;
  collectedBy: string;
  hash?: string;
  metadata: Record<string, any>;
}

export type EvidenceType =
  | 'screenshot'
  | 'log-file'
  | 'configuration'
  | 'code-snippet'
  | 'network-capture'
  | 'scan-result'
  | 'document'
  | 'video'
  | 'other';

export interface AuditArtifact {
  id: string;
  auditId: string;
  name: string;
  type: ArtifactType;
  description: string;
  fileUrl: string;
  size: number;
  hash: string;
  createdAt: Date;
  createdBy: string;
  metadata: Record<string, any>;
}

export type ArtifactType =
  | 'scan-report'
  | 'compliance-report'
  | 'vulnerability-report'
  | 'penetration-test'
  | 'code-analysis'
  | 'network-diagram'
  | 'security-policy'
  | 'audit-workpapers'
  | 'other';

// ============================================================================
// Compliance Check Types
// ============================================================================

export interface ComplianceCheck {
  id: string;
  auditId: string;
  framework: ComplianceFramework;
  controlId: string;
  controlTitle: string;
  controlDescription: string;
  status: ComplianceStatus;
  evidence: Evidence[];
  findings: string[];
  assessedAt: Date;
  assessedBy: string;
  notes: string;
  score?: number;
  metadata: Record<string, any>;
}

export type ComplianceFramework =
  | 'ISO-27001'
  | 'SOC2'
  | 'PCI-DSS'
  | 'HIPAA'
  | 'GDPR'
  | 'NIST-800-53'
  | 'CIS-Controls'
  | 'OWASP-ASVS'
  | 'FedRAMP'
  | 'CCPA'
  | 'custom';

export interface ComplianceReport {
  auditId: string;
  framework: ComplianceFramework;
  totalControls: number;
  compliantControls: number;
  nonCompliantControls: number;
  partiallyCompliantControls: number;
  notApplicableControls: number;
  overallScore: number;
  checks: ComplianceCheck[];
  generatedAt: Date;
  generatedBy: string;
}

// ============================================================================
// Audit Scope and Criteria Types
// ============================================================================

export interface AuditScope {
  id: string;
  name: string;
  description: string;
  targets: AuditTarget[];
  exclusions: string[];
  frameworks: ComplianceFramework[];
  startDate: Date;
  endDate: Date;
  objectives: string[];
  constraints: string[];
  assumptions: string[];
}

export interface AuditTarget {
  id: string;
  type: TargetType;
  name: string;
  description: string;
  url?: string;
  ipAddress?: string;
  environment: 'production' | 'staging' | 'development' | 'test';
  criticality: 'critical' | 'high' | 'medium' | 'low';
  metadata: Record<string, any>;
}

export type TargetType =
  | 'web-application'
  | 'mobile-application'
  | 'api'
  | 'network'
  | 'infrastructure'
  | 'cloud-service'
  | 'database'
  | 'source-code'
  | 'documentation'
  | 'process';

export interface AuditCriteria {
  id: string;
  auditId: string;
  type: 'technical' | 'compliance' | 'process' | 'policy';
  description: string;
  acceptanceCriteria: string;
  testProcedure: string;
  expectedResult: string;
  priority: 'high' | 'medium' | 'low';
}

// ============================================================================
// Remediation Types
// ============================================================================

export interface RemediationPlan {
  id: string;
  findingId: string;
  title: string;
  description: string;
  steps: RemediationStep[];
  assignedTo: string;
  priority: 'critical' | 'high' | 'medium' | 'low';
  effort: 'low' | 'medium' | 'high';
  estimatedHours?: number;
  dueDate: Date;
  status: 'pending' | 'in-progress' | 'completed' | 'blocked' | 'cancelled';
  createdAt: Date;
  updatedAt: Date;
  completedAt?: Date;
  metadata: Record<string, any>;
}

export interface RemediationStep {
  id: string;
  stepNumber: number;
  description: string;
  status: 'pending' | 'in-progress' | 'completed';
  assignedTo?: string;
  completedAt?: Date;
  notes?: string;
}

export interface RemediationTracking {
  findingId: string;
  planId: string;
  status: string;
  progress: number;
  updates: RemediationUpdate[];
  blockers: string[];
  estimatedCompletionDate: Date;
}

export interface RemediationUpdate {
  id: string;
  timestamp: Date;
  author: string;
  status: string;
  comment: string;
  attachments: string[];
}

// ============================================================================
// Audit Log and Activity Types
// ============================================================================

export interface AuditLogEntry {
  id: string;
  auditId: string;
  timestamp: Date;
  action: AuditAction;
  actor: string;
  target: string;
  targetType: string;
  details: Record<string, any>;
  ipAddress?: string;
  userAgent?: string;
  result: 'success' | 'failure' | 'warning';
  metadata: Record<string, any>;
}

export type AuditAction =
  | 'audit-created'
  | 'audit-started'
  | 'audit-completed'
  | 'finding-created'
  | 'finding-updated'
  | 'finding-resolved'
  | 'evidence-collected'
  | 'compliance-checked'
  | 'report-generated'
  | 'remediation-planned'
  | 'remediation-completed'
  | 'scope-modified'
  | 'comment-added';

// ============================================================================
// Report Generation Types
// ============================================================================

export interface AuditReport {
  id: string;
  auditId: string;
  type: ReportType;
  title: string;
  executiveSummary: string;
  scope: AuditScope;
  methodology: string;
  findings: SecurityFinding[];
  complianceResults?: ComplianceReport;
  recommendations: Recommendation[];
  statistics: AuditStatistics;
  generatedAt: Date;
  generatedBy: string;
  format: 'pdf' | 'html' | 'markdown' | 'json';
  fileUrl?: string;
  metadata: Record<string, any>;
}

export type ReportType =
  | 'executive-summary'
  | 'technical-details'
  | 'compliance-report'
  | 'vulnerability-assessment'
  | 'penetration-test'
  | 'full-audit';

export interface Recommendation {
  id: string;
  priority: 'critical' | 'high' | 'medium' | 'low';
  title: string;
  description: string;
  rationale: string;
  implementation: string;
  estimatedEffort: string;
  relatedFindings: string[];
}

export interface AuditStatistics {
  totalFindings: number;
  findingsBySeverity: Record<SeverityLevel, number>;
  findingsByStatus: Record<FindingStatus, number>;
  findingsByCategory: Record<string, number>;
  vulnerabilitiesCount: number;
  complianceScore?: number;
  riskScore: number;
  remediationProgress: number;
}

// ============================================================================
// Main Audit Types
// ============================================================================

export interface SecurityAudit {
  id: string;
  name: string;
  description: string;
  type: AuditType;
  status: AuditStatus;
  scope: AuditScope;
  criteria: AuditCriteria[];
  findings: SecurityFinding[];
  complianceChecks: ComplianceCheck[];
  artifacts: AuditArtifact[];
  team: AuditTeamMember[];
  createdAt: Date;
  createdBy: string;
  startedAt?: Date;
  completedAt?: Date;
  metadata: Record<string, any>;
}

export type AuditType =
  | 'internal-audit'
  | 'external-audit'
  | 'penetration-test'
  | 'vulnerability-assessment'
  | 'compliance-audit'
  | 'code-review'
  | 'configuration-review';

export interface AuditTeamMember {
  id: string;
  name: string;
  role: 'lead-auditor' | 'auditor' | 'technical-specialist' | 'reviewer';
  email: string;
  certifications?: string[];
}

// ============================================================================
// Event Types
// ============================================================================

export interface AuditEvent {
  type: AuditEventType;
  timestamp: Date;
  data: any;
}

export type AuditEventType =
  | 'audit-created'
  | 'audit-started'
  | 'audit-completed'
  | 'finding-created'
  | 'finding-updated'
  | 'evidence-added'
  | 'compliance-checked'
  | 'report-generated'
  | 'remediation-updated';

export type EventHandler = (event: AuditEvent) => void | Promise<void>;
