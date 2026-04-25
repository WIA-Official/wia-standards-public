/**
 * WIA Security Audit Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * Comprehensive security audit, compliance, and vulnerability management SDK
 *
 * @package @wia/security-audit
 * @version 1.0.0
 */

import {
  SecurityAudit,
  SecurityFinding,
  AuditScope,
  AuditCriteria,
  ComplianceCheck,
  Evidence,
  AuditArtifact,
  RemediationPlan,
  RemediationTracking,
  AuditReport,
  AuditLogEntry,
  AuditEvent,
  EventHandler,
  AuditStatus,
  FindingStatus,
  ComplianceFramework,
  ComplianceReport,
  AuditStatistics,
  Vulnerability,
  SeverityLevel,
  FindingType,
  ReportType,
  AuditType,
} from './types';

/**
 * WIA Security Audit SDK
 *
 * Provides comprehensive security audit capabilities including:
 * - Audit planning and scoping
 * - Vulnerability and finding management
 * - Compliance checking and reporting
 * - Evidence collection and tracking
 * - Remediation planning and tracking
 * - Audit reporting and analytics
 */
export class WIASecurityAudit {
  private audits: Map<string, SecurityAudit> = new Map();
  private findings: Map<string, SecurityFinding> = new Map();
  private remediationPlans: Map<string, RemediationPlan> = new Map();
  private auditLogs: AuditLogEntry[] = [];
  private eventHandlers: Map<string, EventHandler[]> = new Map();

  constructor(private options: SecurityAuditOptions = {}) {
    this.initializeEventHandlers();
  }

  // ============================================================================
  // Audit Planning and Scoping
  // ============================================================================

  /**
   * Create a new security audit
   */
  async createAudit(params: CreateAuditParams): Promise<SecurityAudit> {
    const audit: SecurityAudit = {
      id: this.generateId(),
      name: params.name,
      description: params.description,
      type: params.type,
      status: 'planned',
      scope: params.scope,
      criteria: params.criteria || [],
      findings: [],
      complianceChecks: [],
      artifacts: [],
      team: params.team || [],
      createdAt: new Date(),
      createdBy: params.createdBy,
      metadata: params.metadata || {},
    };

    this.audits.set(audit.id, audit);
    await this.logActivity(audit.id, 'audit-created', params.createdBy, audit.id, 'audit', {
      auditName: audit.name,
      auditType: audit.type,
    });
    await this.emitEvent({ type: 'audit-created', timestamp: new Date(), data: audit });

    return audit;
  }

  /**
   * Start an audit
   */
  async startAudit(auditId: string, startedBy: string): Promise<SecurityAudit> {
    const audit = this.getAudit(auditId);
    audit.status = 'in-progress';
    audit.startedAt = new Date();

    await this.logActivity(auditId, 'audit-started', startedBy, auditId, 'audit', {});
    await this.emitEvent({ type: 'audit-started', timestamp: new Date(), data: audit });

    return audit;
  }

  /**
   * Complete an audit
   */
  async completeAudit(auditId: string, completedBy: string): Promise<SecurityAudit> {
    const audit = this.getAudit(auditId);
    audit.status = 'completed';
    audit.completedAt = new Date();

    await this.logActivity(auditId, 'audit-completed', completedBy, auditId, 'audit', {});
    await this.emitEvent({ type: 'audit-completed', timestamp: new Date(), data: audit });

    return audit;
  }

  /**
   * Update audit scope
   */
  async updateAuditScope(auditId: string, scope: Partial<AuditScope>, updatedBy: string): Promise<SecurityAudit> {
    const audit = this.getAudit(auditId);
    audit.scope = { ...audit.scope, ...scope };

    await this.logActivity(auditId, 'scope-modified', updatedBy, auditId, 'scope', { changes: scope });

    return audit;
  }

  /**
   * Add audit criteria
   */
  async addAuditCriteria(auditId: string, criteria: AuditCriteria): Promise<SecurityAudit> {
    const audit = this.getAudit(auditId);
    audit.criteria.push(criteria);

    return audit;
  }

  // ============================================================================
  // Finding Management
  // ============================================================================

  /**
   * Create a security finding
   */
  async createFinding(params: CreateFindingParams): Promise<SecurityFinding> {
    const finding: SecurityFinding = {
      id: this.generateId(),
      auditId: params.auditId,
      type: params.type,
      title: params.title,
      description: params.description,
      severity: params.severity,
      status: 'open',
      category: params.category,
      location: params.location,
      evidence: params.evidence || [],
      recommendation: params.recommendation,
      affectedComponents: params.affectedComponents || [],
      discoveredAt: new Date(),
      discoveredBy: params.discoveredBy,
      metadata: params.metadata || {},
    };

    this.findings.set(finding.id, finding);

    const audit = this.getAudit(params.auditId);
    audit.findings.push(finding);

    await this.logActivity(params.auditId, 'finding-created', params.discoveredBy, finding.id, 'finding', {
      severity: finding.severity,
      type: finding.type,
    });
    await this.emitEvent({ type: 'finding-created', timestamp: new Date(), data: finding });

    return finding;
  }

  /**
   * Update finding status
   */
  async updateFindingStatus(
    findingId: string,
    status: FindingStatus,
    updatedBy: string,
    notes?: string
  ): Promise<SecurityFinding> {
    const finding = this.getFinding(findingId);
    const oldStatus = finding.status;
    finding.status = status;

    if (status === 'resolved') {
      finding.resolvedAt = new Date();
    }

    await this.logActivity(finding.auditId, 'finding-updated', updatedBy, findingId, 'finding', {
      oldStatus,
      newStatus: status,
      notes,
    });
    await this.emitEvent({ type: 'finding-updated', timestamp: new Date(), data: finding });

    return finding;
  }

  /**
   * Assign finding to team member
   */
  async assignFinding(findingId: string, assignedTo: string, dueDate?: Date): Promise<SecurityFinding> {
    const finding = this.getFinding(findingId);
    finding.assignedTo = assignedTo;
    finding.dueDate = dueDate;

    return finding;
  }

  /**
   * Get findings by severity
   */
  async getFindingsBySeverity(auditId: string, severity: SeverityLevel): Promise<SecurityFinding[]> {
    const audit = this.getAudit(auditId);
    return audit.findings.filter((f) => f.severity === severity);
  }

  /**
   * Get findings by status
   */
  async getFindingsByStatus(auditId: string, status: FindingStatus): Promise<SecurityFinding[]> {
    const audit = this.getAudit(auditId);
    return audit.findings.filter((f) => f.status === status);
  }

  // ============================================================================
  // Evidence Collection
  // ============================================================================

  /**
   * Collect and attach evidence to a finding
   */
  async collectEvidence(findingId: string, evidence: Evidence, collectedBy: string): Promise<SecurityFinding> {
    const finding = this.getFinding(findingId);
    finding.evidence.push(evidence);

    await this.logActivity(finding.auditId, 'evidence-collected', collectedBy, findingId, 'evidence', {
      evidenceType: evidence.type,
      evidenceId: evidence.id,
    });
    await this.emitEvent({ type: 'evidence-added', timestamp: new Date(), data: { finding, evidence } });

    return finding;
  }

  /**
   * Add artifact to audit
   */
  async addArtifact(auditId: string, artifact: AuditArtifact): Promise<SecurityAudit> {
    const audit = this.getAudit(auditId);
    audit.artifacts.push(artifact);

    return audit;
  }

  /**
   * Get all evidence for a finding
   */
  async getEvidence(findingId: string): Promise<Evidence[]> {
    const finding = this.getFinding(findingId);
    return finding.evidence;
  }

  // ============================================================================
  // Compliance Checking
  // ============================================================================

  /**
   * Perform compliance check
   */
  async performComplianceCheck(params: ComplianceCheckParams): Promise<ComplianceCheck> {
    const check: ComplianceCheck = {
      id: this.generateId(),
      auditId: params.auditId,
      framework: params.framework,
      controlId: params.controlId,
      controlTitle: params.controlTitle,
      controlDescription: params.controlDescription,
      status: params.status,
      evidence: params.evidence || [],
      findings: params.findings || [],
      assessedAt: new Date(),
      assessedBy: params.assessedBy,
      notes: params.notes || '',
      score: params.score,
      metadata: params.metadata || {},
    };

    const audit = this.getAudit(params.auditId);
    audit.complianceChecks.push(check);

    await this.logActivity(params.auditId, 'compliance-checked', params.assessedBy, check.id, 'compliance', {
      framework: check.framework,
      controlId: check.controlId,
      status: check.status,
    });
    await this.emitEvent({ type: 'compliance-checked', timestamp: new Date(), data: check });

    return check;
  }

  /**
   * Generate compliance report
   */
  async generateComplianceReport(auditId: string, framework: ComplianceFramework): Promise<ComplianceReport> {
    const audit = this.getAudit(auditId);
    const checks = audit.complianceChecks.filter((c) => c.framework === framework);

    const compliantControls = checks.filter((c) => c.status === 'compliant').length;
    const nonCompliantControls = checks.filter((c) => c.status === 'non-compliant').length;
    const partiallyCompliantControls = checks.filter((c) => c.status === 'partially-compliant').length;
    const notApplicableControls = checks.filter((c) => c.status === 'not-applicable').length;

    const totalControls = checks.length;
    const overallScore =
      totalControls > 0 ? Math.round((compliantControls / (totalControls - notApplicableControls)) * 100) : 0;

    const report: ComplianceReport = {
      auditId,
      framework,
      totalControls,
      compliantControls,
      nonCompliantControls,
      partiallyCompliantControls,
      notApplicableControls,
      overallScore,
      checks,
      generatedAt: new Date(),
      generatedBy: this.options.defaultUser || 'system',
    };

    return report;
  }

  /**
   * Get compliance status summary
   */
  async getComplianceStatus(auditId: string): Promise<Record<ComplianceFramework, ComplianceReport>> {
    const audit = this.getAudit(auditId);
    const frameworks = [...new Set(audit.complianceChecks.map((c) => c.framework))];

    const reports: Record<string, ComplianceReport> = {};
    for (const framework of frameworks) {
      reports[framework] = await this.generateComplianceReport(auditId, framework);
    }

    return reports as Record<ComplianceFramework, ComplianceReport>;
  }

  // ============================================================================
  // Report Generation
  // ============================================================================

  /**
   * Generate audit report
   */
  async generateReport(params: GenerateReportParams): Promise<AuditReport> {
    const audit = this.getAudit(params.auditId);
    const statistics = await this.calculateStatistics(params.auditId);

    const report: AuditReport = {
      id: this.generateId(),
      auditId: params.auditId,
      type: params.type,
      title: params.title || `${audit.name} - ${params.type}`,
      executiveSummary: params.executiveSummary || this.generateExecutiveSummary(audit, statistics),
      scope: audit.scope,
      methodology: params.methodology || 'Standard security audit methodology',
      findings: params.includeFindings !== false ? audit.findings : [],
      complianceResults: params.includeCompliance
        ? await this.generateComplianceReport(params.auditId, params.complianceFramework!)
        : undefined,
      recommendations: params.recommendations || [],
      statistics,
      generatedAt: new Date(),
      generatedBy: params.generatedBy,
      format: params.format || 'json',
      metadata: params.metadata || {},
    };

    await this.logActivity(params.auditId, 'report-generated', params.generatedBy, report.id, 'report', {
      reportType: params.type,
    });
    await this.emitEvent({ type: 'report-generated', timestamp: new Date(), data: report });

    return report;
  }

  /**
   * Calculate audit statistics
   */
  async calculateStatistics(auditId: string): Promise<AuditStatistics> {
    const audit = this.getAudit(auditId);

    const totalFindings = audit.findings.length;
    const findingsBySeverity: Record<SeverityLevel, number> = {
      critical: audit.findings.filter((f) => f.severity === 'critical').length,
      high: audit.findings.filter((f) => f.severity === 'high').length,
      medium: audit.findings.filter((f) => f.severity === 'medium').length,
      low: audit.findings.filter((f) => f.severity === 'low').length,
      info: audit.findings.filter((f) => f.severity === 'info').length,
    };

    const findingsByStatus: Record<FindingStatus, number> = {
      open: audit.findings.filter((f) => f.status === 'open').length,
      'in-progress': audit.findings.filter((f) => f.status === 'in-progress').length,
      resolved: audit.findings.filter((f) => f.status === 'resolved').length,
      accepted: audit.findings.filter((f) => f.status === 'accepted').length,
      'false-positive': audit.findings.filter((f) => f.status === 'false-positive').length,
      'wont-fix': audit.findings.filter((f) => f.status === 'wont-fix').length,
    };

    const findingsByCategory: Record<string, number> = {};
    audit.findings.forEach((f) => {
      findingsByCategory[f.category] = (findingsByCategory[f.category] || 0) + 1;
    });

    const vulnerabilitiesCount = audit.findings.filter((f) => f.type === 'vulnerability').length;

    // Calculate risk score (0-100, higher is worse)
    const riskScore = this.calculateRiskScore(findingsBySeverity, totalFindings);

    // Calculate remediation progress
    const resolvedCount = findingsByStatus.resolved + findingsByStatus.accepted;
    const remediationProgress = totalFindings > 0 ? Math.round((resolvedCount / totalFindings) * 100) : 100;

    return {
      totalFindings,
      findingsBySeverity,
      findingsByStatus,
      findingsByCategory,
      vulnerabilitiesCount,
      riskScore,
      remediationProgress,
    };
  }

  // ============================================================================
  // Remediation Tracking
  // ============================================================================

  /**
   * Create remediation plan for a finding
   */
  async createRemediationPlan(params: CreateRemediationParams): Promise<RemediationPlan> {
    const plan: RemediationPlan = {
      id: this.generateId(),
      findingId: params.findingId,
      title: params.title,
      description: params.description,
      steps: params.steps,
      assignedTo: params.assignedTo,
      priority: params.priority,
      effort: params.effort,
      estimatedHours: params.estimatedHours,
      dueDate: params.dueDate,
      status: 'pending',
      createdAt: new Date(),
      updatedAt: new Date(),
      metadata: params.metadata || {},
    };

    this.remediationPlans.set(plan.id, plan);

    const finding = this.getFinding(params.findingId);
    finding.remediation = plan;

    await this.logActivity(finding.auditId, 'remediation-planned', params.assignedTo, plan.id, 'remediation', {
      findingId: params.findingId,
      priority: plan.priority,
    });

    return plan;
  }

  /**
   * Update remediation plan status
   */
  async updateRemediationStatus(
    planId: string,
    status: RemediationPlan['status'],
    updatedBy: string
  ): Promise<RemediationPlan> {
    const plan = this.remediationPlans.get(planId);
    if (!plan) {
      throw new Error(`Remediation plan ${planId} not found`);
    }

    plan.status = status;
    plan.updatedAt = new Date();

    if (status === 'completed') {
      plan.completedAt = new Date();
      const finding = this.getFinding(plan.findingId);
      await this.logActivity(finding.auditId, 'remediation-completed', updatedBy, planId, 'remediation', {
        findingId: plan.findingId,
      });
    }

    await this.emitEvent({ type: 'remediation-updated', timestamp: new Date(), data: plan });

    return plan;
  }

  /**
   * Get remediation tracking status
   */
  async getRemediationTracking(planId: string): Promise<RemediationTracking> {
    const plan = this.remediationPlans.get(planId);
    if (!plan) {
      throw new Error(`Remediation plan ${planId} not found`);
    }

    const completedSteps = plan.steps.filter((s) => s.status === 'completed').length;
    const totalSteps = plan.steps.length;
    const progress = totalSteps > 0 ? Math.round((completedSteps / totalSteps) * 100) : 0;

    return {
      findingId: plan.findingId,
      planId: plan.id,
      status: plan.status,
      progress,
      updates: [],
      blockers: [],
      estimatedCompletionDate: plan.dueDate,
    };
  }

  // ============================================================================
  // Event Handling
  // ============================================================================

  /**
   * Register event handler
   */
  on(eventType: string, handler: EventHandler): void {
    if (!this.eventHandlers.has(eventType)) {
      this.eventHandlers.set(eventType, []);
    }
    this.eventHandlers.get(eventType)!.push(handler);
  }

  /**
   * Remove event handler
   */
  off(eventType: string, handler: EventHandler): void {
    const handlers = this.eventHandlers.get(eventType);
    if (handlers) {
      const index = handlers.indexOf(handler);
      if (index > -1) {
        handlers.splice(index, 1);
      }
    }
  }

  /**
   * Emit event to all registered handlers
   */
  private async emitEvent(event: AuditEvent): Promise<void> {
    const handlers = this.eventHandlers.get(event.type);
    if (handlers) {
      for (const handler of handlers) {
        await handler(event);
      }
    }

    // Also emit to wildcard handlers
    const wildcardHandlers = this.eventHandlers.get('*');
    if (wildcardHandlers) {
      for (const handler of wildcardHandlers) {
        await handler(event);
      }
    }
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  private initializeEventHandlers(): void {
    // Initialize default event handlers if needed
    if (this.options.enableLogging) {
      this.on('*', (event) => {
        console.log(`[WIA Security Audit] ${event.type}:`, event.data);
      });
    }
  }

  private generateId(): string {
    return `${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
  }

  private getAudit(auditId: string): SecurityAudit {
    const audit = this.audits.get(auditId);
    if (!audit) {
      throw new Error(`Audit ${auditId} not found`);
    }
    return audit;
  }

  private getFinding(findingId: string): SecurityFinding {
    const finding = this.findings.get(findingId);
    if (!finding) {
      throw new Error(`Finding ${findingId} not found`);
    }
    return finding;
  }

  private async logActivity(
    auditId: string,
    action: AuditLogEntry['action'],
    actor: string,
    target: string,
    targetType: string,
    details: Record<string, any>
  ): Promise<void> {
    const logEntry: AuditLogEntry = {
      id: this.generateId(),
      auditId,
      timestamp: new Date(),
      action,
      actor,
      target,
      targetType,
      details,
      result: 'success',
      metadata: {},
    };

    this.auditLogs.push(logEntry);
  }

  private generateExecutiveSummary(audit: SecurityAudit, statistics: AuditStatistics): string {
    return `Security audit "${audit.name}" identified ${statistics.totalFindings} findings, including ${statistics.findingsBySeverity.critical} critical and ${statistics.findingsBySeverity.high} high severity issues. Current remediation progress: ${statistics.remediationProgress}%.`;
  }

  private calculateRiskScore(
    findingsBySeverity: Record<SeverityLevel, number>,
    totalFindings: number
  ): number {
    if (totalFindings === 0) return 0;

    const weights = { critical: 10, high: 5, medium: 2, low: 1, info: 0 };
    const weightedScore =
      findingsBySeverity.critical * weights.critical +
      findingsBySeverity.high * weights.high +
      findingsBySeverity.medium * weights.medium +
      findingsBySeverity.low * weights.low;

    const maxPossibleScore = totalFindings * weights.critical;
    return Math.min(100, Math.round((weightedScore / maxPossibleScore) * 100));
  }

  /**
   * Get audit logs
   */
  async getAuditLogs(auditId: string, limit?: number): Promise<AuditLogEntry[]> {
    const logs = this.auditLogs.filter((log) => log.auditId === auditId);
    return limit ? logs.slice(0, limit) : logs;
  }

  /**
   * Export audit data
   */
  async exportAudit(auditId: string): Promise<SecurityAudit> {
    return this.getAudit(auditId);
  }
}

// ============================================================================
// Factory Function
// ============================================================================

/**
 * Create a new WIA Security Audit instance
 */
export function createSecurityAudit(options?: SecurityAuditOptions): WIASecurityAudit {
  return new WIASecurityAudit(options);
}

// ============================================================================
// Type Exports
// ============================================================================

export * from './types';

// ============================================================================
// Interface Definitions
// ============================================================================

export interface SecurityAuditOptions {
  enableLogging?: boolean;
  defaultUser?: string;
  storageBackend?: 'memory' | 'database';
}

export interface CreateAuditParams {
  name: string;
  description: string;
  type: AuditType;
  scope: AuditScope;
  criteria?: AuditCriteria[];
  team?: SecurityAudit['team'];
  createdBy: string;
  metadata?: Record<string, any>;
}

export interface CreateFindingParams {
  auditId: string;
  type: FindingType;
  title: string;
  description: string;
  severity: SeverityLevel;
  category: string;
  location: SecurityFinding['location'];
  evidence?: Evidence[];
  recommendation: string;
  affectedComponents?: string[];
  discoveredBy: string;
  metadata?: Record<string, any>;
}

export interface ComplianceCheckParams {
  auditId: string;
  framework: ComplianceFramework;
  controlId: string;
  controlTitle: string;
  controlDescription: string;
  status: ComplianceCheck['status'];
  evidence?: Evidence[];
  findings?: string[];
  assessedBy: string;
  notes?: string;
  score?: number;
  metadata?: Record<string, any>;
}

export interface GenerateReportParams {
  auditId: string;
  type: ReportType;
  title?: string;
  executiveSummary?: string;
  methodology?: string;
  includeFindings?: boolean;
  includeCompliance?: boolean;
  complianceFramework?: ComplianceFramework;
  recommendations?: AuditReport['recommendations'];
  generatedBy: string;
  format?: AuditReport['format'];
  metadata?: Record<string, any>;
}

export interface CreateRemediationParams {
  findingId: string;
  title: string;
  description: string;
  steps: RemediationPlan['steps'];
  assignedTo: string;
  priority: RemediationPlan['priority'];
  effort: RemediationPlan['effort'];
  estimatedHours?: number;
  dueDate: Date;
  metadata?: Record<string, any>;
}
