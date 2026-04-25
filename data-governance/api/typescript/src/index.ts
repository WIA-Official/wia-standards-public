/**
 * WIA Data Governance Standard - TypeScript SDK
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';

export * from './types';
import type {
  APIConfig, WIADataGovernanceProject, ProjectResponse, ValidationResult, PaginatedResponse,
  GovernanceCouncil, Committee, GovernanceRole, GovernanceProcess, DataAsset, DataSteward,
  Policy, Standard, Procedure, QualityRule, QualityDimension, GovernancePolicy,
  PrivacyPrinciple, SubjectRight, ConsentPurpose, Regulation, Control, KPI, Dashboard
} from './types';

// ============================================================================
// WIA Data Governance Client
// ============================================================================

export class WIADataGovernanceClient {
  private axios: AxiosInstance;

  constructor(config: APIConfig) {
    this.axios = axios.create({
      baseURL: config.baseURL,
      timeout: config.timeout || 30000,
      headers: {
        'Content-Type': 'application/json',
        ...(config.apiKey && { Authorization: `Bearer ${config.apiKey}` }),
      },
    });
  }

  // Project Management
  async createProject(project: WIADataGovernanceProject): Promise<ProjectResponse> {
    return (await this.axios.post<ProjectResponse>('/projects', project)).data;
  }

  async getProject(id: string): Promise<WIADataGovernanceProject> {
    return (await this.axios.get<WIADataGovernanceProject>(`/projects/${id}`)).data;
  }

  async listProjects(params?: { status?: string; organization?: string; limit?: number }): Promise<PaginatedResponse<ProjectResponse>> {
    return (await this.axios.get<PaginatedResponse<ProjectResponse>>('/projects', { params })).data;
  }

  async updateProject(id: string, updates: Partial<WIADataGovernanceProject>): Promise<ProjectResponse> {
    return (await this.axios.put<ProjectResponse>(`/projects/${id}`, updates)).data;
  }

  async deleteProject(id: string): Promise<void> {
    await this.axios.delete(`/projects/${id}`);
  }

  // Governance Framework
  async getCouncil(projectId: string): Promise<GovernanceCouncil> {
    return (await this.axios.get<GovernanceCouncil>(`/projects/${projectId}/framework/council`)).data;
  }

  async updateCouncil(projectId: string, council: Partial<GovernanceCouncil>): Promise<GovernanceCouncil> {
    return (await this.axios.put<GovernanceCouncil>(`/projects/${projectId}/framework/council`, council)).data;
  }

  async listCommittees(projectId: string): Promise<Committee[]> {
    return (await this.axios.get<Committee[]>(`/projects/${projectId}/framework/committees`)).data;
  }

  async createCommittee(projectId: string, committee: Partial<Committee>): Promise<Committee> {
    return (await this.axios.post<Committee>(`/projects/${projectId}/framework/committees`, committee)).data;
  }

  async updateCommittee(projectId: string, committeeId: string, updates: Partial<Committee>): Promise<Committee> {
    return (await this.axios.put<Committee>(`/projects/${projectId}/framework/committees/${committeeId}`, updates)).data;
  }

  async listRoles(projectId: string): Promise<GovernanceRole[]> {
    return (await this.axios.get<GovernanceRole[]>(`/projects/${projectId}/framework/roles`)).data;
  }

  async createRole(projectId: string, role: Partial<GovernanceRole>): Promise<GovernanceRole> {
    return (await this.axios.post<GovernanceRole>(`/projects/${projectId}/framework/roles`, role)).data;
  }

  async listProcesses(projectId: string): Promise<GovernanceProcess[]> {
    return (await this.axios.get<GovernanceProcess[]>(`/projects/${projectId}/framework/processes`)).data;
  }

  async createProcess(projectId: string, process: Partial<GovernanceProcess>): Promise<GovernanceProcess> {
    return (await this.axios.post<GovernanceProcess>(`/projects/${projectId}/framework/processes`, process)).data;
  }

  async getMaturityAssessment(projectId: string): Promise<MaturityAssessmentResult> {
    return (await this.axios.get<MaturityAssessmentResult>(`/projects/${projectId}/framework/maturity`)).data;
  }

  async updateMaturityAssessment(projectId: string, assessment: Partial<MaturityAssessmentResult>): Promise<MaturityAssessmentResult> {
    return (await this.axios.put<MaturityAssessmentResult>(`/projects/${projectId}/framework/maturity`, assessment)).data;
  }

  // Data Asset Management
  async listDataAssets(projectId: string, params?: { domain?: string; classification?: string; owner?: string }): Promise<PaginatedResponse<DataAsset>> {
    return (await this.axios.get<PaginatedResponse<DataAsset>>(`/projects/${projectId}/assets`, { params })).data;
  }

  async createDataAsset(projectId: string, asset: Partial<DataAsset>): Promise<DataAsset> {
    return (await this.axios.post<DataAsset>(`/projects/${projectId}/assets`, asset)).data;
  }

  async getDataAsset(projectId: string, assetId: string): Promise<DataAsset> {
    return (await this.axios.get<DataAsset>(`/projects/${projectId}/assets/${assetId}`)).data;
  }

  async updateDataAsset(projectId: string, assetId: string, updates: Partial<DataAsset>): Promise<DataAsset> {
    return (await this.axios.put<DataAsset>(`/projects/${projectId}/assets/${assetId}`, updates)).data;
  }

  async classifyDataAsset(projectId: string, assetId: string, classification: string, sensitivity: string): Promise<DataAsset> {
    return (await this.axios.post<DataAsset>(`/projects/${projectId}/assets/${assetId}/classify`, { classification, sensitivity })).data;
  }

  async getDataLineage(projectId: string, assetId: string): Promise<LineageResult> {
    return (await this.axios.get<LineageResult>(`/projects/${projectId}/assets/${assetId}/lineage`)).data;
  }

  // Data Stewardship
  async listStewards(projectId: string, params?: { domain?: string }): Promise<DataSteward[]> {
    return (await this.axios.get<DataSteward[]>(`/projects/${projectId}/stewardship/stewards`, { params })).data;
  }

  async createSteward(projectId: string, steward: Partial<DataSteward>): Promise<DataSteward> {
    return (await this.axios.post<DataSteward>(`/projects/${projectId}/stewardship/stewards`, steward)).data;
  }

  async assignSteward(projectId: string, assetId: string, stewardId: string): Promise<DataAsset> {
    return (await this.axios.post<DataAsset>(`/projects/${projectId}/assets/${assetId}/assign-steward`, { stewardId })).data;
  }

  async getStewardshipMetrics(projectId: string, stewardId: string): Promise<StewardshipMetrics> {
    return (await this.axios.get<StewardshipMetrics>(`/projects/${projectId}/stewardship/stewards/${stewardId}/metrics`)).data;
  }

  // Policy Management
  async listPolicies(projectId: string, params?: { type?: string; status?: string }): Promise<Policy[]> {
    return (await this.axios.get<Policy[]>(`/projects/${projectId}/policies`, { params })).data;
  }

  async createPolicy(projectId: string, policy: Partial<Policy>): Promise<Policy> {
    return (await this.axios.post<Policy>(`/projects/${projectId}/policies`, policy)).data;
  }

  async getPolicy(projectId: string, policyId: string): Promise<Policy> {
    return (await this.axios.get<Policy>(`/projects/${projectId}/policies/${policyId}`)).data;
  }

  async updatePolicy(projectId: string, policyId: string, updates: Partial<Policy>): Promise<Policy> {
    return (await this.axios.put<Policy>(`/projects/${projectId}/policies/${policyId}`, updates)).data;
  }

  async approvePolicy(projectId: string, policyId: string, approver: string): Promise<Policy> {
    return (await this.axios.post<Policy>(`/projects/${projectId}/policies/${policyId}/approve`, { approver })).data;
  }

  async retirePolicy(projectId: string, policyId: string): Promise<Policy> {
    return (await this.axios.post<Policy>(`/projects/${projectId}/policies/${policyId}/retire`)).data;
  }

  async listStandards(projectId: string): Promise<Standard[]> {
    return (await this.axios.get<Standard[]>(`/projects/${projectId}/policies/standards`)).data;
  }

  async createStandard(projectId: string, standard: Partial<Standard>): Promise<Standard> {
    return (await this.axios.post<Standard>(`/projects/${projectId}/policies/standards`, standard)).data;
  }

  async listProcedures(projectId: string): Promise<Procedure[]> {
    return (await this.axios.get<Procedure[]>(`/projects/${projectId}/policies/procedures`)).data;
  }

  async createProcedure(projectId: string, procedure: Partial<Procedure>): Promise<Procedure> {
    return (await this.axios.post<Procedure>(`/projects/${projectId}/policies/procedures`, procedure)).data;
  }

  // Data Quality
  async listQualityDimensions(projectId: string): Promise<QualityDimension[]> {
    return (await this.axios.get<QualityDimension[]>(`/projects/${projectId}/quality/dimensions`)).data;
  }

  async createQualityDimension(projectId: string, dimension: Partial<QualityDimension>): Promise<QualityDimension> {
    return (await this.axios.post<QualityDimension>(`/projects/${projectId}/quality/dimensions`, dimension)).data;
  }

  async listQualityRules(projectId: string, params?: { dimension?: string; scope?: string }): Promise<QualityRule[]> {
    return (await this.axios.get<QualityRule[]>(`/projects/${projectId}/quality/rules`, { params })).data;
  }

  async createQualityRule(projectId: string, rule: Partial<QualityRule>): Promise<QualityRule> {
    return (await this.axios.post<QualityRule>(`/projects/${projectId}/quality/rules`, rule)).data;
  }

  async runQualityAssessment(projectId: string, assetIds: string[]): Promise<QualityAssessmentResult> {
    return (await this.axios.post<QualityAssessmentResult>(`/projects/${projectId}/quality/assess`, { assetIds })).data;
  }

  async getQualityScore(projectId: string, assetId: string): Promise<QualityScoreResult> {
    return (await this.axios.get<QualityScoreResult>(`/projects/${projectId}/assets/${assetId}/quality`)).data;
  }

  async getQualityTrend(projectId: string, params?: { dimension?: string; period?: string }): Promise<QualityTrendResult[]> {
    return (await this.axios.get<QualityTrendResult[]>(`/projects/${projectId}/quality/trend`, { params })).data;
  }

  async listQualityIssues(projectId: string, params?: { severity?: string; status?: string }): Promise<PaginatedResponse<QualityIssue>> {
    return (await this.axios.get<PaginatedResponse<QualityIssue>>(`/projects/${projectId}/quality/issues`, { params })).data;
  }

  async createQualityIssue(projectId: string, issue: Partial<QualityIssue>): Promise<QualityIssue> {
    return (await this.axios.post<QualityIssue>(`/projects/${projectId}/quality/issues`, issue)).data;
  }

  async resolveQualityIssue(projectId: string, issueId: string, resolution: string): Promise<QualityIssue> {
    return (await this.axios.post<QualityIssue>(`/projects/${projectId}/quality/issues/${issueId}/resolve`, { resolution })).data;
  }

  // Privacy Management
  async listPrivacyPrinciples(projectId: string): Promise<PrivacyPrinciple[]> {
    return (await this.axios.get<PrivacyPrinciple[]>(`/projects/${projectId}/privacy/principles`)).data;
  }

  async listSubjectRights(projectId: string): Promise<SubjectRight[]> {
    return (await this.axios.get<SubjectRight[]>(`/projects/${projectId}/privacy/rights`)).data;
  }

  async submitRightsRequest(projectId: string, request: RightsRequest): Promise<RightsRequestResult> {
    return (await this.axios.post<RightsRequestResult>(`/projects/${projectId}/privacy/rights/request`, request)).data;
  }

  async getRightsRequestStatus(projectId: string, requestId: string): Promise<RightsRequestResult> {
    return (await this.axios.get<RightsRequestResult>(`/projects/${projectId}/privacy/rights/request/${requestId}`)).data;
  }

  async listConsentPurposes(projectId: string): Promise<ConsentPurpose[]> {
    return (await this.axios.get<ConsentPurpose[]>(`/projects/${projectId}/privacy/consent/purposes`)).data;
  }

  async recordConsent(projectId: string, consent: ConsentRecord): Promise<ConsentRecord> {
    return (await this.axios.post<ConsentRecord>(`/projects/${projectId}/privacy/consent`, consent)).data;
  }

  async withdrawConsent(projectId: string, subjectId: string, purposeId: string): Promise<void> {
    await this.axios.post(`/projects/${projectId}/privacy/consent/withdraw`, { subjectId, purposeId });
  }

  async conductPrivacyImpactAssessment(projectId: string, assetId: string): Promise<PIAResult> {
    return (await this.axios.post<PIAResult>(`/projects/${projectId}/privacy/pia`, { assetId })).data;
  }

  // Compliance Management
  async listRegulations(projectId: string): Promise<Regulation[]> {
    return (await this.axios.get<Regulation[]>(`/projects/${projectId}/compliance/regulations`)).data;
  }

  async addRegulation(projectId: string, regulation: Partial<Regulation>): Promise<Regulation> {
    return (await this.axios.post<Regulation>(`/projects/${projectId}/compliance/regulations`, regulation)).data;
  }

  async listControls(projectId: string, params?: { regulation?: string; status?: string }): Promise<Control[]> {
    return (await this.axios.get<Control[]>(`/projects/${projectId}/compliance/controls`, { params })).data;
  }

  async createControl(projectId: string, control: Partial<Control>): Promise<Control> {
    return (await this.axios.post<Control>(`/projects/${projectId}/compliance/controls`, control)).data;
  }

  async updateControlStatus(projectId: string, controlId: string, status: string, evidence?: string[]): Promise<Control> {
    return (await this.axios.put<Control>(`/projects/${projectId}/compliance/controls/${controlId}`, { status, evidence })).data;
  }

  async getComplianceStatus(projectId: string): Promise<ComplianceStatusResult> {
    return (await this.axios.get<ComplianceStatusResult>(`/projects/${projectId}/compliance/status`)).data;
  }

  async generateComplianceReport(projectId: string, regulations: string[]): Promise<{ reportId: string; status: string }> {
    return (await this.axios.post<{ reportId: string; status: string }>(`/projects/${projectId}/compliance/report`, { regulations })).data;
  }

  // Metrics & Dashboards
  async listKPIs(projectId: string): Promise<KPI[]> {
    return (await this.axios.get<KPI[]>(`/projects/${projectId}/metrics/kpis`)).data;
  }

  async createKPI(projectId: string, kpi: Partial<KPI>): Promise<KPI> {
    return (await this.axios.post<KPI>(`/projects/${projectId}/metrics/kpis`, kpi)).data;
  }

  async getKPIValue(projectId: string, kpiId: string): Promise<KPIValue> {
    return (await this.axios.get<KPIValue>(`/projects/${projectId}/metrics/kpis/${kpiId}/value`)).data;
  }

  async listDashboards(projectId: string): Promise<Dashboard[]> {
    return (await this.axios.get<Dashboard[]>(`/projects/${projectId}/metrics/dashboards`)).data;
  }

  async createDashboard(projectId: string, dashboard: Partial<Dashboard>): Promise<Dashboard> {
    return (await this.axios.post<Dashboard>(`/projects/${projectId}/metrics/dashboards`, dashboard)).data;
  }

  async getGovernanceDashboard(projectId: string): Promise<GovernanceDashboard> {
    return (await this.axios.get<GovernanceDashboard>(`/projects/${projectId}/dashboard`)).data;
  }

  // Validation
  validateProject(project: WIADataGovernanceProject): ValidationResult {
    const errors: { path: string; message: string }[] = [];

    if (!project.standard || project.standard !== 'WIA-DATA-GOVERNANCE') {
      errors.push({ path: 'standard', message: 'Standard must be "WIA-DATA-GOVERNANCE"' });
    }
    if (!project.version) {
      errors.push({ path: 'version', message: 'Version is required' });
    }
    if (!project.metadata?.id) {
      errors.push({ path: 'metadata.id', message: 'Project ID is required' });
    }
    if (!project.metadata?.name) {
      errors.push({ path: 'metadata.name', message: 'Project name is required' });
    }
    if (!project.framework) {
      errors.push({ path: 'framework', message: 'Governance framework is required' });
    }

    return { valid: errors.length === 0, errors: errors.length > 0 ? errors : undefined };
  }
}

// ============================================================================
// Supporting Types
// ============================================================================

export interface MaturityAssessmentResult {
  currentLevel: number;
  targetLevel: number;
  dimensions: { name: string; score: number; target: number }[];
  recommendations: string[];
  assessmentDate: string;
}

export interface LineageResult {
  asset: DataAsset;
  upstream: { asset: DataAsset; relationship: string }[];
  downstream: { asset: DataAsset; relationship: string }[];
}

export interface StewardshipMetrics {
  stewardId: string;
  assetsManaged: number;
  qualityScore: number;
  issuesResolved: number;
  lastActivity: string;
}

export interface QualityAssessmentResult {
  timestamp: string;
  assets: { assetId: string; score: number; issues: number }[];
  overallScore: number;
  duration: number;
}

export interface QualityScoreResult {
  assetId: string;
  overall: number;
  dimensions: { name: string; score: number }[];
  trend: 'up' | 'down' | 'stable';
  lastAssessed: string;
}

export interface QualityTrendResult {
  timestamp: string;
  score: number;
  dimension?: string;
}

export interface QualityIssue {
  id: string;
  assetId: string;
  dimension: string;
  rule: string;
  severity: string;
  description: string;
  status: 'open' | 'in-progress' | 'resolved';
  createdAt: string;
  resolvedAt?: string;
}

export interface RightsRequest {
  subjectId: string;
  right: string;
  details?: string;
}

export interface RightsRequestResult {
  id: string;
  subjectId: string;
  right: string;
  status: 'pending' | 'processing' | 'completed' | 'denied';
  createdAt: string;
  completedAt?: string;
}

export interface ConsentRecord {
  subjectId: string;
  purposeId: string;
  granted: boolean;
  timestamp: string;
  method: string;
  evidence?: string;
}

export interface PIAResult {
  assetId: string;
  riskLevel: 'low' | 'medium' | 'high';
  findings: { category: string; risk: string; mitigation: string }[];
  recommendations: string[];
  assessmentDate: string;
}

export interface ComplianceStatusResult {
  overall: number;
  byRegulation: { regulation: string; score: number; gaps: number }[];
  controlsImplemented: number;
  controlsPending: number;
  lastAudit: string;
}

export interface KPIValue {
  kpiId: string;
  current: number;
  target: number;
  trend: 'up' | 'down' | 'stable';
  history: { timestamp: string; value: number }[];
}

export interface GovernanceDashboard {
  maturity: { current: number; target: number };
  quality: { score: number; trend: string };
  compliance: { score: number; gaps: number };
  stewardship: { coverage: number; activeIssues: number };
  privacy: { requests: number; avgResponseTime: string };
}

// ============================================================================
// Utility Functions
// ============================================================================

export function generateUUID(): string {
  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => {
    const r = (Math.random() * 16) | 0;
    return (c === 'x' ? r : (r & 0x3) | 0x8).toString(16);
  });
}

export function createMinimalProject(name: string, organization: string): WIADataGovernanceProject {
  return {
    standard: 'WIA-DATA-GOVERNANCE',
    version: '1.0.0',
    metadata: {
      id: generateUUID(),
      name,
      organization: { name: organization, industry: 'technology', size: 'medium', regions: ['US'], contact: { name: '', email: '', role: 'Data Governance Lead' } },
      scope: { domains: [], systems: [], dataTypes: [] },
      createdAt: new Date().toISOString(),
      status: 'active',
    },
    framework: {
      model: { type: 'federated', principles: [], objectives: [], charter: '' },
      structure: { council: { name: 'Data Governance Council', charter: '', members: [], meetingFrequency: 'monthly', decisionProcess: '' }, committees: [], roles: [], raci: { activities: [], roles: [] } },
      processes: [],
      decisionRights: { categories: [], escalation: { levels: [], timeframes: [] }, delegation: { allowed: true, conditions: [], documentation: true } },
      maturity: { currentLevel: 1, targetLevel: 3, dimensions: [], roadmap: { phases: [], timeline: '' } },
    },
    dataAssets: {
      inventory: { sources: [], assets: [], cataloging: { automation: true, frequency: 'daily', attributes: [], validation: true } },
      classification: { levels: [], criteria: [], defaultLevel: 'internal', reviewFrequency: 'annually' },
      lifecycle: { stages: [], retention: [], archival: { criteria: [], format: '', location: '', accessibility: '' }, disposal: { methods: [], approval: '', verification: true, documentation: true } },
      lineage: { enabled: true, granularity: 'table', sources: [], visualization: true },
    },
    policies: {
      framework: { hierarchy: { levels: [], inheritance: true }, lifecycle: { stages: ['draft', 'review', 'approved', 'published', 'retired'], reviewFrequency: 'annually', versionControl: true }, enforcement: { methods: ['technical', 'procedural'], exceptions: { allowed: true, approvers: [], documentation: [], maxDuration: '90d' }, violations: { detection: [], notification: [], remediation: '', consequences: [] } } },
      policies: [],
      standards: [],
      procedures: [],
      guidelines: [],
    },
    stewardship: {
      model: { type: 'domain', reporting: '', authority: [] },
      stewards: [],
      responsibilities: [],
      training: { courses: [], certification: true, refresher: 'annually' },
    },
    quality: {
      dimensions: [],
      rules: [],
      monitoring: { frequency: 'daily', automation: true, dashboards: [], alerts: [] },
      improvement: { identification: [], prioritization: '', remediation: '', tracking: true },
    },
    privacy: {
      principles: [],
      rights: { rights: [], processes: [], sla: '30d' },
      consent: { purposes: [], collection: [], withdrawal: { methods: [], timeline: '24h', impact: '' }, records: { retention: '7y', attributes: [], audit: true } },
      impact: { triggers: [], process: { steps: [], stakeholders: [], approval: '' }, documentation: '' },
      incidents: { classification: [], response: { team: [], procedures: [], timeline: '' }, notification: { authorities: [], subjects: { criteria: '', timeline: '', content: [] } } },
    },
    compliance: {
      regulations: [],
      controls: [],
      audits: { internal: { frequency: 'quarterly', scope: [] }, external: { frequency: 'annually', auditor: '' }, findings: { classification: [], remediation: '', tracking: true } },
      reporting: { frequency: 'quarterly', recipients: [], content: [] },
    },
    metrics: {
      kpis: [],
      dashboards: [],
      reporting: { scheduled: [], adhoc: true, distribution: [] },
      benchmarking: { enabled: false, sources: [], frequency: '', dimensions: [] },
    },
  };
}

export default { WIADataGovernanceClient, generateUUID, createMinimalProject };
