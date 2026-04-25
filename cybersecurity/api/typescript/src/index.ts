/**
 * WIA Cybersecurity Standard - TypeScript SDK
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';

export * from './types';
import type {
  APIConfig, WIACybersecurityProject, ProjectResponse, ValidationResult, PaginatedResponse,
  RiskAssessment, RiskFinding, RegisteredRisk, TreatmentPlan, AccessPolicy, IdentityManagement,
  AuthenticationConfig, FirewallConfig, IntrusionDetection, NetworkZone, SecurityTesting,
  DataClassification, DlpPolicy, IncidentPlan, Playbook, SiemConfig, AlertingConfig,
  ComplianceControl, SecurityKpi
} from './types';

// ============================================================================
// WIA Cybersecurity Client
// ============================================================================

export class WIACybersecurityClient {
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
  async createProject(project: WIACybersecurityProject): Promise<ProjectResponse> {
    return (await this.axios.post<ProjectResponse>('/projects', project)).data;
  }

  async getProject(id: string): Promise<WIACybersecurityProject> {
    return (await this.axios.get<WIACybersecurityProject>(`/projects/${id}`)).data;
  }

  async listProjects(params?: { status?: string; organization?: string; limit?: number }): Promise<PaginatedResponse<ProjectResponse>> {
    return (await this.axios.get<PaginatedResponse<ProjectResponse>>('/projects', { params })).data;
  }

  async updateProject(id: string, updates: Partial<WIACybersecurityProject>): Promise<ProjectResponse> {
    return (await this.axios.put<ProjectResponse>(`/projects/${id}`, updates)).data;
  }

  async deleteProject(id: string): Promise<void> {
    await this.axios.delete(`/projects/${id}`);
  }

  // Risk Management
  async createRiskAssessment(projectId: string, assessment: Partial<RiskAssessment>): Promise<RiskAssessment> {
    return (await this.axios.post<RiskAssessment>(`/projects/${projectId}/risk/assessments`, assessment)).data;
  }

  async listRiskAssessments(projectId: string, params?: { status?: string }): Promise<RiskAssessment[]> {
    return (await this.axios.get<RiskAssessment[]>(`/projects/${projectId}/risk/assessments`, { params })).data;
  }

  async addRiskFinding(projectId: string, assessmentId: string, finding: Partial<RiskFinding>): Promise<RiskFinding> {
    return (await this.axios.post<RiskFinding>(`/projects/${projectId}/risk/assessments/${assessmentId}/findings`, finding)).data;
  }

  async listRisks(projectId: string, params?: { status?: string; category?: string }): Promise<RegisteredRisk[]> {
    return (await this.axios.get<RegisteredRisk[]>(`/projects/${projectId}/risk/registry`, { params })).data;
  }

  async updateRisk(projectId: string, riskId: string, updates: Partial<RegisteredRisk>): Promise<RegisteredRisk> {
    return (await this.axios.put<RegisteredRisk>(`/projects/${projectId}/risk/registry/${riskId}`, updates)).data;
  }

  async createTreatmentPlan(projectId: string, plan: Partial<TreatmentPlan>): Promise<TreatmentPlan> {
    return (await this.axios.post<TreatmentPlan>(`/projects/${projectId}/risk/treatment`, plan)).data;
  }

  async listTreatmentPlans(projectId: string, params?: { status?: string }): Promise<TreatmentPlan[]> {
    return (await this.axios.get<TreatmentPlan[]>(`/projects/${projectId}/risk/treatment`, { params })).data;
  }

  // Access Control Management
  async getAccessPolicy(projectId: string): Promise<AccessPolicy> {
    return (await this.axios.get<AccessPolicy>(`/projects/${projectId}/access/policy`)).data;
  }

  async updateAccessPolicy(projectId: string, policy: Partial<AccessPolicy>): Promise<AccessPolicy> {
    return (await this.axios.put<AccessPolicy>(`/projects/${projectId}/access/policy`, policy)).data;
  }

  async getIdentityConfig(projectId: string): Promise<IdentityManagement> {
    return (await this.axios.get<IdentityManagement>(`/projects/${projectId}/access/identity`)).data;
  }

  async updateIdentityConfig(projectId: string, config: Partial<IdentityManagement>): Promise<IdentityManagement> {
    return (await this.axios.put<IdentityManagement>(`/projects/${projectId}/access/identity`, config)).data;
  }

  async getAuthenticationConfig(projectId: string): Promise<AuthenticationConfig> {
    return (await this.axios.get<AuthenticationConfig>(`/projects/${projectId}/access/authentication`)).data;
  }

  async updateAuthenticationConfig(projectId: string, config: Partial<AuthenticationConfig>): Promise<AuthenticationConfig> {
    return (await this.axios.put<AuthenticationConfig>(`/projects/${projectId}/access/authentication`, config)).data;
  }

  // Network Security
  async listNetworkZones(projectId: string): Promise<NetworkZone[]> {
    return (await this.axios.get<NetworkZone[]>(`/projects/${projectId}/network/zones`)).data;
  }

  async createNetworkZone(projectId: string, zone: Partial<NetworkZone>): Promise<NetworkZone> {
    return (await this.axios.post<NetworkZone>(`/projects/${projectId}/network/zones`, zone)).data;
  }

  async updateNetworkZone(projectId: string, zoneId: string, updates: Partial<NetworkZone>): Promise<NetworkZone> {
    return (await this.axios.put<NetworkZone>(`/projects/${projectId}/network/zones/${zoneId}`, updates)).data;
  }

  async listFirewalls(projectId: string): Promise<FirewallConfig[]> {
    return (await this.axios.get<FirewallConfig[]>(`/projects/${projectId}/network/firewalls`)).data;
  }

  async configureFirewall(projectId: string, firewallId: string, config: Partial<FirewallConfig>): Promise<FirewallConfig> {
    return (await this.axios.put<FirewallConfig>(`/projects/${projectId}/network/firewalls/${firewallId}`, config)).data;
  }

  async getIdsConfig(projectId: string): Promise<IntrusionDetection> {
    return (await this.axios.get<IntrusionDetection>(`/projects/${projectId}/network/ids`)).data;
  }

  async updateIdsConfig(projectId: string, config: Partial<IntrusionDetection>): Promise<IntrusionDetection> {
    return (await this.axios.put<IntrusionDetection>(`/projects/${projectId}/network/ids`, config)).data;
  }

  // Application Security
  async getSecurityTestingConfig(projectId: string): Promise<SecurityTesting> {
    return (await this.axios.get<SecurityTesting>(`/projects/${projectId}/appsec/testing`)).data;
  }

  async updateSecurityTestingConfig(projectId: string, config: Partial<SecurityTesting>): Promise<SecurityTesting> {
    return (await this.axios.put<SecurityTesting>(`/projects/${projectId}/appsec/testing`, config)).data;
  }

  async triggerSecurityScan(projectId: string, scanType: 'sast' | 'dast' | 'dependency'): Promise<{ scanId: string; status: string }> {
    return (await this.axios.post<{ scanId: string; status: string }>(`/projects/${projectId}/appsec/scan`, { type: scanType })).data;
  }

  async getScanResults(projectId: string, scanId: string): Promise<{ findings: unknown[]; summary: Record<string, number> }> {
    return (await this.axios.get<{ findings: unknown[]; summary: Record<string, number> }>(`/projects/${projectId}/appsec/scan/${scanId}`)).data;
  }

  // Data Security
  async getDataClassification(projectId: string): Promise<DataClassification> {
    return (await this.axios.get<DataClassification>(`/projects/${projectId}/data/classification`)).data;
  }

  async updateDataClassification(projectId: string, config: Partial<DataClassification>): Promise<DataClassification> {
    return (await this.axios.put<DataClassification>(`/projects/${projectId}/data/classification`, config)).data;
  }

  async listDlpPolicies(projectId: string): Promise<DlpPolicy[]> {
    return (await this.axios.get<DlpPolicy[]>(`/projects/${projectId}/data/dlp`)).data;
  }

  async createDlpPolicy(projectId: string, policy: Partial<DlpPolicy>): Promise<DlpPolicy> {
    return (await this.axios.post<DlpPolicy>(`/projects/${projectId}/data/dlp`, policy)).data;
  }

  async updateDlpPolicy(projectId: string, policyId: string, updates: Partial<DlpPolicy>): Promise<DlpPolicy> {
    return (await this.axios.put<DlpPolicy>(`/projects/${projectId}/data/dlp/${policyId}`, updates)).data;
  }

  // Incident Response
  async getIncidentPlan(projectId: string): Promise<IncidentPlan> {
    return (await this.axios.get<IncidentPlan>(`/projects/${projectId}/incident/plan`)).data;
  }

  async updateIncidentPlan(projectId: string, plan: Partial<IncidentPlan>): Promise<IncidentPlan> {
    return (await this.axios.put<IncidentPlan>(`/projects/${projectId}/incident/plan`, plan)).data;
  }

  async listPlaybooks(projectId: string, params?: { severity?: string }): Promise<Playbook[]> {
    return (await this.axios.get<Playbook[]>(`/projects/${projectId}/incident/playbooks`, { params })).data;
  }

  async createPlaybook(projectId: string, playbook: Partial<Playbook>): Promise<Playbook> {
    return (await this.axios.post<Playbook>(`/projects/${projectId}/incident/playbooks`, playbook)).data;
  }

  async executePlaybook(projectId: string, playbookId: string, context: Record<string, unknown>): Promise<{ executionId: string; status: string }> {
    return (await this.axios.post<{ executionId: string; status: string }>(`/projects/${projectId}/incident/playbooks/${playbookId}/execute`, context)).data;
  }

  async reportIncident(projectId: string, incident: { title: string; severity: string; description: string; indicators?: string[] }): Promise<{ incidentId: string; status: string }> {
    return (await this.axios.post<{ incidentId: string; status: string }>(`/projects/${projectId}/incident/report`, incident)).data;
  }

  // Monitoring & SIEM
  async getSiemConfig(projectId: string): Promise<SiemConfig> {
    return (await this.axios.get<SiemConfig>(`/projects/${projectId}/monitoring/siem`)).data;
  }

  async updateSiemConfig(projectId: string, config: Partial<SiemConfig>): Promise<SiemConfig> {
    return (await this.axios.put<SiemConfig>(`/projects/${projectId}/monitoring/siem`, config)).data;
  }

  async getAlertingConfig(projectId: string): Promise<AlertingConfig> {
    return (await this.axios.get<AlertingConfig>(`/projects/${projectId}/monitoring/alerting`)).data;
  }

  async updateAlertingConfig(projectId: string, config: Partial<AlertingConfig>): Promise<AlertingConfig> {
    return (await this.axios.put<AlertingConfig>(`/projects/${projectId}/monitoring/alerting`, config)).data;
  }

  async listAlerts(projectId: string, params?: { severity?: string; status?: string; limit?: number }): Promise<PaginatedResponse<{ id: string; type: string; severity: string; timestamp: string }>> {
    return (await this.axios.get<PaginatedResponse<{ id: string; type: string; severity: string; timestamp: string }>>(`/projects/${projectId}/monitoring/alerts`, { params })).data;
  }

  async acknowledgeAlert(projectId: string, alertId: string, notes?: string): Promise<void> {
    await this.axios.post(`/projects/${projectId}/monitoring/alerts/${alertId}/ack`, { notes });
  }

  // Compliance
  async listComplianceControls(projectId: string, params?: { framework?: string; status?: string }): Promise<ComplianceControl[]> {
    return (await this.axios.get<ComplianceControl[]>(`/projects/${projectId}/compliance/controls`, { params })).data;
  }

  async updateComplianceControl(projectId: string, controlId: string, updates: Partial<ComplianceControl>): Promise<ComplianceControl> {
    return (await this.axios.put<ComplianceControl>(`/projects/${projectId}/compliance/controls/${controlId}`, updates)).data;
  }

  async generateComplianceReport(projectId: string, framework: string): Promise<{ reportId: string; status: string }> {
    return (await this.axios.post<{ reportId: string; status: string }>(`/projects/${projectId}/compliance/report`, { framework })).data;
  }

  async getComplianceStatus(projectId: string): Promise<{ overall: number; byFramework: Record<string, number>; gaps: number }> {
    return (await this.axios.get<{ overall: number; byFramework: Record<string, number>; gaps: number }>(`/projects/${projectId}/compliance/status`)).data;
  }

  // Security Metrics
  async getSecurityKpis(projectId: string): Promise<SecurityKpi[]> {
    return (await this.axios.get<SecurityKpi[]>(`/projects/${projectId}/metrics/kpis`)).data;
  }

  async getSecurityDashboard(projectId: string): Promise<SecurityDashboard> {
    return (await this.axios.get<SecurityDashboard>(`/projects/${projectId}/dashboard`)).data;
  }

  // Validation
  validateProject(project: WIACybersecurityProject): ValidationResult {
    const errors: { path: string; message: string }[] = [];

    if (!project.standard || project.standard !== 'WIA-CYBERSECURITY') {
      errors.push({ path: 'standard', message: 'Standard must be "WIA-CYBERSECURITY"' });
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
    if (!project.riskManagement) {
      errors.push({ path: 'riskManagement', message: 'Risk management configuration is required' });
    }
    if (!project.accessControl) {
      errors.push({ path: 'accessControl', message: 'Access control configuration is required' });
    }

    return { valid: errors.length === 0, errors: errors.length > 0 ? errors : undefined };
  }
}

// ============================================================================
// Supporting Types
// ============================================================================

export interface SecurityDashboard {
  riskScore: number;
  openRisks: number;
  complianceScore: number;
  activeIncidents: number;
  recentAlerts: number;
  vulnerabilities: { critical: number; high: number; medium: number; low: number };
  trends: { metric: string; direction: 'up' | 'down' | 'stable'; change: number }[];
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

export function createMinimalProject(name: string, organization: string): WIACybersecurityProject {
  return {
    standard: 'WIA-CYBERSECURITY',
    version: '1.0.0',
    metadata: {
      id: generateUUID(),
      name,
      organization: { name: organization, industry: 'technology', size: 'medium', region: 'US', contact: { name: '', email: '', role: 'Security Officer' } },
      scope: { assets: [], networks: [], applications: [], dataClassifications: [] },
      createdAt: new Date().toISOString(),
      status: 'active',
    },
    riskManagement: {
      framework: { methodology: 'NIST', riskAppetite: 'medium', reviewFrequency: 'quarterly', escalationCriteria: [] },
      assessments: [],
      registry: { risks: [], lastUpdated: new Date().toISOString(), owner: '' },
      treatment: { plans: [], budget: 0, timeline: '' },
    },
    accessControl: {
      policy: { model: 'RBAC', principles: ['least-privilege', 'separation-of-duties'], reviewFrequency: 'quarterly', exceptions: [] },
      identityManagement: { provider: '', lifecycle: { onboarding: [], changes: [], offboarding: [], reviewFrequency: 'quarterly' }, federation: { enabled: false, protocols: [], trustedProviders: [] }, provisioning: { automated: false, approvalWorkflow: true, defaultRoles: [] } },
      authentication: { methods: [], mfaRequired: true, passwordPolicy: { minLength: 12, complexity: ['uppercase', 'lowercase', 'number', 'special'], expiryDays: 90, historyCount: 12, lockoutThreshold: 5 }, sessionManagement: { timeout: 30, maxConcurrent: 3, secure: true } },
      authorization: { engine: '', policies: [], enforcement: 'strict' },
      privilegedAccess: { enabled: true, vault: '', justInTime: true, sessionRecording: true, approvalRequired: true },
    },
    networkSecurity: {
      architecture: { topology: 'segmented', zones: [], dmz: true, cloudConnectivity: [] },
      segmentation: { vlans: [], microsegmentation: false, sdnEnabled: false },
      perimeter: { firewalls: [], ids: { type: 'IPS', mode: 'inline', signatureUpdates: 'daily', coverage: [] }, waf: { enabled: true, mode: 'block', rulesets: [], customRules: 0 }, ddosProtection: { enabled: true, provider: '', mitigationCapacity: '', autoMitigation: true } },
      encryption: { inTransit: { required: true, protocols: ['TLS 1.3'], minimumVersion: 'TLS 1.2' }, vpn: { type: 'both', protocol: 'IPSec', encryption: 'AES-256' }, tlsConfig: { minimumVersion: '1.2', cipherSuites: [], certificateManagement: '' } },
      monitoring: { netflow: true, packetCapture: false, anomalyDetection: true, retentionDays: 90 },
    },
    applicationSecurity: {
      sdlc: { methodology: 'secure-sdlc', securityGates: [], training: '', codeReview: { required: true, tools: [], coverage: 0 } },
      testing: { sast: { enabled: true, tools: [], frequency: 'per-commit', coverage: 0 }, dast: { enabled: true, tools: [], frequency: 'weekly', coverage: 0 }, penetrationTesting: { frequency: 'annual', scope: [] }, bugBounty: { enabled: false, scope: [], rewards: [] } },
      runtime: { rasp: false, containerSecurity: { scanning: true, runtime: true, policies: [] }, secrets: { vault: '', rotation: true, scanning: true } },
      apiSecurity: { authentication: ['OAuth2'], rateLimit: true, inputValidation: true, logging: true },
    },
    dataSecurity: {
      classification: { scheme: 'default', levels: [], labeling: true, automation: false },
      encryption: { atRest: { enabled: true, algorithm: 'AES-256', keyLength: 256 }, inUse: { enabled: false, algorithm: '', keyLength: 0 }, keyManagement: { provider: '', rotation: true, rotationPeriod: 'annual', hsm: false } },
      dlp: { enabled: true, channels: ['email', 'web', 'endpoint'], policies: [] },
      backup: { frequency: 'daily', retention: '30 days', encryption: true, offsite: true, testing: 'quarterly' },
      retention: { default: '7 years', byClassification: [], legalHold: false },
    },
    incidentResponse: {
      plan: { version: '1.0', lastReview: new Date().toISOString(), phases: [], escalation: { levels: [], timeframes: [] } },
      team: { structure: '', members: [], onCall: '', training: '', exercises: '' },
      playbooks: [],
      communication: { internal: { channels: [], templates: [] }, external: { stakeholders: [], protocols: [] }, regulatory: { authorities: [], timeframes: [] } },
      forensics: { tools: [], chain: '', preservation: '', analysis: [] },
    },
    compliance: {
      regulations: [],
      standards: [],
      controls: [],
      audits: { internal: { frequency: 'annual', scope: [] }, external: { frequency: 'annual', auditor: '' }, certifications: [] },
      reporting: { frequency: 'quarterly', recipients: [], format: 'pdf', automation: false },
    },
    monitoring: {
      siem: { platform: '', sources: [], retentionDays: 365, correlationRules: 0 },
      logging: { centralized: true, sources: [], format: 'json', integrity: true },
      alerting: { channels: [], severity: [], suppressionRules: 0 },
      metrics: { kpis: [], dashboards: [], reporting: '' },
    },
  };
}

export default { WIACybersecurityClient, generateUUID, createMinimalProject };
