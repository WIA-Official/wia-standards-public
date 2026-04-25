/**
 * WIA Cryo Legal Standard - TypeScript SDK
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';

export * from './types';
import type { APIConfig, WIACryoLegalProject, ProjectResponse, ValidationResult, PaginatedResponse, Contract, ContractTemplate, Dispute, Regulation, ComplianceRequirement, ContractEvent } from './types';

// ============================================================================
// WIA Cryo Legal Client
// ============================================================================

export class WIACryoLegalClient {
  private axios: AxiosInstance;

  constructor(config: APIConfig) {
    this.axios = axios.create({
      baseURL: config.baseURL,
      timeout: config.timeout || 30000,
      headers: { 'Content-Type': 'application/json', ...(config.apiKey && { Authorization: `Bearer ${config.apiKey}` }) },
    });
  }

  // Project Management
  async createProject(project: WIACryoLegalProject): Promise<ProjectResponse> { return (await this.axios.post<ProjectResponse>('/projects', project)).data; }
  async getProject(id: string): Promise<WIACryoLegalProject> { return (await this.axios.get<WIACryoLegalProject>(`/projects/${id}`)).data; }
  async listProjects(params?: { status?: string; limit?: number; offset?: number }): Promise<PaginatedResponse<ProjectResponse>> { return (await this.axios.get<PaginatedResponse<ProjectResponse>>('/projects', { params })).data; }
  async updateProject(id: string, updates: Partial<WIACryoLegalProject>): Promise<ProjectResponse> { return (await this.axios.put<ProjectResponse>(`/projects/${id}`, updates)).data; }

  // Contract Management
  async createContract(projectId: string, contract: Partial<Contract>): Promise<Contract> { return (await this.axios.post<Contract>(`/projects/${projectId}/contracts`, contract)).data; }
  async getContract(projectId: string, contractId: string): Promise<Contract> { return (await this.axios.get<Contract>(`/projects/${projectId}/contracts/${contractId}`)).data; }
  async listContracts(projectId: string, params?: { type?: string; status?: string; party?: string; limit?: number }): Promise<PaginatedResponse<Contract>> { return (await this.axios.get<PaginatedResponse<Contract>>(`/projects/${projectId}/contracts`, { params })).data; }
  async updateContract(projectId: string, contractId: string, updates: Partial<Contract>): Promise<Contract> { return (await this.axios.put<Contract>(`/projects/${projectId}/contracts/${contractId}`, updates)).data; }
  async signContract(projectId: string, contractId: string, signature: SignatureRequest): Promise<Contract> { return (await this.axios.post<Contract>(`/projects/${projectId}/contracts/${contractId}/sign`, signature)).data; }
  async terminateContract(projectId: string, contractId: string, termination: TerminationRequest): Promise<Contract> { return (await this.axios.post<Contract>(`/projects/${projectId}/contracts/${contractId}/terminate`, termination)).data; }
  async getContractHistory(projectId: string, contractId: string): Promise<ContractEvent[]> { return (await this.axios.get<ContractEvent[]>(`/projects/${projectId}/contracts/${contractId}/history`)).data; }

  // Templates
  async createTemplate(projectId: string, template: Partial<ContractTemplate>): Promise<ContractTemplate> { return (await this.axios.post<ContractTemplate>(`/projects/${projectId}/templates`, template)).data; }
  async listTemplates(projectId: string, params?: { type?: string; status?: string }): Promise<ContractTemplate[]> { return (await this.axios.get<ContractTemplate[]>(`/projects/${projectId}/templates`, { params })).data; }
  async getTemplate(projectId: string, templateId: string): Promise<ContractTemplate> { return (await this.axios.get<ContractTemplate>(`/projects/${projectId}/templates/${templateId}`)).data; }
  async generateContract(projectId: string, templateId: string, data: ContractGenerationData): Promise<Contract> { return (await this.axios.post<Contract>(`/projects/${projectId}/templates/${templateId}/generate`, data)).data; }

  // Disputes
  async fileDispute(projectId: string, dispute: Partial<Dispute>): Promise<Dispute> { return (await this.axios.post<Dispute>(`/projects/${projectId}/disputes`, dispute)).data; }
  async getDispute(projectId: string, disputeId: string): Promise<Dispute> { return (await this.axios.get<Dispute>(`/projects/${projectId}/disputes/${disputeId}`)).data; }
  async listDisputes(projectId: string, params?: { type?: string; status?: string; limit?: number }): Promise<PaginatedResponse<Dispute>> { return (await this.axios.get<PaginatedResponse<Dispute>>(`/projects/${projectId}/disputes`, { params })).data; }
  async updateDispute(projectId: string, disputeId: string, update: DisputeUpdate): Promise<Dispute> { return (await this.axios.put<Dispute>(`/projects/${projectId}/disputes/${disputeId}`, update)).data; }
  async resolveDispute(projectId: string, disputeId: string, resolution: ResolutionData): Promise<Dispute> { return (await this.axios.post<Dispute>(`/projects/${projectId}/disputes/${disputeId}/resolve`, resolution)).data; }

  // Compliance
  async listRequirements(projectId: string): Promise<ComplianceRequirement[]> { return (await this.axios.get<ComplianceRequirement[]>(`/projects/${projectId}/compliance/requirements`)).data; }
  async checkCompliance(projectId: string, requirementId: string): Promise<ComplianceCheckResult> { return (await this.axios.post<ComplianceCheckResult>(`/projects/${projectId}/compliance/requirements/${requirementId}/check`)).data; }
  async recordViolation(projectId: string, violation: ViolationReport): Promise<ViolationResult> { return (await this.axios.post<ViolationResult>(`/projects/${projectId}/compliance/violations`, violation)).data; }
  async listViolations(projectId: string, params?: { status?: string }): Promise<Violation[]> { return (await this.axios.get<Violation[]>(`/projects/${projectId}/compliance/violations`, { params })).data; }

  // Regulations
  async listRegulations(projectId: string, params?: { jurisdiction?: string; category?: string }): Promise<Regulation[]> { return (await this.axios.get<Regulation[]>(`/projects/${projectId}/regulations`, { params })).data; }
  async getRegulation(projectId: string, regulationId: string): Promise<Regulation> { return (await this.axios.get<Regulation>(`/projects/${projectId}/regulations/${regulationId}`)).data; }

  // Reporting
  async generateReport(projectId: string, type: string, options?: ReportOptions): Promise<ReportResult> { return (await this.axios.post<ReportResult>(`/projects/${projectId}/reports`, { type, options })).data; }
  async getDashboard(projectId: string): Promise<DashboardData> { return (await this.axios.get<DashboardData>(`/projects/${projectId}/dashboard`)).data; }

  // Validation
  validateProject(project: WIACryoLegalProject): ValidationResult {
    const errors: any[] = [];
    if (!project.standard || project.standard !== 'WIA-CRYO-LEGAL') errors.push({ path: 'standard', message: 'Standard must be "WIA-CRYO-LEGAL"' });
    if (!project.metadata?.id) errors.push({ path: 'metadata.id', message: 'Project ID is required' });
    return { valid: errors.length === 0, errors: errors.length > 0 ? errors : undefined };
  }
}

// ============================================================================
// Supporting Types
// ============================================================================

export interface SignatureRequest { party: string; signatory: string; method: 'wet' | 'electronic' | 'digital'; witness?: string; notarized?: boolean; }
export interface TerminationRequest { reason: string; effectiveDate: string; notice: boolean; }
export interface ContractGenerationData { parties: { id: string; name: string; role: string; contact: { name: string; email: string } }[]; terms?: Record<string, unknown>; customClauses?: { id: string; text: string }[]; }
export interface DisputeUpdate { status?: string; mechanism?: string; notes?: string; }
export interface ResolutionData { mechanism: string; outcome: string; terms: string[]; binding: boolean; }
export interface ComplianceCheckResult { requirementId: string; status: 'compliant' | 'non-compliant' | 'partial'; checkedAt: string; findings?: string[]; }
export interface ViolationReport { requirement: string; description: string; severity: 'minor' | 'major' | 'critical'; discoveredDate: string; }
export interface ViolationResult { id: string; status: string; createdAt: string; }
export interface Violation extends ViolationReport { id: string; status: string; createdAt: string; resolvedAt?: string; remediation?: string; }
export interface ReportOptions { format?: 'pdf' | 'docx' | 'json'; period?: { start: string; end: string }; }
export interface ReportResult { id: string; status: 'queued' | 'processing' | 'completed' | 'failed'; url?: string; createdAt: string; }
export interface DashboardData { contracts: { total: number; active: number; expiring: number; disputed: number }; compliance: { score: number; requirements: number; violations: number }; disputes: { active: number; pending: number }; }

// ============================================================================
// Utility Functions
// ============================================================================

export function generateUUID(): string { return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => { const r = (Math.random() * 16) | 0; return (c === 'x' ? r : (r & 0x3) | 0x8).toString(16); }); }

export function createMinimalProject(name: string, organization: string): WIACryoLegalProject {
  return {
    standard: 'WIA-CRYO-LEGAL', version: '1.0.0',
    metadata: { id: generateUUID(), name, organization: { name: organization, type: 'facility', country: 'US', contact: { name: '', email: '' } }, jurisdictions: [{ country: 'US', type: 'primary', applicableLaws: [] }], createdAt: new Date().toISOString(), status: 'active' },
    legalFramework: { regulations: [], rights: [], obligations: [], liabilities: [], precedents: [] },
    contracts: { templates: [], contracts: [], terms: { storage: [], liability: [], termination: [], dispute: [], privacy: [] }, amendments: { allowed: true, procedure: '', approval: [], documentation: [] }, termination: { notice: '30 days', grounds: [], procedure: [], consequences: [] } },
    disputes: { mechanisms: [], disputes: [], escalation: { levels: [], finalResort: 'litigation' }, documentation: { required: [], retention: '10 years', confidentiality: 'strict' } },
    compliance: { requirements: [], audits: { internal: { frequency: 'annual', scope: [], team: '' }, external: { frequency: 'annual', auditor: '', scope: [] }, regulatory: { agencies: [], schedule: '' } }, violations: { reporting: '', investigation: '', remediation: '', disclosure: '', tracking: true }, training: { topics: [], frequency: 'annual', mandatory: true, records: true } },
    reporting: { internal: { schedule: 'quarterly', recipients: [], content: [] }, regulatory: { agencies: [], format: '', retention: '10 years' }, incidents: { types: [], timeframe: '', authorities: [], procedure: '' } },
  };
}

export default { WIACryoLegalClient, generateUUID, createMinimalProject };
