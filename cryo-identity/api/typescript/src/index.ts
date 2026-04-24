/**
 * WIA Cryo Identity Standard - TypeScript SDK
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';

export * from './types';
import type { APIConfig, WIACryoIdentityProject, ProjectResponse, ValidationResult, PaginatedResponse, Subject, SubjectIdentifier, BiometricData, SpecimenLink, SubjectEvent } from './types';

// ============================================================================
// WIA Cryo Identity Client
// ============================================================================

export class WIACryoIdentityClient {
  private axios: AxiosInstance;

  constructor(config: APIConfig) {
    this.axios = axios.create({
      baseURL: config.baseURL,
      timeout: config.timeout || 30000,
      headers: { 'Content-Type': 'application/json', ...(config.apiKey && { Authorization: `Bearer ${config.apiKey}` }) },
    });
  }

  // Project Management
  async createProject(project: WIACryoIdentityProject): Promise<ProjectResponse> { return (await this.axios.post<ProjectResponse>('/projects', project)).data; }
  async getProject(id: string): Promise<WIACryoIdentityProject> { return (await this.axios.get<WIACryoIdentityProject>(`/projects/${id}`)).data; }
  async listProjects(params?: { status?: string; limit?: number; offset?: number }): Promise<PaginatedResponse<ProjectResponse>> { return (await this.axios.get<PaginatedResponse<ProjectResponse>>('/projects', { params })).data; }
  async updateProject(id: string, updates: Partial<WIACryoIdentityProject>): Promise<ProjectResponse> { return (await this.axios.put<ProjectResponse>(`/projects/${id}`, updates)).data; }

  // Subject Management
  async registerSubject(projectId: string, subject: Partial<Subject>): Promise<Subject> { return (await this.axios.post<Subject>(`/projects/${projectId}/subjects`, subject)).data; }
  async getSubject(projectId: string, subjectId: string): Promise<Subject> { return (await this.axios.get<Subject>(`/projects/${projectId}/subjects/${subjectId}`)).data; }
  async listSubjects(projectId: string, params?: { status?: string; type?: string; limit?: number }): Promise<PaginatedResponse<Subject>> { return (await this.axios.get<PaginatedResponse<Subject>>(`/projects/${projectId}/subjects`, { params })).data; }
  async updateSubject(projectId: string, subjectId: string, updates: Partial<Subject>): Promise<Subject> { return (await this.axios.put<Subject>(`/projects/${projectId}/subjects/${subjectId}`, updates)).data; }
  async searchSubjects(projectId: string, query: SearchQuery): Promise<SearchResult[]> { return (await this.axios.post<SearchResult[]>(`/projects/${projectId}/subjects/search`, query)).data; }

  // Identity Operations
  async addIdentifier(projectId: string, subjectId: string, identifier: SubjectIdentifier): Promise<SubjectIdentifier> { return (await this.axios.post<SubjectIdentifier>(`/projects/${projectId}/subjects/${subjectId}/identifiers`, identifier)).data; }
  async verifyIdentifier(projectId: string, subjectId: string, identifierId: string, verification: VerificationData): Promise<VerificationResult> { return (await this.axios.post<VerificationResult>(`/projects/${projectId}/subjects/${subjectId}/identifiers/${identifierId}/verify`, verification)).data; }
  async addBiometric(projectId: string, subjectId: string, biometric: BiometricData): Promise<BiometricData> { return (await this.axios.post<BiometricData>(`/projects/${projectId}/subjects/${subjectId}/biometrics`, biometric)).data; }
  async verifyBiometric(projectId: string, subjectId: string, biometric: BiometricVerification): Promise<BiometricMatchResult> { return (await this.axios.post<BiometricMatchResult>(`/projects/${projectId}/subjects/${subjectId}/biometrics/verify`, biometric)).data; }

  // Specimen Linking
  async linkSpecimen(projectId: string, subjectId: string, link: SpecimenLink): Promise<SpecimenLink> { return (await this.axios.post<SpecimenLink>(`/projects/${projectId}/subjects/${subjectId}/specimens`, link)).data; }
  async listSpecimens(projectId: string, subjectId: string): Promise<SpecimenLink[]> { return (await this.axios.get<SpecimenLink[]>(`/projects/${projectId}/subjects/${subjectId}/specimens`)).data; }
  async unlinkSpecimen(projectId: string, subjectId: string, specimenId: string): Promise<void> { await this.axios.delete(`/projects/${projectId}/subjects/${subjectId}/specimens/${specimenId}`); }

  // Identity Resolution
  async resolveIdentity(projectId: string, identifiers: Partial<SubjectIdentifier>[]): Promise<ResolutionResult> { return (await this.axios.post<ResolutionResult>(`/projects/${projectId}/resolve`, { identifiers })).data; }
  async mergeSubjects(projectId: string, primaryId: string, secondaryIds: string[]): Promise<MergeResult> { return (await this.axios.post<MergeResult>(`/projects/${projectId}/merge`, { primaryId, secondaryIds })).data; }

  // Status Management
  async updateStatus(projectId: string, subjectId: string, status: string, reason: string): Promise<Subject> { return (await this.axios.put<Subject>(`/projects/${projectId}/subjects/${subjectId}/status`, { status, reason })).data; }
  async recordDeceased(projectId: string, subjectId: string, data: DeceasedRecord): Promise<Subject> { return (await this.axios.post<Subject>(`/projects/${projectId}/subjects/${subjectId}/deceased`, data)).data; }
  async anonymize(projectId: string, subjectId: string, options: AnonymizationOptions): Promise<AnonymizationResult> { return (await this.axios.post<AnonymizationResult>(`/projects/${projectId}/subjects/${subjectId}/anonymize`, options)).data; }

  // Audit & History
  async getHistory(projectId: string, subjectId: string, params?: { start?: string; end?: string; type?: string }): Promise<SubjectEvent[]> { return (await this.axios.get<SubjectEvent[]>(`/projects/${projectId}/subjects/${subjectId}/history`, { params })).data; }
  async getAuditLog(projectId: string, params?: { start?: string; end?: string }): Promise<AuditEntry[]> { return (await this.axios.get<AuditEntry[]>(`/projects/${projectId}/audit`, { params })).data; }

  // Dashboard
  async getDashboard(projectId: string): Promise<DashboardData> { return (await this.axios.get<DashboardData>(`/projects/${projectId}/dashboard`)).data; }

  // Validation
  validateProject(project: WIACryoIdentityProject): ValidationResult {
    const errors: any[] = [];
    if (!project.standard || project.standard !== 'WIA-CRYO-IDENTITY') errors.push({ path: 'standard', message: 'Standard must be "WIA-CRYO-IDENTITY"' });
    if (!project.metadata?.id) errors.push({ path: 'metadata.id', message: 'Project ID is required' });
    return { valid: errors.length === 0, errors: errors.length > 0 ? errors : undefined };
  }
}

// ============================================================================
// Supporting Types
// ============================================================================

export interface SearchQuery { identifiers?: Partial<SubjectIdentifier>[]; name?: string; dateOfBirth?: string; biometric?: { type: string; template: string }; limit?: number; }
export interface SearchResult { subjectId: string; confidence: number; matchedFields: string[]; }
export interface VerificationData { method: string; evidence: string; verifier: string; }
export interface VerificationResult { verified: boolean; confidence: number; verifiedAt: string; verifier: string; }
export interface BiometricVerification { type: string; template: string; threshold?: number; }
export interface BiometricMatchResult { matched: boolean; confidence: number; matchedAgainst?: string; }
export interface ResolutionResult { found: boolean; subjectId?: string; confidence: number; alternatives?: { subjectId: string; confidence: number }[]; }
export interface MergeResult { mergedSubjectId: string; mergedFrom: string[]; timestamp: string; }
export interface DeceasedRecord { dateOfDeath: string; certificate?: string; notifier: string; specimenDirective?: string; }
export interface AnonymizationOptions { method: 'pseudonymization' | 'full'; retainLinks: boolean; documentation: string; }
export interface AnonymizationResult { success: boolean; anonymizedId?: string; timestamp: string; }
export interface AuditEntry { id: string; timestamp: string; actor: string; action: string; resource: string; details: Record<string, unknown>; }
export interface DashboardData { summary: { total: number; active: number; verified: number; linked: number }; recent: { type: string; description: string; timestamp: string }[]; }

// ============================================================================
// Utility Functions
// ============================================================================

export function generateUUID(): string { return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => { const r = (Math.random() * 16) | 0; return (c === 'x' ? r : (r & 0x3) | 0x8).toString(16); }); }

export function createMinimalProject(name: string, organization: string): WIACryoIdentityProject {
  return {
    standard: 'WIA-CRYO-IDENTITY', version: '1.0.0',
    metadata: { id: generateUUID(), name, organization: { name: organization, type: 'facility', country: 'US', contact: { name: '', email: '' } }, jurisdiction: ['US'], createdAt: new Date().toISOString(), status: 'active' },
    identityManagement: { schema: { version: '1.0', attributes: [], required: [], unique: [], immutable: [] }, lifecycle: { states: [], transitions: [], retention: { active: '50 years', suspended: '50 years', deceased: '50 years', archival: '100 years' } }, linking: { specimenToIdentity: { enabled: true, method: 'direct', verification: 'biometric', reversible: true }, crossFacility: { enabled: true, method: 'federation', verification: 'multi-factor', reversible: true }, familial: { enabled: true, method: 'consent', verification: 'document', reversible: true } }, resolution: { priority: ['internal', 'national-id'], conflicts: 'manual', fallback: 'create-new' } },
    subjects: [],
    verification: { methods: [], workflows: [], thresholds: [], reVerification: { triggers: [], frequency: 'annual', gracePeriod: '30 days' } },
    privacy: { principles: [], dataMinimization: { collection: 'necessary', retention: 'purpose-limited', disclosure: 'consent-based' }, anonymization: { methods: [], triggers: [], verification: '', reversibility: false }, accessControl: { model: 'rbac', roles: [], policies: [], audit: true }, breachProtocol: { detection: [], notification: { authorities: [], subjects: { method: '', timeframe: '' }, internal: { recipients: [], timeframe: '' } }, remediation: [], reporting: '' } },
    continuity: { succession: { transferProtocol: '', dataHandling: '', notificationRequired: true }, disasterRecovery: { rto: 4, rpo: 1, testingFrequency: 'annual' }, portability: { formats: ['json', 'xml'], standards: [], exportProcess: '', timeline: '30 days' }, termination: { notice: '90 days', dataTransfer: '', destruction: '', certification: true } },
    audit: { logging: { events: [], details: 'detailed', realtime: true }, reporting: { automated: [], onDemand: [], compliance: [] }, retention: '10 years', integrity: { method: 'hash', verification: 'automatic', alerting: true } },
  };
}

export default { WIACryoIdentityClient, generateUUID, createMinimalProject };
