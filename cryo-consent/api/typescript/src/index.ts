/**
 * WIA Cryo Consent Standard - TypeScript SDK
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';

export * from './types';
import type {
  APIConfig,
  WIACryoConsentProject,
  ProjectResponse,
  ValidationResult,
  PaginatedResponse,
  ConsentSubject,
  ConsentRecord,
  ConsentDocument,
  ConsentStatus,
  ConsentEvent,
  LegalRepresentative,
} from './types';

// ============================================================================
// WIA Cryo Consent Client
// ============================================================================

export class WIACryoConsentClient {
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

  // ========================================================================
  // Project Management
  // ========================================================================

  async createProject(project: WIACryoConsentProject): Promise<ProjectResponse> {
    const response = await this.axios.post<ProjectResponse>('/projects', project);
    return response.data;
  }

  async getProject(id: string): Promise<WIACryoConsentProject> {
    const response = await this.axios.get<WIACryoConsentProject>(`/projects/${id}`);
    return response.data;
  }

  async listProjects(params?: {
    status?: string;
    organization?: string;
    limit?: number;
    offset?: number;
  }): Promise<PaginatedResponse<ProjectResponse>> {
    const response = await this.axios.get<PaginatedResponse<ProjectResponse>>('/projects', {
      params,
    });
    return response.data;
  }

  async updateProject(id: string, updates: Partial<WIACryoConsentProject>): Promise<ProjectResponse> {
    const response = await this.axios.put<ProjectResponse>(`/projects/${id}`, updates);
    return response.data;
  }

  // ========================================================================
  // Subject Management
  // ========================================================================

  async registerSubject(projectId: string, subject: Partial<ConsentSubject>): Promise<ConsentSubject> {
    const response = await this.axios.post<ConsentSubject>(
      `/projects/${projectId}/subjects`,
      subject
    );
    return response.data;
  }

  async getSubject(projectId: string, subjectId: string): Promise<ConsentSubject> {
    const response = await this.axios.get<ConsentSubject>(
      `/projects/${projectId}/subjects/${subjectId}`
    );
    return response.data;
  }

  async listSubjects(projectId: string, params?: {
    status?: string;
    type?: string;
    limit?: number;
    offset?: number;
  }): Promise<PaginatedResponse<ConsentSubject>> {
    const response = await this.axios.get<PaginatedResponse<ConsentSubject>>(
      `/projects/${projectId}/subjects`,
      { params }
    );
    return response.data;
  }

  async updateSubject(
    projectId: string,
    subjectId: string,
    updates: Partial<ConsentSubject>
  ): Promise<ConsentSubject> {
    const response = await this.axios.put<ConsentSubject>(
      `/projects/${projectId}/subjects/${subjectId}`,
      updates
    );
    return response.data;
  }

  async addRepresentative(
    projectId: string,
    subjectId: string,
    representative: LegalRepresentative
  ): Promise<LegalRepresentative> {
    const response = await this.axios.post<LegalRepresentative>(
      `/projects/${projectId}/subjects/${subjectId}/representatives`,
      representative
    );
    return response.data;
  }

  // ========================================================================
  // Consent Management
  // ========================================================================

  async initiateConsent(
    projectId: string,
    subjectId: string,
    consentTypeId: string,
    data: ConsentInitiation
  ): Promise<ConsentRecord> {
    const response = await this.axios.post<ConsentRecord>(
      `/projects/${projectId}/subjects/${subjectId}/consents`,
      { consentTypeId, ...data }
    );
    return response.data;
  }

  async getConsent(
    projectId: string,
    subjectId: string,
    consentId: string
  ): Promise<ConsentRecord> {
    const response = await this.axios.get<ConsentRecord>(
      `/projects/${projectId}/subjects/${subjectId}/consents/${consentId}`
    );
    return response.data;
  }

  async listConsents(
    projectId: string,
    subjectId: string,
    params?: { status?: string; type?: string }
  ): Promise<ConsentRecord[]> {
    const response = await this.axios.get<ConsentRecord[]>(
      `/projects/${projectId}/subjects/${subjectId}/consents`,
      { params }
    );
    return response.data;
  }

  async signConsent(
    projectId: string,
    subjectId: string,
    consentId: string,
    signature: SignatureData
  ): Promise<ConsentRecord> {
    const response = await this.axios.post<ConsentRecord>(
      `/projects/${projectId}/subjects/${subjectId}/consents/${consentId}/sign`,
      signature
    );
    return response.data;
  }

  async verifyConsent(
    projectId: string,
    subjectId: string,
    consentId: string,
    verification: VerificationData
  ): Promise<VerificationResult> {
    const response = await this.axios.post<VerificationResult>(
      `/projects/${projectId}/subjects/${subjectId}/consents/${consentId}/verify`,
      verification
    );
    return response.data;
  }

  async withdrawConsent(
    projectId: string,
    subjectId: string,
    consentId: string,
    withdrawal: WithdrawalRequest
  ): Promise<WithdrawalResult> {
    const response = await this.axios.post<WithdrawalResult>(
      `/projects/${projectId}/subjects/${subjectId}/consents/${consentId}/withdraw`,
      withdrawal
    );
    return response.data;
  }

  async renewConsent(
    projectId: string,
    subjectId: string,
    consentId: string,
    renewal: RenewalRequest
  ): Promise<ConsentRecord> {
    const response = await this.axios.post<ConsentRecord>(
      `/projects/${projectId}/subjects/${subjectId}/consents/${consentId}/renew`,
      renewal
    );
    return response.data;
  }

  async getConsentHistory(
    projectId: string,
    subjectId: string,
    consentId: string
  ): Promise<ConsentEvent[]> {
    const response = await this.axios.get<ConsentEvent[]>(
      `/projects/${projectId}/subjects/${subjectId}/consents/${consentId}/history`
    );
    return response.data;
  }

  // ========================================================================
  // Document Management
  // ========================================================================

  async createDocument(
    projectId: string,
    document: Partial<ConsentDocument>
  ): Promise<ConsentDocument> {
    const response = await this.axios.post<ConsentDocument>(
      `/projects/${projectId}/documents`,
      document
    );
    return response.data;
  }

  async getDocument(projectId: string, documentId: string): Promise<ConsentDocument> {
    const response = await this.axios.get<ConsentDocument>(
      `/projects/${projectId}/documents/${documentId}`
    );
    return response.data;
  }

  async listDocuments(projectId: string, params?: {
    type?: string;
    status?: string;
    language?: string;
  }): Promise<ConsentDocument[]> {
    const response = await this.axios.get<ConsentDocument[]>(
      `/projects/${projectId}/documents`,
      { params }
    );
    return response.data;
  }

  async approveDocument(
    projectId: string,
    documentId: string,
    approval: DocumentApprovalRequest
  ): Promise<ConsentDocument> {
    const response = await this.axios.post<ConsentDocument>(
      `/projects/${projectId}/documents/${documentId}/approve`,
      approval
    );
    return response.data;
  }

  async generateDocument(
    projectId: string,
    subjectId: string,
    templateId: string,
    options?: DocumentGenerationOptions
  ): Promise<GeneratedDocument> {
    const response = await this.axios.post<GeneratedDocument>(
      `/projects/${projectId}/subjects/${subjectId}/generate-document`,
      { templateId, options }
    );
    return response.data;
  }

  // ========================================================================
  // Verification & Compliance
  // ========================================================================

  async verifyIdentity(
    projectId: string,
    subjectId: string,
    verification: IdentityVerificationRequest
  ): Promise<IdentityVerificationResult> {
    const response = await this.axios.post<IdentityVerificationResult>(
      `/projects/${projectId}/subjects/${subjectId}/verify-identity`,
      verification
    );
    return response.data;
  }

  async checkCompliance(projectId: string, subjectId?: string): Promise<ComplianceCheckResult> {
    const response = await this.axios.get<ComplianceCheckResult>(
      `/projects/${projectId}/compliance/check`,
      { params: { subjectId } }
    );
    return response.data;
  }

  async getAuditLog(
    projectId: string,
    params?: { start?: string; end?: string; type?: string; subjectId?: string }
  ): Promise<AuditLogEntry[]> {
    const response = await this.axios.get<AuditLogEntry[]>(
      `/projects/${projectId}/audit`,
      { params }
    );
    return response.data;
  }

  // ========================================================================
  // Reporting
  // ========================================================================

  async generateReport(
    projectId: string,
    type: string,
    options?: ReportOptions
  ): Promise<ReportResult> {
    const response = await this.axios.post<ReportResult>(
      `/projects/${projectId}/reports`,
      { type, options }
    );
    return response.data;
  }

  async getDashboard(projectId: string): Promise<DashboardData> {
    const response = await this.axios.get<DashboardData>(
      `/projects/${projectId}/dashboard`
    );
    return response.data;
  }

  // ========================================================================
  // Validation
  // ========================================================================

  validateProject(project: WIACryoConsentProject): ValidationResult {
    const errors: any[] = [];

    if (!project.standard || project.standard !== 'WIA-CRYO-CONSENT') {
      errors.push({
        path: 'standard',
        message: 'Standard must be "WIA-CRYO-CONSENT"',
      });
    }

    if (!project.metadata?.id) {
      errors.push({ path: 'metadata.id', message: 'Project ID is required' });
    }

    if (!project.consentFramework?.types?.length) {
      errors.push({
        path: 'consentFramework.types',
        message: 'At least one consent type is required',
      });
    }

    return {
      valid: errors.length === 0,
      errors: errors.length > 0 ? errors : undefined,
    };
  }
}

// ============================================================================
// Supporting Types
// ============================================================================

export interface ConsentInitiation {
  scope: string[];
  restrictions?: string[];
  validFrom?: string;
  validTo?: string;
  notes?: string;
}

export interface SignatureData {
  method: 'written' | 'electronic' | 'biometric';
  signature: string;
  timestamp: string;
  ipAddress?: string;
  deviceInfo?: string;
  witnesses?: { name: string; signature: string }[];
}

export interface VerificationData {
  method: string;
  verifier: string;
  documentation?: string[];
  notes?: string;
}

export interface VerificationResult {
  verified: boolean;
  verifiedAt: string;
  verifier: string;
  method: string;
  issues?: string[];
}

export interface WithdrawalRequest {
  reason: string;
  scope: 'full' | 'partial';
  specificConsents?: string[];
  effectiveDate: string;
  confirmation: boolean;
}

export interface WithdrawalResult {
  id: string;
  status: 'processed' | 'pending';
  processedAt?: string;
  affectedConsents: string[];
  consequences: string[];
}

export interface RenewalRequest {
  validTo: string;
  modifications?: { scope?: string[]; restrictions?: string[] };
  reconfirmation: boolean;
}

export interface DocumentApprovalRequest {
  approver: string;
  ethicsApproval?: { committee: string; reference: string };
  legalReview?: { reviewer: string; notes?: string };
}

export interface DocumentGenerationOptions {
  language?: string;
  format?: 'pdf' | 'html';
  includeSignatureBlocks?: boolean;
}

export interface GeneratedDocument {
  id: string;
  url: string;
  format: string;
  generatedAt: string;
  expiresAt?: string;
}

export interface IdentityVerificationRequest {
  method: string;
  documents: { type: string; number: string; expiry?: string }[];
  biometric?: { type: string; data: string };
}

export interface IdentityVerificationResult {
  verified: boolean;
  confidence: number;
  method: string;
  verifiedAt: string;
  issues?: string[];
}

export interface ComplianceCheckResult {
  compliant: boolean;
  score: number;
  issues: { severity: string; description: string; recommendation: string }[];
  lastCheck: string;
}

export interface AuditLogEntry {
  id: string;
  timestamp: string;
  actor: string;
  action: string;
  resource: string;
  details: Record<string, unknown>;
  ipAddress?: string;
}

export interface ReportOptions {
  format?: 'pdf' | 'excel' | 'json';
  period?: { start: string; end: string };
  subjects?: string[];
}

export interface ReportResult {
  id: string;
  status: 'queued' | 'processing' | 'completed' | 'failed';
  url?: string;
  createdAt: string;
}

export interface DashboardData {
  summary: { totalSubjects: number; activeConsents: number; pendingRenewals: number };
  compliance: { score: number; issues: number };
  activity: { recent: { type: string; description: string; timestamp: string }[] };
  expirations: { upcoming: { subjectId: string; consentId: string; expiresAt: string }[] };
}

// ============================================================================
// Utility Functions
// ============================================================================

export function generateUUID(): string {
  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => {
    const r = (Math.random() * 16) | 0;
    const v = c === 'x' ? r : (r & 0x3) | 0x8;
    return v.toString(16);
  });
}

export function createMinimalProject(name: string, organization: string): WIACryoConsentProject {
  return {
    standard: 'WIA-CRYO-CONSENT',
    version: '1.0.0',
    metadata: {
      id: generateUUID(),
      name,
      organization: { name: organization, type: 'facility', country: 'US' },
      jurisdiction: { country: 'US', applicableLaws: [] },
      createdAt: new Date().toISOString(),
      status: 'active',
    },
    consentFramework: { types: [], requirements: [], process: { steps: [], timeline: '', languages: ['en'], accessibility: { formats: ['print', 'digital'], languages: ['en'], interpreter: true, cognitiveSupport: true } }, retention: { consentForms: '10 years', supportingDocuments: '10 years', auditTrails: '10 years', format: 'both', storage: 'secure' }, withdrawal: { allowed: true, process: { steps: [], timeframe: '30 days', confirmation: 'written', documentation: [] }, consequences: [], partialWithdrawal: true } },
    subjects: [],
    documents: [],
    verification: { methods: [], identityVerification: { required: true, methods: ['government-id'], documentation: [] }, signatureVerification: { electronic: { allowed: true, standards: [], certification: '' }, physical: { required: false, retention: '', scanning: true }, witnessing: { required: false, count: 0, qualifications: [], independentRequired: false } }, auditTrail: { enabled: true, events: [], retention: '10 years', immutable: true } },
    compliance: { regulations: [], policies: [], audits: { internal: { frequency: 'annual', scope: [] }, external: { frequency: 'biennial', auditor: '' }, random: { enabled: true, frequency: 'quarterly' } }, breachProtocol: { definition: [], reportingTimeframe: '72 hours', notificationRequired: true, remediation: [] } },
    audit: { enabled: true, scope: [], retention: '10 years', reporting: { automated: true, frequency: 'monthly', recipients: [], format: 'pdf' } },
  };
}

// ============================================================================
// Exports
// ============================================================================

export default {
  WIACryoConsentClient,
  generateUUID,
  createMinimalProject,
};
