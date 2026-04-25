/**
 * WIA Digital Erasure Standard - TypeScript SDK
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';

export * from './types';
import type {
  APIConfig, WIADigitalErasure, ErasureResponse, ExecutionPhase, AuditEvent,
  ErasureCertificate, ValidationResult, PaginatedResponse,
} from './types';

export class WIADigitalErasureClient {
  private axios: AxiosInstance;

  constructor(config: APIConfig) {
    this.axios = axios.create({
      baseURL: config.baseURL,
      timeout: config.timeout || 30000,
      headers: { 'Content-Type': 'application/json', ...(config.apiKey && { Authorization: `Bearer ${config.apiKey}` }) },
    });
  }

  async createRequest(request: WIADigitalErasure): Promise<ErasureResponse> {
    const response = await this.axios.post<ErasureResponse>('/erasure-requests', request);
    return response.data;
  }

  async getRequest(id: string): Promise<WIADigitalErasure> {
    const response = await this.axios.get<WIADigitalErasure>(`/erasure-requests/${id}`);
    return response.data;
  }

  async listRequests(params?: { status?: string; limit?: number; offset?: number }): Promise<PaginatedResponse<ErasureResponse>> {
    const response = await this.axios.get<PaginatedResponse<ErasureResponse>>('/erasure-requests', { params });
    return response.data;
  }

  async updateRequest(id: string, updates: Partial<WIADigitalErasure>): Promise<ErasureResponse> {
    const response = await this.axios.put<ErasureResponse>(`/erasure-requests/${id}`, updates);
    return response.data;
  }

  async cancelRequest(id: string, reason: string): Promise<void> {
    await this.axios.post(`/erasure-requests/${id}/cancel`, { reason });
  }

  async verifyIdentity(requestId: string, method: string, proof: Record<string, unknown>): Promise<{ verified: boolean; message?: string }> {
    const response = await this.axios.post(`/erasure-requests/${requestId}/verify`, { method, proof });
    return response.data;
  }

  async executeRequest(id: string, dryRun?: boolean): Promise<{ executionId: string; status: string }> {
    const response = await this.axios.post(`/erasure-requests/${id}/execute`, { dryRun });
    return response.data;
  }

  async getExecutionStatus(id: string): Promise<{ phases: ExecutionPhase[]; overallStatus: string; progress: number }> {
    const response = await this.axios.get(`/erasure-requests/${id}/execution`);
    return response.data;
  }

  async rollbackExecution(id: string): Promise<void> {
    await this.axios.post(`/erasure-requests/${id}/rollback`);
  }

  async getAuditLog(requestId: string): Promise<AuditEvent[]> {
    const response = await this.axios.get<AuditEvent[]>(`/erasure-requests/${requestId}/audit`);
    return response.data;
  }

  async getCertificate(requestId: string): Promise<ErasureCertificate> {
    const response = await this.axios.get<ErasureCertificate>(`/erasure-requests/${requestId}/certificate`);
    return response.data;
  }

  async notifyThirdParty(requestId: string, vendorId: string): Promise<{ notified: boolean; confirmationId?: string }> {
    const response = await this.axios.post(`/erasure-requests/${requestId}/notify-third-party`, { vendorId });
    return response.data;
  }

  validateRequest(request: WIADigitalErasure): ValidationResult {
    const errors: any[] = [];
    if (request.standard !== 'WIA-DIGITAL-ERASURE') errors.push({ path: 'standard', message: 'Invalid standard' });
    if (!request.request?.id) errors.push({ path: 'request.id', message: 'Request ID required' });
    if (!request.subject?.identifiers?.length) errors.push({ path: 'subject.identifiers', message: 'At least one identifier required' });
    return { valid: errors.length === 0, errors: errors.length > 0 ? errors : undefined };
  }
}

export function generateUUID(): string {
  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => {
    const r = (Math.random() * 16) | 0;
    return (c === 'x' ? r : (r & 0x3) | 0x8).toString(16);
  });
}

export function createMinimalRequest(email: string, regulation: 'gdpr' | 'ccpa' = 'gdpr'): WIADigitalErasure {
  return {
    standard: 'WIA-DIGITAL-ERASURE',
    version: '1.0.0',
    request: {
      id: generateUUID(),
      type: 'right-to-be-forgotten',
      status: 'pending',
      priority: 'normal',
      createdAt: new Date().toISOString(),
      legalBasis: { regulation, justification: 'Data subject request' },
      requestor: { type: 'data-subject', email, verificationMethod: 'email' },
    },
    subject: {
      id: generateUUID(),
      identifiers: [{ type: 'email', value: email, verified: false }],
      accounts: [],
      dataCategories: ['personal'],
    },
    scope: { systems: [], dataTypes: [{ category: 'personal', included: true }], backups: { includeBackups: true } },
    verification: { steps: [], requiredLevel: 'standard', timeoutHours: 72, maxAttempts: 3 },
    execution: { phases: [], rollbackEnabled: true, estimatedDuration: 24, parallelization: false },
    audit: { enabled: true, retention: 365, immutable: true, events: [] },
  };
}

export default { WIADigitalErasureClient, generateUUID, createMinimalRequest };
