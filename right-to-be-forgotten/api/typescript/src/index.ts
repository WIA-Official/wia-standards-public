/**
 * WIA-LEG-009: Right to be Forgotten Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

import EventEmitter from 'eventemitter3';
import {
  WIAConfig,
  APIResponse,
  PaginatedResponse,
  DeletionRequest,
  DeletionStatus,
  DataSubject,
  DataController,
  DataCategory,
  DeletionCertificate,
  ComplianceReport,
  RetentionPolicy,
  RequestStatus,
  DeletionScope,
} from './types';

export * from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface WIARTBFConfig extends WIAConfig {
  controllerId?: string;
}

// ============================================================================
// Main SDK Client
// ============================================================================

export class WIARTBFClient {
  private config: Required<WIARTBFConfig>;
  private eventEmitter = new EventEmitter();

  constructor(config: WIARTBFConfig) {
    this.config = {
      endpoint: 'https://api.wia-standards.org/v1/rtbf',
      timeout: 30000,
      debug: false,
      controllerId: '',
      ...config,
    };

    if (!this.config.apiKey) {
      throw new Error('API key is required');
    }
  }

  // ==========================================================================
  // Deletion Request Operations
  // ==========================================================================

  async submitRequest(request: Omit<DeletionRequest, 'requestId' | 'submittedAt' | 'status'>): Promise<APIResponse<DeletionRequest>> {
    return this.makeRequest('POST', '/requests', request);
  }

  async getRequest(requestId: string): Promise<APIResponse<DeletionRequest>> {
    return this.makeRequest('GET', `/requests/${requestId}`);
  }

  async listRequests(filters?: {
    status?: RequestStatus;
    subjectId?: string;
    startDate?: string;
    endDate?: string;
  }): Promise<PaginatedResponse<DeletionRequest>> {
    const params = new URLSearchParams(filters as Record<string, string>);
    return this.makeRequest('GET', `/requests?${params}`);
  }

  async updateRequestStatus(requestId: string, status: RequestStatus, notes?: string): Promise<APIResponse<DeletionRequest>> {
    return this.makeRequest('PATCH', `/requests/${requestId}/status`, { status, notes });
  }

  async cancelRequest(requestId: string, reason: string): Promise<APIResponse<DeletionRequest>> {
    return this.makeRequest('POST', `/requests/${requestId}/cancel`, { reason });
  }

  async getRequestTimeline(requestId: string): Promise<APIResponse<Array<{
    timestamp: string;
    action: string;
    actor: string;
    details?: string;
  }>>> {
    return this.makeRequest('GET', `/requests/${requestId}/timeline`);
  }

  // ==========================================================================
  // Data Subject Operations
  // ==========================================================================

  async getSubject(subjectId: string): Promise<APIResponse<DataSubject>> {
    return this.makeRequest('GET', `/subjects/${subjectId}`);
  }

  async verifySubjectIdentity(subjectId: string, verificationData: {
    method: 'email' | 'sms' | 'document';
    token?: string;
  }): Promise<APIResponse<{ verified: boolean; expiresAt: string }>> {
    return this.makeRequest('POST', `/subjects/${subjectId}/verify`, verificationData);
  }

  async getSubjectData(subjectId: string): Promise<APIResponse<{
    categories: DataCategory[];
    controllers: DataController[];
    totalRecords: number;
  }>> {
    return this.makeRequest('GET', `/subjects/${subjectId}/data`);
  }

  async getSubjectRequests(subjectId: string): Promise<PaginatedResponse<DeletionRequest>> {
    return this.makeRequest('GET', `/subjects/${subjectId}/requests`);
  }

  // ==========================================================================
  // Data Controller Operations
  // ==========================================================================

  async getController(controllerId: string): Promise<APIResponse<DataController>> {
    return this.makeRequest('GET', `/controllers/${controllerId}`);
  }

  async listControllers(): Promise<PaginatedResponse<DataController>> {
    return this.makeRequest('GET', '/controllers');
  }

  async registerController(controller: Omit<DataController, 'controllerId' | 'registeredAt'>): Promise<APIResponse<DataController>> {
    return this.makeRequest('POST', '/controllers', controller);
  }

  async updateController(controllerId: string, updates: Partial<DataController>): Promise<APIResponse<DataController>> {
    return this.makeRequest('PATCH', `/controllers/${controllerId}`, updates);
  }

  async getControllerRequests(controllerId: string, status?: RequestStatus): Promise<PaginatedResponse<DeletionRequest>> {
    const params = status ? `?status=${status}` : '';
    return this.makeRequest('GET', `/controllers/${controllerId}/requests${params}`);
  }

  // ==========================================================================
  // Certificate Operations
  // ==========================================================================

  async getCertificate(requestId: string): Promise<APIResponse<DeletionCertificate>> {
    return this.makeRequest('GET', `/requests/${requestId}/certificate`);
  }

  async verifyCertificate(certificateId: string): Promise<APIResponse<{
    valid: boolean;
    issuedAt: string;
    requestId: string;
  }>> {
    return this.makeRequest('POST', `/certificates/${certificateId}/verify`);
  }

  async listCertificates(subjectId: string): Promise<PaginatedResponse<DeletionCertificate>> {
    return this.makeRequest('GET', `/subjects/${subjectId}/certificates`);
  }

  // ==========================================================================
  // Retention Policy Operations
  // ==========================================================================

  async getRetentionPolicy(policyId: string): Promise<APIResponse<RetentionPolicy>> {
    return this.makeRequest('GET', `/policies/${policyId}`);
  }

  async listRetentionPolicies(controllerId: string): Promise<PaginatedResponse<RetentionPolicy>> {
    return this.makeRequest('GET', `/controllers/${controllerId}/policies`);
  }

  async createRetentionPolicy(policy: Omit<RetentionPolicy, 'policyId' | 'createdAt'>): Promise<APIResponse<RetentionPolicy>> {
    return this.makeRequest('POST', '/policies', policy);
  }

  async updateRetentionPolicy(policyId: string, updates: Partial<RetentionPolicy>): Promise<APIResponse<RetentionPolicy>> {
    return this.makeRequest('PATCH', `/policies/${policyId}`, updates);
  }

  // ==========================================================================
  // Compliance Operations
  // ==========================================================================

  async getComplianceReport(controllerId: string, period: 'month' | 'quarter' | 'year'): Promise<APIResponse<ComplianceReport>> {
    return this.makeRequest('GET', `/controllers/${controllerId}/compliance?period=${period}`);
  }

  async getComplianceMetrics(controllerId: string): Promise<APIResponse<{
    averageResponseTime: number;
    completionRate: number;
    pendingRequests: number;
    overdueRequests: number;
  }>> {
    return this.makeRequest('GET', `/controllers/${controllerId}/metrics`);
  }

  async generateAuditLog(controllerId: string, startDate: string, endDate: string): Promise<APIResponse<{
    logId: string;
    downloadUrl: string;
    expiresAt: string;
  }>> {
    return this.makeRequest('POST', `/controllers/${controllerId}/audit-log`, { startDate, endDate });
  }

  // ==========================================================================
  // Event Handling
  // ==========================================================================

  on(event: 'requestSubmitted' | 'requestCompleted' | 'certificateIssued' | 'deadlineApproaching' | 'error', callback: (...args: unknown[]) => void): void {
    this.eventEmitter.on(event, callback);
  }

  // ==========================================================================
  // Private Methods
  // ==========================================================================

  private async makeRequest<T>(method: string, path: string, body?: unknown): Promise<T> {
    const url = `${this.config.endpoint}${path}`;

    if (this.config.debug) {
      console.log(`[WIA RTBF] ${method} ${url}`, body);
    }

    try {
      const response = await fetch(url, {
        method,
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${this.config.apiKey}`,
          'X-WIA-Standard': 'LEG-009',
          'X-WIA-Version': '1.0.0',
          ...(this.config.controllerId && { 'X-Controller-ID': this.config.controllerId }),
        },
        body: body ? JSON.stringify(body) : undefined,
      });

      return await response.json();
    } catch (error: unknown) {
      const message = error instanceof Error ? error.message : 'Unknown error';
      this.eventEmitter.emit('error', { code: 'REQUEST_FAILED', message });
      throw error;
    }
  }
}

export function createClient(config: WIARTBFConfig): WIARTBFClient {
  return new WIARTBFClient(config);
}

export default WIARTBFClient;
