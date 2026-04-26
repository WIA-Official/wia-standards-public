/**
 * WIA-MED-014: Medical Alert System Standard - TypeScript SDK
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
  MedicalAlert,
  CreateAlertRequest,
  CreateAlertResponse,
  AlertType,
  AlertPriority,
  AlertStatus,
  EscalationPolicy,
  Recipient,
} from './types';

export * from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface WIAMedicalAlertConfig extends WIAConfig {
  organizationId?: string;
}

// ============================================================================
// Main SDK Client
// ============================================================================

export class MedicalAlertSDK {
  private config: Required<WIAMedicalAlertConfig>;
  private eventEmitter = new EventEmitter();

  constructor(config: WIAMedicalAlertConfig) {
    this.config = {
      endpoint: 'https://api.wia-standards.org/v1/medical-alert',
      timeout: 30000,
      debug: false,
      organizationId: '',
      ...config,
    };

    if (!this.config.apiKey) {
      throw new Error('API key is required');
    }
  }

  // ==========================================================================
  // Alert Operations
  // ==========================================================================

  async createAlert(request: CreateAlertRequest): Promise<CreateAlertResponse> {
    return this.makeRequest('POST', '/alerts', request);
  }

  async getAlert(alertId: string): Promise<APIResponse<MedicalAlert>> {
    return this.makeRequest('GET', `/alerts/${alertId}`);
  }

  async listAlerts(filters?: {
    patientId?: string;
    type?: AlertType;
    priority?: AlertPriority;
    status?: AlertStatus;
    startDate?: string;
    endDate?: string;
  }): Promise<PaginatedResponse<MedicalAlert>> {
    const params = new URLSearchParams(filters as Record<string, string>);
    return this.makeRequest('GET', `/alerts?${params}`);
  }

  async updateAlert(alertId: string, updates: Partial<MedicalAlert>): Promise<APIResponse<MedicalAlert>> {
    return this.makeRequest('PATCH', `/alerts/${alertId}`, updates);
  }

  async acknowledgeAlert(alertId: string, userId: string, notes?: string): Promise<APIResponse<MedicalAlert>> {
    return this.makeRequest('POST', `/alerts/${alertId}/acknowledge`, { userId, notes });
  }

  async resolveAlert(alertId: string, resolution: {
    resolvedBy: string;
    resolutionNotes: string;
    outcome: string;
  }): Promise<APIResponse<MedicalAlert>> {
    return this.makeRequest('POST', `/alerts/${alertId}/resolve`, resolution);
  }

  async cancelAlert(alertId: string, reason?: string): Promise<APIResponse<void>> {
    return this.makeRequest('DELETE', `/alerts/${alertId}`, { reason });
  }

  async escalateAlert(alertId: string, reason: string): Promise<APIResponse<MedicalAlert>> {
    return this.makeRequest('POST', `/alerts/${alertId}/escalate`, { reason });
  }

  // ==========================================================================
  // Recipient Operations
  // ==========================================================================

  async getRecipients(alertId: string): Promise<APIResponse<Recipient[]>> {
    return this.makeRequest('GET', `/alerts/${alertId}/recipients`);
  }

  async addRecipient(alertId: string, recipientId: string): Promise<APIResponse<MedicalAlert>> {
    return this.makeRequest('POST', `/alerts/${alertId}/recipients`, { recipientId });
  }

  // ==========================================================================
  // Escalation Policy Operations
  // ==========================================================================

  async getEscalationPolicies(): Promise<PaginatedResponse<EscalationPolicy>> {
    return this.makeRequest('GET', '/escalation-policies');
  }

  async getEscalationPolicy(policyId: string): Promise<APIResponse<EscalationPolicy>> {
    return this.makeRequest('GET', `/escalation-policies/${policyId}`);
  }

  async createEscalationPolicy(policy: Omit<EscalationPolicy, 'policyId'>): Promise<APIResponse<EscalationPolicy>> {
    return this.makeRequest('POST', '/escalation-policies', policy);
  }

  // ==========================================================================
  // Analytics
  // ==========================================================================

  async getAlertStats(filters?: {
    startDate?: string;
    endDate?: string;
    groupBy?: 'day' | 'week' | 'month';
  }): Promise<APIResponse<{
    totalAlerts: number;
    byType: Record<AlertType, number>;
    byPriority: Record<AlertPriority, number>;
    avgResponseTime: number;
    acknowledgeRate: number;
  }>> {
    const params = new URLSearchParams(filters as Record<string, string>);
    return this.makeRequest('GET', `/analytics/alerts?${params}`);
  }

  // ==========================================================================
  // Event Handling
  // ==========================================================================

  on(event: 'alertCreated' | 'alertAcknowledged' | 'alertEscalated' | 'alertResolved' | 'error', callback: (...args: unknown[]) => void): void {
    this.eventEmitter.on(event, callback);
  }

  off(event: string, callback: (...args: unknown[]) => void): void {
    this.eventEmitter.off(event, callback);
  }

  // ==========================================================================
  // Private Methods
  // ==========================================================================

  private async makeRequest<T>(method: string, path: string, body?: unknown): Promise<T> {
    const url = `${this.config.endpoint}${path}`;

    if (this.config.debug) {
      console.log(`[WIA Medical Alert] ${method} ${url}`, body);
    }

    try {
      const response = await fetch(url, {
        method,
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${this.config.apiKey}`,
          'X-WIA-Standard': 'MED-014',
          'X-WIA-Version': '1.0.0',
          ...(this.config.organizationId && { 'X-Organization-ID': this.config.organizationId }),
        },
        body: body ? JSON.stringify(body) : undefined,
      });

      const data = await response.json();

      if (!response.ok) {
        this.eventEmitter.emit('error', data);
      }

      return data;
    } catch (error: unknown) {
      const message = error instanceof Error ? error.message : 'Unknown error';
      this.eventEmitter.emit('error', { code: 'REQUEST_FAILED', message });
      throw error;
    }
  }
}

export function createClient(config: WIAMedicalAlertConfig): MedicalAlertSDK {
  return new MedicalAlertSDK(config);
}

export default MedicalAlertSDK;
