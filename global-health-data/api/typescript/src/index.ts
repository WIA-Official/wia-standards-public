/**
 * WIA-MED-020: Global Health Data Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @packageDocumentation
 * @module wia-global-health-data
 * @license MIT
 * @author WIA / SmileStory Inc.
 * @version 1.0.0
 */

import EventEmitter from 'eventemitter3';
import {
  WIAConfig,
  APIResponse,
  DataRecord,
  HealthDataset,
  DataExchange,
  ConsentRecord,
  AuditLog,
  DataQualityReport,
  AnonymizationConfig,
  PaginatedResponse,
} from './types';

export * from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface GlobalHealthDataConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  retries?: number;
  debug?: boolean;
}

// ============================================================================
// SDK Client
// ============================================================================

export class GlobalHealthDataClient {
  private config: Required<GlobalHealthDataConfig>;
  private headers: Record<string, string>;
  private eventEmitter = new EventEmitter();

  constructor(config: GlobalHealthDataConfig) {
    this.config = {
      timeout: 60000,
      retries: 3,
      debug: false,
      ...config,
    };

    this.headers = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'X-WIA-Standard': 'MED-020',
      'X-WIA-Version': '1.0.0',
    };
  }

  // ==========================================================================
  // Data Record APIs
  // ==========================================================================

  async createDataRecord(record: Omit<DataRecord, 'recordId'>): Promise<APIResponse<DataRecord>> {
    return this.post<DataRecord>('/api/v1/records', record);
  }

  async getDataRecord(recordId: string): Promise<APIResponse<DataRecord>> {
    return this.get<DataRecord>(`/api/v1/records/${recordId}`);
  }

  async updateDataRecord(recordId: string, updates: Partial<DataRecord>): Promise<APIResponse<DataRecord>> {
    return this.put<DataRecord>(`/api/v1/records/${recordId}`, updates);
  }

  async deleteDataRecord(recordId: string): Promise<APIResponse<void>> {
    return this.delete<void>(`/api/v1/records/${recordId}`);
  }

  async searchRecords(query: Record<string, unknown>): Promise<APIResponse<DataRecord[]>> {
    const queryString = this.buildQueryParams(query);
    return this.get<DataRecord[]>(`/api/v1/records${queryString}`);
  }

  // ==========================================================================
  // Dataset APIs
  // ==========================================================================

  async createDataset(dataset: Omit<HealthDataset, 'datasetId'>): Promise<APIResponse<HealthDataset>> {
    return this.post<HealthDataset>('/api/v1/datasets', dataset);
  }

  async getDataset(datasetId: string): Promise<APIResponse<HealthDataset>> {
    return this.get<HealthDataset>(`/api/v1/datasets/${datasetId}`);
  }

  async listDatasets(): Promise<APIResponse<HealthDataset[]>> {
    return this.get<HealthDataset[]>('/api/v1/datasets');
  }

  async addRecordToDataset(datasetId: string, recordId: string): Promise<APIResponse<void>> {
    return this.post<void>(`/api/v1/datasets/${datasetId}/records`, { recordId });
  }

  // ==========================================================================
  // Data Exchange APIs
  // ==========================================================================

  async initiateExchange(exchange: Omit<DataExchange, 'exchangeId'>): Promise<APIResponse<DataExchange>> {
    return this.post<DataExchange>('/api/v1/exchanges', exchange);
  }

  async getExchange(exchangeId: string): Promise<APIResponse<DataExchange>> {
    return this.get<DataExchange>(`/api/v1/exchanges/${exchangeId}`);
  }

  async approveExchange(exchangeId: string): Promise<APIResponse<DataExchange>> {
    return this.post<DataExchange>(`/api/v1/exchanges/${exchangeId}/approve`, {});
  }

  async rejectExchange(exchangeId: string, reason: string): Promise<APIResponse<DataExchange>> {
    return this.post<DataExchange>(`/api/v1/exchanges/${exchangeId}/reject`, { reason });
  }

  // ==========================================================================
  // Consent APIs
  // ==========================================================================

  async recordConsent(consent: Omit<ConsentRecord, 'consentId'>): Promise<APIResponse<ConsentRecord>> {
    return this.post<ConsentRecord>('/api/v1/consents', consent);
  }

  async getConsent(consentId: string): Promise<APIResponse<ConsentRecord>> {
    return this.get<ConsentRecord>(`/api/v1/consents/${consentId}`);
  }

  async revokeConsent(consentId: string): Promise<APIResponse<ConsentRecord>> {
    return this.post<ConsentRecord>(`/api/v1/consents/${consentId}/revoke`, {});
  }

  async getPatientConsents(patientId: string): Promise<APIResponse<ConsentRecord[]>> {
    return this.get<ConsentRecord[]>(`/api/v1/patients/${patientId}/consents`);
  }

  // ==========================================================================
  // Data Quality APIs
  // ==========================================================================

  async validateData(recordId: string): Promise<APIResponse<DataQualityReport>> {
    return this.post<DataQualityReport>(`/api/v1/records/${recordId}/validate`, {});
  }

  async getQualityReport(datasetId: string): Promise<APIResponse<DataQualityReport>> {
    return this.get<DataQualityReport>(`/api/v1/datasets/${datasetId}/quality`);
  }

  // ==========================================================================
  // Anonymization APIs
  // ==========================================================================

  async anonymizeRecord(recordId: string, config: AnonymizationConfig): Promise<APIResponse<DataRecord>> {
    return this.post<DataRecord>(`/api/v1/records/${recordId}/anonymize`, config);
  }

  async anonymizeDataset(datasetId: string, config: AnonymizationConfig): Promise<APIResponse<HealthDataset>> {
    return this.post<HealthDataset>(`/api/v1/datasets/${datasetId}/anonymize`, config);
  }

  // ==========================================================================
  // Audit APIs
  // ==========================================================================

  async getAuditLogs(recordId: string): Promise<APIResponse<AuditLog[]>> {
    return this.get<AuditLog[]>(`/api/v1/records/${recordId}/audit`);
  }

  // ==========================================================================
  // Event Handling
  // ==========================================================================

  on(event: string, callback: (...args: unknown[]) => void): void {
    this.eventEmitter.on(event, callback);
  }

  off(event: string, callback: (...args: unknown[]) => void): void {
    this.eventEmitter.off(event, callback);
  }

  // ==========================================================================
  // HTTP Methods
  // ==========================================================================

  private async get<T>(path: string): Promise<APIResponse<T>> {
    return this.request<T>('GET', path);
  }

  private async post<T>(path: string, data: unknown): Promise<APIResponse<T>> {
    return this.request<T>('POST', path, data);
  }

  private async put<T>(path: string, data: unknown): Promise<APIResponse<T>> {
    return this.request<T>('PUT', path, data);
  }

  private async delete<T>(path: string): Promise<APIResponse<T>> {
    return this.request<T>('DELETE', path);
  }

  private async request<T>(method: string, path: string, data?: unknown): Promise<APIResponse<T>> {
    const url = `${this.config.endpoint}${path}`;
    if (this.config.debug) {
      console.log(`[WIA Global Health Data] ${method} ${url}`);
    }

    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), this.config.timeout);

    try {
      const response = await fetch(url, {
        method,
        headers: this.headers,
        body: data ? JSON.stringify(data) : undefined,
        signal: controller.signal,
      });
      clearTimeout(timeoutId);
      return response.json();
    } catch (error) {
      clearTimeout(timeoutId);
      return {
        success: false,
        error: error instanceof Error ? error.message : 'Request failed',
        timestamp: new Date().toISOString(),
      };
    }
  }

  private buildQueryParams(params?: Record<string, unknown>): string {
    if (!params) return '';
    const searchParams = new URLSearchParams();
    Object.entries(params).forEach(([key, value]) => {
      if (value !== undefined) searchParams.append(key, String(value));
    });
    return searchParams.toString() ? `?${searchParams.toString()}` : '';
  }
}

// ============================================================================
// Factory Function
// ============================================================================

export function createClient(config: GlobalHealthDataConfig): GlobalHealthDataClient {
  return new GlobalHealthDataClient(config);
}

// ============================================================================
// Constants
// ============================================================================

export const VERSION = '1.0.0';

export const DATA_CATEGORIES = {
  CLINICAL: 'clinical',
  GENOMIC: 'genomic',
  IMAGING: 'imaging',
  BEHAVIORAL: 'behavioral',
  ENVIRONMENTAL: 'environmental',
} as const;

export const SENSITIVITY_LEVELS = {
  PUBLIC: 'public',
  INTERNAL: 'internal',
  CONFIDENTIAL: 'confidential',
  RESTRICTED: 'restricted',
} as const;

export default GlobalHealthDataClient;
