/**
 * WIA CRYO-008 Cryo Monitoring Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @packageDocumentation
 * @module wia-cryo-monitoring
 * @license MIT
 * @author WIA / SmileStory Inc.
 * @version 1.0.0
 */

export * from './types';

import {
  MonitoringRecord,
  MonitoringQuery,
  ApiResponse,
  MonitoringConfig,
  ValidationResult,
  CryoSubject,
  TemperatureReading,
  VitalSignsReading,
  Alert,
  MaintenanceLog,
  PaginatedResponse,
} from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface CryoMonitoringSDKConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  retries?: number;
  debug?: boolean;
}

// ============================================================================
// Main Client
// ============================================================================

export class CryoMonitoringClient {
  private config: Required<CryoMonitoringSDKConfig>;
  private headers: Record<string, string>;

  constructor(config: CryoMonitoringSDKConfig) {
    this.config = {
      timeout: 30000,
      retries: 3,
      debug: false,
      ...config,
    };

    this.headers = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'X-WIA-Standard': 'CRYO-008',
      'X-WIA-Version': '1.0.0',
    };
  }

  // ==========================================================================
  // Subject Management APIs
  // ==========================================================================

  async createSubject(subject: Omit<CryoSubject, 'subjectId'>): Promise<ApiResponse<CryoSubject>> {
    return this.post<CryoSubject>('/api/v1/subjects', subject);
  }

  async getSubject(subjectId: string): Promise<ApiResponse<CryoSubject>> {
    return this.get<CryoSubject>(`/api/v1/subjects/${subjectId}`);
  }

  async listSubjects(query?: MonitoringQuery): Promise<ApiResponse<PaginatedResponse<CryoSubject>>> {
    const queryString = this.buildQueryParams(query);
    return this.get<PaginatedResponse<CryoSubject>>(`/api/v1/subjects${queryString}`);
  }

  async updateSubject(subjectId: string, updates: Partial<CryoSubject>): Promise<ApiResponse<CryoSubject>> {
    return this.put<CryoSubject>(`/api/v1/subjects/${subjectId}`, updates);
  }

  // ==========================================================================
  // Monitoring Record APIs
  // ==========================================================================

  async submitMonitoringRecord(record: MonitoringRecord): Promise<ApiResponse<{ monitoringId: string }>> {
    const validation = this.validateRecord(record);
    if (!validation.valid) {
      return {
        success: false,
        error: 'Validation failed: ' + validation.errors.join(', '),
        errorCode: 'VALIDATION_ERROR',
      };
    }
    return this.post<{ monitoringId: string }>('/api/v1/monitoring', record);
  }

  async getMonitoringRecord(monitoringId: string): Promise<ApiResponse<MonitoringRecord>> {
    return this.get<MonitoringRecord>(`/api/v1/monitoring/${monitoringId}`);
  }

  async queryMonitoringHistory(
    subjectId: string,
    query?: MonitoringQuery
  ): Promise<ApiResponse<MonitoringRecord[]>> {
    const queryString = this.buildQueryParams(query);
    return this.get<MonitoringRecord[]>(`/api/v1/subjects/${subjectId}/monitoring${queryString}`);
  }

  // ==========================================================================
  // Temperature Monitoring APIs
  // ==========================================================================

  async recordTemperature(subjectId: string, reading: TemperatureReading): Promise<ApiResponse<TemperatureReading>> {
    return this.post<TemperatureReading>(`/api/v1/subjects/${subjectId}/temperature`, reading);
  }

  async getTemperatureHistory(subjectId: string, query?: MonitoringQuery): Promise<ApiResponse<TemperatureReading[]>> {
    const queryString = this.buildQueryParams(query);
    return this.get<TemperatureReading[]>(`/api/v1/subjects/${subjectId}/temperature${queryString}`);
  }

  async getLatestTemperature(subjectId: string): Promise<ApiResponse<TemperatureReading>> {
    return this.get<TemperatureReading>(`/api/v1/subjects/${subjectId}/temperature/latest`);
  }

  // ==========================================================================
  // Vital Signs APIs
  // ==========================================================================

  async recordVitalSigns(subjectId: string, vitals: VitalSignsReading): Promise<ApiResponse<VitalSignsReading>> {
    return this.post<VitalSignsReading>(`/api/v1/subjects/${subjectId}/vitals`, vitals);
  }

  async getVitalSignsHistory(subjectId: string, query?: MonitoringQuery): Promise<ApiResponse<VitalSignsReading[]>> {
    const queryString = this.buildQueryParams(query);
    return this.get<VitalSignsReading[]>(`/api/v1/subjects/${subjectId}/vitals${queryString}`);
  }

  // ==========================================================================
  // Alert Management APIs
  // ==========================================================================

  async createAlert(alert: Omit<Alert, 'alertId'>): Promise<ApiResponse<Alert>> {
    return this.post<Alert>('/api/v1/alerts', alert);
  }

  async getAlerts(subjectId: string): Promise<ApiResponse<Alert[]>> {
    return this.get<Alert[]>(`/api/v1/subjects/${subjectId}/alerts`);
  }

  async acknowledgeAlert(alertId: string): Promise<ApiResponse<Alert>> {
    return this.post<Alert>(`/api/v1/alerts/${alertId}/acknowledge`, {});
  }

  async resolveAlert(alertId: string, resolution: string): Promise<ApiResponse<Alert>> {
    return this.post<Alert>(`/api/v1/alerts/${alertId}/resolve`, { resolution });
  }

  // ==========================================================================
  // Maintenance APIs
  // ==========================================================================

  async logMaintenance(log: Omit<MaintenanceLog, 'logId'>): Promise<ApiResponse<MaintenanceLog>> {
    return this.post<MaintenanceLog>('/api/v1/maintenance', log);
  }

  async getMaintenanceHistory(equipmentId: string): Promise<ApiResponse<MaintenanceLog[]>> {
    return this.get<MaintenanceLog[]>(`/api/v1/equipment/${equipmentId}/maintenance`);
  }

  // ==========================================================================
  // Validation
  // ==========================================================================

  validateRecord(record: MonitoringRecord): ValidationResult {
    const errors: string[] = [];
    const warnings: string[] = [];

    if (!record.monitoringId || !/^MON-\d{4}-\d{6}$/.test(record.monitoringId)) {
      errors.push('Invalid monitoringId format (expected: MON-YYYY-NNNNNN)');
    }

    if (!record.subjectId || !/^CRYO-\d{4}-\d{4}$/.test(record.subjectId)) {
      errors.push('Invalid subjectId format (expected: CRYO-YYYY-NNNN)');
    }

    return { valid: errors.length === 0, errors, warnings };
  }

  // ==========================================================================
  // HTTP Methods
  // ==========================================================================

  private async get<T>(path: string): Promise<ApiResponse<T>> {
    return this.request<T>('GET', path);
  }

  private async post<T>(path: string, data: unknown): Promise<ApiResponse<T>> {
    return this.request<T>('POST', path, data);
  }

  private async put<T>(path: string, data: unknown): Promise<ApiResponse<T>> {
    return this.request<T>('PUT', path, data);
  }

  private async request<T>(method: string, path: string, data?: unknown): Promise<ApiResponse<T>> {
    const url = `${this.config.endpoint}${path}`;
    const response = await fetch(url, {
      method,
      headers: this.headers,
      body: data ? JSON.stringify(data) : undefined,
    });
    return response.json();
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
// Utility Functions
// ============================================================================

export const utils = {
  generateMonitoringId(): string {
    const year = new Date().getFullYear();
    const sequence = Math.floor(Math.random() * 1000000).toString().padStart(6, '0');
    return `MON-${year}-${sequence}`;
  },

  generateSubjectId(): string {
    const year = new Date().getFullYear();
    const sequence = Math.floor(Math.random() * 10000).toString().padStart(4, '0');
    return `CRYO-${year}-${sequence}`;
  },

  celsiusToKelvin(celsius: number): number {
    return celsius + 273.15;
  },

  kelvinToCelsius(kelvin: number): number {
    return kelvin - 273.15;
  },
};

// ============================================================================
// Constants
// ============================================================================

export const VERSION = '1.0.0';

export const TEMPERATURE_THRESHOLDS = {
  CRITICAL_HIGH: -180,
  WARNING_HIGH: -190,
  OPTIMAL: -196,
  WARNING_LOW: -200,
  CRITICAL_LOW: -210,
} as const;

export default CryoMonitoringClient;
