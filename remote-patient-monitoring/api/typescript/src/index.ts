/**
 * WIA-MED-027: Remote Patient Monitoring Standard - TypeScript SDK
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
  Patient,
  MonitoringDevice,
  VitalReading,
  MonitoringPlan,
  Alert,
  CareTeamMember,
  MonitoringSession,
  VitalType,
  AlertSeverity,
  DeviceStatus,
  ReadingStatus,
} from './types';

export * from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface WIARemoteMonitoringConfig extends WIAConfig {
  providerId?: string;
}

// ============================================================================
// Main SDK Client
// ============================================================================

export class WIARemoteMonitoringClient {
  private config: Required<WIARemoteMonitoringConfig>;
  private eventEmitter = new EventEmitter();

  constructor(config: WIARemoteMonitoringConfig) {
    this.config = {
      endpoint: 'https://api.wia-standards.org/v1/remote-monitoring',
      timeout: 30000,
      debug: false,
      providerId: '',
      ...config,
    };

    if (!this.config.apiKey) {
      throw new Error('API key is required');
    }
  }

  // ==========================================================================
  // Patient Operations
  // ==========================================================================

  async getPatient(patientId: string): Promise<APIResponse<Patient>> {
    return this.makeRequest('GET', `/patients/${patientId}`);
  }

  async listPatients(filters?: {
    status?: string;
    careTeamId?: string;
  }): Promise<PaginatedResponse<Patient>> {
    const params = new URLSearchParams(filters as Record<string, string>);
    return this.makeRequest('GET', `/patients?${params}`);
  }

  async enrollPatient(patient: Omit<Patient, 'patientId' | 'enrolledAt'>): Promise<APIResponse<Patient>> {
    return this.makeRequest('POST', '/patients', patient);
  }

  async updatePatient(patientId: string, updates: Partial<Patient>): Promise<APIResponse<Patient>> {
    return this.makeRequest('PATCH', `/patients/${patientId}`, updates);
  }

  async dischargePatient(patientId: string, reason: string): Promise<APIResponse<Patient>> {
    return this.makeRequest('POST', `/patients/${patientId}/discharge`, { reason });
  }

  // ==========================================================================
  // Device Operations
  // ==========================================================================

  async getDevice(deviceId: string): Promise<APIResponse<MonitoringDevice>> {
    return this.makeRequest('GET', `/devices/${deviceId}`);
  }

  async listDevices(patientId: string): Promise<PaginatedResponse<MonitoringDevice>> {
    return this.makeRequest('GET', `/patients/${patientId}/devices`);
  }

  async registerDevice(device: Omit<MonitoringDevice, 'deviceId' | 'registeredAt'>): Promise<APIResponse<MonitoringDevice>> {
    return this.makeRequest('POST', '/devices', device);
  }

  async updateDeviceStatus(deviceId: string, status: DeviceStatus): Promise<APIResponse<MonitoringDevice>> {
    return this.makeRequest('PATCH', `/devices/${deviceId}/status`, { status });
  }

  async syncDevice(deviceId: string): Promise<APIResponse<{ lastSync: string; readingsUploaded: number }>> {
    return this.makeRequest('POST', `/devices/${deviceId}/sync`);
  }

  // ==========================================================================
  // Vital Reading Operations
  // ==========================================================================

  async submitReading(reading: Omit<VitalReading, 'readingId' | 'timestamp'>): Promise<APIResponse<VitalReading>> {
    return this.makeRequest('POST', '/readings', reading);
  }

  async getReading(readingId: string): Promise<APIResponse<VitalReading>> {
    return this.makeRequest('GET', `/readings/${readingId}`);
  }

  async getReadingHistory(patientId: string, filters?: {
    vitalType?: VitalType;
    startDate?: string;
    endDate?: string;
    limit?: number;
  }): Promise<PaginatedResponse<VitalReading>> {
    const params = new URLSearchParams({ patientId, ...filters as Record<string, string> });
    return this.makeRequest('GET', `/readings?${params}`);
  }

  async getLatestReadings(patientId: string): Promise<APIResponse<Record<VitalType, VitalReading>>> {
    return this.makeRequest('GET', `/patients/${patientId}/readings/latest`);
  }

  async flagReading(readingId: string, status: ReadingStatus, notes?: string): Promise<APIResponse<VitalReading>> {
    return this.makeRequest('PATCH', `/readings/${readingId}/flag`, { status, notes });
  }

  // ==========================================================================
  // Monitoring Plan Operations
  // ==========================================================================

  async getMonitoringPlan(planId: string): Promise<APIResponse<MonitoringPlan>> {
    return this.makeRequest('GET', `/plans/${planId}`);
  }

  async createMonitoringPlan(plan: Omit<MonitoringPlan, 'planId' | 'createdAt'>): Promise<APIResponse<MonitoringPlan>> {
    return this.makeRequest('POST', '/plans', plan);
  }

  async updateMonitoringPlan(planId: string, updates: Partial<MonitoringPlan>): Promise<APIResponse<MonitoringPlan>> {
    return this.makeRequest('PATCH', `/plans/${planId}`, updates);
  }

  async getPatientPlan(patientId: string): Promise<APIResponse<MonitoringPlan>> {
    return this.makeRequest('GET', `/patients/${patientId}/plan`);
  }

  // ==========================================================================
  // Alert Operations
  // ==========================================================================

  async getAlerts(patientId: string): Promise<PaginatedResponse<Alert>> {
    return this.makeRequest('GET', `/patients/${patientId}/alerts`);
  }

  async acknowledgeAlert(alertId: string, acknowledgedBy: string): Promise<APIResponse<Alert>> {
    return this.makeRequest('POST', `/alerts/${alertId}/acknowledge`, { acknowledgedBy });
  }

  async resolveAlert(alertId: string, resolution: string): Promise<APIResponse<Alert>> {
    return this.makeRequest('POST', `/alerts/${alertId}/resolve`, { resolution });
  }

  async getAlertsByProvider(providerId: string, severity?: AlertSeverity): Promise<PaginatedResponse<Alert>> {
    const params = severity ? `?severity=${severity}` : '';
    return this.makeRequest('GET', `/providers/${providerId}/alerts${params}`);
  }

  // ==========================================================================
  // Care Team Operations
  // ==========================================================================

  async getCareTeam(patientId: string): Promise<PaginatedResponse<CareTeamMember>> {
    return this.makeRequest('GET', `/patients/${patientId}/care-team`);
  }

  async addCareTeamMember(patientId: string, member: Omit<CareTeamMember, 'memberId'>): Promise<APIResponse<CareTeamMember>> {
    return this.makeRequest('POST', `/patients/${patientId}/care-team`, member);
  }

  async removeCareTeamMember(patientId: string, memberId: string): Promise<APIResponse<void>> {
    return this.makeRequest('DELETE', `/patients/${patientId}/care-team/${memberId}`);
  }

  // ==========================================================================
  // Analytics
  // ==========================================================================

  async getPatientTrends(patientId: string, vitalType: VitalType, days: number): Promise<APIResponse<{
    dates: string[];
    values: number[];
    average: number;
    min: number;
    max: number;
    trend: 'improving' | 'stable' | 'declining';
  }>> {
    return this.makeRequest('GET', `/patients/${patientId}/trends/${vitalType}?days=${days}`);
  }

  async getComplianceReport(patientId: string): Promise<APIResponse<{
    overallCompliance: number;
    byVitalType: Record<VitalType, number>;
    missedReadings: number;
  }>> {
    return this.makeRequest('GET', `/patients/${patientId}/compliance`);
  }

  // ==========================================================================
  // Event Handling
  // ==========================================================================

  on(event: 'readingReceived' | 'alertTriggered' | 'deviceOffline' | 'thresholdExceeded' | 'error', callback: (...args: unknown[]) => void): void {
    this.eventEmitter.on(event, callback);
  }

  // ==========================================================================
  // Private Methods
  // ==========================================================================

  private async makeRequest<T>(method: string, path: string, body?: unknown): Promise<T> {
    const url = `${this.config.endpoint}${path}`;

    if (this.config.debug) {
      console.log(`[WIA Remote Monitoring] ${method} ${url}`, body);
    }

    try {
      const response = await fetch(url, {
        method,
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${this.config.apiKey}`,
          'X-WIA-Standard': 'MED-027',
          'X-WIA-Version': '1.0.0',
          ...(this.config.providerId && { 'X-Provider-ID': this.config.providerId }),
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

export function createClient(config: WIARemoteMonitoringConfig): WIARemoteMonitoringClient {
  return new WIARemoteMonitoringClient(config);
}

export default WIARemoteMonitoringClient;
