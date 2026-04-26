/**
 * WIA-MED-022: Medication Adherence Standard - TypeScript SDK
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
  Medication,
  MedicationSchedule,
  AdherenceRecord,
  AdherenceMetrics,
  MonitoringDevice,
  AdherenceAlert,
  AdherenceStatus,
} from './types';

export * from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface WIAMedicationAdherenceConfig extends WIAConfig {
  patientId?: string;
}

// ============================================================================
// Main SDK Client
// ============================================================================

export class WIAMedicationAdherenceClient {
  private config: Required<WIAMedicationAdherenceConfig>;
  private eventEmitter = new EventEmitter();

  constructor(config: WIAMedicationAdherenceConfig) {
    this.config = {
      endpoint: 'https://api.wia-standards.org/v1/medication-adherence',
      timeout: 30000,
      debug: false,
      patientId: '',
      ...config,
    };

    if (!this.config.apiKey) {
      throw new Error('API key is required');
    }
  }

  // ==========================================================================
  // Medication Operations
  // ==========================================================================

  async getMedication(medicationId: string): Promise<APIResponse<Medication>> {
    return this.makeRequest('GET', `/medications/${medicationId}`);
  }

  async listMedications(patientId: string): Promise<PaginatedResponse<Medication>> {
    return this.makeRequest('GET', `/patients/${patientId}/medications`);
  }

  async addMedication(patientId: string, medication: Omit<Medication, 'medicationId'>): Promise<APIResponse<Medication>> {
    return this.makeRequest('POST', `/patients/${patientId}/medications`, medication);
  }

  async updateMedication(medicationId: string, updates: Partial<Medication>): Promise<APIResponse<Medication>> {
    return this.makeRequest('PATCH', `/medications/${medicationId}`, updates);
  }

  async discontinueMedication(medicationId: string, reason: string): Promise<APIResponse<Medication>> {
    return this.makeRequest('POST', `/medications/${medicationId}/discontinue`, { reason });
  }

  // ==========================================================================
  // Schedule Operations
  // ==========================================================================

  async getSchedule(scheduleId: string): Promise<APIResponse<MedicationSchedule>> {
    return this.makeRequest('GET', `/schedules/${scheduleId}`);
  }

  async listSchedules(patientId: string): Promise<PaginatedResponse<MedicationSchedule>> {
    return this.makeRequest('GET', `/patients/${patientId}/schedules`);
  }

  async createSchedule(schedule: Omit<MedicationSchedule, 'scheduleId'>): Promise<APIResponse<MedicationSchedule>> {
    return this.makeRequest('POST', '/schedules', schedule);
  }

  async updateSchedule(scheduleId: string, updates: Partial<MedicationSchedule>): Promise<APIResponse<MedicationSchedule>> {
    return this.makeRequest('PATCH', `/schedules/${scheduleId}`, updates);
  }

  // ==========================================================================
  // Adherence Operations
  // ==========================================================================

  async recordAdherence(record: Omit<AdherenceRecord, 'recordId'>): Promise<APIResponse<AdherenceRecord>> {
    return this.makeRequest('POST', '/adherence-records', record);
  }

  async getAdherenceRecords(patientId: string, filters?: {
    medicationId?: string;
    status?: AdherenceStatus;
    startDate?: string;
    endDate?: string;
  }): Promise<PaginatedResponse<AdherenceRecord>> {
    const params = new URLSearchParams({ patientId, ...filters as Record<string, string> });
    return this.makeRequest('GET', `/adherence-records?${params}`);
  }

  async getAdherenceMetrics(patientId: string, period?: string): Promise<APIResponse<AdherenceMetrics>> {
    const params = period ? `?period=${period}` : '';
    return this.makeRequest('GET', `/patients/${patientId}/adherence-metrics${params}`);
  }

  async getMedicationAdherenceRate(medicationId: string): Promise<APIResponse<{
    rate: number;
    streak: number;
    trend: 'improving' | 'stable' | 'declining';
  }>> {
    return this.makeRequest('GET', `/medications/${medicationId}/adherence-rate`);
  }

  // ==========================================================================
  // Device Operations
  // ==========================================================================

  async registerDevice(device: Omit<MonitoringDevice, 'deviceId' | 'lastSync'>): Promise<APIResponse<MonitoringDevice>> {
    return this.makeRequest('POST', '/devices', device);
  }

  async getDevice(deviceId: string): Promise<APIResponse<MonitoringDevice>> {
    return this.makeRequest('GET', `/devices/${deviceId}`);
  }

  async syncDevice(deviceId: string): Promise<APIResponse<{
    recordsSync: number;
    lastSync: string;
  }>> {
    return this.makeRequest('POST', `/devices/${deviceId}/sync`);
  }

  async listDevices(patientId: string): Promise<PaginatedResponse<MonitoringDevice>> {
    return this.makeRequest('GET', `/patients/${patientId}/devices`);
  }

  // ==========================================================================
  // Alert Operations
  // ==========================================================================

  async getAlerts(patientId: string): Promise<PaginatedResponse<AdherenceAlert>> {
    return this.makeRequest('GET', `/patients/${patientId}/alerts`);
  }

  async acknowledgeAlert(alertId: string): Promise<APIResponse<AdherenceAlert>> {
    return this.makeRequest('POST', `/alerts/${alertId}/acknowledge`);
  }

  // ==========================================================================
  // Event Handling
  // ==========================================================================

  on(event: 'doseTaken' | 'doseMissed' | 'refillNeeded' | 'deviceSynced' | 'error', callback: (...args: unknown[]) => void): void {
    this.eventEmitter.on(event, callback);
  }

  // ==========================================================================
  // Private Methods
  // ==========================================================================

  private async makeRequest<T>(method: string, path: string, body?: unknown): Promise<T> {
    const url = `${this.config.endpoint}${path}`;

    if (this.config.debug) {
      console.log(`[WIA Medication Adherence] ${method} ${url}`, body);
    }

    try {
      const response = await fetch(url, {
        method,
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${this.config.apiKey}`,
          'X-WIA-Standard': 'MED-022',
          'X-WIA-Version': '1.0.0',
          ...(this.config.patientId && { 'X-Patient-ID': this.config.patientId }),
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

export function createClient(config: WIAMedicationAdherenceConfig): WIAMedicationAdherenceClient {
  return new WIAMedicationAdherenceClient(config);
}

export default WIAMedicationAdherenceClient;
