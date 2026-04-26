/**
 * WIA-MED-025: Emergency Medical Data Monitoring Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @packageDocumentation
 * @module wia-emergency-medical-data
 * @license MIT
 * @author WIA / SmileStory Inc.
 * @version 1.0.0
 */

export * from './types';

import {
  WearableDevice,
  HealthMetrics,
  VitalSigns,
  EmergencyAlert,
  Patient,
  MedicalRecord,
  Medication,
  AllergicReaction,
  APIResponse,
  PaginatedResponse,
} from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface EmergencyMedicalSDKConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  retries?: number;
  debug?: boolean;
}

// ============================================================================
// SDK Client
// ============================================================================

export class EmergencyMedicalSDK {
  private config: Required<EmergencyMedicalSDKConfig>;
  private headers: Record<string, string>;

  constructor(config: EmergencyMedicalSDKConfig) {
    this.config = {
      timeout: 30000,
      retries: 3,
      debug: false,
      ...config,
    };

    this.headers = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'X-WIA-Standard': 'MED-025',
      'X-WIA-Version': '1.0.0',
    };
  }

  // ==========================================================================
  // Device Management APIs
  // ==========================================================================

  async listDevices(): Promise<APIResponse<WearableDevice[]>> {
    return this.get<WearableDevice[]>('/api/v1/devices');
  }

  async getDevice(deviceId: string): Promise<APIResponse<WearableDevice>> {
    return this.get<WearableDevice>(`/api/v1/devices/${deviceId}`);
  }

  async registerDevice(device: Omit<WearableDevice, 'deviceId'>): Promise<APIResponse<WearableDevice>> {
    return this.post<WearableDevice>('/api/v1/devices', device);
  }

  async syncDevice(deviceId: string): Promise<APIResponse<{ syncedAt: string }>> {
    return this.post<{ syncedAt: string }>(`/api/v1/devices/${deviceId}/sync`, {});
  }

  async getDeviceStatus(deviceId: string): Promise<APIResponse<{ status: string; battery: number }>> {
    return this.get<{ status: string; battery: number }>(`/api/v1/devices/${deviceId}/status`);
  }

  // ==========================================================================
  // Health Metrics APIs
  // ==========================================================================

  async getMetrics(deviceId: string): Promise<APIResponse<HealthMetrics[]>> {
    return this.get<HealthMetrics[]>(`/api/v1/devices/${deviceId}/metrics`);
  }

  async getLatestMetrics(deviceId: string): Promise<APIResponse<HealthMetrics>> {
    return this.get<HealthMetrics>(`/api/v1/devices/${deviceId}/metrics/latest`);
  }

  async submitMetrics(deviceId: string, metrics: Omit<HealthMetrics, 'metricsId'>): Promise<APIResponse<HealthMetrics>> {
    return this.post<HealthMetrics>(`/api/v1/devices/${deviceId}/metrics`, metrics);
  }

  // ==========================================================================
  // Vital Signs APIs
  // ==========================================================================

  async getVitalSigns(patientId: string): Promise<APIResponse<VitalSigns[]>> {
    return this.get<VitalSigns[]>(`/api/v1/patients/${patientId}/vitals`);
  }

  async getLatestVitals(patientId: string): Promise<APIResponse<VitalSigns>> {
    return this.get<VitalSigns>(`/api/v1/patients/${patientId}/vitals/latest`);
  }

  async submitVitals(patientId: string, vitals: Omit<VitalSigns, 'vitalsId'>): Promise<APIResponse<VitalSigns>> {
    return this.post<VitalSigns>(`/api/v1/patients/${patientId}/vitals`, vitals);
  }

  // ==========================================================================
  // Patient APIs
  // ==========================================================================

  async getPatient(patientId: string): Promise<APIResponse<Patient>> {
    return this.get<Patient>(`/api/v1/patients/${patientId}`);
  }

  async createPatient(patient: Omit<Patient, 'patientId'>): Promise<APIResponse<Patient>> {
    return this.post<Patient>('/api/v1/patients', patient);
  }

  async updatePatient(patientId: string, updates: Partial<Patient>): Promise<APIResponse<Patient>> {
    return this.put<Patient>(`/api/v1/patients/${patientId}`, updates);
  }

  // ==========================================================================
  // Medical Record APIs
  // ==========================================================================

  async getMedicalRecords(patientId: string): Promise<APIResponse<MedicalRecord[]>> {
    return this.get<MedicalRecord[]>(`/api/v1/patients/${patientId}/records`);
  }

  async addMedicalRecord(patientId: string, record: Omit<MedicalRecord, 'recordId'>): Promise<APIResponse<MedicalRecord>> {
    return this.post<MedicalRecord>(`/api/v1/patients/${patientId}/records`, record);
  }

  // ==========================================================================
  // Emergency Alert APIs
  // ==========================================================================

  async triggerEmergencyAlert(alert: Omit<EmergencyAlert, 'alertId'>): Promise<APIResponse<EmergencyAlert>> {
    return this.post<EmergencyAlert>('/api/v1/emergency/alerts', alert);
  }

  async getActiveAlerts(): Promise<APIResponse<EmergencyAlert[]>> {
    return this.get<EmergencyAlert[]>('/api/v1/emergency/alerts/active');
  }

  async acknowledgeAlert(alertId: string): Promise<APIResponse<EmergencyAlert>> {
    return this.post<EmergencyAlert>(`/api/v1/emergency/alerts/${alertId}/acknowledge`, {});
  }

  async resolveAlert(alertId: string, resolution: string): Promise<APIResponse<EmergencyAlert>> {
    return this.post<EmergencyAlert>(`/api/v1/emergency/alerts/${alertId}/resolve`, { resolution });
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

  private async request<T>(method: string, path: string, data?: unknown): Promise<APIResponse<T>> {
    const url = `${this.config.endpoint}${path}`;
    const response = await fetch(url, {
      method,
      headers: this.headers,
      body: data ? JSON.stringify(data) : undefined,
    });
    return response.json();
  }
}

// ============================================================================
// Legacy Support
// ============================================================================

export interface WIAWearableConfig {
  baseUrl: string;
  apiKey: string;
  timeout?: number;
}

export class WIAWearable extends EmergencyMedicalSDK {
  constructor(config: WIAWearableConfig) {
    super({
      apiKey: config.apiKey,
      endpoint: config.baseUrl,
      timeout: config.timeout,
    });
  }
}

export function createClient(config: WIAWearableConfig): WIAWearable {
  return new WIAWearable(config);
}

// ============================================================================
// Constants
// ============================================================================

export const VERSION = '1.0.0';

export const VITAL_SIGN_THRESHOLDS = {
  heartRate: { min: 60, max: 100 },
  bloodPressureSystolic: { min: 90, max: 140 },
  bloodPressureDiastolic: { min: 60, max: 90 },
  temperature: { min: 36.1, max: 37.2 },
  oxygenSaturation: { min: 95, max: 100 },
} as const;

export const ALERT_PRIORITIES = {
  CRITICAL: 1,
  HIGH: 2,
  MEDIUM: 3,
  LOW: 4,
} as const;

export default EmergencyMedicalSDK;
