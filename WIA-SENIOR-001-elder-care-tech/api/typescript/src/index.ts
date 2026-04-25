/**
 * WIA-SENIOR-001: Elder Care Technology Standard
 * TypeScript SDK
 *
 * @philosophy 弘益人間 (Benefit All Humanity)
 * @version 1.0.0
 * @license Apache-2.0
 */

import {
  ElderCareConfig,
  ElderProfile,
  VitalSigns,
  Alert,
  Activity,
  Medication,
  CarePlan,
  Caregiver,
  ApiResponse,
  ElderCareEvent
} from './types';

export * from './types';

/**
 * Main Elder Care SDK Class
 */
export class ElderCareSDK {
  private config: ElderCareConfig;
  private apiUrl: string;
  private websocket?: WebSocket;
  private eventListeners: Map<string, Function[]> = new Map();

  constructor(config: ElderCareConfig) {
    this.config = config;
    this.apiUrl = config.apiUrl || 'https://api.wia.org/senior-001';

    if (config.enableRealTimeMonitoring) {
      this.initializeWebSocket();
    }
  }

  /**
   * Elder Profile Management
   */

  async getElderProfile(elderId: string): Promise<ApiResponse<ElderProfile>> {
    return this.request<ElderProfile>('GET', `/elders/${elderId}`);
  }

  async createElderProfile(profile: Partial<ElderProfile>): Promise<ApiResponse<ElderProfile>> {
    return this.request<ElderProfile>('POST', '/elders', profile);
  }

  async updateElderProfile(elderId: string, updates: Partial<ElderProfile>): Promise<ApiResponse<ElderProfile>> {
    return this.request<ElderProfile>('PATCH', `/elders/${elderId}`, updates);
  }

  async deleteElderProfile(elderId: string): Promise<ApiResponse<void>> {
    return this.request<void>('DELETE', `/elders/${elderId}`);
  }

  /**
   * Vital Signs Monitoring
   */

  async recordVitalSigns(vitals: Partial<VitalSigns>): Promise<ApiResponse<VitalSigns>> {
    return this.request<VitalSigns>('POST', '/vitals', vitals);
  }

  async getVitalSigns(elderId: string, startDate?: Date, endDate?: Date): Promise<ApiResponse<VitalSigns[]>> {
    const params = new URLSearchParams();
    if (startDate) params.append('startDate', startDate.toISOString());
    if (endDate) params.append('endDate', endDate.toISOString());

    return this.request<VitalSigns[]>('GET', `/elders/${elderId}/vitals?${params}`);
  }

  async getLatestVitalSigns(elderId: string): Promise<ApiResponse<VitalSigns>> {
    return this.request<VitalSigns>('GET', `/elders/${elderId}/vitals/latest`);
  }

  /**
   * Real-time Vital Signs Monitoring
   */

  monitorVitals(config: {
    heartRate?: boolean;
    bloodPressure?: boolean;
    temperature?: boolean;
    oxygenLevel?: boolean;
  }): { subscribe: (callback: (vitals: VitalSigns) => void) => void } {
    return {
      subscribe: (callback: (vitals: VitalSigns) => void) => {
        this.on('vitals', callback);
      }
    };
  }

  /**
   * Alert Management
   */

  async createAlert(alert: Partial<Alert>): Promise<ApiResponse<Alert>> {
    return this.request<Alert>('POST', '/alerts', alert);
  }

  async getAlerts(elderId: string, resolved?: boolean): Promise<ApiResponse<Alert[]>> {
    const params = new URLSearchParams();
    if (resolved !== undefined) params.append('resolved', String(resolved));

    return this.request<Alert[]>('GET', `/elders/${elderId}/alerts?${params}`);
  }

  async resolveAlert(alertId: string, resolvedBy: string): Promise<ApiResponse<Alert>> {
    return this.request<Alert>('PATCH', `/alerts/${alertId}/resolve`, { resolvedBy });
  }

  async sendAlert(alert: {
    type: string;
    severity: string;
    data?: any;
  }): Promise<ApiResponse<Alert>> {
    return this.createAlert({
      ...alert,
      elderId: this.config.patientId,
      timestamp: new Date(),
      resolved: false
    } as Alert);
  }

  /**
   * Activity Tracking
   */

  async recordActivity(activity: Partial<Activity>): Promise<ApiResponse<Activity>> {
    return this.request<Activity>('POST', '/activities', activity);
  }

  async getActivities(elderId: string, startDate?: Date, endDate?: Date): Promise<ApiResponse<Activity[]>> {
    const params = new URLSearchParams();
    if (startDate) params.append('startDate', startDate.toISOString());
    if (endDate) params.append('endDate', endDate.toISOString());

    return this.request<Activity[]>('GET', `/elders/${elderId}/activities?${params}`);
  }

  /**
   * Medication Management
   */

  async setMedicationReminder(medication: {
    medication: string;
    schedule: string;
    frequency: string;
  }): Promise<ApiResponse<Medication>> {
    return this.request<Medication>('POST', '/medications', {
      ...medication,
      elderId: this.config.patientId,
      startDate: new Date()
    });
  }

  async getMedications(elderId: string): Promise<ApiResponse<Medication[]>> {
    return this.request<Medication[]>('GET', `/elders/${elderId}/medications`);
  }

  async updateMedication(medicationId: string, updates: Partial<Medication>): Promise<ApiResponse<Medication>> {
    return this.request<Medication>('PATCH', `/medications/${medicationId}`, updates);
  }

  async deleteMedication(medicationId: string): Promise<ApiResponse<void>> {
    return this.request<void>('DELETE', `/medications/${medicationId}`);
  }

  async recordMedicationTaken(medicationId: string, timestamp: Date): Promise<ApiResponse<void>> {
    return this.request<void>('POST', `/medications/${medicationId}/taken`, { timestamp });
  }

  /**
   * Care Plan Management
   */

  async getCarePlan(elderId: string): Promise<ApiResponse<CarePlan>> {
    return this.request<CarePlan>('GET', `/elders/${elderId}/care-plan`);
  }

  async updateCarePlan(elderId: string, updates: Partial<CarePlan>): Promise<ApiResponse<CarePlan>> {
    return this.request<CarePlan>('PATCH', `/elders/${elderId}/care-plan`, updates);
  }

  /**
   * Caregiver Management
   */

  async getCaregivers(elderId: string): Promise<ApiResponse<Caregiver[]>> {
    return this.request<Caregiver[]>('GET', `/elders/${elderId}/caregivers`);
  }

  async assignCaregiver(elderId: string, caregiverId: string): Promise<ApiResponse<void>> {
    return this.request<void>('POST', `/elders/${elderId}/caregivers/${caregiverId}`);
  }

  async removeCaregiver(elderId: string, caregiverId: string): Promise<ApiResponse<void>> {
    return this.request<void>('DELETE', `/elders/${elderId}/caregivers/${caregiverId}`);
  }

  /**
   * Fall Detection
   */

  enableFallDetection(config: {
    sensitivity: 'low' | 'medium' | 'high';
    autoAlert: boolean;
  }): Promise<ApiResponse<void>> {
    return this.request<void>('POST', '/fall-detection/enable', {
      elderId: this.config.patientId,
      ...config
    });
  }

  disableFallDetection(): Promise<ApiResponse<void>> {
    return this.request<void>('POST', '/fall-detection/disable', {
      elderId: this.config.patientId
    });
  }

  /**
   * Emergency Services
   */

  async triggerEmergency(data?: {
    location?: { latitude: number; longitude: number };
    notes?: string;
  }): Promise<ApiResponse<Alert>> {
    return this.createAlert({
      type: 'EMERGENCY_BUTTON',
      severity: 'CRITICAL',
      elderId: this.config.patientId,
      title: 'Emergency Alert',
      message: 'Emergency button pressed',
      timestamp: new Date(),
      resolved: false,
      data
    } as Alert);
  }

  /**
   * Event System
   */

  on(event: string, callback: Function): void {
    if (!this.eventListeners.has(event)) {
      this.eventListeners.set(event, []);
    }
    this.eventListeners.get(event)!.push(callback);
  }

  off(event: string, callback: Function): void {
    const listeners = this.eventListeners.get(event);
    if (listeners) {
      const index = listeners.indexOf(callback);
      if (index > -1) {
        listeners.splice(index, 1);
      }
    }
  }

  private emit(event: string, data: any): void {
    const listeners = this.eventListeners.get(event);
    if (listeners) {
      listeners.forEach(callback => callback(data));
    }
  }

  /**
   * WebSocket for Real-time Updates
   */

  private initializeWebSocket(): void {
    const wsUrl = this.apiUrl.replace('https://', 'wss://').replace('http://', 'ws://');
    this.websocket = new WebSocket(`${wsUrl}/ws?apiKey=${this.config.apiKey}`);

    this.websocket.onmessage = (event) => {
      const data: ElderCareEvent = JSON.parse(event.data);
      this.emit(data.type, data.data);
    };

    this.websocket.onerror = (error) => {
      console.error('WebSocket error:', error);
    };

    this.websocket.onclose = () => {
      // Attempt to reconnect after 5 seconds
      setTimeout(() => this.initializeWebSocket(), 5000);
    };
  }

  /**
   * HTTP Request Helper
   */

  private async request<T>(
    method: string,
    endpoint: string,
    body?: any
  ): Promise<ApiResponse<T>> {
    try {
      const response = await fetch(`${this.apiUrl}${endpoint}`, {
        method,
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${this.config.apiKey}`,
          'X-WIA-Standard': 'SENIOR-001',
        },
        body: body ? JSON.stringify(body) : undefined,
      });

      const data = await response.json();

      if (!response.ok) {
        return {
          success: false,
          error: {
            code: `HTTP_${response.status}`,
            message: data.message || 'Request failed',
            details: data
          },
          timestamp: new Date()
        };
      }

      return {
        success: true,
        data: data,
        timestamp: new Date()
      };
    } catch (error) {
      return {
        success: false,
        error: {
          code: 'NETWORK_ERROR',
          message: error instanceof Error ? error.message : 'Unknown error',
        },
        timestamp: new Date()
      };
    }
  }

  /**
   * Cleanup
   */

  disconnect(): void {
    if (this.websocket) {
      this.websocket.close();
    }
    this.eventListeners.clear();
  }
}

/**
 * Utility Functions
 */

export function isVitalSignAbnormal(vitalSigns: VitalSigns, thresholds?: any): boolean {
  return !!(
    vitalSigns.heartRate?.isAbnormal ||
    vitalSigns.bloodPressure?.isAbnormal ||
    vitalSigns.temperature?.isAbnormal ||
    vitalSigns.oxygenSaturation?.isAbnormal ||
    vitalSigns.respiratoryRate?.isAbnormal ||
    vitalSigns.bloodGlucose?.isAbnormal
  );
}

export function calculateAge(dateOfBirth: Date): number {
  const today = new Date();
  let age = today.getFullYear() - dateOfBirth.getFullYear();
  const monthDiff = today.getMonth() - dateOfBirth.getMonth();

  if (monthDiff < 0 || (monthDiff === 0 && today.getDate() < dateOfBirth.getDate())) {
    age--;
  }

  return age;
}

export default ElderCareSDK;
