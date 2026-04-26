/**
 * WIA-MED-013: Elderly Care Device Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @packageDocumentation
 * @module wia-elderly-care-device
 * @license MIT
 * @author WIA / SmileStory Inc.
 * @version 1.0.0
 */

import axios, { AxiosInstance } from 'axios';
import EventEmitter from 'eventemitter3';
import type * as Types from './types';

export * from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface ElderlyDeviceConfig {
  apiKey: string;
  endpoint: string;
  environment?: 'production' | 'staging' | 'development';
  timeout?: number;
  retries?: number;
  debug?: boolean;
}

// ============================================================================
// SDK Client
// ============================================================================

export class ElderlyDeviceSDK extends EventEmitter {
  private api: AxiosInstance;
  private config: Required<ElderlyDeviceConfig>;

  public devices: DeviceService;
  public falls: FallDetectionService;
  public activity: ActivityService;
  public medication: MedicationService;
  public emergency: EmergencyService;
  public social: SocialService;
  public caregivers: CaregiverService;
  public analytics: AnalyticsService;

  constructor(config: ElderlyDeviceConfig) {
    super();
    this.config = {
      environment: 'production',
      timeout: 30000,
      retries: 3,
      debug: false,
      ...config,
    };

    const baseUrl = config.endpoint || this.getDefaultBaseUrl(this.config.environment);

    this.api = axios.create({
      baseURL: baseUrl,
      timeout: this.config.timeout,
      headers: {
        'Authorization': `Bearer ${this.config.apiKey}`,
        'Content-Type': 'application/json',
        'X-WIA-Standard': 'MED-013',
        'X-WIA-Version': '1.0.0',
      },
    });

    this.devices = new DeviceService(this.api);
    this.falls = new FallDetectionService(this.api);
    this.activity = new ActivityService(this.api);
    this.medication = new MedicationService(this.api);
    this.emergency = new EmergencyService(this.api);
    this.social = new SocialService(this.api);
    this.caregivers = new CaregiverService(this.api);
    this.analytics = new AnalyticsService(this.api);
  }

  private getDefaultBaseUrl(environment: string): string {
    const urls = {
      production: 'https://api.wia.org/elderly-device/v1',
      staging: 'https://staging-api.wia.org/elderly-device/v1',
      development: 'http://localhost:3000/api/v1',
    };
    return urls[environment as keyof typeof urls] || urls.production;
  }
}

// ============================================================================
// Device Service
// ============================================================================

class DeviceService {
  constructor(private api: AxiosInstance) {}

  async register(request: Types.RegisterDeviceRequest): Promise<Types.RegisterDeviceResponse> {
    const response = await this.api.post('/devices/register', request);
    return response.data;
  }

  async get(deviceId: string): Promise<Types.ElderlyDevice> {
    const response = await this.api.get(`/devices/${deviceId}`);
    return response.data;
  }

  async list(userId: string): Promise<Types.ElderlyDevice[]> {
    const response = await this.api.get('/devices', { params: { userId } });
    return response.data;
  }

  async updateConfig(deviceId: string, config: Partial<Types.DeviceConfig>): Promise<Types.ElderlyDevice> {
    const response = await this.api.patch(`/devices/${deviceId}/config`, config);
    return response.data;
  }

  async deactivate(deviceId: string): Promise<void> {
    await this.api.post(`/devices/${deviceId}/deactivate`);
  }
}

// ============================================================================
// Fall Detection Service
// ============================================================================

class FallDetectionService {
  constructor(private api: AxiosInstance) {}

  async report(request: Types.ReportFallRequest): Promise<Types.ReportFallResponse> {
    const response = await this.api.post('/falls/report', request);
    return response.data;
  }

  async get(fallId: string): Promise<Types.FallEvent> {
    const response = await this.api.get(`/falls/${fallId}`);
    return response.data;
  }

  async list(userId: string, period?: string): Promise<Types.FallEvent[]> {
    const response = await this.api.get('/falls', { params: { userId, period } });
    return response.data;
  }

  async markFalseAlarm(fallId: string): Promise<void> {
    await this.api.patch(`/falls/${fallId}/false-alarm`);
  }

  async getStatistics(userId: string): Promise<Types.FallStatistics> {
    const response = await this.api.get(`/falls/statistics/${userId}`);
    return response.data;
  }
}

// ============================================================================
// Activity Service
// ============================================================================

class ActivityService {
  constructor(private api: AxiosInstance) {}

  async getActivity(request: Types.GetActivityRequest): Promise<Types.GetActivityResponse> {
    const response = await this.api.get('/activity', { params: request });
    return response.data;
  }

  async recordVitals(userId: string, vitals: Types.VitalSigns): Promise<void> {
    await this.api.post(`/activity/${userId}/vitals`, vitals);
  }

  async getSleepData(userId: string, date: string): Promise<Types.SleepData> {
    const response = await this.api.get(`/activity/${userId}/sleep`, { params: { date } });
    return response.data;
  }

  async getMovementPatterns(userId: string): Promise<Types.MovementPattern[]> {
    const response = await this.api.get(`/activity/${userId}/movement`);
    return response.data;
  }
}

// ============================================================================
// Medication Service
// ============================================================================

class MedicationService {
  constructor(private api: AxiosInstance) {}

  async updateDose(request: Types.UpdateMedicationRequest): Promise<Types.UpdateMedicationResponse> {
    const response = await this.api.post('/medication/dose', request);
    return response.data;
  }

  async getSchedule(userId: string): Promise<Types.MedicationSchedule[]> {
    const response = await this.api.get(`/medication/${userId}/schedule`);
    return response.data;
  }

  async recordAdherence(userId: string, medicationId: string, taken: boolean): Promise<void> {
    await this.api.post(`/medication/${userId}/adherence`, { medicationId, taken });
  }

  async getAdherenceReport(userId: string, period: string): Promise<Types.AdherenceReport> {
    const response = await this.api.get(`/medication/${userId}/adherence`, { params: { period } });
    return response.data;
  }
}

// ============================================================================
// Emergency Service
// ============================================================================

class EmergencyService {
  constructor(private api: AxiosInstance) {}

  async trigger(alert: Omit<Types.EmergencyAlert, 'id'>): Promise<Types.EmergencyAlert> {
    const response = await this.api.post('/emergency/alert', alert);
    return response.data;
  }

  async acknowledge(alertId: string, acknowledgedBy: string): Promise<void> {
    await this.api.patch(`/emergency/${alertId}/acknowledge`, { acknowledgedBy });
  }

  async resolve(alertId: string, resolution: string): Promise<void> {
    await this.api.patch(`/emergency/${alertId}/resolve`, { resolution });
  }

  async getHistory(userId: string): Promise<Types.EmergencyAlert[]> {
    const response = await this.api.get(`/emergency/${userId}/history`);
    return response.data;
  }
}

// ============================================================================
// Social Service
// ============================================================================

class SocialService {
  constructor(private api: AxiosInstance) {}

  async recordInteraction(interaction: Omit<Types.SocialInteraction, 'id'>): Promise<Types.SocialInteraction> {
    const response = await this.api.post('/social/interaction', interaction);
    return response.data;
  }

  async getMetrics(userId: string, period: string): Promise<Types.SocialMetrics> {
    const response = await this.api.get(`/social/${userId}/metrics`, { params: { period } });
    return response.data;
  }

  async getIsolationRisk(userId: string): Promise<Types.IsolationRisk> {
    const response = await this.api.get(`/social/${userId}/isolation-risk`);
    return response.data;
  }
}

// ============================================================================
// Caregiver Service
// ============================================================================

class CaregiverService {
  constructor(private api: AxiosInstance) {}

  async assign(userId: string, caregiverId: string): Promise<void> {
    await this.api.post(`/caregivers/${userId}/assign`, { caregiverId });
  }

  async list(userId: string): Promise<Types.Caregiver[]> {
    const response = await this.api.get(`/caregivers/${userId}`);
    return response.data;
  }

  async notify(caregiverId: string, notification: Types.CaregiverNotification): Promise<void> {
    await this.api.post(`/caregivers/${caregiverId}/notify`, notification);
  }
}

// ============================================================================
// Analytics Service
// ============================================================================

class AnalyticsService {
  constructor(private api: AxiosInstance) {}

  async getWellnessScore(userId: string): Promise<Types.WellnessScore> {
    const response = await this.api.get(`/analytics/${userId}/wellness`);
    return response.data;
  }

  async getTrends(userId: string, metrics: string[]): Promise<Types.TrendData[]> {
    const response = await this.api.get(`/analytics/${userId}/trends`, { params: { metrics } });
    return response.data;
  }
}

// ============================================================================
// Factory Function
// ============================================================================

export function createClient(config: ElderlyDeviceConfig): ElderlyDeviceSDK {
  return new ElderlyDeviceSDK(config);
}

// ============================================================================
// Constants
// ============================================================================

export const VERSION = '1.0.0';

export const ALERT_TYPES = {
  FALL: 'fall',
  MEDICATION_MISSED: 'medication_missed',
  VITAL_ABNORMAL: 'vital_abnormal',
  INACTIVITY: 'inactivity',
  SOS: 'sos',
} as const;

export const DEVICE_TYPES = {
  WEARABLE: 'wearable',
  PENDANT: 'pendant',
  HOME_SENSOR: 'home_sensor',
  MEDICATION_DISPENSER: 'medication_dispenser',
} as const;

export default ElderlyDeviceSDK;
