/**
 * WIA-PET-007 Pet Wearable Standard - TypeScript SDK
 * @version 1.0.0
 * @standard WIA-PET-007
 * @license MIT
 */

import axios, { AxiosInstance } from 'axios';
import * as Types from './types';

export * from './types';

/**
 * Main SDK client for WIA-PET-007 Pet Wearable Standard
 */
export class WIAPetWearableClient {
  private client: AxiosInstance;
  private config: Types.ClientConfig;

  constructor(config: Types.ClientConfig) {
    this.config = config;
    this.client = axios.create({
      baseURL: config.baseURL,
      timeout: config.timeout || 30000,
      headers: {
        'Content-Type': 'application/json',
        'X-WIA-Standard': 'WIA-PET-007',
        'X-WIA-Version': '1.0.0',
        ...(config.apiKey && { Authorization: `Bearer ${config.apiKey}` }),
      },
    });
  }

  // ========================================================================
  // Pet Profile Management
  // ========================================================================

  /**
   * Create a new pet profile
   */
  async createPetProfile(profile: Omit<Types.PetProfile, 'petId' | 'createdAt' | 'updatedAt'>): Promise<Types.PetProfile> {
    const response = await this.client.post('/api/v1/pets', profile);
    return response.data;
  }

  /**
   * Get pet profile by ID
   */
  async getPetProfile(petId: string): Promise<Types.PetProfile> {
    const response = await this.client.get(`/api/v1/pets/${petId}`);
    return response.data;
  }

  /**
   * Update pet profile
   */
  async updatePetProfile(petId: string, updates: Partial<Types.PetProfile>): Promise<Types.PetProfile> {
    const response = await this.client.put(`/api/v1/pets/${petId}`, updates);
    return response.data;
  }

  /**
   * Delete pet profile
   */
  async deletePetProfile(petId: string): Promise<void> {
    await this.client.delete(`/api/v1/pets/${petId}`);
  }

  // ========================================================================
  // Activity Data
  // ========================================================================

  /**
   * Record activity data
   */
  async recordActivity(activity: Types.ActivityData): Promise<void> {
    await this.client.post('/api/v1/activities', activity);
  }

  /**
   * Get activity history
   */
  async getActivities(
    petId: string,
    options?: { startDate?: string; endDate?: string; activityType?: Types.ActivityType }
  ): Promise<Types.ActivityData[]> {
    const response = await this.client.get(`/api/v1/pets/${petId}/activities`, {
      params: options,
    });
    return response.data;
  }

  /**
   * Get daily activity summary
   */
  async getDailyActivitySummary(petId: string, date: string): Promise<Types.DailyActivitySummary> {
    const response = await this.client.get(`/api/v1/pets/${petId}/activities/daily/${date}`);
    return response.data;
  }

  // ========================================================================
  // Health Metrics
  // ========================================================================

  /**
   * Record health metrics
   */
  async recordHealthMetrics(metrics: Types.HealthMetrics): Promise<void> {
    await this.client.post('/api/v1/health', metrics);
  }

  /**
   * Get health history
   */
  async getHealthMetrics(
    petId: string,
    options?: { startDate?: string; endDate?: string }
  ): Promise<Types.HealthMetrics[]> {
    const response = await this.client.get(`/api/v1/pets/${petId}/health`, {
      params: options,
    });
    return response.data;
  }

  /**
   * Get current vital signs
   */
  async getCurrentVitalSigns(petId: string): Promise<Types.VitalSigns> {
    const response = await this.client.get(`/api/v1/pets/${petId}/health/current`);
    return response.data.vitalSigns;
  }

  // ========================================================================
  // Location & GPS
  // ========================================================================

  /**
   * Get current location
   */
  async getCurrentLocation(petId: string): Promise<Types.LocationUpdate> {
    const response = await this.client.get(`/api/v1/pets/${petId}/location/current`);
    return response.data;
  }

  /**
   * Get location history
   */
  async getLocationHistory(
    petId: string,
    options?: { startDate?: string; endDate?: string }
  ): Promise<Types.LocationUpdate[]> {
    const response = await this.client.get(`/api/v1/pets/${petId}/location/history`, {
      params: options,
    });
    return response.data;
  }

  /**
   * Create geofence
   */
  async createGeofence(petId: string, geofence: Omit<Types.Geofence, 'geofenceId'>): Promise<Types.Geofence> {
    const response = await this.client.post(`/api/v1/pets/${petId}/geofences`, geofence);
    return response.data;
  }

  /**
   * Get all geofences for a pet
   */
  async getGeofences(petId: string): Promise<Types.Geofence[]> {
    const response = await this.client.get(`/api/v1/pets/${petId}/geofences`);
    return response.data;
  }

  /**
   * Update geofence
   */
  async updateGeofence(petId: string, geofenceId: string, updates: Partial<Types.Geofence>): Promise<Types.Geofence> {
    const response = await this.client.put(`/api/v1/pets/${petId}/geofences/${geofenceId}`, updates);
    return response.data;
  }

  /**
   * Delete geofence
   */
  async deleteGeofence(petId: string, geofenceId: string): Promise<void> {
    await this.client.delete(`/api/v1/pets/${petId}/geofences/${geofenceId}`);
  }

  /**
   * Check if pet is in geofence
   */
  async checkGeofence(petId: string, location: Types.LocationData): Promise<Types.SafeZoneStatus[]> {
    const response = await this.client.post(`/api/v1/pets/${petId}/geofences/check`, location);
    return response.data;
  }

  // ========================================================================
  // Sleep Data
  // ========================================================================

  /**
   * Get sleep data
   */
  async getSleepData(petId: string, date: string): Promise<Types.SleepData> {
    const response = await this.client.get(`/api/v1/pets/${petId}/sleep/${date}`);
    return response.data;
  }

  /**
   * Get sleep history
   */
  async getSleepHistory(
    petId: string,
    options?: { startDate?: string; endDate?: string }
  ): Promise<Types.SleepData[]> {
    const response = await this.client.get(`/api/v1/pets/${petId}/sleep`, {
      params: options,
    });
    return response.data;
  }

  // ========================================================================
  // Behavioral Data
  // ========================================================================

  /**
   * Get behavior data
   */
  async getBehaviorData(petId: string, date: string): Promise<Types.BehaviorData> {
    const response = await this.client.get(`/api/v1/pets/${petId}/behavior/${date}`);
    return response.data;
  }

  // ========================================================================
  // Alerts
  // ========================================================================

  /**
   * Get active alerts
   */
  async getActiveAlerts(petId: string): Promise<Types.HealthAlert[]> {
    const response = await this.client.get(`/api/v1/pets/${petId}/alerts/active`);
    return response.data;
  }

  /**
   * Acknowledge alert
   */
  async acknowledgeAlert(petId: string, alertId: string): Promise<void> {
    await this.client.post(`/api/v1/pets/${petId}/alerts/${alertId}/acknowledge`);
  }

  // ========================================================================
  // Device Management
  // ========================================================================

  /**
   * Pair device with pet
   */
  async pairDevice(petId: string, deviceId: string): Promise<void> {
    await this.client.post(`/api/v1/pets/${petId}/devices`, { deviceId });
  }

  /**
   * Get device status
   */
  async getDeviceStatus(deviceId: string): Promise<any> {
    const response = await this.client.get(`/api/v1/devices/${deviceId}/status`);
    return response.data;
  }

  /**
   * Update device settings
   */
  async updateDeviceSettings(deviceId: string, settings: Record<string, any>): Promise<void> {
    await this.client.put(`/api/v1/devices/${deviceId}/settings`, settings);
  }
}

/**
 * Factory function to create WIA Pet Wearable client
 */
export function createClient(config: Types.ClientConfig): WIAPetWearableClient {
  return new WIAPetWearableClient(config);
}

/**
 * Default export
 */
export default {
  createClient,
  WIAPetWearableClient,
};
