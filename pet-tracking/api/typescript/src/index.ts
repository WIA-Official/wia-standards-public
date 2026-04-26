/**
 * WIA-PET-008 Pet Tracking SDK
 * Official TypeScript implementation of the WIA Pet Tracking Standard
 *
 * @module @wia/pet-tracking-sdk
 * @author WIA (World Certification Industry Association)
 * @license MIT
 */

import axios, { AxiosInstance } from 'axios';
import WebSocket from 'ws';
import {
  SDKConfig,
  LocationEvent,
  Geofence,
  Alert,
  Pet,
  Device,
  ActivitySummary,
  LocationHistoryQuery,
  LostPetReport,
} from './types';

export * from './types';

/**
 * Main SDK client for WIA-PET-008 Pet Tracking
 */
export class PetTrackingClient {
  private http: AxiosInstance;
  private ws?: WebSocket;
  private config: SDKConfig;

  /**
   * Create a new Pet Tracking client
   * @param config SDK configuration
   */
  constructor(config: SDKConfig) {
    this.config = {
      timeout: 30000,
      debug: false,
      ...config,
    };

    this.http = axios.create({
      baseURL: this.config.apiUrl,
      timeout: this.config.timeout,
      headers: {
        'Authorization': `Bearer ${this.config.apiKey}`,
        'Content-Type': 'application/json',
        'X-WIA-Standard': 'PET-008',
        'X-SDK-Version': '1.0.0',
      },
    });

    if (this.config.debug) {
      this.http.interceptors.request.use(request => {
        console.log('[WIA-PET-008] Request:', request.method?.toUpperCase(), request.url);
        return request;
      });
      this.http.interceptors.response.use(response => {
        console.log('[WIA-PET-008] Response:', response.status, response.config.url);
        return response;
      });
    }
  }

  // ==================== Pet Management ====================

  /**
   * Get pet by ID
   */
  async getPet(petId: string): Promise<Pet> {
    const response = await this.http.get(`/v1/pets/${petId}`);
    return response.data;
  }

  /**
   * List all pets for authenticated user
   */
  async listPets(): Promise<Pet[]> {
    const response = await this.http.get('/v1/pets');
    return response.data;
  }

  /**
   * Create a new pet profile
   */
  async createPet(pet: Omit<Pet, 'id'>): Promise<Pet> {
    const response = await this.http.post('/v1/pets', pet);
    return response.data;
  }

  /**
   * Update pet profile
   */
  async updatePet(petId: string, updates: Partial<Pet>): Promise<Pet> {
    const response = await this.http.put(`/v1/pets/${petId}`, updates);
    return response.data;
  }

  /**
   * Delete pet profile
   */
  async deletePet(petId: string): Promise<void> {
    await this.http.delete(`/v1/pets/${petId}`);
  }

  // ==================== Location Tracking ====================

  /**
   * Get current location for a pet
   */
  async getCurrentLocation(petId: string): Promise<LocationEvent> {
    const response = await this.http.get(`/v1/pets/${petId}/location`);
    return response.data;
  }

  /**
   * Get location history
   */
  async getLocationHistory(query: LocationHistoryQuery): Promise<LocationEvent[]> {
    const response = await this.http.get(`/v1/pets/${query.petId}/history`, {
      params: {
        startTime: query.startTime,
        endTime: query.endTime,
        limit: query.limit || 100,
        offset: query.offset || 0,
      },
    });
    return response.data;
  }

  /**
   * Subscribe to real-time location updates
   */
  subscribeToLocation(petId: string, callback: (event: LocationEvent) => void): () => void {
    if (!this.config.wsUrl) {
      throw new Error('WebSocket URL not configured');
    }

    this.ws = new WebSocket(this.config.wsUrl);

    this.ws.on('open', () => {
      this.ws?.send(JSON.stringify({
        type: 'subscribe',
        petId,
        events: ['location'],
      }));
      if (this.config.debug) {
        console.log('[WIA-PET-008] WebSocket connected, subscribed to pet:', petId);
      }
    });

    this.ws.on('message', (data: string) => {
      const event = JSON.parse(data);
      if (event.type === 'location_update') {
        callback(event.data);
      }
    });

    this.ws.on('error', (error) => {
      console.error('[WIA-PET-008] WebSocket error:', error);
    });

    // Return unsubscribe function
    return () => {
      this.ws?.close();
      this.ws = undefined;
    };
  }

  // ==================== Geofencing ====================

  /**
   * Create a geofence
   */
  async createGeofence(geofence: Omit<Geofence, 'id'>): Promise<Geofence> {
    const response = await this.http.post('/v1/geofences', geofence);
    return response.data;
  }

  /**
   * List all geofences
   */
  async listGeofences(): Promise<Geofence[]> {
    const response = await this.http.get('/v1/geofences');
    return response.data;
  }

  /**
   * Get geofence by ID
   */
  async getGeofence(geofenceId: string): Promise<Geofence> {
    const response = await this.http.get(`/v1/geofences/${geofenceId}`);
    return response.data;
  }

  /**
   * Update geofence
   */
  async updateGeofence(geofenceId: string, updates: Partial<Geofence>): Promise<Geofence> {
    const response = await this.http.put(`/v1/geofences/${geofenceId}`, updates);
    return response.data;
  }

  /**
   * Delete geofence
   */
  async deleteGeofence(geofenceId: string): Promise<void> {
    await this.http.delete(`/v1/geofences/${geofenceId}`);
  }

  // ==================== Alerts ====================

  /**
   * Get alerts for a pet
   */
  async getAlerts(petId: string, limit = 50): Promise<Alert[]> {
    const response = await this.http.get(`/v1/pets/${petId}/alerts`, {
      params: { limit },
    });
    return response.data;
  }

  /**
   * Acknowledge an alert
   */
  async acknowledgeAlert(alertId: string): Promise<void> {
    await this.http.post(`/v1/alerts/${alertId}/acknowledge`);
  }

  /**
   * Subscribe to real-time alerts
   */
  subscribeToAlerts(petId: string, callback: (alert: Alert) => void): () => void {
    if (!this.config.wsUrl) {
      throw new Error('WebSocket URL not configured');
    }

    this.ws = new WebSocket(this.config.wsUrl);

    this.ws.on('open', () => {
      this.ws?.send(JSON.stringify({
        type: 'subscribe',
        petId,
        events: ['alerts'],
      }));
    });

    this.ws.on('message', (data: string) => {
      const event = JSON.parse(data);
      if (event.type === 'alert') {
        callback(event.data);
      }
    });

    return () => {
      this.ws?.close();
      this.ws = undefined;
    };
  }

  // ==================== Device Management ====================

  /**
   * Get device information
   */
  async getDevice(deviceId: string): Promise<Device> {
    const response = await this.http.get(`/v1/devices/${deviceId}`);
    return response.data;
  }

  /**
   * Update device tracking mode
   */
  async setTrackingMode(deviceId: string, mode: Device['mode']): Promise<Device> {
    const response = await this.http.put(`/v1/devices/${deviceId}/mode`, { mode });
    return response.data;
  }

  /**
   * Activate lost mode for a pet
   */
  async activateLostMode(petId: string): Promise<void> {
    await this.http.post(`/v1/pets/${petId}/lost-mode`);
  }

  /**
   * Deactivate lost mode
   */
  async deactivateLostMode(petId: string): Promise<void> {
    await this.http.delete(`/v1/pets/${petId}/lost-mode`);
  }

  // ==================== Activity Tracking ====================

  /**
   * Get activity summary for a pet
   */
  async getActivitySummary(
    petId: string,
    startTime: string,
    endTime: string
  ): Promise<ActivitySummary> {
    const response = await this.http.get(`/v1/pets/${petId}/activity`, {
      params: { startTime, endTime },
    });
    return response.data;
  }

  // ==================== Lost Pet Recovery ====================

  /**
   * Report a pet as lost
   */
  async reportLostPet(report: Omit<LostPetReport, 'id' | 'status'>): Promise<LostPetReport> {
    const response = await this.http.post('/v1/lost-pets', report);
    return response.data;
  }

  /**
   * Update lost pet report
   */
  async updateLostPetReport(reportId: string, status: LostPetReport['status']): Promise<LostPetReport> {
    const response = await this.http.put(`/v1/lost-pets/${reportId}`, { status });
    return response.data;
  }

  /**
   * Get nearby lost pets (community feature)
   */
  async getNearbyLostPets(latitude: number, longitude: number, radiusMeters: number): Promise<LostPetReport[]> {
    const response = await this.http.get('/v1/lost-pets/nearby', {
      params: { latitude, longitude, radius: radiusMeters },
    });
    return response.data;
  }

  /**
   * Close WebSocket connection and cleanup
   */
  disconnect(): void {
    this.ws?.close();
    this.ws = undefined;
  }
}

/**
 * Create a new Pet Tracking client (convenience factory function)
 */
export function createClient(config: SDKConfig): PetTrackingClient {
  return new PetTrackingClient(config);
}

export default PetTrackingClient;
