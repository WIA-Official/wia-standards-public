/**
 * WIA-MED-017: Medical Drone Delivery Standard - TypeScript SDK
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
  MedicalDrone,
  Delivery,
  Cargo,
  DroneTelemetry,
  GeoLocation,
  DeliveryPriority,
  DeliveryStatus,
  DroneStatus,
} from './types';

export * from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface WIADroneDeliveryConfig extends WIAConfig {
  fleetId?: string;
}

// ============================================================================
// Main SDK Client
// ============================================================================

export class WIADroneDeliveryClient {
  private config: Required<WIADroneDeliveryConfig>;
  private eventEmitter = new EventEmitter();

  constructor(config: WIADroneDeliveryConfig) {
    this.config = {
      endpoint: 'https://api.wia-standards.org/v1/drone-delivery',
      timeout: 60000,
      debug: false,
      fleetId: '',
      ...config,
    };

    if (!this.config.apiKey) {
      throw new Error('API key is required');
    }
  }

  // ==========================================================================
  // Drone Operations
  // ==========================================================================

  async getDrone(droneId: string): Promise<APIResponse<MedicalDrone>> {
    return this.makeRequest('GET', `/drones/${droneId}`);
  }

  async listDrones(filters?: {
    status?: DroneStatus;
    available?: boolean;
  }): Promise<PaginatedResponse<MedicalDrone>> {
    const params = new URLSearchParams(filters as Record<string, string>);
    return this.makeRequest('GET', `/drones?${params}`);
  }

  async updateDroneStatus(droneId: string, status: DroneStatus): Promise<APIResponse<MedicalDrone>> {
    return this.makeRequest('PATCH', `/drones/${droneId}/status`, { status });
  }

  async getDroneTelemetry(droneId: string): Promise<APIResponse<DroneTelemetry>> {
    return this.makeRequest('GET', `/drones/${droneId}/telemetry`);
  }

  async getTelemetryHistory(droneId: string, startTime: string, endTime: string): Promise<APIResponse<DroneTelemetry[]>> {
    const params = new URLSearchParams({ startTime, endTime });
    return this.makeRequest('GET', `/drones/${droneId}/telemetry/history?${params}`);
  }

  // ==========================================================================
  // Delivery Operations
  // ==========================================================================

  async createDelivery(delivery: {
    priority: DeliveryPriority;
    cargo: Omit<Cargo, 'cargoId'>;
    origin: GeoLocation;
    destination: GeoLocation;
    scheduledTime?: string;
  }): Promise<APIResponse<Delivery>> {
    return this.makeRequest('POST', '/deliveries', delivery);
  }

  async getDelivery(deliveryId: string): Promise<APIResponse<Delivery>> {
    return this.makeRequest('GET', `/deliveries/${deliveryId}`);
  }

  async listDeliveries(filters?: {
    status?: DeliveryStatus;
    priority?: DeliveryPriority;
    droneId?: string;
    startDate?: string;
    endDate?: string;
  }): Promise<PaginatedResponse<Delivery>> {
    const params = new URLSearchParams(filters as Record<string, string>);
    return this.makeRequest('GET', `/deliveries?${params}`);
  }

  async cancelDelivery(deliveryId: string, reason: string): Promise<APIResponse<Delivery>> {
    return this.makeRequest('POST', `/deliveries/${deliveryId}/cancel`, { reason });
  }

  async updateDeliveryStatus(deliveryId: string, status: DeliveryStatus): Promise<APIResponse<Delivery>> {
    return this.makeRequest('PATCH', `/deliveries/${deliveryId}/status`, { status });
  }

  async getDeliveryETA(deliveryId: string): Promise<APIResponse<{
    estimatedArrival: string;
    remainingDistance: number;
    currentLocation: GeoLocation;
  }>> {
    return this.makeRequest('GET', `/deliveries/${deliveryId}/eta`);
  }

  // ==========================================================================
  // Route Operations
  // ==========================================================================

  async calculateRoute(origin: GeoLocation, destination: GeoLocation): Promise<APIResponse<{
    distance: number;
    estimatedTime: number;
    waypoints: GeoLocation[];
    weatherClear: boolean;
  }>> {
    return this.makeRequest('POST', '/routes/calculate', { origin, destination });
  }

  async getNoFlyZones(bounds: {
    north: number;
    south: number;
    east: number;
    west: number;
  }): Promise<APIResponse<Array<{
    zoneId: string;
    type: string;
    polygon: GeoLocation[];
    restriction: string;
  }>>> {
    const params = new URLSearchParams(bounds as unknown as Record<string, string>);
    return this.makeRequest('GET', `/routes/no-fly-zones?${params}`);
  }

  // ==========================================================================
  // Cargo Operations
  // ==========================================================================

  async validateCargo(cargo: Omit<Cargo, 'cargoId'>): Promise<APIResponse<{
    valid: boolean;
    issues: string[];
    suitableDrones: string[];
  }>> {
    return this.makeRequest('POST', '/cargo/validate', cargo);
  }

  async getCargoTemperature(deliveryId: string): Promise<APIResponse<{
    currentTemp: number;
    minTemp: number;
    maxTemp: number;
    history: Array<{ timestamp: string; temperature: number }>;
  }>> {
    return this.makeRequest('GET', `/deliveries/${deliveryId}/cargo/temperature`);
  }

  // ==========================================================================
  // Event Handling
  // ==========================================================================

  on(event: 'deliveryDispatched' | 'deliveryCompleted' | 'temperatureAlert' | 'droneError' | 'error', callback: (...args: unknown[]) => void): void {
    this.eventEmitter.on(event, callback);
  }

  // ==========================================================================
  // Private Methods
  // ==========================================================================

  private async makeRequest<T>(method: string, path: string, body?: unknown): Promise<T> {
    const url = `${this.config.endpoint}${path}`;

    if (this.config.debug) {
      console.log(`[WIA Drone Delivery] ${method} ${url}`, body);
    }

    try {
      const response = await fetch(url, {
        method,
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${this.config.apiKey}`,
          'X-WIA-Standard': 'MED-017',
          'X-WIA-Version': '1.0.0',
          ...(this.config.fleetId && { 'X-Fleet-ID': this.config.fleetId }),
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

export function createClient(config: WIADroneDeliveryConfig): WIADroneDeliveryClient {
  return new WIADroneDeliveryClient(config);
}

export default WIADroneDeliveryClient;
