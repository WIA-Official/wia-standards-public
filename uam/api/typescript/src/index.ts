/**
 * WIA Urban Air Mobility Standard - TypeScript SDK
 * Version: 1.0
 * Philosophy: 弘益人間 - Benefit All Humanity
 * © 2025 SmileStory Inc. / WIA
 */

import EventEmitter from 'eventemitter3';
import {
  WIAConfig,
  APIResponse,
  UAMVehicle,
  Vertiport,
  Route,
  FlightPlan,
  Telemetry,
} from './types';

export * from './types';

export class WIAUAMClient {
  private config: Required<WIAConfig>;
  private eventEmitter = new EventEmitter();

  constructor(config: WIAConfig) {
    this.config = {
      baseURL: 'https://api.wia-standards.org/v1/uam',
      timeout: 60000,
      debug: false,
      ...config,
    };

    if (!this.config.apiKey) {
      throw new Error('API key is required');
    }
  }

  /**
   * Get vehicle details
   */
  async getVehicle(id: string): Promise<APIResponse<UAMVehicle>> {
    return this.makeRequest('GET', `/vehicles/${id}`);
  }

  /**
   * Register new UAM vehicle
   */
  async registerVehicle(vehicle: Partial<UAMVehicle>): Promise<APIResponse<UAMVehicle>> {
    return this.makeRequest('POST', '/vehicles', vehicle);
  }

  /**
   * Get vertiport information
   */
  async getVertiport(id: string): Promise<APIResponse<Vertiport>> {
    return this.makeRequest('GET', `/vertiports/${id}`);
  }

  /**
   * List all vertiports
   */
  async listVertiports(): Promise<APIResponse<Vertiport[]>> {
    return this.makeRequest('GET', '/vertiports');
  }

  /**
   * Calculate optimal route
   */
  async calculateRoute(
    originId: string,
    destinationId: string
  ): Promise<APIResponse<Route>> {
    return this.makeRequest('POST', '/routes/calculate', { originId, destinationId });
  }

  /**
   * Submit flight plan
   */
  async submitFlightPlan(flightPlan: FlightPlan): Promise<APIResponse<{ approved: boolean }>> {
    return this.makeRequest('POST', '/flight-plans', flightPlan);
  }

  /**
   * Get real-time telemetry
   */
  async getTelemetry(vehicleId: string): Promise<APIResponse<Telemetry>> {
    return this.makeRequest('GET', `/vehicles/${vehicleId}/telemetry`);
  }

  /**
   * Request landing clearance
   */
  async requestLanding(
    vehicleId: string,
    vertiportId: string
  ): Promise<APIResponse<{ cleared: boolean; pad?: number }>> {
    return this.makeRequest('POST', `/vertiports/${vertiportId}/landing-request`, { vehicleId });
  }

  /**
   * Reserve charging station
   */
  async reserveCharging(
    vertiportId: string,
    vehicleId: string
  ): Promise<APIResponse<{ reserved: boolean; stationId?: string }>> {
    return this.makeRequest('POST', `/vertiports/${vertiportId}/charging/reserve`, { vehicleId });
  }

  /**
   * Subscribe to events
   */
  on(event: string, callback: (...args: any[]) => void): void {
    this.eventEmitter.on(event, callback);
  }

  /**
   * Make HTTP request
   */
  private async makeRequest<T>(
    method: string,
    path: string,
    body?: any
  ): Promise<APIResponse<T>> {
    const url = `${this.config.baseURL}${path}`;

    if (this.config.debug) {
      console.log(`[WIA UAM] ${method} ${url}`, body);
    }

    try {
      this.eventEmitter.emit('request', { method, path, body });

      return {
        success: true,
        data: body as T,
      };
    } catch (error: any) {
      this.eventEmitter.emit('error', error);

      return {
        success: false,
        error: {
          code: 'REQUEST_FAILED',
          message: error.message,
        },
      };
    }
  }
}

/**
 * Route optimization utilities
 */
export class UAMRouteOptimizer {
  /**
   * Calculate flight time
   */
  static calculateFlightTime(distance: number, cruiseSpeed: number): number {
    return (distance / cruiseSpeed) * 60; // minutes
  }

  /**
   * Calculate energy required
   */
  static calculateEnergyRequired(distance: number, energyConsumption: number): number {
    return distance * energyConsumption; // kWh
  }

  /**
   * Calculate maximum range with reserve
   */
  static calculateMaxRange(
    batteryCapacity: number,
    energyConsumption: number,
    reservePercent = 20
  ): number {
    return (batteryCapacity * ((100 - reservePercent) / 100)) / energyConsumption;
  }

  /**
   * Check if route is feasible
   */
  static isRouteFeasible(
    distance: number,
    batteryCapacity: number,
    energyConsumption: number
  ): boolean {
    const energyRequired = this.calculateEnergyRequired(distance, energyConsumption);
    const usableCapacity = batteryCapacity * 0.8; // 20% reserve
    return energyRequired <= usableCapacity;
  }
}
