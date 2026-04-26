/**
 * WIA-TRANS-015: Air Traffic Management Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @packageDocumentation
 * @module wia-air-traffic-management
 * @license MIT
 * @author WIA / SmileStory Inc.
 * @version 1.0.0
 */

import EventEmitter from 'eventemitter3';
import {
  WIAConfig,
  APIResponse,
  Aircraft,
  FlightPlan,
  AirspaceStatus,
  Waypoint,
  ConflictAlert,
  WeatherData,
  NOTAMInfo,
  Runway,
  ControllerSession,
  TrafficFlow,
  PaginatedResponse,
} from './types';

export * from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface ATMConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  retries?: number;
  debug?: boolean;
}

// ============================================================================
// SDK Client
// ============================================================================

export class AirTrafficManagementClient {
  private config: Required<ATMConfig>;
  private headers: Record<string, string>;
  private eventEmitter = new EventEmitter();

  constructor(config: ATMConfig) {
    this.config = {
      timeout: 60000,
      retries: 3,
      debug: false,
      ...config,
    };

    this.headers = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'X-WIA-Standard': 'TRANS-015',
      'X-WIA-Version': '1.0.0',
    };
  }

  // ==========================================================================
  // Aircraft APIs
  // ==========================================================================

  async getAircraft(aircraftId: string): Promise<APIResponse<Aircraft>> {
    return this.get<Aircraft>(`/api/v1/aircraft/${aircraftId}`);
  }

  async listAircraft(query?: { sector?: string; status?: string }): Promise<APIResponse<Aircraft[]>> {
    const queryString = this.buildQueryParams(query);
    return this.get<Aircraft[]>(`/api/v1/aircraft${queryString}`);
  }

  async updateAircraftPosition(aircraftId: string, position: { lat: number; lon: number; alt: number }): Promise<APIResponse<Aircraft>> {
    return this.put<Aircraft>(`/api/v1/aircraft/${aircraftId}/position`, position);
  }

  async getAircraftTrack(aircraftId: string): Promise<APIResponse<Array<{ lat: number; lon: number; alt: number; timestamp: string }>>> {
    return this.get<Array<{ lat: number; lon: number; alt: number; timestamp: string }>>(`/api/v1/aircraft/${aircraftId}/track`);
  }

  // ==========================================================================
  // Flight Plan APIs
  // ==========================================================================

  async createFlightPlan(plan: Omit<FlightPlan, 'planId'>): Promise<APIResponse<FlightPlan>> {
    return this.post<FlightPlan>('/api/v1/flight-plans', plan);
  }

  async getFlightPlan(planId: string): Promise<APIResponse<FlightPlan>> {
    return this.get<FlightPlan>(`/api/v1/flight-plans/${planId}`);
  }

  async updateFlightPlan(planId: string, updates: Partial<FlightPlan>): Promise<APIResponse<FlightPlan>> {
    return this.put<FlightPlan>(`/api/v1/flight-plans/${planId}`, updates);
  }

  async activateFlightPlan(planId: string): Promise<APIResponse<FlightPlan>> {
    return this.post<FlightPlan>(`/api/v1/flight-plans/${planId}/activate`, {});
  }

  async cancelFlightPlan(planId: string, reason: string): Promise<APIResponse<FlightPlan>> {
    return this.post<FlightPlan>(`/api/v1/flight-plans/${planId}/cancel`, { reason });
  }

  // ==========================================================================
  // Airspace APIs
  // ==========================================================================

  async getAirspaceStatus(sectorId: string): Promise<APIResponse<AirspaceStatus>> {
    return this.get<AirspaceStatus>(`/api/v1/airspace/${sectorId}`);
  }

  async listSectors(): Promise<APIResponse<AirspaceStatus[]>> {
    return this.get<AirspaceStatus[]>('/api/v1/airspace/sectors');
  }

  async getTrafficFlow(sectorId: string): Promise<APIResponse<TrafficFlow>> {
    return this.get<TrafficFlow>(`/api/v1/airspace/${sectorId}/traffic`);
  }

  // ==========================================================================
  // Conflict Detection APIs
  // ==========================================================================

  async detectConflicts(aircraftId: string): Promise<APIResponse<ConflictAlert[]>> {
    return this.get<ConflictAlert[]>(`/api/v1/aircraft/${aircraftId}/conflicts`);
  }

  async getSectorConflicts(sectorId: string): Promise<APIResponse<ConflictAlert[]>> {
    return this.get<ConflictAlert[]>(`/api/v1/airspace/${sectorId}/conflicts`);
  }

  async acknowledgeConflict(conflictId: string): Promise<APIResponse<ConflictAlert>> {
    return this.post<ConflictAlert>(`/api/v1/conflicts/${conflictId}/acknowledge`, {});
  }

  async resolveConflict(conflictId: string, resolution: { action: string; details: string }): Promise<APIResponse<ConflictAlert>> {
    return this.post<ConflictAlert>(`/api/v1/conflicts/${conflictId}/resolve`, resolution);
  }

  // ==========================================================================
  // Weather APIs
  // ==========================================================================

  async getWeather(location: { lat: number; lon: number }): Promise<APIResponse<WeatherData>> {
    return this.get<WeatherData>(`/api/v1/weather?lat=${location.lat}&lon=${location.lon}`);
  }

  async getRouteWeather(waypoints: Waypoint[]): Promise<APIResponse<WeatherData[]>> {
    return this.post<WeatherData[]>('/api/v1/weather/route', { waypoints });
  }

  // ==========================================================================
  // NOTAM APIs
  // ==========================================================================

  async getNOTAMs(location: { lat: number; lon: number; radius: number }): Promise<APIResponse<NOTAMInfo[]>> {
    return this.get<NOTAMInfo[]>(`/api/v1/notams?lat=${location.lat}&lon=${location.lon}&radius=${location.radius}`);
  }

  async getRouteNOTAMs(planId: string): Promise<APIResponse<NOTAMInfo[]>> {
    return this.get<NOTAMInfo[]>(`/api/v1/flight-plans/${planId}/notams`);
  }

  // ==========================================================================
  // Runway APIs
  // ==========================================================================

  async getRunwayStatus(airportCode: string): Promise<APIResponse<Runway[]>> {
    return this.get<Runway[]>(`/api/v1/airports/${airportCode}/runways`);
  }

  async requestRunway(airportCode: string, aircraftId: string, operation: 'landing' | 'takeoff'): Promise<APIResponse<{ runway: string; slot: string }>> {
    return this.post<{ runway: string; slot: string }>(`/api/v1/airports/${airportCode}/runway-request`, { aircraftId, operation });
  }

  // ==========================================================================
  // Event Handling
  // ==========================================================================

  on(event: string, callback: (...args: unknown[]) => void): void {
    this.eventEmitter.on(event, callback);
  }

  off(event: string, callback: (...args: unknown[]) => void): void {
    this.eventEmitter.off(event, callback);
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
    if (this.config.debug) {
      console.log(`[WIA ATM] ${method} ${url}`);
    }

    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), this.config.timeout);

    try {
      const response = await fetch(url, {
        method,
        headers: this.headers,
        body: data ? JSON.stringify(data) : undefined,
        signal: controller.signal,
      });
      clearTimeout(timeoutId);
      return response.json();
    } catch (error) {
      clearTimeout(timeoutId);
      return {
        success: false,
        error: { code: 'REQUEST_FAILED', message: error instanceof Error ? error.message : 'Unknown error' },
      };
    }
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
// Factory Function
// ============================================================================

export function createClient(config: ATMConfig): AirTrafficManagementClient {
  return new AirTrafficManagementClient(config);
}

// ============================================================================
// Constants
// ============================================================================

export const VERSION = '1.0.0';

export const FLIGHT_RULES = {
  IFR: 'IFR',
  VFR: 'VFR',
  SVFR: 'SVFR',
} as const;

export const CONFLICT_SEVERITIES = {
  CRITICAL: 'critical',
  WARNING: 'warning',
  ADVISORY: 'advisory',
} as const;

export const SEPARATION_MINIMUMS = {
  RADAR: { horizontal: 3, vertical: 1000 },
  NON_RADAR: { horizontal: 5, vertical: 1000 },
  OCEANIC: { horizontal: 50, vertical: 2000 },
} as const;

export default AirTrafficManagementClient;
