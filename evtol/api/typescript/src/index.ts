/**
 * WIA eVTOL Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @packageDocumentation
 * @module wia-evtol
 * @license MIT
 * @author WIA / SmileStory Inc.
 * @version 1.0.0
 */

import EventEmitter from 'eventemitter3';
import {
  WIAConfig,
  APIResponse,
  EVTOLAircraft,
  FlightPlan,
  BatteryStatus,
  MotorStatus,
  FlightSession,
  Vertiport,
  Telemetry,
  PaginatedResponse,
} from './types';

export * from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface EVTOLSDKConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  retries?: number;
  debug?: boolean;
}

// ============================================================================
// SDK Client
// ============================================================================

export class WIAEVTOLClient {
  private config: Required<EVTOLSDKConfig>;
  private headers: Record<string, string>;
  private eventEmitter = new EventEmitter();

  constructor(config: EVTOLSDKConfig) {
    this.config = {
      timeout: 60000,
      retries: 3,
      debug: false,
      ...config,
    };

    this.headers = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'X-WIA-Standard': 'EVTOL',
      'X-WIA-Version': '1.0.0',
    };
  }

  // ==========================================================================
  // Aircraft APIs
  // ==========================================================================

  async listAircraft(): Promise<APIResponse<EVTOLAircraft[]>> {
    return this.get<EVTOLAircraft[]>('/api/v1/aircraft');
  }

  async getAircraft(id: string): Promise<APIResponse<EVTOLAircraft>> {
    return this.get<EVTOLAircraft>(`/api/v1/aircraft/${id}`);
  }

  async registerAircraft(aircraft: Omit<EVTOLAircraft, 'aircraftId'>): Promise<APIResponse<EVTOLAircraft>> {
    return this.post<EVTOLAircraft>('/api/v1/aircraft', aircraft);
  }

  // ==========================================================================
  // Battery & Motor APIs
  // ==========================================================================

  async getBatteryStatus(aircraftId: string): Promise<APIResponse<BatteryStatus>> {
    return this.get<BatteryStatus>(`/api/v1/aircraft/${aircraftId}/battery`);
  }

  async getMotorStatus(aircraftId: string): Promise<APIResponse<MotorStatus[]>> {
    return this.get<MotorStatus[]>(`/api/v1/aircraft/${aircraftId}/motors`);
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

  async submitFlightPlan(planId: string): Promise<APIResponse<FlightPlan>> {
    return this.post<FlightPlan>(`/api/v1/flight-plans/${planId}/submit`, {});
  }

  // ==========================================================================
  // Flight Session APIs
  // ==========================================================================

  async startFlight(aircraftId: string, planId: string): Promise<APIResponse<FlightSession>> {
    return this.post<FlightSession>('/api/v1/flights', { aircraftId, planId });
  }

  async getFlightSession(sessionId: string): Promise<APIResponse<FlightSession>> {
    return this.get<FlightSession>(`/api/v1/flights/${sessionId}`);
  }

  async endFlight(sessionId: string): Promise<APIResponse<FlightSession>> {
    return this.post<FlightSession>(`/api/v1/flights/${sessionId}/end`, {});
  }

  async getFlightTelemetry(sessionId: string): Promise<APIResponse<Telemetry[]>> {
    return this.get<Telemetry[]>(`/api/v1/flights/${sessionId}/telemetry`);
  }

  // ==========================================================================
  // Vertiport APIs
  // ==========================================================================

  async listVertiports(): Promise<APIResponse<Vertiport[]>> {
    return this.get<Vertiport[]>('/api/v1/vertiports');
  }

  async getVertiport(vertiportId: string): Promise<APIResponse<Vertiport>> {
    return this.get<Vertiport>(`/api/v1/vertiports/${vertiportId}`);
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

  private async request<T>(method: string, path: string, data?: unknown): Promise<APIResponse<T>> {
    const url = `${this.config.endpoint}${path}`;
    if (this.config.debug) {
      console.log(`[WIA eVTOL] ${method} ${url}`);
    }
    const response = await fetch(url, {
      method,
      headers: this.headers,
      body: data ? JSON.stringify(data) : undefined,
    });
    return response.json();
  }
}

// ============================================================================
// Performance Calculator
// ============================================================================

export class EVTOLPerformanceCalculator {
  static calculateHoverPower(mtow: number, motorEff: number, diskLoading = 50): number {
    const g = 9.81;
    const weight = mtow * g;
    const figureOfMerit = 0.75;
    return (Math.pow(weight, 1.5) / Math.sqrt(2 * 1.225 * diskLoading)) / (1000 * motorEff * figureOfMerit);
  }

  static calculateEndurance(batteryCapacity: number, hoverPower: number): number {
    return (batteryCapacity / hoverPower) * 60;
  }

  static calculateRange(batteryCapacity: number, cruisePower: number, cruiseSpeed: number): number {
    const enduranceHours = batteryCapacity / cruisePower;
    return enduranceHours * cruiseSpeed;
  }
}

// ============================================================================
// Constants
// ============================================================================

export const VERSION = '1.0.0';

export const BATTERY_THRESHOLDS = {
  CRITICAL: 10,
  LOW: 20,
  RESERVE: 30,
} as const;

export default WIAEVTOLClient;
