/**
 * WIA-SPACE-010: Launch Vehicle Standard - TypeScript SDK
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
  LaunchVehicle,
  RocketStage,
  TelemetryData,
  LaunchMission,
  LaunchStatus,
} from './types';

export * from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface WIALaunchVehicleConfig extends WIAConfig {
  missionControlUrl?: string;
}

// ============================================================================
// Constants
// ============================================================================

const PHYSICS_CONSTANTS = {
  G0: 9.81, // Standard gravity (m/s²)
  EARTH_RADIUS_KM: 6371,
  LEO_ALTITUDE_KM: 400,
  GEO_ALTITUDE_KM: 35786,
};

// ============================================================================
// Main SDK Client
// ============================================================================

export class WIALaunchVehicleClient {
  private config: Required<WIALaunchVehicleConfig>;
  private eventEmitter = new EventEmitter();

  constructor(config: WIALaunchVehicleConfig) {
    this.config = {
      endpoint: 'https://api.wia-standards.org/v1/launch-vehicle',
      timeout: 60000,
      debug: false,
      missionControlUrl: 'wss://mission-control.wia-standards.org',
      ...config,
    };

    if (!this.config.apiKey) {
      throw new Error('API key is required');
    }
  }

  // ==========================================================================
  // Vehicle Operations
  // ==========================================================================

  async getVehicle(id: string): Promise<APIResponse<LaunchVehicle>> {
    return this.makeRequest('GET', `/vehicles/${id}`);
  }

  async listVehicles(filters?: {
    manufacturer?: string;
    class?: string;
    status?: string;
  }): Promise<PaginatedResponse<LaunchVehicle>> {
    const params = new URLSearchParams(filters as Record<string, string>);
    return this.makeRequest('GET', `/vehicles?${params}`);
  }

  async getVehicleSpecs(vehicleId: string): Promise<APIResponse<{
    stages: RocketStage[];
    totalMass: number;
    payloadToLEO: number;
    payloadToGTO: number;
  }>> {
    return this.makeRequest('GET', `/vehicles/${vehicleId}/specs`);
  }

  // ==========================================================================
  // Telemetry Operations
  // ==========================================================================

  async getTelemetry(vehicleId: string): Promise<APIResponse<TelemetryData>> {
    return this.makeRequest('GET', `/vehicles/${vehicleId}/telemetry`);
  }

  async getTelemetryHistory(vehicleId: string, startTime: string, endTime: string): Promise<APIResponse<TelemetryData[]>> {
    const params = new URLSearchParams({ startTime, endTime });
    return this.makeRequest('GET', `/vehicles/${vehicleId}/telemetry/history?${params}`);
  }

  // ==========================================================================
  // Mission Operations
  // ==========================================================================

  async getMission(missionId: string): Promise<APIResponse<LaunchMission>> {
    return this.makeRequest('GET', `/missions/${missionId}`);
  }

  async listMissions(filters?: {
    vehicleId?: string;
    status?: LaunchStatus;
    startDate?: string;
    endDate?: string;
  }): Promise<PaginatedResponse<LaunchMission>> {
    const params = new URLSearchParams(filters as Record<string, string>);
    return this.makeRequest('GET', `/missions?${params}`);
  }

  async createMission(mission: Omit<LaunchMission, 'missionId' | 'createdAt'>): Promise<APIResponse<LaunchMission>> {
    return this.makeRequest('POST', '/missions', mission);
  }

  // ==========================================================================
  // Rocket Equation Calculations
  // ==========================================================================

  calculateDeltaV(specificImpulseS: number, massRatio: number): number {
    return specificImpulseS * PHYSICS_CONSTANTS.G0 * Math.log(massRatio);
  }

  calculateMultiStageDeltaV(stages: Array<{ isp: number; massRatio: number }>): number {
    return stages.reduce((total, stage) => {
      return total + this.calculateDeltaV(stage.isp, stage.massRatio);
    }, 0);
  }

  calculateMassRatio(wetMass: number, dryMass: number): number {
    return wetMass / dryMass;
  }

  calculatePayloadFraction(payloadMass: number, totalMass: number): number {
    return payloadMass / totalMass;
  }

  calculateOrbitalVelocity(altitudeKm: number): number {
    const r = (PHYSICS_CONSTANTS.EARTH_RADIUS_KM + altitudeKm) * 1000;
    const mu = 3.986e14; // Earth gravitational parameter
    return Math.sqrt(mu / r);
  }

  calculateDeltaVToOrbit(altitudeKm: number): number {
    const orbitalVelocity = this.calculateOrbitalVelocity(altitudeKm);
    const gravityLoss = 1500; // Approximate gravity loss (m/s)
    const dragLoss = 100; // Approximate drag loss (m/s)
    return orbitalVelocity + gravityLoss + dragLoss;
  }

  // ==========================================================================
  // Event Handling
  // ==========================================================================

  on(event: 'launch' | 'stageSeparation' | 'orbitInsertion' | 'landing' | 'error', callback: (...args: unknown[]) => void): void {
    this.eventEmitter.on(event, callback);
  }

  off(event: string, callback: (...args: unknown[]) => void): void {
    this.eventEmitter.off(event, callback);
  }

  // ==========================================================================
  // Private Methods
  // ==========================================================================

  private async makeRequest<T>(method: string, path: string, body?: unknown): Promise<T> {
    const url = `${this.config.endpoint}${path}`;

    if (this.config.debug) {
      console.log(`[WIA Launch Vehicle] ${method} ${url}`, body);
    }

    try {
      const response = await fetch(url, {
        method,
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${this.config.apiKey}`,
          'X-WIA-Standard': 'SPACE-010',
          'X-WIA-Version': '1.0.0',
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

export function createClient(config: WIALaunchVehicleConfig): WIALaunchVehicleClient {
  return new WIALaunchVehicleClient(config);
}

export default WIALaunchVehicleClient;
