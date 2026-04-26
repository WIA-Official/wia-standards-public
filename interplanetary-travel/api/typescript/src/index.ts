/**
 * WIA-SPACE-013: Interplanetary Travel Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

import EventEmitter from 'eventemitter3';
import {
  FlightPlan,
  Trajectory,
  TrajectoryType,
  Spacecraft,
  CrewManifest,
  Planet,
  APIResponse,
  PaginatedResponse,
} from './types';

export * from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface WIAInterplanetaryConfig {
  apiKey: string;
  endpoint?: string;
  timeout?: number;
  debug?: boolean;
}

// ============================================================================
// Constants
// ============================================================================

const ORBITAL_PERIODS: Record<string, number> = {
  Mercury: 88,
  Venus: 225,
  Earth: 365,
  Mars: 687,
  Jupiter: 4333,
  Saturn: 10759,
  Uranus: 30687,
  Neptune: 60190,
};

const MEAN_DISTANCES_AU: Record<string, number> = {
  Mercury: 0.39,
  Venus: 0.72,
  Earth: 1.0,
  Mars: 1.52,
  Jupiter: 5.2,
  Saturn: 9.58,
  Uranus: 19.22,
  Neptune: 30.05,
};

// ============================================================================
// Main SDK Client
// ============================================================================

export class InterplanetaryTravelSDK {
  private config: Required<WIAInterplanetaryConfig>;
  private eventEmitter = new EventEmitter();

  constructor(config: WIAInterplanetaryConfig) {
    this.config = {
      endpoint: 'https://api.wia-standards.org/v1/interplanetary',
      timeout: 60000,
      debug: false,
      ...config,
    };

    if (!this.config.apiKey) {
      throw new Error('API key is required');
    }
  }

  // ==========================================================================
  // Flight Plan Operations
  // ==========================================================================

  async getFlightPlans(filters?: {
    origin?: string;
    destination?: string;
    status?: string;
  }): Promise<PaginatedResponse<FlightPlan>> {
    const params = new URLSearchParams(filters as Record<string, string>);
    return this.request(`/routes?${params}`);
  }

  async getFlightPlan(planId: string): Promise<APIResponse<FlightPlan>> {
    return this.request(`/routes/${planId}`);
  }

  async createFlightPlan(plan: Omit<FlightPlan, 'planId' | 'createdAt'>): Promise<APIResponse<FlightPlan>> {
    return this.request('/routes', {
      method: 'POST',
      body: JSON.stringify(plan),
    });
  }

  async getOptimalRoute(origin: string, destination: string, launchWindow?: string): Promise<APIResponse<FlightPlan>> {
    const params = new URLSearchParams({ from: origin, to: destination });
    if (launchWindow) params.append('launchWindow', launchWindow);
    return this.request(`/optimal?${params}`);
  }

  // ==========================================================================
  // Spacecraft Operations
  // ==========================================================================

  async getSpacecraft(spacecraftId: string): Promise<APIResponse<Spacecraft>> {
    return this.request(`/spacecraft/${spacecraftId}`);
  }

  async listSpacecraft(): Promise<PaginatedResponse<Spacecraft>> {
    return this.request('/spacecraft');
  }

  // ==========================================================================
  // Trajectory Calculations
  // ==========================================================================

  calculateHohmannTransfer(origin: string, destination: string): Trajectory {
    const r1 = MEAN_DISTANCES_AU[origin] || 1.0;
    const r2 = MEAN_DISTANCES_AU[destination] || 1.52;
    const a = (r1 + r2) / 2;
    const transferTime = Math.PI * Math.sqrt(Math.pow(a, 3));
    const durationDays = Math.round(transferTime * 58.13);

    return {
      trajectoryId: `hohmann-${Date.now()}`,
      type: TrajectoryType.HOHMANN,
      originPlanet: origin as Planet,
      destinationPlanet: destination as Planet,
      launchDate: new Date().toISOString(),
      arrivalDate: new Date(Date.now() + durationDays * 86400000).toISOString(),
      durationDays,
      deltaV: this.calculateDeltaV(origin, destination, TrajectoryType.HOHMANN),
    };
  }

  calculateDeltaV(origin: string, destination: string, trajectoryType: TrajectoryType): number {
    const baseDeltas: Record<string, Record<string, number>> = {
      'Earth-Mars': { hohmann: 5.7, conjunction: 5.5, opposition: 8.0 },
      'Earth-Jupiter': { hohmann: 9.0, conjunction: 8.5, opposition: 10.5 },
      'Earth-Venus': { hohmann: 3.5, conjunction: 3.2, opposition: 4.5 },
      'Mars-Jupiter': { hohmann: 5.8, conjunction: 5.5, opposition: 7.0 },
    };

    const key = `${origin}-${destination}`;
    const typeKey = trajectoryType.toLowerCase().replace('_', '');
    return baseDeltas[key]?.[typeKey] || 6.0;
  }

  calculateTransferWindow(origin: string, destination: string): {
    nextWindow: Date;
    synodicPeriod: number;
  } {
    const T1 = ORBITAL_PERIODS[origin] || 365;
    const T2 = ORBITAL_PERIODS[destination] || 687;
    const synodicPeriod = Math.abs((T1 * T2) / (T2 - T1));

    return {
      nextWindow: new Date(Date.now() + Math.random() * synodicPeriod * 86400000),
      synodicPeriod: Math.round(synodicPeriod),
    };
  }

  calculateDuration(origin: string, destination: string): number {
    const routes: Record<string, number> = {
      'Earth-Mars': 0.7,
      'Earth-Jupiter': 6.2,
      'Mars-Jupiter': 5.8,
      'Earth-Saturn': 7.0,
      'Earth-Venus': 0.4,
    };
    return routes[`${origin}-${destination}`] || 5.0;
  }

  // ==========================================================================
  // Crew Management
  // ==========================================================================

  async getCrewManifest(missionId: string): Promise<APIResponse<CrewManifest>> {
    return this.request(`/missions/${missionId}/crew`);
  }

  // ==========================================================================
  // Event Handling
  // ==========================================================================

  on(event: 'missionLaunched' | 'trajectoryUpdated' | 'error', callback: (...args: unknown[]) => void): void {
    this.eventEmitter.on(event, callback);
  }

  // ==========================================================================
  // Private Methods
  // ==========================================================================

  private async request<T>(endpoint: string, options: RequestInit = {}): Promise<T> {
    const url = `${this.config.endpoint}${endpoint}`;

    if (this.config.debug) {
      console.log(`[WIA Interplanetary] ${options.method || 'GET'} ${url}`);
    }

    try {
      const response = await fetch(url, {
        ...options,
        headers: {
          'Authorization': `Bearer ${this.config.apiKey}`,
          'Content-Type': 'application/json',
          'X-WIA-Standard': 'SPACE-013',
          'X-WIA-Version': '1.0.0',
          ...options.headers,
        },
      });

      return await response.json();
    } catch (error: unknown) {
      const message = error instanceof Error ? error.message : 'Unknown error';
      this.eventEmitter.emit('error', { code: 'REQUEST_FAILED', message });
      throw error;
    }
  }
}

export function createClient(config: WIAInterplanetaryConfig): InterplanetaryTravelSDK {
  return new InterplanetaryTravelSDK(config);
}

export default InterplanetaryTravelSDK;
