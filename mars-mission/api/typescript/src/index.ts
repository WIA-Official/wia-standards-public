/**
 * WIA-SPACE-012: Mars Mission Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

import EventEmitter from 'eventemitter3';
import {
  MarsMission,
  Trajectory,
  TrajectoryType,
  MissionPhase,
  Crew,
  LandingSite,
  SurfaceHabitat,
  APIResponse,
  PaginatedResponse,
} from './types';

export * from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface WIAMarsMissionConfig {
  apiKey: string;
  endpoint?: string;
  timeout?: number;
  debug?: boolean;
}

// ============================================================================
// Constants
// ============================================================================

const MARS_CONSTANTS = {
  GRAVITY: 3.71, // m/s²
  ORBITAL_PERIOD_DAYS: 687,
  SYNODIC_PERIOD_DAYS: 780,
  MEAN_DISTANCE_AU: 1.52,
  ATMOSPHERIC_PRESSURE_KPA: 0.636,
};

// ============================================================================
// Main SDK Client
// ============================================================================

export class MarsMissionSDK {
  private config: Required<WIAMarsMissionConfig>;
  private eventEmitter = new EventEmitter();

  constructor(config: WIAMarsMissionConfig) {
    this.config = {
      endpoint: 'https://api.wia-standards.org/v1/mars',
      timeout: 60000,
      debug: false,
      ...config,
    };

    if (!this.config.apiKey) {
      throw new Error('API key is required');
    }
  }

  // ==========================================================================
  // Mission Operations
  // ==========================================================================

  async getMissions(filters?: {
    status?: string;
    type?: string;
    phase?: MissionPhase;
  }): Promise<PaginatedResponse<MarsMission>> {
    const params = new URLSearchParams(filters as Record<string, string>);
    return this.request(`/missions?${params}`);
  }

  async getMissionById(id: string): Promise<APIResponse<MarsMission>> {
    return this.request(`/missions/${id}`);
  }

  async createMission(mission: Omit<MarsMission, 'missionId' | 'createdAt'>): Promise<APIResponse<MarsMission>> {
    return this.request('/missions', {
      method: 'POST',
      body: JSON.stringify(mission),
    });
  }

  async updateMissionPhase(missionId: string, phase: MissionPhase): Promise<APIResponse<MarsMission>> {
    return this.request(`/missions/${missionId}/phase`, {
      method: 'PATCH',
      body: JSON.stringify({ phase }),
    });
  }

  // ==========================================================================
  // Trajectory Operations
  // ==========================================================================

  calculateTransferOrbit(launchDate: Date): Trajectory {
    const arrivalDate = new Date(launchDate);
    arrivalDate.setDate(arrivalDate.getDate() + 201);

    return {
      trajectoryId: `trajectory-${Date.now()}`,
      type: TrajectoryType.HOHMANN,
      launchDate: launchDate.toISOString(),
      arrivalDate: arrivalDate.toISOString(),
      durationDays: 201,
      deltaV: 5.7,
    };
  }

  calculateDeltaV(trajectory: TrajectoryType): number {
    const deltaVMap: Record<TrajectoryType, number> = {
      [TrajectoryType.HOHMANN]: 5.7,
      [TrajectoryType.CONJUNCTION]: 5.5,
      [TrajectoryType.OPPOSITION]: 8.0,
      [TrajectoryType.BALLISTIC]: 6.2,
    };
    return deltaVMap[trajectory];
  }

  getNextLaunchWindow(): { windowStart: Date; windowEnd: Date; optimalDate: Date } {
    const now = new Date();
    const daysToWindow = Math.random() * MARS_CONSTANTS.SYNODIC_PERIOD_DAYS;
    const windowStart = new Date(now.getTime() + daysToWindow * 86400000);
    const windowEnd = new Date(windowStart.getTime() + 30 * 86400000);
    const optimalDate = new Date(windowStart.getTime() + 15 * 86400000);

    return { windowStart, windowEnd, optimalDate };
  }

  // ==========================================================================
  // Landing Site Operations
  // ==========================================================================

  async getLandingSites(): Promise<PaginatedResponse<LandingSite>> {
    return this.request('/landing-sites');
  }

  async getLandingSite(siteId: string): Promise<APIResponse<LandingSite>> {
    return this.request(`/landing-sites/${siteId}`);
  }

  async evaluateLandingSite(siteId: string): Promise<APIResponse<{
    safetyScore: number;
    resourceScore: number;
    accessibilityScore: number;
    overallScore: number;
  }>> {
    return this.request(`/landing-sites/${siteId}/evaluate`);
  }

  // ==========================================================================
  // Habitat Operations
  // ==========================================================================

  async getHabitats(missionId: string): Promise<PaginatedResponse<SurfaceHabitat>> {
    return this.request(`/missions/${missionId}/habitats`);
  }

  async getHabitatStatus(habitatId: string): Promise<APIResponse<{
    pressureKpa: number;
    temperatureC: number;
    oxygenPercent: number;
    co2Ppm: number;
    powerWatts: number;
    waterLiters: number;
  }>> {
    return this.request(`/habitats/${habitatId}/status`);
  }

  // ==========================================================================
  // Crew Operations
  // ==========================================================================

  async getCrewManifest(missionId: string): Promise<APIResponse<Crew>> {
    return this.request(`/missions/${missionId}/crew`);
  }

  async getCrewHealth(missionId: string): Promise<APIResponse<{
    crewId: string;
    name: string;
    healthStatus: 'nominal' | 'attention' | 'critical';
    radiationDose: number;
    daysInSpace: number;
  }[]>> {
    return this.request(`/missions/${missionId}/crew/health`);
  }

  // ==========================================================================
  // Utility Methods
  // ==========================================================================

  calculateCommunicationDelay(earthMarsDistanceAu: number): number {
    const lightSpeedAuPerMinute = 0.002004;
    return earthMarsDistanceAu / lightSpeedAuPerMinute;
  }

  calculateSurfaceGravity(): number {
    return MARS_CONSTANTS.GRAVITY;
  }

  // ==========================================================================
  // Event Handling
  // ==========================================================================

  on(event: 'missionLaunched' | 'phaseChanged' | 'landingComplete' | 'error', callback: (...args: unknown[]) => void): void {
    this.eventEmitter.on(event, callback);
  }

  // ==========================================================================
  // Private Methods
  // ==========================================================================

  private async request<T>(endpoint: string, options: RequestInit = {}): Promise<T> {
    const url = `${this.config.endpoint}${endpoint}`;

    if (this.config.debug) {
      console.log(`[WIA Mars Mission] ${options.method || 'GET'} ${url}`);
    }

    try {
      const response = await fetch(url, {
        ...options,
        headers: {
          'Authorization': `Bearer ${this.config.apiKey}`,
          'Content-Type': 'application/json',
          'X-WIA-Standard': 'SPACE-012',
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

export function createClient(config: WIAMarsMissionConfig): MarsMissionSDK {
  return new MarsMissionSDK(config);
}

export default MarsMissionSDK;
