/**
 * WIA-SPACE-010: Space Debris Management Standard
 * TypeScript SDK Implementation
 */

import {
  DebrisObject,
  ConjunctionDataMessage,
  ManeuverPlan,
  DebrisRemovalMission,
  DebrisStatistics,
  APIResponse,
  CollisionRisk,
  ManeuverAction
} from './types';

export class SpaceDebrisSDK {
  private baseURL: string;
  private apiKey: string;

  constructor(baseURL: string, apiKey: string) {
    this.baseURL = baseURL.replace(/\/$/, '');
    this.apiKey = apiKey;
  }

  private async request<T>(endpoint: string, options: RequestInit = {}): Promise<APIResponse<T>> {
    const url = `${this.baseURL}${endpoint}`;
    const headers = {
      'Authorization': `Bearer ${this.apiKey}`,
      'Content-Type': 'application/json',
      ...options.headers
    };

    try {
      const response = await fetch(url, { ...options, headers });
      const data = await response.json();
      return {
        status: response.ok ? 'success' : 'error',
        data: response.ok ? data : undefined,
        error: response.ok ? undefined : data.message,
        timestamp: new Date().toISOString()
      };
    } catch (error) {
      return {
        status: 'error',
        error: error instanceof Error ? error.message : 'Unknown error',
        timestamp: new Date().toISOString()
      };
    }
  }

  async getTrackedObjects(sizeMin?: number): Promise<APIResponse<DebrisObject[]>> {
    const query = sizeMin ? `?size_min=${sizeMin}` : '';
    return this.request<DebrisObject[]>(`/api/v1/debris/objects${query}`);
  }

  async getObjectById(noradId: string): Promise<APIResponse<DebrisObject>> {
    return this.request<DebrisObject>(`/api/v1/debris/objects/${noradId}`);
  }

  async getConjunctionAlerts(satelliteId?: string): Promise<APIResponse<ConjunctionDataMessage[]>> {
    const query = satelliteId ? `?satellite=${satelliteId}` : '';
    return this.request<ConjunctionDataMessage[]>(`/api/v1/debris/conjunctions${query}`);
  }

  async submitManeuverPlan(plan: ManeuverPlan): Promise<APIResponse<{ plan_id: string }>> {
    return this.request<{ plan_id: string }>('/api/v1/maneuvers', {
      method: 'POST',
      body: JSON.stringify(plan)
    });
  }

  async getRemovalMissions(): Promise<APIResponse<DebrisRemovalMission[]>> {
    return this.request<DebrisRemovalMission[]>('/api/v1/debris/removal-missions');
  }

  async getStatistics(): Promise<APIResponse<DebrisStatistics>> {
    return this.request<DebrisStatistics>('/api/v1/debris/statistics');
  }

  calculateCollisionProbability(
    missDistance_m: number,
    combinedRadius_m: number,
    positionUncertainty_m: number
  ): { probability: number; action: ManeuverAction } {
    const pc = Math.exp(-Math.pow(missDistance_m / (2 * positionUncertainty_m), 2)) *
                Math.pow(combinedRadius_m / positionUncertainty_m, 2);

    let action: ManeuverAction;
    if (pc > 1e-3) action = ManeuverAction.EXECUTE_MANEUVER;
    else if (pc > 1e-4) action = ManeuverAction.CONSIDER_MANEUVER;
    else if (pc > 1e-5) action = ManeuverAction.ANALYZE;
    else action = ManeuverAction.MONITOR;

    return { probability: pc, action };
  }

  calculateDeorbitTime(altitude_km: number, mass_kg: number, area_m2: number): number {
    // Simplified model - actual calculations more complex
    const atmosphericDensity = Math.exp(-altitude_km / 7.5); // Scale height ~7.5 km
    const ballisticCoefficient = mass_kg / area_m2;
    const yearsToDeorbit = (altitude_km - 200) / (0.01 * atmosphericDensity * 365 * ballisticCoefficient);
    return Math.max(0, yearsToDeorbit);
  }
}

export * from './types';
export default SpaceDebrisSDK;
