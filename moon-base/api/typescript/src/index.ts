/**
 * WIA-SPACE-011: Moon Base Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

import EventEmitter from 'eventemitter3';
import {
  LunarBase,
  ISRUProduction,
  Resources,
  BaseModule,
  Location,
  ModuleStatus,
  APIResponse,
  PaginatedResponse,
} from './types';

export * from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface WIAMoonBaseConfig {
  apiKey: string;
  endpoint?: string;
  timeout?: number;
  debug?: boolean;
}

// ============================================================================
// Constants
// ============================================================================

const LUNAR_CONSTANTS = {
  GRAVITY: 1.62, // m/s²
  DAY_LENGTH_HOURS: 708.7,
  WATER_PER_PERSON_KG: 3.5,
  OXYGEN_PER_PERSON_KG: 0.84,
  REGOLITH_OXYGEN_PERCENT: 0.43,
};

// ============================================================================
// Main SDK Client
// ============================================================================

export class MoonBaseSDK {
  private config: Required<WIAMoonBaseConfig>;
  private eventEmitter = new EventEmitter();

  constructor(config: WIAMoonBaseConfig) {
    this.config = {
      endpoint: 'https://api.wia-standards.org/v1/moon-base',
      timeout: 60000,
      debug: false,
      ...config,
    };

    if (!this.config.apiKey) {
      throw new Error('API key is required');
    }
  }

  // ==========================================================================
  // Base Operations
  // ==========================================================================

  async getBaseStatus(baseId?: string): Promise<APIResponse<LunarBase>> {
    const endpoint = baseId ? `/bases/${baseId}` : '/bases/status';
    return this.request(endpoint);
  }

  async listBases(): Promise<PaginatedResponse<LunarBase>> {
    return this.request('/bases');
  }

  async createBase(base: Omit<LunarBase, 'baseId' | 'createdAt'>): Promise<APIResponse<LunarBase>> {
    return this.request('/bases', {
      method: 'POST',
      body: JSON.stringify(base),
    });
  }

  async updateBaseStatus(baseId: string, status: string): Promise<APIResponse<LunarBase>> {
    return this.request(`/bases/${baseId}/status`, {
      method: 'PATCH',
      body: JSON.stringify({ status }),
    });
  }

  // ==========================================================================
  // Module Operations
  // ==========================================================================

  async getModules(baseId: string): Promise<PaginatedResponse<BaseModule>> {
    return this.request(`/bases/${baseId}/modules`);
  }

  async getModule(baseId: string, moduleId: string): Promise<APIResponse<BaseModule>> {
    return this.request(`/bases/${baseId}/modules/${moduleId}`);
  }

  async addModule(baseId: string, module: Omit<BaseModule, 'moduleId'>): Promise<APIResponse<BaseModule>> {
    return this.request(`/bases/${baseId}/modules`, {
      method: 'POST',
      body: JSON.stringify(module),
    });
  }

  async updateModuleStatus(baseId: string, moduleId: string, status: ModuleStatus): Promise<APIResponse<BaseModule>> {
    return this.request(`/bases/${baseId}/modules/${moduleId}/status`, {
      method: 'PATCH',
      body: JSON.stringify({ status }),
    });
  }

  // ==========================================================================
  // Resource Operations
  // ==========================================================================

  async getResources(baseId: string): Promise<APIResponse<Resources>> {
    return this.request(`/bases/${baseId}/resources`);
  }

  async updateResources(baseId: string, resources: Partial<Resources>): Promise<APIResponse<Resources>> {
    return this.request(`/bases/${baseId}/resources`, {
      method: 'PATCH',
      body: JSON.stringify(resources),
    });
  }

  // ==========================================================================
  // ISRU Operations
  // ==========================================================================

  async getISRUProduction(baseId: string): Promise<APIResponse<ISRUProduction>> {
    return this.request(`/bases/${baseId}/isru`);
  }

  async startISRU(baseId: string, targetOutput: number): Promise<APIResponse<ISRUProduction>> {
    return this.request(`/bases/${baseId}/isru/start`, {
      method: 'POST',
      body: JSON.stringify({ targetOutput }),
    });
  }

  // ==========================================================================
  // Calculation Methods
  // ==========================================================================

  calculateISRU(waterIceKg: number, crewSize: number): {
    oxygenPerDayKg: number;
    waterPerDayKg: number;
    missionDurationDays: number;
  } {
    const oxygenPerDay = (waterIceKg * 0.888) / 30;
    const waterPerDay = crewSize * LUNAR_CONSTANTS.WATER_PER_PERSON_KG;
    const duration = waterIceKg / waterPerDay;

    return {
      oxygenPerDayKg: parseFloat(oxygenPerDay.toFixed(2)),
      waterPerDayKg: waterPerDay,
      missionDurationDays: Math.floor(duration),
    };
  }

  calculateRegolithOxygen(regolithKg: number): number {
    return regolithKg * LUNAR_CONSTANTS.REGOLITH_OXYGEN_PERCENT;
  }

  calculateCrewRequirements(crewSize: number, durationDays: number): {
    waterKg: number;
    oxygenKg: number;
    foodKg: number;
  } {
    return {
      waterKg: crewSize * LUNAR_CONSTANTS.WATER_PER_PERSON_KG * durationDays,
      oxygenKg: crewSize * LUNAR_CONSTANTS.OXYGEN_PER_PERSON_KG * durationDays,
      foodKg: crewSize * 1.8 * durationDays,
    };
  }

  calculateSurfaceGravity(): number {
    return LUNAR_CONSTANTS.GRAVITY;
  }

  // ==========================================================================
  // Event Handling
  // ==========================================================================

  on(event: 'baseCreated' | 'moduleAdded' | 'resourceAlert' | 'error', callback: (...args: unknown[]) => void): void {
    this.eventEmitter.on(event, callback);
  }

  // ==========================================================================
  // Private Methods
  // ==========================================================================

  private async request<T>(endpoint: string, options: RequestInit = {}): Promise<T> {
    const url = `${this.config.endpoint}${endpoint}`;

    if (this.config.debug) {
      console.log(`[WIA Moon Base] ${options.method || 'GET'} ${url}`);
    }

    try {
      const response = await fetch(url, {
        ...options,
        headers: {
          'Authorization': `Bearer ${this.config.apiKey}`,
          'Content-Type': 'application/json',
          'X-WIA-Standard': 'SPACE-011',
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

export function createClient(config: WIAMoonBaseConfig): MoonBaseSDK {
  return new MoonBaseSDK(config);
}

export default MoonBaseSDK;
