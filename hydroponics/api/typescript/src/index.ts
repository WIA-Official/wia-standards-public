/**
 * WIA-AGRI-034: Hydroponics Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

import EventEmitter from 'eventemitter3';
import * as Types from './types';

export * from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface WIAHydroponicsConfig {
  apiKey: string;
  endpoint?: string;
  timeout?: number;
  debug?: boolean;
}

// ============================================================================
// Main SDK Client
// ============================================================================

export class WIAHydroponicsClient {
  private config: Required<WIAHydroponicsConfig>;
  private eventEmitter = new EventEmitter();

  constructor(config: WIAHydroponicsConfig) {
    this.config = {
      endpoint: 'https://api.wia-standards.org/v1/hydroponics',
      timeout: 30000,
      debug: false,
      ...config,
    };

    if (!this.config.apiKey) {
      throw new Error('API key is required');
    }
  }

  // ==========================================================================
  // Farm Operations
  // ==========================================================================

  async createFarm(farm: Omit<Types.HydroponicFarm, 'farmId' | 'createdAt' | 'updatedAt'>): Promise<Types.HydroponicFarm> {
    return this.makeRequest('POST', '/farms', farm);
  }

  async getFarm(farmId: string): Promise<Types.HydroponicFarm> {
    return this.makeRequest('GET', `/farms/${farmId}`);
  }

  async updateFarm(farmId: string, updates: Partial<Types.HydroponicFarm>): Promise<Types.HydroponicFarm> {
    return this.makeRequest('PUT', `/farms/${farmId}`, updates);
  }

  async deleteFarm(farmId: string): Promise<void> {
    return this.makeRequest('DELETE', `/farms/${farmId}`);
  }

  async listFarms(filters?: { systemType?: Types.SystemType; facilityType?: string }): Promise<Types.HydroponicFarm[]> {
    const params = new URLSearchParams(filters as Record<string, string>);
    return this.makeRequest('GET', `/farms?${params}`);
  }

  // ==========================================================================
  // Crop Operations
  // ==========================================================================

  async addCrop(farmId: string, crop: Omit<Types.CropInfo, 'cropId'>): Promise<Types.CropInfo> {
    return this.makeRequest('POST', `/farms/${farmId}/crops`, crop);
  }

  async getCrop(farmId: string, cropId: string): Promise<Types.CropInfo> {
    return this.makeRequest('GET', `/farms/${farmId}/crops/${cropId}`);
  }

  async updateCrop(farmId: string, cropId: string, updates: Partial<Types.CropInfo>): Promise<Types.CropInfo> {
    return this.makeRequest('PATCH', `/farms/${farmId}/crops/${cropId}`, updates);
  }

  async getCropHealth(farmId: string, cropId: string): Promise<Types.CropHealth> {
    return this.makeRequest('GET', `/farms/${farmId}/crops/${cropId}/health`);
  }

  async listCrops(farmId: string): Promise<Types.CropInfo[]> {
    return this.makeRequest('GET', `/farms/${farmId}/crops`);
  }

  // ==========================================================================
  // Nutrient Management
  // ==========================================================================

  async getNutrientManagement(farmId: string): Promise<Types.NutrientManagement> {
    return this.makeRequest('GET', `/farms/${farmId}/nutrients`);
  }

  async adjustNutrient(farmId: string, adjustment: Omit<Types.NutrientAdjustment, 'adjustmentId' | 'timestamp'>): Promise<Types.NutrientAdjustment> {
    return this.makeRequest('POST', `/farms/${farmId}/nutrients/adjust`, adjustment);
  }

  async getNutrientHistory(farmId: string, startDate: string, endDate: string): Promise<Types.NutrientAdjustment[]> {
    const params = new URLSearchParams({ startDate, endDate });
    return this.makeRequest('GET', `/farms/${farmId}/nutrients/history?${params}`);
  }

  // ==========================================================================
  // Water System
  // ==========================================================================

  async getWaterSystem(farmId: string): Promise<Types.WaterSystem> {
    return this.makeRequest('GET', `/farms/${farmId}/water`);
  }

  async getWaterQuality(farmId: string): Promise<Types.WaterQuality> {
    return this.makeRequest('GET', `/farms/${farmId}/water/quality`);
  }

  async adjustWaterPH(farmId: string, targetPH: number): Promise<void> {
    return this.makeRequest('POST', `/farms/${farmId}/water/ph`, { targetPH });
  }

  // ==========================================================================
  // Environmental Control
  // ==========================================================================

  async getEnvironmentalControl(farmId: string): Promise<Types.EnvironmentalControl> {
    return this.makeRequest('GET', `/farms/${farmId}/environment`);
  }

  async setTemperature(farmId: string, target: number): Promise<void> {
    return this.makeRequest('POST', `/farms/${farmId}/environment/temperature`, { target });
  }

  async setHumidity(farmId: string, target: number): Promise<void> {
    return this.makeRequest('POST', `/farms/${farmId}/environment/humidity`, { target });
  }

  async setLighting(farmId: string, photoperiod: number, intensity: number): Promise<void> {
    return this.makeRequest('POST', `/farms/${farmId}/environment/lighting`, { photoperiod, intensity });
  }

  // ==========================================================================
  // Metrics
  // ==========================================================================

  async getYieldMetrics(farmId: string): Promise<Types.YieldMetrics> {
    return this.makeRequest('GET', `/farms/${farmId}/yield`);
  }

  async getSustainabilityMetrics(farmId: string): Promise<Types.SustainabilityMetrics> {
    return this.makeRequest('GET', `/farms/${farmId}/sustainability`);
  }

  // ==========================================================================
  // Automation
  // ==========================================================================

  async getAutomationSystem(farmId: string): Promise<Types.AutomationSystem> {
    return this.makeRequest('GET', `/farms/${farmId}/automation`);
  }

  async getAlerts(farmId: string): Promise<Types.Alert[]> {
    return this.makeRequest('GET', `/farms/${farmId}/alerts`);
  }

  async acknowledgeAlert(farmId: string, alertId: string): Promise<void> {
    return this.makeRequest('POST', `/farms/${farmId}/alerts/${alertId}/acknowledge`);
  }

  // ==========================================================================
  // Event Handling
  // ==========================================================================

  on(event: 'farmCreated' | 'cropHarvested' | 'nutrientAlert' | 'environmentAlert' | 'error', callback: (...args: unknown[]) => void): void {
    this.eventEmitter.on(event, callback);
  }

  // ==========================================================================
  // Private Methods
  // ==========================================================================

  private async makeRequest<T>(method: string, path: string, body?: unknown): Promise<T> {
    const url = `${this.config.endpoint}${path}`;

    if (this.config.debug) {
      console.log(`[WIA Hydroponics] ${method} ${url}`, body);
    }

    try {
      const response = await fetch(url, {
        method,
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${this.config.apiKey}`,
          'X-WIA-Standard': 'AGRI-034',
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

export function createClient(config: WIAHydroponicsConfig): WIAHydroponicsClient {
  return new WIAHydroponicsClient(config);
}

export default WIAHydroponicsClient;
