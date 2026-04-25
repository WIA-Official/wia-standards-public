/**
 * WIA Vertical Farming Standard - TypeScript SDK
 * @version 1.0.0
 * @license MIT
 */

import {
  VerticalFarm,
  GrowLayer,
  EnvironmentalData,
  NutrientData,
  HarvestRecord,
  EnergyConsumption,
  AutomationConfig,
  ApiResponse,
  VerticalFarmQuery,
} from './types';

export interface VerticalFarmingConfig {
  baseUrl: string;
  apiKey?: string;
  timeout?: number;
}

export class VerticalFarmingClient {
  private config: VerticalFarmingConfig;
  private headers: Record<string, string>;

  constructor(config: VerticalFarmingConfig) {
    this.config = { timeout: 30000, ...config };
    this.headers = {
      'Content-Type': 'application/json',
      'Accept': 'application/json',
    };
    if (config.apiKey) {
      this.headers['Authorization'] = `Bearer ${config.apiKey}`;
    }
  }

  private async request<T>(endpoint: string, options: RequestInit = {}): Promise<ApiResponse<T>> {
    const url = `${this.config.baseUrl}${endpoint}`;
    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), this.config.timeout);

    try {
      const response = await fetch(url, {
        ...options,
        headers: { ...this.headers, ...options.headers },
        signal: controller.signal,
      });
      clearTimeout(timeoutId);

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }

      const data = await response.json();
      return { success: true, data, timestamp: new Date().toISOString() };
    } catch (error) {
      clearTimeout(timeoutId);
      return {
        success: false,
        error: error instanceof Error ? error.message : 'Unknown error',
        timestamp: new Date().toISOString(),
      };
    }
  }

  // ==================== Farm Management ====================

  async getFarm(farmId: string): Promise<ApiResponse<VerticalFarm>> {
    return this.request<VerticalFarm>(`/farms/${farmId}`);
  }

  async createFarm(farm: VerticalFarm): Promise<ApiResponse<VerticalFarm>> {
    return this.request<VerticalFarm>('/farms', {
      method: 'POST',
      body: JSON.stringify(farm),
    });
  }

  async updateFarm(farmId: string, farm: Partial<VerticalFarm>): Promise<ApiResponse<VerticalFarm>> {
    return this.request<VerticalFarm>(`/farms/${farmId}`, {
      method: 'PUT',
      body: JSON.stringify(farm),
    });
  }

  async listFarms(query?: VerticalFarmQuery): Promise<ApiResponse<VerticalFarm[]>> {
    const params = new URLSearchParams();
    if (query) {
      Object.entries(query).forEach(([key, value]) => {
        if (value !== undefined) params.append(key, String(value));
      });
    }
    return this.request<VerticalFarm[]>(`/farms${params.toString() ? `?${params}` : ''}`);
  }

  // ==================== Grow Layer Management ====================

  async getLayer(layerId: string): Promise<ApiResponse<GrowLayer>> {
    return this.request<GrowLayer>(`/layers/${layerId}`);
  }

  async createLayer(layer: GrowLayer): Promise<ApiResponse<GrowLayer>> {
    return this.request<GrowLayer>('/layers', {
      method: 'POST',
      body: JSON.stringify(layer),
    });
  }

  async updateLayer(layerId: string, layer: Partial<GrowLayer>): Promise<ApiResponse<GrowLayer>> {
    return this.request<GrowLayer>(`/layers/${layerId}`, {
      method: 'PUT',
      body: JSON.stringify(layer),
    });
  }

  async listLayers(farmId: string): Promise<ApiResponse<GrowLayer[]>> {
    return this.request<GrowLayer[]>(`/farms/${farmId}/layers`);
  }

  // ==================== Environmental Monitoring ====================

  async submitEnvironmentalData(data: EnvironmentalData): Promise<ApiResponse<EnvironmentalData>> {
    return this.request<EnvironmentalData>('/environmental-data', {
      method: 'POST',
      body: JSON.stringify(data),
    });
  }

  async getEnvironmentalHistory(
    layerId: string,
    query?: VerticalFarmQuery
  ): Promise<ApiResponse<EnvironmentalData[]>> {
    const params = new URLSearchParams();
    if (query) {
      Object.entries(query).forEach(([key, value]) => {
        if (value !== undefined) params.append(key, String(value));
      });
    }
    return this.request<EnvironmentalData[]>(
      `/layers/${layerId}/environmental${params.toString() ? `?${params}` : ''}`
    );
  }

  // ==================== Nutrient Management ====================

  async submitNutrientData(data: NutrientData): Promise<ApiResponse<NutrientData>> {
    return this.request<NutrientData>('/nutrient-data', {
      method: 'POST',
      body: JSON.stringify(data),
    });
  }

  async getNutrientHistory(layerId: string): Promise<ApiResponse<NutrientData[]>> {
    return this.request<NutrientData[]>(`/layers/${layerId}/nutrients`);
  }

  // ==================== Harvest Management ====================

  async recordHarvest(harvest: HarvestRecord): Promise<ApiResponse<HarvestRecord>> {
    return this.request<HarvestRecord>('/harvests', {
      method: 'POST',
      body: JSON.stringify(harvest),
    });
  }

  async getHarvestHistory(farmId: string): Promise<ApiResponse<HarvestRecord[]>> {
    return this.request<HarvestRecord[]>(`/farms/${farmId}/harvests`);
  }

  // ==================== Energy Management ====================

  async submitEnergyData(data: EnergyConsumption): Promise<ApiResponse<EnergyConsumption>> {
    return this.request<EnergyConsumption>('/energy-data', {
      method: 'POST',
      body: JSON.stringify(data),
    });
  }

  async getEnergyHistory(farmId: string): Promise<ApiResponse<EnergyConsumption[]>> {
    return this.request<EnergyConsumption[]>(`/farms/${farmId}/energy`);
  }

  // ==================== Automation ====================

  async getAutomationConfig(layerId: string): Promise<ApiResponse<AutomationConfig>> {
    return this.request<AutomationConfig>(`/layers/${layerId}/automation`);
  }

  async updateAutomationConfig(
    layerId: string,
    config: AutomationConfig
  ): Promise<ApiResponse<AutomationConfig>> {
    return this.request<AutomationConfig>(`/layers/${layerId}/automation`, {
      method: 'PUT',
      body: JSON.stringify(config),
    });
  }
}

export * from './types';
export default VerticalFarmingClient;
