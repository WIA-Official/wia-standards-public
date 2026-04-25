/**
 * WIA-AGRI-031 Deep-Sea Aquaculture Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @packageDocumentation
 * @module wia-deep-sea-aquaculture
 * @license MIT
 * @author WIA / SmileStory Inc.
 * @version 1.0.0
 */

import axios, { AxiosInstance } from 'axios';
import * as Types from './types';

export * from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface DeepSeaAquacultureConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  retries?: number;
  debug?: boolean;
}

// ============================================================================
// SDK Client
// ============================================================================

export class WIADeepSeaAquacultureClient {
  private client: AxiosInstance;
  private config: Required<DeepSeaAquacultureConfig>;

  constructor(config: DeepSeaAquacultureConfig) {
    this.config = {
      timeout: 30000,
      retries: 3,
      debug: false,
      ...config,
    };

    this.client = axios.create({
      baseURL: this.config.endpoint,
      timeout: this.config.timeout,
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${this.config.apiKey}`,
        'X-WIA-Standard': 'WIA-AGRI-031',
        'X-WIA-Version': '1.0.0',
      },
    });
  }

  // ==========================================================================
  // Farm APIs
  // ==========================================================================

  async createFarm(farm: Omit<Types.DeepSeaFarm, 'farmId' | 'createdAt' | 'updatedAt'>): Promise<Types.DeepSeaFarm> {
    const response = await this.client.post('/api/v1/farms', farm);
    return response.data;
  }

  async getFarm(farmId: string): Promise<Types.DeepSeaFarm> {
    const response = await this.client.get(`/api/v1/farms/${farmId}`);
    return response.data;
  }

  async updateFarm(farmId: string, updates: Partial<Types.DeepSeaFarm>): Promise<Types.DeepSeaFarm> {
    const response = await this.client.put(`/api/v1/farms/${farmId}`, updates);
    return response.data;
  }

  async deleteFarm(farmId: string): Promise<void> {
    await this.client.delete(`/api/v1/farms/${farmId}`);
  }

  async listFarms(filters?: { oceanZone?: string; country?: string }): Promise<Types.DeepSeaFarm[]> {
    const response = await this.client.get('/api/v1/farms', { params: filters });
    return response.data;
  }

  // ==========================================================================
  // Species APIs
  // ==========================================================================

  async addSpecies(farmId: string, species: Omit<Types.SpeciesInfo, 'speciesId'>): Promise<Types.SpeciesInfo> {
    const response = await this.client.post(`/api/v1/farms/${farmId}/species`, species);
    return response.data;
  }

  async getSpecies(farmId: string, speciesId: string): Promise<Types.SpeciesInfo> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/species/${speciesId}`);
    return response.data;
  }

  async listSpecies(farmId: string): Promise<Types.SpeciesInfo[]> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/species`);
    return response.data;
  }

  // ==========================================================================
  // Health APIs
  // ==========================================================================

  async getSpeciesHealth(farmId: string, speciesId: string): Promise<Types.HealthMetrics> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/species/${speciesId}/health`);
    return response.data;
  }

  async recordHealthCheck(farmId: string, speciesId: string, check: Types.HealthCheck): Promise<Types.HealthCheck> {
    const response = await this.client.post(`/api/v1/farms/${farmId}/species/${speciesId}/health`, check);
    return response.data;
  }

  // ==========================================================================
  // Environmental APIs
  // ==========================================================================

  async getWaterConditions(farmId: string): Promise<Types.WaterConditions> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/water`);
    return response.data;
  }

  async recordWaterConditions(farmId: string, conditions: Omit<Types.WaterConditions, 'timestamp'>): Promise<Types.WaterConditions> {
    const response = await this.client.post(`/api/v1/farms/${farmId}/water`, conditions);
    return response.data;
  }

  // ==========================================================================
  // Harvest & Sustainability APIs
  // ==========================================================================

  async getHarvestMetrics(farmId: string): Promise<Types.HarvestMetrics> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/harvest`);
    return response.data;
  }

  async recordHarvest(farmId: string, harvest: Types.HarvestRecord): Promise<Types.HarvestRecord> {
    const response = await this.client.post(`/api/v1/farms/${farmId}/harvest`, harvest);
    return response.data;
  }

  async getSustainabilityMetrics(farmId: string): Promise<Types.SustainabilityMetrics> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/sustainability`);
    return response.data;
  }

  async getCertifications(farmId: string): Promise<Types.Certification[]> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/certifications`);
    return response.data;
  }

  // ==========================================================================
  // Feeding APIs
  // ==========================================================================

  async getFeedingSchedule(farmId: string): Promise<Types.FeedingSchedule> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/feeding`);
    return response.data;
  }

  async updateFeedingSchedule(farmId: string, schedule: Types.FeedingSchedule): Promise<Types.FeedingSchedule> {
    const response = await this.client.put(`/api/v1/farms/${farmId}/feeding`, schedule);
    return response.data;
  }
}

// ============================================================================
// Factory Function
// ============================================================================

export function createClient(config: DeepSeaAquacultureConfig): WIADeepSeaAquacultureClient {
  return new WIADeepSeaAquacultureClient(config);
}

// ============================================================================
// Constants
// ============================================================================

export const VERSION = '1.0.0';

export const OCEAN_ZONES = {
  EPIPELAGIC: { minDepth: 0, maxDepth: 200 },
  MESOPELAGIC: { minDepth: 200, maxDepth: 1000 },
  BATHYPELAGIC: { minDepth: 1000, maxDepth: 4000 },
} as const;

export const WATER_QUALITY_THRESHOLDS = {
  oxygen: { min: 5, max: 14 },
  temperature: { min: 10, max: 25 },
  salinity: { min: 30, max: 40 },
  ph: { min: 7.5, max: 8.5 },
} as const;

export default {
  createClient,
  WIADeepSeaAquacultureClient,
};
