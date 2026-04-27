/**
 * WIA-AGRI-032 Seaweed Farming Standard - TypeScript SDK
 * @version 1.0.0
 * @standard WIA-AGRI-032
 * @license MIT
 */

import axios, { AxiosInstance } from 'axios';
import * as Types from './types';

export * from './types';

export class WIASeaweedFarmingClient {
  private client: AxiosInstance;
  private config: Types.ClientConfig;

  constructor(config: Types.ClientConfig) {
    this.config = config;
    this.client = axios.create({
      baseURL: config.baseURL,
      timeout: config.timeout || 30000,
      headers: {
        'Content-Type': 'application/json',
        'X-WIA-Standard': 'WIA-AGRI-032',
        'X-WIA-Version': '1.0.0',
        ...(config.apiKey && { Authorization: `Bearer ${config.apiKey}` }),
      },
    });
  }

  async createFarm(farm: Omit<Types.SeaweedFarm, 'farmId' | 'createdAt' | 'updatedAt'>): Promise<Types.SeaweedFarm> {
    const response = await this.client.post('/api/v1/farms', farm);
    return response.data;
  }

  async getFarm(farmId: string): Promise<Types.SeaweedFarm> {
    const response = await this.client.get(`/api/v1/farms/${farmId}`);
    return response.data;
  }

  async updateFarm(farmId: string, updates: Partial<Types.SeaweedFarm>): Promise<Types.SeaweedFarm> {
    const response = await this.client.put(`/api/v1/farms/${farmId}`, updates);
    return response.data;
  }

  async deleteFarm(farmId: string): Promise<void> {
    await this.client.delete(`/api/v1/farms/${farmId}`);
  }

  async listFarms(filters?: { waterBody?: string; farmingMethod?: Types.FarmingMethod }): Promise<Types.SeaweedFarm[]> {
    const response = await this.client.get('/api/v1/farms', { params: filters });
    return response.data;
  }

  async addSpecies(farmId: string, species: Omit<Types.SeaweedSpecies, 'speciesId'>): Promise<Types.SeaweedSpecies> {
    const response = await this.client.post(`/api/v1/farms/${farmId}/species`, species);
    return response.data;
  }

  async getWaterConditions(farmId: string): Promise<Types.WaterConditions> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/water`);
    return response.data;
  }

  async getHarvestData(farmId: string): Promise<Types.HarvestData> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/harvest`);
    return response.data;
  }

  async getSustainabilityMetrics(farmId: string): Promise<Types.SustainabilityMetrics> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/sustainability`);
    return response.data;
  }

  async getCarbonSequestration(farmId: string): Promise<Types.Measurement> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/carbon-sequestration`);
    return response.data;
  }
}

export function createClient(config: Types.ClientConfig): WIASeaweedFarmingClient {
  return new WIASeaweedFarmingClient(config);
}

export default {
  createClient,
  WIASeaweedFarmingClient,
};
