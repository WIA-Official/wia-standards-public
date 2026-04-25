/**
 * WIA-AGRI-030 Polar Agriculture Standard - TypeScript SDK
 * @version 1.0.0
 * @standard WIA-AGRI-030
 * @license MIT
 */

import axios, { AxiosInstance } from 'axios';
import * as Types from './types';

export * from './types';

/**
 * Main SDK client for WIA-AGRI-030 Polar Agriculture Standard
 */
export class WIAPolarAgricultureClient {
  private client: AxiosInstance;
  private config: Types.ClientConfig;

  constructor(config: Types.ClientConfig) {
    this.config = config;
    this.client = axios.create({
      baseURL: config.baseURL,
      timeout: config.timeout || 30000,
      headers: {
        'Content-Type': 'application/json',
        'X-WIA-Standard': 'WIA-AGRI-030',
        'X-WIA-Version': '1.0.0',
        ...(config.apiKey && { Authorization: `Bearer ${config.apiKey}` }),
      },
    });
  }

  // ========================================================================
  // Farm Management
  // ========================================================================

  async createFarm(farm: Omit<Types.PolarFarm, 'farmId' | 'createdAt' | 'updatedAt'>): Promise<Types.PolarFarm> {
    const response = await this.client.post('/api/v1/farms', farm);
    return response.data;
  }

  async getFarm(farmId: string): Promise<Types.PolarFarm> {
    const response = await this.client.get(`/api/v1/farms/${farmId}`);
    return response.data;
  }

  async updateFarm(farmId: string, updates: Partial<Types.PolarFarm>): Promise<Types.PolarFarm> {
    const response = await this.client.put(`/api/v1/farms/${farmId}`, updates);
    return response.data;
  }

  async deleteFarm(farmId: string): Promise<void> {
    await this.client.delete(`/api/v1/farms/${farmId}`);
  }

  async listFarms(filters?: { climateZone?: Types.ClimateZone; region?: string }): Promise<Types.PolarFarm[]> {
    const response = await this.client.get('/api/v1/farms', { params: filters });
    return response.data;
  }

  // ========================================================================
  // Environmental Monitoring
  // ========================================================================

  async getEnvironmentalConditions(farmId: string): Promise<Types.EnvironmentalConditions> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/environment`);
    return response.data;
  }

  async getOutdoorConditions(farmId: string): Promise<Types.OutdoorConditions> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/environment/outdoor`);
    return response.data;
  }

  async getIndoorConditions(farmId: string): Promise<Types.IndoorConditions> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/environment/indoor`);
    return response.data;
  }

  // ========================================================================
  // Crop Management
  // ========================================================================

  async addCrop(farmId: string, crop: Omit<Types.CropInfo, 'cropId'>): Promise<Types.CropInfo> {
    const response = await this.client.post(`/api/v1/farms/${farmId}/crops`, crop);
    return response.data;
  }

  async getCrop(farmId: string, cropId: string): Promise<Types.CropInfo> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/crops/${cropId}`);
    return response.data;
  }

  async updateCrop(farmId: string, cropId: string, updates: Partial<Types.CropInfo>): Promise<Types.CropInfo> {
    const response = await this.client.put(`/api/v1/farms/${farmId}/crops/${cropId}`, updates);
    return response.data;
  }

  async getCropHealth(farmId: string, cropId: string): Promise<Types.CropHealth> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/crops/${cropId}/health`);
    return response.data;
  }

  async getColdAdaptation(farmId: string, cropId: string): Promise<Types.ColdAdaptationMetrics> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/crops/${cropId}/adaptation`);
    return response.data;
  }

  // ========================================================================
  // Energy Management
  // ========================================================================

  async getEnergyManagement(farmId: string): Promise<Types.EnergyManagement> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/energy`);
    return response.data;
  }

  async getEnergyConsumption(farmId: string, period?: { startDate?: string; endDate?: string }): Promise<Types.EnergyConsumption> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/energy/consumption`, { params: period });
    return response.data;
  }

  async optimizeEnergy(farmId: string): Promise<{ recommendations: string[]; potentialSavings: number }> {
    const response = await this.client.post(`/api/v1/farms/${farmId}/energy/optimize`);
    return response.data;
  }

  // ========================================================================
  // Permafrost Monitoring
  // ========================================================================

  async getPermafrostMonitoring(farmId: string): Promise<Types.PermafrostMonitoring> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/permafrost`);
    return response.data;
  }

  async addPermafrostMeasurement(farmId: string, measurement: Omit<Types.PermafrostMeasurement, 'measurementId'>): Promise<Types.PermafrostMeasurement> {
    const response = await this.client.post(`/api/v1/farms/${farmId}/permafrost/measurements`, measurement);
    return response.data;
  }

  async getPermafrostAlerts(farmId: string): Promise<Types.PermafrostAlert[]> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/permafrost/alerts`);
    return response.data;
  }

  // ========================================================================
  // Sustainability
  // ========================================================================

  async getSustainabilityMetrics(farmId: string): Promise<Types.SustainabilityMetrics> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/sustainability`);
    return response.data;
  }

  async getFoodSecurity(farmId: string): Promise<Types.FoodSecurityMetrics> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/food-security`);
    return response.data;
  }
}

export function createClient(config: Types.ClientConfig): WIAPolarAgricultureClient {
  return new WIAPolarAgricultureClient(config);
}

export default {
  createClient,
  WIAPolarAgricultureClient,
};
