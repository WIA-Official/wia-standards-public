/**
 * WIA-AGRI-029 Desert Agriculture Standard - TypeScript SDK
 * @version 1.0.0
 * @standard WIA-AGRI-029
 * @license MIT
 */

import axios, { AxiosInstance } from 'axios';
import * as Types from './types';

export * from './types';

/**
 * Main SDK client for WIA-AGRI-029 Desert Agriculture Standard
 */
export class WIADesertAgricultureClient {
  private client: AxiosInstance;
  private config: Types.ClientConfig;

  constructor(config: Types.ClientConfig) {
    this.config = config;
    this.client = axios.create({
      baseURL: config.baseURL,
      timeout: config.timeout || 30000,
      headers: {
        'Content-Type': 'application/json',
        'X-WIA-Standard': 'WIA-AGRI-029',
        'X-WIA-Version': '1.0.0',
        ...(config.apiKey && { Authorization: `Bearer ${config.apiKey}` }),
      },
    });
  }

  // ========================================================================
  // Farm Management
  // ========================================================================

  /**
   * Register a desert farm
   */
  async registerFarm(farm: Omit<Types.DesertFarm, 'farmId' | 'createdAt' | 'updatedAt'>): Promise<Types.DesertFarm> {
    const response = await this.client.post('/api/v1/farms', farm);
    return response.data;
  }

  /**
   * Get farm by ID
   */
  async getFarm(farmId: string): Promise<Types.DesertFarm> {
    const response = await this.client.get(`/api/v1/farms/${farmId}`);
    return response.data;
  }

  /**
   * Update farm information
   */
  async updateFarm(farmId: string, updates: Partial<Types.DesertFarm>): Promise<Types.DesertFarm> {
    const response = await this.client.put(`/api/v1/farms/${farmId}`, updates);
    return response.data;
  }

  /**
   * Delete farm
   */
  async deleteFarm(farmId: string): Promise<void> {
    await this.client.delete(`/api/v1/farms/${farmId}`);
  }

  /**
   * List farms
   */
  async listFarms(filters?: { region?: string; country?: string }): Promise<Types.DesertFarm[]> {
    const response = await this.client.get('/api/v1/farms', { params: filters });
    return response.data;
  }

  // ========================================================================
  // Water Management
  // ========================================================================

  /**
   * Get water management details
   */
  async getWaterManagement(farmId: string): Promise<Types.WaterManagement> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/water`);
    return response.data;
  }

  /**
   * Update water management
   */
  async updateWaterManagement(farmId: string, management: Types.WaterManagement): Promise<Types.WaterManagement> {
    const response = await this.client.put(`/api/v1/farms/${farmId}/water`, management);
    return response.data;
  }

  /**
   * Get water usage metrics
   */
  async getWaterUsage(farmId: string, period?: { startDate?: string; endDate?: string }): Promise<Types.WaterUsageMetrics> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/water/usage`, { params: period });
    return response.data;
  }

  /**
   * Add water source
   */
  async addWaterSource(farmId: string, source: Types.WaterSourceInfo): Promise<Types.WaterSourceInfo> {
    const response = await this.client.post(`/api/v1/farms/${farmId}/water/sources`, source);
    return response.data;
  }

  /**
   * Test water quality
   */
  async testWaterQuality(farmId: string, sourceId: string): Promise<Types.WaterQuality> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/water/sources/${sourceId}/quality`);
    return response.data;
  }

  // ========================================================================
  // Crop Management
  // ========================================================================

  /**
   * Add crop
   */
  async addCrop(farmId: string, crop: Omit<Types.CropInfo, 'cropId'>): Promise<Types.CropInfo> {
    const response = await this.client.post(`/api/v1/farms/${farmId}/crops`, crop);
    return response.data;
  }

  /**
   * Get crop information
   */
  async getCrop(farmId: string, cropId: string): Promise<Types.CropInfo> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/crops/${cropId}`);
    return response.data;
  }

  /**
   * Update crop
   */
  async updateCrop(farmId: string, cropId: string, updates: Partial<Types.CropInfo>): Promise<Types.CropInfo> {
    const response = await this.client.put(`/api/v1/farms/${farmId}/crops/${cropId}`, updates);
    return response.data;
  }

  /**
   * Get crop health
   */
  async getCropHealth(farmId: string, cropId: string): Promise<Types.CropHealth> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/crops/${cropId}/health`);
    return response.data;
  }

  /**
   * Get adaptation metrics
   */
  async getAdaptationMetrics(farmId: string, cropId: string): Promise<Types.AdaptationMetrics> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/crops/${cropId}/adaptation`);
    return response.data;
  }

  /**
   * List crops
   */
  async listCrops(farmId: string): Promise<Types.CropInfo[]> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/crops`);
    return response.data;
  }

  // ========================================================================
  // Soil Management
  // ========================================================================

  /**
   * Get soil profile
   */
  async getSoilProfile(farmId: string): Promise<Types.SoilProfile> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/soil`);
    return response.data;
  }

  /**
   * Update soil profile
   */
  async updateSoilProfile(farmId: string, profile: Types.SoilProfile): Promise<Types.SoilProfile> {
    const response = await this.client.put(`/api/v1/farms/${farmId}/soil`, profile);
    return response.data;
  }

  /**
   * Test soil nutrients
   */
  async testSoilNutrients(farmId: string): Promise<Types.NutrientProfile> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/soil/nutrients`);
    return response.data;
  }

  /**
   * Assess desertification
   */
  async assessDesertification(farmId: string): Promise<{
    level: Types.DesertificationLevel;
    indicators: string[];
    recommendations: string[];
  }> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/soil/desertification`);
    return response.data;
  }

  // ========================================================================
  // Climate Monitoring
  // ========================================================================

  /**
   * Get climate profile
   */
  async getClimateProfile(farmId: string): Promise<Types.ClimateProfile> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/climate`);
    return response.data;
  }

  /**
   * Get weather data
   */
  async getWeatherData(farmId: string, period?: { startDate?: string; endDate?: string }): Promise<any[]> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/weather`, { params: period });
    return response.data;
  }

  /**
   * Get solar radiation data
   */
  async getSolarRadiation(farmId: string): Promise<Types.SolarProfile> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/solar`);
    return response.data;
  }

  // ========================================================================
  // Infrastructure
  // ========================================================================

  /**
   * Get infrastructure details
   */
  async getInfrastructure(farmId: string): Promise<Types.Infrastructure> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/infrastructure`);
    return response.data;
  }

  /**
   * Add greenhouse
   */
  async addGreenhouse(farmId: string, greenhouse: Types.GreenhouseInfo): Promise<Types.GreenhouseInfo> {
    const response = await this.client.post(`/api/v1/farms/${farmId}/greenhouses`, greenhouse);
    return response.data;
  }

  /**
   * Add water storage
   */
  async addWaterStorage(farmId: string, storage: Types.WaterStorageInfo): Promise<Types.WaterStorageInfo> {
    const response = await this.client.post(`/api/v1/farms/${farmId}/water-storage`, storage);
    return response.data;
  }

  /**
   * Get power systems
   */
  async getPowerSystems(farmId: string): Promise<Types.PowerSystem[]> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/power`);
    return response.data;
  }

  // ========================================================================
  // Sustainability
  // ========================================================================

  /**
   * Get sustainability metrics
   */
  async getSustainabilityMetrics(farmId: string): Promise<Types.SustainabilityMetrics> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/sustainability`);
    return response.data;
  }

  /**
   * Calculate carbon footprint
   */
  async calculateCarbonFootprint(farmId: string): Promise<Types.Measurement> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/carbon-footprint`);
    return response.data;
  }

  /**
   * Get economic metrics
   */
  async getEconomicMetrics(farmId: string): Promise<Types.EconomicMetrics> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/economics`);
    return response.data;
  }

  // ========================================================================
  // Challenges & Mitigation
  // ========================================================================

  /**
   * Get environmental challenges
   */
  async getChall enges(farmId: string): Promise<Types.EnvironmentalChallenge[]> {
    const response = await this.client.get(`/api/v1/farms/${farmId}/challenges`);
    return response.data;
  }

  /**
   * Add challenge
   */
  async addChallenge(farmId: string, challenge: Omit<Types.EnvironmentalChallenge, 'challengeId'>): Promise<Types.EnvironmentalChallenge> {
    const response = await this.client.post(`/api/v1/farms/${farmId}/challenges`, challenge);
    return response.data;
  }

  /**
   * Update challenge status
   */
  async updateChallengeStatus(
    farmId: string,
    challengeId: string,
    status: Types.EnvironmentalChallenge['status']
  ): Promise<Types.EnvironmentalChallenge> {
    const response = await this.client.put(`/api/v1/farms/${farmId}/challenges/${challengeId}`, { status });
    return response.data;
  }
}

/**
 * Factory function to create WIA Desert Agriculture client
 */
export function createClient(config: Types.ClientConfig): WIADesertAgricultureClient {
  return new WIADesertAgricultureClient(config);
}

/**
 * Default export
 */
export default {
  createClient,
  WIADesertAgricultureClient,
};
