/**
 * WIA-AGRI-030 Urban Farming Standard - TypeScript SDK
 * @module @wia/urban-farming
 */

import axios, { AxiosInstance } from 'axios';
import * as types from './types';

export * from './types';

/**
 * Main Urban Farming Client
 */
export class UrbanFarmingClient {
  private api: AxiosInstance;
  private config: types.ClientConfig;

  constructor(config: types.ClientConfig) {
    this.config = {
      baseURL: 'https://api.wia-urbanfarming.io/v1',
      timeout: 30000,
      retryAttempts: 3,
      ...config,
    };

    this.api = axios.create({
      baseURL: this.config.baseURL,
      timeout: this.config.timeout,
      headers: {
        'X-API-Key': this.config.apiKey,
        'Content-Type': 'application/json',
      },
    });
  }

  // ========================================================================
  // Farm Management Methods
  // ========================================================================

  /**
   * Get all urban farms
   */
  async getFarms(city?: string, type?: types.FarmType): Promise<types.UrbanFarm[]> {
    const response = await this.api.get('/farms', { params: { city, type } });
    return response.data;
  }

  /**
   * Get farm by ID
   */
  async getFarm(farmId: string): Promise<types.UrbanFarm> {
    const response = await this.api.get(`/farms/${farmId}`);
    return response.data;
  }

  /**
   * Create new farm
   */
  async createFarm(farm: Partial<types.UrbanFarm>): Promise<types.UrbanFarm> {
    const response = await this.api.post('/farms', farm);
    return response.data;
  }

  /**
   * Update farm
   */
  async updateFarm(farmId: string, updates: Partial<types.UrbanFarm>): Promise<types.UrbanFarm> {
    const response = await this.api.patch(`/farms/${farmId}`, updates);
    return response.data;
  }

  /**
   * Delete farm
   */
  async deleteFarm(farmId: string): Promise<void> {
    await this.api.delete(`/farms/${farmId}`);
  }

  // ========================================================================
  // Production Methods
  // ========================================================================

  /**
   * Get production records
   */
  async getProduction(farmId: string, crop?: string): Promise<types.Production[]> {
    const response = await this.api.get('/production', { params: { farmId, crop } });
    return response.data;
  }

  /**
   * Record production
   */
  async recordProduction(production: Partial<types.Production>): Promise<types.Production> {
    const response = await this.api.post('/production', production);
    return response.data;
  }

  /**
   * Get crop schedules
   */
  async getCropSchedules(farmId: string): Promise<types.CropSchedule[]> {
    const response = await this.api.get('/schedules', { params: { farmId } });
    return response.data;
  }

  /**
   * Create crop schedule
   */
  async createCropSchedule(schedule: Partial<types.CropSchedule>): Promise<types.CropSchedule> {
    const response = await this.api.post('/schedules', schedule);
    return response.data;
  }

  // ========================================================================
  // Community Methods
  // ========================================================================

  /**
   * Get community programs
   */
  async getCommunityPrograms(farmId?: string, type?: string): Promise<types.CommunityProgram[]> {
    const response = await this.api.get('/community/programs', { params: { farmId, type } });
    return response.data;
  }

  /**
   * Create community program
   */
  async createCommunityProgram(program: Partial<types.CommunityProgram>): Promise<types.CommunityProgram> {
    const response = await this.api.post('/community/programs', program);
    return response.data;
  }

  /**
   * Get workshops
   */
  async getWorkshops(upcoming?: boolean): Promise<types.Workshop[]> {
    const response = await this.api.get('/community/workshops', { params: { upcoming } });
    return response.data;
  }

  /**
   * Create workshop
   */
  async createWorkshop(workshop: Partial<types.Workshop>): Promise<types.Workshop> {
    const response = await this.api.post('/community/workshops', workshop);
    return response.data;
  }

  // ========================================================================
  // Resource Management Methods
  // ========================================================================

  /**
   * Get water management data
   */
  async getWaterManagement(farmId: string): Promise<types.WaterManagement> {
    const response = await this.api.get(`/farms/${farmId}/water`);
    return response.data;
  }

  /**
   * Update water management
   */
  async updateWaterManagement(farmId: string, data: Partial<types.WaterManagement>): Promise<types.WaterManagement> {
    const response = await this.api.patch(`/farms/${farmId}/water`, data);
    return response.data;
  }

  /**
   * Get soil health data
   */
  async getSoilHealth(farmId: string): Promise<types.SoilHealth> {
    const response = await this.api.get(`/farms/${farmId}/soil`);
    return response.data;
  }

  /**
   * Record soil test
   */
  async recordSoilTest(test: Partial<types.SoilHealth>): Promise<types.SoilHealth> {
    const response = await this.api.post('/soil-tests', test);
    return response.data;
  }

  /**
   * Get energy usage
   */
  async getEnergyUsage(farmId: string, period?: { start: string; end: string }): Promise<types.EnergyUsage> {
    const response = await this.api.get(`/farms/${farmId}/energy`, { params: period });
    return response.data;
  }

  // ========================================================================
  // Market Methods
  // ========================================================================

  /**
   * Get market outlets
   */
  async getMarketOutlets(farmId: string): Promise<types.MarketOutlet[]> {
    const response = await this.api.get('/market/outlets', { params: { farmId } });
    return response.data;
  }

  /**
   * Add market outlet
   */
  async addMarketOutlet(outlet: Partial<types.MarketOutlet>): Promise<types.MarketOutlet> {
    const response = await this.api.post('/market/outlets', outlet);
    return response.data;
  }

  /**
   * Get CSA programs
   */
  async getCSAPrograms(farmId: string): Promise<types.CSAProgram[]> {
    const response = await this.api.get('/market/csa', { params: { farmId } });
    return response.data;
  }

  /**
   * Create CSA program
   */
  async createCSAProgram(program: Partial<types.CSAProgram>): Promise<types.CSAProgram> {
    const response = await this.api.post('/market/csa', program);
    return response.data;
  }

  /**
   * Record sale
   */
  async recordSale(sale: Partial<types.Sale>): Promise<types.Sale> {
    const response = await this.api.post('/sales', sale);
    return response.data;
  }

  /**
   * Get sales history
   */
  async getSales(farmId: string, startDate?: string, endDate?: string): Promise<types.Sale[]> {
    const response = await this.api.get('/sales', { params: { farmId, startDate, endDate } });
    return response.data;
  }

  // ========================================================================
  // Sustainability Methods
  // ========================================================================

  /**
   * Get sustainability metrics
   */
  async getSustainabilityMetrics(farmId: string, period?: { start: string; end: string }): Promise<types.SustainabilityMetrics> {
    const response = await this.api.get(`/farms/${farmId}/sustainability`, { params: period });
    return response.data;
  }

  /**
   * Get biodiversity assessment
   */
  async getBiodiversity(farmId: string): Promise<types.Biodiversity> {
    const response = await this.api.get(`/farms/${farmId}/biodiversity`);
    return response.data;
  }

  /**
   * Update biodiversity data
   */
  async updateBiodiversity(farmId: string, data: Partial<types.Biodiversity>): Promise<types.Biodiversity> {
    const response = await this.api.patch(`/farms/${farmId}/biodiversity`, data);
    return response.data;
  }
}

/**
 * Utility functions for urban farming
 */
export class UrbanFarmingUtils {
  /**
   * Calculate yield per square meter
   */
  static calculateYieldPerSqm(totalYield: number, area: number): number {
    return totalYield / area;
  }

  /**
   * Estimate water needs
   */
  static estimateWaterNeeds(area: number, cropType: string, method: types.GrowingMethod): number {
    const baseWaterPerSqm = 5; // liters per day
    const methodMultiplier = {
      soil: 1.0,
      hydroponics: 0.7,
      aeroponics: 0.5,
      aquaponics: 0.6,
      container: 0.8,
    };
    return area * baseWaterPerSqm * methodMultiplier[method];
  }

  /**
   * Calculate profitability
   */
  static calculateProfitability(revenue: number, costs: number): { profit: number; margin: number } {
    const profit = revenue - costs;
    const margin = (profit / revenue) * 100;
    return { profit, margin };
  }
}

export default UrbanFarmingClient;
