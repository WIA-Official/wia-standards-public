/**
 * WIA-AGRI-032 Food Waste Reduction Standard - TypeScript SDK
 * @module @wia/food-waste-reduction
 */

import axios, { AxiosInstance } from 'axios';
import * as types from './types';

export * from './types';

/**
 * Main Food Waste Reduction Client
 */
export class FoodWasteReductionClient {
  private api: AxiosInstance;
  private config: types.ClientConfig;

  constructor(config: types.ClientConfig) {
    this.config = {
      baseURL: 'https://api.wia-wastereduction.io/v1',
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
  // Waste Tracking Methods
  // ========================================================================

  async recordWaste(waste: Partial<types.FoodWaste>): Promise<types.FoodWaste> {
    const response = await this.api.post('/waste', waste);
    return response.data;
  }

  async getWaste(startDate?: string, endDate?: string, source?: types.WasteSource): Promise<types.FoodWaste[]> {
    const response = await this.api.get('/waste', { params: { startDate, endDate, source } });
    return response.data;
  }

  async getWasteAnalytics(startDate: string, endDate: string): Promise<types.WasteAnalytics> {
    const response = await this.api.get('/analytics', { params: { startDate, endDate } });
    return response.data;
  }

  // ========================================================================
  // Prevention Program Methods
  // ========================================================================

  async getPreventionPrograms(type?: types.PreventionProgram['type']): Promise<types.PreventionProgram[]> {
    const response = await this.api.get('/programs', { params: { type } });
    return response.data;
  }

  async createPreventionProgram(program: Partial<types.PreventionProgram>): Promise<types.PreventionProgram> {
    const response = await this.api.post('/programs', program);
    return response.data;
  }

  async updateProgramMetrics(programId: string, metrics: types.ProgramMetrics): Promise<types.PreventionProgram> {
    const response = await this.api.patch(`/programs/${programId}/metrics`, metrics);
    return response.data;
  }

  // ========================================================================
  // Donation Methods
  // ========================================================================

  async createDonation(donation: Partial<types.FoodDonation>): Promise<types.FoodDonation> {
    const response = await this.api.post('/donations', donation);
    return response.data;
  }

  async getDonations(orgId?: string, status?: types.FoodDonation['status']): Promise<types.FoodDonation[]> {
    const response = await this.api.get('/donations', { params: { orgId, status } });
    return response.data;
  }

  async updateDonationStatus(donationId: string, status: types.FoodDonation['status']): Promise<types.FoodDonation> {
    const response = await this.api.patch(`/donations/${donationId}`, { status });
    return response.data;
  }

  async registerOrganization(org: Partial<types.Organization>): Promise<types.Organization> {
    const response = await this.api.post('/organizations', org);
    return response.data;
  }

  // ========================================================================
  // Smart Storage Methods
  // ========================================================================

  async getSmartStorage(storageId: string): Promise<types.SmartStorage> {
    const response = await this.api.get(`/storage/${storageId}`);
    return response.data;
  }

  async getInventory(storageId: string): Promise<types.InventoryItem[]> {
    const response = await this.api.get(`/storage/${storageId}/inventory`);
    return response.data;
  }

  async addInventoryItem(storageId: string, item: Partial<types.InventoryItem>): Promise<types.InventoryItem> {
    const response = await this.api.post(`/storage/${storageId}/inventory`, item);
    return response.data;
  }

  async getStorageAlerts(storageId: string, severity?: string): Promise<types.StorageAlert[]> {
    const response = await this.api.get(`/storage/${storageId}/alerts`, { params: { severity } });
    return response.data;
  }

  // ========================================================================
  // Upcycling Methods
  // ========================================================================

  async getUpcyclingProjects(status?: string): Promise<types.UpcyclingProject[]> {
    const response = await this.api.get('/upcycling/projects', { params: { status } });
    return response.data;
  }

  async createUpcyclingProject(project: Partial<types.UpcyclingProject>): Promise<types.UpcyclingProject> {
    const response = await this.api.post('/upcycling/projects', project);
    return response.data;
  }

  async getCompostBatches(status?: string): Promise<types.Compost[]> {
    const response = await this.api.get('/composting/batches', { params: { status } });
    return response.data;
  }

  async createCompostBatch(batch: Partial<types.Compost>): Promise<types.Compost> {
    const response = await this.api.post('/composting/batches', batch);
    return response.data;
  }
}

/**
 * Utility functions for waste reduction
 */
export class WasteReductionUtils {
  static calculateCO2Impact(wasteKg: number): number {
    return wasteKg * 2.5; // Average 2.5 kg CO2e per kg of food waste
  }

  static calculateWaterFootprint(wasteKg: number, category: types.FoodCategory): number {
    const waterPerKg: Record<types.FoodCategory, number> = {
      fruits: 900,
      vegetables: 300,
      grains: 1600,
      dairy: 1000,
      meat: 15000,
      prepared_food: 800,
      bakery: 1200,
      other: 500,
    };
    return wasteKg * waterPerKg[category];
  }

  static estimateDonationValue(items: types.DonationItem[]): number {
    return items.reduce((total, item) => total + item.value, 0);
  }
}

export default FoodWasteReductionClient;
