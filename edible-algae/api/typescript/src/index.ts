/**
 * WIA-AGRI-033 Edible Algae Standard - TypeScript SDK
 * @module @wia/edible-algae
 */

import axios, { AxiosInstance } from 'axios';
import * as types from './types';

export * from './types';

/**
 * Main Edible Algae Client
 */
export class EdibleAlgaeClient {
  private api: AxiosInstance;
  private config: types.ClientConfig;

  constructor(config: types.ClientConfig) {
    this.config = {
      baseURL: 'https://api.wia-algae.io/v1',
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
  // Culture Management Methods
  // ========================================================================

  async getCultures(facilityId?: string, species?: types.AlgaeSpecies): Promise<types.AlgaeCulture[]> {
    const response = await this.api.get('/cultures', { params: { facilityId, species } });
    return response.data;
  }

  async getCulture(cultureId: string): Promise<types.AlgaeCulture> {
    const response = await this.api.get(`/cultures/${cultureId}`);
    return response.data;
  }

  async createCulture(culture: Partial<types.AlgaeCulture>): Promise<types.AlgaeCulture> {
    const response = await this.api.post('/cultures', culture);
    return response.data;
  }

  async updateCulture(cultureId: string, updates: Partial<types.AlgaeCulture>): Promise<types.AlgaeCulture> {
    const response = await this.api.patch(`/cultures/${cultureId}`, updates);
    return response.data;
  }

  // ========================================================================
  // Growth Monitoring Methods
  // ========================================================================

  async recordGrowthMetrics(metrics: Partial<types.GrowthMetrics>): Promise<types.GrowthMetrics> {
    const response = await this.api.post('/growth/metrics', metrics);
    return response.data;
  }

  async getGrowthMetrics(cultureId: string, startTime?: number, endTime?: number): Promise<types.GrowthMetrics[]> {
    const response = await this.api.get(`/cultures/${cultureId}/growth`, { params: { startTime, endTime } });
    return response.data;
  }

  async calculateGrowthRate(cultureId: string): Promise<types.GrowthRate> {
    const response = await this.api.get(`/cultures/${cultureId}/growth-rate`);
    return response.data;
  }

  // ========================================================================
  // Harvest Methods
  // ========================================================================

  async recordHarvest(harvest: Partial<types.Harvest>): Promise<types.Harvest> {
    const response = await this.api.post('/harvests', harvest);
    return response.data;
  }

  async getHarvests(cultureId?: string, startDate?: string, endDate?: string): Promise<types.Harvest[]> {
    const response = await this.api.get('/harvests', { params: { cultureId, startDate, endDate } });
    return response.data;
  }

  async getQualityMetrics(harvestId: string): Promise<types.QualityMetrics> {
    const response = await this.api.get(`/harvests/${harvestId}/quality`);
    return response.data;
  }

  // ========================================================================
  // Product Methods
  // ========================================================================

  async getProducts(species?: types.AlgaeSpecies, application?: string): Promise<types.AlgaeProduct[]> {
    const response = await this.api.get('/products', { params: { species, application } });
    return response.data;
  }

  async createProduct(product: Partial<types.AlgaeProduct>): Promise<types.AlgaeProduct> {
    const response = await this.api.post('/products', product);
    return response.data;
  }

  async updateProduct(productId: string, updates: Partial<types.AlgaeProduct>): Promise<types.AlgaeProduct> {
    const response = await this.api.patch(`/products/${productId}`, updates);
    return response.data;
  }

  // ========================================================================
  // Sustainability Methods
  // ========================================================================

  async getSustainabilityMetrics(facilityId: string, period?: { start: string; end: string }): Promise<types.SustainabilityMetrics> {
    const response = await this.api.get(`/facilities/${facilityId}/sustainability`, { params: period });
    return response.data;
  }

  async getBioremediationData(facilityId: string): Promise<types.BioremedationData> {
    const response = await this.api.get(`/facilities/${facilityId}/bioremediation`);
    return response.data;
  }

  // ========================================================================
  // Research Methods
  // ========================================================================

  async getStrainImprovements(status?: string): Promise<types.StrainImprovement[]> {
    const response = await this.api.get('/research/strains', { params: { status } });
    return response.data;
  }

  async createExperiment(experiment: Partial<types.Experiment>): Promise<types.Experiment> {
    const response = await this.api.post('/research/experiments', experiment);
    return response.data;
  }

  async getExperiments(status?: string): Promise<types.Experiment[]> {
    const response = await this.api.get('/research/experiments', { params: { status } });
    return response.data;
  }

  async recordExperimentResults(experimentId: string, results: types.ExperimentResults): Promise<types.Experiment> {
    const response = await this.api.patch(`/research/experiments/${experimentId}/results`, results);
    return response.data;
  }

  // ========================================================================
  // Facility Methods
  // ========================================================================

  async getFacilities(type?: types.CultivationType): Promise<types.CultivationFacility[]> {
    const response = await this.api.get('/facilities', { params: { type } });
    return response.data;
  }

  async getFacility(facilityId: string): Promise<types.CultivationFacility> {
    const response = await this.api.get(`/facilities/${facilityId}`);
    return response.data;
  }
}

/**
 * Utility functions for algae cultivation
 */
export class AlgaeUtils {
  static calculateBiomassYield(volume: number, density: number, dryWeightPercent: number): number {
    return (volume * density * dryWeightPercent) / 100;
  }

  static calculateProteinYield(biomassKg: number, proteinPercent: number): number {
    return (biomassKg * proteinPercent) / 100;
  }

  static estimateCO2Sequestration(biomassKg: number): number {
    return biomassKg * 1.8; // Algae sequesters ~1.8 kg CO2 per kg biomass
  }

  static calculateDoublingTime(specificGrowthRate: number): number {
    return Math.log(2) / specificGrowthRate;
  }
}

export default EdibleAlgaeClient;
