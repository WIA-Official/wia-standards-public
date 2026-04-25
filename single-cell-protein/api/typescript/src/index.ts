/**
 * WIA-AGRI-034 Single Cell Protein Standard - TypeScript SDK
 * @module @wia/single-cell-protein
 */

import axios, { AxiosInstance } from 'axios';
import * as types from './types';

export * from './types';

/**
 * Main Single Cell Protein Client
 */
export class SingleCellProteinClient {
  private api: AxiosInstance;
  private config: types.ClientConfig;

  constructor(config: types.ClientConfig) {
    this.config = {
      baseURL: 'https://api.wia-scp.io/v1',
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
  // Production Methods
  // ========================================================================

  async getProductions(organism?: types.OrganismType, status?: types.ProductionStatus): Promise<types.SCPProduction[]> {
    const response = await this.api.get('/productions', { params: { organism, status } });
    return response.data;
  }

  async getProduction(productionId: string): Promise<types.SCPProduction> {
    const response = await this.api.get(`/productions/${productionId}`);
    return response.data;
  }

  async createProduction(production: Partial<types.SCPProduction>): Promise<types.SCPProduction> {
    const response = await this.api.post('/productions', production);
    return response.data;
  }

  async updateProduction(productionId: string, updates: Partial<types.SCPProduction>): Promise<types.SCPProduction> {
    const response = await this.api.patch(`/productions/${productionId}`, updates);
    return response.data;
  }

  // ========================================================================
  // Process Monitoring Methods
  // ========================================================================

  async recordProcessMetrics(metrics: Partial<types.ProcessMetrics>): Promise<types.ProcessMetrics> {
    const response = await this.api.post('/process/metrics', metrics);
    return response.data;
  }

  async getProcessMetrics(productionId: string, startTime?: number, endTime?: number): Promise<types.ProcessMetrics[]> {
    const response = await this.api.get(`/productions/${productionId}/metrics`, { params: { startTime, endTime } });
    return response.data;
  }

  async getFermentationParameters(productionId: string): Promise<types.FermentationParameters> {
    const response = await this.api.get(`/productions/${productionId}/parameters`);
    return response.data;
  }

  async updateFermentationParameters(productionId: string, params: Partial<types.FermentationParameters>): Promise<types.FermentationParameters> {
    const response = await this.api.patch(`/productions/${productionId}/parameters`, params);
    return response.data;
  }

  // ========================================================================
  // Bioreactor Methods
  // ========================================================================

  async getBioreactors(type?: types.ReactorType): Promise<types.Bioreactor[]> {
    const response = await this.api.get('/bioreactors', { params: { type } });
    return response.data;
  }

  async getBioreactor(reactorId: string): Promise<types.Bioreactor> {
    const response = await this.api.get(`/bioreactors/${reactorId}`);
    return response.data;
  }

  async updateBioreactor(reactorId: string, updates: Partial<types.Bioreactor>): Promise<types.Bioreactor> {
    const response = await this.api.patch(`/bioreactors/${reactorId}`, updates);
    return response.data;
  }

  // ========================================================================
  // Downstream Processing Methods
  // ========================================================================

  async createDownstreamProcess(process: Partial<types.DownstreamProcess>): Promise<types.DownstreamProcess> {
    const response = await this.api.post('/downstream', process);
    return response.data;
  }

  async getDownstreamProcess(processId: string): Promise<types.DownstreamProcess> {
    const response = await this.api.get(`/downstream/${processId}`);
    return response.data;
  }

  async getFinalProduct(productId: string): Promise<types.FinalProduct> {
    const response = await this.api.get(`/products/${productId}`);
    return response.data;
  }

  // ========================================================================
  // Quality Control Methods
  // ========================================================================

  async getQualityMetrics(productId: string): Promise<types.QualityMetrics> {
    const response = await this.api.get(`/products/${productId}/quality`);
    return response.data;
  }

  async recordQualityTest(productId: string, tests: Partial<types.QualityMetrics>): Promise<types.QualityMetrics> {
    const response = await this.api.post(`/products/${productId}/quality`, tests);
    return response.data;
  }

  async getNutritionalProfile(productId: string): Promise<types.NutritionalProfile> {
    const response = await this.api.get(`/products/${productId}/nutrition`);
    return response.data;
  }

  // ========================================================================
  // Application Methods
  // ========================================================================

  async getApplications(sector?: types.ApplicationSector): Promise<types.SCPApplication[]> {
    const response = await this.api.get('/applications', { params: { sector } });
    return response.data;
  }

  async createApplication(application: Partial<types.SCPApplication>): Promise<types.SCPApplication> {
    const response = await this.api.post('/applications', application);
    return response.data;
  }

  async getRegulatoryStatus(applicationId: string, region: string): Promise<types.RegulatoryInfo> {
    const response = await this.api.get(`/applications/${applicationId}/regulatory/${region}`);
    return response.data;
  }

  // ========================================================================
  // Sustainability Methods
  // ========================================================================

  async getLifeCycleAssessment(productionId: string): Promise<types.LifeCycleAssessment> {
    const response = await this.api.get(`/productions/${productionId}/lca`);
    return response.data;
  }

  async compareEnvironmentalImpact(productionId: string, comparisonProducts: string[]): Promise<types.ComparisonData[]> {
    const response = await this.api.post(`/productions/${productionId}/compare`, { comparisonProducts });
    return response.data;
  }
}

/**
 * Utility functions for SCP production
 */
export class SCPUtils {
  static calculateProteinYield(biomassKg: number, proteinPercent: number): number {
    return (biomassKg * proteinPercent) / 100;
  }

  static calculateYieldCoefficient(biomassProduced: number, substrateConsumed: number): number {
    return biomassProduced / substrateConsumed;
  }

  static estimateProductionCost(substrate: number, energy: number, labor: number, overhead: number): number {
    return substrate + energy + labor + overhead;
  }

  static calculateProteinEfficiency(proteinOut: number, proteinIn: number): number {
    return (proteinOut / proteinIn) * 100;
  }
}

export default SingleCellProteinClient;
