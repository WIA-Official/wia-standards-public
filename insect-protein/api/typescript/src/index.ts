/**
 * WIA-AGRI-025: Insect Protein Standard - TypeScript SDK
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

export interface WIAInsectProteinConfig {
  apiKey: string;
  endpoint?: string;
  timeout?: number;
  debug?: boolean;
}

// ============================================================================
// Main SDK Client
// ============================================================================

export class WIAInsectProteinClient {
  private config: Required<WIAInsectProteinConfig>;
  private eventEmitter = new EventEmitter();

  constructor(config: WIAInsectProteinConfig) {
    this.config = {
      endpoint: 'https://api.wia-standards.org/v1/insect-protein',
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

  async createFarm(farm: Omit<Types.InsectFarm, 'farmId' | 'createdAt' | 'updatedAt'>): Promise<Types.InsectFarm> {
    return this.makeRequest('POST', '/farms', farm);
  }

  async getFarm(farmId: string): Promise<Types.InsectFarm> {
    return this.makeRequest('GET', `/farms/${farmId}`);
  }

  async updateFarm(farmId: string, updates: Partial<Types.InsectFarm>): Promise<Types.InsectFarm> {
    return this.makeRequest('PUT', `/farms/${farmId}`, updates);
  }

  async deleteFarm(farmId: string): Promise<void> {
    return this.makeRequest('DELETE', `/farms/${farmId}`);
  }

  async listFarms(filters?: { species?: Types.InsectSpecies; country?: string }): Promise<Types.InsectFarm[]> {
    const params = new URLSearchParams(filters as Record<string, string>);
    return this.makeRequest('GET', `/farms?${params}`);
  }

  // ==========================================================================
  // Species Operations
  // ==========================================================================

  async addSpecies(farmId: string, species: Omit<Types.SpeciesInfo, 'speciesId'>): Promise<Types.SpeciesInfo> {
    return this.makeRequest('POST', `/farms/${farmId}/species`, species);
  }

  async getSpecies(farmId: string, speciesId: string): Promise<Types.SpeciesInfo> {
    return this.makeRequest('GET', `/farms/${farmId}/species/${speciesId}`);
  }

  async listSpecies(farmId: string): Promise<Types.SpeciesInfo[]> {
    return this.makeRequest('GET', `/farms/${farmId}/species`);
  }

  async updateSpecies(farmId: string, speciesId: string, updates: Partial<Types.SpeciesInfo>): Promise<Types.SpeciesInfo> {
    return this.makeRequest('PATCH', `/farms/${farmId}/species/${speciesId}`, updates);
  }

  // ==========================================================================
  // Production Operations
  // ==========================================================================

  async getProduction(farmId: string): Promise<Types.ProductionSystem> {
    return this.makeRequest('GET', `/farms/${farmId}/production`);
  }

  async recordHarvest(farmId: string, harvest: {
    speciesId: string;
    quantityKg: number;
    batchId: string;
    harvestDate: string;
  }): Promise<void> {
    return this.makeRequest('POST', `/farms/${farmId}/production/harvest`, harvest);
  }

  async getHarvestHistory(farmId: string, startDate: string, endDate: string): Promise<Types.HarvestRecord[]> {
    const params = new URLSearchParams({ startDate, endDate });
    return this.makeRequest('GET', `/farms/${farmId}/production/history?${params}`);
  }

  // ==========================================================================
  // Quality Control
  // ==========================================================================

  async getQualityControl(farmId: string): Promise<Types.QualityControl> {
    return this.makeRequest('GET', `/farms/${farmId}/quality`);
  }

  async submitQualityTest(farmId: string, test: {
    batchId: string;
    testType: string;
    results: Record<string, unknown>;
  }): Promise<void> {
    return this.makeRequest('POST', `/farms/${farmId}/quality/tests`, test);
  }

  async getCertifications(farmId: string): Promise<Types.Certification[]> {
    return this.makeRequest('GET', `/farms/${farmId}/quality/certifications`);
  }

  // ==========================================================================
  // Sustainability Metrics
  // ==========================================================================

  async getSustainabilityMetrics(farmId: string): Promise<Types.SustainabilityMetrics> {
    return this.makeRequest('GET', `/farms/${farmId}/sustainability`);
  }

  async getResourceEfficiency(farmId: string): Promise<Types.ResourceEfficiency> {
    return this.makeRequest('GET', `/farms/${farmId}/efficiency`);
  }

  async getCarbonFootprint(farmId: string): Promise<{ co2PerKg: number; totalEmissions: number }> {
    return this.makeRequest('GET', `/farms/${farmId}/sustainability/carbon`);
  }

  // ==========================================================================
  // Products
  // ==========================================================================

  async getProducts(farmId: string): Promise<Types.InsectProduct[]> {
    return this.makeRequest('GET', `/farms/${farmId}/products`);
  }

  async createProduct(farmId: string, product: Omit<Types.InsectProduct, 'productId'>): Promise<Types.InsectProduct> {
    return this.makeRequest('POST', `/farms/${farmId}/products`, product);
  }

  async getProductNutrition(farmId: string, productId: string): Promise<Types.NutritionInfo> {
    return this.makeRequest('GET', `/farms/${farmId}/products/${productId}/nutrition`);
  }

  // ==========================================================================
  // Event Handling
  // ==========================================================================

  on(event: 'farmCreated' | 'harvestCompleted' | 'qualityAlert' | 'certificationExpiring' | 'error', callback: (...args: unknown[]) => void): void {
    this.eventEmitter.on(event, callback);
  }

  // ==========================================================================
  // Private Methods
  // ==========================================================================

  private async makeRequest<T>(method: string, path: string, body?: unknown): Promise<T> {
    const url = `${this.config.endpoint}${path}`;

    if (this.config.debug) {
      console.log(`[WIA Insect Protein] ${method} ${url}`, body);
    }

    try {
      const response = await fetch(url, {
        method,
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${this.config.apiKey}`,
          'X-WIA-Standard': 'AGRI-033',
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

export function createClient(config: WIAInsectProteinConfig): WIAInsectProteinClient {
  return new WIAInsectProteinClient(config);
}

export default WIAInsectProteinClient;
