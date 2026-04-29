/**
 * WIA Energy Cloud Standard - TypeScript SDK
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';

export * from './types';
import type {
  APIConfig, WIAEnergyCloud, PlatformResponse, EnergyAsset, VirtualPowerPlant,
  TradePosition, Settlement, ForecastAccuracy, ValidationResult, PaginatedResponse,
} from './types';

export class WIAEnergyCloudClient {
  private axios: AxiosInstance;

  constructor(config: APIConfig) {
    this.axios = axios.create({
      baseURL: config.baseURL,
      timeout: config.timeout || 30000,
      headers: { 'Content-Type': 'application/json', ...(config.apiKey && { Authorization: `Bearer ${config.apiKey}` }) },
    });
  }

  async createPlatform(platform: WIAEnergyCloud): Promise<PlatformResponse> {
    const response = await this.axios.post<PlatformResponse>('/platforms', platform);
    return response.data;
  }

  async getPlatform(id: string): Promise<WIAEnergyCloud> {
    const response = await this.axios.get<WIAEnergyCloud>(`/platforms/${id}`);
    return response.data;
  }

  async listPlatforms(params?: { type?: string; status?: string; limit?: number }): Promise<PaginatedResponse<PlatformResponse>> {
    const response = await this.axios.get<PaginatedResponse<PlatformResponse>>('/platforms', { params });
    return response.data;
  }

  async updatePlatform(id: string, updates: Partial<WIAEnergyCloud>): Promise<PlatformResponse> {
    const response = await this.axios.put<PlatformResponse>(`/platforms/${id}`, updates);
    return response.data;
  }

  async registerAsset(platformId: string, asset: Omit<EnergyAsset, 'id'>): Promise<EnergyAsset> {
    const response = await this.axios.post<EnergyAsset>(`/platforms/${platformId}/assets`, asset);
    return response.data;
  }

  async listAssets(platformId: string, params?: { type?: string; status?: string }): Promise<EnergyAsset[]> {
    const response = await this.axios.get<EnergyAsset[]>(`/platforms/${platformId}/assets`, { params });
    return response.data;
  }

  async getAssetStatus(platformId: string, assetId: string): Promise<{ status: string; output: number; soc?: number }> {
    const response = await this.axios.get(`/platforms/${platformId}/assets/${assetId}/status`);
    return response.data;
  }

  async controlAsset(platformId: string, assetId: string, command: { action: string; value?: number }): Promise<{ success: boolean }> {
    const response = await this.axios.post(`/platforms/${platformId}/assets/${assetId}/control`, command);
    return response.data;
  }

  async createVPP(platformId: string, vpp: Omit<VirtualPowerPlant, 'id' | 'performance'>): Promise<VirtualPowerPlant> {
    const response = await this.axios.post<VirtualPowerPlant>(`/platforms/${platformId}/vpps`, vpp);
    return response.data;
  }

  async listVPPs(platformId: string): Promise<VirtualPowerPlant[]> {
    const response = await this.axios.get<VirtualPowerPlant[]>(`/platforms/${platformId}/vpps`);
    return response.data;
  }

  async getVPPStatus(platformId: string, vppId: string): Promise<{ status: string; capacity: any; dispatching: any }> {
    const response = await this.axios.get(`/platforms/${platformId}/vpps/${vppId}/status`);
    return response.data;
  }

  async dispatchVPP(platformId: string, vppId: string, dispatch: { power: number; duration: number; type: string }): Promise<{ dispatchId: string; status: string }> {
    const response = await this.axios.post(`/platforms/${platformId}/vpps/${vppId}/dispatch`, dispatch);
    return response.data;
  }

  async submitBid(platformId: string, marketId: string, bid: Omit<TradePosition, 'id' | 'status' | 'timestamp'>): Promise<TradePosition> {
    const response = await this.axios.post<TradePosition>(`/platforms/${platformId}/markets/${marketId}/bids`, bid);
    return response.data;
  }

  async listPositions(platformId: string, marketId: string): Promise<TradePosition[]> {
    const response = await this.axios.get<TradePosition[]>(`/platforms/${platformId}/markets/${marketId}/positions`);
    return response.data;
  }

  async getMarketPrices(platformId: string, marketId: string, params?: { from?: string; to?: string }): Promise<{ prices: { timestamp: string; price: number }[] }> {
    const response = await this.axios.get(`/platforms/${platformId}/markets/${marketId}/prices`, { params });
    return response.data;
  }

  async getForecast(platformId: string, type: string, params?: { horizon?: number; resolution?: number }): Promise<{ forecast: { timestamp: string; value: number }[] }> {
    const response = await this.axios.get(`/platforms/${platformId}/forecasts/${type}`, { params });
    return response.data;
  }

  async getForecastAccuracy(platformId: string, modelId: string): Promise<ForecastAccuracy> {
    const response = await this.axios.get<ForecastAccuracy>(`/platforms/${platformId}/forecasts/models/${modelId}/accuracy`);
    return response.data;
  }

  async runOptimization(platformId: string, params?: { horizon?: number; objectives?: string[] }): Promise<{ optimizationId: string; schedule: any[] }> {
    const response = await this.axios.post(`/platforms/${platformId}/optimization/run`, params);
    return response.data;
  }

  async getOptimizationResult(platformId: string, optimizationId: string): Promise<{ status: string; schedule: any[]; savings: number }> {
    const response = await this.axios.get(`/platforms/${platformId}/optimization/${optimizationId}`);
    return response.data;
  }

  async getSettlements(platformId: string, params?: { period?: string; status?: string }): Promise<Settlement[]> {
    const response = await this.axios.get<Settlement[]>(`/platforms/${platformId}/billing/settlements`, { params });
    return response.data;
  }

  async generateInvoice(platformId: string, settlementId: string): Promise<{ invoiceId: string; url: string }> {
    const response = await this.axios.post(`/platforms/${platformId}/billing/settlements/${settlementId}/invoice`);
    return response.data;
  }

  async getEnergyBalance(platformId: string): Promise<{ generation: number; consumption: number; storage: number; gridExchange: number }> {
    const response = await this.axios.get(`/platforms/${platformId}/balance`);
    return response.data;
  }

  async getDashboardMetrics(platformId: string): Promise<{ assets: any; vpp: any; market: any; financial: any }> {
    const response = await this.axios.get(`/platforms/${platformId}/dashboard`);
    return response.data;
  }

  validateEnergyCloud(energyCloud: WIAEnergyCloud): ValidationResult {
    const errors: { path: string; message: string }[] = [];
    if (energyCloud.standard !== 'WIA-ENERGY-CLOUD') errors.push({ path: 'standard', message: 'Invalid standard' });
    if (!energyCloud.platform?.id) errors.push({ path: 'platform.id', message: 'Platform ID required' });
    if (!energyCloud.platform?.operator) errors.push({ path: 'platform.operator', message: 'Operator info required' });
    return { valid: errors.length === 0, errors: errors.length > 0 ? errors : undefined };
  }
}

export function generateUUID(): string {
  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => {
    const r = (Math.random() * 16) | 0;
    return (c === 'x' ? r : (r & 0x3) | 0x8).toString(16);
  });
}

export function createMinimalPlatform(name: string, type: string = 'community'): WIAEnergyCloud {
  return {
    standard: 'WIA-ENERGY-CLOUD',
    version: '1.0.0',
    platform: {
      id: generateUUID(), name, type: type as any, status: 'initializing', createdAt: new Date().toISOString(),
      operator: { id: generateUUID(), name: 'Platform Operator', type: 'aggregator', contact: { email: 'contact@example.com' } },
      coverage: { regions: [], gridZones: [], totalCapacity: 0, assetCount: 0, unit: 'MW' },
      capabilities: [{ type: 'demand-response', enabled: true }],
    },
    assets: [],
    virtualPowerPlants: [],
    markets: [],
    optimization: {
      enabled: true, algorithm: 'milp', objectives: [{ type: 'cost-minimization', weight: 1.0 }],
      constraints: [], horizon: { value: 24, unit: 'hours' }, resolution: 15,
    },
    forecasting: {
      enabled: true, models: [], dataSources: [],
      accuracy: { mae: 0, rmse: 0, mape: 0, lastEvaluated: new Date().toISOString() },
    },
    billing: { model: 'net-metering', tariffs: [], incentives: [], settlements: [] },
  };
}

export default { WIAEnergyCloudClient, generateUUID, createMinimalPlatform };
