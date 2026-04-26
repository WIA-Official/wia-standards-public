/**
 * WIA-UNI-007: Power Grid Unification Standard - TypeScript SDK
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

export interface WIAPowerGridConfig {
  apiKey: string;
  endpoint?: string;
  region?: string;
  timeout?: number;
  debug?: boolean;
}

// ============================================================================
// Main SDK Client
// ============================================================================

export class WIAPowerGridSDK {
  private config: Required<WIAPowerGridConfig>;
  private eventEmitter = new EventEmitter();

  constructor(config: WIAPowerGridConfig) {
    this.config = {
      endpoint: 'https://api.wia-standards.org/v1/power-grid',
      region: 'global',
      timeout: 30000,
      debug: false,
      ...config,
    };

    if (!this.config.apiKey) {
      throw new Error('API key is required');
    }
  }

  // ==========================================================================
  // Grid Node Operations
  // ==========================================================================

  async listNodes(params?: {
    type?: Types.NodeType;
    region?: string;
    status?: Types.OperationalStatus;
  }): Promise<Types.APIResponse<Types.PowerGridNode[]>> {
    return this.makeRequest('GET', '/nodes', params);
  }

  async getNode(nodeId: string): Promise<Types.APIResponse<Types.PowerGridNode>> {
    return this.makeRequest('GET', `/nodes/${nodeId}`);
  }

  async createNode(node: Omit<Types.PowerGridNode, 'id' | '@context' | '@type'>): Promise<Types.APIResponse<Types.PowerGridNode>> {
    const payload: Types.PowerGridNode = {
      '@context': 'https://wiastandards.com/contexts/uni-007/v1',
      '@type': 'PowerGridNode',
      id: `wia:grid:${Date.now()}`,
      ...node,
    };
    return this.makeRequest('POST', '/nodes', payload);
  }

  async updateNode(nodeId: string, updates: Partial<Types.PowerGridNode>): Promise<Types.APIResponse<Types.PowerGridNode>> {
    return this.makeRequest('PATCH', `/nodes/${nodeId}`, updates);
  }

  async deleteNode(nodeId: string): Promise<Types.APIResponse<void>> {
    return this.makeRequest('DELETE', `/nodes/${nodeId}`);
  }

  // ==========================================================================
  // Power Flow Operations
  // ==========================================================================

  async getPowerFlow(nodeId: string): Promise<Types.APIResponse<Types.PowerFlowMeasurement>> {
    return this.makeRequest('GET', `/power-flow/${nodeId}`);
  }

  async getPowerFlowHistory(nodeId: string, start: string, end: string, interval: string = '5m'): Promise<Types.APIResponse<Types.PowerFlowMeasurement[]>> {
    return this.makeRequest('GET', `/power-flow/${nodeId}/history`, { start, end, interval });
  }

  async getGridStatus(): Promise<Types.APIResponse<{
    totalCapacity: number;
    currentLoad: number;
    utilizationPercent: number;
    frequency: number;
    stability: 'stable' | 'warning' | 'critical';
  }>> {
    return this.makeRequest('GET', '/status');
  }

  // ==========================================================================
  // Renewable Energy Operations
  // ==========================================================================

  async getRenewableMetrics(sourceId: string): Promise<Types.APIResponse<Types.RenewableEnergyMetrics>> {
    return this.makeRequest('GET', `/renewable/${sourceId}`);
  }

  async listRenewableSources(filters?: {
    type?: 'solar' | 'wind' | 'hydro' | 'geothermal';
    region?: string;
  }): Promise<Types.APIResponse<Types.RenewableSource[]>> {
    return this.makeRequest('GET', '/renewable', filters);
  }

  async getRenewableGeneration(region?: string): Promise<Types.APIResponse<{
    totalMW: number;
    byType: Record<string, number>;
    percentOfLoad: number;
  }>> {
    return this.makeRequest('GET', '/renewable/generation', { region });
  }

  // ==========================================================================
  // Energy Trading Operations
  // ==========================================================================

  async submitTradingOrder(order: Types.TradingOrder): Promise<Types.APIResponse<{ orderId: string; status: string }>> {
    return this.makeRequest('POST', '/trading/orders', order);
  }

  async getOrder(orderId: string): Promise<Types.APIResponse<Types.TradingOrder>> {
    return this.makeRequest('GET', `/trading/orders/${orderId}`);
  }

  async cancelOrder(orderId: string): Promise<Types.APIResponse<void>> {
    return this.makeRequest('DELETE', `/trading/orders/${orderId}`);
  }

  async getMarketPrices(params?: {
    interval?: 'hourly' | 'daily';
    start?: string;
    region?: string;
  }): Promise<Types.APIResponse<Types.MarketPrice[]>> {
    return this.makeRequest('GET', '/trading/prices', params);
  }

  // ==========================================================================
  // Forecasting Operations
  // ==========================================================================

  async getLoadForecast(region: string, horizon: string = '24h'): Promise<Types.APIResponse<Types.LoadForecast>> {
    return this.makeRequest('GET', '/forecasts/load', { region, horizon });
  }

  async getRenewableForecast(sourceId: string, horizon: string = '24h'): Promise<Types.APIResponse<Types.RenewableForecast>> {
    return this.makeRequest('GET', `/forecasts/renewable/${sourceId}`, { horizon });
  }

  async getPriceForecast(region: string, horizon: string = '24h'): Promise<Types.APIResponse<Types.PriceForecast>> {
    return this.makeRequest('GET', '/forecasts/price', { region, horizon });
  }

  // ==========================================================================
  // Event Handling
  // ==========================================================================

  on(event: 'nodeStatusChange' | 'priceUpdate' | 'gridAlert' | 'tradeExecuted' | 'error', callback: (...args: unknown[]) => void): void {
    this.eventEmitter.on(event, callback);
  }

  // ==========================================================================
  // Private Methods
  // ==========================================================================

  private async makeRequest<T>(method: string, path: string, body?: unknown): Promise<T> {
    const url = `${this.config.endpoint}${path}`;

    if (this.config.debug) {
      console.log(`[WIA Power Grid] ${method} ${url}`, body);
    }

    try {
      const response = await fetch(url, {
        method,
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${this.config.apiKey}`,
          'X-WIA-Standard': 'UNI-007',
          'X-WIA-Version': '1.0.0',
          'X-WIA-Region': this.config.region,
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

export function createPowerGridSDK(config: WIAPowerGridConfig): WIAPowerGridSDK {
  return new WIAPowerGridSDK(config);
}

export default WIAPowerGridSDK;
