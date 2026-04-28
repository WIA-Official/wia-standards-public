/**
 * WIA-ENE-041: Rare Earth Mining Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

import {
  ApiResponse,
  BatchSubmissionRequest,
  BatchSubmissionResponse,
  MineRegistrationRequest,
  MineRegistrationResponse,
  ProductionBatch,
  REEPrice,
  REEMarketIndex,
  SupplyChainTracking,
  ProductionStatistics,
  MarketReport,
} from './types';

export * from './types';

// ============================================================================
// Client Configuration
// ============================================================================

/**
 * Client configuration options
 */
export interface REEClientConfig {
  apiKey: string;
  endpoint?: string;
  timeout?: number;
  retries?: number;
}

/**
 * Default configuration
 */
const DEFAULT_CONFIG = {
  endpoint: 'https://api.wia.org/ree-041/v1',
  timeout: 30000,
  retries: 3,
};

// ============================================================================
// REE Mining Client
// ============================================================================

/**
 * WIA-ENE-041 Rare Earth Mining API Client
 */
export class REEClient {
  private config: Required<REEClientConfig>;

  constructor(config: REEClientConfig) {
    this.config = {
      ...DEFAULT_CONFIG,
      ...config,
    };
  }

  /**
   * Make HTTP request
   */
  private async request<T>(
    method: string,
    path: string,
    data?: any
  ): Promise<ApiResponse<T>> {
    const url = `${this.config.endpoint}${path}`;
    const headers: Record<string, string> = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
    };

    try {
      const response = await fetch(url, {
        method,
        headers,
        body: data ? JSON.stringify(data) : undefined,
      });

      const result = await response.json();
      return result as ApiResponse<T>;
    } catch (error: any) {
      return {
        success: false,
        error: {
          code: 'REQUEST_FAILED',
          message: error.message,
          details: error,
        },
      };
    }
  }

  // ========================================================================
  // Mine Management
  // ========================================================================

  /**
   * Register a new rare earth mine
   */
  async registerMine(
    request: MineRegistrationRequest
  ): Promise<ApiResponse<MineRegistrationResponse>> {
    return this.request<MineRegistrationResponse>('POST', '/mines/register', request);
  }

  /**
   * Get mine information
   */
  async getMine(mineId: string): Promise<ApiResponse<any>> {
    return this.request<any>('GET', `/mines/${mineId}`);
  }

  /**
   * Update mine information
   */
  async updateMine(mineId: string, updates: Partial<MineRegistrationRequest>): Promise<ApiResponse<any>> {
    return this.request<any>('PUT', `/mines/${mineId}`, updates);
  }

  /**
   * List mines
   */
  async listMines(filters?: {
    country?: string;
    oreType?: string;
    status?: string;
    limit?: number;
    offset?: number;
  }): Promise<ApiResponse<any[]>> {
    const params = new URLSearchParams(filters as any);
    return this.request<any[]>('GET', `/mines?${params}`);
  }

  // ========================================================================
  // Batch Production
  // ========================================================================

  /**
   * Submit production batch
   */
  async submitBatch(
    request: BatchSubmissionRequest
  ): Promise<ApiResponse<BatchSubmissionResponse>> {
    return this.request<BatchSubmissionResponse>('POST', '/batches/submit', request);
  }

  /**
   * Get batch information
   */
  async getBatch(batchId: string): Promise<ApiResponse<ProductionBatch>> {
    return this.request<ProductionBatch>('GET', `/batches/${batchId}`);
  }

  /**
   * List batches
   */
  async listBatches(filters?: {
    mineId?: string;
    startDate?: string;
    endDate?: string;
    limit?: number;
    offset?: number;
  }): Promise<ApiResponse<ProductionBatch[]>> {
    const params = new URLSearchParams(filters as any);
    return this.request<ProductionBatch[]>('GET', `/batches?${params}`);
  }

  // ========================================================================
  // Supply Chain Tracking
  // ========================================================================

  /**
   * Track supply chain
   */
  async trackSupplyChain(batchId: string): Promise<ApiResponse<SupplyChainTracking>> {
    return this.request<SupplyChainTracking>('GET', `/batches/${batchId}/supply-chain`);
  }

  /**
   * Verify batch authenticity
   */
  async verifyBatch(batchId: string): Promise<ApiResponse<{
    verified: boolean;
    blockchainTx: string;
    certifications: string[];
  }>> {
    return this.request('GET', `/batches/${batchId}/verify`);
  }

  // ========================================================================
  // Pricing
  // ========================================================================

  /**
   * Get current REE prices
   */
  async getCurrentPrices(): Promise<ApiResponse<REEPrice[]>> {
    return this.request<REEPrice[]>('GET', '/prices/current');
  }

  /**
   * Get price for specific element
   */
  async getElementPrice(element: string): Promise<ApiResponse<REEPrice>> {
    return this.request<REEPrice>('GET', `/prices/${element}`);
  }

  /**
   * Get price history
   */
  async getPriceHistory(
    element: string,
    startDate: string,
    endDate: string
  ): Promise<ApiResponse<REEPrice[]>> {
    const params = new URLSearchParams({ startDate, endDate });
    return this.request<REEPrice[]>('GET', `/prices/${element}/history?${params}`);
  }

  /**
   * Get market indices
   */
  async getMarketIndices(): Promise<ApiResponse<REEMarketIndex[]>> {
    return this.request<REEMarketIndex[]>('GET', '/market/indices');
  }

  // ========================================================================
  // Statistics & Reporting
  // ========================================================================

  /**
   * Get production statistics
   */
  async getProductionStatistics(filters: {
    startDate: string;
    endDate: string;
    mineId?: string;
    country?: string;
    region?: string;
  }): Promise<ApiResponse<ProductionStatistics>> {
    const params = new URLSearchParams(filters as any);
    return this.request<ProductionStatistics>('GET', `/statistics/production?${params}`);
  }

  /**
   * Get market report
   */
  async getMarketReport(date?: string): Promise<ApiResponse<MarketReport>> {
    const params = date ? `?date=${date}` : '';
    return this.request<MarketReport>('GET', `/market/report${params}`);
  }

  /**
   * Get environmental impact report
   */
  async getEnvironmentalReport(filters: {
    startDate: string;
    endDate: string;
    mineId?: string;
  }): Promise<ApiResponse<any>> {
    const params = new URLSearchParams(filters as any);
    return this.request('GET', `/reports/environmental?${params}`);
  }

  // ========================================================================
  // Utilities
  // ========================================================================

  /**
   * Estimate batch value
   */
  async estimateBatchValue(composition: any): Promise<ApiResponse<{
    totalValue_USD: number;
    byElement: Record<string, number>;
  }>> {
    return this.request('POST', '/utilities/estimate-value', { composition });
  }

  /**
   * Find nearby processing facilities
   */
  async findNearbyFacilities(lat: number, lon: number, radius_km: number = 100): Promise<ApiResponse<any[]>> {
    const params = new URLSearchParams({
      lat: lat.toString(),
      lon: lon.toString(),
      radius: radius_km.toString(),
    });
    return this.request<any[]>('GET', `/utilities/nearby-facilities?${params}`);
  }

  /**
   * Validate radioactive safety
   */
  async validateRadioactiveSafety(data: {
    thorium_ppm: number;
    uranium_ppm: number;
  }): Promise<ApiResponse<{
    level: string;
    compliant: boolean;
    recommendations: string[];
  }>> {
    return this.request('POST', '/utilities/validate-radioactive-safety', data);
  }
}

// ============================================================================
// WebSocket Client
// ============================================================================

/**
 * Real-time price feed client
 */
export class REEPriceFeed {
  private ws: WebSocket | null = null;
  private endpoint: string;
  private apiKey: string;
  private callbacks: Map<string, (data: any) => void> = new Map();

  constructor(config: { endpoint?: string; apiKey: string }) {
    this.endpoint = config.endpoint || 'wss://api.wia.org/v1/ree-041/prices/stream';
    this.apiKey = config.apiKey;
  }

  /**
   * Connect to WebSocket
   */
  connect(): Promise<void> {
    return new Promise((resolve, reject) => {
      this.ws = new WebSocket(`${this.endpoint}?apiKey=${this.apiKey}`);

      this.ws.onopen = () => {
        console.log('Connected to REE price feed');
        resolve();
      };

      this.ws.onerror = (error) => {
        console.error('WebSocket error:', error);
        reject(error);
      };

      this.ws.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          this.handleMessage(data);
        } catch (error) {
          console.error('Failed to parse message:', error);
        }
      };
    });
  }

  /**
   * Subscribe to element price updates
   */
  subscribe(element: string, callback: (price: REEPrice) => void): void {
    this.callbacks.set(element, callback);
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify({
        action: 'subscribe',
        element,
      }));
    }
  }

  /**
   * Unsubscribe from element price updates
   */
  unsubscribe(element: string): void {
    this.callbacks.delete(element);
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify({
        action: 'unsubscribe',
        element,
      }));
    }
  }

  /**
   * Handle incoming message
   */
  private handleMessage(data: any): void {
    if (data.element && this.callbacks.has(data.element)) {
      const callback = this.callbacks.get(data.element);
      if (callback) {
        callback(data);
      }
    }
  }

  /**
   * Disconnect
   */
  disconnect(): void {
    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }
  }
}

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * Calculate TREO (Total Rare Earth Oxide) percentage
 */
export function calculateTREO(composition: any): number {
  const keys = Object.keys(composition).filter(key => key.endsWith('_percent'));
  return keys.reduce((sum, key) => sum + (composition[key] || 0), 0);
}

/**
 * Classify REE category (LREE/HREE)
 */
export function classifyREE(element: string): 'LREE' | 'HREE' | 'ASSOCIATED' {
  const lree = ['La', 'Ce', 'Pr', 'Nd', 'Pm', 'Sm', 'Eu'];
  const hree = ['Gd', 'Tb', 'Dy', 'Ho', 'Er', 'Tm', 'Yb', 'Lu'];

  if (lree.includes(element)) return 'LREE';
  if (hree.includes(element)) return 'HREE';
  return 'ASSOCIATED';
}

/**
 * Determine radioactive waste level
 */
export function determineRadioactiveLevel(totalRadioactivity_Bq_g: number): string {
  if (totalRadioactivity_Bq_g < 100) return 'LOW_LEVEL';
  if (totalRadioactivity_Bq_g < 10000) return 'INTERMEDIATE_LEVEL';
  return 'HIGH_LEVEL';
}

/**
 * Format price with currency
 */
export function formatPrice(price: number, currency: string = 'USD'): string {
  return new Intl.NumberFormat('en-US', {
    style: 'currency',
    currency,
  }).format(price);
}

/**
 * Calculate price change percentage
 */
export function calculatePriceChange(oldPrice: number, newPrice: number): number {
  return ((newPrice - oldPrice) / oldPrice) * 100;
}

// ============================================================================
// Export default client
// ============================================================================

export default REEClient;
