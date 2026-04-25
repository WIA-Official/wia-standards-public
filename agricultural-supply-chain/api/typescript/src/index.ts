/**
 * WIA Agricultural Supply Chain Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @packageDocumentation
 * @module wia-agricultural-supply-chain
 * @license MIT
 * @author WIA / SmileStory Inc.
 * @version 1.0.0
 */

import {
  Shipment,
  ColdChainData,
  ProvenanceRecord,
  QualityControl,
  ApiResponse,
  SupplyChainQuery,
  Farm,
  Crop,
  Harvest,
  PaginatedResponse,
} from './types';

export * from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface SupplyChainConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  retries?: number;
  debug?: boolean;
}

// ============================================================================
// SDK Client
// ============================================================================

export class SupplyChainClient {
  private config: Required<SupplyChainConfig>;
  private headers: Record<string, string>;

  constructor(config: SupplyChainConfig) {
    this.config = {
      timeout: 30000,
      retries: 3,
      debug: false,
      ...config,
    };

    this.headers = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'X-WIA-Standard': 'AGRI-SUPPLY-CHAIN',
      'X-WIA-Version': '1.0.0',
    };
  }

  // ==========================================================================
  // Farm APIs
  // ==========================================================================

  async createFarm(farm: Omit<Farm, 'farmId'>): Promise<ApiResponse<Farm>> {
    return this.post<Farm>('/api/v1/farms', farm);
  }

  async getFarm(farmId: string): Promise<ApiResponse<Farm>> {
    return this.get<Farm>(`/api/v1/farms/${farmId}`);
  }

  async listFarms(query?: SupplyChainQuery): Promise<ApiResponse<Farm[]>> {
    const queryString = this.buildQueryParams(query);
    return this.get<Farm[]>(`/api/v1/farms${queryString}`);
  }

  // ==========================================================================
  // Crop APIs
  // ==========================================================================

  async registerCrop(farmId: string, crop: Omit<Crop, 'cropId'>): Promise<ApiResponse<Crop>> {
    return this.post<Crop>(`/api/v1/farms/${farmId}/crops`, crop);
  }

  async getCrop(cropId: string): Promise<ApiResponse<Crop>> {
    return this.get<Crop>(`/api/v1/crops/${cropId}`);
  }

  // ==========================================================================
  // Harvest APIs
  // ==========================================================================

  async recordHarvest(harvest: Omit<Harvest, 'harvestId'>): Promise<ApiResponse<Harvest>> {
    return this.post<Harvest>('/api/v1/harvests', harvest);
  }

  async getHarvest(harvestId: string): Promise<ApiResponse<Harvest>> {
    return this.get<Harvest>(`/api/v1/harvests/${harvestId}`);
  }

  // ==========================================================================
  // Shipment APIs
  // ==========================================================================

  async getShipment(shipmentId: string): Promise<ApiResponse<Shipment>> {
    return this.get<Shipment>(`/api/v1/shipments/${shipmentId}`);
  }

  async createShipment(shipment: Omit<Shipment, 'shipmentId'>): Promise<ApiResponse<Shipment>> {
    return this.post<Shipment>('/api/v1/shipments', shipment);
  }

  async updateShipment(shipmentId: string, updates: Partial<Shipment>): Promise<ApiResponse<Shipment>> {
    return this.put<Shipment>(`/api/v1/shipments/${shipmentId}`, updates);
  }

  async listShipments(query?: SupplyChainQuery): Promise<ApiResponse<Shipment[]>> {
    const queryString = this.buildQueryParams(query);
    return this.get<Shipment[]>(`/api/v1/shipments${queryString}`);
  }

  // ==========================================================================
  // Cold Chain APIs
  // ==========================================================================

  async submitColdChainData(data: Omit<ColdChainData, 'dataId'>): Promise<ApiResponse<ColdChainData>> {
    return this.post<ColdChainData>('/api/v1/cold-chain', data);
  }

  async getColdChainHistory(shipmentId: string): Promise<ApiResponse<ColdChainData[]>> {
    return this.get<ColdChainData[]>(`/api/v1/shipments/${shipmentId}/cold-chain`);
  }

  // ==========================================================================
  // Provenance APIs
  // ==========================================================================

  async addProvenanceRecord(record: Omit<ProvenanceRecord, 'recordId'>): Promise<ApiResponse<ProvenanceRecord>> {
    return this.post<ProvenanceRecord>('/api/v1/provenance', record);
  }

  async getProvenance(shipmentId: string): Promise<ApiResponse<ProvenanceRecord[]>> {
    return this.get<ProvenanceRecord[]>(`/api/v1/shipments/${shipmentId}/provenance`);
  }

  // ==========================================================================
  // Quality Control APIs
  // ==========================================================================

  async submitQualityControl(qc: Omit<QualityControl, 'qcId'>): Promise<ApiResponse<QualityControl>> {
    return this.post<QualityControl>('/api/v1/quality-control', qc);
  }

  async getQualityControlRecords(shipmentId: string): Promise<ApiResponse<QualityControl[]>> {
    return this.get<QualityControl[]>(`/api/v1/shipments/${shipmentId}/quality-control`);
  }

  // ==========================================================================
  // HTTP Methods
  // ==========================================================================

  private async get<T>(path: string): Promise<ApiResponse<T>> {
    return this.request<T>('GET', path);
  }

  private async post<T>(path: string, data: unknown): Promise<ApiResponse<T>> {
    return this.request<T>('POST', path, data);
  }

  private async put<T>(path: string, data: unknown): Promise<ApiResponse<T>> {
    return this.request<T>('PUT', path, data);
  }

  private async request<T>(method: string, path: string, data?: unknown): Promise<ApiResponse<T>> {
    const url = `${this.config.endpoint}${path}`;
    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), this.config.timeout);

    try {
      const response = await fetch(url, {
        method,
        headers: this.headers,
        body: data ? JSON.stringify(data) : undefined,
        signal: controller.signal,
      });
      clearTimeout(timeoutId);
      return response.json();
    } catch (error) {
      clearTimeout(timeoutId);
      return {
        success: false,
        error: error instanceof Error ? error.message : 'Unknown error',
        timestamp: new Date().toISOString(),
      };
    }
  }

  private buildQueryParams(params?: Record<string, unknown>): string {
    if (!params) return '';
    const searchParams = new URLSearchParams();
    Object.entries(params).forEach(([key, value]) => {
      if (value !== undefined) searchParams.append(key, String(value));
    });
    return searchParams.toString() ? `?${searchParams.toString()}` : '';
  }
}

// ============================================================================
// Constants
// ============================================================================

export const VERSION = '1.0.0';

export const COLD_CHAIN_THRESHOLDS = {
  FROZEN: { min: -25, max: -18 },
  CHILLED: { min: 0, max: 4 },
  COOL: { min: 8, max: 15 },
  AMBIENT: { min: 15, max: 25 },
} as const;

export default SupplyChainClient;
