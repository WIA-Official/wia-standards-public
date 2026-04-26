/**
 * WIA-AGRI-025: Food Traceability Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @packageDocumentation
 * @module wia-food-traceability
 * @license MIT
 * @author WIA / SmileStory Inc.
 * @version 1.0.0
 */

import {
  TraceableProduct,
  TraceEvent,
  TraceChain,
  QRCodeData,
  BlockchainRecord,
  VerifiableCredential,
  ApiResponse,
  TraceabilityQuery,
  Supplier,
  Facility,
  BatchInfo,
  RecallNotice,
  ComplianceReport,
  PaginatedResponse,
} from './types';

export * from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface TraceabilityConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  retries?: number;
  debug?: boolean;
}

// ============================================================================
// SDK Client
// ============================================================================

export class FoodTraceabilityClient {
  private config: Required<TraceabilityConfig>;
  private headers: Record<string, string>;

  constructor(config: TraceabilityConfig) {
    this.config = {
      timeout: 30000,
      retries: 3,
      debug: false,
      ...config,
    };

    this.headers = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'X-WIA-Standard': 'AGRI-025',
      'X-WIA-Version': '1.0.0',
    };
  }

  // ==========================================================================
  // Product APIs
  // ==========================================================================

  async registerProduct(product: Omit<TraceableProduct, 'productId'>): Promise<ApiResponse<TraceableProduct>> {
    return this.post<TraceableProduct>('/api/v1/products', product);
  }

  async getProduct(productId: string): Promise<ApiResponse<TraceableProduct>> {
    return this.get<TraceableProduct>(`/api/v1/products/${productId}`);
  }

  async updateProduct(productId: string, updates: Partial<TraceableProduct>): Promise<ApiResponse<TraceableProduct>> {
    return this.put<TraceableProduct>(`/api/v1/products/${productId}`, updates);
  }

  async searchProducts(query: TraceabilityQuery): Promise<ApiResponse<TraceableProduct[]>> {
    const queryString = this.buildQueryParams(query);
    return this.get<TraceableProduct[]>(`/api/v1/products${queryString}`);
  }

  // ==========================================================================
  // Trace Event APIs
  // ==========================================================================

  async addTraceEvent(event: Omit<TraceEvent, 'eventId'>): Promise<ApiResponse<TraceEvent>> {
    return this.post<TraceEvent>('/api/v1/events', event);
  }

  async getTraceEvent(eventId: string): Promise<ApiResponse<TraceEvent>> {
    return this.get<TraceEvent>(`/api/v1/events/${eventId}`);
  }

  async getProductEvents(productId: string): Promise<ApiResponse<TraceEvent[]>> {
    return this.get<TraceEvent[]>(`/api/v1/products/${productId}/events`);
  }

  async searchEvents(query?: TraceabilityQuery): Promise<ApiResponse<TraceEvent[]>> {
    const queryString = this.buildQueryParams(query);
    return this.get<TraceEvent[]>(`/api/v1/events${queryString}`);
  }

  // ==========================================================================
  // Trace Chain APIs
  // ==========================================================================

  async getTraceChain(productId: string): Promise<ApiResponse<TraceChain>> {
    return this.get<TraceChain>(`/api/v1/products/${productId}/trace-chain`);
  }

  async verifyTraceChain(productId: string): Promise<ApiResponse<{ valid: boolean; issues: string[] }>> {
    return this.post<{ valid: boolean; issues: string[] }>(`/api/v1/products/${productId}/verify`, {});
  }

  // ==========================================================================
  // QR Code APIs
  // ==========================================================================

  async generateQRCode(productId: string): Promise<ApiResponse<QRCodeData>> {
    return this.post<QRCodeData>(`/api/v1/products/${productId}/qr-code`, {});
  }

  async scanQRCode(qrId: string): Promise<ApiResponse<TraceChain>> {
    return this.get<TraceChain>(`/api/v1/qr/${qrId}/trace`);
  }

  async regenerateQRCode(productId: string): Promise<ApiResponse<QRCodeData>> {
    return this.post<QRCodeData>(`/api/v1/products/${productId}/qr-code/regenerate`, {});
  }

  // ==========================================================================
  // Blockchain APIs
  // ==========================================================================

  async getBlockchainRecord(transactionHash: string): Promise<ApiResponse<BlockchainRecord>> {
    return this.get<BlockchainRecord>(`/api/v1/blockchain/${transactionHash}`);
  }

  async anchorToBlockchain(productId: string): Promise<ApiResponse<BlockchainRecord>> {
    return this.post<BlockchainRecord>(`/api/v1/products/${productId}/anchor`, {});
  }

  // ==========================================================================
  // Credential APIs
  // ==========================================================================

  async issueCredential(productId: string): Promise<ApiResponse<VerifiableCredential>> {
    return this.post<VerifiableCredential>(`/api/v1/products/${productId}/credential`, {});
  }

  async verifyCredential(credential: VerifiableCredential): Promise<ApiResponse<{ valid: boolean; details: Record<string, unknown> }>> {
    return this.post<{ valid: boolean; details: Record<string, unknown> }>('/api/v1/credentials/verify', credential);
  }

  // ==========================================================================
  // Supplier APIs
  // ==========================================================================

  async registerSupplier(supplier: Omit<Supplier, 'supplierId'>): Promise<ApiResponse<Supplier>> {
    return this.post<Supplier>('/api/v1/suppliers', supplier);
  }

  async getSupplier(supplierId: string): Promise<ApiResponse<Supplier>> {
    return this.get<Supplier>(`/api/v1/suppliers/${supplierId}`);
  }

  async listSuppliers(): Promise<ApiResponse<Supplier[]>> {
    return this.get<Supplier[]>('/api/v1/suppliers');
  }

  // ==========================================================================
  // Recall APIs
  // ==========================================================================

  async initiateRecall(recall: Omit<RecallNotice, 'recallId'>): Promise<ApiResponse<RecallNotice>> {
    return this.post<RecallNotice>('/api/v1/recalls', recall);
  }

  async getAffectedProducts(recallId: string): Promise<ApiResponse<TraceableProduct[]>> {
    return this.get<TraceableProduct[]>(`/api/v1/recalls/${recallId}/products`);
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
    if (this.config.debug) {
      console.log(`[WIA Food Traceability] ${method} ${url}`);
    }

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
        error: error instanceof Error ? error.message : 'Request failed',
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
// Factory Function
// ============================================================================

export function createClient(config: TraceabilityConfig): FoodTraceabilityClient {
  return new FoodTraceabilityClient(config);
}

// ============================================================================
// Constants
// ============================================================================

export const VERSION = '1.0.0';

export const EVENT_TYPES = {
  HARVEST: 'harvest',
  PROCESSING: 'processing',
  PACKAGING: 'packaging',
  SHIPPING: 'shipping',
  RECEIVING: 'receiving',
  RETAIL: 'retail',
} as const;

export const PRODUCT_CATEGORIES = {
  PRODUCE: 'produce',
  MEAT: 'meat',
  DAIRY: 'dairy',
  SEAFOOD: 'seafood',
  GRAINS: 'grains',
  PROCESSED: 'processed',
} as const;

export default FoodTraceabilityClient;
