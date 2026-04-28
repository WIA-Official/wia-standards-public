/**
 * WIA-ENE-023: Recycling System Standard - TypeScript SDK
 *
 * @version 1.0.0
 * @license CC BY 4.0
 * @description Client library for interacting with WIA-ENE-023 recycling API
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 */

import axios, { AxiosInstance, AxiosError } from 'axios';
import type {
  // Core types
  RecyclingEvent,
  Facility,
  RecycledProduct,

  // Request/Response types
  CreateEventRequest,
  CreateEventResponse,
  EventQueryParams,
  PaginatedResponse,
  UpdatePerformanceRequest,
  TraceChain,

  // Analytics types
  RecoveryRateStats,
  CarbonSavings,
  ContaminationAlert,

  // Configuration
  ClientConfig,
  APIError,

  // Webhook types
  WebhookPayload,
  WebhookEventType
} from './types';

// Re-export all types for convenience
export * from './types';

/**
 * Default API endpoint
 */
const DEFAULT_ENDPOINT = 'https://api.wia.org/ene-023/v1';

/**
 * Default timeout in milliseconds
 */
const DEFAULT_TIMEOUT = 30000;

/**
 * Default number of retries
 */
const DEFAULT_RETRIES = 3;

/**
 * Main client for interacting with WIA-ENE-023 Recycling API
 *
 * @example
 * ```typescript
 * const client = new RecyclingClient({
 *   apiKey: 'your-api-key',
 *   endpoint: 'https://api.wia.org/ene-023/v1'
 * });
 *
 * const event = await client.submitEvent({
 *   facilityId: 'FAC-MRF-001',
 *   materials: [...]
 * });
 * ```
 */
export class RecyclingClient {
  private axios: AxiosInstance;
  private apiKey: string;
  private retries: number;

  constructor(config: ClientConfig) {
    this.apiKey = config.apiKey;
    this.retries = config.retries ?? DEFAULT_RETRIES;

    this.axios = axios.create({
      baseURL: config.endpoint ?? DEFAULT_ENDPOINT,
      timeout: config.timeout ?? DEFAULT_TIMEOUT,
      headers: {
        'Content-Type': 'application/json',
        'X-API-Key': this.apiKey,
        'User-Agent': 'WIA-ENE-023-SDK/1.0.0'
      }
    });

    // Add response interceptor for error handling
    this.axios.interceptors.response.use(
      response => response,
      error => this.handleError(error)
    );
  }

  /**
   * Handle API errors and convert them to APIError format
   */
  private handleError(error: AxiosError): Promise<never> {
    if (error.response) {
      const apiError: APIError = {
        code: error.response.status.toString(),
        message: error.message,
        details: error.response.data,
        timestamp: new Date().toISOString()
      };
      return Promise.reject(apiError);
    }

    const apiError: APIError = {
      code: 'NETWORK_ERROR',
      message: error.message,
      timestamp: new Date().toISOString()
    };
    return Promise.reject(apiError);
  }

  /**
   * Retry a request with exponential backoff
   */
  private async retryRequest<T>(
    fn: () => Promise<T>,
    retries: number = this.retries
  ): Promise<T> {
    try {
      return await fn();
    } catch (error) {
      if (retries > 0) {
        const delay = Math.pow(2, this.retries - retries) * 1000;
        await new Promise(resolve => setTimeout(resolve, delay));
        return this.retryRequest(fn, retries - 1);
      }
      throw error;
    }
  }

  // ==================== Event Management ====================

  /**
   * Submit a new recycling event
   *
   * @param request - Event data to submit
   * @returns Created event response with event ID
   *
   * @example
   * ```typescript
   * const response = await client.submitEvent({
   *   facilityId: 'FAC-MRF-001',
   *   eventType: 'collection',
   *   materials: [
   *     {
   *       materialCode: 'PL-01',
   *       quantity: { value: 1250.5, unit: 'kg' },
   *       quality: { grade: 'A', contamination: 2.5, moisture: 0.8 },
   *       source: 'residential',
   *       batchId: 'BATCH-001'
   *     }
   *   ]
   * });
   * ```
   */
  async submitEvent(request: CreateEventRequest): Promise<CreateEventResponse> {
    return this.retryRequest(async () => {
      const response = await this.axios.post<CreateEventResponse>('/events', request);
      return response.data;
    });
  }

  /**
   * Get details of a specific recycling event
   *
   * @param eventId - The event ID to retrieve
   * @returns Full event details
   *
   * @example
   * ```typescript
   * const event = await client.getEvent('REC-2025-001234');
   * console.log(event.materials);
   * ```
   */
  async getEvent(eventId: string): Promise<RecyclingEvent> {
    const response = await this.axios.get<RecyclingEvent>(`/events/${eventId}`);
    return response.data;
  }

  /**
   * Query events with filters and pagination
   *
   * @param params - Query parameters for filtering and pagination
   * @returns Paginated list of events
   *
   * @example
   * ```typescript
   * const events = await client.queryEvents({
   *   facilityId: 'FAC-MRF-001',
   *   materialCode: 'PL-01',
   *   startDate: '2025-01-01',
   *   endDate: '2025-12-31',
   *   page: 1,
   *   limit: 50
   * });
   * ```
   */
  async queryEvents(params?: EventQueryParams): Promise<PaginatedResponse<RecyclingEvent>> {
    const response = await this.axios.get<PaginatedResponse<RecyclingEvent>>('/events', {
      params
    });
    return response.data;
  }

  // ==================== Facility Management ====================

  /**
   * Register a new recycling facility
   *
   * @param facility - Facility information
   * @returns Created facility with assigned ID
   *
   * @example
   * ```typescript
   * const facility = await client.createFacility({
   *   facilityName: '서울 자원순환센터',
   *   facilityType: 'MRF',
   *   operationalStatus: 'active',
   *   capacity: {
   *     daily: { value: 500, unit: 'tonnes' },
   *     annual: { value: 150000, unit: 'tonnes' }
   *   },
   *   ...
   * });
   * ```
   */
  async createFacility(facility: Omit<Facility, 'facilityId'>): Promise<Facility> {
    return this.retryRequest(async () => {
      const response = await this.axios.post<Facility>('/facilities', facility);
      return response.data;
    });
  }

  /**
   * Get facility information
   *
   * @param facilityId - The facility ID
   * @returns Facility details
   *
   * @example
   * ```typescript
   * const facility = await client.getFacility('FAC-MRF-001');
   * console.log(facility.performance.averageRecoveryRate);
   * ```
   */
  async getFacility(facilityId: string): Promise<Facility> {
    const response = await this.axios.get<Facility>(`/facilities/${facilityId}`);
    return response.data;
  }

  /**
   * Query facilities with filters
   *
   * @param params - Query parameters
   * @returns List of facilities
   *
   * @example
   * ```typescript
   * const facilities = await client.queryFacilities({
   *   type: 'MRF',
   *   country: 'KR',
   *   materialCode: 'PL-01'
   * });
   * ```
   */
  async queryFacilities(params?: {
    type?: string;
    country?: string;
    materialCode?: string;
  }): Promise<Facility[]> {
    const response = await this.axios.get<Facility[]>('/facilities', { params });
    return response.data;
  }

  /**
   * Update facility performance metrics
   *
   * @param facilityId - The facility ID
   * @param performance - Updated performance data
   * @returns Success status
   *
   * @example
   * ```typescript
   * await client.updateFacilityPerformance('FAC-MRF-001', {
   *   averageRecoveryRate: 88.5,
   *   averageContamination: 2.8,
   *   averageUptime: 94.2,
   *   monthlyThroughput: 12500
   * });
   * ```
   */
  async updateFacilityPerformance(
    facilityId: string,
    performance: UpdatePerformanceRequest['performance']
  ): Promise<{ success: boolean }> {
    return this.retryRequest(async () => {
      const response = await this.axios.put<{ success: boolean }>(
        `/facilities/${facilityId}/performance`,
        { performance }
      );
      return response.data;
    });
  }

  // ==================== Product Management ====================

  /**
   * Register a recycled product
   *
   * @param product - Product information
   * @returns Created product with ID
   *
   * @example
   * ```typescript
   * const product = await client.createProduct({
   *   productName: '재생 PET 섬유',
   *   manufacturer: {
   *     id: 'MFG-FIBER-001',
   *     name: '에코섬유 주식회사',
   *     country: 'KR'
   *   },
   *   composition: [...],
   *   ...
   * });
   * ```
   */
  async createProduct(product: Omit<RecycledProduct, 'productId'>): Promise<RecycledProduct> {
    return this.retryRequest(async () => {
      const response = await this.axios.post<RecycledProduct>('/products', product);
      return response.data;
    });
  }

  /**
   * Get product information
   *
   * @param productId - The product ID
   * @returns Product details
   *
   * @example
   * ```typescript
   * const product = await client.getProduct('RPROD-2025-001');
   * console.log(product.carbonFootprint);
   * ```
   */
  async getProduct(productId: string): Promise<RecycledProduct> {
    const response = await this.axios.get<RecycledProduct>(`/products/${productId}`);
    return response.data;
  }

  /**
   * Get full supply chain trace for a product
   *
   * @param productId - The product ID
   * @returns Complete trace chain
   *
   * @example
   * ```typescript
   * const trace = await client.traceProduct('RPROD-2025-001');
   * trace.chain.forEach(event => {
   *   console.log(`${event.facilityName}: ${event.eventType}`);
   * });
   * ```
   */
  async traceProduct(productId: string): Promise<TraceChain> {
    const response = await this.axios.get<TraceChain>(`/products/${productId}/trace`);
    return response.data;
  }

  // ==================== Analytics ====================

  /**
   * Get recovery rate statistics
   *
   * @param params - Query parameters
   * @returns Recovery rate statistics
   *
   * @example
   * ```typescript
   * const stats = await client.getRecoveryRate({
   *   materialCode: 'PL-01',
   *   country: 'KR',
   *   period: 'monthly'
   * });
   * console.log(`Recovery rate: ${stats.recoveryRate}%`);
   * ```
   */
  async getRecoveryRate(params: {
    materialCode?: string;
    country?: string;
    period?: 'daily' | 'weekly' | 'monthly' | 'yearly';
  }): Promise<RecoveryRateStats> {
    const response = await this.axios.get<RecoveryRateStats>('/analytics/recovery-rate', {
      params
    });
    return response.data;
  }

  /**
   * Calculate carbon savings
   *
   * @param params - Query parameters
   * @returns Carbon savings data
   *
   * @example
   * ```typescript
   * const savings = await client.getCarbonSavings({
   *   facilityId: 'FAC-MRF-001',
   *   startDate: '2025-01-01',
   *   endDate: '2025-12-31'
   * });
   * console.log(`Total savings: ${savings.totalSavings} kgCO2e`);
   * ```
   */
  async getCarbonSavings(params: {
    facilityId?: string;
    startDate: string;
    endDate: string;
  }): Promise<CarbonSavings> {
    const response = await this.axios.get<CarbonSavings>('/analytics/carbon-savings', {
      params
    });
    return response.data;
  }

  // ==================== Webhooks ====================

  /**
   * Verify webhook signature (for implementing webhook endpoints)
   *
   * @param payload - The webhook payload
   * @param signature - The signature from webhook headers
   * @param secret - Your webhook secret
   * @returns Whether the signature is valid
   */
  verifyWebhookSignature(payload: string, signature: string, secret: string): boolean {
    // In a real implementation, this would use HMAC-SHA256
    // For now, this is a placeholder
    const crypto = require('crypto');
    const expectedSignature = crypto
      .createHmac('sha256', secret)
      .update(payload)
      .digest('hex');
    return signature === expectedSignature;
  }

  /**
   * Parse webhook payload
   *
   * @param payload - Raw webhook payload string
   * @returns Parsed webhook payload
   */
  parseWebhookPayload<T = any>(payload: string): WebhookPayload<T> {
    return JSON.parse(payload) as WebhookPayload<T>;
  }
}

/**
 * Helper function to create a client instance
 *
 * @param config - Client configuration
 * @returns Configured RecyclingClient instance
 *
 * @example
 * ```typescript
 * const client = createRecyclingClient({
 *   apiKey: process.env.WIA_API_KEY!,
 *   endpoint: 'https://api.wia.org/ene-023/v1'
 * });
 * ```
 */
export function createRecyclingClient(config: ClientConfig): RecyclingClient {
  return new RecyclingClient(config);
}

/**
 * Default export
 */
export default RecyclingClient;
