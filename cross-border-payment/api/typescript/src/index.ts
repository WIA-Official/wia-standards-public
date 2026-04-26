/**
 * WIA-FIN-014 Cross-Border Payment SDK
 *
 * Official TypeScript SDK for the WIA-FIN-014 Cross-Border Payment Standard
 *
 * @module @wia/cross-border-payment-sdk
 * @version 1.0.0
 *
 * @example
 * ```typescript
 * import { CrossBorderPaymentClient } from '@wia/cross-border-payment-sdk';
 *
 * const client = new CrossBorderPaymentClient({
 *   apiKey: 'your-api-key',
 *   environment: 'production'
 * });
 *
 * const payment = await client.createPayment({
 *   beneficiaryId: 'BEN-123',
 *   amount: 1000,
 *   currency: 'USD',
 *   purpose: 'Family support'
 * });
 * ```
 */

import axios, { AxiosInstance, AxiosError } from 'axios';
import crypto from 'crypto';
import * as types from './types';

// Re-export all types
export * from './types';

// ============================================================================
// Main Client
// ============================================================================

export class CrossBorderPaymentClient {
  private axiosInstance: AxiosInstance;
  private config: types.ClientConfig;

  constructor(config: types.ClientConfig) {
    this.config = config;

    const baseURL = config.baseURL || (
      config.environment === 'production'
        ? 'https://api.wia.org/fin-014/v1'
        : 'https://sandbox-api.wia.org/fin-014/v1'
    );

    this.axiosInstance = axios.create({
      baseURL,
      timeout: config.timeout || 30000,
      headers: {
        'Authorization': `Bearer ${config.apiKey}`,
        'Content-Type': 'application/json',
        'User-Agent': '@wia/cross-border-payment-sdk/1.0.0'
      }
    });

    // Add retry logic
    this.setupRetryInterceptor();
  }

  // ==========================================================================
  // Payment Operations
  // ==========================================================================

  /**
   * Create a cross-border payment
   *
   * @param request - Payment details
   * @returns Created payment
   *
   * @example
   * ```typescript
   * const payment = await client.createPayment({
   *   beneficiaryId: 'BEN-123',
   *   amount: 1000,
   *   currency: 'USD',
   *   purpose: 'Family support'
   * });
   * ```
   */
  async createPayment(request: types.CreatePaymentRequest): Promise<types.Payment> {
    const response = await this.axiosInstance.post<types.ApiResponse<types.Payment>>(
      '/payments',
      request
    );
    return response.data.data;
  }

  /**
   * Get payment status by ID
   *
   * @param paymentId - Payment ID
   * @returns Payment details
   */
  async getPayment(paymentId: string): Promise<types.Payment> {
    const response = await this.axiosInstance.get<types.ApiResponse<types.Payment>>(
      `/payments/${paymentId}`
    );
    return response.data.data;
  }

  /**
   * List payments with optional filters
   *
   * @param params - Filter parameters
   * @returns Paginated payment list
   */
  async listPayments(params?: {
    status?: types.PaymentStatus;
    from?: string;
    to?: string;
    page?: number;
    pageSize?: number;
  }): Promise<types.PaginatedResponse<types.Payment>> {
    const response = await this.axiosInstance.get<types.PaginatedResponse<types.Payment>>(
      '/payments',
      { params }
    );
    return response.data;
  }

  // ==========================================================================
  // Beneficiary Operations
  // ==========================================================================

  /**
   * Create a beneficiary
   *
   * @param request - Beneficiary details
   * @returns Created beneficiary
   */
  async createBeneficiary(request: types.CreateBeneficiaryRequest): Promise<types.Beneficiary> {
    const response = await this.axiosInstance.post<types.ApiResponse<types.Beneficiary>>(
      '/beneficiaries',
      request
    );
    return response.data.data;
  }

  /**
   * Get beneficiary by ID
   *
   * @param beneficiaryId - Beneficiary ID
   * @returns Beneficiary details
   */
  async getBeneficiary(beneficiaryId: string): Promise<types.Beneficiary> {
    const response = await this.axiosInstance.get<types.ApiResponse<types.Beneficiary>>(
      `/beneficiaries/${beneficiaryId}`
    );
    return response.data.data;
  }

  /**
   * List beneficiaries
   *
   * @returns List of beneficiaries
   */
  async listBeneficiaries(): Promise<types.Beneficiary[]> {
    const response = await this.axiosInstance.get<types.ApiResponse<types.Beneficiary[]>>(
      '/beneficiaries'
    );
    return response.data.data;
  }

  // ==========================================================================
  // FX & Quote Operations
  // ==========================================================================

  /**
   * Get FX exchange rate
   *
   * @param from - Source currency
   * @param to - Destination currency
   * @returns Exchange rate
   */
  async getFxRate(from: types.Currency, to: types.Currency): Promise<types.ExchangeRate> {
    const response = await this.axiosInstance.get<types.ApiResponse<types.ExchangeRate>>(
      '/fx/rates',
      { params: { from, to } }
    );
    return response.data.data;
  }

  /**
   * Get payment quote
   *
   * @param request - Quote request
   * @returns Payment quote with fees
   */
  async getQuote(request: types.QuoteRequest): Promise<types.Quote> {
    const response = await this.axiosInstance.post<types.ApiResponse<types.Quote>>(
      '/quotes',
      request
    );
    return response.data.data;
  }

  /**
   * Lock FX rate for future payment
   *
   * @param request - Rate lock request
   * @returns Rate lock details
   */
  async lockFxRate(request: types.RateLockRequest): Promise<types.RateLock> {
    const response = await this.axiosInstance.post<types.ApiResponse<types.RateLock>>(
      '/fx/lock-rate',
      request
    );
    return response.data.data;
  }

  // ==========================================================================
  // Batch Operations
  // ==========================================================================

  /**
   * Create batch payment
   *
   * @param request - Batch payment request
   * @returns Batch payment details
   */
  async createBatchPayment(request: types.BatchPaymentRequest): Promise<types.BatchPayment> {
    const response = await this.axiosInstance.post<types.ApiResponse<types.BatchPayment>>(
      '/batch-payments',
      request
    );
    return response.data.data;
  }

  /**
   * Get batch payment status
   *
   * @param batchId - Batch ID
   * @returns Batch payment details
   */
  async getBatchPayment(batchId: string): Promise<types.BatchPayment> {
    const response = await this.axiosInstance.get<types.ApiResponse<types.BatchPayment>>(
      `/batch-payments/${batchId}`
    );
    return response.data.data;
  }

  // ==========================================================================
  // Compliance Operations
  // ==========================================================================

  /**
   * Check compliance for a payment
   *
   * @param request - Compliance check request
   * @returns Compliance check result
   */
  async checkCompliance(request: types.ComplianceCheckRequest): Promise<types.ComplianceCheck> {
    const response = await this.axiosInstance.post<types.ApiResponse<types.ComplianceCheck>>(
      '/compliance/check',
      request
    );
    return response.data.data;
  }

  // ==========================================================================
  // Corridor & Route Operations
  // ==========================================================================

  /**
   * Get available payment corridors
   *
   * @returns List of corridors
   */
  async getCorridors(): Promise<types.PaymentCorridor[]> {
    const response = await this.axiosInstance.get<types.ApiResponse<types.PaymentCorridor[]>>(
      '/corridors'
    );
    return response.data.data;
  }

  /**
   * Get available payment routes for a transaction
   *
   * @param from - Source country
   * @param to - Destination country
   * @param amount - Payment amount
   * @param currency - Payment currency
   * @returns Available routes
   */
  async getRoutes(
    from: types.Country,
    to: types.Country,
    amount: number,
    currency: types.Currency
  ): Promise<types.PaymentRoute[]> {
    const response = await this.axiosInstance.get<types.ApiResponse<types.PaymentRoute[]>>(
      '/routing/options',
      { params: { from, to, amount, currency } }
    );
    return response.data.data;
  }

  // ==========================================================================
  // Webhook Operations
  // ==========================================================================

  /**
   * Verify webhook signature
   *
   * @param payload - Webhook payload
   * @param signature - Signature from header
   * @param secret - Webhook secret
   * @returns True if signature is valid
   */
  verifyWebhookSignature(
    payload: any,
    signature: string,
    secret: string
  ): boolean {
    const expectedSignature = crypto
      .createHmac('sha256', secret)
      .update(JSON.stringify(payload))
      .digest('hex');

    const providedSignature = signature.replace('sha256=', '');

    return crypto.timingSafeEqual(
      Buffer.from(expectedSignature),
      Buffer.from(providedSignature)
    );
  }

  // ==========================================================================
  // Private Helper Methods
  // ==========================================================================

  private setupRetryInterceptor(): void {
    const maxRetries = this.config.retries || 3;
    let retryCount = 0;

    this.axiosInstance.interceptors.response.use(
      (response) => response,
      async (error: AxiosError) => {
        const config = error.config;

        if (!config || retryCount >= maxRetries) {
          return Promise.reject(this.handleError(error));
        }

        // Retry on network errors and 5xx errors
        if (!error.response || error.response.status >= 500) {
          retryCount++;
          const delay = Math.pow(2, retryCount) * 1000; // Exponential backoff

          await new Promise(resolve => setTimeout(resolve, delay));
          return this.axiosInstance.request(config);
        }

        return Promise.reject(this.handleError(error));
      }
    );
  }

  private handleError(error: AxiosError): types.WiaApiError {
    if (error.response) {
      const data = error.response.data as any;
      return new types.WiaApiError(
        data.type || 'UNKNOWN_ERROR',
        error.response.status,
        data.detail || error.message,
        data
      );
    }

    return new types.WiaApiError(
      'NETWORK_ERROR',
      0,
      error.message
    );
  }
}

// ============================================================================
// Default Export
// ============================================================================

export default CrossBorderPaymentClient;
