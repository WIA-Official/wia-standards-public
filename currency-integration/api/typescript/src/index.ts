/**
 * WIA-UNI-013 Currency Integration SDK
 *
 * Official TypeScript SDK for inter-Korean currency integration
 *
 * @module @wia/currency-integration-sdk
 * @version 1.0.0
 */

import axios, { AxiosInstance, AxiosError } from 'axios';
import type {
  ClientConfig,
  ExchangeRate,
  GetExchangeRateRequest,
  GetHistoricalRatesRequest,
  HistoricalRate,
  ConvertCurrencyRequest,
  CurrencyConversion,
  GetConversionStatusRequest,
  CreateWalletRequest,
  CBDCWallet,
  TransferCBDCRequest,
  CBDCTransaction,
  TransactionSummaryRequest,
  TransactionSummary,
  APIResponse,
  AuthenticationError,
  ValidationError,
  NetworkError
} from './types';

export * from './types';

// ============================================================================
// Main SDK Client
// ============================================================================

export class CurrencyIntegrationClient {
  private client: AxiosInstance;
  private config: Required<ClientConfig>;

  constructor(config: ClientConfig) {
    this.config = {
      ...config,
      baseURL: config.baseURL || (
        config.environment === 'production'
          ? 'https://api.wia.org/uni-013/v1'
          : 'https://sandbox.wia.org/uni-013/v1'
      ),
      timeout: config.timeout || 30000,
      retries: config.retries || 3
    };

    this.client = axios.create({
      baseURL: this.config.baseURL,
      timeout: this.config.timeout,
      headers: {
        'Authorization': `Bearer ${this.config.apiKey}`,
        'Content-Type': 'application/json',
        'User-Agent': 'WIA-CurrencyIntegration-SDK/1.0.0'
      }
    });

    this.setupInterceptors();
  }

  private setupInterceptors(): void {
    // Request interceptor
    this.client.interceptors.request.use(
      (config) => {
        return config;
      },
      (error) => {
        return Promise.reject(this.handleError(error));
      }
    );

    // Response interceptor
    this.client.interceptors.response.use(
      (response) => response,
      async (error: AxiosError) => {
        const originalRequest = error.config as any;

        // Retry logic
        if (!originalRequest._retry && this.config.retries > 0) {
          originalRequest._retry = true;
          originalRequest._retryCount = (originalRequest._retryCount || 0) + 1;

          if (originalRequest._retryCount <= this.config.retries) {
            await this.delay(1000 * originalRequest._retryCount);
            return this.client(originalRequest);
          }
        }

        return Promise.reject(this.handleError(error));
      }
    );
  }

  private delay(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }

  private handleError(error: any): Error {
    if (error.response) {
      const { status, data } = error.response;

      if (status === 401 || status === 403) {
        return new (AuthenticationError as any)(
          data.error?.message || 'Authentication failed',
          data.error
        );
      }

      if (status === 400 || status === 422) {
        return new (ValidationError as any)(
          data.error?.message || 'Validation failed',
          data.error
        );
      }

      return new Error(data.error?.message || 'API request failed');
    }

    if (error.request) {
      return new (NetworkError as any)(
        'Network error occurred',
        { originalError: error.message }
      );
    }

    return error;
  }

  // ============================================================================
  // Exchange Rate Methods
  // ============================================================================

  /**
   * Get current exchange rate between two currencies
   */
  async getCurrentExchangeRate(
    request: GetExchangeRateRequest
  ): Promise<ExchangeRate> {
    const response = await this.client.get<APIResponse<ExchangeRate>>(
      '/exchange-rates/current',
      { params: request }
    );

    if (!response.data.success || !response.data.data) {
      throw new Error('Failed to fetch exchange rate');
    }

    return response.data.data;
  }

  /**
   * Get historical exchange rates
   */
  async getHistoricalExchangeRates(
    request: GetHistoricalRatesRequest
  ): Promise<HistoricalRate[]> {
    const response = await this.client.get<APIResponse<{ rates: HistoricalRate[] }>>(
      '/exchange-rates/historical',
      { params: request }
    );

    if (!response.data.success || !response.data.data) {
      throw new Error('Failed to fetch historical rates');
    }

    return response.data.data.rates;
  }

  // ============================================================================
  // Currency Conversion Methods
  // ============================================================================

  /**
   * Convert currency
   */
  async convertCurrency(
    request: ConvertCurrencyRequest
  ): Promise<CurrencyConversion> {
    const response = await this.client.post<APIResponse<CurrencyConversion>>(
      '/conversions',
      request
    );

    if (!response.data.success || !response.data.data) {
      throw new Error('Currency conversion failed');
    }

    return response.data.data;
  }

  /**
   * Get conversion status
   */
  async getConversionStatus(
    request: GetConversionStatusRequest
  ): Promise<CurrencyConversion> {
    const response = await this.client.get<APIResponse<CurrencyConversion>>(
      `/conversions/${request.conversionId}`
    );

    if (!response.data.success || !response.data.data) {
      throw new Error('Failed to fetch conversion status');
    }

    return response.data.data;
  }

  // ============================================================================
  // CBDC Methods
  // ============================================================================

  /**
   * Create CBDC wallet
   */
  async createWallet(request: CreateWalletRequest): Promise<CBDCWallet> {
    const response = await this.client.post<APIResponse<CBDCWallet>>(
      '/cbdc/wallets',
      request
    );

    if (!response.data.success || !response.data.data) {
      throw new Error('Failed to create wallet');
    }

    return response.data.data;
  }

  /**
   * Get wallet information
   */
  async getWallet(walletId: string): Promise<CBDCWallet> {
    const response = await this.client.get<APIResponse<CBDCWallet>>(
      `/cbdc/wallets/${walletId}`
    );

    if (!response.data.success || !response.data.data) {
      throw new Error('Failed to fetch wallet');
    }

    return response.data.data;
  }

  /**
   * Transfer CBDC between wallets
   */
  async transferCBDC(request: TransferCBDCRequest): Promise<CBDCTransaction> {
    const response = await this.client.post<APIResponse<CBDCTransaction>>(
      '/cbdc/transfers',
      request
    );

    if (!response.data.success || !response.data.data) {
      throw new Error('CBDC transfer failed');
    }

    return response.data.data;
  }

  /**
   * Get CBDC transaction details
   */
  async getCBDCTransaction(transactionId: string): Promise<CBDCTransaction> {
    const response = await this.client.get<APIResponse<CBDCTransaction>>(
      `/cbdc/transactions/${transactionId}`
    );

    if (!response.data.success || !response.data.data) {
      throw new Error('Failed to fetch transaction');
    }

    return response.data.data;
  }

  // ============================================================================
  // Analytics Methods
  // ============================================================================

  /**
   * Get transaction summary
   */
  async getTransactionSummary(
    request: TransactionSummaryRequest
  ): Promise<TransactionSummary> {
    const response = await this.client.get<APIResponse<TransactionSummary>>(
      '/analytics/transaction-summary',
      { params: request }
    );

    if (!response.data.success || !response.data.data) {
      throw new Error('Failed to fetch transaction summary');
    }

    return response.data.data;
  }
}

// ============================================================================
// Convenience Functions
// ============================================================================

/**
 * Create a new client instance
 */
export function createClient(config: ClientConfig): CurrencyIntegrationClient {
  return new CurrencyIntegrationClient(config);
}

/**
 * Default export
 */
export default CurrencyIntegrationClient;
