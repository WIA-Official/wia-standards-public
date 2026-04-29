/**
 * WIA-FINANCIAL_FRAUD_DETECTION - TypeScript SDK
 *
 * @version 1.0.0
 * @standard WIA-FINANCIAL_FRAUD_DETECTION
 * @organization WIA (World Certification Industry Association)
 *
 * @example
 * ```typescript
 * import { FraudDetectionClient } from '@wia/fraud-detection';
 *
 * const client = new FraudDetectionClient({
 *   apiKey: 'EXAMPLE_API_KEY_REPLACE_ME...'
 * });
 *
 * const result = await client.analyzeTransaction(transaction);
 * console.log(result.fraud_assessment.risk_score);
 * ```
 */

import axios, { AxiosInstance, AxiosError, AxiosRequestConfig } from 'axios';
import {
  ClientConfig,
  Transaction,
  FraudAnalysisResponse,
  FraudFeedback,
  FeedbackSubmissionResponse,
  BatchAnalysisResponse,
  StatisticsResponse,
  ApiResponse,
  ApiError,
} from './types';
import { validateTransaction, validateFeedback } from './validators';

/**
 * Default configuration
 */
const DEFAULT_CONFIG = {
  baseUrl: 'https://api.wia-fraud.io/v1',
  timeout: 10000,
  maxRetries: 3,
  debug: false,
};

/**
 * Custom error class for SDK errors
 */
export class FraudDetectionError extends Error {
  constructor(
    message: string,
    public code: string,
    public statusCode?: number,
    public errors?: ApiError[]
  ) {
    super(message);
    this.name = 'FraudDetectionError';
    Object.setPrototypeOf(this, FraudDetectionError.prototype);
  }
}

/**
 * Main client class for WIA Fraud Detection API
 */
export class FraudDetectionClient {
  private config: Required<ClientConfig>;
  private httpClient: AxiosInstance;

  /**
   * Create a new FraudDetectionClient
   *
   * @param config - Client configuration
   * @throws {FraudDetectionError} If API key is missing
   *
   * @example
   * ```typescript
   * const client = new FraudDetectionClient({
   *   apiKey: 'EXAMPLE_API_KEY_REPLACE_ME...',
   *   baseUrl: 'https://api.wia-fraud.io/v1',
   *   timeout: 10000
   * });
   * ```
   */
  constructor(config: ClientConfig) {
    if (!config.apiKey) {
      throw new FraudDetectionError(
        'API key is required',
        'missing_api_key'
      );
    }

    this.config = {
      ...DEFAULT_CONFIG,
      ...config,
    } as Required<ClientConfig>;

    // Initialize HTTP client
    this.httpClient = axios.create({
      baseURL: this.config.baseUrl,
      timeout: this.config.timeout,
      headers: {
        'Content-Type': 'application/json',
        'X-API-Key': this.config.apiKey,
        'User-Agent': 'wia-fraud-detection-sdk/1.0.0',
      },
    });

    // Add request interceptor for logging
    if (this.config.debug) {
      this.httpClient.interceptors.request.use((config) => {
        console.log('[FraudDetectionSDK] Request:', {
          method: config.method,
          url: config.url,
          data: config.data,
        });
        return config;
      });
    }

    // Add response interceptor for logging and error handling
    this.httpClient.interceptors.response.use(
      (response) => {
        if (this.config.debug) {
          console.log('[FraudDetectionSDK] Response:', {
            status: response.status,
            data: response.data,
          });
        }
        return response;
      },
      (error: AxiosError) => {
        return Promise.reject(this.handleError(error));
      }
    );
  }

  /**
   * Handle API errors
   */
  private handleError(error: AxiosError): FraudDetectionError {
    if (error.response) {
      // Server responded with error status
      const apiResponse = error.response.data as ApiResponse<any>;
      const errors = apiResponse.errors || [];
      const message = errors.length > 0
        ? errors[0].message
        : 'Unknown API error';
      const code = errors.length > 0
        ? errors[0].code
        : 'unknown_error';

      return new FraudDetectionError(
        message,
        code,
        error.response.status,
        errors
      );
    } else if (error.request) {
      // Request was made but no response received
      return new FraudDetectionError(
        'No response from server',
        'network_error'
      );
    } else {
      // Something else happened
      return new FraudDetectionError(
        error.message,
        'request_error'
      );
    }
  }

  /**
   * Make API request with retry logic
   */
  private async makeRequest<T>(
    config: AxiosRequestConfig,
    retryCount = 0
  ): Promise<ApiResponse<T>> {
    try {
      const response = await this.httpClient.request<ApiResponse<T>>(config);
      return response.data;
    } catch (error) {
      if (
        retryCount < this.config.maxRetries &&
        this.isRetryableError(error as FraudDetectionError)
      ) {
        // Exponential backoff
        const delay = Math.pow(2, retryCount) * 1000;
        await this.sleep(delay);
        return this.makeRequest<T>(config, retryCount + 1);
      }
      throw error;
    }
  }

  /**
   * Check if error is retryable
   */
  private isRetryableError(error: FraudDetectionError): boolean {
    // Retry on network errors and 5xx server errors
    return (
      error.code === 'network_error' ||
      (error.statusCode !== undefined && error.statusCode >= 500)
    );
  }

  /**
   * Sleep for specified milliseconds
   */
  private sleep(ms: number): Promise<void> {
    return new Promise((resolve) => setTimeout(resolve, ms));
  }

  // ============================================================================
  // Public API Methods
  // ============================================================================

  /**
   * Analyze a transaction for fraud
   *
   * @param transaction - Transaction to analyze
   * @returns Fraud analysis result
   * @throws {FraudDetectionError} If validation fails or API request fails
   *
   * @example
   * ```typescript
   * const result = await client.analyzeTransaction({
   *   id: 'txn_123',
   *   amount: 149.99,
   *   currency: 'USD',
   *   // ... other fields
   * });
   *
   * if (result.fraud_assessment.decision === 'block') {
   *   console.log('Transaction blocked:', result.fraud_assessment.decision_reasons);
   * }
   * ```
   */
  async analyzeTransaction(
    transaction: Transaction
  ): Promise<FraudAnalysisResponse> {
    // Validate transaction
    const validationError = validateTransaction(transaction);
    if (validationError) {
      throw new FraudDetectionError(
        validationError,
        'invalid_transaction'
      );
    }

    // Make API request
    const response = await this.makeRequest<FraudAnalysisResponse>({
      method: 'POST',
      url: '/fraud/analyze',
      data: { transaction },
    });

    if (response.status === 'error') {
      throw new FraudDetectionError(
        'Fraud analysis failed',
        'analysis_failed',
        undefined,
        response.errors
      );
    }

    return response.data!;
  }

  /**
   * Analyze multiple transactions in batch
   *
   * @param transactions - Array of transactions (max 100)
   * @returns Batch analysis results
   * @throws {FraudDetectionError} If validation fails or API request fails
   *
   * @example
   * ```typescript
   * const results = await client.analyzeBatch([transaction1, transaction2]);
   * console.log(`Analyzed ${results.summary.total_transactions} transactions`);
   * console.log(`Blocked: ${results.summary.blocked}`);
   * ```
   */
  async analyzeBatch(
    transactions: Transaction[]
  ): Promise<BatchAnalysisResponse> {
    if (transactions.length === 0) {
      throw new FraudDetectionError(
        'At least one transaction required',
        'empty_batch'
      );
    }

    if (transactions.length > 100) {
      throw new FraudDetectionError(
        'Maximum 100 transactions per batch',
        'batch_too_large'
      );
    }

    // Validate all transactions
    for (const transaction of transactions) {
      const validationError = validateTransaction(transaction);
      if (validationError) {
        throw new FraudDetectionError(
          `Transaction ${transaction.id}: ${validationError}`,
          'invalid_transaction'
        );
      }
    }

    // Make API request
    const response = await this.makeRequest<BatchAnalysisResponse>({
      method: 'POST',
      url: '/fraud/batch-analyze',
      data: { transactions },
    });

    if (response.status === 'error') {
      throw new FraudDetectionError(
        'Batch analysis failed',
        'batch_analysis_failed',
        undefined,
        response.errors
      );
    }

    return response.data!;
  }

  /**
   * Submit feedback on a fraud decision
   *
   * @param feedback - Fraud feedback
   * @returns Feedback submission result
   * @throws {FraudDetectionError} If validation fails or API request fails
   *
   * @example
   * ```typescript
   * await client.submitFeedback({
   *   transaction_id: 'txn_123',
   *   feedback_type: FeedbackType.CONFIRMED_FRAUD,
   *   fraud_type: FraudType.STOLEN_CARD,
   *   submitted_by: 'agent_456',
   *   notes: 'Customer confirmed unauthorized transaction'
   * });
   * ```
   */
  async submitFeedback(
    feedback: FraudFeedback
  ): Promise<FeedbackSubmissionResponse> {
    // Validate feedback
    const validationError = validateFeedback(feedback);
    if (validationError) {
      throw new FraudDetectionError(
        validationError,
        'invalid_feedback'
      );
    }

    // Make API request
    const response = await this.makeRequest<FeedbackSubmissionResponse>({
      method: 'POST',
      url: '/fraud/feedback',
      data: feedback,
    });

    if (response.status === 'error') {
      throw new FraudDetectionError(
        'Feedback submission failed',
        'feedback_failed',
        undefined,
        response.errors
      );
    }

    return response.data!;
  }

  /**
   * Get fraud report for a transaction
   *
   * @param transactionId - Transaction identifier
   * @returns Fraud analysis report
   * @throws {FraudDetectionError} If transaction not found or API request fails
   *
   * @example
   * ```typescript
   * const report = await client.getFraudReport('txn_123');
   * console.log('Risk score:', report.fraud_assessment.risk_score);
   * console.log('Similar transactions:', report.similar_transactions);
   * ```
   */
  async getFraudReport(transactionId: string): Promise<any> {
    const response = await this.makeRequest<any>({
      method: 'GET',
      url: `/fraud/reports/${transactionId}`,
    });

    if (response.status === 'error') {
      throw new FraudDetectionError(
        `Fraud report not found for transaction ${transactionId}`,
        'report_not_found',
        undefined,
        response.errors
      );
    }

    return response.data!.report;
  }

  /**
   * Get fraud statistics for a date range
   *
   * @param startDate - Start date (ISO 8601)
   * @param endDate - End date (ISO 8601)
   * @param options - Additional options
   * @returns Fraud statistics
   * @throws {FraudDetectionError} If API request fails
   *
   * @example
   * ```typescript
   * const stats = await client.getStatistics(
   *   '2026-01-01',
   *   '2026-01-11',
   *   { groupBy: 'day' }
   * );
   * console.log('Fraud rate:', stats.statistics.fraud_rate);
   * console.log('Amount saved:', stats.statistics.amount_saved_usd);
   * ```
   */
  async getStatistics(
    startDate: string,
    endDate: string,
    options?: {
      groupBy?: 'day' | 'week' | 'month';
      merchantId?: string;
    }
  ): Promise<StatisticsResponse> {
    const params: any = {
      start_date: startDate,
      end_date: endDate,
    };

    if (options?.groupBy) {
      params.group_by = options.groupBy;
    }

    if (options?.merchantId) {
      params.merchant_id = options.merchantId;
    }

    const response = await this.makeRequest<StatisticsResponse>({
      method: 'GET',
      url: '/fraud/statistics',
      params,
    });

    if (response.status === 'error') {
      throw new FraudDetectionError(
        'Failed to retrieve statistics',
        'statistics_failed',
        undefined,
        response.errors
      );
    }

    return response.data!;
  }

  /**
   * Health check endpoint
   *
   * @returns Health status
   * @throws {FraudDetectionError} If API is unhealthy
   *
   * @example
   * ```typescript
   * const health = await client.healthCheck();
   * console.log('API status:', health.status);
   * ```
   */
  async healthCheck(): Promise<{ status: string }> {
    try {
      const response = await this.httpClient.get('/health');
      return response.data;
    } catch (error) {
      throw new FraudDetectionError(
        'Health check failed',
        'health_check_failed'
      );
    }
  }
}

// ============================================================================
// Exports
// ============================================================================

export * from './types';
export { validateTransaction, validateFeedback } from './validators';
export * as utils from './utils';

export default FraudDetectionClient;
