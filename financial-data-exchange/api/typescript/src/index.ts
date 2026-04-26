/**
 * WIA-FIN-021 Financial Data Exchange SDK
 * Version 2.0
 *
 * @packageDocumentation
 */

import axios, { AxiosInstance, AxiosRequestConfig } from 'axios';
import * as jwt from 'jsonwebtoken';
import WebSocket from 'ws';

import {
  WIAConfig,
  Account,
  Transaction,
  Payment,
  PaymentRequest,
  Consent,
  Webhook,
  WebhookEvent,
  APIResponse,
  APIError,
  Pagination,
  IntelligentValidationRequest,
  IntelligentValidationResult,
  FraudScore,
} from './types';

/**
 * Main client for WIA-FIN-021 Financial Data Exchange API
 */
export class FinancialDataExchangeClient {
  private config: WIAConfig;
  private httpClient: AxiosInstance;
  private accessToken?: string;

  constructor(config: WIAConfig) {
    this.config = {
      version: '2.0',
      timeout: 30000,
      debug: false,
      ...config,
    };

    this.httpClient = axios.create({
      baseURL: this.config.baseUrl,
      timeout: this.config.timeout,
      headers: {
        'Content-Type': 'application/json',
        'X-API-Version': this.config.version,
        ...this.config.headers,
      },
    });

    // Add request interceptor for authentication
    this.httpClient.interceptors.request.use(
      async (config) => {
        const token = await this.getAccessToken();
        if (token) {
          config.headers.Authorization = `Bearer ${token}`;
        }
        return config;
      },
      (error) => Promise.reject(error)
    );

    // Add response interceptor for error handling
    this.httpClient.interceptors.response.use(
      (response) => response,
      (error) => {
        if (this.config.debug) {
          console.error('API Error:', error.response?.data || error.message);
        }
        throw this.handleError(error);
      }
    );
  }

  // ============================================================================
  // Authentication
  // ============================================================================

  private async getAccessToken(): Promise<string | undefined> {
    if (this.config.auth.type === 'jwt' && this.config.auth.jwt) {
      return this.config.auth.jwt.token;
    }

    if (this.config.auth.type === 'api-key' && this.config.auth.apiKey) {
      return this.config.auth.apiKey;
    }

    if (this.config.auth.type === 'oauth2' && this.config.auth.oauth2) {
      if (!this.accessToken) {
        this.accessToken = await this.fetchOAuth2Token();
      }
      return this.accessToken;
    }

    return undefined;
  }

  private async fetchOAuth2Token(): Promise<string> {
    const oauth2Config = this.config.auth.oauth2!;

    const response = await axios.post(
      oauth2Config.tokenEndpoint,
      new URLSearchParams({
        grant_type: 'client_credentials',
        client_id: oauth2Config.clientId,
        client_secret: oauth2Config.clientSecret,
        scope: oauth2Config.scopes.join(' '),
      }),
      {
        headers: {
          'Content-Type': 'application/x-www-form-urlencoded',
        },
      }
    );

    return response.data.access_token;
  }

  // ============================================================================
  // Account Operations
  // ============================================================================

  /**
   * Get account by ID
   */
  async getAccount(accountId: string): Promise<Account> {
    const response = await this.httpClient.get<APIResponse<Account>>(
      `/api/${this.config.version}/accounts/${accountId}`
    );
    return response.data.data;
  }

  /**
   * List all accounts
   */
  async listAccounts(params?: {
    customerId?: string;
    limit?: number;
    cursor?: string;
  }): Promise<{ accounts: Account[]; pagination: Pagination }> {
    const response = await this.httpClient.get<APIResponse<Account[]>>(
      `/api/${this.config.version}/accounts`,
      { params }
    );
    return {
      accounts: response.data.data,
      pagination: response.data.pagination!,
    };
  }

  /**
   * Get account balance
   */
  async getBalance(accountId: string): Promise<any> {
    const response = await this.httpClient.get(
      `/api/${this.config.version}/accounts/${accountId}/balance`
    );
    return response.data.data;
  }

  // ============================================================================
  // Transaction Operations
  // ============================================================================

  /**
   * Get transaction by ID
   */
  async getTransaction(transactionId: string): Promise<Transaction> {
    const response = await this.httpClient.get<APIResponse<Transaction>>(
      `/api/${this.config.version}/transactions/${transactionId}`
    );
    return response.data.data;
  }

  /**
   * List transactions for an account
   */
  async listTransactions(
    accountId: string,
    params?: {
      limit?: number;
      cursor?: string;
      fromDate?: string;
      toDate?: string;
    }
  ): Promise<{ transactions: Transaction[]; pagination: Pagination }> {
    const response = await this.httpClient.get<APIResponse<Transaction[]>>(
      `/api/${this.config.version}/accounts/${accountId}/transactions`,
      { params }
    );
    return {
      transactions: response.data.data,
      pagination: response.data.pagination!,
    };
  }

  // ============================================================================
  // Payment Operations
  // ============================================================================

  /**
   * Initiate a payment
   */
  async initiatePayment(request: PaymentRequest): Promise<Payment> {
    const headers: any = {
      'Content-Type': 'application/json',
    };

    if (request.idempotencyKey) {
      headers['Idempotency-Key'] = request.idempotencyKey;
    }

    const response = await this.httpClient.post<APIResponse<Payment>>(
      `/api/${this.config.version}/payments`,
      request,
      { headers }
    );

    return response.data.data;
  }

  /**
   * Get payment status
   */
  async getPayment(paymentId: string): Promise<Payment> {
    const response = await this.httpClient.get<APIResponse<Payment>>(
      `/api/${this.config.version}/payments/${paymentId}`
    );
    return response.data.data;
  }

  /**
   * Cancel a payment
   */
  async cancelPayment(paymentId: string): Promise<Payment> {
    const response = await this.httpClient.delete<APIResponse<Payment>>(
      `/api/${this.config.version}/payments/${paymentId}`
    );
    return response.data.data;
  }

  // ============================================================================
  // Consent Operations
  // ============================================================================

  /**
   * Get consent by ID
   */
  async getConsent(consentId: string): Promise<Consent> {
    const response = await this.httpClient.get<APIResponse<Consent>>(
      `/api/${this.config.version}/consents/${consentId}`
    );
    return response.data.data;
  }

  /**
   * Revoke a consent
   */
  async revokeConsent(consentId: string): Promise<void> {
    await this.httpClient.delete(
      `/api/${this.config.version}/consents/${consentId}`
    );
  }

  // ============================================================================
  // Webhook Operations
  // ============================================================================

  /**
   * Register a webhook
   */
  async registerWebhook(
    url: string,
    events: WebhookEvent[],
    secret: string
  ): Promise<Webhook> {
    const response = await this.httpClient.post<APIResponse<Webhook>>(
      `/api/${this.config.version}/webhooks`,
      { url, events, secret }
    );
    return response.data.data;
  }

  /**
   * List webhooks
   */
  async listWebhooks(): Promise<Webhook[]> {
    const response = await this.httpClient.get<APIResponse<Webhook[]>>(
      `/api/${this.config.version}/webhooks`
    );
    return response.data.data;
  }

  /**
   * Delete a webhook
   */
  async deleteWebhook(webhookId: string): Promise<void> {
    await this.httpClient.delete(
      `/api/${this.config.version}/webhooks/${webhookId}`
    );
  }

  // ============================================================================
  // AI/ML Operations (v2.0+)
  // ============================================================================

  /**
   * Validate data using AI/ML
   */
  async validateIntelligent(
    request: IntelligentValidationRequest
  ): Promise<IntelligentValidationResult> {
    const response = await this.httpClient.post<
      APIResponse<IntelligentValidationResult>
    >(`/api/${this.config.version}/validate/intelligent`, request);
    return response.data.data;
  }

  /**
   * Get fraud score for a transaction
   */
  async getFraudScore(transactionId: string): Promise<FraudScore> {
    const response = await this.httpClient.get<APIResponse<FraudScore>>(
      `/api/${this.config.version}/fraud/score/${transactionId}`
    );
    return response.data.data;
  }

  // ============================================================================
  // Real-time Streaming
  // ============================================================================

  /**
   * Subscribe to real-time events via WebSocket
   */
  subscribeToEvents(
    accountId: string,
    events: string[],
    callback: (event: any) => void
  ): WebSocket {
    const wsUrl = this.config.baseUrl.replace(/^http/, 'ws');
    const ws = new WebSocket(`${wsUrl}/stream`);

    ws.on('open', () => {
      ws.send(
        JSON.stringify({
          action: 'subscribe',
          accountId,
          events,
        })
      );
    });

    ws.on('message', (data: string) => {
      try {
        const event = JSON.parse(data.toString());
        callback(event);
      } catch (error) {
        console.error('Failed to parse WebSocket message:', error);
      }
    });

    ws.on('error', (error) => {
      console.error('WebSocket error:', error);
    });

    return ws;
  }

  // ============================================================================
  // Batch Operations
  // ============================================================================

  /**
   * Execute multiple API requests in a single batch
   */
  async batch(requests: Array<{ id: string; method: string; url: string; body?: any }>) {
    const response = await this.httpClient.post(
      `/api/${this.config.version}/batch`,
      { requests }
    );
    return response.data;
  }

  // ============================================================================
  // Error Handling
  // ============================================================================

  private handleError(error: any): APIError {
    if (error.response) {
      const apiError: APIError = {
        code: error.response.data?.error?.code || 'UNKNOWN_ERROR',
        message: error.response.data?.error?.message || error.message,
        details: error.response.data?.error?.details,
        retryable: error.response.data?.error?.retryable,
        retryAfter: error.response.data?.error?.retry_after,
      };
      return apiError;
    }

    return {
      code: 'NETWORK_ERROR',
      message: error.message,
      retryable: true,
    };
  }
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Verify webhook signature
 */
export function verifyWebhookSignature(
  payload: string,
  signature: string,
  secret: string
): boolean {
  const crypto = require('crypto');
  const expectedSignature = crypto
    .createHmac('sha256', secret)
    .update(payload)
    .digest('hex');
  return signature === expectedSignature;
}

/**
 * Generate idempotency key
 */
export function generateIdempotencyKey(): string {
  const crypto = require('crypto');
  return crypto.randomBytes(16).toString('hex');
}

// ============================================================================
// Re-export types
// ============================================================================

export * from './types';

// ============================================================================
// Default Export
// ============================================================================

export default FinancialDataExchangeClient;
