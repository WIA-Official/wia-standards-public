/**
 * WIA-FIN-012 Payment System Standard - TypeScript SDK
 * Version: 1.0.0
 *
 * 弘益人間 (Hongik Ingan) - Benefit All Humanity
 */

import axios, { AxiosInstance, AxiosError } from 'axios';
import * as CryptoJS from 'crypto-js';
import * as Types from './types';

export * from './types';

// ============================================================================
// SDK Client
// ============================================================================

export class PaymentClient {
  private client: AxiosInstance;
  private apiKey: string;
  private environment: 'production' | 'sandbox';

  constructor(config: Types.PaymentConfig) {
    this.apiKey = config.apiKey;
    this.environment = config.environment || 'production';

    const baseURL =
      this.environment === 'production'
        ? 'https://api.payment-gateway.com/v1'
        : 'https://sandbox.payment-gateway.com/v1';

    this.client = axios.create({
      baseURL,
      timeout: config.timeout || 30000,
      headers: {
        'Authorization': `Bearer ${this.apiKey}`,
        'Content-Type': 'application/json',
        'X-API-Version': config.apiVersion || '2025-01-15',
      },
    });

    // Add retry logic
    if (config.maxRetries) {
      this.setupRetryInterceptor(config.maxRetries);
    }
  }

  // Payments API
  public payments = {
    /**
     * Create a payment authorization
     */
    authorize: async (
      params: Types.CreatePaymentIntentParams
    ): Promise<Types.PaymentIntent> => {
      return this.request('POST', '/payments/authorize', params);
    },

    /**
     * Capture an authorized payment
     */
    capture: async (
      paymentId: string,
      amount?: number,
      options?: { final?: boolean }
    ): Promise<Types.Capture> => {
      return this.request('POST', `/payments/${paymentId}/capture`, {
        amount,
        ...options,
      });
    },

    /**
     * Create a direct charge (authorize + capture)
     */
    charge: async (
      params: Types.CreatePaymentIntentParams
    ): Promise<Types.PaymentIntent> => {
      return this.request('POST', '/payments/charge', {
        ...params,
        capture: true,
      });
    },

    /**
     * Refund a payment
     */
    refund: async (
      paymentId: string,
      amount?: number,
      reason?: string
    ): Promise<Types.Refund> => {
      return this.request('POST', `/payments/${paymentId}/refund`, {
        amount,
        reason,
      });
    },

    /**
     * Void an authorization
     */
    void: async (
      paymentId: string,
      reason?: string
    ): Promise<{ id: string; status: 'succeeded' }> => {
      return this.request('POST', `/payments/${paymentId}/void`, { reason });
    },

    /**
     * Retrieve a payment
     */
    retrieve: async (paymentId: string): Promise<Types.PaymentIntent> => {
      return this.request('GET', `/payments/${paymentId}`);
    },

    /**
     * List payments
     */
    list: async (params?: {
      limit?: number;
      starting_after?: string;
    }): Promise<Types.ListResponse<Types.PaymentIntent>> => {
      return this.request('GET', '/payments', params);
    },
  };

  // Tokens API
  public tokens = {
    /**
     * Create a card token
     */
    create: async (card: Types.CardInput): Promise<Types.Token> => {
      return this.request('POST', '/tokens', { card });
    },

    /**
     * Retrieve a token
     */
    retrieve: async (tokenId: string): Promise<Types.Token> => {
      return this.request('GET', `/tokens/${tokenId}`);
    },
  };

  // Subscriptions API
  public subscriptions = {
    /**
     * Create a subscription
     */
    create: async (params: {
      customer_id: string;
      payment_method_id: string;
      plan: Types.SubscriptionPlan;
      trial_period_days?: number;
      description?: string;
    }): Promise<Types.Subscription> => {
      return this.request('POST', '/subscriptions', params);
    },

    /**
     * Update a subscription
     */
    update: async (
      subscriptionId: string,
      params: Partial<Types.Subscription>
    ): Promise<Types.Subscription> => {
      return this.request('PUT', `/subscriptions/${subscriptionId}`, params);
    },

    /**
     * Cancel a subscription
     */
    cancel: async (
      subscriptionId: string
    ): Promise<Types.Subscription> => {
      return this.request('DELETE', `/subscriptions/${subscriptionId}`);
    },

    /**
     * Retrieve a subscription
     */
    retrieve: async (
      subscriptionId: string
    ): Promise<Types.Subscription> => {
      return this.request('GET', `/subscriptions/${subscriptionId}`);
    },
  };

  // 3D Secure API
  public threeDSecure = {
    /**
     * Initiate 3D Secure authentication
     */
    authenticate: async (params: {
      payment_method_id: string;
      amount: number;
      currency: Types.Currency;
      return_url: string;
      browser_info?: Types.BrowserInfo;
    }): Promise<{
      id: string;
      status: 'requires_action' | 'succeeded';
      redirect_url?: string;
      version: string;
    }> => {
      return this.request('POST', '/3ds/authenticate', params);
    },
  };

  // Webhooks utilities
  public webhooks = {
    /**
     * Verify webhook signature
     */
    verify: (
      payload: string,
      signature: string,
      secret: string
    ): Types.WebhookEvent => {
      const computedSignature = CryptoJS.HmacSHA256(payload, secret).toString();

      if (signature !== `sha256=${computedSignature}`) {
        throw new Error('Invalid webhook signature');
      }

      return JSON.parse(payload);
    },
  };

  // Private helper methods
  private async request<T>(
    method: string,
    path: string,
    data?: any
  ): Promise<T> {
    try {
      const response = await this.client.request({
        method,
        url: path,
        data: method !== 'GET' ? data : undefined,
        params: method === 'GET' ? data : undefined,
      });

      return response.data;
    } catch (error) {
      if (axios.isAxiosError(error)) {
        throw this.handleError(error);
      }
      throw error;
    }
  }

  private handleError(error: AxiosError): Error {
    if (error.response) {
      const apiError = error.response.data as Types.APIError;
      const message = apiError.error?.message || 'Unknown error occurred';
      return new Error(message);
    }

    return new Error('Network error occurred');
  }

  private setupRetryInterceptor(maxRetries: number): void {
    let retryCount = 0;

    this.client.interceptors.response.use(
      (response) => response,
      async (error) => {
        const config = error.config;

        if (!config || retryCount >= maxRetries) {
          return Promise.reject(error);
        }

        // Retry on 5xx errors and network errors
        if (
          error.response?.status >= 500 ||
          error.code === 'ECONNABORTED' ||
          error.code === 'ETIMEDOUT'
        ) {
          retryCount++;
          const delay = Math.min(1000 * Math.pow(2, retryCount), 10000);
          await new Promise((resolve) => setTimeout(resolve, delay));
          return this.client.request(config);
        }

        return Promise.reject(error);
      }
    );
  }
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Validate card number using Luhn algorithm
 */
export function validateCardNumber(cardNumber: string): boolean {
  const digits = cardNumber.replace(/\D/g, '');

  if (digits.length < 13 || digits.length > 19) {
    return false;
  }

  let sum = 0;
  let isEven = false;

  for (let i = digits.length - 1; i >= 0; i--) {
    let digit = parseInt(digits[i]);

    if (isEven) {
      digit *= 2;
      if (digit > 9) {
        digit -= 9;
      }
    }

    sum += digit;
    isEven = !isEven;
  }

  return sum % 10 === 0;
}

/**
 * Detect card network from PAN
 */
export function detectCardNetwork(cardNumber: string): Types.CardNetwork {
  const digits = cardNumber.replace(/\D/g, '');

  if (/^4/.test(digits)) return 'visa';
  if (/^5[1-5]/.test(digits)) return 'mastercard';
  if (/^2[2-7]/.test(digits)) return 'mastercard';
  if (/^3[47]/.test(digits)) return 'amex';
  if (/^6(?:011|5)/.test(digits)) return 'discover';
  if (/^3(?:0[0-5]|[68])/.test(digits)) return 'diners';
  if (/^35/.test(digits)) return 'jcb';
  if (/^62/.test(digits)) return 'unionpay';

  return 'unknown';
}

/**
 * Format card number with spaces
 */
export function formatCardNumber(
  cardNumber: string,
  network?: Types.CardNetwork
): string {
  const digits = cardNumber.replace(/\D/g, '');

  if (network === 'amex') {
    // American Express: 4-6-5
    return digits.replace(/(\d{4})(\d{6})(\d{5})/, '$1 $2 $3');
  } else {
    // Others: 4-4-4-4
    return digits.replace(/(\d{4})/g, '$1 ').trim();
  }
}

/**
 * Mask card number
 */
export function maskCardNumber(
  cardNumber: string,
  showFirst: number = 0,
  showLast: number = 4
): string {
  const digits = cardNumber.replace(/\D/g, '');
  const masked = '•'.repeat(Math.max(0, digits.length - showFirst - showLast));
  const first = showFirst > 0 ? digits.slice(0, showFirst) : '';
  const last = showLast > 0 ? digits.slice(-showLast) : '';

  return formatCardNumber(first + masked + last);
}

/**
 * Validate expiration date
 */
export function validateExpiry(month: number, year: number): boolean {
  if (month < 1 || month > 12) {
    return false;
  }

  const fullYear = year < 100 ? 2000 + year : year;
  const expiryDate = new Date(fullYear, month, 0);
  const now = new Date();

  return expiryDate >= now;
}

/**
 * Generate test card number
 */
export function generateTestCard(network: Types.CardNetwork = 'visa'): string {
  const testCards = {
    visa: '4532123456789010',
    mastercard: '5425233430109903',
    amex: '378282246310005',
    discover: '6011111111111117',
    diners: '30569309025904',
    jcb: '3530111333300000',
    unionpay: '6200000000000005',
    maestro: '6304000000000000',
    unknown: '0000000000000000',
  };

  return testCards[network] || testCards.visa;
}

// ============================================================================
// Default Export
// ============================================================================

export default PaymentClient;

/**
 * 弘益人間 (Hongik Ingan) - Benefit All Humanity
 *
 * This SDK enables secure payment processing that empowers businesses
 * and serves customers worldwide.
 */
