/**
 * WIA-FIN-022 Open Banking Standard SDK
 * @module @wia/open-banking-sdk
 */

import axios, { AxiosInstance, AxiosRequestConfig } from 'axios';
import { v4 as uuidv4 } from 'uuid';
import * as jose from 'jose';
import * as fs from 'fs';
import * as types from './types';

export * from './types';

export class OpenBankingClient {
  private readonly config: types.OpenBankingConfig;
  private readonly httpClient: AxiosInstance;
  private privateKey?: jose.KeyLike;
  private signingKey?: jose.KeyLike;

  constructor(config: types.OpenBankingConfig) {
    this.config = {
      environment: 'sandbox',
      timeout: 30000,
      version: 'v1.0',
      ...config,
    };

    this.httpClient = axios.create({
      baseURL: this.config.baseUrl,
      timeout: this.config.timeout,
      headers: {
        'Content-Type': 'application/json',
        Accept: 'application/json',
      },
    });

    // Load certificates if provided
    if (this.config.privateKeyPath) {
      this.loadPrivateKey();
    }
    if (this.config.signingCertPath) {
      this.loadSigningKey();
    }
  }

  private async loadPrivateKey(): Promise<void> {
    const keyData = fs.readFileSync(this.config.privateKeyPath!, 'utf8');
    this.privateKey = await jose.importPKCS8(keyData, 'RS256');
  }

  private async loadSigningKey(): Promise<void> {
    const keyData = fs.readFileSync(this.config.signingCertPath!, 'utf8');
    this.signingKey = await jose.importPKCS8(keyData, 'PS256');
  }

  private async signRequest(payload: any): Promise<string> {
    if (!this.signingKey) {
      throw new Error('Signing key not loaded');
    }

    const jwt = await new jose.CompactSign(
      new TextEncoder().encode(JSON.stringify(payload))
    )
      .setProtectedHeader({
        alg: 'PS256',
        b64: false,
        crit: ['b64', 'http://openbanking.org.uk/iat', 'http://openbanking.org.uk/iss'],
        'http://openbanking.org.uk/iat': Math.floor(Date.now() / 1000),
        'http://openbanking.org.uk/iss': this.config.clientId,
      })
      .sign(this.signingKey);

    return jwt;
  }

  private getHeaders(accessToken?: string, options?: types.RequestOptions): Record<string, string> {
    const headers: Record<string, string> = {
      'x-fapi-interaction-id': uuidv4(),
      ...options?.headers,
    };

    if (accessToken) {
      headers['Authorization'] = `Bearer ${accessToken}`;
    }

    if (options?.idempotencyKey) {
      headers['x-idempotency-key'] = options.idempotencyKey;
    }

    return headers;
  }

  // Account Information Services
  public readonly accounts = {
    /**
     * Get list of accounts
     */
    list: async (
      accessToken: string,
      options?: types.RequestOptions
    ): Promise<types.APIResponse<{ Account: types.Account[] }>> => {
      const response = await this.httpClient.get('/accounts', {
        headers: this.getHeaders(accessToken, options),
      });
      return response.data;
    },

    /**
     * Get account balances
     */
    getBalances: async (
      accountId: string,
      accessToken: string,
      options?: types.RequestOptions
    ): Promise<types.APIResponse<{ Balance: types.Balance[] }>> => {
      const response = await this.httpClient.get(`/accounts/${accountId}/balances`, {
        headers: this.getHeaders(accessToken, options),
      });
      return response.data;
    },

    /**
     * Get account transactions
     */
    getTransactions: async (
      accountId: string,
      accessToken: string,
      params?: { fromBookingDateTime?: string; toBookingDateTime?: string },
      options?: types.RequestOptions
    ): Promise<types.APIResponse<{ Transaction: types.Transaction[] }>> => {
      const response = await this.httpClient.get(`/accounts/${accountId}/transactions`, {
        headers: this.getHeaders(accessToken, options),
        params,
      });
      return response.data;
    },
  };

  // Consent Management
  public readonly consents = {
    /**
     * Create account access consent
     */
    create: async (
      permissions: types.Permission[],
      expirationDateTime?: string,
      options?: types.RequestOptions
    ): Promise<types.AccountAccessConsent> => {
      const payload = {
        Data: {
          Permissions: permissions,
          ExpirationDateTime: expirationDateTime,
        },
      };

      const response = await this.httpClient.post('/account-access-consents', payload, {
        headers: this.getHeaders(undefined, options),
      });

      return response.data.Data;
    },

    /**
     * Get consent status
     */
    get: async (
      consentId: string,
      accessToken?: string,
      options?: types.RequestOptions
    ): Promise<types.AccountAccessConsent> => {
      const response = await this.httpClient.get(`/account-access-consents/${consentId}`, {
        headers: this.getHeaders(accessToken, options),
      });
      return response.data.Data;
    },

    /**
     * Delete/revoke consent
     */
    delete: async (
      consentId: string,
      accessToken: string,
      options?: types.RequestOptions
    ): Promise<void> => {
      await this.httpClient.delete(`/account-access-consents/${consentId}`, {
        headers: this.getHeaders(accessToken, options),
      });
    },
  };

  // Payment Services
  public readonly payments = {
    /**
     * Create domestic payment consent
     */
    createConsent: async (
      initiation: types.PaymentInitiation,
      options?: types.RequestOptions
    ): Promise<types.DomesticPaymentConsent> => {
      const payload = {
        Data: { Initiation: initiation },
        Risk: {},
      };

      const signature = await this.signRequest(payload);

      const response = await this.httpClient.post('/domestic-payment-consents', payload, {
        headers: {
          ...this.getHeaders(undefined, options),
          'x-jws-signature': signature,
        },
      });

      return response.data.Data;
    },

    /**
     * Submit domestic payment
     */
    submit: async (
      consentId: string,
      initiation: types.PaymentInitiation,
      accessToken: string,
      options?: types.RequestOptions
    ): Promise<types.DomesticPayment> => {
      const payload = {
        Data: {
          ConsentId: consentId,
          Initiation: initiation,
        },
        Risk: {},
      };

      const signature = await this.signRequest(payload);

      const response = await this.httpClient.post('/domestic-payments', payload, {
        headers: {
          ...this.getHeaders(accessToken, options),
          'x-jws-signature': signature,
          'x-idempotency-key': options?.idempotencyKey || uuidv4(),
        },
      });

      return response.data.Data;
    },

    /**
     * Get payment status
     */
    get: async (
      paymentId: string,
      accessToken: string,
      options?: types.RequestOptions
    ): Promise<types.DomesticPayment> => {
      const response = await this.httpClient.get(`/domestic-payments/${paymentId}`, {
        headers: this.getHeaders(accessToken, options),
      });
      return response.data.Data;
    },
  };

  // Funds Confirmation
  public readonly funds = {
    /**
     * Check funds availability
     */
    check: async (
      amount: types.Amount,
      debtorAccount: types.AccountIdentification,
      accessToken: string,
      options?: types.RequestOptions
    ): Promise<types.FundsConfirmation> => {
      const payload = {
        Data: {
          InstructedAmount: amount,
          DebtorAccount: debtorAccount,
        },
      };

      const response = await this.httpClient.post('/funds-confirmations', payload, {
        headers: this.getHeaders(accessToken, options),
      });

      return response.data.Data;
    },
  };

  // OAuth Authentication
  public readonly auth = {
    /**
     * Get authorization URL for user redirect
     */
    getAuthorizationUrl: (params: {
      consentId: string;
      redirectUri: string;
      scope: string;
      state?: string;
    }): string => {
      const url = new URL(`${this.config.baseUrl}/authorize`);
      url.searchParams.set('response_type', 'code');
      url.searchParams.set('client_id', this.config.clientId);
      url.searchParams.set('redirect_uri', params.redirectUri);
      url.searchParams.set('scope', params.scope);
      url.searchParams.set('state', params.state || uuidv4());
      url.searchParams.set('consent_id', params.consentId);

      return url.toString();
    },

    /**
     * Exchange authorization code for access token
     */
    exchangeCode: async (
      code: string,
      redirectUri: string
    ): Promise<types.OAuthTokens> => {
      const params = new URLSearchParams();
      params.append('grant_type', 'authorization_code');
      params.append('code', code);
      params.append('redirect_uri', redirectUri);
      params.append('client_id', this.config.clientId);
      if (this.config.clientSecret) {
        params.append('client_secret', this.config.clientSecret);
      }

      const response = await axios.post(`${this.config.baseUrl}/token`, params, {
        headers: {
          'Content-Type': 'application/x-www-form-urlencoded',
        },
      });

      return response.data;
    },

    /**
     * Refresh access token
     */
    refreshToken: async (refreshToken: string): Promise<types.OAuthTokens> => {
      const params = new URLSearchParams();
      params.append('grant_type', 'refresh_token');
      params.append('refresh_token', refreshToken);
      params.append('client_id', this.config.clientId);
      if (this.config.clientSecret) {
        params.append('client_secret', this.config.clientSecret);
      }

      const response = await axios.post(`${this.config.baseUrl}/token`, params, {
        headers: {
          'Content-Type': 'application/x-www-form-urlencoded',
        },
      });

      return response.data;
    },
  };

  // Webhook verification
  public readonly webhooks = {
    /**
     * Verify webhook signature
     */
    verify: (signature: string, payload: any): boolean => {
      // Implementation would verify HMAC signature
      // This is a placeholder
      return true;
    },

    /**
     * Parse webhook event
     */
    parse: (payload: any): types.WebhookEvent => {
      return payload as types.WebhookEvent;
    },
  };
}

export default OpenBankingClient;
