/**
 * WIA-FIN-004 Digital Currency Standard - TypeScript SDK
 * @version 1.0.0
 */

import axios, { AxiosInstance } from 'axios';
import WebSocket from 'ws';
import * as types from './types';

export * from './types';

export class WIADigitalCurrency {
  private client: AxiosInstance;
  private apiUrl: string;
  private apiKey: string;
  private ws?: WebSocket;

  constructor(config: types.WIAConfig) {
    this.apiUrl = config.apiUrl || 'https://api.wia.example/v1';
    this.apiKey = config.apiKey;

    this.client = axios.create({
      baseURL: this.apiUrl,
      timeout: config.timeout || 30000,
      headers: {
        'Authorization': `Bearer ${this.apiKey}`,
        'Content-Type': 'application/json'
      }
    });
  }

  // ==================== Account Management ====================

  async getAccount(accountId: string): Promise<types.Account> {
    const response = await this.client.get(`/accounts/${accountId}`);
    return response.data;
  }

  async createAccount(account: Partial<types.Account>): Promise<types.Account> {
    const response = await this.client.post('/accounts', account);
    return response.data;
  }

  async getBalance(accountId: string, currency?: string): Promise<types.BalanceResponse> {
    const params = currency ? { currency } : {};
    const response = await this.client.get(`/accounts/${accountId}/balance`, { params });
    return response.data;
  }

  // ==================== Payments ====================

  async createPayment(
    payment: types.PaymentRequest,
    idempotencyKey?: string
  ): Promise<types.PaymentResponse> {
    const headers = idempotencyKey ? { 'Idempotency-Key': idempotencyKey } : {};
    const response = await this.client.post('/payments', payment, { headers });
    return response.data;
  }

  async getPayment(paymentId: string): Promise<types.Transaction> {
    const response = await this.client.get(`/payments/${paymentId}`);
    return response.data;
  }

  async listPayments(params: {
    accountId?: string;
    status?: types.TransactionStatus;
    limit?: number;
    offset?: number;
  }): Promise<{ data: types.Transaction[]; pagination: any }> {
    const response = await this.client.get('/payments', { params });
    return response.data;
  }

  async cancelPayment(paymentId: string): Promise<void> {
    await this.client.post(`/payments/${paymentId}/cancel`);
  }

  // ==================== Currency Exchange ====================

  async getExchangeRates(from: string, to: string): Promise<{
    from: string;
    to: string;
    rate: string;
    timestamp: string;
    validUntil: string;
  }> {
    const response = await this.client.get('/exchange/rates', {
      params: { from, to }
    });
    return response.data;
  }

  async exchangeCurrency(
    exchange: types.ExchangeRequest
  ): Promise<types.ExchangeResponse> {
    const response = await this.client.post('/exchange', exchange);
    return response.data;
  }

  // ==================== Compliance ====================

  async submitKYC(accountId: string, documents: any): Promise<void> {
    const formData = new FormData();
    formData.append('accountId', accountId);
    // Add document uploads
    await this.client.post('/compliance/kyc', formData, {
      headers: { 'Content-Type': 'multipart/form-data' }
    });
  }

  async getKYCStatus(accountId: string): Promise<types.ComplianceInfo> {
    const response = await this.client.get(`/compliance/kyc/${accountId}`);
    return response.data;
  }

  async screenTransaction(transactionId: string): Promise<{
    amlCheck: string;
    sanctionsCheck: string;
    riskScore: number;
  }> {
    const response = await this.client.post('/compliance/screening', {
      transactionId,
      checks: ['AML', 'SANCTIONS']
    });
    return response.data;
  }

  // ==================== WebSocket ====================

  connectWebSocket(onMessage: (data: any) => void): void {
    const wsUrl = this.apiUrl.replace(/^http/, 'ws');
    this.ws = new WebSocket(`${wsUrl}/ws`);

    this.ws.on('open', () => {
      this.ws?.send(JSON.stringify({
        type: 'auth',
        token: this.apiKey
      }));
    });

    this.ws.on('message', (data: any) => {
      const message = JSON.parse(data.toString());
      onMessage(message);
    });

    this.ws.on('error', (error) => {
      console.error('WebSocket error:', error);
    });
  }

  subscribeToBalance(accountId: string): void {
    if (!this.ws) throw new Error('WebSocket not connected');

    this.ws.send(JSON.stringify({
      type: 'subscribe',
      channel: 'balance',
      accountId
    }));
  }

  subscribeToPayments(accountId: string): void {
    if (!this.ws) throw new Error('WebSocket not connected');

    this.ws.send(JSON.stringify({
      type: 'subscribe',
      channel: 'payments',
      accountId
    }));
  }

  disconnectWebSocket(): void {
    if (this.ws) {
      this.ws.close();
      this.ws = undefined;
    }
  }
}

// ==================== Utility Functions ====================

export class WIAUtils {
  static validateAmount(amount: string, decimals: number): boolean {
    const parts = amount.split('.');
    if (parts.length > 2) return false;
    if (parts[1] && parts[1].length > decimals) return false;
    return /^\d+(\.\d+)?$/.test(amount);
  }

  static validateAddress(address: string, network: string): boolean {
    // Basic validation - implement network-specific validation
    if (network === 'ethereum') {
      return /^0x[a-fA-F0-9]{40}$/.test(address);
    }
    if (network === 'bitcoin') {
      return /^[13][a-km-zA-HJ-NP-Z1-9]{25,34}$/.test(address);
    }
    return true;
  }

  static formatAmount(amount: string, decimals: number): string {
    return parseFloat(amount).toFixed(decimals);
  }
}

// ==================== Export ====================

export default WIADigitalCurrency;
