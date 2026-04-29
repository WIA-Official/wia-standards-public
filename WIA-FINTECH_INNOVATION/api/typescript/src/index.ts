import axios from 'axios';
import { Payment, BNPLPlan, OpenBankingConsent } from './types';

export class FintechClient {
  private baseUrl: string;
  private apiKey: string;

  constructor(config: { apiKey: string; baseUrl?: string }) {
    this.apiKey = config.apiKey;
    this.baseUrl = config.baseUrl || 'https://api.wia-fintech.io/v1';
  }

  async createPayment(payment: Partial<Payment>): Promise<Payment> {
    const response = await axios.post(`${this.baseUrl}/payments`, payment, {
      headers: { 'X-API-Key': this.apiKey }
    });
    return response.data;
  }

  async getBNPLPlans(amount: number): Promise<BNPLPlan[]> {
    const response = await axios.get(`${this.baseUrl}/bnpl/plans?amount=${amount}`, {
      headers: { 'X-API-Key': this.apiKey }
    });
    return response.data;
  }

  async getOpenBankingConsent(consentId: string): Promise<OpenBankingConsent> {
    const response = await axios.get(`${this.baseUrl}/open-banking/consent/${consentId}`, {
      headers: { 'X-API-Key': this.apiKey }
    });
    return response.data;
  }
}

export * from './types';
