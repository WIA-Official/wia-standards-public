import axios from 'axios';
import { FoodProduct, RecallNotice } from './types';

export class FoodSafetyClient {
  private baseUrl: string;
  private apiKey: string;

  constructor(config: {apiKey: string; baseUrl?: string}) {
    this.apiKey = config.apiKey;
    this.baseUrl = config.baseUrl || 'https://api.wia-foodsafety.io/v1';
  }

  async traceProduct(qrCode: string): Promise<FoodProduct> {
    const response = await axios.get(`${this.baseUrl}/products/trace`, {
      params: {qr_code: qrCode},
      headers: {'X-API-Key': this.apiKey}
    });
    return response.data;
  }

  async getRecalls(): Promise<RecallNotice[]> {
    const response = await axios.get(`${this.baseUrl}/recalls`, {
      headers: {'X-API-Key': this.apiKey}
    });
    return response.data.recalls;
  }
}

export * from './types';
