/**
 * WIA-MED-011 Smart Pill - TypeScript SDK
 * @version 1.0.0
 */

import axios, { AxiosInstance } from 'axios';
import type * as Types from './types';
export * from './types';

export class SmartPillSDK {
  private api: AxiosInstance;

  constructor(config: Types.SDKConfig) {
    this.api = axios.create({
      baseURL: config.baseUrl || 'https://api.wia.org/smart-pill/v1',
      headers: { 'Authorization': `Bearer ${config.apiKey}` }
    });
  }

  async getPill(pillId: string): Promise<Types.SmartPill> {
    const response = await this.api.get(`/pills/${pillId}`);
    return response.data;
  }

  async getSensorData(pillId: string): Promise<Types.SensorData[]> {
    const response = await this.api.get(`/pills/${pillId}/sensors`);
    return response.data;
  }

  async getTransitMetrics(pillId: string): Promise<Types.TransitMetrics> {
    const response = await this.api.get(`/pills/${pillId}/transit`);
    return response.data;
  }
}

export default SmartPillSDK;
