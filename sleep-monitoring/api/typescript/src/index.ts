/**
 * WIA-MED-021 Sleep Monitoring - TypeScript SDK
 * @version 1.0.0
 */

import axios, { AxiosInstance } from 'axios';
import type * as Types from './types';
export * from './types';

export class SleepMonitoringSDK {
  private api: AxiosInstance;

  constructor(config: Types.SDKConfig) {
    this.api = axios.create({
      baseURL: config.baseUrl || 'https://api.wia.org/sleep/v1',
      headers: { 'Authorization': `Bearer ${config.apiKey}` }
    });
  }

  async getSleepSession(sessionId: string): Promise<Types.SleepSession> {
    const response = await this.api.get(`/sessions/${sessionId}`);
    return response.data;
  }

  async recordSleep(data: Omit<Types.SleepSession, 'id'>): Promise<Types.SleepSession> {
    const response = await this.api.post('/sessions', data);
    return response.data;
  }
}

export default SleepMonitoringSDK;
