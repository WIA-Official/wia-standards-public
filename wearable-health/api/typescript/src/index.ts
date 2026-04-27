/**
 * WIA-MED-020: Wearable Health Monitoring Standard - TypeScript SDK
 * 弘益人間 - Benefit All Humanity
 * @version 1.0.0
 * @license MIT
 */

export * from './types';
import { WearableDevice, HealthMetrics, APIResponse } from './types';

export interface WIAWearableConfig {
  baseUrl: string;
  apiKey: string;
  timeout?: number;
}

export class WIAWearable {
  private config: WIAWearableConfig;

  constructor(config: WIAWearableConfig) {
    this.config = { timeout: 30000, ...config };
  }

  async getDevices(): Promise<APIResponse<WearableDevice[]>> {
    return this.request('/api/v1/devices');
  }

  async getMetrics(deviceId: string): Promise<APIResponse<HealthMetrics[]>> {
    return this.request(`/api/v1/devices/${deviceId}/metrics`);
  }

  async syncDevice(deviceId: string): Promise<APIResponse> {
    return this.request(`/api/v1/devices/${deviceId}/sync`, { method: 'POST' });
  }

  private async request(url: string, options: RequestInit = {}): Promise<APIResponse> {
    const fullUrl = `${this.config.baseUrl}${url}`;
    const headers: HeadersInit = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'X-WIA-Standard': 'MED-020',
      ...options.headers
    };
    const response = await fetch(fullUrl, { ...options, headers });
    return response.json();
  }
}

export function createClient(config: WIAWearableConfig): WIAWearable {
  return new WIAWearable(config);
}

export default WIAWearable;
