/**
 * WIA-ENE-005 Solar Energy Standard - TypeScript SDK
 * © 2025 SmileStory Inc. / WIA
 * 弘益人間 (Benefit All Humanity)
 */

import {
  ClientConfig,
  SiteProfile,
  ProductionData,
  InverterMetrics,
  BatteryStatus,
  EnvironmentalData,
  SystemStatus,
  PerformanceMetrics,
  APIResponse,
  ProductionHistory
} from './types';

export * from './types';

export class SolarEnergyClient {
  private apiKey: string;
  private baseUrl: string;
  private timeout: number;
  private standard: string;

  constructor(config: ClientConfig) {
    this.apiKey = config.apiKey;
    this.baseUrl = config.baseUrl;
    this.timeout = config.timeout || 30000;
    this.standard = config.standard || 'WIA-ENE-005';
  }

  private async request<T>(
    endpoint: string,
    options: RequestInit = {}
  ): Promise<APIResponse<T>> {
    const url = `${this.baseUrl}${endpoint}`;
    const headers = {
      'Authorization': `Bearer ${this.apiKey}`,
      'Content-Type': 'application/json',
      'X-WIA-Standard': this.standard,
      ...options.headers
    };

    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), this.timeout);

    try {
      const response = await fetch(url, {
        ...options,
        headers,
        signal: controller.signal
      });

      const data = await response.json();
      return data as APIResponse<T>;
    } catch (error) {
      throw new Error(`API request failed: ${error instanceof Error ? error.message : 'Unknown error'}`);
    } finally {
      clearTimeout(timeoutId);
    }
  }

  async getSites(): Promise<APIResponse<SiteProfile[]>> {
    return this.request<SiteProfile[]>('/api/v1/sites');
  }

  async getSite(siteId: string): Promise<APIResponse<SiteProfile>> {
    return this.request<SiteProfile>(`/api/v1/sites/${siteId}`);
  }

  async getProduction(systemId: string): Promise<APIResponse<ProductionData>> {
    return this.request<ProductionData>(`/api/v1/systems/${systemId}/production`);
  }

  async getProductionHistory(
    systemId: string,
    params: { start: string; end: string; resolution?: '1m' | '5m' | '15m' | '1h' | '1d'; }
  ): Promise<APIResponse<ProductionHistory>> {
    const queryParams = new URLSearchParams({
      start: params.start,
      end: params.end,
      resolution: params.resolution || '1h'
    });
    return this.request<ProductionHistory>(
      `/api/v1/systems/${systemId}/production/history?${queryParams}`
    );
  }

  async getInverterMetrics(inverterId: string): Promise<APIResponse<InverterMetrics>> {
    return this.request<InverterMetrics>(`/api/v1/inverters/${inverterId}/metrics`);
  }

  async getBatteryStatus(batteryId: string): Promise<APIResponse<BatteryStatus>> {
    return this.request<BatteryStatus>(`/api/v1/battery/${batteryId}/status`);
  }

  async getEnvironmentalData(systemId: string): Promise<APIResponse<EnvironmentalData>> {
    return this.request<EnvironmentalData>(`/api/v1/systems/${systemId}/environmental`);
  }

  async getSystemStatus(systemId: string): Promise<APIResponse<SystemStatus>> {
    return this.request<SystemStatus>(`/api/v1/systems/${systemId}/status`);
  }

  async getPerformanceMetrics(systemId: string): Promise<APIResponse<PerformanceMetrics>> {
    return this.request<PerformanceMetrics>(`/api/v1/systems/${systemId}/performance`);
  }
}
