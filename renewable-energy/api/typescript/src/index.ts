/**
 * WIA-ENE-004 Renewable Energy Standard - TypeScript SDK
 *
 * @version 1.0.0
 * @license MIT
 * @copyright 2025 SmileStory Inc. / WIA
 *
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */

import axios, { AxiosInstance, AxiosRequestConfig } from 'axios';
import * as types from './types';

export * from './types';

export class RenewableEnergyClient {
  private client: AxiosInstance;
  private config: types.ClientConfig;

  constructor(config: types.ClientConfig) {
    this.config = {
      version: 'v1',
      timeout: 30000,
      retries: 3,
      logLevel: 'info',
      ...config
    };

    this.client = axios.create({
      baseURL: `${this.config.apiEndpoint}/api/${this.config.version}`,
      timeout: this.config.timeout,
      headers: {
        'Content-Type': 'application/json',
        ...(this.config.apiKey && { 'Authorization': `Bearer ${this.config.apiKey}` })
      }
    });

    // Add request interceptor for logging and authentication
    this.client.interceptors.request.use(
      (config) => {
        if (this.config.logLevel === 'debug') {
          console.log(`[WIA-ENE-004] ${config.method?.toUpperCase()} ${config.url}`);
        }
        return config;
      },
      (error) => Promise.reject(error)
    );

    // Add response interceptor for error handling
    this.client.interceptors.response.use(
      (response) => response,
      (error) => {
        const apiError: types.APIError = {
          code: error.response?.data?.error?.code || 'UNKNOWN_ERROR',
          message: error.response?.data?.error?.message || error.message,
          details: error.response?.data?.error?.details,
          documentation: error.response?.data?.error?.documentation,
          requestId: error.response?.headers?.['x-request-id']
        };
        return Promise.reject(apiError);
      }
    );
  }

  // ==================== Energy Sources ====================

  /**
   * List all energy sources
   */
  async listSources(params?: types.ListSourcesParams): Promise<types.ListSourcesResponse> {
    const response = await this.client.get('/sources', { params });
    return response.data;
  }

  /**
   * Get specific energy source
   */
  async getSource(sourceId: string): Promise<types.EnergySource> {
    const response = await this.client.get(`/sources/${sourceId}`);
    return response.data;
  }

  /**
   * Register new energy source
   */
  async createSource(source: Omit<types.EnergySource, 'sourceId' | 'createdAt' | 'updatedAt'>): Promise<types.EnergySource> {
    const response = await this.client.post('/sources', source);
    return response.data;
  }

  /**
   * Update energy source
   */
  async updateSource(sourceId: string, updates: Partial<types.EnergySource>): Promise<types.EnergySource> {
    const response = await this.client.put(`/sources/${sourceId}`, updates);
    return response.data;
  }

  /**
   * Delete energy source
   */
  async deleteSource(sourceId: string): Promise<void> {
    await this.client.delete(`/sources/${sourceId}`);
  }

  // ==================== Production Data ====================

  /**
   * Get production data for a source
   */
  async getProduction(sourceId: string, params?: types.GetProductionParams): Promise<types.GetProductionResponse> {
    const response = await this.client.get(`/sources/${sourceId}/production`, { params });
    return response.data;
  }

  /**
   * Submit production data
   */
  async submitProduction(data: types.ProductionData): Promise<void> {
    await this.client.post(`/sources/${data.sourceId}/production`, data);
  }

  /**
   * Get production summary
   */
  async getProductionSummary(sourceId: string, period?: string): Promise<any> {
    const response = await this.client.get(`/sources/${sourceId}/production/summary`, {
      params: { period }
    });
    return response.data;
  }

  // ==================== System Status ====================

  /**
   * Get source operational status
   */
  async getStatus(sourceId: string): Promise<types.SystemStatus> {
    const response = await this.client.get(`/sources/${sourceId}/status`);
    return response.data;
  }

  /**
   * Update source status
   */
  async updateStatus(sourceId: string, status: types.OperationalState): Promise<types.SystemStatus> {
    const response = await this.client.put(`/sources/${sourceId}/status`, { status });
    return response.data;
  }

  // ==================== Alerts ====================

  /**
   * Get active alerts for a source
   */
  async getAlerts(sourceId: string, severity?: types.AlertSeverity): Promise<types.Alert[]> {
    const response = await this.client.get(`/sources/${sourceId}/alerts`, {
      params: { severity }
    });
    return response.data;
  }

  /**
   * Create alert rule
   */
  async createAlertRule(params: types.CreateAlertRuleParams): Promise<any> {
    const response = await this.client.post('/alerts/rules', params);
    return response.data;
  }

  /**
   * Acknowledge alert
   */
  async acknowledgeAlert(alertId: string, userId: string): Promise<types.Alert> {
    const response = await this.client.put(`/alerts/${alertId}/acknowledge`, { userId });
    return response.data;
  }

  /**
   * Resolve alert
   */
  async resolveAlert(alertId: string, resolution: string): Promise<types.Alert> {
    const response = await this.client.put(`/alerts/${alertId}/resolve`, { resolution });
    return response.data;
  }

  // ==================== Analytics ====================

  /**
   * Get analytics summary
   */
  async getAnalyticsSummary(sourceId?: string): Promise<any> {
    const response = await this.client.get('/analytics/summary', {
      params: { sourceId }
    });
    return response.data;
  }

  /**
   * Get production forecast
   */
  async getForecast(sourceId: string, horizon?: string): Promise<any> {
    const response = await this.client.get(`/sources/${sourceId}/forecast`, {
      params: { horizon }
    });
    return response.data;
  }

  // ==================== Utility Methods ====================

  /**
   * Test API connectivity
   */
  async ping(): Promise<boolean> {
    try {
      const response = await this.client.get('/health');
      return response.status === 200;
    } catch {
      return false;
    }
  }

  /**
   * Get API version info
   */
  async getVersion(): Promise<any> {
    const response = await this.client.get('/version');
    return response.data;
  }
}

// ==================== Convenience Exports ====================

export function createClient(config: types.ClientConfig): RenewableEnergyClient {
  return new RenewableEnergyClient(config);
}

export default RenewableEnergyClient;
