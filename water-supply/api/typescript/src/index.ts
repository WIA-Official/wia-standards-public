/**
 * WIA-SOC-008: Water Supply Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

export * from './types';

import {
  SystemIdentity,
  WaterQuality,
  HydraulicData,
  LeakEvent,
  NetworkTopology,
  ConsumptionRecord,
  Alert,
  ApiConfig,
  PaginatedResponse,
  ErrorResponse,
  UUID
} from './types';

/**
 * Main WIA Water Supply API Client
 */
export class WiaWaterSupply {
  private config: ApiConfig;
  private baseUrl: string;

  constructor(config: ApiConfig) {
    this.config = {
      protocol: 'https',
      port: 443,
      timeout: 30000,
      ...config
    };
    this.baseUrl = `${this.config.protocol}://${this.config.host}:${this.config.port}/wia/soc-008/v1`;
  }

  /**
   * Get system information
   */
  async getSystemInfo(): Promise<SystemIdentity> {
    return this.request<SystemIdentity>('/system/info');
  }

  /**
   * Get current water quality readings
   */
  async getWaterQuality(params?: {
    zoneId?: string;
    stationId?: string;
    parameters?: string[];
  }): Promise<PaginatedResponse<WaterQuality>> {
    const queryParams = new URLSearchParams();
    if (params?.zoneId) queryParams.set('zoneId', params.zoneId);
    if (params?.stationId) queryParams.set('stationId', params.stationId);
    if (params?.parameters) queryParams.set('parameters', params.parameters.join(','));
    
    return this.request<PaginatedResponse<WaterQuality>>(`/water-quality/current?${queryParams}`);
  }

  /**
   * Get historical water quality data
   */
  async getWaterQualityHistory(params: {
    startDate: string;
    endDate: string;
    stationId?: string;
    parameters?: string[];
    interval?: '5min' | '15min' | '1hour' | '1day';
    limit?: number;
  }): Promise<PaginatedResponse<WaterQuality>> {
    const queryParams = new URLSearchParams({
      startDate: params.startDate,
      endDate: params.endDate
    });
    if (params.stationId) queryParams.set('stationId', params.stationId);
    if (params.parameters) queryParams.set('parameters', params.parameters.join(','));
    if (params.interval) queryParams.set('interval', params.interval);
    if (params.limit) queryParams.set('limit', params.limit.toString());
    
    return this.request<PaginatedResponse<WaterQuality>>(`/water-quality/history?${queryParams}`);
  }

  /**
   * Get network status
   */
  async getNetworkStatus(): Promise<{
    timestamp: string;
    overall: {
      status: string;
      efficiency: number;
      waterLoss: number;
    };
    zones: any[];
    alerts: number;
    warnings: number;
  }> {
    return this.request('/network/status');
  }

  /**
   * Get active leaks
   */
  async getActiveLeaks(): Promise<PaginatedResponse<LeakEvent>> {
    return this.request<PaginatedResponse<LeakEvent>>('/leaks/active');
  }

  /**
   * Report a new leak
   */
  async reportLeak(data: {
    reportedBy: 'citizen' | 'sensor' | 'patrol';
    location: {
      coordinates?: { lat: number; lon: number };
      address?: string;
      description?: string;
    };
    urgency: 'low' | 'medium' | 'high';
    contactInfo?: {
      phone?: string;
      email?: string;
    };
  }): Promise<{
    reportId: string;
    status: string;
    estimatedResponse: string;
    trackingUrl: string;
  }> {
    return this.request('/leaks/report', {
      method: 'POST',
      body: JSON.stringify(data)
    });
  }

  /**
   * Get consumption data for a meter
   */
  async getMeterConsumption(
    meterId: UUID,
    params?: {
      startDate?: string;
      endDate?: string;
      interval?: 'hourly' | 'daily' | 'monthly';
    }
  ): Promise<{
    meterId: UUID;
    customerId: string;
    period: { start: string; end: string };
    consumption: {
      total: number;
      average: number;
      peak: number;
      unit: string;
    };
    readings: ConsumptionRecord[];
    billing: any;
  }> {
    const queryParams = new URLSearchParams();
    if (params?.startDate) queryParams.set('startDate', params.startDate);
    if (params?.endDate) queryParams.set('endDate', params.endDate);
    if (params?.interval) queryParams.set('interval', params.interval);
    
    return this.request(`/meters/${meterId}/consumption?${queryParams}`);
  }

  /**
   * Get system alerts
   */
  async getAlerts(params?: {
    severity?: 'info' | 'warning' | 'error' | 'critical';
    status?: 'active' | 'acknowledged' | 'resolved';
    limit?: number;
  }): Promise<PaginatedResponse<Alert>> {
    const queryParams = new URLSearchParams();
    if (params?.severity) queryParams.set('severity', params.severity);
    if (params?.status) queryParams.set('status', params.status);
    if (params?.limit) queryParams.set('limit', params.limit.toString());
    
    return this.request<PaginatedResponse<Alert>>(`/alerts?${queryParams}`);
  }

  /**
   * Acknowledge an alert
   */
  async acknowledgeAlert(
    alertId: UUID,
    data: {
      acknowledgedBy: string;
      comment?: string;
      estimatedResolution?: string;
    }
  ): Promise<void> {
    await this.request(`/alerts/${alertId}/acknowledge`, {
      method: 'POST',
      body: JSON.stringify(data)
    });
  }

  /**
   * Subscribe to real-time events via WebSocket
   */
  subscribeToEvents(
    callback: (event: any) => void,
    options?: {
      channels?: string[];
      filters?: any;
    }
  ): WebSocket {
    const wsUrl = this.baseUrl.replace(/^http/, 'ws') + '/stream';
    const ws = new WebSocket(wsUrl);

    ws.onopen = () => {
      ws.send(JSON.stringify({
        action: 'subscribe',
        channels: options?.channels || ['water-quality', 'leaks', 'alerts'],
        filters: options?.filters || {}
      }));
    };

    ws.onmessage = (event) => {
      const data = JSON.parse(event.data);
      callback(data);
    };

    return ws;
  }

  /**
   * Internal request method
   */
  private async request<T>(path: string, options: RequestInit = {}): Promise<T> {
    const url = `${this.baseUrl}${path}`;
    const headers: HeadersInit = {
      'Content-Type': 'application/json',
      'X-WIA-Version': '1.0.0',
      ...options.headers
    };

    if (this.config.bearerToken) {
      headers['Authorization'] = `Bearer ${this.config.bearerToken}`;
    } else if (this.config.apiKey) {
      headers['X-API-Key'] = this.config.apiKey;
    }

    const response = await fetch(url, {
      ...options,
      headers,
      signal: AbortSignal.timeout(this.config.timeout!)
    });

    if (!response.ok) {
      const error: ErrorResponse = await response.json();
      throw new Error(`API Error: ${error.error.message}`);
    }

    return response.json();
  }
}

/**
 * Example usage:
 * 
 * const client = new WiaWaterSupply({
 *   host: 'api.water-utility.com',
 *   bearerToken: 'your-token-here'
 * });
 * 
 * const quality = await client.getWaterQuality({ zoneId: 'ZONE-A' });
 * const leaks = await client.getActiveLeaks();
 * 
 * const ws = client.subscribeToEvents((event) => {
 *   console.log('Event received:', event);
 * });
 */
