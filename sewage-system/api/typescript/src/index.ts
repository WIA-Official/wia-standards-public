/**
 * WIA-SOC-009 Sewage System Standard API
 * TypeScript Implementation
 *
 * 弘익人間 (홍익인간) - Benefit All Humanity
 *
 * @packageDocumentation
 * @module wia-soc-009
 * @license MIT
 * @author WIA / SmileStory Inc.
 */

import * as types from './types';

// Re-export all types
export * from './types';

/**
 * Main API client for WIA-SOC-009 sewage systems
 */
export class WiaSewageSystem {
  private baseUrl: string;
  private token?: string;
  private timeout: number;

  constructor(options: types.ApiClientOptions) {
    const protocol = options.protocol || 'http';
    const port = options.port || 8080;
    this.baseUrl = `${protocol}://${options.host}:${port}/api/v1`;
    this.token = options.token;
    this.timeout = options.timeout || 5000;
  }

  async connect(): Promise<boolean> {
    try {
      const response = await this.fetch('/health');
      return response.status === 'healthy';
    } catch (error) {
      throw new Error(`Failed to connect: ${error}`);
    }
  }

  async getSystemInfo(): Promise<types.SystemInfo> {
    return await this.fetch('/system/info');
  }

  async getSystemStatus(): Promise<types.SystemState> {
    return await this.fetch('/system/status');
  }

  async getSensorReadings(params?: {
    sensorId?: string;
    type?: types.SensorType;
    zone?: string;
    from?: string;
    to?: string;
    limit?: number;
  }): Promise<types.SensorReading[]> {
    const query = new URLSearchParams(params as any).toString();
    const response = await this.fetch(`/sensors/readings?${query}`);
    return response.results || response;
  }

  async getWaterQuality(location?: string): Promise<types.WaterQualityReading> {
    const url = location ? `/water-quality/current?location=${location}` : '/water-quality/current';
    return await this.fetch(url);
  }

  async getFlowData(): Promise<types.FlowReading[]> {
    const response = await this.fetch('/flow/current');
    return response.monitoringPoints || [];
  }

  async getTreatmentStatus(): Promise<types.TreatmentProcess[]> {
    const response = await this.fetch('/treatment/status');
    return response.stages || [];
  }

  async getActiveAlerts(severity?: types.AlertSeverity): Promise<types.SystemAlert[]> {
    const url = severity ? `/alerts/active?severity=${severity}` : '/alerts/active';
    const response = await this.fetch(url);
    return response.alerts || [];
  }

  async acknowledgeAlert(alertId: string, acknowledgedBy: string, notes?: string): Promise<types.CommandResponse> {
    return await this.fetch(`/alerts/${alertId}/acknowledge`, {
      method: 'POST',
      body: JSON.stringify({ acknowledgedBy, notes })
    });
  }

  async getFlowEvents(from: string, to: string): Promise<types.FlowEvent[]> {
    const response = await this.fetch(`/events/history?from=${from}&to=${to}&type=overflow`);
    return response.events || [];
  }

  async generateComplianceReport(period: string, startDate: string, endDate: string, format: string = 'json'): Promise<types.CommandResponse> {
    return await this.fetch('/reports/compliance', {
      method: 'POST',
      body: JSON.stringify({ period, startDate, endDate, format })
    });
  }

  async getPerformanceMetrics(period: string, startDate: string, endDate?: string): Promise<types.PerformanceMetrics> {
    const params = new URLSearchParams({ period, startDate });
    if (endDate) params.append('endDate', endDate);
    return await this.fetch(`/metrics/performance?${params.toString()}`);
  }

  async getOverflowPredictions(lookahead: number = 24): Promise<types.OverflowPrediction[]> {
    const response = await this.fetch(`/predictions/overflow?lookahead=${lookahead}`);
    return response.predictions || [];
  }

  async getEquipmentFailurePredictions(): Promise<types.EquipmentFailurePrediction[]> {
    const response = await this.fetch('/predictions/equipment');
    return response.predictions || [];
  }

  async getPumpStatus(pumpId?: string): Promise<types.PumpStatus | types.PumpStatus[]> {
    const url = pumpId ? `/equipment/pumps/${pumpId}` : '/equipment/pumps';
    return await this.fetch(url);
  }

  subscribeToEvents(callback: types.EventCallback, options?: types.WebSocketOptions): () => void {
    const wsUrl = this.baseUrl.replace(/^http/, 'ws').replace('/api/v1', '/ws');
    const ws = new WebSocket(wsUrl);

    ws.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        if (data.type === 'data' || data.type === 'event') {
          callback(data as types.SystemEvent);
        }
      } catch (error) {
        console.error('Failed to parse WebSocket message:', error);
      }
    };

    ws.onopen = () => {
      ws.send(JSON.stringify({
        action: 'subscribe',
        channels: ['flow', 'water_quality', 'alerts', 'equipment']
      }));
    };

    if (options?.reconnect) {
      let reconnectAttempts = 0;
      ws.onclose = () => {
        if (reconnectAttempts < (options.maxReconnectAttempts || Infinity)) {
          setTimeout(() => {
            reconnectAttempts++;
            this.subscribeToEvents(callback, options);
          }, options.reconnectInterval || 5000);
        }
      };
    }

    return () => ws.close();
  }

  private async fetch(endpoint: string, options: RequestInit = {}): Promise<any> {
    const headers: HeadersInit = {
      'Content-Type': 'application/json',
      ...options.headers
    };

    if (this.token) {
      headers['Authorization'] = `Bearer ${this.token}`;
    }

    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), this.timeout);

    try {
      const response = await fetch(`${this.baseUrl}${endpoint}`, {
        ...options,
        headers,
        signal: controller.signal
      });

      clearTimeout(timeoutId);

      if (!response.ok) {
        const error = await response.json();
        throw new Error(error.error?.message || `HTTP ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      clearTimeout(timeoutId);
      throw error;
    }
  }
}

export function createSewageSystem(options: types.ApiClientOptions): WiaSewageSystem {
  return new WiaSewageSystem(options);
}

export const VERSION = '1.0.0';
export default WiaSewageSystem;
