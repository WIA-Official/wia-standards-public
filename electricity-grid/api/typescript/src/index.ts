/**
 * WIA-SOC-010 Electricity Grid Standard - TypeScript SDK
 * Version: 1.0.0
 *
 * © 2025 SmileStory Inc. / WIA
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */

import axios, { AxiosInstance } from 'axios';
import WebSocket from 'ws';
import * as types from './types';

export * from './types';

/**
 * Main client for interacting with WIA-SOC-010 Electricity Grid API
 */
export class WiaElectricityGrid {
  private client: AxiosInstance;
  private config: types.WiaElectricityGridConfig;

  constructor(config: types.WiaElectricityGridConfig) {
    this.config = {
      secure: true,
      version: 'v1',
      timeout: 30000,
      ...config,
    };

    const protocol = this.config.secure ? 'https' : 'http';
    const baseURL = `${protocol}://${this.config.host}/api/${this.config.version}`;

    this.client = axios.create({
      baseURL,
      timeout: this.config.timeout,
      headers: this.getAuthHeaders(),
    });
  }

  private getAuthHeaders(): Record<string, string> {
    const headers: Record<string, string> = {
      'Content-Type': 'application/json',
    };

    if (this.config.bearerToken) {
      headers['Authorization'] = `Bearer ${this.config.bearerToken}`;
    } else if (this.config.apiKey) {
      headers['X-API-Key'] = this.config.apiKey;
    }

    return headers;
  }

  // ============================================================================
  // System Information
  // ============================================================================

  /**
   * Get grid operator system information
   */
  async getSystemInfo(): Promise<types.SystemInfo> {
    const response = await this.client.get('/system/info');
    return response.data;
  }

  // ============================================================================
  // Grid Status
  // ============================================================================

  /**
   * Get current grid operational status
   */
  async getGridStatus(query?: types.GridStatusQuery): Promise<types.GridStatus> {
    const response = await this.client.get('/grid-status', { params: query });
    return response.data;
  }

  // ============================================================================
  // Renewable Energy
  // ============================================================================

  /**
   * Get current renewable generation data
   */
  async getRenewableGeneration(
    query?: types.RenewablesQuery
  ): Promise<types.RenewableGeneration> {
    const response = await this.client.get('/renewables/current', { params: query });
    return response.data;
  }

  /**
   * Get renewable energy forecast
   */
  async getRenewableForecast(
    query: types.RenewableForecastQuery
  ): Promise<types.RenewableForecast> {
    const response = await this.client.get('/renewables/forecast', { params: query });
    return response.data;
  }

  // ============================================================================
  // Energy Storage
  // ============================================================================

  /**
   * Get energy storage system status
   */
  async getStorageStatus(): Promise<types.EnergyStorage[]> {
    const response = await this.client.get('/storage/status');
    return response.data.systems || [];
  }

  /**
   * Dispatch storage system charging/discharging
   */
  async dispatchStorage(
    dispatch: types.StorageDispatch
  ): Promise<types.StorageDispatchResponse> {
    const response = await this.client.post('/storage/dispatch', dispatch);
    return response.data;
  }

  // ============================================================================
  // Demand Response
  // ============================================================================

  /**
   * Get demand response events
   */
  async getDemandResponseEvents(
    status?: 'scheduled' | 'active' | 'completed'
  ): Promise<types.DemandResponseEvent[]> {
    const response = await this.client.get('/demand-response/events', {
      params: { status },
    });
    return response.data.events || [];
  }

  /**
   * Create new demand response event
   */
  async createDemandResponseEvent(
    event: types.CreateDREventRequest
  ): Promise<types.DemandResponseEvent> {
    const response = await this.client.post('/demand-response/events', event);
    return response.data;
  }

  // ============================================================================
  // Power Quality
  // ============================================================================

  /**
   * Get current power quality metrics
   */
  async getPowerQuality(
    query?: types.PowerQualityQuery
  ): Promise<types.PowerQualityMetrics> {
    const response = await this.client.get('/power-quality/current', { params: query });
    return response.data;
  }

  // ============================================================================
  // Smart Meters
  // ============================================================================

  /**
   * Get smart meter consumption data
   */
  async getMeterConsumption(
    meterId: string,
    query: types.MeterConsumptionQuery
  ): Promise<types.MeterReading[]> {
    const response = await this.client.get(`/meters/${meterId}/consumption`, {
      params: query,
    });
    return response.data.data || [];
  }

  // ============================================================================
  // Alerts
  // ============================================================================

  /**
   * Get system alerts and notifications
   */
  async getAlerts(query?: types.AlertQuery): Promise<types.Alert[]> {
    const response = await this.client.get('/alerts', { params: query });
    return response.data.alerts || [];
  }

  /**
   * Acknowledge an alert
   */
  async acknowledgeAlert(alertId: string): Promise<void> {
    await this.client.post(`/alerts/${alertId}/acknowledge`);
  }

  // ============================================================================
  // WebSocket / Real-time Events
  // ============================================================================

  /**
   * Subscribe to real-time events via WebSocket
   */
  subscribeToEvents(
    callback: (message: types.WebSocketMessage) => void,
    options: types.WebSocketOptions
  ): WebSocket {
    const protocol = this.config.secure ? 'wss' : 'ws';
    const wsUrl = `${protocol}://${this.config.host}/api/${this.config.version}/stream`;

    const ws = new WebSocket(wsUrl);

    ws.on('open', () => {
      // Authenticate
      if (this.config.bearerToken) {
        const authMessage: types.WebSocketAuth = {
          type: 'auth',
          token: `Bearer ${this.config.bearerToken}`,
        };
        ws.send(JSON.stringify(authMessage));
      }

      // Subscribe to channels
      const subscribeMessage: types.WebSocketSubscribe = {
        type: 'subscribe',
        channels: options.channels,
      };
      ws.send(JSON.stringify(subscribeMessage));
    });

    ws.on('message', (data: WebSocket.Data) => {
      try {
        const message: types.WebSocketMessage = JSON.parse(data.toString());
        callback(message);
      } catch (error) {
        console.error('Failed to parse WebSocket message:', error);
      }
    });

    ws.on('close', () => {
      if (options.autoReconnect !== false) {
        const delay = options.reconnectDelay || 5000;
        setTimeout(() => {
          this.subscribeToEvents(callback, options);
        }, delay);
      }
    });

    ws.on('error', (error) => {
      console.error('WebSocket error:', error);
    });

    return ws;
  }

  // ============================================================================
  // Batch Operations
  // ============================================================================

  /**
   * Execute multiple operations in a single request
   */
  async batchOperations(
    operations: Array<{ method: string; path: string; body?: any }>
  ): Promise<any[]> {
    const response = await this.client.post('/batch', { operations });
    return response.data;
  }
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Validate grid frequency is within acceptable range
 */
export function isFrequencyNormal(frequency: number, nominal: number = 60): boolean {
  const deviation = Math.abs(frequency - nominal);
  const maxDeviation = nominal * 0.05; // ±5%
  return deviation <= maxDeviation;
}

/**
 * Validate voltage is within acceptable range
 */
export function isVoltageNormal(voltage: number, nominal: number): boolean {
  const deviation = Math.abs(voltage - nominal);
  const maxDeviation = nominal * 0.1; // ±10%
  return deviation <= maxDeviation;
}

/**
 * Calculate renewable penetration rate
 */
export function calculateRenewablePenetration(
  renewableGeneration: number,
  totalLoad: number
): number {
  if (totalLoad === 0) return 0;
  return renewableGeneration / totalLoad;
}

/**
 * Estimate battery duration at current power flow
 */
export function estimateBatteryDuration(
  energyCapacity: number,
  stateOfCharge: number,
  powerFlow: number
): number {
  if (powerFlow >= 0) return Infinity; // Charging or idle
  const availableEnergy = energyCapacity * (stateOfCharge / 100);
  return availableEnergy / Math.abs(powerFlow);
}

/**
 * Format power value with appropriate unit
 */
export function formatPower(watts: number): string {
  if (watts >= 1e9) return `${(watts / 1e9).toFixed(2)} GW`;
  if (watts >= 1e6) return `${(watts / 1e6).toFixed(2)} MW`;
  if (watts >= 1e3) return `${(watts / 1e3).toFixed(2)} kW`;
  return `${watts.toFixed(2)} W`;
}

/**
 * Format energy value with appropriate unit
 */
export function formatEnergy(wattHours: number): string {
  if (wattHours >= 1e9) return `${(wattHours / 1e9).toFixed(2)} GWh`;
  if (wattHours >= 1e6) return `${(wattHours / 1e6).toFixed(2)} MWh`;
  if (wattHours >= 1e3) return `${(wattHours / 1e3).toFixed(2)} kWh`;
  return `${wattHours.toFixed(2)} Wh`;
}

// Default export
export default WiaElectricityGrid;
