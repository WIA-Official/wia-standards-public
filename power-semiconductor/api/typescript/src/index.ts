/**
 * WIA-SEMI-003 Power Semiconductor SDK
 *
 * TypeScript/JavaScript SDK for interacting with WIA-SEMI-003 compliant
 * power semiconductor devices.
 *
 * @packageDocumentation
 */

import axios, { AxiosInstance, AxiosError } from 'axios';
import Ajv from 'ajv';

export * from './types';
import type {
  PowerSemiconductorDevice,
  TelemetryData,
  ControlCommand,
  ApiResponse,
  ClientConfig
} from './types';

/**
 * Main client class for WIA-SEMI-003 power semiconductor devices
 */
export class PowerSemiconductorClient {
  private client: AxiosInstance;
  private validator: Ajv;
  private config: Required<ClientConfig>;

  /**
   * Create a new PowerSemiconductorClient
   *
   * @param config - Client configuration
   *
   * @example
   * ```typescript
   * const client = new PowerSemiconductorClient({
   *   baseUrl: 'https://device.example.com',
   *   apiKey: 'your-api-key',
   *   timeout: 5000
   * });
   * ```
   */
  constructor(config: ClientConfig) {
    this.config = {
      timeout: 5000,
      retryAttempts: 3,
      ...config
    };

    this.client = axios.create({
      baseURL: this.config.baseUrl,
      timeout: this.config.timeout,
      headers: {
        'Content-Type': 'application/json',
        ...(this.config.apiKey && { 'Authorization': `Bearer ${this.config.apiKey}` })
      }
    });

    this.validator = new Ajv();

    // Setup request/response interceptors
    this.setupInterceptors();
  }

  /**
   * Setup axios interceptors for error handling and retries
   */
  private setupInterceptors(): void {
    this.client.interceptors.response.use(
      response => response,
      async (error: AxiosError) => {
        const config = error.config;
        if (!config || !config.headers) {
          return Promise.reject(error);
        }

        // Retry logic for transient errors
        const retryCount = (config.headers['x-retry-count'] as number) || 0;
        if (retryCount < this.config.retryAttempts && this.isRetryableError(error)) {
          config.headers['x-retry-count'] = retryCount + 1;
          await this.delay(Math.pow(2, retryCount) * 1000);
          return this.client.request(config);
        }

        return Promise.reject(error);
      }
    );
  }

  /**
   * Check if error is retryable
   */
  private isRetryableError(error: AxiosError): boolean {
    return !error.response || (error.response.status >= 500 && error.response.status < 600);
  }

  /**
   * Delay helper for retries
   */
  private delay(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }

  /**
   * Get complete device specification
   *
   * @returns Device specification conforming to WIA-SEMI-003
   *
   * @example
   * ```typescript
   * const device = await client.getDeviceInfo();
   * console.log(device.metadata.deviceType);
   * console.log(device.electrical.voltageRatings.vds_max);
   * ```
   */
  async getDeviceInfo(): Promise<PowerSemiconductorDevice> {
    const response = await this.client.get<ApiResponse<PowerSemiconductorDevice>>('/api/v1/device/info');
    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to get device info');
    }
    return response.data.data;
  }

  /**
   * Get current real-time telemetry data
   *
   * @returns Current device telemetry
   *
   * @example
   * ```typescript
   * const telemetry = await client.getCurrentTelemetry();
   * console.log(`Temperature: ${telemetry.measurements.junctionTemperature}°C`);
   * console.log(`Current: ${telemetry.measurements.drainCurrent}A`);
   * ```
   */
  async getCurrentTelemetry(): Promise<TelemetryData> {
    const response = await this.client.get<ApiResponse<TelemetryData>>('/api/v1/telemetry/current');
    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to get telemetry');
    }
    return response.data.data;
  }

  /**
   * Get historical telemetry data
   *
   * @param start - Start timestamp (ISO 8601)
   * @param end - End timestamp (ISO 8601)
   * @param interval - Sample interval in seconds
   * @returns Array of historical telemetry data
   *
   * @example
   * ```typescript
   * const history = await client.getTelemetryHistory(
   *   '2025-01-01T00:00:00Z',
   *   '2025-01-02T00:00:00Z',
   *   60 // 1 minute intervals
   * );
   * ```
   */
  async getTelemetryHistory(
    start: string,
    end: string,
    interval: number = 60
  ): Promise<TelemetryData[]> {
    const response = await this.client.get<ApiResponse<TelemetryData[]>>(
      '/api/v1/telemetry/history',
      {
        params: { start, end, interval }
      }
    );
    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to get telemetry history');
    }
    return response.data.data;
  }

  /**
   * Send control command to device
   *
   * @param command - Control command
   * @returns Command execution result
   *
   * @example
   * ```typescript
   * await client.sendCommand({
   *   deviceId: 'DEVICE-001',
   *   command: 'setGateVoltage',
   *   parameters: { voltage: 15 },
   *   timestamp: new Date().toISOString()
   * });
   * ```
   */
  async sendCommand(command: ControlCommand): Promise<void> {
    const response = await this.client.post<ApiResponse<void>>('/api/v1/control/command', command);
    if (!response.data.success) {
      throw new Error(response.data.error?.message || 'Command execution failed');
    }
  }

  /**
   * Calculate power loss based on operating conditions
   *
   * @param voltage - Operating voltage (V)
   * @param current - Operating current (A)
   * @param frequency - Switching frequency (Hz)
   * @param temperature - Junction temperature (°C)
   * @returns Calculated power loss (W)
   *
   * @example
   * ```typescript
   * const loss = await client.calculatePowerLoss(600, 50, 20000, 125);
   * console.log(`Total power loss: ${loss.total}W`);
   * ```
   */
  async calculatePowerLoss(
    voltage: number,
    current: number,
    frequency: number,
    temperature: number
  ): Promise<{
    conduction: number;
    switching: number;
    total: number;
  }> {
    const response = await this.client.post<ApiResponse<any>>(
      '/api/v1/analysis/power-loss',
      { voltage, current, frequency, temperature }
    );
    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Power loss calculation failed');
    }
    return response.data.data;
  }

  /**
   * Validate device data against WIA-SEMI-003 schema
   *
   * @param device - Device specification to validate
   * @returns Validation result
   */
  validateDeviceData(device: PowerSemiconductorDevice): {
    valid: boolean;
    errors?: any[];
  } {
    // In a real implementation, this would validate against the JSON schema
    const requiredFields = ['metadata', 'electrical', 'thermal', 'switching'];
    const hasRequiredFields = requiredFields.every(field => field in device);

    if (!hasRequiredFields) {
      return {
        valid: false,
        errors: ['Missing required fields']
      };
    }

    return { valid: true };
  }

  /**
   * Subscribe to real-time telemetry stream via WebSocket
   *
   * @param callback - Callback function for telemetry updates
   * @param channels - Channels to subscribe to
   * @returns Unsubscribe function
   *
   * @example
   * ```typescript
   * const unsubscribe = client.subscribeToTelemetry(
   *   (data) => console.log('New telemetry:', data),
   *   ['temperature', 'current', 'voltage']
   * );
   *
   * // Later: unsubscribe();
   * ```
   */
  subscribeToTelemetry(
    callback: (data: TelemetryData) => void,
    channels: string[] = ['temperature', 'current', 'voltage']
  ): () => void {
    const wsUrl = this.config.baseUrl.replace(/^http/, 'ws') + '/api/v1/stream';
    const ws = new WebSocket(wsUrl);

    ws.onopen = () => {
      ws.send(JSON.stringify({
        subscribe: channels,
        sampleRate: 1000
      }));
    };

    ws.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        callback(data);
      } catch (error) {
        console.error('Failed to parse telemetry data:', error);
      }
    };

    ws.onerror = (error) => {
      console.error('WebSocket error:', error);
    };

    return () => {
      ws.close();
    };
  }
}

/**
 * Factory function to create a PowerSemiconductorClient instance
 *
 * @param config - Client configuration
 * @returns Configured client instance
 */
export function createClient(config: ClientConfig): PowerSemiconductorClient {
  return new PowerSemiconductorClient(config);
}

// Default export
export default PowerSemiconductorClient;
