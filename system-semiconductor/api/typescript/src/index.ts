/**
 * WIA-SEMI-001 TypeScript SDK
 *
 * Version: 1.0.0
 * Standard: WIA-SEMI-001 System Semiconductor
 *
 * © 2025 SmileStory Inc. / WIA
 * 弘益人間 - Benefit All Humanity
 */

import type {
  ChipInfo,
  ChipCapabilities,
  PowerMode,
  PowerModeConfig,
  PowerModeResponse,
  FrequencyConfig,
  ThermalLimitConfig,
  PowerLimitConfig,
  PowerMonitorData,
  TemperatureMonitorData,
  FrequencyMonitorData,
  UtilizationMonitorData,
  TelemetryData,
  WIAError
} from './types';

export * from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface WIASemiChipConfig {
  host: string;
  apiKey?: string;
  accessToken?: string;
  timeout?: number;
  secure?: boolean;
}

// ============================================================================
// Main SDK Class
// ============================================================================

export class WIASemiChip {
  private config: WIASemiChipConfig;
  private baseUrl: string;
  public config: ConfigurationAPI;
  public monitor: MonitoringAPI;

  constructor(config: WIASemiChipConfig) {
    this.config = {
      timeout: 10000,
      secure: true,
      ...config
    };

    const protocol = this.config.secure ? 'https' : 'http';
    this.baseUrl = `${protocol}://${this.config.host}/api/v1`;

    this.config = new ConfigurationAPI(this);
    this.monitor = new MonitoringAPI(this);
  }

  /**
   * Get complete chip information
   */
  async getInfo(): Promise<ChipInfo> {
    return this.request<ChipInfo>('/chip/info');
  }

  /**
   * Get chip capabilities
   */
  async getCapabilities(): Promise<ChipCapabilities> {
    return this.request<ChipCapabilities>('/chip/capabilities');
  }

  /**
   * Get architecture details
   */
  async getArchitecture(): Promise<any> {
    return this.request('/chip/architecture');
  }

  /**
   * Get available interfaces
   */
  async getInterfaces(): Promise<any> {
    return this.request('/chip/interfaces');
  }

  /**
   * Internal request method
   */
  async request<T = any>(
    path: string,
    options: RequestInit = {}
  ): Promise<T> {
    const url = `${this.baseUrl}${path}`;
    const headers: HeadersInit = {
      'Content-Type': 'application/json',
      ...options.headers
    };

    // Add authentication
    if (this.config.apiKey) {
      headers['Authorization'] = `ApiKey ${this.config.apiKey}`;
    } else if (this.config.accessToken) {
      headers['Authorization'] = `Bearer ${this.config.accessToken}`;
    }

    const controller = new AbortController();
    const timeoutId = setTimeout(
      () => controller.abort(),
      this.config.timeout
    );

    try {
      const response = await fetch(url, {
        ...options,
        headers,
        signal: controller.signal
      });

      clearTimeout(timeoutId);

      if (!response.ok) {
        const error: WIAError = await response.json();
        throw new WIASDKError(error);
      }

      return response.json();
    } catch (err) {
      clearTimeout(timeoutId);
      if (err instanceof WIASDKError) {
        throw err;
      }
      throw new Error(`Request failed: ${err.message}`);
    }
  }
}

// ============================================================================
// Configuration API
// ============================================================================

export class ConfigurationAPI {
  constructor(private chip: WIASemiChip) {}

  /**
   * Set power mode
   */
  async setPowerMode(
    mode: PowerMode,
    options?: Partial<PowerModeConfig>
  ): Promise<PowerModeResponse> {
    return this.chip.request<PowerModeResponse>('/config/power-mode', {
      method: 'PUT',
      body: JSON.stringify({
        mode,
        ...options
      })
    });
  }

  /**
   * Set frequency limits
   */
  async setFrequency(config: FrequencyConfig): Promise<any> {
    return this.chip.request('/config/frequency', {
      method: 'PUT',
      body: JSON.stringify(config)
    });
  }

  /**
   * Set thermal limit
   */
  async setThermalLimit(config: ThermalLimitConfig): Promise<any> {
    return this.chip.request('/config/thermal-limit', {
      method: 'PUT',
      body: JSON.stringify(config)
    });
  }

  /**
   * Set power limit
   */
  async setPowerLimit(config: PowerLimitConfig): Promise<any> {
    return this.chip.request('/config/power-limit', {
      method: 'PUT',
      body: JSON.stringify(config)
    });
  }
}

// ============================================================================
// Monitoring API
// ============================================================================

export class MonitoringAPI {
  private eventEmitter: EventTarget;

  constructor(private chip: WIASemiChip) {
    this.eventEmitter = new EventTarget();
  }

  /**
   * Get current power consumption
   */
  async getPower(): Promise<PowerMonitorData> {
    return this.chip.request<PowerMonitorData>('/monitor/power');
  }

  /**
   * Get temperature readings
   */
  async getTemperature(): Promise<TemperatureMonitorData> {
    return this.chip.request<TemperatureMonitorData>('/monitor/temperature');
  }

  /**
   * Get current frequencies
   */
  async getFrequency(): Promise<FrequencyMonitorData> {
    return this.chip.request<FrequencyMonitorData>('/monitor/frequency');
  }

  /**
   * Get utilization percentages
   */
  async getUtilization(): Promise<UtilizationMonitorData> {
    return this.chip.request<UtilizationMonitorData>('/monitor/utilization');
  }

  /**
   * Stream real-time telemetry via WebSocket
   */
  async startStreaming(options: {
    metrics: string[];
    sampleRate: number;
  }): Promise<void> {
    const protocol = this.chip['config'].secure ? 'wss' : 'ws';
    const wsUrl = `${protocol}://${this.chip['config'].host}/api/v1/stream/telemetry`;

    const ws = new WebSocket(wsUrl);

    ws.onopen = () => {
      ws.send(
        JSON.stringify({
          action: 'subscribe',
          metrics: options.metrics,
          sampleRate: options.sampleRate
        })
      );
    };

    ws.onmessage = (event) => {
      const data: TelemetryData = JSON.parse(event.data);
      this.eventEmitter.dispatchEvent(
        new CustomEvent('telemetry', { detail: data })
      );
    };

    ws.onerror = (error) => {
      this.eventEmitter.dispatchEvent(
        new CustomEvent('error', { detail: error })
      );
    };

    return new Promise((resolve) => {
      ws.onopen = () => {
        ws.send(
          JSON.stringify({
            action: 'subscribe',
            metrics: options.metrics,
            sampleRate: options.sampleRate
          })
        );
        resolve();
      };
    });
  }

  /**
   * Register event listener
   */
  on(
    event: 'telemetry' | 'error',
    callback: (data: TelemetryData | Error) => void
  ): void {
    this.eventEmitter.addEventListener(event, ((e: CustomEvent) => {
      callback(e.detail);
    }) as EventListener);
  }

  /**
   * Remove event listener
   */
  off(
    event: 'telemetry' | 'error',
    callback: (data: TelemetryData | Error) => void
  ): void {
    this.eventEmitter.removeEventListener(event, callback as EventListener);
  }
}

// ============================================================================
// Error Class
// ============================================================================

export class WIASDKError extends Error {
  public code: string;
  public details?: Record<string, any>;
  public timestamp: string;
  public requestId?: string;

  constructor(error: WIAError) {
    super(error.message);
    this.name = 'WIASDKError';
    this.code = error.code;
    this.details = error.details;
    this.timestamp = error.timestamp;
    this.requestId = error.requestId;
  }
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Validate chip specification against WIA-SEMI-001 schema
 */
export function validateChipSpec(spec: any): boolean {
  // Basic validation - full implementation would use JSON Schema validator
  return !!(
    spec.$schema &&
    spec.wiaVersion &&
    spec.metadata &&
    spec.architecture
  );
}

/**
 * Convert frequency units
 */
export function convertFrequency(
  value: number,
  from: 'Hz' | 'kHz' | 'MHz' | 'GHz',
  to: 'Hz' | 'kHz' | 'MHz' | 'GHz'
): number {
  const multipliers = { Hz: 1, kHz: 1e3, MHz: 1e6, GHz: 1e9 };
  const hz = value * multipliers[from];
  return hz / multipliers[to];
}

/**
 * Convert power units
 */
export function convertPower(
  value: number,
  from: 'mW' | 'W',
  to: 'mW' | 'W'
): number {
  const multipliers = { mW: 1, W: 1000 };
  const mw = value * multipliers[from];
  return mw / multipliers[to];
}

/**
 * Convert temperature units
 */
export function convertTemperature(
  value: number,
  from: '°C' | '°F' | 'K',
  to: '°C' | '°F' | 'K'
): number {
  // Convert to Celsius first
  let celsius: number;
  if (from === '°C') celsius = value;
  else if (from === '°F') celsius = ((value - 32) * 5) / 9;
  else celsius = value - 273.15;

  // Convert from Celsius to target
  if (to === '°C') return celsius;
  if (to === '°F') return (celsius * 9) / 5 + 32;
  return celsius + 273.15;
}
