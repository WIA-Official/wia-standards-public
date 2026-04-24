/**
 * WIA BCI (Brain-Computer Interface) Standard API
 * TypeScript Implementation
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @packageDocumentation
 * @module wia-bci
 * @license MIT
 * @author WIA / SmileStory Inc.
 * @version 1.0.0
 *
 * @example
 * ```typescript
 * import { WiaBci, BCISDK } from 'wia-bci';
 *
 * const sdk = new BCISDK({ apiKey: 'xxx', endpoint: 'https://api.wia.org' });
 * const session = await sdk.createSession({ userId: 'user-1' });
 * ```
 */

// Export all types
export * from './types';

// Core classes
export { WiaBci } from './core/WiaBci';
export { BciEventEmitter } from './core/EventEmitter';
export { BciError, ErrorCodes, type ErrorCode } from './core/BciError';
export { SignalProcessor } from './core/SignalProcessor';

// Adapters
export { BaseAdapter, type IBciAdapter } from './adapters/BaseAdapter';
export { SimulatorAdapter } from './adapters/SimulatorAdapter';

// Types - Config
export type {
  DeviceType,
  ConnectionProtocol,
  LogLevel,
  FilterType,
  FilterConfig,
  WiaBciOptions,
  DeviceConfig,
  DeviceIdentifier,
  ConnectionConfig,
  AcquisitionConfig,
} from './types/config';

// Types - Device
export type {
  DeviceStatus,
  DeviceCapabilities,
  DeviceInfo,
  ChannelInfo,
  BciState,
} from './types/device';

// Types - Events
export type {
  EventType,
  SignalEvent,
  MarkerEvent,
  ClassificationEvent,
  ChannelQuality,
  QualityEvent,
  ErrorEvent,
  EventDataMap,
  EventHandler,
} from './types/events';

// Types - Signal
export type {
  FrequencyBand,
  BandPowers,
  PowerSpectrum,
  ComplexNumber,
  ComplexArray,
  HjorthParams,
  FeatureVector,
  ArtifactType,
  ArtifactSegment,
  RecordingInfo,
  SessionInfo,
  WiaBciRecording,
} from './types/signal';

// Protocol
export * from './protocol';

// Transport
export * from './transport';

// Output
export * from './output';

// Constants
export { STANDARD_10_20_CHANNELS, STANDARD_10_10_CHANNELS } from './types/device';
export { FREQUENCY_BANDS } from './types/signal';
export { DEFAULT_OPTIONS } from './types/config';

import {
  ApiResponse,
  PaginatedResponse,
  BciSession,
  DeviceInfo,
  SignalPacket,
  SessionMetrics,
} from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface BCISDKConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  retries?: number;
  debug?: boolean;
}

// ============================================================================
// SDK Client
// ============================================================================

export class BCISDK {
  private config: Required<BCISDKConfig>;
  private headers: Record<string, string>;

  constructor(config: BCISDKConfig) {
    this.config = {
      timeout: 30000,
      retries: 3,
      debug: false,
      ...config,
    };

    this.headers = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'X-WIA-Standard': 'BCI',
      'X-WIA-Version': '1.0.0',
    };
  }

  // ==========================================================================
  // Device Management APIs
  // ==========================================================================

  async listDevices(): Promise<ApiResponse<DeviceInfo[]>> {
    return this.get<DeviceInfo[]>('/api/v1/devices');
  }

  async getDevice(deviceId: string): Promise<ApiResponse<DeviceInfo>> {
    return this.get<DeviceInfo>(`/api/v1/devices/${deviceId}`);
  }

  async connectDevice(deviceId: string): Promise<ApiResponse<DeviceInfo>> {
    return this.post<DeviceInfo>(`/api/v1/devices/${deviceId}/connect`, {});
  }

  async disconnectDevice(deviceId: string): Promise<ApiResponse<void>> {
    return this.post<void>(`/api/v1/devices/${deviceId}/disconnect`, {});
  }

  // ==========================================================================
  // Session Management APIs
  // ==========================================================================

  async createSession(params: { userId: string; deviceId: string }): Promise<ApiResponse<BciSession>> {
    return this.post<BciSession>('/api/v1/sessions', params);
  }

  async getSession(sessionId: string): Promise<ApiResponse<BciSession>> {
    return this.get<BciSession>(`/api/v1/sessions/${sessionId}`);
  }

  async endSession(sessionId: string): Promise<ApiResponse<BciSession>> {
    return this.post<BciSession>(`/api/v1/sessions/${sessionId}/end`, {});
  }

  async getSessionMetrics(sessionId: string): Promise<ApiResponse<SessionMetrics>> {
    return this.get<SessionMetrics>(`/api/v1/sessions/${sessionId}/metrics`);
  }

  // ==========================================================================
  // Signal Data APIs
  // ==========================================================================

  async getSignalData(sessionId: string, params?: { start?: number; end?: number }): Promise<ApiResponse<SignalPacket[]>> {
    const query = this.buildQueryParams(params);
    return this.get<SignalPacket[]>(`/api/v1/sessions/${sessionId}/signals${query}`);
  }

  async addMarker(sessionId: string, label: string): Promise<ApiResponse<void>> {
    return this.post<void>(`/api/v1/sessions/${sessionId}/markers`, { label });
  }

  // ==========================================================================
  // HTTP Methods
  // ==========================================================================

  private async get<T>(path: string): Promise<ApiResponse<T>> {
    return this.request<T>('GET', path);
  }

  private async post<T>(path: string, data: unknown): Promise<ApiResponse<T>> {
    return this.request<T>('POST', path, data);
  }

  private async request<T>(method: string, path: string, data?: unknown): Promise<ApiResponse<T>> {
    const url = `${this.config.endpoint}${path}`;
    const response = await fetch(url, {
      method,
      headers: this.headers,
      body: data ? JSON.stringify(data) : undefined,
    });
    return response.json();
  }

  private buildQueryParams(params?: Record<string, unknown>): string {
    if (!params) return '';
    const searchParams = new URLSearchParams();
    Object.entries(params).forEach(([key, value]) => {
      if (value !== undefined) searchParams.append(key, String(value));
    });
    return searchParams.toString() ? `?${searchParams.toString()}` : '';
  }
}

// ============================================================================
// Constants
// ============================================================================

export const VERSION = '1.0.0';

export const DEFAULT_SDK_CONFIG: Partial<BCISDKConfig> = {
  timeout: 30000,
  retries: 3,
  debug: false,
};

export const ELECTRODE_SYSTEMS = {
  STANDARD_10_20: '10-20',
  STANDARD_10_10: '10-10',
  STANDARD_10_5: '10-5',
} as const;
