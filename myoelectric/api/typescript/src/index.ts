/**
 * WIA-MED-024: Myoelectric Control Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

import EventEmitter from 'eventemitter3';
import {
  WIAConfig,
  APIResponse,
  PaginatedResponse,
  MyoelectricDevice,
  EMGSignal,
  ProcessedSignal,
  ControlSession,
  GestureEvent,
  ClassifierModel,
  GestureType,
  DeviceStatus,
  CalibrationStatus,
} from './types';

export * from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface WIAMyoelectricConfig extends WIAConfig {
  deviceId?: string;
}

// ============================================================================
// Main SDK Client
// ============================================================================

export class WIAMyoelectricClient {
  private config: Required<WIAMyoelectricConfig>;
  private eventEmitter = new EventEmitter();

  constructor(config: WIAMyoelectricConfig) {
    this.config = {
      endpoint: 'https://api.wia-standards.org/v1/myoelectric',
      timeout: 30000,
      debug: false,
      deviceId: '',
      ...config,
    };

    if (!this.config.apiKey) {
      throw new Error('API key is required');
    }
  }

  // ==========================================================================
  // Device Operations
  // ==========================================================================

  async getDevice(deviceId: string): Promise<APIResponse<MyoelectricDevice>> {
    return this.makeRequest('GET', `/devices/${deviceId}`);
  }

  async listDevices(patientId: string): Promise<PaginatedResponse<MyoelectricDevice>> {
    return this.makeRequest('GET', `/patients/${patientId}/devices`);
  }

  async registerDevice(device: Omit<MyoelectricDevice, 'deviceId' | 'lastSync'>): Promise<APIResponse<MyoelectricDevice>> {
    return this.makeRequest('POST', '/devices', device);
  }

  async updateDeviceStatus(deviceId: string, status: DeviceStatus): Promise<APIResponse<MyoelectricDevice>> {
    return this.makeRequest('PATCH', `/devices/${deviceId}/status`, { status });
  }

  async syncDevice(deviceId: string): Promise<APIResponse<{ lastSync: string; dataPoints: number }>> {
    return this.makeRequest('POST', `/devices/${deviceId}/sync`);
  }

  // ==========================================================================
  // Signal Operations
  // ==========================================================================

  async submitSignal(signal: Omit<EMGSignal, 'signalId'>): Promise<APIResponse<ProcessedSignal>> {
    return this.makeRequest('POST', '/signals', signal);
  }

  async getSignal(signalId: string): Promise<APIResponse<EMGSignal>> {
    return this.makeRequest('GET', `/signals/${signalId}`);
  }

  async getSignalHistory(deviceId: string, filters?: {
    startTime?: string;
    endTime?: string;
    limit?: number;
  }): Promise<PaginatedResponse<EMGSignal>> {
    const params = new URLSearchParams({ deviceId, ...filters as Record<string, string> });
    return this.makeRequest('GET', `/signals?${params}`);
  }

  async classifySignal(signalId: string): Promise<APIResponse<{
    gesture: GestureType;
    confidence: number;
    alternatives: Array<{ gesture: GestureType; confidence: number }>;
  }>> {
    return this.makeRequest('POST', `/signals/${signalId}/classify`);
  }

  // ==========================================================================
  // Calibration Operations
  // ==========================================================================

  async startCalibration(deviceId: string): Promise<APIResponse<{
    calibrationId: string;
    gesturesToCalibrate: GestureType[];
  }>> {
    return this.makeRequest('POST', `/devices/${deviceId}/calibration/start`);
  }

  async submitCalibrationSample(deviceId: string, sample: {
    gesture: GestureType;
    signalData: number[];
  }): Promise<APIResponse<{ samplesCollected: number; samplesNeeded: number }>> {
    return this.makeRequest('POST', `/devices/${deviceId}/calibration/sample`, sample);
  }

  async completeCalibration(deviceId: string): Promise<APIResponse<{
    status: CalibrationStatus;
    accuracy: number;
    gestureAccuracies: Record<GestureType, number>;
  }>> {
    return this.makeRequest('POST', `/devices/${deviceId}/calibration/complete`);
  }

  async getCalibrationStatus(deviceId: string): Promise<APIResponse<{
    status: CalibrationStatus;
    calibratedAt?: string;
    expiresAt?: string;
  }>> {
    return this.makeRequest('GET', `/devices/${deviceId}/calibration/status`);
  }

  // ==========================================================================
  // Session Operations
  // ==========================================================================

  async startSession(deviceId: string): Promise<APIResponse<ControlSession>> {
    return this.makeRequest('POST', `/devices/${deviceId}/sessions/start`);
  }

  async endSession(sessionId: string): Promise<APIResponse<ControlSession>> {
    return this.makeRequest('POST', `/sessions/${sessionId}/end`);
  }

  async getSession(sessionId: string): Promise<APIResponse<ControlSession>> {
    return this.makeRequest('GET', `/sessions/${sessionId}`);
  }

  async listSessions(deviceId: string): Promise<PaginatedResponse<ControlSession>> {
    return this.makeRequest('GET', `/devices/${deviceId}/sessions`);
  }

  async recordGestureEvent(sessionId: string, event: Omit<GestureEvent, 'eventId'>): Promise<APIResponse<GestureEvent>> {
    return this.makeRequest('POST', `/sessions/${sessionId}/events`, event);
  }

  // ==========================================================================
  // Classifier Operations
  // ==========================================================================

  async getClassifierModel(modelId: string): Promise<APIResponse<ClassifierModel>> {
    return this.makeRequest('GET', `/models/${modelId}`);
  }

  async listModels(patientId: string): Promise<PaginatedResponse<ClassifierModel>> {
    return this.makeRequest('GET', `/patients/${patientId}/models`);
  }

  async trainModel(patientId: string, gestures: GestureType[]): Promise<APIResponse<ClassifierModel>> {
    return this.makeRequest('POST', `/patients/${patientId}/models/train`, { gestures });
  }

  // ==========================================================================
  // Event Handling
  // ==========================================================================

  on(event: 'gestureDetected' | 'calibrationComplete' | 'deviceConnected' | 'deviceDisconnected' | 'error', callback: (...args: unknown[]) => void): void {
    this.eventEmitter.on(event, callback);
  }

  // ==========================================================================
  // Private Methods
  // ==========================================================================

  private async makeRequest<T>(method: string, path: string, body?: unknown): Promise<T> {
    const url = `${this.config.endpoint}${path}`;

    if (this.config.debug) {
      console.log(`[WIA Myoelectric] ${method} ${url}`, body);
    }

    try {
      const response = await fetch(url, {
        method,
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${this.config.apiKey}`,
          'X-WIA-Standard': 'MED-024',
          'X-WIA-Version': '1.0.0',
          ...(this.config.deviceId && { 'X-Device-ID': this.config.deviceId }),
        },
        body: body ? JSON.stringify(body) : undefined,
      });

      return await response.json();
    } catch (error: unknown) {
      const message = error instanceof Error ? error.message : 'Unknown error';
      this.eventEmitter.emit('error', { code: 'REQUEST_FAILED', message });
      throw error;
    }
  }
}

export function createClient(config: WIAMyoelectricConfig): WIAMyoelectricClient {
  return new WIAMyoelectricClient(config);
}

export default WIAMyoelectricClient;
