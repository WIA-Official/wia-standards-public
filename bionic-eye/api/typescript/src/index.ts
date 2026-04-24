/**
 * WIA Bionic Eye Standard API
 * TypeScript Implementation
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @packageDocumentation
 * @module wia-bionic-eye
 * @license MIT
 * @author WIA / SmileStory Inc.
 * @version 1.0.0
 *
 * @example
 * ```typescript
 * import { BionicEyeSDK, StimulationController } from 'wia-bionic-eye';
 *
 * const sdk = new BionicEyeSDK({ apiKey: 'xxx', endpoint: 'https://api.wia.org' });
 * const device = await sdk.getDevice('device-1');
 * ```
 */

// Export all types
export * from './types';

import {
  ImplantDevice,
  ElectrodeArray,
  StimulationPattern,
  PhospheneMap,
  CalibrationSession,
  VisualProcessingConfig,
  SafetyLimits,
  ApiResponse,
  PaginatedResponse,
} from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface BionicEyeSDKConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  retries?: number;
  debug?: boolean;
}

// ============================================================================
// SDK Client
// ============================================================================

export class BionicEyeSDK {
  private config: Required<BionicEyeSDKConfig>;
  private headers: Record<string, string>;

  constructor(config: BionicEyeSDKConfig) {
    this.config = {
      timeout: 30000,
      retries: 3,
      debug: false,
      ...config,
    };

    this.headers = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'X-WIA-Standard': 'BIONIC-EYE',
      'X-WIA-Version': '1.0.0',
    };
  }

  // ==========================================================================
  // Device Management APIs
  // ==========================================================================

  async listDevices(): Promise<ApiResponse<ImplantDevice[]>> {
    return this.get<ImplantDevice[]>('/api/v1/devices');
  }

  async getDevice(deviceId: string): Promise<ApiResponse<ImplantDevice>> {
    return this.get<ImplantDevice>(`/api/v1/devices/${deviceId}`);
  }

  async registerDevice(device: Omit<ImplantDevice, 'deviceId'>): Promise<ApiResponse<ImplantDevice>> {
    return this.post<ImplantDevice>('/api/v1/devices', device);
  }

  async updateDevice(deviceId: string, updates: Partial<ImplantDevice>): Promise<ApiResponse<ImplantDevice>> {
    return this.put<ImplantDevice>(`/api/v1/devices/${deviceId}`, updates);
  }

  // ==========================================================================
  // Electrode Array APIs
  // ==========================================================================

  async getElectrodeArray(deviceId: string): Promise<ApiResponse<ElectrodeArray>> {
    return this.get<ElectrodeArray>(`/api/v1/devices/${deviceId}/electrodes`);
  }

  async calibrateElectrode(deviceId: string, electrodeId: string): Promise<ApiResponse<void>> {
    return this.post<void>(`/api/v1/devices/${deviceId}/electrodes/${electrodeId}/calibrate`, {});
  }

  async setElectrodeThreshold(deviceId: string, electrodeId: string, threshold: number): Promise<ApiResponse<void>> {
    return this.put<void>(`/api/v1/devices/${deviceId}/electrodes/${electrodeId}/threshold`, { threshold });
  }

  // ==========================================================================
  // Stimulation APIs
  // ==========================================================================

  async createStimulationPattern(pattern: Omit<StimulationPattern, 'patternId'>): Promise<ApiResponse<StimulationPattern>> {
    return this.post<StimulationPattern>('/api/v1/stimulation/patterns', pattern);
  }

  async executeStimulation(deviceId: string, patternId: string): Promise<ApiResponse<void>> {
    return this.post<void>(`/api/v1/devices/${deviceId}/stimulate`, { patternId });
  }

  async stopStimulation(deviceId: string): Promise<ApiResponse<void>> {
    return this.post<void>(`/api/v1/devices/${deviceId}/stop`, {});
  }

  // ==========================================================================
  // Phosphene Mapping APIs
  // ==========================================================================

  async getPhospheneMap(deviceId: string): Promise<ApiResponse<PhospheneMap>> {
    return this.get<PhospheneMap>(`/api/v1/devices/${deviceId}/phosphene-map`);
  }

  async updatePhospheneMap(deviceId: string, map: Partial<PhospheneMap>): Promise<ApiResponse<PhospheneMap>> {
    return this.put<PhospheneMap>(`/api/v1/devices/${deviceId}/phosphene-map`, map);
  }

  // ==========================================================================
  // Calibration APIs
  // ==========================================================================

  async startCalibration(deviceId: string): Promise<ApiResponse<CalibrationSession>> {
    return this.post<CalibrationSession>(`/api/v1/devices/${deviceId}/calibration/start`, {});
  }

  async endCalibration(deviceId: string, sessionId: string): Promise<ApiResponse<CalibrationSession>> {
    return this.post<CalibrationSession>(`/api/v1/devices/${deviceId}/calibration/${sessionId}/end`, {});
  }

  async getCalibrationHistory(deviceId: string): Promise<ApiResponse<CalibrationSession[]>> {
    return this.get<CalibrationSession[]>(`/api/v1/devices/${deviceId}/calibration/history`);
  }

  // ==========================================================================
  // Safety APIs
  // ==========================================================================

  async getSafetyLimits(deviceId: string): Promise<ApiResponse<SafetyLimits>> {
    return this.get<SafetyLimits>(`/api/v1/devices/${deviceId}/safety`);
  }

  async updateSafetyLimits(deviceId: string, limits: Partial<SafetyLimits>): Promise<ApiResponse<SafetyLimits>> {
    return this.put<SafetyLimits>(`/api/v1/devices/${deviceId}/safety`, limits);
  }

  async emergencyShutdown(deviceId: string): Promise<ApiResponse<void>> {
    return this.post<void>(`/api/v1/devices/${deviceId}/emergency-shutdown`, {});
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

  private async put<T>(path: string, data: unknown): Promise<ApiResponse<T>> {
    return this.request<T>('PUT', path, data);
  }

  private async request<T>(method: string, path: string, data?: unknown): Promise<ApiResponse<T>> {
    const url = `${this.config.endpoint}${path}`;
    if (this.config.debug) {
      console.log(`[WIA Bionic Eye] ${method} ${url}`);
    }
    const response = await fetch(url, {
      method,
      headers: this.headers,
      body: data ? JSON.stringify(data) : undefined,
    });
    return response.json();
  }
}

// ============================================================================
// Stimulation Controller
// ============================================================================

export enum MappingStrategy {
  SCOREBOARD = 'scoreboard',
  GAUSSIAN = 'gaussian',
  EDGE_ENHANCED = 'edge_enhanced',
  ADAPTIVE = 'adaptive',
}

export class StimulationController {
  private electrodeArray: ElectrodeArray;
  private strategy: MappingStrategy = MappingStrategy.SCOREBOARD;

  constructor(electrodeArray: ElectrodeArray) {
    this.electrodeArray = electrodeArray;
  }

  setStrategy(strategy: MappingStrategy): void {
    this.strategy = strategy;
  }

  stimulateFrame(visualData: number[][]): StimulationPattern {
    return {
      patternId: `pattern-${Date.now()}`,
      electrodeActivations: [],
      duration: 100,
      frequency: 50,
      created: new Date().toISOString(),
    };
  }
}

// ============================================================================
// Constants
// ============================================================================

export const VERSION = '1.0.0';

export const ELECTRODE_CONFIGURATIONS = {
  ARGUS_II: { rows: 6, cols: 10, spacing: 0.5 },
  PRIMA: { rows: 40, cols: 40, spacing: 0.1 },
  ALPHA_AMS: { rows: 40, cols: 40, spacing: 0.07 },
} as const;

export const STIMULATION_DEFAULTS = {
  maxCurrent: 1000,
  maxFrequency: 250,
  maxDuration: 1000,
  phaseDuration: 500,
} as const;

export default BionicEyeSDK;
