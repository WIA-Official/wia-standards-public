/**
 * WIA Eye Gaze Interoperability Protocol SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * Standard interface for eye tracking devices and gaze-aware applications.
 *
 * @packageDocumentation
 * @module wia-eye-gaze
 * @license MIT
 * @author WIA / SmileStory Inc.
 * @version 1.0.0
 *
 * @example Basic Usage
 * ```typescript
 * import {
 *   EyeGazeSDK,
 *   createEyeTracker,
 *   createMockAdapter,
 *   createDwellController,
 * } from '@wia/eye-gaze';
 *
 * const sdk = new EyeGazeSDK({ apiKey: 'xxx', endpoint: 'https://api.wia.org' });
 * const tracker = createEyeTracker(createMockAdapter({ samplingRate: 60 }));
 *
 * await tracker.connect();
 * await tracker.startCalibration();
 *
 * tracker.subscribe(gaze => {
 *   console.log(`Gaze: (${gaze.x.toFixed(2)}, ${gaze.y.toFixed(2)})`);
 * });
 *
 * tracker.startTracking();
 * ```
 */

// ============================================
// Types
// ============================================
export * from './types';

// ============================================
// Tracker
// ============================================
export {
  IEyeTracker,
  IEyeTrackerAdapter,
  WiaEyeTracker,
  createEyeTracker,
  MockAdapter,
  createMockAdapter,
  TobiiAdapter,
  createTobiiAdapter,
  GazepointAdapter,
  createGazepointAdapter,
  PupilLabsAdapter,
  createPupilLabsAdapter,
} from './tracker';

export type {
  MockAdapterOptions,
  TobiiAdapterOptions,
  GazepointAdapterOptions,
  PupilLabsAdapterOptions,
} from './tracker';

// ============================================
// Dwell
// ============================================
export {
  DwellController,
  createDwellController,
} from './dwell';

export type {
  DwellControllerOptions,
  DwellEventHandler,
} from './dwell';

// ============================================
// App
// ============================================
export {
  GazeAwareApp,
  createGazeAwareApp,
} from './app';

export type {
  GazeAwareAppOptions,
  ControlRequestHandler,
  MessageHandler,
} from './app';

import {
  GazePoint,
  CalibrationResult,
  TrackerCapabilities,
  UserSession,
  AreaOfInterest,
  FixationData,
  SaccadeData,
  ApiResponse,
  PaginatedResponse,
} from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface EyeGazeSDKConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  retries?: number;
  debug?: boolean;
}

// ============================================================================
// SDK Client
// ============================================================================

export class EyeGazeSDK {
  private config: Required<EyeGazeSDKConfig>;
  private headers: Record<string, string>;

  constructor(config: EyeGazeSDKConfig) {
    this.config = {
      timeout: 30000,
      retries: 3,
      debug: false,
      ...config,
    };

    this.headers = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'X-WIA-Standard': 'EYE-GAZE',
      'X-WIA-Version': '1.0.0',
    };
  }

  // ==========================================================================
  // Device Management APIs
  // ==========================================================================

  async listDevices(): Promise<ApiResponse<TrackerCapabilities[]>> {
    return this.get<TrackerCapabilities[]>('/api/v1/devices');
  }

  async getDevice(deviceId: string): Promise<ApiResponse<TrackerCapabilities>> {
    return this.get<TrackerCapabilities>(`/api/v1/devices/${deviceId}`);
  }

  async connectDevice(deviceId: string): Promise<ApiResponse<void>> {
    return this.post<void>(`/api/v1/devices/${deviceId}/connect`, {});
  }

  async disconnectDevice(deviceId: string): Promise<ApiResponse<void>> {
    return this.post<void>(`/api/v1/devices/${deviceId}/disconnect`, {});
  }

  // ==========================================================================
  // Calibration APIs
  // ==========================================================================

  async startCalibration(deviceId: string, points: number): Promise<ApiResponse<string>> {
    return this.post<string>(`/api/v1/devices/${deviceId}/calibration/start`, { points });
  }

  async submitCalibrationPoint(deviceId: string, point: GazePoint): Promise<ApiResponse<void>> {
    return this.post<void>(`/api/v1/devices/${deviceId}/calibration/point`, point);
  }

  async finishCalibration(deviceId: string): Promise<ApiResponse<CalibrationResult>> {
    return this.post<CalibrationResult>(`/api/v1/devices/${deviceId}/calibration/finish`, {});
  }

  async getCalibrationResult(deviceId: string): Promise<ApiResponse<CalibrationResult>> {
    return this.get<CalibrationResult>(`/api/v1/devices/${deviceId}/calibration`);
  }

  // ==========================================================================
  // Session APIs
  // ==========================================================================

  async startSession(userId: string, deviceId: string): Promise<ApiResponse<UserSession>> {
    return this.post<UserSession>('/api/v1/sessions', { userId, deviceId });
  }

  async getSession(sessionId: string): Promise<ApiResponse<UserSession>> {
    return this.get<UserSession>(`/api/v1/sessions/${sessionId}`);
  }

  async endSession(sessionId: string): Promise<ApiResponse<UserSession>> {
    return this.post<UserSession>(`/api/v1/sessions/${sessionId}/end`, {});
  }

  async getSessionGazeData(sessionId: string): Promise<ApiResponse<GazePoint[]>> {
    return this.get<GazePoint[]>(`/api/v1/sessions/${sessionId}/gaze-data`);
  }

  // ==========================================================================
  // Area of Interest APIs
  // ==========================================================================

  async createAOI(aoi: Omit<AreaOfInterest, 'aoiId'>): Promise<ApiResponse<AreaOfInterest>> {
    return this.post<AreaOfInterest>('/api/v1/aoi', aoi);
  }

  async listAOIs(sessionId: string): Promise<ApiResponse<AreaOfInterest[]>> {
    return this.get<AreaOfInterest[]>(`/api/v1/sessions/${sessionId}/aoi`);
  }

  async getAOIMetrics(aoiId: string): Promise<ApiResponse<FixationData[]>> {
    return this.get<FixationData[]>(`/api/v1/aoi/${aoiId}/metrics`);
  }

  // ==========================================================================
  // Analytics APIs
  // ==========================================================================

  async getFixations(sessionId: string): Promise<ApiResponse<FixationData[]>> {
    return this.get<FixationData[]>(`/api/v1/sessions/${sessionId}/fixations`);
  }

  async getSaccades(sessionId: string): Promise<ApiResponse<SaccadeData[]>> {
    return this.get<SaccadeData[]>(`/api/v1/sessions/${sessionId}/saccades`);
  }

  async getHeatmap(sessionId: string): Promise<ApiResponse<number[][]>> {
    return this.get<number[][]>(`/api/v1/sessions/${sessionId}/heatmap`);
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
}

// ============================================
// Constants
// ============================================

export const VERSION = '1.0.0-alpha';
export const PROTOCOL_VERSION = '1.0.0';

export const SAMPLING_RATES = {
  LOW: 30,
  MEDIUM: 60,
  HIGH: 120,
  RESEARCH: 250,
} as const;

export const CALIBRATION_POINTS = {
  QUICK: 5,
  STANDARD: 9,
  PRECISE: 16,
} as const;

export default EyeGazeSDK;
