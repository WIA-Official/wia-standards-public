/**
 * WIA Exoskeleton Standard API
 * TypeScript Implementation
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * Standard API for rehabilitation exoskeletons including
 * control interfaces and intent detection.
 *
 * @packageDocumentation
 * @module wia-exoskeleton
 * @license MIT
 * @author WIA / SmileStory Inc.
 * @version 1.0.0
 *
 * @example
 * ```typescript
 * import {
 *   ExoskeletonSDK,
 *   createExoController,
 *   createIntentDetector,
 *   JointType,
 *   ControlMode,
 * } from '@wia/exoskeleton';
 *
 * const sdk = new ExoskeletonSDK({ apiKey: 'xxx', endpoint: 'https://api.wia.org' });
 * const device = await sdk.getDevice('exo-1');
 * ```
 */

// Controller exports
export {
  JointType,
  Side,
  ControlMode,
  AssistanceMode,
  CommandPriority,
  ResponseStatus,
  AdaptiveAlgorithm,
  TrajectoryType,
  PositionControlParams,
  VelocityControlParams,
  TorqueControlParams,
  ImpedanceParams,
  AdmittanceParams,
  ZeroTorqueParams,
  ControlParams,
  ControlCommand,
  ControlResponse,
  ControlError,
  AdaptiveAssistanceConfig,
  AssistanceState,
  JointLimits,
  BilateralJointLimits,
  RangeLimit,
  TrajectoryConfig,
  TrajectoryPoint,
  TrajectoryWaypoint,
  ControllerState,
  JointAssistanceLevels,
  IExoController,
  IAssistanceController,
  ExoController,
  createExoController,
  generateMinimumJerkTrajectory,
  generateCycloidTrajectory,
} from './controller';

// Intent exports
export {
  UserIntent,
  IntentSource,
  IntentState,
  ConfirmationMode,
  FusionMethod,
  Vector2D,
  Vector3D,
  EMGChannel,
  EMGFeatures,
  EMGData,
  EMGConfig,
  GRFData,
  GRFFeatures,
  GRFConfig,
  IMUData,
  IMUFeatures,
  IMUConfig,
  ButtonData,
  ButtonConfig,
  SourceResult,
  FeatureVector,
  IntentDetection,
  IntentDetectorConfig,
  IntentThresholds,
  CalibrationData,
  CalibrationState,
  DEFAULT_INTENT_CONFIG,
  DEFAULT_THRESHOLDS,
  EMGIntentDetector,
  GRFIntentDetector,
  IMUIntentDetector,
  ButtonIntentDetector,
  IIntentDetector,
  IntentDetector,
  createIntentDetector,
} from './intent';

import {
  ExoskeletonDevice,
  UserProfile,
  TherapySession,
  GaitMetrics,
  SafetyEvent,
  ApiResponse,
  PaginatedResponse,
} from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface ExoskeletonSDKConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  retries?: number;
  debug?: boolean;
}

// ============================================================================
// SDK Client
// ============================================================================

export class ExoskeletonSDK {
  private config: Required<ExoskeletonSDKConfig>;
  private headers: Record<string, string>;

  constructor(config: ExoskeletonSDKConfig) {
    this.config = {
      timeout: 30000,
      retries: 3,
      debug: false,
      ...config,
    };

    this.headers = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'X-WIA-Standard': 'EXOSKELETON',
      'X-WIA-Version': '1.0.0',
    };
  }

  // ==========================================================================
  // Device Management APIs
  // ==========================================================================

  async listDevices(): Promise<ApiResponse<ExoskeletonDevice[]>> {
    return this.get<ExoskeletonDevice[]>('/api/v1/devices');
  }

  async getDevice(deviceId: string): Promise<ApiResponse<ExoskeletonDevice>> {
    return this.get<ExoskeletonDevice>(`/api/v1/devices/${deviceId}`);
  }

  async registerDevice(device: Omit<ExoskeletonDevice, 'deviceId'>): Promise<ApiResponse<ExoskeletonDevice>> {
    return this.post<ExoskeletonDevice>('/api/v1/devices', device);
  }

  async calibrateDevice(deviceId: string): Promise<ApiResponse<void>> {
    return this.post<void>(`/api/v1/devices/${deviceId}/calibrate`, {});
  }

  // ==========================================================================
  // User Profile APIs
  // ==========================================================================

  async getProfile(userId: string): Promise<ApiResponse<UserProfile>> {
    return this.get<UserProfile>(`/api/v1/users/${userId}/profile`);
  }

  async createProfile(profile: Omit<UserProfile, 'profileId'>): Promise<ApiResponse<UserProfile>> {
    return this.post<UserProfile>('/api/v1/profiles', profile);
  }

  async updateProfile(userId: string, updates: Partial<UserProfile>): Promise<ApiResponse<UserProfile>> {
    return this.put<UserProfile>(`/api/v1/users/${userId}/profile`, updates);
  }

  // ==========================================================================
  // Therapy Session APIs
  // ==========================================================================

  async startSession(userId: string, deviceId: string): Promise<ApiResponse<TherapySession>> {
    return this.post<TherapySession>('/api/v1/sessions', { userId, deviceId });
  }

  async getSession(sessionId: string): Promise<ApiResponse<TherapySession>> {
    return this.get<TherapySession>(`/api/v1/sessions/${sessionId}`);
  }

  async endSession(sessionId: string): Promise<ApiResponse<TherapySession>> {
    return this.post<TherapySession>(`/api/v1/sessions/${sessionId}/end`, {});
  }

  async getSessionMetrics(sessionId: string): Promise<ApiResponse<GaitMetrics>> {
    return this.get<GaitMetrics>(`/api/v1/sessions/${sessionId}/metrics`);
  }

  async listUserSessions(userId: string): Promise<ApiResponse<TherapySession[]>> {
    return this.get<TherapySession[]>(`/api/v1/users/${userId}/sessions`);
  }

  // ==========================================================================
  // Safety APIs
  // ==========================================================================

  async getSafetyEvents(deviceId: string): Promise<ApiResponse<SafetyEvent[]>> {
    return this.get<SafetyEvent[]>(`/api/v1/devices/${deviceId}/safety-events`);
  }

  async reportSafetyEvent(event: Omit<SafetyEvent, 'eventId'>): Promise<ApiResponse<SafetyEvent>> {
    return this.post<SafetyEvent>('/api/v1/safety-events', event);
  }

  async emergencyStop(deviceId: string): Promise<ApiResponse<void>> {
    return this.post<void>(`/api/v1/devices/${deviceId}/emergency-stop`, {});
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
    const response = await fetch(url, {
      method,
      headers: this.headers,
      body: data ? JSON.stringify(data) : undefined,
    });
    return response.json();
  }
}

// ============================================================================
// Constants
// ============================================================================

export const VERSION = '1.0.0';

export const JOINT_LIMITS = {
  HIP: { min: -30, max: 120 },
  KNEE: { min: 0, max: 150 },
  ANKLE: { min: -20, max: 45 },
} as const;

export const ASSISTANCE_PRESETS = {
  MINIMAL: { hip: 10, knee: 10, ankle: 10 },
  MODERATE: { hip: 30, knee: 30, ankle: 30 },
  MAXIMUM: { hip: 50, knee: 50, ankle: 50 },
} as const;

export default ExoskeletonSDK;
