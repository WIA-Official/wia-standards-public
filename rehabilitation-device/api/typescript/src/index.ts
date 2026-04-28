/**
 * WIA-MED-028: Rehabilitation Device Standard - TypeScript SDK
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
  RehabDevice,
  ExerciseSession,
  SessionMetrics,
  RehabProgram,
  RehabGoal,
  DeviceStatus,
  ExerciseType,
  RecoveryStage,
} from './types';

export * from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface WIARehabDeviceConfig extends WIAConfig {
  facilityId?: string;
}

// ============================================================================
// Main SDK Client
// ============================================================================

export class WIARehabDeviceClient {
  private config: Required<WIARehabDeviceConfig>;
  private eventEmitter = new EventEmitter();

  constructor(config: WIARehabDeviceConfig) {
    this.config = {
      endpoint: 'https://api.wia-standards.org/v1/rehab-device',
      timeout: 30000,
      debug: false,
      facilityId: '',
      ...config,
    };

    if (!this.config.apiKey) {
      throw new Error('API key is required');
    }
  }

  // ==========================================================================
  // Device Operations
  // ==========================================================================

  async getDevice(deviceId: string): Promise<APIResponse<RehabDevice>> {
    return this.makeRequest('GET', `/devices/${deviceId}`);
  }

  async listDevices(filters?: {
    status?: DeviceStatus;
    type?: string;
  }): Promise<PaginatedResponse<RehabDevice>> {
    const params = new URLSearchParams(filters as Record<string, string>);
    return this.makeRequest('GET', `/devices?${params}`);
  }

  async registerDevice(device: Omit<RehabDevice, 'deviceId'>): Promise<APIResponse<RehabDevice>> {
    return this.makeRequest('POST', '/devices', device);
  }

  async updateDeviceStatus(deviceId: string, status: DeviceStatus): Promise<APIResponse<RehabDevice>> {
    return this.makeRequest('PATCH', `/devices/${deviceId}/status`, { status });
  }

  async calibrateDevice(deviceId: string): Promise<APIResponse<{
    calibrationId: string;
    status: string;
    calibratedAt: string;
  }>> {
    return this.makeRequest('POST', `/devices/${deviceId}/calibrate`);
  }

  // ==========================================================================
  // Session Operations
  // ==========================================================================

  async startSession(session: Omit<ExerciseSession, 'sessionId' | 'startTime' | 'metrics'>): Promise<APIResponse<ExerciseSession>> {
    return this.makeRequest('POST', '/sessions', session);
  }

  async getSession(sessionId: string): Promise<APIResponse<ExerciseSession>> {
    return this.makeRequest('GET', `/sessions/${sessionId}`);
  }

  async endSession(sessionId: string, notes?: string): Promise<APIResponse<ExerciseSession>> {
    return this.makeRequest('POST', `/sessions/${sessionId}/end`, { notes });
  }

  async updateProgress(sessionId: string, metrics: Partial<SessionMetrics>): Promise<APIResponse<ExerciseSession>> {
    return this.makeRequest('PATCH', `/sessions/${sessionId}/metrics`, metrics);
  }

  async listSessions(patientId: string, filters?: {
    deviceId?: string;
    type?: ExerciseType;
    startDate?: string;
    endDate?: string;
  }): Promise<PaginatedResponse<ExerciseSession>> {
    const params = new URLSearchParams({ patientId, ...filters as Record<string, string> });
    return this.makeRequest('GET', `/sessions?${params}`);
  }

  async getSessionMetrics(sessionId: string): Promise<APIResponse<SessionMetrics>> {
    return this.makeRequest('GET', `/sessions/${sessionId}/metrics`);
  }

  // ==========================================================================
  // Program Operations
  // ==========================================================================

  async getProgram(patientId: string): Promise<APIResponse<RehabProgram>> {
    return this.makeRequest('GET', `/patients/${patientId}/program`);
  }

  async createProgram(program: Omit<RehabProgram, 'programId' | 'progress'>): Promise<APIResponse<RehabProgram>> {
    return this.makeRequest('POST', '/programs', program);
  }

  async updateProgram(programId: string, updates: Partial<RehabProgram>): Promise<APIResponse<RehabProgram>> {
    return this.makeRequest('PATCH', `/programs/${programId}`, updates);
  }

  async getProgramProgress(programId: string): Promise<APIResponse<{
    overallProgress: number;
    goalsAchieved: number;
    goalsTotal: number;
    sessionsCompleted: number;
  }>> {
    return this.makeRequest('GET', `/programs/${programId}/progress`);
  }

  // ==========================================================================
  // Goal Operations
  // ==========================================================================

  async addGoal(programId: string, goal: Omit<RehabGoal, 'goalId' | 'achieved'>): Promise<APIResponse<RehabGoal>> {
    return this.makeRequest('POST', `/programs/${programId}/goals`, goal);
  }

  async updateGoal(goalId: string, updates: Partial<RehabGoal>): Promise<APIResponse<RehabGoal>> {
    return this.makeRequest('PATCH', `/goals/${goalId}`, updates);
  }

  async markGoalAchieved(goalId: string): Promise<APIResponse<RehabGoal>> {
    return this.makeRequest('POST', `/goals/${goalId}/achieve`);
  }

  // ==========================================================================
  // Analytics
  // ==========================================================================

  async getPatientAnalytics(patientId: string): Promise<APIResponse<{
    totalSessions: number;
    totalMinutes: number;
    averagePerformance: number;
    improvementTrend: 'improving' | 'stable' | 'declining';
    rangeOfMotionProgress: Record<string, number>;
  }>> {
    return this.makeRequest('GET', `/patients/${patientId}/analytics`);
  }

  // ==========================================================================
  // Event Handling
  // ==========================================================================

  on(event: 'sessionStarted' | 'sessionCompleted' | 'goalAchieved' | 'deviceError' | 'error', callback: (...args: unknown[]) => void): void {
    this.eventEmitter.on(event, callback);
  }

  // ==========================================================================
  // Private Methods
  // ==========================================================================

  private async makeRequest<T>(method: string, path: string, body?: unknown): Promise<T> {
    const url = `${this.config.endpoint}${path}`;

    if (this.config.debug) {
      console.log(`[WIA Rehab Device] ${method} ${url}`, body);
    }

    try {
      const response = await fetch(url, {
        method,
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${this.config.apiKey}`,
          'X-WIA-Standard': 'MED-028',
          'X-WIA-Version': '1.0.0',
          ...(this.config.facilityId && { 'X-Facility-ID': this.config.facilityId }),
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

export function createClient(config: WIARehabDeviceConfig): WIARehabDeviceClient {
  return new WIARehabDeviceClient(config);
}

export default WIARehabDeviceClient;
