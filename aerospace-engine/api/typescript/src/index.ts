/**
 * WIA-SPACE-015: Aerospace Engine Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @packageDocumentation
 * @module wia-aerospace-engine
 * @license MIT
 * @author WIA / SmileStory Inc.
 * @version 1.0.0
 */

import {
  Engine,
  EngineTelemetry,
  EngineType,
  APIResponse,
  MaintenanceRecord,
  PerformanceMetrics,
  FuelSystem,
  Certification,
  TestResult,
  PaginatedResponse,
} from './types';

export * from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface AerospaceEngineSDKConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  retries?: number;
  debug?: boolean;
}

// ============================================================================
// SDK Client
// ============================================================================

export class AerospaceEngineSDK {
  private config: Required<AerospaceEngineSDKConfig>;
  private headers: Record<string, string>;

  constructor(config: AerospaceEngineSDKConfig) {
    this.config = {
      timeout: 30000,
      retries: 3,
      debug: false,
      ...config,
    };

    this.headers = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'X-WIA-Standard': 'SPACE-015',
      'X-WIA-Version': '1.0.0',
    };
  }

  // ==========================================================================
  // Engine APIs
  // ==========================================================================

  async listEngines(): Promise<APIResponse<Engine[]>> {
    return this.get<Engine[]>('/api/v1/engines');
  }

  async getEngine(id: string): Promise<APIResponse<Engine>> {
    return this.get<Engine>(`/api/v1/engines/${id}`);
  }

  async registerEngine(engine: Omit<Engine, 'engineId'>): Promise<APIResponse<Engine>> {
    return this.post<Engine>('/api/v1/engines', engine);
  }

  async updateEngine(id: string, updates: Partial<Engine>): Promise<APIResponse<Engine>> {
    return this.put<Engine>(`/api/v1/engines/${id}`, updates);
  }

  // ==========================================================================
  // Telemetry APIs
  // ==========================================================================

  async getTelemetry(engineId: string): Promise<APIResponse<EngineTelemetry>> {
    return this.get<EngineTelemetry>(`/api/v1/engines/${engineId}/telemetry`);
  }

  async getTelemetryHistory(engineId: string, period?: string): Promise<APIResponse<EngineTelemetry[]>> {
    const query = period ? `?period=${period}` : '';
    return this.get<EngineTelemetry[]>(`/api/v1/engines/${engineId}/telemetry/history${query}`);
  }

  async submitTelemetry(engineId: string, telemetry: Omit<EngineTelemetry, 'telemetryId'>): Promise<APIResponse<EngineTelemetry>> {
    return this.post<EngineTelemetry>(`/api/v1/engines/${engineId}/telemetry`, telemetry);
  }

  // ==========================================================================
  // Performance APIs
  // ==========================================================================

  async getPerformanceMetrics(engineId: string): Promise<APIResponse<PerformanceMetrics>> {
    return this.get<PerformanceMetrics>(`/api/v1/engines/${engineId}/performance`);
  }

  async analyzePerformance(engineId: string): Promise<APIResponse<{ recommendations: string[] }>> {
    return this.post<{ recommendations: string[] }>(`/api/v1/engines/${engineId}/performance/analyze`, {});
  }

  // ==========================================================================
  // Maintenance APIs
  // ==========================================================================

  async getMaintenanceRecords(engineId: string): Promise<APIResponse<MaintenanceRecord[]>> {
    return this.get<MaintenanceRecord[]>(`/api/v1/engines/${engineId}/maintenance`);
  }

  async addMaintenanceRecord(record: Omit<MaintenanceRecord, 'recordId'>): Promise<APIResponse<MaintenanceRecord>> {
    return this.post<MaintenanceRecord>('/api/v1/maintenance', record);
  }

  // ==========================================================================
  // Certification APIs
  // ==========================================================================

  async getCertifications(engineId: string): Promise<APIResponse<Certification[]>> {
    return this.get<Certification[]>(`/api/v1/engines/${engineId}/certifications`);
  }

  async verifyCertification(certId: string): Promise<APIResponse<{ valid: boolean }>> {
    return this.post<{ valid: boolean }>(`/api/v1/certifications/${certId}/verify`, {});
  }

  // ==========================================================================
  // Calculator Methods
  // ==========================================================================

  calculateSFC(type: EngineType, bypassRatio?: number): number {
    const sfcMap: Record<EngineType, number> = {
      [EngineType.TURBOFAN]: 0.52 - ((bypassRatio || 10) / 100),
      [EngineType.TURBOPROP]: 0.48,
      [EngineType.TURBOSHAFT]: 0.50,
      [EngineType.ROCKET_LIQUID]: 7.5,
      [EngineType.ROCKET_SOLID]: 8.0,
      [EngineType.ION]: 0.01,
    };
    return sfcMap[type];
  }

  calculateEfficiency(type: EngineType, bypassRatio?: number): number {
    if (type === EngineType.TURBOFAN) {
      return 15 + ((bypassRatio || 10) * 0.5);
    }
    const efficiencyMap: Record<EngineType, number> = {
      [EngineType.TURBOFAN]: 20,
      [EngineType.TURBOPROP]: 12,
      [EngineType.TURBOSHAFT]: 15,
      [EngineType.ROCKET_LIQUID]: 65,
      [EngineType.ROCKET_SOLID]: 60,
      [EngineType.ION]: 90,
    };
    return efficiencyMap[type];
  }

  // ==========================================================================
  // HTTP Methods
  // ==========================================================================

  private async get<T>(path: string): Promise<APIResponse<T>> {
    return this.request<T>('GET', path);
  }

  private async post<T>(path: string, data: unknown): Promise<APIResponse<T>> {
    return this.request<T>('POST', path, data);
  }

  private async put<T>(path: string, data: unknown): Promise<APIResponse<T>> {
    return this.request<T>('PUT', path, data);
  }

  private async request<T>(method: string, path: string, data?: unknown): Promise<APIResponse<T>> {
    const url = `${this.config.endpoint}${path}`;
    if (this.config.debug) {
      console.log(`[WIA Aerospace Engine] ${method} ${url}`);
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
// Constants
// ============================================================================

export const VERSION = '1.0.0';

export const THRUST_CLASSES = {
  MICRO: { min: 0, max: 100 },
  SMALL: { min: 100, max: 1000 },
  MEDIUM: { min: 1000, max: 10000 },
  LARGE: { min: 10000, max: 100000 },
  HEAVY: { min: 100000, max: Infinity },
} as const;

export default AerospaceEngineSDK;
