/**
 * WIA AAC (Augmentative and Alternative Communication) Standard API
 * TypeScript Implementation
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @packageDocumentation
 * @module wia-aac
 * @license MIT
 * @author WIA / SmileStory Inc.
 * @version 1.0.0
 */

// Export all types
export * from './types';

// Core classes
export { WiaAac } from './core/WiaAac';
export { WiaAacEventEmitter } from './core/EventEmitter';
export { SignalValidator, getSignalValidator } from './core/SignalValidator';

// Adapters
export {
  BaseAdapter,
  ISensorAdapter,
  SignalHandler,
  MockAdapter,
  MockAdapterOptions,
  EyeTrackerAdapter,
  SwitchAdapter,
  MuscleSensorAdapter,
  BrainInterfaceAdapter,
  BreathAdapter,
  HeadMovementAdapter
} from './adapters';

// Protocol
export * from './protocol';

// Transport
export * from './transport';

// Output
export * from './output';

import {
  SensorType,
  SensorStatus,
  Symbol,
  SymbolBoard,
  UserProfile,
  CommunicationSession,
  ApiResponse,
  PaginatedResponse,
  PaginationParams,
} from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface AACSDKConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  retries?: number;
  debug?: boolean;
}

// ============================================================================
// SDK Client
// ============================================================================

export class AACSDK {
  private config: Required<AACSDKConfig>;
  private headers: Record<string, string>;

  constructor(config: AACSDKConfig) {
    this.config = {
      timeout: 30000,
      retries: 3,
      debug: false,
      ...config,
    };

    this.headers = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'X-WIA-Standard': 'AAC',
      'X-WIA-Version': '1.0.0',
    };
  }

  // ==========================================================================
  // Symbol Management APIs
  // ==========================================================================

  async getSymbol(symbolId: string): Promise<ApiResponse<Symbol>> {
    return this.get<Symbol>(`/api/v1/symbols/${symbolId}`);
  }

  async listSymbols(
    params?: PaginationParams & { category?: string }
  ): Promise<ApiResponse<PaginatedResponse<Symbol>>> {
    const queryParams = this.buildQueryParams(params);
    return this.get<PaginatedResponse<Symbol>>(`/api/v1/symbols${queryParams}`);
  }

  async createSymbol(symbol: Omit<Symbol, 'symbolId'>): Promise<ApiResponse<Symbol>> {
    return this.post<Symbol>('/api/v1/symbols', symbol);
  }

  async updateSymbol(symbolId: string, updates: Partial<Symbol>): Promise<ApiResponse<Symbol>> {
    return this.put<Symbol>(`/api/v1/symbols/${symbolId}`, updates);
  }

  async deleteSymbol(symbolId: string): Promise<ApiResponse<void>> {
    return this.delete<void>(`/api/v1/symbols/${symbolId}`);
  }

  // ==========================================================================
  // Symbol Board APIs
  // ==========================================================================

  async getBoard(boardId: string): Promise<ApiResponse<SymbolBoard>> {
    return this.get<SymbolBoard>(`/api/v1/boards/${boardId}`);
  }

  async listBoards(userId: string): Promise<ApiResponse<SymbolBoard[]>> {
    return this.get<SymbolBoard[]>(`/api/v1/users/${userId}/boards`);
  }

  async createBoard(board: Omit<SymbolBoard, 'boardId'>): Promise<ApiResponse<SymbolBoard>> {
    return this.post<SymbolBoard>('/api/v1/boards', board);
  }

  async updateBoard(boardId: string, updates: Partial<SymbolBoard>): Promise<ApiResponse<SymbolBoard>> {
    return this.put<SymbolBoard>(`/api/v1/boards/${boardId}`, updates);
  }

  // ==========================================================================
  // User Profile APIs
  // ==========================================================================

  async getUserProfile(userId: string): Promise<ApiResponse<UserProfile>> {
    return this.get<UserProfile>(`/api/v1/users/${userId}`);
  }

  async updateUserProfile(userId: string, updates: Partial<UserProfile>): Promise<ApiResponse<UserProfile>> {
    return this.put<UserProfile>(`/api/v1/users/${userId}`, updates);
  }

  // ==========================================================================
  // Session APIs
  // ==========================================================================

  async startSession(userId: string): Promise<ApiResponse<CommunicationSession>> {
    return this.post<CommunicationSession>('/api/v1/sessions', { userId });
  }

  async endSession(sessionId: string): Promise<ApiResponse<CommunicationSession>> {
    return this.post<CommunicationSession>(`/api/v1/sessions/${sessionId}/end`, {});
  }

  async getSession(sessionId: string): Promise<ApiResponse<CommunicationSession>> {
    return this.get<CommunicationSession>(`/api/v1/sessions/${sessionId}`);
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

  private async delete<T>(path: string): Promise<ApiResponse<T>> {
    return this.request<T>('DELETE', path);
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

export const SENSOR_TYPES = {
  EYE_TRACKER: 'eye_tracker' as const,
  SWITCH: 'switch' as const,
  MUSCLE_SENSOR: 'muscle_sensor' as const,
  BRAIN_INTERFACE: 'brain_interface' as const,
  BREATH: 'breath' as const,
  HEAD_MOVEMENT: 'head_movement' as const,
  CUSTOM: 'custom' as const,
};

export const DEFAULT_CONFIG: Partial<AACSDKConfig> = {
  timeout: 30000,
  retries: 3,
  debug: false,
};
