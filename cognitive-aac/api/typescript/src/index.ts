/**
 * WIA Cognitive AAC Standard API
 * TypeScript Implementation
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 * 인지 장애 사용자를 위한 적응형 AAC 표준 라이브러리
 *
 * @packageDocumentation
 * @module wia-cognitive-aac
 * @license MIT
 * @author WIA / SmileStory Inc.
 * @version 1.0.0
 */

// Types
export * from './types';

// Engine
export {
  UIEngine,
  type AdaptiveUIEngine,
  type EngineConfig,
  ProfileAdapter,
  isAutismProfile,
  isDementiaProfile,
  COGNITIVE_LEVEL_PRESETS,
  AUTISM_SUPPORT_PRESETS,
  DEMENTIA_STAGE_PRESETS,
  SENSORY_ADJUSTMENTS,
  CELL_SIZE_PX,
  MIN_TOUCH_TARGET,
  getPresetForLevel,
  getAutismAdjustments,
  getDementiaAdjustments,
  mergeConfigurations,
} from './engine';

// Components
export {
  AdaptiveSymbol,
  type AdaptiveSymbolProps,
  AdaptiveGrid,
  type AdaptiveGridProps,
  AdaptiveButton,
  type AdaptiveButtonProps,
  type ButtonVariant,
  type ButtonSize,
  CognitiveLoadManagerProvider,
  useCognitiveLoadManager,
  CognitiveLoadIndicator,
  type CognitiveLoadState,
  type CognitiveLoadManagerContextValue,
  type LoadIndicatorProps,
} from './components';

// Hooks
export {
  useCognitiveProfile,
  type UseCognitiveProfileOptions,
  type UseCognitiveProfileReturn,
  useAdaptiveUI,
  type UseAdaptiveUIOptions,
  type UseAdaptiveUIReturn,
} from './hooks';

// Prediction (Phase 3)
export {
  PredictionEngine,
  PatternLearner,
  ContextDetector,
  PrivacyManager,
  type Context,
  type PredictedSymbol,
  type PredictionResult,
  type UsagePattern,
  type PrivacySettings,
  type PredictionEngineConfig,
  type RoutineBasedPrediction,
  type ReminiscenceBasedPrediction,
} from './prediction';

import {
  CognitiveProfile,
  AdaptiveUIConfig,
  Symbol,
  SymbolBoard,
  CommunicationSession,
  AssessmentResult,
  ApiResponse,
  PaginatedResponse,
} from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface CognitiveAACSDKConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  retries?: number;
  debug?: boolean;
}

// ============================================================================
// SDK Client
// ============================================================================

export class CognitiveAACSDK {
  private config: Required<CognitiveAACSDKConfig>;
  private headers: Record<string, string>;

  constructor(config: CognitiveAACSDKConfig) {
    this.config = {
      timeout: 30000,
      retries: 3,
      debug: false,
      ...config,
    };

    this.headers = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'X-WIA-Standard': 'COGNITIVE-AAC',
      'X-WIA-Version': '1.0.0',
    };
  }

  // ==========================================================================
  // Cognitive Profile APIs
  // ==========================================================================

  async getProfile(userId: string): Promise<ApiResponse<CognitiveProfile>> {
    return this.get<CognitiveProfile>(`/api/v1/profiles/${userId}`);
  }

  async createProfile(profile: Omit<CognitiveProfile, 'profileId'>): Promise<ApiResponse<CognitiveProfile>> {
    return this.post<CognitiveProfile>('/api/v1/profiles', profile);
  }

  async updateProfile(userId: string, updates: Partial<CognitiveProfile>): Promise<ApiResponse<CognitiveProfile>> {
    return this.put<CognitiveProfile>(`/api/v1/profiles/${userId}`, updates);
  }

  async deleteProfile(userId: string): Promise<ApiResponse<void>> {
    return this.delete<void>(`/api/v1/profiles/${userId}`);
  }

  // ==========================================================================
  // Assessment APIs
  // ==========================================================================

  async startAssessment(userId: string): Promise<ApiResponse<AssessmentResult>> {
    return this.post<AssessmentResult>(`/api/v1/profiles/${userId}/assessment/start`, {});
  }

  async submitAssessmentResponse(assessmentId: string, response: unknown): Promise<ApiResponse<void>> {
    return this.post<void>(`/api/v1/assessments/${assessmentId}/response`, response);
  }

  async completeAssessment(assessmentId: string): Promise<ApiResponse<AssessmentResult>> {
    return this.post<AssessmentResult>(`/api/v1/assessments/${assessmentId}/complete`, {});
  }

  async getAssessmentHistory(userId: string): Promise<ApiResponse<AssessmentResult[]>> {
    return this.get<AssessmentResult[]>(`/api/v1/profiles/${userId}/assessments`);
  }

  // ==========================================================================
  // Adaptive UI Configuration APIs
  // ==========================================================================

  async getUIConfig(userId: string): Promise<ApiResponse<AdaptiveUIConfig>> {
    return this.get<AdaptiveUIConfig>(`/api/v1/profiles/${userId}/ui-config`);
  }

  async updateUIConfig(userId: string, config: Partial<AdaptiveUIConfig>): Promise<ApiResponse<AdaptiveUIConfig>> {
    return this.put<AdaptiveUIConfig>(`/api/v1/profiles/${userId}/ui-config`, config);
  }

  async resetUIConfig(userId: string): Promise<ApiResponse<AdaptiveUIConfig>> {
    return this.post<AdaptiveUIConfig>(`/api/v1/profiles/${userId}/ui-config/reset`, {});
  }

  // ==========================================================================
  // Symbol Board APIs
  // ==========================================================================

  async getBoard(boardId: string): Promise<ApiResponse<SymbolBoard>> {
    return this.get<SymbolBoard>(`/api/v1/boards/${boardId}`);
  }

  async listBoards(userId: string): Promise<ApiResponse<SymbolBoard[]>> {
    return this.get<SymbolBoard[]>(`/api/v1/profiles/${userId}/boards`);
  }

  async createBoard(board: Omit<SymbolBoard, 'boardId'>): Promise<ApiResponse<SymbolBoard>> {
    return this.post<SymbolBoard>('/api/v1/boards', board);
  }

  async updateBoard(boardId: string, updates: Partial<SymbolBoard>): Promise<ApiResponse<SymbolBoard>> {
    return this.put<SymbolBoard>(`/api/v1/boards/${boardId}`, updates);
  }

  // ==========================================================================
  // Communication Session APIs
  // ==========================================================================

  async startSession(userId: string): Promise<ApiResponse<CommunicationSession>> {
    return this.post<CommunicationSession>('/api/v1/sessions', { userId });
  }

  async endSession(sessionId: string): Promise<ApiResponse<CommunicationSession>> {
    return this.post<CommunicationSession>(`/api/v1/sessions/${sessionId}/end`, {});
  }

  async recordSymbolSelection(sessionId: string, symbolId: string): Promise<ApiResponse<void>> {
    return this.post<void>(`/api/v1/sessions/${sessionId}/selections`, { symbolId });
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
}

// Version
export const VERSION = '1.0.0';

// Default export
export { UIEngine as default } from './engine';
