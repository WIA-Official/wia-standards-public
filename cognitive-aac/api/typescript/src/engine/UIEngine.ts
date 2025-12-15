/**
 * WIA Cognitive AAC - Adaptive UI Engine
 * 인지 프로파일 기반 적응형 UI 엔진
 *
 * 홍익인간 - 널리 인간을 이롭게 하라
 */

import {
  CognitiveProfile,
  AutismProfile,
  DementiaProfile,
  UIConfiguration,
  UIAdjustment,
  UserInteractionEvent,
  UsageHistory,
  AggregatedMetrics,
  SessionData,
  CognitiveLevel,
} from '../types';

import { ProfileAdapter, isAutismProfile, isDementiaProfile } from './ProfileAdapter';
import { COGNITIVE_LEVEL_PRESETS, mergeConfigurations } from './Presets';

// ============================================================================
// UI Engine Interface
// ============================================================================

export interface AdaptiveUIEngine {
  generateUI(profile: CognitiveProfile): UIConfiguration;
  adjustUI(event: UserInteractionEvent): UIAdjustment | null;
  optimizeFromUsage(history: UsageHistory): UIConfiguration;
}

// ============================================================================
// Engine Configuration
// ============================================================================

export interface EngineConfig {
  enableRealTimeAdjustment: boolean;
  enableLearning: boolean;
  adjustmentThreshold: number;
  maxAdjustmentsPerSession: number;
  minEventsForLearning: number;
}

const DEFAULT_ENGINE_CONFIG: EngineConfig = {
  enableRealTimeAdjustment: true,
  enableLearning: true,
  adjustmentThreshold: 0.3,
  maxAdjustmentsPerSession: 10,
  minEventsForLearning: 50,
};

// ============================================================================
// Cognitive Load Monitor
// ============================================================================

interface CognitiveLoadState {
  currentLoad: number; // 0-1
  errorCount: number;
  hesitationCount: number;
  averageSelectionTime: number;
  sessionDuration: number;
  lastEventTime: number;
}

// ============================================================================
// UI Engine Implementation
// ============================================================================

export class UIEngine implements AdaptiveUIEngine {
  private config: EngineConfig;
  private currentProfile: CognitiveProfile | null = null;
  private currentConfig: UIConfiguration | null = null;
  private loadState: CognitiveLoadState;
  private adjustmentCount: number = 0;
  private eventBuffer: UserInteractionEvent[] = [];

  constructor(config: Partial<EngineConfig> = {}) {
    this.config = { ...DEFAULT_ENGINE_CONFIG, ...config };
    this.loadState = this.initializeLoadState();
  }

  // ==========================================================================
  // Public Methods
  // ==========================================================================

  /**
   * 인지 프로파일 기반 UI 설정 생성
   */
  generateUI(profile: CognitiveProfile): UIConfiguration {
    this.currentProfile = profile;
    this.currentConfig = ProfileAdapter.generateUIConfiguration(profile);
    this.resetSessionState();

    return this.currentConfig;
  }

  /**
   * 사용자 상호작용 이벤트 기반 실시간 UI 조정
   */
  adjustUI(event: UserInteractionEvent): UIAdjustment | null {
    if (!this.config.enableRealTimeAdjustment || !this.currentConfig) {
      return null;
    }

    // 이벤트 버퍼에 추가
    this.eventBuffer.push(event);

    // 인지 부하 상태 업데이트
    this.updateLoadState(event);

    // 조정 필요 여부 판단
    if (this.shouldAdjust()) {
      return this.calculateAdjustment(event);
    }

    return null;
  }

  /**
   * 사용 이력 기반 UI 최적화
   */
  optimizeFromUsage(history: UsageHistory): UIConfiguration {
    if (!this.config.enableLearning || !this.currentProfile) {
      return this.currentConfig || this.getDefaultConfig();
    }

    if (history.sessions.length < this.config.minEventsForLearning) {
      return this.currentConfig || this.getDefaultConfig();
    }

    const metrics = history.aggregatedMetrics;
    const optimizedConfig = this.calculateOptimalConfig(metrics);

    this.currentConfig = optimizedConfig;
    return optimizedConfig;
  }

  /**
   * 현재 설정 반환
   */
  getCurrentConfig(): UIConfiguration | null {
    return this.currentConfig;
  }

  /**
   * 현재 인지 부하 상태 반환
   */
  getLoadState(): CognitiveLoadState {
    return { ...this.loadState };
  }

  /**
   * 자폐 프로파일 특수 관심사 요소 반환
   */
  getSpecialInterestElements(): ReturnType<typeof ProfileAdapter.getSpecialInterestElements> | null {
    if (this.currentProfile && isAutismProfile(this.currentProfile)) {
      return ProfileAdapter.getSpecialInterestElements(this.currentProfile);
    }
    return null;
  }

  /**
   * 치매 프로파일 회상 요소 반환
   */
  getReminiscenceElements(): ReturnType<typeof ProfileAdapter.getReminiscenceElements> | null {
    if (this.currentProfile && isDementiaProfile(this.currentProfile)) {
      return ProfileAdapter.getReminiscenceElements(this.currentProfile);
    }
    return null;
  }

  // ==========================================================================
  // Private Methods - State Management
  // ==========================================================================

  private initializeLoadState(): CognitiveLoadState {
    return {
      currentLoad: 0,
      errorCount: 0,
      hesitationCount: 0,
      averageSelectionTime: 0,
      sessionDuration: 0,
      lastEventTime: Date.now(),
    };
  }

  private resetSessionState(): void {
    this.loadState = this.initializeLoadState();
    this.adjustmentCount = 0;
    this.eventBuffer = [];
  }

  private updateLoadState(event: UserInteractionEvent): void {
    const now = event.timestamp;
    this.loadState.sessionDuration = now - (this.eventBuffer[0]?.timestamp || now);
    this.loadState.lastEventTime = now;

    switch (event.type) {
      case 'error':
        this.loadState.errorCount++;
        this.loadState.currentLoad = Math.min(1, this.loadState.currentLoad + 0.1);
        break;

      case 'hesitation':
        this.loadState.hesitationCount++;
        this.loadState.currentLoad = Math.min(1, this.loadState.currentLoad + 0.05);
        break;

      case 'selection':
        if (event.duration) {
          this.updateAverageSelectionTime(event.duration);
        }
        if (event.successful) {
          this.loadState.currentLoad = Math.max(0, this.loadState.currentLoad - 0.02);
        }
        break;

      case 'timeout':
        this.loadState.currentLoad = Math.min(1, this.loadState.currentLoad + 0.15);
        break;
    }

    // 시간 경과에 따른 부하 감소 (회복)
    const timeSinceLastEvent = now - this.loadState.lastEventTime;
    if (timeSinceLastEvent > 5000) {
      this.loadState.currentLoad = Math.max(
        0,
        this.loadState.currentLoad - (timeSinceLastEvent / 60000) * 0.1
      );
    }
  }

  private updateAverageSelectionTime(duration: number): void {
    const selectionEvents = this.eventBuffer.filter((e) => e.type === 'selection');
    const count = selectionEvents.length;

    if (count === 0) {
      this.loadState.averageSelectionTime = duration;
    } else {
      this.loadState.averageSelectionTime =
        (this.loadState.averageSelectionTime * (count - 1) + duration) / count;
    }
  }

  // ==========================================================================
  // Private Methods - Adjustment Logic
  // ==========================================================================

  private shouldAdjust(): boolean {
    // 세션당 최대 조정 횟수 초과
    if (this.adjustmentCount >= this.config.maxAdjustmentsPerSession) {
      return false;
    }

    // 인지 부하가 임계값 초과
    if (this.loadState.currentLoad >= this.config.adjustmentThreshold) {
      return true;
    }

    // 연속 오류 감지
    const recentEvents = this.eventBuffer.slice(-5);
    const recentErrors = recentEvents.filter((e) => e.type === 'error').length;
    if (recentErrors >= 3) {
      return true;
    }

    // 연속 타임아웃 감지
    const recentTimeouts = recentEvents.filter((e) => e.type === 'timeout').length;
    if (recentTimeouts >= 2) {
      return true;
    }

    return false;
  }

  private calculateAdjustment(event: UserInteractionEvent): UIAdjustment {
    if (!this.currentConfig) {
      return this.createNullAdjustment();
    }

    this.adjustmentCount++;

    const loadLevel = this.loadState.currentLoad;
    const changes: Partial<UIConfiguration> = {};
    let reason = '';
    let confidence = 0.5;

    // 높은 인지 부하 대응
    if (loadLevel >= 0.7) {
      // 극단적 단순화
      changes.cognitiveLoad = {
        ...this.currentConfig.cognitiveLoad,
        maxVisibleItems: Math.max(4, Math.floor(this.currentConfig.cognitiveLoad.maxVisibleItems * 0.6)),
        distractionFilter: Math.min(1, this.currentConfig.cognitiveLoad.distractionFilter + 0.3),
        simplificationLevel: Math.min(5, this.currentConfig.cognitiveLoad.simplificationLevel + 1),
      };
      changes.grid = {
        ...this.currentConfig.grid,
        columns: Math.max(2, this.currentConfig.grid.columns - 1),
        rows: Math.max(2, this.currentConfig.grid.rows - 1),
      };
      reason = 'High cognitive load detected - simplifying interface';
      confidence = 0.8;
    }
    // 중간 인지 부하 대응
    else if (loadLevel >= 0.5) {
      changes.cognitiveLoad = {
        ...this.currentConfig.cognitiveLoad,
        maxVisibleItems: Math.max(6, Math.floor(this.currentConfig.cognitiveLoad.maxVisibleItems * 0.8)),
        progressiveDisclosure: true,
      };
      changes.interaction = {
        ...this.currentConfig.interaction,
        dwellTime: Math.min(2000, this.currentConfig.interaction.dwellTime + 200),
      };
      reason = 'Moderate cognitive load - reducing complexity';
      confidence = 0.7;
    }
    // 오류 기반 조정
    else if (event.type === 'error') {
      changes.interaction = {
        ...this.currentConfig.interaction,
        confirmationRequired: true,
        audioFeedback: true,
      };
      reason = 'Error detected - adding confirmation and feedback';
      confidence = 0.6;
    }
    // 망설임 기반 조정
    else if (event.type === 'hesitation') {
      changes.symbols = {
        ...this.currentConfig.symbols,
        highContrast: true,
      };
      changes.grid = {
        ...this.currentConfig.grid,
        cellSize: this.increaseCellSize(this.currentConfig.grid.cellSize),
      };
      reason = 'Hesitation detected - improving visibility';
      confidence = 0.5;
    }
    // 타임아웃 기반 조정
    else if (event.type === 'timeout') {
      changes.interaction = {
        ...this.currentConfig.interaction,
        dwellTime: Math.min(3000, this.currentConfig.interaction.dwellTime + 500),
      };
      reason = 'Timeout detected - increasing response time';
      confidence = 0.7;
    }

    return {
      type: this.determineAdjustmentType(changes),
      changes,
      reason,
      confidence,
    };
  }

  private createNullAdjustment(): UIAdjustment {
    return {
      type: 'cognitiveLoad',
      changes: {},
      reason: 'No adjustment needed',
      confidence: 0,
    };
  }

  private determineAdjustmentType(changes: Partial<UIConfiguration>): UIAdjustment['type'] {
    if (changes.grid) return 'grid';
    if (changes.symbols) return 'symbols';
    if (changes.interaction) return 'interaction';
    if (changes.theme) return 'theme';
    return 'cognitiveLoad';
  }

  private increaseCellSize(current: UIConfiguration['grid']['cellSize']): UIConfiguration['grid']['cellSize'] {
    const sizes: UIConfiguration['grid']['cellSize'][] = ['small', 'medium', 'large', 'xlarge'];
    const currentIndex = sizes.indexOf(current);
    return sizes[Math.min(currentIndex + 1, sizes.length - 1)];
  }

  // ==========================================================================
  // Private Methods - Learning & Optimization
  // ==========================================================================

  private calculateOptimalConfig(metrics: AggregatedMetrics): UIConfiguration {
    if (!this.currentConfig || !this.currentProfile) {
      return this.getDefaultConfig();
    }

    const baseConfig = { ...this.currentConfig };

    // 평균 선택 시간 기반 dwell time 최적화
    if (metrics.averageSelectionTime > 0) {
      baseConfig.interaction.dwellTime = Math.round(
        metrics.averageSelectionTime * 0.8 // 80%를 최적 dwell time으로
      );
    }

    // 오류율 기반 조정
    if (metrics.errorRate > 0.2) {
      // 20% 이상 오류
      baseConfig.cognitiveLoad.maxVisibleItems = Math.max(
        4,
        Math.floor(baseConfig.cognitiveLoad.maxVisibleItems * 0.7)
      );
      baseConfig.interaction.confirmationRequired = true;
    } else if (metrics.errorRate < 0.05) {
      // 5% 미만 오류 - 복잡도 증가 가능
      baseConfig.cognitiveLoad.maxVisibleItems = Math.min(
        30,
        Math.floor(baseConfig.cognitiveLoad.maxVisibleItems * 1.2)
      );
    }

    // 선호 그리드 크기 적용
    if (metrics.preferredGridSize) {
      baseConfig.grid.columns = metrics.preferredGridSize.columns;
      baseConfig.grid.rows = metrics.preferredGridSize.rows;
    }

    // 최적 dwell time 적용
    if (metrics.optimalDwellTime > 0) {
      baseConfig.interaction.dwellTime = metrics.optimalDwellTime;
    }

    return baseConfig;
  }

  private getDefaultConfig(): UIConfiguration {
    return COGNITIVE_LEVEL_PRESETS[CognitiveLevel.MODERATE];
  }

  // ==========================================================================
  // Static Factory Methods
  // ==========================================================================

  /**
   * 자폐 프로파일용 엔진 생성
   */
  static forAutism(profile: AutismProfile, config?: Partial<EngineConfig>): UIEngine {
    const engine = new UIEngine({
      ...config,
      enableRealTimeAdjustment: true,
      adjustmentThreshold: 0.25, // 더 민감한 조정
    });
    engine.generateUI(profile);
    return engine;
  }

  /**
   * 치매 프로파일용 엔진 생성
   */
  static forDementia(profile: DementiaProfile, config?: Partial<EngineConfig>): UIEngine {
    const engine = new UIEngine({
      ...config,
      enableRealTimeAdjustment: true,
      enableLearning: false, // 치매의 경우 학습보다 안정성 우선
      maxAdjustmentsPerSession: 5, // 변화 최소화
    });
    engine.generateUI(profile);
    return engine;
  }

  /**
   * 일반 프로파일용 엔진 생성
   */
  static forGeneral(profile: CognitiveProfile, config?: Partial<EngineConfig>): UIEngine {
    const engine = new UIEngine(config);
    engine.generateUI(profile);
    return engine;
  }
}

export default UIEngine;
