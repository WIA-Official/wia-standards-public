/**
 * WIA Cognitive AAC - Prediction Engine
 * 사용자 의도 예측 엔진
 *
 * 홍익인간 (弘益人間) - 널리 인간을 이롭게 하라
 */

import {
  Context,
  PredictedSymbol,
  PredictionResult,
  RecommendedBoard,
  Phrase,
  PredictionEngineConfig,
  PredictionEngineState,
  PredictionEngineEvents,
  UsagePattern,
  RoutineBasedPrediction,
  ReminiscenceBasedPrediction,
  LearningEvent,
} from './types';
import { PatternLearner } from './PatternLearner';
import { ContextDetector } from './ContextDetector';
import { PrivacyManager } from './PrivacyManager';

// ============================================================================
// Default Configuration
// ============================================================================

const DEFAULT_CONFIG: PredictionEngineConfig = {
  maxPredictions: 8,
  minConfidence: 0.1,
  enableContextPrediction: true,
  enableSequencePrediction: true,
  enableTimePrediction: true,
  profileType: 'general',
  privacySettings: {
    localLearning: {
      enabled: true,
      modelStorage: 'local_only',
      noCloudUpload: true,
    },
    federatedLearning: {
      enabled: false,
      consentGiven: false,
      anonymization: 'full',
      aggregationOnly: true,
    },
    retention: {
      maxDays: 90,
      autoDelete: true,
      exportFormat: 'encrypted_backup',
    },
    dataMinimization: {
      excludePersonalInfo: true,
      excludeLocationData: false,
      excludeConversationContent: true,
    },
  },
  learnerConfig: {
    minFrequencyThreshold: 2,
    sequenceMaxLength: 5,
    decayFactor: 0.95,
    contextWeight: 0.3,
    recencyWeight: 0.4,
  },
};

// ============================================================================
// Prediction Engine Class
// ============================================================================

export class PredictionEngine {
  private config: PredictionEngineConfig;
  private state: PredictionEngineState;
  private patternLearner: PatternLearner;
  private contextDetector: ContextDetector;
  private privacyManager: PrivacyManager;
  private events: PredictionEngineEvents;

  // Autism-specific
  private routinePrediction: RoutineBasedPrediction | null = null;

  // Dementia-specific
  private reminiscencePrediction: ReminiscenceBasedPrediction | null = null;

  constructor(config?: Partial<PredictionEngineConfig>) {
    this.config = { ...DEFAULT_CONFIG, ...config };
    this.state = {
      isInitialized: false,
      isLearning: false,
      lastUpdate: 0,
      totalPredictions: 0,
      accuracy: 0,
    };
    this.events = {};

    // Initialize sub-modules
    this.patternLearner = new PatternLearner(this.config.learnerConfig);
    this.contextDetector = new ContextDetector();
    this.privacyManager = new PrivacyManager(this.config.privacySettings);
  }

  // ============================================================================
  // Factory Methods
  // ============================================================================

  static forAutism(config?: Partial<PredictionEngineConfig>): PredictionEngine {
    const engine = new PredictionEngine({
      ...config,
      profileType: 'autism',
      learnerConfig: {
        ...DEFAULT_CONFIG.learnerConfig,
        ...config?.learnerConfig,
        // 자폐: 루틴 중심 학습 강화
        contextWeight: 0.5,
        recencyWeight: 0.2,
      },
    });
    engine.initializeRoutinePrediction();
    return engine;
  }

  static forDementia(config?: Partial<PredictionEngineConfig>): PredictionEngine {
    const engine = new PredictionEngine({
      ...config,
      profileType: 'dementia',
      learnerConfig: {
        ...DEFAULT_CONFIG.learnerConfig,
        ...config?.learnerConfig,
        // 치매: 장기 기억 중심 학습
        decayFactor: 0.99, // 느린 감쇠
        minFrequencyThreshold: 1, // 낮은 임계값
      },
    });
    engine.initializeReminiscencePrediction();
    return engine;
  }

  // ============================================================================
  // Core Prediction Methods
  // ============================================================================

  /**
   * 다음 심볼 예측
   */
  predictNext(currentContext: Context, recentSymbols: string[]): PredictedSymbol[] {
    const predictions: PredictedSymbol[] = [];

    // 1. 빈도 기반 예측
    const frequencyPredictions = this.predictByFrequency(currentContext);
    predictions.push(...frequencyPredictions);

    // 2. 시퀀스 기반 예측
    if (this.config.enableSequencePrediction && recentSymbols.length > 0) {
      const sequencePredictions = this.predictBySequence(recentSymbols);
      predictions.push(...sequencePredictions);
    }

    // 3. 컨텍스트 기반 예측
    if (this.config.enableContextPrediction) {
      const contextPredictions = this.predictByContext(currentContext);
      predictions.push(...contextPredictions);
    }

    // 4. 시간 기반 예측
    if (this.config.enableTimePrediction) {
      const timePredictions = this.predictByTime(currentContext);
      predictions.push(...timePredictions);
    }

    // 5. 프로파일 특화 예측
    if (this.config.profileType === 'autism' && this.routinePrediction) {
      const routinePredictions = this.predictByRoutine(currentContext);
      predictions.push(...routinePredictions);
    }

    if (this.config.profileType === 'dementia' && this.reminiscencePrediction) {
      const reminiscencePredictions = this.predictByReminiscence(currentContext);
      predictions.push(...reminiscencePredictions);
    }

    // 예측 결과 병합 및 정렬
    const mergedPredictions = this.mergePredictions(predictions);
    const topPredictions = mergedPredictions
      .filter((p) => p.probability >= this.config.minConfidence)
      .slice(0, this.config.maxPredictions);

    // 순위 할당
    topPredictions.forEach((p, idx) => {
      p.rank = idx + 1;
    });

    // 상태 업데이트
    this.state.totalPredictions++;
    this.state.lastUpdate = Date.now();

    // 이벤트 발생
    const result: PredictionResult = {
      predictions: topPredictions,
      context: currentContext,
      timestamp: Date.now(),
      modelVersion: '1.0.0',
    };
    this.events.onPrediction?.(result);

    return topPredictions;
  }

  /**
   * 문장 완성 예측
   */
  predictCompletion(partialPhrase: string[], context: Context): Phrase[] {
    const patterns = this.patternLearner.getUsagePatterns();
    const matchingPhrases: Phrase[] = [];

    // 시작 부분이 일치하는 문장 찾기
    for (const phrase of patterns.sequences.commonPhrases) {
      if (this.isPrefix(partialPhrase, phrase.symbols)) {
        matchingPhrases.push(phrase);
      }
    }

    // 빈도순 정렬
    matchingPhrases.sort((a, b) => b.frequency - a.frequency);

    // 컨텍스트 필터링
    if (context.recentActivity || context.conversationPartner) {
      return matchingPhrases.filter((p) => {
        if (!p.context) return true;
        return (
          p.context === context.recentActivity ||
          p.context === context.conversationPartner
        );
      });
    }

    return matchingPhrases.slice(0, 5);
  }

  /**
   * 상황 기반 보드 추천
   */
  recommendForContext(context: Context): RecommendedBoard {
    const detectedContext = this.contextDetector.analyze(context);
    const patterns = this.patternLearner.getUsagePatterns();

    // 컨텍스트에 맞는 심볼 선택
    let relevantSymbols: PredictedSymbol[] = [];
    let reason = '';

    // 시간대 기반
    const hour = context.time.getHours();
    if (patterns.temporal.hourlyFrequency.has(hour)) {
      const hourlySymbols = patterns.temporal.hourlyFrequency.get(hour)!;
      relevantSymbols = hourlySymbols.map((sf, idx) => ({
        symbolId: sf.symbolId,
        probability: sf.count / this.getTotalCount(hourlySymbols),
        source: 'time' as const,
        explanation: `이 시간에 자주 사용해요`,
        rank: idx + 1,
      }));
      reason = `${hour}시에 자주 사용하는 심볼`;
    }

    // 활동 기반
    if (context.recentActivity && patterns.contextual.activityBased.has(context.recentActivity)) {
      const activitySymbols = patterns.contextual.activityBased.get(context.recentActivity)!;
      const activityPredictions = activitySymbols.map((sf, idx) => ({
        symbolId: sf.symbolId,
        probability: sf.count / this.getTotalCount(activitySymbols),
        source: 'context' as const,
        explanation: `${context.recentActivity} 중에 자주 사용해요`,
        rank: idx + 1,
      }));
      relevantSymbols = this.mergePredictions([...relevantSymbols, ...activityPredictions]);
      reason = `${context.recentActivity} 활동을 위한 심볼`;
    }

    // 대화 상대 기반
    if (context.conversationPartner && patterns.contextual.personBased.has(context.conversationPartner)) {
      const personSymbols = patterns.contextual.personBased.get(context.conversationPartner)!;
      const personPredictions = personSymbols.map((sf, idx) => ({
        symbolId: sf.symbolId,
        probability: sf.count / this.getTotalCount(personSymbols),
        source: 'context' as const,
        explanation: `${context.conversationPartner}님과 대화할 때 자주 사용해요`,
        rank: idx + 1,
      }));
      relevantSymbols = this.mergePredictions([...relevantSymbols, ...personPredictions]);
      reason = `${context.conversationPartner}님과의 대화를 위한 심볼`;
    }

    // 상위 심볼 선택
    const topSymbols = relevantSymbols.slice(0, this.config.maxPredictions);

    return {
      boardId: `recommended_${Date.now()}`,
      symbols: topSymbols,
      reason: reason || '자주 사용하는 심볼',
      confidence: topSymbols.length > 0 ? topSymbols[0].probability : 0,
    };
  }

  // ============================================================================
  // Learning Methods
  // ============================================================================

  /**
   * 학습 이벤트 기록
   */
  recordLearning(event: Omit<LearningEvent, 'timestamp'>): void {
    if (!this.config.privacySettings.localLearning.enabled) return;

    const learningEvent: LearningEvent = {
      ...event,
      timestamp: Date.now(),
    };

    // 프라이버시 필터링
    const filteredEvent = this.privacyManager.filterLearningEvent(learningEvent);
    if (!filteredEvent) return;

    // 패턴 학습
    this.patternLearner.learn(filteredEvent);
    this.state.isLearning = true;
    this.state.lastUpdate = Date.now();

    // 이벤트 발생
    this.events.onLearning?.(filteredEvent);
  }

  /**
   * 심볼 선택 기록
   */
  recordSymbolSelection(symbolId: string, context: Context, responseTime?: number): void {
    this.recordLearning({
      type: 'symbol_selection',
      data: { symbolId, responseTime },
      context,
    });
  }

  /**
   * 문장 완성 기록
   */
  recordPhraseCompletion(symbols: string[], context: Context): void {
    this.recordLearning({
      type: 'phrase_completion',
      data: { symbols },
      context,
    });
  }

  // ============================================================================
  // Autism-specific Methods
  // ============================================================================

  private initializeRoutinePrediction(): void {
    this.routinePrediction = {
      dailyRoutine: [],
      currentStep: null,
      nextStep: null,
      deviationHistory: [],
    };
  }

  /**
   * 일과 설정
   */
  setDailyRoutine(routine: RoutineBasedPrediction['dailyRoutine']): void {
    if (this.routinePrediction) {
      this.routinePrediction.dailyRoutine = routine;
    }
  }

  /**
   * 루틴 이탈 감지
   */
  detectRoutineDeviation(currentBehavior: string, expectedTime: Date): boolean {
    if (!this.routinePrediction) return false;

    const currentStep = this.getCurrentRoutineStep(expectedTime);
    if (!currentStep) return false;

    const isDeviation = currentBehavior !== currentStep.activity;
    if (isDeviation) {
      this.routinePrediction.deviationHistory.push({
        expectedStep: currentStep,
        actualBehavior: currentBehavior,
        severity: 'moderate',
        timestamp: Date.now(),
      });
    }

    return isDeviation;
  }

  /**
   * 루틴 복귀 제안
   */
  suggestReturnToRoutine(): string[] {
    if (!this.routinePrediction?.currentStep) return [];
    return this.routinePrediction.currentStep.requiredSymbols;
  }

  /**
   * 다음 활동 전환 준비
   */
  prepareTransition(): { nextActivity: string; warningSymbols: string[] } | null {
    if (!this.routinePrediction?.nextStep) return null;

    return {
      nextActivity: this.routinePrediction.nextStep.activity,
      warningSymbols: this.routinePrediction.nextStep.transitionCues ?? [],
    };
  }

  // ============================================================================
  // Dementia-specific Methods
  // ============================================================================

  private initializeReminiscencePrediction(): void {
    this.reminiscencePrediction = {
      longTermMemoryTriggers: {
        familiarPhotos: [],
        significantDates: [],
        musicMemories: [],
      },
      currentOrientationSupport: {
        temporal: this.createTemporalInfo(),
        location: null,
        recognizedPeople: [],
      },
    };
  }

  /**
   * 회상 자료 설정
   */
  setReminiscenceContent(
    photos: ReminiscenceBasedPrediction['longTermMemoryTriggers']['familiarPhotos'],
    dates: ReminiscenceBasedPrediction['longTermMemoryTriggers']['significantDates'],
    music: ReminiscenceBasedPrediction['longTermMemoryTriggers']['musicMemories']
  ): void {
    if (this.reminiscencePrediction) {
      this.reminiscencePrediction.longTermMemoryTriggers = {
        familiarPhotos: photos,
        significantDates: dates,
        musicMemories: music,
      };
    }
  }

  /**
   * 반복 질문 감지
   */
  detectRepetitiveQuestion(recentQuestions: string[]): boolean {
    if (recentQuestions.length < 2) return false;

    // 최근 5분 내 동일 질문 반복 감지
    const questionCounts = new Map<string, number>();
    for (const q of recentQuestions) {
      const normalized = q.toLowerCase().trim();
      questionCounts.set(normalized, (questionCounts.get(normalized) ?? 0) + 1);
    }

    return Array.from(questionCounts.values()).some((count) => count >= 2);
  }

  /**
   * 부드러운 재안내 생성
   */
  generateGentleRedirect(questionType: string): { message: string; symbols: string[] } {
    const redirects: Record<string, { message: string; symbols: string[] }> = {
      time: {
        message: '지금은 {time}이에요. 오늘은 {day}이고요.',
        symbols: ['time', 'today', 'calendar'],
      },
      location: {
        message: '여기는 {location}이에요. 안전한 곳이에요.',
        symbols: ['home', 'safe', 'here'],
      },
      person: {
        message: '저는 {name}이에요. 항상 곁에 있어요.',
        symbols: ['caregiver', 'help', 'together'],
      },
    };

    return redirects[questionType] ?? {
      message: '함께 이야기해요.',
      symbols: ['talk', 'together', 'help'],
    };
  }

  /**
   * 시간 지남력 지원 정보
   */
  getOrientationSupport(): ReminiscenceBasedPrediction['currentOrientationSupport'] | null {
    if (!this.reminiscencePrediction) return null;

    // 현재 시간 정보 업데이트
    this.reminiscencePrediction.currentOrientationSupport.temporal = this.createTemporalInfo();

    return this.reminiscencePrediction.currentOrientationSupport;
  }

  // ============================================================================
  // Private Prediction Methods
  // ============================================================================

  private predictByFrequency(context: Context): PredictedSymbol[] {
    const patterns = this.patternLearner.getUsagePatterns();
    const allSymbols = this.aggregateAllSymbols(patterns);

    const totalCount = this.getTotalCount(allSymbols);
    if (totalCount === 0) return [];

    return allSymbols.slice(0, 10).map((sf, idx) => ({
      symbolId: sf.symbolId,
      probability: sf.count / totalCount,
      source: 'frequency',
      explanation: '자주 사용하는 심볼이에요',
      rank: idx + 1,
    }));
  }

  private predictBySequence(recentSymbols: string[]): PredictedSymbol[] {
    const patterns = this.patternLearner.getUsagePatterns();
    const predictions: PredictedSymbol[] = [];

    for (const chain of patterns.sequences.symbolChains) {
      if (this.matchesSequenceEnd(recentSymbols, chain.symbols)) {
        const nextIdx = recentSymbols.length;
        if (nextIdx < chain.symbols.length) {
          predictions.push({
            symbolId: chain.symbols[nextIdx],
            probability: chain.probability,
            source: 'sequence',
            explanation: '이전에 이어서 사용했던 심볼이에요',
            rank: predictions.length + 1,
          });
        }
      }
    }

    return predictions;
  }

  private predictByContext(context: Context): PredictedSymbol[] {
    const patterns = this.patternLearner.getUsagePatterns();
    const predictions: PredictedSymbol[] = [];

    // 활동 컨텍스트
    if (context.recentActivity) {
      const activitySymbols = patterns.contextual.activityBased.get(context.recentActivity);
      if (activitySymbols) {
        const total = this.getTotalCount(activitySymbols);
        activitySymbols.slice(0, 3).forEach((sf, idx) => {
          predictions.push({
            symbolId: sf.symbolId,
            probability: (sf.count / total) * this.config.learnerConfig.contextWeight,
            source: 'context',
            explanation: `${context.recentActivity} 중에 자주 사용해요`,
            rank: idx + 1,
          });
        });
      }
    }

    // 대화 상대 컨텍스트
    if (context.conversationPartner) {
      const personSymbols = patterns.contextual.personBased.get(context.conversationPartner);
      if (personSymbols) {
        const total = this.getTotalCount(personSymbols);
        personSymbols.slice(0, 3).forEach((sf, idx) => {
          predictions.push({
            symbolId: sf.symbolId,
            probability: (sf.count / total) * this.config.learnerConfig.contextWeight,
            source: 'context',
            explanation: `${context.conversationPartner}님과 대화할 때 자주 사용해요`,
            rank: idx + 1,
          });
        });
      }
    }

    return predictions;
  }

  private predictByTime(context: Context): PredictedSymbol[] {
    const patterns = this.patternLearner.getUsagePatterns();
    const hour = context.time.getHours();

    const hourlySymbols = patterns.temporal.hourlyFrequency.get(hour);
    if (!hourlySymbols) return [];

    const total = this.getTotalCount(hourlySymbols);
    return hourlySymbols.slice(0, 5).map((sf, idx) => ({
      symbolId: sf.symbolId,
      probability: sf.count / total,
      source: 'time',
      explanation: `이 시간에 자주 사용해요`,
      rank: idx + 1,
    }));
  }

  private predictByRoutine(context: Context): PredictedSymbol[] {
    if (!this.routinePrediction) return [];

    const currentStep = this.getCurrentRoutineStep(context.time);
    if (!currentStep) return [];

    this.routinePrediction.currentStep = currentStep;

    // 다음 스텝 계산
    const routineIdx = this.routinePrediction.dailyRoutine.indexOf(currentStep);
    if (routineIdx < this.routinePrediction.dailyRoutine.length - 1) {
      this.routinePrediction.nextStep = this.routinePrediction.dailyRoutine[routineIdx + 1];
    }

    return currentStep.requiredSymbols.map((symbolId, idx) => ({
      symbolId,
      probability: 0.8,
      source: 'routine',
      explanation: `${currentStep.activity} 시간이에요`,
      rank: idx + 1,
    }));
  }

  private predictByReminiscence(context: Context): PredictedSymbol[] {
    if (!this.reminiscencePrediction) return [];

    const predictions: PredictedSymbol[] = [];

    // 오늘 날짜와 관련된 기념일 확인
    const today = context.time;
    for (const sigDate of this.reminiscencePrediction.longTermMemoryTriggers.significantDates) {
      if (
        sigDate.date.getMonth() === today.getMonth() &&
        sigDate.date.getDate() === today.getDate()
      ) {
        predictions.push({
          symbolId: `memory_${sigDate.description}`,
          probability: 0.7,
          source: 'reminiscence',
          explanation: `오늘은 특별한 날이에요: ${sigDate.description}`,
          rank: predictions.length + 1,
        });
      }
    }

    return predictions;
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  private getCurrentRoutineStep(time: Date): RoutineBasedPrediction['dailyRoutine'][0] | null {
    if (!this.routinePrediction) return null;

    const currentHour = time.getHours();
    const currentMinute = time.getMinutes();
    const currentMinutes = currentHour * 60 + currentMinute;

    for (const step of this.routinePrediction.dailyRoutine) {
      if (step.typicalTime) {
        const stepMinutes = step.typicalTime.hour * 60 + step.typicalTime.minute;
        const duration = step.duration ?? 30;

        if (currentMinutes >= stepMinutes && currentMinutes < stepMinutes + duration) {
          return step;
        }
      }
    }

    return null;
  }

  private createTemporalInfo(): ReminiscenceBasedPrediction['currentOrientationSupport']['temporal'] {
    const now = new Date();
    const days = ['일요일', '월요일', '화요일', '수요일', '목요일', '금요일', '토요일'];
    const hour = now.getHours();

    let timeOfDay: string;
    if (hour < 6) timeOfDay = '새벽';
    else if (hour < 12) timeOfDay = '아침';
    else if (hour < 18) timeOfDay = '오후';
    else timeOfDay = '저녁';

    return {
      currentTime: now,
      dayOfWeek: days[now.getDay()],
      timeOfDay,
      displayFormat: 'simple',
    };
  }

  private mergePredictions(predictions: PredictedSymbol[]): PredictedSymbol[] {
    const merged = new Map<string, PredictedSymbol>();

    for (const pred of predictions) {
      const existing = merged.get(pred.symbolId);
      if (existing) {
        // 확률 합산 (최대 1)
        existing.probability = Math.min(1, existing.probability + pred.probability * 0.5);
        // 더 높은 확률의 소스 유지
        if (pred.probability > existing.probability) {
          existing.source = pred.source;
          existing.explanation = pred.explanation;
        }
      } else {
        merged.set(pred.symbolId, { ...pred });
      }
    }

    return Array.from(merged.values()).sort((a, b) => b.probability - a.probability);
  }

  private aggregateAllSymbols(patterns: UsagePattern): { symbolId: string; count: number }[] {
    const aggregated = new Map<string, number>();

    // 모든 시간대 집계
    patterns.temporal.hourlyFrequency.forEach((symbols) => {
      symbols.forEach((sf) => {
        aggregated.set(sf.symbolId, (aggregated.get(sf.symbolId) ?? 0) + sf.count);
      });
    });

    return Array.from(aggregated.entries())
      .map(([symbolId, count]) => ({ symbolId, count }))
      .sort((a, b) => b.count - a.count);
  }

  private getTotalCount(symbols: { count: number }[]): number {
    return symbols.reduce((sum, s) => sum + s.count, 0);
  }

  private isPrefix(prefix: string[], full: string[]): boolean {
    if (prefix.length >= full.length) return false;
    return prefix.every((p, i) => p === full[i]);
  }

  private matchesSequenceEnd(recent: string[], chain: string[]): boolean {
    if (recent.length === 0 || recent.length >= chain.length) return false;
    return recent.every((r, i) => r === chain[i]);
  }

  // ============================================================================
  // Public API
  // ============================================================================

  getState(): PredictionEngineState {
    return { ...this.state };
  }

  getConfig(): PredictionEngineConfig {
    return { ...this.config };
  }

  updateConfig(config: Partial<PredictionEngineConfig>): void {
    this.config = { ...this.config, ...config };
  }

  setEventHandlers(events: PredictionEngineEvents): void {
    this.events = events;
  }

  reset(): void {
    this.patternLearner.reset();
    this.state = {
      isInitialized: false,
      isLearning: false,
      lastUpdate: 0,
      totalPredictions: 0,
      accuracy: 0,
    };
  }

  exportPatterns(): UsagePattern {
    return this.patternLearner.getUsagePatterns();
  }

  importPatterns(patterns: UsagePattern): void {
    this.patternLearner.importPatterns(patterns);
  }
}

export default PredictionEngine;
