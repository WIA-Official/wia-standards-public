/**
 * WIA Cognitive AAC - Pattern Learner
 * 사용 패턴 학습 모듈
 *
 * 홍익인간 (弘益人間) - 널리 인간을 이롭게 하라
 */

import {
  UsagePattern,
  SymbolFrequency,
  Phrase,
  SymbolChain,
  ConversationPattern,
  LearningEvent,
  PatternLearnerConfig,
  Context,
} from './types';

// ============================================================================
// Default Configuration
// ============================================================================

const DEFAULT_LEARNER_CONFIG: PatternLearnerConfig = {
  minFrequencyThreshold: 2,
  sequenceMaxLength: 5,
  decayFactor: 0.95,
  contextWeight: 0.3,
  recencyWeight: 0.4,
};

// ============================================================================
// Pattern Learner Class
// ============================================================================

export class PatternLearner {
  private config: PatternLearnerConfig;
  private patterns: UsagePattern;
  private recentSymbols: string[] = [];
  private currentSessionSymbols: string[] = [];
  private lastDecayTime: number = Date.now();

  constructor(config?: Partial<PatternLearnerConfig>) {
    this.config = { ...DEFAULT_LEARNER_CONFIG, ...config };
    this.patterns = this.createEmptyPatterns();
  }

  // ============================================================================
  // Learning Methods
  // ============================================================================

  /**
   * 학습 이벤트 처리
   */
  learn(event: LearningEvent): void {
    switch (event.type) {
      case 'symbol_selection':
        this.learnSymbolSelection(event);
        break;
      case 'phrase_completion':
        this.learnPhraseCompletion(event);
        break;
      case 'navigation':
        this.learnNavigation(event);
        break;
      case 'context_change':
        this.learnContextChange(event);
        break;
    }

    // 주기적 감쇠 적용
    this.applyPeriodicDecay();
  }

  /**
   * 심볼 선택 학습
   */
  private learnSymbolSelection(event: LearningEvent): void {
    const symbolId = event.data.symbolId as string;
    const responseTime = event.data.responseTime as number | undefined;
    const context = event.context;

    // 1. 시간대별 빈도 업데이트
    this.updateTemporalFrequency(symbolId, context.time);

    // 2. 요일별 빈도 업데이트
    this.updateDayOfWeekFrequency(symbolId, context.time);

    // 3. 컨텍스트별 빈도 업데이트
    this.updateContextualFrequency(symbolId, context);

    // 4. 시퀀스 학습
    this.updateSequencePatterns(symbolId);

    // 5. 응답 시간 기록
    this.updateResponseTime(symbolId, responseTime);

    // 현재 세션에 심볼 추가
    this.currentSessionSymbols.push(symbolId);
    this.recentSymbols.push(symbolId);

    // 최근 심볼 목록 크기 제한
    if (this.recentSymbols.length > this.config.sequenceMaxLength * 2) {
      this.recentSymbols = this.recentSymbols.slice(-this.config.sequenceMaxLength);
    }
  }

  /**
   * 문장 완성 학습
   */
  private learnPhraseCompletion(event: LearningEvent): void {
    const symbols = event.data.symbols as string[];
    const context = event.context;

    if (symbols.length < 2) return;

    // 기존 문장 찾기
    const existingPhrase = this.patterns.sequences.commonPhrases.find(
      (p) => this.arraysEqual(p.symbols, symbols)
    );

    if (existingPhrase) {
      existingPhrase.frequency++;
      existingPhrase.lastUsed = event.timestamp;
      existingPhrase.context = context.recentActivity;
    } else {
      // 새 문장 추가
      this.patterns.sequences.commonPhrases.push({
        id: `phrase_${Date.now()}`,
        symbols,
        frequency: 1,
        lastUsed: event.timestamp,
        context: context.recentActivity,
      });
    }

    // 문장 목록 정렬 및 제한
    this.pruneCommonPhrases();
  }

  /**
   * 네비게이션 학습
   */
  private learnNavigation(event: LearningEvent): void {
    // 페이지 전환 패턴 학습
    const fromPage = event.data.fromPage as string | undefined;
    const toPage = event.data.toPage as string | undefined;

    if (fromPage && toPage) {
      // 향후 페이지 전환 예측에 활용
      // 현재는 로깅만 수행
    }
  }

  /**
   * 컨텍스트 변경 학습
   */
  private learnContextChange(event: LearningEvent): void {
    const newContext = event.data.newContext as string | undefined;

    if (newContext) {
      // 세션 심볼을 컨텍스트에 연결
      if (!this.patterns.temporal.contextual.has(newContext)) {
        this.patterns.temporal.contextual.set(newContext, []);
      }

      const contextSymbols = this.patterns.temporal.contextual.get(newContext)!;
      for (const symbolId of this.currentSessionSymbols) {
        this.incrementSymbolFrequency(contextSymbols, symbolId);
      }
    }

    // 세션 리셋
    this.currentSessionSymbols = [];
  }

  // ============================================================================
  // Frequency Update Methods
  // ============================================================================

  private updateTemporalFrequency(symbolId: string, time: Date): void {
    const hour = time.getHours();

    if (!this.patterns.temporal.hourlyFrequency.has(hour)) {
      this.patterns.temporal.hourlyFrequency.set(hour, []);
    }

    const hourlySymbols = this.patterns.temporal.hourlyFrequency.get(hour)!;
    this.incrementSymbolFrequency(hourlySymbols, symbolId);
  }

  private updateDayOfWeekFrequency(symbolId: string, time: Date): void {
    const day = time.getDay();

    if (!this.patterns.temporal.dayOfWeek.has(day)) {
      this.patterns.temporal.dayOfWeek.set(day, []);
    }

    const daySymbols = this.patterns.temporal.dayOfWeek.get(day)!;
    this.incrementSymbolFrequency(daySymbols, symbolId);
  }

  private updateContextualFrequency(symbolId: string, context: Context): void {
    // 위치 기반
    if (context.location) {
      if (!this.patterns.contextual.locationBased.has(context.location)) {
        this.patterns.contextual.locationBased.set(context.location, []);
      }
      const locationSymbols = this.patterns.contextual.locationBased.get(context.location)!;
      this.incrementSymbolFrequency(locationSymbols, symbolId);
    }

    // 대화 상대 기반
    if (context.conversationPartner) {
      if (!this.patterns.contextual.personBased.has(context.conversationPartner)) {
        this.patterns.contextual.personBased.set(context.conversationPartner, []);
      }
      const personSymbols = this.patterns.contextual.personBased.get(context.conversationPartner)!;
      this.incrementSymbolFrequency(personSymbols, symbolId);
    }

    // 활동 기반
    if (context.recentActivity) {
      if (!this.patterns.contextual.activityBased.has(context.recentActivity)) {
        this.patterns.contextual.activityBased.set(context.recentActivity, []);
      }
      const activitySymbols = this.patterns.contextual.activityBased.get(context.recentActivity)!;
      this.incrementSymbolFrequency(activitySymbols, symbolId);
    }
  }

  private incrementSymbolFrequency(symbols: SymbolFrequency[], symbolId: string): void {
    const existing = symbols.find((s) => s.symbolId === symbolId);

    if (existing) {
      existing.count++;
      existing.lastUsed = Date.now();
    } else {
      symbols.push({
        symbolId,
        count: 1,
        lastUsed: Date.now(),
        avgResponseTime: 0,
      });
    }

    // 빈도순 정렬
    symbols.sort((a, b) => b.count - a.count);
  }

  private updateResponseTime(symbolId: string, responseTime?: number): void {
    if (responseTime === undefined) return;

    // 모든 시간대의 해당 심볼 응답 시간 업데이트
    this.patterns.temporal.hourlyFrequency.forEach((symbols) => {
      const symbol = symbols.find((s) => s.symbolId === symbolId);
      if (symbol) {
        // 이동 평균
        symbol.avgResponseTime =
          symbol.avgResponseTime === 0
            ? responseTime
            : symbol.avgResponseTime * 0.8 + responseTime * 0.2;
      }
    });
  }

  // ============================================================================
  // Sequence Pattern Methods
  // ============================================================================

  private updateSequencePatterns(symbolId: string): void {
    if (this.recentSymbols.length === 0) return;

    // 최근 심볼들과의 연쇄 패턴 업데이트
    const recentSequence = [...this.recentSymbols, symbolId];

    // 다양한 길이의 시퀀스 학습
    for (let len = 2; len <= Math.min(this.config.sequenceMaxLength, recentSequence.length); len++) {
      const sequence = recentSequence.slice(-len);
      this.updateSymbolChain(sequence);
    }
  }

  private updateSymbolChain(sequence: string[]): void {
    const existingChain = this.patterns.sequences.symbolChains.find((c) =>
      this.arraysEqual(c.symbols, sequence)
    );

    if (existingChain) {
      existingChain.frequency++;
      // 확률 재계산
      existingChain.probability = this.calculateChainProbability(existingChain);
    } else {
      const newChain: SymbolChain = {
        symbols: sequence,
        frequency: 1,
        avgInterval: 0,
        probability: 0.1, // 초기 확률
      };
      this.patterns.sequences.symbolChains.push(newChain);
    }

    // 시퀀스 목록 정리
    this.pruneSymbolChains();
  }

  private calculateChainProbability(chain: SymbolChain): number {
    // 시작 심볼로 시작하는 모든 체인의 총 빈도 계산
    const startSymbol = chain.symbols[0];
    let totalFrequency = 0;

    for (const c of this.patterns.sequences.symbolChains) {
      if (c.symbols[0] === startSymbol && c.symbols.length === chain.symbols.length) {
        totalFrequency += c.frequency;
      }
    }

    return totalFrequency > 0 ? chain.frequency / totalFrequency : 0;
  }

  // ============================================================================
  // Decay and Pruning Methods
  // ============================================================================

  private applyPeriodicDecay(): void {
    const now = Date.now();
    const dayMs = 24 * 60 * 60 * 1000;

    // 하루에 한 번 감쇠 적용
    if (now - this.lastDecayTime < dayMs) return;

    this.lastDecayTime = now;

    // 시간대별 빈도 감쇠
    this.patterns.temporal.hourlyFrequency.forEach((symbols) => {
      this.applyDecayToSymbols(symbols);
    });

    // 요일별 빈도 감쇠
    this.patterns.temporal.dayOfWeek.forEach((symbols) => {
      this.applyDecayToSymbols(symbols);
    });

    // 컨텍스트별 빈도 감쇠
    this.patterns.contextual.locationBased.forEach((symbols) => {
      this.applyDecayToSymbols(symbols);
    });
    this.patterns.contextual.personBased.forEach((symbols) => {
      this.applyDecayToSymbols(symbols);
    });
    this.patterns.contextual.activityBased.forEach((symbols) => {
      this.applyDecayToSymbols(symbols);
    });

    // 시퀀스 패턴 감쇠
    this.patterns.sequences.symbolChains.forEach((chain) => {
      chain.frequency = Math.floor(chain.frequency * this.config.decayFactor);
    });

    // 문장 패턴 감쇠
    this.patterns.sequences.commonPhrases.forEach((phrase) => {
      phrase.frequency = Math.floor(phrase.frequency * this.config.decayFactor);
    });

    // 임계값 미만 제거
    this.pruneAllPatterns();
  }

  private applyDecayToSymbols(symbols: SymbolFrequency[]): void {
    symbols.forEach((s) => {
      s.count = Math.floor(s.count * this.config.decayFactor);
    });
  }

  private pruneAllPatterns(): void {
    // 빈도 낮은 심볼 제거
    this.patterns.temporal.hourlyFrequency.forEach((symbols, hour) => {
      const filtered = symbols.filter((s) => s.count >= this.config.minFrequencyThreshold);
      this.patterns.temporal.hourlyFrequency.set(hour, filtered);
    });

    this.pruneSymbolChains();
    this.pruneCommonPhrases();
  }

  private pruneSymbolChains(): void {
    // 빈도 낮은 체인 제거
    this.patterns.sequences.symbolChains = this.patterns.sequences.symbolChains
      .filter((c) => c.frequency >= this.config.minFrequencyThreshold)
      .sort((a, b) => b.frequency - a.frequency)
      .slice(0, 100); // 최대 100개 유지
  }

  private pruneCommonPhrases(): void {
    // 빈도 낮은 문장 제거
    this.patterns.sequences.commonPhrases = this.patterns.sequences.commonPhrases
      .filter((p) => p.frequency >= this.config.minFrequencyThreshold)
      .sort((a, b) => b.frequency - a.frequency)
      .slice(0, 50); // 최대 50개 유지
  }

  // ============================================================================
  // Utility Methods
  // ============================================================================

  private createEmptyPatterns(): UsagePattern {
    return {
      temporal: {
        hourlyFrequency: new Map(),
        dayOfWeek: new Map(),
        contextual: new Map(),
      },
      sequences: {
        commonPhrases: [],
        symbolChains: [],
        conversationPatterns: [],
      },
      contextual: {
        locationBased: new Map(),
        personBased: new Map(),
        activityBased: new Map(),
      },
    };
  }

  private arraysEqual(a: string[], b: string[]): boolean {
    if (a.length !== b.length) return false;
    return a.every((val, idx) => val === b[idx]);
  }

  // ============================================================================
  // Public API
  // ============================================================================

  /**
   * 현재 사용 패턴 반환
   */
  getUsagePatterns(): UsagePattern {
    return this.patterns;
  }

  /**
   * 패턴 가져오기
   */
  importPatterns(patterns: UsagePattern): void {
    this.patterns = patterns;
  }

  /**
   * 패턴 내보내기 (직렬화 가능 형태)
   */
  exportPatterns(): object {
    return {
      temporal: {
        hourlyFrequency: Object.fromEntries(this.patterns.temporal.hourlyFrequency),
        dayOfWeek: Object.fromEntries(this.patterns.temporal.dayOfWeek),
        contextual: Object.fromEntries(this.patterns.temporal.contextual),
      },
      sequences: this.patterns.sequences,
      contextual: {
        locationBased: Object.fromEntries(this.patterns.contextual.locationBased),
        personBased: Object.fromEntries(this.patterns.contextual.personBased),
        activityBased: Object.fromEntries(this.patterns.contextual.activityBased),
      },
    };
  }

  /**
   * 직렬화된 패턴 불러오기
   */
  importSerializedPatterns(data: ReturnType<typeof this.exportPatterns>): void {
    const imported = data as {
      temporal: {
        hourlyFrequency: Record<string, SymbolFrequency[]>;
        dayOfWeek: Record<string, SymbolFrequency[]>;
        contextual: Record<string, SymbolFrequency[]>;
      };
      sequences: UsagePattern['sequences'];
      contextual: {
        locationBased: Record<string, SymbolFrequency[]>;
        personBased: Record<string, SymbolFrequency[]>;
        activityBased: Record<string, SymbolFrequency[]>;
      };
    };

    this.patterns = {
      temporal: {
        hourlyFrequency: new Map(
          Object.entries(imported.temporal.hourlyFrequency).map(([k, v]) => [Number(k), v])
        ),
        dayOfWeek: new Map(
          Object.entries(imported.temporal.dayOfWeek).map(([k, v]) => [Number(k), v])
        ),
        contextual: new Map(Object.entries(imported.temporal.contextual)),
      },
      sequences: imported.sequences,
      contextual: {
        locationBased: new Map(Object.entries(imported.contextual.locationBased)),
        personBased: new Map(Object.entries(imported.contextual.personBased)),
        activityBased: new Map(Object.entries(imported.contextual.activityBased)),
      },
    };
  }

  /**
   * 설정 업데이트
   */
  updateConfig(config: Partial<PatternLearnerConfig>): void {
    this.config = { ...this.config, ...config };
  }

  /**
   * 패턴 초기화
   */
  reset(): void {
    this.patterns = this.createEmptyPatterns();
    this.recentSymbols = [];
    this.currentSessionSymbols = [];
    this.lastDecayTime = Date.now();
  }

  /**
   * 세션 시작
   */
  startSession(): void {
    this.currentSessionSymbols = [];
  }

  /**
   * 세션 종료
   */
  endSession(): void {
    // 세션 데이터를 대화 패턴으로 저장
    if (this.currentSessionSymbols.length > 2) {
      const pattern: ConversationPattern = {
        id: `conv_${Date.now()}`,
        initiator: 'user',
        exchangeSequence: this.currentSessionSymbols,
        frequency: 1,
      };

      // 유사 패턴 찾기
      const existingPattern = this.patterns.sequences.conversationPatterns.find(
        (p) => this.arraysEqual(p.exchangeSequence, this.currentSessionSymbols)
      );

      if (existingPattern) {
        existingPattern.frequency++;
      } else {
        this.patterns.sequences.conversationPatterns.push(pattern);
      }
    }

    this.currentSessionSymbols = [];
  }
}

export default PatternLearner;
