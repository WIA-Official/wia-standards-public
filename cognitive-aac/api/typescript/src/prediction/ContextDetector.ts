/**
 * WIA Cognitive AAC - Context Detector
 * 상황 인식 및 컨텍스트 감지 모듈
 *
 * 홍익인간 (弘益人間) - 널리 인간을 이롭게 하라
 */

import { Context, ContextSignal } from './types';

// ============================================================================
// Types
// ============================================================================

export interface ContextDetectorConfig {
  enableTimeDetection: boolean;
  enableLocationDetection: boolean;
  enableActivityDetection: boolean;
  activityKeywords: Map<string, string[]>;
  mealTimes: { breakfast: number[]; lunch: number[]; dinner: number[] };
  sleepTime: { start: number; end: number };
}

export interface DetectedContext extends Context {
  signals: ContextSignal[];
  confidence: number;
  inferred: {
    activity?: string;
    mood?: string;
    urgency?: 'low' | 'medium' | 'high';
  };
}

// ============================================================================
// Default Configuration
// ============================================================================

const DEFAULT_CONFIG: ContextDetectorConfig = {
  enableTimeDetection: true,
  enableLocationDetection: true,
  enableActivityDetection: true,
  activityKeywords: new Map([
    ['meal', ['eat', 'food', 'hungry', 'breakfast', 'lunch', 'dinner', '밥', '먹다', '배고파']],
    ['rest', ['tired', 'sleep', 'rest', 'nap', '피곤', '자다', '쉬다']],
    ['play', ['play', 'game', 'fun', 'toy', '놀다', '게임', '장난감']],
    ['bathroom', ['toilet', 'bathroom', 'potty', '화장실', '쉬']],
    ['pain', ['hurt', 'pain', 'ache', 'sick', '아파', '아프다', '아픔']],
    ['social', ['hello', 'bye', 'friend', 'talk', '안녕', '친구', '이야기']],
  ]),
  mealTimes: {
    breakfast: [7, 8, 9],
    lunch: [11, 12, 13],
    dinner: [17, 18, 19],
  },
  sleepTime: {
    start: 21,
    end: 7,
  },
};

// ============================================================================
// Context Detector Class
// ============================================================================

export class ContextDetector {
  private config: ContextDetectorConfig;
  private signalHistory: ContextSignal[] = [];
  private lastContext: DetectedContext | null = null;

  constructor(config?: Partial<ContextDetectorConfig>) {
    this.config = { ...DEFAULT_CONFIG, ...config };
  }

  // ============================================================================
  // Main Detection Methods
  // ============================================================================

  /**
   * 컨텍스트 분석
   */
  analyze(context: Context): DetectedContext {
    const signals: ContextSignal[] = [];
    const now = Date.now();

    // 1. 시간 기반 신호
    if (this.config.enableTimeDetection) {
      const timeSignals = this.detectTimeSignals(context.time);
      signals.push(...timeSignals);
    }

    // 2. 위치 기반 신호
    if (this.config.enableLocationDetection && context.location) {
      const locationSignals = this.detectLocationSignals(context.location);
      signals.push(...locationSignals);
    }

    // 3. 활동 추론
    let inferredActivity: string | undefined;
    if (this.config.enableActivityDetection) {
      inferredActivity = this.inferActivity(context, signals);
    }

    // 4. 기분 추론
    const inferredMood = this.inferMood(context, signals);

    // 5. 긴급도 추론
    const urgency = this.inferUrgency(context, signals);

    // 신호 히스토리 업데이트
    this.signalHistory.push(...signals);
    if (this.signalHistory.length > 100) {
      this.signalHistory = this.signalHistory.slice(-50);
    }

    const detectedContext: DetectedContext = {
      ...context,
      timeOfDay: this.getTimeOfDay(context.time),
      dayType: this.getDayType(context.time),
      signals,
      confidence: this.calculateConfidence(signals),
      inferred: {
        activity: inferredActivity ?? context.recentActivity,
        mood: inferredMood ?? context.mood,
        urgency,
      },
    };

    this.lastContext = detectedContext;
    return detectedContext;
  }

  /**
   * 환경 신호 추가
   */
  addEnvironmentalSignal(type: ContextSignal['type'], value: string, confidence: number): void {
    const signal: ContextSignal = {
      type,
      value,
      confidence,
      timestamp: Date.now(),
    };
    this.signalHistory.push(signal);
  }

  // ============================================================================
  // Time-based Detection
  // ============================================================================

  private detectTimeSignals(time: Date): ContextSignal[] {
    const signals: ContextSignal[] = [];
    const hour = time.getHours();
    const now = Date.now();

    // 시간대 감지
    const timeOfDay = this.getTimeOfDay(time);
    signals.push({
      type: 'time',
      value: timeOfDay,
      confidence: 1.0,
      timestamp: now,
    });

    // 식사 시간 감지
    const mealTime = this.detectMealTime(hour);
    if (mealTime) {
      signals.push({
        type: 'activity',
        value: mealTime,
        confidence: 0.8,
        timestamp: now,
      });
    }

    // 수면 시간 감지
    if (this.isSleepTime(hour)) {
      signals.push({
        type: 'activity',
        value: 'sleep',
        confidence: 0.7,
        timestamp: now,
      });
    }

    // 주말/평일 감지
    const dayType = this.getDayType(time);
    signals.push({
      type: 'time',
      value: dayType,
      confidence: 1.0,
      timestamp: now,
    });

    return signals;
  }

  private getTimeOfDay(time: Date): 'morning' | 'afternoon' | 'evening' | 'night' {
    const hour = time.getHours();
    if (hour >= 5 && hour < 12) return 'morning';
    if (hour >= 12 && hour < 17) return 'afternoon';
    if (hour >= 17 && hour < 21) return 'evening';
    return 'night';
  }

  private getDayType(time: Date): 'weekday' | 'weekend' {
    const day = time.getDay();
    return day === 0 || day === 6 ? 'weekend' : 'weekday';
  }

  private detectMealTime(hour: number): string | null {
    if (this.config.mealTimes.breakfast.includes(hour)) return 'breakfast';
    if (this.config.mealTimes.lunch.includes(hour)) return 'lunch';
    if (this.config.mealTimes.dinner.includes(hour)) return 'dinner';
    return null;
  }

  private isSleepTime(hour: number): boolean {
    const { start, end } = this.config.sleepTime;
    if (start > end) {
      // 자정을 넘는 경우 (예: 21-7)
      return hour >= start || hour < end;
    }
    return hour >= start && hour < end;
  }

  // ============================================================================
  // Location-based Detection
  // ============================================================================

  private detectLocationSignals(location: string): ContextSignal[] {
    const signals: ContextSignal[] = [];
    const now = Date.now();
    const normalizedLocation = location.toLowerCase();

    // 위치별 활동 추론
    const locationActivities: Record<string, string> = {
      kitchen: 'meal',
      bathroom: 'bathroom',
      bedroom: 'rest',
      livingroom: 'social',
      playroom: 'play',
      school: 'learning',
      hospital: 'medical',
      park: 'outdoor',
      // 한국어 위치
      부엌: 'meal',
      화장실: 'bathroom',
      침실: 'rest',
      거실: 'social',
      학교: 'learning',
      병원: 'medical',
      공원: 'outdoor',
    };

    for (const [loc, activity] of Object.entries(locationActivities)) {
      if (normalizedLocation.includes(loc)) {
        signals.push({
          type: 'location',
          value: location,
          confidence: 0.9,
          timestamp: now,
        });
        signals.push({
          type: 'activity',
          value: activity,
          confidence: 0.7,
          timestamp: now,
        });
        break;
      }
    }

    return signals;
  }

  // ============================================================================
  // Activity Inference
  // ============================================================================

  private inferActivity(context: Context, signals: ContextSignal[]): string | undefined {
    // 직접 제공된 활동이 있으면 사용
    if (context.recentActivity) return context.recentActivity;

    // 신호에서 활동 추론
    const activitySignals = signals.filter((s) => s.type === 'activity');
    if (activitySignals.length > 0) {
      // 가장 신뢰도 높은 활동 선택
      activitySignals.sort((a, b) => b.confidence - a.confidence);
      return activitySignals[0].value;
    }

    // 환경 신호에서 키워드 기반 활동 추론
    if (context.environmentalCues) {
      for (const cue of context.environmentalCues) {
        const activity = this.matchActivityKeyword(cue);
        if (activity) return activity;
      }
    }

    return undefined;
  }

  private matchActivityKeyword(text: string): string | undefined {
    const normalizedText = text.toLowerCase();

    for (const [activity, keywords] of this.config.activityKeywords) {
      for (const keyword of keywords) {
        if (normalizedText.includes(keyword.toLowerCase())) {
          return activity;
        }
      }
    }

    return undefined;
  }

  // ============================================================================
  // Mood Inference
  // ============================================================================

  private inferMood(context: Context, signals: ContextSignal[]): string | undefined {
    if (context.mood) return context.mood;

    // 활동에서 기분 추론
    const activityMoods: Record<string, string> = {
      play: 'happy',
      rest: 'tired',
      pain: 'uncomfortable',
      social: 'engaged',
      meal: 'neutral',
    };

    const activitySignals = signals.filter((s) => s.type === 'activity');
    for (const signal of activitySignals) {
      if (signal.value in activityMoods) {
        return activityMoods[signal.value];
      }
    }

    return undefined;
  }

  // ============================================================================
  // Urgency Inference
  // ============================================================================

  private inferUrgency(
    context: Context,
    signals: ContextSignal[]
  ): 'low' | 'medium' | 'high' {
    // 긴급 활동 확인
    const urgentActivities = ['pain', 'bathroom', 'emergency'];
    const activitySignals = signals.filter((s) => s.type === 'activity');

    for (const signal of activitySignals) {
      if (urgentActivities.includes(signal.value)) {
        return 'high';
      }
    }

    // 기본 활동 긴급도
    const mediumUrgencyActivities = ['meal', 'rest'];
    for (const signal of activitySignals) {
      if (mediumUrgencyActivities.includes(signal.value)) {
        return 'medium';
      }
    }

    return 'low';
  }

  // ============================================================================
  // Confidence Calculation
  // ============================================================================

  private calculateConfidence(signals: ContextSignal[]): number {
    if (signals.length === 0) return 0;

    // 평균 신뢰도 계산
    const avgConfidence =
      signals.reduce((sum, s) => sum + s.confidence, 0) / signals.length;

    // 신호 다양성 보너스
    const types = new Set(signals.map((s) => s.type));
    const diversityBonus = Math.min(types.size * 0.05, 0.2);

    return Math.min(avgConfidence + diversityBonus, 1.0);
  }

  // ============================================================================
  // Context Comparison
  // ============================================================================

  /**
   * 컨텍스트 변경 감지
   */
  hasContextChanged(newContext: Context): boolean {
    if (!this.lastContext) return true;

    // 주요 컨텍스트 필드 비교
    if (newContext.location !== this.lastContext.location) return true;
    if (newContext.conversationPartner !== this.lastContext.conversationPartner) return true;
    if (newContext.recentActivity !== this.lastContext.recentActivity) return true;

    // 시간대 변경 확인
    const newTimeOfDay = this.getTimeOfDay(newContext.time);
    if (newTimeOfDay !== this.lastContext.timeOfDay) return true;

    return false;
  }

  /**
   * 컨텍스트 유사도 계산
   */
  calculateSimilarity(context1: Context, context2: Context): number {
    let matches = 0;
    let total = 0;

    // 시간대 비교
    total++;
    if (this.getTimeOfDay(context1.time) === this.getTimeOfDay(context2.time)) {
      matches++;
    }

    // 요일 유형 비교
    total++;
    if (this.getDayType(context1.time) === this.getDayType(context2.time)) {
      matches++;
    }

    // 위치 비교
    if (context1.location || context2.location) {
      total++;
      if (context1.location === context2.location) {
        matches++;
      }
    }

    // 활동 비교
    if (context1.recentActivity || context2.recentActivity) {
      total++;
      if (context1.recentActivity === context2.recentActivity) {
        matches++;
      }
    }

    // 대화 상대 비교
    if (context1.conversationPartner || context2.conversationPartner) {
      total++;
      if (context1.conversationPartner === context2.conversationPartner) {
        matches++;
      }
    }

    return total > 0 ? matches / total : 0;
  }

  // ============================================================================
  // Public API
  // ============================================================================

  /**
   * 마지막 감지된 컨텍스트 반환
   */
  getLastContext(): DetectedContext | null {
    return this.lastContext;
  }

  /**
   * 신호 히스토리 반환
   */
  getSignalHistory(): ContextSignal[] {
    return [...this.signalHistory];
  }

  /**
   * 설정 업데이트
   */
  updateConfig(config: Partial<ContextDetectorConfig>): void {
    this.config = { ...this.config, ...config };
  }

  /**
   * 활동 키워드 추가
   */
  addActivityKeywords(activity: string, keywords: string[]): void {
    const existing = this.config.activityKeywords.get(activity) ?? [];
    this.config.activityKeywords.set(activity, [...existing, ...keywords]);
  }

  /**
   * 히스토리 초기화
   */
  clearHistory(): void {
    this.signalHistory = [];
    this.lastContext = null;
  }

  /**
   * 현재 시간 컨텍스트 생성
   */
  createCurrentContext(overrides?: Partial<Context>): Context {
    return {
      time: new Date(),
      timeOfDay: this.getTimeOfDay(new Date()),
      dayType: this.getDayType(new Date()),
      ...overrides,
    };
  }
}

export default ContextDetector;
