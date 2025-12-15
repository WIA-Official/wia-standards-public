/**
 * WIA Cognitive AAC - Caregiver Service
 * 케어기버 대시보드 서비스
 *
 * 홍익인간 (弘益人間) - 널리 인간을 이롭게 하라
 */

import { v4 as uuidv4 } from 'uuid';
import {
  Client,
  Activity,
  Message,
  Mood,
  Alert,
  AlertThreshold,
  AlertType,
  AlertSeverity,
  DailySummary,
  WeeklyProgress,
  CommunicationEntry,
  GoalProgress,
  MoodLevel,
  MoodTrend,
  SymbolUsage,
  LiveSession,
  TimeRange,
} from '../types';

// ============================================================================
// Types
// ============================================================================

export interface LiveMonitoringData {
  currentActivity: Activity | null;
  lastCommunication: Message | null;
  moodIndicator: Mood | null;
  alertStatus: Alert[];
  sessionDuration: number;
  isActive: boolean;
}

export interface CaregiverDashboardData {
  client: Client;
  liveMonitoring: LiveMonitoringData;
  todaySummary: DailySummary | null;
  recentAlerts: Alert[];
  goalProgress: GoalProgress[];
}

export interface NotificationSettings {
  emergencyContacts: EmergencyContact[];
  alertThresholds: AlertThreshold[];
  quietHours: TimeRange | null;
  enablePush: boolean;
  enableSms: boolean;
  enableEmail: boolean;
}

export interface EmergencyContact {
  id: string;
  name: string;
  phone: string;
  email?: string;
  relationship: string;
  notifyOn: AlertSeverity[];
}

// ============================================================================
// Caregiver Service
// ============================================================================

export class CaregiverService {
  // In-memory stores (production: database)
  private activities: Map<string, Activity[]> = new Map();
  private messages: Map<string, Message[]> = new Map();
  private moods: Map<string, Mood[]> = new Map();
  private alerts: Map<string, Alert[]> = new Map();
  private liveSessions: Map<string, LiveSession> = new Map();
  private alertThresholds: Map<string, AlertThreshold[]> = new Map();

  // ============================================================================
  // Live Monitoring
  // ============================================================================

  /**
   * 실시간 모니터링 데이터 조회
   */
  getLiveMonitoring(clientId: string): LiveMonitoringData {
    const session = this.liveSessions.get(clientId);
    const clientActivities = this.activities.get(clientId) ?? [];
    const clientMessages = this.messages.get(clientId) ?? [];
    const clientMoods = this.moods.get(clientId) ?? [];
    const clientAlerts = this.alerts.get(clientId) ?? [];

    const currentActivity = clientActivities[clientActivities.length - 1] ?? null;
    const lastCommunication = clientMessages[clientMessages.length - 1] ?? null;
    const currentMood = clientMoods[clientMoods.length - 1] ?? null;
    const activeAlerts = clientAlerts.filter((a) => !a.acknowledged);

    const sessionDuration = session
      ? Date.now() - session.startedAt.getTime()
      : 0;

    return {
      currentActivity,
      lastCommunication,
      moodIndicator: currentMood,
      alertStatus: activeAlerts,
      sessionDuration,
      isActive: session !== undefined,
    };
  }

  /**
   * 세션 시작 기록
   */
  startSession(clientId: string): LiveSession {
    const session: LiveSession = {
      clientId,
      startedAt: new Date(),
      lastActivity: new Date(),
      activeSymbols: [],
      alerts: [],
    };
    this.liveSessions.set(clientId, session);
    return session;
  }

  /**
   * 세션 종료 기록
   */
  endSession(clientId: string): void {
    this.liveSessions.delete(clientId);
  }

  /**
   * 활동 기록
   */
  recordActivity(clientId: string, activity: Omit<Activity, 'id'>): Activity {
    const newActivity: Activity = {
      ...activity,
      id: uuidv4(),
    };

    const clientActivities = this.activities.get(clientId) ?? [];
    clientActivities.push(newActivity);
    this.activities.set(clientId, clientActivities);

    // 세션 업데이트
    const session = this.liveSessions.get(clientId);
    if (session) {
      session.lastActivity = new Date();
      if (activity.data.symbolId) {
        session.activeSymbols.push(activity.data.symbolId);
      }
    }

    // 알림 체크
    this.checkAlertThresholds(clientId, newActivity);

    return newActivity;
  }

  /**
   * 메시지 기록
   */
  recordMessage(clientId: string, message: Omit<Message, 'id'>): Message {
    const newMessage: Message = {
      ...message,
      id: uuidv4(),
    };

    const clientMessages = this.messages.get(clientId) ?? [];
    clientMessages.push(newMessage);
    this.messages.set(clientId, clientMessages);

    return newMessage;
  }

  /**
   * 기분 업데이트
   */
  updateMood(clientId: string, mood: Mood): void {
    const clientMoods = this.moods.get(clientId) ?? [];
    clientMoods.push(mood);
    this.moods.set(clientId, clientMoods);

    // 기분 급변 시 알림
    if (clientMoods.length >= 2) {
      const prevMood = clientMoods[clientMoods.length - 2];
      if (this.isMoodChangeSignificant(prevMood.level, mood.level)) {
        this.createAlert(clientId, {
          severity: 'warning',
          type: 'distress',
          message: `기분 변화 감지: ${this.getMoodLabel(prevMood.level)} → ${this.getMoodLabel(mood.level)}`,
        });
      }
    }
  }

  // ============================================================================
  // Reports
  // ============================================================================

  /**
   * 일간 요약 생성
   */
  getDailySummary(clientId: string, date: Date = new Date()): DailySummary {
    const dayStart = new Date(date);
    dayStart.setHours(0, 0, 0, 0);
    const dayEnd = new Date(date);
    dayEnd.setHours(23, 59, 59, 999);

    const clientActivities = this.activities.get(clientId) ?? [];
    const clientMessages = this.messages.get(clientId) ?? [];
    const clientMoods = this.moods.get(clientId) ?? [];

    // 해당 날짜 데이터 필터링
    const dayActivities = clientActivities.filter(
      (a) => a.timestamp >= dayStart && a.timestamp <= dayEnd
    );
    const dayMessages = clientMessages.filter(
      (m) => m.timestamp >= dayStart && m.timestamp <= dayEnd
    );
    const dayMoods = clientMoods.filter(
      (m) => m.timestamp >= dayStart && m.timestamp <= dayEnd
    );

    // 통계 계산
    const symbolCounts = new Map<string, number>();
    let totalResponseTime = 0;
    let responseTimeCount = 0;

    for (const activity of dayActivities) {
      if (activity.data.symbolId) {
        const count = symbolCounts.get(activity.data.symbolId) ?? 0;
        symbolCounts.set(activity.data.symbolId, count + 1);
      }
      if (activity.data.responseTime) {
        totalResponseTime += activity.data.responseTime;
        responseTimeCount++;
      }
    }

    const topSymbols: SymbolUsage[] = Array.from(symbolCounts.entries())
      .map(([symbolId, count]) => ({
        symbolId,
        label: symbolId, // 실제로는 심볼 DB에서 조회
        count,
        percentage: (count / dayActivities.length) * 100,
      }))
      .sort((a, b) => b.count - a.count)
      .slice(0, 10);

    const moodTrend = this.calculateMoodTrend(dayMoods);

    // 하이라이트 및 우려사항 생성
    const highlights: string[] = [];
    const concerns: string[] = [];

    if (dayMessages.length > 10) {
      highlights.push(`활발한 의사소통: ${dayMessages.length}개의 메시지`);
    }
    if (symbolCounts.size > 20) {
      highlights.push(`다양한 어휘 사용: ${symbolCounts.size}개의 심볼`);
    }

    if (dayMessages.length < 5) {
      concerns.push('의사소통 횟수가 평소보다 적습니다');
    }
    if (moodTrend.stability < 0.5) {
      concerns.push('기분 변동이 크게 관찰되었습니다');
    }

    return {
      date,
      clientId,
      totalCommunications: dayMessages.length,
      uniqueSymbolsUsed: symbolCounts.size,
      avgResponseTime: responseTimeCount > 0 ? totalResponseTime / responseTimeCount : 0,
      moodTrend,
      peakUsageTime: this.findPeakUsageTime(dayActivities),
      topSymbols,
      highlights,
      concerns,
    };
  }

  /**
   * 주간 진행상황 생성
   */
  getWeeklyProgress(clientId: string, weekStart: Date): WeeklyProgress {
    const dailySummaries: DailySummary[] = [];
    const currentDate = new Date(weekStart);

    for (let i = 0; i < 7; i++) {
      const summary = this.getDailySummary(clientId, currentDate);
      dailySummaries.push(summary);
      currentDate.setDate(currentDate.getDate() + 1);
    }

    const weekEnd = new Date(weekStart);
    weekEnd.setDate(weekEnd.getDate() + 6);

    const totalCommunications = dailySummaries.reduce(
      (sum, s) => sum + s.totalCommunications,
      0
    );

    const allSymbols = new Set<string>();
    dailySummaries.forEach((s) => {
      s.topSymbols.forEach((ts) => allSymbols.add(ts.symbolId));
    });

    // 이전 주와 비교 (간략화)
    const prevWeekCommunications = totalCommunications * 0.9; // 실제로는 이전 주 데이터 조회
    const vocabularyGrowth = allSymbols.size - Math.floor(allSymbols.size * 0.9);

    return {
      weekStart,
      weekEnd,
      clientId,
      dailySummaries,
      overallStats: {
        totalCommunications,
        avgDailyCommunications: totalCommunications / 7,
        vocabularyGrowth,
        responseTimeImprovement: 5, // %
      },
      goalProgress: [], // 실제로는 GoalService에서 조회
      recommendations: this.generateWeeklyRecommendations(dailySummaries),
    };
  }

  /**
   * 의사소통 로그 조회
   */
  getCommunicationLog(
    clientId: string,
    startDate: Date,
    endDate: Date
  ): CommunicationEntry[] {
    const clientMessages = this.messages.get(clientId) ?? [];

    return clientMessages
      .filter((m) => m.timestamp >= startDate && m.timestamp <= endDate)
      .map((m) => ({
        id: m.id,
        timestamp: m.timestamp,
        message: m.content,
        symbols: m.symbols,
        context: m.context,
      }));
  }

  // ============================================================================
  // Alerts & Notifications
  // ============================================================================

  /**
   * 알림 생성
   */
  createAlert(
    clientId: string,
    alertData: { severity: AlertSeverity; type: AlertType; message: string }
  ): Alert {
    const alert: Alert = {
      id: uuidv4(),
      clientId,
      severity: alertData.severity,
      type: alertData.type,
      message: alertData.message,
      timestamp: new Date(),
      acknowledged: false,
    };

    const clientAlerts = this.alerts.get(clientId) ?? [];
    clientAlerts.push(alert);
    this.alerts.set(clientId, clientAlerts);

    // 세션에도 추가
    const session = this.liveSessions.get(clientId);
    if (session) {
      session.alerts.push(alert);
    }

    return alert;
  }

  /**
   * 알림 확인 처리
   */
  acknowledgeAlert(alertId: string, userId: string): boolean {
    for (const [, clientAlerts] of this.alerts) {
      const alert = clientAlerts.find((a) => a.id === alertId);
      if (alert) {
        alert.acknowledged = true;
        alert.acknowledgedBy = userId;
        alert.acknowledgedAt = new Date();
        return true;
      }
    }
    return false;
  }

  /**
   * 알림 임계값 설정
   */
  setAlertThresholds(clientId: string, thresholds: AlertThreshold[]): void {
    this.alertThresholds.set(clientId, thresholds);
  }

  /**
   * 알림 임계값 체크
   */
  private checkAlertThresholds(clientId: string, activity: Activity): void {
    const thresholds = this.alertThresholds.get(clientId) ?? [];

    // 비활동 체크
    const inactivityThreshold = thresholds.find((t) => t.type === 'inactivity');
    if (inactivityThreshold?.enabled) {
      // 마지막 활동 시간 체크
      const lastActivityTime = activity.timestamp.getTime();
      const now = Date.now();
      if (now - lastActivityTime > inactivityThreshold.threshold * 60 * 1000) {
        this.createAlert(clientId, {
          severity: 'warning',
          type: 'inactivity',
          message: `${inactivityThreshold.threshold}분 동안 활동이 없습니다`,
        });
      }
    }

    // 높은 오류율 체크
    const errorThreshold = thresholds.find((t) => t.type === 'high_error_rate');
    if (errorThreshold?.enabled && activity.data.successful === false) {
      const recentActivities = (this.activities.get(clientId) ?? []).slice(-10);
      const errorRate =
        recentActivities.filter((a) => a.data.successful === false).length /
        recentActivities.length;

      if (errorRate > errorThreshold.threshold) {
        this.createAlert(clientId, {
          severity: 'warning',
          type: 'high_error_rate',
          message: `오류율이 ${Math.round(errorRate * 100)}%로 높습니다`,
        });
      }
    }
  }

  // ============================================================================
  // Remote Control
  // ============================================================================

  /**
   * UI 설정 원격 조정
   */
  adjustUIRemotely(
    clientId: string,
    config: Record<string, unknown>
  ): { success: boolean; message: string } {
    // 실제로는 WebSocket을 통해 클라이언트에 설정 전송
    console.log(`UI config adjustment for ${clientId}:`, config);
    return { success: true, message: 'UI 설정이 업데이트되었습니다' };
  }

  /**
   * 메시지 전송
   */
  sendMessageToClient(
    clientId: string,
    message: string
  ): { success: boolean; messageId: string } {
    // 실제로는 WebSocket을 통해 클라이언트에 메시지 전송
    const messageId = uuidv4();
    console.log(`Message sent to ${clientId}: ${message}`);
    return { success: true, messageId };
  }

  /**
   * 기기 잠금/해제
   */
  setDeviceLock(
    clientId: string,
    locked: boolean
  ): { success: boolean; locked: boolean } {
    // 실제로는 WebSocket을 통해 클라이언트에 잠금 명령 전송
    console.log(`Device lock for ${clientId}: ${locked}`);
    return { success: true, locked };
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  private calculateMoodTrend(moods: Mood[]): MoodTrend {
    if (moods.length === 0) {
      return { average: 'neutral', changes: [], stability: 1 };
    }

    const moodValues: Record<MoodLevel, number> = {
      very_low: 1,
      low: 2,
      neutral: 3,
      high: 4,
      very_high: 5,
    };

    const avgValue =
      moods.reduce((sum, m) => sum + moodValues[m.level], 0) / moods.length;

    const average = this.valueToMoodLevel(avgValue);

    const changes: MoodTrend['changes'] = [];
    for (let i = 1; i < moods.length; i++) {
      if (moods[i].level !== moods[i - 1].level) {
        changes.push({
          from: moods[i - 1].level,
          to: moods[i].level,
          timestamp: moods[i].timestamp,
        });
      }
    }

    // 안정성: 변화가 적을수록 높음
    const stability = Math.max(0, 1 - changes.length / moods.length);

    return { average, changes, stability };
  }

  private valueToMoodLevel(value: number): MoodLevel {
    if (value < 1.5) return 'very_low';
    if (value < 2.5) return 'low';
    if (value < 3.5) return 'neutral';
    if (value < 4.5) return 'high';
    return 'very_high';
  }

  private isMoodChangeSignificant(prev: MoodLevel, current: MoodLevel): boolean {
    const levels: MoodLevel[] = ['very_low', 'low', 'neutral', 'high', 'very_high'];
    const diff = Math.abs(levels.indexOf(prev) - levels.indexOf(current));
    return diff >= 2;
  }

  private getMoodLabel(level: MoodLevel): string {
    const labels: Record<MoodLevel, string> = {
      very_low: '매우 낮음',
      low: '낮음',
      neutral: '보통',
      high: '좋음',
      very_high: '매우 좋음',
    };
    return labels[level];
  }

  private findPeakUsageTime(activities: Activity[]): string {
    const hourCounts = new Array(24).fill(0);
    activities.forEach((a) => {
      const hour = a.timestamp.getHours();
      hourCounts[hour]++;
    });

    const peakHour = hourCounts.indexOf(Math.max(...hourCounts));
    return `${peakHour}:00 - ${peakHour + 1}:00`;
  }

  private generateWeeklyRecommendations(summaries: DailySummary[]): string[] {
    const recommendations: string[] = [];

    const avgCommunications =
      summaries.reduce((sum, s) => sum + s.totalCommunications, 0) / summaries.length;

    if (avgCommunications < 10) {
      recommendations.push(
        '의사소통 기회를 늘리기 위해 일상 활동 중 AAC 사용을 권장합니다'
      );
    }

    const avgResponseTime =
      summaries.reduce((sum, s) => sum + s.avgResponseTime, 0) / summaries.length;

    if (avgResponseTime > 3000) {
      recommendations.push(
        '반응 시간이 긴 편입니다. 심볼 배치 최적화를 고려해 보세요'
      );
    }

    return recommendations;
  }
}

export default CaregiverService;
