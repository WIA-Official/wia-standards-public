/**
 * WIA Cognitive AAC - Professional Service
 * SLP/OT 전문가 대시보드 서비스
 *
 * 홍익인간 (弘益人間) - 널리 인간을 이롭게 하라
 */

import { v4 as uuidv4 } from 'uuid';
import {
  Client,
  Goal,
  GoalProgress,
  GoalCategory,
  GoalStatus,
  Milestone,
  AssessmentResult,
  AssessmentType,
  AssessmentScore,
  ProgressComparison,
  ProgressMetric,
  Symbol,
  BoardLayout,
  BoardPage,
  ClinicalReport,
  ReportSection,
} from '../types';

// ============================================================================
// Types
// ============================================================================

export interface ProfessionalDashboardData {
  clients: ClientSummary[];
  currentClient: Client | null;
  pendingAssessments: AssessmentTask[];
  recentReports: ClinicalReport[];
}

export interface ClientSummary {
  id: string;
  name: string;
  type: string;
  lastSession: Date | null;
  nextAppointment: Date | null;
  activeGoals: number;
  progressStatus: 'on_track' | 'needs_attention' | 'excelling';
}

export interface AssessmentTask {
  clientId: string;
  clientName: string;
  assessmentType: AssessmentType;
  dueDate: Date;
  priority: 'high' | 'medium' | 'low';
}

export interface VocabularyAnalysis {
  coreVocabulary: VocabularyItem[];
  fringeVocabulary: VocabularyItem[];
  missingCore: string[];
  recommendations: string[];
}

export interface VocabularyItem {
  symbolId: string;
  label: string;
  frequency: number;
  lastUsed: Date;
  contexts: string[];
}

export interface CommunicationPattern {
  patternType: string;
  description: string;
  frequency: number;
  examples: string[];
  clinicalImplication?: string;
}

export interface ErrorAnalysis {
  totalErrors: number;
  errorRate: number;
  commonErrors: ErrorCategory[];
  recommendations: string[];
}

export interface ErrorCategory {
  type: string;
  count: number;
  percentage: number;
  examples: string[];
  suggestedIntervention?: string;
}

// ============================================================================
// Professional Service
// ============================================================================

export class ProfessionalService {
  // In-memory stores
  private clients: Map<string, Client> = new Map();
  private goals: Map<string, Goal[]> = new Map();
  private assessments: Map<string, AssessmentResult[]> = new Map();
  private vocabularies: Map<string, Symbol[]> = new Map();
  private layouts: Map<string, BoardLayout[]> = new Map();
  private reports: Map<string, ClinicalReport[]> = new Map();

  // ============================================================================
  // Client Management
  // ============================================================================

  /**
   * 전문가의 담당 클라이언트 목록
   */
  getClientList(professionalId: string): ClientSummary[] {
    const clientSummaries: ClientSummary[] = [];

    for (const [, client] of this.clients) {
      const isProfessional = client.professionals.some(
        (p) => p.userId === professionalId
      );
      if (isProfessional) {
        const clientGoals = this.goals.get(client.id) ?? [];
        const activeGoals = clientGoals.filter(
          (g) => g.status === 'in_progress'
        ).length;

        clientSummaries.push({
          id: client.id,
          name: client.name,
          type: client.type,
          lastSession: null, // 실제로는 세션 데이터 조회
          nextAppointment: null,
          activeGoals,
          progressStatus: this.determineProgressStatus(client.id),
        });
      }
    }

    return clientSummaries;
  }

  /**
   * 클라이언트 상세 조회
   */
  getClientDetail(clientId: string): Client | null {
    return this.clients.get(clientId) ?? null;
  }

  // ============================================================================
  // Assessment
  // ============================================================================

  /**
   * 평가 실시
   */
  conductAssessment(
    clientId: string,
    type: AssessmentType,
    conductedBy: string,
    scores: AssessmentScore[],
    notes: string
  ): AssessmentResult {
    const result: AssessmentResult = {
      id: uuidv4(),
      clientId,
      type,
      conductedBy,
      conductedAt: new Date(),
      scores,
      notes,
      recommendations: this.generateAssessmentRecommendations(type, scores),
    };

    const clientAssessments = this.assessments.get(clientId) ?? [];
    clientAssessments.push(result);
    this.assessments.set(clientId, clientAssessments);

    return result;
  }

  /**
   * 평가 기록 조회
   */
  getAssessmentHistory(clientId: string): AssessmentResult[] {
    return this.assessments.get(clientId) ?? [];
  }

  /**
   * 진행 비교 분석
   */
  compareProgress(
    clientId: string,
    startDate: Date,
    endDate: Date
  ): ProgressComparison {
    const clientAssessments = this.assessments.get(clientId) ?? [];

    // 기간 내 평가 찾기
    const startAssessment = clientAssessments.find(
      (a) =>
        a.conductedAt >= startDate &&
        a.conductedAt <= new Date(startDate.getTime() + 30 * 24 * 60 * 60 * 1000)
    );
    const endAssessment = clientAssessments.find(
      (a) =>
        a.conductedAt <= endDate &&
        a.conductedAt >= new Date(endDate.getTime() - 30 * 24 * 60 * 60 * 1000)
    );

    const metrics: ProgressMetric[] = [];

    if (startAssessment && endAssessment) {
      // 동일 도메인 점수 비교
      for (const endScore of endAssessment.scores) {
        const startScore = startAssessment.scores.find(
          (s) => s.domain === endScore.domain
        );
        if (startScore) {
          const change = endScore.score - startScore.score;
          const changePercent =
            startScore.score > 0 ? (change / startScore.score) * 100 : 0;

          metrics.push({
            name: endScore.domain,
            startValue: startScore.score,
            endValue: endScore.score,
            change,
            changePercent,
          });
        }
      }
    }

    const summary = this.generateProgressSummary(metrics);

    return {
      clientId,
      startDate,
      endDate,
      metrics,
      summary,
    };
  }

  // ============================================================================
  // Treatment Planning
  // ============================================================================

  /**
   * 목표 설정
   */
  setGoal(
    clientId: string,
    goalData: Omit<Goal, 'id' | 'createdAt' | 'updatedAt'>
  ): Goal {
    const goal: Goal = {
      ...goalData,
      id: uuidv4(),
      createdAt: new Date(),
      updatedAt: new Date(),
    };

    const clientGoals = this.goals.get(clientId) ?? [];
    clientGoals.push(goal);
    this.goals.set(clientId, clientGoals);

    return goal;
  }

  /**
   * 현재 목표 조회
   */
  getCurrentGoals(clientId: string): Goal[] {
    const clientGoals = this.goals.get(clientId) ?? [];
    return clientGoals.filter(
      (g) => g.status === 'in_progress' || g.status === 'not_started'
    );
  }

  /**
   * 목표 진행 추적
   */
  trackGoalProgress(goalId: string, progress: number): GoalProgress | null {
    for (const [clientId, clientGoals] of this.goals) {
      const goal = clientGoals.find((g) => g.id === goalId);
      if (goal) {
        const previousProgress = goal.progress;
        goal.progress = Math.min(100, Math.max(0, progress));
        goal.updatedAt = new Date();

        // 100% 달성 시 상태 업데이트
        if (goal.progress >= 100) {
          goal.status = 'achieved';
        }

        return {
          goalId,
          goalTitle: goal.title,
          currentProgress: goal.progress,
          previousProgress,
          trend:
            goal.progress > previousProgress
              ? 'improving'
              : goal.progress < previousProgress
              ? 'declining'
              : 'stable',
          lastUpdate: goal.updatedAt,
        };
      }
    }
    return null;
  }

  /**
   * 마일스톤 달성 기록
   */
  achieveMilestone(goalId: string, milestoneId: string): boolean {
    for (const [, clientGoals] of this.goals) {
      const goal = clientGoals.find((g) => g.id === goalId);
      if (goal) {
        const milestone = goal.milestones.find((m) => m.id === milestoneId);
        if (milestone) {
          milestone.achieved = true;
          milestone.achievedDate = new Date();
          goal.updatedAt = new Date();
          return true;
        }
      }
    }
    return false;
  }

  // ============================================================================
  // Vocabulary Management
  // ============================================================================

  /**
   * 어휘 분석
   */
  analyzeVocabulary(clientId: string): VocabularyAnalysis {
    const clientVocabulary = this.vocabularies.get(clientId) ?? [];

    const coreVocabulary = clientVocabulary
      .filter((s) => s.isCore)
      .map((s) => ({
        symbolId: s.id,
        label: s.label,
        frequency: 0, // 실제로는 사용 데이터에서 계산
        lastUsed: new Date(),
        contexts: [],
      }));

    const fringeVocabulary = clientVocabulary
      .filter((s) => !s.isCore)
      .map((s) => ({
        symbolId: s.id,
        label: s.label,
        frequency: 0,
        lastUsed: new Date(),
        contexts: [],
      }));

    // 표준 핵심 어휘와 비교
    const standardCoreWords = [
      'I', 'you', 'want', 'more', 'help', 'stop', 'go', 'yes', 'no', 'like',
      'eat', 'drink', 'play', 'finished', 'again', 'different', 'that', 'here',
    ];
    const missingCore = standardCoreWords.filter(
      (word) => !coreVocabulary.some((v) => v.label.toLowerCase() === word.toLowerCase())
    );

    const recommendations: string[] = [];
    if (missingCore.length > 0) {
      recommendations.push(
        `핵심 어휘 추가 권장: ${missingCore.slice(0, 5).join(', ')}`
      );
    }
    if (fringeVocabulary.length < 10) {
      recommendations.push('관심사 기반 주변 어휘 확장을 고려하세요');
    }

    return {
      coreVocabulary,
      fringeVocabulary,
      missingCore,
      recommendations,
    };
  }

  /**
   * 심볼 추가
   */
  addSymbol(clientId: string, symbol: Omit<Symbol, 'id' | 'createdAt'>): Symbol {
    const newSymbol: Symbol = {
      ...symbol,
      id: uuidv4(),
      customFor: clientId,
      createdAt: new Date(),
    };

    const clientVocabulary = this.vocabularies.get(clientId) ?? [];
    clientVocabulary.push(newSymbol);
    this.vocabularies.set(clientId, clientVocabulary);

    return newSymbol;
  }

  /**
   * 레이아웃 커스터마이징
   */
  customizeLayout(clientId: string, layout: Omit<BoardLayout, 'id' | 'createdAt' | 'updatedAt'>): BoardLayout {
    const newLayout: BoardLayout = {
      ...layout,
      id: uuidv4(),
      clientId,
      createdAt: new Date(),
      updatedAt: new Date(),
    };

    const clientLayouts = this.layouts.get(clientId) ?? [];
    clientLayouts.push(newLayout);
    this.layouts.set(clientId, clientLayouts);

    return newLayout;
  }

  // ============================================================================
  // Analytics
  // ============================================================================

  /**
   * 의사소통 패턴 분석
   */
  analyzeCommunicationPatterns(clientId: string): CommunicationPattern[] {
    // 실제로는 활동 데이터 분석
    return [
      {
        patternType: 'request',
        description: '요청 의사소통이 주를 이룸',
        frequency: 45,
        examples: ['물 주세요', '더 원해요'],
        clinicalImplication: '요청 기능은 잘 발달됨. 다른 의사소통 기능 확장 권장',
      },
      {
        patternType: 'comment',
        description: '자발적 코멘트 증가 추세',
        frequency: 25,
        examples: ['재미있어', '이거 좋아'],
        clinicalImplication: '긍정적 발전. 코멘트 기회 지속 제공 필요',
      },
    ];
  }

  /**
   * 오류 분석
   */
  analyzeErrors(clientId: string): ErrorAnalysis {
    // 실제로는 오류 데이터 분석
    return {
      totalErrors: 23,
      errorRate: 0.08,
      commonErrors: [
        {
          type: 'selection_error',
          count: 12,
          percentage: 52,
          examples: ['인접 심볼 잘못 선택'],
          suggestedIntervention: '심볼 크기 증가 또는 간격 조정',
        },
        {
          type: 'navigation_error',
          count: 8,
          percentage: 35,
          examples: ['원하는 페이지 찾지 못함'],
          suggestedIntervention: '카테고리 구조 단순화',
        },
      ],
      recommendations: [
        '심볼 배치 최적화로 선택 오류 감소 가능',
        '핵심 어휘를 첫 페이지에 배치하여 네비게이션 부담 감소',
      ],
    };
  }

  // ============================================================================
  // Report Generation
  // ============================================================================

  /**
   * 임상 보고서 생성
   */
  generateReport(
    clientId: string,
    type: ClinicalReport['type'],
    generatedBy: string,
    period: { start: Date; end: Date }
  ): ClinicalReport {
    const client = this.clients.get(clientId);
    const clientGoals = this.goals.get(clientId) ?? [];
    const clientAssessments = this.assessments.get(clientId) ?? [];

    const sections: ReportSection[] = [];

    // 클라이언트 정보 섹션
    sections.push({
      title: '클라이언트 정보',
      content: `이름: ${client?.name ?? 'N/A'}\n유형: ${client?.type ?? 'N/A'}`,
    });

    // 평가 기간 내 진행상황
    const periodAssessments = clientAssessments.filter(
      (a) => a.conductedAt >= period.start && a.conductedAt <= period.end
    );

    if (periodAssessments.length > 0) {
      sections.push({
        title: '평가 결과',
        content: periodAssessments
          .map((a) => `${a.type}: ${a.notes}`)
          .join('\n'),
        data: { assessments: periodAssessments },
      });
    }

    // 목표 진행상황
    const activeGoals = clientGoals.filter(
      (g) => g.status === 'in_progress' || g.status === 'achieved'
    );
    if (activeGoals.length > 0) {
      sections.push({
        title: '목표 진행상황',
        content: activeGoals
          .map((g) => `${g.title}: ${g.progress}% (${g.status})`)
          .join('\n'),
        data: { goals: activeGoals },
      });
    }

    // 의사소통 패턴 분석
    const patterns = this.analyzeCommunicationPatterns(clientId);
    sections.push({
      title: '의사소통 패턴 분석',
      content: patterns.map((p) => `${p.patternType}: ${p.description}`).join('\n'),
    });

    // 권장사항
    const recommendations = this.generateClinicalRecommendations(
      clientId,
      patterns,
      activeGoals
    );

    const report: ClinicalReport = {
      id: uuidv4(),
      clientId,
      type,
      generatedBy,
      generatedAt: new Date(),
      period,
      sections,
      recommendations,
    };

    const clientReports = this.reports.get(clientId) ?? [];
    clientReports.push(report);
    this.reports.set(clientId, clientReports);

    return report;
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  private determineProgressStatus(
    clientId: string
  ): 'on_track' | 'needs_attention' | 'excelling' {
    const clientGoals = this.goals.get(clientId) ?? [];
    const activeGoals = clientGoals.filter((g) => g.status === 'in_progress');

    if (activeGoals.length === 0) return 'on_track';

    const avgProgress =
      activeGoals.reduce((sum, g) => sum + g.progress, 0) / activeGoals.length;

    if (avgProgress >= 80) return 'excelling';
    if (avgProgress < 30) return 'needs_attention';
    return 'on_track';
  }

  private generateAssessmentRecommendations(
    type: AssessmentType,
    scores: AssessmentScore[]
  ): string[] {
    const recommendations: string[] = [];

    for (const score of scores) {
      const percentage = score.maxScore > 0 ? score.score / score.maxScore : 0;

      if (percentage < 0.3) {
        recommendations.push(
          `${score.domain} 영역에 집중적인 지원 필요`
        );
      } else if (percentage < 0.6) {
        recommendations.push(
          `${score.domain} 영역 강화를 위한 활동 권장`
        );
      }
    }

    return recommendations;
  }

  private generateProgressSummary(metrics: ProgressMetric[]): string {
    if (metrics.length === 0) {
      return '비교 가능한 데이터가 부족합니다.';
    }

    const improving = metrics.filter((m) => m.changePercent > 5);
    const declining = metrics.filter((m) => m.changePercent < -5);

    let summary = '';

    if (improving.length > 0) {
      summary += `향상된 영역: ${improving.map((m) => m.name).join(', ')}. `;
    }
    if (declining.length > 0) {
      summary += `주의 필요 영역: ${declining.map((m) => m.name).join(', ')}. `;
    }
    if (improving.length === 0 && declining.length === 0) {
      summary = '전반적으로 안정적인 상태를 유지하고 있습니다.';
    }

    return summary;
  }

  private generateClinicalRecommendations(
    clientId: string,
    patterns: CommunicationPattern[],
    goals: Goal[]
  ): string[] {
    const recommendations: string[] = [];

    // 패턴 기반 권장
    const lowFrequencyPatterns = patterns.filter((p) => p.frequency < 20);
    if (lowFrequencyPatterns.length > 0) {
      recommendations.push(
        `다양한 의사소통 기능 확장: ${lowFrequencyPatterns.map((p) => p.patternType).join(', ')}`
      );
    }

    // 목표 기반 권장
    const stagnantGoals = goals.filter(
      (g) => g.status === 'in_progress' && g.progress < 30
    );
    if (stagnantGoals.length > 0) {
      recommendations.push(
        '진행이 느린 목표에 대한 전략 수정 고려'
      );
    }

    return recommendations;
  }
}

export default ProfessionalService;
