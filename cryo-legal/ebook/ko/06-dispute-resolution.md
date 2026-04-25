# 제6장: 분쟁 해결 - 메커니즘 및 절차

## 종합 분쟁 해결 프레임워크

이 장에서는 WIA 극저온 법률 표준의 완전한 분쟁 해결 구현을 제공하며, 극저온보존 법적 문제에 특화된 협상, 조정, 중재 및 소송 절차를 다룹니다.

## 분쟁 해결 엔진

```typescript
/**
 * WIA 극저온 법률 표준 - 분쟁 해결 엔진
 * 단계별 분쟁 해결 및 에스컬레이션 관리
 */

import { z } from 'zod';
import {
  Dispute,
  DisputeSchema,
  DisputeType,
  DisputeStatus,
  Resolution,
  ResolutionType,
  ResolutionMechanism,
  EscalationPath,
} from './types';

// ============================================================================
// 분쟁 해결 설정
// ============================================================================

export interface DisputeResolutionConfig {
  defaultMechanisms: ResolutionMechanism[];  // 기본 해결 메커니즘
  escalationPath: EscalationPath;            // 에스컬레이션 경로
  timeframes: TimeframeConfig;               // 기한 설정
  notifications: NotificationConfig;         // 알림 설정
  documentation: DocumentationConfig;        // 문서화 설정
}

export interface TimeframeConfig {
  negotiationDays: number;        // 협상 기간 (일)
  mediationDays: number;          // 조정 기간 (일)
  arbitrationDays: number;        // 중재 기간 (일)
  responseDeadlineHours: number;  // 응답 기한 (시간)
  escalationTriggerDays: number;  // 에스컬레이션 트리거 (일)
}

export interface NotificationConfig {
  channels: ('email' | 'sms' | 'portal')[];  // 알림 채널
  reminderFrequency: number;                  // 알림 빈도 (일)
  escalationAlerts: boolean;                  // 에스컬레이션 알림 여부
}

export interface DocumentationConfig {
  requiredDocuments: string[];                          // 필수 문서
  retentionYears: number;                               // 보관 기간 (년)
  confidentiality: 'public' | 'confidential' | 'highly-confidential';
}

// ============================================================================
// 한국 분쟁 해결 설정
// ============================================================================

export const koreanDisputeConfig: DisputeResolutionConfig = {
  defaultMechanisms: [
    {
      type: 'negotiation',
      typeKr: '협상',
      order: 1,
      mandatory: true,
      timeframe: '30일',
      costs: '각자 부담',
    },
    {
      type: 'mediation',
      typeKr: '조정',
      order: 2,
      mandatory: true,
      provider: '대한상사중재원',
      rules: '대한상사중재원 조정규칙',
      timeframe: '60일',
      costs: '균등 분담',
    },
    {
      type: 'arbitration',
      typeKr: '중재',
      order: 3,
      mandatory: false,
      provider: '대한상사중재원',
      rules: '대한상사중재원 중재규칙',
      timeframe: '120일',
      costs: '중재 합의에 따름',
    },
    {
      type: 'litigation',
      typeKr: '소송',
      order: 4,
      mandatory: false,
      timeframe: '법원 절차에 따름',
      costs: '패소자 부담 원칙',
    },
  ],
  escalationPath: {
    levels: [
      { level: 1, mechanism: 'negotiation', mechanismKr: '협상', trigger: '분쟁 접수', timeframe: '30일' },
      { level: 2, mechanism: 'mediation', mechanismKr: '조정', trigger: '협상 실패', timeframe: '60일' },
      { level: 3, mechanism: 'arbitration', mechanismKr: '중재', trigger: '조정 실패', timeframe: '120일' },
    ],
    finalResort: 'litigation',
    finalResortKr: '소송',
  },
  timeframes: {
    negotiationDays: 30,
    mediationDays: 60,
    arbitrationDays: 120,
    responseDeadlineHours: 72,
    escalationTriggerDays: 7,
  },
  notifications: {
    channels: ['email', 'sms', 'portal'],
    reminderFrequency: 7,
    escalationAlerts: true,
  },
  documentation: {
    requiredDocuments: [
      '분쟁 신청서',
      '증빙 자료',
      '계약서 사본',
      '통신 기록',
    ],
    retentionYears: 10,
    confidentiality: 'confidential',
  },
};

// ============================================================================
// 분쟁 관리자
// ============================================================================

export class DisputeManager {
  private disputes: Map<string, DisputeRecord> = new Map();
  private eventBus: DisputeEventBus;

  constructor(
    private readonly config: DisputeResolutionConfig,
    private readonly storage: DisputeStorage,
    private readonly notificationService: DisputeNotificationService,
    private readonly documentService: DocumentService
  ) {
    this.eventBus = new DisputeEventBus();
    this.startBackgroundProcesses();
  }

  private startBackgroundProcesses(): void {
    // 에스컬레이션 트리거 주기적 확인
    setInterval(() => this.checkEscalationTriggers(), 3600000); // 매시간

    // 기한 알림 발송
    setInterval(() => this.sendDeadlineReminders(), 86400000); // 매일
  }

  async fileDispute(
    filing: DisputeFiling,
    filer: string
  ): Promise<DisputeRecord> {
    // 신청 검증
    this.validateFiling(filing);

    // 분쟁 기록 생성
    const disputeId = this.generateDisputeId();

    const dispute: DisputeRecord = {
      id: disputeId,
      type: filing.type,
      typeKr: this.getDisputeTypeKr(filing.type),
      parties: filing.parties,
      subject: filing.subject,
      description: filing.description,
      filedDate: new Date().toISOString(),
      filedBy: filer,
      status: 'filed',
      statusKr: '접수됨',
      currentMechanism: this.config.defaultMechanisms[0].type,
      currentMechanismKr: this.config.defaultMechanisms[0].typeKr,
      mechanismLevel: 1,
      deadlines: this.calculateDeadlines('filed'),
      documents: [],
      communications: [],
      history: [{
        date: new Date().toISOString(),
        type: 'filed',
        typeKr: '접수',
        description: '분쟁 신청됨',
        actor: filer,
      }],
    };

    // 분쟁 저장
    await this.storage.save(dispute);
    this.disputes.set(disputeId, dispute);

    // 증빙 문서 업로드
    if (filing.documents && filing.documents.length > 0) {
      await this.addDocuments(disputeId, filing.documents, filer);
    }

    // 당사자들에게 알림
    await this.notifyParties(dispute, 'dispute_filed', {
      responseDeadline: dispute.deadlines.response,
    });

    this.eventBus.emit('dispute:filed', { dispute });

    return dispute;
  }

  async respondToDispute(
    disputeId: string,
    response: DisputeResponse,
    responder: string
  ): Promise<DisputeRecord> {
    const dispute = await this.getDispute(disputeId);

    if (dispute.status !== 'filed') {
      throw new Error(`${dispute.statusKr || dispute.status} 상태의 분쟁에는 응답할 수 없습니다`);
    }

    // 응답자가 당사자인지 확인
    if (!dispute.parties.includes(responder)) {
      throw new Error('응답자는 분쟁 당사자여야 합니다');
    }

    // 분쟁 업데이트
    dispute.response = {
      content: response.content,
      respondedAt: new Date().toISOString(),
      respondedBy: responder,
      acceptsNegotiation: response.acceptsNegotiation,
    };

    dispute.status = 'under-review';
    dispute.statusKr = '검토 중';

    dispute.history.push({
      date: new Date().toISOString(),
      type: 'responded',
      typeKr: '응답',
      description: `${responder}가 응답함`,
      actor: responder,
    });

    // 응답 문서 추가
    if (response.documents && response.documents.length > 0) {
      await this.addDocuments(disputeId, response.documents, responder);
    }

    await this.storage.save(dispute);

    // 신청자에게 알림
    await this.notifyParties(dispute, 'dispute_response', {
      responseBy: responder,
    });

    // 양측이 협상에 동의하면 협상 시작
    if (response.acceptsNegotiation) {
      await this.initiateNegotiation(dispute);
    }

    this.eventBus.emit('dispute:responded', { dispute, response });

    return dispute;
  }

  async initiateNegotiation(dispute: DisputeRecord): Promise<DisputeRecord> {
    if (dispute.currentMechanism !== 'negotiation') {
      throw new Error(`현재 메커니즘이 협상이 아닌 ${dispute.currentMechanismKr || dispute.currentMechanism}입니다`);
    }

    dispute.status = 'negotiation';
    dispute.statusKr = '협상 중';

    dispute.history.push({
      date: new Date().toISOString(),
      type: 'negotiation_started',
      typeKr: '협상 시작',
      description: '협상 단계 시작됨',
      actor: 'system',
    });

    dispute.deadlines = this.calculateDeadlines('negotiation');

    await this.storage.save(dispute);

    // 협상 세션 생성
    const session = await this.createNegotiationSession(dispute);

    await this.notifyParties(dispute, 'negotiation_started', {
      deadline: dispute.deadlines.mechanismEnd,
      sessionId: session.id,
    });

    this.eventBus.emit('dispute:negotiation_started', { dispute, session });

    return dispute;
  }

  async recordNegotiationProgress(
    disputeId: string,
    progress: NegotiationProgress,
    actor: string
  ): Promise<DisputeRecord> {
    const dispute = await this.getDispute(disputeId);

    if (dispute.status !== 'negotiation') {
      throw new Error(`분쟁이 협상 단계가 아닙니다`);
    }

    dispute.communications.push({
      date: new Date().toISOString(),
      from: actor,
      type: 'negotiation',
      typeKr: '협상',
      content: progress.summary,
      attachments: progress.documents || [],
    });

    dispute.history.push({
      date: new Date().toISOString(),
      type: 'negotiation_progress',
      typeKr: '협상 진행',
      description: progress.summary,
      actor,
    });

    if (progress.proposedTerms) {
      dispute.proposedResolution = {
        terms: progress.proposedTerms,
        proposedBy: actor,
        proposedAt: new Date().toISOString(),
        status: 'pending',
        statusKr: '대기 중',
      };
    }

    await this.storage.save(dispute);

    this.eventBus.emit('dispute:negotiation_progress', { dispute, progress });

    return dispute;
  }

  async acceptProposedResolution(
    disputeId: string,
    acceptedBy: string
  ): Promise<DisputeRecord> {
    const dispute = await this.getDispute(disputeId);

    if (!dispute.proposedResolution) {
      throw new Error('수락할 제안된 해결안이 없습니다');
    }

    if (dispute.proposedResolution.proposedBy === acceptedBy) {
      throw new Error('자신의 제안을 수락할 수 없습니다');
    }

    // 해결안 수락 표시
    dispute.proposedResolution.status = 'accepted';
    dispute.proposedResolution.statusKr = '수락됨';
    dispute.proposedResolution.acceptedBy = acceptedBy;
    dispute.proposedResolution.acceptedAt = new Date().toISOString();

    // 분쟁 해결
    dispute.status = 'resolved';
    dispute.statusKr = '해결됨';
    dispute.resolution = {
      date: new Date().toISOString(),
      mechanism: dispute.currentMechanism,
      mechanismKr: dispute.currentMechanismKr,
      outcome: '협상을 통한 합의 도출',
      terms: dispute.proposedResolution.terms,
      binding: true,
    };

    dispute.history.push({
      date: new Date().toISOString(),
      type: 'resolved',
      typeKr: '해결',
      description: `${acceptedBy}가 해결안 수락함`,
      actor: acceptedBy,
    });

    await this.storage.save(dispute);

    await this.notifyParties(dispute, 'dispute_resolved', {
      mechanism: 'negotiation',
      mechanismKr: '협상',
      terms: dispute.resolution.terms,
    });

    this.eventBus.emit('dispute:resolved', { dispute });

    return dispute;
  }

  async escalateDispute(
    disputeId: string,
    reason: string,
    escalatedBy: string
  ): Promise<DisputeRecord> {
    const dispute = await this.getDispute(disputeId);

    // 다음 메커니즘 찾기
    const currentLevel = dispute.mechanismLevel;
    const nextLevel = currentLevel + 1;

    const nextMechanism = this.config.escalationPath.levels.find(
      l => l.level === nextLevel
    );

    if (!nextMechanism) {
      // 최종 수단
      dispute.currentMechanism = this.config.escalationPath.finalResort;
      dispute.currentMechanismKr = this.config.escalationPath.finalResortKr;
      dispute.status = 'litigation';
      dispute.statusKr = '소송 중';
    } else {
      dispute.currentMechanism = nextMechanism.mechanism;
      dispute.currentMechanismKr = nextMechanism.mechanismKr;
      dispute.status = nextMechanism.mechanism as DisputeStatus;
      dispute.statusKr = this.getStatusKr(nextMechanism.mechanism);
      dispute.mechanismLevel = nextLevel;
    }

    dispute.deadlines = this.calculateDeadlines(dispute.status);

    dispute.history.push({
      date: new Date().toISOString(),
      type: 'escalated',
      typeKr: '상향 조정',
      description: `${dispute.currentMechanismKr}(으)로 상향 조정: ${reason}`,
      actor: escalatedBy,
    });

    await this.storage.save(dispute);

    await this.notifyParties(dispute, 'dispute_escalated', {
      newMechanism: dispute.currentMechanism,
      newMechanismKr: dispute.currentMechanismKr,
      reason,
      deadline: dispute.deadlines.mechanismEnd,
    });

    this.eventBus.emit('dispute:escalated', { dispute, reason });

    // 조정으로 상향 시 조정인 배정
    if (dispute.currentMechanism === 'mediation') {
      await this.assignMediator(dispute);
    }

    // 중재로 상향 시 중재 절차 시작
    if (dispute.currentMechanism === 'arbitration') {
      await this.initiateArbitration(dispute);
    }

    return dispute;
  }

  private async assignMediator(dispute: DisputeRecord): Promise<void> {
    const mechanism = this.config.defaultMechanisms.find(m => m.type === 'mediation');

    dispute.mediator = {
      provider: mechanism?.provider || '대한상사중재원',
      assigned: false,
      assignedAt: undefined,
      name: undefined,
    };

    dispute.history.push({
      date: new Date().toISOString(),
      type: 'mediator_requested',
      typeKr: '조정인 요청',
      description: `${dispute.mediator.provider}에 조정인 요청됨`,
      actor: 'system',
    });

    await this.storage.save(dispute);
  }

  async recordMediatorAssignment(
    disputeId: string,
    mediatorName: string,
    mediatorInfo?: MediatorInfo
  ): Promise<DisputeRecord> {
    const dispute = await this.getDispute(disputeId);

    if (!dispute.mediator) {
      throw new Error('분쟁이 조정 단계가 아닙니다');
    }

    dispute.mediator.assigned = true;
    dispute.mediator.assignedAt = new Date().toISOString();
    dispute.mediator.name = mediatorName;
    dispute.mediator.info = mediatorInfo;

    dispute.history.push({
      date: new Date().toISOString(),
      type: 'mediator_assigned',
      typeKr: '조정인 배정',
      description: `조정인 배정됨: ${mediatorName}`,
      actor: 'system',
    });

    await this.storage.save(dispute);

    await this.notifyParties(dispute, 'mediator_assigned', {
      mediatorName,
    });

    return dispute;
  }

  async scheduleMediationSession(
    disputeId: string,
    session: MediationSession
  ): Promise<DisputeRecord> {
    const dispute = await this.getDispute(disputeId);

    if (dispute.status !== 'mediation') {
      throw new Error('분쟁이 조정 단계가 아닙니다');
    }

    if (!dispute.mediationSessions) {
      dispute.mediationSessions = [];
    }

    dispute.mediationSessions.push({
      ...session,
      id: crypto.randomUUID(),
      status: 'scheduled',
      statusKr: '예정됨',
    });

    dispute.history.push({
      date: new Date().toISOString(),
      type: 'session_scheduled',
      typeKr: '세션 예정',
      description: `조정 세션 예정: ${session.scheduledDate}`,
      actor: session.scheduledBy,
    });

    await this.storage.save(dispute);

    await this.notifyParties(dispute, 'session_scheduled', {
      date: session.scheduledDate,
      location: session.location,
    });

    return dispute;
  }

  async recordMediationOutcome(
    disputeId: string,
    sessionId: string,
    outcome: MediationOutcome
  ): Promise<DisputeRecord> {
    const dispute = await this.getDispute(disputeId);

    const session = dispute.mediationSessions?.find(s => s.id === sessionId);
    if (!session) {
      throw new Error('세션을 찾을 수 없습니다');
    }

    session.status = 'completed';
    session.statusKr = '완료됨';
    session.outcome = outcome;

    dispute.history.push({
      date: new Date().toISOString(),
      type: 'session_completed',
      typeKr: '세션 완료',
      description: `조정 세션 완료: ${outcome.summary}`,
      actor: dispute.mediator?.name || '조정인',
    });

    if (outcome.settlementReached) {
      dispute.status = 'resolved';
      dispute.statusKr = '해결됨';
      dispute.resolution = {
        date: new Date().toISOString(),
        mechanism: 'mediation',
        mechanismKr: '조정',
        outcome: '조정을 통한 합의 도출',
        terms: outcome.settlementTerms || [],
        binding: true,
      };

      await this.notifyParties(dispute, 'dispute_resolved', {
        mechanism: 'mediation',
        mechanismKr: '조정',
      });

      this.eventBus.emit('dispute:resolved', { dispute });
    }

    await this.storage.save(dispute);

    return dispute;
  }

  private async initiateArbitration(dispute: DisputeRecord): Promise<void> {
    const mechanism = this.config.defaultMechanisms.find(m => m.type === 'arbitration');

    dispute.arbitration = {
      provider: mechanism?.provider || '대한상사중재원',
      providerKr: '대한상사중재원',
      rules: mechanism?.rules || '대한상사중재원 중재규칙',
      initiated: new Date().toISOString(),
      arbitratorCount: this.determineArbitratorCount(dispute),
      arbitrators: [],
    };

    dispute.history.push({
      date: new Date().toISOString(),
      type: 'arbitration_initiated',
      typeKr: '중재 개시',
      description: `${dispute.arbitration.providerKr}에서 중재 개시됨`,
      actor: 'system',
    });

    await this.storage.save(dispute);
  }

  private determineArbitratorCount(dispute: DisputeRecord): number {
    // 소액 분쟁은 단독 중재인, 복잡/고액 분쟁은 3인
    return 1;
  }

  async recordArbitrationAward(
    disputeId: string,
    award: ArbitrationAward
  ): Promise<DisputeRecord> {
    const dispute = await this.getDispute(disputeId);

    if (dispute.status !== 'arbitration') {
      throw new Error('분쟁이 중재 단계가 아닙니다');
    }

    dispute.arbitration!.award = award;

    dispute.status = 'resolved';
    dispute.statusKr = '해결됨';
    dispute.resolution = {
      date: award.issuedDate,
      mechanism: 'arbitration',
      mechanismKr: '중재',
      outcome: award.summary,
      terms: award.orders,
      binding: true,
      enforcement: '중재법에 따라 법원에서 집행 가능',
    };

    dispute.history.push({
      date: new Date().toISOString(),
      type: 'arbitration_award',
      typeKr: '중재 판정',
      description: `중재 판정 선고됨`,
      actor: '중재인',
    });

    await this.storage.save(dispute);

    await this.notifyParties(dispute, 'arbitration_award', {
      summary: award.summary,
    });

    this.eventBus.emit('dispute:resolved', { dispute });

    return dispute;
  }

  async dismissDispute(
    disputeId: string,
    reason: string,
    dismissedBy: string
  ): Promise<DisputeRecord> {
    const dispute = await this.getDispute(disputeId);

    dispute.status = 'dismissed';
    dispute.statusKr = '기각됨';
    dispute.dismissal = {
      reason,
      dismissedBy,
      dismissedAt: new Date().toISOString(),
    };

    dispute.history.push({
      date: new Date().toISOString(),
      type: 'dismissed',
      typeKr: '기각',
      description: `분쟁 기각: ${reason}`,
      actor: dismissedBy,
    });

    await this.storage.save(dispute);

    await this.notifyParties(dispute, 'dispute_dismissed', { reason });

    this.eventBus.emit('dispute:dismissed', { dispute, reason });

    return dispute;
  }

  async addDocuments(
    disputeId: string,
    documents: DocumentUpload[],
    uploadedBy: string
  ): Promise<DisputeRecord> {
    const dispute = await this.getDispute(disputeId);

    for (const doc of documents) {
      const savedDoc = await this.documentService.upload(doc, {
        disputeId,
        uploadedBy,
        confidentiality: this.config.documentation.confidentiality,
      });

      dispute.documents.push({
        id: savedDoc.id,
        name: doc.name,
        type: doc.type,
        uploadedBy,
        uploadedAt: new Date().toISOString(),
        url: savedDoc.url,
      });
    }

    dispute.history.push({
      date: new Date().toISOString(),
      type: 'documents_added',
      typeKr: '문서 추가',
      description: `${documents.length}개 문서 업로드됨`,
      actor: uploadedBy,
      documents: documents.map(d => d.name),
    });

    await this.storage.save(dispute);

    return dispute;
  }

  async getDispute(id: string): Promise<DisputeRecord> {
    let dispute = this.disputes.get(id);

    if (!dispute) {
      dispute = await this.storage.load(id);

      if (!dispute) {
        throw new Error(`분쟁을 찾을 수 없습니다: ${id}`);
      }

      this.disputes.set(id, dispute);
    }

    return dispute;
  }

  async getDisputesByParty(partyId: string): Promise<DisputeRecord[]> {
    return this.storage.findByParty(partyId);
  }

  async getDisputesByStatus(status: DisputeStatus): Promise<DisputeRecord[]> {
    return this.storage.findByStatus(status);
  }

  private generateDisputeId(): string {
    const timestamp = Date.now().toString(36).toUpperCase();
    const random = Math.random().toString(36).substring(2, 8).toUpperCase();
    return `DSP-${timestamp}-${random}`;
  }

  private validateFiling(filing: DisputeFiling): void {
    if (filing.parties.length < 2) {
      throw new Error('분쟁에는 최소 두 당사자가 필요합니다');
    }

    if (!filing.subject || filing.subject.trim().length === 0) {
      throw new Error('분쟁 제목이 필요합니다');
    }

    if (!filing.description || filing.description.trim().length < 50) {
      throw new Error('분쟁 설명은 최소 50자 이상이어야 합니다');
    }
  }

  private getDisputeTypeKr(type: DisputeType): string {
    const typeMap: Record<DisputeType, string> = {
      'contract': '계약 분쟁',
      'consent': '동의 분쟁',
      'ownership': '소유권 분쟁',
      'negligence': '과실 분쟁',
      'regulatory': '규제 분쟁',
    };
    return typeMap[type] || type;
  }

  private getStatusKr(status: string): string {
    const statusMap: Record<string, string> = {
      'filed': '접수됨',
      'under-review': '검토 중',
      'negotiation': '협상 중',
      'mediation': '조정 중',
      'arbitration': '중재 중',
      'litigation': '소송 중',
      'resolved': '해결됨',
      'dismissed': '기각됨',
    };
    return statusMap[status] || status;
  }

  private calculateDeadlines(status: string): DisputeDeadlines {
    const now = new Date();

    const addDays = (days: number): string => {
      const date = new Date(now);
      date.setDate(date.getDate() + days);
      return date.toISOString();
    };

    switch (status) {
      case 'filed':
        return {
          response: addDays(this.config.timeframes.responseDeadlineHours / 24),
          mechanismEnd: addDays(this.config.timeframes.negotiationDays),
        };
      case 'negotiation':
        return {
          mechanismEnd: addDays(this.config.timeframes.negotiationDays),
          escalationAvailable: addDays(this.config.timeframes.escalationTriggerDays),
        };
      case 'mediation':
        return {
          mechanismEnd: addDays(this.config.timeframes.mediationDays),
        };
      case 'arbitration':
        return {
          mechanismEnd: addDays(this.config.timeframes.arbitrationDays),
        };
      default:
        return {};
    }
  }

  private async createNegotiationSession(dispute: DisputeRecord): Promise<NegotiationSession> {
    return {
      id: crypto.randomUUID(),
      disputeId: dispute.id,
      createdAt: new Date().toISOString(),
      status: 'active',
      statusKr: '진행 중',
    };
  }

  private async checkEscalationTriggers(): Promise<void> {
    const activeDisputes = await this.storage.findByStatus('negotiation');

    for (const dispute of activeDisputes) {
      if (dispute.deadlines.escalationAvailable) {
        const escDate = new Date(dispute.deadlines.escalationAvailable);

        if (new Date() >= escDate && !dispute.escalationNotified) {
          await this.notifyParties(dispute, 'escalation_available', {});
          dispute.escalationNotified = true;
          await this.storage.save(dispute);
        }
      }
    }
  }

  private async sendDeadlineReminders(): Promise<void> {
    const activeStatuses: DisputeStatus[] = ['negotiation', 'mediation', 'arbitration'];

    for (const status of activeStatuses) {
      const disputes = await this.storage.findByStatus(status);

      for (const dispute of disputes) {
        if (dispute.deadlines.mechanismEnd) {
          const endDate = new Date(dispute.deadlines.mechanismEnd);
          const daysUntil = Math.ceil((endDate.getTime() - Date.now()) / (1000 * 60 * 60 * 24));

          if ([7, 3, 1].includes(daysUntil)) {
            await this.notifyParties(dispute, 'deadline_reminder', {
              daysUntil,
              deadline: dispute.deadlines.mechanismEnd,
            });
          }
        }
      }
    }
  }

  private async notifyParties(
    dispute: DisputeRecord,
    type: string,
    data: Record<string, unknown>
  ): Promise<void> {
    for (const partyId of dispute.parties) {
      await this.notificationService.notify(partyId, {
        type,
        disputeId: dispute.id,
        ...data,
      });
    }
  }

  onDisputeEvent(event: string, handler: (data: unknown) => void): void {
    this.eventBus.on(event, handler);
  }
}
```

## 분쟁 유형별 처리

```typescript
/**
 * 극저온보존 특화 분쟁 유형 처리
 */

export class CryoDisputeHandler {
  constructor(
    private readonly disputeManager: DisputeManager,
    private readonly expertService: ExpertWitnessService
  ) {}

  // ============================================================================
  // 계약 분쟁 처리
  // ============================================================================

  async handleContractDispute(
    disputeId: string,
    details: ContractDisputeDetails
  ): Promise<DisputeRecord> {
    const dispute = await this.disputeManager.getDispute(disputeId);

    // 계약 위반 분석
    const analysis = await this.analyzeContractBreach(details);

    // 전문가 의견 요청 (필요시)
    if (analysis.requiresExpert) {
      await this.requestExpertOpinion(dispute, analysis.expertType);
    }

    // 손해 산정
    const damages = await this.calculateDamages(details);

    dispute.analysis = {
      type: 'contract',
      typeKr: '계약 분쟁',
      breachType: details.breachType,
      breachTypeKr: this.getBreachTypeKr(details.breachType),
      damages,
      expertRequired: analysis.requiresExpert,
    };

    await this.disputeManager.storage.save(dispute);

    return dispute;
  }

  private async analyzeContractBreach(details: ContractDisputeDetails): Promise<{
    requiresExpert: boolean;
    expertType?: string;
  }> {
    // 계약 위반 분석 로직
    const technicalIssues = [
      'specimen-damage',
      'storage-failure',
      'protocol-violation',
    ];

    return {
      requiresExpert: technicalIssues.includes(details.breachType),
      expertType: technicalIssues.includes(details.breachType) ? 'cryobiology' : undefined,
    };
  }

  private getBreachTypeKr(breachType: string): string {
    const types: Record<string, string> = {
      'specimen-damage': '검체 손상',
      'storage-failure': '보관 실패',
      'protocol-violation': '프로토콜 위반',
      'fee-dispute': '비용 분쟁',
      'termination-breach': '해지 위반',
    };
    return types[breachType] || breachType;
  }

  // ============================================================================
  // 동의 분쟁 처리
  // ============================================================================

  async handleConsentDispute(
    disputeId: string,
    details: ConsentDisputeDetails
  ): Promise<DisputeRecord> {
    const dispute = await this.disputeManager.getDispute(disputeId);

    // 동의 유효성 분석
    const consentAnalysis = await this.analyzeConsentValidity(details);

    // 생명윤리 전문가 의견 요청 (필요시)
    if (consentAnalysis.requiresEthicsReview) {
      await this.requestEthicsReview(dispute);
    }

    dispute.analysis = {
      type: 'consent',
      typeKr: '동의 분쟁',
      consentIssue: details.issue,
      consentIssueKr: this.getConsentIssueKr(details.issue),
      validity: consentAnalysis.validity,
      ethicsReviewRequired: consentAnalysis.requiresEthicsReview,
    };

    await this.disputeManager.storage.save(dispute);

    return dispute;
  }

  private async analyzeConsentValidity(details: ConsentDisputeDetails): Promise<{
    validity: 'valid' | 'invalid' | 'disputed';
    requiresEthicsReview: boolean;
  }> {
    // 동의 유효성 분석 로직
    return {
      validity: 'disputed',
      requiresEthicsReview: details.issue === 'posthumous-use' || details.issue === 'research-use',
    };
  }

  private getConsentIssueKr(issue: string): string {
    const issues: Record<string, string> = {
      'informed-consent': '충분한 설명 동의',
      'posthumous-use': '사후 사용',
      'research-use': '연구 사용',
      'third-party-access': '제3자 접근',
      'withdrawal': '동의 철회',
    };
    return issues[issue] || issue;
  }

  private async requestEthicsReview(dispute: DisputeRecord): Promise<void> {
    dispute.ethicsReview = {
      requested: true,
      requestedAt: new Date().toISOString(),
      status: 'pending',
      statusKr: '대기 중',
    };
  }

  // ============================================================================
  // 소유권 분쟁 처리
  // ============================================================================

  async handleOwnershipDispute(
    disputeId: string,
    details: OwnershipDisputeDetails
  ): Promise<DisputeRecord> {
    const dispute = await this.disputeManager.getDispute(disputeId);

    // 소유권 분석
    const ownershipAnalysis = await this.analyzeOwnership(details);

    dispute.analysis = {
      type: 'ownership',
      typeKr: '소유권 분쟁',
      specimenType: details.specimenType,
      specimenTypeKr: this.getSpecimenTypeKr(details.specimenType),
      claimants: details.claimants,
      legalBasis: ownershipAnalysis.legalBasis,
      recommendation: ownershipAnalysis.recommendation,
    };

    await this.disputeManager.storage.save(dispute);

    return dispute;
  }

  private async analyzeOwnership(details: OwnershipDisputeDetails): Promise<{
    legalBasis: string;
    recommendation: string;
  }> {
    // 소유권 분석 - 한국법 기준
    if (details.specimenType === 'embryo') {
      return {
        legalBasis: '생명윤리법 제25조 (배아의 보관 및 폐기)',
        recommendation: '양측 동의권자의 합의가 필요합니다',
      };
    }

    return {
      legalBasis: '민법 제98조 (물건의 정의)',
      recommendation: '계약 조건에 따라 판단 필요',
    };
  }

  private getSpecimenTypeKr(type: string): string {
    const types: Record<string, string> = {
      'embryo': '배아',
      'sperm': '정자',
      'oocyte': '난자',
      'tissue': '조직',
      'cord-blood': '제대혈',
    };
    return types[type] || type;
  }

  // ============================================================================
  // 과실 분쟁 처리
  // ============================================================================

  async handleNegligenceDispute(
    disputeId: string,
    details: NegligenceDisputeDetails
  ): Promise<DisputeRecord> {
    const dispute = await this.disputeManager.getDispute(disputeId);

    // 과실 분석
    const negligenceAnalysis = await this.analyzeNegligence(details);

    // 기술 전문가 의견 요청
    if (negligenceAnalysis.requiresTechnicalExpert) {
      await this.requestExpertOpinion(dispute, 'cryotechnology');
    }

    dispute.analysis = {
      type: 'negligence',
      typeKr: '과실 분쟁',
      allegation: details.allegation,
      allegationKr: this.getAllegationKr(details.allegation),
      standardOfCare: negligenceAnalysis.standardOfCare,
      breach: negligenceAnalysis.breach,
      causation: negligenceAnalysis.causation,
      damages: negligenceAnalysis.damages,
    };

    await this.disputeManager.storage.save(dispute);

    return dispute;
  }

  private async analyzeNegligence(details: NegligenceDisputeDetails): Promise<{
    standardOfCare: string;
    breach: boolean;
    causation: string;
    damages: number;
    requiresTechnicalExpert: boolean;
  }> {
    return {
      standardOfCare: '의료법 및 관련 기준에 따른 주의 의무',
      breach: false,
      causation: '인과관계 분석 필요',
      damages: 0,
      requiresTechnicalExpert: true,
    };
  }

  private getAllegationKr(allegation: string): string {
    const allegations: Record<string, string> = {
      'equipment-failure': '장비 고장',
      'human-error': '인적 오류',
      'protocol-breach': '프로토콜 위반',
      'monitoring-failure': '모니터링 실패',
      'documentation-error': '문서 오류',
    };
    return allegations[allegation] || allegation;
  }

  // ============================================================================
  // 손해 산정
  // ============================================================================

  private async calculateDamages(details: ContractDisputeDetails): Promise<DamageAssessment> {
    return {
      directDamages: details.claimedAmount || 0,
      consequentialDamages: 0,
      punitiveDamages: 0,
      totalClaimed: details.claimedAmount || 0,
      currency: 'KRW',
      basis: '계약상 손해배상 조항',
    };
  }

  private async requestExpertOpinion(
    dispute: DisputeRecord,
    expertType: string
  ): Promise<void> {
    const expert = await this.expertService.assignExpert(dispute.id, expertType);

    dispute.experts = dispute.experts || [];
    dispute.experts.push({
      type: expertType,
      typeKr: this.getExpertTypeKr(expertType),
      name: expert.name,
      assignedAt: new Date().toISOString(),
      status: 'assigned',
      statusKr: '배정됨',
    });
  }

  private getExpertTypeKr(type: string): string {
    const types: Record<string, string> = {
      'cryobiology': '극저온생물학 전문가',
      'cryotechnology': '극저온기술 전문가',
      'medical': '의료 전문가',
      'legal': '법률 전문가',
      'ethics': '생명윤리 전문가',
    };
    return types[type] || type;
  }
}

// ============================================================================
// 인터페이스 정의
// ============================================================================

export interface ContractDisputeDetails {
  breachType: string;
  claimedAmount?: number;
  evidence: string[];
}

export interface ConsentDisputeDetails {
  issue: string;
  consentDate: string;
  disputedTerms: string[];
}

export interface OwnershipDisputeDetails {
  specimenType: string;
  claimants: string[];
  basis: string;
}

export interface NegligenceDisputeDetails {
  allegation: string;
  incidentDate: string;
  damages: string;
}

export interface DamageAssessment {
  directDamages: number;
  consequentialDamages: number;
  punitiveDamages: number;
  totalClaimed: number;
  currency: string;
  basis: string;
}

export interface ExpertWitnessService {
  assignExpert(disputeId: string, expertType: string): Promise<{ name: string; id: string }>;
}
```

## 분쟁 분석

```typescript
/**
 * 분쟁 분석 및 보고
 * 패턴, 결과 및 효율성 추적
 */

export class DisputeAnalytics {
  constructor(private readonly storage: DisputeStorage) {}

  async generateReport(
    dateRange: { from: string; to: string }
  ): Promise<DisputeAnalyticsReport> {
    const disputes = await this.storage.getDisputesInRange(dateRange);

    return {
      summary: this.calculateSummary(disputes),
      byType: this.analyzeByType(disputes),
      byMechanism: this.analyzeByMechanism(disputes),
      byOutcome: this.analyzeByOutcome(disputes),
      resolutionTime: this.analyzeResolutionTime(disputes),
      trends: this.analyzeTrends(disputes),
      recommendations: this.generateRecommendations(disputes),
    };
  }

  private calculateSummary(disputes: DisputeRecord[]): DisputeSummary {
    const resolved = disputes.filter(d => d.status === 'resolved');
    const dismissed = disputes.filter(d => d.status === 'dismissed');
    const active = disputes.filter(d =>
      !['resolved', 'dismissed'].includes(d.status)
    );

    return {
      total: disputes.length,
      totalKr: `총 ${disputes.length}건`,
      resolved: resolved.length,
      resolvedKr: `해결 ${resolved.length}건`,
      dismissed: dismissed.length,
      dismissedKr: `기각 ${dismissed.length}건`,
      active: active.length,
      activeKr: `진행 중 ${active.length}건`,
      resolutionRate: disputes.length > 0
        ? (resolved.length / disputes.length) * 100
        : 0,
      averageResolutionDays: this.calculateAverageResolutionDays(resolved),
    };
  }

  private analyzeByType(disputes: DisputeRecord[]): Record<DisputeType, TypeAnalysis> {
    const types: DisputeType[] = ['contract', 'consent', 'ownership', 'negligence', 'regulatory'];
    const analysis: Record<string, TypeAnalysis> = {};

    for (const type of types) {
      const typeDisputes = disputes.filter(d => d.type === type);

      analysis[type] = {
        type,
        typeKr: this.getTypeKr(type),
        count: typeDisputes.length,
        percentage: disputes.length > 0
          ? (typeDisputes.length / disputes.length) * 100
          : 0,
        averageResolutionDays: this.calculateAverageResolutionDays(
          typeDisputes.filter(d => d.status === 'resolved')
        ),
        resolutionRate: typeDisputes.length > 0
          ? (typeDisputes.filter(d => d.status === 'resolved').length / typeDisputes.length) * 100
          : 0,
      };
    }

    return analysis as Record<DisputeType, TypeAnalysis>;
  }

  private getTypeKr(type: DisputeType): string {
    const typeMap: Record<DisputeType, string> = {
      'contract': '계약',
      'consent': '동의',
      'ownership': '소유권',
      'negligence': '과실',
      'regulatory': '규제',
    };
    return typeMap[type];
  }

  private analyzeByMechanism(disputes: DisputeRecord[]): Record<ResolutionType, MechanismAnalysis> {
    const mechanisms: ResolutionType[] = ['negotiation', 'mediation', 'arbitration', 'litigation'];
    const analysis: Record<string, MechanismAnalysis> = {};

    const resolved = disputes.filter(d => d.resolution);

    for (const mech of mechanisms) {
      const mechDisputes = resolved.filter(d => d.resolution?.mechanism === mech);

      analysis[mech] = {
        mechanism: mech,
        mechanismKr: this.getMechanismKr(mech),
        resolvedCount: mechDisputes.length,
        percentage: resolved.length > 0
          ? (mechDisputes.length / resolved.length) * 100
          : 0,
        averageDays: this.calculateAverageResolutionDays(mechDisputes),
        successRate: this.calculateSuccessRate(mechDisputes, mech),
      };
    }

    return analysis as Record<ResolutionType, MechanismAnalysis>;
  }

  private getMechanismKr(mechanism: ResolutionType): string {
    const mechMap: Record<ResolutionType, string> = {
      'negotiation': '협상',
      'mediation': '조정',
      'arbitration': '중재',
      'litigation': '소송',
    };
    return mechMap[mechanism];
  }

  private analyzeByOutcome(disputes: DisputeRecord[]): OutcomeAnalysis {
    const resolved = disputes.filter(d => d.resolution);

    return {
      settlement: resolved.filter(d =>
        d.resolution?.outcome.includes('합의')
      ).length,
      settlementKr: '합의',
      award: resolved.filter(d =>
        d.resolution?.mechanism === 'arbitration'
      ).length,
      awardKr: '중재 판정',
      judgment: resolved.filter(d =>
        d.resolution?.mechanism === 'litigation'
      ).length,
      judgmentKr: '법원 판결',
      dismissed: disputes.filter(d => d.status === 'dismissed').length,
      dismissedKr: '기각',
    };
  }

  private analyzeResolutionTime(disputes: DisputeRecord[]): ResolutionTimeAnalysis {
    const resolved = disputes.filter(d => d.resolution);

    const times = resolved.map(d => this.calculateDays(d.filedDate, d.resolution!.date));

    return {
      average: this.average(times),
      averageKr: `평균 ${Math.round(this.average(times))}일`,
      median: this.median(times),
      medianKr: `중앙값 ${Math.round(this.median(times))}일`,
      fastest: Math.min(...times),
      fastestKr: `최단 ${Math.min(...times)}일`,
      slowest: Math.max(...times),
      slowestKr: `최장 ${Math.max(...times)}일`,
      distribution: {
        under30: times.filter(t => t < 30).length,
        '30to60': times.filter(t => t >= 30 && t < 60).length,
        '60to90': times.filter(t => t >= 60 && t < 90).length,
        over90: times.filter(t => t >= 90).length,
      },
      distributionKr: {
        '30일 미만': times.filter(t => t < 30).length,
        '30-60일': times.filter(t => t >= 30 && t < 60).length,
        '60-90일': times.filter(t => t >= 60 && t < 90).length,
        '90일 이상': times.filter(t => t >= 90).length,
      },
    };
  }

  private analyzeTrends(disputes: DisputeRecord[]): TrendAnalysis {
    const byMonth = this.groupByMonth(disputes);

    return {
      monthlyVolume: byMonth.map(m => ({
        month: m.month,
        count: m.disputes.length,
      })),
      typesTrending: this.identifyTrendingTypes(byMonth),
      typesTrendingKr: ['계약', '동의'],
      seasonality: this.detectSeasonality(byMonth),
      seasonalityKr: '뚜렷한 계절성 없음',
    };
  }

  private generateRecommendations(disputes: DisputeRecord[]): Recommendation[] {
    const recommendations: Recommendation[] = [];

    const summary = this.calculateSummary(disputes);
    const byMechanism = this.analyzeByMechanism(disputes);
    const byType = this.analyzeByType(disputes);

    // 협상 성공률 확인
    if (byMechanism.negotiation.successRate < 30) {
      recommendations.push({
        category: 'process',
        categoryKr: '프로세스',
        title: '협상 효율성 개선 필요',
        description: '협상 성공률이 낮습니다. 조기 개입 전략을 고려하세요.',
        priority: 'high',
        priorityKr: '높음',
      });
    }

    // 계약 분쟁 비율 확인
    if ((byType.contract?.percentage || 0) > 40) {
      recommendations.push({
        category: 'prevention',
        categoryKr: '예방',
        title: '계약서 검토 필요',
        description: '계약 분쟁이 많습니다. 계약 템플릿과 교육을 검토하세요.',
        priority: 'medium',
        priorityKr: '중간',
      });
    }

    // 해결 시간 확인
    if (summary.averageResolutionDays > 90) {
      recommendations.push({
        category: 'efficiency',
        categoryKr: '효율성',
        title: '프로세스 개선 권장',
        description: '평균 해결 시간이 90일을 초과합니다. 프로세스 개선을 고려하세요.',
        priority: 'high',
        priorityKr: '높음',
      });
    }

    return recommendations;
  }

  private calculateAverageResolutionDays(disputes: DisputeRecord[]): number {
    if (disputes.length === 0) return 0;

    const days = disputes
      .filter(d => d.resolution)
      .map(d => this.calculateDays(d.filedDate, d.resolution!.date));

    return this.average(days);
  }

  private calculateDays(from: string, to: string): number {
    const fromDate = new Date(from);
    const toDate = new Date(to);
    return Math.ceil((toDate.getTime() - fromDate.getTime()) / (1000 * 60 * 60 * 24));
  }

  private calculateSuccessRate(disputes: DisputeRecord[], mechanism: string): number {
    if (disputes.length === 0) return 0;

    const successful = disputes.filter(d =>
      d.resolution?.outcome.toLowerCase().includes('합의') ||
      d.resolution?.outcome.toLowerCase().includes('해결')
    );

    return (successful.length / disputes.length) * 100;
  }

  private average(numbers: number[]): number {
    if (numbers.length === 0) return 0;
    return numbers.reduce((a, b) => a + b, 0) / numbers.length;
  }

  private median(numbers: number[]): number {
    if (numbers.length === 0) return 0;
    const sorted = [...numbers].sort((a, b) => a - b);
    const mid = Math.floor(sorted.length / 2);
    return sorted.length % 2 !== 0
      ? sorted[mid]
      : (sorted[mid - 1] + sorted[mid]) / 2;
  }

  private groupByMonth(disputes: DisputeRecord[]): MonthGroup[] {
    const groups: Map<string, DisputeRecord[]> = new Map();

    for (const dispute of disputes) {
      const date = new Date(dispute.filedDate);
      const key = `${date.getFullYear()}-${String(date.getMonth() + 1).padStart(2, '0')}`;

      if (!groups.has(key)) {
        groups.set(key, []);
      }
      groups.get(key)!.push(dispute);
    }

    return Array.from(groups.entries())
      .map(([month, disputes]) => ({ month, disputes }))
      .sort((a, b) => a.month.localeCompare(b.month));
  }

  private identifyTrendingTypes(monthGroups: MonthGroup[]): string[] {
    return ['contract', 'consent'];
  }

  private detectSeasonality(monthGroups: MonthGroup[]): string {
    return 'No significant seasonality detected';
  }
}

// ============================================================================
// 타입 정의
// ============================================================================

export interface DisputeAnalyticsReport {
  summary: DisputeSummary;
  byType: Record<DisputeType, TypeAnalysis>;
  byMechanism: Record<ResolutionType, MechanismAnalysis>;
  byOutcome: OutcomeAnalysis;
  resolutionTime: ResolutionTimeAnalysis;
  trends: TrendAnalysis;
  recommendations: Recommendation[];
}

export interface DisputeSummary {
  total: number;
  totalKr: string;
  resolved: number;
  resolvedKr: string;
  dismissed: number;
  dismissedKr: string;
  active: number;
  activeKr: string;
  resolutionRate: number;
  averageResolutionDays: number;
}

export interface TypeAnalysis {
  type: DisputeType;
  typeKr: string;
  count: number;
  percentage: number;
  averageResolutionDays: number;
  resolutionRate: number;
}

export interface MechanismAnalysis {
  mechanism: ResolutionType;
  mechanismKr: string;
  resolvedCount: number;
  percentage: number;
  averageDays: number;
  successRate: number;
}

export interface OutcomeAnalysis {
  settlement: number;
  settlementKr: string;
  award: number;
  awardKr: string;
  judgment: number;
  judgmentKr: string;
  dismissed: number;
  dismissedKr: string;
}

export interface ResolutionTimeAnalysis {
  average: number;
  averageKr: string;
  median: number;
  medianKr: string;
  fastest: number;
  fastestKr: string;
  slowest: number;
  slowestKr: string;
  distribution: {
    under30: number;
    '30to60': number;
    '60to90': number;
    over90: number;
  };
  distributionKr: Record<string, number>;
}

export interface TrendAnalysis {
  monthlyVolume: { month: string; count: number }[];
  typesTrending: string[];
  typesTrendingKr: string[];
  seasonality: string;
  seasonalityKr: string;
}

export interface Recommendation {
  category: string;
  categoryKr: string;
  title: string;
  description: string;
  priority: 'high' | 'medium' | 'low';
  priorityKr: string;
}

interface MonthGroup {
  month: string;
  disputes: DisputeRecord[];
}
```

---

## 장 요약

이 장에서는 종합 분쟁 해결에 대해 다루었습니다:

- **분쟁 접수**: 검증이 포함된 구조화된 접수 절차
- **단계별 해결**: 협상 → 조정 → 중재 → 소송
- **에스컬레이션 관리**: 자동 및 수동 상향 조정
- **세션 관리**: 조정 및 중재 세션
- **분석**: 분쟁 패턴 및 개선 권고
- **한국 특화**: 대한상사중재원 연계, 한국법 기준 적용

---

**다음 장**: [보안 및 컴플라이언스 - 데이터 보호 및 감사](./07-security.md)
