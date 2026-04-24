# 제5장: 제어 프로토콜

## 동의 생명주기 관리

본 장에서는 동의의 전체 생명주기를 관리하기 위한 제어 프로토콜을 정의합니다. 상태 전환, 워크플로우 관리, 의사결정 실행 및 에스컬레이션 절차를 포함합니다.

---

## 5.1 동의 상태 머신

```typescript
// 동의 상태 머신 정의
interface ConsentStateMachine {
  states: ConsentStatus[];
  transitions: StateTransition[];
  guards: TransitionGuard[];
  actions: TransitionAction[];
}

const consentStateMachine: ConsentStateMachine = {
  states: [
    'DRAFT',
    'PENDING_WITNESS',
    'PENDING_NOTARIZATION',
    'PENDING_REVIEW',
    'ACTIVE',
    'SUSPENDED',
    'REVOKED',
    'EXPIRED',
    'SUPERSEDED',
  ],

  transitions: [
    // DRAFT에서
    { from: 'DRAFT', to: 'PENDING_WITNESS', event: 'SUBMIT_FOR_WITNESS' },
    { from: 'DRAFT', to: 'PENDING_REVIEW', event: 'SUBMIT_FOR_REVIEW' },
    { from: 'DRAFT', to: 'ACTIVE', event: 'ACTIVATE_DIRECTLY' },

    // PENDING_WITNESS에서
    { from: 'PENDING_WITNESS', to: 'PENDING_NOTARIZATION', event: 'WITNESS_COMPLETE' },
    { from: 'PENDING_WITNESS', to: 'PENDING_REVIEW', event: 'SKIP_NOTARIZATION' },
    { from: 'PENDING_WITNESS', to: 'DRAFT', event: 'RETURN_TO_DRAFT' },

    // PENDING_NOTARIZATION에서
    { from: 'PENDING_NOTARIZATION', to: 'PENDING_REVIEW', event: 'NOTARIZATION_COMPLETE' },
    { from: 'PENDING_NOTARIZATION', to: 'DRAFT', event: 'RETURN_TO_DRAFT' },

    // PENDING_REVIEW에서
    { from: 'PENDING_REVIEW', to: 'ACTIVE', event: 'APPROVE' },
    { from: 'PENDING_REVIEW', to: 'DRAFT', event: 'REQUEST_CHANGES' },

    // ACTIVE에서
    { from: 'ACTIVE', to: 'SUSPENDED', event: 'SUSPEND' },
    { from: 'ACTIVE', to: 'REVOKED', event: 'REVOKE' },
    { from: 'ACTIVE', to: 'EXPIRED', event: 'EXPIRE' },
    { from: 'ACTIVE', to: 'SUPERSEDED', event: 'SUPERSEDE' },

    // SUSPENDED에서
    { from: 'SUSPENDED', to: 'ACTIVE', event: 'REACTIVATE' },
    { from: 'SUSPENDED', to: 'REVOKED', event: 'REVOKE' },

    // 종료 상태 - 전환 없음
  ],

  guards: [
    {
      transition: { from: 'DRAFT', to: 'ACTIVE', event: 'ACTIVATE_DIRECTLY' },
      condition: 'hasValidCapacityAssessment && hasRequiredDecisions',
    },
    {
      transition: { from: 'PENDING_WITNESS', to: 'PENDING_NOTARIZATION', event: 'WITNESS_COMPLETE' },
      condition: 'hasMinimumWitnesses && allWitnessesAttested',
    },
    {
      transition: { from: 'ACTIVE', to: 'REVOKED', event: 'REVOKE' },
      condition: 'hasRevocationAuthority && notInProtectedPeriod',
    },
  ],

  actions: [
    {
      on: 'ACTIVATE',
      actions: ['setEffectiveDate', 'notifyStakeholders', 'scheduleReviews'],
    },
    {
      on: 'REVOKE',
      actions: ['recordRevocation', 'notifyStakeholders', 'archiveConsent'],
    },
    {
      on: 'EXPIRE',
      actions: ['setExpirationDate', 'notifyStakeholders', 'initiateRenewalProcess'],
    },
  ],
};

// 상태 머신 구현
class ConsentStateManager {
  private stateMachine: ConsentStateMachine;
  private consentRepository: ConsentRepository;
  private eventBus: EventBus;

  constructor(
    stateMachine: ConsentStateMachine,
    consentRepository: ConsentRepository,
    eventBus: EventBus
  ) {
    this.stateMachine = stateMachine;
    this.consentRepository = consentRepository;
    this.eventBus = eventBus;
  }

  async transition(
    consentId: string,
    event: string,
    context: TransitionContext
  ): Promise<TransitionResult> {
    // 현재 동의 조회
    const consent = await this.consentRepository.findById(consentId);
    if (!consent) {
      throw new NotFoundError(`동의 ${consentId}를 찾을 수 없습니다`);
    }

    const currentState = consent.validity.status;

    // 유효한 전환 찾기
    const transition = this.findTransition(currentState, event);
    if (!transition) {
      throw new InvalidTransitionError(
        `${currentState}에서 ${event} 이벤트로의 전환이 없습니다`
      );
    }

    // 가드 검사
    const guardResult = await this.evaluateGuards(transition, consent, context);
    if (!guardResult.passed) {
      throw new GuardFailedError(
        `전환 가드 실패: ${guardResult.failedGuard}`,
        guardResult.reason
      );
    }

    // 전환 전 액션 실행
    await this.executePreActions(transition, consent, context);

    // 상태 업데이트
    const updated = await this.consentRepository.updateStatus(consentId, {
      status: transition.to,
      statusChange: {
        fromStatus: currentState,
        toStatus: transition.to,
        changeDate: new Date(),
        changedBy: context.userId,
        reason: context.reason,
      },
    });

    // 전환 후 액션 실행
    await this.executePostActions(transition, updated, context);

    // 이벤트 발생
    await this.eventBus.emit({
      type: 'CONSENT_STATE_CHANGED',
      consentId,
      fromState: currentState,
      toState: transition.to,
      event,
      context,
      timestamp: new Date(),
    });

    return {
      success: true,
      previousState: currentState,
      newState: transition.to,
      consent: updated,
    };
  }

  private findTransition(
    currentState: ConsentStatus,
    event: string
  ): StateTransition | undefined {
    return this.stateMachine.transitions.find(
      t => t.from === currentState && t.event === event
    );
  }

  private async evaluateGuards(
    transition: StateTransition,
    consent: ConsentRecord,
    context: TransitionContext
  ): Promise<GuardEvaluationResult> {
    const applicableGuards = this.stateMachine.guards.filter(
      g =>
        g.transition.from === transition.from &&
        g.transition.to === transition.to &&
        g.transition.event === transition.event
    );

    for (const guard of applicableGuards) {
      const result = await this.evaluateGuard(guard, consent, context);
      if (!result.passed) {
        return {
          passed: false,
          failedGuard: guard.condition,
          reason: result.reason,
        };
      }
    }

    return { passed: true };
  }

  private async evaluateGuard(
    guard: TransitionGuard,
    consent: ConsentRecord,
    context: TransitionContext
  ): Promise<{ passed: boolean; reason?: string }> {
    // 가드 조건 파싱 및 평가
    const conditions = guard.condition.split(' && ');

    for (const condition of conditions) {
      const result = await this.evaluateCondition(condition.trim(), consent, context);
      if (!result.passed) {
        return { passed: false, reason: result.reason };
      }
    }

    return { passed: true };
  }

  private async evaluateCondition(
    condition: string,
    consent: ConsentRecord,
    context: TransitionContext
  ): Promise<{ passed: boolean; reason?: string }> {
    switch (condition) {
      case 'hasValidCapacityAssessment':
        const hasCapacity = consent.authority.capacityAssessment?.hasCapacity === true;
        return {
          passed: hasCapacity,
          reason: hasCapacity ? undefined : '유효한 의사결정능력 평가가 필요합니다',
        };

      case 'hasRequiredDecisions':
        const hasDecisions = consent.decisions.length > 0;
        return {
          passed: hasDecisions,
          reason: hasDecisions ? undefined : '최소 하나의 의사결정이 필요합니다',
        };

      case 'hasMinimumWitnesses':
        const witnessCount = consent.authority.witnesses?.length || 0;
        const minWitnesses = 2; // 설정 가능
        return {
          passed: witnessCount >= minWitnesses,
          reason:
            witnessCount >= minWitnesses
              ? undefined
              : `최소 ${minWitnesses}명의 증인이 필요합니다`,
        };

      case 'allWitnessesAttested':
        const allAttested =
          consent.authority.witnesses?.every(
            w => w.attestation.attestationDate
          ) ?? false;
        return {
          passed: allAttested,
          reason: allAttested ? undefined : '모든 증인이 증언해야 합니다',
        };

      case 'hasRevocationAuthority':
        return this.checkRevocationAuthority(consent, context);

      case 'notInProtectedPeriod':
        return this.checkNotInProtectedPeriod(consent);

      default:
        // 알 수 없는 조건 - 안전하게 실패
        return { passed: false, reason: `알 수 없는 조건: ${condition}` };
    }
  }

  private async checkRevocationAuthority(
    consent: ConsentRecord,
    context: TransitionContext
  ): Promise<{ passed: boolean; reason?: string }> {
    // 환자는 항상 자신의 동의를 철회할 수 있음
    if (context.userId === consent.patientId) {
      return { passed: true };
    }

    // 권한 있는 대리인인지 확인
    const isAuthorizedProxy = await this.isAuthorizedProxy(
      context.userId,
      consent.patientId,
      'REVOCATION'
    );
    if (isAuthorizedProxy) {
      return { passed: true };
    }

    // 법적 권한인지 확인
    if (context.legalAuthority) {
      return { passed: true };
    }

    return {
      passed: false,
      reason: '사용자가 이 동의를 철회할 권한이 없습니다',
    };
  }

  private checkNotInProtectedPeriod(
    consent: ConsentRecord
  ): { passed: boolean; reason?: string } {
    // 일부 동의는 활성화 후 보호 기간이 있을 수 있음
    const protectedPeriodDays = 30; // 설정 가능
    const activationDate = consent.validity.effectiveDate;
    const protectedUntil = new Date(activationDate);
    protectedUntil.setDate(protectedUntil.getDate() + protectedPeriodDays);

    if (new Date() < protectedUntil) {
      return {
        passed: false,
        reason: `동의가 ${protectedUntil.toISOString()}까지 보호 기간 중입니다`,
      };
    }

    return { passed: true };
  }
}

interface StateTransition {
  from: ConsentStatus;
  to: ConsentStatus;
  event: string;
}

interface TransitionGuard {
  transition: StateTransition;
  condition: string;
}

interface TransitionAction {
  on: string;
  actions: string[];
}

interface TransitionContext {
  userId: string;
  reason: string;
  legalAuthority?: boolean;
  metadata?: Record<string, any>;
}

interface TransitionResult {
  success: boolean;
  previousState: ConsentStatus;
  newState: ConsentStatus;
  consent: ConsentRecord;
}

interface GuardEvaluationResult {
  passed: boolean;
  failedGuard?: string;
  reason?: string;
}
```

---

## 5.2 워크플로우 엔진

```typescript
// 동의 워크플로우 정의
interface ConsentWorkflow {
  workflowId: string;
  name: string;
  description: string;
  category: ConsentCategory;

  steps: WorkflowStep[];
  triggers: WorkflowTrigger[];
  escalations: EscalationRule[];
}

interface WorkflowStep {
  stepId: string;
  name: string;
  type: WorkflowStepType;

  // 실행
  handler: string;
  config: Record<string, any>;

  // 제어 흐름
  next: WorkflowNext[];
  timeout?: string;        // ISO 8601 기간
  retryPolicy?: RetryPolicy;

  // 요구 사항
  requiredApprovals?: ApprovalRequirement[];
  requiredDocuments?: string[];
}

enum WorkflowStepType {
  MANUAL = 'MANUAL',
  AUTOMATIC = 'AUTOMATIC',
  APPROVAL = 'APPROVAL',
  NOTIFICATION = 'NOTIFICATION',
  VALIDATION = 'VALIDATION',
  INTEGRATION = 'INTEGRATION',
  DECISION = 'DECISION',
  WAIT = 'WAIT',
}

interface WorkflowNext {
  condition?: string;
  targetStep: string;
  default?: boolean;
}

// 워크플로우 엔진 구현
class ConsentWorkflowEngine {
  private workflows: Map<string, ConsentWorkflow> = new Map();
  private stepHandlers: Map<string, WorkflowStepHandler> = new Map();
  private executionRepository: WorkflowExecutionRepository;

  constructor(
    executionRepository: WorkflowExecutionRepository
  ) {
    this.executionRepository = executionRepository;
    this.registerDefaultHandlers();
    this.loadWorkflows();
  }

  private registerDefaultHandlers(): void {
    // 기본 단계 핸들러 등록
    this.stepHandlers.set('validateConsent', new ValidateConsentHandler());
    this.stepHandlers.set('collectWitnesses', new CollectWitnessesHandler());
    this.stepHandlers.set('notarizeDocument', new NotarizeDocumentHandler());
    this.stepHandlers.set('reviewConsent', new ReviewConsentHandler());
    this.stepHandlers.set('activateConsent', new ActivateConsentHandler());
    this.stepHandlers.set('sendNotification', new SendNotificationHandler());
    this.stepHandlers.set('scheduleReview', new ScheduleReviewHandler());
  }

  async startWorkflow(
    consentId: string,
    workflowId: string,
    context: WorkflowContext
  ): Promise<WorkflowExecution> {
    const workflow = this.workflows.get(workflowId);
    if (!workflow) {
      throw new NotFoundError(`워크플로우 ${workflowId}를 찾을 수 없습니다`);
    }

    // 실행 레코드 생성
    const execution: WorkflowExecution = {
      executionId: generateExecutionId(),
      workflowId,
      consentId,
      status: 'RUNNING',
      currentStep: workflow.steps[0].stepId,
      stepHistory: [],
      context,
      startedAt: new Date(),
    };

    await this.executionRepository.create(execution);

    // 첫 번째 단계 시작
    await this.executeStep(execution, workflow.steps[0]);

    return execution;
  }

  private async executeStep(
    execution: WorkflowExecution,
    step: WorkflowStep
  ): Promise<void> {
    // 실행 상태 업데이트
    execution.currentStep = step.stepId;
    await this.executionRepository.update(execution);

    // 단계 시작 기록
    const stepExecution: StepExecution = {
      stepId: step.stepId,
      status: 'RUNNING',
      startedAt: new Date(),
    };
    execution.stepHistory.push(stepExecution);

    try {
      // 핸들러 가져오기
      const handler = this.stepHandlers.get(step.handler);
      if (!handler) {
        throw new Error(`핸들러 ${step.handler}를 찾을 수 없습니다`);
      }

      // 단계 실행
      const result = await handler.execute(step, execution);

      // 단계 실행 업데이트
      stepExecution.status = 'COMPLETED';
      stepExecution.completedAt = new Date();
      stepExecution.result = result;

      await this.executionRepository.update(execution);

      // 다음 단계 결정
      const nextStep = this.determineNextStep(step, result, execution);

      if (nextStep) {
        await this.executeStep(execution, nextStep);
      } else {
        // 워크플로우 완료
        execution.status = 'COMPLETED';
        execution.completedAt = new Date();
        await this.executionRepository.update(execution);
      }
    } catch (error) {
      // 단계 실패 처리
      stepExecution.status = 'FAILED';
      stepExecution.error = error.message;
      stepExecution.completedAt = new Date();

      // 재시도 정책 확인
      if (step.retryPolicy && stepExecution.retryCount < step.retryPolicy.maxRetries) {
        stepExecution.retryCount = (stepExecution.retryCount || 0) + 1;
        const delay = this.calculateRetryDelay(step.retryPolicy, stepExecution.retryCount);
        await this.scheduleRetry(execution, step, delay);
      } else {
        // 에스컬레이션 또는 워크플로우 실패
        await this.handleStepFailure(execution, step, error);
      }
    }
  }

  private determineNextStep(
    currentStep: WorkflowStep,
    result: StepResult,
    execution: WorkflowExecution
  ): WorkflowStep | undefined {
    const workflow = this.workflows.get(execution.workflowId)!;

    // 조건 평가하여 다음 단계 찾기
    for (const next of currentStep.next) {
      if (next.condition) {
        const conditionMet = this.evaluateCondition(next.condition, result, execution);
        if (conditionMet) {
          return workflow.steps.find(s => s.stepId === next.targetStep);
        }
      } else if (next.default) {
        return workflow.steps.find(s => s.stepId === next.targetStep);
      }
    }

    // 다음 단계 없음 - 워크플로우 종료
    return undefined;
  }

  private evaluateCondition(
    condition: string,
    result: StepResult,
    execution: WorkflowExecution
  ): boolean {
    // 간단한 조건 평가
    // 프로덕션에서는 적절한 표현식 평가기 사용
    const context = {
      result,
      execution,
      stepHistory: execution.stepHistory,
    };

    try {
      // 안전한 평가 (프로덕션에서는 적절한 샌드박스 사용)
      return eval(condition);
    } catch {
      return false;
    }
  }

  private async handleStepFailure(
    execution: WorkflowExecution,
    step: WorkflowStep,
    error: Error
  ): Promise<void> {
    const workflow = this.workflows.get(execution.workflowId)!;

    // 에스컬레이션 규칙 확인
    const escalation = workflow.escalations.find(
      e => e.triggerStep === step.stepId || e.triggerStep === '*'
    );

    if (escalation) {
      await this.executeEscalation(execution, escalation, error);
    } else {
      // 에스컬레이션 없음 - 워크플로우 실패
      execution.status = 'FAILED';
      execution.error = error.message;
      execution.completedAt = new Date();
      await this.executionRepository.update(execution);
    }
  }

  private async executeEscalation(
    execution: WorkflowExecution,
    escalation: EscalationRule,
    error: Error
  ): Promise<void> {
    // 에스컬레이션 알림 전송
    for (const target of escalation.notifyTargets) {
      await this.notificationService.sendEscalation({
        executionId: execution.executionId,
        consentId: execution.consentId,
        error: error.message,
        target,
        escalationLevel: escalation.level,
      });
    }

    // 실행 상태 업데이트
    execution.status = 'ESCALATED';
    execution.escalation = {
      escalatedAt: new Date(),
      level: escalation.level,
      reason: error.message,
    };
    await this.executionRepository.update(execution);
  }
}

interface WorkflowExecution {
  executionId: string;
  workflowId: string;
  consentId: string;
  status: 'RUNNING' | 'PAUSED' | 'COMPLETED' | 'FAILED' | 'ESCALATED';
  currentStep: string;
  stepHistory: StepExecution[];
  context: WorkflowContext;
  startedAt: Date;
  completedAt?: Date;
  error?: string;
  escalation?: EscalationInfo;
}

interface StepExecution {
  stepId: string;
  status: 'RUNNING' | 'COMPLETED' | 'FAILED' | 'SKIPPED';
  startedAt: Date;
  completedAt?: Date;
  result?: StepResult;
  error?: string;
  retryCount?: number;
}

interface StepResult {
  success: boolean;
  data?: Record<string, any>;
  nextStep?: string;
}

interface WorkflowContext {
  userId: string;
  patientId: string;
  initiatedBy: string;
  metadata?: Record<string, any>;
}

interface EscalationRule {
  triggerStep: string;
  level: number;
  notifyTargets: string[];
  timeout?: string;
}

// 단계 핸들러 인터페이스
interface WorkflowStepHandler {
  execute(step: WorkflowStep, execution: WorkflowExecution): Promise<StepResult>;
}

// 예제 단계 핸들러
class ValidateConsentHandler implements WorkflowStepHandler {
  async execute(step: WorkflowStep, execution: WorkflowExecution): Promise<StepResult> {
    const consent = await getConsent(execution.consentId);

    const validationResult = await validateConsent(consent);

    if (!validationResult.valid) {
      return {
        success: false,
        data: { errors: validationResult.errors },
        nextStep: 'handleValidationFailure',
      };
    }

    return {
      success: true,
      data: { validatedAt: new Date() },
    };
  }
}

class CollectWitnessesHandler implements WorkflowStepHandler {
  async execute(step: WorkflowStep, execution: WorkflowExecution): Promise<StepResult> {
    const consent = await getConsent(execution.consentId);
    const requiredWitnesses = step.config.requiredWitnesses || 2;

    const currentWitnesses = consent.authority.witnesses?.length || 0;

    if (currentWitnesses < requiredWitnesses) {
      // 더 많은 증인 필요 - 대기
      return {
        success: false,
        data: {
          required: requiredWitnesses,
          current: currentWitnesses,
          waiting: true,
        },
      };
    }

    // 모든 증인 증언 확인
    const allAttested = consent.authority.witnesses!.every(
      w => w.attestation.attestationDate
    );

    if (!allAttested) {
      return {
        success: false,
        data: { waiting: true, reason: '증인 증언 대기 중' },
      };
    }

    return {
      success: true,
      data: { witnessCount: currentWitnesses },
    };
  }
}

class ActivateConsentHandler implements WorkflowStepHandler {
  async execute(step: WorkflowStep, execution: WorkflowExecution): Promise<StepResult> {
    const consent = await getConsent(execution.consentId);

    // 동의 상태를 활성으로 업데이트
    await updateConsentStatus(consent.id, {
      status: 'ACTIVE',
      effectiveDate: new Date(),
    });

    // 검토 일정 예약
    if (step.config.scheduleReview) {
      await scheduleConsentReview(consent.id, {
        frequency: step.config.reviewFrequency || 'P1Y',
        firstReview: calculateFirstReviewDate(step.config.reviewFrequency || 'P1Y'),
      });
    }

    // 이해관계자에게 알림
    await notifyConsentActivated(consent);

    return {
      success: true,
      data: {
        activatedAt: new Date(),
        reviewScheduled: step.config.scheduleReview,
      },
    };
  }
}
```

---

## 5.3 의사결정 실행 프로토콜

```typescript
// 의사결정 실행 프로토콜
class DecisionExecutionService {
  private consentService: CryoConsentManagementService;
  private proxyService: ProxyManagementService;
  private auditService: AuditService;
  private notificationService: NotificationService;

  constructor(
    consentService: CryoConsentManagementService,
    proxyService: ProxyManagementService,
    auditService: AuditService,
    notificationService: NotificationService
  ) {
    this.consentService = consentService;
    this.proxyService = proxyService;
    this.auditService = auditService;
    this.notificationService = notificationService;
  }

  // 동의에 기반한 의사결정 실행
  async executeDecision(
    request: DecisionExecutionRequest
  ): Promise<DecisionExecutionResult> {
    // 1단계: 적용 가능한 동의 찾기
    const effectiveConsent = await this.consentService.getEffectiveConsent(
      request.patientId,
      request.decisionType,
      request.context
    );

    // 2단계: 권한 결정
    const authority = await this.determineAuthority(
      request,
      effectiveConsent
    );

    // 3단계: 실행 조건 검증
    const validationResult = await this.validateExecution(
      request,
      effectiveConsent,
      authority
    );

    if (!validationResult.valid) {
      return {
        executed: false,
        reason: 'VALIDATION_FAILED',
        details: validationResult.errors,
        requiresEscalation: validationResult.requiresEscalation,
      };
    }

    // 4단계: 의사결정 실행
    const execution = await this.performExecution(request, effectiveConsent, authority);

    // 5단계: 실행 기록
    await this.recordExecution(request, effectiveConsent, authority, execution);

    // 6단계: 이해관계자에게 알림
    await this.notifyExecution(request, execution);

    return {
      executed: true,
      execution,
      consentUsed: effectiveConsent.consent?.id,
      authority,
      confidence: effectiveConsent.confidence,
    };
  }

  private async determineAuthority(
    request: DecisionExecutionRequest,
    effectiveConsent: EffectiveConsent
  ): Promise<DecisionAuthority> {
    // 환자가 직접 의사결정하는 경우
    if (request.requestedBy.type === 'PATIENT' &&
        request.requestedBy.id === request.patientId) {
      return {
        type: 'PATIENT_DIRECT',
        entityId: request.patientId,
        basis: 'PATIENT_REQUEST',
      };
    }

    // 명시적 동의가 있는 경우
    if (effectiveConsent.found && effectiveConsent.confidence >= 0.8) {
      return {
        type: 'EXPLICIT_CONSENT',
        entityId: request.patientId,
        consentId: effectiveConsent.consent!.id,
        basis: 'DOCUMENTED_CONSENT',
      };
    }

    // 대리인이 요청하는 경우
    if (request.requestedBy.type === 'PROXY') {
      const proxyAuthority = await this.proxyService.validateProxyAuthority(
        request.requestedBy.id,
        request.patientId,
        request.decisionType
      );

      if (proxyAuthority.valid) {
        return {
          type: 'PROXY_DECISION',
          entityId: request.requestedBy.id,
          patientId: request.patientId,
          basis: 'PROXY_AUTHORITY',
          proxyDesignation: proxyAuthority.designation,
        };
      }
    }

    // 에스컬레이션 필요
    return {
      type: 'ESCALATION_REQUIRED',
      basis: 'INSUFFICIENT_AUTHORITY',
      escalationReason: '이 의사결정에 대한 유효한 권한을 찾을 수 없습니다',
    };
  }

  private async validateExecution(
    request: DecisionExecutionRequest,
    effectiveConsent: EffectiveConsent,
    authority: DecisionAuthority
  ): Promise<ExecutionValidationResult> {
    const errors: string[] = [];

    // 권한 유효성 확인
    if (authority.type === 'ESCALATION_REQUIRED') {
      return {
        valid: false,
        errors: ['이 의사결정을 실행할 유효한 권한이 없습니다'],
        requiresEscalation: true,
      };
    }

    // 동의 조건 확인
    if (effectiveConsent.consent) {
      const conditionResults = await this.evaluateConsentConditions(
        effectiveConsent.consent,
        request.context
      );

      if (!conditionResults.allMet) {
        errors.push(...conditionResults.unmetConditions);
      }
    }

    // 범위 제한 확인
    if (effectiveConsent.consent?.scope.limitations) {
      for (const limitation of effectiveConsent.consent.scope.limitations) {
        if (this.limitationApplies(limitation, request)) {
          if (limitation.hardLimit) {
            errors.push(`엄격한 제한 위반: ${limitation.description}`);
          } else {
            // 유연한 제한 - 경고
          }
        }
      }
    }

    // 시간 제약 확인
    if (request.timing) {
      const timingValid = await this.validateTiming(request.timing, effectiveConsent);
      if (!timingValid.valid) {
        errors.push(`시간 제약: ${timingValid.reason}`);
      }
    }

    return {
      valid: errors.length === 0,
      errors,
      requiresEscalation: false,
    };
  }

  private async performExecution(
    request: DecisionExecutionRequest,
    effectiveConsent: EffectiveConsent,
    authority: DecisionAuthority
  ): Promise<ExecutionRecord> {
    // 실행할 의사결정 가져오기
    const decision = effectiveConsent.decision;

    // 의사결정 유형에 따라 실행
    let executionResult: any;

    switch (decision?.decisionType) {
      case 'BINARY':
        executionResult = await this.executeBinaryDecision(request, decision);
        break;

      case 'CHOICE':
        executionResult = await this.executeChoiceDecision(request, decision);
        break;

      case 'THRESHOLD':
        executionResult = await this.executeThresholdDecision(request, decision);
        break;

      case 'CONDITIONAL':
        executionResult = await this.executeConditionalDecision(request, decision);
        break;

      case 'DELEGATION':
        executionResult = await this.executeDelegation(request, decision);
        break;

      default:
        throw new Error(`알 수 없는 의사결정 유형: ${decision?.decisionType}`);
    }

    return {
      executionId: generateExecutionId(),
      requestId: request.requestId,
      decisionType: request.decisionType,
      decision: decision?.answer,
      executedAt: new Date(),
      executedBy: authority.entityId,
      authorityType: authority.type,
      result: executionResult,
    };
  }

  private async executeBinaryDecision(
    request: DecisionExecutionRequest,
    decision: ConsentDecision
  ): Promise<BinaryExecutionResult> {
    const answer = decision.answer.binaryValue;

    if (answer === true) {
      // 요청된 작업 진행
      return {
        proceed: true,
        action: request.proposedAction,
      };
    } else {
      // 진행하지 않음
      return {
        proceed: false,
        reason: decision.reasoning || '환자 동의가 진행하지 않음을 나타냅니다',
      };
    }
  }

  private async executeChoiceDecision(
    request: DecisionExecutionRequest,
    decision: ConsentDecision
  ): Promise<ChoiceExecutionResult> {
    const selectedOption = decision.answer.selectedOption ||
                          decision.answer.selectedOptions?.[0];

    return {
      selectedOption,
      alternatives: decision.answer.selectedOptions || [],
      reasoning: decision.reasoning,
    };
  }

  private async executeThresholdDecision(
    request: DecisionExecutionRequest,
    decision: ConsentDecision
  ): Promise<ThresholdExecutionResult> {
    const threshold = decision.answer.thresholdValue!;
    const unit = decision.answer.thresholdUnit!;
    const operator = decision.answer.thresholdOperator!;

    // 요청 컨텍스트에서 현재 값 가져오기
    const currentValue = request.context.currentValue as number;

    // 임계값 평가
    let thresholdMet = false;
    switch (operator) {
      case 'GT':
        thresholdMet = currentValue > threshold;
        break;
      case 'GTE':
        thresholdMet = currentValue >= threshold;
        break;
      case 'LT':
        thresholdMet = currentValue < threshold;
        break;
      case 'LTE':
        thresholdMet = currentValue <= threshold;
        break;
      case 'EQ':
        thresholdMet = currentValue === threshold;
        break;
    }

    return {
      threshold,
      unit,
      operator,
      currentValue,
      thresholdMet,
      action: thresholdMet ? 'PROCEED' : 'HALT',
    };
  }

  private async executeConditionalDecision(
    request: DecisionExecutionRequest,
    decision: ConsentDecision
  ): Promise<ConditionalExecutionResult> {
    const rules = decision.answer.conditionalRules || [];

    // 우선순위 순서로 규칙 평가
    const sortedRules = [...rules].sort((a, b) => a.priority - b.priority);

    for (const rule of sortedRules) {
      const conditionMet = await this.evaluateConditionExpression(
        rule.condition,
        request.context
      );

      if (conditionMet) {
        return {
          ruleApplied: rule.ruleId,
          condition: rule.condition,
          action: rule.thenAction,
        };
      }
    }

    // 조건 충족 없음 - 기본값 존재 시 사용
    const defaultRule = sortedRules.find(r => r.elseAction);
    if (defaultRule) {
      return {
        ruleApplied: 'DEFAULT',
        action: defaultRule.elseAction!,
      };
    }

    return {
      ruleApplied: 'NONE',
      action: 'NO_ACTION',
      reason: '적용 가능한 규칙을 찾을 수 없습니다',
    };
  }

  private async executeDelegation(
    request: DecisionExecutionRequest,
    decision: ConsentDecision
  ): Promise<DelegationExecutionResult> {
    const delegateTo = decision.answer.delegateTo!;

    // 위임 대상 찾기
    let delegate: any;
    switch (delegateTo.targetType) {
      case 'INDIVIDUAL':
        delegate = await this.findIndividualDelegate(delegateTo.targetId);
        break;
      case 'ROLE':
        delegate = await this.findRoleDelegate(delegateTo.targetId, request.context);
        break;
      case 'COMMITTEE':
        delegate = await this.findCommitteeDelegate(delegateTo.targetId);
        break;
      case 'ORGANIZATION':
        delegate = await this.findOrganizationDelegate(delegateTo.targetId);
        break;
    }

    if (!delegate) {
      return {
        delegated: false,
        reason: '위임 대상을 찾을 수 없거나 불가용 상태입니다',
        fallbackRequired: true,
      };
    }

    // 위임 대상에게 의사결정 요청
    const delegateDecision = await this.requestDelegateDecision(
      delegate,
      request,
      delegateTo.scopeLimitations
    );

    return {
      delegated: true,
      delegateType: delegateTo.targetType,
      delegateId: delegateTo.targetId,
      delegateName: delegateTo.targetName,
      delegateDecision,
      requiresConfirmation: delegateTo.requiresConfirmation,
    };
  }

  private async recordExecution(
    request: DecisionExecutionRequest,
    effectiveConsent: EffectiveConsent,
    authority: DecisionAuthority,
    execution: ExecutionRecord
  ): Promise<void> {
    // 감사 레코드 생성
    await this.auditService.logDecisionExecution({
      executionId: execution.executionId,
      requestId: request.requestId,
      patientId: request.patientId,
      decisionType: request.decisionType,
      consentId: effectiveConsent.consent?.id,
      authorityType: authority.type,
      authorityId: authority.entityId,
      decision: execution.decision,
      result: execution.result,
      context: request.context,
      timestamp: execution.executedAt,
    });

    // 동의 사용 통계 업데이트
    if (effectiveConsent.consent) {
      await this.consentService.recordConsentUsage(effectiveConsent.consent.id, {
        usedFor: request.decisionType,
        usedAt: execution.executedAt,
        executionId: execution.executionId,
      });
    }
  }
}

interface DecisionExecutionRequest {
  requestId: string;
  patientId: string;
  decisionType: string;
  proposedAction: string;
  context: DecisionContext;
  requestedBy: {
    type: 'PATIENT' | 'PROXY' | 'STAFF' | 'SYSTEM';
    id: string;
  };
  timing?: {
    notBefore?: Date;
    notAfter?: Date;
    urgent?: boolean;
  };
}

interface DecisionExecutionResult {
  executed: boolean;
  reason?: string;
  details?: any;
  requiresEscalation?: boolean;
  execution?: ExecutionRecord;
  consentUsed?: string;
  authority?: DecisionAuthority;
  confidence?: number;
}

interface DecisionAuthority {
  type: 'PATIENT_DIRECT' | 'EXPLICIT_CONSENT' | 'PROXY_DECISION' | 'ESCALATION_REQUIRED';
  entityId?: string;
  patientId?: string;
  consentId?: string;
  basis: string;
  proxyDesignation?: ProxyDesignation;
  escalationReason?: string;
}

interface ExecutionRecord {
  executionId: string;
  requestId: string;
  decisionType: string;
  decision: any;
  executedAt: Date;
  executedBy: string;
  authorityType: string;
  result: any;
}
```

---

## 5.4 검토 및 갱신 프로토콜

```typescript
// 검토 일정 관리 및 관리
class ConsentReviewService {
  private consentRepository: ConsentRepository;
  private scheduler: SchedulerService;
  private notificationService: NotificationService;

  constructor(
    consentRepository: ConsentRepository,
    scheduler: SchedulerService,
    notificationService: NotificationService
  ) {
    this.consentRepository = consentRepository;
    this.scheduler = scheduler;
    this.notificationService = notificationService;
  }

  // 정기 검토 예약
  async scheduleReview(
    consentId: string,
    schedule: ReviewScheduleConfig
  ): Promise<ScheduledReview> {
    const consent = await this.consentRepository.findById(consentId);
    if (!consent) {
      throw new NotFoundError(`동의 ${consentId}를 찾을 수 없습니다`);
    }

    // 첫 번째 검토 날짜 계산
    const firstReviewDate = this.calculateFirstReviewDate(
      consent.validity.effectiveDate,
      schedule.frequency
    );

    // 검토 일정 생성
    const reviewSchedule: ReviewSchedule = {
      reviewRequired: true,
      reviewFrequency: schedule.frequency,
      nextReviewDate: firstReviewDate,
      reviewType: schedule.reviewType,
      reviewProcess: {
        initiatedBy: schedule.initiatedBy || 'SYSTEM',
        notificationDays: schedule.notificationDays || 30,
        reminderSchedule: schedule.reminderSchedule || [30, 14, 7, 1],
        escalationProcess: schedule.escalationProcess || 'STANDARD',
        defaultIfNoResponse: schedule.defaultIfNoResponse || 'MAINTAIN',
      },
      reviewHistory: [],
    };

    // 검토 일정으로 동의 업데이트
    await this.consentRepository.update(consentId, {
      validity: {
        ...consent.validity,
        reviewSchedule,
      },
    });

    // 알림 예약
    await this.scheduleReviewNotifications(consentId, reviewSchedule);

    return {
      consentId,
      nextReviewDate: firstReviewDate,
      schedule: reviewSchedule,
    };
  }

  // 검토 처리
  async processReview(
    consentId: string,
    review: ReviewInput
  ): Promise<ReviewResult> {
    const consent = await this.consentRepository.findById(consentId);
    if (!consent) {
      throw new NotFoundError(`동의 ${consentId}를 찾을 수 없습니다`);
    }

    // 검토자 권한 검증
    await this.validateReviewerAuthority(consent, review.reviewedBy);

    // 검토 기록
    const reviewRecord: ReviewRecord = {
      reviewId: generateReviewId(),
      scheduledDate: consent.validity.reviewSchedule?.nextReviewDate || new Date(),
      completedDate: new Date(),
      reviewType: review.reviewType,
      reviewedBy: review.reviewedBy,
      outcome: review.outcome,
      modifications: review.modifications,
      notes: review.notes,
    };

    // 결과 처리
    let updatedConsent: ConsentRecord;

    switch (review.outcome) {
      case 'REAFFIRMED':
        updatedConsent = await this.processReaffirmation(consent, reviewRecord);
        break;

      case 'MODIFIED':
        updatedConsent = await this.processModification(
          consent,
          reviewRecord,
          review.modifications!
        );
        break;

      case 'REVOKED':
        updatedConsent = await this.processRevocationFromReview(
          consent,
          reviewRecord,
          review.revocationReason!
        );
        break;

      case 'DEFERRED':
        updatedConsent = await this.processDeferral(
          consent,
          reviewRecord,
          review.deferralDate!
        );
        break;

      default:
        throw new Error(`알 수 없는 검토 결과: ${review.outcome}`);
    }

    // 해당되는 경우 다음 검토 예약
    if (review.outcome !== 'REVOKED') {
      await this.scheduleNextReview(updatedConsent);
    }

    return {
      reviewId: reviewRecord.reviewId,
      consentId,
      outcome: review.outcome,
      completedAt: reviewRecord.completedDate,
      nextReviewDate: updatedConsent.validity.reviewSchedule?.nextReviewDate,
    };
  }

  private async processReaffirmation(
    consent: ConsentRecord,
    reviewRecord: ReviewRecord
  ): Promise<ConsentRecord> {
    // 기록에 검토 추가
    const reviewHistory = consent.validity.reviewSchedule?.reviewHistory || [];
    reviewHistory.push(reviewRecord);

    // 동의 업데이트
    return this.consentRepository.update(consent.id, {
      validity: {
        ...consent.validity,
        reviewSchedule: {
          ...consent.validity.reviewSchedule!,
          reviewHistory,
        },
      },
      metadata: {
        ...consent.metadata,
        lastReviewedAt: reviewRecord.completedDate,
        lastReviewedBy: reviewRecord.reviewedBy,
      },
    });
  }

  private async processModification(
    consent: ConsentRecord,
    reviewRecord: ReviewRecord,
    modifications: ConsentModification[]
  ): Promise<ConsentRecord> {
    // 수정 적용
    let updatedConsent = { ...consent };

    for (const mod of modifications) {
      updatedConsent = this.applyModification(updatedConsent, mod);
    }

    // 버전 증가
    updatedConsent.metadata = {
      ...updatedConsent.metadata,
      version: updatedConsent.metadata.version + 1,
      updatedAt: new Date(),
      previousVersion: consent.metadata.version,
    };

    // 기록에 검토 추가
    const reviewHistory = consent.validity.reviewSchedule?.reviewHistory || [];
    reviewHistory.push(reviewRecord);
    updatedConsent.validity.reviewSchedule!.reviewHistory = reviewHistory;

    // 저장
    return this.consentRepository.update(consent.id, updatedConsent);
  }

  private applyModification(
    consent: ConsentRecord,
    modification: ConsentModification
  ): ConsentRecord {
    switch (modification.type) {
      case 'UPDATE_DECISION':
        return {
          ...consent,
          decisions: consent.decisions.map(d =>
            d.decisionId === modification.targetId
              ? { ...d, ...modification.changes }
              : d
          ),
        };

      case 'ADD_DECISION':
        return {
          ...consent,
          decisions: [...consent.decisions, modification.newDecision!],
        };

      case 'REMOVE_DECISION':
        return {
          ...consent,
          decisions: consent.decisions.filter(
            d => d.decisionId !== modification.targetId
          ),
        };

      case 'UPDATE_SCOPE':
        return {
          ...consent,
          scope: { ...consent.scope, ...modification.changes },
        };

      case 'UPDATE_PROXY':
        // 대리인 업데이트 처리
        return consent;

      default:
        return consent;
    }
  }

  // 검토 타임아웃 처리
  async handleReviewTimeout(consentId: string): Promise<void> {
    const consent = await this.consentRepository.findById(consentId);
    if (!consent) return;

    const defaultAction = consent.validity.reviewSchedule?.reviewProcess.defaultIfNoResponse;

    switch (defaultAction) {
      case 'MAINTAIN':
        // 동의 활성 유지, 검토 재예약
        await this.processReview(consentId, {
          reviewType: 'TIMEOUT',
          reviewedBy: 'SYSTEM',
          outcome: 'REAFFIRMED',
          notes: '검토 타임아웃으로 인한 자동 재확인',
        });
        break;

      case 'SUSPEND':
        // 동의 일시 중단
        await this.consentRepository.updateStatus(consentId, {
          status: 'SUSPENDED',
          statusChange: {
            fromStatus: consent.validity.status,
            toStatus: 'SUSPENDED',
            changeDate: new Date(),
            changedBy: 'SYSTEM',
            reason: '검토 타임아웃 - 검토 보류 중 동의 일시 중단',
          },
        });
        break;

      case 'EXPIRE':
        // 동의 만료
        await this.consentRepository.updateStatus(consentId, {
          status: 'EXPIRED',
          statusChange: {
            fromStatus: consent.validity.status,
            toStatus: 'EXPIRED',
            changeDate: new Date(),
            changedBy: 'SYSTEM',
            reason: '검토 타임아웃 - 동의 만료',
          },
        });
        break;
    }

    // 이해관계자에게 알림
    await this.notificationService.notifyReviewTimeout(consent, defaultAction);
  }

  // 알림 예약
  private async scheduleReviewNotifications(
    consentId: string,
    schedule: ReviewSchedule
  ): Promise<void> {
    const reminderDays = schedule.reviewProcess.reminderSchedule;
    const reviewDate = schedule.nextReviewDate;

    for (const days of reminderDays) {
      const notificationDate = new Date(reviewDate);
      notificationDate.setDate(notificationDate.getDate() - days);

      if (notificationDate > new Date()) {
        await this.scheduler.schedule({
          type: 'REVIEW_REMINDER',
          consentId,
          scheduledFor: notificationDate,
          data: {
            daysUntilReview: days,
            reviewDate,
          },
        });
      }
    }

    // 타임아웃 핸들러 예약
    const timeoutDate = new Date(reviewDate);
    timeoutDate.setDate(timeoutDate.getDate() + 7); // 7일 유예 기간

    await this.scheduler.schedule({
      type: 'REVIEW_TIMEOUT',
      consentId,
      scheduledFor: timeoutDate,
    });
  }
}

interface ReviewScheduleConfig {
  frequency: string;           // ISO 8601 기간
  reviewType: 'REAFFIRMATION' | 'UPDATE' | 'CAPACITY_CHECK';
  initiatedBy?: string;
  notificationDays?: number;
  reminderSchedule?: number[];
  escalationProcess?: string;
  defaultIfNoResponse?: 'MAINTAIN' | 'SUSPEND' | 'EXPIRE';
}

interface ScheduledReview {
  consentId: string;
  nextReviewDate: Date;
  schedule: ReviewSchedule;
}

interface ReviewInput {
  reviewType: string;
  reviewedBy: string;
  outcome: 'REAFFIRMED' | 'MODIFIED' | 'REVOKED' | 'DEFERRED' | 'NO_RESPONSE';
  modifications?: ConsentModification[];
  revocationReason?: string;
  deferralDate?: Date;
  notes?: string;
}

interface ConsentModification {
  type: 'UPDATE_DECISION' | 'ADD_DECISION' | 'REMOVE_DECISION' | 'UPDATE_SCOPE' | 'UPDATE_PROXY';
  targetId?: string;
  changes?: Record<string, any>;
  newDecision?: ConsentDecision;
}

interface ReviewResult {
  reviewId: string;
  consentId: string;
  outcome: string;
  completedAt: Date;
  nextReviewDate?: Date;
}
```

---

## 5.5 응급 프로토콜

```typescript
// 응급 의사결정 프로토콜
class EmergencyDecisionService {
  private consentService: CryoConsentManagementService;
  private proxyService: ProxyManagementService;
  private ethicsService: EthicsCommitteeService;
  private auditService: AuditService;

  async handleEmergencyDecision(
    request: EmergencyDecisionRequest
  ): Promise<EmergencyDecisionResult> {
    // 응급 개시 기록
    await this.auditService.logEmergencyInitiation(request);

    // 1단계: 명시적 동의 찾기 시도
    const explicitConsent = await this.findExplicitEmergencyConsent(request);
    if (explicitConsent) {
      return this.executeFromExplicitConsent(request, explicitConsent);
    }

    // 2단계: 대리인 가용성 확인
    const proxyDecision = await this.seekEmergencyProxyDecision(request);
    if (proxyDecision) {
      return this.executeFromProxyDecision(request, proxyDecision);
    }

    // 3단계: 응급 기본 프로토콜 적용
    const defaultDecision = await this.applyEmergencyDefaults(request);

    // 4단계: 기록 및 알림
    await this.recordEmergencyDecision(request, defaultDecision);

    return defaultDecision;
  }

  private async findExplicitEmergencyConsent(
    request: EmergencyDecisionRequest
  ): Promise<ConsentRecord | null> {
    // 응급 특정 동의 찾기
    const consents = await this.consentService.queryConsents({
      patientId: request.patientId,
      consentTypes: ['EMERGENCY'],
      statuses: ['ACTIVE'],
      includesDecisionType: request.decisionType,
    });

    // 응급 시나리오를 포함하는 일반 동의도 확인
    const generalConsents = await this.consentService.queryConsents({
      patientId: request.patientId,
      statuses: ['ACTIVE'],
      includesDecisionType: request.decisionType,
    });

    // 결합하여 가장 적용 가능한 것 찾기
    const allConsents = [...consents.consents, ...generalConsents.consents];

    for (const consent of allConsents) {
      const decision = consent.decisions.find(
        d => d.question.toLowerCase().includes('응급') ||
             d.question.toLowerCase().includes('emergency') ||
             d.context?.scenarioDescription?.includes('응급')
      );
      if (decision) {
        return consent;
      }
    }

    return null;
  }

  private async seekEmergencyProxyDecision(
    request: EmergencyDecisionRequest
  ): Promise<ProxyDecisionResult | null> {
    // 순서대로 대리인 가져오기
    const proxies = await this.proxyService.getProxiesForPatient(
      request.patientId
    );

    // 응급 권한이 있는 대리인 필터링
    const emergencyProxies = proxies.filter(p =>
      p.authorityScope.categories.includes(ConsentCategory.PRESERVATION) ||
      p.authorityScope.categories.includes(ConsentCategory.CARE)
    );

    // 순서대로 대리인 연락 시도
    for (const proxy of emergencyProxies) {
      // 대리인 연락 가능 여부 확인
      const reachable = await this.isProxyReachable(proxy);
      if (!reachable) continue;

      // 응급 의사결정 요청
      const decision = await this.requestEmergencyProxyDecision(
        proxy,
        request,
        request.urgencyLevel
      );

      if (decision) {
        return {
          proxy,
          decision,
          responseTime: decision.responseTime,
        };
      }
    }

    return null;
  }

  private async applyEmergencyDefaults(
    request: EmergencyDecisionRequest
  ): Promise<EmergencyDecisionResult> {
    // 가용한 경우 환자의 문서화된 가치관 가져오기
    const values = await this.getPatientValues(request.patientId);

    // 의사결정 유형과 가치관에 따라 기본값 결정
    let decision: any;
    let basis: string;

    switch (request.decisionCategory) {
      case 'PRESERVATION':
        // 기본: 가능한 경우 보존
        decision = this.getPreservationDefault(request, values);
        basis = 'PRESERVATION_PRIORITY_DEFAULT';
        break;

      case 'TREATMENT':
        // 기본: 가장 덜 침습적인 가역적 옵션
        decision = this.getTreatmentDefault(request, values);
        basis = 'LEAST_INVASIVE_DEFAULT';
        break;

      case 'TRANSFER':
        // 기본: 안전하지 않은 경우가 아니면 현재 위치 유지
        decision = this.getTransferDefault(request, values);
        basis = 'STABILITY_DEFAULT';
        break;

      default:
        // 윤리위원회로 에스컬레이션
        return this.escalateToEthics(request);
    }

    return {
      executed: true,
      decision,
      basis,
      authorityType: 'EMERGENCY_DEFAULT',
      confidence: this.calculateDefaultConfidence(request, values),
      requiresFollowUp: true,
      followUpActions: [
        '환자/대리인에게 응급 의사결정 알림',
        '전체 근거 문서화',
        '소급 검토 예약',
      ],
    };
  }

  private getPreservationDefault(
    request: EmergencyDecisionRequest,
    values: PatientValueDocument | null
  ): any {
    // 명시적 선호도 확인
    if (values?.scenarioPreferences) {
      const emergencyPref = values.scenarioPreferences.find(
        p => p.scenarioDescription.toLowerCase().includes('응급') ||
             p.scenarioDescription.toLowerCase().includes('emergency')
      );
      if (emergencyPref) {
        return {
          action: emergencyPref.decision,
          source: 'DOCUMENTED_PREFERENCE',
        };
      }
    }

    // 가능한 경우 보존으로 기본 설정
    return {
      action: 'PROCEED_WITH_PRESERVATION',
      source: 'STANDARD_EMERGENCY_PROTOCOL',
      reasoning: '명시적 지시가 없는 경우, 표준 냉동보존 응급 프로토콜에 따라 환자 보존',
    };
  }

  private async escalateToEthics(
    request: EmergencyDecisionRequest
  ): Promise<EmergencyDecisionResult> {
    // 응급 윤리 상담 요청
    const ethicsResponse = await this.ethicsService.requestEmergencyConsultation({
      patientId: request.patientId,
      decisionType: request.decisionType,
      urgencyLevel: request.urgencyLevel,
      timeConstraint: request.timeConstraint,
      context: request.context,
    });

    return {
      executed: true,
      decision: ethicsResponse.recommendation,
      basis: 'ETHICS_COMMITTEE_EMERGENCY',
      authorityType: 'ETHICS_ESCALATION',
      confidence: ethicsResponse.confidence,
      ethicsReference: ethicsResponse.consultationId,
      requiresFollowUp: true,
    };
  }

  private async recordEmergencyDecision(
    request: EmergencyDecisionRequest,
    result: EmergencyDecisionResult
  ): Promise<void> {
    // 상세 감사 레코드 생성
    await this.auditService.logEmergencyDecision({
      requestId: request.requestId,
      patientId: request.patientId,
      decisionType: request.decisionType,
      urgencyLevel: request.urgencyLevel,
      decision: result.decision,
      basis: result.basis,
      authorityType: result.authorityType,
      confidence: result.confidence,
      timestamp: new Date(),
      context: request.context,
    });

    // 응급 의사결정에 대한 동의 레코드 생성
    if (result.executed) {
      await this.createEmergencyConsentRecord(request, result);
    }

    // 후속 조치 예약
    if (result.requiresFollowUp) {
      await this.scheduleEmergencyFollowUp(request, result);
    }
  }

  private async createEmergencyConsentRecord(
    request: EmergencyDecisionRequest,
    result: EmergencyDecisionResult
  ): Promise<void> {
    const emergencyConsent: ConsentRecord = {
      id: generateConsentId(),
      patientId: request.patientId,
      organizationId: request.organizationId,
      consentType: ConsentType.EMERGENCY,
      category: request.decisionCategory as ConsentCategory,
      subcategories: ['EMERGENCY'],

      scope: {
        procedures: [{
          procedureType: request.decisionType,
          specificProcedures: [request.specificProcedure],
          exclusions: [],
          conditions: [],
        }],
        timeframe: {
          type: 'DEFINITE',
          startDate: new Date(),
          endDate: new Date(Date.now() + 24 * 60 * 60 * 1000), // 24시간
        },
      },

      decisions: [{
        decisionId: generateDecisionId(),
        decisionType: 'BINARY',
        question: `응급 의사결정: ${request.decisionType}`,
        answer: {
          type: 'BINARY',
          binaryValue: result.decision.action !== 'DO_NOT_PROCEED',
        },
        reasoning: result.decision.reasoning,
        context: {
          scenarioDescription: '응급 상황',
          relevantFactors: request.context.factors || [],
          assumedConditions: ['시간 긴급 의사결정', '일반 동의 프로세스 불가'],
          uncertainties: ['환자 의향이 명시적으로 문서화되지 않음'],
          timestamp: new Date(),
        },
        metadata: {
          version: 1,
          createdAt: new Date(),
          createdBy: 'EMERGENCY_SYSTEM',
          confidenceLevel: result.confidence,
          requiresPeriodicReview: true,
        },
      }],

      authority: {
        grantor: {
          entityId: 'EMERGENCY_SYSTEM',
          entityType: 'ORGANIZATION',
          identityVerification: {
            verificationMethod: 'SYSTEM',
            verificationDate: new Date(),
            verifiedBy: 'SYSTEM',
            documentTypes: [],
            documentReferences: [],
            verificationScore: 1,
          },
          capacityConfirmed: true,
          capacityDate: new Date(),
          contactInfo: {
            email: 'emergency@facility.org',
          },
        },
      },

      validity: {
        status: 'ACTIVE',
        statusHistory: [],
        effectiveDate: new Date(),
        expirationDate: new Date(Date.now() + 24 * 60 * 60 * 1000),
        conditions: [{
          conditionId: 'EMERGENCY_CONDITION',
          conditionType: 'TERMINATION',
          description: '일반 동의를 받으면 만료',
          evaluationExpression: 'normalConsentObtained',
          mustBeTrueFor: 'CONTINUATION',
          evaluationMethod: 'MANUAL',
          failureAction: 'TERMINATE',
        }],
      },

      documents: [],

      metadata: {
        version: 1,
        createdAt: new Date(),
        updatedAt: new Date(),
        createdBy: 'EMERGENCY_SYSTEM',
        documentFormat: 'WIA-CONSENT-V1',
      },
    };

    await this.consentService.createConsent(emergencyConsent);
  }
}

interface EmergencyDecisionRequest {
  requestId: string;
  patientId: string;
  organizationId: string;
  decisionType: string;
  decisionCategory: string;
  specificProcedure: string;
  urgencyLevel: 'CRITICAL' | 'URGENT' | 'HIGH';
  timeConstraint: number;      // 의사결정 가능 시간(분)
  context: {
    situation: string;
    factors: string[];
    alternatives: string[];
  };
}

interface EmergencyDecisionResult {
  executed: boolean;
  decision: any;
  basis: string;
  authorityType: string;
  confidence: number;
  ethicsReference?: string;
  requiresFollowUp: boolean;
  followUpActions?: string[];
}

interface ProxyDecisionResult {
  proxy: ProxyDesignation;
  decision: any;
  responseTime: number;
}
```

---

*다음 장: 통합 - 의료 시스템, 법적 프레임워크 및 냉동보존 기관과의 연결*
