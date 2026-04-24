# Chapter 5: Control Protocols

## Consent Lifecycle Management

This chapter defines the control protocols for managing consent throughout its lifecycle, including state transitions, workflow management, decision execution, and escalation procedures.

---

## 5.1 Consent State Machine

```typescript
// Consent state machine definition
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
    // From DRAFT
    { from: 'DRAFT', to: 'PENDING_WITNESS', event: 'SUBMIT_FOR_WITNESS' },
    { from: 'DRAFT', to: 'PENDING_REVIEW', event: 'SUBMIT_FOR_REVIEW' },
    { from: 'DRAFT', to: 'ACTIVE', event: 'ACTIVATE_DIRECTLY' },

    // From PENDING_WITNESS
    { from: 'PENDING_WITNESS', to: 'PENDING_NOTARIZATION', event: 'WITNESS_COMPLETE' },
    { from: 'PENDING_WITNESS', to: 'PENDING_REVIEW', event: 'SKIP_NOTARIZATION' },
    { from: 'PENDING_WITNESS', to: 'DRAFT', event: 'RETURN_TO_DRAFT' },

    // From PENDING_NOTARIZATION
    { from: 'PENDING_NOTARIZATION', to: 'PENDING_REVIEW', event: 'NOTARIZATION_COMPLETE' },
    { from: 'PENDING_NOTARIZATION', to: 'DRAFT', event: 'RETURN_TO_DRAFT' },

    // From PENDING_REVIEW
    { from: 'PENDING_REVIEW', to: 'ACTIVE', event: 'APPROVE' },
    { from: 'PENDING_REVIEW', to: 'DRAFT', event: 'REQUEST_CHANGES' },

    // From ACTIVE
    { from: 'ACTIVE', to: 'SUSPENDED', event: 'SUSPEND' },
    { from: 'ACTIVE', to: 'REVOKED', event: 'REVOKE' },
    { from: 'ACTIVE', to: 'EXPIRED', event: 'EXPIRE' },
    { from: 'ACTIVE', to: 'SUPERSEDED', event: 'SUPERSEDE' },

    // From SUSPENDED
    { from: 'SUSPENDED', to: 'ACTIVE', event: 'REACTIVATE' },
    { from: 'SUSPENDED', to: 'REVOKED', event: 'REVOKE' },

    // Terminal states - no transitions out
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

// State machine implementation
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
    // Get current consent
    const consent = await this.consentRepository.findById(consentId);
    if (!consent) {
      throw new NotFoundError(`Consent ${consentId} not found`);
    }

    const currentState = consent.validity.status;

    // Find valid transition
    const transition = this.findTransition(currentState, event);
    if (!transition) {
      throw new InvalidTransitionError(
        `No transition from ${currentState} with event ${event}`
      );
    }

    // Check guards
    const guardResult = await this.evaluateGuards(transition, consent, context);
    if (!guardResult.passed) {
      throw new GuardFailedError(
        `Transition guard failed: ${guardResult.failedGuard}`,
        guardResult.reason
      );
    }

    // Execute pre-transition actions
    await this.executePreActions(transition, consent, context);

    // Update state
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

    // Execute post-transition actions
    await this.executePostActions(transition, updated, context);

    // Emit event
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
    // Parse and evaluate guard condition
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
          reason: hasCapacity ? undefined : 'Valid capacity assessment required',
        };

      case 'hasRequiredDecisions':
        const hasDecisions = consent.decisions.length > 0;
        return {
          passed: hasDecisions,
          reason: hasDecisions ? undefined : 'At least one decision required',
        };

      case 'hasMinimumWitnesses':
        const witnessCount = consent.authority.witnesses?.length || 0;
        const minWitnesses = 2; // Configurable
        return {
          passed: witnessCount >= minWitnesses,
          reason:
            witnessCount >= minWitnesses
              ? undefined
              : `Minimum ${minWitnesses} witnesses required`,
        };

      case 'allWitnessesAttested':
        const allAttested =
          consent.authority.witnesses?.every(
            w => w.attestation.attestationDate
          ) ?? false;
        return {
          passed: allAttested,
          reason: allAttested ? undefined : 'All witnesses must attest',
        };

      case 'hasRevocationAuthority':
        return this.checkRevocationAuthority(consent, context);

      case 'notInProtectedPeriod':
        return this.checkNotInProtectedPeriod(consent);

      default:
        // Unknown condition - fail safe
        return { passed: false, reason: `Unknown condition: ${condition}` };
    }
  }

  private async checkRevocationAuthority(
    consent: ConsentRecord,
    context: TransitionContext
  ): Promise<{ passed: boolean; reason?: string }> {
    // Patient can always revoke their own consent
    if (context.userId === consent.patientId) {
      return { passed: true };
    }

    // Check if user is authorized proxy
    const isAuthorizedProxy = await this.isAuthorizedProxy(
      context.userId,
      consent.patientId,
      'REVOCATION'
    );
    if (isAuthorizedProxy) {
      return { passed: true };
    }

    // Check if user is legal authority
    if (context.legalAuthority) {
      return { passed: true };
    }

    return {
      passed: false,
      reason: 'User lacks authority to revoke this consent',
    };
  }

  private checkNotInProtectedPeriod(
    consent: ConsentRecord
  ): { passed: boolean; reason?: string } {
    // Some consents may have a protected period after activation
    const protectedPeriodDays = 30; // Configurable
    const activationDate = consent.validity.effectiveDate;
    const protectedUntil = new Date(activationDate);
    protectedUntil.setDate(protectedUntil.getDate() + protectedPeriodDays);

    if (new Date() < protectedUntil) {
      return {
        passed: false,
        reason: `Consent is in protected period until ${protectedUntil.toISOString()}`,
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

## 5.2 Workflow Engine

```typescript
// Consent workflow definitions
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

  // Execution
  handler: string;
  config: Record<string, any>;

  // Control flow
  next: WorkflowNext[];
  timeout?: string;        // ISO 8601 duration
  retryPolicy?: RetryPolicy;

  // Requirements
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

// Workflow engine implementation
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
    // Register built-in step handlers
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
      throw new NotFoundError(`Workflow ${workflowId} not found`);
    }

    // Create execution record
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

    // Start first step
    await this.executeStep(execution, workflow.steps[0]);

    return execution;
  }

  private async executeStep(
    execution: WorkflowExecution,
    step: WorkflowStep
  ): Promise<void> {
    // Update execution status
    execution.currentStep = step.stepId;
    await this.executionRepository.update(execution);

    // Record step start
    const stepExecution: StepExecution = {
      stepId: step.stepId,
      status: 'RUNNING',
      startedAt: new Date(),
    };
    execution.stepHistory.push(stepExecution);

    try {
      // Get handler
      const handler = this.stepHandlers.get(step.handler);
      if (!handler) {
        throw new Error(`Handler ${step.handler} not found`);
      }

      // Execute step
      const result = await handler.execute(step, execution);

      // Update step execution
      stepExecution.status = 'COMPLETED';
      stepExecution.completedAt = new Date();
      stepExecution.result = result;

      await this.executionRepository.update(execution);

      // Determine next step
      const nextStep = this.determineNextStep(step, result, execution);

      if (nextStep) {
        await this.executeStep(execution, nextStep);
      } else {
        // Workflow complete
        execution.status = 'COMPLETED';
        execution.completedAt = new Date();
        await this.executionRepository.update(execution);
      }
    } catch (error) {
      // Handle step failure
      stepExecution.status = 'FAILED';
      stepExecution.error = error.message;
      stepExecution.completedAt = new Date();

      // Check retry policy
      if (step.retryPolicy && stepExecution.retryCount < step.retryPolicy.maxRetries) {
        stepExecution.retryCount = (stepExecution.retryCount || 0) + 1;
        const delay = this.calculateRetryDelay(step.retryPolicy, stepExecution.retryCount);
        await this.scheduleRetry(execution, step, delay);
      } else {
        // Escalate or fail workflow
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

    // Evaluate conditions to find next step
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

    // No next step - workflow ends
    return undefined;
  }

  private evaluateCondition(
    condition: string,
    result: StepResult,
    execution: WorkflowExecution
  ): boolean {
    // Simple condition evaluation
    // In production, use a proper expression evaluator
    const context = {
      result,
      execution,
      stepHistory: execution.stepHistory,
    };

    try {
      // Safe evaluation (would use proper sandbox in production)
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

    // Check for escalation rules
    const escalation = workflow.escalations.find(
      e => e.triggerStep === step.stepId || e.triggerStep === '*'
    );

    if (escalation) {
      await this.executeEscalation(execution, escalation, error);
    } else {
      // No escalation - fail workflow
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
    // Send escalation notifications
    for (const target of escalation.notifyTargets) {
      await this.notificationService.sendEscalation({
        executionId: execution.executionId,
        consentId: execution.consentId,
        error: error.message,
        target,
        escalationLevel: escalation.level,
      });
    }

    // Update execution status
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

// Step handler interface
interface WorkflowStepHandler {
  execute(step: WorkflowStep, execution: WorkflowExecution): Promise<StepResult>;
}

// Example step handlers
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
      // Need more witnesses - wait
      return {
        success: false,
        data: {
          required: requiredWitnesses,
          current: currentWitnesses,
          waiting: true,
        },
      };
    }

    // Check all witnesses have attested
    const allAttested = consent.authority.witnesses!.every(
      w => w.attestation.attestationDate
    );

    if (!allAttested) {
      return {
        success: false,
        data: { waiting: true, reason: 'Awaiting witness attestations' },
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

    // Update consent status to active
    await updateConsentStatus(consent.id, {
      status: 'ACTIVE',
      effectiveDate: new Date(),
    });

    // Schedule reviews
    if (step.config.scheduleReview) {
      await scheduleConsentReview(consent.id, {
        frequency: step.config.reviewFrequency || 'P1Y',
        firstReview: calculateFirstReviewDate(step.config.reviewFrequency || 'P1Y'),
      });
    }

    // Notify stakeholders
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

## 5.3 Decision Execution Protocol

```typescript
// Decision execution protocol
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

  // Execute a decision based on consent
  async executeDecision(
    request: DecisionExecutionRequest
  ): Promise<DecisionExecutionResult> {
    // Step 1: Find applicable consent
    const effectiveConsent = await this.consentService.getEffectiveConsent(
      request.patientId,
      request.decisionType,
      request.context
    );

    // Step 2: Determine authority
    const authority = await this.determineAuthority(
      request,
      effectiveConsent
    );

    // Step 3: Validate execution conditions
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

    // Step 4: Execute decision
    const execution = await this.performExecution(request, effectiveConsent, authority);

    // Step 5: Record execution
    await this.recordExecution(request, effectiveConsent, authority, execution);

    // Step 6: Notify stakeholders
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
    // If patient is making decision
    if (request.requestedBy.type === 'PATIENT' &&
        request.requestedBy.id === request.patientId) {
      return {
        type: 'PATIENT_DIRECT',
        entityId: request.patientId,
        basis: 'PATIENT_REQUEST',
      };
    }

    // If explicit consent exists
    if (effectiveConsent.found && effectiveConsent.confidence >= 0.8) {
      return {
        type: 'EXPLICIT_CONSENT',
        entityId: request.patientId,
        consentId: effectiveConsent.consent!.id,
        basis: 'DOCUMENTED_CONSENT',
      };
    }

    // If proxy is requesting
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

    // Need escalation
    return {
      type: 'ESCALATION_REQUIRED',
      basis: 'INSUFFICIENT_AUTHORITY',
      escalationReason: 'No valid authority found for this decision',
    };
  }

  private async validateExecution(
    request: DecisionExecutionRequest,
    effectiveConsent: EffectiveConsent,
    authority: DecisionAuthority
  ): Promise<ExecutionValidationResult> {
    const errors: string[] = [];

    // Check authority is valid
    if (authority.type === 'ESCALATION_REQUIRED') {
      return {
        valid: false,
        errors: ['No valid authority to execute this decision'],
        requiresEscalation: true,
      };
    }

    // Check consent conditions
    if (effectiveConsent.consent) {
      const conditionResults = await this.evaluateConsentConditions(
        effectiveConsent.consent,
        request.context
      );

      if (!conditionResults.allMet) {
        errors.push(...conditionResults.unmetConditions);
      }
    }

    // Check scope limitations
    if (effectiveConsent.consent?.scope.limitations) {
      for (const limitation of effectiveConsent.consent.scope.limitations) {
        if (this.limitationApplies(limitation, request)) {
          if (limitation.hardLimit) {
            errors.push(`Hard limitation violated: ${limitation.description}`);
          } else {
            // Soft limitation - warning
          }
        }
      }
    }

    // Check timing constraints
    if (request.timing) {
      const timingValid = await this.validateTiming(request.timing, effectiveConsent);
      if (!timingValid.valid) {
        errors.push(`Timing constraint: ${timingValid.reason}`);
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
    // Get the decision to execute
    const decision = effectiveConsent.decision;

    // Execute based on decision type
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
        throw new Error(`Unknown decision type: ${decision?.decisionType}`);
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
      // Proceed with requested action
      return {
        proceed: true,
        action: request.proposedAction,
      };
    } else {
      // Do not proceed
      return {
        proceed: false,
        reason: decision.reasoning || 'Patient consent indicates do not proceed',
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

    // Get current value from request context
    const currentValue = request.context.currentValue as number;

    // Evaluate threshold
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

    // Evaluate rules in priority order
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

    // No condition met - use default if exists
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
      reason: 'No applicable rule found',
    };
  }

  private async executeDelegation(
    request: DecisionExecutionRequest,
    decision: ConsentDecision
  ): Promise<DelegationExecutionResult> {
    const delegateTo = decision.answer.delegateTo!;

    // Find the delegate
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
        reason: 'Delegate not found or unavailable',
        fallbackRequired: true,
      };
    }

    // Request decision from delegate
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
    // Create audit record
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

    // Update consent usage statistics
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

## 5.4 Review and Renewal Protocols

```typescript
// Review scheduling and management
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

  // Schedule periodic review
  async scheduleReview(
    consentId: string,
    schedule: ReviewScheduleConfig
  ): Promise<ScheduledReview> {
    const consent = await this.consentRepository.findById(consentId);
    if (!consent) {
      throw new NotFoundError(`Consent ${consentId} not found`);
    }

    // Calculate first review date
    const firstReviewDate = this.calculateFirstReviewDate(
      consent.validity.effectiveDate,
      schedule.frequency
    );

    // Create review schedule
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

    // Update consent with review schedule
    await this.consentRepository.update(consentId, {
      validity: {
        ...consent.validity,
        reviewSchedule,
      },
    });

    // Schedule notifications
    await this.scheduleReviewNotifications(consentId, reviewSchedule);

    return {
      consentId,
      nextReviewDate: firstReviewDate,
      schedule: reviewSchedule,
    };
  }

  // Process review
  async processReview(
    consentId: string,
    review: ReviewInput
  ): Promise<ReviewResult> {
    const consent = await this.consentRepository.findById(consentId);
    if (!consent) {
      throw new NotFoundError(`Consent ${consentId} not found`);
    }

    // Validate reviewer authority
    await this.validateReviewerAuthority(consent, review.reviewedBy);

    // Record review
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

    // Process outcome
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
        throw new Error(`Unknown review outcome: ${review.outcome}`);
    }

    // Schedule next review if applicable
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
    // Add review to history
    const reviewHistory = consent.validity.reviewSchedule?.reviewHistory || [];
    reviewHistory.push(reviewRecord);

    // Update consent
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
    // Apply modifications
    let updatedConsent = { ...consent };

    for (const mod of modifications) {
      updatedConsent = this.applyModification(updatedConsent, mod);
    }

    // Increment version
    updatedConsent.metadata = {
      ...updatedConsent.metadata,
      version: updatedConsent.metadata.version + 1,
      updatedAt: new Date(),
      previousVersion: consent.metadata.version,
    };

    // Add review to history
    const reviewHistory = consent.validity.reviewSchedule?.reviewHistory || [];
    reviewHistory.push(reviewRecord);
    updatedConsent.validity.reviewSchedule!.reviewHistory = reviewHistory;

    // Store
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
        // Handle proxy updates
        return consent;

      default:
        return consent;
    }
  }

  // Handle review timeout
  async handleReviewTimeout(consentId: string): Promise<void> {
    const consent = await this.consentRepository.findById(consentId);
    if (!consent) return;

    const defaultAction = consent.validity.reviewSchedule?.reviewProcess.defaultIfNoResponse;

    switch (defaultAction) {
      case 'MAINTAIN':
        // Keep consent active, reschedule review
        await this.processReview(consentId, {
          reviewType: 'TIMEOUT',
          reviewedBy: 'SYSTEM',
          outcome: 'REAFFIRMED',
          notes: 'Auto-reaffirmed due to review timeout',
        });
        break;

      case 'SUSPEND':
        // Suspend consent
        await this.consentRepository.updateStatus(consentId, {
          status: 'SUSPENDED',
          statusChange: {
            fromStatus: consent.validity.status,
            toStatus: 'SUSPENDED',
            changeDate: new Date(),
            changedBy: 'SYSTEM',
            reason: 'Review timeout - consent suspended pending review',
          },
        });
        break;

      case 'EXPIRE':
        // Expire consent
        await this.consentRepository.updateStatus(consentId, {
          status: 'EXPIRED',
          statusChange: {
            fromStatus: consent.validity.status,
            toStatus: 'EXPIRED',
            changeDate: new Date(),
            changedBy: 'SYSTEM',
            reason: 'Review timeout - consent expired',
          },
        });
        break;
    }

    // Notify stakeholders
    await this.notificationService.notifyReviewTimeout(consent, defaultAction);
  }

  // Schedule notifications
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

    // Schedule timeout handler
    const timeoutDate = new Date(reviewDate);
    timeoutDate.setDate(timeoutDate.getDate() + 7); // 7 days grace period

    await this.scheduler.schedule({
      type: 'REVIEW_TIMEOUT',
      consentId,
      scheduledFor: timeoutDate,
    });
  }
}

interface ReviewScheduleConfig {
  frequency: string;           // ISO 8601 duration
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

## 5.5 Emergency Protocols

```typescript
// Emergency decision protocols
class EmergencyDecisionService {
  private consentService: CryoConsentManagementService;
  private proxyService: ProxyManagementService;
  private ethicsService: EthicsCommitteeService;
  private auditService: AuditService;

  async handleEmergencyDecision(
    request: EmergencyDecisionRequest
  ): Promise<EmergencyDecisionResult> {
    // Log emergency initiation
    await this.auditService.logEmergencyInitiation(request);

    // Step 1: Attempt to find explicit consent
    const explicitConsent = await this.findExplicitEmergencyConsent(request);
    if (explicitConsent) {
      return this.executeFromExplicitConsent(request, explicitConsent);
    }

    // Step 2: Check for proxy availability
    const proxyDecision = await this.seekEmergencyProxyDecision(request);
    if (proxyDecision) {
      return this.executeFromProxyDecision(request, proxyDecision);
    }

    // Step 3: Apply emergency default protocols
    const defaultDecision = await this.applyEmergencyDefaults(request);

    // Step 4: Record and notify
    await this.recordEmergencyDecision(request, defaultDecision);

    return defaultDecision;
  }

  private async findExplicitEmergencyConsent(
    request: EmergencyDecisionRequest
  ): Promise<ConsentRecord | null> {
    // Look for emergency-specific consent
    const consents = await this.consentService.queryConsents({
      patientId: request.patientId,
      consentTypes: ['EMERGENCY'],
      statuses: ['ACTIVE'],
      includesDecisionType: request.decisionType,
    });

    // Also check general consents that cover emergency scenarios
    const generalConsents = await this.consentService.queryConsents({
      patientId: request.patientId,
      statuses: ['ACTIVE'],
      includesDecisionType: request.decisionType,
    });

    // Combine and find most applicable
    const allConsents = [...consents.consents, ...generalConsents.consents];

    for (const consent of allConsents) {
      const decision = consent.decisions.find(
        d => d.question.toLowerCase().includes('emergency') ||
             d.context?.scenarioDescription?.includes('emergency')
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
    // Get proxies in order
    const proxies = await this.proxyService.getProxiesForPatient(
      request.patientId
    );

    // Filter to those with emergency authority
    const emergencyProxies = proxies.filter(p =>
      p.authorityScope.categories.includes(ConsentCategory.PRESERVATION) ||
      p.authorityScope.categories.includes(ConsentCategory.CARE)
    );

    // Try to reach proxies in order
    for (const proxy of emergencyProxies) {
      // Check if proxy is reachable
      const reachable = await this.isProxyReachable(proxy);
      if (!reachable) continue;

      // Request emergency decision
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
    // Get patient's documented values if available
    const values = await this.getPatientValues(request.patientId);

    // Determine default based on decision type and values
    let decision: any;
    let basis: string;

    switch (request.decisionCategory) {
      case 'PRESERVATION':
        // Default: Preserve when possible
        decision = this.getPreservationDefault(request, values);
        basis = 'PRESERVATION_PRIORITY_DEFAULT';
        break;

      case 'TREATMENT':
        // Default: Least invasive reversible option
        decision = this.getTreatmentDefault(request, values);
        basis = 'LEAST_INVASIVE_DEFAULT';
        break;

      case 'TRANSFER':
        // Default: Maintain current location unless unsafe
        decision = this.getTransferDefault(request, values);
        basis = 'STABILITY_DEFAULT';
        break;

      default:
        // Escalate to ethics committee
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
        'Notify patient/proxy of emergency decision',
        'Document full reasoning',
        'Schedule retrospective review',
      ],
    };
  }

  private getPreservationDefault(
    request: EmergencyDecisionRequest,
    values: PatientValueDocument | null
  ): any {
    // Check for any explicit preferences
    if (values?.scenarioPreferences) {
      const emergencyPref = values.scenarioPreferences.find(
        p => p.scenarioDescription.toLowerCase().includes('emergency')
      );
      if (emergencyPref) {
        return {
          action: emergencyPref.decision,
          source: 'DOCUMENTED_PREFERENCE',
        };
      }
    }

    // Default to preservation when possible
    return {
      action: 'PROCEED_WITH_PRESERVATION',
      source: 'STANDARD_EMERGENCY_PROTOCOL',
      reasoning: 'In absence of explicit instructions, preserve patient per standard cryonics emergency protocol',
    };
  }

  private async escalateToEthics(
    request: EmergencyDecisionRequest
  ): Promise<EmergencyDecisionResult> {
    // Request emergency ethics consultation
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
    // Create detailed audit record
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

    // Create consent record for the emergency decision
    if (result.executed) {
      await this.createEmergencyConsentRecord(request, result);
    }

    // Schedule follow-up
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
          endDate: new Date(Date.now() + 24 * 60 * 60 * 1000), // 24 hours
        },
      },

      decisions: [{
        decisionId: generateDecisionId(),
        decisionType: 'BINARY',
        question: `Emergency decision: ${request.decisionType}`,
        answer: {
          type: 'BINARY',
          binaryValue: result.decision.action !== 'DO_NOT_PROCEED',
        },
        reasoning: result.decision.reasoning,
        context: {
          scenarioDescription: 'Emergency situation',
          relevantFactors: request.context.factors || [],
          assumedConditions: ['Time-critical decision', 'Normal consent process not possible'],
          uncertainties: ['Patient wishes not explicitly documented'],
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
          description: 'Expires when normal consent can be obtained',
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
  timeConstraint: number;      // Minutes available for decision
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

*Next Chapter: Integration - Connecting with healthcare systems, legal frameworks, and cryonics organizations*
