# WIA AI Embodiment Ethics API Interface Standard
## Phase 2 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT

---

## Table of Contents

1. [Overview](#overview)
2. [Terminology](#terminology)
3. [Core Interfaces](#core-interfaces)
4. [REST API Endpoints](#rest-api-endpoints)
5. [Ethics Engine Interface](#ethics-engine-interface)
6. [Authentication & Authorization](#authentication--authorization)
7. [Error Handling](#error-handling)
8. [Rate Limiting](#rate-limiting)
9. [SDK Examples](#sdk-examples)
10. [References](#references)

---

## Overview

### 1.1 Purpose

The WIA AI Embodiment Ethics API Interface Standard defines the programming interfaces for implementing, monitoring, and enforcing ethical constraints in AI-powered physical systems. This specification enables developers to integrate ethical decision-making into embodied AI.

**Core Objectives**:
- Provide APIs for ethical constraint management
- Enable real-time ethical decision evaluation
- Support accountability and audit trail generation
- Define consent management interfaces
- Facilitate ethical override mechanisms

### 1.2 Scope

| Component | Description |
|-----------|-------------|
| **Ethics Engine** | Core ethical evaluation service |
| **Constraint Manager** | Ethical constraint CRUD operations |
| **Audit Service** | Decision logging and retrieval |
| **Consent Manager** | User consent handling |
| **Override Controller** | Human intervention management |

### 1.3 Phase 1 Compatibility

Phase 2 API operates on Phase 1 Data Format:

```
Phase 1: Ethics Data Format (JSON structures)
    ↓
Phase 2: Ethics API Interface (Programming APIs)
    ↓
Phase 3: Ethics Protocol (Communication)
```

---

## Terminology

### 2.1 Core Terms

| Term | Definition |
|------|------------|
| **WiaEthics** | Main ethics API client class |
| **EthicsEngine** | Ethical evaluation processor |
| **ConstraintManager** | Constraint configuration handler |
| **AuditLogger** | Decision audit trail manager |
| **ConsentManager** | User consent handler |
| **OverrideController** | Human intervention manager |

### 2.2 API Conventions

| Convention | Description |
|------------|-------------|
| Base URL | `https://api.wia.live/embodiment-ethics/v1` |
| Content-Type | `application/json` |
| Date Format | ISO 8601 |
| Authentication | Bearer token (JWT) |

---

## Core Interfaces

### 3.1 WiaEthics Class

Main API entry point for ethics management.

#### TypeScript

```typescript
class WiaEthics {
  // Constructor
  constructor(options?: WiaEthicsOptions);

  // Connection
  connect(config: EthicsConfig): Promise<void>;
  disconnect(): Promise<void>;
  isConnected(): boolean;

  // Framework Management
  setFramework(framework: EthicalFramework): Promise<void>;
  getFramework(): Promise<EthicalFramework>;

  // Constraint Management
  addConstraint(constraint: EthicalConstraint): Promise<string>;
  updateConstraint(id: string, constraint: Partial<EthicalConstraint>): Promise<void>;
  removeConstraint(id: string): Promise<void>;
  listConstraints(filter?: ConstraintFilter): Promise<EthicalConstraint[]>;

  // Ethics Evaluation
  evaluate(context: EvaluationContext): Promise<EthicsEvaluation>;
  evaluateAction(action: ProposedAction): Promise<ActionEvaluation>;

  // Consent Management
  recordConsent(consent: ConsentRecord): Promise<string>;
  checkConsent(subjectId: string, scope: string[]): Promise<ConsentStatus>;
  revokeConsent(consentId: string): Promise<void>;

  // Accountability
  logDecision(decision: DecisionRecord): Promise<string>;
  getAuditTrail(filter: AuditFilter): Promise<AuditRecord[]>;

  // Override Management
  requestOverride(request: OverrideRequest): Promise<OverrideResult>;
  approveOverride(overrideId: string, authorization: Authorization): Promise<void>;
  rejectOverride(overrideId: string, reason: string): Promise<void>;

  // Impact Assessment
  assessImpact(context: ImpactContext): Promise<ImpactAssessment>;

  // Events
  on<T extends EthicsEventType>(event: T, handler: EthicsEventHandler<T>): void;
  off<T extends EthicsEventType>(event: T, handler: EthicsEventHandler<T>): void;
}
```

#### Python

```python
class WiaEthics:
    def __init__(self, options: Optional[WiaEthicsOptions] = None):
        ...

    # Connection
    async def connect(self, config: EthicsConfig) -> None: ...
    async def disconnect(self) -> None: ...
    def is_connected(self) -> bool: ...

    # Framework Management
    async def set_framework(self, framework: EthicalFramework) -> None: ...
    async def get_framework(self) -> EthicalFramework: ...

    # Constraint Management
    async def add_constraint(self, constraint: EthicalConstraint) -> str: ...
    async def update_constraint(self, id: str, constraint: EthicalConstraint) -> None: ...
    async def remove_constraint(self, id: str) -> None: ...
    async def list_constraints(self, filter: Optional[ConstraintFilter] = None) -> List[EthicalConstraint]: ...

    # Ethics Evaluation
    async def evaluate(self, context: EvaluationContext) -> EthicsEvaluation: ...
    async def evaluate_action(self, action: ProposedAction) -> ActionEvaluation: ...

    # Consent Management
    async def record_consent(self, consent: ConsentRecord) -> str: ...
    async def check_consent(self, subject_id: str, scope: List[str]) -> ConsentStatus: ...
    async def revoke_consent(self, consent_id: str) -> None: ...

    # Accountability
    async def log_decision(self, decision: DecisionRecord) -> str: ...
    async def get_audit_trail(self, filter: AuditFilter) -> List[AuditRecord]: ...

    # Override Management
    async def request_override(self, request: OverrideRequest) -> OverrideResult: ...
    async def approve_override(self, override_id: str, authorization: Authorization) -> None: ...
    async def reject_override(self, override_id: str, reason: str) -> None: ...
```

### 3.2 Configuration Options

```typescript
interface WiaEthicsOptions {
  // Connection
  baseUrl?: string;              // default: 'https://api.wia.live/embodiment-ethics/v1'
  timeout?: number;              // default: 5000 (ms)

  // Evaluation
  evaluationMode?: 'strict' | 'balanced' | 'permissive';
  defaultFramework?: string;     // Framework ID to use

  // Logging
  auditLogging?: boolean;        // default: true
  logLevel?: 'debug' | 'info' | 'warn' | 'error' | 'none';

  // Override
  overrideTimeout?: number;      // default: 60000 (ms)
  requireOverrideReason?: boolean; // default: true
}

interface EthicsConfig {
  embodimentId: string;
  frameworkId: string;
  strictMode?: boolean;
  humanOversight?: 'always' | 'critical_only' | 'none';
  auditRetention?: number;       // days
}
```

### 3.3 Evaluation Types

```typescript
interface EvaluationContext {
  embodimentId: string;
  timestamp: number;
  action: ProposedAction;
  environment: EnvironmentState;
  stakeholders: Stakeholder[];
  constraints: string[];         // Constraint IDs to evaluate
}

interface ProposedAction {
  actionId: string;
  actionType: string;
  parameters: Record<string, any>;
  predictedOutcomes: PredictedOutcome[];
  urgency: 'immediate' | 'soon' | 'scheduled';
}

interface EthicsEvaluation {
  evaluationId: string;
  timestamp: number;
  context: EvaluationContext;
  result: 'approved' | 'denied' | 'requires_confirmation' | 'conditional';
  score: number;                 // 0-1 ethical score
  principleScores: PrincipleScore[];
  constraintResults: ConstraintResult[];
  explanation: string;
  conditions?: string[];
  alternativeActions?: ProposedAction[];
}

interface ActionEvaluation {
  actionId: string;
  approved: boolean;
  ethicalScore: number;
  riskScore: number;
  constraints: {
    violated: string[];
    warnings: string[];
    satisfied: string[];
  };
  requiredConfirmations: string[];
  explanation: string;
}
```

---

## REST API Endpoints

### 4.1 Endpoint Summary

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/frameworks` | List ethical frameworks |
| GET | `/frameworks/{id}` | Get framework details |
| POST | `/frameworks` | Create custom framework |
| PUT | `/frameworks/{id}` | Update framework |
| GET | `/constraints` | List constraints |
| POST | `/constraints` | Add constraint |
| PUT | `/constraints/{id}` | Update constraint |
| DELETE | `/constraints/{id}` | Remove constraint |
| POST | `/evaluate` | Evaluate ethical context |
| POST | `/evaluate/action` | Evaluate proposed action |
| POST | `/consent` | Record consent |
| GET | `/consent/{subjectId}` | Check consent status |
| DELETE | `/consent/{id}` | Revoke consent |
| GET | `/audit` | Get audit trail |
| POST | `/audit/decisions` | Log decision |
| POST | `/overrides` | Request override |
| PUT | `/overrides/{id}/approve` | Approve override |
| PUT | `/overrides/{id}/reject` | Reject override |
| POST | `/impact-assessment` | Assess impact |

### 4.2 List Frameworks

**GET** `/frameworks`

Lists available ethical frameworks.

**Response:**
```json
{
  "success": true,
  "data": {
    "frameworks": [
      {
        "id": "wia-hybrid-v1",
        "name": "WIA Hybrid Ethical Framework",
        "version": "1.0.0",
        "principleCount": 6,
        "status": "active",
        "certified": true
      },
      {
        "id": "ieee-ethically-aligned",
        "name": "IEEE Ethically Aligned Design",
        "version": "2.0.0",
        "principleCount": 5,
        "status": "active",
        "certified": true
      }
    ],
    "total": 2
  }
}
```

### 4.3 Add Constraint

**POST** `/constraints`

Adds a new ethical constraint.

**Request:**
```json
{
  "embodimentId": "emb-001",
  "constraint": {
    "type": "behavioral",
    "level": "mandatory",
    "rule": {
      "condition": "force_applied > 10 AND target_is_human",
      "action": "deny",
      "message": "Force exceeds safe limit for human contact"
    },
    "rationale": "Prevent injury from excessive force",
    "sourcePrinciple": "non_maleficence",
    "enforcement": "hard"
  }
}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "constraintId": "c-123456",
    "status": "active",
    "createdAt": "2025-01-01T00:00:00Z"
  }
}
```

### 4.4 Evaluate Action

**POST** `/evaluate/action`

Evaluates a proposed action against ethical constraints.

**Request:**
```json
{
  "embodimentId": "emb-001",
  "action": {
    "actionId": "action-001",
    "actionType": "physical_contact",
    "parameters": {
      "contactType": "handshake",
      "targetId": "human-001",
      "force": 5.0
    },
    "predictedOutcomes": [
      {
        "outcome": "successful_handshake",
        "probability": 0.95,
        "impact": "positive"
      }
    ],
    "urgency": "immediate"
  },
  "context": {
    "humanPresent": true,
    "humanConsent": true,
    "environmentSafe": true
  }
}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "actionId": "action-001",
    "approved": true,
    "ethicalScore": 0.92,
    "riskScore": 0.08,
    "constraints": {
      "violated": [],
      "warnings": [],
      "satisfied": ["c-001", "c-002", "c-003"]
    },
    "requiredConfirmations": [],
    "explanation": "Action approved. Human consent obtained, force within safe limits."
  }
}
```

### 4.5 Record Consent

**POST** `/consent`

Records user consent for AI interactions.

**Request:**
```json
{
  "subjectId": "user-001",
  "consentType": "informed",
  "scope": [
    "physical_assistance",
    "data_collection",
    "learning_from_interaction"
  ],
  "granted": true,
  "expiry": "2026-01-01T00:00:00Z",
  "metadata": {
    "language": "ko",
    "informedAbout": ["capabilities", "limitations", "data_usage"],
    "guardianConsent": false
  }
}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "consentId": "consent-abc123",
    "status": "active",
    "recordedAt": "2025-01-01T00:00:00Z",
    "expiresAt": "2026-01-01T00:00:00Z"
  }
}
```

### 4.6 Get Audit Trail

**GET** `/audit?embodimentId=emb-001&startTime=2025-01-01&endTime=2025-01-31`

Retrieves ethical decision audit trail.

**Response:**
```json
{
  "success": true,
  "data": {
    "auditRecords": [
      {
        "recordId": "audit-001",
        "timestamp": "2025-01-15T10:30:00Z",
        "embodimentId": "emb-001",
        "eventType": "decision",
        "decision": {
          "decisionId": "dec-001",
          "actionType": "path_selection",
          "outcome": "approved",
          "ethicalScore": 0.95,
          "humanInvolved": false
        },
        "constraints": ["c-001", "c-002"],
        "explanation": "Path selected to maximize safety while completing task"
      }
    ],
    "total": 1,
    "page": 1,
    "pageSize": 50
  }
}
```

### 4.7 Request Override

**POST** `/overrides`

Requests human override of ethical constraint.

**Request:**
```json
{
  "embodimentId": "emb-001",
  "constraintId": "c-001",
  "reason": "Emergency medical procedure requires exceeding normal force limits",
  "requestedBy": "operator-001",
  "duration": 300,
  "context": {
    "emergency": true,
    "emergencyType": "medical",
    "beneficiary": "patient-001"
  }
}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "overrideId": "ovr-001",
    "status": "pending_approval",
    "requiredApprovers": ["supervisor-001"],
    "expiresAt": "2025-01-01T00:05:00Z"
  }
}
```

### 4.8 Impact Assessment

**POST** `/impact-assessment`

Assesses ethical impact of planned actions.

**Request:**
```json
{
  "embodimentId": "emb-001",
  "assessmentType": "pre_deployment",
  "scope": {
    "actions": ["physical_assistance", "autonomous_navigation"],
    "environment": "hospital_ward",
    "duration": "6_months"
  },
  "stakeholders": [
    { "type": "patient", "count": 50 },
    { "type": "staff", "count": 20 },
    { "type": "visitor", "count": 100 }
  ]
}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "assessmentId": "ia-001",
    "overallRiskScore": 0.25,
    "stakeholderImpacts": [
      {
        "stakeholderType": "patient",
        "impactScore": 0.7,
        "impactType": "physical",
        "description": "Improved care delivery",
        "risks": ["Adjustment period"],
        "mitigations": ["Gradual introduction", "Human backup"]
      }
    ],
    "recommendedMitigations": [
      "Implement gradual rollout",
      "Train staff on AI collaboration",
      "Establish clear override procedures"
    ],
    "approvalRequired": true
  }
}
```

---

## Ethics Engine Interface

### 5.1 Engine Configuration

```typescript
interface EthicsEngineConfig {
  // Framework
  framework: EthicalFramework;
  principleWeights: Record<string, number>;

  // Evaluation
  evaluationMode: 'deontological' | 'consequentialist' | 'hybrid';
  conflictResolution: 'hierarchical' | 'weighted' | 'human_decision';

  // Thresholds
  approvalThreshold: number;     // default: 0.7
  warningThreshold: number;      // default: 0.5
  denyThreshold: number;         // default: 0.3

  // Human oversight
  humanOversightLevel: 'none' | 'critical' | 'all';
  humanTimeoutAction: 'deny' | 'approve_with_log' | 'escalate';
}
```

### 5.2 Real-Time Evaluation

```typescript
// Real-time ethics evaluation for embodiment control
interface RealTimeEthicsEngine {
  // Continuous evaluation
  startContinuousEvaluation(rate: number): void;
  stopContinuousEvaluation(): void;

  // State monitoring
  monitorState(state: EmbodimentState): EthicsStatus;

  // Pre-action check
  preActionCheck(action: ActuatorCommand): PreActionResult;

  // Emergency evaluation
  emergencyEvaluate(context: EmergencyContext): EmergencyDecision;
}

interface PreActionResult {
  allowed: boolean;
  modifications?: ActuatorCommand;  // Modified safe command
  reason?: string;
  escalate?: boolean;
}
```

---

## Authentication & Authorization

### 6.1 JWT Authentication

All API requests require JWT bearer tokens:

```http
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

### 6.2 Token Structure

```json
{
  "sub": "user-123",
  "aud": "wia-embodiment-ethics",
  "iat": 1704110400,
  "exp": 1704196800,
  "permissions": [
    "ethics:read",
    "ethics:evaluate",
    "ethics:configure",
    "ethics:override"
  ],
  "ethicsRole": "ethics_officer",
  "embodiments": ["emb-001", "emb-002"]
}
```

### 6.3 Permission Levels

| Permission | Description |
|------------|-------------|
| `ethics:read` | View ethics records and configurations |
| `ethics:evaluate` | Request ethical evaluations |
| `ethics:configure` | Modify constraints and frameworks |
| `ethics:override` | Request ethical overrides |
| `ethics:approve_override` | Approve override requests |
| `ethics:admin` | Full administrative access |

### 6.4 Role-Based Access

| Role | Permissions |
|------|-------------|
| Viewer | `ethics:read` |
| Operator | `ethics:read`, `ethics:evaluate` |
| Configurator | Above + `ethics:configure` |
| Ethics Officer | Above + `ethics:override`, `ethics:approve_override` |
| Admin | All permissions |

---

## Error Handling

### 7.1 Error Response Format

```json
{
  "success": false,
  "error": {
    "code": "CONSTRAINT_VIOLATION",
    "message": "Action violates mandatory ethical constraint",
    "details": {
      "constraintId": "c-001",
      "constraintName": "Human Safety First",
      "violation": "Predicted force exceeds safe limit"
    },
    "timestamp": 1704110400000,
    "requestId": "req-12345"
  }
}
```

### 7.2 Error Codes

#### Evaluation Errors (1xxx)

| Code | Name | Description |
|------|------|-------------|
| 1001 | `EVALUATION_FAILED` | Ethical evaluation error |
| 1002 | `CONSTRAINT_VIOLATION` | Mandatory constraint violated |
| 1003 | `FRAMEWORK_NOT_FOUND` | Ethical framework unavailable |
| 1004 | `INSUFFICIENT_CONTEXT` | Evaluation context incomplete |
| 1005 | `CONFLICT_UNRESOLVED` | Principle conflict cannot be resolved |

#### Consent Errors (2xxx)

| Code | Name | Description |
|------|------|-------------|
| 2001 | `CONSENT_NOT_FOUND` | No consent record exists |
| 2002 | `CONSENT_EXPIRED` | Consent has expired |
| 2003 | `CONSENT_SCOPE_MISMATCH` | Action outside consent scope |
| 2004 | `CONSENT_REVOKED` | Consent was revoked |

#### Override Errors (3xxx)

| Code | Name | Description |
|------|------|-------------|
| 3001 | `OVERRIDE_DENIED` | Override request rejected |
| 3002 | `OVERRIDE_EXPIRED` | Override request timed out |
| 3003 | `OVERRIDE_UNAUTHORIZED` | Insufficient authority |
| 3004 | `OVERRIDE_NOT_ALLOWED` | Constraint cannot be overridden |

#### Authorization Errors (4xxx)

| Code | Name | Description |
|------|------|-------------|
| 4001 | `UNAUTHORIZED` | Invalid or missing token |
| 4002 | `FORBIDDEN` | Insufficient permissions |
| 4003 | `TOKEN_EXPIRED` | Token has expired |

---

## Rate Limiting

### 8.1 Rate Limits

| Endpoint Category | Rate Limit | Window |
|-------------------|------------|--------|
| Evaluation requests | 100 req/min | Per embodiment |
| Read operations | 1000 req/min | Per user |
| Write operations | 50 req/min | Per user |
| Override requests | 10 req/min | Per embodiment |

### 8.2 Rate Limit Headers

```http
X-RateLimit-Limit: 100
X-RateLimit-Remaining: 95
X-RateLimit-Reset: 1704110460
```

---

## SDK Examples

### 9.1 TypeScript - Ethics Evaluation

```typescript
import { WiaEthics, EthicalFramework } from 'wia-embodiment-ethics';

async function evaluateRobotAction() {
  const ethics = new WiaEthics({
    evaluationMode: 'strict',
    auditLogging: true
  });

  // Connect to ethics service
  await ethics.connect({
    embodimentId: 'emb-001',
    frameworkId: 'wia-hybrid-v1',
    humanOversight: 'critical_only'
  });

  // Evaluate proposed action
  const evaluation = await ethics.evaluateAction({
    actionId: 'action-001',
    actionType: 'physical_assistance',
    parameters: {
      assistanceType: 'lift_support',
      targetPerson: 'patient-001',
      predictedForce: 50
    },
    predictedOutcomes: [
      { outcome: 'successful_assistance', probability: 0.9, impact: 'positive' }
    ],
    urgency: 'soon'
  });

  if (evaluation.approved) {
    console.log('Action approved:', evaluation.explanation);
    return true;
  } else {
    console.log('Action denied:', evaluation.explanation);
    console.log('Alternative actions:', evaluation.alternativeActions);
    return false;
  }
}
```

### 9.2 Python - Consent Management

```python
from wia_embodiment_ethics import WiaEthics, ConsentRecord

async def manage_patient_consent():
    ethics = WiaEthics(audit_logging=True)

    await ethics.connect(
        embodiment_id='emb-001',
        framework_id='wia-hybrid-v1'
    )

    # Record informed consent
    consent_id = await ethics.record_consent(ConsentRecord(
        subject_id='patient-001',
        consent_type='informed',
        scope=['physical_assistance', 'health_monitoring'],
        granted=True,
        expiry='2026-01-01T00:00:00Z',
        metadata={
            'language': 'ko',
            'informed_about': ['capabilities', 'data_usage'],
            'guardian_consent': False
        }
    ))

    print(f'Consent recorded: {consent_id}')

    # Check consent before action
    consent_status = await ethics.check_consent(
        subject_id='patient-001',
        scope=['physical_assistance']
    )

    if consent_status.granted:
        print('Consent valid, proceeding with action')
    else:
        print(f'Consent not valid: {consent_status.reason}')

    await ethics.disconnect()
```

### 9.3 Real-Time Ethics Integration

```typescript
import { WiaEthics, WiaEmbodiment } from 'wia-ecosystem';

async function ethicsIntegratedControl() {
  const ethics = new WiaEthics();
  const embodiment = new WiaEmbodiment();

  await Promise.all([
    ethics.connect({ embodimentId: 'emb-001', frameworkId: 'wia-hybrid-v1' }),
    embodiment.connect({ embodimentId: 'emb-001' })
  ]);

  // Pre-action ethics check
  embodiment.onBeforeCommand(async (command) => {
    const evaluation = await ethics.evaluateAction({
      actionId: command.commandId,
      actionType: command.commandType,
      parameters: command
    });

    if (!evaluation.approved) {
      // Log decision
      await ethics.logDecision({
        decisionId: `dec-${Date.now()}`,
        actionId: command.commandId,
        outcome: 'blocked',
        reason: evaluation.explanation
      });

      throw new Error(`Ethics blocked: ${evaluation.explanation}`);
    }

    return command;
  });

  // Handle ethics events
  ethics.on('constraint_violation', (event) => {
    console.log('Constraint violation:', event.constraintId);
    embodiment.emergencyStop();
  });

  ethics.on('override_required', async (event) => {
    console.log('Human override required:', event.reason);
    // Wait for human decision
  });
}
```

---

## References

### Related Standards

- [WIA AI Embodiment Data Format (Phase 1)](/ai-embodiment/spec/PHASE-1-DATA-FORMAT.md)
- [WIA AI Embodiment Ethics Data Format (Phase 1)](/spec/PHASE-1-DATA-FORMAT.md)
- [WIA AI Human Coexistence](/ai-human-coexistence/)

### External References

- IEEE Ethically Aligned Design
- EU AI Act
- ISO/IEC TR 24028:2020 - AI Trustworthiness

---

<div align="center">

**WIA AI Embodiment Ethics API Interface Standard v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA Standards Committee**

**MIT License**

</div>
