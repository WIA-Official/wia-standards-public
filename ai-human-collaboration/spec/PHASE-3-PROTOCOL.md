# WIA-AI-015 Phase 3: Protocol Specification

> **Version:** 1.0.0
> **Status:** Stable
> **Last Updated:** 2025-12-25

## Overview

This document defines communication protocols for real-time AI-human collaboration, including handoff protocols, escalation workflows, and feedback mechanisms.

## Handoff Protocol

### AI-to-Human Handoff

When AI confidence falls below threshold or complexity exceeds limits:

```
1. AI ASSESSMENT
   ├─ Calculate confidence score
   ├─ Assess complexity metrics
   └─ Evaluate escalation criteria

2. HANDOFF DECISION
   ├─ Trigger type: confidence | complexity | stakes
   ├─ Urgency level: low | medium | high | critical
   └─ Required expertise: domain | specialized

3. CONTEXT PACKAGING
   ├─ Task data and features
   ├─ AI prediction and confidence
   ├─ Explanation and reasoning
   ├─ Similar past cases
   └─ Suggested actions

4. HUMAN NOTIFICATION
   ├─ Queue assignment based on expertise
   ├─ Priority routing
   └─ Context presentation

5. HANDOFF CONFIRMATION
   ├─ Human acknowledges receipt
   ├─ Estimated review time
   └─ Status update to AI system
```

### Human-to-AI Handoff

When human completes review and returns control to AI:

```
1. DECISION CAPTURE
   ├─ Final decision/output
   ├─ Confidence level
   ├─ Modifications made
   └─ Rationale (optional)

2. FEEDBACK EXTRACTION
   ├─ AI suggestion quality
   ├─ Explanation clarity
   ├─ Specific corrections
   └─ Learning signals

3. CONTEXT UPDATE
   ├─ Task status: completed
   ├─ Ground truth label
   ├─ Processing time
   └─ Outcome metrics

4. LEARNING INTEGRATION
   ├─ Add to training data
   ├─ Update model (if applicable)
   ├─ Refine confidence calibration
   └─ Improve explanations

5. ACKNOWLEDGMENT
   ├─ Confirm receipt
   ├─ Log audit trail
   └─ Update dashboards
```

## Escalation Workflow

### Escalation Triggers

1. **Confidence-Based**
   - Threshold: Configurable (default 0.75)
   - Condition: `confidence < threshold`
   - Action: Immediate escalation

2. **Complexity-Based**
   - Threshold: Configurable (default 0.6)
   - Condition: `complexity_score > threshold`
   - Action: Escalation with high priority

3. **Stakes-Based**
   - Condition: Financial, legal, or safety impact
   - Action: Mandatory human review

4. **Random Sampling**
   - Rate: Configurable (default 2%)
   - Purpose: Quality assurance
   - Action: Background review

### Escalation Routing

```
TASK ESCALATION
│
├─ PRIORITY ASSESSMENT
│  ├─ Critical: <15 min SLA
│  ├─ High: <1 hour SLA
│  ├─ Medium: <4 hours SLA
│  └─ Low: <24 hours SLA
│
├─ EXPERTISE MATCHING
│  ├─ Domain requirements
│  ├─ Complexity level
│  └─ Historical performance
│
├─ WORKLOAD BALANCING
│  ├─ Current queue depth
│  ├─ Reviewer availability
│  └─ Capacity limits
│
└─ ASSIGNMENT
   ├─ Primary reviewer
   ├─ Backup reviewer
   └─ Escalation path
```

## Feedback Loop Protocol

### Implicit Feedback

Captured automatically from user actions:

```json
{
  "feedback_type": "implicit",
  "signals": {
    "accepted_ai_suggestion": "boolean",
    "modified_ai_suggestion": "boolean",
    "time_spent_seconds": "number",
    "viewed_explanation": "boolean",
    "requested_alternatives": "boolean"
  },
  "interpretation": {
    "agreement_with_ai": "high | medium | low",
    "task_difficulty": "easy | medium | hard",
    "ai_helpfulness": "high | medium | low"
  }
}
```

### Explicit Feedback

Requested from users:

```json
{
  "feedback_type": "explicit",
  "ratings": {
    "prediction_quality": "1-5",
    "explanation_clarity": "1-5",
    "overall_helpfulness": "1-5"
  },
  "structured": {
    "error_type": "wrong_class | low_confidence | poor_explanation",
    "severity": "minor | moderate | major | critical"
  },
  "freeform": {
    "what_went_wrong": "string",
    "what_would_help": "string"
  }
}
```

## Real-Time Collaboration

### WebSocket Protocol

For live collaboration:

```
CLIENT -> SERVER: CONNECT
{
  "session_id": "uuid",
  "agent_id": "human-reviewer-42",
  "capabilities": ["review", "annotate", "feedback"]
}

SERVER -> CLIENT: CONNECTED
{
  "connection_id": "uuid",
  "queue_depth": 5,
  "next_task": {...}
}

CLIENT -> SERVER: REQUEST_TASK
{
  "priority": "high",
  "expertise": ["domain_x"]
}

SERVER -> CLIENT: TASK_ASSIGNED
{
  "task_id": "uuid",
  "deadline": "ISO 8601",
  "ai_prediction": {...},
  "context": {...}
}

CLIENT -> SERVER: SUBMIT_DECISION
{
  "task_id": "uuid",
  "decision": {...},
  "feedback": {...}
}

SERVER -> CLIENT: DECISION_ACCEPTED
{
  "task_id": "uuid",
  "next_task": {...}
}
```

## Synchronization Protocol

### State Synchronization

Ensure consistency across distributed components:

```
1. OPTIMISTIC UPDATES
   - Client updates UI immediately
   - Server processes in background
   - Conflict resolution if needed

2. VERSION CONTROL
   - Each state change has version number
   - Concurrent modifications detected
   - Last-write-wins or manual merge

3. EVENT SOURCING
   - All changes logged as events
   - State rebuilt from event log
   - Enables replay and debugging

4. EVENTUAL CONSISTENCY
   - Temporary inconsistencies allowed
   - Convergence guaranteed
   - Conflict-free replicated data types
```

## Error Handling Protocol

### Error Categories

```
1. TRANSIENT ERRORS
   - Network timeouts
   - Temporary overload
   - Action: Retry with backoff

2. INVALID INPUT
   - Malformed data
   - Missing required fields
   - Action: Return error, don't retry

3. SYSTEM FAILURES
   - Database down
   - Model unavailable
   - Action: Graceful degradation

4. BUSINESS LOGIC ERRORS
   - Policy violations
   - Constraint violations
   - Action: Human review required
```

### Recovery Protocol

```
ERROR DETECTED
│
├─ LOG ERROR
│  ├─ Error type and code
│  ├─ Stack trace
│  ├─ Context data
│  └─ Timestamp
│
├─ NOTIFY STAKEHOLDERS
│  ├─ Error severity
│  ├─ Impact assessment
│  └─ Recovery ETA
│
├─ ATTEMPT AUTO-RECOVERY
│  ├─ Retry transient errors
│  ├─ Fallback to backup systems
│  └─ Degrade gracefully if needed
│
└─ ESCALATE IF NEEDED
   ├─ Human intervention
   ├─ System restart
   └─ Manual recovery
```

## Security Protocol

### Data Protection

- **Encryption in Transit**: TLS 1.3+
- **Encryption at Rest**: AES-256
- **Key Management**: Rotate every 90 days
- **Access Control**: Role-based, least privilege

### Audit Trail

Every interaction logged:

```json
{
  "timestamp": "ISO 8601",
  "event": "human_decision",
  "actor": "anonymized_id",
  "action": "approved",
  "resource": "task:uuid",
  "outcome": "success",
  "ip_address": "hashed",
  "user_agent": "redacted"
}
```

---

**弘益人間** (Hongik Ingan) · Benefit All Humanity

© 2025 SmileStory Inc. / WIA
