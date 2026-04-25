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

---

## Annex A — Conformance Tier Matrix

WIA conformance for ai-human-collaboration is evaluated across three tiers:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references published standards alongside this document. Implementers SHOULD review the relevant standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- ISO/IEC 17065:2012 — Conformity assessment — Requirements for bodies certifying products, processes and services
- ISO/IEC 27001:2022 — Information security management systems
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- W3C PROV-DM — provenance data model

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/ai-human-collaboration/api/` — TypeScript SDK skeleton
- `wia-standards/standards/ai-human-collaboration/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/ai-human-collaboration/simulator/` — interactive browser-based simulator for the PHASE protocol

Each reference artifact ships with an MIT license and is intended as a starting point, not as a production-grade implementation.

### C.2 Test Vectors

A normative set of request/response pairs covering the schemas defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector in both serialization (request) and deserialization (response) directions, and MUST publish a signed report identifying which vectors were exercised.

Test vectors are versioned independently of the PHASE document; refer to the `test-vectors/` directory for the active version.

---

## Annex D — Open Questions and Future Work

This PHASE document captures the consensus position at v1.0. The following items are tracked for future minor or major revisions:

1. **Schema versioning policy** — formal MUST/SHOULD rules for backward-compatible vs. breaking changes between minor releases.
2. **Privacy-preserving aggregation** — guidance on differential privacy and secure aggregation patterns for telemetry channels, without prescribing a specific algorithm.
3. **Multilingual error catalogs** — localization strategy for the standard error codes defined in this PHASE.
4. **Long-term retention** — alignment with sectoral retention regulations across jurisdictions.
5. **Sustainability disclosure** — optional fields for energy and emissions reporting tied to operations covered by this PHASE.

Items in this annex are non-normative. Comments and proposals are accepted via the GitHub issues tracker on the WIA-Official organization.

---


## Annex E — Implementation Notes for PHASE-3-PROTOCOL

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-3-PROTOCOL.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.
