# WIA-AI-015 Phase 2: API Specification

> **Version:** 1.0.0
> **Status:** Stable
> **Last Updated:** 2025-12-25

## Overview

This document defines the RESTful API for AI-human collaboration systems. The API enables creation, management, and monitoring of collaborative workflows.

## Base URL

```
https://api.example.com/v1/collaboration
```

## Authentication

All requests require authentication using API keys:

```http
Authorization: Bearer <your-api-key>
```

## Core Endpoints

### Sessions

#### Create Session

```http
POST /sessions
Content-Type: application/json

{
  "configuration": {
    "confidence_threshold": 0.75,
    "escalation_strategy": "uncertainty",
    "feedback_enabled": true
  },
  "participants": {
    "ai_agents": [...],
    "human_agents": [...]
  }
}

Response: 201 Created
{
  "session_id": "uuid",
  "status": "active",
  "created_at": "2025-12-25T10:00:00Z"
}
```

#### Get Session

```http
GET /sessions/{session_id}

Response: 200 OK
{
  "session_id": "uuid",
  "status": "active",
  "configuration": {...},
  "metrics": {...}
}
```

### Tasks

#### Submit Task

```http
POST /sessions/{session_id}/tasks
Content-Type: application/json

{
  "input_data": {
    "type": "classification",
    "content": {...},
    "features": {...}
  },
  "priority": "high",
  "deadline": "2025-12-25T18:00:00Z"
}

Response: 202 Accepted
{
  "task_id": "uuid",
  "status": "pending",
  "estimated_completion": "2025-12-25T10:05:00Z"
}
```

#### Get Task Status

```http
GET /tasks/{task_id}

Response: 200 OK
{
  "task_id": "uuid",
  "status": "completed",
  "result": {...},
  "processing_history": [...]
}
```

### Predictions

#### Submit Prediction

```http
POST /tasks/{task_id}/predictions
Content-Type: application/json

{
  "agent_id": "ai-model-v1",
  "prediction": {
    "class": "category_a",
    "confidence": 0.87
  },
  "explanation": {...}
}

Response: 201 Created
{
  "prediction_id": "uuid",
  "escalation_required": false
}
```

### Human Review

#### Get Review Queue

```http
GET /review/queue?reviewer_id={id}&priority=high

Response: 200 OK
{
  "queue_depth": 15,
  "tasks": [
    {
      "task_id": "uuid",
      "priority": "high",
      "ai_prediction": {...},
      "context": {...}
    }
  ]
}
```

#### Submit Decision

```http
POST /tasks/{task_id}/decisions
Content-Type: application/json

{
  "reviewer_id": "reviewer-123",
  "decision": {
    "action": "modify",
    "final_output": {...},
    "confidence": 0.95
  },
  "feedback": {
    "ai_helpfulness": 4,
    "comments": "..."
  }
}

Response: 201 Created
{
  "decision_id": "uuid",
  "task_completed": true
}
```

### Metrics

#### Get Performance Metrics

```http
GET /sessions/{session_id}/metrics?period=7d

Response: 200 OK
{
  "period_start": "2025-12-18T00:00:00Z",
  "period_end": "2025-12-25T00:00:00Z",
  "productivity": {...},
  "quality": {...},
  "collaboration": {...}
}
```

## Webhooks

Subscribe to events:

```http
POST /webhooks
Content-Type: application/json

{
  "url": "https://your-app.com/webhook",
  "events": ["task.completed", "escalation.created"],
  "secret": "your-webhook-secret"
}
```

## Rate Limits

- **Free tier**: 100 requests/minute
- **Pro tier**: 1,000 requests/minute
- **Enterprise**: Custom limits

## Error Responses

```json
{
  "error": {
    "code": "INVALID_THRESHOLD",
    "message": "Confidence threshold must be between 0 and 1",
    "details": {...}
  }
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


## Annex E — Implementation Notes for PHASE-2-API

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2-API.

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
