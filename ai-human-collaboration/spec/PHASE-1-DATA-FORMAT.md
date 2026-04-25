# WIA-AI-015 Phase 1: Data Format Specification

> **Version:** 1.0.0
> **Status:** Stable
> **Last Updated:** 2025-12-25

## Overview

This document defines standard data formats for AI-human collaboration systems. Consistent data structures enable interoperability between different implementations and facilitate integration across platforms.

## Philosophy

Data formats are designed according to 弘益人間 (Hongik Ingan) principles:
- **Transparency**: Data structures are self-documenting
- **Privacy**: Personal information is protected by design
- **Extensibility**: Formats accommodate future enhancements
- **Accessibility**: JSON-based for universal compatibility

## Core Data Types

### 1. Collaboration Session

Represents an active collaboration between AI and human agents.

```json
{
  "session_id": "string (UUID)",
  "created_at": "ISO 8601 timestamp",
  "status": "active | paused | completed | failed",
  "participants": {
    "ai_agents": [
      {
        "agent_id": "string",
        "agent_type": "classifier | predictor | generator | analyzer",
        "model_version": "string",
        "capabilities": ["capability1", "capability2"]
      }
    ],
    "human_agents": [
      {
        "agent_id": "string (anonymized)",
        "role": "reviewer | expert | approver",
        "expertise_areas": ["area1", "area2"]
      }
    ]
  },
  "configuration": {
    "confidence_threshold": "number (0-1)",
    "escalation_strategy": "uncertainty | complexity | stakes | random",
    "feedback_enabled": "boolean"
  },
  "metadata": {
    "use_case": "string",
    "organization": "string",
    "compliance_requirements": ["requirement1", "requirement2"]
  }
}
```

### 2. Task

Represents a single unit of work to be processed collaboratively.

```json
{
  "task_id": "string (UUID)",
  "session_id": "string (UUID)",
  "created_at": "ISO 8601 timestamp",
  "updated_at": "ISO 8601 timestamp",
  "status": "pending | processing | ai_decided | human_review | completed | error",
  "priority": "low | medium | high | critical",
  "input_data": {
    "type": "string",
    "content": "object (schema varies by type)",
    "features": {
      "feature_name": "value"
    },
    "metadata": {
      "source": "string",
      "context": "object"
    }
  },
  "complexity_assessment": {
    "overall_complexity": "number (0-1)",
    "novelty_score": "number (0-1)",
    "feature_diversity": "number",
    "constraint_conflicts": "number"
  },
  "deadline": "ISO 8601 timestamp (optional)",
  "constraints": {
    "must_be_human": "boolean",
    "must_be_ai": "boolean",
    "required_expertise": ["expertise1"]
  }
}
```

### 3. AI Prediction

Represents AI model output with confidence and explanation.

```json
{
  "prediction_id": "string (UUID)",
  "task_id": "string (UUID)",
  "timestamp": "ISO 8601 timestamp",
  "agent_id": "string",
  "model_version": "string",
  "prediction": {
    "class": "string | number | object",
    "confidence": "number (0-1)",
    "probability_distribution": {
      "class_label": "probability"
    }
  },
  "explanation": {
    "feature_importance": [
      {
        "feature_name": "string",
        "importance": "number (0-1)",
        "direction": "positive | negative"
      }
    ],
    "similar_cases": [
      {
        "case_id": "string",
        "similarity": "number (0-1)",
        "outcome": "string"
      }
    ],
    "reasoning": "string (natural language explanation)",
    "contrastive": {
      "alternative": "string",
      "reasons": ["reason1", "reason2"]
    },
    "counterfactual": {
      "changes_needed": [
        {
          "feature": "string",
          "current_value": "any",
          "required_value": "any"
        }
      ]
    }
  },
  "metadata": {
    "inference_time_ms": "number",
    "model_uncertainty": "number (0-1)",
    "out_of_distribution_score": "number (0-1)"
  }
}
```

### 4. Human Decision

Represents human review and decision.

```json
{
  "decision_id": "string (UUID)",
  "task_id": "string (UUID)",
  "prediction_id": "string (UUID, optional)",
  "timestamp": "ISO 8601 timestamp",
  "reviewer_id": "string (anonymized)",
  "decision": {
    "action": "accept | reject | modify",
    "final_output": "object",
    "confidence": "number (0-1, optional)",
    "rationale": "string (optional)",
    "time_spent_seconds": "number"
  },
  "ai_interaction": {
    "viewed_prediction": "boolean",
    "viewed_explanation": "boolean",
    "modified_ai_suggestion": "boolean",
    "disagreed_with_ai": "boolean"
  },
  "feedback": {
    "ai_helpfulness": "number (1-5, optional)",
    "suggestion_quality": "number (1-5, optional)",
    "explanation_clarity": "number (1-5, optional)",
    "comments": "string (optional)"
  }
}
```

### 5. Escalation Event

Represents when and why a task was escalated to human review.

```json
{
  "escalation_id": "string (UUID)",
  "task_id": "string (UUID)",
  "prediction_id": "string (UUID)",
  "timestamp": "ISO 8601 timestamp",
  "trigger_type": "confidence | complexity | stakes | policy | random | manual",
  "trigger_details": {
    "confidence_below_threshold": "boolean",
    "threshold_value": "number (optional)",
    "complexity_score": "number (optional)",
    "stakes_level": "low | medium | high | critical (optional)",
    "policy_rule_triggered": "string (optional)"
  },
  "assigned_to": "string (reviewer_id)",
  "priority": "low | medium | high | critical",
  "context": {
    "similar_past_escalations": "number",
    "reviewer_workload": "number",
    "queue_depth": "number"
  }
}
```

### 6. Feedback Loop

Represents learning feedback from human decisions.

```json
{
  "feedback_id": "string (UUID)",
  "task_id": "string (UUID)",
  "decision_id": "string (UUID)",
  "timestamp": "ISO 8601 timestamp",
  "feedback_type": "correction | confirmation | enhancement",
  "learning_value": {
    "uncertainty_reduction": "number (0-1)",
    "representativeness": "number (0-1)",
    "expected_impact": "number (0-1)"
  },
  "incorporated": "boolean",
  "incorporation_timestamp": "ISO 8601 timestamp (optional)",
  "model_version_before": "string",
  "model_version_after": "string (optional)"
}
```

### 7. Performance Metrics

Represents system performance measurements.

```json
{
  "metrics_id": "string (UUID)",
  "session_id": "string (UUID, optional)",
  "period_start": "ISO 8601 timestamp",
  "period_end": "ISO 8601 timestamp",
  "productivity": {
    "tasks_completed": "number",
    "throughput_per_hour": "number",
    "average_cycle_time_seconds": "number",
    "efficiency_gain_percent": "number"
  },
  "quality": {
    "accuracy": "number (0-1)",
    "precision": "number (0-1)",
    "recall": "number (0-1)",
    "f1_score": "number (0-1)",
    "error_rate": "number (0-1)"
  },
  "collaboration": {
    "ai_acceptance_rate": "number (0-1)",
    "ai_modification_rate": "number (0-1)",
    "ai_rejection_rate": "number (0-1)",
    "average_review_time_seconds": "number",
    "feedback_quality_score": "number (0-1)"
  },
  "user_experience": {
    "satisfaction_score": "number (1-5)",
    "net_promoter_score": "number (-100 to 100)",
    "adoption_rate": "number (0-1)",
    "engagement_rate": "number (0-1)"
  }
}
```

### 8. Audit Log

Represents complete audit trail for compliance.

```json
{
  "log_id": "string (UUID)",
  "timestamp": "ISO 8601 timestamp",
  "event_type": "prediction | decision | escalation | feedback | error",
  "session_id": "string (UUID)",
  "task_id": "string (UUID)",
  "actors": {
    "ai_agents": ["agent_id"],
    "human_agents": ["agent_id (anonymized)"]
  },
  "event_data": {
    "before_state": "object (optional)",
    "after_state": "object",
    "changes": ["change_description"]
  },
  "accountability": {
    "primary_responsible": "ai | human",
    "responsible_party_id": "string",
    "oversight_party_id": "string (optional)"
  },
  "compliance": {
    "policy_version": "string",
    "regulations_applicable": ["regulation1"],
    "consent_obtained": "boolean"
  },
  "system_context": {
    "system_version": "string",
    "model_versions": {
      "model_name": "version"
    },
    "configuration_hash": "string"
  }
}
```

## Data Validation Rules

### Required Fields
All data structures MUST include:
- Unique identifier (`*_id` field)
- Timestamp (`created_at` or `timestamp`)
- Status or type indicator

### Data Types
- **UUID**: RFC 4122 compliant UUIDs
- **Timestamp**: ISO 8601 format with timezone
- **Numbers**: IEEE 754 double-precision
- **Strings**: UTF-8 encoded, max 10,000 characters unless specified
- **Arrays**: Max 1,000 elements unless specified
- **Objects**: Max nesting depth of 5 levels

### Privacy Requirements
- Personal identifiers MUST be anonymized or hashed
- Sensitive data MUST be encrypted at rest
- Access to data MUST be logged in audit trail
- Data retention MUST follow applicable regulations

## Versioning

Data format versions use semantic versioning:
- **Major**: Breaking changes to data structure
- **Minor**: Backward-compatible additions
- **Patch**: Clarifications and bug fixes

Current version: **1.0.0**

## Extension Fields

All data structures support custom extension fields under `"extensions"` key:

```json
{
  "standard_field": "value",
  "extensions": {
    "org_specific_field": "value",
    "custom_metadata": {}
  }
}
```

Extensions MUST NOT conflict with standard field names.

## Examples

### Complete Collaboration Flow

```json
// 1. Session Created
{
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "status": "active",
  "configuration": {
    "confidence_threshold": 0.75,
    "escalation_strategy": "uncertainty"
  }
}

// 2. Task Submitted
{
  "task_id": "660e8400-e29b-41d4-a716-446655440001",
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "status": "pending",
  "input_data": {
    "type": "customer_support_ticket",
    "content": {"text": "My order hasn't arrived"}
  }
}

// 3. AI Prediction
{
  "prediction_id": "770e8400-e29b-41d4-a716-446655440002",
  "task_id": "660e8400-e29b-41d4-a716-446655440001",
  "prediction": {
    "class": "shipping_inquiry",
    "confidence": 0.68
  }
}

// 4. Escalation (confidence below threshold)
{
  "escalation_id": "880e8400-e29b-41d4-a716-446655440003",
  "task_id": "660e8400-e29b-41d4-a716-446655440001",
  "trigger_type": "confidence",
  "trigger_details": {
    "confidence_below_threshold": true,
    "threshold_value": 0.75
  }
}

// 5. Human Decision
{
  "decision_id": "990e8400-e29b-41d4-a716-446655440004",
  "task_id": "660e8400-e29b-41d4-a716-446655440001",
  "decision": {
    "action": "modify",
    "final_output": {"class": "delivery_issue"},
    "confidence": 0.9
  }
}

// 6. Feedback Captured
{
  "feedback_id": "aa0e8400-e29b-41d4-a716-446655440005",
  "task_id": "660e8400-e29b-41d4-a716-446655440001",
  "feedback_type": "correction",
  "learning_value": {
    "uncertainty_reduction": 0.8
  }
}
```

## Compliance

This data format complies with:
- **GDPR**: Privacy by design, right to erasure
- **CCPA**: Consumer data rights
- **HIPAA**: Healthcare data protection (when applicable)
- **SOC 2**: Security and availability controls

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


## Annex E — Implementation Notes for PHASE-1-DATA-FORMAT

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1-DATA-FORMAT.

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
