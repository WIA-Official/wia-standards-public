# PHASE 2: Advanced Features

**Timeline:** Q2 2026 - Q4 2026
**Focus:** Enhanced capabilities for enterprise deployments

## 2.1 Delegation Framework

### 2.1.1 Overview

Delegation allows users to temporarily grant their permissions to other users, enabling flexible workflows while maintaining security.

### 2.1.2 Delegation Types

#### Administrative Delegation
Administrators delegate administrative privileges:
```json
{
  "delegationType": "administrative",
  "delegator": "admin@example.com",
  "delegate": "deputy@example.com",
  "delegatedRoles": ["user-manager", "policy-editor"],
  "constraints": {
    "validFrom": "2025-12-26T00:00:00Z",
    "validUntil": "2025-12-31T23:59:59Z",
    "maxDepth": 0,
    "revocable": true
  }
}
```

#### Workflow Delegation
Task-specific delegation for business processes:
```json
{
  "delegationType": "workflow",
  "delegator": "manager@example.com",
  "delegate": "assistant@example.com",
  "delegatedPermissions": [
    {
      "action": "approve",
      "resource": "/expenses/pending/*",
      "conditions": {
        "amount": { "lte": 1000 }
      }
    }
  ],
  "validUntil": "2025-12-27T17:00:00Z"
}
```

#### Emergency Delegation
Break-glass scenarios for critical situations:
```json
{
  "delegationType": "emergency",
  "reason": "Primary admin unavailable - system outage",
  "delegate": "oncall@example.com",
  "delegatedRoles": ["emergency-admin"],
  "requiresJustification": true,
  "auditLevel": "CRITICAL",
  "autoExpire": "6h"
}
```

### 2.1.3 Delegation Chain Tracking

```json
{
  "delegationChain": [
    {
      "level": 0,
      "userId": "ceo@example.com",
      "role": "executive"
    },
    {
      "level": 1,
      "userId": "vp@example.com",
      "delegatedBy": "ceo@example.com",
      "delegationId": "del-001",
      "validUntil": "2025-12-30T00:00:00Z"
    },
    {
      "level": 2,
      "userId": "manager@example.com",
      "delegatedBy": "vp@example.com",
      "delegationId": "del-002",
      "validUntil": "2025-12-28T00:00:00Z"
    }
  ],
  "maxDepthAllowed": 3,
  "currentDepth": 2
}
```

## 2.2 Dynamic Policy Generation

### 2.2.1 Context-Aware Policies

Policies that adapt based on runtime context:

```json
{
  "policyId": "adaptive-access-001",
  "type": "dynamic",
  "generator": {
    "function": "riskBasedAccess",
    "parameters": {
      "baselineRisk": "medium",
      "factors": [
        {
          "factor": "location",
          "weight": 0.3,
          "mapping": {
            "office": 0.1,
            "vpn": 0.3,
            "public": 0.8
          }
        },
        {
          "factor": "deviceTrust",
          "weight": 0.4,
          "mapping": {
            "managed": 0.1,
            "byod-enrolled": 0.4,
            "unknown": 0.9
          }
        },
        {
          "factor": "timeOfDay",
          "weight": 0.3,
          "mapping": {
            "businessHours": 0.2,
            "afterHours": 0.6
          }
        }
      ]
    }
  },
  "rules": {
    "lowRisk": {
      "threshold": 0.3,
      "action": "PERMIT",
      "additionalAuth": false
    },
    "mediumRisk": {
      "threshold": 0.6,
      "action": "PERMIT",
      "additionalAuth": "MFA"
    },
    "highRisk": {
      "threshold": 1.0,
      "action": "DENY",
      "notification": ["security-team"]
    }
  }
}
```

### 2.2.2 Policy Templates

Reusable policy patterns:

```json
{
  "templateId": "department-data-access",
  "parameters": [
    { "name": "department", "type": "string", "required": true },
    { "name": "classification", "type": "string", "default": "internal" },
    { "name": "allowedActions", "type": "array", "default": ["read"] }
  ],
  "policyTemplate": {
    "policyId": "dept-{{department}}-access",
    "target": {
      "resources": ["/data/{{department}}/*"]
    },
    "rule": {
      "effect": "PERMIT",
      "condition": {
        "allOf": [
          { "match": { "subject.department": "{{department}}" } },
          { "match": { "resource.classification": "{{classification}}" } },
          { "match": { "action": { "in": "{{allowedActions}}" } } }
        ]
      }
    }
  }
}
```

## 2.3 Advanced Attribute Providers

### 2.3.1 External Attribute Sources

Integration with external identity and attribute providers:

```json
{
  "attributeProviderId": "hr-system",
  "type": "REST",
  "endpoint": "https://hr.example.com/api/employee/{userId}",
  "authentication": {
    "type": "OAuth2",
    "tokenEndpoint": "https://hr.example.com/oauth/token"
  },
  "attributeMapping": {
    "employee_id": "subject.employeeId",
    "department": "subject.department",
    "manager_id": "subject.managerId",
    "job_title": "subject.jobTitle",
    "clearance_level": "subject.clearanceLevel"
  },
  "cache": {
    "enabled": true,
    "ttl": 3600,
    "invalidateOn": ["user_update", "role_change"]
  }
}
```

### 2.3.2 Computed Attributes

Attributes calculated at evaluation time:

```json
{
  "computedAttributes": {
    "isBusinessHours": {
      "type": "boolean",
      "expression": "environment.time >= '09:00' && environment.time <= '17:00' && !isWeekend(environment.date)"
    },
    "riskScore": {
      "type": "number",
      "expression": "calculateRisk(subject.history, environment.location, environment.deviceTrust)"
    },
    "effectiveRole": {
      "type": "string",
      "expression": "resolveHighestRole(subject.roles, delegation.activeRoles)"
    }
  }
}
```

## 2.4 Multi-Tenancy Support

### 2.4.1 Tenant Isolation

```json
{
  "tenantId": "corp-a",
  "isolationLevel": "strict",
  "policyNamespace": "/tenants/corp-a/policies",
  "resourceNamespace": "/tenants/corp-a/resources",
  "crossTenantAccess": {
    "enabled": false,
    "allowedTenants": [],
    "sharedResources": []
  }
}
```

### 2.4.2 Hierarchical Organizations

```json
{
  "organizationId": "global-corp",
  "structure": {
    "root": "global-corp",
    "subsidiaries": [
      {
        "id": "corp-us",
        "inheritsFrom": "global-corp",
        "overridablePolicies": ["data-retention", "privacy"],
        "children": [
          { "id": "corp-us-west", "inheritsFrom": "corp-us" },
          { "id": "corp-us-east", "inheritsFrom": "corp-us" }
        ]
      },
      {
        "id": "corp-eu",
        "inheritsFrom": "global-corp",
        "additionalPolicies": ["gdpr-compliance"]
      }
    ]
  }
}
```

---

---

## Annex A — Conformance Tier Matrix

WIA conformance for access-control is evaluated across three tiers:

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

- `wia-standards/standards/access-control/api/` — TypeScript SDK skeleton
- `wia-standards/standards/access-control/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/access-control/simulator/` — interactive browser-based simulator for the PHASE protocol

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


## Annex E — Implementation Notes for PHASE-2-IMPLEMENTATION

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2-IMPLEMENTATION.

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

