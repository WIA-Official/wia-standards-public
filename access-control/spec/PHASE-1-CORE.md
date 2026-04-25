# WIA-SEC-010: Access Control - Phase 1 Core Specification

**Version:** 1.0
**Status:** DRAFT
**Last Updated:** 2025-12-25
**Standard ID:** WIA-SEC-010
**Category:** Security (SEC)
**Color:** #8B5CF6

---

## 1. Introduction

### 1.1 Purpose

WIA-SEC-010 defines a comprehensive, standardized approach to access control that enables organizations to implement secure, scalable, and flexible authorization mechanisms. This specification provides:

- **Unified Framework**: A single standard supporting multiple access control models (RBAC, ABAC, MAC)
- **Interoperability**: Standard formats and protocols for cross-system authorization
- **Scalability**: High-performance decision engines capable of millions of evaluations per second
- **Auditability**: Complete audit trails for compliance and security analysis
- **Flexibility**: Extensible policy language supporting complex authorization scenarios

### 1.2 Scope

This Phase 1 specification covers:

- Core access control models and concepts
- Standard data formats for policies, roles, and permissions
- Authorization decision protocols
- Basic policy evaluation algorithms
- Audit and logging requirements
- Integration patterns with existing systems

Out of scope for Phase 1 (covered in later phases):
- Advanced delegation mechanisms
- Distributed policy evaluation
- Machine learning-based anomaly detection
- Blockchain-based audit trails

### 1.3 Philosophy

弘益人間 (Hongik Ingan) - Benefit All Humanity

Access control should protect resources while enabling legitimate users to perform their duties efficiently. Security mechanisms must be:

- **Transparent**: Clear policies that users can understand
- **Fair**: Consistent application of rules without discrimination
- **Proportional**: Security measures appropriate to risk levels
- **Accountable**: Complete audit trails for all decisions
- **Empowering**: Enable users rather than obstruct them

---

## 2. Core Concepts

### 2.1 Access Control Models

#### 2.1.1 Role-Based Access Control (RBAC)

RBAC assigns permissions to roles rather than individual users. Users are then assigned to roles based on their responsibilities.

**Key Components:**
- **Users**: Human or system entities requiring access
- **Roles**: Named collections of permissions
- **Permissions**: Authorizations to perform specific actions on resources
- **Sessions**: User-role associations with temporal scope

**Example:**
```json
{
  "model": "RBAC",
  "roles": {
    "admin": {
      "permissions": ["read", "write", "delete", "manage_users"],
      "resources": ["*"]
    },
    "editor": {
      "permissions": ["read", "write"],
      "resources": ["/documents/*", "/media/*"]
    },
    "viewer": {
      "permissions": ["read"],
      "resources": ["/documents/public/*"]
    }
  },
  "roleHierarchy": {
    "admin": ["editor", "viewer"],
    "editor": ["viewer"]
  }
}
```

#### 2.1.2 Attribute-Based Access Control (ABAC)

ABAC makes authorization decisions based on attributes of subjects, resources, actions, and environmental context.

**Attribute Categories:**
- **Subject Attributes**: User properties (department, clearance level, location)
- **Resource Attributes**: Object properties (classification, owner, creation date)
- **Action Attributes**: Operation properties (read, write, delete)
- **Environmental Attributes**: Context (time, location, threat level)

**Example Policy:**
```json
{
  "model": "ABAC",
  "policy": {
    "policyId": "financial-data-access",
    "description": "Access to financial data requires finance department membership and high clearance",
    "rule": {
      "effect": "PERMIT",
      "condition": {
        "allOf": [
          {
            "match": {
              "subject.department": "finance"
            }
          },
          {
            "match": {
              "subject.clearanceLevel": { "gte": 3 }
            }
          },
          {
            "match": {
              "resource.classification": "financial"
            }
          },
          {
            "match": {
              "environment.time": { "between": ["09:00", "17:00"] }
            }
          }
        ]
      }
    }
  }
}
```

#### 2.1.3 Mandatory Access Control (MAC)

MAC enforces system-wide policies that cannot be changed by individual users. Access is determined by comparing security labels.

**Key Concepts:**
- **Security Labels**: Hierarchical classifications (TOP_SECRET, SECRET, CONFIDENTIAL, UNCLASSIFIED)
- **Clearance Levels**: User authorization levels
- **Read Down / Write Up**: Bell-LaPadula model principles
- **Compartments**: Non-hierarchical categories (NUCLEAR, CRYPTO, INTEL)

**Example:**
```json
{
  "model": "MAC",
  "securityPolicy": "Bell-LaPadula",
  "classifications": {
    "levels": [
      { "name": "TOP_SECRET", "rank": 4 },
      { "name": "SECRET", "rank": 3 },
      { "name": "CONFIDENTIAL", "rank": 2 },
      { "name": "UNCLASSIFIED", "rank": 1 }
    ],
    "compartments": ["NUCLEAR", "CRYPTO", "INTEL", "NATO"]
  },
  "rules": {
    "read": "subject.clearance >= resource.classification",
    "write": "subject.clearance <= resource.classification"
  }
}
```

---

## 3. Standard Data Formats

### 3.1 Policy Document Format

All access control policies MUST be representable in the standard WIA-SEC-010 JSON format.

#### 3.1.1 Policy Structure

```json
{
  "wiaVersion": "1.0",
  "standard": "WIA-SEC-010",
  "policySet": {
    "policySetId": "corporate-access-policies",
    "version": "2.1.0",
    "description": "Corporate access control policies",
    "combiningAlgorithm": "deny-unless-permit",
    "policies": [
      {
        "policyId": "policy-001",
        "description": "Administrative access policy",
        "target": {
          "resources": ["/admin/*"],
          "actions": ["*"]
        },
        "rule": {
          "effect": "PERMIT",
          "condition": {
            "match": {
              "subject.role": "administrator"
            }
          }
        }
      }
    ]
  },
  "metadata": {
    "author": "security-team@example.com",
    "created": "2025-01-01T00:00:00Z",
    "lastModified": "2025-12-25T10:00:00Z",
    "effectiveDate": "2025-12-26T00:00:00Z",
    "expirationDate": "2026-12-31T23:59:59Z"
  }
}
```

#### 3.1.2 Required Fields

- **wiaVersion**: WIA-SEC-010 specification version
- **standard**: Must be "WIA-SEC-010"
- **policySet.policySetId**: Unique identifier for the policy set
- **policySet.version**: Semantic version of the policy set
- **policySet.combiningAlgorithm**: How to combine multiple policy decisions

#### 3.1.3 Combining Algorithms

- **permit-overrides**: First PERMIT decision wins
- **deny-overrides**: First DENY decision wins (default)
- **first-applicable**: First applicable policy's decision wins
- **deny-unless-permit**: DENY unless at least one PERMIT
- **permit-unless-deny**: PERMIT unless at least one DENY

### 3.2 Role Definition Format

```json
{
  "roleId": "data-analyst",
  "displayName": "Data Analyst",
  "description": "Analysts with access to data analysis tools and datasets",
  "permissions": [
    {
      "action": "read",
      "resource": "/datasets/*"
    },
    {
      "action": "execute",
      "resource": "/analytics-tools/*"
    }
  ],
  "inherits": ["basic-user"],
  "constraints": {
    "maxConcurrentSessions": 3,
    "allowedNetworks": ["10.0.0.0/8", "192.168.1.0/24"],
    "timeRestrictions": {
      "allowedDays": ["monday", "tuesday", "wednesday", "thursday", "friday"],
      "allowedHours": { "start": "08:00", "end": "18:00" }
    }
  },
  "metadata": {
    "created": "2025-01-15T00:00:00Z",
    "owner": "hr-department",
    "approvalRequired": true
  }
}
```

### 3.3 Authorization Request Format

```json
{
  "requestId": "req-abc123def456",
  "timestamp": "2025-12-25T14:30:00Z",
  "subject": {
    "userId": "alice@example.com",
    "roles": ["data-analyst", "team-lead"],
    "attributes": {
      "department": "analytics",
      "clearanceLevel": 3,
      "employeeId": "E12345",
      "groups": ["analysts", "managers"]
    }
  },
  "resource": {
    "resourceId": "/datasets/customer-behavior",
    "type": "dataset",
    "attributes": {
      "classification": "confidential",
      "owner": "marketing-dept",
      "createdDate": "2025-01-01T00:00:00Z"
    }
  },
  "action": {
    "actionId": "read",
    "attributes": {
      "method": "API",
      "bulkOperation": false
    }
  },
  "environment": {
    "ipAddress": "192.168.1.100",
    "userAgent": "Mozilla/5.0...",
    "deviceId": "device-xyz789",
    "deviceTrust": "high",
    "location": "HQ-Building-A",
    "threatLevel": "low"
  }
}
```

### 3.4 Authorization Response Format

```json
{
  "requestId": "req-abc123def456",
  "decision": "PERMIT",
  "timestamp": "2025-12-25T14:30:00.123Z",
  "evaluationTime": 12,
  "obligations": [
    {
      "obligationId": "log-access",
      "parameters": {
        "logLevel": "INFO",
        "includeDetails": true
      }
    },
    {
      "obligationId": "rate-limit",
      "parameters": {
        "maxRequests": 100,
        "timeWindow": "1h"
      }
    }
  ],
  "advice": [
    {
      "adviceId": "recommend-mfa",
      "message": "Consider enabling MFA for sensitive data access"
    }
  ],
  "appliedPolicies": [
    "policy-001",
    "policy-rbac-analyst"
  ],
  "validUntil": "2025-12-25T15:30:00Z",
  "metadata": {
    "policyVersion": "2.1.0",
    "evaluationEngine": "WIA-SEC-010-Engine/1.0",
    "pdpId": "pdp-primary-01"
  }
}
```

#### 3.4.1 Decision Values

- **PERMIT**: Access is allowed
- **DENY**: Access is denied
- **NOT_APPLICABLE**: No applicable policy found
- **INDETERMINATE**: Error during evaluation

---

## 4. Authorization Protocol

### 4.1 Protocol Architecture

The WIA-SEC-010 architecture follows the XACML reference model with these components:

```
┌─────────────┐
│   Client    │
└──────┬──────┘
       │ (1) Access Request
       ▼
┌─────────────────────┐
│        PEP          │ Policy Enforcement Point
│  (Enforcement)      │
└──────┬──────────────┘
       │ (2) Authorization Request
       ▼
┌─────────────────────┐
│        PDP          │ Policy Decision Point
│  (Decision)         │◄─────────┐
└──────┬──────────────┘          │
       │ (3) Retrieve Attributes │
       ▼                         │
┌─────────────────────┐          │
│        PIP          │──────────┘
│  (Information)      │
└─────────────────────┘
       │
       ▼
┌─────────────────────┐
│        PAP          │ Policy Administration Point
│  (Management)       │
└─────────────────────┘
```

### 4.2 Request Flow

#### Step 1: Client Access Request
Client attempts to access a protected resource.

#### Step 2: PEP Intercepts Request
The Policy Enforcement Point (PEP) intercepts the request and formulates an authorization request to the PDP.

#### Step 3: PDP Retrieves Context
The Policy Decision Point (PDP) queries the Policy Information Point (PIP) for additional attributes needed for policy evaluation.

#### Step 4: Policy Evaluation
PDP evaluates applicable policies against the request context.

#### Step 5: Decision Response
PDP returns authorization decision to PEP.

#### Step 6: Enforcement
PEP enforces the decision, either allowing access or denying it.

### 4.3 REST API Endpoints

#### 4.3.1 Authorization Endpoint

**Request:**
```
POST /api/v1/authorize
Content-Type: application/json
Authorization: Bearer <api-token>

{
  "subject": { ... },
  "resource": { ... },
  "action": { ... },
  "environment": { ... }
}
```

**Response:**
```
HTTP/1.1 200 OK
Content-Type: application/json

{
  "decision": "PERMIT",
  "obligations": [ ... ],
  "validUntil": "2025-12-25T15:30:00Z"
}
```

#### 4.3.2 Batch Authorization Endpoint

For efficiency, multiple authorization requests can be batched:

```
POST /api/v1/authorize/batch
Content-Type: application/json

{
  "requests": [
    { "requestId": "req-1", "subject": {...}, "resource": {...}, "action": {...} },
    { "requestId": "req-2", "subject": {...}, "resource": {...}, "action": {...} }
  ]
}
```

**Response:**
```json
{
  "responses": [
    { "requestId": "req-1", "decision": "PERMIT" },
    { "requestId": "req-2", "decision": "DENY" }
  ]
}
```

---

## 5. Policy Evaluation Engine

### 5.1 Evaluation Algorithm

```
FUNCTION Evaluate(request, policySet):
  applicablePolicies = FindApplicablePolicies(request, policySet)

  IF applicablePolicies.isEmpty():
    RETURN NOT_APPLICABLE

  decisions = []
  FOR EACH policy IN applicablePolicies:
    decision = EvaluatePolicy(request, policy)
    decisions.append(decision)

  combinedDecision = CombineDecisions(decisions, policySet.combiningAlgorithm)

  RETURN combinedDecision
```

### 5.2 Condition Evaluation

Conditions support various operators:

- **Comparison**: `eq`, `ne`, `lt`, `lte`, `gt`, `gte`
- **String**: `contains`, `startsWith`, `endsWith`, `matches` (regex)
- **Set**: `in`, `notIn`, `subset`, `superset`
- **Logical**: `and`, `or`, `not`
- **Temporal**: `before`, `after`, `between`

**Example:**
```json
{
  "condition": {
    "allOf": [
      { "match": { "subject.department": { "in": ["finance", "accounting"] } } },
      { "match": { "subject.clearance": { "gte": 2 } } },
      { "match": { "environment.time": { "between": ["09:00", "17:00"] } } }
    ]
  }
}
```

### 5.3 Performance Requirements

- **Decision Latency**: < 10ms (p99) for simple policies
- **Throughput**: > 10,000 decisions/second per PDP instance
- **Cache Hit Ratio**: > 90% for repeated queries
- **Policy Loading**: < 100ms for policy updates

---

## 6. Audit and Logging

### 6.1 Audit Log Format

All authorization decisions MUST be logged in this format:

```json
{
  "eventId": "evt-xyz123",
  "timestamp": "2025-12-25T14:30:00.123Z",
  "eventType": "AUTHORIZATION_DECISION",
  "decision": "PERMIT",
  "subject": {
    "userId": "alice@example.com",
    "ipAddress": "192.168.1.100"
  },
  "resource": "/datasets/customer-behavior",
  "action": "read",
  "appliedPolicies": ["policy-001"],
  "evaluationTime": 12,
  "pdpId": "pdp-primary-01",
  "signature": "SHA256:abc123...",
  "metadata": {
    "sessionId": "session-xyz",
    "requestId": "req-abc123def456"
  }
}
```

### 6.2 Audit Requirements

- **Completeness**: All decisions MUST be logged
- **Integrity**: Logs MUST be tamper-proof (signed or hashed)
- **Retention**: Minimum 90 days, configurable up to 7 years
- **Performance**: Logging MUST NOT add > 1ms to decision latency
- **Privacy**: PII should be hashed or redacted based on policy

---

## 7. Security Considerations

### 7.1 Threat Model

Protected against:
- **Privilege Escalation**: Unauthorized role assumption
- **Policy Bypass**: Circumventing access controls
- **Information Disclosure**: Leaking sensitive policy information
- **Denial of Service**: Resource exhaustion attacks

### 7.2 Security Requirements

- **Transport Security**: TLS 1.3+ required for all API communications
- **Authentication**: PEP and PIP MUST authenticate to PDP
- **API Security**: Rate limiting and API key rotation
- **Policy Integrity**: Digital signatures on policy documents
- **Secrets Management**: Secure storage of credentials and keys

---

## 8. Compliance

### 8.1 Supported Standards

WIA-SEC-010 aligns with:
- **XACML 3.0**: eXtensible Access Control Markup Language
- **OAuth 2.0**: Authorization framework
- **SAML 2.0**: Security Assertion Markup Language
- **GDPR**: Privacy and data protection
- **SOC 2**: Security controls
- **ISO 27001**: Information security management

---

## 9. Implementation Guidelines

### 9.1 Minimum Implementation

A conformant WIA-SEC-010 implementation MUST support:
- RBAC model with role hierarchy
- JSON policy format
- REST API authorization endpoint
- Audit logging
- At least one combining algorithm (deny-overrides)

### 9.2 Recommended Features

- ABAC model support
- Policy caching with configurable TTL
- Batch authorization endpoint
- Policy validation tools
- Performance monitoring

---

## Appendix A: Examples

See separate examples document for complete implementation samples.

## Appendix B: References

- XACML 3.0 Specification
- OAuth 2.0 RFC 6749
- NIST RBAC Model
- Bell-LaPadula Security Model

---

**Document Control**

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-25 | WIA Security Working Group | Initial release |

---

弘益人間 (홍익인간) · Benefit All Humanity

© 2025 SmileStory Inc. / WIA - World Certification Industry Association

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


## Annex E — Implementation Notes for PHASE-1-CORE

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1-CORE.

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

