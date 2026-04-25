# WIA BCI Consent Protocol
## Phase 2: API Interface Specification

**Version:** 1.0.0  
**Status:** Release  
**Last Updated:** 2025-12-25

---

## 1. Introduction

This specification defines the standard APIs for managing BCI consent throughout its lifecycle. All APIs follow REST principles and use JSON for request/response payloads.

## 2. Core API Endpoints

### 2.1 Request Consent

```
POST /api/v1/consent/request
Content-Type: application/json
Authorization: Bearer {token}

Request Body:
{
  "subjectId": "string",
  "consentType": "initial|ongoing|research|...",
  "deviceType": "invasive|semi_invasive|non_invasive",
  "purpose": "string",
  "permissions": {...},
  "jurisdiction": "string"
}

Response 201 Created:
{
  "consentId": "string",
  "status": "pending_signature",
  "expiresAt": "ISO 8601 timestamp",
  "consentDocument": "URL to PDF document"
}
```

### 2.2 Verify Consent

```
POST /api/v1/consent/verify
Content-Type: application/json

Request:
{
  "consentId": "string",
  "requiredPermissions": ["string"]
}

Response 200 OK:
{
  "valid": boolean,
  "permissions": {...},
  "expiresAt": "string",
  "capacityScore": number
}

Response 403 Forbidden (if invalid):
{
  "valid": false,
  "reason": "CONSENT_EXPIRED|CONSENT_REVOKED|PERMISSION_DENIED",
  "message": "string"
}
```

### 2.3 Modify Consent

```
PATCH /api/v1/consent/{id}
Content-Type: application/json

Request:
{
  "permissions": {...},  // Updated permissions
  "reason": "string"     // Reason for modification
}

Response 200 OK:
{
  "consentId": "string",
  "version": number,
  "effectiveDate": "string"
}
```

### 2.4 Revoke Consent

```
DELETE /api/v1/consent/{id}
Content-Type: application/json

Request:
{
  "reason": "string",
  "effectiveDate": "string"  // Optional, defaults to immediate
}

Response 200 OK:
{
  "consentId": "string",
  "status": "revoked",
  "revokedAt": "string",
  "dataRetentionPeriod": "string"
}
```

### 2.5 Capacity Assessment

```
POST /api/v1/capacity/assess
Content-Type: application/json

Request:
{
  "subjectId": "string",
  "cognitiveScore": number (0-100),
  "understandingLevel": number (0-10),
  "decisionAbility": number (0-10),
  "method": "string"
}

Response 200 OK:
{
  "capacityScore": number (0-100),
  "sufficient": boolean,
  "recommendation": "string",
  "assessmentId": "string"
}
```

### 2.6 Emergency Override

```
POST /api/v1/emergency/override
Authorization: Bearer {medical_professional_token}

Request:
{
  "subjectId": "string",
  "reason": "medical_emergency|seizure_detection|system_malfunction|immediate_danger",
  "situation": "string",
  "authorizingPhysician": "string",
  "license": "string",
  "duration": number  // hours
}

Response 200 OK:
{
  "overrideId": "string",
  "status": "active",
  "expiresAt": "string",
  "reviewRequired": true,
  "ethicsBoardNotified": boolean
}
```

### 2.7 Audit Log

```
GET /api/v1/consent/{id}/audit

Response 200 OK:
{
  "consentId": "string",
  "events": [
    {
      "timestamp": "string",
      "action": "string",
      "actor": "string",
      "details": "string",
      "ipAddress": "string"
    }
  ]
}
```

## 3. Authentication & Authorization

### 3.1 Authentication Methods

- **API Key**: For server-to-server communication
- **OAuth 2.0**: For user-facing applications
- **JWT Tokens**: For mobile/web applications
- **mTLS**: For high-security medical devices

### 3.2 Authorization Levels

| Role | Permissions |
|------|------------|
| Subject | View own consent, modify permissions, revoke |
| Clinician | Request consent, assess capacity, view audit |
| Researcher | Request consent, verify permissions |
| Administrator | All operations except subject-specific actions |
| System | Automated operations, webhooks |

## 4. Rate Limiting

```
HTTP Headers:
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1640462400

Response 429 Too Many Requests:
{
  "error": "rate_limit_exceeded",
  "message": "API rate limit exceeded. Limit: 1000 req/hour",
  "retryAfter": 3600
}
```

## 5. Error Codes

| Code | HTTP Status | Description |
|------|------------|-------------|
| CONSENT_NOT_FOUND | 404 | Consent ID does not exist |
| CONSENT_EXPIRED | 403 | Consent has passed expiration |
| CONSENT_REVOKED | 403 | Consent was revoked |
| INSUFFICIENT_CAPACITY | 422 | Subject lacks capacity |
| PERMISSION_DENIED | 403 | Required permission not granted |
| INVALID_SIGNATURE | 422 | Signature verification failed |
| UNAUTHORIZED | 401 | Invalid/missing authentication |
| RATE_LIMIT_EXCEEDED | 429 | Too many requests |

## 6. Webhooks

### 6.1 Webhook Events

- `consent.granted` - New consent granted
- `consent.modified` - Consent permissions modified
- `consent.revoked` - Consent revoked by subject
- `consent.expired` - Consent reached expiration
- `consent.renewed` - Consent successfully renewed
- `emergency.override.activated` - Emergency override triggered
- `capacity.assessment.failed` - Capacity assessment insufficient

### 6.2 Webhook Payload

```json
{
  "event": "consent.revoked",
  "timestamp": "2025-12-25T10:30:00Z",
  "data": {
    "consentId": "CST-001",
    "subjectId": "SUBJ-001",
    "reason": "User request"
  }
}
```

---

**弘益人間 (Hongik Ingan)** - *Benefit All Humanity*

© 2025 SmileStory Inc. / WIA

---

## Annex A — Conformance Tier Matrix

WIA conformance for bci-consent is evaluated across three tiers:

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

- `wia-standards/standards/bci-consent/api/` — TypeScript SDK skeleton
- `wia-standards/standards/bci-consent/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/bci-consent/simulator/` — interactive browser-based simulator for the PHASE protocol

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
