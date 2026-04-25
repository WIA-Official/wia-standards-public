# PHASE 2: API INTERFACE SPECIFICATION
## WIA-CONTACT-001: First Contact Protocol

> 弘益人間 (홍익인간) · Benefit All Humanity

---

## 1. Introduction

This document specifies the Application Programming Interface (API) for the First Contact Protocol, enabling developers to integrate detection, verification, analysis, and response capabilities.

## 2. API Overview

### 2.1 Base URL
```
Production: https://api.wia-contact.org/v1
Staging: https://staging-api.wia-contact.org/v1
Development: http://localhost:8080/v1
```

### 2.2 Authentication
```http
Authorization: Bearer {API_KEY}
X-WIA-Client-ID: {CLIENT_ID}
X-WIA-Timestamp: {UNIX_TIMESTAMP}
X-WIA-Signature: {HMAC_SHA256_SIGNATURE}
```

### 2.3 Rate Limiting
- Standard tier: 1,000 requests/hour
- Premium tier: 10,000 requests/hour
- Enterprise tier: Unlimited
- Headers: `X-RateLimit-Limit`, `X-RateLimit-Remaining`, `X-RateLimit-Reset`

## 3. Detection APIs

### 3.1 Detect Signal
**Endpoint**: `POST /signals/detect`

**Request**:
```json
{
  "frequency": 1420.4,
  "source": {
    "ra": 19.45,
    "dec": 38.78
  },
  "strength": -140,
  "duration": 60,
  "modulation": "unknown",
  "metadata": {}
}
```

**Response** (201 Created):
```json
{
  "signalId": "SIG-2025-0001",
  "status": "detected",
  "timestamp": "2025-12-27T14:23:45.123Z",
  "verificationRequired": true,
  "nextSteps": [
    "initiate-verification",
    "begin-analysis"
  ]
}
```

### 3.2 Get Signal
**Endpoint**: `GET /signals/{signalId}`

**Response** (200 OK):
```json
{
  "signalId": "SIG-2025-0001",
  "detectionTimestamp": "2025-12-27T14:23:45.123Z",
  "frequency": {
    "center": 1420.4,
    "bandwidth": 0.1,
    "unit": "MHz"
  },
  "source": {
    "rightAscension": 19.45,
    "declination": 38.78
  },
  "strength": {
    "value": -140,
    "unit": "dBm"
  },
  "status": "under-verification",
  "confidence": 0.87
}
```

### 3.3 List Signals
**Endpoint**: `GET /signals`

**Query Parameters**:
- `startDate`: ISO8601 timestamp
- `endDate`: ISO8601 timestamp
- `minStrength`: number
- `verified`: boolean
- `threatLevel`: string
- `limit`: number (default: 100)
- `offset`: number (default: 0)

**Response** (200 OK):
```json
{
  "signals": [],
  "pagination": {
    "total": 142,
    "limit": 100,
    "offset": 0,
    "hasMore": true
  }
}
```

### 3.4 Monitor Frequency
**Endpoint**: `POST /monitoring/start`

**Request**:
```json
{
  "frequency": 1420.4,
  "bandwidth": 10,
  "duration": 3600,
  "callback": "https://your-server.com/webhook"
}
```

**Response** (202 Accepted):
```json
{
  "monitoringId": "MON-2025-0001",
  "status": "active",
  "expiresAt": "2025-12-27T15:23:45.123Z"
}
```

## 4. Verification APIs

### 4.1 Initiate Verification
**Endpoint**: `POST /signals/{signalId}/verify`

**Request**:
```json
{
  "multiSiteConfirmation": true,
  "falsePositiveCheck": true,
  "scientificConsensus": 0.95,
  "timeout": 172800000
}
```

**Response** (202 Accepted):
```json
{
  "verificationId": "VER-2025-0001",
  "signalId": "SIG-2025-0001",
  "status": "in-progress",
  "estimatedCompletion": "2025-12-29T14:23:45.123Z"
}
```

### 4.2 Get Verification Status
**Endpoint**: `GET /verifications/{verificationId}`

**Response** (200 OK):
```json
{
  "verificationId": "VER-2025-0001",
  "signalId": "SIG-2025-0001",
  "status": "completed",
  "observatories": [
    {
      "observatoryId": "OBS-001",
      "observatoryName": "Arecibo",
      "confirmed": true,
      "confidence": 0.92,
      "timestamp": "2025-12-28T10:15:30.000Z"
    }
  ],
  "consensus": 0.96,
  "authentic": true,
  "completedAt": "2025-12-28T12:30:00.000Z"
}
```

### 4.3 Request Additional Verification
**Endpoint**: `POST /verifications/{verificationId}/request-additional`

**Request**:
```json
{
  "observatories": ["OBS-005", "OBS-006"],
  "urgency": "high"
}
```

## 5. Analysis APIs

### 5.1 Analyze Pattern
**Endpoint**: `POST /signals/{signalId}/analyze`

**Request**:
```json
{
  "analysisType": "pattern-recognition",
  "parameters": {
    "searchPrimes": true,
    "searchFibonacci": true,
    "searchConstants": true
  }
}
```

**Response** (200 OK):
```json
{
  "analysisId": "ANA-2025-0001",
  "signalId": "SIG-2025-0001",
  "patternType": "prime-sequence",
  "mathematicalSignificance": "high",
  "artificialProbability": 0.94,
  "entropy": 0.23,
  "decodedContent": "Prime sequence: 2, 3, 5, 7, 11...",
  "interpretation": "Intentional mathematical communication"
}
```

### 5.2 Assess Threat
**Endpoint**: `POST /signals/{signalId}/assess-threat`

**Response** (200 OK):
```json
{
  "assessmentId": "THR-2025-0001",
  "signalId": "SIG-2025-0001",
  "threatLevel": "benign",
  "indicators": [
    {
      "type": "mathematical-pattern",
      "description": "Prime number sequence suggests peaceful communication",
      "severity": "low"
    }
  ],
  "confidence": 0.91,
  "recommendation": "Proceed with diplomatic response protocol"
}
```

## 6. Response APIs

### 6.1 Generate Universal Message
**Endpoint**: `POST /responses/generate`

**Request**:
```json
{
  "type": "mathematical-acknowledgment",
  "targetSignalId": "SIG-2025-0001"
}
```

**Response** (200 OK):
```json
{
  "responseId": "RES-2025-0001",
  "content": "π = 3.14159...\ne = 2.71828...\nPrime acknowledgment: 2, 3, 5, 7, 11...",
  "encoding": "text",
  "checksum": "sha256:abc123..."
}
```

### 6.2 Create Response
**Endpoint**: `POST /responses`

**Request**:
```json
{
  "signalId": "SIG-2025-0001",
  "type": "mathematical-acknowledgment",
  "content": "Custom message content",
  "transmission": {
    "frequency": 1420.4,
    "power": 1000,
    "duration": 300,
    "targetCoordinates": {
      "ra": 19.45,
      "dec": 38.78
    }
  }
}
```

**Response** (201 Created):
```json
{
  "responseId": "RES-2025-0001",
  "status": "draft",
  "requiresApproval": true
}
```

### 6.3 Submit for Review
**Endpoint**: `POST /responses/{responseId}/submit`

**Response** (202 Accepted):
```json
{
  "responseId": "RES-2025-0001",
  "status": "under-review",
  "reviewers": [
    "UN Security Council",
    "International Scientific Committee"
  ],
  "estimatedDecision": "2025-12-30T00:00:00.000Z"
}
```

### 6.4 Transmit Response
**Endpoint**: `POST /responses/{responseId}/transmit`

**Request**:
```json
{
  "authorization": {
    "unApprovalId": "UN-APP-2025-001",
    "scientificConsensus": 0.97
  }
}
```

**Response** (200 OK):
```json
{
  "responseId": "RES-2025-0001",
  "status": "transmitted",
  "transmissionTimestamp": "2025-12-30T12:00:00.000Z",
  "confirmation": "TX-CONF-2025-001"
}
```

## 7. Observatory APIs

### 7.1 List Observatories
**Endpoint**: `GET /observatories`

**Response** (200 OK):
```json
{
  "observatories": [
    {
      "id": "OBS-001",
      "name": "Arecibo",
      "location": {
        "latitude": 18.3464,
        "longitude": -66.7528,
        "altitude": 497
      },
      "capabilities": ["detection", "verification", "transmission"],
      "status": "active"
    }
  ]
}
```

### 7.2 Get Observatory Status
**Endpoint**: `GET /observatories/{observatoryId}/status`

## 8. Webhook System

### 8.1 Webhook Events
- `signal.detected`
- `signal.verified`
- `pattern.decoded`
- `threat.assessed`
- `response.approved`
- `response.transmitted`

### 8.2 Webhook Payload
```json
{
  "event": "signal.detected",
  "timestamp": "2025-12-27T14:23:45.123Z",
  "data": {
    "signalId": "SIG-2025-0001"
  }
}
```

### 8.3 Register Webhook
**Endpoint**: `POST /webhooks`

**Request**:
```json
{
  "url": "https://your-server.com/webhook",
  "events": ["signal.detected", "signal.verified"],
  "secret": "your-webhook-secret"
}
```

## 9. Error Handling

### 9.1 Error Response Format
```json
{
  "error": {
    "code": "INVALID_FREQUENCY",
    "message": "Frequency must be between 1 MHz and 100 GHz",
    "details": {
      "provided": 150000,
      "min": 1,
      "max": 100000
    }
  }
}
```

### 9.2 Error Codes
- `AUTHENTICATION_FAILED`: 401
- `AUTHORIZATION_ERROR`: 403
- `RESOURCE_NOT_FOUND`: 404
- `VALIDATION_ERROR`: 422
- `RATE_LIMIT_EXCEEDED`: 429
- `INTERNAL_ERROR`: 500

## 10. SDK Libraries

### 10.1 Official SDKs
- TypeScript/JavaScript: `@wia/contact-001`
- Python: `wia-contact-001`
- Java: `org.wia.contact.v1`
- Go: `github.com/wia-official/contact-001-go`
- Rust: `wia_contact_001`

### 10.2 Example Usage (TypeScript)
```typescript
import { FirstContactProtocol } from '@wia/contact-001';

const protocol = new FirstContactProtocol({
  apiKey: process.env.WIA_API_KEY
});

const signal = await protocol.detectSignal({
  frequency: 1420.4,
  source: { ra: 19.45, dec: 38.78 },
  strength: -140
});

console.log(`Signal detected: ${signal.signalId}`);
```

---

**Document Version**: 1.0.0
**Last Updated**: 2025-12-27
**Status**: Active
**Maintainer**: WIA Technical Committee

© 2025 SmileStory Inc. / WIA · CC BY-SA 4.0

---

## Annex A — Conformance Tier Matrix

WIA conformance for WIA-CONTACT-001-first-contact-protocol is evaluated across three tiers:

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

- `wia-standards/standards/WIA-CONTACT-001-first-contact-protocol/api/` — TypeScript SDK skeleton
- `wia-standards/standards/WIA-CONTACT-001-first-contact-protocol/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/WIA-CONTACT-001-first-contact-protocol/simulator/` — interactive browser-based simulator for the PHASE protocol

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


## Annex E — Implementation Notes for PHASE-2-API-INTERFACE

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2-API-INTERFACE.

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
