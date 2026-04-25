# WIA-CRYO-010 PHASE 3: PROTOCOLS

**Standard**: WIA-CRYO-010  
**Phase**: 3 - API and Communication Protocols  
**Version**: 1.0.0  
**Date**: January 2025  
**Status**: Active  

## Overview

Phase 3 defines REST API specifications, WebSocket protocols, and data exchange mechanisms for cryopreservation research systems.

## 3.1 RESTful API Specification

### Base URL
```
https://api.wia.org/cryo-research/v1
```

### Authentication
All requests require OAuth 2.0 Bearer token:
```
Authorization: Bearer {access_token}
```

### Endpoints

#### GET /experiments
Query experiments with filters
```http
GET /experiments?cell_type=hepatocyte&viability_min=80&limit=50&offset=0
```

Response (200 OK):
```json
{
  "results": [...],
  "total": 145,
  "page": 1,
  "pageSize": 50,
  "links": {
    "self": "/experiments?page=1",
    "next": "/experiments?page=2"
  }
}
```

#### GET /experiments/{id}
Retrieve specific experiment
```http
GET /experiments/exp-2025-001
```

#### POST /experiments
Submit new experiment
```http
POST /experiments
Content-Type: application/ld+json

{experiment JSON-LD}
```

Response (201 Created):
```json
{
  "status": "created",
  "experimentId": "exp-2025-456",
  "location": "/experiments/exp-2025-456"
}
```

#### PUT /experiments/{id}
Update experiment
```http
PUT /experiments/exp-2025-001
```

#### DELETE /experiments/{id}
Archive experiment (soft delete)

### Error Responses
```json
{
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Required field 'principalInvestigator' missing",
    "details": {...}
  }
}
```

Status codes:
- 400: Bad Request
- 401: Unauthorized
- 403: Forbidden
- 404: Not Found
- 409: Conflict
- 422: Unprocessable Entity
- 500: Internal Server Error

## 3.2 WebSocket Protocol

Real-time data streaming for monitoring.

### Connection
```javascript
ws://api.wia.org/cryo-research/ws/v1
```

### Message Format
```json
{
  "type": "temperature_update",
  "experimentId": "exp-2025-001",
  "timestamp": "2025-01-21T10:15:23.456Z",
  "data": {
    "probe": "T001",
    "temperature": -125.3,
    "unit": "°C"
  }
}
```

### Subscription
```json
{
  "action": "subscribe",
  "channel": "experiment.exp-2025-001.temperature"
}
```

## 3.3 Data Export Formats

### CSV Export
```http
GET /experiments/exp-2025-001/export?format=csv
```

Header row required, UTF-8 encoding.

### Bulk Export
```http
POST /bulk-export
{
  "experimentIds": ["exp-2025-001", "exp-2025-002"],
  "format": "json-ld",
  "compression": "gzip"
}
```

Returns job ID for async processing.

## 3.4 Pagination

Cursor-based pagination for large datasets:
```http
GET /experiments?cursor=eyJpZCI6MTIzfQ&limit=50
```

Response includes next cursor:
```json
{
  "results": [...],
  "nextCursor": "eyJpZCI6MTczfQ",
  "hasMore": true
}
```

## 3.5 Rate Limiting

- 1000 requests/hour per API key
- 100 concurrent WebSocket connections

Headers:
```
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 950
X-RateLimit-Reset: 1642780800
```

## 3.6 Webhooks

Register webhooks for event notifications:
```http
POST /webhooks
{
  "url": "https://myapp.com/webhook",
  "events": ["experiment.completed", "viability.low_alert"],
  "secret": "shared_secret_for_hmac"
}
```

Webhook payload:
```json
{
  "event": "experiment.completed",
  "experimentId": "exp-2025-001",
  "timestamp": "2025-01-21T15:30:00Z",
  "data": {...}
}
```

HMAC signature in header:
```
X-WIA-Signature: sha256=abc123...
```

## 3.7 GraphQL API (Optional)

```graphql
query {
  experiments(filter: {cellType: "hepatocyte", viabilityMin: 80}) {
    experimentId
    title
    results {
      viability
      recovery
    }
  }
}
```

## 3.8 FHIR Integration

For clinical cryopreservation data, support FHIR resources:
- Specimen
- Procedure
- Observation

## 3.9 Data Validation

All submissions validated against JSON Schema before acceptance.
Validation errors returned with 422 status.

---

**Previous**: [PHASE-2: Algorithms](PHASE-2.md)  
**Next**: [PHASE-4: Integration](PHASE-4.md)

© 2025 SmileStory Inc. / WIA  
弘益人間 (Hongik Ingan) · Benefit All Humanity

---

## Annex A — Conformance Tier Matrix

WIA conformance for cryo-research is evaluated across three tiers:

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

- `wia-standards/standards/cryo-research/api/` — TypeScript SDK skeleton
- `wia-standards/standards/cryo-research/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/cryo-research/simulator/` — interactive browser-based simulator for the PHASE protocol

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


## Annex E — Implementation Notes for PHASE-3

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-3.

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
