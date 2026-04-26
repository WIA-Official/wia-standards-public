# WIA E-Waste Management Standard
# Phase 2: API Interface Specification v1.0

## Document Information
- **Standard**: WIA E-Waste Management  
- **Phase**: 2 - API Interface
- **Version**: 1.0.0
- **Status**: Published
- **Date**: 2025-01-15

## 1. Overview

Phase 2 defines RESTful API specifications enabling system integration across e-waste management stakeholders. These APIs facilitate device tracking, collection management, processing documentation, and compliance reporting.

## 2. Architecture Principles

### 2.1 REST Design
- Stateless HTTP-based communication
- Resource-oriented URLs
- Standard HTTP methods (GET, POST, PUT, PATCH, DELETE)
- JSON request/response bodies
- OAuth 2.0 authentication

### 2.2 Base URL Structure
```
https://api.wia-ewaste.org/v1
```

## 3. Core Endpoints

### 3.1 Device Management

**POST /api/v1/devices**  
Register new device with manufacturer declaration
```json
Request:
{
  "device": { /* Phase 1 device object */ }
}
Response: 201 Created
{
  "device_id": "550e8400-e29b-41d4-a716-446655440000",
  "qr_code_url": "https://api.wia-ewaste.org/qr/550e8400",
  "created_at": "2025-01-15T10:00:00Z"
}
```

**GET /api/v1/devices/{device_id}**  
Retrieve device information
```
Response: 200 OK
{
  "device": { /* Full device object with lifecycle history */ }
}
```

### 3.2 Collection API

**POST /api/v1/collections**
Log collection event
```json
Request:
{
  "device_ids": ["uuid1", "uuid2"],
  "collector_id": "COL-12345",
  "location": {...},
  "collection_method": "retail_takeback"
}
Response: 201 Created
{
  "collection_id": "col_xyz",
  "devices_count": 2
}
```

### 3.3 Processing API

**POST /api/v1/processing/stages**
Document processing stage
```json
Request:
{
  "stage_type": "dismantling",
  "input_devices": ["uuid1"],
  "facility_id": "RC-US-CA-001",
  "outputs": [...]
}
```

### 3.4 Recovery API

**POST /api/v1/recovery/materials**
Report material recovery
```json
Request:
{
  "source_devices": ["uuid1", "uuid2"],
  "facility_id": "RC-US-CA-001",
  "recovered_materials": [...]
}
```

## 4. Authentication

### 4.1 OAuth 2.0 Flow
```
POST /api/v1/oauth/token
Request:
{
  "grant_type": "client_credentials",
  "client_id": "your_client_id",
  "client_secret": "your_secret",
  "scope": "devices:read collections:write"
}
Response:
{
  "access_token": "eyJhbG...",
  "token_type": "Bearer",
  "expires_in": 3600
}
```

### 4.2 Roles and Permissions
| Role | Permissions |
|------|-------------|
| Manufacturer | devices:write, devices:read (own products) |
| Collector | collections:write, devices:read |
| Processor | processing:write, recovery:write, devices:read |
| Regulator | *:read, reports:generate |
| Consumer | devices:read (own devices) |

## 5. Rate Limiting
- 1000 requests/hour for standard tier
- 10000 requests/hour for enterprise tier
- Headers: `X-RateLimit-Limit`, `X-RateLimit-Remaining`, `X-RateLimit-Reset`
- HTTP 429 when exceeded

## 6. Error Responses
```json
HTTP 400 Bad Request
{
  "error": "validation_error",
  "message": "Request body failed schema validation",
  "details": [
    {"field": "weight_kg", "error": "must be positive"}
  ]
}
```

Standard codes: 200 OK, 201 Created, 400 Bad Request, 401 Unauthorized, 403 Forbidden, 404 Not Found, 409 Conflict, 429 Too Many Requests, 500 Server Error

## 7. Webhooks
Register webhook endpoints for event notifications:
```
POST /api/v1/webhooks
{
  "url": "https://your-server.com/webhook",
  "events": ["device.collected", "processing.complete"]
}
```

## 8. Compliance Reporting
```
GET /api/v1/compliance/reports/epr?jurisdiction=EU&period=2025-Q1
GET /api/v1/compliance/reports/basel?shipment_id=SHP-123
GET /api/v1/compliance/certificates/{device_id}
```

## 9. Versioning
- URL-based: `/v1/`, `/v2/`
- 2-year support for previous version
- 6-month deprecation notice
- Version info in response headers

## 10. Performance Requirements
- 95th percentile response time < 500ms
- 99.5% uptime (excluding maintenance)
- Support 100 requests/second per facility

---
© 2025 SmileStory Inc. / WIA · 弘益人間


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

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-2-API-INTERFACE. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-2-api-interface/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-2-API-INTERFACE with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-2-API-INTERFACE does not require bespoke
auditor tooling.
