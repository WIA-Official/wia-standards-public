# WIA 3D Printing Construction Standard
## Phase 2: API Interface Specification
### Version 1.0.0

---

## 1. Introduction

Phase 2 defines RESTful APIs enabling system communication and data exchange. Building on Phase 1 data formats, these APIs support project management, material tracking, print job control, and quality monitoring.

### 1.1 Architecture

REST (Representational State Transfer) principles:
- Resource-oriented URLs
- Standard HTTP methods (GET, POST, PUT, DELETE)
- Stateless communication
- Standard status codes
- JSON payloads using Phase 1 schemas

### 1.2 Base URL Structure

```
https://{host}/wia/v1/{resource}/{id}
```

Example endpoints:
```
GET    /wia/v1/projects
POST   /wia/v1/projects
GET    /wia/v1/projects/{id}
PUT    /wia/v1/projects/{id}
DELETE /wia/v1/projects/{id}
```

## 2. Authentication

### 2.1 OAuth 2.0

REQUIRED authentication mechanism: OAuth 2.0

**Grant Types Supported:**
- `authorization_code` - User authentication
- `client_credentials` - Machine-to-machine

**Token Request:**
```http
POST /wia/v1/auth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id={client_id}
&client_secret={client_secret}
&scope=projects:read projects:write
```

**Token Response:**
```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "projects:read projects:write"
}
```

### 2.2 Permission Scopes

| Scope | Permissions |
|-------|-------------|
| `projects:read` | View project data |
| `projects:write` | Create/modify projects |
| `materials:read` | View material inventory |
| `materials:write` | Update inventory |
| `jobs:read` | View job status |
| `jobs:control` | Start/stop/pause jobs |
| `quality:read` | View quality data |
| `quality:write` | Submit inspections |

### 2.3 Using Access Tokens

```http
GET /wia/v1/projects
Authorization: Bearer {access_token}
```

## 3. Project Management API

### 3.1 Create Project

```http
POST /wia/v1/projects
Authorization: Bearer {token}
Content-Type: application/json

{
  "name": "Building A1",
  "type": "residential",
  "location": {...},
  "geometry": {...},
  "materials": {...}
}
```

**Response: 201 Created**
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "name": "Building A1",
  "status": "created",
  "createdAt": "2025-03-01T10:00:00Z",
  "links": {
    "self": "/wia/v1/projects/550e8400-e29b-41d4-a716-446655440000"
  }
}
```

### 3.2 List Projects

```http
GET /wia/v1/projects?status=active&limit=20&offset=0
```

**Query Parameters:**
- `status` - Filter by status (created|approved|printing|completed|failed)
- `type` - Filter by building type
- `limit` - Results per page (default: 20, max: 100)
- `offset` - Pagination offset

**Response: 200 OK**
```json
{
  "projects": [...],
  "total": 45,
  "limit": 20,
  "offset": 0,
  "links": {
    "next": "/wia/v1/projects?status=active&limit=20&offset=20"
  }
}
```

### 3.3 Get Project

```http
GET /wia/v1/projects/{id}
```

**Response: 200 OK**
Returns complete project data in Phase 1 format.

### 3.4 Update Project

```http
PUT /wia/v1/projects/{id}
Content-Type: application/json

{
  "status": "approved",
  "approvals": {...}
}
```

**Response: 200 OK**

### 3.5 Delete Project

```http
DELETE /wia/v1/projects/{id}
```

**Response: 204 No Content**

## 4. Material Management API

### 4.1 Get Inventory

```http
GET /wia/v1/materials/inventory
```

**Response: 200 OK**
```json
{
  "materials": [
    {
      "id": "WIA-CONCRETE-STD-001",
      "quantity": 15000,
      "unit": "kg",
      "location": "Silo A",
      "batchNumber": "20250301-A",
      "certifications": [...]
    }
  ]
}
```

### 4.2 Record Consumption

```http
POST /wia/v1/materials/consumption

{
  "projectId": "{project_id}",
  "materialId": "WIA-CONCRETE-STD-001",
  "quantity": 500,
  "unit": "kg",
  "jobId": "{job_id}"
}
```

**Response: 201 Created**

### 4.3 Submit Certification

```http
POST /wia/v1/materials/certifications

{
  "materialId": "WIA-CONCRETE-STD-001",
  "batchNumber": "20250301-A",
  "testType": "compressive-strength",
  "results": {...}
}
```

**Response: 201 Created**

## 5. Print Job API

### 5.1 Submit Job

```http
POST /wia/v1/jobs

{
  "projectId": "{project_id}",
  "name": "Building A1 - Layers 1-50",
  "priority": "normal",
  "schedule": {...},
  "layers": {"start": 1, "end": 50}
}
```

**Response: 201 Created**
```json
{
  "id": "job-12345",
  "status": "queued",
  "queuePosition": 2,
  "estimatedStartTime": "2025-03-12T06:00:00Z"
}
```

### 5.2 Get Job Status

```http
GET /wia/v1/jobs/{id}/status
```

**Response: 200 OK**
```json
{
  "id": "job-12345",
  "status": "printing",
  "progress": {
    "currentLayer": 23,
    "totalLayers": 50,
    "percentage": 0.46
  },
  "printer": {...},
  "quality": {...}
}
```

### 5.3 Control Job

```http
POST /wia/v1/jobs/{id}/pause
POST /wia/v1/jobs/{id}/resume
POST /wia/v1/jobs/{id}/cancel
```

**Response: 200 OK**

## 6. Quality Assurance API

### 6.1 Record Inspection

```http
POST /wia/v1/quality/inspections

{
  "jobId": "job-12345",
  "layer": 25,
  "inspector": "Jane Doe",
  "method": "laser-scan",
  "measurements": {...},
  "overallStatus": "pass"
}
```

**Response: 201 Created**

### 6.2 Get Quality Metrics

```http
GET /wia/v1/quality/metrics?jobId={job_id}
```

**Response: 200 OK**
```json
{
  "jobId": "job-12345",
  "metrics": {
    "dimensionalAccuracy": {...},
    "layerAdhesion": {...}
  },
  "overallScore": 94.3
}
```

### 6.3 Generate Compliance Report

```http
GET /wia/v1/quality/compliance-report?projectId={project_id}
```

**Response: 200 OK**

## 7. Error Handling

### 7.1 HTTP Status Codes

| Code | Meaning | Usage |
|------|---------|-------|
| 200 | OK | Successful request |
| 201 | Created | Resource created |
| 400 | Bad Request | Invalid data |
| 401 | Unauthorized | Missing/invalid auth |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource doesn't exist |
| 409 | Conflict | State conflict |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Error | Server error |

### 7.2 Error Response Format

```json
{
  "error": {
    "code": "INVALID_MATERIAL_SPECIFICATION",
    "message": "Material compressive strength must be between 10 and 100 MPa",
    "details": {
      "field": "materials.primary.properties.compressiveStrength",
      "providedValue": 150,
      "allowedRange": [10, 100]
    },
    "timestamp": "2025-03-12T10:15:00Z",
    "requestId": "req-abc123"
  }
}
```

## 8. Pagination and Filtering

### 8.1 Query Parameters

```http
GET /wia/v1/projects?status=active&type=residential&limit=20&offset=40&sortBy=createdAt&order=desc
```

### 8.2 Response Links

```json
{
  "projects": [...],
  "pagination": {
    "total": 125,
    "limit": 20,
    "offset": 40,
    "hasMore": true
  },
  "links": {
    "first": "...",
    "prev": "...",
    "self": "...",
    "next": "...",
    "last": "..."
  }
}
```

## 9. Webhooks

### 9.1 Register Webhook

```http
POST /wia/v1/webhooks

{
  "url": "https://your-system.example.com/webhooks/wia",
  "events": [
    "job.started",
    "job.completed",
    "job.failed"
  ],
  "secret": "your-webhook-secret"
}
```

### 9.2 Webhook Delivery

```http
POST https://your-system.example.com/webhooks/wia
X-WIA-Signature: sha256=...
Content-Type: application/json

{
  "event": "job.completed",
  "timestamp": "2025-03-12T14:30:00Z",
  "data": {...}
}
```

## 10. Rate Limiting

### 10.1 Limits

- Authenticated requests: 1000 requests/hour
- Unauthenticated requests: 60 requests/hour

### 10.2 Headers

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 523
X-RateLimit-Reset: 1647097200
```

### 10.3 Exceeded Limit

**Response: 429 Too Many Requests**
```json
{
  "error": {
    "code": "RATE_LIMIT_EXCEEDED",
    "retryAfter": 300
  }
}
```

## 11. Versioning

### 11.1 URL Versioning

API version in URL path: `/wia/v1/`, `/wia/v2/`

### 11.2 Header Versioning (Optional)

```http
Accept: application/vnd.wia.v1+json
```

### 11.3 Deprecation

Deprecated endpoints return:
```http
Warning: 299 - "Deprecated API. Please migrate to v2 by 2026-01-01"
```

## 12. Compliance Checklist

For Phase 2 certification:

**Basic Compliance:**
- ☐ OAuth 2.0 authentication
- ☐ All core project endpoints
- ☐ Material inventory endpoints
- ☐ Job submission and status
- ☐ Standard error responses
- ☐ Rate limiting

**Full Compliance (adds):**
- ☐ All optional endpoints
- ☐ Webhook support
- ☐ Advanced filtering
- ☐ Pagination with links
- ☐ Comprehensive error details

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

---

**Document Information:**
- **Version:** 1.0.0
- **Status:** Published
- **Date:** 2025-01-15
- **Depends on:** Phase 1 v1.0.0
- **License:** CC BY 4.0

**弘益人間 (Benefit All Humanity)**

© 2025 SmileStory Inc. / WIA
