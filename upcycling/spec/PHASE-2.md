# WIA Upcycling Standard
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
- `type` - Filter by product type
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

---

**Document Information:**
- **Version:** 1.0.0
- **Status:** Published
- **Date:** 2025-01-15
- **Depends on:** Phase 1 v1.0.0
- **License:** CC BY 4.0

**弘益人間 (Benefit All Humanity)**

© 2025 SmileStory Inc. / WIA

---

## 12. Webhook Subsystem

The standard recommends, and Full Compliance requires, a webhook subsystem so that integrating systems can react to upcycling events without polling.

### 12.1 Webhook Registration

A consumer registers a webhook with a target URL and a list of event topics:

```http
POST /wia/v1/webhooks
Content-Type: application/json
Authorization: Bearer <token>

{
  "url": "https://consumer.example.com/wia-upcycling-events",
  "events": ["project.completed", "material.depleted", "job.failed"],
  "secret": "<HMAC verification secret>"
}
```

The server returns a webhook identifier and the time-of-next-rotation for the secret.

### 12.2 Delivery Semantics

The webhook delivery is at-least-once with exponential backoff. The server retries failed deliveries on a declared schedule and gives up after the configured horizon. Each delivery carries a monotonically increasing sequence number per webhook so that a consumer can detect gaps.

The server signs the body with HMAC-SHA-256 over the registered secret. The consumer must verify the signature before acting on the payload.

### 12.3 Required Events

The standard requires the following event topics to be available:

| Topic | Triggers |
|-------|---------|
| `project.created` | new upcycling project registered |
| `project.completed` | project closed; final material accounting available |
| `material.intake` | a new material has entered inventory |
| `material.depleted` | material is below the configured threshold |
| `job.submitted` | a process job has been submitted |
| `job.completed` | a job has finished with success or failure |
| `audit.event` | a non-routine audit event has been recorded |

A deployment may add proprietary events; it must not omit required events.

## 13. Pagination & Filtering

Collection endpoints support cursor-based pagination via `?cursor=<opaque>&limit=<n>`. Cursor stability across requests is guaranteed within the cursor's TTL declared in the endpoint documentation.

Filter parameters follow the form `?filter[field]=value` with documented field names per resource. A consumer that needs filters not in the documented set must request them via the standard's change process; ad-hoc filter parameters are non-conformant.

## 14. Idempotency

Mutating endpoints (POST, PUT, DELETE) accept an optional `Idempotency-Key` header. When provided, the server records the key and the response, and a retry with the same key returns the same response without re-applying the mutation. Idempotency keys are scoped to the authenticated client and expire after the configured horizon.

A consumer that submits a project create twice without an idempotency key may produce two projects; the standard recommends idempotency for any client that retries on network failure.

## 15. Conformance Tests for Phase 2

Phase 2 conformance is demonstrated by, at minimum:

- a request to each required endpoint returning the documented schema and status code,
- an authentication test exercising OAuth 2.0 client_credentials,
- a webhook delivery test exercising at-least-once delivery and HMAC verification,
- an idempotency test demonstrating retry safety.

Test reports are signed by the commissioner.
