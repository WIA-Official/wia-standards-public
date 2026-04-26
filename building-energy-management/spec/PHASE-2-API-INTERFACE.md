# WIA Building Energy Management Standard
## Phase 2: API Interface Specification
### Version 1.0

---

## Document Information

- **Standard**: WIA-BEMS (Building Energy Management Standard)
- **Phase**: 2 - API Interface
- **Version**: 1.0.0
- **Status**: Published
- **Date**: January 2025
- **Organization**: WIA (World Certification Industry Association)
- **License**: Open Standard (Freely Implementable)

---

## 1. Introduction

Phase 2 defines RESTful API interfaces that enable applications to access, control, and manage building energy systems using the standardized data formats established in Phase 1. These APIs eliminate custom integration code, enable a competitive marketplace of applications, and provide foundation for sophisticated energy management.

### 1.1 Design Principles

- **RESTful Architecture**: Resource-oriented design using standard HTTP methods
- **Stateless Communication**: Each request contains all necessary information
- **HATEOAS**: Hypermedia links guide client navigation
- **Standard Status Codes**: Consistent HTTP status code usage
- **JSON Format**: All requests and responses use JSON

---

## 2. Authentication and Authorization

### 2.1 OAuth 2.0

WIA-BEMS APIs use OAuth 2.0 for authentication, supporting:
- **Authorization Code Flow**: For user applications with browser
- **Client Credentials Flow**: For service accounts and backend systems
- **Refresh Token Flow**: For maintaining long-lived sessions

### 2.2 Token Structure

```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIs...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "refresh_token": "tGzv3JOkF0XG5Qx2TlKWIA",
  "scope": "building:read energy:read equipment:control"
}
```

### 2.3 Scopes

- `building:read`: Read building information and configuration
- `building:write`: Modify building configuration
- `energy:read`: Read energy consumption and demand data
- `energy:write`: Submit energy data (for data providers)
- `equipment:read`: Read equipment status
- `equipment:control`: Send commands to equipment
- `analytics:read`: Access analytics and optimization results
- `analytics:execute`: Trigger analytics calculations

---

## 3. Core API Endpoints

### 3.1 Buildings

**GET /api/v1/buildings**
List all accessible buildings

Query Parameters:
- `limit`: Results per page (default: 50, max: 200)
- `offset`: Pagination offset (default: 0)
- `sort`: Sort field (name, area, created_at)

Response: 200 OK
```json
{
  "buildings": [{
    "building_id": "BLDG-001",
    "name": "Headquarters Tower",
    "area_sqm": 25000,
    "type": "commercial_office",
    "links": {
      "self": "/api/v1/buildings/BLDG-001",
      "meters": "/api/v1/buildings/BLDG-001/meters",
      "energy": "/api/v1/buildings/BLDG-001/energy"
    }
  }],
  "pagination": {
    "total": 125,
    "limit": 50,
    "offset": 0
  }
}
```

**GET /api/v1/buildings/{building_id}**
Get building details

**PUT /api/v1/buildings/{building_id}**
Update building information (requires building:write scope)

### 3.2 Energy Data

**GET /api/v1/buildings/{building_id}/energy**
Query energy consumption data

Query Parameters:
- `start`: ISO 8601 timestamp (required)
- `end`: ISO 8601 timestamp (required)
- `interval`: Aggregation interval (1m, 5m, 15m, 1h, 1d)
- `meter_id`: Filter by specific meter
- `floor`: Filter by floor
- `zone`: Filter by zone
- `quality`: Minimum quality level (verified, estimated, questionable)

Response: 200 OK
```json
{
  "building_id": "BLDG-001",
  "data": [{
    "timestamp_start": "2025-01-01T00:00:00Z",
    "timestamp_end": "2025-01-01T01:00:00Z",
    "energy_kwh": 125.5,
    "cost_usd": 18.75,
    "quality": "verified"
  }],
  "summary": {
    "total_energy_kwh": 95240.5,
    "avg_power_kw": 128.0,
    "peak_power_kw": 205.3
  }
}
```

**GET /api/v1/buildings/{building_id}/realtime**
WebSocket endpoint for real-time data streaming

### 3.3 Equipment Control

**GET /api/v1/buildings/{building_id}/equipment**
List all controllable equipment

**POST /api/v1/buildings/{building_id}/equipment/{equipment_id}/commands**
Send control command

Request Body:
```json
{
  "command": "set_temperature",
  "parameters": {
    "supply_air_temp_setpoint_c": 14.0
  },
  "authorization": {
    "override_reason": "Optimization test",
    "duration_minutes": 60
  }
}
```

Response: 202 Accepted
```json
{
  "command_id": "cmd-12345",
  "status": "accepted",
  "links": {
    "status": "/api/v1/commands/cmd-12345"
  }
}
```

---

## 4. Error Handling

### 4.1 Error Response Format

```json
{
  "error": {
    "code": "ERROR_CODE",
    "message": "Human-readable description",
    "details": {
      "field": "additional context"
    },
    "timestamp": "2025-01-15T14:30:00Z",
    "request_id": "req-abc123"
  }
}
```

### 4.2 Standard Error Codes

- `400 INVALID_PARAMETER`: Request parameter invalid
- `401 UNAUTHORIZED`: Authentication required or failed
- `403 FORBIDDEN`: Insufficient permissions
- `404 RESOURCE_NOT_FOUND`: Resource doesn't exist
- `429 RATE_LIMIT_EXCEEDED`: Too many requests
- `500 INTERNAL_ERROR`: Server error
- `503 SERVICE_UNAVAILABLE`: Temporary outage

---

## 5. Rate Limiting

### 5.1 Rate Limit Headers

```
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 987
X-RateLimit-Reset: 1642267200
```

### 5.2 Default Limits

- **Authenticated users**: 1000 requests/hour
- **Service accounts**: 5000 requests/hour
- **Premium tier**: 10000 requests/hour

When exceeded: 429 Too Many Requests with Retry-After header

---

## 6. Pagination

### 6.1 Cursor-Based Pagination

For large datasets, use cursor-based pagination:

```json
{
  "data": [...],
  "pagination": {
    "next_cursor": "eyJsYXN0X2lkIjo...",
    "has_more": true
  },
  "links": {
    "next": "/api/v1/endpoint?cursor=eyJsYXN0X2lkIjo..."
  }
}
```

---

## 7. Versioning

### 7.1 URL-Based Versioning

API version in URL path: `/api/v1/`, `/api/v2/`

### 7.2 Version Support Policy

- Major versions supported minimum 2 years after successor release
- Deprecated endpoints return warning headers 6 months before removal
- Breaking changes only in new major versions

---

## 8. WebSocket Protocol

### 8.1 Real-Time Streaming

Connect to: `wss://api.example.com/api/v1/buildings/{building_id}/realtime`

Subscribe to data streams:
```json
{
  "action": "subscribe",
  "streams": [{
    "type": "power_demand",
    "meter_id": "METER-E-301",
    "sample_rate": "1s"
  }]
}
```

Receive updates:
```json
{
  "stream_id": "power_demand_METER-E-301",
  "timestamp": "2025-01-15T14:30:00.000Z",
  "data": {
    "power_kw": 150.25
  }
}
```

---

## 9. Implementation Requirements

### 9.1 TLS/SSL

All API endpoints must use TLS 1.3 or newer. HTTP connections are not permitted.

### 9.2 Content Negotiation

APIs must support:
- Content-Type: application/json
- Accept: application/json

### 9.3 CORS

APIs serving browser clients must implement appropriate CORS headers.

---

## 10. Conformance Testing

### 10.1 Test Suite

WIA provides comprehensive test suite covering:
- Authentication flows
- All endpoint operations
- Error handling
- Rate limiting
- Pagination
- WebSocket connectivity

### 10.2 Certification Requirements

Phase 2 certification requires:
- 100% test suite pass rate
- Performance benchmarks met (p95 latency < 200ms)
- Security audit passed
- Interoperability with reference implementation

---

**© 2025 SmileStory Inc. / WIA**
**弘益人間 - Benefit All Humanity**


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
