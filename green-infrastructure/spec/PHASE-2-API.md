# WIA Green Infrastructure API Standard
## Phase 2 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #EF4444 (Red - ENE Category)

---

## Table of Contents

1. [Overview](#overview)
2. [API Endpoints](#api-endpoints)
3. [Authentication](#authentication)
4. [Request/Response Format](#requestresponse-format)
5. [Error Handling](#error-handling)
6. [Rate Limiting](#rate-limiting)
7. [Examples](#examples)

---

## Overview

### 1.1 Purpose

The WIA Green Infrastructure API Standard defines REST API endpoints for registering, monitoring, and managing green infrastructure installations across urban environments.

### 1.2 Base URL

```
Production: https://api.wia.live/green-infrastructure/v1
Staging: https://staging-api.wia.live/green-infrastructure/v1
```

### 1.3 Protocol

- **Protocol**: HTTPS only
- **Format**: JSON
- **Encoding**: UTF-8
- **Authentication**: Bearer token (JWT)

---

## API Endpoints

### 2.1 Infrastructure Management

#### Register New Infrastructure

```http
POST /api/v1/infrastructure/register
```

**Request Body:**
```json
{
  "type": "green_roof",
  "subtype": "extensive",
  "location": {
    "gps": {"latitude": 37.5665, "longitude": 126.9780},
    "address": "123 Green Street, Seoul"
  },
  "dimensions": {
    "area": {"value": 500, "unit": "m2"}
  },
  "vegetation": {
    "coverage": 85,
    "types": ["sedum", "grasses"]
  }
}
```

**Response (201 Created):**
```json
{
  "status": "success",
  "data": {
    "infrastructureId": "GI-2025-000001",
    "certification": "WIA-GREEN-ABC12345",
    "created": "2025-01-15T10:30:00Z"
  }
}
```

#### Get Infrastructure Details

```http
GET /api/v1/infrastructure/{id}
```

**Response (200 OK):**
```json
{
  "status": "success",
  "data": {
    "infrastructureId": "GI-2025-000001",
    "type": "green_roof",
    "status": "active",
    "location": {
      "gps": {"latitude": 37.5665, "longitude": 126.9780}
    },
    "performance": {
      "stormwaterRetention": 450,
      "carbonSequestration": 510,
      "coolingEffect": 3.5
    }
  }
}
```

#### Update Infrastructure

```http
PUT /api/v1/infrastructure/{id}
```

#### Delete Infrastructure

```http
DELETE /api/v1/infrastructure/{id}
```

### 2.2 Monitoring Endpoints

#### Submit Sensor Data

```http
POST /api/v1/infrastructure/{id}/monitor
```

**Request Body:**
```json
{
  "sensorId": "SENSOR-GI-001",
  "type": "soil_moisture",
  "data": {
    "moisture": 65,
    "temperature": 22.5
  },
  "timestamp": "2025-01-15T10:30:00Z"
}
```

**Response (200 OK):**
```json
{
  "status": "success",
  "monitorId": "MON-2025-001",
  "received": "2025-01-15T10:30:05Z"
}
```

#### Get Latest Readings

```http
GET /api/v1/infrastructure/{id}/readings
```

**Query Parameters:**
- `sensor_type` (optional): Filter by sensor type
- `start_date` (optional): Start date for range
- `end_date` (optional): End date for range
- `limit` (optional): Max results (default: 100)

#### Get Alerts

```http
GET /api/v1/infrastructure/{id}/alerts
```

### 2.3 Performance Analytics

#### Calculate Impact

```http
GET /api/v1/infrastructure/{id}/impact
```

**Query Parameters:**
- `period`: "daily", "monthly", "annual"
- `metrics`: Comma-separated list (e.g., "stormwater,carbon,temperature")

**Response (200 OK):**
```json
{
  "status": "success",
  "period": "annual",
  "impact": {
    "stormwaterRetained": {
      "value": 450.5,
      "unit": "m3_per_year"
    },
    "carbonSequestered": {
      "value": 612.3,
      "unit": "kg_co2_per_year"
    },
    "coolingEffect": {
      "value": 3.2,
      "unit": "celsius"
    },
    "costSavings": {
      "value": 1250.75,
      "unit": "USD"
    }
  }
}
```

#### Generate Report

```http
POST /api/v1/infrastructure/{id}/report
```

**Request Body:**
```json
{
  "reportType": "performance",
  "period": {
    "start": "2024-01-01T00:00:00Z",
    "end": "2024-12-31T23:59:59Z"
  },
  "format": "pdf"
}
```

### 2.4 Maintenance Endpoints

#### Schedule Maintenance

```http
POST /api/v1/infrastructure/{id}/maintenance
```

#### Get Maintenance History

```http
GET /api/v1/infrastructure/{id}/maintenance/history
```

---

## Authentication

### 3.1 JWT Token

All API requests require authentication using JWT Bearer token:

```http
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

### 3.2 Obtaining Token

```http
POST /api/v1/auth/token
```

**Request Body:**
```json
{
  "apiKey": "your-api-key",
  "apiSecret": "your-api-secret"
}
```

**Response:**
```json
{
  "token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "expiresIn": 3600
}
```

---

## Request/Response Format

### 4.1 Standard Response Structure

**Success Response:**
```json
{
  "status": "success",
  "data": {},
  "metadata": {
    "timestamp": "2025-01-15T10:30:00Z",
    "version": "1.0.0"
  }
}
```

**Error Response:**
```json
{
  "status": "error",
  "error": {
    "code": "ERR_INVALID_DATA",
    "message": "Invalid infrastructure data",
    "details": {}
  },
  "metadata": {
    "timestamp": "2025-01-15T10:30:00Z"
  }
}
```

### 4.2 Pagination

For list endpoints:

```http
GET /api/v1/infrastructure?page=1&limit=20
```

**Response:**
```json
{
  "status": "success",
  "data": [],
  "pagination": {
    "page": 1,
    "limit": 20,
    "total": 150,
    "totalPages": 8
  }
}
```

---

## Error Handling

### 5.1 HTTP Status Codes

| Code | Status | Description |
|------|--------|-------------|
| 200 | OK | Request successful |
| 201 | Created | Resource created |
| 400 | Bad Request | Invalid request data |
| 401 | Unauthorized | Missing or invalid token |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource not found |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server error |

### 5.2 Error Codes

| Code | Description |
|------|-------------|
| `ERR_INVALID_DATA` | Invalid request data |
| `ERR_INVALID_LOCATION` | Invalid GPS coordinates |
| `ERR_INFRASTRUCTURE_NOT_FOUND` | Infrastructure not found |
| `ERR_SENSOR_OFFLINE` | Sensor not responding |
| `ERR_RATE_LIMIT` | Rate limit exceeded |

---

## Rate Limiting

### 6.1 Limits

| Tier | Requests/minute | Requests/hour |
|------|-----------------|---------------|
| Free | 60 | 1,000 |
| Standard | 300 | 10,000 |
| Premium | 1,000 | 50,000 |

### 6.2 Headers

```http
X-RateLimit-Limit: 60
X-RateLimit-Remaining: 45
X-RateLimit-Reset: 1642342800
```

---

## Examples

### 7.1 Register Green Roof

```bash
curl -X POST https://api.wia.live/green-infrastructure/v1/infrastructure/register \
  -H "Authorization: Bearer YOUR_TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "type": "green_roof",
    "location": {
      "gps": {"latitude": 37.5665, "longitude": 126.9780}
    },
    "dimensions": {
      "area": {"value": 500, "unit": "m2"}
    },
    "vegetation": {
      "coverage": 85
    }
  }'
```

### 7.2 Submit Sensor Reading

```bash
curl -X POST https://api.wia.live/green-infrastructure/v1/infrastructure/GI-2025-001/monitor \
  -H "Authorization: Bearer YOUR_TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "sensorId": "SENSOR-001",
    "type": "soil_moisture",
    "data": {
      "moisture": 65,
      "temperature": 22.5
    }
  }'
```

### 7.3 Get Performance Impact

```bash
curl -X GET "https://api.wia.live/green-infrastructure/v1/infrastructure/GI-2025-001/impact?period=annual" \
  -H "Authorization: Bearer YOUR_TOKEN"
```

---

<div align="center">

**WIA Green Infrastructure API v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA**

**MIT License**

</div>


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

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-2-API. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-2-api/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-2-API with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-2-API does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-2-API.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-2-API. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P2-API-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.
