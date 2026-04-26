# WIA Ecosystem Monitoring Standard - Phase 2: API Interface Specification v1.0

**Status:** Official Release  
**Version:** 1.0.0  
**Date:** December 26, 2025  
**License:** CC BY 4.0

## 1. Introduction

Phase 2 specifies RESTful and real-time APIs for accessing ecosystem monitoring data programmatically.

## 2. RESTful API Specification

### 2.1 Base URL Structure
```
https://api.{domain}/v1/
```

### 2.2 Core Endpoints

**GET /observations**
Query species observations with filtering parameters.

Request parameters:
- `taxon` (string): Scientific name
- `start_date` (ISO 8601): Filter by date range
- `end_date` (ISO 8601): Filter by date range
- `bbox` (string): Bounding box "minLon,minLat,maxLon,maxLat"
- `limit` (integer): Maximum records returned (default: 100, max: 1000)
- `offset` (integer): Pagination offset
- `format` (string): Response format - "json", "csv", "geojson"

Response format:
```json
{
  "status": "success",
  "api_version": "1.0",
  "request_id": "string",
  "timestamp": "ISO 8601 datetime",
  "query": { /* echoed query parameters */ },
  "pagination": {
    "total_records": 1000,
    "returned_records": 100,
    "page": 1,
    "total_pages": 10,
    "next_page": "URL"
  },
  "data": [ /* array of observation objects */ ]
}
```

**POST /observations**
Submit new observation(s).

Request body: Single observation object or array
Response: Created observation ID(s) with validation results

**GET /observations/{id}**
Retrieve specific observation by ID.

**GET /sensors**
List available sensors.

**GET /sensors/{id}/data**
Retrieve sensor time series data.

Parameters:
- `start_time` (ISO 8601): Start of time range
- `end_time` (ISO 8601): End of time range
- `aggregation` (enum): "raw", "hourly", "daily", "monthly"
- `format` (string): "json", "csv", "netcdf"

**GET /sites**
Query monitoring sites.

**GET /datasets**
Discover available datasets.

### 2.3 Authentication

Supported methods:
1. **API Key**: `Authorization: Bearer YOUR_API_KEY`
2. **OAuth 2.0**: Standard OAuth 2.0 flow
3. **JWT**: JSON Web Tokens with claims

### 2.4 Rate Limiting

Headers returned:
```
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 847
X-RateLimit-Reset: 1735228800
```

Default limits:
- Anonymous: 100 requests/hour
- Authenticated: 1000 requests/hour
- Premium: 10000 requests/hour

### 2.5 Error Responses

```json
{
  "status": "error",
  "error_code": "INVALID_PARAMETER",
  "message": "Human-readable error description",
  "details": { /* additional error context */ },
  "request_id": "string",
  "timestamp": "ISO 8601 datetime"
}
```

HTTP Status Codes:
- 200: Success
- 201: Created
- 400: Bad Request
- 401: Unauthorized
- 403: Forbidden
- 404: Not Found
- 429: Too Many Requests
- 500: Internal Server Error

## 3. Real-Time Streaming API

### 3.1 WebSocket Protocol

Connection:
```javascript
ws://api.{domain}/stream
wss://api.{domain}/stream  // SSL/TLS
```

Subscribe message:
```json
{
  "action": "subscribe",
  "sensors": ["SENSOR-001", "SENSOR-002"],
  "filters": {
    "quality_min": 0.8
  }
}
```

Data messages:
```json
{
  "type": "sensor_reading",
  "sensor_id": "SENSOR-001",
  "timestamp": "ISO 8601",
  "value": 12.3,
  "unit": "celsius",
  "qc_flag": "good"
}
```

### 3.2 MQTT Protocol

Topics:
```
sensors/{sensor_id}/data
sensors/{sensor_id}/status
sensors/{sensor_id}/alerts
observations/{site_id}/species
```

QoS Levels:
- 0: At most once
- 1: At least once (recommended for data)
- 2: Exactly once (for critical alerts)

## 4. Bulk Data Access

### 4.1 Asynchronous Queries

**POST /bulk-query**
Submit large query for asynchronous processing.

**GET /jobs/{job_id}**
Check job status.

**GET /jobs/{job_id}/download**
Download completed results.

### 4.2 Data Dumps

Pre-generated datasets available at `/dumps/`:
- Current year observations: Updated daily
- Sensor data: Updated hourly
- Complete archive: Updated monthly

## 5. OpenAPI Specification

Complete OpenAPI 3.0 specification available at:
```
https://api.{domain}/openapi.json
```

## 6. SDK Support

Official client libraries:
- JavaScript/TypeScript: npm package `@wia/ecosystem-monitoring`
- Python: pip package `wia-ecosystem-monitoring`
- R: CRAN package `wiaR`
- Java: Maven artifact `org.wia:ecosystem-monitoring`

## 7. Versioning

API versions in URL path: `/v1/`, `/v2/`
Minimum 12-month support for deprecated versions.


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

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-2-API-INTERFACE.
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
