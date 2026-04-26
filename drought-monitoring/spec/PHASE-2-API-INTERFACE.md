# WIA Drought Monitoring Standard - Phase 2: API Interface Specification v1.0

## Overview

Phase 2 defines RESTful API interfaces for accessing WIA-compliant drought data programmatically. This enables automated integration, real-time monitoring, and alert subscription services.

**Version:** 1.0.0
**Status:** Published
**Last Updated:** 2025-12-26

## Base URL Structure

All WIA-compliant APIs MUST follow this URL pattern:

```
https://{domain}/wia/drought/v1/{resource}
```

**Components:**
- `{domain}`: Service provider domain
- `/wia/drought`: WIA drought standard namespace (REQUIRED)
- `/v1`: API version
- `{resource}`: Specific endpoint (pdsi, spi, ndvi, etc.)

## Authentication

### Supported Methods

1. **API Key (Required for all implementations)**
```http
GET /wia/drought/v1/pdsi?lat=40.7128&lon=-74.0060
Headers:
  X-WIA-API-Key: your-api-key-here
```

2. **OAuth 2.0 (Optional)**
```http
GET /wia/drought/v1/pdsi?lat=40.7128&lon=-74.0060
Headers:
  Authorization: Bearer {access_token}
```

### Access Tiers

| Tier | Authentication | Rate Limit | Features |
|------|---------------|------------|----------|
| Public | None | 100 req/day | Basic current data |
| Registered | API Key | 10,000 req/day | Historical data, alerts |
| Premium | OAuth 2.0 | 100,000 req/day | All features |
| Research/Gov | OAuth 2.0 | Unlimited | Bulk export, streaming |

## Core Endpoints

### Get Current PDSI

```http
GET /wia/drought/v1/pdsi

Query Parameters:
  lat (required): Latitude (-90 to 90)
  lon (required): Longitude (-180 to 180)
  date (optional): ISO 8601 date

Response: 200 OK
{
  "wia_version": "1.0",
  "data_type": "pdsi",
  "timestamp": "2025-12-26T00:00:00Z",
  "location": {...},
  "pdsi": {...},
  "metadata": {...}
}
```

### Get Current SPI

```http
GET /wia/drought/v1/spi

Query Parameters:
  lat (required): Latitude
  lon (required): Longitude
  scale (required): Time scale (1, 3, 6, 12, 24 months)
  date (optional): ISO 8601 date
```

### Get Soil Moisture

```http
GET /wia/drought/v1/soil-moisture

Query Parameters:
  lat (required): Latitude
  lon (required): Longitude
  depth (optional): Depth in cm or "all"
  date (optional): ISO 8601 date
```

### Get NDVI

```http
GET /wia/drought/v1/ndvi

Query Parameters:
  bbox (required): Bounding box [west,south,east,north]
  date (optional): ISO 8601 date
  resolution (optional): Resolution in meters (250, 500, 1000)
```

## Historical Data

### Time Series Query

```http
GET /wia/drought/v1/{index}/timeseries

Query Parameters:
  lat (required): Latitude
  lon (required): Longitude
  start_date (required): ISO 8601 date
  end_date (required): ISO 8601 date
  interval (optional): daily, weekly, monthly (default: monthly)

Response:
{
  "wia_version": "1.0",
  "data_type": "pdsi_timeseries",
  "location": {...},
  "time_series": [
    {
      "timestamp": "2024-01-01T00:00:00Z",
      "pdsi": {"value": 1.2, "classification": "slightly_wet"}
    },
    ...
  ],
  "statistics": {
    "count": 24,
    "mean": -0.45,
    "trend": "declining"
  }
}
```

## Alert Subscriptions

### Create Subscription

```http
POST /wia/drought/v1/alerts/subscribe

Request Body:
{
  "locations": [
    {"lat": 40.7128, "lon": -74.0060, "name": "Field A"}
  ],
  "indices": ["pdsi", "soil_moisture"],
  "thresholds": {
    "pdsi": {"warning": -2.0, "alert": -3.0}
  },
  "notification_methods": [
    {"type": "webhook", "url": "https://app.com/alert"},
    {"type": "email", "address": "user@example.com"}
  ],
  "frequency": "daily"
}

Response: 201 Created
{
  "subscription_id": "sub_a1b2c3",
  "status": "active",
  "created_at": "2025-12-26T14:30:00Z"
}
```

### Webhook Alert Format

```http
POST {webhook_url}

Request Body:
{
  "alert_id": "alert_x1y2z3",
  "subscription_id": "sub_a1b2c3",
  "timestamp": "2025-12-27T08:15:00Z",
  "location": {"name": "Field A", "lat": 40.7128, "lon": -74.0060},
  "trigger": {
    "index": "pdsi",
    "value": -3.2,
    "threshold": "alert"
  },
  "severity": "high",
  "recommended_actions": [...]
}
```

## Error Handling

### Standard Error Format

```json
{
  "error": {
    "code": "ERROR_CODE",
    "message": "Human-readable description",
    "details": {...},
    "request_id": "req_xyz789",
    "documentation_url": "https://docs.wia.org/errors#ERROR_CODE"
  }
}
```

### HTTP Status Codes

| Status | Error Code | Meaning | Action |
|--------|-----------|---------|--------|
| 400 | INVALID_PARAMETERS | Bad request parameters | Fix parameters |
| 401 | UNAUTHORIZED | Missing/invalid auth | Provide valid credentials |
| 404 | DATA_NOT_FOUND | No data available | Try different location/date |
| 429 | RATE_LIMIT_EXCEEDED | Too many requests | Wait before retry |
| 500 | INTERNAL_ERROR | Server error | Contact support |

## Rate Limiting

All responses MUST include rate limit headers:

```http
X-RateLimit-Limit: 10000
X-RateLimit-Remaining: 9847
X-RateLimit-Reset: 1735257600
X-RateLimit-Window: daily
```

## Implementation Requirements

Phase 2 compliant systems MUST:

1. Implement base URL structure `/wia/drought/v1/`
2. Support API key authentication
3. Provide at least PDSI and SPI endpoints
4. Return standardized error responses
5. Include rate limit headers
6. Support CORS for web applications
7. Provide OpenAPI/Swagger documentation

## Conformance Checklist

- [ ] Base URL follows `/wia/drought/v1/` pattern
- [ ] API key authentication working
- [ ] Core endpoints implemented (PDSI, SPI)
- [ ] Error responses match standard format
- [ ] Rate limiting implemented
- [ ] CORS enabled
- [ ] OpenAPI documentation available
- [ ] Load test: 100 requests/second sustained

## References

- REST API Design Best Practices
- OpenAPI Specification 3.0
- OAuth 2.0 RFC 6749

---

© 2025 SmileStory Inc. / WIA
弘益人間


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
