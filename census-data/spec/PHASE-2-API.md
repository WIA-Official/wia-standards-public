# WIA-SOC-016 PHASE 2: API Specification

## Census Data Standard - REST API Design

**Version:** 1.0
**Status:** PUBLISHED
**Last Updated:** 2025-12-26

---

## 1. Overview

This document specifies the RESTful API for census data access under WIA-SOC-016 standard.

### 1.1 Base URL

```
https://api.census.wia.org/v1
```

### 1.2 Authentication

All API requests require authentication via API key:

```
Authorization: Bearer YOUR_API_KEY
```

---

## 2. Core Endpoints

### 2.1 Population Statistics

```
GET /population
GET /population/{geo_code}
GET /population/{geo_code}/{characteristic}
```

**Query Parameters:**
- `year` (integer): Census year
- `geo_level` (enum): nation, state, county, tract, block
- `age_group` (string): Filter by age range
- `sex` (enum): MALE, FEMALE, ALL
- `format` (enum): json, csv, xml

**Example Request:**
```bash
curl -H "Authorization: Bearer API_KEY" \
  "https://api.census.wia.org/v1/population/USA-CA?year=2025&format=json"
```

**Example Response:**
```json
{
  "data": {
    "geography": {
      "code": "USA-CA",
      "name": "California",
      "level": "state"
    },
    "population": {
      "total": 39538223,
      "male": 19639838,
      "female": 19898385,
      "median_age": 37.0
    },
    "quality": {
      "margin_of_error": 125000,
      "confidence_level": 0.95
    }
  },
  "metadata": {
    "source": "Census 2025",
    "privacy_method": "differential_privacy",
    "epsilon": 1.0
  }
}
```

### 2.2 Demographics

```
GET /demographics
GET /demographics/{geo_code}
```

**Dimensions:**
- Age distribution
- Education levels
- Marital status
- Citizenship
- Language spoken

### 2.3 Housing Data

```
GET /housing
GET /housing/{geo_code}
```

**Includes:**
- Housing units by type
- Tenure (owned/rented)
- Monthly housing costs
- Utilities and amenities

### 2.4 Economic Indicators

```
GET /economy
GET /economy/{geo_code}
```

**Includes:**
- Labor force participation
- Unemployment rate
- Occupation distribution
- Income distribution
- Commuting patterns

### 2.5 Time Series

```
GET /timeseries/{variable}
```

**Parameters:**
- `start_year`, `end_year`
- `geo_code`
- `frequency` (annual, intercensal)

---

## 3. Data Formats

### 3.1 JSON (Default)

Content-Type: `application/json`

### 3.2 CSV

Content-Type: `text/csv`

```
geo_code,geo_name,total_population,median_age
USA-CA,California,39538223,37.0
USA-TX,Texas,29145505,35.5
```

### 3.3 XML

Content-Type: `application/xml`

### 3.4 SDMX

Content-Type: `application/vnd.sdmx.data+xml`

---

## 4. Error Handling

```json
{
  "error": {
    "code": "INVALID_GEO_CODE",
    "message": "Geographic code 'XYZ' not found",
    "status": 404,
    "details": {
      "valid_codes": ["USA-CA", "USA-TX", ...]
    }
  }
}
```

**HTTP Status Codes:**
- 200: Success
- 400: Bad Request
- 401: Unauthorized
- 404: Not Found
- 429: Rate Limit Exceeded
- 500: Internal Server Error

---

## 5. Rate Limiting

**Public:** 100 requests/hour
**Registered:** 1,000 requests/hour
**Government:** 10,000 requests/hour

Headers:
```
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 987
X-RateLimit-Reset: 1640995200
```

---

## 6. Pagination

```
GET /population?page=1&per_page=100
```

**Response:**
```json
{
  "data": [...],
  "pagination": {
    "page": 1,
    "per_page": 100,
    "total": 5432,
    "total_pages": 55
  },
  "links": {
    "first": "/population?page=1",
    "prev": null,
    "next": "/population?page=2",
    "last": "/population?page=55"
  }
}
```

---

## 7. Filtering and Querying

```
GET /population?filter[age]=25-34&filter[sex]=FEMALE&filter[education]=BACHELORS
```

---

## 8. GraphQL Alternative

```
POST /graphql
```

**Query:**
```graphql
query {
  population(geoCode: "USA-CA") {
    total
    byAge {
      ageGroup
      count
    }
    bySex {
      sex
      count
    }
  }
}
```

---

## 9. WebSocket for Real-time Updates

```
wss://api.census.wia.org/ws
```

Subscribe to live census processing updates.

---

## 10. SDK Support

Official SDKs available for:
- Python
- R
- JavaScript/TypeScript
- Java
- Go

---

**Document Version:** 1.0
© 2025 SmileStory Inc. / WIA

---

## Annex A — Conformance Tier Matrix

WIA conformance for census-data is evaluated across three tiers:

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

- `wia-standards/standards/census-data/api/` — TypeScript SDK skeleton
- `wia-standards/standards/census-data/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/census-data/simulator/` — interactive browser-based simulator for the PHASE protocol

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

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.
