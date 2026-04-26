# WIA-ENE-062: Glacier Preservation
## Phase 2 - API Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-25

---

## Overview

This document defines the RESTful API for accessing glacier monitoring data, submitting measurements, and retrieving preservation insights within the WIA-ENE-062 standard.

## Base URL

```
https://api.wia.org/glacier-preservation/v1
```

## Authentication

All API requests require authentication using OAuth 2.0 Bearer tokens:

```http
Authorization: Bearer {access_token}
```

### Obtaining Access Tokens

```http
POST /auth/token
Content-Type: application/json

{
  "client_id": "your_client_id",
  "client_secret": "your_client_secret",
  "grant_type": "client_credentials"
}
```

**Response:**

```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600
}
```

## Core Endpoints

### 1. Glacier Registry

#### List All Glaciers

```http
GET /glaciers
```

**Query Parameters:**

| Parameter | Type | Description | Default |
|-----------|------|-------------|---------|
| region | string | Filter by region (e.g., "Himalaya") | - |
| country | string | Filter by country code (ISO 3166-1) | - |
| size | string | Filter by size (small/medium/large) | - |
| status | string | Filter by monitoring status | active |
| limit | integer | Results per page (max 100) | 20 |
| offset | integer | Pagination offset | 0 |

**Example Request:**

```http
GET /glaciers?region=Himalaya&limit=50&offset=0
Authorization: Bearer {token}
```

**Response (200 OK):**

```json
{
  "data": [
    {
      "glacierId": "GLR-001-HIM",
      "name": "Gangotri Glacier",
      "location": {
        "region": "Himalayan Range",
        "country": "IN",
        "coordinates": {
          "latitude": 30.9229,
          "longitude": 79.0831,
          "elevation": 4200
        }
      },
      "status": "active",
      "lastMeasurement": "2025-12-20T14:30:00Z"
    }
  ],
  "pagination": {
    "total": 152,
    "limit": 50,
    "offset": 0,
    "hasMore": true
  }
}
```

#### Get Specific Glacier

```http
GET /glaciers/{glacierId}
```

**Path Parameters:**

- `glacierId`: Unique glacier identifier (e.g., GLR-001-HIM)

**Response (200 OK):**

```json
{
  "glacierId": "GLR-001-HIM",
  "name": "Gangotri Glacier",
  "location": {
    "region": "Himalayan Range",
    "country": "IN",
    "coordinates": {
      "latitude": 30.9229,
      "longitude": 79.0831,
      "elevation": 4200
    },
    "mountainRange": "Garhwal Himalaya"
  },
  "classification": {
    "type": "valley",
    "size": "large",
    "regime": "temperate"
  },
  "currentStatus": {
    "mass": 450.5,
    "area": 143.7,
    "meltRate": 2.3,
    "lastUpdated": "2025-12-20T14:30:00Z"
  }
}
```

#### Register New Glacier

```http
POST /glaciers
Content-Type: application/json
```

**Request Body:**

```json
{
  "name": "Example Glacier",
  "location": {
    "region": "Alps",
    "country": "CH",
    "coordinates": {
      "latitude": 46.5,
      "longitude": 8.0,
      "elevation": 3500
    }
  },
  "classification": {
    "type": "valley",
    "size": "medium",
    "regime": "temperate"
  }
}
```

**Response (201 Created):**

```json
{
  "glacierId": "GLR-234-ALP",
  "message": "Glacier registered successfully",
  "createdAt": "2025-12-25T10:00:00Z"
}
```

### 2. Mass Balance Measurements

#### Submit Measurement

```http
POST /glaciers/{glacierId}/measurements
Content-Type: application/json
```

**Request Body:**

```json
{
  "timestamp": "2025-12-20T14:30:00Z",
  "massBalance": {
    "totalMass": {
      "value": 450.5,
      "unit": "Gt",
      "uncertainty": 5.2
    },
    "surfaceArea": {
      "value": 143.7,
      "unit": "km²",
      "uncertainty": 2.1
    },
    "volume": {
      "value": 28.5,
      "unit": "km³",
      "uncertainty": 1.5
    }
  },
  "methodology": {
    "technique": "satellite_altimetry",
    "instrument": "ICESat-2",
    "resolution": "30m"
  }
}
```

**Response (201 Created):**

```json
{
  "measurementId": "MSR-20251220-001",
  "glacierId": "GLR-001-HIM",
  "status": "accepted",
  "validationScore": 0.98,
  "createdAt": "2025-12-25T10:05:00Z"
}
```

#### Get Measurement History

```http
GET /glaciers/{glacierId}/measurements
```

**Query Parameters:**

| Parameter | Type | Description | Default |
|-----------|------|-------------|---------|
| start | ISO8601 | Start date | 30 days ago |
| end | ISO8601 | End date | now |
| parameter | string | Filter by parameter (mass/area/volume) | all |
| limit | integer | Results per page | 50 |

**Response (200 OK):**

```json
{
  "glacierId": "GLR-001-HIM",
  "measurements": [
    {
      "measurementId": "MSR-20251220-001",
      "timestamp": "2025-12-20T14:30:00Z",
      "massBalance": {
        "totalMass": {
          "value": 450.5,
          "unit": "Gt"
        }
      }
    }
  ],
  "summary": {
    "count": 45,
    "period": "2025-11-20T00:00:00Z / 2025-12-20T23:59:59Z"
  }
}
```

### 3. Melt Rate Analysis

#### Get Melt Rate Trends

```http
GET /glaciers/{glacierId}/melt-rate
```

**Query Parameters:**

| Parameter | Type | Description | Default |
|-----------|------|-------------|---------|
| period | string | Time period (1y/5y/10y/all) | 5y |
| resolution | string | Data resolution (daily/monthly/yearly) | monthly |

**Response (200 OK):**

```json
{
  "glacierId": "GLR-001-HIM",
  "period": {
    "start": "2020-12-25T00:00:00Z",
    "end": "2025-12-25T00:00:00Z",
    "duration": "5 years"
  },
  "meltRate": {
    "current": {
      "value": 2.3,
      "unit": "Gt/year"
    },
    "trend": {
      "acceleration": 0.08,
      "direction": "increasing",
      "confidence": 0.94
    },
    "seasonal": {
      "summer": 4.2,
      "winter": 0.4,
      "unit": "Gt/season"
    }
  },
  "contributingFactors": {
    "temperatureAnomaly": 1.8,
    "albedoChange": -0.05,
    "precipitationChange": -12.5
  }
}
```

#### Calculate Future Projections

```http
POST /glaciers/{glacierId}/projections
Content-Type: application/json
```

**Request Body:**

```json
{
  "timeHorizon": 50,
  "scenario": "SSP2-4.5",
  "parameters": {
    "temperatureIncrease": 2.0,
    "precipitationChange": -10,
    "sensitivity": 0.15
  }
}
```

**Response (200 OK):**

```json
{
  "glacierId": "GLR-001-HIM",
  "scenario": "SSP2-4.5",
  "projections": {
    "year2050": {
      "mass": 390.2,
      "meltRate": 3.5,
      "percentLoss": 13.4
    },
    "year2075": {
      "mass": 325.8,
      "meltRate": 4.8,
      "percentLoss": 27.7
    },
    "year2100": {
      "mass": 245.3,
      "meltRate": 6.2,
      "percentLoss": 45.5
    }
  },
  "confidence": 0.82
}
```

### 4. Environmental Monitoring

#### Submit Environmental Data

```http
POST /glaciers/{glacierId}/environment
Content-Type: application/json
```

**Request Body:**

```json
{
  "timestamp": "2025-12-20T14:30:00Z",
  "temperature": {
    "surface": {
      "value": -5.2,
      "unit": "°C"
    },
    "anomaly": 1.8
  },
  "albedo": {
    "value": 0.65,
    "change": -0.02
  },
  "precipitation": {
    "annual": 2500,
    "snowfall": 2100,
    "rainfall": 400,
    "unit": "mm"
  }
}
```

**Response (201 Created):**

```json
{
  "conditionId": "ENV-20251220-001",
  "glacierId": "GLR-001-HIM",
  "status": "recorded",
  "createdAt": "2025-12-25T10:10:00Z"
}
```

### 5. Sea Level Impact

#### Get Sea Level Contribution

```http
GET /glaciers/{glacierId}/sea-level-impact
```

**Response (200 OK):**

```json
{
  "glacierId": "GLR-001-HIM",
  "historical": {
    "contribution": 0.045,
    "period": "2000-2025",
    "unit": "mm"
  },
  "projected": {
    "year2050": 0.082,
    "year2100": 0.156,
    "unit": "mm"
  },
  "globalContext": {
    "percentOfTotal": 0.0012,
    "ranking": 47
  }
}
```

#### Get Regional Summary

```http
GET /regions/{region}/sea-level-impact
```

**Response (200 OK):**

```json
{
  "region": "Himalaya",
  "glacierCount": 152,
  "totalMass": 38420.5,
  "seaLevelContribution": {
    "historical": 2.34,
    "projected2050": 4.21,
    "projected2100": 8.67,
    "unit": "mm"
  }
}
```

### 6. Water Resource Assessment

#### Get Water Supply Impact

```http
GET /glaciers/{glacierId}/water-supply
```

**Response (200 OK):**

```json
{
  "glacierId": "GLR-001-HIM",
  "waterSupply": {
    "current": {
      "annualFlow": 9.2,
      "seasonalPeak": "June-August",
      "unit": "km³/year"
    },
    "projected": {
      "year2050": 7.1,
      "year2100": 4.3,
      "changePercent": -53.3
    }
  },
  "downstreamDependency": {
    "population": 125000,
    "agricultureArea": 45000,
    "hydropower": 250,
    "criticalityLevel": "high"
  }
}
```

### 7. Preservation Actions

#### Submit Preservation Action

```http
POST /glaciers/{glacierId}/preservation-actions
Content-Type: application/json
```

**Request Body:**

```json
{
  "actionType": "reflective_materials",
  "implementation": {
    "startDate": "2025-06-01T00:00:00Z",
    "coverage": 15000,
    "investment": 500000
  },
  "methodology": {
    "technique": "geotextile_blankets",
    "materials": ["UV-resistant_fabric", "reflective_coating"],
    "expectedReduction": 30
  }
}
```

**Response (201 Created):**

```json
{
  "actionId": "ACT-20250601-001",
  "glacierId": "GLR-001-HIM",
  "status": "planned",
  "createdAt": "2025-12-25T10:15:00Z"
}
```

#### Track Action Effectiveness

```http
GET /glaciers/{glacierId}/preservation-actions/{actionId}/effectiveness
```

**Response (200 OK):**

```json
{
  "actionId": "ACT-20250601-001",
  "implementation": {
    "startDate": "2025-06-01T00:00:00Z",
    "status": "active",
    "coverage": 15000,
    "unit": "m²"
  },
  "effectiveness": {
    "meltReduction": 28.5,
    "albedoIncrease": 0.12,
    "costPerTon": 217,
    "sustainabilityScore": 0.87
  },
  "monitoring": {
    "lastAssessment": "2025-12-15T00:00:00Z",
    "nextAssessment": "2026-03-15T00:00:00Z"
  }
}
```

## Webhook Notifications

Subscribe to real-time updates for glacier events:

```http
POST /webhooks
Content-Type: application/json
```

**Request Body:**

```json
{
  "url": "https://your-server.com/webhook",
  "events": ["measurement_submitted", "critical_melt_rate", "preservation_milestone"],
  "glacierIds": ["GLR-001-HIM", "GLR-002-ALP"]
}
```

**Webhook Payload Example:**

```json
{
  "event": "critical_melt_rate",
  "glacierId": "GLR-001-HIM",
  "timestamp": "2025-12-25T10:20:00Z",
  "data": {
    "meltRate": 5.2,
    "threshold": 5.0,
    "severity": "high"
  }
}
```

## Error Responses

### Standard Error Format

```json
{
  "error": {
    "code": "INVALID_PARAMETER",
    "message": "Mass value cannot be negative",
    "details": {
      "field": "massBalance.totalMass.value",
      "value": -10.5,
      "constraint": "value >= 0"
    }
  }
}
```

### HTTP Status Codes

| Code | Description |
|------|-------------|
| 200 | Success |
| 201 | Created |
| 400 | Bad Request (invalid parameters) |
| 401 | Unauthorized (missing/invalid token) |
| 403 | Forbidden (insufficient permissions) |
| 404 | Not Found (glacier/resource doesn't exist) |
| 429 | Too Many Requests (rate limit exceeded) |
| 500 | Internal Server Error |

## Rate Limiting

- **Free tier**: 100 requests/hour
- **Standard tier**: 1,000 requests/hour
- **Enterprise tier**: 10,000 requests/hour

Rate limit headers:

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 847
X-RateLimit-Reset: 1640444400
```

## Versioning

API versions are specified in the URL path. Breaking changes increment the major version:

- v1: Current stable version
- v2: Planned for Q2 2026 (backward-incompatible changes)

## SDKs and Libraries

Official SDKs available:

- **JavaScript/TypeScript**: `npm install @wia/glacier-preservation`
- **Python**: `pip install wia-glacier-preservation`
- **Go**: `go get github.com/wia/glacier-preservation-go`
- **Java**: Maven/Gradle package available

---

**弘益人間 (홍익인간) - Benefit All Humanity**

© 2025 SmileStory Inc. / WIA


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
