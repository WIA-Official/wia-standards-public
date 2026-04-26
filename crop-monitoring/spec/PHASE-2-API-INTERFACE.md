# WIA Crop Monitoring API Interface Standard
## Phase 2 Specification

---

**Version**: 1.0.0
**Status**: Complete
**Date**: 2025-01
**Standard ID**: WIA-AGRI-006
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #84CC16 (Lime - AGRI)

---

## Table of Contents

1. [Overview](#overview)
2. [API Architecture](#api-architecture)
3. [Authentication](#authentication)
4. [Core Endpoints](#core-endpoints)
5. [Request/Response Formats](#requestresponse-formats)
6. [Error Handling](#error-handling)
7. [Rate Limiting](#rate-limiting)
8. [SDK Examples](#sdk-examples)
9. [Version History](#version-history)

---

## Overview

### 1.1 Purpose

The WIA Crop Monitoring API Interface Standard defines RESTful API endpoints, WebSocket streaming protocols, and SDK interfaces for crop monitoring systems. This enables seamless integration of IoT sensors, AI analysis engines, and agricultural management platforms.

**Core Capabilities**:
- Real-time crop data submission and retrieval
- AI-powered disease detection API
- Yield prediction and forecasting
- Weather integration and alerts
- Marketplace and insurance integrations

### 1.2 API Architecture

```
┌─────────────────┐
│  Client Apps   │
│ (Farm Mgmt UI) │
└────────┬────────┘
         │
         ▼
┌─────────────────────────────┐
│   WIA Crop API Gateway      │
│   - Authentication          │
│   - Rate Limiting           │
│   - Load Balancing          │
└────────┬────────────────────┘
         │
    ┌────┴────┬─────────┬──────────┐
    ▼         ▼         ▼          ▼
┌────────┐ ┌──────┐ ┌────────┐ ┌────────┐
│ Crop   │ │ AI   │ │ Weather│ │ Market │
│ Data   │ │ Model│ │ API    │ │ API    │
└────────┘ └──────┘ └────────┘ └────────┘
```

### 1.3 Base URL

```
Production:  https://api.crop-monitoring.wiastandards.com/v1
Staging:     https://staging-api.crop-monitoring.wiastandards.com/v1
Development: https://dev-api.crop-monitoring.wiastandards.com/v1
```

---

## Authentication

### 2.1 API Key Authentication

All API requests require an API key in the request header:

```http
GET /crops/CROP-2025-001
Host: api.crop-monitoring.wiastandards.com
Authorization: Bearer wia_api_key_1234567890abcdef
```

### 2.2 OAuth 2.0 (Enterprise)

For enterprise integrations, OAuth 2.0 is supported:

```http
POST /oauth/token
Content-Type: application/json

{
  "grant_type": "client_credentials",
  "client_id": "your_client_id",
  "client_secret": "your_client_secret",
  "scope": "crop:read crop:write"
}
```

Response:
```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIs...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "crop:read crop:write"
}
```

### 2.3 Scopes

| Scope | Description |
|-------|-------------|
| `crop:read` | Read crop data |
| `crop:write` | Submit crop data |
| `ai:detect` | Use AI detection endpoints |
| `forecast:read` | Access yield predictions |
| `admin` | Full administrative access |

---

## Core Endpoints

### 3.1 Crop Data Management

#### POST /crops
Submit new crop monitoring data

**Request:**
```http
POST /crops
Authorization: Bearer {api_key}
Content-Type: application/json

{
  "cropId": "CROP-2025-001",
  "farmId": "FARM-KR-12345",
  "timestamp": "2025-06-15T10:30:00Z",
  "location": {
    "gps": {"latitude": 37.5665, "longitude": 126.9780}
  },
  "cropType": "rice",
  "growthStage": {
    "code": "BBCH-30",
    "description": "Vegetative stage"
  },
  "measurements": {
    "plantHeight": {"value": 45.5, "unit": "cm"},
    "leafAreaIndex": {"value": 3.8, "unit": "m²/m²"},
    "chlorophyllSPAD": {"value": 42.0, "unit": "SPAD"}
  }
}
```

**Response (201 Created):**
```json
{
  "status": "success",
  "data": {
    "cropId": "CROP-2025-001",
    "recordId": "REC-20250615-001",
    "timestamp": "2025-06-15T10:30:00Z",
    "validationStatus": "passed"
  }
}
```

#### GET /crops/{cropId}
Retrieve crop data by ID

**Request:**
```http
GET /crops/CROP-2025-001
Authorization: Bearer {api_key}
```

**Response (200 OK):**
```json
{
  "status": "success",
  "data": {
    "cropId": "CROP-2025-001",
    "farmId": "FARM-KR-12345",
    "latestMeasurement": "2025-06-15T10:30:00Z",
    "currentStage": "BBCH-30",
    "healthStatus": "healthy",
    "records": [
      {
        "timestamp": "2025-06-15T10:30:00Z",
        "measurements": {...}
      }
    ]
  }
}
```

#### GET /crops
List all crops for a farm

**Request:**
```http
GET /crops?farmId=FARM-KR-12345&limit=50&offset=0
Authorization: Bearer {api_key}
```

**Response (200 OK):**
```json
{
  "status": "success",
  "data": {
    "total": 150,
    "limit": 50,
    "offset": 0,
    "crops": [
      {
        "cropId": "CROP-2025-001",
        "cropType": "rice",
        "currentStage": "BBCH-30",
        "healthStatus": "healthy"
      }
    ]
  }
}
```

#### PATCH /crops/{cropId}
Update crop information

**Request:**
```http
PATCH /crops/CROP-2025-001
Authorization: Bearer {api_key}
Content-Type: application/json

{
  "growthStage": {
    "code": "BBCH-51",
    "description": "Flowering"
  }
}
```

#### DELETE /crops/{cropId}
Delete crop data (admin only)

---

### 3.2 AI Disease Detection

#### POST /ai/detect-disease
Analyze crop image for diseases

**Request:**
```http
POST /ai/detect-disease
Authorization: Bearer {api_key}
Content-Type: multipart/form-data

{
  "cropId": "CROP-2025-001",
  "image": [binary image data],
  "cropType": "tomato"
}
```

**Response (200 OK):**
```json
{
  "status": "success",
  "data": {
    "detections": [
      {
        "disease": "Late Blight (Phytophthora infestans)",
        "confidence": 0.87,
        "severity": "medium",
        "affectedArea": 15.5,
        "recommendations": [
          "Apply copper-based fungicide within 24 hours",
          "Remove severely affected leaves",
          "Improve air circulation"
        ]
      }
    ],
    "modelVersion": "WIA-CropVision-v2.1",
    "processingTime": 1.23
  }
}
```

#### POST /ai/detect-pest
Identify pest infestations

**Response (200 OK):**
```json
{
  "status": "success",
  "data": {
    "detections": [
      {
        "pest": "Fall Armyworm (Spodoptera frugiperda)",
        "confidence": 0.92,
        "severity": "high",
        "estimatedCount": 25,
        "recommendations": [
          "Apply Bacillus thuringiensis (Bt) spray",
          "Scout fields every 2-3 days",
          "Consider pheromone traps"
        ]
      }
    ]
  }
}
```

---

### 3.3 Yield Prediction

#### GET /forecast/yield/{cropId}
Get yield prediction for a crop

**Request:**
```http
GET /forecast/yield/CROP-2025-001
Authorization: Bearer {api_key}
```

**Response (200 OK):**
```json
{
  "status": "success",
  "data": {
    "cropId": "CROP-2025-001",
    "prediction": {
      "yieldEstimate": {
        "value": 5500,
        "unit": "kg/ha",
        "confidence": 0.82
      },
      "harvestDate": "2025-09-01",
      "qualityScore": 85,
      "factors": {
        "weather": "favorable",
        "soilHealth": "good",
        "diseaseRisk": "low"
      }
    },
    "modelVersion": "YieldPredictor-v3.0"
  }
}
```

#### POST /forecast/scenario
Run "what-if" scenario analysis

**Request:**
```http
POST /forecast/scenario
Authorization: Bearer {api_key}
Content-Type: application/json

{
  "cropId": "CROP-2025-001",
  "scenario": {
    "weatherOverride": {
      "rainfall": "+20%",
      "temperature": "+2°C"
    },
    "interventions": [
      "extra_fertilizer",
      "pest_control"
    ]
  }
}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "baselineYield": 5500,
    "scenarioYield": 6200,
    "difference": "+12.7%",
    "confidence": 0.75
  }
}
```

---

### 3.4 Weather Integration

#### GET /weather/forecast/{location}
Get weather forecast for field location

**Request:**
```http
GET /weather/forecast/37.5665,126.9780?days=7
Authorization: Bearer {api_key}
```

**Response (200 OK):**
```json
{
  "status": "success",
  "data": {
    "location": {"latitude": 37.5665, "longitude": 126.9780},
    "forecast": [
      {
        "date": "2025-06-16",
        "temperature": {"min": 18, "max": 28, "unit": "°C"},
        "humidity": 65,
        "precipitation": 5.2,
        "windSpeed": 12,
        "cropImpact": {
          "diseaseRisk": "medium",
          "wateringNeeded": false,
          "alerts": ["High humidity - monitor for fungal diseases"]
        }
      }
    ]
  }
}
```

#### GET /weather/alerts/{cropId}
Get weather alerts for specific crop

---

### 3.5 Marketplace Integration

#### GET /marketplace/prices
Get current market prices

**Request:**
```http
GET /marketplace/prices?cropType=rice&region=KR
Authorization: Bearer {api_key}
```

**Response (200 OK):**
```json
{
  "status": "success",
  "data": {
    "cropType": "rice",
    "region": "KR",
    "prices": [
      {
        "grade": "premium",
        "pricePerKg": 2.50,
        "currency": "USD",
        "buyers": 8,
        "trend": "stable"
      }
    ]
  }
}
```

---

## Request/Response Formats

### 4.1 Common Headers

**Request Headers:**
```http
Authorization: Bearer {api_key}
Content-Type: application/json
Accept: application/json
X-Request-ID: uuid-v4
```

**Response Headers:**
```http
Content-Type: application/json
X-Request-ID: uuid-v4
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1625097600
```

### 4.2 Standard Response Format

```json
{
  "status": "success" | "error",
  "data": {...},
  "error": {
    "code": "string",
    "message": "string",
    "details": {}
  },
  "metadata": {
    "requestId": "uuid",
    "timestamp": "ISO8601",
    "version": "v1"
  }
}
```

---

## Error Handling

### 5.1 HTTP Status Codes

| Code | Meaning | Description |
|------|---------|-------------|
| 200 | OK | Request successful |
| 201 | Created | Resource created |
| 400 | Bad Request | Invalid input data |
| 401 | Unauthorized | Missing or invalid API key |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource not found |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server error |

### 5.2 Error Response Format

```json
{
  "status": "error",
  "error": {
    "code": "INVALID_CROP_DATA",
    "message": "Invalid chlorophyll SPAD value",
    "details": {
      "field": "measurements.chlorophyllSPAD.value",
      "value": 75,
      "constraint": "Must be between 0 and 60"
    }
  },
  "metadata": {
    "requestId": "req-123456",
    "timestamp": "2025-06-15T10:30:00Z"
  }
}
```

### 5.3 Error Codes

| Code | Description |
|------|-------------|
| `INVALID_API_KEY` | API key is missing or invalid |
| `INVALID_CROP_DATA` | Crop data validation failed |
| `CROP_NOT_FOUND` | Crop ID not found |
| `AI_MODEL_UNAVAILABLE` | AI detection service unavailable |
| `RATE_LIMIT_EXCEEDED` | Too many requests |

---

## Rate Limiting

### 6.1 Rate Limits

| Tier | Requests/Hour | Requests/Day |
|------|---------------|--------------|
| Free | 100 | 1,000 |
| Basic | 1,000 | 10,000 |
| Pro | 10,000 | 100,000 |
| Enterprise | Unlimited | Unlimited |

### 6.2 Rate Limit Headers

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 950
X-RateLimit-Reset: 1625097600
```

---

## SDK Examples

### 7.1 JavaScript/TypeScript SDK

```typescript
import { WIACropMonitoring } from '@wia/crop-monitoring';

const client = new WIACropMonitoring({
  apiKey: 'wia_api_key_1234567890abcdef',
  environment: 'production'
});

// Submit crop data
const result = await client.crops.create({
  cropId: 'CROP-2025-001',
  farmId: 'FARM-KR-12345',
  cropType: 'rice',
  measurements: {
    plantHeight: { value: 45.5, unit: 'cm' },
    chlorophyllSPAD: { value: 42.0, unit: 'SPAD' }
  }
});

// AI disease detection
const detection = await client.ai.detectDisease({
  cropId: 'CROP-2025-001',
  image: imageBuffer,
  cropType: 'tomato'
});

console.log('Detected:', detection.detections);
```

### 7.2 Python SDK

```python
from wia_crop_monitoring import CropMonitoringClient

client = CropMonitoringClient(api_key='wia_api_key_1234567890abcdef')

# Submit crop data
result = client.crops.create(
    crop_id='CROP-2025-001',
    farm_id='FARM-KR-12345',
    crop_type='rice',
    measurements={
        'plant_height': {'value': 45.5, 'unit': 'cm'},
        'chlorophyll_spad': {'value': 42.0, 'unit': 'SPAD'}
    }
)

# Yield prediction
forecast = client.forecast.yield_estimate('CROP-2025-001')
print(f"Estimated yield: {forecast['yieldEstimate']['value']} kg/ha")
```

### 7.3 cURL Example

```bash
# Submit crop data
curl -X POST https://api.crop-monitoring.wiastandards.com/v1/crops \
  -H "Authorization: Bearer wia_api_key_1234567890abcdef" \
  -H "Content-Type: application/json" \
  -d '{
    "cropId": "CROP-2025-001",
    "farmId": "FARM-KR-12345",
    "cropType": "rice",
    "measurements": {
      "plantHeight": {"value": 45.5, "unit": "cm"}
    }
  }'
```

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial API specification |

---

**Philosophy**: 弘益人間 (Benefit All Humanity)
**License**: MIT
**Contact**: api-support@wiastandards.com
**Documentation**: https://docs.crop-monitoring.wiastandards.com


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

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-2-API-INTERFACE. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P2-API-INTERFACE-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-2-API-INTERFACE validation when the
  organization signs the manifest itself.
- **Multi-tenant gateway** — one shared runtime serves multiple
  tenants via header-based isolation. Typically backed by a
  rate-limited gateway (Envoy or NGINX) and a shared OAuth 2.1
  identity provider. The manifest is per-tenant; the runtime
  publishes a federation manifest that aggregates tenant manifests.
- **Federated mesh** — multiple runtimes peer to one another and
  publish their manifests to a directory service. Each peer signs
  its own manifest; the directory service signs the aggregated
  index. This is the topology used by cross-organization deployments
  that need to compose conformance.
- **Air-gapped batch** — no network connection between the runtime
  and the directory service. The runtime emits a signed evidence
  package on each batch and the operator transports the package via
  out-of-band channels. This is the topology used by regulators that
  prohibit live connectivity from sensitive environments.

Implementations declare their topology in the manifest (see Annex I).
A topology change MUST be reflected in a new manifest signature; the
prior topology's manifest remains valid for the deprecation window
described in Annex H to preserve audit traceability.
