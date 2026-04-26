# WIA-DATA-005: Data Quality - Phase 2 API Specification

**Version:** 1.0
**Status:** Draft
**Last Updated:** 2025-12-26

---

## Overview

This document defines the RESTful API specification for the WIA-DATA-005 Data Quality standard. It provides standardized endpoints for data quality profiling, validation, monitoring, and governance.

## Base URL

```
https://api.example.com/wia/data-quality/v1
```

## Authentication

All API requests require authentication using OAuth 2.0 Bearer tokens or API keys.

```http
Authorization: Bearer <access_token>
```

or

```http
X-API-Key: <api_key>
```

## Common Response Format

### Success Response

```json
{
  "status": "success",
  "data": {},
  "metadata": {
    "requestId": "uuid",
    "timestamp": "ISO8601"
  }
}
```

### Error Response

```json
{
  "status": "error",
  "error": {
    "code": "ERROR_CODE",
    "message": "Human-readable error message",
    "details": {}
  },
  "metadata": {
    "requestId": "uuid",
    "timestamp": "ISO8601"
  }
}
```

## API Endpoints

### 1. Data Profiling

#### Profile Dataset

Generate quality profile for a dataset.

```http
POST /datasets/{datasetId}/profile
```

**Request Body:**
```json
{
  "sampleSize": 10000,
  "columns": ["col1", "col2"],
  "includeDistributions": true,
  "detectPatterns": true
}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "profileId": "uuid",
    "status": "completed",
    "profile": {
      // DataQualityProfile object
    }
  }
}
```

#### Get Profile

Retrieve existing profile.

```http
GET /profiles/{profileId}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "profile": {
      // DataQualityProfile object
    }
  }
}
```

### 2. Validation Rules

#### Create Validation Rule

```http
POST /rules
```

**Request Body:**
```json
{
  "ruleName": "Email Format Check",
  "description": "Validates email format",
  "ruleType": "format",
  "severity": "high",
  "target": {
    "dataset": "customers",
    "column": "email"
  },
  "condition": {
    "operator": "regex",
    "pattern": "^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\\.[a-zA-Z]{2,}$"
  }
}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "ruleId": "uuid",
    "rule": {
      // ValidationRule object
    }
  }
}
```

#### List Validation Rules

```http
GET /rules?dataset={dataset}&active={true|false}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "rules": [
      // Array of ValidationRule objects
    ],
    "pagination": {
      "total": 100,
      "page": 1,
      "pageSize": 20
    }
  }
}
```

#### Update Validation Rule

```http
PUT /rules/{ruleId}
```

#### Delete Validation Rule

```http
DELETE /rules/{ruleId}
```

### 3. Validation Execution

#### Run Validation

Execute validation rules against dataset.

```http
POST /validations
```

**Request Body:**
```json
{
  "datasetId": "customers",
  "ruleIds": ["uuid-1", "uuid-2"],
  "sampleSize": 1000,
  "stopOnFirstFailure": false
}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "validationId": "uuid",
    "status": "running",
    "estimatedCompletion": "ISO8601"
  }
}
```

#### Get Validation Results

```http
GET /validations/{validationId}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "validationId": "uuid",
    "status": "completed",
    "results": [
      // Array of ValidationResult objects
    ],
    "summary": {
      "totalRules": 10,
      "passed": 8,
      "failed": 2,
      "passRate": 80.0
    }
  }
}
```

### 4. Quality Metrics

#### Submit Metrics

```http
POST /metrics
```

**Request Body:**
```json
{
  "metrics": [
    {
      "metricName": "completeness",
      "datasetId": "customers",
      "dimension": "completeness",
      "value": 95.5,
      "unit": "percentage",
      "timestamp": "2025-12-26T10:00:00Z"
    }
  ]
}
```

#### Query Metrics

```http
GET /metrics?datasetId={dataset}&dimension={dimension}&from={ISO8601}&to={ISO8601}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "metrics": [
      // Array of QualityMetric objects
    ]
  }
}
```

### 5. Issues Management

#### Create Issue

```http
POST /issues
```

**Request Body:**
```json
{
  "issueType": "invalid",
  "severity": "high",
  "description": "Invalid email format",
  "location": {
    "dataset": "customers",
    "column": "email",
    "row": 12345
  }
}
```

#### List Issues

```http
GET /issues?status={status}&severity={severity}&assignedTo={userId}
```

#### Update Issue

```http
PUT /issues/{issueId}
```

**Request Body:**
```json
{
  "status": "resolved",
  "resolution": "Fixed via data cleansing script",
  "assignedTo": "user-uuid"
}
```

### 6. Data Cleansing

#### Create Cleansing Job

```http
POST /cleansing-jobs
```

**Request Body:**
```json
{
  "datasetId": "customers",
  "operations": [
    {
      "type": "trim",
      "columns": ["name", "email"]
    },
    {
      "type": "standardize",
      "column": "email",
      "transformation": "lowercase"
    }
  ]
}
```

#### Get Cleansing Job Status

```http
GET /cleansing-jobs/{jobId}
```

### 7. Dashboards and Reports

#### Get Quality Dashboard

```http
GET /dashboards/quality?datasetId={dataset}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "overallScore": 92.5,
    "dimensions": {
      "accuracy": 95.0,
      "completeness": 90.0,
      "consistency": 92.0,
      "timeliness": 94.0,
      "validity": 93.0,
      "uniqueness": 91.0
    },
    "topIssues": [
      // Top 5 quality issues
    ]
  }
}
```

#### Generate Quality Report

```http
POST /reports
```

**Request Body:**
```json
{
  "reportType": "quality-scorecard",
  "datasetId": "customers",
  "period": {
    "from": "2025-12-01",
    "to": "2025-12-26"
  },
  "format": "pdf"
}
```

## Webhooks

Subscribe to quality events.

```http
POST /webhooks
```

**Request Body:**
```json
{
  "url": "https://your-app.com/webhooks/quality",
  "events": ["validation.failed", "issue.created", "threshold.exceeded"],
  "secret": "webhook-secret"
}
```

**Webhook Payload:**
```json
{
  "event": "validation.failed",
  "timestamp": "ISO8601",
  "data": {
    "validationId": "uuid",
    "ruleId": "uuid",
    "failedRecords": 150
  },
  "signature": "HMAC-SHA256 signature"
}
```

## Rate Limiting

- 1000 requests per minute per API key
- 10,000 requests per hour per API key
- Rate limit headers included in responses:

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1640534400
```

## Pagination

All list endpoints support pagination:

```http
GET /rules?page=2&pageSize=50
```

## Error Codes

| Code | Message | Description |
|------|---------|-------------|
| 400 | Bad Request | Invalid request parameters |
| 401 | Unauthorized | Missing or invalid authentication |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource not found |
| 409 | Conflict | Resource conflict |
| 422 | Unprocessable Entity | Validation error |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server error |
| 503 | Service Unavailable | Service temporarily unavailable |

## SDK Support

Official SDKs available for:
- Python
- JavaScript/TypeScript
- Java
- Go
- C#

---

**Copyright © 2025 SmileStory Inc. / WIA**
**弘益人間 (홍익인간) · Benefit All Humanity**


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

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-2-API validation when the
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
