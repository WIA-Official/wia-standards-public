# WIA Data Warehouse API Standard
## Phase 2 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #10B981 (Emerald - DATA domain)

---

## Table of Contents

1. [Overview](#overview)
2. [API Architecture](#api-architecture)
3. [Authentication](#authentication)
4. [Endpoints](#endpoints)
5. [Request/Response Formats](#requestresponse-formats)
6. [Error Handling](#error-handling)
7. [Rate Limiting](#rate-limiting)
8. [Examples](#examples)

---

## Overview

### 1.1 Purpose

The WIA Data Warehouse API Standard defines RESTful APIs for interacting with data warehouses, enabling programmatic access to dimensional data, metadata, and analytics functions.

**Core Capabilities**:
- Query fact and dimension tables
- Execute OLAP operations (slice, dice, drill-down, roll-up)
- Manage ETL pipelines
- Access metadata and data lineage
- Monitor warehouse performance

### 1.2 API Design Principles

1. **RESTful**: Follow REST architectural constraints
2. **Stateless**: Each request contains all necessary information
3. **Versioned**: Support multiple API versions
4. **Secure**: HTTPS, authentication, authorization
5. **Performant**: Pagination, caching, compression

---

## API Architecture

### 2.1 Base URL

```
https://api.datawarehouse.{organization}.com/v1
```

### 2.2 API Versioning

Version is specified in the URL path:
- `/v1/` - Version 1 (current)
- `/v2/` - Version 2 (future)

### 2.3 Content Type

All requests and responses use JSON:
```
Content-Type: application/json
Accept: application/json
```

---

## Authentication

### 3.1 API Key Authentication

Include API key in request header:
```http
Authorization: Bearer {api_key}
```

### 3.2 OAuth 2.0

For user-specific operations:
```http
Authorization: Bearer {oauth_token}
```

### 3.3 Service Account

For service-to-service:
```http
Authorization: ServiceAccount {service_account_key}
```

---

## Endpoints

### 4.1 Query Endpoints

#### Execute Query
```http
POST /v1/query
Content-Type: application/json

{
  "sql": "SELECT * FROM fact_sales WHERE date_key = 20250115 LIMIT 100",
  "format": "json|csv|parquet",
  "use_cache": true
}
```

**Response**:
```json
{
  "query_id": "q-12345-abc",
  "status": "completed",
  "rows_returned": 100,
  "execution_time_ms": 234,
  "data": [
    {
      "fact_sales_key": 1,
      "date_key": 20250115,
      "sales_amount": 1250.00
    }
  ],
  "metadata": {
    "columns": [
      {"name": "fact_sales_key", "type": "bigint"},
      {"name": "date_key", "type": "int"},
      {"name": "sales_amount", "type": "decimal"}
    ]
  }
}
```

#### Get Query Status
```http
GET /v1/query/{query_id}/status
```

### 4.2 Dimension Endpoints

#### Get Dimension Data
```http
GET /v1/dimensions/{dimension_name}?limit=100&offset=0
```

#### Get Dimension by Natural Key
```http
GET /v1/dimensions/{dimension_name}/{natural_key}
```

#### Create Dimension Record (SCD Type 1)
```http
POST /v1/dimensions/{dimension_name}

{
  "customer_id": "CUST-12345",
  "customer_name": "John Doe",
  "email": "john@example.com",
  "city": "Seoul"
}
```

#### Update Dimension Record (SCD Type 2)
```http
PUT /v1/dimensions/{dimension_name}/{natural_key}

{
  "email": "newemail@example.com",
  "scd_type": 2
}
```

### 4.3 Fact Endpoints

#### Insert Fact Records
```http
POST /v1/facts/{fact_table_name}

{
  "records": [
    {
      "date_key": 20250115,
      "product_key": 1001,
      "store_key": 5,
      "customer_key": 10234,
      "sales_amount": 1250.00,
      "quantity_sold": 3
    }
  ]
}
```

#### Get Fact Aggregations
```http
POST /v1/facts/{fact_table_name}/aggregate

{
  "measures": ["sum(sales_amount)", "count(*)"],
  "dimensions": ["date_key", "product_key"],
  "filters": {
    "date_key": {"gte": 20250101, "lte": 20250131},
    "product_key": {"in": [1001, 1002, 1003]}
  },
  "group_by": ["date_key", "product_key"],
  "order_by": ["sum(sales_amount) DESC"]
}
```

### 4.4 Metadata Endpoints

#### Get Schema Information
```http
GET /v1/metadata/schemas
GET /v1/metadata/schemas/{schema_name}/tables
GET /v1/metadata/tables/{table_name}
```

#### Get Data Lineage
```http
GET /v1/metadata/lineage/{table_name}
```

**Response**:
```json
{
  "table_name": "fact_sales",
  "upstream": [
    {
      "source_system": "ERP",
      "source_table": "orders",
      "transformation": "ETL-001"
    }
  ],
  "downstream": [
    {
      "target": "sales_summary_mv",
      "type": "materialized_view"
    }
  ]
}
```

### 4.5 ETL Endpoints

#### Trigger ETL Job
```http
POST /v1/etl/jobs/{job_name}/run

{
  "parameters": {
    "start_date": "2025-01-01",
    "end_date": "2025-01-31"
  }
}
```

#### Get ETL Job Status
```http
GET /v1/etl/jobs/{job_id}/status
```

---

## Request/Response Formats

### 5.1 Pagination

```http
GET /v1/dimensions/dim_customer?limit=100&offset=200
```

**Response**:
```json
{
  "data": [...],
  "pagination": {
    "total_rows": 10000,
    "limit": 100,
    "offset": 200,
    "has_more": true
  }
}
```

### 5.2 Filtering

```json
{
  "filters": {
    "field_name": {"operator": "value"},
    "city": {"eq": "Seoul"},
    "sales_amount": {"gte": 1000, "lte": 5000},
    "product_category": {"in": ["Electronics", "Computers"]}
  }
}
```

**Supported Operators**:
- `eq`: Equal
- `ne`: Not equal
- `gt`: Greater than
- `gte`: Greater than or equal
- `lt`: Less than
- `lte`: Less than or equal
- `in`: In list
- `like`: Pattern match

---

## Error Handling

### 6.1 Error Response Format

```json
{
  "error": {
    "code": "INVALID_QUERY",
    "message": "Syntax error in SQL query",
    "details": "Line 1: Unexpected token 'FORM'",
    "request_id": "req-abc123"
  }
}
```

### 6.2 HTTP Status Codes

| Code | Meaning | Use Case |
|------|---------|----------|
| 200 | OK | Successful request |
| 201 | Created | Resource created |
| 400 | Bad Request | Invalid input |
| 401 | Unauthorized | Missing/invalid auth |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource doesn't exist |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server error |
| 503 | Service Unavailable | Maintenance mode |

---

## Rate Limiting

### 7.1 Rate Limit Headers

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 997
X-RateLimit-Reset: 1640995200
```

### 7.2 Default Limits

- **Standard tier**: 1000 requests/hour
- **Premium tier**: 10000 requests/hour
- **Enterprise tier**: Unlimited

---

## Examples

### 8.1 Complete Query Example

```bash
curl -X POST https://api.datawarehouse.example.com/v1/query \
  -H "Authorization: Bearer api_key_12345" \
  -H "Content-Type: application/json" \
  -d '{
    "sql": "SELECT d.year, d.month, p.category, SUM(f.sales_amount) as total_sales FROM fact_sales f JOIN dim_date d ON f.date_key = d.date_key JOIN dim_product p ON f.product_key = p.product_key WHERE d.year = 2025 GROUP BY d.year, d.month, p.category ORDER BY total_sales DESC LIMIT 10"
  }'
```

### 8.2 Dimension Update (SCD Type 2)

```bash
curl -X PUT https://api.datawarehouse.example.com/v1/dimensions/dim_customer/CUST-12345 \
  -H "Authorization: Bearer api_key_12345" \
  -H "Content-Type: application/json" \
  -d '{
    "email": "newemail@example.com",
    "city": "Busan",
    "scd_type": 2
  }'
```

---

**License**: MIT
**Copyright**: © 2025 WIA Standards Committee
**Philosophy**: 弘益人間 (Benefit All Humanity)


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
