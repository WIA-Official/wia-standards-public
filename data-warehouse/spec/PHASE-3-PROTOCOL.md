# WIA Data Warehouse Protocol Standard
## Phase 3 Specification

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
2. [Communication Protocols](#communication-protocols)
3. [Data Transfer Protocols](#data-transfer-protocols)
4. [ETL Orchestration Protocol](#etl-orchestration-protocol)
5. [Query Execution Protocol](#query-execution-protocol)
6. [Security Protocols](#security-protocols)
7. [Monitoring and Logging](#monitoring-and-logging)

---

## Overview

### 1.1 Purpose

The WIA Data Warehouse Protocol Standard defines communication and data exchange protocols for data warehouse systems, ensuring interoperability between ETL tools, query engines, and BI platforms.

**Protocol Coverage**:
- Data ingestion and extraction
- Query execution and result streaming
- ETL job orchestration
- Metadata exchange
- Security and encryption

---

## Communication Protocols

### 2.1 HTTP/HTTPS

**Primary protocol** for RESTful API communication:
- HTTPS (TLS 1.2+) for all production traffic
- HTTP/2 for improved performance
- WebSocket for real-time streaming

### 2.2 gRPC

**High-performance RPC** for internal services:
```protobuf
service DataWarehouse {
  rpc ExecuteQuery(QueryRequest) returns (stream QueryResult);
  rpc LoadData(stream DataBatch) returns (LoadResponse);
  rpc GetMetadata(MetadataRequest) returns (MetadataResponse);
}
```

### 2.3 JDBC/ODBC

**Standard database connectivity**:
- JDBC Driver: `com.wia.datawarehouse.jdbc.Driver`
- ODBC DSN configuration for BI tools
- Connection string format:
  ```
  jdbc:wia-dw://{host}:{port}/{database}?user={user}&password={password}
  ```

---

## Data Transfer Protocols

### 3.1 Bulk Data Load Protocol

#### CSV Upload
```http
POST /v1/data/load/csv
Content-Type: multipart/form-data

{
  "file": binary_csv_data,
  "table_name": "fact_sales",
  "delimiter": ",",
  "skip_header": true,
  "encoding": "UTF-8"
}
```

#### Parquet Upload
```http
POST /v1/data/load/parquet
Content-Type: application/octet-stream

{
  "file": binary_parquet_data,
  "table_name": "fact_sales",
  "schema_validation": true
}
```

### 3.2 Streaming Data Protocol

**Apache Kafka Integration**:
```json
{
  "protocol": "kafka",
  "bootstrap_servers": ["kafka1:9092", "kafka2:9092"],
  "topic": "sales-events",
  "consumer_group": "dw-consumer",
  "auto_offset_reset": "earliest",
  "target_table": "fact_sales"
}
```

**Amazon Kinesis Integration**:
```json
{
  "protocol": "kinesis",
  "stream_name": "sales-stream",
  "region": "us-east-1",
  "shard_iterator_type": "LATEST",
  "target_table": "fact_sales"
}
```

### 3.3 Change Data Capture (CDC) Protocol

```json
{
  "cdc_protocol": {
    "method": "log_based|trigger_based|timestamp_based",
    "source_database": {
      "type": "mysql|postgresql|oracle",
      "host": "source-db.example.com",
      "port": 3306,
      "database": "production"
    },
    "capture_tables": [
      {
        "source_table": "orders",
        "target_table": "fact_orders",
        "capture_deletes": true,
        "capture_updates": true
      }
    ],
    "checkpoint_interval_seconds": 60
  }
}
```

---

## ETL Orchestration Protocol

### 4.1 Job Definition Format

```json
{
  "job_id": "etl-daily-sales",
  "job_name": "Daily Sales ETL",
  "schedule": "0 2 * * *",
  "dependencies": ["etl-customer-refresh", "etl-product-refresh"],
  "stages": [
    {
      "stage_name": "extract",
      "type": "extract",
      "source": {
        "type": "mysql",
        "connection": "source-db",
        "query": "SELECT * FROM orders WHERE order_date = CURRENT_DATE - 1"
      }
    },
    {
      "stage_name": "transform",
      "type": "transform",
      "transformations": [
        {
          "operation": "join",
          "join_table": "dim_customer",
          "join_key": "customer_id"
        },
        {
          "operation": "aggregate",
          "group_by": ["customer_id", "product_id"],
          "measures": ["SUM(amount)", "COUNT(*)"]
        }
      ]
    },
    {
      "stage_name": "load",
      "type": "load",
      "target": {
        "table": "fact_sales",
        "load_strategy": "incremental|full|merge"
      }
    }
  ],
  "error_handling": {
    "max_retries": 3,
    "retry_delay_seconds": 300,
    "on_failure": "alert|skip|abort"
  }
}
```

### 4.2 Job Status Protocol

```json
{
  "job_id": "etl-daily-sales-20250115-001",
  "status": "running|completed|failed|cancelled",
  "progress": {
    "current_stage": "transform",
    "stage_progress_percent": 65,
    "overall_progress_percent": 45,
    "rows_processed": 1234567,
    "estimated_completion": "2025-01-15T03:30:00Z"
  },
  "metrics": {
    "start_time": "2025-01-15T02:00:00Z",
    "end_time": null,
    "duration_seconds": 5400,
    "rows_extracted": 2000000,
    "rows_transformed": 1800000,
    "rows_loaded": 1234567,
    "errors": 123
  }
}
```

---

## Query Execution Protocol

### 5.1 Query Submission

```json
{
  "query_id": "uuid-generated",
  "sql": "SELECT ...",
  "execution_mode": "synchronous|asynchronous",
  "timeout_seconds": 300,
  "result_format": "json|csv|parquet",
  "use_cache": true,
  "max_rows": 10000
}
```

### 5.2 Query Result Streaming

**Server-Sent Events (SSE)**:
```http
GET /v1/query/{query_id}/stream
Accept: text/event-stream

event: metadata
data: {"columns": [...], "total_rows": 10000}

event: data
data: {"row": [1, "value", 123.45]}

event: data
data: {"row": [2, "value2", 456.78]}

event: complete
data: {"rows_returned": 10000, "execution_time_ms": 234}
```

### 5.3 Prepared Statements

```json
{
  "prepare_query": {
    "statement_id": "stmt-001",
    "sql": "SELECT * FROM fact_sales WHERE date_key = ? AND store_key = ?",
    "parameter_types": ["int", "int"]
  },
  "execute_prepared": {
    "statement_id": "stmt-001",
    "parameters": [20250115, 5]
  }
}
```

---

## Security Protocols

### 6.1 Encryption

**Data at Rest**:
- AES-256 encryption for all stored data
- Transparent Data Encryption (TDE) support
- Customer-managed encryption keys (CMEK)

**Data in Transit**:
- TLS 1.2+ for all network traffic
- Certificate pinning for mobile/desktop clients
- mTLS for service-to-service communication

### 6.2 Authentication Methods

```json
{
  "authentication": {
    "method": "api_key|oauth2|saml|ldap",
    "api_key": {
      "header": "Authorization",
      "prefix": "Bearer"
    },
    "oauth2": {
      "authorization_endpoint": "https://auth.example.com/oauth/authorize",
      "token_endpoint": "https://auth.example.com/oauth/token",
      "scopes": ["warehouse:read", "warehouse:write"]
    }
  }
}
```

### 6.3 Authorization Protocol

**Role-Based Access Control (RBAC)**:
```json
{
  "principal": "user@example.com",
  "roles": ["analyst", "data_engineer"],
  "permissions": {
    "fact_sales": ["SELECT"],
    "dim_customer": ["SELECT", "INSERT", "UPDATE"],
    "dim_product": ["SELECT"]
  },
  "row_level_security": {
    "table": "fact_sales",
    "filter": "store_key IN (SELECT store_key FROM user_stores WHERE user_id = CURRENT_USER)"
  }
}
```

---

## Monitoring and Logging

### 6.1 Metrics Protocol

**Prometheus Format**:
```
# HELP dw_query_duration_seconds Query execution duration
# TYPE dw_query_duration_seconds histogram
dw_query_duration_seconds_bucket{le="0.1"} 100
dw_query_duration_seconds_bucket{le="1.0"} 450
dw_query_duration_seconds_bucket{le="10.0"} 980
dw_query_duration_seconds_sum 4523.4
dw_query_duration_seconds_count 1000

# HELP dw_rows_scanned_total Total rows scanned
# TYPE dw_rows_scanned_total counter
dw_rows_scanned_total 123456789
```

### 6.2 Logging Protocol

**Structured JSON Logs**:
```json
{
  "timestamp": "2025-01-15T10:30:00.123Z",
  "level": "INFO|WARN|ERROR",
  "service": "query-engine",
  "query_id": "q-12345",
  "user": "user@example.com",
  "action": "query_execution",
  "duration_ms": 234,
  "rows_returned": 1000,
  "message": "Query executed successfully",
  "metadata": {
    "tables_accessed": ["fact_sales", "dim_date"],
    "cache_hit": true
  }
}
```

### 6.3 Audit Trail Protocol

```json
{
  "audit_event": {
    "event_id": "audit-12345",
    "timestamp": "2025-01-15T10:30:00Z",
    "event_type": "data_access|data_modification|schema_change",
    "principal": "user@example.com",
    "action": "SELECT",
    "resource": "fact_sales",
    "result": "success|failure",
    "details": {
      "rows_accessed": 1000,
      "filters_applied": "date_key = 20250115"
    }
  }
}
```

---

**License**: MIT
**Copyright**: © 2025 WIA Standards Committee
**Philosophy**: 弘益人間 (Benefit All Humanity)


## Annex E — Implementation Notes for PHASE-3-PROTOCOL

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-3-PROTOCOL.

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
evidence for PHASE-3-PROTOCOL. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-3-protocol/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-3-PROTOCOL with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-3-PROTOCOL does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-3-PROTOCOL.
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
for PHASE-3-PROTOCOL. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P3-PROTOCOL-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-3-PROTOCOL validation when the
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
