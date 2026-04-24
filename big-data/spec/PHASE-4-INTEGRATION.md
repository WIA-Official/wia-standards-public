# WIA-DATA-001: Phase 4 — Integration Specification

**Version:** 1.1.0
**Status:** Active
**Last Updated:** 2026-04-25
**Philosophy:** 弘益人間 (Hongik Ingan) · Benefit All Humanity

---

## 1. Introduction

Phase 4 specifies **how WIA-DATA-001 federates with the surrounding data ecosystem**: open lakehouse table formats, catalogs, BI tools, orchestrators, transformation frameworks, lineage systems, and migration paths. The objective is to deliver *open*, not lock-in: any WIA dataset must be readable by Spark, Trino, DuckDB, Polars, or Snowflake External Volumes without re-materialisation, and any query engine must be usable as a front end without re-ingest.

### 1.1 Normative references

- Apache Iceberg 1.5 — Table Format Specification
- Delta Lake 3.1 — Protocol v3 (Reader v3 / Writer v7)
- Apache Hudi 0.14 — MoR / CoW tables
- Apache Parquet 2.9; Apache Avro 1.11; Apache ORC 2.0
- Apache Arrow 15 — IPC, Flight, and ADBC
- Substrait 0.42 — cross-engine logical plan
- Iceberg REST Catalog 1.5; Unity Catalog 0.2 (OSS); Nessie 0.83
- OpenLineage 1.1; Marquez 0.46
- dbt 1.9; Apache Airflow 2.10 / Airflow 3.0; Dagster 1.9
- OpenID Connect Federation 1.0 — for SSO into BI tools

---

## 2. Lakehouse table format support

Every WIA engine MUST read **Iceberg** natively and MUST support at least one writer. Reading Delta and Hudi MAY be implemented via the engine or via an interop shim (Iceberg-Hive-compatible metadata, XTable); writing them is OPTIONAL but if implemented MUST be strictly spec-conformant.

### 2.1 Iceberg v2 — reference writer

```json
{
  "format-version": 2,
  "properties": {
    "write.format.default":              "parquet",
    "write.parquet.compression-codec":   "zstd",
    "write.target-file-size-bytes":      134217728,
    "write.metadata.delete-after-commit.enabled": true,
    "write.metadata.previous-versions-max":       20,
    "history.expire.min-snapshots-to-keep":       10,
    "history.expire.max-snapshot-age-ms":         604800000
  }
}
```

- **Partition spec evolution**: supported; readers MUST honour partition-id v0/v1/v2.
- **Row-level operations**: equality deletes (MoR) for CDC sinks; position deletes for rewrite jobs; merge-on-read default for streaming, copy-on-write for batch recipes.
- **Snapshot commit**: atomic CAS on metadata pointer; reference implementation is `org.apache.iceberg.catalog.Catalog.newTransaction()`.

### 2.2 Delta Lake interop

Delta tables are accessed through the Uniform layer: WIA engines MUST generate Iceberg-compatible metadata alongside Delta's `_delta_log/*.json` when the table option `delta.uniform.iceberg.enabled=true` is present. For Delta-native writers:

- Protocol minReaderVersion ≥ 3, minWriterVersion ≥ 7.
- `columnMapping.mode = "name"` by default (safe rename).
- `deletionVectors.enabled = true` for CDC ingest; row-level deletes are durable.

### 2.3 Hudi interop

Supported through the DataSource v2 bridge; queries MUST work on *snapshot* and *read-optimised* tables. MoR compaction scheduling is surfaced in the WIA scheduler (Phase 2 `/jobs/{id}`).

---

## 3. Catalog integration

### 3.1 Iceberg REST Catalog (canonical)

Base: `https://catalog.wia-data.com/v1`. Endpoints follow the Iceberg OpenAPI (`/v1/{prefix}/namespaces`, `/v1/{prefix}/namespaces/{ns}/tables/{tbl}`). Authentication is OAuth 2.1 bearer; scopes gate namespace and table operations.

### 3.2 Alternative catalogs supported

| Catalog                       | Notes                                              |
|-------------------------------|----------------------------------------------------|
| **Nessie 0.83**               | Git-like branches/tags; required for blue/green DML |
| **Unity Catalog 0.2 (OSS)**   | Cross-lakehouse governance                          |
| **AWS Glue Data Catalog**     | Read-only interop; write via Iceberg Glue handler   |
| **Hive Metastore 3.1**        | Legacy read; writes MUST co-commit to Iceberg meta  |

Every catalog connector MUST report its capability set at `/capabilities` so WIA orchestrators know whether branching, rollback, or tags are available.

### 3.3 Branching and promotion

Nessie enables a tested pattern for regulated promotions:

```bash
nessie branch create ingest_2026_04_25 main
# ingest continues on the branch
nessie merge ingest_2026_04_25 main --strategy squash \
    --commit-message "Promote 2026-04-25 batch (n=52.1M rows)"
```

Rollbacks use `nessie merge --reverse` or `nessie branch delete --force`; both are zero-downtime because readers still pin to `main`.

---

## 4. Query engine compatibility

### 4.1 Required SQL surface

| Dialect           | Status            | Notes                                  |
|-------------------|-------------------|----------------------------------------|
| Trino 436+        | ✅ First-class    | Iceberg & Delta, RLS via Substrait plan |
| Apache Spark 3.5  | ✅ First-class    | Iceberg v2 writer, streaming + batch   |
| Presto 0.288+     | ✅ Supported      | Iceberg connector                      |
| DuckDB 0.10+      | ✅ Supported      | Iceberg reader + Parquet scan          |
| Snowflake         | ✅ Read via ExtVol | Iceberg External Tables                |
| Databricks        | ✅ Read / Write   | Delta, Iceberg Uniform                 |
| BigQuery          | ✅ Read           | BigLake Iceberg tables                 |
| ClickHouse 24.3+  | ✅ Read           | `iceberg()` table function             |
| Polars 0.20+ / pandas 2.2 | ✅ Read via Arrow  | Flight SQL + Iceberg table scans |

### 4.2 Substrait plan portability

Cross-engine plan migration uses Substrait 0.42. Reference producers: Spark `PlanWriter.writeSubstrait()`, DuckDB `duckdb_to_substrait()`. WIA's query planner MUST emit Substrait when `?format=substrait` is appended to `/datasets/{id}/query`.

---

## 5. BI tools

### 5.1 First-class connectors

- **Tableau** — WIA JDBC (wia-jdbc-1.1.jar) or Arrow Flight SQL; Live and Hyper-extract modes.
- **Power BI** — Custom connector (`wia.mez`) + DirectQuery / Import; incremental refresh by `event_ts`.
- **Looker** — LookML dialect `wia-trino`; PDTs on the user's warehouse.
- **Superset** — SQLAlchemy `wia+flightsql://…` + native Trino cluster.
- **Metabase** — JDBC driver plugin; row-level security mapped from the Metabase group to CEL predicates.
- **Hex, Mode, Preset, Omni, Observable** — Arrow Flight SQL via ADBC.

### 5.2 SSO to BI

OpenID Connect Federation 1.0 lets a corporate IdP issue tokens accepted by the WIA data plane, the BI tool, and the catalog. Token exchange (RFC 8693) swaps the BI session token for a short-lived WIA token carrying the user's RLS/CLS claims. Every BI tool session MUST record `iss`, `sub`, and `aud` on the query audit log (§ 11).

---

## 6. Orchestrators and transformation

### 6.1 Apache Airflow (2.10 + 3.0)

```python
from airflow.decorators import dag, task
from airflow.providers.wia.operators.data import WIAQueryOperator, WIADatasetSensor
from pendulum import datetime

@dag(start_date=datetime(2026, 4, 1), schedule="@daily", catchup=False)
def daily_rollup():
    wait = WIADatasetSensor(
        task_id="wait_events",
        dataset="ds-events",
        min_snapshot_ts="{{ data_interval_end.isoformat() }}",
    )
    rollup = WIAQueryOperator(
        task_id="rollup",
        sql="""
            INSERT INTO warehouse.events_daily
            SELECT date_trunc('day', event_ts) dt, user_id, COUNT(*) cnt
              FROM ds_events
             WHERE event_ts >= :lo AND event_ts < :hi
          GROUP BY 1, 2
        """,
        parameters={
            "lo": "{{ data_interval_start }}",
            "hi": "{{ data_interval_end }}",
        },
    )
    wait >> rollup
```

### 6.2 Dagster

```python
from dagster import asset, AssetKey, AssetIn
from dagster_wia import wia_resource, materialize_with_wia

@asset(io_manager_key="wia_io",
       compute_kind="wia-data",
       partitions_def=DailyPartitionsDefinition("2026-04-01"))
def events_daily(context, events: AssetIn(AssetKey(["ds_events"]))):
    return materialize_with_wia(
        sql="""INSERT INTO warehouse.events_daily
               SELECT date_trunc('day', event_ts), user_id, COUNT(*)
                 FROM ds_events
                WHERE event_ts >= :lo AND event_ts < :hi
             GROUP BY 1, 2""",
        parameters={"lo": context.partition_time_window.start,
                    "hi": context.partition_time_window.end})
```

### 6.3 dbt + WIA adapter

`dbt-wia` configures the profile as:

```yaml
profile: wia_warehouse
target: prod
outputs:
  prod:
    type: wia
    method: flight_sql
    host:   flight.wia-data.com
    port:   443
    token:  "{{ env_var('WIA_TOKEN') }}"
    catalog: nessie
    schema:  warehouse
```

Incremental models use Iceberg MERGE:

```sql
{{ config(materialized="incremental", unique_key="user_id",
          incremental_strategy="merge",
          merge_update_columns=["segment","last_seen"]) }}
SELECT user_id,
       MAX(event_ts)  AS last_seen,
       ANY_VALUE(seg) AS segment
  FROM {{ source('raw','events') }}
{% if is_incremental() %}
 WHERE event_ts > (SELECT MAX(last_seen) FROM {{ this }})
{% endif %}
GROUP BY user_id
```

---

## 7. Lineage, provenance, governance

### 7.1 OpenLineage (required)

Every job — ingest, transform, BI refresh — emits `RunEvent`s to `https://lineage.wia-data.com/api/v1/lineage`. Facets:

- `wia.dataset`, `wia.snapshot_id`
- `wia.sensitive_columns` (CLS-protected)
- `wia.rls_policy` id
- `wia.query_cost` (rows, bytes)
- `openlineage.run.nominalTime` start/end

### 7.2 Column-level lineage

Spark listener `io.openlineage.spark:openlineage-spark:1.1.0` and dbt `on-run-end` hook emit column-level dependencies so downstream governance tools (Atlan, Collibra, DataHub) can map a PII field's full lifecycle.

### 7.3 Classification tags

Tags (`pii.email`, `pii.ssn`, `finance.revenue`, `phi.diagnosis`) attach to columns and propagate. Downstream queries touching tagged columns are audited; exports outside a region are blocked via CLS + policy.

### 7.4 Retention

Dataset-level retention policy; enforced by the expiration job. Sensitive classes (`pii.*`) support crypto-shredding: destroying the KMS key renders all persisted ciphertext unreadable without rewrites.

---

## 8. Cloud storage and file gateways

### 8.1 S3-compatible sinks

```yaml
sink:
  type: s3
  bucket: wia-lake-prod
  region: us-east-1
  path:   "warehouse/events/{year}/{month}/{day}/"
  format: parquet
  compression: zstd
  sse:    aws:kms
  kms_key_id: arn:aws:kms:us-east-1:123456789012:key/abcd-efgh
```

S3 Express One Zone and MinIO behave identically for the WIA contract. Conditional writes (`If-None-Match: *`) are used for commit files; retries are safe.

### 8.2 Google Cloud Storage

```yaml
sink:
  type: gcs
  bucket: wia-lake-prod
  path:   "warehouse/events/{year}/{month}/{day}/"
  format: parquet
  compression: zstd
  kms_key: projects/p/locations/us/keyRings/r/cryptoKeys/k
```

### 8.3 Azure Data Lake Storage Gen2

ADLS Gen2 hierarchical namespaces are supported; `abfss://…` URIs are canonical; RBAC + ACL are both enforced. Shared keys are deprecated for new deployments — use managed identities + OAuth.

---

## 9. Connector framework

### 9.1 Relational sources

| Source        | Mode                   | Features                                                  |
|---------------|------------------------|-----------------------------------------------------------|
| PostgreSQL    | pgoutput CDC / JDBC    | Logical replication, publication management               |
| MySQL         | binlog CDC / JDBC      | GTID tracking, schema drift alerts                        |
| SQL Server    | CDC tables / JDBC      | Mirrored database support                                 |
| Oracle        | LogMiner / XStream     | Minesession retention policy                              |

### 9.2 NoSQL sources

| Source              | Mode                   | Notes                           |
|---------------------|------------------------|---------------------------------|
| MongoDB ≥ 5         | Change streams         | Resume tokens persisted         |
| Cassandra ≥ 4       | CDC commit-log agent   | `nodetool` triggered snapshots  |
| DynamoDB            | Streams + KCL          | iteration via sharded hash      |
| Elasticsearch ≥ 8   | Scroll + PIT           | For periodic catch-up            |

### 9.3 Message queues

Apache Kafka, Pulsar, Kinesis — fully covered in Phase 3. Others:

- **RabbitMQ** (AMQP 0.9.1): fan-out exchanges with confirm-select.
- **AWS SNS/SQS**: SNS → SQS-FIFO for exactly-once semantics; FIFO messages include `MessageGroupId = wia.partition_key`.
- **NATS JetStream**: durable consumers, per-subject retention.
- **Google Pub/Sub**: exactly-once subscription enabled.

---

## 10. Migration paths

### 10.1 Data migration

```bash
wia-migrate \
  --source "jdbc:postgresql://db.corp.local:5432/analytics" \
  --destination "wia://prod/ds-events" \
  --schema-source postgres \
  --schema-target-registry https://registry.wia-data.com/v1 \
  --parallel 16 --batch-size 100000 \
  --idempotency-prefix migration-2026-04-25-
```

Checkpoints in `s3://wia-state/migrations/<id>`; resumable on SIGINT/SIGTERM.

### 10.2 Schema migration

```bash
wia-schema migrate --from avro --to wia \
  --input schema.avsc \
  --registry https://registry.wia-data.com/v1 \
  --subject com.wia.events.EventRecord \
  --compat BACKWARD
```

### 10.3 Dual-write window

For high-stakes migrations, a dual-write window (1–14 days) mirrors writes to old and new systems, WIA emits a daily *reconciliation report* hashing row-level content, and cutover is triggered only when mismatches are zero for three consecutive days.

---

## 11. Observability, audit, cost accounting

### 11.1 Query audit log

Every query emits an immutable audit row:

```json
{ "ts": "2026-04-25T14:30:00Z",
  "principal": "alice@corp",
  "auth":      { "iss": "https://sso.corp", "sub": "u-42", "aud": "wia" },
  "dataset":   "ds-events",
  "sql":       "SELECT … FROM events …",
  "cost":      { "rows_scanned": 125000000, "bytes_scanned": 52428800 },
  "duration_ms": 942,
  "result":    { "rows_returned": 100, "bytes_returned": 48210 },
  "trace_id":  "0a1b2c3d…" }
```

Audit is stored in a tamper-evident table (`events_audit`) with compact MoR and verified against an external witness every hour.

### 11.2 Cost attribution

Per-query cost → per-project budget. A `costCenter` claim on the JWT is mandatory for Tier 1; queries missing it are rejected with `403 cost_center_required`.

### 11.3 SLOs

| SLO                                  | Tier 1 target | Tier 2 target |
|--------------------------------------|---------------|---------------|
| Query API availability               | 99.95 %       | 99.5 %        |
| Mutation API availability            | 99.9 %        | 99.5 %        |
| Flight SQL p95 latency (100 k rows)  | < 1.5 s       | < 3.0 s       |
| Stream consumer lag p99              | < 5 s         | < 30 s        |
| CDC end-to-end lag (binlog → sink)   | < 10 s        | < 60 s        |

---

## 12. Regulated deployments

- **GDPR / CCPA**: DSR tooling (`POST /datasets/{id}/erase`) triggers row-level deletes + vector-key shredding; erasure receipt stored for 10 years.
- **HIPAA**: PHI columns MUST be tagged `phi.*`; CLS + encryption are mandatory.
- **PCI-DSS 4.0**: scope-limited vaults; cardholder data never leaves the tokenisation zone.
- **SOX / ISMS**: WIA Chain anchor of daily data-lineage hashes provides an auditable witness.

---

## 13. Compliance checklist

- [ ] Iceberg v2 reference format supported as both reader and writer.
- [ ] At least one additional catalog besides the built-in REST (Nessie, UC, Glue, HMS).
- [ ] Substrait plan export enabled.
- [ ] OpenLineage events on every ingest, transform, and BI refresh.
- [ ] RLS + CLS predicates expressed in CEL and applied server-side.
- [ ] SSO federation via OIDC Federation 1.0 with token exchange.
- [ ] CDC connectors for relational sources with log-based semantics.
- [ ] Migration CLI produces reconciliation report.
- [ ] Audit log tamper-evident and witness-backed.
- [ ] Tier-specific SLOs measured and published.

---

## 14. References

1. Apache Iceberg 1.5 Table Format Spec
2. Delta Lake 3.1 Protocol
3. Apache Hudi 0.14 docs
4. Apache Arrow 15 — IPC, Flight SQL, ADBC
5. Substrait 0.42 logical plan
6. Iceberg REST Catalog 1.5; Nessie 0.83; Unity Catalog 0.2
7. OpenLineage 1.1 spec; Marquez 0.46
8. dbt 1.9 + `dbt-core/adapters` interfaces
9. Apache Airflow 2.10 / 3.0 provider model
10. Dagster 1.9 asset-oriented orchestration
11. OIDC Federation 1.0; RFC 8693 Token Exchange
12. Common Expression Language 1.0 (row-/column-level predicates)

---

**© 2026 WIA — World Certification Industry Association**
*弘益人間 · Benefit All Humanity*
