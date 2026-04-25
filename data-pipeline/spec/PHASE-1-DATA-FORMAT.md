# WIA-DATA-004: PHASE 1 — DATA FORMAT

**Version:** 1.1.0
**Status:** ✅ Deep-Published
**Last Updated:** 2026-04-25
**Conforms to:** Apache Parquet 2.10, Apache Avro 1.12, Apache Iceberg v2, Delta Lake 3.2, Apache Hudi 0.15, JSON Schema 2020-12, OpenLineage 1.21, Data Contract Specification 1.1.0 (Bitol/LF AI &amp; Data), Schema Registry (Confluent / Apicurio / Karapace).

---

## 1. Storage formats

### 1.1 Apache Parquet 2.10 (default for analytical layers)

Columnar, compressed, predicate-pushdown friendly. WIA-DATA-004 mandates:

- Page index (PARQUET-1404, GA in 2.9) for O(log n) row-group skipping.
- Column statistics MUST be present for every column (min, max, null_count, distinct_count when feasible).
- Bloom filters on high-cardinality predicate columns.
- Row group size 128 MB ± 25%, page size 1 MiB.
- Compression: ZSTD level 3 default; LZ4 for hot streaming sinks; SNAPPY for legacy compatibility.

```python
import pyarrow as pa, pyarrow.parquet as pq
schema = pa.schema([
    ("order_id",  pa.int64(),               nullable=False),
    ("user_id",   pa.int64(),               nullable=False),
    ("total_cents", pa.int64(),             nullable=False),
    ("currency",  pa.dictionary(pa.int8(), pa.string())),
    ("created_at", pa.timestamp("us","UTC"), nullable=False),
])
pq.write_table(table, "s3://lake/orders/dt=2026-04-25/part-0001.parquet",
               compression="zstd", compression_level=3,
               write_statistics=True, write_page_index=True,
               row_group_size=128*1024*1024, data_page_size=1*1024*1024,
               version="2.6", store_schema=True)
```

### 1.2 Apache Avro 1.12 (default for streaming and schema-registry use)

Compact binary, embedded schema, first-class evolution. Required logical types: `decimal`, `date`, `time-millis`, `timestamp-micros` (UTC), `uuid`.

```json
{
  "type":"record","namespace":"com.wia.events","name":"OrderPlaced",
  "fields":[
    {"name":"order_id","type":"long"},
    {"name":"user_id","type":"long"},
    {"name":"amount","type":{"type":"bytes","logicalType":"decimal","precision":12,"scale":2}},
    {"name":"currency","type":"string","default":"USD"},
    {"name":"created_at","type":{"type":"long","logicalType":"timestamp-micros"}},
    {"name":"properties","type":{"type":"map","values":"string"},"default":{}}
  ]
}
```

### 1.3 Lakehouse table formats

WIA-DATA-004 supports three open formats; an implementation MUST choose one as the canonical lakehouse.

| Format            | Strengths                                                              | Trade-offs                              |
|-------------------|------------------------------------------------------------------------|-----------------------------------------|
| Apache Iceberg v2 | Hidden partitioning, snapshot isolation, time-travel, schema evolution | Catalogue (Glue/Hive/Polaris) required  |
| Delta Lake 3.2    | UniForm (read as Iceberg), DML rich, Spark-native                      | Best inside Spark/Databricks ecosystem  |
| Apache Hudi 0.15  | Record-level upserts, CDC streaming                                    | Operationally heaviest                  |

Iceberg is the recommended default. Standard catalogue interface is the **Apache Polaris REST catalog** (Iceberg REST spec 1.5).

```sql
CREATE TABLE lake.orders (
  order_id    BIGINT NOT NULL,
  user_id     BIGINT NOT NULL,
  total_cents BIGINT NOT NULL,
  currency    STRING NOT NULL,
  created_at  TIMESTAMP NOT NULL
)
USING iceberg
PARTITIONED BY (days(created_at), bucket(16, user_id))
TBLPROPERTIES (
  'format-version'='2',
  'write.target-file-size-bytes'='134217728',
  'write.parquet.compression-codec'='zstd',
  'write.merge.mode'='copy-on-write'
);
```

### 1.4 Semi-structured: JSON, JSON-LD, NDJSON, CSV

JSON: UTF-8, RFC 8259. JSON-LD with pinned `@context` for machine-readable contracts. NDJSON for line-oriented streams. CSV: header row, RFC 4180, UTF-8, `,` separator, `"` quote, `\` escape.

---

## 2. Pipeline metadata schema (JSON Schema 2020-12)

### 2.1 Pipeline configuration

```json
{
  "$schema":"https://json-schema.org/draft/2020-12/schema",
  "$id":"https://schemas.wia-data-pipeline.org/Pipeline.json",
  "type":"object",
  "required":["id","name","version","owner","source","destination"],
  "properties":{
    "id":      {"type":"string","pattern":"^pip_[a-z0-9_-]{4,32}$"},
    "name":    {"type":"string","minLength":1,"maxLength":256},
    "version": {"type":"string","pattern":"^\\d+\\.\\d+\\.\\d+$"},
    "owner":   {"type":"string","format":"email"},
    "schedule":{"type":"string","description":"RFC 5545 RRULE or cron-5"},
    "enabled": {"type":"boolean","default":true},
    "retry_policy":{"$ref":"#/$defs/retryPolicy"},
    "timeout_seconds":{"type":"integer","minimum":1,"maximum":86400},
    "tags":    {"type":"array","items":{"type":"string"}}
  },
  "$defs":{
    "retryPolicy":{
      "type":"object",
      "properties":{
        "max_attempts":{"type":"integer","minimum":1,"maximum":10,"default":5},
        "backoff_strategy":{"enum":["fixed","exponential"]},
        "base_delay_seconds":{"type":"integer","minimum":1,"default":5},
        "max_delay_seconds":{"type":"integer","minimum":1,"default":300},
        "jitter":{"type":"boolean","default":true}
      }
    }
  }
}
```

### 2.2 Source / extraction

```json
{
  "source":{
    "type":"postgresql","connection_id":"prod-db",
    "extraction_mode":"cdc",
    "cdc":{"mechanism":"debezium","slot":"wia_pip_orders","publication":"wia_pip_orders_pub","snapshot_mode":"initial"},
    "incremental_key":"updated_at","watermark":"2026-04-24T00:00:00Z",
    "include_tables":["public.orders","public.order_items"],
    "exclude_columns":["users.email","users.phone"]
  }
}
```

### 2.3 Transformation steps

```json
{
  "transformations":[
    {"step_id":"clean","type":"sql",
     "operation":"clean","sql":"SELECT * EXCEPT (raw_blob) FROM {{ source('orders_raw') }} WHERE total_cents &gt;= 0"},
    {"step_id":"enrich","type":"python",
     "operation":"enrich",
     "python_function":"transforms.enrich_currency",
     "dependencies":["currencies"]}
  ]
}
```

### 2.4 Destination / load

```json
{
  "destination":{
    "type":"iceberg","catalog":"polaris-prod","namespace":"lake","table":"orders",
    "mode":"merge","merge_keys":["order_id"],"merge_when_matched":"update_all",
    "partition_spec":["days(created_at)","bucket(16,user_id)"],
    "properties":{"write.merge.mode":"copy-on-write"}
  }
}
```

---

## 3. Data Contract Specification 1.1.0 (Bitol)

The Linux Foundation **Open Data Contract Standard** is the wire form for producer ↔ consumer agreements.

```yaml
dataContractSpecification: "1.1.0"
id: orders-clean.v1
info:
  title: Orders (cleaned)
  version: 1.0.0
  owner: data-platform@company.com
  description: Cleaned, currency-normalised orders feed.
servers:
  production: { type: iceberg, location: lake.orders }
terms:
  usage: Internal analytics &amp; ML feature engineering. PII masked.
  limitations: Not authoritative for finance reconciliation.
  billing: free
  noticePeriod: P30D
schema:
  models:
    orders:
      type: table
      fields:
        order_id:   { type: long, required: true, primary: true }
        user_id:    { type: long, required: true }
        total_cents:{ type: long, required: true, examples: [1299, 4999] }
        currency:   { type: string, required: true, format: ISO-4217 }
        created_at: { type: timestamp, required: true }
quality:
  type: SodaCL
  specification: |
    checks for orders:
      - missing_count(order_id) = 0
      - duplicate_count(order_id) = 0
      - row_count between 1000 and 100000000
      - freshness(created_at) &lt; 1d
servicelevels:
  freshness: { description: data is updated daily by 02:00 UTC, threshold: P1D }
  retention: { description: 7 years per finance policy, period: P7Y }
```

---

## 4. OpenLineage 1.21 events

Every job execution emits four lifecycle events: `START`, `RUNNING` (optional, throttled), `COMPLETE`, `FAIL` or `ABORT`. The CompletedEvent attaches input/output datasets, schema facet, columnLineage facet, and run-level data quality facet.

```json
{
  "eventType":"COMPLETE",
  "eventTime":"2026-04-25T10:15:30.412Z",
  "producer":"https://wia-data-pipeline.org/oss/airflow/2.10",
  "schemaURL":"https://openlineage.io/spec/1-0-5/OpenLineage.json#/$defs/RunEvent",
  "run":{"runId":"5b9a..."},
  "job":{"namespace":"pip.daily_sales","name":"transform_orders"},
  "inputs":[{"namespace":"postgres://prod-db/app","name":"public.orders",
              "facets":{"schema":{"_producer":"...","fields":[...]}}}],
  "outputs":[{"namespace":"iceberg://polaris-prod","name":"lake.orders",
               "facets":{"columnLineage":{"fields":{
                  "total_cents":{"inputFields":[{"namespace":"...","name":"public.orders","field":"amount_in_cents"}]}}}}}]
}
```

Lineage events are routed to **Marquez** or **DataHub**; the WIA conformance suite verifies that every Iceberg snapshot has a matching OpenLineage run.

---

## 5. Event log schema (run history)

```json
{
  "execution_id":"exec_5b9a...",
  "pipeline_id":"pip_daily_sales",
  "start_time":"2026-04-25T02:00:00Z","end_time":"2026-04-25T02:14:03Z",
  "status":"completed",
  "trigger_type":"scheduled",
  "metrics":{
    "records_extracted":127500,"records_transformed":127450,
    "records_loaded":127450,"records_failed":50,
    "bytes_in":389127498,"bytes_out":201874322,
    "wallclock_seconds":843
  },
  "openlineage_run_id":"5b9a...",
  "iceberg_snapshot_id":"7841029384712340987",
  "data_quality":{"passed":12,"failed":0,"warnings":1}
}
```

---

## 6. Partitioning

### 6.1 Hidden partitioning (Iceberg-only, recommended)

```sql
PARTITIONED BY (days(created_at), bucket(16, user_id))
-- Predicate `WHERE created_at &gt;= '2026-04-25'` automatically prunes; users do not write `dt=...`
```

### 6.2 Hive-style partitioning (legacy interop)

```
s3://lake/orders/dt=2026-04-25/hour=10/part-0000.parquet
```

### 6.3 Sort order and z-order

`SORT BY (user_id, created_at)` improves dictionary encoding and predicate pushdown. Z-order on `(geohash, user_id)` for geo-temporal joins.

---

## 7. Naming conventions

- Datasets: `{domain}_{entity}_{layer}` (`sales_orders_raw`, `sales_orders_clean`, `sales_orders_curated`).
- Columns: lowercase snake_case. Identifier keys end in `_id`. Money columns end in `_cents` or `_micros` (no floats).
- Files: `part-{NNNN}-{uuid}.parquet`.

---

## 8. Schema evolution rules

| Change                                    | Avro          | Parquet       | Iceberg       |
|-------------------------------------------|---------------|---------------|---------------|
| Add nullable column with default          | ✅ backward   | ✅            | ✅            |
| Drop column                               | ✅ forward    | ✅            | ✅ (id-based) |
| Rename column                             | alias only    | breaking      | ✅ (id-based) |
| Promote int32 → int64                     | ✅            | ⚠ tooling    | ✅            |
| Promote float → double                    | ✅            | ⚠            | ✅            |
| Change required → optional                | ✅            | ✅            | ✅            |
| Change optional → required                | breaking      | breaking      | breaking      |

Schema Registry compatibility level MUST be set to `BACKWARD_TRANSITIVE` for analytical subjects, `FULL_TRANSITIVE` for inter-team event subjects.

---

## 9. Compression policy

| Layer       | Codec   | Reason                                    |
|-------------|---------|-------------------------------------------|
| Hot stream  | LZ4     | Lowest CPU on producer                    |
| Bronze lake | ZSTD-3  | Best size/CPU ratio                       |
| Silver lake | ZSTD-9  | Read-heavy; compress once, scan often     |
| Gold mart   | ZSTD-19 | Tiny, query-frequent                      |

---

## 10. PII handling

Columns marked `pii: true` in the data contract MUST be:

- Tokenised at ingest (FF3-1 format-preserving encryption per NIST SP 800-38G) when downstream layers do not need raw values.
- Masked in non-prod environments using deterministic hashing (HMAC-SHA-256 with environment-scoped salt).
- Redacted from `OpenLineage.run.facets.dataSource` field.

---

**弘益人間 — Benefit All Humanity**
*WIA — World Industry Association · © 2026 MIT License*
