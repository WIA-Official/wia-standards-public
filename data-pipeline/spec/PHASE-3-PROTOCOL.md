# WIA-DATA-004: PHASE 3 — PROTOCOL

**Version:** 1.1.0
**Status:** ✅ Deep-Published
**Last Updated:** 2026-04-25
**Conforms to:** Apache Airflow 2.10 (TaskFlow), Dagster 1.10 (software-defined assets), Prefect 3, Apache Kafka 3.8 (KIP-1018 exactly-once), Debezium 3 (Outbox pattern), OpenLineage 1.21, Iceberg REST 1.5, Schema Registry compat (BACKWARD/FULL), CloudEvents 1.0, RFC 6585 (rate limit), RFC 7234 (caching), TLS 1.3.

---

## 1. Orchestration protocol

### 1.1 DAG / asset graph definition

```yaml
dag:
  id: daily_sales_pipeline
  schedule: "0 2 * * *"
  max_active_runs: 1
  catchup: false
  default_args:
    retries: 3
    retry_delay_seconds: 300
    timeout_seconds: 3600
  tasks:
    - { id: extract_orders,   type: extract,   source: postgresql,    depends_on: [] }
    - { id: transform_orders, type: transform, operation: clean,      depends_on: [extract_orders] }
    - { id: load_iceberg,     type: load,      destination: iceberg,  depends_on: [transform_orders] }
    - { id: dq_gate,          type: quality,   gate: qg_orders,       depends_on: [load_iceberg] }
    - { id: notify,           type: notify,    channel: slack,        depends_on: [dq_gate] }
```

Modern equivalents (the WIA spec is orchestrator-agnostic):

```python
# Apache Airflow 2.10 TaskFlow API
from airflow.decorators import dag, task
from datetime import datetime, timedelta

@dag(schedule="0 2 * * *", start_date=datetime(2026,4,25), catchup=False,
     default_args={"retries":3,"retry_delay":timedelta(minutes=5)})
def daily_sales():
    @task
    def extract(): ...
    @task
    def transform(rows): ...
    @task
    def load(rows): ...
    @task
    def dq_gate(snapshot_id): ...
    load(transform(extract())) >> dq_gate("$snapshot$")
daily_sales()
```

```python
# Dagster 1.10 software-defined assets
import dagster as dg

@dg.asset
def orders_raw(): ...

@dg.asset(deps=["orders_raw"])
def orders_clean(orders_raw): ...

@dg.asset_check(asset="orders_clean")
def orders_clean_quality(): ...
```

### 1.2 Task execution protocol

States: `pending → running → success | failed | timeout | upstream_failed | skipped`. Heartbeat MUST be sent every 30 s; the orchestrator marks tasks `zombie` after 90 s of silence and triggers retry.

```json
{
  "task_execution":{
    "task_id":"extract_orders","execution_id":"exec_5b9a","state":"running",
    "start_time":"2026-04-25T02:00:00Z","heartbeat_interval_seconds":30,
    "timeout_seconds":3600,"retry_count":0,"max_retries":3,
    "worker_id":"airflow-worker-7","resource_class":"medium"
  }
}
```

---

## 2. Data transfer protocol

### 2.1 Batch transfer

```json
{
  "transfer":{
    "transfer_id":"xfer_abc123",
    "source":{"type":"s3","location":"s3://bronze/orders/dt=2026-04-25/",
              "format":"parquet","compression":"zstd"},
    "destination":{"type":"iceberg","catalog":"polaris-prod","namespace":"lake","table":"orders"},
    "mode":"merge","merge_keys":["order_id"],
    "batch_size":10000,"parallel_streams":8,
    "checkpoint":{"location":"s3://checkpoints/xfer_abc123/","interval_seconds":60}
  }
}
```

### 2.2 Stream transfer

```json
{
  "stream":{
    "stream_id":"stream_xyz789","protocol":"kafka",
    "bootstrap":"kafka.prod:9093","topic":"user-events",
    "consumer_group":"pipeline-processor",
    "isolation.level":"read_committed",
    "auto.offset.reset":"earliest",
    "max.poll.records":500,"poll.timeout.ms":1000,
    "schema_registry":"https://sr.prod:8081"
  }
}
```

---

## 3. Kafka exactly-once semantics — the three-rule recipe

Required to claim **exactly-once** end-to-end:

1. **Producer:** `enable.idempotence=true` (default since Kafka 3.0), `acks=all`, `max.in.flight.requests.per.connection ≤ 5`, stable `transactional.id`. Wrap writes in `producer.beginTransaction() / commitTransaction()`.
2. **Consumer:** `isolation.level=read_committed`. Disable auto-commit; commit offsets inside the producer transaction.
3. **Sink:** Idempotent destination — either an Iceberg `MERGE INTO` with merge keys, or a destination that supports a unique idempotency key column.

```java
// Java — producer-consumer-sink in one transaction
producer.initTransactions();
while (true) {
  ConsumerRecords<K,V> rs = consumer.poll(Duration.ofMillis(1000));
  if (rs.isEmpty()) continue;
  producer.beginTransaction();
  for (ConsumerRecord<K,V> r : rs)
    producer.send(new ProducerRecord<>("processed-events", r.key(), transform(r.value())));
  producer.sendOffsetsToTransaction(currentOffsets(rs), consumer.groupMetadata());
  producer.commitTransaction();
}
```

KIP-1018 (Kafka 3.8) hardens transactional protocol against zombie producers.

---

## 4. Change Data Capture — Debezium 3 + Outbox

### 4.1 CDC event format (Debezium envelope)

```json
{
  "before": null,
  "after":  {"order_id":12345,"user_id":789,"total_cents":9999,"created_at":"2026-04-25T10:00:00Z"},
  "source": {"version":"3.0.4","connector":"postgresql","name":"prod-db","ts_ms":1714032000000,
             "snapshot":"false","db":"app","schema":"public","table":"orders","txId":12387,"lsn":"0/16B374D8"},
  "op":"c","ts_ms":1714032000123,"transaction":{"id":"112233","total_order":1,"data_collection_order":1}
}
```

`op`: `c` create, `u` update, `d` delete, `r` snapshot read, `t` truncate.

### 4.2 Outbox pattern (Kafka Connect Outbox SMT)

```sql
CREATE TABLE outbox_events (
  id          UUID PRIMARY KEY,
  aggregate   TEXT NOT NULL,
  aggregate_id TEXT NOT NULL,
  event_type  TEXT NOT NULL,
  payload     JSONB NOT NULL,
  created_at  TIMESTAMPTZ NOT NULL DEFAULT now()
);

-- Application writes domain row + outbox row in same DB transaction.
-- Debezium tails outbox_events; SMT routes to per-aggregate Kafka topic.
```

This guarantees the event is published *if and only if* the database transaction committed — the canonical solution to dual-write inconsistency.

### 4.3 CDC subscription

```json
{
  "subscription":{
    "subscription_id":"sub_orders","source":"postgresql://prod-db:5432/app",
    "tables":["public.outbox_events"],
    "operations":["INSERT"],
    "destination":{"protocol":"kafka","topic_template":"events.{aggregate}"},
    "snapshot_mode":"initial"
  }
}
```

---

## 5. Idempotency

Every mutating call MUST carry `Idempotency-Key`:

```http
POST /v1/pipelines/pip_daily_sales/runs
X-Idempotency-Key: run_2026-04-25_02
```

Server:
- First request with that key: process and store response.
- Duplicate (within 24 h): return stored response, do NOT re-process.
- Key must be unique per logical operation; SDKs default to `f"{op}_{iso8601_floor_minute}"`.

---

## 6. Retry, back-off, circuit breaker

### 6.1 Exponential back-off with jitter (RFC 7231 §5.5.5 friendly)

```python
def retry_delay(attempt: int) -> float:
    base, mx = 5.0, 300.0
    delay = min(base * (2 ** attempt), mx)
    return delay + random.uniform(0, delay * 0.1)   # full jitter recommended
```

### 6.2 Retry policy

```json
{
  "retry_policy":{
    "max_attempts":5,"backoff_strategy":"exponential",
    "base_delay_seconds":5,"max_delay_seconds":300,"jitter":true,
    "retryable_errors":["connection_timeout","rate_limit_exceeded","temporary_failure"],
    "non_retryable_errors":["authentication_failed","schema_mismatch","data_validation_failed"]
  }
}
```

### 6.3 Circuit breaker

```json
{"circuit_breaker":{"failure_threshold":5,"success_threshold":2,
                     "timeout_seconds":60,"half_open_max_calls":3,"state":"closed"}}
```

State transitions: `closed → open` after N failures · `open → half_open` after timeout · `half_open → closed` after M successes · `half_open → open` on any failure.

---

## 7. Quality protocol

### 7.1 Quality gate

```json
{
  "quality_gate":{
    "gate_id":"qg_orders",
    "checks":[
      {"metric":"null_rate","column":"user_id","threshold":0.01,"severity":"blocker"},
      {"metric":"duplicate_rate","columns":["order_id"],"threshold":0.001,"severity":"critical"},
      {"metric":"freshness_lag_seconds","column":"created_at","threshold":86400,"severity":"warning"}
    ],
    "action_on_failure":"quarantine"
  }
}
```

### 7.2 Quarantine

```json
{
  "quarantine":{
    "bad_records_location":"s3://lake/quarantine/orders/dt=2026-04-25/",
    "error_log_location":"s3://logs/errors/orders/dt=2026-04-25/",
    "retention_days":30,
    "notification":{"channel":"slack","webhook":"https://hooks.slack.com/services/..."}
  }
}
```

---

## 8. Observability protocol

### 8.1 Metrics push (Prometheus + OTLP)

Pipelines push every 60 s using OpenTelemetry OTLP (gRPC):

```json
{
  "pipeline_id":"pip_daily_sales","execution_id":"exec_5b9a",
  "timestamp":"2026-04-25T02:05:00Z",
  "counters":{"records_processed":127500,"records_failed":50,"bytes_processed":104857600},
  "gauges":{"pipeline_lag_seconds":120,"memory_usage_mb":2048,"cpu_usage_percent":75},
  "histograms":{"processing_duration_ms":[45,52,48,61,55]}
}
```

### 8.2 Structured logs

```json
{"timestamp":"2026-04-25T02:05:00.123Z","level":"INFO",
 "pipeline_id":"pip_daily_sales","execution_id":"exec_5b9a","task_id":"extract_orders",
 "message":"Extracted 10000 records",
 "context":{"source":"postgresql","table":"orders","duration_ms":245},
 "trace_id":"00f067aa0ba902b7","span_id":"00f067aa0ba902b7"}
```

### 8.3 OpenLineage event stream

Every START/COMPLETE/FAIL/ABORT is published to `wia.data-pipeline.lineage.events` Kafka topic for downstream Marquez/DataHub.

---

## 9. Schema registry protocol

```json
{
  "schema_registration":{
    "subject":"orders-value","version":3,"schema_type":"AVRO",
    "schema":"{...avro schema...}",
    "compatibility":"BACKWARD_TRANSITIVE",
    "metadata":{"owner":"data-platform","data_contract":"orders-clean.v1"}
  }
}
```

Compatibility modes: `BACKWARD` (read old with new), `FORWARD` (read new with old), `FULL` (both), `NONE`. Add `_TRANSITIVE` to apply the rule across all prior versions, not only the previous one.

---

## 10. Security protocol

| Layer                | Required                                                                   |
|----------------------|----------------------------------------------------------------------------|
| Transport            | TLS 1.3 with TLS_AES_256_GCM_SHA384 or TLS_CHACHA20_POLY1305_SHA256         |
| Authentication       | OAuth 2.1 + PKCE + DPoP (RFC 9449); mTLS for service identities           |
| Application signing  | Ed25519 (RFC 8032) over RFC 8785 canonical JSON                            |
| Encryption at rest   | AES-256-GCM; keys rotated every 90 days                                    |
| Audit log            | Append-only, signed daily root, 7-year retention                           |

```json
{"audit":{"action":"pipeline_run","actor":"user@company.com",
          "resource":"pip_daily_sales","timestamp":"2026-04-25T02:00:00Z",
          "ip":"10.0.0.7","user_agent":"WIA-SDK/1.1","result":"success"}}
```

---

## 11. Failure modes the protocol layer must handle

- **Source unavailable** — circuit-break, alert, retain offset.
- **Late events** — process with grace window (default 6 h); record on `cache/...` topic prefix.
- **Clock drift** — NTP every 6 h; reject events with `|now − ts| > 600 s`.
- **Schema break** — reject with `409 Conflict` problem+json type `https://errors.wia-data-pipeline.org/schema-break`.
- **Backpressure** — sink advertises lag; producer slows on `429 Too Many Requests` + `Retry-After`.
- **Zombie task** — orchestrator detects 90 s heartbeat silence, fences the worker, retries.

---

**弘益人間 — Benefit All Humanity**
*WIA — World Industry Association · © 2026 MIT License*
