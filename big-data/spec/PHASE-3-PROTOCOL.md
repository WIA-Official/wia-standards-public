# WIA-DATA-001: Phase 3 — Protocol Specification

**Version:** 1.1.0
**Status:** Active
**Last Updated:** 2026-04-25
**Philosophy:** 弘益人間 (Hongik Ingan) · Benefit All Humanity

---

## 1. Introduction

Phase 3 specifies how rows **move through** a WIA-DATA-001 platform: stream transport, exactly-once write semantics, pipeline configuration, change data capture (CDC), watermarks, checkpoints, savepoints, and backpressure. The philosophy is *strict on correctness, liberal on framework*: the same wire contracts serve Apache Kafka, Pulsar, Kinesis, Flink, Spark Structured Streaming, and Beam.

### 1.1 Normative references

- Apache Kafka Wire Protocol (AK 3.7+)
- Apache Pulsar 3.2 Binary Protocol
- AWS Kinesis Data Streams API v2
- Apache Flink 1.19 — Chandy–Lamport checkpointing
- Spark 3.5 — Structured Streaming, idempotent sinks
- Apache Iceberg 1.5 — Snapshot commit protocol
- Debezium 2.6 — Envelope schema (`io.debezium.data.Envelope`)
- Confluent Schema Registry REST v1 (compatibility subject)
- OpenTelemetry 1.30 — span semantic conventions for messaging

### 1.2 Keywords

MUST/SHOULD/MAY per RFC 2119.

---

## 2. Pipeline primitives

```
  Source ─► Decode ─► Validate ─► Transform ─► Enrich ─► Commit ─► Sink
                           │             │         │
                        schema         CEL/DSL   lookup
                       registry
```

### 2.1 Pipeline document

```yaml
pipeline:
  id: user_events_v3
  parallelism: 16
  source:
    type: kafka
    bootstrap: ["kafka-1:9092","kafka-2:9092"]
    topic: raw.events
    group: user_events_v3
    isolation_level: read_committed          # exactly-once
  codec:
    value: avro
    schema: "com.wia.events.EventRecord.v1"
    registry: https://registry.wia-data.com/v1
  transforms:
    - { type: filter,  where: "event_type IN ('click','view')" }
    - { type: map,     expr:  "record | merge({received_at: now()})" }
    - { type: enrich,  lookup: "users_by_id",
                       key: "user_id",
                       select: ["country","segment"] }
  sink:
    type: iceberg
    catalog: nessie
    table: warehouse.events_processed
    commit: exactly_once
    target_file_bytes: 134217728
    compression: zstd
  state:
    backend: rocksdb
    checkpoint_interval: 30s
    checkpoint_dir: s3://wia-state/user_events_v3
  observability:
    lineage: openlineage
    otel_endpoint: https://otel.wia-data.com
```

### 2.2 Execution semantics

- **Ordering**: per source partition.
- **Delivery**: `at_most_once` | `at_least_once` | `exactly_once` (default).
- **Parallelism**: `source_parallelism ≤ number_of_partitions`.
- **Back-pressure**: reactive-streams credit-based flow; every operator MUST expose `wia_data_buffer_fill_ratio` ≥ 0.9 as an amber signal and ≥ 0.98 red.

---

## 3. Kafka / Pulsar / Kinesis wire parity

WIA engines MUST speak native wire protocols of the chosen broker. Feature parity across backends:

| Feature                     | Kafka             | Pulsar            | Kinesis           |
|-----------------------------|-------------------|-------------------|-------------------|
| Exactly-once producer       | idempotent + TX   | effectively-once  | idempotent keys   |
| Consumer offset store       | `__consumer_offsets` | Cursor, Broker | DynamoDB (KCL)    |
| Transactional writes        | Yes (isolation)   | Yes (PIP-31)      | KPL + KCL checkpoint |
| Schema enforcement          | Schema Registry   | Built-in          | Glue / external   |
| Compaction                  | Log compaction    | Compaction topics | N/A (retention)   |
| Max message size (default)  | 1 MiB             | 5 MiB             | 1 MiB (1000 ops/s)|

WIA records published to Kafka MUST use `CreateTime` timestamps matching the record's `event_ts` field and include these headers:

```
content-type:      application/avro
schema-id:         17293     (Confluent-compatible big-endian int32)
wia-dataset:       ds-12345
wia-idempotency-key: <uuidv7>
wia-tenant:        acme
```

Pulsar equivalent uses message *properties*; Kinesis uses record *partition-key* + per-record metadata.

---

## 4. Message envelope

All records on the wire (regardless of broker) use the binary form:

```
┌──────────┬──────────┬──────────────┬──────────────────────────┐
│ magic=1B │ schema_id │ key (Avro/JSON) │ value (Avro/Parquet cell) │
└──────────┴──────────┴──────────────┴──────────────────────────┘
```

This matches the Confluent wire format so existing Kafka Connect / Debezium / ksqlDB deployments interoperate. A `schema_id = 0` indicates "use the broker's topic-default subject" (aligns with Confluent TopicNameStrategy).

JSON envelope (control plane, alerts, dev tools):

```json
{
  "header": {
    "message_id":  "01JCG0T0A2V5M8W7P3Q9S1R6N4",
    "wia_dataset": "ds-12345",
    "schema":      "com.wia.events.EventRecord.v1",
    "event_ts":    1714046400000,
    "ingest_ts":   1714046400152,
    "source":      "gateway-us-east-1",
    "trace_id":    "0a1b2c3d4e5f60718293a4b5c6d7e8f9"
  },
  "payload": { "event_id": "e-1", "user_id": 42,
               "event_type": "click" },
  "metadata": { "priority": "normal", "ttl_seconds": 3600 }
}
```

---

## 5. Delivery guarantees

### 5.1 At-most-once

Producer: fire-and-forget; consumer: auto-commits before processing. Use only for metrics/logs where loss is acceptable.

### 5.2 At-least-once (default for Tier 3)

Producer: acks=all + retries; consumer: commits after processing. Deduplication MUST be implemented at the sink using the composite `(wia_dataset, idempotency_key)` and a bloom filter or storage-level primary key.

### 5.3 Exactly-once (required for Tier 1)

Exactly-once requires **three** cooperating properties:

1. **Idempotent producer** — same `idempotency_key` yields a single append no matter how many retries occur.
2. **Transactional commit** — Kafka `initTransactions()`/`commitTransaction()` or Pulsar `Transaction.newBuilder().build()`, with `isolation.level=read_committed` on downstream consumers.
3. **Two-phase commit with sink** — sinks participate in `preCommit` → `commit` → `abort` semantics. Iceberg's snapshot-commit protocol is the reference sink (§ 8); S3-as-sink uses `PutObject` with conditional `If-None-Match: *` plus a manifest registered in a transactional catalog.

If any of the three is absent, the pipeline MUST degrade its declared guarantee to at-least-once in the discovery document so downstream auditors can see it.

### 5.4 Causal ordering across partitions

When multi-partition ordering matters (e.g., "all events for user 42 in order"), WIA engines MUST route by a stable hash of the **business key**, never round-robin. The protocol header `wia-partition-key` is reserved for downstream operators that need to preserve the hash.

---

## 6. Change Data Capture (CDC)

### 6.1 Envelope

The CDC envelope is byte-compatible with Debezium's `io.debezium.data.Envelope`, extended with a `wia` block:

```json
{
  "op": "u",
  "ts_ms": 1714046400012,
  "before": { "id": 42, "balance": 100.00 },
  "after":  { "id": 42, "balance":  90.00 },
  "source": { "connector": "postgres", "name": "pg1",
              "txId": 9_482_173, "lsn": "0/1A2B3C4D",
              "table": "accounts", "db": "banking" },
  "wia":    { "dataset": "ds-accounts",
              "snapshot_id": "7219743893013213000",
              "sequence": 178_429_017 }
}
```

Operations: `c` (create), `u` (update), `d` (delete), `r` (read/snapshot), `t` (truncate).

### 6.2 Log-based CDC sources

| Source             | Mechanism                      |
|--------------------|--------------------------------|
| PostgreSQL ≥ 13    | logical replication (pgoutput) |
| MySQL ≥ 8          | binlog row-based (GTID)        |
| Oracle             | LogMiner / XStream             |
| SQL Server         | CDC tables + sys.fn_cdc_…      |
| MongoDB ≥ 5        | change streams                 |
| Cassandra ≥ 4      | CDC commit-log agents          |

Sources MUST provide **exactly-once** by combining WAL/binlog position with transactional snapshot keys on the catalog. Consumers MUST be able to rewind to any prior snapshot that the source retains.

### 6.3 Initial snapshot + incremental

Connectors emit `r` events for the initial snapshot, watermarks bracket the snapshot with low/high LSNs, and the pipeline switches to live `c/u/d` stream when the snapshot high-watermark is observed. This is the contract most BI tools and lakehouse loaders rely on; WIA sinks MUST respect it.

---

## 7. Stream processing primitives

### 7.1 Event-time semantics

All operators MUST use event-time (the record's `event_ts`) by default. Processing-time is allowed only for ops that explicitly document it. Reordering within `allowed_lateness` is buffered; records beyond it flow to `late_output`.

### 7.2 Watermarks

```json
{ "watermark": {
    "strategy":   "bounded_out_of_orderness",
    "max_delay":  "PT1M",
    "idle_ms":    60000
} }
```

`bounded_out_of_orderness` is mandatory for Tier 1; `ascending` is only valid when the source provably delivers monotonic timestamps (e.g., single-partition Kinesis with `putRecord` in order).

### 7.3 Windows

```json
{ "window": {"type": "tumbling", "size": "PT5M"} }
{ "window": {"type": "sliding",  "size": "PT1H", "slide": "PT15M"} }
{ "window": {"type": "session",  "gap":  "PT30M"} }
{ "window": {"type": "global",   "trigger": "count:1000"} }
```

Session windows MUST emit a late firing when additional records arrive inside `allowed_lateness` after the initial close, producing an update event labelled `wia.window.emission = "update"`.

### 7.4 State and joins

- Keyed state backed by RocksDB (bounded) or in-memory (fail-fast).
- **Interval joins**: `L.event_ts BETWEEN R.event_ts - 5m AND R.event_ts + 5m`.
- **Temporal joins**: use the right-side version valid at the left record's `event_ts` (Flink FOR SYSTEM_TIME AS OF).

### 7.5 Triggers

Triggers close a window early (burst detection) or emit updates (incremental aggregates). Each trigger firing MUST be annotated with `wia.trigger = "early" | "on-time" | "late"`.

---

## 8. Commit protocol with the lakehouse

Iceberg is the reference sink; Delta and Hudi follow the same semantics with their own commit atoms.

### 8.1 Two-phase commit

```
pre-commit  ──▶  write data files, produce manifest draft
     │
     ▼
commit      ──▶  append to table metadata (CAS on current snapshot)
     │
     └──▶  on CAS failure: validate no conflict, re-commit;
           on unrecoverable failure: orphan files are GC'd by
           wia_data retain.expire-snapshots job.
```

### 8.2 Snapshot-id propagation

Each committed batch stamps the snapshot id on every sink record it produced: `X-WIA-Snapshot-Id` header (for REST sinks), `wia.snapshot_id` metadata (Avro), or Iceberg manifest entry. Downstream pipelines MAY subscribe to `/streams/{id}?since=snapshot:…` to replay exactly from a known point.

### 8.3 File compaction and retention

- Rewrite files when: **count > 50 in a partition** OR **smallest file < 16 MiB**.
- Expire snapshots older than 7 days (production) / 30 days (research).
- `rewrite_data_files(target_file_size_bytes=128 MiB)`; compression `zstd:3`.

---

## 9. Transport and encryption

- **TLS 1.3** on every broker hop.
- **mTLS** for producer ↔ broker ↔ consumer on Tier 1 pipelines.
- **Payload encryption**: field-level envelope encryption (AEAD `AES-256-GCM`) for PII columns, with key material from KMS (AWS/GCP/HSM). Ciphertext stored; decryption keyed by dataset ACL.
- **Kerberos/SASL-SCRAM**: permitted for legacy enterprise brokers, but MUST sit inside TLS.

---

## 10. Checkpoints, savepoints, recovery

### 10.1 Checkpoint format

```json
{
  "job_id":        "job-7f1c",
  "checkpoint_id": "cp-20260425-1400-0001",
  "wall_clock_ms": 1714046400000,
  "event_time_lo": 1714046300000,
  "event_time_hi": 1714046400000,
  "offsets": {
    "kafka:raw.events:0": 18_429_117,
    "kafka:raw.events:1": 18_430_044
  },
  "state": {
    "backend":  "rocksdb",
    "manifest": "s3://wia-state/user_events_v3/cp-20260425-1400-0001/MANIFEST",
    "bytes":    52_428_800
  },
  "sha256": "b4e4b2…"
}
```

- **Interval**: ≥ 30 s for Tier 1, ≥ 5 min for Tier 3.
- **Storage**: object store (S3/GCS/Azure) with versioning enabled.
- **Atomicity**: Chandy–Lamport barrier injected per source partition; operator takes state snapshot when barrier received on all inputs.

### 10.2 Savepoints

Manual, non-periodic checkpoints for version upgrades, topology changes, and cluster migrations. Savepoints MUST be self-describing (`job-schema-hash`, `wia-version`, `operator-ids`) so a new deployment of the same logical job can resume.

### 10.3 Recovery policy

On crash:
1. Find latest *successful* checkpoint by its manifest `.sha256`.
2. Reset source offsets to checkpointed values.
3. Restore operator state.
4. Replay events from offsets; deliver with exactly-once sink semantics.

Stuck operators (> 3× checkpoint interval without progress) MUST be killed and retried under the supervisor's back-off schedule (`base=1s`, `cap=5m`, jitter 20 %).

---

## 11. Monitoring and observability

### 11.1 Required metrics

```
wia_data_pipeline_records_in_total{pipeline="…",partition="0"}
wia_data_pipeline_records_out_total{pipeline="…"}
wia_data_pipeline_records_dropped_total{pipeline="…",reason="late|filter|error"}
wia_data_pipeline_bytes_in_total
wia_data_pipeline_latency_ms{quantile="0.5|0.95|0.99"}
wia_data_pipeline_watermark_skew_ms
wia_data_pipeline_buffer_fill_ratio
wia_data_pipeline_checkpoint_duration_ms
wia_data_pipeline_checkpoint_last_success_ts
```

### 11.2 OpenTelemetry spans

Each record's lifecycle is traced via W3C Trace Context. Spans bear attributes `wia.dataset`, `wia.pipeline`, `wia.operator`, `wia.partition`, `wia.idempotency_key`. Sampling SHOULD be consistent (parent-based) so a downstream sink trace is still attachable.

### 11.3 Lineage (OpenLineage 1.1)

Every job emits OpenLineage `RunEvent`s on `START` and `COMPLETE` with input/output datasets, facet `wia:snapshot` on lakehouse tables, and facet `wia:sensitive_columns` for any column protected by CLS.

---

## 12. Worked scenarios

### 12.1 Flink pipeline consuming Kafka → Iceberg with exactly-once

```java
StreamExecutionEnvironment env =
    StreamExecutionEnvironment.getExecutionEnvironment();
env.enableCheckpointing(30_000,
    CheckpointingMode.EXACTLY_ONCE);

KafkaSource<Event> src = KafkaSource.<Event>builder()
    .setBootstrapServers("kafka-1:9092,kafka-2:9092")
    .setTopics("raw.events")
    .setGroupId("user_events_v3")
    .setStartingOffsets(OffsetsInitializer.committedOffsets())
    .setProperty("isolation.level", "read_committed")
    .setValueOnlyDeserializer(new WIAAvroDeserializer("com.wia.events.EventRecord.v1"))
    .build();

env.fromSource(src, WatermarkStrategy
        .<Event>forBoundedOutOfOrderness(Duration.ofMinutes(1))
        .withTimestampAssigner((e, t) -> e.getEventTs()),
      "kafka-events")
   .filter(e -> e.getType() == EventType.CLICK || e.getType() == EventType.VIEW)
   .keyBy(Event::getUserId)
   .window(TumblingEventTimeWindows.of(Time.minutes(5)))
   .aggregate(new ClickCountAgg())
   .sinkTo(FlinkSink.forRowData(icebergTable)
         .tableLoader(loader)
         .equalityFieldColumns(List.of("user_id","window_start"))
         .writeParallelism(16)
         .build());

env.execute("user_events_v3");
```

### 12.2 Rewinding a consumer to a snapshot

```
GET /streams/cdc-accounts?since=snapshot:7219743893013213000
```

Server streams all change events whose `wia.snapshot_id ≥ 7219743893013213000`, then switches to live. This is how hot-standby consumers recover after a disaster without tolerating any gap or duplicate.

### 12.3 Schema evolution mid-stream

A producer upgrades `EventRecord` from `v1` to `v2` (adds optional `device_id`). The schema registry enforces `BACKWARD` compatibility, brokers publish records with the new `schema_id`, and the Flink job auto-detects the change via the deserializer which tolerates added optional fields. No restart required; old consumers keep reading `v1` subsets of the payload.

---

## 13. Compliance checklist

- [ ] Broker supports exactly-once (Kafka TX / Pulsar TX / Kinesis KCL + idempotency).
- [ ] Messages carry `content-type`, `schema-id`, `wia-dataset`, idempotency headers.
- [ ] Schema registry enforces compatibility mode of the dataset.
- [ ] CDC sources use log-based mechanisms with Debezium-compatible envelope.
- [ ] Watermark strategy declared; late data routed, never silently dropped.
- [ ] Two-phase commit to the lakehouse sink is verifiable.
- [ ] Checkpoints at ≥ one every 30 s; savepoints exercised before upgrades.
- [ ] OTel traces + OpenLineage runs emitted.
- [ ] Backpressure metrics exposed; supervisor auto-restarts stalled operators.

---

## 14. References

1. Apache Kafka Wire Protocol (AK 3.7)
2. Apache Pulsar 3.2 Binary Protocol; PIP-31 transactions
3. AWS Kinesis Data Streams API v2 and KCL
4. Apache Flink 1.19 — Chandy–Lamport snapshots
5. Apache Spark 3.5 — Structured Streaming sinks
6. Apache Iceberg 1.5 — Snapshot commit protocol
7. Debezium 2.6 — `io.debezium.data.Envelope` reference
8. Confluent Schema Registry REST API
9. OpenLineage 1.1; OpenTelemetry 1.30 messaging semantic conventions

---

**© 2026 WIA — World Certification Industry Association**
*弘益人間 · Benefit All Humanity*
