# WIA-DATA-004: PHASE 2 — API

**Version:** 1.1.0
**Status:** ✅ Deep-Published
**Last Updated:** 2026-04-25
**Conforms to:** OpenAPI 3.1.0, AsyncAPI 3.0.0, RFC 9457 problem+json, RFC 9421 HTTP Message Signatures, RFC 9449 DPoP, draft-ietf-httpapi-idempotency-key, OAuth 2.1 (RFC 9700 BCP), Iceberg REST Catalog 1.5, OpenLineage 1.21.

---

## 1. Scope and conformance

Phase 2 defines two equally normative façades:

1. **REST + OpenAPI 3.1** — synchronous CRUD over JSON resources for pipelines, runs, datasets, lineage, quality, metrics.
2. **AsyncAPI 3.0** — event-driven contract for stream ingestion, CDC, run events, lineage events.

A `WIA-DATA-004:2026 Gold` implementation passes the suite at <code>https://conformance.wia-data-pipeline.org</code> against both façades; result hashes MUST match.

### 1.1 Base URLs and content negotiation

```
Production : https://api.wia-data-pipeline.org/v1
Sandbox    : https://staging-api.wia-data-pipeline.org/v1
Catalogue  : https://api.wia-data-pipeline.org/iceberg/v1   (Iceberg REST 1.5)
Async      : amqps://broker.wia-data-pipeline.org / kafka://kafka.wia-data-pipeline.org:9093
```

OpenAPI: `application/vnd.oai.openapi+json;version=3.1`. Async: `application/cloudevents+json` (CloudEvents 1.0).

### 1.2 Authentication

| Method                         | RFC          | Use case                                   |
|--------------------------------|--------------|--------------------------------------------|
| OAuth 2.1 + PKCE + DPoP        | RFC 9700, 9449 | Operator portal, IDE plugin              |
| mTLS X.509                     | RFC 8446     | Pipeline runner, self-hosted Airflow worker |
| HTTP Message Signature         | RFC 9421     | Cross-tenant lineage push, partner ingest   |

```http
Authorization: DPoP eyJhbGciOiJFZERTQSIs...
DPoP: eyJ0eXAiOiJkcG9wK2p3dCIsImFsZyI6IkVkRFNBIiwianRpIjoi...
```

---

## 2. OpenAPI 3.1 contract (excerpt)

```yaml
openapi: 3.1.0
info: { title: WIA Data Pipeline API, version: 1.1.0 }
servers: [{ url: https://api.wia-data-pipeline.org/v1 }]
components:
  securitySchemes:
    bearerAuth: { type: http, scheme: bearer, bearerFormat: JWT }
  schemas:
    Pipeline:
      $id: https://schemas.wia-data-pipeline.org/Pipeline.json
      type: object
      required: [id, name, version, owner, source, destination]
      properties:
        id:      { type: string, pattern: '^pip_[a-z0-9_-]{4,32}$' }
        version: { type: string, pattern: '^\d+\.\d+\.\d+$' }
        owner:   { type: string, format: email }
        schedule:{ type: string }
        source:  { $ref: '#/components/schemas/Source' }
        destination: { $ref: '#/components/schemas/Destination' }
paths:
  /pipelines:
    post:
      operationId: createPipeline
      security: [{ bearerAuth: [] }]
      parameters:
        - { in: header, name: Idempotency-Key, schema: { type: string }, required: true }
      requestBody:
        required: true
        content: { application/json: { schema: { $ref: '#/components/schemas/Pipeline' } } }
      responses:
        '201':
          headers:
            Location: { schema: { type: string, format: uri } }
            ETag:     { schema: { type: string } }
          content: { application/json: { schema: { $ref: '#/components/schemas/Pipeline' } } }
        '409': { description: Conflict }
        '422': { description: Validation failure (problem+json) }
```

---

## 3. Pipeline management

### 3.1 Create pipeline

```bash
curl -X POST https://api.wia-data-pipeline.org/v1/pipelines \
  -H "Authorization: Bearer $TOKEN" \
  -H "Content-Type: application/json" \
  -H "Idempotency-Key: 4f1d3c0e-7b2c-4a98-9c7e-2c0e9b8a4a01" \
  -d '{
    "id":"pip_daily_sales","name":"Daily Sales","version":"1.0.0",
    "owner":"data-platform@company.com",
    "schedule":"0 2 * * *",
    "source":{"type":"postgresql","connection_id":"prod-db",
              "extraction_mode":"cdc","cdc":{"mechanism":"debezium"}},
    "destination":{"type":"iceberg","catalog":"polaris-prod","namespace":"lake","table":"orders","mode":"merge","merge_keys":["order_id"]}
  }'
```

Response includes `Location`, strong `ETag`, and the canonical resource representation. `Idempotency-Key` (draft-ietf-httpapi-idempotency-key) makes retries safe; the server caches the response for 24 h.

### 3.2 Patch pipeline (concurrency-safe)

```bash
curl -X PATCH https://api.wia-data-pipeline.org/v1/pipelines/pip_daily_sales \
  -H "Authorization: Bearer $TOKEN" \
  -H "Content-Type: application/merge-patch+json" \
  -H "If-Match: \"v3:f3a2e1c\"" \
  -d '{"schedule":"0 1 * * *"}'
```

RFC 7396 merge-patch + If-Match (RFC 9110 §13.1.1) prevents lost-update.

### 3.3 Trigger run

```http
POST /v1/pipelines/pip_daily_sales/runs
Idempotency-Key: run_2026-04-25_02

{
  "trigger_type":"manual",
  "parameters":{"start_date":"2026-04-25","end_date":"2026-04-25"}
}
```

### 3.4 Run status (with conditional read)

```http
GET /v1/pipelines/pip_daily_sales/runs/exec_5b9a
If-None-Match: "v3:1cd0..."
```

Returns `200 OK` with full body or `304 Not Modified`.

---

## 4. Errors — RFC 9457 problem+json

```http
HTTP/1.1 422 Unprocessable Entity
Content-Type: application/problem+json

{
  "type":"https://errors.wia-data-pipeline.org/quality-gate-failed",
  "title":"Quality gate blocker tripped",
  "status":422,
  "detail":"orders.user_id null_rate 1.4% &gt; threshold 1.0%",
  "instance":"/v1/pipelines/pip_daily_sales/runs/exec_5b9a",
  "violations":[
    {"check":"null_rate","column":"user_id","value":0.014,"threshold":0.01,"severity":"blocker"}
  ]
}
```

---

## 5. Data ingestion

### 5.1 Stream ingest

```bash
curl -X POST https://api.wia-data-pipeline.org/v1/ingest/stream/user-events \
  -H "Authorization: Bearer $TOKEN" \
  -H "Content-Type: application/cloudevents-batch+json" \
  -d '[{
    "specversion":"1.0","id":"evt_001","source":"app://checkout",
    "type":"com.wia.events.OrderPlaced","time":"2026-04-25T10:00:00Z",
    "datacontenttype":"application/json",
    "data":{"order_id":12345,"user_id":789,"amount":99.99}
  }]'
```

The server appends to a Kafka topic with idempotent producer + transactional commit (Phase 3 §3).

### 5.2 Batch ingest

```http
POST /v1/ingest/batch
Content-Type: multipart/form-data
Idempotency-Key: batch-orders-2026-04-25

[ file part: orders_2026-04-25.parquet ]
```

Response includes a `file_id` and an Iceberg `snapshot_id` once committed.

---

## 6. Iceberg REST Catalog 1.5

The standard mounts an Iceberg REST endpoint so any Iceberg-aware engine (Trino, Spark, DuckDB, Snowflake-with-Polaris) can transparently query lakehouse tables.

```bash
curl -G https://api.wia-data-pipeline.org/iceberg/v1/namespaces/lake/tables \
  -H "Authorization: Bearer $TOKEN"

curl -X POST https://api.wia-data-pipeline.org/iceberg/v1/namespaces/lake/tables/orders/snapshots \
  -H "Content-Type: application/json" \
  -d '{"target-snapshot-id":7841029384712340987}'
```

---

## 7. Data quality API

### 7.1 Run a check (Soda Core / Great Expectations / dbt-tests)

```bash
curl -X POST https://api.wia-data-pipeline.org/v1/quality/checks \
  -H "Content-Type: application/json" \
  -d '{
    "dataset_id":"lake.orders","engine":"soda",
    "checks":[
      {"type":"missing_count","column":"user_id","threshold":0},
      {"type":"duplicate_count","columns":["order_id"],"threshold":0},
      {"type":"row_count","min":1000,"max":1000000000},
      {"type":"freshness","column":"created_at","max_lag":"P1D"}
    ]
  }'
```

### 7.2 Quality gate decision

```http
GET /v1/quality/gates/qg_orders/decision?snapshot_id=7841029384712340987
&rarr; { "decision":"PASS","blockers":0,"warnings":1, "report":"..." }
```

A `quality.gate.failed` CloudEvent is published when a blocker trips.

---

## 8. Lineage and catalogue (OpenLineage 1.21)

### 8.1 Lineage event ingest

```http
POST /v1/lineage/events
Content-Type: application/json

{
  "eventType":"COMPLETE","eventTime":"2026-04-25T02:14:03Z",
  "producer":"airflow:2.10",
  "run":{"runId":"5b9a..."},
  "job":{"namespace":"pip.daily_sales","name":"transform_orders"},
  "inputs":[{"namespace":"postgres://prod-db/app","name":"public.orders"}],
  "outputs":[{"namespace":"iceberg://polaris-prod","name":"lake.orders"}]
}
```

### 8.2 Walk lineage

```http
GET /v1/lineage/datasets/lake.orders?direction=upstream&depth=3
```

Returns a graph with run, job, dataset nodes, and column-level edges where present.

### 8.3 Catalogue

```http
POST /v1/catalog/datasets
{
  "name":"lake.orders","description":"Cleaned orders feed",
  "schema":{"columns":[
    {"name":"order_id","type":"int64","nullable":false},
    {"name":"user_id","type":"int64","nullable":false},
    {"name":"total_cents","type":"int64","nullable":false},
    {"name":"created_at","type":"timestamp(us,UTC)","nullable":false}]},
  "format":"iceberg","partition_columns":["days(created_at)","bucket(16,user_id)"],
  "owner":"data-platform@company.com",
  "data_contract":"orders-clean.v1"
}
```

---

## 9. Monitoring API

```http
GET /v1/metrics/pipelines/pip_daily_sales?from=2026-04-25T00:00:00Z&to=2026-04-25T23:59:59Z

&rarr; {
  "executions_total":24,"executions_success":23,"executions_failed":1,
  "p95_duration_seconds":945,"records_total":3060000,"bytes_total":98423124100,
  "freshness_seconds_max":86340
}
```

Prometheus exposition format also offered at `GET /v1/metrics/prometheus`.

---

## 10. Webhooks (CloudEvents 1.0 + RFC 9421)

```http
POST /webhooks/your-endpoint
Content-Type: application/cloudevents+json
Signature-Input: sig1=("@method" "@target-uri" "content-digest");
                 created=1714032000;keyid="wia-data-pipeline";alg="ed25519"

{"specversion":"1.0","id":"evt-7b9","source":"https://api.wia-data-pipeline.org/v1",
 "type":"org.wia.data-pipeline.run.failed","subject":"pip_daily_sales/exec_5b9a",
 "time":"2026-04-25T02:14:03Z","datacontenttype":"application/json",
 "data":{"failure":"quality-gate","detail":"user_id null_rate 1.4%"}}
```

---

## 11. AsyncAPI 3.0 stream contract

```yaml
asyncapi: 3.0.0
info: { title: WIA Data Pipeline events, version: 1.1.0 }
servers:
  kafka:
    host: kafka.wia-data-pipeline.org:9093
    protocol: kafka-secure
channels:
  pipeline.run.events:
    address: wia.data-pipeline.run.events
    messages:
      runEvent:
        contentType: application/cloudevents+json
        payload:
          $ref: 'https://schemas.wia-data-pipeline.org/RunEvent.json'
operations:
  publishRunEvent:
    action: receive
    channel: { $ref: '#/channels/pipeline.run.events' }
```

---

## 12. SDKs

### 12.1 Python

```python
from wia_data_pipeline import Client
c = Client(token=os.environ["WIA_TOKEN"], dpop_key=load_jwk("dpop.jwk"))
pip = c.pipelines.get("pip_daily_sales")
run = pip.runs.create(parameters={"start_date":"2026-04-25"}, idempotency_key="run_2026-04-25_02")
run.wait_until_complete(timeout=3600)
print(run.status, run.metrics.records_loaded, run.openlineage_run_id)
```

### 12.2 TypeScript

```ts
import { Client } from "@wia/data-pipeline";
const c = new Client({ token: process.env.WIA_TOKEN! });
const run = await c.pipelines.runs.create("pip_daily_sales",
  { parameters: { start_date: "2026-04-25" }, idempotencyKey: "run_2026-04-25_02" });
await run.waitUntilComplete();
```

---

## 13. Rate limits and SLOs

| Tier        | Sustained TPS | Stream ingest events/s | P95 read | P95 write |
|-------------|--------------:|-----------------------:|---------:|----------:|
| Standard    |           100 |                  5 000 |    80 ms |    250 ms |
| Premium     |         1 000 |                 50 000 |    50 ms |    180 ms |
| Enterprise  |        10 000 |                500 000 |    30 ms |    120 ms |

`X-RateLimit-*` headers (draft-ietf-httpapi-ratelimit-headers) on every response. Error budget: 0.1 % monthly availability.

---

**弘益人間 — Benefit All Humanity**
*WIA — World Industry Association · © 2026 MIT License*
