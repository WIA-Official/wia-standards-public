# WIA-DATA-001: Phase 2 — API Interface Specification

**Version:** 1.1.0
**Status:** Active
**Last Updated:** 2026-04-25
**Philosophy:** 弘益人間 (Hongik Ingan) · Benefit All Humanity

---

## 1. Introduction

Phase 2 defines every interface through which a consumer **writes, reads, and queries** data held in a WIA-DATA-001 conformant platform. Four surfaces are specified:

1. **REST/HTTP** — CRUD, bulk ingest, query submission (§ 2–4).
2. **GraphQL** — federation-friendly, for dashboards and apps (§ 5).
3. **Arrow Flight SQL / ADBC** — columnar, zero-copy analytics transport (§ 6).
4. **Client SDKs** — Python, Java/JVM, Rust, TypeScript (§ 7).

Together they sit on top of the Phase 1 schema registry and WBF/Parquet/Avro storage layer; Phase 3 then defines how records flow through streams and Phase 4 how we federate with external systems.

### 1.1 Normative references

- RFC 9110 / 9111 — HTTP semantics and caching
- RFC 9457 — Problem Details for HTTP APIs
- RFC 6902 — JSON Patch
- RFC 8594 — The Sunset HTTP Header Field
- OpenAPI Specification 3.1.0
- GraphQL (June 2018, with October 2021 addenda)
- Apache Arrow 15 — Columnar Format and Flight SQL
- Arrow Database Connectivity (ADBC) 1.1
- Prometheus Exposition Format 0.0.4 (for `/metrics`)

### 1.2 Conformance keywords

MUST / SHOULD / MAY per RFC 2119. All requirements apply to Tier 1/2 deployments; Tier 3/4 MAY relax SLOs but MUST preserve wire semantics.

---

## 2. Base URL, versioning, discovery

```
https://api.wia-data.com/v1/
```

A discovery document MUST be served at `/.well-known/wia-data`:

```json
{
  "standard":      "WIA-DATA-001",
  "version":       "1.1",
  "base_url":      "https://api.wia-data.com/v1",
  "openapi":       "https://api.wia-data.com/v1/openapi.json",
  "graphql":       "https://api.wia-data.com/v1/graphql",
  "flight_sql":    "grpc+tls://flight.wia-data.com:443",
  "schema_registry":"https://registry.wia-data.com/v1",
  "capabilities":  ["bulk.append", "query.sql", "query.wsql", "cdc.subscribe",
                    "time-travel", "row-level-security", "arrow.flight"]
}
```

Breaking changes bump the URL major (`/v2/`). Deprecations MUST use `Sunset` and `Deprecation` headers (RFC 8594) with at least 180 days' notice.

---

## 3. Resource model

```
/datasets                     collection (metadata only)
/datasets/{id}                document
/datasets/{id}/data           row gateway (append, patch, delete)
/datasets/{id}/query          synchronous query
/datasets/{id}/queries        async query submission
/datasets/{id}/schema         Phase 1 schema history + pins
/datasets/{id}/snapshots      time-travel index (Iceberg-aligned)
/datasets/{id}/acl            row/column access policy
/jobs/{id}                    long-running job status
/streams/{id}                 durable CDC subscription
/schema/{namespace}/{name}    global schema-registry object
```

### 3.1 Pagination

Collections larger than 100 items MUST paginate with **cursor** form (offset is supported up to 10 000 for legacy):

```
GET /datasets/ds-12345/data?limit=1000&cursor=eyJyb3ciOjEwMDAwMDB9
```

```json
{
  "data": [ … ],
  "paging": {
    "next":    "https://…?cursor=eyJyb3ciOjIwMDAwMDB9&limit=1000",
    "prev":    null,
    "has_more": true,
    "returned": 1000
  }
}
```

### 3.2 Sparse fieldsets and projection

```
?fields=event_id,timestamp,user_id
```

Nested selections use `/`-joined paths: `?fields=user/id,user/email`.

### 3.3 Filtering grammar (WSQL-subset)

```
?filter=event_type=click AND user_id IN (1,2,3) AND timestamp >= "2026-04-01"
```

Allowed operators: `= != < <= > >= IN NOT_IN BETWEEN LIKE IS_NULL IS_NOT_NULL AND OR NOT`. No function calls (use POST `/query` for those).

---

## 4. Query interfaces

### 4.1 Synchronous query (`POST /datasets/{id}/query`)

Accepts both WIA JSON query language (WSQL) and ANSI SQL. Selection is by `Content-Type`:

```http
POST /datasets/ds-12345/query HTTP/1.1
Authorization: Bearer $TOKEN
Content-Type: application/sql
Accept: application/vnd.apache.arrow.stream

SELECT user_id, COUNT(*) AS cnt
FROM events
WHERE event_type = 'click'
  AND event_ts BETWEEN '2026-04-01' AND '2026-04-25'
GROUP BY user_id
ORDER BY cnt DESC
LIMIT 100
```

Response bodies:

| `Accept`                                   | Shape                       |
|--------------------------------------------|-----------------------------|
| `application/json`                         | `{schema, rows}`            |
| `application/vnd.apache.arrow.stream`      | Arrow IPC stream            |
| `application/vnd.apache.arrow.file`        | Arrow IPC file              |
| `application/parquet`                      | Parquet snapshot            |
| `text/csv`                                 | CSV, UTF-8, RFC 4180        |

Arrow is the canonical analytic wire format; JSON is provided for portability and debugging.

### 4.2 Asynchronous query (`POST /datasets/{id}/queries`)

Returns `202 Accepted` with `Location: /jobs/{id}`. Poll the job endpoint or subscribe via WebSocket (`/streams/{id}`). Jobs expose:

```json
{
  "id":        "job-7f1c",
  "state":    "running",
  "progress":  0.42,
  "submitted": "2026-04-25T14:30:00Z",
  "cost_rows": 125_000_000,
  "cost_bytes":  52_428_800,
  "result":    { "type": "arrow.stream",
                 "url":  "https://…/jobs/job-7f1c/result",
                 "expires": "2026-04-26T14:30:00Z" }
}
```

States: `queued`, `planning`, `running`, `succeeded`, `failed`, `cancelled`.

### 4.3 WSQL (WIA Structured Query Language)

WSQL is a JSON-native subset of SQL for clients that prefer structured predicates. All WSQL queries have a lossless SQL equivalent; servers MUST print the SQL translation into `X-WIA-SQL-Translated` for debug clients.

```json
{
  "select": ["user_id", {"fn": "COUNT", "args": ["*"], "as": "cnt"}],
  "from":   "events",
  "where":  { "and": [
                { "=": ["event_type", "click"] },
                { "between": ["event_ts", "2026-04-01", "2026-04-25"] }
            ] },
  "groupBy": ["user_id"],
  "orderBy": [{"cnt": "DESC"}],
  "limit":   100
}
```

### 4.4 Time-travel and snapshots

Every read MAY pin to a specific snapshot, aligning with Iceberg/Delta semantics:

```
?snapshot_id=7219743893013213000
?as_of_timestamp=2026-03-01T00:00:00Z
```

Servers MUST include `X-WIA-Snapshot-Id` in responses so clients can reproduce reads.

---

## 5. GraphQL

`POST /graphql`. Required root types:

```graphql
type Dataset {
  id: ID!
  name: String!
  schema: SchemaRef!
  rowCount: BigInt!
  sizeBytes: BigInt!
  snapshots(limit: Int = 50): [Snapshot!]!
  query(sql: String, wsql: JSON, snapshotId: ID,
        asOf: DateTime): QueryResult!
}

type QueryResult {
  schema: JSON!
  rows: [JSON!]!
  stats: QueryStats!
  cursor: String
}

type Query {
  dataset(id: ID!): Dataset
  datasets(filter: DatasetFilter, after: String, first: Int = 50): DatasetConnection!
}

type Mutation {
  createDataset(input: CreateDatasetInput!): Dataset!
  appendRows(datasetId: ID!, rows: [JSON!]!,
             idempotencyKey: String): AppendResult!
}

type Subscription {
  datasetChanges(id: ID!, includeBackfill: Boolean = false): ChangeEvent!
}
```

Query cost counts against the per-key rate-limit bucket (§ 9) as `cost = 1 + ceil(returned_rows / 1_000)`; clients can cap execution with `?maxCost=500`.

---

## 6. Arrow Flight SQL and ADBC

For analytic workloads, WIA exposes an **Arrow Flight SQL** endpoint at `grpc+tls://flight.wia-data.com:443`. Clients authenticate with the same bearer token (header `authorization: bearer $TOKEN`) and issue Flight SQL commands:

```python
import adbc_driver_flightsql.dbapi as flight_sql

conn = flight_sql.connect(
    uri="grpc+tls://flight.wia-data.com:443",
    db_kwargs={"adbc.flight.sql.authorization_header": f"Bearer {TOKEN}"},
)
cur = conn.cursor()
cur.execute("""
    SELECT user_id, COUNT(*) AS cnt
      FROM events
     WHERE event_ts >= ?
  GROUP BY user_id
""", parameters=[date(2026, 4, 1)])
tbl = cur.fetch_arrow_table()   # zero-copy Arrow Table
```

Benefits: avoids JSON serialisation cost (5–50× faster than REST for > 100 k rows) and keeps the columnar layout through pandas, Polars, DuckDB, and Spark.

---

## 7. Client SDKs

All SDKs MUST wrap the same semantic operations and honour the same retry model (§ 10).

### 7.1 Python

```python
from wia_data import Client

cli = Client(base_url="https://api.wia-data.com/v1", token=TOKEN)

# 1. Create dataset
ds = cli.datasets.create(
    name="user_events",
    schema="com.wia.events.EventRecord.v1",
    partitioning={"type": "time", "field": "event_ts", "granularity": "day"},
    compression="zstd",
)

# 2. Append with idempotency
res = cli.datasets[ds.id].append(
    rows=[{"event_id": "e-1", "user_id": 42, "event_ts": 1714046400000,
           "event_type": "click"}],
    idempotency_key="batch-2026-04-25-0001",
)

# 3. Query → pandas
df = cli.datasets[ds.id].sql(
    "SELECT user_id, COUNT(*) cnt FROM events "
    "WHERE event_ts > :since GROUP BY user_id ORDER BY cnt DESC LIMIT 100",
    since=date(2026, 4, 1),
).to_pandas()
```

### 7.2 TypeScript (fetch-only, no runtime deps)

```ts
import { WiaClient } from "@wia/data-client";

const cli = new WiaClient({ baseUrl: "https://api.wia-data.com/v1", token });

const { rows } = await cli.query("ds-12345", {
  sql:  `SELECT user_id, COUNT(*) cnt FROM events
         WHERE event_ts > $1 GROUP BY user_id`,
  bind: ["2026-04-01"],
  accept: "application/json",
});
```

### 7.3 Rust (async)

```rust
use wia_data::{Client, AppendBatch};

let cli = Client::new("https://api.wia-data.com/v1", token);
let batch = AppendBatch::new()
    .row(serde_json::json!({"event_id":"e-1","user_id":42,"event_ts":1_714_046_400_000,
                            "event_type":"click"}));
let r = cli.dataset("ds-12345").append(batch)
           .idempotency_key("batch-2026-04-25-0001")
           .send().await?;
```

### 7.4 Java (Arrow Flight SQL first-class)

```java
Client cli = Client.builder()
    .baseUrl("https://api.wia-data.com/v1")
    .flightUri("grpc+tls://flight.wia-data.com:443")
    .token(TOKEN).build();

try (FlightStream stream = cli.flight().execute(
        "SELECT user_id, COUNT(*) cnt FROM events WHERE event_ts > ?",
        LocalDate.of(2026, 4, 1))) {
    while (stream.next()) {
        VectorSchemaRoot root = stream.getRoot();
        // columnar processing
    }
}
```

---

## 8. Authentication and authorisation

### 8.1 OAuth 2.1 / OIDC (recommended)

Authorization Code + PKCE for interactive clients; Client Credentials for services. JWT access tokens carry `wia.scope`, `wia.tier`, `wia.datasets` claims. Refresh tokens SHOULD be rotated per RFC 6749bis.

### 8.2 API keys (legacy)

`Authorization: Bearer wia_{env}_{32-hex}`; rotated on the 90-day cadence.

### 8.3 Scopes and permissions

```
datasets:read
datasets:write
datasets:delete
datasets:admin
schema:read
schema:write
query:run
query:cancel
cdc:subscribe
acl:manage
```

### 8.4 Row/column-level security

RLS and CLS are expressed as Common Expression Language (CEL) predicates stored with the dataset ACL:

```json
{
  "rls": "resource.region == request.auth.claims.region",
  "cls": {"pii.ssn": "request.auth.claims.role == 'compliance'"}
}
```

Queries are rewritten server-side; unauthorised columns are omitted from the result schema, not nulled.

---

## 9. Rate limiting

Token bucket per `(principal, resource class)`:

| Class           | Burst | Refill (tokens/s) | Examples                     |
|-----------------|-------|-------------------|------------------------------|
| meta-read       | 600   | 10                | `GET /datasets`              |
| data-read       | 120   | 2                 | `POST /datasets/*/query`     |
| data-write      | 60    | 1                 | `POST /datasets/*/data`      |
| bulk-write      | 5     | 1 / 5 s           | multipart `/data?mode=bulk`  |
| stream-connect  | 5     | 1 / 60 s          | WebSocket / Flight connect   |

Every response MUST include:

```
RateLimit-Limit:     120
RateLimit-Remaining: 96
RateLimit-Reset:     17
```

`429` bodies are RFC 9457 `application/problem+json` with `Retry-After` (seconds).

---

## 10. Idempotency, retries, and error model

### 10.1 Idempotency keys

All mutating endpoints honour the `Idempotency-Key` header (RFC draft idempotency-key). Replays within 24 h return the original status/body; diverging payloads yield `409 duplicate_operation`.

### 10.2 Retry safety

| Status        | Retry?                 |
|---------------|------------------------|
| 408, 425, 429 | Yes, back off          |
| 5xx except 501| Yes, back off          |
| 400–403, 409  | No (fix the request)   |

Clients SHOULD apply jittered exponential back-off (`base=200ms`, `cap=30s`).

### 10.3 RFC 9457 error body

```json
{
  "type":     "https://errors.wia-data.com/invalid_schema",
  "title":    "Schema validation failed",
  "status":   422,
  "detail":   "Field 'user_id' missing in row 12 of batch.",
  "instance": "/datasets/ds-12345/data?mode=bulk",
  "errors": [
    { "path": "/rows/12/user_id", "rule": "required" }
  ]
}
```

Canonical error types:

| HTTP | `type` suffix         | Trigger                                      |
|------|-----------------------|----------------------------------------------|
| 400  | `bad_filter`          | Unparseable filter / WSQL                    |
| 400  | `query_too_expensive` | Estimated cost > plan limit                  |
| 401  | `invalid_token`       | Missing / expired JWT                        |
| 403  | `rls_denied`          | Row-level security rejection                 |
| 404  | `no_dataset`          | Dataset id not found                         |
| 409  | `duplicate_operation` | Idempotency key collision                    |
| 409  | `schema_conflict`     | Incompatible schema evolution                |
| 412  | `stale_snapshot`      | If-Match snapshot-id no longer current       |
| 422  | `invalid_schema`      | Row fails registered schema                  |
| 429  | `rate_limited`        | Bucket empty (see `Retry-After`)             |
| 503  | `partition_unavailable`| Backend partition offline                   |

---

## 11. Bulk ingest

`POST /datasets/{id}/data?mode=bulk` accepts Avro-OCF, Parquet, Arrow IPC, or line-delimited JSON. Request is `multipart/mixed`; each part is a logical batch with its own `Idempotency-Key`. Response is a per-part status list plus an overall **append transaction id**, aligned with Iceberg snapshot commits — see Phase 4 for the lakehouse consistency guarantees.

Example (curl, 1 GiB Arrow file):

```bash
curl -X POST "https://api.wia-data.com/v1/datasets/ds-12345/data?mode=bulk" \
  -H "Authorization: Bearer $TOKEN" \
  -H "Idempotency-Key: bulk-2026-04-25-0001" \
  -H "Content-Type: application/vnd.apache.arrow.file" \
  --data-binary @events-2026-04-25.arrow
```

---

## 12. Streaming reads (CDC + WebSocket)

`/streams/{id}` is a durable CDC subscription. Over WebSocket:

```json
{ "op": "subscribe",
  "dataset": "ds-12345",
  "since":   "snapshot:7219743893013213000",
  "include": ["insert", "update", "delete"],
  "token":   "eyJhb…" }
```

The server replays changes since the cursor, then switches to live; clients MUST `ack` cursors every ≤ 10 s or the consumer lag meter resets. The wire protocol is compatible with Debezium `io.debezium.data.Envelope` so existing sinks can plug in unchanged (see Phase 4 § 5).

---

## 13. Caching and conditional requests

- List endpoints: `Cache-Control: public, max-age=60`
- Dataset metadata: `max-age=300` with `ETag`
- Query results: **not cached** by default; clients MAY opt in with `Cache-Control: max-age=300` and receive a `Vary: Authorization, X-WIA-Snapshot-Id` header.

Conditional requests via `If-None-Match` and `If-Match` MUST be honoured; mismatched snapshot pins return `412 stale_snapshot`.

---

## 14. Observability

- **OpenAPI 3.1**: `/openapi.json` reflects every implemented path.
- **AsyncAPI 3.0**: `/asyncapi.json` for WebSocket/CDC.
- **Prometheus**: `/metrics` exposes `wia_data_query_duration_seconds`, `wia_data_query_rows_returned`, `wia_data_append_bytes_total`.
- **OpenTelemetry**: traces carry `wia.dataset`, `wia.query_id`, `wia.plan.cost_rows`.

---

## 15. Worked examples

### 15.1 Create → append → query round-trip

```bash
# 1. Create
curl -X POST https://api.wia-data.com/v1/datasets \
  -H "Authorization: Bearer $TOKEN" -H "Content-Type: application/json" \
  -d '{"name":"user_events","schema":"com.wia.events.EventRecord.v1",
       "partitioning":{"type":"time","field":"event_ts","granularity":"day"}}'
# → 201, Location: /datasets/ds-12345

# 2. Append
curl -X POST https://api.wia-data.com/v1/datasets/ds-12345/data \
  -H "Authorization: Bearer $TOKEN" -H "Content-Type: application/json" \
  -H "Idempotency-Key: row-$(uuidgen)" \
  -d '[{"event_id":"e-1","user_id":42,"event_ts":1714046400000,"event_type":"click"}]'

# 3. Query
curl -X POST https://api.wia-data.com/v1/datasets/ds-12345/query \
  -H "Authorization: Bearer $TOKEN" -H "Content-Type: application/sql" \
  -H "Accept: application/json" \
  --data "SELECT COUNT(*) AS total FROM events WHERE user_id = 42"
```

### 15.2 Arrow Flight pull into DuckDB

```python
import duckdb, adbc_driver_flightsql.dbapi as fs
conn = fs.connect(uri="grpc+tls://flight.wia-data.com:443",
                  db_kwargs={"adbc.flight.sql.authorization_header":
                             f"Bearer {TOKEN}"})
rb = conn.cursor().execute(
  "SELECT * FROM events WHERE event_ts >= current_date - 7").fetch_arrow_table()
duckdb.register("events", rb)
print(duckdb.sql("SELECT event_type, COUNT(*) FROM events GROUP BY 1").df())
```

---

## 16. Compliance checklist

- [ ] `/openapi.json` + `/asyncapi.json` current and reachable
- [ ] `/.well-known/wia-data` discovery document served
- [ ] RFC 9457 problem bodies on every 4xx/5xx
- [ ] Cursor pagination at ≥ 10 000 rows
- [ ] Arrow Flight SQL endpoint exposed
- [ ] Idempotency-Key honoured on every write
- [ ] OAuth 2.1/OIDC tokens accepted; API keys degrade gracefully
- [ ] Row- and column-level security enforced via CEL predicates
- [ ] Time-travel (`snapshot_id` / `as_of_timestamp`) returns deterministic rows
- [ ] `/metrics` and OTel traces published

---

## 17. References

1. RFC 9110, 9111, 9457 — HTTP semantics, caching, problem details
2. RFC 6902 — JSON Patch; RFC 8594 — Sunset header
3. OpenAPI 3.1, AsyncAPI 3.0
4. GraphQL June 2018 spec
5. Apache Arrow 15 — Columnar Format and Flight SQL
6. Arrow Database Connectivity (ADBC) 1.1
7. Common Expression Language (CEL) v1.0
8. Prometheus Exposition Format 0.0.4

---

**© 2026 WIA — World Certification Industry Association**
*弘益人間 · Benefit All Humanity*
