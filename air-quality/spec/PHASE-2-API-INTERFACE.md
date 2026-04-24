# WIA-ENE-017: Air Quality Monitoring Standard
## PHASE 2: API INTERFACE

**Document Version:** 1.1
**Status:** Active
**Last Updated:** April 25, 2026
**Philosophy:** 弘益人間 (Hongik Ingan) · Benefit All Humanity

---

## 1. Introduction

Phase 2 specifies the **HTTP/REST, WebSocket, and GraphQL interfaces** that any WIA-ENE-017 conformant system MUST expose so that air-quality data described in Phase 1 can be discovered, queried, subscribed to, and published in an interoperable manner. The interface is designed to coexist with two widely deployed standards:

- **OGC SensorThings API 1.1** (ISO 19156 Observations & Measurements)
- **US EPA AQS** (Air Quality System) REST conventions

A WIA endpoint is considered *bridge-compatible* if it either serves or maps to the SensorThings resource model (§ 6), allowing federated ingestion by OpenAQ, EEA Air Quality e-Reporting, and WMO WIS 2.0 brokers without bespoke adapters.

### 1.1 Normative references

- RFC 9110 — HTTP Semantics
- RFC 9457 — Problem Details for HTTP APIs (error model)
- RFC 7946 — The GeoJSON Format
- RFC 6750 — OAuth 2.0 Bearer Token Usage
- RFC 8446 — TLS 1.3
- OpenAPI Specification 3.1.0
- OGC 15-078r6 — SensorThings API Part 1: Sensing v1.1
- ISO 8601:2019 — Date and time (already required in Phase 1)

### 1.2 Conformance keywords

MUST / SHOULD / MAY follow RFC 2119. Unless otherwise stated, requirements apply to **Tier 1 (Regulatory)** and **Tier 2 (Research)** endpoints; Tier 3–4 endpoints MAY relax performance envelopes but MUST NOT break the wire format.

---

## 2. Base URL, versioning, and discovery

### 2.1 Base URL

```
https://{host}/api/v1
```

A reference deployment runs at `https://api.airquality.wia.org/v1`. Operators MUST expose a machine-readable discovery document at the unversioned root:

```http
GET /.well-known/wia-airquality HTTP/1.1
Host: api.airquality.wia.org
Accept: application/json
```

```json
{
  "standard": "WIA-ENE-017",
  "version": "1.1",
  "base_url": "https://api.airquality.wia.org/v1",
  "openapi": "https://api.airquality.wia.org/v1/openapi.json",
  "sensorthings_bridge": "https://api.airquality.wia.org/sta/v1.1",
  "streams": {
    "websocket": "wss://stream.airquality.wia.org/v1/ws",
    "mqtt": "mqtts://mqtt.airquality.wia.org:8883"
  },
  "capabilities": ["geo.bbox", "geo.radius", "aggregate.hourly",
                   "aggregate.daily", "forecast.72h", "alerts.push"]
}
```

### 2.2 Content negotiation

| `Accept` | Meaning                                      |
|----------|----------------------------------------------|
| `application/json` (default) | Canonical Phase 1 JSON        |
| `application/geo+json`       | GeoJSON FeatureCollection     |
| `text/csv`                   | Flat spreadsheet (UTF-8, BOM-free) |
| `application/x-netcdf`       | Climate-Forecast CF-1.10 NetCDF |
| `application/vnd.ogc.sta+json` | SensorThings 1.1 envelope   |

Servers MUST support JSON and GeoJSON; others are OPTIONAL but, if implemented, MUST preserve the QC-flag column.

---

## 3. Resource model

The object graph is deliberately shallow. Every resource is addressable by a stable URI and exposes `ETag`, `Last-Modified`, and `Cache-Control` headers.

```
/stations                  ← Station collection (metadata)
/stations/{id}             ← Station document
/stations/{id}/current     ← Most recent valid reading per pollutant
/stations/{id}/readings    ← Time-series observations
/stations/{id}/aggregates  ← Pre-computed 1h / 8h / 24h averages
/stations/{id}/forecasts   ← Model-driven 1–72 h forecasts
/readings                  ← Cross-station feed (write + query)
/aqi/calculate             ← Pure function: concentrations → AQI
/alerts                    ← Active public-health notices
/reports                   ← Exceedance & compliance reports (Tier 1)
```

### 3.1 Pagination

Collections larger than 100 items MUST paginate. Two styles coexist; servers MUST accept either, producers SHOULD emit cursor form.

**Cursor (preferred — stable under concurrent writes):**

```
GET /stations/WIA-AQ-KR-Seoul-001/readings?limit=500&cursor=eyJ0cyI6IjIwMjYtMDQtMjVUMDA6MDA6MDBaIn0
```

Response envelope:
```json
{
  "data": [ … ],
  "paging": {
    "next":  "https://api…/readings?cursor=eyJ0cyI6I…A2OjAwOjAwWiJ9&limit=500",
    "prev":  null,
    "count": 500,
    "has_more": true
  }
}
```

**Offset (legacy):** `?offset=1000&limit=500`. Offsets beyond 10 000 SHOULD return `400 bad_pagination` and redirect the client to cursors.

### 3.2 Sparse fieldsets

Clients MAY request a subset via comma-separated `fields`:
```
GET /stations?fields=station_id,location,sensors.pollutant
```

### 3.3 Sorting and filtering

```
?sort=timestamp:desc,value:asc
?filter=pollutant=pm25 AND value>35 AND qc_flag=valid
```

The filter grammar is a subset of the OData `$filter` expression: logical `AND/OR/NOT`, comparators `= != < <= > >=`, and the ranges `between(a,b)` and `in(a,b,c)`.

---

## 4. Geographic queries

Three interchangeable forms accepted on any collection that has a spatial aspect:

1. **Bounding box** — `?bbox=126.76,37.43,127.18,37.70` *(minLon, minLat, maxLon, maxLat — WGS84)*
2. **Radius** — `?near=37.5665,126.9780&radius=5km`
3. **Polygon (GeoJSON)** — body for `POST /stations/_search`, `Content-Type: application/geo+json`

GeoJSON responses follow RFC 7946 exactly; a `Feature` carries Phase 1 measurement objects under `properties`, while `geometry` holds the station point. This is the shape OpenAQ, PurpleAir, and Leaflet/MapLibre clients expect natively.

---

## 5. OpenAPI 3.1 excerpt (normative)

A complete OpenAPI 3.1 document MUST be served at `/openapi.json`. The following fragment is the minimum an implementer MUST NOT diverge from:

```yaml
openapi: 3.1.0
info:
  title: WIA-ENE-017 Air Quality API
  version: "1.1"
  license: { name: CC-BY-4.0 }
servers:
  - url: https://api.airquality.wia.org/v1
security:
  - bearerAuth: []
paths:
  /stations/{id}/readings:
    get:
      operationId: listReadings
      parameters:
        - { name: id,        in: path,  required: true, schema: { type: string } }
        - { name: start,     in: query, schema: { type: string, format: date-time } }
        - { name: end,       in: query, schema: { type: string, format: date-time } }
        - { name: pollutant, in: query, schema: { type: string,
            enum: [pm25, pm10, o3, no2, so2, co] } }
        - { name: interval,  in: query, schema: { type: string,
            enum: [raw, 1min, 5min, 15min, 1hour, 8hour, 24hour] } }
        - { name: qc,        in: query, schema: { type: string,
            enum: [valid, valid+suspect, all] } }
        - { name: limit,     in: query, schema: { type: integer,
            minimum: 1, maximum: 10000, default: 500 } }
        - { name: cursor,    in: query, schema: { type: string } }
      responses:
        "200":
          description: Paginated time-series
          content:
            application/json:
              schema: { $ref: "#/components/schemas/ReadingsPage" }
            application/geo+json:
              schema: { $ref: "#/components/schemas/GeoReadings" }
        "304": { description: Not modified (ETag match) }
        "400": { $ref: "#/components/responses/Problem" }
        "429": { $ref: "#/components/responses/Problem" }
components:
  securitySchemes:
    bearerAuth: { type: http, scheme: bearer, bearerFormat: JWT }
  responses:
    Problem:
      description: RFC 9457 problem document
      content:
        application/problem+json:
          schema: { $ref: "#/components/schemas/Problem" }
  schemas:
    Problem:
      type: object
      required: [type, title, status]
      properties:
        type:     { type: string, format: uri }
        title:    { type: string }
        status:   { type: integer }
        detail:   { type: string }
        instance: { type: string, format: uri }
        errors:   { type: array, items: { type: object } }
```

---

## 6. SensorThings 1.1 bridge mapping

Every WIA resource has a deterministic SensorThings equivalent. A conformant server MUST either expose native SensorThings URLs **or** guarantee the following one-to-one mapping for brokers:

| WIA resource                           | SensorThings entity                      |
|----------------------------------------|------------------------------------------|
| `/stations/{id}`                       | `/Things(id)` + `/Locations(id)`         |
| `/stations/{id}/sensors/{p}`           | `/Datastreams(id)` (one per pollutant)   |
| `/stations/{id}/readings?pollutant=p`  | `/Datastreams(id)/Observations`          |
| `/alerts`                              | `/FeaturesOfInterest` + custom extension |

Observation payload conversion (WIA → STA):

```json
{
  "phenomenonTime": "2026-04-25T14:30:00Z",
  "resultTime":     "2026-04-25T14:30:00Z",
  "result":         35.4,
  "resultQuality":  "valid",
  "parameters": {
    "unit":        "ug/m3",
    "uncertainty": 3.5,
    "method":      "beta_attenuation"
  }
}
```

The station identifier convention `WIA-AQ-{CC}-{City}-{seq}` is preserved in `Thing.properties.wia_station_id` so downstream indexers can round-trip without collision.

---

## 7. Real-time streaming

### 7.1 WebSocket subscription

```
wss://stream.airquality.wia.org/v1/ws
```

Client frame (JSON, one per line; `op` is required):

```json
{ "op": "subscribe",
  "topics": ["readings.KR.Seoul.*", "alerts.KR.*"],
  "qc": "valid",
  "token": "eyJhbGciOi…" }
```

Server frames use the same envelope as REST responses. A `ping` is sent every 30 s; clients MUST reply with `pong` within 10 s or be disconnected. Replay is supported via `since`:

```json
{ "op": "subscribe", "topics": ["readings.KR.Seoul.001"],
  "since": "2026-04-25T14:00:00Z" }
```

The server MUST deliver missed messages in order before switching to live mode.

### 7.2 MQTT 5.0 (defined in Phase 3)

Topic map is canonical across transports:

```
wia/airquality/{country}/{city}/{station_id}/readings   # QoS 1
wia/airquality/{country}/{city}/{station_id}/status     # QoS 1, retained
wia/airquality/{country}/{city}/{station_id}/alerts     # QoS 2
```

---

## 8. GraphQL (optional, but recommended for dashboards)

Endpoint: `POST /graphql`. The schema MUST expose at least:

```graphql
type Station {
  id: ID!
  name: String!
  location: Point!
  currentAqi: Int
  sensors: [Sensor!]!
  readings(pollutant: Pollutant!, start: DateTime, end: DateTime,
           interval: Interval = RAW): [Reading!]!
}

type Reading {
  timestamp: DateTime!
  pollutant: Pollutant!
  value: Float!
  unit: Unit!
  qcFlag: QcFlag!
  uncertainty: Float
}

enum Pollutant { PM25 PM10 O3 NO2 SO2 CO }
enum Interval  { RAW MIN_5 MIN_15 HOUR_1 HOUR_8 DAY_1 }

type Query {
  station(id: ID!): Station
  stationsNear(lat: Float!, lon: Float!, radiusKm: Float! = 5.0): [Station!]!
  aqiNow(bbox: [Float!]!): [Station!]!
}
```

GraphQL responses count against the same rate-limit bucket (§ 10) using the query cost formula `cost = 1 + (#Readings returned / 100)`.

---

## 9. Error model (RFC 9457)

All 4xx/5xx responses MUST set `Content-Type: application/problem+json`.

```json
{
  "type":     "https://errors.airquality.wia.org/bad_filter",
  "title":    "Unsupported filter expression",
  "status":   400,
  "detail":   "Operator 'like' is not allowed on column 'value'.",
  "instance": "/stations/WIA-AQ-KR-Seoul-001/readings?filter=value%20like…",
  "errors": [
    { "field": "filter", "rule": "operator_whitelist",
      "allowed": ["=", "!=", "<", "<=", ">", ">=", "between", "in"] }
  ]
}
```

Canonical error `type` URIs:

| HTTP | `type` suffix           | Trigger                                   |
|------|-------------------------|-------------------------------------------|
| 400  | `bad_filter`            | Unparseable/unsafe `filter=`              |
| 400  | `bad_pagination`        | Offset > 10 000 or cursor tampered        |
| 401  | `invalid_token`         | Missing/expired bearer token              |
| 403  | `tier_forbidden`        | Tier 1 data requested without clearance   |
| 404  | `no_station`            | Unknown station_id                        |
| 409  | `duplicate_reading`     | Idempotency key re-used with new payload  |
| 410  | `retired_station`       | Station decommissioned                    |
| 422  | `schema_violation`      | POST body fails Phase 1 JSON Schema       |
| 429  | `rate_limited`          | Bucket empty (see `Retry-After`)          |
| 503  | `tier_downgraded`       | Sensor in calibration; only suspect data  |

---

## 10. Rate limiting

A token-bucket per `(api_key, resource_class)` with the defaults:

| Class      | Burst | Refill         | Examples                    |
|------------|-------|----------------|-----------------------------|
| read-light | 600   | 10 / s         | `GET /stations`, `/current` |
| read-heavy | 60    | 1 / s          | `readings?interval=raw`     |
| write      | 120   | 2 / s          | `POST /readings`            |
| stream     | 5     | 1 / 60 s       | WebSocket connect           |

Every response MUST carry:

```
RateLimit-Limit:     600
RateLimit-Remaining: 583
RateLimit-Reset:     17          ; seconds to bucket refill
```

---

## 11. Idempotency for writes

`POST /readings` MUST honour the `Idempotency-Key` header (RFC draft-ietf-httpapi-idempotency-key). Replays within 24 h return the original response body and status unchanged; diverging payloads for the same key yield `409 duplicate_reading`.

---

## 12. Worked examples

### 12.1 curl — current AQI for a bounding box

```bash
curl -sS -H "Authorization: Bearer $WIA_TOKEN" \
  "https://api.airquality.wia.org/v1/aqi/now?bbox=126.76,37.43,127.18,37.70" \
  | jq '.data[] | {station: .station_id, aqi: .aqi.value, dom: .aqi.dominant_pollutant}'
```

### 12.2 Python (httpx) — hourly PM2.5 for a week, QC-valid only

```python
import httpx, datetime as dt

token = os.environ["WIA_TOKEN"]
end   = dt.datetime.now(dt.timezone.utc).replace(minute=0, second=0, microsecond=0)
start = end - dt.timedelta(days=7)

params = {
    "start":     start.isoformat(),
    "end":       end.isoformat(),
    "pollutant": "pm25",
    "interval":  "1hour",
    "qc":        "valid",
    "limit":     10000,
}

with httpx.Client(base_url="https://api.airquality.wia.org/v1",
                  headers={"Authorization": f"Bearer {token}"}) as c:
    r = c.get("/stations/WIA-AQ-KR-Seoul-001/readings", params=params)
    r.raise_for_status()
    for row in r.json()["data"]:
        print(row["timestamp"], row["value"], row["qc_flag"])
```

### 12.3 JavaScript (browser, WebSocket) — live map pins

```js
const ws = new WebSocket("wss://stream.airquality.wia.org/v1/ws");
ws.addEventListener("open", () => ws.send(JSON.stringify({
  op: "subscribe",
  topics: ["readings.KR.Seoul.*"],
  qc: "valid",
  token: WIA_TOKEN,
})));
ws.addEventListener("message", (ev) => {
  const r = JSON.parse(ev.data);
  updatePin(r.station_id, r.measurements.pm25?.value, r.aqi?.value);
});
```

### 12.4 Rust (reqwest) — POST a reading with idempotency

```rust
let body = serde_json::json!({
    "station_id": "WIA-AQ-KR-Seoul-001",
    "timestamp":  chrono::Utc::now().to_rfc3339(),
    "measurements": {
        "pm25": {"value": 35.4, "unit": "ug/m3", "qc_flag": "valid"}
    }
});

let resp = client.post("https://api.airquality.wia.org/v1/readings")
    .bearer_auth(&token)
    .header("Idempotency-Key", uuid::Uuid::new_v4().to_string())
    .json(&body)
    .send().await?;
assert_eq!(resp.status(), 201);
```

---

## 13. Caching and freshness

- `current` endpoints: `Cache-Control: public, max-age=30`
- `readings?interval=1hour`: `Cache-Control: public, max-age=300`
- Immutable historical ranges (end < now − 24 h) MAY be marked `immutable`.
- Conditional requests via `If-None-Match: "<etag>"` MUST return `304` without body.

---

## 14. Backwards compatibility and deprecation

Breaking changes bump the major prefix (`/v2/…`). Within a major line, servers MUST:

- accept the union of all documented fields,
- emit deprecation notices via `Sunset` and `Deprecation` headers (RFC 8594) at least 180 days in advance,
- maintain a `CHANGELOG.md` in the OpenAPI repo linked from `info.x-changelog`.

---

## 15. Compliance checklist

**Phase 2 conformance requires:**

- [ ] `/openapi.json` reflects every implemented path
- [ ] `/.well-known/wia-airquality` discovery document is served
- [ ] JSON and GeoJSON responses both validate against Phase 1 schema
- [ ] Cursor pagination works on `readings` at ≥ 10 000 rows
- [ ] RFC 9457 problem details returned on every 4xx/5xx
- [ ] Rate-limit headers present on every response
- [ ] `Idempotency-Key` round-trips correctly on `POST /readings`
- [ ] SensorThings bridge fields preserved (§ 6)
- [ ] WebSocket replay works via `since=` parameter
- [ ] TLS 1.3, OAuth 2.0 / Bearer tokens enforced

---

## 16. References

1. RFC 9110 — HTTP Semantics
2. RFC 9457 — Problem Details for HTTP APIs
3. RFC 7946 — The GeoJSON Format
4. RFC 8594 — The Sunset HTTP Header Field
5. OGC 15-078r6 — SensorThings API Part 1 v1.1
6. WMO WIS 2.0 — Technical Regulations Vol. IV
7. US EPA AQS API Technical Documentation
8. GraphQL June 2018 Specification (stable)
9. OpenAPI 3.1.0 Specification

---

**Document Status:** ACTIVE
**Effective Date:** April 25, 2026
**Review Date:** April 25, 2028

© 2026 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
