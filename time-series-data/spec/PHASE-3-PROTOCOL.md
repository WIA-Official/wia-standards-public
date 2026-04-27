# WIA-DATA-014: PHASE 3 - Protocol Specification

**Version:** 1.0.0
**Status:** Complete
**Date:** 2025-12-26

---

## 1. Overview

PHASE 3 defines communication protocols, security, and transport mechanisms for time-series data.

---

## 2. Transport Protocols

### 2.1 HTTP/HTTPS

**Primary protocol for API communications**

- **TLS 1.2+** required for production
- **HTTP/2** recommended for multiplexing
- **Keep-alive** connections for batch operations

### 2.2 WebSocket

**For real-time streaming data**

```javascript
const ws = new WebSocket('wss://api.example.com/v1/stream');

ws.onmessage = (event) => {
  const dataPoint = JSON.parse(event.data);
  console.log(dataPoint);
};
```

### 2.3 gRPC

**High-performance binary protocol**

```protobuf
service TimeSeriesService {
  rpc Write(WriteRequest) returns (WriteResponse);
  rpc Query(QueryRequest) returns (stream QueryResponse);
}

message DataPoint {
  int64 timestamp = 1;
  string measurement = 2;
  map<string, string> tags = 3;
  map<string, Value> fields = 4;
}
```

### 2.4 MQTT

**For IoT sensor networks**

**Topic Structure:**
```
timeseries/{database}/{measurement}/{sensor_id}
```

**Payload:**
```json
{
  "timestamp": 1735208400000,
  "fields": {"temperature": 23.5, "humidity": 65.2}
}
```

---

## 3. Security

### 3.1 Authentication Methods

#### API Key
```http
Authorization: Bearer wia_sk_abc123def456
```

#### OAuth 2.0
```http
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

#### mTLS (Mutual TLS)
Client certificates for machine-to-machine authentication

### 3.2 Encryption

- **In Transit:** TLS 1.2+ with strong cipher suites
- **At Rest:** AES-256 encryption for stored data
- **Field-level:** Optional encryption for sensitive fields

### 3.3 Authorization

**Role-Based Access Control (RBAC)**

| Role | Read | Write | Delete | Admin |
|------|------|-------|--------|-------|
| Reader | ✓ | ✗ | ✗ | ✗ |
| Writer | ✓ | ✓ | ✗ | ✗ |
| Admin | ✓ | ✓ | ✓ | ✓ |

**Resource-level permissions:**
```json
{
  "user": "user@example.com",
  "permissions": [
    {
      "database": "production",
      "measurement": "cpu",
      "actions": ["read", "write"]
    }
  ]
}
```

---

## 4. Data Compression

### 4.1 HTTP Compression

```http
Accept-Encoding: gzip, deflate, br
Content-Encoding: gzip
```

### 4.2 Time-Series Specific

- **Gorilla compression** for float values
- **Delta encoding** for timestamps
- **Dictionary compression** for tags

---

## 5. Protocol Negotiation

### 5.1 Content Types

| Content-Type | Use Case |
|-------------|----------|
| `application/json` | Human-readable, APIs |
| `application/x-influxdb-line` | Line protocol |
| `application/octet-stream` | Binary format |
| `application/grpc+proto` | gRPC |

### 5.2 Version Negotiation

```http
Accept: application/vnd.wia.timeseries.v1+json
```

---

## 6. Streaming Protocols

### 6.1 Server-Sent Events (SSE)

```http
GET /stream?measurement=cpu&host=server1
Accept: text/event-stream
```

```
data: {"timestamp":"2025-12-26T10:30:00Z","value":75.2}

data: {"timestamp":"2025-12-26T10:30:01Z","value":76.1}
```

### 6.2 WebSocket Streaming

```javascript
// Subscribe to real-time updates
ws.send(JSON.stringify({
  action: 'subscribe',
  measurement: 'temperature',
  tags: {location: 'warehouse'}
}));
```

---

## 7. Error Handling

### 7.1 Retry Policies

**Exponential Backoff:**
```
retry_delay = min(max_delay, base_delay * 2^attempt)
```

**Example:**
- Attempt 1: 1 second
- Attempt 2: 2 seconds
- Attempt 3: 4 seconds
- Attempt 4: 8 seconds
- Max: 60 seconds

### 7.2 Circuit Breaker

- **Threshold:** 50% error rate over 10 requests
- **Open duration:** 30 seconds
- **Half-open:** Test with single request

---

## 8. Quality of Service

### 8.1 QoS Levels

| Level | Guarantee | Use Case |
|-------|-----------|----------|
| 0 | At most once | Non-critical metrics |
| 1 | At least once | General monitoring |
| 2 | Exactly once | Financial data |

### 8.2 Delivery Semantics

**At-least-once:** Default for most operations
**Exactly-once:** Use idempotency keys

```json
{
  "idempotency_key": "unique-request-id",
  "timestamp": "2025-12-26T10:30:00Z",
  "measurement": "payment",
  "fields": {"amount": 100.00}
}
```

---

## 9. Connection Management

### 9.1 Connection Pooling

- **Min connections:** 2
- **Max connections:** 50
- **Idle timeout:** 300 seconds
- **Max lifetime:** 3600 seconds

### 9.2 Keep-Alive

```http
Connection: keep-alive
Keep-Alive: timeout=300, max=1000
```

---

## 10. Monitoring and Observability

### 10.1 Protocol Metrics

- **Request rate:** requests/second
- **Error rate:** errors/requests
- **Latency:** p50, p95, p99
- **Throughput:** bytes/second

### 10.2 Health Checks

```http
GET /health

{
  "status": "healthy",
  "version": "1.0.0",
  "uptime": 3600,
  "connections": 42
}
```

---

## 11. Compliance Requirements

WIA-DATA-014 PHASE 3 compliant implementations MUST:

1. Support HTTPS with TLS 1.2+
2. Implement API key authentication
3. Support JSON content type
4. Provide error responses in standard format
5. Implement rate limiting

Implementations SHOULD:

1. Support HTTP/2
2. Implement OAuth 2.0
3. Support WebSocket streaming
4. Implement compression
5. Provide health check endpoint

---

## Appendix — Federation Protocol Implementation Notes

### Conformance test suite

A black-box test suite is published at
`https://github.com/WIA-Official/wia-time-series-data-conformance` and
walks through the public surface of this Phase plus the cross-Phase
integration scenarios. Hosts publishing `bridge_profile=Full` SHOULD
additionally pass the suite's extension tests for at least one
supported third-party time-series database (InfluxDB, Prometheus,
TimescaleDB, Apache IoTDB, VictoriaMetrics).

### Reference container

The `wia/time-series-data-host:1.0.0` container image implements every
endpoint specified in this Phase with mock data; integrators exercise
their bridge against it before going to production. The container ships
with a SQLite-backed store suitable for small-to-medium scale (up to
10 million points per series); production deployments typically swap in
a purpose-built time-series database via the WIA-supplied schema.

### Companion CLI

The `cli/time-series-data.sh` script ships sample envelope generators
(validate, series, point, query, rollup, info subcommands) so an
implementer can produce conformant payloads without hand-rolling JSON.
The CLI has no dependency beyond `jq` and POSIX shell, so it runs in
any CI environment without additional tooling installation.

### Operational considerations

Time-series data infrastructure has three operational considerations
that integrators consistently underestimate. First, retention policy
scales aggressively with cardinality; the standard recommends explicit
retention windows per series in the Phase 1 series envelope. Second,
rollup lag (the delay between a raw point arriving and being included
in a downsampled aggregate) is a UX issue for live dashboards; the
standard recommends documenting the rollup latency budget in the host’s
discovery document. Third, late-arriving points (out-of-order timestamps
from mobile devices on flaky networks) are common; hosts MUST accept
late points up to a configurable bound (default 24 hours) and MUST
trigger a re-rollup of the affected window.

### Backwards-compatibility promise

Within the 1.x line every endpoint and envelope listed in this Phase
MUST remain reachable and MUST continue to honour the documented
status codes and content shapes. Hosts MAY add optional query parameters,
response fields, new endpoints, or media types. Hosts MUST NOT remove
or repurpose existing ones. Breaking changes ride a major version bump
and MUST be preceded by a 12-month deprecation window per IETF
RFC 8594 and RFC 9745.

### Federation-specific notes

The Phase 3 federation handshake reuses the WIA-SOCIAL Phase 3 §5
receipt shape so that vendor implementations can share their federation
library across multiple WIA family standards. This is deliberate:
operators running multiple WIA standards (Time-Series Data plus
Vital Sign Streaming plus Social) should not have to maintain three
separate handshake state machines.

Replay defence bounds (300 second skew window, 600 second nonce cache)
apply uniformly across all WIA family handshakes. For high-volume
time-series ingestion, the replay defence cache MAY be a Bloom filter;
false positives MUST trigger a re-fetch via the standard's pull
endpoints rather than silent drops.

Audience controls are particularly important for time-series data
because the same time series can carry very different sensitivity
profiles depending on the deployment (an industrial vibration sensor
is operationally sensitive but not personally identifying, while a
home-thermostat time series can reveal occupancy patterns and is
therefore highly personally identifying). The standard does not pick
a single sensitivity classification — it requires the host to declare
the per-series classification and the consumer to match the declared
audience.

### Trust list maintenance

Each host maintains a signed trust list of federated peers (consumer
hosts, downstream aggregators, regulator subscribers). Trust lists are
republished at least monthly; peers refuse stale lists older than 60
days. A peer may self-publish a `revocation` envelope to immediately
drop trust between list refresh windows.

## Appendix — Worked Operator Failover

A representative scenario for a mid-scale industrial deployment:
the operator runs 200 production lines, each line has 50 sensors,
each sensor emits one measurement per second. Total ingest rate is
10 000 points per second.

```
series_id : ts_line_47_temp_sensor_3
labels    : { line=47, sensor=3, region=plant-A, env=prod }
unit      : Cel
cadence   : 1 Hz raw, 60 s avg downsample after 1h, 1h avg after 7d
retention : 90 days raw, 1y rollup, 5y archive
```

The 200 lines × 50 sensors = 10 000 distinct series; per series
cardinality (label combinations) is bounded at 1 since the sensor is
fixed to a line. At 1 Hz × 10 000 series × 86 400 seconds/day = 864 M
points/day, which a SQLite-backed reference container can serve
directly until raw retention boundary at 90 days. Production deployments
typically swap to a purpose-built time-series database (InfluxDB,
TimescaleDB, VictoriaMetrics, Apache IoTDB) once daily ingest exceeds
100 M points.

The rollup policy (1 Hz → 60 s avg after 1h, then 1h avg after 7d)
reduces storage by ~98% over the first 7 days while preserving the
operationally meaningful aggregate. Late-arriving points (the standard
accepts them up to 24 hours) trigger a re-rollup of the affected
60 s window; the re-rollup MUST emit a `rollup_updated` notice to
subscribers so live dashboards can refresh.

## Appendix — Conformance and Verification

Every host claiming conformance MUST pass the open conformance test
suite at `https://github.com/WIA-Official/wia-time-series-data-conformance`.
The suite runs against the host’s public Phase 2 endpoints and
verifies envelope round-trips, idempotency-key handling, replay-defence
bounds, audience-control enforcement, and the late-arriving-points
re-rollup contract.

Hosts publishing `bridge_profile=Full` SHOULD additionally pass the
suite’s extension tests for at least one supported third-party
time-series database. Bridge tests verify that the WIA wire format
maps deterministically to the target database’s native ingest format
without information loss.

---

**License:** MIT
**Copyright:** © 2025 WIA Standards
**弘益人間 (Benefit All Humanity)**
