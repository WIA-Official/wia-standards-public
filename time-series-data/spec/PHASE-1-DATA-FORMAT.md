# WIA-DATA-014: PHASE 1 - Data Format Specification

**Version:** 1.0.0
**Status:** Complete
**Date:** 2025-12-26

---

## 1. Overview

PHASE 1 defines the standard data format for time-series data points, ensuring interoperability across different time-series database systems and applications.

### 1.1 Goals

- **Interoperability:** Enable data exchange between different TSDB systems
- **Efficiency:** Minimize storage and transmission overhead
- **Extensibility:** Support future enhancements without breaking changes
- **Human-readable:** JSON format for debugging and manual inspection
- **Machine-optimized:** Binary format for production use

### 1.2 Scope

This specification covers:
- Data point structure
- Encoding formats (JSON, Line Protocol, Binary)
- Data types and validation rules
- Metadata schemas

---

## 2. Data Point Structure

### 2.1 Core Components

Every time-series data point MUST contain:

| Component | Type | Required | Description |
|-----------|------|----------|-------------|
| `timestamp` | ISO8601 or Unix timestamp | Yes | When the measurement was taken |
| `measurement` | String | Yes | What is being measured |
| `fields` | Object | Yes | Actual measured values |
| `tags` | Object | No | Indexed metadata for filtering |

### 2.2 JSON Format

#### 2.2.1 Single Data Point

```json
{
  "timestamp": "2025-12-26T10:30:00Z",
  "measurement": "temperature",
  "tags": {
    "location": "server1",
    "region": "us-east-1",
    "sensor_type": "DHT22"
  },
  "fields": {
    "value": 23.5,
    "unit": "celsius",
    "accuracy": 0.5
  }
}
```

#### 2.2.2 Batch Data Points

```json
{
  "version": "1.0",
  "points": [
    {
      "timestamp": "2025-12-26T10:30:00Z",
      "measurement": "cpu",
      "tags": {"host": "server1"},
      "fields": {"usage": 75.2, "cores": 8}
    },
    {
      "timestamp": "2025-12-26T10:30:00Z",
      "measurement": "memory",
      "tags": {"host": "server1"},
      "fields": {"used": 4096, "total": 8192}
    }
  ]
}
```

### 2.3 Line Protocol Format

Compatible with InfluxDB Line Protocol:

```
measurement[,tag_key=tag_value,...] field_key=field_value[,field_key=field_value,...] [timestamp]
```

#### Examples:

```
temperature,location=server1,region=us-east value=23.5 1735208400000000000
cpu,host=server1 usage=75.2,cores=8i 1735208400000000000
network,interface=eth0 bytes_in=1024000u,bytes_out=512000u 1735208400000000000
```

### 2.4 Binary Format

For high-performance scenarios, WIA-DATA-014 defines a compact binary format:

```
[Header: 8 bytes]
[Timestamp: 8 bytes - nanoseconds since epoch]
[Measurement length: 2 bytes]
[Measurement: variable]
[Tag count: 1 byte]
[Tags: variable]
[Field count: 1 byte]
[Fields: variable]
```

---

## 3. Data Types

### 3.1 Timestamp Types

| Format | Example | Precision | Use Case |
|--------|---------|-----------|----------|
| ISO8601 | 2025-12-26T10:30:00Z | Variable | Human-readable, APIs |
| Unix (seconds) | 1735208400 | 1 second | General monitoring |
| Unix (milliseconds) | 1735208400000 | 1 ms | Application metrics |
| Unix (microseconds) | 1735208400000000 | 1 μs | High-frequency trading |
| Unix (nanoseconds) | 1735208400000000000 | 1 ns | Hardware-level metrics |

### 3.2 Field Types

| Type | Line Protocol | JSON | Range |
|------|--------------|------|-------|
| Float | `value=23.5` | `"value": 23.5` | IEEE 754 double |
| Integer | `count=100i` | `"count": 100` | -2^63 to 2^63-1 |
| Unsigned | `bytes=1024u` | `"bytes": 1024` | 0 to 2^64-1 |
| String | `status="ok"` | `"status": "ok"` | UTF-8 encoded |
| Boolean | `active=true` | `"active": true` | true/false |

### 3.3 Tag Constraints

- **Type:** Tags MUST be strings
- **Indexing:** Tags are indexed for fast filtering
- **Cardinality:** Recommended max 100,000 unique combinations per measurement
- **Naming:** Snake_case recommended, alphanumeric + underscore

---

## 4. Validation Rules

### 4.1 Measurement Names

- MUST be non-empty string
- MUST NOT contain whitespace
- RECOMMENDED: lowercase, snake_case
- RECOMMENDED: namespace (e.g., `system.cpu.usage`)

**Valid:**
```
temperature
cpu_usage
system.network.bytes_in
```

**Invalid:**
```
Temperature    # uppercase
cpu usage      # whitespace
""             # empty
```

### 4.2 Tag Rules

- Tag keys MUST be non-empty strings
- Tag values MUST be strings (no null)
- Duplicate tag keys are NOT allowed
- Order does NOT matter (tags are sorted for consistency)

### 4.3 Field Rules

- At least ONE field is REQUIRED
- Field keys MUST be non-empty strings
- Field values can be float, integer, unsigned, string, or boolean
- Duplicate field keys are NOT allowed

### 4.4 Timestamp Rules

- MUST be valid timestamp
- RECOMMENDED: UTC timezone
- Future timestamps allowed (for forecasts)
- Nanosecond precision preferred

---

## 5. Metadata Schema

### 5.1 Measurement Metadata

```json
{
  "measurement": "temperature",
  "description": "Temperature sensor readings",
  "unit": "celsius",
  "retention_policy": "7d",
  "tags": [
    {
      "key": "location",
      "description": "Physical location of sensor",
      "values": ["server1", "server2", "warehouse"]
    }
  ],
  "fields": [
    {
      "key": "value",
      "type": "float",
      "description": "Temperature in celsius",
      "range": [-273.15, 1000]
    }
  ]
}
```

---

## 6. Compression

### 6.1 Recommended Algorithms

| Data Type | Algorithm | Compression Ratio |
|-----------|-----------|-------------------|
| Timestamps | Delta-of-delta | 10:1 |
| Floats | Gorilla | 12:1 |
| Integers | Simple8b | 8:1 |
| Strings | LZ4 | 3:1 |

### 6.2 Gorilla Compression (Floats)

Facebook's Gorilla algorithm for time-series floats:
1. XOR current value with previous value
2. Store leading/trailing zeros count
3. Store significant bits only

---

## 7. Best Practices

### 7.1 Tag Design

✅ **DO:**
- Use tags for dimensions you filter/group by
- Keep cardinality under 100,000
- Use descriptive names
- Document allowed values

❌ **DON'T:**
- Use UUIDs or timestamps as tags
- Create tags with unbounded cardinality
- Use tags for high-cardinality data (use fields instead)

### 7.2 Field Design

✅ **DO:**
- Use appropriate data types
- Store measurements as fields
- Use descriptive names
- Include units in metadata

❌ **DON'T:**
- Use fields for filtering (use tags)
- Store computed values (compute on read)
- Mix incompatible units

### 7.3 Measurement Design

✅ **DO:**
- Use namespaces for organization
- Group related metrics
- Follow naming conventions
- Document schema

❌ **DON'T:**
- Use overly generic names
- Mix unrelated metrics
- Change schema frequently

---

## 8. Examples

### 8.1 IoT Sensor

```json
{
  "timestamp": "2025-12-26T10:30:00Z",
  "measurement": "iot.sensor",
  "tags": {
    "sensor_id": "DHT22_001",
    "location": "warehouse_a",
    "type": "environmental"
  },
  "fields": {
    "temperature": 23.5,
    "humidity": 65.2,
    "pressure": 1013.25,
    "battery_voltage": 3.7
  }
}
```

### 8.2 Infrastructure Monitoring

```json
{
  "timestamp": 1735208400000000000,
  "measurement": "system.cpu",
  "tags": {
    "host": "server1",
    "datacenter": "us-east-1a",
    "environment": "production"
  },
  "fields": {
    "usage_percent": 75.2,
    "cores": 8,
    "user_time": 45.1,
    "system_time": 30.1
  }
}
```

### 8.3 Financial Data

```json
{
  "timestamp": "2025-12-26T10:30:00.000Z",
  "measurement": "stock.price",
  "tags": {
    "symbol": "AAPL",
    "exchange": "NASDAQ",
    "currency": "USD"
  },
  "fields": {
    "open": 180.25,
    "high": 182.50,
    "low": 179.80,
    "close": 181.75,
    "volume": 75234567
  }
}
```

---

## 9. Compliance

Implementations claiming WIA-DATA-014 PHASE 1 compliance MUST:

1. Support JSON format for data ingestion
2. Accept ISO8601 timestamps
3. Support all specified data types
4. Validate data points according to rules
5. Provide error messages for invalid data

Implementations SHOULD:

1. Support Line Protocol format
2. Support nanosecond timestamp precision
3. Implement data compression
4. Provide metadata schema

---

## 10. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-12-26 | Initial specification |

---

## Appendix — Data Format Implementation Notes

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

## Appendix — Worked Sample Envelope

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
