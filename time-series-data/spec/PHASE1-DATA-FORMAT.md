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

**License:** MIT
**Copyright:** © 2025 WIA Standards
**弘益人間 (Benefit All Humanity)**
