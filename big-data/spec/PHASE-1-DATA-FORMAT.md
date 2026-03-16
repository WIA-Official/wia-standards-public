# WIA-DATA-001: Phase 1 - Data Format Specification

**Version:** 1.0.0
**Status:** Complete
**Last Updated:** 2025-01-01

---

## 1. Overview

Phase 1 of the WIA-DATA-001 Big Data Standard defines standardized data formats, schema definition languages, and serialization formats for big data systems.

### 1.1 Objectives

- Provide a universal schema definition language
- Define efficient serialization formats
- Enable schema evolution and versioning
- Support multiple data types and structures
- Ensure interoperability across platforms

---

## 2. Schema Definition Language (SDL)

### 2.1 SDL Syntax

WIA SDL uses JSON-based syntax for defining data schemas:

```json
{
  "namespace": "com.wia.data",
  "name": "EventRecord",
  "type": "record",
  "doc": "Standard event record format",
  "fields": [
    {
      "name": "event_id",
      "type": "string",
      "doc": "Unique event identifier"
    },
    {
      "name": "timestamp",
      "type": "long",
      "logicalType": "timestamp-millis",
      "doc": "Event timestamp in milliseconds since epoch"
    },
    {
      "name": "event_type",
      "type": "string",
      "doc": "Type of event"
    },
    {
      "name": "properties",
      "type": ["null", {"type": "map", "values": "string"}],
      "default": null,
      "doc": "Additional event properties"
    }
  ]
}
```

### 2.2 Primitive Types

| Type | Description | Size |
|------|-------------|------|
| `null` | No value | 0 bytes |
| `boolean` | True or false | 1 byte |
| `int` | 32-bit signed integer | 4 bytes |
| `long` | 64-bit signed integer | 8 bytes |
| `float` | Single-precision floating point | 4 bytes |
| `double` | Double-precision floating point | 8 bytes |
| `bytes` | Sequence of 8-bit unsigned bytes | Variable |
| `string` | Unicode character sequence | Variable |

### 2.3 Complex Types

#### 2.3.1 Records

```json
{
  "type": "record",
  "name": "Address",
  "fields": [
    {"name": "street", "type": "string"},
    {"name": "city", "type": "string"},
    {"name": "zipcode", "type": "string"}
  ]
}
```

#### 2.3.2 Arrays

```json
{"type": "array", "items": "string"}
```

#### 2.3.3 Maps

```json
{"type": "map", "values": "int"}
```

#### 2.3.4 Unions

```json
["null", "string", "int"]
```

### 2.4 Logical Types

| Logical Type | Base Type | Description |
|--------------|-----------|-------------|
| `decimal` | `bytes` | Arbitrary-precision decimal |
| `date` | `int` | Days since Unix epoch |
| `time-millis` | `int` | Milliseconds since midnight |
| `time-micros` | `long` | Microseconds since midnight |
| `timestamp-millis` | `long` | Milliseconds since Unix epoch |
| `timestamp-micros` | `long` | Microseconds since Unix epoch |
| `duration` | `fixed[12]` | Time duration |
| `uuid` | `string` | UUID format |

---

## 3. WIA Binary Format (WBF)

### 3.1 Design Principles

- **Compact:** Minimal overhead through efficient encoding
- **Fast:** Optimized for serialization and deserialization
- **Schema-aware:** Embedded or referenced schema information
- **Streaming-friendly:** Support for incremental processing

### 3.2 Encoding Rules

#### 3.2.1 Primitive Encoding

**Boolean:**
- `0x00` for false
- `0x01` for true

**Integer (Variable-length zigzag encoding):**
- Positive integers encoded as 2n
- Negative integers encoded as 2|n|-1
- Reduces size for small values

**Long (Variable-length zigzag encoding):**
- Same as integer but supports 64-bit range

**Float/Double:**
- IEEE 754 standard encoding
- Little-endian byte order

**String:**
- Length prefix (variable-length encoded)
- UTF-8 encoded bytes

**Bytes:**
- Length prefix
- Raw byte sequence

#### 3.2.2 Complex Type Encoding

**Array:**
```
[long count] [long block_size] [items...] [0]
```

**Map:**
```
[long count] [long block_size] [key value pairs...] [0]
```

**Record:**
```
[field1] [field2] ... [fieldN]
```

**Union:**
```
[long index] [value]
```

### 3.3 WBF File Format

```
WBF Header:
  Magic bytes: 0x57 0x42 0x46 0x01 ("WBF" + version)
  Schema: Embedded or reference to schema registry
  Codec: Compression codec identifier
  Metadata: Key-value pairs

Data Blocks:
  [Block 1]
  [Block 2]
  ...
  [Block N]

Sync Marker: 16-byte random marker for splitting
```

---

## 4. Alternative Serialization Formats

### 4.1 JSON Format

- Human-readable text format
- Use for debugging and development
- Inefficient for production workloads
- Schema validation via JSON Schema

### 4.2 Avro Format

- Full compatibility with Apache Avro
- Schema evolution support
- Binary and JSON encoding
- RPC framework integration

### 4.3 Parquet Format

- Columnar storage format
- Optimized for analytics
- Excellent compression ratios
- Predicate pushdown support

### 4.4 ORC Format

- Optimized Row Columnar format
- High compression
- Built-in indexes
- ACID transaction support

---

## 5. Schema Registry

### 5.1 Purpose

- Centralized schema management
- Version control and evolution
- Compatibility checking
- Schema reuse across teams

### 5.2 Schema Versioning

Each schema is assigned a unique version identifier:

```
{namespace}.{name}.v{version}
```

Example: `com.wia.data.EventRecord.v1`

### 5.3 Compatibility Modes

| Mode | Forward | Backward | Description |
|------|---------|----------|-------------|
| BACKWARD | No | Yes | New schema can read old data |
| FORWARD | Yes | No | Old schema can read new data |
| FULL | Yes | Yes | Both backward and forward compatible |
| NONE | No | No | No compatibility checks |

### 5.4 Evolution Rules

**Backward compatible changes:**
- Add optional field with default
- Remove optional field

**Forward compatible changes:**
- Add optional field
- Remove field with default

**Breaking changes:**
- Change field type
- Rename field (without alias)
- Remove required field

---

## 6. Data Partitioning

### 6.1 Partition Strategies

#### 6.1.1 Time-based Partitioning

```json
{
  "partition_spec": {
    "type": "time",
    "field": "timestamp",
    "granularity": "day",
    "format": "yyyy-MM-dd"
  }
}
```

#### 6.1.2 Hash Partitioning

```json
{
  "partition_spec": {
    "type": "hash",
    "field": "user_id",
    "num_partitions": 256
  }
}
```

#### 6.1.3 Range Partitioning

```json
{
  "partition_spec": {
    "type": "range",
    "field": "age",
    "ranges": [
      {"min": 0, "max": 18},
      {"min": 18, "max": 65},
      {"min": 65, "max": null}
    ]
  }
}
```

---

## 7. Compression

### 7.1 Supported Codecs

| Codec | Compression Ratio | Speed | Use Case |
|-------|-------------------|-------|----------|
| Snappy | 2-3x | Very Fast | General purpose, real-time |
| LZ4 | 2-3x | Extremely Fast | Low latency requirements |
| Gzip | 3-5x | Slow | Archival, storage optimization |
| Zstd | 3-6x | Medium | Balanced performance |
| Brotli | 3-5x | Slow | Web serving, static data |

### 7.2 Compression Configuration

```json
{
  "compression": {
    "codec": "zstd",
    "level": 3,
    "block_size": 262144
  }
}
```

---

## 8. Metadata

### 8.1 Required Metadata

Every WIA dataset must include:

```json
{
  "created_at": "2025-01-01T00:00:00Z",
  "created_by": "WIA-DATA-001",
  "schema_version": "1.0.0",
  "format_version": "1.0.0",
  "row_count": 1000000,
  "byte_size": 52428800
}
```

### 8.2 Optional Metadata

```json
{
  "description": "User events from 2025-01-01",
  "tags": ["events", "production"],
  "owner": "data-team",
  "retention_days": 365
}
```

---

## 9. Implementation Examples

### 9.1 Python Example

```python
from wia.data import Schema, WBFWriter, WBFReader

# Define schema
schema = Schema.parse("""
{
  "namespace": "com.example",
  "name": "Event",
  "type": "record",
  "fields": [
    {"name": "id", "type": "string"},
    {"name": "timestamp", "type": "long"},
    {"name": "value", "type": "double"}
  ]
}
""")

# Write data
with WBFWriter("events.wbf", schema) as writer:
    writer.write({"id": "evt-1", "timestamp": 1704067200000, "value": 42.5})
    writer.write({"id": "evt-2", "timestamp": 1704067260000, "value": 38.2})

# Read data
with WBFReader("events.wbf") as reader:
    for record in reader:
        print(record)
```

### 9.2 Java Example

```java
import com.wia.data.*;

// Parse schema
Schema schema = new Schema.Parser().parse(new File("schema.wsd"));

// Write data
WBFWriter writer = new WBFWriter(new File("events.wbf"), schema);
GenericRecord record = new GenericData.Record(schema);
record.put("id", "evt-1");
record.put("timestamp", System.currentTimeMillis());
record.put("value", 42.5);
writer.append(record);
writer.close();

// Read data
WBFReader reader = new WBFReader(new File("events.wbf"));
while (reader.hasNext()) {
    GenericRecord r = reader.next();
    System.out.println(r);
}
reader.close();
```

---

## 10. Validation and Testing

### 10.1 Schema Validation

All schemas must pass validation:
- Well-formed JSON
- Valid type definitions
- No circular references
- Namespace conventions followed

### 10.2 Compatibility Testing

Test schema evolution scenarios:
- Add optional field
- Remove optional field
- Change field type (should fail)
- Add required field (should warn)

### 10.3 Performance Benchmarks

Serialization performance targets:
- Write throughput: > 100MB/s
- Read throughput: > 150MB/s
- Compression ratio: > 2x (Snappy)
- Schema lookup: < 1ms

---

## 11. Conformance Requirements

To be WIA-DATA-001 Phase 1 conformant, an implementation must:

1. Support WIA Schema Definition Language
2. Implement WBF binary format
3. Provide at least one alternative format (JSON, Avro, or Parquet)
4. Support schema versioning
5. Implement at least two compression codecs
6. Pass conformance test suite

---

## 12. References

- [Apache Avro Specification](https://avro.apache.org/docs/current/spec.html)
- [Parquet Format Specification](https://parquet.apache.org/docs/)
- [Protocol Buffers](https://developers.google.com/protocol-buffers)
- [JSON Schema](https://json-schema.org/)

---

**© 2025 WIA - World Certification Industry Association**
*弘益人間 · Benefit All Humanity*
