# WIA-DATA-010: PHASE 1 - Data Format Specification

**Version:** 1.0.0  
**Status:** Complete  
**Last Updated:** 2025-01-15

---

## Overview

This document specifies the standard data formats, schemas, and serialization protocols for WIA-DATA-010 Data Integration Standard. It ensures interoperability across different data integration tools and platforms.

## 1. Standard Data Formats

### 1.1 Structured Data Formats

#### JSON (JavaScript Object Notation)
- **Use Case:** API payloads, configuration files, semi-structured data
- **Specification:** RFC 8259
- **Character Encoding:** UTF-8
- **MIME Type:** `application/json`

**Example:**
```json
{
  "customer_id": "cust_123",
  "name": "John Doe",
  "email": "john@example.com",
  "created_at": "2025-01-15T10:30:00Z",
  "lifetime_value": 1250.50,
  "metadata": {
    "segment": "premium",
    "region": "NA"
  }
}
```

#### CSV (Comma-Separated Values)
- **Use Case:** Tabular data exports, batch file transfers
- **Specification:** RFC 4180
- **Character Encoding:** UTF-8
- **Delimiter:** Comma (`,`), customizable
- **Quote Character:** Double quote (`"`)
- **Header Row:** Required (first row contains column names)

**Example:**
```csv
customer_id,name,email,created_at,lifetime_value
cust_123,John Doe,john@example.com,2025-01-15T10:30:00Z,1250.50
cust_124,Jane Smith,jane@example.com,2025-01-15T11:00:00Z,3400.75
```

#### Parquet
- **Use Case:** Analytical workloads, data lake storage, columnar analytics
- **Specification:** Apache Parquet Format
- **Compression:** Snappy, Gzip, LZ4 (Snappy recommended)
- **Schema:** Embedded within file
- **Benefits:** Columnar storage, efficient compression, predicate pushdown

#### Avro
- **Use Case:** Data serialization, streaming data, schema evolution
- **Specification:** Apache Avro 1.11+
- **Schema:** JSON-based schema definition
- **Benefits:** Schema evolution, compact binary format, code generation

**Avro Schema Example:**
```json
{
  "type": "record",
  "name": "Customer",
  "namespace": "com.wia.data",
  "fields": [
    {"name": "customer_id", "type": "string"},
    {"name": "name", "type": "string"},
    {"name": "email", "type": "string"},
    {"name": "created_at", "type": "long", "logicalType": "timestamp-millis"},
    {"name": "lifetime_value", "type": "double"}
  ]
}
```

### 1.2 Semi-Structured Data Formats

#### XML
- **Use Case:** Legacy enterprise systems, SOAP APIs, configuration
- **Specification:** XML 1.0 (Fifth Edition)
- **Character Encoding:** UTF-8
- **Validation:** XSD schema validation supported

#### YAML
- **Use Case:** Configuration files, data pipelines, infrastructure as code
- **Specification:** YAML 1.2
- **Character Encoding:** UTF-8

### 1.3 Binary Formats

#### Protocol Buffers (Protobuf)
- **Use Case:** gRPC APIs, high-performance microservices
- **Specification:** Protocol Buffers v3
- **Benefits:** Compact binary format, strong typing, code generation

#### Apache ORC
- **Use Case:** Hive tables, data lakes, analytical queries
- **Specification:** Apache ORC Format
- **Compression:** Zlib, Snappy (Zlib recommended)
- **Benefits:** Columnar storage, predicate pushdown, stripe-based storage

## 2. Data Types

### 2.1 Primitive Types

| WIA Type | JSON | CSV | Parquet | Avro | Description |
|----------|------|-----|---------|------|-------------|
| `string` | string | string | BYTE_ARRAY | string | UTF-8 encoded text |
| `integer` | number | number | INT32 | int | 32-bit signed integer |
| `long` | number | number | INT64 | long | 64-bit signed integer |
| `float` | number | number | FLOAT | float | 32-bit floating point |
| `double` | number | number | DOUBLE | double | 64-bit floating point |
| `boolean` | boolean | true/false | BOOLEAN | boolean | true or false |
| `date` | string (ISO 8601) | string (ISO 8601) | INT32 (days since epoch) | int (logicalType: date) | Calendar date |
| `timestamp` | string (ISO 8601) | string (ISO 8601) | INT64 (ms since epoch) | long (logicalType: timestamp-millis) | Date and time |
| `bytes` | base64 string | base64 string | BYTE_ARRAY | bytes | Binary data |
| `null` | null | empty | NULL | null | Null value |

### 2.2 Complex Types

#### Arrays
- **JSON:** `[value1, value2, ...]`
- **Parquet:** Repeated fields
- **Avro:** `{"type": "array", "items": "type"}`

#### Maps/Dictionaries
- **JSON:** `{"key1": "value1", "key2": "value2"}`
- **Parquet:** MAP type
- **Avro:** `{"type": "map", "values": "type"}`

#### Nested Records
- **JSON:** Objects within objects
- **Parquet:** GROUP type
- **Avro:** `{"type": "record", "fields": [...]}`

## 3. Schema Definition

### 3.1 JSON Schema
All JSON data should provide optional JSON Schema for validation.

**Example:**
```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "properties": {
    "customer_id": {"type": "string"},
    "name": {"type": "string"},
    "email": {"type": "string", "format": "email"},
    "created_at": {"type": "string", "format": "date-time"},
    "lifetime_value": {"type": "number", "minimum": 0}
  },
  "required": ["customer_id", "name", "email"]
}
```

### 3.2 Schema Registry
For streaming and schema evolution scenarios:
- **Recommended:** Confluent Schema Registry or AWS Glue Schema Registry
- **Format:** Avro, JSON Schema, or Protobuf
- **Versioning:** Backward, forward, and full compatibility modes
- **Subject Naming:** `<topic>-value` for Kafka topics

## 4. Encoding and Compression

### 4.1 Character Encoding
- **Standard:** UTF-8 for all text data
- **Byte Order Mark (BOM):** Not required, but acceptable

### 4.2 Compression

| Format | Recommended Compression | Alternative |
|--------|------------------------|-------------|
| CSV | Gzip | None |
| JSON | Gzip | LZ4 |
| Parquet | Snappy | Gzip, LZ4, Zstd |
| Avro | Snappy | Deflate |
| ORC | Zlib | Snappy |

### 4.3 File Naming Conventions
```
<dataset>_<partition>_<timestamp>.<format>[.<compression>]

Examples:
- customers_2025-01-15_103000.parquet.snappy
- orders_part_001_20250115.csv.gz
- events_20250115.avro
```

## 5. Data Quality Constraints

### 5.1 Null Handling
- **Nullable fields:** Explicitly mark in schema
- **Required fields:** Must not be null
- **Default values:** Specify in schema when appropriate

### 5.2 Data Validation Rules
```yaml
validations:
  - field: email
    rule: format
    value: email
  
  - field: created_at
    rule: not_null
  
  - field: lifetime_value
    rule: range
    min: 0
    max: 1000000
  
  - field: customer_id
    rule: unique
```

## 6. Partitioning Standards

### 6.1 Date-Based Partitioning
```
/year=YYYY/month=MM/day=DD/
/dt=YYYY-MM-DD/
/date_partition=YYYYMMDD/
```

### 6.2 Hive-Style Partitioning
```
s3://bucket/dataset/year=2025/month=01/day=15/data.parquet
```

### 6.3 Multi-Level Partitioning
```
s3://bucket/dataset/region=NA/country=US/state=CA/data.parquet
```

## 7. Metadata Standards

### 7.1 File-Level Metadata
Every data file should include embedded or sidecar metadata:

```json
{
  "version": "1.0",
  "created_at": "2025-01-15T10:30:00Z",
  "created_by": "etl-pipeline-v2",
  "source_system": "salesforce",
  "row_count": 125000,
  "schema_version": "2.1",
  "checksum": "sha256:abc123..."
}
```

### 7.2 Column-Level Metadata
- **Description:** Human-readable description
- **Data Type:** Semantic data type (e.g., email, currency, phone)
- **Classification:** PII, PHI, public, internal
- **Format:** Expected format or pattern

## 8. Interoperability Requirements

### 8.1 Format Conversion
Implementations must support conversion between:
- JSON ↔ CSV
- JSON ↔ Parquet
- Avro ↔ Parquet
- CSV ↔ Parquet

### 8.2 Schema Evolution
- **Backward Compatibility:** New schemas can read old data
- **Forward Compatibility:** Old schemas can read new data (ignoring new fields)
- **Full Compatibility:** Both backward and forward compatible

**Allowed Changes:**
- Add optional fields
- Remove optional fields
- Widen data types (int32 → int64)

**Prohibited Changes:**
- Change field types (incompatible)
- Rename fields without aliases
- Remove required fields

## 9. Compliance and Security

### 9.1 PII/PHI Handling
- **Encryption:** Sensitive fields must be encrypted at rest
- **Masking:** Support for field-level masking in non-production
- **Tokenization:** Recommended for highly sensitive fields (SSN, credit cards)

### 9.2 Data Lineage
Every data file must track:
- Source system
- Transformation pipeline
- Processing timestamp
- Data quality checks performed

## 10. Implementation Checklist

- [ ] Choose appropriate format (JSON, Parquet, Avro, CSV)
- [ ] Define schema with proper data types
- [ ] Implement compression (Snappy for Parquet/Avro, Gzip for CSV/JSON)
- [ ] Apply partitioning strategy
- [ ] Add metadata (file-level and column-level)
- [ ] Implement data validation rules
- [ ] Plan for schema evolution
- [ ] Handle PII/PHI appropriately
- [ ] Document data lineage
- [ ] Test interoperability with target systems

---

**Next Phase:** [PHASE 2 - API Interface](PHASE2-API.md)

**Status:** ✅ Complete  
**Compliance:** WIA-DATA-010 v1.0.0
