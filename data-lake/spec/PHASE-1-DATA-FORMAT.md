# WIA Data Lake Data Format Standard
## Phase 1 Specification

---

**Version**: 1.0.0
**Status**: Complete
**Date**: 2025-12-26
**Authors**: WIA Standards Committee
**License**: MIT

---

## Table of Contents

1. [Overview](#overview)
2. [Terminology](#terminology)
3. [File Formats](#file-formats)
4. [Partitioning Strategies](#partitioning-strategies)
5. [Compression Codecs](#compression-codecs)
6. [Schema Design](#schema-design)
7. [Examples](#examples)
8. [References](#references)

---

## Overview

### 1.1 Purpose

The WIA Data Lake Data Format Standard defines the file formats, partitioning strategies, and compression methods for storing data in a compliant data lake.

**Core Objectives**:
- Standardize data storage formats across implementations
- Enable interoperability between different processing engines
- Optimize for both storage efficiency and query performance
- Support schema evolution without breaking compatibility

### 1.2 Scope

This specification covers:

| Component | Description |
|-----------|-------------|
| **File Formats** | Parquet, ORC, Avro, JSON, CSV |
| **Partitioning** | Date-based, hash-based, range-based |
| **Compression** | Snappy, GZIP, ZSTD, LZ4, Brotli |
| **Data Types** | Primitive and complex types |
| **Schema Evolution** | Adding, removing, renaming columns |

### 1.3 Design Principles

1. **Interoperability**: All formats must be readable by major compute engines (Spark, Presto, Hive)
2. **Performance**: Columnar formats preferred for analytical workloads
3. **Efficiency**: Compression enabled by default
4. **Evolvability**: Schema changes without data rewriting
5. **Simplicity**: Clear naming conventions and directory structures

---

## Terminology

### 2.1 Core Terms

| Term | Definition |
|------|------------|
| **File Format** | The structure and encoding of data within files |
| **Partition** | A logical grouping of data based on column values |
| **Compression Codec** | Algorithm for reducing file size |
| **Schema** | The structure definition of data (columns, types) |
| **Row Group** | A logical horizontal partitioning within a file |
| **Column Chunk** | Data for a single column within a row group |

### 2.2 File Format Categories

| Category | Description | Examples |
|----------|-------------|----------|
| **Columnar** | Data organized by column | Parquet, ORC |
| **Row-based** | Data organized by row | Avro, JSON, CSV |
| **Table Format** | ACID-enabled layer over files | Delta Lake, Iceberg, Hudi |

---

## File Formats

### 3.1 Apache Parquet (REQUIRED)

**Status**: PRIMARY format for analytical workloads

#### 3.1.1 Specification

```
Format: Apache Parquet 1.13.0+
Encoding: UTF-8 for strings
Compression: ZSTD (default), Snappy, GZIP, LZ4
Row Group Size: 128 MB (default)
Page Size: 1 MB (default)
Dictionary Encoding: Enabled by default
```

#### 3.1.2 File Structure

```
Parquet File:
┌─────────────────────┐
│ Magic: PAR1         │
├─────────────────────┤
│ Row Group 1         │
│  ├─ Column Chunk A  │
│  │   ├─ Data Pages  │
│  │   └─ Statistics  │
│  ├─ Column Chunk B  │
│  └─ Column Chunk C  │
├─────────────────────┤
│ Row Group N         │
├─────────────────────┤
│ File Metadata       │
│  ├─ Schema          │
│  ├─ Row Group Meta  │
│  └─ Column Stats    │
├─────────────────────┤
│ Footer Length       │
├─────────────────────┤
│ Magic: PAR1         │
└─────────────────────┘
```

#### 3.1.3 Configuration

```python
# PySpark Example
df.write.format("parquet") \
    .option("compression", "zstd") \
    .option("parquet.block.size", 134217728) \  # 128 MB
    .option("parquet.page.size", 1048576) \      # 1 MB
    .option("parquet.enable.dictionary", "true") \
    .mode("append") \
    .save("s3://lake/data/")
```

### 3.2 Apache ORC (SUPPORTED)

**Status**: SUPPORTED for Hive-native workloads

#### 3.2.1 Specification

```
Format: Apache ORC 1.8.0+
Compression: ZSTD (default), Snappy, LZ4
Stripe Size: 64 MB (default)
Row Index Stride: 10,000 rows
Bloom Filters: Enabled for string columns
```

#### 3.2.2 Usage Guidelines

Use ORC when:
- Primary compute engine is Hive
- Need better compression than Parquet
- String-heavy data benefits from bloom filters

### 3.3 Apache Avro (SUPPORTED)

**Status**: SUPPORTED for schema evolution scenarios

#### 3.3.1 Specification

```
Format: Apache Avro 1.11.0+
Compression: Snappy (default), Deflate
Codec: Binary encoding
Schema Registry: Optional but recommended
```

#### 3.3.2 Use Cases

- Streaming data ingestion (Kafka integration)
- Frequent schema changes
- Row-based access patterns
- Inter-service data exchange

### 3.4 JSON / JSONL (SUPPORTED)

**Status**: SUPPORTED for semi-structured data

#### 3.4.1 Specification

```
Format: JSON Lines (newline-delimited)
Encoding: UTF-8
Compression: GZIP (strongly recommended)
Max Line Size: 10 MB (recommended)
```

#### 3.4.2 Guidelines

- Use ONLY in Raw zone
- Convert to Parquet in Curated zone
- Enable compression to reduce costs
- NOT recommended for large-scale analytics

### 3.5 CSV (LEGACY SUPPORT)

**Status**: LEGACY - migrate to modern formats

#### 3.5.1 Specification

```
Delimiter: , (comma) or \t (tab)
Quote Character: " (double quote)
Escape Character: \ (backslash)
Header: Required
Encoding: UTF-8
```

#### 3.5.2 Migration Path

```sql
-- Read CSV
CREATE EXTERNAL TABLE raw_csv (
  id STRING,
  name STRING,
  timestamp STRING
)
ROW FORMAT DELIMITED
FIELDS TERMINATED BY ','
LOCATION 's3://lake/raw/csv/';

-- Convert to Parquet
CREATE TABLE curated_parquet
STORED AS PARQUET
LOCATION 's3://lake/curated/parquet/'
AS SELECT
  CAST(id AS BIGINT),
  name,
  CAST(timestamp AS TIMESTAMP)
FROM raw_csv;
```

---

## Partitioning Strategies

### 4.1 Date-Based Partitioning (RECOMMENDED)

#### 4.1.1 Directory Structure

```
s3://lake/events/
├── year=2025/
│   ├── month=01/
│   │   ├── day=15/
│   │   │   ├── part-00000.snappy.parquet
│   │   │   └── part-00001.snappy.parquet
│   │   ├── day=16/
│   │   └── day=17/
│   ├── month=02/
│   └── month=03/
└── year=2024/
```

#### 4.1.2 Query Optimization

```sql
-- Partition pruning example
SELECT * FROM events
WHERE year = 2025
  AND month = 01
  AND day = 15;
-- Only scans s3://lake/events/year=2025/month=01/day=15/
```

#### 4.1.3 Retention Policies

```python
# Delete old partitions
spark.sql("""
  ALTER TABLE events
  DROP IF EXISTS PARTITION (year=2020)
""")
```

### 4.2 Hash-Based Partitioning

#### 4.2.1 Implementation

```python
from pyspark.sql.functions import hash, col

df_partitioned = df.withColumn(
    "hash_bucket",
    (hash(col("user_id")) % 256).cast("int")
)

df_partitioned.write \
    .partitionBy("hash_bucket") \
    .parquet("s3://lake/users/")
```

#### 4.2.2 Directory Structure

```
s3://lake/users/
├── hash_bucket=0/
├── hash_bucket=1/
├── hash_bucket=2/
...
├── hash_bucket=255/
```

### 4.3 Range-Based Partitioning

#### 4.3.1 Implementation

```sql
CREATE TABLE transactions (
  id BIGINT,
  amount DECIMAL(10,2),
  timestamp TIMESTAMP
)
PARTITIONED BY (amount_range STRING)
STORED AS PARQUET;

-- Insert with manual partition assignment
INSERT INTO transactions PARTITION(amount_range='0-100')
SELECT id, amount, timestamp FROM raw_transactions
WHERE amount BETWEEN 0 AND 100;
```

---

## Compression Codecs

### 5.1 Codec Comparison

| Codec | Compression Ratio | Encode Speed | Decode Speed | CPU Usage | Recommendation |
|-------|-------------------|--------------|--------------|-----------|----------------|
| **None** | 1.0x | Fastest | Fastest | None | Testing only |
| **Snappy** | 2-3x | Very Fast | Very Fast | Low | Good default |
| **LZ4** | 2-3x | Very Fast | Fastest | Very Low | Real-time streaming |
| **ZSTD** | 3-5x | Fast | Fast | Medium | **RECOMMENDED** |
| **GZIP** | 3-4x | Slow | Medium | High | Long-term storage |
| **Brotli** | 4-6x | Very Slow | Medium | High | Archival |

### 5.2 Selection Guidelines

```
Choose compression based on workload:

Real-time Streaming → LZ4
General Purpose    → ZSTD
Long-term Storage  → GZIP
Archival           → Brotli
```

### 5.3 Configuration Examples

```python
# Parquet with ZSTD
df.write.format("parquet") \
    .option("compression", "zstd") \
    .save("s3://lake/data/")

# ORC with Snappy
df.write.format("orc") \
    .option("compression", "snappy") \
    .save("s3://lake/data/")

# JSON with GZIP
df.write.format("json") \
    .option("compression", "gzip") \
    .save("s3://lake/data/")
```

---

## Schema Design

### 6.1 Data Types

#### 6.1.1 Primitive Types

| WIA Type | Parquet Type | ORC Type | Avro Type | Description |
|----------|--------------|----------|-----------|-------------|
| BOOLEAN | BOOLEAN | BOOLEAN | boolean | True/False |
| BYTE | INT32 (INT_8) | TINYINT | int | 8-bit signed |
| SHORT | INT32 (INT_16) | SMALLINT | int | 16-bit signed |
| INT | INT32 | INT | int | 32-bit signed |
| LONG | INT64 | BIGINT | long | 64-bit signed |
| FLOAT | FLOAT | FLOAT | float | 32-bit floating point |
| DOUBLE | DOUBLE | DOUBLE | double | 64-bit floating point |
| DECIMAL | DECIMAL | DECIMAL | bytes (logical) | Fixed precision |
| STRING | BINARY (UTF8) | STRING | string | UTF-8 text |
| BINARY | BINARY | BINARY | bytes | Raw bytes |
| DATE | INT32 (DATE) | DATE | int (logical) | Days since epoch |
| TIMESTAMP | INT64 (TIMESTAMP) | TIMESTAMP | long (logical) | Microseconds since epoch |

#### 6.1.2 Complex Types

| Type | Syntax | Example |
|------|--------|---------|
| **ARRAY** | ARRAY<type> | ARRAY<STRING> |
| **MAP** | MAP<key_type, value_type> | MAP<STRING, INT> |
| **STRUCT** | STRUCT<field1: type1, field2: type2> | STRUCT<name: STRING, age: INT> |

### 6.2 Schema Evolution

#### 6.2.1 Supported Operations

| Operation | Impact | Safety |
|-----------|--------|--------|
| Add column | Backward compatible | ✅ Safe |
| Remove column | Forward compatible | ⚠️ Use with caution |
| Rename column | Breaking change | ❌ Requires migration |
| Change type (widening) | Backward compatible | ✅ Safe (INT → LONG) |
| Change type (narrowing) | Breaking change | ❌ Data loss risk |

#### 6.2.2 Schema Evolution Example

```python
# Initial schema
schema_v1 = StructType([
    StructField("id", LongType(), False),
    StructField("name", StringType(), False),
    StructField("email", StringType(), True)
])

# Add column (safe)
schema_v2 = StructType([
    StructField("id", LongType(), False),
    StructField("name", StringType(), False),
    StructField("email", StringType(), True),
    StructField("phone", StringType(), True)  # New column
])

# Type widening (safe)
schema_v3 = StructType([
    StructField("id", LongType(), False),  # Was IntType
    StructField("name", StringType(), False),
    StructField("email", StringType(), True),
    StructField("phone", StringType(), True)
])
```

---

## Examples

### 7.1 Complete Data Lake Structure

```
s3://my-datalake/
├── raw/                          # Raw Zone
│   ├── events/
│   │   ├── year=2025/month=01/day=15/
│   │   │   └── events-20250115.json.gz
│   ├── logs/
│   │   └── date=2025-01-15/
│   │       └── logs.jsonl.gz
├── curated/                      # Curated Zone
│   ├── events/
│   │   ├── _delta_log/
│   │   ├── year=2025/month=01/day=15/
│   │   │   └── part-00000.snappy.parquet
│   ├── users/
│   │   ├── metadata/
│   │   └── data/
│   │       └── part-00000.zstd.parquet
└── consumption/                  # Consumption Zone
    ├── daily_metrics/
    │   └── date=2025-01-15/
    │       └── metrics.snappy.parquet
    └── user_profiles/
        └── version=v1/
            └── profiles.snappy.parquet
```

### 7.2 ETL Pipeline Example

```python
from pyspark.sql import SparkSession
from pyspark.sql.functions import col, to_date

spark = SparkSession.builder.getOrCreate()

# 1. Read from Raw (JSON)
raw_df = spark.read.json("s3://lake/raw/events/year=2025/month=01/day=15/")

# 2. Transform
curated_df = raw_df \
    .filter(col("event_type").isNotNull()) \
    .withColumn("event_date", to_date(col("timestamp"))) \
    .select("event_id", "user_id", "event_type", "event_date", "properties")

# 3. Write to Curated (Parquet + Delta Lake)
curated_df.write \
    .format("delta") \
    .mode("append") \
    .partitionBy("event_date") \
    .option("compression", "zstd") \
    .save("s3://lake/curated/events/")

# 4. Aggregate for Consumption
consumption_df = curated_df \
    .groupBy("event_date", "event_type") \
    .agg({"event_id": "count"}) \
    .withColumnRenamed("count(event_id)", "event_count")

# 5. Write to Consumption (Parquet)
consumption_df.write \
    .format("parquet") \
    .mode("overwrite") \
    .partitionBy("event_date") \
    .save("s3://lake/consumption/daily_metrics/")
```

---

## References

- [Apache Parquet Format Specification](https://parquet.apache.org/docs/)
- [Apache ORC Specification](https://orc.apache.org/specification/)
- [Apache Avro Specification](https://avro.apache.org/docs/current/spec.html)
- [Delta Lake Protocol](https://github.com/delta-io/delta/blob/master/PROTOCOL.md)
- [Apache Iceberg Spec](https://iceberg.apache.org/spec/)
- [Apache Hudi RFC](https://hudi.apache.org/docs/rfc)

---

**Next Phase**: [Phase 2 - API Interface Standard](PHASE-2-API.md)

---

© 2025 WIA Standards / MIT License
