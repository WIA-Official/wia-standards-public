# WIA-DATA-004: PHASE 1 - DATA FORMAT

## Overview

This phase defines the standard data formats and schemas for data pipeline operations.

## 1. Standard Data Formats

### 1.1 Structured Formats

#### Parquet (Recommended)
- **Use Case:** Analytics, data warehousing
- **Advantages:** Columnar storage, excellent compression, schema evolution
- **File Extension:** `.parquet`

```python
# Writing Parquet
df.write.format("parquet").save("s3://bucket/data/")

# Reading Parquet
df = spark.read.parquet("s3://bucket/data/")
```

#### Avro
- **Use Case:** Streaming, schema evolution
- **Advantages:** Compact binary format, schema embedded
- **File Extension:** `.avro`

```json
{
  "type": "record",
  "name": "UserEvent",
  "fields": [
    {"name": "user_id", "type": "long"},
    {"name": "event_type", "type": "string"},
    {"name": "timestamp", "type": "long"},
    {"name": "properties", "type": {"type": "map", "values": "string"}}
  ]
}
```

#### ORC
- **Use Case:** Hive, big data processing
- **Advantages:** High compression, predicate pushdown

### 1.2 Semi-Structured Formats

#### JSON
```json
{
  "pipeline_id": "dp-20251226-001",
  "source": "postgresql",
  "destination": "snowflake",
  "records_processed": 127500,
  "start_time": "2025-12-26T10:00:00Z",
  "end_time": "2025-12-26T10:15:30Z",
  "status": "completed"
}
```

#### CSV
- Header row required
- UTF-8 encoding
- Comma delimiter (`,`)
- Quote character (`"`)
- Escape character (`\`)

## 2. Pipeline Metadata Schema

### 2.1 Pipeline Configuration

```json
{
  "pipeline": {
    "id": "string",
    "name": "string",
    "version": "string",
    "owner": "string",
    "schedule": "cron_expression",
    "enabled": boolean,
    "retry_policy": {
      "max_retries": integer,
      "retry_delay_seconds": integer,
      "backoff_multiplier": float
    },
    "timeout_seconds": integer,
    "tags": ["string"]
  }
}
```

### 2.2 Data Source Schema

```json
{
  "source": {
    "type": "enum(database|api|file|stream)",
    "connection": {
      "host": "string",
      "port": integer,
      "database": "string",
      "schema": "string",
      "table": "string"
    },
    "authentication": {
      "type": "enum(basic|oauth|token|key)",
      "credentials_secret": "string"
    },
    "extraction_mode": "enum(full|incremental|cdc)",
    "incremental_key": "string",
    "watermark": "timestamp"
  }
}
```

### 2.3 Transformation Schema

```json
{
  "transformations": [
    {
      "step_id": "string",
      "type": "enum(sql|python|spark)",
      "operation": "enum(clean|normalize|enrich|aggregate|filter)",
      "config": {
        "sql": "string",
        "python_function": "string",
        "dependencies": ["string"]
      }
    }
  ]
}
```

### 2.4 Data Quality Schema

```json
{
  "quality_checks": [
    {
      "check_id": "string",
      "column": "string",
      "check_type": "enum(not_null|unique|range|regex|custom)",
      "params": {},
      "severity": "enum(blocker|critical|warning)",
      "action": "enum(fail|quarantine|log)"
    }
  ]
}
```

## 3. Event Log Schema

### 3.1 Pipeline Execution Log

```json
{
  "execution_id": "uuid",
  "pipeline_id": "string",
  "start_time": "timestamp",
  "end_time": "timestamp",
  "status": "enum(running|completed|failed|cancelled)",
  "trigger_type": "enum(scheduled|manual|event)",
  "records_extracted": integer,
  "records_transformed": integer,
  "records_loaded": integer,
  "records_failed": integer,
  "duration_seconds": float,
  "error_message": "string",
  "metadata": {}
}
```

### 3.2 Data Lineage

```json
{
  "lineage": {
    "dataset_id": "string",
    "upstream_datasets": [
      {
        "dataset_id": "string",
        "transformation": "string",
        "timestamp": "timestamp"
      }
    ],
    "downstream_datasets": [
      {
        "dataset_id": "string",
        "consumer": "string"
      }
    ]
  }
}
```

## 4. Data Partitioning Strategy

### 4.1 Time-Based Partitioning

```
s3://bucket/data/
  └── table_name/
      └── date=2025-12-26/
          └── hour=10/
              └── data.parquet
```

### 4.2 Hash-Based Partitioning

```python
# Partition by user_id hash
df.write.partitionBy("user_id_hash").parquet("s3://bucket/users/")
```

## 5. Naming Conventions

### 5.1 Dataset Names
- Format: `{domain}_{entity}_{type}`
- Example: `sales_orders_raw`, `users_profile_clean`

### 5.2 Column Names
- Lowercase with underscores
- Example: `user_id`, `created_at`, `order_total`

### 5.3 File Names
- Format: `{dataset}_{date}_{partition}.{extension}`
- Example: `orders_20251226_00.parquet`

## 6. Schema Evolution

### 6.1 Backward Compatible Changes (Allowed)
- Adding nullable columns
- Removing columns
- Widening column types (e.g., int32 → int64)

### 6.2 Incompatible Changes (Requires Version Bump)
- Removing non-nullable columns
- Changing column types
- Renaming columns

## 7. Compression Standards

### 7.1 Recommended Compression

| Format  | Compression | Ratio | Speed |
|---------|-------------|-------|-------|
| Parquet | Snappy      | 4:1   | Fast  |
| Parquet | GZIP        | 8:1   | Slow  |
| Avro    | Snappy      | 3:1   | Fast  |
| CSV     | GZIP        | 10:1  | Slow  |

---

**Status:** ✅ Complete
**Version:** 1.0.0
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) · Benefit All Humanity
