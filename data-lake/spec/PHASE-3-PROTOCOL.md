# WIA Data Lake Table Format Protocol
## Phase 3 Specification

---

**Version**: 1.0.0
**Status**: Complete
**Date**: 2025-12-26
**Authors**: WIA Standards Committee
**License**: MIT

---

## Table of Contents

1. [Overview](#overview)
2. [Delta Lake Protocol](#delta-lake-protocol)
3. [Apache Iceberg Protocol](#apache-iceberg-protocol)
4. [Apache Hudi Protocol](#apache-hudi-protocol)
5. [ACID Semantics](#acid-semantics)
6. [Schema Evolution](#schema-evolution)
7. [Time Travel](#time-travel)
8. [Comparison Matrix](#comparison-matrix)

---

## Overview

### 1.1 Purpose

Phase 3 defines the table format protocols that enable ACID transactions, schema evolution, and time travel on data lakes.

**Supported Table Formats**:
- Delta Lake 2.4.0+
- Apache Iceberg 1.4.0+
- Apache Hudi 0.14.0+

### 1.2 Why Table Formats Matter

Traditional data lakes lack:
- ❌ ACID transactions
- ❌ Efficient updates/deletes
- ❌ Schema enforcement
- ❌ Time travel capabilities

Table formats provide:
- ✅ ACID guarantees
- ✅ Efficient MERGE/UPDATE/DELETE
- ✅ Schema evolution
- ✅ Snapshot isolation
- ✅ Time travel & auditing

---

## Delta Lake Protocol

### 2.1 Architecture

```
Delta Table Structure:
s3://lake/events/
├── _delta_log/
│   ├── 00000000000000000000.json    # Version 0
│   ├── 00000000000000000001.json    # Version 1
│   ├── 00000000000000000002.json    # Version 2
│   ├── _last_checkpoint              # Optimization
│   └── 00000000000000000010.checkpoint.parquet
├── part-00000-v1.snappy.parquet
├── part-00001-v1.snappy.parquet
└── part-00002-v2.snappy.parquet
```

### 2.2 Transaction Log Format

```json
{
  "commitInfo": {
    "timestamp": 1705320000000,
    "operation": "WRITE",
    "operationParameters": {
      "mode": "Append",
      "partitionBy": "[\"date\"]"
    },
    "operationMetrics": {
      "numFiles": "2",
      "numOutputBytes": "268435456",
      "numOutputRows": "1000000"
    },
    "version": 1
  },
  "add": {
    "path": "part-00000-v1.snappy.parquet",
    "size": 134217728,
    "modificationTime": 1705320000000,
    "dataChange": true,
    "partitionValues": {
      "date": "2025-01-15"
    },
    "stats": "{\"numRecords\":500000,\"minValues\":{\"id\":1},\"maxValues\":{\"id\":500000},\"nullCount\":{\"id\":0}}"
  }
}
```

### 2.3 Core Operations

```sql
-- Create Delta table
CREATE TABLE events (
  id BIGINT,
  user_id BIGINT,
  event_type STRING,
  timestamp TIMESTAMP,
  date DATE
)
USING delta
PARTITIONED BY (date)
LOCATION 's3://lake/events/';

-- Insert
INSERT INTO events VALUES (1, 100, 'click', now(), current_date());

-- Update
UPDATE events SET event_type = 'purchase' WHERE id = 1;

-- Delete
DELETE FROM events WHERE date < '2024-01-01';

-- Merge (Upsert)
MERGE INTO target
USING source
ON target.id = source.id
WHEN MATCHED THEN UPDATE SET *
WHEN NOT MATCHED THEN INSERT *;

-- Time travel
SELECT * FROM events VERSION AS OF 5;
SELECT * FROM events TIMESTAMP AS OF '2025-01-15 10:00:00';

-- Optimize
OPTIMIZE events WHERE date = '2025-01-15';
OPTIMIZE events ZORDER BY (user_id, event_type);

-- Vacuum
VACUUM events RETAIN 168 HOURS;
```

### 2.4 Checkpoint Mechanism

```
Every 10 commits, Delta creates a checkpoint:

00000000000000000000.json
00000000000000000001.json
...
00000000000000000009.json
00000000000000000010.checkpoint.parquet  ← Checkpoint

Checkpoint contains:
- All add/remove file actions
- Protocol version
- Metadata
- Enables fast state reconstruction
```

---

## Apache Iceberg Protocol

### 3.1 Architecture

```
Iceberg Table Structure:
s3://lake/events/
├── metadata/
│   ├── v1.metadata.json
│   ├── v2.metadata.json
│   ├── v3.metadata.json
│   ├── snap-8120873689437310534.avro
│   ├── 5b12cd81-cd58-4e34-8a59-a3e6e5e8f4b3-m0.avro
│   └── manifest-list-file-v3.avro
└── data/
    ├── 00001-1-85a89f93-ee5b-440e-a1d8-6e73ca8e7e8f.parquet
    └── 00002-2-a4c1c5f3-5a2b-4b8c-9e7f-3d2e1c4a5b6c.parquet
```

### 3.2 Metadata JSON Format

```json
{
  "format-version": 2,
  "table-uuid": "9c12d441-03fe-4693-9a96-a0705ddf69c1",
  "location": "s3://lake/events",
  "last-updated-ms": 1705320000000,
  "last-column-id": 4,
  "current-snapshot-id": 8120873689437310534,
  "schemas": [
    {
      "schema-id": 0,
      "fields": [
        {"id": 1, "name": "id", "required": true, "type": "long"},
        {"id": 2, "name": "event_type", "required": false, "type": "string"},
        {"id": 3, "name": "timestamp", "required": false, "type": "timestamp"},
        {"id": 4, "name": "date", "required": false, "type": "date"}
      ]
    }
  ],
  "partition-specs": [
    {
      "spec-id": 0,
      "fields": [
        {"name": "date", "transform": "day", "source-id": 4, "field-id": 1000}
      ]
    }
  ],
  "properties": {
    "write.format.default": "parquet",
    "write.parquet.compression-codec": "zstd"
  }
}
```

### 3.3 Hidden Partitioning

```sql
-- Create table with hidden partitioning
CREATE TABLE events (
  id BIGINT,
  event_type STRING,
  timestamp TIMESTAMP
)
USING iceberg
PARTITIONED BY (days(timestamp));

-- Query WITHOUT specifying partition (Iceberg handles it)
SELECT * FROM events WHERE timestamp = '2025-01-15';
-- Iceberg automatically prunes to date=2025-01-15 partition
```

### 3.4 Partition Evolution

```sql
-- Initial partitioning: Daily
CREATE TABLE events PARTITIONED BY (days(timestamp));

-- Later: Change to hourly WITHOUT rewriting data
ALTER TABLE events SET PARTITION SPEC (hours(timestamp));

-- Old data remains in daily partitions
-- New data written to hourly partitions
-- Queries work seamlessly across both!
```

### 3.5 Snapshot Isolation

```
Concurrent Operations:

Writer 1: START TRANSACTION
Writer 1: INSERT 1M rows
...
Reader 1: SELECT * (sees pre-insert snapshot)
...
Writer 1: COMMIT (creates snapshot S2)
Reader 2: SELECT * (sees post-insert snapshot S2)

Each snapshot is immutable and independent.
```

---

## Apache Hudi Protocol

### 4.1 Table Types

| Type | Write Latency | Read Latency | Use Case |
|------|---------------|--------------|----------|
| **Copy-On-Write (COW)** | Higher | Lower | Read-heavy |
| **Merge-On-Read (MOR)** | Lower | Higher | Write-heavy |

### 4.2 Architecture

```
Hudi Table Structure:
s3://lake/events/
├── .hoodie/
│   ├── 20250115100000.commit
│   ├── 20250115103000.deltacommit
│   ├── 20250115110000.compaction
│   ├── archived/
│   └── hoodie.properties
├── 2025/01/15/
│   ├── 8e8ef1e0-5c1e-4b3d-a4e1-8c5e7b3d2a1f-0_0-1-0_20250115100000.parquet
│   └── 8e8ef1e0-5c1e-4b3d-a4e1-8c5e7b3d2a1f-0_0-1-0_20250115100000.log.1_0-2-0
```

### 4.3 Timeline Service

```
Hudi Timeline (ordered commits):

20250115100000.commit       → Initial write
20250115103000.deltacommit  → Incremental update (MOR)
20250115110000.compaction   → Merge delta → base (MOR)
20250115120000.commit       → Another write
```

### 4.4 Incremental Queries

```python
# Read only changes since last checkpoint
df = spark.read.format("hudi") \
    .option("hoodie.datasource.query.type", "incremental") \
    .option("hoodie.datasource.read.begin.instanttime", "20250115100000") \
    .option("hoodie.datasource.read.end.instanttime", "20250115120000") \
    .load("s3://lake/events/")

# Only records added/updated between timestamps returned
```

### 4.5 Upsert Operation

```python
# Upsert (update + insert)
df.write.format("hudi") \
    .option("hoodie.table.name", "events") \
    .option("hoodie.datasource.write.operation", "upsert") \
    .option("hoodie.datasource.write.recordkey.field", "id") \
    .option("hoodie.datasource.write.precombine.field", "timestamp") \
    .mode("append") \
    .save("s3://lake/events/")
```

---

## ACID Semantics

### 5.1 Atomicity

```sql
-- All-or-nothing guarantee
BEGIN TRANSACTION;
  INSERT INTO events VALUES (...);  -- 1 million rows
  UPDATE events SET processed = true WHERE date = '2025-01-15';
  DELETE FROM events WHERE date < '2024-01-01';
COMMIT;

-- If ANY operation fails:
-- - Entire transaction rolls back
-- - No partial changes visible
-- - Transaction log remains consistent
```

### 5.2 Consistency

```sql
-- Schema enforcement prevents invalid data
CREATE TABLE events (
  id BIGINT NOT NULL,
  event_type STRING,
  timestamp TIMESTAMP
) USING delta;

-- Valid insert
INSERT INTO events VALUES (1, 'click', '2025-01-15 10:00:00');
-- ✅ Success

-- Invalid insert (type mismatch)
INSERT INTO events VALUES ('abc', 'click', 'not-a-date');
-- ❌ Error: Cannot cast 'abc' to BIGINT
```

### 5.3 Isolation

```
Snapshot Isolation Levels:

Reader at T1: Sees snapshot S1
Writer commits at T2: Creates snapshot S2
Reader at T3: Sees snapshot S2

No dirty reads, no phantom reads.
Each reader sees a consistent snapshot.
```

### 5.4 Durability

```
Once committed, data persists:

1. Transaction writes to log
2. Log entry becomes durable
3. Even if cluster crashes immediately after commit,
   data is recoverable from transaction log
```

---

## Schema Evolution

### 6.1 Supported Operations

| Operation | Delta | Iceberg | Hudi | Backward Compatible |
|-----------|-------|---------|------|---------------------|
| Add column | ✅ | ✅ | ✅ | ✅ Yes |
| Remove column | ✅ | ✅ | ✅ | ⚠️ Conditional |
| Rename column | ❌ View | ✅ Native | ❌ View | ❌ No |
| Widen type (INT→LONG) | ✅ | ✅ | ✅ | ✅ Yes |
| Narrow type (LONG→INT) | ❌ | ❌ | ❌ | ❌ No |
| Change nullable | ✅ | ✅ | ✅ | ⚠️ Conditional |

### 6.2 Examples

```sql
-- Add column (safe)
ALTER TABLE events ADD COLUMNS (session_id STRING);

-- Widen type (safe)
ALTER TABLE events ALTER COLUMN user_id SET DATA TYPE BIGINT;

-- Rename (Iceberg only)
ALTER TABLE events RENAME COLUMN name TO full_name;

-- Delta Lake workaround for rename (use view)
CREATE OR REPLACE VIEW events_legacy AS
  SELECT id, full_name AS name FROM events;
```

---

## Time Travel

### 7.1 Delta Lake Time Travel

```sql
-- By version
SELECT * FROM events VERSION AS OF 5;

-- By timestamp
SELECT * FROM events TIMESTAMP AS OF '2025-01-15 10:00:00';

-- Describe history
DESCRIBE HISTORY events LIMIT 10;

-- Restore to previous version
RESTORE TABLE events TO VERSION AS OF 3;
```

### 7.2 Iceberg Time Travel

```sql
-- By snapshot ID
SELECT * FROM events.snapshots WHERE snapshot_id = 8120873689437310534;

-- By timestamp
SELECT * FROM events FOR SYSTEM_TIME AS OF '2025-01-15 10:00:00';

-- List snapshots
SELECT * FROM events.history;

-- Rollback to snapshot
CALL iceberg.system.rollback_to_snapshot('events', 8120873689437310534);
```

### 7.3 Hudi Time Travel

```python
# Read as of instant time
df = spark.read.format("hudi") \
    .option("as.of.instant", "20250115100000") \
    .load("s3://lake/events/")
```

---

## Comparison Matrix

### 8.1 Feature Comparison

| Feature | Delta Lake | Iceberg | Hudi |
|---------|------------|---------|------|
| **ACID** | ✅ Full | ✅ Full | ✅ Full |
| **Time Travel** | ✅ Excellent | ✅ Excellent | ✅ Good |
| **Schema Evolution** | ✅ Good | ✅ Excellent | ✅ Good |
| **Partition Evolution** | ❌ No | ✅ Yes | ❌ No |
| **Hidden Partitioning** | ❌ No | ✅ Yes | ❌ No |
| **Incremental Reads** | ✅ CDF | ✅ Snapshots | ✅✅ Native |
| **Streaming** | ✅✅ Excellent | ✅ Good | ✅✅ Excellent |
| **Upserts** | ✅ MERGE | ✅ MERGE | ✅✅ Native |
| **Z-Ordering** | ✅ Built-in | ⚠️ Manual | ⚠️ Manual |
| **Multi-Engine Support** | ✅ Good | ✅✅ Excellent | ✅ Good |
| **Maturity** | ✅✅ Very Mature | ✅ Mature | ✅ Mature |

### 8.2 When to Use Each

**Use Delta Lake when**:
- Using Databricks platform
- Need excellent streaming support
- Want built-in optimization (Z-order)
- Prioritize ecosystem maturity

**Use Apache Iceberg when**:
- Need multi-engine support (Spark, Flink, Presto, Trino, Hive)
- Require partition evolution
- Want hidden partitioning
- Building cloud-agnostic platform

**Use Apache Hudi when**:
- Need incremental processing (CDC)
- Require frequent upserts
- Want MOR for low-latency writes
- Building real-time data pipelines

---

**Next Phase**: [Phase 4 - Ecosystem Integration](PHASE-4-INTEGRATION.md)

---

© 2025 WIA Standards / MIT License
