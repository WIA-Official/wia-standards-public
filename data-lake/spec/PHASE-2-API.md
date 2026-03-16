# WIA Data Lake API Interface Standard
## Phase 2 Specification

---

**Version**: 1.0.0
**Status**: Complete
**Date**: 2025-12-26
**Authors**: WIA Standards Committee
**License**: MIT

---

## Table of Contents

1. [Overview](#overview)
2. [Core API Design](#core-api-design)
3. [WiaDataLake Class](#wiadatalake-class)
4. [Data Ingestion](#data-ingestion)
5. [Query Interface](#query-interface)
6. [Metadata Operations](#metadata-operations)
7. [Optimization](#optimization)
8. [Python SDK](#python-sdk)
9. [TypeScript SDK](#typescript-sdk)

---

## Overview

### 1.1 Purpose

The WIA Data Lake API Interface Standard defines a unified programming interface for interacting with WIA-compliant data lakes across all implementations.

**Core Objectives**:
- Provide consistent API across Python, TypeScript, Java, Rust
- Abstract underlying storage and compute complexities
- Enable rapid application development
- Support both batch and streaming workloads

### 1.2 Design Philosophy

```
Simplicity > Complexity
Consistency > Flexibility (where conflicts arise)
Explicit > Implicit
Composability > Monolithic operations
```

---

## Core API Design

### 2.1 API Structure

```
WiaDataLake (main class)
├── Connection Management
│   ├── __init__()
│   ├── connect()
│   └── disconnect()
├── Data Operations
│   ├── ingest()
│   ├── ingest_stream()
│   ├── query()
│   └── table()
├── Metadata Operations
│   ├── get_metadata()
│   ├── list_tables()
│   ├── add_column()
│   └── rename_column()
└── Optimization Operations
    ├── optimize()
    ├── vacuum()
    └── compact()
```

### 2.2 Configuration Model

```python
{
    "storage": {
        "path": "s3://my-lake/",
        "type": "s3",
        "region": "us-east-1"
    },
    "catalog": {
        "uri": "glue://us-east-1/my-catalog",
        "type": "glue"
    },
    "compute": {
        "engine": "spark",
        "cluster_id": "cluster-123"
    },
    "table_format": "delta",
    "compression": "zstd"
}
```

---

## WiaDataLake Class

### 3.1 Initialization

```python
class WiaDataLake:
    def __init__(self,
                 storage_path: str,
                 catalog_uri: str,
                 table_format: str = "delta",
                 config: Optional[Dict] = None):
        """
        Initialize Data Lake connection.

        Args:
            storage_path: S3/ADLS/GCS path (e.g., "s3://my-lake/")
            catalog_uri: Catalog URI (e.g., "glue://region/catalog")
            table_format: "delta", "iceberg", or "hudi"
            config: Additional configuration options
        """
```

### 3.2 Usage Example

```python
from wia_datalake import WiaDataLake

# Initialize
lake = WiaDataLake(
    storage_path="s3://my-datalake/",
    catalog_uri="glue://us-east-1/prod-catalog",
    table_format="delta",
    config={
        "spark.sql.adaptive.enabled": "true",
        "spark.sql.adaptive.coalescePartitions.enabled": "true"
    }
)
```

---

## Data Ingestion

### 4.1 Batch Ingestion

```python
def ingest(self,
           source: Union[str, DataFrame],
           target_zone: str,
           target_table: str,
           format: str = "parquet",
           partition_by: Optional[List[str]] = None,
           schema: Optional[Dict] = None,
           mode: str = "append",
           transform: Optional[Callable] = None,
           **options) -> IngestJob:
    """
    Ingest data into the lake.

    Args:
        source: Source path or DataFrame
        target_zone: "raw", "curated", or "consumption"
        target_table: Target table name
        format: Output format ("parquet", "delta", "iceberg", "hudi")
        partition_by: List of partition columns
        schema: Schema definition (optional)
        mode: "append", "overwrite", "error", "ignore"
        transform: Optional transformation function
        **options: Additional format-specific options

    Returns:
        IngestJob: Job handle for monitoring
    """
```

#### 4.1.1 Basic Example

```python
job = lake.ingest(
    source="s3://raw-bucket/events/*.json",
    target_zone="raw",
    target_table="events",
    format="parquet",
    mode="append"
)

print(f"Job ID: {job.job_id}")
print(f"Status: {job.status}")
print(f"Records: {job.records_processed}")
```

#### 4.1.2 Advanced Example with Schema

```python
schema = {
    "user_id": "bigint",
    "event_type": "string",
    "timestamp": "timestamp",
    "properties": "map<string,string>"
}

job = lake.ingest(
    source="s3://source/events/",
    target_zone="curated",
    target_table="events_processed",
    format="delta",
    partition_by=["year", "month", "day"],
    schema=schema,
    transform=lambda df: df.filter("event_type != 'test'"),
    mode="overwrite",
    mergeSchema="true"
)
```

### 4.2 Streaming Ingestion

```python
def ingest_stream(self,
                  source: str,
                  target_zone: str,
                  target_table: str,
                  format: str = "delta",
                  checkpoint_location: str = None,
                  trigger: str = "processingTime='10 seconds'",
                  **options) -> StreamJob:
    """
    Continuous data ingestion from stream.

    Args:
        source: Kafka/Kinesis URI
        target_zone: Target zone
        target_table: Target table
        format: Table format (must support streaming)
        checkpoint_location: Checkpoint path
        trigger: Trigger interval

    Returns:
        StreamJob: Streaming job handle
    """
```

#### 4.2.1 Kafka Example

```python
stream = lake.ingest_stream(
    source="kafka://localhost:9092/events",
    target_zone="curated",
    target_table="real_time_events",
    format="delta",
    checkpoint_location="s3://checkpoints/events/",
    trigger="processingTime='5 seconds'",
    kafka_options={
        "subscribe": "events",
        "startingOffsets": "latest"
    }
)

# Monitor stream
print(f"Stream ID: {stream.id}")
print(f"Status: {stream.status}")
stream.await_termination()
```

---

## Query Interface

### 5.1 SQL Query

```python
def query(self, sql: str, **options) -> DataFrame:
    """
    Execute SQL query.

    Args:
        sql: SQL query string
        **options: Query options (e.g., cache, explain)

    Returns:
        DataFrame: Query results
    """
```

#### 5.1.1 Example

```python
df = lake.query("""
    SELECT
        user_id,
        COUNT(*) as event_count,
        MAX(timestamp) as last_event
    FROM curated.events
    WHERE date = '2025-01-15'
    GROUP BY user_id
    ORDER BY event_count DESC
    LIMIT 100
""")

# Show results
df.show()

# Save to file
df.write.csv("results.csv")
```

### 5.2 DataFrame API

```python
def table(self, table_name: str) -> Table:
    """
    Get table handle for programmatic queries.

    Args:
        table_name: Fully qualified table name

    Returns:
        Table: Table object with DataFrame-like API
    """
```

#### 5.2.1 Example

```python
events = lake.table("curated.events")

result = events \
    .filter("date = '2025-01-15'") \
    .groupBy("user_id") \
    .agg({"*": "count"}) \
    .orderBy("count", ascending=False) \
    .limit(100)

result.show()
```

### 5.3 Time Travel

```python
# Delta Lake time travel
df = lake.query("""
    SELECT * FROM curated.events
    VERSION AS OF 10
""")

# Timestamp-based
df = lake.query("""
    SELECT * FROM curated.events
    TIMESTAMP AS OF '2025-01-15 10:00:00'
""")
```

---

## Metadata Operations

### 6.1 Get Table Metadata

```python
def get_metadata(self, table_name: str) -> TableMetadata:
    """
    Retrieve table metadata.

    Args:
        table_name: Table name

    Returns:
        TableMetadata: Metadata object
    """
```

#### 6.1.1 TableMetadata Class

```python
@dataclass
class TableMetadata:
    table_name: str
    location: str
    format: str
    schema: Dict[str, str]
    partitions: List[str]
    num_rows: int
    size_bytes: int
    created_at: datetime
    last_modified: datetime
    properties: Dict[str, Any]
```

#### 6.1.2 Example

```python
metadata = lake.get_metadata("curated.events")

print(f"Table: {metadata.table_name}")
print(f"Location: {metadata.location}")
print(f"Format: {metadata.format}")
print(f"Schema: {metadata.schema}")
print(f"Rows: {metadata.num_rows:,}")
print(f"Size: {metadata.size_bytes / 1e9:.2f} GB")
```

### 6.2 List Tables

```python
def list_tables(self, zone: Optional[str] = None) -> List[TableInfo]:
    """
    List all tables in the lake.

    Args:
        zone: Filter by zone ("raw", "curated", "consumption")

    Returns:
        List of TableInfo objects
    """
```

### 6.3 Schema Evolution

```python
def add_column(self,
               table: str,
               column_name: str,
               column_type: str,
               nullable: bool = True,
               comment: Optional[str] = None):
    """Add column to existing table."""

def rename_column(self,
                  table: str,
                  old_name: str,
                  new_name: str):
    """Rename column (supported by Iceberg)."""

def drop_column(self,
                table: str,
                column_name: str):
    """Drop column from table."""
```

---

## Optimization

### 7.1 Optimize Table

```python
def optimize(self,
             table: str,
             strategy: str = "compact",
             where: Optional[str] = None,
             zorder_by: Optional[List[str]] = None,
             **options) -> OptimizeResult:
    """
    Optimize table storage.

    Args:
        table: Table name
        strategy: "compact", "zorder", "vacuum"
        where: Partition filter
        zorder_by: Columns for Z-ordering

    Returns:
        OptimizeResult: Optimization statistics
    """
```

#### 7.1.1 Example

```python
# File compaction
result = lake.optimize(
    table="curated.events",
    strategy="compact",
    where="date = '2025-01-15'"
)

print(f"Files before: {result.files_before}")
print(f"Files after: {result.files_after}")
print(f"Space saved: {result.space_saved_mb:.2f} MB")

# Z-ordering
result = lake.optimize(
    table="curated.events",
    strategy="zorder",
    zorder_by=["user_id", "event_type"]
)
```

### 7.2 Vacuum

```python
def vacuum(self,
           table: str,
           retention_hours: int = 168,
           dry_run: bool = False) -> VacuumResult:
    """
    Remove old data files.

    Args:
        table: Table name
        retention_hours: Files older than this are deleted
        dry_run: Preview files to be deleted

    Returns:
        VacuumResult: Cleanup statistics
    """
```

---

## Python SDK

### 8.1 Installation

```bash
pip install wia-datalake
```

### 8.2 Complete Example

```python
from wia_datalake import WiaDataLake, IngestException

# Initialize
lake = WiaDataLake(
    storage_path="s3://my-datalake/",
    catalog_uri="glue://us-east-1/catalog"
)

# Ingest data
try:
    job = lake.ingest(
        source="s3://raw/events.json",
        target_zone="curated",
        target_table="events",
        format="delta",
        partition_by=["date"]
    )
    print(f"Ingested {job.records_processed} records")
except IngestException as e:
    print(f"Ingestion failed: {e}")

# Query
df = lake.query("SELECT * FROM curated.events LIMIT 10")
df.show()

# Optimize
result = lake.optimize("curated.events")
print(f"Optimization saved {result.space_saved_mb} MB")

# Vacuum
lake.vacuum("curated.events", retention_hours=168)
```

---

## TypeScript SDK

### 9.1 Installation

```bash
npm install @wia/datalake
```

### 9.2 Interface Definition

```typescript
interface WiaDataLakeConfig {
  storagePath: string;
  catalogUri: string;
  tableFormat?: 'delta' | 'iceberg' | 'hudi';
  config?: Record<string, any>;
}

class WiaDataLake {
  constructor(config: WiaDataLakeConfig);

  async ingest(options: IngestOptions): Promise<IngestJob>;
  async query(sql: string): Promise<DataFrame>;
  async getMetadata(tableName: string): Promise<TableMetadata>;
  async optimize(table: string, strategy: OptimizeStrategy): Promise<OptimizeResult>;
  async vacuum(table: string, retentionHours?: number): Promise<VacuumResult>;
}
```

### 9.3 Complete Example

```typescript
import { WiaDataLake } from '@wia/datalake';

// Initialize
const lake = new WiaDataLake({
  storagePath: 's3://my-datalake/',
  catalogUri: 'glue://us-east-1/catalog',
  tableFormat: 'delta'
});

// Ingest
const job = await lake.ingest({
  source: 's3://raw/events.json',
  targetZone: 'curated',
  targetTable: 'events',
  format: 'delta',
  partitionBy: ['date']
});

console.log(`Ingested ${job.recordsProcessed} records`);

// Query
const df = await lake.query(
  'SELECT * FROM curated.events LIMIT 10'
);

// Optimize
const result = await lake.optimize('curated.events', {
  strategy: 'compact'
});

console.log(`Saved ${result.spaceSavedMb} MB`);
```

---

**Next Phase**: [Phase 3 - Table Format Protocol](PHASE-3-PROTOCOL.md)

---

© 2025 WIA Standards / MIT License
