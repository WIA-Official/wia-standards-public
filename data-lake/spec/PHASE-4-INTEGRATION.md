# WIA Data Lake Ecosystem Integration
## Phase 4 Specification

---

**Version**: 1.0.0
**Status**: Complete
**Date**: 2025-12-26
**Authors**: WIA Standards Committee
**License**: MIT

---

## Table of Contents

1. [Overview](#overview)
2. [Compute Engine Integration](#compute-engine-integration)
3. [Metadata Catalog Integration](#metadata-catalog-integration)
4. [Cloud Platform Integration](#cloud-platform-integration)
5. [Streaming Integration](#streaming-integration)
6. [BI Tool Integration](#bi-tool-integration)
7. [Data Governance](#data-governance)
8. [Monitoring & Observability](#monitoring--observability)

---

## Overview

### 1.1 Purpose

Phase 4 defines how WIA-compliant data lakes integrate with the broader data ecosystem including compute engines, catalogs, cloud platforms, streaming systems, BI tools, and governance frameworks.

**Integration Categories**:
- Compute engines (Spark, Presto, Trino, Hive, Flink)
- Metadata catalogs (Glue, Unity, Hive Metastore, Nessie)
- Cloud platforms (AWS, Azure, GCP)
- Streaming systems (Kafka, Kinesis, Pulsar)
- BI tools (Tableau, Power BI, Looker)
- Governance (Lineage, Access Control, Audit)

### 1.2 Design Principles

- **Vendor Neutrality**: Work across all major cloud providers
- **Open Standards**: Use open protocols and APIs where possible
- **Interoperability**: Seamless integration between components
- **Backward Compatibility**: Support legacy systems during migration

---

## Compute Engine Integration

### 2.1 Apache Spark

#### 2.1.1 Delta Lake Integration

```python
from pyspark.sql import SparkSession

# Initialize Spark with Delta Lake
spark = SparkSession.builder \
    .appName("WIA Data Lake") \
    .config("spark.sql.extensions", "io.delta.sql.DeltaSparkSessionExtension") \
    .config("spark.sql.catalog.spark_catalog", "org.apache.spark.sql.delta.catalog.DeltaCatalog") \
    .config("spark.databricks.delta.retentionDurationCheck.enabled", "false") \
    .getOrCreate()

# Read Delta table
df = spark.read.format("delta").load("s3://lake/curated/events/")

# Write Delta table
df.write.format("delta") \
    .mode("append") \
    .option("mergeSchema", "true") \
    .save("s3://lake/curated/events/")

# Delta Lake SQL operations
spark.sql("OPTIMIZE curated.events ZORDER BY (user_id, event_type)")
spark.sql("VACUUM curated.events RETAIN 168 HOURS")
```

#### 2.1.2 Iceberg Integration

```python
# Initialize Spark with Iceberg
spark = SparkSession.builder \
    .config("spark.sql.extensions", "org.apache.iceberg.spark.extensions.IcebergSparkSessionExtensions") \
    .config("spark.sql.catalog.iceberg", "org.apache.iceberg.spark.SparkCatalog") \
    .config("spark.sql.catalog.iceberg.type", "hive") \
    .config("spark.sql.catalog.iceberg.uri", "thrift://metastore:9083") \
    .getOrCreate()

# Read/Write Iceberg table
df = spark.read.format("iceberg").load("iceberg.db.events")
df.write.format("iceberg").mode("append").save("iceberg.db.events")
```

#### 2.1.3 Hudi Integration

```python
# Initialize Spark with Hudi
spark = SparkSession.builder \
    .config("spark.serializer", "org.apache.spark.serializer.KryoSerializer") \
    .config("spark.sql.catalog.spark_catalog", "org.apache.spark.sql.hudi.catalog.HoodieCatalog") \
    .getOrCreate()

# Hudi write options
hudi_options = {
    'hoodie.table.name': 'events',
    'hoodie.datasource.write.recordkey.field': 'id',
    'hoodie.datasource.write.precombine.field': 'timestamp',
    'hoodie.datasource.write.partitionpath.field': 'date',
    'hoodie.datasource.write.operation': 'upsert',
    'hoodie.datasource.write.table.type': 'COPY_ON_WRITE'
}

# Write Hudi table
df.write.format("hudi") \
    .options(**hudi_options) \
    .mode("append") \
    .save("s3://lake/curated/events/")
```

### 2.2 Presto / Trino

#### 2.2.1 Catalog Configuration

```properties
# catalog/delta.properties
connector.name=delta_lake
hive.metastore.uri=thrift://metastore:9083
delta.register-table-procedure.enabled=true

# catalog/iceberg.properties
connector.name=iceberg
hive.metastore.uri=thrift://metastore:9083
iceberg.catalog.type=hive_metastore

# catalog/hudi.properties
connector.name=hudi
hive.metastore.uri=thrift://metastore:9083
```

#### 2.2.2 Query Examples

```sql
-- Query Delta table via Trino
SELECT * FROM delta.curated.events
WHERE date = '2025-01-15'
LIMIT 100;

-- Query Iceberg table
SELECT user_id, COUNT(*) as event_count
FROM iceberg.analytics.events
GROUP BY user_id
ORDER BY event_count DESC;

-- Join across catalogs
SELECT
    d.user_id,
    d.event_count,
    i.user_name
FROM delta.curated.events d
JOIN iceberg.analytics.users i ON d.user_id = i.id;
```

### 2.3 Apache Flink

#### 2.3.1 Iceberg Integration

```java
// Flink + Iceberg for real-time processing
StreamExecutionEnvironment env = StreamExecutionEnvironment.getExecutionEnvironment();
StreamTableEnvironment tableEnv = StreamTableEnvironment.create(env);

// Create Iceberg catalog
tableEnv.executeSql(
    "CREATE CATALOG iceberg_catalog WITH (\n" +
    "  'type'='iceberg',\n" +
    "  'catalog-type'='hive',\n" +
    "  'uri'='thrift://metastore:9083',\n" +
    "  'warehouse'='s3://lake/'\n" +
    ")"
);

// Read from Kafka, write to Iceberg
tableEnv.executeSql(
    "INSERT INTO iceberg_catalog.db.events\n" +
    "SELECT * FROM kafka_source"
);
```

---

## Metadata Catalog Integration

### 3.1 AWS Glue Catalog

#### 3.1.1 Setup

```python
import boto3

glue = boto3.client('glue', region_name='us-east-1')

# Create database
glue.create_database(
    DatabaseInput={
        'Name': 'datalake',
        'Description': 'WIA Data Lake'
    }
)

# Register Delta table
glue.create_table(
    DatabaseName='datalake',
    TableInput={
        'Name': 'events',
        'StorageDescriptor': {
            'Location': 's3://lake/curated/events/',
            'InputFormat': 'org.apache.hadoop.hive.ql.io.parquet.MapredParquetInputFormat',
            'OutputFormat': 'org.apache.hadoop.hive.ql.io.parquet.MapredParquetOutputFormat',
            'SerdeInfo': {
                'SerializationLibrary': 'org.apache.hadoop.hive.ql.io.parquet.serde.ParquetHiveSerDe'
            },
            'Columns': [
                {'Name': 'id', 'Type': 'bigint'},
                {'Name': 'event_type', 'Type': 'string'},
                {'Name': 'timestamp', 'Type': 'timestamp'}
            ]
        },
        'PartitionKeys': [
            {'Name': 'date', 'Type': 'string'}
        ],
        'Parameters': {
            'table_type': 'DELTA',
            'delta.minReaderVersion': '1',
            'delta.minWriterVersion': '2'
        }
    }
)
```

#### 3.1.2 Spark + Glue Integration

```python
spark = SparkSession.builder \
    .config("hive.metastore.client.factory.class", "com.amazonaws.glue.catalog.metastore.AWSGlueDataCatalogHiveClientFactory") \
    .enableHiveSupport() \
    .getOrCreate()

# Tables automatically registered in Glue
df.write.format("delta") \
    .mode("append") \
    .saveAsTable("datalake.events")
```

### 3.2 Databricks Unity Catalog

#### 3.2.1 Catalog Hierarchy

```
Unity Catalog Structure:
catalog (e.g., prod_catalog)
├── schema (e.g., curated)
│   ├── table (e.g., events)
│   ├── view (e.g., active_users)
│   └── function (e.g., mask_email)
└── schema (e.g., consumption)
    └── table (e.g., metrics)
```

#### 3.2.2 Setup

```sql
-- Create catalog
CREATE CATALOG IF NOT EXISTS wia_datalake;

-- Create schema
CREATE SCHEMA IF NOT EXISTS wia_datalake.curated;

-- Create Delta table
CREATE TABLE wia_datalake.curated.events (
  id BIGINT,
  event_type STRING,
  timestamp TIMESTAMP,
  date DATE
)
USING delta
PARTITIONED BY (date)
LOCATION 's3://lake/curated/events/';

-- Grant permissions
GRANT SELECT ON TABLE wia_datalake.curated.events TO analytics_team;
GRANT MODIFY ON TABLE wia_datalake.curated.events TO data_engineers;
```

### 3.3 Project Nessie (Git-like Catalog)

#### 3.3.1 Setup

```python
from pynessie import init

# Initialize Nessie client
catalog = init("http://nessie:19120/api/v1")

# Create branch (like git branch)
catalog.create_branch("dev", from_ref="main")

# Commit table to branch
catalog.commit(
    branch="dev",
    message="Add events table",
    operations=[
        # Table operations
    ]
)

# Merge to main (like git merge)
catalog.merge(from_ref="dev", into_ref="main")

# List branches
branches = catalog.list_references()
for branch in branches:
    print(f"Branch: {branch.name}, Hash: {branch.hash}")
```

---

## Cloud Platform Integration

### 4.1 AWS

| Service | Purpose | Integration Point |
|---------|---------|-------------------|
| **S3** | Storage | Primary data lake storage |
| **Glue** | Catalog & ETL | Metadata management |
| **Athena** | Query engine | SQL queries on lake data |
| **EMR** | Spark cluster | Big data processing |
| **Kinesis** | Streaming | Real-time ingestion |
| **Lake Formation** | Governance | Security & access control |

#### 4.1.1 Complete AWS Stack

```yaml
# AWS Data Lake Architecture
AWSTemplateFormatVersion: '2010-09-09'
Resources:
  DataLakeBucket:
    Type: AWS::S3::Bucket
    Properties:
      BucketName: my-datalake
      VersioningConfiguration:
        Status: Enabled
      LifecycleConfiguration:
        Rules:
          - Id: TierToGlacier
            Status: Enabled
            Transitions:
              - Days: 90
                StorageClass: GLACIER_IR

  GlueCatalog:
    Type: AWS::Glue::Database
    Properties:
      CatalogId: !Ref AWS::AccountId
      DatabaseInput:
        Name: datalake

  EMRCluster:
    Type: AWS::EMR::Cluster
    Properties:
      Name: DataLakeCluster
      ReleaseLabel: emr-6.10.0
      Applications:
        - Name: Spark
        - Name: Hive
      Instances:
        MasterInstanceGroup:
          InstanceCount: 1
          InstanceType: m5.xlarge
        CoreInstanceGroup:
          InstanceCount: 2
          InstanceType: m5.xlarge
```

### 4.2 Azure

| Service | Purpose | Integration Point |
|---------|---------|-------------------|
| **ADLS Gen2** | Storage | Primary data lake storage |
| **Synapse Analytics** | Analytics platform | Unified analytics workspace |
| **Databricks** | Spark platform | Delta Lake lakehouse |
| **Data Factory** | ETL | Data pipeline orchestration |
| **Event Hubs** | Streaming | Real-time ingestion |
| **Purview** | Governance | Metadata & lineage |

### 4.3 GCP

| Service | Purpose | Integration Point |
|---------|---------|-------------------|
| **Cloud Storage** | Storage | Primary data lake storage |
| **BigQuery** | Data warehouse | Query external tables |
| **Dataproc** | Spark cluster | Big data processing |
| **Dataflow** | Stream processing | Apache Beam pipelines |
| **Pub/Sub** | Messaging | Event-driven ingestion |
| **Data Catalog** | Metadata | Data discovery |

---

## Streaming Integration

### 5.1 Apache Kafka

```python
from pyspark.sql import SparkSession
from pyspark.sql.functions import from_json, col

spark = SparkSession.builder.getOrCreate()

# Define schema
schema = "id BIGINT, event_type STRING, timestamp TIMESTAMP"

# Read from Kafka
df = spark.readStream \
    .format("kafka") \
    .option("kafka.bootstrap.servers", "localhost:9092") \
    .option("subscribe", "events") \
    .option("startingOffsets", "latest") \
    .load()

# Parse JSON
parsed_df = df \
    .selectExpr("CAST(value AS STRING) as json") \
    .select(from_json(col("json"), schema).alias("data")) \
    .select("data.*")

# Write to Delta Lake
query = parsed_df.writeStream \
    .format("delta") \
    .option("checkpointLocation", "s3://checkpoints/events/") \
    .trigger(processingTime="10 seconds") \
    .start("s3://lake/curated/events/")

query.awaitTermination()
```

### 5.2 AWS Kinesis

```python
# Read from Kinesis
df = spark.readStream \
    .format("kinesis") \
    .option("streamName", "events-stream") \
    .option("region", "us-east-1") \
    .option("initialPosition", "latest") \
    .option("awsAccessKeyId", "ACCESS_KEY") \
    .option("awsSecretKey", "SECRET_KEY") \
    .load()

# Write to Delta
df.writeStream \
    .format("delta") \
    .option("checkpointLocation", "s3://checkpoints/") \
    .start("s3://lake/curated/events/")
```

---

## BI Tool Integration

### 6.1 Tableau

```
Connection Configuration:
1. Install Spark JDBC driver
2. Server: spark-thrift-server.example.com
3. Port: 10000
4. Type: Spark SQL
5. Database: curated
6. Authentication: Username/Password or Kerberos

Tables appear as standard SQL tables in Tableau.
```

### 6.2 Power BI

```
Connection Steps:
1. Get Data → Azure → Azure Synapse Analytics
2. Server: workspace.sql.azuresynapse.net
3. Database: curated
4. Mode: DirectQuery (recommended) or Import
5. Delta/Iceberg tables appear as views
```

### 6.3 Apache Superset

```python
# Database connection string
{
  "database_name": "WIA Data Lake",
  "sqlalchemy_uri": "trino://user@trino:8080/iceberg/curated",
  "expose_in_sqllab": true,
  "allow_ctas": true,
  "allow_cvas": true,
  "extra": {
    "metadata_params": {},
    "engine_params": {},
    "metadata_cache_timeout": {},
    "schemas_allowed_for_csv_upload": []
  }
}
```

---

## Data Governance

### 7.1 Data Lineage (OpenLineage)

```python
from openlineage.spark import setup_openlineage

# Setup lineage tracking
setup_openlineage(
    spark,
    transport="http://lineage-api:5000/api/v1/lineage",
    namespace="production"
)

# Lineage automatically tracked for:
# - Reads (source datasets)
# - Transforms (Spark jobs)
# - Writes (output datasets)
```

### 7.2 Access Control

```sql
-- Table-level permissions (Unity Catalog)
GRANT SELECT ON TABLE curated.events TO analytics_team;
GRANT MODIFY ON TABLE curated.events TO data_engineers;
REVOKE ALL ON TABLE curated.sensitive_data FROM public;

-- Row-level security
CREATE OR REPLACE FUNCTION customer_filter()
RETURN current_user() = 'admin' OR customer_id = current_user_id();

ALTER TABLE customers SET ROW FILTER customer_filter ON (customer_id);

-- Column-level masking
CREATE FUNCTION mask_email(email STRING)
  RETURN CASE
    WHEN is_member('pii_viewers') THEN email
    ELSE 'REDACTED'
  END;

ALTER TABLE users ALTER COLUMN email SET MASK mask_email;
```

### 7.3 Audit Logging

```sql
-- View audit log (Delta Lake)
DESCRIBE HISTORY curated.events;

-- Output:
-- version | timestamp           | operation | user      | operationMetrics
-- --------|---------------------|-----------|-----------|------------------
-- 5       | 2025-01-15 10:00:00 | MERGE     | etl_user  | {"numTargetRowsInserted":"1000"}
-- 4       | 2025-01-15 09:00:00 | UPDATE    | admin     | {"numUpdatedRows":"500"}
```

---

## Monitoring & Observability

### 8.1 Metrics

```python
# Prometheus metrics
spark.conf.set("spark.metrics.conf.*.sink.prometheus.class", "org.apache.spark.metrics.sink.PrometheusSink")
spark.conf.set("spark.metrics.conf.*.sink.prometheus.port", "9091")

# Key metrics to track:
# - Ingestion throughput (records/sec)
# - Query latency (P50, P95, P99)
# - Storage growth (bytes/day)
# - Error rates
# - Cost per query
```

### 8.2 Grafana Dashboard

```json
{
  "dashboard": {
    "title": "WIA Data Lake Metrics",
    "panels": [
      {
        "title": "Ingestion Throughput",
        "targets": [
          {
            "expr": "rate(spark_ingestion_records_total[5m])"
          }
        ]
      },
      {
        "title": "Query Latency",
        "targets": [
          {
            "expr": "histogram_quantile(0.95, spark_query_duration_seconds)"
          }
        ]
      },
      {
        "title": "Storage Growth",
        "targets": [
          {
            "expr": "delta(s3_bucket_size_bytes[1d])"
          }
        ]
      }
    ]
  }
}
```

### 8.3 Alerting

```yaml
# Prometheus alerting rules
groups:
  - name: datalake_alerts
    rules:
      - alert: HighIngestionLatency
        expr: spark_ingestion_latency_seconds > 300
        for: 5m
        labels:
          severity: warning
        annotations:
          summary: "High ingestion latency detected"

      - alert: StorageGrowthAnomaly
        expr: rate(s3_bucket_size_bytes[1h]) > 1e11
        for: 10m
        labels:
          severity: critical
        annotations:
          summary: "Unusual storage growth rate"
```

---

## End-to-End Integration Example

```
Complete Data Lake Pipeline:

┌──────────────┐
│ Data Sources │ (Apps, DBs, APIs, IoT)
└──────┬───────┘
       ↓
┌──────────────┐
│   Kafka /    │ (Streaming ingestion)
│   Kinesis    │
└──────┬───────┘
       ↓
┌──────────────┐
│    Spark     │ (Real-time processing)
│  Structured  │
│  Streaming   │
└──────┬───────┘
       ↓
┌──────────────┐
│ Delta Lake / │ (ACID storage)
│  Iceberg /   │
│    Hudi      │
└──────┬───────┘
       ↓
┌──────────────┐
│ Glue Catalog │ (Metadata)
└──────┬───────┘
       ↓
┌──────┴───────┐
│  Athena /    │ (Query engines)
│ Presto /     │
│   Trino      │
└──────┬───────┘
       ↓
┌──────────────┐
│  Tableau /   │ (BI & visualization)
│  Power BI /  │
│  Superset    │
└──────────────┘
```

---

© 2025 WIA Standards / MIT License
