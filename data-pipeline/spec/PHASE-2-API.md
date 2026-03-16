# WIA-DATA-004: PHASE 2 - API

## Overview

This phase defines the standard APIs for data pipeline operations.

## 1. Pipeline Management API

### 1.1 Create Pipeline

```http
POST /api/v1/pipelines
Content-Type: application/json

{
  "name": "daily_sales_pipeline",
  "description": "Aggregate daily sales from orders",
  "source": {
    "type": "postgresql",
    "connection_id": "prod-db",
    "query": "SELECT * FROM orders WHERE date >= :watermark"
  },
  "transformations": [
    {
      "type": "sql",
      "query": "SELECT date, SUM(total) as revenue FROM orders GROUP BY date"
    }
  ],
  "destination": {
    "type": "snowflake",
    "connection_id": "warehouse",
    "table": "daily_sales"
  },
  "schedule": "0 2 * * *"
}
```

**Response:**
```json
{
  "pipeline_id": "pip_abc123",
  "status": "created",
  "created_at": "2025-12-26T10:00:00Z"
}
```

### 1.2 Trigger Pipeline

```http
POST /api/v1/pipelines/{pipeline_id}/run
Content-Type: application/json

{
  "trigger_type": "manual",
  "parameters": {
    "start_date": "2025-12-26",
    "end_date": "2025-12-26"
  }
}
```

**Response:**
```json
{
  "execution_id": "exec_xyz789",
  "status": "running",
  "started_at": "2025-12-26T10:00:00Z"
}
```

### 1.3 Get Pipeline Status

```http
GET /api/v1/pipelines/{pipeline_id}/executions/{execution_id}
```

**Response:**
```json
{
  "execution_id": "exec_xyz789",
  "pipeline_id": "pip_abc123",
  "status": "completed",
  "started_at": "2025-12-26T10:00:00Z",
  "completed_at": "2025-12-26T10:15:30Z",
  "metrics": {
    "records_extracted": 127500,
    "records_transformed": 127450,
    "records_loaded": 127450,
    "records_failed": 50,
    "duration_seconds": 930
  }
}
```

## 2. Data Ingestion API

### 2.1 Stream Ingestion

```http
POST /api/v1/ingest/stream
Content-Type: application/json

{
  "stream_id": "user-events",
  "events": [
    {
      "event_id": "evt_001",
      "user_id": 123,
      "event_type": "purchase",
      "timestamp": "2025-12-26T10:00:00Z",
      "properties": {
        "product_id": 456,
        "amount": 99.99
      }
    }
  ]
}
```

**Response:**
```json
{
  "ingested": 1,
  "failed": 0,
  "batch_id": "batch_123"
}
```

### 2.2 Batch Ingestion

```http
POST /api/v1/ingest/batch
Content-Type: multipart/form-data

file: orders_20251226.csv
```

**Response:**
```json
{
  "file_id": "file_abc123",
  "status": "processing",
  "estimated_records": 10000
}
```

## 3. Data Quality API

### 3.1 Run Quality Check

```http
POST /api/v1/quality/check
Content-Type: application/json

{
  "dataset_id": "orders_clean",
  "checks": [
    {
      "type": "not_null",
      "column": "user_id"
    },
    {
      "type": "range",
      "column": "total",
      "min": 0,
      "max": 1000000
    }
  ]
}
```

**Response:**
```json
{
  "check_id": "qc_123",
  "status": "passed",
  "results": [
    {
      "check": "not_null",
      "column": "user_id",
      "passed": true,
      "null_count": 0
    },
    {
      "check": "range",
      "column": "total",
      "passed": true,
      "out_of_range_count": 0
    }
  ]
}
```

## 4. Monitoring API

### 4.1 Get Metrics

```http
GET /api/v1/metrics/pipelines/{pipeline_id}?start_time=2025-12-26T00:00:00Z&end_time=2025-12-26T23:59:59Z
```

**Response:**
```json
{
  "pipeline_id": "pip_abc123",
  "metrics": {
    "executions_total": 24,
    "executions_success": 23,
    "executions_failed": 1,
    "avg_duration_seconds": 945,
    "records_processed_total": 3060000
  },
  "time_series": [
    {
      "timestamp": "2025-12-26T00:00:00Z",
      "records_processed": 127500,
      "duration_seconds": 930
    }
  ]
}
```

### 4.2 Set Alert

```http
POST /api/v1/alerts
Content-Type: application/json

{
  "pipeline_id": "pip_abc123",
  "metric": "error_rate",
  "condition": ">",
  "threshold": 0.01,
  "action": "slack_notification",
  "config": {
    "channel": "#data-alerts"
  }
}
```

## 5. Data Catalog API

### 5.1 Register Dataset

```http
POST /api/v1/catalog/datasets
Content-Type: application/json

{
  "name": "orders_clean",
  "description": "Cleaned order data",
  "schema": {
    "columns": [
      {"name": "order_id", "type": "int64", "nullable": false},
      {"name": "user_id", "type": "int64", "nullable": false},
      {"name": "total", "type": "decimal(10,2)", "nullable": false},
      {"name": "created_at", "type": "timestamp", "nullable": false}
    ]
  },
  "location": "s3://bucket/orders_clean/",
  "format": "parquet",
  "partition_columns": ["date"],
  "owner": "data-team@company.com"
}
```

### 5.2 Search Datasets

```http
GET /api/v1/catalog/datasets?q=orders&owner=data-team
```

**Response:**
```json
{
  "datasets": [
    {
      "dataset_id": "ds_123",
      "name": "orders_clean",
      "description": "Cleaned order data",
      "owner": "data-team@company.com",
      "created_at": "2025-12-26T10:00:00Z",
      "updated_at": "2025-12-26T10:00:00Z"
    }
  ],
  "total": 1
}
```

## 6. Lineage API

### 6.1 Get Lineage

```http
GET /api/v1/lineage/datasets/{dataset_id}
```

**Response:**
```json
{
  "dataset_id": "orders_clean",
  "upstream": [
    {
      "dataset_id": "orders_raw",
      "pipeline_id": "pip_cleaning"
    }
  ],
  "downstream": [
    {
      "dataset_id": "daily_sales",
      "pipeline_id": "pip_aggregation"
    }
  ]
}
```

## 7. SDK Examples

### 7.1 Python SDK

```python
from wia_data_pipeline import PipelineClient

client = PipelineClient(api_key="your_api_key")

# Create pipeline
pipeline = client.pipelines.create(
    name="daily_sales",
    source={"type": "postgresql", "query": "SELECT * FROM orders"},
    destination={"type": "snowflake", "table": "sales"}
)

# Trigger run
execution = pipeline.run()

# Wait for completion
execution.wait_until_complete()

print(f"Status: {execution.status}")
print(f"Records processed: {execution.metrics.records_processed}")
```

### 7.2 TypeScript SDK

```typescript
import { PipelineClient } from '@wia/data-pipeline';

const client = new PipelineClient({ apiKey: 'your_api_key' });

// Create pipeline
const pipeline = await client.pipelines.create({
  name: 'daily_sales',
  source: { type: 'postgresql', query: 'SELECT * FROM orders' },
  destination: { type: 'snowflake', table: 'sales' }
});

// Trigger run
const execution = await pipeline.run();

// Wait for completion
await execution.waitUntilComplete();

console.log(`Status: ${execution.status}`);
console.log(`Records: ${execution.metrics.recordsProcessed}`);
```

## 8. Error Codes

| Code | Message | Description |
|------|---------|-------------|
| 1001 | Pipeline not found | Pipeline ID does not exist |
| 1002 | Invalid schedule | Cron expression is invalid |
| 2001 | Source connection failed | Cannot connect to data source |
| 2002 | Query execution failed | SQL query error |
| 3001 | Transformation failed | Data transformation error |
| 4001 | Destination write failed | Cannot write to destination |
| 5001 | Quality check failed | Data quality validation failed |

---

**Status:** ✅ Complete
**Version:** 1.0.0
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) · Benefit All Humanity
