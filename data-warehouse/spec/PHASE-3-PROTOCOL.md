# WIA Data Warehouse Protocol Standard
## Phase 3 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #10B981 (Emerald - DATA domain)

---

## Table of Contents

1. [Overview](#overview)
2. [Communication Protocols](#communication-protocols)
3. [Data Transfer Protocols](#data-transfer-protocols)
4. [ETL Orchestration Protocol](#etl-orchestration-protocol)
5. [Query Execution Protocol](#query-execution-protocol)
6. [Security Protocols](#security-protocols)
7. [Monitoring and Logging](#monitoring-and-logging)

---

## Overview

### 1.1 Purpose

The WIA Data Warehouse Protocol Standard defines communication and data exchange protocols for data warehouse systems, ensuring interoperability between ETL tools, query engines, and BI platforms.

**Protocol Coverage**:
- Data ingestion and extraction
- Query execution and result streaming
- ETL job orchestration
- Metadata exchange
- Security and encryption

---

## Communication Protocols

### 2.1 HTTP/HTTPS

**Primary protocol** for RESTful API communication:
- HTTPS (TLS 1.2+) for all production traffic
- HTTP/2 for improved performance
- WebSocket for real-time streaming

### 2.2 gRPC

**High-performance RPC** for internal services:
```protobuf
service DataWarehouse {
  rpc ExecuteQuery(QueryRequest) returns (stream QueryResult);
  rpc LoadData(stream DataBatch) returns (LoadResponse);
  rpc GetMetadata(MetadataRequest) returns (MetadataResponse);
}
```

### 2.3 JDBC/ODBC

**Standard database connectivity**:
- JDBC Driver: `com.wia.datawarehouse.jdbc.Driver`
- ODBC DSN configuration for BI tools
- Connection string format:
  ```
  jdbc:wia-dw://{host}:{port}/{database}?user={user}&password={password}
  ```

---

## Data Transfer Protocols

### 3.1 Bulk Data Load Protocol

#### CSV Upload
```http
POST /v1/data/load/csv
Content-Type: multipart/form-data

{
  "file": binary_csv_data,
  "table_name": "fact_sales",
  "delimiter": ",",
  "skip_header": true,
  "encoding": "UTF-8"
}
```

#### Parquet Upload
```http
POST /v1/data/load/parquet
Content-Type: application/octet-stream

{
  "file": binary_parquet_data,
  "table_name": "fact_sales",
  "schema_validation": true
}
```

### 3.2 Streaming Data Protocol

**Apache Kafka Integration**:
```json
{
  "protocol": "kafka",
  "bootstrap_servers": ["kafka1:9092", "kafka2:9092"],
  "topic": "sales-events",
  "consumer_group": "dw-consumer",
  "auto_offset_reset": "earliest",
  "target_table": "fact_sales"
}
```

**Amazon Kinesis Integration**:
```json
{
  "protocol": "kinesis",
  "stream_name": "sales-stream",
  "region": "us-east-1",
  "shard_iterator_type": "LATEST",
  "target_table": "fact_sales"
}
```

### 3.3 Change Data Capture (CDC) Protocol

```json
{
  "cdc_protocol": {
    "method": "log_based|trigger_based|timestamp_based",
    "source_database": {
      "type": "mysql|postgresql|oracle",
      "host": "source-db.example.com",
      "port": 3306,
      "database": "production"
    },
    "capture_tables": [
      {
        "source_table": "orders",
        "target_table": "fact_orders",
        "capture_deletes": true,
        "capture_updates": true
      }
    ],
    "checkpoint_interval_seconds": 60
  }
}
```

---

## ETL Orchestration Protocol

### 4.1 Job Definition Format

```json
{
  "job_id": "etl-daily-sales",
  "job_name": "Daily Sales ETL",
  "schedule": "0 2 * * *",
  "dependencies": ["etl-customer-refresh", "etl-product-refresh"],
  "stages": [
    {
      "stage_name": "extract",
      "type": "extract",
      "source": {
        "type": "mysql",
        "connection": "source-db",
        "query": "SELECT * FROM orders WHERE order_date = CURRENT_DATE - 1"
      }
    },
    {
      "stage_name": "transform",
      "type": "transform",
      "transformations": [
        {
          "operation": "join",
          "join_table": "dim_customer",
          "join_key": "customer_id"
        },
        {
          "operation": "aggregate",
          "group_by": ["customer_id", "product_id"],
          "measures": ["SUM(amount)", "COUNT(*)"]
        }
      ]
    },
    {
      "stage_name": "load",
      "type": "load",
      "target": {
        "table": "fact_sales",
        "load_strategy": "incremental|full|merge"
      }
    }
  ],
  "error_handling": {
    "max_retries": 3,
    "retry_delay_seconds": 300,
    "on_failure": "alert|skip|abort"
  }
}
```

### 4.2 Job Status Protocol

```json
{
  "job_id": "etl-daily-sales-20250115-001",
  "status": "running|completed|failed|cancelled",
  "progress": {
    "current_stage": "transform",
    "stage_progress_percent": 65,
    "overall_progress_percent": 45,
    "rows_processed": 1234567,
    "estimated_completion": "2025-01-15T03:30:00Z"
  },
  "metrics": {
    "start_time": "2025-01-15T02:00:00Z",
    "end_time": null,
    "duration_seconds": 5400,
    "rows_extracted": 2000000,
    "rows_transformed": 1800000,
    "rows_loaded": 1234567,
    "errors": 123
  }
}
```

---

## Query Execution Protocol

### 5.1 Query Submission

```json
{
  "query_id": "uuid-generated",
  "sql": "SELECT ...",
  "execution_mode": "synchronous|asynchronous",
  "timeout_seconds": 300,
  "result_format": "json|csv|parquet",
  "use_cache": true,
  "max_rows": 10000
}
```

### 5.2 Query Result Streaming

**Server-Sent Events (SSE)**:
```http
GET /v1/query/{query_id}/stream
Accept: text/event-stream

event: metadata
data: {"columns": [...], "total_rows": 10000}

event: data
data: {"row": [1, "value", 123.45]}

event: data
data: {"row": [2, "value2", 456.78]}

event: complete
data: {"rows_returned": 10000, "execution_time_ms": 234}
```

### 5.3 Prepared Statements

```json
{
  "prepare_query": {
    "statement_id": "stmt-001",
    "sql": "SELECT * FROM fact_sales WHERE date_key = ? AND store_key = ?",
    "parameter_types": ["int", "int"]
  },
  "execute_prepared": {
    "statement_id": "stmt-001",
    "parameters": [20250115, 5]
  }
}
```

---

## Security Protocols

### 6.1 Encryption

**Data at Rest**:
- AES-256 encryption for all stored data
- Transparent Data Encryption (TDE) support
- Customer-managed encryption keys (CMEK)

**Data in Transit**:
- TLS 1.2+ for all network traffic
- Certificate pinning for mobile/desktop clients
- mTLS for service-to-service communication

### 6.2 Authentication Methods

```json
{
  "authentication": {
    "method": "api_key|oauth2|saml|ldap",
    "api_key": {
      "header": "Authorization",
      "prefix": "Bearer"
    },
    "oauth2": {
      "authorization_endpoint": "https://auth.example.com/oauth/authorize",
      "token_endpoint": "https://auth.example.com/oauth/token",
      "scopes": ["warehouse:read", "warehouse:write"]
    }
  }
}
```

### 6.3 Authorization Protocol

**Role-Based Access Control (RBAC)**:
```json
{
  "principal": "user@example.com",
  "roles": ["analyst", "data_engineer"],
  "permissions": {
    "fact_sales": ["SELECT"],
    "dim_customer": ["SELECT", "INSERT", "UPDATE"],
    "dim_product": ["SELECT"]
  },
  "row_level_security": {
    "table": "fact_sales",
    "filter": "store_key IN (SELECT store_key FROM user_stores WHERE user_id = CURRENT_USER)"
  }
}
```

---

## Monitoring and Logging

### 6.1 Metrics Protocol

**Prometheus Format**:
```
# HELP dw_query_duration_seconds Query execution duration
# TYPE dw_query_duration_seconds histogram
dw_query_duration_seconds_bucket{le="0.1"} 100
dw_query_duration_seconds_bucket{le="1.0"} 450
dw_query_duration_seconds_bucket{le="10.0"} 980
dw_query_duration_seconds_sum 4523.4
dw_query_duration_seconds_count 1000

# HELP dw_rows_scanned_total Total rows scanned
# TYPE dw_rows_scanned_total counter
dw_rows_scanned_total 123456789
```

### 6.2 Logging Protocol

**Structured JSON Logs**:
```json
{
  "timestamp": "2025-01-15T10:30:00.123Z",
  "level": "INFO|WARN|ERROR",
  "service": "query-engine",
  "query_id": "q-12345",
  "user": "user@example.com",
  "action": "query_execution",
  "duration_ms": 234,
  "rows_returned": 1000,
  "message": "Query executed successfully",
  "metadata": {
    "tables_accessed": ["fact_sales", "dim_date"],
    "cache_hit": true
  }
}
```

### 6.3 Audit Trail Protocol

```json
{
  "audit_event": {
    "event_id": "audit-12345",
    "timestamp": "2025-01-15T10:30:00Z",
    "event_type": "data_access|data_modification|schema_change",
    "principal": "user@example.com",
    "action": "SELECT",
    "resource": "fact_sales",
    "result": "success|failure",
    "details": {
      "rows_accessed": 1000,
      "filters_applied": "date_key = 20250115"
    }
  }
}
```

---

**License**: MIT
**Copyright**: © 2025 WIA Standards Committee
**Philosophy**: 弘益人間 (Benefit All Humanity)
