# WIA-DATA-004: PHASE 3 - PROTOCOL

## Overview

This phase defines communication protocols and patterns for data pipeline operations.

## 1. Orchestration Protocol

### 1.1 DAG (Directed Acyclic Graph) Definition

```yaml
dag:
  id: daily_sales_pipeline
  schedule: "0 2 * * *"
  max_active_runs: 1
  catchup: false
  default_args:
    retries: 3
    retry_delay: 300  # seconds
    timeout: 3600

  tasks:
    - id: extract_orders
      type: extract
      source: postgresql
      depends_on: []

    - id: transform_orders
      type: transform
      operation: clean_and_aggregate
      depends_on: [extract_orders]

    - id: load_warehouse
      type: load
      destination: snowflake
      depends_on: [transform_orders]

    - id: notify_completion
      type: notify
      channel: slack
      depends_on: [load_warehouse]
```

### 1.2 Task Execution Protocol

```json
{
  "task_execution": {
    "task_id": "extract_orders",
    "execution_id": "exec_123",
    "state": "running",
    "start_time": "2025-12-26T02:00:00Z",
    "heartbeat_interval_seconds": 30,
    "timeout_seconds": 3600,
    "retry_count": 0,
    "max_retries": 3
  }
}
```

**States:** `pending` → `running` → `success` | `failed` | `timeout`

### 1.3 Heartbeat Protocol

Tasks must send heartbeat every 30 seconds:

```json
{
  "execution_id": "exec_123",
  "task_id": "extract_orders",
  "status": "running",
  "progress_percent": 45,
  "records_processed": 57375,
  "timestamp": "2025-12-26T02:05:30Z"
}
```

## 2. Data Transfer Protocol

### 2.1 Batch Transfer

```json
{
  "transfer": {
    "transfer_id": "xfer_abc123",
    "source": {
      "type": "s3",
      "location": "s3://source-bucket/data/",
      "format": "parquet",
      "compression": "snappy"
    },
    "destination": {
      "type": "snowflake",
      "database": "ANALYTICS",
      "schema": "RAW",
      "table": "ORDERS"
    },
    "mode": "overwrite",  # or "append", "upsert"
    "batch_size": 10000,
    "parallel_streams": 4
  }
}
```

### 2.2 Stream Transfer

```json
{
  "stream": {
    "stream_id": "stream_xyz789",
    "protocol": "kafka",
    "topic": "user-events",
    "consumer_group": "pipeline-processor",
    "offset_management": "exactly_once",
    "poll_timeout_ms": 1000,
    "max_poll_records": 500
  }
}
```

## 3. Change Data Capture (CDC) Protocol

### 3.1 CDC Event Format

```json
{
  "event": {
    "event_id": "cdc_evt_001",
    "source": {
      "database": "production",
      "schema": "public",
      "table": "orders"
    },
    "operation": "INSERT",  # or "UPDATE", "DELETE"
    "before": null,
    "after": {
      "order_id": 12345,
      "user_id": 789,
      "total": 99.99,
      "created_at": "2025-12-26T10:00:00Z"
    },
    "timestamp": "2025-12-26T10:00:01Z",
    "transaction_id": "txn_123",
    "lsn": "0/16B374D8"  # Log Sequence Number
  }
}
```

### 3.2 CDC Subscription

```json
{
  "subscription": {
    "subscription_id": "sub_123",
    "source": "postgresql://prod-db:5432/app",
    "tables": ["orders", "users", "products"],
    "operations": ["INSERT", "UPDATE", "DELETE"],
    "destination": "kafka://localhost:9092/cdc-events",
    "snapshot_mode": "initial"  # or "never", "when_needed"
  }
}
```

## 4. Idempotency Protocol

### 4.1 Idempotency Key

Every operation must include an idempotency key:

```http
POST /api/v1/pipelines/{pipeline_id}/run
X-Idempotency-Key: run_20251226_020000

{
  "parameters": {...}
}
```

Server behavior:
- First request: Process and store result with key
- Duplicate request: Return stored result (do not re-process)
- Key expiration: 24 hours

### 4.2 Exactly-Once Semantics

```json
{
  "exactly_once_config": {
    "enable": true,
    "transaction_id": "txn_abc123",
    "checkpointing": {
      "interval_seconds": 60,
      "location": "s3://bucket/checkpoints/"
    },
    "deduplication": {
      "window_hours": 24,
      "key_column": "event_id"
    }
  }
}
```

## 5. Retry Protocol

### 5.1 Exponential Backoff

```python
def retry_delay(attempt):
    base_delay = 5  # seconds
    max_delay = 300  # 5 minutes
    delay = min(base_delay * (2 ** attempt), max_delay)
    jitter = random.uniform(0, delay * 0.1)
    return delay + jitter
```

### 5.2 Retry Policy

```json
{
  "retry_policy": {
    "max_attempts": 5,
    "backoff_strategy": "exponential",
    "base_delay_seconds": 5,
    "max_delay_seconds": 300,
    "jitter": true,
    "retryable_errors": [
      "connection_timeout",
      "rate_limit_exceeded",
      "temporary_failure"
    ],
    "non_retryable_errors": [
      "authentication_failed",
      "schema_mismatch",
      "data_validation_failed"
    ]
  }
}
```

## 6. Circuit Breaker Protocol

```json
{
  "circuit_breaker": {
    "failure_threshold": 5,
    "success_threshold": 2,
    "timeout_seconds": 60,
    "half_open_max_calls": 3,
    "state": "closed"  # or "open", "half_open"
  }
}
```

**State Transitions:**
- `closed` → `open`: After N failures
- `open` → `half_open`: After timeout
- `half_open` → `closed`: After M successes
- `half_open` → `open`: After any failure

## 7. Data Quality Protocol

### 7.1 Quality Gate

```json
{
  "quality_gate": {
    "gate_id": "qg_orders",
    "checks": [
      {
        "metric": "null_rate",
        "column": "user_id",
        "threshold": 0.01,
        "severity": "blocker"
      },
      {
        "metric": "duplicate_rate",
        "columns": ["order_id"],
        "threshold": 0.001,
        "severity": "critical"
      }
    ],
    "action_on_failure": "quarantine"  # or "fail", "log"
  }
}
```

### 7.2 Quarantine Protocol

```json
{
  "quarantine": {
    "bad_records_location": "s3://bucket/quarantine/orders/",
    "error_log_location": "s3://bucket/logs/errors/",
    "retention_days": 30,
    "notification": {
      "channel": "slack",
      "webhook": "https://hooks.slack.com/services/xxx"
    }
  }
}
```

## 8. Monitoring Protocol

### 8.1 Metrics Push

Pipelines push metrics every 60 seconds:

```json
{
  "metrics": {
    "pipeline_id": "pip_abc123",
    "execution_id": "exec_xyz789",
    "timestamp": "2025-12-26T10:05:00Z",
    "counters": {
      "records_processed": 127500,
      "records_failed": 50,
      "bytes_processed": 10485760
    },
    "gauges": {
      "pipeline_lag_seconds": 120,
      "memory_usage_mb": 2048,
      "cpu_usage_percent": 75
    },
    "histograms": {
      "processing_duration_ms": [45, 52, 48, 61, 55]
    }
  }
}
```

### 8.2 Log Streaming

Structured logs sent to central collector:

```json
{
  "log": {
    "timestamp": "2025-12-26T10:05:00.123Z",
    "level": "INFO",
    "pipeline_id": "pip_abc123",
    "execution_id": "exec_xyz789",
    "task_id": "extract_orders",
    "message": "Extracted 10000 records",
    "context": {
      "source": "postgresql",
      "table": "orders",
      "duration_ms": 245
    }
  }
}
```

## 9. Schema Evolution Protocol

### 9.1 Schema Registry

```json
{
  "schema_registration": {
    "subject": "orders-value",
    "version": 2,
    "schema_type": "AVRO",
    "schema": "{...avro schema...}",
    "compatibility": "BACKWARD",
    "metadata": {
      "owner": "data-team",
      "created_at": "2025-12-26T10:00:00Z"
    }
  }
}
```

### 9.2 Compatibility Modes

- **BACKWARD:** New schema can read old data
- **FORWARD:** Old schema can read new data
- **FULL:** Both backward and forward compatible
- **NONE:** No compatibility check

## 10. Security Protocol

### 10.1 Authentication

```http
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

### 10.2 Encryption

- **In Transit:** TLS 1.3 minimum
- **At Rest:** AES-256-GCM
- **Keys:** Rotated every 90 days

### 10.3 Audit Log

```json
{
  "audit": {
    "action": "pipeline_run",
    "actor": "user@company.com",
    "resource": "pip_abc123",
    "timestamp": "2025-12-26T10:00:00Z",
    "ip_address": "192.168.1.1",
    "user_agent": "PipelineSDK/1.0",
    "result": "success"
  }
}
```

---

**Status:** ✅ Complete
**Version:** 1.0.0
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) · Benefit All Humanity
