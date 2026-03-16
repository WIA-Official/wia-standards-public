# WIA-DATA-001: Phase 3 - Protocol Specification

**Version:** 1.0.0
**Status:** Complete
**Last Updated:** 2025-01-01

---

## 1. Overview

Phase 3 defines communication protocols, data pipelines, and streaming specifications for big data systems.

## 2. Data Pipeline Protocol

### 2.1 Pipeline Stages

```
Source → Transform → Enrich → Validate → Sink
```

### 2.2 Pipeline Configuration

```json
{
  "pipeline": {
    "name": "user_events_pipeline",
    "source": {
      "type": "kafka",
      "topic": "raw_events",
      "consumer_group": "pipeline_001"
    },
    "transforms": [
      {
        "type": "filter",
        "condition": "status == 'active'"
      },
      {
        "type": "map",
        "expression": "value * 2"
      }
    ],
    "sink": {
      "type": "s3",
      "bucket": "processed-events",
      "format": "parquet"
    }
  }
}
```

## 3. Messaging Protocol

### 3.1 Message Structure

```json
{
  "header": {
    "message_id": "msg-12345",
    "timestamp": 1704067200000,
    "source": "sensor-42",
    "schema_version": "1.0"
  },
  "payload": {
    "temperature": 22.5,
    "humidity": 65.0
  },
  "metadata": {
    "priority": "normal",
    "ttl": 3600
  }
}
```

### 3.2 Delivery Semantics

| Semantic | Guarantee | Use Case |
|----------|-----------|----------|
| At-most-once | May be lost, never duplicated | Metrics, logs |
| At-least-once | Delivered, may duplicate | Event tracking |
| Exactly-once | Delivered exactly once | Financial transactions |

## 4. Stream Processing

### 4.1 Windowing Operations

**Tumbling Windows:**
```json
{
  "window": {
    "type": "tumbling",
    "size": "5m"
  }
}
```

**Sliding Windows:**
```json
{
  "window": {
    "type": "sliding",
    "size": "1h",
    "slide": "15m"
  }
}
```

**Session Windows:**
```json
{
  "window": {
    "type": "session",
    "gap": "30m"
  }
}
```

### 4.2 Watermarks

```json
{
  "watermark": {
    "strategy": "bounded",
    "max_delay": "1m"
  }
}
```

## 5. Batch Processing

### 5.1 Job Specification

```json
{
  "job": {
    "name": "daily_aggregation",
    "type": "batch",
    "schedule": "0 2 * * *",
    "input": {
      "dataset": "raw_events",
      "partition": "date=2025-01-01"
    },
    "transformations": [
      {"op": "filter", "condition": "status='active'"},
      {"op": "groupby", "keys": ["user_id"]},
      {"op": "aggregate", "functions": ["COUNT", "SUM(amount)"]}
    ],
    "output": {
      "dataset": "daily_summary",
      "format": "parquet"
    }
  }
}
```

## 6. State Management

### 6.1 Checkpoint Protocol

```json
{
  "checkpoint": {
    "job_id": "job-123",
    "checkpoint_id": "cp-456",
    "timestamp": 1704067200000,
    "offsets": {
      "topic1-partition0": 1000,
      "topic1-partition1": 2000
    },
    "state": {
      "counters": {...},
      "aggregates": {...}
    }
  }
}
```

### 6.2 Savepoints

Manual checkpoints for:
- Version upgrades
- Configuration changes
- Cluster migration

## 7. Data Transfer Protocol

### 7.1 Chunked Transfer

```http
POST /api/v1/transfer/init
{
  "file_size": 10737418240,
  "chunk_size": 10485760,
  "checksum": "sha256:abc123..."
}

Response:
{
  "transfer_id": "txf-67890",
  "num_chunks": 1024,
  "upload_urls": [...]
}
```

### 7.2 Resumable Uploads

- Support for interrupted transfers
- Chunk-level checksums
- Retry logic with exponential backoff
- Concurrent chunk uploads

## 8. Monitoring Protocol

### 8.1 Metrics

```json
{
  "metrics": {
    "job_id": "job-123",
    "timestamp": 1704067200000,
    "throughput": {
      "records_per_sec": 10000,
      "bytes_per_sec": 1048576
    },
    "latency": {
      "p50": 50,
      "p95": 200,
      "p99": 500
    },
    "errors": {
      "count": 5,
      "rate": 0.0001
    }
  }
}
```

### 8.2 Health Checks

```http
GET /health

{
  "status": "healthy",
  "version": "1.0.0",
  "uptime": 86400,
  "components": {
    "database": "healthy",
    "kafka": "healthy",
    "storage": "healthy"
  }
}
```

---

**© 2025 WIA - World Certification Industry Association**
*弘益人間 · Benefit All Humanity*
