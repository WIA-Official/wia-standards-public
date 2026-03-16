# WIA-DATA-012: Data Analytics Standard
## PHASE 3 - PROTOCOL SPECIFICATION

### Version: 1.0
### Status: Active
### Last Updated: 2025-12-26

---

## 1. Overview

This document defines the communication protocols for WIA-DATA-012 Data Analytics Standard. These protocols enable real-time, distributed analytics across different systems and platforms.

**Philosophy**: 弘益人間 (홍익인간) - Benefit All Humanity through efficient, reliable analytics communication.

## 2. Protocol Principles

### 2.1 Core Principles
- **Reliability**: Guaranteed message delivery
- **Scalability**: Handle high throughput
- **Low Latency**: Minimize processing delays
- **Fault Tolerance**: Graceful degradation
- **Security**: End-to-end encryption
- **Interoperability**: Cross-platform compatibility

## 3. Message Queue Protocols

### 3.1 Kafka Protocol
Standard protocol for event streaming in WIA-DATA-012.

**Configuration**:
```json
{
  "bootstrap_servers": ["kafka1:9092", "kafka2:9092"],
  "topic": "wia.analytics.events",
  "partitions": 10,
  "replication_factor": 3,
  "compression_type": "snappy",
  "batch_size": 16384,
  "linger_ms": 10
}
```

**Topic Naming Convention**:
```
wia.analytics.{environment}.{data_type}.{version}

Examples:
- wia.analytics.prod.events.v1
- wia.analytics.dev.metrics.v1
- wia.analytics.staging.models.v1
```

**Message Format**:
```json
{
  "headers": {
    "wia_version": "1.0",
    "message_type": "analytics_event",
    "timestamp": 1735200000000,
    "correlation_id": "CORR-12345",
    "source": "web-app-01"
  },
  "key": "user_123",
  "value": {
    "event_id": "EVT-001",
    "event_type": "purchase",
    "user_id": "user_123",
    "amount": 99.99,
    "properties": {
      "category": "electronics",
      "product_id": "PROD-456"
    }
  }
}
```

### 3.2 MQTT Protocol
Lightweight protocol for IoT and edge analytics.

**Connection**:
```
mqtt://broker.wia.org:1883
mqtts://broker.wia.org:8883  # TLS
```

**Topic Structure**:
```
wia/analytics/{client_id}/{data_type}/{action}

Examples:
- wia/analytics/device001/metrics/publish
- wia/analytics/sensor042/events/publish
- wia/analytics/gateway01/aggregated/publish
```

**QoS Levels**:
- **QoS 0**: At most once (fire and forget)
- **QoS 1**: At least once (acknowledged delivery)
- **QoS 2**: Exactly once (guaranteed delivery)

**Payload Format**:
```json
{
  "wia_version": "1.0",
  "device_id": "device001",
  "timestamp": 1735200000000,
  "metrics": {
    "temperature": 23.5,
    "humidity": 45.2,
    "cpu_usage": 67.8
  }
}
```

## 4. WebSocket Protocol

### 4.1 Real-time Streaming
WebSocket connection for real-time analytics updates.

**Connection URL**:
```
wss://api.wia.org/v1/streams/{stream_id}
```

**Authentication**:
```json
{
  "type": "auth",
  "token": "your_access_token",
  "stream_id": "STREAM-001"
}
```

**Message Types**:

**Subscribe to Metrics**:
```json
{
  "type": "subscribe",
  "stream_id": "STREAM-001",
  "metrics": ["count", "sum", "avg"],
  "window": "5m"
}
```

**Data Update**:
```json
{
  "type": "update",
  "timestamp": "2025-12-26T10:00:00Z",
  "window": {
    "start": "2025-12-26T09:55:00Z",
    "end": "2025-12-26T10:00:00Z"
  },
  "metrics": {
    "count": 1245,
    "sum": 125430.50,
    "avg": 100.74
  }
}
```

**Anomaly Alert**:
```json
{
  "type": "alert",
  "alert_id": "ALERT-001",
  "severity": "high",
  "message": "Unusual spike in transaction volume",
  "details": {
    "metric": "count",
    "threshold": 1000,
    "actual": 1245,
    "deviation": 1.245
  }
}
```

## 5. gRPC Protocol

### 5.1 Service Definition
Protocol Buffers for high-performance analytics.

**Proto Definition**:
```protobuf
syntax = "proto3";

package wia.analytics.v1;

service AnalyticsService {
  rpc DescriptiveAnalysis(DescriptiveRequest) returns (AnalyticsResponse);
  rpc PredictiveAnalysis(PredictiveRequest) returns (PredictionResponse);
  rpc StreamAnalytics(stream AnalyticsEvent) returns (stream StreamResult);
}

message DescriptiveRequest {
  string dataset_id = 1;
  repeated string metrics = 2;
  repeated string group_by = 3;
  Filter filters = 4;
}

message AnalyticsResponse {
  string request_id = 1;
  int64 execution_time_ms = 2;
  Statistics stats = 3;
  repeated GroupResult grouped_results = 4;
}

message AnalyticsEvent {
  string event_id = 1;
  int64 timestamp = 2;
  string user_id = 3;
  string event_type = 4;
  map<string, string> properties = 5;
  map<string, double> metrics = 6;
}

message StreamResult {
  int64 timestamp = 1;
  map<string, double> aggregations = 2;
  repeated Anomaly anomalies = 3;
}
```

**Usage Example**:
```python
import grpc
from wia.analytics.v1 import analytics_pb2, analytics_pb2_grpc

channel = grpc.secure_channel(
    'api.wia.org:443',
    grpc.ssl_channel_credentials()
)
stub = analytics_pb2_grpc.AnalyticsServiceStub(channel)

request = analytics_pb2.DescriptiveRequest(
    dataset_id='sales_2025',
    metrics=['mean', 'median', 'std'],
    group_by=['category']
)

response = stub.DescriptiveAnalysis(request)
```

## 6. Batch Processing Protocol

### 6.1 Spark Connector
Standard interface for Apache Spark integration.

**Configuration**:
```python
from pyspark.sql import SparkSession

spark = SparkSession.builder \
    .appName("WIA-Analytics") \
    .config("spark.wia.apiKey", "your_api_key") \
    .config("spark.wia.endpoint", "https://api.wia.org/v1") \
    .getOrCreate()

# Read from WIA-DATA-012 source
df = spark.read \
    .format("wia") \
    .option("dataset_id", "sales_2025") \
    .option("format", "parquet") \
    .load()

# Write analytics results
df.write \
    .format("wia") \
    .option("result_type", "aggregation") \
    .option("output_id", "monthly_summary") \
    .save()
```

## 7. State Management Protocol

### 7.1 Distributed State
Protocol for managing state in distributed analytics.

**State Snapshot Format**:
```json
{
  "snapshot_id": "SNAP-2025-12-26-001",
  "timestamp": "2025-12-26T10:00:00Z",
  "stream_id": "STREAM-001",
  "checkpoint": {
    "offset": 123456,
    "partition": 3,
    "topic": "wia.analytics.events"
  },
  "state": {
    "window_aggregations": {
      "count": 1245,
      "sum": 125430.50,
      "running_avg": 100.74
    },
    "user_sessions": {
      "active_count": 234,
      "session_map": { /* ... */ }
    }
  }
}
```

**State Recovery**:
```python
def recover_state(snapshot_id):
    snapshot = load_snapshot(snapshot_id)
    restore_checkpoint(snapshot['checkpoint'])
    restore_aggregations(snapshot['state']['window_aggregations'])
    restore_sessions(snapshot['state']['user_sessions'])
    return True
```

## 8. Data Lineage Protocol

### 8.1 Lineage Tracking
Track data transformations and dependencies.

**Lineage Metadata**:
```json
{
  "lineage_id": "LIN-2025-12-26-001",
  "dataset_id": "monthly_revenue_summary",
  "created_at": "2025-12-26T10:00:00Z",
  "lineage_chain": [
    {
      "step": 1,
      "operation": "extract",
      "source": {
        "type": "database",
        "connection": "postgres://sales-db",
        "table": "transactions"
      },
      "timestamp": "2025-12-26T09:00:00Z"
    },
    {
      "step": 2,
      "operation": "transform",
      "transformation": "aggregation",
      "code": "SELECT date, SUM(amount) FROM transactions GROUP BY date",
      "timestamp": "2025-12-26T09:30:00Z"
    },
    {
      "step": 3,
      "operation": "load",
      "destination": {
        "type": "data_warehouse",
        "connection": "snowflake://analytics",
        "table": "monthly_revenue_summary"
      },
      "timestamp": "2025-12-26T10:00:00Z"
    }
  ],
  "dependencies": [
    "sales_transactions",
    "exchange_rates"
  ]
}
```

## 9. Security Protocols

### 9.1 TLS/SSL Configuration
```json
{
  "tls_version": "1.3",
  "cipher_suites": [
    "TLS_AES_256_GCM_SHA384",
    "TLS_CHACHA20_POLY1305_SHA256"
  ],
  "certificate": {
    "path": "/path/to/cert.pem",
    "key_path": "/path/to/key.pem",
    "ca_path": "/path/to/ca.pem"
  }
}
```

### 9.2 Message Encryption
```python
from cryptography.fernet import Fernet

def encrypt_message(message, key):
    f = Fernet(key)
    encrypted = f.encrypt(message.encode())
    return {
        "encrypted": True,
        "algorithm": "Fernet",
        "data": encrypted.decode()
    }

def decrypt_message(encrypted_msg, key):
    f = Fernet(key)
    decrypted = f.decrypt(encrypted_msg['data'].encode())
    return decrypted.decode()
```

## 10. Monitoring Protocol

### 10.1 Metrics Reporting
```json
{
  "metric_type": "stream_performance",
  "timestamp": "2025-12-26T10:00:00Z",
  "stream_id": "STREAM-001",
  "metrics": {
    "throughput": {
      "events_per_second": 142.5,
      "bytes_per_second": 145820
    },
    "latency": {
      "avg_ms": 23,
      "p50_ms": 18,
      "p95_ms": 45,
      "p99_ms": 78
    },
    "errors": {
      "count": 3,
      "rate_per_second": 0.021
    },
    "resources": {
      "cpu_percent": 45.2,
      "memory_mb": 1024,
      "network_mbps": 12.5
    }
  }
}
```

## 11. Discovery Protocol

### 11.1 Service Discovery
```json
{
  "service_type": "analytics_api",
  "version": "1.0",
  "endpoints": [
    {
      "name": "descriptive_analytics",
      "url": "https://api.wia.org/v1/analytics/descriptive",
      "protocol": "https",
      "health_check": "/health"
    },
    {
      "name": "stream_processor",
      "url": "grpc://stream.wia.org:443",
      "protocol": "grpc",
      "health_check": "/grpc.health.v1.Health/Check"
    }
  ],
  "metadata": {
    "region": "us-west-2",
    "availability_zone": "us-west-2a",
    "capacity": "high"
  }
}
```

## 12. Conclusion

This PHASE 3 specification defines the communication protocols for WIA-DATA-012, enabling:
- Reliable message delivery across systems
- Real-time streaming analytics
- High-performance data processing
- Secure communication
- Distributed state management

**Next Phase**: PHASE 4 - Integration Specification

---

© 2025 SmileStory Inc. / WIA  
弘益人間 (홍익인간) · Benefit All Humanity
