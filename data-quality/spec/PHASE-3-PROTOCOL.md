# WIA-DATA-005: Data Quality - Phase 3 Protocol Specification

**Version:** 1.0
**Status:** Draft
**Last Updated:** 2025-12-26

---

## Overview

This document defines the communication protocols for the WIA-DATA-005 Data Quality standard, including real-time streaming, event-driven architecture, and inter-service communication patterns.

## Protocol Stack

### Layer 1: Transport
- HTTP/2 for RESTful APIs
- WebSocket for real-time streams
- gRPC for high-performance RPC
- MQTT for IoT data quality monitoring

### Layer 2: Serialization
- JSON for human-readable data
- Protocol Buffers for binary efficiency
- Avro for schema evolution
- MessagePack for compact representation

### Layer 3: Application
- Data Quality Streaming Protocol (DQSP)
- Quality Event Protocol (QEP)
- Validation Request Protocol (VRP)

## Data Quality Streaming Protocol (DQSP)

### Connection Establishment

WebSocket handshake:

```http
GET /wia/dq/stream HTTP/1.1
Host: api.example.com
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==
Sec-WebSocket-Protocol: dqsp-v1
Sec-WebSocket-Version: 13
```

Response:

```http
HTTP/1.1 101 Switching Protocols
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Accept: s3pPLMBiTxaQ9kYGzzhZRbK+xOo=
Sec-WebSocket-Protocol: dqsp-v1
```

### Message Format

```json
{
  "messageType": "quality.metric|validation.result|issue.alert",
  "messageId": "uuid",
  "timestamp": "ISO8601",
  "payload": {}
}
```

### Subscription

Client subscribes to quality metrics stream:

```json
{
  "action": "subscribe",
  "channels": [
    "dataset.customers.metrics",
    "dataset.orders.validations"
  ],
  "filters": {
    "dimensions": ["accuracy", "completeness"],
    "severity": ["high", "critical"]
  }
}
```

Server acknowledgment:

```json
{
  "messageType": "subscription.confirmed",
  "subscriptionId": "uuid",
  "channels": ["dataset.customers.metrics"]
}
```

### Streaming Metrics

```json
{
  "messageType": "quality.metric",
  "messageId": "uuid",
  "timestamp": "2025-12-26T10:00:00Z",
  "payload": {
    "datasetId": "customers",
    "dimension": "completeness",
    "value": 95.5,
    "delta": -0.5,
    "trend": "declining"
  }
}
```

### Heartbeat

Client and server exchange heartbeats every 30 seconds:

```json
{
  "messageType": "heartbeat",
  "timestamp": "ISO8601"
}
```

## Quality Event Protocol (QEP)

### Event Categories

1. **Validation Events**
   - validation.started
   - validation.completed
   - validation.failed
   - validation.error

2. **Issue Events**
   - issue.created
   - issue.assigned
   - issue.resolved
   - issue.escalated

3. **Threshold Events**
   - threshold.warning
   - threshold.critical
   - threshold.recovered

4. **System Events**
   - profile.generated
   - cleansing.completed
   - rule.created

### Event Message Structure

```json
{
  "eventId": "uuid",
  "eventType": "validation.failed",
  "timestamp": "ISO8601",
  "source": {
    "service": "data-quality-validator",
    "version": "1.0.0",
    "instance": "validator-01"
  },
  "data": {
    "validationId": "uuid",
    "ruleId": "uuid",
    "datasetId": "customers",
    "failedRecords": 150,
    "passRate": 85.0
  },
  "metadata": {
    "correlationId": "uuid",
    "causationId": "uuid"
  }
}
```

### Event Delivery Guarantees

- **At-least-once delivery**: Events may be delivered multiple times
- **Ordering**: Events within same partition maintain order
- **Retention**: Events retained for 7 days by default

## Validation Request Protocol (VRP)

### Request-Response Pattern

Request:

```json
{
  "requestId": "uuid",
  "requestType": "validate",
  "timestamp": "ISO8601",
  "payload": {
    "datasetId": "customers",
    "ruleIds": ["uuid-1", "uuid-2"],
    "options": {
      "async": true,
      "sampleSize": 1000
    }
  }
}
```

Response:

```json
{
  "requestId": "uuid",
  "responseType": "validation.accepted",
  "timestamp": "ISO8601",
  "payload": {
    "validationId": "uuid",
    "status": "queued",
    "estimatedCompletion": "ISO8601"
  }
}
```

### Batch Validation Protocol

Submit multiple validation requests:

```json
{
  "batchId": "uuid",
  "requests": [
    {
      "datasetId": "customers",
      "ruleIds": ["uuid-1"]
    },
    {
      "datasetId": "orders",
      "ruleIds": ["uuid-2"]
    }
  ]
}
```

## gRPC Service Definitions

### Quality Service

```protobuf
syntax = "proto3";

package wia.dataquality.v1;

service QualityService {
  rpc ProfileDataset(ProfileRequest) returns (ProfileResponse);
  rpc ValidateDataset(ValidationRequest) returns (ValidationResponse);
  rpc StreamMetrics(StreamRequest) returns (stream Metric);
  rpc SubmitIssue(IssueRequest) returns (IssueResponse);
}

message ProfileRequest {
  string dataset_id = 1;
  int32 sample_size = 2;
  repeated string columns = 3;
}

message ProfileResponse {
  string profile_id = 1;
  DataQualityProfile profile = 2;
}

message ValidationRequest {
  string dataset_id = 1;
  repeated string rule_ids = 2;
  ValidationOptions options = 3;
}

message ValidationResponse {
  string validation_id = 1;
  string status = 2;
  repeated ValidationResult results = 3;
}
```

## Message Queue Integration

### Apache Kafka Topics

```
wia.dataquality.metrics           # Quality metrics stream
wia.dataquality.validations       # Validation results
wia.dataquality.issues            # Quality issues
wia.dataquality.events            # General events
```

### Message Format

Kafka message with Avro schema:

```json
{
  "key": "customers",
  "value": {
    "metricName": "completeness",
    "value": 95.5,
    "timestamp": 1640534400000
  },
  "headers": {
    "schema-version": "1.0",
    "content-type": "application/avro"
  }
}
```

## MQTT for IoT

### Topic Structure

```
wia/dq/{device-id}/metrics/{metric-name}
wia/dq/{device-id}/status
wia/dq/{device-id}/alerts
```

### QoS Levels

- QoS 0: At most once (metrics)
- QoS 1: At least once (validations)
- QoS 2: Exactly once (critical alerts)

## Security

### TLS/SSL
- Minimum TLS 1.2
- Strong cipher suites only
- Certificate-based authentication for services

### Message Encryption
- Payload encryption for sensitive data
- End-to-end encryption for compliance requirements

### Authentication
- OAuth 2.0 for API access
- JWT tokens for service-to-service
- mTLS for critical services

## Error Handling

### Protocol-Level Errors

```json
{
  "errorCode": "PROTOCOL_ERROR",
  "errorMessage": "Invalid message format",
  "details": {
    "field": "timestamp",
    "violation": "must be ISO8601 format"
  }
}
```

### Retry Strategy

- Exponential backoff: 1s, 2s, 4s, 8s, 16s
- Maximum retries: 5
- Circuit breaker after 3 consecutive failures

## Performance Specifications

### Latency Targets
- API response: < 100ms (p95)
- WebSocket message delivery: < 50ms (p95)
- gRPC call: < 20ms (p95)

### Throughput
- REST API: 10,000 requests/sec
- WebSocket: 100,000 messages/sec
- Kafka: 1,000,000 messages/sec

### Connection Limits
- WebSocket: 10,000 concurrent connections per instance
- HTTP/2: 1,000 concurrent streams per connection

---

**Copyright © 2025 SmileStory Inc. / WIA**
**弘益人間 (홍익인간) · Benefit All Humanity**
