# WIA-DATA-013: Streaming Data Standard
## PHASE 2: API Specification

### Version: 1.0
### Status: Active
### Last Updated: 2025-01-15

---

## 1. Overview

This document defines the API specifications for streaming data platforms, including producer/consumer APIs, admin operations, and management interfaces.

## 2. Producer API

### 2.1 Send Event

**Endpoint**: `POST /api/v1/streams/{streamName}/events`

**Description**: Publish a single event to a stream.

**Request**:
```json
{
  "key": "user-123",
  "value": {
    "eventType": "user-action",
    "timestamp": "2025-01-15T10:30:00Z",
    "data": {...}
  },
  "headers": {
    "correlationId": "abc-123"
  },
  "partitionKey": "user-123"
}
```

**Response**:
```json
{
  "eventId": "evt_1234567890",
  "partition": 3,
  "offset": 1234567,
  "timestamp": "2025-01-15T10:30:00.123Z",
  "status": "published"
}
```

### 2.2 Batch Send

**Endpoint**: `POST /api/v1/streams/{streamName}/events/batch`

**Description**: Publish multiple events in a single batch.

**Parameters**:
- `compression`: gzip, snappy, lz4 (optional)
- `timeout`: milliseconds (default: 5000)

## 3. Consumer API

### 3.1 Consume Events

**Endpoint**: `GET /api/v1/streams/{streamName}/events`

**Query Parameters**:
- `consumerGroup` (required): Consumer group identifier
- `offset` (optional): earliest, latest, or specific offset
- `limit` (optional): Maximum events to return (default: 100)
- `timeout` (optional): Poll timeout in milliseconds

**Response**:
```json
{
  "events": [...],
  "metadata": {
    "consumerGroup": "analytics-group",
    "lag": 0,
    "nextOffset": 1235
  }
}
```

### 3.2 Commit Offsets

**Endpoint**: `POST /api/v1/consumers/{consumerGroup}/commit`

**Request**:
```json
{
  "offsets": [
    {"stream": "user-events", "partition": 0, "offset": 1234},
    {"stream": "user-events", "partition": 1, "offset": 5678}
  ]
}
```

## 4. Stream Management API

### 4.1 Create Stream

**Endpoint**: `POST /api/v1/streams`

**Request**:
```json
{
  "name": "user-events",
  "partitions": 10,
  "replicationFactor": 3,
  "config": {
    "retentionMs": 604800000,
    "compressionType": "gzip"
  }
}
```

### 4.2 Get Stream Info

**Endpoint**: `GET /api/v1/streams/{streamName}`

### 4.3 Update Stream

**Endpoint**: `PUT /api/v1/streams/{streamName}`

### 4.4 Delete Stream

**Endpoint**: `DELETE /api/v1/streams/{streamName}`

## 5. Configuration

Producer configuration:
- `acks`: 0, 1, -1 (all)
- `retries`: Number of retry attempts
- `batch.size`: Batch size in bytes
- `linger.ms`: Batching delay
- `compression.type`: none, gzip, snappy, lz4, zstd
- `idempotence`: true/false

Consumer configuration:
- `group.id`: Consumer group identifier
- `auto.offset.reset`: earliest, latest
- `enable.auto.commit`: true/false
- `max.poll.records`: Maximum records per poll
- `session.timeout.ms`: Session timeout

## 6. Error Codes

| Code | Message | Description |
|------|---------|-------------|
| 1001 | STREAM_NOT_FOUND | Stream does not exist |
| 1002 | INVALID_OFFSET | Offset invalid or out of range |
| 1003 | CONSUMER_GROUP_NOT_FOUND | Consumer group not found |
| 1004 | SERIALIZATION_ERROR | Failed to serialize/deserialize |
| 1005 | QUOTA_EXCEEDED | Rate or throughput quota exceeded |
| 1006 | TIMEOUT | Operation timed out |

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
