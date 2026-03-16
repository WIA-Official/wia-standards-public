# WIA-AGING Phase 3: Protocol Specification

> **Version:** 1.0.0
> **Status:** Official
> **Last Updated:** 2025-01-01
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

The WIA-AGING Protocol specification defines communication protocols for real-time data exchange between aging assessment systems, wearable devices, healthcare platforms, and research databases. This specification ensures secure, efficient, and interoperable data streaming.

### 1.1 Protocol Stack

```
┌─────────────────────────────────────┐
│      Application Layer              │
│   (WIA-AGING Messages)              │
├─────────────────────────────────────┤
│      Presentation Layer             │
│   (JSON/Protocol Buffers)           │
├─────────────────────────────────────┤
│      Session Layer                  │
│   (Connection Management)           │
├─────────────────────────────────────┤
│      Transport Layer                │
│   (WebSocket / gRPC)                │
├─────────────────────────────────────┤
│      Security Layer                 │
│   (TLS 1.3 / E2E Encryption)        │
└─────────────────────────────────────┘
```

---

## 2. Transport Layer

### 2.1 Supported Transports

| Transport | Port | Use Case | Latency |
|-----------|------|----------|---------|
| WebSocket | 443 | Web clients, mobile apps | Low |
| gRPC | 443 | Server-to-server, high throughput | Very Low |
| HTTP/2 | 443 | REST fallback | Medium |

### 2.2 WebSocket Protocol

#### Connection Establishment

```
Client                           Server
  │                                 │
  │  1. HTTP Upgrade Request        │
  │────────────────────────────────▶│
  │                                 │
  │  2. 101 Switching Protocols     │
  │◀────────────────────────────────│
  │                                 │
  │  3. Authentication Frame        │
  │────────────────────────────────▶│
  │                                 │
  │  4. Auth Confirmation           │
  │◀────────────────────────────────│
  │                                 │
  │  5. Bidirectional Data          │
  │◀───────────────────────────────▶│
```

#### WebSocket Endpoint

```
wss://stream.aging.wia.org/v1/ws
```

#### Connection Request

```http
GET /v1/ws HTTP/1.1
Host: stream.aging.wia.org
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==
Sec-WebSocket-Version: 13
Sec-WebSocket-Protocol: wia-aging-v1
Authorization: Bearer {access_token}
```

### 2.3 gRPC Protocol

#### Service Definition

```protobuf
syntax = "proto3";

package wia.aging.v1;

service AgingStreamService {
  // Bidirectional streaming for real-time biomarker data
  rpc StreamBiomarkers(stream BiomarkerEvent) returns (stream BiomarkerResponse);

  // Server streaming for assessment updates
  rpc SubscribeAssessments(SubscriptionRequest) returns (stream AssessmentEvent);

  // Unary call for health check
  rpc HealthCheck(HealthCheckRequest) returns (HealthCheckResponse);
}

message BiomarkerEvent {
  string profile_id = 1;
  string biomarker_code = 2;
  double value = 3;
  string unit = 4;
  int64 timestamp = 5;
  string source = 6;
}

message BiomarkerResponse {
  string event_id = 1;
  bool accepted = 2;
  string status = 3;
  repeated Alert alerts = 4;
}

message Alert {
  string level = 1;  // info, warning, critical
  string message = 2;
  string biomarker_code = 3;
}
```

---

## 3. Message Format

### 3.1 Message Structure

```json
{
  "header": {
    "version": "WIA-AGING/1.0",
    "messageId": "msg_abc123xyz",
    "type": "biomarker.update",
    "timestamp": "2025-01-15T10:30:00.000Z",
    "source": {
      "id": "device_wearable_001",
      "type": "wearable"
    },
    "destination": {
      "id": "profile_abc123",
      "type": "profile"
    }
  },
  "payload": {
    "biomarkers": [
      {
        "code": "WIA-AGE-HR",
        "value": 68,
        "unit": "bpm"
      }
    ]
  },
  "security": {
    "signature": "sha256=abc123...",
    "encryption": "none"
  }
}
```

### 3.2 Message Types

| Type | Direction | Description |
|------|-----------|-------------|
| `auth.request` | Client → Server | Authentication request |
| `auth.response` | Server → Client | Authentication response |
| `biomarker.update` | Client → Server | New biomarker data |
| `biomarker.ack` | Server → Client | Acknowledgment |
| `assessment.request` | Client → Server | Request assessment |
| `assessment.result` | Server → Client | Assessment result |
| `subscription.add` | Client → Server | Subscribe to events |
| `subscription.remove` | Client → Server | Unsubscribe |
| `alert.notify` | Server → Client | Health alert |
| `heartbeat.ping` | Both | Keep-alive |
| `heartbeat.pong` | Both | Keep-alive response |
| `error` | Server → Client | Error notification |

### 3.3 Message Priority

| Priority | Value | Use Case |
|----------|-------|----------|
| Critical | 0 | Health alerts, emergencies |
| High | 1 | Assessment results |
| Normal | 2 | Biomarker updates |
| Low | 3 | Bulk data sync |

---

## 4. Real-Time Streaming

### 4.1 Wearable Data Streaming

```
Wearable Device              WIA Platform
      │                           │
      │  biomarker.update         │
      │  (heart_rate: 72 bpm)     │
      │──────────────────────────▶│
      │                           │
      │  biomarker.ack            │
      │◀──────────────────────────│
      │                           │
      │  biomarker.update         │
      │  (hrv: 45 ms)             │
      │──────────────────────────▶│
      │                           │
      │  alert.notify             │
      │  (abnormal pattern)       │
      │◀──────────────────────────│
```

### 4.2 Streaming Configuration

```json
{
  "streaming": {
    "bufferSize": 100,
    "flushInterval": 5000,
    "compressionEnabled": true,
    "compressionAlgorithm": "gzip",
    "retryPolicy": {
      "maxRetries": 3,
      "backoffMultiplier": 2,
      "initialDelay": 1000,
      "maxDelay": 30000
    }
  }
}
```

### 4.3 Data Aggregation

For high-frequency data (e.g., heart rate every second), the protocol supports aggregation:

```json
{
  "type": "biomarker.batch",
  "payload": {
    "profileId": "profile_abc123",
    "biomarker": "WIA-AGE-HR",
    "aggregation": {
      "period": "1m",
      "values": {
        "min": 58,
        "max": 85,
        "avg": 68.5,
        "count": 60
      },
      "startTime": "2025-01-15T10:30:00Z",
      "endTime": "2025-01-15T10:31:00Z"
    }
  }
}
```

---

## 5. Connection Management

### 5.1 Heartbeat Protocol

```
Client                           Server
  │                                 │
  │  heartbeat.ping                 │
  │  (seq: 1)                       │
  │────────────────────────────────▶│
  │                                 │
  │  heartbeat.pong                 │
  │  (seq: 1, latency: 45ms)        │
  │◀────────────────────────────────│
  │                                 │
  │         ... 30 seconds ...      │
  │                                 │
  │  heartbeat.ping                 │
  │  (seq: 2)                       │
  │────────────────────────────────▶│
```

### 5.2 Connection Parameters

| Parameter | Default | Range |
|-----------|---------|-------|
| Heartbeat Interval | 30s | 10s - 60s |
| Heartbeat Timeout | 90s | 30s - 180s |
| Max Reconnect Attempts | 5 | 1 - 10 |
| Reconnect Backoff | Exponential | - |
| Max Message Size | 1 MB | 64 KB - 16 MB |

### 5.3 Reconnection Strategy

```javascript
const reconnect = {
  attempts: 0,
  maxAttempts: 5,
  baseDelay: 1000,
  maxDelay: 30000,

  getDelay() {
    const delay = Math.min(
      this.baseDelay * Math.pow(2, this.attempts),
      this.maxDelay
    );
    return delay + Math.random() * 1000; // Jitter
  }
};
```

---

## 6. Security

### 6.1 Transport Security

- **TLS 1.3** required for all connections
- **Certificate Pinning** recommended for mobile apps
- **Perfect Forward Secrecy** enabled

### 6.2 Message Security

| Layer | Mechanism |
|-------|-----------|
| Transport | TLS 1.3 |
| Authentication | JWT Bearer Token |
| Message Integrity | HMAC-SHA256 |
| End-to-End | Optional AES-256-GCM |

### 6.3 End-to-End Encryption

For sensitive biomarker data, optional E2E encryption:

```json
{
  "header": {
    "type": "biomarker.update",
    "encryption": {
      "algorithm": "AES-256-GCM",
      "keyId": "key_recipient_001",
      "iv": "base64-encoded-iv"
    }
  },
  "payload": "base64-encoded-encrypted-data"
}
```

### 6.4 Key Exchange

```
Client                           Server
  │                                 │
  │  key.exchange.init              │
  │  (client_public_key)            │
  │────────────────────────────────▶│
  │                                 │
  │  key.exchange.complete          │
  │  (server_public_key, signature) │
  │◀────────────────────────────────│
  │                                 │
  │  [Derive shared secret]         │
  │                                 │
```

---

## 7. Error Handling

### 7.1 Error Codes

| Code | Name | Description |
|------|------|-------------|
| 1001 | AUTH_FAILED | Authentication failure |
| 1002 | AUTH_EXPIRED | Token expired |
| 2001 | INVALID_MESSAGE | Malformed message |
| 2002 | UNKNOWN_TYPE | Unknown message type |
| 3001 | RATE_LIMITED | Too many messages |
| 3002 | QUOTA_EXCEEDED | Daily quota exceeded |
| 4001 | PROFILE_NOT_FOUND | Profile doesn't exist |
| 4002 | BIOMARKER_INVALID | Invalid biomarker data |
| 5001 | SERVER_ERROR | Internal server error |
| 5002 | SERVICE_UNAVAILABLE | Service temporarily down |

### 7.2 Error Message

```json
{
  "type": "error",
  "header": {
    "messageId": "err_xyz789",
    "timestamp": "2025-01-15T10:35:00Z"
  },
  "error": {
    "code": 2002,
    "name": "UNKNOWN_TYPE",
    "message": "Message type 'unknown.type' is not supported",
    "requestId": "msg_abc123"
  }
}
```

---

## 8. Quality of Service

### 8.1 Delivery Guarantees

| Level | Description | Use Case |
|-------|-------------|----------|
| At-most-once | No acknowledgment | Real-time telemetry |
| At-least-once | With acknowledgment | Biomarker updates |
| Exactly-once | With deduplication | Critical health alerts |

### 8.2 Message Ordering

- Messages within a session are ordered by sequence number
- Cross-session ordering uses logical timestamps
- Out-of-order messages are buffered and reordered

---

**弘益人間 (홍익인간) - Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
