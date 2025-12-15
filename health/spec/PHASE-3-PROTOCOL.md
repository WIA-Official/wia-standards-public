# Phase 3: Communication Protocol Standard

**WIA Health Standard v1.0.0**
**Status**: Draft
**Last Updated**: December 2025

---

## 1. Overview

### 1.1 Purpose

This document defines the communication protocol for the WIA Health Standard, enabling real-time data exchange between health devices, applications, and platforms.

### 1.2 Scope

The protocol covers:
- Message formats and types
- Connection management
- Transport layer abstractions
- Error handling and recovery
- Security requirements

### 1.3 Design Principles

1. **Transport Agnostic**: Same message format across WebSocket, MQTT, BLE
2. **Phase 1/2 Compatible**: Payload uses defined data types
3. **Real-Time First**: Optimized for streaming data
4. **Secure by Default**: TLS required, OAuth 2.0 authentication
5. **Healthcare Interoperable**: FHIR-compatible design

---

## 2. Terminology

| Term | Definition |
|------|------------|
| **Client** | Application connecting to WIA Health services |
| **Server** | WIA Health platform or gateway |
| **Device** | Wearable or health monitoring hardware |
| **Stream** | Continuous flow of health data |
| **Session** | Active connection between client and server |
| **Payload** | Message data (Phase 1 types) |

---

## 3. Protocol Architecture

### 3.1 Layer Model

```
┌─────────────────────────────────────────────────────────────┐
│                    Application Layer                        │
│         (HealthProfile, Biomarkers, Digital Twin)           │
├─────────────────────────────────────────────────────────────┤
│                     Message Layer                           │
│              (WIA Health Protocol Messages)                 │
├─────────────────────────────────────────────────────────────┤
│                    Transport Layer                          │
│          (WebSocket | MQTT | BLE | HTTP/REST)               │
├─────────────────────────────────────────────────────────────┤
│                     Security Layer                          │
│              (TLS 1.3 | OAuth 2.0 | mTLS)                   │
├─────────────────────────────────────────────────────────────┤
│                     Network Layer                           │
│                (TCP/IP | UDP | Bluetooth)                   │
└─────────────────────────────────────────────────────────────┘
```

### 3.2 Message Flow

```
Client                                             Server
   │                                                  │
   │──────────── connect ────────────────────────────>│
   │<─────────── connect_ack ─────────────────────────│
   │                                                  │
   │──────────── subscribe (biomarkers) ─────────────>│
   │<─────────── subscribe_ack ───────────────────────│
   │                                                  │
   │<─────────── biomarker (stream) ──────────────────│
   │<─────────── biomarker (stream) ──────────────────│
   │<─────────── biomarker (stream) ──────────────────│
   │                                                  │
   │──────────── ping ───────────────────────────────>│
   │<─────────── pong ────────────────────────────────│
   │                                                  │
   │──────────── disconnect ─────────────────────────>│
   │<─────────── disconnect_ack ──────────────────────│
   │                                                  │
```

---

## 4. Message Format

### 4.1 Base Message Structure

```json
{
  "protocol": "wia-health",
  "version": "1.0.0",
  "messageId": "uuid-v4",
  "timestamp": 1702483200000,
  "type": "message_type",
  "correlationId": "optional-uuid",
  "payload": {}
}
```

### 4.2 Field Definitions

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `protocol` | string | Yes | Always "wia-health" |
| `version` | string | Yes | Protocol version (semver) |
| `messageId` | uuid | Yes | Unique message identifier |
| `timestamp` | integer | Yes | Unix timestamp (milliseconds) |
| `type` | string | Yes | Message type (see 5.0) |
| `correlationId` | uuid | No | Reference to related message |
| `payload` | object | Yes | Type-specific data |

### 4.3 Example Message

```json
{
  "protocol": "wia-health",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": 1702483200000,
  "type": "biomarker",
  "correlationId": null,
  "payload": {
    "subjectId": "patient-001",
    "dataType": "heart_rate",
    "value": 72,
    "unit": "bpm",
    "source": "apple_watch",
    "quality": 0.95
  }
}
```

---

## 5. Message Types

### 5.1 Connection Messages

#### connect

Client initiates connection.

```json
{
  "type": "connect",
  "payload": {
    "clientId": "app-12345",
    "clientName": "My Health App",
    "clientVersion": "2.1.0",
    "capabilities": ["biomarkers", "genomics", "digital_twin"],
    "subscriptions": ["heart_rate", "glucose", "activity"],
    "options": {
      "streamRate": 60,
      "compression": false,
      "format": "json"
    },
    "auth": {
      "type": "bearer",
      "token": "eyJhbGciOiJSUzI1NiIs..."
    }
  }
}
```

#### connect_ack

Server acknowledges connection.

```json
{
  "type": "connect_ack",
  "payload": {
    "sessionId": "session-uuid",
    "serverId": "wia-health-us-east-1",
    "serverVersion": "1.0.0",
    "capabilities": ["biomarkers", "genomics", "digital_twin", "simulation"],
    "settings": {
      "maxStreamRate": 100,
      "keepAliveInterval": 30000,
      "sessionTimeout": 3600000
    }
  }
}
```

#### disconnect

Clean disconnection request.

```json
{
  "type": "disconnect",
  "payload": {
    "reason": "user_logout",
    "code": 1000
  }
}
```

### 5.2 Subscription Messages

#### subscribe

Subscribe to data streams.

```json
{
  "type": "subscribe",
  "payload": {
    "streams": [
      {
        "type": "biomarker",
        "filter": {
          "markers": ["heart_rate", "hrv", "blood_pressure"],
          "sources": ["apple_watch", "withings"],
          "minQuality": 0.8
        },
        "options": {
          "rate": 60,
          "aggregation": "none"
        }
      },
      {
        "type": "aging_clock",
        "filter": {
          "clockTypes": ["grimAge", "phenoAge"]
        }
      }
    ]
  }
}
```

#### subscribe_ack

```json
{
  "type": "subscribe_ack",
  "payload": {
    "subscriptionId": "sub-uuid",
    "streams": [
      {
        "type": "biomarker",
        "status": "active",
        "streamId": "stream-001"
      },
      {
        "type": "aging_clock",
        "status": "active",
        "streamId": "stream-002"
      }
    ]
  }
}
```

#### unsubscribe

```json
{
  "type": "unsubscribe",
  "payload": {
    "streamIds": ["stream-001", "stream-002"]
  }
}
```

### 5.3 Data Messages

#### biomarker

Real-time biomarker data.

```json
{
  "type": "biomarker",
  "payload": {
    "streamId": "stream-001",
    "subjectId": "patient-001",
    "sequence": 12345,
    "data": {
      "marker": "heart_rate",
      "value": 72,
      "unit": "bpm",
      "timestamp": 1702483200000,
      "source": {
        "device": "apple_watch",
        "model": "Series 9",
        "firmware": "10.2"
      },
      "quality": 0.98,
      "metadata": {
        "activity": "resting",
        "position": "sitting"
      }
    }
  }
}
```

#### profile_update

Health profile changes.

```json
{
  "type": "profile_update",
  "payload": {
    "subjectId": "patient-001",
    "updateType": "biomarkers",
    "changes": {
      "path": "biomarkers.agingClocks",
      "operation": "update",
      "data": {
        "biologicalAge": 36.5,
        "clockType": "grimAge",
        "calculatedAt": "2025-12-14T10:30:00Z"
      }
    },
    "version": 42
  }
}
```

#### simulation_result

Digital twin simulation results.

```json
{
  "type": "simulation_result",
  "payload": {
    "simulationId": "sim-uuid",
    "subjectId": "patient-001",
    "simulationType": "aging",
    "status": "completed",
    "results": {
      "predictions": [
        {
          "outcome": "biological_age_10y",
          "value": 45.2,
          "confidence": 0.85,
          "timeframe": "10 years"
        }
      ],
      "recommendations": [
        {
          "intervention": "NMN supplementation",
          "expectedBenefit": -2.5,
          "confidence": 0.7
        }
      ]
    },
    "duration": 1250
  }
}
```

### 5.4 Control Messages

#### ping / pong

Keep-alive heartbeat.

```json
{
  "type": "ping",
  "payload": {
    "timestamp": 1702483200000
  }
}
```

```json
{
  "type": "pong",
  "payload": {
    "timestamp": 1702483200000,
    "serverTime": 1702483200005,
    "latency": 5
  }
}
```

#### alert

Health alerts and notifications.

```json
{
  "type": "alert",
  "payload": {
    "alertId": "alert-uuid",
    "subjectId": "patient-001",
    "severity": "warning",
    "category": "biomarker",
    "title": "Elevated heart rate detected",
    "message": "Heart rate above 100 bpm for 30+ minutes while resting",
    "data": {
      "marker": "heart_rate",
      "value": 105,
      "threshold": 100,
      "duration": 1800
    },
    "actions": [
      {
        "type": "acknowledge",
        "label": "Dismiss"
      },
      {
        "type": "view_details",
        "label": "View Details",
        "url": "/biomarkers/heart_rate"
      }
    ]
  }
}
```

### 5.5 Error Messages

#### error

```json
{
  "type": "error",
  "payload": {
    "code": 2001,
    "name": "DEVICE_NOT_FOUND",
    "message": "Device 'apple_watch_001' not connected",
    "details": {
      "deviceId": "apple_watch_001",
      "lastSeen": "2025-12-14T09:00:00Z"
    },
    "recoverable": true,
    "retryAfter": 5000
  }
}
```

---

## 6. Error Codes

### 6.1 Connection Errors (1xxx)

| Code | Name | Description |
|------|------|-------------|
| 1000 | CONNECTION_CLOSED | Normal closure |
| 1001 | CONNECTION_LOST | Unexpected disconnection |
| 1002 | PROTOCOL_ERROR | Invalid message format |
| 1003 | UNSUPPORTED_VERSION | Protocol version mismatch |
| 1004 | SESSION_EXPIRED | Session timed out |
| 1005 | RATE_LIMITED | Too many requests |

### 6.2 Device Errors (2xxx)

| Code | Name | Description |
|------|------|-------------|
| 2001 | DEVICE_NOT_FOUND | Device not connected |
| 2002 | DEVICE_BUSY | Device in use |
| 2003 | DEVICE_ERROR | Device malfunction |
| 2004 | DEVICE_UNSUPPORTED | Device not compatible |
| 2005 | SYNC_FAILED | Data sync failed |

### 6.3 Authentication Errors (3xxx)

| Code | Name | Description |
|------|------|-------------|
| 3001 | AUTH_FAILED | Authentication failed |
| 3002 | TOKEN_EXPIRED | Auth token expired |
| 3003 | PERMISSION_DENIED | Insufficient permissions |
| 3004 | INVALID_TOKEN | Malformed token |

### 6.4 Data Errors (4xxx)

| Code | Name | Description |
|------|------|-------------|
| 4001 | INVALID_DATA | Data validation failed |
| 4002 | PROFILE_NOT_FOUND | Health profile not found |
| 4003 | STREAM_NOT_FOUND | Subscription not found |
| 4004 | SIMULATION_FAILED | Simulation error |

### 6.5 Server Errors (5xxx)

| Code | Name | Description |
|------|------|-------------|
| 5001 | INTERNAL_ERROR | Server internal error |
| 5002 | SERVICE_UNAVAILABLE | Service temporarily unavailable |
| 5003 | TIMEOUT | Operation timed out |

---

## 7. Connection State Machine

```
                    ┌─────────────┐
                    │DISCONNECTED │
                    └──────┬──────┘
                           │ connect()
                           ▼
                    ┌─────────────┐
        ┌──────────>│ CONNECTING  │
        │           └──────┬──────┘
        │                  │ connect_ack
        │                  ▼
        │           ┌─────────────┐
        │  ┌───────>│  CONNECTED  │<──────────┐
        │  │        └──────┬──────┘           │
        │  │               │                  │
        │  │ reconnect     │ error/timeout    │ reconnect
        │  │ success       │                  │ success
        │  │               ▼                  │
        │  │        ┌─────────────┐           │
        │  └────────│RECONNECTING ├───────────┘
        │           └──────┬──────┘
        │                  │ max retries
        │                  ▼
        │           ┌─────────────┐
        └───────────│   ERROR     │
                    └──────┬──────┘
                           │ reset()
                           ▼
                    ┌─────────────┐
                    │DISCONNECTED │
                    └─────────────┘
```

### 7.1 State Transitions

| From | Event | To |
|------|-------|-----|
| DISCONNECTED | connect() | CONNECTING |
| CONNECTING | connect_ack | CONNECTED |
| CONNECTING | error/timeout | RECONNECTING |
| CONNECTED | disconnect | DISCONNECTED |
| CONNECTED | error/timeout | RECONNECTING |
| RECONNECTING | connect_ack | CONNECTED |
| RECONNECTING | max_retries | ERROR |
| ERROR | reset() | DISCONNECTED |

### 7.2 Reconnection Strategy

```rust
fn reconnect_delay(attempt: u32) -> Duration {
    let base = Duration::from_secs(1);
    let max = Duration::from_secs(30);
    let jitter = rand::random::<f64>() * 0.3;

    let delay = base * 2u32.pow(attempt.min(5));
    let delay = delay.min(max);
    delay + Duration::from_secs_f64(delay.as_secs_f64() * jitter)
}
```

---

## 8. Transport Layer

### 8.1 Transport Interface

```rust
#[async_trait]
pub trait Transport: Send + Sync {
    async fn connect(&mut self, url: &str) -> Result<()>;
    async fn disconnect(&mut self) -> Result<()>;
    async fn send(&mut self, message: &Message) -> Result<()>;
    async fn receive(&mut self) -> Result<Message>;
    fn is_connected(&self) -> bool;
    fn transport_type(&self) -> TransportType;
}

pub enum TransportType {
    WebSocket,
    Mqtt,
    Ble,
    Http,
}
```

### 8.2 WebSocket Transport

#### Connection URL Format

```
wss://api.wia.live/health/v1/stream?token={jwt}
```

#### Subprotocol

```
Sec-WebSocket-Protocol: wia-health-v1
```

#### Frame Types

| Frame | Use |
|-------|-----|
| Text | JSON messages |
| Binary | Protocol Buffer messages (optional) |
| Ping/Pong | WebSocket keep-alive |
| Close | Connection termination |

### 8.3 MQTT Transport

#### Topic Structure

```
wia/health/{subject_id}/biomarkers/{marker_type}
wia/health/{subject_id}/profile/updates
wia/health/{subject_id}/alerts
wia/health/{subject_id}/simulation/results
```

#### QoS Levels

| Data Type | QoS |
|-----------|-----|
| Biomarker streams | 0 (at most once) |
| Profile updates | 1 (at least once) |
| Alerts | 2 (exactly once) |

### 8.4 BLE Transport

#### Service UUID

```
WIA Health Service: 0000FE40-0000-1000-8000-00805F9B34FB
```

#### Characteristics

| UUID | Name | Properties |
|------|------|------------|
| FE41 | Control | Write |
| FE42 | Data Stream | Notify |
| FE43 | Status | Read |

---

## 9. Security

### 9.1 Transport Security

- **TLS 1.3 Required**: All connections must use TLS 1.3+
- **Certificate Pinning**: Optional for mobile apps
- **Perfect Forward Secrecy**: Required

### 9.2 Authentication

#### OAuth 2.0 / SMART on FHIR

```json
{
  "auth": {
    "type": "bearer",
    "token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
    "refresh_token": "dGhpcyBpcyBhIHJlZnJlc2ggdG9rZW4..."
  }
}
```

#### Scopes

| Scope | Access |
|-------|--------|
| `health.read` | Read health data |
| `health.write` | Write health data |
| `biomarkers.stream` | Subscribe to streams |
| `simulation.run` | Run simulations |
| `profile.manage` | Manage profiles |

### 9.3 Data Privacy

- End-to-end encryption for sensitive data
- Data anonymization options
- Consent-based access control
- Audit logging

---

## 10. Quality of Service

### 10.1 Message Delivery

| Level | Guarantee | Use Case |
|-------|-----------|----------|
| 0 | At most once | High-frequency biomarkers |
| 1 | At least once | Profile updates |
| 2 | Exactly once | Critical alerts |

### 10.2 Ordering

- Messages within a stream are ordered by sequence number
- Cross-stream ordering not guaranteed
- Reordering buffer for out-of-order messages

### 10.3 Rate Limits

| Tier | Messages/sec | Streams | Connections |
|------|--------------|---------|-------------|
| Free | 10 | 3 | 1 |
| Pro | 100 | 10 | 5 |
| Enterprise | 1000 | Unlimited | 50 |

---

## 11. FHIR Integration

### 11.1 Resource Mapping

| WIA Type | FHIR Resource |
|----------|---------------|
| Subject | Patient |
| Biomarker | Observation |
| AgingClock | Observation (component) |
| Intervention | MedicationRequest / Procedure |
| Alert | Flag / CommunicationRequest |

### 11.2 Subscription

FHIR R5 Subscriptions for real-time updates:

```json
{
  "resourceType": "Subscription",
  "status": "active",
  "topic": "http://wia.live/health/SubscriptionTopic/biomarker-update",
  "channel": {
    "type": "websocket",
    "endpoint": "wss://api.wia.live/health/fhir/websocket"
  }
}
```

---

## 12. Examples

### 12.1 Full Connection Flow

```json
// 1. Client connects
{
  "protocol": "wia-health",
  "version": "1.0.0",
  "messageId": "msg-001",
  "timestamp": 1702483200000,
  "type": "connect",
  "payload": {
    "clientId": "health-app-001",
    "capabilities": ["biomarkers", "digital_twin"],
    "auth": {
      "type": "bearer",
      "token": "eyJhbGciOiJSUzI1NiIs..."
    }
  }
}

// 2. Server acknowledges
{
  "protocol": "wia-health",
  "version": "1.0.0",
  "messageId": "msg-002",
  "timestamp": 1702483200005,
  "type": "connect_ack",
  "correlationId": "msg-001",
  "payload": {
    "sessionId": "session-12345",
    "serverId": "wia-health-us-1"
  }
}

// 3. Client subscribes
{
  "protocol": "wia-health",
  "version": "1.0.0",
  "messageId": "msg-003",
  "timestamp": 1702483200010,
  "type": "subscribe",
  "payload": {
    "streams": [
      {
        "type": "biomarker",
        "filter": { "markers": ["heart_rate", "hrv"] }
      }
    ]
  }
}

// 4. Server confirms subscription
{
  "protocol": "wia-health",
  "version": "1.0.0",
  "messageId": "msg-004",
  "timestamp": 1702483200015,
  "type": "subscribe_ack",
  "correlationId": "msg-003",
  "payload": {
    "subscriptionId": "sub-001",
    "streams": [
      { "type": "biomarker", "streamId": "stream-001", "status": "active" }
    ]
  }
}

// 5. Server streams data
{
  "protocol": "wia-health",
  "version": "1.0.0",
  "messageId": "msg-005",
  "timestamp": 1702483201000,
  "type": "biomarker",
  "payload": {
    "streamId": "stream-001",
    "sequence": 1,
    "data": {
      "marker": "heart_rate",
      "value": 72,
      "unit": "bpm"
    }
  }
}
```

---

## Appendix A: Message Schema (JSON Schema)

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://wia.live/health/protocol/message.schema.json",
  "title": "WIA Health Protocol Message",
  "type": "object",
  "required": ["protocol", "version", "messageId", "timestamp", "type", "payload"],
  "properties": {
    "protocol": {
      "type": "string",
      "const": "wia-health"
    },
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$"
    },
    "messageId": {
      "type": "string",
      "format": "uuid"
    },
    "timestamp": {
      "type": "integer",
      "minimum": 0
    },
    "type": {
      "type": "string",
      "enum": [
        "connect", "connect_ack", "disconnect", "disconnect_ack",
        "subscribe", "subscribe_ack", "unsubscribe", "unsubscribe_ack",
        "biomarker", "profile_update", "simulation_result",
        "ping", "pong", "alert", "error"
      ]
    },
    "correlationId": {
      "type": ["string", "null"],
      "format": "uuid"
    },
    "payload": {
      "type": "object"
    }
  }
}
```

---

## Appendix B: Protocol Buffers Definition

```protobuf
syntax = "proto3";
package wia.health.protocol;

message Message {
  string protocol = 1;
  string version = 2;
  string message_id = 3;
  int64 timestamp = 4;
  MessageType type = 5;
  optional string correlation_id = 6;
  bytes payload = 7;
}

enum MessageType {
  UNKNOWN = 0;
  CONNECT = 1;
  CONNECT_ACK = 2;
  DISCONNECT = 3;
  DISCONNECT_ACK = 4;
  SUBSCRIBE = 5;
  SUBSCRIBE_ACK = 6;
  UNSUBSCRIBE = 7;
  UNSUBSCRIBE_ACK = 8;
  BIOMARKER = 9;
  PROFILE_UPDATE = 10;
  SIMULATION_RESULT = 11;
  PING = 12;
  PONG = 13;
  ALERT = 14;
  ERROR = 15;
}
```

---

**弘益人間** - Benefit All Humanity
