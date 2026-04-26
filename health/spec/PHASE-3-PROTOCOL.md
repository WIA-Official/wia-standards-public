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


## Annex E — Implementation Notes for PHASE-3-PROTOCOL

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-3-PROTOCOL.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-3-PROTOCOL. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-3-protocol/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-3-PROTOCOL with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-3-PROTOCOL does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-3-PROTOCOL.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-3-PROTOCOL. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P3-PROTOCOL-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-3-PROTOCOL validation when the
  organization signs the manifest itself.
- **Multi-tenant gateway** — one shared runtime serves multiple
  tenants via header-based isolation. Typically backed by a
  rate-limited gateway (Envoy or NGINX) and a shared OAuth 2.1
  identity provider. The manifest is per-tenant; the runtime
  publishes a federation manifest that aggregates tenant manifests.
- **Federated mesh** — multiple runtimes peer to one another and
  publish their manifests to a directory service. Each peer signs
  its own manifest; the directory service signs the aggregated
  index. This is the topology used by cross-organization deployments
  that need to compose conformance.
- **Air-gapped batch** — no network connection between the runtime
  and the directory service. The runtime emits a signed evidence
  package on each batch and the operator transports the package via
  out-of-band channels. This is the topology used by regulators that
  prohibit live connectivity from sensitive environments.

Implementations declare their topology in the manifest (see Annex I).
A topology change MUST be reflected in a new manifest signature; the
prior topology's manifest remains valid for the deprecation window
described in Annex H to preserve audit traceability.
