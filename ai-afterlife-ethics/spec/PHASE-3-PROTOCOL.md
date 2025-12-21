# WIA AI Afterlife Ethics Protocol Standard
## Phase 3 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #64748B (Slate)

---

## Table of Contents

1. [Overview](#overview)
2. [Terminology](#terminology)
3. [Protocol Architecture](#protocol-architecture)
4. [Transport Layer](#transport-layer)
5. [Message Format](#message-format)
6. [Connection Lifecycle](#connection-lifecycle)
7. [Security](#security)
8. [Ethics Operations Protocol](#ethics-operations-protocol)
9. [Real-time Monitoring Protocol](#real-time-monitoring-protocol)
10. [Error Recovery](#error-recovery)
11. [Implementation Examples](#implementation-examples)
12. [References](#references)

---

## Overview

### 1.1 Purpose

The WIA AI Afterlife Ethics Protocol Standard defines the communication protocols for real-time, secure, and ethically-compliant interactions with posthumous AI personas. This Phase 3 specification builds upon Phase 1 (Data Format) and Phase 2 (API Interface) to enable distributed, event-driven ethics enforcement.

**Core Objectives**:
- Enable real-time psychological safety monitoring
- Provide secure, encrypted communication channels
- Support distributed consent verification
- Enable multi-party ethics governance
- Ensure tamper-proof audit trails

### 1.2 Scope

This standard defines:

| Component | Description |
|-----------|-------------|
| **Transport Layer** | WebSocket, MQTT, gRPC communication |
| **Message Format** | Standardized protocol messages |
| **Security** | TLS 1.3, end-to-end encryption |
| **Event Streaming** | Real-time ethics events |
| **Consensus Protocol** | Multi-party approval mechanisms |

### 1.3 Protocol Stack

```
┌─────────────────────────────────────────┐
│     Application Layer (Phase 2 API)     │
├─────────────────────────────────────────┤
│    Ethics Operations Protocol (Phase 3) │
├─────────────────────────────────────────┤
│   Message Format & Serialization        │
├─────────────────────────────────────────┤
│   Transport (WebSocket/MQTT/gRPC)       │
├─────────────────────────────────────────┤
│   Security (TLS 1.3, E2EE)              │
├─────────────────────────────────────────┤
│   Network Layer (TCP/IP)                │
└─────────────────────────────────────────┘
```

---

## Terminology

### 2.1 Core Terms

| Term | Definition |
|------|------------|
| **Protocol Message** | Structured communication unit |
| **Transport Channel** | Communication pathway (WebSocket/MQTT/gRPC) |
| **Session** | Authenticated connection instance |
| **Event Stream** | Real-time event notification channel |
| **Consensus Round** | Multi-party approval cycle |
| **Heartbeat** | Connection liveness signal |

### 2.2 Message Types

| Type | Description | Direction |
|------|-------------|-----------|
| `CONSENT_REQUEST` | Request consent validation | Client → Server |
| `CONSENT_RESPONSE` | Consent validation result | Server → Client |
| `PERSONA_INTERACTION` | AI persona interaction data | Bidirectional |
| `PSYCHOLOGICAL_ALERT` | Safety concern notification | Server → Client |
| `COMMERCIAL_AUDIT` | Commercial usage audit | Client → Server |
| `EVENT_NOTIFICATION` | Real-time event broadcast | Server → Clients |

---

## Protocol Architecture

### 3.1 Communication Patterns

#### 3.1.1 Request-Response

```
Client                          Server
  │                               │
  ├─── CONSENT_REQUEST ──────────>│
  │                               │
  │<──── CONSENT_RESPONSE ────────┤
  │                               │
```

#### 3.1.2 Publish-Subscribe

```
Publisher                  Broker                  Subscriber
  │                          │                          │
  ├─── PUBLISH_EVENT ───────>│                          │
  │                          │                          │
  │                          ├─── EVENT_NOTIFICATION ──>│
  │                          │                          │
```

#### 3.1.3 Streaming

```
Client                          Server
  │                               │
  ├─── START_MONITORING ─────────>│
  │                               │
  │<──── SAFETY_UPDATE ───────────┤
  │<──── SAFETY_UPDATE ───────────┤
  │<──── SAFETY_UPDATE ───────────┤
  │                               │
  ├─── STOP_MONITORING ──────────>│
```

### 3.2 Protocol Modes

| Mode | Transport | Use Case |
|------|-----------|----------|
| **Interactive** | WebSocket | Real-time persona interactions |
| **Monitoring** | MQTT | Psychological safety monitoring |
| **Batch** | gRPC | Bulk consent processing |
| **Streaming** | Server-Sent Events | One-way event notifications |

---

## Transport Layer

### 4.1 WebSocket Transport

**Primary transport for real-time interactive sessions**

#### 4.1.1 Connection URL

```
wss://api.wia.live/ai-afterlife-ethics/v1/ws
```

#### 4.1.2 Subprotocols

```
Sec-WebSocket-Protocol: wia.afterlife.ethics.v1
```

#### 4.1.3 Connection Headers

```http
GET /ai-afterlife-ethics/v1/ws HTTP/1.1
Host: api.wia.live
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==
Sec-WebSocket-Version: 13
Sec-WebSocket-Protocol: wia.afterlife.ethics.v1
Authorization: Bearer <token>
X-Ethics-Id: ETHICS-2025-000001
```

#### 4.1.4 Frame Types

| Opcode | Type | Description |
|--------|------|-------------|
| `0x1` | Text | JSON protocol messages |
| `0x2` | Binary | Encrypted binary payloads |
| `0x8` | Close | Connection termination |
| `0x9` | Ping | Heartbeat request |
| `0xA` | Pong | Heartbeat response |

#### 4.1.5 WebSocket Example

```typescript
const ws = new WebSocket(
  'wss://api.wia.live/ai-afterlife-ethics/v1/ws',
  'wia.afterlife.ethics.v1',
  {
    headers: {
      'Authorization': 'Bearer <token>',
      'X-Ethics-Id': 'ETHICS-2025-000001'
    }
  }
);

ws.on('open', () => {
  ws.send(JSON.stringify({
    type: 'SUBSCRIBE',
    topics: ['psychological_alerts', 'consent_updates']
  }));
});

ws.on('message', (data) => {
  const message = JSON.parse(data);
  handleProtocolMessage(message);
});
```

### 4.2 MQTT Transport

**Optimized for distributed monitoring and IoT devices**

#### 4.2.1 Connection Parameters

```
Broker: mqtts://mqtt.wia.live:8883
Protocol: MQTT 5.0
Keep-Alive: 60 seconds
Clean Session: false
```

#### 4.2.2 Topic Structure

```
wia/afterlife-ethics/v1/{ethicsId}/{messageType}

Examples:
- wia/afterlife-ethics/v1/ETHICS-2025-000001/psychological/alerts
- wia/afterlife-ethics/v1/ETHICS-2025-000001/consent/updates
- wia/afterlife-ethics/v1/ETHICS-2025-000001/interactions/logs
```

#### 4.2.3 QoS Levels

| QoS | Level | Use Case |
|-----|-------|----------|
| `0` | At most once | Non-critical metrics |
| `1` | At least once | Interaction logs |
| `2` | Exactly once | Consent updates, safety alerts |

#### 4.2.4 MQTT Example

```python
import paho.mqtt.client as mqtt

client = mqtt.Client(protocol=mqtt.MQTTv5)
client.username_pw_set("api_key", "secret_key")
client.tls_set()

def on_connect(client, userdata, flags, rc, properties):
    # Subscribe to psychological alerts with QoS 2
    client.subscribe(
        "wia/afterlife-ethics/v1/ETHICS-2025-000001/psychological/alerts",
        qos=2
    )

def on_message(client, userdata, msg):
    alert = json.loads(msg.payload.decode())
    handle_psychological_alert(alert)

client.on_connect = on_connect
client.on_message = on_message
client.connect("mqtt.wia.live", 8883, 60)
client.loop_forever()
```

### 4.3 gRPC Transport

**High-performance for batch operations**

#### 4.3.1 Service Definition

```protobuf
syntax = "proto3";

package wia.afterlife.ethics.v1;

service AfterlifeEthicsService {
  // Consent operations
  rpc CreateConsent(ConsentRequest) returns (ConsentResponse);
  rpc ValidateConsent(ValidateConsentRequest) returns (ValidateConsentResponse);
  rpc RevokeConsent(RevokeConsentRequest) returns (RevokeConsentResponse);

  // Persona operations
  rpc CreatePersona(PersonaRequest) returns (PersonaResponse);
  rpc InteractWithPersona(stream InteractionMessage) returns (stream InteractionResponse);

  // Monitoring
  rpc MonitorSafety(SafetyMonitorRequest) returns (stream SafetyUpdate);
  rpc ReportIncident(IncidentReport) returns (IncidentResponse);

  // Batch operations
  rpc BatchValidateConsent(stream ValidateConsentRequest) returns (stream ValidateConsentResponse);
}

message ConsentRequest {
  string data_subject_id = 1;
  string consent_level = 2;
  repeated WitnessSignature witnesses = 3;
  google.protobuf.Timestamp expiry_date = 4;
}

message ConsentResponse {
  string consent_id = 1;
  string status = 2;
  google.protobuf.Timestamp created = 3;
}
```

#### 4.3.2 gRPC Example

```python
import grpc
from wia.afterlife.ethics.v1 import afterlife_ethics_pb2_grpc, afterlife_ethics_pb2

channel = grpc.secure_channel(
    'api.wia.live:443',
    grpc.ssl_channel_credentials()
)
stub = afterlife_ethics_pb2_grpc.AfterlifeEthicsServiceStub(channel)

# Create consent
response = stub.CreateConsent(
    afterlife_ethics_pb2.ConsentRequest(
        data_subject_id="SUBJ-2025-001",
        consent_level="family_interaction",
        witnesses=[...]
    )
)

print(f"Consent created: {response.consent_id}")
```

---

## Message Format

### 5.1 Base Message Structure

All protocol messages follow this base structure:

```json
{
  "version": "1.0.0",
  "messageId": "MSG-2025-000001",
  "timestamp": "2025-01-15T10:00:00Z",
  "type": "CONSENT_REQUEST",
  "ethicsId": "ETHICS-2025-000001",
  "sender": {
    "type": "client",
    "id": "CLIENT-001",
    "authenticated": true
  },
  "payload": {},
  "metadata": {
    "correlationId": "CORR-001",
    "priority": "high",
    "ttl": 300
  },
  "signature": "ed25519:..."
}
```

### 5.2 Message Types

#### 5.2.1 CONSENT_REQUEST

```json
{
  "type": "CONSENT_REQUEST",
  "messageId": "MSG-2025-000001",
  "timestamp": "2025-01-15T10:00:00Z",
  "ethicsId": "ETHICS-2025-000001",
  "payload": {
    "action": "validate",
    "consentId": "CONSENT-2025-000001",
    "requestedUsage": "family_interaction",
    "requestor": "USER-123"
  }
}
```

#### 5.2.2 CONSENT_RESPONSE

```json
{
  "type": "CONSENT_RESPONSE",
  "messageId": "MSG-2025-000002",
  "timestamp": "2025-01-15T10:00:01Z",
  "ethicsId": "ETHICS-2025-000001",
  "payload": {
    "valid": true,
    "consentLevel": "family_interaction",
    "restrictions": ["non_commercial", "time_limited"],
    "expiryDate": "2025-12-31T23:59:59Z"
  },
  "metadata": {
    "correlationId": "CORR-001"
  }
}
```

#### 5.2.3 PERSONA_INTERACTION

```json
{
  "type": "PERSONA_INTERACTION",
  "messageId": "MSG-2025-000003",
  "timestamp": "2025-01-15T10:05:23Z",
  "payload": {
    "sessionId": "SESSION-2025-000001",
    "personaId": "PERSONA-2025-000001",
    "userId": "USER-123",
    "interaction": {
      "type": "message",
      "content": "encrypted:aes256:...",
      "sentiment": "positive"
    }
  }
}
```

#### 5.2.4 PSYCHOLOGICAL_ALERT

```json
{
  "type": "PSYCHOLOGICAL_ALERT",
  "messageId": "MSG-2025-000004",
  "timestamp": "2025-01-15T10:10:45Z",
  "payload": {
    "alertLevel": "warning",
    "sessionId": "SESSION-2025-000001",
    "riskScore": 0.65,
    "factors": [
      {
        "factor": "emotional_distress",
        "severity": "moderate"
      }
    ],
    "recommendations": [
      "reduce_session_duration",
      "notify_guardian"
    ]
  },
  "metadata": {
    "priority": "high",
    "requiresAcknowledgment": true
  }
}
```

#### 5.2.5 COMMERCIAL_AUDIT

```json
{
  "type": "COMMERCIAL_AUDIT",
  "messageId": "MSG-2025-000005",
  "timestamp": "2025-01-15T10:00:00Z",
  "payload": {
    "licenseId": "LICENSE-2025-000001",
    "period": "2025-01",
    "revenue": 4200,
    "interactions": 150,
    "complianceStatus": "compliant"
  }
}
```

#### 5.2.6 EVENT_NOTIFICATION

```json
{
  "type": "EVENT_NOTIFICATION",
  "messageId": "MSG-2025-000006",
  "timestamp": "2025-01-15T10:00:00Z",
  "payload": {
    "eventType": "consent_revoked",
    "ethicsId": "ETHICS-2025-000001",
    "consentId": "CONSENT-2025-000001",
    "revokedBy": "AGENT-001",
    "reason": "Family request",
    "effectiveDate": "2025-02-01T00:00:00Z"
  }
}
```

### 5.3 Message Encoding

| Encoding | Content-Type | Use Case |
|----------|--------------|----------|
| JSON | `application/json` | Standard messages |
| MessagePack | `application/msgpack` | Compact binary |
| Protocol Buffers | `application/protobuf` | High-performance gRPC |
| Encrypted JSON | `application/encrypted+json` | Sensitive data |

---

## Connection Lifecycle

### 6.1 Connection States

```
┌──────────┐
│  CLOSED  │
└────┬─────┘
     │ connect()
     ▼
┌──────────┐
│CONNECTING│
└────┬─────┘
     │ on_open
     ▼
┌──────────┐     authenticate()      ┌──────────────┐
│  OPEN    │ ───────────────────────>│AUTHENTICATING│
└────┬─────┘                         └──────┬───────┘
     │                                      │ on_auth
     │◄─────────────────────────────────────┘
     ▼
┌──────────┐
│AUTHORIZED│
└────┬─────┘
     │ close() / error
     ▼
┌──────────┐
│  CLOSED  │
└──────────┘
```

### 6.2 Connection Establishment

```typescript
// 1. Establish connection
const connection = await ethicsProtocol.connect({
  transport: 'websocket',
  url: 'wss://api.wia.live/ai-afterlife-ethics/v1/ws'
});

// 2. Authenticate
await connection.authenticate({
  token: 'Bearer <token>',
  ethicsId: 'ETHICS-2025-000001'
});

// 3. Subscribe to events
await connection.subscribe([
  'psychological_alerts',
  'consent_updates'
]);

// 4. Connection is ready
console.log('Connection state:', connection.state); // AUTHORIZED
```

### 6.3 Heartbeat Protocol

**Keep-alive mechanism to detect connection failures**

```
Client                          Server
  │                               │
  ├─── PING ─────────────────────>│
  │                               │
  │<──── PONG ─────────────────────┤
  │                               │
  │    ... 30 seconds ...         │
  │                               │
  ├─── PING ─────────────────────>│
  │                               │
  │<──── PONG ─────────────────────┤
  │                               │
```

**Heartbeat Message**:
```json
{
  "type": "HEARTBEAT",
  "messageId": "MSG-2025-HB-001",
  "timestamp": "2025-01-15T10:00:00Z"
}
```

**Configuration**:
- Interval: 30 seconds
- Timeout: 90 seconds (3 missed heartbeats)
- Action on timeout: Reconnect with exponential backoff

### 6.4 Graceful Shutdown

```typescript
// 1. Unsubscribe from events
await connection.unsubscribe(['psychological_alerts']);

// 2. End active sessions
await connection.send({
  type: 'END_ALL_SESSIONS',
  reason: 'Client shutdown'
});

// 3. Close connection
await connection.close({
  code: 1000,
  reason: 'Normal closure'
});
```

---

## Security

### 7.1 Transport Security

#### 7.1.1 TLS Configuration

```
Protocol: TLS 1.3
Cipher Suites:
  - TLS_AES_256_GCM_SHA384
  - TLS_CHACHA20_POLY1305_SHA256
  - TLS_AES_128_GCM_SHA256

Certificate Requirements:
  - X.509 certificates with ECC or RSA ≥ 2048-bit
  - Certificate pinning for production
  - OCSP stapling enabled
```

#### 7.1.2 WebSocket Security

```typescript
const secureWs = new WebSocket('wss://api.wia.live/...', {
  rejectUnauthorized: true,
  ca: [certificateAuthorityCert],
  cert: clientCertificate,
  key: clientPrivateKey
});
```

### 7.2 Message-Level Security

#### 7.2.1 Message Signing

All messages must be signed with Ed25519 keys:

```json
{
  "type": "CONSENT_REQUEST",
  "payload": {...},
  "signature": "ed25519:base64-encoded-signature",
  "publicKey": "ed25519:base64-encoded-public-key"
}
```

**Signature Computation**:
```typescript
const message = {
  type: "CONSENT_REQUEST",
  payload: {...}
};

const messageBytes = canonicalJSON.stringify(message);
const signature = ed25519.sign(messageBytes, privateKey);

message.signature = `ed25519:${base64.encode(signature)}`;
```

#### 7.2.2 End-to-End Encryption

Sensitive payloads use AES-256-GCM encryption:

```json
{
  "type": "PERSONA_INTERACTION",
  "payload": {
    "sessionId": "SESSION-2025-000001",
    "interaction": {
      "encrypted": true,
      "algorithm": "AES-256-GCM",
      "ciphertext": "base64-encoded-ciphertext",
      "nonce": "base64-encoded-nonce",
      "tag": "base64-encoded-tag"
    }
  }
}
```

### 7.3 Authentication

#### 7.3.1 OAuth 2.0 Flow

```
1. Client requests authorization code
   GET /oauth/authorize?client_id=...&response_type=code

2. User grants permission

3. Client exchanges code for token
   POST /oauth/token
   {
     "grant_type": "authorization_code",
     "code": "...",
     "client_id": "...",
     "client_secret": "..."
   }

4. Client uses access token
   Authorization: Bearer <access_token>
```

#### 7.3.2 API Key Authentication

```http
X-API-Key: wia_live_abc123...
X-Ethics-Id: ETHICS-2025-000001
```

#### 7.3.3 JWT Token Structure

```json
{
  "header": {
    "alg": "ES256",
    "typ": "JWT"
  },
  "payload": {
    "sub": "USER-123",
    "iss": "https://wia.live",
    "aud": "ai-afterlife-ethics",
    "exp": 1737028800,
    "iat": 1737025200,
    "scope": "ethics:read ethics:write",
    "ethicsId": "ETHICS-2025-000001"
  }
}
```

### 7.4 Authorization

#### 7.4.1 Permission Scopes

| Scope | Description |
|-------|-------------|
| `ethics:read` | Read ethics records |
| `ethics:write` | Create/update ethics records |
| `consent:manage` | Manage consent records |
| `persona:create` | Create AI personas |
| `persona:interact` | Interact with personas |
| `commercial:manage` | Manage commercial licenses |
| `psychological:monitor` | Access psychological data |

#### 7.4.2 Role-Based Access Control

| Role | Scopes |
|------|--------|
| `family_member` | `ethics:read`, `persona:interact` |
| `authorized_agent` | `ethics:read`, `ethics:write`, `consent:manage` |
| `guardian` | `psychological:monitor`, `ethics:read` |
| `commercial_licensee` | `persona:interact`, `commercial:manage` |
| `admin` | All scopes |

---

## Ethics Operations Protocol

### 8.1 Consent Validation Protocol

```
Client                          Ethics Server
  │                               │
  ├─ CONSENT_REQUEST ────────────>│
  │  {                            │
  │    consentId: "...",          │
  │    requestedUsage: "..."      │
  │  }                            │
  │                               │ Validate against
  │                               │ ethics record
  │                               │
  │<─ CONSENT_RESPONSE ───────────┤
  │  {                            │
  │    valid: true,               │
  │    restrictions: [...]        │
  │  }                            │
  │                               │
```

### 8.2 Multi-Party Approval Protocol

For operations requiring multiple approvals:

```
Initiator              Ethics Server              Approvers
  │                         │                         │
  ├─ APPROVAL_REQUEST ─────>│                         │
  │                         │                         │
  │                         ├── NOTIFY_APPROVERS ────>│
  │                         │                         │
  │                         │<── APPROVAL_1 ──────────┤
  │                         │                         │
  │                         │<── APPROVAL_2 ──────────┤
  │                         │                         │
  │                         │<── APPROVAL_3 ──────────┤
  │                         │                         │
  │                         │ Check threshold (3/5)   │
  │                         │                         │
  │<─ APPROVAL_GRANTED ─────┤                         │
  │                         │                         │
```

**Approval Request**:
```json
{
  "type": "APPROVAL_REQUEST",
  "payload": {
    "requestId": "REQ-2025-000001",
    "operation": "revoke_consent",
    "ethicsId": "ETHICS-2025-000001",
    "threshold": 3,
    "totalApprovers": 5,
    "approvers": ["AGENT-001", "AGENT-002", "AGENT-003", "AGENT-004", "AGENT-005"],
    "deadline": "2025-01-22T00:00:00Z"
  }
}
```

### 8.3 Real-time Interaction Protocol

```
Client              Persona Service           Safety Monitor
  │                       │                         │
  ├─ START_SESSION ──────>│                         │
  │                       │                         │
  │                       ├── MONITOR_START ───────>│
  │                       │                         │
  ├─ INTERACTION_MSG ────>│                         │
  │                       │                         │
  │<── PERSONA_RESPONSE ──┤                         │
  │                       │                         │
  │                       │<── SAFETY_UPDATE ───────┤
  │                       │                         │
  │                       │ (Risk score: 0.3)       │
  │                       │                         │
  ├─ INTERACTION_MSG ────>│                         │
  │                       │                         │
  │<── PERSONA_RESPONSE ──┤                         │
  │                       │                         │
  │                       │<── SAFETY_ALERT ────────┤
  │                       │                         │
  │<─ SESSION_PAUSED ─────┤ (High risk detected)    │
  │                       │                         │
```

---

## Real-time Monitoring Protocol

### 9.1 Psychological Safety Monitoring

```typescript
// Subscribe to safety monitoring stream
const monitoringStream = await connection.stream({
  type: 'SAFETY_MONITORING',
  sessionId: 'SESSION-2025-000001'
});

monitoringStream.on('update', (update) => {
  console.log('Risk score:', update.riskScore);

  if (update.alertLevel === 'warning') {
    displayWarningToUser(update.recommendations);
  }

  if (update.alertLevel === 'critical') {
    pauseSession(update.sessionId);
    notifyGuardian(update);
  }
});
```

**Safety Update Message**:
```json
{
  "type": "SAFETY_UPDATE",
  "payload": {
    "sessionId": "SESSION-2025-000001",
    "timestamp": "2025-01-15T10:15:30Z",
    "riskScore": 0.45,
    "alertLevel": "normal",
    "factors": [
      {
        "factor": "session_duration",
        "value": 930,
        "contribution": 0.15
      },
      {
        "factor": "emotional_tone",
        "value": "slightly_sad",
        "contribution": 0.30
      }
    ]
  }
}
```

### 9.2 Commercial Usage Monitoring

```typescript
// Report commercial usage in real-time
const usageStream = connection.createStream('COMMERCIAL_USAGE');

usageStream.send({
  type: 'USAGE_EVENT',
  payload: {
    licenseId: 'LICENSE-2025-000001',
    eventType: 'interaction',
    timestamp: new Date().toISOString(),
    metadata: {
      userId: 'USER-456',
      duration: 300
    }
  }
});
```

---

## Error Recovery

### 10.1 Connection Recovery

```typescript
class ConnectionRecovery {
  maxRetries = 5;
  baseDelay = 1000; // 1 second

  async reconnect(attempt = 0) {
    if (attempt >= this.maxRetries) {
      throw new Error('Max reconnection attempts reached');
    }

    const delay = this.baseDelay * Math.pow(2, attempt);
    await sleep(delay);

    try {
      await this.connection.connect();
      await this.restoreSession();
    } catch (error) {
      return this.reconnect(attempt + 1);
    }
  }

  async restoreSession() {
    // Resume subscriptions
    await this.connection.subscribe(this.activeSubscriptions);

    // Replay missed messages
    const lastMessageId = this.getLastMessageId();
    await this.connection.send({
      type: 'REPLAY_MESSAGES',
      payload: { afterMessageId: lastMessageId }
    });
  }
}
```

### 10.2 Message Delivery Guarantees

**At-Least-Once Delivery**:
```typescript
class MessageSender {
  async sendWithRetry(message, maxRetries = 3) {
    let attempt = 0;

    while (attempt < maxRetries) {
      try {
        const ack = await this.sendAndWaitForAck(message);
        return ack;
      } catch (error) {
        attempt++;
        if (attempt >= maxRetries) {
          throw new Error(`Failed after ${maxRetries} attempts`);
        }
        await sleep(1000 * attempt);
      }
    }
  }

  async sendAndWaitForAck(message) {
    const correlationId = generateId();
    message.metadata.correlationId = correlationId;

    this.connection.send(message);

    return new Promise((resolve, reject) => {
      const timeout = setTimeout(() => {
        reject(new Error('Acknowledgment timeout'));
      }, 5000);

      this.once(`ack:${correlationId}`, (ack) => {
        clearTimeout(timeout);
        resolve(ack);
      });
    });
  }
}
```

### 10.3 State Synchronization

```typescript
// Periodic state sync
setInterval(async () => {
  const localState = await getLocalState();
  const remoteState = await connection.request({
    type: 'GET_STATE',
    payload: { ethicsId: 'ETHICS-2025-000001' }
  });

  if (localState.hash !== remoteState.hash) {
    await reconcileState(localState, remoteState);
  }
}, 60000); // Every minute
```

---

## Implementation Examples

### 11.1 Complete WebSocket Implementation

```typescript
import { WebSocket } from 'ws';
import { EventEmitter } from 'events';

class AfterlifeEthicsProtocol extends EventEmitter {
  private ws: WebSocket;
  private heartbeatInterval: NodeJS.Timer;
  private messageQueue: Map<string, Promise>;

  async connect(url: string, token: string) {
    this.ws = new WebSocket(url, {
      headers: {
        'Authorization': `Bearer ${token}`
      }
    });

    this.ws.on('open', () => this.onOpen());
    this.ws.on('message', (data) => this.onMessage(data));
    this.ws.on('close', () => this.onClose());
    this.ws.on('error', (error) => this.onError(error));
  }

  private onOpen() {
    this.startHeartbeat();
    this.emit('connected');
  }

  private onMessage(data: Buffer) {
    const message = JSON.parse(data.toString());

    // Handle acknowledgments
    if (message.type === 'ACK') {
      const promise = this.messageQueue.get(message.metadata.correlationId);
      if (promise) {
        promise.resolve(message);
        this.messageQueue.delete(message.metadata.correlationId);
      }
    }

    this.emit('message', message);
    this.emit(`message:${message.type}`, message);
  }

  private startHeartbeat() {
    this.heartbeatInterval = setInterval(() => {
      this.send({ type: 'HEARTBEAT' });
    }, 30000);
  }

  async send(message: any): Promise<any> {
    const correlationId = generateId();
    message.metadata = {
      ...message.metadata,
      correlationId
    };

    return new Promise((resolve, reject) => {
      this.messageQueue.set(correlationId, { resolve, reject });
      this.ws.send(JSON.stringify(message));

      setTimeout(() => {
        if (this.messageQueue.has(correlationId)) {
          this.messageQueue.delete(correlationId);
          reject(new Error('Message timeout'));
        }
      }, 5000);
    });
  }

  async subscribe(topics: string[]) {
    return this.send({
      type: 'SUBSCRIBE',
      payload: { topics }
    });
  }

  close() {
    clearInterval(this.heartbeatInterval);
    this.ws.close();
  }
}
```

### 11.2 MQTT Implementation

```python
import paho.mqtt.client as mqtt
import json
from typing import Callable, Dict

class AfterlifeEthicsMQTT:
    def __init__(self, broker: str, port: int = 8883):
        self.client = mqtt.Client(protocol=mqtt.MQTTv5)
        self.broker = broker
        self.port = port
        self.handlers: Dict[str, Callable] = {}

    def connect(self, username: str, password: str):
        self.client.username_pw_set(username, password)
        self.client.tls_set()
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.connect(self.broker, self.port, 60)
        self.client.loop_start()

    def _on_connect(self, client, userdata, flags, rc, properties):
        print(f"Connected with result code {rc}")

    def _on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = json.loads(msg.payload.decode())

        if topic in self.handlers:
            self.handlers[topic](payload)

    def subscribe(self, ethics_id: str, message_type: str, handler: Callable, qos: int = 2):
        topic = f"wia/afterlife-ethics/v1/{ethics_id}/{message_type}"
        self.client.subscribe(topic, qos=qos)
        self.handlers[topic] = handler

    def publish(self, ethics_id: str, message_type: str, payload: dict, qos: int = 2):
        topic = f"wia/afterlife-ethics/v1/{ethics_id}/{message_type}"
        self.client.publish(topic, json.dumps(payload), qos=qos)

    def disconnect(self):
        self.client.loop_stop()
        self.client.disconnect()

# Usage
mqtt_client = AfterlifeEthicsMQTT("mqtt.wia.live")
mqtt_client.connect("api_key", "secret_key")

def handle_psychological_alert(alert):
    print(f"Alert: {alert['alertLevel']}")
    if alert['alertLevel'] == 'critical':
        trigger_intervention(alert['sessionId'])

mqtt_client.subscribe(
    "ETHICS-2025-000001",
    "psychological/alerts",
    handle_psychological_alert,
    qos=2
)
```

---

## References

### Related Standards

- [WIA AI Afterlife Ethics Data Format (Phase 1)](/ai-afterlife-ethics/spec/PHASE-1-DATA-FORMAT.md)
- [WIA AI Afterlife Ethics API Interface (Phase 2)](/ai-afterlife-ethics/spec/PHASE-2-API-INTERFACE.md)

### Protocol Standards

- [RFC 6455 - WebSocket Protocol](https://tools.ietf.org/html/rfc6455)
- [MQTT Version 5.0](https://docs.oasis-open.org/mqtt/mqtt/v5.0/mqtt-v5.0.html)
- [gRPC Protocol](https://grpc.io/docs/what-is-grpc/introduction/)

### Security Standards

- [RFC 8446 - TLS 1.3](https://tools.ietf.org/html/rfc8446)
- [RFC 8032 - EdDSA](https://tools.ietf.org/html/rfc8032)
- [NIST SP 800-38D - AES-GCM](https://csrc.nist.gov/publications/detail/sp/800-38d/final)

---

<div align="center">

**WIA AI Afterlife Ethics Standard v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA**

**MIT License**

</div>
