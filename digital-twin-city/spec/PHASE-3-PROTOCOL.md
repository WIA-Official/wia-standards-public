# WIA Digital Twin City Protocol Standard
## Phase 3 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #0EA5E9 (Sky)

---

## Table of Contents

1. [Overview](#overview)
2. [Terminology](#terminology)
3. [Protocol Architecture](#protocol-architecture)
4. [Transport Layer](#transport-layer)
5. [Message Format](#message-format)
6. [Connection Lifecycle](#connection-lifecycle)
7. [Real-time Streaming](#real-time-streaming)
8. [Security](#security)
9. [Quality of Service](#quality-of-service)
10. [Protocol Examples](#protocol-examples)
11. [References](#references)

---

## Overview

### 1.1 Purpose

The WIA Digital Twin City Protocol Standard defines the communication protocols for real-time data exchange in digital twin city platforms. This Phase 3 specification builds upon the Phase 1 Data Format and Phase 2 API Interface, providing standardized protocols for IoT sensor streaming, city event notifications, and distributed system coordination.

**Core Objectives**:
- Define transport protocols for real-time sensor data streaming
- Enable efficient bi-directional communication between city systems
- Support massive IoT device connectivity (100,000+ sensors)
- Ensure low-latency event delivery for critical city operations
- Provide reliable message delivery with QoS guarantees

### 1.2 Scope

This standard defines:

| Component | Description |
|-----------|-------------|
| **Transport Layer** | WebSocket, MQTT, HTTP/2 protocols |
| **Message Format** | Binary and JSON message encoding |
| **Connection Management** | Lifecycle, reconnection, heartbeat |
| **Event Streaming** | Real-time sensor data and city events |
| **Security** | TLS 1.3, authentication, authorization |
| **QoS** | Message delivery guarantees |

### 1.3 Protocol Stack

```
┌─────────────────────────────────────┐
│     Application Layer               │
│  (Digital Twin City API)            │
├─────────────────────────────────────┤
│     Message Layer                   │
│  (WIA Protocol Messages)            │
├─────────────────────────────────────┤
│     Transport Layer                 │
│  (WebSocket / MQTT / HTTP/2)        │
├─────────────────────────────────────┤
│     Security Layer                  │
│  (TLS 1.3)                          │
├─────────────────────────────────────┤
│     Network Layer                   │
│  (TCP/IP)                           │
└─────────────────────────────────────┘
```

---

## Terminology

### 2.1 Core Terms

| Term | Definition |
|------|------------|
| **Message** | Atomic unit of communication between systems |
| **Topic** | Message routing channel (e.g., sensor/temperature) |
| **Subscription** | Client registration for specific topics |
| **QoS Level** | Message delivery guarantee level |
| **Heartbeat** | Keep-alive signal to maintain connection |
| **Backpressure** | Flow control mechanism |

### 2.2 Message Types

| Type | Direction | Description |
|------|-----------|-------------|
| `CONNECT` | Client → Server | Initiate connection |
| `CONNACK` | Server → Client | Connection acknowledgment |
| `SUBSCRIBE` | Client → Server | Subscribe to topic |
| `SUBACK` | Server → Client | Subscription acknowledgment |
| `PUBLISH` | Bi-directional | Publish message to topic |
| `PUBACK` | Bi-directional | Publish acknowledgment |
| `UNSUBSCRIBE` | Client → Server | Unsubscribe from topic |
| `PING` | Bi-directional | Heartbeat signal |
| `PONG` | Bi-directional | Heartbeat response |
| `DISCONNECT` | Bi-directional | Close connection |

---

## Protocol Architecture

### 3.1 Communication Patterns

#### 3.1.1 Request-Response (HTTP/2)

```
Client                    Server
  │                         │
  ├──── HTTP Request ──────>│
  │                         │
  │<──── HTTP Response ─────┤
  │                         │
```

**Use Cases**:
- City object CRUD operations
- Simulation execution
- Historical data queries
- Configuration management

#### 3.1.2 Publish-Subscribe (MQTT)

```
Sensor                    Broker                   Client
  │                         │                         │
  ├──── PUBLISH ───────────>│                         │
  │                         ├───── PUBLISH ──────────>│
  │                         │                         │
  ├──── PUBLISH ───────────>│                         │
  │                         ├───── PUBLISH ──────────>│
```

**Use Cases**:
- IoT sensor data streaming
- City event notifications
- Distributed system coordination
- Multi-client synchronization

#### 3.1.3 Bi-directional Streaming (WebSocket)

```
Client                    Server
  │                         │
  ├──── WS CONNECT ────────>│
  │<──── WS ACCEPT ─────────┤
  │                         │
  ├──── SUBSCRIBE ─────────>│
  │<──── SUBACK ────────────┤
  │                         │
  │<──── DATA STREAM ───────┤
  │<──── DATA STREAM ───────┤
  │                         │
  ├──── CONTROL MSG ───────>│
  │<──── RESPONSE ──────────┤
```

**Use Cases**:
- Real-time dashboards
- Interactive simulations
- Live monitoring
- Command and control

---

## Transport Layer

### 4.1 WebSocket Protocol

#### 4.1.1 Connection Establishment

```
Client -> Server:
GET wss://api.wia.live/digital-twin-city/v1/stream HTTP/1.1
Host: api.wia.live
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Key: x3JJHMbDL1EzLkh9GBhXDw==
Sec-WebSocket-Version: 13
Sec-WebSocket-Protocol: wia-dtc-v1
Authorization: Bearer <token>

Server -> Client:
HTTP/1.1 101 Switching Protocols
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Accept: HSmrc0sMlYUkAGmm5OPpG2HaGWk=
Sec-WebSocket-Protocol: wia-dtc-v1
```

#### 4.1.2 WebSocket Message Frame

```
Binary Frame Format (for efficiency):

 0                   1                   2                   3
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|F|R|R|R| opcode|M| Payload len |    Extended payload length    |
|I|S|S|S|  (4)  |A|     (7)     |             (16/64)           |
|N|V|V|V|       |S|             |   (if payload len==126/127)   |
| |1|2|3|       |K|             |                               |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|     Extended payload length continued, if payload len == 127  |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                               Payload Data                    |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
```

Text Frame Format (JSON):

```json
{
  "type": "PUBLISH",
  "topic": "sensor/temperature/gangnam-001",
  "payload": {
    "value": 23.5,
    "unit": "celsius",
    "timestamp": "2025-01-15T10:30:00+09:00"
  },
  "qos": 1,
  "messageId": "msg-123456"
}
```

#### 4.1.3 WebSocket Configuration

| Parameter | Value | Description |
|-----------|-------|-------------|
| Max Frame Size | 64 KB | Maximum single frame size |
| Max Message Size | 1 MB | Maximum complete message size |
| Ping Interval | 30s | Heartbeat frequency |
| Pong Timeout | 10s | Heartbeat response timeout |
| Compression | permessage-deflate | Per-message compression |

### 4.2 MQTT Protocol

#### 4.2.1 MQTT Connection

```
MQTT CONNECT Packet:

┌──────────────────────────────────┐
│ Fixed Header                     │
├──────────────────────────────────┤
│ Protocol Name: "MQTT"            │
│ Protocol Level: 5 (MQTT 5.0)     │
│ Connect Flags:                   │
│   - Clean Start: 1               │
│   - Will Flag: 0                 │
│   - QoS: 1                       │
│   - Username Flag: 1             │
│   - Password Flag: 1             │
│ Keep Alive: 60 seconds           │
├──────────────────────────────────┤
│ Properties:                      │
│   - Session Expiry: 3600         │
│   - Receive Maximum: 100         │
│   - Maximum Packet Size: 262144  │
│   - Topic Alias Maximum: 10      │
├──────────────────────────────────┤
│ Client ID: "dtc-client-001"      │
│ Username: API key or token       │
│ Password: API secret             │
└──────────────────────────────────┘
```

#### 4.2.2 Topic Hierarchy

```
city/{cityId}/sensors/{sensorType}/{sensorId}/{parameter}
city/{cityId}/objects/{objectType}/{objectId}/updates
city/{cityId}/simulations/{simId}/status
city/{cityId}/alerts/{severity}/{category}
city/{cityId}/services/{serviceId}/status
```

**Examples**:
```
city/KR-SEOUL-2025/sensors/environmental/env-001/temperature
city/KR-SEOUL-2025/sensors/traffic/trf-015/volume
city/KR-SEOUL-2025/objects/Building/building-001/updates
city/KR-SEOUL-2025/alerts/high/air-quality
city/KR-SEOUL-2025/services/subway/status
```

#### 4.2.3 MQTT QoS Levels

| QoS | Name | Guarantee | Use Case |
|-----|------|-----------|----------|
| 0 | At most once | Best effort, no ack | Non-critical sensor data |
| 1 | At least once | Acknowledged, may duplicate | Most sensor data |
| 2 | Exactly once | Guaranteed delivery, no duplicates | Critical alerts, commands |

#### 4.2.4 MQTT PUBLISH Packet

```json
{
  "topic": "city/KR-SEOUL-2025/sensors/environmental/env-001/temperature",
  "qos": 1,
  "retain": false,
  "properties": {
    "messageExpiryInterval": 300,
    "contentType": "application/json",
    "responseTopic": "city/KR-SEOUL-2025/responses/env-001",
    "correlationData": "req-12345"
  },
  "payload": {
    "sensorId": "sensor-env-gangnam-001",
    "parameter": "temperature",
    "value": 23.5,
    "unit": "celsius",
    "timestamp": "2025-01-15T10:30:00+09:00",
    "quality": "excellent"
  }
}
```

### 4.3 HTTP/2 Protocol

#### 4.3.1 Server Push

```
Client -> Server:
GET /cities/KR-SEOUL-2025/objects/building-001 HTTP/2
Host: api.wia.live
X-API-Key: your-api-key

Server -> Client (Push Promise):
PUSH_PROMISE
:method: GET
:path: /cities/KR-SEOUL-2025/sensors/sensor-env-gangnam-001
:authority: api.wia.live

Server -> Client (Original Response):
HTTP/2 200
content-type: application/json
{ "id": "building-001", ... }

Server -> Client (Pushed Response):
HTTP/2 200
content-type: application/json
{ "id": "sensor-env-gangnam-001", ... }
```

#### 4.3.2 HTTP/2 Stream Multiplexing

```
Connection
├── Stream 1: GET /cities/KR-SEOUL-2025/objects
├── Stream 3: GET /cities/KR-SEOUL-2025/sensors
├── Stream 5: POST /cities/KR-SEOUL-2025/simulations
└── Stream 7: GET /cities/KR-SEOUL-2025/services
```

---

## Message Format

### 5.1 Message Envelope

```typescript
interface WIAMessage {
  // Message metadata
  version: string;              // Protocol version (e.g., "1.0.0")
  type: MessageType;            // Message type
  messageId: string;            // Unique message identifier
  timestamp: string;            // ISO 8601 timestamp

  // Routing
  topic?: string;               // Topic for pub/sub
  source?: string;              // Message source identifier
  destination?: string;         // Message destination identifier

  // Quality of Service
  qos?: 0 | 1 | 2;             // QoS level
  ttl?: number;                 // Time to live (seconds)

  // Payload
  payload: any;                 // Message content

  // Optional metadata
  correlationId?: string;       // For request-response correlation
  contentType?: string;         // Payload content type
  encoding?: 'json' | 'binary' | 'compressed';
}
```

### 5.2 Sensor Data Message

```json
{
  "version": "1.0.0",
  "type": "PUBLISH",
  "messageId": "msg-env-001-1642234200",
  "timestamp": "2025-01-15T10:30:00+09:00",
  "topic": "city/KR-SEOUL-2025/sensors/environmental/env-001/temperature",
  "source": "sensor-env-gangnam-001",
  "qos": 1,
  "ttl": 300,
  "payload": {
    "sensorId": "sensor-env-gangnam-001",
    "measurements": [
      {
        "parameter": "temperature",
        "value": 23.5,
        "unit": "celsius",
        "timestamp": "2025-01-15T10:30:00+09:00",
        "quality": "excellent"
      }
    ]
  },
  "contentType": "application/json",
  "encoding": "json"
}
```

### 5.3 City Event Message

```json
{
  "version": "1.0.0",
  "type": "PUBLISH",
  "messageId": "evt-alert-20250115-001",
  "timestamp": "2025-01-15T10:30:00+09:00",
  "topic": "city/KR-SEOUL-2025/alerts/high/air-quality",
  "source": "monitoring-system",
  "qos": 2,
  "payload": {
    "eventType": "air-quality-alert",
    "severity": "high",
    "location": {
      "type": "Point",
      "coordinates": [127.0276, 37.4979]
    },
    "affectedArea": {
      "type": "Polygon",
      "coordinates": [[[127.0, 37.4], [127.1, 37.4], [127.1, 37.5], [127.0, 37.5], [127.0, 37.4]]]
    },
    "details": {
      "aqi": 155,
      "primaryPollutant": "PM2.5",
      "recommendation": "Sensitive groups should avoid outdoor activities"
    }
  },
  "contentType": "application/json",
  "encoding": "json"
}
```

### 5.4 Control Message

```json
{
  "version": "1.0.0",
  "type": "SUBSCRIBE",
  "messageId": "sub-client-001-001",
  "timestamp": "2025-01-15T10:30:00+09:00",
  "source": "dtc-client-001",
  "destination": "dtc-broker",
  "payload": {
    "topics": [
      {
        "topic": "city/KR-SEOUL-2025/sensors/+/+/temperature",
        "qos": 1
      },
      {
        "topic": "city/KR-SEOUL-2025/alerts/#",
        "qos": 2
      }
    ],
    "subscriptionId": "sub-001"
  },
  "contentType": "application/json"
}
```

### 5.5 Binary Message Format

For high-frequency sensor data, binary encoding is more efficient:

```
Binary Message Format:

┌──────────────────────────────────────────────────────────────┐
│ Header (32 bytes)                                            │
├──────────────────────────────────────────────────────────────┤
│ Version (4 bytes)          │ Type (4 bytes)                  │
├────────────────────────────┼─────────────────────────────────┤
│ Message ID (16 bytes)                                        │
├──────────────────────────────────────────────────────────────┤
│ Timestamp (8 bytes, Unix milliseconds)                       │
├──────────────────────────────────────────────────────────────┤
│ Payload (variable)                                           │
├──────────────────────────────────────────────────────────────┤
│ Sensor ID (16 bytes UUID)                                    │
├────────────────────────────┬─────────────────────────────────┤
│ Parameter ID (2 bytes)     │ Value (8 bytes, double)         │
├────────────────────────────┼─────────────────────────────────┤
│ Quality (1 byte)           │ Reserved (7 bytes)              │
└──────────────────────────────────────────────────────────────┘

Total: 64 bytes (vs ~200 bytes JSON)
```

**Parameter IDs**:
- 0x0001: Temperature
- 0x0002: Humidity
- 0x0003: Air Quality
- 0x0004: Traffic Volume
- 0x0005: Energy Consumption

---

## Connection Lifecycle

### 6.1 Connection States

```
┌──────────────┐
│ DISCONNECTED │
└──────┬───────┘
       │ connect()
       ▼
┌──────────────┐
│  CONNECTING  │
└──────┬───────┘
       │ CONNACK
       ▼
┌──────────────┐     heartbeat    ┌──────────────┐
│  CONNECTED   │ ◄───────────────►│  HEARTBEAT   │
└──────┬───────┘                  └──────────────┘
       │
       │ error / timeout
       ▼
┌──────────────┐
│ RECONNECTING │
└──────┬───────┘
       │ retry
       │ ◄─────┐
       │       │
       │ max retries
       ▼       │
┌──────────────┐
│ DISCONNECTED │
└──────────────┘
```

### 6.2 Connection Handshake

```
Client                                Server
  │                                     │
  ├───── CONNECT ──────────────────────>│
  │     (clientId, auth, keepAlive)     │
  │                                     │
  │                                     ├─── Validate credentials
  │                                     ├─── Allocate resources
  │                                     ├─── Generate session
  │                                     │
  │<───── CONNACK ──────────────────────┤
  │     (sessionPresent, returnCode)    │
  │                                     │
  ├───── SUBSCRIBE ────────────────────>│
  │     (topics, qos)                   │
  │                                     │
  │<───── SUBACK ───────────────────────┤
  │     (grantedQos)                    │
  │                                     │
  │<───── PUBLISH ──────────────────────┤
  │     (topic, payload, qos)           │
  │                                     │
  ├───── PUBACK ───────────────────────>│
  │     (messageId)                     │
  │                                     │
```

### 6.3 Heartbeat Mechanism

```
Client                                Server
  │                                     │
  │                 30s                 │
  ├───── PING ─────────────────────────>│
  │                                     │
  │<───── PONG ─────────────────────────┤
  │                                     │
  │                 30s                 │
  ├───── PING ─────────────────────────>│
  │                                     │
  │        (no response)                │
  │                                     │
  │         10s timeout                 │
  │                                     │
  ├───── RECONNECT ─────────────────────┤
```

### 6.4 Graceful Disconnect

```
Client                                Server
  │                                     │
  ├───── DISCONNECT ───────────────────>│
  │     (reasonCode, properties)        │
  │                                     │
  │                                     ├─── Clean up session
  │                                     ├─── Release resources
  │                                     │
  │<───── DISCONNECT ACK ───────────────┤
  │                                     │
  └─── Close TCP Connection ───────────┘
```

### 6.5 Reconnection Strategy

```typescript
interface ReconnectionConfig {
  maxRetries: number;           // Maximum reconnection attempts
  initialDelay: number;         // Initial delay (ms)
  maxDelay: number;            // Maximum delay (ms)
  backoffMultiplier: number;   // Exponential backoff multiplier
  jitter: boolean;             // Add random jitter
}

const defaultConfig: ReconnectionConfig = {
  maxRetries: 10,
  initialDelay: 1000,          // 1 second
  maxDelay: 60000,             // 60 seconds
  backoffMultiplier: 2,
  jitter: true
};

// Reconnection delay calculation
function calculateDelay(attempt: number, config: ReconnectionConfig): number {
  let delay = config.initialDelay * Math.pow(config.backoffMultiplier, attempt);
  delay = Math.min(delay, config.maxDelay);

  if (config.jitter) {
    delay = delay * (0.5 + Math.random() * 0.5);
  }

  return delay;
}
```

**Reconnection Timeline**:
```
Attempt 1: 1s delay
Attempt 2: 2s delay
Attempt 3: 4s delay
Attempt 4: 8s delay
Attempt 5: 16s delay
Attempt 6: 32s delay
Attempt 7: 60s delay (capped)
Attempt 8: 60s delay
Attempt 9: 60s delay
Attempt 10: 60s delay
Max attempts reached → Give up
```

---

## Real-time Streaming

### 7.1 Sensor Data Streaming

```typescript
// Client subscribes to sensor data stream
const client = new DigitalTwinCityClient({
  endpoint: 'wss://api.wia.live/digital-twin-city/v1/stream',
  apiKey: 'your-api-key'
});

await client.connect();

// Subscribe to specific sensors
await client.subscribe([
  'city/KR-SEOUL-2025/sensors/environmental/+/temperature',
  'city/KR-SEOUL-2025/sensors/traffic/+/volume'
], {
  qos: 1,
  onMessage: (message) => {
    console.log(`Received: ${message.topic}`, message.payload);
    updateDashboard(message.payload);
  }
});
```

### 7.2 Event Streaming

```typescript
// Subscribe to city events
await client.subscribe([
  'city/KR-SEOUL-2025/alerts/high/#',
  'city/KR-SEOUL-2025/services/+/status'
], {
  qos: 2,
  onMessage: (message) => {
    if (message.topic.includes('alerts/high')) {
      triggerAlert(message.payload);
    } else if (message.topic.includes('status')) {
      updateServiceStatus(message.payload);
    }
  }
});
```

### 7.3 Backpressure Handling

```
Client                                Server
  │                                     │
  │<───── PUBLISH (1) ──────────────────┤
  ├───── PUBACK (1) ───────────────────>│
  │                                     │
  │<───── PUBLISH (2) ──────────────────┤
  │<───── PUBLISH (3) ──────────────────┤
  │<───── PUBLISH (4) ──────────────────┤
  │                                     │
  ├───── FLOW CONTROL (pause) ─────────>│
  │                                     │
  │     Processing...                   │
  │                                     │
  ├───── PUBACK (2) ───────────────────>│
  ├───── PUBACK (3) ───────────────────>│
  ├───── PUBACK (4) ───────────────────>│
  │                                     │
  ├───── FLOW CONTROL (resume) ────────>│
  │                                     │
  │<───── PUBLISH (5) ──────────────────┤
```

```typescript
interface FlowControlConfig {
  receiveMaximum: number;      // Max in-flight messages
  highWaterMark: number;       // Trigger backpressure
  lowWaterMark: number;        // Resume streaming
}

const flowControl: FlowControlConfig = {
  receiveMaximum: 100,
  highWaterMark: 80,
  lowWaterMark: 20
};
```

### 7.4 Stream Aggregation

```typescript
// Aggregate multiple sensor streams
const aggregator = client.createAggregator({
  sources: [
    'city/KR-SEOUL-2025/sensors/environmental/env-001/temperature',
    'city/KR-SEOUL-2025/sensors/environmental/env-002/temperature',
    'city/KR-SEOUL-2025/sensors/environmental/env-003/temperature'
  ],
  window: {
    type: 'tumbling',
    duration: 60000  // 1 minute
  },
  aggregation: 'average',
  onResult: (result) => {
    console.log('Average temperature:', result.value);
  }
});
```

---

## Security

### 8.1 Transport Layer Security (TLS 1.3)

```
TLS 1.3 Handshake:

Client                                Server
  │                                     │
  ├───── ClientHello ──────────────────>│
  │     (supported ciphers, extensions) │
  │                                     │
  │<───── ServerHello ──────────────────┤
  │     (selected cipher, cert)         │
  │<───── EncryptedExtensions ──────────┤
  │<───── Certificate ──────────────────┤
  │<───── CertificateVerify ────────────┤
  │<───── Finished ─────────────────────┤
  │                                     │
  ├───── Certificate (optional) ───────>│
  ├───── CertificateVerify ────────────>│
  ├───── Finished ────────────────────>│
  │                                     │
  │<═════ Encrypted Application Data ══>│
```

**Supported Cipher Suites**:
- TLS_AES_128_GCM_SHA256
- TLS_AES_256_GCM_SHA384
- TLS_CHACHA20_POLY1305_SHA256

### 8.2 Authentication

#### 8.2.1 API Key Authentication

```
WebSocket Connection:
GET wss://api.wia.live/digital-twin-city/v1/stream HTTP/1.1
X-API-Key: sk_live_abc123def456

MQTT Connection:
CONNECT
  Username: api-key
  Password: sk_live_abc123def456
```

#### 8.2.2 OAuth 2.0 Token

```
WebSocket Connection:
GET wss://api.wia.live/digital-twin-city/v1/stream HTTP/1.1
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...

MQTT Connection:
CONNECT
  Username: oauth
  Password: eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

#### 8.2.3 Mutual TLS (mTLS)

```
TLS Handshake with Client Certificate:

Client                                Server
  │                                     │
  ├───── ClientHello ──────────────────>│
  │                                     │
  │<───── ServerHello ──────────────────┤
  │<───── Certificate ──────────────────┤
  │<───── CertificateRequest ───────────┤
  │                                     │
  ├───── Certificate ──────────────────>│
  │     (client certificate)            │
  ├───── CertificateVerify ────────────>│
  │                                     │
  │                                     ├─── Validate client cert
  │                                     ├─── Check CN, SAN
  │                                     │
  │<───── Finished ─────────────────────┤
```

### 8.3 Authorization

```typescript
interface AccessControl {
  // Topic-based access control
  allowedTopics: {
    subscribe: string[];   // Topics client can subscribe to
    publish: string[];     // Topics client can publish to
  };

  // Operation-based access control
  permissions: {
    read: boolean;
    write: boolean;
    admin: boolean;
  };

  // Rate limiting
  rateLimit: {
    messagesPerSecond: number;
    bytesPerSecond: number;
  };
}

// Example ACL for sensor device
const sensorACL: AccessControl = {
  allowedTopics: {
    subscribe: [],
    publish: [
      'city/KR-SEOUL-2025/sensors/environmental/env-001/#'
    ]
  },
  permissions: {
    read: false,
    write: true,
    admin: false
  },
  rateLimit: {
    messagesPerSecond: 10,
    bytesPerSecond: 10240
  }
};

// Example ACL for monitoring dashboard
const dashboardACL: AccessControl = {
  allowedTopics: {
    subscribe: [
      'city/KR-SEOUL-2025/sensors/#',
      'city/KR-SEOUL-2025/alerts/#'
    ],
    publish: []
  },
  permissions: {
    read: true,
    write: false,
    admin: false
  },
  rateLimit: {
    messagesPerSecond: 1000,
    bytesPerSecond: 1048576
  }
};
```

### 8.4 Message Integrity

```typescript
// Message signature for integrity verification
interface SignedMessage extends WIAMessage {
  signature: {
    algorithm: 'HMAC-SHA256' | 'Ed25519';
    value: string;
    keyId: string;
  };
}

// Example signed message
const signedMessage: SignedMessage = {
  version: "1.0.0",
  type: "PUBLISH",
  messageId: "msg-001",
  timestamp: "2025-01-15T10:30:00+09:00",
  topic: "city/KR-SEOUL-2025/sensors/environmental/env-001/temperature",
  payload: { value: 23.5, unit: "celsius" },
  signature: {
    algorithm: "HMAC-SHA256",
    value: "a1b2c3d4e5f6...",
    keyId: "key-001"
  }
};
```

---

## Quality of Service

### 9.1 QoS Levels

#### QoS 0: At Most Once

```
Sensor                               Broker
  │                                     │
  ├───── PUBLISH (QoS 0) ──────────────>│
  │                                     │
  │     (no acknowledgment)             │
```

**Characteristics**:
- Fire and forget
- No retransmission
- Fastest, lowest overhead
- Message may be lost

**Use Cases**:
- High-frequency sensor data where occasional loss is acceptable
- Metrics and telemetry
- Non-critical monitoring

#### QoS 1: At Least Once

```
Sensor                               Broker
  │                                     │
  ├───── PUBLISH (QoS 1, msgId=1) ────>│
  │                                     │
  │<───── PUBACK (msgId=1) ─────────────┤
  │                                     │
```

**Characteristics**:
- Acknowledged delivery
- Retransmission on timeout
- May receive duplicates

**Use Cases**:
- Most sensor data
- Event notifications
- State updates

#### QoS 2: Exactly Once

```
Sensor                               Broker
  │                                     │
  ├───── PUBLISH (QoS 2, msgId=1) ────>│
  │                                     │
  │<───── PUBREC (msgId=1) ─────────────┤
  │                                     │
  ├───── PUBREL (msgId=1) ─────────────>│
  │                                     │
  │<───── PUBCOMP (msgId=1) ────────────┤
  │                                     │
```

**Characteristics**:
- Guaranteed delivery
- No duplicates
- 4-way handshake
- Highest overhead

**Use Cases**:
- Critical alerts
- Commands and control
- Financial transactions
- Safety-critical operations

### 9.2 Message Retention

```typescript
interface RetentionPolicy {
  enabled: boolean;
  duration: number;        // Seconds
  maxMessages: number;     // Per topic
  storageLimit: number;    // Bytes
}

const retentionPolicies = {
  'city/+/sensors/#': {
    enabled: true,
    duration: 86400,       // 24 hours
    maxMessages: 10000,
    storageLimit: 10485760  // 10 MB
  },
  'city/+/alerts/#': {
    enabled: true,
    duration: 604800,      // 7 days
    maxMessages: 100000,
    storageLimit: 104857600 // 100 MB
  }
};
```

### 9.3 Delivery Guarantees

| Scenario | QoS 0 | QoS 1 | QoS 2 |
|----------|-------|-------|-------|
| Network failure during transmission | Message lost | Retransmitted | Retransmitted |
| Duplicate messages | Possible | Possible | Not possible |
| Message ordering | Not guaranteed | Not guaranteed | Guaranteed |
| Latency | Lowest | Medium | Highest |
| Bandwidth usage | Lowest | Medium | Highest |

---

## Protocol Examples

### 10.1 Complete Sensor Integration

```typescript
import { MQTTClient } from 'wia-mqtt-client';

// Sensor device code
const sensor = new MQTTClient({
  broker: 'mqtts://mqtt.wia.live:8883',
  clientId: 'sensor-env-gangnam-001',
  username: 'api-key',
  password: process.env.WIA_API_KEY,
  keepAlive: 60,
  reconnect: true
});

sensor.on('connect', () => {
  console.log('Sensor connected to broker');

  // Start publishing sensor data
  setInterval(() => {
    const data = {
      sensorId: 'sensor-env-gangnam-001',
      measurements: [
        {
          parameter: 'temperature',
          value: readTemperature(),
          unit: 'celsius',
          timestamp: new Date().toISOString(),
          quality: 'excellent'
        }
      ]
    };

    sensor.publish(
      'city/KR-SEOUL-2025/sensors/environmental/env-001/temperature',
      JSON.stringify(data),
      { qos: 1 }
    );
  }, 5000); // Every 5 seconds
});

sensor.on('error', (error) => {
  console.error('MQTT error:', error);
});

sensor.connect();
```

### 10.2 Real-time Dashboard

```typescript
import { WebSocketClient } from 'wia-ws-client';

// Dashboard client code
const dashboard = new WebSocketClient({
  url: 'wss://api.wia.live/digital-twin-city/v1/stream',
  apiKey: process.env.WIA_API_KEY
});

await dashboard.connect();

// Subscribe to all environmental sensors
await dashboard.subscribe({
  topics: [
    'city/KR-SEOUL-2025/sensors/environmental/+/temperature',
    'city/KR-SEOUL-2025/sensors/environmental/+/humidity',
    'city/KR-SEOUL-2025/sensors/environmental/+/air_quality'
  ],
  qos: 1,
  onMessage: (message) => {
    const { topic, payload } = message;
    const [, cityId, , sensorType, sensorId, parameter] = topic.split('/');

    updateChart(sensorId, parameter, payload.measurements[0].value);

    // Trigger alert if threshold exceeded
    if (parameter === 'air_quality' && payload.measurements[0].value > 100) {
      showAlert(`High AQI: ${payload.measurements[0].value} at ${sensorId}`);
    }
  }
});

// Subscribe to alerts
await dashboard.subscribe({
  topics: ['city/KR-SEOUL-2025/alerts/#'],
  qos: 2,
  onMessage: (message) => {
    const alert = message.payload;
    displayAlert({
      severity: alert.severity,
      message: alert.details.recommendation,
      location: alert.location
    });
  }
});
```

### 10.3 IoT Gateway

```typescript
import { MQTTClient } from 'wia-mqtt-client';

// Gateway aggregating multiple sensors
const gateway = new MQTTClient({
  broker: 'mqtts://mqtt.wia.live:8883',
  clientId: 'gateway-gangnam-district',
  username: 'api-key',
  password: process.env.WIA_API_KEY
});

await gateway.connect();

// Collect data from local sensors (via serial, I2C, etc.)
const localSensors = [
  { id: 'env-001', port: '/dev/ttyUSB0' },
  { id: 'env-002', port: '/dev/ttyUSB1' },
  { id: 'env-003', port: '/dev/ttyUSB2' }
];

for (const sensor of localSensors) {
  // Read from local sensor
  const data = await readSensorData(sensor.port);

  // Publish to MQTT broker
  await gateway.publish(
    `city/KR-SEOUL-2025/sensors/environmental/${sensor.id}/temperature`,
    JSON.stringify({
      sensorId: sensor.id,
      measurements: [data],
      gatewayId: 'gateway-gangnam-district'
    }),
    { qos: 1, retain: false }
  );
}
```

### 10.4 Alert Processing System

```python
import asyncio
from wia_mqtt_client import MQTTClient

async def process_alerts():
    client = MQTTClient(
        broker='mqtts://mqtt.wia.live:8883',
        client_id='alert-processor',
        username='api-key',
        password=os.getenv('WIA_API_KEY')
    )

    await client.connect()

    # Subscribe to all alerts
    await client.subscribe([
        ('city/KR-SEOUL-2025/alerts/high/#', 2),
        ('city/KR-SEOUL-2025/alerts/medium/#', 1)
    ])

    async for message in client.messages():
        alert = message.payload

        # Process based on severity
        if 'high' in message.topic:
            # Immediate action
            await send_emergency_notification(alert)
            await activate_emergency_protocol(alert)
        elif 'medium' in message.topic:
            # Queue for processing
            await queue_alert(alert)

        # Acknowledge
        await client.ack(message.message_id)

asyncio.run(process_alerts())
```

---

## References

### Related Standards

- [WIA Digital Twin City Data Format (Phase 1)](/digital-twin-city/spec/PHASE-1-DATA-FORMAT.md)
- [WIA Digital Twin City API Interface (Phase 2)](/digital-twin-city/spec/PHASE-2-API-INTERFACE.md)
- [WIA Digital Twin City Integration (Phase 4)](/digital-twin-city/spec/PHASE-4-INTEGRATION.md)

### Protocol Standards

- [WebSocket Protocol RFC 6455](https://tools.ietf.org/html/rfc6455)
- [MQTT 5.0 Specification](https://docs.oasis-open.org/mqtt/mqtt/v5.0/mqtt-v5.0.html)
- [HTTP/2 RFC 7540](https://tools.ietf.org/html/rfc7540)
- [TLS 1.3 RFC 8446](https://tools.ietf.org/html/rfc8446)

### Security Standards

- [OAuth 2.0 RFC 6749](https://tools.ietf.org/html/rfc6749)
- [JSON Web Token (JWT) RFC 7519](https://tools.ietf.org/html/rfc7519)
- [HMAC RFC 2104](https://tools.ietf.org/html/rfc2104)

---

<div align="center">

**WIA Digital Twin City Standard v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA**

**MIT License**

</div>
