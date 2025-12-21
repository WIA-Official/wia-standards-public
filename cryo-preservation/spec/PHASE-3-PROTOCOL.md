# WIA Cryo-Preservation Communication Protocol
## Phase 3 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #06B6D4 (Cyan)

---

## Table of Contents

1. [Overview](#overview)
2. [Protocol Architecture](#protocol-architecture)
3. [Transport Layer](#transport-layer)
4. [Message Format](#message-format)
5. [Connection Lifecycle](#connection-lifecycle)
6. [Message Types](#message-types)
7. [Security](#security)
8. [Error Handling](#error-handling)
9. [Examples](#examples)

---

## Overview

### 1.1 Purpose

The WIA Cryo-Preservation Communication Protocol defines real-time communication standards for monitoring storage conditions, receiving alerts, and coordinating facility operations.

**Core Objectives**:
- Real-time storage condition monitoring
- Instant alert notification delivery
- Secure facility-to-facility communication
- Reliable message delivery with acknowledgment

### 1.2 Protocol Selection

| Use Case | Recommended Protocol |
|----------|---------------------|
| Real-time monitoring | WebSocket |
| IoT sensor data | MQTT |
| High-throughput data | gRPC |
| Event notifications | Server-Sent Events (SSE) |

### 1.3 Design Principles

1. **Reliability**: At-least-once delivery guarantee
2. **Low Latency**: Sub-second message delivery
3. **Security**: End-to-end encryption
4. **Resilience**: Automatic reconnection
5. **Scalability**: Horizontal scaling support

---

## Protocol Architecture

### 2.1 Layer Model

```
┌──────────────────────────────────────────────────────────────┐
│                    Application Layer                          │
│            (WIA Cryo-Preservation Services)                   │
├──────────────────────────────────────────────────────────────┤
│                    Protocol Layer                             │
│              (Message Format, Handlers)                       │
├──────────────────────────────────────────────────────────────┤
│                    Security Layer                             │
│                  (TLS 1.3, JWT Auth)                         │
├──────────────────────────────────────────────────────────────┤
│                    Transport Layer                            │
│           (WebSocket / MQTT / gRPC / SSE)                    │
├──────────────────────────────────────────────────────────────┤
│                    Network Layer                              │
│                     (TCP/IP)                                  │
└──────────────────────────────────────────────────────────────┘
```

### 2.2 Component Architecture

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│  Storage Sensor │     │  Control Panel  │     │  Mobile App     │
│     Devices     │     │    Dashboard    │     │   (Alerts)      │
└────────┬────────┘     └────────┬────────┘     └────────┬────────┘
         │                       │                       │
         │ MQTT                  │ WebSocket             │ WebSocket
         │                       │                       │
         ▼                       ▼                       ▼
┌──────────────────────────────────────────────────────────────────┐
│                    Message Broker / Gateway                       │
│               (Authentication, Routing, Buffering)               │
└──────────────────────────────────────────────────────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│  Storage Data   │     │  Alert Service  │     │  Audit Log      │
│    Service      │     │                 │     │   Service       │
└─────────────────┘     └─────────────────┘     └─────────────────┘
```

---

## Transport Layer

### 3.1 WebSocket (Primary)

**Connection URL:**
```
wss://ws.wia.live/cryo-preservation/v1
```

**Default Port:** 443 (WSS)

**Subprotocol:** `wia-cryo-v1`

**Connection Example:**
```javascript
const ws = new WebSocket('wss://ws.wia.live/cryo-preservation/v1', 'wia-cryo-v1');
```

### 3.2 MQTT (IoT Sensors)

**Broker URL:**
```
mqtts://mqtt.wia.live:8883
```

**Topic Structure:**
```
wia/cryo/{facility_id}/{container_id}/{metric_type}

Examples:
wia/cryo/FAC-KR-001/DEW-001/temperature
wia/cryo/FAC-KR-001/DEW-001/nitrogen_level
wia/cryo/FAC-KR-001/+/alerts
```

**QoS Levels:**
| QoS | Use Case |
|-----|----------|
| 0 | Routine sensor readings |
| 1 | Status updates |
| 2 | Critical alerts |

### 3.3 gRPC (Service-to-Service)

**Proto Definition:**
```protobuf
syntax = "proto3";

package wia.cryo.v1;

service CryoMonitoring {
  rpc StreamConditions(ContainerRequest) returns (stream StorageCondition);
  rpc SendAlert(Alert) returns (AlertAck);
  rpc GetStatus(StatusRequest) returns (StatusResponse);
}

message StorageCondition {
  string container_id = 1;
  int64 timestamp = 2;
  double temperature = 3;
  double nitrogen_level = 4;
  double vacuum_pressure = 5;
}
```

### 3.4 Server-Sent Events (Alerts)

**Endpoint:**
```
GET /api/v1/events/stream
Accept: text/event-stream
```

**Event Format:**
```
event: storage.alert
data: {"containerId":"DEW-001","type":"temperature","value":-195.0}

event: heartbeat
data: {"timestamp":1704067200}
```

---

## Message Format

### 4.1 Base Message Structure

```json
{
  "protocol": "wia-cryo",
  "version": "1.0.0",
  "messageId": "msg-uuid-v4",
  "timestamp": 1704067200000,
  "type": "message_type",
  "source": {
    "type": "sensor|service|client",
    "id": "source-identifier"
  },
  "payload": {}
}
```

### 4.2 Field Definitions

| Field | Type | Required | Description |
|-------|------|:--------:|-------------|
| `protocol` | string | Yes | Protocol identifier `"wia-cryo"` |
| `version` | string | Yes | Protocol version |
| `messageId` | string | Yes | Unique message ID (UUID v4) |
| `timestamp` | integer | Yes | Unix timestamp (ms) |
| `type` | string | Yes | Message type |
| `source` | object | Yes | Message source info |
| `payload` | object | Yes | Type-specific data |

### 4.3 JSON Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.live/cryo-preservation/protocol/v1/message.schema.json",
  "title": "WIA Cryo Protocol Message",
  "type": "object",
  "required": ["protocol", "version", "messageId", "timestamp", "type", "payload"],
  "properties": {
    "protocol": {
      "type": "string",
      "const": "wia-cryo"
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
        "connect", "connect_ack", "disconnect",
        "subscribe", "subscribe_ack", "unsubscribe",
        "sensor_data", "alert", "alert_ack",
        "command", "command_ack",
        "ping", "pong", "error"
      ]
    },
    "source": {
      "type": "object",
      "required": ["type", "id"],
      "properties": {
        "type": { "type": "string" },
        "id": { "type": "string" }
      }
    },
    "payload": {
      "type": "object"
    }
  }
}
```

---

## Connection Lifecycle

### 5.1 Connection States

```
                          ┌─────────────────┐
                          │  DISCONNECTED   │
                          └────────┬────────┘
                                   │ connect()
                                   ▼
                          ┌─────────────────┐
                          │   CONNECTING    │
                          └────────┬────────┘
                                   │ connect_ack
                    ┌──────────────┼──────────────┐
                    │              ▼              │
         error/     │     ┌─────────────────┐     │ auth failed
         timeout    │     │   CONNECTED     │     │
                    │     └────────┬────────┘     │
                    │              │              │
                    │         disconnect/         │
                    │         error               │
                    │              ▼              │
                    │     ┌─────────────────┐     │
                    │     │  RECONNECTING   │     │
                    │     └────────┬────────┘     │
                    │              │              │
                    └──────────────┴──────────────┘
                                   │
                                   ▼
                          ┌─────────────────┐
                          │  DISCONNECTED   │
                          └─────────────────┘
```

### 5.2 Connection Sequence

```
Client                                           Server
   │                                                │
   │ ─────────── TLS Handshake ──────────────────► │
   │                                                │
   │ ◄─────────── TLS Established ─────────────── │
   │                                                │
   │ ─────────── WebSocket Upgrade ──────────────► │
   │                                                │
   │ ◄─────────── 101 Switching ──────────────── │
   │                                                │
   │ ─────────────── connect ────────────────────► │
   │     { "type": "connect",                      │
   │       "payload": { "token": "jwt..." } }      │
   │                                                │
   │ ◄────────────── connect_ack ────────────────  │
   │     { "type": "connect_ack",                  │
   │       "payload": { "sessionId": "..." } }     │
   │                                                │
   │ ─────────────── subscribe ──────────────────► │
   │                                                │
   │ ◄────────────── subscribe_ack ──────────────  │
   │                                                │
   │ ◄────────────── sensor_data ────────────────  │
   │ ◄────────────── sensor_data ────────────────  │
   │                  ...                           │
```

### 5.3 Reconnection Policy

| Parameter | Default | Description |
|-----------|---------|-------------|
| `autoReconnect` | `true` | Enable auto-reconnect |
| `initialDelay` | `1000ms` | First retry delay |
| `maxDelay` | `30000ms` | Maximum retry delay |
| `maxAttempts` | `10` | Maximum retry attempts |
| `backoffMultiplier` | `2.0` | Exponential backoff |

**Reconnection Schedule:**
```
Attempt 1:  1,000ms
Attempt 2:  2,000ms
Attempt 3:  4,000ms
Attempt 4:  8,000ms
Attempt 5: 16,000ms
Attempt 6+: 30,000ms (capped)
```

### 5.4 Heartbeat

- **Interval:** 30 seconds
- **Timeout:** 10 seconds
- **Max Missed:** 3 pings before disconnect

```json
// Ping
{
  "type": "ping",
  "messageId": "ping-001",
  "timestamp": 1704067200000,
  "payload": { "sequence": 1 }
}

// Pong
{
  "type": "pong",
  "messageId": "pong-001",
  "timestamp": 1704067200005,
  "payload": { "sequence": 1, "latency_ms": 5 }
}
```

---

## Message Types

### 6.1 Message Type Summary

| Type | Direction | Description |
|------|-----------|-------------|
| `connect` | C → S | Connection request |
| `connect_ack` | S → C | Connection response |
| `disconnect` | Both | Disconnection notice |
| `subscribe` | C → S | Subscribe to topics |
| `subscribe_ack` | S → C | Subscription confirmed |
| `unsubscribe` | C → S | Unsubscribe from topics |
| `sensor_data` | S → C | Sensor readings |
| `alert` | S → C | Alert notification |
| `alert_ack` | C → S | Alert acknowledgment |
| `command` | C → S | Device command |
| `command_ack` | S → C | Command response |
| `ping` | C → S | Heartbeat ping |
| `pong` | S → C | Heartbeat pong |
| `error` | Both | Error message |

### 6.2 Connection Messages

#### connect

```json
{
  "protocol": "wia-cryo",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": 1704067200000,
  "type": "connect",
  "source": {
    "type": "client",
    "id": "dashboard-001"
  },
  "payload": {
    "token": "eyJhbGciOiJSUzI1NiIs...",
    "facilityId": "FAC-KR-001",
    "clientVersion": "2.1.0",
    "capabilities": ["sensor_data", "alerts", "commands"]
  }
}
```

#### connect_ack

```json
{
  "protocol": "wia-cryo",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440001",
  "timestamp": 1704067200050,
  "type": "connect_ack",
  "source": {
    "type": "service",
    "id": "gateway-001"
  },
  "payload": {
    "success": true,
    "sessionId": "session-abc123",
    "heartbeatInterval": 30000,
    "serverVersion": "1.0.0"
  }
}
```

### 6.3 Subscription Messages

#### subscribe

```json
{
  "type": "subscribe",
  "messageId": "550e8400-e29b-41d4-a716-446655440010",
  "timestamp": 1704067200100,
  "payload": {
    "topics": [
      "containers/DEW-KR-001/temperature",
      "containers/DEW-KR-001/nitrogen",
      "containers/+/alerts"
    ],
    "options": {
      "includeHistory": true,
      "historyDuration": 3600
    }
  }
}
```

### 6.4 Sensor Data Messages

#### sensor_data

```json
{
  "protocol": "wia-cryo",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440100",
  "timestamp": 1704067200200,
  "type": "sensor_data",
  "source": {
    "type": "sensor",
    "id": "SENSOR-DEW-KR-001-TEMP"
  },
  "payload": {
    "containerId": "DEW-KR-001",
    "metricType": "temperature",
    "value": -196.2,
    "unit": "celsius",
    "quality": {
      "accuracy": 0.1,
      "status": "normal"
    }
  }
}
```

### 6.5 Alert Messages

#### alert

```json
{
  "protocol": "wia-cryo",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440200",
  "timestamp": 1704067200300,
  "type": "alert",
  "source": {
    "type": "service",
    "id": "alert-service"
  },
  "payload": {
    "alertId": "ALERT-2025-001",
    "severity": "warning",
    "category": "temperature",
    "containerId": "DEW-KR-001",
    "message": "Temperature variance detected",
    "details": {
      "currentValue": -195.0,
      "expectedValue": -196.0,
      "threshold": 0.5
    },
    "actions": ["acknowledge", "dismiss", "escalate"]
  }
}
```

#### alert_ack

```json
{
  "type": "alert_ack",
  "messageId": "550e8400-e29b-41d4-a716-446655440201",
  "timestamp": 1704067200400,
  "payload": {
    "alertId": "ALERT-2025-001",
    "action": "acknowledge",
    "acknowledgedBy": "STAFF-001",
    "notes": "Checking LN2 levels"
  }
}
```

### 6.6 Command Messages

#### command

```json
{
  "type": "command",
  "messageId": "550e8400-e29b-41d4-a716-446655440300",
  "timestamp": 1704067200500,
  "payload": {
    "command": "refill_nitrogen",
    "target": "DEW-KR-001",
    "parameters": {
      "targetLevel": 0.95,
      "priority": "normal"
    }
  }
}
```

---

## Security

### 7.1 Transport Security

- **TLS Version:** 1.3 required
- **Cipher Suites:** TLS_AES_256_GCM_SHA384, TLS_CHACHA20_POLY1305_SHA256
- **Certificate:** Valid TLS certificate required

### 7.2 Authentication

All connections require JWT authentication:

```json
{
  "type": "connect",
  "payload": {
    "token": "eyJhbGciOiJSUzI1NiIs..."
  }
}
```

### 7.3 Authorization

Topic-based access control:

| Role | Allowed Topics |
|------|---------------|
| `viewer` | `containers/+/temperature`, `containers/+/nitrogen` |
| `operator` | All viewer + `commands/*` |
| `admin` | All topics |

### 7.4 Message Signing (Optional)

For critical messages:

```json
{
  "payload": { ... },
  "signature": {
    "algorithm": "ES256",
    "value": "MEUCIQDi...",
    "keyId": "key-001"
  }
}
```

---

## Error Handling

### 8.1 Error Codes

| Code | Name | Description |
|------|------|-------------|
| 1000 | `CONNECTION_CLOSED` | Normal closure |
| 1001 | `CONNECTION_LOST` | Unexpected disconnect |
| 1002 | `PROTOCOL_ERROR` | Protocol violation |
| 2001 | `AUTH_FAILED` | Authentication failed |
| 2002 | `AUTH_EXPIRED` | Token expired |
| 2003 | `PERMISSION_DENIED` | Insufficient permissions |
| 3001 | `INVALID_MESSAGE` | Malformed message |
| 3002 | `INVALID_TOPIC` | Unknown topic |
| 3003 | `RATE_LIMITED` | Too many messages |
| 4001 | `CONTAINER_NOT_FOUND` | Container doesn't exist |
| 4002 | `SENSOR_OFFLINE` | Sensor not responding |
| 5001 | `INTERNAL_ERROR` | Server error |

### 8.2 Error Message Format

```json
{
  "protocol": "wia-cryo",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440999",
  "timestamp": 1704067200600,
  "type": "error",
  "payload": {
    "code": 2001,
    "name": "AUTH_FAILED",
    "message": "Invalid or expired token",
    "recoverable": true,
    "retryAfter": 0,
    "relatedMessageId": "550e8400-e29b-41d4-a716-446655440000"
  }
}
```

---

## Examples

### 9.1 Complete Session Example (TypeScript)

```typescript
import { WiaCryoProtocol, ConnectionState } from '@wia/cryo-protocol';

const protocol = new WiaCryoProtocol({
  url: 'wss://ws.wia.live/cryo-preservation/v1',
  token: 'your-jwt-token',
  facilityId: 'FAC-KR-001',
  autoReconnect: true
});

// Connection events
protocol.on('connected', (session) => {
  console.log('Connected:', session.sessionId);
});

protocol.on('disconnected', (reason) => {
  console.log('Disconnected:', reason);
});

// Subscribe to sensor data
await protocol.subscribe([
  'containers/DEW-KR-001/temperature',
  'containers/DEW-KR-001/nitrogen',
  'containers/+/alerts'
]);

// Handle sensor data
protocol.on('sensor_data', (data) => {
  console.log(`${data.containerId} ${data.metricType}: ${data.value}${data.unit}`);
});

// Handle alerts
protocol.on('alert', async (alert) => {
  console.log(`Alert [${alert.severity}]: ${alert.message}`);

  // Acknowledge alert
  await protocol.acknowledgeAlert(alert.alertId, {
    action: 'acknowledge',
    notes: 'Investigating'
  });
});

// Send command
await protocol.sendCommand({
  command: 'refill_nitrogen',
  target: 'DEW-KR-001',
  parameters: { targetLevel: 0.95 }
});

// Cleanup
await protocol.disconnect();
```

### 9.2 Python Example

```python
import asyncio
from wia_cryo import WiaCryoProtocol

async def main():
    protocol = WiaCryoProtocol(
        url='wss://ws.wia.live/cryo-preservation/v1',
        token='your-jwt-token',
        facility_id='FAC-KR-001'
    )

    await protocol.connect()

    # Subscribe
    await protocol.subscribe([
        'containers/DEW-KR-001/temperature',
        'containers/+/alerts'
    ])

    # Handle messages
    @protocol.on('sensor_data')
    async def on_sensor_data(data):
        print(f"{data['containerId']}: {data['value']}")

    @protocol.on('alert')
    async def on_alert(alert):
        print(f"Alert: {alert['message']}")
        await protocol.acknowledge_alert(alert['alertId'])

    # Keep running
    await asyncio.sleep(3600)

    await protocol.disconnect()

asyncio.run(main())
```

---

<div align="center">

**WIA Cryo-Preservation Communication Protocol v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA Standards Committee**

**MIT License**

</div>
