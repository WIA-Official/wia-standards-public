# WIA Physics Standard - Phase 3: Communication Protocol

**Version**: 1.0.0
**Date**: 2025-12-14
**Status**: Specification Complete

---

## 1. Overview

This specification defines the WIA Physics Protocol (WPP), a transport-agnostic communication protocol for real-time physics data streaming, device control, and system interoperability.

### 1.1 Design Goals

| Goal | Description |
|------|-------------|
| **Real-time Streaming** | Support continuous data streams from physics instruments |
| **Request-Response** | Enable device control and query operations |
| **Event-Driven** | Push notifications for state changes and alerts |
| **Transport Agnostic** | Work over WebSocket, TCP, and gRPC |
| **Schema Compatible** | Integrate with Phase 1/2 JSON schemas and types |

### 1.2 Protocol Layers

```
┌─────────────────────────────────────────────────────────────┐
│                    Application Layer                         │
│         (Physics data, commands, subscriptions)              │
├─────────────────────────────────────────────────────────────┤
│                    Protocol Layer                            │
│    (Message framing, serialization, routing, sessions)       │
├─────────────────────────────────────────────────────────────┤
│                  Transport Abstraction                       │
│         (WebSocket, TCP, gRPC adapters)                      │
├─────────────────────────────────────────────────────────────┤
│                    Network Layer                             │
│              (TCP/IP, TLS, HTTP/2)                           │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. Message Format

### 2.1 Message Envelope

All messages use a standard envelope format:

```json
{
  "wpp": "1.0",
  "id": "msg-uuid-here",
  "type": "data",
  "timestamp": "2025-12-14T10:30:00.123Z",
  "payload": { ... },
  "metadata": { ... }
}
```

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `wpp` | string | Yes | Protocol version (semantic versioning) |
| `id` | string | Yes | Unique message identifier (UUID v4) |
| `type` | string | Yes | Message type (see Section 2.2) |
| `timestamp` | string | Yes | ISO 8601 timestamp with milliseconds |
| `payload` | object | Yes | Type-specific payload |
| `metadata` | object | No | Optional metadata |

### 2.2 Message Types

| Type | Direction | Description |
|------|-----------|-------------|
| `connect` | C → S | Connection request |
| `connect_ack` | S → C | Connection acknowledgment |
| `disconnect` | Both | Graceful disconnect |
| `subscribe` | C → S | Subscribe to data channel |
| `subscribe_ack` | S → C | Subscription acknowledgment |
| `unsubscribe` | C → S | Unsubscribe from channel |
| `unsubscribe_ack` | S → C | Unsubscription acknowledgment |
| `data` | S → C | Physics data (streaming) |
| `command` | C → S | Device control command |
| `response` | S → C | Command response |
| `event` | S → C | Asynchronous event notification |
| `error` | Both | Error notification |
| `ping` | C → S | Keepalive ping |
| `pong` | S → C | Keepalive pong |

---

## 3. Connection Management

### 3.1 Connect Message

```json
{
  "wpp": "1.0",
  "id": "msg-001",
  "type": "connect",
  "timestamp": "2025-12-14T10:30:00Z",
  "payload": {
    "client_id": "client-uuid",
    "client_name": "ITER Control System",
    "client_version": "2.1.0",
    "capabilities": ["streaming", "commands", "events"],
    "auth": {
      "method": "token",
      "token": "jwt-token-here"
    },
    "options": {
      "heartbeat_interval": 30000,
      "compression": "lz4",
      "binary_mode": false
    }
  }
}
```

### 3.2 Connect Acknowledgment

```json
{
  "wpp": "1.0",
  "id": "msg-002",
  "type": "connect_ack",
  "timestamp": "2025-12-14T10:30:00.050Z",
  "payload": {
    "session_id": "session-uuid",
    "server_name": "WIA Physics Hub",
    "server_version": "1.0.0",
    "capabilities": ["streaming", "commands", "events", "binary"],
    "heartbeat_interval": 30000,
    "max_subscriptions": 1000,
    "channels": [
      {"name": "fusion/iter/plasma", "type": "fusion"},
      {"name": "particle/lhc/atlas", "type": "particle"},
      {"name": "darkmatter/xenon/events", "type": "dark_matter"}
    ]
  }
}
```

### 3.3 Connection State Machine

```
     ┌──────────────┐
     │ Disconnected │
     └──────┬───────┘
            │ connect
            ▼
     ┌──────────────┐
     │  Connecting  │
     └──────┬───────┘
            │ connect_ack
            ▼
     ┌──────────────┐    ping/pong     ┌──────────────┐
     │  Connected   │◄────────────────►│   Active     │
     └──────┬───────┘                  └──────────────┘
            │ disconnect / timeout
            ▼
     ┌──────────────┐
     │ Disconnecting│
     └──────┬───────┘
            │
            ▼
     ┌──────────────┐
     │ Disconnected │
     └──────────────┘
```

### 3.4 Disconnect Message

```json
{
  "wpp": "1.0",
  "id": "msg-100",
  "type": "disconnect",
  "timestamp": "2025-12-14T11:00:00Z",
  "payload": {
    "reason": "client_shutdown",
    "message": "Graceful shutdown"
  }
}
```

Disconnect reasons:
- `client_shutdown` - Client initiated shutdown
- `server_shutdown` - Server initiated shutdown
- `timeout` - Connection timeout
- `error` - Protocol error
- `auth_failure` - Authentication failure

---

## 4. Subscription Management

### 4.1 Subscribe Message

```json
{
  "wpp": "1.0",
  "id": "msg-010",
  "type": "subscribe",
  "timestamp": "2025-12-14T10:31:00Z",
  "payload": {
    "channel": "fusion/iter/plasma",
    "filter": {
      "quality": ["GOOD", "VALIDATED"],
      "min_rate": 10
    },
    "options": {
      "buffer_size": 100,
      "throttle_ms": 100,
      "include_history": true,
      "history_limit": 1000
    }
  }
}
```

### 4.2 Channel Naming Convention

Channels follow a hierarchical naming pattern:

```
{domain}/{facility}/{subsystem}[/{component}]
```

Examples:
- `fusion/iter/plasma` - ITER plasma data
- `fusion/iter/magnets/toroidal` - ITER toroidal magnets
- `particle/lhc/atlas/triggers` - ATLAS trigger data
- `darkmatter/xenon/detector/events` - XENON detector events

### 4.3 Wildcard Subscriptions

| Pattern | Description |
|---------|-------------|
| `fusion/*` | All fusion facilities |
| `fusion/iter/*` | All ITER subsystems |
| `*/lhc/*` | All LHC channels across domains |

### 4.4 Subscribe Acknowledgment

```json
{
  "wpp": "1.0",
  "id": "msg-011",
  "type": "subscribe_ack",
  "timestamp": "2025-12-14T10:31:00.050Z",
  "payload": {
    "channel": "fusion/iter/plasma",
    "subscription_id": "sub-uuid",
    "status": "active",
    "effective_rate": 10,
    "history_sent": 500
  }
}
```

---

## 5. Data Streaming

### 5.1 Data Message

Data messages carry physics data in Phase 1 schema format:

```json
{
  "wpp": "1.0",
  "id": "msg-1000",
  "type": "data",
  "timestamp": "2025-12-14T10:31:01.123Z",
  "payload": {
    "channel": "fusion/iter/plasma",
    "sequence": 12345,
    "data_type": "fusion",
    "data": {
      "metadata": {
        "id": "fusion-2025-001",
        "experiment": {"name": "ITER"},
        "created": "2025-12-14T10:31:01Z"
      },
      "plasma": {
        "temperature": {
          "value": 150000000,
          "uncertainty": {"total": 11180340},
          "unit": "K"
        },
        "density": {
          "value": 1e20,
          "uncertainty": {"total": 1e18},
          "unit": "m^-3"
        }
      },
      "quality": "GOOD"
    }
  }
}
```

### 5.2 Data Types

| Type | Schema | Description |
|------|--------|-------------|
| `fusion` | fusion.schema.json | Fusion plasma data |
| `time_crystal` | time-crystal.schema.json | Time crystal data |
| `particle` | particle.schema.json | Particle physics data |
| `dark_matter` | dark-matter.schema.json | Dark matter data |
| `antimatter` | antimatter.schema.json | Antimatter data |
| `quantum_gravity` | quantum-gravity.schema.json | Quantum gravity data |

### 5.3 Batch Data Message

For high-volume scenarios:

```json
{
  "wpp": "1.0",
  "id": "msg-2000",
  "type": "data",
  "timestamp": "2025-12-14T10:31:02Z",
  "payload": {
    "channel": "particle/lhc/atlas/events",
    "batch": true,
    "sequence_start": 100000,
    "count": 100,
    "data_type": "particle",
    "items": [
      { "event_id": "evt-001", ... },
      { "event_id": "evt-002", ... },
      ...
    ]
  }
}
```

---

## 6. Command and Control

### 6.1 Command Message

```json
{
  "wpp": "1.0",
  "id": "msg-500",
  "type": "command",
  "timestamp": "2025-12-14T10:35:00Z",
  "payload": {
    "target": "fusion/iter/heating/icrf",
    "action": "set_power",
    "parameters": {
      "power": 10.0,
      "unit": "MW",
      "ramp_time": 5.0
    },
    "timeout": 30000,
    "priority": "normal"
  }
}
```

### 6.2 Command Response

```json
{
  "wpp": "1.0",
  "id": "msg-501",
  "type": "response",
  "timestamp": "2025-12-14T10:35:00.100Z",
  "payload": {
    "command_id": "msg-500",
    "status": "success",
    "result": {
      "actual_power": 10.0,
      "achieved_at": "2025-12-14T10:35:05Z"
    },
    "execution_time": 5100
  }
}
```

### 6.3 Command Status Codes

| Status | Description |
|--------|-------------|
| `success` | Command executed successfully |
| `pending` | Command queued for execution |
| `executing` | Command in progress |
| `failed` | Command failed |
| `timeout` | Command timed out |
| `rejected` | Command rejected (auth/validation) |
| `cancelled` | Command cancelled |

---

## 7. Event Notifications

### 7.1 Event Message

```json
{
  "wpp": "1.0",
  "id": "msg-600",
  "type": "event",
  "timestamp": "2025-12-14T10:40:00Z",
  "payload": {
    "source": "fusion/iter/plasma",
    "event_type": "state_change",
    "severity": "info",
    "data": {
      "previous_state": "heating",
      "current_state": "burning",
      "reason": "Q > 10 achieved"
    }
  }
}
```

### 7.2 Event Types

| Type | Description |
|------|-------------|
| `state_change` | Device/system state change |
| `threshold` | Value crossed threshold |
| `alarm` | Alarm condition |
| `warning` | Warning condition |
| `info` | Informational |
| `discovery` | Scientific discovery event |

### 7.3 Severity Levels

| Level | Description |
|-------|-------------|
| `critical` | Immediate action required |
| `error` | Error condition |
| `warning` | Warning condition |
| `info` | Informational |
| `debug` | Debug information |

---

## 8. Error Handling

### 8.1 Error Message

```json
{
  "wpp": "1.0",
  "id": "msg-700",
  "type": "error",
  "timestamp": "2025-12-14T10:45:00Z",
  "payload": {
    "code": 4001,
    "category": "subscription",
    "message": "Channel not found",
    "details": {
      "channel": "fusion/unknown/plasma",
      "available_channels": ["fusion/iter/plasma", "fusion/jet/plasma"]
    },
    "related_message_id": "msg-010"
  }
}
```

### 8.2 Error Codes

| Range | Category | Description |
|-------|----------|-------------|
| 1000-1999 | Connection | Connection errors |
| 2000-2999 | Authentication | Auth errors |
| 3000-3999 | Protocol | Protocol errors |
| 4000-4999 | Subscription | Subscription errors |
| 5000-5999 | Command | Command errors |
| 6000-6999 | Data | Data errors |
| 7000-7999 | Server | Server errors |

#### Connection Errors (1000-1999)
| Code | Message |
|------|---------|
| 1001 | Connection refused |
| 1002 | Connection timeout |
| 1003 | Connection closed |
| 1004 | Maximum connections exceeded |
| 1005 | Invalid protocol version |

#### Authentication Errors (2000-2999)
| Code | Message |
|------|---------|
| 2001 | Authentication required |
| 2002 | Invalid credentials |
| 2003 | Token expired |
| 2004 | Insufficient permissions |

#### Protocol Errors (3000-3999)
| Code | Message |
|------|---------|
| 3001 | Invalid message format |
| 3002 | Unknown message type |
| 3003 | Missing required field |
| 3004 | Invalid field value |
| 3005 | Message too large |

#### Subscription Errors (4000-4999)
| Code | Message |
|------|---------|
| 4001 | Channel not found |
| 4002 | Invalid channel pattern |
| 4003 | Subscription limit exceeded |
| 4004 | Already subscribed |
| 4005 | Not subscribed |

#### Command Errors (5000-5999)
| Code | Message |
|------|---------|
| 5001 | Unknown command |
| 5002 | Invalid parameters |
| 5003 | Target not found |
| 5004 | Command timeout |
| 5005 | Command rejected |

---

## 9. Keepalive Mechanism

### 9.1 Ping Message

```json
{
  "wpp": "1.0",
  "id": "msg-ping-001",
  "type": "ping",
  "timestamp": "2025-12-14T10:50:00Z",
  "payload": {}
}
```

### 9.2 Pong Message

```json
{
  "wpp": "1.0",
  "id": "msg-pong-001",
  "type": "pong",
  "timestamp": "2025-12-14T10:50:00.005Z",
  "payload": {
    "ping_id": "msg-ping-001",
    "latency_ms": 5
  }
}
```

### 9.3 Heartbeat Configuration

| Parameter | Default | Description |
|-----------|---------|-------------|
| `heartbeat_interval` | 30000 ms | Interval between pings |
| `heartbeat_timeout` | 10000 ms | Timeout for pong response |
| `max_missed_heartbeats` | 3 | Max missed before disconnect |

---

## 10. Transport Adapters

### 10.1 WebSocket Transport

**Primary transport for web clients and general use.**

| Aspect | Details |
|--------|---------|
| URL Format | `wss://host:port/wpp/v1` |
| Subprotocol | `wpp.v1` |
| Frame Type | Text (JSON) or Binary |
| Compression | Per-message deflate |

Connection example:
```javascript
const ws = new WebSocket('wss://physics.wia.live/wpp/v1', 'wpp.v1');
```

### 10.2 TCP Transport

**High-performance transport for native applications.**

| Aspect | Details |
|--------|---------|
| Port | 5740 (default) |
| TLS | Required in production |
| Framing | Length-prefixed (4-byte big-endian) |
| Format | JSON or MessagePack |

Frame format:
```
┌──────────────┬──────────────────────────┐
│ Length (4B)  │ Payload (N bytes)        │
│ Big-endian   │ JSON or MessagePack      │
└──────────────┴──────────────────────────┘
```

### 10.3 gRPC Transport

**Future transport for high-performance microservices.**

```protobuf
service WiaPhysics {
  rpc Connect(ConnectRequest) returns (ConnectResponse);
  rpc Subscribe(SubscribeRequest) returns (stream DataMessage);
  rpc Command(CommandRequest) returns (CommandResponse);
  rpc Events(EventsRequest) returns (stream EventMessage);
}
```

---

## 11. Binary Mode

### 11.1 Binary Encoding

For high-volume data, binary mode uses MessagePack encoding with optional LZ4 compression.

```json
{
  "wpp": "1.0",
  "id": "msg-001",
  "type": "connect",
  "payload": {
    "options": {
      "binary_mode": true,
      "compression": "lz4"
    }
  }
}
```

### 11.2 Binary Frame Format

```
┌────────────┬───────────┬────────────────────────┐
│ Flags (1B) │ Len (4B)  │ Payload (N bytes)      │
│            │           │ MessagePack + LZ4      │
└────────────┴───────────┴────────────────────────┘

Flags:
  Bit 0: Compressed (1 = LZ4 compressed)
  Bit 1: Continued (1 = more frames follow)
  Bit 2-7: Reserved
```

---

## 12. Security

### 12.1 Authentication Methods

| Method | Description |
|--------|-------------|
| `token` | JWT bearer token |
| `api_key` | API key authentication |
| `certificate` | Client certificate (mTLS) |

### 12.2 Authorization

Channels support role-based access:

```json
{
  "channel": "fusion/iter/plasma",
  "permissions": {
    "read": ["operator", "scientist", "admin"],
    "write": ["operator", "admin"],
    "admin": ["admin"]
  }
}
```

### 12.3 Transport Security

| Transport | Security |
|-----------|----------|
| WebSocket | WSS (TLS 1.3) |
| TCP | TLS 1.3 |
| gRPC | TLS 1.3 / mTLS |

---

## 13. Quality of Service

### 13.1 Delivery Guarantees

| Level | Description |
|-------|-------------|
| `at_most_once` | Fire and forget (default for data) |
| `at_least_once` | With acknowledgment |
| `exactly_once` | With deduplication |

### 13.2 Flow Control

```json
{
  "type": "subscribe",
  "payload": {
    "channel": "particle/lhc/atlas",
    "options": {
      "qos": "at_least_once",
      "flow_control": {
        "max_in_flight": 100,
        "ack_interval": 10
      }
    }
  }
}
```

---

## 14. Compatibility

### 14.1 EPICS Bridge

The protocol supports bridging to EPICS systems:

```json
{
  "type": "subscribe",
  "payload": {
    "channel": "epics:ITER:PLASMA:TEMP",
    "bridge": "epics",
    "options": {
      "pv_name": "ITER:PLASMA:TEMP",
      "monitor": true
    }
  }
}
```

### 14.2 TANGO Bridge

```json
{
  "type": "subscribe",
  "payload": {
    "channel": "tango:esrf/plasma/temperature",
    "bridge": "tango",
    "options": {
      "device": "esrf/plasma",
      "attribute": "temperature"
    }
  }
}
```

---

## 15. Implementation Notes

### 15.1 Rust SDK Integration

The protocol integrates with Phase 2 Rust SDK types:

```rust
use wia_physics::prelude::*;
use wia_physics::protocol::*;

// Create client
let client = WppClient::connect("wss://physics.wia.live/wpp/v1").await?;

// Subscribe to fusion data
let mut stream = client.subscribe("fusion/iter/plasma").await?;

// Receive typed data
while let Some(msg) = stream.next().await {
    let data: FusionData = msg.into_data()?;
    println!("Temperature: {} K", data.plasma.temperature.value);
}
```

### 15.2 Recommended Timeouts

| Operation | Timeout |
|-----------|---------|
| Connect | 10 seconds |
| Subscribe | 5 seconds |
| Command | 30 seconds |
| Heartbeat | 10 seconds |

---

## 16. References

- [EPICS Channel Access Protocol](https://epics.anl.gov/base/R3-16/1-docs/CAproto/index.html)
- [TANGO Controls](https://www.tango-controls.org/)
- [WebSocket RFC 6455](https://tools.ietf.org/html/rfc6455)
- [gRPC](https://grpc.io/)
- [MessagePack](https://msgpack.org/)
- [LZ4](https://lz4.github.io/lz4/)

---

**弘益人間** 🤟

---

*Specification Date: 2025-12-14*
*Next: Protocol Implementation (Rust)*
