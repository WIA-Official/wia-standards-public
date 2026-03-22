# Vehicle Infotainment — Phase 3: Protocol Specification

> **Version:** 1.0.0
> **Status:** Official
> **Last Updated:** 2025-01-01
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 3 defines communication protocols for real-time data exchange between Vehicle Infotainment components, including transportation networks and vehicular systems. This specification ensures secure, efficient, and interoperable data streaming.

### 1.1 Protocol Stack

```
┌─────────────────────────────────────┐
│      Application Layer              │
│   (vehicle-infotainment Messages)             │
├─────────────────────────────────────┤
│      Presentation Layer             │
│   (JSON / Protocol Buffers)         │
├─────────────────────────────────────┤
│      Session Layer                  │
│   (Connection Management)           │
├─────────────────────────────────────┤
│      Transport Layer                │
│   (WebSocket / gRPC / MQTT)         │
├─────────────────────────────────────┤
│      Security Layer                 │
│   (TLS 1.3 / mTLS)                 │
└─────────────────────────────────────┘
```

---

## 2. Transport Layer

### 2.1 Supported Transports

| Transport | Port | Use Case | Latency |
|-----------|------|----------|---------|
| WebSocket | 443 | Web clients, dashboards | Low |
| gRPC | 443 | Server-to-server, high throughput | Very Low |
| MQTT | 8883 | IoT devices, sensors | Low |
| HTTP/2 | 443 | REST fallback, batch operations | Medium |

### 2.2 Connection Management

- Heartbeat interval: 30 seconds
- Connection timeout: 60 seconds
- Maximum reconnection attempts: 10 with exponential backoff
- Session resumption supported via session tokens

---

## 3. Message Format

### 3.1 Message Envelope

```json
{
  "header": {
    "messageId": "uuid-v4",
    "type": "DATA|COMMAND|EVENT|ACK|ERROR",
    "source": "node-id",
    "destination": "node-id-or-broadcast",
    "timestamp": "2025-01-15T10:30:00.000Z",
    "correlationId": "uuid-v4",
    "version": "1.0.0"
  },
  "payload": {},
  "signature": "base64-encoded-hmac"
}
```

### 3.2 Message Types

| Type | Direction | Description |
|------|-----------|-------------|
| DATA | Uni/Bi | vehicle data transmission |
| COMMAND | Request | Control instruction |
| EVENT | Publish | State change notification |
| ACK | Response | Delivery confirmation |
| ERROR | Response | Error notification |

---

## 4. Protocol Flow

### 4.1 Connection Establishment

```
Client                          Server
  │                                │
  │─── 1. TLS Handshake ─────────▶│
  │◀── 2. Certificate Verify ─────│
  │─── 3. Auth Token ────────────▶│
  │◀── 4. Session Established ────│
  │─── 5. Subscribe Channels ────▶│
  │◀── 6. Subscription ACK ──────│
  │                                │
  │◀══ 7. Data Stream ═══════════▶│
```

### 4.2 Data Exchange

1. Publisher sends DATA message to designated channel
2. Server validates message schema and permissions
3. Server routes to subscribed consumers
4. Consumers send ACK upon successful processing
5. Server tracks delivery status per consumer

---

## 5. State Machine

### 5.1 Connection States

| State | Description | Transitions |
|-------|-------------|-------------|
| DISCONNECTED | No active connection | → CONNECTING |
| CONNECTING | Handshake in progress | → CONNECTED, DISCONNECTED |
| CONNECTED | Authenticated session | → SUBSCRIBED, DISCONNECTED |
| SUBSCRIBED | Receiving data | → CONNECTED, DISCONNECTED |
| RECONNECTING | Auto-reconnect | → CONNECTED, DISCONNECTED |

### 5.2 Message Processing States

| State | Description |
|-------|-------------|
| PENDING | Message queued for delivery |
| DELIVERED | Message received by consumer |
| PROCESSED | Consumer acknowledged processing |
| FAILED | Delivery or processing failed |
| EXPIRED | TTL exceeded |

---

## 6. Quality of Service

| QoS Level | Guarantee | Use Case |
|-----------|-----------|----------|
| 0 | At most once | Telemetry, non-critical |
| 1 | At least once | Standard operations |
| 2 | Exactly once | Financial, safety-critical |

---

## 7. Security

### 7.1 Requirements

- TLS 1.3 mandatory for all connections
- Mutual TLS (mTLS) for server-to-server
- Message-level signing with HMAC-SHA256
- Token rotation every 3600 seconds

### 7.2 Access Control

Channel-based permissions with role-based access control (RBAC):
- `publish`: Send messages to channel
- `subscribe`: Receive messages from channel
- `admin`: Manage channel configuration

---

**© 2025 SmileStory Inc. / WIA - World Certification Industry Association**
**弘益人間 · Benefit All Humanity**
