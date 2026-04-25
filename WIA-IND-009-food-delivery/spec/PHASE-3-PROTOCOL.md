# WIA-IND-009: PHASE 3 - PROTOCOL SPECIFICATION
## Food Delivery Platform Standard
### Version 1.0 | 弘益人間 (Benefit All Humanity)

---

## Table of Contents
1. [Overview](#overview)
2. [WebSocket Protocol](#websocket-protocol)
3. [Real-time Updates](#real-time-updates)
4. [Push Notifications](#push-notifications)
5. [Event Streaming](#event-streaming)
6. [State Synchronization](#state-synchronization)
7. [Offline Handling](#offline-handling)
8. [Security Protocols](#security-protocols)

---

## Overview

Phase 3 specifies real-time communication protocols for food delivery platforms. These protocols enable live tracking, instant notifications, and synchronized state across clients and servers.

**Protocol Stack:**
- WebSocket (RFC 6455) for bidirectional real-time communication
- Server-Sent Events (SSE) for server-to-client streaming
- HTTP/2 Server Push for proactive resource delivery
- MQTT for lightweight IoT device communication (optional)

**Design Principles:**
- **Low Latency**: Sub-second message delivery
- **Reliability**: Guaranteed message delivery with retry mechanisms
- **Scalability**: Support millions of concurrent connections
- **Efficiency**: Minimal bandwidth and battery consumption
- **Resilience**: Graceful handling of network disruptions

---

## WebSocket Protocol

### Connection Establishment

**Client Initiates Connection:**
```
GET wss://api.{platform-domain}/v1/realtime HTTP/1.1
Host: api.{platform-domain}
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==
Sec-WebSocket-Version: 13
Sec-WebSocket-Protocol: wia-ind-009-v1
Authorization: Bearer {access_token}
```

**Server Response:**
```
HTTP/1.1 101 Switching Protocols
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Accept: s3pPLMBiTxaQ9kYGzzhZRbK+xOo=
Sec-WebSocket-Protocol: wia-ind-009-v1
```

### Message Format

All WebSocket messages use JSON format:

```json
{
  "type": "message_type",
  "id": "unique-message-id",
  "timestamp": "2025-12-27T12:00:00Z",
  "payload": {...}
}
```

### Message Types

**Client to Server:**
- `subscribe` - Subscribe to event streams
- `unsubscribe` - Unsubscribe from streams
- `location_update` - Driver location update
- `status_change` - Status change notification
- `ping` - Keep-alive ping

**Server to Client:**
- `order_update` - Order status changed
- `location_update` - Driver location changed
- `notification` - Push notification
- `error` - Error message
- `pong` - Keep-alive response

### Subscribe to Order Updates

**Client Sends:**
```json
{
  "type": "subscribe",
  "id": "sub-001",
  "timestamp": "2025-12-27T12:00:00Z",
  "payload": {
    "channel": "orders",
    "filter": {
      "orderId": "ORD-2025-001234"
    }
  }
}
```

**Server Acknowledges:**
```json
{
  "type": "subscribe_ack",
  "id": "sub-001",
  "timestamp": "2025-12-27T12:00:01Z",
  "payload": {
    "status": "subscribed",
    "channel": "orders.ORD-2025-001234"
  }
}
```

### Location Update

**Driver Client Sends:**
```json
{
  "type": "location_update",
  "id": "loc-123",
  "timestamp": "2025-12-27T12:25:30Z",
  "payload": {
    "driverId": "DRV-45789",
    "orderId": "ORD-2025-001234",
    "location": {
      "latitude": 40.7589,
      "longitude": -73.9851,
      "accuracy": 12,
      "heading": 90,
      "speed": 25
    }
  }
}
```

### Keep-Alive Mechanism

**Heartbeat Interval:** 30 seconds

**Client Ping:**
```json
{
  "type": "ping",
  "id": "ping-001",
  "timestamp": "2025-12-27T12:00:00Z"
}
```

**Server Pong:**
```json
{
  "type": "pong",
  "id": "ping-001",
  "timestamp": "2025-12-27T12:00:00Z",
  "payload": {
    "serverTime": "2025-12-27T12:00:00.123Z"
  }
}
```

### Connection Lifecycle

1. **Establish**: Client connects with authentication
2. **Subscribe**: Client subscribes to relevant channels
3. **Active**: Bidirectional message exchange
4. **Ping/Pong**: Periodic keep-alive
5. **Reconnect**: Automatic reconnection on disconnect
6. **Close**: Graceful connection termination

### Reconnection Strategy

**Exponential Backoff:**
- Initial retry: 1 second
- Subsequent retries: Previous delay × 2
- Maximum delay: 60 seconds
- Maximum attempts: Unlimited (with backoff)

**Reconnection Flow:**
```json
{
  "type": "reconnect",
  "id": "recon-001",
  "timestamp": "2025-12-27T12:05:00Z",
  "payload": {
    "lastMessageId": "msg-999",
    "subscriptions": ["orders.ORD-2025-001234"]
  }
}
```

Server responds with missed messages since `lastMessageId`.

---

## Real-time Updates

### Order Status Updates

**Server Pushes to Subscribed Clients:**
```json
{
  "type": "order_update",
  "id": "upd-001",
  "timestamp": "2025-12-27T12:20:00Z",
  "payload": {
    "orderId": "ORD-2025-001234",
    "status": "PREPARING",
    "previousStatus": "CONFIRMED",
    "estimatedPreparation": "2025-12-27T12:30:00Z",
    "message": "Your food is being prepared"
  }
}
```

### Driver Location Updates

**Update Frequency:**
- Active delivery: Every 5-10 seconds
- Stationary: Every 30 seconds
- Low battery (<20%): Reduced frequency
- Poor network: Batch and send when reconnected

**Location Stream:**
```json
{
  "type": "location_update",
  "id": "loc-456",
  "timestamp": "2025-12-27T12:25:35Z",
  "payload": {
    "driverId": "DRV-45789",
    "orderId": "ORD-2025-001234",
    "location": {
      "latitude": 40.7590,
      "longitude": -73.9850,
      "accuracy": 10
    },
    "eta": "2025-12-27T12:40:00Z"
  }
}
```

### ETA Updates

**Dynamic ETA Recalculation:**
```json
{
  "type": "eta_update",
  "id": "eta-001",
  "timestamp": "2025-12-27T12:28:00Z",
  "payload": {
    "orderId": "ORD-2025-001234",
    "previousEta": "2025-12-27T12:40:00Z",
    "newEta": "2025-12-27T12:45:00Z",
    "reason": "traffic_delay",
    "confidence": 0.85
  }
}
```

---

## Push Notifications

### Notification Types

**Order Lifecycle Notifications:**
- `order_confirmed` - Restaurant accepted order
- `preparing` - Food preparation started
- `ready_for_pickup` - Food ready
- `driver_assigned` - Driver assigned
- `out_for_delivery` - Driver picked up order
- `nearby` - Driver approaching (within 5 min)
- `delivered` - Order delivered
- `completed` - Order fully completed

### Notification Format

```json
{
  "type": "notification",
  "id": "notif-001",
  "timestamp": "2025-12-27T12:00:15Z",
  "payload": {
    "notificationType": "order_confirmed",
    "priority": "high",
    "orderId": "ORD-2025-001234",
    "title": "Order Confirmed",
    "body": "Your order from Pizza Hut has been confirmed",
    "data": {
      "orderId": "ORD-2025-001234",
      "estimatedDelivery": "2025-12-27T12:40:00Z"
    },
    "actions": [
      {
        "action": "view_order",
        "title": "Track Order"
      }
    ]
  }
}
```

### Delivery Channels

1. **WebSocket** - For active connections
2. **Push Notifications** - FCM (Android), APNs (iOS)
3. **SMS** - For critical updates
4. **Email** - For receipts and confirmations

### Notification Preferences

Users can configure notification preferences:

```json
{
  "channels": {
    "push": true,
    "sms": false,
    "email": true
  },
  "events": {
    "order_confirmed": true,
    "preparing": false,
    "out_for_delivery": true,
    "delivered": true
  },
  "quietHours": {
    "enabled": true,
    "start": "22:00",
    "end": "08:00"
  }
}
```

---

## Event Streaming

### Server-Sent Events (SSE)

For clients that prefer SSE over WebSocket:

```
GET /v1/stream/orders/{orderId} HTTP/1.1
Host: api.{platform-domain}
Accept: text/event-stream
Authorization: Bearer {access_token}
```

**Server Response:**
```
HTTP/1.1 200 OK
Content-Type: text/event-stream
Cache-Control: no-cache
Connection: keep-alive

event: order_update
id: 1
data: {"orderId":"ORD-2025-001234","status":"PREPARING"}

event: location_update
id: 2
data: {"driverId":"DRV-45789","location":{...}}

event: eta_update
id: 3
data: {"orderId":"ORD-2025-001234","eta":"2025-12-27T12:40:00Z"}
```

### Event Replay

Clients can request missed events:

```
GET /v1/stream/orders/{orderId}?lastEventId=100 HTTP/1.1
```

Server replays events after `lastEventId`.

---

## State Synchronization

### State Reconciliation

After reconnection, client and server synchronize state:

**Client Sends State Digest:**
```json
{
  "type": "sync_request",
  "id": "sync-001",
  "timestamp": "2025-12-27T12:05:00Z",
  "payload": {
    "orders": {
      "ORD-2025-001234": {
        "status": "CONFIRMED",
        "version": 5
      }
    }
  }
}
```

**Server Responds with Delta:**
```json
{
  "type": "sync_response",
  "id": "sync-001",
  "timestamp": "2025-12-27T12:05:01Z",
  "payload": {
    "updates": [
      {
        "orderId": "ORD-2025-001234",
        "status": "PREPARING",
        "version": 6,
        "changes": [...]
      }
    ]
  }
}
```

### Conflict Resolution

**Last-Write-Wins (LWW):**
- Use timestamp to determine winning update
- Server timestamp is authoritative

**Vector Clocks (Advanced):**
- For distributed systems
- Detect concurrent updates
- Merge conflicting states

---

## Offline Handling

### Offline Queue

Clients buffer messages when offline:

```json
{
  "queuedMessages": [
    {
      "type": "location_update",
      "id": "loc-789",
      "timestamp": "2025-12-27T12:10:00Z",
      "payload": {...}
    },
    {
      "type": "status_change",
      "id": "status-123",
      "timestamp": "2025-12-27T12:11:00Z",
      "payload": {...}
    }
  ]
}
```

### Bulk Upload on Reconnect

```json
{
  "type": "bulk_update",
  "id": "bulk-001",
  "timestamp": "2025-12-27T12:15:00Z",
  "payload": {
    "messages": [...]
  }
}
```

### Offline Data Access

Clients cache critical data locally:
- Active order details
- Restaurant menus
- Delivery addresses
- Navigation maps

---

## Security Protocols

### Transport Security

- **TLS 1.3** required for all connections
- **Certificate pinning** recommended for mobile apps
- **Perfect Forward Secrecy** (PFS)

### Message Authentication

**HMAC Signatures:**
```json
{
  "type": "location_update",
  "id": "loc-999",
  "timestamp": "2025-12-27T12:25:30Z",
  "payload": {...},
  "signature": "a1b2c3d4e5f6..."
}
```

Signature computed as:
```
HMAC-SHA256(secret, type + id + timestamp + JSON(payload))
```

### Rate Limiting

**Per Connection:**
- Maximum 100 messages per second
- Burst allowance: 200 messages
- Violations result in connection throttling or termination

**Per User:**
- Maximum 10 active WebSocket connections
- Maximum 1000 location updates per hour

### Access Control

Clients can only:
- Subscribe to their own orders
- Update their own status/location
- Receive notifications intended for them

Server validates authorization on every message.

---

## Performance Recommendations

**Message Compression:**
- Use gzip or deflate for large payloads
- Binary formats (MessagePack, Protocol Buffers) for efficiency

**Connection Pooling:**
- Reuse connections across multiple subscriptions
- Limit connections per client

**Adaptive Update Frequency:**
- Reduce frequency when battery low
- Increase frequency when approaching delivery

**Bandwidth Optimization:**
- Send only changed fields (delta updates)
- Aggregate multiple small updates

---

**Copyright © 2025 SmileStory Inc. / WIA**
**弘益人間 (Benefit All Humanity)**

---

## Annex A — Conformance Tier Matrix

WIA conformance for WIA-IND-009-food-delivery is evaluated across three tiers:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references published standards alongside this document. Implementers SHOULD review the relevant standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- ISO/IEC 17065:2012 — Conformity assessment — Requirements for bodies certifying products, processes and services
- ISO/IEC 27001:2022 — Information security management systems
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- W3C PROV-DM — provenance data model

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/WIA-IND-009-food-delivery/api/` — TypeScript SDK skeleton
- `wia-standards/standards/WIA-IND-009-food-delivery/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/WIA-IND-009-food-delivery/simulator/` — interactive browser-based simulator for the PHASE protocol

Each reference artifact ships with an MIT license and is intended as a starting point, not as a production-grade implementation.

### C.2 Test Vectors

A normative set of request/response pairs covering the schemas defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector in both serialization (request) and deserialization (response) directions, and MUST publish a signed report identifying which vectors were exercised.

Test vectors are versioned independently of the PHASE document; refer to the `test-vectors/` directory for the active version.

---

## Annex D — Open Questions and Future Work

This PHASE document captures the consensus position at v1.0. The following items are tracked for future minor or major revisions:

1. **Schema versioning policy** — formal MUST/SHOULD rules for backward-compatible vs. breaking changes between minor releases.
2. **Privacy-preserving aggregation** — guidance on differential privacy and secure aggregation patterns for telemetry channels, without prescribing a specific algorithm.
3. **Multilingual error catalogs** — localization strategy for the standard error codes defined in this PHASE.
4. **Long-term retention** — alignment with sectoral retention regulations across jurisdictions.
5. **Sustainability disclosure** — optional fields for energy and emissions reporting tied to operations covered by this PHASE.

Items in this annex are non-normative. Comments and proposals are accepted via the GitHub issues tracker on the WIA-Official organization.

---
