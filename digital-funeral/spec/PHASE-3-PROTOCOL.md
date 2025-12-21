# WIA Digital Funeral Standard - Phase 3: Protocol Specification

**Version:** 1.0.0
**Status:** Draft
**Date:** 2025-12-18
**Primary Color:** #64748B (Slate)
**Series:** Digital Death Services

---

## Table of Contents

1. [Introduction](#introduction)
2. [WebSocket Protocol](#websocket-protocol)
3. [Webhook System](#webhook-system)
4. [Event Streaming](#event-streaming)
5. [Real-Time Streaming Protocol](#real-time-streaming-protocol)
6. [Chat and Interaction Protocol](#chat-and-interaction-protocol)
7. [Synchronization Protocol](#synchronization-protocol)
8. [Security and Encryption](#security-and-encryption)
9. [Message Format Specifications](#message-format-specifications)
10. [Protocol State Management](#protocol-state-management)
11. [Implementation Examples](#implementation-examples)

---

## 1. Introduction

### 1.1 Purpose

This specification defines communication protocols for real-time interactions in digital funeral services. These protocols enable live streaming, real-time chat, event notifications, and data synchronization between clients and servers, ensuring seamless virtual and hybrid funeral experiences.

### 1.2 Protocol Overview

| Protocol | Use Case | Transport | Persistence |
|----------|----------|-----------|-------------|
| WebSocket | Real-time bidirectional communication | WSS (WebSocket Secure) | Connection-based |
| Webhook | Event notifications to external systems | HTTPS | Event-based |
| Server-Sent Events (SSE) | One-way server-to-client updates | HTTPS | Connection-based |
| WebRTC | Peer-to-peer audio/video streaming | UDP/TCP | Session-based |
| MQTT | IoT device integration | TCP/TLS | Subscription-based |

### 1.3 Connection Requirements

```
WebSocket URL: wss://ws.funeral-service.example.com/v1
Webhook Endpoint: https://your-server.com/webhooks/funeral-events
SSE Endpoint: https://api.funeral-service.example.com/v1/events/stream
MQTT Broker: mqtts://mqtt.funeral-service.example.com:8883
```

---

## 2. WebSocket Protocol

### 2.1 Connection Establishment

```javascript
// Client connection with authentication
const ws = new WebSocket('wss://ws.funeral-service.example.com/v1');

ws.onopen = () => {
  // Authenticate after connection
  ws.send(JSON.stringify({
    type: 'auth',
    payload: {
      token: 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...',
      clientId: 'client-12345',
      userAgent: 'Mozilla/5.0...'
    }
  }));
};
```

### 2.2 Authentication Message

```json
{
  "type": "auth",
  "payload": {
    "token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
    "clientId": "client-12345",
    "userAgent": "Mozilla/5.0...",
    "capabilities": ["video", "audio", "chat"]
  },
  "timestamp": "2025-12-18T14:00:00Z",
  "messageId": "msg_1234567890"
}
```

**Server Response:**
```json
{
  "type": "auth_success",
  "payload": {
    "sessionId": "session_abc123",
    "expiresAt": "2025-12-18T18:00:00Z",
    "permissions": ["stream.view", "chat.send", "react.send"]
  },
  "timestamp": "2025-12-18T14:00:01Z",
  "messageId": "msg_1234567891"
}
```

### 2.3 Subscription Management

```json
{
  "type": "subscribe",
  "payload": {
    "channels": [
      "service:660e8400-e29b-41d4-a716-446655440001",
      "chat:660e8400-e29b-41d4-a716-446655440001",
      "reactions:660e8400-e29b-41d4-a716-446655440001"
    ]
  },
  "timestamp": "2025-12-18T14:00:02Z",
  "messageId": "msg_1234567892"
}
```

**Server Confirmation:**
```json
{
  "type": "subscribed",
  "payload": {
    "channels": [
      "service:660e8400-e29b-41d4-a716-446655440001",
      "chat:660e8400-e29b-41d4-a716-446655440001",
      "reactions:660e8400-e29b-41d4-a716-446655440001"
    ],
    "subscriptionId": "sub_xyz789"
  },
  "timestamp": "2025-12-18T14:00:03Z",
  "messageId": "msg_1234567893"
}
```

### 2.4 Heartbeat and Keep-Alive

```json
{
  "type": "ping",
  "timestamp": "2025-12-18T14:05:00Z",
  "messageId": "msg_1234567894"
}
```

**Server Response:**
```json
{
  "type": "pong",
  "timestamp": "2025-12-18T14:05:00Z",
  "messageId": "msg_1234567895"
}
```

**Heartbeat Configuration:**
| Parameter | Value | Description |
|-----------|-------|-------------|
| Interval | 30 seconds | Client sends ping every 30s |
| Timeout | 60 seconds | Server closes if no ping received |
| Reconnect Delay | 5 seconds | Initial reconnection delay |
| Max Reconnect Attempts | 10 | Maximum retry attempts |

### 2.5 Message Types

| Type | Direction | Description |
|------|-----------|-------------|
| auth | Client → Server | Authentication request |
| auth_success | Server → Client | Authentication confirmed |
| auth_error | Server → Client | Authentication failed |
| subscribe | Client → Server | Channel subscription |
| subscribed | Server → Client | Subscription confirmed |
| unsubscribe | Client → Server | Unsubscribe from channel |
| message | Bidirectional | Generic message |
| chat | Bidirectional | Chat message |
| reaction | Client → Server | Emoji reaction |
| stream_update | Server → Client | Stream status update |
| viewer_count | Server → Client | Current viewer count |
| service_update | Server → Client | Service information update |
| error | Server → Client | Error notification |
| ping | Client → Server | Heartbeat ping |
| pong | Server → Client | Heartbeat pong |

---

## 3. Webhook System

### 3.1 Webhook Registration

```http
POST /v1/webhooks
Authorization: Bearer {token}
Content-Type: application/json

{
  "url": "https://your-server.com/webhooks/funeral-events",
  "events": [
    "service.created",
    "service.updated",
    "service.started",
    "service.ended",
    "rsvp.submitted",
    "condolence.posted",
    "donation.received",
    "stream.started",
    "stream.ended"
  ],
  "secret": "whsec_1234567890abcdef",
  "active": true
}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "id": "webhook_abc123",
    "url": "https://your-server.com/webhooks/funeral-events",
    "events": ["service.created", "service.updated", "..."],
    "secret": "whsec_1234567890abcdef",
    "active": true,
    "createdAt": "2025-12-18T10:00:00Z"
  }
}
```

### 3.2 Webhook Event Payload

```json
{
  "id": "evt_1234567890",
  "type": "service.started",
  "created": "2025-12-18T14:00:00Z",
  "data": {
    "serviceId": "660e8400-e29b-41d4-a716-446655440001",
    "deceasedId": "550e8400-e29b-41d4-a716-446655440000",
    "serviceType": "memorial",
    "startDateTime": "2025-12-18T14:00:00-08:00",
    "virtualService": {
      "enabled": true,
      "streamingUrl": "https://stream.example.com/service/660e8400",
      "viewerUrl": "https://watch.example.com/660e8400"
    }
  },
  "metadata": {
    "webhookId": "webhook_abc123",
    "attempt": 1,
    "signature": "t=1734529200,v1=5257a869e7ecebeda32affa62cdca3fa51cad7e77a0e56ff536d0ce8e108d8bd"
  }
}
```

### 3.3 Webhook Signature Verification

```javascript
const crypto = require('crypto');

function verifyWebhookSignature(payload, signature, secret) {
  const [timestampPart, signaturePart] = signature.split(',');
  const timestamp = timestampPart.split('=')[1];
  const expectedSignature = signaturePart.split('=')[1];

  const signedPayload = `${timestamp}.${JSON.stringify(payload)}`;
  const computedSignature = crypto
    .createHmac('sha256', secret)
    .update(signedPayload)
    .digest('hex');

  return crypto.timingSafeEqual(
    Buffer.from(expectedSignature),
    Buffer.from(computedSignature)
  );
}

// Usage
app.post('/webhooks/funeral-events', (req, res) => {
  const signature = req.headers['x-webhook-signature'];
  const payload = req.body;
  const secret = 'whsec_1234567890abcdef';

  if (!verifyWebhookSignature(payload, signature, secret)) {
    return res.status(401).json({ error: 'Invalid signature' });
  }

  // Process webhook event
  handleWebhookEvent(payload);

  res.status(200).json({ received: true });
});
```

### 3.4 Webhook Event Types

| Event Type | Trigger | Payload Includes |
|------------|---------|------------------|
| service.created | New service scheduled | Service details |
| service.updated | Service information changed | Updated fields |
| service.started | Service begins | Start time, stream URL |
| service.ended | Service concludes | End time, duration, stats |
| service.cancelled | Service cancelled | Cancellation reason |
| rsvp.submitted | Guest RSVP received | RSVP details |
| rsvp.updated | Guest updates RSVP | Changed fields |
| condolence.posted | New condolence message | Message content |
| donation.received | Donation processed | Amount, donor info |
| stream.started | Live stream begins | Stream URL, viewer URL |
| stream.ended | Live stream ends | Stats, recording URL |
| tribute.created | New memorial tribute | Tribute details |

### 3.5 Webhook Retry Policy

| Attempt | Delay | Total Elapsed Time |
|---------|-------|--------------------|
| 1 | Immediate | 0s |
| 2 | 5 seconds | 5s |
| 3 | 15 seconds | 20s |
| 4 | 1 minute | 1m 20s |
| 5 | 5 minutes | 6m 20s |
| 6 | 15 minutes | 21m 20s |
| 7 | 1 hour | 1h 21m 20s |
| 8 | 3 hours | 4h 21m 20s |

**Retry Criteria:**
- HTTP 5xx errors: Retry
- HTTP 4xx errors (except 429): Do not retry
- HTTP 429 (Rate Limited): Retry with exponential backoff
- Connection timeout: Retry
- DNS errors: Retry

---

## 4. Event Streaming

### 4.1 Server-Sent Events (SSE) Connection

```javascript
const eventSource = new EventSource(
  'https://api.funeral-service.example.com/v1/events/stream?token=eyJhbG...',
  { withCredentials: true }
);

eventSource.addEventListener('service_update', (event) => {
  const data = JSON.parse(event.data);
  console.log('Service update:', data);
});

eventSource.addEventListener('viewer_count', (event) => {
  const data = JSON.parse(event.data);
  console.log('Current viewers:', data.count);
});

eventSource.onerror = (error) => {
  console.error('SSE error:', error);
  // Implement reconnection logic
};
```

### 4.2 SSE Event Format

```
event: service_update
id: evt_1234567890
data: {"serviceId":"660e8400","status":"in-progress","currentSegment":"eulogy"}

event: viewer_count
id: evt_1234567891
data: {"serviceId":"660e8400","count":87,"peak":92}

event: chat_message
id: evt_1234567892
data: {"serviceId":"660e8400","author":"John Smith","message":"Beautiful service","timestamp":"2025-12-18T14:30:00Z"}
```

### 4.3 Event Stream Filtering

```http
GET /v1/events/stream?types=service_update,viewer_count&serviceId=660e8400
Authorization: Bearer {token}
```

### 4.4 Event Replay

```http
GET /v1/events/stream?lastEventId=evt_1234567890
Authorization: Bearer {token}
```

**Server Response:**
```
: Connection established
id: evt_1234567890
retry: 10000

event: service_update
id: evt_1234567891
data: {"serviceId":"660e8400","status":"in-progress"}

event: viewer_count
id: evt_1234567892
data: {"count":87}
```

---

## 5. Real-Time Streaming Protocol

### 5.1 WebRTC Signaling

```json
{
  "type": "offer",
  "payload": {
    "sessionId": "session_abc123",
    "serviceId": "660e8400-e29b-41d4-a716-446655440001",
    "sdp": "v=0\r\no=- 1234567890 1234567890 IN IP4 127.0.0.1\r\n...",
    "ice_candidates": [
      {
        "candidate": "candidate:1 1 UDP 2130706431 192.168.1.100 54321 typ host",
        "sdpMid": "0",
        "sdpMLineIndex": 0
      }
    ]
  },
  "timestamp": "2025-12-18T14:00:00Z"
}
```

### 5.2 Stream Quality Levels

| Quality | Resolution | Bitrate | Frame Rate | Use Case |
|---------|-----------|---------|------------|----------|
| low | 640x360 | 500 Kbps | 24 fps | Mobile, slow connections |
| medium | 1280x720 | 2 Mbps | 30 fps | Standard viewing |
| high | 1920x1080 | 5 Mbps | 30 fps | Desktop, fast connections |
| ultra | 3840x2160 | 15 Mbps | 60 fps | Premium viewing |

### 5.3 Adaptive Bitrate Streaming

```json
{
  "type": "stream_quality_change",
  "payload": {
    "serviceId": "660e8400-e29b-41d4-a716-446655440001",
    "previousQuality": "high",
    "newQuality": "medium",
    "reason": "bandwidth_reduction",
    "detectedBandwidth": 1500000,
    "requiredBandwidth": 2000000
  },
  "timestamp": "2025-12-18T14:15:00Z"
}
```

### 5.4 Stream Health Monitoring

```json
{
  "type": "stream_health",
  "payload": {
    "serviceId": "660e8400-e29b-41d4-a716-446655440001",
    "metrics": {
      "bitrate": 2100000,
      "framerate": 29.97,
      "packetLoss": 0.02,
      "latency": 150,
      "bufferHealth": 95,
      "qualityScore": 8.7
    },
    "issues": []
  },
  "timestamp": "2025-12-18T14:20:00Z"
}
```

### 5.5 HLS/DASH Manifest

**HLS Master Playlist (m3u8):**
```
#EXTM3U
#EXT-X-VERSION:6
#EXT-X-STREAM-INF:BANDWIDTH=500000,RESOLUTION=640x360,FRAME-RATE=24.000
low/index.m3u8
#EXT-X-STREAM-INF:BANDWIDTH=2000000,RESOLUTION=1280x720,FRAME-RATE=30.000
medium/index.m3u8
#EXT-X-STREAM-INF:BANDWIDTH=5000000,RESOLUTION=1920x1080,FRAME-RATE=30.000
high/index.m3u8
```

**Media Playlist:**
```
#EXTM3U
#EXT-X-VERSION:6
#EXT-X-TARGETDURATION:6
#EXT-X-MEDIA-SEQUENCE:0
#EXTINF:6.0,
segment_0.ts
#EXTINF:6.0,
segment_1.ts
#EXTINF:6.0,
segment_2.ts
```

---

## 6. Chat and Interaction Protocol

### 6.1 Chat Message

```json
{
  "type": "chat",
  "payload": {
    "serviceId": "660e8400-e29b-41d4-a716-446655440001",
    "author": {
      "id": "user_12345",
      "name": "John Smith",
      "relationship": "friend"
    },
    "message": "Thank you for sharing these beautiful memories of Dr. Johnson.",
    "timestamp": "2025-12-18T14:25:00Z",
    "metadata": {
      "clientId": "client-12345",
      "userAgent": "Mozilla/5.0..."
    }
  },
  "messageId": "msg_chat_1234567890"
}
```

**Server Broadcast:**
```json
{
  "type": "chat",
  "payload": {
    "id": "chat_msg_abc123",
    "serviceId": "660e8400-e29b-41d4-a716-446655440001",
    "author": {
      "name": "John Smith",
      "relationship": "friend"
    },
    "message": "Thank you for sharing these beautiful memories of Dr. Johnson.",
    "timestamp": "2025-12-18T14:25:00Z",
    "moderationStatus": "approved"
  },
  "messageId": "msg_chat_1234567891"
}
```

### 6.2 Reaction System

```json
{
  "type": "reaction",
  "payload": {
    "serviceId": "660e8400-e29b-41d4-a716-446655440001",
    "targetType": "service",
    "targetId": "660e8400-e29b-41d4-a716-446655440001",
    "reactionType": "heart",
    "userId": "user_12345"
  },
  "messageId": "msg_reaction_1234567892"
}
```

**Available Reactions:**
| Reaction | Unicode | Meaning |
|----------|---------|---------|
| heart | ❤️ | Love, support |
| pray | 🙏 | Prayer, gratitude |
| dove | 🕊️ | Peace |
| candle | 🕯️ | Remembrance |
| rose | 🌹 | Tribute |
| hug | 🤗 | Comfort, sympathy |

**Reaction Aggregation:**
```json
{
  "type": "reaction_summary",
  "payload": {
    "serviceId": "660e8400-e29b-41d4-a716-446655440001",
    "reactions": {
      "heart": 156,
      "pray": 89,
      "dove": 45,
      "candle": 123,
      "rose": 67,
      "hug": 78
    },
    "totalReactions": 558
  },
  "timestamp": "2025-12-18T14:30:00Z"
}
```

### 6.3 Chat Moderation

```json
{
  "type": "chat_moderate",
  "payload": {
    "messageId": "chat_msg_abc123",
    "action": "approve",
    "moderatorId": "moderator_12345",
    "reason": "appropriate content"
  },
  "messageId": "msg_moderate_1234567893"
}
```

**Moderation Actions:**
| Action | Effect | Notification |
|--------|--------|--------------|
| approve | Message visible to all | None |
| hide | Message hidden from view | Moderator only |
| delete | Message permanently removed | Author notified |
| flag | Mark for review | Moderator team |
| timeout_user | Temporarily mute user | User notified |
| ban_user | Permanently ban user | User notified |

### 6.4 Private Messages

```json
{
  "type": "private_message",
  "payload": {
    "serviceId": "660e8400-e29b-41d4-a716-446655440001",
    "recipientId": "user_67890",
    "message": "Would you like to share some words about Dr. Johnson?",
    "senderName": "Sarah Martinez"
  },
  "messageId": "msg_private_1234567894"
}
```

---

## 7. Synchronization Protocol

### 7.1 State Synchronization

```json
{
  "type": "sync_request",
  "payload": {
    "serviceId": "660e8400-e29b-41d4-a716-446655440001",
    "lastSyncTimestamp": "2025-12-18T14:00:00Z",
    "clientState": {
      "currentSegment": "eulogy",
      "chatScrollPosition": 150,
      "reactions": {"heart": 120}
    }
  },
  "messageId": "msg_sync_1234567895"
}
```

**Server Response:**
```json
{
  "type": "sync_response",
  "payload": {
    "serviceId": "660e8400-e29b-41d4-a716-446655440001",
    "serverState": {
      "status": "in-progress",
      "currentSegment": "video-tribute",
      "segmentStartedAt": "2025-12-18T14:25:00Z",
      "viewerCount": 87,
      "reactions": {"heart": 156, "pray": 89, "candle": 123}
    },
    "updates": [
      {
        "type": "segment_change",
        "from": "eulogy",
        "to": "video-tribute",
        "timestamp": "2025-12-18T14:25:00Z"
      }
    ],
    "syncTimestamp": "2025-12-18T14:30:00Z"
  },
  "messageId": "msg_sync_1234567896"
}
```

### 7.2 Conflict Resolution

| Conflict Type | Resolution Strategy | Priority |
|--------------|---------------------|----------|
| Simultaneous edits | Last-write-wins | Server timestamp |
| Chat message order | Server-assigned sequence | Server sequence |
| Reaction counts | Server aggregation | Server sum |
| Viewer count | Server authority | Server only |
| Stream state | Server source-of-truth | Server state |

### 7.3 Offline Support

```json
{
  "type": "offline_sync",
  "payload": {
    "clientId": "client-12345",
    "offlineSince": "2025-12-18T14:20:00Z",
    "onlineAt": "2025-12-18T14:35:00Z",
    "queuedActions": [
      {
        "type": "chat",
        "payload": {...},
        "timestamp": "2025-12-18T14:22:00Z"
      },
      {
        "type": "reaction",
        "payload": {...},
        "timestamp": "2025-12-18T14:28:00Z"
      }
    ]
  },
  "messageId": "msg_offline_1234567897"
}
```

---

## 8. Security and Encryption

### 8.1 Transport Layer Security

| Protocol | Encryption | Certificate Requirements |
|----------|-----------|--------------------------|
| WebSocket | WSS (TLS 1.3) | Valid SSL certificate |
| HTTPS | TLS 1.3 | Valid SSL certificate |
| WebRTC | DTLS-SRTP | Self-signed acceptable |
| MQTT | TLS 1.3 | Valid SSL certificate |

### 8.2 End-to-End Encryption for Private Messages

```json
{
  "type": "encrypted_message",
  "payload": {
    "recipientId": "user_67890",
    "encryptedContent": "AES256:iv:ciphertext",
    "algorithm": "AES-256-GCM",
    "keyId": "key_abc123"
  },
  "messageId": "msg_encrypted_1234567898"
}
```

### 8.3 Access Control

```json
{
  "type": "access_check",
  "payload": {
    "resourceType": "service",
    "resourceId": "660e8400-e29b-41d4-a716-446655440001",
    "requestedPermission": "stream.view",
    "userId": "user_12345"
  },
  "messageId": "msg_access_1234567899"
}
```

**Server Response:**
```json
{
  "type": "access_result",
  "payload": {
    "allowed": true,
    "permissions": ["stream.view", "chat.send", "react.send"],
    "expiresAt": "2025-12-18T18:00:00Z"
  },
  "messageId": "msg_access_1234567900"
}
```

### 8.4 Rate Limiting

| Action | Limit | Window | Penalty |
|--------|-------|--------|---------|
| Chat messages | 10 messages | 1 minute | 5 min mute |
| Reactions | 30 reactions | 1 minute | 1 min cooldown |
| API requests | 100 requests | 1 minute | 429 error |
| WebSocket messages | 60 messages | 1 minute | Connection throttle |

---

## 9. Message Format Specifications

### 9.1 Base Message Structure

```json
{
  "type": "string",
  "payload": {},
  "messageId": "msg_unique_identifier",
  "timestamp": "2025-12-18T14:00:00Z",
  "version": "1.0.0",
  "metadata": {
    "clientId": "client-12345",
    "requestId": "req_1234567890"
  }
}
```

### 9.2 Message Acknowledgment

```json
{
  "type": "ack",
  "payload": {
    "acknowledgedMessageId": "msg_1234567890",
    "status": "received"
  },
  "messageId": "msg_ack_1234567891",
  "timestamp": "2025-12-18T14:00:01Z"
}
```

### 9.3 Error Messages

```json
{
  "type": "error",
  "payload": {
    "code": "PERMISSION_DENIED",
    "message": "You do not have permission to send chat messages",
    "originalMessageId": "msg_1234567890",
    "retryable": false
  },
  "messageId": "msg_error_1234567892",
  "timestamp": "2025-12-18T14:00:02Z"
}
```

### 9.4 Batch Messages

```json
{
  "type": "batch",
  "payload": {
    "messages": [
      {
        "type": "viewer_count",
        "payload": {"count": 87}
      },
      {
        "type": "reaction_summary",
        "payload": {"reactions": {"heart": 156}}
      },
      {
        "type": "stream_health",
        "payload": {"metrics": {...}}
      }
    ]
  },
  "messageId": "msg_batch_1234567893",
  "timestamp": "2025-12-18T14:00:03Z"
}
```

---

## 10. Protocol State Management

### 10.1 Connection States

```
┌──────────┐
│  CLOSED  │
└────┬─────┘
     │ connect()
     ▼
┌──────────────┐
│ CONNECTING   │
└────┬─────────┘
     │ auth_success
     ▼
┌──────────────┐
│  CONNECTED   │◄──┐
└────┬─────────┘   │
     │             │ reconnect
     │ subscribe   │
     ▼             │
┌──────────────┐   │
│ SUBSCRIBED   │───┘
└────┬─────────┘
     │ error / disconnect
     ▼
┌──────────────┐
│ RECONNECTING │
└────┬─────────┘
     │ max_retries
     ▼
┌──────────────┐
│   CLOSED     │
└──────────────┘
```

### 10.2 Service States

| State | Description | Allowed Transitions |
|-------|-------------|---------------------|
| planned | Service scheduled | → confirmed, cancelled |
| confirmed | Service confirmed | → in-progress, postponed, cancelled |
| in-progress | Service ongoing | → completed, interrupted |
| completed | Service finished | None |
| interrupted | Service temporarily stopped | → in-progress, cancelled |
| postponed | Service rescheduled | → confirmed, cancelled |
| cancelled | Service cancelled | None |

### 10.3 Stream States

| State | Description | Duration |
|-------|-------------|----------|
| idle | Stream not started | Indefinite |
| preparing | Initializing stream | 30-60 seconds |
| live | Stream active | Service duration |
| paused | Temporarily paused | Up to 5 minutes |
| ended | Stream concluded | N/A |
| error | Stream failed | N/A |

---

## 11. Implementation Examples

### 11.1 Complete WebSocket Client

```javascript
class FuneralServiceWebSocket {
  constructor(serviceId, token) {
    this.serviceId = serviceId;
    this.token = token;
    this.ws = null;
    this.reconnectAttempts = 0;
    this.maxReconnectAttempts = 10;
    this.heartbeatInterval = null;
  }

  connect() {
    this.ws = new WebSocket('wss://ws.funeral-service.example.com/v1');

    this.ws.onopen = () => {
      console.log('WebSocket connected');
      this.authenticate();
      this.startHeartbeat();
      this.reconnectAttempts = 0;
    };

    this.ws.onmessage = (event) => {
      const message = JSON.parse(event.data);
      this.handleMessage(message);
    };

    this.ws.onerror = (error) => {
      console.error('WebSocket error:', error);
    };

    this.ws.onclose = () => {
      console.log('WebSocket closed');
      this.stopHeartbeat();
      this.reconnect();
    };
  }

  authenticate() {
    this.send({
      type: 'auth',
      payload: {
        token: this.token,
        clientId: this.generateClientId(),
        userAgent: navigator.userAgent
      }
    });
  }

  subscribe() {
    this.send({
      type: 'subscribe',
      payload: {
        channels: [
          `service:${this.serviceId}`,
          `chat:${this.serviceId}`,
          `reactions:${this.serviceId}`
        ]
      }
    });
  }

  sendChatMessage(message) {
    this.send({
      type: 'chat',
      payload: {
        serviceId: this.serviceId,
        message: message,
        timestamp: new Date().toISOString()
      }
    });
  }

  sendReaction(reactionType) {
    this.send({
      type: 'reaction',
      payload: {
        serviceId: this.serviceId,
        targetType: 'service',
        targetId: this.serviceId,
        reactionType: reactionType
      }
    });
  }

  handleMessage(message) {
    switch (message.type) {
      case 'auth_success':
        console.log('Authenticated successfully');
        this.subscribe();
        break;

      case 'subscribed':
        console.log('Subscribed to channels');
        break;

      case 'chat':
        this.onChatMessage(message.payload);
        break;

      case 'reaction_summary':
        this.onReactionUpdate(message.payload);
        break;

      case 'viewer_count':
        this.onViewerCountUpdate(message.payload);
        break;

      case 'service_update':
        this.onServiceUpdate(message.payload);
        break;

      case 'pong':
        // Heartbeat acknowledged
        break;

      case 'error':
        console.error('Server error:', message.payload);
        break;
    }
  }

  send(data) {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      data.messageId = this.generateMessageId();
      data.timestamp = new Date().toISOString();
      this.ws.send(JSON.stringify(data));
    }
  }

  startHeartbeat() {
    this.heartbeatInterval = setInterval(() => {
      this.send({ type: 'ping' });
    }, 30000); // 30 seconds
  }

  stopHeartbeat() {
    if (this.heartbeatInterval) {
      clearInterval(this.heartbeatInterval);
      this.heartbeatInterval = null;
    }
  }

  reconnect() {
    if (this.reconnectAttempts < this.maxReconnectAttempts) {
      this.reconnectAttempts++;
      const delay = Math.min(1000 * Math.pow(2, this.reconnectAttempts), 30000);
      console.log(`Reconnecting in ${delay}ms (attempt ${this.reconnectAttempts})`);

      setTimeout(() => {
        this.connect();
      }, delay);
    } else {
      console.error('Max reconnection attempts reached');
    }
  }

  generateClientId() {
    return 'client-' + Math.random().toString(36).substr(2, 9);
  }

  generateMessageId() {
    return 'msg_' + Date.now() + '_' + Math.random().toString(36).substr(2, 9);
  }

  // Event handlers (to be implemented by user)
  onChatMessage(data) {}
  onReactionUpdate(data) {}
  onViewerCountUpdate(data) {}
  onServiceUpdate(data) {}
}

// Usage
const wsClient = new FuneralServiceWebSocket(
  '660e8400-e29b-41d4-a716-446655440001',
  'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...'
);

wsClient.onChatMessage = (data) => {
  console.log(`${data.author.name}: ${data.message}`);
  // Update UI with chat message
};

wsClient.onViewerCountUpdate = (data) => {
  console.log(`Current viewers: ${data.count}`);
  // Update viewer count display
};

wsClient.connect();
```

### 11.2 Webhook Handler Implementation

```javascript
const express = require('express');
const crypto = require('crypto');
const app = express();

app.use(express.json());

const WEBHOOK_SECRET = 'whsec_1234567890abcdef';

function verifySignature(payload, signature) {
  const [timestampPart, signaturePart] = signature.split(',');
  const timestamp = timestampPart.split('=')[1];
  const expectedSignature = signaturePart.split('=')[1];

  const signedPayload = `${timestamp}.${JSON.stringify(payload)}`;
  const computedSignature = crypto
    .createHmac('sha256', WEBHOOK_SECRET)
    .update(signedPayload)
    .digest('hex');

  return crypto.timingSafeEqual(
    Buffer.from(expectedSignature),
    Buffer.from(computedSignature)
  );
}

app.post('/webhooks/funeral-events', (req, res) => {
  const signature = req.headers['x-webhook-signature'];
  const payload = req.body;

  // Verify signature
  if (!verifySignature(payload, signature)) {
    return res.status(401).json({ error: 'Invalid signature' });
  }

  // Process event based on type
  switch (payload.type) {
    case 'service.started':
      handleServiceStarted(payload.data);
      break;

    case 'service.ended':
      handleServiceEnded(payload.data);
      break;

    case 'donation.received':
      handleDonationReceived(payload.data);
      break;

    case 'condolence.posted':
      handleCondolencePosted(payload.data);
      break;

    default:
      console.log('Unhandled event type:', payload.type);
  }

  // Acknowledge receipt
  res.status(200).json({ received: true });
});

function handleServiceStarted(data) {
  console.log(`Service ${data.serviceId} started`);
  // Send notifications to family members
  // Update internal systems
}

function handleServiceEnded(data) {
  console.log(`Service ${data.serviceId} ended`);
  console.log(`Total viewers: ${data.statistics.totalViewers}`);
  // Generate reports
  // Archive recordings
}

function handleDonationReceived(data) {
  console.log(`Donation received: $${data.amount}`);
  // Send thank you email
  // Update donation totals
}

function handleCondolencePosted(data) {
  console.log(`New condolence from ${data.authorName}`);
  // Notify family members if enabled
}

app.listen(3000, () => {
  console.log('Webhook server listening on port 3000');
});
```

### 11.3 Server-Sent Events Client

```javascript
class ServiceEventStream {
  constructor(serviceId, token) {
    this.serviceId = serviceId;
    this.token = token;
    this.eventSource = null;
  }

  connect() {
    const url = `https://api.funeral-service.example.com/v1/events/stream?serviceId=${this.serviceId}&token=${this.token}`;

    this.eventSource = new EventSource(url, {
      withCredentials: true
    });

    // Generic message handler
    this.eventSource.onmessage = (event) => {
      console.log('Generic event:', event.data);
    };

    // Specific event handlers
    this.eventSource.addEventListener('service_update', (event) => {
      const data = JSON.parse(event.data);
      this.onServiceUpdate(data);
    });

    this.eventSource.addEventListener('viewer_count', (event) => {
      const data = JSON.parse(event.data);
      this.onViewerCount(data);
    });

    this.eventSource.addEventListener('chat_message', (event) => {
      const data = JSON.parse(event.data);
      this.onChatMessage(data);
    });

    // Error handling
    this.eventSource.onerror = (error) => {
      console.error('SSE error:', error);
      if (this.eventSource.readyState === EventSource.CLOSED) {
        console.log('SSE connection closed, reconnecting...');
        setTimeout(() => this.connect(), 5000);
      }
    };
  }

  disconnect() {
    if (this.eventSource) {
      this.eventSource.close();
      this.eventSource = null;
    }
  }

  // Event handlers (to be implemented by user)
  onServiceUpdate(data) {}
  onViewerCount(data) {}
  onChatMessage(data) {}
}

// Usage
const eventStream = new ServiceEventStream(
  '660e8400-e29b-41d4-a716-446655440001',
  'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...'
);

eventStream.onServiceUpdate = (data) => {
  console.log('Service update:', data.status);
};

eventStream.onViewerCount = (data) => {
  document.getElementById('viewer-count').textContent = data.count;
};

eventStream.connect();
```

### 11.4 Live Stream Integration

```javascript
class LiveStreamPlayer {
  constructor(serviceId, containerElement) {
    this.serviceId = serviceId;
    this.container = containerElement;
    this.player = null;
    this.ws = null;
  }

  async initialize() {
    // Get stream URL from API
    const response = await fetch(
      `https://api.funeral-service.example.com/v1/services/${this.serviceId}/stream/status`,
      {
        headers: {
          'Authorization': `Bearer ${token}`
        }
      }
    );
    const { data } = await response.json();

    if (data.status === 'live') {
      this.initializePlayer(data.streamUrl);
      this.connectWebSocket();
    }
  }

  initializePlayer(streamUrl) {
    // Using HLS.js for HLS streams
    if (Hls.isSupported()) {
      this.player = new Hls({
        enableWorker: true,
        lowLatencyMode: true,
        backBufferLength: 90
      });

      this.player.loadSource(streamUrl);
      this.player.attachMedia(this.container);

      this.player.on(Hls.Events.MANIFEST_PARSED, () => {
        this.container.play();
      });

      this.player.on(Hls.Events.ERROR, (event, data) => {
        if (data.fatal) {
          switch (data.type) {
            case Hls.ErrorTypes.NETWORK_ERROR:
              console.error('Network error, trying to recover');
              this.player.startLoad();
              break;
            case Hls.ErrorTypes.MEDIA_ERROR:
              console.error('Media error, trying to recover');
              this.player.recoverMediaError();
              break;
            default:
              console.error('Fatal error, cannot recover');
              this.player.destroy();
              break;
          }
        }
      });
    }
  }

  connectWebSocket() {
    this.ws = new FuneralServiceWebSocket(this.serviceId, token);

    this.ws.onStreamUpdate = (data) => {
      if (data.status === 'ended') {
        this.handleStreamEnded(data);
      }
    };

    this.ws.connect();
  }

  handleStreamEnded(data) {
    console.log('Stream ended');
    if (this.player) {
      this.player.destroy();
    }
    // Show recording if available
    if (data.recordingUrl) {
      this.showRecording(data.recordingUrl);
    }
  }

  showRecording(recordingUrl) {
    // Switch to recorded video
    this.container.src = recordingUrl;
    this.container.controls = true;
  }
}

// Usage
const player = new LiveStreamPlayer(
  '660e8400-e29b-41d4-a716-446655440001',
  document.getElementById('video-player')
);

player.initialize();
```

### 11.5 Chat Interface with Moderation

```javascript
class FuneralChatInterface {
  constructor(serviceId, token) {
    this.serviceId = serviceId;
    this.token = token;
    this.ws = null;
    this.messages = [];
    this.moderationQueue = [];
  }

  initialize() {
    this.ws = new FuneralServiceWebSocket(this.serviceId, this.token);

    this.ws.onChatMessage = (data) => {
      this.addMessage(data);
    };

    this.ws.connect();
  }

  sendMessage(text) {
    if (text.trim().length === 0) return;

    // Client-side filtering
    if (this.containsInappropriateContent(text)) {
      alert('Your message contains inappropriate content');
      return;
    }

    this.ws.sendChatMessage(text);
  }

  addMessage(data) {
    if (data.moderationStatus === 'approved' || !data.moderationStatus) {
      this.messages.push(data);
      this.renderMessage(data);
    } else if (data.moderationStatus === 'pending') {
      this.moderationQueue.push(data);
    }
  }

  renderMessage(data) {
    const chatContainer = document.getElementById('chat-messages');
    const messageElement = document.createElement('div');
    messageElement.className = 'chat-message';
    messageElement.innerHTML = `
      <div class="message-author">${data.author.name} <span class="relationship">(${data.author.relationship})</span></div>
      <div class="message-content">${this.escapeHtml(data.message)}</div>
      <div class="message-time">${new Date(data.timestamp).toLocaleTimeString()}</div>
    `;
    chatContainer.appendChild(messageElement);
    chatContainer.scrollTop = chatContainer.scrollHeight;
  }

  containsInappropriateContent(text) {
    // Simple profanity filter (use more sophisticated solution in production)
    const bannedWords = ['spam', 'inappropriate'];
    return bannedWords.some(word => text.toLowerCase().includes(word));
  }

  escapeHtml(text) {
    const div = document.createElement('div');
    div.textContent = text;
    return div.innerHTML;
  }
}

// Usage
const chat = new FuneralChatInterface(
  '660e8400-e29b-41d4-a716-446655440001',
  'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...'
);

chat.initialize();

document.getElementById('send-button').addEventListener('click', () => {
  const input = document.getElementById('chat-input');
  chat.sendMessage(input.value);
  input.value = '';
});
```

---

## Conclusion

This Phase 3 specification defines comprehensive protocols for real-time communication in digital funeral services. The protocols enable seamless live streaming, interactive chat, event notifications, and data synchronization, creating engaging and meaningful virtual funeral experiences.

**Next Phase:** Phase 4 will define integration patterns and best practices for implementing these protocols in production systems.

---

**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
