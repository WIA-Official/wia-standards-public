# WIA AI Interoperability Standard
## Phase 3: Protocol Specification

**Version**: 1.0
**Status**: Draft
**Last Updated**: 2025-12-25

---

## Philosophy

**弘益人間 (홍익인간)** - *Benefit All Humanity*

Real-time communication enables AI systems to collaborate instantly, creating seamless experiences that benefit users worldwide. This protocol breaks down barriers between AI platforms, fostering a connected ecosystem where AI serves humanity more effectively.

---

## Overview

Phase 3 defines the real-time communication protocol for AI interoperability. This protocol enables:
- Bidirectional communication between AI systems
- Real-time message streaming
- Multi-party AI collaboration
- Event-driven architectures
- Peer-to-peer AI connections
- Federation across AI networks

### Protocol Layers

1. **Transport Layer**: WebSocket, WebRTC
2. **Message Layer**: Binary and JSON encoding
3. **Session Layer**: Connection management
4. **Application Layer**: AI-specific operations

---

## Schema

### WebSocket Protocol

#### Connection URL

```
wss://realtime.wia-ai-interop.org/v1/connect
```

#### Connection Headers

```
Authorization: Bearer wia_sk_abc123
WIA-Protocol-Version: 1.0
WIA-Client-ID: client_xyz789
```

#### Message Frame Format

All WebSocket messages use this envelope:

```json
{
  "type": "message" | "event" | "control",
  "id": "frame_001",
  "timestamp": "2025-12-25T10:30:00Z",
  "payload": { /* type-specific payload */ }
}
```

---

## Fields

### Frame Types

| Type | Description | Direction |
|------|-------------|-----------|
| `message` | AI message exchange | Bidirectional |
| `event` | System events | Server → Client |
| `control` | Protocol control | Bidirectional |

### Message Frame

```json
{
  "type": "message",
  "id": "frame_001",
  "timestamp": "2025-12-25T10:30:00Z",
  "payload": {
    "message": {
      /* AIMessage from Phase 1 */
    },
    "deliveryMode": "immediate" | "queued",
    "priority": "high" | "normal" | "low"
  }
}
```

### Event Frame

```json
{
  "type": "event",
  "id": "frame_002",
  "timestamp": "2025-12-25T10:30:01Z",
  "payload": {
    "eventType": "typing" | "thinking" | "tool_use" | "error" | "connection",
    "data": {
      /* event-specific data */
    }
  }
}
```

### Control Frame

```json
{
  "type": "control",
  "id": "frame_003",
  "timestamp": "2025-12-25T10:30:02Z",
  "payload": {
    "command": "ping" | "pong" | "subscribe" | "unsubscribe" | "close",
    "parameters": {}
  }
}
```

---

## Connection Lifecycle

### 1. Handshake

**Client → Server**:
```json
{
  "type": "control",
  "id": "handshake_001",
  "timestamp": "2025-12-25T10:30:00Z",
  "payload": {
    "command": "handshake",
    "parameters": {
      "protocolVersion": "1.0",
      "clientId": "client_xyz",
      "capabilities": ["streaming", "multimodal", "federation"]
    }
  }
}
```

**Server → Client**:
```json
{
  "type": "control",
  "id": "handshake_002",
  "timestamp": "2025-12-25T10:30:00Z",
  "payload": {
    "command": "handshake_ack",
    "parameters": {
      "sessionId": "session_abc123",
      "serverCapabilities": ["streaming", "multimodal", "persistence"],
      "heartbeatInterval": 30000
    }
  }
}
```

### 2. Subscription

Subscribe to specific topics or conversations:

```json
{
  "type": "control",
  "id": "sub_001",
  "timestamp": "2025-12-25T10:30:01Z",
  "payload": {
    "command": "subscribe",
    "parameters": {
      "topics": ["conversation:conv_001", "model:claude-sonnet-4-5"],
      "filters": {
        "messageTypes": ["text", "code"]
      }
    }
  }
}
```

### 3. Heartbeat

Keep connection alive:

**Client → Server** (every 30 seconds):
```json
{
  "type": "control",
  "id": "ping_001",
  "timestamp": "2025-12-25T10:30:30Z",
  "payload": {
    "command": "ping"
  }
}
```

**Server → Client**:
```json
{
  "type": "control",
  "id": "pong_001",
  "timestamp": "2025-12-25T10:30:30Z",
  "payload": {
    "command": "pong",
    "parameters": {
      "serverTime": "2025-12-25T10:30:30Z"
    }
  }
}
```

### 4. Graceful Shutdown

```json
{
  "type": "control",
  "id": "close_001",
  "timestamp": "2025-12-25T11:00:00Z",
  "payload": {
    "command": "close",
    "parameters": {
      "reason": "client_disconnect",
      "message": "User logged out"
    }
  }
}
```

---

## Event Types

### Typing Event

Indicates AI is preparing a response:

```json
{
  "type": "event",
  "id": "event_001",
  "timestamp": "2025-12-25T10:30:01Z",
  "payload": {
    "eventType": "typing",
    "data": {
      "conversationId": "conv_001",
      "senderId": "claude_assistant",
      "active": true
    }
  }
}
```

### Thinking Event

AI model is processing (extended thinking):

```json
{
  "type": "event",
  "id": "event_002",
  "timestamp": "2025-12-25T10:30:02Z",
  "payload": {
    "eventType": "thinking",
    "data": {
      "conversationId": "conv_001",
      "modelId": "claude-sonnet-4-5",
      "stage": "analyzing" | "reasoning" | "generating",
      "progress": 0.45
    }
  }
}
```

### Tool Use Event

AI is using a tool:

```json
{
  "type": "event",
  "id": "event_003",
  "timestamp": "2025-12-25T10:30:03Z",
  "payload": {
    "eventType": "tool_use",
    "data": {
      "toolName": "web_search",
      "toolId": "tool_001",
      "status": "started" | "in_progress" | "completed" | "failed"
    }
  }
}
```

### Error Event

Error occurred during processing:

```json
{
  "type": "event",
  "id": "event_004",
  "timestamp": "2025-12-25T10:30:04Z",
  "payload": {
    "eventType": "error",
    "data": {
      "code": "RATE_LIMIT_EXCEEDED",
      "message": "Too many requests",
      "severity": "warning" | "error" | "fatal",
      "recoverable": true
    }
  }
}
```

### Connection Event

Connection status changes:

```json
{
  "type": "event",
  "id": "event_005",
  "timestamp": "2025-12-25T10:30:05Z",
  "payload": {
    "eventType": "connection",
    "data": {
      "status": "connected" | "disconnected" | "reconnecting",
      "reason": "network_error",
      "reconnectIn": 5000
    }
  }
}
```

---

## Validation

### Connection Requirements

1. **TLS 1.3+**: All connections must use secure WebSocket (wss://)
2. **Authentication**: Valid API key or OAuth token required
3. **Protocol Version**: Must specify compatible protocol version
4. **Heartbeat**: Clients must respond to pings within 10 seconds

### Message Validation

1. **Frame ID**: Must be unique within session
2. **Timestamp**: Must be ISO 8601 format
3. **Payload**: Must conform to type-specific schema
4. **Size Limit**: Max 10MB per frame
5. **Rate Limit**: Max 100 messages/second per connection

### Error Handling

| Error Code | Description | Action |
|------------|-------------|--------|
| 1000 | Normal closure | - |
| 1001 | Going away | Reconnect |
| 1002 | Protocol error | Check protocol version |
| 1003 | Unsupported data | Check message format |
| 1006 | Abnormal closure | Reconnect with backoff |
| 1008 | Policy violation | Check rate limits |
| 1011 | Internal error | Retry with exponential backoff |

---

## Examples

### Example 1: Real-time Chat Session

**1. Connect and Handshake**:
```javascript
const ws = new WebSocket('wss://realtime.wia-ai-interop.org/v1/connect', {
  headers: {
    'Authorization': 'Bearer wia_sk_abc123',
    'WIA-Protocol-Version': '1.0'
  }
});

ws.onopen = () => {
  ws.send(JSON.stringify({
    type: 'control',
    id: 'handshake_001',
    timestamp: new Date().toISOString(),
    payload: {
      command: 'handshake',
      parameters: {
        protocolVersion: '1.0',
        clientId: 'web_client_001',
        capabilities: ['streaming', 'multimodal']
      }
    }
  }));
};
```

**2. Subscribe to Conversation**:
```javascript
ws.send(JSON.stringify({
  type: 'control',
  id: 'sub_001',
  timestamp: new Date().toISOString(),
  payload: {
    command: 'subscribe',
    parameters: {
      topics: ['conversation:conv_001']
    }
  }
}));
```

**3. Send Message**:
```javascript
ws.send(JSON.stringify({
  type: 'message',
  id: 'msg_frame_001',
  timestamp: new Date().toISOString(),
  payload: {
    message: {
      version: '1.0',
      messageId: 'msg_001',
      timestamp: new Date().toISOString(),
      sender: {
        type: 'human',
        id: 'user_alice'
      },
      content: {
        type: 'text',
        parts: [{
          type: 'text',
          data: 'Hello, AI! How are you?'
        }]
      },
      context: {
        conversationId: 'conv_001'
      }
    }
  }
}));
```

**4. Receive Events and Response**:
```javascript
ws.onmessage = (event) => {
  const frame = JSON.parse(event.data);

  switch (frame.type) {
    case 'event':
      if (frame.payload.eventType === 'typing') {
        console.log('AI is typing...');
      }
      break;

    case 'message':
      console.log('AI response:', frame.payload.message.content.parts[0].data);
      break;

    case 'control':
      if (frame.payload.command === 'ping') {
        // Respond to heartbeat
        ws.send(JSON.stringify({
          type: 'control',
          id: 'pong_' + Date.now(),
          timestamp: new Date().toISOString(),
          payload: { command: 'pong' }
        }));
      }
      break;
  }
};
```

### Example 2: Multi-AI Collaboration

Multiple AI systems working together on a task:

```json
{
  "type": "message",
  "id": "collab_001",
  "timestamp": "2025-12-25T10:30:00Z",
  "payload": {
    "message": {
      "version": "1.0",
      "messageId": "msg_collab_001",
      "timestamp": "2025-12-25T10:30:00Z",
      "sender": {
        "type": "ai",
        "id": "ai_coordinator"
      },
      "content": {
        "type": "text",
        "parts": [{
          "type": "text",
          "data": "I need help analyzing this dataset. @ai_data_analyst and @ai_visualizer, can you collaborate?"
        }]
      },
      "context": {
        "conversationId": "conv_collab_001",
        "metadata": {
          "collaboration": true,
          "participants": ["ai_coordinator", "ai_data_analyst", "ai_visualizer"]
        }
      }
    }
  }
}
```

### Example 3: Federated AI Network

AI systems across different organizations communicating:

```json
{
  "type": "message",
  "id": "federation_001",
  "timestamp": "2025-12-25T10:30:00Z",
  "payload": {
    "message": {
      "version": "1.0",
      "messageId": "msg_fed_001",
      "timestamp": "2025-12-25T10:30:00Z",
      "sender": {
        "type": "ai",
        "id": "ai_medical@hospital-a.org",
        "metadata": {
          "organization": "Hospital A",
          "domain": "medical"
        }
      },
      "content": {
        "type": "text",
        "parts": [{
          "type": "text",
          "data": "Query: Similar cases for diagnosis XYZ"
        }]
      },
      "context": {
        "conversationId": "conv_fed_medical_001",
        "metadata": {
          "federation": "medical_ai_network",
          "privacy": "HIPAA_compliant",
          "encryption": "end_to_end"
        }
      }
    },
    "deliveryMode": "federated",
    "routing": {
      "targetDomains": ["medical"],
      "excludeOrganizations": ["Hospital A"]
    }
  }
}
```

---

## Advanced Features

### Binary Message Encoding

For efficiency, messages can be sent in binary format using MessagePack or Protocol Buffers:

```javascript
// Example with MessagePack
const msgpack = require('msgpack-lite');
const binaryData = msgpack.encode({
  type: 'message',
  payload: { /* message data */ }
});
ws.send(binaryData);
```

### WebRTC Peer-to-Peer

For direct AI-to-AI communication without central server:

```javascript
// Establish WebRTC data channel
const peerConnection = new RTCPeerConnection(config);
const dataChannel = peerConnection.createDataChannel('ai-interop');

dataChannel.onopen = () => {
  dataChannel.send(JSON.stringify({
    type: 'message',
    payload: { /* AI message */ }
  }));
};
```

### Priority Queuing

High-priority messages bypass normal queues:

```json
{
  "type": "message",
  "id": "urgent_001",
  "timestamp": "2025-12-25T10:30:00Z",
  "payload": {
    "message": { /* emergency alert */ },
    "priority": "high",
    "deliveryMode": "immediate"
  }
}
```

---

## Implementation Notes

1. **Reconnection**: Implement exponential backoff (1s, 2s, 4s, 8s, max 60s)
2. **Message Queue**: Buffer messages during disconnection
3. **Deduplication**: Use message IDs to prevent duplicate processing
4. **Compression**: Use WebSocket permessage-deflate extension
5. **Monitoring**: Track connection health, latency, message rates

---

## Security Considerations

1. **Encryption**: TLS 1.3 for transport, optional E2E for payload
2. **Authentication**: Token refresh every 24 hours
3. **Authorization**: Validate permissions for each message
4. **Rate Limiting**: Per-connection and per-user limits
5. **DDoS Protection**: Connection limits, message validation
6. **Privacy**: No logging of message content (opt-in only)

---

## Next Steps

Phase 3 establishes real-time protocol. Continue to:
- **Phase 4**: Integration patterns with existing AI platforms

---

© 2025 SmileStory Inc. / WIA
**弘益人間 (홍익인간)** · Benefit All Humanity
