# WIA-CORE-007: Universal Protocol Specification v1.0

> **Standard ID:** WIA-CORE-007
> **Version:** 1.0.0
> **Published:** 2025-12-27
> **Status:** Active
> **Authors:** WIA Core Standards Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Message Format](#2-message-format)
3. [Protocol Layers](#3-protocol-layers)
4. [Transport Mechanisms](#4-transport-mechanisms)
5. [RPC (Remote Procedure Call)](#5-rpc-remote-procedure-call)
6. [Event Streaming](#6-event-streaming)
7. [Protocol Negotiation](#7-protocol-negotiation)
8. [Security](#8-security)
9. [Error Handling](#9-error-handling)
10. [Implementation Guidelines](#10-implementation-guidelines)

---

## 1. Introduction

### 1.1 Purpose

This specification defines a universal communication protocol that enables seamless message exchange and remote procedure calls across heterogeneous systems, platforms, and programming languages.

### 1.2 Scope

The standard covers:
- Message format and structure
- Protocol layers and abstractions
- Transport mechanisms (HTTP, WebSocket, TCP, UDP)
- RPC patterns and conventions
- Event streaming and pub/sub
- Security and authentication
- Error handling and recovery

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This protocol aims to create a universal communication standard that transcends technological boundaries, enabling all systems to communicate effortlessly and fostering global collaboration.

### 1.4 Terminology

- **Message**: A unit of communication containing headers, payload, and metadata
- **RPC**: Remote Procedure Call - invoking a method on a remote system
- **Transport**: The underlying mechanism for message delivery (HTTP, WebSocket, etc.)
- **Method**: A callable function or operation on a remote system
- **Payload**: The data content of a message
- **Stream**: A sequence of related messages forming a continuous data flow

---

## 2. Message Format

### 2.1 Base Message Structure

Every message in the Universal Protocol follows this structure:

```json
{
  "id": "string",
  "version": "string",
  "type": "request|response|event|stream",
  "method": "string",
  "headers": {
    "key": "value"
  },
  "payload": any,
  "metadata": {
    "timestamp": "ISO8601",
    "source": "string",
    "destination": "string",
    "correlation_id": "string",
    "ttl": number
  },
  "status": "success|error|pending",
  "error": {
    "code": "string",
    "message": "string",
    "details": any
  }
}
```

### 2.2 Message Fields

#### 2.2.1 Required Fields

| Field | Type | Description |
|-------|------|-------------|
| `id` | string | Unique message identifier (UUID v4 recommended) |
| `version` | string | Protocol version (semver format: "1.0.0") |
| `type` | string | Message type: request, response, event, or stream |

#### 2.2.2 Optional Fields

| Field | Type | Description |
|-------|------|-------------|
| `method` | string | Method name in dot notation (e.g., "user.getProfile") |
| `headers` | object | Key-value pairs for metadata and routing |
| `payload` | any | Message data content |
| `metadata` | object | System-generated metadata |
| `status` | string | Response status: success, error, or pending |
| `error` | object | Error information (required if status is "error") |

### 2.3 Message Types

#### 2.3.1 Request Message

```json
{
  "id": "req_1234567890abcdef",
  "version": "1.0.0",
  "type": "request",
  "method": "user.getProfile",
  "headers": {
    "authorization": "Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
    "content-type": "application/json",
    "accept": "application/json"
  },
  "payload": {
    "userId": "12345"
  },
  "metadata": {
    "timestamp": "2025-12-27T00:00:00Z",
    "source": "client-app-001",
    "correlation_id": "trace_abc123",
    "ttl": 30000
  }
}
```

#### 2.3.2 Response Message

```json
{
  "id": "req_1234567890abcdef",
  "version": "1.0.0",
  "type": "response",
  "method": "user.getProfile",
  "headers": {
    "content-type": "application/json"
  },
  "payload": {
    "user": {
      "id": "12345",
      "name": "John Doe",
      "email": "john@example.com"
    }
  },
  "metadata": {
    "timestamp": "2025-12-27T00:00:01Z",
    "source": "server-node-05",
    "processing_time": 45
  },
  "status": "success"
}
```

#### 2.3.3 Event Message

```json
{
  "id": "evt_9876543210fedcba",
  "version": "1.0.0",
  "type": "event",
  "method": "user.profileUpdated",
  "headers": {
    "event-source": "user-service"
  },
  "payload": {
    "userId": "12345",
    "changes": {
      "name": "John Smith"
    }
  },
  "metadata": {
    "timestamp": "2025-12-27T00:05:00Z",
    "source": "user-service-002"
  }
}
```

#### 2.3.4 Stream Message

```json
{
  "id": "stream_abc123def456",
  "version": "1.0.0",
  "type": "stream",
  "method": "data.stream",
  "headers": {
    "stream-id": "data_stream_001"
  },
  "payload": {
    "chunk": "base64_encoded_data...",
    "size": 1024
  },
  "metadata": {
    "timestamp": "2025-12-27T00:10:00Z",
    "sequence": 42,
    "total": 100,
    "final": false
  }
}
```

### 2.4 Message Serialization

#### 2.4.1 JSON (Default)

- **Encoding**: UTF-8
- **Format**: Compact (no whitespace)
- **Numbers**: IEEE 754 double precision
- **Dates**: ISO 8601 format

#### 2.4.2 MessagePack (Binary)

- Compact binary serialization
- ~30% smaller than JSON
- Recommended for high-performance scenarios

#### 2.4.3 Protocol Buffers

- Strongly typed binary format
- Requires .proto schema definition
- Recommended for gRPC transport

---

## 3. Protocol Layers

### 3.1 Layer Architecture

```
┌─────────────────────────────────────┐
│     Application Layer               │  ← Your Application Logic
├─────────────────────────────────────┤
│     Protocol Layer (WIA-CORE-007)   │  ← Message Format & RPC
├─────────────────────────────────────┤
│     Transport Layer                 │  ← HTTP, WebSocket, TCP, UDP
├─────────────────────────────────────┤
│     Network Layer                   │  ← IP, Routing
└─────────────────────────────────────┘
```

### 3.2 Protocol Layer Responsibilities

1. **Message Construction**: Creating and parsing messages
2. **Method Routing**: Directing requests to appropriate handlers
3. **Serialization**: Converting messages to/from wire format
4. **Validation**: Ensuring message integrity and schema compliance
5. **Middleware**: Intercepting and processing messages
6. **Error Handling**: Managing errors and timeouts

### 3.3 Transport Layer Abstraction

The protocol is transport-agnostic and can work over:

- **HTTP/HTTPS**: Request-response pattern
- **WebSocket**: Bidirectional real-time communication
- **TCP**: Reliable stream-based communication
- **UDP**: Connectionless datagram communication
- **Custom**: Any transport implementing the transport interface

---

## 4. Transport Mechanisms

### 4.1 HTTP/HTTPS Transport

#### 4.1.1 Request Mapping

```
POST /rpc HTTP/1.1
Host: api.example.com
Content-Type: application/json
Authorization: Bearer token123

{
  "id": "req_001",
  "version": "1.0.0",
  "type": "request",
  "method": "user.getProfile",
  "payload": { "userId": "12345" }
}
```

#### 4.1.2 Response Mapping

```
HTTP/1.1 200 OK
Content-Type: application/json

{
  "id": "req_001",
  "version": "1.0.0",
  "type": "response",
  "status": "success",
  "payload": { "user": { ... } }
}
```

#### 4.1.3 HTTP Status Codes

| HTTP Code | Protocol Status | Description |
|-----------|----------------|-------------|
| 200 | success | Request succeeded |
| 400 | error | Invalid request format |
| 401 | error | Authentication required |
| 403 | error | Permission denied |
| 404 | error | Method not found |
| 429 | error | Rate limit exceeded |
| 500 | error | Server error |
| 503 | error | Service unavailable |

### 4.2 WebSocket Transport

#### 4.2.1 Connection Handshake

```
GET /ws HTTP/1.1
Host: api.example.com
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==
Sec-WebSocket-Protocol: wia-core-007-v1
Sec-WebSocket-Version: 13
```

#### 4.2.2 Message Exchange

```javascript
// Client sends request
ws.send(JSON.stringify({
  id: "req_001",
  type: "request",
  method: "user.getProfile",
  payload: { userId: "12345" }
}));

// Server sends response
ws.send(JSON.stringify({
  id: "req_001",
  type: "response",
  status: "success",
  payload: { user: { ... } }
}));

// Server sends event
ws.send(JSON.stringify({
  id: "evt_001",
  type: "event",
  method: "notification.received",
  payload: { message: "Hello" }
}));
```

#### 4.2.3 Heartbeat

To maintain connection health:

```json
{
  "id": "ping_001",
  "type": "request",
  "method": "system.ping"
}

{
  "id": "ping_001",
  "type": "response",
  "status": "success",
  "payload": { "pong": true }
}
```

### 4.3 TCP Transport

#### 4.3.1 Frame Format

```
┌──────────────┬──────────────┬────────────────────┐
│ Length (4B)  │ Message ID   │ JSON/Binary Data   │
│ Big Endian   │ (16B UUID)   │ (Length bytes)     │
└──────────────┴──────────────┴────────────────────┘
```

#### 4.3.2 Connection Lifecycle

1. **Connect**: TCP 3-way handshake
2. **Authenticate**: Send authentication message
3. **Communicate**: Exchange messages
4. **Disconnect**: Graceful shutdown with FIN

### 4.4 UDP Transport

#### 4.4.1 Datagram Format

```
┌──────────────┬──────────────┬────────────────────┐
│ Sequence (4B)│ Message ID   │ JSON/Binary Data   │
│ Big Endian   │ (16B UUID)   │ (up to 65507 bytes)│
└──────────────┴──────────────┴────────────────────┘
```

#### 4.4.2 Reliability

- **No ACK**: Fire-and-forget for high-performance scenarios
- **Optional ACK**: Application-level acknowledgment
- **Checksum**: UDP checksum for integrity

---

## 5. RPC (Remote Procedure Call)

### 5.1 RPC Pattern

#### 5.1.1 Synchronous RPC

```typescript
// Client
const response = await rpc.call('math.add', { a: 5, b: 3 });
console.log(response.payload.result); // 8

// Server
rpc.register('math.add', async (payload) => {
  return { result: payload.a + payload.b };
});
```

#### 5.1.2 Asynchronous RPC

```typescript
// Client
rpc.call('job.process', { data: '...' }, (response) => {
  console.log('Job completed:', response.payload);
});

// Server
rpc.register('job.process', async (payload) => {
  // Long-running operation
  await processJob(payload.data);
  return { status: 'completed' };
});
```

### 5.2 Method Naming Convention

Methods follow a hierarchical dot notation:

```
<service>.<resource>.<action>

Examples:
- user.profile.get
- user.profile.update
- order.item.create
- payment.transaction.process
```

### 5.3 Parameter Passing

#### 5.3.1 Named Parameters (Recommended)

```json
{
  "method": "user.create",
  "payload": {
    "name": "John Doe",
    "email": "john@example.com",
    "age": 30
  }
}
```

#### 5.3.2 Positional Parameters

```json
{
  "method": "math.add",
  "payload": {
    "params": [5, 3]
  }
}
```

### 5.4 Return Values

#### 5.4.1 Single Value

```json
{
  "type": "response",
  "status": "success",
  "payload": {
    "result": 8
  }
}
```

#### 5.4.2 Multiple Values

```json
{
  "type": "response",
  "status": "success",
  "payload": {
    "sum": 8,
    "product": 15,
    "difference": 2
  }
}
```

#### 5.4.3 Complex Objects

```json
{
  "type": "response",
  "status": "success",
  "payload": {
    "user": {
      "id": "12345",
      "profile": { ... },
      "settings": { ... }
    }
  }
}
```

### 5.5 Timeout Handling

- **Default Timeout**: 30 seconds
- **Custom Timeout**: Set via `ttl` in metadata (milliseconds)
- **Timeout Response**: Error with code `ERR_TIMEOUT`

```json
{
  "type": "response",
  "status": "error",
  "error": {
    "code": "ERR_TIMEOUT",
    "message": "Request timed out after 30000ms"
  }
}
```

---

## 6. Event Streaming

### 6.1 Publish-Subscribe Pattern

#### 6.1.1 Subscribe to Channel

```json
{
  "type": "request",
  "method": "events.subscribe",
  "payload": {
    "channels": ["notifications", "updates"]
  }
}
```

#### 6.1.2 Publish Event

```json
{
  "type": "event",
  "method": "notification.received",
  "payload": {
    "message": "New message from Alice",
    "priority": "high"
  }
}
```

#### 6.1.3 Unsubscribe

```json
{
  "type": "request",
  "method": "events.unsubscribe",
  "payload": {
    "channels": ["notifications"]
  }
}
```

### 6.2 Stream Processing

#### 6.2.1 Initiate Stream

```json
{
  "type": "request",
  "method": "data.streamFile",
  "payload": {
    "fileId": "file_12345",
    "chunkSize": 1024
  }
}
```

#### 6.2.2 Stream Chunks

```json
{
  "type": "stream",
  "method": "data.chunk",
  "payload": {
    "data": "base64_encoded_chunk..."
  },
  "metadata": {
    "sequence": 1,
    "total": 100,
    "final": false
  }
}
```

#### 6.2.3 Stream Completion

```json
{
  "type": "stream",
  "method": "data.chunk",
  "payload": {
    "data": "base64_encoded_chunk..."
  },
  "metadata": {
    "sequence": 100,
    "total": 100,
    "final": true
  }
}
```

### 6.3 Bidirectional Streaming

```typescript
// Client sends stream
for (let i = 0; i < 100; i++) {
  await stream.send({ sequence: i, data: chunk });
}
await stream.end();

// Server processes stream
stream.on('data', (chunk) => {
  processChunk(chunk);
});

stream.on('end', () => {
  sendFinalResult();
});
```

---

## 7. Protocol Negotiation

### 7.1 Version Negotiation

#### 7.1.1 Client Request

```json
{
  "type": "request",
  "method": "protocol.negotiate",
  "payload": {
    "versions": ["1.0.0", "1.1.0"],
    "features": ["compression", "encryption", "streaming"]
  }
}
```

#### 7.1.2 Server Response

```json
{
  "type": "response",
  "status": "success",
  "payload": {
    "version": "1.0.0",
    "features": ["compression", "streaming"]
  }
}
```

### 7.2 Capability Discovery

```json
{
  "type": "request",
  "method": "protocol.capabilities"
}

{
  "type": "response",
  "status": "success",
  "payload": {
    "transports": ["http", "websocket", "tcp"],
    "serialization": ["json", "messagepack"],
    "compression": ["gzip", "brotli"],
    "encryption": ["tls", "aes-256-gcm"],
    "max_message_size": 10485760,
    "streaming": true,
    "bidirectional": true
  }
}
```

### 7.3 Feature Flags

```json
{
  "headers": {
    "x-protocol-features": "compression,encryption"
  }
}
```

---

## 8. Security

### 8.1 Authentication

#### 8.1.1 Bearer Token

```json
{
  "headers": {
    "authorization": "Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9..."
  }
}
```

#### 8.1.2 API Key

```json
{
  "headers": {
    "x-api-key": "sk_REPLACE_WITH_YOUR_API_KEY_PLACEHOLDER"
  }
}
```

#### 8.1.3 OAuth 2.0

```json
{
  "headers": {
    "authorization": "Bearer oauth_token_here"
  }
}
```

#### 8.1.4 Mutual TLS (mTLS)

Client and server present certificates during TLS handshake.

### 8.2 Message Signing

#### 8.2.1 HMAC Signature

```json
{
  "headers": {
    "x-signature": "sha256=abc123...",
    "x-signature-timestamp": "2025-12-27T00:00:00Z"
  }
}
```

Signature calculation:
```
signature = HMAC-SHA256(secret, timestamp + message_body)
```

#### 8.2.2 Digital Signature (RSA/ECDSA)

```json
{
  "headers": {
    "x-signature": "base64_encoded_signature",
    "x-signature-algorithm": "RS256",
    "x-signature-keyid": "key_12345"
  }
}
```

### 8.3 Encryption

#### 8.3.1 Transport Layer (TLS)

- TLS 1.2 minimum (TLS 1.3 recommended)
- Strong cipher suites only
- Certificate pinning optional

#### 8.3.2 Message Layer (E2E)

```json
{
  "payload": {
    "encrypted": true,
    "algorithm": "AES-256-GCM",
    "ciphertext": "base64_encrypted_data",
    "iv": "base64_iv",
    "tag": "base64_auth_tag"
  }
}
```

### 8.4 Rate Limiting

```json
{
  "type": "response",
  "status": "error",
  "error": {
    "code": "ERR_RATE_LIMIT",
    "message": "Rate limit exceeded"
  },
  "headers": {
    "x-ratelimit-limit": "1000",
    "x-ratelimit-remaining": "0",
    "x-ratelimit-reset": "1735257600"
  }
}
```

---

## 9. Error Handling

### 9.1 Error Response Format

```json
{
  "type": "response",
  "status": "error",
  "error": {
    "code": "ERR_CODE",
    "message": "Human-readable error message",
    "details": {
      "field": "userId",
      "reason": "User not found"
    },
    "trace_id": "trace_abc123"
  }
}
```

### 9.2 Standard Error Codes

| Code | HTTP | Description |
|------|------|-------------|
| ERR_INVALID_REQUEST | 400 | Malformed request |
| ERR_UNAUTHORIZED | 401 | Authentication required |
| ERR_FORBIDDEN | 403 | Permission denied |
| ERR_NOT_FOUND | 404 | Method or resource not found |
| ERR_METHOD_NOT_ALLOWED | 405 | Method not supported |
| ERR_TIMEOUT | 408 | Request timeout |
| ERR_RATE_LIMIT | 429 | Rate limit exceeded |
| ERR_INTERNAL | 500 | Internal server error |
| ERR_NOT_IMPLEMENTED | 501 | Method not implemented |
| ERR_SERVICE_UNAVAILABLE | 503 | Service unavailable |
| ERR_GATEWAY_TIMEOUT | 504 | Gateway timeout |

### 9.3 Error Recovery

#### 9.3.1 Retry Policy

```typescript
const retryPolicy = {
  maxAttempts: 3,
  backoff: 'exponential', // linear, exponential, constant
  initialDelay: 1000, // ms
  maxDelay: 30000, // ms
  retryableErrors: ['ERR_TIMEOUT', 'ERR_SERVICE_UNAVAILABLE']
};
```

#### 9.3.2 Circuit Breaker

```typescript
const circuitBreaker = {
  failureThreshold: 5,
  timeout: 60000, // ms
  resetTimeout: 30000 // ms
};
```

---

## 10. Implementation Guidelines

### 10.1 Required Components

Any WIA-CORE-007 compliant implementation must include:

1. **Message Parser**: Parse and validate messages
2. **Serializer**: Convert messages to/from wire format
3. **Transport Adapter**: Interface for transport mechanisms
4. **RPC Handler**: Execute remote procedure calls
5. **Event Manager**: Publish-subscribe event handling
6. **Error Handler**: Manage errors and retries

### 10.2 API Interface

#### 10.2.1 Client Interface

```typescript
interface UniversalProtocolClient {
  // Connect to server
  connect(endpoint: string, options?: ConnectionOptions): Promise<void>;

  // Disconnect from server
  disconnect(): Promise<void>;

  // Send RPC request
  call(method: string, payload: any, options?: CallOptions): Promise<Response>;

  // Subscribe to events
  subscribe(channel: string, handler: EventHandler): Subscription;

  // Unsubscribe from events
  unsubscribe(subscription: Subscription): void;

  // Send custom message
  send(message: Message): Promise<void>;

  // Create stream
  stream(method: string, payload: any): Stream;
}
```

#### 10.2.2 Server Interface

```typescript
interface UniversalProtocolServer {
  // Start server
  start(port: number, options?: ServerOptions): Promise<void>;

  // Stop server
  stop(): Promise<void>;

  // Register RPC method
  register(method: string, handler: MethodHandler): void;

  // Unregister RPC method
  unregister(method: string): void;

  // Publish event
  publish(channel: string, event: Event): void;

  // Add middleware
  use(middleware: Middleware): void;
}
```

### 10.3 Data Formats

#### 10.3.1 Message ID Format

```
UUID v4: xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx
Or prefixed: req_xxxxxxxxxxxxxxxx, evt_xxxxxxxxxxxxxxxx
```

#### 10.3.2 Timestamp Format

```
ISO 8601: 2025-12-27T00:00:00Z
Unix epoch (milliseconds): 1735257600000
```

#### 10.3.3 Method Name Format

```
<service>.<resource>.<action>
Pattern: ^[a-z][a-z0-9]*(\.[a-z][a-z0-9]*)*$
Examples: user.profile.get, order.create, system.ping
```

### 10.4 Performance Optimization

1. **Connection Pooling**: Reuse connections
2. **Message Batching**: Combine multiple messages
3. **Compression**: Enable gzip/brotli for large payloads
4. **Caching**: Cache responses when appropriate
5. **Async Processing**: Non-blocking I/O
6. **Load Balancing**: Distribute load across servers

### 10.5 Testing Requirements

1. **Unit Tests**: Test individual components
2. **Integration Tests**: Test end-to-end flows
3. **Performance Tests**: Measure latency and throughput
4. **Security Tests**: Verify authentication and encryption
5. **Compatibility Tests**: Test with different transports
6. **Stress Tests**: Test under high load

---

## Appendix A: Complete Examples

### A.1 Simple RPC Call

```typescript
// Client
const client = new UniversalProtocol({
  transport: 'http',
  endpoint: 'https://api.example.com'
});

await client.connect();

const response = await client.call('user.getProfile', {
  userId: '12345'
});

console.log(response.payload);

await client.disconnect();
```

### A.2 Event Subscription

```typescript
// Client
const client = new UniversalProtocol({
  transport: 'websocket',
  endpoint: 'wss://api.example.com'
});

await client.connect();

const subscription = client.subscribe('notifications', (event) => {
  console.log('Notification:', event.payload);
});

// Later: unsubscribe
client.unsubscribe(subscription);
```

### A.3 Bidirectional Streaming

```typescript
// Client
const stream = client.stream('data.process', { input: 'data' });

stream.on('data', (chunk) => {
  console.log('Received chunk:', chunk);
});

stream.on('end', () => {
  console.log('Stream completed');
});

for (let i = 0; i < 100; i++) {
  await stream.send({ chunk: i });
}

await stream.end();
```

---

## Appendix B: Wire Format Examples

### B.1 JSON Over HTTP

```
POST /rpc HTTP/1.1
Host: api.example.com
Content-Type: application/json
Content-Length: 234

{"id":"req_001","version":"1.0.0","type":"request","method":"user.getProfile","payload":{"userId":"12345"}}
```

### B.2 MessagePack Over WebSocket

```
Binary frame (hex):
82 a2 69 64 aa 72 65 71 5f 30 30 31 a7 76 65 72 73 69 6f 6e ...
```

### B.3 Protocol Buffers Over TCP

```
Frame header (12 bytes):
00 00 00 AC  (length: 172 bytes)
01 23 45 67 89 AB CD EF  (message ID)

Protobuf message (172 bytes):
08 01 12 0A ...
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-CORE-007 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
