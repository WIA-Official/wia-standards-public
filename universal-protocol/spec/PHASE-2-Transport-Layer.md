# WIA-CORE-007 PHASE 2: Transport Layer Specification

**Version:** 1.0
**Status:** APPROVED
**Last Updated:** 2025-12-27
**Authors:** WIA Standards Committee

## 1. Introduction

PHASE 2 defines the transport layer specifications for WIA-CORE-007 Universal Protocol. This document establishes requirements for transport implementations, transport-agnostic design, and interoperability between different transport mechanisms.

## 2. Transport Abstraction

### 2.1 Transport Interface

All transport implementations MUST implement the following interface:

```typescript
interface Transport {
  // Connection management
  connect(): Promise<void>;
  disconnect(): Promise<void>;
  isConnected(): boolean;

  // Message transmission
  send(message: Message): Promise<void>;
  receive(): AsyncIterableIterator<Message>;

  // Event handling
  on(event: TransportEvent, handler: EventHandler): void;
  off(event: TransportEvent, handler: EventHandler): void;

  // Configuration
  configure(options: TransportOptions): void;
  getCapabilities(): TransportCapabilities;
}

enum TransportEvent {
  CONNECTED = 'connected',
  DISCONNECTED = 'disconnected',
  ERROR = 'error',
  MESSAGE = 'message'
}
```

### 2.2 Transport Capabilities

```typescript
interface TransportCapabilities {
  maxMessageSize?: number;        // Maximum message size in bytes
  supportsBidirectional: boolean; // Supports bidirectional streaming
  supportsMultiplexing: boolean;  // Supports multiple concurrent streams
  compression: string[];          // Supported compression algorithms
  encryption: string[];           // Supported encryption methods
  qosLevels: QoSLevel[];         // Supported QoS levels
}
```

## 3. HTTP Transport

### 3.1 HTTP/REST Implementation

**Endpoint Structure:**
```
POST /rpc              - RPC calls
GET  /stream/{method}  - Server streaming
POST /upload           - Client streaming
```

**Headers:**
```
Content-Type: application/json
X-Protocol-Version: 1.0
X-Message-ID: {uuid}
X-Method: {method.name}
```

**Request Format:**
```http
POST /rpc HTTP/1.1
Host: api.example.com
Content-Type: application/json
X-Protocol-Version: 1.0
X-Message-ID: 550e8400-e29b-41d4-a716-446655440000

{
  "method": "users.get",
  "params": {
    "userId": "12345"
  }
}
```

**Response Format:**
```http
HTTP/1.1 200 OK
Content-Type: application/json
X-Protocol-Version: 1.0
X-Message-ID: 550e8400-e29b-41d4-a716-446655440000

{
  "result": {
    "userId": "12345",
    "name": "John Doe"
  }
}
```

### 3.2 Server-Sent Events (SSE)

For server streaming over HTTP:

```http
GET /stream/logs.tail HTTP/1.1
Host: api.example.com
Accept: text/event-stream

HTTP/1.1 200 OK
Content-Type: text/event-stream
Cache-Control: no-cache

data: {"sequence":1,"payload":{"message":"Log entry 1"}}

data: {"sequence":2,"payload":{"message":"Log entry 2"}}

data: [DONE]
```

### 3.3 HTTP/2 Support

- SHOULD support HTTP/2 for multiplexing
- MAY use HTTP/2 server push for events
- MUST support HTTP/1.1 fallback

## 4. WebSocket Transport

### 4.1 Connection Establishment

```typescript
const transport = new WebSocketTransport({
  url: 'wss://api.example.com/ws',
  protocols: ['universal-protocol-v1'],
  reconnect: {
    enabled: true,
    maxAttempts: 10,
    backoff: 'exponential'
  }
});
```

### 4.2 Message Framing

WebSocket messages MUST contain complete Universal Protocol messages:

```typescript
// Send
ws.send(JSON.stringify(message));

// Receive
ws.onmessage = (event) => {
  const message = JSON.parse(event.data);
  handleMessage(message);
};
```

### 4.3 Heartbeat/Ping-Pong

```typescript
interface HeartbeatConfig {
  enabled: boolean;
  interval: number;     // Milliseconds between pings
  timeout: number;      // Timeout for pong response
  payload?: any;        // Optional ping payload
}
```

**Ping Message:**
```json
{
  "id": "ping-123",
  "type": "ping",
  "metadata": {
    "timestamp": "2025-12-27T10:00:00.000Z"
  }
}
```

**Pong Message:**
```json
{
  "id": "ping-123",
  "type": "pong",
  "metadata": {
    "timestamp": "2025-12-27T10:00:00.050Z"
  }
}
```

### 4.4 Compression

- MUST support permessage-deflate extension
- SHOULD compress messages > 1KB
- MAY support custom compression algorithms

## 5. gRPC Transport

### 5.1 Protocol Buffers Definition

```protobuf
syntax = "proto3";

package wia.universal_protocol;

service UniversalProtocol {
  // Unary RPC
  rpc Call(Request) returns (Response);

  // Server streaming
  rpc ServerStream(Request) returns (stream StreamMessage);

  // Client streaming
  rpc ClientStream(stream StreamMessage) returns (Response);

  // Bidirectional streaming
  rpc BidirectionalStream(stream StreamMessage) returns (stream StreamMessage);
}

message Request {
  string id = 1;
  string method = 2;
  bytes payload = 3;
  map<string, string> metadata = 4;
}

message Response {
  string id = 1;
  bytes payload = 2;
  Error error = 3;
  map<string, string> metadata = 4;
}

message StreamMessage {
  string id = 1;
  uint64 sequence = 2;
  bytes payload = 3;
  bool is_complete = 4;
  map<string, string> metadata = 5;
}

message Error {
  string code = 1;
  string message = 2;
  bytes details = 3;
}
```

### 5.2 gRPC Metadata

```typescript
const metadata = new grpc.Metadata();
metadata.set('x-protocol-version', '1.0');
metadata.set('x-message-id', messageId);
metadata.set('x-method', methodName);
```

### 5.3 Error Handling

Map gRPC status codes to Universal Protocol error codes:

```typescript
const grpcToProtocolError = {
  [grpc.status.CANCELLED]: 'CANCELLED',
  [grpc.status.UNKNOWN]: 'INTERNAL_ERROR',
  [grpc.status.INVALID_ARGUMENT]: 'VALIDATION_ERROR',
  [grpc.status.DEADLINE_EXCEEDED]: 'TIMEOUT',
  [grpc.status.NOT_FOUND]: 'NOT_FOUND',
  [grpc.status.ALREADY_EXISTS]: 'CONFLICT',
  [grpc.status.PERMISSION_DENIED]: 'FORBIDDEN',
  [grpc.status.UNAUTHENTICATED]: 'UNAUTHORIZED',
  [grpc.status.RESOURCE_EXHAUSTED]: 'RATE_LIMIT_EXCEEDED',
  [grpc.status.FAILED_PRECONDITION]: 'PRECONDITION_FAILED',
  [grpc.status.UNIMPLEMENTED]: 'NOT_IMPLEMENTED',
  [grpc.status.UNAVAILABLE]: 'SERVICE_UNAVAILABLE'
};
```

## 6. MQTT Transport

### 6.1 Topic Structure

```
{prefix}/{clientId}/request/{method}
{prefix}/{clientId}/response/{requestId}
{prefix}/events/{topic}
```

**Example:**
```
universal-protocol/client-123/request/users.get
universal-protocol/client-123/response/550e8400-e29b-41d4-a716-446655440000
universal-protocol/events/user.registered
```

### 6.2 QoS Mapping

```typescript
const qosMapping = {
  [UniversalQoS.AT_MOST_ONCE]: 0,
  [UniversalQoS.AT_LEAST_ONCE]: 1,
  [UniversalQoS.EXACTLY_ONCE]: 2
};
```

### 6.3 Retained Messages

- Event messages MAY be retained
- Response messages MUST NOT be retained
- Last Will and Testament SHOULD be configured

### 6.4 Clean Session

```typescript
const mqttOptions = {
  clientId: 'universal-protocol-client-' + uuid(),
  clean: true,  // Start fresh each time
  keepalive: 60,
  reconnectPeriod: 5000
};
```

## 7. Custom Transport Implementation

### 7.1 Implementation Guide

```typescript
class CustomTransport implements Transport {
  private connection: any;
  private listeners: Map<string, Set<Function>>;

  async connect(): Promise<void> {
    // Establish connection
    this.connection = await createConnection(this.config);

    // Setup event listeners
    this.connection.on('data', this.handleData.bind(this));
    this.connection.on('error', this.handleError.bind(this));

    this.emit('connected');
  }

  async disconnect(): Promise<void> {
    await this.connection.close();
    this.emit('disconnected');
  }

  async send(message: Message): Promise<void> {
    const buffer = this.serialize(message);
    await this.connection.write(buffer);
  }

  async* receive(): AsyncIterableIterator<Message> {
    for await (const data of this.connection) {
      const message = this.deserialize(data);
      yield message;
    }
  }
}
```

### 7.2 Required Features

Custom transports MUST implement:
- Connection lifecycle management
- Message serialization/deserialization
- Error handling
- Event emission

## 8. Transport Selection

### 8.1 Selection Criteria

| Transport | Latency | Throughput | Compatibility | Use Case |
|-----------|---------|------------|---------------|----------|
| HTTP      | Medium  | Medium     | Excellent     | Web APIs, Simple RPC |
| WebSocket | Low     | High       | Good          | Real-time, Bidirectional |
| gRPC      | Very Low| Very High  | Good          | Microservices, High Performance |
| MQTT      | Low     | Medium     | Good          | IoT, Pub/Sub |
| Custom    | Varies  | Varies     | Limited       | Specialized Requirements |

### 8.2 Decision Tree

```
Need bidirectional streaming?
  ├─ Yes → WebSocket or gRPC
  └─ No
      ├─ Need high performance?
      │   ├─ Yes → gRPC
      │   └─ No → HTTP
      └─ Need pub/sub?
          └─ Yes → MQTT
```

## 9. Transport Negotiation

### 9.1 Capability Advertisement

```json
{
  "transports": [
    {
      "type": "websocket",
      "url": "wss://api.example.com/ws",
      "capabilities": {
        "compression": ["gzip", "deflate"],
        "maxMessageSize": 1048576
      }
    },
    {
      "type": "http",
      "url": "https://api.example.com",
      "capabilities": {
        "streaming": true,
        "compression": ["gzip", "br"]
      }
    }
  ]
}
```

### 9.2 Client Selection

Clients SHOULD:
1. Request available transports
2. Select best transport based on requirements
3. Fall back to alternative if preferred unavailable

## 10. Performance Optimization

### 10.1 Connection Pooling

```typescript
class ConnectionPool {
  private pool: Transport[] = [];
  private maxSize: number;

  async acquire(): Promise<Transport> {
    if (this.pool.length > 0) {
      return this.pool.pop()!;
    }

    if (this.pool.length < this.maxSize) {
      const transport = await this.createTransport();
      await transport.connect();
      return transport;
    }

    return await this.waitForAvailable();
  }

  release(transport: Transport): void {
    this.pool.push(transport);
  }
}
```

### 10.2 Message Batching

```typescript
class BatchTransport implements Transport {
  private queue: Message[] = [];
  private batchSize: number = 100;
  private batchTimeout: number = 50;

  async send(message: Message): Promise<void> {
    this.queue.push(message);

    if (this.queue.length >= this.batchSize) {
      await this.flush();
    } else {
      this.scheduleFlush();
    }
  }

  private async flush(): Promise<void> {
    if (this.queue.length === 0) return;

    const batch = this.queue.splice(0);
    await this.transport.send({
      type: 'batch',
      messages: batch
    });
  }
}
```

### 10.3 Compression

Transports SHOULD compress messages when:
- Message size > 1KB
- Compression ratio > 20%
- CPU usage acceptable

## 11. Security

### 11.1 TLS/SSL

All production transports MUST use TLS/SSL:
- Minimum TLS 1.2
- Strong cipher suites only
- Certificate validation required

### 11.2 Authentication

Transports SHOULD support:
- Token-based authentication (JWT, OAuth)
- Certificate-based authentication (mTLS)
- API key authentication

### 11.3 Rate Limiting

Transports SHOULD implement:
- Connection rate limiting
- Message rate limiting
- Bandwidth limiting

## 12. Monitoring

### 12.1 Metrics

Transports SHOULD expose:
- Connection count
- Message rate (sent/received)
- Error rate
- Latency percentiles
- Bandwidth usage

### 12.2 Health Checks

```typescript
interface TransportHealth {
  status: 'healthy' | 'degraded' | 'unhealthy';
  latency: number;
  errorRate: number;
  lastError?: Error;
}
```

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
