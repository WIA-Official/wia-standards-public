# WIA-CORE-007 PHASE 1: Core Protocol Specification

**Version:** 1.0
**Status:** APPROVED
**Last Updated:** 2025-12-27
**Authors:** WIA Standards Committee

## 1. Introduction

PHASE 1 defines the core protocol specification for WIA-CORE-007 Universal Protocol. This document establishes the fundamental message structure, communication patterns, and base requirements that all implementations MUST follow.

## 2. Message Structure

### 2.1 Universal Message Format

All messages in the Universal Protocol MUST conform to the following structure:

```typescript
interface UniversalMessage {
  id: string;              // Unique message identifier (UUID v4 recommended)
  type: MessageType;       // Message type enum
  method?: string;         // Method name for RPC calls
  topic?: string;          // Topic name for pub/sub
  payload?: any;           // Message payload (encoding-dependent)
  metadata: Metadata;      // Message metadata
  error?: ErrorObject;     // Error information (if applicable)
  sequence?: number;       // Sequence number for streams
}

enum MessageType {
  REQUEST = 'request',
  RESPONSE = 'response',
  STREAM = 'stream',
  EVENT = 'event'
}
```

### 2.2 Message Identifier

- **Format:** UUID v4 (RFC 4122)
- **Uniqueness:** MUST be unique within the scope of a single connection
- **Generation:** Client-generated for requests, server-generated for server-initiated messages
- **Correlation:** Response messages MUST use the same ID as the corresponding request

### 2.3 Metadata Object

```typescript
interface Metadata {
  timestamp: string;           // ISO 8601 timestamp
  version: string;             // Protocol version (e.g., "1.0")
  encoding?: string;           // Encoding format (e.g., "json", "protobuf")
  transport?: string;          // Transport type (e.g., "websocket", "http")
  timeout?: number;            // Timeout in milliseconds
  priority?: MessagePriority;  // Message priority
  [key: string]: any;          // Additional custom metadata
}

enum MessagePriority {
  LOW = 'low',
  NORMAL = 'normal',
  HIGH = 'high',
  CRITICAL = 'critical'
}
```

### 2.4 Error Object

```typescript
interface ErrorObject {
  code: string;              // Error code (e.g., "TIMEOUT", "NOT_FOUND")
  message: string;           // Human-readable error message
  details?: any;             // Additional error details
  stack?: string;            // Stack trace (development only)
  retryable?: boolean;       // Whether the error is retryable
  retryAfter?: number;       // Suggested retry delay in milliseconds
}
```

## 3. Communication Patterns

### 3.1 Request-Response Pattern

**Description:** Client sends a request and waits for exactly one response.

**Message Flow:**
```
Client → Server: REQUEST message
Server → Client: RESPONSE message (with same ID)
```

**Requirements:**
- Request MUST include `method` field
- Response MUST include same `id` as request
- Server MUST respond to every request (success or error)
- Client SHOULD implement timeout handling

**Example:**
```json
// Request
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "type": "request",
  "method": "users.get",
  "payload": {
    "userId": "12345"
  },
  "metadata": {
    "timestamp": "2025-12-27T10:00:00.000Z",
    "version": "1.0",
    "timeout": 5000
  }
}

// Response
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "type": "response",
  "payload": {
    "userId": "12345",
    "name": "John Doe",
    "email": "john@example.com"
  },
  "metadata": {
    "timestamp": "2025-12-27T10:00:00.123Z",
    "version": "1.0"
  }
}
```

### 3.2 Server Streaming Pattern

**Description:** Client sends one request, server responds with multiple messages.

**Message Flow:**
```
Client → Server: REQUEST message
Server → Client: STREAM message (multiple, with sequence numbers)
Server → Client: STREAM message (isComplete: true)
```

**Requirements:**
- Stream messages MUST include `sequence` field
- Sequence numbers MUST be monotonically increasing
- Final message MUST indicate completion via metadata
- Client MAY cancel stream at any time

**Example:**
```json
// Request
{
  "id": "stream-123",
  "type": "request",
  "method": "logs.stream",
  "payload": {
    "service": "api-server"
  }
}

// Stream messages
{
  "id": "stream-123",
  "type": "stream",
  "sequence": 1,
  "payload": {
    "timestamp": "2025-12-27T10:00:01.000Z",
    "level": "info",
    "message": "Server started"
  },
  "metadata": {
    "isComplete": false
  }
}
```

### 3.3 Client Streaming Pattern

**Description:** Client sends multiple messages, server responds with one message.

**Message Flow:**
```
Client → Server: STREAM message (multiple)
Client → Server: STREAM message (isComplete: true)
Server → Client: RESPONSE message
```

**Requirements:**
- Client MUST indicate completion
- Server MUST buffer or process stream appropriately
- Server responds after receiving completion signal

### 3.4 Bidirectional Streaming Pattern

**Description:** Both client and server can send multiple messages independently.

**Message Flow:**
```
Client ⇄ Server: STREAM messages (bidirectional)
```

**Requirements:**
- Both sides MAY send messages at any time
- Both sides MUST handle backpressure
- Stream MAY be cancelled by either side

### 3.5 Publish-Subscribe Pattern

**Description:** Fire-and-forget message broadcasting.

**Message Flow:**
```
Publisher → Server: EVENT message
Server → Subscribers: EVENT message (to all matching subscribers)
```

**Requirements:**
- Event messages MUST include `topic` field
- No response expected
- Delivery MAY be unreliable (QoS dependent)

## 4. Protocol Versioning

### 4.1 Version Format

Protocol version MUST follow Semantic Versioning 2.0.0 (semver.org):

- **Format:** MAJOR.MINOR.PATCH
- **Example:** "1.0.0", "1.2.3", "2.0.0"

### 4.2 Version Negotiation

```typescript
interface VersionNegotiation {
  supported: string[];     // List of supported versions
  preferred: string;       // Preferred version
  current: string;         // Currently used version
}
```

### 4.3 Compatibility Rules

- **MAJOR version:** Breaking changes
- **MINOR version:** New features (backward compatible)
- **PATCH version:** Bug fixes (backward compatible)

Implementations MUST support at least the current MAJOR version.

## 5. Connection Lifecycle

### 5.1 Connection States

```typescript
enum ConnectionState {
  DISCONNECTED = 'disconnected',
  CONNECTING = 'connecting',
  CONNECTED = 'connected',
  RECONNECTING = 'reconnecting',
  CLOSING = 'closing',
  CLOSED = 'closed'
}
```

### 5.2 State Transitions

```
DISCONNECTED → CONNECTING → CONNECTED
CONNECTED → DISCONNECTED (on error)
CONNECTED → RECONNECTING → CONNECTED
CONNECTED → CLOSING → CLOSED
```

### 5.3 Connection Events

Implementations MUST emit the following events:

- `connecting`: Connection attempt started
- `connected`: Connection established
- `disconnected`: Connection lost
- `reconnecting`: Reconnection attempt
- `error`: Error occurred
- `close`: Connection closed intentionally

## 6. Quality of Service (QoS)

### 6.1 QoS Levels

```typescript
enum QoSLevel {
  AT_MOST_ONCE = 0,    // Fire-and-forget
  AT_LEAST_ONCE = 1,   // Guaranteed delivery, possible duplicates
  EXACTLY_ONCE = 2     // Guaranteed delivery, no duplicates
}
```

### 6.2 QoS Implementation

- **Level 0:** No acknowledgment required
- **Level 1:** Acknowledgment required, retransmission on timeout
- **Level 2:** Two-phase commit with deduplication

## 7. Security Considerations

### 7.1 Authentication

- Implementations SHOULD support token-based authentication
- Tokens SHOULD be transmitted in metadata
- Tokens MUST be validated before processing requests

### 7.2 Authorization

- Implementations SHOULD implement role-based access control
- Each method SHOULD declare required permissions
- Unauthorized requests MUST return appropriate error

### 7.3 Encryption

- Implementations SHOULD support TLS/SSL for transport encryption
- Payload encryption MAY be implemented at application layer
- Sensitive data SHOULD NOT be logged

## 8. Error Handling

### 8.1 Standard Error Codes

```typescript
enum ErrorCode {
  // Client errors (4xx)
  BAD_REQUEST = 'BAD_REQUEST',
  UNAUTHORIZED = 'UNAUTHORIZED',
  FORBIDDEN = 'FORBIDDEN',
  NOT_FOUND = 'NOT_FOUND',
  VALIDATION_ERROR = 'VALIDATION_ERROR',

  // Server errors (5xx)
  INTERNAL_ERROR = 'INTERNAL_ERROR',
  SERVICE_UNAVAILABLE = 'SERVICE_UNAVAILABLE',
  TIMEOUT = 'TIMEOUT',

  // Protocol errors
  INVALID_MESSAGE = 'INVALID_MESSAGE',
  UNSUPPORTED_VERSION = 'UNSUPPORTED_VERSION',

  // Transport errors
  CONNECTION_FAILED = 'CONNECTION_FAILED',
  NETWORK_ERROR = 'NETWORK_ERROR'
}
```

### 8.2 Error Response Format

Error responses MUST include:
- `error.code`: Machine-readable error code
- `error.message`: Human-readable description
- `error.retryable`: Whether retry is appropriate

## 9. Performance Requirements

### 9.1 Latency

- Request-response latency SHOULD be < 100ms for local network
- Message overhead SHOULD be < 1KB for typical messages

### 9.2 Throughput

- Implementations SHOULD support at least 1000 messages/second
- Streaming SHOULD support sustained rates without buffering issues

### 9.3 Scalability

- Implementations SHOULD support concurrent connections
- Connection pooling SHOULD be implemented
- Backpressure mechanisms MUST be provided

## 10. Compliance

### 10.1 Conformance Levels

- **Level 1 (Basic):** Request-response pattern only
- **Level 2 (Standard):** All patterns except bidirectional streaming
- **Level 3 (Full):** All patterns and features

### 10.2 Testing Requirements

Implementations MUST pass:
- Unit tests for message parsing
- Integration tests for all supported patterns
- Load tests demonstrating performance requirements

## 11. References

- RFC 4122: UUID
- RFC 3339: Date/Time format
- Semantic Versioning 2.0.0
- JSON Schema Draft 7
- Protocol Buffers v3

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
