# WIA-LANG PHASE 3: Communication Protocol

## Version 1.0 | 弘益人間 · Benefit All Humanity

## 1. Protocol Overview

The WIA-LANG protocol defines standard communication methods for language preservation systems.

## 2. Transport Layer

### 2.1 HTTP/2
- Primary transport protocol
- TLS 1.3 required
- ALPN negotiation support

### 2.2 WebSocket
- Real-time bidirectional communication
- Automatic reconnection
- Heartbeat every 30 seconds

## 3. Message Format

### 3.1 Request Structure
```json
{
  "version": "1.0",
  "messageId": "uuid-v4",
  "timestamp": "2025-12-27T00:00:00Z",
  "action": "ACTION_TYPE",
  "payload": {...}
}
```

### 3.2 Response Structure
```json
{
  "messageId": "uuid-v4",
  "status": "success|error",
  "code": 200,
  "data": {...}
}
```

## 4. Authentication & Security

### 4.1 OAuth 2.0 Flow
```
1. Authorization Request
2. User Consent
3. Authorization Code
4. Access Token
5. API Access
```

### 4.2 JWT Token Format
```
Header.Payload.Signature
{
  "alg": "RS256",
  "typ": "JWT"
}
{
  "sub": "user-id",
  "iss": "wia-lang.org",
  "exp": 1703635200,
  "scopes": ["read", "write"]
}
```

## 5. Data Streaming Protocol

### 5.1 Chunked Transfer
- Chunk size: 64KB - 1MB
- Progress tracking
- Resumable uploads

### 5.2 Compression
- gzip (default)
- br (Brotli) for modern clients
- Content-Encoding header

## 6. Synchronization Protocol

### 6.1 Delta Sync
```json
{
  "since": "2025-12-27T00:00:00Z",
  "changes": [
    {
      "type": "update|delete|create",
      "entity": "recording",
      "id": "rec-123",
      "data": {...}
    }
  ]
}
```

## 7. Error Recovery

- Automatic retry with exponential backoff
- Circuit breaker pattern
- Graceful degradation

---
© 2025 SmileStory Inc. / WIA · Licensed under CC BY-SA 4.0
