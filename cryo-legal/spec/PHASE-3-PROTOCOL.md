# WIA Cryo-Legal Standard - Phase 3: Communication Protocol

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #06B6D4 (Cyan)

---

## 1. Overview

### 1.1 Purpose

This document defines the communication protocol for the WIA Cryo-Legal Standard, enabling secure, real-time synchronization of legal documents, signatures, and compliance status across distributed systems.

### 1.2 Protocol Selection Rationale

| Protocol | Use Case | Justification |
|----------|----------|---------------|
| WebSocket | Real-time updates | Bi-directional, low latency |
| HTTPS | API calls | Standard, secure, cacheable |
| gRPC | Service-to-service | High performance, typed |

### 1.3 Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Client Applications                       │
│           (Web, Mobile, Desktop, Facility Systems)          │
└─────────────────────┬───────────────────────────────────────┘
                      │ WebSocket / HTTPS
                      ▼
┌─────────────────────────────────────────────────────────────┐
│                    API Gateway                               │
│              (Authentication, Rate Limiting)                │
└─────────────────────┬───────────────────────────────────────┘
                      │ gRPC
                      ▼
┌─────────────────────────────────────────────────────────────┐
│              Cryo-Legal Service Cluster                     │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │  Document   │  │  Signature  │  │ Compliance  │        │
│  │  Service    │  │  Service    │  │  Service    │        │
│  └─────────────┘  └─────────────┘  └─────────────┘        │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. Transport Layer

### 2.1 WebSocket Connection

**Endpoint:**
```
wss://ws.wia.live/cryo-legal/v1
```

**Connection Parameters:**
```
wss://ws.wia.live/cryo-legal/v1?token={jwt_token}&client_id={client_id}
```

### 2.2 Connection Handshake

```
Client                                    Server
   │                                        │
   │ ────── WebSocket Upgrade Request ────► │
   │         (with JWT token)               │
   │                                        │
   │ ◄───── 101 Switching Protocols ─────── │
   │                                        │
   │ ────────── auth message ─────────────► │
   │                                        │
   │ ◄──────── auth_ack message ────────── │
   │                                        │
   │ ────────── subscribe ────────────────► │
   │                                        │
   │ ◄──────── subscribe_ack ───────────── │
   │                                        │
   │ ◄─────── real-time events ──────────  │
   │                                        │
```

### 2.3 TLS Requirements

| Requirement | Value |
|-------------|-------|
| TLS Version | 1.3 (minimum 1.2) |
| Cipher Suites | TLS_AES_256_GCM_SHA384, TLS_CHACHA20_POLY1305_SHA256 |
| Certificate | Valid CA-signed certificate |
| HSTS | Enabled, max-age=31536000 |

---

## 3. Message Format

### 3.1 Base Message Structure

```json
{
  "protocol": "wia-cryo-legal",
  "version": "1.0.0",
  "messageId": "msg-uuid-v4",
  "timestamp": 1705312200000,
  "type": "message_type",
  "payload": {}
}
```

### 3.2 Message Types

| Type | Direction | Description |
|------|-----------|-------------|
| `auth` | C → S | Authentication request |
| `auth_ack` | S → C | Authentication response |
| `subscribe` | C → S | Subscribe to events |
| `subscribe_ack` | S → C | Subscription confirmation |
| `unsubscribe` | C → S | Cancel subscription |
| `document.created` | S → C | New document event |
| `document.updated` | S → C | Document modified |
| `document.signed` | S → C | Signature added |
| `signature.request` | S → C | Signature requested |
| `compliance.alert` | S → C | Compliance issue |
| `ping` | C → S | Keep-alive |
| `pong` | S → C | Keep-alive response |
| `error` | S → C | Error notification |

### 3.3 JSON Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.live/cryo-legal/protocol/v1/message.schema.json",
  "type": "object",
  "required": ["protocol", "version", "messageId", "timestamp", "type", "payload"],
  "properties": {
    "protocol": {
      "type": "string",
      "const": "wia-cryo-legal"
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
      "description": "Unix timestamp in milliseconds"
    },
    "type": {
      "type": "string",
      "enum": [
        "auth", "auth_ack",
        "subscribe", "subscribe_ack", "unsubscribe",
        "document.created", "document.updated", "document.signed",
        "signature.request", "compliance.alert",
        "ping", "pong", "error"
      ]
    },
    "payload": {
      "type": "object"
    }
  }
}
```

---

## 4. Connection Lifecycle

### 4.1 Authentication Message

```json
{
  "protocol": "wia-cryo-legal",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440001",
  "timestamp": 1705312200000,
  "type": "auth",
  "payload": {
    "token": "eyJhbGciOiJSUzI1NiIs...",
    "clientId": "facility-001",
    "clientType": "facility_system",
    "capabilities": ["documents", "signatures", "compliance"]
  }
}
```

### 4.2 Authentication Response

```json
{
  "protocol": "wia-cryo-legal",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440002",
  "timestamp": 1705312200050,
  "type": "auth_ack",
  "payload": {
    "success": true,
    "sessionId": "session-abc123",
    "expiresAt": 1705398600000,
    "permissions": ["documents:read", "documents:write", "signatures:create"],
    "serverInfo": {
      "version": "1.0.0",
      "region": "us-west-2"
    }
  }
}
```

### 4.3 Subscription

```json
{
  "protocol": "wia-cryo-legal",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440003",
  "timestamp": 1705312200100,
  "type": "subscribe",
  "payload": {
    "channels": [
      {
        "channel": "documents",
        "filter": {
          "partyId": "party-uuid-123",
          "documentTypes": ["cryopreservation_contract", "consent_form"]
        }
      },
      {
        "channel": "signatures",
        "filter": {
          "pendingOnly": true
        }
      },
      {
        "channel": "compliance",
        "filter": {
          "jurisdictions": ["US", "CA"]
        }
      }
    ]
  }
}
```

### 4.4 Keep-Alive

**Ping (Client):**
```json
{
  "protocol": "wia-cryo-legal",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440010",
  "timestamp": 1705312230000,
  "type": "ping",
  "payload": {
    "sequence": 1
  }
}
```

**Pong (Server):**
```json
{
  "protocol": "wia-cryo-legal",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440011",
  "timestamp": 1705312230005,
  "type": "pong",
  "payload": {
    "sequence": 1,
    "latency": 5
  }
}
```

### 4.5 Reconnection Policy

| Attempt | Delay | Max Delay |
|---------|-------|-----------|
| 1 | 1s | - |
| 2 | 2s | - |
| 3 | 4s | - |
| 4 | 8s | - |
| 5+ | 16s | 30s |

```javascript
const delay = Math.min(Math.pow(2, attempt) * 1000, 30000);
```

---

## 5. Event Messages

### 5.1 Document Created

```json
{
  "protocol": "wia-cryo-legal",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440020",
  "timestamp": 1705312300000,
  "type": "document.created",
  "payload": {
    "documentId": "doc-uuid-123",
    "documentType": "cryopreservation_contract",
    "status": "draft",
    "createdBy": "user-123",
    "parties": [
      {
        "partyId": "party-uuid-1",
        "role": "subject",
        "legalName": "John Smith"
      }
    ],
    "jurisdiction": {
      "primaryCountry": "US",
      "governingLaw": "Arizona"
    },
    "requiredActions": [
      {
        "action": "sign",
        "partyId": "party-uuid-1",
        "deadline": "2025-02-15T00:00:00Z"
      }
    ]
  }
}
```

### 5.2 Document Signed

```json
{
  "protocol": "wia-cryo-legal",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440021",
  "timestamp": 1705312400000,
  "type": "document.signed",
  "payload": {
    "documentId": "doc-uuid-123",
    "signatureId": "sig-uuid-456",
    "signerId": "party-uuid-1",
    "signerName": "John Smith",
    "signedAt": "2025-01-15T14:30:00Z",
    "documentStatus": "pending",
    "signatureProgress": {
      "completed": 1,
      "required": 2
    },
    "nextSigner": {
      "partyId": "party-uuid-2",
      "role": "facility",
      "legalName": "CryoLife Inc."
    }
  }
}
```

### 5.3 Signature Request

```json
{
  "protocol": "wia-cryo-legal",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440022",
  "timestamp": 1705312500000,
  "type": "signature.request",
  "payload": {
    "requestId": "req-uuid-789",
    "documentId": "doc-uuid-123",
    "documentType": "cryopreservation_contract",
    "documentTitle": "Cryopreservation Services Agreement",
    "requestedSigner": {
      "partyId": "party-uuid-2",
      "role": "facility"
    },
    "requestedBy": {
      "partyId": "party-uuid-1",
      "legalName": "John Smith"
    },
    "urgency": "normal",
    "deadline": "2025-02-01T00:00:00Z",
    "signUrl": "https://sign.wia.live/doc-uuid-123?token=xyz"
  }
}
```

### 5.4 Compliance Alert

```json
{
  "protocol": "wia-cryo-legal",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440023",
  "timestamp": 1705312600000,
  "type": "compliance.alert",
  "payload": {
    "alertId": "alert-uuid-101",
    "severity": "warning",
    "documentId": "doc-uuid-123",
    "jurisdiction": "US-CA",
    "issue": {
      "code": "WITNESS_COUNT",
      "message": "California requires 2 witnesses for advance directives",
      "currentValue": 1,
      "requiredValue": 2
    },
    "remediation": {
      "action": "add_witness",
      "deadline": "2025-02-01T00:00:00Z",
      "instructions": "Add one additional witness to comply with California requirements"
    }
  }
}
```

---

## 6. Security

### 6.1 Message Encryption

All WebSocket messages are encrypted using TLS. For additional security with sensitive payloads:

```json
{
  "protocol": "wia-cryo-legal",
  "version": "1.0.0",
  "messageId": "...",
  "timestamp": 1705312200000,
  "type": "document.signed",
  "encrypted": true,
  "payload": {
    "algorithm": "AES-256-GCM",
    "iv": "base64-iv",
    "data": "base64-encrypted-payload",
    "tag": "base64-auth-tag"
  }
}
```

### 6.2 Message Signing

Critical messages include digital signatures:

```json
{
  "protocol": "wia-cryo-legal",
  "version": "1.0.0",
  "messageId": "...",
  "timestamp": 1705312200000,
  "type": "document.signed",
  "payload": {...},
  "signature": {
    "algorithm": "RS256",
    "keyId": "server-key-001",
    "value": "base64-signature"
  }
}
```

### 6.3 Signature Verification

```python
import jwt
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.asymmetric import padding

def verify_message_signature(message: dict, public_key) -> bool:
    signature_info = message.get('signature')
    if not signature_info:
        return False

    # Create canonical payload
    payload = json.dumps({
        k: v for k, v in message.items()
        if k != 'signature'
    }, sort_keys=True, separators=(',', ':'))

    signature = base64.b64decode(signature_info['value'])

    try:
        public_key.verify(
            signature,
            payload.encode(),
            padding.PKCS1v15(),
            hashes.SHA256()
        )
        return True
    except Exception:
        return False
```

---

## 7. Error Handling

### 7.1 Error Message Format

```json
{
  "protocol": "wia-cryo-legal",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440099",
  "timestamp": 1705312200000,
  "type": "error",
  "payload": {
    "code": 4001,
    "name": "SUBSCRIPTION_FAILED",
    "message": "Invalid channel specified",
    "details": {
      "channel": "invalid_channel",
      "validChannels": ["documents", "signatures", "compliance"]
    },
    "recoverable": true,
    "relatedMessageId": "550e8400-e29b-41d4-a716-446655440003"
  }
}
```

### 7.2 Error Codes

| Code | Name | Description | Recoverable |
|------|------|-------------|-------------|
| 1001 | `AUTH_FAILED` | Authentication failed | No |
| 1002 | `AUTH_EXPIRED` | Token expired | Yes (refresh) |
| 1003 | `PERMISSION_DENIED` | Insufficient permissions | No |
| 2001 | `DOCUMENT_NOT_FOUND` | Document doesn't exist | No |
| 2002 | `DOCUMENT_LOCKED` | Document is locked | Retry later |
| 3001 | `SIGNATURE_INVALID` | Invalid signature | No |
| 3002 | `CERTIFICATE_EXPIRED` | Certificate expired | Yes (renew) |
| 4001 | `SUBSCRIPTION_FAILED` | Invalid subscription | Yes (fix params) |
| 4002 | `CHANNEL_UNAVAILABLE` | Channel not available | Retry later |
| 5001 | `RATE_LIMITED` | Too many messages | Yes (backoff) |
| 5002 | `SERVER_ERROR` | Internal error | Retry later |

### 7.3 Recovery Procedures

```typescript
async function handleError(error: ProtocolError): Promise<void> {
  switch (error.code) {
    case 1002: // AUTH_EXPIRED
      await refreshToken();
      await reconnect();
      break;

    case 2002: // DOCUMENT_LOCKED
    case 4002: // CHANNEL_UNAVAILABLE
    case 5002: // SERVER_ERROR
      await delay(exponentialBackoff(retryCount));
      await retry();
      break;

    case 5001: // RATE_LIMITED
      await delay(error.details.retryAfter || 60000);
      await retry();
      break;

    default:
      logError(error);
      notifyUser(error);
  }
}
```

---

## 8. Implementation Examples

### 8.1 TypeScript Client

```typescript
import { CryoLegalWebSocket } from '@wia/cryo-legal';

const ws = new CryoLegalWebSocket({
  url: 'wss://ws.wia.live/cryo-legal/v1',
  token: 'jwt-token',
  clientId: 'my-app',
  autoReconnect: true
});

// Connect and authenticate
await ws.connect();

// Subscribe to channels
await ws.subscribe({
  channels: [
    { channel: 'documents', filter: { partyId: 'my-party-id' } },
    { channel: 'signatures', filter: { pendingOnly: true } }
  ]
});

// Handle events
ws.on('document.created', (event) => {
  console.log('New document:', event.payload.documentId);
});

ws.on('signature.request', (event) => {
  console.log('Signature requested:', event.payload.documentTitle);
  showSignatureModal(event.payload);
});

ws.on('compliance.alert', (event) => {
  console.log('Compliance issue:', event.payload.issue.message);
  handleComplianceAlert(event.payload);
});

ws.on('error', (error) => {
  console.error('Protocol error:', error);
});

// Graceful disconnect
await ws.disconnect();
```

### 8.2 Python Client

```python
import asyncio
from wia_cryo_legal import CryoLegalWebSocket

async def main():
    ws = CryoLegalWebSocket(
        url='wss://ws.wia.live/cryo-legal/v1',
        token='jwt-token',
        client_id='my-app'
    )

    await ws.connect()

    # Subscribe
    await ws.subscribe(
        channels=[
            {'channel': 'documents', 'filter': {'partyId': 'my-party-id'}},
            {'channel': 'signatures', 'filter': {'pendingOnly': True}}
        ]
    )

    # Handle events
    @ws.on('document.created')
    async def on_document_created(event):
        print(f"New document: {event['payload']['documentId']}")

    @ws.on('signature.request')
    async def on_signature_request(event):
        print(f"Signature requested: {event['payload']['documentTitle']}")

    # Run until cancelled
    try:
        await asyncio.Future()
    except asyncio.CancelledError:
        await ws.disconnect()

asyncio.run(main())
```

---

## 9. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial release |

---

<div align="center">

**WIA Cryo-Legal Standard v1.0.0**

Phase 3: Communication Protocol

**弘益人間 (홍익인간)** · Benefit All Humanity

---

© 2025 WIA Standards Committee

MIT License

</div>
