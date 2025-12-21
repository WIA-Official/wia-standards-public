# WIA Food Allergy Passport Protocol Standard
## Phase 3 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #84CC16 (Lime)

---

## Table of Contents

1. [Overview](#overview)
2. [Terminology](#terminology)
3. [Transport Layer](#transport-layer)
4. [Message Format](#message-format)
5. [Connection Lifecycle](#connection-lifecycle)
6. [QR Code Protocol](#qr-code-protocol)
7. [Real-Time Alerts](#real-time-alerts)
8. [Security](#security)
9. [Protocol Examples](#protocol-examples)
10. [Version History](#version-history)

---

## Overview

### 1.1 Purpose

The WIA Food Allergy Passport Protocol Standard defines communication protocols for secure, real-time exchange of allergy information between mobile applications, restaurant POS systems, airline catering systems, and emergency medical services. This Phase 3 specification builds upon Phase 1 (Data Format) and Phase 2 (API Interface) to provide network-level communication standards.

**Core Objectives**:
- Enable secure real-time allergy data transmission
- Support offline QR code scanning with delayed sync
- Provide WebSocket-based live alerts for critical allergies
- Ensure end-to-end encryption of sensitive medical data
- Support multi-platform communication (iOS, Android, Web, POS)

### 1.2 Scope

This standard defines:

| Component | Description |
|-----------|-------------|
| **Transport Layer** | HTTPS, WebSocket, NFC protocols |
| **Message Format** | JSON-based message structure |
| **QR Code Protocol** | Offline-capable QR encoding/decoding |
| **Real-Time Protocol** | WebSocket event streaming |
| **Security Layer** | TLS 1.3, encryption, authentication |

### 1.3 Protocol Stack

```
┌─────────────────────────────────────────┐
│     Application Layer                   │
│  (Passport Apps, Restaurant POS)        │
├─────────────────────────────────────────┤
│     Phase 2: API Interface              │
│  (REST, GraphQL, gRPC endpoints)        │
├─────────────────────────────────────────┤
│     Phase 3: Protocol Layer             │
│  (Message Format, Event Streams)        │
├─────────────────────────────────────────┤
│     Transport Layer                     │
│  (HTTPS, WebSocket, NFC)                │
├─────────────────────────────────────────┤
│     Security Layer                      │
│  (TLS 1.3, E2E Encryption)              │
└─────────────────────────────────────────┘
```

---

## Terminology

### 2.1 Core Terms

| Term | Definition |
|------|------------|
| **Message Envelope** | Standardized wrapper for all protocol messages |
| **Event Stream** | Real-time WebSocket connection for alerts |
| **QR Payload** | Encoded allergy data in QR code format |
| **Handshake** | Initial connection establishment process |
| **Heartbeat** | Keep-alive signal for persistent connections |
| **Channel** | Logical communication path for specific events |

### 2.2 Protocol Types

| Protocol | Transport | Use Case |
|----------|-----------|----------|
| **HTTP/REST** | HTTPS | Standard API operations |
| **WebSocket** | WSS (Secure) | Real-time alerts, live updates |
| **QR Code** | Optical | Offline sharing, quick access |
| **NFC** | Near Field | Contactless sharing |
| **MQTT** | TCP/TLS | IoT device integration |

---

## Transport Layer

### 3.1 HTTPS Transport

Primary transport for API operations.

#### Configuration

```
Protocol: HTTPS
Version: HTTP/2 or HTTP/3
TLS Version: 1.3 (minimum)
Cipher Suites: TLS_AES_256_GCM_SHA384, TLS_CHACHA20_POLY1305_SHA256
Certificate: Valid X.509 certificate (Let's Encrypt or commercial CA)
```

#### Headers

```http
# Request Headers
User-Agent: WIA-AllergyPassport/1.0 (iOS 17.2; iPhone)
Accept: application/json
Content-Type: application/json
Authorization: Bearer {token}
X-API-Version: 1.0.0
X-Request-ID: uuid-v4
X-Client-Platform: ios|android|web|pos
X-Client-Version: 1.2.3

# Response Headers
Content-Type: application/json
X-Request-ID: uuid-v4
X-Rate-Limit-Remaining: 95
X-Rate-Limit-Reset: 1642345678
Cache-Control: private, max-age=60
```

#### Request Flow

```
Client                                  Server
  │                                       │
  ├──── HTTPS Request ───────────────────>│
  │     (TLS 1.3 Handshake)              │
  │                                       │
  │<──── TLS Server Hello ────────────────┤
  │                                       │
  ├──── Encrypted Application Data ─────>│
  │     (JSON payload)                    │
  │                                       │
  │<──── Encrypted Response ──────────────┤
  │     (JSON response)                   │
  │                                       │
```

### 3.2 WebSocket Transport

Real-time bidirectional communication.

#### Connection URL

```
wss://ws.wia.live/food-allergy-passport/v1/stream
```

#### Connection Parameters

```javascript
const ws = new WebSocket('wss://ws.wia.live/food-allergy-passport/v1/stream', {
  headers: {
    'Authorization': 'Bearer {token}',
    'X-Client-ID': 'client-uuid',
    'X-Protocol-Version': '1.0.0'
  }
});
```

#### Message Types

| Type | Direction | Purpose |
|------|-----------|---------|
| `hello` | Client → Server | Initial handshake |
| `welcome` | Server → Client | Handshake acknowledgment |
| `subscribe` | Client → Server | Subscribe to event channels |
| `unsubscribe` | Client → Server | Unsubscribe from channels |
| `event` | Server → Client | Event notification |
| `ping` | Client ↔ Server | Keep-alive heartbeat |
| `pong` | Client ↔ Server | Heartbeat response |
| `error` | Server → Client | Error notification |
| `close` | Client ↔ Server | Connection termination |

### 3.3 NFC Protocol

Near Field Communication for contactless sharing.

#### NDEF Record Format

```
Record Type: application/vnd.wia.allergy-passport
Payload: Compressed JSON (gzip)
Max Size: 8KB
```

#### Data Structure

```javascript
{
  "type": "allergy-passport",
  "version": "1.0.0",
  "passportId": "FAP-2025-000001",
  "compressed": true,
  "payload": "H4sIAAAAAAAA/6tWKk5NLErP..." // base64 gzipped JSON
}
```

---

## Message Format

### 4.1 Message Envelope

All protocol messages use a standardized envelope.

```typescript
interface MessageEnvelope<T> {
  // Envelope metadata
  id: string;                    // Unique message ID (UUID v4)
  type: MessageType;             // Message type
  version: string;               // Protocol version (1.0.0)
  timestamp: string;             // ISO 8601 timestamp

  // Authentication & Security
  sender?: string;               // Sender identifier
  signature?: string;            // Message signature (optional)
  encrypted?: boolean;           // Is payload encrypted

  // Payload
  data: T;                       // Typed message payload

  // Metadata
  meta?: {
    requestId?: string;          // Correlation ID
    ttl?: number;                // Time to live (seconds)
    priority?: 'low' | 'normal' | 'high' | 'critical';
    retryCount?: number;
  };
}
```

### 4.2 Message Types

#### Passport Sync Message

```json
{
  "id": "msg_abc123",
  "type": "passport.sync",
  "version": "1.0.0",
  "timestamp": "2025-01-15T10:30:00Z",
  "data": {
    "passportId": "FAP-2025-000001",
    "action": "update",
    "changes": {
      "allergies": [
        {
          "allergen": "shellfish",
          "allergenCode": "FDA-SHELLFISH",
          "severity": "severe",
          "added": true
        }
      ]
    },
    "version": 2
  }
}
```

#### Real-Time Alert Message

```json
{
  "id": "msg_xyz789",
  "type": "alert.allergy",
  "version": "1.0.0",
  "timestamp": "2025-01-15T18:45:00Z",
  "data": {
    "passportId": "FAP-2025-000001",
    "severity": "critical",
    "allergen": "peanuts",
    "location": {
      "type": "restaurant",
      "id": "REST-12345",
      "name": "Seoul Bistro",
      "tableNumber": "15"
    },
    "action": "immediate_notification",
    "message": "Customer with severe peanut allergy seated at table 15"
  },
  "meta": {
    "priority": "critical",
    "ttl": 300
  }
}
```

#### QR Scan Event

```json
{
  "id": "msg_qr_001",
  "type": "qr.scanned",
  "version": "1.0.0",
  "timestamp": "2025-01-15T12:00:00Z",
  "data": {
    "passportId": "FAP-2025-000001",
    "scannedBy": {
      "type": "restaurant",
      "id": "REST-12345",
      "name": "Seoul Bistro"
    },
    "location": {
      "latitude": 37.5665,
      "longitude": 126.9780
    },
    "allergySummary": {
      "count": 1,
      "severityLevel": "anaphylaxis",
      "requiresEpiPen": true
    }
  }
}
```

---

## Connection Lifecycle

### 5.1 WebSocket Connection Flow

```
Client                                Server
  │                                     │
  ├─── WebSocket Upgrade Request ─────>│
  │    (HTTP → WSS)                     │
  │                                     │
  │<─── 101 Switching Protocols ────────┤
  │                                     │
  ├─── hello ─────────────────────────>│
  │    { clientId, version }            │
  │                                     │
  │<─── welcome ────────────────────────┤
  │    { sessionId, capabilities }      │
  │                                     │
  ├─── subscribe ─────────────────────>│
  │    { channels: ["passport.sync"] }  │
  │                                     │
  │<─── subscribed ─────────────────────┤
  │    { channels: [...] }              │
  │                                     │
  │     ┌─────────────────┐             │
  │     │ Active Session  │             │
  │     │ (Event Stream)  │             │
  │     └─────────────────┘             │
  │                                     │
  │<─── event ──────────────────────────┤
  │    (Real-time updates)              │
  │                                     │
  ├─── ping ──────────────────────────>│
  │    (Every 30 seconds)               │
  │                                     │
  │<─── pong ───────────────────────────┤
  │                                     │
  ├─── close ─────────────────────────>│
  │    { code: 1000, reason: "..." }    │
  │                                     │
  │<─── Connection Closed ──────────────┤
  │                                     │
```

### 5.2 Handshake Messages

#### Client Hello

```json
{
  "type": "hello",
  "data": {
    "clientId": "client_abc123",
    "version": "1.0.0",
    "platform": "ios",
    "capabilities": [
      "passport.sync",
      "alert.allergy",
      "qr.generate"
    ],
    "timezone": "Asia/Seoul",
    "language": "ko"
  }
}
```

#### Server Welcome

```json
{
  "type": "welcome",
  "data": {
    "sessionId": "session_xyz789",
    "serverVersion": "1.0.0",
    "capabilities": [
      "passport.sync",
      "alert.allergy",
      "qr.generate",
      "translation.realtime",
      "emergency.broadcast"
    ],
    "heartbeatInterval": 30000,
    "maxMessageSize": 1048576
  }
}
```

### 5.3 Channel Subscription

#### Subscribe Request

```json
{
  "type": "subscribe",
  "data": {
    "channels": [
      {
        "name": "passport.FAP-2025-000001",
        "events": ["sync", "update", "share"]
      },
      {
        "name": "alerts.critical",
        "filter": {
          "severity": ["anaphylaxis", "severe"]
        }
      },
      {
        "name": "restaurant.REST-12345",
        "events": ["notification", "menu-update"]
      }
    ]
  }
}
```

#### Subscribe Response

```json
{
  "type": "subscribed",
  "data": {
    "channels": [
      {
        "name": "passport.FAP-2025-000001",
        "subscribed": true,
        "events": ["sync", "update", "share"]
      },
      {
        "name": "alerts.critical",
        "subscribed": true
      },
      {
        "name": "restaurant.REST-12345",
        "subscribed": true
      }
    ]
  }
}
```

### 5.4 Heartbeat Protocol

```javascript
// Client sends ping every 30 seconds
{
  "type": "ping",
  "timestamp": "2025-01-15T10:30:00Z"
}

// Server responds with pong
{
  "type": "pong",
  "timestamp": "2025-01-15T10:30:00Z",
  "latency": 45  // milliseconds
}
```

---

## QR Code Protocol

### 6.1 QR Code Format

#### Structure

```
Protocol: FAP (Food Allergy Passport)
Version: v1
Encoding: Base64-encoded JSON
Compression: Gzip (optional for large payloads)
Error Correction: Level H (30% recovery)
Max Size: Version 10 (57x57 modules, ~700 bytes)
```

#### URL Scheme

```
fap://v1/{base64-encoded-data}
```

or

```
https://wia.live/fap/v1/{base64-encoded-data}
```

### 6.2 QR Payload Structure

```typescript
interface QRPayload {
  // Version & Identification
  v: string;                  // Protocol version (1.0.0)
  pid: string;                // Passport ID

  // Essential Allergy Data
  a: Array<{                  // Allergies
    n: string;                // Allergen name
    c: string;                // Allergen code
    s: 1 | 2 | 3 | 4;         // Severity (1=mild, 4=anaphylaxis)
    t?: string;               // Threshold
  }>;

  // Emergency Information
  e?: {                       // Emergency
    ep?: boolean;             // Has EpiPen
    ph?: string;              // Emergency phone (encrypted)
  };

  // Metadata
  m: {
    exp: number;              // Expiration timestamp
    sig: string;              // Digital signature
    enc?: boolean;            // Is encrypted
  };
}
```

### 6.3 QR Code Example

#### Uncompressed

```json
{
  "v": "1.0.0",
  "pid": "FAP-2025-000001",
  "a": [
    {
      "n": "peanuts",
      "c": "FDA-PEANUT",
      "s": 4,
      "t": "trace"
    }
  ],
  "e": {
    "ep": true,
    "ph": "encrypted:..."
  },
  "m": {
    "exp": 1737000000,
    "sig": "ed25519:..."
  }
}
```

Encoded as:
```
fap://v1/eyJ2IjoiMS4wLjAiLCJwaWQiOiJGQVAtMjAyNS0wMDAwMDEiLCJhIjpbeyJuIjoicGVhbnV0cyIsImMiOiJGREEtUEVBTlVUIiwicyI6NCwidCI6InRyYWNlIn1dLCJlIjp7ImVwIjp0cnVlLCJwaCI6ImVuY3J5cHRlZDouLi4ifSwibSI6eyJleHAiOjE3MzcwMDAwMDAsInNpZyI6ImVkMjU1MTk6Li4uIn19
```

### 6.4 QR Code Scanning Protocol

```
┌─────────────────────────────────────────────┐
│  1. Scan QR Code                            │
│     ↓                                       │
│  2. Decode Base64                           │
│     ↓                                       │
│  3. Verify Signature                        │
│     ↓                                       │
│  4. Check Expiration                        │
│     ↓                                       │
│  5. Display Allergy Summary                 │
│     ↓                                       │
│  6. (Optional) Fetch Full Details via API   │
└─────────────────────────────────────────────┘
```

---

## Real-Time Alerts

### 7.1 Alert Types

| Alert Type | Severity | Use Case |
|------------|----------|----------|
| `alert.allergy.critical` | Critical | Anaphylaxis risk detected |
| `alert.allergy.warning` | High | Severe allergy detected |
| `alert.cross-contamination` | Medium | Potential cross-contamination |
| `alert.medication.reminder` | Low | EpiPen expiration reminder |
| `alert.emergency.activated` | Critical | Emergency protocol triggered |

### 7.2 Critical Alert Flow

```
Mobile App                 WebSocket Server              Restaurant POS
    │                            │                              │
    │── Scan Passport QR ───────>│                              │
    │                            │                              │
    │                            │── Detect Anaphylaxis ───────>│
    │                            │   Severity Alert              │
    │                            │                              │
    │                            │<── Acknowledge ───────────────┤
    │                            │                              │
    │<── Alert Confirmation ─────┤                              │
    │   (Kitchen notified)       │                              │
    │                            │                              │
```

### 7.3 Alert Message Format

```json
{
  "id": "alert_critical_001",
  "type": "alert.allergy.critical",
  "version": "1.0.0",
  "timestamp": "2025-01-15T18:45:00Z",
  "data": {
    "passportId": "FAP-2025-000001",
    "severity": "anaphylaxis",
    "allergen": "peanuts",
    "location": {
      "type": "restaurant",
      "id": "REST-12345",
      "name": "Seoul Bistro",
      "table": "15",
      "coordinates": {
        "lat": 37.5665,
        "lng": 126.9780
      }
    },
    "action": {
      "type": "immediate_notification",
      "recipients": ["kitchen", "manager", "server"],
      "message": {
        "en": "CRITICAL: Customer at table 15 has severe peanut allergy",
        "ko": "긴급: 15번 테이블 고객은 심각한 땅콩 알레르기가 있습니다"
      }
    },
    "emergency": {
      "hasEpiPen": true,
      "emergencyContact": "encrypted:...",
      "nearestHospital": {
        "name": "Seoul National University Hospital",
        "distance": "1.2 km",
        "phone": "02-2072-2114"
      }
    }
  },
  "meta": {
    "priority": "critical",
    "ttl": 300,
    "requiresAcknowledgment": true,
    "acknowledgmentDeadline": "2025-01-15T18:46:00Z"
  }
}
```

### 7.4 Acknowledgment Protocol

```json
{
  "type": "alert.acknowledge",
  "data": {
    "alertId": "alert_critical_001",
    "acknowledgedBy": "Chef Kim",
    "acknowledgedAt": "2025-01-15T18:45:10Z",
    "action": "Kitchen notified, preparing peanut-free meal",
    "estimatedReadyTime": "2025-01-15T19:15:00Z"
  }
}
```

---

## Security

### 8.1 Transport Security

#### TLS Configuration

```
Protocol: TLS 1.3
Cipher Suites (in order of preference):
  - TLS_AES_256_GCM_SHA384
  - TLS_CHACHA20_POLY1305_SHA256
  - TLS_AES_128_GCM_SHA256

Certificate Requirements:
  - Valid X.509 certificate from trusted CA
  - Subject Alternative Name (SAN) for all domains
  - Certificate Transparency (CT) logs
  - OCSP stapling enabled
  - Minimum 2048-bit RSA or 256-bit ECC

HSTS Header:
  Strict-Transport-Security: max-age=31536000; includeSubDomains; preload
```

### 8.2 End-to-End Encryption

#### Encryption Flow

```
┌──────────────────────────────────────────────┐
│  Client                                      │
│    ↓                                         │
│  1. Generate ephemeral key pair              │
│    ↓                                         │
│  2. Encrypt sensitive data with AES-256-GCM  │
│    ↓                                         │
│  3. Encrypt AES key with recipient's public  │
│     key (RSA-OAEP or ECIES)                  │
│    ↓                                         │
│  4. Send encrypted payload + encrypted key   │
│                                              │
│  Server/Recipient                            │
│    ↓                                         │
│  5. Decrypt AES key with private key         │
│    ↓                                         │
│  6. Decrypt payload with AES key             │
│    ↓                                         │
│  7. Verify message signature                 │
└──────────────────────────────────────────────┘
```

#### Encrypted Message Format

```typescript
interface EncryptedMessage {
  version: string;
  algorithm: 'AES-256-GCM';
  keyAlgorithm: 'RSA-OAEP-256' | 'ECIES-P256';
  encryptedData: string;      // Base64
  encryptedKey: string;       // Base64
  iv: string;                 // Base64 initialization vector
  authTag: string;            // Base64 authentication tag
  recipient: string;          // Public key fingerprint
  signature?: string;         // Optional message signature
}
```

### 8.3 Message Authentication

#### Digital Signatures

```typescript
interface SignedMessage<T> {
  data: T;
  signature: {
    algorithm: 'Ed25519' | 'ECDSA-P256';
    value: string;            // Base64 signature
    publicKey: string;        // Base64 public key
    timestamp: string;
  };
}
```

#### Signature Verification Process

```javascript
// 1. Extract message data and signature
const { data, signature } = signedMessage;

// 2. Reconstruct canonical message
const canonical = JSON.stringify(data, Object.keys(data).sort());

// 3. Verify signature
const isValid = await crypto.subtle.verify(
  { name: 'Ed25519' },
  publicKey,
  base64Decode(signature.value),
  new TextEncoder().encode(canonical)
);

// 4. Check timestamp freshness (within 5 minutes)
const timestamp = new Date(signature.timestamp);
const now = new Date();
const isFresh = (now - timestamp) < 300000;

return isValid && isFresh;
```

---

## Protocol Examples

### 9.1 Full WebSocket Session

```javascript
// Client connects
const ws = new WebSocket('wss://ws.wia.live/food-allergy-passport/v1/stream');

// 1. Send hello
ws.send(JSON.stringify({
  type: 'hello',
  data: {
    clientId: 'app-ios-12345',
    version: '1.0.0',
    platform: 'ios',
    capabilities: ['passport.sync', 'alert.allergy']
  }
}));

// 2. Receive welcome
ws.onmessage = (event) => {
  const message = JSON.parse(event.data);

  if (message.type === 'welcome') {
    console.log('Session:', message.data.sessionId);

    // 3. Subscribe to channels
    ws.send(JSON.stringify({
      type: 'subscribe',
      data: {
        channels: [
          { name: 'passport.FAP-2025-000001', events: ['sync', 'update'] },
          { name: 'alerts.critical', filter: { severity: ['anaphylaxis'] } }
        ]
      }
    }));
  }

  if (message.type === 'subscribed') {
    console.log('Subscribed to:', message.data.channels);
  }

  // 4. Handle real-time events
  if (message.type === 'event') {
    console.log('Event:', message.data);

    if (message.data.type === 'alert.allergy.critical') {
      showCriticalAlert(message.data);
    }
  }
};

// 5. Heartbeat
setInterval(() => {
  ws.send(JSON.stringify({
    type: 'ping',
    timestamp: new Date().toISOString()
  }));
}, 30000);
```

### 9.2 QR Code Generation and Scanning

```python
import json
import base64
import qrcode
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.asymmetric import ed25519

# Generate QR Code
def generate_passport_qr(passport_id, allergies):
    # 1. Create minimal payload
    payload = {
        "v": "1.0.0",
        "pid": passport_id,
        "a": [
            {
                "n": allergy["allergen"],
                "c": allergy["allergenCode"],
                "s": severity_to_int(allergy["severity"]),
                "t": allergy.get("threshold")
            }
            for allergy in allergies
        ],
        "e": {
            "ep": has_epipen(allergies)
        },
        "m": {
            "exp": int(time.time()) + (365 * 24 * 60 * 60),  # 1 year
            "sig": sign_payload(payload, private_key)
        }
    }

    # 2. Encode to base64
    json_str = json.dumps(payload, separators=(',', ':'))
    encoded = base64.urlsafe_b64encode(json_str.encode()).decode()

    # 3. Create QR code
    qr_data = f"fap://v1/{encoded}"
    qr = qrcode.QRCode(
        version=10,
        error_correction=qrcode.constants.ERROR_CORRECT_H,
        box_size=10,
        border=4
    )
    qr.add_data(qr_data)
    qr.make(fit=True)

    return qr.make_image(fill_color="black", back_color="white")

# Scan QR Code
def scan_passport_qr(qr_data):
    # 1. Parse URL
    if not qr_data.startswith("fap://v1/"):
        raise ValueError("Invalid QR code format")

    encoded = qr_data.replace("fap://v1/", "")

    # 2. Decode base64
    json_str = base64.urlsafe_b64decode(encoded).decode()
    payload = json.loads(json_str)

    # 3. Verify signature
    if not verify_signature(payload):
        raise ValueError("Invalid signature")

    # 4. Check expiration
    if payload["m"]["exp"] < time.time():
        raise ValueError("QR code expired")

    # 5. Return allergy summary
    return {
        "passportId": payload["pid"],
        "allergies": [
            {
                "allergen": a["n"],
                "severity": int_to_severity(a["s"]),
                "threshold": a.get("t")
            }
            for a in payload["a"]
        ],
        "hasEpiPen": payload["e"].get("ep", False)
    }
```

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial release |

---

<div align="center">

**WIA Food Allergy Passport Standard v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA**

**MIT License**

</div>
