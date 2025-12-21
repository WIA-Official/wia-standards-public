# WIA Cryo-Identity Communication Protocol
## Phase 3 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #06B6D4 (Cyan)

---

## Table of Contents

1. [Overview](#overview)
2. [Terminology](#terminology)
3. [Protocol Architecture](#protocol-architecture)
4. [Message Format](#message-format)
5. [Message Types](#message-types)
6. [Connection Management](#connection-management)
7. [Security](#security)
8. [Transport Layer](#transport-layer)
9. [Error Handling](#error-handling)
10. [Examples](#examples)
11. [References](#references)

---

## Overview

### 1.1 Purpose

The WIA Cryo-Identity Communication Protocol defines standardized message formats and communication patterns for secure identity data exchange between preservation facilities, verification systems, legal institutions, and future revival centers. This Phase 3 specification enables interoperable identity management across geographically distributed systems with cryptographic guarantees of data integrity and authenticity.

**Core Objectives**:
- Enable secure peer-to-peer identity verification
- Support distributed identity storage and retrieval
- Provide real-time identity event synchronization
- Ensure cryptographic integrity of all communications
- Enable cross-facility identity portability

### 1.2 Scope

This document defines:

| Component | Description |
|-----------|-------------|
| Message Format | JSON-based protocol messages |
| Message Types | Identity operations and events |
| Connection Management | Session lifecycle and reconnection |
| Security | Encryption, authentication, and authorization |
| Transport Layer | WebSocket, HTTPS, and P2P transports |

### 1.3 Related Documents

| Document | Description |
|----------|-------------|
| PHASE-1-DATA-FORMAT.md | Identity record structure |
| PHASE-2-API-INTERFACE.md | API interface specification |
| PHASE-4-INTEGRATION.md | Ecosystem integration |

---

## Terminology

### 2.1 Core Terms

| Term | Definition |
|------|------------|
| **Client** | Identity management system (facility, legal, medical) |
| **Server** | Identity verification service or registry |
| **Peer** | Equal participant in distributed identity network |
| **Message** | Protocol data unit exchanged between entities |
| **Session** | Authenticated connection between client and server |
| **Channel** | Logical communication path for specific identity |

### 2.2 Message Categories

| Category | Description |
|----------|-------------|
| **Identity** | Create, read, update, delete operations |
| **Verification** | Biometric and cryptographic verification |
| **Sync** | Distributed identity synchronization |
| **Event** | Real-time identity lifecycle events |
| **Control** | Connection and session management |

---

## Protocol Architecture

### 3.1 Layered Architecture

```
┌─────────────────────────────────────────────────────────┐
│              Application Layer                           │
│         (Identity Management Systems)                    │
├─────────────────────────────────────────────────────────┤
│              Protocol Layer                              │
│    (Message Format, Handlers, Validators)                │
├─────────────────────────────────────────────────────────┤
│              Security Layer                              │
│         (TLS, Encryption, Signatures)                    │
├─────────────────────────────────────────────────────────┤
│              Transport Layer                             │
│    (WebSocket / HTTPS / IPFS / Blockchain)               │
└─────────────────────────────────────────────────────────┘
```

### 3.2 Communication Patterns

| Pattern | Use Case | Example |
|---------|----------|---------|
| Request-Response | Identity queries | GET identity by ID |
| Publish-Subscribe | Event notifications | Identity status changes |
| Peer-to-Peer | Distributed verification | Cross-facility verification |
| Streaming | Biometric capture | Real-time fingerprint stream |

### 3.3 Network Topology

```
┌──────────────┐         ┌──────────────┐         ┌──────────────┐
│  Facility A  │◄───────►│Central Registry│◄──────►│  Facility B  │
│  (Client)    │         │   (Server)    │         │  (Client)    │
└──────────────┘         └──────────────┘         └──────────────┘
       │                        │                         │
       │                        │                         │
       ▼                        ▼                         ▼
┌──────────────┐         ┌──────────────┐         ┌──────────────┐
│  Blockchain  │         │  IPFS Node   │         │Legal Archive │
│   Anchor     │         │  (Storage)   │         │  (Backup)    │
└──────────────┘         └──────────────┘         └──────────────┘
```

---

## Message Format

### 4.1 Base Message Structure

All protocol messages follow this structure:

```json
{
  "protocol": "wia-cryo-identity",
  "version": "1.0.0",
  "messageId": "uuid-v4-string",
  "timestamp": 1702483200000,
  "type": "message_type",
  "sender": {
    "entityId": "facility-001",
    "entityType": "preservation_facility",
    "publicKey": "ed25519:abc123..."
  },
  "payload": {},
  "signature": "ed25519:signature..."
}
```

### 4.2 Field Definitions

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `protocol` | string | Y | Protocol identifier ("wia-cryo-identity") |
| `version` | string | Y | Protocol version (SemVer) |
| `messageId` | string | Y | Unique message ID (UUID v4) |
| `timestamp` | number | Y | Unix timestamp (milliseconds) |
| `type` | string | Y | Message type identifier |
| `sender` | object | Y | Sender identification |
| `payload` | object | Y | Message-specific data |
| `signature` | string | Y | Ed25519 signature of message |

### 4.3 JSON Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.live/cryo-identity/protocol/v1/message.schema.json",
  "title": "WIA Cryo-Identity Protocol Message",
  "type": "object",
  "required": ["protocol", "version", "messageId", "timestamp", "type", "sender", "payload", "signature"],
  "properties": {
    "protocol": {
      "type": "string",
      "const": "wia-cryo-identity"
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
      "minimum": 0
    },
    "type": {
      "type": "string",
      "enum": [
        "connect", "connect_ack", "disconnect",
        "identity_create", "identity_create_ack",
        "identity_get", "identity_get_ack",
        "identity_update", "identity_update_ack",
        "identity_delete", "identity_delete_ack",
        "verify_biometric", "verify_biometric_ack",
        "anchor_blockchain", "anchor_blockchain_ack",
        "subscribe", "subscribe_ack", "unsubscribe",
        "event", "error", "ping", "pong"
      ]
    },
    "sender": {
      "type": "object",
      "required": ["entityId", "entityType", "publicKey"],
      "properties": {
        "entityId": { "type": "string" },
        "entityType": { "type": "string" },
        "publicKey": { "type": "string" }
      }
    },
    "payload": {
      "type": "object"
    },
    "signature": {
      "type": "string"
    }
  }
}
```

---

## Message Types

### 5.1 Connection Messages

#### 5.1.1 connect

Establish authenticated connection.

```json
{
  "protocol": "wia-cryo-identity",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": 1702483200000,
  "type": "connect",
  "sender": {
    "entityId": "facility-001",
    "entityType": "preservation_facility",
    "publicKey": "ed25519:abc123..."
  },
  "payload": {
    "capabilities": ["identity_create", "biometric_verify", "blockchain_anchor"],
    "supportedBiometrics": ["fingerprint", "dna", "facial", "retinal"],
    "blockchainNetworks": ["ethereum", "polygon"],
    "maxMessageSize": 10485760,
    "compressionSupported": true
  },
  "signature": "ed25519:sig..."
}
```

#### 5.1.2 connect_ack

Connection acknowledgment.

```json
{
  "protocol": "wia-cryo-identity",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440001",
  "timestamp": 1702483200050,
  "type": "connect_ack",
  "sender": {
    "entityId": "registry-central",
    "entityType": "identity_registry",
    "publicKey": "ed25519:def456..."
  },
  "payload": {
    "success": true,
    "sessionId": "session-xyz789",
    "sessionExpiry": 1702569600000,
    "serverCapabilities": ["identity_storage", "verification", "search"],
    "rateLimit": {
      "requestsPerMinute": 100,
      "burstSize": 20
    }
  },
  "signature": "ed25519:sig..."
}
```

### 5.2 Identity Messages

#### 5.2.1 identity_create

Create new identity record.

```json
{
  "protocol": "wia-cryo-identity",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440010",
  "timestamp": 1702483200100,
  "type": "identity_create",
  "sender": {
    "entityId": "facility-001",
    "entityType": "preservation_facility",
    "publicKey": "ed25519:abc123..."
  },
  "payload": {
    "identity": {
      "$schema": "https://wia.live/cryo-identity/v1/schema.json",
      "version": "1.0.0",
      "personal": {
        "legalName": {
          "given": "encrypted:aes256:...",
          "family": "encrypted:aes256:...",
          "hash": "sha256:..."
        },
        "dateOfBirth": "encrypted:aes256:...",
        "nationality": ["US"]
      },
      "biometrics": {
        "fingerprints": [...],
        "dna": {...}
      }
    },
    "encryptionKey": "encrypted-with-server-public-key",
    "requestBlockchainAnchor": true
  },
  "signature": "ed25519:sig..."
}
```

#### 5.2.2 identity_create_ack

Identity creation acknowledgment.

```json
{
  "protocol": "wia-cryo-identity",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440011",
  "timestamp": 1702483200150,
  "type": "identity_create_ack",
  "sender": {
    "entityId": "registry-central",
    "entityType": "identity_registry",
    "publicKey": "ed25519:def456..."
  },
  "payload": {
    "success": true,
    "identityId": "ID-2025-000001",
    "recordHash": "sha256:abc123...",
    "storageLocation": "ipfs://QmXyz...",
    "blockchainAnchor": {
      "network": "ethereum",
      "transactionHash": "0xabc123...",
      "blockNumber": 12345678
    }
  },
  "signature": "ed25519:sig..."
}
```

#### 5.2.3 identity_get

Retrieve identity record.

```json
{
  "type": "identity_get",
  "payload": {
    "identityId": "ID-2025-000001",
    "fields": ["personal", "biometrics.fingerprints", "cryptographic"],
    "includeHistory": false,
    "decryptWith": "requestor-public-key"
  }
}
```

#### 5.2.4 identity_update

Update identity record.

```json
{
  "type": "identity_update",
  "payload": {
    "identityId": "ID-2025-000001",
    "updates": {
      "status": "preserved",
      "biometrics.retinal": {...}
    },
    "updateReason": "preservation_completed",
    "updateAuthorization": "signed-authorization-token"
  }
}
```

#### 5.2.5 identity_delete

Delete (archive) identity record.

```json
{
  "type": "identity_delete",
  "payload": {
    "identityId": "ID-2025-000001",
    "deleteReason": "legal_request",
    "deleteAuthorization": "signed-legal-order",
    "archiveLocation": "secure-archive-id",
    "permanentDelete": false
  }
}
```

### 5.3 Verification Messages

#### 5.3.1 verify_biometric

Verify biometric data.

```json
{
  "type": "verify_biometric",
  "payload": {
    "identityId": "ID-2025-000001",
    "biometricType": "fingerprint",
    "biometricData": {
      "template": "base64-encoded-template",
      "quality": 0.95,
      "capturedAt": "2025-01-15T10:00:00Z"
    },
    "requiredConfidence": 0.95,
    "verificationContext": "pre_revival_verification"
  }
}
```

#### 5.3.2 verify_biometric_ack

Biometric verification result.

```json
{
  "type": "verify_biometric_ack",
  "payload": {
    "success": true,
    "verified": true,
    "confidence": 0.98,
    "matchedBiometric": {
      "type": "fingerprint",
      "finger": "right_index",
      "registeredAt": "2024-06-15T10:00:00Z"
    },
    "verificationId": "verify-abc123",
    "verificationTimestamp": "2025-01-15T10:00:00Z"
  }
}
```

### 5.4 Blockchain Messages

#### 5.4.1 anchor_blockchain

Anchor identity to blockchain.

```json
{
  "type": "anchor_blockchain",
  "payload": {
    "identityId": "ID-2025-000001",
    "network": "ethereum",
    "dataHash": "sha256:abc123...",
    "contractAddress": "0x...",
    "gasLimit": 100000,
    "priority": "standard"
  }
}
```

#### 5.4.2 anchor_blockchain_ack

Blockchain anchor confirmation.

```json
{
  "type": "anchor_blockchain_ack",
  "payload": {
    "success": true,
    "network": "ethereum",
    "transactionHash": "0xabc123...",
    "blockNumber": 12345678,
    "confirmations": 12,
    "gasUsed": 21000,
    "timestamp": "2025-01-15T10:00:00Z"
  }
}
```

### 5.5 Subscription Messages

#### 5.5.1 subscribe

Subscribe to identity events.

```json
{
  "type": "subscribe",
  "payload": {
    "events": ["identity:updated", "identity:verified", "blockchain:anchored"],
    "filters": {
      "identityIds": ["ID-2025-000001", "ID-2025-000002"],
      "facilities": ["facility-001"]
    },
    "deliveryMode": "websocket"
  }
}
```

#### 5.5.2 event

Event notification.

```json
{
  "type": "event",
  "payload": {
    "eventType": "identity:verified",
    "eventId": "evt-abc123",
    "identityId": "ID-2025-000001",
    "eventData": {
      "verificationLevel": "cryptographic",
      "confidence": 0.99,
      "verifiedBy": "facility-002"
    },
    "eventTimestamp": "2025-01-15T10:00:00Z"
  }
}
```

### 5.6 Control Messages

#### 5.6.1 ping/pong

Heartbeat messages.

```json
{
  "type": "ping",
  "payload": {
    "sequence": 1,
    "clientTimestamp": 1702483200000
  }
}
```

```json
{
  "type": "pong",
  "payload": {
    "sequence": 1,
    "serverTimestamp": 1702483200005,
    "latency": 5
  }
}
```

### 5.7 Error Messages

```json
{
  "type": "error",
  "payload": {
    "code": 2003,
    "name": "BIOMETRIC_MATCH_FAILED",
    "message": "Biometric verification failed - confidence too low",
    "recoverable": true,
    "details": {
      "confidence": 0.65,
      "required": 0.95,
      "biometricType": "fingerprint"
    },
    "relatedMessageId": "550e8400-e29b-41d4-a716-446655440020"
  }
}
```

---

## Connection Management

### 6.1 Connection States

```
                    ┌─────────────────┐
                    │  DISCONNECTED   │
                    └────────┬────────┘
                             │ connect()
                             ▼
                    ┌─────────────────┐
                    │   CONNECTING    │
                    └────────┬────────┘
                             │ connect_ack (success)
              ┌──────────────┼──────────────┐
              │              ▼              │
              │     ┌─────────────────┐     │
   error/     │     │   CONNECTED     │     │ connect_ack
   timeout    │     └────────┬────────┘     │ (failure)
              │              │              │
              │              │ error/       │
              │              │ disconnect   │
              │              ▼              │
              │     ┌─────────────────┐     │
              │     │  RECONNECTING   │     │
              │     └────────┬────────┘     │
              │              │              │
              │   success    │   max retry  │
              │   ┌──────────┘   exceeded   │
              │   │              │          │
              │   │              ▼          │
              │   │     ┌─────────────────┐ │
              │   │     │     ERROR       │ │
              │   │     └────────┬────────┘ │
              │   │              │          │
              └───┼──────────────┼──────────┘
                  │              │
                  │              ▼
                  │     ┌─────────────────┐
                  └────►│  DISCONNECTED   │
                        └─────────────────┘
```

### 6.2 Connection Sequence

```
Client                                           Server
  │                                                │
  │ ──────────── TCP/WebSocket Connect ─────────► │
  │                                                │
  │ ◄────────── Connection Established ──────────  │
  │                                                │
  │ ──────────────── connect ──────────────────► │
  │                                                │
  │ ◄───────────── connect_ack ─────────────────  │
  │                                                │
  │ ──────────────── subscribe ─────────────────► │
  │                                                │
  │ ◄──────────── subscribe_ack ────────────────  │
  │                                                │
  │ ◄────────────── event ──────────────────────  │
  │ ◄────────────── event ──────────────────────  │
  │                  ...                           │
```

### 6.3 Session Management

| Setting | Default | Description |
|---------|---------|-------------|
| `sessionTimeout` | 3600000ms | Session expiration time |
| `heartbeatInterval` | 30000ms | Ping interval |
| `heartbeatTimeout` | 10000ms | Pong timeout |
| `maxReconnectAttempts` | 5 | Maximum reconnection attempts |
| `reconnectBackoff` | exponential | Reconnection delay strategy |

### 6.4 Reconnection Policy

**Exponential Backoff**:
```
attempt 1: 1000ms
attempt 2: 2000ms
attempt 3: 4000ms
attempt 4: 8000ms
attempt 5: 16000ms (max 30000ms)
```

---

## Security

### 7.1 Transport Security

| Layer | Technology | Purpose |
|-------|------------|---------|
| Transport | TLS 1.3 | Encrypted channel |
| Message | Ed25519 | Message signing |
| Payload | AES-256-GCM | Data encryption |
| Identity | JWT | Authentication |

### 7.2 Authentication Flow

```
Client                                Server
  │                                     │
  │ ─── connect (publicKey) ─────────► │
  │                                     │
  │                                     │ Verify signature
  │                                     │ Generate challenge
  │ ◄── connect_ack (challenge) ──────  │
  │                                     │
  │ Sign challenge                      │
  │ ─── auth_response (signature) ───► │
  │                                     │
  │                                     │ Verify signature
  │                                     │ Create session
  │ ◄── auth_complete (sessionId) ────  │
```

### 7.3 Message Signing

All messages must be signed using Ed25519:

```typescript
// Message signature generation
const messageData = JSON.stringify({
  protocol: message.protocol,
  version: message.version,
  messageId: message.messageId,
  timestamp: message.timestamp,
  type: message.type,
  sender: message.sender,
  payload: message.payload
});

const signature = ed25519.sign(messageData, privateKey);
message.signature = `ed25519:${base64url(signature)}`;
```

### 7.4 Payload Encryption

Sensitive data in payloads must be encrypted:

```typescript
// Encrypt sensitive fields
const encryptedData = aes256gcm.encrypt(
  JSON.stringify(sensitiveData),
  sharedSecret,
  nonce
);

payload.personal.legalName.given = `encrypted:aes256:${base64url(encryptedData)}`;
```

### 7.5 Access Control

| Role | Permissions |
|------|-------------|
| `preservation_facility` | Create, update, verify identities |
| `legal_representative` | Read, audit identities |
| `medical_professional` | Read biometrics, update status |
| `revival_center` | Read, verify, update (revival status) |
| `registry_admin` | All operations |

---

## Transport Layer

### 8.1 WebSocket Transport (Primary)

**Connection URL**:
```
wss://identity.wia.live/v1/ws
```

**Subprotocol**: `wia-cryo-identity-v1`

**Example**:
```javascript
const ws = new WebSocket(
  'wss://identity.wia.live/v1/ws',
  'wia-cryo-identity-v1'
);

ws.onmessage = (event) => {
  const message = JSON.parse(event.data);
  handleMessage(message);
};
```

### 8.2 HTTPS Transport (Secondary)

**Base URL**: `https://identity.wia.live/v1/api`

**REST Endpoints**:
```http
POST   /identities
GET    /identities/:id
PATCH  /identities/:id
DELETE /identities/:id
POST   /verify/biometric
POST   /blockchain/anchor
```

### 8.3 IPFS Transport (Storage)

**Purpose**: Distributed identity storage

**Format**:
```
ipfs://QmXyz.../identity-ID-2025-000001.json
```

**Metadata**:
```json
{
  "identityId": "ID-2025-000001",
  "ipfsHash": "QmXyz...",
  "encryptionKey": "encrypted-with-facility-key",
  "timestamp": "2025-01-15T10:00:00Z"
}
```

### 8.4 Blockchain Transport (Anchoring)

**Networks**: Ethereum, Polygon, Avalanche

**Smart Contract**:
```solidity
contract CryoIdentityRegistry {
  struct IdentityAnchor {
    bytes32 identityHash;
    uint256 timestamp;
    address facility;
  }

  mapping(bytes32 => IdentityAnchor) public anchors;

  function anchorIdentity(bytes32 identityHash) public {
    anchors[identityHash] = IdentityAnchor({
      identityHash: identityHash,
      timestamp: block.timestamp,
      facility: msg.sender
    });
  }
}
```

---

## Error Handling

### 9.1 Error Codes

#### Connection Errors (1xxx)

| Code | Name | Description |
|------|------|-------------|
| 1000 | `CONNECTION_CLOSED` | Normal disconnection |
| 1001 | `CONNECTION_LOST` | Unexpected disconnection |
| 1002 | `CONNECTION_TIMEOUT` | Connection timeout |
| 1003 | `AUTHENTICATION_FAILED` | Auth failure |
| 1004 | `SESSION_EXPIRED` | Session timeout |

#### Identity Errors (2xxx)

| Code | Name | Description |
|------|------|-------------|
| 2001 | `IDENTITY_NOT_FOUND` | Identity doesn't exist |
| 2002 | `IDENTITY_ALREADY_EXISTS` | Duplicate identity |
| 2003 | `BIOMETRIC_MATCH_FAILED` | Verification failed |
| 2004 | `INVALID_IDENTITY_DATA` | Data validation error |
| 2005 | `PERMISSION_DENIED` | Insufficient permissions |

#### Blockchain Errors (3xxx)

| Code | Name | Description |
|------|------|-------------|
| 3001 | `BLOCKCHAIN_UNAVAILABLE` | Network unavailable |
| 3002 | `ANCHOR_FAILED` | Anchoring failed |
| 3003 | `TRANSACTION_FAILED` | TX failed |
| 3004 | `INSUFFICIENT_FUNDS` | Not enough gas |

### 9.2 Error Recovery Strategies

| Error Type | Recovery Strategy |
|------------|-------------------|
| `CONNECTION_LOST` | Auto-reconnect with backoff |
| `SESSION_EXPIRED` | Re-authenticate |
| `BIOMETRIC_MATCH_FAILED` | Request recapture |
| `BLOCKCHAIN_UNAVAILABLE` | Queue for later |

---

## Examples

### 10.1 Complete Connection Flow

```typescript
import { CryoIdentityClient } from 'wia-cryo-identity/protocol';

async function connect() {
  const client = new CryoIdentityClient({
    serverUrl: 'wss://identity.wia.live/v1/ws',
    privateKey: facilityPrivateKey,
    entityId: 'facility-001',
    entityType: 'preservation_facility'
  });

  // Connect
  await client.connect();

  // Subscribe to events
  await client.subscribe({
    events: ['identity:verified', 'blockchain:anchored']
  });

  // Event handler
  client.on('event', (event) => {
    console.log('Event received:', event.eventType);
  });

  return client;
}
```

### 10.2 Create Identity via Protocol

```typescript
async function createIdentity(client: CryoIdentityClient, identityData: any) {
  const message = {
    type: 'identity_create',
    payload: {
      identity: identityData,
      requestBlockchainAnchor: true
    }
  };

  const response = await client.send(message);

  if (response.payload.success) {
    console.log('Identity created:', response.payload.identityId);
    console.log('Blockchain TX:', response.payload.blockchainAnchor.transactionHash);
  }

  return response.payload;
}
```

### 10.3 Verify Biometric

```typescript
async function verifyBiometric(
  client: CryoIdentityClient,
  identityId: string,
  biometricData: any
) {
  const message = {
    type: 'verify_biometric',
    payload: {
      identityId,
      biometricType: 'fingerprint',
      biometricData,
      requiredConfidence: 0.95
    }
  };

  const response = await client.send(message);

  if (response.payload.verified) {
    console.log('Verification successful');
    console.log('Confidence:', response.payload.confidence);
    return true;
  } else {
    console.log('Verification failed');
    return false;
  }
}
```

### 10.4 Event Subscription

```typescript
async function subscribeToIdentity(
  client: CryoIdentityClient,
  identityId: string
) {
  await client.subscribe({
    events: ['identity:updated', 'identity:verified'],
    filters: { identityIds: [identityId] }
  });

  client.on('event', (event) => {
    if (event.payload.identityId === identityId) {
      console.log(`Event for ${identityId}:`, event.payload.eventType);

      switch (event.payload.eventType) {
        case 'identity:updated':
          handleUpdate(event.payload);
          break;
        case 'identity:verified':
          handleVerification(event.payload);
          break;
      }
    }
  });
}
```

### 10.5 Blockchain Verification

```typescript
async function verifyBlockchainAnchor(
  client: CryoIdentityClient,
  identityId: string,
  txHash: string
) {
  // Get identity from blockchain
  const anchor = await client.getBlockchainAnchor(txHash);

  // Verify hash matches
  const identity = await client.getIdentity(identityId);
  const computedHash = sha256(JSON.stringify(identity));

  if (anchor.identityHash === computedHash) {
    console.log('Blockchain anchor verified');
    console.log('Block:', anchor.blockNumber);
    console.log('Confirmations:', anchor.confirmations);
    return true;
  }

  return false;
}
```

---

## References

### Protocol Standards

- [RFC 6455 - The WebSocket Protocol](https://tools.ietf.org/html/rfc6455)
- [RFC 7519 - JSON Web Token (JWT)](https://tools.ietf.org/html/rfc7519)
- [RFC 8032 - Edwards-Curve Digital Signature Algorithm](https://tools.ietf.org/html/rfc8032)

### Security Standards

- [NIST SP 800-63 - Digital Identity Guidelines](https://pages.nist.gov/800-63-3/)
- [TLS 1.3 - RFC 8446](https://tools.ietf.org/html/rfc8446)

### Related WIA Standards

- [WIA Cryo-Identity Data Format (Phase 1)](/cryo-identity/spec/PHASE-1-DATA-FORMAT.md)
- [WIA Cryo-Identity API Interface (Phase 2)](/cryo-identity/spec/PHASE-2-API-INTERFACE.md)

---

<div align="center">

**WIA Cryo-Identity Communication Protocol v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA Standards Committee**

**MIT License**

</div>
