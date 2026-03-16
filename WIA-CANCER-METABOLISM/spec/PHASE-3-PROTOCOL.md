# WIA-CANCER-METABOLISM Phase 3: Protocol Specification

**Version**: 1.0
**Status**: Draft
**Last Updated**: 2025-12-29

## Overview

The WIA-CANCER-METABOLISM Protocol specification defines the communication architecture, transport mechanisms, security protocols, and data exchange patterns for cancer metabolism data. This specification ensures secure, reliable, and efficient transmission of metabolomic data across institutions, platforms, and jurisdictions.

### Protocol Design Principles

1. **Security First**: End-to-end encryption, zero-trust architecture
2. **Interoperability**: Multiple transport options for diverse environments
3. **Reliability**: Guaranteed delivery with retry mechanisms
4. **Performance**: Optimized for both batch and real-time operations
5. **Compliance**: HIPAA, GDPR, and regulatory framework adherence
6. **Scalability**: Support from single-institution to global networks

## Protocol Architecture

### Layered Architecture Model

```
┌─────────────────────────────────────────────────────────┐
│         Application Layer (Phase 2 - API)               │
│  (RESTful endpoints, GraphQL, Business Logic)           │
├─────────────────────────────────────────────────────────┤
│         Protocol Layer (Phase 3 - THIS SPEC)            │
│  (Message Format, Routing, Security, QoS)               │
├─────────────────────────────────────────────────────────┤
│         Transport Layer                                  │
│  (HTTPS, WebSocket, gRPC, AMQP)                         │
├─────────────────────────────────────────────────────────┤
│         Network Layer (TCP/IP)                          │
└─────────────────────────────────────────────────────────┘
```

### Communication Patterns

1. **Request-Response**: Synchronous API calls (HTTPS)
2. **Publish-Subscribe**: Event-driven updates (WebSocket, AMQP)
3. **Streaming**: Real-time data feeds (gRPC, WebSocket)
4. **Message Queue**: Asynchronous batch processing (AMQP, Kafka)
5. **Federation**: Institution-to-institution data sharing (HTTPS, gRPC)

## Transport Options

### 1. HTTPS (Primary)

The default transport for synchronous request-response operations.

**Configuration**:
- **TLS Version**: TLS 1.3 (minimum TLS 1.2)
- **Cipher Suites**:
  - `TLS_AES_256_GCM_SHA384`
  - `TLS_CHACHA20_POLY1305_SHA256`
  - `TLS_AES_128_GCM_SHA256`
- **Certificate**: X.509 v3 with 2048-bit RSA minimum (4096-bit recommended)
- **HSTS**: Enabled with `max-age=31536000; includeSubDomains; preload`
- **Certificate Pinning**: Recommended for mobile/desktop clients

**Endpoint Example**:
```
https://api.wia.org/cancer-metabolism/v1/profiles
```

**Connection Parameters**:
```javascript
{
  "protocol": "https",
  "host": "api.wia.org",
  "port": 443,
  "tlsVersion": "1.3",
  "timeout": 30000,
  "keepAlive": true,
  "maxConnections": 100
}
```

### 2. WebSocket (Real-time)

For real-time streaming of metabolic data, live updates, and collaborative analysis.

**Configuration**:
- **Protocol**: WSS (WebSocket Secure over TLS 1.3)
- **Subprotocol**: `wia-cancer-metabolism-v1`
- **Heartbeat**: 30-second ping/pong
- **Message Format**: JSON or MessagePack
- **Max Frame Size**: 1MB
- **Compression**: Permessage-deflate enabled

**Connection URL**:
```
wss://stream.wia.org/cancer-metabolism/v1
```

**Handshake**:
```http
GET /cancer-metabolism/v1 HTTP/1.1
Host: stream.wia.org
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==
Sec-WebSocket-Protocol: wia-cancer-metabolism-v1
Sec-WebSocket-Version: 13
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

**Frame Structure**:
```json
{
  "type": "message",
  "event": "profile.updated",
  "data": {
    "profileId": "a1b2c3d4-e5f6-4a7b-8c9d-0e1f2a3b4c5d",
    "changes": [...]
  },
  "timestamp": "2025-03-20T14:30:00Z",
  "sequence": 12345
}
```

### 3. gRPC (High-performance)

For high-throughput, low-latency communication between services.

**Configuration**:
- **Protocol**: gRPC over HTTP/2 with TLS 1.3
- **Serialization**: Protocol Buffers (protobuf)
- **Compression**: gzip
- **Deadline**: 10 seconds default
- **Retry Policy**: Exponential backoff

**Service Definition** (cancer_metabolism.proto):
```protobuf
syntax = "proto3";

package wia.cancer_metabolism.v1;

service CancerMetabolismService {
  // Unary RPC
  rpc GetProfile(GetProfileRequest) returns (MetabolicProfile);

  // Server streaming
  rpc StreamProfiles(StreamProfilesRequest) returns (stream MetabolicProfile);

  // Client streaming
  rpc UploadProfiles(stream MetabolicProfile) returns (UploadResponse);

  // Bidirectional streaming
  rpc AnalyzeRealtime(stream AnalysisRequest) returns (stream AnalysisResponse);
}

message GetProfileRequest {
  string profile_id = 1;
}

message MetabolicProfile {
  string profile_id = 1;
  string patient_id = 2;
  string sample_id = 3;
  google.protobuf.Timestamp collection_date = 4;
  string cancer_type = 5;
  repeated Metabolite metabolites = 6;
}

message Metabolite {
  string metabolite_id = 1;
  string name = 2;
  Concentration concentration = 3;
  float confidence = 4;
}

message Concentration {
  double value = 1;
  string unit = 2;
  string method = 3;
}
```

**gRPC Client Example**:
```javascript
const grpc = require('@grpc/grpc-js');
const protoLoader = require('@grpc/proto-loader');

const packageDefinition = protoLoader.loadSync('cancer_metabolism.proto');
const proto = grpc.loadPackageDefinition(packageDefinition);

const client = new proto.wia.cancer_metabolism.v1.CancerMetabolismService(
  'grpc.wia.org:443',
  grpc.credentials.createSsl()
);

// Get profile
client.GetProfile(
  { profile_id: 'a1b2c3d4-e5f6-4a7b-8c9d-0e1f2a3b4c5d' },
  (error, response) => {
    if (!error) {
      console.log('Profile:', response);
    }
  }
);
```

### 4. AMQP (Message Queue)

For asynchronous batch processing and event-driven architectures.

**Configuration**:
- **Protocol**: AMQP 0-9-1 over TLS
- **Broker**: RabbitMQ, Azure Service Bus, Amazon MQ
- **Exchange Type**: Topic
- **Durability**: Persistent messages
- **QoS**: Prefetch count = 10

**Exchange and Queue Setup**:
```javascript
const amqp = require('amqplib');

const connection = await amqp.connect('amqps://mq.wia.org', {
  cert: fs.readFileSync('client-cert.pem'),
  key: fs.readFileSync('client-key.pem'),
  ca: [fs.readFileSync('ca-cert.pem')]
});

const channel = await connection.createChannel();

// Declare exchange
await channel.assertExchange('cancer-metabolism', 'topic', {
  durable: true
});

// Declare queue
await channel.assertQueue('profile-processing', {
  durable: true,
  maxPriority: 10
});

// Bind queue
await channel.bindQueue(
  'profile-processing',
  'cancer-metabolism',
  'profile.created'
);

// Publish message
channel.publish(
  'cancer-metabolism',
  'profile.created',
  Buffer.from(JSON.stringify(profile)),
  {
    persistent: true,
    priority: 5,
    contentType: 'application/json',
    timestamp: Date.now()
  }
);
```

**Routing Keys**:
- `profile.created`
- `profile.updated`
- `profile.deleted`
- `biomarker.evaluated`
- `pathway.enriched`
- `analysis.completed`

## Message Format and Envelope

### Standard Message Envelope

All protocol messages use a standardized envelope structure:

```json
{
  "envelope": {
    "version": "1.0",
    "messageId": "msg_a1b2c3d4e5f6",
    "correlationId": "corr_x1y2z3a4b5c6",
    "timestamp": "2025-03-20T14:30:00.123Z",
    "sender": {
      "institutionId": "INST-001",
      "systemId": "LIMS-001",
      "userId": "user@institution.org"
    },
    "recipient": {
      "institutionId": "INST-002",
      "systemId": "EHR-001"
    },
    "security": {
      "encryption": "AES-256-GCM",
      "signature": "RS256",
      "keyId": "key-2025-03"
    },
    "qos": {
      "priority": "high",
      "ttl": 3600,
      "deliveryMode": "persistent"
    }
  },
  "payload": {
    "type": "MetabolicProfile",
    "action": "create",
    "data": {
      /* Actual metabolic profile data */
    }
  },
  "metadata": {
    "contentType": "application/json",
    "encoding": "utf-8",
    "compression": "gzip",
    "size": 45678
  }
}
```

### Payload Types

| Type | Description | Schema Reference |
|------|-------------|------------------|
| `MetabolicProfile` | Complete metabolic profile | Phase 1 - MetabolicProfile |
| `Biomarker` | Biomarker definition or evaluation | Phase 1 - Biomarker |
| `Metabolite` | Metabolite information | Phase 1 - Metabolite |
| `Pathway` | Metabolic pathway data | Phase 1 - MetabolicPathway |
| `AnalysisRequest` | Request for computational analysis | Custom schema |
| `AnalysisResult` | Analysis output | Custom schema |

### Message Compression

Large payloads should be compressed:

**Supported Algorithms**:
- `gzip` (default, best compatibility)
- `brotli` (better compression, modern clients)
- `zstd` (fastest, best ratio for similar data)

**Example (gzip)**:
```javascript
const zlib = require('zlib');

const payload = JSON.stringify(metabolicProfile);
const compressed = zlib.gzipSync(payload);

const message = {
  envelope: { /* ... */ },
  payload: compressed.toString('base64'),
  metadata: {
    contentType: 'application/json',
    encoding: 'base64',
    compression: 'gzip',
    size: compressed.length,
    originalSize: payload.length
  }
};
```

## Security Protocols

### Transport Layer Security (TLS)

**Requirements**:
- TLS 1.3 mandatory for all new implementations
- TLS 1.2 acceptable for legacy systems (deprecated 2026-01-01)
- TLS 1.1 and below strictly forbidden
- Perfect Forward Secrecy (PFS) required
- Certificate transparency monitoring

**Certificate Management**:
```javascript
{
  "certificate": {
    "type": "X.509 v3",
    "keyAlgorithm": "RSA",
    "keySize": 4096,
    "signatureAlgorithm": "SHA-256 with RSA",
    "validFrom": "2025-01-01T00:00:00Z",
    "validTo": "2026-01-01T00:00:00Z",
    "issuer": "WIA Certificate Authority",
    "subject": "CN=api.wia.org",
    "san": [
      "api.wia.org",
      "*.api.wia.org",
      "stream.wia.org"
    ],
    "ocspStapling": true
  }
}
```

### End-to-End Encryption

For sensitive patient data, implement additional application-layer encryption:

**Algorithm**: AES-256-GCM (Galois/Counter Mode)

**Key Exchange**: ECDH (Elliptic Curve Diffie-Hellman) with P-384

**Encryption Flow**:
```javascript
const crypto = require('crypto');

// Generate ephemeral key pair
const { publicKey, privateKey } = crypto.generateKeyPairSync('ec', {
  namedCurve: 'secp384r1'
});

// Derive shared secret (ECDH)
const sharedSecret = crypto.diffieHellman({
  privateKey: privateKey,
  publicKey: recipientPublicKey
});

// Derive encryption key (HKDF)
const encryptionKey = crypto.hkdfSync(
  'sha256',
  sharedSecret,
  'wia-cancer-metabolism-v1',
  'encryption-key',
  32
);

// Encrypt payload
const iv = crypto.randomBytes(12);
const cipher = crypto.createCipheriv('aes-256-gcm', encryptionKey, iv);

let encrypted = cipher.update(JSON.stringify(payload), 'utf8', 'base64');
encrypted += cipher.final('base64');
const authTag = cipher.getAuthTag();

const encryptedMessage = {
  ciphertext: encrypted,
  iv: iv.toString('base64'),
  authTag: authTag.toString('base64'),
  algorithm: 'AES-256-GCM',
  keyId: 'ephemeral-key-123',
  publicKey: publicKey.export({ type: 'spki', format: 'pem' })
};
```

### Digital Signatures

All messages should be digitally signed to ensure authenticity and integrity:

**Algorithm**: RS256 (RSA Signature with SHA-256)

**Signing Process**:
```javascript
const jwt = require('jsonwebtoken');

const payload = {
  envelope: { /* ... */ },
  payload: { /* ... */ },
  metadata: { /* ... */ }
};

const signature = jwt.sign(
  payload,
  privateKey,
  {
    algorithm: 'RS256',
    keyid: 'signing-key-2025-03',
    expiresIn: '1h'
  }
);

// Send signed message
const signedMessage = {
  message: payload,
  signature: signature,
  signingKeyId: 'signing-key-2025-03',
  algorithm: 'RS256'
};
```

**Verification**:
```javascript
const jwt = require('jsonwebtoken');

try {
  const verified = jwt.verify(
    signedMessage.signature,
    publicKey,
    {
      algorithms: ['RS256']
    }
  );
  // Message is authentic
} catch (error) {
  // Signature verification failed
}
```

### Authentication Protocols

**1. OAuth 2.0 Bearer Tokens** (Primary):
```http
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

**2. Mutual TLS (mTLS)** (Institution-to-institution):
- Client presents certificate during TLS handshake
- Server validates client certificate against trusted CA
- Bidirectional authentication established

**3. API Keys** (Legacy, deprecated):
```http
X-API-Key: wia_live_ak_a1b2c3d4e5f6
```

### Authorization Model

Role-Based Access Control (RBAC) with fine-grained permissions:

**Roles**:
- `researcher`: Read access to anonymized data
- `clinician`: Read/write access to patient-associated data
- `lab-technician`: Create and update metabolic profiles
- `data-analyst`: Read access with bulk export capabilities
- `admin`: Full administrative access

**Permission Matrix**:
```json
{
  "roles": {
    "clinician": {
      "profiles": ["read", "write", "delete"],
      "biomarkers": ["read", "evaluate"],
      "patients": ["read", "write"],
      "metabolites": ["read"],
      "pathways": ["read"]
    },
    "researcher": {
      "profiles": ["read"],
      "biomarkers": ["read"],
      "metabolites": ["read"],
      "pathways": ["read", "analyze"]
    }
  }
}
```

## Real-time Streaming

### WebSocket Streaming Protocol

**Connection Lifecycle**:
1. **Connect**: Client initiates WSS connection with authentication
2. **Subscribe**: Client subscribes to specific data streams
3. **Stream**: Server pushes real-time updates
4. **Heartbeat**: Periodic ping/pong to maintain connection
5. **Disconnect**: Graceful connection termination

**Subscription Message**:
```json
{
  "type": "subscribe",
  "streams": [
    {
      "resource": "profiles",
      "filter": {
        "cancerType": "C50.9",
        "institution": "INST-001"
      }
    },
    {
      "resource": "biomarkers",
      "filter": {
        "category": "diagnostic"
      }
    }
  ],
  "options": {
    "throttle": 1000,
    "batchSize": 10
  }
}
```

**Stream Update**:
```json
{
  "type": "update",
  "stream": "profiles",
  "event": "created",
  "data": {
    "profileId": "a1b2c3d4-e5f6-4a7b-8c9d-0e1f2a3b4c5d",
    "patientId": "PATIENT-BC-00126",
    "cancerType": "C50.9",
    "collectionDate": "2025-03-20T14:30:00Z"
  },
  "timestamp": "2025-03-20T14:30:01.234Z",
  "sequence": 12346
}
```

### Server-Sent Events (SSE)

Alternative to WebSocket for unidirectional streaming:

**Connection**:
```http
GET /stream/profiles HTTP/1.1
Host: stream.wia.org
Accept: text/event-stream
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

**Event Stream**:
```
id: 12346
event: profile.created
data: {"profileId":"a1b2c3d4-e5f6-4a7b-8c9d-0e1f2a3b4c5d","cancerType":"C50.9"}

id: 12347
event: biomarker.evaluated
data: {"biomarkerId":"f6e5d4c3-b2a1-4d5e-6f7a-8b9c0d1e2f3a","value":6.87}
```

## Cross-institutional Data Exchange

### Federation Protocol

Enable secure data sharing between institutions:

**1. Institution Discovery**:
```http
GET /federation/institutions
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

**Response**:
```json
{
  "institutions": [
    {
      "institutionId": "INST-002",
      "name": "City Cancer Institute",
      "endpoint": "https://cancer-metabolism.city-cancer.org/api/v1",
      "publicKey": "-----BEGIN PUBLIC KEY-----\n...",
      "capabilities": [
        "profile-sharing",
        "biomarker-collaboration",
        "federated-analysis"
      ],
      "certifications": ["WIA-CERTIFIED-2025"]
    }
  ]
}
```

**2. Data Sharing Request**:
```http
POST /federation/share
Content-Type: application/json
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...

{
  "requestId": "req_a1b2c3d4",
  "sourceInstitution": "INST-001",
  "targetInstitution": "INST-002",
  "dataType": "MetabolicProfile",
  "profileIds": [
    "a1b2c3d4-e5f6-4a7b-8c9d-0e1f2a3b4c5d"
  ],
  "purpose": "collaborative-research",
  "consent": {
    "patientConsent": true,
    "irbApproval": "IRB-2025-001",
    "dataUseAgreement": "DUA-2025-003"
  },
  "encryption": {
    "algorithm": "AES-256-GCM",
    "keyExchange": "ECDH-P384"
  }
}
```

**3. Secure Transfer**:
- Establish mTLS connection
- Exchange encryption keys via ECDH
- Transfer encrypted data
- Verify digital signature
- Confirm receipt and log transaction

### Audit Logging

All data exchanges must be logged for compliance:

```json
{
  "auditId": "audit_x1y2z3a4",
  "timestamp": "2025-03-20T14:30:00Z",
  "event": "data-shared",
  "actor": {
    "institutionId": "INST-001",
    "userId": "user@inst001.org",
    "role": "researcher"
  },
  "action": "share-profile",
  "resource": {
    "type": "MetabolicProfile",
    "id": "a1b2c3d4-e5f6-4a7b-8c9d-0e1f2a3b4c5d",
    "patientId": "PATIENT-BC-00123"
  },
  "recipient": {
    "institutionId": "INST-002",
    "purpose": "collaborative-research"
  },
  "outcome": "success",
  "compliance": {
    "hipaa": true,
    "gdpr": true,
    "consent": true,
    "irb": "IRB-2025-001"
  }
}
```

## Quality of Service (QoS)

### Message Priority

Three-tier priority system:

**Priority Levels**:
- `critical` (0): Life-threatening alerts, emergency results
- `high` (1): Time-sensitive clinical data, diagnostic results
- `normal` (2): Research data, batch processing

**Implementation**:
```json
{
  "envelope": {
    "qos": {
      "priority": "high",
      "ttl": 1800,
      "deliveryMode": "persistent"
    }
  }
}
```

### Delivery Guarantees

**1. At-Most-Once**: Fire-and-forget (not recommended for clinical data)
**2. At-Least-Once**: Message may be delivered multiple times (idempotency required)
**3. Exactly-Once**: Guaranteed single delivery (use for critical clinical data)

**Idempotency Key**:
```http
POST /profiles
Idempotency-Key: idem_a1b2c3d4e5f6
Content-Type: application/json
```

### Retry Policy

Exponential backoff with jitter:

```javascript
const retry = {
  initialDelay: 1000,      // 1 second
  maxDelay: 30000,         // 30 seconds
  multiplier: 2,
  maxAttempts: 5,
  jitter: 0.1
};

function calculateDelay(attempt) {
  const baseDelay = Math.min(
    retry.initialDelay * Math.pow(retry.multiplier, attempt),
    retry.maxDelay
  );
  const jitter = baseDelay * retry.jitter * Math.random();
  return baseDelay + jitter;
}
```

---
弘益人間 (홍익인간) - Benefit All Humanity
© 2025 WIA Standards | MIT License
