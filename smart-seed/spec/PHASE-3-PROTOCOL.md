# WIA-AGRI-011: Smart Seed Standard
## Phase 3 - Protocol Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-01-01

---

## 1. Overview

This specification defines communication protocols for smart seed traceability, blockchain integration, IoT sensor data collection from seed storage facilities, and interoperability with international seed certification systems.

### 1.1 Protocol Stack

```
┌─────────────────────────────────────────────────────────┐
│            Application Layer (REST API)                 │
├─────────────────────────────────────────────────────────┤
│         Seed Traceability Protocol (STP/1.0)            │
├─────────────────────────────────────────────────────────┤
│    Blockchain Layer (Ethereum/Polygon/Hyperledger)      │
├─────────────────────────────────────────────────────────┤
│          IoT Data Layer (MQTT/CoAP/HTTP)                │
├─────────────────────────────────────────────────────────┤
│              Transport Layer (TLS 1.3)                  │
└─────────────────────────────────────────────────────────┘
```

---

## 2. Seed Traceability Protocol (STP)

### 2.1 Protocol Version

```
STP/1.0
```

### 2.2 Message Format

All STP messages use JSON format with the following structure:

```json
{
  "protocol": "STP/1.0",
  "messageType": "string (enum: REGISTER, UPDATE, CERTIFY, TRANSFER, VERIFY)",
  "messageId": "string (UUID)",
  "timestamp": "string (ISO 8601)",
  "sender": {
    "did": "string (W3C DID)",
    "role": "string (enum: breeder, producer, lab, certifier, distributor)"
  },
  "payload": { },
  "signature": {
    "type": "Ed25519Signature2020",
    "value": "string (base64)"
  }
}
```

### 2.3 Message Types

#### 2.3.1 REGISTER (Variety Registration)

```json
{
  "protocol": "STP/1.0",
  "messageType": "REGISTER",
  "messageId": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": "2025-01-15T10:30:00Z",
  "sender": {
    "did": "did:wia:breeder:nsi-kr",
    "role": "breeder"
  },
  "payload": {
    "varietyName": "Rice Supreme 2025",
    "scientificName": "Oryza sativa L.",
    "parentLines": {
      "maternal": "KR-2024-A",
      "paternal": "KR-2024-B"
    },
    "pvpApplication": {
      "country": "KR",
      "applicationNumber": "PVP-2025-001"
    }
  },
  "signature": {
    "type": "Ed25519Signature2020",
    "value": "z4kM7..."
  }
}
```

#### 2.3.2 UPDATE (Quality Update)

```json
{
  "protocol": "STP/1.0",
  "messageType": "UPDATE",
  "messageId": "660e8400-e29b-41d4-a716-446655440001",
  "timestamp": "2025-01-20T14:00:00Z",
  "sender": {
    "did": "did:wia:lab:kstl",
    "role": "lab"
  },
  "payload": {
    "lotId": "KSC-2025-001",
    "updateType": "germination-test",
    "results": {
      "germinationRate": 95.5,
      "testStandard": "ISTA",
      "testDate": "2025-01-20"
    }
  },
  "signature": {
    "type": "Ed25519Signature2020",
    "value": "z7jK3..."
  }
}
```

#### 2.3.3 CERTIFY (Seed Certification)

```json
{
  "protocol": "STP/1.0",
  "messageType": "CERTIFY",
  "messageId": "770e8400-e29b-41d4-a716-446655440002",
  "timestamp": "2025-02-10T10:00:00Z",
  "sender": {
    "did": "did:wia:agency:ksvs",
    "role": "certifier"
  },
  "payload": {
    "lotId": "KSC-2025-001",
    "certificateId": "CERT-2025-12345",
    "certificationType": "Certified",
    "scheme": "OECD",
    "validUntil": "2026-02-10",
    "complianceStatus": {
      "germination": true,
      "purity": true,
      "overallCompliance": true
    }
  },
  "signature": {
    "type": "Ed25519Signature2020",
    "value": "z9mP5..."
  }
}
```

#### 2.3.4 TRANSFER (Ownership Transfer)

```json
{
  "protocol": "STP/1.0",
  "messageType": "TRANSFER",
  "messageId": "880e8400-e29b-41d4-a716-446655440003",
  "timestamp": "2025-03-01T09:00:00Z",
  "sender": {
    "did": "did:wia:producer:ksc",
    "role": "producer"
  },
  "payload": {
    "lotId": "KSC-2025-001",
    "fromParty": "did:wia:producer:ksc",
    "toParty": "did:wia:distributor:seeds-inc",
    "quantity": { "value": 500, "unit": "kg" },
    "location": "Seoul Distribution Center",
    "transportConditions": {
      "temperature": 5,
      "humidity": 40
    }
  },
  "signature": {
    "type": "Ed25519Signature2020",
    "value": "z2nQ8..."
  }
}
```

#### 2.3.5 VERIFY (Verification Request)

```json
{
  "protocol": "STP/1.0",
  "messageType": "VERIFY",
  "messageId": "990e8400-e29b-41d4-a716-446655440004",
  "timestamp": "2025-03-15T11:30:00Z",
  "sender": {
    "did": "did:wia:farmer:kim-farm",
    "role": "farmer"
  },
  "payload": {
    "lotId": "KSC-2025-001",
    "verificationRequest": {
      "checkCertification": true,
      "checkTraceability": true,
      "checkIntellectualProperty": true
    }
  },
  "signature": {
    "type": "Ed25519Signature2020",
    "value": "z6pR9..."
  }
}
```

---

## 3. Blockchain Integration Protocol

### 3.1 Supported Blockchain Networks

- **Ethereum Mainnet** (for high-value breeding programs)
- **Polygon** (recommended for production use)
- **Hyperledger Fabric** (for private consortium networks)
- **IPFS** (for document storage)

### 3.2 Smart Contract Interface

#### 3.2.1 Variety Registration Contract

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

interface IVarietyRegistry {
    event VarietyRegistered(
        bytes32 indexed varietyId,
        string varietyName,
        address breeder,
        uint256 timestamp
    );

    function registerVariety(
        bytes32 varietyId,
        string memory varietyName,
        string memory scientificName,
        string memory ipfsHash
    ) external returns (bool);

    function getVariety(bytes32 varietyId)
        external view returns (
            string memory varietyName,
            address breeder,
            uint256 registrationDate,
            string memory ipfsHash
        );
}
```

#### 3.2.2 Seed Lot Tracking Contract

```solidity
interface ISeedLotTracker {
    event LotCreated(
        bytes32 indexed lotId,
        bytes32 indexed varietyId,
        address producer,
        uint256 timestamp
    );

    event QualityUpdated(
        bytes32 indexed lotId,
        uint256 germinationRate,
        uint256 purity,
        uint256 timestamp
    );

    event LotCertified(
        bytes32 indexed lotId,
        bytes32 certificateId,
        address certifier,
        uint256 validUntil
    );

    event LotTransferred(
        bytes32 indexed lotId,
        address from,
        address to,
        uint256 quantity,
        uint256 timestamp
    );

    function createLot(
        bytes32 lotId,
        bytes32 varietyId,
        uint256 quantity,
        string memory ipfsHash
    ) external returns (bool);

    function updateQuality(
        bytes32 lotId,
        uint256 germinationRate,
        uint256 purity,
        string memory testReportHash
    ) external returns (bool);

    function certifyLot(
        bytes32 lotId,
        bytes32 certificateId,
        uint256 validUntil
    ) external returns (bool);

    function transferLot(
        bytes32 lotId,
        address recipient,
        uint256 quantity
    ) external returns (bool);

    function getLotHistory(bytes32 lotId)
        external view returns (
            bytes32[] memory eventHashes,
            uint256[] memory timestamps
        );
}
```

### 3.3 Transaction Format

```json
{
  "network": "polygon",
  "contractAddress": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
  "function": "certifyLot",
  "parameters": {
    "lotId": "0x4b736332303235303031000000000000000000000000000000000000000000",
    "certificateId": "0x434552543230323531323334350000000000000000000000000000000000",
    "validUntil": 1739203200
  },
  "from": "0x1234567890123456789012345678901234567890",
  "gas": 150000,
  "gasPrice": "30000000000",
  "nonce": 42
}
```

---

## 4. IoT Sensor Protocol

### 4.1 MQTT Topics

Seed storage facilities SHOULD publish sensor data to MQTT topics:

```
wia/seed/storage/{facilityId}/temperature
wia/seed/storage/{facilityId}/humidity
wia/seed/storage/{facilityId}/door
wia/seed/storage/{facilityId}/alert
```

### 4.2 MQTT Message Format

```json
{
  "facilityId": "STORAGE-KR-001",
  "lotId": "KSC-2025-001",
  "sensorType": "temperature",
  "value": 5.2,
  "unit": "celsius",
  "location": "Zone A, Shelf 3",
  "timestamp": "2025-01-15T10:30:00Z",
  "qualityStatus": "optimal"
}
```

### 4.3 Alert Protocol

```json
{
  "alertType": "TEMPERATURE_EXCEEDS_THRESHOLD",
  "severity": "warning",
  "facilityId": "STORAGE-KR-001",
  "affectedLots": ["KSC-2025-001", "KSC-2025-002"],
  "sensorReading": {
    "temperature": 18.5,
    "threshold": 10.0,
    "unit": "celsius"
  },
  "timestamp": "2025-01-15T15:45:00Z",
  "actionRequired": "Check refrigeration system immediately"
}
```

---

## 5. Interoperability Protocols

### 5.1 ISTA (International Seed Testing Association)

**Endpoint:** `https://ista.api.org/v1`

**Authentication:** OAuth 2.0

**Test Result Submission:**
```json
{
  "testId": "ISTA-TEST-2025-001",
  "lotId": "KSC-2025-001",
  "testStandard": "ISTA-2024",
  "results": {
    "germinationPercentage": 95.5,
    "methodUsed": "Top of Paper (TP)",
    "temperature": "25°C constant",
    "duration": "7 days"
  },
  "laboratory": {
    "istaAccreditationNumber": "ISTA-LAB-KR-001",
    "name": "Korea Seed Testing Lab"
  }
}
```

### 5.2 OECD Seed Schemes

**Endpoint:** `https://oecd-seed.api.org/v2`

**Certification Report:**
```json
{
  "scheme": "OECD-Cereals",
  "varietyCode": "OECD-2025-RIC-001",
  "lotNumber": "KSC-2025-001",
  "generation": "Certified",
  "fieldInspection": {
    "date": "2024-09-15",
    "isolationDistance": 500,
    "varietyPurity": 99.5,
    "offTypes": 0.3
  },
  "seedTesting": {
    "germination": 95.5,
    "purity": 98.2
  }
}
```

### 5.3 UPOV PLUTO Database

**Endpoint:** `https://pluto.upov.int/api/v1`

**Variety Protection Registration:**
```json
{
  "applicationNumber": "PVP-2025-001",
  "varietyDenomination": "Rice Supreme 2025",
  "botanicalTaxon": "Oryza sativa L.",
  "breeder": {
    "name": "National Seed Institute",
    "country": "KR"
  },
  "upovCountry": "KR",
  "dateOfApplication": "2025-01-15"
}
```

---

## 6. Security Protocols

### 6.1 Transport Security

All communications MUST use TLS 1.3 or higher.

```
Cipher Suites (Required):
- TLS_AES_256_GCM_SHA384
- TLS_CHACHA20_POLY1305_SHA256
- TLS_AES_128_GCM_SHA256
```

### 6.2 Digital Signatures

All critical transactions MUST be signed using Ed25519 or ECDSA (secp256k1).

**Signature Process:**
```javascript
const messageHash = sha256(JSON.stringify(payload));
const signature = ed25519.sign(messageHash, privateKey);

const signedMessage = {
  payload: payload,
  signature: {
    type: "Ed25519Signature2020",
    publicKey: "did:wia:producer:ksc#key-1",
    value: base64encode(signature)
  }
};
```

### 6.3 Access Control

Role-based access control (RBAC) using W3C DIDs:

```json
{
  "did": "did:wia:producer:ksc",
  "roles": ["seed-producer", "lot-creator"],
  "permissions": [
    "create:lot",
    "update:lot-quality",
    "transfer:lot"
  ],
  "restrictions": {
    "cannotCertify": true,
    "cannotRegisterVariety": true
  }
}
```

---

## 7. Data Synchronization Protocol

### 7.1 Event Sourcing

All state changes MUST be recorded as immutable events:

```json
{
  "eventId": "evt-2025-001",
  "eventType": "LOT_CREATED",
  "aggregateId": "KSC-2025-001",
  "aggregateType": "SeedLot",
  "timestamp": "2025-01-15T10:30:00Z",
  "version": 1,
  "data": {
    "lotId": "KSC-2025-001",
    "varietyId": "550e8400-e29b-41d4-a716-446655440000",
    "quantity": 1000
  },
  "metadata": {
    "causationId": "req-abc123",
    "correlationId": "corr-xyz789",
    "userId": "did:wia:producer:ksc"
  }
}
```

### 7.2 State Replication

Systems SHOULD implement eventual consistency using:
- **Conflict Resolution:** Last-Write-Wins (LWW) with vector clocks
- **Sync Protocol:** WebSocket or Server-Sent Events (SSE)

---

## 8. QR Code Protocol

### 8.1 QR Code Format

**Data Encoding:** UTF-8 URL

**Example:**
```
https://verify.wiastandards.com/seed/KSC-2025-001?cert=CERT-2025-12345
```

### 8.2 Verification Flow

```
1. Scan QR Code
   ↓
2. HTTP GET to verification URL
   ↓
3. Server retrieves lot data + blockchain proof
   ↓
4. Return verification result with VC
   ↓
5. Display seed passport on mobile device
```

---

## 9. Protocol Versioning

Protocol versions are negotiated using HTTP headers:

```http
WIA-Seed-Protocol-Version: STP/1.0
Accept: application/json; version=1.0
```

Backward compatibility maintained for 2 major versions.

---

## 10. Performance Requirements

- **Latency:** API responses < 200ms (p95)
- **Blockchain Confirmation:** < 30 seconds
- **MQTT Message Delivery:** < 1 second
- **Sync Lag:** < 5 seconds (eventual consistency)

---

**Next Phase:** [Phase 4 - Integration Specification](./PHASE-4-INTEGRATION.md)
