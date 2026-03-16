# WIA-AGRI-029: Food Security Standard
## Phase 3 - Protocol Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-01-15

---

## 1. Overview

This specification defines communication protocols for food security systems, enabling real-time coordination between regional food authorities, emergency responders, supply chain partners, and international organizations.

### 1.1 Protocol Objectives

- **Real-time Coordination**: Synchronize food security data across multiple jurisdictions
- **Emergency Response**: Rapid mobilization during food crises
- **Supply Chain Synchronization**: Coordinate multi-regional supply operations
- **International Cooperation**: Cross-border food security collaboration

---

## 2. Core Protocols

### 2.1 Food Security Status Synchronization Protocol (FSSSP)

Synchronizes food security status between regional systems.

#### 2.1.1 Protocol Flow

```
┌─────────────┐                    ┌─────────────┐
│  Region A   │                    │  Region B   │
│   System    │                    │   System    │
└──────┬──────┘                    └──────┬──────┘
       │                                  │
       │  1. Sync Request                 │
       │─────────────────────────────────>│
       │                                  │
       │  2. Current Status               │
       │<─────────────────────────────────│
       │                                  │
       │  3. Update Notification          │
       │─────────────────────────────────>│
       │                                  │
       │  4. Acknowledgment               │
       │<─────────────────────────────────│
       │                                  │
```

#### 2.1.2 Message Format

**Sync Request:**
```json
{
  "protocolVersion": "1.0",
  "messageType": "SYNC_REQUEST",
  "messageId": "msg-a1b2c3d4",
  "timestamp": "2025-01-15T14:30:00Z",
  "source": {
    "regionId": "ASIA-KR-SEOUL",
    "systemId": "sys-seoul-001",
    "did": "did:wia:region:asia-kr-seoul"
  },
  "target": {
    "regionId": "ASIA-KR-BUSAN",
    "systemId": "sys-busan-001"
  },
  "syncScope": ["reserves", "supply_chain", "emergency_status"],
  "lastSyncTimestamp": "2025-01-15T14:00:00Z"
}
```

**Status Response:**
```json
{
  "protocolVersion": "1.0",
  "messageType": "SYNC_RESPONSE",
  "messageId": "msg-b2c3d4e5",
  "inReplyTo": "msg-a1b2c3d4",
  "timestamp": "2025-01-15T14:30:05Z",
  "source": {
    "regionId": "ASIA-KR-BUSAN",
    "systemId": "sys-busan-001"
  },
  "data": {
    "reserves": {
      "rice": 30000,
      "wheat": 15000,
      "lastUpdated": "2025-01-15T14:00:00Z"
    },
    "foodSecurityIndex": 79.5,
    "alerts": [],
    "emergencyStatus": "normal"
  },
  "signature": "base64_encoded_signature"
}
```

#### 2.1.3 Synchronization Frequency

- **Normal Mode**: Every 6 hours
- **Monitoring Mode**: Every 1 hour
- **Emergency Mode**: Every 15 minutes
- **Critical Mode**: Real-time (immediate push)

---

### 2.2 Emergency Alert Protocol (EAP)

Broadcasts food security emergencies to relevant stakeholders.

#### 2.2.1 Alert Levels

| Level | Severity | Response Time | Notification Scope |
|-------|----------|---------------|-------------------|
| 1 | Advisory | 24 hours | Regional authorities |
| 2 | Watch | 6 hours | National + neighboring regions |
| 3 | Warning | 2 hours | National + international partners |
| 4 | Emergency | 30 minutes | Global emergency response network |

#### 2.2.2 Alert Message Format

```json
{
  "protocolVersion": "1.0",
  "messageType": "EMERGENCY_ALERT",
  "messageId": "alert-e1f2g3h4",
  "timestamp": "2025-01-15T14:30:00Z",
  "alertLevel": 3,
  "source": {
    "regionId": "ASIA-KR-AFFECTED",
    "authorityName": "Ministry of Food Security",
    "did": "did:wia:authority:kr-mofs"
  },
  "emergency": {
    "emergencyId": "emerg-234567890",
    "type": "drought",
    "severity": "high",
    "affectedArea": {
      "regionIds": ["ASIA-KR-AFFECTED"],
      "population": 500000,
      "coordinates": {
        "center": {"lat": 37.5665, "lon": 126.9780},
        "radius": 50
      }
    },
    "impact": {
      "foodInsecure": 150000,
      "reservesAffected": 15000,
      "estimatedDuration": 90
    },
    "requestedSupport": {
      "food": {
        "rice": 3000,
        "grain": 2000
      },
      "cash": 5000000,
      "logistics": true
    }
  },
  "validFrom": "2025-01-15T14:30:00Z",
  "validUntil": "2025-01-15T20:30:00Z",
  "priority": "urgent",
  "signature": "base64_encoded_signature"
}
```

#### 2.2.3 Alert Acknowledgment

```json
{
  "protocolVersion": "1.0",
  "messageType": "ALERT_ACK",
  "messageId": "ack-h4i5j6k7",
  "inReplyTo": "alert-e1f2g3h4",
  "timestamp": "2025-01-15T14:35:00Z",
  "source": {
    "regionId": "ASIA-KR-SEOUL",
    "systemId": "sys-seoul-001"
  },
  "acknowledgment": {
    "received": true,
    "responseStatus": "mobilizing",
    "commitments": {
      "food": {
        "rice": 1000,
        "grain": 500
      },
      "cash": 1000000,
      "personnel": 50
    },
    "eta": "2025-01-16T08:00:00Z"
  },
  "signature": "base64_encoded_signature"
}
```

---

### 2.3 Supply Chain Coordination Protocol (SCCP)

Coordinates multi-regional food supply operations.

#### 2.3.1 Transfer Request

```json
{
  "protocolVersion": "1.0",
  "messageType": "TRANSFER_REQUEST",
  "messageId": "trans-k7l8m9n0",
  "timestamp": "2025-01-15T14:30:00Z",
  "source": {
    "regionId": "ASIA-KR-SEOUL",
    "facilityId": "fac-seoul-001",
    "did": "did:wia:facility:seoul-001"
  },
  "target": {
    "regionId": "ASIA-KR-BUSAN",
    "facilityId": "fac-busan-001"
  },
  "transfer": {
    "transferId": "tf-123456789",
    "foodType": "rice",
    "quantity": 5000,
    "unit": "metric_tons",
    "urgency": "normal",
    "purpose": "reserve_replenishment",
    "preferredTransport": "rail",
    "requestedDeliveryDate": "2025-01-20"
  },
  "traceability": {
    "blockchain": true,
    "certificationRequired": ["HACCP", "ISO 22000"]
  },
  "signature": "base64_encoded_signature"
}
```

#### 2.3.2 Transfer Confirmation

```json
{
  "protocolVersion": "1.0",
  "messageType": "TRANSFER_CONFIRMATION",
  "messageId": "conf-n0o1p2q3",
  "inReplyTo": "trans-k7l8m9n0",
  "timestamp": "2025-01-15T15:00:00Z",
  "source": {
    "regionId": "ASIA-KR-BUSAN",
    "facilityId": "fac-busan-001"
  },
  "confirmation": {
    "transferId": "tf-123456789",
    "status": "accepted",
    "confirmedQuantity": 5000,
    "estimatedDeparture": "2025-01-18T08:00:00Z",
    "estimatedArrival": "2025-01-20T16:00:00Z",
    "transportDetails": {
      "mode": "rail",
      "trackingId": "RAIL-98765",
      "vehicleId": "TRAIN-456"
    },
    "blockchainTxHash": "0xabc123def456..."
  },
  "signature": "base64_encoded_signature"
}
```

#### 2.3.3 Transfer Tracking Updates

```json
{
  "protocolVersion": "1.0",
  "messageType": "TRANSFER_UPDATE",
  "messageId": "upd-q3r4s5t6",
  "timestamp": "2025-01-19T12:00:00Z",
  "transfer": {
    "transferId": "tf-123456789",
    "status": "in_transit",
    "currentLocation": {
      "lat": 36.3504,
      "lon": 127.3845,
      "description": "Daejeon Junction"
    },
    "progress": 60,
    "estimatedArrival": "2025-01-20T14:00:00Z",
    "conditions": {
      "temperature": 15.5,
      "humidity": 55,
      "quality": "good"
    }
  },
  "signature": "base64_encoded_signature"
}
```

---

### 2.4 International Aid Coordination Protocol (IACP)

Coordinates international food aid during emergencies.

#### 2.4.1 Aid Request

```json
{
  "protocolVersion": "1.0",
  "messageType": "AID_REQUEST",
  "messageId": "aid-t6u7v8w9",
  "timestamp": "2025-01-15T14:30:00Z",
  "requestor": {
    "countryCode": "KR",
    "regionId": "ASIA-KR-AFFECTED",
    "authorityName": "Ministry of Food Security",
    "did": "did:wia:authority:kr-mofs"
  },
  "emergency": {
    "emergencyId": "emerg-234567890",
    "type": "drought",
    "severity": "level_3",
    "affectedPopulation": 500000
  },
  "aidRequest": {
    "food": {
      "rice": 10000,
      "wheat": 5000,
      "nutritionSupplements": 2000
    },
    "cash": 20000000,
    "technicalSupport": {
      "waterPurification": true,
      "emergencyLogistics": true
    },
    "urgency": "high",
    "deliveryDeadline": "2025-02-15"
  },
  "distributionPlan": {
    "method": "direct_distribution",
    "targetBeneficiaries": 500000,
    "deliveryPoints": 50
  },
  "signature": "base64_encoded_signature"
}
```

#### 2.4.2 Aid Commitment

```json
{
  "protocolVersion": "1.0",
  "messageType": "AID_COMMITMENT",
  "messageId": "com-w9x0y1z2",
  "inReplyTo": "aid-t6u7v8w9",
  "timestamp": "2025-01-16T10:00:00Z",
  "donor": {
    "organization": "UN World Food Programme",
    "countryCode": "INT",
    "did": "did:wia:org:unwfp"
  },
  "commitment": {
    "food": {
      "rice": 5000,
      "wheat": 2000
    },
    "cash": 5000000,
    "technicalSupport": {
      "emergencyLogistics": true,
      "personnel": 20
    },
    "deliveryTimeline": {
      "estimatedArrival": "2025-01-25",
      "deliveryMethod": "air_freight"
    },
    "conditions": [
      "Distribution transparency reports required",
      "Beneficiary verification system must be in place"
    ]
  },
  "signature": "base64_encoded_signature"
}
```

---

### 2.5 Blockchain Integration Protocol (BIP)

Records critical food security transactions on blockchain.

#### 2.5.1 Reserve Transaction Recording

```json
{
  "protocolVersion": "1.0",
  "messageType": "BLOCKCHAIN_RECORD",
  "messageId": "bc-z2a3b4c5",
  "timestamp": "2025-01-15T14:30:00Z",
  "transaction": {
    "type": "reserve_update",
    "reserveId": "a1b2c3d4-5678-90ab-cdef-1234567890ab",
    "regionId": "ASIA-KR-SEOUL",
    "foodType": "rice",
    "action": "addition",
    "quantity": 5000,
    "previousQuantity": 45000,
    "newQuantity": 50000,
    "source": "domestic_production",
    "certifications": ["HACCP", "ISO 22000"]
  },
  "blockchain": {
    "network": "WIA-FoodChain",
    "contractAddress": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb1",
    "gasPrice": "20 gwei",
    "estimatedFee": "0.001 ETH"
  },
  "participants": [
    {
      "role": "supplier",
      "did": "did:wia:supplier:farm-001",
      "signature": "sig_supplier"
    },
    {
      "role": "receiver",
      "did": "did:wia:facility:seoul-001",
      "signature": "sig_receiver"
    }
  ]
}
```

#### 2.5.2 Blockchain Confirmation

```json
{
  "protocolVersion": "1.0",
  "messageType": "BLOCKCHAIN_CONFIRMATION",
  "messageId": "bconf-c5d6e7f8",
  "inReplyTo": "bc-z2a3b4c5",
  "timestamp": "2025-01-15T14:32:00Z",
  "blockchain": {
    "network": "WIA-FoodChain",
    "transactionHash": "0xabc123def456...",
    "blockNumber": 12345678,
    "confirmations": 12,
    "status": "confirmed",
    "gasUsed": 65000,
    "finalFee": "0.0013 ETH"
  },
  "verification": {
    "merkleProof": "proof_data_here",
    "timestamp": "2025-01-15T14:31:30Z",
    "verifiable": true
  }
}
```

---

## 3. Protocol Security

### 3.1 Message Authentication

All protocol messages MUST be signed using one of:
- **ECDSA** (secp256k1 or secp256r1)
- **EdDSA** (Ed25519)
- **RSA** (minimum 2048-bit)

### 3.2 Message Encryption

Sensitive protocol messages SHOULD be encrypted using:
- **TLS 1.3** for transport
- **AES-256-GCM** for message-level encryption

### 3.3 Signature Verification

```javascript
// Example signature verification
const message = JSON.stringify(protocolMessage);
const signature = protocolMessage.signature;
const publicKey = await resolveDidToPublicKey(protocolMessage.source.did);

const isValid = await verifySignature(message, signature, publicKey);
if (!isValid) {
  throw new Error('Invalid message signature');
}
```

---

## 4. Protocol Error Handling

### 4.1 Error Response Format

```json
{
  "protocolVersion": "1.0",
  "messageType": "ERROR",
  "messageId": "err-f8g9h0i1",
  "inReplyTo": "msg-a1b2c3d4",
  "timestamp": "2025-01-15T14:30:00Z",
  "error": {
    "code": "INSUFFICIENT_RESERVES",
    "message": "Cannot fulfill transfer request due to insufficient reserves",
    "details": {
      "requested": 5000,
      "available": 3000,
      "minimumRequired": 40000
    },
    "severity": "high",
    "retryable": false
  },
  "signature": "base64_encoded_signature"
}
```

### 4.2 Common Error Codes

- `INVALID_MESSAGE`: Malformed protocol message
- `AUTHENTICATION_FAILED`: Invalid signature or credentials
- `INSUFFICIENT_RESERVES`: Not enough food reserves
- `REGION_NOT_FOUND`: Unknown region identifier
- `PROTOCOL_VERSION_MISMATCH`: Incompatible protocol versions
- `TIMEOUT`: Response not received within expected timeframe

---

## 5. Protocol Compliance

### 5.1 Mandatory Features

All implementations MUST support:
- Food Security Status Synchronization Protocol (FSSSP)
- Emergency Alert Protocol (EAP)
- Message authentication and signature verification
- Error handling

### 5.2 Optional Features

Implementations MAY support:
- Supply Chain Coordination Protocol (SCCP)
- International Aid Coordination Protocol (IACP)
- Blockchain Integration Protocol (BIP)
- Real-time WebSocket connections

---

## 6. Protocol Testing

### 6.1 Conformance Tests

```bash
# Test FSSSP implementation
wia-protocol-test fsssp --endpoint https://api.region.example.com

# Test Emergency Alert Protocol
wia-protocol-test eap --simulate-emergency --level 3

# Full protocol suite test
wia-protocol-test all --verbose
```

### 6.2 Mock Protocol Server

```javascript
import { FoodSecurityProtocolServer } from '@wia/protocol-test';

const mockServer = new FoodSecurityProtocolServer({
  port: 8080,
  protocols: ['FSSSP', 'EAP', 'SCCP'],
  simulateLatency: true
});

await mockServer.start();
```

---

## 7. Integration Examples

### 7.1 Node.js Implementation

```javascript
import { FoodSecurityProtocol } from '@wia/food-security-protocol';

const protocol = new FoodSecurityProtocol({
  regionId: 'ASIA-KR-SEOUL',
  did: 'did:wia:region:asia-kr-seoul',
  privateKey: process.env.PRIVATE_KEY
});

// Listen for emergency alerts
protocol.on('emergency.alert', async (alert) => {
  console.log(`Emergency alert received: ${alert.emergency.type}`);

  // Send acknowledgment
  await protocol.sendAcknowledgment(alert.messageId, {
    responseStatus: 'mobilizing',
    commitments: {
      food: { rice: 1000 },
      personnel: 20
    }
  });
});

// Request supply chain transfer
const transfer = await protocol.requestTransfer({
  targetRegion: 'ASIA-KR-BUSAN',
  foodType: 'rice',
  quantity: 5000,
  urgency: 'normal'
});
```

---

**Document Status**: ✅ Complete
**Next Phase**: [PHASE-4-INTEGRATION.md](./PHASE-4-INTEGRATION.md)

---

© 2025 WIA Standards | MIT License
弘익人間 · Benefit All Humanity
