# WIA-ORGAN-SHORTAGE - Phase 3: Protocol

> **Version:** 1.0.0
> **Status:** Complete
> **弘益人間 (Hongik Ingan)** - Benefit All Humanity

## 1. Overview

This document defines the real-time communication protocols for the WIA-ORGAN-SHORTAGE standard, enabling urgent organ availability notifications and time-critical matching operations.

## 2. Protocol Stack

```
┌─────────────────────────────────────┐
│         Application Layer           │
│   (WIA-ORGAN-SHORTAGE Messages)     │
├─────────────────────────────────────┤
│         Security Layer              │
│   (TLS 1.3 + End-to-End Encryption) │
├─────────────────────────────────────┤
│         Transport Layer             │
│   (WebSocket / gRPC / HTTPS)        │
├─────────────────────────────────────┤
│         Network Layer               │
│   (TCP/IP)                          │
└─────────────────────────────────────┘
```

## 3. Message Format

### 3.1 Message Header

```json
{
  "header": {
    "version": "WIA-ORGAN-SHORTAGE/1.0",
    "messageId": "msg_550e8400-e29b-41d4-a716-446655440000",
    "type": "organ.available",
    "timestamp": "2025-01-05T10:30:00.000Z",
    "source": {
      "id": "opo_northeast",
      "type": "organ_procurement_organization"
    },
    "destination": {
      "id": "broadcast",
      "type": "all_transplant_centers"
    },
    "priority": "critical",
    "ttl": 14400
  }
}
```

### 3.2 Message Types

| Type | Priority | Description |
|------|----------|-------------|
| `organ.available` | critical | New organ available |
| `organ.accepted` | high | Organ accepted by center |
| `organ.declined` | normal | Organ declined |
| `match.request` | high | Match calculation request |
| `match.result` | high | Match calculation result |
| `eligibility.check` | normal | Eligibility verification |
| `eligibility.result` | normal | Eligibility response |
| `alert.urgent` | critical | Urgent system alert |
| `heartbeat.ping` | low | Connection keepalive |
| `heartbeat.pong` | low | Keepalive response |

### 3.3 Priority Levels

| Level | Response Time | Use Case |
|-------|---------------|----------|
| critical | < 1 minute | Organ availability, urgent alerts |
| high | < 5 minutes | Match results, acceptances |
| normal | < 30 minutes | Eligibility checks, status updates |
| low | Best effort | Heartbeats, analytics |

## 4. WebSocket Protocol

### 4.1 Connection

```javascript
const ws = new WebSocket('wss://stream.organ-shortage.wia.org/v1/ws',
  ['wia-organ-shortage-v1']);

ws.onopen = () => {
  // Authenticate
  ws.send(JSON.stringify({
    header: {
      version: 'WIA-ORGAN-SHORTAGE/1.0',
      messageId: 'auth_' + Date.now(),
      type: 'auth.request',
      timestamp: new Date().toISOString()
    },
    payload: {
      token: 'Bearer <access_token>',
      subscriptions: ['organ.available', 'match.result']
    }
  }));
};
```

### 4.2 Subscription Management

```json
{
  "header": {
    "type": "subscription.add"
  },
  "payload": {
    "events": ["organ.available"],
    "filters": {
      "organ_types": ["kidney", "liver"],
      "regions": ["Region 5", "Region 7"],
      "blood_types": ["O", "A"]
    }
  }
}
```

### 4.3 Organ Availability Notification

```json
{
  "header": {
    "version": "WIA-ORGAN-SHORTAGE/1.0",
    "messageId": "msg_organ_001",
    "type": "organ.available",
    "timestamp": "2025-01-05T10:30:00Z",
    "priority": "critical",
    "ttl": 14400
  },
  "payload": {
    "organ": {
      "type": "kidney",
      "side": "left",
      "donor_id": "donor_anon_789",
      "blood_type": "O",
      "hla_typing": {
        "classI": { "A": ["02:01", "24:02"], "B": ["07:02", "35:01"] },
        "classII": { "DR": ["15:01", "03:01"] }
      }
    },
    "donor_info": {
      "age": 45,
      "cause_of_death": "CVA",
      "kdpi": 35,
      "creatinine": 1.1
    },
    "logistics": {
      "opo": "opo_northeast",
      "location": "Boston, MA",
      "procurement_time": "2025-01-05T12:00:00Z",
      "cold_ischemia_limit_hours": 24
    },
    "expires_at": "2025-01-05T14:30:00Z"
  },
  "security": {
    "encryption": "AES-256-GCM",
    "signature": "sha256=abc123..."
  }
}
```

## 5. gRPC Protocol

### 5.1 Service Definition

```protobuf
syntax = "proto3";

package wia.organ_shortage.v1;

service OrganShortageService {
  // Streaming organ availability
  rpc StreamOrganAvailability(StreamRequest) returns (stream OrganAvailableEvent);

  // Run matching algorithm
  rpc RunMatch(MatchRequest) returns (MatchResponse);

  // Check eligibility
  rpc CheckEligibility(EligibilityRequest) returns (EligibilityResponse);

  // Bidirectional streaming for real-time updates
  rpc LiveUpdates(stream ClientMessage) returns (stream ServerMessage);
}

message OrganAvailableEvent {
  string organ_id = 1;
  string organ_type = 2;
  string blood_type = 3;
  HLATyping hla = 4;
  DonorInfo donor = 5;
  LogisticsInfo logistics = 6;
  google.protobuf.Timestamp expires_at = 7;
}

message MatchRequest {
  string patient_id = 1;
  string organ_id = 2;
  bool include_xenotransplant = 3;
}

message MatchResponse {
  int32 score = 1;
  bool compatible = 2;
  repeated string reasons = 3;
}
```

## 6. Security

### 6.1 Transport Security

- TLS 1.3 required for all connections
- Certificate pinning recommended
- Perfect Forward Secrecy (PFS) enabled

### 6.2 Message Security

```json
{
  "security": {
    "encryption": "AES-256-GCM",
    "keyId": "key_2025_01",
    "iv": "base64_encoded_iv",
    "signature": "Ed25519_signature",
    "signedFields": ["header", "payload"]
  }
}
```

### 6.3 PHI Protection

All Protected Health Information (PHI) must be:
- Encrypted at rest and in transit
- Anonymized where possible
- Access logged for HIPAA compliance

## 7. Reliability

### 7.1 Message Acknowledgment

```json
{
  "header": {
    "type": "message.ack"
  },
  "payload": {
    "messageId": "msg_organ_001",
    "status": "received",
    "timestamp": "2025-01-05T10:30:01Z"
  }
}
```

### 7.2 Retry Policy

| Attempt | Delay | Max Time |
|---------|-------|----------|
| 1 | 1s | 1s |
| 2 | 2s | 3s |
| 3 | 4s | 7s |
| 4 | 8s | 15s |
| 5 | 16s | 31s |

### 7.3 Dead Letter Queue

Failed messages after max retries are sent to DLQ for manual review.

## 8. Flow Diagrams

### 8.1 Organ Availability Flow

```
OPO                  WIA Platform              Transplant Centers
 │                        │                           │
 │──organ.available──────>│                           │
 │                        │──broadcast────────────────>│
 │                        │                           │
 │                        │<──match.request───────────│
 │                        │                           │
 │                        │──match.result─────────────>│
 │                        │                           │
 │<──organ.accepted───────│<──organ.accept────────────│
 │                        │                           │
```

### 8.2 Xenotransplant Eligibility Flow

```
Patient System          WIA Platform              Trial Registry
      │                      │                          │
      │──eligibility.check──>│                          │
      │                      │──trial.query────────────>│
      │                      │<──trial.available────────│
      │<──eligibility.result─│                          │
      │                      │                          │
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
