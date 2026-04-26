# WIA-AGRI-015: Food Safety Standard
## Phase 3 - Protocol Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-01-15

---

## 1. Overview

This specification defines communication protocols, data streaming, real-time monitoring, and inter-system messaging for food safety networks.

### 1.1 Protocol Stack

```
Application Layer:  Food Safety Messages (JSON/Protocol Buffers)
Transport Layer:    HTTPS, WebSocket, MQTT
Security Layer:     TLS 1.3, JWT, OAuth 2.0
Data Layer:         Blockchain, IPFS, Traditional DB
```

---

## 2. Safety Alert Protocol

### 2.1 Alert Levels

| Level | Code | Response Time | Notification Channels |
|-------|------|--------------|----------------------|
| Critical | CRIT | Immediate | SMS, Email, Push, Media, Regulatory |
| High | HIGH | < 15 minutes | SMS, Email, Push, Regulatory |
| Medium | MED | < 1 hour | Email, Push |
| Low | LOW | < 24 hours | Email |

### 2.2 Alert Message Format

```json
{
  "protocol": "WIA-FOOD-SAFETY-ALERT",
  "version": "1.0.0",
  "alert": {
    "id": "ALERT-2025-001",
    "timestamp": "2025-01-15T10:00:00Z",
    "severity": "CRIT",
    "type": "contamination_detected",
    "source": {
      "system": "lab-testing-system",
      "facility": "Certified Labs Inc.",
      "inspector": "did:wia:inspector:john-doe"
    },
    "product": {
      "productId": "FS-2025-STR-001",
      "batchNumbers": ["FARM-20250115-001"],
      "affectedQuantity": 5000,
      "unit": "pounds"
    },
    "contamination": {
      "type": "Salmonella",
      "detectionMethod": "PCR testing",
      "concentration": "detected",
      "healthRisk": "Class I - Life-threatening"
    },
    "actions": {
      "immediate": [
        "Cease distribution",
        "Quarantine all affected batches",
        "Notify FDA within 24 hours"
      ],
      "recall": {
        "required": true,
        "scope": "nationwide",
        "consumerMessage": "Do not consume. Return for full refund."
      }
    },
    "recipients": [
      "regulatory:FDA",
      "regulatory:USDA",
      "manufacturer:MFR-12345",
      "distributors:all",
      "retailers:affected-region",
      "consumers:purchased-products",
      "media:press-release"
    ],
    "signature": {
      "algorithm": "Ed25519",
      "publicKey": "did:wia:food-safety#key-1",
      "value": "0x1234abcd..."
    }
  }
}
```

### 2.3 Alert Acknowledgment

```json
{
  "alertId": "ALERT-2025-001",
  "recipient": "did:wia:manufacturer:sunshine-farms",
  "status": "acknowledged",
  "timestamp": "2025-01-15T10:05:00Z",
  "actions_taken": [
    "Distribution halted at 10:02 AM",
    "Retail notification sent at 10:04 AM",
    "FDA notification prepared"
  ],
  "signature": "0xabcd1234..."
}
```

---

## 3. MQTT Protocol for IoT Sensors

### 3.1 Topic Structure

```
wia/food-safety/{facility_id}/{zone}/{sensor_type}/{sensor_id}
```

**Examples:**
```
wia/food-safety/FAC-001/cold-storage/temperature/TEMP-001
wia/food-safety/FAC-001/production/humidity/HUM-002
wia/food-safety/FAC-001/warehouse/contamination/CONT-003
```

### 3.2 Temperature Monitoring Message

```json
{
  "timestamp": "2025-01-15T10:00:00Z",
  "sensorId": "TEMP-001",
  "facilityId": "FAC-001",
  "zone": "cold-storage",
  "reading": {
    "temperature": 4.2,
    "unit": "celsius",
    "criticalLimit": 7.0,
    "status": "normal"
  },
  "metadata": {
    "batteryLevel": 85,
    "signalStrength": -45
  }
}
```

**Publish:**
```bash
mosquitto_pub -t "wia/food-safety/FAC-001/cold-storage/temperature/TEMP-001" \
  -m '{"temperature": 4.2, "status": "normal"}'
```

**Subscribe:**
```bash
mosquitto_sub -t "wia/food-safety/FAC-001/+/+/+"
```

### 3.3 QoS Levels

| QoS | Use Case | Delivery Guarantee |
|-----|----------|-------------------|
| 0 | Non-critical monitoring | At most once |
| 1 | Standard temperature logs | At least once |
| 2 | Critical alerts, CCP data | Exactly once |

---

## 4. Blockchain Protocol

### 4.1 Traceability Transaction

```javascript
// Ethereum Smart Contract Call
contract FoodSafetyTraceability {
  function recordCustodyTransfer(
    bytes32 productId,
    address from,
    address to,
    uint256 timestamp,
    bytes32 temperature,
    bytes signature
  ) public;
}
```

**Transaction Format:**
```json
{
  "from": "0x1234...abcd",
  "to": "0xFoodSafetyContract",
  "data": {
    "method": "recordCustodyTransfer",
    "params": {
      "productId": "0xFS2025STR001",
      "from": "did:wia:producer:sunshine-farms",
      "to": "did:wia:distributor:freshco",
      "timestamp": 1642248000,
      "temperature": "0x04200000",
      "signature": "0x..."
    }
  },
  "gas": 150000,
  "gasPrice": "20000000000"
}
```

### 4.2 IPFS Document Storage

**Store Test Results:**
```bash
ipfs add lab-test-results.pdf
# QmXYZ123...abc
```

**Link in Blockchain:**
```json
{
  "productId": "FS-2025-STR-001",
  "testResults": {
    "ipfsHash": "QmXYZ123...abc",
    "documentType": "lab-test-results",
    "uploadDate": "2025-01-15T14:00:00Z"
  }
}
```

---

## 5. Real-time Streaming Protocol

### 5.1 WebSocket Subscription

```javascript
const ws = new WebSocket('wss://api.wia.org/food-safety/stream');

// Subscribe to temperature monitoring
ws.send(JSON.stringify({
  "action": "subscribe",
  "channel": "temperature-monitoring",
  "filter": {
    "facilityId": "FAC-001",
    "zone": "cold-storage"
  }
}));

// Receive updates
ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  if (data.temperature > 7.0) {
    triggerAlert(data);
  }
};
```

### 5.2 Server-Sent Events (SSE)

```javascript
const eventSource = new EventSource('https://api.wia.org/food-safety/events');

eventSource.addEventListener('temperature-alert', (event) => {
  const alert = JSON.parse(event.data);
  console.log('Temperature Alert:', alert);
});

eventSource.addEventListener('recall-notification', (event) => {
  const recall = JSON.parse(event.data);
  notifyStakeholders(recall);
});
```

---

## 6. Inter-System Messaging

### 6.1 Regulatory Reporting (FDA FSMA)

**HL7 FHIR Format:**
```json
{
  "resourceType": "AdverseEvent",
  "identifier": [{
    "system": "https://fda.gov/fsma",
    "value": "FSMA-2025-001"
  }],
  "event": {
    "coding": [{
      "system": "http://snomed.info/sct",
      "code": "235910000",
      "display": "Food poisoning"
    }]
  },
  "subject": {
    "reference": "Product/FS-2025-STR-001"
  },
  "suspectEntity": [{
    "instance": {
      "display": "Strawberries - Batch FARM-20250115-001"
    }
  }],
  "outcome": "resolved"
}
```

### 6.2 Retailer Integration (EDI)

**EDIFACT Message:**
```
UNH+1+RECADV:D:96A:UN'
BGM+351+REC-2025-001+9'
DTM+137:20250115:102'
RFF+ON:FARM-20250115-001'
NAD+SU+MFR-12345::92'
LIN+1++FS-2025-STR-001:EN'
QTY+12:5000:LB'
UNT+8+1'
```

---

## 7. Security Protocols

### 7.1 TLS Configuration

```nginx
ssl_protocols TLSv1.3;
ssl_ciphers 'ECDHE-ECDSA-AES256-GCM-SHA384:ECDHE-RSA-AES256-GCM-SHA384';
ssl_prefer_server_ciphers on;
ssl_session_cache shared:SSL:10m;
ssl_session_timeout 10m;
```

### 7.2 Message Signing

**Ed25519 Signature:**
```python
from cryptography.hazmat.primitives.asymmetric import ed25519

# Sign alert message
private_key = ed25519.Ed25519PrivateKey.from_private_bytes(key_bytes)
message = json.dumps(alert).encode('utf-8')
signature = private_key.sign(message)

# Verify
public_key = ed25519.Ed25519PublicKey.from_public_bytes(pubkey_bytes)
public_key.verify(signature, message)
```

### 7.3 Access Control

**JWT Claims:**
```json
{
  "sub": "did:wia:manufacturer:sunshine-farms",
  "iss": "https://wia.org",
  "aud": "food-safety-api",
  "exp": 1642248000,
  "iat": 1642244400,
  "roles": ["manufacturer", "haccp-certified"],
  "permissions": [
    "product:create",
    "product:update",
    "monitoring:submit",
    "recall:view"
  ]
}
```

---

## 8. Error Handling

### 8.1 Protocol Errors

| Error Code | Description | Recovery |
|------------|-------------|----------|
| PROTO-001 | Invalid message format | Reject, log error |
| PROTO-002 | Missing signature | Reject, request re-send |
| PROTO-003 | Expired timestamp | Reject, request fresh data |
| PROTO-004 | Unauthorized sender | Reject, alert security |
| PROTO-005 | Network timeout | Retry with exponential backoff |

### 8.2 Retry Policy

```javascript
async function sendAlert(alert, retries = 3) {
  for (let i = 0; i < retries; i++) {
    try {
      const response = await fetch('https://api.wia.org/alerts', {
        method: 'POST',
        body: JSON.stringify(alert),
        timeout: 5000
      });
      if (response.ok) return response;
    } catch (error) {
      if (i === retries - 1) throw error;
      await sleep(Math.pow(2, i) * 1000); // Exponential backoff
    }
  }
}
```

---

## 9. Versioning

### 9.1 Protocol Version Negotiation

```json
{
  "protocol": "WIA-FOOD-SAFETY",
  "supportedVersions": ["1.0.0", "1.1.0"],
  "preferredVersion": "1.1.0"
}
```

### 9.2 Backward Compatibility

- Version 1.x must support all 1.0.0 message types
- New fields are optional, with sensible defaults
- Deprecated fields marked in documentation 6 months before removal

---

**弘益人間 (Hongik Ingan) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*


## Annex E — Implementation Notes for PHASE-3-PROTOCOL

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-3-PROTOCOL.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.
