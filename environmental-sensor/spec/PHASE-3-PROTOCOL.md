# WIA Environmental Sensor Standard - Phase 3: Communication Protocol
## Version 1.0.0 | WIA-ENE-027-PHASE-3

**Status:** Final
**Published:** 2025-01-01
**Organization:** World Certification Industry Association (WIA)
**License:** MIT License

---

## 1. Overview

Phase 3 defines communication protocols for environmental sensor data transmission from devices to gateways and cloud platforms. This specification covers MQTT, CoAP, LoRaWAN, and HTTP protocols with standardized message formats, topic structures, and security requirements.

### 1.1 Scope

- MQTT topic structures and message formats
- CoAP resource paths and observe patterns
- LoRaWAN payload encoding and decoding
- HTTP/REST communication patterns
- Transport layer security (TLS/DTLS)
- Device authentication mechanisms
- Edge computing and gateway patterns

---

## 2. MQTT Protocol Specification

### 2.1 Broker Requirements

- MQTT 5.0 support (MQTT 3.1.1 minimum)
- TLS 1.3 encryption (production)
- Client authentication (certificates or username/password)
- Quality of Service (QoS) 0, 1, and 2 support

### 2.2 Topic Structure

**Standard Topic Pattern:**
```
wia/{standard}/{region}/{location}/{deviceId}/{messageType}
```

**Examples:**
```
wia/env027/us-west/seattle/ENV-AIR-001/data
wia/env027/us-west/seattle/ENV-AIR-001/status
wia/env027/us-west/seattle/ENV-AIR-001/calibration
wia/env027/kr-seoul/gangnam/ENV-WATER-042/data
```

**Wildcard Subscriptions:**
```
wia/env027/us-west/+/+/data          # All data from US West
wia/env027/+/+/ENV-AIR-001/#         # All messages from sensor
wia/env027/+/+/+/status              # All status messages
```

### 2.3 Message Types

| Type | Topic Suffix | Purpose | QoS Recommendation |
|------|--------------|---------|-------------------|
| data | `/data` | Sensor measurements | QoS 1 |
| status | `/status` | Device health | QoS 1 |
| calibration | `/calibration` | Calibration data | QoS 2 |
| command | `/command` | Control commands | QoS 2 |
| alert | `/alert` | Threshold alerts | QoS 1 |

### 2.4 Payload Format

**Data Message:**
```json
{
  "version": "1.0.0",
  "standard": "WIA-ENE-027",
  "deviceId": "ENV-AIR-001",
  "timestamp": "2024-12-26T10:30:00.000Z",
  "sensorType": "air_quality",
  "readings": {
    "pm2_5": {"value": 15.3, "unit": "μg/m³"}
  }
}
```

**Status Message:**
```json
{
  "deviceId": "ENV-AIR-001",
  "timestamp": "2024-12-26T10:30:00Z",
  "status": "online",
  "battery": 85,
  "signalStrength": -65,
  "uptime": 86400
}
```

### 2.5 Connection Management

**Client ID:** `wia-{deviceId}-{randomString}`

**Keep-Alive:** 60-300 seconds (based on reporting frequency)

**Clean Session:**
- `false` for critical data (persists subscriptions)
- `true` for stateless sensors

**Last Will and Testament (LWT):**
```json
{
  "topic": "wia/env027/us-west/seattle/ENV-AIR-001/status",
  "payload": {
    "deviceId": "ENV-AIR-001",
    "status": "offline",
    "timestamp": "2024-12-26T10:30:00Z"
  },
  "qos": 1,
  "retain": true
}
```

---

## 3. CoAP Protocol Specification

### 3.1 Resource Structure

**Base URI:**
```
coap://sensor.local/wia/env/{resource}
```

**Standard Resources:**
```
/wia/env/data          # Current measurements
/wia/env/config        # Sensor configuration
/wia/env/status        # Device status
/wia/env/calibration   # Calibration data
```

### 3.2 Resource Discovery

```
GET coap://sensor.local/.well-known/core

Response:
</wia/env/data>;ct=50;rt="wia.env.data";obs,
</wia/env/status>;ct=50;rt="wia.env.status";obs
```

(ct=50 indicates JSON content type, obs indicates Observable resource)

### 3.3 CoAP Methods

| Method | Usage | Example |
|--------|-------|---------|
| GET | Retrieve data | `GET /wia/env/data` |
| POST | Submit data | `POST /wia/env/data` |
| PUT | Update config | `PUT /wia/env/config` |
| OBSERVE | Subscribe | `GET /wia/env/data` + Observe:0 |

### 3.4 Observe Pattern

**Client Registration:**
```
GET coap://sensor.local/wia/env/data
Observe: 0
```

**Server Responses:**
```
Response 1 (seq=0): {"pm2_5": 15.3, "timestamp": "..."}
Response 2 (seq=1): {"pm2_5": 15.5, "timestamp": "..."}
Response 3 (seq=2): {"pm2_5": 15.2, "timestamp": "..."}
```

**Cancel Observation:**
```
GET coap://sensor.local/wia/env/data
Observe: 1
```

### 3.5 DTLS Security

- DTLS 1.3 for encrypted communication
- Pre-Shared Key (PSK) or certificate-based authentication
- Cipher suites: TLS_AES_128_GCM_SHA256, TLS_CHACHA20_POLY1305_SHA256

---

## 4. LoRaWAN Protocol Specification

### 4.1 Architecture

```
Sensors → LoRaWAN Gateways → Network Server → Application Server
```

### 4.2 Device Classes

- **Class A:** Bi-directional, lowest power (recommended for most sensors)
- **Class B:** Scheduled receive windows (for time-critical applications)
- **Class C:** Continuous receive (for powered sensors)

### 4.3 Payload Encoding

**Compact Binary Format (for bandwidth efficiency):**

```
Air Quality Sensor Payload (17 bytes):

Byte 0:      Message Type (0x01 = data)
Byte 1-4:    Device ID hash (4 bytes)
Byte 5-8:    Unix timestamp (4 bytes)
Byte 9-10:   PM2.5 (uint16, scale 0.1 μg/m³)
Byte 11-12:  PM10 (uint16, scale 0.1 μg/m³)
Byte 13-14:  Temperature (int16, scale 0.01 °C)
Byte 15:     Humidity (uint8, scale 1 %RH)
Byte 16:     Battery (uint8, scale 1 %)
```

**Decoding Function (pseudo-code):**
```javascript
function decodePayload(bytes) {
  return {
    deviceId: lookupDeviceId(bytes.slice(1, 5)),
    timestamp: new Date(readUint32(bytes, 5) * 1000),
    readings: {
      pm2_5: {value: readUint16(bytes, 9) * 0.1, unit: "μg/m³"},
      pm10: {value: readUint16(bytes, 11) * 0.1, unit: "μg/m³"},
      temperature: {value: readInt16(bytes, 13) * 0.01, unit: "°C"},
      humidity: {value: bytes[15], unit: "%RH"}
    },
    metadata: {battery: bytes[16]}
  };
}
```

### 4.4 Adaptive Data Rate

| Data Rate | Spreading Factor | Max Payload | WIA Parameters |
|-----------|------------------|-------------|----------------|
| DR0 | SF12 | 51 bytes | Core only |
| DR1 | SF11 | 51 bytes | Core + status |
| DR2 | SF10 | 115 bytes | Standard set |
| DR3 | SF9 | 222 bytes | Extended set |
| DR4-DR5 | SF7-SF8 | 222 bytes | Full suite |

### 4.5 Port Mapping

| FPort | Purpose | Format |
|-------|---------|--------|
| 1 | Measurement data | Binary (compact) |
| 2 | Status/diagnostics | Binary |
| 10 | Configuration | Binary |
| 100 | JSON (debug/testing) | JSON string |

---

## 5. HTTP/REST Protocol

### 5.1 Data Submission

```http
POST /api/v1/sensors/{deviceId}/data
Content-Type: application/json
Authorization: Bearer {token}

{WIA Phase 1 JSON payload}
```

### 5.2 Polling Pattern

Sensors can poll for commands:

```http
GET /api/v1/sensors/{deviceId}/commands
Authorization: Bearer {token}

Response:
{
  "commands": [
    {"id": "cmd-123", "action": "calibrate", "parameters": {}}
  ]
}
```

---

## 6. Security Requirements

### 6.1 Transport Security

| Protocol | Security Mechanism | Required |
|----------|-------------------|----------|
| MQTT | TLS 1.3 (MQTTS) | Yes (production) |
| CoAP | DTLS 1.3 (CoAPS) | Yes (production) |
| HTTP | TLS 1.3 (HTTPS) | Yes |
| LoRaWAN | AES-128 encryption | Built-in |

### 6.2 Device Authentication

**Certificate-Based (Recommended):**
- X.509 certificates
- Mutual TLS (mTLS)
- Certificate rotation every 1-2 years

**Pre-Shared Key:**
- Unique per-device keys
- Minimum 128-bit strength
- Secure key provisioning

**Username/Password:**
- Strong passwords (16+ characters)
- Hashed storage (bcrypt, Argon2)
- Rate limiting on failed attempts

### 6.3 Message Integrity

**Optional Digital Signatures:**
```json
{
  "deviceId": "ENV-AIR-001",
  "readings": {...},
  "signature": {
    "algorithm": "Ed25519",
    "publicKey": "base64...==",
    "value": "base64signature=="
  }
}
```

---

## 7. Edge Computing Patterns

### 7.1 Gateway Responsibilities

- Protocol translation (sensor protocol → cloud API)
- Data aggregation and filtering
- Local anomaly detection
- Buffering during connectivity loss
- Edge analytics

### 7.2 Data Processing Pipeline

```
Raw Sensor Data → Validation → Filtering → Aggregation → Forwarding
                      ↓           ↓           ↓            ↓
                   Discard    Remove      Compute      To Cloud
                   Invalid    Outliers    Statistics   (Reduced)
```

### 7.3 Local Decision Making

Gateways can trigger local actions:
- Threshold alerts (sound alarm, send SMS)
- Actuator control (close valve, adjust fan)
- Adaptive sampling (increase frequency on events)

---

## 8. Compliance Requirements

### 8.1 Minimum Requirements

- Support at least one protocol (MQTT, CoAP, LoRaWAN, or HTTP)
- Implement security (TLS/DTLS, authentication)
- Use WIA Phase 1 data format (JSON or binary equivalent)
- Proper error handling and reconnection logic

### 8.2 Recommended Features

- MQTT for connected sensors
- CoAP for battery-powered sensors
- LoRaWAN for remote deployments
- Edge processing for bandwidth reduction
- Multiple QoS levels for reliability

---

## 9. Testing and Validation

### 9.1 Protocol Conformance

- Message format validation
- Topic/resource structure verification
- Security mechanism testing
- Reconnection behavior
- Error handling

### 9.2 Interoperability Testing

- Cross-vendor gateway compatibility
- Multiple protocol support
- Cloud platform integration
- Load and stress testing

---

**弘益人間 (Hongik Ingan) - Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*

---

## Annex A — Conformance Tier Matrix

WIA conformance for environmental-sensor is evaluated across three tiers:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references published standards alongside this document. Implementers SHOULD review the relevant standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- ISO/IEC 17065:2012 — Conformity assessment — Requirements for bodies certifying products, processes and services
- ISO/IEC 27001:2022 — Information security management systems
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- W3C PROV-DM — provenance data model

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/environmental-sensor/api/` — TypeScript SDK skeleton
- `wia-standards/standards/environmental-sensor/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/environmental-sensor/simulator/` — interactive browser-based simulator for the PHASE protocol

Each reference artifact ships with an MIT license and is intended as a starting point, not as a production-grade implementation.

### C.2 Test Vectors

A normative set of request/response pairs covering the schemas defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector in both serialization (request) and deserialization (response) directions, and MUST publish a signed report identifying which vectors were exercised.

Test vectors are versioned independently of the PHASE document; refer to the `test-vectors/` directory for the active version.

---

## Annex D — Open Questions and Future Work

This PHASE document captures the consensus position at v1.0. The following items are tracked for future minor or major revisions:

1. **Schema versioning policy** — formal MUST/SHOULD rules for backward-compatible vs. breaking changes between minor releases.
2. **Privacy-preserving aggregation** — guidance on differential privacy and secure aggregation patterns for telemetry channels, without prescribing a specific algorithm.
3. **Multilingual error catalogs** — localization strategy for the standard error codes defined in this PHASE.
4. **Long-term retention** — alignment with sectoral retention regulations across jurisdictions.
5. **Sustainability disclosure** — optional fields for energy and emissions reporting tied to operations covered by this PHASE.

Items in this annex are non-normative. Comments and proposals are accepted via the GitHub issues tracker on the WIA-Official organization.

---
