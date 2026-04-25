# WIA-IND-003: Phase 3 - Protocol Specification
## Wearable Fashion Communication Protocols

**Version:** 1.0
**Status:** Final
**Date:** 2025-01-15
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 3 defines communication protocols for wearable fashion devices, including Bluetooth Low Energy (BLE) pairing, device discovery, data synchronization, and multi-device coordination.

## 2. Bluetooth Low Energy (BLE) Protocol

### 2.1 BLE Service UUID

**Primary Service UUID:** `0000FE00-1234-5678-9ABC-DEF012345678`

**Service Name:** "WIA Wearable Fashion Service"

### 2.2 GATT Characteristics

| Characteristic | UUID | Properties | Description |
|----------------|------|------------|-------------|
| Device Info | 0xFE01 | Read | Device identification and capabilities |
| Sensor Data | 0xFE02 | Read, Notify | Real-time sensor readings |
| Command | 0xFE03 | Write | Control commands to device |
| Status | 0xFE04 | Read, Notify | Device status updates |
| Configuration | 0xFE05 | Read, Write | Device settings |
| Firmware | 0xFE06 | Write | Firmware update channel |

### 2.3 Device Discovery

**Advertising Packet Format:**

```
Flags: 0x06 (General Discoverable, BR/EDR not supported)
Complete Local Name: "WIA-{DeviceType}-{ShortID}"
Service UUID: 0xFE00
Manufacturer Data: [CompanyID][DeviceType][BatteryLevel][Status]
```

**Example:**
```
Name: "WIA-SW-12AB"
Service: 0xFE00
Mfr Data: [0xFF 0xE0][0x01][0x55][0x01]
           ^^^^^^^^^ ^^^^ ^^^^ ^^^^
           CompanyID Type Bat% Status
```

### 2.4 Pairing Modes

#### 2.4.1 Just Works Pairing (Basic)
- No user interaction required
- Security Level: Low
- Use case: Basic fitness trackers, low-value data

#### 2.4.2 Numeric Comparison (Recommended)
- Display 6-digit code on both devices
- User confirms match
- Security Level: High
- Use case: Health monitoring, payment-enabled devices

#### 2.4.3 Passkey Entry
- User enters 6-digit code displayed on device
- Security Level: High
- Use case: Devices without displays

#### 2.4.4 Out of Band (OOB)
- NFC tap for initial pairing
- Security Level: Very High
- Use case: Premium devices with NFC

### 2.5 Security Requirements

**Minimum Security Level:** LE Secure Connections (BLE 4.2+)

**Encryption:** AES-128-CCM mandatory

**Authentication:** MITM protection for health/payment data

**Key Storage:** Keys MUST be stored in secure element if available

## 3. Data Synchronization Protocol

### 3.1 Sync Initiation

**Device → App:**
```json
{
  "type": "sync_request",
  "deviceId": "WF-SW-12345",
  "lastSyncToken": "token-abc123",
  "dataTypes": ["heartRate", "steps", "battery"],
  "recordCount": 142
}
```

**App → Device:**
```json
{
  "type": "sync_ack",
  "syncToken": "token-xyz789",
  "proceed": true
}
```

### 3.2 Data Transfer

**Chunked Transfer for Large Datasets:**

```json
{
  "type": "data_chunk",
  "chunkId": 1,
  "totalChunks": 5,
  "data": [ /* Sensor readings */ ],
  "checksum": "crc32-value"
}
```

**Acknowledgment:**
```json
{
  "type": "chunk_ack",
  "chunkId": 1,
  "status": "received"
}
```

### 3.3 Conflict Resolution

When device and app have conflicting data:

1. **Timestamp Priority:** Most recent timestamp wins
2. **Device Authority:** Device sensor data takes precedence over manual entries
3. **User Confirmation:** Critical changes require user approval

## 4. NFC Protocol

### 4.1 NFC Data Exchange Format (NDEF)

**Record Type:** `application/vnd.wia.ind003`

**Payload Structure:**
```
[Version:1][DeviceID:16][Capabilities:4][PairingData:32]
```

### 4.2 NFC Pairing Flow

1. User taps device to phone
2. App reads NDEF record
3. Extract device ID and pairing data
4. Initiate BLE connection using pairing data
5. Complete secure pairing

### 4.3 NFC Payment Integration

**EMV Contactless Standard:** ISO/IEC 14443

**Payment Token Format:**
```json
{
  "tokenId": "encrypted-token",
  "expiryDate": "2027-12",
  "cardNetwork": "visa",
  "lastFourDigits": "1234"
}
```

## 5. Multi-Device Coordination

### 5.1 Device Priority Levels

| Priority | Device Type | Use Case |
|----------|-------------|----------|
| 1 (Highest) | Smart Watch | Primary activity tracker |
| 2 | Smart Ring | Sleep/recovery tracking |
| 3 | Smart Glasses | AR/notifications |
| 4 | Other Accessories | Supplementary data |

### 5.2 Sensor Arbitration

When multiple devices measure same metric:

**Rule 1: Optimal Placement**
- Heart rate: Ring > Watch > Band (finger PPG more accurate)
- Steps: Watch > Ring > Shoes
- Sleep: Ring > Watch (less movement interference)

**Rule 2: Battery Conservation**
- If multiple devices can measure, use device with highest battery
- Coordinate to distribute sensing load

**Rule 3: User Override**
- User can designate primary device per metric

### 5.3 Handoff Protocol

When user switches devices mid-activity:

```json
{
  "type": "activity_handoff",
  "fromDevice": "WF-SW-12345",
  "toDevice": "WF-RG-67890",
  "activity": {
    "type": "workout",
    "startTime": "2025-01-15T10:00:00Z",
    "currentMetrics": {
      "duration": 1800,
      "heartRate": 145,
      "distance": 2500
    }
  },
  "timestamp": "2025-01-15T10:30:00Z"
}
```

## 6. Communication Optimization

### 6.1 Connection Intervals

**Active Use:** 30-50ms (low latency for notifications)

**Background Sync:** 1000-2000ms (battery conservation)

**Idle:** 4000ms (minimum activity)

### 6.2 Data Batching

- Batch sensor readings every 5 minutes during normal use
- Immediate transmission for critical events (fall detection, irregular heart rhythm)
- Compress batches using gzip before transmission

### 6.3 Transmission Power

**Dynamic Power Adjustment:**

| RSSI | Distance | TX Power |
|------|----------|----------|
| > -50 dBm | Very Close | -20 dBm |
| -50 to -70 dBm | Normal | -12 dBm |
| -70 to -85 dBm | Far | -4 dBm |
| < -85 dBm | Very Far | +4 dBm (max) |

## 7. Error Handling and Retry Logic

### 7.1 Connection Failures

**Retry Strategy:**
1. Immediate retry (0s delay)
2. Short delay retry (5s delay)
3. Medium delay retry (30s delay)
4. Long delay retry (2 min delay)
5. Give up, notify user

### 7.2 Data Corruption

**Detection:**
- CRC32 checksum for all data packets
- Sequence numbers to detect missing packets

**Recovery:**
- Request retransmission of corrupted chunks
- Full resync if corruption persistent

### 7.3 Timeout Handling

| Operation | Timeout | Action on Timeout |
|-----------|---------|-------------------|
| Connection | 30s | Retry connection |
| Data transfer | 60s | Resume from last chunk |
| Command response | 10s | Retry command |
| Pairing | 120s | Abort, retry pairing |

## 8. Protocol State Machine

### 8.1 Connection States

```
DISCONNECTED → SCANNING → CONNECTING → CONNECTED → SYNCING → IDLE
     ↑                                      ↓
     └──────────────────────────────────────┘
```

### 8.2 State Transitions

**DISCONNECTED → SCANNING:**
- Trigger: App opened, periodic check, user action

**SCANNING → CONNECTING:**
- Trigger: Device discovered

**CONNECTING → CONNECTED:**
- Trigger: BLE connection established

**CONNECTED → SYNCING:**
- Trigger: Authentication complete

**SYNCING → IDLE:**
- Trigger: All data synchronized

**IDLE → DISCONNECTED:**
- Trigger: Timeout, user action, low battery

## 9. Firmware Update Protocol

### 9.1 Update Discovery

**Device → App:**
```json
{
  "currentFirmware": "2.1.0",
  "updateCheckInterval": 86400
}
```

**App → Device:**
```json
{
  "updateAvailable": true,
  "version": "2.2.0",
  "size": 524288,
  "chunkSize": 4096,
  "checksum": "sha256-hash"
}
```

### 9.2 Update Transfer

**OTA (Over-The-Air) Process:**

1. **Download:** Transfer firmware in chunks
2. **Verify:** Validate checksum
3. **Install:** Flash new firmware
4. **Reboot:** Restart with new firmware
5. **Verify:** Confirm successful update

**Chunk Transfer:**
```json
{
  "type": "firmware_chunk",
  "offset": 0,
  "size": 4096,
  "data": "base64-encoded-binary",
  "checksum": "chunk-crc32"
}
```

### 9.3 Rollback Mechanism

If update fails:
- Automatic rollback to previous firmware
- Preserve user data and settings
- Report failure to server

## 10. Power Management Protocol

### 10.1 Low Power Mode Negotiation

**Device → App:**
```json
{
  "batteryLevel": 15,
  "requestPowerMode": "low",
  "disableFeatures": ["always-on-display", "continuous-hr"]
}
```

**App → Device:**
```json
{
  "approvePowerMode": "low",
  "syncInterval": 3600
}
```

### 10.2 Critical Battery Protocol

When battery < 5%:
1. Disable non-essential features
2. Extend connection intervals to 4000ms
3. Reduce sensor sampling rates
4. Notify user
5. Prepare for shutdown, preserve critical data

---

**Philosophy Note:** 弘益人間 (Benefit All Humanity)

These protocols benefit humanity by:
- Ensuring reliable, secure communication
- Optimizing battery life for all-day use
- Protecting user privacy through encryption
- Enabling seamless multi-device experiences

---

**© 2025 SmileStory Inc. / WIA**
**WIA-IND-003 Phase 3 Specification v1.0**

---

## Annex A — Conformance Tier Matrix

WIA conformance for WIA-IND-003-wearable-fashion is evaluated across three tiers:

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

- `wia-standards/standards/WIA-IND-003-wearable-fashion/api/` — TypeScript SDK skeleton
- `wia-standards/standards/WIA-IND-003-wearable-fashion/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/WIA-IND-003-wearable-fashion/simulator/` — interactive browser-based simulator for the PHASE protocol

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
