# WIA-IND-002: Smart Textile Standard
## PHASE 3: COMMUNICATION PROTOCOL SPECIFICATION
### 弘益人間 - Benefit All Humanity

---

## 1. Overview

This document defines communication protocols for WIA-IND-002 Smart Textile Standard. These specifications ensure reliable, secure, and efficient data transmission between smart textile devices and external systems.

**Version:** 1.0  
**Status:** Final  
**Last Updated:** 2025-12-27

## 2. Bluetooth Low Energy (BLE) GATT Services

### 2.1 Primary Service UUID

```
WIA Smart Textile Service
UUID: 0000181A-0000-1000-8000-00805F9B34FB
Type: Primary Service
```

### 2.2 Characteristic Definitions

#### 2.2.1 Temperature Characteristic

```
UUID: 00002A1C-0000-1000-8000-00805F9B34FB
Properties: READ, NOTIFY
Permissions: Read (authenticated)
Value Format: IEEE-11073 FLOAT (4 bytes)
  - Exponent: 1 byte (signed)
  - Mantissa: 3 bytes (signed)
Unit: Celsius (UUID: 0x272F)
Notification: Every 1-60 seconds (configurable)
```

#### 2.2.2 Heart Rate Characteristic

```
UUID: 00002A37-0000-1000-8000-00805F9B34FB
Properties: READ, NOTIFY
Value Format:
  - Byte 0: Flags
    - Bit 0: HR format (0=uint8, 1=uint16)
    - Bit 1-2: Sensor contact
    - Bit 3: Energy expended present
    - Bit 4: RR intervals present
  - Byte 1-2: Heart rate value
  - Byte 3+: Optional RR intervals
Notification: On change or every 1 second
```

#### 2.2.3 ECG Waveform Characteristic

```
UUID: 00002ABC-0000-1000-8000-00805F9B34FB (Custom)
Properties: NOTIFY
Value Format:
  - Byte 0-1: Sequence number
  - Byte 2: Lead configuration
  - Byte 3-4: Sampling rate (Hz)
  - Byte 5+: ECG samples (int16 array)
Notification: Continuous streaming at 250-1000 Hz
Max packet size: 20 bytes (BLE 4.2) or 244 bytes (BLE 5.0)
```

#### 2.2.4 Device Configuration Characteristic

```
UUID: 00002ABD-0000-1000-8000-00805F9B34FB (Custom)
Properties: READ, WRITE
Value Format:
  - Byte 0: Sensor enable flags
  - Byte 1: Sampling rate index
  - Byte 2: Notification interval
  - Byte 3: Power mode
  - Byte 4-7: Reserved
```

### 2.3 BLE Connection Parameters

```
Connection Interval: 7.5ms - 30ms (recommended: 15ms)
Slave Latency: 0-4 intervals
Supervision Timeout: 2000ms minimum
MTU Size: 247 bytes (BLE 4.2+)
Data Length Extension: Enabled (251 bytes payload)
PHY: LE 2M (for higher throughput)
```

### 2.4 BLE Security

```
Pairing Method: LE Secure Connections (ECDH)
Authentication: MITM protection required
Encryption: AES-128-CCM
Bonding: Persistent pairing information
Privacy: RPA (Resolvable Private Address) rotation every 15 minutes
```

### 2.5 BLE Advertisement

```
Advertisement Type: Connectable, Scannable
Advertisement Interval: 100ms - 1000ms
Advertisement Data:
  - Flags: 0x06 (LE General Discoverable, BR/EDR not supported)
  - Complete 128-bit Service UUID
  - Local Name: "WIA-IND-002-{deviceId}"
  - Manufacturer Data:
    - Company ID: 0xFFFF (placeholder)
    - Battery Level: 1 byte
    - Device Status: 1 byte
```

## 3. WiFi Direct Protocol

### 3.1 Connection Setup

```
Operating Frequency: 2.4 GHz or 5 GHz
Channel Width: 20 MHz / 40 MHz / 80 MHz
Security: WPA2-PSK or WPA3-SAE
Connection Method: Push Button or PIN
Maximum Clients: 4 simultaneous connections
```

### 3.2 Service Discovery

```
Protocol: Wi-Fi Direct Service Discovery (WFD SD)
Service Type: "wia.ind-002.textile"
Service Info: JSON metadata
{
  "deviceId": "UUID",
  "version": "1.0",
  "sensors": ["temp", "ecg", "motion"],
  "capabilities": ["streaming", "storage"]
}
```

### 3.3 Data Transfer

```
Protocol: TCP/IP over WiFi Direct
Port: 8080 (HTTP API) or 8883 (MQTTS)
Throughput: Up to 100 Mbps (depends on PHY)
Latency: <10ms typical
Use Case: High-bandwidth ECG streaming, firmware updates
```

## 4. NFC Pairing

### 4.1 NFC Tag Configuration

```
Tag Type: NFC Forum Type 2 (NTAG216) or Type 4
Memory: 888 bytes usable
Data Format: NDEF (NFC Data Exchange Format)
Record Types:
  - URI: Connection handoff to BLE/WiFi
  - MIME: Device configuration JSON
  - External: Custom WIA records
```

### 4.2 Connection Handover

```
NDEF Message:
  - Handover Select Record
  - Alternative Carrier Record (BLE)
    - BLE Device Address
    - BLE Service UUID
    - Optional: Pairing key
  - Alternative Carrier Record (WiFi Direct)
    - SSID
    - Password
    - Operating Channel
```

### 4.3 Configuration via NFC

```
Write Configuration:
  - Tap to configure WiFi credentials
  - Tap to set user profile
  - Tap to update firmware URL

Read Status:
  - Tap to read last measurement
  - Tap to check battery status
  - Tap to retrieve device info
```

## 5. Thread Networking

### 5.1 Thread Network Configuration

```
IEEE 802.15.4 PHY/MAC:
  - Frequency: 2.4 GHz
  - Channel: 11-26 (configurable)
  - Power: -20 dBm to +8 dBm
  - Data Rate: 250 kbps

Network Parameters:
  - Network Name: "WIA-Textile-Net"
  - PAN ID: 0x1234 (configurable)
  - Extended PAN ID: 64-bit unique identifier
  - Master Key: 128-bit encryption key
```

### 5.2 Device Roles

```
End Device (Sleepy):
  - Smart textile sensors
  - Battery-powered
  - Can sleep to conserve power
  - Poll parent for messages

Router:
  - Gateway devices
  - Always-on
  - Route messages
  - Provide IPv6 connectivity
```

### 5.3 IPv6 Addressing

```
Global Unicast Address: 2001:db8::/64
Link-Local Address: fe80::/64
Multicast Groups:
  - All Nodes: ff02::1
  - All Routers: ff02::2
  - WIA Textiles: ff05::1a (custom)
```

### 5.4 CoAP Messaging

```
Protocol: Constrained Application Protocol (CoAP)
Transport: UDP over IPv6
Default Port: 5683 (CoAP), 5684 (CoAPs)

Resource Structure:
  - /sensors/temperature
  - /sensors/heartrate
  - /sensors/ecg
  - /config
  - /status
```

## 6. LoRaWAN Integration

### 6.1 LoRaWAN Configuration

```
Class: Class A (mandatory), Class C (optional)
Activation: OTAA (Over-The-Air Activation) preferred
Regional Parameters: EU868, US915, AS923, etc.
Data Rates: SF7-SF12 (adaptive)
```

### 6.2 Device Activation (OTAA)

```
Join Request:
  - DevEUI: 64-bit unique device identifier
  - AppEUI: 64-bit application identifier
  - AppKey: 128-bit AES key

Join Accept:
  - AppNonce
  - NetID
  - DevAddr
  - Derived Keys: NwkSKey, AppSKey
```

### 6.3 Message Format

```
FPort: 1 (application data)
Payload: Binary encoded sensor data
Max Size: 51 bytes (SF12) to 242 bytes (SF7)

Encoding Example:
  - Byte 0: Message type (0x01 = sensor data)
  - Byte 1-2: Temperature (int16, 0.01°C resolution)
  - Byte 3-4: Heart rate (uint16)
  - Byte 5: Battery level (uint8, %)
  - Byte 6+: Additional sensors
```

### 6.4 Transmission Strategy

```
Confirmed Messages: For critical alerts only
Unconfirmed Messages: Routine sensor data
ADR (Adaptive Data Rate): Enabled
Transmission Interval: 10 minutes (configurable)
Retry Policy: 3 attempts with exponential backoff
```

## 7. Data Synchronization Protocol

### 7.1 Time Synchronization

```
Protocol: NTP (Network Time Protocol) or SNTP
Accuracy: ±100ms for fitness, ±10ms for medical
Sync Interval: Every 1 hour or after connection
Fallback: Device RTC with drift compensation
```

### 7.2 Data Buffering

```
Local Storage:
  - Ring buffer: 24 hours of data
  - Compression: LZ4 or gzip
  - Priority: Critical events > routine data

Sync Strategy:
  - Real-time: High-priority events
  - Batch: Low-priority data every 15 minutes
  - Opportunistic: When WiFi available
```

### 7.3 Conflict Resolution

```
Timestamp-based: Latest write wins
Version vectors: For distributed systems
CRDTs: For eventual consistency
Device ID tiebreaker: Higher ID wins in case of exact timestamp match
```

## 8. Power Management Protocols

### 8.1 Sleep Modes

```
Active Mode:
  - All sensors operational
  - Continuous BLE connection
  - Power: 50-200 mW

Low Power Mode:
  - Essential sensors only
  - BLE connection interval extended
  - Power: 5-20 mW

Sleep Mode:
  - All sensors paused
  - BLE advertising only
  - Power: <1 mW
```

### 8.2 Wake-Up Events

```
Scheduled: Timer-based periodic wake
Motion Triggered: Accelerometer threshold
User Initiated: Button press
External: NFC field detected
Network: BLE connection request
```

### 8.3 Power Optimization

```
Sensor Duty Cycling: Sample-sleep-sample pattern
Data Compression: Reduce transmission size
Batch Transmission: Aggregate multiple readings
Adaptive Sampling: Increase rate during activity, decrease at rest
```

## 9. Quality of Service (QoS)

### 9.1 Priority Levels

```
Critical (P0): Medical alerts, fall detection
  - Latency: <1 second
  - Reliability: 99.99%
  - Guaranteed delivery

High (P1): Real-time biometric data
  - Latency: <5 seconds
  - Reliability: 99.9%
  - At-least-once delivery

Normal (P2): Routine sensor data
  - Latency: <60 seconds
  - Reliability: 99%
  - Best-effort delivery

Low (P3): Environmental data, statistics
  - Latency: <5 minutes
  - Reliability: 95%
  - Best-effort delivery
```

### 9.2 Bandwidth Allocation

```
Critical Events: Reserved 20% bandwidth
Real-time Streaming: Dynamic 30-50% bandwidth
Batch Data: Remaining bandwidth
Background Tasks: Opportunistic
```

## 10. Protocol Security

### 10.1 Encryption

```
BLE: AES-128-CCM
WiFi: WPA2/WPA3
Thread: AES-128-CCM
LoRaWAN: AES-128 (NwkSKey, AppSKey)
Application Layer: TLS 1.3 (for internet connectivity)
```

### 10.2 Authentication

```
Device-to-Gateway: Mutual TLS with device certificates
User-to-Device: PIN or biometric authentication
API Access: OAuth 2.0 + JWT tokens
```

### 10.3 Data Integrity

```
Message Authentication: HMAC-SHA256
Sequence Numbers: Prevent replay attacks
Timestamps: Detect stale data
CRC: Error detection for sensor data
```

### 10.4 Secure Boot and Updates

```
Boot Process:
  1. Verify bootloader signature
  2. Verify firmware signature
  3. Check firmware version
  4. Load trusted firmware only

OTA Updates:
  - Signed firmware images
  - Version rollback protection
  - Encrypted transmission
  - Integrity verification before flash
```

---

**Philosophy:** 弘益人間 - These protocol specifications ensure secure, reliable, and efficient communication for smart textiles, enabling technology that benefits all humanity through robust, interoperable connectivity standards.

**License:** Creative Commons BY-SA 4.0  
**Contact:** standards@wia.org

---

## Annex A — Conformance Tier Matrix

WIA conformance for WIA-IND-002-smart-textile is evaluated across three tiers:

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

- `wia-standards/standards/WIA-IND-002-smart-textile/api/` — TypeScript SDK skeleton
- `wia-standards/standards/WIA-IND-002-smart-textile/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/WIA-IND-002-smart-textile/simulator/` — interactive browser-based simulator for the PHASE protocol

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
