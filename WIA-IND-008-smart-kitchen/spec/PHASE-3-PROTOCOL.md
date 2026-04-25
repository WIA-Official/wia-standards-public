# WIA-IND-008: Smart Kitchen Standard
# Phase 3: Protocol Specification

**Version:** 1.0
**Category:** Industrial (IND)
**Philosophy:** 弘益人間 (Benefit All Humanity)
**Last Updated:** 2025-01-15

## Overview

Phase 3 defines communication protocols for IoT device interoperability, security mechanisms, voice command processing, and real-time messaging in smart kitchen ecosystems.

## Core Principles

1. **Interoperability:** Support multiple standard protocols
2. **Security:** End-to-end encryption, authentication, authorization
3. **Reliability:** Message delivery guarantees, fault tolerance
4. **Low Latency:** Real-time responsiveness for critical operations
5. **Energy Efficient:** Optimized for battery-powered sensors
6. **Privacy-First:** Local processing where possible, minimal data transmission

## 1. Supported IoT Protocols

### 1.1 Matter (Primary Recommendation)

**Why Matter:**
- Industry-standard for smart home devices
- IP-based, works over Wi-Fi, Thread, and Ethernet
- Strong security model with device attestation
- Manufacturer-agnostic interoperability

**Implementation Requirements:**
- All smart kitchen appliances SHOULD support Matter
- Device certification through Connectivity Standards Alliance (CSA)
- Support for Matter device types: Oven, Refrigerator, Dishwasher, etc.

**Connection Flow:**
```
1. Device advertising (BLE or mDNS)
2. Commissioning via QR code or NFC
3. Device provisioning to network
4. Secure session establishment
5. Ongoing encrypted communication
```

### 1.2 MQTT (Message Queuing Telemetry Transport)

**Use Cases:**
- Telemetry streaming from devices
- Asynchronous notifications
- Low-bandwidth sensor networks

**Topic Structure:**
```
wia-ind-008/{home-id}/devices/{device-id}/{message-type}

Examples:
wia-ind-008/home-abc123/devices/oven-001/status
wia-ind-008/home-abc123/devices/fridge-001/temperature
wia-ind-008/home-abc123/devices/dishwasher-001/cycle-complete
```

**QoS Levels:**
- QoS 0: Telemetry data (best effort)
- QoS 1: Commands (at least once delivery)
- QoS 2: Critical operations (exactly once delivery)

**Message Format:**
```json
{
  "timestamp": "ISO 8601 datetime",
  "deviceId": "string",
  "messageType": "enum: status|telemetry|command|event",
  "payload": "object (device-specific)",
  "sequence": "number (message sequence)"
}
```

### 1.3 Thread

**Use Cases:**
- Low-power mesh networking for sensors
- Battery-powered devices (door sensors, temperature sensors)
- Distributed sensor networks

**Network Architecture:**
```
[Border Router] ←→ [Thread Network]
                    ├─ Temperature Sensor
                    ├─ Door Sensor
                    ├─ Water Leak Detector
                    └─ Smart Scale
```

**Benefits:**
- IPv6-based
- Self-healing mesh topology
- Low power consumption
- Secure (AES-128 encryption)

### 1.4 Zigbee (Legacy Support)

**Status:** Supported for backwards compatibility

**Migration Path:**
- New devices SHOULD use Matter over Thread
- Existing Zigbee devices supported via gateway/bridge
- Gradual deprecation over 5-year timeline

### 1.5 Bluetooth Low Energy (BLE)

**Use Cases:**
- Device commissioning and setup
- Proximity-based operations
- Mobile app direct communication
- Sensor data from wearables

**GATT Services:**
```
WIA Kitchen Service (UUID: 0x1234)
├─ Device Info Characteristic (read)
├─ Control Characteristic (write)
├─ Status Characteristic (read, notify)
└─ Configuration Characteristic (read, write)
```

## 2. Security Protocols

### 2.1 Transport Layer Security (TLS 1.3+)

**Requirements:**
- All network communication MUST use TLS 1.3 or later
- Certificate-based authentication for devices
- Perfect forward secrecy (PFS) enabled
- Strong cipher suites only (AES-GCM, ChaCha20-Poly1305)

**Certificate Management:**
```
[Root CA] → [Intermediate CA] → [Device Certificate]
                              → [Server Certificate]
```

### 2.2 Device Authentication

**Methods:**

1. **Certificate-Based:**
   - X.509 certificates provisioned at manufacturing
   - Device attestation during commissioning
   - Regular certificate rotation (annual)

2. **Token-Based:**
   - JWT tokens for API access
   - Refresh token rotation
   - Expiration: Access tokens (1 hour), Refresh tokens (30 days)

3. **API Keys:**
   - For server-to-server communication
   - Scoped permissions
   - Regular rotation recommended

### 2.3 Data Encryption

**At Rest:**
- AES-256 encryption for stored data
- Encrypted database fields for sensitive information
- Secure key storage (TPM, Secure Enclave)

**In Transit:**
- TLS 1.3 for all network communication
- End-to-end encryption for user data
- Perfect forward secrecy

**Privacy-Preserving:**
- Local processing for sensitive operations
- Differential privacy for analytics
- Anonymization before cloud transmission

### 2.4 Access Control

**Role-Based Access Control (RBAC):**

Roles:
- `owner`: Full control over all devices and data
- `admin`: Manage devices, users, but not delete home
- `member`: Use devices, view data, no configuration
- `guest`: Limited temporary access
- `child`: Restricted access to safe operations

Permissions Matrix:
```
Operation           | Owner | Admin | Member | Guest | Child
--------------------|-------|-------|--------|-------|-------
Add/Remove Devices  |   ✓   |   ✓   |   ✗    |   ✗   |   ✗
Control Appliances  |   ✓   |   ✓   |   ✓    |   ✓   |   ○
View Inventory      |   ✓   |   ✓   |   ✓    |   ✓   |   ✗
Modify Meal Plans   |   ✓   |   ✓   |   ✓    |   ✗   |   ✗
View Energy Data    |   ✓   |   ✓   |   ✓    |   ✗   |   ✗
Manage Users        |   ✓   |   ✓   |   ✗    |   ✗   |   ✗

✓ = Allowed, ✗ = Denied, ○ = Restricted subset
```

## 3. Voice Command Protocol

### 3.1 Voice Processing Pipeline

```
[Microphone] → [Wake Word Detection] → [Speech Recognition]
→ [Natural Language Understanding] → [Intent Classification]
→ [Parameter Extraction] → [Action Execution] → [Response Generation]
```

### 3.2 Wake Words

Standardized wake phrases:
- "Hey Kitchen"
- "Smart Kitchen"
- Or custom manufacturer wake words

**Wake Word Processing:**
- Local edge processing (privacy)
- Low-power always-listening mode
- False positive rate < 1%

### 3.3 Intent Schema

```json
{
  "utterance": "preheat the oven to 350 degrees",
  "intent": {
    "action": "control_appliance",
    "target": {
      "deviceType": "oven",
      "deviceId": "auto-resolve"
    },
    "command": "preheat",
    "parameters": {
      "temperature": 350,
      "unit": "fahrenheit"
    },
    "confidence": 0.95
  },
  "alternatives": [
    {
      "interpretation": "alternate intent",
      "confidence": 0.05
    }
  ]
}
```

### 3.4 Supported Intent Types

**Device Control:**
- `turn_on`, `turn_off`
- `set_temperature`, `set_timer`
- `start_cycle`, `pause_operation`, `cancel_operation`

**Information Queries:**
- `get_status`, `check_temperature`
- `inventory_check`, `recipe_search`
- `nutritional_info`

**Cooking Assistance:**
- `start_recipe`, `next_step`, `previous_step`
- `set_timer`, `check_timer`
- `conversion_query` (measurements)

**Shopping & Planning:**
- `add_to_shopping_list`, `read_shopping_list`
- `suggest_meal`, `plan_week`

### 3.5 Multilingual Support

**Required Languages (Tier 1):**
- English (en-US, en-GB, en-AU)
- Spanish (es-ES, es-MX)
- Chinese (zh-CN, zh-TW)
- Japanese (ja-JP)
- Korean (ko-KR)
- French (fr-FR, fr-CA)
- German (de-DE)
- Italian (it-IT)
- Portuguese (pt-BR, pt-PT)
- Hindi (hi-IN)

**Extended Support (Tier 2):** 89+ additional languages

**Language Detection:**
- Automatic language detection from speech
- User profile language preference
- Per-user language settings in multi-user households

### 3.6 Voice Response Protocol

```json
{
  "responseId": "string",
  "intent": "original intent object",
  "status": "enum: success|error|clarification_needed",
  "response": {
    "speech": "string (TTS text)",
    "display": "object (visual response for screens)",
    "action": {
      "deviceId": "string",
      "command": "string",
      "executed": "boolean"
    }
  },
  "followUp": {
    "enabled": "boolean",
    "context": "object (for multi-turn conversation)"
  }
}
```

## 4. Real-Time Communication

### 4.1 WebSocket Protocol

**Endpoint:** `wss://api.{domain}/wia-ind-008/v1/ws`

**Connection Establishment:**
```
1. Client initiates WebSocket handshake
2. Server validates authentication token
3. Client subscribes to channels
4. Bidirectional message flow begins
```

**Message Format:**
```json
{
  "type": "enum: subscribe|unsubscribe|message|ping|pong",
  "channel": "string",
  "data": "object",
  "messageId": "string",
  "timestamp": "datetime"
}
```

**Channels:**
- `devices/{deviceId}/status`
- `cooking/session/{sessionId}`
- `inventory/updates`
- `energy/realtime`
- `alerts/all`

### 4.2 Server-Sent Events (SSE)

Alternative to WebSocket for one-way real-time updates:

**Endpoint:** `GET /v1/events`

**Event Stream Format:**
```
event: device-status
data: {"deviceId": "oven-001", "status": "on"}
id: 12345

event: inventory-alert
data: {"itemId": "milk-001", "alert": "expiring"}
id: 12346
```

## 5. Local Network Discovery

### 5.1 mDNS/DNS-SD

**Service Advertisement:**
```
Service Type: _wia-kitchen._tcp.local.
Instance Name: Kitchen Refrigerator._wia-kitchen._tcp.local.
Port: 443
TXT Records:
  - model=WIA-REF-2025
  - version=1.0
  - capabilities=inventory,camera,ice-maker
  - manufacturer=SmartHome Inc.
```

### 5.2 UPnP/SSDP

For legacy compatibility:
```
NOTIFY * HTTP/1.1
HOST: 239.255.255.250:1900
NT: urn:wia-official-org:device:SmartKitchen:1
NTS: ssdp:alive
USN: uuid:12345678-1234-1234-1234-123456789012::urn:wia-official-org:device:SmartKitchen:1
```

## 6. Firmware Update Protocol

### 6.1 OTA (Over-The-Air) Updates

**Update Flow:**
```
1. Device checks for updates (daily)
2. Server provides update metadata
3. Device downloads firmware (chunked)
4. Cryptographic verification (signature check)
5. Staged installation (A/B partitions)
6. Automatic rollback on failure
7. Update confirmation
```

**Update Manifest:**
```json
{
  "version": "2.0.1",
  "releaseDate": "2025-01-15",
  "size": 15728640,
  "checksum": "sha256:abcdef...",
  "signature": "RSA signature",
  "downloadUrl": "https://updates.example.com/...",
  "releaseNotes": "string",
  "critical": "boolean",
  "compatibleModels": ["array of model numbers"]
}
```

### 6.2 Security Requirements

- Signed firmware images (RSA-2048 minimum)
- Secure boot verification
- Encrypted download channel (HTTPS)
- Version rollback protection
- Update window configuration (avoid peak usage)

## 7. Time Synchronization

**Protocol:** NTP (Network Time Protocol) or SNTP

**Requirements:**
- Devices MUST synchronize time at boot
- Regular sync every 24 hours minimum
- Timezone awareness (IANA timezone database)
- DST (Daylight Saving Time) handling

**Importance:**
- Accurate cooking timers
- Scheduled operations
- Event logging
- Certificate validation

## 弘益人間 Considerations

- Multiple protocol support ensures device accessibility
- Strong security protections serve all users equally
- Multilingual voice support (99 languages) promotes inclusivity
- Local processing respects privacy globally
- Open protocols prevent vendor lock-in
- Energy-efficient protocols reduce environmental impact

---

**Previous Phase:** [Phase 2: API Interface](PHASE-2-API-INTERFACE.md)
**Next Phase:** [Phase 4: Integration](PHASE-4-INTEGRATION.md)

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity

---

## Annex A — Conformance Tier Matrix

WIA conformance for WIA-IND-008-smart-kitchen is evaluated across three tiers:

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

- `wia-standards/standards/WIA-IND-008-smart-kitchen/api/` — TypeScript SDK skeleton
- `wia-standards/standards/WIA-IND-008-smart-kitchen/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/WIA-IND-008-smart-kitchen/simulator/` — interactive browser-based simulator for the PHASE protocol

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
