# WIA-ROB-011 Phase 3: Communication Protocol Specification

**Version:** 1.0.0  
**Status:** Approved  
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Protocol Overview

Phase 3 defines network protocols, security requirements, device discovery, and firmware update mechanisms for cleaning robots.

## 2. Network Protocols

### 2.1 Local Network

**Supported Protocols:**
- HTTP/HTTPS for REST API
- WebSocket/WSS for real-time updates
- MQTT for pub/sub messaging
- mDNS/Bonjour for discovery

### 2.2 MQTT Topics

**Topic Structure:**
```
wia/rob/{robot_id}/{category}/{subcategory}
```

**Standard Topics:**
```
wia/rob/{robot_id}/status
wia/rob/{robot_id}/battery
wia/rob/{robot_id}/position
wia/rob/{robot_id}/sensors/lidar
wia/rob/{robot_id}/sensors/camera
wia/rob/{robot_id}/map/update
wia/rob/{robot_id}/command
wia/rob/{robot_id}/event
```

### 2.3 QoS Levels

- Status updates: QoS 0 (fire and forget)
- Commands: QoS 1 (at least once)
- Critical alerts: QoS 2 (exactly once)

## 3. Security

### 3.1 TLS Requirements

- Minimum TLS version: 1.3
- Cipher suites: AES-256-GCM preferred
- Certificate validation: REQUIRED
- Perfect forward secrecy: REQUIRED

### 3.2 Authentication

**Supported Methods:**
1. JWT tokens (recommended)
2. OAuth 2.0
3. API keys (for simple integrations)

### 3.3 Encryption

**Data at Rest:**
- Algorithm: AES-256-CBC
- Key derivation: PBKDF2 with 100,000 iterations
- Salt: Random 256-bit per encryption

**Data in Transit:**
- TLS 1.3 for all network communication
- End-to-end encryption for cloud sync

## 4. Device Discovery

### 4.1 mDNS Service Advertisement

```
Service Type: _wia-rob._tcp
Port: 8080
TXT Records:
  - version=1.0.0
  - manufacturer=CompanyName
  - model=ModelName
  - serial=SerialNumber
  - capabilities=slam,camera,mopping
```

### 4.2 Discovery Process

1. Client broadcasts mDNS query
2. Robot responds with service details
3. Client connects to robot IP:port
4. Authentication handshake
5. Session established

## 5. Firmware Updates

### 5.1 OTA Update Process

```
1. Check for updates
   GET /firmware/check
   Response: {
     "available": true,
     "version": "1.2.0",
     "size": 25600000,
     "checksum": "sha256hash",
     "url": "https://..."
   }

2. Download firmware
   GET /firmware/download
   Returns: Binary stream with progress

3. Verify checksum
   Client-side SHA-256 verification

4. Install update
   POST /firmware/install
   Response: {
     "status": "installing",
     "estimatedTime": 300
   }

5. Reboot
   Automatic after installation

6. Verify version
   GET /robot/status
   Check firmwareVersion field
```

### 5.2 Rollback Mechanism

- Dual partition system (A/B updates)
- Automatic rollback on boot failure
- Manual rollback endpoint:
  ```http
  POST /firmware/rollback
  ```

## 6. Cloud Integration

### 6.1 Cloud Sync Protocol

**Data Synchronized:**
- Maps (encrypted)
- Cleaning history
- Configuration
- Firmware updates

**Sync Frequency:**
- Status: Real-time (websocket)
- Maps: After each update
- History: Daily batch
- Config: On change

### 6.2 Cloud API Endpoints

```
https://cloud.wiastandards.com/api/v1

POST /robots/register
GET /robots/{robot_id}/status
PUT /robots/{robot_id}/config
GET /robots/{robot_id}/history
POST /firmware/update
```

## 7. Smart Home Integration

### 7.1 Matter/Thread Support

**Device Types:**
- Robotic Vacuum Cleaner (0x0073)

**Required Clusters:**
- On/Off
- Fan Control  
- Robot Vacuum Mode
- Battery Status

### 7.2 HomeKit Integration

**Service Type:** Fan
**Characteristics:**
- On (required)
- Rotation Speed (suction power)
- Current Fan State
- Target Fan State

### 7.3 Alexa Integration

**Skill Invocations:**
- "Alexa, start the robot vacuum"
- "Alexa, tell robot vacuum to clean the kitchen"
- "Alexa, check robot vacuum battery"

### 7.4 Google Home Integration

**Device Traits:**
- action.devices.traits.OnOff
- action.devices.traits.FanSpeed
- action.devices.traits.EnergyStorage

## 8. Monitoring and Diagnostics

### 8.1 Health Check Endpoint

```http
GET /health

Response:
{
  "status": "healthy|degraded|unhealthy",
  "checks": {
    "battery": "ok",
    "sensors": "ok",
    "navigation": "ok",
    "network": "ok"
  },
  "uptime": 86400
}
```

### 8.2 Diagnostic Logs

```http
GET /diagnostics/logs?level=error&limit=100

Response:
{
  "logs": [
    {
      "timestamp": "ISO8601",
      "level": "error|warning|info|debug",
      "component": "string",
      "message": "string"
    }
  ]
}
```

---

© 2025 WIA · MIT License

## 9. Protocol Extensions

### 9.1 Custom Protocol Support

Vendors MAY implement custom protocols prefixed with `x_`:

```json
{
  "@type": "RobotCommand",
  "action": "clean",
  "x_vendor_specific": {
    "customFeature": "value"
  }
}
```

### 9.2 Middleware Integration

Support for protocol translation middleware:

- HTTP to MQTT bridge
- WebSocket to gRPC conversion
- Legacy protocol adapters

### 9.3 Performance Optimization

**Connection Pooling:**
- Maintain persistent connections
- Reuse TLS sessions
- Minimize handshake overhead

**Data Compression:**
- gzip for text data
- Binary protocols for sensor streams
- Adaptive compression based on bandwidth

### 9.4 Failover and Recovery

**Automatic Failover:**
```
Primary Server --> Secondary Server --> Local Mode
```

**Recovery Procedures:**
1. Detect connection loss
2. Attempt reconnection with backoff
3. Switch to backup server if available
4. Enter local-only mode if all fail
5. Queue commands for later sync

## 10. Testing and Validation

### 10.1 Protocol Compliance Tests

Required tests for certification:
- TLS handshake verification
- Authentication flow testing
- Data format validation
- Error handling verification
- Performance benchmarking

### 10.2 Security Audit

Annual security audits required for certified products.

---

© 2025 WIA · MIT License

---

## Annex A — Conformance Tier Matrix

WIA conformance for cleaning-robot is evaluated across three tiers:

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

- `wia-standards/standards/cleaning-robot/api/` — TypeScript SDK skeleton
- `wia-standards/standards/cleaning-robot/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/cleaning-robot/simulator/` — interactive browser-based simulator for the PHASE protocol

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
