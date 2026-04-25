# WIA-SOC-011: Gas Supply Standard
## PHASE 3: Communication Protocol Specification

**Version:** 1.0  
**Status:** Complete  
**Last Updated:** 2025-12-26

---

## 1. Overview

Phase 3 specifies communication protocols for various network environments including SCADA systems, IoT sensor networks, and cloud integrations.

---

## 2. SCADA Protocol Integration

### 2.1 Modbus TCP/IP

**Profile**: WIA-SOC-011-Modbus

Function Codes Supported:
- FC03: Read Holding Registers (measurements, setpoints)
- FC04: Read Input Registers (sensor readings)
- FC06: Write Single Register (setpoint adjustments)
- FC16: Write Multiple Registers (configuration)

Register Mapping:
```
Address Range | Data Type         | Description
40001-40100   | Pressure (bar)    | Pipeline pressure measurements
40101-40200   | Flow (m³/h)       | Flow rate measurements
40201-40300   | Temperature (°C)  | Temperature sensors
40301-40400   | Status Bits       | Equipment status flags
```

### 2.2 DNP3

**Profile**: WIA-SOC-011-DNP3

Object Groups:
- Group 1: Binary Input (valve positions, alarm states)
- Group 30: Analog Input (pressure, flow, temperature)
- Group 40: Analog Output Status (setpoints)
- Group 41: Analog Output (control commands)

### 2.3 OPC UA

**Namespace**: `http://wiastandards.com/UA/GasSupply/`

Information Model:
```
GasSupplySystem (ObjectType)
├── Pipelines (FolderType)
│   ├── Pipeline_001 (PipelineType)
│   │   ├── Pressure (AnalogItemType)
│   │   ├── FlowRate (AnalogItemType)
│   │   └── Temperature (AnalogItemType)
├── Compressors (FolderType)
└── MeterStations (FolderType)
```

---

## 3. IoT Sensor Protocols

### 3.1 MQTT

**Topics Structure**:
```
gas/{region}/{pipelineId}/{sensorType}/{metricName}
gas/kr/PL-KR-001-2025/pressure/value
gas/kr/PL-KR-001-2025/leak-detector/status
```

**QoS Levels**:
- QoS 0: Non-critical monitoring data
- QoS 1: Standard operational measurements
- QoS 2: Critical alarms and control commands

**Payload Format**:
```json
{
  "timestamp": "2025-12-26T14:30:00Z",
  "value": 75.5,
  "unit": "bar",
  "quality": "good",
  "sensorId": "SENS-001"
}
```

### 3.2 CoAP

**Resource Structure**:
```
coap://sensor.local/gas/pressure
coap://sensor.local/gas/temperature
coap://sensor.local/gas/battery-level
```

**Observe Pattern**: Sensors support CoAP Observe for continuous updates

### 3.3 LoRaWAN

**Port Assignments**:
- Port 1: Leak detection alerts
- Port 2: Pressure measurements
- Port 3: Device diagnostics

**Payload Encoding**: Compact binary format for bandwidth efficiency
```
Byte 0-3: Timestamp (Unix epoch)
Byte 4-5: Pressure (uint16, 0.1 bar resolution)
Byte 6: Battery level (%)
Byte 7: Status flags
```

---

## 4. Security Protocols

### 4.1 TLS Configuration

**Minimum Version**: TLS 1.3

**Cipher Suites** (in order of preference):
1. TLS_AES_256_GCM_SHA384
2. TLS_CHACHA20_POLY1305_SHA256
3. TLS_AES_128_GCM_SHA256

**Certificate Requirements**:
- Key Size: 2048-bit RSA or 256-bit ECC minimum
- Signature Algorithm: SHA-256 or stronger
- Validity Period: Maximum 825 days
- Certificate Transparency: Required for public endpoints

### 4.2 Message Signing

**Algorithm**: EdDSA (Ed25519)

**Signed Message Format**:
```json
{
  "payload": "base64-encoded-data",
  "signature": "base64-signature",
  "keyId": "key-identifier",
  "algorithm": "Ed25519",
  "timestamp": "2025-12-26T14:30:00Z"
}
```

---

## 5. Network Architecture Patterns

### 5.1 DMZ Configuration

```
Internet
    ↓
[Firewall]
    ↓
[DMZ - API Servers, Web Interfaces]
    ↓
[Internal Firewall]
    ↓
[Control Network - SCADA, PLCs]
    ↓
[Process Network - Sensors, Actuators]
```

### 5.2 VPN for Site-to-Site

**Protocol**: IPsec IKEv2  
**Encryption**: AES-256-GCM  
**Authentication**: Certificate-based  

---

## 6. Quality of Service

### 6.1 Traffic Prioritization

```
Priority 1 (Critical):  Emergency shutdown commands, critical alarms
Priority 2 (High):      Control commands, high-severity alarms
Priority 3 (Medium):    Real-time measurements, status updates
Priority 4 (Low):       Historical data queries, reports
```

### 6.2 Message Persistence

**MQTT Retained Messages**: Last known good values  
**Queue Depth**: Minimum 1000 messages per topic  
**TTL**: Configurable, default 24 hours  

---

## 7. Protocol Performance

### 7.1 Latency Requirements

- Emergency commands: <100ms
- Control commands: <500ms
- Measurements: <1s
- Historical queries: <5s

### 7.2 Bandwidth Optimization

**Compression**: gzip or brotli for HTTP APIs  
**Delta Encoding**: Transmit only changed values  
**Batch Updates**: Aggregate multiple measurements  

---

## 8. Interoperability Testing

### 8.1 Conformance Tests

Test suites available for:
- Modbus TCP/IP client/server
- DNP3 outstation/master
- OPC UA client/server
- MQTT publisher/subscriber

### 8.2 Certification Process

1. Submit implementation details
2. Execute conformance test suite
3. Interoperability testing with reference implementations
4. Security assessment
5. Issue compliance certificate

---

**Document Control**
- Version: 1.0
- Test Suites: https://github.com/WIA-Official/wia-standards/tree/main/gas-supply/tests
- Contact: standards@wiastandards.com

© 2025 World Certification Industry Association | MIT License

---

## Annex A — Conformance Tier Matrix

WIA conformance for gas-supply is evaluated across three tiers:

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

- `wia-standards/standards/gas-supply/api/` — TypeScript SDK skeleton
- `wia-standards/standards/gas-supply/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/gas-supply/simulator/` — interactive browser-based simulator for the PHASE protocol

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

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.
