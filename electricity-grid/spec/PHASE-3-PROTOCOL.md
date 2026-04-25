# PHASE 3: Communication Protocol Specification

**WIA-SOC-010 Electricity Grid Standard**
Version: 1.0
Status: Draft
Last Updated: 2025-12-26

---

## 1. Overview

This document specifies communication protocols for the WIA-SOC-010 Electricity Grid Standard, covering SCADA protocols, IoT messaging, and security requirements.

## 2. Protocol Stack

### 2.1 Application Layer
- HTTPS/REST (primary)
- WebSocket (real-time)
- MQTT (IoT devices)
- IEC 61850 (substations)
- DNP3 (SCADA)
- Modbus TCP (field devices)

### 2.2 Transport Layer
- TLS 1.3 (required for all connections)
- TCP (primary)
- UDP (time-critical applications)

### 2.3 Network Layer
- IPv4 and IPv6 (dual-stack support)
- IPsec (optional, for VPNs)

## 3. MQTT for IoT Devices

### 3.1 Broker Configuration
- Protocol: MQTT 5.0
- Port: 8883 (TLS), 1883 (non-TLS, discouraged)
- QoS Levels: 0, 1, 2 (all supported)

### 3.2 Topic Structure
```
wia-soc-010/{region}/{zone}/{device-type}/{device-id}/{metric}
```

**Examples:**
```
wia-soc-010/northeast/zone-a/solar/pv-001/generation
wia-soc-010/northeast/zone-a/battery/ess-001/soc
wia-soc-010/northeast/zone-a/meter/meter-123/consumption
```

### 3.3 Message Format
```json
{
  "timestamp": "2025-12-26T14:30:00Z",
  "value": 580,
  "unit": "MW",
  "quality": "good"
}
```

### 3.4 Security
- Username/password authentication (minimum)
- Client certificates (recommended)
- TLS 1.3 encryption (required)

## 4. IEC 61850 for Substations

### 4.1 Profile Support
- IEC 61850-7-2: Abstract communication service interface (ACSI)
- IEC 61850-8-1: MMS mapping
- IEC 61850-9-2: Sampled values
- IEC 61850-90-5: Synchrophasors

### 4.2 GOOSE Messaging
Generic Object Oriented Substation Event (GOOSE) for peer-to-peer communication:
- Protocol: IEC 61850-8-1
- Transport: Ethernet Layer 2 (no IP)
- Latency: < 4 milliseconds

### 4.3 Sampled Values
- Protocol: IEC 61850-9-2LE
- Sampling rate: 80 samples/cycle (4800 Hz for 60 Hz systems)
- Precision: 16-bit minimum

## 5. DNP3 for SCADA

### 5.1 Protocol Version
- DNP3: IEEE 1815-2012
- Secure Authentication: DNP3 SAv5

### 5.2 Object Groups
- Binary Input (Group 1)
- Binary Output (Group 10)
- Analog Input (Group 30)
- Analog Output (Group 40)
- Time and Date (Group 50)

### 5.3 Security
- Challenge-response authentication
- Session key management
- Message authentication codes (MAC)

## 6. Modbus TCP

### 6.1 Configuration
- Port: 502
- Unit ID: 1-247
- Timeout: 5 seconds

### 6.2 Function Codes
- 01: Read Coils
- 02: Read Discrete Inputs
- 03: Read Holding Registers
- 04: Read Input Registers
- 05: Write Single Coil
- 06: Write Single Register
- 15: Write Multiple Coils
- 16: Write Multiple Registers

## 7. Security Requirements

### 7.1 Encryption
- TLS 1.3 (required)
- AES-256-GCM (symmetric)
- RSA-4096 or ECC P-384 (asymmetric)

### 7.2 Authentication
- OAuth 2.0 + JWT (APIs)
- X.509 certificates (device authentication)
- Multi-factor authentication (administrative access)

### 7.3 Authorization
- Role-based access control (RBAC)
- Principle of least privilege
- Audit logging

### 7.4 Network Security
- Firewalls and DMZ architecture
- Intrusion detection/prevention systems
- Network segmentation (IT/OT separation)

## 8. Time Synchronization

### 8.1 Protocol
- NTP (Network Time Protocol)
- PTP (Precision Time Protocol) for synchrophasors

### 8.2 Accuracy Requirements
- SCADA: ±1 second
- Synchrophasors: ±1 microsecond
- Smart meters: ±10 seconds

## 9. Quality of Service

### 9.1 Latency Targets
- Critical control: < 10 ms
- SCADA telemetry: < 100 ms
- Smart meter data: < 1 second
- Historical data: Best effort

### 9.2 Availability
- Control systems: 99.99%
- Data collection: 99.9%
- Historical retrieval: 99%

---

**End of PHASE 3 Specification**

---

## Annex A — Conformance Tier Matrix

WIA conformance for electricity-grid is evaluated across three tiers:

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

- `wia-standards/standards/electricity-grid/api/` — TypeScript SDK skeleton
- `wia-standards/standards/electricity-grid/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/electricity-grid/simulator/` — interactive browser-based simulator for the PHASE protocol

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
