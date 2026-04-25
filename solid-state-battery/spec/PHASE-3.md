# WIA-AUTO-028: Solid-State Battery Standard
## Phase 3: Communication Protocols

**Version:** 1.0.0
**Status:** Active
**Last Updated:** 2025-12-27
**Category:** AUTO / Mobility

---

## 1. Overview

Phase 3 defines communication protocols for solid-state batteries across multiple transport layers. Protocols ensure reliable, secure, real-time data exchange between batteries, vehicles, chargers, and management systems.

### 1.1 Scope

- CAN bus protocols for in-vehicle communication
- Ethernet protocols for diagnostics and updates
- Wireless protocols for cloud connectivity
- Charging protocols (ISO 15118, CCS, CHAdeMO integration)
- BMS communication hierarchies
- Security and encryption specifications

---

## 2. CAN Bus Protocol

### 2.1 Physical Layer

- **Standard:** CAN 2.0B (11-bit identifiers) and CAN FD (flexible data rate)
- **Bit Rate:** 500 kbps (classic CAN), up to 5 Mbps (CAN FD data phase)
- **Termination:** 120Ω at both ends
- **Topology:** Linear bus with short stubs (<300mm)

### 2.2 Message Structure

Following SAE J1939 Parameter Group Number (PGN) format:

| PGN | Name | Priority | DLC | Frequency | Description |
|-----|------|----------|-----|-----------|-------------|
| 0xFE40 | Battery Status | 3 | 8 | 100 ms | SOC, SOH, voltage, current |
| 0xFE41 | Cell Voltages | 6 | 8 | 500 ms | Individual cell voltages |
| 0xFE42 | Temperatures | 6 | 8 | 500 ms | Temperature sensors |
| 0xFE43 | Power Limits | 3 | 8 | 100 ms | Charge/discharge limits |
| 0xFE44 | Fault Codes | 2 | 8 | On event | Diagnostic trouble codes |
| 0xFE45 | Energy Stats | 6 | 8 | 1000 ms | Energy throughput |

### 2.3 Battery Status Message (PGN 0xFE40)

**Data Bytes:**
- Byte 0-1: State of Charge (0-100%, 0.01% resolution)
- Byte 2-3: Pack Voltage (0-655.35V, 0.01V resolution)
- Byte 4-5: Pack Current (-3276.8 to +3276.7A, 0.1A resolution)
- Byte 6: State of Health (0-100%, 1% resolution)
- Byte 7: Status flags

**Example:**
```
CAN ID: 0x18FE4000 (Priority 3, PGN 0xFE40, Source Address 0)
Data: 1A 1A 0F 28 FF 38 5E 00
Decoded: SOC=67.5%, V=386.4V, I=-45.2A, SOH=94%, No faults
```

---

## 3. Charging Protocol

### 3.1 ISO 15118 Integration

WIA-AUTO-028 extends ISO 15118 for solid-state battery optimization:

**Charging Phases:**
1. Physical Connection (plug detection)
2. Digital Handshake (capability exchange)
3. Authentication (payment, authorization)
4. Negotiation (voltage, current, power limits)
5. Preconditioning (thermal management)
6. Fast Charging (high-power transfer)
7. Constant Voltage (final stage)
8. Completion (disconnect, billing)

### 3.2 Capability Exchange

**Battery → Charger:**
```xml
<BatteryCapability>
  <MaxVoltage unit="V">460</MaxVoltage>
  <MaxCurrent unit="A">375</MaxCurrent>
  <MaxPower unit="kW">150</MaxPower>
  <OptimalTemp unit="C">
    <Min>15</Min>
    <Max>35</Max>
  </OptimalTemp>
  <ChargingCurve>
    <!-- SOC vs. max current table -->
  </ChargingCurve>
</BatteryCapability>
```

### 3.3 Dynamic Charging Control

Battery continuously updates charging limits based on:
- Current temperature
- SOC level
- Cell balance state
- Health metrics
- Thermal management capability

**Update Frequency:** Every 1-5 seconds during charging

---

## 4. BMS Communication Hierarchy

### 4.1 Architecture

```
Cloud Backend (LTE/5G)
       ↕
Pack Master BMS (CAN/Ethernet)
       ↕
Module Controllers (CAN)
       ↕
Cell Monitoring ICs (Daisy-chain SPI/I2C)
       ↕
Individual Cells
```

### 4.2 Cell Monitoring

- **Protocol:** SPI or I2C daisy-chain
- **Measurements:** Voltage (per cell), temperature (multiple points)
- **Sample Rate:** 10-100 Hz
- **Accuracy:** ±10 mV (voltage), ±1°C (temperature)

### 4.3 Module Controllers

- **Protocol:** CAN bus
- **Functions:** Aggregate cell data, module balancing, fault detection
- **Update Rate:** 100 ms

### 4.4 Pack Master BMS

- **In-Vehicle:** CAN bus to vehicle ECU
- **Diagnostics:** Ethernet (100BASE-T1 or 1000BASE-T)
- **Cloud:** LTE/5G cellular or WiFi

---

## 5. State Machines

### 5.1 Battery Operational States

```
┌──────┐   Self-test    ┌───────┐
│ Init │───────Passed──→│ Ready │
└──────┘                └───┬───┘
    │                       │
   Failed              ┌────┼────┐
    │                  │    │    │
    ↓              Charging  │  Discharging
┌───────┐              │    │    │
│ Fault │←─────────────┘    │    │
└───┬───┘                   │    │
    │                       ↓    ↓
  Cleared                ┌──────────┐
    │                    │  Ready   │
    └───────────────────→│          │
                         └─────┬────┘
                               │
                            Timeout
                               ↓
                         ┌─────────┐
                         │  Sleep  │
                         └─────────┘
```

### 5.2 Charging State Machine

```
Idle → Physical Connection → Digital Handshake → Authentication
  ↑            ↓                     ↓                  ↓
  └────────────┴─────────────────────┴──────────────────┘
                                                         ↓
                                                    Negotiation
                                                         ↓
                                                  Preconditioning
                                                         ↓
                                                   Fast Charging
                                                         ↓
                                                 Constant Voltage
                                                         ↓
                                                    Completion
```

---

## 6. Security Protocols

### 6.1 Encryption

- **CAN Bus:** AUTOSAR SecOC (AES-128-GCM)
- **Ethernet:** TLS 1.3
- **Wireless:** WPA3 (WiFi), IPsec (cellular)

### 6.2 Authentication

- **Device:** X.509 certificates, PKI infrastructure
- **User:** OAuth 2.0, OpenID Connect
- **Message:** HMAC-SHA256 authentication codes

### 6.3 Key Management

- **Storage:** Hardware Security Module (HSM) or Secure Element
- **Rotation:** Keys rotated every 90 days minimum
- **Backup:** Secure key escrow for recovery

### 6.4 Secure Boot

- **Boot Chain:** Each stage verifies next stage signature
- **Root of Trust:** Manufacturer certificate in HSM
- **Rollback Protection:** Version monotonic counter

---

## 7. Network Topologies

### 7.1 In-Vehicle Network

```
Vehicle ECU ──── CAN Bus ──┬─── Battery Master BMS
                           │
                           ├─── Charger Controller
                           │
                           ├─── Thermal Management
                           │
                           └─── Dashboard/HMI
```

### 7.2 Charging Network

```
Battery ──── ISO 15118 ──── Charger ──── Backend
                                │
                                └──── Grid Connection
```

### 7.3 Cloud Connectivity

```
Battery ──── LTE/5G ──── Cloud Platform ──┬─── Mobile App
                                          │
                                          ├─── Web Dashboard
                                          │
                                          └─── Analytics/AI
```

---

## 8. Protocol Testing and Validation

### 8.1 CAN Bus Testing

- **Tools:** Vector CANoe, PEAK PCAN
- **Tests:** Message timing, priority handling, error frames, bus load
- **Pass Criteria:** <1% message loss, <10ms latency

### 8.2 Charging Protocol Testing

- **Tools:** Comemso charging tester, ISO 15118 test suite
- **Tests:** Handshake sequences, negotiation, error recovery
- **Pass Criteria:** 100% successful sessions over 100 tests

### 8.3 Security Testing

- **Penetration Testing:** Third-party security audit
- **Fuzzing:** Protocol input validation
- **Cryptographic Validation:** NIST test vectors

---

## 9. Compliance Requirements

### 9.1 Level 1

- Basic CAN bus protocol (Battery Status, Fault Codes)
- Simple charging handshake
- Basic encryption (static keys)

### 9.2 Level 2

- Full CAN bus protocol (all PGNs)
- ISO 15118 charging protocol
- Dynamic encryption (key rotation)
- BMS hierarchy support

### 9.3 Level 3

- All Level 2 requirements
- Cloud connectivity protocols
- Advanced security (HSM, secure boot)
- Multi-network topology support
- Protocol testing certification

---

**弘益人間 (Hongik Ingan) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*

---

## Annex A — Conformance Tier Matrix

WIA conformance for WIA-MFG-SSB (Solid-State Battery) is evaluated across three tiers, applied to cell chemistry metadata · state-of-health telemetry · cycle test reporting · safety thresholds:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | None (annual self-review recommended) |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST clearly disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references the following published standards. Implementers SHOULD review the listed standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- IEC 62660-1:2018 — Secondary lithium-ion cells for propulsion of electric road vehicles
- IEC 62660-2:2018 — Reliability and abuse testing
- IEC 62619:2022 — Safety requirements for secondary lithium batteries for industrial applications
- ISO 12405-4:2018 — Test specification for lithium-ion traction battery packs and systems
- UN 38.3 — Manual of Tests and Criteria, Section 38.3 (transport of lithium batteries)

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/solid-state-battery/api/` — TypeScript SDK skeleton
- `wia-standards/standards/solid-state-battery/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/solid-state-battery/simulator/` — interactive browser-based simulator for the PHASE protocol

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
