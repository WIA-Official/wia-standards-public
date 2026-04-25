# WIA-SEMI-003 - Phase 1: Device Specification Data Structures

**Version:** 1.0.0  
**Status:** Active  
**Last Updated:** 2025-12-26

---

## 1. Overview

Phase 1 of the WIA Power Semiconductor Standard (SEMI-003) defines device specification data structures for power electronic devices including IGBTs, MOSFETs, SiC, and GaN devices. This specification ensures interoperability, reduces integration complexity, and accelerates adoption of advanced power semiconductor technologies across automotive, industrial, and renewable energy applications.

### 1.1 Scope

This phase covers:
- Standardized data structures for device characteristics
- JSON schema definitions and validation rules
- Integration interfaces with simulation and design tools
- Version management and backward compatibility
- Certification and conformance testing requirements

### 1.2 Normative References

- WIA-CORE-001: Core Data Types and Schemas
- WIA-CERT-001: Certification and Compliance Framework
- IEEE 1800.2: Universal Verification Methodology (UVM)
- IEC 60747: Semiconductor Devices Standards
- ISO/IEC 27001: Information Security Management

---

## 2. Data Format Specification

### 2.1 Device Metadata

All power semiconductor devices must provide metadata in standardized JSON format:

```json
{
  "standardId": "WIA-SEMI-003",
  "version": "1.0.0",
  "deviceId": "MANUFACTURER-PARTNUMBER-REVISION",
  "manufacturer": {
    "name": "Manufacturer Name",
    "registryId": "WIA-MFG-XXXXX",
    "contact": "standards@manufacturer.com"
  },
  "deviceType": "SiC_MOSFET",
  "packageType": "TO-247-4",
  "certificationLevel": 3,
  "certificationDate": "2025-06-15",
  "datasheet": "https://manufacturer.com/datasheets/partnumber.pdf"
}
```

### 2.2 Electrical Characteristics

Electrical parameters must include:

#### Voltage Ratings
- Drain-Source Voltage (V_DSS) or Collector-Emitter Voltage (V_CES)
- Gate-Source Voltage (V_GSS) or Gate-Emitter Voltage (V_GES)
- Breakdown voltage characteristics
- Safe operating area (SOA) boundaries

```json
{
  "voltageRatings": {
    "vds_max": {
      "value": 1200,
      "unit": "V",
      "testConditions": {
        "temperature": 25,
        "temperatureUnit": "°C",
        "gateVoltage": 0,
        "testStandard": "IEC 60747-8"
      }
    },
    "vgs_max": {
      "value": 25,
      "unit": "V",
      "polarity": "±"
    }
  }
}
```

#### Current Ratings
- Continuous drain/collector current
- Pulsed current capability
- Temperature derating curves
- Avalanche current ratings

#### On-State Characteristics
- RDS(on) for MOSFETs or VCE(sat) for IGBTs
- Temperature coefficient
- Current dependency
- Gate voltage dependency

### 2.3 Thermal Characteristics

Thermal data must include transient and steady-state impedance:

```json
{
  "thermalImpedance": {
    "junctionToCase": {
      "steadyState": {
        "value": 0.35,
        "unit": "°C/W",
        "testStandard": "JESD51-1"
      },
      "transient": {
        "model": "Foster",
        "parameters": [
          {"R": 0.05, "tau": 0.001},
          {"R": 0.12, "tau": 0.01},
          {"R": 0.18, "tau": 0.1}
        ]
      }
    },
    "junctionToAmbient": {
      "value": 40,
      "unit": "°C/W",
      "testConditions": "Still air, PCB mounted"
    }
  },
  "maxJunctionTemperature": {
    "value": 175,
    "unit": "°C"
  }
}
```

### 2.4 Switching Characteristics

Dynamic switching parameters including:

```json
{
  "switchingCharacteristics": {
    "turnOnTime": {
      "value": 25,
      "unit": "ns",
      "testConditions": {
        "vds": 600,
        "id": 50,
        "vgs": 15,
        "rg": 10,
        "temperature": 25
      }
    },
    "turnOffTime": {
      "value": 45,
      "unit": "ns",
      "testConditions": "same as turnOnTime"
    },
    "switchingEnergy": {
      "turnOn": {
        "value": 0.35,
        "unit": "mJ"
      },
      "turnOff": {
        "value": 0.42,
        "unit": "mJ"
      }
    }
  }
}
```

---

## 3. API Interface Specification (Phase 1)

### 3.1 RESTful Endpoints

Power modules implementing API support must expose the following endpoints:

#### Device Information
```
GET /api/v1/device/info
Response: Complete device metadata and capabilities
```

#### Real-Time Monitoring
```
GET /api/v1/telemetry/current
Response: Current operational parameters (temperature, current, voltage)
```

#### Historical Data
```
GET /api/v1/telemetry/history?start=<timestamp>&end=<timestamp>&interval=<seconds>
Response: Time-series data for specified period
```

### 3.2 WebSocket Streaming

Real-time data streaming via WebSocket:

```
WS /api/v1/stream
{
  "subscribe": ["temperature", "current", "voltage"],
  "sampleRate": 1000  // Hz
}
```

### 3.3 Authentication

All API access requires OAuth 2.0 authentication with device-specific client credentials.

---

## 4. Integration Patterns

### 4.1 Simulation Tool Integration

Standardized SPICE models provided in formats:
- LTspice (.asc, .lib)
- HSPICE (.sp, .lib)
- PLECS (.xml)
- MATLAB/Simulink (.slx)

### 4.2 Design Tool Integration

CAD library packages including:
- Schematic symbols
- PCB footprints  
- 3D STEP models
- Thermal models

---

## 5. Conformance Testing

### 5.1 Mandatory Tests

Devices must pass all mandatory conformance tests:

| Test ID | Description | Pass Criteria |
|---------|-------------|---------------|
| SEMI-003-P1-001 | Schema validation | 100% schema compliance |
| SEMI-003-P1-002 | Data completeness | All mandatory fields present |
| SEMI-003-P1-003 | Value range validation | Within specified bounds |
| SEMI-003-P1-004 | Unit consistency | Correct SI units |
| SEMI-003-P1-005 | Cross-validation | Internal consistency |

### 5.2 Optional Tests

Optional feature tests for advanced capabilities:
- Extended temperature range data
- Advanced thermal modeling
- Multi-chip module coordination
- Predictive maintenance integration

---

## 6. Security Requirements

### 6.1 Data Integrity

All device data must be cryptographically signed:
- RSA-4096 or ECDSA P-384 signatures
- SHA-256 hash verification
- Certificate chain validation

### 6.2 Firmware Authentication

Firmware updates require:
- Code signing with manufacturer key
- Secure boot verification
- Rollback protection

---

## 7. Version Management

### 7.1 Semantic Versioning

Standard uses semantic versioning: MAJOR.MINOR.PATCH
- MAJOR: Incompatible API changes
- MINOR: Backward-compatible additions
- PATCH: Backward-compatible fixes

### 7.2 Deprecation Policy

Features deprecated with 18-month notice period minimum.

---

## 8. Certification Process

### 8.1 Self-Certification

Manufacturers can self-certify Level 1 compliance using WIA validation tools.

### 8.2 Laboratory Certification

Levels 2-4 require testing at WIA-accredited laboratories:
- Comprehensive test execution
- Security audit
- Interoperability validation
- Documentation review

### 8.3 Certification Validity

Certificates valid for 36 months with annual surveillance audits.

---

## 9. Implementation Timeline

Recommended implementation schedule:

| Milestone | Target Date | Deliverables |
|-----------|-------------|--------------|
| Specification Study | Week 1-2 | Requirements analysis |
| Data Structure Implementation | Week 3-6 | JSON schema compliance |
| API Development | Week 7-12 | Endpoint implementation |
| Testing | Week 13-16 | Conformance validation |
| Certification | Week 17-20 | Laboratory testing |
| Production | Week 21+ | Certified devices |

---

## 10. Support and Resources

### 10.1 Reference Implementations

WIA provides reference implementations:
- C/C++ libraries
- Python SDK
- JavaScript/TypeScript packages
- Example device profiles

### 10.2 Validation Tools

Automated validation tools available:
- JSON schema validators
- API test clients
- Security scanners
- Conformance test suites

### 10.3 Community Support

- WIA Standards Forum: https://forum.wiastandards.com
- Technical Support: support@wiastandards.com
- GitHub Repository: https://github.com/WIA-Official/wia-semi-003

---

**弘益人間 (Hongik Ingan) - Benefit All Humanity**

*WIA - World Certification Industry Association*  
*© 2025 MIT License*

---

## Annex A — Conformance Tier Matrix

WIA conformance for power-semiconductor is evaluated across three tiers:

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

- `wia-standards/standards/power-semiconductor/api/` — TypeScript SDK skeleton
- `wia-standards/standards/power-semiconductor/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/power-semiconductor/simulator/` — interactive browser-based simulator for the PHASE protocol

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


## Annex E — Implementation Notes for PHASE-1-DATA-FORMAT

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1-DATA-FORMAT.

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

