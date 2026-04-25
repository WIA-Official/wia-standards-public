# PHASE 1: Data Format Specification

**WIA-SOC-010 Electricity Grid Standard**
Version: 1.0
Status: Draft
Last Updated: 2025-12-26

---

## 1. Overview

This document defines the data formats and structures for the WIA-SOC-010 Electricity Grid Standard. All implementations MUST conform to these specifications to ensure interoperability across different systems, vendors, and jurisdictions.

## 2. Data Exchange Format

### 2.1 Primary Format: JSON-LD

All data exchanges SHALL use JSON-LD (JSON for Linked Data) as the primary format. JSON-LD enables:
- Human-readable data structures
- Machine-processable linked data
- Semantic interoperability
- Extensibility through vocabularies

**Example:**
```json
{
  "@context": "https://wia-official.org/standards/soc-010/context.jsonld",
  "@type": "GridStatusReport",
  "@id": "urn:uuid:550e8400-e29b-41d4-a716-446655440000",
  "timestamp": "2025-12-26T14:30:00Z",
  "gridOperator": {
    "@type": "Organization",
    "name": "Metropolitan Power Grid",
    "id": "urn:grid:operator:mpg-001"
  },
  "systemLoad": {
    "@type": "PowerMeasurement",
    "value": 2450,
    "unit": "MW",
    "timestamp": "2025-12-26T14:30:00Z"
  },
  "renewableGeneration": {
    "@type": "PowerMeasurement",
    "value": 1028,
    "unit": "MW",
    "percentage": 42.0
  }
}
```

### 2.2 Alternative Formats

Implementations MAY support additional formats for specific use cases:
- **XML**: For legacy system integration
- **Protocol Buffers**: For high-performance applications
- **CSV**: For bulk data exchange
- **Parquet**: For big data analytics

However, JSON-LD MUST always be supported as the canonical format.

## 3. Core Data Models

### 3.1 Grid Status

```json
{
  "@context": "https://wia-official.org/standards/soc-010/context.jsonld",
  "@type": "GridStatus",
  "id": "string (UUID)",
  "timestamp": "ISO 8601 datetime",
  "region": "string",
  "frequency": {
    "value": "number",
    "unit": "Hz",
    "deviation": "number"
  },
  "voltage": {
    "nominal": "number",
    "actual": "number",
    "unit": "kV"
  },
  "currentLoad": {
    "value": "number",
    "unit": "MW"
  },
  "capacity": {
    "total": "number",
    "available": "number",
    "unit": "MW"
  },
  "loadFactor": "number (0-1)",
  "status": "enum (normal|warning|critical|emergency)"
}
```

### 3.2 Renewable Energy Data

```json
{
  "@type": "RenewableGeneration",
  "id": "string (UUID)",
  "timestamp": "ISO 8601 datetime",
  "sources": [
    {
      "type": "enum (solar|wind|hydro|biomass|geothermal)",
      "capacity": "number (MW)",
      "generation": "number (MW)",
      "availabilityFactor": "number (0-1)",
      "location": {
        "latitude": "number",
        "longitude": "number",
        "altitude": "number (optional)"
      },
      "forecast": {
        "horizon": "ISO 8601 duration",
        "predictions": [
          {
            "timestamp": "ISO 8601 datetime",
            "expectedGeneration": "number (MW)",
            "confidenceInterval": {
              "lower": "number",
              "upper": "number",
              "level": "number (0-1)"
            }
          }
        ]
      }
    }
  ],
  "totalGeneration": "number (MW)",
  "penetrationRate": "number (0-1)"
}
```

### 3.3 Energy Storage

```json
{
  "@type": "EnergyStorage",
  "id": "string (UUID)",
  "timestamp": "ISO 8601 datetime",
  "technology": "enum (lithium-ion|flow-battery|pumped-hydro|compressed-air|other)",
  "capacity": {
    "power": "number (MW)",
    "energy": "number (MWh)",
    "duration": "number (hours)"
  },
  "stateOfCharge": "number (0-100 %)",
  "powerFlow": {
    "value": "number (MW, positive=charging, negative=discharging)",
    "direction": "enum (charging|discharging|idle)"
  },
  "efficiency": {
    "roundTrip": "number (0-1)",
    "charging": "number (0-1)",
    "discharging": "number (0-1)"
  },
  "cycleCount": "integer",
  "healthStatus": {
    "stateOfHealth": "number (0-100 %)",
    "estimatedRemainingLife": "ISO 8601 duration"
  }
}
```

### 3.4 Demand Response Event

```json
{
  "@type": "DemandResponseEvent",
  "id": "string (UUID)",
  "programType": "enum (dynamic-pricing|direct-control|interruptible|emergency)",
  "startTime": "ISO 8601 datetime",
  "endTime": "ISO 8601 datetime",
  "targetLoadReduction": "number (MW)",
  "actualLoadReduction": "number (MW)",
  "participants": {
    "enrolled": "integer",
    "active": "integer",
    "opted-out": "integer"
  },
  "pricing": {
    "baseline": "number (currency/kWh)",
    "event": "number (currency/kWh)",
    "currency": "ISO 4217 code"
  },
  "incentives": {
    "total": "number (currency)",
    "perParticipant": "number (currency)",
    "currency": "ISO 4217 code"
  },
  "status": "enum (scheduled|active|completed|canceled)"
}
```

### 3.5 Power Quality Metrics

```json
{
  "@type": "PowerQualityMetrics",
  "id": "string (UUID)",
  "timestamp": "ISO 8601 datetime",
  "location": "string (substation/feeder ID)",
  "frequency": {
    "nominal": "number (Hz)",
    "actual": "number (Hz)",
    "deviation": "number (Hz)",
    "stability": "number (0-1)"
  },
  "voltage": {
    "nominal": "number (kV)",
    "actual": "number (kV)",
    "sags": "integer (count)",
    "swells": "integer (count)",
    "interruptions": "integer (count)"
  },
  "harmonics": {
    "thd": "number (0-100 %)",
    "individual": [
      {
        "harmonic": "integer (2, 3, 5, 7, ...)",
        "magnitude": "number (% of fundamental)"
      }
    ]
  },
  "powerFactor": "number (-1 to 1)",
  "flicker": {
    "pst": "number",
    "plt": "number"
  }
}
```

### 3.6 Smart Meter Reading

```json
{
  "@type": "MeterReading",
  "meterId": "string",
  "timestamp": "ISO 8601 datetime",
  "customerId": "string",
  "location": {
    "address": "string",
    "coordinates": {
      "latitude": "number",
      "longitude": "number"
    }
  },
  "consumption": {
    "energy": "number (kWh)",
    "interval": "ISO 8601 duration",
    "imported": "number (kWh)",
    "exported": "number (kWh, for prosumers)"
  },
  "demand": {
    "peak": "number (kW)",
    "average": "number (kW)"
  },
  "voltage": {
    "min": "number (V)",
    "max": "number (V)",
    "average": "number (V)"
  },
  "events": [
    {
      "type": "enum (outage|voltage-sag|voltage-swell|tamper|...)",
      "timestamp": "ISO 8601 datetime",
      "duration": "ISO 8601 duration",
      "severity": "enum (info|warning|critical)"
    }
  ]
}
```

## 4. Data Quality Requirements

### 4.1 Timestamp Precision
- All timestamps MUST use ISO 8601 format with timezone
- Precision MUST be at least 1 second
- For synchrophasor data, precision MUST be 1 millisecond or better
- UTC timezone is RECOMMENDED for all data exchange

### 4.2 Measurement Accuracy
- Voltage measurements: ±0.5%
- Current measurements: ±1%
- Power measurements: ±1.5%
- Frequency measurements: ±0.01 Hz
- Energy measurements: ±2%

### 4.3 Data Completeness
- Missing data MUST be explicitly indicated (null or NaN)
- Estimated values MUST be flagged with quality indicator
- Data gaps exceeding 15 minutes MUST trigger alarms

### 4.4 Validation Rules
1. Frequency MUST be within ±5% of nominal (57-63 Hz for 60 Hz systems)
2. Voltage MUST be within ±10% of nominal under normal conditions
3. Power factor MUST be between -1 and 1
4. State of charge MUST be 0-100%
5. Efficiency values MUST be 0-1 (0-100%)

## 5. Schema Versioning

### 5.1 Version Format
- Semantic versioning: MAJOR.MINOR.PATCH
- Example: 1.2.3

### 5.2 Compatibility Rules
- MAJOR: Breaking changes (backward incompatible)
- MINOR: New features (backward compatible)
- PATCH: Bug fixes (backward compatible)

### 5.3 Version Negotiation
```json
{
  "@context": "https://wia-official.org/standards/soc-010/context.jsonld",
  "schemaVersion": "1.0.0",
  "supportedVersions": ["1.0.0", "1.0.1", "1.1.0"]
}
```

## 6. Compression and Encoding

### 6.1 Compression
- Implementations SHOULD support gzip compression
- Large datasets (>1 MB) SHOULD be compressed
- HTTP Content-Encoding: gzip header MUST be used

### 6.2 Character Encoding
- UTF-8 MUST be used for all text data
- Binary data MUST be Base64 encoded when included in JSON

## 7. Security Considerations

### 7.1 Sensitive Data
Personal identifiable information (PII) MUST be:
- Encrypted at rest
- Encrypted in transit (TLS 1.3+)
- Anonymized when possible
- Subject to access controls

### 7.2 Data Integrity
- Digital signatures SHOULD be used for critical data
- Checksums MUST be provided for large datasets
- Tamper detection mechanisms MUST be implemented

## 8. Metadata

All data objects SHOULD include metadata:
```json
{
  "metadata": {
    "source": "string (system ID)",
    "quality": "enum (raw|validated|estimated|derived)",
    "reliability": "number (0-1)",
    "latency": "ISO 8601 duration",
    "processingTimestamp": "ISO 8601 datetime"
  }
}
```

---

**End of PHASE 1 Specification**

For API endpoints and protocols, see PHASE-2-API.md
For communication protocols, see PHASE-3-PROTOCOL.md
For integration patterns, see PHASE-4-INTEGRATION.md

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

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.
