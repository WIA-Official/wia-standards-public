# WIA-SOC-009 Phase 1: Data Format Specification

**Version:** 1.0.0  
**Status:** Approved  
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Overview

Phase 1 defines standardized data formats for smart sewage systems, including sensor readings, water quality parameters, flow metrics, and system status. All data MUST use JSON-LD format for semantic interoperability.

## 2. Core Data Types

### 2.1 System Identity

```json
{
  "@context": "https://wiastandards.com/soc-009/v1",
  "@type": "SewageSystem",
  "systemId": "SYS-2025-XXXX-YYYY",
  "municipality": "string",
  "operator": "string",
  "location": {
    "latitude": "float",
    "longitude": "float",
    "elevation": "float (meters)"
  },
  "capacity": "float (m³/day)",
  "servingPopulation": "integer",
  "installDate": "ISO8601 date",
  "capabilities": ["array", "of", "capability", "strings"]
}
```

### 2.2 System State

```json
{
  "@type": "SystemState",
  "timestamp": "ISO8601 datetime",
  "flowRate": {
    "value": "float (m³/s)",
    "location": "string",
    "direction": "inflow|outflow|bypass"
  },
  "waterQuality": {
    "pH": 0-14,
    "dissolvedOxygen": "float (mg/L)",
    "temperature": "float (°C)",
    "turbidity": "float (NTU)",
    "conductivity": "float (µS/cm)",
    "BOD": "float (mg/L)",
    "COD": "float (mg/L)",
    "TSS": "float (mg/L)",
    "ammoniaNitrogen": "float (mg/L)",
    "totalPhosphorus": "float (mg/L)"
  },
  "treatmentStatus": {
    "stage": "primary|secondary|tertiary",
    "efficiency": 0-100,
    "chemicalUsage": "float (kg/day)"
  },
  "alerts": ["array", "of", "AlertEvent"]
}
```

### 2.3 Sensor Data Format

```json
{
  "@type": "SensorReading",
  "sensorId": "UUID",
  "sensorType": "flow|quality|level|pressure|gas",
  "timestamp": "ISO8601 datetime",
  "location": {
    "latitude": "float",
    "longitude": "float",
    "zone": "string"
  },
  "reading": {
    "value": "float",
    "unit": "string",
    "quality": "good|fair|poor|invalid",
    "confidence": 0-1
  },
  "calibration": {
    "lastCalibrated": "ISO8601 datetime",
    "nextDue": "ISO8601 datetime"
  }
}
```

### 2.4 Water Quality Report

```json
{
  "@type": "WaterQualityReport",
  "reportId": "UUID",
  "timestamp": "ISO8601 datetime",
  "location": "string",
  "sampleType": "grab|composite|continuous",
  "parameters": {
    "physical": {
      "temperature": "float (°C)",
      "turbidity": "float (NTU)",
      "color": "float (Pt-Co units)",
      "odor": "string"
    },
    "chemical": {
      "pH": "float",
      "DO": "float (mg/L)",
      "BOD5": "float (mg/L)",
      "COD": "float (mg/L)",
      "TSS": "float (mg/L)",
      "TDS": "float (mg/L)",
      "ammonia": "float (mg/L)",
      "nitrate": "float (mg/L)",
      "phosphate": "float (mg/L)",
      "chloride": "float (mg/L)",
      "sulfate": "float (mg/L)"
    },
    "biological": {
      "fecalColiform": "float (CFU/100ml)",
      "EColi": "float (CFU/100ml)",
      "totalColiform": "float (CFU/100ml)"
    },
    "heavyMetals": {
      "lead": "float (µg/L)",
      "mercury": "float (µg/L)",
      "cadmium": "float (µg/L)",
      "chromium": "float (µg/L)",
      "arsenic": "float (µg/L)"
    }
  },
  "compliance": {
    "regulatoryLimit": "object",
    "status": "compliant|warning|violation"
  }
}
```

### 2.5 Flow Event Log

```json
{
  "@type": "FlowEvent",
  "eventId": "UUID",
  "timestamp": "ISO8601 datetime",
  "eventType": "overflow|bypass|surge|blockage|leak",
  "location": {
    "latitude": "float",
    "longitude": "float",
    "description": "string"
  },
  "severity": "low|medium|high|critical",
  "flowData": {
    "peakFlow": "float (m³/s)",
    "duration": "integer (seconds)",
    "volume": "float (m³)",
    "estimatedContaminantLoad": "float (kg)"
  },
  "response": {
    "alertSent": "ISO8601 datetime",
    "responseTime": "integer (minutes)",
    "actionTaken": "string",
    "resolvedAt": "ISO8601 datetime"
  },
  "environmentalImpact": {
    "receivingWater": "string",
    "estimatedImpact": "minimal|moderate|severe"
  }
}
```

## 3. Equipment Data Formats

### 3.1 Pump Status

```json
{
  "@type": "PumpStatus",
  "pumpId": "UUID",
  "location": "string",
  "status": "running|stopped|maintenance|fault",
  "flowRate": "float (m³/s)",
  "power": "float (kW)",
  "current": "float (A)",
  "voltage": "float (V)",
  "vibration": "float (mm/s)",
  "temperature": "float (°C)",
  "runtime": "integer (hours)",
  "cycleCount": "integer",
  "maintenance": {
    "lastService": "ISO8601 datetime",
    "nextDue": "ISO8601 datetime",
    "predictedFailure": "ISO8601 datetime (optional)"
  }
}
```

### 3.2 Treatment Process Data

```json
{
  "@type": "TreatmentProcess",
  "processId": "UUID",
  "stage": "screening|grit_removal|primary|secondary|tertiary|disinfection",
  "timestamp": "ISO8601 datetime",
  "influent": {
    "flow": "float (m³/s)",
    "quality": "WaterQualityReport"
  },
  "effluent": {
    "flow": "float (m³/s)",
    "quality": "WaterQualityReport"
  },
  "efficiency": {
    "BODremoval": "float (%)",
    "SSSremoval": "float (%)",
    "pathogenReduction": "float (log reduction)"
  },
  "chemicals": [
    {
      "type": "coagulant|flocculant|disinfectant|pH_adjuster",
      "name": "string",
      "dosage": "float (mg/L)",
      "costPerUnit": "float ($/kg)"
    }
  ],
  "energy": {
    "consumption": "float (kWh)",
    "cost": "float ($)"
  }
}
```

## 4. Alert and Event Formats

```json
{
  "@type": "SystemAlert",
  "alertId": "UUID",
  "timestamp": "ISO8601 datetime",
  "severity": "info|warning|error|critical",
  "category": "water_quality|flow|equipment|environmental|safety",
  "message": "string",
  "location": "string",
  "parameters": {
    "threshold": "float",
    "actual": "float",
    "deviation": "float (%)"
  },
  "actionRequired": "boolean",
  "acknowledgedBy": "string (optional)",
  "acknowledgedAt": "ISO8601 datetime (optional)",
  "resolvedAt": "ISO8601 datetime (optional)"
}
```

## 5. Environmental Discharge Format

```json
{
  "@type": "DischargeEvent",
  "dischargeId": "UUID",
  "timestamp": "ISO8601 datetime",
  "location": {
    "outfall": "string",
    "receivingWater": "string",
    "latitude": "float",
    "longitude": "float"
  },
  "discharge": {
    "volume": "float (m³)",
    "duration": "integer (minutes)",
    "flowRate": "float (m³/s)"
  },
  "quality": "WaterQualityReport",
  "permit": {
    "permitNumber": "string",
    "limits": "object",
    "compliance": "boolean"
  },
  "conditions": {
    "weatherEvent": "dry|rain|storm",
    "upstreamFlow": "float (m³/s)",
    "dilutionFactor": "float"
  }
}
```

## 6. Validation Rules

1. All timestamps MUST use ISO 8601 format with timezone
2. All measurements MUST use SI units
3. All arrays MUST have consistent element types
4. All UUIDs MUST be version 4
5. Required fields MUST NOT be null
6. Enum values MUST match specification exactly
7. Quality parameters MUST include measurement uncertainty when available
8. Location data MUST use WGS84 coordinate system

## 7. Data Quality Indicators

```json
{
  "@type": "DataQuality",
  "accuracy": "float (±units)",
  "precision": "float",
  "completeness": "float (%)",
  "timeliness": "float (seconds delay)",
  "validity": "boolean",
  "qualityFlags": ["calibration_due", "sensor_drift", "anomaly_detected"]
}
```

## 8. Extensibility

Implementations MAY add custom fields prefixed with "x_" to avoid conflicts with future standard additions.

Example:
```json
{
  "@type": "SystemState",
  "flowRate": 4.2,
  "x_customParameter": "vendor-specific data",
  "x_localRegulation": "municipality-specific field"
}
```

---

© 2025 WIA · MIT License

## P.1 Data Format Cross-References

This Phase defines the canonical data types referenced by the API surface (Phase 2),
the wire protocol (Phase 3), and integration scenarios (Phase 4). Implementations
MUST round-trip every canonical type through serialization and deserialization
without loss of precision or semantics.

### P.1.1 Canonical Encoding Rules

1. UTF-8 is the required character encoding for textual fields.
2. Numeric fields use IEEE 754 binary64 unless explicitly marked as fixed-point.
3. Timestamps use RFC 3339 with timezone offset; durations use ISO 8601.
4. UUIDs follow RFC 4122 v4 unless deterministic IDs are required.
5. Binary payloads are encoded as Base64 (RFC 4648 §4) in JSON contexts and as
   raw octet strings in Protocol Buffers / CBOR contexts.

### P.1.2 Schema Evolution

Schema changes follow these compatibility classes:

| Class | Allowed Changes | Wire-Compat |
|-------|-----------------|-------------|
| Patch | Doc fixes, examples, validator tightening within existing range | Forward & backward |
| Minor | New optional fields, new enum values with default fallback        | Forward |
| Major | Field rename, type change, removal, semantics change              | None |

### P.1.3 Validation Order

Validators MUST apply checks in this order: (1) syntactic well-formedness,
(2) schema conformance, (3) cross-field invariants, (4) external referential
integrity, (5) policy / authorization. A failure short-circuits subsequent
checks; the response message identifies the first failing rule by ID.


---

## Appendix · Common WIA Standard Provisions

> The following provisions apply to every WIA standard and are kept in sync
> across the WIA-Standards corpus. Standard-specific deviations, where they
> exist, are listed in the standard's normative body.

### A. Conformance & Compliance

#### A.1 Conformance Levels

WIA-Standards defines four conformance levels:

| Level | Required | Description |
|-------|----------|-------------|
| L1 — Format | Phase 1 | Implementation produces and consumes the canonical data format losslessly |
| L2 — Interface | Phase 1 + 2 | Implementation exposes the API surface with required behaviour |
| L3 — Protocol | Phase 1 + 2 + 3 | Implementation interoperates over at least one normative transport binding |
| L4 — Integration | All Phases | Implementation passes the conformance test suite end-to-end in a production-shaped deployment |

Conformance claims MUST cite the level and the version of the standard
against which the claim is made (e.g. "L3 conformant against v1.0").

#### A.2 Compliance Verification

The conformance test suite is published alongside this standard at
`/cli/conformance/` and `/api/conformance/`. Implementations claiming
L2 or higher MUST publish their test report. Independent re-tests are
encouraged; the WIA Working Group accepts third-party verification reports
under the policy in §E.

### B. Security Considerations

#### B.1 Threat Model

Implementers SHOULD apply STRIDE analysis covering: spoofing of identity,
tampering with messages or stored state, repudiation of operations,
information disclosure, denial of service, and elevation of privilege.

| Threat | Default Control | Where Strengthened |
|--------|-----------------|--------------------|
| Spoofing | Mutual TLS or signed tokens | Phase 3 §P.3 |
| Tampering | TLS in transit, AEAD at rest | Phase 1 §P.1 |
| Repudiation | Append-only audit log with notarization | Phase 4 §P.4 |
| Disclosure | Field-level encryption for PII / secrets | Phase 1 §P.1 |
| DoS | Rate limit per principal & global circuit breaker | Phase 2 §P.2 |
| EoP | Least-privilege RBAC + scoped tokens | Phase 2 §P.2 |

#### B.2 Cryptographic Suites

Mandatory: TLS 1.3 with AEAD ciphers (AES-128-GCM, AES-256-GCM,
CHACHA20-POLY1305). Forbidden: TLS 1.0, TLS 1.1, RC4, MD5, SHA-1 for
signatures, RSA below 2048 bits, ECDSA on curves smaller than P-256.

Post-quantum migration: implementations SHOULD adopt hybrid key
exchange combining a classical primitive with ML-KEM (FIPS 203) once a
profile is published; signature migration to ML-DSA (FIPS 204) is
expected within the L4 conformance window of v2.0.

#### B.3 Audit Requirements

L3 and L4 implementations MUST log: (a) every authentication decision,
(b) every authorization decision, (c) every state-changing operation,
(d) every export of data outside its sovereignty boundary. Logs are
write-once for at least 1 year and 90 days indexed for incident search.

### C. Versioning & Lifecycle

Versions follow Semantic Versioning 2.0.0 (MAJOR.MINOR.PATCH).

| Phase | Duration | Conformance Status |
|-------|---------:|-------------------|
| Draft | until ratification | Non-binding |
| Active | indefinite | Binding for new deployments |
| Maintenance | 24 months from successor's Active date | Binding for existing deployments |
| Retired | indefinite | Non-binding; conformance claims rescinded |

Deprecation MUST be announced at least one minor version before a feature
is removed in a major version.

### D. Internationalization & Accessibility

Implementations SHOULD support locale negotiation via the
`Accept-Language` header (RFC 4647). Date, time, number, and currency
formatting follow CLDR. User-facing surfaces MUST satisfy WCAG 2.1 AA at
minimum and SHOULD progress towards WCAG 2.2 AA. Right-to-left scripts
(Arabic, Hebrew, Persian, Urdu) and East-Asian wide characters MUST be
laid out correctly without line-breaking heuristics that split graphemes.

### E. Governance & IP Policy

This standard is maintained by the WIA Working Group under the WIA
governance charter. Editorial changes are merged via pull request. Normative
changes require working-group consensus and a 30-day public review.
Contributions are accepted under the Apache License 2.0 with explicit
patent grant. Members participate under the WIA Patent Policy,
which requires royalty-free licensing of any essential claim necessary
to implement a normative requirement.

### F. Normative References

The following references are normative; implementations MUST satisfy
the cited clauses:

- ISO/IEC 27001:2022 — Information security management systems
- ISO/IEC 27017:2015 — Cloud-services security controls
- ISO/IEC 27701:2019 — Privacy information management
- ISO/IEC 19790:2012 — Security requirements for cryptographic modules
- ISO 8601-1:2019 — Date and time representation
- IETF RFC 8446 — TLS 1.3
- IETF RFC 7519 — JSON Web Token
- IETF RFC 6749 — OAuth 2.0
- IETF RFC 9110 — HTTP Semantics
- IETF RFC 9112 — HTTP/1.1 message syntax
- IETF RFC 9113 — HTTP/2
- IETF RFC 9114 — HTTP/3
- IETF RFC 9000 — QUIC transport
- IETF RFC 4122 — UUID URN namespace
- IETF RFC 3339 — Date and time on the Internet
- IETF RFC 6838 — Media-type specifications and registration
- W3C TraceContext — Distributed tracing context
- W3C WCAG 2.1 — Accessibility guidelines
- FIPS PUB 197 — AES
- FIPS PUB 180-4 — SHA-2 family
- FIPS PUB 203 — ML-KEM (post-quantum KEM)
- FIPS PUB 204 — ML-DSA (post-quantum signature)

### G. Glossary

| Term | Definition |
|------|------------|
| Conformance | The state of satisfying every normative requirement at a given level |
| Implementation | A software, hardware, or composite artefact that claims conformance |
| Principal | The authenticated entity bound to a security context |
| Subject | The resource or person to which an operation applies |
| Sovereignty Boundary | The legal / regulatory perimeter outside of which data export is restricted |

---

*This Appendix is authored by the WIA Standards Working Group and is kept
in lockstep across Phases 1–4 of sewage-system so that conformance claims at any
Phase remain unambiguous.*

