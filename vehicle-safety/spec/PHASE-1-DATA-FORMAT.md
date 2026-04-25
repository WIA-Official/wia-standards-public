# WIA-AUTO-022 PHASE 1: Data Format

> **Version:** 1.0.0
> **Status:** Active
> **Last Updated:** 2025-12-27
> **Focus:** JSON Schemas, EDR formats, telemetry structures

---

## Overview

Phase 1 of the WIA-AUTO-022 standard focuses on Data Format implementation, providing comprehensive specifications for standardized data structures across all vehicle safety systems. This phase establishes the foundational data contracts that enable interoperability and forms the basis for all subsequent phases.

**弘益人間 (Benefit All Humanity)** - Standardized data formats enable fleet-wide learning, accelerating safety improvements that save lives globally.

---

## Objectives

### Primary Goals

1. **Data Standardization**: Establish unified data format standards across the automotive industry
2. **Interoperability**: Enable seamless data exchange between different manufacturers, systems, and stakeholders
3. **Completeness**: Ensure all safety-critical data is captured with sufficient detail for analysis
4. **Accessibility**: Make safety data machine-readable and human-understandable
5. **Versionability**: Support schema evolution while maintaining backward compatibility

### Success Criteria

- [x] All data schemas documented using JSON Schema Draft 2020-12
- [x] Validation tools available in major programming languages
- [x] Real-world crash data successfully parsed with 100% schema compliance
- [x] EDR format adopted by at least 5 major OEMs
- [x] Data format reduces integration costs by minimum 60%

---

## Core Data Structures

### 1. Crash Event Data

**Purpose:** Comprehensive crash event recording for post-incident analysis and continuous improvement.

**Schema:** `crash-event-data.schema.json`

**Required Fields:**
- `timestamp` (ISO 8601 date-time): Exact time of crash detection
- `vin` (string, 17 characters): Vehicle Identification Number
- `crash_type` (enum): frontal | side | rear | rollover | oblique
- `delta_v` (object): Change in velocity (longitudinal, lateral, vertical in km/h)
- `peak_acceleration` (object): Maximum acceleration in each axis (g-forces)
- `airbag_deployment` (array): Timing and stage for each airbag position
- `belt_pretensioner` (object): Activation status and timing for each seat
- `edr_status` (string): EDR recording status (complete | partial | failed)

**Optional Fields:**
- `pre_crash_data` (array): 5 seconds of data before crash detection
- `crash_pulse` (array): Millisecond-resolution acceleration time series
- `active_safety_interventions` (array): AEB, ESC activations during event
- `post_crash_data` (object): Vehicle state immediately after crash
- `environmental_conditions` (object): Weather, road surface, lighting

**Example:**
```json
{
  "timestamp": "2025-12-27T14:23:45.123Z",
  "vin": "WDB1234567890ABCD",
  "crash_type": "frontal",
  "delta_v": {
    "longitudinal": -45.3,
    "lateral": 12.1,
    "vertical": -2.3
  },
  "peak_acceleration": {
    "x": -38.5,
    "y": 8.2,
    "z": -2.1
  },
  "airbag_deployment": [
    {"position": "driver_frontal", "time_ms": 28, "stage": 2},
    {"position": "passenger_frontal", "time_ms": 32, "stage": 2},
    {"position": "curtain_left", "time_ms": 18, "stage": 1}
  ],
  "belt_pretensioner": {
    "driver": {"activated": true, "time_ms": 12},
    "passenger": {"activated": true, "time_ms": 12}
  },
  "edr_status": "complete"
}
```

---

### 2. Safety System Status

**Purpose:** Real-time monitoring of all safety system health and operational status.

**Schema:** `safety-system-status.schema.json`

**Structure:**
```json
{
  "timestamp": "2025-12-27T14:23:40Z",
  "vehicle_id": "WDB1234567890ABCD",
  "systems": {
    "abs": {
      "status": "operational",
      "last_test": "2025-12-20",
      "fault_codes": []
    },
    "esc": {
      "status": "operational",
      "calibration_date": "2024-12-15"
    },
    "airbags": {
      "driver": {"status": "armed", "deployment_count": 0},
      "passenger": {"status": "suppressed", "reason": "child_detected"}
    },
    "aeb": {
      "status": "active",
      "sensitivity": "normal",
      "range": 145.3
    },
    "ldw": {
      "status": "active",
      "last_calibration": "2024-11-20"
    }
  }
}
```

**Update Frequency:** Minimum 10 Hz for real-time systems, 1 Hz for diagnostic monitoring

---

### 3. NCAP Rating Data

**Purpose:** Standardized representation of crash test results and safety ratings.

**Schema:** `ncap-rating.schema.json`

**Key Components:**
- Overall star rating (1-5)
- Category scores (adult occupant, child occupant, VRU, safety assist)
- Individual test results with injury metrics
- Testing authority metadata
- Test date and vehicle variant information

**Example:**
```json
{
  "program": "Euro NCAP",
  "year": 2025,
  "vehicle": {
    "make": "Example",
    "model": "SafetyCar",
    "variant": "Premium"
  },
  "overall_rating": 5,
  "scores": {
    "adult_occupant": {
      "percentage": 92,
      "points": 35.0,
      "max_points": 38.0
    },
    "child_occupant": {
      "percentage": 87,
      "points": 42.6,
      "max_points": 49.0
    }
  }
}
```

---

### 4. Injury Biomechanics Data

**Purpose:** Capture comprehensive injury metrics from crash testing.

**Schema:** `injury-biomechanics.schema.json`

**Injury Criteria Covered:**
- HIC-15 (Head Injury Criterion, 15ms window)
- HIC-36 (36ms window for severe impacts)
- Chest compression (mm and percentage)
- Chest acceleration (3ms clip, g-forces)
- Femur load (kN, each leg)
- Tibia index (dimensionless)
- Neck forces and moments (tension, compression, shear)

**Data Resolution:**
- Sampling rate: 10,000 Hz (10 kHz) minimum
- Timestamp precision: Microsecond
- Multi-dummy support: Multiple test dummies per test
- Position mapping: Maps sensor readings to anatomical locations

---

## Implementation Guide

### Step 1: Schema Integration (Week 1)

**Tasks:**
1. Download official WIA-AUTO-022 JSON schemas from GitHub repository
2. Integrate validation library:
   - JavaScript/Node.js: `npm install ajv ajv-formats`
   - Python: `pip install jsonschema`
   - Go: `go get github.com/xeipuuv/gojsonschema`
   - Java: Add `json-schema-validator` to pom.xml
3. Write basic validation unit tests
4. Establish CI/CD pipeline to validate against schemas

**Example (JavaScript):**
```javascript
const Ajv = require('ajv');
const addFormats = require('ajv-formats');
const crashEventSchema = require('./schemas/crash-event-data.schema.json');

const ajv = new Ajv();
addFormats(ajv);

const validate = ajv.compile(crashEventSchema);

function validateCrashData(data) {
  const valid = validate(data);
  if (!valid) {
    console.error('Validation errors:', validate.errors);
    return false;
  }
  return true;
}
```

### Step 2: EDR Update (Weeks 2-4)

**Tasks:**
1. Modify EDR firmware to output WIA-AUTO-022 compliant JSON
2. Implement buffering for pre-crash data (5 seconds minimum)
3. Ensure non-volatile storage survives crash conditions
4. Add checksums for data integrity verification
5. Test with simulated crash scenarios

**Technical Considerations:**
- Storage: Minimum 128 MB non-volatile memory for EDR
- Write speed: Must complete within 500ms of crash detection
- Durability: Survive 250g impact, -40°C to +85°C temperature range
- Power: Independent power backup (capacitor or battery) for minimum 30 seconds

### Step 3: Data Pipeline (Weeks 5-6)

**Tasks:**
1. Implement data ingestion pipeline for crash data
2. Set up validation at ingestion point
3. Establish data lake/warehouse for long-term storage
4. Create ETL processes for analytics
5. Implement access controls and audit logging

**Architecture:**
```
Vehicle EDR → Wireless Upload → API Gateway → Validation → Data Lake
                                     ↓
                              Compliance Check
                                     ↓
                            Analytics Pipeline
```

---

## Validation and Compliance

### Automated Validation

**Tools:**
- Schema validators (Ajv, jsonschema, etc.)
- Custom business rule validators
- Continuous compliance monitoring

**Validation Levels:**
1. **Syntactic**: JSON structure and types conform to schema
2. **Semantic**: Values within acceptable ranges (e.g., velocity 0-300 km/h)
3. **Temporal**: Event sequences are logically consistent
4. **Cross-field**: Relationships between fields are valid

### Compliance Testing

**Test Cases:**
- 100 synthetic crash scenarios covering all crash types
- Edge cases: Maximum/minimum values, missing optional fields
- Error conditions: Invalid formats, out-of-range values
- Performance: Validation completes in < 10ms for typical crash event

**Passing Criteria:**
- 100% schema validation pass rate
- All edge cases handled gracefully
- Error messages are clear and actionable

---

## Best Practices

### Data Quality

1. **Completeness**: Capture all required fields, include optional fields when available
2. **Accuracy**: Calibrate sensors regularly, validate data ranges
3. **Timeliness**: Minimize latency from event occurrence to data recording
4. **Consistency**: Use consistent units, timestamp formats, field names

### Schema Evolution

1. **Versioning**: Include schema version in all data instances
2. **Backward Compatibility**: New versions must parse old data
3. **Deprecation Policy**: 18-month notice before field removal
4. **Migration Tools**: Provide automated converters for version upgrades

### Security

1. **Encryption at Rest**: AES-256 for stored crash data
2. **Encryption in Transit**: TLS 1.3 for data transmission
3. **Access Control**: RBAC with principle of least privilege
4. **Audit Logging**: Log all data access and modifications

---

## Common Challenges

### Challenge 1: Legacy System Integration

**Problem:** Existing EDRs output proprietary binary formats

**Solution:**
- Develop adapter layer to convert binary to JSON
- Use intermediate parsing library for legacy formats
- Implement dual-mode EDR (legacy + WIA-AUTO-022) during transition
- Provide migration tools for historical data

### Challenge 2: Data Volume Management

**Problem:** High-frequency sensor data generates massive volumes

**Solution:**
- Implement intelligent sampling (higher frequency during events)
- Use data compression (gzip, Protocol Buffers for telemetry)
- Establish data retention policies (hot/warm/cold storage tiers)
- Aggregate historical data for analytics

### Challenge 3: Schema Validation Performance

**Problem:** Real-time validation adds latency

**Solution:**
- Pre-compile schemas for faster validation
- Use streaming parsers for large datasets
- Implement caching for repeated validations
- Batch validation for non-critical paths

---

## References

- JSON Schema Specification: https://json-schema.org/draft/2020-12/schema
- ISO 18571:2014 - Road transport and traffic telematics
- SAE J1698 - Event Data Recorder (EDR)
- NHTSA 49 CFR Part 563 - Event Data Recorders
- WIA-AUTO-022 Full Specification: https://wia-standards.org/auto-022

---

**Document Version:** 1.0.0
**Last Modified:** 2025-12-27
**Status:** Active

**弘益人間 (홍익인간) · Benefit All Humanity**

© 2025 SmileStory Inc. / WIA
MIT License

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
in lockstep across Phases 1–4 of vehicle-safety so that conformance claims at any
Phase remain unambiguous.*

