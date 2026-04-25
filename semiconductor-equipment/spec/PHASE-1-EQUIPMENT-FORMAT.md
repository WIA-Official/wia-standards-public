# WIA-SEMI-019 - Phase 1: Equipment Data Format

> **Version:** 1.0  
> **Status:** Active  
> **Last Updated:** 2025-12-26

## 1. Overview

Phase 1 defines the foundational data format for describing semiconductor equipment specifications, capabilities, and parameters. All WIA-SEMI-019 compliant equipment must provide machine-readable specifications in standardized JSON format.

### 1.1 Purpose

- Enable automated equipment discovery and capability assessment
- Provide semantic interoperability across vendors
- Support intelligent equipment selection and scheduling
- Facilitate data-driven analytics and optimization

### 1.2 Scope

Phase 1 covers:
- Equipment specification schema
- Parameter naming conventions
- Data type definitions
- Unit standardization
- Equipment-specific extensions (lithography, etch, deposition, inspection, CMP)

## 2. Equipment Specification Schema

### 2.1 Core Schema Structure

```json
{
  "standard": "WIA-SEMI-019",
  "version": "1.0",
  "last_updated": "2025-12-26T00:00:00Z",
  "equipment": {
    "manufacturer": "string",
    "model": "string",
    "serial_number": "string",
    "type": "string",
    "subtype": "string",
    "installation_date": "ISO8601",
    "wafer_size_mm": [200, 300],
    "process_node_nm": [3, 5, 7],
    "chambers": "integer",
    "load_ports": "integer"
  },
  "capabilities": {},
  "parameters": {},
  "interfaces": {},
  "certification": {}
}
```

### 2.2 Equipment Types

Standardized equipment type values:

- `lithography`: Lithography systems (EUV, DUV, i-line)
- `etch`: Etch equipment (Plasma, RIE, wet etch)
- `deposition`: Deposition tools (CVD, ALD, PVD, epitaxy)
- `inspection`: Inspection and metrology (e-beam, optical, x-ray)
- `cmp`: Chemical mechanical planarization
- `implant`: Ion implantation
- `cleaning`: Wafer cleaning equipment
- `metrology`: Standalone metrology tools
- `anneal`: Thermal processing equipment
- `photolithography`: Coater/developer tracks

### 2.3 Capabilities Object

Equipment-specific capabilities describing performance characteristics:

```json
"capabilities": {
  "technology": "string",
  "throughput_wph": "float",
  "max_wafer_size_mm": "integer",
  "min_feature_size_nm": "float",
  "uniformity_percent": "float",
  "process_temperature_range_celsius": {
    "min": "float",
    "max": "float"
  },
  "materials_supported": ["array of strings"],
  "automation_level": "string"
}
```

## 3. Parameter Dictionary

### 3.1 Naming Conventions

All parameters follow the pattern: `{category}_{parameter}_{unit}`

Rules:
- All lowercase
- Words separated by underscores
- Units always specified
- No abbreviations unless industry-standard (rf, uv, ir)
- Maximum 64 characters

Examples:
- `substrate_temperature_celsius`
- `chamber_pressure_pascal`
- `rf_power_forward_watts`
- `gas_flow_n2_sccm`

### 3.2 Standard Units

| Physical Quantity | Standard Unit | Symbol |
|-------------------|---------------|--------|
| Temperature | Celsius | celsius |
| Pressure | Pascal | pascal |
| Flow Rate | sccm | sccm |
| Power | Watts | watts |
| Voltage | Volts | volts |
| Current | Amperes | amperes |
| Frequency | Hertz | hertz |
| Time | Seconds | seconds |
| Length | Meters | meters |
| Length (small) | Nanometers | nanometers |

### 3.3 Data Types

| Type | Description | Example |
|------|-------------|---------|
| float | Floating-point number | 425.3 |
| integer | Integer number | 100 |
| boolean | True/false | true |
| string | Text string | "WAFER_ID_123" |
| timestamp | ISO 8601 datetime | "2025-12-26T15:30:45.123Z" |
| array | Array of values | [1, 2, 3, 4, 5] |
| object | Nested object | {"key": "value"} |

### 3.4 Parameter Categories

Parameters are organized into categories:

1. **Process Parameters** (4000-4999): Core process control
   - Temperature, pressure, flow, power, time
   - Recipe setpoints and targets

2. **Sensor Parameters** (5000-5999): Measured values
   - Actual temperatures, pressures, flows
   - Real-time measurements

3. **Control Parameters** (6000-6999): Equipment control
   - Valve positions, motor speeds, heater power
   - Hardware control signals

4. **Metrology Parameters** (7000-7999): Measurement results
   - Film thickness, overlay, CD, defect counts
   - Quality metrics

5. **Status Parameters** (8000-8999): Equipment status
   - States, alarms, events, health
   - Operational status

## 4. Equipment-Specific Extensions

### 4.1 Lithography Equipment

```json
{
  "lithography_specific": {
    "exposure": {
      "wavelength_nm": "float",
      "dose_mj_cm2": "float",
      "exposure_time_ms": "float",
      "numerical_aperture": "float",
      "slit_width_mm": "float",
      "scan_speed_mm_s": "float"
    },
    "overlay": {
      "overlay_x_nm": "float",
      "overlay_y_nm": "float",
      "overlay_3sigma_nm": "float",
      "alignment_marks": "integer",
      "alignment_residual_nm": "float"
    },
    "focus": {
      "focus_offset_nm": "float",
      "focus_depth_nm": "float",
      "tilt_x_nm": "float",
      "tilt_y_nm": "float",
      "leveling_residual_nm": "float"
    },
    "reticle": {
      "reticle_id": "string",
      "magnification": "float",
      "pellicle_installed": "boolean",
      "inspection_date": "timestamp"
    }
  }
}
```

### 4.2 Etch Equipment

```json
{
  "etch_specific": {
    "plasma": {
      "rf_power_top_watts": "float",
      "rf_power_bottom_watts": "float",
      "rf_frequency_mhz": "float",
      "dc_bias_volts": "float",
      "plasma_density_cm3": "float"
    },
    "process": {
      "etch_rate_nm_min": "float",
      "selectivity_ratio": "float",
      "uniformity_percent": "float",
      "aspect_ratio_max": "float",
      "sidewall_angle_degrees": "float"
    },
    "gases": {
      "gas_chemistry": "string",
      "total_flow_sccm": "float"
    }
  }
}
```

### 4.3 Deposition Equipment

```json
{
  "deposition_specific": {
    "film": {
      "material": "string",
      "target_thickness_nm": "float",
      "measured_thickness_nm": "float",
      "uniformity_percent": "float",
      "step_coverage_percent": "float",
      "stress_mpa": "float",
      "refractive_index": "float"
    },
    "process": {
      "deposition_rate_nm_min": "float",
      "temperature_celsius": "float",
      "pressure_pascal": "float",
      "method": "string"
    }
  }
}
```

### 4.4 Inspection Equipment

```json
{
  "inspection_specific": {
    "method": "string",
    "resolution_nm": "float",
    "defect_sensitivity_nm": "float",
    "throughput_wph": "float",
    "inspection_area_percent": "float",
    "defects": {
      "total_count": "integer",
      "critical_count": "integer",
      "defect_types": {}
    },
    "classification": {
      "nuisance_percent": "float",
      "systematic_percent": "float",
      "random_percent": "float"
    }
  }
}
```

## 5. Data Quality Indicators

All parameter values include quality metadata:

```json
{
  "parameter": "substrate_temperature_celsius",
  "value": 425.3,
  "timestamp": "2025-12-26T15:30:45.123Z",
  "quality": {
    "status": "GOOD",
    "confidence": 0.98,
    "sensor_health": "NORMAL",
    "calibration_date": "2025-12-01",
    "calibration_due": "2026-03-01",
    "out_of_spec": false,
    "error_code": null
  }
}
```

Quality Status Values:
- `GOOD`: Parameter value is valid and reliable
- `UNCERTAIN`: Parameter may be questionable
- `BAD`: Parameter value is invalid or unreliable
- `UNKNOWN`: Quality cannot be determined

## 6. Validation Rules

### 6.1 Required Fields

All equipment specifications must include:
- `standard`, `version`, `last_updated`
- `equipment.manufacturer`, `equipment.model`, `equipment.type`
- `parameters` object with count of each category
- `interfaces` object indicating supported protocols

### 6.2 Data Range Validation

Parameters must include valid ranges:

```json
{
  "parameter_id": "substrate_temp_celsius",
  "range": {
    "min": 20,
    "max": 500,
    "normal_min": 350,
    "normal_max": 450
  },
  "precision": 0.1,
  "resolution": 0.01
}
```

### 6.3 JSON Schema Validation

Equipment specifications must validate against the WIA-SEMI-019 JSON Schema available at:
`https://standards.wia.org/semi-019/v1.0/equipment-spec.schema.json`

## 7. Implementation Guidelines

### 7.1 Equipment Specification File

Each equipment instance must provide a specification file accessible via:
- HTTP/HTTPS: `http://{equipment-ip}/wia/equipment-spec.json`
- SECS S9F100/S9F101: WIA Equipment Specification Request
- File system: `/wia/equipment-spec.json`

### 7.2 Parameter Discovery

Equipment must support parameter enumeration:
- List all available parameters with metadata
- Indicate which parameters are readable vs. writable
- Provide update frequency for sensor parameters

### 7.3 Custom Parameters

Vendor-specific parameters not in standard dictionary must:
- Follow naming conventions
- Include full metadata (description, unit, range, type)
- Use parameter IDs >=10000 to avoid conflicts
- Document in equipment specification

## 8. Bronze Certification Requirements

To achieve Bronze certification, equipment must:

1. Provide valid equipment specification JSON file
2. Use standardized parameter names for at least 80% of parameters
3. Follow naming conventions for custom parameters
4. Include data quality indicators
5. Validate against JSON schema
6. Document all parameters with complete metadata
7. Support parameter discovery via at least one interface

## 9. Examples

### 9.1 Complete Equipment Specification

```json
{
  "standard": "WIA-SEMI-019",
  "version": "1.0",
  "last_updated": "2025-12-26T10:00:00Z",
  "equipment": {
    "manufacturer": "Applied Materials",
    "model": "Centura 5200",
    "serial_number": "AMAT-2025-001",
    "type": "deposition",
    "subtype": "pecvd",
    "installation_date": "2025-01-15",
    "wafer_size_mm": [200, 300],
    "process_node_nm": [5, 7, 10, 14],
    "chambers": 4,
    "load_ports": 2
  },
  "capabilities": {
    "technology": "PECVD",
    "throughput_wph": 100,
    "materials_supported": ["SiO2", "SiN", "SiON"],
    "max_temperature_celsius": 450,
    "uniformity_percent": 1.5,
    "step_coverage_percent": 95
  },
  "parameters": {
    "process": 127,
    "sensor": 438,
    "control": 156,
    "metrology": 45,
    "status": 89
  },
  "interfaces": {
    "secs_gem": true,
    "rest_api": true,
    "websocket": true,
    "e84": true
  },
  "certification": {
    "wia_level": "Silver",
    "certified_date": "2025-11-10",
    "certificate_id": "WIA-SEMI-019-AMAT-001",
    "valid_until": "2026-11-10"
  }
}
```

---

**弘益人間 · Benefit All Humanity**

*WIA - World Certification Industry Association*  
*© 2025 MIT License*

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
in lockstep across Phases 1–4 of semiconductor-equipment so that conformance claims at any
Phase remain unambiguous.*

