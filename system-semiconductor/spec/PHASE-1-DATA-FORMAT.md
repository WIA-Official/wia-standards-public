# WIA-SEMI-001: Phase 1 - Data Format Specification

Version: 1.0
Status: Final
Date: 2025-01-15

## Overview

Phase 1 of WIA-SEMI-001 establishes standardized data formats for semiconductor specifications, enabling machine-readable, consistent documentation across the industry. This phase forms the foundation for all subsequent phases.

## Design Principles

1. **Machine Readability**: All data must be parseable by automated tools
2. **Human Friendliness**: Formats should be editable by engineers
3. **Validation**: Strict schemas prevent invalid data
4. **Completeness**: Capture all necessary information
5. **Versioning**: Clear version identification and evolution

## Core Data Format

All WIA-SEMI-001 specifications use JSON format with strict schema validation.

### Root Schema Structure

```json
{
  "$schema": "https://wia.org/schemas/semi-001/v1/chip-spec.json",
  "wiaVersion": "1.0",
  "metadata": { ... },
  "architecture": { ... },
  "power": { ... },
  "thermal": { ... },
  "interfaces": { ... },
  "performance": { ... }
}
```

## Metadata Specification

### Required Fields

| Field | Type | Description | Example |
|-------|------|-------------|---------|
| id | string | Globally unique identifier | "WIA-SOC-2025-001" |
| manufacturer | object | Manufacturer information | See below |
| productName | string | Commercial product name | "ExampleSoC Gen 5" |
| productFamily | string | Product family | "ExampleSoC Gen 5" |
| marketSegment | array | Target markets | ["mobile", "tablet"] |
| releaseDate | ISO 8601 | Release date | "2025-03-15" |
| productionStatus | enum | Lifecycle status | "active" |

### Manufacturer Object

```json
{
  "manufacturer": {
    "name": "Example Semiconductor Corp",
    "division": "Mobile Solutions",
    "contact": "support@example.com",
    "website": "https://example.com"
  }
}
```

### Production Status Values

- `announced`: Product announced but not sampling
- `sampling`: Engineering samples available
- `active`: In mass production
- `eol-announced`: End-of-life announced
- `eol`: No longer manufactured

## Architecture Specification

### CPU Architecture

```json
{
  "cpu": {
    "clusters": [
      {
        "name": "Performance",
        "coreCount": 2,
        "microarchitecture": "Cortex-X5",
        "frequency": {
          "min": 1.0,
          "max": 3.8,
          "unit": "GHz"
        },
        "cache": {
          "l1i": {"size": 64, "unit": "KB"},
          "l1d": {"size": 64, "unit": "KB"},
          "l2": {"size": 2, "unit": "MB"}
        },
        "features": ["out-of-order", "SMT", "vector-extensions"]
      }
    ],
    "sharedCache": {
      "l3": {"size": 16, "unit": "MB"}
    }
  }
}
```

### GPU Architecture

```json
{
  "gpu": {
    "architecture": "Immortalis-G925",
    "computeUnits": 16,
    "frequency": {
      "min": 400,
      "max": 1150,
      "unit": "MHz"
    },
    "features": ["ray-tracing", "variable-rate-shading"],
    "apis": ["Vulkan 1.3", "OpenGL ES 3.2"]
  }
}
```

### NPU Architecture

```json
{
  "npu": {
    "architecture": "Custom-AI-v5",
    "performance": {
      "value": 50,
      "unit": "TOPS"
    },
    "precision": ["INT4", "INT8", "FP16", "BF16"],
    "frameworks": ["TensorFlow Lite", "PyTorch Mobile"]
  }
}
```

## Power Specification

### TDP and Power Rails

```json
{
  "power": {
    "tdp": {
      "typical": {"value": 8.5, "unit": "W"},
      "maximum": {"value": 12.0, "unit": "W"}
    },
    "voltageRails": [
      {
        "name": "VDD_CPU",
        "nominal": 0.85,
        "min": 0.65,
        "max": 1.05,
        "unit": "V",
        "tolerance": 0.05
      }
    ],
    "powerStates": [
      {
        "name": "Active-Max",
        "cpuFrequency": 3.8,
        "gpuFrequency": 1150,
        "power": {"typical": 12.0, "unit": "W"}
      },
      {
        "name": "Idle",
        "cpuFrequency": 0.5,
        "gpuFrequency": 400,
        "power": {"typical": 0.8, "unit": "W"}
      }
    ]
  }
}
```

### Power Management Features

- **DVFS**: Dynamic Voltage and Frequency Scaling
- **Power Gating**: Turning off unused components
- **Clock Gating**: Stopping clocks to idle components
- **Adaptive Voltage**: Adjusting voltage based on process variation

## Thermal Specification

```json
{
  "thermal": {
    "junctionTemperature": {
      "typical": 75,
      "maximum": 105,
      "unit": "°C"
    },
    "thermalResistance": {
      "junctionToCase": {"value": 0.8, "unit": "°C/W"}
    },
    "hotspots": [
      {
        "location": "CPU-Performance-Cluster",
        "peakTemperature": 95,
        "area": {"value": 8.5, "unit": "mm²"}
      }
    ],
    "thermalThrottling": {
      "enabled": true,
      "thresholds": [
        {
          "temperature": 95,
          "action": "reduce-frequency-25%",
          "unit": "°C"
        }
      ]
    }
  }
}
```

## Performance Metrics

### Standard Benchmarks

All performance claims must specify benchmark, version, and test conditions:

```json
{
  "performance": {
    "cpu": {
      "singleCore": {
        "benchmark": "Geekbench 6.2",
        "score": 2850,
        "testConditions": "Max frequency, typical power"
      },
      "multiCore": {
        "benchmark": "Geekbench 6.2",
        "score": 9200,
        "testConditions": "All cores active, sustained"
      }
    },
    "gpu": {
      "graphics": {
        "benchmark": "GFXBench 5.0 Manhattan 3.1.1",
        "offscreen": {"value": 245, "unit": "fps"},
        "conditions": {
          "resolution": "1080p",
          "api": "Vulkan 1.3"
        }
      }
    }
  }
}
```

## Type Definitions

### Frequency Type

```json
{
  "type": "object",
  "properties": {
    "value": {"type": "number", "minimum": 0},
    "unit": {"type": "string", "enum": ["Hz", "kHz", "MHz", "GHz"]}
  },
  "required": ["value", "unit"]
}
```

### Power Type

```json
{
  "type": "object",
  "properties": {
    "value": {"type": "number", "minimum": 0},
    "unit": {"type": "string", "enum": ["mW", "W"]}
  },
  "required": ["value", "unit"]
}
```

### Temperature Type

```json
{
  "type": "object",
  "properties": {
    "value": {"type": "number"},
    "unit": {"type": "string", "enum": ["°C", "°F", "K"]}
  },
  "required": ["value", "unit"]
}
```

## Validation

All specifications must pass JSON Schema validation:

```bash
wia-validator validate --schema chip-spec-v1.json --input my-chip.json
```

### Validation Rules

1. All required fields must be present
2. Values must match specified types
3. Enumerations must use approved values
4. Units must be specified for all measurements
5. Cross-field validation (e.g., min < max)

## Extensibility

Vendors can add custom fields using the `x-` prefix:

```json
{
  "x-vendor-specific": {
    "customFeature": "value"
  }
}
```

Extensions must not conflict with standard fields and should be documented.

## Version Management

- Specifications include `wiaVersion` field
- Semantic versioning: MAJOR.MINOR.PATCH
- Breaking changes increment MAJOR version
- New features increment MINOR version
- Bug fixes increment PATCH version

## Compliance Requirements

To be Phase 1 compliant:

1. ✓ All chip specifications in WIA JSON format
2. ✓ Pass schema validation without errors
3. ✓ Include all required fields with accurate data
4. ✓ Use standard units and enumerations
5. ✓ Maintain specification version control

## Tools and Resources

- **Schema Files**: https://wia.org/schemas/semi-001/
- **Validator**: `npm install -g wia-semi-validator`
- **Examples**: https://github.com/WIA-Official/wia-standards/examples
- **Documentation**: https://docs.wia.org/semi-001/phase-1

## Migration Guide

### From PDF Documentation

1. Extract structured data from PDF specifications
2. Map fields to WIA schema
3. Validate against schema
4. Fill in any missing required fields
5. Publish WIA-compliant JSON

### From Proprietary Formats

1. Write conversion script using WIA SDK
2. Validate output
3. Review and correct automated conversion
4. Establish ongoing conversion process

## Benefits of Phase 1 Adoption

- **Immediate**: Better data exchange with customers
- **Automated Tools**: Enable specification parsing and analysis
- **Comparisons**: Facilitate accurate cross-vendor comparisons
- **Low Cost**: Minimal implementation cost, high value

---

**Next**: [Phase 2 - API Interface Standards](PHASE-2-API-INTERFACE.md)

© 2025 SmileStory Inc. / WIA - World Certification Industry Association
弘益人間 (Hongik Ingan) - Benefit All Humanity

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
in lockstep across Phases 1–4 of system-semiconductor so that conformance claims at any
Phase remain unambiguous.*

