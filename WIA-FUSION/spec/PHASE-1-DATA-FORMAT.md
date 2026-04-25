# WIA-FUSION Phase 1: Data Format Specification

**Version:** 1.0.0
**Status:** Official
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

This specification defines the standardized data formats for nuclear fusion energy systems, enabling interoperability between different reactor types (Tokamak, Stellarator, Laser Fusion) and research institutions worldwide.

### 1.1 Scope

- Plasma state representation
- Energy output metrics
- Diagnostic data formats
- Control system parameters
- Safety monitoring data

### 1.2 Design Principles

1. **Technology Agnostic**: Support all fusion approaches
2. **Real-time Capable**: Sub-millisecond data updates
3. **Extensible**: Custom fields for experimental data
4. **Interoperable**: Compatible with ITER, KSTAR, JET systems

---

## 2. Plasma State Schema

### 2.1 Core Schema Definition

```json
{
  "$schema": "https://wia.live/schemas/fusion/v1.0.0",
  "type": "object",
  "required": ["plasma_state"],
  "properties": {
    "plasma_state": {
      "type": "object",
      "required": ["shot_id", "timestamp", "reactor", "core_parameters"],
      "properties": {
        "shot_id": {
          "type": "string",
          "pattern": "^[A-Z]+-[0-9]+$",
          "description": "Unique identifier for plasma shot"
        },
        "timestamp": {
          "type": "string",
          "format": "date-time",
          "description": "ISO8601 timestamp"
        },
        "reactor": {
          "type": "string",
          "enum": ["ITER", "KSTAR", "JET", "EAST", "SPARC", "W7X", "NIF", "custom"],
          "description": "Reactor identification"
        },
        "core_parameters": {
          "$ref": "#/definitions/CoreParameters"
        },
        "performance": {
          "$ref": "#/definitions/Performance"
        },
        "stability": {
          "$ref": "#/definitions/Stability"
        },
        "control": {
          "$ref": "#/definitions/Control"
        }
      }
    }
  }
}
```

### 2.2 Core Parameters

```json
{
  "definitions": {
    "CoreParameters": {
      "type": "object",
      "properties": {
        "temperature_keV": {
          "type": "object",
          "properties": {
            "ion": { "type": "number", "minimum": 0, "maximum": 100 },
            "electron": { "type": "number", "minimum": 0, "maximum": 100 }
          },
          "description": "Plasma temperature in keV (1 keV ≈ 11.6 million °C)"
        },
        "density_m3": {
          "type": "object",
          "properties": {
            "value": { "type": "number", "minimum": 0 },
            "unit": { "type": "string", "const": "1e20/m3" }
          },
          "description": "Plasma density in units of 10²⁰ particles per cubic meter"
        },
        "confinement_time_s": {
          "type": "number",
          "minimum": 0,
          "description": "Energy confinement time in seconds"
        },
        "triple_product": {
          "type": "object",
          "properties": {
            "value": { "type": "number" },
            "unit": { "type": "string", "const": "keV·s·1e20/m3" }
          },
          "description": "Fusion triple product (n × T × τ)"
        }
      }
    }
  }
}
```

### 2.3 Performance Metrics

```json
{
  "definitions": {
    "Performance": {
      "type": "object",
      "properties": {
        "q_factor": {
          "type": "number",
          "minimum": 0,
          "description": "Fusion gain factor (P_fusion / P_input)"
        },
        "fusion_power_mw": {
          "type": "number",
          "minimum": 0,
          "description": "Total fusion power output in megawatts"
        },
        "plasma_current_ma": {
          "type": "number",
          "description": "Plasma current in mega-amperes"
        },
        "beta_percent": {
          "type": "number",
          "minimum": 0,
          "maximum": 10,
          "description": "Ratio of plasma pressure to magnetic pressure"
        },
        "bootstrap_fraction": {
          "type": "number",
          "minimum": 0,
          "maximum": 1,
          "description": "Self-generated current fraction"
        }
      }
    }
  }
}
```

### 2.4 Stability Parameters

```json
{
  "definitions": {
    "Stability": {
      "type": "object",
      "properties": {
        "disruption_risk": {
          "type": "object",
          "properties": {
            "value": { "type": "number", "minimum": 0, "maximum": 1 },
            "confidence": { "type": "number", "minimum": 0, "maximum": 1 }
          },
          "description": "AI-predicted disruption probability"
        },
        "elm_frequency_hz": {
          "type": "number",
          "minimum": 0,
          "description": "Edge Localized Mode frequency"
        },
        "mhd_activity": {
          "type": "array",
          "items": {
            "type": "string",
            "pattern": "^m=[0-9]+,n=[0-9]+$"
          },
          "description": "Active MHD modes (m,n notation)"
        },
        "locked_mode": {
          "type": "boolean",
          "description": "Locked mode detection"
        },
        "vertical_displacement": {
          "type": "number",
          "description": "Vertical displacement event indicator"
        }
      }
    }
  }
}
```

### 2.5 Control Parameters

```json
{
  "definitions": {
    "Control": {
      "type": "object",
      "properties": {
        "heating_power_mw": {
          "type": "object",
          "properties": {
            "ohmic": { "type": "number", "minimum": 0 },
            "nbi": { "type": "number", "minimum": 0, "description": "Neutral Beam Injection" },
            "icrh": { "type": "number", "minimum": 0, "description": "Ion Cyclotron Resonance Heating" },
            "ecrh": { "type": "number", "minimum": 0, "description": "Electron Cyclotron Resonance Heating" },
            "lhcd": { "type": "number", "minimum": 0, "description": "Lower Hybrid Current Drive" }
          }
        },
        "magnetic_field_t": {
          "type": "number",
          "description": "Toroidal magnetic field in Tesla"
        },
        "divertor_heat_mw_m2": {
          "type": "number",
          "description": "Divertor heat flux in MW/m²"
        },
        "fuel_mix": {
          "type": "object",
          "properties": {
            "deuterium": { "type": "number", "minimum": 0, "maximum": 1 },
            "tritium": { "type": "number", "minimum": 0, "maximum": 1 }
          }
        }
      }
    }
  }
}
```

---

## 3. Energy Output Schema

### 3.1 Power Plant Metrics

```json
{
  "energy_output": {
    "type": "object",
    "properties": {
      "gross_fusion_power_mw": {
        "type": "number",
        "description": "Total fusion power generated"
      },
      "thermal_power_mw": {
        "type": "number",
        "description": "Thermal power to heat exchangers"
      },
      "net_electric_power_mw": {
        "type": "number",
        "description": "Net electrical output after auxiliaries"
      },
      "plant_efficiency_percent": {
        "type": "number",
        "minimum": 0,
        "maximum": 100
      },
      "availability_factor": {
        "type": "number",
        "minimum": 0,
        "maximum": 1
      },
      "tritium_breeding_ratio": {
        "type": "number",
        "description": "TBR ≥ 1.05 for self-sufficiency"
      },
      "neutron_wall_loading": {
        "type": "object",
        "properties": {
          "value": { "type": "number" },
          "unit": { "type": "string", "const": "MW/m2" }
        }
      }
    }
  }
}
```

---

## 4. Diagnostic Data Format

### 4.1 Standard Diagnostics

| Diagnostic | Data Type | Sampling Rate | Description |
|------------|-----------|---------------|-------------|
| Thomson Scattering | Array[float] | 100 Hz | Electron temperature/density profile |
| Interferometry | float | 10 kHz | Line-integrated density |
| ECE | Array[float] | 1 MHz | Electron temperature profile |
| Bolometry | Array[float] | 10 kHz | Radiated power distribution |
| Magnetics | Array[float] | 100 kHz | Magnetic field measurements |
| Neutron Counter | int | 1 kHz | Fusion reaction rate |
| Soft X-ray | Array[float] | 100 kHz | Core impurity monitoring |

### 4.2 Diagnostic Data Schema

```json
{
  "diagnostic_data": {
    "type": "object",
    "properties": {
      "diagnostic_id": { "type": "string" },
      "timestamp": { "type": "string", "format": "date-time" },
      "sampling_rate_hz": { "type": "number" },
      "data": {
        "type": "array",
        "items": { "type": "number" }
      },
      "units": { "type": "string" },
      "calibration_version": { "type": "string" },
      "quality_flag": {
        "type": "string",
        "enum": ["good", "suspect", "bad", "missing"]
      }
    }
  }
}
```

---

## 5. Data Exchange Protocols

### 5.1 Real-time Streaming

```yaml
protocol: WebSocket
endpoint: wss://fusion.wia.live/v1/stream
format: JSON
compression: gzip
max_latency_ms: 10
authentication: JWT Bearer Token
```

### 5.2 Batch Data Transfer

```yaml
protocol: HTTPS
endpoint: https://fusion.wia.live/v1/shots/{shot_id}
format: HDF5 / NetCDF / JSON
max_size_gb: 100
authentication: API Key + Signature
```

---

## 6. Compliance Requirements

### 6.1 Mandatory Fields

All implementations MUST include:
- `shot_id`
- `timestamp`
- `reactor`
- `core_parameters.temperature_keV`
- `core_parameters.density_m3`

### 6.2 Data Validation

```python
# Example validation
def validate_plasma_state(data: dict) -> bool:
    required = ['shot_id', 'timestamp', 'reactor', 'core_parameters']
    return all(field in data.get('plasma_state', {}) for field in required)
```

---

## 7. References

- ITER Organization Data Standards
- IAEA Nuclear Fusion Data Guidelines
- KSTAR Data Management System
- IMAS (Integrated Modelling & Analysis Suite)

---

**弘益人間 - Benefit All Humanity**

© 2025 WIA - World Certification Industry Association

---

## Annex A — Conformance Tier Matrix

WIA conformance for WIA-FUSION is evaluated across three tiers:

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

- `wia-standards/standards/WIA-FUSION/api/` — TypeScript SDK skeleton
- `wia-standards/standards/WIA-FUSION/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/WIA-FUSION/simulator/` — interactive browser-based simulator for the PHASE protocol

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
