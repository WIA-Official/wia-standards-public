# WIA-PLASTIC-ENZYME Phase 1: Data Format Specification

> **Version:** 1.0.0
> **Status:** Official
> **Last Updated:** 2025-01-01
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

The WIA-PLASTIC-ENZYME Data Format specification defines standardized data structures for representing enzyme profiles, plastic substrates, degradation processes, and quality metrics. This specification enables interoperability between research laboratories, production facilities, regulatory bodies, and supply chain partners.

### 1.1 Purpose

The primary goals of this data format specification are:

- **Interoperability:** Enable seamless data exchange between diverse systems
- **Extensibility:** Support future enzymes, plastics, and assessment methods
- **Traceability:** Maintain complete chain of custody for recycled materials
- **Quality Assurance:** Ensure consistent product quality through standardized metrics

### 1.2 Scope

This specification covers:

- Enzyme profile data structures
- Plastic substrate characterization
- Degradation process recording
- Monomer quality metrics
- Metadata and provenance information

---

## 2. Data Model Architecture

### 2.1 Entity Relationship

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│     Enzyme      │───▶│     Process      │───▶│    Products     │
└─────────────────┘    └──────────────────┘    └─────────────────┘
        │                      │                        │
        │                      │                        │
        ▼                      ▼                        ▼
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│    Kinetics     │    │   Conditions     │    │  QualityMetrics │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

### 2.2 Core Entities

| Entity | Description | Primary Key |
|--------|-------------|-------------|
| Enzyme | Plastic-degrading enzyme profile | enzyme_id |
| Plastic | Substrate material to be degraded | plastic_id |
| Process | Complete degradation operation | process_id |
| Products | Recovered monomers (TPA, EG) | product_id |
| Kinetics | Enzyme kinetic parameters | enzyme_id |
| QualityMetrics | Product quality measurements | batch_id |

---

## 3. JSON Schema Definitions

### 3.1 Enzyme Profile Schema

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://wia.live/schemas/plastic-enzyme/v1.0.0/enzyme-profile.json",
  "title": "WIA Enzyme Profile",
  "description": "Complete enzyme profile following WIA-PLASTIC-ENZYME v1.0",
  "type": "object",
  "required": ["enzyme_id", "name", "classification", "source", "optimal_conditions"],
  "properties": {
    "enzyme_id": {
      "type": "string",
      "format": "uri",
      "description": "Globally unique identifier (URN format)",
      "pattern": "^urn:wia:enzyme:[a-z0-9-]+$"
    },
    "name": {
      "type": "string",
      "description": "Common name of the enzyme",
      "minLength": 1,
      "maxLength": 100
    },
    "classification": {
      "type": "string",
      "enum": ["PETase", "MHETase", "BHETase", "Cutinase", "Lipase", "Esterase"],
      "description": "Enzyme classification category"
    },
    "source": {
      "$ref": "#/$defs/EnzymeSource"
    },
    "target_plastics": {
      "type": "array",
      "items": {
        "type": "string",
        "enum": ["PET", "PBAT", "PLA", "PCL", "PHA", "PBS"]
      },
      "minItems": 1
    },
    "kinetics": {
      "$ref": "#/$defs/EnzymeKinetics"
    },
    "optimal_conditions": {
      "$ref": "#/$defs/OptimalConditions"
    },
    "stability": {
      "$ref": "#/$defs/EnzymeStability"
    },
    "products": {
      "$ref": "#/$defs/EnzymeProducts"
    },
    "sequence": {
      "type": "string",
      "description": "Amino acid sequence (single-letter code)"
    },
    "structure_pdb": {
      "type": "string",
      "description": "PDB ID for structural data"
    },
    "reference": {
      "type": "string",
      "format": "uri",
      "description": "DOI or URL to primary publication"
    },
    "metadata": {
      "$ref": "#/$defs/Metadata"
    }
  },
  "$defs": {
    "EnzymeSource": {
      "type": "object",
      "required": ["organism"],
      "properties": {
        "organism": { "type": "string" },
        "domain": {
          "type": "string",
          "enum": ["bacteria", "archaea", "fungi", "engineered"]
        },
        "environment": {
          "type": "string",
          "enum": ["marine", "soil", "geothermal", "wastewater", "laboratory"]
        },
        "engineered": { "type": "boolean", "default": false },
        "parent_enzyme": { "type": "string" },
        "mutations": {
          "type": "array",
          "items": { "type": "string" }
        }
      }
    },
    "EnzymeKinetics": {
      "type": "object",
      "properties": {
        "km": {
          "type": "object",
          "properties": {
            "value": { "type": "number", "minimum": 0 },
            "unit": { "type": "string", "default": "mM" }
          }
        },
        "kcat": {
          "type": "object",
          "properties": {
            "value": { "type": "number", "minimum": 0 },
            "unit": { "type": "string", "default": "s-1" }
          }
        },
        "kcat_km": {
          "type": "object",
          "properties": {
            "value": { "type": "number", "minimum": 0 },
            "unit": { "type": "string", "default": "M-1s-1" }
          }
        }
      }
    },
    "OptimalConditions": {
      "type": "object",
      "properties": {
        "temperature_c": {
          "type": "object",
          "properties": {
            "optimal": { "type": "number" },
            "range": {
              "type": "array",
              "items": { "type": "number" },
              "minItems": 2,
              "maxItems": 2
            }
          }
        },
        "ph": {
          "type": "object",
          "properties": {
            "optimal": { "type": "number" },
            "range": {
              "type": "array",
              "items": { "type": "number" },
              "minItems": 2,
              "maxItems": 2
            }
          }
        },
        "substrate_loading_percent": {
          "type": "number",
          "minimum": 0,
          "maximum": 100
        }
      }
    },
    "EnzymeStability": {
      "type": "object",
      "properties": {
        "half_life_hours": { "type": "number", "minimum": 0 },
        "thermostability_tm_c": { "type": "number" },
        "storage_conditions": { "type": "string" }
      }
    },
    "EnzymeProducts": {
      "type": "object",
      "properties": {
        "primary": {
          "type": "array",
          "items": {
            "type": "string",
            "enum": ["TPA", "MHET", "BHET", "EG", "oligomers"]
          }
        },
        "toxicity_class": {
          "type": "string",
          "enum": ["non-toxic", "low", "moderate", "high"]
        }
      }
    },
    "Metadata": {
      "type": "object",
      "properties": {
        "created_at": { "type": "string", "format": "date-time" },
        "updated_at": { "type": "string", "format": "date-time" },
        "wia_version": { "type": "string" },
        "created_by": { "type": "string" }
      }
    }
  }
}
```

### 3.2 Degradation Process Schema

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://wia.live/schemas/plastic-enzyme/v1.0.0/degradation-process.json",
  "title": "WIA Degradation Process",
  "type": "object",
  "required": ["process_id", "plastic_input", "enzyme_cocktail", "conditions"],
  "properties": {
    "process_id": {
      "type": "string",
      "format": "uri",
      "pattern": "^urn:wia:process:[a-z0-9-]+$"
    },
    "facility_id": {
      "type": "string",
      "format": "uri"
    },
    "timestamp": {
      "type": "string",
      "format": "date-time"
    },
    "plastic_input": {
      "type": "object",
      "required": ["type", "weight_kg"],
      "properties": {
        "type": {
          "type": "string",
          "enum": ["PET", "PBAT", "PLA", "PCL", "PHA"]
        },
        "form": {
          "type": "string",
          "enum": ["flakes", "powder", "film", "fiber", "mixed"]
        },
        "weight_kg": { "type": "number", "minimum": 0 },
        "crystallinity_percent": { "type": "number", "minimum": 0, "maximum": 100 },
        "contamination_level": {
          "type": "string",
          "enum": ["none", "low", "medium", "high"]
        },
        "source": { "type": "string" }
      }
    },
    "enzyme_cocktail": {
      "type": "array",
      "items": {
        "type": "object",
        "required": ["enzyme_id", "concentration_mg_g"],
        "properties": {
          "enzyme_id": { "type": "string" },
          "concentration_mg_g": { "type": "number", "minimum": 0 }
        }
      },
      "minItems": 1
    },
    "conditions": {
      "type": "object",
      "properties": {
        "temperature_c": { "type": "number" },
        "ph": { "type": "number" },
        "duration_hours": { "type": "number", "minimum": 0 },
        "agitation_rpm": { "type": "number", "minimum": 0 },
        "volume_liters": { "type": "number", "minimum": 0 }
      }
    },
    "output": {
      "type": "object",
      "properties": {
        "degradation_percent": { "type": "number", "minimum": 0, "maximum": 100 },
        "tpa_yield_kg": { "type": "number", "minimum": 0 },
        "tpa_purity_percent": { "type": "number", "minimum": 0, "maximum": 100 },
        "eg_yield_kg": { "type": "number", "minimum": 0 },
        "eg_purity_percent": { "type": "number", "minimum": 0, "maximum": 100 },
        "residue_kg": { "type": "number", "minimum": 0 }
      }
    },
    "quality_grade": {
      "type": "string",
      "enum": ["food-contact", "bottle-grade", "textile-grade", "industrial"]
    },
    "certification_status": {
      "type": "string",
      "enum": ["pending", "WIA-certified", "rejected"]
    }
  }
}
```

---

## 4. Plastic Type Codes

| Code | Full Name | Description | Primary Enzymes |
|------|-----------|-------------|-----------------|
| PET | Polyethylene Terephthalate | Bottles, containers, textiles | PETase, MHETase |
| PBAT | Polybutylene Adipate Terephthalate | Biodegradable packaging | PETase, Cutinase |
| PLA | Polylactic Acid | Bioplastic from renewable sources | Proteinase K |
| PCL | Polycaprolactone | Medical and packaging | Lipase |
| PHA | Polyhydroxyalkanoates | Bacterial bioplastics | PHA depolymerase |
| PBS | Polybutylene Succinate | Biodegradable films | Cutinase |

---

## 5. Data Exchange Formats

### 5.1 Supported Formats

| Format | Use Case | Content-Type |
|--------|----------|--------------|
| JSON | API exchanges, storage | application/json |
| JSON-LD | Linked data, semantic web | application/ld+json |
| CSV | Bulk import/export | text/csv |
| Protocol Buffers | High-performance binary | application/protobuf |

### 5.2 JSON-LD Context

```json
{
  "@context": {
    "@version": 1.1,
    "wia": "https://wia.live/schemas/plastic-enzyme/v1.0.0/",
    "enzyme_id": "@id",
    "name": "wia:name",
    "classification": "wia:classification",
    "kinetics": "wia:kinetics",
    "km": "wia:km",
    "kcat": "wia:kcat"
  }
}
```

---

## 6. Validation Rules

### 6.1 Enzyme Profile Validation

- `enzyme_id` must be unique and in URN format
- `kinetics.km` and `kinetics.kcat` must be positive numbers
- `optimal_conditions.temperature_c.range[0]` must be less than `range[1]`
- `optimal_conditions.ph.range` must be within 0-14

### 6.2 Process Validation

- `output.degradation_percent` must be 0-100
- Total output yields cannot exceed input mass (within 5% tolerance)
- `conditions.temperature_c` should be within enzyme's optimal range
- `conditions.ph` should be within enzyme's optimal range

---

## 7. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01-01 | Initial release |

---

**弘益人間 (Benefit All Humanity)**

© 2025 WIA - World Certification Industry Association

---

## Annex A — Conformance Tier Matrix

WIA conformance for WIA-PLASTIC-ENZYME is evaluated across three tiers:

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

- `wia-standards/standards/WIA-PLASTIC-ENZYME/api/` — TypeScript SDK skeleton
- `wia-standards/standards/WIA-PLASTIC-ENZYME/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/WIA-PLASTIC-ENZYME/simulator/` — interactive browser-based simulator for the PHASE protocol

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

