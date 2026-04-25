# WIA Mangrove Restoration Data Format Standard
## Phase 1 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #EF4444 (Red)

---

## Table of Contents

1. [Overview](#overview)
2. [Terminology](#terminology)
3. [Base Structure](#base-structure)
4. [Data Schema](#data-schema)
5. [Field Specifications](#field-specifications)
6. [Data Types](#data-types)
7. [Validation Rules](#validation-rules)
8. [Examples](#examples)
9. [Version History](#version-history)

---

## Overview

### 1.1 Purpose

The WIA Mangrove Restoration Data Format Standard defines a unified format for mangrove ecosystem monitoring, coastal protection assessment, blue carbon accounting, and marine nursery habitat preservation, enabling global coordination of coastal restoration efforts.

**Core Objectives**:
- Enable precise monitoring of mangrove coverage and species diversity
- Track blue carbon sequestration and coastal protection benefits
- Integrate remote sensing and ground survey data
- Support marine conservation and fisheries management

### 1.2 Scope

This standard covers:

| Domain | Description |
|--------|-------------|
| Ecosystem Monitoring | Mangrove coverage, species composition, health assessment |
| Carbon Accounting | Blue carbon sequestration, biomass estimation, carbon credits |
| Coastal Protection | Wave attenuation, erosion control, storm surge protection |
| Marine Habitat | Nursery habitat quality, biodiversity, fisheries support |
| Water Quality | Salinity, tidal range, nutrient levels, sediment dynamics |

### 1.3 Design Principles

1. **Precision**: Accurate measurement of ecosystem health and carbon storage
2. **Scalability**: Support for site-level to global monitoring networks
3. **Interoperability**: Compatible with blue carbon markets and conservation systems
4. **Sustainability**: Long-term ecosystem restoration tracking
5. **Community**: Integration with local community and fisheries data

---

## Terminology

### 2.1 Core Terms

| Term | Definition |
|------|------------|
| **Mangrove** | Salt-tolerant trees and shrubs growing in intertidal coastal zones |
| **Blue Carbon** | Carbon captured and stored by coastal and marine ecosystems |
| **Tidal Range** | Vertical difference between high tide and low tide levels |
| **Salinity** | Salt concentration in water, measured in parts per thousand (ppt) |
| **Restoration Site** | Designated area for mangrove ecosystem restoration activities |
| **Wave Attenuation** | Reduction in wave energy provided by mangrove forests |

### 2.2 Data Types

| Type | Description | Example |
|------|-------------|---------|
| `string` | UTF-8 encoded text | `"MANG-2025-001"` |
| `area` | Hectares with precision | `{"value": 25.5, "unit": "hectares"}` |
| `salinity` | Parts per thousand | `{"value": 15.5, "unit": "ppt"}` |
| `depth` | Meters with precision | `{"value": 2.5, "unit": "m"}` |
| `timestamp` | ISO 8601 datetime | `"2025-01-15T10:30:00Z"` |
| `gps_coordinate` | Decimal degrees | `{"lat": 10.4806, "lon": 123.3050}` |

### 2.3 Field Requirements

| Marker | Meaning |
|--------|---------|
| **REQUIRED** | Must be present |
| **OPTIONAL** | May be omitted |
| **CONDITIONAL** | Required under specific conditions |

---

## Base Structure

### 3.1 Mangrove Restoration Record Format

```json
{
  "$schema": "https://wia.live/mangrove-restoration/v1/schema.json",
  "version": "1.0.0",
  "siteId": "MANG-2025-000001",
  "status": "active",
  "created": "2025-01-15T10:30:00Z",
  "lastUpdated": "2025-01-15T10:30:00Z",
  "location": {
    "gps": {
      "latitude": 10.4806,
      "longitude": 123.3050,
      "elevation": 2.5,
      "accuracy": 3.0
    },
    "region": "Central Visayas",
    "ecosystemType": "fringe",
    "country": "Philippines"
  },
  "coverage": {
    "total": {"value": 25.5, "unit": "hectares"},
    "restored": {"value": 18.2, "unit": "hectares"},
    "natural": {"value": 7.3, "unit": "hectares"},
    "degraded": {"value": 0, "unit": "hectares"}
  },
  "species": [
    {
      "scientificName": "Rhizophora apiculata",
      "commonName": "Red Mangrove",
      "coverage": {"value": 12.5, "unit": "hectares"},
      "density": {"value": 1200, "unit": "trees_per_hectare"},
      "avgHeight": {"value": 8.5, "unit": "m"},
      "avgAge": {"value": 5, "unit": "years"}
    },
    {
      "scientificName": "Avicennia marina",
      "commonName": "Grey Mangrove",
      "coverage": {"value": 8.0, "unit": "hectares"},
      "density": {"value": 1500, "unit": "trees_per_hectare"},
      "avgHeight": {"value": 6.2, "unit": "m"},
      "avgAge": {"value": 4, "unit": "years"}
    }
  ],
  "waterQuality": {
    "salinity": {"value": 15.5, "unit": "ppt"},
    "temperature": {"value": 28.5, "unit": "celsius"},
    "tidalRange": {"value": 2.5, "unit": "m"},
    "pH": 8.1,
    "dissolvedOxygen": {"value": 6.5, "unit": "mg_l"}
  },
  "carbon": {
    "abovegroundBiomass": {"value": 125.5, "unit": "tons_c_ha"},
    "belowgroundBiomass": {"value": 85.2, "unit": "tons_c_ha"},
    "soilCarbon": {"value": 450.0, "unit": "tons_c_ha"},
    "totalCarbon": {"value": 660.7, "unit": "tons_c_ha"},
    "sequestrationRate": {"value": 12.5, "unit": "tons_co2_ha_year"}
  },
  "coastal": {
    "shorelineProtected": {"value": 2.5, "unit": "km"},
    "waveAttenuation": {"value": 70, "unit": "percent"},
    "erosionReduction": {"value": 85, "unit": "percent"},
    "stormSurgeProtection": "high"
  },
  "biodiversity": {
    "marineSpeciesCount": 45,
    "birdSpeciesCount": 12,
    "fishNurseryScore": 88,
    "habitatQuality": "excellent"
  },
  "meta": {
    "hash": "sha256:...",
    "signature": "...",
    "previousHash": "..."
  }
}
```

### 3.2 Field Details

#### 3.2.1 `siteId` (REQUIRED)

```
Type: string
Format: MANG-YYYY-NNNNNN
Description: Unique identifier for restoration site
Example: "MANG-2025-000001"
```

#### 3.2.2 `status` (REQUIRED)

```
Type: string
Valid values:
  - "planning"     : Site assessment and planning phase
  - "active"       : Active restoration in progress
  - "monitoring"   : Post-restoration monitoring
  - "mature"       : Fully established ecosystem
  - "degraded"     : Ecosystem health declining
```

---

## Data Schema

### 4.1 Complete JSON Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.live/mangrove-restoration/v1/schema.json",
  "title": "WIA Mangrove Restoration Record",
  "type": "object",
  "required": ["version", "siteId", "status", "created", "location", "coverage"],
  "properties": {
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$"
    },
    "siteId": {
      "type": "string",
      "pattern": "^MANG-\\d{4}-\\d{6}$"
    },
    "status": {
      "type": "string",
      "enum": ["planning", "active", "monitoring", "mature", "degraded"]
    },
    "created": {
      "type": "string",
      "format": "date-time"
    },
    "location": {
      "type": "object",
      "required": ["gps", "ecosystemType"],
      "properties": {
        "gps": {
          "type": "object",
          "required": ["latitude", "longitude"],
          "properties": {
            "latitude": {
              "type": "number",
              "minimum": -90,
              "maximum": 90
            },
            "longitude": {
              "type": "number",
              "minimum": -180,
              "maximum": 180
            },
            "elevation": {"type": "number"},
            "accuracy": {"type": "number"}
          }
        },
        "region": {"type": "string"},
        "ecosystemType": {
          "type": "string",
          "enum": ["coastal", "riverine", "basin", "fringe", "dwarf"]
        },
        "country": {"type": "string"}
      }
    },
    "coverage": {
      "type": "object",
      "required": ["total"],
      "properties": {
        "total": {
          "type": "object",
          "required": ["value", "unit"],
          "properties": {
            "value": {"type": "number", "minimum": 0},
            "unit": {"type": "string", "enum": ["hectares", "acres", "km2"]}
          }
        }
      }
    },
    "species": {
      "type": "array",
      "items": {
        "type": "object",
        "required": ["scientificName", "commonName", "coverage"],
        "properties": {
          "scientificName": {"type": "string"},
          "commonName": {"type": "string"},
          "coverage": {
            "type": "object",
            "required": ["value", "unit"],
            "properties": {
              "value": {"type": "number", "minimum": 0},
              "unit": {"type": "string"}
            }
          }
        }
      }
    }
  }
}
```

### 4.2 Species Array Schema

```json
{
  "species": [
    {
      "scientificName": "Rhizophora apiculata",
      "commonName": "Red Mangrove",
      "coverage": {"value": 12.5, "unit": "hectares"},
      "density": {"value": 1200, "unit": "trees_per_hectare"},
      "avgHeight": {"value": 8.5, "unit": "m"},
      "avgAge": {"value": 5, "unit": "years"},
      "healthStatus": "healthy",
      "reproductionRate": "high"
    }
  ]
}
```

### 4.3 Carbon Accounting Schema

```json
{
  "carbon": {
    "abovegroundBiomass": {"value": 125.5, "unit": "tons_c_ha"},
    "belowgroundBiomass": {"value": 85.2, "unit": "tons_c_ha"},
    "soilCarbon": {"value": 450.0, "unit": "tons_c_ha"},
    "totalCarbon": {"value": 660.7, "unit": "tons_c_ha"},
    "sequestrationRate": {"value": 12.5, "unit": "tons_co2_ha_year"},
    "carbonCredits": {
      "certified": 2500,
      "pending": 500,
      "standard": "Verified Carbon Standard",
      "value": {"amount": 50000, "currency": "USD"}
    }
  }
}
```

---

## Field Specifications

### 5.1 Location Fields

| Field | Type | Required | Description | Example |
|-------|------|----------|-------------|---------|
| `location.gps.latitude` | number | REQUIRED | GPS latitude (-90 to 90) | `10.4806` |
| `location.gps.longitude` | number | REQUIRED | GPS longitude (-180 to 180) | `123.3050` |
| `location.gps.elevation` | number | OPTIONAL | Elevation above sea level (m) | `2.5` |
| `location.ecosystemType` | string | REQUIRED | Mangrove ecosystem classification | `"fringe"` |

**Valid ecosystem types:**

| Value | Description | Characteristics |
|-------|-------------|-----------------|
| `coastal` | Coastal mangrove | Direct ocean exposure |
| `riverine` | Riverine mangrove | Along river channels |
| `basin` | Basin mangrove | Interior depressions |
| `fringe` | Fringe mangrove | Shoreline edge |
| `dwarf` | Dwarf mangrove | Stunted growth conditions |

### 5.2 Species Fields

| Field | Type | Unit | Typical Range |
|-------|------|------|---------------|
| `density` | number | trees/ha | 500-2000 |
| `avgHeight` | number | meters | 1-30 |
| `avgAge` | number | years | 1-100 |
| `coverage` | number | hectares | 0.1-1000 |

### 5.3 Water Quality Parameters

| Parameter | Unit | Optimal Range | Description |
|-----------|------|---------------|-------------|
| Salinity | ppt | 5-35 | Salt concentration |
| Temperature | °C | 20-35 | Water temperature |
| pH | - | 7.5-8.5 | Acidity/alkalinity |
| Dissolved O₂ | mg/L | >5.0 | Oxygen availability |
| Tidal Range | m | 0.5-5.0 | Tide variation |

---

## Data Types

### 6.1 Custom Types

```typescript
type RestorationStatus =
  | 'planning'
  | 'active'
  | 'monitoring'
  | 'mature'
  | 'degraded';

type EcosystemType =
  | 'coastal'
  | 'riverine'
  | 'basin'
  | 'fringe'
  | 'dwarf';

type HealthStatus =
  | 'excellent'
  | 'good'
  | 'fair'
  | 'poor'
  | 'critical';

type ProtectionLevel =
  | 'low'
  | 'moderate'
  | 'high'
  | 'very_high';

interface Area {
  value: number;
  unit: 'hectares' | 'acres' | 'km2';
}

interface Salinity {
  value: number;
  unit: 'ppt';
}

interface CarbonStock {
  value: number;
  unit: 'tons_c_ha' | 'tons_co2_ha' | 'kg_c_m2';
}
```

### 6.2 Common Mangrove Species

| Scientific Name | Common Name | Region |
|----------------|-------------|--------|
| *Rhizophora apiculata* | Red Mangrove | Indo-Pacific |
| *Rhizophora mangle* | American Red Mangrove | Americas |
| *Avicennia marina* | Grey Mangrove | Indo-Pacific |
| *Avicennia germinans* | Black Mangrove | Americas |
| *Bruguiera gymnorrhiza* | Large-leafed Mangrove | Indo-Pacific |
| *Sonneratia alba* | Mangrove Apple | Indo-Pacific |

---

## Validation Rules

### 7.1 Required Field Validation

| Rule ID | Field | Validation |
|---------|-------|------------|
| VAL-001 | `siteId` | Must match `^MANG-\d{4}-\d{6}$` |
| VAL-002 | `location.gps` | Latitude: -90 to 90, Longitude: -180 to 180 |
| VAL-003 | `coverage.total` | Must be > 0 |
| VAL-004 | `created` | Cannot be in future |
| VAL-005 | `status` | Must be valid enum value |

### 7.2 Business Logic Validation

| Rule ID | Description | Error Code |
|---------|-------------|------------|
| BUS-001 | Total coverage = sum of species coverage | `ERR_COVERAGE_MISMATCH` |
| BUS-002 | Salinity must be 0-50 ppt | `ERR_INVALID_SALINITY` |
| BUS-003 | Carbon total = aboveground + belowground + soil | `ERR_CARBON_CALC` |
| BUS-004 | Species coverage ≤ total coverage | `ERR_SPECIES_OVERFLOW` |
| BUS-005 | Tidal range must be > 0 | `ERR_INVALID_TIDE` |

### 7.3 Error Codes

| Code | Message | Description |
|------|---------|-------------|
| `ERR_INVALID_SITE` | Invalid site format | Site ID format violation |
| `ERR_INVALID_LOCATION` | Invalid GPS coordinates | Not in coastal region |
| `ERR_SPECIES_UNKNOWN` | Unknown species | Species not in database |
| `ERR_CARBON_NEGATIVE` | Negative carbon value | Invalid carbon measurement |
| `ERR_COVERAGE_INVALID` | Invalid coverage area | Area calculation error |

---

## Examples

### 8.1 Valid Restoration Record - Tropical Fringe

```json
{
  "$schema": "https://wia.live/mangrove-restoration/v1/schema.json",
  "version": "1.0.0",
  "siteId": "MANG-2025-000001",
  "status": "active",
  "created": "2025-01-15T10:30:00Z",
  "lastUpdated": "2025-01-15T10:30:00Z",
  "location": {
    "gps": {
      "latitude": 10.4806,
      "longitude": 123.3050,
      "elevation": 2.5,
      "accuracy": 3.0
    },
    "region": "Central Visayas",
    "ecosystemType": "fringe",
    "country": "Philippines"
  },
  "coverage": {
    "total": {"value": 25.5, "unit": "hectares"},
    "restored": {"value": 18.2, "unit": "hectares"},
    "natural": {"value": 7.3, "unit": "hectares"}
  },
  "species": [
    {
      "scientificName": "Rhizophora apiculata",
      "commonName": "Red Mangrove",
      "coverage": {"value": 12.5, "unit": "hectares"},
      "density": {"value": 1200, "unit": "trees_per_hectare"},
      "avgHeight": {"value": 8.5, "unit": "m"},
      "avgAge": {"value": 5, "unit": "years"}
    }
  ],
  "waterQuality": {
    "salinity": {"value": 15.5, "unit": "ppt"},
    "temperature": {"value": 28.5, "unit": "celsius"},
    "tidalRange": {"value": 2.5, "unit": "m"},
    "pH": 8.1
  },
  "carbon": {
    "totalCarbon": {"value": 660.7, "unit": "tons_c_ha"},
    "sequestrationRate": {"value": 12.5, "unit": "tons_co2_ha_year"}
  },
  "coastal": {
    "shorelineProtected": {"value": 2.5, "unit": "km"},
    "waveAttenuation": {"value": 70, "unit": "percent"}
  }
}
```

### 8.2 Valid Record - Riverine Mangrove

```json
{
  "$schema": "https://wia.live/mangrove-restoration/v1/schema.json",
  "version": "1.0.0",
  "siteId": "MANG-2025-000042",
  "status": "monitoring",
  "created": "2024-01-01T00:00:00Z",
  "lastUpdated": "2025-01-15T14:30:00Z",
  "location": {
    "gps": {
      "latitude": -6.2088,
      "longitude": 106.8456,
      "elevation": 1.5,
      "accuracy": 2.0
    },
    "region": "Java",
    "ecosystemType": "riverine",
    "country": "Indonesia"
  },
  "coverage": {
    "total": {"value": 50.0, "unit": "hectares"},
    "restored": {"value": 35.0, "unit": "hectares"},
    "natural": {"value": 15.0, "unit": "hectares"}
  },
  "species": [
    {
      "scientificName": "Avicennia marina",
      "commonName": "Grey Mangrove",
      "coverage": {"value": 30.0, "unit": "hectares"},
      "density": {"value": 1500, "unit": "trees_per_hectare"}
    },
    {
      "scientificName": "Sonneratia alba",
      "commonName": "Mangrove Apple",
      "coverage": {"value": 20.0, "unit": "hectares"},
      "density": {"value": 1000, "unit": "trees_per_hectare"}
    }
  ],
  "carbon": {
    "totalCarbon": {"value": 720.0, "unit": "tons_c_ha"},
    "carbonCredits": {
      "certified": 3600,
      "value": {"amount": 72000, "currency": "USD"}
    }
  },
  "biodiversity": {
    "marineSpeciesCount": 62,
    "birdSpeciesCount": 18,
    "fishNurseryScore": 92
  }
}
```

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial release |

---

<div align="center">

**WIA Mangrove Restoration Standard v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA**

**MIT License**

</div>

---

## Annex A — Conformance Tier Matrix

WIA conformance for mangrove-restoration is evaluated across three tiers:

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

- `wia-standards/standards/mangrove-restoration/api/` — TypeScript SDK skeleton
- `wia-standards/standards/mangrove-restoration/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/mangrove-restoration/simulator/` — interactive browser-based simulator for the PHASE protocol

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

