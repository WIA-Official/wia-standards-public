# WIA Green Infrastructure Data Format Standard
## Phase 1 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #EF4444 (Red - ENE Category)

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

The WIA Green Infrastructure Data Format Standard defines a unified format for tracking, monitoring, and managing green infrastructure systems including green roofs, permeable surfaces, urban vegetation, and stormwater management facilities to support sustainable urban development.

**Core Objectives**:
- Enable precise tracking of green infrastructure installations and performance
- Integrate IoT sensors for real-time monitoring of environmental metrics
- Support stormwater management and climate resilience planning
- Facilitate collaboration between cities, developers, and environmental agencies

### 1.2 Scope

This standard covers:

| Domain | Description |
|--------|-------------|
| Infrastructure Types | Green roofs, permeable pavement, bioswales, rain gardens, tree canopy |
| Monitoring Data | Soil moisture, vegetation health, stormwater retention, temperature |
| IoT Integration | Sensor networks, automated monitoring, predictive maintenance |
| Environmental Impact | Carbon sequestration, cooling effect, water quality, biodiversity |
| Compliance | Building codes, sustainability certifications, performance metrics |

### 1.3 Design Principles

1. **Sustainability**: Track environmental benefits and climate resilience
2. **Real-time**: IoT sensor integration for continuous monitoring
3. **Transparency**: Open data for public benefit and accountability
4. **Interoperability**: Compatible with urban planning and environmental systems
5. **Scalability**: From individual buildings to city-wide networks

---

## Terminology

### 2.1 Core Terms

| Term | Definition |
|------|------------|
| **Green Infrastructure** | Natural and engineered systems that manage stormwater and provide environmental benefits |
| **Green Roof** | Vegetated roof system for stormwater management and insulation |
| **Permeable Surface** | Porous pavement allowing water infiltration |
| **Bioswale** | Landscape element designed to remove silt and pollution from surface runoff |
| **Rain Garden** | Planted depression that absorbs rainwater runoff |
| **Urban Tree Canopy** | Layer of leaves, branches, and stems that cover the ground when viewed from above |

### 2.2 Data Types

| Type | Description | Example |
|------|-------------|---------|
| `string` | UTF-8 encoded text | `"GI-2025-001"` |
| `gps_coordinate` | Decimal degrees | `{"lat": 37.5665, "lon": 126.9780}` |
| `area` | Square meters | `{"value": 500, "unit": "m2"}` |
| `percentage` | 0-100 value | `{"value": 85, "unit": "%"}` |
| `timestamp` | ISO 8601 datetime | `"2025-01-15T10:30:00Z"` |
| `sensor_reading` | IoT sensor data | `{"type": "soil_moisture", "value": 65}` |

### 2.3 Field Requirements

| Marker | Meaning |
|--------|---------|
| **REQUIRED** | Must be present |
| **OPTIONAL** | May be omitted |
| **CONDITIONAL** | Required under specific conditions |

---

## Base Structure

### 3.1 Green Infrastructure Record Format

```json
{
  "$schema": "https://wia.live/green-infrastructure/v1/schema.json",
  "version": "1.0.0",
  "infrastructureId": "GI-2025-000001",
  "status": "active",
  "created": "2025-01-15T10:30:00Z",
  "lastUpdated": "2025-01-15T10:30:00Z",
  "infrastructure": {
    "type": "green_roof",
    "subtype": "extensive",
    "location": {
      "gps": {
        "latitude": 37.5665,
        "longitude": 126.9780,
        "altitude": 45.0,
        "accuracy": 3.0
      },
      "address": "123 Green Street, Seoul, Korea",
      "buildingId": "BLDG-2025-001",
      "district": "Gangnam-gu"
    },
    "dimensions": {
      "area": {
        "value": 500,
        "unit": "m2"
      },
      "depth": {
        "value": 150,
        "unit": "mm"
      },
      "slope": {
        "value": 2,
        "unit": "degrees"
      }
    },
    "vegetation": {
      "coverage": 85,
      "types": ["sedum", "grasses", "wildflowers"],
      "density": "high",
      "biodiversity_index": 0.75
    }
  },
  "installation": {
    "installDate": "2024-06-15T00:00:00Z",
    "contractor": "Green Solutions Inc.",
    "certification": "LEED-Gold",
    "warranty": {
      "years": 10,
      "expires": "2034-06-15T00:00:00Z"
    }
  },
  "monitoring": {
    "sensors": [],
    "lastReading": {},
    "alerts": []
  },
  "performance": {
    "stormwater": {},
    "carbon": {},
    "temperature": {},
    "biodiversity": {}
  },
  "maintenance": {
    "schedule": "quarterly",
    "lastMaintenance": "2025-01-01T00:00:00Z",
    "nextMaintenance": "2025-04-01T00:00:00Z",
    "history": []
  },
  "meta": {
    "hash": "sha256:...",
    "signature": "...",
    "previousHash": "..."
  }
}
```

### 3.2 Field Details

#### 3.2.1 `infrastructureId` (REQUIRED)

```
Type: string
Format: GI-YYYY-NNNNNN
Description: Unique identifier for this green infrastructure installation
Example: "GI-2025-000001"
```

#### 3.2.2 `status` (REQUIRED)

```
Type: string
Valid values:
  - "planned"      : Design phase, not yet installed
  - "active"       : Operating and monitored
  - "maintenance"  : Undergoing maintenance
  - "degraded"     : Performance below standards
  - "decommissioned" : No longer in service
```

---

## Data Schema

### 4.1 Infrastructure Types Schema

```json
{
  "infrastructure": {
    "type": {
      "enum": [
        "green_roof",
        "permeable_pavement",
        "bioswale",
        "rain_garden",
        "tree_canopy",
        "green_wall",
        "detention_basin",
        "constructed_wetland"
      ]
    },
    "subtype": {
      "green_roof": ["extensive", "intensive", "semi-intensive"],
      "permeable_pavement": ["pervious_concrete", "porous_asphalt", "permeable_pavers"],
      "bioswale": ["dry", "wet", "combination"],
      "rain_garden": ["residential", "commercial", "public"]
    }
  }
}
```

### 4.2 Monitoring Sensors Schema

```json
{
  "monitoring": {
    "sensors": [
      {
        "sensorId": "SENSOR-GI-001",
        "type": "soil_moisture",
        "location": "zone_a",
        "data": {
          "moisture": 65,
          "temperature": 22.5,
          "unit": "celsius"
        },
        "timestamp": "2025-01-15T10:30:00Z",
        "status": "active",
        "battery": 85
      },
      {
        "sensorId": "SENSOR-GI-002",
        "type": "flow_meter",
        "location": "drainage_outlet",
        "data": {
          "flowRate": 15.5,
          "totalVolume": 1250,
          "unit": "liters"
        },
        "timestamp": "2025-01-15T10:30:00Z",
        "status": "active"
      },
      {
        "sensorId": "SENSOR-GI-003",
        "type": "temperature",
        "location": "surface",
        "data": {
          "surfaceTemp": 28.5,
          "ambientTemp": 32.0,
          "coolingEffect": 3.5,
          "unit": "celsius"
        },
        "timestamp": "2025-01-15T10:30:00Z",
        "status": "active"
      },
      {
        "sensorId": "SENSOR-GI-004",
        "type": "vegetation_health",
        "location": "zone_b",
        "data": {
          "ndvi": 0.75,
          "leafAreaIndex": 3.2,
          "healthScore": "good"
        },
        "timestamp": "2025-01-15T10:30:00Z",
        "status": "active"
      }
    ],
    "lastReading": {
      "timestamp": "2025-01-15T10:30:00Z",
      "dataQuality": "excellent",
      "sensorsCommunicating": 4,
      "sensorsTotal": 4
    },
    "alerts": [
      {
        "alertId": "ALERT-001",
        "severity": "warning",
        "type": "low_moisture",
        "message": "Soil moisture below threshold in zone_a",
        "timestamp": "2025-01-15T09:00:00Z",
        "resolved": false
      }
    ]
  }
}
```

### 4.3 Performance Metrics Schema

```json
{
  "performance": {
    "stormwater": {
      "retentionCapacity": {
        "value": 75,
        "unit": "m3"
      },
      "retentionRate": 0.65,
      "annualRetention": {
        "value": 450,
        "unit": "m3_per_year"
      },
      "peakFlowReduction": 0.55,
      "waterQuality": {
        "tssRemoval": 0.80,
        "nitrogenRemoval": 0.45,
        "phosphorusRemoval": 0.50
      }
    },
    "carbon": {
      "sequestrationRate": {
        "value": 510,
        "unit": "kg_co2_per_year"
      },
      "totalSequestered": {
        "value": 340,
        "unit": "kg_co2"
      },
      "equivalentTrees": 15
    },
    "temperature": {
      "coolingEffect": {
        "value": 3.5,
        "unit": "celsius"
      },
      "energySavings": {
        "value": 1250,
        "unit": "kwh_per_year"
      },
      "heatIslandMitigation": 0.25
    },
    "biodiversity": {
      "speciesCount": 35,
      "pollinatorSupport": "high",
      "habitatValue": 0.75,
      "nativeSpeciesRatio": 0.80
    }
  }
}
```

### 4.4 Maintenance History Schema

```json
{
  "maintenance": {
    "schedule": "quarterly",
    "lastMaintenance": "2025-01-01T00:00:00Z",
    "nextMaintenance": "2025-04-01T00:00:00Z",
    "history": [
      {
        "maintenanceId": "MAINT-001",
        "date": "2025-01-01T00:00:00Z",
        "type": "routine",
        "activities": [
          "vegetation_inspection",
          "drainage_check",
          "weeding",
          "irrigation_test"
        ],
        "findings": [
          "All systems functioning normally",
          "Minor weed removal required"
        ],
        "cost": {
          "value": 250,
          "unit": "USD"
        },
        "technician": "John Smith",
        "nextRecommendedDate": "2025-04-01T00:00:00Z"
      }
    ],
    "annualCost": {
      "value": 1000,
      "unit": "USD"
    }
  }
}
```

---

## Field Specifications

### 5.1 Infrastructure Fields

| Field | Type | Required | Description | Example |
|-------|------|----------|-------------|---------|
| `infrastructure.type` | string | REQUIRED | Type of green infrastructure | `"green_roof"` |
| `infrastructure.dimensions.area` | object | REQUIRED | Coverage area in m² | `{"value": 500, "unit": "m2"}` |
| `infrastructure.vegetation.coverage` | number | REQUIRED | Vegetation coverage (0-100%) | `85` |
| `infrastructure.location.gps` | object | REQUIRED | GPS coordinates | `{"latitude": 37.5665, "longitude": 126.9780}` |

### 5.2 Sensor Types

| Type | Measurements | Update Frequency | Purpose |
|------|--------------|------------------|---------|
| `soil_moisture` | Moisture %, temperature | 15-60 minutes | Irrigation management |
| `flow_meter` | Flow rate, volume | Real-time | Stormwater tracking |
| `temperature` | Surface, ambient temp | 5-30 minutes | Heat island monitoring |
| `vegetation_health` | NDVI, LAI, health score | Daily | Plant health assessment |
| `water_quality` | pH, turbidity, nutrients | Hourly | Runoff quality monitoring |

### 5.3 Performance Metrics

| Metric | Unit | Calculation | Purpose |
|--------|------|-------------|---------|
| Stormwater Retention | m³/year | Area × rainfall × retention rate | Flood mitigation |
| Carbon Sequestration | kg CO₂/year | Area × vegetation × carbon rate | Climate benefit |
| Cooling Effect | °C | Surface temp - ambient temp | Heat island reduction |
| Energy Savings | kWh/year | Cooling effect × area × factor | Cost benefit |

---

## Data Types

### 6.1 Custom Types

```typescript
type InfrastructureStatus =
  | 'planned'
  | 'active'
  | 'maintenance'
  | 'degraded'
  | 'decommissioned';

type InfrastructureType =
  | 'green_roof'
  | 'permeable_pavement'
  | 'bioswale'
  | 'rain_garden'
  | 'tree_canopy'
  | 'green_wall'
  | 'detention_basin'
  | 'constructed_wetland';

type SensorType =
  | 'soil_moisture'
  | 'flow_meter'
  | 'temperature'
  | 'vegetation_health'
  | 'water_quality';

type VegetationDensity =
  | 'low'
  | 'medium'
  | 'high'
  | 'very_high';

interface GPSCoordinate {
  latitude: number;    // -90 to 90
  longitude: number;   // -180 to 180
  altitude?: number;   // meters
  accuracy?: number;   // meters
}

interface Measurement {
  value: number;
  unit: string;
}
```

---

## Validation Rules

### 7.1 Required Field Validation

| Rule ID | Field | Validation |
|---------|-------|------------|
| VAL-001 | `infrastructureId` | Must match `^GI-\d{4}-\d{6}$` |
| VAL-002 | `infrastructure.location.gps` | Latitude: -90 to 90, Longitude: -180 to 180 |
| VAL-003 | `infrastructure.vegetation.coverage` | Must be 0-100 |
| VAL-004 | `status` | Must be valid enum value |
| VAL-005 | `infrastructure.dimensions.area.value` | Must be > 0 |

### 7.2 Business Logic Validation

| Rule ID | Description | Error Code |
|---------|-------------|------------|
| BUS-001 | GPS coordinates must be valid location | `ERR_INVALID_LOCATION` |
| BUS-002 | Installation date cannot be in future | `ERR_FUTURE_DATE` |
| BUS-003 | Sensor readings must be within valid ranges | `ERR_INVALID_READING` |
| BUS-004 | Vegetation coverage cannot exceed 100% | `ERR_INVALID_COVERAGE` |
| BUS-005 | Maintenance schedule must be valid | `ERR_INVALID_SCHEDULE` |

---

## Examples

### 8.1 Valid Green Roof Installation

```json
{
  "$schema": "https://wia.live/green-infrastructure/v1/schema.json",
  "version": "1.0.0",
  "infrastructureId": "GI-2025-000001",
  "status": "active",
  "created": "2024-06-15T10:00:00Z",
  "lastUpdated": "2025-01-15T10:30:00Z",
  "infrastructure": {
    "type": "green_roof",
    "subtype": "extensive",
    "location": {
      "gps": {
        "latitude": 37.5665,
        "longitude": 126.9780,
        "altitude": 45.0,
        "accuracy": 3.0
      },
      "address": "123 Green Street, Seoul, Korea",
      "buildingId": "BLDG-2025-001",
      "district": "Gangnam-gu"
    },
    "dimensions": {
      "area": {"value": 500, "unit": "m2"},
      "depth": {"value": 150, "unit": "mm"},
      "slope": {"value": 2, "unit": "degrees"}
    },
    "vegetation": {
      "coverage": 85,
      "types": ["sedum", "grasses", "wildflowers"],
      "density": "high",
      "biodiversity_index": 0.75
    }
  },
  "installation": {
    "installDate": "2024-06-15T00:00:00Z",
    "contractor": "Green Solutions Inc.",
    "certification": "LEED-Gold",
    "warranty": {
      "years": 10,
      "expires": "2034-06-15T00:00:00Z"
    }
  },
  "performance": {
    "stormwater": {
      "retentionRate": 0.65,
      "annualRetention": {"value": 450, "unit": "m3_per_year"}
    },
    "carbon": {
      "sequestrationRate": {"value": 510, "unit": "kg_co2_per_year"}
    },
    "temperature": {
      "coolingEffect": {"value": 3.5, "unit": "celsius"}
    }
  }
}
```

### 8.2 Valid Permeable Pavement

```json
{
  "$schema": "https://wia.live/green-infrastructure/v1/schema.json",
  "version": "1.0.0",
  "infrastructureId": "GI-2025-000042",
  "status": "active",
  "created": "2024-08-01T10:00:00Z",
  "infrastructure": {
    "type": "permeable_pavement",
    "subtype": "permeable_pavers",
    "location": {
      "gps": {
        "latitude": 37.5165,
        "longitude": 127.0280,
        "altitude": 10.0
      },
      "address": "456 Eco Plaza, Seoul, Korea",
      "district": "Songpa-gu"
    },
    "dimensions": {
      "area": {"value": 1200, "unit": "m2"},
      "depth": {"value": 300, "unit": "mm"}
    }
  },
  "performance": {
    "stormwater": {
      "infiltrationRate": {"value": 85, "unit": "mm_per_hour"},
      "retentionRate": 0.75,
      "annualRetention": {"value": 1080, "unit": "m3_per_year"}
    }
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

**WIA Green Infrastructure Standard v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA**

**MIT License**

</div>


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

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-1-DATA-FORMAT. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-1-data-format/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-1-DATA-FORMAT with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-1-DATA-FORMAT does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-1-DATA-FORMAT.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-1-DATA-FORMAT. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P1-DATA-FORMAT-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.
