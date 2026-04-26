# WIA Crop Monitoring Data Format Standard
## Phase 1 Specification

---

**Version**: 1.0.0
**Status**: Complete
**Date**: 2025-01
**Standard ID**: WIA-AGRI-006
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #84CC16 (Lime - AGRI)

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

The WIA Crop Monitoring Data Format Standard defines a unified format for tracking crop growth, health monitoring, disease detection, pest identification, and yield forecasting. This standard enables real-time observation of agricultural fields through IoT sensors, cameras, and AI-powered analysis.

**Core Objectives**:
- Enable precise tracking of crop growth stages and health metrics
- Standardize phenology data for global agricultural interoperability
- Support AI-based disease and pest detection
- Facilitate yield prediction and harvest planning
- Enable integration with weather, marketplace, and insurance systems

### 1.2 Scope

This standard covers:

| Domain | Description |
|--------|-------------|
| Growth Monitoring | Height, leaf area, chlorophyll, biomass tracking |
| Phenology Stages | Seedling, vegetative, flowering, fruiting, maturity |
| Health Assessment | Disease detection, pest identification, nutrient status |
| Environmental Data | Soil moisture, temperature, humidity, light intensity |
| Yield Forecasting | Predictive models based on growth patterns |

### 1.3 Design Principles

1. **Real-Time**: Sub-hourly data collection and streaming
2. **AI-Ready**: Structured for machine learning and computer vision
3. **Scalability**: Support from small farms to large-scale agriculture
4. **Interoperability**: Compatible with major agricultural standards (FAO, USDA)
5. **Privacy**: Farm data encrypted, aggregated insights public

---

## Terminology

### 2.1 Core Terms

| Term | Definition |
|------|------------|
| **Crop Growth Stage** | Phenological development phase (BBCH scale) |
| **Leaf Area Index (LAI)** | Total leaf surface area per unit ground area |
| **Chlorophyll Content** | SPAD value indicating photosynthetic capacity |
| **NDVI** | Normalized Difference Vegetation Index (remote sensing) |
| **Disease Pressure** | Risk level for crop diseases based on conditions |
| **Pest Infestation** | Severity and type of pest damage |

### 2.2 Data Types

| Type | Description | Example |
|------|-------------|---------|
| `crop_id` | Unique crop/field identifier | `"CROP-2025-001"` |
| `stage_code` | BBCH phenology code | `"BBCH-51"` (flowering) |
| `measurement` | Numeric value with unit | `{"value": 45.5, "unit": "cm"}` |
| `gps_location` | Geographic coordinates | `{"lat": 37.5665, "lon": 126.9780}` |
| `image_data` | Base64 or URL to crop image | `"https://..."` |

### 2.3 Field Requirements

| Marker | Meaning |
|--------|---------|
| **REQUIRED** | Must be present |
| **OPTIONAL** | May be omitted |
| **CONDITIONAL** | Required under specific conditions |

---

## Base Structure

### 3.1 Crop Monitoring Record Format

All crop monitoring records follow this base structure:

```json
{
  "cropId": "string (REQUIRED)",
  "farmId": "string (REQUIRED)",
  "timestamp": "ISO8601 (REQUIRED)",
  "location": {
    "gps": "object (REQUIRED)",
    "fieldId": "string (OPTIONAL)"
  },
  "cropType": "string (REQUIRED)",
  "variety": "string (OPTIONAL)",
  "growthStage": "object (REQUIRED)",
  "measurements": "object (REQUIRED)",
  "healthStatus": "object (OPTIONAL)",
  "predictions": "object (OPTIONAL)",
  "metadata": "object (OPTIONAL)"
}
```

### 3.2 Growth Stage Object

```json
{
  "growthStage": {
    "code": "BBCH-XX (REQUIRED)",
    "description": "string (REQUIRED)",
    "daysSinceSeeding": "integer (OPTIONAL)",
    "estimatedDaysToHarvest": "integer (OPTIONAL)"
  }
}
```

### 3.3 Measurements Object

```json
{
  "measurements": {
    "plantHeight": {"value": "number", "unit": "cm"},
    "leafAreaIndex": {"value": "number", "unit": "m²/m²"},
    "chlorophyllSPAD": {"value": "number", "unit": "SPAD"},
    "ndvi": {"value": "number", "range": "-1 to 1"},
    "biomass": {"value": "number", "unit": "kg/ha"},
    "soilMoisture": {"value": "number", "unit": "%"},
    "temperature": {"value": "number", "unit": "°C"},
    "humidity": {"value": "number", "unit": "%"}
  }
}
```

---

## Data Schema

### 4.1 Complete Crop Monitoring Schema

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "title": "WIA Crop Monitoring Data v1.0",
  "type": "object",
  "required": ["cropId", "farmId", "timestamp", "location", "cropType", "growthStage", "measurements"],
  "properties": {
    "cropId": {
      "type": "string",
      "pattern": "^CROP-[0-9]{4}-[A-Z0-9]{3,10}$",
      "description": "Unique crop batch identifier"
    },
    "farmId": {
      "type": "string",
      "description": "Farm or field owner identifier"
    },
    "timestamp": {
      "type": "string",
      "format": "date-time",
      "description": "Measurement timestamp (ISO 8601)"
    },
    "location": {
      "type": "object",
      "required": ["gps"],
      "properties": {
        "gps": {
          "type": "object",
          "required": ["latitude", "longitude"],
          "properties": {
            "latitude": {"type": "number", "minimum": -90, "maximum": 90},
            "longitude": {"type": "number", "minimum": -180, "maximum": 180}
          }
        },
        "fieldId": {"type": "string"},
        "areaHectares": {"type": "number", "minimum": 0}
      }
    },
    "cropType": {
      "type": "string",
      "enum": ["rice", "corn", "wheat", "soybean", "tomato", "pepper", "cabbage", "potato", "other"]
    },
    "variety": {
      "type": "string",
      "description": "Specific crop variety or cultivar"
    },
    "growthStage": {
      "type": "object",
      "required": ["code", "description"],
      "properties": {
        "code": {
          "type": "string",
          "pattern": "^BBCH-[0-9]{2}$",
          "description": "BBCH phenology code (00-99)"
        },
        "description": {"type": "string"},
        "daysSinceSeeding": {"type": "integer", "minimum": 0},
        "estimatedDaysToHarvest": {"type": "integer", "minimum": 0}
      }
    },
    "measurements": {
      "type": "object",
      "properties": {
        "plantHeight": {
          "type": "object",
          "properties": {
            "value": {"type": "number", "minimum": 0},
            "unit": {"type": "string", "const": "cm"}
          }
        },
        "leafAreaIndex": {
          "type": "object",
          "properties": {
            "value": {"type": "number", "minimum": 0, "maximum": 10},
            "unit": {"type": "string", "const": "m²/m²"}
          }
        },
        "chlorophyllSPAD": {
          "type": "object",
          "properties": {
            "value": {"type": "number", "minimum": 0, "maximum": 60},
            "unit": {"type": "string", "const": "SPAD"}
          }
        },
        "ndvi": {
          "type": "object",
          "properties": {
            "value": {"type": "number", "minimum": -1, "maximum": 1}
          }
        }
      }
    },
    "healthStatus": {
      "type": "object",
      "properties": {
        "overall": {"type": "string", "enum": ["healthy", "stressed", "diseased", "critical"]},
        "diseases": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "name": {"type": "string"},
              "severity": {"type": "string", "enum": ["low", "medium", "high"]},
              "confidence": {"type": "number", "minimum": 0, "maximum": 1}
            }
          }
        },
        "pests": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "species": {"type": "string"},
              "severity": {"type": "string", "enum": ["low", "medium", "high"]},
              "confidence": {"type": "number", "minimum": 0, "maximum": 1}
            }
          }
        }
      }
    },
    "predictions": {
      "type": "object",
      "properties": {
        "yieldEstimate": {
          "type": "object",
          "properties": {
            "value": {"type": "number"},
            "unit": {"type": "string", "const": "kg/ha"},
            "confidence": {"type": "number", "minimum": 0, "maximum": 1}
          }
        },
        "harvestDate": {"type": "string", "format": "date"},
        "diseaseRisk": {"type": "number", "minimum": 0, "maximum": 100}
      }
    }
  }
}
```

---

## Field Specifications

### 5.1 Crop Identification Fields

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `cropId` | string | REQUIRED | Unique identifier (e.g., "CROP-2025-001") |
| `farmId` | string | REQUIRED | Farm owner or organization ID |
| `cropType` | string | REQUIRED | Crop species (rice, corn, etc.) |
| `variety` | string | OPTIONAL | Specific cultivar or variety |

### 5.2 Growth Stage Fields (BBCH Scale)

| Code | Stage | Description |
|------|-------|-------------|
| BBCH-00 | Dry seed | Before germination |
| BBCH-10 | Seedling | First leaves emerging |
| BBCH-30 | Stem elongation | Vegetative growth |
| BBCH-50 | Flowering | Inflorescence emergence |
| BBCH-70 | Fruit development | Fruit set and growth |
| BBCH-90 | Maturity | Ready for harvest |

### 5.3 Measurement Fields

| Field | Unit | Range | Description |
|-------|------|-------|-------------|
| `plantHeight` | cm | 0-500 | Average plant height |
| `leafAreaIndex` | m²/m² | 0-10 | Leaf surface area ratio |
| `chlorophyllSPAD` | SPAD | 0-60 | Chlorophyll content |
| `ndvi` | - | -1 to 1 | Vegetation index |
| `soilMoisture` | % | 0-100 | Soil water content |
| `temperature` | °C | -20 to 50 | Air temperature |
| `humidity` | % | 0-100 | Relative humidity |

---

## Validation Rules

### 6.1 Data Integrity Rules

1. **Timestamp Validation**
   - Must be valid ISO 8601 format
   - Cannot be in the future (> current time + 5 minutes)
   - Must be within planting season for crop type

2. **GPS Validation**
   - Latitude: -90 to 90
   - Longitude: -180 to 180
   - Must be on land (not ocean)

3. **Growth Stage Consistency**
   - BBCH code must match crop type
   - Days since seeding must align with stage
   - Cannot regress to earlier stage

4. **Measurement Ranges**
   - All numeric values must be within specified ranges
   - Units must match schema
   - Negative values not allowed (except NDVI)

### 6.2 Cross-Field Validation

```python
def validate_crop_data(data):
    # LAI and NDVI correlation
    if data['measurements']['lai']['value'] > 5:
        assert data['measurements']['ndvi']['value'] > 0.6

    # Chlorophyll and health status
    if data['measurements']['chlorophyllSPAD']['value'] < 30:
        assert data['healthStatus']['overall'] in ['stressed', 'diseased']

    # Harvest readiness
    if data['growthStage']['code'] == 'BBCH-90':
        assert data['predictions']['harvestDate'] is not None
```

---

## Examples

### 7.1 Healthy Rice Crop (Vegetative Stage)

```json
{
  "cropId": "CROP-2025-RICE-001",
  "farmId": "FARM-KR-12345",
  "timestamp": "2025-06-15T10:30:00Z",
  "location": {
    "gps": {
      "latitude": 37.5665,
      "longitude": 126.9780
    },
    "fieldId": "FIELD-A-01",
    "areaHectares": 2.5
  },
  "cropType": "rice",
  "variety": "Koshihikari",
  "growthStage": {
    "code": "BBCH-30",
    "description": "Vegetative stage - stem elongation",
    "daysSinceSeeding": 45,
    "estimatedDaysToHarvest": 75
  },
  "measurements": {
    "plantHeight": {"value": 45.5, "unit": "cm"},
    "leafAreaIndex": {"value": 3.8, "unit": "m²/m²"},
    "chlorophyllSPAD": {"value": 42.0, "unit": "SPAD"},
    "ndvi": {"value": 0.75},
    "soilMoisture": {"value": 85, "unit": "%"},
    "temperature": {"value": 25.5, "unit": "°C"},
    "humidity": {"value": 70, "unit": "%"}
  },
  "healthStatus": {
    "overall": "healthy",
    "diseases": [],
    "pests": []
  },
  "predictions": {
    "yieldEstimate": {
      "value": 5500,
      "unit": "kg/ha",
      "confidence": 0.82
    },
    "harvestDate": "2025-09-01",
    "diseaseRisk": 15
  }
}
```

### 7.2 Tomato with Disease Detection

```json
{
  "cropId": "CROP-2025-TOMA-102",
  "farmId": "FARM-US-98765",
  "timestamp": "2025-07-20T14:45:00Z",
  "location": {
    "gps": {"latitude": 40.7128, "longitude": -74.0060},
    "fieldId": "GREENHOUSE-3"
  },
  "cropType": "tomato",
  "variety": "Beefsteak",
  "growthStage": {
    "code": "BBCH-70",
    "description": "Fruit development",
    "daysSinceSeeding": 80,
    "estimatedDaysToHarvest": 25
  },
  "measurements": {
    "plantHeight": {"value": 120, "unit": "cm"},
    "leafAreaIndex": {"value": 4.2, "unit": "m²/m²"},
    "chlorophyllSPAD": {"value": 28.5, "unit": "SPAD"},
    "temperature": {"value": 28.0, "unit": "°C"},
    "humidity": {"value": 85, "unit": "%"}
  },
  "healthStatus": {
    "overall": "diseased",
    "diseases": [
      {
        "name": "Late Blight (Phytophthora infestans)",
        "severity": "medium",
        "confidence": 0.87
      }
    ],
    "pests": []
  },
  "predictions": {
    "yieldEstimate": {
      "value": 45000,
      "unit": "kg/ha",
      "confidence": 0.65
    },
    "diseaseRisk": 75
  }
}
```

### 7.3 Corn with Pest Infestation

```json
{
  "cropId": "CROP-2025-CORN-505",
  "farmId": "FARM-BR-55443",
  "timestamp": "2025-08-10T09:15:00Z",
  "location": {
    "gps": {"latitude": -15.7801, "longitude": -47.9292},
    "areaHectares": 100
  },
  "cropType": "corn",
  "variety": "Pioneer 30F35",
  "growthStage": {
    "code": "BBCH-51",
    "description": "Flowering - tassel emergence",
    "daysSinceSeeding": 65,
    "estimatedDaysToHarvest": 45
  },
  "measurements": {
    "plantHeight": {"value": 180, "unit": "cm"},
    "leafAreaIndex": {"value": 5.5, "unit": "m²/m²"},
    "chlorophyllSPAD": {"value": 50.0, "unit": "SPAD"},
    "ndvi": {"value": 0.82}
  },
  "healthStatus": {
    "overall": "stressed",
    "diseases": [],
    "pests": [
      {
        "species": "Fall Armyworm (Spodoptera frugiperda)",
        "severity": "high",
        "confidence": 0.92
      }
    ]
  },
  "predictions": {
    "yieldEstimate": {
      "value": 8500,
      "unit": "kg/ha",
      "confidence": 0.70
    },
    "harvestDate": "2025-09-25",
    "diseaseRisk": 25
  }
}
```

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial release with full schema |

---

**Philosophy**: 弘益人間 (Benefit All Humanity)
**License**: MIT
**Contact**: standards@wiastandards.com
**Repository**: https://github.com/WIA-Official/wia-standards/crop-monitoring


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

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-1-DATA-FORMAT validation when the
  organization signs the manifest itself.
- **Multi-tenant gateway** — one shared runtime serves multiple
  tenants via header-based isolation. Typically backed by a
  rate-limited gateway (Envoy or NGINX) and a shared OAuth 2.1
  identity provider. The manifest is per-tenant; the runtime
  publishes a federation manifest that aggregates tenant manifests.
- **Federated mesh** — multiple runtimes peer to one another and
  publish their manifests to a directory service. Each peer signs
  its own manifest; the directory service signs the aggregated
  index. This is the topology used by cross-organization deployments
  that need to compose conformance.
- **Air-gapped batch** — no network connection between the runtime
  and the directory service. The runtime emits a signed evidence
  package on each batch and the operator transports the package via
  out-of-band channels. This is the topology used by regulators that
  prohibit live connectivity from sensitive environments.

Implementations declare their topology in the manifest (see Annex I).
A topology change MUST be reflected in a new manifest signature; the
prior topology's manifest remains valid for the deprecation window
described in Annex H to preserve audit traceability.
