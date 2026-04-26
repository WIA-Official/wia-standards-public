# WIA-ENE-057: Desertification Prevention
## PHASE 1 - Data Format Specification

**Version:** 1.0.0
**Status:** Standard
**Last Updated:** 2025-12-25

---

## Overview

This document defines the standardized data formats for monitoring land degradation, tracking vegetation health, measuring soil conditions, and recording restoration activities in the WIA-ENE-057 Desertification Prevention standard.

**Philosophy:** 弘益人間 (홍익인간) - Benefit All Humanity

---

## 1. Vegetation Index Data Format

### 1.1 NDVI/EVI Monitoring

```json
{
  "vegetationData": {
    "id": "VEG-20251225-SAHEL-001",
    "locationId": "SAHEL-REGION-001",
    "timestamp": "2025-12-25T10:00:00Z",
    "coordinates": {
      "latitude": 14.5,
      "longitude": -4.0,
      "datum": "WGS84"
    },
    "indices": {
      "ndvi": {
        "value": 0.35,
        "scale": "0-1",
        "source": "MODIS",
        "resolution": "250m"
      },
      "evi": {
        "value": 0.28,
        "scale": "0-1",
        "source": "MODIS",
        "resolution": "250m"
      },
      "lai": {
        "value": 1.2,
        "unit": "m²/m²",
        "description": "Leaf Area Index"
      }
    },
    "coverage": {
      "vegetationCover": 45.2,
      "bareGround": 42.8,
      "water": 2.0,
      "built": 10.0,
      "unit": "percentage"
    },
    "biomass": {
      "aboveGround": {
        "value": 2.4,
        "unit": "tons/hectare"
      },
      "belowGround": {
        "value": 1.8,
        "unit": "tons/hectare"
      }
    },
    "trendAnalysis": {
      "monthlyChange": -0.05,
      "annualChange": -0.15,
      "fiveYearTrend": -0.42,
      "status": "declining",
      "confidence": 0.87
    },
    "metadata": {
      "sensorType": "satellite",
      "cloudCover": 5,
      "quality": "high"
    }
  }
}
```

### 1.2 Field Definitions

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `id` | string | Yes | Unique identifier for this vegetation record |
| `locationId` | string | Yes | Reference to monitored location |
| `timestamp` | ISO8601 | Yes | Time of measurement |
| `coordinates` | object | Yes | Geographic coordinates |
| `indices.ndvi.value` | float | Yes | Normalized Difference Vegetation Index (0-1) |
| `indices.evi.value` | float | Yes | Enhanced Vegetation Index (0-1) |
| `coverage.vegetationCover` | float | Yes | Percentage of vegetation cover |
| `trendAnalysis.status` | enum | Yes | One of: declining, stable, improving |

---

## 2. Soil Condition Data Format

### 2.1 Soil Moisture & Quality

```json
{
  "soilData": {
    "id": "SOIL-20251225-SAHEL-001",
    "locationId": "SAHEL-REGION-001",
    "timestamp": "2025-12-25T10:00:00Z",
    "coordinates": {
      "latitude": 14.5,
      "longitude": -4.0
    },
    "moisture": {
      "surfaceLevel": 12.5,
      "unit": "percentage",
      "depthProfile": [
        {
          "depth": "0-10cm",
          "moisture": 15.2,
          "temperature": 28.5
        },
        {
          "depth": "10-30cm",
          "moisture": 12.8,
          "temperature": 26.2
        },
        {
          "depth": "30-60cm",
          "moisture": 10.1,
          "temperature": 24.8
        }
      ]
    },
    "soilType": {
      "classification": "sandy-loam",
      "texture": {
        "sand": 65,
        "silt": 25,
        "clay": 10,
        "unit": "percentage"
      }
    },
    "chemistry": {
      "ph": 7.2,
      "organicMatter": 1.8,
      "nitrogen": 0.08,
      "phosphorus": 12.5,
      "potassium": 145.2,
      "unit": "ppm for nutrients, percentage for organic matter"
    },
    "degradationIndicators": {
      "erosionRisk": {
        "level": "high",
        "score": 7.5,
        "scale": "0-10"
      },
      "compaction": {
        "level": "moderate",
        "bulkDensity": 1.45,
        "unit": "g/cm³"
      },
      "salinization": {
        "level": "low",
        "ec": 0.8,
        "unit": "dS/m"
      },
      "crusting": {
        "severity": "moderate",
        "thickness": 2.5,
        "unit": "cm"
      }
    },
    "waterRetention": {
      "fieldCapacity": 18.5,
      "wiltingPoint": 8.2,
      "availableWater": 10.3,
      "unit": "percentage"
    }
  }
}
```

### 2.2 Erosion Assessment

```json
{
  "erosionData": {
    "locationId": "SAHEL-REGION-001",
    "assessmentDate": "2025-12-25",
    "erosionType": "wind",
    "severity": {
      "rating": "severe",
      "soilLossRate": {
        "value": 15.2,
        "unit": "tons/hectare/year"
      }
    },
    "factors": {
      "windSpeed": {
        "average": 25.5,
        "gusts": 45.2,
        "unit": "km/h"
      },
      "vegetationCover": 35.2,
      "soilMoisture": 8.5,
      "surfaceRoughness": "low"
    },
    "interventions": [
      "windbreak-establishment",
      "cover-crop-planting",
      "mulching"
    ]
  }
}
```

---

## 3. Rainfall Pattern Data Format

### 3.1 Precipitation Monitoring

```json
{
  "rainfallData": {
    "id": "RAIN-2025-SAHEL-001",
    "locationId": "SAHEL-REGION-001",
    "period": {
      "start": "2025-01-01",
      "end": "2025-12-31",
      "type": "annual"
    },
    "totals": {
      "annualTotal": 325.4,
      "unit": "mm"
    },
    "seasonalDistribution": {
      "wetSeason": {
        "months": ["Jun", "Jul", "Aug", "Sep"],
        "total": 285.2,
        "percentage": 87.6
      },
      "drySeason": {
        "months": ["Oct", "Nov", "Dec", "Jan", "Feb", "Mar", "Apr", "May"],
        "total": 40.2,
        "percentage": 12.4
      }
    },
    "intensityMetrics": {
      "averageEventSize": 18.5,
      "maxSingleEvent": 65.3,
      "daysWithRain": 42,
      "heavyRainDays": 8,
      "heavyRainThreshold": 25.0
    },
    "historicalComparison": {
      "10yearAverage": 425.8,
      "30yearAverage": 485.2,
      "deviation": -23.6,
      "trend": "decreasing",
      "droughtIndex": {
        "spi": -1.8,
        "category": "moderate-drought",
        "description": "Standardized Precipitation Index"
      }
    },
    "eventRecords": [
      {
        "date": "2025-07-15",
        "amount": 65.3,
        "duration": 4.5,
        "intensity": "high",
        "unit": "mm for amount, hours for duration"
      }
    ]
  }
}
```

---

## 4. Land Use & Management Data Format

### 4.1 Land Use Classification

```json
{
  "landUseData": {
    "locationId": "SAHEL-REGION-001",
    "timestamp": "2025-12-25T10:00:00Z",
    "area": {
      "total": 10000,
      "unit": "hectares"
    },
    "classification": {
      "agriculture": {
        "area": 4500,
        "percentage": 45.0,
        "type": "subsistence-farming",
        "intensity": "moderate",
        "crops": ["millet", "sorghum", "peanuts"]
      },
      "grazing": {
        "area": 3200,
        "percentage": 32.0,
        "intensity": "high",
        "animalUnits": 2400,
        "stockingRate": 0.75
      },
      "forest": {
        "area": 1200,
        "percentage": 12.0,
        "canopyCover": 35.5,
        "condition": "degraded"
      },
      "barren": {
        "area": 800,
        "percentage": 8.0,
        "type": "degraded-land"
      },
      "water": {
        "area": 200,
        "percentage": 2.0,
        "type": "seasonal"
      },
      "built": {
        "area": 100,
        "percentage": 1.0
      }
    },
    "managementPractices": {
      "tillage": "conventional",
      "irrigation": "none",
      "fertilization": "minimal",
      "pestManagement": "traditional",
      "soilConservation": ["none"]
    },
    "pressure": {
      "populationDensity": 85,
      "livestockDensity": 240,
      "fuelwoodCollection": "high",
      "overallPressure": "high"
    }
  }
}
```

---

## 5. Restoration Activity Data Format

### 5.1 Restoration Project Record

```json
{
  "restorationProject": {
    "projectId": "REST-2025-SAHEL-001",
    "projectName": "Sahel Green Belt Initiative",
    "organization": "Great Green Wall Foundation",
    "location": {
      "locationId": "SAHEL-REGION-001",
      "coordinates": {
        "latitude": 14.5,
        "longitude": -4.0
      },
      "area": {
        "total": 1000,
        "unit": "hectares"
      }
    },
    "timeline": {
      "startDate": "2023-03-15",
      "plannedEndDate": "2028-03-15",
      "currentPhase": "implementation"
    },
    "objectives": [
      "soil-stabilization",
      "vegetation-recovery",
      "biodiversity-enhancement",
      "livelihood-improvement"
    ],
    "interventions": {
      "reforestation": {
        "area": 450,
        "species": [
          {
            "name": "Acacia senegal",
            "quantity": 25000,
            "survivalRate": 78.5
          },
          {
            "name": "Balanites aegyptiaca",
            "quantity": 15000,
            "survivalRate": 82.3
          }
        ],
        "plantingMethod": "direct-seeding-and-nursery",
        "spacing": "3x3 meters"
      },
      "soilConservation": {
        "measures": [
          {
            "type": "half-moon-technique",
            "area": 200,
            "quantity": 5000
          },
          {
            "type": "stone-bunds",
            "length": 15.5,
            "unit": "km"
          }
        ]
      },
      "waterManagement": {
        "techniques": [
          {
            "type": "rainwater-harvesting",
            "structures": 45,
            "capacity": 2250,
            "unit": "cubic-meters"
          }
        ]
      }
    },
    "progress": {
      "completion": 65.0,
      "milestones": [
        {
          "name": "Site preparation",
          "status": "completed",
          "completionDate": "2023-06-30"
        },
        {
          "name": "Tree planting",
          "status": "completed",
          "completionDate": "2024-09-15"
        },
        {
          "name": "Monitoring & maintenance",
          "status": "in-progress",
          "progress": 40.0
        }
      ]
    },
    "outcomes": {
      "environmental": {
        "vegetationCoverIncrease": 35.2,
        "soilErosionReduction": 60.5,
        "carbonSequestration": {
          "value": 1250,
          "unit": "tons-CO2"
        },
        "biodiversityImprovement": {
          "speciesCount": 42,
          "increase": 25.0
        }
      },
      "social": {
        "beneficiaries": 2500,
        "jobsCreated": 145,
        "incomeIncrease": 18.5
      }
    },
    "funding": {
      "totalBudget": 500000,
      "spent": 325000,
      "currency": "USD",
      "sources": [
        {
          "name": "Green Climate Fund",
          "amount": 300000
        },
        {
          "name": "Local Government",
          "amount": 200000
        }
      ]
    }
  }
}
```

---

## 6. Composite Monitoring Report

### 6.1 Integrated Assessment

```json
{
  "monitoringReport": {
    "reportId": "REPORT-2025-Q4-SAHEL-001",
    "locationId": "SAHEL-REGION-001",
    "reportingPeriod": {
      "start": "2025-10-01",
      "end": "2025-12-31",
      "quarter": "Q4",
      "year": 2025
    },
    "desertificationRisk": {
      "overallScore": 68.5,
      "category": "high-risk",
      "factors": {
        "vegetation": {
          "score": 35.0,
          "weight": 0.30,
          "status": "degraded"
        },
        "soil": {
          "score": 72.0,
          "weight": 0.25,
          "status": "severely-degraded"
        },
        "climate": {
          "score": 85.0,
          "weight": 0.25,
          "status": "adverse"
        },
        "humanActivity": {
          "score": 90.0,
          "weight": 0.20,
          "status": "high-pressure"
        }
      }
    },
    "keyIndicators": {
      "vegetation": {
        "ndvi": 0.35,
        "trend": "declining",
        "change": -0.05
      },
      "soil": {
        "moisture": 12.5,
        "organicMatter": 1.8,
        "erosionRisk": "high"
      },
      "rainfall": {
        "annual": 325.4,
        "deviation": -23.6,
        "trend": "decreasing"
      }
    },
    "alerts": [
      {
        "severity": "high",
        "type": "rapid-vegetation-decline",
        "message": "NDVI decreased by 15% in 3 months",
        "date": "2025-11-15"
      },
      {
        "severity": "medium",
        "type": "soil-moisture-low",
        "message": "Soil moisture below critical threshold",
        "date": "2025-12-10"
      }
    ],
    "recommendations": [
      "Implement immediate soil conservation measures",
      "Establish green belt with drought-resistant species",
      "Reduce grazing intensity by 40%",
      "Introduce water harvesting techniques"
    ],
    "certifications": {
      "ldnCompliant": true,
      "unccdReported": true,
      "verificationStatus": "blockchain-anchored"
    }
  }
}
```

---

## 7. Data Quality & Validation

### 7.1 Quality Assurance

```json
{
  "qualityMetrics": {
    "dataCompleteness": 95.5,
    "accuracy": 92.3,
    "timeliness": 98.7,
    "consistency": 94.1,
    "validationMethod": "automated-and-expert-review",
    "lastValidation": "2025-12-25T10:00:00Z"
  }
}
```

### 7.2 Validation Rules

| Field | Validation Rule | Error Handling |
|-------|----------------|----------------|
| NDVI | Must be between -1 and 1 | Reject if outside range |
| Soil Moisture | Must be 0-100% | Flag for review if >60% |
| Rainfall | Non-negative values | Flag negative values |
| Coordinates | Valid lat/long | Reject invalid coordinates |

---

## 8. Versioning & Compatibility

- **Current Version:** 1.0.0
- **Backwards Compatibility:** Maintained for v0.9.x
- **Breaking Changes:** None in 1.0.0
- **Migration Path:** Automated conversion tools available

---

## 9. Standards Compliance

- **ISO 19115:** Geographic Information - Metadata ✓
- **OGC Standards:** WMS, WFS, WCS ✓
- **UNCCD LDN:** Land Degradation Neutrality Reporting ✓
- **W3C VC:** Verifiable Credentials for Certifications ✓

---

**Document Control:**
- **Author:** WIA Standards Committee
- **Review Date:** 2025-12-25
- **Next Review:** 2026-06-25
- **Philosophy:** 弘益人間 (홍익인간) - Benefit All Humanity

© 2025 SmileStory Inc. / WIA


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
