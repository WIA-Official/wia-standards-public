# WIA-IND-029 PHASE 4 — Integration Specification

**Standard:** WIA-IND-029
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 4 of 4)

---

"mechanicalTesting": {
    "tests": [
      {
        "type": "tensile-strength",
        "standard": "ASTM-D638",
        "specimens": 5,
        "results": {
          "average": 52.3,
          "stdDev": 2.1,
          "min": 49.8,
          "max": 54.7,
          "unit": "MPa",
          "target": 50.0,
          "status": "pass"
        }
      },
      {
        "type": "impact-resistance",
        "standard": "ASTM-D256",
        "results": {
          "average": 45.2,
          "unit": "J/m",
          "target": 40.0,
          "status": "pass"
        }
      }
    ]
  }
}
```

### 7.3 Statistical Process Control

#### 7.3.1 SPC Charting

Monitor process stability:

```json
{
  "spc": {
    "parameter": "layer-height",
    "target": 0.2,
    "ucl": 0.22,
    "lcl": 0.18,
    "samples": [0.201, 0.199, 0.202, 0.198, 0.200],
    "cpk": 1.67,
    "status": "in-control"
  }
}
```

---

## 8. Post-Processing Workflows

### 8.1 Support Removal

#### 8.1.1 Manual Removal

```json
{
  "supportRemoval": {
    "method": "manual",
    "tools": ["flush-cutters", "needle-nose-pliers", "knife"],
    "estimatedTime": 30,
    "skill": "medium",
    "safety": ["gloves", "eye-protection"]
  }
}
```

#### 8.1.2 Soluble Supports

```json
{
  "supportRemoval": {
    "method": "soluble",
    "supportMaterial": "PVA",
    "solution": {
      "type": "water",
      "temperature": 40,
      "agitation": true,
      "duration": 180
    },
    "drying": {
      "method": "air-dry",
      "duration": 120
    }
  }
}
```

### 8.2 Surface Finishing

#### 8.2.1 Sanding

Progressive grit sanding:

```json
{
  "sanding": {
    "method": "wet-sanding",
    "grits": [120, 220, 400, 600, 1000, 2000],
    "estimatedTime": 60,
    "finish": "smooth-matte"
  }
}
```

#### 8.2.2 Vapor Smoothing

For ABS parts:

```json
{
  "vaporSmoothing": {
    "material": "ABS",
    "chemical": "acetone",
    "method": "chamber",
    "temperature": 60,
    "duration": 180,
    "safety": ["ventilation", "gloves", "respirator"],
    "finish": "glossy"
  }
}
```

#### 8.2.3 Coating and Painting

```json
{
  "coating": {
    "primer": {
      "type": "filler-primer",
      "coats": 2,
      "dryTime": 60
    },
    "sanding": {
      "grit": 400,
      "wetSanding": true
    },
    "paint": {
      "type": "acrylic",
      "color": "custom",
      "coats": 3,
      "technique": "spray"
    },
    "clearCoat": {
      "type": "polyurethane",
      "coats": 2,
      "finish": "glossy"
    }
  }
}
```

### 8.3 Heat Treatment

#### 8.3.1 Annealing

Improve strength and heat resistance:

```json
{
  "annealing": {
    "material": "PLA",
    "temperature": 90,
    "duration": 60,
    "cooling": "slow-in-oven",
    "benefits": {
      "heatResistance": "+40°C",
      "strength": "+20%",
      "shrinkage": "2-3%"
    }
  }
}
```

---

## 9. Multi-Material Printing

### 9.1 Multi-Extruder Systems

#### 9.1.1 Independent Dual Extrusion (IDEX)

```json
{
  "system": "IDEX",
  "extruders": [
    {
      "id": 0,
      "material": "PETG",
      "color": "red",
      "nozzle": 0.4,
      "temperature": 235
    },
    {
      "id": 1,
      "material": "PVA",
      "color": "natural",
      "nozzle": 0.4,
      "temperature": 215,
      "purpose": "support"
    }
  ],
  "modes": {
    "duplication": false,
    "mirror": false,
    "multiMaterial": true
  }
}
```

### 9.2 Material Assignment

```json
{
  "materialAssignment": {
    "modelId": "MODEL-123",
    "regions": [
      {
        "regionId": "body",
        "extruder": 0,
        "material": "ABS",
        "infill": 20
      },
      {
        "regionId": "hinge",
        "extruder": 1,
        "material": "TPU",
        "infill": 80
      }
    ]
  }
}
```

---

## 10. Large-Scale Industrial Printing

### 10.1 Build Volume Expansion

Large-format printers (1m³+):

```json
{
  "printer": {
    "model": "Industrial-XL",
    "buildVolume": {
      "x": 1000,
      "y": 1000,
      "z": 1000,
      "unit": "mm"
    },
    "technology": "pellet-extrusion",
    "nozzleSize": 2.0,
    "layerHeight": { "min": 0.5, "max": 2.0 },
    "applications": [
      "furniture",
      "automotive-tooling",
      "architectural-models",
      "concrete-formwork"
    ]
  }
}
```

---

## 11. Print Farm Management

### 11.1 Fleet Management

```json
{
  "printFarm": {
    "farmId": "FARM-001",
    "printers": 50,
    "active": 42,
    "utilization": 84,
    "queuedJobs": 127,
    "completedToday": 315,
    "materials": {
      "spools": 200,
      "lowStock": 12
    }
  }
}
```

### 11.2 Optimization Algorithms

```json
{
  "optimization": {
    "objective": "minimize-makespan",
    "algorithm": "genetic-algorithm",
    "constraints": {
      "materialAvailability": true,
      "skillLevel": true,
      "maintenanceSchedule": true
    }
  }
}
```

---

## 12. Certification for 3D Printed Parts

### 12.1 Quality Certification

```json
{
  "certification": {
    "certId": "CERT-12345",
    "jobId": "JOB-12345",
    "standard": "ISO-9001",
    "inspector": "INSP-001",
    "date": "2025-12-27",
    "traceability": {
      "materialBatch": "BATCH-789",
      "printerId": "PRINTER-001",
      "operator": "OP-042"
    },
    "testResults": {
      "dimensional": "pass",
      "visual": "pass",
      "mechanical": "pass"
    },
    "certificateUrl": "https://cert.example.com/CERT-12345.pdf",
    "blockchainHash": "0x1234567890abcdef",
    "status": "certified"
  }
}
```

---

## 13. Communication Protocols

### 13.1 G-code Communication

Standard G-code commands for 3D printing.

### 13.2 OctoPrint API

REST API for printer control.

### 13.3 MQTT for IoT

Real-time telemetry and control.

---

## 14. Data Models

Complete JSON schemas for all entities defined above.

---

## 15. Security and Access Control

Role-based access control (RBAC) for print farm operations.

---

## 16. Implementation Guidelines

Best practices for implementing WIA-IND-029 compliant systems.

---

## 17. References

- ISO/ASTM 52900: Additive manufacturing — General principles — Terminology
- ASTM F2792: Standard Terminology for Additive Manufacturing Technologies
- ISO/ASTM 52921: Standard Terminology for Additive Manufacturing—Coordinate Systems and Test Methodologies
- G-code specification for 3D printing

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*


## Annex E — Implementation Notes for PHASE-4-INTEGRATION

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-4-INTEGRATION.

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
evidence for PHASE-4-INTEGRATION. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-4-integration/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-4-INTEGRATION with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-4-INTEGRATION does not require bespoke
auditor tooling.
