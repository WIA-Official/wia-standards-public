# WIA-IND-029 PHASE 1 — Data Format Specification

**Standard:** WIA-IND-029
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 1 of 4)

---

# WIA-IND-029: Additive Manufacturing Specification v1.0

> **Standard ID:** WIA-IND-029
> **Version:** 1.0.0
> **Published:** 2025-12-27
> **Status:** Active
> **Authors:** WIA Industry Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [3D Printing Technologies](#2-3d-printing-technologies)
3. [CAD File Formats and Processing](#3-cad-file-formats-and-processing)
4. [Slicing Algorithms and Optimization](#4-slicing-algorithms-and-optimization)
5. [Material Specifications](#5-material-specifications)
6. [Print Job Management](#6-print-job-management)
7. [Quality Assurance and Inspection](#7-quality-assurance-and-inspection)
8. [Post-Processing Workflows](#8-post-processing-workflows)
9. [Multi-Material Printing](#9-multi-material-printing)
10. [Large-Scale Industrial Printing](#10-large-scale-industrial-printing)
11. [Print Farm Management](#11-print-farm-management)
12. [Certification for 3D Printed Parts](#12-certification-for-3d-printed-parts)
13. [Communication Protocols](#13-communication-protocols)
14. [Data Models](#14-data-models)
15. [Security and Access Control](#15-security-and-access-control)
16. [Implementation Guidelines](#16-implementation-guidelines)
17. [References](#17-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines a comprehensive standard for additive manufacturing (3D printing) systems, enabling interoperability between design software, slicing engines, printing hardware, quality control systems, and manufacturing execution platforms across the entire AM value chain.

### 1.2 Scope

The standard covers:
- All major 3D printing technologies (FDM, SLA, SLS, MJF, DMLS, etc.)
- CAD file format support and mesh processing
- Advanced slicing algorithms with AI optimization
- Comprehensive material specifications and tracking
- Print job orchestration and queue management
- AI-powered quality inspection and defect detection
- Automated post-processing workflows
- Multi-material and multi-color printing
- Large-format and industrial-scale printing
- Distributed print farm management and optimization
- Quality certification and regulatory compliance

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard democratizes additive manufacturing technology, making advanced 3D printing accessible to innovators, manufacturers, researchers, and educators worldwide. By establishing open standards, we enable:

- **Innovation Acceleration**: Rapid prototyping and product development
- **Sustainable Manufacturing**: Reduced waste and on-demand production
- **Medical Advancement**: Personalized medical devices and implants
- **Educational Access**: Hands-on STEM learning for all
- **Economic Opportunity**: Democratized manufacturing capabilities

### 1.4 Terminology

- **AM**: Additive Manufacturing
- **FDM/FFF**: Fused Deposition Modeling / Fused Filament Fabrication
- **SLA**: Stereolithography
- **DLP**: Digital Light Processing
- **SLS**: Selective Laser Sintering
- **MJF**: Multi Jet Fusion
- **DMLS/SLM**: Direct Metal Laser Sintering / Selective Laser Melting
- **EBM**: Electron Beam Melting
- **STL**: Stereolithography file format
- **3MF**: 3D Manufacturing Format
- **G-code**: Numerical control programming language
- **Infill**: Internal structure pattern
- **Support Structure**: Temporary support for overhangs
- **Layer Height**: Vertical resolution of printed layers
- **Build Volume**: Maximum printable dimensions
- **Retraction**: Filament pullback to prevent stringing
- **OctoPrint**: Open-source 3D printer controller
- **Slicer**: Software that converts 3D models to G-code

### 1.5 Design Principles

1. **Technology Agnostic**: Support all AM technologies and vendors
2. **Open Standards**: Open file formats and protocols
3. **AI-Powered**: Machine learning for optimization and quality
4. **Scalability**: From desktop to industrial print farms
5. **Traceability**: Complete part genealogy and certification
6. **Sustainability**: Material efficiency and recycling
7. **Safety**: Compliance with international safety standards
8. **Accessibility**: Easy-to-use interfaces for all skill levels

---

## 2. 3D Printing Technologies

### 2.1 Material Extrusion (FDM/FFF)

#### 2.1.1 Technology Overview

Fused Deposition Modeling (FDM), also known as Fused Filament Fabrication (FFF), is the most widely used 3D printing technology. Material is heated to a semi-liquid state and extruded through a nozzle, depositing layer by layer.

**Key Parameters:**
- **Nozzle Diameter**: 0.2mm - 1.2mm (standard: 0.4mm)
- **Layer Height**: 0.05mm - 0.4mm
- **Print Temperature**: 180°C - 450°C (material dependent)
- **Bed Temperature**: 0°C - 120°C
- **Print Speed**: 20mm/s - 300mm/s
- **Travel Speed**: 80mm/s - 500mm/s

#### 2.1.2 Process Parameters

```json
{
  "technology": "FDM",
  "nozzle": {
    "diameter": 0.4,
    "material": "hardened-steel",
    "temperature": 240,
    "temperatureTolerance": 2
  },
  "bed": {
    "temperature": 80,
    "levelingType": "auto-mesh",
    "adhesion": "glue-stick",
    "surface": "pei-sheet"
  },
  "motion": {
    "printSpeed": 60,
    "travelSpeed": 120,
    "firstLayerSpeed": 20,
    "wallSpeed": 40,
    "infillSpeed": 80
  },
  "retraction": {
    "distance": 6.5,
    "speed": 45,
    "minimumDistance": 1.5,
    "zhop": 0.2
  },
  "cooling": {
    "fanSpeed": 100,
    "fanSpeedLayer1": 0,
    "minimumLayerTime": 10,
    "liftHead": true
  }
}
```

#### 2.1.3 Material Compatibility

| Material | Temp (°C) | Bed (°C) | Speed (mm/s) | Difficulty |
|----------|-----------|----------|--------------|------------|
| PLA | 190-220 | 0-60 | 60-80 | Easy |
| PETG | 220-250 | 70-85 | 40-60 | Easy |
| ABS | 230-250 | 95-110 | 40-60 | Medium |
| Nylon | 240-280 | 70-90 | 30-50 | Hard |
| TPU | 210-230 | 0-60 | 20-30 | Medium |
| PC | 260-310 | 100-120 | 30-50 | Hard |
| PEEK | 360-400 | 120-150 | 20-40 | Expert |

### 2.2 Vat Photopolymerization (SLA/DLP/MSLA)

#### 2.2.1 Technology Overview

Vat photopolymerization uses UV light to cure liquid resin layer by layer. Variants include:
- **SLA**: Laser-based point scanning
- **DLP**: Projector-based layer curing
- **MSLA**: Masked LCD screen curing

**Key Parameters:**
- **Layer Height**: 0.025mm - 0.1mm
- **XY Resolution**: 25μm - 100μm
- **Wavelength**: 355nm - 405nm
- **Exposure Time**: 1s - 20s per layer
- **Peel Force**: Critical for large parts

#### 2.2.2 Process Parameters

```json
{
  "technology": "MSLA",
  "resin": {
    "type": "standard",
    "wavelength": 405,
    "viscosity": "medium",
    "temperature": 25
  },
  "exposure": {
    "layerHeight": 0.05,
    "normalExposure": 8,
    "bottomExposure": 40,
    "bottomLayers": 6,
    "liftSpeed": 60,
    "liftDistance": 5,
    "retractSpeed": 150
  },
  "support": {
    "type": "heavy",
    "contactDiameter": 0.5,
    "density": 30,
    "angle": 60
  },
  "postCuring": {
    "time": 600,
    "wavelength": 405,
    "temperature": 60
  }
}
```

#### 2.2.3 Resin Types

| Resin Type | Shore Hardness | Tensile Strength | Applications |
|------------|----------------|------------------|--------------|
| Standard | 80D | 50-65 MPa | General purpose, prototypes |
| Tough | 85D | 55 MPa | Functional parts, snaps |
| Flexible | 75A-95A | 15-30 MPa | Gaskets, soft-touch |
| Castable | 80D | 40 MPa | Jewelry casting |
| Dental | 85D | 60 MPa | Dental models, guides |
| Biocompatible | 85D | 50 MPa | Medical devices |
| High-Temp | 90D | 55 MPa | Heat resistance (238°C HDT) |

### 2.3 Powder Bed Fusion (SLS/MJF/DMLS)

#### 2.3.1 Selective Laser Sintering (SLS)

Uses a laser to sinter powdered material (typically nylon) layer by layer.

**Key Parameters:**
- **Layer Height**: 0.08mm - 0.15mm
- **Laser Power**: 30W - 100W
- **Scan Speed**: 5m/s - 25m/s
- **Build Temperature**: 170°C - 200°C (Nylon PA12)
- **Powder Particle Size**: 40μm - 80μm

```json
{
  "technology": "SLS",
  "material": "PA12-nylon",
  "laser": {
    "power": 50,
    "spotSize": 0.4,
    "scanSpeed": 12000,
    "hatchSpacing": 0.1
  },
  "temperature": {
    "buildChamber": 175,
    "partBed": 172,
    "feedBed": 180
  },
  "layerHeight": 0.1,
  "refreshRate": 50
}
```

#### 2.3.2 Multi Jet Fusion (MJF)

HP's proprietary technology using inkjet-applied fusing and detailing agents.

**Advantages:**
- Higher throughput than SLS
- Better surface finish
- More uniform mechanical properties
- Faster cooling

#### 2.3.3 Direct Metal Laser Sintering (DMLS)

Metal powder bed fusion for aerospace, medical, and industrial applications.

**Materials:**
- Stainless Steel 316L, 17-4PH
- Titanium Ti64 (TiAl6V4)
- Aluminum AlSi10Mg
- Inconel 625, 718
- Cobalt Chrome

**Key Parameters:**
- Layer Height: 0.02mm - 0.06mm
- Laser Power: 200W - 400W
- Build Temperature: Room temp (with argon atmosphere)
- Post-Processing: Heat treatment, HIP, machining

### 2.4 Material Jetting

#### 2.4.1 PolyJet Technology

Inkjet-style deposition of photopolymer droplets, immediately cured with UV light.

**Capabilities:**
- Multi-material printing (up to 7 materials)
- Full color (CMYK + support)
- Gradient properties (rigid to flexible)
- Ultra-high resolution (16μm layers)

```json
{
  "technology": "PolyJet",
  "materials": [
    { "slot": 0, "type": "VeroWhitePlus", "use": "rigid-body" },
    { "slot": 1, "type": "Agilus30", "use": "flexible-hinge" },
    { "slot": 2, "type": "SUP706", "use": "support" }
  ],
  "layerHeight": 0.016,
  "printMode": "high-quality",
  "gloss": "glossy"
}
```

### 2.5 Binder Jetting

#### 2.5.1 Technology Overview

Selective deposition of liquid binding agent onto powder bed, followed by sintering.

**Applications:**
- Sand casting molds
- Metal parts (after sintering)
- Full-color sandstone models

**Process:**
1. Spread powder layer
2. Inkjet binder onto selected areas
3. Lower build platform
4. Repeat
5. Post-process: depowdering, sintering (for metals)

### 2.6 Direct Energy Deposition (DED)

#### 2.6.1 Laser Engineered Net Shaping (LENS)

Focused thermal energy melts material as it's deposited.

**Applications:**
- Large aerospace components
- Repair of high-value parts
- Hybrid manufacturing (adding to existing parts)
- Gradient materials

**Materials:**
- Titanium alloys
- Nickel superalloys
- Tool steels

---

## 3. CAD File Formats and Processing

### 3.1 Supported File Formats

#### 3.1.1 STL (Stereolithography)

**Format:** ASCII or Binary
**Encoding:** Triangle mesh (vertices and normals)

```
solid name
  facet normal 0 0 1
    outer loop
      vertex 0 0 0
      vertex 1 0 0
      vertex 0 1 0
    endloop
  endfacet
endsolid name
```

**Limitations:**
- No color information
- No unit specification
- File size can be large
- Potential mesh errors (gaps, inverted normals)

#### 3.1.2 3MF (3D Manufacturing Format)

**Format:** XML-based ZIP archive
**Advantages:**
- Compact file size
- Built-in support for colors, textures, materials
- Multiple objects and build plates
- Slicing information storage
- Unit specification
- Digital signatures

```xml
<?xml version="1.0" encoding="UTF-8"?>


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
