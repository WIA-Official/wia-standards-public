# WIA-IND-001 PHASE 1 — Data Format Specification

**Standard:** WIA-IND-001
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 1 of 4)

---

# WIA-IND-001: Fashion Tech Specification v1.0

> **Standard ID:** WIA-IND-001
> **Version:** 1.0.0
> **Published:** 2025-12-27
> **Status:** Active
> **Authors:** WIA Fashion Technology Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Digital Fashion Architecture](#2-digital-fashion-architecture)
3. [Virtual Garment Modeling](#3-virtual-garment-modeling)
4. [Material & Fabric Systems](#4-material--fabric-systems)
5. [AI & Machine Learning](#5-ai--machine-learning)
6. [Sustainability Framework](#6-sustainability-framework)
7. [Universal Sizing](#7-universal-sizing)
8. [Virtual Try-On Technology](#8-virtual-try-on-technology)
9. [Trend Prediction](#9-trend-prediction)
10. [Supply Chain & Traceability](#10-supply-chain--traceability)
11. [NFT & Digital Fashion](#11-nft--digital-fashion)
12. [Data Formats](#12-data-formats)
13. [API Interface](#13-api-interface)
14. [Privacy & Security](#14-privacy--security)
15. [References](#15-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the comprehensive framework for fashion technology, providing standardized methods for digital fashion design, virtual try-on, AI-powered recommendations, sustainability tracking, and fashion data interchange. The standard enables seamless integration across the fashion industry ecosystem, from designers to consumers.

### 1.2 Scope

The standard covers:
- Digital fashion design and 3D garment modeling
- Virtual try-on and AR/VR experiences
- AI-powered trend prediction and style recommendations
- Sustainability metrics and environmental impact tracking
- Universal sizing algorithms and body measurement standards
- Material database with physical and digital properties
- Supply chain transparency and traceability
- NFT fashion and digital-only wearables
- Fashion data interchange formats

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to democratize fashion by making design tools accessible, reduce environmental impact through virtual prototyping, eliminate waste through better fit prediction, and create a sustainable, inclusive fashion ecosystem that benefits all of humanity while protecting our planet for future generations.

### 1.4 Terminology

- **Digital Garment**: 3D model of a physical clothing item
- **Virtual Try-On**: AR/VR technology for fitting clothes digitally
- **Fashion AI**: Machine learning models for trend prediction and recommendations
- **Sustainability Score**: Comprehensive metric of environmental and social impact
- **Digital Twin**: Virtual representation of physical garment
- **NFT Fashion**: Blockchain-based digital-only wearables
- **Circular Fashion**: Sustainable lifecycle from design to recycling
- **Metaverse Wearable**: Digital clothing for virtual worlds
- **Body Scan**: 3D capture of human body measurements
- **Fabric Simulation**: Physics-based cloth draping and movement

---

## 2. Digital Fashion Architecture

### 2.1 System Overview

The fashion tech ecosystem consists of:

```
Design Layer → Digital Twin → Virtual Try-On → Production → Consumer → Resale/Recycle
                    ↓
              AI Analysis
                    ↓
         Sustainability Tracking
```

### 2.2 Component Hierarchy

#### 2.2.1 Design & Creation Layer
- **3D Design Tools**: CAD software for garment modeling
- **Material Library**: Digital fabric database with properties
- **Pattern System**: 2D patterns with 3D rendering
- **AI Design Assistant**: Generative design from prompts

#### 2.2.2 Digital Twin Layer
- **3D Garment Model**: Mesh, texture, physics properties
- **Metadata**: Size, material, brand, sustainability data
- **Simulation Engine**: Cloth physics and draping
- **Rendering**: Real-time and photorealistic rendering

#### 2.2.3 Consumer Experience Layer
- **Virtual Try-On**: AR camera overlay or 3D avatar
- **Size Recommendation**: ML-based fit prediction
- **Style Assistant**: AI recommendations
- **Virtual Wardrobe**: Digital closet management

#### 2.2.4 Sustainability Layer
- **Carbon Tracking**: Lifecycle emissions calculation
- **Material Impact**: Water, energy, waste metrics
- **Social Metrics**: Labor conditions, fair trade
- **Circularity**: Recyclability, durability, repairability

### 2.3 Data Flow

#### 2.3.1 Design to Production
```
Designer → 3D Model → Pattern Generation → Sample Approval → Production
           ↓
    Virtual Sample (eliminates 70% of physical samples)
```

#### 2.3.2 Consumer Purchase Flow
```
Browse → Virtual Try-On → Size Recommendation → Purchase → Fit Feedback
            ↓
     Return Rate Reduction (35-45%)
```

#### 2.3.3 Sustainability Tracking
```
Material Sourcing → Production → Transport → Use → End-of-Life
        ↓              ↓            ↓         ↓         ↓
    Carbon Footprint tracked at every stage
```

---

## 3. Virtual Garment Modeling

### 3.1 3D Mesh Specifications

#### 3.1.1 Mesh Requirements

**Topology**:
- Quad-dominant mesh preferred
- Triangle count: 2,000-50,000 per garment
- Edge flow follows fabric grain and stress lines
- Clean topology without overlapping faces

**UV Mapping**:
- Non-overlapping UV islands
- Texture resolution: 2K-4K for hero items, 1K for standard
- Seam placement along natural garment seams
- Texture density consistency across surfaces

**Level of Detail (LOD)**:
```
LOD0: Full detail (20,000-50,000 tris) - Product pages, close-up
LOD1: Medium (5,000-10,000 tris) - Virtual try-on
LOD2: Low (1,000-3,000 tris) - Virtual wardrobe, mobile
LOD3: Ultra-low (500-1,000 tris) - Thumbnails, distant views
```

#### 3.1.2 Mesh Format Standards

**Supported Formats**:
- **glTF 2.0**: Primary format for web and AR
- **FBX**: Design and production workflows
- **OBJ**: Universal compatibility
- **USD/USDZ**: Apple ecosystem
- **Alembic**: Animation and simulation caching

**Required Data**:
```json
{
  "geometry": {
    "vertices": [...],
    "normals": [...],
    "uvs": [...],
    "indices": [...]
  },
  "materials": {
    "baseColor": "texture_or_value",
    "metallic": 0.0,
    "roughness": 0.8,
    "normal": "normal_map",
    "emission": "optional"
  },
  "physics": {
    "clothType": "cotton|silk|denim|leather",
    "weight": 200,  // g/m²
    "stretch": [1.1, 1.05],  // [warp, weft]
    "damping": 0.1
  }
}
```

### 3.2 Fabric Simulation

#### 3.2.1 Cloth Physics Model

**Mass-Spring System**:
```
F = -k(L - L₀) - c(v)
```
Where:
- `F` = Force on particle
- `k` = Spring stiffness (fabric dependent)
- `L` = Current length
- `L₀` = Rest length
- `c` = Damping coefficient
- `v` = Velocity

**Fabric Properties**:
```
Stiffness (N/m):
  Cotton: 100-200
  Silk: 50-100
  Denim: 300-500
  Leather: 500-1000
  Knit: 30-80

Damping (0-1):
  Light fabrics: 0.05-0.1
  Medium weight: 0.1-0.2
  Heavy fabrics: 0.2-0.4

Stretch Factor (0-2):
  Woven: 1.0-1.1
  Stretch denim: 1.1-1.3
  Knit: 1.3-1.8
  Spandex blend: 1.5-2.0
```

#### 3.2.2 Collision Detection

**Body Collision**:
- Use simplified collision mesh for body (500-1000 tris)
- Sphere-based collision for fast approximation
- Continuous collision detection for fast movements
- Self-collision detection for complex draping

**Performance Targets**:
```
Desktop (High-end): 60 FPS with full simulation
Desktop (Mid-range): 30 FPS with full simulation
Mobile (High-end): 30 FPS with simplified simulation
Mobile (Mid-range): 15 FPS with simplified simulation
```

### 3.3 Material System

#### 3.3.1 PBR (Physically-Based Rendering)

**Texture Maps**:
```
Base Color: RGB (sRGB color space)
  - Fabric color and pattern
  - Resolution: 2048x2048 or 4096x4096

Metallic: Grayscale (0 = dielectric, 1 = metal)
  - Most fabrics: 0.0
  - Metallic accents: 1.0

Roughness: Grayscale (0 = smooth, 1 = rough)
  - Silk: 0.1-0.3
  - Cotton: 0.5-0.7
  - Denim: 0.7-0.9
  - Leather: 0.3-0.5

Normal Map: RGB (tangent space)
  - Fabric weave texture
  - Seams and stitching detail

Ambient Occlusion: Grayscale
  - Fabric folds and crevices
  - Enhances depth perception

Emission: RGB (optional)
  - LED/smart fabric elements
  - Reflective materials
```

#### 3.3.2 Fabric Database

**Material Categories**:

**Natural Fibers**:
```json
{
  "cotton": {
    "density": 1.54,  // g/cm³
    "weight_range": [80, 300],  // g/m²
    "carbon_factor": 5.9,  // kg CO₂e per kg
    "water_usage": 10000,  // L per kg
    "recyclability": 0.7,
    "durability": 0.6,
    "breathability": 0.9,
    "moisture_wicking": 0.6,
    "stretch": 1.05,
    "roughness": 0.6
  },
  "organic_cotton": {
    "density": 1.54,
    "weight_range": [80, 300],
    "carbon_factor": 2.1,
    "water_usage": 7000,
    "recyclability": 0.8,
    "sustainability_score": 85
  },
  "linen": {
    "density": 1.5,
    "weight_range": [100, 400],
    "carbon_factor": 2.0,
    "water_usage": 2500,
    "recyclability": 0.9,
    "sustainability_score": 86
  },
  "silk": {
    "density": 1.3,
    "weight_range": [50, 200],
    "carbon_factor": 6.5,
    "water_usage": 8000,
    "recyclability": 0.6,
    "luxury_factor": 0.95,
    "roughness": 0.15
  },
  "wool": {
    "density": 1.31,
    "weight_range": [100, 500],
    "carbon_factor": 10.5,
    "water_usage": 125000,
    "recyclability": 0.8,
    "insulation": 0.95,
    "breathability": 0.85
  }
}
```

**Synthetic Fibers**:
```json
{
  "polyester": {
    "density": 1.38,
    "weight_range": [60, 250],
    "carbon_factor": 7.0,
    "water_usage": 1000,
    "recyclability": 0.3,
    "durability": 0.85,
    "sustainability_score": 35
  },
  "recycled_polyester": {
    "density": 1.38,
    "weight_range": [60, 250],
    "carbon_factor": 3.0,
    "water_usage": 500,
    "recyclability": 0.9,
    "sustainability_score": 82
  },
  "nylon": {
    "density": 1.14,
    "weight_range": [50, 200],
    "carbon_factor": 7.6,
    "water_usage": 1200,
    "recyclability": 0.3,
    "strength": 0.95,
    "sustainability_score": 32
  },
  "spandex": {
    "density": 1.2,
    "weight_range": [100, 300],
    "carbon_factor": 9.0,
    "water_usage": 800,
    "stretch": 1.8,
    "sustainability_score": 28
  }
}
```

**Innovative Materials**:
```json
{
  "tencel": {
    "type": "lyocell",
    "density": 1.5,
    "carbon_factor": 2.5,
    "water_usage": 500,
    "recyclability": 0.95,
    "sustainability_score": 90,
    "biodegradable": true
  },
  "pinatex": {
    "type": "pineapple_leaf_fiber",
    "carbon_factor": 1.5,
    "vegan": true,
    "sustainability_score": 92,
    "leather_alternative": true
  },
  "mushroom_leather": {
    "type": "mycelium",
    "carbon_factor": 1.2,
    "vegan": true,
    "sustainability_score": 94,
    "biodegradable": true
  }
}
```

---

## 4. Material & Fabric Systems

### 4.1 Material Properties

#### 4.1.1 Physical Properties

**Density and Weight**:
```
Fabric Weight (g/m²) = Material Density × Thickness × 1000
Garment Weight (kg) = Fabric Weight × Surface Area / 1000
```

**Drape Coefficient**:
```
Drape = f(Bending Rigidity, Weight, Fabric Construction)
Range: 0 (stiff) to 1 (fluid)

Silk: 0.9-0.95
Cotton: 0.6-0.7
Denim: 0.3-0.4
Leather: 0.2-0.3
```

**Stretch and Recovery**:
```
Stretch Factor = Extended Length / Original Length
Recovery = (Original - Residual Deformation) / (Extended - Original)

Woven cotton: Stretch 1.05, Recovery 0.95
Knit jersey: Stretch 1.4, Recovery 0.85
Stretch denim: Stretch 1.25, Recovery 0.90
Spandex blend: Stretch 1.8, Recovery 0.95
```

#### 4.1.2 Optical Properties

**Color Representation**:
```json
{
  "color": {
    "sRGB": [255, 107, 157],  // Standard web color
    "hex": "#FF6B9D",
    "pantone": "17-2034 TCX",  // Pantone code
    "spectral": [...],  // Spectral reflectance curve (optional)
    "name": "Coral Pink",
    "category": "warm",
    "season": ["spring", "summer"]
  }
}
```

**Reflectance Properties**:
```
Diffuse Reflection: 0.7-0.9 (most fabrics)
Specular Reflection: 0.05-0.3 (depends on finish)
Subsurface Scattering: 0.1-0.4 (translucent fabrics)
Anisotropy: 0.0-0.7 (fabric grain direction)
```

### 4.2 Sustainability Assessment

#### 4.2.1 Environmental Impact Score

**Formula**:
```
Environmental Score (0-100) = 100 - (
  Carbon Impact × 0.35 +
  Water Impact × 0.25 +
  Chemical Impact × 0.20 +
  Waste Impact × 0.20
)

Carbon Impact = (Material Carbon / Max Carbon) × 100
Water Impact = (Material Water / Max Water) × 100
Chemical Impact = Toxicity Rating (0-100)


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
