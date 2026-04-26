# WIA-IND-004 PHASE 1 — Data Format Specification

**Standard:** WIA-IND-004
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 1 of 4)

---

# WIA-IND-004: Beauty Tech Specification v1.0

> **Standard ID:** WIA-IND-004
> **Version:** 1.0.0
> **Published:** 2025-12-27
> **Status:** Active
> **Authors:** WIA Beauty Technology Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Skin Analysis Technology](#2-skin-analysis-technology)
3. [Virtual Makeup Try-On](#3-virtual-makeup-try-on)
4. [Beauty Device IoT](#4-beauty-device-iot)
5. [Personalized Skincare](#5-personalized-skincare)
6. [Hair Analysis and Care](#6-hair-analysis-and-care)
7. [Ingredient Database](#7-ingredient-database)
8. [Progress Tracking](#8-progress-tracking)
9. [Color Science](#9-color-science)
10. [Data Formats](#10-data-formats)
11. [API Interface](#11-api-interface)
12. [Privacy and Security](#12-privacy-and-security)
13. [Safety Standards](#13-safety-standards)
14. [References](#14-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the comprehensive framework for beauty technology applications, providing standardized methods for skin analysis, virtual makeup try-on, beauty device integration, personalized skincare recommendations, and hair care technology. The standard enables consistent implementation across beauty tech platforms, devices, and applications.

### 1.2 Scope

The standard covers:
- AI-powered skin analysis using computer vision and machine learning
- Virtual makeup try-on with augmented reality and real-time rendering
- Beauty device IoT protocols for smart devices (cleansing, LED therapy, etc.)
- Personalized skincare formulation and routine optimization
- Hair and scalp analysis technology
- Comprehensive ingredient safety database (10,000+ ingredients)
- Before/after progress tracking with quantitative metrics
- Professional integration tools for dermatologists and aestheticians
- E-commerce integration for product recommendations

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to democratize access to professional-grade beauty analysis and personalized skincare. By providing open, standardized interfaces, we enable everyone to receive expert-level beauty consultations regardless of their location or economic resources, promoting self-care, confidence, and well-being for all humanity.

### 1.4 Terminology

- **Stratum Corneum**: Outermost layer of the epidermis
- **Melanin**: Pigment responsible for skin color
- **Sebum**: Natural oil produced by sebaceous glands
- **TEWL**: Trans-Epidermal Water Loss
- **Fitzpatrick Scale**: Skin type classification (I-VI)
- **Baumann Scale**: 16-type skin classification system
- **AR**: Augmented Reality
- **CV**: Computer Vision
- **ML**: Machine Learning
- **CIELAB**: Perceptually uniform color space
- **IoT**: Internet of Things
- **BLE**: Bluetooth Low Energy
- **SPF**: Sun Protection Factor
- **PA**: Protection Grade of UVA
- **AHA**: Alpha Hydroxy Acid
- **BHA**: Beta Hydroxy Acid
- **PHA**: Poly Hydroxy Acid

---

## 2. Skin Analysis Technology

### 2.1 Image Acquisition

#### 2.1.1 Camera Requirements

**Minimum Specifications:**
- Resolution: 8MP (3264×2448) minimum, 12MP+ recommended
- Color Depth: 24-bit RGB minimum, 48-bit RAW preferred
- Lens: Fixed focal length 28mm equivalent, f/2.0 or wider
- Auto-focus: Phase detection or contrast detection
- Color Accuracy: ΔE < 3 in sRGB color space

**Lighting Conditions:**
- Natural daylight (D65 illuminant) preferred
- Controlled studio lighting (5000K-6500K color temperature)
- Minimum illumination: 500 lux
- Maximum variation: ±10% across face area
- Flash disabled to avoid specular reflection

#### 2.1.2 Image Capture Protocol

```
Standard Protocol:
1. Subject positioning: 30-50cm from camera
2. Angle: 0° (frontal), ±45° (profile), 90° (lateral)
3. Expression: Neutral, relaxed face
4. Preparation: Clean, makeup-free skin
5. Environment: Controlled lighting, neutral background
6. Multiple captures: 3-5 images per session for consistency
```

#### 2.1.3 Image Preprocessing

```
Processing Pipeline:
1. Color calibration using ColorChecker reference
2. White balance adjustment (D65 standard illuminant)
3. Gamma correction (γ = 2.2)
4. Noise reduction (bilateral filter, σ = 1.5)
5. Face detection and alignment
6. Region of interest (ROI) extraction
7. Normalization to standard size (1024×1024)
```

### 2.2 Skin Type Classification

#### 2.2.1 Fitzpatrick Phototype (I-VI)

**Classification Method:**
```
Melanin Index (MI) = log₁₀(1/R₆₈₀) - log₁₀(1/R₄₂₀)

Type I:   MI < 35   (Very fair, always burns)
Type II:  MI 35-42  (Fair, usually burns)
Type III: MI 42-48  (Medium, sometimes burns)
Type IV:  MI 48-55  (Olive, rarely burns)
Type V:   MI 55-65  (Brown, very rarely burns)
Type VI:  MI > 65   (Very dark, never burns)
```

**Spectrophotometric Measurement:**
- Red channel (620-750nm): Hemoglobin absorption
- Green channel (495-570nm): Melanin and hemoglobin
- Blue channel (450-495nm): Melanin absorption
- UV response: Minimal erythema dose (MED) testing

#### 2.2.2 Baumann Skin Type (16 Types)

**Four Binary Characteristics:**

**1. Oily (O) vs Dry (D)**
```
Sebum Level Measurement:
- Photometric sebum tape analysis
- Impedance measurement
- Visual assessment (Sebumeter® SM 815)

Threshold: Sebum > 150 μg/cm² → Oily
          Sebum < 100 μg/cm² → Dry
```

**2. Sensitive (S) vs Resistant (R)**
```
Sensitivity Indicators:
- Erythema index > 300
- Visible capillaries > 10/cm²
- Self-reported reactions to cosmetics
- Lactic acid sting test positive
- TEWL > 20 g/m²/h
```

**3. Pigmented (P) vs Non-Pigmented (N)**
```
Pigmentation Assessment:
- Melanin index standard deviation > 15
- Visible hyperpigmentation > 5 spots
- Post-inflammatory hyperpigmentation history
- Melasma or age spots present
```

**4. Wrinkle-prone (W) vs Tight (T)**
```
Aging Indicators:
- Wrinkle depth > 0.5mm (crow's feet)
- Elasticity score < 70/100
- Sagging assessment (visual grading scale)
- Pore size increase > 20% vs age 20
- Loss of facial volume
```

### 2.3 Quantitative Skin Metrics

#### 2.3.1 Hydration Measurement

**Methods:**
1. **Capacitance Measurement (Corneometer®)**
   - Range: 0-130 arbitrary units
   - < 30: Very dry
   - 30-40: Dry
   - 40-60: Normal
   - > 60: Well hydrated

2. **Trans-Epidermal Water Loss (TEWL)**
   ```
   TEWL (g/m²/h) = k × (P₁ - P₂) / d

   Where:
   - k = constant (0.0625)
   - P₁ = water vapor pressure at sensor 1
   - P₂ = water vapor pressure at sensor 2
   - d = distance between sensors (cm)

   Normal: < 15 g/m²/h
   Dry: 15-25 g/m²/h
   Very dry: > 25 g/m²/h
   ```

3. **Image-based Estimation**
   ```
   Hydration Score = 100 × (1 - (Wrinkle Density × 0.4 + Flakiness × 0.3 + Dullness × 0.3))
   ```

#### 2.3.2 Elasticity and Firmness

**Cutometer® Measurement:**
```
Parameters:
- R0: Immediate deformation (skin firmness)
- R2: Gross elasticity (Ua/Uf ratio)
- R5: Net elasticity
- R7: Biological elasticity

Elasticity Score = (R2 × 0.4) + (R5 × 0.35) + (R7 × 0.25)

Age-adjusted normal ranges:
Age 20-30: R2 > 0.80
Age 30-40: R2 > 0.75
Age 40-50: R2 > 0.70
Age 50-60: R2 > 0.65
Age 60+:    R2 > 0.60
```

**Suction Method:**
```
Firmness (N/mm) = Force applied / Skin displacement

Young skin: > 0.15 N/mm
Mature skin: 0.10-0.15 N/mm
Aged skin: < 0.10 N/mm
```

#### 2.3.3 Pore Analysis

**Computer Vision Detection:**
```
Pore Detection Algorithm:
1. Convert to grayscale
2. Apply Gaussian blur (σ = 1.0)
3. Adaptive thresholding (block size = 11)
4. Morphological opening (kernel = 3×3)
5. Contour detection (min area = 4 pixels)
6. Ellipse fitting for each pore

Metrics:
- Pore Count: Number per cm²
- Average Diameter: μm
- Size Distribution: Histogram
- Pore Density: % of skin surface

Classification:
Small pores: < 250 μm
Medium pores: 250-400 μm
Large pores: > 400 μm
```

**Pore Quality Score:**
```
Pore Score = 100 - (Count_normalized × 0.4 + Size_normalized × 0.4 + Distribution_variance × 0.2)

Where:
Count_normalized = min(100, (count - 20) × 2)
Size_normalized = min(100, (avg_diameter - 200) / 3)
```

#### 2.3.4 Wrinkle Assessment

**3D Surface Profiling:**
```
Wrinkle Metrics:
- Ra: Average roughness (μm)
- Rz: Maximum peak-to-valley height (μm)
- Rmax: Maximum wrinkle depth (μm)
- Wrinkle length: Total length in ROI (mm)
- Wrinkle area: % of surface area

Depth Classification:
Superficial: < 0.2 mm (fine lines)
Medium: 0.2-0.5 mm (expression lines)
Deep: > 0.5 mm (static wrinkles)
```

**Age-based Wrinkle Severity:**
```
Wrinkle Score = Σ(Depth_i × Length_i) / Area

Severity Scale (0-10):
0-2: Minimal (age 20-30)
2-4: Mild (age 30-40)
4-6: Moderate (age 40-50)
6-8: Significant (age 50-60)
8-10: Severe (age 60+)
```

#### 2.3.5 Pigmentation Analysis

**Melanin and Hemoglobin Quantification:**
```
RGB to Melanin/Hemoglobin Decomposition:

Melanin Index = -100 × log₁₀((1/R) / (1/G))
Hemoglobin Index = -100 × log₁₀((1/R) / (1/B))

Normalized values:
MI_norm = (MI - MI_min) / (MI_max - MI_min) × 100
HI_norm = (HI - HI_min) / (HI_max - HI_min) × 100
```

**Spot Detection:**
```
Pigmentation Spot Algorithm:
1. Convert to LAB color space
2. Extract L* channel (lightness)
3. Calculate local contrast: C = |L*_pixel - L*_mean| / σ
4. Threshold: C > 2.0 for hyperpigmentation
5. Morphological filtering (min size = 2mm²)
6. Classification: Freckles, age spots, melasma

Spot Metrics:
- Total count
- Average size (mm²)
- Color intensity (ΔE from surrounding skin)
- Distribution uniformity
```

**Evenness Score:**
```
Tone Evenness = 100 × (1 - (σ_melanin / μ_melanin))

Excellent: > 90
Good: 80-90
Fair: 70-80
Poor: < 70
```

#### 2.3.6 Redness and Inflammation

**Erythema Index:**
```
EI = 100 × (R - G) / G

Where:
R = Red channel intensity (0-255)
G = Green channel intensity (0-255)

Classification:
Normal: EI < 150
Mild redness: 150-250
Moderate: 250-400
Severe: > 400
```

**Vascular Pattern Analysis:**
```
Capillary Detection:
1. Multi-scale vessel enhancement filter
2. Frangi vesselness measure
3. Hysteresis thresholding
4. Skeleton extraction

Metrics:
- Vessel density (% area)
- Average vessel diameter (μm)
- Tortuosity index
- Distribution pattern

Conditions:
Rosacea: Vessel density > 15%
Spider veins: Large vessels (> 300 μm)
Sensitive skin: High capillary density
```

### 2.4 Overall Skin Health Score

**Composite Scoring Algorithm:**
```
Skin Health Score (0-100) =
  Hydration × 0.25 +
  Elasticity × 0.25 +
  Texture_smoothness × 0.20 +
  Tone_evenness × 0.15 +
  Pore_quality × 0.15

Component calculations:
- Hydration: Corneometer reading normalized to 0-100
- Elasticity: Cutometer R2 × 100
- Texture: 100 - (Roughness / 50)
- Tone: Melanin distribution uniformity
- Pore: Based on size and density

Interpretation:
90-100: Excellent (optimal skin health)
80-89: Very Good (minor improvements possible)
70-79: Good (some concerns to address)
60-69: Fair (multiple concerns)
< 60: Poor (significant concerns)
```

**Skin Age Estimation:**
```
Skin Age = Chronological Age + Age_offset

Age_offset =
  (Wrinkle_score - Wrinkle_expected) × 0.35 +
  (Elasticity_expected - Elasticity_score) × 0.30 +
  (Pigmentation_score - Pigmentation_expected) × 0.20 +
  (Texture_expected - Texture_score) × 0.15

Typical ranges:
Younger than chronological: -10 to 0 years
Same as chronological: ±2 years
Older than chronological: +5 to +20 years
```

---

## 3. Virtual Makeup Try-On

### 3.1 Facial Landmark Detection

#### 3.1.1 Key Point Detection

**468-Point Facial Mesh:**
```
Face regions:
- Face oval: 0-16 (17 points)
- Eyebrows: 17-26, 27-36 (20 points)
- Eyes: 36-47, 48-59 (24 points)
- Nose: 60-87 (28 points)
- Mouth: 88-107 (20 points)
- Jawline: 108-126 (19 points)
- Full mesh: 0-467 (468 points)

Detection methods:
1. MediaPipe Face Mesh (real-time)
2. Dlib 68-point detector (classic)
3. Custom CNN-based detector
4. 3D morphable model fitting
```

#### 3.1.2 3D Face Reconstruction

**Structure from Motion:**
```
3D Reconstruction Pipeline:
1. Multi-view stereo matching
2. Depth map generation
3. Point cloud construction
4. Mesh generation (Poisson surface reconstruction)
5. Texture mapping

Or using single image:
1. Deep learning depth estimation
2. 3DMM (3D Morphable Model) fitting
3. Photometric stereo refinement
```

### 3.2 Makeup Product Categories

#### 3.2.1 Foundation and Base

**Foundation Rendering:**
```
Blending Algorithm:
Result = Base_skin × (1 - Coverage × Alpha) + Foundation_color × Coverage × Alpha

Where:
- Coverage: 0.3 (sheer), 0.6 (medium), 0.9 (full)
- Alpha: Edge feathering (Gaussian blur σ = 5-10)
- Base_skin: Original skin tone (RGB)
- Foundation_color: Product color (RGB)

Finish types:
- Matte: Specular reflection × 0.3
- Satin: Specular reflection × 0.6
- Dewy: Specular reflection × 1.0 + highlight areas
```

**Shade Matching:**
```
Color Match Score = 100 × exp(-ΔE / 10)

Where ΔE (CIELAB color difference):
ΔE = √((L₁* - L₂*)² + (a₁* - a₂*)² + (b₁* - b₂*)²)

Undertone classification:
If b* > 0 and a* > 0: Warm (golden/yellow)
If b* < 0 and a* < 0: Cool (pink/blue)
If |b*| < 5 and |a*| < 5: Neutral
```

#### 3.2.2 Eye Makeup

**Eyeshadow Application:**
```
Region segmentation:
1. Lid space: From lash line to crease
2. Crease: Natural eye fold
3. Brow bone: Below eyebrow
4. Inner corner: Highlight area
5. Outer corner: Depth area

Blending zones:
- Hard edge: 0% blend (eyeliner)
- Soft edge: 30% blend (lid color)
- Full blend: 60% blend (crease, outer V)

Color application:
Color_final = Base_color × (1 - Intensity) + Shadow_color × Intensity
Intensity: 0.3 (sheer), 0.6 (medium), 0.9 (opaque)
```

**Eyeliner Rendering:**
```
Stroke generation:
1. Upper lash line detection
2. Bezier curve fitting
3. Thickness control (1-5 pixels)
4. Wing angle and length (optional)

Styles:
- Pencil: Soft edge, matte finish
- Liquid: Sharp edge, glossy finish
- Gel: Medium edge, semi-matte
```

**Mascara Simulation:**
```
Lash enhancement:
1. Individual lash detection


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
