# WIA-IND-001 PHASE 3 — Protocol Specification

**Standard:** WIA-IND-001
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 3 of 4)

---

- Water recycling systems
```

**Use Phase**:
```
Water_use = Number of Washes × Water per Wash

Standard machine: 50 L per wash
High-efficiency: 25 L per wash

Example (75 washes, HE machine):
  75 × 25 = 1,875 L
```

### 6.2 Circular Fashion Model

#### 6.2.1 Design for Circularity

**Principles**:
```
1. Durability: Design for longevity
   - Quality materials
   - Reinforced stress points
   - Timeless design (not fast fashion)

2. Modularity: Replaceable components
   - Removable/replaceable buttons, zippers
   - Interchangeable pieces
   - Adjustable sizing

3. Mono-materials: Easy recycling
   - Avoid blends when possible
   - Use separable components
   - Minimize mixed material usage

4. Disassembly: Design for deconstruction
   - Easy-to-remove embellishments
   - Snap buttons instead of sewn
   - Minimal glue usage

5. Information: Clear labeling
   - Material composition
   - Care instructions
   - Recycling information
```

**Circularity Score**:
```
Circularity = (
  Mono-material Usage × 0.25 +
  Recyclable Components × 0.25 +
  Design Longevity × 0.25 +
  Repair Information × 0.15 +
  Take-back Program × 0.10
)
```

#### 6.2.2 Resale & Rental Models

**Resale Value Prediction**:
```
Resale Value = Original Price × (
  Brand Premium × 0.30 +
  Condition × 0.25 +
  Trend Status × 0.20 +
  Rarity × 0.15 +
  Age Factor × 0.10
)

Brand Premium:
  - Luxury: 0.5-0.8
  - Premium: 0.3-0.5
  - Mid-range: 0.2-0.3
  - Fast fashion: 0.05-0.15

Condition:
  - New with tags: 1.0
  - Excellent: 0.9
  - Good: 0.7
  - Fair: 0.5
  - Poor: 0.2

Age Factor:
  - <6 months: 1.0
  - 6-12 months: 0.85
  - 1-2 years: 0.65
  - 2-3 years: 0.45
  - >3 years: 0.25
```

**Rental Pricing**:
```
Rental Price = (
  Original Price × Wear Factor × Occasion Factor
) / Rental Duration

Wear Factor:
  - Designer/Luxury: 0.08-0.12 per day
  - Premium: 0.05-0.08 per day
  - Mid-range: 0.03-0.05 per day

Occasion Factor:
  - Special occasion: ×1.5
  - Workwear: ×1.0
  - Casual: ×0.8

Example:
  $500 designer dress for 4-day rental:
  500 × 0.10 × 1.5 = $75 per day × 4 = $300
```

---

## 7. Universal Sizing

### 7.1 Body Measurement Standards

#### 7.1.1 Key Measurements

**Primary Measurements** (cm):
```json
{
  "height": 165,
  "chest": 88,      // Fullest part of bust
  "waist": 70,      // Natural waistline
  "hips": 95,       // Fullest part of hips
  "inseam": 78,     // Inner leg length
  "arm_length": 60, // Shoulder to wrist
  "shoulder": 40,   // Shoulder to shoulder
  "neck": 35        // Neck circumference
}
```

**Secondary Measurements**:
```json
{
  "under_bust": 78,
  "high_hip": 90,
  "thigh": 58,
  "calf": 36,
  "ankle": 23,
  "wrist": 16,
  "bicep": 28,
  "back_length": 42,
  "front_length": 44,
  "rise": 28
}
```

#### 7.1.2 Size Classification Algorithm

**Multi-dimensional Sizing**:
```
Size = f(Chest, Waist, Hips, Height, Proportion)

Base Size Classification:
  XXS: Chest 76-80
  XS:  Chest 81-85
  S:   Chest 86-90
  M:   Chest 91-95
  L:   Chest 96-100
  XL:  Chest 101-106
  XXL: Chest 107-112

Proportion Modifiers:
  - Petite (Height < 160 cm)
  - Regular (160-173 cm)
  - Tall (> 173 cm)

  - Pear (Hips >> Bust)
  - Hourglass (Bust ≈ Hips, small waist)
  - Apple (Bust > Hips, fuller waist)
  - Rectangle (Bust ≈ Waist ≈ Hips)
```

**Machine Learning Size Prediction**:
```
Model: XGBoost Classifier

Features:
  - Body measurements (8-15 dimensions)
  - Garment type
  - Brand sizing history
  - Fabric stretch factor
  - User's past purchases and returns
  - Similar users' data

Output:
  - Primary size: S (confidence: 85%)
  - Alternative: M (confidence: 12%)
  - Fit prediction: "May run small"

Accuracy: 92% within one size
Return Reduction: 38% when used
```

### 7.2 Virtual Body Scanning

#### 7.2.1 Scanning Technologies

**Smartphone-based Scanning**:
```
Method: Structure-from-Motion (SfM)
  - User takes 3-5 photos from different angles
  - AI reconstructs 3D body model
  - Extracts measurements automatically

Accuracy: ±2-3 cm
Time: 60-90 seconds
Requirements: Smartphone camera, form-fitting clothes
```

**Depth Camera Scanning**:
```
Method: Time-of-Flight or Structured Light
  - Microsoft Kinect, Intel RealSense, LiDAR
  - Real-time 3D capture
  - High accuracy measurements

Accuracy: ±1-2 cm
Time: 10-30 seconds
Requirements: Depth camera device
```

**Professional 3D Body Scanner**:
```
Method: Multi-camera photogrammetry or laser scanning
  - 360° capture in scanning booth
  - Medical-grade accuracy
  - Full body mesh export

Accuracy: ±0.5 cm
Time: 5-10 seconds
Cost: $10,000-$100,000 per scanner
```

#### 7.2.2 Avatar Generation

**Body Model Parameterization**:
```
Base Model: SMPL (Skinned Multi-Person Linear Model)
  - 6,890 vertices
  - 10 shape parameters (β)
  - 23 joint angles (θ)

Customization:
  Body Shape (β): Controls overall proportions
    β₀: Height
    β₁: Weight/build
    β₂-β₉: Specific body regions

  Pose (θ): Joint rotations for posing
    - Standing neutral (A-pose or T-pose)
    - Walking/movement animations
    - Custom poses

Generated Avatar:
  - Photorealistic texturing
  - Physically accurate proportions
  - Real-time cloth simulation
  - Export formats: FBX, glTF, USDZ
```

---

## 8. Virtual Try-On Technology

### 8.1 AR Try-On

#### 8.1.1 Camera-based Overlay

**Implementation**:
```
Pipeline:
  1. Camera input (RGB video stream)
  2. Body detection (pose estimation)
  3. Segmentation (person vs background)
  4. 3D pose estimation
  5. Garment rendering
  6. Compositing

Technologies:
  - MediaPipe (Google): Body pose detection
  - TensorFlow/PyTorch: ML models
  - ARCore/ARKit: AR frameworks
  - WebGL/Three.js: 3D rendering
```

**Performance Requirements**:
```
Frame Rate: 30 FPS minimum, 60 FPS ideal
Latency: <50ms for real-time feel
Resolution: 720p minimum, 1080p ideal
Tracking: Stable even with movement

Mobile Requirements:
  - iPhone 11+ / Android flagship
  - iOS 13+ / Android 9+
  - GPU acceleration
```

#### 8.1.2 Virtual Fitting Room

**3D Avatar Try-On**:
```
User Input:
  - Body measurements (manual or scanned)
  - Height, weight
  - Body type

Avatar Generation:
  - Create personalized 3D avatar
  - Apply realistic skin tone and features
  - Match proportions to measurements

Garment Fitting:
  - Load 3D garment model
  - Simulate cloth physics on avatar
  - Show realistic draping and fit
  - Multiple viewing angles (360°)

Fit Analysis:
  - Highlight areas of tension (too tight)
  - Show areas of excess fabric (too loose)
  - Provide size recommendation
  - Show fit comparison across sizes
```

### 8.2 Fit Prediction

#### 8.2.1 Size Recommendation

**ML Model**:
```
Input Features:
  - User measurements
  - Garment measurements
  - Fabric stretch
  - Brand sizing curve
  - Historical fit data

Model: Gradient Boosted Trees
  - Trained on 100M+ purchase + return data
  - Includes user feedback on fit

Output:
  {
    "recommended_size": "M",
    "confidence": 0.88,
    "fit_prediction": {
      "overall": "true_to_size",
      "chest": "comfortable",
      "waist": "slightly_loose",
      "length": "perfect"
    },
    "alternative_sizes": [
      {"size": "S", "confidence": 0.10, "note": "May be tight in chest"},
      {"size": "L", "confidence": 0.02, "note": "May be too loose"}
    ]
  }
```

**Confidence Thresholds**:
```
High Confidence (>85%): Strong recommendation
Medium (70-85%): Recommend with notes
Low (<70%): Suggest multiple sizes or measurements
```

#### 8.2.2 Return Risk Prediction

**Model**:
```
Return Probability = f(
  Fit Confidence,
  Size Recommendation Strength,
  User Return History,
  Garment Category,
  Price Point,
  Brand Familiarity
)

Output:
  - Return risk: Low (0-20%), Medium (20-40%), High (>40%)
  - Primary return reason: Size, Color, Quality, Style
  - Interventions: Better photos, size guide, reviews

Business Impact:
  - Reduce returns by 35-45%
  - Improve customer satisfaction
  - Lower logistics costs
  - Environmental benefit (reduced shipping)
```

---

## 9. Trend Prediction

### 9.1 Forecasting Models

#### 9.1.1 Short-term Trends (1-3 months)

**Data Sources**:
- Real-time social media (Instagram, TikTok)
- Google Trends searches
- Retail sales velocity
- Influencer activity

**Model**: LSTM + Attention
```
Accuracy: 85-90% for 1-month predictions
Update Frequency: Daily
Confidence: High for fast fashion, moderate for classics
```

#### 9.1.2 Medium-term Trends (3-12 months)

**Data Sources**:
- Fashion weeks (designer collections)
- Celebrity sightings
- Editorial coverage
- Historical seasonal patterns

**Model**: Ensemble (LSTM + XGBoost + Expert System)
```
Accuracy: 75-82% for 6-month predictions
Update Frequency: Weekly
Confidence: Moderate to high
```

#### 9.1.3 Long-term Trends (1-3 years)

**Data Sources**:
- Macro trends (sustainability, technology)
- Cultural shifts
- Economic indicators
- Generational preferences

**Model**: Scenario planning + Expert input
```
Accuracy: 60-70% for directional trends
Update Frequency: Monthly/Quarterly
Confidence: Moderate
```

### 9.2 Trend Categories

#### 9.2.1 Color Trends

**Analysis**:
```json
{
  "season": "Spring 2026",
  "trending_colors": [
    {
      "name": "Digital Lavender",
      "hex": "#B19CD9",
      "pantone": "18-3838",
      "trend_strength": 0.92,
      "categories": ["digital", "futuristic", "calm"],
      "best_for": ["dresses", "accessories", "athleisure"]
    },
    {
      "name": "Living Coral",
      "hex": "#FF6F61",
      "trend_strength": 0.87,
      "mood": "vibrant, optimistic, warm"
    }
  ],
  "color_families": {
    "pastels": 0.85,
    "earth_tones": 0.78,
    "neons": 0.42,
    "metallics": 0.61
  }
}
```

#### 9.2.2 Style Trends

**Current Trends** (2025-2026):


## Annex E — Implementation Notes for PHASE-3-PROTOCOL

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-3-PROTOCOL.

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
evidence for PHASE-3-PROTOCOL. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-3-protocol/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-3-PROTOCOL with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-3-PROTOCOL does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-3-PROTOCOL.
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
for PHASE-3-PROTOCOL. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P3-PROTOCOL-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-3-PROTOCOL validation when the
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
