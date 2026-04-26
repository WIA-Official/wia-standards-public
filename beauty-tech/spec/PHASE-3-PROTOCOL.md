# WIA-IND-004 PHASE 3 — Protocol Specification

**Standard:** WIA-IND-004
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 3 of 4)

---

## 6. Hair Analysis and Care

### 6.1 Scalp Analysis

**Scalp Health Metrics:**
```
Scalp condition assessment:
1. Sebum level (oil production)
2. Hydration status
3. pH level (ideal: 4.5-5.5)
4. Inflammation/redness
5. Flakiness (dandruff)
6. Sensitivity

Microscopic analysis:
- Follicle density: Follicles per cm²
- Pore condition: Clean vs blocked
- Fungal/bacterial presence
- Blood circulation (thermal imaging)

Classification:
- Oily scalp: Sebum > 200 μg/cm², needs frequent washing
- Dry scalp: Sebum < 100 μg/cm², flakiness without oil
- Balanced: Sebum 100-200 μg/cm², healthy range
- Sensitive: Redness, irritation, inflammation
```

### 6.2 Hair Structure Analysis

**Hair Fiber Assessment:**
```
Thickness measurement:
- Fine: < 50 μm diameter
- Medium: 50-70 μm
- Thick: > 70 μm

Density calculation:
Total density = (Hair count in 1 cm²) × (Average hairs per follicle)

Low: < 100 hairs/cm²
Medium: 100-150 hairs/cm²
High: > 150 hairs/cm²

Porosity test:
- Low porosity: Cuticle tightly closed, water repels
- Medium porosity: Normal moisture absorption
- High porosity: Cuticle damaged, over-absorbs

Elasticity test:
Healthy hair: Stretches 30-50% before breaking
Damaged hair: Breaks with minimal stretching

Damage assessment:
- Cuticle integrity (microscopy)
- Split ends percentage
- Breakage frequency
- Color damage (if applicable)
- Chemical damage (perm, relaxer)
```

### 6.3 Hair Color Analysis and Recommendation

**Natural Hair Color Classification:**
```
Level system (1-10):
1: Black
2-3: Darkest to dark brown
4-5: Medium to light brown
6-7: Dark to medium blonde
8-9: Light to very light blonde
10: Lightest blonde/platinum

Undertone:
- Warm: Golden, red, copper tones
- Cool: Ash, blue, violet tones
- Neutral: Balance of warm and cool

Melanin composition:
- Eumelanin: Brown-black pigment
- Pheomelanin: Red-yellow pigment
- Ratio determines natural color
```

**Virtual Hair Color Try-On:**
```
Hair segmentation algorithm:
1. Semantic segmentation (U-Net, DeepLab)
2. Hair mask refinement
3. Strand-level detail preservation
4. Lighting estimation

Color application:
Result = Base_hair × (1 - Intensity) + New_color × Intensity

Realistic rendering:
- Highlight preservation
- Root shadow gradient
- Natural variation (±10% color shift)
- Shine/gloss simulation
- Dimension (multiple tones)

Maintenance prediction:
- Fade timeline (6-8 weeks typical)
- Root regrowth visibility
- Color refresh frequency
- Damage potential assessment
```

### 6.4 Hair Loss and Growth Tracking

**Trichometry Measurements:**
```
Hair count tracking:
- Total hair count in defined area
- Percentage change over time
- Growth vs resting phase ratio

Growth rate:
Average: 0.3-0.5 mm per day (1 cm per month)
Measurement: Photo tracking of marked area

Miniaturization assessment:
- Follicle size reduction
- Vellus vs terminal hair ratio
- Pattern baldness progression (Norwood, Ludwig scales)

Shedding analysis:
Normal: 50-100 hairs per day
Excessive: > 150 hairs per day

Telogen effluvium detection:
- Increased shedding (2-3 months after trigger)
- Hair pull test (> 6 hairs per pull = positive)
- Trichogram analysis
```

---

## 7. Ingredient Database

### 7.1 Active Ingredients

**Anti-Aging Actives:**
```
Retinoids:
- Retinol: 0.01-1.0%
  - Entry level: 0.01-0.1%
  - Moderate: 0.3-0.5%
  - Advanced: 0.5-1.0%
  - Stability: Requires airless packaging
  - pH: 5.5-6.0
  - Efficacy: Wrinkle reduction 20-30% in 12 weeks

- Retinaldehyde: 0.01-0.1%
  - Faster conversion to retinoic acid
  - Less irritating than retinol
  - Antibacterial properties

- Adapalene: 0.1-0.3%
  - Prescription strength for acne
  - Photostable vs retinol
  - Less irritation

Peptides:
- Matrixyl 3000: 2-10%
  - Palmitoyl tripeptide-1
  - Palmitoyl tetrapeptide-7
  - Collagen synthesis stimulation

- Argireline: 5-10%
  - Acetyl hexapeptide-8
  - "Botox-like" effect (muscle relaxation)
  - Expression line reduction

- Copper peptides: 0.05-2%
  - GHK-Cu
  - Wound healing, anti-inflammatory
  - Collagen and elastin production

Antioxidants:
- Vitamin C (L-Ascorbic Acid): 5-20%
  - pH: 2.5-3.5 (optimal stability)
  - Concentration: 10-15% most effective
  - Paired with Vitamin E and Ferulic Acid

- Vitamin E (Tocopherol): 0.1-5%
  - Oil-soluble antioxidant
  - Moisturizing properties
  - Synergy with Vitamin C

- Ferulic Acid: 0.5-1%
  - Photoprotection enhancement
  - Stabilizes Vitamin C and E
  - Brown spot reduction

- Resveratrol: 0.5-5%
  - Polyphenol antioxidant
  - Anti-inflammatory
  - Sirtuin activation (longevity)

- Coenzyme Q10: 0.01-0.3%
  - Ubiquinone
  - Cellular energy production
  - Wrinkle reduction
```

**Hydration Ingredients:**
```
Humectants:
- Hyaluronic Acid: 0.1-2%
  - Multiple molecular weights
  - Low MW: Penetration (< 50 kDa)
  - Medium MW: Surface hydration (50-1000 kDa)
  - High MW: Film formation (> 1000 kDa)
  - Binds 1000× its weight in water

- Glycerin: 3-10%
  - Most common humectant
  - Safe, effective, inexpensive
  - Can be drying if > 10%

- Sodium PCA: 0.2-2%
  - Natural moisturizing factor (NMF)
  - Superior to glycerin at low humidity

- Panthenol (Pro-Vitamin B5): 1-5%
  - Humectant and emollient
  - Anti-inflammatory
  - Wound healing

Occlusives:
- Petrolatum: 1-100%
  - Most effective occlusive (98% TEWL reduction)
  - Non-comedogenic despite myth
  - Inert, hypoallergenic

- Dimethicone: 1-10%
  - Silicone-based
  - Smooth, non-greasy feel
  - Breathable occlusive

Emollients:
- Ceramides: 0.5-5%
  - Skin barrier repair
  - Lipid bilayer components
  - Types 1, 3, 6 most important

- Squalane: 1-10%
  - Biomimetic lipid
  - Non-comedogenic
  - Antioxidant properties

- Niacinamide (Vitamin B3): 2-10%
  - Barrier strengthening
  - Ceramide synthesis stimulation
  - Multi-functional active
```

**Exfoliants:**
```
Alpha Hydroxy Acids (AHA):
- Glycolic Acid: 5-30%
  - Smallest molecule, best penetration
  - Anti-aging, texture improvement
  - pH 3.0-4.0

- Lactic Acid: 5-30%
  - Larger than glycolic, gentler
  - Hydrating properties
  - pH 3.0-4.0

- Mandelic Acid: 5-15%
  - Largest AHA, most gentle
  - Antibacterial (acne treatment)
  - Safe for darker skin tones

Beta Hydroxy Acid (BHA):
- Salicylic Acid: 0.5-2%
  - Oil-soluble, penetrates pores
  - Anti-inflammatory
  - Acne treatment gold standard
  - pH 3.0-4.0

Poly Hydroxy Acids (PHA):
- Gluconolactone: 4-10%
  - Gentle, no sun sensitivity
  - Antioxidant properties
  - Suitable for sensitive skin

- Lactobionic Acid: 4-10%
  - Hydrating exfoliant
  - Chelating agent
```

**Brightening Ingredients:**
```
- Niacinamide: 2-10%
  - Melanin transfer inhibition
  - Reduces dark spots 30-40% in 8 weeks
  - Multi-functional (also oil control, barrier)

- Alpha Arbutin: 1-2%
  - Tyrosinase inhibitor
  - Stable form of hydroquinone
  - No irritation or sensitization

- Kojic Acid: 1-4%
  - Tyrosinase inhibitor
  - Can cause sensitization > 2%
  - Often paired with Vitamin C

- Tranexamic Acid: 2-5%
  - Melanocyte activation inhibitor
  - Particularly effective for melasma
  - Anti-inflammatory

- Vitamin C (various forms): 5-20%
  - Melanin reduction
  - Photoprotection
  - Collagen synthesis

- Licorice Extract: 1-5%
  - Glabridin (active component)
  - Gentle brightening
  - Anti-inflammatory
```

**Acne Treatment:**
```
- Benzoyl Peroxide: 2.5-10%
  - Antibacterial (kills P. acnes)
  - Keratolytic
  - Can bleach fabrics
  - Start with 2.5% (as effective as 10%, less irritating)

- Salicylic Acid: 0.5-2%
  - Comedolytic
  - Anti-inflammatory
  - Oil-soluble penetration

- Niacinamide: 4-10%
  - Sebum regulation
  - Anti-inflammatory
  - Post-acne mark reduction

- Azelaic Acid: 10-20%
  - Antimicrobial
  - Keratolytic
  - Brightening (PIH treatment)
  - Rosacea treatment

- Sulfur: 3-10%
  - Keratolytic
  - Antibacterial
  - Absorbs excess oil

- Tea Tree Oil: 2.5-5%
  - Natural antimicrobial
  - Anti-inflammatory
  - Can cause sensitization if too high
```

### 7.2 Ingredient Safety Database

**EWG Scoring System (0-10):**
```
0-2: Low hazard (green)
3-6: Moderate hazard (yellow)
7-10: High hazard (red)

Factors considered:
- Cancer risk
- Developmental & reproductive toxicity
- Allergies & immunotoxicity
- Use restrictions
- Data availability

Common high-concern ingredients:
- Parabens (4-7): Endocrine disruption concerns
- Oxybenzone (8): Hormone disruption
- Formaldehyde releasers (7-9): Carcinogen
- Fragrance (8): Allergen, undisclosed ingredients
- Hydroquinone (7): Restricted, carcinogen concerns
```

**Common Allergens:**
```
Preservatives:
- Formaldehyde releasers
- Methylisothiazolinone (MIT)
- Iodopropynyl butylcarbamate

Fragrances:
- Linalool
- Limonene
- Geraniol
- Citronellol
- Eugenol

Others:
- Propylene glycol
- Lanolin
- Essential oils (various)
```

### 7.3 Ingredient Interaction Matrix

**Safe Combinations:**
```
✓ Vitamin C + Vitamin E + Ferulic Acid
✓ Niacinamide + Hyaluronic Acid
✓ Retinol + Peptides (different times)
✓ AHA + Niacinamide (pH buffered)
✓ Vitamin C + SPF (daytime)
```

**Avoid Combinations:**
```
✗ Retinol + Vitamin C (pH conflict, irritation)
✗ Retinol + AHA/BHA (over-exfoliation)
✗ Retinol + Benzoyl Peroxide (deactivation)
✗ Multiple strong acids (irritation)
✗ Copper peptides + Vitamin C (oxidation)
```

---

## 8. Progress Tracking

### 8.1 Photo Standardization

**Consistent Documentation Protocol:**
```
Camera settings:
- Resolution: 12MP minimum
- Flash: Off
- HDR: Off
- Filters: None
- Format: RAW or highest quality JPEG

Positioning:
- Distance: 30 cm from face
- Angles: 0° (frontal), ±45° (oblique), 90° (profile)
- Facial expression: Neutral, relaxed
- Eyes: Open, looking at camera

Lighting:
- Source: D65 daylight equivalent (6500K)
- Intensity: 800-1000 lux
- Direction: Frontal, 45° elevation
- Background: Neutral gray (18% gray card)

Timing:
- Frequency: Weekly or bi-weekly
- Time of day: Same time each session
- Skin preparation: Clean, no products for 2 hours
```

### 8.2 Quantitative Comparison

**Image Registration and Alignment:**
```
Alignment algorithm:
1. Facial landmark detection (68-point or 468-point)
2. Affine transformation (rotation, scale, translation)
3. Perspective correction
4. ROI extraction (same regions)

Metrics for comparison:
- Pixel-wise difference (MSE, SSIM)
- Feature-based matching (SIFT, ORB)
- Histogram comparison (Chi-square distance)
- Perceptual difference (LPIPS)
```

**Quantified Improvements:**
```
Wrinkle reduction:
- Wrinkle length: Baseline vs current (% reduction)
- Wrinkle depth: 3D profiling (mm decrease)
- Wrinkle area: % of skin surface

Pigmentation improvement:
- Spot count: Number reduction
- Spot size: Average area reduction (mm²)
- Color intensity: ΔE reduction
- Evenness score: Increase in uniformity

Texture enhancement:
- Roughness (Ra): Decrease in microns
- Pore size: Average diameter reduction
- Smoothness score: 0-100 scale increase

Hydration increase:
- Corneometer reading: Baseline vs current
- TEWL reduction: g/m²/h decrease
- Visual radiance: Luminosity increase

Overall skin health score:
- Composite metric: 0-100 scale
- Week-over-week change
- Statistical significance (p-value)
```

### 8.3 Timeline Analysis

**Treatment Response Curves:**
```
Typical timelines:
- Hydration: 1-3 days (immediate)
- Exfoliation glow: 3-7 days
- Acne improvement: 2-4 weeks
- Brightening: 4-8 weeks
- Anti-aging (peptides): 8-12 weeks
- Retinol effects: 12-24 weeks
- Scar fading: 3-12 months

Progress tracking intervals:
- Acute conditions (acne): Weekly
- Active treatment (retinol): Bi-weekly
- Maintenance: Monthly
- Long-term (anti-aging): Quarterly

Plateau detection:
- No improvement for 3 consecutive measurements
- Recommendation: Adjust routine or increase concentration
```

---


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
