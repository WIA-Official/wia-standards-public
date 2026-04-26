# WIA-IND-004 PHASE 2 — API Interface Specification

**Standard:** WIA-IND-004
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

---

2. Length extension (+20-50%)
3. Thickness increase (+30-80%)
4. Curl adjustment (lift angle)
5. Color application (black, brown, blue, etc.)

Volume calculation:
Volume = Base_volume × (1 + Intensity × 0.8)
Intensity: 0.3 (natural), 0.6 (volumizing), 0.9 (dramatic)
```

#### 3.2.3 Lip Products

**Lip Segmentation:**
```
Lip detection:
1. Upper lip: Cupid's bow, philtrum ridges
2. Lower lip: Center fullness, corners
3. Vermillion border: Edge definition
4. Highlight area: Center top and bottom

Color application:
Lip_final = Lip_original × (1 - Opacity) + Product_color × Opacity

Opacity by product type:
- Lip gloss: 0.3-0.5
- Tinted balm: 0.4-0.6
- Satin lipstick: 0.7-0.85
- Matte lipstick: 0.9-1.0
- Liquid lipstick: 0.95-1.0
```

**Finish Simulation:**
```
Matte:
- Specular reflection: 0.1
- Surface roughness: High
- Color intensity: 100%

Satin:
- Specular reflection: 0.5
- Surface roughness: Medium
- Color intensity: 95%

Glossy:
- Specular reflection: 0.9
- Surface roughness: Low
- Color intensity: 70%
- Highlight overlay: +30% brightness

Metallic:
- Specular reflection: 0.8
- Metallic particles: Random sparkle overlay
- Color shift: Angle-dependent hue rotation
```

#### 3.2.4 Blush and Contour

**Application Zones:**
```
Blush placement:
1. Apple of cheeks (smile detection)
2. High on cheekbones (lifted effect)
3. Draping style (temple to apple)

Blend area:
- Center: 100% color intensity
- Edge (5-15mm): Gradient fade to 0%
- Feathering: Gaussian blur (σ = 8-12)

Contour zones:
1. Cheekbones: Below natural shadow
2. Jawline: Edge definition
3. Nose bridge: Slimming effect
4. Forehead: Hairline shadow

Contour intensity:
Darken = Original × (1 - Intensity × 0.3)
Intensity: 0.3 (subtle), 0.6 (medium), 0.9 (dramatic)
```

#### 3.2.5 Highlighter

**Placement Algorithm:**
```
Highlight zones:
1. Cheekbone tops (C-shaped curve)
2. Brow bone (below eyebrow arch)
3. Nose bridge (thin line)
4. Cupid's bow (upper lip)
5. Inner eye corners
6. Center of chin

Glow effect:
Highlight = Base × 1.3 + Shimmer_particle × 0.4

Shimmer types:
- Pearl: White/pink shift, fine particles
- Gold: Warm metallic, medium particles
- Champagne: Neutral metallic, fine particles
- Holographic: Multi-color shift, ultra-fine
```

### 3.3 Real-Time AR Rendering

#### 3.3.1 Performance Optimization

**Frame Rate Requirements:**
```
Target: 30 FPS minimum, 60 FPS ideal
Latency: < 100ms end-to-end

Optimization strategies:
1. GPU acceleration (Metal, OpenGL ES)
2. Texture atlasing for products
3. LOD (Level of Detail) based on distance
4. Occlusion culling
5. Predictive tracking (Kalman filter)
```

#### 3.3.2 Lighting Compensation

**Environment Lighting Estimation:**
```
Illumination detection:
1. Spherical harmonics (9 coefficients)
2. Image-based lighting (IBL)
3. Shadow map generation

Color temperature adaptation:
If detected_temp < 4000K: Warm compensation (+yellow)
If detected_temp > 7000K: Cool compensation (+blue)

Dynamic range adjustment:
Makeup_adjusted = Makeup_base × (Detected_brightness / Reference_brightness)
```

---

## 4. Beauty Device IoT

### 4.1 Device Categories and Protocols

#### 4.1.1 Cleansing Devices

**Smart Sonic Cleanser:**
```
Device capabilities:
- Frequency: 100-300 Hz (adjustable)
- Amplitude: 0.5-2.0 mm
- Timer: 60-180 seconds
- Zones: T-zone, U-zone detection
- Pressure sensor: 0-500g force

BLE Communication Protocol:
Service UUID: 0x1800 (Device Information)
Characteristic UUID: 0x2A00 (Device Name)
Custom Service: 0xBEA0 (Beauty Device Service)

Commands:
0x01: Start cleansing
0x02: Stop
0x03: Set frequency (1 byte, 0-255)
0x04: Set timer (2 bytes, seconds)
0x05: Get session data

Data format:
[Timestamp][Duration][Avg_pressure][Zone_coverage][Battery]
```

**Silicone Sonic Cleanser:**
```
Features:
- Pulsation patterns: 8000 pulses/min
- Heat therapy: 37-42°C optional
- Material: Medical-grade silicone
- Waterproof: IPX7 rating

Usage tracking:
- Sessions per week
- Average duration
- Pressure distribution heatmap
- Skin zone coverage (%)
```

#### 4.1.2 LED Light Therapy Devices

**Multi-wavelength LED Mask:**
```
Wavelength specifications:
- Red (630-660 nm): Collagen stimulation, anti-aging
- Blue (405-420 nm): Acne treatment, antibacterial
- Yellow (570-590 nm): Redness reduction, calming
- Green (525-550 nm): Hyperpigmentation, brightening
- Infrared (850-900 nm): Deep tissue healing

Power density: 20-40 mW/cm²
Treatment time: 10-20 minutes
Safety: Automatic shutoff, eye protection

Protocol specification:
Mode selection:
0x10: Red only
0x11: Blue only
0x12: Red + Infrared
0x13: Yellow + Green
0x14: Custom program

Intensity levels: 0-100 (%)
Duration: 1-30 minutes
Pulsed mode: On/off intervals

Data logging:
- Total usage time per wavelength
- Session effectiveness rating (user input)
- Temperature monitoring
- Safety alerts
```

#### 4.1.3 Microcurrent Devices

**Facial Toning Device:**
```
Electrical specifications:
- Current: 100-500 μA (microamps)
- Frequency: 0.1-500 Hz
- Waveform: Sinusoidal, square, or custom
- Safety limit: < 1000 μA maximum

Treatment modes:
1. Lift mode: 300 μA, 0.5 Hz
2. Tone mode: 400 μA, 1.0 Hz
3. Contour mode: 350 μA, 0.3 Hz

Skin contact detection:
- Impedance measurement: 10-100 kΩ
- Auto-shutoff if no contact > 5 seconds
- Gel/conductor required warning

IoT features:
- Session progress tracking
- Personalized intensity adjustment
- Treatment area mapping
- Results photo comparison
```

#### 4.1.4 Skin Analysis Devices

**Smart Skin Scanner:**
```
Sensors:
1. RGB camera: 12MP, macro lens
2. UV camera: 1.3MP, 365nm LED
3. Moisture sensor: Capacitance-based
4. Oil sensor: Photometric
5. Temperature sensor: Infrared

Measurements:
- Hydration: 0-100 arbitrary units
- Oil level: 0-100 (Sebumeter equivalent)
- Temperature: ±0.1°C accuracy
- Pigmentation: Melanin index
- Pore count and size

Data transmission:
- Bluetooth 5.0
- WiFi (2.4GHz/5GHz)
- Cloud sync enabled
- Offline mode with 1000 measurement buffer

Analysis outputs:
- Instant skin health score
- 7-day trend graphs
- Product recommendations
- Treatment effectiveness tracking
```

### 4.2 Device Integration API

**Universal Beauty Device Protocol (UBDP):**
```json
{
  "protocol_version": "1.0",
  "device": {
    "id": "uuid",
    "type": "led_mask | cleanser | scanner | toning",
    "manufacturer": "string",
    "model": "string",
    "firmware_version": "string"
  },
  "capabilities": {
    "modes": ["mode1", "mode2"],
    "intensity_levels": 10,
    "timer_range": [60, 1800],
    "sensors": ["temperature", "pressure"]
  },
  "session": {
    "start_time": "ISO8601",
    "duration": "seconds",
    "mode": "string",
    "settings": {
      "intensity": 75,
      "temperature": 38
    },
    "measurements": {
      "avg_pressure": 250,
      "coverage": 95
    }
  }
}
```

---

## 5. Personalized Skincare

### 5.1 Skin Profiling

**Comprehensive Skin Profile:**
```json
{
  "user_id": "uuid",
  "age": 28,
  "gender": "female",
  "ethnicity": "asian",
  "fitzpatrick_type": "III",
  "baumann_type": "OSPT",

  "measurements": {
    "hydration": 65,
    "oil_level": 78,
    "elasticity": 82,
    "pore_quality": 70,
    "pigmentation_evenness": 75,
    "wrinkle_score": 15,
    "redness_index": 180,
    "skin_age": 26
  },

  "concerns": [
    "large_pores",
    "occasional_breakouts",
    "dark_spots",
    "oily_t_zone"
  ],

  "goals": [
    "minimize_pores",
    "prevent_aging",
    "even_skin_tone",
    "control_oil"
  ],

  "preferences": {
    "natural_ingredients": true,
    "fragrance_free": true,
    "cruelty_free": true,
    "budget": "moderate",
    "routine_complexity": "moderate"
  },

  "allergies": [
    "parabens",
    "artificial_fragrance"
  ],

  "environment": {
    "climate": "humid_subtropical",
    "pollution_level": "moderate",
    "uv_index_avg": 7,
    "indoor_heating": true
  }
}
```

### 5.2 Product Recommendation Algorithm

**Multi-factor Scoring System:**
```
Recommendation Score =
  Ingredient_efficacy × 0.35 +
  Safety_score × 0.30 +
  Skin_type_match × 0.20 +
  User_reviews × 0.10 +
  Price_value × 0.05

Ingredient Efficacy:
- Clinical studies (gold standard)
- Concentration (% active ingredient)
- Formulation stability
- Penetration enhancers
- Synergistic combinations

Safety Score:
- EWG rating (1-10 scale)
- CIR safety assessment
- EU Cosmetics Regulation compliance
- Allergen content
- Irritation potential

Skin Type Match:
- pH compatibility (4.5-6.5 ideal)
- Texture (gel for oily, cream for dry)
- Comedogenicity rating (0-5 scale)
- Specific concern targeting

User Reviews:
- Average rating (1-5 stars)
- Number of reviews (weight factor)
- Verified purchases only
- Filtered by similar skin type

Price Value:
- Cost per ml
- Expected usage duration
- Concentration of actives
- Multi-function benefits
```

### 5.3 Routine Optimization

**Personalized Regimen Builder:**
```
Morning Routine (4-7 steps):
1. Cleanser (optional if no overnight treatment)
2. Toner/Essence (pH balancing, hydration prep)
3. Treatment serum (vitamin C, niacinamide)
4. Eye cream (optional, if concerns)
5. Moisturizer (SPF if combined)
6. Sunscreen (SPF 30+ PA+++ minimum)

Evening Routine (6-10 steps):
1. Oil cleanser / Makeup remover
2. Water-based cleanser (double cleanse)
3. Exfoliant (2-3x per week)
4. Toner/Essence
5. Treatment serum (retinol, acids)
6. Eye cream
7. Moisturizer
8. Sleeping mask (1-2x per week)
9. Spot treatment (as needed)

Layering principles:
- Thinnest to thickest consistency
- Water-based before oil-based
- pH-dependent order (acids first)
- Wait time between actives (3-5 min)
- Avoid incompatible combinations
```

**Ingredient Conflict Detection:**
```
Incompatible combinations (avoid same routine):
- Retinol + AHA/BHA (irritation risk)
- Vitamin C + Niacinamide (old myth, but pH conflict)
- Retinol + Benzoyl Peroxide (deactivation)
- Multiple acids (over-exfoliation)

Synergistic combinations (enhance together):
- Vitamin C + Vitamin E + Ferulic Acid
- Niacinamide + Zinc
- Hyaluronic Acid + Ceramides
- Retinol + Peptides

pH considerations:
- AHA/BHA: pH 3.0-4.0
- Vitamin C (LAA): pH 2.5-3.5
- Niacinamide: pH 5.0-7.0
- Retinol: pH 5.5-6.0
- Peptides: pH 4.0-7.0
```

### 5.4 Product Database Schema

**Comprehensive Product Information:**
```json
{
  "product_id": "uuid",
  "name": "string",
  "brand": "string",
  "category": "cleanser|serum|moisturizer|sunscreen|etc",
  "sub_category": "string",

  "ingredients": [
    {
      "name": "Niacinamide",
      "inci_name": "Niacinamide",
      "concentration": 5.0,
      "function": ["brightening", "anti-aging", "oil_control"],
      "safety_rating": 1,
      "allergen_potential": "low"
    }
  ],

  "formulation": {
    "ph": 5.5,
    "texture": "gel-cream",
    "color": "white",
    "fragrance": "unscented",
    "preservative_system": ["phenoxyethanol", "ethylhexylglycerin"]
  },

  "suitability": {
    "skin_types": ["oily", "combination", "normal"],
    "concerns": ["large_pores", "dullness", "uneven_tone"],
    "age_range": [20, 50],
    "pregnancy_safe": true,
    "vegan": true,
    "cruelty_free": true
  },

  "usage": {
    "frequency": "twice_daily",
    "when": ["morning", "evening"],
    "amount": "1-2 pumps",
    "duration_months": 3
  },

  "clinical_data": {
    "studies_count": 12,
    "efficacy_proven": true,
    "improvement_percentage": 35,
    "time_to_results_weeks": 4
  },

  "ratings": {
    "average": 4.5,
    "count": 2847,
    "by_skin_type": {
      "oily": 4.7,
      "dry": 4.2,
      "combination": 4.6
    }
  },

  "price": {
    "amount": 39.00,
    "currency": "USD",
    "size_ml": 50,
    "price_per_ml": 0.78
  }
}
```

---


## Annex E — Implementation Notes for PHASE-2-API-INTERFACE

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2-API-INTERFACE.

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
evidence for PHASE-2-API-INTERFACE. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-2-api-interface/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-2-API-INTERFACE with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-2-API-INTERFACE does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-2-API-INTERFACE.
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
for PHASE-2-API-INTERFACE. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P2-API-INTERFACE-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-2-API-INTERFACE validation when the
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
