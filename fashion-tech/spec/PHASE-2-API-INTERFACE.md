# WIA-IND-001 PHASE 2 — API Interface Specification

**Standard:** WIA-IND-001
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

---

Waste Impact = 100 - (Recyclability × 100)
```

**Reference Values**:
```
Max Carbon: 17 kg CO₂e/kg (leather)
Max Water: 125,000 L/kg (wool)
```

#### 4.2.2 Social Impact Score

**Formula**:
```
Social Score (0-100) = (
  Fair Labor × 0.40 +
  Safe Conditions × 0.30 +
  Living Wage × 0.20 +
  Community Impact × 0.10
)

Certifications:
  - Fair Trade: +30 points
  - B-Corp: +25 points
  - SA8000: +20 points
  - GOTS (Organic): +25 points
```

#### 4.2.3 Circular Fashion Score

**Formula**:
```
Circular Score (0-100) = (
  Recyclability × 0.35 +
  Durability × 0.30 +
  Repairability × 0.20 +
  Biodegradability × 0.15
)

Recyclability:
  - Mono-material: 90-100
  - Simple blend: 70-85
  - Complex blend: 40-60
  - Non-recyclable: 0-30

Durability (years of use):
  - <1 year: 20
  - 1-3 years: 50
  - 3-5 years: 70
  - 5-10 years: 85
  - >10 years: 95
```

#### 4.2.4 Total Sustainability Score

**Formula**:
```
Total Sustainability (0-100) = (
  Environmental × 0.40 +
  Social × 0.30 +
  Circular × 0.30
)

Rating Scale:
  90-100: Exceptional (A+)
  80-89:  Excellent (A)
  70-79:  Good (B)
  60-69:  Fair (C)
  50-59:  Poor (D)
  <50:    Very Poor (F)
```

---

## 5. AI & Machine Learning

### 5.1 Trend Prediction

#### 5.1.1 Data Sources

**Social Media Signals** (40% weight):
```
Sources:
  - Instagram: Hashtags, posts, engagement
  - TikTok: Video trends, sounds, challenges
  - Pinterest: Pins, boards, searches
  - Twitter: Fashion conversations, influencer activity

Metrics:
  - Mention Volume: Frequency of style/item mentions
  - Engagement Rate: Likes, shares, comments
  - Velocity: Rate of growth
  - Sentiment: Positive/negative/neutral

Processing:
  - NLP for text analysis
  - Computer vision for image/video analysis
  - Trend clustering algorithms
  - Anomaly detection for emerging trends
```

**Designer & Runway Data** (30% weight):
```
Sources:
  - Fashion weeks: Paris, Milan, NYC, London, Tokyo
  - Designer collections: SS/FW seasons
  - Trade shows: Pitti Uomo, Première Vision
  - Fashion publications: Vogue, WWD, BoF

Analysis:
  - Color palette extraction
  - Silhouette classification
  - Pattern and print categorization
  - Material trend tracking
  - Accessory trends
```

**Retail & Sales Data** (30% weight):
```
Sources:
  - E-commerce sales data
  - Brick-and-mortar POS data
  - Inventory movements
  - Search queries
  - Customer reviews

Metrics:
  - Sales velocity
  - Price elasticity
  - Return rates
  - Restock frequency
  - Search-to-purchase ratio
```

#### 5.1.2 Prediction Models

**Time Series Forecasting**:
```
LSTM Neural Network:
  - Input: Historical trend data (3-5 years)
  - Features: Sales, mentions, runway appearances
  - Output: Trend strength for next 1-12 months
  - Accuracy: 78-85% for 6-month predictions

ARIMA Model:
  - For seasonal patterns
  - Identifies recurring trends
  - Predicts color/style cycles
```

**Classification Models**:
```
Random Forest / XGBoost:
  - Classifies items into trend categories
  - Features: Color, style, price, brand, season
  - Output: Trend/Not Trend, Confidence score

Neural Style Transfer:
  - Identifies visual similarities
  - Predicts style evolution
  - Generates trend mood boards
```

**Confidence Scoring**:
```
Confidence = (
  Data Volume × 0.25 +
  Source Diversity × 0.25 +
  Historical Accuracy × 0.30 +
  Expert Validation × 0.20
)

Confidence Levels:
  85-100%: Very High (Strong prediction)
  70-84%:  High (Reliable prediction)
  55-69%:  Moderate (Possible trend)
  40-54%:  Low (Weak signal)
  <40%:    Very Low (Insufficient data)
```

### 5.2 Style Recommendations

#### 5.2.1 Personalization Engine

**User Profile**:
```json
{
  "demographics": {
    "age": 28,
    "gender": "female",
    "location": "New York, NY"
  },
  "body_measurements": {
    "height": 165,
    "chest": 88,
    "waist": 70,
    "hips": 95,
    "shoe_size": 7.5
  },
  "style_preferences": {
    "styles": ["minimalist", "modern", "sustainable"],
    "colors": ["neutral", "earth_tones", "pastels"],
    "patterns": ["solid", "subtle_stripes"],
    "fit": "fitted"
  },
  "sustainability": {
    "importance": 0.9,
    "min_score": 70,
    "preferred_materials": ["organic_cotton", "tencel", "recycled"]
  },
  "budget": {
    "min": 30,
    "max": 200,
    "average_spend": 85
  },
  "occasions": {
    "work": 0.5,
    "casual": 0.3,
    "evening": 0.1,
    "sport": 0.1
  }
}
```

**Collaborative Filtering**:
```
Similarity(User A, User B) = cosine_similarity(Profile_A, Profile_B)

Recommendation Score = Σ(Similarity × Rating) / Σ(Similarity)

Where:
  - Similarity: User-to-user similarity
  - Rating: Other users' ratings of items
  - Top-K neighbors: 50-100 similar users
```

**Content-Based Filtering**:
```
Item Similarity = f(
  Style Match,
  Color Harmony,
  Price Range,
  Brand Affinity,
  Sustainability Score
)

Features:
  - Item embeddings (512-dim vectors)
  - Style attributes (one-hot encoded)
  - Image features (CNN extracted)
  - Text descriptions (BERT embeddings)
```

**Hybrid Approach**:
```
Final Score = (
  Collaborative × 0.40 +
  Content-Based × 0.35 +
  Trending Items × 0.15 +
  Personalized Ranking × 0.10
)
```

#### 5.2.2 Outfit Composition

**Color Harmony**:
```
Harmony Rules:
  - Monochromatic: Same hue, different tints/shades
  - Analogous: Adjacent colors on color wheel
  - Complementary: Opposite colors
  - Triadic: Three evenly spaced colors
  - Split-Complementary: Base + two adjacent to complement

Compatibility Score:
  Harmony Type Score × Color Balance × Occasion Appropriateness

Where:
  - Harmony Type Score: 0.8-1.0 for rule-following combinations
  - Color Balance: Ratio of dominant/accent colors
  - Occasion: Formal (conservative), Casual (flexible)
```

**Style Coherence**:
```
Coherence = (
  Style Category Match × 0.30 +
  Era Consistency × 0.20 +
  Formality Level × 0.25 +
  Seasonal Appropriateness × 0.25
)

Examples:
  - Minimalist: Clean lines, neutral colors, simple silhouettes
  - Bohemian: Flowing fabrics, prints, layering
  - Preppy: Classic pieces, structured, traditional
  - Streetwear: Urban, casual, branded, comfortable
```

**Wardrobe Optimization**:
```
Utility Score = (Wear Frequency × Versatility × Quality) / Cost Per Wear

Versatility = Number of Compatible Outfits / Total Wardrobe Items

Recommendations:
  - Identify gaps in wardrobe
  - Suggest high-utility additions
  - Recommend items that work with existing pieces
  - Optimize for cost-per-wear efficiency
```

### 5.3 Generative Design

#### 5.3.1 AI Design Assistant

**Text-to-Design**:
```
Input: "A-line midi dress in coral pink with flutter sleeves"

Processing:
  1. NLP to extract attributes:
     - Garment type: Dress
     - Style: A-line
     - Length: Midi
     - Color: Coral pink
     - Details: Flutter sleeves

  2. Generate 3D model:
     - Base mesh from template library
     - Modify proportions for A-line silhouette
     - Add sleeve details
     - Apply material and color

  3. Variations:
     - Generate 3-5 design variations
     - Different sleeve styles, necklines, lengths
     - Color palette variations

Output: 3D model(s) ready for refinement
```

**Style Transfer**:
```
Input: Base garment + Style reference image

Model: Modified StyleGAN / CycleGAN
  - Extracts style features (patterns, textures, colors)
  - Applies to base garment while preserving structure
  - Maintains wearability constraints

Output: New design combining both inputs
```

**Pattern Optimization**:
```
Objective: Minimize fabric waste during cutting

Algorithm:
  1. Generate 2D patterns from 3D model
  2. Optimize pattern placement on fabric roll
  3. Use nesting algorithms (bin packing)
  4. Respect grain line and pattern matching
  5. Calculate material efficiency

Target: >85% fabric utilization (industry average: 75%)
Waste Reduction: 10-15% material savings
```

---

## 6. Sustainability Framework

### 6.1 Lifecycle Assessment

#### 6.1.1 Carbon Footprint Calculation

**Material Production**:
```
Carbon_material = Material Weight (kg) × Carbon Factor (kg CO₂e/kg)

Examples:
  Cotton dress (0.3 kg):  0.3 × 5.9 = 1.77 kg CO₂e
  Polyester jacket (0.5 kg): 0.5 × 7.0 = 3.5 kg CO₂e
  Organic cotton (0.3 kg): 0.3 × 2.1 = 0.63 kg CO₂e
```

**Manufacturing**:
```
Carbon_manufacturing = Base Manufacturing + Complexity Factor

Base: 0.5-1.5 kg CO₂e per garment
Complexity:
  - Simple (T-shirt): ×1.0
  - Medium (dress): ×1.3
  - Complex (jacket): ×1.8
  - Very complex (suit): ×2.5

Example:
  Simple dress: 1.0 × 1.3 = 1.3 kg CO₂e
```

**Transportation**:
```
Carbon_transport = Distance (km) × Weight (kg) × Mode Factor

Mode Factors (kg CO₂e per tonne-km):
  - Ship: 0.01
  - Train: 0.02
  - Truck: 0.06
  - Air: 0.50

Example:
  Garment (0.3 kg) shipped from China (10,000 km):
  Sea: 10,000 × 0.3/1000 × 0.01 = 0.03 kg CO₂e
  Air: 10,000 × 0.3/1000 × 0.50 = 1.5 kg CO₂e
```

**Use Phase**:
```
Carbon_use = (Washing + Drying + Ironing) × Number of Washes

Per wash cycle:
  - Washing (cold water): 0.15 kg CO₂e
  - Washing (hot water): 0.40 kg CO₂e
  - Tumble drying: 0.60 kg CO₂e
  - Ironing: 0.10 kg CO₂e

Example (50 washes, cold wash, line dry):
  0.15 × 50 = 7.5 kg CO₂e
```

**End-of-Life**:
```
Carbon_EOL = Landfill or Recycling

Landfill: 0.3-0.5 kg CO₂e (methane emissions)
Incineration: 0.8-1.2 kg CO₂e
Recycling: -0.5 to -2.0 kg CO₂e (avoided virgin material)
Donation/Resale: -1.0 to -3.0 kg CO₂e (extended life)
```

**Total Lifecycle Carbon**:
```
Total Carbon = Material + Manufacturing + Transport + Use + EOL

Example: Cotton Dress (3-year life, 75 washes)
  Material:      1.77 kg CO₂e
  Manufacturing: 1.30 kg CO₂e
  Transport:     0.03 kg CO₂e (sea)
  Use:          11.25 kg CO₂e (cold wash, line dry)
  End-of-Life:  -1.00 kg CO₂e (donation)

  Total:        13.35 kg CO₂e

  Per Wear:     13.35 / (75 wears) = 0.178 kg CO₂e per wear
```

#### 6.1.2 Water Footprint

**Material Production**:
```
Water (L) = Material Weight (kg) × Water Factor (L/kg)

Cotton: 10,000 L/kg
Organic cotton: 7,000 L/kg
Polyester: 1,000 L/kg
Recycled polyester: 500 L/kg
Linen: 2,500 L/kg
Tencel: 500 L/kg

Example (0.3 kg cotton): 0.3 × 10,000 = 3,000 L
```

**Manufacturing & Dyeing**:
```
Water_manufacturing = 20-200 L per garment

Varies by:
  - Dyeing method (natural dyes use less)
  - Washing processes
  - Finishing treatments


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
