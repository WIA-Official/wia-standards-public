# WIA-IND-004 PHASE 4 — Integration Specification

**Standard:** WIA-IND-004
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 4 of 4)

---

## 9. Color Science

### 9.1 Color Spaces

**RGB (Red-Green-Blue):**
```
Device-dependent color space
Range: 0-255 per channel (8-bit)
Total colors: 16.7 million (2^24)

sRGB: Standard RGB (web, consumer devices)
Adobe RGB: Wider gamut (professional)
ProPhoto RGB: Widest gamut (editing)

Not perceptually uniform: ΔE ≠ perceived difference
```

**CIELAB (L\*a\*b\*):**
```
Perceptually uniform color space
Standard illuminant: D65 (daylight)

L*: Lightness (0-100)
  - 0 = black
  - 50 = middle gray
  - 100 = white

a*: Green (-) to Red (+)
  - Range: -128 to +127
  - Negative = green
  - Positive = red

b*: Blue (-) to Yellow (+)
  - Range: -128 to +127
  - Negative = blue
  - Positive = yellow

Color difference (ΔE):
ΔE = √((L₁* - L₂*)² + (a₁* - a₂*)² + (b₁* - b₂*)²)

Perceptual interpretation:
ΔE < 1: Not perceptible
ΔE 1-2: Perceptible through close observation
ΔE 2-3.5: Perceptible at a glance
ΔE 3.5-5: Clear difference
ΔE > 5: Colors are more different than similar
```

### 9.2 Skin Tone Classification

**Individual Typology Angle (ITA):**
```
ITA = [arctan((L* - 50) / b*)] × (180 / π)

Classification:
ITA > 55°: Very light
ITA 41-55°: Light
ITA 28-41°: Intermediate
ITA 19-28°: Tan
ITA 10-19°: Brown
ITA < 10°: Dark
```

**Undertone Detection:**
```
Undertone classification:
If b* > 0 and a* > 0: Warm (golden/yellow)
If b* < 0 and a* < 0: Cool (pink/blue)
If |b*| < 5 and |a*| < 5: Neutral
If a* > 0 and b* < 0: Olive (rare)

Vein test correlation:
- Green veins → Warm undertone
- Blue/purple veins → Cool undertone
- Both/unsure → Neutral undertone

Jewelry test:
- Gold looks better → Warm
- Silver looks better → Cool
- Both → Neutral
```

### 9.3 Foundation Matching

**Shade Matching Algorithm:**
```
1. Measure skin tone (L*, a*, b*)
2. Determine undertone (warm/cool/neutral)
3. Calculate ΔE for all foundations in database
4. Filter by undertone category
5. Rank by lowest ΔE
6. Consider oxidation (foundation darkens over time)

Oxidation adjustment:
Final_match = Initial_color + Oxidation_shift
Oxidation typically: +5 to +10 in L* over 2-4 hours

Top 5 matches presented:
1. Exact match (ΔE < 2)
2. Slightly lighter (L* + 2)
3. Slightly darker (L* - 2)
4. Alternative undertone
5. Different formula (matte vs dewy)
```

---

## 10. Data Formats

### 10.1 Skin Analysis Report (JSON)

```json
{
  "report_id": "uuid",
  "user_id": "uuid",
  "timestamp": "2025-12-27T10:30:00Z",
  "version": "1.0",

  "image_metadata": {
    "resolution": "4032x3024",
    "camera": "iPhone 15 Pro",
    "lighting": "natural_daylight",
    "distance_cm": 35,
    "angle": "frontal"
  },

  "classification": {
    "fitzpatrick_type": "III",
    "baumann_type": "OSPT",
    "ita_value": 38.5,
    "undertone": "warm"
  },

  "measurements": {
    "hydration": {
      "value": 68,
      "method": "capacitance",
      "unit": "AU",
      "zone_breakdown": {
        "forehead": 70,
        "cheeks": 72,
        "t_zone": 62
      }
    },
    "elasticity": {
      "value": 0.78,
      "method": "cutometer_R2",
      "age_adjusted": 82
    },
    "pores": {
      "count_per_cm2": 45,
      "average_diameter_um": 320,
      "quality_score": 68
    },
    "wrinkles": {
      "severity_score": 22,
      "depth_max_mm": 0.35,
      "total_length_mm": 145,
      "zones": {
        "forehead": 3,
        "crows_feet": 4,
        "smile_lines": 2
      }
    },
    "pigmentation": {
      "melanin_index": 45,
      "evenness_score": 74,
      "spot_count": 12,
      "hyperpigmentation_area_percent": 3.2
    },
    "redness": {
      "erythema_index": 185,
      "severity": "mild",
      "vessel_density_percent": 8.5
    }
  },

  "overall_scores": {
    "skin_health": 76,
    "skin_age": 27,
    "chronological_age": 30
  },

  "concerns_detected": [
    {
      "concern": "large_pores",
      "severity": "moderate",
      "affected_area": "t_zone"
    },
    {
      "concern": "fine_lines",
      "severity": "mild",
      "affected_area": "eye_area"
    }
  ],

  "recommendations": {
    "immediate": [
      "increase_hydration",
      "add_antioxidant_serum"
    ],
    "long_term": [
      "introduce_retinol",
      "consistent_sunscreen_use"
    ],
    "products": [
      {
        "product_id": "uuid",
        "name": "Hyaluronic Acid Serum",
        "purpose": "hydration",
        "priority": 1
      }
    ]
  }
}
```

### 10.2 Virtual Makeup Session (JSON)

```json
{
  "session_id": "uuid",
  "user_id": "uuid",
  "timestamp": "2025-12-27T14:15:00Z",

  "base_image": {
    "url": "s3://bucket/user_images/original.jpg",
    "hash": "sha256_hash",
    "resolution": "1920x1080"
  },

  "facial_landmarks": {
    "method": "mediapipe_468",
    "confidence": 0.96,
    "points": [
      {"id": 0, "x": 960, "y": 540, "z": 0.1},
      // ... 467 more points
    ]
  },

  "products_applied": [
    {
      "type": "foundation",
      "brand": "Example Brand",
      "name": "Luminous Foundation",
      "shade": "Natural Beige 4.5",
      "coverage": 0.7,
      "finish": "satin",
      "color_lab": {
        "L": 68.5,
        "a": 8.2,
        "b": 18.3
      }
    },
    {
      "type": "blush",
      "brand": "Example Brand",
      "name": "Peachy Glow",
      "color_hex": "#FF9A9E",
      "intensity": 0.6,
      "placement": "apples_of_cheeks",
      "blend_radius": 12
    },
    {
      "type": "lipstick",
      "brand": "Example Brand",
      "name": "Bold Red",
      "color_hex": "#DC143C",
      "finish": "matte",
      "opacity": 0.95
    }
  ],

  "rendering_settings": {
    "lighting_model": "pbr",
    "environment_map": "natural_daylight",
    "skin_subsurface_scattering": true,
    "quality": "high"
  },

  "result_image": {
    "url": "s3://bucket/results/session_uuid.jpg",
    "thumbnail_url": "s3://bucket/results/session_uuid_thumb.jpg"
  },

  "user_feedback": {
    "rating": 5,
    "saved": true,
    "shared": false,
    "purchased": ["product_id_1", "product_id_2"]
  }
}
```

---

## 11. API Interface

### 11.1 RESTful API Endpoints

**Skin Analysis:**
```
POST /api/v1/skin/analyze
Content-Type: multipart/form-data

Request:
{
  "image": <file>,
  "options": {
    "depth": "comprehensive" | "quick",
    "detect_pores": true,
    "detect_wrinkles": true,
    "detect_pigmentation": true,
    "detect_redness": true
  }
}

Response:
{
  "status": "success",
  "report_id": "uuid",
  "data": { /* SkinAnalysisReport */ }
}
```

**Virtual Makeup:**
```
POST /api/v1/makeup/try-on
Content-Type: application/json

Request:
{
  "image_url": "string",
  "products": [
    {
      "type": "lipstick",
      "color": "#DC143C",
      "finish": "matte"
    }
  ],
  "options": {
    "auto_adjust": true,
    "lighting": "natural"
  }
}

Response:
{
  "status": "success",
  "session_id": "uuid",
  "result_url": "string",
  "landmarks": { /* facial landmarks */ }
}
```

**Product Recommendations:**
```
POST /api/v1/recommendations/products
Content-Type: application/json

Request:
{
  "skin_profile": {
    "type": "oily",
    "concerns": ["acne", "large_pores"]
  },
  "preferences": {
    "budget": "moderate",
    "natural": true
  }
}

Response:
{
  "status": "success",
  "recommendations": [
    {
      "product_id": "uuid",
      "score": 92,
      "reasoning": "..."
    }
  ]
}
```

### 11.2 WebSocket API (Real-time AR)

```
wss://api.wia-beauty.com/v1/ar/makeup

Connection message:
{
  "action": "connect",
  "api_key": "string",
  "session_id": "uuid"
}

Frame update:
{
  "action": "frame",
  "image": "base64_encoded_jpg",
  "timestamp": 1234567890
}

Product change:
{
  "action": "apply_product",
  "product": {
    "type": "lipstick",
    "color": "#DC143C"
  }
}

Response (rendered frame):
{
  "action": "frame_result",
  "image": "base64_encoded_jpg",
  "landmarks": [...],
  "latency_ms": 45
}
```

---

## 12. Privacy and Security

### 12.1 Data Protection

**Encryption:**
```
At rest: AES-256-GCM
In transit: TLS 1.3
Key management: AWS KMS / Azure Key Vault

Image storage:
- Original images: Encrypted S3 bucket
- Retention: User-defined (default 1 year)
- Deletion: Immediate upon request
- Anonymization: Face detection with blur/removal option
```

**GDPR Compliance:**
```
Rights provided:
1. Right to access (data export)
2. Right to rectification (update profile)
3. Right to erasure ("right to be forgotten")
4. Right to data portability (JSON export)
5. Right to object (opt-out of processing)

Data minimization:
- Only collect necessary data
- Automatic deletion after retention period
- Anonymization for analytics
```

### 12.2 Authentication and Authorization

**OAuth 2.0 + OpenID Connect:**
```
Authentication flow:
1. User login via provider (Google, Apple, etc.)
2. Receive JWT access token
3. Include in Authorization header

Token format:
Authorization: Bearer <jwt_token>

Token expiration: 1 hour
Refresh token: 30 days
```

**API Key Management:**
```
For partner integrations:
- API key generation (SHA-256 hash stored)
- Rate limiting (1000 req/hour default)
- Usage analytics
- Key rotation support
```

---

## 13. Safety Standards

### 13.1 Cosmetic Regulations

**FDA (United States):**
```
- Color additives: FDA-approved list
- Prohibited ingredients: Lead acetate, mercury compounds
- Labeling requirements: INCI names, warnings
- OTC drug classification: Sunscreen, acne treatments
```

**EU Cosmetics Regulation:**
```
- Banned substances: 1300+ ingredients
- Restricted substances: Concentration limits
- Preservatives: Positive list (parabens limited to 0.4%)
- CMR substances: Carcinogens, mutagens, reprotoxic
- Nanomaterials: Special labeling required
```

### 13.2 Clinical Testing Standards

**Safety Assessment:**
```
Required tests:
1. Patch test (skin irritation)
2. HRIPT (Human Repeat Insult Patch Test)
3. Phototoxicity test
4. Eye irritation test (in vitro alternatives)
5. Sensitization test

Efficacy testing:
1. Instrumental measurements (Corneometer, etc.)
2. Expert grading (dermatologist assessment)
3. Consumer perception studies
4. Statistical analysis (p < 0.05 significance)
```

---

## 14. References

1. Skin Analysis and Measurement:
   - Fluhr, J.W., et al. "Bioengineering of the Skin" (2006)
   - Dobrev, H. "In vivo study of skin mechanical properties" (2000)

2. Color Science:
   - CIE (Commission Internationale de l'Eclairage) Standards
   - Chardon, A., et al. "Skin colour typology and suntanning pathways" (1991)

3. Cosmetic Science:
   - Baumann, L. "Cosmetic Dermatology" (2009)
   - Draelos, Z.D. "Cosmeceuticals" (2009)

4. Ingredients and Formulation:
   - Cosmetic Ingredient Review (CIR) Database
   - Environmental Working Group (EWG) Skin Deep Database
   - European Commission Cosmetic Ingredients Database (CosIng)

5. Regulations:
   - FDA Cosmetics Guidance Documents
   - EU Cosmetics Regulation (EC) No 1223/2009
   - ISO 22715:2006 Cosmetics -- Packaging and labelling

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*This specification is maintained by the WIA Beauty Technology Research Group*
*For updates and contributions, visit: https://github.com/WIA-Official/wia-standards*

*© 2025 SmileStory Inc. / WIA*
*Version 1.0.0 - Published 2025-12-27*


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

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-4-INTEGRATION.
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
for PHASE-4-INTEGRATION. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P4-INTEGRATION-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-4-INTEGRATION validation when the
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
