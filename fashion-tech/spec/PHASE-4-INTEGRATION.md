# WIA-IND-001 PHASE 4 — Integration Specification

**Standard:** WIA-IND-001
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 4 of 4)

---

```json
{
  "trending_styles": [
    {
      "name": "Tech-Sustainability",
      "description": "High-tech fabrics with eco-friendly materials",
      "strength": 0.94,
      "keywords": ["smart fabrics", "recycled", "performance"],
      "demographic": "millennials, gen-z"
    },
    {
      "name": "New Romanticism",
      "description": "Soft, flowing silhouettes with vintage inspiration",
      "strength": 0.88,
      "keywords": ["ruffles", "lace", "prairie"],
      "demographic": "women 25-45"
    },
    {
      "name": "Utility Minimalism",
      "description": "Functional design with clean lines",
      "strength": 0.85,
      "keywords": ["pockets", "neutral", "versatile"],
      "demographic": "professionals, urban"
    }
  ]
}
```

---

## 10. Supply Chain & Traceability

### 10.1 Blockchain Integration

#### 10.1.1 Garment Lifecycle Tracking

**Blockchain Record**:
```json
{
  "garment_id": "WIA-IND-001-2025-12345",
  "nft_token": "0x742d35Cc6634C0532925a3b8",
  "stages": [
    {
      "stage": "material_sourcing",
      "timestamp": "2025-01-15T10:00:00Z",
      "location": "Organic Cotton Farm, Tamil Nadu, India",
      "certification": "GOTS Certified",
      "carbon_footprint": 0.63,
      "verified_by": "third_party_auditor"
    },
    {
      "stage": "fabric_production",
      "timestamp": "2025-02-01T14:30:00Z",
      "location": "Textile Mill, Gujarat, India",
      "process": "Sustainable dyeing",
      "water_usage": 50,
      "carbon_footprint": 0.85
    },
    {
      "stage": "garment_manufacturing",
      "timestamp": "2025-02-20T09:00:00Z",
      "location": "Fair Trade Factory, Bangladesh",
      "workers": 25,
      "wage_certification": "Living Wage Certified",
      "carbon_footprint": 1.30
    },
    {
      "stage": "quality_control",
      "timestamp": "2025-03-01T11:00:00Z",
      "passed": true,
      "inspector": "QC-Agent-7721"
    },
    {
      "stage": "retail_distribution",
      "timestamp": "2025-03-15T08:00:00Z",
      "transport": "container_ship",
      "route": "Dhaka -> Hamburg",
      "carbon_footprint": 0.03
    },
    {
      "stage": "first_sale",
      "timestamp": "2025-04-10T16:30:00Z",
      "retailer": "Sustainable Fashion Co.",
      "price": 89.99,
      "currency": "USD"
    }
  ],
  "total_carbon_footprint": 2.81,
  "sustainability_score": 85
}
```

### 10.2 Material Passports

**Digital Product Passport**:
```json
{
  "product_id": "DRESS-2025-12345",
  "brand": "EcoFashion",
  "model": "Coral Summer Dress",
  "material_composition": [
    {
      "material": "Organic Cotton",
      "percentage": 95,
      "origin": "India",
      "certification": "GOTS"
    },
    {
      "material": "Elastane",
      "percentage": 5,
      "recyclable": false
    }
  ],
  "care_instructions": {
    "washing": "Cold water (30°C)",
    "drying": "Line dry",
    "ironing": "Low heat if needed",
    "dry_cleaning": false
  },
  "end_of_life": {
    "recyclability": 0.95,
    "instructions": "Separate elastane waistband before recycling cotton",
    "take_back_program": true,
    "partner": "Fashion Recycle Initiative"
  },
  "repair_information": {
    "common_repairs": ["hem adjustment", "button replacement"],
    "spare_parts": true,
    "repair_guides": "https://brand.com/repair/dress-12345"
  }
}
```

---

## 11. NFT & Digital Fashion

### 11.1 Digital-Only Wearables

#### 11.1.1 Metaverse Fashion

**Platform Support**:
```json
{
  "platforms": [
    {
      "name": "Decentraland",
      "format": "GLB/GLTF 2.0",
      "polygon_limit": 5000,
      "texture_size": "512x512 to 1024x1024",
      "rigging": "Humanoid avatar"
    },
    {
      "name": "The Sandbox",
      "format": "VXM (Voxel)",
      "voxel_limit": 3000,
      "palette": "256 colors"
    },
    {
      "name": "Roblox",
      "format": "RBXM",
      "polygon_limit": 4000,
      "rigging": "R15 or R6"
    },
    {
      "name": "Fortnite",
      "format": "UEFN compatible",
      "requirements": "Epic Games specifications"
    }
  ]
}
```

#### 11.1.2 NFT Fashion Metadata

**Standard NFT Metadata**:
```json
{
  "name": "Digital Couture Dress #001",
  "description": "Exclusive digital dress by Designer X",
  "image": "ipfs://QmXxxx.../image.png",
  "animation_url": "ipfs://QmYyyy.../model.glb",
  "attributes": [
    {
      "trait_type": "Designer",
      "value": "Designer X"
    },
    {
      "trait_type": "Collection",
      "value": "Spring 2026"
    },
    {
      "trait_type": "Rarity",
      "value": "Legendary"
    },
    {
      "trait_type": "Wearable Type",
      "value": "Dress"
    },
    {
      "trait_type": "Platform",
      "value": "Multi-platform"
    }
  ],
  "external_url": "https://digitalfashion.example/dress/001",
  "wia_fashion_data": {
    "standard": "WIA-IND-001",
    "version": "1.0",
    "3d_models": {
      "decentraland": "ipfs://QmZzzz.../decentraland.glb",
      "sandbox": "ipfs://QmWwww.../sandbox.vxm",
      "roblox": "ipfs://QmQqqq.../roblox.rbxm"
    },
    "sustainability": {
      "digital_only": true,
      "carbon_footprint": 0.0,
      "sustainable_design": true
    },
    "unlockable_content": {
      "physical_version": false,
      "ar_filter": "ipfs://QmRrrr.../ar_filter.zip",
      "3d_print_file": false
    }
  }
}
```

### 11.2 Phygital Fashion

**Physical + Digital Combination**:
```json
{
  "product_type": "phygital",
  "physical_item": {
    "sku": "JACKET-2025-789",
    "description": "Limited Edition Designer Jacket",
    "quantity": 100,
    "price": 499.99
  },
  "digital_twin": {
    "nft_contract": "0x123...",
    "token_id": 789,
    "platforms": ["Decentraland", "Sandbox", "AR Filter"],
    "transferable": true
  },
  "authentication": {
    "nfc_chip": true,
    "qr_code": "WIA-IND-001-2025-789",
    "blockchain_verified": true
  },
  "benefits": [
    "Proof of authenticity",
    "Wear in metaverse",
    "Access to exclusive events",
    "Resale tracking",
    "Designer community access"
  ]
}
```

---

## 12. Data Formats

### 12.1 Garment Interchange Format (GIF)

**WIA Fashion JSON Schema**:
```json
{
  "$schema": "https://wiastandards.com/schemas/fashion/v1.0/garment.json",
  "wia_standard": "WIA-IND-001",
  "version": "1.0.0",
  "garment": {
    "id": "unique_garment_id",
    "type": "dress",
    "brand": "Brand Name",
    "name": "Product Name",
    "season": "Spring 2026",
    "gender": "women",
    "category": "dresses",
    "subcategory": "midi_dress",

    "design": {
      "style": "A-line",
      "silhouette": "fitted_bodice_flared_skirt",
      "neckline": "v-neck",
      "sleeves": "flutter_sleeves",
      "length": "midi",
      "closure": "back_zipper"
    },

    "materials": [
      {
        "type": "organic_cotton",
        "percentage": 95,
        "weight_gsm": 180,
        "origin": "India",
        "certification": "GOTS",
        "sustainability_score": 85
      },
      {
        "type": "elastane",
        "percentage": 5
      }
    ],

    "colors": [
      {
        "name": "Coral Pink",
        "hex": "#FF6B9D",
        "pantone": "17-2034 TCX",
        "primary": true
      }
    ],

    "sizes": {
      "system": "WIA Universal",
      "available": ["XS", "S", "M", "L", "XL"],
      "measurements": {
        "S": {
          "chest": 86,
          "waist": 68,
          "hips": 92,
          "length": 105
        }
        // ... other sizes
      }
    },

    "3d_assets": {
      "models": [
        {
          "format": "glTF",
          "lod": "high",
          "url": "https://cdn.example.com/models/dress_001_high.glb",
          "polygon_count": 25000
        },
        {
          "format": "glTF",
          "lod": "medium",
          "url": "https://cdn.example.com/models/dress_001_med.glb",
          "polygon_count": 8000
        }
      ],
      "textures": {
        "base_color": "https://cdn.example.com/textures/dress_001_basecolor.png",
        "normal": "https://cdn.example.com/textures/dress_001_normal.png",
        "roughness": "https://cdn.example.com/textures/dress_001_roughness.png"
      }
    },

    "sustainability": {
      "total_score": 85,
      "environmental_score": 82,
      "social_score": 87,
      "circular_score": 86,
      "carbon_footprint": 13.35,
      "water_footprint": 2150,
      "certifications": ["GOTS", "Fair Trade"]
    },

    "pricing": {
      "retail_price": 89.99,
      "currency": "USD",
      "market": "US"
    },

    "blockchain": {
      "nft_enabled": true,
      "contract_address": "0x742d35Cc6634C0532925a3b8",
      "token_id": 12345,
      "material_passport": "ipfs://QmXxxx..."
    }
  }
}
```

---

## 13. API Interface

### 13.1 RESTful API

**Base URL**: `https://api.wiastandards.com/fashion/v1`

**Authentication**:
```
Authorization: Bearer YOUR_API_KEY
```

**Endpoints**:

```
POST /garments/create
  - Create virtual garment
  - Body: Garment JSON
  - Response: Garment ID + 3D asset URLs

GET /garments/{id}
  - Retrieve garment details
  - Response: Full garment data

POST /virtual-tryon
  - Generate virtual try-on
  - Body: {garment_id, body_measurements, render_mode}
  - Response: Try-on image/video URL

POST /size-recommend
  - Get size recommendation
  - Body: {garment_id, body_measurements}
  - Response: Recommended size + confidence

POST /sustainability/calculate
  - Calculate sustainability score
  - Body: Material, production, transport data
  - Response: Detailed sustainability breakdown

POST /trends/predict
  - Predict fashion trends
  - Body: {season, category, region, timeframe}
  - Response: Trend predictions with confidence scores

POST /wardrobe/optimize
  - Optimize wardrobe
  - Body: {items[], budget, occasions, style_preferences}
  - Response: Recommendations + utility scores
```

---

## 14. Privacy & Security

### 14.1 Data Protection

**Personal Data**:
- Body measurements: Encrypted at rest and in transit
- Photos/scans: Deleted after processing (unless user saves)
- Purchase history: Anonymized for trend analysis
- GDPR/CCPA compliant

**User Rights**:
- Right to access data
- Right to deletion
- Right to export data
- Right to opt-out of data collection

### 14.2 Ethical AI

**Principles**:
- No discriminatory sizing or recommendations
- Transparent AI decision-making
- Diverse training data (body types, ethnicities, ages)
- Regular bias auditing
- User control over personalization

---

## 15. References

### 15.1 Standards

- ISO 4416: Clothing sizes
- ISO 8559: Garment construction
- ISO 14040: Life Cycle Assessment
- GRI Standards: Sustainability reporting
- Textile Exchange: Material standards

### 15.2 Research

- Ellen MacArthur Foundation: Circular Economy
- Fashion Revolution: Transparency Index
- UNFCCC: Fashion Industry Charter
- Textile Exchange: Material Impact Reports

---

**弘익人間 (홍익인간) · Benefit All Humanity**

*WIA Fashion Technology Research Group*
*© 2025 SmileStory Inc. / WIA*
*Version 1.0.0 - Published December 27, 2025*


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
