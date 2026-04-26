# WIA-CHIP-006 PHASE 1 — Data Format Specification

**Standard:** WIA-CHIP-006 Advanced Packaging
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** consolidated from `advanced-packaging-spec-v1.0.md` §5 (Data
Format Specification), §6 (2.5D Packaging), §7 (3D IC Stacking)

This document defines the canonical data structures for advanced
packaging interoperability: package identification, die topology,
interposer descriptors, TSV/uBump tables, and stacking metadata.
Schemas are expressed in JSON Schema 2020-12 and stable for the
lifetime of this PHASE.

References:
- IEEE 1838-2019 (Test Access Architecture for 3D-Stacked ICs)
- UCIe 1.1 (Universal Chiplet Interconnect Express)
- JEDEC JESD235D (HBM3 specification)
- ISO/IEC 5169:2024 (hardware-component identification)
- OpenAPI 3.1, JSON Schema 2020-12

---

## 5. Data Format Specification

### 5.1 Package Definition Format

All package designs SHALL be described using JSON format conforming to the following schema:

```json
{
  "package": {
    "id": "string",
    "type": "2.5D" | "3D" | "hybrid",
    "version": "string",
    "substrate": {
      "type": "silicon" | "organic",
      "dimensions": {
        "width": "number (mm)",
        "height": "number (mm)",
        "thickness": "number (μm)"
      },
      "layers": "number"
    },
    "dies": [
      {
        "id": "string",
        "position": {"x": "number", "y": "number", "z": "number"},
        "dimensions": {"width": "number", "height": "number"},
        "technology": "string (e.g., '5nm')",
        "power": "number (W)",
        "function": "string"
      }
    ],
    "interconnects": [
      {
        "type": "microbump" | "tsv" | "hybrid",
        "source": "die_id",
        "target": "die_id",
        "count": "number",
        "pitch": "number (μm)",
        "diameter": "number (μm)"
      }
    ]
  }
}
```

### 5.2 Thermal Map Format

Thermal analysis results SHALL be exported in the following format:

```json
{
  "thermal": {
    "timestamp": "ISO8601",
    "ambient": "number (°C)",
    "nodes": [
      {
        "id": "string",
        "position": {"x": "number", "y": "number", "z": "number"},
        "temperature": "number (°C)"
      }
    ],
    "hotspots": [
      {
        "location": {"x": "number", "y": "number", "z": "number"},
        "temperature": "number (°C)",
        "area": "number (mm²)"
      }
    ]
  }
}
```

---


## 6. 2.5D Packaging Standard

### 6.1 Interposer Specifications

#### 6.1.1 Silicon Interposer

Silicon interposers SHALL meet the following requirements:

- **Minimum feature size**: 2μm line/space
- **Metal layers**: 2-8 layers
- **Via diameter**: 5-20μm
- **Wafer thickness**: 100-750μm
- **CTE**: 2.6 ppm/°C

#### 6.1.2 Organic Interposer

Organic interposers SHALL meet:

- **Minimum feature size**: 10μm line/space
- **Metal layers**: 2-4 layers
- **Via diameter**: 50-100μm
- **Thickness**: 200-800μm
- **CTE**: Match to substrate (typically 17 ppm/°C)

### 6.2 Die Placement Rules

Dies on interposer SHALL be placed according to:

- **Minimum die-to-die spacing**: 500μm (silicon), 1000μm (organic)
- **Minimum die-to-edge spacing**: 1000μm
- **Maximum die area coverage**: 70% of interposer area
- **Thermal balance**: ±20% power density between quadrants

### 6.3 Micro-bump Specifications

Micro-bumps connecting dies to interposer SHALL meet:

- **Pitch**: 20-55μm
- **Diameter**: 15-40μm
- **Height**: 10-25μm
- **Material**: Copper pillar with solder cap or pure solder
- **Underfill**: Required for pitches <40μm

---


## 7. 3D IC Stacking Standard

### 7.1 TSV Specifications

TSVs SHALL meet the following requirements:

#### 7.1.1 Via-First TSVs
- **Diameter**: 5-10μm
- **Depth**: 50-100μm
- **Aspect ratio**: 8:1 to 15:1
- **Keep-out zone**: 3× diameter

#### 7.1.2 Via-Last TSVs
- **Diameter**: 10-30μm
- **Depth**: 20-100μm
- **Aspect ratio**: 3:1 to 8:1
- **Keep-out zone**: 2× diameter

### 7.2 Die Stacking Rules

Stacked dies SHALL comply with:

- **Maximum stack height**: 8 dies or 1000μm
- **Die thickness**: 20-100μm (thinned), 775μm (base die)
- **Die size variation**: ±5% within stack
- **Alignment tolerance**: ±2μm

### 7.3 Bonding Specifications

#### 7.3.1 Hybrid Bonding
- **Bond pitch**: 0.4-10μm
- **Bond pad size**: 0.3-8μm
- **Surface roughness**: <2nm Ra
- **Bonding force**: 20-100 kN
- **Bonding temperature**: 200-400°C

#### 7.3.2 Thermal Compression Bonding
- **Bump pitch**: 20-55μm
- **Bonding force**: 50-200 kN
- **Bonding temperature**: 220-280°C
- **Bonding time**: 1-30 seconds

---


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
