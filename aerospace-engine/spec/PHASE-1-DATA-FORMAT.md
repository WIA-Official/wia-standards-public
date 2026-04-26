# WIA-SPACE-015 PHASE 1 — Data Format Specification

**Standard:** WIA-SPACE-015 Aerospace Engine
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** consolidated from `WIA-SPACE-015-v1.0.md` §4 (Engine
Architecture) and §5 (Materials and Manufacturing)

This document defines the canonical data structures for aerospace
engine type declaration, configuration descriptors, performance maps,
and material certification records. Schemas are expressed in JSON
Schema 2020-12 and are stable for the lifetime of this PHASE.

References:
- ICAO Annex 16 Vol. II (engine emissions)
- ARP-5580 (aerospace recommended practice for engine certification data)
- AS9100D (quality management systems for aviation)
- OpenAPI 3.1, JSON Schema 2020-12

---

## 4. Engine Architecture

### 4.1 Turbofan Engine

#### 4.1.1 Configuration

Modern high-bypass turbofan engines SHALL consist of:

1. **Fan section:**
   - Diameter: 1.5~3.4m
   - Blade count: 16~32
   - Pressure ratio: 1.4~1.7
   - Material: Titanium alloy or carbon fiber composite

2. **Low-pressure compressor (LPC):**
   - Stages: 2~4
   - Pressure ratio: 2~4:1

3. **High-pressure compressor (HPC):**
   - Stages: 8~14
   - Pressure ratio: 15~30:1
   - Overall pressure ratio: 40~60:1

4. **Combustor:**
   - Type: Annular (preferred) or can-annular
   - Temperature: 1,400~1,700°C
   - Pressure: 50~300 bar
   - Combustion efficiency: > 99.5%

5. **High-pressure turbine (HPT):**
   - Stages: 1~2
   - Inlet temperature: 1,400~1,700°C
   - Blade material: Single-crystal nickel superalloy with TBC

6. **Low-pressure turbine (LPT):**
   - Stages: 3~7
   - Drives fan and LPC

#### 4.1.2 Performance Requirements

- **Bypass ratio:** 8~12 (modern civil engines)
- **SFC (cruise):** < 0.55 lb/lbf/hr
- **Noise:** Comply with ICAO Chapter 14
- **Emissions:** Comply with CAEP/10 or later

### 4.2 Turboprop Engine

- **Optimum speed:** 250~400 knots
- **Optimum altitude:** 20,000~30,000 ft
- **Propeller RPM:** 1,000~1,500
- **Reduction gear ratio:** 10:1 ~ 20:1
- **SFC:** 0.5~0.6 lb/shp/hr

### 4.3 Rocket Engines

#### 4.3.1 Liquid Rocket Engines

**Propellant combinations:**
- LOX/RP-1: Isp 300~350s (vacuum)
- LOX/LH2: Isp 420~450s (vacuum)
- LOX/Methane: Isp 360~380s (vacuum)

**Cycle types:**
- Gas generator
- Staged combustion
- Expander
- Full-flow staged combustion (FFSC)

#### 4.3.2 Solid Rocket Motors

- **Propellant:** Composite (AP/HTPB/Al)
- **Isp:** 250~280s (vacuum)
- **Applications:** Boosters, upper stages

---


## 5. Materials and Manufacturing

### 5.1 Nickel Superalloys

High-temperature turbine blades SHALL use nickel-based superalloys:

- **Generations:**
  - 1st Gen: Nimonic 80A (900°C)
  - 2nd Gen: IN-738, Rene 80 (1,000°C)
  - 3rd Gen: CMSX-4, Rene N5 (1,100°C, single crystal)
  - 4th Gen: CMSX-10, TMS-138 (1,150°C, Re+Ru)

- **Microstructure:** Single crystal (preferred for HPT blades)
- **Coating:** Thermal Barrier Coating (TBC) with YSZ ceramic (100~500 μm)

### 5.2 Composite Materials

**Carbon Fiber Reinforced Polymer (CFRP):**
- Applications: Fan blades, fan case, nacelle
- Weight reduction: 50% vs. titanium
- Manufacturing: Automated Fiber Placement (AFP)

**Ceramic Matrix Composites (CMC):**
- Material: SiC/SiC
- Temperature capability: ~1,315°C
- Applications: Turbine shrouds, nozzles, future turbine blades
- Weight reduction: 1/3 vs. nickel alloys

### 5.3 Additive Manufacturing (3D Printing)

**Approved processes:**
- Selective Laser Melting (SLM)
- Electron Beam Melting (EBM)
- Directed Energy Deposition (DED)

**Applications:**
- Fuel nozzles (production: > 100,000 units)
- Turbine blades (titanium aluminide)
- Heat exchangers
- Brackets and housings

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
