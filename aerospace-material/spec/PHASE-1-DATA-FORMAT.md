# WIA-SPACE-022 PHASE 1 — Data Format Specification

**Standard:** WIA-SPACE-022 Aerospace Material
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** consolidated from `WIA-SPACE-022-v1.0.md` Material
Categories

This document defines the canonical data structures for aerospace
material identification, characterization, certification, and
traceability. Schemas are expressed in JSON Schema 2020-12 and stable
for the lifetime of this PHASE.

References:
- AMS (SAE Aerospace Material Specifications) numbering scheme
- AS9100D (quality management systems for aviation)
- ASTM E8/E8M-22 (metallic-tension test methods)
- ASTM D3039/D3039M-17 (composite-tension test methods)
- ISO/IEC 5169:2024 (hardware-component identification)
- OpenAPI 3.1, JSON Schema 2020-12

---

## Material Categories

### 1. Aluminum Alloys

**2xxx Series (Al-Cu)**
- 2024-T3: Fuselage skins, pressure bulkheads
  - Tensile Strength: ≥470 MPa
  - Yield Strength: ≥325 MPa
  - Elongation: ≥18%
- 2014-T6: Forged components
- 2219-T87: Cryogenic applications, weldable

**7xxx Series (Al-Zn)**
- 7075-T6: Wing upper surfaces, high-strength structure
  - Tensile Strength: ≥570 MPa
  - Yield Strength: ≥505 MPa
- 7075-T73: Improved SCC resistance
- 7050-T74: Next-generation, improved toughness
- 7085-T74: 10% lighter than 7050

**Requirements:**
- VAR (Vacuum Arc Remelting) for critical applications
- 100% ultrasonic inspection for thick sections
- Alclad coating for corrosion protection
- Traceability to melt lot

### 2. Titanium Alloys

**Ti-6Al-4V (Grade 5)**
- Tensile Strength: ≥950 MPa (annealed)
- Service Temperature: 400°C continuous
- Applications: Engine components, landing gear, fasteners
- Manufacturing: VAR 2×, forging or machining

**High-Temperature Alloys**
- Ti-6Al-2Sn-4Zr-2Mo (Ti-6242): 500°C service
- Ti-6Al-2Sn-4Zr-6Mo: High strength, 650°C
- Ti-5Al-5V-5Mo-3Cr (Ti-5553): Ultra-high strength, 1,200 MPa

**Titanium Aluminides**
- γ-TiAl: 700-800°C, turbine blades
- Density: 3.9 g/cm³
- Applications: GEnx, LEAP, PW1000G engines

**Requirements:**
- Triple VAR for critical rotating components
- 100% UT inspection
- Certified heat treatment
- Controlled atmosphere welding (TIG, EB)

### 3. Composite Materials

**Carbon Fiber Reinforced Polymer (CFRP)**

**Fiber Types:**
- T300: Standard modulus (230 GPa)
- T800, IM7: Intermediate modulus (290 GPa)
- M40J, M55J: High modulus (400-540 GPa)

**Matrix Systems:**
- Epoxy: -55°C to 120°C (e.g., 3501-6, 8552)
- BMI: -55°C to 180°C (e.g., 5250-4)
- PEEK: Thermoplastic, recyclable

**Manufacturing:**
- Autoclave cure: 120-180°C, 6-7 bar
- Out-of-autoclave (OoA): Vacuum only
- RTM: Resin transfer molding
- AFP/ATL: Automated fiber placement/tape laying

**Quality Requirements:**
- Fiber volume fraction: 55-65%
- Void content: <1% (autoclave), <3% (OoA)
- NDT: Ultrasonic C-scan, thermography
- Building-block testing: Lamina → Laminate → Element → Full-scale

**Applications:**
- Boeing 787: 50% by weight
- Airbus A350: 53% by weight
- F-35: 35% by weight

**Other Fibers:**
- Glass fiber (GFRP): Radomes, fairings
- Kevlar (Aramid): Impact protection, ballistics

### 4. Superalloys

**Nickel-Based**

**Inconel 718:**
- Service temp: 650°C
- Tensile Strength (650°C): ≥1,000 MPa
- Applications: Compressor/turbine disks, cases, bolts
- ~50% of all superalloy usage

**Inconel 625:**
- Superior corrosion resistance
- Applications: Combustors, exhaust systems
- 3D printing capability

**René Alloys:**
- René 41: 980°C service
- René N5: Single-crystal, 1,150°C
- Applications: High-pressure turbine blades

**Cobalt-Based**
- Haynes 188: 1,040°C, combustor liners
- L-605: Excellent thermal fatigue resistance
- Stellite: Wear-resistant coatings

**Requirements:**
- Directional solidification or single-crystal casting for blades
- Powder metallurgy (HIP) for disks
- Thermal barrier coatings (TBC): YSZ, 100-500 μm
- 100% fluorescent penetrant inspection

### 5. Ceramic Matrix Composites (CMC)

**SiC/SiC CMC:**
- Fiber: SiC (Hi-Nicalon, Sylramic)
- Matrix: SiC
- Interface: BN or PyC
- Service temperature: 1,316°C
- Density: 2.5 g/cm³ (1/3 of nickel superalloys)

**Manufacturing:**
- CVI (Chemical Vapor Infiltration)
- MI (Melt Infiltration)
- RMI (Reactive Melt Infiltration)

**Applications:**
- GE LEAP: High-pressure turbine shrouds (18 segments/engine)
- GE9X: HPT shrouds + HPT nozzles
- Future: Turbine blades, combustor liners

**Benefits:**
- Weight: 70% reduction vs. nickel alloys
- Cooling air: 50-75% reduction
- Fuel efficiency: +1-2%

**Other Ceramics:**
- C-C Composite: Space Shuttle nose/leading edges, brakes (~2,000°C)
- UHTC (HfC, ZrB₂): Hypersonic vehicles (3,000°C+)

### 6. Next-Generation Materials

**Nanomaterials:**
- Carbon Nanotubes (CNT): Composite reinforcement, conductivity
- Graphene: Ultra-high strength (130 GPa), thermal conductivity

**Smart Materials:**
- Shape Memory Alloys (Nitinol): Morphing structures, chevrons
- Piezoelectrics (PZT): Structural health monitoring, vibration control
- Self-healing: Microcapsule or vascular systems

**Additive Manufacturing:**
- SLM/EBM: Ti, Inconel, AlSi10Mg
- Applications: GE LEAP fuel nozzles (20→1 parts)
- Benefits: Complex geometries, weight reduction, lead time


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
