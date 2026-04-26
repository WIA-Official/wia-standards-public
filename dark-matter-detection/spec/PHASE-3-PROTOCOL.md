# WIA-QUA-013 PHASE 3 — Protocol Specification

**Standard:** WIA-QUA-013
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 3 of 4)

---

2. Pulsars (confirmed nearby pulsars)
3. Astrophysical sources

**Challenge**: Propagation uncertainties

#### 4.3.2 Antiproton Flux

**AMS-02**: Antiproton-to-proton ratio

**Energy**: 1-450 GeV

**Excess?**: Some tension at ~10-20 GeV

**Secondary Production**:
```
p + ISM → p̄ + X
```

**WIMP Signature**:
```
χχ → qq̄ → p̄
```

---

## 5. Collider Searches

### 5.1 LHC WIMP Production

#### 5.1.1 Missing Transverse Energy

**Signature**:
```
pp → χχ + X
```

WIMPs escape → missing energy

**Channels**:
1. **Monojet**: pp → χχ + j
2. **Mono-photon**: pp → χχ + γ
3. **Mono-Z**: pp → χχ + Z
4. **Mono-Higgs**: pp → χχ + h

**Missing ET**:
```
E̸T = -∑ pT,visible
```

#### 5.1.2 Effective Field Theory

**Contact Interaction**:
```
ℒeff = (1/Λ²) × (q̄Γq)(χ̄Γ'χ)
```

Where Λ is the mediator mass scale.

**Validity**: Λ >> √s (low-energy limit)

**Simplified Models**: Include explicit mediator

#### 5.1.3 ATLAS and CMS Results

**Luminosity**: 139 fb⁻¹ (Run 2)

**Limits**: Λ > 1-3 TeV (depending on operator)

**Monojet**: Most stringent for spin-independent

**Translation**: Collider → direct detection cross section

### 5.2 Supersymmetry Searches

#### 5.2.1 Neutralino Production

**Processes**:
- Squark pair: pp → q̃q̃ → qq + χχ
- Gluino pair: pp → g̃g̃ → qq̄qq̄ + χχ
- Chargino/neutralino: pp → χ̃⁺χ̃⁻ → W⁺W⁻ + χχ

**Signatures**:
- Jets + MET
- Leptons + MET
- Same-sign dileptons

**Exclusions**:
- Squarks: > 2 TeV
- Gluinos: > 2.3 TeV
- Winos: > 650 GeV

---

## 6. Astrophysical Observations

### 6.1 Gravitational Lensing

#### 6.1.1 Strong Lensing

**Einstein Radius**:
```
θE = √(4GM/c² × Dₗₛ/(DₗDₛ))
```

Where:
- M = lens mass
- Dₗ, Dₛ, Dₗₛ = angular diameter distances

**Arc Formation**: Background galaxy distorted into arc

**Multiple Images**: Typically 2-4 images

**Mass Reconstruction**: Invert lens equation

#### 6.1.2 Weak Lensing

**Shear**:
```
γ = (a - b)/(a + b) × e^(2iφ)
```

Where a, b are semi-major/minor axes.

**Convergence**:
```
κ = Σ / Σcrit
```

Where Σ is surface mass density.

**Power Spectrum**: P(ℓ) measures matter distribution

#### 6.1.3 Cluster Mass Maps

**Abell 1689**: Total mass ~2 × 10¹⁵ M☉

**Mass-to-Light Ratio**: M/L ≈ 300 M☉/L☉

**Dark Matter Fraction**: ~85%

**Bullet Cluster**: Offset between DM (lensing) and gas (X-ray)

### 6.2 Rotation Curves

#### 6.2.1 Galaxy Rotation

**Expected** (visible matter only):
```
v(r) ∝ r⁻¹/² for r > R
```

**Observed**: v(r) ≈ constant (flat)

**NFW Profile**:
```
ρ(r) = ρₛ / [(r/rₛ)(1 + r/rₛ)²]
```

**Circular Velocity**:
```
v²(r) = GM(r)/r
```

Where M(r) = ∫ 4πr²ρ(r) dr

#### 6.2.2 Halo Mass

**Milky Way**: Mhalo ≈ 10¹² M☉

**Virial Radius**: Rvir ≈ 200 kpc

**Concentration**: c = Rvir/rs ≈ 10-20

### 6.3 Cosmic Microwave Background

#### 6.3.1 Dark Matter Density

**Planck Results**:
```
Ωch² = 0.1200 ± 0.0012
```

**Dark Matter Fraction**:
```
Ωc ≈ 27% of total energy density
```

**Baryon-to-DM Ratio**: Ωb/Ωc ≈ 0.18

#### 6.3.2 Structure Formation

**Matter Power Spectrum**: P(k)

**Transfer Function**: Suppression below horizon at matter-radiation equality

**CDM vs. WDM**: Free-streaming scale

---

## 7. Background Reduction

### 7.1 Radioactive Backgrounds

#### 7.1.1 Material Selection

**Screening**:
- HPGe detectors for γ spectroscopy
- ICP-MS for trace elements
- NAA for activation analysis

**Radiopurity Goals**:
- U/Th: < 1 ppb
- ⁴⁰K: < 1 ppt
- ⁶⁰Co: < μBq/kg
- ²²⁶Ra: < μBq/kg

**Clean Materials**:
- Electroformed copper
- Ancient lead (Roman, pre-industrial)
- High-purity germanium
- Radiopure plastics

#### 7.1.2 Radon Control

**²²²Rn** (τ = 3.8 days):
- Emanates from materials
- Plates out on surfaces
- Progeny: ²¹⁴Pb, ²¹⁴Bi (β/γ emitters)

**Mitigation**:
- Nitrogen atmosphere
- Radon-free cleanroom
- Active radon removal
- Charcoal traps

### 7.2 Cosmogenic Activation

#### 7.2.1 Neutron-Induced

**Surface Exposure**:
- Cosmic ray spallation
- Thermal neutrons

**Activated Isotopes** (Ge):
- ⁶⁸Ge (τ = 271 d) → ⁶⁸Ga → ⁶⁸Zn
- ⁶⁰Co (τ = 5.27 yr)
- ³H (τ = 12.3 yr)

**Xenon**:
- ¹²⁷Xe (τ = 36.4 d)
- ¹²⁵I from ¹²⁶Xe (τ = 13.1 d)

**Mitigation**:
- Minimize surface time
- Shield during transport
- Underground storage
- Decay time

### 7.3 Depth Requirements

#### 7.3.1 Muon Flux Reduction

**Surface Muon Flux**: ~1 cm⁻²min⁻¹

**Vertical Intensity**:
```
I(h) = I₀ × exp(-h/Λ)
```

Where Λ ≈ 1500 m.w.e.

**Muon-Induced Neutrons**:
```
Yn ≈ 10⁻³ neutrons per muon
```

**Depth Requirements**:
- Ton-scale: > 3000 m.w.e.
- 10-ton: > 4000 m.w.e.
- 100-ton: > 5000 m.w.e.

### 7.4 Shielding

#### 7.4.1 Passive Shielding

**Water Tank**: Neutron moderator, muon veto


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
