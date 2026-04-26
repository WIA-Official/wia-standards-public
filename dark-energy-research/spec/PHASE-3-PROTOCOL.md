# WIA-QUA-014 PHASE 3 — Protocol Specification

**Standard:** WIA-QUA-014
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 3 of 4)

---

## 9. Hubble Constant Tension

### 9.1 Early Universe Measurements

**Planck CMB** (2018):
```
H₀ = 67.36 ± 0.54 km/s/Mpc
```

**Assumption**: ΛCDM model extrapolated to z=0

### 9.2 Late Universe Measurements

**SH0ES Collaboration**:
```
H₀ = 73.04 ± 1.04 km/s/Mpc
```

**Methods**: Cepheid distance ladder + Type Ia supernovae

**Other Late-Time Probes**:
- H0LiCOW (lensing): H₀ = 73.3⁺¹·⁷₋₁.₈ km/s/Mpc
- Tip of Red Giant Branch: H₀ = 69.8 ± 1.9 km/s/Mpc

### 9.3 Tension Quantification

**Discrepancy**: ~5.0σ statistical significance

```
Δχ² ≈ 25 → ~5σ tension
```

### 9.4 Proposed Solutions

#### 9.4.1 Systematic Errors
- Cepheid calibration
- SNe Ia standardization
- CMB foregrounds

#### 9.4.2 New Physics
- Early dark energy
- Interacting dark energy
- Modified gravity
- Extra relativistic species
- Decaying dark matter

#### 9.4.3 Local Void
- Underdense region affecting local measurements
- Requires large void (~300 Mpc)
- Inconsistent with CMB

### 9.5 Impact on Dark Energy

Different H₀ values affect derived dark energy properties:

**Higher H₀**:
- Lower Ω_m
- Higher Ω_Λ or modified w(z)
- Affects age of universe

---

## 10. Dark Energy Surveys

### 10.1 Dark Energy Survey (DES)

**Coverage**: 5000 deg² of southern sky

**Observables**:
- Galaxy clustering
- Weak gravitational lensing
- Galaxy clusters
- Type Ia supernovae

**Key Results** (Y3):
- S₈ = σ₈√(Ω_m/0.3) = 0.776 ± 0.017
- Consistent with ΛCDM

### 10.2 DESI (Dark Energy Spectroscopic Instrument)

**Target**: 35 million galaxy redshifts

**Redshift Range**: 0 < z < 3.5

**Goals**:
- BAO measurements at multiple redshifts
- Redshift-space distortions
- w(z) constraints to ~1%

### 10.3 Euclid Space Telescope

**Launch**: 2023

**Observables**:
- Weak lensing over 15,000 deg²
- Galaxy clustering for 50 million galaxies
- BAO and growth rate

**Expected Precision**:
- w₀ to ~2%
- w_a to ~10%

### 10.4 LSST/Vera Rubin Observatory

**Survey**: Legacy Survey of Space and Time

**Coverage**: 18,000 deg² (full southern sky)

**Depth**: 10 billion galaxies

**Probes**:
- Weak lensing
- Large-scale structure
- Supernova time-domain astronomy
- Strong lensing time delays

### 10.5 Roman Space Telescope

**Formerly**: WFIRST

**Key Programs**:
- High-redshift SNe Ia survey
- Weak lensing survey
- BAO survey

**Expected**:
- 2,700 SNe Ia at z < 1.7
- σ(w₀) ~ 0.01, σ(w_a) ~ 0.1

---

## 11. Modified Gravity Theories

### 11.1 f(R) Gravity

**Action**:
```
S = ∫ d⁴x √(-g) f(R)/(16πG) + S_matter
```

Instead of Einstein-Hilbert f(R) = R

**Examples**:
- f(R) = R + αR² (Starobinsky)
- f(R) = R - μ⁴/R (MOND-like)

**Field Equations**:
```
f'(R)Rμν - ½f(R)gμν - ∇μ∇νf'(R) + gμν□f'(R) = 8πGTμν
```

### 11.2 Scalar-Tensor Theories

**Brans-Dicke Theory**:
```
S = ∫ d⁴x √(-g) [φR - ω/φ ∂μφ∂μφ]/(16πG)
```

Where φ is scalar field, ω is coupling parameter.

**Current Constraints**: ω > 40,000 (solar system tests)

### 11.3 DGP Model (Dvali-Gabadadze-Porrati)

**5-Dimensional Braneworld**:
- 4D brane embedded in 5D bulk
- Gravity leaks into bulk at large scales

**Modified Friedmann Equation**:
```
H² = 8πGρ/3 + H/r_c
```

Where r_c is crossover scale.

**Self-acceleration**: w_eff ≈ -1/3 at late times

### 11.4 Horndeski Theory

Most general scalar-tensor theory with second-order equations of motion:

**Lagrangian**:
```
L = Σᵢ₌₂⁵ Lᵢ(φ, ∂φ, gμν, ∂g)
```

**Includes**:
- Quintessence
- K-essence
- Covariant Galileon
- f(R) gravity

### 11.5 Tests of General Relativity

#### 11.5.1 Gravitational Wave Tests

**GW170817** (neutron star merger):
- Constrains speed of gravity: |c_g - c|/c < 10⁻¹⁵
- Rules out many modified gravity models

#### 11.5.2 Growth Rate

**Growth Factor**:
```
d(δρ_m/ρ_m)/dt = f(Ω_m)H δρ_m/ρ_m
```

**GR Prediction**: f ≈ Ω_m^0.55

**Parameterization**:
```
f(Ω_m) = Ω_m^γ
```

**Measurement**: γ = 0.55 ± 0.05 (consistent with GR)

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
