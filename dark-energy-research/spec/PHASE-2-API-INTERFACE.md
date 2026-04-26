# WIA-QUA-014 PHASE 2 — API Interface Specification

**Standard:** WIA-QUA-014
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

---

For ΛCDM model:
```
z_acc = (2Ω_Λ/Ω_m)^(1/3) - 1 ≈ 0.67
```

**Cosmic Time**: t_acc ≈ 7.5 billion years after Big Bang

### 4.4 Scale Factor Evolution

For matter + dark energy:

```
a(t) ∝ sinh^(2/3)(3H₀√Ω_Λ t/2)
```

**Asymptotic Behavior**:
- Early times: a(t) ∝ t^(2/3) (matter-dominated)
- Late times: a(t) ∝ exp(H₀√Ω_Λ t) (dark energy-dominated)

---

## 5. Equation of State Parameter

### 5.1 Definition

```
w(z) = P(z)/ρ(z)
```

**Standard Values**:
- Radiation: w = 1/3
- Matter: w = 0
- Cosmological constant: w = -1
- Quintessence: -1 < w < -1/3
- Phantom energy: w < -1

### 5.2 Parameterizations

#### 5.2.1 Constant w
```
w(z) = w₀
```

Simplest extension of ΛCDM.

#### 5.2.2 CPL Parameterization (Chevallier-Polarski-Linder)
```
w(z) = w₀ + w_a z/(1+z)
```

**Parameters**:
- w₀ = present-day equation of state
- w_a = evolution parameter

**Current Constraints**:
- w₀ = -1.03 ± 0.03 (Planck + BAO + Pantheon)
- w_a = -0.03 ± 0.3

#### 5.2.3 Linear Parameterization
```
w(a) = w₀ + w_a(1-a)
```

Where a = 1/(1+z) is scale factor.

### 5.3 Energy Density Evolution

```
ρ_DE(z) = ρ_DE,0 exp[3∫₀^z (1+w(z'))dz'/(1+z')]
```

For constant w:
```
ρ_DE(z) = ρ_DE,0 (1+z)^(3(1+w))
```

---

## 6. Type Ia Supernovae Observations

### 6.1 Standard Candles

Type Ia supernovae (SNe Ia) are standardizable candles:

**Absolute Magnitude**: M ≈ -19.3 (after corrections)

**Phillips Relationship**:
```
M = M₀ + α×(Δm₁₅ - 1.1)
```

Where Δm₁₅ = magnitude decline in 15 days after peak.

### 6.2 Distance Modulus

```
μ = m - M = 5 log₁₀(d_L/10 pc)
```

Where:
- m = apparent magnitude
- M = absolute magnitude
- d_L = luminosity distance

### 6.3 Luminosity Distance

```
d_L(z) = (1+z) ∫₀^z dz'/H(z')
```

For flat ΛCDM:
```
d_L(z) = (c/H₀)(1+z) ∫₀^z dz'/√[Ω_m(1+z')³ + Ω_Λ]
```

### 6.4 Hubble Diagram

Plot of distance modulus vs redshift:

**Linear Regime** (z << 1):
```
μ ≈ 5 log₁₀(cz/H₀) + 25 + (1-q₀)z + ...
```

**Acceleration Discovery** (late-1990s high-z supernova surveys):
- High-z supernovae dimmer than expected
- Implies accelerating expansion
- Nobel Prize in Physics 2011 (awarded for the discovery of accelerating expansion)

### 6.5 Pantheon+ Sample

Latest SNe Ia compilation:
- 1,701 light curves
- 1,550 supernovae
- Redshift range: 0.001 < z < 2.26
- Systematic uncertainties ~0.01 mag

---

## 7. Baryon Acoustic Oscillations

### 7.1 Sound Horizon

Comoving distance sound waves traveled before recombination:

```
r_s = ∫₀^(t_rec) c_s dt/a(t)
```

Where c_s = sound speed in baryon-photon plasma.

**Standard Ruler**: r_s ≈ 147 Mpc

### 7.2 Angular Diameter Distance

```
d_A(z) = ∫₀^z dz'/H(z')/(1+z)
```

**BAO Peak in Galaxy Correlation**:
```
θ_BAO(z) = r_s/d_A(z)
```

### 7.3 Spherically Averaged Distance

```
D_V(z) = [(1+z)²d_A²(z) cz/H(z)]^(1/3)
```

**Measurement**:
```
D_V(z)/r_s
```

### 7.4 Observational Data

**SDSS-III/BOSS**:
- z = 0.32: D_V/r_s = 8.25 ± 0.15
- z = 0.57: D_V/r_s = 13.77 ± 0.13

**eBOSS**:
- z = 0.698: D_M/r_s = 17.65 ± 0.30
- z = 1.48: D_M/r_s = 30.21 ± 0.79

### 7.5 Alcock-Paczynski Test

Anisotropic BAO measurements constrain:
- H(z) from line-of-sight clustering
- d_A(z) from transverse clustering

**Ratio**:
```
F_AP(z) = (1+z)d_A(z)H(z)/c
```

---

## 8. CMB Measurements

### 8.1 Cosmic Microwave Background

**Temperature**: T_CMB = 2.7255 K

**Blackbody Spectrum**: Perfect to 1 part in 10⁵

### 8.2 Angular Power Spectrum

Temperature fluctuations expanded in spherical harmonics:

```
ΔT(θ,φ)/T = Σ a_lm Y_lm(θ,φ)
```

**Power Spectrum**:
```
C_l = ⟨|a_lm|²⟩
```

### 8.3 Acoustic Peaks

**First Peak** (l ≈ 220):
- Determines spatial curvature
- Constrains Ω_total ≈ 1.000 ± 0.001

**Peak Spacing**:
- Determines baryon density Ω_b h²
- Determines matter density Ω_m h²

**ISW Effect** (Integrated Sachs-Wolfe):
- Large-scale power (l < 30)
- Sensitive to dark energy
- Enhanced by accelerating expansion

### 8.4 Sound Horizon at Recombination

```
r_s(z_*) = ∫_(z_*)^∞ c_s dz/H(z)
```

Where z_* ≈ 1090 (recombination redshift)

**Planck 2018**: r_s = 144.43 ± 0.26 Mpc

### 8.5 Angular Diameter Distance to Last Scattering

```
θ_* = r_s(z_*)/d_A(z_*)
```

**Constraint**:
```
θ_* = (1.04110 ± 0.00031)°
```

### 8.6 Planck Constraints

From Planck 2018 (TT,TE,EE+lowE):
- Ω_Λ = 0.6889 ± 0.0056
- Ω_m = 0.3111 ± 0.0056
- H₀ = 67.36 ± 0.54 km/s/Mpc
- σ₈ = 0.8111 ± 0.0060

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
