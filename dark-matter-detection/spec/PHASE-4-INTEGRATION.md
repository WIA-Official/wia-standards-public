# WIA-QUA-013 PHASE 4 — Integration Specification

**Standard:** WIA-QUA-013
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 4 of 4)

---

**Polyethylene**: Neutron moderator (high H content)

**Lead**: Gamma shield
- Ancient lead: Low ²¹⁰Pb
- Thickness: 10-20 cm

**Copper**: Inner shielding
- Electroformed or OFHC
- Thickness: 5-10 cm

#### 7.4.2 Active Veto

**Muon Veto**:
- Water Cherenkov
- Plastic scintillator
- Efficiency: > 99.5%

**Neutron Veto**:
- Gd-loaded water
- ³He tubes
- Boron-loaded plastic

---

## 8. Signal Discrimination

### 8.1 Pulse Shape Discrimination

#### 8.1.1 Liquid Argon

**Scintillation Time Profile**:
```
N(t) = Nf exp(-t/τf) + Ns exp(-t/τs)
```

Where:
- τf ≈ 7 ns (singlet)
- τs ≈ 1.6 μs (triplet)

**Prompt Fraction**:
```
fprompt = Nf / (Nf + Ns)
```

**Nuclear Recoils**: fprompt ≈ 0.7
**Electron Recoils**: fprompt ≈ 0.3

**Rejection Factor**: > 10⁸

#### 8.1.2 Liquid Xenon

**S2/S1 Ratio**:
- Nuclear: 0.1-0.3
- Electron: 1.0-3.0

**Log-Likelihood**:
```
ℒNR = Σ log[P(S1, S2 | NR)]
ℒER = Σ log[P(S1, S2 | ER)]
```

**Discrimination**: ℒNR - ℒER > threshold

### 8.2 Multiple Scatter Rejection

**Single Scatter**: WIMP candidate

**Multiple Scatter**: Background (Compton, neutron)

**Position Reconstruction**: Reject events with > 1 site

**Fiducial Volume**: Inner region only

### 8.3 Machine Learning

#### 8.3.1 Neural Networks

**Input Features**:
- S1, S2 (charge, light)
- Pulse shape parameters
- Position (x, y, z)
- Timing information

**Architecture**:
- Dense layers: 3-5 layers
- Activation: ReLU, sigmoid
- Output: P(NR) vs. P(ER)

**Training**: Labeled calibration data

**Performance**: Comparable or better than traditional

#### 8.3.2 Boosted Decision Trees

**XGBoost, LightGBM**:
- Fast training
- Handle missing data
- Feature importance

**Features**:
- Energy
- S2/S1
- Pulse widths
- Asymmetry

**Hyperparameters**:
- Trees: 100-1000
- Depth: 3-10
- Learning rate: 0.01-0.1

---

## 9. Statistical Analysis

### 9.1 Likelihood Analysis

#### 9.1.1 Profile Likelihood

**Likelihood**:
```
ℒ(μ, θ) = ∏ᵢ P(nᵢ | μsᵢ(θ) + bᵢ(θ))
```

Where:
- μ = signal strength
- θ = nuisance parameters
- sᵢ = expected signal
- bᵢ = expected background

**Profile Likelihood Ratio**:
```
λ(μ) = ℒ(μ, θ̂μ) / ℒ(μ̂, θ̂)
```

**Test Statistic**:
```
qμ = -2 ln λ(μ)
```

#### 9.1.2 p-value

**Significance**:
```
p = ∫qobs^∞ f(q | H₀) dq
```

**Discovery**: p < 2.87 × 10⁻⁷ (5σ)

**Evidence**: p < 0.0013 (3σ)

### 9.2 Limits

#### 9.2.1 Frequentist (CLₛ)

**CLₛ Method**:
```
CLₛ = CLₛ₊b / CLb
```

**90% CL Upper Limit**: CLₛ(σ₉₀) = 0.10

**Advantages**:
- Conservative
- Standard in particle physics

#### 9.2.2 Bayesian

**Posterior**:
```
P(σ | data) ∝ ℒ(data | σ) × π(σ)
```

**Prior**: Flat in log(σ) or σ

**90% Credible Interval**:
```
∫₀^σ₉₀ P(σ | data) dσ = 0.90
```

### 9.3 Optimum Interval Method

**Feldman-Cousins**: Unified approach

**Confidence Belt**: Neyman construction

**Ordering**: Likelihood ratio

---

## 10. Implementation Guidelines

### 10.1 Detector Design

**Energy Threshold**: As low as possible
- Phonon: < 100 eV
- Ionization: ~keV
- Scintillation: ~keV

**Fiducial Mass**: Maximize signal

**Background Budget**: < 1 event in ROI

**Radiopurity**: Screen all materials

### 10.2 Calibration

**Electron Recoil**:
- ⁵⁷Co (122 keV)
- ¹³³Ba (356 keV)
- ⁶⁰Co (1173, 1332 keV)
- ²²⁸Th (2615 keV)

**Nuclear Recoil**:
- ²⁴¹AmBe (neutron source)
- ²⁵²Cf (fission neutrons)
- DD generator (2.45 MeV n)
- DT generator (14.1 MeV n)

**Frequency**: Daily to monthly

### 10.3 Data Quality

**Cuts**:
- Detector performance
- Data acquisition
- Environmental (temperature, etc.)

**Blinding**: Hide signal region until analysis complete

**Validation**: Cross-checks, consistency

### 10.4 Systematics

**Sources**:
- Energy scale
- Background model
- Efficiency
- Astrophysical parameters

**Treatment**: Nuisance parameters in likelihood

---

## 11. References

### 11.1 Foundational Papers

1. Jungman, G., Kamionkowski, M., & Griest, K. (1996). "Supersymmetric dark matter." *Physics Reports*, 267(5-6), 195-373.

2. Lewin, J. D., & Smith, P. F. (1996). "Review of mathematics, numerical factors, and corrections for dark matter experiments based on elastic nuclear recoil." *Astroparticle Physics*, 6(1), 87-112.

3. Goodman, M. W., & Witten, E. (1985). "Detectability of certain dark-matter candidates." *Physical Review D*, 31(12), 3059.

4. Peccei, R. D., & Quinn, H. R. (1977). "CP conservation in the presence of pseudoparticles." *Physical Review Letters*, 38(25), 1440.

### 11.2 Direct Detection

5. Aprile, E., et al. (XENON Collaboration). (2020). "Excess electronic recoil events in XENON1T." *Physical Review D*, 102(7), 072004.

6. Agnese, R., et al. (SuperCDMS Collaboration). (2018). "Low-mass dark matter search with CDMSlite." *Physical Review D*, 97(2), 022002.

7. Angloher, G., et al. (CRESST Collaboration). (2019). "Results on MeV-scale dark matter from a gram-scale cryogenic calorimeter." *European Physical Journal C*, 79(12), 1014.

### 11.3 Indirect Detection

8. Ackermann, M., et al. (Fermi-LAT Collaboration). (2015). "Searching for dark matter annihilation from Milky Way dwarf spheroidal galaxies." *Physical Review Letters*, 115(23), 231301.

9. Aartsen, M. G., et al. (IceCube Collaboration). (2016). "Search for annihilating dark matter in the Sun with 3 years of IceCube data." *European Physical Journal C*, 77(3), 146.

10. Aguilar, M., et al. (AMS Collaboration). (2019). "Towards understanding the origin of cosmic-ray positrons." *Physical Review Letters*, 122(4), 041102.

### 11.4 Colliders

11. Aad, G., et al. (ATLAS Collaboration). (2021). "Search for new phenomena in events with an energetic jet and missing transverse momentum in pp collisions." *Physical Review D*, 103(11), 112006.

### 11.5 Astrophysics


13. Planck Collaboration. (2020). "Planck 2018 results. VI. Cosmological parameters." *Astronomy & Astrophysics*, 641, A6.

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*


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
