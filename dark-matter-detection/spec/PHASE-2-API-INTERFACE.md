# WIA-QUA-013 PHASE 2 — API Interface Specification

**Standard:** WIA-QUA-013
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

---

#### 3.2.2 Liquid Argon

**Isotope**: ⁴⁰Ar (99.6% natural abundance)

**Scintillation**: 128 nm UV light

**Pulse Shape Discrimination**:
- Fast component: τf ≈ 7 ns
- Slow component: τs ≈ 1.6 μs
- Nuclear recoils: Higher singlet/triplet ratio

**Formula**:
```
fprompt = Sprompt / (Sprompt + Slate)
```

Nuclear recoils: fprompt ≈ 0.7
Electron recoils: fprompt ≈ 0.3

**Underground Argon**: Depleted ³⁹Ar (β: 565 keV, τ = 269 yr)

**Advantages**:
- No ⁸⁵Kr contamination
- Lower cost than xenon
- Good pulse shape discrimination

### 3.3 Cryogenic Bolometers

#### 3.3.1 Phonon Detection

**Operating Temperature**: 10-50 mK

**Heat Capacity**:
```
C(T) = βT³
```

Where β depends on material.

**Temperature Rise**:
```
ΔT = E / C(T)
```

For energy deposition E.

**Sensor Types**:
1. **NTD Germanium**: Neutron transmutation doped
2. **TES**: Transition edge sensors
3. **MMC**: Metallic magnetic calorimeters

**Energy Resolution**: <100 eV (world's best)

#### 3.3.2 Dual Readout

**Phonon + Ionization**:
```
Yield = Eionization / Ephonon
```

Nuclear recoils: Lower yield (~30%)
Electron recoils: Higher yield (~100%)

**SuperCDMS**:
- Si and Ge crystals
- Mass: ~1 kg per detector
- Threshold: ~50 eV

**CRESST**:
- CaWO₄ crystals
- Phonon + scintillation
- Light yield discrimination

### 3.4 Directional Detection

#### 3.4.1 Principle

WIMP flux direction changes due to Earth's motion:
```
v⃗(t) = v⃗sun + v⃗earth(t)
```

**Expected Direction**: Toward Cygnus constellation

**Daily Modulation**: Due to Earth's rotation

#### 3.4.2 Gas Time Projection Chambers

**Gas**: CF₄, CS₂, or Ar:SF₆

**Readout**: Charge or optical (camera)

**Track Length**:
```
L ≈ 1 mm for 10 keV F recoil in CF₄
```

**Directionality**: Head-tail discrimination

**DRIFT Experiment**:
- CS₂ gas (1 atm)
- Negative ion drift
- Multi-wire proportional chamber

**Challenges**:
- Low density → large volume needed
- Head-tail asymmetry
- Backgrounds

### 3.5 Annual Modulation

#### 3.5.1 DAMA/LIBRA

**Detector**: NaI(Tl) scintillators (250 kg)

**Signal**:
```
S(t) = S₀ + Sₘ cos(2π(t - t₀)/T)
```

**Observation**: ~9σ modulation (2-6 keV)

**Amplitude**: Sₘ/S₀ ≈ 0.03

**Phase**: t₀ ≈ June 2 (as expected)

**Controversy**: Other experiments see no signal

### 3.6 Threshold and Low-Mass WIMPs

#### 3.6.1 Sub-GeV Detection

**Challenge**: Low recoil energy

**Maximum Recoil** (mχ = 1 GeV, mₙ = 131 GeV Xe):
```
Eᴿ,max ≈ 3.5 keV
```

**Solutions**:
1. **Light Targets**: He, Si (lower A)
2. **Electron Recoils**: WIMP-electron scattering
3. **Semiconductors**: Electron-hole pairs
4. **Superconductors**: Cooper pair breaking

**SENSEI**: Skipper CCDs, ~1 eV threshold

**SuperCDMS HVeV**: High voltage, ~10 eV threshold

---

## 4. Indirect Detection Methods

### 4.1 Gamma Ray Searches

#### 4.1.1 WIMP Annihilation

**Flux**:
```
Φγ = (⟨σv⟩/8πmχ²) × ∫ ρ²(r) dV × dNγ/dE
```

Where:
- ⟨σv⟩ = velocity-averaged cross section
- ρ(r) = dark matter density profile
- dNγ/dE = photon spectrum per annihilation

**Channels**:
- χχ → bb̄ → γ (continuous)
- χχ → W⁺W⁻ → γ (continuous)
- χχ → γγ (line at Eγ = mχ)
- χχ → γZ (line at Eγ = mχ(1 - mZ²/4mχ²))

**J-factor** (astrophysical factor):
```
J = ∫ ρ²(r) dΩ dl
```

#### 4.1.2 Targets

**Galactic Center**:
- High J-factor (~10²⁴ GeV² cm⁻⁵)
- Large astrophysical backgrounds
- NFW profile: ρ(r) ∝ r⁻¹(r + rₛ)⁻²

**Dwarf Spheroidal Galaxies**:
- Lower J-factor (~10¹⁸-10¹⁹)
- Clean targets (low background)
- Examples: Segue 1, Ursa Major II

**Galaxy Clusters**:
- Moderate J-factor
- Extended sources
- Example: Fornax cluster

#### 4.1.3 Fermi-LAT

**Energy Range**: 20 MeV - 300 GeV

**Angular Resolution**: 0.6° at 1 GeV

**Effective Area**: 8000 cm² at 1 GeV

**Galactic Center Excess**:
- ~1-3 GeV photons
- Spherically symmetric
- Interpretation: WIMPs (30-40 GeV) or pulsars?

#### 4.1.4 Ground-Based Cherenkov

**HESS, VERITAS, MAGIC**:
- Energy: 100 GeV - 100 TeV
- Angular resolution: 0.1°
- Effective area: ~10⁵ m²

**Atmospheric Cherenkov**:
- γ-ray → e⁺e⁻ cascade
- Cherenkov light in atmosphere
- Timing: ~ns

### 4.2 Neutrino Searches

#### 4.2.1 Solar WIMP Capture

**Capture Rate**:
```
Cₒ = ∫ nχ(v) σχ,i v f(u) du
```

**Equilibrium**: Capture = Annihilation²/2

**Time to Equilibrium**:
```
τₑq = 1/√(CₒΓₐ)
```

For Sun: τₑq < 10¹⁰ yr (reached equilibrium)

**Annihilation Rate**:
```
Γₐ = √(CₒΓₐ) tanh²(t/τₑq)
```

#### 4.2.2 Neutrino Flux from Sun

**Energy Spectrum**:
```
dΦν/dEν ∝ Γₐ × dNν/dEν
```

**Channels**:
- χχ → bb̄ → ν (soft spectrum)
- χχ → W⁺W⁻ → ν (hard spectrum)
- χχ → τ⁺τ⁻ → ν (intermediate)

**Oscillations**: νμ/ντ easier to detect than νₑ

#### 4.2.3 IceCube

**Detector**: 1 km³ ice, 5160 PMTs

**Depth**: 1450-2450 m

**Energy Range**: GeV - PeV

**Angular Resolution**: ~1° for νμ tracks

**Signal**: Upward-going muons
```
ν + N → μ + X
```

**Background**: Atmospheric neutrinos

**Limit**: ⟨σv⟩ < 10⁻²³ cm³/s (TeV WIMPs)

### 4.3 Antimatter Searches

#### 4.3.1 Positron Excess

**AMS-02 Observation**: e⁺/(e⁺ + e⁻) increases with energy

**Energy Range**: 1-350 GeV

**Possible Explanations**:
1. WIMP annihilation (mχ ~1-3 TeV)


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
