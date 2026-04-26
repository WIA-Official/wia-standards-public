# WIA-BIO-009 PHASE 2 — API Interface Specification

**Standard:** WIA-BIO-009
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

---

## 4. Lead Optimization

### 4.1 Optimization Objectives

Multi-parameter optimization (MPO) score:
```
MPO = Σ(wᵢ × Tᵢ(pᵢ))
```

Where:
- `wᵢ` = Weight for parameter i
- `Tᵢ` = Transformation function (sigmoid)
- `pᵢ` = Parameter value

### 4.2 Potency Optimization

Target progression:
```
Hit (IC50 ~ 1-10 μM) → Lead (IC50 ~ 10-100 nM) → Clinical Candidate (IC50 < 10 nM)
```

Structure-Activity Relationship (SAR):
```
ΔpIC50 = log₁₀(IC50ᵣₑ𝒻) - log₁₀(IC50ₙₑw)
```

**Significant improvement**: ΔpIC50 ≥ 0.5 (3-fold increase)

### 4.3 Physicochemical Properties

#### 4.3.1 Lipinski's Rule of Five
For oral drugs:
- Molecular Weight ≤ 500 Da
- LogP ≤ 5
- H-bond Donors ≤ 5
- H-bond Acceptors ≤ 10

#### 4.3.2 Extended Lipinski
- Polar Surface Area (PSA): 40-140 Ų
- Rotatable Bonds: ≤ 10
- Aromatic Rings: ≤ 3
- Sp³ Carbon Fraction: > 0.25

### 4.4 Selectivity Optimization

```
Selectivity Index = IC50(off-target) / IC50(on-target)
```

**Minimum**: SI ≥ 100 for most similar off-target

### 4.5 Medicinal Chemistry Strategies

#### 4.5.1 Scaffold Hopping
Replace core while maintaining:
- Key interactions (H-bonds, π-stacking)
- 3D shape similarity (Tanimoto > 0.7)
- Lipophilicity (ΔLogP < 1)

#### 4.5.2 Bioisosteric Replacement
Common bioisosteres:
- COOH ↔ Tetrazole
- Benzene ↔ Pyridine
- CH₂ ↔ O, S, NH
- C=O ↔ C=S

#### 4.5.3 Conformational Restriction
```
ΔΔG(binding) = -RT ln(f)
```

Where `f` = fraction of bioactive conformer

---

## 5. ADMET Prediction

### 5.1 Absorption

#### 5.1.1 Solubility
```
LogS = -0.01 × (MW - 200) - LogP - 0.01 × TPSA
```

**Target**: LogS > -4 (>100 μM)

#### 5.1.2 Permeability
Caco-2 model:
```
Papp = (dQ/dt) × (1 / (A × C₀))
```

Where:
- `dQ/dt` = Transport rate
- `A` = Membrane area
- `C₀` = Initial concentration

**High permeability**: Papp > 100 nm/s

### 5.2 Distribution

#### 5.2.1 Volume of Distribution
```
Vd = (Dose × F) / AUC × ke
```

**Typical range**: 0.1-10 L/kg

#### 5.2.2 Plasma Protein Binding
```
fu = [Drug]ᵤₙᵦₒᵤₙ𝒹 / [Drug]ₜₒₜₐₗ
```

Where `fu` = fraction unbound

**Target**: 0.1 < fu < 0.5 (10-50% free)

### 5.3 Metabolism

#### 5.3.1 Metabolic Stability
```
CLint = (0.693 / t½) × (Vᵢₙcᵤᵦₐₜᵢₒₙ / mg protein)
```

**Good stability**: t½ > 60 min in liver microsomes

#### 5.3.2 CYP450 Interactions
Test against main isoforms:
- CYP3A4 (50% of drugs)
- CYP2D6 (25% of drugs)
- CYP2C9, 2C19, 1A2

**Acceptable**: IC50 > 10 μM for all CYPs

### 5.4 Excretion

#### 5.4.1 Clearance
```
CL = Dose × F / AUC
```

**Target**: CL < 20 mL/min/kg

#### 5.4.2 Half-Life
```
t½ = 0.693 × Vd / CL
```

**Optimal**: 4-12 hours for QD dosing

### 5.5 Toxicity

#### 5.5.1 hERG Inhibition
```
TdP Risk = (hERG IC50 / Cmax) < 30
```

**Safe**: hERG IC50 / Cmax > 30

#### 5.5.2 Genotoxicity
Required tests:
- Ames test (bacterial mutation)
- Micronucleus assay (chromosomal damage)
- Comet assay (DNA strand breaks)

**Pass**: All negative

#### 5.5.3 Hepatotoxicity
Markers:
- ALT/AST elevation (< 2× ULN)
- Bilirubin (< 1.5× ULN)
- Alkaline phosphatase (< 1.5× ULN)

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
