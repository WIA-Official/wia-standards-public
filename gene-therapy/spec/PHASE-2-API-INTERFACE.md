# WIA-BIO-003 PHASE 2 — API Interface Specification

**Standard:** WIA-BIO-003
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

---

## 4. Dosage Calculations

### 4.1 AAV Dosing Formula

```
Dose (vg/kg) = (Target cells × vg per cell) / Patient weight
```

Standard calculations:

#### 4.1.1 Liver-directed Therapy
```
Dose = (2×10¹¹ hepatocytes × 10⁵ vg/cell) / 70 kg
Dose ≈ 3×10¹⁴ vg/kg
```

#### 4.1.2 Muscle-directed Therapy
```
Dose = (Target muscle mass × Cell density × vg/cell) / Weight
Dose = (500 g × 10⁹ cells/g × 10⁴ vg/cell) / 70 kg
Dose ≈ 7×10¹³ vg/kg
```

#### 4.1.3 CNS-directed Therapy
```
Dose (intrathecal) = 1×10¹³ to 2×10¹⁴ vg total
```

### 4.2 Dose Scaling

Allometric scaling from preclinical to clinical:
```
Dose_human = Dose_animal × (Weight_human / Weight_animal)^0.75
```

### 4.3 Maximum Tolerated Dose (MTD)

Safety margins:
- Preclinical MTD: Highest dose without toxicity
- Clinical starting dose: MTD / 10 (safety factor)
- Dose escalation: 3-fold increments (3+3 design)

### 4.4 Dose-Response Relationship

```
Response = R_max × (Dose^n) / (ED50^n + Dose^n)
```

Where:
- `R_max` = Maximum response
- `ED50` = Dose producing 50% response
- `n` = Hill coefficient (cooperativity)

---

## 5. Safety Assessment

### 5.1 Immunogenicity Testing

#### 5.1.1 Humoral Immunity

Pre-existing antibody testing:
```
NAb titer = 1 / (Highest dilution blocking ≥50% transduction)
```

Exclusion criteria:
- AAV: NAb titer >1:5
- Lentivirus: Not typically tested (ex vivo use)

#### 5.1.2 Cellular Immunity

T-cell response monitoring:
```
IFN-γ ELISpot: >50 spots per 10⁶ cells = positive
```

Monitor at: Days 0, 7, 14, 28, 90, 180, 365

### 5.2 Liver Toxicity

Transaminase monitoring:
```
ALT/AST elevation = (Current level / Baseline) × ULN
```

Action thresholds:
- Grade 1 (1-3× ULN): Continue, monitor weekly
- Grade 2 (3-5× ULN): Consider corticosteroids
- Grade 3 (5-20× ULN): Initiate immunosuppression
- Grade 4 (>20× ULN): Emergency intervention

### 5.3 Thrombotic Microangiopathy (TMA)

For high-dose AAV (>2×10¹⁴ vg/kg):

Monitor:
- Platelet count
- LDH levels
- Haptoglobin
- Schistocytes

TMA risk score:
```
Risk = 0.3×(Dose/10¹⁴) + 0.2×(Age/10) + 0.5×(Baseline_platelets/100)
```

Risk > 2.0: Enhanced monitoring required

### 5.4 Insertional Mutagenesis

For integrating vectors (lentivirus):
```
Integration site analysis via LAM-PCR or targeted sequencing
```

Monitor:
- Clonal expansion (>30% of population)
- Integration near oncogenes (±50 kb)
- Chromosome abnormalities

### 5.5 Biodistribution

Vector biodistribution measurement:
```
Copies/diploid genome = (2 × vg detected) / Total genomic DNA
```

Expected tissues:
- Target tissue: >1 copy/cell
- Liver: Variable (systemic administration)
- Gonads: <0.001 copy/cell (regulatory limit)

---

## 6. Gene Expression Monitoring

### 6.1 mRNA Quantification

RT-qPCR measurement:
```
Relative expression = 2^(-ΔΔCt)
```

Where:
```
ΔΔCt = (Ct_target - Ct_reference)_treated - (Ct_target - Ct_reference)_control
```

### 6.2 Protein Expression

#### 6.2.1 ELISA Quantification
```
Concentration = (Sample OD - Blank) / Standard curve slope
```

#### 6.2.2 Western Blot Densitometry
```
Relative protein = (Band intensity_target / Band intensity_loading control)
```

### 6.3 Functional Assays

For Factor VIII (hemophilia):
```
FVIII activity (%) = (Patient clotting time / Normal clotting time) × 100%
```

Therapeutic range: 40-150% of normal

### 6.4 Expression Kinetics

```
E(t) = E_max × (1 - e^(-k×t))
```

Where:
- `E(t)` = Expression at time t
- `E_max` = Maximum expression
- `k` = Expression rate constant
- `t` = Time post-administration

Typical kinetics:
- AAV: Peak at 4-8 weeks, plateau by 12 weeks
- Lentivirus: Peak at 2-4 weeks (ex vivo)

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
