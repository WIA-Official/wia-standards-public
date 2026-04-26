# WIA-BIO-003: Quality Metrics Specification

## Version 1.0 | 2025-01-15

## Table of Contents
1. [Overview](#overview)
2. [Analytical Validation Metrics](#analytical-validation-metrics)
3. [Clinical Validation Metrics](#clinical-validation-metrics)
4. [Biomarker Performance Metrics](#biomarker-performance-metrics)
5. [Quality Control](#quality-control)

## Overview

This document defines quality metrics for biomarker validation under WIA-BIO-003, covering analytical performance (LOD, LOQ, precision, accuracy) and clinical utility (sensitivity, specificity, PPV, NPV).

### Validation Levels

| Level | Purpose | Sample Size | Acceptance |
|-------|---------|-------------|------------|
| **Analytical** | Technical performance | n=20-50 per parameter | CV <20%, R² >0.95 |
| **Clinical** | Diagnostic accuracy | n=200-500+ | AUC >0.8, Sens/Spec >80% |
| **Clinical Utility** | Impact on outcomes | n=500-1000+ | Improved patient management |

## Analytical Validation Metrics

### 1. Limit of Detection (LOD)

**Formula:**
```
LOD = meanblank + 1.645 × SDblank (95% confidence)
or
LOD = meanblank + 3 × SDblank (99% confidence)
```

**Protocol:**
- Analyze 20 blank samples
- Analyze 20 low-positive samples near expected LOD
- Calculate mean and SD of blank
- Verify 95% of low samples > LOD

**Acceptance:** 95% detection rate at declared LOD

### 2. Limit of Quantification (LOQ)

**Definition:** Lowest concentration with acceptable precision (CV <20%)

**Protocol:**
- Prepare 5-7 concentration levels spanning expected range
- Analyze each level in replicates (n=10)
- Calculate CV at each level
- LOQ = lowest concentration with CV <20%

**Clinical Biomarkers:** Typically CV <15% at LOQ

### 3. Precision

**Within-Run (Repeatability):**
```
CV = (SD / Mean) × 100%

Target: CV <15% (clinical), <20% (research)
```

**Between-Run (Intermediate Precision):**
- Different days, operators, reagent lots
- Target: CV <20%

**Total Precision:**
```
CV_total = √(CV²_within + CV²_between)
```

### 4. Accuracy (Recovery)

**Spiking Study:**
```
% Recovery = (Measured - Unspiked) / Spiked × 100%

Acceptance: 80-120% recovery
```

**Reference Material:**
- Use certified reference materials (CRM) if available
- Target: ±10% of reference value

### 5. Linearity

**Protocol:**
- Prepare 5-7 concentration levels
- Analyze in duplicate
- Linear regression

**Acceptance:**
- R² >0.95 (clinical)
- R² >0.90 (research)
- Residuals randomly distributed

## Clinical Validation Metrics

### Diagnostic Performance

| Metric | Formula | Target | Interpretation |
|--------|---------|--------|----------------|
| **Sensitivity** | TP / (TP + FN) | >80% | Ability to detect disease |
| **Specificity** | TN / (TN + FP) | >90% | Ability to exclude disease |
| **PPV** | TP / (TP + FP) | Depends on prevalence | Probability disease present if +|
| **NPV** | TN / (TN + FN) | >95% for rule-out | Probability disease absent if - |
| **Accuracy** | (TP + TN) / Total | >85% | Overall correctness |
| **AUC-ROC** | Area under curve | >0.8 excellent | Discriminatory ability |

### Sample Size Calculation

```python
# For sensitivity/specificity estimation
n = (Z²_α × p × (1-p)) / d²

where:
Z_α = 1.96 (95% CI)
p = expected sensitivity/specificity
d = desired precision (e.g., ±0.05)

Example: Expected sensitivity 90%, precision ±5%
n = (1.96² × 0.9 × 0.1) / 0.05² = 138 diseased patients
```

### Clinical Utility Metrics

**Decision Curve Analysis:**
```
Net Benefit = (TP / n) - (FP / n) × (pt / (1-pt))

where pt = decision threshold probability
```

**Number Needed to Test (NNT):**
```
NNT = 1 / |Outcome_biomarker - Outcome_standard|
```

## Biomarker Performance Metrics

### Multi-Omics Biomarkers

**Integration Score:**
```
Score = Σ(wi × xi)

where wi = weight for variable i, xi = normalized value
```

**Cross-Validation:**
- k-fold CV (typically k=5 or 10)
- Leave-one-out CV (LOOCV) for small datasets
- Independent validation cohort (gold standard)

### Stability Metrics

| Test | Condition | Acceptance |
|------|-----------|------------|
| **Freeze-Thaw** | 3 cycles | <20% change |
| **Bench-top** | 4h at RT | <15% change |
| **Processed Sample** | 24h at 4°C | <15% change |
| **Long-term Storage** | -80°C for 6mo | <20% change |

## Quality Control

### Internal QC

**Control Levels:**
- Low: Near LOQ
- Medium: Mid-range
- High: Near upper limit

**Acceptance Rules (Westgard):**
- 1-2s: Warning
- 1-3s: Out of control (reject run)
- 2-2s: Two consecutive controls outside 2SD
- R-4s: Range between controls >4SD
- 4-1s: Four consecutive controls outside 1SD

### External QC (Proficiency Testing)

**Frequency:** Quarterly or biannual

**Acceptance:** Within ±2SD of peer group mean or ±20% of target value

---

**Document Version:** 1.0
**Last Updated:** 2025-01-15
**Maintained by:** WIA BIO Working Group
**License:** Apache 2.0


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
