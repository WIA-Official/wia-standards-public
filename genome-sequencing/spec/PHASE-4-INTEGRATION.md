# WIA-BIO-002 PHASE 4 — Integration Specification

**Standard:** WIA-BIO-002
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 4 of 4)

---

## 8. Safety & Ethics Protocols

### 8.1 Informed Consent

#### 8.1.1 Consent Requirements

```
Required Elements:
1. Purpose of sequencing
2. Potential findings (primary and incidental)
3. Data storage and retention
4. Data sharing and access
5. Right to withdraw
6. Genetic counseling availability
```

#### 8.1.2 Incidental Findings

**ACMG Recommended Genes**: 73 genes with actionable findings
```
Categories:
- Cancer susceptibility (BRCA1, BRCA2, TP53, etc.)
- Cardiac conditions (MYH7, KCNQ1, etc.)
- Metabolic disorders (LDLR, APOB, etc.)
```

**Return Policy**:
```
Option 1: Return all findings
Option 2: Return only actionable findings
Option 3: Do not return incidental findings
```

### 8.2 Data Privacy

#### 8.2.1 HIPAA Compliance

```
Protected Health Information (PHI):
- Remove 18 HIPAA identifiers
- Use hashed patient IDs
- Encrypt data at rest and in transit
- Maintain audit logs
```

#### 8.2.2 GDPR Compliance

```
Article 9: Special Category Data
- Genetic data requires explicit consent
- Purpose limitation
- Data minimization
- Right to erasure (Article 17)
```

#### 8.2.3 Encryption Standards

```
At Rest: AES-256
In Transit: TLS 1.3
Key Management: HSM or KMS
Backup: Encrypted with separate keys
```

### 8.3 Data Retention

#### 8.3.1 Retention Policies

```
Raw Data (FASTQ): 1-2 years (research), 5+ years (clinical)
Aligned Data (BAM): 5-10 years
Variants (VCF): Permanent
Reports: Permanent
Consent Forms: Permanent
```

#### 8.3.2 Secure Deletion

```
Method: NIST SP 800-88 Guidelines
- Cryptographic erasure (preferred)
- Physical destruction (for hardware)
- Verification of deletion
```

---

## 9. Quality Control

### 9.1 Pre-Sequencing QC

#### 9.1.1 DNA Quality Assessment

```
Metrics:
- Concentration: 10-100 ng/µL (Qubit)
- Purity: A260/A280 = 1.8-2.0
- Integrity: DIN ≥ 7.0 (TapeStation/Bioanalyzer)
- Fragment size: 150-500 bp (library)
```

#### 9.1.2 Library QC

```
- Library concentration: 2-10 nM
- Fragment size distribution: Tight peak
- Adapter dimer: <5%
- qPCR validation: CT 15-25
```

### 9.2 Sequencing QC

#### 9.2.1 Real-Time Metrics

```
- Cluster density: 180-220 K/mm² (Illumina)
- Clusters passing filter: >80%
- Phasing/prephasing: <0.2
- Error rate: <2%
```

#### 9.2.2 Run Termination Criteria

```
Stop run if:
- Cluster PF% < 70%
- Error rate > 5%
- Q30 < 70%
- Significant tile failures
```

### 9.3 Post-Sequencing QC

#### 9.3.1 FastQC Metrics

```
✓ Per base sequence quality
✓ Per sequence quality scores
✓ Per base N content
✓ Sequence length distribution
✓ Duplicate sequences
✓ Overrepresented sequences
✓ Adapter content
```

#### 9.3.2 Alignment QC

```
- Mapping rate: ≥95%
- Properly paired: ≥90%
- Insert size: Mean ± 2σ
- Chimeric reads: <2%
```

### 9.4 Variant QC

#### 9.4.1 Variant-Level Filters

```
- Depth: ≥10× (germline), ≥20× (somatic)
- Quality score: ≥30
- Strand bias: p-value > 0.001
- Allele balance: 0.2-0.8 (heterozygous)
- VAF: ≥5% (somatic)
```

#### 9.4.2 Sample-Level QC

```
- Ti/Tv ratio: 2.0-2.1 (WGS), 3.0-3.3 (WES)
- Het/Hom ratio: 1.5-2.0
- Novel variants: <1%
- Contamination: <1%
```

---

## 10. References

### 10.1 Scientific Papers

1. Illumina (2017). "An Introduction to Next-Generation Sequencing Technology"
2. Van 관련 분야 자료. "Ten years of next-generation sequencing technology." Trends in Genetics.

### 10.2 Reference Genomes

| Genome | Build | Release | Size |
|--------|-------|---------|------|
| Human | GRCh38/hg38 | 2013 | 3.1 Gb |
| Human | GRCh37/hg19 | 2009 | 3.1 Gb |
| Mouse | GRCm39 | 2020 | 2.7 Gb |
| Arabidopsis | TAIR10 | 2010 | 135 Mb |

### 10.3 WIA Standards

- WIA-INTENT: Intent-based genomic queries
- WIA-OMNI-API: Universal bioinformatics API
- WIA-PRIVACY: Encrypted health data storage
- WIA-SOCIAL: Research collaboration platform

---

## Appendix A: Example Calculations

### A.1 Coverage Calculation

```
Given:
- Target coverage: 30×
- Genome size: 3 Gb (human)
- Read length: 150 bp (paired-end)

Total bases needed = 30 × 3,000,000,000 = 90 Gb
Effective read length = 2 × 150 = 300 bp
Reads required = 90,000,000,000 / 300 = 300,000,000 reads
Read pairs = 150,000,000 pairs

Sequencing: NovaSeq 6000 S4 (3 Tb capacity)
Samples per run: 3,000,000,000,000 / 90,000,000,000 = 33 samples
```

### A.2 VAF Calculation

```
Given:
- Reference reads: 45
- Alternative reads: 55
- Total depth: 100

VAF = 55 / (45 + 55) = 0.55 or 55%

Interpretation:
- Expected heterozygous VAF ≈ 50%
- Observed VAF = 55% (within normal range)
- Conclusion: Likely germline heterozygous variant
```

### A.3 Quality Score Conversion

```
ASCII: 'I' (uppercase i)
ASCII value: 73
Phred score: 73 - 33 = 40
Accuracy: 10^(-40/10) = 0.0001 or 99.99%
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-BIO-002 Specification v1.0*
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
