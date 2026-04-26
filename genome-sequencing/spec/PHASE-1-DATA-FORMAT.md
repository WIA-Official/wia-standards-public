# WIA-BIO-002 PHASE 1 — Data Format Specification

**Standard:** WIA-BIO-002
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 1 of 4)

---

# WIA-BIO-002: Genome Sequencing Specification v1.0

> **Standard ID:** WIA-BIO-002
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Bioinformatics Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Sequencing Technologies](#2-sequencing-technologies)
3. [Quality Metrics](#3-quality-metrics)
4. [Data Formats](#4-data-formats)
5. [Variant Calling](#5-variant-calling)
6. [Genome Annotation](#6-genome-annotation)
7. [Implementation Guidelines](#7-implementation-guidelines)
8. [Safety & Ethics Protocols](#8-safety--ethics-protocols)
9. [Quality Control](#9-quality-control)
10. [References](#10-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the standardized framework for genome sequencing, analysis, and interpretation across multiple sequencing platforms and applications.

### 1.2 Scope

The standard covers:
- Next-generation sequencing (NGS) technologies
- Quality control and metrics
- Variant calling algorithms
- Genome annotation standards
- Clinical interpretation guidelines
- Data privacy and ethical considerations

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to democratize genomic medicine by providing accessible, standardized tools that benefit all of humanity while protecting individual privacy and maintaining ethical standards.

### 1.4 Terminology

- **Read**: A sequence obtained from a single sequencing reaction
- **Coverage/Depth**: Number of reads covering a genomic position
- **Phred Score**: Quality score for base calling accuracy
- **Variant**: A difference from the reference genome
- **VAF**: Variant Allele Frequency
- **SNP**: Single Nucleotide Polymorphism
- **Indel**: Insertion or Deletion
- **CNV**: Copy Number Variant
- **SV**: Structural Variant

---

## 2. Sequencing Technologies

### 2.1 Illumina Next-Generation Sequencing

#### 2.1.1 Technology Overview

Sequencing-by-synthesis using reversible terminators:

```
Chemistry: Bridge amplification → Cluster generation → Sequencing-by-synthesis
Read Length: 50-300 bp (paired-end)
Accuracy: >99.9% (Q30+)
Throughput: 50 Mb - 6 Tb per run
```

#### 2.1.2 Applications

- Whole Genome Sequencing (WGS)
- Whole Exome Sequencing (WES)
- RNA Sequencing (RNA-seq)
- Targeted Panel Sequencing
- Metagenomics

#### 2.1.3 Quality Parameters

```
Cluster Density: 180-220 K/mm²
Cluster Passing Filter: >80%
Q30 Score: >85% of bases
Error Rate: <1%
GC Bias: <5%
```

### 2.2 PacBio HiFi Sequencing

#### 2.2.1 Technology Overview

Single-Molecule Real-Time (SMRT) sequencing with circular consensus:

```
Chemistry: Zero-mode waveguides → Real-time detection
Read Length: 10-25 kb (HiFi)
Accuracy: >99.9% (after CCS)
Throughput: 10-30 Gb per SMRT Cell
```

#### 2.2.2 Applications

- De novo genome assembly
- Structural variant detection
- Full-length transcript sequencing
- Repeat region sequencing
- Haplotype phasing

### 2.3 Oxford Nanopore Sequencing

#### 2.3.1 Technology Overview

Nanopore-based sequencing of native DNA/RNA:

```
Chemistry: Protein nanopore → Ionic current detection
Read Length: 10 kb - 2 Mb+
Accuracy: 95-99% (base calling)
Throughput: 10-50 Gb per flow cell
Real-time: Yes
```

#### 2.3.2 Applications

- Ultra-long reads (>100 kb)
- Direct RNA sequencing
- Epigenetic modification detection
- Real-time pathogen identification
- Portable sequencing

### 2.4 Single-Cell Sequencing

#### 2.4.1 Technology Overview

```
Platform: 10x Genomics, Drop-seq, Smart-seq
Cell Throughput: 100 - 10,000+ cells
Read Depth: 50-100K reads per cell
Applications: Cell-type identification, rare cell discovery
```

---

## 3. Quality Metrics

### 3.1 Phred Quality Score

#### 3.1.1 Definition

The Phred quality score quantifies base-calling accuracy:

```
Q = -10 × log₁₀(P)
```

Where:
- `Q` = Phred quality score
- `P` = Probability of incorrect base call

#### 3.1.2 Quality Score Interpretation

```
Q10 = 90% accuracy (1 error per 10 bases)
Q20 = 99% accuracy (1 error per 100 bases)
Q30 = 99.9% accuracy (1 error per 1,000 bases)
Q40 = 99.99% accuracy (1 error per 10,000 bases)
```

#### 3.1.3 Clinical Standards

**Minimum Requirements**:
- Clinical WGS: ≥90% bases at Q30
- Research WGS: ≥85% bases at Q30
- RNA-seq: ≥80% bases at Q30

### 3.2 Sequencing Coverage

#### 3.2.1 Coverage Calculation

```
Coverage = (N × L) / G
```

Where:
- `N` = Total number of reads
- `L` = Read length (bp)
- `G` = Genome size (bp)

Example for human genome:
```
Given:
- Reads: 100,000,000
- Read length: 150 bp
- Genome size: 3,000,000,000 bp

Coverage = (100,000,000 × 150) / 3,000,000,000
Coverage = 5×
```

#### 3.2.2 Recommended Coverage

| Application | Minimum | Clinical | Research |
|-------------|---------|----------|----------|
| WGS (Germline) | 30× | 40-60× | 20-30× |
| WGS (Tumor) | 60× | 80-100× | 40-60× |
| WES | 50× | 100× | 30-50× |
| RNA-seq | 20M reads | 30M reads | 10M reads |
| ChIP-seq | 20M reads | 40M reads | 10M reads |

### 3.3 Coverage Uniformity

#### 3.3.1 Uniformity Metric

```
Uniformity = (Bases at ≥0.2×mean) / Total_bases
```

Clinical standard: **≥90% uniformity**

#### 3.3.2 Coverage Distribution

```
Coefficient of Variation (CV) = σ / μ
```

Where:
- `σ` = Standard deviation of coverage
- `μ` = Mean coverage

Target: **CV < 0.3** for clinical sequencing

### 3.4 Mapping Quality

#### 3.4.1 Mapping Quality Score

```
MAPQ = -10 × log₁₀(P_error)
```

Where `P_error` is the probability of incorrect mapping.


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

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-1-DATA-FORMAT validation when the
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
