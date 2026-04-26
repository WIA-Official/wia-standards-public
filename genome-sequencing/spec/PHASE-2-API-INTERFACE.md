# WIA-BIO-002 PHASE 2 — API Interface Specification

**Standard:** WIA-BIO-002
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

---

#### 3.4.2 Quality Thresholds

```
MAPQ ≥ 20: 99% mapping confidence (standard threshold)
MAPQ ≥ 30: 99.9% mapping confidence (high-quality)
MAPQ ≥ 60: Unique mapping (preferred)
```

### 3.5 Duplication Rate

#### 3.5.1 Calculation

```
Duplication_Rate = Duplicate_Reads / Total_Reads
```

#### 3.5.2 Acceptable Ranges

- Clinical WGS: <20%
- Research WGS: <30%
- RNA-seq: <50% (expected due to expression levels)
- Amplicon sequencing: <70%

---

## 4. Data Formats

### 4.1 FASTQ Format

#### 4.1.1 Structure

```
@SEQ_ID
GATTTGGGGTTCAAAGCAGTATCGATCAAATAGTAAATCCATTTGTTCAACTCACAGTTT
+
!''*((((***+))%%%++)(%%%%).1***-+*''))**55CCF>>>>>>CCCCCCC65
```

Lines:
1. Identifier (starts with @)
2. Sequence
3. Separator (starts with +)
4. Quality scores (ASCII-encoded Phred+33)

#### 4.1.2 Quality Encoding

```
Quality = ASCII_value - 33
```

Example: `!` = 33 - 33 = Q0, `I` = 73 - 33 = Q40

### 4.2 SAM/BAM/CRAM Format

#### 4.2.1 SAM Format

Tab-delimited alignment format:

```
QNAME  FLAG  RNAME  POS  MAPQ  CIGAR  RNEXT  PNEXT  TLEN  SEQ  QUAL  [TAGS]
```

#### 4.2.2 BAM Format

Binary, compressed version of SAM:
- Faster processing
- Smaller file size (~1/3 of SAM)
- Requires indexing (.bai)

#### 4.2.3 CRAM Format

Reference-based compression:
- ~50% smaller than BAM
- Requires reference genome
- Lossless or lossy compression options

### 4.3 VCF/BCF Format

#### 4.3.1 VCF Structure

```
##fileformat=VCFv4.2
##reference=hg38
#CHROM  POS     ID      REF  ALT  QUAL  FILTER  INFO    FORMAT  SAMPLE
chr1    123456  rs789   A    G    100   PASS    DP=50   GT:GQ   0/1:99
```

#### 4.3.2 Variant Types

- SNP: Single nucleotide change
- Indel: Insertion or deletion <50 bp
- SV: Structural variant >50 bp
- CNV: Copy number variant

### 4.4 BED Format

#### 4.4.1 Structure

```
chrom  chromStart  chromEnd  [name]  [score]  [strand]
chr1   1000        2000      region1 100      +
```

Applications:
- Target regions
- Excluded regions
- Genomic features
- Coverage tracks

---

## 5. Variant Calling

### 5.1 Variant Calling Pipeline

#### 5.1.1 Standard Workflow

```
1. Raw Reads (FASTQ)
   ↓
2. Quality Control (FastQC)
   ↓
3. Alignment (BWA-MEM, Bowtie2)
   ↓
4. Post-alignment Processing
   - Sort (samtools sort)
   - Mark duplicates (Picard)
   - Base quality recalibration (GATK BQSR)
   ↓
5. Variant Calling
   - Germline: GATK HaplotypeCaller, FreeBayes
   - Somatic: Mutect2, Strelka2, VarScan2
   ↓
6. Variant Filtration (GATK VQSR, hard filters)
   ↓
7. Annotation (VEP, ANNOVAR, SnpEff)
   ↓
8. Interpretation & Reporting
```

### 5.2 Germline Variant Calling

#### 5.2.1 GATK Best Practices

```
Algorithm: HaplotypeCaller (local de novo assembly)

Parameters:
- Min base quality: 20
- Min mapping quality: 20
- Min coverage: 10×
- Min GQ: 20
```

#### 5.2.2 Quality Filters

```
SNPs:
- QD < 2.0 (QualByDepth)
- FS > 60.0 (FisherStrand)
- MQ < 40.0 (MappingQuality)
- SOR > 3.0 (StrandOddsRatio)

Indels:
- QD < 2.0
- FS > 200.0
- ReadPosRankSum < -20.0
```

### 5.3 Somatic Variant Calling

#### 5.3.1 Tumor-Normal Paired Analysis

```
Algorithm: Mutect2 (Bayesian classifier)

Requirements:
- Matched normal sample
- Tumor purity ≥20%
- Tumor coverage ≥60×
- Normal coverage ≥30×
```

#### 5.3.2 Variant Allele Frequency

```
VAF = Alt / (Ref + Alt)
```

For diploid genome with heterozygous variant:
```
Expected VAF ≈ 0.5 (germline)
VAF < 0.4 or > 0.6 may indicate:
- Allele-specific amplification
- Loss of heterozygosity
- Subclonal variants
```

### 5.4 Structural Variant Calling

#### 5.4.1 Detection Methods

```
1. Read-pair analysis (discordant pairs)
2. Split-read analysis
3. Read-depth analysis
4. Local assembly
```

#### 5.4.2 SV Categories

- Deletion: Loss of sequence
- Insertion: Gain of sequence
- Duplication: Tandem or interspersed
- Inversion: Reversed orientation
- Translocation: Inter-chromosomal

#### 5.4.3 Tools

- Manta: Illumina structural variants
- PBSV: PacBio structural variants
- Sniffles: Nanopore structural variants
- LUMPY: Multi-platform SV calling

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
