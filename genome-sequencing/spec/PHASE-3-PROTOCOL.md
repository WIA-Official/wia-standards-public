# WIA-BIO-002 PHASE 3 — Protocol Specification

**Standard:** WIA-BIO-002
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 3 of 4)

---

## 6. Genome Annotation

### 6.1 Functional Annotation

#### 6.1.1 Variant Effect Predictor (VEP)

Consequence types:
```
- Transcript consequences:
  * stop_gained
  * missense_variant
  * synonymous_variant
  * splice_donor_variant
  * frameshift_variant

- Regulatory consequences:
  * TF_binding_site_variant
  * regulatory_region_variant
```

#### 6.1.2 Impact Categories

```
HIGH: Protein-truncating variants
  - stop_gained, frameshift, splice_donor/acceptor

MODERATE: Non-synonymous variants
  - missense_variant, inframe_indel

LOW: Synonymous variants
  - synonymous_variant

MODIFIER: Non-coding variants
  - intron_variant, upstream/downstream_variant
```

### 6.2 Database Annotations

#### 6.2.1 Population Databases

**gnomAD (Genome Aggregation Database)**
```
- 125,748 exomes
- 71,702 genomes
- Purpose: Filter common variants
- Threshold: MAF > 0.01 (1%)
```

**dbSNP (Single Nucleotide Polymorphism Database)**
```
- >600 million variants
- rsID identifiers
- Population frequencies
```

#### 6.2.2 Clinical Databases

**ClinVar**
```
- Clinical significance:
  * Pathogenic
  * Likely pathogenic
  * Uncertain significance (VUS)
  * Likely benign
  * Benign
```

**COSMIC (Catalogue of Somatic Mutations in Cancer)**
```
- Cancer-specific mutations
- Mutation frequency
- Tissue distribution
```

**OMIM (Online Mendelian Inheritance in Man)**
```
- Gene-disease associations
- Inheritance patterns
- Phenotype descriptions
```

### 6.3 Pathogenicity Prediction

#### 6.3.1 In Silico Tools

**SIFT (Sorting Intolerant From Tolerant)**
```
Score: 0-1
Prediction: <0.05 = Deleterious
```

**PolyPhen-2 (Polymorphism Phenotyping v2)**
```
Score: 0-1
Prediction: >0.85 = Probably damaging
```

**CADD (Combined Annotation Dependent Depletion)**
```
Phred-scaled score
Threshold: >15 = deleterious
         >20 = highly deleterious
```

#### 6.3.2 ACMG Guidelines

**Evidence Categories**:
```
Pathogenic:
- PVS1: Null variant in LoF-intolerant gene
- PS1-4: Strong evidence
- PM1-6: Moderate evidence
- PP1-5: Supporting evidence

Benign:
- BA1: Stand-alone benign
- BS1-4: Strong evidence
- BP1-7: Supporting evidence
```

**Classification**:
```
Pathogenic: ≥1 PVS + ≥1 PS, or ≥2 PS, or 1 PS + ≥2 PM
Likely Pathogenic: 1 PVS + 1 PM, or 1 PS + 1-2 PM, or ≥3 PM
VUS: Not meeting criteria
Likely Benign: 1 BS + 1 BP, or ≥2 BP
Benign: 1 BA, or ≥2 BS
```

---

## 7. Implementation Guidelines

### 7.1 Required Components

Any WIA-BIO-002 compliant system must include:

1. **Quality Control Module**: FastQC, MultiQC
2. **Alignment Engine**: BWA-MEM, STAR, Minimap2
3. **Variant Caller**: GATK, FreeBayes, Mutect2
4. **Annotation Engine**: VEP, ANNOVAR, SnpEff
5. **Reporting System**: Clinical report generation

### 7.2 API Interface

#### 7.2.1 Calculate Coverage

```typescript
interface CoverageRequest {
  totalReads: number;      // Number of reads
  readLength: number;      // Read length (bp)
  genomeSize: number;      // Genome size (bp)
  pairedEnd?: boolean;     // Paired-end sequencing
}

interface CoverageResponse {
  averageDepth: number;    // Mean coverage
  expectedUniformity: number;
  totalBases: number;      // Total bases sequenced
  feasibility: 'excellent' | 'good' | 'acceptable' | 'insufficient';
}
```

#### 7.2.2 Validate Quality

```typescript
interface QualityValidation {
  q30Percentage: number;     // % bases ≥Q30
  meanCoverage: number;      // Mean depth
  coverageUniformity: number; // 0-1
  mappingRate: number;       // 0-1
  duplicationRate: number;   // 0-1
  contaminationRate: number; // 0-1
}

interface QualityResult {
  isValid: boolean;
  grade: 'clinical' | 'research' | 'failed';
  errors: string[];
  warnings: string[];
  recommendations: string[];
}
```

#### 7.2.3 Call Variants

```typescript
interface VariantCallRequest {
  bamFile: string;
  referenceGenome: 'hg38' | 'hg19' | 'GRCh38' | 'GRCh37';
  caller: 'gatk' | 'freebayes' | 'mutect2';
  minDepth: number;
  minQuality: number;
  minVAF: number;
}

interface Variant {
  chromosome: string;
  position: number;
  reference: string;
  alternative: string;
  quality: number;
  depth: number;
  vaf: number;
  filter: 'PASS' | 'FAIL';
  genotype: string;
}
```

### 7.3 Data Formats

#### 7.3.1 Sequencing Run Metadata

```json
{
  "run_id": "SEQ20250126_001",
  "platform": "Illumina NovaSeq 6000",
  "chemistry": "S4",
  "read_length": 150,
  "paired_end": true,
  "flow_cell": "HXXXYYYY",
  "date": "2025-01-26T10:00:00Z",
  "operator": "lab_tech_01"
}
```

#### 7.3.2 Sample Metadata

```json
{
  "sample_id": "SAMPLE001",
  "patient_id": "PATIENT_HASHED_ID",
  "sample_type": "blood",
  "tissue_type": "germline",
  "collection_date": "2025-01-25",
  "library_prep": "TruSeq DNA Nano",
  "target_coverage": 30,
  "reference_genome": "hg38"
}
```

### 7.4 Error Handling

Standard error codes:

| Code | Meaning | Action |
|------|---------|--------|
| B001 | Insufficient coverage | Resequence |
| B002 | Low Q30 percentage | Check chemistry/cluster density |
| B003 | High duplication rate | Increase DNA input |
| B004 | Poor uniformity | Check library prep |
| B005 | Contamination detected | Validate sample/reagents |
| B006 | Mapping failure | Check reference genome |

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
