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
2. Van 선행 연구. "Ten years of next-generation sequencing technology." Trends in Genetics.
3. 선행 연구. "The Genome Analysis Toolkit: A MapReduce framework." Genome Research.
4. 선행 연구. "The Ensembl Variant Effect Predictor." Genome Biology.
5. 선행 연구. "Standards and guidelines for the interpretation of sequence variants." ACMG/AMP.

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
