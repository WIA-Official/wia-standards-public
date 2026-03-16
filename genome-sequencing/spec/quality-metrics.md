# WIA-BIO-001: Quality Metrics Specification

## Version 1.0 | 2025-01-15

---

## Table of Contents
1. [Overview](#overview)
2. [Sequencing Quality Metrics](#sequencing-quality-metrics)
3. [Alignment Quality Metrics](#alignment-quality-metrics)
4. [Variant Quality Metrics](#variant-quality-metrics)
5. [Coverage Metrics](#coverage-metrics)
6. [Contamination Assessment](#contamination-assessment)
7. [Quality Control Thresholds](#quality-control-thresholds)

---

## Overview

This document defines comprehensive quality metrics for genome sequencing data as specified in WIA-BIO-001. These metrics ensure data reliability, reproducibility, and clinical validity across all sequencing platforms and applications.

### Quality Metric Categories

| Category | Purpose | Tools | Frequency |
|----------|---------|-------|-----------|
| **Read Quality** | Assess base call accuracy | FastQC, MultiQC | Per run |
| **Alignment Quality** | Evaluate mapping performance | SAMtools, Qualimap | Per sample |
| **Variant Quality** | Validate variant calls | bcftools, GATK | Per VCF |
| **Coverage** | Measure sequencing depth | Mosdepth, GATK DepthOfCoverage | Per sample |
| **Contamination** | Detect sample mixing | VerifyBAMID, Conpair | Per sample |

---

## Sequencing Quality Metrics

### 1. Phred Quality Scores

**Definition:** Logarithmic measure of base calling error probability.

```
Q = -10 × log₁₀(P)

where:
Q = Phred quality score
P = probability of incorrect base call
```

**Quality Score Interpretation:**

| Phred Score | Base Call Accuracy | Error Rate | Clinical Grade |
|-------------|-------------------|------------|----------------|
| Q10 | 90% | 1 in 10 | Unacceptable |
| Q20 | 99% | 1 in 100 | Minimum |
| Q30 | 99.9% | 1 in 1,000 | Good |
| Q40 | 99.99% | 1 in 10,000 | Excellent |

**Requirements:**
- **Research Grade:** ≥85% of bases at Q30
- **Clinical Grade:** ≥95% of bases at Q30
- **Diagnostic Grade:** ≥98% of bases at Q30

### 2. Per-Base Sequence Quality

**Metric:** Average quality score across all reads at each base position.

**Acceptable Criteria:**
- Median quality score >28 across all positions
- Lower quartile >22 for positions 1-100
- No positions with median <20 in first 75bp
- Slight degradation acceptable in final 10% of read length

**Common Issues:**
- **Quality drop at read ends:** Normal for Illumina, trim if severe
- **Unexpected dips:** Indicates sequencing chemistry issues
- **Bimodal distribution:** Possible contamination or mixed library

### 3. Per-Sequence Quality Scores

**Metric:** Average quality across entire length of each read.

**Acceptable Distribution:**
- Peak at Q35-Q40 for modern Illumina platforms
- <5% of reads with mean quality <25
- No secondary peak at low quality (indicates contamination)

### 4. GC Content Distribution

**Metric:** Percentage of G and C bases per read.

**Expected Values:**
- **Human genome:** 41-42% GC (bimodal due to isochores)
- **Bacterial genomes:** Species-specific (30-70% range)
- **Viral genomes:** Highly variable

**Flags:**
- Sharp peaks: Possible adapter contamination or specific sequence overrepresentation
- Unusual distribution: Sample contamination or bias in library prep

### 5. Sequence Duplication Levels

**Metric:** Percentage of reads that are duplicates (PCR or optical).

**Thresholds:**
- **WGS (low input):** <20% PCR duplicates
- **WGS (standard):** <10% PCR duplicates
- **WES:** <40% (expected due to target enrichment)
- **RNA-Seq:** High duplication expected for abundant transcripts

**Tools:**
```bash
# Picard MarkDuplicates
picard MarkDuplicates \
  INPUT=aligned.bam \
  OUTPUT=marked.bam \
  METRICS_FILE=dup_metrics.txt \
  OPTICAL_DUPLICATE_PIXEL_DISTANCE=2500
```

---

## Alignment Quality Metrics

### 1. Mapping Quality (MAPQ)

**Definition:** Phred-scaled probability that read is incorrectly mapped.

```
MAPQ = -10 × log₁₀(P_error)
```

**MAPQ Score Interpretation:**

| MAPQ | Mapping Accuracy | Interpretation | Usage |
|------|------------------|----------------|-------|
| 0 | Ambiguous | Multiple equally good alignments | Exclude from analysis |
| 1-9 | Low confidence | Repetitive regions | Use with caution |
| 10-19 | Moderate | May have alternative alignments | Acceptable for coverage |
| 20-39 | Good | <1% error probability | Standard threshold |
| 40-60 | Excellent | <0.01% error | High confidence calls |

**Requirements:**
- ≥90% of reads with MAPQ ≥20 (research)
- ≥95% of reads with MAPQ ≥30 (clinical)

### 2. Properly Paired Reads

**Metric:** Percentage of paired-end reads mapping in correct orientation and distance.

**Criteria:**
- ≥95% properly paired for high-quality data
- ≥90% acceptable for difficult genomes or degraded samples

**Insert Size Distribution:**
- Mean: 350-500bp (library-dependent)
- Standard deviation: <100bp
- No unexpected peaks or long tail

### 3. Alignment Rate

**Metric:** Percentage of reads successfully aligned to reference.

| Sample Type | Minimum % Aligned | Optimal % Aligned |
|-------------|-------------------|-------------------|
| Human WGS | >95% | >98% |
| Human WES | >90% | >95% |
| Microbial | >85% | >95% |
| Metagenomics | Variable | N/A |

**Low alignment causes:**
- Sample contamination (human DNA in microbial sample)
- Wrong reference genome
- Adapter contamination
- Low-quality library

### 4. On-Target Rate (WES/Panel)

**Metric:** Percentage of aligned reads falling within target regions.

**Thresholds:**
- **Exome capture:** ≥70% on-target
- **Panel sequencing:** ≥80% on-target
- **Custom capture:** Depends on design efficiency

**Off-target analysis:**
- High off-target rate: Poor capture efficiency or contamination
- Uniform off-target: Library complexity issues

---

## Variant Quality Metrics

### 1. Variant Quality Score (QUAL)

**Definition:** Phred-scaled quality score for the variant call.

**Filtering Thresholds:**
```bash
# SNPs
QUAL ≥ 30  # Research
QUAL ≥ 50  # Clinical

# Indels
QUAL ≥ 40  # Research (indels are harder to call)
QUAL ≥ 60  # Clinical
```

### 2. Genotype Quality (GQ)

**Metric:** Confidence in the assigned genotype (0/0, 0/1, 1/1).

**Requirements:**
- GQ ≥20 for research variants
- GQ ≥30 for clinical reporting
- GQ ≥50 for diagnostic critical variants

### 3. Allele Balance (AB)

**Formula:**
```
AB = Alternate Allele Depth / Total Depth
```

**Expected Values:**
- **Heterozygous (0/1):** AB = 0.40-0.60 (ideally 0.50)
- **Homozygous Alt (1/1):** AB > 0.90

**Filtering:**
```bash
# Filter heterozygous variants with extreme allele balance
bcftools filter -i '(GT="het" & FORMAT/AB>0.3 & FORMAT/AB<0.7) | GT="hom"' input.vcf
```

### 4. Strand Bias

**Metrics:**
- **FS (Fisher Strand):** Phred-scaled p-value using Fisher's exact test
- **SOR (StrandOddsRatio):** Odds ratio of strand bias

**Thresholds:**
```
FS < 60.0 for SNPs
FS < 200.0 for indels
SOR < 3.0 for SNPs
SOR < 10.0 for indels
```

**Interpretation:**
- High strand bias suggests sequencing artifact or mapping error
- Particularly important for low-frequency variant calling

### 5. Quality by Depth (QD)

**Formula:**
```
QD = QUAL / DP
```

**Thresholds:**
- QD ≥ 2.0 for SNPs
- QD ≥ 1.0 for indels (harder to call, lower threshold)

**Purpose:** Normalizes variant quality by depth to avoid bias toward high-coverage sites.

### 6. Transition/Transversion Ratio (Ti/Tv)

**Expected Ratios:**
- **Whole genome:** 2.0-2.1
- **Exome:** 2.8-3.0 (higher in coding regions)

**Interpretation:**
```
Ti/Tv < 2.0: Likely enriched for false positives
Ti/Tv > 2.3 (WGS): Possibly over-filtered, losing true variants
```

---

## Coverage Metrics

### 1. Mean Coverage Depth

**Formula:**
```
Mean Depth = Total Aligned Bases / Reference Genome Length
```

**Recommended Depths:**

| Application | Minimum Depth | Recommended Depth | Notes |
|-------------|---------------|-------------------|-------|
| **WGS (Germline)** | 30x | 40-60x | Higher for clinical |
| **WGS (Somatic)** | 80x tumor / 40x normal | 100x / 50x | Detect low-frequency variants |
| **WES (Germline)** | 50x | 100x | Account for capture variability |
| **WES (Somatic)** | 150x tumor / 75x normal | 200x / 100x | Low VAF detection |
| **Targeted Panel** | 250x | 500x | High sensitivity needed |
| **RNA-Seq** | N/A (TPM-based) | 30-50M reads | Expression quantification |

### 2. Coverage Uniformity

**Metric:** Coefficient of variation (CV) of coverage across target regions.

**Formula:**
```
CV = Standard Deviation / Mean Coverage
```

**Acceptable Values:**
- **WGS:** CV < 0.3
- **WES:** CV < 0.5 (higher due to capture efficiency variation)

### 3. Percentage of Bases at Depth Thresholds

**Key Metrics:**
```
% Bases ≥10x: Minimum for any analysis
% Bases ≥20x: Research-grade variant calling
% Bases ≥30x: Clinical-grade variant calling
% Bases ≥100x: Low-frequency variant detection
```

**Requirements (Clinical WES):**
- ≥95% of target bases at ≥20x
- ≥90% of target bases at ≥30x

### 4. Gap Coverage

**Definition:** Regions with zero or insufficient coverage.

**Reporting:**
- List all clinically relevant genes with <10x coverage
- Identify systematic gaps (GC-rich regions, pseudo genes)
- Document limitations in clinical reports

---

## Contamination Assessment

### 1. Sample Swaps

**Tool:** VerifyBAMID2

**Metrics:**
```
FREEMIX: Estimated contamination fraction
CHIPMIX: Genotype concordance with expected sample
```

**Thresholds:**
- FREEMIX < 3% (research acceptable)
- FREEMIX < 1% (clinical required)
- FREEMIX > 5% (fail sample)

### 2. Cross-Sample Contamination

**Tool:** Conpair (for tumor-normal pairs)

**Checks:**
- Concordance: Verify tumor and normal are from same individual (>99%)
- Contamination: Estimate normal tissue contamination in tumor

### 3. Species Contamination

**Method:** Align to multiple reference genomes and quantify mapping rates.

**Common Contaminants:**
- **Human samples:** E. coli, Staph, Strep (skin flora)
- **Environmental:** PhiX (Illumina control)

**Screening:**
```bash
# Align unmapped reads to bacterial/viral databases
bwa mem bacterial_db.fa unmapped.fastq | samtools view -c
```

---

## Quality Control Thresholds

### Research-Grade Data

| Metric | Threshold |
|--------|-----------|
| % Bases ≥Q30 | ≥85% |
| % Mapped Reads | ≥95% |
| Mean Coverage | ≥30x |
| % Duplicates | <20% |
| % Properly Paired | ≥90% |
| FREEMIX | <3% |

### Clinical-Grade Data

| Metric | Threshold |
|--------|-----------|
| % Bases ≥Q30 | ≥95% |
| % Mapped Reads | ≥98% |
| Mean Coverage | ≥100x |
| % Duplicates | <10% |
| % Properly Paired | ≥95% |
| FREEMIX | <1% |
| % Target at 30x | ≥90% |

### Diagnostic-Grade Data (CAP/CLIA)

| Metric | Threshold |
|--------|-----------|
| % Bases ≥Q30 | ≥98% |
| % Mapped Reads | ≥99% |
| Mean Coverage | ≥200x (panel) / ≥150x (exome) |
| % Duplicates | <5% |
| % Properly Paired | ≥98% |
| FREEMIX | <0.5% |
| % Target at 100x | ≥95% |

---

## Automated Quality Control

### MultiQC Integration

```bash
# Generate comprehensive QC report
multiqc . \
  --title "WIA-BIO-001 Quality Report" \
  --comment "Sample: PATIENT_001" \
  --config multiqc_config.yaml
```

### Pass/Fail Decision Tree

```
1. Check % Bases ≥Q30
   ├─ FAIL if <85% → Resequence
   └─ PASS → Continue

2. Check Alignment Rate
   ├─ FAIL if <95% → Check contamination/reference
   └─ PASS → Continue

3. Check Mean Coverage
   ├─ FAIL if <30x → Pool more lanes
   └─ PASS → Continue

4. Check Contamination
   ├─ FAIL if >3% → Reject sample
   └─ PASS → Approved for analysis
```

---

## References

1. GATK Best Practices: https://gatk.broadinstitute.org/
2. FastQC Manual: https://www.bioinformatics.babraham.ac.uk/projects/fastqc/
3. SAMtools Specifications: http://www.htslib.org/
4. GA4GH Quality Metrics: https://www.ga4gh.org/

---

**Document Version:** 1.0
**Last Updated:** 2025-01-15
**Maintained by:** WIA BIO Working Group
**License:** Apache 2.0
