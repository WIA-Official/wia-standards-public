# WIA-BIO-013: Microbiome Specification v1.0

> **Standard ID:** WIA-BIO-013
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Biotechnology Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Sample Collection and Storage](#2-sample-collection-and-storage)
3. [DNA Extraction](#3-dna-extraction)
4. [Sequencing Methods](#4-sequencing-methods)
5. [Bioinformatics Pipelines](#5-bioinformatics-pipelines)
6. [Diversity Analysis](#6-diversity-analysis)
7. [Taxonomic Classification](#7-taxonomic-classification)
8. [Functional Annotation](#8-functional-annotation)
9. [Clinical Applications](#9-clinical-applications)
10. [References](#10-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the standard methods and computational protocols for microbiome analysis, from sample collection to clinical interpretation, ensuring reproducibility and interoperability across studies.

### 1.2 Scope

The standard covers:
- Sample collection and preservation protocols
- DNA/RNA extraction methods
- Sequencing technologies (16S, ITS, shotgun)
- Bioinformatics analysis pipelines
- Diversity metrics and statistical methods
- Taxonomic and functional annotation
- Clinical interpretation and reporting

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to democratize microbiome science by providing accessible, standardized methods that advance human health research worldwide.

### 1.4 Terminology

- **Microbiome**: Complete collection of microorganisms in an environment
- **Microbiota**: The microorganisms themselves
- **16S rRNA**: Bacterial ribosomal RNA gene for identification
- **OTU**: Operational Taxonomic Unit (97% similarity clusters)
- **ASV**: Amplicon Sequence Variant (exact sequences)
- **Alpha Diversity**: Within-sample diversity
- **Beta Diversity**: Between-sample diversity

---

## 2. Sample Collection and Storage

### 2.1 Collection Methods

#### 2.1.1 Gut Microbiome (Fecal)

**Collection Protocol**:
```
1. Collect 200-500 mg of stool sample
2. Place in sterile collection tube
3. Add DNA stabilization buffer if available
4. Mix thoroughly by vortexing
5. Store immediately as per Section 2.2
```

**Timing**: Morning sample preferred, avoid antibiotics for 4 weeks prior

#### 2.1.2 Skin Microbiome

**Collection Protocol**:
```
1. Swab skin area (2 × 2 cm) for 30 seconds
2. Place swab in sterile tube with buffer
3. Break off swab handle and cap tube
4. Store as per Section 2.2
```

**Sites**: Forehead, antecubital fossa, volar forearm, plantar foot

#### 2.1.3 Oral Microbiome

**Collection Protocol**:
```
1. Collect saliva (2 mL) in sterile tube
2. Add DNA/RNA stabilizer (1:1 ratio)
3. Mix by inversion
4. Store as per Section 2.2
```

**Timing**: Fasting state, no oral hygiene for 2 hours prior

### 2.2 Storage Conditions

| Storage Method | Temperature | Duration | DNA Quality |
|----------------|-------------|----------|-------------|
| Immediate extraction | 4°C | 0-24 hours | Excellent |
| Short-term | -20°C | 1-7 days | Good |
| Long-term | -80°C | Years | Excellent |
| Stabilization buffer | Room temp | Weeks | Good |

### 2.3 Sample Metadata

Required metadata (following MIxS standards):
- Sample ID (unique identifier)
- Collection date and time
- Subject demographics (age, sex, BMI)
- Body site or environment
- Recent medication (antibiotics, probiotics)
- Diet information (24-48 hour recall)
- Health status and conditions
- Storage method and duration

---

## 3. DNA Extraction

### 3.1 Extraction Protocols

#### 3.1.1 Mechanical Lysis + Column Purification

**Recommended for**: Gram-positive bacteria, spores

```
1. Add 200 mg sample to bead-beating tube
2. Add lysis buffer (750 μL) and proteinase K (20 μL)
3. Bead-beat: 3 min at 5000 rpm
4. Incubate: 70°C for 10 min
5. Add binding buffer and ethanol
6. Column purification (wash and elute)
7. Elute DNA in 50-100 μL buffer
```

#### 3.1.2 Enzymatic Lysis

**Recommended for**: Gram-negative bacteria, low biomass

```
1. Add sample to tube with enzymatic lysis buffer
2. Incubate with lysozyme (37°C, 30 min)
3. Add proteinase K (56°C, 30 min)
4. Column purification as above
```

### 3.2 Quality Control

**DNA Quantification**:
- Method: Fluorometry (Qubit) or spectrophotometry
- Concentration: >5 ng/μL (ideal: 10-50 ng/μL)
- Purity: A260/A280 = 1.8-2.0, A260/A230 > 1.5

**DNA Integrity**:
- Gel electrophoresis: High molecular weight band
- Fragment size: >10 kb for shotgun, >1 kb for 16S

---

## 4. Sequencing Methods

### 4.1 16S rRNA Gene Sequencing

#### 4.1.1 Variable Regions

| Region | Forward Primer | Reverse Primer | Length | Resolution |
|--------|----------------|----------------|---------|-----------|
| V1-V2 | 27F | 338R | 311 bp | Low |
| V3-V4 | 341F | 805R | 464 bp | High |
| V4 | 515F | 806R | 291 bp | Medium |
| V4-V5 | 515F | 926R | 411 bp | High |

**Recommended**: V3-V4 or V4 region for Illumina MiSeq (2×300 bp)

#### 4.1.2 PCR Protocol

```
Master Mix (25 μL reaction):
- 2× PCR Master Mix: 12.5 μL
- Forward primer (10 μM): 1 μL
- Reverse primer (10 μM): 1 μL
- Template DNA (10 ng): 2 μL
- Water: 8.5 μL

Cycling Conditions:
1. Initial denaturation: 95°C, 3 min
2. 25 cycles:
   - Denaturation: 95°C, 30 sec
   - Annealing: 55°C, 30 sec
   - Extension: 72°C, 30 sec
3. Final extension: 72°C, 5 min
4. Hold: 4°C
```

#### 4.1.3 Library Preparation

1. PCR amplification with barcoded primers
2. PCR cleanup (AMPure XP beads, 0.8× ratio)
3. Quantification and normalization
4. Pool libraries in equimolar amounts
5. Quality check (Bioanalyzer or TapeStation)
6. Sequence on Illumina (MiSeq, NextSeq, or NovaSeq)

### 4.2 Shotgun Metagenomics

#### 4.2.1 Library Preparation

**Input**: 100-500 ng high-quality DNA

```
1. DNA fragmentation: 300-400 bp (Covaris or enzymatic)
2. End repair and A-tailing
3. Adapter ligation (indexed adapters)
4. Size selection (AMPure XP beads)
5. PCR enrichment (5-8 cycles)
6. Final cleanup and QC
```

#### 4.2.2 Sequencing Depth

| Sample Type | Recommended Depth | Coverage |
|-------------|-------------------|----------|
| Gut microbiome | 10-20 M reads | 1-5 Gb |
| Skin microbiome | 5-10 M reads | 0.5-2 Gb |
| Low biomass | 20-50 M reads | 5-10 Gb |

### 4.3 ITS Sequencing (Fungi)

**Target**: Internal Transcribed Spacer (ITS1 or ITS2)

**Primers**:
- ITS1F / ITS2 (ITS1 region)
- fITS7 / ITS4 (ITS2 region)

**Protocol**: Similar to 16S, with optimized annealing temperature (52-54°C)

---

## 5. Bioinformatics Pipelines

### 5.1 16S rRNA Analysis (QIIME2)

#### 5.1.1 Quality Control and Denoising

```bash
# Import data
qiime tools import \
  --type 'SampleData[PairedEndSequencesWithQuality]' \
  --input-path manifest.tsv \
  --output-path demux.qza

# Quality filtering and denoising (DADA2)
qiime dada2 denoise-paired \
  --i-demultiplexed-seqs demux.qza \
  --p-trim-left-f 13 \
  --p-trim-left-r 13 \
  --p-trunc-len-f 250 \
  --p-trunc-len-r 250 \
  --o-table table.qza \
  --o-representative-sequences rep-seqs.qza \
  --o-denoising-stats denoising-stats.qza
```

**Quality Thresholds**:
- Minimum Q score: 30
- Expected errors: <2 per read
- Minimum length: 250 bp (after trimming)

#### 5.1.2 Taxonomic Classification

```bash
# Train classifier (Silva 138 database)
qiime feature-classifier classify-sklearn \
  --i-classifier silva-138-99-nb-classifier.qza \
  --i-reads rep-seqs.qza \
  --o-classification taxonomy.qza

# Generate taxonomy table
qiime metadata tabulate \
  --m-input-file taxonomy.qza \
  --o-visualization taxonomy.qzv
```

**Databases**:
- Silva: Comprehensive, regularly updated
- Greengenes: Well-curated, widely used
- RDP: Ribosomal Database Project

### 5.2 Shotgun Metagenomics

#### 5.2.1 Quality Control (Trimmomatic)

```bash
trimmomatic PE -phred33 \
  input_R1.fastq.gz input_R2.fastq.gz \
  output_R1_paired.fastq.gz output_R1_unpaired.fastq.gz \
  output_R2_paired.fastq.gz output_R2_unpaired.fastq.gz \
  ILLUMINACLIP:adapters.fa:2:30:10 \
  LEADING:3 TRAILING:3 SLIDINGWINDOW:4:20 MINLEN:50
```

#### 5.2.2 Host Removal (Bowtie2)

```bash
# Build host genome index (human GRCh38)
bowtie2-build GRCh38.fa GRCh38_index

# Remove host reads
bowtie2 -p 16 -x GRCh38_index \
  -1 R1_paired.fastq.gz -2 R2_paired.fastq.gz \
  --un-conc-gz nonhost_R%.fastq.gz \
  -S aligned.sam
```

#### 5.2.3 Taxonomic Profiling (MetaPhlAn)

```bash
metaphlan nonhost_R1.fastq.gz,nonhost_R2.fastq.gz \
  --input_type fastq \
  --nproc 16 \
  --bowtie2out metagenome.bowtie2.bz2 \
  -o profiled_metagenome.txt
```

#### 5.2.4 Functional Profiling (HUMAnN)

```bash
humann --input nonhost_R1.fastq.gz \
  --output humann_output \
  --threads 16 \
  --nucleotide-database chocophlan \
  --protein-database uniref90
```

### 5.3 Assembly-Based Analysis

#### 5.3.1 Metagenomic Assembly (MEGAHIT)

```bash
megahit -1 R1.fastq.gz -2 R2.fastq.gz \
  -o assembly_output \
  --min-contig-len 500 \
  --k-min 27 --k-max 127 --k-step 10 \
  -t 32
```

#### 5.3.2 Binning (MetaBAT2)

```bash
# Calculate depth
jgi_summarize_bam_contig_depths --outputDepth depth.txt alignment.bam

# Bin contigs
metabat2 -i contigs.fa -a depth.txt -o bins/bin \
  -m 1500 -t 32 --saveCls
```

---

## 6. Diversity Analysis

### 6.1 Alpha Diversity

#### 6.1.1 Shannon Diversity Index

**Formula**:
```
H' = -Σ(pi × ln(pi))
```

**Interpretation**:
- H' = 0: No diversity (single species)
- H' = 1-2: Low diversity
- H' = 3-4: Medium diversity
- H' > 4: High diversity

**Implementation**:
```python
import numpy as np

def shannon_diversity(abundances):
    """Calculate Shannon diversity index"""
    proportions = abundances / abundances.sum()
    proportions = proportions[proportions > 0]  # Remove zeros
    return -np.sum(proportions * np.log(proportions))
```

#### 6.1.2 Simpson Diversity Index

**Formula**:
```
D = 1 - Σ(pi²)
```

**Implementation**:
```python
def simpson_diversity(abundances):
    """Calculate Simpson diversity index"""
    proportions = abundances / abundances.sum()
    return 1 - np.sum(proportions ** 2)
```

#### 6.1.3 Observed Species (Richness)

**Formula**:
```
S = Number of unique species with abundance > 0
```

#### 6.1.4 Chao1 Estimator

**Formula**:
```
Chao1 = S_obs + (n₁² / 2n₂)
```

Where:
- S_obs = Observed species
- n₁ = Number of singletons
- n₂ = Number of doubletons

### 6.2 Beta Diversity

#### 6.2.1 Bray-Curtis Dissimilarity

**Formula**:
```
BC_ij = 1 - (2 × C_ij / (S_i + S_j))
```

Where:
- C_ij = Sum of minimum abundances between samples i and j
- S_i, S_j = Total abundances in samples i and j

**Implementation**:
```python
def bray_curtis(sample1, sample2):
    """Calculate Bray-Curtis dissimilarity"""
    numerator = 2 * np.sum(np.minimum(sample1, sample2))
    denominator = np.sum(sample1) + np.sum(sample2)
    return 1 - (numerator / denominator)
```

#### 6.2.2 UniFrac Distance

**Weighted UniFrac**:
```
d = Σ(b_i × |p_i^A - p_i^B|) / Σb_i
```

Where:
- b_i = Branch length
- p_i^A, p_i^B = Proportions in samples A and B

**Unweighted UniFrac**: Presence/absence only (binary)

#### 6.2.3 Jaccard Index

**Formula**:
```
J = |A ∩ B| / |A ∪ B|
```

**Dissimilarity**: 1 - J

### 6.3 Rarefaction

**Purpose**: Normalize samples to equal depth for fair comparison

**Method**:
```python
def rarefy(abundances, depth):
    """Rarefy to specified depth"""
    total = abundances.sum()
    if total < depth:
        raise ValueError("Sample depth below rarefaction threshold")

    # Random sampling without replacement
    indices = np.random.choice(total, size=depth, replace=False)
    rarefied = np.bincount(indices, minlength=len(abundances))
    return rarefied[:len(abundances)]
```

**Recommended Depth**: Minimum sample depth or 10,000 reads

---

## 7. Taxonomic Classification

### 7.1 Classification Algorithms

#### 7.1.1 Naive Bayes Classifier

**Method**: Probabilistic classification based on k-mer frequencies

**Databases**:
- Silva 138 (99% identity)
- Greengenes 13_8 (97% OTUs)
- RDP (Ribosomal Database Project)

**Confidence Threshold**: >0.7 (70%) for genus-level assignment

#### 7.1.2 BLAST-Based Classification

**Parameters**:
- E-value: <1e-10
- Identity: >97% for species, >94% for genus
- Coverage: >80% of query sequence

#### 7.1.3 Exact Match (ASV)

**Method**: DADA2 or Deblur for single-nucleotide resolution

**Advantages**:
- No arbitrary clustering threshold
- Reproducible across studies
- Strain-level resolution

### 7.2 Taxonomic Ranks

| Rank | Abbreviation | Example |
|------|--------------|---------|
| Domain | d__ | d__Bacteria |
| Phylum | p__ | p__Firmicutes |
| Class | c__ | c__Clostridia |
| Order | o__ | o__Clostridiales |
| Family | f__ | f__Lachnospiraceae |
| Genus | g__ | g__Roseburia |
| Species | s__ | s__Roseburia intestinalis |

### 7.3 Relative Abundance Table

**Format** (TSV):
```
Feature_ID    Sample1    Sample2    Sample3    Taxonomy
ASV001        1523       2341       987        d__Bacteria;p__Firmicutes;c__Clostridia;...
ASV002        234        456        123        d__Bacteria;p__Bacteroidetes;c__Bacteroidia;...
ASV003        5678       4321       6789       d__Bacteria;p__Firmicutes;c__Bacilli;...
```

---

## 8. Functional Annotation

### 8.1 Gene Prediction (Prodigal)

```bash
prodigal -i contigs.fa -o genes.gff -a proteins.faa \
  -p meta -f gff
```

### 8.2 Functional Databases

| Database | Description | Size | Update Frequency |
|----------|-------------|------|------------------|
| KEGG | Metabolic pathways | ~23K pathways | Annual |
| COG | Orthologous groups | ~200K proteins | Periodic |
| UniRef90 | Protein clusters | ~140M sequences | Quarterly |
| Pfam | Protein families | ~20K families | Annual |

### 8.3 Pathway Analysis

#### 8.3.1 KEGG Orthology (KO)

**Example Pathways**:
- ko00010: Glycolysis / Gluconeogenesis
- ko00020: Citrate cycle (TCA cycle)
- ko00620: Pyruvate metabolism
- ko00650: Butanoate metabolism (butyrate production)

#### 8.3.2 MetaCyc Pathways

**Example**:
- PWY-5677: Succinate fermentation to butanoate
- PWY-7323: Superpathway of GDP-mannose-derived O-antigen

### 8.4 Metabolite Prediction

**Short-Chain Fatty Acids (SCFA)**:
- Acetate: Bacteroides, Bifidobacterium
- Propionate: Bacteroides, Phascolarctobacterium
- Butyrate: Faecalibacterium, Roseburia, Eubacterium

**Formula for Butyrate Production Potential**:
```
BPP = Σ(RA_i × BP_i)
```

Where:
- RA_i = Relative abundance of butyrate-producing genus i
- BP_i = Butyrate production capacity (0-1 scale)

---

## 9. Clinical Applications

### 9.1 Gut Health Assessment

#### 9.1.1 Dysbiosis Index

**Formula**:
```
DI = log₁₀(Σ[pathobionts] / Σ[commensals])
```

**Interpretation**:
- DI < 0: Healthy (more commensals)
- DI = 0: Balanced
- DI > 0: Dysbiosis (more pathobionts)

#### 9.1.2 Key Markers

**Healthy Gut**:
- High: Faecalibacterium, Akkermansia, Roseburia
- Firmicutes/Bacteroidetes ratio: 1-3
- Alpha diversity: Shannon > 3.5

**Dysbiosis**:
- High: Escherichia, Klebsiella, Enterococcus
- Low: Beneficial commensals
- Alpha diversity: Shannon < 2.5

### 9.2 Disease Associations

| Condition | Microbiome Signature | Key Taxa |
|-----------|---------------------|----------|
| IBD (Crohn's) | Low diversity, dysbiosis | ↑ Proteobacteria, ↓ Faecalibacterium |
| IBS | Variable, often low diversity | ↑ Firmicutes, ↓ Bacteroidetes |
| Obesity | Low diversity | ↑ Firmicutes/Bacteroidetes ratio |
| Type 2 Diabetes | Dysbiosis, low butyrate | ↓ Roseburia, ↓ Faecalibacterium |
| Autism | Altered diversity | ↑ Clostridia, ↓ Prevotella |

### 9.3 Probiotic Recommendations

**Selection Criteria**:
1. Clinical evidence (RCTs)
2. Strain specificity
3. Viability (CFU > 10⁹)
4. Safety profile

**Common Strains**:
- Lactobacillus rhamnosus GG: Diarrhea, immune support
- Bifidobacterium longum: IBS, stress
- Saccharomyces boulardii: C. difficile, antibiotic-associated diarrhea

### 9.4 Fecal Microbiota Transplant (FMT)

#### 9.4.1 Donor Screening

**Exclusion Criteria**:
- Antibiotic use (6 months)
- GI disorders or symptoms
- Metabolic syndrome
- Recent travel to endemic areas
- High-risk sexual behavior

**Testing**:
- Stool: Culture, parasites, C. difficile, viruses
- Blood: HIV, hepatitis, syphilis
- Microbiome: Diversity and composition analysis

#### 9.4.2 FMT Efficacy Prediction

**Formula**:
```
FMT_Score = (D_post - D_pre) / D_donor
```

Where:
- D = Diversity (Shannon index)
- Higher score = Better engraftment

**Success Criteria**:
- Clinical improvement
- Increased alpha diversity
- Donor microbiome engraftment >30%

### 9.5 Clinical Reporting

**Report Sections**:
1. Sample Information
2. Quality Metrics
3. Diversity Analysis
4. Taxonomic Composition
5. Functional Potential
6. Clinical Interpretation
7. Recommendations

**Example Interpretation**:
```
Patient ID: GUT12345
Shannon Diversity: 2.8 (Low - Reference: 3.5-5.0)
Dysbiosis Index: +0.45 (Moderate dysbiosis)

Key Findings:
- Reduced beneficial bacteria (Faecalibacterium 1.2%, ref: 5-10%)
- Elevated pathobionts (Escherichia 8.5%, ref: <2%)
- Low butyrate production potential

Recommendations:
1. Dietary fiber supplementation (25-30g/day)
2. Consider probiotic: Lactobacillus rhamnosus GG
3. Follow-up testing in 3 months
```

---

## 10. References

### 10.1 Normative References (Standards & Guidance)

1. ISO 20387:2018 — Biotechnology — Biobanking — General requirements for biobanking.
2. ISO 23418:2022 — Microbiology of the food chain — Whole genome sequencing for typing and genomic characterization of bacteria — General requirements and guidance.
3. ISO/IEC 11179 — Information technology — Metadata registries.
4. CLSI MM09 — Nucleic Acid Sequencing Methods in Diagnostic Laboratory Medicine.
5. Genomic Standards Consortium (GSC) — Minimum Information about any (x) Sequence (MIxS) checklists.
6. Genomic Standards Consortium (GSC) — Minimum Information about a Marker Gene Sequence (MIMARKS).
7. Genomic Standards Consortium (GSC) — Minimum Information about a Metagenomic Sequence (MIMS).

### 10.2 Diversity Metrics (Foundational Definitions)

The diversity indices used in §6 are defined in widely adopted public sources:
- Shannon entropy — defined in classical information theory (Bell System Technical Journal, 1948).
- Simpson's index — defined in standard ecological-diversity literature (1949).
- UniFrac phylogenetic distance — defined in the open scientific literature (2005) and implemented in the public QIIME 2 toolchain.

### 10.3 Public Data Resources

- SILVA rRNA database — https://www.arb-silva.de
- Greengenes2 — https://greengenes2.ucsd.edu
- KEGG — https://www.genome.jp/kegg
- MIxS / GSC — https://genomicsstandardsconsortium.github.io/mixs
- NCBI Sequence Read Archive (SRA) — https://www.ncbi.nlm.nih.gov/sra
- EMBL-EBI MGnify — https://www.ebi.ac.uk/metagenomics
- Human Microbiome Project Data Coordination Center — https://hmpdacc.org

### 10.4 Informative References (Field Background)

The clinical microbiome literature — including metagenome-wide association studies of gut microbiota in type-2 diabetes, reduced faecal-microbiota diversity in Crohn's disease, faecal microbiota transplantation for recurrent *Clostridioides difficile* infection, and the methodological foundations of the QIIME 2 and MetaPhlAn pipelines — is documented in the open peer-reviewed literature accessible through the data resources in §10.3. Implementers requiring historical or experimental context are referred to those archives rather than to specific bibliographic entries reproduced here.

### 10.5 WIA Standards

- WIA-INTENT: Intent-based interfaces
- WIA-OMNI-API: Universal API gateway
- WIA-BIO-001: General biotechnology standards
- WIA-SOCIAL: Data sharing and collaboration

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-BIO-013 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
