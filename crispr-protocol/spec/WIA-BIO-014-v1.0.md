# WIA-BIO-014: CRISPR Protocol Specification v1.0

> **Standard ID:** WIA-BIO-014
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Biotechnology Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [CRISPR Systems](#2-crispr-systems)
3. [Guide RNA Design](#3-guide-rna-design)
4. [Delivery Methods](#4-delivery-methods)
5. [Off-Target Analysis](#5-off-target-analysis)
6. [Editing Outcomes](#6-editing-outcomes)
7. [Validation Methods](#7-validation-methods)
8. [Implementation Guidelines](#8-implementation-guidelines)
9. [Safety and Ethics](#9-safety-and-ethics)
10. [References](#10-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines comprehensive protocols for CRISPR-based genome editing, including system selection, guide RNA design, delivery optimization, and validation methods to ensure reproducible and safe genetic modifications.

### 1.2 Scope

The standard covers:
- CRISPR system types and selection criteria
- Computational guide RNA design algorithms
- Delivery method optimization
- Off-target prediction and validation
- Editing efficiency measurements
- Safety protocols and ethical guidelines

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard provides a framework for responsible genome editing that maximizes therapeutic and research benefits while minimizing risks through rigorous validation and ethical oversight.

### 1.4 Terminology

- **gRNA (guide RNA)**: RNA molecule that directs Cas nuclease to target DNA
- **PAM (Protospacer Adjacent Motif)**: DNA sequence required for Cas binding
- **HDR (Homology-Directed Repair)**: Precise DNA repair using template
- **NHEJ (Non-Homologous End Joining)**: Error-prone DNA repair mechanism
- **Indel**: Insertion or deletion mutation
- **On-target**: Intended genomic target site
- **Off-target**: Unintended genomic sites with sequence similarity

---

## 2. CRISPR Systems

### 2.1 Cas9 Systems

#### 2.1.1 SpCas9 (Streptococcus pyogenes)

**PAM Sequence:** 5'-NGG-3'

**Specifications:**
- Guide RNA length: 20 nucleotides
- Protein size: 1,368 amino acids (160 kDa)
- Optimal temperature: 37°C
- Activity: DNA double-strand breaks

**Formula for Target Site:**
```
Target = 5'-[N₂₀]-NGG-3'
```

Where N₂₀ represents any 20 nucleotide sequence.

**Advantages:**
- High editing efficiency (50-90%)
- Extensively characterized
- Abundant reagents available
- Well-established protocols

**Limitations:**
- Large size limits AAV delivery
- NGG PAM restriction (~1 site per 8bp)
- Off-target potential at NGN sites

#### 2.1.2 SaCas9 (Staphylococcus aureus)

**PAM Sequence:** 5'-NNGRRT-3'

**Specifications:**
- Guide RNA length: 21 nucleotides
- Protein size: 1,053 amino acids (123 kDa)
- Smaller size enables AAV packaging
- Lower off-target activity

#### 2.1.3 High-Fidelity Variants

**SpCas9-HF1:**
- Mutations: N497A, R661A, Q695A, Q926A
- Reduced off-targets by 10-1000×
- Maintained on-target activity: 70-100%

**eSpCas9:**
- Enhanced specificity variant
- Mutations: K848A, K1003A, R1060A
- Reduced off-targets by 10-100×

### 2.2 Cas12a (Cpf1)

**PAM Sequence:** 5'-TTTV-3' (V = A, C, or G)

**Specifications:**
- Guide RNA length: 23 nucleotides
- Staggered cut: 5' overhang
- RNase activity for crRNA processing
- Enables multiplex editing

**Advantages:**
- T-rich PAM for AT-rich genomes
- Single transcript for multiplex
- Smaller size than Cas9

### 2.3 Cas13 (RNA Targeting)

**Target:** Single-stranded RNA

**Specifications:**
- Guide RNA length: 28-30 nucleotides
- No PAM requirement
- Collateral RNase activity
- Applications: RNA knockdown, diagnostics

### 2.4 Base Editors

#### 2.4.1 Cytosine Base Editor (CBE)

**Mechanism:** C → T conversion (or G → A on opposite strand)

**Components:**
- Cas9 nickase (D10A)
- Cytidine deaminase (APOBEC/AID)
- Uracil glycosylase inhibitor (UGI)

**Editing Window:** Positions 4-8 within protospacer

**Efficiency:** 20-60% conversion

#### 2.4.2 Adenine Base Editor (ABE)

**Mechanism:** A → G conversion (or T → C on opposite strand)

**Components:**
- Cas9 nickase (D10A)
- Adenine deaminase (TadA)

**Editing Window:** Positions 4-7 within protospacer

**Efficiency:** 30-70% conversion

### 2.5 Prime Editing

**Mechanism:** Precise insertions, deletions, and all base transitions/transversions

**Components:**
- Cas9 nickase (H840A)
- Reverse transcriptase (M-MLV RT)
- Prime editing guide RNA (pegRNA)

**pegRNA Design:**
```
pegRNA = spacer (20bp) + scaffold + RT template + PBS
```

Where:
- PBS = Primer Binding Site (13 bp)
- RT template = desired edit + homology

**Capabilities:**
- All 12 base-to-base conversions
- Insertions up to 44 bp
- Deletions up to 80 bp
- Efficiency: 10-40%

---

## 3. Guide RNA Design

### 3.1 Target Sequence Selection

#### 3.1.1 Genomic Context

**Criteria:**
1. **Exon targeting**: Target early exons for gene knockout
2. **Functional domains**: Target critical protein domains
3. **Splice sites**: Avoid unless intended disruption
4. **Repetitive elements**: Avoid LINE, SINE, Alu elements

#### 3.1.2 PAM Location

For SpCas9 (NGG PAM):
```
Target Site = [20 bp protospacer] + NGG
```

**PAM Density:**
```
Expected sites = Genome_size / 8
```

For human genome (~3.2 Gbp):
```
NGG sites ≈ 400 million
```

### 3.2 On-Target Scoring

#### 3.2.1 Doench Score (Rule Set 2)

Machine learning model incorporating:
- Nucleotide position weights
- Dinucleotide features
- GC content
- Thermodynamic stability

**Formula:**
```
Score = sigmoid(Σ wᵢfᵢ)
```

Where:
- wᵢ = learned weights
- fᵢ = sequence features
- sigmoid(x) = 1 / (1 + e⁻ˣ)

**Interpretation:**
- Score > 0.6: High activity predicted
- Score 0.4-0.6: Medium activity
- Score < 0.4: Low activity

#### 3.2.2 GC Content

**Optimal Range:** 40-60%

```
GC_content = (G + C) / 20 × 100%
```

**Penalty Function:**
```
GC_score = 1 - |GC_content - 0.5| / 0.5
```

#### 3.2.3 Sequence Constraints

**Avoid:**
- Poly-T tracts (≥4 T's): Transcriptional termination
- High GC (>70%): Poor synthesis
- Low GC (<30%): Low activity
- Homopolymer runs (≥4 identical bases)

### 3.3 Off-Target Prediction

#### 3.3.1 Mismatch Tolerance

SpCas9 tolerates:
- PAM-distal mismatches (positions 1-12): 3-5 mismatches
- PAM-proximal mismatches (positions 13-20): 1-2 mismatches
- PAM variants (NAG, NGA): Reduced activity

**Scoring Function:**
```
Off-target_score = Π (1 - wᵢ × mᵢ)
```

Where:
- wᵢ = position weight (higher near PAM)
- mᵢ = mismatch penalty (0 or 1)

#### 3.3.2 CFD Score (Cutting Frequency Determination)

Empirically derived mismatch penalties:

| Position | Single MM | Double MM | Triple MM |
|----------|-----------|-----------|-----------|
| 1-5      | 0.1       | 0.01      | 0.001     |
| 6-12     | 0.3       | 0.05      | 0.005     |
| 13-17    | 0.6       | 0.2       | 0.02      |
| 18-20    | 0.9       | 0.5       | 0.1       |

**Aggregated CFD Score:**
```
CFD = Π (1 - penalty_i)
```

#### 3.3.3 MIT Specificity Score

```
MIT_score = 100 / (100 + Σ off-target weights)
```

Where off-target weights depend on:
- Number of mismatches
- Mismatch positions
- PAM sequence

**Threshold:** MIT score > 50 recommended

### 3.4 Design Algorithm

```
Algorithm: Optimal_gRNA_Design
Input: Target gene, organism, PAM type
Output: Ranked list of gRNA candidates

1. Extract gene sequence and genomic coordinates
2. Scan for all PAM sites in target region
3. For each PAM site:
   a. Extract 20bp protospacer
   b. Calculate on-target score (Doench)
   c. Calculate GC content
   d. Check sequence constraints
   e. Search genome for off-targets (≤3 mismatches)
   f. Calculate specificity scores (CFD, MIT)
4. Rank by composite score:
   Score = w₁×on-target + w₂×specificity - w₃×off-targets
5. Filter: on-target > 0.4, specificity > 50, off-targets < 5
6. Return top 10 candidates
```

---

## 4. Delivery Methods

### 4.1 Ribonucleoprotein (RNP) Complexes

**Composition:**
- Cas9 protein (purified)
- gRNA (chemically synthesized or in vitro transcribed)
- Molar ratio: 1:1.2 (Cas9:gRNA)

**Advantages:**
- Immediate activity
- No DNA integration
- Reduced off-targets (transient)
- Lower immunogenicity

**Delivery Methods:**
- Electroporation
- Nucleofection
- Microinjection (embryos)
- Cell-penetrating peptides

**Protocol:**
```
1. Complex formation:
   - Mix Cas9 (500 nM) + gRNA (600 nM)
   - Incubate 10 min at room temperature

2. Electroporation:
   - Resuspend cells at 1×10⁶/mL
   - Add RNP complex
   - Pulse: 1,200 V, 20 ms, 2 pulses
   - Recovery in growth medium
```

**Efficiency:** 50-90% in cell lines, 30-70% in primary cells

### 4.2 Plasmid DNA

**Expression Cassette:**
```
CMV/U6 → gRNA → Scaffold
CMV/EF1α → Cas9 → polyA
```

**Advantages:**
- Cost-effective
- Easy to produce
- Prolonged expression

**Limitations:**
- Risk of integration
- Delayed activity
- Higher off-targets

**Delivery:**
- Lipofection
- Electroporation
- Calcium phosphate

### 4.3 Viral Vectors

#### 4.3.1 Lentivirus

**Capacity:** ~8 kb insert

**Advantages:**
- Infects dividing and non-dividing cells
- Stable integration
- High efficiency (>80%)

**Applications:**
- Stable cell lines
- In vivo delivery

**Safety:**
- Use 3rd generation system
- Self-inactivating (SIN) LTR
- Separate packaging plasmids

#### 4.3.2 Adeno-Associated Virus (AAV)

**Capacity:** ~4.7 kb insert

**Strategy for large Cas9:**
```
AAV1: ITR-Promoter-SaCas9-polyA-ITR
AAV2: ITR-U6-gRNA-ITR
```

**Serotypes:**
- AAV9: CNS, heart, muscle
- AAV2: Liver, eye
- AAV8: Liver
- AAV-PHP.eB: Enhanced BBB crossing

**Advantages:**
- Low immunogenicity
- Broad tropism
- Episomal (safer)

**Dose:**
- In vitro: 10⁴-10⁵ vg/cell
- In vivo: 10¹¹-10¹³ vg/kg

#### 4.3.3 Adenovirus

**Capacity:** ~7.5 kb insert

**Advantages:**
- Large capacity
- High transduction efficiency
- No integration

**Limitations:**
- Immunogenic
- Transient expression

### 4.4 Nanoparticles

#### 4.4.1 Lipid Nanoparticles (LNPs)

**Composition:**
- Ionizable lipid (50%)
- DSPC (10%)
- Cholesterol (38.5%)
- PEG-lipid (1.5%)

**Payload:**
- Cas9 mRNA + gRNA
- Or Cas9 protein + gRNA

**Advantages:**
- Clinical-grade manufacturing
- Liver targeting (>80%)
- Lower immunogenicity

#### 4.4.2 Gold Nanoparticles

**Surface Modification:**
- Cas9-gRNA RNP conjugation
- Cell-penetrating peptides
- Targeting ligands

**Applications:**
- Local delivery
- Controlled release

---

## 5. Off-Target Analysis

### 5.1 Computational Prediction

#### 5.1.1 Whole-Genome Alignment

**Tools:**
- Cas-OFFinder: GPU-accelerated search
- CHOPCHOP: Integrated design
- CRISPOR: Comprehensive scoring

**Search Parameters:**
```
- Mismatches: 0-4
- DNA bulges: 0-2
- RNA bulges: 0-2
- PAM variants: Include NAG, NGA
```

**Output:**
```json
{
  "off_target_site": "chr3:12345678",
  "sequence": "ATCGATCGATCGATCGAT-G",
  "mismatches": 1,
  "position": [15],
  "CFD_score": 0.82,
  "chromosome_context": "intergenic"
}
```

#### 5.1.2 Chromatin Accessibility

Off-target activity correlates with chromatin state:

```
Activity = CFD_score × accessibility
```

**Data Sources:**
- ENCODE DNase-seq
- ATAC-seq
- ChIP-seq (H3K4me3, H3K27ac)

### 5.2 Experimental Validation

#### 5.2.1 GUIDE-seq

**Principle:** Integrates double-strand oligonucleotides at break sites

**Protocol:**
```
1. Transfect cells with:
   - Cas9 + gRNA
   - dsODN tag (34 bp, phosphorothioate ends)

2. Extract genomic DNA (3-7 days post)

3. PCR amplify integration sites

4. NGS sequencing

5. Bioinformatic analysis:
   - Align reads to tag sequence
   - Map genomic locations
   - Quantify read counts
```

**Sensitivity:** Detects off-targets >0.1% frequency

#### 5.2.2 CIRCLE-seq

**Principle:** In vitro cleavage of circularized genomic DNA

**Advantages:**
- No cell culture required
- Highly sensitive
- Unbiased genome-wide

**Protocol:**
```
1. Fragment genomic DNA (300 bp)
2. Circularize with ligase
3. In vitro cleavage with Cas9-gRNA RNP
4. Library prep from linear fragments
5. NGS and analysis
```

**Detection:** Off-targets down to 0.01% activity

#### 5.2.3 Digenome-seq

**Principle:** Whole-genome sequencing after in vitro digestion

**Advantages:**
- Direct DSB detection
- No bias

**Limitations:**
- High sequencing depth required (>30×)
- False positives from random breaks

### 5.3 Minimizing Off-Targets

**Strategies:**
```
1. gRNA Design:
   - High specificity score (>70)
   - Avoid seed region mismatches
   - Use truncated gRNAs (17-18 bp)

2. Cas9 Engineering:
   - High-fidelity variants (HF1, HiFi)
   - Enhanced specificity (eSpCas9)

3. Delivery Optimization:
   - RNP for transient activity
   - Optimal dose (minimize excess)
   - Timed delivery

4. Chemical Modifications:
   - 2'-O-methyl-3'-phosphonoacetate (gRNA ends)
   - 2'-O-methyl (positions 1, 2, 20)
```

---

## 6. Editing Outcomes

### 6.1 Non-Homologous End Joining (NHEJ)

**Mechanism:** Error-prone repair of DSBs

**Outcomes:**
- Small insertions (1-10 bp): ~30%
- Small deletions (1-50 bp): ~60%
- Large deletions (>50 bp): ~5%
- Inversions: ~5%

**Indel Spectrum:**
```
+1 insertion (A): 15%
-1 deletion: 25%
-3 deletion: 10%
Other: 50%
```

**Efficiency Formula:**
```
NHEJ_efficiency = (indel_reads / total_reads) × 100%
```

**Typical Range:** 30-90%

### 6.2 Homology-Directed Repair (HDR)

**Mechanism:** Template-based precise repair

**Template Design:**
```
Donor = [Left HA] + [Desired Edit] + [Right HA]
```

Where:
- Left/Right HA = Homology Arms (30-1000 bp)
- Shorter for ssODN (30-90 bp)
- Longer for plasmid (500-1000 bp)

**Efficiency Factors:**

| Factor | Effect | Optimal |
|--------|--------|---------|
| Cell cycle | S/G2 phase highest | Synchronize cells |
| HA length | Longer = better | 500-1000 bp |
| Template type | ssODN > dsDNA | Use ssODN for small edits |
| Strand | Non-target strand better | Asymmetric design |

**Enhancement Strategies:**
```
1. Cell cycle synchronization:
   - Nocodazole arrest (G2/M)
   - Release for S-phase enrichment

2. HDR pathway stimulation:
   - SCR7 (DNA ligase IV inhibitor)
   - RS-1 (RAD51 stimulator)

3. NHEJ suppression:
   - Cas9 nickase (reduces NHEJ)
   - Dn53BP1 expression
```

**Typical HDR Efficiency:** 1-20%

### 6.3 Base Editing

**Cytosine Base Editor (CBE):**
```
C → T conversion efficiency: 20-60%
Bystander editing: ~10-30% of cytosines in window
```

**Adenine Base Editor (ABE):**
```
A → G conversion efficiency: 30-70%
Bystander editing: ~5-15% of adenines in window
```

**Purity Calculation:**
```
Purity = (desired_edit / total_edits) × 100%
```

**Strategies for Single-Base Precision:**
```
1. Narrow editing window variants:
   - YE1-BE4 (positions 5-6)
   - ABE8e (positions 4-6)

2. gRNA design:
   - Position target base at window center
   - Avoid other editable bases in window
```

### 6.4 Prime Editing

**Editing Outcomes:**
- All base transitions: A↔G, C↔T
- All transversions: A↔T, A↔C, G↔T, G↔C
- Insertions: 1-44 bp
- Deletions: 1-80 bp
- Combinations

**Efficiency:**
```
PE_efficiency = (exact_edit / total_reads) × 100%
```

**Typical Range:** 10-40%

**Optimization:**
```
1. PBS length: 13 bp optimal
2. RT template: +3 extension beyond edit
3. PE3 strategy: Use nicking sgRNA
4. epegRNA: Extended scaffold (+tevopreQ1)
```

---

## 7. Validation Methods

### 7.1 T7 Endonuclease I (T7E1) Assay

**Principle:** T7E1 cleaves heteroduplex DNA at mismatches

**Protocol:**
```
1. PCR amplify target region (400-800 bp)
2. Denature and reanneal:
   95°C 5 min → 85°C to 25°C at -2°C/sec
3. T7E1 digestion:
   10 U enzyme, 37°C, 15-30 min
4. Run on 2% agarose gel
5. Quantify bands
```

**Calculation:**
```
Indel % = 100 × (1 - √(1 - fraction_cleaved))
```

Where:
```
fraction_cleaved = (b + c) / (a + b + c)
```
- a = uncleaved band
- b, c = cleavage products

**Sensitivity:** >5% editing

**Limitations:**
- Semi-quantitative
- Misses precise edits (HDR, base editing)

### 7.2 Sanger Sequencing

**Workflow:**
```
1. PCR amplify target region
2. Sanger sequence
3. Analyze chromatogram:
   - Look for double peaks (heterozygous)
   - Trace quality degradation (indels)
```

**Decomposition Tools:**
- TIDE (Tracking of Indels by Decomposition)
- ICE (Inference of CRISPR Edits)
- Synthego ICE Analysis

**Output:**
```json
{
  "WT": 35%,
  "+1A_insertion": 25%,
  "-3_deletion": 30%,
  "other": 10%
}
```

**Sensitivity:** >10% editing

### 7.3 Next-Generation Sequencing (NGS)

**Amplicon Sequencing:**
```
1. Design primers with adapters:
   P5-[barcode]-[locus-specific] (Forward)
   P7-[barcode]-[locus-specific] (Reverse)

2. PCR amplify (25-30 cycles)

3. Purify and quantify

4. Pool libraries (equimolar)

5. Sequence on Illumina (MiSeq/NextSeq)
   - Paired-end 150-250 bp
   - Coverage: >1000× per sample

6. Bioinformatic analysis:
   - Demultiplex by barcode
   - Align to reference
   - Call variants
   - Quantify editing outcomes
```

**Analysis Pipeline:**
```bash
# CRISPResso2 analysis
CRISPResso --fastq_r1 sample_R1.fastq.gz \
           --fastq_r2 sample_R2.fastq.gz \
           --amplicon_seq REFERENCE_SEQ \
           --guide_seq GUIDE_RNA_SEQ \
           --output_folder results/
```

**Output Metrics:**
- Editing efficiency
- Indel spectrum
- HDR efficiency
- Allele frequencies
- Off-target editing

**Sensitivity:** >0.1% editing

### 7.4 Functional Validation

**Protein Level:**
```
1. Western blot:
   - Confirm protein knockout
   - Detect size changes (in-frame indels)

2. Immunofluorescence:
   - Cellular localization
   - Expression levels

3. Flow cytometry:
   - Knockout efficiency
   - Reporter expression
```

**RNA Level:**
```
1. RT-qPCR:
   - Transcript abundance
   - Splice variants

2. RNA-seq:
   - Transcriptome changes
   - Isoform analysis
```

**Phenotypic Assays:**
```
1. Cell viability
2. Proliferation rate
3. Pathway activity (luciferase, etc.)
4. Disease-relevant phenotypes
```

---

## 8. Implementation Guidelines

### 8.1 Required Components

Any WIA-BIO-014 compliant system must include:

1. **gRNA Designer**: Algorithm for optimal gRNA selection
2. **Off-Target Predictor**: Genome-wide off-target search
3. **Delivery Optimizer**: Method selection based on cell type
4. **Efficiency Calculator**: Quantify editing outcomes
5. **Protocol Validator**: Check compliance with safety standards

### 8.2 API Interface

#### 8.2.1 Design Guide RNA
```typescript
interface GuideRNARequest {
  targetSequence: string;    // Including PAM
  pamType: 'NGG' | 'TTTV' | 'NNGRRT';
  organism: string;          // e.g., 'human', 'mouse'
  chromosome?: string;
  position?: number;
}

interface GuideRNAResponse {
  sequence: string;          // 20bp guide
  onTargetScore: number;     // 0-1
  gcContent: number;         // 0-100%
  predictedEfficiency: number; // 0-100%
  warnings: string[];
  offTargets: OffTargetSite[];
}
```

#### 8.2.2 Predict Off-Targets
```typescript
interface OffTargetRequest {
  guideRNA: string;
  genome: string;            // 'hg38', 'mm10', etc.
  maxMismatches: number;     // 0-4
  includeBulges: boolean;
}

interface OffTargetSite {
  chromosome: string;
  position: number;
  sequence: string;
  mismatches: number[];      // Positions of mismatches
  cfdScore: number;          // 0-1
  annotation: string;        // Gene/intergenic
  chromatinState: string;    // Open/closed
}
```

#### 8.2.3 Calculate Editing Efficiency
```typescript
interface EfficiencyRequest {
  editedReads: number;
  totalReads: number;
  editType: 'NHEJ' | 'HDR' | 'base_edit' | 'prime_edit';
}

interface EfficiencyResponse {
  efficiency: number;        // Percentage
  confidenceInterval: [number, number];
  quality: 'high' | 'medium' | 'low';
  recommendation: string;
}
```

### 8.3 Data Formats

#### 8.3.1 CRISPR Protocol JSON
```json
{
  "protocol_id": "CRISPR-2025-001",
  "version": "1.0",
  "target": {
    "gene": "BRCA1",
    "genome": "hg38",
    "chromosome": "chr17",
    "position": 43044295,
    "strand": "+"
  },
  "guide_rna": {
    "sequence": "ATCGATCGATCGATCGATCG",
    "pam": "NGG",
    "on_target_score": 0.72,
    "gc_content": 55
  },
  "cas_system": {
    "type": "SpCas9",
    "variant": "WT",
    "source": "recombinant"
  },
  "delivery": {
    "method": "RNP_electroporation",
    "dose": "500nM",
    "parameters": {
      "voltage": 1200,
      "pulse_duration": 20,
      "num_pulses": 2
    }
  },
  "validation": {
    "methods": ["T7E1", "NGS"],
    "coverage": 1000,
    "threshold": 0.05
  }
}
```

### 8.4 Error Handling

Standard error codes:

| Code | Meaning | Action |
|------|---------|--------|
| C001 | No PAM found | Use different Cas system |
| C002 | High off-target risk | Redesign gRNA |
| C003 | Low predicted efficiency | Select alternative target |
| C004 | Invalid sequence | Check input format |
| C005 | Delivery incompatible | Choose alternative method |
| C006 | Ethics violation | Abort and review |

---

## 9. Safety and Ethics

### 9.1 Biosafety

#### 9.1.1 Containment Level

**BSL-2 Minimum:**
- Human/mammalian cells
- Recombinant DNA
- Viral vectors

**BSL-3:**
- Pathogenic organisms
- High-risk viral vectors

#### 9.1.2 Waste Disposal

```
1. Autoclave:
   - Cell culture waste
   - Contaminated plastics
   - 121°C, 20 psi, 30 min

2. Chemical inactivation:
   - 10% bleach for 30 min
   - Lentivirus: 1% SDS + 10% bleach

3. Sharps disposal:
   - Biohazard sharps containers
   - Autoclave before disposal
```

### 9.2 Ethical Guidelines

#### 9.2.1 Somatic vs. Germline

**Somatic Editing (Allowed):**
- Therapeutic: cancer, genetic diseases
- Research: disease models
- Agriculture: crop improvement

**Germline Editing (Restricted):**
- Prohibited in most jurisdictions
- Requires special approval
- Must follow international guidelines:
  - National Academy of Sciences criteria
  - WHO recommendations
  - Local regulatory frameworks

#### 9.2.2 Research Ethics

**Requirements:**
```
1. IRB/Ethics approval:
   - Human subjects research
   - Clinical trials
   - Germline research

2. Informed consent:
   - Clear explanation of risks
   - Voluntary participation
   - Right to withdraw

3. Data privacy:
   - HIPAA compliance
   - Genetic information protection
   - Secure storage

4. Dual-use considerations:
   - Prevent bioweapons development
   - Responsible publication
   - Access restrictions
```

#### 9.2.3 Clinical Applications

**Regulatory Pathways:**
```
1. IND (Investigational New Drug):
   - FDA approval required
   - Safety data
   - Manufacturing details

2. Clinical Trials:
   - Phase I: Safety (n=10-30)
   - Phase II: Efficacy (n=30-100)
   - Phase III: Large-scale (n=100-1000)

3. Post-market surveillance:
   - Long-term follow-up
   - Adverse event reporting
   - Registry maintenance
```

### 9.3 Quality Control

#### 9.3.1 Reagent Validation

```
1. Cas9 protein:
   - Purity >95% (SDS-PAGE)
   - Activity assay (in vitro cleavage)
   - Endotoxin <10 EU/mg

2. Guide RNA:
   - Concentration (NanoDrop)
   - Integrity (denaturing gel)
   - Sequence verification

3. Viral vectors:
   - Titer (qPCR, ELISA)
   - Replication competence (RCL/RCA)
   - Sterility testing
```

#### 9.3.2 Process Validation

```
1. Editing efficiency:
   - Consistent ≥3 replicates
   - Defined acceptance criteria
   - Statistical analysis

2. Off-target analysis:
   - Top 10 predicted sites
   - Whole-genome if clinical
   - <0.1% off-target editing

3. Cell quality:
   - Viability >80%
   - Mycoplasma negative
   - Karyotype normal
```

---

## 10. References

### 10.1 Scientific Papers

1. Jinek, M. et al. (2012). "A programmable dual-RNA-guided DNA endonuclease"
2. Cong, L. et al. (2013). "Multiplex genome engineering using CRISPR/Cas systems"
3. Doench, J.G. et al. (2016). "Optimized sgRNA design to maximize activity"
4. Kleinstiver, B.P. et al. (2016). "High-fidelity CRISPR-Cas9 nucleases"
5. Anzalone, A.V. et al. (2019). "Search-and-replace genome editing"

### 10.2 Computational Tools

| Tool | URL | Purpose |
|------|-----|---------|
| CHOPCHOP | chopchop.cbu.uib.no | gRNA design |
| CRISPOR | crispor.org | Comprehensive design |
| Cas-OFFinder | cas-offinder.org | Off-target search |
| GUIDE-seq | guide-seq.com | Experimental off-targets |
| CRISPResso2 | crispresso.pinellolab.org | NGS analysis |

### 10.3 WIA Standards

- WIA-BIOBANK: Biological sample management
- WIA-GENOMICS: Genomic data formats
- WIA-LAB-AUTOMATION: Automated workflows
- WIA-INTENT: Intent-based design interfaces

---

## Appendix A: Example Calculations

### A.1 On-Target Score Calculation

```
Given gRNA: GAGTCCGAGCAGAAGAAGAA (NGG)

1. GC content:
   G+C = 11
   GC% = 11/20 × 100% = 55%

2. Position weights (simplified):
   Σ(position × nucleotide_score) = 0.65

3. Doench score:
   Score = sigmoid(0.65) = 0.66

Interpretation: High predicted activity
```

### A.2 Editing Efficiency

```
Given NGS data:
- Total reads: 10,000
- Reads with indels: 6,500

NHEJ efficiency = (6,500 / 10,000) × 100% = 65%

Breakdown:
- +1 insertion: 15% (1,500 reads)
- -1 deletion: 25% (2,500 reads)
- -3 deletion: 10% (1,000 reads)
- Other indels: 15% (1,500 reads)
- WT: 35% (3,500 reads)
```

### A.3 Off-Target CFD Score

```
Off-target site with 2 mismatches:
- Position 8: C→T (penalty = 0.3)
- Position 15: G→A (penalty = 0.6)

CFD = (1 - 0.3) × (1 - 0.6) = 0.7 × 0.4 = 0.28

Interpretation: Moderate off-target risk
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-BIO-014 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
