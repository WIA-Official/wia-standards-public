# WIA-SOIL-MICROBIOME Phase 3: Protocol Specification

**Version:** 1.0.0
**Date:** 2025-12-29
**Status:** Official
**Philosophy:** 弘益人間 (Benefit All Humanity)

## Overview

Phase 3 defines standardized protocols for soil sample collection, DNA extraction, sequencing, bioinformatics analysis, and Measurement, Reporting & Verification (MRV) procedures. These protocols ensure data quality, reproducibility, and comparability across laboratories and geographic regions.

## 1. Soil Sample Collection Protocol

### 1.1 Pre-Sampling Requirements

#### Equipment Checklist

| Item | Specification | Quantity | Sterilization |
|------|--------------|----------|---------------|
| Soil auger or probe | Stainless steel, 2.5 cm diameter | 1 per site | 70% ethanol between sites |
| Sample bags | Sterile, 500 mL capacity | 3-5 per site | Pre-sterilized |
| GPS device | ±2 m accuracy | 1 | N/A |
| Soil thermometer | -10°C to 60°C range | 1 | 70% ethanol |
| pH meter (field) | ±0.1 pH accuracy | 1 | Rinse with DI water |
| Moisture meter | 0-100% range | 1 | Clean between sites |
| Cooler with ice packs | Maintain 4°C | 1 | N/A |
| Field notebook | Waterproof | 1 | N/A |
| Gloves | Nitrile, powder-free | 1 pair per sample | N/A |
| Marking flags | Various colors | 10-20 | N/A |
| Shovel (optional) | For profile examination | 1 | Clean between sites |

#### Site Selection Criteria

1. **Representative areas**: Select locations representative of the management zone
2. **Avoid edges**: Maintain >10 m distance from field boundaries
3. **Avoid anomalies**: No sampling near:
   - Water bodies or drainage ditches
   - Equipment turning areas
   - Manure or fertilizer piles
   - Recently disturbed areas
   -异常植被区域
4. **Accessibility**: Ensure safe and legal access
5. **Documentation**: Photograph site before and after sampling

### 1.2 Sampling Design

#### Composite Sampling

Standard approach for field-scale characterization:

```
Sampling Pattern: W-Pattern or Grid
Number of sub-samples: 10-15 per composite
Area represented: 2-10 hectares
Depth: 0-15 cm (surface) and/or 15-30 cm (subsurface)
```

**W-Pattern Procedure:**
```
Field Layout:
    1     5     9
       3  ·  7    11
    2  ·  6  ·  10
       4     8    12
```

1. Walk a W-pattern across the field
2. Collect sub-samples at each numbered point
3. Mix thoroughly in clean bucket
4. Create 3 replicate composite samples (500g each)

#### Stratified Sampling

For precision agriculture or research:

```
Management zones: Sample each zone separately
Replicates per zone: 3-5
Sub-samples per replicate: 5-10
Spatial distribution: Random or systematic within zone
```

### 1.3 Sample Collection Procedure

#### Step-by-Step Protocol

**Step 1: Site Preparation**
1. Clear surface debris (leaves, stones, vegetation) from sampling point
2. If needed, remove top 2-3 cm of soil
3. Record GPS coordinates (WGS84 datum)
4. Take site photographs

**Step 2: Core Collection**
1. Insert auger/probe vertically to target depth
2. Extract core carefully to avoid contamination
3. For 0-15 cm depth:
   - Collect entire 0-15 cm section
   - Discard any surface contamination
4. For depth-specific samples:
   - Use depth markers on auger
   - Collect only target depth range

**Step 3: Sample Handling**
1. Place soil in sterile sample bag
2. Remove large roots, rocks, and debris
3. Break up large aggregates gently
4. For composite samples, combine in clean mixing container
5. Mix thoroughly using clean spatula or gloved hands

**Step 4: Metadata Recording**
```
Required Information:
- Sample ID (WSM-XX-YYYYMMDD-####)
- GPS coordinates (decimal degrees, 6 places)
- Elevation (meters)
- Sampling depth (cm)
- Time and date (ISO 8601 format)
- Weather conditions
- Soil temperature
- Field moisture (dry/moist/wet/saturated)
- Current crop and growth stage
- Days since last rainfall
- Days since last tillage/disturbance
- Recent amendments (type and date)
```

**Step 5: Storage and Transport**
1. Label sample bags immediately with waterproof marker
2. Place samples in cooler with ice packs (maintain 4°C)
3. Transport to laboratory within 24 hours
4. If delay >24 hours, freeze samples at -20°C

### 1.4 Quality Control for Sampling

#### Field Blanks

Collect 1 blank per 20 samples:
```
Procedure:
1. Open sterile sample bag in field
2. Close immediately without adding soil
3. Process identically to actual samples
4. Use to detect contamination
```

#### Equipment Blanks

After every 10 samples:
```
Procedure:
1. Swab sterilized auger with sterile swab
2. Place swab in sterile tube
3. Extract DNA and sequence
4. Verify no contamination from equipment
```

#### Replicate Samples

Collect 3 technical replicates per site:
```
Purpose: Assess sampling variability
Analysis: Calculate coefficient of variation
Acceptance criteria: CV < 20% for abundance metrics
```

### 1.5 Special Sample Types

#### Rhizosphere Sampling

For root-associated microbiomes:

```
Procedure:
1. Excavate entire plant with root system intact
2. Shake gently to remove bulk soil
3. Collect soil adhering to roots (1-2 mm layer)
4. Use sterile brush if needed
5. Store roots with adhering soil at 4°C
6. Process within 12 hours

Sample mass needed: 5-10 g rhizosphere soil
```

#### Aggregate Sampling

For soil structure analysis:

```
Procedure:
1. Collect undisturbed soil core
2. Gently break apart by hand
3. Separate aggregates by size:
   - Macroaggregates: >2000 μm
   - Microaggregates: 250-2000 μm
   - Silt+clay: <250 μm
4. Store each fraction separately

Sieving: Dry sieve or wet sieve (specify in metadata)
```

#### Deep Soil Sampling

For carbon sequestration studies:

```
Depths:
- 0-15 cm (surface)
- 15-30 cm (shallow subsurface)
- 30-60 cm (subsurface)
- 60-100 cm (deep subsurface)

Requirements:
- Hydraulic or mechanical soil probe
- Prevent cross-contamination between depths
- Record bulk density for each depth
```

## 2. DNA Extraction Protocol

### 2.1 Pre-Extraction Sample Preparation

#### Homogenization

```
Equipment: Mortar and pestle (sterilized) or bead beater
Temperature: Keep samples cold (4°C or on ice)
Procedure:
1. Sieve soil through 2 mm mesh
2. Remove large debris, roots, and rocks
3. Mix thoroughly to ensure uniform subsampling
4. Aliquot for DNA extraction
```

#### Sample Storage

| Storage Condition | Duration | DNA Degradation | Recommended Use |
|-------------------|----------|-----------------|-----------------|
| 4°C (refrigerated) | 1-7 days | Minimal | Short-term |
| -20°C (frozen) | 1-6 months | Low | Medium-term |
| -80°C (ultra-cold) | >1 year | Very low | Long-term |
| Freeze-dried | Years | Minimal if dry | Archival |
| Stabilization buffer | Months at RT | Low | Field collection |

### 2.2 DNA Extraction Methods

#### Method 1: CTAB-Based Extraction (Manual)

**Applications**: High humic acid soils, research applications
**DNA Yield**: 5-50 μg per 0.5 g soil
**Purity**: High (low humic acid contamination)

**Reagents:**
- CTAB buffer: 100 mM Tris-HCl pH 8.0, 1.4 M NaCl, 20 mM EDTA, 2% CTAB
- Phenol:chloroform:isoamyl alcohol (25:24:1)
- Chloroform:isoamyl alcohol (24:1)
- 10% CTAB in 0.7 M NaCl
- Isopropanol (ice-cold)
- 70% ethanol
- TE buffer pH 8.0

**Procedure:**
```
1. Add 0.5 g soil to 2 mL tube
2. Add 1 mL CTAB buffer + 2 μL β-mercaptoethanol
3. Vortex 30 seconds
4. Incubate 30 min at 65°C (vortex every 10 min)
5. Add 800 μL phenol:chloroform:isoamyl alcohol
6. Vortex 10 seconds, centrifuge 5 min at 13,000g
7. Transfer aqueous phase to new tube
8. Add 800 μL chloroform:isoamyl alcohol
9. Vortex, centrifuge 5 min at 13,000g
10. Transfer aqueous phase, add 100 μL 10% CTAB
11. Mix, incubate 10 min at 65°C
12. Add 800 μL chloroform:isoamyl alcohol
13. Vortex, centrifuge 5 min at 13,000g
14. Transfer aqueous phase, add 0.6 volumes isopropanol
15. Incubate 30 min at -20°C
16. Centrifuge 15 min at 13,000g
17. Discard supernatant, wash pellet with 70% ethanol
18. Air dry pellet 10-30 min
19. Resuspend in 50 μL TE buffer
20. Quantify and store at -20°C

Time required: 2-3 hours
Throughput: 24-48 samples per day
```

#### Method 2: Commercial Kit (DNeasy PowerSoil)

**Applications**: High-throughput, standardization
**DNA Yield**: 3-20 μg per 0.25 g soil
**Purity**: Good for most applications

**Procedure:**
```
Follow manufacturer's protocol with modifications:
1. Use 0.25 g soil (weigh accurately)
2. Bead beating: 5 min at maximum speed
3. Increase elution volume to 100 μL for low biomass soils
4. Optional: Repeat elution for maximum yield
5. Store at -20°C or -80°C

Time required: 1 hour
Throughput: 96 samples per day (with multichannel pipettes)
```

#### Method 3: Automated Extraction (KingFisher)

**Applications**: Very high throughput, clinical labs
**DNA Yield**: Similar to manual kits
**Purity**: Excellent consistency

```
Platform: Thermo Fisher KingFisher Flex
Protocol: PowerSoil-compatible magnetic bead protocol
Throughput: 96 samples in 1.5 hours
Advantages: Minimal hands-on time, excellent reproducibility
```

### 2.3 DNA Quality Control

#### Quantification

**Method 1: Fluorometry (Qubit)**
```
Assay: Qubit dsDNA BR (Broad Range) or HS (High Sensitivity)
Working range: 0.2-1000 ng/μL
Advantages: Specific for dsDNA, not affected by contaminants
Protocol: Follow manufacturer instructions
Acceptance: >5 ng/μL for PCR, >10 ng/μL for shotgun sequencing
```

**Method 2: Spectrophotometry (NanoDrop)**
```
Measurement: 260/280 ratio and 260/230 ratio
Acceptance criteria:
- 260/280 ratio: 1.8-2.0 (pure DNA)
- 260/230 ratio: 2.0-2.2 (low humic acid)
- If 260/230 < 1.7: Re-purify (AMPure beads)
Limitations: Overestimates concentration with contaminants
```

#### Purity Assessment

**Gel Electrophoresis:**
```
Gel: 0.8% agarose in TAE buffer
Voltage: 5 V/cm for 45 min
Staining: SYBR Safe or GelRed
Assessment:
- High molecular weight band (>10 kb): good quality
- Smearing: moderate degradation (acceptable)
- Low MW smear only: degraded (re-extract)
```

**260/280 and 260/230 Ratios:**

| Ratio | Value | Interpretation | Action |
|-------|-------|----------------|--------|
| 260/280 | >2.0 | RNA contamination | RNase treatment |
| 260/280 | 1.8-2.0 | Pure DNA | Acceptable |
| 260/280 | <1.8 | Protein/phenol contamination | Re-purify |
| 260/230 | >2.2 | Pure DNA | Acceptable |
| 260/230 | 1.8-2.2 | Slight contamination | Acceptable for most |
| 260/230 | <1.8 | Humic acid/salts | Re-purify (AMPure) |

#### PCR Inhibition Test

```
Test: qPCR with known template at multiple dilutions
Dilutions: 1:1, 1:10, 1:50, 1:100
Interpretation:
- Cq values increase proportionally: No inhibition
- Delayed or no amplification at 1:1: Inhibition present
- Solution: Dilute DNA or use BSA in PCR
```

### 2.4 DNA Storage and Handling

**Storage Conditions:**
```
Working stock (frequent use):
- Concentration: 5-20 ng/μL
- Temperature: -20°C
- Buffer: TE pH 8.0 or nuclease-free water
- Duration: 6-12 months

Long-term archive:
- Concentration: >50 ng/μL
- Temperature: -80°C
- Buffer: TE pH 8.0
- Aliquots: 10-20 μL to avoid freeze-thaw
- Duration: >5 years

Freeze-thaw cycles: Maximum 3 cycles recommended
```

## 3. DNA Sequencing Protocol

### 3.1 16S rRNA Gene Amplicon Sequencing

#### Primer Selection

**V3-V4 Region (Recommended for bacteria):**
```
Forward primer: 341F (5'-CCTACGGGNGGCWGCAG-3')
Reverse primer: 785R (5'-GACTACHVGGGTATCTAATCC-3')
Amplicon length: ~460 bp
Coverage: ~90% of bacteria and archaea
Platform: Illumina MiSeq (2×300 bp) or NextSeq
```

**V4 Region (Alternative):**
```
Forward primer: 515F (5'-GTGCCAGCMGCCGCGGTAA-3')
Reverse primer: 806R (5'-GGACTACHVGGGTWTCTAAT-3')
Amplicon length: ~290 bp
Coverage: >95% of bacteria and archaea
Platform: Illumina MiSeq (2×250 bp)
```

**Full-Length 16S (PacBio/Nanopore):**
```
Forward primer: 27F (5'-AGRGTTYGATYMTGGCTCAG-3')
Reverse primer: 1492R (5'-RGYTACCTTGTTACGACTT-3')
Amplicon length: ~1500 bp
Platform: PacBio Sequel or Oxford Nanopore
Advantages: Species-level resolution
```

#### PCR Amplification Protocol

**Two-Step PCR Approach:**

**Step 1: Target Amplification**
```
Reaction (25 μL):
- Template DNA: 1-10 ng (2 μL)
- Forward primer (10 μM): 0.5 μL
- Reverse primer (10 μM): 0.5 μL
- 2× KAPA HiFi HotStart ReadyMix: 12.5 μL
- Nuclease-free water: to 25 μL

Cycling conditions:
- Initial denaturation: 95°C for 3 min
- 25-30 cycles:
  - Denaturation: 95°C for 30 sec
  - Annealing: 55°C for 30 sec
  - Extension: 72°C for 30 sec
- Final extension: 72°C for 5 min
- Hold: 4°C

Quality check: 5 μL on 1.5% agarose gel
Expected: Single band at ~460 bp
```

**Step 2: Index PCR**
```
Reaction (25 μL):
- Step 1 PCR product: 5 μL
- Index primers (10 μM each): 5 μL
- 2× KAPA HiFi HotStart ReadyMix: 12.5 μL
- Nuclease-free water: 7.5 μL

Cycling conditions:
- 95°C for 3 min
- 8 cycles:
  - 95°C for 30 sec
  - 55°C for 30 sec
  - 72°C for 30 sec
- 72°C for 5 min
- Hold: 4°C
```

#### Library Preparation and Pooling

**Cleanup:**
```
Method: AMPure XP beads (0.8× ratio)
Procedure:
1. Add 20 μL beads to 25 μL PCR product
2. Incubate 5 min at room temperature
3. Place on magnetic stand, remove supernatant
4. Wash 2× with 200 μL 80% ethanol
5. Air dry 5-10 min
6. Elute in 25 μL nuclease-free water
```

**Quantification:**
```
Method: Qubit dsDNA HS assay
Target concentration: 5-20 ng/μL
Volume needed: Calculate for equimolar pooling
```

**Pooling:**
```
Target: Equal molarity from each sample
Calculation:
  - Convert ng/μL to nM using: nM = (ng/μL × 10^6) / (660 × bp)
  - For 460 bp: nM = ng/μL × 3.29
  - Pool to achieve 4 nM final concentration
  - Volume per sample = (4 nM × total volume) / (nM of sample)
```

**Final Library QC:**
```
Quantification: qPCR (KAPA Library Quant Kit)
Fragment analysis: Bioanalyzer or TapeStation
Expected size: ~550-600 bp (including adapters)
Acceptance: Single peak, no primer dimers (<200 bp)
```

#### Sequencing Run Setup

**Illumina MiSeq (V3 kit, 2×300 bp):**
```
Loading concentration: 8-12 pM
PhiX spike-in: 10-20% (for diversity)
Read configuration: Paired-end 300×300
Expected output: 20-25M reads per run
Samples per run: 96-384 (depending on depth)
Target depth: 20,000-50,000 reads per sample
```

**Quality metrics during run:**
```
Cluster density: 1000-1200 K/mm²
% bases ≥Q30: >70% for both reads
% PhiX aligned: 10-20%
Error rate: <1%
```

### 3.2 ITS Amplicon Sequencing (Fungi)

**Primer Set:**
```
ITS1 region:
- Forward: ITS1F (5'-CTTGGTCATTTAGAGGAAGTAA-3')
- Reverse: ITS2R (5'-GCTGCGTTCTTCATCGATGC-3')

ITS2 region (recommended):
- Forward: ITS3 (5'-GCATCGATGAAGAACGCAGC-3')
- Reverse: ITS4 (5'-TCCTCCGCTTATTGATATGC-3')

Platform: Illumina MiSeq 2×300 bp
Expected length: 200-600 bp (variable)
```

**Special considerations for ITS:**
```
- Variable amplicon length (200-600 bp)
- No size selection (to avoid bias)
- Higher PCR cycle number may be needed (30-35 cycles)
- Database: UNITE for classification
```

### 3.3 Shotgun Metagenomic Sequencing

**Library Preparation:**
```
Input DNA: 100-500 ng
Fragmentation: Covaris or enzymatic (Nextera)
Target fragment size: 350-500 bp
Library kit: Nextera XT or TruSeq DNA PCR-Free
Sequencing depth: 5-20 Gb per sample (10M-40M reads)
```

**Sequencing platform selection:**

| Platform | Read Length | Output/Run | Cost/Gb | Best For |
|----------|-------------|------------|---------|----------|
| Illumina NovaSeq | 2×150 bp | 1-6 Tb | $ | Large studies |
| Illumina NextSeq | 2×150 bp | 100-400 Gb | $$ | Medium studies |
| Illumina MiSeq | 2×300 bp | 15 Gb | $$$ | Small studies, long reads |
| PacBio Sequel II | >10 kb | 160 Gb | $$$$ | Long-read assembly |
| Oxford Nanopore | >10 kb | Variable | $$$ | Field sequencing, long reads |

**Quality requirements:**
```
DNA purity: 260/280 = 1.8-2.0
DNA integrity: >10 kb on gel
Library concentration: >2 nM
Insert size: 350-500 bp peak
% bases ≥Q30: >80%
```

## 4. Bioinformatics Analysis Protocol

### 4.1 16S rRNA Gene Analysis Pipeline

#### Quality Control and Preprocessing

**QIIME2 Pipeline (Recommended):**

```bash
# Import sequences
qiime tools import \
  --type 'SampleData[PairedEndSequencesWithQuality]' \
  --input-path raw_sequences/ \
  --output-path demux.qza \
  --input-format CasavaOneEightSingleLanePerSampleDirFmt

# Quality filtering and denoising (DADA2)
qiime dada2 denoise-paired \
  --i-demultiplexed-seqs demux.qza \
  --p-trim-left-f 0 \
  --p-trim-left-r 0 \
  --p-trunc-len-f 250 \
  --p-trunc-len-r 200 \
  --p-n-threads 16 \
  --o-table feature-table.qza \
  --o-representative-sequences rep-seqs.qza \
  --o-denoising-stats denoising-stats.qza

# Taxonomic classification
qiime feature-classifier classify-sklearn \
  --i-classifier silva-138-99-nb-classifier.qza \
  --i-reads rep-seqs.qza \
  --p-confidence 0.8 \
  --p-n-jobs 16 \
  --o-classification taxonomy.qza

# Filter out chloroplast, mitochondria
qiime taxa filter-table \
  --i-table feature-table.qza \
  --i-taxonomy taxonomy.qza \
  --p-exclude chloroplast,mitochondria \
  --o-filtered-table feature-table-filtered.qza

# Diversity analyses
qiime diversity core-metrics-phylogenetic \
  --i-phylogeny rooted-tree.qza \
  --i-table feature-table-filtered.qza \
  --p-sampling-depth 10000 \
  --m-metadata-file metadata.tsv \
  --output-dir diversity-results/
```

**Quality Control Thresholds:**

| Metric | Threshold | Action if Failed |
|--------|-----------|------------------|
| Raw read count | >10,000 | Re-sequence sample |
| % reads passing filter | >70% | Check for contamination |
| Merged read count | >5,000 | Adjust truncation parameters |
| % non-chimeric | >80% | Check for contamination |
| % assigned taxonomy (genus) | >70% | Check database version |
| Good's coverage | >0.95 | Increase sequencing depth |

#### Diversity Metrics

**Alpha Diversity (Within-Sample):**
```
Observed OTUs/ASVs: Richness measure
Shannon index: Diversity incorporating evenness
Simpson index: Probability two sequences are same species
Chao1: Richness estimator (accounts for rare taxa)
ACE: Another richness estimator
Faith's PD: Phylogenetic diversity

Rarefaction: Normalize to equal depth (min depth across samples)
Statistical tests: Kruskal-Wallis, Mann-Whitney U
```

**Beta Diversity (Between-Sample):**
```
Bray-Curtis: Abundance-weighted dissimilarity
Jaccard: Presence/absence dissimilarity
Weighted UniFrac: Phylogenetic, abundance-weighted
Unweighted UniFrac: Phylogenetic, presence/absence

Ordination: PCoA, NMDS, PCA
Statistical tests: PERMANOVA, ANOSIM, MRPP
```

### 4.2 Shotgun Metagenomics Analysis

**Quality Control:**
```bash
# Trim adapters and low-quality bases (Trimmomatic)
trimmomatic PE -threads 16 \
  reads_R1.fastq.gz reads_R2.fastq.gz \
  R1_trimmed.fastq.gz R1_unpaired.fastq.gz \
  R2_trimmed.fastq.gz R2_unpaired.fastq.gz \
  ILLUMINACLIP:adapters.fa:2:30:10 \
  LEADING:3 TRAILING:3 SLIDINGWINDOW:4:20 MINLEN:50

# Remove host contamination (Bowtie2)
bowtie2 -x human_genome -1 R1_trimmed.fastq.gz -2 R2_trimmed.fastq.gz \
  --un-conc nonhost_reads.fastq.gz -p 16
```

**Taxonomic Profiling:**
```bash
# MetaPhlAn4
metaphlan nonhost_reads_R1.fastq.gz,nonhost_reads_R2.fastq.gz \
  --input_type fastq \
  --nproc 16 \
  --bowtie2out metagenome.bowtie2.bz2 \
  -o taxonomic_profile.txt

# Kraken2 + Bracken
kraken2 --db kraken2_db --threads 16 \
  --paired nonhost_reads_R1.fastq.gz nonhost_reads_R2.fastq.gz \
  --report kraken2_report.txt --output kraken2_output.txt

bracken -d kraken2_db -i kraken2_report.txt \
  -o bracken_species.txt -l S
```

**Functional Profiling:**
```bash
# HUMAnN3
humann --input nonhost_reads.fastq.gz \
  --output humann_output/ \
  --threads 16 \
  --nucleotide-database chocophlan \
  --protein-database uniref90

# Normalize to relative abundance
humann_renorm_table --input humann_output/genefamilies.tsv \
  --output humann_output/genefamilies_relab.tsv \
  --units relab

# Regroup to pathways
humann_regroup_table --input humann_output/genefamilies.tsv \
  --groups uniref90_ko --output humann_output/ko_families.tsv
```

### 4.3 Statistical Analysis Requirements

**Minimum Requirements:**

```
Sample size: n ≥ 3 per group (n ≥ 5 recommended)
Rarefaction: To minimum depth across samples
Normalization: TSS, CSS, or DESeq2 normalization
Multiple testing correction: FDR (Benjamini-Hochberg)
Significance threshold: P < 0.05 (adjusted)
Effect size reporting: Required (Cohen's d, fold-change)
```

**Differential Abundance Testing:**

| Tool | Best For | Input | Output |
|------|----------|-------|--------|
| DESeq2 | Count data, few samples | ASV/OTU table | Log2 fold-change, P-values |
| edgeR | Count data, more samples | ASV/OTU table | Log2 fold-change, P-values |
| ANCOM-BC | Compositional data | ASV/OTU table | Bias-corrected abundances |
| LEfSe | Biomarker discovery | Taxonomic table | LDA scores |
| ALDEx2 | Compositional data | ASV/OTU table | Effect sizes |

## 5. Measurement, Reporting & Verification (MRV) Protocol

### 5.1 Quality Assurance Levels

#### Level 1: Basic QA (Minimum Standard)

```
Sample Collection:
✓ GPS coordinates recorded (±10 m)
✓ Sampling depth documented
✓ Field notes completed
✓ Samples stored at 4°C

DNA Extraction:
✓ Commercial kit used
✓ DNA quantified (Qubit or equivalent)
✓ 260/280 ratio checked

Sequencing:
✓ Read depth ≥10,000 per sample
✓ Negative controls included
✓ Raw data deposited in public repository

Analysis:
✓ Standard pipeline used (QIIME2, etc.)
✓ Rarefaction performed
✓ Alpha and beta diversity calculated
```

#### Level 2: Standard QA (Recommended)

```
All Level 1 requirements plus:

Sample Collection:
✓ GPS coordinates ±2 m accuracy
✓ Soil properties measured (pH, texture, OM)
✓ Weather data recorded
✓ Site photographs taken
✓ Field blanks collected (1 per 20 samples)

DNA Extraction:
✓ Extraction blanks included
✓ 260/230 ratio checked
✓ Gel electrophoresis performed
✓ PCR inhibition tested

Sequencing:
✓ Read depth ≥20,000 per sample
✓ Mock communities included
✓ PhiX spike-in (10-20%)
✓ Sequencing QC metrics pass (Q30 >70%)

Analysis:
✓ Chimera checking performed
✓ Contamination removed (chloroplast, mitochondria)
✓ Good's coverage ≥95%
✓ Statistics include effect sizes
✓ Code and parameters documented
```

#### Level 3: Research Grade (Full Validation)

```
All Level 2 requirements plus:

Sample Collection:
✓ Stratified or systematic sampling design
✓ Technical replicates (n≥3)
✓ Full soil characterization (texture, chemistry, physical)
✓ Environmental monitoring (temperature, moisture)
✓ Land use history documented
✓ Equipment blanks collected

DNA Extraction:
✓ Multiple extraction methods compared
✓ DNA integrity assessed (gel + quantification)
✓ Storage conditions validated
✓ Spike-in controls used

Sequencing:
✓ Read depth ≥50,000 per sample
✓ Multiple mock communities (different compositions)
✓ Technical sequencing replicates
✓ Multiple sequencing platforms (if applicable)

Analysis:
✓ Multiple bioinformatics pipelines compared
✓ Parameter sensitivity analysis
✓ Batch effects assessed and corrected
✓ Reproducibility analysis
✓ Complete computational workflow archived
✓ Peer review of analysis code
```

### 5.2 Negative Controls

**Extraction Blank:**
```
Purpose: Detect contamination from extraction reagents
Procedure:
1. Process nuclease-free water through extraction protocol
2. Include 1 blank per extraction batch (24-96 samples)
3. Quantify DNA (should be <0.1 ng/μL)
4. Sequence at same depth as samples
5. Remove any contaminating taxa from actual samples

Acceptance criteria:
- DNA yield <5% of average sample yield
- Read count <5% of average sample reads
- Shared taxa removed from samples
```

**PCR Blank:**
```
Purpose: Detect contamination from PCR reagents
Procedure:
1. Include nuclease-free water in PCR setup
2. One blank per PCR plate
3. Amplify and sequence with samples

Acceptance criteria:
- No visible band on gel
- Read count <1% of average sample
```

**Field Blank:**
```
Purpose: Detect contamination during field sampling
Procedure:
1. Open sterile sample bag in field
2. Close without adding soil
3. Process identically to samples

Acceptance criteria:
- DNA yield <1% of average sample yield
```

### 5.3 Positive Controls

**Mock Community:**
```
Composition: Defined mixture of bacterial strains
Commercial options:
- ZymoBIOMICS Microbial Community Standard
- ATCC MSA-1000
- Custom mixtures

Expected composition (example):
- Pseudomonas aeruginosa: 12%
- Escherichia coli: 12%
- Salmonella enterica: 12%
- Lactobacillus fermentum: 12%
- Enterococcus faecalis: 12%
- Staphylococcus aureus: 12%
- Listeria monocytogenes: 12%
- Bacillus subtilis: 12%

Acceptance criteria:
- Observed vs expected composition within ±5%
- All species detected
- No false positives
```

### 5.4 Data Validation Checklist

```
☐ Sample metadata complete (all required fields)
☐ GPS coordinates within valid range
☐ Soil properties within plausible ranges
☐ Sample IDs unique and follow format
☐ Negative controls DNA yield <5% of samples
☐ Mock community composition within ±5% of expected
☐ Read depth ≥minimum threshold
☐ Q30 score ≥70%
☐ Chimera removal performed
☐ Contamination filtering performed
☐ Rarefaction curves plateau
☐ Good's coverage ≥95%
☐ Statistical tests appropriate for data type
☐ Multiple testing correction applied
☐ Effect sizes reported
☐ Raw data deposited in public repository
☐ Analysis code available
☐ Reproducible workflow documented
```

### 5.5 Reporting Standards

**Minimum Information Reporting:**

```
1. Sample Metadata (MIxS compliant):
   - Geographic location (decimal degrees)
   - Sampling date and time
   - Depth
   - Land use and management
   - Soil properties (texture, pH, OM)

2. DNA Extraction:
   - Method and kit name/version
   - Input soil mass
   - DNA yield and quality metrics

3. Sequencing:
   - Platform and chemistry
   - Target gene and primers
   - Read length and configuration
   - Number of reads per sample
   - Quality metrics (Q30, error rate)

4. Bioinformatics:
   - Pipeline and version
   - Database and version
   - Key parameter settings
   - Quality filtering criteria

5. Statistics:
   - Sample size per group
   - Statistical tests used
   - P-values (adjusted)
   - Effect sizes

6. Data Availability:
   - Repository accession numbers
   - Code repository link
   - Supplementary data location
```

### 5.6 Data Deposition Requirements

**Public Repository Submission:**

```
Raw Sequencing Data:
- NCBI SRA (Sequence Read Archive)
- ENA (European Nucleotide Archive)
- DDBJ (DNA Data Bank of Japan)

Format: FASTQ files
Metadata: BioSample/BioProject
Embargo: Maximum 1 year from publication

Processed Data:
- Qiita (microbiome studies)
- MG-RAST (metagenomics)
- FigShare, Zenodo (supplementary)

Include:
- Feature table (ASV/OTU table)
- Taxonomy table
- Phylogenetic tree
- Metadata table
- Diversity metrics
```

### 5.7 Verification Audit Procedure

**Third-Party Verification Steps:**

```
1. Documentation Review:
   ☐ Field notes complete and consistent
   ☐ Chain of custody documented
   ☐ QC data within acceptance criteria
   ☐ Deviations from protocol documented

2. Data Integrity Check:
   ☐ Sample IDs match across datasets
   ☐ No duplicate sample IDs
   ☐ Negative controls meet criteria
   ☐ Positive controls meet criteria
   ☐ Statistical analyses appropriate

3. Reproducibility Test:
   ☐ Re-run bioinformatics pipeline
   ☐ Verify same results obtained
   ☐ Check for batch effects
   ☐ Validate reported statistics

4. Spot Checks:
   ☐ Re-extract 5% of samples
   ☐ Re-sequence 2% of samples
   ☐ Compare to original results
   ☐ Correlation >0.9 required

5. Certification:
   ☐ All checks passed
   ☐ Non-conformances documented
   ☐ Corrective actions implemented
   ☐ Certificate of analysis issued
```

---

**© 2025 SmileStory Inc. / WIA - World Certification Industry Association**
**弘益人間 · Benefit All Humanity**
