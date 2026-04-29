# WIA-GENOME_EDITING: Phase 3 - Protocol Specification

**Version:** 1.0
**Status:** FULL Implementation
**Last Updated:** 2026-01-12

---

## 1. Overview

This specification defines standardized protocols for genome editing operations, including experimental workflows, clinical procedures, quality control, safety validation, and regulatory compliance for CRISPR-Cas9, base editing, prime editing, and gene therapy applications.

### 1.1 Protocol Scope

- **Research Protocols**: In vitro and in vivo editing
- **Clinical Protocols**: Gene therapy procedures (Casgevy and similar)
- **Manufacturing Protocols**: Cell therapy production
- **Quality Control**: Validation and safety assessment
- **Regulatory Compliance**: FDA, EMA, and international standards

---

## 2. CRISPR-Cas9 Editing Protocol

### 2.1 Guide RNA Design Protocol

**Protocol ID:** CRISPR-DESIGN-001
**Version:** 1.0

#### Objectives
- Design highly specific guide RNAs for target gene editing
- Minimize off-target effects
- Maximize on-target efficiency

#### Materials
- Target DNA sequence (reference genome GRCh38.p14)
- CRISPOR or Benchling design software
- Off-target prediction tools (Cas-OFFinder, GUIDE-seq)

#### Procedure

**Step 1: Target Selection**
```
1.1 Identify target gene and mutation site
    Example: HBB gene, position chr11:5227021 (HbS mutation)

1.2 Extract genomic sequence (±500 bp from target)

1.3 Annotate sequence features:
    - Exons/introns
    - Regulatory elements
    - Known polymorphisms
    - Conservation scores
```

**Step 2: Guide RNA Design**
```
2.1 Input target sequence into CRISPOR
    - Organism: Homo sapiens
    - Genome: GRCh38
    - PAM: NGG (for SpCas9)

2.2 Generate candidate gRNAs (minimum 10)

2.3 Score each gRNA:
    - Specificity score (target ≥95)
    - On-target score (target ≥90)
    - Off-target count (target ≤5)
    - Predicted efficiency (target ≥80%)

2.4 Select top 3 candidates for experimental validation
```

**Step 3: Off-Target Analysis**
```
3.1 Run Cas-OFFinder for each candidate
    Parameters:
    - Max mismatches: 4
    - Max bulge size: 2
    - Reference: GRCh38

3.2 Analyze off-target sites:
    - Count total sites
    - Classify by risk (high/medium/low)
    - Check proximity to oncogenes/tumor suppressors
    - Evaluate functional impact

3.3 Document all sites with ≥3 mismatches
```

**Step 4: Final Selection**
```
4.1 Rank candidates by composite score:
    Score = (specificity × 0.4) + (on-target × 0.3) +
            (efficiency × 0.2) + (safety × 0.1)

4.2 Select highest-scoring gRNA

4.3 Design control gRNA (non-targeting)

4.4 Document design rationale
```

#### Quality Control
- Verify PAM sequence compatibility
- Confirm no common SNPs in target region
- Check GC content (40-60% optimal)
- Validate in silico predictions

#### Expected Results
- Primary gRNA with specificity ≥95%, efficiency ≥80%
- Comprehensive off-target map
- Risk assessment report

---

### 2.2 In Vitro CRISPR Editing Protocol

**Protocol ID:** CRISPR-INVITRO-001
**Version:** 1.0

#### Objectives
- Perform CRISPR-Cas9 editing in cultured cells
- Validate editing efficiency and specificity
- Assess off-target effects

#### Materials
- Target cells (e.g., K562, HEK293T, primary CD34+ HSPCs)
- Cas9 protein or plasmid (SpCas9, Alt-R Cas9)
- Synthetic guide RNA or gRNA expression plasmid
- Electroporation system (Lonza 4D-Nucleofector)
- Cell culture reagents (IMDM, X-VIVO 15, cytokines)

#### Procedure

**Day 0: Cell Preparation**
```
1.1 Culture target cells to 70-80% confluence
    For HSPCs: Thaw and pre-stimulate 48h with:
    - SCF (100 ng/mL)
    - TPO (100 ng/mL)
    - Flt3-L (100 ng/mL)
    - IL-3 (20 ng/mL)

1.2 Count cells and assess viability (target ≥90%)

1.3 Prepare 2×10^6 cells per condition
```

**Day 0: RNP Complex Assembly**
```
2.1 Prepare Cas9-gRNA ribonucleoprotein (RNP):
    - Cas9 protein: 100 pmol
    - Guide RNA: 120 pmol (1.2:1 ratio)
    - Incubate 15 min at room temperature

2.2 Quality check:
    - Verify RNP formation by gel shift assay
    - Confirm gRNA integrity (bioanalyzer)
```

**Day 0: Electroporation**
```
3.1 Wash cells with PBS (no calcium/magnesium)

3.2 Resuspend 2×10^6 cells in 20 μL nucleofection buffer

3.3 Add RNP complex, mix gently

3.4 Transfer to electroporation cuvette

3.5 Electroporate using optimized program:
    - For HSPCs: EO-100 program
    - For K562: FF-120 program

3.6 Immediately add 80 μL pre-warmed media

3.7 Transfer to culture plate
```

**Day 0-2: Recovery**
```
4.1 Incubate at 37°C, 5% CO2

4.2 Monitor cell viability daily

4.3 Add fresh media if needed (do not passage)
```

**Day 2-7: Analysis**
```
5.1 Harvest cells for genomic DNA extraction

5.2 PCR amplify target region:
    - Primers: ±300 bp from cut site
    - High-fidelity polymerase
    - 30-35 cycles

5.3 Analyze editing outcomes:
    - Sanger sequencing + TIDE analysis
    - NGS (target depth: 50,000 reads)
    - ICE analysis or CRISPResso2

5.4 Calculate editing efficiency:
    Efficiency = (edited reads / total reads) × 100

5.5 Characterize indel spectrum:
    - Insertions vs deletions
    - Size distribution
    - In-frame vs frameshift
```

**Off-Target Analysis**
```
6.1 Amplify predicted off-target sites (top 10)

6.2 Deep sequence each site (depth: 10,000 reads)

6.3 Calculate off-target frequency:
    OT frequency = (edited reads / total reads) × 100

6.4 Compare to on-target efficiency
```

#### Quality Control
- Cell viability post-electroporation ≥70%
- DNA quality (260/280 ratio: 1.8-2.0)
- PCR amplicon single band on gel
- Sequencing coverage ≥10,000 reads per site

#### Expected Results
- On-target editing: 50-90%
- Indel frequency: 60-85% of on-target
- Off-target editing: <1% of on-target
- Cell viability: 75-95%

---

## 3. Base Editing Protocol

### 3.1 Adenine Base Editor (ABE) Protocol

**Protocol ID:** ABE-EDIT-001
**Version:** 1.0

#### Objectives
- Perform precise A-to-G base editing
- Correct pathogenic mutations (e.g., HbS E6V)
- Minimize bystander editing and off-target effects

#### Materials
- ABE8.20-m protein or mRNA
- Guide RNA for target site
- Target cells (CD34+ HSPCs for therapeutic application)
- Electroporation system

#### Procedure

**Day -2: Cell Preparation**
```
1.1 Thaw or isolate CD34+ HSPCs from:
    - Patient apheresis product (for autologous therapy)
    - Healthy donor (for research)

1.2 Culture in X-VIVO 15 media with cytokines:
    - SCF: 100 ng/mL
    - TPO: 100 ng/mL
    - Flt3-L: 100 ng/mL
    - IL-3: 20 ng/mL

1.3 Pre-stimulate for 48 hours
```

**Day 0: ABE RNP Assembly**
```
2.1 Prepare ABE8.20-m RNP:
    - ABE protein: 100 pmol
    - Guide RNA: 120 pmol
    - Ratio: 1:1.2 (protein:RNA)
    - Incubate: 15 min at RT

2.2 For HBB E6V correction:
    gRNA sequence: 5'-GTGCACCTGACTCCTGAGGAGAA-3'
    Target position: chr11:5227021
    Target base: A (GAG codon)
    Edited base: G (GTG codon)
    Editing window: positions 4-8
```

**Day 0: Electroporation**
```
3.1 Wash 2×10^6 HSPCs with PBS

3.2 Resuspend in 20 μL P3 buffer

3.3 Add ABE RNP complex

3.4 Electroporate: EO-100 program

3.5 Recovery in pre-warmed X-VIVO 15 + cytokines
```

**Day 0-7: Culture and Monitoring**
```
4.1 Culture at 37°C, 5% CO2

4.2 Monitor daily:
    - Cell count
    - Viability (target ≥80%)
    - Morphology

4.3 Passage if cell density >1×10^6/mL
```

**Day 7: Editing Analysis**
```
5.1 Extract genomic DNA (DNeasy Blood & Tissue Kit)

5.2 Amplify HBB target region:
    Forward: 5'-ACACAACTGTGTTCACTAGC-3'
    Reverse: 5'-CAACTTCATCCACGTTCACC-3'
    Product size: ~500 bp

5.3 Deep sequencing (Illumina MiSeq):
    - Target depth: 50,000 reads
    - Paired-end 2×150 bp

5.4 Analyze with CRISPResso2:
    - On-target A>G editing frequency
    - Bystander editing frequency
    - Indel frequency
    - Product purity

5.5 Calculate outcomes:
    Primary edit (E6V correction): Target ≥60%
    Bystander edits: Target ≤10%
    Indels: Target ≤5%
    Product purity: Target ≥85%
```

**Off-Target Validation**
```
6.1 Amplify top 10 predicted off-target sites

6.2 Deep sequence (depth: 10,000 reads each)

6.3 Calculate off-target editing:
    Frequency = (edited reads / total reads) × 100

6.4 Risk assessment:
    - High risk: >10% editing in coding region
    - Medium risk: 1-10% editing in coding region
    - Low risk: <1% editing or non-coding region
```

#### Quality Control
- ABE protein purity ≥95% (SDS-PAGE)
- gRNA integrity verified (bioanalyzer)
- Cell viability post-electroporation ≥70%
- Sequencing depth ≥50,000 reads
- No contamination in negative controls

#### Expected Results
- On-target A>G editing: 50-80%
- Product purity: 85-95%
- Bystander editing: 5-15%
- Indels: 1-5%
- Cell viability: 75-90%

---

### 3.2 Cytosine Base Editor (CBE) Protocol

**Protocol ID:** CBE-EDIT-001
**Version:** 1.0

#### Objectives
- Perform C-to-T base editing
- Introduce stop codons or correct C>T mutations
- Optimize editing window and minimize bystander effects

#### Materials
- BE4max or AncBE4max protein/mRNA
- Target-specific guide RNA
- Uracil glycosylase inhibitor (UGI)
- Target cells

#### Procedure
[Similar structure to ABE protocol with CBE-specific parameters]

**Key Differences:**
- CBE targets C bases in editing window (positions 4-8)
- Generates C>T (or C>G>A) transitions
- UGI fusion enhances C>T product purity
- Consider potential bystander C editing

---

## 4. Prime Editing Protocol

### 4.1 Prime Editor 5 (PE5) Protocol

**Protocol ID:** PE5-EDIT-001
**Version:** 1.0

#### Objectives
- Perform precise substitutions, insertions, or deletions
- Install specific sequences without DSBs
- Achieve high precision with minimal byproducts

#### Materials
- PE5 system components:
  - PE2 protein (Cas9-H840A nickase + RT)
  - pegRNA (prime editing guide RNA)
  - ngRNA (nicking guide RNA)
- MLH1dn protein (mismatch repair inhibitor)
- Target cells
- Electroporation system

#### Procedure

**Day -2: pegRNA and ngRNA Design**
```
1.1 Design pegRNA:
    Components:
    - Spacer (20 nt): guides PE to target
    - PBS (primer binding site, 10-15 nt): anneals to nicked strand
    - RT template (10-30 nt): encodes desired edit

    For HBB E6V correction:
    Spacer: GTGCACCTGACTCCTGAGGA
    PBS (13 nt): TCCTCAGGAGTCA
    RT template (16 nt): GTGCACCTGACTCCTG
    Edit position: 5 (within RT template)

1.2 Design ngRNA:
    - Nicking guide on non-edited strand
    - Distance from pegRNA: 40-90 bp (optimal ~65 bp)
    - Orientation: opposite strand from pegRNA

    ngRNA sequence: CTGGGCAGGTTGGTATCAAG
    Nick position: +65 bp from pegRNA
```

**Day 0: PE5 RNP Assembly**
```
2.1 Assemble PE2-pegRNA complex:
    - PE2 protein: 100 pmol
    - pegRNA: 120 pmol
    - Incubate: 15 min at RT

2.2 Prepare ngRNA RNP:
    - PE2 protein: 100 pmol
    - ngRNA: 120 pmol
    - Incubate: 15 min at RT

2.3 Add MLH1dn protein:
    - MLH1dn: 50 pmol
    - Ratio: 1:1:0.5 (PE2-pegRNA:PE2-ngRNA:MLH1dn)
```

**Day 0: Electroporation**
```
3.1 Prepare 2×10^6 target cells

3.2 Resuspend in nucleofection buffer

3.3 Add PE5 RNP mixture

3.4 Electroporate with optimized program

3.5 Recovery in culture media
```

**Day 0-7: Culture**
```
4.1 Culture at 37°C, 5% CO2

4.2 Monitor viability and proliferation

4.3 Peak editing typically observed at day 5-7
```

**Day 7: Editing Analysis**
```
5.1 Extract genomic DNA

5.2 Amplify target region (±300 bp)

5.3 Deep sequence (depth: 50,000 reads)

5.4 Analyze editing outcomes:
    - Precise edit frequency (install rate)
    - Indel frequency
    - Scaffold integration
    - Other byproducts

5.5 Calculate metrics:
    Efficiency = precise edits / total alleles
    Precision = precise edits / (precise edits + byproducts)
    Install rate: Target ≥40%
    Indel rate: Target ≤5%
    Precision: Target ≥90%
```

#### Quality Control
- pegRNA and ngRNA integrity (bioanalyzer)
- PE2 protein activity (in vitro RT assay)
- Sequencing depth ≥50,000 reads
- Multiple biological replicates (n≥3)

#### Expected Results
- Precise editing: 30-60%
- Precision: 90-98%
- Indels: 1-5%
- Other byproducts: 2-8%

---

## 5. Clinical Gene Therapy Protocol (Casgevy)

### 5.1 Manufacturing Protocol

**Protocol ID:** CASGEVY-MFG-001
**Version:** 2.0

#### Regulatory Status
- FDA Approved: January 16, 2024 (BLA 125755)
- EMA Approved: December 14, 2023
- Indication: Transfusion-dependent β-thalassemia (TDT) and severe sickle cell disease (SCD)

#### Mechanism
- Ex vivo CRISPR-Cas9 editing of autologous CD34+ HSPCs
- Target: BCL11A erythroid-specific enhancer
- Effect: Reactivation of fetal hemoglobin (HbF)
- Outcome: Clinical benefit without transfusions

#### Materials
- Patient autologous CD34+ hematopoietic stem/progenitor cells
- Cas9 nuclease (SpCas9)
- BCL11A-targeting guide RNA
- Electroporation system (MaxCyte GT)
- GMP-grade reagents and consumables

#### Procedure

**Phase 1: Mobilization and Apheresis (Days -120 to -90)**
```
1.1 Patient mobilization:
    Agent: Plerixafor (CXCR4 antagonist)
    Dose: 0.24 mg/kg subcutaneous
    Schedule: Daily for 4-5 days

1.2 Monitor CD34+ cell mobilization:
    Target: ≥20 CD34+ cells/μL peripheral blood

1.3 Perform apheresis:
    Sessions: 1-3 (based on mobilization)
    Target collection: ≥6×10^6 CD34+/kg patient body weight

1.4 Cryopreserve apheresis product:
    Freeze in 10% DMSO + autologous plasma
    Store in vapor phase liquid nitrogen
```

**Phase 2: Cell Processing and Editing (Days -90 to -30)**
```
2.1 Thaw apheresis product at GMP facility

2.2 Enrich CD34+ cells:
    Method: CliniMACS CD34+ selection
    Purity target: ≥90% CD34+
    Recovery target: ≥70%
    Quality check: Flow cytometry

2.3 Pre-stimulate cells (48 hours):
    Media: X-VIVO 15
    Cytokines:
    - SCF: 300 ng/mL
    - TPO: 300 ng/mL
    - Flt3-L: 300 ng/mL
    - IL-3: 60 ng/mL

2.4 Prepare Cas9 RNP:
    Cas9 protein: Clinical-grade SpCas9
    Guide RNA: BCL11A enhancer-targeting
    Sequence: [Proprietary - Vertex/CRISPR Therapeutics]
    Incubation: 15 min at RT

2.5 Electroporation (MaxCyte GT):
    Cell dose: 20-40×10^6 cells/mL
    RNP dose: Optimized ratio
    Program: Clinical protocol
    Perform in closed system

2.6 Post-editing culture (1-2 days):
    Allow recovery and initial editing
    Monitor viability (target ≥70%)

2.7 Wash and formulate final product:
    Cryopreservation medium: 5% DMSO
    Fill in cryobags
    Total dose: 2-10×10^6 CD34+/kg

2.8 Quality control release testing:
    - Cell count and viability (≥70%)
    - CD34+ purity (≥80%)
    - Editing efficiency (≥60% by NGS)
    - Sterility (14-day culture)
    - Endotoxin (<5 EU/kg)
    - Mycoplasma (negative by PCR)
    - RCL (replication competent lentivirus, N/A for this product)

2.9 Cryopreserve drug product until treatment
```

**Phase 3: Conditioning and Infusion (Days -7 to 0)**
```
3.1 Admit patient for conditioning

3.2 Administer myeloablative conditioning:
    Agent: Busulfan
    Dosing: Targeted AUC (area under curve)
    Target AUC: 20,000-24,000 μM·min
    Schedule: Days -7 to -4
    Monitoring: PK-guided dose adjustments

3.3 Rest period: Days -3 to -1

3.4 Day 0 - Infusion of Casgevy:
    3.4.1 Thaw drug product at bedside
    3.4.2 Infuse without washing
    3.4.3 Cell dose: 2-10×10^6 CD34+/kg
    3.4.4 Infusion time: 15-60 minutes
    3.4.5 Monitor for reactions
```

**Phase 4: Post-Transplant Care (Days +1 to +28)**
```
4.1 Supportive care:
    - Antimicrobial prophylaxis
    - Transfusion support (RBC, platelets)
    - Growth factor support (G-CSF)
    - Nutritional support

4.2 Monitor for engraftment:
    - Daily CBC with differential
    - Neutrophil recovery (ANC >500/μL)
    - Platelet recovery (>20,000/μL)

4.3 Expected engraftment:
    - Neutrophil engraftment: Day +15 to +30
    - Platelet engraftment: Day +20 to +40

4.4 Manage complications:
    - Febrile neutropenia
    - Mucositis
    - Nausea/vomiting
    - VOD (rare)
```

**Phase 5: Long-Term Follow-Up (Months 1-24+)**
```
5.1 Month 1-3 assessments:
    - CBC with differential (weekly)
    - HbF levels (monthly)
    - Transfusion requirements
    - Adverse events

5.2 Month 6-12 assessments:
    - Full blood count
    - Hemoglobin electrophoresis
    - HbF quantification
    - Quality of life surveys
    - Vaso-occlusive crises (SCD)
    - Transfusion independence

5.3 Long-term monitoring (15 years):
    - Annual health assessments
    - Clonal evolution monitoring
    - Off-target editing surveillance
    - Overall survival and morbidity
```

#### Quality Control

**Manufacturing QC:**
- CD34+ purity: ≥80%
- Viability: ≥70%
- Editing efficiency: ≥60%
- Sterility: Pass
- Endotoxin: <5 EU/kg

**Clinical QC:**
- Engraftment success: ≥95%
- HbF response: ≥20% HbF at 6 months
- Transfusion independence: ≥80% patients

#### Expected Clinical Outcomes

**Efficacy (from clinical trials):**
- Transfusion independence (TDT): 91% (31/34 patients)
- Transfusion independence (SCD): 97% (29/30 patients)
- Vaso-occlusive crisis resolution (SCD): 95% patients
- Median HbF at 12 months: 40-45%
- Median total Hb: 11-13 g/dL

**Safety:**
- Serious adverse events: Primarily due to conditioning/transplant
- Common: Febrile neutropenia, mucositis, nausea
- No graft failure observed
- No treatment-related deaths in trials

---

## 6. Safety and Off-Target Analysis Protocol

### 6.1 Comprehensive Off-Target Detection

**Protocol ID:** SAFETY-OT-001
**Version:** 1.0

#### Objectives
- Detect genome-wide off-target editing
- Quantify off-target frequencies
- Assess clinical risk

#### Methods

**Method 1: Bioinformatic Prediction**
```
1.1 Use Cas-OFFinder or CRISPOR:
    - Input: gRNA sequence + PAM
    - Reference: GRCh38
    - Max mismatches: 4
    - Max bulge size: 2

1.2 Generate list of predicted sites (typically 100-1000)

1.3 Prioritize by:
    - Mismatch count
    - Proximity to genes
    - Conservation score
```

**Method 2: GUIDE-seq (Genome-wide Unbiased Identification of DSBs)**
```
2.1 Co-deliver Cas9 RNP + dsODN tag

2.2 Extract genomic DNA after 72h

2.3 Library preparation:
    - Shear DNA to 500 bp
    - Ligate adaptors
    - PCR amplify with tag-specific primers

2.4 NGS sequencing (depth: 10-20 million reads)

2.5 Bioinformatic analysis:
    - Align to reference genome
    - Identify tag integration sites
    - Quantify read counts
```

**Method 3: CIRCLE-seq (Circularization for In vitro Reporting of CLeavage Effects)**
```
3.1 Prepare genomic DNA library

3.2 Circularize DNA fragments

3.3 Incubate with Cas9 RNP in vitro

3.4 Linearize at cleavage sites

3.5 NGS and analysis
```

**Method 4: Targeted Amplicon Sequencing**
```
4.1 Select top 50 predicted off-target sites

4.2 Design amplicon primers (product size: 300-500 bp)

4.3 PCR amplify from treated samples and controls

4.4 Deep sequencing (depth: 10,000-50,000 reads per site)

4.5 Analyze with CRISPResso2:
    - Quantify indels at each site
    - Compare to background
    - Calculate fold-change vs control
```

#### Analysis and Reporting
```
5.1 Compile all detected off-target sites

5.2 Annotate each site:
    - Chromosomal location
    - Nearest gene
    - Exonic/intronic/intergenic
    - Conservation score
    - Clinical significance (ClinVar)

5.3 Calculate off-target index:
    OT Index = (Total OT edits) / (On-target edits)
    Acceptable: <0.1 (i.e., OT <10% of on-target)

5.4 Risk classification:
    - High risk: >10% editing in coding exon of critical gene
    - Medium risk: 1-10% editing in gene body
    - Low risk: <1% editing or intergenic

5.5 Generate safety report
```

#### Quality Control
- Sequencing depth sufficient (≥10,000 reads per site)
- Include negative controls (no RNP)
- Include positive control (known OT site)
- Validate top hits by orthogonal method

---

## 7. Validation and Characterization Protocol

### 7.1 Editing Outcome Validation

**Protocol ID:** VAL-EDIT-001
**Version:** 1.0

#### Methods

**Sanger Sequencing + TIDE Analysis:**
```
1.1 PCR amplify target region
1.2 Sanger sequence PCR product
1.3 Analyze with TIDE software
1.4 Quantify editing efficiency
```

**Next-Generation Sequencing (NGS):**
```
2.1 Prepare amplicon library (target + 10 OT sites)
2.2 Multiplex PCR with indexed primers
2.3 Sequence on Illumina MiSeq (depth: 50,000 reads per amplicon)
2.4 Analyze with CRISPResso2 or custom pipeline
2.5 Report:
    - On-target editing efficiency
    - Indel spectrum
    - Precise edit frequency (for base/prime editing)
    - Off-target frequencies
```

**Functional Validation:**
```
3.1 For HBB editing:
    - Hemoglobin electrophoresis
    - HPLC quantification of HbA, HbF, HbS
    - Sickling assay (for SCD)
    - Erythroid differentiation and HbF induction

3.2 For gene knockout:
    - Western blot for protein loss
    - Functional assay for pathway disruption

3.3 For knock-in:
    - Confirm integration by PCR/sequencing
    - Verify expression by qRT-PCR/Western blot
```

---

## 8. Clinical Trial Protocol (Example)

### 8.1 Phase I/II Gene Therapy Trial

**Protocol ID:** CLINICAL-001
**Version:** 1.0

#### Trial Design
- Phase: I/II, Open-label, Single-arm
- Indication: Severe Sickle Cell Disease
- Intervention: Autologous CRISPR-edited CD34+ HSPCs
- Primary endpoint: Safety and engraftment
- Secondary endpoint: Transfusion independence, HbF levels, VOC frequency

#### Eligibility Criteria

**Inclusion:**
- Age 18-50 years
- Confirmed SCD diagnosis (HbSS or HbSβ0-thalassemia)
- ≥4 VOCs per year or transfusion-dependent
- Karnofsky score ≥70
- Adequate organ function

**Exclusion:**
- Prior allogeneic HSCT
- Active infection or malignancy
- HIV, HBV, HCV positivity
- Pregnancy or nursing

#### Treatment Plan
```
Day -120 to -90: Mobilization and apheresis
Day -90 to -30: Cell manufacturing
Day -7 to -4: Myeloablative conditioning (busulfan)
Day 0: Infusion of edited cells
Day 1-28: Inpatient supportive care
Month 1-24: Outpatient follow-up
```

#### Assessments
```
Baseline:
- Complete medical history
- CBC, comprehensive metabolic panel
- Hemoglobin electrophoresis
- HbF quantification
- Quality of life questionnaires
- Baseline VOC frequency

Follow-up (Months 1, 3, 6, 12, 18, 24):
- CBC, chemistry
- Hemoglobin electrophoresis and HbF
- VOC tracking
- Transfusion requirements
- Quality of life
- Adverse events

Long-term (Years 3-15):
- Annual assessments
- Long-term safety monitoring
```

#### Safety Monitoring
- Data Safety Monitoring Board (DSMB) oversight
- Stopping rules for serious adverse events
- Off-target editing surveillance (Years 1, 3, 5)

---

## 9. Quality Assurance and Regulatory Compliance

### 9.1 GMP Compliance

**Requirements:**
- FDA 21 CFR Part 210/211 (cGMP)
- EMA Annex 1 (Sterile Products)
- ISO 13485 (Medical Devices)

**Key Elements:**
- Cleanroom facilities (Grade A/B)
- Validated equipment and processes
- Batch record documentation
- Quality control testing
- Deviation management
- CAPA (Corrective and Preventive Actions)

### 9.2 Regulatory Submissions

**IND (Investigational New Drug) Application:**
- Preclinical data package
- Manufacturing information
- Clinical protocol
- Investigator brochure
- Informed consent forms

**BLA (Biologics License Application):**
- Clinical trial results
- Chemistry, Manufacturing, and Controls (CMC)
- Pharmacology/toxicology
- Clinical safety and efficacy

---

## 10. Standard Operating Procedures (SOPs)

### 10.1 Required SOPs

| SOP ID | Title |
|--------|-------|
| SOP-001 | gRNA Design and Validation |
| SOP-002 | RNP Preparation and Quality Control |
| SOP-003 | Cell Culture and Electroporation |
| SOP-004 | Genomic DNA Extraction |
| SOP-005 | PCR and Sequencing |
| SOP-006 | NGS Library Preparation |
| SOP-007 | Bioinformatic Analysis |
| SOP-008 | Off-Target Detection |
| SOP-009 | CD34+ Cell Isolation and Culture |
| SOP-010 | Cell Therapy Manufacturing |
| SOP-011 | Quality Control Testing |
| SOP-012 | Cryopreservation and Thawing |
| SOP-013 | Product Release Criteria |
| SOP-014 | Clinical Administration |
| SOP-015 | Adverse Event Reporting |

---

## 11. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2026-01-12 | Initial FULL protocol specification |

---

## 12. References

1. FDA. "Casgevy (exagamglogene autotemcel) BLA Approval." January 16, 2024.
2. Frangoul et al. "Exagamglogene Autotemcel for Sickle Cell Disease." NEJM, 2021.
3. Anzalone et al. "Search-and-replace genome editing without double-strand breaks." Nature, 2019.
4. Komor et al. "Programmable editing of a target base in genomic DNA." Nature, 2016.
5. Tsai et al. "GUIDE-seq enables genome-wide profiling of off-target cleavage." Nat Biotechnol, 2015.

---

© 2026 WIA (World Certification Industry Association)
弘益人間 (홍익인간) - Benefit All Humanity
