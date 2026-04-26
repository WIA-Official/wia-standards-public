# WIA-AUG-017: Longevity Gene Editing Specification v1.0

> **Standard ID:** WIA-AUG-017
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Human Augmentation Longevity Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Target Gene Database](#2-target-gene-database)
3. [Editing Technologies](#3-editing-technologies)
4. [Delivery Mechanisms](#4-delivery-mechanisms)
5. [Aging Biomarker Assessment](#5-aging-biomarker-assessment)
6. [Safety Protocols](#6-safety-protocols)
7. [Efficacy Metrics](#7-efficacy-metrics)
8. [Ethical Framework](#8-ethical-framework)
9. [Implementation Guidelines](#9-implementation-guidelines)
10. [References](#10-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines comprehensive standards for longevity gene editing technologies, establishing protocols for target gene selection, editing methodologies, safety assessment, efficacy measurement, and ethical frameworks to enable safe and effective extension of human healthspan and lifespan.

### 1.2 Scope

The standard covers:
- Longevity-associated target gene catalog
- Gene editing technology standards (CRISPR, base editing, prime editing, epigenetic modification)
- Delivery mechanism protocols
- Aging biomarker assessment methodologies
- Off-target detection and safety monitoring
- Efficacy metrics for biological age and healthspan
- Ethical considerations and consent frameworks
- Long-term population-level monitoring

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - Longevity gene editing represents humanity's potential to extend healthy, productive life. This specification ensures that such interventions are developed and deployed with the highest standards of safety, efficacy, and equity, benefiting all of humanity rather than creating a longevity divide.

### 1.4 Terminology

- **Biological Age**: Measure of physiological age based on molecular biomarkers, independent of chronological age
- **Healthspan**: Period of life spent in good health, free from chronic diseases and disabilities
- **Lifespan**: Total duration of life from birth to death
- **Age Acceleration**: Difference between biological age and chronological age (positive = accelerated aging)
- **Epigenetic Clock**: Biomarker based on DNA methylation patterns that predicts biological age
- **Senescent Cells**: Aged cells that have stopped dividing but resist death and secrete inflammatory factors
- **Telomere**: Protective cap at the end of chromosomes that shortens with each cell division
- **Off-Target Effect**: Unintended genetic modification at sites other than the intended target
- **Germline Editing**: Genetic modification of reproductive cells (sperm, eggs) or embryos

---

## 2. Target Gene Database

### 2.1 Gene Classification

Longevity-associated genes are classified into functional categories:

| Category | Primary Mechanism | Example Genes |
|----------|-------------------|---------------|
| Telomere Maintenance | Maintain telomere length, prevent replicative senescence | TERT, TERC, RTEL1 |
| Metabolic Regulation | Energy sensing, autophagy, caloric restriction mimetics | SIRT1, SIRT3, SIRT6, AMPK, MTOR, FOXO3 |
| DNA Repair | Maintain genome stability, repair DNA damage | BRCA1, TP53, ATM, SIRT6 |
| Stress Response | Cellular stress resistance, antioxidant response | FOXO3, NRF2, HSF1 |
| Inflammation | Reduce chronic inflammation (inflammaging) | IL6, TNF, NLRP3 |
| Mitochondrial Function | Mitochondrial biogenesis and quality control | TFAM, PGC1A, NRF1, SIRT3 |
| Lipid Metabolism | Cardiovascular health, lipid homeostasis | APOE, CETP, PCSK9 |
| Growth Factors | Systemic rejuvenation, tissue repair | GDF11, KLOTHO |

### 2.2 Evidence Classification

Each target gene is assigned an evidence level based on research support:

#### Strong Evidence
Demonstrated lifespan extension in multiple model organisms (C. elegans, Drosophila, mice) AND human genetic association studies.

**Examples:**
- **FOXO3**: Associated with human longevity in multiple populations; extends lifespan 20-30% in model organisms
- **SIRT1**: NAD+ dependent deacetylase; mimics caloric restriction; extends lifespan 10-20% in mice
- **SIRT6**: DNA repair and inflammation suppressor; extends lifespan 15% in male mice
- **TERT**: Telomerase reverse transcriptase; extends cellular lifespan; associated with cellular immortalization

#### Moderate Evidence
Demonstrated lifespan extension in at least one model organism OR strong human genetic associations.

**Examples:**
- **KLOTHO**: Anti-aging hormone; associated with cognitive function and longevity in humans
- **GDF11**: Rejuvenating factor in parabiosis studies; controversial but promising
- **SIRT3**: Mitochondrial sirtuin; moderate evidence for lifespan extension

#### Emerging Evidence
Mechanistic rationale and preliminary data, but lacking definitive lifespan studies.

**Examples:**
- **NRF2**: Antioxidant response; strong mechanism but mixed longevity data
- **PGC1A**: Mitochondrial biogenesis; indirect longevity effects

### 2.3 Target Gene Profiles

#### TERT (Telomerase Reverse Transcriptase)

**Function**: Maintains telomere length by adding TTAGGG repeats to chromosome ends

**Longevity Mechanism**:
- Prevents replicative senescence
- Extends cellular lifespan
- Maintains stem cell function

**Evidence Level**: Strong (but complex)
- Extends cellular lifespan in vitro
- Associated with longevity in centenarian studies
- **Caution**: Constitutive activation increases cancer risk

**Editing Strategy**: Conditional upregulation (tissue-specific, inducible)

**Safety Considerations**:
- Monitor for tumorigenesis
- Limit upregulation to <3-fold
- Exclude patients with cancer history
- Continuous cancer biomarker surveillance

---

#### FOXO3 (Forkhead Box O3)

**Function**: Transcription factor regulating stress response, DNA repair, and metabolism

**Longevity Mechanism**:
- Promotes autophagy
- Enhances DNA repair
- Increases stress resistance
- Regulates metabolism

**Evidence Level**: Strong
- Strongest human genetic association with longevity
- Extends lifespan 20-30% in C. elegans (DAF-16 ortholog)
- Protective against age-related diseases

**Editing Strategy**: Upregulation via enhancer activation or protective allele introduction

**Safety Considerations**: Excellent safety profile; no major concerns

---

#### SIRT1 (Sirtuin 1)

**Function**: NAD+-dependent deacetylase regulating metabolism and stress response

**Longevity Mechanism**:
- Mimics caloric restriction
- Enhances mitochondrial function
- Improves insulin sensitivity
- Neuroprotective effects

**Evidence Level**: Strong
- Extends lifespan 10-15% in mice
- Protects against metabolic syndrome, neurodegeneration
- Activated by resveratrol and NAD+ precursors

**Editing Strategy**: Moderate upregulation (1.5-2x)

**Safety Considerations**: Generally safe; monitor metabolic parameters

---

#### SIRT6 (Sirtuin 6)

**Function**: NAD+-dependent deacetylase with roles in DNA repair, inflammation, and metabolism

**Longevity Mechanism**:
- Enhances DNA double-strand break repair
- Suppresses NF-κB inflammatory signaling
- Regulates glucose and lipid metabolism
- Maintains telomeric chromatin

**Evidence Level**: Strong
- Extends lifespan 15% in male mice
- Protective against age-related inflammation
- Cancer suppressor

**Editing Strategy**: Upregulation (1.5-2x)

**Safety Considerations**: Excellent profile; anti-cancer effects

---

#### AMPK (AMP-activated Protein Kinase)

**Function**: Master energy sensor regulating metabolism

**Longevity Mechanism**:
- Promotes autophagy
- Enhances mitochondrial biogenesis
- Improves insulin sensitivity
- Activated by metformin and exercise

**Evidence Level**: Strong
- Extends lifespan in multiple organisms
- Mimics exercise and caloric restriction
- Protective against metabolic syndrome

**Editing Strategy**: Activating mutations or upregulation

**Safety Considerations**: Monitor for hypoglycemia

---

#### MTOR (Mechanistic Target of Rapamycin)

**Function**: Nutrient sensor regulating growth and metabolism

**Longevity Mechanism**:
- **Inhibition** extends lifespan by promoting autophagy
- Reduces protein synthesis stress
- Mimics dietary restriction

**Evidence Level**: Strong
- mTOR inhibition (rapamycin) extends lifespan 10-15% in mice
- Most reproducible longevity intervention

**Editing Strategy**: Partial inhibition (NOT complete knockout)

**Safety Considerations**:
- Monitor immune function (mTOR inhibitors are immunosuppressive)
- Avoid excessive inhibition (<50%)
- Monitor wound healing

---

#### KLOTHO (Alpha-Klotho)

**Function**: Anti-aging hormone regulating mineral metabolism and FGF23 signaling

**Longevity Mechanism**:
- Increases resistance to oxidative stress
- Enhances cognition
- Protects kidney and cardiovascular function
- Anti-inflammatory effects

**Evidence Level**: Moderate-Strong
- KL-VS variant associated with human longevity and cognition
- Klotho overexpression extends lifespan 20-30% in mice
- Declines with age

**Editing Strategy**: Upregulation or secreted protein delivery

**Safety Considerations**: Generally safe; monitor phosphate levels

---

#### APOE (Apolipoprotein E)

**Function**: Lipid transport protein; APOE2 protective, APOE4 increases Alzheimer's risk

**Longevity Mechanism**:
- APOE2 allele associated with longevity and reduced cardiovascular/Alzheimer's risk
- APOE4 allele increases Alzheimer's risk 10-15x

**Evidence Level**: Strong for human genetics
- APOE2 carriers overrepresented in centenarians
- APOE4 carriers underrepresented

**Editing Strategy**: Convert APOE4 → APOE2 or APOE3 in carriers

**Safety Considerations**:
- Complex gene; requires precise editing
- Potential lipid metabolism disruption
- Monitor lipid profiles carefully

---

### 2.4 Gene Selection Algorithm

```
Gene Selection Score = Evidence (40%) + Safety (25%) + Age-Appropriateness (20%) + Goal-Alignment (15%)

Where:
- Evidence: Strong = 10, Moderate = 6, Emerging = 3
- Safety: Safety Profile score (1-10)
- Age-Appropriateness: Match to patient age group
- Goal-Alignment: Match to patient goals (healthspan, disease prevention, etc.)
```

**Age-Based Recommendations:**

| Age Group | Primary Targets | Rationale |
|-----------|----------------|-----------|
| 18-40 | SIRT1, FOXO3, SIRT6 | Prevention focus; enhance cellular maintenance |
| 40-60 | SIRT1, FOXO3, AMPK, SIRT3 | Metabolic health; maintain function |
| 60-75 | TERT, KLOTHO, SIRT6, FOXO3 | Cellular repair; telomere maintenance |
| 75+ | TERT, KLOTHO, GDF11 | Tissue rejuvenation; systemic effects |

---

## 3. Editing Technologies

### 3.1 Technology Overview

| Technology | Mechanism | Precision | Efficiency | Off-Target Risk | Reversibility | Best Use Case |
|------------|-----------|-----------|------------|-----------------|---------------|---------------|
| CRISPR-Cas9 | Double-strand break + NHEJ/HDR | 85% | 90% | 15% | 10% | Gene knockouts |
| Base Editing | Direct C→T or A→G conversion | 95% | 80% | 5% | 10% | Point mutations |
| Prime Editing | Search-and-replace via pegRNA | 98% | 70% | 2% | 5% | Precise edits, insertions |
| Epigenetic Mod | DNA methylation/histone acetylation | 80% | 85% | 10% | 60% | Reversible regulation |
| RNA Interference | mRNA degradation | 75% | 90% | 20% | 95% | Temporary suppression |

### 3.2 CRISPR-Cas9

**Mechanism**: Cas9 endonuclease creates double-strand break; repaired by NHEJ (error-prone) or HDR (precise with template)

**Applications for Longevity**:
- Gene knockouts (e.g., MTOR partial inhibition)
- Gene knock-ins (e.g., protective alleles)

**Protocol**:
1. Design guide RNA (20bp + NGG PAM)
2. Deliver Cas9 + gRNA via AAV, LNP, or RNP complex
3. Allow 48-72 hours for editing
4. Screen for on-target editing by sequencing
5. Screen for off-targets by WGS or GUIDE-seq

**Advantages**:
- Well-established, widely used
- High efficiency
- Can achieve knockouts or knock-ins

**Limitations**:
- Off-target effects (10-20% of cells may have off-targets)
- Permanent (mostly irreversible)
- Potential for large deletions at target site

**Safety Requirements**:
- Whole genome sequencing pre- and post-treatment
- Off-target prediction using computational tools (Cas-OFFinder, CRISPOR)
- Limit to somatic cells (no germline editing)

---

### 3.3 Base Editing

**Mechanism**: Fused deaminase enzyme converts C→T (or A→G) without creating double-strand break

**Applications for Longevity**:
- Introduce protective point mutations (e.g., APOE4→APOE2 requires R158C and R112C)
- Correct pathogenic mutations
- Create gain-of-function alleles

**Protocol**:
1. Design base editor (e.g., BE3 for C→T, ABE for A→G)
2. Design gRNA to position target base in editing window (positions 4-8)
3. Deliver via AAV or LNP
4. Verify editing by Sanger sequencing
5. Check for bystander edits in window

**Advantages**:
- Higher precision than CRISPR-Cas9
- Lower off-target rate (5% vs 15%)
- No double-strand breaks (less genotoxic)
- Product mixture more predictable

**Limitations**:
- Limited to C→T or A→G transitions
- Bystander editing of other Cs/As in window
- PAM requirement (NGG for SpCas9-BE)
- Slightly lower efficiency than CRISPR-Cas9

**Safety Requirements**:
- Check bystander edits in editing window
- Verify no off-target deamination by RNA-seq
- Monitor for adenine deaminase RNA off-targets (ABE)

---

### 3.4 Prime Editing

**Mechanism**: Engineered reverse transcriptase fused to Cas9 nickase; pegRNA encodes desired edit

**Applications for Longevity**:
- Precise point mutations, small insertions/deletions
- All 12 types of point mutations possible
- Insertions up to 44bp, deletions up to 80bp

**Protocol**:
1. Design pegRNA with spacer, gRNA scaffold, reverse transcriptase template, primer binding site
2. Design nicking sgRNA (optional, increases efficiency)
3. Deliver prime editor (PE2 or PE3) + pegRNA
4. Verify editing by sequencing (Sanger or NGS)

**Advantages**:
- Highest precision (98%)
- Lowest off-target rate (2%)
- All mutation types possible
- No double-strand breaks

**Limitations**:
- Lower efficiency (70%)
- More complex design
- Larger cargo (difficult for AAV)
- Newer technology (less validated)

**Safety Requirements**:
- Validate pegRNA design in vitro first
- Check for pegRNA-independent off-targets
- Monitor for reverse transcriptase errors

---

### 3.5 Epigenetic Modification

**Mechanism**: Modify DNA methylation or histone acetylation to regulate gene expression without changing DNA sequence

**Applications for Longevity**:
- Upregulate longevity genes (e.g., SIRT1, FOXO3)
- Downregulate aging-promoting genes (e.g., IL6, TNF)
- Rejuvenate epigenetic age (methylation reprogramming)

**Technologies**:
- **dCas9-DNMT3A**: Add DNA methylation (silence gene)
- **dCas9-TET1**: Remove DNA methylation (activate gene)
- **dCas9-p300**: Add histone acetylation (activate gene)
- **dCas9-KRAB**: Add histone methylation (silence gene)

**Protocol**:
1. Design gRNA targeting gene promoter or enhancer
2. Deliver dCas9-effector + gRNA
3. Measure gene expression change by qPCR or RNA-seq
4. Monitor methylation changes by bisulfite sequencing

**Advantages**:
- Reversible (60% reversibility)
- No permanent DNA changes
- Can achieve strong gene regulation
- Lower off-target concern (no DNA cutting)

**Limitations**:
- Moderate precision (80%)
- May require repeated dosing
- Epigenetic memory varies by target
- Off-target gene expression changes possible

**Safety Requirements**:
- RNA-seq to detect off-target gene expression changes
- Monitor for unintended silencing of nearby genes
- Assess reversibility timeline

---

## 4. Delivery Mechanisms

### 4.1 Delivery Method Selection

| Method | Tropism | Payload Capacity | Immunogenicity | Best For |
|--------|---------|------------------|----------------|----------|
| AAV (Adeno-Associated Virus) | Liver, muscle, brain, heart | 4.7 kb | Low-Moderate | Long-term expression, specific tissues |
| Lentivirus | Broad; integrates into genome | 8 kb | Moderate | Ex vivo (stem cells) |
| Lipid Nanoparticle (LNP) | Liver > spleen > other | 10+ kb | Low | Systemic, large cargo, transient |
| Electroporation | Local only | Unlimited | None | Ex vivo, muscle |
| Direct Injection | Local tissue | Unlimited | Variable | Muscle, eye |

### 4.2 AAV Vectors

**Advantages**:
- Low immunogenicity
- Non-integrating (mostly episomal)
- Tissue-specific serotypes (AAV9 for CNS/muscle, AAV8 for liver)
- Long-term expression (years)

**Limitations**:
- Size limit: 4.7 kb (difficult for Cas9 ~4.2 kb)
- Pre-existing immunity in 40-60% of population
- Potential for integration at low frequency

**Recommended Serotypes for Longevity Targets**:
- **AAV9**: Systemic delivery; crosses blood-brain barrier; targets muscle, liver, brain
- **AAV8**: Liver-specific; ideal for metabolic genes (SIRT1, AMPK)
- **AAV-PHP.eB**: Enhanced CNS delivery (mice; human variant needed)

**Dosing**:
- Typical dose: 1e13 to 5e13 viral genomes per kilogram (vg/kg)
- Higher doses increase risk of immune response

**Safety Monitoring**:
- Pre-treatment: Check anti-AAV neutralizing antibodies
- Post-treatment: Monitor liver enzymes (ALT, AST) weekly for 4 weeks
- Long-term: Annual antibody titers

---

### 4.3 Lipid Nanoparticles (LNP)

**Advantages**:
- Large payload capacity (10+ kb, can fit base editors and prime editors)
- No pre-existing immunity
- Rapid manufacturing
- Transient expression (safer for some applications)

**Limitations**:
- Liver tropism (70-90% accumulation)
- Transient expression (days to weeks)
- Inflammatory response possible

**Applications for Longevity**:
- Liver-targeted SIRT1, SIRT3, AMPK upregulation
- APOE editing in hepatocytes
- Short-term epigenetic reprogramming

**Formulation**:
- Ionizable lipid (e.g., SM-102, ALC-0315)
- Helper lipids (DSPC, cholesterol)
- PEGylated lipid
- mRNA or DNA cargo

**Dosing**:
- Typical: 0.5 to 5 mg/kg
- Can repeat doses every 4-8 weeks

**Safety Monitoring**:
- Monitor liver enzymes
- Check for complement activation (rare)
- Monitor inflammatory markers (IL-6, CRP)

---

### 4.4 Ex Vivo Gene Editing (Lentivirus, Electroporation)

**Approach**:
1. Harvest patient's hematopoietic stem cells (HSCs) or T cells
2. Edit ex vivo using lentivirus or electroporation
3. Expand and select edited cells
4. Re-infuse into patient

**Applications for Longevity**:
- Edit HSCs with longevity genes (systemic long-term expression)
- Create rejuvenated immune cells (T cell rejuvenation)

**Advantages**:
- High efficiency (>80% in optimized protocols)
- Can sequence and select correctly edited cells
- Minimize off-target risk by selection

**Limitations**:
- Requires specialized facilities (GMP manufacturing)
- Expensive ($100,000+ per treatment)
- Time-consuming (weeks to months)
- Requires myeloablation for HSC engraftment

---

## 5. Aging Biomarker Assessment

### 5.1 Epigenetic Clocks

Epigenetic clocks measure biological age based on DNA methylation patterns at specific CpG sites.

#### 5.1.1 Horvath Clock (2013)

**Description**: Multi-tissue age predictor based on 353 CpG sites

**Accuracy**: ±3.6 years (across all tissues)

**Measurement**:
- Extract DNA from blood, saliva, or tissue
- Bisulfite conversion
- Illumina Infinium MethylationEPIC BeadChip or targeted sequencing
- Apply Horvath algorithm

**Interpretation**:
- Age acceleration = Horvath age - Chronological age
- Positive acceleration = accelerated aging
- Negative acceleration = decelerated aging (desirable)

**Applications**: Universal aging biomarker; suitable for pre/post intervention

---

#### 5.1.2 GrimAge (2019)

**Description**: Predictor of mortality and healthspan based on 1,030 CpG sites and smoking pack-years

**Accuracy**: Best predictor of mortality and morbidity

**Measurement**: Same as Horvath (BeadChip or sequencing)

**Interpretation**:
- GrimAge acceleration predicts:
  - All-cause mortality
  - Time to coronary heart disease, cancer
  - Cognitive decline

**Applications**: Best for assessing healthspan extension interventions

---

#### 5.1.3 DunedinPACE (Pace of Aging)

**Description**: Measures rate of biological aging (not biological age itself)

**Metric**: Years of biological aging per calendar year
- 1.0 = normal aging
- <1.0 = slower aging (desirable)
- >1.0 = faster aging

**Applications**: Best for tracking aging rate changes during intervention

---

### 5.2 Telomere Length

**Measurement Methods**:
1. **qPCR** (T/S ratio): High-throughput, cost-effective; moderate precision
2. **Flow-FISH**: Measures distribution; detects critically short telomeres
3. **Southern blot**: Gold standard; low throughput

**Reference Values**:
- Newborn: ~11 kb
- Adult (age 20): ~9-10 kb
- Adult (age 50): ~7-8 kb
- Adult (age 80): ~6-7 kb
- Critically short: <5 kb (senescence threshold)

**Interpretation**:
- Telomere length decreases ~25-50 bp per year
- Accelerated shortening associated with stress, smoking, obesity
- TERT upregulation can restore length

**Target for Intervention**:
- Goal: Maintain or increase telomere length by >0.5 kb
- Success threshold: No critically short telomeres (<5 kb)

---

### 5.3 Senescent Cell Burden

**Markers**:
- **p16INK4a** (CDKN2A): Cell cycle arrest marker
- **p21** (CDKN1A): DNA damage response marker
- **SA-β-galactosidase**: Senescence-associated enzyme

**Measurement**:
- Tissue biopsy (skin, muscle, adipose)
- Immunohistochemistry for p16, p21
- SA-β-gal staining
- RNA-seq for senescence-associated secretory phenotype (SASP) markers

**SASP Factors**:
- IL-6, IL-8 (inflammatory cytokines)
- MMP-3, MMP-9 (matrix metalloproteinases)
- Growth factors (VEGF, HGF)

**Reference Values**:
- Young adult (<30 years): <5% senescent cells
- Middle-aged (50-60): 10-15%
- Elderly (>70): 15-25%

**Target for Intervention**:
- Reduce senescent cell burden by >30%
- Goal: <10% p16+ cells

**Senolytic Interventions** (adjunct to gene editing):
- Dasatinib + Quercetin
- Fisetin
- Navitoclax (BCL-2 inhibitor)

---

### 5.4 Inflammation Markers (Inflammaging)

Chronic low-grade inflammation accelerates aging.

**Key Markers**:
- **C-Reactive Protein (CRP)**: General inflammation
  - Normal: <3 mg/L
  - Elevated: >3 mg/L
  - Target: <1 mg/L

- **Interleukin-6 (IL-6)**: Pro-inflammatory cytokine
  - Normal: <5 pg/mL
  - Elevated: >5 pg/mL
  - Target: <2 pg/mL

- **TNF-α**: Pro-inflammatory cytokine
  - Normal: <8 pg/mL
  - Target: <5 pg/mL

**Measurement**: ELISA or multiplex immunoassay

**Interventions**:
- SIRT6 upregulation (suppresses NF-κB → reduces IL-6, TNF)
- IL-6 or NLRP3 downregulation via epigenetic silencing

---

### 5.5 Functional Capacity

**Assessments**:

1. **VO2 Max** (Cardiorespiratory fitness)
   - Measure: Treadmill or cycle ergometer test
   - Reference:
     - Age 20-29: 45-55 mL/kg/min (men), 38-48 (women)
     - Age 60-69: 25-35 (men), 20-30 (women)
   - Target: Increase by >10%

2. **Grip Strength**
   - Measure: Dynamometer
   - Reference: Age- and sex-specific norms
   - Decline: Predicts mortality and disability
   - Target: Maintain or increase

3. **Gait Speed**
   - Measure: 4-meter walk test
   - Reference:
     - Normal: >1.0 m/s
     - Slow: <0.8 m/s (increased mortality risk)
   - Target: Maintain >1.0 m/s

4. **Cognitive Function**
   - Montreal Cognitive Assessment (MoCA)
   - Target: Maintain or improve score

---

### 5.6 Metabolic Biomarkers

| Biomarker | Normal Range | Optimal Range | Aging Concern |
|-----------|-------------|---------------|---------------|
| Fasting Glucose | 70-99 mg/dL | 70-85 mg/dL | >100: prediabetes |
| HbA1c | <5.7% | <5.4% | >5.7%: prediabetes |
| Total Cholesterol | <200 mg/dL | <180 mg/dL | >240: high |
| LDL | <100 mg/dL | <70 mg/dL | >130: high |
| HDL | >40 (men), >50 (women) | >60 mg/dL | Low: CVD risk |
| Triglycerides | <150 mg/dL | <100 mg/dL | >200: high |
| HOMA-IR | <2.0 | <1.0 | >2.5: insulin resistance |

**Interventions**:
- SIRT1, AMPK upregulation: Improve insulin sensitivity, lipid profiles
- APOE2 editing: Reduce LDL, CVD risk

---

## 6. Safety Protocols

### 6.1 Off-Target Detection

**Computational Prediction** (Pre-Treatment):
- Tools: Cas-OFFinder, CRISPOR, COSMID
- Identify potential off-targets with ≤3 mismatches
- Score by position, mismatch type, chromatin accessibility
- Avoid guides with high-score off-targets in coding regions

**Experimental Detection** (Post-Treatment):

1. **GUIDE-seq** (Genome-wide Unbiased Identification of DSBs Enabled by Sequencing)
   - Detects actual double-strand breaks genome-wide
   - Sensitivity: Detects off-targets down to 0.1% frequency

2. **Whole Genome Sequencing (WGS)**
   - Compare pre- and post-treatment genomes
   - Identify all SNVs, indels, structural variants
   - Cost: ~$1,000 per genome
   - Required: All patients receiving gene editing

3. **Targeted Deep Sequencing**
   - Sequence predicted off-target sites at 10,000x coverage
   - Detect rare off-target events (<0.1%)

**Acceptance Criteria**:
- No off-target edits in coding exons with frequency >0.1%
- Total off-target rate <0.1% genome-wide
- No large deletions (>50 bp) at on-target site

---

### 6.2 Cancer Surveillance

**Rationale**: Telomerase activation (TERT upregulation) and cell cycle modifications (TP53, CDKN2A) carry cancer risk.

**Pre-Treatment**:
- Exclude patients with:
  - Active cancer
  - History of cancer (within 5 years)
  - High-risk genetic variants (BRCA1/2, TP53 mutations, etc.)
- Baseline cancer screening (age-appropriate)

**Post-Treatment**:
- Monthly monitoring (Year 1):
  - Tumor markers (CEA, CA 19-9, PSA, AFP as appropriate)
  - Blood counts (complete blood count)
- Quarterly monitoring (Years 2-5):
  - Tumor markers
  - Imaging (CT or MRI) if indicated
- Annual monitoring (Years 5-10+):
  - Age-appropriate cancer screening
  - Circulating tumor DNA (ctDNA) screening

**Telomerase Monitoring** (if TERT edited):
- Measure telomerase activity in leukocytes
- Target: <3x baseline activity
- If >3x: Consider senolytic intervention or closer surveillance

---

### 6.3 Immune Response

**AAV-Related Immune Response**:
- Screen for pre-existing neutralizing antibodies
  - If titer >1:5, avoid AAV delivery or use alternative serotype
- Monitor for T cell response to capsid:
  - Measure transaminases (ALT, AST) weekly for 4 weeks
  - If elevated: Short-term immunosuppression (prednisone)
- Monitor for antibody generation post-treatment

**LNP-Related Immune Response**:
- Monitor for complement activation (C3a, C5a)
- Rare but possible: Infusion reactions
- Monitor inflammatory markers (IL-6, CRP)

**Management**:
- Mild elevation: Observation
- Moderate elevation: Short-term corticosteroids
- Severe reaction: Discontinue, supportive care

---

### 6.4 Germline Editing Prevention

**Prohibition**: Germline editing (modification of sperm, eggs, or embryos) is prohibited under this standard.

**Rationale**:
- Unknown long-term effects on future generations
- Ethical concerns (consent of unborn)
- Potential for heritable off-target effects

**Safeguards**:
- Somatic cell editing only
- Exclude reproductive tissues from viral vector tropism
- Contraception required for 6 months post-treatment
- Sperm/egg banking before treatment (if desired for future reproduction)

**Monitoring**:
- Screen for inadvertent germline modification:
  - If patient conceives post-treatment: Offer prenatal genetic testing
  - If editing detected in offspring: Long-term monitoring, report to registry

---

### 6.5 Reversibility Protocols

**Epigenetic Modifications** (60% reversible):
- Discontinue delivery → gradual return to baseline (weeks to months)
- Active reversal: Deliver opposite epigenetic effector (e.g., DNMT3A to reverse TET1 demethylation)

**RNA Interference** (95% reversible):
- Discontinue siRNA/shRNA delivery → mRNA levels recover in days

**Permanent Edits** (CRISPR, base editing, prime editing):
- Mostly irreversible
- Potential reversal strategies (experimental):
  - Edit back to original sequence (if no fitness advantage of edited cells)
  - Selectively eliminate edited cells (if cell-type specific marker)

**Safety Valve**:
- Incorporate inducible suicide gene (HSV-TK, iCaspase9) for emergency elimination of edited cells
- Trigger: Ganciclovir (HSV-TK) or small molecule AP1903 (iCaspase9)

---

## 7. Efficacy Metrics

### 7.1 Primary Outcomes

#### Biological Age Reduction

**Definition**: Decrease in biological age (measured by epigenetic clocks) relative to chronological age progression

**Measurement**:
- Baseline: Pre-treatment biological age (Horvath, GrimAge, DunedinPACE)
- Follow-up: 6 months, 12 months, 24 months, annually
- Calculate: ΔBiological Age - ΔChronological Age

**Success Criteria**:
- Minimal clinically significant: 2 years biological age reduction
- Good response: 5 years reduction
- Excellent response: 10+ years reduction

**Example**:
- Patient: Chronological age 50, Biological age 58 (age acceleration +8)
- After 1 year of treatment: Chronological age 51, Biological age 54
- Biological age reduction: (58-54) - (51-50) = 4 - 1 = 3 years ✓ Success

---

#### Healthspan Extension

**Definition**: Years of disability-free, disease-free life added

**Measurement**:
- Prospective cohort study with long-term follow-up (10+ years)
- Time to first age-related disease diagnosis (cardiovascular, diabetes, cancer, dementia)
- Quality-adjusted life years (QALYs)
- Functional independence

**Success Criteria**:
- Delay age-related disease onset by ≥5 years
- Maintain functional independence ≥3 years longer than matched controls

**Challenges**:
- Requires decades to measure definitively
- Use surrogate markers: Biomarkers, disease risk scores, functional capacity

---

#### Telomere Elongation

**Measurement**: Change in mean telomere length (qPCR or Flow-FISH)

**Success Criteria**:
- Increase in mean telomere length by ≥0.5 kb
- Reduction in critically short telomeres (<5 kb)

**Timeline**: Measurable within 6-12 months of TERT upregulation

---

#### Senescent Cell Clearance

**Measurement**: % p16+ cells in tissue biopsy

**Success Criteria**:
- Reduction by ≥30% from baseline
- Target: <10% p16+ cells

**Timeline**: Measurable within 3-6 months

---

### 7.2 Secondary Outcomes

**Biomarker Improvements**:
- CRP reduction to <1 mg/L
- IL-6 reduction by >50%
- Improved metabolic markers (glucose, HbA1c, lipids, HOMA-IR)

**Functional Capacity**:
- VO2 max increase by >10%
- Grip strength maintenance or increase
- Gait speed maintained >1.0 m/s

**Cognitive Function**:
- MoCA score improvement or maintenance
- Delayed cognitive decline

**Disease Incidence**:
- Reduced incidence of age-related diseases vs. matched cohort

---

### 7.3 Safety Outcomes

**Off-Target Editing**:
- Acceptable: <0.1% genome-wide off-target rate
- No off-targets in coding exons at frequency >0.1%

**Adverse Events**:
- Grade 3+ adverse events: <5%
- Serious adverse events (SAEs): <2%

**Cancer Incidence**:
- Not exceeding age-matched control population
- If TERT upregulation: Enhanced surveillance, ensure no increase above baseline risk

---

## 8. Ethical Framework

### 8.1 Core Principles

#### Autonomy

**Informed Consent Requirements**:
- Disclosure of:
  - Experimental nature of intervention
  - Uncertain long-term effects (novel therapy)
  - Potential risks: Off-target effects, cancer risk (esp. TERT), immune reactions
  - Alternative approaches (lifestyle, pharmaceuticals)
  - Right to withdraw
- Enhanced consent process:
  - Separate session with genetics counselor
  - Comprehension assessment
  - Waiting period (minimum 2 weeks before treatment)

#### Beneficence

**Maximize Benefit**:
- Prioritize healthspan over lifespan
- Focus on disability-free years, quality of life
- Use best available evidence for gene selection

**Risk-Benefit Assessment**:
- Require favorable risk-benefit ratio
- Exclude high-risk patients (cancer history, immunodeficiency)
- Start with safest genes (FOXO3, SIRT6) before higher-risk (TERT)

#### Non-Maleficence

**Do No Harm**:
- Extensive pre-clinical testing required
- Phased rollout: Healthy adults → elderly → broader population
- Continuous safety monitoring
- Halt trials immediately if unexpected safety signals

#### Justice

**Equitable Access**:
- Avoid creating "longevity divide" between rich and poor
- Public funding for research
- Require affordability plans for commercialization
- Reserve X% of treatments for underserved populations

**Equity Access Fund**:
- Pharmaceutical/biotech companies allocate ≥10% of revenue to fund treatments for low-income patients
- Government subsidies for medically necessary interventions

---

### 8.2 Germline Editing Prohibition

**Policy**: Germline editing (sperm, eggs, embryos) is **prohibited** under WIA-AUG-017.

**Rationale**:
- **Consent**: Cannot obtain consent from future generations
- **Uncertainty**: Unknown long-term effects over multiple generations
- **Off-Targets**: Heritable off-target effects unacceptable
- **Equity**: Risk of "genetic enhancement" divide
- **Slippery Slope**: Could lead to non-therapeutic enhancements

**Exception Process**:
- May be reconsidered in future if:
  - Decades of somatic editing safety data
  - Off-target rates <0.01%
  - Broad societal consensus
  - Regulatory approval process established
- Requires international consensus (WHO, UNESCO)

---

### 8.3 Consent Framework

**Standard Consent Elements**:
1. Nature of intervention (gene editing)
2. Purpose (extend healthspan, reduce biological age)
3. Procedures (delivery method, monitoring)
4. Risks: Off-target effects, cancer, immune response, unknown long-term effects
5. Benefits: Potential healthspan extension, disease prevention
6. Alternatives: Lifestyle, pharmaceuticals, no intervention
7. Confidentiality and data use
8. Right to withdraw
9. Compensation for injury

**Enhanced Consent for Longevity Gene Editing**:
10. **Generational Considerations**:
    - Somatic editing only; no germline modification
    - Unknown effects if patient conceives post-treatment
    - Contraception requirement (6 months)
    - Offer prenatal testing if pregnancy occurs
11. **Long-Term Commitment**:
    - Agree to lifetime monitoring (annual visits minimum)
    - Report adverse events immediately
    - Participate in registry
12. **Financial**:
    - Cost disclosure
    - Insurance may not cover (experimental)
13. **Psychological**:
    - Potential psychological effects of extended healthspan
    - Family dynamics (outliving peers/family)

---

### 8.4 Equity and Access

**Challenges**:
- High cost ($100,000-$500,000 per treatment)
- Risk of longevity divide: Wealthy live longer and healthier
- Exacerbation of existing health disparities

**Mitigation Strategies**:

1. **Public Research Funding**:
   - Government investment in longevity gene editing research
   - Results in public domain (prevent monopolization)

2. **Price Regulation**:
   - Cost-effectiveness analysis: Compare to lifetime healthcare costs saved
   - Price ceiling based on QALY gains

3. **Universal Access Programs**:
   - National health systems include coverage
   - Tiered pricing by income

4. **Equity Access Fund**:
   - Companies allocate ≥10% of revenue to subsidize treatments for low-income patients
   - Tax incentives for equitable access

5. **Open-Source Tools**:
   - Guide RNA design tools, protocols, safety algorithms freely available
   - Capacity building in low-resource settings

---

### 8.5 Long-Term Monitoring and Registry

**Rationale**: Longevity gene editing has unknown long-term effects (decades). Mandatory monitoring is essential.

**WIA Longevity Gene Editing Registry**:

**Enrollment**: All patients receiving longevity gene editing

**Data Collected**:
- Baseline: Demographics, medical history, genetic profile, biomarkers
- Treatment: Genes edited, technology, delivery method, dose
- Follow-up (annual minimum):
  - Biomarkers (epigenetic age, telomeres, senescent cells, inflammation)
  - Adverse events
  - Disease diagnoses
  - Functional status
  - Quality of life
- Vital status (lifetime)

**Data Governance**:
- De-identified for research
- Patient access to own data
- Transparent reporting of aggregate outcomes
- Independent oversight committee

**Duration**: Lifetime monitoring

---

## 9. Implementation Guidelines

### 9.1 Patient Selection Criteria

**Inclusion Criteria**:
- Age: 18+ years (no upper limit, but risk assessment for elderly)
- Willing to comply with monitoring requirements
- Informed consent
- Baseline biological age assessment completed
- No contraindications (see below)

**Exclusion Criteria** (Absolute):
- Active cancer
- Pregnancy or breastfeeding
- Severe immunodeficiency
- Uncontrolled autoimmune disease
- Life expectancy <5 years from other causes

**Exclusion Criteria** (Relative):
- History of cancer (within 5 years)
- High-risk genetic variants (BRCA1/2 mutation, Lynch syndrome)
- Pre-existing anti-AAV neutralizing antibodies (if AAV delivery planned)
- Age >85 years (increased risk; individualized assessment)

---

### 9.2 Treatment Protocol

**Phase 1: Pre-Treatment Assessment (4-8 weeks)**

Week -8 to -4:
- Initial consultation and informed consent
- Medical history and physical exam
- Baseline biomarkers:
  - Epigenetic age (Horvath, GrimAge, DunedinPACE)
  - Telomere length
  - Senescent cell markers (tissue biopsy)
  - Inflammation markers (CRP, IL-6, TNF-α)
  - Metabolic panel
  - Functional assessments (VO2 max, grip strength, gait speed)
- Whole genome sequencing (baseline)
- Cancer screening (age-appropriate)
- AAV antibody testing (if applicable)

Week -4 to 0:
- Review results with patient
- Finalize gene targets and editing protocol
- Off-target prediction and guide RNA design
- Manufacture gene editing reagents
- Final consent

**Phase 2: Treatment (Day 0)**

- Admit for observation (outpatient or overnight depending on delivery method)
- Administer gene editing therapy:
  - AAV: IV infusion over 1 hour
  - LNP: IV infusion over 30 minutes
  - Ex vivo: Stem cell re-infusion
- Monitor for immediate adverse reactions (4-24 hours)

**Phase 3: Acute Monitoring (Weeks 1-12)**

- Week 1, 2, 4: Clinic visit
  - Vital signs, symptom review
  - Blood counts (CBC)
  - Liver enzymes (ALT, AST)
- Week 8, 12: Clinic visit
  - Same as above
  - Preliminary gene expression analysis (qPCR)

**Phase 4: Efficacy Assessment (Months 6, 12, 24)**

- Biomarker reassessment:
  - Epigenetic age (all clocks)
  - Telomere length
  - Senescent cell burden
  - Inflammation markers
  - Metabolic markers
  - Functional capacity
- Off-target screening:
  - Month 6: Targeted deep sequencing of predicted off-targets
  - Month 12: Whole genome sequencing
- Cancer surveillance:
  - Tumor markers
  - Imaging if indicated

**Phase 5: Long-Term Monitoring (Annual, Lifetime)**

- Annual visits:
  - Biomarkers (simplified panel: Epigenetic age, inflammation, metabolic)
  - Adverse events and disease diagnoses
  - Quality of life assessment
  - Cancer screening (age-appropriate)
- Every 5 years:
  - Whole genome sequencing
  - Comprehensive biomarker panel

---

### 9.3 Clinical Trial Design

**Recommended Phased Approach**:

**Phase 1: Safety and Dose-Finding (n=10-20)**
- Objective: Safety, tolerability, preliminary efficacy signals
- Population: Healthy adults age 40-60, biological age acceleration >5 years
- Intervention: Single safest gene (e.g., FOXO3 or SIRT6 upregulation)
- Dose escalation: 3 dose levels
- Duration: 2 years
- Endpoints: Adverse events, off-target rate, biomarker changes

**Phase 2: Efficacy (n=50-100)**
- Objective: Efficacy on biological age reduction
- Population: Adults age 40-70, biological age acceleration >5 years
- Intervention: Optimized dose from Phase 1
- Comparator: Placebo or standard of care
- Duration: 2 years
- Primary endpoint: Biological age reduction (GrimAge) at 12 months
- Secondary endpoints: Healthspan markers, functional capacity, disease incidence

**Phase 3: Confirmatory (n=500-1000)**
- Objective: Confirm efficacy and long-term safety
- Population: Broader age range, multiple sites
- Intervention: Optimal protocol from Phase 2
- Comparator: Randomized to treatment vs. no treatment (may be unethical to use placebo given Phase 2 results)
- Duration: 5+ years
- Primary endpoint: Healthspan-adjusted life years (QALY), disease-free survival
- Secondary endpoints: All-cause mortality, disease incidence

**Adaptive Design**:
- Allow modification of gene targets, doses based on interim data
- Basket trial design: Multiple genes tested in parallel
- Biomarker-driven: Stratify by baseline biological age, response prediction

---

### 9.4 Regulatory Pathway

**United States (FDA)**:
- Classification: Gene therapy (Biologics License Application, BLA)
- Pathway: Investigational New Drug (IND) → Phase 1/2/3 trials → BLA approval
- Requirements:
  - Preclinical data: Animal models (mice, primates)
  - CMC (Chemistry, Manufacturing, Controls): GMP manufacturing
  - Clinical trial data demonstrating safety and efficacy
- Timeline: 10-15 years typical for gene therapy approval

**European Union (EMA)**:
- Classification: Advanced Therapy Medicinal Product (ATMP)
- Pathway: Clinical Trial Application (CTA) → Marketing Authorization Application (MAA)
- Committee for Advanced Therapies (CAT) review

**Accelerated Approval**:
- Potential for accelerated approval based on biomarker endpoints (biological age reduction)
- Require confirmatory trials for long-term healthspan outcomes
- Conditional approval with post-market surveillance

---

### 9.5 Cost-Effectiveness

**Estimated Costs**:
- R&D and clinical trials: $500M - $1B
- Per-patient manufacturing (AAV or LNP): $10,000 - $50,000
- Delivery and monitoring: $20,000 - $100,000
- **Total per patient: $100,000 - $500,000**

**Cost-Effectiveness Analysis**:

**Scenario**: 50-year-old treated with longevity gene editing
- Cost: $200,000 (one-time)
- Benefit: 5 years healthspan extension (QALY gain = 4-5, assuming 0.8-1.0 quality factor)
- Cost per QALY: $40,000 - $50,000

**Comparison**:
- Acceptable threshold (US): $100,000 - $150,000 per QALY
- **Conclusion**: Cost-effective if achieves ≥3 years healthspan extension

**Lifetime Healthcare Savings**:
- Prevented diseases (cardiovascular, diabetes, dementia) save $100,000 - $500,000 in healthcare costs
- Net cost may be neutral or cost-saving over lifetime

**Society-Level Considerations**:
- Extended working years: Economic productivity gain
- Reduced disability: Lower long-term care costs
- BUT: Potential increase in total lifetime healthcare (living longer)

---

## 10. References

### 10.1 Foundational Longevity Research

1. López-Otín C, Blasco MA, Partridge L, et al. The hallmarks of aging. Cell. 2013;153(6):1194-1217.
   - Defines 9 hallmarks of aging (genomic instability, telomere attrition, epigenetic alterations, etc.)

2. Kenyon CJ. The genetics of ageing. Nature. 2010;464(7288):504-512.
   - Review of longevity genes in model organisms (DAF-2/insulin signaling, sirtuins, etc.)

3. Fontana L, Partridge L, Longo VD. Extending healthy life span--from yeast to humans. Science. 2010;328(5976):321-326.
   - Conserved longevity pathways across species

### 10.2 Epigenetic Clocks

4. Horvath S. DNA methylation age of human tissues and cell types. Genome Biol. 2013;14(10):R115.
   - Original Horvath epigenetic clock

5. Lu AT, Quach A, Wilson JG, et al. DNA methylation GrimAge strongly predicts lifespan and healthspan. Aging. 2019;11(2):303-327.
   - GrimAge clock predicting mortality

6. Belsky DW, Caspi A, Arseneault L, et al. DunedinPACE, a DNA methylation biomarker of the pace of aging. eLife. 2022;11:e73420.
   - Pace of aging measurement

### 10.3 Sirtuins and NAD+

7. Imai S, Guarente L. NAD+ and sirtuins in aging and disease. Trends Cell Biol. 2014;24(8):464-471.
   - Role of sirtuins in longevity

8. Kanfi Y, Naiman S, Amir G, et al. The sirtuin SIRT6 regulates lifespan in male mice. Nature. 2012;483(7388):218-221.
   - SIRT6 overexpression extends lifespan in mice

### 10.4 FOXO3 and Longevity

9. Willcox BJ, Donlon TA, He Q, et al. FOXO3A genotype is strongly associated with human longevity. PNAS. 2008;105(37):13987-13992.
   - FOXO3 variants in human centenarians

### 10.5 Telomeres and Aging

10. Blackburn EH, Epel ES, Lin J. Human telomere biology: A contributory and interactive factor in aging, disease risks, and protection. Science. 2015;350(6265):1193-1198.
    - Comprehensive review of telomeres and aging

11. Jaskelioff M, Muller FL, Paik JH, et al. Telomerase reactivation reverses tissue degeneration in aged telomerase-deficient mice. Nature. 2011;469(7328):102-106.
    - TERT reactivation reverses aging phenotypes in mice

### 10.6 Senescent Cells

12. Baker DJ, Wijshake T, Tchkonia T, et al. Clearance of p16Ink4a-positive senescent cells delays ageing-associated disorders. Nature. 2011;479(7372):232-236.
    - Senescent cell clearance extends healthspan in mice

13. Xu M, Pirtskhalava T, Farr JN, et al. Senolytics improve physical function and increase lifespan in old age. Nat Med. 2018;24(8):1246-1256.
    - Dasatinib + Quercetin senolytic therapy

### 10.7 Gene Editing Technologies

14. Jinek M, Chylinski K, Fonfara I, et al. A programmable dual-RNA-guided DNA endonuclease in adaptive bacterial immunity. Science. 2012;337(6096):816-821.
    - Original CRISPR-Cas9 paper

15. Komor AC, Kim YB, Packer MS, et al. Programmable editing of a target base in genomic DNA without double-stranded DNA cleavage. Nature. 2016;533(7603):420-424.
    - Base editing technology

16. Anzalone AV, Randolph PB, Davis JR, et al. Search-and-replace genome editing without double-strand breaks or donor DNA. Nature. 2019;576(7785):149-157.
    - Prime editing technology

### 10.8 Safety and Off-Target Effects

17. Tsai SQ, Zheng Z, Nguyen NT, et al. GUIDE-seq enables genome-wide profiling of off-target cleavage by CRISPR-Cas nucleases. Nat Biotechnol. 2015;33(2):187-197.
    - GUIDE-seq off-target detection method

18. Kosicki M, Tomberg K, Bradley A. Repair of double-strand breaks induced by CRISPR-Cas9 leads to large deletions and complex rearrangements. Nat Biotechnol. 2018;36(8):765-771.
    - Potential for large deletions at CRISPR sites

### 10.9 Delivery Methods

19. Wang D, Tai PWL, Gao G. Adeno-associated virus vector as a platform for gene therapy delivery. Nat Rev Drug Discov. 2019;18(5):358-378.
    - AAV vector review

20. Pardi N, Hogan MJ, Porter FW, Weissman D. mRNA vaccines — a new era in vaccinology. Nat Rev Drug Discov. 2018;17(4):261-279.
    - mRNA and lipid nanoparticle delivery

### 10.10 Ethics and Equity

21. National Academies of Sciences, Engineering, and Medicine. Human Genome Editing: Science, Ethics, and Governance. Washington, DC: The National Academies Press; 2017.
    - Comprehensive ethical framework for genome editing

22. Farahany NA, Greely HT, Hyman S, et al. The ethics of experimenting with human brain tissue. Nature. 2018;556(7702):429-432.
    - Ethical considerations for human biological interventions

---

**End of WIA-AUG-017 Specification v1.0**

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
