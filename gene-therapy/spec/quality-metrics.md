# WIA-BIO-002: Quality Metrics Specification

## Version 1.0 | 2025-01-15

---

## Table of Contents
1. [Overview](#overview)
2. [Vector Quality Metrics](#vector-quality-metrics)
3. [Manufacturing Process Controls](#manufacturing-process-controls)
4. [Potency Assays](#potency-assays)
5. [Safety Testing](#safety-testing)
6. [Clinical Monitoring Metrics](#clinical-monitoring-metrics)
7. [Release Criteria](#release-criteria)

---

## Overview

This document defines comprehensive quality metrics for gene therapy products as specified in WIA-BIO-002. These metrics ensure vector safety, efficacy, and manufacturing consistency across AAV, lentiviral, and non-viral delivery systems.

### Quality Attribute Categories

| Category | Purpose | Testing Frequency | Regulatory Requirement |
|----------|---------|-------------------|------------------------|
| **Identity** | Confirm correct vector | Every batch | Required (FDA/EMA) |
| **Purity** | Measure contaminants | Every batch | Required |
| **Potency** | Functional activity | Every batch | Required |
| **Safety** | Detect adventitious agents | Every batch + annually | Required |
| **Stability** | Shelf-life determination | Real-time + accelerated | Required for BLA |

---

## Vector Quality Metrics

### 1. Vector Genome Titer (vg/mL)

**Definition:** Total number of vector genomes per unit volume, including empty and full capsids.

**Methods:**

**qPCR (ITR-based for AAV):**
```
Standard Curve:
- Serial dilutions of linearized plasmid standard (known copy number)
- Primers: AAV2 ITR forward/reverse
- Detection: SYBR Green or TaqMan probe

Calculation:
vg/mL = (Ct-derived copy number) × (dilution factor) / (sample volume)
```

**ddPCR (Digital Droplet PCR):**
```
Advantages:
- Absolute quantification without standard curve
- Higher precision (CV <5%)
- Less susceptible to PCR inhibitors

Acceptance:
- vg/mL within ±20% of target concentration
- Inter-assay precision: CV <15%
```

**AAV-Specific Metrics:**

| Metric | Method | Acceptable Range | Clinical Significance |
|--------|--------|------------------|----------------------|
| **Full/Empty Ratio** | Analytical ultracentrifugation (AUC) | >80% full | Empty capsids compete for cell entry, reduce efficacy |
| **Vector Genome Integrity** | Southern blot, long-read sequencing | >95% full-length | Truncations reduce expression |
| **Capsid Titer (cp/mL)** | ELISA (anti-AAV antibody) | vg:cp ratio 1:1 to 1:10 | Ratios >1:10 indicate high empty content |

---

### 2. Infectious Titer (Functional Potency)

**TCID50 (Tissue Culture Infectious Dose 50%):**
```
Protocol:
1. Serial 10-fold dilutions of vector
2. Infect permissive cells (HeLa for AAV, HT1080 for LV)
3. 48-72h incubation
4. Transgene detection: ELISA, flow cytometry, or reporter assay
5. Calculate dilution where 50% of wells positive (Reed-Muench method)

Acceptance:
- vg:TCID50 ratio between 10:1 and 1000:1
- Ratios >1000:1 suggest low potency (many defective particles)
```

**Transduction Units (TU/mL) for Lentivirus:**
```
Method:
1. Transduce target cells (e.g., 293T) at multiple MOIs
2. Measure transgene+ cells by flow cytometry or qPCR
3. Calculate: TU/mL = (% transduced × cell number) / volume

Specification:
- TU/mL > 1×10⁸ for clinical-grade lentivirus
- vg:TU ratio typically 10:1 to 100:1
```

---

### 3. Purity Metrics

**Protein Purity (SDS-PAGE, Silver Stain):**

| Component | Expected MW (kDa) | Acceptance | Method |
|-----------|-------------------|------------|--------|
| **AAV VP1** | 87 | VP1:VP2:VP3 = 1:1:10 | Densitometry |
| **AAV VP2** | 73 | See ratio | Coomassie/silver stain |
| **AAV VP3** | 62 | Predominant band | SDS-PAGE |
| **Host Cell Protein (HCP)** | Variable | <100 ng/dose | ELISA |
| **Plasmid DNA** | N/A | <10 ng/dose | qPCR |

**SEC-HPLC (Size Exclusion Chromatography):**
```
Parameters:
- Column: TSKgel G5000PWXL or equivalent
- Detection: UV 260nm, 280nm
- Expected peaks: Full capsids (~25 nm), empty capsids, aggregates

Acceptance:
- Full capsids: >80% of total capsid peak area
- Aggregates: <5%
- HMW contaminants: <2%
```

**Residual Process Contaminants:**

| Contaminant | LOQ (Limit of Quantification) | Specification | Method |
|-------------|-------------------------------|---------------|--------|
| **Benzonase** | 1 ng/mL | <0.15 U/dose | ELISA |
| **Polyethylene Glycol (PEG)** | 10 μg/mL | <1000 μg/dose | Barium-Iodine assay |
| **Detergent (Triton X-100)** | 1 ppm | <10 ppm | GC-MS |
| **Endotoxin** | 0.01 EU/mL | <5 EU/kg | LAL (Limulus Amebocyte Lysate) |

---

## Manufacturing Process Controls

### In-Process Testing

**Transfection Efficiency (Triple Transfection Method):**
```
Metric: % GFP+ cells 24h post-transfection
Specification: >70% for research, >80% for clinical
Method: Flow cytometry with GFP reporter plasmid
```

**Cell Viability at Harvest:**
```
Metric: % viable cells (Trypan blue exclusion)
Specification: >90%
Rationale: Low viability → increased cellular debris, harder purification
```

**Harvest Timing:**
```
AAV: 48-72h post-transfection (peak titer)
Lentivirus: 48-72h post-transfection (before cytotoxicity)
Monitor: Viral titer kinetics by qPCR
```

---

### Purification Process Metrics

**Iodixanol Gradient Fractionation (AAV):**

| Fraction | Density (g/mL) | Expected Content | Action |
|----------|----------------|------------------|--------|
| **Top** | 1.15-1.25 | Empty capsids, debris | Discard |
| **40% Iodixanol** | 1.32-1.38 | Full AAV capsids | Collect |
| **58% Iodixanol** | 1.42-1.48 | Aggregates, free DNA | Discard |

**Column Chromatography Performance:**

```
Affinity Chromatography (POROS CaptureSelect):
- Binding capacity: >1×10¹³ vg/mL resin
- Recovery: >70%
- HCP clearance: >99%

Ion Exchange (POROS HQ/HS):
- Resolution: Full vs. empty separation
- Peak symmetry factor: 0.9-1.2
- Conductivity gradient: 0-500 mM NaCl over 20 CV
```

---

## Potency Assays

### 1. In Vitro Transduction Assay

**Standard Potency Assay:**
```
Protocol:
1. Transduce target cells (disease-relevant if possible)
2. MOI: 1×10³ to 1×10⁵ vg/cell
3. 48-72h incubation
4. Measure transgene expression

Readouts:
- Protein ELISA (e.g., Factor IX for hemophilia vectors)
- Flow cytometry (% transgene+ cells)
- Functional assay (e.g., clotting time for FIX)

Acceptance:
- Within ±30% of reference standard
- Dose-response curve: R² >0.95
```

### 2. In Vivo Biodistribution (Nonclinical)

**Animal Model Requirements:**

| Species | Purpose | Timepoints | Tissues Analyzed |
|---------|---------|------------|------------------|
| **Mouse** | Initial biodistribution | 1, 4, 12 weeks | Target organ, liver, spleen, gonads |
| **NHP (Non-Human Primate)** | Clinical prediction | 1, 4, 13, 26 weeks | All major organs + CSF for CNS vectors |

**Vector Genome Copy Number (VGCN) per Cell:**
```
Method: qPCR on extracted genomic DNA

Calculation:
VGCN/cell = (vg by qPCR) / (diploid genome copies)
           = (vg) / (ng DNA × 330ng per diploid genome)

Target Organ Specification:
- CNS vectors (AAV9): >1 vg/cell in motor neurons
- Liver vectors (AAV8): >5 vg/cell in hepatocytes
- Muscle vectors (AAV1): >10 vg/cell in myofibers
```

---

### 3. Transgene Expression Kinetics

**Time-Course Monitoring:**

| Timepoint | Expression Level | Clinical Action |
|-----------|------------------|----------------|
| **Week 1** | <10% peak | Normal lag phase |
| **Week 2-4** | Increasing | Monitor for immune response |
| **Week 4-12** | Peak expression | Assess therapeutic efficacy |
| **Month 3-12** | Stable plateau | Confirm durability |
| **Year 1+** | Gradual decline acceptable | Long-term follow-up |

**Immune-Mediated Loss Detection:**
```
Indicators:
- Rapid drop in expression (>50% in 2 weeks)
- Concurrent rise in ALT/AST (liver vectors)
- Anti-transgene antibodies (ELISA)
- T-cell responses (IFN-γ ELISpot)

Intervention:
- Initiate immunosuppression (prednisone 1-2 mg/kg)
- Taper slowly over 2-3 months
```

---

## Safety Testing

### 1. Replication-Competent Virus (RCV) Testing

**AAV RCR (Replication-Competent AAV):**
```
Method: Infectious center assay
1. Inoculate permissive cells (e.g., A549 + Ad5 helper)
2. Serial passage (3-4 rounds)
3. qPCR for rep/cap genes (should be absent in AAV vectors)

Specification: Not detected (LOD: 1 RCR per 1×10¹² vg)
Frequency: Every manufacturing run
```

**Lentivirus RCL (Replication-Competent Lentivirus):**
```
Method: C8166-SEAP indicator cell assay
1. Co-culture transduced cells with C8166 indicator cells
2. Weekly passages for 4 weeks
3. SEAP (secreted alkaline phosphatase) detection

Specification: Not detected (LOD: 1 RCL per 1×10⁸ TU)
Frequency: End-of-production cells, final product
```

---

### 2. Adventitious Agent Testing

**Sterility (USP <71>):**
```
Media:
- Fluid Thioglycollate Medium (bacteria)
- Soybean-Casein Digest Medium (fungi)

Incubation: 14 days at 20-25°C (fungi), 30-35°C (bacteria)
Specification: No growth
```

**Mycoplasma (USP <63>):**
```
Methods:
- Culture-based (28-day incubation)
- PCR (faster, 16S rRNA gene detection)

Specification: Negative by both methods
```

**Endotoxin (LAL Assay):**
```
Specification:
- In vivo AAV: <5 EU/kg patient weight
- Ex vivo lentivirus: <0.5 EU/mL for cells prior to reinfusion

Method: Chromogenic or turbidimetric LAL
```

---

### 3. Integration Site Analysis (Integrating Vectors)

**LAM-PCR Workflow:**
```
1. Genomic DNA extraction from transduced cells
2. Restriction digest (MseI)
3. Linker ligation
4. Linear amplification with LTR primer
5. Nested PCR with LTR + linker primers
6. NGS (Illumina MiSeq)
7. Bioinformatics: Map to human genome (hg38)

Analysis:
- Total unique integration sites
- Distribution: genic vs. intergenic
- Proximity to proto-oncogenes (<50kb from LMO2, MECOM, etc.)
- Clonal abundance (% of total reads)
```

**Clonality Threshold:**

| Clone Abundance | Interpretation | Action |
|-----------------|----------------|--------|
| <1% | Polyclonal, normal | Routine monitoring |
| 1-5% | Oligoclonal | Increase monitoring frequency |
| 5-10% | Single clone expansion | Safety assessment, check gene |
| >10% | Dominant clone | Immediate clinical hold, bone marrow biopsy |

---

## Clinical Monitoring Metrics

### 1. Immunogenicity Assessment

**Anti-AAV Neutralizing Antibodies (NAbs):**
```
Method: Cell-based NAb assay
1. Pre-incubate AAV with serial dilutions of patient serum
2. Transduce HeLa or HEK293 cells
3. Measure % reduction in transduction vs. no-serum control
4. Calculate NAb titer (dilution giving 50% inhibition)

Exclusion Criteria:
- Pre-existing NAb titer >1:5 for target serotype
- Post-treatment NAbs expected (transient ↑ then plateau)
```

**Anti-Transgene Antibodies:**
```
Method: ELISA
- Capture: Recombinant transgene protein
- Detection: Patient serum (IgG, IgM)
- Quantification: Comparison to standard curve

Clinical Impact:
- Neutralizing anti-Factor IX antibodies → loss of efficacy
- Anti-GFP antibodies → reporter signal loss (not therapeutic impact)
```

**T-cell Responses (IFN-γ ELISpot):**
```
Antigens:
- AAV capsid peptide pools
- Transgene protein peptide pools

Positive Response: >50 spot-forming cells (SFC) per 10⁶ PBMCs
Clinical Correlation: T-cell response to capsid → hepatotoxicity risk
```

---

### 2. Biodistribution and Shedding

**Vector Shedding Monitoring:**

| Sample Type | Collection Schedule | Method | Specification |
|-------------|---------------------|--------|---------------|
| **Blood** | Days 1, 3, 7, 14, 30 | qPCR (ITR or LTR) | Clearance by Day 30 |
| **Urine** | Days 1-14 | qPCR | Peak Day 1-3, clear by Week 4 |
| **Saliva** | Days 1-7 | qPCR | Minimal shedding expected |
| **Semen** (males) | Months 1, 3, 6 | qPCR | Contraception until 2 consecutive negatives |

**Germline Transmission Risk:**
```
Preclinical:
- Gonadal biodistribution in animals
- Sperm/ova qPCR in treated animals

Clinical:
- Contraception requirement: 6-12 months post-treatment
- Pregnancy registry for exposed partners
```

---

### 3. Liver Function Monitoring (AAV Liver Vectors)

**Hepatotoxicity Surveillance:**

| Parameter | Baseline | Monitoring Schedule | Action Threshold |
|-----------|----------|---------------------|------------------|
| **ALT** | <40 U/L | Weekly x 12 weeks | >2x ULN → start prednisone |
| **AST** | <40 U/L | Weekly x 12 weeks | >2x ULN → start prednisone |
| **Bilirubin** | <1.2 mg/dL | Weekly x 12 weeks | >1.5x ULN → hepatology consult |
| **INR** | 0.9-1.1 | Biweekly x 12 weeks | >1.3 → coagulation assessment |
| **Albumin** | 3.5-5.0 g/dL | Monthly | <3.0 → assess liver synthetic function |

**Steroid Management Protocol:**
```
Initiation: ALT/AST >2x ULN and rising
Dosing:
  - Mild (2-5x ULN): Prednisone 0.5-1 mg/kg PO daily
  - Moderate (5-10x ULN): Prednisone 1-2 mg/kg PO daily
  - Severe (>10x ULN): Methylprednisolone 1-2 mg/kg IV daily

Taper: Reduce by 5-10 mg every 1-2 weeks over 8-12 weeks
Monitoring: LFTs twice weekly during taper
```

---

## Release Criteria

### Final Product Release Testing

**Required Tests (Every Batch):**

| Test | Specification | Method | Acceptance |
|------|--------------|--------|------------|
| **Appearance** | Clear, no particulates | Visual inspection | Pass |
| **Identity** | Correct transgene | Sanger sequencing | 100% match |
| **Titer (vg/mL)** | Target ± 20% | qPCR/ddPCR | Within range |
| **Infectivity** | vg:TCID50 ratio | Cell-based assay | 10:1 to 1000:1 |
| **Purity** | Full capsids >80% | AUC/SEC-HPLC | Meet spec |
| **Potency** | Functional activity | In vitro transduction | ±30% of standard |
| **Sterility** | No growth | USP <71> | Pass |
| **Mycoplasma** | Negative | Culture + PCR | Negative |
| **Endotoxin** | <5 EU/kg | LAL | Pass |
| **RCV/RCL** | Not detected | Infectious assay | ND |

**Stability-Indicating Tests:**
```
Real-Time Stability:
- Storage: -80°C or -20°C
- Timepoints: 0, 3, 6, 12, 24 months
- Tests: Titer, infectivity, purity
- Acceptance: ≤0.5 log drop in titer

Accelerated Stability:
- Storage: 4°C, 25°C
- Timepoints: 0, 1, 2, 3 months
- Purpose: Predict shelf life, guide storage conditions
```

---

**Document Version:** 1.0
**Last Updated:** 2025-01-15
**Maintained by:** WIA BIO Working Group
**License:** Apache 2.0
