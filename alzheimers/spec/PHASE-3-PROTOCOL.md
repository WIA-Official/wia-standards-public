# WIA-ALZHEIMERS - Phase 3: Protocol

> **Version:** 1.0.0
> **Last Updated:** 2025-12-29
> **Status:** Complete
> **Standard ID:** WIA-MED-ALZHEIMERS

---

## 1. Overview

Phase 3 defines the measurement and intervention protocols for the WIA-ALZHEIMERS standard. These protocols ensure consistent, reproducible assessment of NAD+ homeostasis and standardized treatment approaches based on the latest 2024-2025 research.

### 1.1 Protocol Categories

1. **Measurement Protocols**: Standardized methods for NAD+ metabolism and biomarker assessment
2. **Cognitive Assessment Protocols**: Validated cognitive testing procedures
3. **Intervention Protocols**: NAD+ precursor supplementation and combination therapies
4. **Monitoring Protocols**: Follow-up schedules and response assessment

---

## 2. Measurement Protocols

### 2.1 NAD+ Metabolism Measurement

#### 2.1.1 HPLC-MS/MS Protocol (Gold Standard)

**Purpose:** Quantitative measurement of NAD+ and related metabolites in biological samples.

**Sample Requirements:**
- Blood: 5 mL whole blood in EDTA tube, process within 2 hours
- CSF: 2 mL collected via lumbar puncture, freeze immediately
- Tissue: Flash-frozen, stored at -80°C

**Analytes:**
| Metabolite | Detection Limit | Linear Range | CV% |
|------------|-----------------|--------------|-----|
| NAD+ | 0.1 μM | 0.5-100 μM | <8% |
| NADH | 0.05 μM | 0.2-50 μM | <10% |
| NMN | 0.02 μM | 0.1-20 μM | <12% |
| NR | 0.05 μM | 0.2-30 μM | <10% |
| Nicotinamide | 0.1 μM | 0.5-200 μM | <8% |

**Procedure:**
1. Sample extraction with 80% methanol (-20°C)
2. Protein precipitation (15,000g, 10 min, 4°C)
3. Supernatant collection and concentration
4. Chromatographic separation (C18 column, gradient elution)
5. MS/MS detection (MRM mode, ESI positive)

**Quality Control:**
- Internal standards: [13C]-NAD+, [D4]-nicotinamide
- Calibration curve: 6-point, r² > 0.99
- QC samples at low, medium, high concentrations

#### 2.1.2 Enzymatic Cycling Assay

**Purpose:** Rapid colorimetric/fluorometric NAD+/NADH quantification.

**Method:**
```
NAD+ + Ethanol → (ADH) → NADH + Acetaldehyde
NADH + Thiazolyl Blue → (Diaphorase) → NAD+ + Formazan (λ = 570nm)
```

**Protocol Steps:**
1. Extract cells/tissue with 0.5M HClO₄ (NAD+) or 0.5M KOH (NADH)
2. Neutralize with K₂CO₃ or HCl
3. Add cycling mixture (ADH, diaphorase, ethanol, MTT)
4. Incubate 37°C, 30 min
5. Read absorbance at 570 nm

**Reference Ranges:**
| Sample Type | NAD+ (μM) | NADH (μM) | Ratio |
|-------------|-----------|-----------|-------|
| Blood (healthy adult) | 20-40 | 2-8 | 0.1-0.3 |
| Blood (AD patient) | 12-25 | 2-6 | 0.1-0.4 |
| CSF (healthy) | 0.5-2 | 0.05-0.2 | 0.1-0.2 |
| Brain tissue | 200-800 | 20-80 | 0.1-0.15 |

#### 2.1.3 Fluorescent Biosensor Protocol (Research)

**Purpose:** Real-time NADH/NAD+ ratio imaging in live cells.

**Sensors:**
- **Peredox**: Circularly permuted GFP, sensitive to NADH/NAD+ ratio
- **SoNar**: NADH/NAD+ sensor based on T-Rex circularly permuted YFP

**Imaging Protocol:**
1. Transfect/transduce cells with sensor construct
2. Allow 48h expression
3. Image at dual wavelengths (405nm and 488nm excitation)
4. Calculate ratio (R = F405/F488)
5. Normalize to baseline and calibrate with known NADH/NAD+ ratios

### 2.2 AD Biomarker Measurement

#### 2.2.1 Plasma Biomarkers (Simoa/Lumipulse)

**Platform Requirements:**
- Simoa HD-X Analyzer (Quanterix) or
- Lumipulse G1200/600II (Fujirebio) or
- Elecsys (Roche)

**Biomarker Panel:**

| Biomarker | Platform | Sensitivity | Cutoff |
|-----------|----------|-------------|--------|
| Aβ42/40 ratio | Lumipulse | 5 pg/mL | <0.067 (positive) |
| p-tau181 | Simoa | 0.02 pg/mL | >20 pg/mL (elevated) |
| p-tau217 | UGOT | 0.05 pg/mL | >0.4 pg/mL (elevated) |
| NfL | Simoa | 0.2 pg/mL | >35 pg/mL (elevated, age-adj) |
| GFAP | Simoa | 1 pg/mL | >200 pg/mL (elevated) |

**Sample Handling:**
1. Collect blood in K2-EDTA tubes
2. Centrifuge within 2 hours (2000g, 10 min, RT)
3. Aliquot plasma (500 μL per aliquot)
4. Store at -80°C
5. Thaw only once before analysis

#### 2.2.2 CSF Biomarkers

**Collection Protocol:**
1. Patient positioned (sitting or lateral decubitus)
2. Lumbar puncture at L3-L4 or L4-L5 interspace
3. Collect 10-15 mL CSF in polypropylene tubes
4. Centrifuge if bloody (2000g, 10 min)
5. Aliquot (500 μL) and freeze within 2 hours

**Biomarker Panel:**

| Biomarker | Method | Reference Range |
|-----------|--------|-----------------|
| Aβ42 | ELISA/Lumipulse | >600 pg/mL (normal) |
| Aβ40 | ELISA/Lumipulse | 4000-8000 pg/mL |
| Total tau | Lumipulse | <300 pg/mL (normal) |
| p-tau181 | Lumipulse | <60 pg/mL (normal) |

#### 2.2.3 Neuroimaging Protocols

**Amyloid PET:**
- Tracer: [18F]Florbetapir, [18F]Flutemetamol, or [18F]Florbetaben
- Acquisition: 50-70 min post-injection (10-20 min scan)
- Analysis: Centiloid scale (CL)
  - CL < 20: Amyloid negative
  - CL 20-40: Borderline
  - CL > 40: Amyloid positive

**Tau PET:**
- Tracer: [18F]Flortaucipir (Tauvid)
- Acquisition: 80-100 min post-injection
- Analysis: SUVR (standardized uptake value ratio)
  - SUVR < 1.1: Tau negative
  - SUVR > 1.3: Tau positive

**MRI Volumetrics:**
- Sequence: 3D T1-weighted MPRAGE
- Resolution: 1mm isotropic
- Analysis: FreeSurfer or similar
  - Hippocampal volume: Compare to age-matched norms
  - Cortical thickness: AD signature regions

---

## 3. Cognitive Assessment Protocols

### 3.1 Primary Assessments

#### 3.1.1 MMSE (Mini-Mental State Examination)

**Administration:** 10-15 minutes

**Domains (30 points total):**
| Domain | Points | Items |
|--------|--------|-------|
| Orientation | 10 | Time (5), Place (5) |
| Registration | 3 | Immediate recall of 3 words |
| Attention/Calculation | 5 | Serial 7s or WORLD backward |
| Recall | 3 | Delayed recall of 3 words |
| Language | 8 | Naming, repetition, commands |
| Construction | 1 | Copy interlocking pentagons |

**Interpretation:**
- 27-30: Normal
- 21-26: Mild impairment
- 11-20: Moderate impairment
- 0-10: Severe impairment

#### 3.1.2 MoCA (Montreal Cognitive Assessment)

**Administration:** 10-15 minutes

**Domains (30 points total):**
| Domain | Points | Tests |
|--------|--------|-------|
| Visuospatial/Executive | 5 | Trail Making B, cube copy, clock |
| Naming | 3 | Animal naming (lion, rhino, camel) |
| Memory | 5 | Delayed recall (5 words) |
| Attention | 6 | Digit span, vigilance, serial 7s |
| Language | 3 | Repetition, fluency |
| Abstraction | 2 | Similarity |
| Orientation | 6 | Date, place |

**Interpretation:**
- ≥26: Normal
- 22-25: Mild Cognitive Impairment
- 17-21: Mild dementia
- <17: Moderate-severe dementia

*Note: Add 1 point if education ≤12 years*

#### 3.1.3 CDR-SB (Clinical Dementia Rating - Sum of Boxes)

**Domains Assessed (0-18 points):**
1. Memory (0-3)
2. Orientation (0-3)
3. Judgment & Problem Solving (0-3)
4. Community Affairs (0-3)
5. Home & Hobbies (0-3)
6. Personal Care (0-3)

**Global CDR Interpretation:**
- 0: Normal
- 0.5: Questionable/Very Mild
- 1: Mild
- 2: Moderate
- 3: Severe

**CDR-SB Staging:**
- 0-0.5: Normal aging
- 0.5-4: MCI / Very mild dementia
- 4.5-9: Mild dementia
- 9.5-15.5: Moderate dementia
- 16-18: Severe dementia

---

## 4. Intervention Protocols

### 4.1 NAD+ Precursor Supplementation

#### 4.1.1 Nicotinamide Riboside (NR)

**Indications:**
- NAD+ homeostasis index < 0.7
- MCI or mild AD
- No contraindications

**Protocol:**

| Phase | Duration | Dosage | Monitoring |
|-------|----------|--------|------------|
| Initiation | Week 1-2 | 250 mg once daily | Tolerability |
| Escalation | Week 3-4 | 250 mg twice daily | Adverse events |
| Maintenance | Week 5+ | 500 mg twice daily | NAD+, cognition q8w |

**Expected Outcomes:**
- NAD+ increase: 40-60% from baseline
- Cognitive improvement: 8-15% (8 weeks)
- Time to response: 4-8 weeks

**Adverse Events:**
- Common: Flushing (5%), GI upset (8%), headache (3%)
- Rare: Insomnia, pruritus

#### 4.1.2 Nicotinamide Mononucleotide (NMN)

**Protocol:**

| Phase | Duration | Dosage | Notes |
|-------|----------|--------|-------|
| Standard | Ongoing | 250-500 mg/day | Morning dose preferred |
| High-dose | 8 weeks | 500-1000 mg/day | Research setting |

**Comparison to NR:**
- Similar NAD+ boosting efficacy
- Potentially faster absorption
- More research data emerging

#### 4.1.3 P7C3-A20 (Research Protocol)

**Status:** Investigational (not approved for clinical use)

**Mechanism:** Direct NAMPT activator, restores NAD+ biosynthesis

**Preclinical Data (5xFAD mice, Cell Reports Medicine 2025):**
- Complete reversal of cognitive deficits
- Tau phosphorylation reversal
- Blood-brain barrier restoration
- Hippocampal neurogenesis increase

### 4.2 Anti-Amyloid Therapy

#### 4.2.1 Lecanemab Protocol

**Indication:** Early symptomatic AD (MCI or mild dementia), amyloid-positive

**Dosage:**
- 10 mg/kg IV every 2 weeks

**Monitoring for ARIA:**
- Baseline MRI (before first dose)
- MRI at weeks 5, 14, and 52
- Clinical monitoring for headache, confusion, dizziness

**ARIA Management:**
| Severity | MRI Findings | Action |
|----------|--------------|--------|
| Mild | ARIA-E <5cm, asymptomatic | Continue, monitor |
| Moderate | ARIA-E 5-10cm or symptomatic | Hold, monitor MRI q4w |
| Severe | ARIA-E >10cm, hemorrhage | Discontinue |

#### 4.2.2 Donanemab Protocol

**Indication:** Early symptomatic AD, amyloid and tau positive

**Dosage:**
- 700 mg IV q4 weeks × 3 doses
- Then 1400 mg IV q4 weeks

**Unique Features:**
- Targets pyroglutamate-modified Aβ (N3pG)
- Treatment until amyloid clearance
- Potential to stop after achieving amyloid negativity

### 4.3 Combination Therapy Protocol

**Recommended Combination:**
```
NAD+ Precursor (NR 500mg BID)
       +
Lifestyle Interventions
       ±
Anti-Amyloid Therapy (if eligible)
```

**Lifestyle Components:**
1. **Exercise:** 150 min/week moderate aerobic activity
2. **Diet:** Mediterranean-MIND hybrid
3. **Sleep:** 7-9 hours, optimize sleep quality
4. **Cognitive Engagement:** Brain training, social activities
5. **Stress Management:** Mindfulness, meditation

---

## 5. Monitoring Protocols

### 5.1 Initial Assessment (Week 0)

**Required:**
- [ ] Complete NAD+ metabolic profile
- [ ] MMSE and MoCA
- [ ] Plasma biomarker panel (Aβ, tau, NfL)
- [ ] Brain MRI with volumetrics
- [ ] Medical history and medication review

**Optional:**
- [ ] CSF biomarkers
- [ ] Amyloid PET
- [ ] Genetic testing (APOE)

### 5.2 Early Follow-up (Week 4-8)

**Required:**
- [ ] Plasma NAD+ level
- [ ] Brief cognitive assessment (MoCA)
- [ ] Adverse event monitoring
- [ ] Medication adherence check

### 5.3 Interim Assessment (Week 12-16)

**Required:**
- [ ] Full NAD+ metabolic profile
- [ ] Complete cognitive battery (MMSE, MoCA, CDR-SB)
- [ ] Plasma biomarkers
- [ ] Treatment response assessment

**Decision Points:**
| Response | NAD+ Change | Cognitive Change | Action |
|----------|-------------|------------------|--------|
| Full | ≥30% increase | Stable/improved | Continue |
| Partial | 15-30% increase | Stable | Optimize dose |
| Non | <15% increase | Decline | Re-evaluate, consider alternatives |

### 5.4 Long-term Monitoring (Month 6+)

**Quarterly:**
- NAD+ level
- Cognitive assessment
- Safety labs

**Semi-annually:**
- Full NAD+ profile
- Comprehensive cognitive battery
- Plasma biomarkers

**Annually:**
- Brain MRI (volumetrics)
- Full medical review
- Treatment plan reassessment

---

## 6. Protocol Variations

### 6.1 By Disease Stage

| Stage | NAD+ Frequency | Cognitive Frequency | Imaging |
|-------|----------------|---------------------|---------|
| Preclinical | Every 6 months | Annually | Baseline only |
| MCI | Every 3 months | Every 3 months | Annually |
| Mild AD | Every 2 months | Every 2 months | Every 6 months |
| Moderate AD | Monthly | Monthly | As needed |

### 6.2 By Treatment Type

| Treatment | Specific Monitoring |
|-----------|---------------------|
| NR/NMN only | NAD+ q8w, cognition q12w |
| Anti-amyloid | ARIA MRI protocol, infusion monitoring |
| Combination | Integrate both schedules |

---

## 7. References

1. Cell Reports Medicine (2025). "NAD+ homeostasis restoration reverses AD pathology in 5xFAD mice"
2. Nature Cell Death & Disease (2024). "NMN improves mitochondrial stress response"
3. Alzheimer's & Dementia: TRC2 (2025). "NR clinical trial in MCI"
4. eNeuro (2024). "Lecanemab and Donanemab clinical comparison"
5. NIA-AA Consensus Guidelines (2024). "Biomarker-based AD staging"

---

**弘益人間 (홍익인간) - Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
