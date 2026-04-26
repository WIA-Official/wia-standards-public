# WIA-BIO-005: Cellular Therapy Specification v1.0

> **Standard ID:** WIA-BIO-005
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Biotechnology Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Cell Types and Sources](#2-cell-types-and-sources)
3. [Manufacturing Processes](#3-manufacturing-processes)
4. [Quality Control Standards](#4-quality-control-standards)
5. [Cryopreservation Protocols](#5-cryopreservation-protocols)
6. [Clinical Administration](#6-clinical-administration)
7. [Safety Monitoring](#7-safety-monitoring)
8. [Implementation Guidelines](#8-implementation-guidelines)
9. [Regulatory Compliance](#9-regulatory-compliance)
10. [References](#10-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines standards for cellular therapy development, manufacturing, quality assurance, and clinical use. It covers autologous and allogeneic cell products for therapeutic applications.

### 1.2 Scope

The standard covers:
- Cell sourcing, isolation, and characterization
- Manufacturing processes and GMP compliance
- Quality control and release testing
- Cryopreservation and storage
- Clinical dosing and administration
- Safety monitoring and adverse event management

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to advance cellular therapies that treat previously incurable diseases while ensuring the highest standards of quality and patient safety.

### 1.4 Terminology

- **CAR-T**: Chimeric Antigen Receptor T cells
- **GMP**: Good Manufacturing Practice
- **CRS**: Cytokine Release Syndrome
- **ICANS**: Immune Effector Cell-Associated Neurotoxicity Syndrome
- **iPSC**: Induced Pluripotent Stem Cells
- **MSC**: Mesenchymal Stem Cells
- **Potency**: Biological activity of cell product
- **Viability**: Percentage of living cells

---

## 2. Cell Types and Sources

### 2.1 T Cell Products

#### 2.1.1 CAR-T Cells

Autologous T cells engineered to express chimeric antigen receptors:

```
Target Antigens: CD19, CD22, BCMA, CD30, HER2, GD2
Manufacturing Time: 10-14 days
Typical Dose: 1×10⁶ - 5×10⁶ cells/kg
```

**Manufacturing Steps:**
1. Leukapheresis (4-6 hours)
2. T cell isolation and activation (24-48 hours)
3. Viral transduction or electroporation (48-72 hours)
4. Expansion (7-10 days)
5. Formulation and cryopreservation (24 hours)

#### 2.1.2 TIL (Tumor Infiltrating Lymphocytes)

```
Source: Tumor biopsy
Expansion: 4-6 weeks
Typical Dose: 1×10¹⁰ - 1×10¹¹ total cells
```

### 2.2 NK Cell Products

#### 2.2.1 CAR-NK Cells

```
Source: Peripheral blood, cord blood, iPSCs
Advantages: Off-the-shelf potential, lower toxicity
Typical Dose: 1×10⁷ - 1×10⁹ cells/kg
```

### 2.3 Stem Cells

#### 2.3.1 Mesenchymal Stem Cells (MSC)

```
Sources: Bone marrow, adipose tissue, umbilical cord
Passage: P2-P5 for clinical use
Viability Requirement: >90%
Typical Dose: 1×10⁶ - 2×10⁶ cells/kg
```

**MSC Characterization:**
- Positive markers: CD73+, CD90+, CD105+
- Negative markers: CD34−, CD45−, CD11b−, CD19−, HLA-DR−
- Tri-lineage differentiation: Adipogenic, osteogenic, chondrogenic

#### 2.3.2 Induced Pluripotent Stem Cells (iPSC)

```
Reprogramming: Yamanaka factors (Oct4, Sox2, Klf4, c-Myc)
Timeline: 21-28 days
Characterization: Pluripotency markers (SSEA-4, TRA-1-60, OCT4, NANOG)
```

### 2.4 Cell Sourcing Standards

#### 2.4.1 Donor Screening

**Required Tests:**
- HIV-1/2 antibodies and NAT
- HBV surface antigen and core antibody
- HCV antibody and NAT
- HTLV-I/II antibodies
- Syphilis (RPR or equivalent)
- CMV antibody (for allogeneic products)

#### 2.4.2 Starting Material Quality

```
Minimum Cell Count: 1×10⁶ viable cells/mL (leukapheresis)
Viability: ≥85%
CD3+ Percentage: ≥30% (for T cell products)
```

---

## 3. Manufacturing Processes

### 3.1 GMP Requirements

#### 3.1.1 Facility Standards

**Cleanroom Classification:**
- Critical steps (cell culture): ISO 5 (Class 100)
- Supporting areas: ISO 7 (Class 10,000)
- Airlock/gowning: ISO 8 (Class 100,000)

**Environmental Monitoring:**
```
ISO 5 Limits:
- Particles ≥0.5 μm: ≤3,520 per m³
- Particles ≥5.0 μm: ≤29 per m³
- Viable organisms: ≤1 CFU per plate
```

#### 3.1.2 Equipment Validation

**Critical Equipment:**
1. Biological Safety Cabinets (BSC): Class II, Type A2 or B2
2. Incubators: 37°C ± 0.5°C, 5% CO₂ ± 0.5%
3. Centrifuges: Validated for cell processing
4. Cell counters: Automated with viability measurement
5. Cryogenic storage: -150°C or colder

**Qualification Requirements:**
- IQ (Installation Qualification)
- OQ (Operational Qualification)
- PQ (Performance Qualification)
- Annual requalification

### 3.2 CAR-T Manufacturing Process

#### 3.2.1 T Cell Activation

```
Method: CD3/CD28 beads or anti-CD3 antibody
Ratio: 1:3 (cells:beads) or 3:1 (cells:beads) depending on system
Duration: 24-48 hours
Target: >50% activated cells (CD25+, CD69+)
```

#### 3.2.2 Genetic Modification

**Lentiviral Transduction:**
```
MOI (Multiplicity of Infection): 1-10
Incubation: 24-48 hours
Transduction Efficiency Target: >40%
```

**Electroporation (mRNA/DNA):**
```
Voltage: 300-500V
Duration: 1-5 milliseconds
Target Viability: >70% post-electroporation
```

#### 3.2.3 Expansion

```
Starting Density: 0.5-1.0 × 10⁶ cells/mL
Target Expansion: 100-1000 fold
Duration: 7-10 days
Media: X-VIVO 15 or AIM-V with IL-2 (100 IU/mL)
Feeding Schedule: Every 48-72 hours
```

**Expansion Calculation:**
```
Expansion Ratio = Final Cell Count / Initial Cell Count
Target: ≥100 fold for CAR-T products
```

#### 3.2.4 Harvest and Formulation

```
Wash Steps: 2-3 washes with saline or PlasmaLyte
Final Concentration: 1-10 × 10⁷ cells/mL
Cryoprotectant: 10% DMSO + 5% dextran 40 or equivalent
Fill Volume: 10-50 mL per bag
```

### 3.3 Process Controls

#### 3.3.1 In-Process Testing

| Timepoint | Test | Acceptance Criteria |
|-----------|------|---------------------|
| Day 0 (start) | Viability | ≥85% |
| Day 2 (activation) | CD25/CD69 expression | ≥50% positive |
| Day 4 (transduction) | Transduction efficiency | ≥30% CAR+ |
| Day 7 (mid-expansion) | Cell count, viability | Count doubling, viability >80% |
| Day 10-14 (harvest) | Full release panel | See section 4.2 |

#### 3.3.2 Critical Process Parameters (CPP)

```
Temperature: 37°C ± 0.5°C
pH: 7.2-7.4
Dissolved Oxygen: 20-40% air saturation
Cell Density: 0.5-2.0 × 10⁶ cells/mL
Osmolality: 260-320 mOsm/kg
```

---

## 4. Quality Control Standards

### 4.1 Release Testing

#### 4.1.1 Identity Testing

**Flow Cytometry Panel:**
```
T Cell Products:
- CD3+: ≥80%
- CAR+: ≥20% (for CAR-T)
- TCRαβ+: ≥70% (for conventional T cells)

NK Cell Products:
- CD56+: ≥80%
- CD3−: ≥95%

MSC Products:
- CD73+, CD90+, CD105+: ≥95%
- CD34−, CD45−: ≥98%
```

#### 4.1.2 Viability Testing

```
Methods:
1. Trypan Blue Exclusion
2. Flow Cytometry (7-AAD or PI)
3. Automated Cell Counter

Acceptance: ≥85% viable cells
```

**Viability Calculation:**
```
Viability (%) = (Live Cells / Total Cells) × 100

Where:
- Live Cells = Cells excluding vital dye
- Total Cells = All nucleated cells counted
```

#### 4.1.3 Potency Assays

**Cytotoxicity Assay (CAR-T):**
```
Method: 51Cr release, flow cytometry, or impedance-based
Target Cells: Antigen-positive cell line
E:T Ratio: 1:1, 5:1, 10:1
Duration: 4-6 hours
Acceptance: ≥70% specific lysis at 10:1 E:T
```

**Potency Calculation:**
```
Specific Lysis (%) = [(Experimental Release - Spontaneous Release) /
                      (Maximum Release - Spontaneous Release)] × 100

Acceptance: ≥70% at highest E:T ratio
```

**Cytokine Secretion (CAR-T):**
```
Cytokines: IFN-γ, TNF-α, IL-2
Method: ELISA or multiplex assay
Acceptance: ≥2-fold increase vs unstimulated
```

#### 4.1.4 Sterility Testing

```
Method: USP <71> or equivalent
Media: Thioglycollate (bacteria) + Tryptic Soy Broth (fungi)
Duration: 14 days
Acceptance: No growth
```

#### 4.1.5 Endotoxin Testing

```
Method: LAL (Limulus Amebocyte Lysate) assay
Limit: <5 EU/kg patient dose
For 70 kg patient: <350 EU per dose
```

#### 4.1.6 Mycoplasma Testing

```
Methods:
1. Culture-based (28 days)
2. PCR-based (rapid, 24 hours)
3. Luminescence-based

Acceptance: Negative for Mycoplasma
```

### 4.2 Release Criteria Summary

| Test | Method | Specification |
|------|--------|---------------|
| Viability | Flow cytometry or Trypan Blue | ≥85% |
| Identity | Flow cytometry | Per product type |
| Potency | Cytotoxicity assay | ≥70% specific lysis |
| Sterility | USP <71> | No growth |
| Endotoxin | LAL assay | <5 EU/kg dose |
| Mycoplasma | PCR or culture | Negative |
| Appearance | Visual | Milky white, no clumps |
| Cell count | Automated counter | Per batch record |

### 4.3 Stability Studies

#### 4.3.1 Cryopreserved Product Stability

```
Storage: -150°C or colder (vapor phase LN₂)
Timepoints: 0, 3, 6, 12, 18, 24 months

Testing at Each Timepoint:
- Viability: ≥85%
- Recovery: ≥70% of starting cells
- Potency: ≥70% of T0 value
- Sterility: Negative
```

---

## 5. Cryopreservation Protocols

### 5.1 Freezing Protocol

#### 5.1.1 Cryoprotectant Formulation

```
Standard Formula:
- 90% patient plasma or human serum albumin (HSA) solution
- 10% DMSO (pharmaceutical grade)
- Optional: 5% dextran 40 or pentastarch
```

**Alternative Formulation:**
```
- 80% PlasmaLyte or Saline
- 20% HSA (25%)
- 10% DMSO
- 5% Dextrose
```

#### 5.1.2 Controlled-Rate Freezing

```
Cooling Rate: -1°C/min from 4°C to -90°C
Hold: -90°C for ≥30 minutes
Transfer: To -150°C or colder for storage

Steps:
1. Cool to 4°C
2. Add cryoprotectant dropwise with gentle mixing
3. Transfer to cryovials or bags
4. Load into controlled-rate freezer
5. Run validated freezing program
6. Transfer to vapor-phase LN₂ storage
```

### 5.2 Thawing Protocol

#### 5.2.1 Rapid Thaw Method

```
Method: 37°C water bath
Time: 2-5 minutes until small ice crystal remains
Dilution: Immediate 10-fold dilution with thaw media
Wash: 1-2 washes to remove DMSO
```

**Post-Thaw Quality:**
```
Viability: ≥85%
Recovery: ≥70%
Potency: Retained
Time to Infusion: <4 hours (ideally <1 hour)
```

#### 5.2.2 Bedside Thaw

```
For products infused immediately:
1. Thaw at 37°C (3-5 minutes)
2. Remove from water bath when small ice crystal remains
3. Infuse immediately without washing
4. Monitor patient closely (DMSO infusion reactions)
```

### 5.3 Storage Conditions

```
Temperature: -150°C or colder
Duration: Validated up to 24 months
Monitoring: Continuous temperature monitoring with alarms
Backup: Redundant LN₂ supply and backup storage
```

---

## 6. Clinical Administration

### 6.1 Dose Calculation

#### 6.1.1 Weight-Based Dosing

```
Dose (cells/kg) = Total Viable Cells / Patient Weight (kg)

Example CAR-T Doses:
- Tisagenlecleucel: 0.2-5.0 × 10⁶ CAR+ cells/kg (pediatric)
                     0.6-6.0 × 10⁸ CAR+ cells (adult, flat dose)
- Axicabtagene ciloleucel: 2 × 10⁶ CAR+ cells/kg
- Brexucabtagene autoleucel: 2 × 10⁶ CAR+ cells/kg
```

**Dose Calculation Example:**
```
Patient: 70 kg adult
Product: 2.5 × 10⁸ total viable cells, 95% viability, 80% CAR+
Target: 2 × 10⁶ CAR+ cells/kg

CAR+ Cells = 2.5 × 10⁸ × 0.95 × 0.80 = 1.9 × 10⁸ CAR+ cells
Dose = 1.9 × 10⁸ / 70 kg = 2.7 × 10⁶ cells/kg ✓ (within range)
```

#### 6.1.2 Flat Dosing

```
Total cells administered regardless of weight
Example: 1-6 × 10⁸ CAR+ cells per patient
Used for: Adult B-cell malignancies
```

### 6.2 Patient Preparation

#### 6.2.1 Lymphodepletion Chemotherapy

```
Purpose: Create "space" for CAR-T expansion
Timing: 2-7 days before CAR-T infusion

Common Regimens:
1. Fludarabine 30 mg/m²/day × 3 days + Cyclophosphamide 500 mg/m²/day × 3 days
2. Bendamustine 90 mg/m² × 2 days
3. Cyclophosphamide 300 mg/m²/day × 3 days (alone)
```

#### 6.2.2 Pre-Infusion Checklist

- [ ] Product identity verified (patient name, DOB, product ID)
- [ ] Infusion orders signed
- [ ] Pre-medications administered
- [ ] Baseline vitals recorded
- [ ] IV access confirmed (central line preferred)
- [ ] Emergency equipment available
- [ ] Tocilizumab on-site (for CAR-T)

### 6.3 Infusion Protocol

#### 6.3.1 Standard Infusion

```
Method: Gravity drip or infusion pump
Rate: 10-20 mL/min
Duration: 10-30 minutes
Flush: Normal saline to ensure complete delivery
Monitoring: Continuous vitals during and 1 hour post-infusion
```

**Infusion Steps:**
1. Verify product and patient identity
2. Record pre-infusion vitals
3. Prime tubing with normal saline
4. Connect product bag
5. Infuse at prescribed rate
6. Flush line with saline
7. Monitor patient for 1 hour minimum

#### 6.3.2 Split Dosing (if needed)

```
Day 1: 10% of total dose
Day 2: 30% of total dose
Day 3: 60% of total dose

Use for: High tumor burden, prior severe reactions
```

### 6.4 Post-Infusion Monitoring

#### 6.4.1 Inpatient Monitoring Period

```
Duration: Minimum 10 days post-CAR-T infusion
Location: Hospital with ICU capability
Frequency: Q4-6 hours vitals and neurological checks
```

**Daily Assessments:**
- Temperature, blood pressure, heart rate, respiratory rate
- Neurological examination (ICE score)
- Complete blood count
- Comprehensive metabolic panel
- CRP, ferritin, fibrinogen
- Cytokine panel (if available)

#### 6.4.2 Outpatient Monitoring

```
Duration: Up to 4 weeks post-discharge
Frequency: At least weekly clinic visits
Proximity: Patient must remain within 2 hours of treatment center
```

---

## 7. Safety Monitoring

### 7.1 Cytokine Release Syndrome (CRS)

#### 7.1.1 Grading Criteria (ASTCT 2019)

**Grade 1:**
- Fever ≥38°C

**Grade 2:**
- Fever ≥38°C
- Hypotension not requiring vasopressors OR
- Hypoxia requiring low-flow oxygen (≤6 L/min or FiO₂ <40%)

**Grade 3:**
- Fever ≥38°C
- Hypotension requiring one vasopressor with/without vasopressin OR
- Hypoxia requiring high-flow oxygen (>6 L/min or FiO₂ ≥40%)

**Grade 4:**
- Fever ≥38°C
- Hypotension requiring multiple vasopressors OR
- Hypoxia requiring positive pressure ventilation

#### 7.1.2 CRS Management Algorithm

```
Grade 1:
- Supportive care
- Antipyretics
- Monitor closely

Grade 2:
- Tocilizumab 8 mg/kg IV (max 800 mg)
- Consider corticosteroids if no improvement in 24 hours
- Repeat tocilizumab after 8 hours if needed (max 3-4 doses)

Grade 3-4:
- Tocilizumab 8 mg/kg IV immediately
- Methylprednisolone 1-2 mg/kg/day (or dexamethasone 10 mg q6h)
- Intensive care management
- Repeat tocilizumab q8h as needed
```

**Tocilizumab Dosing:**
```
Pediatric: 12 mg/kg IV (max 800 mg)
Adult: 8 mg/kg IV (max 800 mg)
Frequency: Can repeat after 8 hours if needed
Maximum: 3-4 doses total
```

#### 7.1.3 Biomarkers

```
Predictive of Severe CRS:
- Peak CRP >20 mg/dL
- Peak ferritin >10,000 ng/mL
- IL-6 >1,000 pg/mL
- IFN-γ >400 pg/mL
- Lactate >2 mmol/L
```

### 7.2 Immune Effector Cell-Associated Neurotoxicity (ICANS)

#### 7.2.1 ICE Score Assessment

**10-Point ICE Score:**

| Domain | Test | Points |
|--------|------|--------|
| Orientation | Orientation to year, month, city, hospital | 4 (1 each) |
| Naming | Name 3 objects (clock, pen, button) | 3 (1 each) |
| Following Commands | "Show me 2 fingers," "Close your eyes and stick out your tongue" | 2 (1 each) |
| Writing | Write a standard sentence | 1 |

**Scoring:**
- 10 points: Normal
- 7-9 points: Mild impairment
- 3-6 points: Moderate impairment
- 0-2 points: Severe impairment

#### 7.2.2 ICANS Grading (ASTCT 2019)

**Grade 1:**
- ICE score 7-9
- No impairment of awakening

**Grade 2:**
- ICE score 3-6
- Awakening to voice

**Grade 3:**
- ICE score 0-2
- Awakening only to tactile stimulus OR
- Seizure (any clinical or subclinical)

**Grade 4:**
- ICE score 0
- Unresponsive to tactile stimulus OR
- Motor posturing OR
- Cerebral edema

#### 7.2.3 ICANS Management

```
Grade 1:
- Monitor ICE score q4-6h
- Minimize sedation
- Neurologic checks

Grade 2:
- Dexamethasone 10 mg IV q6h
- Consider anti-seizure prophylaxis (levetiracetam 500-1500 mg BID)
- MRI brain if new focal findings
- Continuous neurologic monitoring

Grade 3-4:
- Methylprednisolone 1-2 mg/kg/day or dexamethasone 10-20 mg q6h
- Anti-seizure medication
- ICU level care
- Intubation if needed for airway protection
- Avoid tocilizumab (may worsen ICANS)
- Consider neurology consultation
```

### 7.3 Other Adverse Events

#### 7.3.1 Cytopenias

```
Monitoring: CBC at least weekly for 4 weeks
Common: Neutropenia, thrombocytopenia, anemia

Management:
- Neutropenia: G-CSF support
- Thrombocytopenia: Platelet transfusions
- Anemia: RBC transfusions
- Consider infectious prophylaxis
```

#### 7.3.2 Infections

```
Risk Period: Highest in first 30 days
Common Pathogens:
- Bacterial: Opportunistic, encapsulated organisms
- Viral: CMV, EBV, HHV-6
- Fungal: Candida, Aspergillus (if prolonged neutropenia)

Prophylaxis:
- Antibacterial: Per institutional guidelines
- Antiviral: Acyclovir or valacyclovir
- Antifungal: If ANC <500 for >7 days
- PJP: Trimethoprim-sulfamethoxazole
- IgG replacement: If IgG <400 mg/dL
```

#### 7.3.3 Tumor Lysis Syndrome

```
Prevention:
- Hydration: 2-3 L/m²/day
- Allopurinol or rasburicase
- Monitor: Electrolytes, uric acid, LDH, phosphate q6-12h

Management:
- Aggressive hydration
- Rasburicase for hyperuricemia
- Correct electrolyte abnormalities
- Consider RRT if severe
```

---

## 8. Implementation Guidelines

### 8.1 Required Components

Any WIA-BIO-005 compliant system must include:

1. **Dose Calculator**: Calculate patient-specific doses
2. **Quality Assessor**: Evaluate release criteria
3. **Manufacturing Tracker**: Monitor batch progress
4. **Safety Monitor**: Track adverse events
5. **Compliance Checker**: Verify regulatory requirements

### 8.2 API Interface

#### 8.2.1 Calculate Dose

```typescript
interface DoseRequest {
  cellType: 'CAR-T' | 'CAR-NK' | 'MSC' | 'TIL' | 'iPSC';
  totalCells: number;        // Total cell count
  viability: number;         // % (0-100)
  patientWeight: number;     // kg
  targetMarkerPercent?: number; // % CAR+ or target marker
}

interface DoseResponse {
  dose: number;              // cells/kg
  totalViableCells: number;
  withinRange: boolean;
  recommendedDose: {
    minimum: number;
    maximum: number;
  };
}
```

#### 8.2.2 Assess Quality

```typescript
interface QualityMetrics {
  viability: number;         // %
  potency: number;          // %
  sterility: boolean;
  endotoxin: number;        // EU/mL
  mycoplasma: boolean;
  identity: {
    targetMarker: number;   // % positive
    contaminants: number;   // % negative markers
  };
}

interface QualityResult {
  passRelease: boolean;
  grade: 'A' | 'B' | 'C' | 'F';
  failures: string[];
  warnings: string[];
  recommendation: 'release' | 'conditional' | 'reject';
}
```

#### 8.2.3 Track Manufacturing

```typescript
interface ManufacturingBatch {
  batchId: string;
  cellType: string;
  stage: 'collection' | 'activation' | 'transduction' | 'expansion' |
         'harvest' | 'cryopreservation' | 'complete';
  startDate: Date;
  expectedCompletion: Date;
  currentMetrics: {
    cellCount: number;
    viability: number;
    dayInCulture: number;
  };
  alerts: Alert[];
}
```

### 8.3 Data Formats

#### 8.3.1 Cell Product

```json
{
  "product_id": "CT-2024-001",
  "cell_type": "CAR-T CD19",
  "patient_id": "PT-12345",
  "manufacturing": {
    "start_date": "2024-01-15",
    "completion_date": "2024-01-29",
    "facility": "GMP Suite A"
  },
  "quality_control": {
    "viability": 92.5,
    "total_cells": 5.2e8,
    "car_positive": 85.3,
    "potency": 78.5,
    "sterility": "negative",
    "endotoxin": 0.25,
    "mycoplasma": "negative"
  },
  "storage": {
    "location": "LN2-Tank-3-A5",
    "freeze_date": "2024-01-29",
    "temperature": -185.5
  }
}
```

### 8.4 Error Handling

Standard error codes:

| Code | Meaning | Action |
|------|---------|--------|
| C001 | Viability below threshold | Reject or investigate |
| C002 | Potency failure | Reject or retest |
| C003 | Sterility failure | Reject immediately |
| C004 | Dose out of range | Recalculate or adjust |
| C005 | Identity mismatch | Stop and investigate |
| C006 | Manufacturing deviation | Document and assess impact |

---

## 9. Regulatory Compliance

### 9.1 FDA Regulations

```
21 CFR Part 1271: Human Cells, Tissues, and Cellular and Tissue-Based Products
21 CFR Part 211: Current Good Manufacturing Practice
21 CFR Part 312: Investigational New Drug Application

Reporting Requirements:
- IND Safety Reports: 15-day and 7-day reports
- Annual Reports: Manufacturing changes, protocol amendments
- Lot Release: Submit release testing data
```

### 9.2 EMA Regulations

```
Regulation (EC) No 1394/2007: Advanced Therapy Medicinal Products (ATMP)
Directive 2001/83/EC: Medicinal Products
EudraLex Volume 4: GMP Guidelines
Annex 2: Biological Active Substances and Medicinal Products
```

### 9.3 ICH Guidelines

```
ICH Q7: Good Manufacturing Practice for Active Pharmaceutical Ingredients
ICH Q10: Pharmaceutical Quality System
ICH E6: Good Clinical Practice
ICH E2A: Clinical Safety Data Management
```

---

## 10. References

### 10.1 Key Publications


2. FDA (2020). "Chemistry, Manufacturing, and Control (CMC) Information for Human Gene Therapy Investigational New Drug Applications (INDs)."



5. USP <1046>: "Cellular and Tissue-Based Products"

### 10.2 Quality Standards

| Standard | Title | Relevance |
|----------|-------|-----------|
| USP <71> | Sterility Tests | Release testing |
| USP <85> | Bacterial Endotoxins Test | Safety testing |
| USP <1043> | Ancillary Materials for Cell Therapy | Raw materials |
| ISO 13022 | Medical laboratories — Practical guide to ISO 15189 | Quality management |
| FACT-JACIE | International Standards for Cellular Therapy | Accreditation |

### 10.3 WIA Standards

- WIA-INTENT: Intent-based treatment selection
- WIA-OMNI-API: Healthcare interoperability
- WIA-HEALTH: Electronic health records
- WIA-PHARMA: Drug safety monitoring

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-BIO-005 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
