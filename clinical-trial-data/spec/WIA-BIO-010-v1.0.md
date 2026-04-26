# WIA-BIO-010: Clinical Trial Data Specification v1.0

> **Standard ID:** WIA-BIO-010
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Biotechnology Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Clinical Trial Phases](#2-clinical-trial-phases)
3. [Data Collection Standards](#3-data-collection-standards)
4. [Statistical Analysis](#4-statistical-analysis)
5. [Adverse Event Reporting](#5-adverse-event-reporting)
6. [Regulatory Compliance](#6-regulatory-compliance)
7. [Data Models](#7-data-models)
8. [Implementation Guidelines](#8-implementation-guidelines)
9. [Safety Protocols](#9-safety-protocols)
10. [References](#10-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the comprehensive framework for clinical trial data management, ensuring consistency, integrity, and regulatory compliance across all phases of clinical research.

### 1.2 Scope

The standard covers:
- Trial registration and protocol management
- Patient data collection and storage (CDISC-compliant)
- Endpoint measurement and validation
- Adverse event reporting (MedDRA coding)
- Statistical analysis plans and execution
- Regulatory submission (FDA, EMA, PMDA)

### 1.3 Philosophy

**弘익人間 (Benefit All Humanity)** - This standard aims to accelerate the development of safe and effective medical treatments while maintaining the highest standards of patient safety and data integrity.

### 1.4 Terminology

- **ICF**: Informed Consent Form
- **CRF**: Case Report Form
- **EDC**: Electronic Data Capture
- **SAE**: Serious Adverse Event
- **AE**: Adverse Event
- **ITT**: Intention-To-Treat
- **PP**: Per-Protocol
- **LOCF**: Last Observation Carried Forward

---

## 2. Clinical Trial Phases

### 2.1 Phase I: Safety and Dosage

**Objective**: Determine safe dosage range and identify side effects

```
Sample Size: 20-100 healthy volunteers
Duration: Several months
Success Rate: ~70%
Key Endpoints:
  - Maximum Tolerated Dose (MTD)
  - Pharmacokinetics (PK)
  - Pharmacodynamics (PD)
```

**Primary Assessments**:
1. Dose-Limiting Toxicity (DLT)
2. Adverse event frequency and severity
3. Drug absorption, distribution, metabolism, excretion (ADME)

**Statistical Considerations**:
```
3+3 Design for dose escalation:
- Start with 3 patients at dose level 1
- If 0/3 DLT → escalate to next dose
- If 1/3 DLT → add 3 more patients
- If ≥2/3 DLT → stop, previous dose is MTD
```

### 2.2 Phase II: Efficacy and Side Effects

**Objective**: Evaluate efficacy and further assess safety

```
Sample Size: 100-300 patients with condition
Duration: Several months to 2 years
Success Rate: ~33%
Key Endpoints:
  - Preliminary efficacy measures
  - Optimal dose selection
  - Common adverse events
```

**Study Designs**:
1. Single-arm trials
2. Randomized Phase II
3. Simon's two-stage design
4. Adaptive designs

**Futility Analysis**:
```
P(success in Phase III | Phase II data) < threshold → stop
```

### 2.3 Phase III: Confirmation and Monitoring

**Objective**: Confirm efficacy and monitor adverse reactions

```
Sample Size: 300-3,000 patients
Duration: 1-4 years
Success Rate: 25-30%
Key Endpoints:
  - Primary efficacy endpoint
  - Secondary endpoints
  - Safety profile
  - Quality of life
```

**Statistical Requirements**:
- Pre-specified primary endpoint
- Type I error control (α = 0.05)
- Adequate statistical power (≥80%)
- Intention-to-treat analysis

**Regulatory Thresholds**:
```
Efficacy: p < 0.05 for primary endpoint
Safety: Benefit-risk assessment
Quality: GCP compliance
```

### 2.4 Phase IV: Post-Market Surveillance

**Objective**: Monitor long-term safety and effectiveness

```
Sample Size: Thousands to millions
Duration: Ongoing
Key Activities:
  - Real-world evidence (RWE)
  - Rare adverse event detection
  - Long-term efficacy
  - Comparative effectiveness
```

---

## 3. Data Collection Standards

### 3.1 CDISC CDASH (Clinical Data Acquisition Standards Harmonization)

**Core Domains**:

1. **Demographics (DM)**
   - Subject identifier
   - Age, sex, race, ethnicity
   - Enrollment date

2. **Adverse Events (AE)**
   - Event term
   - Start/end date
   - Severity (mild, moderate, severe)
   - Causality assessment

3. **Vital Signs (VS)**
   - Blood pressure (systolic/diastolic)
   - Heart rate
   - Temperature
   - Respiratory rate

4. **Laboratory Tests (LB)**
   - Hematology
   - Chemistry
   - Urinalysis

5. **Medications (CM/ME)**
   - Concomitant medications
   - Study medications
   - Dose, frequency, route

### 3.2 CDISC SDTM (Study Data Tabulation Model)

**Standard Structure**:
```
STUDYID: Unique study identifier
DOMAIN: Two-letter domain code (DM, AE, VS, LB, etc.)
USUBJID: Unique subject identifier
--SEQ: Sequence number within domain
--TESTCD: Test/observation code
--TEST: Test/observation name
--ORRES: Original result
--ORRESU: Original units
--STRESC: Standardized result
--STRESU: Standardized units
```

**Example SDTM Dataset (LB - Laboratory)**:
```
STUDYID | DOMAIN | USUBJID    | LBSEQ | LBTESTCD | LBTEST  | LBORRES | LBORRESU | LBSTRESC | LBSTRESU
--------|--------|------------|-------|----------|---------|---------|----------|----------|----------
STUDY01 | LB     | STUDY01-01 | 1     | HBA1C    | HbA1c   | 7.2     | %        | 7.2      | %
STUDY01 | LB     | STUDY01-01 | 2     | GLUC     | Glucose | 135     | mg/dL    | 7.49     | mmol/L
```

### 3.3 CDISC ADaM (Analysis Data Model)

**Analysis Datasets**:

1. **ADSL (Subject-Level Analysis Dataset)**
   - One record per subject
   - Demographics
   - Treatment assignment
   - Disposition
   - Key baseline variables

2. **ADAE (Adverse Event Analysis Dataset)**
   - Adverse event analysis
   - Severity, causality
   - Treatment-emergent flags

3. **ADLB (Laboratory Analysis Dataset)**
   - Laboratory parameter analysis
   - Baseline values
   - Change from baseline
   - Shift tables

**Derived Variables**:
```
CHG: Change from baseline = AVAL - BASE
PCHG: Percent change = 100 × (AVAL - BASE) / BASE
BNRIND: Baseline reference range indicator
ANRIND: Analysis reference range indicator
```

### 3.4 Electronic Data Capture (EDC)

**Requirements**:
1. 21 CFR Part 11 compliance
2. Audit trail for all changes
3. Electronic signatures
4. Data encryption
5. Role-based access control

**Validation**:
- Range checks
- Logical checks
- Query management
- Data monitoring

---

## 4. Statistical Analysis

### 4.1 Sample Size Calculation

**Formula for Continuous Outcome**:
```
n = (Zα/2 + Zβ)² × (2σ²) / Δ²
```

**Example**:
```
Given:
  α = 0.05 (two-sided) → Zα/2 = 1.96
  Power = 80% → Zβ = 0.84
  σ = 1.5 (standard deviation)
  Δ = 0.5 (minimum detectable difference)

Calculation:
  n = (1.96 + 0.84)² × (2 × 1.5²) / 0.5²
  n = 7.84 × 4.5 / 0.25
  n = 141 per group

Total sample size = 282 (accounting for 20% dropout: 353)
```

**Formula for Binary Outcome**:
```
n = [Zα/2√(2p̄(1-p̄)) + Zβ√(p₁(1-p₁) + p₂(1-p₂))]² / (p₁ - p₂)²
```

Where:
- `p₁, p₂` = Event rates in two groups
- `p̄ = (p₁ + p₂) / 2`

### 4.2 Statistical Power

**Definition**:
```
Power = 1 - β = P(Reject H₀ | H₁ is true)
```

**Factors Affecting Power**:
1. Sample size (↑n → ↑power)
2. Effect size (↑Δ → ↑power)
3. Variability (↓σ → ↑power)
4. Significance level (↑α → ↑power)

**Power Curves**:
```
For n = 100 per group, σ = 1.5, α = 0.05:
  Δ = 0.3 → Power = 42%
  Δ = 0.5 → Power = 80%
  Δ = 0.7 → Power = 96%
  Δ = 1.0 → Power = 100%
```

### 4.3 Hypothesis Testing

**Primary Endpoint Analysis**:

**Null Hypothesis (H₀)**:
```
H₀: μ₁ = μ₂  (no difference between treatments)
```

**Alternative Hypothesis (H₁)**:
```
H₁: μ₁ ≠ μ₂  (two-sided)
H₁: μ₁ > μ₂  (one-sided, superiority)
H₁: μ₁ - μ₂ > -δ  (non-inferiority)
```

**Test Statistics**:

1. **T-test** (continuous, normal):
```
t = (x̄₁ - x̄₂) / √(s²(1/n₁ + 1/n₂))
df = n₁ + n₂ - 2
```

2. **Chi-square test** (categorical):
```
χ² = Σ(O - E)² / E
df = (r-1)(c-1)
```

3. **Log-rank test** (survival):
```
χ² = [Σ(O₁ - E₁)]² / Σ V₁
df = 1
```

### 4.4 Confidence Intervals

**95% CI for Mean Difference**:
```
CI = (x̄₁ - x̄₂) ± t₀.₀₂₅,df × SE
```

**95% CI for Risk Ratio**:
```
RR = (a/n₁) / (c/n₂)
log(RR) ± 1.96 × √(1/a - 1/n₁ + 1/c - 1/n₂)
```

**95% CI for Odds Ratio**:
```
OR = (a×d) / (b×c)
log(OR) ± 1.96 × √(1/a + 1/b + 1/c + 1/d)
```

### 4.5 Multiple Testing Adjustment

**Bonferroni Correction**:
```
α_adjusted = α / m
```

**Holm-Bonferroni Method**:
```
Sort p-values: p₁ ≤ p₂ ≤ ... ≤ pₘ
Compare: pᵢ vs α/(m-i+1)
```

**False Discovery Rate (FDR)**:
```
Benjamini-Hochberg procedure
```

### 4.6 Interim Analysis

**O'Brien-Fleming Boundaries**:
```
Critical values at k equally-spaced looks:
  α₁ = 0.00001
  α₂ = 0.014
  α₃ = 0.045
  Final α = 0.05
```

**Futility Stopping**:
```
Conditional power < 20% → stop for futility
```

---

## 5. Adverse Event Reporting

### 5.1 MedDRA Coding

**Hierarchy**:
```
SOC (System Organ Class)
  ↓
HLGT (High Level Group Term)
  ↓
HLT (High Level Term)
  ↓
PT (Preferred Term)
  ↓
LLT (Lowest Level Term)
```

**Example**:
```
SOC: Cardiac disorders
HLGT: Cardiac arrhythmias
HLT: Supraventricular arrhythmias
PT: Atrial fibrillation
LLT: Atrial fibrillation paroxysmal
```

### 5.2 Severity Grading

**CTCAE (Common Terminology Criteria for Adverse Events)**:

| Grade | Description |
|-------|-------------|
| 1 | Mild; asymptomatic or mild symptoms |
| 2 | Moderate; minimal, local or noninvasive intervention indicated |
| 3 | Severe or medically significant but not immediately life-threatening |
| 4 | Life-threatening consequences; urgent intervention indicated |
| 5 | Death related to AE |

### 5.3 Causality Assessment

**WHO-UMC Criteria**:

1. **Certain**: Event with plausible temporal relationship, cannot be explained by other factors
2. **Probable/Likely**: Event with reasonable temporal relationship, unlikely to be attributed to other causes
3. **Possible**: Event with reasonable temporal relationship but could have been produced by other factors
4. **Unlikely**: Temporal relationship improbable, other factors provide plausible explanations
5. **Unassessable**: Insufficient information

### 5.4 Serious Adverse Events (SAE)

**Definition** (21 CFR 312.32):

An SAE is any AE that:
- Results in death
- Is life-threatening
- Requires hospitalization or prolongation of existing hospitalization
- Results in persistent or significant disability/incapacity
- Is a congenital anomaly/birth defect
- Requires intervention to prevent permanent impairment or damage

**Reporting Timeline**:
```
Fatal/Life-threatening: 7 calendar days
Other SAE: 15 calendar days
Follow-up information: 8 calendar days
```

### 5.5 Safety Monitoring

**Data Safety Monitoring Board (DSMB)**:

**Review Schedule**:
- Enrollment milestones (25%, 50%, 75%)
- Pre-specified time points
- After SAE threshold breach

**Stopping Rules**:
```
If P(treatment safer than control | data) < 0.05 → consider stopping
If O'Brien-Fleming boundary crossed → stop for efficacy or futility
```

---

## 6. Regulatory Compliance

### 6.1 FDA Requirements (United States)

**IND (Investigational New Drug) Application**:

Components:
1. Cover sheet (Form FDA 1571)
2. Table of contents
3. Introductory statement
4. General investigational plan
5. Investigator's brochure
6. Clinical protocol
7. Chemistry, manufacturing, and controls
8. Pharmacology and toxicology
9. Previous human experience

**NDA (New Drug Application)**:

**CTD (Common Technical Document) Format**:
```
Module 1: Administrative and prescribing information (regional)
Module 2: Common Technical Document summaries
Module 3: Quality (CMC)
Module 4: Nonclinical study reports
Module 5: Clinical study reports
```

### 6.2 EMA Requirements (European Union)

**CTA (Clinical Trial Application)**:

**EudraCT Number**: European Clinical Trials Database registration

**IMPD (Investigational Medicinal Product Dossier)**:
- Part I: Quality information
- Part II: Non-clinical information
- Part III: Clinical information

### 6.3 PMDA Requirements (Japan)

**CTN (Clinical Trial Notification)**:

Components:
1. Basic information
2. Quality information
3. Non-clinical data
4. Clinical data
5. Risk management plan

### 6.4 ICH-GCP Compliance

**Essential Documents**:

**Before Trial Initiation**:
- Protocol and amendments
- Investigator's brochure
- Ethics committee approval
- Regulatory authority approvals
- Informed consent forms

**During Trial**:
- Source documents
- CRFs
- SAE reports
- Monitoring reports
- Audit certificates

**After Trial Completion**:
- Final study report
- Documentation of statistical analysis

---

## 7. Data Models

### 7.1 Trial Registration

```json
{
  "trialId": "NCT12345678",
  "title": "Phase III Study of Drug X in Type 2 Diabetes",
  "phase": "III",
  "indication": "Type 2 Diabetes Mellitus",
  "sponsor": "Pharma Company Inc.",
  "principalInvestigator": {
    "name": "Dr. Jane Smith",
    "institution": "University Medical Center"
  },
  "primaryEndpoint": {
    "parameter": "HbA1c",
    "metric": "Change from baseline",
    "timepoint": "Week 52"
  },
  "sampleSize": {
    "planned": 600,
    "enrolled": 0,
    "randomized": 0,
    "completed": 0
  },
  "startDate": "2025-01-15",
  "estimatedCompletionDate": "2026-12-31",
  "status": "recruiting"
}
```

### 7.2 Patient Data

```json
{
  "subjectId": "STUDY01-001",
  "demographics": {
    "age": 58,
    "sex": "M",
    "race": "Asian",
    "ethnicity": "Not Hispanic or Latino"
  },
  "enrollmentDate": "2025-02-01",
  "randomization": {
    "date": "2025-02-01",
    "treatment": "Drug X",
    "stratification": {
      "baseline_hba1c": "≥8.0%",
      "diabetes_duration": ">10 years"
    }
  },
  "visits": [
    {
      "visitId": "V1",
      "visitName": "Baseline",
      "visitDate": "2025-02-01",
      "measurements": {
        "hba1c": 8.5,
        "fasting_glucose": 165,
        "weight": 85.2,
        "sbp": 132,
        "dbp": 84
      }
    }
  ]
}
```

### 7.3 Adverse Event

```json
{
  "aeId": "AE-001-015",
  "subjectId": "STUDY01-001",
  "term": "Hypoglycemia",
  "meddraCode": {
    "pt": "10020993",
    "ptTerm": "Hypoglycaemia",
    "soc": "10027433",
    "socTerm": "Metabolism and nutrition disorders"
  },
  "startDate": "2025-03-15",
  "endDate": "2025-03-15",
  "ongoing": false,
  "severity": "mild",
  "seriousness": {
    "serious": false,
    "death": false,
    "lifeThreatening": false,
    "hospitalization": false,
    "disability": false,
    "congenitalAnomaly": false,
    "other": false
  },
  "causality": "probable",
  "action": "none",
  "outcome": "recovered"
}
```

---

## 8. Implementation Guidelines

### 8.1 Required Components

Any WIA-BIO-010 compliant system must include:

1. **Trial Registration Module**: Protocol and metadata management
2. **EDC System**: Electronic data capture with validation
3. **Safety Reporting**: SAE detection and reporting
4. **Statistics Engine**: Sample size, power, analysis
5. **Regulatory Submission**: CDISC export and eCTD generation

### 8.2 API Interface

#### 8.2.1 Register Trial
```typescript
interface TrialRegistration {
  title: string;
  phase: 'I' | 'II' | 'III' | 'IV';
  indication: string;
  primaryEndpoint: string;
  sampleSize: number;
  duration: number; // weeks
}

interface Trial {
  id: string;
  registrationDate: Date;
  status: 'planned' | 'recruiting' | 'active' | 'completed' | 'terminated';
  metadata: TrialRegistration;
}
```

#### 8.2.2 Collect Patient Data
```typescript
interface PatientDataCollection {
  trialId: string;
  patientId: string;
  visit: string;
  measurements: Record<string, number>;
  laboratoryResults?: LabResults;
  adverseEvents?: AdverseEvent[];
}

interface DataCollectionResult {
  success: boolean;
  validationErrors: string[];
  warnings: string[];
  dataId: string;
}
```

#### 8.2.3 Calculate Statistics
```typescript
interface StatisticalParameters {
  alpha: number; // significance level
  power?: number; // statistical power
  effectSize: number;
  sampleSize?: number;
  testType: 'one-tailed' | 'two-tailed';
}

interface StatisticalResult {
  sampleSize?: number;
  power?: number;
  pValue?: number;
  confidenceInterval?: [number, number];
  testStatistic?: number;
}
```

### 8.3 Data Validation Rules

**Vital Signs**:
```
Systolic BP: 60-250 mmHg
Diastolic BP: 40-150 mmHg
Heart Rate: 30-200 bpm
Temperature: 35-42°C
Respiratory Rate: 6-60 breaths/min
```

**Laboratory Values**:
```
HbA1c: 4.0-15.0%
Glucose: 20-600 mg/dL
Creatinine: 0.3-15.0 mg/dL
ALT/AST: 5-2000 U/L
```

### 8.4 Error Handling

Standard error codes:

| Code | Meaning | Action |
|------|---------|--------|
| B001 | Invalid trial ID | Verify trial registration |
| B002 | Patient not enrolled | Check enrollment status |
| B003 | Validation failure | Review data entry |
| B004 | SAE not reported | Immediate reporting required |
| B005 | Statistical assumptions violated | Review analysis plan |
| B006 | Missing informed consent | Cannot proceed with data collection |

---

## 9. Safety Protocols

### 9.1 Patient Protection

**Informed Consent Elements** (21 CFR 50.25):

Required information:
1. Purpose of research
2. Expected duration
3. Procedures to be followed
4. Foreseeable risks or discomforts
5. Potential benefits
6. Alternative procedures or treatments
7. Confidentiality
8. Compensation for injury
9. Contact information
10. Voluntary participation

### 9.2 Safety Monitoring

**Real-time Monitoring**:
- Vital sign alerts (>2SD from normal)
- Laboratory value alerts (Grade 3+ abnormality)
- SAE immediate notification
- Protocol deviation tracking

**Periodic Review**:
```
Weekly: SAE review
Monthly: Safety data review
Quarterly: DSMB meeting
Annually: Regulatory safety report
```

### 9.3 Stopping Rules

**Safety Stopping**:
```
If P(experimental treatment worse than control) > 0.95 → stop
If SAE rate > pre-specified threshold → stop
If Benefit-Risk ratio unfavorable → stop
```

**Futility Stopping**:
```
Conditional power < 20% → stop for futility
Bayesian predictive probability of success < 10% → stop
```

---

## 10. References

### 10.1 Regulatory Guidance

1. FDA (2018). "E6(R2) Good Clinical Practice"
2. ICH (2016). "E9 Statistical Principles for Clinical Trials"
3. EMA (2015). "Guideline on Clinical Trials in Small Populations"
4. PMDA (2020). "Basic Principles on Global Clinical Trials"

### 10.2 Data Standards

| Standard | Version | Description |
|----------|---------|-------------|
| CDISC CDASH | 2.2 | Clinical Data Acquisition Standards |
| CDISC SDTM | 1.7 | Study Data Tabulation Model |
| CDISC ADaM | 1.1 | Analysis Data Model |
| MedDRA | 26.0 | Medical Dictionary |
| CTCAE | 5.0 | Adverse Event Grading |

### 10.3 Statistical Methods

2. Piantadosi, S. (2017). "Clinical Trials: A Methodologic Perspective"
3. Jennison, C., Turnbull, B.W. (2000). "Group Sequential Methods"

### 10.4 WIA Standards

- WIA-INTENT: Intent-based clinical queries
- WIA-OMNI-API: Universal healthcare API
- WIA-AI: AI-powered clinical analytics
- WIA-SOCIAL: Patient engagement platforms

---

## Appendix A: Example Calculations

### A.1 Sample Size for HbA1c Study

```
Given:
- α = 0.05 (two-sided)
- Power = 80%
- σ = 1.2% (HbA1c SD)
- Δ = 0.5% (clinically meaningful difference)

Calculation:
- Zα/2 = 1.96
- Zβ = 0.84
- n = (1.96 + 0.84)² × (2 × 1.2²) / 0.5²
- n = 7.84 × 2.88 / 0.25
- n = 90.3 per group

With 20% dropout: n = 113 per group
Total sample size: 226 patients
```

### A.2 Relative Risk Calculation

```
Given:
           Event   No Event   Total
Treatment    30       170      200
Control      50       150      200

RR = (30/200) / (50/200)
RR = 0.15 / 0.25
RR = 0.60

95% CI:
SE(log RR) = √(1/30 - 1/200 + 1/50 - 1/200)
SE(log RR) = √(0.0333 - 0.005 + 0.02 - 0.005)
SE(log RR) = 0.217

log(0.60) ± 1.96 × 0.217
-0.511 ± 0.425
(-0.936, -0.086)

95% CI for RR: (0.39, 0.92)
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-BIO-010 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
