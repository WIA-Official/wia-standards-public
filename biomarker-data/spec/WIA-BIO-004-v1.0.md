# WIA-BIO-004: Biomarker Data Specification v1.0

> **Standard ID:** WIA-BIO-004
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Biomarker Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Biomarker Classification](#2-biomarker-classification)
3. [Data Collection Standards](#3-data-collection-standards)
4. [Statistical Validation Methods](#4-statistical-validation-methods)
5. [Diagnostic Performance Metrics](#5-diagnostic-performance-metrics)
6. [Clinical Correlation Analysis](#6-clinical-correlation-analysis)
7. [Data Exchange Formats](#7-data-exchange-formats)
8. [Implementation Guidelines](#8-implementation-guidelines)
9. [Privacy and Security](#9-privacy-and-security)
10. [References](#10-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the comprehensive framework for biomarker data management, including discovery, validation, quantification, and clinical correlation of biological markers used in disease diagnosis, prognosis, and treatment monitoring.

### 1.2 Scope

The standard covers:
- Biomarker types and classification
- Data collection and measurement protocols
- Statistical validation and performance assessment
- Clinical utility evaluation
- Data standardization and interoperability
- Quality control and assurance

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to accelerate biomarker discovery and validation to improve global healthcare outcomes through early disease detection, personalized medicine, and better treatment monitoring.

### 1.4 Terminology

- **Biomarker**: A measurable biological characteristic indicating normal processes, disease, or response to therapy
- **Sensitivity**: Proportion of true positive results among diseased individuals
- **Specificity**: Proportion of true negative results among healthy individuals
- **ROC Curve**: Receiver Operating Characteristic curve showing diagnostic performance
- **AUC**: Area Under the ROC Curve, overall measure of diagnostic accuracy
- **PPV/NPV**: Positive/Negative Predictive Values

---

## 2. Biomarker Classification

### 2.1 Molecular Biomarkers

#### 2.1.1 Protein Biomarkers

Protein-based markers measured in blood, tissue, or body fluids:

```
Examples:
- PSA (Prostate-Specific Antigen): Prostate cancer
- Troponin: Cardiac damage
- CRP (C-Reactive Protein): Inflammation
- HbA1c: Diabetes control
```

Measurement methods:
- ELISA (Enzyme-Linked Immunosorbent Assay)
- Mass spectrometry
- Immunohistochemistry
- Western blotting

#### 2.1.2 Genetic Biomarkers

DNA/RNA-based markers:

```
Examples:
- BRCA1/2 mutations: Breast/ovarian cancer risk
- EGFR mutations: Lung cancer targeted therapy
- BCR-ABL fusion: Chronic myeloid leukemia
- HLA alleles: Autoimmune disease risk
```

Measurement methods:
- PCR (Polymerase Chain Reaction)
- Next-generation sequencing
- Microarray analysis
- qRT-PCR for gene expression

#### 2.1.3 Metabolic Biomarkers

Small molecule metabolites:

```
Examples:
- Glucose: Diabetes
- Cholesterol/Lipids: Cardiovascular disease
- Creatinine: Kidney function
- Lactate: Tissue hypoxia
```

Measurement methods:
- Clinical chemistry analyzers
- LC-MS/MS (Liquid Chromatography-Mass Spectrometry)
- NMR (Nuclear Magnetic Resonance)

### 2.2 Imaging Biomarkers

Quantitative imaging features:

```
Examples:
- Tumor size: Cancer progression
- Ejection fraction: Cardiac function
- Bone density: Osteoporosis
- Brain atrophy: Neurodegeneration
```

Imaging modalities:
- CT (Computed Tomography)
- MRI (Magnetic Resonance Imaging)
- PET (Positron Emission Tomography)
- Ultrasound

### 2.3 Cellular Biomarkers

Cell-based markers:

```
Examples:
- CD4 count: HIV/AIDS monitoring
- Circulating tumor cells: Cancer metastasis
- Neutrophil count: Infection/inflammation
- Platelet count: Clotting disorders
```

Measurement methods:
- Flow cytometry
- Cell sorting
- Microscopy
- Automated cell counting

---

## 3. Data Collection Standards

### 3.1 Sample Requirements

#### 3.1.1 Sample Types

```json
{
  "sampleTypes": [
    "blood_serum",
    "blood_plasma",
    "whole_blood",
    "urine",
    "tissue_biopsy",
    "cerebrospinal_fluid",
    "saliva",
    "stool"
  ]
}
```

#### 3.1.2 Pre-analytical Requirements

```
Sample collection:
- Fasting status (if required)
- Time of day (circadian variation)
- Patient position (sitting/standing)
- Tourniquet time (<1 minute for blood)

Sample processing:
- Time to processing (<2 hours for plasma)
- Centrifugation speed and time
- Storage temperature
- Freeze-thaw cycles (<3)
```

### 3.2 Measurement Standards

#### 3.2.1 Analytical Performance

Required analytical characteristics:

```
Precision (Repeatability):
CV (Coefficient of Variation) <10% for diagnostic use
CV <15% for screening use

Accuracy:
Recovery: 90-110%
Bias: <5% from reference method

Linearity:
R² >0.95 across measuring range

Detection Limits:
LOD (Limit of Detection): <10% of reference range lower limit
LOQ (Limit of Quantification): <20% of reference range lower limit
```

#### 3.2.2 Quality Control

```
Internal QC:
- Run controls at 2-3 levels per batch
- Westgard rules for QC acceptance
- Monthly QC data review

External QC:
- Participate in proficiency testing
- Compare with peer laboratories
- Annual method validation
```

### 3.3 Reference Ranges

#### 3.3.1 Establishing Reference Intervals

```
Population requirements:
- Minimum 120 reference individuals
- Age, sex, ethnicity representation
- Health status verification
- Lifestyle factors documentation

Statistical method:
- Non-parametric: 2.5th to 97.5th percentile
- Parametric: Mean ± 1.96 × SD
- Box-Cox transformation for skewed data
```

#### 3.3.2 Reference Range Format

```json
{
  "biomarker": "PSA",
  "unit": "ng/mL",
  "referenceRanges": [
    {
      "ageGroup": "40-49",
      "sex": "male",
      "lowerLimit": 0.0,
      "upperLimit": 2.5,
      "median": 0.7
    },
    {
      "ageGroup": "50-59",
      "sex": "male",
      "lowerLimit": 0.0,
      "upperLimit": 3.5,
      "median": 1.0
    }
  ]
}
```

---

## 4. Statistical Validation Methods

### 4.1 Diagnostic Accuracy Assessment

#### 4.1.1 Sensitivity Calculation

Sensitivity measures the proportion of diseased individuals correctly identified:

```
Sensitivity = TP / (TP + FN) × 100%

Where:
- TP = True Positives (diseased, test positive)
- FN = False Negatives (diseased, test negative)
```

Example calculation:
```
Given: 85 diseased patients
       75 test positive (TP)
       10 test negative (FN)

Sensitivity = 75 / (75 + 10) × 100%
            = 75 / 85 × 100%
            = 88.2%
```

#### 4.1.2 Specificity Calculation

Specificity measures the proportion of healthy individuals correctly identified:

```
Specificity = TN / (TN + FP) × 100%

Where:
- TN = True Negatives (healthy, test negative)
- FP = False Positives (healthy, test positive)
```

Example calculation:
```
Given: 100 healthy individuals
       95 test negative (TN)
       5 test positive (FP)

Specificity = 95 / (95 + 5) × 100%
            = 95 / 100 × 100%
            = 95.0%
```

#### 4.1.3 Predictive Values

**Positive Predictive Value (PPV)**:
```
PPV = TP / (TP + FP) × 100%

PPV depends on disease prevalence:
PPV = (Sensitivity × Prevalence) /
      [(Sensitivity × Prevalence) + ((1-Specificity) × (1-Prevalence))]
```

**Negative Predictive Value (NPV)**:
```
NPV = TN / (TN + FN) × 100%

NPV = (Specificity × (1-Prevalence)) /
      [(Specificity × (1-Prevalence)) + ((1-Sensitivity) × Prevalence)]
```

### 4.2 ROC Analysis

#### 4.2.1 ROC Curve Construction

The ROC curve plots True Positive Rate (Sensitivity) vs False Positive Rate (1-Specificity):

```
For each threshold t:
  TPR(t) = TP(t) / (TP(t) + FN(t))
  FPR(t) = FP(t) / (FP(t) + TN(t))

Plot: (FPR(t), TPR(t)) for all thresholds
```

#### 4.2.2 AUC Calculation

Area Under the ROC Curve using trapezoidal rule:

```
AUC = Σᵢ [(FPRᵢ₊₁ - FPRᵢ) × (TPRᵢ₊₁ + TPRᵢ) / 2]

Interpretation:
- AUC = 1.0: Perfect discrimination
- AUC = 0.9-0.99: Excellent
- AUC = 0.8-0.89: Good
- AUC = 0.7-0.79: Fair
- AUC = 0.5-0.69: Poor
- AUC = 0.5: Random (no discrimination)
```

#### 4.2.3 Optimal Threshold Selection

Methods for selecting optimal cutoff:

```
1. Youden Index (J):
   J = Sensitivity + Specificity - 1
   Choose threshold that maximizes J

2. Closest to (0,1):
   Distance = √[(1-Sensitivity)² + (1-Specificity)²]
   Minimize distance to perfect classifier

3. Cost-weighted:
   Cost = (CFN × FN × Prevalence) + (CFP × FP × (1-Prevalence))
   Where CFN = cost of false negative
         CFP = cost of false positive
```

### 4.3 Statistical Comparisons

#### 4.3.1 Comparing Two Biomarkers

DeLong test for comparing AUC values:

```
Test statistic:
Z = (AUC₁ - AUC₂) / SE(AUC₁ - AUC₂)

Where SE is estimated from covariance matrix

p-value < 0.05 indicates significant difference
```

#### 4.3.2 Sample Size Requirements

For biomarker validation studies:

```
n = [(Zα + Zβ)² × (AUC × (1-AUC))] / [(AUC - 0.5)² × Prevalence × (1-Prevalence)]

Where:
- Zα = Z-score for type I error (1.96 for α=0.05)
- Zβ = Z-score for type II error (0.84 for β=0.20, power=80%)
- AUC = Expected area under curve
- Prevalence = Disease prevalence in study population

Minimum recommended:
- Discovery: 50 cases, 50 controls
- Validation: 100 cases, 100 controls
- Clinical utility: 200+ cases, 200+ controls
```

---

## 5. Diagnostic Performance Metrics

### 5.1 Likelihood Ratios

#### 5.1.1 Positive Likelihood Ratio

```
LR+ = Sensitivity / (1 - Specificity)

Interpretation:
- LR+ >10: Large increase in disease probability
- LR+ 5-10: Moderate increase
- LR+ 2-5: Small increase
- LR+ 1-2: Minimal increase
- LR+ = 1: No change in probability
```

#### 5.1.2 Negative Likelihood Ratio

```
LR- = (1 - Sensitivity) / Specificity

Interpretation:
- LR- <0.1: Large decrease in disease probability
- LR- 0.1-0.2: Moderate decrease
- LR- 0.2-0.5: Small decrease
- LR- 0.5-1: Minimal decrease
- LR- = 1: No change in probability
```

### 5.2 Clinical Utility Metrics

#### 5.2.1 Net Reclassification Improvement (NRI)

```
NRI = P(up|event) - P(down|event) + P(down|non-event) - P(up|non-event)

Where:
- P(up|event) = Proportion of cases moved to higher risk category
- P(down|event) = Proportion of cases moved to lower risk category
- P(down|non-event) = Proportion of controls moved to lower risk
- P(up|non-event) = Proportion of controls moved to higher risk

NRI >0 indicates improved classification
```

#### 5.2.2 Integrated Discrimination Improvement (IDI)

```
IDI = (ΔSensitivity_events - ΔSensitivity_non-events)

Where Δ represents change in average predicted probability
```

### 5.3 Performance Reporting

#### 5.3.1 Standard Report Format

```json
{
  "biomarker": "Example Biomarker",
  "studyPopulation": {
    "cases": 150,
    "controls": 150,
    "prevalence": 0.50
  },
  "diagnosticPerformance": {
    "auc": 0.85,
    "aucCI95": [0.80, 0.90],
    "optimalThreshold": 125.5,
    "sensitivity": 82.0,
    "sensitivityCI95": [75.0, 88.0],
    "specificity": 88.0,
    "specificityCI95": [82.0, 93.0],
    "ppv": 86.0,
    "npv": 84.5,
    "lrPositive": 6.83,
    "lrNegative": 0.20,
    "accuracy": 85.0
  }
}
```

---

## 6. Clinical Correlation Analysis

### 6.1 Disease Staging

Correlation with disease severity:

```
Analysis methods:
- Spearman correlation: Biomarker vs disease stage
- Kruskal-Wallis: Compare biomarker across stages
- Trend test: Linear trend across ordinal stages

Example:
Stage I: Biomarker mean = 50 ± 10
Stage II: Biomarker mean = 75 ± 15
Stage III: Biomarker mean = 120 ± 25
Stage IV: Biomarker mean = 180 ± 40

Spearman ρ = 0.85, p < 0.001
```

### 6.2 Prognostic Value

#### 6.2.1 Survival Analysis

Cox proportional hazards model:

```
h(t) = h₀(t) × exp(β × Biomarker)

Where:
- h(t) = Hazard at time t
- h₀(t) = Baseline hazard
- β = Coefficient (log hazard ratio)
- exp(β) = Hazard ratio

Interpretation:
- HR >1: Increased risk (poor prognosis)
- HR <1: Decreased risk (good prognosis)
- HR = 1: No effect
```

#### 6.2.2 Time-Dependent ROC

```
AUC(t) = P(Biomarker_case > Biomarker_control | T = t)

Where T = time to event

Calculate AUC at multiple time points:
- 1-year AUC
- 3-year AUC
- 5-year AUC
```

### 6.3 Treatment Response

#### 6.3.1 Response Criteria

```
Complete Response (CR): Biomarker within normal range
Partial Response (PR): ≥50% decrease from baseline
Stable Disease (SD): <50% decrease, <25% increase
Progressive Disease (PD): ≥25% increase from baseline

Kinetic parameters:
- T_half: Time to 50% reduction
- Decay rate: k = -ln(0.5) / T_half
```

#### 6.3.2 Longitudinal Analysis

Mixed effects models:

```
Biomarker_ij = β₀ + β₁×Time_ij + β₂×Treatment_i +
               β₃×(Time×Treatment)_ij + u_i + ε_ij

Where:
- i = subject index
- j = time point index
- u_i = random subject effect
- ε_ij = residual error
- β₃ = Treatment effect on slope
```

---

## 7. Data Exchange Formats

### 7.1 Biomarker Measurement Format

```json
{
  "standard": "WIA-BIO-004",
  "version": "1.0.0",
  "measurement": {
    "biomarkerID": "BIO-12345",
    "biomarkerName": "C-Reactive Protein",
    "biomarkerType": "protein",
    "value": 12.5,
    "unit": "mg/L",
    "referenceRange": {
      "min": 0,
      "max": 10,
      "unit": "mg/L"
    },
    "measurementMethod": "High-sensitivity immunoturbidimetric assay",
    "laboratoryID": "LAB-789",
    "dateCollected": "2025-01-15T08:30:00Z",
    "dateMeasured": "2025-01-15T10:45:00Z",
    "sampleType": "blood_serum",
    "qualityFlags": {
      "hemolyzed": false,
      "lipemic": false,
      "icteric": false
    }
  },
  "patient": {
    "patientID": "PATIENT-456",
    "age": 55,
    "sex": "male",
    "fastingStatus": true,
    "medications": ["aspirin", "metformin"]
  },
  "clinicalContext": {
    "indication": "Cardiovascular risk assessment",
    "diseaseStatus": null,
    "priorHistory": "Type 2 diabetes"
  }
}
```

### 7.2 Validation Study Format

```json
{
  "standard": "WIA-BIO-004",
  "version": "1.0.0",
  "studyID": "VALID-2025-001",
  "studyType": "diagnostic_validation",
  "biomarker": {
    "id": "BIO-12345",
    "name": "Novel Cancer Marker",
    "type": "protein"
  },
  "population": {
    "totalParticipants": 300,
    "cases": 150,
    "controls": 150,
    "ageRange": [45, 75],
    "sexDistribution": {
      "male": 145,
      "female": 155
    },
    "ethnicity": {
      "caucasian": 180,
      "african": 60,
      "asian": 40,
      "hispanic": 20
    }
  },
  "referenceStandard": {
    "method": "Histopathological confirmation",
    "sensitivity": 100,
    "specificity": 100
  },
  "results": {
    "rocAnalysis": {
      "auc": 0.87,
      "aucCI95": [0.82, 0.92],
      "optimalThreshold": {
        "value": 125.5,
        "unit": "ng/mL",
        "sensitivity": 84.0,
        "specificity": 90.0,
        "youdenIndex": 0.74
      }
    },
    "performanceMetrics": {
      "sensitivity": 84.0,
      "sensitivityCI95": [77.5, 89.5],
      "specificity": 90.0,
      "specificityCI95": [84.5, 94.2],
      "ppv": 89.4,
      "npv": 84.9,
      "accuracy": 87.0,
      "lrPositive": 8.4,
      "lrNegative": 0.18
    },
    "subgroupAnalysis": [
      {
        "subgroup": "Age <60",
        "n": 140,
        "auc": 0.89
      },
      {
        "subgroup": "Age >=60",
        "n": 160,
        "auc": 0.85
      }
    ]
  },
  "validation": {
    "crossValidation": "10-fold",
    "externalValidation": true,
    "validationCohorts": [
      {
        "cohort": "External Cohort 1",
        "n": 200,
        "auc": 0.84
      }
    ]
  }
}
```

---

## 8. Implementation Guidelines

### 8.1 Required Components

Any WIA-BIO-004 compliant system must include:

1. **Data Collection**: Standardized biomarker measurement capture
2. **Quality Control**: Automated QC checks and validation
3. **Statistical Analysis**: ROC, sensitivity, specificity calculations
4. **Clinical Correlation**: Integration with patient outcomes
5. **Reporting**: Standardized performance reports

### 8.2 API Interface

#### 8.2.1 Calculate ROC

```typescript
interface ROCRequest {
  trueLabels: number[];      // 1 = diseased, 0 = healthy
  predictions: number[];     // Biomarker values or probabilities
  thresholds?: number[];     // Optional specific thresholds
}

interface ROCResponse {
  auc: number;
  aucCI95: [number, number];
  rocCurve: Array<{
    threshold: number;
    sensitivity: number;
    specificity: number;
    fpr: number;
    tpr: number;
  }>;
  optimalThreshold: {
    value: number;
    sensitivity: number;
    specificity: number;
    youdenIndex: number;
  };
}
```

#### 8.2.2 Validate Biomarker

```typescript
interface BiomarkerValidation {
  biomarkerType: 'protein' | 'genetic' | 'metabolic' | 'imaging' | 'cellular';
  measurements: Array<{
    value: number;
    unit: string;
    diseaseStatus: boolean;
    patientID?: string;
  }>;
  referenceRange: {
    min: number;
    max: number;
    unit: string;
  };
  cutoffThreshold?: number;
}

interface ValidationResult {
  sensitivity: number;
  specificity: number;
  ppv: number;
  npv: number;
  accuracy: number;
  confusionMatrix: {
    tp: number;
    tn: number;
    fp: number;
    fn: number;
  };
  recommendedCutoff: number;
  clinicalUtility: 'excellent' | 'good' | 'fair' | 'poor';
}
```

### 8.3 Error Handling

Standard error codes:

| Code | Meaning | Action |
|------|---------|--------|
| B001 | Insufficient sample size | Collect more samples |
| B002 | Invalid measurement values | Check data quality |
| B003 | Missing reference standard | Define gold standard |
| B004 | Poor analytical performance | Improve assay |
| B005 | Failed validation | Re-validate biomarker |
| B006 | Low statistical power | Increase sample size |

---

## 9. Privacy and Security

### 9.1 Data Protection

**HIPAA Compliance**:
- De-identification of patient data
- Encryption at rest and in transit
- Access control and audit logging
- Minimum necessary principle

**GDPR Requirements**:
- Explicit consent for data collection
- Right to access and deletion
- Data portability
- Privacy by design

### 9.2 Anonymization

```
De-identification methods:
1. Remove direct identifiers (name, SSN, etc.)
2. Date shifting (±180 days)
3. Geographic aggregation (postal code truncation)
4. Rare value suppression
5. K-anonymity (k ≥ 5)
```

### 9.3 Secure Data Exchange

```json
{
  "encryption": "AES-256-GCM",
  "keyExchange": "ECDH-P256",
  "authentication": "HMAC-SHA256",
  "transport": "TLS 1.3",
  "dataIntegrity": "SHA-256 checksum"
}
```

---

## 10. References

### 10.1 Scientific Literature

1. Pepe, M.S. (2003). "The Statistical Evaluation of Medical Tests for Classification and Prediction"
2. Zhou, X.H. et al. (2011). "Statistical Methods in Diagnostic Medicine"
3. Zweig, M.H. & Campbell, G. (1993). "ROC plots: a fundamental evaluation tool in clinical medicine"
4. DeLong, E.R. et al. (1988). "Comparing the areas under two or more correlated ROC curves"
5. FDA-NIH BEST (2016). "Biomarkers, EndpointS, and other Tools"

### 10.2 Regulatory Guidelines

| Organization | Document | Year |
|--------------|----------|------|
| FDA | Bioanalytical Method Validation | 2018 |
| EMA | Guideline on bioanalytical method validation | 2011 |
| CLSI | EP09c: Measurement Procedure Comparison | 2018 |
| CLSI | EP15-A3: User Verification of Precision | 2014 |
| CLSI | C28-A3: Reference Intervals | 2010 |

### 10.3 Statistical Methods

```
Key formulas:

Sensitivity = TP / (TP + FN)
Specificity = TN / (TN + FP)
PPV = TP / (TP + FP)
NPV = TN / (TN + FN)
Accuracy = (TP + TN) / (TP + TN + FP + FN)
Prevalence = (TP + FN) / Total
LR+ = Sensitivity / (1 - Specificity)
LR- = (1 - Sensitivity) / Specificity

Youden Index = Sensitivity + Specificity - 1

AUC Standard Error (Hanley & McNeil):
SE(AUC) = √[(AUC×(1-AUC) + (n₁-1)×(Q₁-AUC²) + (n₀-1)×(Q₂-AUC²)) / (n₁×n₀)]

Where:
- Q₁ = AUC / (2 - AUC)
- Q₂ = 2×AUC² / (1 + AUC)
- n₁ = number of diseased
- n₀ = number of healthy
```

---

## Appendix A: Example Calculations

### A.1 ROC Analysis Example

```
Given dataset:
- 50 diseased patients
- 50 healthy controls
- Biomarker measurements for all

True Labels: [1,1,1,...,0,0,0]  (50 ones, 50 zeros)
Predictions: [95, 88, 92,..., 45, 38, 52]  (biomarker values)

Threshold = 75:
- TP = 42 (diseased with value ≥75)
- FN = 8 (diseased with value <75)
- TN = 45 (healthy with value <75)
- FP = 5 (healthy with value ≥75)

Sensitivity = 42/(42+8) = 84%
Specificity = 45/(45+5) = 90%
PPV = 42/(42+5) = 89.4%
NPV = 45/(45+8) = 84.9%
Accuracy = (42+45)/100 = 87%

Repeat for all thresholds, plot ROC curve
Calculate AUC using trapezoidal rule
Result: AUC = 0.91 (95% CI: 0.85-0.97)
```

### A.2 Sample Size Calculation

```
For biomarker validation study:

Desired parameters:
- Expected AUC = 0.85
- Null hypothesis: AUC = 0.50
- α = 0.05 (two-sided)
- Power = 90% (β = 0.10)
- Prevalence = 0.50

Calculation:
Zα/2 = 1.96
Zβ = 1.28

n = [(1.96 + 1.28)² × (0.85 × 0.15)] / [(0.85 - 0.50)² × 0.50 × 0.50]
n = [10.50 × 0.1275] / [0.1225 × 0.25]
n = 1.34 / 0.031
n = 43.2

Required: 44 cases and 44 controls
Recommended with 20% dropout: 55 cases, 55 controls
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-BIO-004 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
