# WIA-BIO-003: Quality Metrics Specification

## Version 1.0 | 2025-01-15

## Table of Contents
1. [Overview](#overview)
2. [Analytical Validation Metrics](#analytical-validation-metrics)
3. [Clinical Validation Metrics](#clinical-validation-metrics)
4. [Biomarker Performance Metrics](#biomarker-performance-metrics)
5. [Quality Control](#quality-control)

## Overview

This document defines quality metrics for biomarker validation under WIA-BIO-003, covering analytical performance (LOD, LOQ, precision, accuracy) and clinical utility (sensitivity, specificity, PPV, NPV).

### Validation Levels

| Level | Purpose | Sample Size | Acceptance |
|-------|---------|-------------|------------|
| **Analytical** | Technical performance | n=20-50 per parameter | CV <20%, R² >0.95 |
| **Clinical** | Diagnostic accuracy | n=200-500+ | AUC >0.8, Sens/Spec >80% |
| **Clinical Utility** | Impact on outcomes | n=500-1000+ | Improved patient management |

## Analytical Validation Metrics

### 1. Limit of Detection (LOD)

**Formula:**
```
LOD = meanblank + 1.645 × SDblank (95% confidence)
or
LOD = meanblank + 3 × SDblank (99% confidence)
```

**Protocol:**
- Analyze 20 blank samples
- Analyze 20 low-positive samples near expected LOD
- Calculate mean and SD of blank
- Verify 95% of low samples > LOD

**Acceptance:** 95% detection rate at declared LOD

### 2. Limit of Quantification (LOQ)

**Definition:** Lowest concentration with acceptable precision (CV <20%)

**Protocol:**
- Prepare 5-7 concentration levels spanning expected range
- Analyze each level in replicates (n=10)
- Calculate CV at each level
- LOQ = lowest concentration with CV <20%

**Clinical Biomarkers:** Typically CV <15% at LOQ

### 3. Precision

**Within-Run (Repeatability):**
```
CV = (SD / Mean) × 100%

Target: CV <15% (clinical), <20% (research)
```

**Between-Run (Intermediate Precision):**
- Different days, operators, reagent lots
- Target: CV <20%

**Total Precision:**
```
CV_total = √(CV²_within + CV²_between)
```

### 4. Accuracy (Recovery)

**Spiking Study:**
```
% Recovery = (Measured - Unspiked) / Spiked × 100%

Acceptance: 80-120% recovery
```

**Reference Material:**
- Use certified reference materials (CRM) if available
- Target: ±10% of reference value

### 5. Linearity

**Protocol:**
- Prepare 5-7 concentration levels
- Analyze in duplicate
- Linear regression

**Acceptance:**
- R² >0.95 (clinical)
- R² >0.90 (research)
- Residuals randomly distributed

## Clinical Validation Metrics

### Diagnostic Performance

| Metric | Formula | Target | Interpretation |
|--------|---------|--------|----------------|
| **Sensitivity** | TP / (TP + FN) | >80% | Ability to detect disease |
| **Specificity** | TN / (TN + FP) | >90% | Ability to exclude disease |
| **PPV** | TP / (TP + FP) | Depends on prevalence | Probability disease present if +|
| **NPV** | TN / (TN + FN) | >95% for rule-out | Probability disease absent if - |
| **Accuracy** | (TP + TN) / Total | >85% | Overall correctness |
| **AUC-ROC** | Area under curve | >0.8 excellent | Discriminatory ability |

### Sample Size Calculation

```python
# For sensitivity/specificity estimation
n = (Z²_α × p × (1-p)) / d²

where:
Z_α = 1.96 (95% CI)
p = expected sensitivity/specificity
d = desired precision (e.g., ±0.05)

Example: Expected sensitivity 90%, precision ±5%
n = (1.96² × 0.9 × 0.1) / 0.05² = 138 diseased patients
```

### Clinical Utility Metrics

**Decision Curve Analysis:**
```
Net Benefit = (TP / n) - (FP / n) × (pt / (1-pt))

where pt = decision threshold probability
```

**Number Needed to Test (NNT):**
```
NNT = 1 / |Outcome_biomarker - Outcome_standard|
```

## Biomarker Performance Metrics

### Multi-Omics Biomarkers

**Integration Score:**
```
Score = Σ(wi × xi)

where wi = weight for variable i, xi = normalized value
```

**Cross-Validation:**
- k-fold CV (typically k=5 or 10)
- Leave-one-out CV (LOOCV) for small datasets
- Independent validation cohort (gold standard)

### Stability Metrics

| Test | Condition | Acceptance |
|------|-----------|------------|
| **Freeze-Thaw** | 3 cycles | <20% change |
| **Bench-top** | 4h at RT | <15% change |
| **Processed Sample** | 24h at 4°C | <15% change |
| **Long-term Storage** | -80°C for 6mo | <20% change |

## Quality Control

### Internal QC

**Control Levels:**
- Low: Near LOQ
- Medium: Mid-range
- High: Near upper limit

**Acceptance Rules (Westgard):**
- 1-2s: Warning
- 1-3s: Out of control (reject run)
- 2-2s: Two consecutive controls outside 2SD
- R-4s: Range between controls >4SD
- 4-1s: Four consecutive controls outside 1SD

### External QC (Proficiency Testing)

**Frequency:** Quarterly or biannual

**Acceptance:** Within ±2SD of peer group mean or ±20% of target value

---

**Document Version:** 1.0
**Last Updated:** 2025-01-15
**Maintained by:** WIA BIO Working Group
**License:** Apache 2.0
