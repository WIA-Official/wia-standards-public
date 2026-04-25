# 📊 WIA-BIO-004: Biomarker Data Standard

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![Standard](https://img.shields.io/badge/WIA-Standard-purple.svg)]()


> **Standard ID:** WIA-BIO-004
> **Version:** 1.0.0
> **Status:** Active
> **Category:** BIO / Biotechnology & Life Sciences
> **Color:** Teal (#14B8A6)

---

## 🌟 Overview

The WIA-BIO-004 standard defines the comprehensive framework for biomarker data collection, validation, quantification, and clinical correlation, including statistical analysis methods, diagnostic performance metrics, and data exchange protocols.

**홍익인간 (弘益人間) - Benefit All Humanity** - This standard aims to accelerate biomarker discovery and validation to improve disease diagnosis, treatment monitoring, and patient outcomes worldwide.

## 🎯 Key Features

- **Biomarker Discovery**: Systematic identification and characterization of biomarkers
- **Validation Framework**: Statistical methods for biomarker validation and clinical utility
- **Quantification Standards**: Precise measurement and quantification protocols
- **Clinical Correlation**: Integration with clinical outcomes and patient data
- **Diagnostic Performance**: ROC-AUC, sensitivity, specificity, and predictive value calculations
- **Data Exchange**: Standardized formats for interoperability across systems

## 📊 Core Concepts

### 1. Diagnostic Sensitivity

```
Sensitivity = TP / (TP + FN) × 100%
```

Where:
- `TP` = True Positives (correctly identified cases)
- `FN` = False Negatives (missed cases)
- High sensitivity = fewer false negatives

### 2. Diagnostic Specificity

```
Specificity = TN / (TN + FP) × 100%
```

Where:
- `TN` = True Negatives (correctly identified non-cases)
- `FP` = False Positives (false alarms)
- High specificity = fewer false positives

### 3. ROC-AUC (Area Under Curve)

```
AUC = ∫₀¹ TPR(FPR) d(FPR)
```

Where:
- `TPR` = True Positive Rate (Sensitivity)
- `FPR` = False Positive Rate (1 - Specificity)
- `AUC = 1.0` = Perfect classifier
- `AUC = 0.5` = Random classifier

### 4. Positive Predictive Value

```
PPV = TP / (TP + FP) × 100%
```

Probability that positive result indicates disease.

### 5. Negative Predictive Value

```
NPV = TN / (TN + FN) × 100%
```

Probability that negative result indicates no disease.

## 🔧 Components

### TypeScript SDK

```typescript
import {
  calculateROC,
  validateBiomarker,
  analyzeSensitivity,
  predictOutcome
} from '@wia/bio-004';

// Calculate ROC curve and AUC
const rocAnalysis = calculateROC({
  trueLabels: [1, 1, 0, 1, 0, 0, 1],
  predictions: [0.9, 0.8, 0.3, 0.85, 0.4, 0.2, 0.95],
  thresholds: [0.5, 0.6, 0.7, 0.8, 0.9]
});

console.log(`AUC: ${rocAnalysis.auc.toFixed(3)}`);
console.log(`Optimal threshold: ${rocAnalysis.optimalThreshold}`);

// Validate biomarker
const validation = validateBiomarker({
  biomarkerType: 'protein',
  measurements: [
    { value: 120, unit: 'ng/mL', diseaseStatus: true },
    { value: 85, unit: 'ng/mL', diseaseStatus: false }
  ],
  referenceRange: { min: 0, max: 100, unit: 'ng/mL' }
});

console.log(`Sensitivity: ${validation.sensitivity}%`);
console.log(`Specificity: ${validation.specificity}%`);
```

### CLI Tool

```bash
# Calculate ROC-AUC from data file
wia-bio-004 calc-roc --data biomarker_results.csv --threshold 0.5

# Validate biomarker performance
wia-bio-004 validate --type protein --data measurements.json

# Analyze sensitivity and specificity
wia-bio-004 analyze --sensitivity --specificity --data results.csv

# Generate diagnostic performance report
wia-bio-004 report --input data.csv --output report.pdf

# Predict clinical outcomes
wia-bio-004 predict --model trained_model.json --data patient_data.csv
```

## 📚 Documentation

| Document | Description |
|----------|-------------|
| [WIA-BIO-004-v1.0.md](./spec/WIA-BIO-004-v1.0.md) | Complete specification with validation methods |
| [TypeScript SDK](./api/typescript/) | Full TypeScript implementation |
| [CLI Tool](./cli/wia-bio-004.sh) | Command-line interface |

## 🚀 Quick Start

### Installation

```bash
# Clone repository
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/standards/biomarker-data

# Run installation script
./install.sh

# Verify installation
wia-bio-004 --version
```

### TypeScript Usage

```bash
# Install via npm
npm install @wia/bio-004

# Or yarn
yarn add @wia/bio-004
```

```typescript
import { BiomarkerDataSDK } from '@wia/bio-004';

const sdk = new BiomarkerDataSDK();

// Analyze biomarker performance
const performance = sdk.calculateROC({
  trueLabels: diseaseStatus,
  predictions: biomarkerValues,
  thresholds: [0.3, 0.5, 0.7, 0.9]
});

console.log(`Diagnostic Performance:`);
console.log(`  AUC: ${performance.auc.toFixed(3)}`);
console.log(`  Sensitivity: ${performance.sensitivity}%`);
console.log(`  Specificity: ${performance.specificity}%`);
console.log(`  PPV: ${performance.ppv}%`);
console.log(`  NPV: ${performance.npv}%`);
```

## 🔬 Biomarker Types

| Type | Category | Examples | Applications |
|------|----------|----------|--------------|
| Protein | Molecular | PSA, Troponin, CRP | Cancer, cardiac, inflammation |
| Genetic | Molecular | BRCA1/2, EGFR mutations | Cancer risk, targeted therapy |
| Metabolic | Molecular | Glucose, Cholesterol, Creatinine | Diabetes, cardiovascular, renal |
| Imaging | Physiological | Tumor size, Ejection fraction | Cancer staging, cardiac function |
| Cellular | Molecular | CD4 count, Circulating tumor cells | HIV, cancer monitoring |

## 📈 Performance Metrics

| Metric | Formula | Interpretation | Target |
|--------|---------|----------------|--------|
| Sensitivity | TP/(TP+FN) | Disease detection rate | >90% |
| Specificity | TN/(TN+FP) | Healthy identification rate | >95% |
| ROC-AUC | ∫TPR(FPR) | Overall discrimination | >0.80 |
| PPV | TP/(TP+FP) | Positive result accuracy | >80% |
| NPV | TN/(TN+FN) | Negative result accuracy | >95% |
| Likelihood Ratio + | Sens/(1-Spec) | Positive test impact | >10 |
| Likelihood Ratio - | (1-Sens)/Spec | Negative test impact | <0.1 |

## ⚠️ Validation Requirements

1. **Sample Size**: Minimum 100 cases and 100 controls for initial validation
2. **Statistical Power**: ≥80% power to detect clinically meaningful differences
3. **Multiple Cohorts**: Validation in ≥2 independent populations
4. **Blinding**: Measurements performed blind to clinical status
5. **Reference Standards**: Gold standard comparison for diagnostic biomarkers
6. **Reproducibility**: CV <15% for quantitative biomarkers
7. **Stability**: Pre-analytical stability documentation required

## 🌐 WIA Integration

This standard integrates with:
- **WIA-INTENT**: Intent-based biomarker queries
- **WIA-OMNI-API**: Universal biomarker data API
- **WIA-HEALTH**: Healthcare data integration
- **WIA-GENOMICS**: Genetic biomarker analysis
- **WIA-DATA**: Data standardization and exchange

## 📖 Use Cases

1. **Disease Diagnosis**: Early detection of cancer, cardiovascular disease, infections
2. **Drug Response Prediction**: Pharmacogenomics and personalized medicine
3. **Prognosis Assessment**: Predict disease progression and patient outcomes
4. **Treatment Monitoring**: Track therapeutic response and disease recurrence
5. **Risk Stratification**: Identify high-risk patients for preventive interventions
6. **Clinical Trials**: Patient selection and endpoint assessment
7. **Population Screening**: Mass screening for prevalent diseases

## 🤝 Contributing

Contributions welcome! Please see our [Contributing Guide](https://github.com/WIA-Official/wia-standards/CONTRIBUTING.md).

## 📄 License

MIT License - see [LICENSE](https://github.com/WIA-Official/wia-standards/LICENSE)

## 🔗 Links

- **Website**: [wiastandards.com](https://wiastandards.com)
- **GitHub**: [github.com/WIA-Official/wia-standards](https://github.com/WIA-Official/wia-standards)
- **Documentation**: [docs.wiastandards.com](https://docs.wiastandards.com)
- **Certification**: [cert.wiastandards.com](https://cert.wiastandards.com)

---

**홍익인간 (弘益人間) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*

---

**홍익인간 (弘益人間) - Benefit All Humanity** 🌍
