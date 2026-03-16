# WIA-BIO-003: Integration & API Specification

## Version 1.0 | 2025-01-15

## Overview

WIA-BIO-003 APIs enable biomarker discovery, validation, and clinical deployment across platforms.

### Base URL

```
https://api.wia-bio.org/biomarker/v1
```

## Biomarker Registry API

### Create Biomarker

**POST** `/v1/biomarkers`

```json
{
  "name": "Novel Alzheimer Blood Biomarker",
  "type": "diagnostic",
  "disease": "Alzheimer Disease",
  "analytes": [
    {"protein": "p-tau181", "method": "ELISA"},
    {"protein": "Abeta42", "method": "ELISA"}
  ],
  "algorithm": "ratio",
  "cutoff": 0.022,
  "performance": {
    "sensitivity": 0.91,
    "specificity": 0.88,
    "AUC": 0.92
  }
}
```

### Query Biomarkers

**GET** `/v1/biomarkers?disease=cancer&type=predictive`

```json
{
  "biomarkers": [
    {
      "id": "BM_001",
      "name": "EGFR mutation",
      "type": "predictive",
      "disease": "Non-small cell lung cancer",
      "therapy": "EGFR TKIs (erlotinib, gefitinib)",
      "validationStatus": "FDA approved"
    }
  ]
}
```

## Clinical Results API

### Submit Biomarker Result

**POST** `/v1/clinical/results`

```json
{
  "patientId": "PATIENT_001_CODED",
  "biomarkerId": "BM_001",
  "specimenId": "SPEC_12345",
  "result": {
    "value": 15,
    "unit": "score",
    "interpretation": "Low risk"
  },
  "performedBy": "GENOMIC_HEALTH_LAB",
  "performedDate": "2025-01-15"
}
```

## Validation Study API

### Register Validation Study

**POST** `/v1/validation/studies`

```json
{
  "biomarkerId": "BM_002",
  "studyType": "clinical_validation",
  "design": "prospective_cohort",
  "sampleSize": 500,
  "endpoints": ["disease_free_survival", "overall_survival"],
  "expectedCompletion": "2027-12-31"
}
```

## Python SDK

```python
from wia_bio import BiomarkerClient

client = BiomarkerClient(api_key='your_key')

# Create biomarker
biomarker = client.biomarkers.create(
    name='p-tau181/Abeta42 ratio',
    type='diagnostic',
    disease='Alzheimer Disease',
    analytes=[
        {'protein': 'p-tau181', 'method': 'ELISA'},
        {'protein': 'Abeta42', 'method': 'ELISA'}
    ]
)

# Validate biomarker
validation = client.validation.analytical(
    biomarker_id=biomarker.id,
    LOD_samples=20,
    precision_replicates=10
)

# Submit clinical result
result = client.clinical.submit_result(
    patient_id='PATIENT_001',
    biomarker_id=biomarker.id,
    value=15,
    interpretation='Low risk'
)
```

---

**Document Version:** 1.0
**Last Updated:** 2025-01-15
**Maintained by:** WIA BIO Working Group
**License:** Apache 2.0
