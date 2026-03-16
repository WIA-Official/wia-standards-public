# WIA-AGING Phase 1: Data Format Specification

> **Version:** 1.0.0
> **Status:** Official
> **Last Updated:** 2025-01-01
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

The WIA-AGING Data Format specification defines standardized data structures for representing aging-related health information. This specification enables interoperability between aging assessment platforms, healthcare systems, research databases, and wearable devices.

### 1.1 Purpose

The primary goals of this data format specification are:

- **Interoperability:** Enable seamless data exchange between diverse systems
- **Extensibility:** Support future biomarkers and assessment methods
- **Privacy:** Incorporate privacy-preserving mechanisms
- **Accuracy:** Ensure precise representation of biological age data

### 1.2 Scope

This specification covers:

- Biological age assessment data structures
- Biomarker data representation
- Health metrics and measurements
- Longevity intervention records
- Metadata and provenance information

---

## 2. Data Model Architecture

### 2.1 Entity Relationship

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│     Subject     │───▶│    Assessment    │───▶│   BiologicalAge │
└─────────────────┘    └──────────────────┘    └─────────────────┘
        │                      │                        │
        │                      │                        │
        ▼                      ▼                        ▼
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│  HealthMetric   │    │    Biomarker     │    │  Intervention   │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

### 2.2 Core Entities

| Entity | Description | Primary Key |
|--------|-------------|-------------|
| Subject | Individual being assessed | subject_id |
| Assessment | Complete aging evaluation | assessment_id |
| BiologicalAge | Calculated biological age | biological_age_id |
| Biomarker | Individual biomarker measurement | biomarker_id |
| HealthMetric | General health measurement | metric_id |
| Intervention | Longevity intervention record | intervention_id |

---

## 3. JSON Schema Definitions

### 3.1 Root Schema: AgingProfile

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://wia.org/aging/v1/aging-profile.schema.json",
  "title": "WIA-AGING Profile",
  "description": "Complete aging assessment profile following WIA-AGING v1.0",
  "type": "object",
  "required": ["@context", "type", "id", "subject", "assessmentDate"],
  "properties": {
    "@context": {
      "type": "array",
      "items": { "type": "string" },
      "default": ["https://www.w3.org/2018/credentials/v1", "https://wia.org/aging/v1"]
    },
    "type": {
      "type": "array",
      "items": { "type": "string" },
      "contains": { "const": "AgingProfile" }
    },
    "id": {
      "type": "string",
      "format": "uri",
      "description": "Unique identifier for this profile (URN format)"
    },
    "subject": { "$ref": "#/$defs/Subject" },
    "assessmentDate": { "type": "string", "format": "date-time" },
    "biologicalAge": { "$ref": "#/$defs/BiologicalAge" },
    "biomarkers": {
      "type": "array",
      "items": { "$ref": "#/$defs/Biomarker" }
    },
    "healthMetrics": {
      "type": "array",
      "items": { "$ref": "#/$defs/HealthMetric" }
    },
    "interventions": {
      "type": "array",
      "items": { "$ref": "#/$defs/Intervention" }
    },
    "metadata": { "$ref": "#/$defs/Metadata" }
  }
}
```

### 3.2 Subject Schema

```json
{
  "$defs": {
    "Subject": {
      "type": "object",
      "required": ["id", "chronologicalAge"],
      "properties": {
        "id": {
          "type": "string",
          "pattern": "^did:wia:[a-zA-Z0-9-]+$",
          "description": "Decentralized Identifier for the subject"
        },
        "chronologicalAge": {
          "type": "number",
          "minimum": 0,
          "maximum": 150,
          "description": "Age in years"
        },
        "dateOfBirth": {
          "type": "string",
          "format": "date",
          "description": "ISO 8601 date format (optional for privacy)"
        },
        "gender": {
          "type": "string",
          "enum": ["male", "female", "other", "prefer-not-to-say"]
        },
        "ethnicity": {
          "type": "string",
          "description": "Self-reported ethnicity (optional)"
        }
      }
    }
  }
}
```

### 3.3 BiologicalAge Schema

```json
{
  "$defs": {
    "BiologicalAge": {
      "type": "object",
      "required": ["value", "method", "timestamp"],
      "properties": {
        "value": {
          "type": "number",
          "minimum": 0,
          "maximum": 150,
          "description": "Calculated biological age in years"
        },
        "method": {
          "type": "string",
          "enum": [
            "epigenetic-horvath",
            "epigenetic-hannum",
            "epigenetic-grimage",
            "epigenetic-phenoage",
            "phenotypic-levine",
            "telomere-length",
            "transcriptomic",
            "proteomic",
            "composite"
          ],
          "description": "Method used to calculate biological age"
        },
        "confidence": {
          "type": "number",
          "minimum": 0,
          "maximum": 1,
          "description": "Confidence score (0-1)"
        },
        "ageDifference": {
          "type": "number",
          "description": "Difference from chronological age (positive = older)"
        },
        "agingRate": {
          "type": "number",
          "description": "Rate of aging (1.0 = normal, >1.0 = accelerated)"
        },
        "timestamp": {
          "type": "string",
          "format": "date-time"
        },
        "laboratory": {
          "type": "object",
          "properties": {
            "id": { "type": "string" },
            "name": { "type": "string" },
            "certification": { "type": "string" }
          }
        }
      }
    }
  }
}
```

### 3.4 Biomarker Schema

```json
{
  "$defs": {
    "Biomarker": {
      "type": "object",
      "required": ["code", "value", "unit", "timestamp"],
      "properties": {
        "code": {
          "type": "string",
          "description": "LOINC code or WIA biomarker code"
        },
        "name": {
          "type": "string",
          "description": "Human-readable biomarker name"
        },
        "value": {
          "type": "number",
          "description": "Measured value"
        },
        "unit": {
          "type": "string",
          "description": "UCUM unit code"
        },
        "referenceRange": {
          "type": "object",
          "properties": {
            "low": { "type": "number" },
            "high": { "type": "number" },
            "ageAdjusted": { "type": "boolean" }
          }
        },
        "status": {
          "type": "string",
          "enum": ["normal", "low", "high", "critical-low", "critical-high"]
        },
        "timestamp": {
          "type": "string",
          "format": "date-time"
        },
        "category": {
          "type": "string",
          "enum": [
            "inflammatory",
            "metabolic",
            "organ-function",
            "hematological",
            "epigenetic",
            "telomere",
            "hormonal",
            "oxidative-stress"
          ]
        }
      }
    }
  }
}
```

---

## 4. Standard Biomarker Catalog

### 4.1 Core Aging Biomarkers

| Code | Name | Unit | Category | Reference Range |
|------|------|------|----------|-----------------|
| WIA-AGE-001 | C-Reactive Protein (CRP) | mg/L | Inflammatory | 0-3.0 |
| WIA-AGE-002 | Interleukin-6 (IL-6) | pg/mL | Inflammatory | 0-7.0 |
| WIA-AGE-003 | Albumin | g/dL | Organ-Function | 3.4-5.4 |
| WIA-AGE-004 | Creatinine | mg/dL | Organ-Function | 0.6-1.2 |
| WIA-AGE-005 | Fasting Glucose | mg/dL | Metabolic | 70-100 |
| WIA-AGE-006 | HbA1c | % | Metabolic | 4.0-5.6 |
| WIA-AGE-007 | Lymphocyte Percentage | % | Hematological | 20-40 |
| WIA-AGE-008 | Mean Cell Volume (MCV) | fL | Hematological | 80-100 |
| WIA-AGE-009 | Red Cell Distribution Width | % | Hematological | 11.5-14.5 |
| WIA-AGE-010 | Alkaline Phosphatase | U/L | Organ-Function | 44-147 |
| WIA-AGE-011 | GDF-15 | pg/mL | Inflammatory | 200-1200 |
| WIA-AGE-012 | Cystatin C | mg/L | Organ-Function | 0.5-1.0 |

### 4.2 Epigenetic Markers

| Code | Name | Description |
|------|------|-------------|
| WIA-EPI-001 | DNA Methylation Age (Horvath) | First-generation epigenetic clock |
| WIA-EPI-002 | DNA Methylation Age (Hannum) | Blood-based epigenetic clock |
| WIA-EPI-003 | GrimAge | Mortality-predicting epigenetic clock |
| WIA-EPI-004 | PhenoAge | Phenotypic epigenetic age |
| WIA-EPI-005 | DunedinPACE | Pace of aging measurement |
| WIA-EPI-006 | Telomere Length | Leukocyte telomere length |

---

## 5. Validation Rules

### 5.1 Required Field Validation

All profiles MUST include:
- Valid JSON-LD context
- Profile type declaration
- Unique identifier (URN format)
- Subject information with chronological age
- Assessment date in ISO 8601 format

### 5.2 Biomarker Validation

- Value must be within physiologically possible range
- Unit must be valid UCUM code
- Timestamp must not be in the future
- Reference ranges must have low < high

### 5.3 Biological Age Validation

- Value must be between 0 and 150
- Must include calculation method
- Confidence score (if provided) must be 0-1
- AgingRate (if provided) must be positive

---

## 6. Example Payload

```json
{
  "@context": [
    "https://www.w3.org/2018/credentials/v1",
    "https://wia.org/aging/v1"
  ],
  "type": ["AgingProfile", "VerifiableCredential"],
  "id": "urn:uuid:a1b2c3d4-e5f6-7890-abcd-ef1234567890",
  "issuer": "did:wia:longevity-clinic-seoul",
  "issuanceDate": "2025-01-15T09:00:00Z",
  "subject": {
    "id": "did:wia:subject:kim-seoul-001",
    "chronologicalAge": 55,
    "gender": "female"
  },
  "biologicalAge": {
    "value": 48.3,
    "method": "phenotypic-levine",
    "confidence": 0.92,
    "ageDifference": -6.7,
    "agingRate": 0.88,
    "timestamp": "2025-01-15T08:30:00Z"
  },
  "biomarkers": [
    {
      "code": "WIA-AGE-001",
      "name": "C-Reactive Protein",
      "value": 0.8,
      "unit": "mg/L",
      "status": "normal",
      "category": "inflammatory",
      "timestamp": "2025-01-15T07:00:00Z"
    },
    {
      "code": "WIA-AGE-003",
      "name": "Albumin",
      "value": 4.5,
      "unit": "g/dL",
      "status": "normal",
      "category": "organ-function",
      "timestamp": "2025-01-15T07:00:00Z"
    }
  ],
  "metadata": {
    "standard": "WIA-AGING",
    "version": "1.0.0",
    "philosophy": "弘益人間"
  }
}
```

---

## 7. Versioning

This specification follows Semantic Versioning 2.0.0:
- **MAJOR:** Incompatible changes
- **MINOR:** Backwards-compatible additions
- **PATCH:** Backwards-compatible fixes

Current Version: **1.0.0**

---

**弘益人間 (홍익인간) - Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
