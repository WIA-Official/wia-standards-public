# Phase 1: Data Format Standard

**WIA Health Standard v1.0.0**
**Status**: Draft
**Last Updated**: December 2025

---

## 1. Introduction

### 1.1 Purpose

This document defines the standard data formats for the WIA Health Standard, enabling interoperability across health and longevity applications, devices, and systems.

### 1.2 Scope

The data format standard covers:
- Biomarker measurements and aging clocks
- Genomic and epigenetic profiles
- Digital twin data structures
- Telomere measurements
- Longevity intervention tracking

### 1.3 Design Principles

1. **Extensibility**: Support for emerging longevity technologies
2. **Interoperability**: Compatible with FHIR, HL7, and GA4GH standards
3. **Privacy-First**: Built-in support for data anonymization
4. **Versioning**: Semantic versioning for schema evolution
5. **Validation**: JSON Schema for strict type checking

---

## 2. Core Data Types

### 2.1 Primitive Types

| Type | Format | Example |
|------|--------|---------|
| `uuid` | UUID v4 | `"550e8400-e29b-41d4-a716-446655440000"` |
| `timestamp` | ISO 8601 | `"2025-12-14T10:30:00Z"` |
| `date` | ISO 8601 date | `"2025-12-14"` |
| `semver` | Semantic version | `"1.0.0"` |
| `concentration` | Number with unit | `{"value": 5.2, "unit": "mg/dL"}` |

### 2.2 Common Structures

#### Measurement

```json
{
  "value": 5.2,
  "unit": "mg/dL",
  "referenceRange": {
    "low": 0,
    "high": 10,
    "unit": "mg/dL"
  },
  "timestamp": "2025-12-14T10:30:00Z",
  "method": "immunoassay",
  "laboratory": "string",
  "flags": ["normal", "abnormal", "critical"]
}
```

#### Identifier

```json
{
  "system": "https://wia.live/health/id",
  "value": "uuid"
}
```

#### Metadata

```json
{
  "version": "1.0.0",
  "createdAt": "timestamp",
  "updatedAt": "timestamp",
  "source": {
    "system": "string",
    "version": "string"
  }
}
```

---

## 3. Health Profile

The root data structure for an individual's health data.

### 3.1 Schema

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "type": "object",
  "properties": {
    "id": { "type": "string", "format": "uuid" },
    "version": { "type": "string", "pattern": "^\\d+\\.\\d+\\.\\d+$" },
    "subject": { "$ref": "#/$defs/Subject" },
    "biomarkers": { "$ref": "#/$defs/BiomarkerProfile" },
    "genomics": { "$ref": "#/$defs/GenomicProfile" },
    "epigenetics": { "$ref": "#/$defs/EpigeneticProfile" },
    "telomeres": { "$ref": "#/$defs/TelomereProfile" },
    "digitalTwin": { "$ref": "#/$defs/DigitalTwinProfile" },
    "interventions": { "$ref": "#/$defs/InterventionHistory" },
    "metadata": { "$ref": "#/$defs/Metadata" }
  },
  "required": ["id", "version", "subject", "metadata"]
}
```

### 3.2 Subject

```json
{
  "id": "uuid",
  "anonymizedId": "string",
  "birthYear": 1990,
  "biologicalSex": "male|female|other",
  "ethnicity": "string",
  "consent": {
    "dataSharing": true,
    "research": true,
    "consentDate": "2025-01-01",
    "version": "1.0"
  }
}
```

---

## 4. Biomarker Profile

### 4.1 Structure

```json
{
  "inflammatoryMarkers": {
    "crp": { "$ref": "#/$defs/Measurement" },
    "il6": { "$ref": "#/$defs/Measurement" },
    "tnfAlpha": { "$ref": "#/$defs/Measurement" },
    "il1beta": { "$ref": "#/$defs/Measurement" }
  },
  "metabolicMarkers": {
    "igf1": { "$ref": "#/$defs/Measurement" },
    "gdf15": { "$ref": "#/$defs/Measurement" },
    "glucose": { "$ref": "#/$defs/Measurement" },
    "insulin": { "$ref": "#/$defs/Measurement" },
    "hba1c": { "$ref": "#/$defs/Measurement" },
    "cholesterol": {
      "total": { "$ref": "#/$defs/Measurement" },
      "ldl": { "$ref": "#/$defs/Measurement" },
      "hdl": { "$ref": "#/$defs/Measurement" },
      "triglycerides": { "$ref": "#/$defs/Measurement" }
    }
  },
  "hormonalMarkers": {
    "testosterone": { "$ref": "#/$defs/Measurement" },
    "estradiol": { "$ref": "#/$defs/Measurement" },
    "dheas": { "$ref": "#/$defs/Measurement" },
    "cortisol": { "$ref": "#/$defs/Measurement" },
    "thyroid": {
      "tsh": { "$ref": "#/$defs/Measurement" },
      "t3": { "$ref": "#/$defs/Measurement" },
      "t4": { "$ref": "#/$defs/Measurement" }
    }
  },
  "organFunction": {
    "liver": {
      "alt": { "$ref": "#/$defs/Measurement" },
      "ast": { "$ref": "#/$defs/Measurement" },
      "ggt": { "$ref": "#/$defs/Measurement" },
      "albumin": { "$ref": "#/$defs/Measurement" }
    },
    "kidney": {
      "creatinine": { "$ref": "#/$defs/Measurement" },
      "egfr": { "$ref": "#/$defs/Measurement" },
      "bun": { "$ref": "#/$defs/Measurement" }
    }
  },
  "agingClocks": {
    "chronologicalAge": { "type": "number" },
    "biologicalAge": { "type": "number" },
    "clockType": "horvath|grimAge|phenoAge|dunedinPACE|custom",
    "ageDelta": { "type": "number" },
    "confidence": { "type": "number", "minimum": 0, "maximum": 1 },
    "calculatedAt": "timestamp",
    "algorithm": {
      "name": "string",
      "version": "string",
      "provider": "string"
    }
  }
}
```

### 4.2 Units Reference

| Marker | Standard Unit | Alternative Units |
|--------|---------------|-------------------|
| CRP | mg/L | mg/dL (÷10) |
| IL-6 | pg/mL | ng/L |
| IGF-1 | ng/mL | nmol/L (×0.131) |
| Glucose | mg/dL | mmol/L (÷18) |
| HbA1c | % | mmol/mol |
| Testosterone | ng/dL | nmol/L (÷28.8) |

---

## 5. Genomic Profile

### 5.1 Structure

```json
{
  "sequencing": {
    "type": "WGS|WES|TargetedPanel|SNPArray",
    "platform": "string",
    "coverage": { "type": "number" },
    "qualityScore": { "type": "number" },
    "date": "timestamp",
    "laboratory": "string",
    "accession": "string"
  },
  "variants": [
    {
      "gene": "string",
      "rsId": "string",
      "chromosome": "string",
      "position": { "type": "integer" },
      "reference": "string",
      "alternate": "string",
      "zygosity": "homozygous|heterozygous|hemizygous",
      "clinicalSignificance": "pathogenic|likelyPathogenic|uncertain|likelyBenign|benign",
      "condition": "string",
      "inheritance": "autosomalDominant|autosomalRecessive|xLinked|mitochondrial"
    }
  ],
  "pharmacogenomics": {
    "metabolizerStatus": {
      "CYP2D6": "poorMetabolizer|intermediateMetabolizer|normalMetabolizer|ultrarapidMetabolizer",
      "CYP2C19": "string",
      "CYP3A4": "string"
    },
    "drugResponses": [
      {
        "drug": "string",
        "gene": "string",
        "phenotype": "string",
        "recommendation": "string",
        "evidenceLevel": "1A|1B|2A|2B|3|4"
      }
    ]
  },
  "polygeneticRiskScores": [
    {
      "condition": "string",
      "score": { "type": "number" },
      "percentile": { "type": "number", "minimum": 0, "maximum": 100 },
      "riskCategory": "low|average|elevated|high",
      "algorithm": "string",
      "snpCount": { "type": "integer" }
    }
  ],
  "ancestry": {
    "populations": [
      {
        "population": "string",
        "percentage": { "type": "number" }
      }
    ],
    "haplogroups": {
      "maternal": "string",
      "paternal": "string"
    }
  }
}
```

---

## 6. Epigenetic Profile

### 6.1 Structure

```json
{
  "methylationAge": {
    "age": { "type": "number" },
    "clockType": "horvath|hannum|phenoAge|grimAge|skinBlood|pace",
    "tissue": "blood|saliva|skin|buccal",
    "cpgSites": { "type": "integer" },
    "platform": "450K|EPIC|EPICv2|sequencing",
    "calculatedAt": "timestamp"
  },
  "senescenceMarkers": {
    "p16INK4a": {
      "expression": { "type": "number" },
      "unit": "relativeExpression",
      "method": "qPCR|RNAseq"
    },
    "p21CIP1": {
      "expression": { "type": "number" },
      "unit": "relativeExpression"
    },
    "p53": {
      "expression": { "type": "number" },
      "unit": "relativeExpression"
    },
    "senescenceAssociatedBetaGalactosidase": {
      "positive": { "type": "boolean" },
      "percentage": { "type": "number" }
    }
  },
  "chromatinState": {
    "globalMethylation": { "type": "number" },
    "lineElements": { "type": "number" },
    "heterochromatinLevel": "normal|reduced|increased"
  },
  "reprogrammingHistory": [
    {
      "date": "timestamp",
      "method": "yamanakaFactors|partialReprogramming|other",
      "factors": ["Oct4", "Sox2", "Klf4", "c-Myc"],
      "duration": "string",
      "outcome": "successful|partial|unsuccessful",
      "sideEffects": ["string"]
    }
  ]
}
```

---

## 7. Telomere Profile

### 7.1 Structure

```json
{
  "measurements": [
    {
      "averageLength": {
        "value": { "type": "number" },
        "unit": "kilobases|TRF"
      },
      "shortestTelomere": {
        "value": { "type": "number" },
        "unit": "kilobases"
      },
      "method": "qPCR|TRF|FISH|FlowFISH|STELA",
      "cellType": "leukocytes|lymphocytes|granulocytes|pbmc",
      "timestamp": "timestamp",
      "laboratory": "string",
      "coefficient​OfVariation": { "type": "number" }
    }
  ],
  "telomeraseActivity": {
    "level": "undetectable|low|normal|elevated",
    "quantitative": { "type": "number" },
    "method": "TRAP|ddTRAP|qTRAP"
  },
  "ageEquivalent": {
    "years": { "type": "number" },
    "percentile": { "type": "number" },
    "referencePopulation": "string"
  },
  "interventions": [
    {
      "type": "telomeraseActivator|geneTherapy|lifestyle",
      "name": "string",
      "startDate": "date",
      "endDate": "date",
      "dosage": "string",
      "outcome": {
        "lengthChange": { "type": "number" },
        "activityChange": "string"
      }
    }
  ]
}
```

---

## 8. Digital Twin Profile

### 8.1 Structure

```json
{
  "id": "uuid",
  "version": "semver",
  "type": "wholebody|cardiac|metabolic|neurological|custom",
  "status": "active|archived|updating",
  "fidelity": "low|medium|high|research",
  "dataStreams": {
    "clinical": {
      "enabled": true,
      "sources": ["EHR", "lab", "imaging"],
      "lastSync": "timestamp"
    },
    "physiological": {
      "enabled": true,
      "sources": ["wearables", "cgm", "ecg"],
      "frequency": "realtime|hourly|daily"
    },
    "behavioral": {
      "enabled": true,
      "sources": ["activity", "sleep", "nutrition"],
      "privacy": "anonymized|pseudonymized|identified"
    },
    "environmental": {
      "enabled": true,
      "sources": ["air_quality", "uv", "temperature"]
    },
    "genomic": {
      "enabled": true,
      "dataTypes": ["variants", "expression", "methylation"]
    }
  },
  "models": [
    {
      "organ": "heart|brain|liver|kidney|pancreas|lung",
      "modelType": "physiological|mechanical|metabolic|electrical",
      "accuracy": { "type": "number", "minimum": 0, "maximum": 1 },
      "lastCalibrated": "timestamp",
      "predictions": [
        {
          "outcome": "string",
          "probability": { "type": "number" },
          "timeframe": "string",
          "confidence": { "type": "number" }
        }
      ]
    }
  ],
  "simulations": [
    {
      "id": "uuid",
      "type": "drugResponse|surgery|lifestyle|disease",
      "parameters": {},
      "results": {},
      "timestamp": "timestamp"
    }
  ],
  "lastUpdated": "timestamp",
  "nextCalibration": "timestamp"
}
```
