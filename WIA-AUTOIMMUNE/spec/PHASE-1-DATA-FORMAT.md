# WIA-AUTOIMMUNE Phase 1: Data Format Specification

> **Version:** 1.0.0
> **Status:** Official
> **Last Updated:** 2026-01-04
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

The WIA-AUTOIMMUNE Data Format specification defines standardized data structures for representing autoimmune disease profiles with focus on the **Treg-Microbiome Axis** - the unified principle connecting regulatory T cell function with gut microbiome balance.

### 1.1 Core Discovery

```
┌─────────────────────────────────────────────────────────────────────────────┐
│  💡 Unified Principle: Immune Tolerance = Treg Function + Gut Microbiome   │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  🔬 Treg Dysfunction (2024-2025):                                          │
│  • Autoimmune = Treg functional defect, not count                          │
│  • GRAIL E3 ligase reduction → IL-2R signaling impairment                  │
│  • CAR-Treg therapies in development                                       │
│                                                                             │
│  🔬 SCFA (Short-Chain Fatty Acids):                                        │
│  • Butyrate: FOXP3 stabilization, Treg function enhancement                │
│  • Gut microbiome → SCFA → Immune regulation                               │
│  • RA, SLE, MS, T1D all linked to gut dysbiosis                           │
│                                                                             │
│  🔬 Treatment Approaches:                                                   │
│  • Low-dose IL-2: Selective Treg expansion                                 │
│  • FMT: Fecal Microbiota Transplantation                                   │
│  • Synthetic receptors: Inflammation → Tolerance conversion                │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 1.2 Supported Diseases

| Code | Disease | Primary Markers |
|------|---------|-----------------|
| RA | Rheumatoid Arthritis | RF, Anti-CCP, DAS28 |
| SLE | Systemic Lupus Erythematosus | ANA, Anti-dsDNA, SLEDAI |
| MS | Multiple Sclerosis | OCB, MRI lesions, EDSS |
| T1D | Type 1 Diabetes | GAD65, IA-2, C-peptide |
| IBD | Inflammatory Bowel Disease | Calprotectin, CRP |
| PSO | Psoriasis | PASI, IL-17 |
| HT | Hashimoto's Thyroiditis | Anti-TPO, Anti-Tg |
| GD | Graves' Disease | TRAb, TSI |

---

## 2. Data Model Architecture

### 2.1 Entity Relationship

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│     Patient     │───▶│  AutoimmuneProfile│───▶│  DiseaseActivity │
└─────────────────┘    └──────────────────┘    └─────────────────┘
        │                      │                        │
        │                      │                        │
        ▼                      ▼                        ▼
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   TregStatus    │◀──▶│    Microbiome    │    │    Treatment    │
└─────────────────┘    └──────────────────┘    └─────────────────┘
         ↑                     ↑
         └─────────────────────┘
          Treg-Microbiome Axis
```

### 2.2 Core Entities

| Entity | Description | Primary Key |
|--------|-------------|-------------|
| Patient | Individual with autoimmune condition | patient_id |
| AutoimmuneProfile | Complete immunological assessment | profile_id |
| TregStatus | Regulatory T cell evaluation | treg_id |
| Microbiome | Gut microbiome analysis | microbiome_id |
| DiseaseActivity | Disease-specific activity score | activity_id |
| Treatment | Therapeutic intervention record | treatment_id |

---

## 3. JSON Schema Definitions

### 3.1 Root Schema: AutoimmuneProfile

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://wia.live/schemas/autoimmune/v1.0.0",
  "title": "WIA-AUTOIMMUNE Profile",
  "description": "Complete autoimmune assessment profile with Treg-Microbiome Axis focus",
  "type": "object",
  "required": ["@context", "type", "id", "patient_id", "disease_type", "assessment_date"],
  "properties": {
    "@context": {
      "type": "array",
      "items": { "type": "string" },
      "default": ["https://www.w3.org/2018/credentials/v1", "https://wia.live/autoimmune/v1"]
    },
    "type": {
      "type": "array",
      "items": { "type": "string" },
      "contains": { "const": "AutoimmuneProfile" }
    },
    "id": {
      "type": "string",
      "format": "uri",
      "description": "Unique identifier for this profile (URN format)"
    },
    "patient_id": {
      "type": "string",
      "format": "uuid",
      "description": "Patient unique identifier"
    },
    "disease_type": {
      "type": "string",
      "enum": ["RA", "SLE", "MS", "T1D", "IBD", "PSO", "HT", "GD"],
      "description": "Primary autoimmune disease classification"
    },
    "assessment_date": {
      "type": "string",
      "format": "date-time"
    },
    "immune_markers": { "$ref": "#/$defs/ImmuneMarkers" },
    "microbiome": { "$ref": "#/$defs/Microbiome" },
    "disease_activity": { "$ref": "#/$defs/DiseaseActivity" },
    "treatment": { "$ref": "#/$defs/Treatment" }
  }
}
```

### 3.2 Treg Status Schema

```json
{
  "$defs": {
    "TregStatus": {
      "type": "object",
      "description": "Regulatory T cell functional assessment",
      "properties": {
        "count": {
          "type": "object",
          "properties": {
            "value": { "type": "number", "minimum": 0 },
            "unit": { "type": "string", "const": "cells/μL" },
            "reference_range": {
              "type": "object",
              "properties": {
                "low": { "type": "number" },
                "high": { "type": "number" }
              }
            }
          }
        },
        "foxp3_expression": {
          "type": "number",
          "minimum": 0,
          "maximum": 100,
          "description": "FOXP3 expression level (% of CD4+ T cells)"
        },
        "suppressive_function": {
          "type": "number",
          "minimum": 0,
          "maximum": 100,
          "description": "In vitro suppressive capacity (%)"
        },
        "cd25_expression": {
          "type": "number",
          "description": "IL-2 receptor alpha chain expression"
        },
        "ctla4_expression": {
          "type": "number",
          "description": "CTLA-4 surface expression"
        },
        "grail_activity": {
          "type": "number",
          "minimum": 0,
          "maximum": 100,
          "description": "GRAIL E3 ligase activity (%)"
        },
        "stability_score": {
          "type": "number",
          "minimum": 0,
          "maximum": 100,
          "description": "Treg lineage stability score"
        }
      }
    }
  }
}
```

### 3.3 Immune Markers Schema

```json
{
  "$defs": {
    "ImmuneMarkers": {
      "type": "object",
      "description": "Comprehensive immune marker panel",
      "properties": {
        "treg": { "$ref": "#/$defs/TregStatus" },
        "th17_treg_ratio": {
          "type": "number",
          "minimum": 0,
          "description": "Th17/Treg balance ratio (elevated in autoimmunity)"
        },
        "autoantibodies": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "name": { "type": "string" },
              "code": { "type": "string" },
              "value": { "type": "number" },
              "unit": { "type": "string" },
              "positive": { "type": "boolean" },
              "titer": { "type": "string" }
            }
          }
        },
        "inflammatory_cytokines": {
          "type": "object",
          "properties": {
            "il6": { "type": "number", "description": "IL-6 (pg/mL)" },
            "il17": { "type": "number", "description": "IL-17 (pg/mL)" },
            "tnf_alpha": { "type": "number", "description": "TNF-α (pg/mL)" },
            "ifn_gamma": { "type": "number", "description": "IFN-γ (pg/mL)" },
            "il1_beta": { "type": "number", "description": "IL-1β (pg/mL)" },
            "il23": { "type": "number", "description": "IL-23 (pg/mL)" }
          }
        },
        "anti_inflammatory_cytokines": {
          "type": "object",
          "properties": {
            "il10": { "type": "number", "description": "IL-10 (pg/mL)" },
            "tgf_beta": { "type": "number", "description": "TGF-β (ng/mL)" },
            "il35": { "type": "number", "description": "IL-35 (pg/mL)" }
          }
        }
      }
    }
  }
}
```

### 3.4 Microbiome Schema

```json
{
  "$defs": {
    "Microbiome": {
      "type": "object",
      "description": "Gut microbiome analysis with SCFA production focus",
      "properties": {
        "sample_date": { "type": "string", "format": "date-time" },
        "sample_type": { "type": "string", "enum": ["stool", "biopsy", "swab"] },
        "sequencing_method": { "type": "string", "enum": ["16S", "shotgun", "metatranscriptomics"] },
        "diversity_index": {
          "type": "object",
          "properties": {
            "shannon": { "type": "number", "description": "Shannon diversity index" },
            "simpson": { "type": "number", "description": "Simpson diversity index" },
            "chao1": { "type": "number", "description": "Chao1 richness estimator" }
          }
        },
        "scfa_producers": {
          "type": "object",
          "description": "Short-chain fatty acid producing bacteria",
          "properties": {
            "butyrate_producers": {
              "type": "object",
              "properties": {
                "abundance": { "type": "number", "description": "Relative abundance (%)" },
                "genera": {
                  "type": "array",
                  "items": {
                    "type": "object",
                    "properties": {
                      "name": { "type": "string" },
                      "abundance": { "type": "number" }
                    }
                  }
                }
              }
            },
            "propionate_producers": {
              "type": "object",
              "properties": {
                "abundance": { "type": "number" },
                "genera": { "type": "array" }
              }
            },
            "acetate_producers": {
              "type": "object",
              "properties": {
                "abundance": { "type": "number" },
                "genera": { "type": "array" }
              }
            }
          }
        },
        "scfa_levels": {
          "type": "object",
          "description": "Measured SCFA concentrations",
          "properties": {
            "butyrate": { "type": "number", "description": "μmol/g" },
            "propionate": { "type": "number", "description": "μmol/g" },
            "acetate": { "type": "number", "description": "μmol/g" }
          }
        },
        "dysbiosis_score": {
          "type": "number",
          "minimum": 0,
          "maximum": 100,
          "description": "Overall dysbiosis severity (0=healthy, 100=severe dysbiosis)"
        },
        "leaky_gut_markers": {
          "type": "object",
          "properties": {
            "zonulin": { "type": "number", "description": "Zonulin (ng/mL)" },
            "lps": { "type": "number", "description": "Lipopolysaccharide (EU/mL)" },
            "i_fabp": { "type": "number", "description": "Intestinal fatty acid binding protein (pg/mL)" },
            "lactulose_mannitol_ratio": { "type": "number", "description": "L/M ratio" }
          }
        },
        "pathobionts": {
          "type": "array",
          "description": "Potentially pathogenic organisms",
          "items": {
            "type": "object",
            "properties": {
              "name": { "type": "string" },
              "abundance": { "type": "number" },
              "pathogenic_potential": { "type": "string", "enum": ["low", "moderate", "high"] }
            }
          }
        }
      }
    }
  }
}
```

### 3.5 Disease Activity Schema

```json
{
  "$defs": {
    "DiseaseActivity": {
      "type": "object",
      "description": "Disease-specific activity measurements",
      "properties": {
        "disease_type": { "type": "string" },
        "activity_level": { "type": "string", "enum": ["remission", "low", "moderate", "high", "severe"] },
        "scores": {
          "type": "object",
          "properties": {
            "das28": { "type": "number", "description": "Disease Activity Score-28 (RA)" },
            "das28_crp": { "type": "number", "description": "DAS28-CRP variant" },
            "sledai": { "type": "integer", "description": "SLE Disease Activity Index" },
            "sledai_2k": { "type": "integer", "description": "SLEDAI-2K variant" },
            "edss": { "type": "number", "description": "Expanded Disability Status Scale (MS)" },
            "msfc": { "type": "number", "description": "MS Functional Composite" },
            "pasi": { "type": "number", "description": "Psoriasis Area Severity Index" },
            "mayo_score": { "type": "integer", "description": "Mayo Score (UC)" },
            "cdai": { "type": "number", "description": "Crohn's Disease Activity Index" },
            "hba1c": { "type": "number", "description": "HbA1c (T1D)" }
          }
        },
        "flare_status": {
          "type": "object",
          "properties": {
            "is_flaring": { "type": "boolean" },
            "flare_severity": { "type": "string", "enum": ["mild", "moderate", "severe"] },
            "days_since_last_flare": { "type": "integer" },
            "flare_frequency": { "type": "number", "description": "Flares per year" }
          }
        },
        "remission": {
          "type": "object",
          "properties": {
            "in_remission": { "type": "boolean" },
            "remission_type": { "type": "string", "enum": ["clinical", "biochemical", "deep", "drug-free"] },
            "duration_months": { "type": "integer" }
          }
        }
      }
    }
  }
}
```

---

## 4. Complete Profile Example

```json
{
  "@context": ["https://www.w3.org/2018/credentials/v1", "https://wia.live/autoimmune/v1"],
  "type": ["AutoimmuneProfile"],
  "id": "urn:uuid:550e8400-e29b-41d4-a716-446655440000",
  "patient_id": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
  "disease_type": "RA",
  "assessment_date": "2026-01-04T10:30:00Z",

  "immune_markers": {
    "treg": {
      "count": { "value": 45.2, "unit": "cells/μL", "reference_range": { "low": 50, "high": 150 } },
      "foxp3_expression": 68.5,
      "suppressive_function": 42.0,
      "cd25_expression": 72.3,
      "grail_activity": 35.0,
      "stability_score": 55.0
    },
    "th17_treg_ratio": 3.2,
    "autoantibodies": [
      { "name": "Rheumatoid Factor", "code": "RF", "value": 85.0, "unit": "IU/mL", "positive": true },
      { "name": "Anti-CCP", "code": "ACPA", "value": 320.0, "unit": "U/mL", "positive": true, "titer": "high" }
    ],
    "inflammatory_cytokines": {
      "il6": 28.5,
      "il17": 45.2,
      "tnf_alpha": 32.1,
      "ifn_gamma": 18.7
    },
    "anti_inflammatory_cytokines": {
      "il10": 5.2,
      "tgf_beta": 12.3
    }
  },

  "microbiome": {
    "sample_date": "2026-01-03T08:00:00Z",
    "sample_type": "stool",
    "sequencing_method": "shotgun",
    "diversity_index": {
      "shannon": 2.8,
      "simpson": 0.75,
      "chao1": 180
    },
    "scfa_producers": {
      "butyrate_producers": {
        "abundance": 8.5,
        "genera": [
          { "name": "Faecalibacterium", "abundance": 3.2 },
          { "name": "Roseburia", "abundance": 2.1 },
          { "name": "Eubacterium", "abundance": 1.8 }
        ]
      },
      "propionate_producers": { "abundance": 12.3 },
      "acetate_producers": { "abundance": 25.6 }
    },
    "scfa_levels": {
      "butyrate": 45.2,
      "propionate": 32.1,
      "acetate": 85.6
    },
    "dysbiosis_score": 62.0,
    "leaky_gut_markers": {
      "zonulin": 85.3,
      "lps": 0.45,
      "i_fabp": 1250
    },
    "pathobionts": [
      { "name": "Prevotella copri", "abundance": 15.2, "pathogenic_potential": "moderate" }
    ]
  },

  "disease_activity": {
    "disease_type": "RA",
    "activity_level": "moderate",
    "scores": {
      "das28": 4.2,
      "das28_crp": 3.8
    },
    "flare_status": {
      "is_flaring": false,
      "days_since_last_flare": 45,
      "flare_frequency": 4.5
    },
    "remission": {
      "in_remission": false,
      "remission_type": null,
      "duration_months": 0
    }
  }
}
```

---

## 5. Biomarker Codes

### 5.1 Treg Markers

| Code | Name | Unit | Reference Range |
|------|------|------|-----------------|
| WIA-AI-TREG-001 | Treg Count | cells/μL | 50-150 |
| WIA-AI-TREG-002 | FOXP3 Expression | % | >70 |
| WIA-AI-TREG-003 | Suppressive Function | % | >60 |
| WIA-AI-TREG-004 | CD25 Expression | MFI | Variable |
| WIA-AI-TREG-005 | GRAIL Activity | % | >50 |

### 5.2 Microbiome Markers

| Code | Name | Unit | Reference Range |
|------|------|------|-----------------|
| WIA-AI-MB-001 | Shannon Diversity | index | >3.0 |
| WIA-AI-MB-002 | Butyrate Level | μmol/g | >60 |
| WIA-AI-MB-003 | Dysbiosis Score | score | <30 |
| WIA-AI-MB-004 | Zonulin | ng/mL | <50 |
| WIA-AI-MB-005 | LPS | EU/mL | <0.2 |

### 5.3 Inflammatory Markers

| Code | Name | Unit | Reference Range |
|------|------|------|-----------------|
| WIA-AI-INF-001 | IL-6 | pg/mL | <7 |
| WIA-AI-INF-002 | IL-17 | pg/mL | <20 |
| WIA-AI-INF-003 | TNF-α | pg/mL | <15 |
| WIA-AI-INF-004 | IFN-γ | pg/mL | <10 |
| WIA-AI-INF-005 | CRP | mg/L | <3 |

---

## 6. Validation Rules

### 6.1 Required Fields

- `patient_id` must be valid UUID
- `disease_type` must be from approved enum
- `assessment_date` must be valid ISO 8601 datetime

### 6.2 Value Constraints

- All percentage values: 0-100
- All count values: non-negative
- Diversity indices: non-negative
- Disease activity scores: within defined ranges per disease

### 6.3 Cross-Field Validation

- If `remission.in_remission` is true, `activity_level` should be "remission"
- Treg-Microbiome correlation: low butyrate typically correlates with low Treg function

---

© 2026 WIA (World Certification Industry Association)
弘益人間 (홍익인간) · Benefit All Humanity
