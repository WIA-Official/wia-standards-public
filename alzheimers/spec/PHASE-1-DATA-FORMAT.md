# WIA-ALZHEIMERS - Phase 1: Data Format

> **Version:** 1.0.0
> **Last Updated:** 2025-12-29
> **Status:** Complete
> **Standard ID:** WIA-MED-ALZHEIMERS

---

## 1. Overview

Phase 1 defines the standardized data formats for representing Alzheimer's disease assessment data, with a focus on NAD+ homeostasis as the unifying principle for treatment evaluation. These formats enable interoperability between healthcare systems, research institutions, and clinical applications worldwide.

### 1.1 Design Philosophy

The WIA-ALZHEIMERS data format follows the principle of **弘益人間 (Benefit All Humanity)** by:

- Creating universal data structures accessible to all healthcare providers
- Enabling seamless data exchange across different platforms and regions
- Supporting both clinical and research use cases
- Maintaining compatibility with existing healthcare standards (FHIR, HL7)

### 1.2 Key Innovations

Based on 2024-2025 research breakthroughs:

- **NAD+ Homeostasis Index**: A novel composite score integrating multiple metabolic markers
- **Unified Pathology Profile**: Standardized representation of amyloid, tau, and neuroinflammation markers
- **Treatment Response Tracking**: Longitudinal data structures for monitoring intervention efficacy

---

## 2. Data Schema

### 2.1 Root Schema

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://wia.live/schemas/alzheimers/v1.0.0",
  "title": "WIA-ALZHEIMERS Data Schema",
  "description": "Comprehensive data format for Alzheimer's disease assessment and treatment tracking",
  "type": "object",
  "properties": {
    "schema_version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$",
      "description": "Semantic version of the schema"
    },
    "document_type": {
      "type": "string",
      "enum": ["nad_homeostasis_index", "pathology_profile", "treatment_response", "comprehensive_assessment"]
    },
    "subject": {
      "$ref": "#/$defs/SubjectInfo"
    },
    "timestamp": {
      "type": "string",
      "format": "date-time"
    },
    "data": {
      "oneOf": [
        { "$ref": "#/$defs/NADHomeostasisIndex" },
        { "$ref": "#/$defs/PathologyProfile" },
        { "$ref": "#/$defs/TreatmentResponse" }
      ]
    }
  },
  "required": ["schema_version", "document_type", "subject", "timestamp", "data"]
}
```

### 2.2 NAD+ Homeostasis Index Schema

The NAD+ Homeostasis Index is the core innovation of this standard, based on the 2025 Cell Reports Medicine finding that NAD+ homeostasis restoration leads to complete neurological recovery in mouse models.

```json
{
  "$defs": {
    "NADHomeostasisIndex": {
      "type": "object",
      "properties": {
        "version": {
          "type": "string",
          "const": "1.0.0"
        },
        "nad_metabolism": {
          "type": "object",
          "properties": {
            "nad_plus": {
              "$ref": "#/$defs/Measurement",
              "description": "NAD+ concentration"
            },
            "nadh": {
              "$ref": "#/$defs/Measurement",
              "description": "NADH concentration"
            },
            "nadh_nad_ratio": {
              "type": "object",
              "properties": {
                "value": { "type": "number", "minimum": 0, "maximum": 2 },
                "optimal_range": {
                  "type": "array",
                  "items": { "type": "number" },
                  "minItems": 2,
                  "maxItems": 2
                }
              }
            },
            "nadp_plus": { "$ref": "#/$defs/Measurement" },
            "nadph": { "$ref": "#/$defs/Measurement" },
            "nicotinamide": { "$ref": "#/$defs/Measurement" },
            "nmn": { "$ref": "#/$defs/Measurement" },
            "nr": { "$ref": "#/$defs/Measurement" }
          },
          "required": ["nad_plus", "nadh", "nadh_nad_ratio"]
        },
        "nad_synthesizing_enzymes": {
          "type": "object",
          "properties": {
            "nampt": { "$ref": "#/$defs/EnzymeProfile" },
            "nadsyn1": { "$ref": "#/$defs/EnzymeProfile" },
            "nmnat1": { "$ref": "#/$defs/EnzymeProfile" },
            "nmnat2": { "$ref": "#/$defs/EnzymeProfile" },
            "nmnat3": { "$ref": "#/$defs/EnzymeProfile" }
          }
        },
        "nad_consuming_enzymes": {
          "type": "object",
          "properties": {
            "sirt1": { "$ref": "#/$defs/EnzymeProfile" },
            "sirt2": { "$ref": "#/$defs/EnzymeProfile" },
            "sirt3": { "$ref": "#/$defs/EnzymeProfile" },
            "parp1": { "$ref": "#/$defs/EnzymeProfile" },
            "parp4": { "$ref": "#/$defs/EnzymeProfile" },
            "nadk2": { "$ref": "#/$defs/EnzymeProfile" },
            "cd38": { "$ref": "#/$defs/EnzymeProfile" }
          }
        },
        "mitochondrial_function": {
          "type": "object",
          "properties": {
            "membrane_potential_mv": {
              "type": "number",
              "minimum": -200,
              "maximum": 0,
              "description": "Mitochondrial membrane potential in millivolts"
            },
            "atp_production": { "$ref": "#/$defs/Measurement" },
            "ros_level": {
              "type": "object",
              "properties": {
                "value": { "type": "number", "minimum": 0, "maximum": 1 },
                "max": { "type": "number", "const": 1 }
              }
            },
            "oxidative_stress": {
              "type": "object",
              "properties": {
                "value": { "type": "number", "minimum": 0, "maximum": 1 },
                "max": { "type": "number", "const": 1 }
              }
            },
            "upmt_activation": {
              "type": "object",
              "description": "Mitochondrial unfolded protein response markers",
              "properties": {
                "atf4": { "type": "number", "minimum": 0, "maximum": 1 },
                "hsp60": { "type": "number", "minimum": 0, "maximum": 1 },
                "lonp1": { "type": "number", "minimum": 0, "maximum": 1 }
              }
            }
          }
        },
        "composite_score": {
          "type": "object",
          "properties": {
            "homeostasis_index": {
              "type": "number",
              "minimum": 0,
              "maximum": 1,
              "description": "Normalized NAD+ homeostasis score"
            },
            "percentile": {
              "type": "integer",
              "minimum": 0,
              "maximum": 100
            },
            "interpretation": {
              "type": "string",
              "enum": ["excellent", "good", "moderate", "poor", "critical"]
            }
          },
          "required": ["homeostasis_index", "interpretation"]
        }
      },
      "required": ["version", "nad_metabolism", "composite_score"]
    }
  }
}
```

### 2.3 Alzheimer's Pathology Profile Schema

```json
{
  "$defs": {
    "PathologyProfile": {
      "type": "object",
      "properties": {
        "cognitive_assessment": {
          "type": "object",
          "properties": {
            "mmse": {
              "type": "object",
              "properties": {
                "score": { "type": "integer", "minimum": 0, "maximum": 30 },
                "max": { "type": "integer", "const": 30 }
              },
              "description": "Mini-Mental State Examination"
            },
            "moca": {
              "type": "object",
              "properties": {
                "score": { "type": "integer", "minimum": 0, "maximum": 30 },
                "max": { "type": "integer", "const": 30 }
              },
              "description": "Montreal Cognitive Assessment"
            },
            "cdr_sb": {
              "type": "object",
              "properties": {
                "score": { "type": "number", "minimum": 0, "maximum": 18 },
                "baseline": { "type": "number" }
              },
              "description": "Clinical Dementia Rating - Sum of Boxes"
            },
            "adas_cog": {
              "type": "object",
              "properties": {
                "score": { "type": "integer" }
              },
              "description": "Alzheimer's Disease Assessment Scale - Cognitive"
            },
            "stage": {
              "type": "string",
              "enum": ["preclinical", "mci", "mild", "moderate", "severe"]
            }
          },
          "required": ["stage"]
        },
        "amyloid_markers": {
          "type": "object",
          "properties": {
            "plasma_abeta42": { "$ref": "#/$defs/BiomarkerMeasurement" },
            "plasma_abeta40": { "$ref": "#/$defs/BiomarkerMeasurement" },
            "abeta42_40_ratio": {
              "type": "object",
              "properties": {
                "value": { "type": "number" },
                "cutoff": { "type": "number", "default": 0.067 },
                "positive": { "type": "boolean" }
              }
            },
            "csf_abeta42": { "$ref": "#/$defs/BiomarkerMeasurement" },
            "pet_amyloid": {
              "type": "object",
              "properties": {
                "centiloid": { "type": "integer" },
                "positive": { "type": "boolean" }
              }
            }
          }
        },
        "tau_markers": {
          "type": "object",
          "properties": {
            "plasma_ptau181": { "$ref": "#/$defs/BiomarkerMeasurement" },
            "plasma_ptau217": { "$ref": "#/$defs/BiomarkerMeasurement" },
            "csf_total_tau": { "$ref": "#/$defs/BiomarkerMeasurement" },
            "csf_ptau181": { "$ref": "#/$defs/BiomarkerMeasurement" },
            "pet_tau": {
              "type": "object",
              "properties": {
                "suvr": { "type": "number" },
                "positive": { "type": "boolean" }
              }
            }
          }
        },
        "neurodegeneration_markers": {
          "type": "object",
          "properties": {
            "plasma_nfl": { "$ref": "#/$defs/BiomarkerMeasurement" },
            "plasma_gfap": { "$ref": "#/$defs/BiomarkerMeasurement" },
            "csf_nfl": { "$ref": "#/$defs/BiomarkerMeasurement" },
            "hippocampal_volume": {
              "type": "object",
              "properties": {
                "value": { "type": "number" },
                "unit": { "type": "string", "default": "cm3" },
                "atrophy_percent": { "type": "number" }
              }
            },
            "cortical_thickness": { "$ref": "#/$defs/Measurement" }
          }
        },
        "neuroinflammation": {
          "type": "object",
          "properties": {
            "il6": { "$ref": "#/$defs/BiomarkerMeasurement" },
            "tnf_alpha": { "$ref": "#/$defs/BiomarkerMeasurement" },
            "ykl40": { "$ref": "#/$defs/BiomarkerMeasurement" },
            "trem2": { "$ref": "#/$defs/BiomarkerMeasurement" },
            "reactive_astrocytes": { "type": "number", "minimum": 0, "maximum": 1 },
            "activated_microglia": { "type": "number", "minimum": 0, "maximum": 1 }
          }
        },
        "blood_brain_barrier": {
          "type": "object",
          "properties": {
            "integrity_score": { "type": "number", "minimum": 0, "maximum": 1 },
            "albumin_ratio": { "type": "number" },
            "tight_junction_proteins": {
              "type": "object",
              "properties": {
                "claudin5": { "type": "number" },
                "occludin": { "type": "number" }
              }
            }
          }
        },
        "synaptic_function": {
          "type": "object",
          "properties": {
            "neurogranin": { "$ref": "#/$defs/BiomarkerMeasurement" },
            "snap25": { "$ref": "#/$defs/BiomarkerMeasurement" },
            "synaptic_density": { "type": "number", "minimum": 0, "maximum": 1 },
            "ltp_capacity": { "type": "number", "minimum": 0, "maximum": 1 }
          }
        }
      },
      "required": ["cognitive_assessment"]
    }
  }
}
```

---

## 3. Field Definitions

### 3.1 Core Definitions

| Field | Type | Description | Required |
|-------|------|-------------|----------|
| `schema_version` | string | Semantic version (e.g., "1.0.0") | Yes |
| `document_type` | enum | Type of data document | Yes |
| `subject.id` | string (UUID) | Unique subject identifier | Yes |
| `subject.age` | integer | Subject age in years | Yes |
| `subject.sex` | enum | Biological sex | Yes |
| `timestamp` | ISO8601 | Document creation time | Yes |

### 3.2 Measurement Definition

```json
{
  "$defs": {
    "Measurement": {
      "type": "object",
      "properties": {
        "value": { "type": "number" },
        "unit": { "type": "string" },
        "tissue": { "type": "string", "enum": ["blood", "csf", "brain", "plasma"] },
        "method": { "type": "string" },
        "timestamp": { "type": "string", "format": "date-time" }
      },
      "required": ["value", "unit"]
    }
  }
}
```

### 3.3 Enzyme Profile Definition

```json
{
  "$defs": {
    "EnzymeProfile": {
      "type": "object",
      "properties": {
        "activity": {
          "type": "number",
          "minimum": 0,
          "maximum": 1,
          "description": "Normalized enzyme activity (0-1)"
        },
        "expression": {
          "oneOf": [
            { "type": "number", "minimum": 0 },
            { "type": "string", "enum": ["low", "normal", "elevated", "high"] }
          ],
          "description": "Gene expression level"
        },
        "protein_level": {
          "type": "number",
          "description": "Protein concentration"
        }
      }
    }
  }
}
```

### 3.4 Biomarker Measurement Definition

```json
{
  "$defs": {
    "BiomarkerMeasurement": {
      "type": "object",
      "properties": {
        "value": { "type": "number" },
        "unit": { "type": "string", "default": "pg/mL" },
        "reference_range": {
          "type": "object",
          "properties": {
            "low": { "type": "number" },
            "high": { "type": "number" }
          }
        },
        "interpretation": {
          "type": "string",
          "enum": ["normal", "borderline", "abnormal", "critical"]
        }
      },
      "required": ["value", "unit"]
    }
  }
}
```

---

## 4. Validation Rules

### 4.1 NAD+ Homeostasis Index Validation

```typescript
interface NADHomeostasisValidation {
  // NAD+ must be positive
  nad_plus_positive: (value: number) => value > 0;

  // NADH/NAD+ ratio should be between 0.05 and 0.5
  ratio_range: (ratio: number) => ratio >= 0.05 && ratio <= 0.5;

  // Optimal NADH/NAD+ ratio is 0.1-0.3
  ratio_optimal: (ratio: number) => ratio >= 0.1 && ratio <= 0.3;

  // Homeostasis index must be 0-1
  index_range: (index: number) => index >= 0 && index <= 1;

  // Sirtuin and PARP activities are inversely related
  enzyme_balance: (sirtuin: number, parp: number) =>
    (sirtuin + parp) <= 1.5;
}
```

### 4.2 Cognitive Assessment Validation

| Assessment | Valid Range | Impairment Threshold |
|------------|-------------|---------------------|
| MMSE | 0-30 | <24 (mild), <18 (moderate), <10 (severe) |
| MoCA | 0-30 | <26 (mild), <22 (moderate), <17 (severe) |
| CDR-SB | 0-18 | >0.5 (MCI), >4.5 (mild), >9.5 (moderate) |
| ADAS-Cog | 0-70 | >12 (impaired) |

### 4.3 Biomarker Validation

```typescript
const biomarkerCutoffs = {
  abeta42_40_ratio: {
    positive: (v: number) => v < 0.067,
    borderline: (v: number) => v >= 0.067 && v < 0.08
  },
  plasma_ptau181: {
    elevated: (v: number) => v > 20,  // pg/mL
    high: (v: number) => v > 35
  },
  plasma_ptau217: {
    elevated: (v: number) => v > 0.4,  // pg/mL
    high: (v: number) => v > 0.8
  },
  plasma_nfl: {
    elevated: (v: number) => v > 30,  // pg/mL (age-adjusted)
  }
};
```

---

## 5. Examples

### 5.1 Complete NAD+ Homeostasis Index

```json
{
  "schema_version": "1.0.0",
  "document_type": "nad_homeostasis_index",
  "subject": {
    "id": "550e8400-e29b-41d4-a716-446655440000",
    "age": 68,
    "sex": "female",
    "apoe_genotype": "e3/e4"
  },
  "timestamp": "2025-12-29T10:30:00Z",
  "data": {
    "version": "1.0.0",
    "nad_metabolism": {
      "nad_plus": { "value": 22.5, "unit": "μM", "tissue": "blood" },
      "nadh": { "value": 4.5, "unit": "μM" },
      "nadh_nad_ratio": { "value": 0.2, "optimal_range": [0.1, 0.3] },
      "nicotinamide": { "value": 8.2, "unit": "μM" },
      "nmn": { "value": 0.8, "unit": "μM" }
    },
    "nad_synthesizing_enzymes": {
      "nampt": { "activity": 0.7, "expression": "normal" },
      "nmnat2": { "activity": 0.6, "expression": "low" }
    },
    "nad_consuming_enzymes": {
      "sirt1": { "activity": 0.75, "expression": "normal" },
      "parp1": { "activity": 0.4, "expression": "elevated" },
      "cd38": { "activity": 0.55, "expression": "elevated" }
    },
    "mitochondrial_function": {
      "membrane_potential_mv": -140,
      "atp_production": { "value": 85, "unit": "μmol/min/mg" },
      "ros_level": { "value": 0.35, "max": 1.0 },
      "oxidative_stress": { "value": 0.4, "max": 1.0 },
      "upmt_activation": { "atf4": 0.3, "hsp60": 0.4, "lonp1": 0.35 }
    },
    "composite_score": {
      "homeostasis_index": 0.68,
      "percentile": 45,
      "interpretation": "moderate"
    }
  }
}
```

### 5.2 Pathology Profile with Biomarkers

```json
{
  "schema_version": "1.0.0",
  "document_type": "pathology_profile",
  "subject": {
    "id": "550e8400-e29b-41d4-a716-446655440000",
    "age": 68,
    "sex": "female"
  },
  "timestamp": "2025-12-29T10:30:00Z",
  "data": {
    "cognitive_assessment": {
      "mmse": { "score": 24, "max": 30 },
      "moca": { "score": 22, "max": 30 },
      "cdr_sb": { "score": 2.5, "baseline": 0 },
      "stage": "mci"
    },
    "amyloid_markers": {
      "plasma_abeta42": { "value": 42.5, "unit": "pg/mL" },
      "plasma_abeta40": { "value": 685, "unit": "pg/mL" },
      "abeta42_40_ratio": { "value": 0.062, "cutoff": 0.067, "positive": true },
      "pet_amyloid": { "centiloid": 45, "positive": true }
    },
    "tau_markers": {
      "plasma_ptau181": { "value": 28.5, "unit": "pg/mL" },
      "plasma_ptau217": { "value": 0.52, "unit": "pg/mL" }
    },
    "neurodegeneration_markers": {
      "plasma_nfl": { "value": 22.5, "unit": "pg/mL" },
      "plasma_gfap": { "value": 145, "unit": "pg/mL" },
      "hippocampal_volume": { "value": 3.2, "unit": "cm3", "atrophy_percent": 12 }
    },
    "neuroinflammation": {
      "reactive_astrocytes": 0.35,
      "activated_microglia": 0.4
    },
    "blood_brain_barrier": {
      "integrity_score": 0.78
    },
    "synaptic_function": {
      "synaptic_density": 0.72,
      "ltp_capacity": 0.65
    }
  }
}
```

### 5.3 Treatment Response Tracking

```json
{
  "schema_version": "1.0.0",
  "document_type": "treatment_response",
  "subject": {
    "id": "550e8400-e29b-41d4-a716-446655440000",
    "age": 68,
    "sex": "female"
  },
  "timestamp": "2025-12-29T10:30:00Z",
  "data": {
    "intervention": {
      "type": "nad_precursor",
      "agent": "NR",
      "dosage": { "value": 500, "unit": "mg", "frequency": "twice daily" },
      "duration_days": 56,
      "started_at": "2025-11-01T08:00:00Z"
    },
    "baseline": {
      "nad_homeostasis_index": 0.52,
      "cognitive_score": 22,
      "biomarkers": {
        "plasma_nad": 18.5,
        "plasma_ptau181": 32.5
      }
    },
    "follow_up": [
      {
        "timepoint_days": 28,
        "nad_homeostasis_index": 0.62,
        "cognitive_change": 1.5,
        "biomarker_change": { "plasma_nad": 24.2 },
        "adverse_events": []
      },
      {
        "timepoint_days": 56,
        "nad_homeostasis_index": 0.72,
        "cognitive_change": 2.8,
        "biomarker_change": { "plasma_nad": 28.5 },
        "adverse_events": []
      }
    ],
    "outcomes": {
      "responder_status": "full",
      "cognitive_improvement_percent": 12.7,
      "nad_recovery_percent": 54.1,
      "time_to_response_days": 21
    }
  }
}
```

---

## 6. References

1. Cell Reports Medicine (2025). "Complete neurological recovery in AD mouse model through NAD+ homeostasis restoration with P7C3-A20"
2. Nature Cell Death & Disease (2024). "NMN improves mitochondrial UPR through ATF4-dependent pathways"
3. Alzheimer's & Dementia: Translational Research (2025). "Nicotinamide riboside clinical trial in MCI"
4. ScienceDirect Neurobiology of Disease (2024). "Plasma nicotinamide as neuroinflammation biomarker"
5. eNeuro (2024). "Lecanemab and Donanemab: Clinical efficacy comparison"

---

**弘益人間 (홍익인간) - Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
