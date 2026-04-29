# WIA-CHRONIC-PAIN Phase 1: Data Format Specification

> **Version:** 1.0.0
> **Status:** Official
> **Last Updated:** 2026-01-04
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

The WIA-CHRONIC-PAIN Data Format specification defines standardized data structures for chronic pain assessment and management, centered on the **Neuroplasticity Reversal** paradigm - the unified principle that chronic pain represents maladaptive neural plasticity that can be reversed.

### 1.1 Core Discovery

```
┌─────────────────────────────────────────────────────────────────────────────┐
│  💡 Unified Principle: Chronic Pain = Maladaptive Neuroplasticity          │
│                        → REVERSIBLE through targeted intervention           │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  🧠 Central Sensitization (2024-2025):                                     │
│  • Chronic pain = CNS neuroplasticity disorder                             │
│  • Brain structural changes: ACC, Insula, PFC gray matter reduction        │
│  • REVERSIBLE: CBT + Exercise = Gray matter recovery demonstrated          │
│                                                                             │
│  ⚡ Non-Invasive Neuromodulation (2025):                                    │
│  • TMS, tDCS, Focused Ultrasound: Target pain processing regions           │
│  • Surgery-free neural activity modulation                                 │
│  • Opioid-sparing alternative                                              │
│                                                                             │
│  🎯 Personalized Approach:                                                 │
│  • Neuroplasticity state assessment (LTP/Sensitization)                    │
│  • Tailored treatment selection                                            │
│  • Pain phenotype-specific protocols                                       │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 1.2 Pain Classification

| Code | Pain Type | Mechanism | Examples |
|------|-----------|-----------|----------|
| NOC | Nociceptive | Tissue damage signal | Arthritis, injury |
| NEU | Neuropathic | Nerve damage/dysfunction | Diabetic neuropathy, PHN |
| NOP | Nociplastic | Central sensitization | Fibromyalgia, chronic LBP |
| MIX | Mixed | Multiple mechanisms | Complex regional pain |

---

## 2. Data Model Architecture

### 2.1 Entity Relationship

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│     Patient     │───▶│ ChronicPainProfile│───▶│  PainAssessment  │
└─────────────────┘    └──────────────────┘    └─────────────────┘
        │                      │                        │
        │                      │                        │
        ▼                      ▼                        ▼
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│  Sensitization  │◀──▶│   Neuroimaging   │    │    Treatment    │
└─────────────────┘    └──────────────────┘    └─────────────────┘
         ↑                     ↑
         └─────────────────────┘
          Neuroplasticity Axis
```

### 2.2 Core Entities

| Entity | Description | Primary Key |
|--------|-------------|-------------|
| Patient | Individual with chronic pain | patient_id |
| ChronicPainProfile | Complete pain assessment | profile_id |
| PainAssessment | Multidimensional pain evaluation | assessment_id |
| Sensitization | Central sensitization status | sensitization_id |
| Neuroimaging | Brain structure/function data | imaging_id |
| Treatment | Intervention records | treatment_id |

---

## 3. JSON Schema Definitions

### 3.1 Root Schema: ChronicPainProfile

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://wia.live/schemas/chronic-pain/v1.0.0",
  "title": "WIA-CHRONIC-PAIN Profile",
  "description": "Complete chronic pain profile with neuroplasticity focus",
  "type": "object",
  "required": ["@context", "type", "id", "patient_id", "pain_type", "assessment_date"],
  "properties": {
    "@context": {
      "type": "array",
      "items": { "type": "string" },
      "default": ["https://www.w3.org/2018/credentials/v1", "https://wia.live/chronic-pain/v1"]
    },
    "type": {
      "type": "array",
      "items": { "type": "string" },
      "contains": { "const": "ChronicPainProfile" }
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
    "pain_type": {
      "type": "string",
      "enum": ["nociceptive", "neuropathic", "nociplastic", "mixed"],
      "description": "Primary pain mechanism classification"
    },
    "assessment_date": {
      "type": "string",
      "format": "date-time"
    },
    "pain_assessment": { "$ref": "#/$defs/PainAssessment" },
    "central_sensitization": { "$ref": "#/$defs/CentralSensitization" },
    "neuroimaging": { "$ref": "#/$defs/Neuroimaging" },
    "psychosocial": { "$ref": "#/$defs/Psychosocial" },
    "treatment": { "$ref": "#/$defs/Treatment" }
  }
}
```

### 3.2 Pain Assessment Schema

```json
{
  "$defs": {
    "PainAssessment": {
      "type": "object",
      "description": "Multidimensional pain assessment",
      "properties": {
        "vas": {
          "type": "object",
          "description": "Visual Analog Scale",
          "properties": {
            "current": { "type": "number", "minimum": 0, "maximum": 100 },
            "average_7days": { "type": "number", "minimum": 0, "maximum": 100 },
            "worst_7days": { "type": "number", "minimum": 0, "maximum": 100 },
            "best_7days": { "type": "number", "minimum": 0, "maximum": 100 }
          }
        },
        "nrs": {
          "type": "object",
          "description": "Numeric Rating Scale (0-10)",
          "properties": {
            "current": { "type": "integer", "minimum": 0, "maximum": 10 },
            "average_7days": { "type": "number", "minimum": 0, "maximum": 10 },
            "worst_7days": { "type": "integer", "minimum": 0, "maximum": 10 }
          }
        },
        "mcgill": {
          "type": "object",
          "description": "McGill Pain Questionnaire",
          "properties": {
            "sensory": { "type": "integer", "minimum": 0, "maximum": 42 },
            "affective": { "type": "integer", "minimum": 0, "maximum": 14 },
            "evaluative": { "type": "integer", "minimum": 0, "maximum": 5 },
            "miscellaneous": { "type": "integer", "minimum": 0, "maximum": 17 },
            "total_ppi": { "type": "integer", "minimum": 0, "maximum": 78 },
            "descriptors": {
              "type": "array",
              "items": { "type": "string" }
            }
          }
        },
        "brief_pain_inventory": {
          "type": "object",
          "description": "BPI - Pain interference",
          "properties": {
            "pain_severity": { "type": "number", "minimum": 0, "maximum": 10 },
            "pain_interference": { "type": "number", "minimum": 0, "maximum": 10 },
            "general_activity": { "type": "integer", "minimum": 0, "maximum": 10 },
            "mood": { "type": "integer", "minimum": 0, "maximum": 10 },
            "walking": { "type": "integer", "minimum": 0, "maximum": 10 },
            "work": { "type": "integer", "minimum": 0, "maximum": 10 },
            "relations": { "type": "integer", "minimum": 0, "maximum": 10 },
            "sleep": { "type": "integer", "minimum": 0, "maximum": 10 },
            "enjoyment": { "type": "integer", "minimum": 0, "maximum": 10 }
          }
        },
        "duration_months": {
          "type": "number",
          "minimum": 0,
          "description": "Duration of chronic pain in months"
        },
        "onset_date": {
          "type": "string",
          "format": "date"
        },
        "locations": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "body_region": { "type": "string" },
              "icd_code": { "type": "string" },
              "laterality": { "type": "string", "enum": ["left", "right", "bilateral", "midline"] },
              "referred": { "type": "boolean" },
              "intensity": { "type": "number", "minimum": 0, "maximum": 10 }
            }
          }
        },
        "pain_quality": {
          "type": "array",
          "items": {
            "type": "string",
            "enum": ["burning", "shooting", "stabbing", "aching", "throbbing", "tingling", "numbness", "electric", "pressure", "cramping"]
          }
        },
        "aggravating_factors": {
          "type": "array",
          "items": { "type": "string" }
        },
        "relieving_factors": {
          "type": "array",
          "items": { "type": "string" }
        },
        "diurnal_pattern": {
          "type": "string",
          "enum": ["constant", "morning_worse", "evening_worse", "night_worse", "variable"]
        }
      }
    }
  }
}
```

### 3.3 Central Sensitization Schema

```json
{
  "$defs": {
    "CentralSensitization": {
      "type": "object",
      "description": "Central sensitization and neuroplasticity assessment",
      "properties": {
        "csi_score": {
          "type": "object",
          "description": "Central Sensitization Inventory",
          "properties": {
            "part_a": { "type": "integer", "minimum": 0, "maximum": 100 },
            "part_b": { "type": "integer", "description": "Number of diagnosed conditions" },
            "interpretation": {
              "type": "string",
              "enum": ["subclinical", "mild", "moderate", "severe", "extreme"]
            }
          }
        },
        "qst_profile": {
          "type": "object",
          "description": "Quantitative Sensory Testing",
          "properties": {
            "thermal_detection": {
              "type": "object",
              "properties": {
                "cold_detection": { "type": "number", "description": "°C" },
                "warm_detection": { "type": "number", "description": "°C" },
                "cold_pain": { "type": "number", "description": "°C" },
                "heat_pain": { "type": "number", "description": "°C" }
              }
            },
            "mechanical_detection": {
              "type": "object",
              "properties": {
                "mdt": { "type": "number", "description": "Mechanical detection threshold (mN)" },
                "mpt": { "type": "number", "description": "Mechanical pain threshold (mN)" },
                "mps": { "type": "number", "description": "Mechanical pain sensitivity" },
                "dma": { "type": "number", "description": "Dynamic mechanical allodynia" }
              }
            },
            "pressure_pain": {
              "type": "object",
              "properties": {
                "ppt": { "type": "number", "description": "Pressure pain threshold (kPa)" },
                "ppt_remote": { "type": "number", "description": "Remote PPT for widespread sensitization" }
              }
            },
            "vibration": {
              "type": "number",
              "description": "Vibration detection threshold"
            },
            "z_scores": {
              "type": "object",
              "description": "Z-scores compared to normative data"
            }
          }
        },
        "temporal_summation": {
          "type": "object",
          "description": "Wind-up / Temporal summation of pain",
          "properties": {
            "present": { "type": "boolean" },
            "ratio": { "type": "number", "description": "Pain rating 10th/1st stimulus" },
            "interpretation": { "type": "string", "enum": ["normal", "enhanced", "severely_enhanced"] }
          }
        },
        "conditioned_pain_modulation": {
          "type": "object",
          "description": "CPM - Endogenous pain inhibition",
          "properties": {
            "cpm_effect": { "type": "number", "description": "Percentage change in test pain" },
            "status": { "type": "string", "enum": ["intact", "reduced", "absent", "paradoxical"] },
            "conditioning_stimulus": { "type": "string" },
            "test_stimulus": { "type": "string" }
          }
        },
        "allodynia": {
          "type": "object",
          "properties": {
            "mechanical": { "type": "boolean" },
            "thermal_cold": { "type": "boolean" },
            "thermal_heat": { "type": "boolean" }
          }
        },
        "hyperalgesia": {
          "type": "object",
          "properties": {
            "primary": { "type": "boolean", "description": "At injury site" },
            "secondary": { "type": "boolean", "description": "Beyond injury site" },
            "widespread": { "type": "boolean", "description": "Generalized" }
          }
        },
        "neuroplasticity_index": {
          "type": "number",
          "minimum": 0,
          "maximum": 100,
          "description": "Composite maladaptive plasticity score"
        }
      }
    }
  }
}
```

### 3.4 Neuroimaging Schema

```json
{
  "$defs": {
    "Neuroimaging": {
      "type": "object",
      "description": "Brain structure and function in chronic pain",
      "properties": {
        "scan_date": { "type": "string", "format": "date-time" },
        "modality": { "type": "string", "enum": ["MRI", "fMRI", "PET", "EEG", "MEG"] },
        "gray_matter_changes": {
          "type": "object",
          "description": "Structural changes in pain-related regions",
          "properties": {
            "acc": {
              "type": "object",
              "description": "Anterior Cingulate Cortex",
              "properties": {
                "volume_change": { "type": "number", "description": "% change from normative" },
                "status": { "type": "string", "enum": ["normal", "reduced", "increased"] }
              }
            },
            "insula": {
              "type": "object",
              "properties": {
                "volume_change": { "type": "number" },
                "status": { "type": "string" }
              }
            },
            "pfc": {
              "type": "object",
              "description": "Prefrontal Cortex",
              "properties": {
                "dlpfc_volume": { "type": "number" },
                "mpfc_volume": { "type": "number" },
                "status": { "type": "string" }
              }
            },
            "thalamus": {
              "type": "object",
              "properties": {
                "volume_change": { "type": "number" },
                "status": { "type": "string" }
              }
            },
            "pag": {
              "type": "object",
              "description": "Periaqueductal Gray",
              "properties": {
                "volume_change": { "type": "number" },
                "status": { "type": "string" }
              }
            },
            "s1_s2": {
              "type": "object",
              "description": "Primary/Secondary Somatosensory Cortex",
              "properties": {
                "reorganization": { "type": "boolean" },
                "expansion": { "type": "number" }
              }
            }
          }
        },
        "functional_connectivity": {
          "type": "object",
          "properties": {
            "dmn_salience": {
              "type": "number",
              "description": "Default Mode - Salience network connectivity"
            },
            "pain_matrix_connectivity": {
              "type": "number",
              "description": "Pain matrix network coherence"
            },
            "acc_pag_connectivity": {
              "type": "number",
              "description": "Descending modulation pathway"
            }
          }
        },
        "default_mode_network": {
          "type": "object",
          "properties": {
            "activity_at_rest": { "type": "number" },
            "deactivation_during_pain": { "type": "number" },
            "status": { "type": "string", "enum": ["normal", "disrupted"] }
          }
        },
        "spectral_analysis": {
          "type": "object",
          "description": "EEG/MEG frequency analysis",
          "properties": {
            "theta_power": { "type": "number" },
            "alpha_power": { "type": "number" },
            "beta_power": { "type": "number" },
            "gamma_power": { "type": "number" },
            "thalamo_cortical_dysrhythmia": { "type": "boolean" }
          }
        }
      }
    }
  }
}
```

### 3.5 Psychosocial Schema

```json
{
  "$defs": {
    "Psychosocial": {
      "type": "object",
      "description": "Psychological and social factors in chronic pain",
      "properties": {
        "pain_catastrophizing": {
          "type": "object",
          "description": "Pain Catastrophizing Scale (PCS)",
          "properties": {
            "total": { "type": "integer", "minimum": 0, "maximum": 52 },
            "rumination": { "type": "integer", "minimum": 0, "maximum": 16 },
            "magnification": { "type": "integer", "minimum": 0, "maximum": 12 },
            "helplessness": { "type": "integer", "minimum": 0, "maximum": 24 },
            "level": { "type": "string", "enum": ["low", "moderate", "high", "clinical"] }
          }
        },
        "kinesiophobia": {
          "type": "object",
          "description": "Tampa Scale of Kinesiophobia (TSK)",
          "properties": {
            "total": { "type": "integer", "minimum": 17, "maximum": 68 },
            "activity_avoidance": { "type": "integer" },
            "harm_beliefs": { "type": "integer" },
            "level": { "type": "string", "enum": ["low", "moderate", "high"] }
          }
        },
        "fear_avoidance": {
          "type": "object",
          "description": "Fear-Avoidance Beliefs Questionnaire (FABQ)",
          "properties": {
            "fabq_physical": { "type": "integer", "minimum": 0, "maximum": 24 },
            "fabq_work": { "type": "integer", "minimum": 0, "maximum": 42 }
          }
        },
        "depression": {
          "type": "object",
          "properties": {
            "phq9_score": { "type": "integer", "minimum": 0, "maximum": 27 },
            "severity": { "type": "string", "enum": ["minimal", "mild", "moderate", "moderately_severe", "severe"] }
          }
        },
        "anxiety": {
          "type": "object",
          "properties": {
            "gad7_score": { "type": "integer", "minimum": 0, "maximum": 21 },
            "severity": { "type": "string", "enum": ["minimal", "mild", "moderate", "severe"] }
          }
        },
        "sleep": {
          "type": "object",
          "properties": {
            "psqi_score": { "type": "integer", "minimum": 0, "maximum": 21 },
            "insomnia_severity": { "type": "integer", "minimum": 0, "maximum": 28 },
            "pain_disrupted_sleep": { "type": "boolean" }
          }
        },
        "quality_of_life": {
          "type": "object",
          "properties": {
            "sf36_physical": { "type": "number", "minimum": 0, "maximum": 100 },
            "sf36_mental": { "type": "number", "minimum": 0, "maximum": 100 },
            "eq5d_index": { "type": "number", "minimum": -0.5, "maximum": 1.0 }
          }
        },
        "self_efficacy": {
          "type": "object",
          "properties": {
            "pain_self_efficacy": { "type": "integer", "minimum": 0, "maximum": 60 },
            "level": { "type": "string", "enum": ["low", "moderate", "high"] }
          }
        },
        "social_support": {
          "type": "object",
          "properties": {
            "perceived_support": { "type": "number", "minimum": 0, "maximum": 100 },
            "functional_status": { "type": "string", "enum": ["working", "disabled", "retired", "limited_work"] }
          }
        }
      }
    }
  }
}
```

### 3.6 Treatment Schema

```json
{
  "$defs": {
    "Treatment": {
      "type": "object",
      "description": "Treatment history and current interventions",
      "properties": {
        "current_medications": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "name": { "type": "string" },
              "class": {
                "type": "string",
                "enum": ["opioid", "nsaid", "antidepressant", "anticonvulsant", "muscle_relaxant", "topical", "other"]
              },
              "dose": { "type": "string" },
              "frequency": { "type": "string" },
              "effectiveness": { "type": "integer", "minimum": 0, "maximum": 10 },
              "side_effects": { "type": "array", "items": { "type": "string" } }
            }
          }
        },
        "opioid_use": {
          "type": "object",
          "properties": {
            "current_use": { "type": "boolean" },
            "mme_daily": { "type": "number", "description": "Morphine Milligram Equivalents per day" },
            "duration_months": { "type": "number" },
            "opioid_risk_tool": { "type": "integer", "minimum": 0, "maximum": 26 },
            "tapering_status": { "type": "string", "enum": ["not_applicable", "stable", "tapering", "completed"] }
          }
        },
        "interventional": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "procedure": { "type": "string" },
              "date": { "type": "string", "format": "date" },
              "location": { "type": "string" },
              "response": { "type": "string", "enum": ["none", "partial", "good", "excellent"] },
              "duration_relief_weeks": { "type": "number" }
            }
          }
        },
        "neuromodulation": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "type": {
                "type": "string",
                "enum": ["tms", "tdcs", "tens", "scs", "dbs", "focused_ultrasound", "prp"]
              },
              "target_region": { "type": "string" },
              "parameters": { "type": "object" },
              "sessions_completed": { "type": "integer" },
              "response": { "type": "string" }
            }
          }
        },
        "behavioral": {
          "type": "object",
          "properties": {
            "cbt": {
              "type": "object",
              "properties": {
                "sessions_completed": { "type": "integer" },
                "current_status": { "type": "string", "enum": ["not_started", "in_progress", "completed", "maintenance"] }
              }
            },
            "act": {
              "type": "object",
              "description": "Acceptance and Commitment Therapy",
              "properties": {
                "sessions_completed": { "type": "integer" },
                "current_status": { "type": "string" }
              }
            },
            "mindfulness": {
              "type": "object",
              "properties": {
                "type": { "type": "string", "enum": ["mbsr", "mbct", "general_meditation"] },
                "practice_minutes_week": { "type": "integer" }
              }
            },
            "pain_neuroscience_education": {
              "type": "object",
              "properties": {
                "completed": { "type": "boolean" },
                "understanding_score": { "type": "integer", "minimum": 0, "maximum": 100 }
              }
            }
          }
        },
        "physical": {
          "type": "object",
          "properties": {
            "physical_therapy": {
              "type": "object",
              "properties": {
                "sessions_completed": { "type": "integer" },
                "frequency_per_week": { "type": "number" },
                "focus": { "type": "array", "items": { "type": "string" } }
              }
            },
            "exercise_program": {
              "type": "object",
              "properties": {
                "type": { "type": "array", "items": { "type": "string" } },
                "frequency_per_week": { "type": "number" },
                "duration_minutes": { "type": "integer" },
                "adherence_percentage": { "type": "number" }
              }
            },
            "graded_exposure": {
              "type": "object",
              "properties": {
                "in_progress": { "type": "boolean" },
                "current_level": { "type": "integer" }
              }
            }
          }
        },
        "response_history": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "treatment": { "type": "string" },
              "start_date": { "type": "string", "format": "date" },
              "end_date": { "type": "string", "format": "date" },
              "pain_change_percent": { "type": "number" },
              "function_change_percent": { "type": "number" },
              "reason_discontinued": { "type": "string" }
            }
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
  "@context": ["https://www.w3.org/2018/credentials/v1", "https://wia.live/chronic-pain/v1"],
  "type": ["ChronicPainProfile"],
  "id": "urn:uuid:660e8400-e29b-41d4-a716-446655440001",
  "patient_id": "b2c3d4e5-f6a7-8901-bcde-f12345678901",
  "pain_type": "nociplastic",
  "assessment_date": "2026-01-04T14:00:00Z",

  "pain_assessment": {
    "nrs": {
      "current": 7,
      "average_7days": 6.5,
      "worst_7days": 9
    },
    "brief_pain_inventory": {
      "pain_severity": 6.5,
      "pain_interference": 7.2,
      "general_activity": 7,
      "mood": 6,
      "sleep": 8
    },
    "duration_months": 36,
    "locations": [
      { "body_region": "lower_back", "laterality": "bilateral", "intensity": 7 },
      { "body_region": "neck", "laterality": "bilateral", "intensity": 5 },
      { "body_region": "shoulders", "laterality": "bilateral", "intensity": 4 }
    ],
    "pain_quality": ["aching", "burning", "pressure"],
    "diurnal_pattern": "morning_worse"
  },

  "central_sensitization": {
    "csi_score": {
      "part_a": 62,
      "part_b": 3,
      "interpretation": "severe"
    },
    "temporal_summation": {
      "present": true,
      "ratio": 2.8,
      "interpretation": "enhanced"
    },
    "conditioned_pain_modulation": {
      "cpm_effect": -5,
      "status": "reduced"
    },
    "hyperalgesia": {
      "primary": true,
      "secondary": true,
      "widespread": true
    },
    "neuroplasticity_index": 72
  },

  "neuroimaging": {
    "scan_date": "2026-01-02T10:00:00Z",
    "modality": "MRI",
    "gray_matter_changes": {
      "acc": { "volume_change": -8.5, "status": "reduced" },
      "insula": { "volume_change": -6.2, "status": "reduced" },
      "pfc": { "dlpfc_volume": -7.1, "status": "reduced" },
      "thalamus": { "volume_change": -4.3, "status": "reduced" }
    },
    "functional_connectivity": {
      "dmn_salience": 0.72,
      "pain_matrix_connectivity": 0.85
    },
    "default_mode_network": {
      "activity_at_rest": 0.65,
      "status": "disrupted"
    }
  },

  "psychosocial": {
    "pain_catastrophizing": {
      "total": 38,
      "rumination": 14,
      "magnification": 8,
      "helplessness": 16,
      "level": "clinical"
    },
    "kinesiophobia": {
      "total": 48,
      "level": "high"
    },
    "depression": {
      "phq9_score": 14,
      "severity": "moderate"
    },
    "anxiety": {
      "gad7_score": 12,
      "severity": "moderate"
    },
    "quality_of_life": {
      "sf36_physical": 32,
      "sf36_mental": 38,
      "eq5d_index": 0.45
    },
    "self_efficacy": {
      "pain_self_efficacy": 22,
      "level": "low"
    }
  },

  "treatment": {
    "current_medications": [
      { "name": "Duloxetine", "class": "antidepressant", "dose": "60mg", "frequency": "daily", "effectiveness": 4 },
      { "name": "Pregabalin", "class": "anticonvulsant", "dose": "150mg", "frequency": "BID", "effectiveness": 3 }
    ],
    "opioid_use": {
      "current_use": false,
      "mme_daily": 0,
      "tapering_status": "completed"
    },
    "behavioral": {
      "cbt": {
        "sessions_completed": 4,
        "current_status": "in_progress"
      },
      "pain_neuroscience_education": {
        "completed": true,
        "understanding_score": 85
      }
    },
    "physical": {
      "exercise_program": {
        "type": ["walking", "swimming", "yoga"],
        "frequency_per_week": 3,
        "duration_minutes": 30,
        "adherence_percentage": 65
      }
    }
  }
}
```

---

## 5. Biomarker Codes

### 5.1 Pain Assessment Markers

| Code | Name | Unit | Reference |
|------|------|------|-----------|
| WIA-CP-PAIN-001 | NRS Score | 0-10 | <4 acceptable |
| WIA-CP-PAIN-002 | BPI Severity | 0-10 | <3 mild |
| WIA-CP-PAIN-003 | BPI Interference | 0-10 | <3 low impact |
| WIA-CP-PAIN-004 | Pain Duration | months | N/A |

### 5.2 Central Sensitization Markers

| Code | Name | Unit | Reference |
|------|------|------|-----------|
| WIA-CP-CS-001 | CSI Score | 0-100 | <40 normal |
| WIA-CP-CS-002 | Temporal Summation Ratio | ratio | <1.5 normal |
| WIA-CP-CS-003 | CPM Effect | % | >15% intact |
| WIA-CP-CS-004 | Neuroplasticity Index | 0-100 | <30 healthy |

### 5.3 Psychosocial Markers

| Code | Name | Unit | Reference |
|------|------|------|-----------|
| WIA-CP-PSY-001 | PCS Total | 0-52 | <20 low |
| WIA-CP-PSY-002 | TSK Score | 17-68 | <37 low fear |
| WIA-CP-PSY-003 | PHQ-9 | 0-27 | <5 minimal |
| WIA-CP-PSY-004 | Self-Efficacy | 0-60 | >40 high |

---

## 6. Validation Rules

### 6.1 Required Fields

- `patient_id` must be valid UUID
- `pain_type` must be from approved enum
- `assessment_date` must be valid ISO 8601 datetime
- At least one pain assessment measure required

### 6.2 Value Constraints

- All scale scores must be within defined ranges
- Duration must be ≥3 months for chronic pain classification
- Neuroplasticity index: 0-100

### 6.3 Clinical Logic Validation

- If CSI ≥40, `pain_type` should likely be "nociplastic" or "mixed"
- If opioid MME >90, high-risk flag required
- Widespread hyperalgesia suggests central sensitization

---

© 2026 WIA (World Certification Industry Association)
弘益人間 (홍익인간) · Benefit All Humanity
