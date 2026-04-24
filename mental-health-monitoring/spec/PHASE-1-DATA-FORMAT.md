# WIA-MENTAL-HEALTH: Phase 1 - Data Format Specification
**Version:** 1.0
**Status:** Official
**Philosophy:** 弘益人間 (Benefit All Humanity)

## 1. Overview

This document specifies the standardized data formats for mental health assessment, treatment, and monitoring in the WIA-MENTAL-HEALTH standard. The core principle is that **neuroplasticity promotion** is the unified pathway to mental health recovery across all therapeutic modalities.

### 1.1 Scope

- Mental health assessment indices
- Biomarker data formats
- Neuroimaging metrics
- Treatment protocol schemas
- Digital therapeutics data
- Psychedelic-assisted therapy records

### 1.2 Unified Principle

```
Fragmented Approaches    →    Unified Principle         →    Universal Solution
Dozens of therapies      →    "Neuroplasticity Promotion"  →    Mental Health Recovery
```

## 2. Mental Health Index Schema

### 2.1 Core Assessment Structure

```json
{
  "@context": "https://wia.org/standards/MENTAL-HEALTH/v1",
  "@type": "MentalHealthAssessment",
  "id": "string (UUID v4)",
  "version": "1.0",
  "timestamp": "ISO 8601 datetime",
  "subject": {
    "id": "string (anonymized patient ID)",
    "age_group": "enum: child|adolescent|adult|elderly",
    "consent_status": "verified"
  },

  "mental_health_index": {
    "depression_score": {
      "value": 0.0-1.0,
      "instrument": "enum: PHQ-9|BDI-II|HAMD|CES-D",
      "raw_score": "number",
      "severity": "enum: none|mild|moderate|moderately-severe|severe"
    },
    "anxiety_score": {
      "value": 0.0-1.0,
      "instrument": "enum: GAD-7|BAI|STAI|HAM-A",
      "raw_score": "number",
      "severity": "enum: none|mild|moderate|severe"
    },
    "ptsd_score": {
      "value": 0.0-1.0,
      "instrument": "enum: PCL-5|CAPS-5|IES-R",
      "raw_score": "number",
      "meets_criteria": "boolean"
    },
    "wellbeing_score": {
      "value": 0.0-1.0,
      "instrument": "enum: WHO-5|WEMWBS|SWLS|PANAS",
      "raw_score": "number"
    },
    "resilience_score": {
      "value": 0.0-1.0,
      "instrument": "enum: CD-RISC|BRS|RSA",
      "raw_score": "number"
    }
  }
}
```

### 2.2 Score Normalization

All mental health indices MUST be normalized to a 0.0-1.0 scale:

| Score Range | Interpretation |
|-------------|----------------|
| 0.00 - 0.20 | Minimal/None |
| 0.21 - 0.40 | Mild |
| 0.41 - 0.60 | Moderate |
| 0.61 - 0.80 | Moderately Severe |
| 0.81 - 1.00 | Severe |

For wellbeing and resilience scores, higher values indicate better outcomes.

## 3. Biomarker Data Schema

### 3.1 Physiological Biomarkers

```json
{
  "biomarkers": {
    "stress_axis": {
      "cortisol": {
        "value": "number",
        "unit": "ng/mL",
        "time_of_day": "enum: morning|afternoon|evening|night",
        "sample_type": "enum: saliva|blood|urine|hair"
      },
      "cortisol_awakening_response": {
        "baseline": "number (ng/mL)",
        "peak": "number (ng/mL)",
        "auc": "number (area under curve)"
      },
      "dhea_s": {
        "value": "number",
        "unit": "μg/dL"
      }
    },
    "inflammatory_markers": {
      "crp": {
        "value": "number",
        "unit": "mg/L",
        "status": "enum: normal|elevated"
      },
      "il_6": {
        "value": "number",
        "unit": "pg/mL"
      },
      "tnf_alpha": {
        "value": "number",
        "unit": "pg/mL"
      },
      "il_1_beta": {
        "value": "number",
        "unit": "pg/mL"
      }
    },
    "neurotransmitters": {
      "serotonin": {
        "value": "number",
        "unit": "ng/mL",
        "sample_type": "enum: blood|csf"
      },
      "dopamine": {
        "value": "number",
        "unit": "pg/mL"
      },
      "gaba": {
        "value": "number",
        "unit": "ng/mL"
      },
      "glutamate": {
        "value": "number",
        "unit": "μmol/L"
      }
    },
    "heart_rate_variability": {
      "rmssd": {
        "value": "number",
        "unit": "ms"
      },
      "sdnn": {
        "value": "number",
        "unit": "ms"
      },
      "lf_hf_ratio": {
        "value": "number"
      },
      "coherence_score": {
        "value": 0.0-1.0
      }
    },
    "sleep_metrics": {
      "total_sleep_time": {
        "value": "number",
        "unit": "minutes"
      },
      "sleep_efficiency": {
        "value": 0.0-1.0
      },
      "rem_percentage": {
        "value": 0.0-1.0
      },
      "deep_sleep_percentage": {
        "value": 0.0-1.0
      },
      "awakenings": {
        "value": "number"
      }
    },
    "gut_brain_axis": {
      "microbiome_diversity": {
        "shannon_index": "number",
        "species_richness": "number"
      },
      "key_taxa": {
        "lactobacillus": "relative_abundance (0.0-1.0)",
        "bifidobacterium": "relative_abundance (0.0-1.0)",
        "firmicutes_bacteroidetes_ratio": "number"
      }
    }
  }
}
```

### 3.2 Biomarker Reference Ranges

| Biomarker | Normal Range | Unit | Clinical Significance |
|-----------|--------------|------|----------------------|
| Cortisol (AM) | 10-20 | ng/mL | Stress response |
| CRP | < 3.0 | mg/L | Inflammation |
| IL-6 | < 5.0 | pg/mL | Neuroinflammation |
| HRV (RMSSD) | 25-45 | ms | Autonomic balance |
| Sleep Efficiency | > 0.85 | ratio | Sleep quality |

## 4. Neuroimaging Data Schema

### 4.1 Functional Connectivity Metrics

```json
{
  "neuroimaging": {
    "scan_type": "enum: fMRI|PET|EEG|MEG|fNIRS",
    "acquisition_date": "ISO 8601 datetime",
    "device_info": {
      "manufacturer": "string",
      "model": "string",
      "field_strength": "number (Tesla, for MRI)"
    },
    "default_mode_network": {
      "connectivity_score": 0.0-1.0,
      "posterior_cingulate": "activation_level (-1.0 to 1.0)",
      "medial_prefrontal": "activation_level (-1.0 to 1.0)",
      "angular_gyrus": "activation_level (-1.0 to 1.0)",
      "within_network_connectivity": "number (z-score)"
    },
    "salience_network": {
      "connectivity_score": 0.0-1.0,
      "anterior_insula": "activation_level (-1.0 to 1.0)",
      "anterior_cingulate": "activation_level (-1.0 to 1.0)"
    },
    "central_executive_network": {
      "connectivity_score": 0.0-1.0,
      "dlpfc": "activation_level (-1.0 to 1.0)",
      "posterior_parietal": "activation_level (-1.0 to 1.0)"
    },
    "amygdala": {
      "activity_level": 0.0-1.0,
      "prefrontal_connectivity": "number (z-score)",
      "reactivity": "enum: hypo|normal|hyper"
    },
    "prefrontal_cortex": {
      "activity_level": 0.0-1.0,
      "dorsolateral_activity": "number (z-score)",
      "ventromedial_activity": "number (z-score)"
    },
    "hippocampus": {
      "volume_left": "number (mm³)",
      "volume_right": "number (mm³)",
      "volume_normalized": 0.0-1.0
    },
    "neuroplasticity_markers": {
      "bdnf_expression": "relative_level (0.0-1.0)",
      "synaptic_density_proxy": "number (z-score)",
      "network_flexibility": 0.0-1.0
    }
  }
}
```

### 4.2 EEG-Specific Metrics

```json
{
  "eeg_metrics": {
    "frequency_bands": {
      "delta": { "power": "μV²", "range": "1-4 Hz" },
      "theta": { "power": "μV²", "range": "4-8 Hz" },
      "alpha": { "power": "μV²", "range": "8-12 Hz" },
      "beta": { "power": "μV²", "range": "12-30 Hz" },
      "gamma": { "power": "μV²", "range": "30-100 Hz" }
    },
    "asymmetry": {
      "frontal_alpha_asymmetry": "number",
      "hemisphere": "enum: left|right|balanced"
    },
    "event_related_potentials": {
      "p300_amplitude": "μV",
      "p300_latency": "ms",
      "n400_amplitude": "μV"
    }
  }
}
```

## 5. Digital Phenotyping Schema

### 5.1 Passive Sensing Data

```json
{
  "digital_phenotype": {
    "collection_period": {
      "start": "ISO 8601 datetime",
      "end": "ISO 8601 datetime"
    },
    "mobility_patterns": {
      "home_time_ratio": 0.0-1.0,
      "location_variance": "number (km²)",
      "circadian_movement_score": 0.0-1.0,
      "significant_locations_count": "number"
    },
    "social_patterns": {
      "call_frequency": "number per day",
      "message_frequency": "number per day",
      "social_interaction_diversity": 0.0-1.0,
      "response_latency": "minutes (average)"
    },
    "device_usage": {
      "screen_time_daily": "minutes",
      "app_switches_per_hour": "number",
      "nighttime_usage": "boolean",
      "usage_variability": 0.0-1.0
    },
    "speech_patterns": {
      "speech_rate": "words per minute",
      "pause_duration_avg": "seconds",
      "voice_energy": 0.0-1.0,
      "prosody_score": 0.0-1.0
    },
    "activity_patterns": {
      "steps_daily": "number",
      "sedentary_time": "minutes",
      "activity_regularity": 0.0-1.0
    }
  }
}
```

## 6. Treatment Protocol Schema

### 6.1 Protocol Categories

```json
{
  "treatment_protocol": {
    "id": "string (UUID)",
    "category": "enum: psychotherapy|pharmacotherapy|neuromodulation|complementary|psychedelic|digital",
    "modality": {
      "psychotherapy": [
        "CBT", "DBT", "ACT", "EMDR", "IPT",
        "psychoanalytic", "humanistic", "integrative"
      ],
      "pharmacotherapy": [
        "SSRI", "SNRI", "TCA", "MAOI",
        "anxiolytic", "mood_stabilizer", "antipsychotic"
      ],
      "neuromodulation": [
        "TMS", "rTMS", "tDCS", "ECT", "VNS",
        "DBS", "neurofeedback"
      ],
      "complementary": [
        "mindfulness", "meditation", "yoga",
        "exercise", "nutrition", "acupuncture"
      ],
      "psychedelic": [
        "psilocybin", "MDMA", "ketamine",
        "LSD", "ayahuasca", "ibogaine"
      ],
      "digital": [
        "CBT_app", "mindfulness_app", "VR_therapy",
        "AI_chatbot", "biofeedback_app"
      ]
    },
    "evidence_level": "enum: 1A|1B|2A|2B|3|4|5",
    "target_condition": ["array of ICD-11 codes"],
    "neuroplasticity_mechanism": {
      "primary": "string (description)",
      "pathways": ["array of molecular/neural pathways"]
    }
  }
}
```

### 6.2 Session Data Format

```json
{
  "treatment_session": {
    "id": "string (UUID)",
    "protocol_id": "string (reference to protocol)",
    "session_number": "number",
    "date": "ISO 8601 datetime",
    "duration": "number (minutes)",
    "format": "enum: in_person|telehealth|app_based|group",
    "provider": {
      "id": "string (anonymized)",
      "credentials": "array of strings",
      "specialization": "string"
    },
    "interventions": [
      {
        "technique": "string",
        "duration": "number (minutes)",
        "notes": "string (encrypted)"
      }
    ],
    "homework_assigned": "boolean",
    "patient_engagement": 0.0-1.0,
    "adverse_events": {
      "occurred": "boolean",
      "severity": "enum: none|mild|moderate|severe",
      "description": "string (encrypted)"
    }
  }
}
```

## 7. Psychedelic-Assisted Therapy Schema

### 7.1 Psychedelic Session Protocol

```json
{
  "psychedelic_session": {
    "id": "string (UUID)",
    "substance": "enum: psilocybin|MDMA|ketamine|LSD|ayahuasca",
    "regulatory_status": "enum: approved|breakthrough_therapy|clinical_trial|compassionate_use",
    "phase": "enum: preparation|dosing|integration",

    "preparation_session": {
      "sessions_completed": "number",
      "therapeutic_alliance_score": 0.0-1.0,
      "intentions_set": "boolean",
      "medical_clearance": "verified"
    },

    "dosing_session": {
      "date": "ISO 8601 datetime",
      "dose": {
        "amount": "number",
        "unit": "mg|μg",
        "route": "enum: oral|sublingual|intramuscular|intranasal"
      },
      "set_and_setting": {
        "environment": "enum: clinical|retreat|home",
        "music_protocol": "string",
        "lighting": "string",
        "support_persons": "number"
      },
      "monitoring": {
        "vital_signs_frequency": "minutes",
        "psychological_support_level": "enum: minimal|moderate|intensive"
      },
      "experience_measures": {
        "mystical_experience_questionnaire": 0.0-1.0,
        "ego_dissolution_inventory": 0.0-1.0,
        "emotional_breakthrough": 0.0-1.0,
        "challenging_experience": 0.0-1.0
      }
    },

    "integration_sessions": {
      "sessions_scheduled": "number",
      "sessions_completed": "number",
      "techniques": ["array: talk_therapy|somatic|art|movement|journaling"],
      "insights_processed": "boolean"
    },

    "safety_protocol": {
      "screening_completed": "boolean",
      "contraindications_cleared": "boolean",
      "emergency_protocol": "documented",
      "follow_up_scheduled": "boolean"
    }
  }
}
```

## 8. Digital Therapeutics Schema

### 8.1 Digital Therapy Session

```json
{
  "digital_therapeutic": {
    "product_id": "string",
    "regulatory_status": "enum: FDA_cleared|CE_marked|pending|non_regulated",
    "therapeutic_class": "enum: prescription|OTC",
    "target_condition": "ICD-11 code",

    "session": {
      "id": "string (UUID)",
      "date": "ISO 8601 datetime",
      "duration": "number (minutes)",
      "completion_rate": 0.0-1.0,
      "modules_completed": ["array of module IDs"],
      "interactions": "number",
      "engagement_score": 0.0-1.0
    },

    "biometric_integration": {
      "wearable_connected": "boolean",
      "data_synced": ["array: hrv|sleep|activity|location"],
      "just_in_time_interventions": "number triggered"
    },

    "ai_components": {
      "chatbot_conversations": "number",
      "personalization_level": 0.0-1.0,
      "recommendation_accuracy": 0.0-1.0
    },

    "outcomes": {
      "symptom_change": "number (% change)",
      "adherence_rate": 0.0-1.0,
      "user_satisfaction": 0.0-1.0
    }
  }
}
```

## 9. Privacy and Encryption Requirements

### 9.1 Data Protection Standards

All mental health data MUST comply with:

1. **HIPAA** (US) - Health Insurance Portability and Accountability Act
2. **GDPR** (EU) - General Data Protection Regulation
3. **PIPEDA** (Canada) - Personal Information Protection and Electronic Documents Act

### 9.2 Encryption Requirements

```json
{
  "security": {
    "encryption_at_rest": "AES-256-GCM",
    "encryption_in_transit": "TLS 1.3",
    "key_management": "HSM or equivalent",
    "pseudonymization": "required",
    "audit_logging": "required",
    "data_retention": "configurable per jurisdiction"
  }
}
```

### 9.3 Consent Schema

```json
{
  "consent": {
    "id": "string (UUID)",
    "subject_id": "string (pseudonymized)",
    "timestamp": "ISO 8601 datetime",
    "consent_type": "enum: treatment|research|data_sharing",
    "scope": ["array of data categories"],
    "duration": "ISO 8601 duration",
    "withdrawal_allowed": true,
    "guardian_consent": "boolean (for minors)",
    "capacity_verified": "boolean",
    "signature": "cryptographic_signature"
  }
}
```

## 10. Validation Requirements

All mental health data MUST pass these validation checks:

1. ✓ Valid JSON structure with required fields
2. ✓ All scores within defined ranges (0.0-1.0)
3. ✓ Valid instrument/questionnaire references
4. ✓ ISO 8601 compliant timestamps
5. ✓ Pseudonymized patient identifiers
6. ✓ Valid consent records attached
7. ✓ Encryption verification for sensitive fields
8. ✓ Provider credentials verified
9. ✓ ICD-11 code validation for diagnoses
10. ✓ Regulatory compliance markers present

---

**弘益人間 (Benefit All Humanity)**

*© 2025 WIA - World Certification Industry Association*
*MIT License*
