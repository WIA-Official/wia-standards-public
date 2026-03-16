# WIA AI Diagnosis Standard - Phase 1: Data Format Specification

> **Version**: 1.0.0
> **Status**: Stable
> **Last Updated**: 2025
> **Standard**: WIA-MED-009

---

## 1. Overview

Phase 1 defines the standardized JSON schema for representing AI diagnostic data. This specification ensures interoperability across medical AI platforms, vendors, and healthcare systems by establishing a common language for diagnostic data exchange.

### 1.1 Design Principles

- **Interoperability**: Universal format compatible with any AI diagnosis system
- **Clinical Validity**: Support for evidence-based diagnostic criteria
- **Traceability**: Complete audit trail of AI decision-making process
- **Safety**: Built-in safeguards for clinical use

---

## 2. Core Schema

### 2.1 DiagnosticEvent Root Schema

```json
{
    "$schema": "https://json-schema.org/draft/2020-12/schema",
    "$id": "https://wiastandards.com/schemas/ai-diagnosis/v1/diagnostic-event.json",
    "title": "WIA Diagnostic Event",
    "description": "Standardized AI diagnostic event data format",
    "type": "object",
    "required": ["event_id", "timestamp", "version", "patient_context", "diagnosis"],
    "properties": {
        "event_id": {
            "type": "string",
            "format": "uuid",
            "description": "Unique identifier for the diagnostic event (UUID v4)"
        },
        "timestamp": {
            "type": "string",
            "format": "date-time",
            "description": "ISO 8601 timestamp of diagnosis generation"
        },
        "version": {
            "type": "string",
            "pattern": "^\\d+\\.\\d+\\.\\d+$",
            "description": "Schema version (SemVer format)"
        },
        "session_id": {
            "type": "string",
            "description": "Optional session identifier for grouping related diagnostics"
        },
        "patient_context": {
            "$ref": "#/$defs/PatientContext"
        },
        "diagnosis": {
            "$ref": "#/$defs/DiagnosisArray"
        },
        "clinical_findings": {
            "$ref": "#/$defs/ClinicalFindings"
        },
        "recommendations": {
            "$ref": "#/$defs/RecommendationArray"
        },
        "explainability": {
            "$ref": "#/$defs/Explainability"
        },
        "metadata": {
            "$ref": "#/$defs/Metadata"
        }
    }
}
```

### 2.2 Required Fields

| Field | Type | Description |
|-------|------|-------------|
| event_id | string (UUID) | Unique identifier for the diagnostic event |
| timestamp | string (ISO 8601) | Time of diagnosis generation |
| version | string (SemVer) | Schema version (e.g., "1.0.0") |
| patient_context | object | Anonymized patient information |
| diagnosis | array | Array of diagnostic hypotheses |

### 2.3 Optional Fields

| Field | Type | Description |
|-------|------|-------------|
| session_id | string | Groups multiple diagnostics in a session |
| clinical_findings | object | Structured clinical findings |
| recommendations | array | Clinical action recommendations |
| explainability | object | AI decision explanation data |
| metadata | object | System and performance metadata |

---

## 3. Patient Context

### 3.1 PatientContext Schema

```json
{
    "$defs": {
        "PatientContext": {
            "type": "object",
            "required": ["patient_id", "age_range", "sex"],
            "properties": {
                "patient_id": {
                    "type": "string",
                    "description": "Encrypted or anonymized patient identifier"
                },
                "age_range": {
                    "type": "string",
                    "enum": ["0-17", "18-39", "40-64", "65-79", "80+"]
                },
                "sex": {
                    "type": "string",
                    "enum": ["male", "female", "other", "unknown"]
                },
                "pregnancy_status": {
                    "type": "string",
                    "enum": ["not_applicable", "not_pregnant", "pregnant", "postpartum", "unknown"]
                },
                "medical_history": {
                    "$ref": "#/$defs/MedicalHistory"
                },
                "current_symptoms": {
                    "$ref": "#/$defs/SymptomArray"
                },
                "vital_signs": {
                    "$ref": "#/$defs/VitalSigns"
                }
            }
        }
    }
}
```

### 3.2 Medical History Schema

```json
{
    "$defs": {
        "MedicalHistory": {
            "type": "object",
            "properties": {
                "comorbidities": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                            "condition": {
                                "type": "string",
                                "description": "ICD-10 code or SNOMED CT code"
                            },
                            "condition_name": {
                                "type": "string"
                            },
                            "onset_date": {
                                "type": "string",
                                "format": "date"
                            },
                            "status": {
                                "type": "string",
                                "enum": ["active", "resolved", "chronic"]
                            }
                        }
                    }
                },
                "medications": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                            "name": {
                                "type": "string"
                            },
                            "rxnorm_code": {
                                "type": "string"
                            },
                            "dosage": {
                                "type": "string"
                            },
                            "frequency": {
                                "type": "string"
                            }
                        }
                    }
                },
                "allergies": {
                    "type": "array",
                    "items": {
                        "type": "string"
                    }
                },
                "family_history": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                            "condition": {
                                "type": "string"
                            },
                            "relation": {
                                "type": "string",
                                "enum": ["parent", "sibling", "grandparent", "other"]
                            }
                        }
                    }
                }
            }
        }
    }
}
```

### 3.3 Vital Signs Schema

```json
{
    "$defs": {
        "VitalSigns": {
            "type": "object",
            "properties": {
                "systolic_bp": {
                    "type": "number",
                    "description": "mmHg"
                },
                "diastolic_bp": {
                    "type": "number",
                    "description": "mmHg"
                },
                "heart_rate": {
                    "type": "number",
                    "description": "bpm"
                },
                "respiratory_rate": {
                    "type": "number",
                    "description": "breaths/min"
                },
                "temperature": {
                    "type": "number",
                    "description": "Celsius"
                },
                "oxygen_saturation": {
                    "type": "number",
                    "description": "SpO2 percentage"
                },
                "bmi": {
                    "type": "number",
                    "description": "Body Mass Index"
                }
            }
        }
    }
}
```

---

## 4. Diagnosis Representation

### 4.1 Diagnosis Object Schema

```json
{
    "$defs": {
        "Diagnosis": {
            "type": "object",
            "required": ["condition", "confidence", "severity"],
            "properties": {
                "condition": {
                    "type": "string",
                    "description": "ICD-10 or SNOMED CT code"
                },
                "condition_name": {
                    "type": "string",
                    "description": "Human-readable disease name"
                },
                "confidence": {
                    "type": "number",
                    "minimum": 0.0,
                    "maximum": 1.0,
                    "description": "AI confidence score"
                },
                "severity": {
                    "type": "string",
                    "enum": ["mild", "moderate", "severe", "critical"]
                },
                "urgency": {
                    "type": "string",
                    "enum": ["routine", "urgent", "emergent", "life_threatening"]
                },
                "differential_rank": {
                    "type": "integer",
                    "minimum": 1,
                    "description": "Rank in differential diagnosis (1 = most likely)"
                },
                "supporting_evidence": {
                    "$ref": "#/$defs/EvidenceArray"
                },
                "contradicting_evidence": {
                    "$ref": "#/$defs/EvidenceArray"
                }
            }
        }
    }
}
```

### 4.2 Confidence Interpretation

| Range | Level | Clinical Use |
|-------|-------|--------------|
| 0.0 - 0.3 | Very Low | Do not use for clinical decisions |
| 0.3 - 0.5 | Low | Consider as distant possibility only |
| 0.5 - 0.7 | Moderate | Include in differential diagnosis |
| 0.7 - 0.85 | High | Strong consideration for diagnosis |
| 0.85 - 1.0 | Very High | Primary diagnostic hypothesis |

### 4.3 Severity Classification

| Level | Description | Examples |
|-------|-------------|----------|
| mild | Self-limiting, minimal intervention | Common cold, mild gastritis |
| moderate | Requires treatment, not life-threatening | Uncomplicated pneumonia, UTI |
| severe | Serious condition, intensive treatment | Severe pneumonia, acute MI |
| critical | Life-threatening, immediate intervention | Septic shock, massive PE |

### 4.4 Evidence Schema

```json
{
    "$defs": {
        "Evidence": {
            "type": "object",
            "properties": {
                "type": {
                    "type": "string",
                    "enum": ["symptom", "sign", "lab_result", "imaging_finding", "risk_factor"]
                },
                "description": {
                    "type": "string"
                },
                "value": {
                    "type": ["string", "number", "boolean"]
                },
                "weight": {
                    "type": "number",
                    "minimum": 0.0,
                    "maximum": 1.0,
                    "description": "Contribution to diagnosis confidence"
                },
                "source": {
                    "type": "string",
                    "description": "Data source (patient_report, ehr, imaging, lab)"
                }
            }
        }
    }
}
```

---

## 5. Clinical Findings

### 5.1 ClinicalFindings Schema

```json
{
    "$defs": {
        "ClinicalFindings": {
            "type": "object",
            "properties": {
                "imaging_findings": {
                    "$ref": "#/$defs/ImagingFindings"
                },
                "laboratory_results": {
                    "$ref": "#/$defs/LaboratoryResults"
                },
                "physical_examination": {
                    "$ref": "#/$defs/PhysicalExamination"
                },
                "pathology": {
                    "$ref": "#/$defs/Pathology"
                }
            }
        }
    }
}
```

### 5.2 Imaging Findings Schema

```json
{
    "$defs": {
        "ImagingFindings": {
            "type": "object",
            "properties": {
                "modality": {
                    "type": "string",
                    "enum": ["x_ray", "ct", "mri", "ultrasound", "pet", "mammography"]
                },
                "body_region": {
                    "type": "string"
                },
                "findings": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                            "finding": {
                                "type": "string"
                            },
                            "location": {
                                "type": "string"
                            },
                            "size_mm": {
                                "type": "number"
                            },
                            "confidence": {
                                "type": "number",
                                "minimum": 0.0,
                                "maximum": 1.0
                            },
                            "coordinates": {
                                "type": "object",
                                "properties": {
                                    "x": {"type": "number"},
                                    "y": {"type": "number"},
                                    "z": {"type": "number"}
                                }
                            }
                        }
                    }
                },
                "image_quality": {
                    "type": "string",
                    "enum": ["excellent", "good", "adequate", "poor", "non_diagnostic"]
                }
            }
        }
    }
}
```

### 5.3 Laboratory Results Schema

```json
{
    "$defs": {
        "LaboratoryResults": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "test_name": {
                        "type": "string"
                    },
                    "loinc_code": {
                        "type": "string",
                        "description": "LOINC standardized code"
                    },
                    "value": {
                        "type": "number"
                    },
                    "unit": {
                        "type": "string"
                    },
                    "reference_range": {
                        "type": "object",
                        "properties": {
                            "min": {"type": "number"},
                            "max": {"type": "number"}
                        }
                    },
                    "abnormal_flag": {
                        "type": "string",
                        "enum": ["normal", "low", "high", "critical_low", "critical_high"]
                    },
                    "timestamp": {
                        "type": "string",
                        "format": "date-time"
                    }
                }
            }
        }
    }
}
```

---

## 6. Recommendations

### 6.1 Recommendation Schema

```json
{
    "$defs": {
        "Recommendation": {
            "type": "object",
            "required": ["type", "description", "priority"],
            "properties": {
                "type": {
                    "type": "string",
                    "enum": ["immediate_action", "further_testing", "specialist_referral", "treatment", "follow_up", "lifestyle_modification"]
                },
                "description": {
                    "type": "string"
                },
                "priority": {
                    "type": "string",
                    "enum": ["urgent", "high", "medium", "low"]
                },
                "timeframe": {
                    "type": "string",
                    "description": "e.g., '24 hours', '1 week', '1 month'"
                },
                "rationale": {
                    "type": "string",
                    "description": "Clinical reasoning for recommendation"
                }
            }
        }
    }
}
```

---

## 7. Explainability

### 7.1 Explainability Schema

```json
{
    "$defs": {
        "Explainability": {
            "type": "object",
            "properties": {
                "method": {
                    "type": "string",
                    "enum": ["SHAP", "LIME", "GradCAM", "attention", "rule_based"]
                },
                "feature_importance": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                            "feature": {
                                "type": "string"
                            },
                            "importance": {
                                "type": "number",
                                "minimum": 0.0,
                                "maximum": 1.0
                            }
                        }
                    }
                },
                "natural_language_explanation": {
                    "type": "string",
                    "description": "Human-readable explanation of AI reasoning"
                },
                "visualization_url": {
                    "type": "string",
                    "format": "uri",
                    "description": "URL to attention map, Grad-CAM, or other visualization"
                }
            }
        }
    }
}
```

---

## 8. Metadata

### 8.1 Metadata Schema

```json
{
    "$defs": {
        "Metadata": {
            "type": "object",
            "properties": {
                "ai_system": {
                    "type": "object",
                    "properties": {
                        "provider": {
                            "type": "string"
                        },
                        "model_name": {
                            "type": "string"
                        },
                        "model_version": {
                            "type": "string"
                        },
                        "training_date": {
                            "type": "string",
                            "format": "date"
                        },
                        "wia_certification": {
                            "type": "string",
                            "enum": ["bronze", "silver", "gold", "platinum", "none"]
                        },
                        "regulatory_approval": {
                            "type": "array",
                            "items": {
                                "type": "string",
                                "enum": ["FDA_510k", "FDA_De_Novo", "FDA_PMA", "CE_Mark", "MFDS", "TGA", "PMDA"]
                            }
                        }
                    }
                },
                "performance_metrics": {
                    "type": "object",
                    "properties": {
                        "sensitivity": {
                            "type": "number"
                        },
                        "specificity": {
                            "type": "number"
                        },
                        "auc_roc": {
                            "type": "number"
                        },
                        "validation_dataset": {
                            "type": "string"
                        }
                    }
                },
                "processing_time_ms": {
                    "type": "integer"
                },
                "institution": {
                    "type": "string",
                    "description": "Healthcare institution where diagnosis was generated"
                }
            }
        }
    }
}
```

---

## 9. Complete Example

```json
{
    "event_id": "550e8400-e29b-41d4-a716-446655440000",
    "timestamp": "2025-01-15T14:30:00.000Z",
    "version": "1.0.0",
    "session_id": "session-abc-123",
    "patient_context": {
        "patient_id": "encrypted-patient-xyz",
        "age_range": "40-64",
        "sex": "female",
        "medical_history": {
            "comorbidities": [
                {
                    "condition": "E11",
                    "condition_name": "Type 2 Diabetes Mellitus",
                    "onset_date": "2020-03-15",
                    "status": "active"
                }
            ],
            "medications": [
                {
                    "name": "Metformin",
                    "rxnorm_code": "860975",
                    "dosage": "500mg",
                    "frequency": "twice daily"
                }
            ]
        },
        "vital_signs": {
            "systolic_bp": 145,
            "diastolic_bp": 92,
            "heart_rate": 88,
            "temperature": 37.2,
            "oxygen_saturation": 97
        }
    },
    "diagnosis": [
        {
            "condition": "I50.9",
            "condition_name": "Heart Failure, Unspecified",
            "confidence": 0.87,
            "severity": "moderate",
            "urgency": "urgent",
            "differential_rank": 1,
            "supporting_evidence": [
                {
                    "type": "imaging_finding",
                    "description": "Cardiomegaly on chest X-ray",
                    "weight": 0.85
                },
                {
                    "type": "lab_result",
                    "description": "Elevated BNP (450 pg/mL)",
                    "value": 450,
                    "weight": 0.92
                }
            ]
        }
    ],
    "recommendations": [
        {
            "type": "specialist_referral",
            "description": "Cardiology consultation for heart failure management",
            "priority": "high",
            "timeframe": "48 hours",
            "rationale": "Newly diagnosed heart failure requires specialist evaluation"
        }
    ],
    "explainability": {
        "method": "SHAP",
        "feature_importance": [
            {
                "feature": "BNP level",
                "importance": 0.35
            },
            {
                "feature": "Cardiomegaly",
                "importance": 0.28
            },
            {
                "feature": "Dyspnea on exertion",
                "importance": 0.22
            }
        ],
        "natural_language_explanation": "The diagnosis of heart failure is primarily supported by elevated BNP levels (450 pg/mL) and cardiomegaly visible on chest imaging. Patient's symptoms of dyspnea on exertion and history of hypertension further support this diagnosis."
    },
    "metadata": {
        "ai_system": {
            "provider": "WIA-Certified-AI-Diagnostics",
            "model_name": "CardioAI-DX",
            "model_version": "v2.3.1",
            "wia_certification": "gold",
            "regulatory_approval": ["FDA_510k", "CE_Mark"]
        },
        "performance_metrics": {
            "sensitivity": 0.91,
            "specificity": 0.88,
            "auc_roc": 0.94
        },
        "processing_time_ms": 1250
    }
}
```

---

## 10. Validation

### 10.1 Schema Validation

All implementations MUST validate diagnostic events against the official JSON Schema before processing or storage.

### 10.2 Required Validation Rules

1. `event_id` MUST be a valid UUID v4
2. `timestamp` MUST be valid ISO 8601 format
3. `version` MUST match SemVer pattern
4. `confidence` MUST be in range [0.0, 1.0]
5. `condition` codes MUST be valid ICD-10 or SNOMED CT codes
6. `loinc_code` MUST be valid LOINC code when provided
7. Minimum one diagnosis with confidence ≥ 0.5 MUST be present

---

**홍익인간 (弘益人間)**: "Benefit all humanity"

The WIA AI Diagnosis Standard belongs to humanity. Free forever.

---

**Copyright 2025 SmileStory Inc. / WIA**
MIT License
