# WIA Clinical Decision Support Standard - Phase 1: Data Format Specification

> **Version**: 1.0.0
> **Status**: Stable
> **Last Updated**: 2025
> **Standard**: WIA-MED-015

---

## 1. Overview

Phase 1 defines the standardized JSON schema for Clinical Decision Support (CDS) data. This specification ensures interoperability across healthcare systems by establishing a common format for CDS recommendations, alerts, and clinical guidelines.

### 1.1 Design Principles

- **Interoperability**: Universal format compatible with any CDS system
- **Evidence-Based**: Link recommendations to clinical evidence
- **Safety-First**: Built-in alert prioritization and fatigue prevention
- **Actionable**: Clear, implementable clinical recommendations

---

## 2. Core Schema

### 2.1 CDSRecommendation Root Schema

```json
{
    "$schema": "https://json-schema.org/draft/2020-12/schema",
    "$id": "https://wiastandards.com/schemas/cds/v1/cds-recommendation.json",
    "title": "WIA CDS Recommendation",
    "description": "Standardized clinical decision support recommendation",
    "type": "object",
    "required": ["recommendation_id", "timestamp", "version", "trigger", "recommendation"],
    "properties": {
        "recommendation_id": {
            "type": "string",
            "format": "uuid"
        },
        "timestamp": {
            "type": "string",
            "format": "date-time"
        },
        "version": {
            "type": "string",
            "pattern": "^\\d+\\.\\d+\\.\\d+$"
        },
        "patient_context": {
            "$ref": "#/$defs/PatientContext"
        },
        "trigger": {
            "$ref": "#/$defs/Trigger"
        },
        "recommendation": {
            "$ref": "#/$defs/RecommendationArray"
        },
        "alert": {
            "$ref": "#/$defs/Alert"
        },
        "evidence": {
            "$ref": "#/$defs/Evidence"
        },
        "metadata": {
            "$ref": "#/$defs/Metadata"
        }
    }
}
```

### 2.2 Trigger Schema

```json
{
    "$defs": {
        "Trigger": {
            "type": "object",
            "required": ["trigger_type", "trigger_data"],
            "properties": {
                "trigger_type": {
                    "type": "string",
                    "enum": ["medication_order", "lab_result", "vital_sign", "diagnosis", "procedure_order"]
                },
                "trigger_data": {
                    "type": "object"
                },
                "context": {
                    "type": "string",
                    "enum": ["order_entry", "documentation", "results_review", "patient_chart_review"]
                }
            }
        }
    }
}
```

---

## 3. Medication Safety Alerts

### 3.1 Drug-Drug Interaction Schema

```json
{
    "$defs": {
        "DrugDrugInteraction": {
            "type": "object",
            "properties": {
                "interaction_id": {
                    "type": "string"
                },
                "severity": {
                    "type": "string",
                    "enum": ["contraindicated", "major", "moderate", "minor"]
                },
                "drug_1": {
                    "type": "object",
                    "properties": {
                        "name": {"type": "string"},
                        "rxnorm_code": {"type": "string"}
                    }
                },
                "drug_2": {
                    "type": "object",
                    "properties": {
                        "name": {"type": "string"},
                        "rxnorm_code": {"type": "string"}
                    }
                },
                "mechanism": {
                    "type": "string",
                    "description": "Pharmacodynamic or pharmacokinetic mechanism"
                },
                "clinical_effect": {
                    "type": "string",
                    "description": "Expected clinical outcome of interaction"
                },
                "management": {
                    "type": "array",
                    "items": {
                        "type": "string"
                    },
                    "description": "Recommended management strategies"
                },
                "evidence_level": {
                    "type": "string",
                    "enum": ["established", "probable", "suspected", "theoretical"]
                }
            }
        }
    }
}
```

### 3.2 Allergy Alert Schema

```json
{
    "$defs": {
        "AllergyAlert": {
            "type": "object",
            "properties": {
                "alert_id": {
                    "type": "string"
                },
                "allergen": {
                    "type": "string"
                },
                "ordered_medication": {
                    "type": "object",
                    "properties": {
                        "name": {"type": "string"},
                        "rxnorm_code": {"type": "string"}
                    }
                },
                "cross_sensitivity": {
                    "type": "boolean",
                    "description": "True if cross-reaction possible"
                },
                "reaction_type": {
                    "type": "string",
                    "enum": ["anaphylaxis", "severe", "moderate", "mild", "intolerance"]
                },
                "previous_reactions": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                            "reaction": {"type": "string"},
                            "date": {"type": "string", "format": "date"}
                        }
                    }
                },
                "recommendation": {
                    "type": "string",
                    "enum": ["do_not_order", "use_with_caution", "monitor_closely"]
                }
            }
        }
    }
}
```

### 3.3 Dose Range Check Schema

```json
{
    "$defs": {
        "DoseRangeCheck": {
            "type": "object",
            "properties": {
                "medication": {
                    "type": "string"
                },
                "ordered_dose": {
                    "type": "object",
                    "properties": {
                        "amount": {"type": "number"},
                        "unit": {"type": "string"},
                        "frequency": {"type": "string"}
                    }
                },
                "recommended_range": {
                    "type": "object",
                    "properties": {
                        "min_dose": {"type": "number"},
                        "max_dose": {"type": "number"},
                        "unit": {"type": "string"}
                    }
                },
                "patient_factors": {
                    "type": "object",
                    "properties": {
                        "age": {"type": "number"},
                        "weight_kg": {"type": "number"},
                        "renal_function": {
                            "type": "object",
                            "properties": {
                                "creatinine_clearance": {"type": "number"},
                                "adjustment_needed": {"type": "boolean"}
                            }
                        },
                        "hepatic_function": {
                            "type": "object",
                            "properties": {
                                "child_pugh_score": {"type": "string"},
                                "adjustment_needed": {"type": "boolean"}
                            }
                        }
                    }
                },
                "alert_type": {
                    "type": "string",
                    "enum": ["overdose", "underdose", "within_range"]
                },
                "suggested_dose": {
                    "type": "object",
                    "properties": {
                        "amount": {"type": "number"},
                        "unit": {"type": "string"},
                        "frequency": {"type": "string"},
                        "rationale": {"type": "string"}
                    }
                }
            }
        }
    }
}
```

---

## 4. Diagnostic Support

### 4.1 Differential Diagnosis Schema

```json
{
    "$defs": {
        "DifferentialDiagnosis": {
            "type": "object",
            "properties": {
                "chief_complaint": {
                    "type": "string"
                },
                "presenting_symptoms": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                            "symptom": {"type": "string"},
                            "duration": {"type": "string"},
                            "severity": {"type": "string", "enum": ["mild", "moderate", "severe"]}
                        }
                    }
                },
                "differential_list": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                            "condition": {"type": "string"},
                            "icd10_code": {"type": "string"},
                            "probability": {"type": "number", "minimum": 0.0, "maximum": 1.0},
                            "urgency": {"type": "string", "enum": ["emergent", "urgent", "routine"]},
                            "supporting_features": {"type": "array", "items": {"type": "string"}},
                            "discriminating_features": {"type": "array", "items": {"type": "string"}},
                            "suggested_workup": {
                                "type": "array",
                                "items": {
                                    "type": "object",
                                    "properties": {
                                        "test": {"type": "string"},
                                        "rationale": {"type": "string"}
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
```

---

## 5. Clinical Guidelines Integration

### 5.1 Guideline Recommendation Schema

```json
{
    "$defs": {
        "GuidelineRecommendation": {
            "type": "object",
            "properties": {
                "guideline": {
                    "type": "object",
                    "properties": {
                        "name": {"type": "string"},
                        "source": {"type": "string"},
                        "version": {"type": "string"},
                        "publication_date": {"type": "string", "format": "date"}
                    }
                },
                "condition": {
                    "type": "string"
                },
                "recommendation_text": {
                    "type": "string"
                },
                "strength": {
                    "type": "string",
                    "enum": ["strong", "moderate", "weak", "conditional"]
                },
                "evidence_quality": {
                    "type": "string",
                    "enum": ["high", "moderate", "low", "very_low"]
                },
                "applicable": {
                    "type": "boolean",
                    "description": "Is recommendation applicable to current patient?"
                },
                "criteria_met": {
                    "type": "array",
                    "items": {"type": "string"}
                },
                "criteria_not_met": {
                    "type": "array",
                    "items": {"type": "string"}
                }
            }
        }
    }
}
```

---

## 6. Alert Management

### 6.1 Alert Schema

```json
{
    "$defs": {
        "Alert": {
            "type": "object",
            "required": ["alert_id", "severity", "message"],
            "properties": {
                "alert_id": {
                    "type": "string"
                },
                "severity": {
                    "type": "string",
                    "enum": ["critical", "high", "medium", "low", "info"]
                },
                "category": {
                    "type": "string",
                    "enum": ["drug_safety", "diagnostic", "preventive_care", "guideline_adherence", "documentation"]
                },
                "message": {
                    "type": "string",
                    "description": "Clear, concise alert message"
                },
                "detailed_explanation": {
                    "type": "string"
                },
                "interruptive": {
                    "type": "boolean",
                    "description": "Should alert interrupt workflow?"
                },
                "dismissible": {
                    "type": "boolean"
                },
                "response_options": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                            "action": {"type": "string"},
                            "label": {"type": "string"},
                            "requires_reason": {"type": "boolean"}
                        }
                    }
                }
            }
        }
    }
}
```

### 6.2 Alert Fatigue Prevention

| Alert Severity | Interruptive | Dismissible | Display Duration |
|----------------|--------------|-------------|------------------|
| Critical | Yes | No | Until acknowledged |
| High | Yes | Yes (with reason) | Until acknowledged |
| Medium | No | Yes | 30 seconds |
| Low | No | Yes | 15 seconds |
| Info | No | Yes | 10 seconds |

---

## 7. Evidence and References

### 7.1 Evidence Schema

```json
{
    "$defs": {
        "Evidence": {
            "type": "object",
            "properties": {
                "source": {
                    "type": "string",
                    "enum": ["clinical_trial", "guideline", "systematic_review", "expert_opinion", "local_protocol"]
                },
                "reference": {
                    "type": "object",
                    "properties": {
                        "title": {"type": "string"},
                        "authors": {"type": "array", "items": {"type": "string"}},
                        "journal": {"type": "string"},
                        "year": {"type": "integer"},
                        "doi": {"type": "string"},
                        "pubmed_id": {"type": "string"}
                    }
                },
                "level_of_evidence": {
                    "type": "string",
                    "enum": ["1a", "1b", "2a", "2b", "3a", "3b", "4", "5"]
                },
                "recommendation_grade": {
                    "type": "string",
                    "enum": ["A", "B", "C", "D"]
                }
            }
        }
    }
}
```

---

## 8. Preventive Care Recommendations

### 8.1 Screening Recommendation Schema

```json
{
    "$defs": {
        "ScreeningRecommendation": {
            "type": "object",
            "properties": {
                "screening_test": {
                    "type": "string"
                },
                "condition_screened": {
                    "type": "string"
                },
                "patient_eligible": {
                    "type": "boolean"
                },
                "eligibility_criteria": {
                    "type": "object",
                    "properties": {
                        "age_range": {"type": "string"},
                        "sex": {"type": "string"},
                        "risk_factors": {"type": "array", "items": {"type": "string"}}
                    }
                },
                "last_performed": {
                    "type": "string",
                    "format": "date"
                },
                "next_due_date": {
                    "type": "string",
                    "format": "date"
                },
                "overdue": {
                    "type": "boolean"
                },
                "overdue_by_days": {
                    "type": "integer"
                },
                "guideline_source": {
                    "type": "string"
                },
                "uspstf_grade": {
                    "type": "string",
                    "enum": ["A", "B", "C", "D", "I"]
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
    "recommendation_id": "cds-rec-12345",
    "timestamp": "2025-01-15T14:30:00.000Z",
    "version": "1.0.0",
    "patient_context": {
        "patient_id": "encrypted-patient-xyz",
        "age": 62,
        "sex": "male",
        "conditions": ["I10", "E11"],
        "medications": ["Metformin", "Lisinopril"]
    },
    "trigger": {
        "trigger_type": "medication_order",
        "trigger_data": {
            "medication": "Clarithromycin",
            "rxnorm_code": "21212"
        },
        "context": "order_entry"
    },
    "alert": {
        "alert_id": "alert-67890",
        "severity": "high",
        "category": "drug_safety",
        "message": "Drug-Drug Interaction: Clarithromycin + Simvastatin",
        "detailed_explanation": "Concurrent use of clarithromycin with simvastatin increases risk of rhabdomyolysis. Consider temporarily discontinuing simvastatin during clarithromycin therapy.",
        "interruptive": true,
        "dismissible": true,
        "response_options": [
            {
                "action": "change_antibiotic",
                "label": "Select alternative antibiotic",
                "requires_reason": false
            },
            {
                "action": "hold_statin",
                "label": "Temporarily hold simvastatin",
                "requires_reason": false
            },
            {
                "action": "override",
                "label": "Override alert",
                "requires_reason": true
            }
        ]
    },
    "recommendation": [
        {
            "type": "medication_alternative",
            "description": "Consider azithromycin as alternative antibiotic",
            "priority": "high",
            "evidence_level": "established"
        }
    ],
    "evidence": {
        "source": "guideline",
        "reference": {
            "title": "Drug Interaction Database",
            "authors": ["Lexicomp"],
            "year": 2025
        },
        "level_of_evidence": "1a"
    }
}
```

---

## 10. Validation Rules

1. `recommendation_id` MUST be a valid UUID v4
2. `timestamp` MUST be valid ISO 8601 format
3. `severity` for critical/high alerts MUST include detailed_explanation
4. Drug codes MUST use RxNorm
5. Condition codes MUST use ICD-10 or SNOMED CT
6. All interruptive alerts MUST provide clear response options

---

**홍익인간 (弘益人間)**: "Benefit all humanity"

The WIA Clinical Decision Support Standard belongs to humanity. Free forever.

---

**Copyright 2025 SmileStory Inc. / WIA**
MIT License
