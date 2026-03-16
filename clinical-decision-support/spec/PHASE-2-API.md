# WIA Clinical Decision Support Standard - Phase 2: API Interface Specification

> **Version**: 1.0.0
> **Status**: Stable
> **Last Updated**: 2025
> **Standard**: WIA-MED-015

---

## 1. Overview

Phase 2 defines the RESTful API interface for Clinical Decision Support (CDS) services. This specification ensures consistent interaction patterns across different healthcare systems while maintaining clinical safety and enabling real-time decision support.

### 1.1 Design Principles

- **RESTful**: Follow REST architectural constraints
- **Real-time**: Support low-latency clinical decision alerts
- **Interruptive**: Intelligent alert delivery based on severity
- **Evidence-Based**: Link all recommendations to clinical evidence
- **HIPAA Compliant**: Full audit trail and data protection

---

## 2. API Structure

### 2.1 Base URL

```
https://api.{provider}.com/wia/cds/v1/
```

### 2.2 Versioning

| Version | URL Path | Status |
|---------|----------|--------|
| v1 | /wia/cds/v1/ | Current (Stable) |
| v1-beta | /wia/cds/v1-beta/ | Beta features |
| v2 | /wia/cds/v2/ | Planned |

### 2.3 Common Headers

#### Request Headers

```http
Authorization: Bearer {access_token}
Content-Type: application/json
Accept: application/json
X-WIA-Version: 1.0.0
X-Request-ID: {uuid}
X-Institution-ID: {institution_id}
X-EHR-System: {ehr_vendor}
```

#### Response Headers

```http
Content-Type: application/json
X-WIA-Version: 1.0.0
X-Request-ID: {uuid}
X-Processing-Time-Ms: 45
X-Alert-Count: 3
```

---

## 3. CDS Hooks Integration

### 3.1 Discovery Endpoint

**Endpoint**: `GET /wia/cds/v1/cds-services`

**Response**:
```json
{
    "services": [
        {
            "hook": "medication-prescribe",
            "title": "Drug-Drug Interaction Checker",
            "description": "Checks for dangerous drug interactions",
            "id": "wia-ddi-checker",
            "prefetch": {
                "patient": "Patient/{{context.patientId}}",
                "medications": "MedicationRequest?patient={{context.patientId}}&status=active"
            }
        },
        {
            "hook": "order-select",
            "title": "Dose Range Checker",
            "description": "Validates medication dose ranges",
            "id": "wia-dose-checker"
        },
        {
            "hook": "patient-view",
            "title": "Preventive Care Reminders",
            "description": "Alerts for overdue screenings",
            "id": "wia-preventive-care"
        }
    ]
}
```

### 3.2 Service Invocation

**Endpoint**: `POST /wia/cds/v1/cds-services/{service_id}`

**Request**:
```json
{
    "hookInstance": "d1577c69-dfbe-44ad-ba6d-3e05e953b2ea",
    "hook": "medication-prescribe",
    "context": {
        "userId": "Practitioner/12345",
        "patientId": "Patient/67890",
        "medications": [
            {
                "resourceType": "MedicationRequest",
                "medicationCodeableConcept": {
                    "coding": [
                        {
                            "system": "http://www.nlm.nih.gov/research/umls/rxnorm",
                            "code": "197361",
                            "display": "Simvastatin 20 MG"
                        }
                    ]
                }
            }
        ],
        "draftOrders": [
            {
                "resourceType": "MedicationRequest",
                "medicationCodeableConcept": {
                    "coding": [
                        {
                            "system": "http://www.nlm.nih.gov/research/umls/rxnorm",
                            "code": "197517",
                            "display": "Clarithromycin 500 MG"
                        }
                    ]
                }
            }
        ]
    },
    "prefetch": {
        "patient": {...},
        "allergies": {...}
    }
}
```

**Response**:
```json
{
    "cards": [
        {
            "uuid": "card-12345",
            "summary": "Drug-Drug Interaction: Clarithromycin + Simvastatin",
            "detail": "Concurrent use increases risk of rhabdomyolysis. Consider temporarily discontinuing simvastatin.",
            "indicator": "critical",
            "source": {
                "label": "WIA Clinical Decision Support",
                "url": "https://wiastandards.com/cds",
                "icon": "https://wiastandards.com/icons/cds.png"
            },
            "suggestions": [
                {
                    "label": "Select alternative antibiotic",
                    "uuid": "suggestion-1",
                    "actions": [
                        {
                            "type": "delete",
                            "description": "Remove Clarithromycin order"
                        }
                    ]
                },
                {
                    "label": "Hold simvastatin temporarily",
                    "uuid": "suggestion-2",
                    "actions": [
                        {
                            "type": "update",
                            "description": "Suspend simvastatin for 7 days"
                        }
                    ]
                }
            ],
            "selectionBehavior": "any",
            "overrideReasons": [
                {
                    "code": "patient-aware",
                    "display": "Patient aware of risk and agrees to proceed"
                },
                {
                    "code": "no-alternative",
                    "display": "No suitable alternative available"
                }
            ]
        }
    ]
}
```

---

## 4. Alert Endpoints

### 4.1 Check Drug Interactions

**Endpoint**: `POST /wia/cds/v1/check/interactions`

**Request**:
```json
{
    "patient_id": "encrypted-patient-xyz",
    "current_medications": [
        {
            "name": "Simvastatin",
            "rxnorm_code": "197361",
            "dose": "20mg",
            "frequency": "daily"
        }
    ],
    "new_medication": {
        "name": "Clarithromycin",
        "rxnorm_code": "197517",
        "dose": "500mg",
        "frequency": "twice daily"
    }
}
```

**Response**:
```json
{
    "check_id": "ddi-check-12345",
    "timestamp": "2025-01-15T14:30:00.000Z",
    "interactions_found": 1,
    "interactions": [
        {
            "severity": "contraindicated",
            "drug_1": {
                "name": "Simvastatin",
                "rxnorm_code": "197361"
            },
            "drug_2": {
                "name": "Clarithromycin",
                "rxnorm_code": "197517"
            },
            "mechanism": "CYP3A4 inhibition by clarithromycin increases simvastatin levels",
            "clinical_effect": "Increased risk of myopathy and rhabdomyolysis",
            "evidence_level": "established",
            "management": [
                "Temporarily discontinue simvastatin during clarithromycin therapy",
                "Consider alternative antibiotic (azithromycin)",
                "If must use together, limit simvastatin to 10mg/day"
            ]
        }
    ],
    "recommendations": [
        {
            "type": "alternative",
            "medication": "Azithromycin",
            "rationale": "Azithromycin does not inhibit CYP3A4"
        }
    ]
}
```

### 4.2 Check Allergies

**Endpoint**: `POST /wia/cds/v1/check/allergies`

**Request**:
```json
{
    "patient_id": "encrypted-patient-xyz",
    "known_allergies": [
        {
            "allergen": "Penicillin",
            "reaction": "anaphylaxis",
            "severity": "severe"
        }
    ],
    "medication_to_check": {
        "name": "Amoxicillin",
        "rxnorm_code": "723",
        "drug_class": "beta-lactam"
    }
}
```

**Response**:
```json
{
    "check_id": "allergy-check-67890",
    "timestamp": "2025-01-15T14:30:00.000Z",
    "allergy_alert": true,
    "alert": {
        "severity": "critical",
        "allergen": "Penicillin",
        "ordered_medication": "Amoxicillin",
        "cross_sensitivity": true,
        "cross_sensitivity_rate": 0.08,
        "previous_reaction": "anaphylaxis",
        "recommendation": "do_not_order",
        "message": "Patient has documented anaphylaxis to Penicillin. Amoxicillin has 8% cross-sensitivity rate with penicillin class.",
        "alternatives": [
            {
                "medication": "Azithromycin",
                "rationale": "Macrolide antibiotic, no cross-sensitivity with beta-lactams"
            },
            {
                "medication": "Doxycycline",
                "rationale": "Tetracycline antibiotic, no cross-sensitivity with beta-lactams"
            }
        ]
    }
}
```

### 4.3 Check Dose

**Endpoint**: `POST /wia/cds/v1/check/dose`

**Request**:
```json
{
    "patient_context": {
        "patient_id": "encrypted-patient-xyz",
        "age": 72,
        "weight_kg": 65,
        "creatinine_clearance": 35,
        "hepatic_function": "normal"
    },
    "medication_order": {
        "name": "Metformin",
        "rxnorm_code": "6809",
        "dose": 1000,
        "unit": "mg",
        "frequency": "twice daily"
    }
}
```

**Response**:
```json
{
    "check_id": "dose-check-11111",
    "timestamp": "2025-01-15T14:30:00.000Z",
    "dose_alert": true,
    "alert": {
        "alert_type": "overdose",
        "severity": "high",
        "message": "Ordered dose exceeds recommended maximum for patient with renal impairment",
        "ordered_dose": {
            "amount": 1000,
            "unit": "mg",
            "frequency": "twice daily",
            "daily_total": 2000
        },
        "recommended_range": {
            "max_dose": 500,
            "unit": "mg",
            "frequency": "twice daily",
            "daily_maximum": 1000,
            "rationale": "CrCl 35 mL/min requires dose reduction"
        },
        "patient_factors": {
            "renal_function": {
                "creatinine_clearance": 35,
                "stage": "CKD Stage 3b",
                "adjustment_required": true
            }
        },
        "suggested_dose": {
            "amount": 500,
            "unit": "mg",
            "frequency": "twice daily",
            "daily_total": 1000
        }
    }
}
```

---

## 5. Preventive Care

### 5.1 Get Screening Recommendations

**Endpoint**: `GET /wia/cds/v1/preventive-care/{patient_id}`

**Response**:
```json
{
    "patient_id": "encrypted-patient-xyz",
    "timestamp": "2025-01-15T14:30:00.000Z",
    "screenings_due": [
        {
            "screening": "Colorectal Cancer Screening",
            "method": "Colonoscopy",
            "last_performed": "2015-03-20",
            "next_due": "2025-03-20",
            "overdue": false,
            "uspstf_grade": "A",
            "eligibility_criteria": {
                "age_range": "45-75",
                "patient_age": 67,
                "eligible": true
            }
        },
        {
            "screening": "Mammography",
            "method": "Digital Mammogram",
            "last_performed": "2022-08-15",
            "next_due": "2024-08-15",
            "overdue": true,
            "overdue_by_days": 153,
            "uspstf_grade": "B",
            "eligibility_criteria": {
                "age_range": "50-74",
                "sex": "female",
                "patient_age": 62,
                "patient_sex": "female",
                "eligible": true
            }
        }
    ],
    "immunizations_due": [
        {
            "vaccine": "Influenza",
            "last_received": "2023-10-15",
            "next_due": "2024-10-01",
            "overdue": true,
            "recommendation": "Annual influenza vaccination recommended"
        }
    ]
}
```

---

## 6. Guideline Recommendations

### 6.1 Get Applicable Guidelines

**Endpoint**: `POST /wia/cds/v1/guidelines/check`

**Request**:
```json
{
    "patient_context": {
        "patient_id": "encrypted-patient-xyz",
        "age": 58,
        "sex": "male",
        "conditions": ["I10", "E11"],
        "medications": ["Lisinopril", "Metformin"],
        "lab_results": {
            "hba1c": 7.8,
            "ldl": 145,
            "egfr": 62
        }
    },
    "focus_area": "diabetes_management"
}
```

**Response**:
```json
{
    "check_id": "guideline-check-22222",
    "timestamp": "2025-01-15T14:30:00.000Z",
    "recommendations": [
        {
            "guideline": {
                "name": "ADA Standards of Care in Diabetes",
                "source": "American Diabetes Association",
                "version": "2025",
                "publication_date": "2025-01-01"
            },
            "recommendation_text": "Consider adding SGLT2 inhibitor for patients with T2DM and CKD",
            "strength": "strong",
            "evidence_quality": "high",
            "applicable": true,
            "criteria_met": [
                "Type 2 Diabetes (E11)",
                "eGFR 62 mL/min (CKD Stage 2)",
                "No SGLT2 inhibitor in current medications"
            ],
            "suggested_action": {
                "type": "medication_addition",
                "medication": "Empagliflozin",
                "dose": "10mg daily",
                "rationale": "Renal and cardiovascular protection in diabetic CKD"
            }
        },
        {
            "guideline": {
                "name": "ACC/AHA Cholesterol Guidelines",
                "source": "American College of Cardiology",
                "version": "2024"
            },
            "recommendation_text": "High-intensity statin for diabetic patients aged 40-75 with LDL ≥70",
            "strength": "strong",
            "evidence_quality": "high",
            "applicable": true,
            "criteria_met": [
                "Diabetes mellitus",
                "Age 58 (40-75 range)",
                "LDL 145 mg/dL (≥70)"
            ],
            "suggested_action": {
                "type": "medication_addition",
                "medication": "Atorvastatin",
                "dose": "40mg daily"
            }
        }
    ]
}
```

---

## 7. Alert Analytics

### 7.1 Get Alert Statistics

**Endpoint**: `GET /wia/cds/v1/analytics/alerts`

**Query Parameters**:
- `start_date`: Start date (ISO 8601)
- `end_date`: End date (ISO 8601)
- `institution_id`: Filter by institution

**Response**:
```json
{
    "period": {
        "start": "2025-01-01",
        "end": "2025-01-15"
    },
    "total_alerts": 15234,
    "by_severity": {
        "critical": 523,
        "high": 2341,
        "medium": 5678,
        "low": 6692
    },
    "by_category": {
        "drug_safety": 8432,
        "diagnostic": 2341,
        "preventive_care": 3456,
        "guideline_adherence": 1005
    },
    "override_rate": 0.23,
    "override_reasons": [
        {"reason": "patient_aware", "count": 1234},
        {"reason": "no_alternative", "count": 567}
    ]
}
```

---

## 8. Error Handling

### 8.1 Error Response Format

```json
{
    "error": {
        "code": "INSUFFICIENT_PATIENT_DATA",
        "message": "Cannot evaluate drug interaction without current medication list",
        "details": {
            "missing_fields": ["current_medications"],
            "minimum_required": "At least one current medication"
        },
        "request_id": "req-12345-abcde"
    }
}
```

### 8.2 Error Codes

| HTTP Code | Error Code | Description |
|-----------|------------|-------------|
| 400 | INVALID_INPUT | Malformed request data |
| 400 | INSUFFICIENT_PATIENT_DATA | Missing required patient data |
| 401 | UNAUTHORIZED | Authentication failed |
| 403 | FORBIDDEN | Insufficient permissions |
| 404 | MEDICATION_NOT_FOUND | RxNorm code not recognized |
| 429 | RATE_LIMITED | Rate limit exceeded |
| 500 | INTERNAL_ERROR | Server error |
| 503 | SERVICE_UNAVAILABLE | CDS service temporarily down |

---

## 9. SDK Examples

### 9.1 JavaScript/TypeScript

```typescript
import { CDSClient } from '@wia/cds';

const client = new CDSClient({
    apiKey: 'your-api-key',
    baseUrl: 'https://api.provider.com/wia/cds/v1/'
});

// Check drug interactions
const interactions = await client.checkInteractions({
    patientId: 'patient-123',
    currentMedications: [
        { name: 'Simvastatin', rxnormCode: '197361' }
    ],
    newMedication: { name: 'Clarithromycin', rxnormCode: '197517' }
});

if (interactions.interactionsFound > 0) {
    console.log('Warning:', interactions.interactions[0].message);
}
```

### 9.2 Python

```python
from wia_cds import CDSClient

client = CDSClient(
    api_key='your-api-key',
    base_url='https://api.provider.com/wia/cds/v1/'
)

# Check dose
result = client.check_dose(
    patient_context={
        'age': 72,
        'weight_kg': 65,
        'creatinine_clearance': 35
    },
    medication_order={
        'name': 'Metformin',
        'dose': 1000,
        'unit': 'mg',
        'frequency': 'twice daily'
    }
)

if result['dose_alert']:
    print(f"Alert: {result['alert']['message']}")
```

---

**홍익인간 (弘益人間)**: "Benefit all humanity"

The WIA Clinical Decision Support Standard belongs to humanity. Free forever.

---

**Copyright 2025 SmileStory Inc. / WIA**
MIT License
