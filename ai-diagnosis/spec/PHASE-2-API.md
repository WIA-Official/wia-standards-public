# WIA AI Diagnosis Standard - Phase 2: API Interface Specification

> **Version**: 1.0.0
> **Status**: Stable
> **Last Updated**: 2025
> **Standard**: WIA-MED-009

---

## 1. Overview

Phase 2 defines the RESTful API interface for AI diagnosis services. This specification ensures consistent interaction patterns across different AI diagnostic platforms while maintaining clinical safety and regulatory compliance.

### 1.1 Design Principles

- **RESTful**: Follow REST architectural constraints
- **Stateless**: Each request contains all necessary information
- **Versioned**: URL-based versioning for compatibility
- **Secure**: TLS encryption and HIPAA-compliant authentication
- **Auditable**: Complete request/response logging for clinical traceability

---

## 2. API Structure

### 2.1 Base URL

```
https://api.{provider}.com/wia/ai-diagnosis/v1/
```

### 2.2 Versioning

| Version | URL Path | Status |
|---------|----------|--------|
| v1 | /wia/ai-diagnosis/v1/ | Current (Stable) |
| v1-beta | /wia/ai-diagnosis/v1-beta/ | Beta features |
| v2 | /wia/ai-diagnosis/v2/ | Planned |

### 2.3 Common Headers

#### Request Headers

```http
Authorization: Bearer {access_token}
Content-Type: application/json
Accept: application/json
X-WIA-Version: 1.0.0
X-Request-ID: {uuid}
X-Patient-ID: {encrypted_patient_id}
X-Institution-ID: {institution_id}
```

#### Response Headers

```http
Content-Type: application/json
X-WIA-Version: 1.0.0
X-Request-ID: {uuid}
X-Processing-Time-Ms: 1250
X-Model-Version: v2.3.1
X-Clinical-Validation: passed
```

---

## 3. Authentication

### 3.1 Supported Methods

| Method | Use Case | Header |
|--------|----------|--------|
| Bearer Token | EHR integration | Authorization: Bearer {token} |
| OAuth 2.0 | User delegation | Authorization: Bearer {oauth_token} |
| Mutual TLS | Hospital-to-hospital | Client certificate |

### 3.2 Token Structure (JWT)

```json
{
    "header": {
        "alg": "RS256",
        "typ": "JWT"
    },
    "payload": {
        "sub": "physician-id-12345",
        "iss": "https://auth.hospital.com",
        "aud": "wia-ai-diagnosis-api",
        "iat": 1705329000,
        "exp": 1705332600,
        "scope": ["diagnose:read", "diagnose:write"],
        "license": {
            "type": "physician",
            "number": "MD-12345-KR",
            "specialties": ["cardiology", "internal_medicine"]
        },
        "institution": "hospital-xyz"
    }
}
```

---

## 4. Diagnostic Endpoints

### 4.1 Generate Diagnosis

**Endpoint**: `POST /wia/ai-diagnosis/v1/diagnose`

**Request**:
```json
{
    "patient_context": {
        "patient_id": "encrypted-patient-xyz",
        "age_range": "40-64",
        "sex": "female",
        "medical_history": {
            "comorbidities": [
                {
                    "condition": "E11",
                    "condition_name": "Type 2 Diabetes Mellitus",
                    "onset_date": "2020-03-15"
                }
            ],
            "medications": [
                {
                    "name": "Metformin",
                    "dosage": "500mg",
                    "frequency": "twice daily"
                }
            ]
        },
        "current_symptoms": [
            {
                "symptom": "dyspnea_on_exertion",
                "onset": "2_weeks_ago",
                "severity": "moderate"
            },
            {
                "symptom": "peripheral_edema",
                "location": "bilateral_lower_extremities",
                "severity": "mild"
            }
        ],
        "vital_signs": {
            "systolic_bp": 145,
            "diastolic_bp": 92,
            "heart_rate": 88,
            "oxygen_saturation": 97
        }
    },
    "clinical_data": {
        "laboratory_results": [
            {
                "test_name": "BNP",
                "loinc_code": "33762-6",
                "value": 450,
                "unit": "pg/mL",
                "abnormal_flag": "high"
            }
        ],
        "imaging": [
            {
                "modality": "x_ray",
                "body_region": "chest",
                "image_url": "https://storage.hospital.com/images/abc123",
                "findings_text": "Cardiomegaly noted"
            }
        ]
    },
    "options": {
        "return_differential": true,
        "max_differential_count": 5,
        "include_explainability": true,
        "confidence_threshold": 0.5,
        "specialty_focus": "cardiology"
    }
}
```

**Response**:
```json
{
    "event_id": "550e8400-e29b-41d4-a716-446655440000",
    "timestamp": "2025-01-15T14:30:00.000Z",
    "processing_time_ms": 1250,
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
                    "type": "lab_result",
                    "description": "Elevated BNP (450 pg/mL)",
                    "weight": 0.92
                },
                {
                    "type": "imaging_finding",
                    "description": "Cardiomegaly on chest X-ray",
                    "weight": 0.85
                },
                {
                    "type": "symptom",
                    "description": "Dyspnea on exertion",
                    "weight": 0.78
                }
            ]
        },
        {
            "condition": "I11.0",
            "condition_name": "Hypertensive Heart Disease with Heart Failure",
            "confidence": 0.72,
            "severity": "moderate",
            "differential_rank": 2
        }
    ],
    "recommendations": [
        {
            "type": "specialist_referral",
            "description": "Cardiology consultation for heart failure management",
            "priority": "high",
            "timeframe": "48 hours"
        },
        {
            "type": "further_testing",
            "description": "Echocardiogram to assess ejection fraction",
            "priority": "high",
            "timeframe": "1 week"
        }
    ],
    "explainability": {
        "method": "SHAP",
        "feature_importance": [
            {"feature": "BNP level", "importance": 0.35},
            {"feature": "Cardiomegaly", "importance": 0.28},
            {"feature": "Dyspnea on exertion", "importance": 0.22}
        ],
        "natural_language_explanation": "The diagnosis of heart failure is primarily supported by elevated BNP levels (450 pg/mL) and cardiomegaly visible on chest imaging."
    },
    "metadata": {
        "ai_system": {
            "provider": "WIA-Certified-AI-Diagnostics",
            "model_name": "CardioAI-DX",
            "model_version": "v2.3.1",
            "wia_certification": "gold"
        },
        "performance_metrics": {
            "sensitivity": 0.91,
            "specificity": 0.88,
            "auc_roc": 0.94
        }
    }
}
```

### 4.2 Analyze Medical Image

**Endpoint**: `POST /wia/ai-diagnosis/v1/analyze/image`

**Request**:
```json
{
    "patient_context": {
        "patient_id": "encrypted-patient-xyz",
        "age_range": "40-64",
        "sex": "female"
    },
    "image": {
        "modality": "x_ray",
        "body_region": "chest",
        "view": "PA",
        "content": "{base64_encoded_image}",
        "format": "dicom"
    },
    "options": {
        "detect_abnormalities": true,
        "generate_report": true,
        "include_attention_map": true,
        "comparison_image": null
    }
}
```

**Response**:
```json
{
    "analysis_id": "img-analysis-12345",
    "timestamp": "2025-01-15T14:30:00.000Z",
    "findings": [
        {
            "finding": "cardiomegaly",
            "confidence": 0.89,
            "location": {
                "region": "cardiac_silhouette",
                "coordinates": {"x": 512, "y": 384}
            },
            "severity": "moderate",
            "clinical_significance": "high"
        },
        {
            "finding": "pulmonary_edema",
            "confidence": 0.76,
            "location": {
                "region": "bilateral_lung_fields"
            },
            "severity": "mild"
        }
    ],
    "measurements": {
        "cardiothoracic_ratio": 0.58,
        "reference_range": {"min": 0.0, "max": 0.50}
    },
    "report": "FINDINGS: The cardiac silhouette is enlarged (CTR 0.58). There is evidence of mild pulmonary vascular congestion. No pleural effusion is identified.\n\nIMPRESSION: Cardiomegaly with signs of early pulmonary edema.",
    "attention_map_url": "https://storage.provider.com/attention-maps/abc123.png",
    "quality_assessment": {
        "image_quality": "good",
        "diagnostic_confidence": 0.92
    }
}
```

### 4.3 Analyze Lab Results

**Endpoint**: `POST /wia/ai-diagnosis/v1/analyze/labs`

**Request**:
```json
{
    "patient_context": {
        "patient_id": "encrypted-patient-xyz",
        "age_range": "40-64",
        "sex": "female",
        "medical_history": {
            "comorbidities": ["E11"]
        }
    },
    "laboratory_results": [
        {
            "test_name": "Hemoglobin A1c",
            "loinc_code": "4548-4",
            "value": 8.2,
            "unit": "%"
        },
        {
            "test_name": "Fasting Glucose",
            "loinc_code": "1558-6",
            "value": 165,
            "unit": "mg/dL"
        },
        {
            "test_name": "Creatinine",
            "loinc_code": "2160-0",
            "value": 1.4,
            "unit": "mg/dL"
        }
    ]
}
```

**Response**:
```json
{
    "analysis_id": "lab-analysis-67890",
    "timestamp": "2025-01-15T14:30:00.000Z",
    "interpretation": {
        "primary_findings": [
            {
                "finding": "poorly_controlled_diabetes",
                "confidence": 0.95,
                "evidence": ["HbA1c 8.2%", "Fasting glucose 165 mg/dL"]
            },
            {
                "finding": "mild_renal_impairment",
                "confidence": 0.82,
                "evidence": ["Creatinine 1.4 mg/dL"]
            }
        ],
        "abnormal_results": [
            {
                "test_name": "Hemoglobin A1c",
                "value": 8.2,
                "reference_range": {"max": 5.7},
                "flag": "high",
                "clinical_significance": "diabetes control inadequate"
            }
        ],
        "clinical_recommendations": [
            {
                "action": "Adjust diabetes medications",
                "rationale": "HbA1c above target of 7.0%"
            },
            {
                "action": "Monitor renal function",
                "rationale": "Elevated creatinine suggests early diabetic nephropathy"
            }
        ]
    }
}
```

---

## 5. Multi-Modal Diagnosis

### 5.1 Comprehensive Analysis

**Endpoint**: `POST /wia/ai-diagnosis/v1/analyze/comprehensive`

**Request**:
```json
{
    "patient_context": {...},
    "clinical_data": {
        "symptoms": [...],
        "vital_signs": {...},
        "laboratory_results": [...],
        "imaging": [...]
    },
    "options": {
        "fusion_strategy": "late_fusion",
        "specialty": "internal_medicine"
    }
}
```

**Response**: Returns integrated diagnosis combining all data modalities with weighted evidence from each source.

---

## 6. Batch Processing

### 6.1 Submit Batch Job

**Endpoint**: `POST /wia/ai-diagnosis/v1/batch`

**Request**:
```json
{
    "job_name": "weekly_screening_cohort",
    "patients": [
        {
            "patient_id": "patient-001",
            "data": {...}
        },
        {
            "patient_id": "patient-002",
            "data": {...}
        }
    ],
    "callback_url": "https://your-hospital.com/webhook/batch-results"
}
```

**Response**:
```json
{
    "job_id": "batch-job-12345",
    "status": "processing",
    "total_patients": 50,
    "estimated_completion_time": "2025-01-15T16:00:00.000Z",
    "status_url": "/wia/ai-diagnosis/v1/batch/batch-job-12345/status"
}
```

### 6.2 Check Batch Status

**Endpoint**: `GET /wia/ai-diagnosis/v1/batch/{job_id}/status`

**Response**:
```json
{
    "job_id": "batch-job-12345",
    "status": "completed",
    "progress": {
        "total": 50,
        "processed": 50,
        "failed": 2
    },
    "results_url": "/wia/ai-diagnosis/v1/batch/batch-job-12345/results"
}
```

---

## 7. Clinical Decision Support

### 7.1 Get Treatment Recommendations

**Endpoint**: `POST /wia/ai-diagnosis/v1/recommend/treatment`

**Request**:
```json
{
    "diagnosis": {
        "condition": "I50.9",
        "severity": "moderate"
    },
    "patient_context": {
        "age_range": "40-64",
        "comorbidities": ["E11", "I10"],
        "current_medications": ["Metformin"],
        "allergies": ["Penicillin"]
    }
}
```

**Response**:
```json
{
    "recommendations": [
        {
            "treatment_type": "medication",
            "drug_name": "Furosemide",
            "dosage": "20mg daily",
            "evidence_level": "A",
            "guideline_source": "ACC/AHA Heart Failure Guidelines 2022"
        },
        {
            "treatment_type": "lifestyle",
            "recommendation": "Sodium restriction < 2g/day",
            "evidence_level": "A"
        }
    ],
    "contraindications": [],
    "monitoring_parameters": [
        "Serum potassium",
        "Renal function"
    ]
}
```

---

## 8. Error Handling

### 8.1 Error Response Format

```json
{
    "error": {
        "code": "INSUFFICIENT_DATA",
        "message": "Cannot generate diagnosis with provided information",
        "details": {
            "missing_fields": ["vital_signs", "laboratory_results"],
            "minimum_required": "At least 3 clinical data points required"
        },
        "request_id": "req-12345-abcde",
        "documentation_url": "https://docs.wiastandards.com/errors/INSUFFICIENT_DATA"
    }
}
```

### 8.2 Error Codes

| HTTP Code | Error Code | Description |
|-----------|------------|-------------|
| 400 | INVALID_INPUT | Malformed request data |
| 400 | INSUFFICIENT_DATA | Not enough clinical data for diagnosis |
| 401 | UNAUTHORIZED | Authentication failed |
| 403 | FORBIDDEN | Insufficient permissions |
| 403 | LICENSE_EXPIRED | Physician license verification failed |
| 404 | NOT_FOUND | Resource not found |
| 422 | UNPROCESSABLE | Cannot process clinical data |
| 429 | RATE_LIMITED | Rate limit exceeded |
| 500 | INTERNAL_ERROR | Server error |
| 503 | SERVICE_UNAVAILABLE | Service temporarily down |
| 503 | MODEL_UNAVAILABLE | AI model unavailable |

### 8.3 Clinical Safety Errors

```json
{
    "error": {
        "code": "CLINICAL_SAFETY_ALERT",
        "message": "Diagnosis confidence below clinical threshold",
        "severity": "warning",
        "details": {
            "confidence": 0.42,
            "threshold": 0.50,
            "recommendation": "Seek additional clinical data or expert consultation"
        }
    }
}
```

---

## 9. Rate Limiting

### 9.1 Rate Limit Headers

```http
X-Rate-Limit-Limit: 100
X-Rate-Limit-Remaining: 95
X-Rate-Limit-Reset: 1705329600
Retry-After: 60
```

### 9.2 Rate Limits by Plan

| Plan | Requests/min | Requests/day | Patients/month |
|------|--------------|--------------|----------------|
| Research | 10 | 1,000 | 100 |
| Clinic | 60 | 10,000 | 1,000 |
| Hospital | 300 | 100,000 | 10,000 |
| Enterprise | Unlimited | Unlimited | Unlimited |

---

## 10. Webhooks

### 10.1 Register Webhook

**Endpoint**: `POST /wia/ai-diagnosis/v1/webhooks`

**Request**:
```json
{
    "url": "https://your-hospital.com/webhook",
    "events": ["diagnosis.completed", "diagnosis.failed", "critical_finding.detected"],
    "secret": "webhook-secret-key"
}
```

### 10.2 Webhook Payload

```json
{
    "event": "critical_finding.detected",
    "timestamp": "2025-01-15T14:30:00.000Z",
    "data": {
        "event_id": "550e8400-e29b-41d4-a716-446655440000",
        "patient_id": "encrypted-patient-xyz",
        "finding": {
            "condition": "I21.0",
            "condition_name": "ST-Elevation Myocardial Infarction",
            "urgency": "life_threatening",
            "confidence": 0.94
        }
    },
    "signature": "sha256=..."
}
```

---

## 11. SDK Support

### 11.1 JavaScript/TypeScript

```typescript
import { AIDetectionClient } from '@wia/ai-diagnosis';

const client = new AIDetectionClient({
    apiKey: 'your-api-key',
    baseUrl: 'https://api.provider.com/wia/ai-diagnosis/v1/'
});

const diagnosis = await client.diagnose({
    patientContext: {...},
    clinicalData: {...},
    options: {
        returnDifferential: true,
        includeExplainability: true
    }
});

console.log('Primary diagnosis:', diagnosis.diagnosis[0].condition_name);
```

### 11.2 Python

```python
from wia_ai_diagnosis import DiagnosisClient

client = DiagnosisClient(
    api_key='your-api-key',
    base_url='https://api.provider.com/wia/ai-diagnosis/v1/'
)

diagnosis = client.diagnose(
    patient_context={...},
    clinical_data={...},
    options={'return_differential': True}
)

print(f"Primary diagnosis: {diagnosis['diagnosis'][0]['condition_name']}")
```

---

## 12. OpenAPI Specification

The complete OpenAPI 3.0 specification is available at:
- YAML: `https://api.{provider}.com/wia/ai-diagnosis/v1/openapi.yaml`
- JSON: `https://api.{provider}.com/wia/ai-diagnosis/v1/openapi.json`

---

**홍익인간 (弘益人間)**: "Benefit all humanity"

The WIA AI Diagnosis Standard belongs to humanity. Free forever.

---

**Copyright 2025 SmileStory Inc. / WIA**
MIT License
