# WIA-MED-009 API Specification
## AI Diagnosis System API v1.0.0

---

## 1. Overview

This document specifies the standard API for AI diagnosis systems compliant with WIA-MED-009.

### 1.1 Design Principles

- **RESTful** — HTTP/HTTPS, JSON format
- **Stateless** — Each request is independent
- **Versioned** — `/api/v1/...` for backwards compatibility
- **Secure** — TLS 1.2+, OAuth 2.0 authentication
- **Documented** — OpenAPI/Swagger specification

---

## 2. Authentication

### 2.1 OAuth 2.0

```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials&
client_id=your_client_id&
client_secret=your_client_secret

Response:
{
  "access_token": "eyJhbGciOiJIUzI1NiIs...",
  "token_type": "Bearer",
  "expires_in": 3600
}
```

### 2.2 Usage

```http
GET /api/v1/diagnosis/{id}
Authorization: Bearer eyJhbGciOiJIUzI1NiIs...
```

---

## 3. Core Endpoints

### 3.1 Submit Diagnosis Request

**Endpoint:** `POST /api/v1/diagnosis`

**Request:**

```json
{
  "patient_id": "P12345",
  "study_id": "S67890",
  "modality": "X-ray",
  "body_part": "chest",
  "image": {
    "format": "DICOM",
    "data": "base64_encoded_image_data",
    "metadata": {
      "acquisition_date": "2025-01-18T10:30:00Z",
      "device": "Siemens AXIOM",
      "kvp": 120,
      "mas": 5
    }
  },
  "clinical_context": {
    "symptoms": ["cough", "fever"],
    "duration_days": 3,
    "temperature_celsius": 38.5
  }
}
```

**Response (202 Accepted):**

```json
{
  "diagnosis_id": "D98765",
  "status": "processing",
  "estimated_completion": "2025-01-18T10:30:05Z"
}
```

### 3.2 Get Diagnosis Result

**Endpoint:** `GET /api/v1/diagnosis/{diagnosis_id}`

**Response (200 OK):**

```json
{
  "diagnosis_id": "D98765",
  "status": "completed",
  "completed_at": "2025-01-18T10:30:03Z",
  "result": {
    "primary_diagnosis": {
      "code": "J18.9",
      "name": "Pneumonia, unspecified organism",
      "confidence": 0.92,
      "severity": "moderate"
    },
    "differential_diagnoses": [
      {
        "code": "J20.9",
        "name": "Acute bronchitis",
        "confidence": 0.15
      },
      {
        "code": "J12.9",
        "name": "Viral pneumonia",
        "confidence": 0.08
      }
    ],
    "findings": [
      {
        "location": "left_lower_lobe",
        "finding": "infiltrate",
        "confidence": 0.94,
        "bbox": {"x": 120, "y": 200, "width": 80, "height": 60}
      }
    ],
    "explanation": {
      "method": "Grad-CAM",
      "heatmap": "base64_encoded_heatmap",
      "key_features": [
        "Left lower lobe opacity",
        "Air bronchograms present",
        "Blunted costophrenic angle"
      ],
      "reasoning": "Model identified consolidation pattern consistent with pneumonia in left lower lobe."
    },
    "recommendations": [
      "Consider antibiotic therapy",
      "Follow-up chest X-ray in 1 week",
      "Check sputum culture"
    ],
    "metadata": {
      "model_version": "pneumonia-v2.1.0",
      "processing_time_ms": 2850,
      "wia_certified": "Gold"
    }
  }
}
```

### 3.3 Batch Processing

**Endpoint:** `POST /api/v1/diagnosis/batch`

**Request:**

```json
{
  "diagnoses": [
    {...},  // Diagnosis request 1
    {...}   // Diagnosis request 2
  ]
}
```

**Response:**

```json
{
  "batch_id": "B12345",
  "total": 2,
  "diagnosis_ids": ["D98765", "D98766"]
}
```

---

## 4. Feedback and Learning

### 4.1 Submit Feedback

**Endpoint:** `POST /api/v1/diagnosis/{diagnosis_id}/feedback`

**Request:**

```json
{
  "physician_diagnosis": {
    "code": "J18.1",
    "name": "Lobar pneumonia",
    "agreement": "partial",
    "notes": "Correct pneumonia diagnosis, but specific type differs"
  },
  "feedback_type": "correction",
  "rating": 4
}
```

**Response:**

```json
{
  "feedback_id": "F11111",
  "status": "received",
  "message": "Thank you for your feedback. It will help improve the model."
}
```

---

## 5. Model Information

### 5.1 Get Model Metadata

**Endpoint:** `GET /api/v1/models/{model_id}`

**Response:**

```json
{
  "model_id": "pneumonia-v2.1.0",
  "name": "Pneumonia Detection Model",
  "version": "2.1.0",
  "release_date": "2025-01-01",
  "performance": {
    "sensitivity": 0.935,
    "specificity": 0.911,
    "auc_roc": 0.964,
    "test_set_size": 5000
  },
  "training_data": {
    "size": 50000,
    "sources": ["Hospital A", "Hospital B", "Public Dataset X"],
    "demographics": {
      "age_range": [0, 95],
      "gender_distribution": {"male": 0.52, "female": 0.48}
    }
  },
  "certifications": [
    {
      "organization": "WIA",
      "level": "Gold",
      "valid_until": "2026-01-01"
    },
    {
      "organization": "FDA",
      "clearance": "510(k)",
      "number": "K253456"
    }
  ]
}
```

---

## 6. Error Handling

### 6.1 Error Response Format

```json
{
  "error": {
    "code": "INVALID_IMAGE_FORMAT",
    "message": "Image must be in DICOM or JPEG format",
    "details": {
      "received_format": "PNG",
      "supported_formats": ["DICOM", "JPEG"]
    },
    "request_id": "req_abc123"
  }
}
```

### 6.2 Error Codes

| HTTP Status | Error Code | Description |
|------------|-----------|-------------|
| 400 | INVALID_REQUEST | Malformed request body |
| 400 | INVALID_IMAGE_FORMAT | Unsupported image format |
| 401 | UNAUTHORIZED | Missing or invalid token |
| 403 | FORBIDDEN | Insufficient permissions |
| 404 | NOT_FOUND | Resource not found |
| 429 | RATE_LIMIT_EXCEEDED | Too many requests |
| 500 | INTERNAL_ERROR | Server error |
| 503 | SERVICE_UNAVAILABLE | System maintenance |

---

## 7. Rate Limiting

### 7.1 Limits

- **Standard:** 100 requests/hour
- **Premium:** 1,000 requests/hour
- **Enterprise:** Custom limits

### 7.2 Headers

```http
X-RateLimit-Limit: 100
X-RateLimit-Remaining: 95
X-RateLimit-Reset: 1642550400
```

---

## 8. Webhooks (Optional)

### 8.1 Register Webhook

**Endpoint:** `POST /api/v1/webhooks`

**Request:**

```json
{
  "url": "https://your-server.com/webhooks/diagnosis",
  "events": ["diagnosis.completed", "diagnosis.failed"]
}
```

### 8.2 Webhook Payload

```json
{
  "event": "diagnosis.completed",
  "timestamp": "2025-01-18T10:30:03Z",
  "data": {
    "diagnosis_id": "D98765",
    "patient_id": "P12345",
    "result": {...}
  }
}
```

---

## 9. SDKs and Client Libraries

### 9.1 Python

```python
from wia_diagnosis import DiagnosisClient

client = DiagnosisClient(
    api_key="your_api_key",
    base_url="https://api.wia-diagnosis.com"
)

# Submit diagnosis
result = client.diagnose(
    patient_id="P12345",
    image_path="chest_xray.dcm",
    modality="X-ray",
    body_part="chest"
)

print(f"Diagnosis: {result.primary_diagnosis.name}")
print(f"Confidence: {result.primary_diagnosis.confidence:.2%}")
```

### 9.2 JavaScript/TypeScript

```typescript
import { DiagnosisClient } from '@wia/diagnosis';

const client = new DiagnosisClient({
  apiKey: 'your_api_key',
  baseUrl: 'https://api.wia-diagnosis.com'
});

const result = await client.diagnose({
  patientId: 'P12345',
  image: imageBuffer,
  modality: 'X-ray',
  bodyPart: 'chest'
});

console.log(`Diagnosis: ${result.primaryDiagnosis.name}`);
```

---

## 10. FHIR Integration

### 10.1 FHIR DiagnosticReport

```json
{
  "resourceType": "DiagnosticReport",
  "id": "D98765",
  "status": "final",
  "category": [{
    "coding": [{
      "system": "http://terminology.hl7.org/CodeSystem/v2-0074",
      "code": "RAD"
    }]
  }],
  "code": {
    "coding": [{
      "system": "http://loinc.org",
      "code": "30746-2",
      "display": "Chest X-ray"
    }]
  },
  "subject": {
    "reference": "Patient/P12345"
  },
  "conclusionCode": [{
    "coding": [{
      "system": "http://hl7.org/fhir/sid/icd-10",
      "code": "J18.9",
      "display": "Pneumonia, unspecified organism"
    }]
  }],
  "extension": [{
    "url": "http://wiastandards.com/fhir/StructureDefinition/ai-confidence",
    "valueDecimal": 0.92
  }]
}
```

---

## Contact

**WIA API Support**
Email: api-support@wiastandards.com
Documentation: https://docs.wiastandards.com
GitHub: https://github.com/WIA-Official/wia-diagnosis-sdk

**Philosophy:** 弘益人間 (Benefit All Humanity)

**License:** MIT License
**Copyright:** © 2025 World Certification Industry Association
