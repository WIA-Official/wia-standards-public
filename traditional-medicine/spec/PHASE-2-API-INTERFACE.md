# WIA-TRADITIONAL-MEDICINE: Phase 2 - API Interface

## 弘益人間 (Benefit All Humanity)

**Version:** 1.0.0
**Status:** Official
**Last Updated:** 2025-01

---

## 1. Overview

Phase 2 defines RESTful APIs for integrating traditional medicine data into modern healthcare systems. The API follows OpenAPI 3.0 specification and provides endpoints for constitution assessment, diagnosis, herbal medicine queries, and safety checking.

### 1.1 Base URL

```
Production: https://api.wia.live/tm/v1
Staging: https://api-staging.wia.live/tm/v1
```

### 1.2 Authentication

All API requests require OAuth 2.0 Bearer token authentication:

```http
Authorization: Bearer <access_token>
```

---

## 2. Constitution Assessment API

### 2.1 Submit Constitution Assessment

```yaml
POST /constitution/assess
Content-Type: application/json

Request:
{
  "patient_id": "PAT-2025-001",
  "system": "tcm|ayurveda|sasang",
  "questionnaire_type": "CCMQ|prakriti|QSCCII",
  "responses": {
    "q1": 4,
    "q2": 3,
    ...
  },
  "include_recommendations": true
}

Response: 200 OK
{
  "profile_id": "PROF-2025-12345",
  "timestamp": "2025-01-15T09:30:00Z",
  "primary_constitution": "phlegm_dampness",
  "secondary_constitutions": ["qi_deficiency"],
  "scores": {
    "phlegm_dampness": 0.72,
    "qi_deficiency": 0.45,
    "yin_deficiency": 0.25,
    ...
  },
  "confidence": 0.85,
  "recommendations": {
    "diet": [
      "Reduce greasy and sweet foods",
      "Increase light, bland foods",
      "Avoid cold and raw foods"
    ],
    "lifestyle": [
      "Regular aerobic exercise",
      "Avoid humid environments"
    ],
    "herbs_to_consider": ["茯苓", "白朮", "陳皮"]
  }
}
```

### 2.2 Get Constitution Profile

```yaml
GET /constitution/profile/{patient_id}

Response: 200 OK
{
  "profile_id": "PROF-2025-12345",
  "patient_id": "PAT-2025-001",
  "assessments": [
    {
      "timestamp": "2025-01-15T09:30:00Z",
      "system": "tcm",
      "primary_constitution": "phlegm_dampness",
      "confidence": 0.85
    }
  ],
  "genomic_correlates": {
    "available": true,
    "last_updated": "2024-12-01"
  }
}
```

---

## 3. Diagnosis API

### 3.1 Submit Traditional Diagnosis

```yaml
POST /diagnosis/traditional
Content-Type: application/json

Request:
{
  "patient_id": "PAT-2025-001",
  "practitioner_id": "PRAC-001",
  "four_examinations": {
    "inspection": {
      "complexion": "pale_yellow",
      "tongue": {
        "body_color": "pale",
        "coating_color": "white",
        "coating_thickness": "thick"
      }
    },
    "inquiry": {
      "chief_complaint": "Fatigue, heavy sensation",
      "ten_questions": {
        "cold_heat": "cold_preference",
        "perspiration": "spontaneous_sweating"
      }
    },
    "palpation": {
      "pulse": {
        "rate": 68,
        "quality": ["slippery", "soft"]
      }
    }
  }
}

Response: 201 Created
{
  "diagnosis_id": "DIAG-2025-789",
  "pattern_diagnosis": {
    "icd11_tm_code": "TM1:SF32.2",
    "pattern_name": {
      "en": "Spleen Qi Deficiency with Dampness",
      "zh": "脾氣虛夾濕",
      "ko": "비기허협습"
    }
  },
  "confidence": 0.87
}
```

### 3.2 Tongue Image Analysis

```yaml
POST /diagnosis/tongue/analyze
Content-Type: multipart/form-data

Request:
  image: <tongue_photo.jpg>
  patient_id: PAT-2025-001

Response: 200 OK
{
  "analysis_id": "TONG-2025-456",
  "image_quality": {
    "score": 0.92,
    "issues": []
  },
  "findings": {
    "body_color": "pale",
    "body_shape": "teeth_marked",
    "coating": {
      "color": "white",
      "thickness": "thick",
      "distribution": "root_thicker"
    },
    "moisture": "wet",
    "special_features": ["teeth_marks"]
  },
  "patterns_suggested": [
    {
      "pattern": "spleen_qi_deficiency",
      "confidence": 0.89,
      "evidence": ["pale tongue", "teeth marks", "white coating"]
    },
    {
      "pattern": "dampness",
      "confidence": 0.82,
      "evidence": ["thick coating", "wet tongue"]
    }
  ],
  "icd11_tm_codes": ["TM1:SF32.2", "TM1:SJ10"]
}
```

---

## 4. Herbal Medicine API

### 4.1 Search Herbs

```yaml
GET /herbs/search?query=人參&property=tonify_qi&limit=10

Response: 200 OK
{
  "total": 15,
  "results": [
    {
      "herb_id": "HB-RenShen-001",
      "names": {
        "scientific": "Panax ginseng C.A.Mey.",
        "chinese": "人參",
        "pinyin": "Rén Shēn",
        "korean": "인삼"
      },
      "traditional_properties": {
        "taste": ["sweet", "slightly_bitter"],
        "temperature": "warm",
        "meridians": ["spleen", "lung", "heart"]
      },
      "actions": ["Tonify Yuan Qi", "Strengthen Spleen and Lung"]
    }
  ]
}
```

### 4.2 Get Formula Recommendations

```yaml
POST /herbs/formula/recommend

Request:
{
  "pattern": "spleen_qi_deficiency_with_dampness",
  "constitution": "phlegm_dampness",
  "current_medications": ["Metformin"],
  "allergies": [],
  "pregnancy_status": false
}

Response: 200 OK
{
  "recommendations": [
    {
      "formula": {
        "name_zh": "六君子湯",
        "name_pinyin": "Liu Jun Zi Tang",
        "name_en": "Six Gentlemen Decoction"
      },
      "match_score": 0.92,
      "composition": [
        {"herb": "人參", "pinyin": "Rén Shēn", "dose_g": 9},
        {"herb": "白朮", "pinyin": "Bái Zhú", "dose_g": 9},
        {"herb": "茯苓", "pinyin": "Fú Líng", "dose_g": 9},
        {"herb": "炙甘草", "pinyin": "Zhì Gān Cǎo", "dose_g": 6},
        {"herb": "陳皮", "pinyin": "Chén Pí", "dose_g": 6},
        {"herb": "半夏", "pinyin": "Bàn Xià", "dose_g": 6}
      ],
      "modifications_suggested": [
        "Add 薏苡仁 9g for stronger dampness drainage"
      ],
      "safety_check": {
        "status": "safe",
        "interactions": [],
        "warnings": []
      }
    }
  ]
}
```

### 4.3 Check Drug-Herb Interactions

```yaml
POST /herbs/interactions

Request:
{
  "herbs": ["人參", "當歸", "川芎"],
  "medications": ["Warfarin", "Aspirin"]
}

Response: 200 OK
{
  "interactions": [
    {
      "type": "herb_drug",
      "herb": "當歸",
      "drug": "Warfarin",
      "severity": "major",
      "effect": "May enhance anticoagulant effect",
      "mechanism": "Coumarin compounds inhibit vitamin K metabolism",
      "clinical_significance": "Increased bleeding risk",
      "recommendation": "Monitor INR closely if combination unavoidable; consider alternative herbs",
      "evidence_level": "moderate",
      "references": ["PMID: 12345678", "Natural Medicines Database"]
    },
    {
      "type": "herb_drug",
      "herb": "人參",
      "drug": "Warfarin",
      "severity": "moderate",
      "effect": "May reduce anticoagulant effect",
      "mechanism": "Possible vitamin K-like activity",
      "recommendation": "Monitor INR if combining"
    }
  ],
  "herb_herb_interactions": [],
  "overall_risk": "high",
  "summary": "Combination of 當歸 with anticoagulants poses significant bleeding risk.",
  "alternatives": [
    {
      "replace": "當歸",
      "with": "黃耆",
      "reason": "Lower bleeding risk while maintaining blood-nourishing effect"
    }
  ]
}
```

---

## 5. Safety Reporting API

### 5.1 Report Adverse Event

```yaml
POST /safety/report

Request:
{
  "patient_id": "PAT-2025-001",
  "reporter_id": "PRAC-001",
  "event_date": "2025-01-10",
  "products": [
    {
      "type": "herbal_formula",
      "name": "六君子湯",
      "dose": "9g three times daily",
      "duration_days": 7
    }
  ],
  "reaction": {
    "description": "Mild gastrointestinal discomfort",
    "symptoms": ["nausea", "bloating"],
    "onset": "2 days after starting",
    "outcome": "resolved",
    "severity": "mild"
  },
  "concomitant_medications": ["Metformin 500mg twice daily"]
}

Response: 201 Created
{
  "report_id": "AE-2025-001",
  "status": "submitted",
  "causality_assessment": "possible",
  "follow_up_required": false
}
```

---

## 6. Rate Limits and Quotas

| Tier | Requests/Hour | Concurrent Connections |
|------|---------------|------------------------|
| Free | 100 | 2 |
| Standard | 1,000 | 10 |
| Professional | 10,000 | 50 |
| Enterprise | Unlimited | Unlimited |

---

## 7. Error Responses

```json
{
  "error": {
    "code": "INVALID_PATTERN",
    "message": "Unknown pattern code provided",
    "details": {
      "field": "pattern",
      "value": "invalid_pattern_code"
    }
  }
}
```

| Code | HTTP Status | Description |
|------|-------------|-------------|
| AUTH_REQUIRED | 401 | Missing or invalid authentication |
| FORBIDDEN | 403 | Insufficient permissions |
| NOT_FOUND | 404 | Resource not found |
| VALIDATION_ERROR | 422 | Invalid request data |
| RATE_LIMIT | 429 | Rate limit exceeded |

---

## 8. SDK Support

### 8.1 TypeScript/JavaScript

```typescript
import { WIATraditionalMedicine } from '@wia/traditional-medicine-sdk';

const client = new WIATraditionalMedicine({
  apiKey: 'your-api-key'
});

const result = await client.constitution.assess({
  patientId: 'PAT-001',
  system: 'sasang',
  responses: { q1: 4, q2: 3 }
});
```

### 8.2 Python

```python
from wia_traditional_medicine import WIAClient

client = WIAClient(api_key='your-api-key')
result = client.constitution.assess(
    patient_id='PAT-001',
    system='sasang',
    responses={'q1': 4, 'q2': 3}
)
```

---

**弘益人間 - Benefit All Humanity**

© 2025 WIA - World Certification Industry Association
