# WIA-MENTAL-HEALTH: Phase 2 - API Interface Specification
**Version:** 1.0
**Status:** Official
**Philosophy:** 弘益人間 (Benefit All Humanity)

## 1. Overview

This document specifies the REST API interfaces for mental health assessment, treatment management, and digital therapeutics platforms implementing the WIA-MENTAL-HEALTH standard.

### 1.1 Base URL

```
https://api.{provider}.com/wia/mental-health/v1
```

### 1.2 Authentication

All endpoints require OAuth 2.0 authentication with JWT tokens.

```http
Authorization: Bearer <access_token>
X-WIA-Standard: MENTAL-HEALTH-v1.0
X-Privacy-Level: PHI
```

## 2. Assessment API

### 2.1 Submit Assessment

**Endpoint:** `POST /assessments`

Submit a new mental health assessment.

**Request:**
```json
{
  "subject_id": "string (pseudonymized)",
  "assessment_type": "enum: screening|diagnostic|monitoring|outcome",
  "instruments": ["PHQ-9", "GAD-7"],
  "responses": {
    "PHQ-9": {
      "items": [
        { "item_id": 1, "response": 2 },
        { "item_id": 2, "response": 1 }
      ]
    },
    "GAD-7": {
      "items": [
        { "item_id": 1, "response": 1 }
      ]
    }
  },
  "context": {
    "setting": "enum: clinical|telehealth|self_report|research",
    "administered_by": "string (optional, provider ID)"
  }
}
```

**Response (201 Created):**
```json
{
  "assessment_id": "uuid",
  "timestamp": "2025-01-15T10:30:00Z",
  "mental_health_index": {
    "depression_score": {
      "value": 0.45,
      "severity": "moderate",
      "raw_score": 12,
      "instrument": "PHQ-9"
    },
    "anxiety_score": {
      "value": 0.28,
      "severity": "mild",
      "raw_score": 8,
      "instrument": "GAD-7"
    }
  },
  "clinical_flags": {
    "suicide_risk": false,
    "requires_immediate_attention": false
  },
  "next_assessment_recommended": "2025-01-29T10:30:00Z"
}
```

### 2.2 Get Assessment History

**Endpoint:** `GET /assessments/history/{subject_id}`

**Query Parameters:**
| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| start_date | ISO 8601 | No | Filter start date |
| end_date | ISO 8601 | No | Filter end date |
| instrument | string | No | Filter by instrument |
| limit | integer | No | Max results (default 50) |
| offset | integer | No | Pagination offset |

**Response (200 OK):**
```json
{
  "subject_id": "string",
  "assessments": [
    {
      "assessment_id": "uuid",
      "timestamp": "ISO 8601",
      "mental_health_index": { ... },
      "trend": "enum: improving|stable|declining"
    }
  ],
  "aggregate_trends": {
    "depression": {
      "30_day_change": -0.15,
      "trend": "improving"
    },
    "anxiety": {
      "30_day_change": -0.08,
      "trend": "stable"
    }
  },
  "pagination": {
    "total": 24,
    "limit": 50,
    "offset": 0
  }
}
```

### 2.3 Crisis Detection

**Endpoint:** `POST /assessments/crisis-screen`

Real-time crisis assessment with immediate routing.

**Request:**
```json
{
  "subject_id": "string",
  "responses": {
    "suicide_ideation": 0-3,
    "self_harm_urges": 0-3,
    "substance_intoxication": "boolean",
    "psychosis_symptoms": "boolean"
  },
  "context": {
    "caller_type": "enum: patient|family|provider",
    "current_location": "encrypted_location"
  }
}
```

**Response (200 OK):**
```json
{
  "crisis_level": "enum: none|low|moderate|high|imminent",
  "recommended_action": "enum: continue_care|safety_plan|crisis_line|emergency_services",
  "resources": [
    {
      "name": "988 Suicide & Crisis Lifeline",
      "phone": "988",
      "available": true
    }
  ],
  "safety_plan_active": true,
  "notification_sent_to": ["primary_provider", "emergency_contact"]
}
```

## 3. Treatment API

### 3.1 Get Treatment Recommendations

**Endpoint:** `POST /treatments/recommend`

AI-powered treatment recommendations based on assessment data.

**Request:**
```json
{
  "subject_id": "string",
  "assessment_id": "uuid (latest)",
  "preferences": {
    "modality_preferences": ["telehealth", "in_person"],
    "medication_openness": 0.0-1.0,
    "therapy_experience": "enum: none|some|extensive",
    "time_availability": "hours_per_week"
  },
  "constraints": {
    "insurance": "string",
    "location": "geo_region",
    "language": ["en", "es"]
  }
}
```

**Response (200 OK):**
```json
{
  "recommendations": [
    {
      "rank": 1,
      "protocol": {
        "category": "psychotherapy",
        "modality": "CBT",
        "format": "individual",
        "frequency": "weekly",
        "duration_weeks": 12
      },
      "evidence_level": "1A",
      "match_score": 0.92,
      "neuroplasticity_mechanism": "Prefrontal cortex strengthening, amygdala regulation",
      "expected_outcome": {
        "symptom_reduction": 0.40,
        "confidence_interval": [0.30, 0.50]
      }
    },
    {
      "rank": 2,
      "protocol": {
        "category": "digital",
        "modality": "CBT_app",
        "product_name": "Woebot",
        "regulatory_status": "FDA_cleared"
      },
      "evidence_level": "2A",
      "match_score": 0.85,
      "accessibility_score": 0.95
    }
  ],
  "combined_approach": {
    "primary": "CBT",
    "adjunct": ["mindfulness_app", "exercise_prescription"],
    "rationale": "Multi-modal approach for moderate depression"
  }
}
```

### 3.2 Track Treatment Progress

**Endpoint:** `GET /treatments/progress/{subject_id}`

**Response (200 OK):**
```json
{
  "subject_id": "string",
  "current_treatments": [
    {
      "protocol_id": "uuid",
      "modality": "CBT",
      "start_date": "2024-12-01",
      "sessions_completed": 6,
      "sessions_planned": 12,
      "adherence_rate": 0.92,
      "progress": {
        "baseline_depression": 0.65,
        "current_depression": 0.42,
        "improvement": 0.35,
        "on_track": true
      }
    }
  ],
  "milestones": [
    {
      "name": "50% symptom reduction",
      "target_date": "2025-02-15",
      "status": "on_track",
      "current_progress": 0.35
    }
  ],
  "alerts": []
}
```

### 3.3 Log Treatment Session

**Endpoint:** `POST /treatments/sessions`

**Request:**
```json
{
  "protocol_id": "uuid",
  "session_number": 7,
  "date": "2025-01-15T14:00:00Z",
  "duration_minutes": 50,
  "format": "telehealth",
  "interventions": [
    {
      "technique": "cognitive_restructuring",
      "duration_minutes": 20,
      "focus_area": "automatic_negative_thoughts"
    },
    {
      "technique": "behavioral_activation",
      "duration_minutes": 15,
      "homework_assigned": true
    }
  ],
  "patient_engagement": 0.85,
  "notes_encrypted": "base64_encrypted_string"
}
```

## 4. Biomarker API

### 4.1 Submit Biomarker Data

**Endpoint:** `POST /biomarkers`

**Request:**
```json
{
  "subject_id": "string",
  "collection_date": "2025-01-15T08:00:00Z",
  "source": "enum: laboratory|wearable|self_report",
  "biomarkers": {
    "cortisol": {
      "value": 15.2,
      "unit": "ng/mL",
      "time_of_day": "morning"
    },
    "heart_rate_variability": {
      "rmssd": 35.5,
      "unit": "ms"
    },
    "sleep_metrics": {
      "total_sleep_time": 420,
      "sleep_efficiency": 0.88
    }
  }
}
```

### 4.2 Get Biomarker Trends

**Endpoint:** `GET /biomarkers/trends/{subject_id}`

**Query Parameters:**
- `biomarker_type`: cortisol, hrv, sleep, inflammatory
- `period`: 7d, 30d, 90d, 1y

**Response (200 OK):**
```json
{
  "subject_id": "string",
  "period": "30d",
  "trends": {
    "cortisol": {
      "average": 14.8,
      "trend": "decreasing",
      "variability": 0.15,
      "within_normal": true
    },
    "heart_rate_variability": {
      "rmssd_average": 38.2,
      "trend": "increasing",
      "interpretation": "improving_autonomic_balance"
    }
  },
  "correlations": [
    {
      "biomarker_pair": ["hrv", "depression_score"],
      "correlation": -0.72,
      "significance": "p<0.01"
    }
  ]
}
```

## 5. Neuroimaging API

### 5.1 Submit Neuroimaging Data

**Endpoint:** `POST /neuroimaging`

**Request:**
```json
{
  "subject_id": "string",
  "scan_date": "2025-01-15T10:00:00Z",
  "scan_type": "fMRI",
  "protocol": "resting_state",
  "device_info": {
    "manufacturer": "Siemens",
    "model": "Prisma",
    "field_strength": 3.0
  },
  "processed_metrics": {
    "default_mode_network": {
      "connectivity_score": 0.72,
      "within_network_connectivity": 1.24
    },
    "amygdala": {
      "activity_level": 0.68,
      "prefrontal_connectivity": -0.45
    }
  },
  "raw_data_reference": "secure_storage_uri"
}
```

## 6. Digital Therapeutics API

### 6.1 Register Digital Therapeutic

**Endpoint:** `POST /digital-therapeutics/register`

**Request:**
```json
{
  "product_id": "string",
  "subject_id": "string",
  "prescription_id": "string (optional)",
  "regulatory_info": {
    "status": "FDA_cleared",
    "clearance_number": "K123456"
  },
  "configuration": {
    "personalization_level": "high",
    "notification_preferences": {
      "frequency": "daily",
      "quiet_hours": ["22:00", "07:00"]
    }
  }
}
```

### 6.2 Sync DTx Session Data

**Endpoint:** `POST /digital-therapeutics/sessions`

**Request:**
```json
{
  "product_id": "string",
  "subject_id": "string",
  "session": {
    "id": "uuid",
    "date": "2025-01-15T19:00:00Z",
    "duration_minutes": 15,
    "modules_completed": ["mood_check", "cbt_exercise_3"],
    "engagement_score": 0.88,
    "biometrics_synced": {
      "hrv_during_session": 42.5,
      "stress_level_change": -0.15
    }
  }
}
```

## 7. Psychedelic Therapy API

### 7.1 Register Psychedelic Protocol

**Endpoint:** `POST /psychedelic-therapy/protocols`

⚠️ **Restricted Access**: Requires verified clinical credentials and regulatory authorization.

**Request:**
```json
{
  "subject_id": "string",
  "protocol": {
    "substance": "psilocybin",
    "regulatory_context": "FDA_breakthrough_therapy",
    "clinical_trial_id": "NCT12345678"
  },
  "provider": {
    "id": "string",
    "credentials": ["MD", "psychiatry_board_certified"],
    "psychedelic_training": "MAPS_certified"
  },
  "screening": {
    "completed": true,
    "contraindications_cleared": true,
    "conditions_excluded": ["schizophrenia", "bipolar_I", "psychotic_history"]
  }
}
```

### 7.2 Log Dosing Session

**Endpoint:** `POST /psychedelic-therapy/sessions`

**Request:**
```json
{
  "protocol_id": "uuid",
  "session_type": "dosing",
  "date": "2025-01-15T09:00:00Z",
  "dose": {
    "amount": 25,
    "unit": "mg"
  },
  "setting": {
    "environment": "clinical",
    "music_protocol": "Johns_Hopkins_playlist",
    "support_team": 2
  },
  "monitoring": {
    "vital_signs": [
      { "time": "+0h", "bp": "120/80", "hr": 72 },
      { "time": "+2h", "bp": "130/85", "hr": 82 }
    ],
    "adverse_events": []
  },
  "experience_measures": {
    "mystical_experience_questionnaire": 0.78,
    "ego_dissolution_inventory": 0.65,
    "challenging_experience": 0.22
  }
}
```

## 8. Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| MH001 | 400 | Invalid assessment instrument |
| MH002 | 400 | Score out of valid range |
| MH003 | 401 | Authentication required |
| MH004 | 403 | Insufficient credentials for endpoint |
| MH005 | 404 | Subject not found |
| MH006 | 409 | Duplicate assessment within time window |
| MH007 | 422 | Missing required consent |
| MH008 | 429 | Rate limit exceeded |
| MH009 | 500 | Encryption/decryption failure |
| MH010 | 503 | Crisis service unavailable |

**Error Response Format:**
```json
{
  "error": {
    "code": "MH007",
    "message": "Valid consent record required for this data type",
    "details": {
      "required_consent_type": "treatment",
      "consent_status": "expired"
    }
  }
}
```

## 9. Rate Limiting

| Endpoint Category | Rate Limit | Window |
|-------------------|------------|--------|
| Assessments | 100 | per hour |
| Treatment Recommendations | 20 | per hour |
| Biomarkers | 500 | per hour |
| Crisis Screening | 1000 | per hour (priority) |
| Psychedelic Therapy | 10 | per hour |

## 10. Webhooks

### 10.1 Event Types

| Event | Description |
|-------|-------------|
| `assessment.completed` | Assessment submitted and scored |
| `crisis.detected` | Crisis screening triggered alert |
| `treatment.milestone` | Patient reached treatment milestone |
| `biomarker.alert` | Biomarker outside normal range |
| `session.completed` | Therapy session logged |

### 10.2 Webhook Payload

```json
{
  "event": "crisis.detected",
  "timestamp": "2025-01-15T14:32:00Z",
  "data": {
    "subject_id": "string (pseudonymized)",
    "crisis_level": "high",
    "action_taken": "emergency_contact_notified"
  },
  "signature": "HMAC-SHA256 signature"
}
```

---

**弘益人間 (Benefit All Humanity)**

*© 2025 WIA - World Certification Industry Association*
*MIT License*
