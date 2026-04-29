# WIA-MENTAL_WELLNESS: Phase 2 - API Interface Specification
**Version:** 1.0
**Status:** Official
**Philosophy:** 弘益人間 (Benefit All Humanity)

## 1. Overview

This document defines the API interfaces for mental wellness systems. All implementations MUST provide these endpoints to ensure interoperability with therapy platforms, wellness applications, and healthcare providers.

## 2. Base Configuration

### 2.1 API Base URL
```
Production:  https://api.wia-official.org/mental-wellness/v1
Staging:     https://staging-api.wia-official.org/mental-wellness/v1
Development: http://localhost:8080/mental-wellness/v1
```

### 2.2 Authentication
```http
Authorization: Bearer <JWT_TOKEN>
X-API-Key: <API_KEY>
X-Client-ID: <CLIENT_ID>
X-Request-ID: <UUID>
```

### 2.3 Headers
```http
Content-Type: application/json
Accept: application/json
X-WIA-Version: 1.0
X-Privacy-Level: high
X-Consent-ID: <UUID>
```

## 3. Mood Tracking API

### 3.1 Record Mood Entry

**Endpoint:** `POST /mood/record`

**Request:**
```json
{
  "mood": {
    "primary": "anxious",
    "intensity": 7,
    "valence": -6,
    "arousal": 8,
    "dominance": 3,
    "tags": ["work", "deadline"],
    "triggers": ["presentation"],
    "coping": ["breathing exercises"]
  },
  "context": {
    "location": "work",
    "activity": "preparing slides",
    "social": "alone"
  },
  "physiological": {
    "heartRate": 92,
    "hrv": 45,
    "respirationRate": 18
  }
}
```

**Response:** `201 Created`
```json
{
  "success": true,
  "moodId": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": "2026-01-12T10:30:00Z",
  "insights": {
    "pattern": "Anxiety levels elevated during work hours",
    "recommendation": "Consider scheduled breaks",
    "similarEvents": 12
  }
}
```

### 3.2 Get Mood History

**Endpoint:** `GET /mood/history`

**Query Parameters:**
- `startDate` (ISO 8601): Start of date range
- `endDate` (ISO 8601): End of date range
- `limit` (integer): Max results (default: 100)
- `offset` (integer): Pagination offset
- `moodType` (string): Filter by mood type

**Response:** `200 OK`
```json
{
  "success": true,
  "count": 45,
  "data": [
    {
      "moodId": "uuid",
      "timestamp": "2026-01-12T10:30:00Z",
      "mood": {
        "primary": "anxious",
        "intensity": 7
      },
      "context": {
        "location": "work"
      }
    }
  ],
  "pagination": {
    "limit": 100,
    "offset": 0,
    "total": 45,
    "hasMore": false
  }
}
```

### 3.3 Get Mood Analytics

**Endpoint:** `GET /mood/analytics`

**Query Parameters:**
- `period` (enum): day, week, month, year
- `timezone` (string): IANA timezone

**Response:** `200 OK`
```json
{
  "success": true,
  "period": "week",
  "analytics": {
    "moodDistribution": {
      "happy": 25,
      "calm": 30,
      "anxious": 20,
      "sad": 15,
      "neutral": 10
    },
    "averageIntensity": 6.2,
    "averageValence": -2.1,
    "trends": {
      "improving": false,
      "stable": true,
      "declining": false
    },
    "patterns": [
      {
        "pattern": "Anxiety peaks Monday mornings",
        "confidence": 0.85,
        "occurrences": 8
      }
    ],
    "recommendations": [
      "Consider mindfulness practices on Monday mornings",
      "Sleep quality impacts mood significantly"
    ]
  }
}
```

## 4. Assessment API

### 4.1 Start Assessment

**Endpoint:** `POST /assessment/start`

**Request:**
```json
{
  "assessmentType": "PHQ-9",
  "reason": "routine|symptoms|followup|screening",
  "triggeredBy": "self|clinician|algorithm"
}
```

**Response:** `201 Created`
```json
{
  "success": true,
  "assessmentId": "a1b2c3d4-e5f6-4a5b-8c9d-0e1f2a3b4c5d",
  "assessmentType": "PHQ-9",
  "questions": [
    {
      "id": "phq9_q1",
      "text": "Little interest or pleasure in doing things",
      "scale": "0-3",
      "options": [
        {"value": 0, "label": "Not at all"},
        {"value": 1, "label": "Several days"},
        {"value": 2, "label": "More than half the days"},
        {"value": 3, "label": "Nearly every day"}
      ]
    }
  ],
  "totalQuestions": 9,
  "estimatedTime": 300
}
```

### 4.2 Submit Assessment Response

**Endpoint:** `POST /assessment/{assessmentId}/response`

**Request:**
```json
{
  "questionId": "phq9_q1",
  "response": 2,
  "timeTaken": 15
}
```

**Response:** `200 OK`
```json
{
  "success": true,
  "questionNumber": 1,
  "totalQuestions": 9,
  "nextQuestion": {
    "id": "phq9_q2",
    "text": "Feeling down, depressed, or hopeless",
    "scale": "0-3"
  }
}
```

### 4.3 Complete Assessment

**Endpoint:** `POST /assessment/{assessmentId}/complete`

**Response:** `200 OK`
```json
{
  "success": true,
  "assessmentId": "uuid",
  "completedAt": "2026-01-12T10:45:00Z",
  "results": {
    "score": {
      "total": 15,
      "percentile": 72,
      "interpretation": "moderate",
      "riskLevel": "medium"
    },
    "subscales": {
      "anhedonia": 6,
      "depressedMood": 5,
      "sleepDisturbance": 4
    },
    "recommendations": [
      "Consider speaking with a mental health professional",
      "Regular exercise may help improve symptoms",
      "Maintain consistent sleep schedule"
    ],
    "clinicalNotes": "Scores indicate moderate depression. Recommend follow-up within 2 weeks."
  },
  "nextAssessment": {
    "recommended": "2026-01-26T10:00:00Z",
    "type": "PHQ-9"
  }
}
```

### 4.4 Get Assessment History

**Endpoint:** `GET /assessment/history`

**Response:** `200 OK`
```json
{
  "success": true,
  "assessments": [
    {
      "assessmentId": "uuid",
      "type": "PHQ-9",
      "completedAt": "2026-01-12T10:45:00Z",
      "score": 15,
      "interpretation": "moderate",
      "trend": "improving"
    }
  ],
  "longitudinalData": {
    "firstAssessment": "2025-06-01T00:00:00Z",
    "totalAssessments": 12,
    "averageScore": 14.2,
    "trend": "stable"
  }
}
```

## 5. Therapy Session API

### 5.1 Schedule Session

**Endpoint:** `POST /therapy/session/schedule`

**Request:**
```json
{
  "therapistId": "therapist_uuid",
  "sessionType": "individual",
  "modality": "CBT",
  "format": "video",
  "preferredTime": "2026-01-15T14:00:00Z",
  "duration": 50,
  "timezone": "America/New_York"
}
```

**Response:** `201 Created`
```json
{
  "success": true,
  "sessionId": "session_uuid",
  "scheduledTime": "2026-01-15T14:00:00Z",
  "therapist": {
    "id": "therapist_uuid",
    "name": "Dr. Jane Smith",
    "credentials": "PhD, Licensed Psychologist",
    "specialties": ["CBT", "anxiety", "depression"]
  },
  "meetingDetails": {
    "format": "video",
    "link": "https://session.wia-mental-wellness.org/join/session_uuid",
    "accessCode": "encrypted_code"
  },
  "reminders": [
    "2026-01-15T13:45:00Z",
    "2026-01-14T14:00:00Z"
  ]
}
```

### 5.2 Record Session Notes

**Endpoint:** `POST /therapy/session/{sessionId}/notes`

**Request:**
```json
{
  "subjective": "encrypted_patient_report",
  "objective": "encrypted_therapist_observations",
  "assessment": "encrypted_clinical_assessment",
  "plan": "encrypted_treatment_plan",
  "topics": ["anxiety management", "cognitive restructuring"],
  "techniques": ["thought records", "behavioral activation"],
  "homework": ["Daily mood tracking", "Practice relaxation exercises"],
  "goals": [
    {
      "id": "goal_1",
      "progress": 60,
      "status": "in-progress"
    }
  ],
  "outcomes": {
    "clientSatisfaction": 5,
    "therapeuticAlliance": 6,
    "symptomChange": 2
  }
}
```

**Response:** `201 Created`
```json
{
  "success": true,
  "notesId": "notes_uuid",
  "sessionId": "session_uuid",
  "savedAt": "2026-01-15T14:55:00Z",
  "nextSession": {
    "recommended": "2026-01-22T14:00:00Z",
    "focus": "Continue cognitive restructuring"
  }
}
```

### 5.3 Get Session History

**Endpoint:** `GET /therapy/session/history`

**Response:** `200 OK`
```json
{
  "success": true,
  "sessions": [
    {
      "sessionId": "uuid",
      "timestamp": "2026-01-15T14:00:00Z",
      "therapist": "Dr. Jane Smith",
      "modality": "CBT",
      "duration": 50,
      "sessionNumber": 8,
      "phase": "active"
    }
  ],
  "summary": {
    "totalSessions": 8,
    "startDate": "2025-11-01T00:00:00Z",
    "averageSatisfaction": 4.6,
    "goalsAchieved": 2,
    "goalsInProgress": 3
  }
}
```

## 6. Mindfulness & Meditation API

### 6.1 Start Meditation Session

**Endpoint:** `POST /mindfulness/session/start`

**Request:**
```json
{
  "type": "meditation",
  "technique": "mindfulness",
  "duration": 600,
  "guided": true,
  "instructor": "meditation_guide_1"
}
```

**Response:** `201 Created`
```json
{
  "success": true,
  "sessionId": "meditation_uuid",
  "startedAt": "2026-01-12T08:00:00Z",
  "audio": {
    "url": "https://media.wia-wellness.org/meditation/mindfulness-10min.mp3",
    "duration": 600,
    "format": "mp3"
  },
  "instructions": [
    "Find a comfortable seated position",
    "Close your eyes gently",
    "Focus on your breath"
  ]
}
```

### 6.2 Complete Meditation Session

**Endpoint:** `POST /mindfulness/session/{sessionId}/complete`

**Request:**
```json
{
  "actualDuration": 585,
  "experience": {
    "focusQuality": 7,
    "distractions": 3,
    "emotionalStateBefore": "anxious",
    "emotionalStateAfter": "calm",
    "insights": ["Noticed tension in shoulders", "Breath awareness improved"]
  },
  "biometrics": {
    "heartRateAverage": 68,
    "hrvAverage": 62,
    "respirationRate": 12
  }
}
```

**Response:** `200 OK`
```json
{
  "success": true,
  "sessionId": "meditation_uuid",
  "completedAt": "2026-01-12T08:10:00Z",
  "analysis": {
    "effectivenessScore": 8.5,
    "stressReduction": 35,
    "physiologicalChange": {
      "heartRateReduction": 12,
      "hrvIncrease": 18
    }
  },
  "streak": {
    "currentStreak": 7,
    "longestStreak": 15,
    "totalSessions": 42
  },
  "recommendations": [
    "Excellent focus today",
    "Try 15-minute sessions for deeper practice"
  ]
}
```

## 7. Crisis Support API

### 7.1 Initiate Crisis Contact

**Endpoint:** `POST /crisis/contact`

**Request:**
```json
{
  "severity": "high",
  "type": "suicidal",
  "immediate": true,
  "message": "encrypted_crisis_message",
  "location": {
    "latitude": 40.7128,
    "longitude": -74.0060,
    "consent": true
  }
}
```

**Response:** `201 Created` (Priority)
```json
{
  "success": true,
  "crisisId": "crisis_uuid",
  "priority": "immediate",
  "responseTime": 60,
  "resources": {
    "hotlines": [
      {
        "name": "National Suicide Prevention Lifeline",
        "number": "988",
        "available": "24/7"
      }
    ],
    "emergency": {
      "number": "911",
      "text": "Text 911 if calling is unsafe"
    },
    "chatSupport": {
      "url": "https://crisis.wia-wellness.org/chat",
      "estimatedWait": 30
    }
  },
  "counselorAssigned": {
    "id": "crisis_counselor_123",
    "name": "Anonymous Crisis Counselor",
    "eta": 45
  },
  "safetyCheck": {
    "immediate": true,
    "interval": 300
  }
}
```

### 7.2 Update Crisis Status

**Endpoint:** `PUT /crisis/{crisisId}/status`

**Request:**
```json
{
  "status": "stabilized|ongoing|escalated|resolved",
  "assessment": {
    "suicidalIdeation": "passive",
    "selfHarmRisk": "low",
    "support": "moderate"
  },
  "actions": ["created_safety_plan", "contacted_support_person"]
}
```

**Response:** `200 OK`
```json
{
  "success": true,
  "crisisId": "crisis_uuid",
  "status": "stabilized",
  "followUp": {
    "scheduled": "2026-01-13T10:00:00Z",
    "type": "check-in-call",
    "contacts": ["therapist", "support-person"]
  },
  "safetyPlan": {
    "active": true,
    "warningSignsIdentified": 5,
    "copingStrategies": 7,
    "supportContacts": 3
  }
}
```

## 8. Wellness Reports API

### 8.1 Generate Wellness Report

**Endpoint:** `POST /reports/generate`

**Request:**
```json
{
  "reportType": "comprehensive|mood-only|therapy-progress|crisis-summary",
  "dateRange": {
    "start": "2025-12-01T00:00:00Z",
    "end": "2026-01-01T00:00:00Z"
  },
  "includeRecommendations": true,
  "shareWith": ["therapist_uuid"]
}
```

**Response:** `201 Created`
```json
{
  "success": true,
  "reportId": "report_uuid",
  "generatedAt": "2026-01-12T11:00:00Z",
  "summary": {
    "overallWellness": 72,
    "moodStability": "moderate",
    "therapyProgress": "good",
    "riskLevel": "low"
  },
  "downloadUrl": "https://reports.wia-wellness.org/download/report_uuid",
  "expiresAt": "2026-01-19T11:00:00Z",
  "format": "pdf"
}
```

## 9. Error Responses

### 9.1 Error Format
```json
{
  "success": false,
  "error": {
    "code": "E001",
    "message": "Invalid mood value",
    "details": "Mood primary must be one of: happy, sad, anxious, calm, angry, neutral, mixed",
    "timestamp": "2026-01-12T10:30:00Z"
  }
}
```

### 9.2 Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| E001 | 400 | Invalid input data |
| E002 | 401 | Authentication required |
| E003 | 403 | Insufficient permissions |
| E004 | 404 | Resource not found |
| E005 | 409 | Consent not provided |
| E006 | 422 | Invalid assessment response |
| E007 | 429 | Rate limit exceeded |
| E008 | 500 | Internal server error |
| E009 | 503 | Service temporarily unavailable |
| E999 | 500 | Crisis escalation required |

## 10. Rate Limiting

```
Standard: 100 requests/minute
Assessment: 10 assessments/day
Crisis: Unlimited (priority routing)
Reports: 5 reports/day
```

---

**弘益人間 (Benefit All Humanity)**

*© 2025 WIA - World Certification Industry Association*
*MIT License*
