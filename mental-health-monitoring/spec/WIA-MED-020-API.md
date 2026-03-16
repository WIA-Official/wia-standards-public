# WIA-MED-020 API Specification

**Version:** 1.0.0
**Protocol:** REST API over HTTPS (TLS 1.3)
**Format:** JSON
**Authentication:** OAuth 2.0 + PKCE

---

## Base URL

```
https://api.mentalhealth-app.com/v1
```

## Authentication

All requests require Bearer token:

```http
Authorization: Bearer {access_token}
```

## Core Endpoints

### 1. Mood Tracking

#### POST /mood
Record mood entry

**Request:**
```json
{
  "timestamp": "2025-12-26T14:30:00Z",
  "mood_score": 7,
  "energy_level": 6,
  "anxiety_level": 3,
  "sleep_hours": 7.5,
  "tags": ["work", "exercise"],
  "note": "Good day overall, morning run helped"
}
```

**Response:**
```json
{
  "id": "mood_abc123",
  "created_at": "2025-12-26T14:30:00Z",
  "insights": {
    "trend": "improving",
    "vs_7day_avg": "+1.2"
  }
}
```

---

### 2. Screening Assessments

#### POST /assessments/phq9
Submit PHQ-9 depression screening

**Request:**
```json
{
  "responses": [2, 2, 1, 2, 1, 1, 2, 1, 0],
  "timestamp": "2025-12-26T10:00:00Z"
}
```

**Response:**
```json
{
  "id": "phq9_xyz789",
  "total_score": 12,
  "severity": "moderate",
  "recommendations": [
    "Consider professional consultation",
    "Continue mood tracking"
  ],
  "suicide_risk": false
}
```

**PHQ-9 Question 9 Alert:**
If `responses[8] >= 1` (suicidal ideation), trigger crisis protocol:

```json
{
  "total_score": 15,
  "severity": "moderately_severe",
  "suicide_risk": true,
  "crisis_protocol": {
    "level": "high",
    "actions": [
      "Display crisis hotline",
      "Notify therapist (if enrolled)",
      "Safety plan activation"
    ]
  }
}
```

---

### 3. Digital Phenotyping

#### POST /phenotyping/mobility
Upload mobility metrics

**Request:**
```json
{
  "date": "2025-12-26",
  "home_time_percent": 65,
  "unique_locations": 4,
  "max_distance_km": 8.2,
  "total_movement_km": 12.5
}
```

---

### 4. Crisis Intervention

#### POST /crisis/alert
Trigger crisis intervention

**Request:**
```json
{
  "trigger": "keyword_detection",
  "keyword": "suicide",
  "context": "I can't take this anymore...",
  "severity": "emergency"
}
```

**Response:**
```json
{
  "protocol_activated": true,
  "hotlines": [
    {
      "country": "US",
      "number": "988",
      "name": "Suicide & Crisis Lifeline"
    },
    {
      "country": "KR",
      "number": "1393",
      "name": "자살예방상담전화"
    }
  ],
  "emergency_contacts_notified": ["contact_123"],
  "therapist_alerted": true
}
```

---

### 5. Safety Planning

#### PUT /safety-plan
Create or update safety plan

**Request:**
```json
{
  "warning_signs": ["chest tightness", "don't want to get out of bed"],
  "coping_strategies": ["listen to music", "take a walk"],
  "social_distractions": [
    {"type": "person", "name": "Best friend", "phone": "+1234567890"}
  ],
  "professional_contacts": [
    {"name": "Dr. Smith", "phone": "+0987654321", "role": "therapist"}
  ],
  "crisis_hotlines": ["988", "1393"],
  "environment_safety": "Medications locked in safe"
}
```

---

## Privacy & Security

### Encryption
- **In Transit**: TLS 1.3
- **At Rest**: AES-256
- **E2E Messaging**: Signal Protocol

### Data Retention
- Active use: Indefinite (with consent)
- After account deletion: 30 days grace period, then permanent deletion
- Anonymized research data: Retained with identifier removal

### GDPR Compliance Endpoints

#### GET /data/export
Export all user data (Data Portability)

**Response:** JSON file with all user data

#### DELETE /data/delete-account
Permanent account and data deletion

---

## Rate Limits

- **Mood tracking**: 10 entries/day
- **Assessments**: 5 submissions/day
- **API calls**: 1000 requests/hour/user

---

## Error Codes

| Code | Meaning |
|------|---------|
| 200 | Success |
| 400 | Bad Request (validation error) |
| 401 | Unauthorized (invalid token) |
| 403 | Forbidden (insufficient permissions) |
| 429 | Too Many Requests (rate limit) |
| 500 | Internal Server Error |

---

**Published by WIA · MIT License · 弘益人間**
