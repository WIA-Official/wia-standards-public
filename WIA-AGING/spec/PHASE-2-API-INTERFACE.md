# WIA-AGING Phase 2: API Interface Specification

> **Version:** 1.0.0
> **Status:** Official
> **Last Updated:** 2025-01-01
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

The WIA-AGING API Interface specification defines RESTful endpoints for interacting with aging assessment services. This specification enables standardized programmatic access to biological age calculations, biomarker data, and longevity interventions.

### 1.1 Base URL

```
Production: https://api.aging.wia.org/v1
Sandbox:    https://sandbox.aging.wia.org/v1
```

### 1.2 API Versioning

The API version is included in the URL path. Major versions may introduce breaking changes. Minor updates are backwards-compatible.

---

## 2. Authentication

### 2.1 OAuth 2.0 Authorization

WIA-AGING API uses OAuth 2.0 with the following grant types:

| Grant Type | Use Case |
|------------|----------|
| Authorization Code | Web applications with user interaction |
| Client Credentials | Server-to-server communication |
| Device Code | IoT and wearable devices |
| Refresh Token | Long-lived access |

### 2.2 Authorization Flow

```
┌──────────┐     ┌──────────────┐     ┌─────────────┐
│  Client  │────▶│ Auth Server  │────▶│ WIA-AGING   │
└──────────┘     └──────────────┘     │    API      │
     │                  │              └─────────────┘
     │  1. Auth Request │                    │
     │─────────────────▶│                    │
     │                  │                    │
     │  2. Auth Code    │                    │
     │◀─────────────────│                    │
     │                  │                    │
     │  3. Token Request│                    │
     │─────────────────▶│                    │
     │                  │                    │
     │  4. Access Token │                    │
     │◀─────────────────│                    │
     │                  │                    │
     │  5. API Request with Bearer Token     │
     │──────────────────────────────────────▶│
```

### 2.3 JWT Token Structure

```json
{
  "header": {
    "alg": "RS256",
    "typ": "JWT",
    "kid": "wia-aging-key-001"
  },
  "payload": {
    "iss": "https://auth.wia.org",
    "sub": "user:12345",
    "aud": "https://api.aging.wia.org",
    "exp": 1704067200,
    "iat": 1704063600,
    "scope": "read:profile write:assessment read:biomarkers"
  }
}
```

### 2.4 Scopes

| Scope | Description |
|-------|-------------|
| `read:profile` | Read aging profiles |
| `write:profile` | Create/update profiles |
| `read:assessment` | Read assessments |
| `write:assessment` | Create assessments |
| `read:biomarkers` | Read biomarker data |
| `write:biomarkers` | Submit biomarker data |
| `admin:organization` | Organization admin access |

---

## 3. API Endpoints

### 3.1 Profiles

#### GET /profiles
List aging profiles for the authenticated user.

**Request:**
```http
GET /v1/profiles?limit=10&offset=0 HTTP/1.1
Host: api.aging.wia.org
Authorization: Bearer {access_token}
Accept: application/json
```

**Response:**
```json
{
  "data": [
    {
      "id": "profile_abc123",
      "subject_id": "did:wia:subject:001",
      "chronological_age": 55,
      "biological_age": 48.3,
      "created_at": "2025-01-15T09:00:00Z"
    }
  ],
  "pagination": {
    "total": 42,
    "limit": 10,
    "offset": 0,
    "has_more": true
  }
}
```

#### POST /profiles
Create a new aging profile.

**Request:**
```http
POST /v1/profiles HTTP/1.1
Host: api.aging.wia.org
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "subject": {
    "chronological_age": 55,
    "gender": "female"
  }
}
```

**Response:**
```json
{
  "id": "profile_xyz789",
  "subject_id": "did:wia:subject:generated",
  "created_at": "2025-01-15T10:00:00Z",
  "status": "pending_assessment"
}
```

#### GET /profiles/{id}
Retrieve a specific profile.

#### PUT /profiles/{id}
Update an existing profile.

#### DELETE /profiles/{id}
Delete a profile (soft delete).

### 3.2 Assessments

#### POST /profiles/{id}/assessments
Create a biological age assessment.

**Request:**
```json
{
  "method": "phenotypic-levine",
  "biomarkers": [
    { "code": "WIA-AGE-001", "value": 0.8, "unit": "mg/L" },
    { "code": "WIA-AGE-003", "value": 4.5, "unit": "g/dL" },
    { "code": "WIA-AGE-004", "value": 0.9, "unit": "mg/dL" },
    { "code": "WIA-AGE-005", "value": 92, "unit": "mg/dL" },
    { "code": "WIA-AGE-007", "value": 28, "unit": "%" }
  ]
}
```

**Response:**
```json
{
  "id": "assessment_001",
  "profile_id": "profile_abc123",
  "biological_age": {
    "value": 48.3,
    "method": "phenotypic-levine",
    "confidence": 0.92,
    "age_difference": -6.7,
    "aging_rate": 0.88
  },
  "health_score": 85,
  "interpretation": "Excellent - Aging slower than average",
  "recommendations": [
    "Maintain current exercise routine",
    "Continue anti-inflammatory diet"
  ],
  "created_at": "2025-01-15T10:05:00Z"
}
```

#### GET /profiles/{id}/assessments
List all assessments for a profile.

#### GET /profiles/{id}/assessments/{assessment_id}
Get specific assessment details.

### 3.3 Biomarkers

#### POST /biomarkers/batch
Submit multiple biomarkers at once.

**Request:**
```json
{
  "profile_id": "profile_abc123",
  "source": "lab_results",
  "laboratory": {
    "id": "lab_seoul_001",
    "name": "Seoul Longevity Lab"
  },
  "biomarkers": [
    {
      "code": "WIA-AGE-001",
      "name": "C-Reactive Protein",
      "value": 0.8,
      "unit": "mg/L",
      "collected_at": "2025-01-15T07:00:00Z"
    }
  ]
}
```

#### GET /biomarkers/catalog
List all supported biomarkers.

### 3.4 Interventions

#### POST /profiles/{id}/interventions
Record a longevity intervention.

**Request:**
```json
{
  "type": "supplement",
  "name": "NMN",
  "dosage": "500mg",
  "frequency": "daily",
  "start_date": "2025-01-01",
  "notes": "Morning dose with breakfast"
}
```

#### GET /profiles/{id}/interventions
List interventions for a profile.

---

## 4. Error Handling

### 4.1 Error Response Format

```json
{
  "error": {
    "code": "INVALID_BIOMARKER_VALUE",
    "message": "Biomarker value is outside acceptable range",
    "details": {
      "field": "biomarkers[0].value",
      "value": -5,
      "constraint": "must be >= 0"
    },
    "request_id": "req_abc123xyz"
  }
}
```

### 4.2 Error Codes

| HTTP Status | Error Code | Description |
|-------------|------------|-------------|
| 400 | INVALID_REQUEST | Malformed request body |
| 400 | VALIDATION_ERROR | Field validation failed |
| 401 | UNAUTHORIZED | Missing or invalid token |
| 403 | FORBIDDEN | Insufficient permissions |
| 404 | NOT_FOUND | Resource not found |
| 409 | CONFLICT | Resource conflict |
| 429 | RATE_LIMITED | Too many requests |
| 500 | INTERNAL_ERROR | Server error |

---

## 5. Rate Limiting

### 5.1 Limits by Tier

| Tier | Requests/min | Requests/day | Concurrent |
|------|--------------|--------------|------------|
| Free | 60 | 1,000 | 5 |
| Basic | 300 | 10,000 | 20 |
| Professional | 1,000 | 100,000 | 50 |
| Enterprise | Custom | Custom | Custom |

### 5.2 Rate Limit Headers

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 985
X-RateLimit-Reset: 1704067260
X-RateLimit-Window: 60
```

---

## 6. Webhooks

### 6.1 Supported Events

| Event | Description |
|-------|-------------|
| `assessment.completed` | Biological age calculation complete |
| `profile.updated` | Profile information changed |
| `biomarker.alert` | Biomarker outside normal range |
| `intervention.reminder` | Intervention reminder due |

### 6.2 Webhook Payload

```json
{
  "id": "evt_abc123",
  "type": "assessment.completed",
  "timestamp": "2025-01-15T10:05:00Z",
  "data": {
    "profile_id": "profile_abc123",
    "assessment_id": "assessment_001",
    "biological_age": 48.3
  },
  "signature": "sha256=abc123..."
}
```

---

## 7. SDK Overview

### 7.1 TypeScript/JavaScript

```typescript
import { WiaAgingClient } from '@wia/aging-sdk';

const client = new WiaAgingClient({
  apiKey: 'your-api-key',
  environment: 'production'
});

const assessment = await client.assessments.create({
  profileId: 'profile_abc123',
  method: 'phenotypic-levine',
  biomarkers: [
    { code: 'WIA-AGE-001', value: 0.8, unit: 'mg/L' }
  ]
});

console.log(`Biological Age: ${assessment.biologicalAge.value}`);
```

### 7.2 Python

```python
from wia_aging import AgingClient

client = AgingClient(api_key='your-api-key')

assessment = client.assessments.create(
    profile_id='profile_abc123',
    method='phenotypic-levine',
    biomarkers=[
        {'code': 'WIA-AGE-001', 'value': 0.8, 'unit': 'mg/L'}
    ]
)

print(f"Biological Age: {assessment.biological_age.value}")
```

---

## 8. OpenAPI Specification

The complete OpenAPI 3.0 specification is available at:
- Documentation: https://docs.aging.wia.org
- OpenAPI JSON: https://api.aging.wia.org/v1/openapi.json
- OpenAPI YAML: https://api.aging.wia.org/v1/openapi.yaml

---

**弘益人間 (홍익인간) - Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
