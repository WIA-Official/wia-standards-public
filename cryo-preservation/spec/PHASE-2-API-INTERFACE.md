# WIA Cryo-Preservation API Interface Standard
## Phase 2 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #06B6D4 (Cyan)

---

## Table of Contents

1. [Overview](#overview)
2. [Authentication](#authentication)
3. [Base URL Structure](#base-url-structure)
4. [Endpoints](#endpoints)
5. [Request/Response Format](#requestresponse-format)
6. [Error Handling](#error-handling)
7. [Rate Limiting](#rate-limiting)
8. [SDK Examples](#sdk-examples)
9. [Webhooks](#webhooks)

---

## Overview

### 1.1 Purpose

The WIA Cryo-Preservation API Interface Standard defines RESTful APIs for managing cryopreservation records, monitoring storage conditions, and facilitating data exchange between facilities.

**Core Objectives**:
- Provide secure programmatic access to preservation data
- Enable real-time monitoring of storage conditions
- Support facility-to-facility data transfer
- Ensure audit compliance through comprehensive logging

### 1.2 API Design Principles

| Principle | Description |
|-----------|-------------|
| RESTful | Resource-oriented architecture |
| Versioned | API versioning in URL path |
| Secure | OAuth 2.0 + JWT authentication |
| Consistent | Uniform response structure |
| Paginated | Cursor-based pagination for lists |

### 1.3 Supported Operations

| Operation | HTTP Method | Description |
|-----------|-------------|-------------|
| Create | POST | Create new records |
| Read | GET | Retrieve records |
| Update | PATCH | Partial updates |
| Delete | DELETE | Soft delete (archival) |
| List | GET | Paginated listing |

---

## Authentication

### 2.1 API Key Authentication

For server-to-server communication:

```http
GET /api/v1/subjects
Authorization: Bearer <api_key>
X-Facility-ID: FAC-KR-001
```

### 2.2 OAuth 2.0 Flow

For user-facing applications:

```
┌──────────┐                               ┌──────────┐
│  Client  │                               │   Auth   │
│   App    │                               │  Server  │
└────┬─────┘                               └────┬─────┘
     │                                          │
     │  1. Authorization Request                │
     │  ─────────────────────────────────────► │
     │                                          │
     │  2. Authorization Code                   │
     │  ◄───────────────────────────────────── │
     │                                          │
     │  3. Token Request (code + secret)        │
     │  ─────────────────────────────────────► │
     │                                          │
     │  4. Access Token + Refresh Token         │
     │  ◄───────────────────────────────────── │
     │                                          │
```

#### Token Request

```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=authorization_code
&code=AUTH_CODE
&client_id=CLIENT_ID
&client_secret=CLIENT_SECRET
&redirect_uri=https://app.example.com/callback
```

#### Token Response

```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIs...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "refresh_token": "dGhpcyBpcyBhIHJlZnJlc2g...",
  "scope": "read write"
}
```

### 2.3 Scopes

| Scope | Description |
|-------|-------------|
| `read` | Read-only access to records |
| `write` | Create and update records |
| `admin` | Administrative operations |
| `transfer` | Inter-facility transfers |
| `quality` | Quality assessment reports |

### 2.4 JWT Token Structure

```json
{
  "header": {
    "alg": "RS256",
    "typ": "JWT"
  },
  "payload": {
    "sub": "user-id",
    "iss": "https://auth.wia.live",
    "aud": "cryo-preservation-api",
    "exp": 1704067200,
    "iat": 1704063600,
    "scope": ["read", "write"],
    "facility_id": "FAC-KR-001"
  }
}
```

---

## Base URL Structure

### 3.1 URL Format

```
https://api.wia.live/cryo-preservation/v1/{resource}
```

### 3.2 Environment URLs

| Environment | Base URL |
|-------------|----------|
| Production | `https://api.wia.live/cryo-preservation/v1` |
| Staging | `https://staging-api.wia.live/cryo-preservation/v1` |
| Development | `https://dev-api.wia.live/cryo-preservation/v1` |

---

## Endpoints

### 4.1 Subject Management

#### Create Subject

```http
POST /api/v1/subjects
Content-Type: application/json
Authorization: Bearer <token>

{
  "anonymizedId": "ANON-550e8400",
  "consentId": "CONSENT-2024-1234",
  "demographics": {
    "birthYear": 1950,
    "biologicalSex": "male",
    "bloodType": "A+"
  }
}
```

**Response** `201 Created`
```json
{
  "success": true,
  "data": {
    "id": "SUBJ-2025-001",
    "anonymizedId": "ANON-550e8400",
    "consentId": "CONSENT-2024-1234",
    "demographics": {
      "birthYear": 1950,
      "biologicalSex": "male",
      "bloodType": "A+"
    },
    "createdAt": "2025-01-15T10:30:00Z",
    "updatedAt": "2025-01-15T10:30:00Z"
  }
}
```

#### Get Subject

```http
GET /api/v1/subjects/{subjectId}
Authorization: Bearer <token>
```

**Response** `200 OK`
```json
{
  "success": true,
  "data": {
    "id": "SUBJ-2025-001",
    "anonymizedId": "ANON-550e8400",
    "consentId": "CONSENT-2024-1234",
    "preservationStatus": "long_term_storage",
    "facilityId": "FAC-KR-001",
    "demographics": {
      "birthYear": 1950,
      "biologicalSex": "male",
      "bloodType": "A+"
    },
    "createdAt": "2025-01-15T10:30:00Z",
    "updatedAt": "2025-01-15T10:30:00Z"
  }
}
```

#### List Subjects

```http
GET /api/v1/subjects?status=long_term_storage&limit=20&cursor=abc123
Authorization: Bearer <token>
```

**Response** `200 OK`
```json
{
  "success": true,
  "data": [
    {
      "id": "SUBJ-2025-001",
      "anonymizedId": "ANON-550e8400",
      "preservationStatus": "long_term_storage"
    }
  ],
  "pagination": {
    "cursor": "def456",
    "hasMore": true,
    "total": 150
  }
}
```

### 4.2 Preservation Records

#### Create Preservation Record

```http
POST /api/v1/subjects/{subjectId}/preservation
Content-Type: application/json
Authorization: Bearer <token>

{
  "preservationType": "whole_body",
  "timeline": {
    "pronouncement": "2025-01-15T08:00:00Z",
    "stabilization_start": "2025-01-15T08:15:00Z"
  },
  "perfusion": {
    "cryoprotectant": "M22",
    "concentration": 0.7,
    "volume_liters": 15.5
  }
}
```

**Response** `201 Created`
```json
{
  "success": true,
  "data": {
    "recordId": "PRV-2025-001",
    "subjectId": "SUBJ-2025-001",
    "preservationType": "whole_body",
    "status": "in_progress",
    "createdAt": "2025-01-15T10:30:00Z"
  }
}
```

#### Update Preservation Status

```http
PATCH /api/v1/subjects/{subjectId}/preservation/{recordId}
Content-Type: application/json
Authorization: Bearer <token>

{
  "status": "long_term_storage",
  "timeline": {
    "storage_start": "2025-01-16T03:00:00Z"
  },
  "storage": {
    "container_id": "DEW-KR-001",
    "position": "A1-05"
  }
}
```

**Response** `200 OK`
```json
{
  "success": true,
  "data": {
    "recordId": "PRV-2025-001",
    "status": "long_term_storage",
    "updatedAt": "2025-01-16T03:00:00Z"
  }
}
```

#### Get Preservation History

```http
GET /api/v1/subjects/{subjectId}/preservation/history
Authorization: Bearer <token>
```

**Response** `200 OK`
```json
{
  "success": true,
  "data": [
    {
      "version": 3,
      "status": "long_term_storage",
      "timestamp": "2025-01-16T03:00:00Z",
      "changedBy": "STAFF-001"
    },
    {
      "version": 2,
      "status": "cooldown",
      "timestamp": "2025-01-15T14:30:00Z",
      "changedBy": "STAFF-001"
    },
    {
      "version": 1,
      "status": "perfusion",
      "timestamp": "2025-01-15T10:00:00Z",
      "changedBy": "STAFF-001"
    }
  ]
}
```

### 4.3 Storage Monitoring

#### Get Current Storage Conditions

```http
GET /api/v1/storage/containers/{containerId}/conditions
Authorization: Bearer <token>
```

**Response** `200 OK`
```json
{
  "success": true,
  "data": {
    "containerId": "DEW-KR-001",
    "timestamp": "2025-01-20T10:00:00Z",
    "temperature": {
      "current": -196.2,
      "unit": "celsius",
      "status": "normal"
    },
    "liquidNitrogen": {
      "level": 0.85,
      "estimatedDaysRemaining": 45
    },
    "vacuum": {
      "pressure": 1.2e-6,
      "unit": "mbar",
      "status": "normal"
    },
    "alerts": []
  }
}
```

#### Get Temperature History

```http
GET /api/v1/storage/containers/{containerId}/temperature?from=2025-01-01&to=2025-01-31
Authorization: Bearer <token>
```

**Response** `200 OK`
```json
{
  "success": true,
  "data": {
    "containerId": "DEW-KR-001",
    "period": {
      "from": "2025-01-01T00:00:00Z",
      "to": "2025-01-31T23:59:59Z"
    },
    "statistics": {
      "min": -196.5,
      "max": -195.8,
      "average": -196.1,
      "variance": 0.15
    },
    "readings": [
      { "timestamp": "2025-01-01T00:00:00Z", "value": -196.0 },
      { "timestamp": "2025-01-01T01:00:00Z", "value": -196.1 }
    ]
  }
}
```

### 4.4 Quality Reports

#### Submit Quality Report

```http
POST /api/v1/subjects/{subjectId}/quality-reports
Content-Type: application/json
Authorization: Bearer <token>

{
  "assessmentType": "monthly_review",
  "quality": {
    "vitrification_score": 0.92,
    "tissue_integrity": 0.88,
    "cpa_distribution": 0.95
  },
  "inspector": "STAFF-002",
  "notes": "All metrics within acceptable range"
}
```

**Response** `201 Created`
```json
{
  "success": true,
  "data": {
    "reportId": "QR-2025-001",
    "subjectId": "SUBJ-2025-001",
    "assessmentType": "monthly_review",
    "overallStatus": "excellent",
    "createdAt": "2025-01-20T10:00:00Z"
  }
}
```

### 4.5 Facility Transfers

#### Initiate Transfer

```http
POST /api/v1/transfers
Content-Type: application/json
Authorization: Bearer <token>

{
  "subjectId": "SUBJ-2025-001",
  "sourceFacility": "FAC-KR-001",
  "destinationFacility": "FAC-US-001",
  "reason": "family_request",
  "scheduledDate": "2025-02-15",
  "transportMethod": "cryogenic_dewar"
}
```

**Response** `201 Created`
```json
{
  "success": true,
  "data": {
    "transferId": "TRF-2025-001",
    "status": "pending_approval",
    "subjectId": "SUBJ-2025-001",
    "sourceFacility": "FAC-KR-001",
    "destinationFacility": "FAC-US-001",
    "createdAt": "2025-01-20T10:00:00Z"
  }
}
```

#### Approve Transfer

```http
POST /api/v1/transfers/{transferId}/approve
Authorization: Bearer <token>
```

#### Complete Transfer

```http
POST /api/v1/transfers/{transferId}/complete
Content-Type: application/json
Authorization: Bearer <token>

{
  "actualArrival": "2025-02-15T14:00:00Z",
  "conditionOnArrival": "excellent",
  "temperatureLog": [
    { "timestamp": "2025-02-15T08:00:00Z", "value": -196.0 },
    { "timestamp": "2025-02-15T14:00:00Z", "value": -195.8 }
  ]
}
```

---

## Request/Response Format

### 5.1 Standard Response Structure

#### Success Response

```json
{
  "success": true,
  "data": { },
  "meta": {
    "requestId": "req-123456",
    "timestamp": "2025-01-20T10:00:00Z",
    "version": "1.0.0"
  }
}
```

#### Error Response

```json
{
  "success": false,
  "error": {
    "code": "ERR_VALIDATION_FAILED",
    "message": "Validation failed",
    "details": [
      {
        "field": "perfusion.concentration",
        "message": "Value must be between 0.0 and 1.0"
      }
    ]
  },
  "meta": {
    "requestId": "req-123456",
    "timestamp": "2025-01-20T10:00:00Z"
  }
}
```

### 5.2 Pagination

```json
{
  "success": true,
  "data": [],
  "pagination": {
    "cursor": "eyJpZCI6MTIzfQ==",
    "hasMore": true,
    "limit": 20,
    "total": 150
  }
}
```

### 5.3 Common Headers

| Header | Description | Required |
|--------|-------------|----------|
| `Authorization` | Bearer token | Yes |
| `Content-Type` | `application/json` | Yes (POST/PATCH) |
| `X-Request-ID` | Client request ID | No |
| `X-Facility-ID` | Facility identifier | Conditional |
| `Accept-Language` | Response language | No |

---

## Error Handling

### 6.1 HTTP Status Codes

| Code | Description | Usage |
|------|-------------|-------|
| 200 | OK | Successful GET/PATCH |
| 201 | Created | Successful POST |
| 204 | No Content | Successful DELETE |
| 400 | Bad Request | Invalid input |
| 401 | Unauthorized | Authentication failed |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource not found |
| 409 | Conflict | Resource conflict |
| 422 | Unprocessable | Validation failed |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Error | Server error |

### 6.2 Error Codes

| Code | Description |
|------|-------------|
| `ERR_AUTHENTICATION_FAILED` | Invalid credentials |
| `ERR_AUTHORIZATION_FAILED` | Insufficient permissions |
| `ERR_VALIDATION_FAILED` | Input validation error |
| `ERR_RESOURCE_NOT_FOUND` | Resource does not exist |
| `ERR_RESOURCE_CONFLICT` | Duplicate or conflict |
| `ERR_RATE_LIMIT_EXCEEDED` | Too many requests |
| `ERR_INTERNAL_ERROR` | Server error |
| `ERR_FACILITY_MISMATCH` | Wrong facility |

---

## Rate Limiting

### 7.1 Limits

| Tier | Requests/Minute | Requests/Hour |
|------|-----------------|---------------|
| Basic | 60 | 1,000 |
| Standard | 300 | 10,000 |
| Enterprise | 1,000 | 100,000 |

### 7.2 Rate Limit Headers

```http
X-RateLimit-Limit: 300
X-RateLimit-Remaining: 299
X-RateLimit-Reset: 1704067200
```

### 7.3 Rate Limit Exceeded Response

```json
{
  "success": false,
  "error": {
    "code": "ERR_RATE_LIMIT_EXCEEDED",
    "message": "Rate limit exceeded. Please retry after 60 seconds.",
    "retryAfter": 60
  }
}
```

---

## SDK Examples

### 8.1 TypeScript/JavaScript

```typescript
import { WiaCryoClient } from '@wia/cryo-preservation';

const client = new WiaCryoClient({
  apiKey: 'your-api-key',
  facilityId: 'FAC-KR-001',
  environment: 'production'
});

// Create subject
const subject = await client.subjects.create({
  anonymizedId: 'ANON-550e8400',
  consentId: 'CONSENT-2024-1234',
  demographics: {
    birthYear: 1950,
    biologicalSex: 'male',
    bloodType: 'A+'
  }
});

// Get preservation record
const preservation = await client.preservation.get(subject.id);

// Monitor storage conditions
const conditions = await client.storage.getConditions('DEW-KR-001');

// Subscribe to alerts
client.alerts.subscribe('DEW-KR-001', (alert) => {
  console.log('Alert received:', alert);
});
```

### 8.2 Python

```python
from wia_cryo import WiaCryoClient

client = WiaCryoClient(
    api_key='your-api-key',
    facility_id='FAC-KR-001',
    environment='production'
)

# Create subject
subject = client.subjects.create(
    anonymized_id='ANON-550e8400',
    consent_id='CONSENT-2024-1234',
    demographics={
        'birth_year': 1950,
        'biological_sex': 'male',
        'blood_type': 'A+'
    }
)

# Get preservation record
preservation = client.preservation.get(subject.id)

# Monitor storage conditions
conditions = client.storage.get_conditions('DEW-KR-001')

# Submit quality report
report = client.quality.submit_report(
    subject_id=subject.id,
    assessment_type='monthly_review',
    quality={
        'vitrification_score': 0.92,
        'tissue_integrity': 0.88
    }
)
```

### 8.3 cURL Examples

```bash
# Create subject
curl -X POST https://api.wia.live/cryo-preservation/v1/subjects \
  -H "Authorization: Bearer $API_KEY" \
  -H "Content-Type: application/json" \
  -d '{
    "anonymizedId": "ANON-550e8400",
    "consentId": "CONSENT-2024-1234"
  }'

# Get storage conditions
curl https://api.wia.live/cryo-preservation/v1/storage/containers/DEW-KR-001/conditions \
  -H "Authorization: Bearer $API_KEY"

# Update preservation status
curl -X PATCH https://api.wia.live/cryo-preservation/v1/subjects/SUBJ-2025-001/preservation/PRV-2025-001 \
  -H "Authorization: Bearer $API_KEY" \
  -H "Content-Type: application/json" \
  -d '{
    "status": "long_term_storage"
  }'
```

---

## Webhooks

### 9.1 Webhook Events

| Event | Description |
|-------|-------------|
| `preservation.status_changed` | Preservation status updated |
| `storage.alert` | Storage condition alert |
| `transfer.initiated` | Transfer request created |
| `transfer.completed` | Transfer completed |
| `quality.report_submitted` | Quality report filed |

### 9.2 Webhook Payload

```json
{
  "event": "storage.alert",
  "timestamp": "2025-01-20T10:00:00Z",
  "data": {
    "containerId": "DEW-KR-001",
    "alertType": "temperature_variance",
    "severity": "warning",
    "message": "Temperature variance above threshold",
    "currentValue": -195.0,
    "threshold": -195.5
  },
  "signature": "sha256=..."
}
```

### 9.3 Webhook Security

Verify webhook signatures:

```typescript
import crypto from 'crypto';

function verifyWebhook(payload: string, signature: string, secret: string): boolean {
  const expected = crypto
    .createHmac('sha256', secret)
    .update(payload)
    .digest('hex');
  return `sha256=${expected}` === signature;
}
```

---

<div align="center">

**WIA Cryo-Preservation API Interface Standard v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA Standards Committee**

**MIT License**

</div>
