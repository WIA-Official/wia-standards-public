# WIA BCI Consent Protocol
## Phase 2: API Interface Specification

**Version:** 1.0.0  
**Status:** Release  
**Last Updated:** 2025-12-25

---

## 1. Introduction

This specification defines the standard APIs for managing BCI consent throughout its lifecycle. All APIs follow REST principles and use JSON for request/response payloads.

## 2. Core API Endpoints

### 2.1 Request Consent

```
POST /api/v1/consent/request
Content-Type: application/json
Authorization: Bearer {token}

Request Body:
{
  "subjectId": "string",
  "consentType": "initial|ongoing|research|...",
  "deviceType": "invasive|semi_invasive|non_invasive",
  "purpose": "string",
  "permissions": {...},
  "jurisdiction": "string"
}

Response 201 Created:
{
  "consentId": "string",
  "status": "pending_signature",
  "expiresAt": "ISO 8601 timestamp",
  "consentDocument": "URL to PDF document"
}
```

### 2.2 Verify Consent

```
POST /api/v1/consent/verify
Content-Type: application/json

Request:
{
  "consentId": "string",
  "requiredPermissions": ["string"]
}

Response 200 OK:
{
  "valid": boolean,
  "permissions": {...},
  "expiresAt": "string",
  "capacityScore": number
}

Response 403 Forbidden (if invalid):
{
  "valid": false,
  "reason": "CONSENT_EXPIRED|CONSENT_REVOKED|PERMISSION_DENIED",
  "message": "string"
}
```

### 2.3 Modify Consent

```
PATCH /api/v1/consent/{id}
Content-Type: application/json

Request:
{
  "permissions": {...},  // Updated permissions
  "reason": "string"     // Reason for modification
}

Response 200 OK:
{
  "consentId": "string",
  "version": number,
  "effectiveDate": "string"
}
```

### 2.4 Revoke Consent

```
DELETE /api/v1/consent/{id}
Content-Type: application/json

Request:
{
  "reason": "string",
  "effectiveDate": "string"  // Optional, defaults to immediate
}

Response 200 OK:
{
  "consentId": "string",
  "status": "revoked",
  "revokedAt": "string",
  "dataRetentionPeriod": "string"
}
```

### 2.5 Capacity Assessment

```
POST /api/v1/capacity/assess
Content-Type: application/json

Request:
{
  "subjectId": "string",
  "cognitiveScore": number (0-100),
  "understandingLevel": number (0-10),
  "decisionAbility": number (0-10),
  "method": "string"
}

Response 200 OK:
{
  "capacityScore": number (0-100),
  "sufficient": boolean,
  "recommendation": "string",
  "assessmentId": "string"
}
```

### 2.6 Emergency Override

```
POST /api/v1/emergency/override
Authorization: Bearer {medical_professional_token}

Request:
{
  "subjectId": "string",
  "reason": "medical_emergency|seizure_detection|system_malfunction|immediate_danger",
  "situation": "string",
  "authorizingPhysician": "string",
  "license": "string",
  "duration": number  // hours
}

Response 200 OK:
{
  "overrideId": "string",
  "status": "active",
  "expiresAt": "string",
  "reviewRequired": true,
  "ethicsBoardNotified": boolean
}
```

### 2.7 Audit Log

```
GET /api/v1/consent/{id}/audit

Response 200 OK:
{
  "consentId": "string",
  "events": [
    {
      "timestamp": "string",
      "action": "string",
      "actor": "string",
      "details": "string",
      "ipAddress": "string"
    }
  ]
}
```

## 3. Authentication & Authorization

### 3.1 Authentication Methods

- **API Key**: For server-to-server communication
- **OAuth 2.0**: For user-facing applications
- **JWT Tokens**: For mobile/web applications
- **mTLS**: For high-security medical devices

### 3.2 Authorization Levels

| Role | Permissions |
|------|------------|
| Subject | View own consent, modify permissions, revoke |
| Clinician | Request consent, assess capacity, view audit |
| Researcher | Request consent, verify permissions |
| Administrator | All operations except subject-specific actions |
| System | Automated operations, webhooks |

## 4. Rate Limiting

```
HTTP Headers:
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1640462400

Response 429 Too Many Requests:
{
  "error": "rate_limit_exceeded",
  "message": "API rate limit exceeded. Limit: 1000 req/hour",
  "retryAfter": 3600
}
```

## 5. Error Codes

| Code | HTTP Status | Description |
|------|------------|-------------|
| CONSENT_NOT_FOUND | 404 | Consent ID does not exist |
| CONSENT_EXPIRED | 403 | Consent has passed expiration |
| CONSENT_REVOKED | 403 | Consent was revoked |
| INSUFFICIENT_CAPACITY | 422 | Subject lacks capacity |
| PERMISSION_DENIED | 403 | Required permission not granted |
| INVALID_SIGNATURE | 422 | Signature verification failed |
| UNAUTHORIZED | 401 | Invalid/missing authentication |
| RATE_LIMIT_EXCEEDED | 429 | Too many requests |

## 6. Webhooks

### 6.1 Webhook Events

- `consent.granted` - New consent granted
- `consent.modified` - Consent permissions modified
- `consent.revoked` - Consent revoked by subject
- `consent.expired` - Consent reached expiration
- `consent.renewed` - Consent successfully renewed
- `emergency.override.activated` - Emergency override triggered
- `capacity.assessment.failed` - Capacity assessment insufficient

### 6.2 Webhook Payload

```json
{
  "event": "consent.revoked",
  "timestamp": "2025-12-25T10:30:00Z",
  "data": {
    "consentId": "CST-001",
    "subjectId": "SUBJ-001",
    "reason": "User request"
  }
}
```

---

**弘益人間 (Hongik Ingan)** - *Benefit All Humanity*

© 2025 SmileStory Inc. / WIA
