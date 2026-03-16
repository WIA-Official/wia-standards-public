# WIA-CORE-002 PHASE 2: API Interface Specification

**Version:** 1.0  
**Status:** Stable  
**Last Updated:** January 2025

## Overview

Phase 2 defines standardized REST and GraphQL APIs for programmatic consent management. These APIs enable systems to create, read, update, delete, and verify consent records in compliance with WIA-CORE-002 standards.

## API Design Principles

1. **RESTful:** Standard HTTP methods and predictable URLs
2. **Secure:** OAuth 2.0 authentication and TLS 1.3 encryption
3. **Versioned:** API versioning through URL path (/api/v1/)
4. **Idempotent:** Safe retry behavior for operations
5. **Documented:** OpenAPI 3.0 specification available
6. **Rate-Limited:** Protection against abuse
7. **Paginated:** Cursor-based pagination for large result sets

## Authentication

All API requests require authentication using OAuth 2.0 or API keys.

### OAuth 2.0 Scopes

- `consents.read` - Read consent records
- `consents.write` - Create and update consent records
- `consents.delete` - Delete and revoke consent records
- `consents.admin` - Administrative operations

### Request Headers

```
Authorization: Bearer {access_token}
Content-Type: application/json
X-WIA-Request-ID: {unique-request-id}
```

## Core Endpoints

### 1. Create Consent

**Endpoint:** `POST /api/v1/consents`

**Request:**
```json
{
  "userId": "user-789012",
  "purposes": [
    {
      "purposeId": "marketing-email",
      "granted": true
    },
    {
      "purposeId": "analytics",
      "granted": true
    }
  ],
  "jurisdiction": "EU",
  "legalBasis": "consent",
  "metadata": {
    "source": "web-signup",
    "ipAddress": "192.0.2.1",
    "consentFormVersion": "2.3"
  }
}
```

**Response (201 Created):**
```json
{
  "consentId": "consent-550e8400-e29b-41d4-a716-446655440000",
  "userId": "user-789012",
  "status": "active",
  "createdAt": "2025-01-15T10:30:00Z",
  "expiresAt": "2026-01-15T10:30:00Z",
  "purposes": [...],
  "_links": {
    "self": "/api/v1/consents/consent-550e8400...",
    "user": "/api/v1/users/user-789012/consents"
  }
}
```

### 2. Retrieve Consent

**Endpoint:** `GET /api/v1/consents/{consentId}`

**Response (200 OK):**
```json
{
  "consentId": "consent-550e8400...",
  "userId": "user-789012",
  "version": "1.0",
  "standard": "WIA-CORE-002",
  "timestamp": "2025-01-15T10:30:00Z",
  "status": "active",
  "purposes": [...],
  "metadata": {...},
  "auditTrail": [...]
}
```

### 3. List User Consents

**Endpoint:** `GET /api/v1/users/{userId}/consents`

**Query Parameters:**
- `status` - Filter by status (active, revoked, expired)
- `purposeId` - Filter by purpose
- `limit` - Number of results (default: 50, max: 100)
- `cursor` - Pagination cursor

**Response (200 OK):**
```json
{
  "data": [
    {
      "consentId": "consent-1...",
      "status": "active",
      "createdAt": "2025-01-15T10:30:00Z"
    },
    {
      "consentId": "consent-2...",
      "status": "active",
      "createdAt": "2025-01-10T14:20:00Z"
    }
  ],
  "pagination": {
    "cursor": "eyJpZCI6MTIzfQ==",
    "hasMore": true,
    "total": 247
  }
}
```

### 4. Update Consent

**Endpoint:** `PATCH /api/v1/consents/{consentId}`

**Request:**
```json
{
  "purposes": [
    {
      "purposeId": "marketing-email",
      "granted": false
    }
  ],
  "metadata": {
    "source": "preference-center"
  }
}
```

**Response (200 OK):**
```json
{
  "consentId": "consent-550e8400...",
  "status": "active",
  "updatedAt": "2025-06-20T14:22:00Z",
  "purposes": [...]
}
```

### 5. Revoke Consent

**Endpoint:** `POST /api/v1/consents/{consentId}/revoke`

**Request:**
```json
{
  "reason": "User requested via preference center",
  "revokeAll": false
}
```

**Response (200 OK):**
```json
{
  "consentId": "consent-550e8400...",
  "status": "revoked",
  "revokedAt": "2025-12-01T09:15:00Z",
  "revokedBy": "user-789012"
}
```

### 6. Verify Consent

**Endpoint:** `POST /api/v1/consents/verify`

**Request:**
```json
{
  "userId": "user-789012",
  "purposeId": "marketing-email",
  "context": {
    "timestamp": "2025-06-20T14:30:00Z",
    "source": "email-campaign-system"
  }
}
```

**Response (200 OK):**
```json
{
  "isValid": true,
  "consentId": "consent-550e8400...",
  "grantedAt": "2025-01-15T10:30:00Z",
  "expiresAt": "2026-01-15T10:30:00Z",
  "purposes": ["marketing-email"],
  "verificationToken": "verify-abc123...",
  "validUntil": "2025-06-20T15:30:00Z"
}
```

## Batch Operations

**Endpoint:** `POST /api/v1/consents/batch`

**Request:**
```json
{
  "operations": [
    {
      "operation": "create",
      "data": {
        "userId": "user-1",
        "purposes": [...]
      }
    },
    {
      "operation": "update",
      "consentId": "consent-abc...",
      "data": {
        "purposes": [...]
      }
    },
    {
      "operation": "revoke",
      "consentId": "consent-xyz..."
    }
  ]
}
```

**Response (200 OK):**
```json
{
  "results": [
    {
      "success": true,
      "consentId": "consent-new...",
      "operation": "create"
    },
    {
      "success": true,
      "consentId": "consent-abc...",
      "operation": "update"
    },
    {
      "success": false,
      "consentId": "consent-xyz...",
      "operation": "revoke",
      "error": {
        "code": "ALREADY_REVOKED",
        "message": "Consent already revoked"
      }
    }
  ],
  "summary": {
    "total": 3,
    "successful": 2,
    "failed": 1
  }
}
```

## Webhooks

**Configuration Endpoint:** `POST /api/v1/webhooks`

**Request:**
```json
{
  "url": "https://your-app.com/webhooks/consent",
  "events": ["consent.created", "consent.updated", "consent.revoked"],
  "secret": "whsec_...",
  "active": true
}
```

**Webhook Payload:**
```json
{
  "eventId": "evt-123...",
  "eventType": "consent.updated",
  "timestamp": "2025-06-20T14:22:00Z",
  "data": {
    "consentId": "consent-550e8400...",
    "userId": "user-789012",
    "changes": {
      "purposes.marketing-email.granted": {
        "old": true,
        "new": false
      }
    }
  }
}
```

**Webhook Verification:**
```
X-WIA-Signature: t=1642521600,v1=sha256_hash
```

## GraphQL Interface

**Endpoint:** `POST /api/v1/graphql`

**Query Example:**
```graphql
query GetUserConsents($userId: ID!) {
  user(id: $userId) {
    consents(status: ACTIVE) {
      consentId
      status
      createdAt
      purposes {
        purposeId
        purposeName
        granted
      }
    }
  }
}
```

**Variables:**
```json
{
  "userId": "user-789012"
}
```

## Error Handling

### Standard Error Response

```json
{
  "error": {
    "code": "INVALID_PURPOSE_ID",
    "message": "The purpose ID 'invalid-id' is not recognized",
    "details": {
      "field": "purposes[0].purposeId",
      "value": "invalid-id",
      "validValues": ["marketing-email", "analytics", ...]
    },
    "requestId": "req-abc123...",
    "timestamp": "2025-06-20T14:30:00Z"
  }
}
```

### Error Codes

- `INVALID_REQUEST` (400) - Malformed request
- `UNAUTHORIZED` (401) - Authentication required
- `FORBIDDEN` (403) - Insufficient permissions
- `NOT_FOUND` (404) - Resource not found
- `CONFLICT` (409) - Resource conflict
- `RATE_LIMIT_EXCEEDED` (429) - Too many requests
- `INTERNAL_ERROR` (500) - Server error

## Rate Limiting

```
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 987
X-RateLimit-Reset: 1642524000
```

## SDK Support

Official SDKs available for:
- TypeScript/JavaScript (`@wia/consent-sdk`)
- Python (`wia-consent-sdk`)
- Java (`com.wia:consent-sdk`)
- Go (`github.com/wia-official/consent-sdk-go`)
- Ruby (`wia-consent-sdk`)
- PHP (`wia/consent-sdk`)
- .NET (`WIA.Consent.SDK`)

---

**Previous:** [PHASE 1: Data Format](PHASE-1-DATA-FORMAT.md)  
**Next:** [PHASE 3: Protocol](PHASE-3-PROTOCOL.md)

© 2025 SmileStory Inc. / WIA · 弘益人間 (Benefit All Humanity)
