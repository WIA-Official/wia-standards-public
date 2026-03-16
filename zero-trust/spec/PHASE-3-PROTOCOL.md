# WIA-SEC-005: Zero Trust Architecture
## Phase 3 - Protocol Specifications

**Standard ID:** WIA-SEC-005
**Category:** Security (SEC)
**Version:** 1.0.0
**Status:** Active
**Last Updated:** 2025-12-25

---

## 1. Overview

This document defines the communication protocols, message flows, and interaction patterns for Zero Trust Architecture implementations. All protocols MUST use HTTPS with TLS 1.3 or higher unless otherwise specified.

---

## 2. Access Request Protocol

### 2.1 Initial Access Request Flow

```
┌──────┐                 ┌─────┐                ┌─────┐               ┌──────────┐
│Client│                 │ PEP │                │ PDP │               │Resource  │
└──┬───┘                 └──┬──┘                └──┬──┘               └────┬─────┘
   │                        │                      │                        │
   │  1. Access Request     │                      │                        │
   │───────────────────────>│                      │                        │
   │                        │                      │                        │
   │                        │ 2. Validate Request  │                        │
   │                        │───────────────────── │                        │
   │                        │                      │                        │
   │                        │ 3. Collect Context   │                        │
   │                        │<─ ─ ─ ─ ─ ─ ─ ─ ─ ─ │                        │
   │                        │                      │                        │
   │                        │ 4. Policy Request    │                        │
   │                        │─────────────────────>│                        │
   │                        │                      │                        │
   │                        │                      │ 5. Calculate Trust     │
   │                        │                      │─ ─ ─ ─ ─ ─ ─ ─ ─ ─    │
   │                        │                      │                        │
   │                        │                      │ 6. Evaluate Policy     │
   │                        │                      │<─ ─ ─ ─ ─ ─ ─ ─ ─ ─   │
   │                        │                      │                        │
   │                        │ 7. Policy Decision   │                        │
   │                        │<─────────────────────│                        │
   │                        │                      │                        │
   │  8. Access Response    │                      │                        │
   │<───────────────────────│                      │                        │
   │                        │                      │                        │
   │  9. Access Resource (if allowed)              │                        │
   │───────────────────────────────────────────────────────────────────────>│
   │                        │                      │                        │
   │ 10. Resource Response  │                      │                        │
   │<───────────────────────────────────────────────────────────────────────│
   │                        │                      │                        │
```

### 2.2 HTTP Request Specification

#### Endpoint
```
POST /api/v1/access/request
Host: zt-policy-engine.example.com
Content-Type: application/json
Authorization: Bearer {initial-auth-token}
X-Request-ID: {uuid}
X-Device-ID: {device-id}
X-Client-Version: 2.5.0
```

#### Request Body
```json
{
  "requestId": "req-{uuid}",
  "version": "1.0",
  "timestamp": "2025-12-25T10:30:00.000Z",
  "subject": { /* See PHASE-2-DATA.md */ },
  "device": { /* See PHASE-2-DATA.md */ },
  "resource": { /* See PHASE-2-DATA.md */ },
  "context": { /* See PHASE-2-DATA.md */ }
}
```

#### Success Response (200 OK)
```json
{
  "success": true,
  "timestamp": "2025-12-25T10:30:00.500Z",
  "requestId": "req-{uuid}",
  "data": {
    "decision": "allow",
    "decisionId": "dec-{uuid}",
    "trustScore": 91.5,
    "accessToken": "eyJhbGciOiJIUzI1NiIs...",
    "tokenType": "Bearer",
    "expiresIn": 3600,
    "expiresAt": "2025-12-25T11:30:00.000Z",
    "conditions": {
      "maxDuration": 3600,
      "requireContinuousVerification": true,
      "verificationInterval": 900,
      "allowedActions": ["read"]
    }
  }
}
```

#### Denial Response (403 Forbidden)
```json
{
  "success": false,
  "timestamp": "2025-12-25T10:30:00.500Z",
  "requestId": "req-{uuid}",
  "error": {
    "code": "ACCESS_DENIED",
    "message": "Access denied: Insufficient trust score",
    "details": {
      "decision": "deny",
      "decisionId": "dec-{uuid}",
      "trustScore": 65,
      "requiredScore": 85,
      "reasons": [
        "Device compliance below threshold",
        "Unusual location detected"
      ]
    },
    "remediation": [
      "Ensure device meets compliance requirements",
      "Contact IT support for assistance"
    ]
  }
}
```

#### Step-Up Required Response (401 Unauthorized)
```json
{
  "success": false,
  "timestamp": "2025-12-25T10:30:00.500Z",
  "requestId": "req-{uuid}",
  "error": {
    "code": "STEP_UP_REQUIRED",
    "message": "Additional authentication required",
    "details": {
      "decision": "step-up",
      "decisionId": "dec-{uuid}",
      "trustScore": 72,
      "requiredScore": 85,
      "challengeId": "challenge-{uuid}",
      "requiredActions": [
        {
          "type": "biometric",
          "method": "fingerprint",
          "timeout": 300
        }
      ]
    }
  }
}
```

---

## 3. Continuous Verification Protocol

### 3.1 Re-verification Flow

```
┌──────┐                 ┌─────┐                ┌─────┐
│Client│                 │ PEP │                │ PDP │
└──┬───┘                 └──┬──┘                └──┬──┘
   │                        │                      │
   │  Active Session        │                      │
   │<══════════════════════>│                      │
   │                        │                      │
   │                        │ [Verification Timer] │
   │                        │                      │
   │                        │ 1. Re-verify Request │
   │                        │─────────────────────>│
   │                        │                      │
   │                        │                      │ 2. Calculate Trust
   │                        │                      │─ ─ ─ ─ ─ ─ ─ ─ ─
   │                        │                      │
   │                        │ 3. Verification OK   │
   │                        │<─────────────────────│
   │                        │                      │
   │  Continue Session      │                      │
   │<══════════════════════>│                      │
```

### 3.2 Re-verification Request

#### Endpoint
```
POST /api/v1/session/verify
Authorization: Bearer {access-token}
X-Session-ID: {session-id}
```

#### Request Body
```json
{
  "sessionId": "sess-{uuid}",
  "timestamp": "2025-12-25T10:45:00.000Z",
  "context": {
    "currentIP": "192.168.1.100",
    "currentLocation": {
      "latitude": 37.7749,
      "longitude": -122.4194
    }
  }
}
```

#### Success Response (200 OK)
```json
{
  "success": true,
  "verified": true,
  "trustScore": 90.5,
  "nextVerification": "2025-12-25T11:00:00.000Z",
  "sessionExtended": true,
  "expiresAt": "2025-12-25T11:45:00.000Z"
}
```

#### Trust Score Degraded (200 OK with warnings)
```json
{
  "success": true,
  "verified": true,
  "trustScore": 72,
  "warnings": [
    {
      "code": "TRUST_SCORE_DEGRADED",
      "message": "Trust score has decreased",
      "action": "Step-up authentication recommended"
    }
  ],
  "nextVerification": "2025-12-25T10:50:00.000Z",
  "verificationInterval": 300
}
```

#### Session Revoked (401 Unauthorized)
```json
{
  "success": false,
  "verified": false,
  "error": {
    "code": "SESSION_REVOKED",
    "message": "Session revoked due to low trust score",
    "trustScore": 45,
    "reason": "Significant location change detected"
  }
}
```

---

## 4. Step-Up Authentication Protocol

### 4.1 Step-Up Challenge Flow

```
┌──────┐                 ┌─────┐                ┌─────┐
│Client│                 │ PEP │                │ PDP │
└──┬───┘                 └──┬──┘                └──┬──┘
   │                        │                      │
   │  1. Access Request     │                      │
   │───────────────────────>│─────────────────────>│
   │                        │                      │
   │  2. Step-Up Required   │                      │
   │<───────────────────────│<─────────────────────│
   │  (challenge)           │                      │
   │                        │                      │
   │  3. Complete Challenge │                      │
   │───────────────────────>│                      │
   │  (biometric/MFA)       │                      │
   │                        │                      │
   │                        │ 4. Verify Challenge  │
   │                        │─────────────────────>│
   │                        │                      │
   │  5. Access Granted     │                      │
   │<───────────────────────│<─────────────────────│
   │  (with token)          │                      │
```

### 4.2 Challenge Request

#### Endpoint
```
POST /api/v1/auth/challenge/{challengeId}
Authorization: Bearer {initial-token}
```

#### Request Body
```json
{
  "challengeId": "challenge-{uuid}",
  "response": {
    "type": "biometric",
    "data": {
      "method": "fingerprint",
      "biometricHash": "sha256-hash-of-biometric-template",
      "timestamp": "2025-12-25T10:30:15.000Z"
    }
  }
}
```

#### Success Response (200 OK)
```json
{
  "success": true,
  "verified": true,
  "accessToken": "eyJhbGciOiJIUzI1NiIs...",
  "tokenType": "Bearer",
  "expiresIn": 3600,
  "trustScore": 92,
  "decision": "allow"
}
```

---

## 5. Session Management Protocol

### 5.1 Session Lifecycle

```
┌─────────────┐
│   Created   │
└──────┬──────┘
       │
       ▼
┌─────────────┐      ┌──────────────┐
│   Active    │─────>│   Expired    │
└──────┬──────┘      └──────────────┘
       │
       ├────────────>┌──────────────┐
       │             │   Revoked    │
       │             └──────────────┘
       │
       ▼
┌─────────────┐
│  Suspended  │
└──────┬──────┘
       │
       ▼
┌─────────────┐
│   Active    │
└─────────────┘
```

### 5.2 Session Creation

#### Endpoint
```
POST /api/v1/session
Authorization: Bearer {auth-token}
```

#### Request Body
```json
{
  "userId": "user-12345",
  "deviceId": "device-67890",
  "requestedDuration": 14400,
  "context": { /* context data */ }
}
```

#### Response (201 Created)
```json
{
  "success": true,
  "session": {
    "sessionId": "sess-{uuid}",
    "status": "active",
    "created": "2025-12-25T10:30:00.000Z",
    "expiresAt": "2025-12-25T14:30:00.000Z",
    "accessToken": "eyJhbGciOiJIUzI1NiIs...",
    "refreshToken": "eyJhbGciOiJIUzI1NiIs...",
    "verificationInterval": 900
  }
}
```

### 5.3 Session Termination

#### Endpoint
```
DELETE /api/v1/session/{sessionId}
Authorization: Bearer {access-token}
```

#### Response (200 OK)
```json
{
  "success": true,
  "message": "Session terminated successfully",
  "sessionId": "sess-{uuid}",
  "terminatedAt": "2025-12-25T12:00:00.000Z"
}
```

---

## 6. Token Management Protocol

### 6.1 Token Structure (JWT)

```json
{
  "header": {
    "alg": "RS256",
    "typ": "JWT",
    "kid": "key-2025-01"
  },
  "payload": {
    "iss": "https://zt-policy-engine.example.com",
    "sub": "user-12345",
    "aud": "https://api.example.com",
    "exp": 1735128600,
    "iat": 1735125000,
    "nbf": 1735125000,
    "jti": "token-{uuid}",
    "sid": "sess-{uuid}",
    "device_id": "device-67890",
    "trust_score": 91.5,
    "access": {
      "resources": ["res-finance-001"],
      "actions": ["read"],
      "conditions": {
        "ip_pinning": "192.168.1.100",
        "verification_required": true
      }
    }
  },
  "signature": "..."
}
```

### 6.2 Token Refresh

#### Endpoint
```
POST /api/v1/token/refresh
Content-Type: application/json
```

#### Request Body
```json
{
  "refreshToken": "eyJhbGciOiJIUzI1NiIs...",
  "sessionId": "sess-{uuid}"
}
```

#### Response (200 OK)
```json
{
  "success": true,
  "accessToken": "eyJhbGciOiJIUzI1NiIs...",
  "tokenType": "Bearer",
  "expiresIn": 3600,
  "refreshToken": "eyJhbGciOiJIUzI1NiIs..."
}
```

### 6.3 Token Revocation

#### Endpoint
```
POST /api/v1/token/revoke
Authorization: Bearer {access-token}
```

#### Request Body
```json
{
  "token": "eyJhbGciOiJIUzI1NiIs...",
  "tokenType": "access_token|refresh_token"
}
```

#### Response (200 OK)
```json
{
  "success": true,
  "revoked": true,
  "tokenId": "token-{uuid}",
  "revokedAt": "2025-12-25T12:00:00.000Z"
}
```

---

## 7. Policy Query Protocol

### 7.1 Policy Evaluation Request

#### Endpoint
```
POST /api/v1/policy/evaluate
Authorization: Bearer {admin-token}
```

#### Request Body
```json
{
  "subject": {
    "userId": "user-12345",
    "groups": ["finance-team"]
  },
  "resource": {
    "resourceId": "res-finance-001",
    "actions": ["read", "write"]
  },
  "context": {
    "trustScore": 85,
    "deviceCompliant": true
  }
}
```

#### Response (200 OK)
```json
{
  "success": true,
  "evaluation": {
    "decision": "allow",
    "matchedPolicies": [
      {
        "policyId": "pol-12345",
        "policyName": "Finance Dashboard Access",
        "priority": 100,
        "effect": "allow"
      }
    ],
    "allowedActions": ["read"],
    "deniedActions": ["write"],
    "conditions": {
      "maxDuration": 3600
    }
  }
}
```

---

## 8. Event Streaming Protocol

### 8.1 Server-Sent Events (SSE)

#### Endpoint
```
GET /api/v1/events/stream
Authorization: Bearer {access-token}
Accept: text/event-stream
```

#### Event Stream
```
event: trust_score_update
data: {"sessionId":"sess-abc","trustScore":90.5,"timestamp":"2025-12-25T10:45:00Z"}

event: session_warning
data: {"sessionId":"sess-abc","warning":"location_change","timestamp":"2025-12-25T10:46:00Z"}

event: session_revoked
data: {"sessionId":"sess-abc","reason":"low_trust_score","timestamp":"2025-12-25T10:47:00Z"}
```

### 8.2 WebSocket Protocol

#### Connection
```
ws://zt-policy-engine.example.com/api/v1/events/ws
Sec-WebSocket-Protocol: wia-zero-trust-v1
Authorization: Bearer {access-token}
```

#### Message Format
```json
{
  "type": "event",
  "eventType": "trust_score_update|session_warning|access_denied",
  "timestamp": "2025-12-25T10:45:00.000Z",
  "data": {
    "sessionId": "sess-abc",
    "trustScore": 90.5
  }
}
```

---

## 9. Health Check Protocol

### 9.1 Service Health

#### Endpoint
```
GET /health
```

#### Response (200 OK)
```json
{
  "status": "healthy|degraded|unhealthy",
  "timestamp": "2025-12-25T10:30:00.000Z",
  "version": "2.5.0",
  "services": {
    "policyEngine": "healthy",
    "trustCalculator": "healthy",
    "database": "healthy",
    "cache": "degraded"
  },
  "metrics": {
    "requestsPerSecond": 1250,
    "averageLatency": 125,
    "errorRate": 0.01
  }
}
```

### 9.2 Readiness Check

#### Endpoint
```
GET /ready
```

#### Response (200 OK)
```json
{
  "ready": true,
  "timestamp": "2025-12-25T10:30:00.000Z"
}
```

---

## 10. Error Handling

### 10.1 Standard Error Codes

| HTTP Status | Error Code | Description |
|------------|------------|-------------|
| 400 | INVALID_REQUEST | Malformed request |
| 401 | UNAUTHORIZED | Authentication required |
| 401 | STEP_UP_REQUIRED | Additional auth needed |
| 403 | ACCESS_DENIED | Insufficient permissions |
| 403 | INSUFFICIENT_TRUST_SCORE | Trust score too low |
| 404 | NOT_FOUND | Resource not found |
| 409 | CONFLICT | Resource state conflict |
| 429 | RATE_LIMIT_EXCEEDED | Too many requests |
| 500 | INTERNAL_ERROR | Server error |
| 503 | SERVICE_UNAVAILABLE | Service temporarily down |

### 10.2 Error Response Format

```json
{
  "success": false,
  "timestamp": "2025-12-25T10:30:00.000Z",
  "requestId": "req-{uuid}",
  "error": {
    "code": "ERROR_CODE",
    "message": "Human-readable error message",
    "details": {
      "field": "Additional error details"
    },
    "remediation": [
      "Step 1 to fix",
      "Step 2 to fix"
    ]
  }
}
```

---

## 11. Rate Limiting

### 11.1 Rate Limit Headers

```
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1735128600
X-RateLimit-Policy: 1000req/hour
```

### 11.2 Rate Limit Exceeded Response

```json
{
  "success": false,
  "error": {
    "code": "RATE_LIMIT_EXCEEDED",
    "message": "Rate limit exceeded",
    "details": {
      "limit": 1000,
      "window": 3600,
      "resetAt": "2025-12-25T12:00:00.000Z"
    },
    "remediation": [
      "Wait until rate limit resets",
      "Contact support to increase limit"
    ]
  }
}
```

---

## 12. Protocol Security

### 12.1 TLS Requirements

- **Minimum Version:** TLS 1.3
- **Cipher Suites (Recommended):**
  - TLS_AES_256_GCM_SHA384
  - TLS_CHACHA20_POLY1305_SHA256
  - TLS_AES_128_GCM_SHA256

### 12.2 Certificate Pinning

```json
{
  "publicKeyPins": [
    "sha256/AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=",
    "sha256/BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB="
  ],
  "maxAge": 5184000,
  "includeSubdomains": true
}
```

### 12.3 Request Signing

```
POST /api/v1/access/request
Authorization: Bearer {token}
X-Request-Signature: sha256={signature}
X-Request-Timestamp: 2025-12-25T10:30:00.000Z
X-Request-Nonce: {uuid}
```

Signature calculation:
```
signature = HMAC-SHA256(
    secret_key,
    method + "\n" +
    path + "\n" +
    timestamp + "\n" +
    nonce + "\n" +
    body_hash
)
```

---

## 13. API Versioning

### 13.1 Version in URL

```
https://zt-policy-engine.example.com/api/v1/access/request
https://zt-policy-engine.example.com/api/v2/access/request
```

### 13.2 Version in Header

```
GET /api/access/request
Accept: application/vnd.wia.zero-trust.v1+json
```

### 13.3 Version Deprecation

```json
{
  "success": true,
  "data": { /* response data */ },
  "warnings": [
    {
      "code": "DEPRECATED_VERSION",
      "message": "API version 1.0 is deprecated and will be removed on 2026-01-01",
      "upgradeUrl": "https://docs.wia.org/zero-trust/migration/v2"
    }
  ]
}
```

---

## 14. Batch Operations

### 14.1 Batch Access Request

#### Endpoint
```
POST /api/v1/access/batch
```

#### Request Body
```json
{
  "requests": [
    {
      "requestId": "req-1",
      "resource": { /* resource 1 */ }
    },
    {
      "requestId": "req-2",
      "resource": { /* resource 2 */ }
    }
  ]
}
```

#### Response (200 OK)
```json
{
  "success": true,
  "results": [
    {
      "requestId": "req-1",
      "success": true,
      "decision": "allow"
    },
    {
      "requestId": "req-2",
      "success": false,
      "error": { /* error details */ }
    }
  ]
}
```

---

## 15. Pagination

### 15.1 Pagination Parameters

```
GET /api/v1/audit/events?page=2&pageSize=50&sort=timestamp:desc
```

### 15.2 Pagination Response

```json
{
  "success": true,
  "data": [ /* items */ ],
  "pagination": {
    "page": 2,
    "pageSize": 50,
    "totalPages": 100,
    "totalItems": 4987,
    "hasNext": true,
    "hasPrevious": true
  },
  "links": {
    "first": "/api/v1/audit/events?page=1&pageSize=50",
    "previous": "/api/v1/audit/events?page=1&pageSize=50",
    "self": "/api/v1/audit/events?page=2&pageSize=50",
    "next": "/api/v1/audit/events?page=3&pageSize=50",
    "last": "/api/v1/audit/events?page=100&pageSize=50"
  }
}
```

---

## 16. Protocol Compliance

### 16.1 HTTP/2 Requirements

- Support HTTP/2 with server push for policy updates
- Multiplexing for concurrent requests
- Header compression (HPACK)

### 16.2 OAuth 2.0 Integration

```
POST /oauth/authorize
Content-Type: application/x-www-form-urlencoded

response_type=code&
client_id=client123&
redirect_uri=https://app.example.com/callback&
scope=zero-trust:access&
state=random-state
```

### 16.3 OpenID Connect Integration

```json
{
  "id_token": "eyJhbGciOiJSUzI1NiIs...",
  "access_token": "eyJhbGciOiJIUzI1NiIs...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "openid profile email zero-trust:access"
}
```

---

**Document Status:** ✅ Complete
**Next Phase:** [PHASE-4-INTEGRATION.md](./PHASE-4-INTEGRATION.md)

---

© 2025 SmileStory Inc. / WIA
**弘益人間 (홍익인간)** - Benefit All Humanity
