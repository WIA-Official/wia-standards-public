# WIA-UNI-002: Phase 2 - API Interface Specification

**Version:** 1.0
**Status:** Complete
**Last Updated:** 2025-01-15

---

## 1. Overview

This document specifies the REST API for WIA-UNI-002 Unified ID System operations including credential issuance, verification, revocation, and zero-knowledge proof generation.

## 2. Base URL

```
Production: https://api.unified-id.kr/v1/
Staging: https://api-staging.unified-id.kr/v1/
```

## 3. Authentication

### 3.1 OAuth 2.0 with OpenID Connect

All API requests require Bearer token authentication:

```http
Authorization: Bearer {access_token}
```

### 3.2 Token Endpoint

```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials&
client_id={client_id}&
client_secret={client_secret}&
scope=verify:credentials
```

## 4. Core Endpoints

### 4.1 Issue Credential

**Endpoint:** `POST /v1/credentials`

**Authorization:** Government Authority only

**Request:**
```json
{
  "personalInfo": {
    "familyName": "Kim",
    "givenName": "MinJun",
    "birthDate": "1990-05-15"
  },
  "biometric": {
    "fingerprintHash": "sha256:...",
    "facialHash": "sha256:..."
  }
}
```

**Response (201 Created):**
```json
{
  "unifiedId": "UNI-KR-900515-1234",
  "credential": {
    "@context": ["https://www.w3.org/2018/credentials/v1"],
    "type": ["VerifiableCredential", "UnifiedIDCredential"],
    "issuer": "did:wia:unified-korea-authority",
    "credentialSubject": {
      "id": "did:wia:UNI-KR-900515-1234"
    },
    "proof": {...}
  }
}
```

### 4.2 Verify Credential

**Endpoint:** `POST /v1/verify`

**Authorization:** Service Provider

**Request:**
```json
{
  "credential": {
    "@context": ["https://www.w3.org/2018/credentials/v1"],
    ...
  }
}
```

**Response (200 OK):**
```json
{
  "verified": true,
  "checks": {
    "signature": "valid",
    "expiration": "valid",
    "revocation": "not_revoked"
  }
}
```

### 4.3 Generate Zero-Knowledge Proof

**Endpoint:** `POST /v1/proofs/zk`

**Authorization:** Citizen

**Request:**
```json
{
  "unifiedId": "UNI-KR-900515-1234",
  "proofType": "age-over-18",
  "parameters": {
    "currentDate": "2025-01-15"
  }
}
```

**Response (200 OK):**
```json
{
  "proof": {
    "type": "zk-SNARK",
    "proof": "0x...",
    "publicInputs": {
      "currentDate": "2025-01-15",
      "ageThreshold": 18
    }
  }
}
```

### 4.4 Revoke Credential

**Endpoint:** `POST /v1/revoke`

**Authorization:** Authority or Citizen

**Request:**
```json
{
  "unifiedId": "UNI-KR-900515-1234",
  "reason": "lost",
  "effectiveDate": "2025-01-15T10:00:00Z"
}
```

**Response (200 OK):**
```json
{
  "revoked": true,
  "revocationId": "rev-abc123"
}
```

## 5. Rate Limiting

### 5.1 Rate Limits by Client Type

| Client Type | Requests/Hour | Burst Limit |
|-------------|---------------|-------------|
| Government Authority | 100,000 | 1,000 |
| Certified Service Provider | 10,000 | 100 |
| Standard Service Provider | 1,000 | 50 |
| Individual Citizen | 100 | 10 |

### 5.2 Rate Limit Headers

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1642252800
```

## 6. Error Handling

### 6.1 Standard Error Response

```json
{
  "error": {
    "code": "invalid_request",
    "message": "Missing required field: personalInfo.birthDate",
    "timestamp": "2025-01-15T10:30:00Z",
    "requestId": "req-xyz789"
  }
}
```

### 6.2 HTTP Status Codes

| Code | Description | Example |
|------|-------------|---------|
| 200 | Success | Verification successful |
| 201 | Created | Credential issued |
| 400 | Bad Request | Invalid JSON |
| 401 | Unauthorized | Missing/invalid token |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Credential not found |
| 429 | Rate Limit Exceeded | Too many requests |
| 500 | Internal Server Error | System error |

## 7. Webhooks

### 7.1 Credential Revocation Notifications

Registered service providers receive webhook notifications when credentials are revoked:

```http
POST {webhook_url}
Content-Type: application/json
X-WIA-Signature: sha256-hmac-signature

{
  "event": "credential.revoked",
  "unifiedId": "UNI-KR-900515-1234",
  "timestamp": "2025-01-15T10:00:00Z"
}
```

---

**弘益人間 (Benefit All Humanity)**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
