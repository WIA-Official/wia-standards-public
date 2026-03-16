# WIA-CORE-001: Phase 2 - API Interface Specification

> **Phase:** 2 of 4  
> **Version:** 1.0.0  
> **Status:** Active  
> **Last Updated:** 2025-12-27

---

## Overview

Phase 2 defines standardized APIs for identity operations. These RESTful and GraphQL interfaces enable creating, managing, verifying, and revoking identities and credentials across platforms.

## API Design Principles

1. **RESTful:** HTTP methods match operations (GET, POST, PUT, DELETE)
2. **Stateless:** No server-side session state
3. **Versioned:** Clear API versioning (`/v1/`, `/v2/`)
4. **Authenticated:** Sensitive operations require authentication
5. **Rate Limited:** Prevent abuse through rate limiting
6. **Well-Documented:** OpenAPI 3.0 specifications

## Base URL Structure

```
https://api.wia.org/v1/{resource}
```

## Authentication

All API requests require Bearer token authentication:

```http
Authorization: Bearer eyJhbGciOiJFZERTQSJ9...
```

## Core API Endpoints

### Identity Management

#### POST /identity/create

Create a new universal identity.

**Request:**
```json
{
  "type": "email",
  "attributes": {
    "email": "user@example.com",
    "emailVerified": true,
    "name": "Alice Smith"
  },
  "didMethod": "web",
  "didHost": "example.com"
}
```

**Response (201 Created):**
```json
{
  "id": "did:web:example.com:users:alice",
  "type": "email",
  "trustLevel": 1,
  "attributes": {
    "email": "user@example.com",
    "emailVerified": true,
    "name": "Alice Smith"
  },
  "createdAt": "2025-01-15T19:23:24Z",
  "status": "active"
}
```

#### GET /identity/{did}

Retrieve identity information.

**Response (200 OK):**
```json
{
  "id": "did:web:example.com:users:alice",
  "type": "email",
  "trustLevel": 2,
  "createdAt": "2025-01-15T19:23:24Z",
  "updatedAt": "2025-01-20T10:15:30Z",
  "status": "active"
}
```

#### PUT /identity/{did}

Update identity attributes.

#### DELETE /identity/{did}

Delete identity (requires high trust level and MFA).

### Credential Operations

#### POST /credential/issue

Issue a verifiable credential.

**Request:**
```json
{
  "issuer": "did:web:example.com",
  "subject": "did:web:example.com:users:alice",
  "type": "EmailCredential",
  "claims": {
    "email": "alice@example.com",
    "emailVerified": true
  },
  "validityPeriod": "1y"
}
```

**Response (201 Created):**
```json
{
  "credential": {
    "@context": ["https://www.w3.org/2018/credentials/v1"],
    "id": "urn:uuid:3978344f-8596-4c3a-a978-8fcaba3903c5",
    "type": ["VerifiableCredential", "EmailCredential"],
    "issuer": "did:web:example.com",
    "issuanceDate": "2025-01-15T19:23:24Z",
    "expirationDate": "2026-01-15T19:23:24Z",
    "credentialSubject": {
      "id": "did:web:example.com:users:alice",
      "email": "alice@example.com",
      "emailVerified": true
    },
    "proof": { /* ... */ }
  }
}
```

#### POST /credential/verify

Verify a credential's authenticity.

**Request:**
```json
{
  "credential": { /* VC object */ }
}
```

**Response (200 OK):**
```json
{
  "valid": true,
  "checks": {
    "signatureValid": true,
    "notExpired": true,
    "notRevoked": true,
    "issuerTrusted": true
  },
  "issuer": {
    "did": "did:web:example.com",
    "name": "Example Inc",
    "trustScore": 95
  }
}
```

#### POST /credential/revoke

Revoke a credential.

### DID Operations

#### GET /did/resolve/{did}

Resolve DID to DID Document.

**Response (200 OK):**
```json
{
  "didDocument": {
    "@context": ["https://www.w3.org/ns/did/v1"],
    "id": "did:web:example.com:users:alice",
    "verificationMethod": [/* ... */]
  },
  "didDocumentMetadata": {
    "created": "2025-01-15T19:23:24Z",
    "updated": "2025-01-20T10:15:30Z"
  }
}
```

### Presentation Operations

#### POST /presentation/create

Create a verifiable presentation.

**Request:**
```json
{
  "holder": "did:web:example.com:users:alice",
  "verifier": "did:web:verifier.com",
  "credentials": [
    "urn:uuid:3978344f-8596-4c3a-a978-8fcaba3903c5"
  ],
  "challenge": "1f44d-6f6f61-71-72"
}
```

**Response (200 OK):**
```json
{
  "presentation": {
    "@context": ["https://www.w3.org/2018/credentials/v1"],
    "type": ["VerifiablePresentation"],
    "holder": "did:web:example.com:users:alice",
    "verifiableCredential": [/* ... */],
    "proof": {
      "challenge": "1f44d-6f6f61-71-72",
      /* ... */
    }
  }
}
```

## Error Responses

### Standard Error Format

```json
{
  "error": {
    "code": "INVALID_CREDENTIAL",
    "message": "Credential signature verification failed",
    "details": {
      "issuer": "did:web:example.com",
      "reason": "Public key not found in DID document"
    }
  }
}
```

### HTTP Status Codes

| Code | Meaning | Usage |
|------|---------|-------|
| 200 | OK | Successful request |
| 201 | Created | Resource created successfully |
| 400 | Bad Request | Invalid request format |
| 401 | Unauthorized | Missing or invalid authentication |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource doesn't exist |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server-side error |

## Rate Limiting

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 987
X-RateLimit-Reset: 1642262400
```

## Pagination

```http
GET /credentials?limit=50&offset=100
```

**Response:**
```json
{
  "items": [/* ... */],
  "pagination": {
    "total": 523,
    "limit": 50,
    "offset": 100,
    "hasMore": true
  }
}
```

## GraphQL Alternative

```graphql
query {
  identity(did: "did:web:example.com:users:alice") {
    id
    trustLevel
    attributes {
      email
      emailVerified
    }
    credentials {
      id
      type
      issuanceDate
      expirationDate
    }
  }
}
```

---

**Phase 2 Complete.** Proceed to [Phase 3: Protocol Implementation](PHASE-3-PROTOCOL.md).

**弘益人間 (Benefit All Humanity)**

© 2025 SmileStory Inc. / WIA
