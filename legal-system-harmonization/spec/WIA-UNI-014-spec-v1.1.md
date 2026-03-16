# WIA-UNI-014: Legal System Harmonization - v1.1 Specification

## Phase 2: API Interface

**Version:** 1.1
**Status:** Final
**Date:** 2025-01-20
**Category:** UNI (Unification/Peace)
**Builds on:** v1.0 (Legal Data Format)

---

## 1. Overview

This specification defines RESTful API interfaces for accessing and managing legal documents in the WIA-UNI-014 ecosystem. It enables programmatic access to legal systems for applications and services.

## 2. API Design Principles

- **RESTful:** Resource-based URLs with standard HTTP methods
- **Stateless:** Each request contains all necessary information
- **Secure:** HTTPS only, OAuth 2.0 authentication
- **Versioned:** API version in URL path (`/api/v1/...`)
- **Paginated:** Large result sets use pagination
- **Rate Limited:** Prevent abuse with quotas

## 3. Base URL

Production: `https://api.wiastandards.com/uni-014/v1`
Sandbox: `https://sandbox-api.wiastandards.com/uni-014/v1`

## 4. Authentication

### 4.1 OAuth 2.0

All API requests require OAuth 2.0 Bearer token:

```http
Authorization: Bearer {access_token}
```

### 4.2 Scopes

- `legal:read` - Read public legal documents
- `legal:search` - Search legal databases
- `property:read` - Read property records
- `property:write` - Modify property records
- `contract:create` - Create contracts

## 5. Core Endpoints

### 5.1 Document Retrieval

```http
GET /api/v1/documents/{id}
Authorization: Bearer {token}

Response: 200 OK
{
  "standard": "WIA-UNI-014",
  "document": { ... }
}
```

### 5.2 Document Search

```http
GET /api/v1/documents?q={query}&type={type}&limit=20&offset=0
Authorization: Bearer {token}

Response: 200 OK
{
  "results": [...],
  "pagination": { "total": 100, "limit": 20, "offset": 0 }
}
```

### 5.3 Property APIs

```http
GET /api/v1/properties/{id}/owner
GET /api/v1/properties/{id}/history
POST /api/v1/properties/{id}/transfer
```

### 5.4 Contract APIs

```http
POST /api/v1/contracts
GET /api/v1/contracts/{id}
GET /api/v1/contracts/{id}/validity
```

## 6. Error Handling

```json
{
  "error": {
    "code": "INVALID_REQUEST",
    "message": "Human-readable error message",
    "requestId": "req_abc123",
    "timestamp": "2025-01-25T10:30:00Z"
  }
}
```

Error codes: `INVALID_REQUEST`, `UNAUTHORIZED`, `FORBIDDEN`, `NOT_FOUND`, `RATE_LIMIT_EXCEEDED`, `INTERNAL_ERROR`

## 7. Rate Limiting

- Free tier: 100 requests/hour
- Basic: 1,000 requests/hour
- Professional: 10,000 requests/hour
- Enterprise: 100,000 requests/hour

Headers:
```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 987
X-RateLimit-Reset: 1642582800
```

## 8. Pagination

```http
GET /api/v1/documents?limit=20&offset=40

Response:
{
  "data": [...],
  "pagination": {
    "total": 247,
    "limit": 20,
    "offset": 40,
    "links": {
      "next": "/api/v1/documents?limit=20&offset=60"
    }
  }
}
```

## 9. Conformance

API implementations MUST:
- Support all core endpoints
- Implement OAuth 2.0 authentication
- Return standardized error responses
- Implement rate limiting
- Support pagination for list endpoints

---

**© 2025 WIA - World Certification Industry Association**
**弘益人間 · Benefit All Humanity**
