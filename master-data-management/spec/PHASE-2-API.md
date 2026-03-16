# WIA-DATA-009: Master Data Management Standard
## PHASE 2: API Specification

**Version:** 1.0.0  
**Status:** Draft  
**Last Updated:** 2025-12-26

---

## Overview

This phase defines the RESTful API specification for accessing and manipulating master data according to the WIA-DATA-009 standard.

## 1. API Design Principles

- **RESTful Architecture**: Standard HTTP methods and status codes
- **Resource-Oriented**: APIs organized around business entities
- **Stateless**: Each request contains all necessary information
- **Versioned**: APIs include version in URL path
- **Secure**: OAuth 2.0 / JWT authentication required
- **Consistent**: Standard request/response patterns across all endpoints

## 2. Base URL Structure

```
https://api.mdm.example.com/v1/
```

## 3. Authentication

### OAuth 2.0 Client Credentials Flow

```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials&
client_id=YOUR_CLIENT_ID&
client_secret=YOUR_CLIENT_SECRET&
scope=mdm:read mdm:write
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIs...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "mdm:read mdm:write"
}
```

### Using Access Token

```http
GET /v1/customers/123
Authorization: Bearer eyJhbGciOiJSUzI1NiIs...
```

## 4. Entity CRUD Operations

### 4.1 Create Entity

```http
POST /v1/{entityType}
Content-Type: application/json
Authorization: Bearer {token}

{
  "sourceSystem": "CRM",
  "sourceSystemId": "CRM-12345",
  "attributes": {
    "person": {
      "firstName": "John",
      "lastName": "Smith"
    }
  }
}
```

**Response (201 Created):**
```json
{
  "entityId": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
  "entityType": "customer",
  "status": "active",
  "createdAt": "2025-12-26T10:30:00Z"
}
```

### 4.2 Read Entity

```http
GET /v1/{entityType}/{entityId}
Authorization: Bearer {token}
```

**Response (200 OK):**
```json
{
  "entityId": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
  "entityType": "customer",
  "status": "active",
  "attributes": {
    "person": {
      "firstName": "John",
      "lastName": "Smith"
    }
  },
  "metadata": {
    "dataQualityScore": 95,
    "lastVerifiedDate": "2025-12-26T10:30:00Z"
  }
}
```

### 4.3 Update Entity

```http
PUT /v1/{entityType}/{entityId}
Content-Type: application/json
Authorization: Bearer {token}

{
  "attributes": {
    "person": {
      "firstName": "John",
      "lastName": "Smith",
      "email": "john.smith@example.com"
    }
  }
}
```

**Response (200 OK):**
```json
{
  "entityId": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
  "version": 2,
  "updatedAt": "2025-12-26T11:15:00Z"
}
```

### 4.4 Delete Entity

```http
DELETE /v1/{entityType}/{entityId}
Authorization: Bearer {token}
```

**Response (204 No Content)**

## 5. Search and Query Operations

### 5.1 Search Entities

```http
POST /v1/{entityType}/search
Content-Type: application/json
Authorization: Bearer {token}

{
  "query": {
    "filters": [
      {
        "field": "attributes.person.lastName",
        "operator": "eq",
        "value": "Smith"
      }
    ],
    "sort": [
      {"field": "createdAt", "order": "desc"}
    ],
    "pagination": {
      "page": 1,
      "pageSize": 20
    }
  }
}
```

**Response (200 OK):**
```json
{
  "results": [...],
  "pagination": {
    "page": 1,
    "pageSize": 20,
    "totalRecords": 150,
    "totalPages": 8
  }
}
```

### 5.2 Match Entities

```http
POST /v1/{entityType}/match
Content-Type: application/json
Authorization: Bearer {token}

{
  "attributes": {
    "person": {
      "firstName": "John",
      "lastName": "Smyth",
      "email": "j.smith@example.com"
    }
  },
  "matchThreshold": 0.8
}
```

**Response (200 OK):**
```json
{
  "matches": [
    {
      "entityId": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
      "matchScore": 0.92,
      "matchedAttributes": [
        {"name": "firstName", "similarity": 1.0},
        {"name": "lastName", "similarity": 0.85}
      ]
    }
  ]
}
```

## 6. Golden Record Operations

### 6.1 Get Golden Record

```http
GET /v1/{entityType}/{entityId}/golden
Authorization: Bearer {token}
```

### 6.2 Merge Entities

```http
POST /v1/{entityType}/merge
Content-Type: application/json
Authorization: Bearer {token}

{
  "sourceEntityIds": [
    "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
    "b2c3d4e5-f6g7-8901-bcde-f12345678901"
  ],
  "survivorshipRules": {
    "person.email": "most_recent",
    "person.phone": "most_trusted"
  }
}
```

## 7. Relationship Operations

### 7.1 Create Relationship

```http
POST /v1/relationships
Content-Type: application/json
Authorization: Bearer {token}

{
  "sourceEntityId": "entity-123",
  "targetEntityId": "entity-456",
  "relationshipType": "parent_child",
  "effectiveDate": "2025-12-26T00:00:00Z"
}
```

### 7.2 Get Entity Relationships

```http
GET /v1/{entityType}/{entityId}/relationships
Authorization: Bearer {token}
```

## 8. Data Quality Operations

### 8.1 Validate Entity

```http
POST /v1/{entityType}/validate
Content-Type: application/json
Authorization: Bearer {token}

{
  "attributes": {...}
}
```

**Response (200 OK):**
```json
{
  "isValid": false,
  "violations": [
    {
      "field": "attributes.person.email",
      "rule": "email_format",
      "severity": "error",
      "message": "Invalid email format"
    }
  ]
}
```

### 8.2 Get Quality Metrics

```http
GET /v1/{entityType}/{entityId}/quality
Authorization: Bearer {token}
```

## 9. Change Tracking Operations

### 9.1 Get Change History

```http
GET /v1/{entityType}/{entityId}/history
Authorization: Bearer {token}
```

### 9.2 Subscribe to Change Events

```http
POST /v1/subscriptions
Content-Type: application/json
Authorization: Bearer {token}

{
  "entityType": "customer",
  "eventTypes": ["create", "update", "delete"],
  "webhookUrl": "https://your-app.com/webhooks/mdm"
}
```

## 10. Bulk Operations

### 10.1 Bulk Import

```http
POST /v1/{entityType}/bulk/import
Content-Type: application/json
Authorization: Bearer {token}

{
  "entities": [...]
}
```

### 10.2 Bulk Export

```http
POST /v1/{entityType}/bulk/export
Content-Type: application/json
Authorization: Bearer {token}

{
  "filters": {...},
  "format": "json"
}
```

## 11. Error Handling

### Standard Error Response

```json
{
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Invalid entity data",
    "details": [
      {
        "field": "email",
        "message": "Email format is invalid"
      }
    ],
    "requestId": "req-123abc"
  }
}
```

### HTTP Status Codes

- 200 OK - Success
- 201 Created - Resource created
- 204 No Content - Success, no response body
- 400 Bad Request - Invalid request
- 401 Unauthorized - Authentication required
- 403 Forbidden - Insufficient permissions
- 404 Not Found - Resource not found
- 409 Conflict - Resource conflict
- 422 Unprocessable Entity - Validation failed
- 429 Too Many Requests - Rate limit exceeded
- 500 Internal Server Error - Server error

## 12. Rate Limiting

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1735210800
```

## 13. Pagination

All list endpoints support pagination via query parameters:

```http
GET /v1/customers?page=2&pageSize=50
```

## 14. Versioning

API version in URL path:
- `/v1/` - Current stable version
- `/v2/` - Next major version (breaking changes)

## Summary

This API specification provides comprehensive programmatic access to MDM functionality through a RESTful interface.
