# Exoskeleton — Phase 2: API Interface Specification

> **Version:** 1.0.0
> **Status:** Official
> **Last Updated:** 2025-01-01
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 2 defines RESTful API endpoints for interacting with Exoskeleton services. This specification enables standardized programmatic access to biomechanical data across exoskeleton control systems.

### 1.1 Base URL

```
Production: https://api.exoskeleton.wia.org/v1
Sandbox:    https://sandbox.exoskeleton.wia.org/v1
```

---

## 2. Authentication

### 2.1 OAuth 2.0

| Grant Type | Use Case |
|------------|----------|
| Authorization Code | Web applications |
| Client Credentials | Server-to-server |
| Device Code | IoT and embedded devices |

### 2.2 API Keys

For sandbox access, API keys may be used via the `X-WIA-API-Key` header.

---

## 3. Endpoints

### 3.1 Resource Management

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | /resources | List resources with pagination |
| POST | /resources | Create new resource |
| GET | /resources/{id} | Retrieve specific resource |
| PUT | /resources/{id} | Update resource |
| DELETE | /resources/{id} | Remove resource |

### 3.2 Data Operations

| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | /data/ingest | Submit biomechanical data |
| GET | /data/query | Query with filters |
| POST | /data/export | Export data in specified format |
| GET | /data/stream | Server-Sent Events stream |

### 3.3 Analytics

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | /analytics/summary | Aggregated summary |
| GET | /analytics/trends | Trend analysis |
| POST | /analytics/report | Generate custom report |

---

## 4. Request/Response Format

### 4.1 Standard Response Envelope

```json
{
  "status": "success",
  "data": {},
  "meta": {
    "page": 1,
    "per_page": 20,
    "total": 100,
    "timestamp": "2025-01-15T10:30:00Z"
  }
}
```

### 4.2 Pagination

Use cursor-based pagination for large datasets:
- `?cursor=<token>&limit=20`

---

## 5. Error Handling

### 5.1 Error Response Format

```json
{
  "status": "error",
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Invalid input parameters",
    "details": [
      {"field": "timestamp", "issue": "Format must be ISO 8601"}
    ]
  }
}
```

### 5.2 Error Codes

| HTTP Status | Code | Description |
|-------------|------|-------------|
| 400 | VALIDATION_ERROR | Invalid request parameters |
| 401 | UNAUTHORIZED | Missing or invalid credentials |
| 403 | FORBIDDEN | Insufficient permissions |
| 404 | NOT_FOUND | Resource not found |
| 409 | CONFLICT | Resource conflict |
| 429 | RATE_LIMITED | Too many requests |
| 500 | INTERNAL_ERROR | Server error |

---

## 6. Rate Limiting

| Tier | Requests/min | Burst |
|------|-------------|-------|
| Free | 60 | 10 |
| Standard | 600 | 100 |
| Enterprise | 6000 | 1000 |

Rate limit headers: `X-RateLimit-Limit`, `X-RateLimit-Remaining`, `X-RateLimit-Reset`

---

## 7. Webhooks

### 7.1 Event Types

| Event | Description |
|-------|-------------|
| `resource.created` | New resource created |
| `resource.updated` | Resource modified |
| `data.processed` | Data processing complete |
| `alert.triggered` | Alert condition met |

### 7.2 Webhook Payload

```json
{
  "event": "resource.created",
  "timestamp": "2025-01-15T10:30:00Z",
  "data": {},
  "signature": "sha256=..."
}
```

---

**© 2025 SmileStory Inc. / WIA - World Certification Industry Association**
**弘益人間 · Benefit All Humanity**
