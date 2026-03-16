# WIA-UNI-005 - Phase 2: API Interface

**Version:** 1.0.0  
**Status:** Active  
**Last Updated:** 2025-12-25

## 1. Overview

Phase 2 defines RESTful APIs for programmatic access to infrastructure data and operations. Built on Phase 1 data formats, these APIs enable system integration, automation, and real-time coordination.

## 2. API Architecture

### 2.1 Design Principles

- RESTful architecture following HTTP/1.1 and HTTP/2
- JSON request/response bodies (Phase 1 formats)
- Stateless request handling
- HATEOAS for resource discovery
- Versioned endpoints (`/api/v1/`)

### 2.2 Base URL Structure

```
https://api.infrastructure.wia/{region}/v1/{resource}
```

## 3. Authentication & Authorization

### 3.1 OAuth 2.0 Implementation

All API requests require OAuth 2.0 authentication:

```http
Authorization: Bearer {access_token}
```

**Supported Flows:**
- Client Credentials (system-to-system)
- Authorization Code (user delegation)
- Device Code (IoT devices)

### 3.2 Token Format (JWT)

```json
{
  "iss": "https://auth.wia",
  "sub": "system:infrastructure-mgmt",
  "aud": "api.infrastructure.wia",
  "exp": 1735257600,
  "iat": 1735171200,
  "scope": "projects:read assets:write monitoring:read"
}
```

## 4. Project Management API

### 4.1 Create Project

```http
POST /api/v1/projects
Content-Type: application/json
Authorization: Bearer {token}

{
  // Phase 1 InfrastructureProject schema
}
```

**Response:** `201 Created`
```json
{
  "id": "wia:uni:proj:2025-001",
  "status": "created",
  "_links": {
    "self": "/api/v1/projects/2025-001",
    "assets": "/api/v1/projects/2025-001/assets"
  }
}
```

### 4.2 Query Projects

```http
GET /api/v1/projects?type=railway&status=active&limit=50&offset=0
```

**Query Parameters:**
| Parameter | Type | Description |
|-----------|------|-------------|
| type | string | Filter by project type |
| status | string | Filter by status |
| region | string | Geographic filter |
| limit | integer | Results per page (max 100) |
| offset | integer | Pagination offset |

## 5. Asset Registry API

### 5.1 Register Asset

```http
POST /api/v1/assets
Content-Type: application/json

{
  // Phase 1 InfrastructureAsset schema
}
```

### 5.2 Update Asset Status

```http
PATCH /api/v1/assets/{assetId}/status
Content-Type: application/json

{
  "operationalStatus": "maintenance",
  "reason": "Scheduled inspection",
  "estimatedDuration": "PT4H"
}
```

## 6. Real-time Monitoring API

### 6.1 Stream Sensor Data

```http
GET /api/v1/monitoring/sensors/{sensorId}/stream
Accept: text/event-stream
```

**Server-Sent Events:**
```
event: reading
data: {"sensorId":"...", "value":24.5, "timestamp":"..."}

event: reading
data: {"sensorId":"...", "value":24.7, "timestamp":"..."}
```

### 6.2 Query Historical Data

```http
GET /api/v1/monitoring/sensors/{sensorId}/history
  ?start=2025-12-01T00:00:00Z
  &end=2025-12-25T23:59:59Z
  &aggregation=1h
```

## 7. Maintenance API

### 7.1 Schedule Maintenance

```http
POST /api/v1/maintenance/schedules
Content-Type: application/json

{
  "assetId": "wia:uni:asset:bridge-001",
  "scheduledDate": "2025-12-30T09:00:00Z",
  "maintenanceType": "inspection",
  "assignedTeam": "team-alpha"
}
```

### 7.2 Record Maintenance

```http
POST /api/v1/maintenance/records
Content-Type: application/json

{
  // Phase 1 MaintenanceRecord schema
}
```

## 8. Error Handling

### 8.1 Error Response Format

```json
{
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Invalid project data",
    "details": [
      {
        "field": "budget.total",
        "issue": "Must be a positive number"
      }
    ],
    "timestamp": "2025-12-25T10:30:00Z",
    "requestId": "req-12345"
  }
}
```

### 8.2 Standard Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| VALIDATION_ERROR | 400 | Invalid request data |
| AUTHENTICATION_REQUIRED | 401 | Missing/invalid token |
| FORBIDDEN | 403 | Insufficient permissions |
| NOT_FOUND | 404 | Resource not found |
| CONFLICT | 409 | Resource conflict |
| RATE_LIMIT_EXCEEDED | 429 | Too many requests |
| INTERNAL_ERROR | 500 | Server error |

## 9. Rate Limiting

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 987
X-RateLimit-Reset: 1735174800
```

**Limits:**
- Standard tier: 1,000 req/hour
- Premium tier: 10,000 req/hour
- Real-time monitoring: 100 req/second

## 10. Webhooks

### 10.1 Event Subscription

```http
POST /api/v1/webhooks
Content-Type: application/json

{
  "url": "https://your-system.com/webhook",
  "events": ["asset.status.changed", "sensor.alert"],
  "secret": "webhook-secret-key"
}
```

### 10.2 Event Delivery

```http
POST {subscriber_url}
Content-Type: application/json
X-WIA-Event: asset.status.changed
X-WIA-Signature: sha256={hmac}

{
  "event": "asset.status.changed",
  "timestamp": "2025-12-25T10:30:00Z",
  "data": {
    "assetId": "wia:uni:asset:bridge-001",
    "oldStatus": "operational",
    "newStatus": "maintenance"
  }
}
```

---

**© 2025 SmileStory Inc. / WIA**  
**弘益人間 (홍익인간) · Benefit All Humanity**
