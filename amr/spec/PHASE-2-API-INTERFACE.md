# WIA-AMR Phase 2: API Interface Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-01-15

---

## 1. Overview

Phase 2 of the WIA-AMR standard defines the RESTful API interfaces for communication between AMR systems, Fleet Management Systems (FMS), and enterprise applications. This specification covers endpoint design, authentication, error handling, and pagination.

### 1.1 Design Principles

- **REST Architecture**: Resource-oriented design following REST constraints
- **JSON**: All request/response bodies use JSON format
- **Versioning**: API versioning through URL path (/api/v1/)
- **Security**: OAuth 2.0 and API Key authentication

### 1.2 Base URL

All API endpoints are prefixed with the base URL:

```
https://{host}/api/v1/
```

---

## 2. Authentication

### 2.1 OAuth 2.0

WIA-AMR supports OAuth 2.0 for secure authentication, particularly the Client Credentials grant for service-to-service communication.

#### Token Request

```http
POST /oauth/token HTTP/1.1
Host: auth.example.com
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id=your-client-id
&client_secret=your-client-secret
&scope=robots:read tasks:write
```

#### Token Response

```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIs...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "robots:read tasks:write"
}
```

#### Using the Token

```http
GET /api/v1/robots HTTP/1.1
Host: api.example.com
Authorization: Bearer eyJhbGciOiJSUzI1NiIs...
```

### 2.2 API Key

For simpler authentication scenarios:

```http
GET /api/v1/robots HTTP/1.1
Host: api.example.com
X-API-Key: wia_amr_key_abc123def456
```

### 2.3 Scopes

| Scope | Description |
|-------|-------------|
| robots:read | Read robot information |
| robots:write | Control robots |
| tasks:read | Read task information |
| tasks:write | Create/modify tasks |
| maps:read | Read map data |
| maps:write | Upload/modify maps |
| admin | Full administrative access |

---

## 3. Robot Management API

### 3.1 List Robots

```http
GET /api/v1/robots
```

**Query Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| status | string | Filter by operating state |
| limit | integer | Max results (default: 20, max: 100) |
| offset | integer | Pagination offset |
| sort | string | Sort field (prefix - for descending) |

**Response:**

```json
{
  "data": [
    {
      "robotId": "amr-001",
      "name": "Warehouse Robot 1",
      "manufacturer": "WIA Robotics",
      "model": "WIA-AMR-500",
      "status": "IDLE",
      "position": { "x": 10.5, "y": 20.3, "theta": 0 },
      "batteryLevel": 85,
      "lastSeen": "2025-01-15T10:30:00Z"
    }
  ],
  "pagination": {
    "total": 50,
    "limit": 20,
    "offset": 0,
    "hasMore": true
  }
}
```

### 3.2 Get Robot Details

```http
GET /api/v1/robots/{robotId}
```

**Response:**

```json
{
  "robotId": "amr-001",
  "name": "Warehouse Robot 1",
  "manufacturer": "WIA Robotics",
  "model": "WIA-AMR-500",
  "serialNumber": "WIA-2024-001234",
  "softwareVersion": "2.3.1",
  "capabilities": ["LIFT", "SCAN", "DOCK"],
  "maxPayload": 500,
  "dimensions": {
    "length": 0.8,
    "width": 0.6,
    "height": 0.4
  },
  "status": "IDLE",
  "position": { "x": 10.5, "y": 20.3, "theta": 0, "mapId": "warehouse-01" },
  "battery": {
    "level": 85,
    "voltage": 48.2,
    "charging": false
  },
  "currentTaskId": null,
  "registeredAt": "2024-06-15T08:00:00Z",
  "lastSeen": "2025-01-15T10:30:00Z"
}
```

### 3.3 Get Robot State

```http
GET /api/v1/robots/{robotId}/state
```

Returns real-time state conforming to the RobotState schema.

### 3.4 Send Command to Robot

```http
POST /api/v1/robots/{robotId}/commands
```

**Request:**

```json
{
  "commandType": "NAVIGATE",
  "destination": {
    "x": 50.0,
    "y": 30.0,
    "theta": 1.57
  },
  "parameters": {
    "maxSpeed": 1.5,
    "avoidanceMode": "DYNAMIC"
  }
}
```

**Response:**

```json
{
  "commandId": "cmd-12345",
  "status": "ACCEPTED",
  "estimatedDuration": 45,
  "message": "Command accepted"
}
```

**Command Types:**

| Type | Description |
|------|-------------|
| NAVIGATE | Move to destination |
| PAUSE | Pause current operation |
| RESUME | Resume paused operation |
| STOP | Stop immediately |
| EMERGENCY_STOP | Emergency stop |
| DOCK | Dock at charging station |
| UNDOCK | Undock from station |

---

## 4. Task Management API

### 4.1 Create Task

```http
POST /api/v1/tasks
```

**Request:**

```json
{
  "taskType": "PICK_AND_TRANSPORT",
  "priority": 80,
  "source": {
    "locationId": "RACK-A01-02-03",
    "coordinates": { "x": 45.5, "y": 28.3, "theta": 0 }
  },
  "destination": {
    "locationId": "PACK-STATION-01",
    "coordinates": { "x": 100.0, "y": 50.0, "theta": 1.57 }
  },
  "actions": [
    {
      "actionType": "PICK",
      "parameters": {
        "itemId": "SKU-12345",
        "quantity": 3
      }
    }
  ],
  "constraints": {
    "deadline": "2025-01-15T12:00:00Z",
    "requiredCapabilities": ["LIFT"]
  },
  "externalRef": {
    "system": "SAP_EWM",
    "orderId": "4500012345"
  }
}
```

**Response:**

```json
{
  "taskId": "task-67890",
  "status": "PENDING",
  "createdAt": "2025-01-15T10:30:00Z",
  "estimatedStartTime": "2025-01-15T10:32:00Z"
}
```

### 4.2 Get Task

```http
GET /api/v1/tasks/{taskId}
```

### 4.3 Update Task

```http
PUT /api/v1/tasks/{taskId}
```

Only allowed when task status is PENDING.

### 4.4 Cancel Task

```http
DELETE /api/v1/tasks/{taskId}
```

**Response:**

```json
{
  "taskId": "task-67890",
  "status": "CANCELLED",
  "cancelledAt": "2025-01-15T10:35:00Z",
  "reason": "User cancelled"
}
```

### 4.5 List Tasks

```http
GET /api/v1/tasks
```

**Query Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| status | string | Filter by status (comma-separated) |
| robotId | string | Filter by assigned robot |
| taskType | string | Filter by task type |
| createdAt[gte] | datetime | Created after |
| createdAt[lt] | datetime | Created before |
| priority[gte] | integer | Minimum priority |
| sort | string | Sort field |
| limit | integer | Max results |
| offset | integer | Pagination offset |

---

## 5. Map Management API

### 5.1 List Maps

```http
GET /api/v1/maps
```

### 5.2 Get Map

```http
GET /api/v1/maps/{mapId}
```

### 5.3 Upload Map

```http
POST /api/v1/maps
Content-Type: multipart/form-data
```

### 5.4 Get Map Nodes

```http
GET /api/v1/maps/{mapId}/nodes
```

### 5.5 Get Map Edges

```http
GET /api/v1/maps/{mapId}/edges
```

### 5.6 Get Map Zones

```http
GET /api/v1/maps/{mapId}/zones
```

---

## 6. Fleet Management API

### 6.1 Fleet Summary

```http
GET /api/v1/fleet/summary
```

**Response:**

```json
{
  "timestamp": "2025-01-15T10:30:00Z",
  "totalRobots": 50,
  "robotsByStatus": {
    "IDLE": 15,
    "NAVIGATING": 28,
    "CHARGING": 5,
    "ERROR": 2
  },
  "utilizationRate": 0.90,
  "tasksInProgress": 42,
  "tasksPending": 15,
  "tasksCompletedToday": 523
}
```

### 6.2 Fleet Alerts

```http
GET /api/v1/fleet/alerts
```

---

## 7. Error Handling

### 7.1 HTTP Status Codes

| Code | Meaning | Usage |
|------|---------|-------|
| 200 | OK | Successful GET, PUT, PATCH |
| 201 | Created | Successful POST |
| 204 | No Content | Successful DELETE |
| 400 | Bad Request | Invalid request format |
| 401 | Unauthorized | Authentication failed |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource not found |
| 409 | Conflict | State conflict |
| 422 | Unprocessable Entity | Validation failed |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server error |
| 503 | Service Unavailable | Service down |

### 7.2 Error Response Format

```json
{
  "error": {
    "code": "ROBOT_NOT_FOUND",
    "message": "Robot 'amr-999' not found",
    "details": {
      "robotId": "amr-999",
      "suggestion": "Check robot ID or list available robots at /api/v1/robots"
    },
    "requestId": "req-abc123",
    "timestamp": "2025-01-15T10:30:00Z"
  }
}
```

### 7.3 Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| VALIDATION_ERROR | 400 | Request validation failed |
| UNAUTHORIZED | 401 | Authentication required |
| FORBIDDEN | 403 | Insufficient permissions |
| ROBOT_NOT_FOUND | 404 | Robot not found |
| TASK_NOT_FOUND | 404 | Task not found |
| MAP_NOT_FOUND | 404 | Map not found |
| ROBOT_BUSY | 409 | Robot is busy |
| TASK_ALREADY_ASSIGNED | 409 | Task already assigned |
| RATE_LIMIT_EXCEEDED | 429 | Too many requests |
| INTERNAL_ERROR | 500 | Internal server error |

---

## 8. Pagination

### 8.1 Offset-Based Pagination

```http
GET /api/v1/tasks?limit=20&offset=40
```

**Response:**

```json
{
  "data": [...],
  "pagination": {
    "total": 150,
    "limit": 20,
    "offset": 40,
    "hasMore": true,
    "links": {
      "first": "/api/v1/tasks?limit=20&offset=0",
      "prev": "/api/v1/tasks?limit=20&offset=20",
      "next": "/api/v1/tasks?limit=20&offset=60",
      "last": "/api/v1/tasks?limit=20&offset=140"
    }
  }
}
```

### 8.2 Cursor-Based Pagination (Optional)

For high-frequency data:

```http
GET /api/v1/robots/{robotId}/telemetry?cursor=abc123&limit=100
```

---

## 9. Rate Limiting

### 9.1 Rate Limit Headers

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 950
X-RateLimit-Reset: 1705316400
```

### 9.2 Default Limits

| Endpoint | Limit |
|----------|-------|
| General | 1000/hour |
| Robot commands | 100/minute |
| Bulk operations | 10/minute |

---

## 10. OpenAPI Specification

Full OpenAPI 3.0 specification is available at:

```
GET /api/v1/openapi.json
GET /api/v1/openapi.yaml
```

Interactive documentation:

```
GET /api/v1/docs
```

---

## 11. Webhooks (Optional)

### 11.1 Register Webhook

```http
POST /api/v1/webhooks
```

**Request:**

```json
{
  "url": "https://your-server.com/webhooks/wia-amr",
  "events": ["task.completed", "robot.error"],
  "secret": "your-webhook-secret"
}
```

### 11.2 Webhook Payload

```json
{
  "eventId": "evt-12345",
  "eventType": "task.completed",
  "timestamp": "2025-01-15T10:30:00Z",
  "data": {
    "taskId": "task-67890",
    "robotId": "amr-001",
    "completedAt": "2025-01-15T10:30:00Z"
  }
}
```

---

## 12. References

- OpenAPI Specification: https://spec.openapis.org/oas/latest.html
- OAuth 2.0: https://oauth.net/2/
- JSON API: https://jsonapi.org/

---

© 2025 WIA Standards | 弘益人間 · Benefit All Humanity
