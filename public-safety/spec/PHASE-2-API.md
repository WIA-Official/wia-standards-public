# WIA-SOC-004 Phase 2: API Interface Specification

**Version:** 1.0.0
**Status:** Approved
**Last Updated:** 2025-12-26

弘익人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Overview

Phase 2 defines the REST API and WebSocket interfaces for public safety systems, enabling standardized communication between emergency dispatch centers, first responders, and integrated systems.

## 2. Base API Structure

### 2.1 Endpoint Conventions

**Base URL:** `https://api.publicsafety.example.com/v1`

All endpoints follow REST principles:
- GET: Retrieve resources
- POST: Create resources
- PUT: Update entire resources
- PATCH: Partial update
- DELETE: Remove resources

### 2.2 Authentication

All API requests MUST include authentication via one of:

1. **API Key** (header): `X-API-Key: your-api-key`
2. **OAuth 2.0** (header): `Authorization: Bearer token`
3. **JWT** (header): `Authorization: Bearer jwt-token`

### 2.3 Rate Limiting

- Standard tier: 1000 requests/hour
- Emergency tier: Unlimited during active incidents
- Headers included in all responses:
  - `X-RateLimit-Limit: 1000`
  - `X-RateLimit-Remaining: 999`
  - `X-RateLimit-Reset: 1640534400`

## 3. Emergency Incident Endpoints

### 3.1 Report New Incident

**POST** `/incidents`

Request:
```json
{
  "incidentType": "medical",
  "priority": "critical",
  "location": {
    "latitude": 37.7749,
    "longitude": -122.4194,
    "address": "123 Market St, SF, CA"
  },
  "caller": {
    "phoneNumber": "+14155551234",
    "language": "en"
  },
  "description": "Chest pain, difficulty breathing"
}
```

Response (201 Created):
```json
{
  "incidentId": "INC-2025-0042",
  "status": "reported",
  "createdAt": "2025-12-26T14:23:15Z",
  "estimatedResponse": {
    "units": 2,
    "eta": 240
  }
}
```

### 3.2 Get Incident Details

**GET** `/incidents/{incidentId}`

Response (200 OK):
```json
{
  "incidentId": "INC-2025-0042",
  "incidentType": "medical",
  "priority": "critical",
  "status": "responded",
  "location": {...},
  "units": [
    {
      "unitId": "AMB-101",
      "status": "on_scene",
      "arrivedAt": "2025-12-26T14:27:30Z"
    }
  ],
  "timeline": [
    {"timestamp": "2025-12-26T14:23:15Z", "event": "reported"},
    {"timestamp": "2025-12-26T14:24:00Z", "event": "dispatched"},
    {"timestamp": "2025-12-26T14:27:30Z", "event": "arrived"}
  ]
}
```

### 3.3 Update Incident Status

**PATCH** `/incidents/{incidentId}/status`

Request:
```json
{
  "status": "resolved",
  "resolution": "Patient transported to SF General Hospital",
  "resolvedBy": "UNIT-AMB-101"
}
```

Response (200 OK):
```json
{
  "incidentId": "INC-2025-0042",
  "status": "resolved",
  "updatedAt": "2025-12-26T14:45:00Z"
}
```

### 3.4 List Active Incidents

**GET** `/incidents?status=active&priority=critical`

Query Parameters:
- `status`: reported|dispatched|responded|active|resolved
- `priority`: low|medium|high|critical
- `type`: fire|medical|police|disaster
- `limit`: integer (default 50, max 100)
- `offset`: integer (default 0)

Response (200 OK):
```json
{
  "total": 12,
  "limit": 50,
  "offset": 0,
  "incidents": [...]
}
```

## 4. First Responder Unit Endpoints

### 4.1 Get Unit Status

**GET** `/units/{unitId}`

Response (200 OK):
```json
{
  "unitId": "AMB-101",
  "unitType": "ambulance",
  "status": "available",
  "location": {
    "latitude": 37.7749,
    "longitude": -122.4194,
    "heading": 180,
    "speed": 0
  },
  "crew": [...],
  "lastUpdated": "2025-12-26T14:50:00Z"
}
```

### 4.2 Dispatch Unit

**POST** `/dispatch`

Request:
```json
{
  "incidentId": "INC-2025-0042",
  "units": ["AMB-101", "AMB-102"],
  "priority": "critical",
  "instructions": "Code 3 response required"
}
```

Response (200 OK):
```json
{
  "dispatchId": "DISP-2025-0123",
  "units": [
    {
      "unitId": "AMB-101",
      "status": "dispatched",
      "eta": 240,
      "route": {
        "distance": 3.2,
        "duration": 240
      }
    }
  ]
}
```

### 4.3 Update Unit Location

**PUT** `/units/{unitId}/location`

Request:
```json
{
  "latitude": 37.7750,
  "longitude": -122.4195,
  "heading": 90,
  "speed": 45.5,
  "timestamp": "2025-12-26T14:51:00Z"
}
```

Response (204 No Content)

### 4.4 List Available Units

**GET** `/units?status=available&type=ambulance&radius=5000&lat=37.7749&lng=-122.4194`

Query Parameters:
- `status`: available|dispatched|on_scene|unavailable
- `type`: fire|ambulance|police|rescue
- `radius`: meters
- `lat`, `lng`: center point for radius search

## 5. Alert Notification Endpoints

### 5.1 Issue Alert

**POST** `/alerts`

Request:
```json
{
  "alertType": "earthquake",
  "priority": "critical",
  "affectedArea": {
    "type": "circle",
    "center": [37.7749, -122.4194],
    "radius": 50000
  },
  "message": {
    "headline": "Earthquake Alert",
    "description": "Magnitude 6.5 earthquake detected. Take cover immediately.",
    "instruction": "Drop, Cover, and Hold On"
  },
  "expiresAt": "2025-12-26T15:00:00Z"
}
```

Response (201 Created):
```json
{
  "alertId": "ALERT-2025-0042",
  "issuedAt": "2025-12-26T14:30:00Z",
  "estimatedReach": 1500000
}
```

### 5.2 Get Alert Details

**GET** `/alerts/{alertId}`

### 5.3 Cancel Alert

**DELETE** `/alerts/{alertId}`

### 5.4 List Active Alerts

**GET** `/alerts?status=active&priority=critical`

## 6. Communication Endpoints

### 6.1 Initiate Communication

**POST** `/communications`

Request:
```json
{
  "incidentId": "INC-2025-0042",
  "channel": "radio",
  "participants": ["DISP-01", "UNIT-AMB-101"],
  "priority": "emergency"
}
```

### 6.2 Get Communication Log

**GET** `/communications/{logId}`

### 6.3 List Communications for Incident

**GET** `/incidents/{incidentId}/communications`

## 7. Analytics Endpoints

### 7.1 Response Time Statistics

**GET** `/analytics/response-times?from=2025-01-01&to=2025-12-31&type=medical`

Response (200 OK):
```json
{
  "period": {
    "from": "2025-01-01T00:00:00Z",
    "to": "2025-12-31T23:59:59Z"
  },
  "statistics": {
    "average": 312,
    "median": 280,
    "p95": 420,
    "min": 120,
    "max": 900
  },
  "byPriority": {
    "critical": {"average": 240, "median": 220},
    "high": {"average": 300, "median": 280}
  }
}
```

### 7.2 Incident Trends

**GET** `/analytics/trends?granularity=daily&from=2025-12-01`

### 7.3 Unit Utilization

**GET** `/analytics/units/utilization?type=ambulance`

## 8. WebSocket Interface

### 8.1 Connection

**WebSocket URL:** `wss://api.publicsafety.example.com/v1/ws`

Authentication via query parameter or initial message:
```
wss://api.publicsafety.example.com/v1/ws?token=jwt-token
```

### 8.2 Subscribe to Events

Client Message:
```json
{
  "action": "subscribe",
  "topics": ["incidents.*", "units.AMB-101.*", "alerts.critical"]
}
```

Server Response:
```json
{
  "status": "subscribed",
  "topics": ["incidents.*", "units.AMB-101.*", "alerts.critical"]
}
```

### 8.3 Event Notifications

Server Message (Incident Update):
```json
{
  "event": "incident.status_changed",
  "timestamp": "2025-12-26T14:30:00Z",
  "data": {
    "incidentId": "INC-2025-0042",
    "previousStatus": "reported",
    "newStatus": "dispatched"
  }
}
```

Server Message (Unit Location Update):
```json
{
  "event": "unit.location_updated",
  "timestamp": "2025-12-26T14:30:05Z",
  "data": {
    "unitId": "AMB-101",
    "location": {
      "latitude": 37.7750,
      "longitude": -122.4195
    }
  }
}
```

## 9. Error Handling

All errors follow standard HTTP status codes with JSON body:

```json
{
  "error": {
    "code": "INCIDENT_NOT_FOUND",
    "message": "Incident INC-2025-0042 not found",
    "details": {
      "incidentId": "INC-2025-0042"
    },
    "timestamp": "2025-12-26T14:30:00Z"
  }
}
```

Common Error Codes:
- 400 Bad Request: `INVALID_REQUEST`, `VALIDATION_ERROR`
- 401 Unauthorized: `AUTHENTICATION_REQUIRED`, `INVALID_TOKEN`
- 403 Forbidden: `INSUFFICIENT_PERMISSIONS`
- 404 Not Found: `RESOURCE_NOT_FOUND`
- 409 Conflict: `RESOURCE_CONFLICT`, `DUPLICATE_INCIDENT`
- 429 Too Many Requests: `RATE_LIMIT_EXCEEDED`
- 500 Internal Server Error: `INTERNAL_ERROR`
- 503 Service Unavailable: `SERVICE_UNAVAILABLE`, `MAINTENANCE_MODE`

## 10. Pagination

For list endpoints, use standard pagination:

Request:
```
GET /incidents?limit=50&offset=100
```

Response Headers:
```
Link: <https://api.publicsafety.example.com/v1/incidents?limit=50&offset=150>; rel="next",
      <https://api.publicsafety.example.com/v1/incidents?limit=50&offset=50>; rel="prev"
X-Total-Count: 1250
```

---

© 2025 WIA · MIT License
