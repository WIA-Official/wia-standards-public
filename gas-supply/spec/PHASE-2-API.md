# WIA-SOC-011: Gas Supply Standard
## PHASE 2: API Interface Specification

**Version:** 1.0  
**Status:** Complete  
**Last Updated:** 2025-12-26

---

## 1. Overview

Phase 2 defines RESTful API interfaces for accessing gas supply system data and controlling equipment. APIs follow OpenAPI 3.1 specification and REST architectural principles.

---

## 2. Base API Structure

### 2.1 Endpoint Base URL

```
https://api.gas-operator.com/wia-soc-011/v1
```

### 2.2 Authentication

All requests require OAuth 2.0 Bearer token:

```http
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

---

## 3. Pipeline Endpoints

### 3.1 List Pipelines

```http
GET /pipelines
Query Parameters:
  - type: string (transmission|distribution|gathering|service)
  - region: string
  - status: string (active|inactive|planned)
  - limit: integer (default: 50, max: 500)
  - offset: integer

Response: 200 OK
{
  "data": [
    {
      "pipelineId": "PL-KR-001-2025",
      "pipelineType": "transmission",
      "length_km": 145.3,
      "diameter_mm": 914,
      "material": {"type": "steel", "grade": "API-5L-X70"},
      "status": "active"
    }
  ],
  "pagination": {
    "total": 156,
    "limit": 50,
    "offset": 0,
    "hasMore": true
  }
}
```

### 3.2 Get Pipeline Details

```http
GET /pipelines/{pipelineId}

Response: 200 OK
{
  "pipelineId": "PL-KR-001-2025",
  "pipelineType": "transmission",
  "geometry": {...},
  "material": {...},
  "pressureRating": {...},
  "installation": {...},
  "measurements": {
    "latest": "/pipelines/PL-KR-001-2025/measurements/latest",
    "history": "/pipelines/PL-KR-001-2025/measurements"
  }
}
```

---

## 4. Operational Data Endpoints

### 4.1 Real-time Measurements

```http
GET /measurements/realtime
Query Parameters:
  - assetIds: string[] (comma-separated)
  - types: string[] (pressure,flow,temperature)
  - since: ISO8601 timestamp

Response: 200 OK
{
  "measurements": [
    {
      "measurementId": "uuid-here",
      "timestamp": "2025-12-26T14:30:00Z",
      "assetId": "PL-KR-001-2025",
      "type": "pressure",
      "value": 75.5,
      "unit": "bar",
      "quality": "good"
    }
  ]
}
```

### 4.2 Historical Time Series

```http
GET /measurements/history
Query Parameters:
  - assetId: string (required)
  - type: string (required)
  - start: ISO8601 timestamp
  - end: ISO8601 timestamp
  - interval: string (1min|5min|1hour|1day)
  - aggregation: string (avg|min|max|sum)

Response: 200 OK
{
  "assetId": "PL-KR-001-2025",
  "type": "pressure",
  "unit": "bar",
  "interval": "5min",
  "data": [
    {"timestamp": "2025-12-26T00:00:00Z", "value": 75.2},
    {"timestamp": "2025-12-26T00:05:00Z", "value": 75.4}
  ]
}
```

---

## 5. Equipment Control Endpoints

### 5.1 Compressor Control

```http
POST /equipment/compressors/{compressorId}/commands
Request Body:
{
  "command": "start|stop|adjust_speed",
  "parameters": {
    "targetSpeed_rpm": 3600,
    "rampRate_rpm_min": 100
  },
  "authorization": {
    "approvedBy": "operator-id",
    "reason": "Increase pipeline pressure"
  }
}

Response: 202 Accepted
{
  "commandId": "CMD-2025-001",
  "status": "pending",
  "estimatedCompletion": "2025-12-26T14:35:00Z",
  "statusUrl": "/commands/CMD-2025-001/status"
}
```

### 5.2 Valve Control

```http
POST /equipment/valves/{valveId}/commands
Request Body:
{
  "command": "open|close|modulate",
  "parameters": {
    "position_percent": 75,
    "rateOfChange_percent_sec": 5
  }
}
```

---

## 6. Gas Quality Endpoints

```http
GET /gas-quality/composition
Query Parameters:
  - location: string (pipelineId or GPS coordinates)
  - timestamp: ISO8601

Response: 200 OK
{
  "compositionId": "COMP-2025-001",
  "timestamp": "2025-12-26T14:30:00Z",
  "location": {...},
  "components": [...],
  "calculatedProperties": {...}
}
```

---

## 7. Event and Alarm Endpoints

### 7.1 Subscribe to Alarms

```http
POST /events/subscriptions
Request Body:
{
  "filters": {
    "severity": ["high", "critical"],
    "categories": ["safety", "environmental"],
    "assets": ["PL-KR-001-2025"]
  },
  "delivery": {
    "method": "webhook",
    "url": "https://client.com/webhooks/alarms",
    "headers": {"X-API-Key": "secret"}
  }
}

Response: 201 Created
{
  "subscriptionId": "SUB-001",
  "status": "active"
}
```

### 7.2 Acknowledge Alarm

```http
POST /events/{eventId}/acknowledge
Request Body:
{
  "acknowledgedBy": "operator-id",
  "notes": "Investigating high pressure reading"
}
```

---

## 8. Batch Operations

```http
POST /batch
Request Body:
{
  "operations": [
    {
      "method": "GET",
      "path": "/pipelines/PL-KR-001-2025/measurements/latest"
    },
    {
      "method": "GET",
      "path": "/pipelines/PL-KR-002-2025/measurements/latest"
    }
  ]
}

Response: 200 OK
{
  "results": [
    {"status": 200, "body": {...}},
    {"status": 200, "body": {...}}
  ]
}
```

---

## 9. WebSocket Real-time Streaming

```javascript
const ws = new WebSocket('wss://api.gas-operator.com/wia-soc-011/v1/stream');

ws.send(JSON.stringify({
  "action": "subscribe",
  "channels": [
    {"type": "measurements", "assetIds": ["PL-KR-001-2025"]},
    {"type": "alarms", "severity": ["high", "critical"]}
  ]
}));

ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log('Received:', data);
};
```

---

## 10. Error Handling

### 10.1 Error Response Format

```json
{
  "error": {
    "code": "INVALID_PARAMETER",
    "message": "Pressure value exceeds pipeline MAOP",
    "details": {
      "field": "pressure_bar",
      "value": 95.0,
      "constraint": "maop_bar",
      "limit": 80.0
    },
    "correlationId": "req-123-456-789"
  }
}
```

### 10.2 HTTP Status Codes

- 200 OK: Success
- 201 Created: Resource created
- 202 Accepted: Command accepted, processing async
- 400 Bad Request: Invalid parameters
- 401 Unauthorized: Missing/invalid authentication
- 403 Forbidden: Insufficient permissions
- 404 Not Found: Resource not found
- 429 Too Many Requests: Rate limit exceeded
- 500 Internal Server Error: Server error
- 503 Service Unavailable: Temporary unavailability

---

## 11. Rate Limiting

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 987
X-RateLimit-Reset: 1640529600
```

Limits:
- Standard: 1000 requests/hour
- Premium: 10000 requests/hour
- Real-time streaming: No limit on WebSocket connections

---

## 12. API Versioning

- URL versioning: `/v1/`, `/v2/`
- Deprecation warnings in headers:
  ```http
  Sunset: Sat, 01 Jan 2027 00:00:00 GMT
  Deprecation: true
  Link: <https://api.gas-operator.com/wia-soc-011/v2/>; rel="successor-version"
  ```

---

**Document Control**
- Version: 1.0
- OpenAPI Spec: https://schemas.wiastandards.com/soc-011/openapi.yaml
- Contact: standards@wiastandards.com

© 2025 World Certification Industry Association | MIT License
