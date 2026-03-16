# PHASE 2: API Specification

**WIA-SOC-010 Electricity Grid Standard**
Version: 1.0
Status: Draft
Last Updated: 2025-12-26

---

## 1. Overview

This document specifies the RESTful API and WebSocket interfaces for the WIA-SOC-010 Electricity Grid Standard. All implementations MUST support these endpoints to enable interoperability.

## 2. Base URL Structure

```
https://{host}/api/v1/{resource}
```

- `{host}`: Grid operator's API endpoint
- `{resource}`: Specific resource (grid-status, renewables, storage, etc.)

## 3. Authentication

### 3.1 OAuth 2.0 + JWT

All API requests MUST be authenticated using OAuth 2.0 with JWT tokens.

**Token Request:**
```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id={client_id}
&client_secret={client_secret}
&scope=grid:read grid:write
```

**Token Response:**
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "grid:read grid:write"
}
```

**Using Token:**
```http
GET /api/v1/grid-status
Authorization: Bearer {access_token}
```

### 3.2 API Keys (Alternative)

For simpler integrations, API keys MAY be supported:
```http
GET /api/v1/grid-status
X-API-Key: {api_key}
```

## 4. Core API Endpoints

### 4.1 System Information

**GET /api/v1/system/info**

Returns grid operator system information.

**Response:**
```json
{
  "@context": "https://wia-official.org/standards/soc-010/context.jsonld",
  "@type": "SystemInfo",
  "gridOperator": {
    "name": "Metropolitan Power Grid",
    "id": "mpg-001",
    "region": "Northeast",
    "timezone": "America/New_York"
  },
  "capabilities": {
    "renewableIntegration": true,
    "energyStorage": true,
    "demandResponse": true,
    "synchrophasors": true
  },
  "version": "1.0.0",
  "status": "operational"
}
```

### 4.2 Grid Status

**GET /api/v1/grid-status**

Returns current grid operational status.

**Query Parameters:**
- `zone` (optional): Filter by zone ID
- `detail` (optional): Level of detail (basic|full)

**Response:**
```json
{
  "@type": "GridStatus",
  "timestamp": "2025-12-26T14:30:00Z",
  "frequency": {
    "value": 60.02,
    "unit": "Hz",
    "deviation": 0.02
  },
  "currentLoad": {
    "value": 2450,
    "unit": "MW"
  },
  "capacity": {
    "total": 3500,
    "available": 3200,
    "unit": "MW"
  },
  "loadFactor": 0.70,
  "status": "normal"
}
```

### 4.3 Renewable Energy

**GET /api/v1/renewables/current**

Returns current renewable generation data.

**Query Parameters:**
- `source`: Filter by type (solar|wind|hydro|all)
- `zone`: Filter by zone ID

**Response:**
```json
{
  "@type": "RenewableGeneration",
  "timestamp": "2025-12-26T14:30:00Z",
  "sources": [
    {
      "type": "solar",
      "generation": 580,
      "capacity": 800,
      "unit": "MW"
    },
    {
      "type": "wind",
      "generation": 450,
      "capacity": 600,
      "unit": "MW"
    }
  ],
  "totalGeneration": 1030,
  "penetrationRate": 0.42
}
```

**GET /api/v1/renewables/forecast**

Returns renewable energy forecast.

**Query Parameters:**
- `horizon`: Forecast horizon (1h|6h|24h|7d)
- `source`: Energy source type
- `resolution`: Time resolution (15min|1h)

**Response:**
```json
{
  "@type": "RenewableForecast",
  "generatedAt": "2025-12-26T14:30:00Z",
  "horizon": "24h",
  "resolution": "1h",
  "predictions": [
    {
      "timestamp": "2025-12-26T15:00:00Z",
      "solar": 520,
      "wind": 480,
      "total": 1000,
      "confidence": 0.85
    }
  ]
}
```

### 4.4 Energy Storage

**GET /api/v1/storage/status**

Returns energy storage system status.

**Response:**
```json
{
  "@type": "StorageStatus",
  "timestamp": "2025-12-26T14:30:00Z",
  "systems": [
    {
      "id": "ess-001",
      "technology": "lithium-ion",
      "capacity": {
        "power": 100,
        "energy": 400,
        "unit": "MW/MWh"
      },
      "stateOfCharge": 78,
      "powerFlow": 25,
      "direction": "charging"
    }
  ],
  "totalCapacity": 500,
  "totalStored": 390,
  "aggregatePower": 120
}
```

**POST /api/v1/storage/dispatch**

Dispatch storage system charging/discharging.

**Request:**
```json
{
  "storageId": "ess-001",
  "power": -50,
  "duration": "PT2H",
  "priority": "high"
}
```

**Response:**
```json
{
  "dispatchId": "dispatch-12345",
  "status": "accepted",
  "scheduledStart": "2025-12-26T15:00:00Z",
  "estimatedRevenue": 2500
}
```

### 4.5 Demand Response

**GET /api/v1/demand-response/events**

Returns active and scheduled DR events.

**Query Parameters:**
- `status`: Filter by status (scheduled|active|completed)
- `from`: Start date (ISO 8601)
- `to`: End date (ISO 8601)

**Response:**
```json
{
  "@type": "DemandResponseEvents",
  "events": [
    {
      "id": "dr-event-001",
      "programType": "critical-peak-pricing",
      "startTime": "2025-12-26T18:00:00Z",
      "endTime": "2025-12-26T21:00:00Z",
      "targetReduction": 150,
      "currentReduction": 185,
      "participants": {
        "active": 12450
      },
      "status": "active"
    }
  ]
}
```

**POST /api/v1/demand-response/events**

Create new demand response event.

**Request:**
```json
{
  "programType": "dynamic-pricing",
  "startTime": "2025-12-27T17:00:00Z",
  "endTime": "2025-12-27T20:00:00Z",
  "targetReduction": 200,
  "pricing": {
    "event": 0.45,
    "currency": "USD"
  }
}
```

### 4.6 Power Quality

**GET /api/v1/power-quality/current**

Returns current power quality metrics.

**Query Parameters:**
- `location`: Substation/feeder ID
- `metrics`: Specific metrics (frequency|voltage|harmonics|all)

**Response:**
```json
{
  "@type": "PowerQualityMetrics",
  "timestamp": "2025-12-26T14:30:00Z",
  "location": "sub-001",
  "frequency": {
    "nominal": 60.0,
    "actual": 60.02,
    "deviation": 0.02
  },
  "voltage": {
    "nominal": 115.0,
    "actual": 115.2
  },
  "harmonics": {
    "thd": 2.1
  },
  "powerFactor": 0.95
}
```

### 4.7 Smart Meters

**GET /api/v1/meters/{meterId}/consumption**

Returns smart meter consumption data.

**Query Parameters:**
- `from`: Start timestamp
- `to`: End timestamp
- `interval`: Data interval (15min|1h|1d)

**Response:**
```json
{
  "@type": "MeterConsumption",
  "meterId": "meter-123456",
  "interval": "15min",
  "data": [
    {
      "timestamp": "2025-12-26T14:00:00Z",
      "energy": 3.2,
      "demand": 12.8,
      "unit": "kWh/kW"
    }
  ]
}
```

### 4.8 Alerts

**GET /api/v1/alerts**

Returns system alerts and notifications.

**Query Parameters:**
- `severity`: Filter by severity (info|warning|critical)
- `category`: Filter by category
- `status`: Filter by status (new|acknowledged|resolved)

**Response:**
```json
{
  "@type": "AlertList",
  "alerts": [
    {
      "id": "alert-001",
      "severity": "warning",
      "category": "grid-stability",
      "message": "Frequency deviation exceeding threshold",
      "timestamp": "2025-12-26T14:25:00Z",
      "status": "new"
    }
  ]
}
```

**POST /api/v1/alerts/{alertId}/acknowledge**

Acknowledge an alert.

## 5. WebSocket API

### 5.1 Real-Time Event Stream

**Connection:**
```
wss://{host}/api/v1/stream
```

**Authentication:**
```json
{
  "type": "auth",
  "token": "Bearer {access_token}"
}
```

**Subscribe:**
```json
{
  "type": "subscribe",
  "channels": [
    "grid-status",
    "renewables",
    "storage",
    "alerts",
    "demand-response"
  ]
}
```

**Event Message:**
```json
{
  "channel": "grid-status",
  "timestamp": "2025-12-26T14:30:15Z",
  "data": {
    "@type": "GridStatusUpdate",
    "frequency": 60.01,
    "load": 2455
  }
}
```

## 6. Batch Operations

**POST /api/v1/batch**

Execute multiple operations in single request.

**Request:**
```json
{
  "operations": [
    {
      "method": "GET",
      "path": "/api/v1/grid-status"
    },
    {
      "method": "GET",
      "path": "/api/v1/renewables/current"
    }
  ]
}
```

## 7. Error Handling

### 7.1 Error Response Format

```json
{
  "error": {
    "code": "INVALID_PARAMETER",
    "message": "The 'zone' parameter is invalid",
    "details": {
      "parameter": "zone",
      "provided": "invalid-zone",
      "expected": "Valid zone ID"
    },
    "timestamp": "2025-12-26T14:30:00Z",
    "requestId": "req-12345"
  }
}
```

### 7.2 HTTP Status Codes

- 200: Success
- 201: Created
- 400: Bad Request
- 401: Unauthorized
- 403: Forbidden
- 404: Not Found
- 429: Too Many Requests
- 500: Internal Server Error
- 503: Service Unavailable

## 8. Rate Limiting

- Default: 1000 requests/hour
- WebSocket: 100 messages/second
- Header: `X-RateLimit-Remaining: 950`

## 9. Pagination

**Request:**
```http
GET /api/v1/meters?limit=100&offset=0
```

**Response:**
```json
{
  "data": [...],
  "pagination": {
    "limit": 100,
    "offset": 0,
    "total": 1000,
    "hasMore": true
  }
}
```

---

**End of PHASE 2 Specification**
