# WIA-AGRI-020: Agricultural Data Exchange Standard
## Phase 2: API Interface Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-12-26

---

## 1. Overview

This specification defines RESTful API endpoints, authentication mechanisms, and integration patterns for agricultural data exchange.

### 1.1 API Design Principles

- **RESTful**: Standard HTTP methods (GET, POST, PUT, DELETE)
- **Stateless**: No server-side session management
- **Cacheable**: Appropriate cache headers for static data
- **Rate-Limited**: Protection against abuse
- **Versioned**: API version in URL path

### 1.2 Base URL Structure

```
https://api.wia-agri.org/v1
```

---

## 2. Authentication & Authorization

### 2.1 API Key Authentication

**Header-based authentication:**
```http
GET /api/v1/data/query HTTP/1.1
Host: api.wia-agri.org
X-API-Key: wia_live_abc123def456ghi789
Content-Type: application/json
```

**API Key Types:**
- `wia_test_*`: Test environment (rate-limited, sandbox data)
- `wia_live_*`: Production environment

### 2.2 OAuth 2.0 Flow

For third-party integrations:

```http
POST /oauth/token HTTP/1.1
Host: auth.wia-agri.org
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials&
client_id=YOUR_CLIENT_ID&
client_secret=YOUR_CLIENT_SECRET&
scope=data:read data:write
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "data:read data:write"
}
```

### 2.3 Scopes

- `data:read` - Read sensor data and queries
- `data:write` - Submit new data
- `farm:manage` - Manage farm profiles
- `device:register` - Register IoT devices
- `analytics:access` - Access analytics endpoints

---

## 3. Core API Endpoints

### 3.1 Data Submission

#### Submit Single Data Record

```http
POST /api/v1/data/submit
Content-Type: application/json
X-API-Key: wia_live_abc123def456ghi789

{
  "standardVersion": "WIA-AGRI-020-v1.0",
  "farmId": "FARM-2025-001",
  "dataType": "SOIL_SENSOR",
  "timestamp": "2025-12-26T10:30:00.000Z",
  "readings": {
    "soilMoisture": { "value": 45.5 },
    "soilPH": { "value": 6.8 }
  }
}
```

**Success Response (201 Created):**
```json
{
  "status": "success",
  "dataId": "DATA-20251226-001",
  "timestamp": "2025-12-26T10:30:00.000Z",
  "validationResult": {
    "isValid": true,
    "warnings": [],
    "errors": []
  },
  "receivedAt": "2025-12-26T10:30:01.234Z"
}
```

**Error Response (400 Bad Request):**
```json
{
  "status": "error",
  "code": "VALIDATION_ERROR",
  "message": "Data validation failed",
  "details": [
    {
      "field": "readings.soilMoisture.value",
      "error": "Value 150 exceeds maximum allowed (100)"
    }
  ]
}
```

#### Batch Submission

```http
POST /api/v1/data/batch
Content-Type: application/json

{
  "batchId": "BATCH-20251226-001",
  "farmId": "FARM-2025-001",
  "data": [
    { "dataId": "SOIL-001", "dataType": "SOIL_SENSOR", "readings": {...} },
    { "dataId": "WEATHER-001", "dataType": "WEATHER", "current": {...} },
    { "dataId": "CROP-001", "dataType": "CROP_HEALTH", "healthMetrics": {...} }
  ]
}
```

**Response:**
```json
{
  "status": "success",
  "batchId": "BATCH-20251226-001",
  "processedCount": 3,
  "successCount": 3,
  "failureCount": 0,
  "results": [
    { "dataId": "SOIL-001", "status": "accepted" },
    { "dataId": "WEATHER-001", "status": "accepted" },
    { "dataId": "CROP-001", "status": "accepted" }
  ]
}
```

### 3.2 Data Retrieval

#### Query Historical Data

```http
GET /api/v1/data/query?farmId=FARM-2025-001&dataType=SOIL_SENSOR&from=2025-12-20&to=2025-12-26&limit=100
```

**Response:**
```json
{
  "status": "success",
  "query": {
    "farmId": "FARM-2025-001",
    "dataType": "SOIL_SENSOR",
    "dateRange": {
      "from": "2025-12-20T00:00:00.000Z",
      "to": "2025-12-26T23:59:59.999Z"
    }
  },
  "results": {
    "count": 45,
    "data": [
      {
        "dataId": "DATA-20251226-001",
        "timestamp": "2025-12-26T10:30:00.000Z",
        "readings": {
          "soilMoisture": { "value": 45.5 },
          "soilPH": { "value": 6.8 }
        }
      }
    ]
  },
  "pagination": {
    "page": 1,
    "pageSize": 100,
    "totalPages": 1,
    "totalRecords": 45
  }
}
```

#### Get Latest Reading

```http
GET /api/v1/data/latest?farmId=FARM-2025-001&dataType=SOIL_SENSOR&fieldId=FIELD-NORTH-01
```

**Response:**
```json
{
  "status": "success",
  "dataId": "DATA-20251226-001",
  "timestamp": "2025-12-26T10:30:00.000Z",
  "age": 120,
  "ageUnit": "seconds",
  "readings": {
    "soilMoisture": { "value": 45.5 },
    "soilPH": { "value": 6.8 }
  }
}
```

### 3.3 Data Validation

```http
POST /api/v1/data/validate
Content-Type: application/json

{
  "standardVersion": "WIA-AGRI-020-v1.0",
  "farmId": "FARM-2025-001",
  "dataType": "SOIL_SENSOR",
  "readings": {
    "soilMoisture": { "value": 45.5 }
  }
}
```

**Response:**
```json
{
  "isValid": true,
  "schemaVersion": "WIA-AGRI-020-v1.0",
  "validatedAt": "2025-12-26T10:30:00.000Z",
  "warnings": [
    "soilTemperature not provided (optional but recommended)"
  ],
  "errors": [],
  "qualityScore": 85
}
```

### 3.4 Device Management

#### Register IoT Device

```http
POST /api/v1/devices/register
Content-Type: application/json

{
  "deviceId": "SENSOR-SOIL-001",
  "farmId": "FARM-2025-001",
  "deviceType": "SOIL_SENSOR",
  "manufacturer": "AgriTech Inc.",
  "model": "SoilPro-X200",
  "location": {
    "fieldId": "FIELD-NORTH-01",
    "latitude": 37.5665,
    "longitude": 126.9780
  }
}
```

**Response:**
```json
{
  "status": "success",
  "deviceId": "SENSOR-SOIL-001",
  "deviceToken": "dev_abc123xyz789",
  "registeredAt": "2025-12-26T10:30:00.000Z",
  "expiresAt": "2026-12-26T10:30:00.000Z"
}
```

#### List Farm Devices

```http
GET /api/v1/devices?farmId=FARM-2025-001&status=active
```

**Response:**
```json
{
  "status": "success",
  "count": 12,
  "devices": [
    {
      "deviceId": "SENSOR-SOIL-001",
      "deviceType": "SOIL_SENSOR",
      "status": "active",
      "lastSeen": "2025-12-26T10:25:00.000Z",
      "dataPoints": 1245,
      "batteryLevel": 78
    }
  ]
}
```

---

## 4. Real-Time Data Streaming

### 4.1 WebSocket Connection

```javascript
const ws = new WebSocket('wss://stream.wia-agri.org/v1/stream');

ws.onopen = function() {
  // Subscribe to farm data stream
  ws.send(JSON.stringify({
    action: 'subscribe',
    farmId: 'FARM-2025-001',
    dataTypes: ['SOIL_SENSOR', 'WEATHER'],
    token: 'wia_live_abc123def456ghi789'
  }));
};

ws.onmessage = function(event) {
  const data = JSON.parse(event.data);
  console.log('Real-time data:', data);
};
```

**Stream Message Format:**
```json
{
  "type": "data",
  "dataId": "DATA-20251226-001",
  "farmId": "FARM-2025-001",
  "dataType": "SOIL_SENSOR",
  "timestamp": "2025-12-26T10:30:00.000Z",
  "readings": {
    "soilMoisture": { "value": 45.5 }
  }
}
```

### 4.2 MQTT Integration

**Topic Structure:**
```
wia-agri/{farmId}/{dataType}/{deviceId}
```

**Example:**
```
Topic: wia-agri/FARM-2025-001/SOIL_SENSOR/SENSOR-SOIL-001
Payload: {"timestamp":"2025-12-26T10:30:00.000Z","readings":{"soilMoisture":{"value":45.5}}}
```

**MQTT Broker:**
```
Host: mqtt.wia-agri.org
Port: 1883 (unencrypted) / 8883 (TLS)
QoS: 0, 1, or 2
```

---

## 5. Rate Limiting

### 5.1 Rate Limit Headers

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1703592000
```

### 5.2 Limits by Tier

| Tier | Requests/Hour | Burst | WebSocket Connections |
|------|---------------|-------|----------------------|
| Free | 1,000 | 100 | 2 |
| Basic | 10,000 | 500 | 10 |
| Pro | 100,000 | 2,000 | 50 |
| Enterprise | Unlimited | Unlimited | Unlimited |

### 5.3 Rate Limit Exceeded Response

```http
HTTP/1.1 429 Too Many Requests
Content-Type: application/json
Retry-After: 3600

{
  "status": "error",
  "code": "RATE_LIMIT_EXCEEDED",
  "message": "Rate limit exceeded. Try again in 3600 seconds.",
  "limit": 1000,
  "remaining": 0,
  "resetAt": "2025-12-26T11:00:00.000Z"
}
```

---

## 6. Pagination

### 6.1 Cursor-Based Pagination

```http
GET /api/v1/data/query?farmId=FARM-2025-001&limit=100&cursor=eyJpZCI6MTIzNDU2fQ
```

**Response:**
```json
{
  "data": [...],
  "pagination": {
    "nextCursor": "eyJpZCI6MTIzNTU2fQ",
    "hasMore": true
  }
}
```

### 6.2 Offset-Based Pagination

```http
GET /api/v1/data/query?farmId=FARM-2025-001&page=2&pageSize=50
```

---

## 7. Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| `VALIDATION_ERROR` | 400 | Invalid data format |
| `AUTHENTICATION_FAILED` | 401 | Invalid API key |
| `AUTHORIZATION_FAILED` | 403 | Insufficient permissions |
| `NOT_FOUND` | 404 | Resource not found |
| `RATE_LIMIT_EXCEEDED` | 429 | Too many requests |
| `INTERNAL_ERROR` | 500 | Server error |
| `SERVICE_UNAVAILABLE` | 503 | Temporary outage |

---

## 8. Webhooks

### 8.1 Configure Webhook

```http
POST /api/v1/webhooks
Content-Type: application/json

{
  "url": "https://your-server.com/webhook",
  "events": ["data.submitted", "device.offline"],
  "farmId": "FARM-2025-001",
  "secret": "whsec_abc123xyz789"
}
```

### 8.2 Webhook Payload

```json
{
  "event": "data.submitted",
  "timestamp": "2025-12-26T10:30:00.000Z",
  "farmId": "FARM-2025-001",
  "data": {
    "dataId": "DATA-20251226-001",
    "dataType": "SOIL_SENSOR"
  }
}
```

---

## 9. SDK Examples

### 9.1 JavaScript/Node.js

```javascript
const WIAAgri = require('wia-agri-sdk');

const client = new WIAAgri({
  apiKey: 'wia_live_abc123def456ghi789'
});

// Submit data
await client.data.submit({
  farmId: 'FARM-2025-001',
  dataType: 'SOIL_SENSOR',
  readings: {
    soilMoisture: { value: 45.5 }
  }
});

// Query data
const results = await client.data.query({
  farmId: 'FARM-2025-001',
  dataType: 'SOIL_SENSOR',
  from: '2025-12-20',
  to: '2025-12-26'
});
```

### 9.2 Python

```python
from wia_agri import WIAAgriClient

client = WIAAgriClient(api_key='wia_live_abc123def456ghi789')

# Submit data
client.data.submit(
    farm_id='FARM-2025-001',
    data_type='SOIL_SENSOR',
    readings={
        'soilMoisture': {'value': 45.5}
    }
)

# Query data
results = client.data.query(
    farm_id='FARM-2025-001',
    data_type='SOIL_SENSOR',
    from_date='2025-12-20',
    to_date='2025-12-26'
)
```

---

## 10. API Testing

### 10.1 Test Environment

```
Base URL: https://api-test.wia-agri.org/v1
API Key: wia_test_sandbox_123456
```

### 10.2 Mock Data

Test API provides realistic mock data for development.

---

## 11. Implementation Checklist

- [ ] Authentication (API Key + OAuth 2.0)
- [ ] Data submission (single + batch)
- [ ] Data retrieval (query + latest)
- [ ] Data validation endpoint
- [ ] Device registration
- [ ] WebSocket streaming
- [ ] Rate limiting
- [ ] Pagination
- [ ] Error handling
- [ ] Webhook support

---

**Next Phase:** [PHASE-3-PROTOCOL.md](./PHASE-3-PROTOCOL.md)

---

© 2025 WIA Standards · MIT License
弘益人間 · Benefit All Humanity
