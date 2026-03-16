# WIA-AGRI-005 PHASE 2: API Interface

## Overview
Phase 2 defines the RESTful API interfaces and SDK specifications for the WIA-AGRI-005 Soil Sensor Standard, enabling seamless integration with farm management systems, irrigation controllers, and agricultural analytics platforms.

## API Architecture

### Base URL
```
https://api.wia-agri.org/v1/soil-sensor
```

### Authentication
All API requests require authentication using API keys or OAuth 2.0.

```http
Authorization: Bearer <access_token>
X-API-Key: <your_api_key>
```

### Response Format
All responses follow the JSON:API specification with consistent structure:

```json
{
  "data": { ... },
  "meta": {
    "timestamp": "2025-12-26T10:00:00Z",
    "version": "1.0",
    "standard": "WIA-AGRI-005"
  },
  "links": {
    "self": "https://api.wia-agri.org/v1/soil-sensor/readings/123"
  }
}
```

## Core API Endpoints

### 1. Sensor Management

#### Register New Sensor
```http
POST /sensors
Content-Type: application/json

{
  "sensorId": "SOIL-001",
  "name": "Field A North Sector",
  "location": {
    "latitude": 37.5665,
    "longitude": 126.9780,
    "depth": 15
  },
  "sensorType": "multi-parameter",
  "manufacturer": "AgriTech Solutions",
  "model": "NPK-pH-EC-2024"
}
```

**Response**: `201 Created`
```json
{
  "data": {
    "id": "550e8400-e29b-41d4-a716-446655440000",
    "sensorId": "SOIL-001",
    "status": "registered",
    "createdAt": "2025-12-26T10:00:00Z"
  }
}
```

#### Get Sensor Details
```http
GET /sensors/{sensorId}
```

**Response**: `200 OK`
```json
{
  "data": {
    "sensorId": "SOIL-001",
    "name": "Field A North Sector",
    "status": "active",
    "location": {
      "latitude": 37.5665,
      "longitude": 126.9780,
      "depth": 15
    },
    "lastReading": "2025-12-26T10:00:00Z",
    "batteryLevel": 87,
    "calibrationStatus": "current",
    "nextCalibration": "2025-07-15"
  }
}
```

#### Update Sensor Configuration
```http
PATCH /sensors/{sensorId}
Content-Type: application/json

{
  "name": "Field A North Sector Updated",
  "calibrationDate": "2025-12-26"
}
```

#### Delete Sensor
```http
DELETE /sensors/{sensorId}
```

**Response**: `204 No Content`

### 2. Sensor Readings

#### Submit Sensor Reading
```http
POST /readings
Content-Type: application/json

{
  "sensorId": "SOIL-001",
  "timestamp": "2025-12-26T10:00:00Z",
  "npk": {
    "nitrogen": 85,
    "phosphorus": 35,
    "potassium": 150
  },
  "ph": 6.8,
  "ec": 1.2,
  "moisture": 35,
  "temperature": 22
}
```

**Response**: `201 Created`
```json
{
  "data": {
    "id": "reading-12345",
    "sensorId": "SOIL-001",
    "timestamp": "2025-12-26T10:00:00Z",
    "status": "processed"
  }
}
```

#### Get Latest Reading
```http
GET /sensors/{sensorId}/readings/latest
```

**Response**: `200 OK`
```json
{
  "data": {
    "sensorId": "SOIL-001",
    "timestamp": "2025-12-26T10:00:00Z",
    "npk": {
      "nitrogen": 85,
      "phosphorus": 35,
      "potassium": 150
    },
    "ph": 6.8,
    "ec": 1.2,
    "moisture": 35,
    "temperature": 22,
    "quality": {
      "status": "valid",
      "flags": []
    }
  }
}
```

#### Get Historical Readings
```http
GET /sensors/{sensorId}/readings?start=2025-12-20&end=2025-12-26&interval=1h
```

**Query Parameters**:
- `start`: ISO 8601 timestamp (required)
- `end`: ISO 8601 timestamp (required)
- `interval`: Data aggregation interval (1m, 5m, 15m, 1h, 1d)
- `limit`: Maximum number of records (default: 100, max: 1000)
- `offset`: Pagination offset

**Response**: `200 OK`
```json
{
  "data": [
    {
      "timestamp": "2025-12-26T09:00:00Z",
      "npk": { "n": 85, "p": 35, "k": 150 },
      "ph": 6.8,
      "moisture": 35
    },
    {
      "timestamp": "2025-12-26T10:00:00Z",
      "npk": { "n": 86, "p": 35, "k": 148 },
      "ph": 6.9,
      "moisture": 33
    }
  ],
  "meta": {
    "total": 144,
    "limit": 100,
    "offset": 0
  },
  "links": {
    "next": "/sensors/SOIL-001/readings?start=2025-12-20&end=2025-12-26&limit=100&offset=100"
  }
}
```

### 3. Batch Operations

#### Submit Batch Readings
```http
POST /readings/batch
Content-Type: application/json

{
  "batchId": "BATCH-2025-12-26-001",
  "timestamp": "2025-12-26T10:00:00Z",
  "readings": [
    {
      "sensorId": "SOIL-001",
      "npk": { "n": 85, "p": 35, "k": 150 },
      "ph": 6.8,
      "moisture": 35
    },
    {
      "sensorId": "SOIL-002",
      "npk": { "n": 92, "p": 38, "k": 145 },
      "ph": 7.0,
      "moisture": 38
    }
  ]
}
```

**Response**: `201 Created`
```json
{
  "data": {
    "batchId": "BATCH-2025-12-26-001",
    "processed": 2,
    "failed": 0,
    "status": "success"
  }
}
```

### 4. Analytics & Recommendations

#### Get Nutrient Analysis
```http
GET /sensors/{sensorId}/analysis/nutrients?crop=rice
```

**Response**: `200 OK`
```json
{
  "data": {
    "sensorId": "SOIL-001",
    "crop": "rice",
    "analysis": {
      "nitrogen": {
        "current": 85,
        "ideal": [80, 120],
        "status": "optimal",
        "recommendation": "Maintain current levels"
      },
      "phosphorus": {
        "current": 35,
        "ideal": [30, 50],
        "status": "optimal",
        "recommendation": "Maintain current levels"
      },
      "potassium": {
        "current": 150,
        "ideal": [100, 150],
        "status": "optimal",
        "recommendation": "Maintain current levels"
      }
    },
    "overallHealth": "excellent"
  }
}
```

#### Get Irrigation Recommendation
```http
GET /sensors/{sensorId}/recommendations/irrigation?crop=rice
```

**Response**: `200 OK`
```json
{
  "data": {
    "sensorId": "SOIL-001",
    "moisture": 35,
    "recommendation": "good",
    "action": "monitor",
    "schedule": null,
    "waterAmount": null,
    "reason": "Moisture level is within optimal range for rice cultivation",
    "nextCheck": "2025-12-27T10:00:00Z"
  }
}
```

### 5. Alerts & Notifications

#### Get Active Alerts
```http
GET /sensors/{sensorId}/alerts
```

**Response**: `200 OK`
```json
{
  "data": [
    {
      "id": "alert-001",
      "sensorId": "SOIL-001",
      "type": "moisture_low",
      "severity": "warning",
      "message": "Soil moisture below optimal range",
      "value": 25,
      "threshold": 30,
      "triggeredAt": "2025-12-26T08:00:00Z",
      "status": "active"
    }
  ]
}
```

#### Configure Alert Rules
```http
POST /sensors/{sensorId}/alerts/rules
Content-Type: application/json

{
  "type": "moisture_low",
  "threshold": 30,
  "severity": "warning",
  "action": "notify",
  "recipients": ["farmer@example.com"]
}
```

### 6. Calibration Management

#### Submit Calibration Record
```http
POST /sensors/{sensorId}/calibration
Content-Type: application/json

{
  "calibratedAt": "2025-12-26T10:00:00Z",
  "calibratedBy": "John Doe",
  "parameters": {
    "ph": {
      "buffer4": 4.01,
      "buffer7": 7.00,
      "buffer10": 10.01
    },
    "ec": {
      "standard1413": 1.413
    }
  },
  "nextCalibration": "2026-06-26"
}
```

## TypeScript SDK

### Installation
```bash
npm install @wia-standards/soil-sensor
```

### Basic Usage
```typescript
import { WIASoilSensor } from '@wia-standards/soil-sensor';

const client = new WIASoilSensor({
  apiKey: 'your-api-key',
  baseURL: 'https://api.wia-agri.org/v1'
});

// Register sensor
const sensor = await client.sensors.register({
  sensorId: 'SOIL-001',
  name: 'Field A North Sector',
  location: {
    latitude: 37.5665,
    longitude: 126.9780,
    depth: 15
  }
});

// Submit reading
const reading = await client.readings.submit({
  sensorId: 'SOIL-001',
  npk: { nitrogen: 85, phosphorus: 35, potassium: 150 },
  ph: 6.8,
  ec: 1.2,
  moisture: 35,
  temperature: 22
});

// Get latest reading
const latest = await client.sensors.getLatestReading('SOIL-001');

// Get nutrient analysis
const analysis = await client.analysis.nutrients('SOIL-001', { crop: 'rice' });

// Get irrigation recommendation
const irrigation = await client.recommendations.irrigation('SOIL-001', {
  crop: 'rice'
});
```

### Advanced Features
```typescript
// Real-time data streaming
const stream = client.sensors.stream('SOIL-001');
stream.on('data', (reading) => {
  console.log('New reading:', reading);
});

// Batch operations
const batchResult = await client.readings.submitBatch({
  readings: [
    { sensorId: 'SOIL-001', npk: {...}, ph: 6.8 },
    { sensorId: 'SOIL-002', npk: {...}, ph: 7.0 }
  ]
});

// Alert subscription
client.alerts.subscribe('SOIL-001', (alert) => {
  console.log('Alert triggered:', alert);
});

// Historical data query
const history = await client.readings.getHistory({
  sensorId: 'SOIL-001',
  start: '2025-12-20T00:00:00Z',
  end: '2025-12-26T23:59:59Z',
  interval: '1h',
  aggregation: 'avg'
});
```

## Error Handling

### Error Response Format
```json
{
  "error": {
    "code": "SENSOR_NOT_FOUND",
    "message": "Sensor with ID 'SOIL-999' not found",
    "details": {
      "sensorId": "SOIL-999"
    },
    "timestamp": "2025-12-26T10:00:00Z"
  }
}
```

### Common Error Codes
- `400 BAD_REQUEST`: Invalid request format
- `401 UNAUTHORIZED`: Missing or invalid authentication
- `403 FORBIDDEN`: Insufficient permissions
- `404 NOT_FOUND`: Resource not found
- `409 CONFLICT`: Resource already exists
- `422 UNPROCESSABLE_ENTITY`: Validation error
- `429 TOO_MANY_REQUESTS`: Rate limit exceeded
- `500 INTERNAL_SERVER_ERROR`: Server error
- `503 SERVICE_UNAVAILABLE`: Service temporarily unavailable

### SDK Error Handling
```typescript
try {
  const reading = await client.readings.submit({...});
} catch (error) {
  if (error instanceof WIASensorNotFoundError) {
    console.error('Sensor not found:', error.sensorId);
  } else if (error instanceof WIAValidationError) {
    console.error('Validation failed:', error.details);
  } else if (error instanceof WIARateLimitError) {
    console.error('Rate limit exceeded. Retry after:', error.retryAfter);
  } else {
    console.error('Unknown error:', error);
  }
}
```

## Rate Limiting

### Limits
- **Free tier**: 100 requests/minute, 10,000 requests/day
- **Standard tier**: 1,000 requests/minute, 100,000 requests/day
- **Enterprise tier**: Custom limits

### Rate Limit Headers
```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1640516400
```

## Webhooks

### Configure Webhook
```http
POST /webhooks
Content-Type: application/json

{
  "url": "https://your-app.com/webhook",
  "events": ["reading.created", "alert.triggered"],
  "sensorIds": ["SOIL-001", "SOIL-002"]
}
```

### Webhook Payload
```json
{
  "event": "reading.created",
  "timestamp": "2025-12-26T10:00:00Z",
  "data": {
    "sensorId": "SOIL-001",
    "reading": {
      "npk": { "n": 85, "p": 35, "k": 150 },
      "ph": 6.8,
      "moisture": 35
    }
  }
}
```

## 弘益人間 (Benefit All Humanity)
Phase 2 provides powerful yet accessible APIs that enable developers worldwide to build innovative agricultural solutions, democratizing access to soil health data and precision farming technologies.

---
© 2025 SmileStory Inc. / WIA
