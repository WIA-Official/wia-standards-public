# WIA-AGRI-021: Desert Agriculture
## Phase 2 - API Interface Specification

**Version:** 1.0
**Status:** Official Standard
**Last Updated:** 2025-12-26
**Standard ID:** WIA-AGRI-021

---

## 1. Overview

This specification defines the REST API interface for the WIA-AGRI-021 Desert Agriculture standard. The API provides programmatic access to sensor data, irrigation control, analytics, and system management functions optimized for desert agricultural operations.

### 1.1 API Design Principles

- **RESTful**: Follows REST architectural constraints
- **Stateless**: Each request contains all necessary information
- **Cacheable**: Responses include caching directives
- **Versioned**: API version included in URL path
- **Secure**: HTTPS required for all endpoints
- **Rate Limited**: Protection against abuse and overload

### 1.2 Base URL

```
Production: https://api.wia-agri-021.org/v1
Sandbox: https://sandbox-api.wia-agri-021.org/v1
```

---

## 2. Authentication

### 2.1 API Key Authentication

All API requests require an API key passed in the request header:

```http
GET /api/v1/sensors/data HTTP/1.1
Host: api.wia-agri-021.org
X-API-Key: your_api_key_here
Content-Type: application/json
```

### 2.2 Obtaining API Keys

API keys can be obtained through:
1. WIA Developer Portal: https://developer.wia-agri-021.org
2. Farm Management Dashboard
3. Programmatic key generation API

### 2.3 API Key Security

- Store keys securely (environment variables, secrets management)
- Never commit keys to version control
- Rotate keys every 90 days
- Use different keys for development and production
- Revoke compromised keys immediately

### 2.4 Rate Limiting

| Plan | Requests/Hour | Requests/Day |
|------|--------------|--------------|
| Free | 100 | 1,000 |
| Basic | 1,000 | 10,000 |
| Professional | 10,000 | 100,000 |
| Enterprise | Unlimited | Unlimited |

Rate limit information is included in response headers:

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 847
X-RateLimit-Reset: 1735214400
```

---

## 3. Sensor Data Endpoints

### 3.1 Get Sensor Data

Retrieve current or historical sensor readings.

**Endpoint:** `GET /api/v1/sensors/data`

**Query Parameters:**

| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| farmId | string | Yes | Farm identifier |
| sensorType | string | No | Filter by sensor type |
| sensorId | string | No | Specific sensor ID |
| startTime | ISO-8601 | No | Start of time range |
| endTime | ISO-8601 | No | End of time range |
| limit | integer | No | Max results (default: 100, max: 1000) |

**Example Request:**

```http
GET /api/v1/sensors/data?farmId=farm_sahara_001&sensorType=soil_moisture&limit=10
X-API-Key: your_api_key_here
```

**Example Response:**

```json
{
  "success": true,
  "data": [
    {
      "sensorId": "sensor_sm_zone_a1_001",
      "type": "soil_moisture",
      "farmId": "farm_sahara_001",
      "zoneId": "zone_a1",
      "timestamp": "2025-12-26T10:30:00Z",
      "measurements": [
        {
          "depth": 0.10,
          "value": 12.8,
          "unit": "percent"
        },
        {
          "depth": 0.20,
          "value": 15.3,
          "unit": "percent"
        }
      ],
      "metadata": {
        "soilType": "sandy_loam",
        "calibrationDate": "2025-11-15"
      }
    }
  ],
  "pagination": {
    "total": 1,
    "limit": 10,
    "offset": 0
  },
  "timestamp": "2025-12-26T10:35:00Z"
}
```

### 3.2 Get Aggregated Sensor Data

Retrieve statistical aggregations over time periods.

**Endpoint:** `GET /api/v1/sensors/aggregate`

**Query Parameters:**

| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| farmId | string | Yes | Farm identifier |
| sensorType | string | Yes | Sensor type |
| interval | string | Yes | hour, day, week, month |
| aggregation | string | Yes | mean, min, max, sum |
| startTime | ISO-8601 | Yes | Start time |
| endTime | ISO-8601 | Yes | End time |

**Example Request:**

```http
GET /api/v1/sensors/aggregate?farmId=farm_sahara_001&sensorType=temperature&interval=hour&aggregation=mean&startTime=2025-12-26T00:00:00Z&endTime=2025-12-26T23:59:59Z
X-API-Key: your_api_key_here
```

**Example Response:**

```json
{
  "success": true,
  "data": {
    "farmId": "farm_sahara_001",
    "sensorType": "temperature",
    "interval": "hour",
    "aggregation": "mean",
    "period": {
      "start": "2025-12-26T00:00:00Z",
      "end": "2025-12-26T23:59:59Z"
    },
    "values": [
      {
        "timestamp": "2025-12-26T00:00:00Z",
        "value": 22.5
      },
      {
        "timestamp": "2025-12-26T01:00:00Z",
        "value": 21.8
      }
    ]
  }
}
```

### 3.3 Stream Sensor Data (WebSocket)

Real-time sensor data streaming via WebSocket connection.

**Endpoint:** `wss://ws.wia-agri-021.org/v1/sensors/stream`

**Connection:**

```javascript
const ws = new WebSocket('wss://ws.wia-agri-021.org/v1/sensors/stream');

// Authenticate
ws.send(JSON.stringify({
  type: 'auth',
  apiKey: 'your_api_key_here'
}));

// Subscribe to sensors
ws.send(JSON.stringify({
  type: 'subscribe',
  farmId: 'farm_sahara_001',
  sensorTypes: ['soil_moisture', 'temperature']
}));

// Receive data
ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log(data);
};
```

**Message Format:**

```json
{
  "type": "sensor_data",
  "timestamp": "2025-12-26T10:30:00Z",
  "sensorId": "sensor_sm_001",
  "data": {
    "value": 12.8,
    "unit": "percent"
  }
}
```

---

## 4. Irrigation Control Endpoints

### 4.1 Start Irrigation

**Endpoint:** `POST /api/v1/irrigation/start`

**Request Body:**

```json
{
  "farmId": "farm_sahara_001",
  "zoneId": "zone_a1",
  "duration": 1800,
  "flowRate": 15,
  "mode": "manual",
  "parameters": {
    "targetMoisture": 25,
    "maxVolume": 500
  }
}
```

**Response:**

```json
{
  "success": true,
  "data": {
    "taskId": "irr_task_12345",
    "farmId": "farm_sahara_001",
    "zoneId": "zone_a1",
    "status": "running",
    "startTime": "2025-12-26T10:30:00Z",
    "estimatedEndTime": "2025-12-26T11:00:00Z",
    "estimatedVolume": 450
  },
  "message": "Irrigation started successfully"
}
```

### 4.2 Stop Irrigation

**Endpoint:** `POST /api/v1/irrigation/stop`

**Request Body:**

```json
{
  "farmId": "farm_sahara_001",
  "zoneId": "zone_a1",
  "reason": "manual_override"
}
```

**Response:**

```json
{
  "success": true,
  "data": {
    "farmId": "farm_sahara_001",
    "zoneId": "zone_a1",
    "status": "stopped",
    "stopTime": "2025-12-26T10:45:00Z",
    "actualDuration": 900,
    "waterUsed": 225
  },
  "message": "Irrigation stopped successfully"
}
```

### 4.3 Get Irrigation Status

**Endpoint:** `GET /api/v1/irrigation/status`

**Query Parameters:**

| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| farmId | string | Yes | Farm identifier |
| zoneId | string | No | Zone identifier |

**Example Response:**

```json
{
  "success": true,
  "data": {
    "farmId": "farm_sahara_001",
    "zones": [
      {
        "zoneId": "zone_a1",
        "status": "running",
        "currentTask": {
          "taskId": "irr_task_12345",
          "startTime": "2025-12-26T10:30:00Z",
          "duration": 1800,
          "elapsed": 600,
          "waterUsed": 150
        }
      },
      {
        "zoneId": "zone_a2",
        "status": "idle",
        "lastIrrigation": "2025-12-26T08:00:00Z"
      }
    ]
  }
}
```

### 4.4 Get Irrigation Schedule

**Endpoint:** `GET /api/v1/irrigation/schedule`

**Query Parameters:**

| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| farmId | string | Yes | Farm identifier |
| startDate | ISO-8601 | No | Schedule start date |
| endDate | ISO-8601 | No | Schedule end date |

**Example Response:**

```json
{
  "success": true,
  "data": {
    "farmId": "farm_sahara_001",
    "schedule": [
      {
        "scheduleId": "sched_001",
        "zoneId": "zone_a1",
        "enabled": true,
        "frequency": "daily",
        "time": "06:00",
        "duration": 1800,
        "conditions": {
          "minSoilMoisture": 15,
          "weatherDependent": true
        }
      }
    ]
  }
}
```

### 4.5 Update Irrigation Schedule

**Endpoint:** `PUT /api/v1/irrigation/schedule/{scheduleId}`

**Request Body:**

```json
{
  "enabled": true,
  "time": "07:00",
  "duration": 2400,
  "conditions": {
    "minSoilMoisture": 12,
    "weatherDependent": true,
    "skipIfRain": true
  }
}
```

---

## 5. Analytics Endpoints

### 5.1 Yield Prediction

**Endpoint:** `GET /api/v1/analytics/yield`

**Query Parameters:**

| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| farmId | string | Yes | Farm identifier |
| cropType | string | Yes | Type of crop |
| zoneId | string | No | Specific zone |

**Example Response:**

```json
{
  "success": true,
  "data": {
    "farmId": "farm_sahara_001",
    "cropType": "tomato",
    "prediction": {
      "estimatedYield": 8500,
      "unit": "kg",
      "confidence": 0.89,
      "variance": 450,
      "harvestDate": "2025-01-15"
    },
    "factors": {
      "currentHealth": 87,
      "waterAvailability": 95,
      "climateOptimality": 82,
      "pestPressure": 5
    },
    "historical": {
      "previousYield": 8200,
      "averageYield": 7800
    }
  }
}
```

### 5.2 Water Usage Analytics

**Endpoint:** `GET /api/v1/analytics/water-usage`

**Query Parameters:**

| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| farmId | string | Yes | Farm identifier |
| startDate | ISO-8601 | Yes | Start date |
| endDate | ISO-8601 | Yes | End date |
| groupBy | string | No | zone, crop, day, week |

**Example Response:**

```json
{
  "success": true,
  "data": {
    "farmId": "farm_sahara_001",
    "period": {
      "start": "2025-12-01",
      "end": "2025-12-26"
    },
    "usage": {
      "total": 125000,
      "unit": "liters",
      "average_daily": 4807,
      "efficiency": 87.5
    },
    "breakdown": {
      "zone_a1": 45000,
      "zone_a2": 38000,
      "zone_b1": 42000
    },
    "comparison": {
      "previous_period": 142000,
      "change_percent": -11.97,
      "status": "improved"
    }
  }
}
```

### 5.3 Crop Health Analysis

**Endpoint:** `GET /api/v1/analytics/crop-health`

**Query Parameters:**

| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| farmId | string | Yes | Farm identifier |
| cropId | string | No | Specific crop |
| zoneId | string | No | Specific zone |

**Example Response:**

```json
{
  "success": true,
  "data": {
    "farmId": "farm_sahara_001",
    "zoneId": "zone_a1",
    "overall_health": 87,
    "metrics": {
      "growth_rate": {
        "value": 92,
        "status": "optimal"
      },
      "stress_level": {
        "value": 15,
        "status": "low"
      },
      "nutrient_status": {
        "nitrogen": 85,
        "phosphorus": 78,
        "potassium": 82
      }
    },
    "recommendations": [
      "Maintain current irrigation schedule",
      "Monitor for heat stress during peak hours",
      "Consider potassium supplement in 2 weeks"
    ]
  }
}
```

### 5.4 Climate Impact Analysis

**Endpoint:** `GET /api/v1/analytics/climate-impact`

**Query Parameters:**

| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| farmId | string | Yes | Farm identifier |
| timeframe | string | Yes | 7d, 30d, 90d, 1y |

**Example Response:**

```json
{
  "success": true,
  "data": {
    "farmId": "farm_sahara_001",
    "timeframe": "30d",
    "climate_summary": {
      "avg_temperature": 32.5,
      "max_temperature": 45.2,
      "avg_humidity": 18.3,
      "total_solar_radiation": 28500,
      "wind_events": 5
    },
    "impact": {
      "water_demand_increase": 22,
      "heat_stress_days": 8,
      "optimal_growth_days": 18,
      "yield_impact_estimate": -5
    },
    "adaptation_score": 78
  }
}
```

---

## 6. Alert Management Endpoints

### 6.1 Get Alerts

**Endpoint:** `GET /api/v1/alerts`

**Query Parameters:**

| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| farmId | string | Yes | Farm identifier |
| severity | string | No | low, medium, high, critical |
| status | string | No | active, acknowledged, resolved |
| startTime | ISO-8601 | No | Start time |
| limit | integer | No | Max results |

**Example Response:**

```json
{
  "success": true,
  "data": {
    "alerts": [
      {
        "alertId": "alert_12345",
        "farmId": "farm_sahara_001",
        "severity": "high",
        "category": "irrigation",
        "title": "Low Soil Moisture Detected",
        "message": "Soil moisture in Zone A1 below critical threshold",
        "timestamp": "2025-12-26T10:30:00Z",
        "details": {
          "zoneId": "zone_a1",
          "current_value": 8.5,
          "threshold": 12.0,
          "unit": "percent"
        },
        "status": "active",
        "auto_action_taken": true
      }
    ],
    "summary": {
      "total": 1,
      "active": 1,
      "acknowledged": 0,
      "resolved": 0
    }
  }
}
```

### 6.2 Acknowledge Alert

**Endpoint:** `POST /api/v1/alerts/{alertId}/acknowledge`

**Request Body:**

```json
{
  "acknowledgedBy": "user_john_doe",
  "note": "Investigating issue"
}
```

**Response:**

```json
{
  "success": true,
  "data": {
    "alertId": "alert_12345",
    "status": "acknowledged",
    "acknowledgedAt": "2025-12-26T10:35:00Z",
    "acknowledgedBy": "user_john_doe"
  }
}
```

### 6.3 Resolve Alert

**Endpoint:** `POST /api/v1/alerts/{alertId}/resolve`

**Request Body:**

```json
{
  "resolvedBy": "user_john_doe",
  "resolution": "Irrigation system activated",
  "preventiveMeasures": "Updated threshold to 15%"
}
```

---

## 7. Farm Management Endpoints

### 7.1 Get Farm Information

**Endpoint:** `GET /api/v1/farms/{farmId}`

**Example Response:**

```json
{
  "success": true,
  "data": {
    "farmId": "farm_sahara_001",
    "name": "Sahara Green Farm",
    "location": {
      "latitude": 31.7917,
      "longitude": -7.0926,
      "altitude": 450,
      "address": "Sahara Desert, Morocco"
    },
    "size": {
      "total": 50000,
      "cultivated": 35000,
      "unit": "m2"
    },
    "zones": [
      {
        "zoneId": "zone_a1",
        "name": "Greenhouse A1",
        "type": "greenhouse",
        "size": 5000,
        "crops": ["tomato"]
      }
    ],
    "sensors": {
      "total": 24,
      "active": 24,
      "types": {
        "soil_moisture": 8,
        "temperature": 6,
        "humidity": 4,
        "solar": 2,
        "wind": 2,
        "other": 2
      }
    },
    "status": "active",
    "created": "2024-01-15T00:00:00Z"
  }
}
```

### 7.2 Update Farm Configuration

**Endpoint:** `PUT /api/v1/farms/{farmId}/config`

**Request Body:**

```json
{
  "settings": {
    "auto_irrigation": true,
    "alert_notifications": true,
    "data_collection_interval": 300,
    "weather_integration": true
  },
  "thresholds": {
    "soil_moisture_min": 12,
    "temperature_max": 45,
    "humidity_min": 10
  }
}
```

---

## 8. Error Responses

### 8.1 Error Format

All error responses follow this structure:

```json
{
  "success": false,
  "error": {
    "code": "ERROR_CODE",
    "message": "Human-readable error message",
    "details": {
      "field": "Field that caused error",
      "value": "Invalid value",
      "expected": "Expected format"
    },
    "timestamp": "2025-12-26T10:30:00Z",
    "requestId": "req_12345"
  }
}
```

### 8.2 Common Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| INVALID_API_KEY | 401 | Invalid or missing API key |
| RATE_LIMIT_EXCEEDED | 429 | Too many requests |
| INVALID_PARAMETER | 400 | Invalid request parameter |
| RESOURCE_NOT_FOUND | 404 | Resource not found |
| VALIDATION_ERROR | 422 | Request validation failed |
| SERVER_ERROR | 500 | Internal server error |
| SERVICE_UNAVAILABLE | 503 | Service temporarily unavailable |

### 8.3 Example Error Response

```json
{
  "success": false,
  "error": {
    "code": "INVALID_PARAMETER",
    "message": "Invalid farmId parameter",
    "details": {
      "field": "farmId",
      "value": "invalid_farm",
      "expected": "Valid farm identifier (e.g., farm_sahara_001)"
    },
    "timestamp": "2025-12-26T10:30:00Z",
    "requestId": "req_abc123"
  }
}
```

---

## 9. Webhooks

### 9.1 Webhook Configuration

**Endpoint:** `POST /api/v1/webhooks`

**Request Body:**

```json
{
  "url": "https://your-server.com/webhook",
  "events": [
    "sensor.data",
    "alert.critical",
    "irrigation.complete",
    "harvest.ready"
  ],
  "secret": "your_webhook_secret",
  "active": true
}
```

**Response:**

```json
{
  "success": true,
  "data": {
    "webhookId": "webhook_12345",
    "url": "https://your-server.com/webhook",
    "events": ["sensor.data", "alert.critical"],
    "status": "active",
    "created": "2025-12-26T10:30:00Z"
  }
}
```

### 9.2 Webhook Payload

```json
{
  "event": "alert.critical",
  "timestamp": "2025-12-26T10:30:00Z",
  "farmId": "farm_sahara_001",
  "data": {
    "alertId": "alert_12345",
    "severity": "critical",
    "message": "Equipment failure detected"
  },
  "signature": "sha256_hash_of_payload"
}
```

---

## 10. SDK Examples

### 10.1 Node.js

```javascript
const WIA = require('wia-agri-021-sdk');

const client = new WIA.Client({
  apiKey: process.env.WIA_API_KEY,
  endpoint: 'https://api.wia-agri-021.org/v1'
});

// Get sensor data
const sensorData = await client.sensors.getData({
  farmId: 'farm_sahara_001',
  sensorType: 'soil_moisture'
});

// Start irrigation
await client.irrigation.start({
  farmId: 'farm_sahara_001',
  zoneId: 'zone_a1',
  duration: 1800
});
```

### 10.2 Python

```python
from wia_agri_021 import Client

client = Client(
    api_key=os.environ['WIA_API_KEY'],
    endpoint='https://api.wia-agri-021.org/v1'
)

# Get yield prediction
prediction = client.analytics.predict_yield(
    farm_id='farm_sahara_001',
    crop_type='tomato'
)

print(f"Expected yield: {prediction['estimatedYield']} kg")
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

© 2025 SmileStory Inc. / WIA · All Rights Reserved
