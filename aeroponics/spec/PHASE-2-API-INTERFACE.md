# WIA-AGRI-028: Aeroponics Standard
## Phase 2: API Interface Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-01-01

---

## 1. Overview

This specification defines the RESTful API interfaces for aeroponic systems, enabling programmatic control of misting cycles, nutrient delivery, environmental monitoring, and harvest management.

### 1.1 Base URL

```
https://api.aeroponics.example.com/v1
```

### 1.2 Authentication

All API requests require authentication using API keys or OAuth 2.0 tokens.

**API Key Header:**
```
Authorization: Bearer {API_KEY}
X-API-Key: {API_KEY}
```

---

## 2. Misting System APIs

### 2.1 Get Misting Status

Retrieve current misting cycle information.

**Endpoint:** `GET /systems/{systemId}/misting`

**Response:**
```json
{
  "systemId": "AERO-SEOUL-001",
  "chamber": 3,
  "status": "active",
  "currentCycle": {
    "interval": 180,
    "duration": 5,
    "pressure": 85,
    "lastCycle": "2025-01-01T14:27:00Z",
    "nextCycle": "2025-01-01T14:30:00Z",
    "cyclesCompleted": 248
  },
  "nozzles": {
    "total": 24,
    "active": 24,
    "clogged": 0,
    "status": "optimal"
  },
  "pump": {
    "rpm": 3400,
    "temperature": 42,
    "pressure": 85,
    "status": "operational"
  }
}
```

### 2.2 Update Misting Configuration

Modify misting cycle parameters.

**Endpoint:** `PUT /systems/{systemId}/misting`

**Request:**
```json
{
  "chamber": 3,
  "configuration": {
    "interval": 150,
    "duration": 6,
    "pressure": 90
  }
}
```

**Response:**
```json
{
  "success": true,
  "message": "Misting configuration updated",
  "newConfiguration": {
    "interval": 150,
    "duration": 6,
    "pressure": 90,
    "appliedAt": "2025-01-01T14:35:00Z"
  }
}
```

### 2.3 Trigger Manual Mist Cycle

Execute immediate misting cycle.

**Endpoint:** `POST /systems/{systemId}/misting/manual`

**Request:**
```json
{
  "chamber": 3,
  "duration": 5,
  "pressure": 85
}
```

**Response:**
```json
{
  "success": true,
  "cycleId": "CYCLE-20250101-143500",
  "executedAt": "2025-01-01T14:35:00Z",
  "metrics": {
    "volume": 125,
    "dropletSize": 25,
    "coverage": 98.5
  }
}
```

---

## 3. Sensor APIs

### 3.1 Get All Sensors

Retrieve all sensor readings for a chamber.

**Endpoint:** `GET /systems/{systemId}/sensors?chamber={chamberId}`

**Response:**
```json
{
  "systemId": "AERO-SEOUL-001",
  "chamber": 3,
  "timestamp": "2025-01-01T14:30:00Z",
  "sensors": [
    {
      "id": "TEMP-01",
      "type": "temperature",
      "location": "air",
      "value": 22.5,
      "unit": "°C",
      "status": "normal"
    },
    {
      "id": "TEMP-02",
      "type": "temperature",
      "location": "root_zone",
      "value": 20.0,
      "unit": "°C",
      "status": "optimal"
    },
    {
      "id": "HUM-01",
      "type": "humidity",
      "location": "root_zone",
      "value": 95,
      "unit": "%",
      "status": "optimal"
    },
    {
      "id": "EC-01",
      "type": "ec",
      "location": "nutrient_solution",
      "value": 1.8,
      "unit": "mS/cm",
      "status": "normal"
    },
    {
      "id": "PH-01",
      "type": "ph",
      "location": "nutrient_solution",
      "value": 6.1,
      "unit": "pH",
      "status": "normal"
    },
    {
      "id": "PRESS-01",
      "type": "pressure",
      "location": "pump",
      "value": 85,
      "unit": "PSI",
      "status": "optimal"
    }
  ]
}
```

### 3.2 Get Specific Sensor

Retrieve data from a specific sensor.

**Endpoint:** `GET /systems/{systemId}/sensors/{sensorId}`

**Response:**
```json
{
  "sensorId": "TEMP-01",
  "type": "temperature",
  "location": "air",
  "currentValue": 22.5,
  "unit": "°C",
  "status": "normal",
  "history": [
    { "timestamp": "2025-01-01T14:25:00Z", "value": 22.3 },
    { "timestamp": "2025-01-01T14:20:00Z", "value": 22.4 },
    { "timestamp": "2025-01-01T14:15:00Z", "value": 22.6 }
  ],
  "calibration": {
    "lastCalibrated": "2024-12-15T00:00:00Z",
    "nextCalibration": "2025-03-15T00:00:00Z"
  }
}
```

---

## 4. Nutrient Management APIs

### 4.1 Get Nutrient Status

Retrieve nutrient solution information.

**Endpoint:** `GET /systems/{systemId}/nutrients`

**Response:**
```json
{
  "systemId": "AERO-SEOUL-001",
  "timestamp": "2025-01-01T14:30:00Z",
  "solution": {
    "ec": 1.8,
    "ph": 6.1,
    "temperature": 20,
    "dissolvedOxygen": 8.5
  },
  "nutrients": {
    "nitrogen": 180,
    "phosphorus": 60,
    "potassium": 220,
    "calcium": 200,
    "magnesium": 60
  },
  "reservoir": {
    "volume": 850,
    "capacity": 1000,
    "level": 85,
    "status": "sufficient",
    "lastRefill": "2024-12-28T10:00:00Z"
  },
  "recommendations": [
    {
      "type": "refill",
      "priority": "low",
      "estimatedDays": 12,
      "message": "Reservoir at 85%, refill recommended in 12 days"
    }
  ]
}
```

### 4.2 Adjust Nutrient Levels

Modify nutrient concentration.

**Endpoint:** `POST /systems/{systemId}/nutrients/adjust`

**Request:**
```json
{
  "adjustments": {
    "nitrogen": "+20",
    "potassium": "+10"
  },
  "targetEC": 2.0,
  "reason": "Growth stage transition to flowering"
}
```

**Response:**
```json
{
  "success": true,
  "message": "Nutrient adjustment scheduled",
  "newLevels": {
    "nitrogen": 200,
    "potassium": 230,
    "ec": 2.0
  },
  "appliedAt": "2025-01-01T14:40:00Z"
}
```

---

## 5. Root Zone Monitoring APIs

### 5.1 Get Root Zone Status

Retrieve root zone health metrics.

**Endpoint:** `GET /systems/{systemId}/root-zone?chamber={chamberId}`

**Response:**
```json
{
  "systemId": "AERO-SEOUL-001",
  "chamber": 3,
  "timestamp": "2025-01-01T14:30:00Z",
  "environment": {
    "oxygenLevel": 21.5,
    "humidity": 95,
    "temperature": 20.5,
    "airflow": "optimal"
  },
  "rootMetrics": {
    "rootDensity": "high",
    "rootColor": "white",
    "rootLength": 25,
    "healthScore": 95,
    "issues": []
  },
  "moisture": {
    "postMist": 100,
    "preMist": 45,
    "status": "optimal"
  }
}
```

---

## 6. Crop Management APIs

### 6.1 Get Crop Status

Retrieve current crop information.

**Endpoint:** `GET /systems/{systemId}/crops?chamber={chamberId}`

**Response:**
```json
{
  "systemId": "AERO-SEOUL-001",
  "chamber": 3,
  "batchId": "BATCH-2025-001",
  "crop": {
    "type": "basil",
    "variety": "Genovese",
    "plantCount": 375
  },
  "timeline": {
    "transplantDate": "2024-12-12T00:00:00Z",
    "expectedHarvest": "2025-01-05T00:00:00Z",
    "daysToHarvest": 4,
    "growthDays": 31
  },
  "growth": {
    "stage": "late_vegetative",
    "averageHeight": 18.5,
    "healthScore": 96,
    "growthRate": "+45%"
  },
  "readiness": {
    "harvestReady": false,
    "estimatedYield": 18.75,
    "unit": "kg"
  }
}
```

### 6.2 Create Harvest Record

Record harvest data.

**Endpoint:** `POST /systems/{systemId}/harvests`

**Request:**
```json
{
  "chamber": 3,
  "batchId": "BATCH-2025-001",
  "harvestDate": "2025-01-05T10:00:00Z",
  "plantsHarvested": 375,
  "totalWeight": 18.75,
  "quality": {
    "grade": "Premium",
    "brixLevel": 8.5
  },
  "destination": {
    "type": "restaurant",
    "name": "Seoul Organic Bistro"
  }
}
```

**Response:**
```json
{
  "success": true,
  "harvestId": "HRV-AERO-20250105-001",
  "metrics": {
    "yieldPerPlant": 50,
    "waterEfficiency": 98,
    "growthCycle": 31
  },
  "qualityScore": 96,
  "createdAt": "2025-01-05T10:05:00Z"
}
```

---

## 7. Alert & Notification APIs

### 7.1 Get Active Alerts

Retrieve system alerts.

**Endpoint:** `GET /systems/{systemId}/alerts?status=active`

**Response:**
```json
{
  "systemId": "AERO-SEOUL-001",
  "alerts": [
    {
      "alertId": "ALERT-001",
      "severity": "warning",
      "type": "pressure_low",
      "message": "Misting pressure below optimal range",
      "chamber": 3,
      "currentValue": 65,
      "threshold": 75,
      "createdAt": "2025-01-01T14:25:00Z",
      "recommendations": [
        "Check pump performance",
        "Inspect pressure regulator",
        "Verify nozzle flow"
      ]
    }
  ]
}
```

### 7.2 Subscribe to Webhooks

Register webhook for real-time notifications.

**Endpoint:** `POST /systems/{systemId}/webhooks`

**Request:**
```json
{
  "url": "https://your-server.com/webhook",
  "events": [
    "misting.cycle.completed",
    "alert.critical",
    "harvest.ready",
    "nutrient.low"
  ],
  "secret": "webhook_secret_key"
}
```

**Response:**
```json
{
  "success": true,
  "webhookId": "WH-001",
  "status": "active",
  "createdAt": "2025-01-01T14:30:00Z"
}
```

---

## 8. Analytics APIs

### 8.1 Get Performance Metrics

Retrieve system performance data.

**Endpoint:** `GET /systems/{systemId}/analytics?period=30d`

**Response:**
```json
{
  "systemId": "AERO-SEOUL-001",
  "period": "30d",
  "metrics": {
    "totalHarvests": 12,
    "totalYield": 225,
    "averageYield": 18.75,
    "waterUsage": 900,
    "waterEfficiency": 98,
    "energyUsage": 2550,
    "growthRate": "+45%",
    "cropHealth": 95,
    "systemUptime": 99.8
  },
  "trends": {
    "yieldTrend": "increasing",
    "efficiencyTrend": "stable",
    "healthTrend": "excellent"
  }
}
```

---

## 9. Error Responses

### 9.1 Standard Error Format

```json
{
  "error": {
    "code": "AERO-1001",
    "message": "Misting pressure too low",
    "severity": "warning",
    "timestamp": "2025-01-01T14:30:00Z",
    "details": {
      "currentValue": 65,
      "requiredValue": 75,
      "chamber": 3
    },
    "recommendations": [
      "Check pump performance",
      "Inspect pressure regulator"
    ]
  }
}
```

### 9.2 HTTP Status Codes

| Code | Description |
|------|-------------|
| 200 | Success |
| 201 | Created |
| 400 | Bad Request |
| 401 | Unauthorized |
| 403 | Forbidden |
| 404 | Not Found |
| 429 | Too Many Requests |
| 500 | Internal Server Error |
| 503 | Service Unavailable |

---

## 10. Rate Limiting

- **Standard Tier**: 100 requests/minute
- **Professional Tier**: 500 requests/minute
- **Enterprise Tier**: Unlimited

**Rate Limit Headers:**
```
X-RateLimit-Limit: 100
X-RateLimit-Remaining: 87
X-RateLimit-Reset: 1704117600
```

---

## 11. SDK Examples

### 11.1 JavaScript/TypeScript

```typescript
import { AeroponicsClient } from '@wia/aeroponics-sdk';

const client = new AeroponicsClient({
  apiKey: process.env.API_KEY,
  baseURL: 'https://api.aeroponics.example.com/v1'
});

// Get misting status
const misting = await client.misting.getStatus('AERO-SEOUL-001');

// Update misting configuration
await client.misting.updateConfig('AERO-SEOUL-001', {
  chamber: 3,
  interval: 150,
  duration: 6,
  pressure: 90
});

// Get sensor data
const sensors = await client.sensors.getAll('AERO-SEOUL-001', { chamber: 3 });

// Subscribe to alerts
client.on('alert.critical', (alert) => {
  console.log('Critical alert:', alert);
});
```

### 11.2 Python

```python
from wia_aeroponics import AeroponicsClient

client = AeroponicsClient(
    api_key=os.environ['API_KEY'],
    base_url='https://api.aeroponics.example.com/v1'
)

# Get misting status
misting = client.misting.get_status('AERO-SEOUL-001')

# Update misting configuration
client.misting.update_config('AERO-SEOUL-001',
    chamber=3,
    interval=150,
    duration=6,
    pressure=90
)

# Get sensor data
sensors = client.sensors.get_all('AERO-SEOUL-001', chamber=3)
```

---

**Document History:**
- v1.0.0 (2025-01-01): Initial release

© 2025 WIA Standards · MIT License
弘익人間 (Benefit All Humanity)
