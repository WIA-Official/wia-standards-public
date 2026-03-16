# WIA-AGRI-035: Space Agriculture Standard
## Phase 2: API Interface Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-12-26

---

## 1. Overview

This specification defines RESTful APIs and real-time interfaces for space agriculture systems, enabling data submission, environmental monitoring, harvest tracking, and integration with mission control systems.

### 1.1 API Principles

- **Reliability**: Fault-tolerant with automatic retry mechanisms
- **Bandwidth Efficiency**: Optimized for limited space-to-ground communication
- **Real-time**: MQTT for time-sensitive telemetry
- **Asynchronous**: Store-and-forward during communication blackouts
- **Security**: TLS encryption, authentication, and authorization

---

## 2. Base Configuration

### 2.1 Base URL
```
Production: https://api.wia.space/v1
Staging: https://api-staging.wia.space/v1
ISS: https://iss-gateway.nasa.gov/wia/v1
```

### 2.2 Authentication
```http
Authorization: Bearer <JWT_TOKEN>
X-API-Key: <MODULE_API_KEY>
X-Mission-ID: <MISSION_IDENTIFIER>
```

### 2.3 Common Headers
```http
Content-Type: application/json
Accept: application/json
X-WIA-Standard: AGRI-035-v1.0
X-Module-ID: ISS-VEGGIE-001
X-Timestamp: 2025-12-26T10:30:00.000Z
```

---

## 3. REST API Endpoints

### 3.1 Submit Space Agriculture Data

**Endpoint:** `POST /api/v1/space-agri/submit`

**Description:** Submit environmental data, sensor readings, and crop status from space agriculture modules.

**Request:**
```http
POST /api/v1/space-agri/submit
Content-Type: application/json
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...

{
  "spaceAgriculture": {
    "standardVersion": "WIA-AGRI-035-v1.0",
    "dataId": "SPACE-AGRI-20251226-001",
    "timestamp": "2025-12-26T10:30:00.000Z",
    "moduleId": "ISS-VEGGIE-001",
    "location": {
      "type": "LEO",
      "facility": "ISS",
      "altitude": 408000
    },
    "environment": {
      "gravity": 0.0,
      "temperature": 22.5,
      "humidity": 65,
      "co2Level": 800,
      "o2Level": 21.5
    },
    "cropData": {
      "cropType": "lettuce-romaine",
      "plantCount": 6,
      "currentDay": 28,
      "growthStage": "vegetative"
    }
  }
}
```

**Response (200 OK):**
```json
{
  "status": "success",
  "message": "Space agriculture data submitted successfully",
  "data": {
    "submissionId": "SUB-20251226-001",
    "moduleId": "ISS-VEGGIE-001",
    "receivedAt": "2025-12-26T10:30:15.250Z",
    "validated": true,
    "nextDataWindow": "2025-12-26T11:00:00Z",
    "storageLocation": "s3://wia-space-agri/ISS-VEGGIE-001/2025/12/26/",
    "acknowledgment": {
      "missionControl": true,
      "dataArchive": true,
      "researchPortal": true
    }
  }
}
```

**Error Response (400 Bad Request):**
```json
{
  "status": "error",
  "errorCode": "VALIDATION_FAILED",
  "message": "Invalid temperature value: must be between 18-26°C",
  "field": "environment.temperature",
  "receivedValue": 30.5,
  "allowedRange": {
    "min": 18,
    "max": 26
  }
}
```

---

### 3.2 Get Module Status

**Endpoint:** `GET /api/v1/space-agri/module/:moduleId`

**Description:** Retrieve current status and latest readings from a specific module.

**Request:**
```http
GET /api/v1/space-agri/module/ISS-VEGGIE-001
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

**Response (200 OK):**
```json
{
  "status": "success",
  "data": {
    "moduleId": "ISS-VEGGIE-001",
    "moduleType": "VEGGIE",
    "status": "OPERATIONAL",
    "lastUpdate": "2025-12-26T10:30:00Z",
    "currentCrop": {
      "cropType": "lettuce-romaine",
      "plantCount": 6,
      "daysSinceGermination": 28,
      "growthStage": "vegetative",
      "expectedHarvest": "2025-12-30",
      "health": "EXCELLENT"
    },
    "environment": {
      "temperature": 22.5,
      "humidity": 65,
      "co2": 800,
      "o2": 21.5,
      "lightCycle": "ON",
      "nextLightChange": "2025-12-26T18:30:00Z"
    },
    "systemHealth": {
      "waterPump": "NOMINAL",
      "airCirculation": "NOMINAL",
      "ledArray": "NOMINAL",
      "sensors": "NOMINAL",
      "powerConsumption": 220
    },
    "alerts": []
  }
}
```

---

### 3.3 Submit Harvest Data

**Endpoint:** `POST /api/v1/space-agri/harvest`

**Description:** Record harvest events and yield data.

**Request:**
```http
POST /api/v1/space-agri/harvest
Content-Type: application/json
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...

{
  "harvestEvent": {
    "eventId": "HARVEST-20251230-001",
    "moduleId": "ISS-VEGGIE-001",
    "timestamp": "2025-12-30T14:00:00.000Z",
    "cropType": "lettuce-romaine",
    "plantCount": 6,
    "yield": {
      "freshWeight": {
        "total": 178.5,
        "perPlant": 29.75,
        "unit": "grams"
      },
      "edibleWeight": {
        "total": 165.2,
        "unit": "grams"
      }
    },
    "qualityAssessment": {
      "appearance": "EXCELLENT",
      "taste": "FRESH",
      "microbialTest": "PASS"
    },
    "crewNotes": "Exceptional quality. Crew morale boost confirmed.",
    "harvestedBy": "Astronaut-Smith"
  }
}
```

**Response (200 OK):**
```json
{
  "status": "success",
  "message": "Harvest data recorded successfully",
  "data": {
    "harvestId": "HARVEST-20251230-001",
    "moduleId": "ISS-VEGGIE-001",
    "totalYield": 178.5,
    "efficiency": 98,
    "comparisonToTarget": 99.2,
    "nutritionProvided": {
      "calories": 50,
      "vitaminC": "15% daily value",
      "vitaminK": "145% daily value"
    },
    "nextCycleRecommendation": {
      "cropType": "mizuna",
      "germination": "2025-12-31",
      "expectedHarvest": "2026-01-28"
    }
  }
}
```

---

### 3.4 Get Historical Data

**Endpoint:** `GET /api/v1/space-agri/history/:moduleId`

**Description:** Retrieve historical data for analysis and trending.

**Request:**
```http
GET /api/v1/space-agri/history/ISS-VEGGIE-001?
    startDate=2025-11-01&
    endDate=2025-12-26&
    dataType=environment&
    interval=1h
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

**Query Parameters:**
- `startDate`: ISO 8601 date (required)
- `endDate`: ISO 8601 date (required)
- `dataType`: environment, crop, harvest, eclss (optional)
- `interval`: 1m, 5m, 15m, 1h, 6h, 1d (default: 1h)
- `format`: json, csv (default: json)

**Response (200 OK):**
```json
{
  "status": "success",
  "data": {
    "moduleId": "ISS-VEGGIE-001",
    "period": {
      "start": "2025-11-01T00:00:00Z",
      "end": "2025-12-26T23:59:59Z"
    },
    "interval": "1h",
    "dataPoints": 1320,
    "readings": [
      {
        "timestamp": "2025-11-01T00:00:00Z",
        "temperature": 22.8,
        "humidity": 63,
        "co2": 790,
        "o2": 21.6
      },
      {
        "timestamp": "2025-11-01T01:00:00Z",
        "temperature": 22.6,
        "humidity": 64,
        "co2": 795,
        "o2": 21.5
      }
    ],
    "statistics": {
      "temperature": {
        "avg": 22.5,
        "min": 21.8,
        "max": 23.2,
        "stdDev": 0.35
      },
      "humidity": {
        "avg": 64.8,
        "min": 60,
        "max": 70,
        "stdDev": 2.1
      }
    }
  }
}
```

---

### 3.5 Get ECLSS Integration Data

**Endpoint:** `GET /api/v1/space-agri/eclss/:moduleId`

**Description:** Retrieve life support integration metrics.

**Request:**
```http
GET /api/v1/space-agri/eclss/ISS-VEGGIE-001
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

**Response (200 OK):**
```json
{
  "status": "success",
  "data": {
    "moduleId": "ISS-VEGGIE-001",
    "timestamp": "2025-12-26T10:30:00Z",
    "gasExchange": {
      "o2Production": {
        "value": 12.5,
        "unit": "grams/day",
        "crewEquivalent": 0.15
      },
      "co2Consumption": {
        "value": 15.2,
        "unit": "grams/day",
        "crewEquivalent": 0.18
      }
    },
    "waterRecycling": {
      "condensateRecovered": 2.1,
      "nutrientRecycled": 95.5,
      "efficiency": 98.2
    },
    "powerConsumption": {
      "current": 220,
      "peak": 280,
      "average24h": 185,
      "unit": "watts"
    },
    "contribution": {
      "lifeSupportPercentage": 5.2,
      "foodProductionPercentage": 2.8,
      "crewMorale": "HIGH"
    }
  }
}
```

---

### 3.6 Alert Management

**Endpoint:** `GET /api/v1/space-agri/alerts/:moduleId`

**Description:** Retrieve active alerts and warnings.

**Request:**
```http
GET /api/v1/space-agri/alerts/ISS-VEGGIE-001?severity=CRITICAL
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

**Response (200 OK):**
```json
{
  "status": "success",
  "data": {
    "moduleId": "ISS-VEGGIE-001",
    "activeAlerts": [
      {
        "alertId": "ALERT-20251226-001",
        "severity": "WARNING",
        "type": "WATER_LEVEL_LOW",
        "message": "Water reservoir at 25% capacity",
        "timestamp": "2025-12-26T09:15:00Z",
        "currentValue": 25,
        "threshold": 30,
        "recommendation": "Refill water reservoir within 48 hours",
        "acknowledged": false
      }
    ],
    "resolvedAlerts": [
      {
        "alertId": "ALERT-20251225-003",
        "severity": "INFO",
        "type": "HARVEST_READY",
        "message": "Lettuce ready for harvest",
        "timestamp": "2025-12-25T12:00:00Z",
        "resolvedAt": "2025-12-26T10:00:00Z",
        "resolvedBy": "Astronaut-Smith"
      }
    ]
  }
}
```

---

## 4. MQTT Real-Time Interface

### 4.1 MQTT Broker Configuration
```
Broker: mqtt.wia.space
Port: 8883 (TLS)
Protocol: MQTT 5.0
QoS: 1 (At least once delivery)
Retain: true (for latest values)
Keep-Alive: 60 seconds
```

### 4.2 Topic Structure
```
wia/space-agri/{moduleId}/environment
wia/space-agri/{moduleId}/sensors
wia/space-agri/{moduleId}/crop
wia/space-agri/{moduleId}/alerts
wia/space-agri/{moduleId}/eclss
wia/space-agri/{moduleId}/command
wia/space-agri/{moduleId}/status
```

### 4.3 Publish: Environment Data
```json
Topic: wia/space-agri/ISS-VEGGIE-001/environment
QoS: 1
Retain: true

{
  "timestamp": "2025-12-26T10:30:00.000Z",
  "temperature": 22.5,
  "humidity": 65,
  "co2": 800,
  "o2": 21.5,
  "pressure": 101.3,
  "lightCycle": "ON"
}
```

### 4.4 Publish: Critical Alert
```json
Topic: wia/space-agri/ISS-VEGGIE-001/alerts
QoS: 2
Retain: false

{
  "alertId": "ALERT-20251226-002",
  "severity": "CRITICAL",
  "type": "LED_ARRAY_FAILURE",
  "timestamp": "2025-12-26T10:35:00.000Z",
  "message": "LED array power failure detected",
  "actionRequired": "IMMEDIATE",
  "backupActivated": true
}
```

### 4.5 Subscribe: Commands
```json
Topic: wia/space-agri/ISS-VEGGIE-001/command
QoS: 2

{
  "commandId": "CMD-20251226-001",
  "timestamp": "2025-12-26T10:40:00.000Z",
  "command": "SET_LIGHT_CYCLE",
  "parameters": {
    "photoperiod": "18h-on-6h-off",
    "startTime": "2025-12-27T00:00:00Z"
  },
  "issuedBy": "MissionControl-Houston"
}
```

---

## 5. Batch Operations

### 5.1 Bulk Data Upload

**Endpoint:** `POST /api/v1/space-agri/bulk-upload`

**Description:** Upload multiple data points in a single request (for post-blackout sync).

**Request:**
```http
POST /api/v1/space-agri/bulk-upload
Content-Type: application/json
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...

{
  "moduleId": "ISS-VEGGIE-001",
  "uploadReason": "COMM_BLACKOUT_RECOVERY",
  "dataPoints": [
    {
      "timestamp": "2025-12-26T08:00:00Z",
      "environment": { "temperature": 22.3, "humidity": 64, "co2": 795 }
    },
    {
      "timestamp": "2025-12-26T08:15:00Z",
      "environment": { "temperature": 22.5, "humidity": 65, "co2": 800 }
    }
  ]
}
```

**Response (200 OK):**
```json
{
  "status": "success",
  "message": "Bulk upload completed",
  "data": {
    "uploaded": 120,
    "failed": 0,
    "duplicates": 5,
    "timeRange": {
      "start": "2025-12-26T08:00:00Z",
      "end": "2025-12-26T10:00:00Z"
    }
  }
}
```

---

## 6. WebSocket Interface (Optional)

### 6.1 Connection
```javascript
const ws = new WebSocket('wss://api.wia.space/v1/space-agri/stream');

ws.on('open', () => {
  ws.send(JSON.stringify({
    action: 'subscribe',
    moduleId: 'ISS-VEGGIE-001',
    dataTypes: ['environment', 'alerts']
  }));
});

ws.on('message', (data) => {
  const update = JSON.parse(data);
  console.log('Real-time update:', update);
});
```

### 6.2 Server Push
```json
{
  "type": "environment_update",
  "moduleId": "ISS-VEGGIE-001",
  "timestamp": "2025-12-26T10:30:00Z",
  "data": {
    "temperature": 22.5,
    "humidity": 65,
    "co2": 800
  }
}
```

---

## 7. Rate Limiting & Bandwidth

### 7.1 Rate Limits
- **REST API**: 100 requests/minute per module
- **MQTT Publish**: 10 messages/second per topic
- **Bulk Upload**: 1 request/minute, max 1000 data points
- **WebSocket**: 1 connection per module

### 7.2 Bandwidth Optimization
- **Compression**: GZIP enabled (60-70% reduction)
- **Delta Encoding**: Send only changed values
- **Aggregation**: Batch sensor readings every 5 minutes
- **Priority**: Critical alerts > real-time telemetry > historical data

---

## 8. Error Codes

| Code | Message | Description |
|------|---------|-------------|
| 400 | BAD_REQUEST | Invalid request format or parameters |
| 401 | UNAUTHORIZED | Missing or invalid authentication token |
| 403 | FORBIDDEN | Insufficient permissions for this operation |
| 404 | NOT_FOUND | Module or resource not found |
| 409 | CONFLICT | Duplicate submission or state conflict |
| 422 | VALIDATION_ERROR | Data validation failed |
| 429 | RATE_LIMIT | Too many requests |
| 500 | INTERNAL_ERROR | Server error |
| 503 | SERVICE_UNAVAILABLE | Temporary service outage |

---

## 9. SDK Examples

### 9.1 Python SDK
```python
from wia_space_agri import SpaceAgriClient

client = SpaceAgriClient(
    api_key='your-api-key',
    module_id='ISS-VEGGIE-001'
)

# Submit data
response = client.submit_data({
    'environment': {
        'temperature': 22.5,
        'humidity': 65,
        'co2': 800
    }
})

# Get module status
status = client.get_module_status()
print(f"Crop health: {status['currentCrop']['health']}")
```

### 9.2 JavaScript SDK
```javascript
const { SpaceAgriClient } = require('@wia/space-agri');

const client = new SpaceAgriClient({
  apiKey: 'your-api-key',
  moduleId: 'ISS-VEGGIE-001'
});

// Subscribe to real-time updates
client.subscribe('environment', (data) => {
  console.log('Temperature:', data.temperature);
});

// Submit harvest data
await client.submitHarvest({
  cropType: 'lettuce-romaine',
  yield: { total: 178.5, unit: 'grams' }
});
```

---

**Document Version:** 1.0.0
**Effective Date:** 2025-12-26
**Next Review:** 2026-06-26

---

© 2025 WIA - World Certification Industry Association
弘益人間 (Benefit All Humanity)
