# WIA-AGRI-004: Agricultural Drone Standard
## Phase 2: API Interface Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-01-01

---

## 1. Overview

This specification defines RESTful and WebSocket APIs for agricultural drone operations, including mission management, real-time telemetry, imagery processing, and farm management integration.

### 1.1 Base URL

```
Production: https://api.wia-agri-drone.org/v1
Sandbox:    https://sandbox.wia-agri-drone.org/v1
```

### 1.2 Authentication

All API requests require Bearer token authentication:

```http
Authorization: Bearer YOUR_JWT_TOKEN
```

---

## 2. Mission Management API

### 2.1 Create Mission

**Endpoint:** `POST /missions`

**Request:**
```json
{
  "farmId": "FARM-KR-001",
  "operatorId": "OPERATOR-001",
  "droneId": "AGRI-DRONE-001",
  "missionType": "CROP_SPRAYING",
  "scheduledStart": "2025-01-01T10:00:00Z",
  "fieldBoundary": {
    "type": "Polygon",
    "coordinates": [[[126.9780, 37.5665], [126.9790, 37.5675]]]
  },
  "parameters": {
    "altitude": 5.0,
    "speed": 3.5,
    "swathWidth": 4.5,
    "overlap": 0.2
  },
  "chemical": {
    "name": "Organic Pesticide A",
    "concentration": 2.5,
    "sprayRate": 15.0
  }
}
```

**Response:** `201 Created`
```json
{
  "success": true,
  "data": {
    "missionId": "MISSION-2025-001",
    "status": "PLANNED",
    "estimatedDuration": 45,
    "estimatedArea": 10.5,
    "waypointCount": 24,
    "estimatedFluidUsage": 157.5,
    "refillsRequired": 2,
    "createdAt": "2025-01-01T09:00:00Z"
  }
}
```

### 2.2 Get Mission Details

**Endpoint:** `GET /missions/{missionId}`

**Response:** `200 OK`
```json
{
  "success": true,
  "data": {
    "missionId": "MISSION-2025-001",
    "status": "IN_PROGRESS",
    "progress": 67.5,
    "startedAt": "2025-01-01T10:00:00Z",
    "currentWaypoint": 16,
    "totalWaypoints": 24,
    "areaCovered": 7.1,
    "totalArea": 10.5,
    "fluidUsed": 106.5,
    "batteryRemaining": 42,
    "estimatedCompletion": "2025-01-01T10:35:00Z"
  }
}
```

### 2.3 Update Mission Status

**Endpoint:** `PATCH /missions/{missionId}`

**Request:**
```json
{
  "status": "PAUSED",
  "reason": "Weather conditions deteriorating"
}
```

**Response:** `200 OK`
```json
{
  "success": true,
  "data": {
    "missionId": "MISSION-2025-001",
    "status": "PAUSED",
    "updatedAt": "2025-01-01T10:15:00Z"
  }
}
```

### 2.4 List Missions

**Endpoint:** `GET /missions`

**Query Parameters:**
- `farmId` (optional): Filter by farm
- `status` (optional): PLANNED, IN_PROGRESS, COMPLETED, PAUSED, CANCELLED
- `fromDate` (optional): ISO 8601 date
- `toDate` (optional): ISO 8601 date
- `page` (default: 1)
- `pageSize` (default: 50, max: 100)

**Response:** `200 OK`
```json
{
  "success": true,
  "data": {
    "missions": [
      {
        "missionId": "MISSION-2025-001",
        "farmId": "FARM-KR-001",
        "missionType": "CROP_SPRAYING",
        "status": "COMPLETED",
        "scheduledStart": "2025-01-01T10:00:00Z",
        "completedAt": "2025-01-01T10:45:00Z",
        "areaCovered": 10.5
      }
    ],
    "total": 234,
    "page": 1,
    "pageSize": 50
  }
}
```

---

## 3. Real-Time Telemetry API

### 3.1 WebSocket Connection

**Endpoint:** `wss://api.wia-agri-drone.org/v1/telemetry`

**Connection:**
```javascript
const ws = new WebSocket('wss://api.wia-agri-drone.org/v1/telemetry');

// Subscribe to drone telemetry
ws.send(JSON.stringify({
  action: 'subscribe',
  droneId: 'AGRI-DRONE-001',
  token: 'YOUR_JWT_TOKEN'
}));

// Receive real-time telemetry
ws.onmessage = (event) => {
  const telemetry = JSON.parse(event.data);
  console.log(telemetry);
};
```

**Telemetry Message:**
```json
{
  "type": "telemetry",
  "droneId": "AGRI-DRONE-001",
  "timestamp": "2025-01-01T10:30:45.123Z",
  "location": {
    "latitude": 37.5665,
    "longitude": 126.9780,
    "altitude": 5.2
  },
  "velocity": {
    "groundSpeed": 3.5,
    "heading": 87.4
  },
  "battery": {
    "percentage": 78,
    "voltage": 22.4
  },
  "status": "SPRAYING",
  "sprayActive": true,
  "tankLevel": 65
}
```

### 3.2 Get Historical Telemetry

**Endpoint:** `GET /telemetry/{droneId}`

**Query Parameters:**
- `fromTime`: ISO 8601 timestamp
- `toTime`: ISO 8601 timestamp
- `interval`: 1s, 5s, 10s, 30s, 1m, 5m (default: 1s)

**Response:** `200 OK`
```json
{
  "success": true,
  "data": {
    "droneId": "AGRI-DRONE-001",
    "interval": "5s",
    "records": [
      {
        "timestamp": "2025-01-01T10:30:00Z",
        "latitude": 37.5665,
        "longitude": 126.9780,
        "altitude": 5.2,
        "speed": 3.5,
        "battery": 78
      }
    ],
    "count": 540
  }
}
```

---

## 4. Imagery & Analysis API

### 4.1 Upload Aerial Image

**Endpoint:** `POST /images`

**Request:** `multipart/form-data`
```
Content-Type: multipart/form-data

file: [binary image data]
metadata: {
  "missionId": "MISSION-2025-001",
  "timestamp": "2025-01-01T10:35:22Z",
  "location": {"latitude": 37.5670, "longitude": 126.9785},
  "camera": "AGRI-CAM-MS5",
  "bands": ["RED", "NIR", "GREEN"]
}
```

**Response:** `201 Created`
```json
{
  "success": true,
  "data": {
    "imageId": "IMG-2025-001-0123",
    "uploadedAt": "2025-01-01T10:36:00Z",
    "fileSize": 15728640,
    "url": "https://cdn.wia-agri.org/images/IMG-2025-001-0123.tif",
    "thumbnailUrl": "https://cdn.wia-agri.org/images/IMG-2025-001-0123-thumb.jpg"
  }
}
```

### 4.2 Request NDVI Analysis

**Endpoint:** `POST /analysis/ndvi`

**Request:**
```json
{
  "imageId": "IMG-2025-001-0123",
  "fieldId": "FIELD-KR-001",
  "algorithm": "STANDARD_NDVI",
  "options": {
    "generateZones": true,
    "generateReport": true,
    "minHealthThreshold": 0.3
  }
}
```

**Response:** `202 Accepted`
```json
{
  "success": true,
  "data": {
    "analysisId": "NDVI-2025-001",
    "status": "PROCESSING",
    "estimatedCompletion": "2025-01-01T10:40:00Z",
    "webhookUrl": "https://your-app.com/webhooks/ndvi"
  }
}
```

### 4.3 Get NDVI Analysis Results

**Endpoint:** `GET /analysis/ndvi/{analysisId}`

**Response:** `200 OK`
```json
{
  "success": true,
  "data": {
    "analysisId": "NDVI-2025-001",
    "status": "COMPLETED",
    "completedAt": "2025-01-01T10:39:15Z",
    "statistics": {
      "mean": 0.65,
      "median": 0.67,
      "stdDev": 0.12,
      "min": 0.25,
      "max": 0.85
    },
    "healthZones": [
      {
        "category": "EXCELLENT",
        "area": 4.5,
        "percentage": 42.8
      },
      {
        "category": "GOOD",
        "area": 5.2,
        "percentage": 49.5
      }
    ],
    "outputFiles": {
      "ndviMap": "https://cdn.wia-agri.org/analysis/NDVI-2025-001-map.tif",
      "report": "https://cdn.wia-agri.org/analysis/NDVI-2025-001-report.pdf"
    }
  }
}
```

### 4.4 List Images

**Endpoint:** `GET /images`

**Query Parameters:**
- `missionId` (optional)
- `fieldId` (optional)
- `fromDate`, `toDate` (optional)
- `bandType` (optional): RGB, MULTISPECTRAL, THERMAL
- `page`, `pageSize`

---

## 5. Spray Operation API

### 5.1 Log Spray Operation

**Endpoint:** `POST /spray-operations`

**Request:**
```json
{
  "missionId": "MISSION-2025-001",
  "chemical": {
    "name": "Organic Pesticide A",
    "registrationNumber": "KR-PEST-2024-123",
    "concentration": 2.5
  },
  "application": {
    "tankVolume": 10.0,
    "sprayRate": 15.0,
    "swathWidth": 4.5
  },
  "environmental": {
    "temperature": 22.5,
    "humidity": 65,
    "windSpeed": 2.3
  }
}
```

**Response:** `201 Created`
```json
{
  "success": true,
  "data": {
    "operationId": "SPRAY-2025-001",
    "startedAt": "2025-01-01T10:30:00Z"
  }
}
```

### 5.2 Update Spray Coverage

**Endpoint:** `PATCH /spray-operations/{operationId}`

**Request:**
```json
{
  "areaCovered": 10.3,
  "volumeSprayed": 155.0,
  "completedAt": "2025-01-01T10:45:00Z"
}
```

---

## 6. Farm Management API

### 6.1 Register Farm

**Endpoint:** `POST /farms`

**Request:**
```json
{
  "name": "Green Valley Farm",
  "owner": {
    "name": "Kim Nongbu",
    "email": "kim@greenfarm.kr",
    "phone": "+82-10-1234-5678"
  },
  "location": {
    "address": "123 Farm Road, Gangwon-do, South Korea",
    "latitude": 37.5665,
    "longitude": 126.9780
  },
  "totalArea": 50.5
}
```

**Response:** `201 Created`
```json
{
  "success": true,
  "data": {
    "farmId": "FARM-KR-001",
    "createdAt": "2025-01-01T09:00:00Z"
  }
}
```

### 6.2 Add Field to Farm

**Endpoint:** `POST /farms/{farmId}/fields`

**Request:**
```json
{
  "name": "North Rice Field",
  "cropType": "RICE",
  "plantingDate": "2025-04-15",
  "boundary": {
    "type": "Polygon",
    "coordinates": [[[126.9780, 37.5665], [126.9790, 37.5675]]]
  },
  "area": 10.5,
  "soilType": "CLAY_LOAM"
}
```

**Response:** `201 Created`
```json
{
  "success": true,
  "data": {
    "fieldId": "FIELD-KR-001",
    "calculatedArea": 10.52,
    "perimeter": 450.2
  }
}
```

---

## 7. Weather Integration API

### 7.1 Get Current Weather

**Endpoint:** `GET /weather/current`

**Query Parameters:**
- `latitude`, `longitude` (required)

**Response:** `200 OK`
```json
{
  "success": true,
  "data": {
    "timestamp": "2025-01-01T10:00:00Z",
    "current": {
      "temperature": 22.5,
      "humidity": 65,
      "windSpeed": 2.3,
      "windDirection": 180
    },
    "sprayConditions": {
      "suitable": true,
      "score": 85,
      "warnings": [],
      "recommendations": ["Optimal spraying conditions"]
    }
  }
}
```

### 7.2 Get Weather Forecast

**Endpoint:** `GET /weather/forecast`

**Query Parameters:**
- `latitude`, `longitude` (required)
- `hours` (default: 24, max: 168)

---

## 8. Regulatory Compliance API

### 8.1 Request Flight Authorization

**Endpoint:** `POST /authorizations`

**Request:**
```json
{
  "missionId": "MISSION-2025-001",
  "operator": {
    "name": "Kim Nongbu",
    "license": "KR-AGRI-2025-001"
  },
  "flightArea": {
    "type": "Polygon",
    "coordinates": [[[126.9780, 37.5665], [126.9790, 37.5675]]]
  },
  "scheduledStart": "2025-01-01T10:00:00Z",
  "scheduledEnd": "2025-01-01T12:00:00Z",
  "maxAltitude": 30
}
```

**Response:** `201 Created`
```json
{
  "success": true,
  "data": {
    "authId": "AUTH-2025-001",
    "status": "APPROVED",
    "validFrom": "2025-01-01T09:00:00Z",
    "validUntil": "2025-01-01T18:00:00Z",
    "restrictions": [
      "No flights within 500m of residential areas",
      "Maintain visual line of sight"
    ]
  }
}
```

### 8.2 Submit Flight Report

**Endpoint:** `POST /flight-reports`

**Request:**
```json
{
  "missionId": "MISSION-2025-001",
  "authId": "AUTH-2025-001",
  "actualStart": "2025-01-01T10:05:00Z",
  "actualEnd": "2025-01-01T10:45:00Z",
  "maxAltitudeReached": 5.5,
  "areaCovered": 10.5,
  "incidents": [],
  "remarks": "Mission completed successfully"
}
```

---

## 9. Drone Fleet Management API

### 9.1 Register Drone

**Endpoint:** `POST /drones`

**Request:**
```json
{
  "model": "WIA-AGRI-X1",
  "serialNumber": "WAX1-2024-0123",
  "registrationNumber": "KR-DRONE-001",
  "capabilities": {
    "maxPayload": 10,
    "tankCapacity": 10,
    "maxFlightTime": 30,
    "maxSpeed": 15,
    "cameras": ["RGB", "MULTISPECTRAL"]
  },
  "owner": "OPERATOR-001"
}
```

**Response:** `201 Created`
```json
{
  "success": true,
  "data": {
    "droneId": "AGRI-DRONE-001",
    "status": "ACTIVE",
    "nextMaintenanceDue": "2025-02-01"
  }
}
```

### 9.2 Log Maintenance

**Endpoint:** `POST /drones/{droneId}/maintenance`

**Request:**
```json
{
  "maintenanceType": "SCHEDULED",
  "date": "2025-01-15",
  "description": "100-hour inspection",
  "technician": "Park Technician",
  "itemsChecked": ["Motors", "Propellers", "Battery", "Spray system"],
  "itemsReplaced": ["Nozzles"],
  "nextMaintenanceDue": "2025-03-15"
}
```

---

## 10. Analytics & Reporting API

### 10.1 Get Farm Analytics

**Endpoint:** `GET /analytics/farms/{farmId}`

**Query Parameters:**
- `fromDate`, `toDate`
- `metrics`: missions, area_covered, yield_estimate

**Response:** `200 OK`
```json
{
  "success": true,
  "data": {
    "farmId": "FARM-KR-001",
    "period": {
      "from": "2025-01-01",
      "to": "2025-01-31"
    },
    "metrics": {
      "totalMissions": 24,
      "totalAreaCovered": 252.0,
      "totalFlightTime": 18.5,
      "averageNDVI": 0.67,
      "healthTrend": "IMPROVING"
    },
    "fieldHealth": [
      {
        "fieldId": "FIELD-KR-001",
        "currentNDVI": 0.68,
        "previousNDVI": 0.62,
        "change": 0.06
      }
    ]
  }
}
```

### 10.2 Generate Report

**Endpoint:** `POST /reports`

**Request:**
```json
{
  "reportType": "SEASONAL_SUMMARY",
  "farmId": "FARM-KR-001",
  "season": "2025-SPRING",
  "includeImages": true,
  "includeNDVI": true,
  "format": "PDF"
}
```

**Response:** `202 Accepted`
```json
{
  "success": true,
  "data": {
    "reportId": "REPORT-2025-001",
    "status": "GENERATING",
    "estimatedCompletion": "2025-01-01T11:00:00Z"
  }
}
```

---

## 11. Webhook Events

### 11.1 Configure Webhook

**Endpoint:** `POST /webhooks`

**Request:**
```json
{
  "url": "https://your-app.com/webhooks/agri-drone",
  "events": [
    "mission.started",
    "mission.completed",
    "ndvi.analysis_completed",
    "alert.low_battery",
    "alert.weather_warning"
  ],
  "secret": "your-webhook-secret"
}
```

### 11.2 Webhook Payload Example

```json
{
  "event": "mission.completed",
  "timestamp": "2025-01-01T10:45:00Z",
  "data": {
    "missionId": "MISSION-2025-001",
    "status": "COMPLETED",
    "areaCovered": 10.5,
    "duration": 45
  },
  "signature": "sha256=..."
}
```

---

## 12. Error Handling

### 12.1 Error Response Format

```json
{
  "success": false,
  "error": {
    "code": "INVALID_MISSION_PARAMS",
    "message": "Field boundary polygon is invalid",
    "details": {
      "field": "fieldBoundary",
      "issue": "Polygon coordinates must form a closed ring"
    }
  },
  "timestamp": "2025-01-01T10:00:00Z"
}
```

### 12.2 HTTP Status Codes

- `200 OK`: Success
- `201 Created`: Resource created
- `202 Accepted`: Request accepted, processing
- `400 Bad Request`: Invalid input
- `401 Unauthorized`: Missing/invalid auth token
- `403 Forbidden`: Insufficient permissions
- `404 Not Found`: Resource not found
- `409 Conflict`: Resource conflict
- `422 Unprocessable Entity`: Validation failed
- `429 Too Many Requests`: Rate limit exceeded
- `500 Internal Server Error`: Server error

---

## 13. Rate Limiting

```
Default: 1000 requests/hour per API key
Telemetry WebSocket: 100 connections per account
Image Upload: 1000 MB/hour
```

**Rate Limit Headers:**
```
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 742
X-RateLimit-Reset: 1609502400
```

---

## 14. SDK Examples

### 14.1 JavaScript/Node.js

```javascript
const WIAAgriDrone = require('@wia/agri-drone-sdk');

const client = new WIAAgriDrone({
  apiKey: 'YOUR_API_KEY',
  environment: 'production'
});

// Create mission
const mission = await client.missions.create({
  farmId: 'FARM-KR-001',
  missionType: 'CROP_SPRAYING',
  fieldBoundary: { /* GeoJSON */ }
});

// Subscribe to telemetry
client.telemetry.subscribe('AGRI-DRONE-001', (data) => {
  console.log('Telemetry:', data);
});

// Request NDVI analysis
const analysis = await client.analysis.ndvi({
  imageId: 'IMG-2025-001-0123'
});
```

### 14.2 Python

```python
from wia_agri_drone import WIAAgriDrone

client = WIAAgriDrone(api_key='YOUR_API_KEY')

# Create mission
mission = client.missions.create(
    farm_id='FARM-KR-001',
    mission_type='CROP_SPRAYING',
    field_boundary={'type': 'Polygon', 'coordinates': [...]}
)

# Get NDVI analysis
analysis = client.analysis.get_ndvi('NDVI-2025-001')
print(f"Mean NDVI: {analysis.statistics.mean}")
```

---

**© 2025 WIA Standards | MIT License**
**弘益人間 · Benefit All Humanity**
