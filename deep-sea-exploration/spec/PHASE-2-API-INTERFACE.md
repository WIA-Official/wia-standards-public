# WIA Deep Sea Exploration - Phase 2: API Interface Specification
**Version:** 1.0
**Status:** Active
**Last Updated:** 2025-01-15
**Philosophy:** 弘익人間 (Benefit All Humanity)

---

## 1. Overview

Phase 2 of the WIA Deep Sea Exploration Standard defines RESTful API interfaces for controlling underwater vehicles (ROVs/AUVs), retrieving mission data, managing sensor configurations, and coordinating multi-vehicle operations. This specification enables seamless integration between surface control systems, underwater vehicles, and shore-based research facilities.

### 1.1 Scope

This specification covers:
- Vehicle control and navigation APIs
- Real-time telemetry streaming
- Mission planning and execution APIs
- Sensor configuration and data retrieval
- Sample management and tracking
- Video/image streaming and storage
- Multi-vehicle coordination
- Emergency response protocols

### 1.2 API Design Principles

1. **RESTful**: Follow REST architectural constraints
2. **Versioned**: API version in URL path for backward compatibility
3. **Authenticated**: OAuth 2.0 and API key authentication
4. **Rate-limited**: Protect against abuse and ensure fair access
5. **Well-documented**: OpenAPI 3.0 specification available
6. **Idempotent**: Safe retry semantics for unreliable networks

---

## 2. Base API Configuration

### 2.1 Base URL Structure

```
https://api.wia-dse.org/v1/{resource}
```

**Example Endpoints:**
```
https://api.wia-dse.org/v1/vehicles
https://api.wia-dse.org/v1/missions
https://api.wia-dse.org/v1/telemetry
```

### 2.2 Authentication

All API requests MUST include authentication via one of these methods:

**Bearer Token (OAuth 2.0):**
```http
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

**API Key:**
```http
X-API-Key: wia_dse_ak_1234567890abcdef
```

### 2.3 Standard Headers

**Request Headers:**
```http
Content-Type: application/json
Accept: application/json
X-WIA-Version: 1.0
X-Request-ID: uuid-v4
```

**Response Headers:**
```http
Content-Type: application/json
X-WIA-Version: 1.0
X-Request-ID: uuid-v4
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1642262400
```

### 2.4 Standard Error Response

```json
{
  "error": {
    "code": "INVALID_DEPTH_PARAMETER",
    "message": "Depth value exceeds vehicle maximum operating depth",
    "details": {
      "requestedDepth": 7000,
      "maxDepth": 6000,
      "unit": "meters"
    },
    "timestamp": "2025-01-15T14:30:00.000Z",
    "requestId": "req-uuid-1234",
    "documentation": "https://docs.wia-dse.org/errors/INVALID_DEPTH_PARAMETER"
  }
}
```

**Standard Error Codes:**
- `AUTHENTICATION_FAILED` (401)
- `AUTHORIZATION_DENIED` (403)
- `RESOURCE_NOT_FOUND` (404)
- `RATE_LIMIT_EXCEEDED` (429)
- `VALIDATION_ERROR` (400)
- `INTERNAL_ERROR` (500)
- `SERVICE_UNAVAILABLE` (503)

---

## 3. Vehicle Control API

### 3.1 List Vehicles

Retrieve all vehicles accessible to the authenticated user.

**Endpoint:** `GET /v1/vehicles`

**Query Parameters:**
| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| status | enum | No | Filter by status (OPERATIONAL, MAINTENANCE, DEPLOYED) |
| type | enum | No | Filter by type (ROV, AUV, SUBMERSIBLE, GLIDER) |
| page | integer | No | Page number (default: 1) |
| limit | integer | No | Results per page (default: 50, max: 100) |

**Example Request:**
```http
GET /v1/vehicles?status=OPERATIONAL&type=ROV
Authorization: Bearer <token>
```

**Example Response:**
```json
{
  "data": [
    {
      "vehicleId": "ROV-ATLANTIS-001",
      "name": "Atlantis ROV Alpha",
      "type": "ROV",
      "manufacturer": "Oceaneering",
      "model": "Magnum Plus",
      "status": "OPERATIONAL",
      "location": {
        "latitude": 36.7977,
        "longitude": -121.8472,
        "depth": 3547.2,
        "lastUpdate": "2025-01-15T14:30:00.000Z"
      },
      "capabilities": {
        "maxDepth": 6000,
        "maxDepthUnit": "meters",
        "manipulators": 2,
        "cameras": 3,
        "sensors": ["CTD", "SONAR", "LIDAR", "MAGNETOMETER"],
        "payloadCapacity": 150,
        "payloadCapacityUnit": "kg"
      },
      "currentMission": {
        "missionId": "MISSION-2025-001",
        "missionName": "Hydrothermal Vent Survey",
        "startTime": "2025-01-15T08:00:00.000Z"
      },
      "health": {
        "batteryLevel": 68,
        "systemStatus": "NOMINAL",
        "lastMaintenance": "2025-01-01T00:00:00.000Z"
      }
    }
  ],
  "pagination": {
    "page": 1,
    "limit": 50,
    "total": 1,
    "totalPages": 1
  }
}
```

### 3.2 Get Vehicle Status

Retrieve detailed status for a specific vehicle.

**Endpoint:** `GET /v1/vehicles/{vehicleId}`

**Example Response:**
```json
{
  "vehicleId": "ROV-ATLANTIS-001",
  "name": "Atlantis ROV Alpha",
  "type": "ROV",
  "status": "DEPLOYED",
  "telemetry": {
    "timestamp": "2025-01-15T14:30:00.000Z",
    "position": {
      "latitude": 36.7977,
      "longitude": -121.8472,
      "depth": 3547.2,
      "altitude": 12.5,
      "heading": 187.5,
      "pitch": -2.3,
      "roll": 1.7
    },
    "environment": {
      "temperature": 2.47,
      "pressure": 354.72,
      "salinity": 34.89
    },
    "power": {
      "batteryVoltage": 48.2,
      "batteryCurrent": 23.5,
      "batteryCapacityRemaining": 68,
      "estimatedTimeRemaining": 242
    },
    "systems": {
      "propulsion": "NOMINAL",
      "hydraulics": "NOMINAL",
      "communications": "NOMINAL",
      "sensors": "NOMINAL"
    }
  },
  "alerts": []
}
```

### 3.3 Send Vehicle Command

Control vehicle movement and operations.

**Endpoint:** `POST /v1/vehicles/{vehicleId}/commands`

**Request Body:**
```json
{
  "command": "NAVIGATE_TO_WAYPOINT",
  "parameters": {
    "waypoint": {
      "latitude": 36.7980,
      "longitude": -121.8475,
      "depth": 3550.0,
      "altitude": 10.0
    },
    "speed": 0.5,
    "speedUnit": "m/s",
    "obstacleAvoidance": true,
    "timeout": 300,
    "timeoutUnit": "seconds"
  },
  "priority": "NORMAL",
  "acknowledgementRequired": true
}
```

**Supported Commands:**
- `NAVIGATE_TO_WAYPOINT`
- `HOVER_AT_POSITION`
- `SURFACE`
- `EMERGENCY_ASCENT`
- `ACTIVATE_SENSOR`
- `DEACTIVATE_SENSOR`
- `CAPTURE_IMAGE`
- `RECORD_VIDEO`
- `COLLECT_SAMPLE`
- `DEPLOY_INSTRUMENT`
- `RETRIEVE_INSTRUMENT`

**Example Response:**
```json
{
  "commandId": "CMD-2025-01-15-001",
  "status": "ACKNOWLEDGED",
  "timestamp": "2025-01-15T14:30:01.000Z",
  "estimatedCompletionTime": "2025-01-15T14:35:01.000Z",
  "acknowledgement": {
    "vehicleId": "ROV-ATLANTIS-001",
    "receivedAt": "2025-01-15T14:30:01.123Z",
    "validationStatus": "VALIDATED",
    "executionStarted": "2025-01-15T14:30:01.456Z"
  }
}
```

### 3.4 Get Command Status

Check the execution status of a previously issued command.

**Endpoint:** `GET /v1/vehicles/{vehicleId}/commands/{commandId}`

**Example Response:**
```json
{
  "commandId": "CMD-2025-01-15-001",
  "status": "IN_PROGRESS",
  "progress": 45,
  "progressUnit": "percent",
  "currentStep": "Navigating to waypoint",
  "startTime": "2025-01-15T14:30:01.456Z",
  "estimatedCompletionTime": "2025-01-15T14:35:01.000Z",
  "errors": []
}
```

---

## 4. Telemetry API

### 4.1 Stream Real-time Telemetry (WebSocket)

**Endpoint:** `wss://api.wia-dse.org/v1/telemetry/stream`

**Connection Parameters:**
```javascript
const ws = new WebSocket('wss://api.wia-dse.org/v1/telemetry/stream?vehicleId=ROV-ATLANTIS-001&dataRate=10Hz');

ws.onmessage = (event) => {
  const telemetry = JSON.parse(event.data);
  console.log('Telemetry update:', telemetry);
};
```

**Stream Data Format:**
```json
{
  "streamId": "STREAM-2025-001",
  "vehicleId": "ROV-ATLANTIS-001",
  "timestamp": "2025-01-15T14:30:00.000Z",
  "sequenceNumber": 12345,
  "data": {
    "position": {
      "latitude": 36.7977,
      "longitude": -121.8472,
      "depth": 3547.2,
      "heading": 187.5
    },
    "sensors": {
      "temperature": 2.47,
      "pressure": 354.72,
      "salinity": 34.89
    },
    "power": {
      "batteryLevel": 68
    }
  }
}
```

### 4.2 Retrieve Historical Telemetry

**Endpoint:** `GET /v1/telemetry/history`

**Query Parameters:**
| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| vehicleId | string | Yes | Vehicle identifier |
| startTime | ISO8601 | Yes | Start of time range |
| endTime | ISO8601 | Yes | End of time range |
| parameters | array | No | Specific parameters to retrieve |
| interval | integer | No | Sampling interval in seconds |

**Example Request:**
```http
GET /v1/telemetry/history?vehicleId=ROV-ATLANTIS-001&startTime=2025-01-15T08:00:00Z&endTime=2025-01-15T16:00:00Z&parameters=depth,temperature&interval=60
```

**Example Response:**
```json
{
  "vehicleId": "ROV-ATLANTIS-001",
  "timeRange": {
    "start": "2025-01-15T08:00:00Z",
    "end": "2025-01-15T16:00:00Z"
  },
  "interval": 60,
  "data": [
    {
      "timestamp": "2025-01-15T08:00:00Z",
      "depth": 0,
      "temperature": 15.2
    },
    {
      "timestamp": "2025-01-15T08:01:00Z",
      "depth": 50.3,
      "temperature": 14.8
    }
  ],
  "downloadUrl": "https://api.wia-dse.org/v1/downloads/telemetry-export-abc123.csv"
}
```

---

## 5. Mission API

### 5.1 Create Mission

**Endpoint:** `POST /v1/missions`

**Request Body:**
```json
{
  "missionName": "Hydrothermal Vent Survey 2025",
  "description": "Survey and sample collection at Juan de Fuca Ridge",
  "vehicleId": "ROV-ATLANTIS-001",
  "plannedStartTime": "2025-01-20T08:00:00Z",
  "plannedEndTime": "2025-01-20T18:00:00Z",
  "missionType": "SCIENTIFIC_RESEARCH",
  "objectives": [
    "Map hydrothermal vent locations",
    "Collect biological samples",
    "Measure water chemistry"
  ],
  "waypoints": [
    {
      "name": "Launch Point",
      "latitude": 46.0,
      "longitude": -130.0,
      "depth": 0,
      "tasks": ["SYSTEM_CHECK"]
    },
    {
      "name": "Vent Field Alpha",
      "latitude": 46.05,
      "longitude": -130.05,
      "depth": 2200,
      "tasks": ["BATHYMETRIC_SURVEY", "SAMPLE_COLLECTION"]
    }
  ],
  "permissions": {
    "chiefScientist": "dr.jane.smith@institution.edu",
    "permits": ["NOAA-PERMIT-2025-001"]
  }
}
```

**Example Response:**
```json
{
  "missionId": "MISSION-2025-001",
  "status": "PLANNED",
  "createdAt": "2025-01-15T14:30:00Z",
  "missionUrl": "https://api.wia-dse.org/v1/missions/MISSION-2025-001"
}
```

### 5.2 Get Mission Data

**Endpoint:** `GET /v1/missions/{missionId}`

**Example Response:**
```json
{
  "missionId": "MISSION-2025-001",
  "missionName": "Hydrothermal Vent Survey 2025",
  "status": "IN_PROGRESS",
  "vehicleId": "ROV-ATLANTIS-001",
  "timeline": {
    "planned": {
      "start": "2025-01-20T08:00:00Z",
      "end": "2025-01-20T18:00:00Z"
    },
    "actual": {
      "start": "2025-01-20T08:15:00Z",
      "end": null
    }
  },
  "progress": {
    "waypointsCompleted": 3,
    "waypointsTotal": 8,
    "percentComplete": 37.5
  },
  "data": {
    "samplesCollected": 12,
    "imagesCaptures": 347,
    "videoHours": 4.2,
    "dataSizeMB": 2847
  }
}
```

---

## 6. Sensor API

### 6.1 Configure Sensor

**Endpoint:** `POST /v1/vehicles/{vehicleId}/sensors/{sensorId}/configure`

**Request Body:**
```json
{
  "sensorId": "CTD-01",
  "configuration": {
    "samplingRate": 10,
    "samplingRateUnit": "Hz",
    "averagingWindow": 5,
    "calibration": {
      "temperatureOffset": 0.0,
      "pressureOffset": 0.0,
      "salinityCoefficient": 1.0
    },
    "alarms": {
      "temperatureMax": 50,
      "pressureMax": 650,
      "enabled": true
    }
  }
}
```

### 6.2 Get Sensor Data

**Endpoint:** `GET /v1/vehicles/{vehicleId}/sensors/{sensorId}/data`

**Query Parameters:**
- `startTime` (ISO8601)
- `endTime` (ISO8601)
- `limit` (integer, max 10000)

---

## 7. Sample Management API

### 7.1 Register Sample

**Endpoint:** `POST /v1/samples`

**Request Body:**
```json
{
  "sampleType": "BIOLOGICAL",
  "missionId": "MISSION-2025-001",
  "vehicleId": "ROV-ATLANTIS-001",
  "collectionTimestamp": "2025-01-20T10:30:00Z",
  "location": {
    "latitude": 46.05,
    "longitude": -130.05,
    "depth": 2200
  },
  "description": "Riftia pachyptila specimen",
  "containerId": "BIO-BOX-07",
  "collector": {
    "name": "Dr. Robert Chen",
    "orcid": "0000-0002-1234-5678"
  },
  "chainOfCustody": [
    {
      "timestamp": "2025-01-20T10:30:00Z",
      "custodian": "ROV Pilot",
      "action": "COLLECTED"
    }
  ]
}
```

**Example Response:**
```json
{
  "sampleId": "SAMPLE-2025-001-012",
  "registrationTimestamp": "2025-01-20T10:30:15Z",
  "qrCode": "https://api.wia-dse.org/v1/samples/SAMPLE-2025-001-012/qrcode.png",
  "trackingUrl": "https://portal.wia-dse.org/samples/SAMPLE-2025-001-012"
}
```

---

## 8. Rate Limiting

API requests are rate-limited per authentication credential:

| Tier | Requests per Hour | Burst Limit |
|------|-------------------|-------------|
| Free | 1,000 | 100/min |
| Research | 10,000 | 500/min |
| Institution | 100,000 | 2000/min |
| Enterprise | Unlimited | Custom |

Rate limit headers included in every response:
```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1642262400
```

---

## 9. Webhooks

Subscribe to real-time events:

**Webhook Events:**
- `vehicle.status.changed`
- `mission.started`
- `mission.completed`
- `sample.collected`
- `alert.critical`
- `command.completed`

**Webhook Payload Example:**
```json
{
  "eventId": "evt_2025_001",
  "eventType": "sample.collected",
  "timestamp": "2025-01-20T10:30:00Z",
  "data": {
    "sampleId": "SAMPLE-2025-001-012",
    "missionId": "MISSION-2025-001",
    "vehicleId": "ROV-ATLANTIS-001"
  }
}
```

---

## 10. OpenAPI Specification

Full OpenAPI 3.0 specification available at:
```
https://api.wia-dse.org/v1/openapi.json
https://api.wia-dse.org/v1/openapi.yaml
```

---

**Document Control:**
- Author: WIA Standards Committee
- License: CC BY 4.0
- Repository: https://github.com/WIA-Official/wia-standards

弘益人間 · Benefit All Humanity
