# Chapter 5: API Interface Specification (Phase 2)

## RESTful and Real-Time APIs for Underwater Vehicle Control and Data Access

---

## 5.1 API Architecture Overview

### Design Philosophy

The WIA Deep Sea Exploration API follows REST principles for resource-based operations and WebSocket for real-time streaming. The architecture prioritizes:

- **Statelessness**: Each request contains all necessary information
- **Resource Orientation**: Clear URL patterns for data access
- **Consistency**: Uniform response formats across endpoints
- **Real-Time Capability**: WebSocket streams for telemetry and control
- **Security**: Authentication and authorization at every layer

### API Base URL Structure

```
https://{host}/api/v{version}/{resource}

Examples:
https://vehicle.mbari.org/api/v1/telemetry
https://data.noaa.gov/api/v1/bathymetry
https://rov-control.ship.local/api/v1/commands
```

### API Versioning

| Version | Status | Support Until |
|---------|--------|---------------|
| v1 | Current | 2028-01 |
| v2 | Planned | - |

### Response Format

All responses use JSON with consistent structure:

**Success Response**:
```json
{
  "success": true,
  "data": { ... },
  "meta": {
    "timestamp": "2025-01-15T14:30:00.000Z",
    "requestId": "req-abc123",
    "processingTime": 45
  }
}
```

**Error Response**:
```json
{
  "success": false,
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Invalid depth value: must be between 0 and 11000",
    "field": "depth",
    "details": { ... }
  },
  "meta": {
    "timestamp": "2025-01-15T14:30:00.000Z",
    "requestId": "req-abc123"
  }
}
```

### HTTP Status Codes

| Code | Meaning | Usage |
|------|---------|-------|
| 200 | OK | Successful GET, PUT |
| 201 | Created | Successful POST |
| 204 | No Content | Successful DELETE |
| 400 | Bad Request | Validation error |
| 401 | Unauthorized | Authentication required |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource doesn't exist |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Error | Server error |
| 503 | Service Unavailable | System overloaded |

---

## 5.2 Vehicle Control Endpoints

### Command Execution

**POST /api/v1/commands**

Execute a command on the vehicle:

```json
// Request
{
  "commandType": "THRUSTER_CONTROL",
  "target": "ALL",
  "parameters": {
    "mode": "STATION_KEEP",
    "heading": 180.0,
    "altitude": 5.0
  },
  "priority": "NORMAL",
  "timeout": 30000
}

// Response
{
  "success": true,
  "data": {
    "commandId": "cmd-2025-01-15-001",
    "status": "ACCEPTED",
    "estimatedExecutionTime": 2500,
    "queuePosition": 0
  }
}
```

### Command Types

| Command Type | Description | Parameters |
|--------------|-------------|------------|
| THRUSTER_CONTROL | Propulsion control | mode, heading, speed, depth |
| LIGHTS_CONTROL | Lighting control | intensity, target |
| CAMERA_CONTROL | Camera operations | action, zoom, pan, tilt |
| MANIPULATOR_CONTROL | Arm operations | arm, action, position |
| SAMPLING | Sample collection | sampler, action |
| NAVIGATION | Waypoint navigation | waypoint, speed |
| EMERGENCY | Emergency procedures | action |

### Command Status

**GET /api/v1/commands/{commandId}**

```json
{
  "success": true,
  "data": {
    "commandId": "cmd-2025-01-15-001",
    "commandType": "THRUSTER_CONTROL",
    "status": "COMPLETED",
    "submittedAt": "2025-01-15T14:30:00.000Z",
    "startedAt": "2025-01-15T14:30:00.500Z",
    "completedAt": "2025-01-15T14:30:02.100Z",
    "result": {
      "success": true,
      "message": "Station keeping established"
    }
  }
}
```

### Configuration Management

**GET /api/v1/configuration**
```json
{
  "data": {
    "vehicle": {
      "id": "ROV-ATLANTIS-001",
      "type": "ROV",
      "maxDepth": 6000
    },
    "thrusters": {
      "count": 8,
      "maxPower": 100
    },
    "cameras": [
      {"id": "HD_MAIN", "resolution": "4K"},
      {"id": "SD_PILOT", "resolution": "1080p"}
    ],
    "sensors": [...]
  }
}
```

**PUT /api/v1/configuration**
```json
// Request
{
  "lights": {
    "defaultIntensity": 50
  },
  "cameras": {
    "HD_MAIN": {
      "whiteBalance": "AUTO"
    }
  }
}
```

---

## 5.3 Telemetry Streaming (WebSocket)

### Connection Establishment

```javascript
const ws = new WebSocket('wss://vehicle.example.com/api/v1/telemetry/stream');

ws.onopen = () => {
  // Subscribe to channels
  ws.send(JSON.stringify({
    action: 'subscribe',
    channels: ['navigation', 'environment', 'systems'],
    rate: 1,  // Hz
    format: 'COMPACT'
  }));
};

ws.onmessage = (event) => {
  const message = JSON.parse(event.data);
  handleTelemetry(message);
};
```

### Subscription Message

```json
{
  "action": "subscribe",
  "channels": ["navigation", "environment", "systems", "alerts"],
  "rate": 1,
  "format": "FULL",
  "filter": {
    "systems": ["thrusters", "cameras"]
  }
}
```

### Telemetry Channels

| Channel | Data Content | Typical Rate |
|---------|--------------|--------------|
| navigation | Position, heading, velocity | 1-10 Hz |
| environment | CTD, water current | 0.1-1 Hz |
| systems | Thruster, camera, power status | 1 Hz |
| alerts | Warning and error messages | Event-driven |
| video | Video stream metadata | Frame rate |
| commands | Command status updates | Event-driven |

### Telemetry Message Format

```json
{
  "channel": "navigation",
  "timestamp": "2025-01-15T14:30:00.123Z",
  "sequenceNumber": 12345,
  "data": {
    "position": {
      "latitude": 36.7977,
      "longitude": -121.8472,
      "depth": 3547.2,
      "altitude": 12.5
    },
    "orientation": {
      "heading": 187.5,
      "pitch": -2.3,
      "roll": 1.7
    },
    "velocity": {
      "forward": 0.5,
      "lateral": 0.1,
      "vertical": -0.02
    }
  }
}
```

### Heartbeat and Connection Management

```json
// Client heartbeat (every 30 seconds)
{"action": "heartbeat", "timestamp": "2025-01-15T14:30:00.000Z"}

// Server acknowledgment
{"type": "heartbeat_ack", "serverTime": "2025-01-15T14:30:00.002Z"}
```

---

## 5.4 Data Query and Export

### Telemetry History

**GET /api/v1/telemetry**

Query parameters:
- `start`: Start timestamp (ISO8601)
- `end`: End timestamp (ISO8601)
- `channels`: Comma-separated channel list
- `resolution`: Decimation factor (1, 10, 60, 300)
- `format`: json, csv, netcdf
- `limit`: Maximum records (default 1000)
- `offset`: Pagination offset

```
GET /api/v1/telemetry?start=2025-01-15T10:00:00Z&end=2025-01-15T14:00:00Z&channels=navigation,environment&resolution=60&format=json
```

### Bathymetric Data

**GET /api/v1/bathymetry**

```
GET /api/v1/bathymetry?bbox=-121.85,-121.84,36.79,36.80&resolution=1&format=geotiff
```

Response:
```json
{
  "data": {
    "type": "grid",
    "boundingBox": {...},
    "resolution": 1.0,
    "crs": "EPSG:4326",
    "downloadUrl": "/api/v1/downloads/bathy-abc123.tiff",
    "expiresAt": "2025-01-15T15:30:00.000Z",
    "size": 52428800
  }
}
```

### Sample Records

**GET /api/v1/samples**

```json
{
  "data": {
    "total": 145,
    "page": 1,
    "pageSize": 20,
    "samples": [
      {
        "sampleId": "SAMPLE-2025-01-15-001",
        "sampleType": "BIOLOGICAL",
        "taxon": "Riftia pachyptila",
        "location": {...},
        "collectedAt": "2025-01-15T14:30:00.000Z"
      }
    ]
  }
}
```

**POST /api/v1/samples**

Create new sample record:
```json
{
  "sampleType": "BIOLOGICAL",
  "location": {
    "latitude": 36.7977,
    "longitude": -121.8472,
    "depth": 3547.2
  },
  "specimen": {
    "taxonCandidate": "Unknown tubeworm",
    "description": "Novel species, red coloration"
  }
}
```

---

## 5.5 Mission Management APIs

### Mission CRUD Operations

**GET /api/v1/missions**
```json
{
  "data": {
    "missions": [
      {
        "missionId": "MISSION-2025-01-15-001",
        "name": "Hydrothermal Survey Alpha",
        "status": "IN_PROGRESS",
        "startedAt": "2025-01-15T10:00:00.000Z",
        "vehicle": "ROV-ATLANTIS-001",
        "progress": 65
      }
    ]
  }
}
```

**POST /api/v1/missions**
```json
{
  "name": "Hydrothermal Survey Beta",
  "description": "Follow-up survey of vent field",
  "vehicle": "ROV-ATLANTIS-001",
  "waypoints": [
    {"latitude": 36.7977, "longitude": -121.8472, "depth": 3500, "action": "SURVEY"},
    {"latitude": 36.7980, "longitude": -121.8470, "depth": 3520, "action": "SAMPLE"}
  ],
  "parameters": {
    "maxDepth": 3600,
    "surveySpeed": 0.5
  }
}
```

### Waypoint Management

**GET /api/v1/missions/{missionId}/waypoints**

**PUT /api/v1/missions/{missionId}/waypoints/{waypointId}**
```json
{
  "latitude": 36.7982,
  "longitude": -121.8468,
  "depth": 3530,
  "action": "SAMPLE",
  "holdTime": 300
}
```

### Mission Control

**POST /api/v1/missions/{missionId}/control**
```json
{
  "action": "PAUSE"  // START, PAUSE, RESUME, ABORT
}
```

---

## 5.6 Authentication and Authorization

### Authentication Methods

**API Key Authentication**:
```
Authorization: ApiKey sk_live_abc123xyz
```

**JWT Bearer Token**:
```
Authorization: Bearer eyJhbGciOiJIUzI1NiIs...
```

**OAuth 2.0**:
```
Authorization: Bearer {access_token}
```

### Permission Levels

| Role | Permissions |
|------|-------------|
| OBSERVER | Read telemetry, query data |
| SCIENTIST | Observer + create samples/annotations |
| PILOT | Scientist + vehicle control |
| SUPERVISOR | Pilot + mission management |
| ADMIN | Full access + user management |

### Role-Based Access Control

```json
{
  "user": {
    "id": "user-12345",
    "email": "scientist@institution.edu",
    "roles": ["SCIENTIST"],
    "permissions": [
      "telemetry:read",
      "data:read",
      "samples:create",
      "annotations:create"
    ]
  }
}
```

---

## 5.7 Rate Limiting and Quotas

### Rate Limits

| Endpoint Category | Limit | Window |
|-------------------|-------|--------|
| Telemetry read | 100 req/sec | Per IP |
| Data query | 60 req/min | Per API key |
| Commands | 10 req/sec | Per vehicle |
| Bulk export | 10 req/hour | Per API key |

### Rate Limit Headers

```http
X-RateLimit-Limit: 100
X-RateLimit-Remaining: 87
X-RateLimit-Reset: 1705329600
```

### Quota Management

```json
{
  "data": {
    "quotas": {
      "apiCalls": {"used": 45000, "limit": 100000, "resetAt": "2025-02-01"},
      "dataExport": {"used": 52428800000, "limit": 107374182400, "unit": "bytes"},
      "storageUpload": {"used": 1073741824, "limit": 10737418240, "unit": "bytes"}
    }
  }
}
```

---

## 5.8 SDK Reference

### TypeScript SDK

```typescript
import { WIADeepSeaClient } from '@wia/deep-sea-sdk';

const client = new WIADeepSeaClient({
  baseUrl: 'https://vehicle.example.com/api/v1',
  apiKey: 'sk_live_abc123'
});

// Get current telemetry
const telemetry = await client.telemetry.getCurrent();
console.log(`Depth: ${telemetry.position.depth}m`);

// Subscribe to real-time updates
client.telemetry.subscribe({
  channels: ['navigation', 'environment'],
  rate: 1,
  onMessage: (data) => {
    console.log('Telemetry update:', data);
  }
});

// Send command
const result = await client.commands.execute({
  commandType: 'LIGHTS_CONTROL',
  parameters: { intensity: 75 }
});

// Query historical data
const history = await client.data.queryTelemetry({
  start: new Date('2025-01-15T10:00:00Z'),
  end: new Date('2025-01-15T14:00:00Z'),
  channels: ['navigation'],
  resolution: 60
});
```

### Python SDK

```python
from wia_deepsea import DeepSeaClient

client = DeepSeaClient(
    base_url='https://vehicle.example.com/api/v1',
    api_key='sk_live_abc123'
)

# Get current telemetry
telemetry = client.telemetry.get_current()
print(f"Depth: {telemetry.position.depth}m")

# Query samples
samples = client.samples.list(
    sample_type='BIOLOGICAL',
    start_date='2025-01-15',
    limit=50
)

# Create sample record
new_sample = client.samples.create(
    sample_type='GEOLOGICAL',
    location={'latitude': 36.7977, 'longitude': -121.8472, 'depth': 3547},
    specimen={'rock_type': 'BASALT'}
)
```

---

## Chapter Summary

Phase 2 of the WIA Deep Sea Exploration Standard provides comprehensive API specifications for interacting with underwater vehicles and oceanographic data systems. The RESTful design ensures broad compatibility, while WebSocket streaming enables real-time telemetry and control.

Authentication, rate limiting, and role-based access control ensure secure operations, while the SDK implementations in TypeScript and Python simplify integration for developers.

---

## Key Takeaways

1. **REST for resources, WebSocket for real-time** streaming telemetry
2. **Consistent response format** with success/error structure
3. **Role-based access control** protects vehicle operations
4. **Rate limiting prevents abuse** while allowing legitimate usage
5. **SDKs in TypeScript and Python** accelerate development

---

## Review Questions

1. What HTTP method should be used to create a new sample record?
2. How does WebSocket telemetry subscription work?
3. What role is required to execute vehicle commands?
4. Design an API request to query bathymetric data for a specific area.
5. How are rate limits communicated to API clients?

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity
