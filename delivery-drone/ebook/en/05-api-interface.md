# Chapter 5: API Interface and Integration

## Phase 2: RESTful APIs, WebSocket Streaming, and SDK Implementation

---

## 5.1 API Architecture Overview

### Design Philosophy

The WIA Delivery Drone API follows REST principles for resource-based operations and WebSocket for real-time streaming. The architecture prioritizes:

- **Simplicity**: Intuitive resource naming and HTTP semantics
- **Consistency**: Uniform response formats and error handling
- **Security**: Authentication, authorization, and encryption
- **Performance**: Efficient pagination, caching, and streaming
- **Extensibility**: Versioning and backward compatibility

### Base URL Structure

```
https://{environment}.api.wia.com/v{version}/{resource}

Environments:
  - production: api.wia.com
  - staging: staging.api.wia.com
  - sandbox: sandbox.api.wia.com

Examples:
  https://api.wia.com/v1/missions
  https://api.wia.com/v1/drones/WIA-DRN-X1-0042/telemetry
  https://staging.api.wia.com/v1/flight-plans
```

### Response Format

**Success Response**:
```json
{
  "success": true,
  "data": {
    "missionId": "MSN-20250101-1234",
    "status": "SCHEDULED",
    ...
  },
  "meta": {
    "requestId": "req-abc123",
    "timestamp": "2025-01-01T10:00:00.000Z",
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
    "message": "Invalid pickup location coordinates",
    "field": "pickup.location.latitude",
    "details": {
      "received": 200.5,
      "constraint": "Latitude must be between -90 and 90"
    }
  },
  "meta": {
    "requestId": "req-abc123",
    "timestamp": "2025-01-01T10:00:00.000Z"
  }
}
```

### HTTP Status Codes

| Code | Meaning | Use Case |
|------|---------|----------|
| 200 | OK | Successful GET, PUT |
| 201 | Created | Successful POST |
| 202 | Accepted | Async operation started |
| 204 | No Content | Successful DELETE |
| 400 | Bad Request | Validation error |
| 401 | Unauthorized | Missing/invalid auth |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource doesn't exist |
| 409 | Conflict | Resource state conflict |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Error | Server error |

---

## 5.2 Mission Management API

### Create Mission

**POST /api/v1/missions**

Create a new delivery mission:

```json
// Request
{
  "pickup": {
    "location": {
      "latitude": 37.7749,
      "longitude": -122.4194,
      "altitude": 0
    },
    "address": "123 Market St, San Francisco, CA 94102",
    "contact": {
      "name": "John Doe",
      "phone": "+1-555-0100",
      "email": "john@example.com"
    },
    "instructions": "Ring doorbell, package is at reception"
  },
  "dropoff": {
    "location": {
      "latitude": 37.7849,
      "longitude": -122.4094,
      "altitude": 0
    },
    "address": "456 Mission St, San Francisco, CA 94105",
    "contact": {
      "name": "Jane Smith",
      "phone": "+1-555-0200",
      "email": "jane@example.com"
    },
    "instructions": "Leave at front door",
    "deliveryWindow": {
      "start": "2025-01-01T10:00:00Z",
      "end": "2025-01-01T12:00:00Z"
    }
  },
  "package": {
    "weight": 2.5,
    "dimensions": {
      "length": 30,
      "width": 20,
      "height": 15
    },
    "fragile": false,
    "value": 150.00
  },
  "priority": "STANDARD",
  "scheduledTime": "2025-01-01T10:00:00Z"
}

// Response (201 Created)
{
  "success": true,
  "data": {
    "missionId": "MSN-20250101-1234",
    "status": "SCHEDULED",
    "pickup": { ... },
    "dropoff": { ... },
    "package": { ... },
    "scheduling": {
      "scheduledPickup": "2025-01-01T10:00:00Z",
      "estimatedDelivery": "2025-01-01T10:18:00Z",
      "estimatedFlightTime": 12
    },
    "drone": {
      "assigned": "WIA-DRN-X1-0042",
      "name": "Falcon-42",
      "class": "LIGHT"
    },
    "tracking": {
      "url": "https://track.wia.com/MSN-20250101-1234",
      "code": "ABCD1234"
    },
    "createdAt": "2025-01-01T09:30:00.000Z"
  }
}
```

### Get Mission Status

**GET /api/v1/missions/{missionId}**

```json
// Response
{
  "success": true,
  "data": {
    "missionId": "MSN-20250101-1234",
    "status": "IN_FLIGHT",
    "phase": "EN_ROUTE_DROPOFF",
    "progress": 65,
    "currentPosition": {
      "latitude": 37.7799,
      "longitude": -122.4144,
      "altitude": 100,
      "heading": 45,
      "speed": 18.5
    },
    "timing": {
      "scheduledPickup": "2025-01-01T10:00:00Z",
      "actualPickup": "2025-01-01T10:02:00Z",
      "estimatedDelivery": "2025-01-01T10:14:00Z",
      "eta": 180
    },
    "battery": {
      "current": 72,
      "estimatedAtDelivery": 58
    },
    "weather": {
      "temperature": 18,
      "windSpeed": 5.2,
      "conditions": "CLEAR"
    }
  }
}
```

### List Missions

**GET /api/v1/missions**

Query parameters:
- `status`: Filter by status (PENDING, SCHEDULED, IN_FLIGHT, COMPLETED, CANCELLED)
- `droneId`: Filter by assigned drone
- `startDate`: Missions after date
- `endDate`: Missions before date
- `limit`: Results per page (default 20, max 100)
- `offset`: Pagination offset

```
GET /api/v1/missions?status=COMPLETED&startDate=2025-01-01&limit=50
```

### Cancel Mission

**POST /api/v1/missions/{missionId}/cancel**

```json
// Request
{
  "reason": "Customer requested cancellation",
  "code": "CUSTOMER_REQUEST"
}

// Response
{
  "success": true,
  "data": {
    "missionId": "MSN-20250101-1234",
    "status": "CANCELLED",
    "cancellation": {
      "reason": "Customer requested cancellation",
      "code": "CUSTOMER_REQUEST",
      "cancelledAt": "2025-01-01T09:45:00.000Z",
      "refundEligible": true
    }
  }
}
```

---

## 5.3 Drone Management API

### Register Drone

**POST /api/v1/drones**

```json
// Request
{
  "serialNumber": "SN-2025-00042",
  "manufacturer": "WIA Aerospace",
  "model": "Falcon X1",
  "class": "LIGHT",
  "specifications": {
    "mtow": 8.5,
    "maxPayload": 3.0,
    "maxRange": 15,
    "maxSpeed": 22,
    "maxFlightTime": 35,
    "batteryCapacity": 10000
  },
  "certifications": {
    "remoteId": "RID-2025-00042",
    "typeApproval": "FAA-TC-2024-0042",
    "insurancePolicy": "POL-2025-001234"
  },
  "homeBase": {
    "latitude": 37.7749,
    "longitude": -122.4194,
    "name": "SF Distribution Center"
  }
}

// Response (201 Created)
{
  "success": true,
  "data": {
    "droneId": "WIA-DRN-X1-0042",
    "status": "REGISTERED",
    "serialNumber": "SN-2025-00042",
    "name": "Falcon-42",
    ...
  }
}
```

### Get Drone Status

**GET /api/v1/drones/{droneId}**

```json
// Response
{
  "success": true,
  "data": {
    "droneId": "WIA-DRN-X1-0042",
    "name": "Falcon-42",
    "status": "AVAILABLE",
    "operational": true,
    "location": {
      "latitude": 37.7749,
      "longitude": -122.4194,
      "altitude": 0,
      "updatedAt": "2025-01-01T09:55:00.000Z"
    },
    "battery": {
      "level": 100,
      "health": 95,
      "cycleCount": 142
    },
    "maintenance": {
      "lastInspection": "2025-01-01",
      "nextInspection": "2025-01-15",
      "flightHours": 245.5,
      "totalFlights": 892
    },
    "currentMission": null,
    "todayStats": {
      "flights": 5,
      "distance": 42.5,
      "deliveries": 5,
      "flightTime": 1.2
    }
  }
}
```

### Drone Commands

**POST /api/v1/drones/{droneId}/commands**

```json
// Request: Return to Home
{
  "command": "RETURN_TO_HOME",
  "priority": "HIGH",
  "parameters": {}
}

// Request: Go to Position
{
  "command": "GOTO_POSITION",
  "priority": "NORMAL",
  "parameters": {
    "latitude": 37.7800,
    "longitude": -122.4150,
    "altitude": 50,
    "speed": 10
  }
}

// Response
{
  "success": true,
  "data": {
    "commandId": "CMD-20250101-5678",
    "status": "ACCEPTED",
    "droneId": "WIA-DRN-X1-0042",
    "command": "RETURN_TO_HOME",
    "acceptedAt": "2025-01-01T10:05:00.000Z",
    "estimatedCompletion": "2025-01-01T10:12:00.000Z"
  }
}
```

---

## 5.4 Real-Time Streaming (WebSocket)

### Connection Establishment

```javascript
const ws = new WebSocket('wss://api.wia.com/v1/stream');

ws.onopen = () => {
  // Authenticate
  ws.send(JSON.stringify({
    type: 'AUTH',
    token: 'Bearer eyJhbGciOiJIUzI1NiIs...'
  }));

  // Subscribe to channels
  ws.send(JSON.stringify({
    type: 'SUBSCRIBE',
    channels: [
      { type: 'mission', id: 'MSN-20250101-1234' },
      { type: 'drone', id: 'WIA-DRN-X1-0042' },
      { type: 'fleet', id: 'FLEET-001' }
    ],
    options: {
      telemetryRate: 1,  // Hz
      includeVideo: false
    }
  }));
};

ws.onmessage = (event) => {
  const message = JSON.parse(event.data);
  handleStreamMessage(message);
};

function handleStreamMessage(message) {
  switch (message.type) {
    case 'TELEMETRY':
      updateDronePosition(message.data);
      break;
    case 'STATUS':
      updateDroneStatus(message.data);
      break;
    case 'EVENT':
      handleEvent(message.data);
      break;
    case 'MISSION_UPDATE':
      updateMissionStatus(message.data);
      break;
  }
}
```

### Stream Message Types

**Telemetry Stream**:
```json
{
  "type": "TELEMETRY",
  "channel": "drone",
  "id": "WIA-DRN-X1-0042",
  "timestamp": "2025-01-01T10:05:30.123Z",
  "data": {
    "position": {
      "latitude": 37.7799,
      "longitude": -122.4144,
      "altitude": 100
    },
    "velocity": {
      "groundSpeed": 18.5,
      "verticalSpeed": 0.2
    },
    "attitude": {
      "heading": 45,
      "roll": 2.1,
      "pitch": 5.3
    },
    "battery": 72
  }
}
```

**Mission Update Stream**:
```json
{
  "type": "MISSION_UPDATE",
  "channel": "mission",
  "id": "MSN-20250101-1234",
  "timestamp": "2025-01-01T10:05:00.000Z",
  "data": {
    "previousStatus": "PICKUP",
    "currentStatus": "EN_ROUTE",
    "progress": 15,
    "eta": 780,
    "event": "DEPARTED_PICKUP"
  }
}
```

**Event Stream**:
```json
{
  "type": "EVENT",
  "channel": "drone",
  "id": "WIA-DRN-X1-0042",
  "timestamp": "2025-01-01T10:06:30.000Z",
  "data": {
    "eventType": "WARNING",
    "code": "WIND_ADVISORY",
    "severity": "MEDIUM",
    "message": "Wind speed increasing, monitoring situation",
    "details": {
      "currentWindSpeed": 8.5,
      "threshold": 10
    }
  }
}
```

---

## 5.5 Fleet Management API

### Get Fleet Status

**GET /api/v1/fleets/{fleetId}**

```json
// Response
{
  "success": true,
  "data": {
    "fleetId": "FLEET-001",
    "name": "SF Bay Area Fleet",
    "summary": {
      "totalDrones": 25,
      "available": 18,
      "inFlight": 5,
      "charging": 2,
      "maintenance": 0
    },
    "drones": [
      {
        "droneId": "WIA-DRN-X1-0042",
        "status": "IN_FLIGHT",
        "currentMission": "MSN-20250101-1234",
        "battery": 72
      },
      ...
    ],
    "todayMetrics": {
      "totalFlights": 142,
      "totalDeliveries": 140,
      "successRate": 98.6,
      "totalDistance": 1250.5,
      "totalFlightTime": 42.3,
      "averageDeliveryTime": 12.5
    }
  }
}
```

### Fleet Analytics

**GET /api/v1/fleets/{fleetId}/analytics**

Query parameters:
- `period`: DAY, WEEK, MONTH, YEAR
- `startDate`: Period start
- `endDate`: Period end
- `metrics`: Comma-separated metric names

```json
// Response
{
  "success": true,
  "data": {
    "period": {
      "start": "2025-01-01",
      "end": "2025-01-31"
    },
    "metrics": {
      "deliveries": {
        "total": 4250,
        "successful": 4180,
        "failed": 35,
        "cancelled": 35,
        "successRate": 98.35
      },
      "flights": {
        "total": 4300,
        "totalDistance": 52500,
        "totalFlightTime": 1850,
        "averageFlightTime": 25.8
      },
      "efficiency": {
        "deliveriesPerDrone": 170,
        "utilizationRate": 72.5,
        "energyPerDelivery": 0.85
      },
      "reliability": {
        "mtbf": 125.5,
        "downtime": 2.3,
        "incidentRate": 0.12
      }
    },
    "trends": {
      "daily": [
        { "date": "2025-01-01", "deliveries": 142, "successRate": 98.5 },
        { "date": "2025-01-02", "deliveries": 138, "successRate": 99.2 },
        ...
      ]
    }
  }
}
```

---

## 5.6 Authentication and Security

### API Key Authentication

```http
GET /api/v1/missions HTTP/1.1
Host: api.wia.com
Authorization: ApiKey EXAMPLE_API_KEY_REPLACE_ME
```

### JWT Bearer Token

```http
POST /api/v1/auth/token HTTP/1.1
Host: api.wia.com
Content-Type: application/json

{
  "clientId": "client_abc123",
  "clientSecret": "secret_xyz789",
  "scope": "missions:read missions:write drones:read"
}

// Response
{
  "access_token": "eyJhbGciOiJIUzI1NiIs...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "missions:read missions:write drones:read"
}
```

### Permission Scopes

| Scope | Access Level |
|-------|--------------|
| missions:read | View mission data |
| missions:write | Create/modify missions |
| drones:read | View drone data |
| drones:control | Send commands to drones |
| fleet:manage | Fleet management access |
| admin | Full administrative access |

### Rate Limiting

```http
HTTP/1.1 200 OK
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 998
X-RateLimit-Reset: 1704110400
```

| Endpoint Category | Limit | Window |
|-------------------|-------|--------|
| Read operations | 1000/min | Per API key |
| Write operations | 100/min | Per API key |
| Streaming connections | 10 | Concurrent |
| Drone commands | 60/min | Per drone |

---

## 5.7 SDK Implementation

### TypeScript/JavaScript SDK

```typescript
import { WIADeliveryClient } from '@wia/delivery-sdk';

// Initialize client
const client = new WIADeliveryClient({
  apiKey: 'EXAMPLE_API_KEY_REPLACE_ME',
  environment: 'production'
});

// Create a mission
async function createDeliveryMission() {
  const mission = await client.missions.create({
    pickup: {
      location: { latitude: 37.7749, longitude: -122.4194 },
      address: '123 Market St, San Francisco, CA',
      contact: { name: 'John Doe', phone: '+1-555-0100' }
    },
    dropoff: {
      location: { latitude: 37.7849, longitude: -122.4094 },
      address: '456 Mission St, San Francisco, CA',
      contact: { name: 'Jane Smith', phone: '+1-555-0200' }
    },
    package: {
      weight: 2.5,
      dimensions: { length: 30, width: 20, height: 15 }
    },
    priority: 'STANDARD'
  });

  console.log(`Mission created: ${mission.missionId}`);
  return mission;
}

// Track mission in real-time
async function trackMission(missionId: string) {
  const stream = client.missions.track(missionId);

  stream.on('position', (data) => {
    console.log(`Position: ${data.latitude}, ${data.longitude}`);
  });

  stream.on('status', (data) => {
    console.log(`Status: ${data.status}, Progress: ${data.progress}%`);
  });

  stream.on('delivered', (data) => {
    console.log(`Delivered at ${data.timestamp}`);
    stream.close();
  });
}

// Get fleet status
async function getFleetStatus() {
  const fleet = await client.fleets.get('FLEET-001');

  console.log(`Total drones: ${fleet.summary.totalDrones}`);
  console.log(`Available: ${fleet.summary.available}`);
  console.log(`In flight: ${fleet.summary.inFlight}`);

  return fleet;
}
```

### Python SDK

```python
from wia_delivery import DeliveryClient

# Initialize client
client = DeliveryClient(
    api_key='EXAMPLE_API_KEY_REPLACE_ME',
    environment='production'
)

# Create a mission
def create_delivery_mission():
    mission = client.missions.create(
        pickup={
            'location': {'latitude': 37.7749, 'longitude': -122.4194},
            'address': '123 Market St, San Francisco, CA',
            'contact': {'name': 'John Doe', 'phone': '+1-555-0100'}
        },
        dropoff={
            'location': {'latitude': 37.7849, 'longitude': -122.4094},
            'address': '456 Mission St, San Francisco, CA',
            'contact': {'name': 'Jane Smith', 'phone': '+1-555-0200'}
        },
        package={
            'weight': 2.5,
            'dimensions': {'length': 30, 'width': 20, 'height': 15}
        },
        priority='STANDARD'
    )

    print(f"Mission created: {mission.mission_id}")
    return mission

# Track mission in real-time
def track_mission(mission_id: str):
    for update in client.missions.track(mission_id):
        if update.type == 'position':
            print(f"Position: {update.latitude}, {update.longitude}")
        elif update.type == 'status':
            print(f"Status: {update.status}, Progress: {update.progress}%")
        elif update.type == 'delivered':
            print(f"Delivered at {update.timestamp}")
            break

# Get analytics
def get_fleet_analytics():
    analytics = client.fleets.analytics(
        fleet_id='FLEET-001',
        period='MONTH',
        start_date='2025-01-01',
        end_date='2025-01-31'
    )

    print(f"Total deliveries: {analytics.metrics.deliveries.total}")
    print(f"Success rate: {analytics.metrics.deliveries.success_rate}%")

    return analytics
```

---

## Chapter Summary

The WIA Delivery Drone API provides a comprehensive interface for managing all aspects of drone delivery operations. The RESTful design enables intuitive resource management, while WebSocket streaming provides real-time visibility into mission progress and drone status.

Mission management APIs support the full delivery lifecycle from creation through completion, with rich status tracking and event notifications. Drone management enables registration, monitoring, and command/control operations. Fleet management APIs provide aggregated views and analytics for operational optimization.

Authentication via API keys and JWT tokens ensures secure access, with granular permission scopes enabling appropriate access control. Rate limiting protects system resources while ensuring fair usage. TypeScript and Python SDKs simplify integration for developers.

---

## Key Takeaways

1. **RESTful APIs for resources, WebSocket for real-time** streaming
2. **Consistent response formats** with success/error structure
3. **Comprehensive mission lifecycle** management from creation to completion
4. **Real-time tracking** via WebSocket subscriptions
5. **SDK implementations** in TypeScript and Python simplify integration

---

## Review Questions

1. What HTTP status code indicates a mission was successfully created?
2. How would you implement real-time position tracking for a delivery mission?
3. Design an API endpoint for batch mission creation (multiple deliveries).
4. What authentication method is appropriate for server-to-server integration?
5. How does rate limiting protect the API from abuse?

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity
