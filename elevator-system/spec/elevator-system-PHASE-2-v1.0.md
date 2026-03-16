# WIA Elevator System Standard
## Phase 2: API Interface Specification v1.0

**Status:** APPROVED  
**Date:** 2025-12-26  
**弘益人間** · Benefit All Humanity

---

## 1. Introduction

Phase 2 defines standardized APIs for elevator control, monitoring, and integration. RESTful HTTP, WebSocket, and GraphQL interfaces enable consistent programmatic access to elevator systems across all manufacturers.

### 1.1 Scope

- RESTful HTTP API endpoints
- WebSocket for real-time communication
- GraphQL query interface
- Authentication and authorization
- Rate limiting and throttling
- API versioning
- Error handling standards

---

## 2. RESTful HTTP API

### 2.1 Base URL Structure

```
https://{host}/api/v1/elevator/{elevatorId}
```

### 2.2 Core Endpoints

#### GET /api/v1/elevator/{elevatorId}/status
Retrieve current elevator status

**Response (200 OK):**
```json
{
  "elevatorId": "ELV-001",
  "buildingId": "BLD-2025",
  "status": {
    "currentFloor": 5,
    "direction": "UP",
    "doorStatus": "CLOSED",
    "occupancy": 8,
    "speed": 2.5
  },
  "timestamp": "2025-12-26T10:30:00Z"
}
```

#### POST /api/v1/elevator/dispatch
Request elevator dispatch

**Request Body:**
```json
{
  "originFloor": 1,
  "destinationFloor": 15,
  "passengerCount": 3,
  "priority": "NORMAL",
  "accessibility": {
    "wheelchairRequired": false
  }
}
```

**Response (201 Created):**
```json
{
  "requestId": "REQ-2025-12345",
  "assignedElevator": "ELV-002",
  "estimatedWaitTime": 12,
  "estimatedArrival": "2025-12-26T10:30:27Z"
}
```

#### PUT /api/v1/elevator/{elevatorId}/command
Send control command

**Request Body:**
```json
{
  "command": "GO_TO_FLOOR",
  "parameters": {
    "targetFloor": 10,
    "priority": "NORMAL"
  },
  "authentication": {
    "token": "Bearer eyJhbGci..."
  }
}
```

#### GET /api/v1/elevator/{elevatorId}/telemetry
Retrieve sensor telemetry

**Query Parameters:**
- `start` - ISO 8601 start timestamp
- `end` - ISO 8601 end timestamp
- `sensors` - Comma-separated sensor list

**Response (200 OK):**
```json
{
  "elevatorId": "ELV-001",
  "period": {
    "start": "2025-12-26T00:00:00Z",
    "end": "2025-12-26T23:59:59Z"
  },
  "dataPoints": [
    {
      "timestamp": "2025-12-26T10:30:00Z",
      "sensors": {
        "loadWeight": 640,
        "temperature": 22.5,
        "vibration": 0.02
      }
    }
  ]
}
```

#### GET /api/v1/building/{buildingId}/elevators
List all elevators in building

**Response (200 OK):**
```json
{
  "buildingId": "BLD-2025",
  "elevatorCount": 12,
  "elevators": [
    {
      "elevatorId": "ELV-001",
      "type": "PASSENGER",
      "floors": [1, 2, 3, 4, 5, 6, 7, 8, 9, 10],
      "status": "OPERATIONAL"
    }
  ]
}
```

---

## 3. WebSocket Real-Time API

### 3.1 Connection

```
wss://{host}/ws/elevator/{elevatorId}
```

### 3.2 Message Format

All WebSocket messages use JSON:

```json
{
  "type": "STATUS_UPDATE",
  "elevatorId": "ELV-001",
  "timestamp": "2025-12-26T10:30:00Z",
  "payload": {
    "currentFloor": 6,
    "direction": "UP"
  }
}
```

**Message Types:**
- `STATUS_UPDATE` - Status change notification
- `TELEMETRY` - Real-time sensor data
- `EVENT` - Event notification
- `ALARM` - Alarm trigger
- `COMMAND` - Control command (bidirectional)
- `HEARTBEAT` - Connection keepalive

### 3.3 Subscription Model

Clients subscribe to specific data streams:

```json
{
  "action": "SUBSCRIBE",
  "streams": ["STATUS", "TELEMETRY", "ALARMS"],
  "elevatorIds": ["ELV-001", "ELV-002"]
}
```

---

## 4. GraphQL API

### 4.1 Schema

```graphql
type Elevator {
  id: ID!
  buildingId: String!
  status: ElevatorStatus!
  telemetry(start: DateTime, end: DateTime): [TelemetryData!]!
  maintenance: [MaintenanceRecord!]!
  traffic: TrafficMetrics
}

type ElevatorStatus {
  currentFloor: Int!
  direction: Direction!
  doorStatus: DoorStatus!
  occupancy: Int
  speed: Float
  timestamp: DateTime!
}

enum Direction {
  UP
  DOWN
  IDLE
}

enum DoorStatus {
  OPEN
  CLOSED
  OPENING
  CLOSING
}

type Query {
  elevator(id: ID!): Elevator
  building(id: ID!): Building
  elevators(buildingId: ID!): [Elevator!]!
}

type Mutation {
  dispatchElevator(input: DispatchInput!): DispatchResponse!
  sendCommand(elevatorId: ID!, command: CommandInput!): CommandResponse!
}
```

### 4.2 Example Queries

**Get elevator status:**
```graphql
query {
  elevator(id: "ELV-001") {
    status {
      currentFloor
      direction
      doorStatus
    }
  }
}
```

**Dispatch elevator:**
```graphql
mutation {
  dispatchElevator(input: {
    originFloor: 1
    destinationFloor: 15
    passengerCount: 3
  }) {
    requestId
    assignedElevator
    estimatedWaitTime
  }
}
```

---

## 5. Authentication & Authorization

### 5.1 OAuth 2.0

All API access requires OAuth 2.0 authentication:

```http
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

### 5.2 JWT Token Structure

```json
{
  "sub": "user@example.com",
  "iss": "https://auth.wiastandards.com",
  "aud": "elevator-api",
  "exp": 1735228800,
  "iat": 1735142400,
  "scope": ["elevator:read", "elevator:control"]
}
```

### 5.3 Scopes

- `elevator:read` - Read status and telemetry
- `elevator:control` - Send control commands
- `elevator:admin` - Full administrative access
- `building:read` - Read building data
- `maintenance:write` - Update maintenance records

### 5.4 API Keys

For server-to-server communication:

```http
X-API-Key: wia_live_sk_abc123def456...
```

---

## 6. Rate Limiting

### 6.1 Rate Limits

- **Standard tier:** 100 requests/minute
- **Professional tier:** 1,000 requests/minute
- **Enterprise tier:** 10,000 requests/minute

### 6.2 Rate Limit Headers

```http
X-RateLimit-Limit: 100
X-RateLimit-Remaining: 87
X-RateLimit-Reset: 1735142460
```

### 6.3 Throttling Response

```http
HTTP/1.1 429 Too Many Requests
Retry-After: 45

{
  "error": "RATE_LIMIT_EXCEEDED",
  "message": "Rate limit exceeded. Retry after 45 seconds.",
  "limit": 100,
  "retryAfter": 45
}
```

---

## 7. Error Handling

### 7.1 Standard Error Response

```json
{
  "error": {
    "code": "ELEVATOR_OFFLINE",
    "message": "Elevator ELV-001 is currently offline",
    "requestId": "req_abc123",
    "timestamp": "2025-12-26T10:30:00Z",
    "details": {
      "elevatorId": "ELV-001",
      "lastSeen": "2025-12-26T09:15:00Z"
    }
  }
}
```

### 7.2 HTTP Status Codes

- `200 OK` - Successful request
- `201 Created` - Resource created
- `400 Bad Request` - Invalid request
- `401 Unauthorized` - Missing/invalid authentication
- `403 Forbidden` - Insufficient permissions
- `404 Not Found` - Resource not found
- `429 Too Many Requests` - Rate limit exceeded
- `500 Internal Server Error` - Server error
- `503 Service Unavailable` - System unavailable

### 7.3 Error Codes

- `INVALID_REQUEST` - Malformed request
- `ELEVATOR_OFFLINE` - Elevator not responding
- `UNAUTHORIZED_ACCESS` - Permission denied
- `INVALID_FLOOR` - Floor does not exist
- `OVERLOAD_CONDITION` - Elevator overloaded
- `MAINTENANCE_MODE` - Elevator in maintenance
- `EMERGENCY_STOP` - Emergency stop activated

---

## 8. API Versioning

### 8.1 Version Strategy

APIs use URL-based versioning:
- `/api/v1/` - Version 1 (current)
- `/api/v2/` - Version 2 (future)

### 8.2 Deprecation Policy

1. **Announcement:** 6 months before deprecation
2. **Deprecation:** Version marked deprecated
3. **Sunset:** Version removed after 12 months

### 8.3 Version Headers

```http
X-API-Version: 1.0
X-API-Deprecated: false
X-API-Sunset-Date: null
```

---

## 9. SDK Support

### 9.1 Official SDKs

WIA provides official SDKs for:
- **TypeScript/JavaScript** (Node.js, Browser)
- **Python** (3.8+)
- **Java** (11+)
- **C#** (.NET 6.0+)
- **Go** (1.19+)

### 9.2 SDK Example (TypeScript)

```typescript
import { WIAElevatorClient } from '@wia/elevator-sdk';

const client = new WIAElevatorClient({
  apiKey: process.env.WIA_API_KEY
});

// Get elevator status
const status = await client.elevators.get('ELV-001').status();
console.log(`Floor: ${status.currentFloor}, Direction: ${status.direction}`);

// Dispatch elevator
const dispatch = await client.dispatch.request({
  originFloor: 1,
  destinationFloor: 15
});
console.log(`Assigned: ${dispatch.assignedElevator}, Wait: ${dispatch.estimatedWaitTime}s`);
```

---

## 10. Compliance

Phase 2 compliance requires:
1. ✅ RESTful API implementation
2. ✅ WebSocket real-time support
3. ✅ OAuth 2.0 authentication
4. ✅ Rate limiting
5. ✅ Standard error handling
6. ✅ API versioning
7. ✅ GraphQL (recommended)

---

**© 2025 WIA - World Certification Industry Association**  
**MIT License**  
**弘益人間 · Benefit All Humanity**
