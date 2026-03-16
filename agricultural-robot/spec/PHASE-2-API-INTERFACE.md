# WIA-AGRI-003: Agricultural Robot Standard
## Phase 2: API Interface Specification

**Version:** 1.1.0
**Status:** ✅ Complete
**Last Updated:** 2025-12-26
**Category:** Agriculture (AGRI)

---

## 1. Overview

### 1.1 Purpose
This specification defines the RESTful API, WebSocket, and GraphQL interfaces for agricultural robots, enabling real-time communication, task management, and data exchange between robots, farm management systems, and cloud platforms.

### 1.2 API Architecture

```
┌─────────────────┐
│  Farm Manager   │
│   Dashboard     │
└────────┬────────┘
         │
    HTTPS/WSS
         │
┌────────▼────────┐
│   WIA Gateway   │
│   API Server    │
└────────┬────────┘
         │
    ┌────┴────┬────────┬────────┐
    │         │        │        │
┌───▼──┐  ┌──▼──┐  ┌──▼──┐  ┌──▼──┐
│Robot1│  │Robot2│  │Robot3│  │Robot4│
└──────┘  └─────┘  └─────┘  └─────┘
```

### 1.3 Authentication

All API requests must include authentication:

```http
Authorization: Bearer <JWT_TOKEN>
X-WIA-Robot-ID: AGRI-ROBOT-2025-001
X-WIA-Standard: AGRI-003
X-WIA-Version: 1.0.0
```

---

## 2. RESTful API Endpoints

### 2.1 Robot Registration

**POST** `/api/v1/robots/register`

Register a new agricultural robot with the WIA network.

**Request:**
```json
{
  "robotId": "AGRI-ROBOT-2025-001",
  "robotType": "autonomous-tractor",
  "manufacturer": {
    "name": "AgriBot Technologies",
    "did": "did:wia:agribot-tech",
    "certification": "WIA-CERT-2025-042"
  },
  "specifications": {
    "dimensions": {
      "width": 2.5,
      "length": 4.2,
      "height": 2.8,
      "weight": 3500
    },
    "powerSystem": {
      "type": "electric",
      "capacity": 150,
      "range": 12
    },
    "maxSpeed": 25,
    "workingWidth": 6.0
  }
}
```

**Response (201 Created):**
```json
{
  "status": "success",
  "robotId": "AGRI-ROBOT-2025-001",
  "did": "did:wia:agri-robot:2025-001",
  "apiKey": "wia_agri_xxxxxxxxxxxx",
  "registered": "2025-12-26T10:30:00Z"
}
```

---

### 2.2 Telemetry Submission

**POST** `/api/v1/robots/{robotId}/telemetry`

Submit real-time telemetry data from the robot.

**Request:**
```json
{
  "timestamp": "2025-12-26T10:35:22Z",
  "location": {
    "latitude": 37.5665,
    "longitude": 126.9780,
    "altitude": 120.5,
    "accuracy": "RTK",
    "heading": 275
  },
  "status": {
    "workMode": "plowing",
    "battery": 85,
    "speed": 5.2,
    "temperature": 28.5
  },
  "sensors": {
    "soilMoisture": 45.2,
    "soilTemperature": 18.3,
    "airTemperature": 22.1,
    "humidity": 65,
    "cropHealth": 0.87
  }
}
```

**Response (200 OK):**
```json
{
  "status": "received",
  "timestamp": "2025-12-26T10:35:23Z",
  "nextUpdate": "2025-12-26T10:35:32Z"
}
```

---

### 2.3 Task Assignment

**POST** `/api/v1/robots/{robotId}/tasks`

Assign a new task to a robot.

**Request:**
```json
{
  "taskType": "harvesting",
  "fieldId": "FIELD-KR-2025-042",
  "priority": "high",
  "scheduledStart": "2025-12-26T14:00:00Z",
  "parameters": {
    "cropType": "wheat",
    "workingDepth": 0,
    "targetYield": 4500
  }
}
```

**Response (201 Created):**
```json
{
  "taskId": "TASK-2025-001-H42",
  "status": "assigned",
  "estimatedDuration": 180,
  "estimatedCompletion": "2025-12-26T17:00:00Z"
}
```

---

### 2.4 Task Status Update

**PATCH** `/api/v1/tasks/{taskId}`

Update task progress and status.

**Request:**
```json
{
  "status": "in-progress",
  "progress": 45,
  "currentLocation": {
    "latitude": 37.5670,
    "longitude": 126.9785
  },
  "estimatedCompletion": "2025-12-26T16:30:00Z"
}
```

**Response (200 OK):**
```json
{
  "taskId": "TASK-2025-001-H42",
  "status": "updated",
  "timestamp": "2025-12-26T15:15:00Z"
}
```

---

### 2.5 Field Map Retrieval

**GET** `/api/v1/fields/{fieldId}/map`

Retrieve detailed field map including boundaries and obstacles.

**Response (200 OK):**
```json
{
  "fieldId": "FIELD-KR-2025-042",
  "farmId": "FARM-KR-2025-042",
  "boundary": {
    "type": "Polygon",
    "coordinates": [
      [
        [126.9770, 37.5660],
        [126.9790, 37.5660],
        [126.9790, 37.5680],
        [126.9770, 37.5680],
        [126.9770, 37.5660]
      ]
    ]
  },
  "area": 50,
  "obstacles": [
    {
      "type": "tree",
      "location": {"latitude": 37.5670, "longitude": 126.9780},
      "radius": 3
    }
  ]
}
```

---

### 2.6 Fleet Status

**GET** `/api/v1/farms/{farmId}/fleet/status`

Get real-time status of all robots in a farm fleet.

**Response (200 OK):**
```json
{
  "farmId": "FARM-KR-2025-042",
  "timestamp": "2025-12-26T15:20:00Z",
  "robots": [
    {
      "robotId": "AGRI-ROBOT-2025-001",
      "status": "working",
      "currentTask": "TASK-2025-001-H42",
      "location": {"latitude": 37.5670, "longitude": 126.9785},
      "battery": 85,
      "progress": 45
    },
    {
      "robotId": "AGRI-ROBOT-2025-002",
      "status": "charging",
      "location": {"latitude": 37.5665, "longitude": 126.9770},
      "battery": 35
    }
  ]
}
```

---

### 2.7 Crop Health Report

**POST** `/api/v1/fields/{fieldId}/health-report`

Submit crop health monitoring data.

**Request:**
```json
{
  "reportId": "HEALTH-2025-042",
  "timestamp": "2025-12-26T15:30:00Z",
  "cropType": "wheat",
  "growthStage": "flowering",
  "measurements": {
    "ndvi": 0.78,
    "leafAreaIndex": 4.2,
    "chlorophyllContent": 42.5,
    "plantHeight": 85,
    "canopyCover": 92
  },
  "issues": [
    {
      "type": "weed-pressure",
      "severity": "medium",
      "location": {"latitude": 37.5672, "longitude": 126.9782},
      "area": 150
    }
  ]
}
```

**Response (201 Created):**
```json
{
  "reportId": "HEALTH-2025-042",
  "status": "processed",
  "recommendations": [
    {
      "action": "weeding",
      "priority": "medium",
      "estimatedCost": 50
    }
  ]
}
```

---

### 2.8 Maintenance Alert

**POST** `/api/v1/robots/{robotId}/maintenance/alert`

Report maintenance issues or schedule preventive maintenance.

**Request:**
```json
{
  "alertType": "scheduled",
  "severity": "medium",
  "component": "hydraulic-system",
  "description": "Oil change required - 1500 operating hours reached",
  "operatingHours": 1502,
  "nextMaintenance": 2000
}
```

**Response (200 OK):**
```json
{
  "alertId": "MAINT-2025-042",
  "status": "scheduled",
  "scheduledDate": "2025-12-28T09:00:00Z",
  "estimatedDowntime": 120
}
```

---

## 3. WebSocket Real-time API

### 3.1 Connection

```javascript
const ws = new WebSocket('wss://api.wiastandards.com/v1/ws');

ws.onopen = () => {
  ws.send(JSON.stringify({
    type: 'auth',
    token: 'Bearer <JWT_TOKEN>',
    robotId: 'AGRI-ROBOT-2025-001'
  }));
};
```

### 3.2 Real-time Telemetry Stream

**Subscribe:**
```json
{
  "type": "subscribe",
  "channel": "telemetry",
  "robotId": "AGRI-ROBOT-2025-001",
  "interval": 10
}
```

**Receive:**
```json
{
  "type": "telemetry",
  "robotId": "AGRI-ROBOT-2025-001",
  "timestamp": "2025-12-26T15:45:22Z",
  "location": {
    "latitude": 37.5670,
    "longitude": 126.9785
  },
  "status": {
    "workMode": "plowing",
    "battery": 84,
    "speed": 5.2
  }
}
```

### 3.3 Command Execution

**Send Command:**
```json
{
  "type": "command",
  "robotId": "AGRI-ROBOT-2025-001",
  "command": "pause",
  "reason": "operator-intervention"
}
```

**Receive Acknowledgment:**
```json
{
  "type": "command-ack",
  "commandId": "CMD-2025-042",
  "status": "executed",
  "timestamp": "2025-12-26T15:46:00Z"
}
```

---

## 4. GraphQL API

### 4.1 Schema

```graphql
type Robot {
  robotId: ID!
  robotType: RobotType!
  status: RobotStatus!
  location: Location
  battery: Int
  currentTask: Task
  telemetry: [Telemetry!]!
}

type Field {
  fieldId: ID!
  farmId: ID!
  boundary: Polygon!
  area: Float!
  cropType: String
  obstacles: [Obstacle!]!
}

type Task {
  taskId: ID!
  robotId: ID!
  fieldId: ID!
  taskType: TaskType!
  status: TaskStatus!
  progress: Int
  scheduledStart: DateTime!
  estimatedDuration: Int
}

type Query {
  robot(robotId: ID!): Robot
  field(fieldId: ID!): Field
  task(taskId: ID!): Task
  fleet(farmId: ID!): [Robot!]!
}

type Mutation {
  assignTask(input: TaskInput!): Task!
  updateTaskStatus(taskId: ID!, status: TaskStatus!, progress: Int): Task!
  submitTelemetry(input: TelemetryInput!): Boolean!
}

type Subscription {
  robotTelemetry(robotId: ID!): Telemetry!
  fleetStatus(farmId: ID!): [Robot!]!
}
```

### 4.2 Example Queries

**Get Robot Status:**
```graphql
query GetRobotStatus {
  robot(robotId: "AGRI-ROBOT-2025-001") {
    robotId
    robotType
    status
    location {
      latitude
      longitude
    }
    battery
    currentTask {
      taskId
      taskType
      progress
    }
  }
}
```

**Assign Task:**
```graphql
mutation AssignHarvestingTask {
  assignTask(input: {
    robotId: "AGRI-ROBOT-2025-001"
    fieldId: "FIELD-KR-2025-042"
    taskType: HARVESTING
    scheduledStart: "2025-12-26T14:00:00Z"
  }) {
    taskId
    status
    estimatedDuration
  }
}
```

**Subscribe to Telemetry:**
```graphql
subscription RobotTelemetry {
  robotTelemetry(robotId: "AGRI-ROBOT-2025-001") {
    timestamp
    location {
      latitude
      longitude
    }
    battery
    speed
  }
}
```

---

## 5. Error Handling

### 5.1 HTTP Status Codes

| Code | Meaning | Description |
|------|---------|-------------|
| 200 | OK | Request successful |
| 201 | Created | Resource created |
| 400 | Bad Request | Invalid input data |
| 401 | Unauthorized | Missing or invalid auth |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource doesn't exist |
| 409 | Conflict | Resource conflict |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server error |
| 503 | Service Unavailable | Maintenance mode |

### 5.2 Error Response Format

```json
{
  "error": {
    "code": "INVALID_ROBOT_ID",
    "message": "Robot ID format is invalid",
    "details": {
      "field": "robotId",
      "expected": "AGRI-ROBOT-YYYY-NNN",
      "received": "ROBOT-001"
    },
    "timestamp": "2025-12-26T16:00:00Z"
  }
}
```

---

## 6. Rate Limiting

### 6.1 Limits by Endpoint

| Endpoint | Rate Limit | Window |
|----------|------------|--------|
| `/telemetry` | 60 req/min | 1 minute |
| `/tasks` | 100 req/hour | 1 hour |
| `/fields` | 300 req/hour | 1 hour |
| `/fleet` | 60 req/min | 1 minute |

### 6.2 Rate Limit Headers

```http
X-RateLimit-Limit: 60
X-RateLimit-Remaining: 45
X-RateLimit-Reset: 1640524800
```

---

## 7. Webhooks

### 7.1 Webhook Registration

**POST** `/api/v1/webhooks`

```json
{
  "url": "https://farm.example.com/webhook",
  "events": [
    "task.completed",
    "robot.battery.low",
    "maintenance.required"
  ],
  "secret": "webhook_secret_key"
}
```

### 7.2 Webhook Payload

```json
{
  "event": "task.completed",
  "timestamp": "2025-12-26T17:00:00Z",
  "data": {
    "taskId": "TASK-2025-001-H42",
    "robotId": "AGRI-ROBOT-2025-001",
    "fieldId": "FIELD-KR-2025-042",
    "duration": 178,
    "areaCompleted": 50
  },
  "signature": "sha256=..."
}
```

---

## 8. SDK Examples

### 8.1 JavaScript/Node.js

```javascript
const WIA_AGRI = require('@wia/agricultural-robot-sdk');

const client = new WIA_AGRI.Client({
  apiKey: 'wia_agri_xxxxxxxxxxxx',
  robotId: 'AGRI-ROBOT-2025-001'
});

// Submit telemetry
await client.telemetry.submit({
  location: { latitude: 37.5665, longitude: 126.9780 },
  status: { workMode: 'plowing', battery: 85 },
  sensors: { soilMoisture: 45.2 }
});

// Get assigned tasks
const tasks = await client.tasks.list({ status: 'assigned' });
```

### 8.2 Python

```python
from wia_agri import AgriRobotClient

client = AgriRobotClient(
    api_key='wia_agri_xxxxxxxxxxxx',
    robot_id='AGRI-ROBOT-2025-001'
)

# Submit telemetry
client.telemetry.submit({
    'location': {'latitude': 37.5665, 'longitude': 126.9780},
    'status': {'workMode': 'plowing', 'battery': 85}
})

# Stream real-time telemetry
for telemetry in client.telemetry.stream():
    print(f"Battery: {telemetry['status']['battery']}%")
```

### 8.3 ROS2 Integration

```python
import rclpy
from wia_agri_ros2 import WIATelemetryPublisher

node = WIATelemetryPublisher(
    robot_id='AGRI-ROBOT-2025-001',
    api_key='wia_agri_xxxxxxxxxxxx'
)

# Automatically publishes ROS topics to WIA API
node.publish_telemetry()
```

---

## 9. Security Best Practices

### 9.1 API Key Management

- Rotate API keys every 90 days
- Use environment variables, never hardcode
- Implement key revocation mechanism
- Monitor for unauthorized access

### 9.2 Data Encryption

- TLS 1.3 for all HTTPS traffic
- End-to-end encryption for sensitive data
- Digital signatures for command verification

### 9.3 Access Control

```json
{
  "role": "robot-operator",
  "permissions": [
    "telemetry:write",
    "tasks:read",
    "fields:read"
  ],
  "restrictions": {
    "farmId": "FARM-KR-2025-042",
    "robotIds": ["AGRI-ROBOT-2025-001", "AGRI-ROBOT-2025-002"]
  }
}
```

---

## 10. Integration with WIA Ecosystem

### 10.1 WIA-INTENT Integration

Natural language task assignment:

```json
{
  "intent": "Plow the north field tomorrow morning",
  "parsed": {
    "taskType": "plowing",
    "fieldId": "FIELD-KR-2025-042-NORTH",
    "scheduledStart": "2025-12-27T06:00:00Z"
  }
}
```

### 10.2 WIA-BLOCKCHAIN Integration

Immutable task logging:

```json
{
  "blockchainId": "0x7f8a9b2c3d4e5f6a",
  "taskId": "TASK-2025-001-H42",
  "hash": "sha256:...",
  "previousHash": "sha256:...",
  "timestamp": "2025-12-26T17:00:00Z"
}
```

---

**© 2025 WIA (World Certification Industry Association)**
**弘益人間 · Benefit All Humanity**
**License:** MIT
