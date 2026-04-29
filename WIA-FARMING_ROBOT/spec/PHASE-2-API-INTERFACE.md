# WIA-FARMING_ROBOT: PHASE 2 - API Interface Specification

**Version:** 1.0
**Status:** Active
**Last Updated:** 2026-01-12

---

## Overview

This specification defines RESTful APIs and real-time interfaces for agricultural robot control, monitoring, and data exchange.

---

## REST API Endpoints

### Base URL

```
https://api.farming-robot.wia.org/v1
```

### Authentication

```http
Authorization: Bearer <JWT_TOKEN>
X-API-Key: <API_KEY>
X-Robot-ID: <ROBOT_UUID>
```

---

## 1. Robot Management

### Register Robot

```http
POST /robots
Content-Type: application/json

{
  "manufacturer": "John Deere",
  "model": "8R 410",
  "serialNumber": "1M08R410XXXXXX",
  "type": "AUTONOMOUS_TRACTOR",
  "certifications": ["ISO18497", "FIRA-L4"]
}

Response 201:
{
  "robotId": "550e8400-e29b-41d4-a716-446655440000",
  "status": "registered",
  "apiKey": "robot_ak_...",
  "registeredAt": "2026-01-12T10:30:00Z"
}
```

### Get Robot Status

```http
GET /robots/{robotId}/status

Response 200:
{
  "robotId": "550e8400-e29b-41d4-a716-446655440000",
  "status": "WORKING",
  "position": {
    "latitude": 42.3601,
    "longitude": -71.0589,
    "accuracy": 2.3
  },
  "battery": 87.5,
  "currentTask": "task-2026-001",
  "health": {
    "overall": "HEALTHY",
    "alerts": []
  }
}
```

### Update Robot Configuration

```http
PATCH /robots/{robotId}/config

{
  "workingSpeed": 2.5,
  "safetyDistance": 5.0,
  "autoResumeEnabled": true
}

Response 200:
{
  "success": true,
  "updated": ["workingSpeed", "safetyDistance", "autoResumeEnabled"]
}
```

---

## 2. Task Management

### Create Task

```http
POST /tasks
Content-Type: application/json

{
  "robotId": "550e8400-e29b-41d4-a716-446655440000",
  "taskType": "SEEDING",
  "fieldId": "field-001",
  "parameters": {
    "seedType": "corn",
    "seedingRate": 75000,
    "rowSpacing": 0.76,
    "depth": 5.0
  },
  "schedule": {
    "startTime": "2026-01-12T14:00:00Z",
    "priority": "HIGH"
  }
}

Response 201:
{
  "taskId": "task-2026-001",
  "status": "SCHEDULED",
  "estimatedDuration": 4.5,
  "estimatedCompletion": "2026-01-12T18:30:00Z"
}
```

### Get Task Status

```http
GET /tasks/{taskId}

Response 200:
{
  "taskId": "task-2026-001",
  "status": "IN_PROGRESS",
  "progress": 67.5,
  "startedAt": "2026-01-12T14:00:00Z",
  "currentPosition": {
    "latitude": 42.3605,
    "longitude": -71.0582
  },
  "metrics": {
    "areaCovered": 27.5,
    "areaRemaining": 13.2,
    "workRate": 6.2
  }
}
```

### Pause/Resume Task

```http
POST /tasks/{taskId}/pause
POST /tasks/{taskId}/resume

Response 200:
{
  "taskId": "task-2026-001",
  "status": "PAUSED",
  "reason": "USER_REQUESTED"
}
```

---

## 3. Path Planning

### Generate Path Plan

```http
POST /pathplanning/generate

{
  "fieldId": "field-001",
  "taskType": "SEEDING",
  "workingWidth": 12.2,
  "overlap": 0.15,
  "pattern": "AB_LINE",
  "constraints": {
    "maxSlope": 15,
    "minTurningRadius": 8.0
  }
}

Response 200:
{
  "planId": "plan-2026-001",
  "pathSegments": 47,
  "totalDistance": 5430,
  "estimatedTime": 4.2,
  "efficiency": 94.3
}
```

### Get Path Plan

```http
GET /pathplanning/{planId}

Response 200:
{
  "planId": "plan-2026-001",
  "field": {
    "fieldId": "field-001",
    "area": 40.5,
    "boundary": {...}
  },
  "path": [
    {
      "segmentId": "seg-001",
      "startPoint": {"lat": 42.3601, "lon": -71.0589},
      "endPoint": {"lat": 42.3608, "lon": -71.0589},
      "heading": 0,
      "speed": 2.5
    }
  ]
}
```

---

## 4. Sensor Data

### Upload Sensor Data

```http
POST /sensors/data
Content-Type: application/json

{
  "robotId": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": "2026-01-12T10:30:00.123Z",
  "sensors": {
    "lidar": {
      "scanId": "scan-001",
      "obstacles": [...]
    },
    "camera": {
      "cropHealth": 87.5,
      "diseaseDetected": false
    },
    "soil": {
      "moisture": 35.2,
      "temperature": 18.7
    }
  }
}

Response 202:
{
  "dataId": "data-2026-001",
  "accepted": true,
  "processedAt": "2026-01-12T10:30:00.456Z"
}
```

### Query Sensor History

```http
GET /sensors/data?robotId={id}&start={iso8601}&end={iso8601}&type=lidar

Response 200:
{
  "total": 1247,
  "page": 1,
  "pageSize": 100,
  "data": [...]
}
```

---

## 5. Real-Time Control

### WebSocket Connection

```javascript
const ws = new WebSocket('wss://api.farming-robot.wia.org/v1/realtime');

ws.on('open', () => {
  ws.send(JSON.stringify({
    type: 'AUTH',
    token: 'Bearer <JWT_TOKEN>',
    robotId: '550e8400-e29b-41d4-a716-446655440000'
  }));
});

ws.on('message', (data) => {
  const message = JSON.parse(data);
  switch(message.type) {
    case 'TELEMETRY':
      // Real-time position, status
      break;
    case 'ALERT':
      // Safety alerts, obstacles
      break;
    case 'TASK_UPDATE':
      // Task progress updates
      break;
  }
});
```

### Send Control Command

```javascript
ws.send(JSON.stringify({
  type: 'CONTROL',
  command: 'EMERGENCY_STOP',
  robotId: '550e8400-e29b-41d4-a716-446655440000'
}));

// Response
{
  type: 'CONTROL_ACK',
  command: 'EMERGENCY_STOP',
  status: 'EXECUTED',
  timestamp: '2026-01-12T10:30:01.234Z'
}
```

---

## 6. Analytics & Reporting

### Get Field Analytics

```http
GET /analytics/fields/{fieldId}?period=30d

Response 200:
{
  "fieldId": "field-001",
  "period": "2025-12-13 to 2026-01-12",
  "operations": 12,
  "totalWorkingHours": 48.5,
  "areaCovered": 485.2,
  "yieldPrediction": {
    "crop": "corn",
    "estimatedYield": 12500,
    "confidence": 87.3
  },
  "efficiency": {
    "fuelEfficiency": 8.7,
    "costPerHectare": 45.2
  }
}
```

### Get Robot Performance

```http
GET /analytics/robots/{robotId}/performance?period=7d

Response 200:
{
  "robotId": "550e8400-e29b-41d4-a716-446655440000",
  "period": "Last 7 days",
  "metrics": {
    "operatingHours": 52.3,
    "tasksCompleted": 8,
    "avgEfficiency": 94.7,
    "downtime": 0.5,
    "maintenanceAlerts": 0
  }
}
```

---

## 7. Fleet Management

### List Fleet

```http
GET /fleet?farmId={farmId}

Response 200:
{
  "farmId": "farm-001",
  "robots": [
    {
      "robotId": "...",
      "type": "AUTONOMOUS_TRACTOR",
      "status": "WORKING",
      "currentTask": "task-001"
    },
    {
      "robotId": "...",
      "type": "HARVESTING_ROBOT",
      "status": "IDLE",
      "battery": 95.2
    }
  ]
}
```

### Coordinate Fleet Tasks

```http
POST /fleet/coordinate

{
  "farmId": "farm-001",
  "tasks": [
    {"robotId": "...", "taskType": "SEEDING", "fieldId": "field-001"},
    {"robotId": "...", "taskType": "SPRAYING", "fieldId": "field-002"}
  ],
  "optimization": "MINIMIZE_TIME"
}

Response 200:
{
  "coordinationId": "coord-001",
  "schedule": [...],
  "estimatedCompletion": "2026-01-12T18:00:00Z"
}
```

---

## Error Codes

| Code | Description |
|------|-------------|
| 4001 | Robot not found |
| 4002 | Invalid task parameters |
| 4003 | Robot offline |
| 4004 | Task conflict |
| 5001 | Path planning failed |
| 5002 | Sensor data corrupt |
| 5003 | Communication timeout |

---

## Rate Limits

| Endpoint Type | Limit |
|---------------|-------|
| REST API | 1000 req/hour per robot |
| WebSocket | 100 msg/sec per connection |
| Sensor Upload | 10 Hz maximum |

---

**弘益人間 (Benefit All Humanity)**
© 2026 WIA (World Industry Association)
