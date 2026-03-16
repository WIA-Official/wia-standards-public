# WIA-CITY-007: Construction Robot Standard
## PHASE 3 - PROTOCOL SPECIFICATION

**Version:** 1.0
**Status:** Draft
**Last Updated:** 2025-12-25
**Category:** CITY (Smart City & Urban Infrastructure)

---

## 1. Overview

Phase 3 defines communication protocols for construction robot control, monitoring, and coordination. These protocols ensure reliable, secure, and real-time communication between robots, operators, site management systems, and safety monitoring platforms.

### 1.1 Protocol Stack

```
┌─────────────────────────────────────────┐
│   Application Layer (Robot Commands)    │
├─────────────────────────────────────────┤
│   Security Layer (TLS 1.3 + DID Auth)  │
├─────────────────────────────────────────┤
│   Transport Layer (MQTT / WebSocket)    │
├─────────────────────────────────────────┤
│   Network Layer (IPv6 / 5G / WiFi-6)   │
└─────────────────────────────────────────┘
```

### 1.2 Design Principles

**Real-time:** Commands and telemetry with <100ms latency
**Reliable:** Guaranteed message delivery for critical commands
**Secure:** End-to-end encryption and authentication
**Scalable:** Support for 1000+ robots per construction site
**Resilient:** Graceful degradation and automatic reconnection

---

## 2. Robot Control Protocol

### 2.1 Command Message Format

All robot commands use this standardized format:

```json
{
  "version": "1.0",
  "messageType": "COMMAND",
  "messageId": "msg-2025-12-25-0001",
  "timestamp": "2025-12-25T10:30:00Z",

  "sender": {
    "id": "did:wia:operator:john-smith-742",
    "role": "robot-operator",
    "authorization": "jwt-token..."
  },

  "recipient": {
    "id": "did:wia:robot:CR-BRICK-001",
    "type": "bricklaying-robot"
  },

  "command": {
    "type": "START_TASK",
    "taskId": "wall-5A-section-3",
    "priority": "normal",
    "parameters": {
      "brickType": "red-clay-standard",
      "bondPattern": "running-bond",
      "mortarType": "type-n",
      "speed": "optimal"
    },
    "safetyChecks": {
      "areaCleared": true,
      "equipmentReady": true,
      "weatherAcceptable": true
    }
  },

  "signature": {
    "type": "Ed25519Signature2020",
    "created": "2025-12-25T10:30:00Z",
    "proofValue": "z5h8k2m..."
  }
}
```

### 2.2 Command Types

#### START_TASK
Begin a new construction task

**Parameters:**
- `taskId`: BIM element or work order ID
- `parameters`: Task-specific configuration
- `safetyChecks`: Pre-flight safety validation

**Response:** `TASK_STARTED` or `TASK_REJECTED`

#### STOP_TASK
Immediately stop current task

**Priority:** Emergency (highest)
**Response Time:** <200ms
**Response:** `TASK_STOPPED`

#### PAUSE_TASK
Temporarily pause current task

**Use Case:** Worker enters work area, material resupply
**Response:** `TASK_PAUSED`

#### RESUME_TASK
Resume paused task

**Validation:** Safety checks must pass again
**Response:** `TASK_RESUMED`

#### UPDATE_PARAMETERS
Modify task parameters during execution

**Parameters:**
- `speed`: Adjust operation speed
- `precision`: Change accuracy requirements
- `safetyRadius`: Update safety zone size

**Response:** `PARAMETERS_UPDATED`

#### REQUEST_STATUS
Query current robot status

**Response:** Full status report (see Phase 2)

#### EMERGENCY_STOP
Global emergency shutdown

**Scope:** All robots in safety zone
**Priority:** Critical (absolute highest)
**Response Time:** <100ms
**Response:** `EMERGENCY_STOPPED`

---

## 3. Telemetry Protocol

### 3.1 Real-time Telemetry Stream

Robots MUST publish telemetry data at configurable intervals:

**High-frequency (10 Hz):** Position, velocity, safety sensors
**Medium-frequency (1 Hz):** Status, battery, performance
**Low-frequency (0.1 Hz):** Health, statistics, summaries

```json
{
  "version": "1.0",
  "messageType": "TELEMETRY",
  "streamId": "telemetry-CR-BRICK-001",
  "timestamp": "2025-12-25T10:30:00.123Z",
  "sequenceNumber": 184523,

  "robot": {
    "id": "did:wia:robot:CR-BRICK-001",
    "state": "ACTIVE"
  },

  "position": {
    "coordinates": [126.9780, 37.5665, 15.0],
    "heading": 135,
    "velocity": [0.5, 0.3, 0.0]
  },

  "safety": {
    "nearestObstacle": 3.2,
    "workersInZone": 0,
    "emergencyStop": false,
    "safetyScore": 1.0
  },

  "performance": {
    "currentTask": "wall-5A-section-3",
    "progress": 0.67,
    "efficiency": 0.92
  },

  "power": {
    "batteryLevel": 85,
    "estimatedRuntime": 240
  }
}
```

### 3.2 Event-driven Messages

Robots MUST publish messages immediately when events occur:

**Safety Events:** Hazard detection, emergency stops
**Task Events:** Task start/stop/complete
**Health Events:** Errors, warnings, malfunctions
**Maintenance Events:** Service required, component failures

```json
{
  "version": "1.0",
  "messageType": "EVENT",
  "eventType": "SAFETY_HAZARD_DETECTED",
  "timestamp": "2025-12-25T10:30:15.456Z",
  "severity": "WARNING",

  "robot": {
    "id": "did:wia:robot:CR-BRICK-001"
  },

  "event": {
    "hazardType": "worker-proximity",
    "distance": 1.8,
    "threshold": 2.0,
    "action": "TASK_PAUSED"
  },

  "notification": {
    "targets": [
      "did:wia:operator:john-smith-742",
      "did:wia:supervisor:sarah-jones-123"
    ],
    "urgency": "high"
  }
}
```

---

## 4. Remote Monitoring Protocol

### 4.1 Dashboard Subscription

Site managers can subscribe to real-time updates from multiple robots:

**Request:**
```json
{
  "version": "1.0",
  "messageType": "SUBSCRIBE",
  "subscriptionId": "dashboard-001",

  "subscriber": {
    "id": "did:wia:manager:alice-chen-456",
    "authorization": "jwt-token..."
  },

  "filters": {
    "robots": ["CR-BRICK-*", "CR-3DP-*"],
    "eventTypes": ["SAFETY_HAZARD", "TASK_COMPLETE"],
    "updateInterval": 1.0
  }
}
```

**Response:**
```json
{
  "version": "1.0",
  "messageType": "SUBSCRIPTION_CONFIRMED",
  "subscriptionId": "dashboard-001",
  "streamUrl": "wss://monitoring.site.com/stream/dashboard-001"
}
```

### 4.2 Aggregated Metrics

Fleet-wide statistics for site management:

```json
{
  "version": "1.0",
  "messageType": "FLEET_METRICS",
  "timestamp": "2025-12-25T10:30:00Z",

  "site": {
    "id": "PROJ-2025-001",
    "name": "Smart City Tower"
  },

  "fleet": {
    "totalRobots": 14,
    "active": 12,
    "paused": 1,
    "charging": 1,
    "offline": 0
  },

  "productivity": {
    "tasksCompletedToday": 87,
    "averageEfficiency": 0.89,
    "totalWorkHours": 168
  },

  "safety": {
    "incidentsToday": 0,
    "hazardDetections": 15,
    "emergencyStops": 0,
    "safetyScore": 0.98
  },

  "resources": {
    "averageBatteryLevel": 72,
    "robotsNeedingMaintenance": 2,
    "materialsConsumed": {
      "bricks": 12400,
      "mortar": 620,
      "concrete": 8500
    }
  }
}
```

---

## 5. Multi-Robot Coordination

### 5.1 Task Allocation Protocol

Central coordinator assigns tasks to available robots:

**Request (from coordinator):**
```json
{
  "version": "1.0",
  "messageType": "TASK_ALLOCATION",
  "allocationId": "alloc-2025-12-25-0015",

  "coordinator": {
    "id": "did:wia:system:task-coordinator-1"
  },

  "task": {
    "taskId": "wall-5A-section-4",
    "type": "bricklaying",
    "priority": "normal",
    "estimatedDuration": 180,
    "requiredCapabilities": ["bricklaying", "mortar-application"]
  },

  "candidates": [
    "did:wia:robot:CR-BRICK-001",
    "did:wia:robot:CR-BRICK-002",
    "did:wia:robot:CR-BRICK-003"
  ]
}
```

**Response (from robots):**
```json
{
  "version": "1.0",
  "messageType": "TASK_BID",
  "allocationId": "alloc-2025-12-25-0015",

  "robot": {
    "id": "did:wia:robot:CR-BRICK-001"
  },

  "availability": {
    "ready": true,
    "currentTask": null,
    "batteryLevel": 85,
    "distance": 5.2,
    "estimatedStartTime": "2025-12-25T10:35:00Z"
  },

  "capability": {
    "efficiency": 0.92,
    "accuracy": 0.98,
    "experience": 2840
  }
}
```

### 5.2 Collision Avoidance

Robots coordinate to avoid workspace conflicts:

**Broadcasting:**
```json
{
  "version": "1.0",
  "messageType": "POSITION_BROADCAST",
  "timestamp": "2025-12-25T10:30:00.123Z",

  "robot": {
    "id": "did:wia:robot:CR-BRICK-001"
  },

  "currentPosition": {
    "coordinates": [126.9780, 37.5665, 15.0],
    "heading": 135,
    "velocity": [0.5, 0.3, 0.0]
  },

  "plannedPath": [
    {"coordinates": [126.9781, 37.5665, 15.0], "time": "+5s"},
    {"coordinates": [126.9782, 37.5665, 15.0], "time": "+10s"},
    {"coordinates": [126.9783, 37.5665, 15.0], "time": "+15s"}
  ],

  "reservedSpace": {
    "type": "cylinder",
    "center": [126.9780, 37.5665, 15.0],
    "radius": 2.0,
    "height": 3.0,
    "duration": 300
  }
}
```

**Conflict Detection:**
```json
{
  "version": "1.0",
  "messageType": "CONFLICT_DETECTED",
  "timestamp": "2025-12-25T10:30:00.456Z",

  "conflictingRobots": [
    "did:wia:robot:CR-BRICK-001",
    "did:wia:robot:CR-BRICK-002"
  ],

  "conflict": {
    "type": "workspace-overlap",
    "location": [126.9781, 37.5665, 15.0],
    "estimatedTime": "2025-12-25T10:30:10Z"
  },

  "resolution": {
    "priority": "CR-BRICK-001",
    "action": "CR-BRICK-002 delay by 30 seconds"
  }
}
```

---

## 6. Drone Coordination Protocol

### 6.1 Flight Plan Filing

Inspection drones file flight plans before takeoff:

```json
{
  "version": "1.0",
  "messageType": "FLIGHT_PLAN",
  "planId": "flight-2025-12-25-0042",

  "drone": {
    "id": "did:wia:robot:DRONE-INSP-127",
    "operator": "did:wia:operator:jane-doe-531"
  },

  "flight": {
    "departureTime": "2025-12-25T10:30:00Z",
    "estimatedDuration": 1800,
    "maxAltitude": 50,
    "flightArea": {
      "type": "Polygon",
      "coordinates": [
        [126.9770, 37.5660],
        [126.9790, 37.5660],
        [126.9790, 37.5670],
        [126.9770, 37.5670],
        [126.9770, 37.5660]
      ]
    }
  },

  "waypoints": [
    {"coordinates": [126.9780, 37.5665, 25], "action": "capture-photo"},
    {"coordinates": [126.9782, 37.5665, 30], "action": "capture-photo"},
    {"coordinates": [126.9784, 37.5665, 25], "action": "thermal-scan"}
  ],

  "safetyParameters": {
    "returnToHome": true,
    "minBattery": 25,
    "maxWindSpeed": 15,
    "obstacleAvoidance": "enabled"
  }
}
```

### 6.2 No-Fly Zone Management

Dynamic airspace restrictions for safety:

```json
{
  "version": "1.0",
  "messageType": "NO_FLY_ZONE",
  "zoneId": "nfz-2025-12-25-005",

  "zone": {
    "type": "cylinder",
    "center": [126.9780, 37.5665],
    "radius": 10,
    "floor": 0,
    "ceiling": 100
  },

  "validity": {
    "startTime": "2025-12-25T10:30:00Z",
    "endTime": "2025-12-25T11:30:00Z"
  },

  "reason": "crane-operation",
  "severity": "mandatory",

  "affectedDrones": [
    "did:wia:robot:DRONE-INSP-127",
    "did:wia:robot:DRONE-INSP-128"
  ],

  "notification": {
    "method": "immediate-broadcast",
    "acknowledgmentRequired": true
  }
}
```

---

## 7. BIM Synchronization Protocol

### 7.1 Progress Update

Robots report work completion to BIM system:

```json
{
  "version": "1.0",
  "messageType": "BIM_PROGRESS_UPDATE",
  "timestamp": "2025-12-25T11:45:00Z",

  "robot": {
    "id": "did:wia:robot:CR-BRICK-001"
  },

  "project": {
    "id": "PROJ-2025-001",
    "bimModel": "smart-city-tower.rvt"
  },

  "elements": [
    {
      "bimElementId": "wall-5A-3",
      "status": "COMPLETED",
      "completionTime": "2025-12-25T11:45:00Z",
      "asBuilt": {
        "dimensions": {"length": 6.002, "height": 2.398, "thickness": 0.201},
        "deviation": 0.003,
        "qualityScore": 0.98
      },
      "documentation": {
        "photos": ["s3://asbuilt/wall-5A-3-view1.jpg"],
        "3dScan": "s3://asbuilt/wall-5A-3-scan.ply"
      }
    }
  ],

  "verification": {
    "inspector": "did:wia:inspector:jane-doe-531",
    "signature": "0x7d8a9b..."
  }
}
```

---

## 8. Security Protocols

### 8.1 Authentication

All communication MUST be authenticated using DIDs:

**Authentication Flow:**
1. Robot presents DID and signs challenge
2. System verifies DID signature
3. JWT token issued for session
4. Token included in all subsequent messages

**Challenge-Response:**
```json
{
  "version": "1.0",
  "messageType": "AUTH_CHALLENGE",
  "challenge": "random-nonce-7d8a9b...",
  "timestamp": "2025-12-25T10:30:00Z"
}
```

**Response:**
```json
{
  "version": "1.0",
  "messageType": "AUTH_RESPONSE",
  "did": "did:wia:robot:CR-BRICK-001",
  "signature": "z5h8k2m...",
  "publicKey": "z6Mk..."
}
```

### 8.2 Encryption

**Transport Encryption:** TLS 1.3 for all connections
**Message Encryption:** Optional AES-256-GCM for sensitive data
**Key Management:** DID-based key rotation every 90 days

### 8.3 Authorization

**Role-Based Access Control (RBAC):**
- **Robot Operator:** Send commands, view telemetry
- **Site Manager:** View all robots, override commands
- **Safety Officer:** Emergency stop authority
- **Inspector:** View records, approve quality
- **Maintenance:** Service access, diagnostics

---

## 9. Network Requirements

### 9.1 Connectivity

**Primary:** 5G (preferred), WiFi-6 (fallback)
**Latency:** <100ms for commands, <500ms for telemetry
**Bandwidth:** 1 Mbps per robot (telemetry), 10 Mbps per drone (video)
**Reliability:** 99.9% uptime required

### 9.2 Edge Computing

Local processing for low-latency decisions:

**Edge Node:** On-site server for local coordination
**Capabilities:** Path planning, collision avoidance, local logging
**Fallback:** Robots continue autonomous operation if cloud disconnected

---

## 10. Protocol Implementation

### 10.1 MQTT Topics

```
wia/site/{site-id}/robot/{robot-id}/command
wia/site/{site-id}/robot/{robot-id}/telemetry
wia/site/{site-id}/robot/{robot-id}/events
wia/site/{site-id}/fleet/metrics
wia/site/{site-id}/safety/alerts
```

### 10.2 WebSocket Endpoints

```
wss://api.construction-site.com/robots/{robot-id}/control
wss://api.construction-site.com/robots/{robot-id}/stream
wss://api.construction-site.com/fleet/monitor
```

---

## 11. Next Steps

**Phase 4:** System integration specifications for complete construction automation ecosystem

---

**© 2025 SmileStory Inc. / WIA**
**弘益人間 (홍익인간) · Benefit All Humanity**
