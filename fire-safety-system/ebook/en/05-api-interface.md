# Chapter 5: API Interface Design (Phase 2)

## Overview

Phase 2 of the WIA Standard defines comprehensive API specifications enabling software applications, building management systems, and third-party services to interact with fire safety infrastructure uniformly. This chapter details RESTful API design, WebSocket event streaming, authentication mechanisms, and implementation best practices.

---

## API Architecture

### RESTful API Design Principles

The WIA Standard adopts RESTful (Representational State Transfer) architecture for its proven effectiveness and widespread adoption:

```
REST Principles Applied:

1. Resource-Based URLs
   ✓ /api/v1/devices/{id}
   ✗ /api/v1/getDevice?id=123

2. Standard HTTP Methods
   ✓ GET, POST, PUT, DELETE
   ✗ Custom verbs in URL

3. Stateless Communication
   ✓ Each request contains all needed information
   ✗ Server-side session state

4. JSON Representations
   ✓ Content-Type: application/json
   ✗ Custom binary formats

5. HATEOAS (optional)
   ✓ Links to related resources
```

### API Base URL Structure

```
Format: https://{host}:{port}/api/{version}/

Examples:
https://panel.building.com:8443/api/v1/
https://192.168.1.100:443/api/v1/
https://fire-safety-api.example.com/api/v1/

Components:
- Protocol: HTTPS (TLS 1.3 required)
- Host: Panel IP or domain name
- Port: 443 (standard HTTPS) or custom
- API prefix: /api/
- Version: /v1/ (current version)
```

---

## Device Management API

### List All Devices

**Endpoint:** `GET /api/v1/devices`

**Description:** Retrieve list of all devices in the system

**Query Parameters:**
```
type       : Filter by device type (smoke, heat, flame, co, multi)
status     : Filter by status (normal, alarm, trouble, disabled)
building   : Filter by building identifier
floor      : Filter by floor number
zone       : Filter by zone identifier
limit      : Number of results per page (default: 100, max: 1000)
offset     : Pagination offset (default: 0)
```

**Request Example:**
```http
GET /api/v1/devices?type=smoke&status=normal&limit=50 HTTP/1.1
Host: panel.building.com
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
Accept: application/json
```

**Response (200 OK):**
```json
{
  "total": 237,
  "limit": 50,
  "offset": 0,
  "devices": [
    {
      "sensorId": "550e8400-e29b-41d4-a716-446655440000",
      "type": "smoke",
      "location": {
        "building": "Main Tower",
        "floor": 12,
        "zone": "East Wing E12-A"
      },
      "status": "normal",
      "readings": {
        "value": 0.8,
        "unit": "OD/meter",
        "timestamp": "2025-12-27T14:32:15Z"
      },
      "metadata": {
        "manufacturer": "SafetyTech Inc",
        "model": "ST-5000",
        "firmwareVersion": "2.4.1"
      }
    },
    {
      "sensorId": "7c9e6679-7425-40de-944b-e07fc1f90ae7",
      "type": "smoke",
      "location": {
        "building": "Main Tower",
        "floor": 11,
        "zone": "West Wing W11-B"
      },
      "status": "normal",
      "readings": {
        "value": 1.2,
        "unit": "OD/meter",
        "timestamp": "2025-12-27T14:32:18Z"
      },
      "metadata": {
        "manufacturer": "SafetyTech Inc",
        "model": "ST-5000",
        "firmwareVersion": "2.4.1"
      }
    }
  ]
}
```

### Get Device Details

**Endpoint:** `GET /api/v1/devices/{id}`

**Description:** Retrieve detailed information for a specific device

**Path Parameters:**
```
id : Device UUID
```

**Request Example:**
```http
GET /api/v1/devices/550e8400-e29b-41d4-a716-446655440000 HTTP/1.1
Host: panel.building.com
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
Accept: application/json
```

**Response (200 OK):**
```json
{
  "sensorId": "550e8400-e29b-41d4-a716-446655440000",
  "type": "smoke",
  "location": {
    "building": "Main Tower",
    "floor": 12,
    "zone": "East Wing E12-A",
    "coordinates": {
      "x": 45.2,
      "y": 23.8,
      "z": 3.5
    }
  },
  "status": "normal",
  "readings": {
    "value": 0.8,
    "unit": "OD/meter",
    "timestamp": "2025-12-27T14:32:15Z"
  },
  "metadata": {
    "manufacturer": "SafetyTech Inc",
    "model": "ST-5000",
    "firmwareVersion": "2.4.1",
    "installationDate": "2024-06-15T10:00:00Z",
    "lastMaintenance": "2025-10-12T09:30:00Z",
    "serialNumber": "ST5K-2024-061567",
    "certifications": ["UL-268", "FM-3260", "EN-54-7"]
  },
  "history": {
    "totalAlarms": 3,
    "lastAlarm": "2025-11-08T09:15:30Z",
    "totalTroubles": 1,
    "lastTrouble": "2025-09-22T14:42:10Z",
    "uptime": 99.97
  }
}
```

**Response (404 Not Found):**
```json
{
  "error": "DeviceNotFound",
  "message": "Device with ID 550e8400-e29b-41d4-a716-446655440000 not found",
  "timestamp": "2025-12-27T14:35:20Z"
}
```

### Update Device Configuration

**Endpoint:** `PUT /api/v1/devices/{id}`

**Description:** Update device configuration parameters

**Required Role:** Technician or Administrator

**Request Example:**
```http
PUT /api/v1/devices/550e8400-e29b-41d4-a716-446655440000 HTTP/1.1
Host: panel.building.com
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
Content-Type: application/json

{
  "location": {
    "building": "Main Tower",
    "floor": 12,
    "zone": "East Wing E12-B"
  },
  "sensitivity": "high",
  "alarmThreshold": 3.5
}
```

**Response (200 OK):**
```json
{
  "message": "Device configuration updated successfully",
  "deviceId": "550e8400-e29b-41d4-a716-446655440000",
  "updatedFields": ["location.zone", "sensitivity", "alarmThreshold"],
  "timestamp": "2025-12-27T14:40:15Z"
}
```

### Test Device

**Endpoint:** `POST /api/v1/devices/{id}/test`

**Description:** Initiate device self-test

**Required Role:** Technician or Administrator

**Request Example:**
```http
POST /api/v1/devices/550e8400-e29b-41d4-a716-446655440000/test HTTP/1.1
Host: panel.building.com
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
Content-Type: application/json

{
  "testType": "functional",
  "silenceNotifications": true
}
```

**Response (202 Accepted):**
```json
{
  "message": "Device test initiated",
  "deviceId": "550e8400-e29b-41d4-a716-446655440000",
  "testId": "test-9a8b7c6d-5e4f-3a2b-1c0d-9e8f7a6b5c4d",
  "estimatedDuration": 30,
  "status": "in_progress",
  "timestamp": "2025-12-27T14:45:00Z"
}
```

### Remove Device

**Endpoint:** `DELETE /api/v1/devices/{id}`

**Description:** Remove device from system

**Required Role:** Administrator

**Request Example:**
```http
DELETE /api/v1/devices/550e8400-e29b-41d4-a716-446655440000 HTTP/1.1
Host: panel.building.com
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

**Response (200 OK):**
```json
{
  "message": "Device removed successfully",
  "deviceId": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": "2025-12-27T14:50:00Z"
}
```

---

## Alarm Management API

### List Active Alarms

**Endpoint:** `GET /api/v1/alarms`

**Description:** Retrieve list of alarm events

**Query Parameters:**
```
status     : Filter by status (active, acknowledged, resolved)
type       : Filter by type (fire, supervisory, trouble, test)
priority   : Filter by priority (critical, high, medium, low)
building   : Filter by building
floor      : Filter by floor
startDate  : Filter events after this timestamp (ISO 8601)
endDate    : Filter events before this timestamp (ISO 8601)
limit      : Results per page (default: 100, max: 1000)
offset     : Pagination offset
```

**Request Example:**
```http
GET /api/v1/alarms?status=active&priority=critical HTTP/1.1
Host: panel.building.com
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
Accept: application/json
```

**Response (200 OK):**
```json
{
  "total": 2,
  "limit": 100,
  "offset": 0,
  "alarms": [
    {
      "eventId": "9f4e2c8a-5d3b-4a7e-9c1f-8e2d4a6c3b5f",
      "eventType": "fire",
      "priority": "critical",
      "source": {
        "deviceId": "550e8400-e29b-41d4-a716-446655440000",
        "deviceType": "smoke_detector",
        "location": {
          "building": "Main Tower",
          "floor": 12,
          "zone": "East Wing E12-A"
        }
      },
      "timestamp": "2025-12-27T14:32:15Z",
      "description": "Smoke detected in East Wing Zone E12-A, Floor 12",
      "status": "active",
      "duration": 180
    },
    {
      "eventId": "1a2b3c4d-5e6f-4a5b-8c9d-0e1f2a3b4c5d",
      "eventType": "fire",
      "priority": "critical",
      "source": {
        "deviceId": "7c9e6679-7425-40de-944b-e07fc1f90ae7",
        "deviceType": "heat_detector",
        "location": {
          "building": "West Annex",
          "floor": 3,
          "zone": "Storage Room W3-S2"
        }
      },
      "timestamp": "2025-12-27T14:35:42Z",
      "description": "High temperature detected in Storage Room",
      "status": "active",
      "duration": 47
    }
  ]
}
```

### Get Alarm Details

**Endpoint:** `GET /api/v1/alarms/{id}`

**Description:** Retrieve detailed information for a specific alarm

**Response (200 OK):**
```json
{
  "eventId": "9f4e2c8a-5d3b-4a7e-9c1f-8e2d4a6c3b5f",
  "eventType": "fire",
  "priority": "critical",
  "source": {
    "deviceId": "550e8400-e29b-41d4-a716-446655440000",
    "deviceType": "smoke_detector",
    "location": {
      "building": "Main Tower",
      "floor": 12,
      "zone": "East Wing E12-A"
    }
  },
  "timestamp": "2025-12-27T14:32:15Z",
  "description": "Smoke detected in East Wing Zone E12-A, Floor 12",
  "notifications": [
    "panel",
    "monitoring-center",
    "emergency-services",
    "building-management"
  ],
  "status": "active",
  "actions": [
    {
      "type": "notification_sent",
      "target": "panel",
      "timestamp": "2025-12-27T14:32:16Z"
    },
    {
      "type": "notification_sent",
      "target": "monitoring-center",
      "timestamp": "2025-12-27T14:32:17Z"
    },
    {
      "type": "emergency_services_notified",
      "timestamp": "2025-12-27T14:32:18Z"
    }
  ],
  "relatedDevices": [
    "550e8400-e29b-41d4-a716-446655440000",
    "660f9511-f3ac-52e5-b827-557766551111"
  ]
}
```

### Acknowledge Alarm

**Endpoint:** `POST /api/v1/alarms/{id}/acknowledge`

**Description:** Acknowledge an active alarm

**Required Role:** Operator, Technician, or Administrator

**Request Example:**
```http
POST /api/v1/alarms/9f4e2c8a-5d3b-4a7e-9c1f-8e2d4a6c3b5f/acknowledge HTTP/1.1
Host: panel.building.com
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
Content-Type: application/json

{
  "notes": "Fire department en route. Evacuating floor 12."
}
```

**Response (200 OK):**
```json
{
  "message": "Alarm acknowledged successfully",
  "eventId": "9f4e2c8a-5d3b-4a7e-9c1f-8e2d4a6c3b5f",
  "acknowledgedBy": "operator-john-smith",
  "acknowledgedAt": "2025-12-27T14:35:20Z",
  "timestamp": "2025-12-27T14:35:20Z"
}
```

### Silence Alarm Notifications

**Endpoint:** `POST /api/v1/alarms/{id}/silence`

**Description:** Silence audible/visual notifications while maintaining alarm state

**Required Role:** Operator, Technician, or Administrator

**Request Example:**
```http
POST /api/v1/alarms/9f4e2c8a-5d3b-4a7e-9c1f-8e2d4a6c3b5f/silence HTTP/1.1
Host: panel.building.com
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
Content-Type: application/json

{
  "duration": 300,
  "reason": "Fire department on-site conducting investigation"
}
```

**Response (200 OK):**
```json
{
  "message": "Alarm notifications silenced",
  "eventId": "9f4e2c8a-5d3b-4a7e-9c1f-8e2d4a6c3b5f",
  "silencedBy": "operator-john-smith",
  "silencedAt": "2025-12-27T14:40:00Z",
  "silenceDuration": 300,
  "autoResumeAt": "2025-12-27T14:45:00Z"
}
```

---

## System Status API

### Get Overall System Health

**Endpoint:** `GET /api/v1/status`

**Description:** Retrieve overall system health and statistics

**Response (200 OK):**
```json
{
  "systemStatus": "operational",
  "timestamp": "2025-12-27T14:50:00Z",
  "uptime": 2592000,
  "statistics": {
    "totalDevices": 2847,
    "devicesNormal": 2840,
    "devicesAlarm": 2,
    "devicesTrouble": 5,
    "devicesDisabled": 0,
    "activeAlarms": 2,
    "acknowledgedAlarms": 3,
    "resolvedAlarmsToday": 12
  },
  "panels": {
    "total": 8,
    "online": 8,
    "offline": 0
  },
  "network": {
    "status": "healthy",
    "latency": 12,
    "packetLoss": 0.02
  },
  "power": {
    "status": "ac_power",
    "batteryCharge": 100,
    "batteryHealth": "good"
  }
}
```

### Get Control Panel Status

**Endpoint:** `GET /api/v1/status/panels`

**Description:** Retrieve status of all control panels

**Response (200 OK):**
```json
{
  "panels": [
    {
      "panelId": "panel-001",
      "name": "Main Tower North",
      "status": "operational",
      "location": {
        "building": "Main Tower",
        "floor": 1,
        "room": "Security Office"
      },
      "connectedDevices": 856,
      "activeAlarms": 1,
      "uptime": 2592000,
      "firmwareVersion": "4.2.1",
      "lastCommunication": "2025-12-27T14:50:00Z"
    },
    {
      "panelId": "panel-002",
      "name": "Main Tower South",
      "status": "operational",
      "location": {
        "building": "Main Tower",
        "floor": 1,
        "room": "Electrical Room"
      },
      "connectedDevices": 743,
      "activeAlarms": 0,
      "uptime": 2592000,
      "firmwareVersion": "4.2.1",
      "lastCommunication": "2025-12-27T14:49:58Z"
    }
  ]
}
```

---

## Authentication & Authorization

### JWT Authentication

**Token Request:**
```http
POST /api/v1/auth/login HTTP/1.1
Host: panel.building.com
Content-Type: application/json

{
  "username": "john.smith",
  "password": "SecurePassword123!",
  "mfaCode": "123456"
}
```

**Token Response:**
```json
{
  "accessToken": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJzdWIiOiJqb2huLnNtaXRoIiwicm9sZSI6Im9wZXJhdG9yIiwiaWF0IjoxNzM1MzE1MjAwLCJleHAiOjE3MzUzMTg4MDB9.signature",
  "refreshToken": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "expiresIn": 3600,
  "tokenType": "Bearer"
}
```

**Using Token:**
```http
GET /api/v1/devices HTTP/1.1
Host: panel.building.com
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

### Role-Based Access Control

```
Role Hierarchy and Permissions:

┌─────────────────────────────────────────────┐
│ Administrator (Full Access)                 │
│ ✓ All Technician permissions               │
│ ✓ User management                           │
│ ✓ System configuration                      │
│ ✓ Device add/remove                         │
│ ✓ Audit log access                          │
├─────────────────────────────────────────────┤
│ Technician                                  │
│ ✓ All Operator permissions                 │
│ ✓ Device configuration                      │
│ ✓ Device testing                            │
│ ✓ Maintenance operations                    │
├─────────────────────────────────────────────┤
│ Operator                                    │
│ ✓ All Viewer permissions                   │
│ ✓ Acknowledge alarms                        │
│ ✓ Silence notifications                     │
│ ✓ Add notes to events                       │
├─────────────────────────────────────────────┤
│ Viewer (Read-Only)                          │
│ ✓ View device status                        │
│ ✓ View alarm events                         │
│ ✓ View system status                        │
│ ✓ Download reports                          │
└─────────────────────────────────────────────┘
```

---

## WebSocket Real-Time Events

### Connection Establishment

```javascript
// Connect to event stream
const ws = new WebSocket('wss://panel.building.com/api/v1/events');

// Include authentication
ws.onopen = () => {
  ws.send(JSON.stringify({
    type: 'authenticate',
    token: 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...'
  }));
};

// Handle authentication response
ws.onmessage = (event) => {
  const data = JSON.parse(event.data);

  if (data.type === 'authenticated') {
    console.log('Connected to event stream');
  }
};
```

### Event Types

**Alarm Triggered:**
```json
{
  "eventType": "alarm.triggered",
  "timestamp": "2025-12-27T14:32:15Z",
  "data": {
    "eventId": "9f4e2c8a-5d3b-4a7e-9c1f-8e2d4a6c3b5f",
    "alarmType": "fire",
    "priority": "critical",
    "source": {
      "deviceId": "550e8400-e29b-41d4-a716-446655440000",
      "location": {
        "building": "Main Tower",
        "floor": 12,
        "zone": "East Wing E12-A"
      }
    }
  }
}
```

**Device Status Changed:**
```json
{
  "eventType": "device.status.changed",
  "timestamp": "2025-12-27T14:38:20Z",
  "data": {
    "deviceId": "a1b2c3d4-e5f6-4a5b-8c9d-0e1f2a3b4c5d",
    "previousStatus": "normal",
    "newStatus": "trouble",
    "reason": "Communication failure"
  }
}
```

---

## Error Handling

### Standard Error Response Format

```json
{
  "error": "ErrorCode",
  "message": "Human-readable error description",
  "details": {
    "field": "specificField",
    "reason": "Detailed explanation"
  },
  "timestamp": "2025-12-27T14:50:00Z",
  "requestId": "req-1234567890"
}
```

### Common HTTP Status Codes

```
200 OK                  - Successful request
201 Created             - Resource created successfully
202 Accepted            - Request accepted, processing async
204 No Content          - Successful, no response body
400 Bad Request         - Invalid request format
401 Unauthorized        - Authentication required/failed
403 Forbidden           - Insufficient permissions
404 Not Found           - Resource not found
409 Conflict            - Request conflicts with current state
429 Too Many Requests   - Rate limit exceeded
500 Internal Server Error - Server error
503 Service Unavailable - Temporary service disruption
```

---

## Key Takeaways

1. **RESTful API design** provides universal, predictable interfaces for all fire safety systems.

2. **Device and alarm management** endpoints enable complete system control and monitoring.

3. **Role-based access control** ensures security through appropriate permission levels.

4. **WebSocket events** provide real-time updates with minimal latency.

5. **Standard error handling** facilitates robust client implementations.

---

## Next Steps

Chapter 6 explores Phase 3, defining the communication protocols that ensure reliable, secure, and timely operation of fire safety systems.

---

© 2025 WIA Standards Committee. 弘익人間 (홍익인간) - Benefit All Humanity
