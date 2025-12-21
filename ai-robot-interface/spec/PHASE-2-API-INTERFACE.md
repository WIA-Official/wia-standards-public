# WIA AI Robot Interface - Phase 2: API Interface Standard

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Primary Color**: #6366F1 (Indigo)

---

## Table of Contents

1. [Overview](#overview)
2. [REST API Specification](#rest-api-specification)
3. [Robot Discovery API](#robot-discovery-api)
4. [Command Dispatch API](#command-dispatch-api)
5. [State Monitoring API](#state-monitoring-api)
6. [Sensor Data API](#sensor-data-api)
7. [Multi-Robot Coordination API](#multi-robot-coordination-api)
8. [WebSocket Real-Time Interface](#websocket-real-time-interface)
9. [Authentication & Authorization](#authentication--authorization)
10. [SDK Reference](#sdk-reference)

---

## Overview

### 1.1 Purpose

Phase 2 defines the REST API and WebSocket interfaces for interacting with robots conforming to the WIA AI Robot Interface standard. These APIs enable AI systems to discover, command, monitor, and coordinate robots in real-time.

**API Design Principles**:
- RESTful resource-oriented design
- Real-time WebSocket for streaming data
- Stateless HTTP/2 for commands
- JWT-based authentication
- Rate limiting and throttling
- Comprehensive error handling

### 1.2 Architecture Overview

```mermaid
graph TB
    A[AI System] -->|HTTPS/WSS| B[API Gateway]
    B -->|Auth| C[Auth Service]
    B -->|Discovery| D[Robot Registry]
    B -->|Commands| E[Command Dispatcher]
    B -->|State| F[State Service]
    B -->|Sensors| G[Sensor Service]
    E -->|WebSocket/MQTT| H[Robot 1]
    E -->|WebSocket/MQTT| I[Robot 2]
    E -->|WebSocket/MQTT| J[Robot N]
    style B fill:#6366F1,stroke:#4F46E5,color:#fff
```

### 1.3 Base URL Structure

```
Production:  https://api.wia-robot.org/v1
Staging:     https://api-staging.wia-robot.org/v1
Development: http://localhost:8080/v1
```

### 1.4 Protocol Support

| Protocol | Use Case | Port |
|----------|----------|------|
| HTTPS (HTTP/2) | REST API | 443 |
| WSS (WebSocket Secure) | Real-time state/sensor streams | 443 |
| MQTT over TLS | Pub/Sub coordination | 8883 |
| gRPC | High-performance RPC | 50051 |

---

## REST API Specification

### 2.1 Standard Response Format

All API responses follow this envelope:

```json
{
  "status": "success",
  "timestamp": "2025-01-15T14:30:00.123Z",
  "data": {
    // Response payload
  },
  "metadata": {
    "request_id": "req-8f7e6d5c-4b3a-2918-7654-3210fedcba98",
    "version": "1.0.0"
  }
}
```

### 2.2 Error Response Format

```json
{
  "status": "error",
  "timestamp": "2025-01-15T14:30:00.123Z",
  "error": {
    "code": "ROBOT_NOT_FOUND",
    "message": "Robot with ID 'mobile-manipulator-001' not found",
    "details": {
      "robot_id": "mobile-manipulator-001"
    }
  },
  "metadata": {
    "request_id": "req-12345678",
    "version": "1.0.0"
  }
}
```

### 2.3 HTTP Status Codes

| Code | Meaning | Use Case |
|------|---------|----------|
| 200 | OK | Successful GET/PUT/PATCH |
| 201 | Created | Successful POST |
| 202 | Accepted | Async operation started |
| 204 | No Content | Successful DELETE |
| 400 | Bad Request | Invalid request data |
| 401 | Unauthorized | Missing/invalid auth |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource doesn't exist |
| 409 | Conflict | Resource state conflict |
| 422 | Unprocessable Entity | Validation failed |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server-side error |
| 503 | Service Unavailable | Robot offline/busy |

### 2.4 Common Headers

```http
# Request Headers
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
Content-Type: application/json
Accept: application/json
X-Request-ID: req-12345678
X-Robot-ID: mobile-manipulator-001

# Response Headers
Content-Type: application/json
X-Request-ID: req-12345678
X-Rate-Limit-Limit: 100
X-Rate-Limit-Remaining: 95
X-Rate-Limit-Reset: 1736951460
```

---

## Robot Discovery API

### 3.1 List All Robots

**Endpoint**: `GET /robots`

**Query Parameters**:
- `type`: Filter by robot type (e.g., `mobile_manipulator`)
- `status`: Filter by status (e.g., `operational`)
- `capability`: Filter by capability (e.g., `navigation`)
- `limit`: Max results (default: 50, max: 200)
- `offset`: Pagination offset

**Example Request**:

```bash
curl -X GET "https://api.wia-robot.org/v1/robots?type=mobile_manipulator&status=operational&limit=10" \
  -H "Authorization: Bearer ${TOKEN}"
```

**Example Response**:

```json
{
  "status": "success",
  "timestamp": "2025-01-15T14:30:00.123Z",
  "data": {
    "robots": [
      {
        "robot_id": "mobile-manipulator-001",
        "robot_name": "Fetch Research Robot",
        "robot_type": "mobile_manipulator",
        "status": "operational",
        "last_seen": "2025-01-15T14:29:55Z",
        "capabilities_summary": ["navigation", "manipulation", "perception"],
        "api_endpoint": "https://api.wia-robot.org/v1/robots/mobile-manipulator-001"
      },
      {
        "robot_id": "mobile-manipulator-002",
        "robot_name": "PR2 Lab Robot",
        "robot_type": "mobile_manipulator",
        "status": "operational",
        "last_seen": "2025-01-15T14:29:58Z",
        "capabilities_summary": ["navigation", "manipulation", "perception"],
        "api_endpoint": "https://api.wia-robot.org/v1/robots/mobile-manipulator-002"
      }
    ],
    "pagination": {
      "total": 25,
      "limit": 10,
      "offset": 0,
      "has_more": true
    }
  },
  "metadata": {
    "request_id": "req-12345678",
    "version": "1.0.0"
  }
}
```

### 3.2 Get Robot Details

**Endpoint**: `GET /robots/{robot_id}`

**Example Request**:

```bash
curl -X GET "https://api.wia-robot.org/v1/robots/mobile-manipulator-001" \
  -H "Authorization: Bearer ${TOKEN}"
```

**Example Response**:

```json
{
  "status": "success",
  "data": {
    "robot_id": "mobile-manipulator-001",
    "robot_name": "Fetch Research Robot",
    "robot_type": "mobile_manipulator",
    "manufacturer": "Fetch Robotics",
    "model": "Fetch",
    "firmware_version": "3.2.1",
    "status": "operational",
    "capabilities": {
      "mobility": {
        "type": "differential_drive",
        "max_linear_velocity": 1.0,
        "max_angular_velocity": 2.0
      },
      "manipulation": {
        "type": "serial_manipulator",
        "dof": 7,
        "payload_kg": 5.0
      }
    },
    "network": {
      "ip_address": "192.168.1.100",
      "websocket_url": "wss://api.wia-robot.org/v1/robots/mobile-manipulator-001/ws"
    }
  }
}
```

### 3.3 Register New Robot

**Endpoint**: `POST /robots`

**Request Body**:

```json
{
  "robot_descriptor": {
    "robot_id": "mobile-manipulator-003",
    "robot_name": "HSR Lab Robot",
    "robot_type": "mobile_manipulator",
    "manufacturer": "Toyota",
    "model": "HSR",
    "capabilities": {
      "mobility": {
        "type": "omnidirectional",
        "max_linear_velocity": 0.8
      }
    }
  }
}
```

**Example Response** (201 Created):

```json
{
  "status": "success",
  "data": {
    "robot_id": "mobile-manipulator-003",
    "registered_at": "2025-01-15T14:30:00Z",
    "api_key": "wia_sk_live_abc123...",
    "websocket_url": "wss://api.wia-robot.org/v1/robots/mobile-manipulator-003/ws"
  }
}
```

### 3.4 Update Robot Status

**Endpoint**: `PATCH /robots/{robot_id}`

```json
{
  "status": "maintenance",
  "accepting_tasks": false
}
```

### 3.5 Deregister Robot

**Endpoint**: `DELETE /robots/{robot_id}`

**Response**: 204 No Content

---

## Command Dispatch API

### 4.1 Send Command

**Endpoint**: `POST /robots/{robot_id}/commands`

**Request Body**:

```json
{
  "command_type": "navigation",
  "priority": "normal",
  "timeout_ms": 30000,
  "payload": {
    "goal": {
      "position": {"x": 5.0, "y": 3.0, "z": 0.0},
      "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
    },
    "frame_id": "map",
    "tolerance": {"position": 0.1, "orientation": 0.05}
  }
}
```

**Example Response** (202 Accepted):

```json
{
  "status": "success",
  "data": {
    "command_id": "cmd-8f7e6d5c-4b3a-2918",
    "robot_id": "mobile-manipulator-001",
    "status": "accepted",
    "accepted_at": "2025-01-15T14:30:00.200Z",
    "estimated_completion": "2025-01-15T14:30:30Z",
    "status_url": "/robots/mobile-manipulator-001/commands/cmd-8f7e6d5c-4b3a-2918"
  }
}
```

### 4.2 Get Command Status

**Endpoint**: `GET /robots/{robot_id}/commands/{command_id}`

**Example Response**:

```json
{
  "status": "success",
  "data": {
    "command_id": "cmd-8f7e6d5c-4b3a-2918",
    "robot_id": "mobile-manipulator-001",
    "command_type": "navigation",
    "status": "executing",
    "progress": 0.65,
    "started_at": "2025-01-15T14:30:00.250Z",
    "updated_at": "2025-01-15T14:30:15.100Z",
    "estimated_completion": "2025-01-15T14:30:25Z"
  }
}
```

### 4.3 Cancel Command

**Endpoint**: `DELETE /robots/{robot_id}/commands/{command_id}`

**Response** (200 OK):

```json
{
  "status": "success",
  "data": {
    "command_id": "cmd-8f7e6d5c-4b3a-2918",
    "status": "cancelled",
    "cancelled_at": "2025-01-15T14:30:20Z"
  }
}
```

### 4.4 Batch Commands

**Endpoint**: `POST /robots/{robot_id}/commands/batch`

```json
{
  "commands": [
    {
      "command_type": "navigation",
      "payload": { /* navigation payload */ }
    },
    {
      "command_type": "manipulation",
      "payload": { /* manipulation payload */ }
    }
  ],
  "execution": "sequential"
}
```

### 4.5 Emergency Stop

**Endpoint**: `POST /robots/{robot_id}/emergency_stop`

**Response** (200 OK):

```json
{
  "status": "success",
  "data": {
    "robot_id": "mobile-manipulator-001",
    "emergency_stop_activated": true,
    "timestamp": "2025-01-15T14:30:00.050Z",
    "all_commands_cancelled": true
  }
}
```

---

## State Monitoring API

### 5.1 Get Current State

**Endpoint**: `GET /robots/{robot_id}/state`

**Example Response**:

```json
{
  "status": "success",
  "data": {
    "robot_id": "mobile-manipulator-001",
    "timestamp": "2025-01-15T14:30:00.500Z",
    "operational_state": "executing",
    "kinematic_state": {
      "base_pose": {
        "position": {"x": 5.2, "y": 3.1, "z": 0.0},
        "orientation": {"w": 0.998, "x": 0.0, "y": 0.0, "z": 0.062}
      }
    },
    "battery": {
      "percentage": 78.5,
      "time_remaining_minutes": 142
    },
    "current_task_id": "task-123"
  }
}
```

### 5.2 Get State History

**Endpoint**: `GET /robots/{robot_id}/state/history`

**Query Parameters**:
- `start_time`: ISO 8601 timestamp
- `end_time`: ISO 8601 timestamp
- `fields`: Comma-separated state fields (e.g., `battery,position`)
- `interval`: Sampling interval in seconds

```bash
curl "https://api.wia-robot.org/v1/robots/mobile-manipulator-001/state/history?start_time=2025-01-15T14:00:00Z&end_time=2025-01-15T14:30:00Z&fields=battery&interval=60"
```

### 5.3 Subscribe to State Updates

**Endpoint**: `GET /robots/{robot_id}/state/stream` (Server-Sent Events)

```bash
curl -N -H "Authorization: Bearer ${TOKEN}" \
  "https://api.wia-robot.org/v1/robots/mobile-manipulator-001/state/stream"
```

**Response Stream**:

```
data: {"timestamp": "2025-01-15T14:30:00Z", "battery": 78.5, "position": {...}}

data: {"timestamp": "2025-01-15T14:30:01Z", "battery": 78.5, "position": {...}}

data: {"timestamp": "2025-01-15T14:30:02Z", "battery": 78.4, "position": {...}}
```

---

## Sensor Data API

### 6.1 Get Latest Sensor Data

**Endpoint**: `GET /robots/{robot_id}/sensors`

**Query Parameters**:
- `sensor_type`: Filter by type (`camera`, `lidar`, `imu`)
- `sensor_id`: Specific sensor ID

**Example Response**:

```json
{
  "status": "success",
  "data": {
    "robot_id": "mobile-manipulator-001",
    "timestamp": "2025-01-15T14:30:00.123Z",
    "sensors": {
      "cameras": [
        {
          "sensor_id": "head_camera",
          "image_url": "https://cdn.wia-robot.org/images/mobile-manipulator-001/frame_12345.jpg",
          "depth_url": "https://cdn.wia-robot.org/depth/mobile-manipulator-001/frame_12345.bin"
        }
      ],
      "lidars": [
        {
          "sensor_id": "base_scan",
          "scan_url": "https://cdn.wia-robot.org/scans/mobile-manipulator-001/scan_67890.bin"
        }
      ]
    }
  }
}
```

### 6.2 Get Camera Stream

**Endpoint**: `GET /robots/{robot_id}/sensors/cameras/{camera_id}/stream`

Returns MJPEG or H.264 stream.

### 6.3 Get Point Cloud

**Endpoint**: `GET /robots/{robot_id}/sensors/pointcloud`

**Query Parameters**:
- `format`: `pcd`, `ply`, `las`, or `json`

---

## Multi-Robot Coordination API

### 7.1 Create Fleet

**Endpoint**: `POST /fleets`

```json
{
  "fleet_name": "Warehouse Fleet Alpha",
  "robot_ids": [
    "mobile-manipulator-001",
    "mobile-base-002",
    "mobile-base-003"
  ]
}
```

**Response**:

```json
{
  "status": "success",
  "data": {
    "fleet_id": "fleet-abc123",
    "fleet_name": "Warehouse Fleet Alpha",
    "robot_count": 3,
    "created_at": "2025-01-15T14:30:00Z"
  }
}
```

### 7.2 Assign Multi-Robot Task

**Endpoint**: `POST /fleets/{fleet_id}/tasks`

```json
{
  "task_type": "coordinated_transport",
  "task_allocation": {
    "mobile-manipulator-001": {
      "role": "picker",
      "commands": [ /* array of commands */ ]
    },
    "mobile-base-002": {
      "role": "carrier",
      "commands": [ /* array of commands */ ]
    }
  },
  "synchronization_points": [
    {
      "point_id": "handoff_station",
      "robots": ["mobile-manipulator-001", "mobile-base-002"],
      "max_time_difference_s": 30
    }
  ]
}
```

### 7.3 Get Fleet Status

**Endpoint**: `GET /fleets/{fleet_id}/status`

```json
{
  "status": "success",
  "data": {
    "fleet_id": "fleet-abc123",
    "active_task_id": "task-456",
    "robots": [
      {
        "robot_id": "mobile-manipulator-001",
        "status": "executing",
        "current_action": "navigating_to_shelf"
      },
      {
        "robot_id": "mobile-base-002",
        "status": "waiting",
        "current_action": "at_handoff_station"
      }
    ]
  }
}
```

---

## WebSocket Real-Time Interface

### 8.1 Connection Establishment

**WebSocket URL**: `wss://api.wia-robot.org/v1/robots/{robot_id}/ws`

**Connection Headers**:

```
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
Sec-WebSocket-Protocol: wia-robot-v1
```

### 8.2 Message Format

All WebSocket messages use JSON with a type field:

```json
{
  "type": "state_update",
  "timestamp": "2025-01-15T14:30:00.123Z",
  "data": {
    /* payload specific to message type */
  }
}
```

### 8.3 Client → Server Messages

**Subscribe to Updates**:

```json
{
  "type": "subscribe",
  "channels": ["state", "sensors", "events"],
  "rate_hz": 10
}
```

**Send Command**:

```json
{
  "type": "command",
  "command_id": "cmd-12345",
  "command_type": "navigation",
  "payload": { /* command payload */ }
}
```

### 8.4 Server → Client Messages

**State Update**:

```json
{
  "type": "state_update",
  "timestamp": "2025-01-15T14:30:00.500Z",
  "data": {
    "robot_id": "mobile-manipulator-001",
    "kinematic_state": { /* ... */ },
    "battery": { /* ... */ }
  }
}
```

**Sensor Data**:

```json
{
  "type": "sensor_data",
  "timestamp": "2025-01-15T14:30:00.123Z",
  "data": {
    "sensor_id": "head_camera",
    "image_url": "https://..."
  }
}
```

**Event Notification**:

```json
{
  "type": "event",
  "timestamp": "2025-01-15T14:30:05.000Z",
  "data": {
    "event_type": "task_completed",
    "task_id": "task-123"
  }
}
```

### 8.5 Heartbeat Protocol

Client MUST send ping every 30 seconds:

```json
{"type": "ping", "timestamp": "2025-01-15T14:30:00Z"}
```

Server responds:

```json
{"type": "pong", "timestamp": "2025-01-15T14:30:00.001Z"}
```

---

## Authentication & Authorization

### 9.1 JWT Token Structure

```json
{
  "sub": "ai-system-001",
  "iat": 1736951400,
  "exp": 1736955000,
  "scope": "robot:read robot:write robot:control",
  "robot_ids": ["mobile-manipulator-001", "mobile-base-002"]
}
```

### 9.2 Authentication Flow

```mermaid
sequenceDiagram
    participant Client
    participant Auth
    participant API

    Client->>Auth: POST /auth/login (credentials)
    Auth->>Client: JWT Token
    Client->>API: GET /robots (Bearer token)
    API->>Auth: Validate token
    Auth->>API: Valid
    API->>Client: Robot list
```

### 9.3 API Scopes

| Scope | Permissions |
|-------|-------------|
| `robot:read` | Read robot state, sensors |
| `robot:write` | Update robot configuration |
| `robot:control` | Send commands, control robot |
| `robot:admin` | Register/delete robots |
| `fleet:read` | Read fleet information |
| `fleet:write` | Create/modify fleets |
| `fleet:control` | Assign multi-robot tasks |

### 9.4 Rate Limiting

| Tier | Requests/minute | WebSocket connections |
|------|----------------|----------------------|
| Free | 100 | 5 |
| Developer | 1000 | 50 |
| Production | 10000 | 500 |
| Enterprise | Unlimited | Unlimited |

---

## SDK Reference

### 10.1 TypeScript SDK

**Installation**:

```bash
npm install @wia/robot-interface
```

**Usage**:

```typescript
import { WiaRobotClient } from '@wia/robot-interface';

const client = new WiaRobotClient({
  apiKey: process.env.WIA_API_KEY,
  baseUrl: 'https://api.wia-robot.org/v1'
});

// Discover robots
const robots = await client.robots.list({ type: 'mobile_manipulator' });

// Get robot
const robot = await client.robots.get('mobile-manipulator-001');

// Send command
const command = await robot.sendCommand({
  command_type: 'navigation',
  payload: {
    goal: { position: { x: 5.0, y: 3.0, z: 0.0 } }
  }
});

// Monitor state via WebSocket
robot.onStateUpdate((state) => {
  console.log('Battery:', state.battery.percentage);
});

await robot.connect();
```

### 10.2 Python SDK

**Installation**:

```bash
pip install wia-robot-interface
```

**Usage**:

```python
from wia_robot_interface import WiaRobotClient
import asyncio

async def main():
    client = WiaRobotClient(api_key=os.getenv('WIA_API_KEY'))

    # Discover robots
    robots = await client.robots.list(type='mobile_manipulator')

    # Get robot
    robot = await client.robots.get('mobile-manipulator-001')

    # Send command
    command = await robot.send_command({
        'command_type': 'navigation',
        'payload': {
            'goal': {'position': {'x': 5.0, 'y': 3.0, 'z': 0.0}}
        }
    })

    # Monitor state
    @robot.on_state_update
    def handle_state(state):
        print(f"Battery: {state['battery']['percentage']}")

    await robot.connect()

asyncio.run(main())
```

### 10.3 ROS2 Client

**Installation**:

```bash
sudo apt install ros-humble-wia-robot-interface
```

**Usage**:

```bash
# Launch robot interface node
ros2 run wia_robot_interface robot_client \
  --robot-id mobile-manipulator-001 \
  --api-key $WIA_API_KEY
```

**Topics Published**:
- `/wia/robot/state` - Robot state
- `/wia/robot/sensors/camera` - Camera images
- `/wia/robot/sensors/scan` - LIDAR scans

**Topics Subscribed**:
- `/wia/robot/cmd_vel` - Velocity commands
- `/wia/robot/goal` - Navigation goals

---

## Error Codes Reference

| Code | HTTP Status | Description |
|------|-------------|-------------|
| `ROBOT_NOT_FOUND` | 404 | Robot ID doesn't exist |
| `ROBOT_OFFLINE` | 503 | Robot not connected |
| `INVALID_COMMAND` | 400 | Malformed command |
| `COMMAND_REJECTED` | 422 | Robot rejected command |
| `COMMAND_TIMEOUT` | 504 | Command execution timeout |
| `INSUFFICIENT_PERMISSIONS` | 403 | Missing required scope |
| `RATE_LIMIT_EXCEEDED` | 429 | Too many requests |
| `INVALID_TOKEN` | 401 | JWT expired or invalid |
| `ROBOT_BUSY` | 409 | Robot executing other task |
| `CAPABILITY_NOT_SUPPORTED` | 422 | Robot lacks capability |

---

## References

1. [OpenAPI 3.1 Specification](https://spec.openapis.org/oas/v3.1.0)
2. [RFC 6455 - WebSocket Protocol](https://tools.ietf.org/html/rfc6455)
3. [RFC 7519 - JSON Web Tokens](https://tools.ietf.org/html/rfc7519)
4. [REST API Best Practices](https://restfulapi.net/)
5. [Server-Sent Events](https://html.spec.whatwg.org/multipage/server-sent-events.html)

---

**Document Version**: 1.0.0
**Last Updated**: 2025-01
**Status**: Draft

---

<div align="center" style="color: #6366F1;">

**WIA AI Robot Interface - Phase 2: API Interface Standard**

Seamless AI-Robot Communication at Scale

弘益人間 - Benefit All Humanity

</div>
