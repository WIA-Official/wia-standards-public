# WIA AI Embodiment API Interface Standard
## Phase 2 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT

---

## Table of Contents

1. [Overview](#overview)
2. [Terminology](#terminology)
3. [Core Interfaces](#core-interfaces)
4. [REST API Endpoints](#rest-api-endpoints)
5. [WebSocket Interface](#websocket-interface)
6. [Authentication](#authentication)
7. [Error Handling](#error-handling)
8. [Rate Limiting](#rate-limiting)
9. [SDK Examples](#sdk-examples)
10. [References](#references)

---

## Overview

### 1.1 Purpose

The WIA AI Embodiment API Interface Standard defines the programming interfaces for controlling and monitoring AI-powered physical systems. This specification enables developers to integrate AI decision-making with physical embodiments through standardized APIs.

**Core Objectives**:
- Provide RESTful APIs for embodiment management
- Enable real-time control via WebSocket connections
- Define standardized SDK interfaces for TypeScript/Python
- Ensure safe and controlled physical system operation
- Support multi-embodiment coordination

### 1.2 Scope

| Component | Description |
|-----------|-------------|
| **REST API** | CRUD operations, configuration, state queries |
| **WebSocket** | Real-time control and sensor streaming |
| **SDK** | TypeScript and Python client libraries |
| **Authentication** | JWT-based authentication system |
| **Safety Controls** | API-level safety enforcement |

### 1.3 Phase 1 Compatibility

Phase 2 API operates on Phase 1 Data Format:

```
Phase 1: Data Format (JSON structures)
    ↓
Phase 2: API Interface (Programming APIs)
    ↓
Phase 3: Protocol (Communication)
```

---

## Terminology

### 2.1 Core Terms

| Term | Definition |
|------|------------|
| **WiaEmbodiment** | Main API client class |
| **Controller** | Real-time motion controller |
| **StateManager** | Embodiment state handler |
| **SafetyMonitor** | Safety constraint enforcer |
| **SensorStream** | Real-time sensor data stream |
| **CommandQueue** | Prioritized command buffer |

### 2.2 API Conventions

| Convention | Description |
|------------|-------------|
| Base URL | `https://api.wia.live/embodiment/v1` |
| Content-Type | `application/json` |
| Date Format | ISO 8601 |
| Authentication | Bearer token (JWT) |

---

## Core Interfaces

### 3.1 WiaEmbodiment Class

Main API entry point.

#### TypeScript

```typescript
class WiaEmbodiment {
  // Constructor
  constructor(options?: WiaEmbodimentOptions);

  // Connection Management
  connect(config: EmbodimentConfig): Promise<void>;
  disconnect(): Promise<void>;
  isConnected(): boolean;

  // Embodiment Discovery
  listEmbodiments(): Promise<EmbodimentInfo[]>;
  getEmbodiment(id: string): Promise<EmbodimentInfo>;

  // State Management
  getState(): Promise<EmbodimentState>;
  setState(state: Partial<EmbodimentState>): Promise<void>;

  // Control
  sendCommand(command: ActuatorCommand): Promise<CommandResult>;
  sendCommands(commands: ActuatorCommand[]): Promise<CommandResult[]>;
  emergencyStop(): Promise<void>;

  // Sensor Access
  getSensorData(): Promise<SensorData>;
  subscribeSensors(callback: SensorCallback): Subscription;

  // Safety
  getSafetyStatus(): Promise<SafetyData>;
  setSafetyMode(mode: SafetyMode): Promise<void>;

  // Events
  on<T extends EventType>(event: T, handler: EventHandler<T>): void;
  off<T extends EventType>(event: T, handler: EventHandler<T>): void;

  // Configuration
  configure(options: Partial<ControllerOptions>): void;
  getConfig(): EmbodimentConfig;
}
```

#### Python

```python
class WiaEmbodiment:
    def __init__(self, options: Optional[WiaEmbodimentOptions] = None):
        ...

    # Connection Management
    async def connect(self, config: EmbodimentConfig) -> None: ...
    async def disconnect(self) -> None: ...
    def is_connected(self) -> bool: ...

    # Embodiment Discovery
    async def list_embodiments(self) -> List[EmbodimentInfo]: ...
    async def get_embodiment(self, id: str) -> EmbodimentInfo: ...

    # State Management
    async def get_state(self) -> EmbodimentState: ...
    async def set_state(self, state: EmbodimentState) -> None: ...

    # Control
    async def send_command(self, command: ActuatorCommand) -> CommandResult: ...
    async def send_commands(self, commands: List[ActuatorCommand]) -> List[CommandResult]: ...
    async def emergency_stop(self) -> None: ...

    # Sensor Access
    async def get_sensor_data(self) -> SensorData: ...
    def subscribe_sensors(self, callback: SensorCallback) -> Subscription: ...

    # Safety
    async def get_safety_status(self) -> SafetyData: ...
    async def set_safety_mode(self, mode: SafetyMode) -> None: ...

    # Events
    def on(self, event: EventType, handler: EventHandler) -> None: ...
    def off(self, event: EventType, handler: EventHandler) -> None: ...
```

### 3.2 Configuration Options

```typescript
interface WiaEmbodimentOptions {
  // Connection
  baseUrl?: string;              // default: 'https://api.wia.live/embodiment/v1'
  wsUrl?: string;                // default: 'wss://api.wia.live/embodiment/v1/ws'
  timeout?: number;              // default: 5000 (ms)

  // Auto-reconnect
  autoReconnect?: boolean;       // default: true
  reconnectInterval?: number;    // default: 1000 (ms)
  maxReconnectAttempts?: number; // default: 10

  // Control
  commandBufferSize?: number;    // default: 100
  controlRate?: number;          // default: 100 (Hz)

  // Safety
  safetyEnabled?: boolean;       // default: true
  emergencyStopOnError?: boolean;// default: true

  // Logging
  logLevel?: 'debug' | 'info' | 'warn' | 'error' | 'none';
}

interface EmbodimentConfig {
  embodimentId: string;
  embodimentType: EmbodimentType;
  controlMode?: 'position' | 'velocity' | 'torque' | 'impedance';
  safetyMode?: SafetyMode;
  jointLimits?: JointLimits[];
  workspaceLimits?: WorkspaceLimits;
}
```

### 3.3 Control Modes

```typescript
enum ControlMode {
  POSITION = 'position',      // Position control
  VELOCITY = 'velocity',      // Velocity control
  TORQUE = 'torque',          // Torque/force control
  IMPEDANCE = 'impedance'     // Impedance control
}

interface ImpedanceParams {
  stiffness: number[];   // Nm/rad or N/m
  damping: number[];     // Nm·s/rad or N·s/m
  inertia?: number[];    // kg·m² or kg
}
```

---

## REST API Endpoints

### 4.1 Endpoint Summary

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/embodiments` | List all embodiments |
| GET | `/embodiments/{id}` | Get embodiment details |
| POST | `/embodiments/{id}/connect` | Connect to embodiment |
| POST | `/embodiments/{id}/disconnect` | Disconnect from embodiment |
| GET | `/embodiments/{id}/state` | Get current state |
| PUT | `/embodiments/{id}/state` | Update state |
| POST | `/embodiments/{id}/commands` | Send commands |
| POST | `/embodiments/{id}/emergency-stop` | Emergency stop |
| GET | `/embodiments/{id}/sensors` | Get sensor data |
| GET | `/embodiments/{id}/safety` | Get safety status |
| PUT | `/embodiments/{id}/safety` | Update safety config |
| GET | `/embodiments/{id}/config` | Get configuration |
| PUT | `/embodiments/{id}/config` | Update configuration |
| POST | `/embodiments/{id}/calibrate` | Start calibration |

### 4.2 List Embodiments

**GET** `/embodiments`

Lists all available embodiments.

**Response:**
```json
{
  "success": true,
  "data": {
    "embodiments": [
      {
        "id": "emb-001",
        "name": "HumanoidX-1",
        "type": "humanoid_robot",
        "status": "available",
        "manufacturer": "WIA Robotics",
        "model": "HX-1",
        "firmware": "2.1.0",
        "lastSeen": "2025-01-01T12:00:00Z"
      }
    ],
    "total": 1
  }
}
```

### 4.3 Get Embodiment State

**GET** `/embodiments/{id}/state`

Returns current embodiment state (Phase 1 format).

**Response:**
```json
{
  "success": true,
  "data": {
    "embodiment_id": "emb-001",
    "timestamp": 1704110400000,
    "state": {
      "operational_mode": "active",
      "power_status": {
        "battery_level": 85.5,
        "voltage": 48.2
      },
      "joints": [
        {
          "joint_id": "right_shoulder_pitch",
          "position": 0.5,
          "velocity": 0.1,
          "torque": 15.0
        }
      ]
    }
  }
}
```

### 4.4 Send Commands

**POST** `/embodiments/{id}/commands`

Sends actuator commands.

**Request:**
```json
{
  "commands": [
    {
      "actuator_id": "right_shoulder_pitch",
      "command_type": "position",
      "target_value": 0.8,
      "duration_ms": 500,
      "priority": 5
    },
    {
      "actuator_id": "right_elbow_pitch",
      "command_type": "position",
      "target_value": 1.5,
      "duration_ms": 500,
      "priority": 5
    }
  ],
  "execution_mode": "synchronized"
}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "command_id": "cmd-12345",
    "status": "accepted",
    "estimated_completion_ms": 500,
    "results": [
      { "actuator_id": "right_shoulder_pitch", "status": "queued" },
      { "actuator_id": "right_elbow_pitch", "status": "queued" }
    ]
  }
}
```

### 4.5 Emergency Stop

**POST** `/embodiments/{id}/emergency-stop`

Triggers immediate emergency stop.

**Response:**
```json
{
  "success": true,
  "data": {
    "emergency_stop_active": true,
    "timestamp": 1704110400000,
    "all_actuators_stopped": true
  }
}
```

### 4.6 Get Sensor Data

**GET** `/embodiments/{id}/sensors`

**Query Parameters:**
- `types`: Comma-separated sensor types (e.g., `imu,force_torque`)
- `include_raw`: Include raw sensor data (boolean)

**Response:**
```json
{
  "success": true,
  "data": {
    "timestamp": 1704110400000,
    "sensors": {
      "imu": {
        "acceleration": { "x": 0.1, "y": -0.05, "z": 9.81 },
        "angular_velocity": { "x": 0.01, "y": 0.02, "z": 0.0 }
      },
      "force_torque": [
        {
          "sensor_id": "right_wrist_ft",
          "force": { "x": 2.5, "y": 0.5, "z": -15.0 },
          "torque": { "x": 0.1, "y": 0.2, "z": 0.05 }
        }
      ]
    }
  }
}
```

### 4.7 Update Safety Configuration

**PUT** `/embodiments/{id}/safety`

**Request:**
```json
{
  "safety_mode": "collaborative",
  "constraints": {
    "max_velocity": 0.5,
    "max_force": 150.0,
    "human_proximity_threshold": 0.3,
    "restricted_zones": [
      {
        "zone_id": "restricted_1",
        "type": "sphere",
        "center": { "x": 2.0, "y": 1.0, "z": 1.0 },
        "radius": 0.5
      }
    ]
  }
}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "safety_mode": "collaborative",
    "constraints_applied": true,
    "active_constraints": 4
  }
}
```

### 4.8 Calibration

**POST** `/embodiments/{id}/calibrate`

**Request:**
```json
{
  "calibration_type": "full",
  "components": ["joints", "sensors", "end_effectors"],
  "options": {
    "use_external_reference": false,
    "save_calibration": true
  }
}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "calibration_id": "cal-001",
    "status": "in_progress",
    "estimated_duration_ms": 30000,
    "progress_url": "/embodiments/emb-001/calibrations/cal-001"
  }
}
```

---

## WebSocket Interface

### 5.1 Connection

Connect to real-time interface:

```
wss://api.wia.live/embodiment/v1/ws?embodiment_id={id}&token={jwt}
```

### 5.2 Message Types

| Type | Direction | Description |
|------|-----------|-------------|
| `state_update` | S → C | Real-time state updates |
| `sensor_data` | S → C | Sensor data stream |
| `command` | C → S | Actuator commands |
| `command_ack` | S → C | Command acknowledgment |
| `safety_event` | S → C | Safety alerts |
| `heartbeat` | Both | Connection keepalive |

### 5.3 State Subscription

**Subscribe to state updates:**
```json
{
  "type": "subscribe",
  "channel": "state",
  "options": {
    "rate": 100,
    "include": ["joints", "end_effectors"]
  }
}
```

**State update message:**
```json
{
  "type": "state_update",
  "timestamp": 1704110400000,
  "sequence": 12345,
  "data": {
    "joints": [
      { "joint_id": "j1", "position": 0.5, "velocity": 0.1 }
    ]
  }
}
```

### 5.4 Real-Time Commands

**Send real-time command:**
```json
{
  "type": "command",
  "command_id": "rt-cmd-001",
  "timestamp": 1704110400000,
  "data": {
    "actuator_id": "right_shoulder_pitch",
    "command_type": "velocity",
    "target_value": 0.5
  }
}
```

**Command acknowledgment:**
```json
{
  "type": "command_ack",
  "command_id": "rt-cmd-001",
  "status": "executed",
  "timestamp": 1704110400005,
  "latency_ms": 5
}
```

---

## Authentication

### 6.1 JWT Authentication

All API requests require JWT bearer tokens:

```http
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

### 6.2 Token Structure

```json
{
  "sub": "user-123",
  "aud": "wia-embodiment",
  "iat": 1704110400,
  "exp": 1704196800,
  "permissions": [
    "embodiment:read",
    "embodiment:control",
    "embodiment:configure"
  ],
  "embodiments": ["emb-001", "emb-002"]
}
```

### 6.3 Permission Levels

| Permission | Description |
|------------|-------------|
| `embodiment:read` | Read state and sensor data |
| `embodiment:control` | Send commands |
| `embodiment:configure` | Modify configuration |
| `embodiment:admin` | Full administrative access |
| `safety:override` | Override safety constraints |

### 6.4 API Key Authentication

For server-to-server communication:

```http
X-API-Key: wia_emb_xxxxxxxxxxxxxxxxxxxxxxxx
```

---

## Error Handling

### 7.1 Error Response Format

```json
{
  "success": false,
  "error": {
    "code": "COMMAND_REJECTED",
    "message": "Command exceeds safety limits",
    "details": {
      "constraint": "max_velocity",
      "requested": 2.5,
      "limit": 1.0
    },
    "timestamp": 1704110400000,
    "request_id": "req-12345"
  }
}
```

### 7.2 Error Codes

#### Connection Errors (1xxx)

| Code | Name | Description |
|------|------|-------------|
| 1001 | `CONNECTION_FAILED` | Failed to connect |
| 1002 | `CONNECTION_LOST` | Connection dropped |
| 1003 | `CONNECTION_TIMEOUT` | Connection timed out |
| 1004 | `EMBODIMENT_NOT_FOUND` | Embodiment doesn't exist |
| 1005 | `EMBODIMENT_BUSY` | Embodiment in use |

#### Control Errors (2xxx)

| Code | Name | Description |
|------|------|-------------|
| 2001 | `COMMAND_REJECTED` | Command not accepted |
| 2002 | `COMMAND_TIMEOUT` | Command execution timeout |
| 2003 | `INVALID_TARGET` | Invalid target value |
| 2004 | `ACTUATOR_ERROR` | Actuator fault |
| 2005 | `MOTION_INTERRUPTED` | Motion was interrupted |

#### Safety Errors (3xxx)

| Code | Name | Description |
|------|------|-------------|
| 3001 | `SAFETY_LIMIT_EXCEEDED` | Safety constraint violated |
| 3002 | `EMERGENCY_STOP_ACTIVE` | E-stop is engaged |
| 3003 | `COLLISION_DETECTED` | Collision occurred |
| 3004 | `HUMAN_PROXIMITY` | Human too close |
| 3005 | `WORKSPACE_VIOLATION` | Outside workspace |

#### Authentication Errors (4xxx)

| Code | Name | Description |
|------|------|-------------|
| 4001 | `UNAUTHORIZED` | Invalid or missing token |
| 4002 | `FORBIDDEN` | Insufficient permissions |
| 4003 | `TOKEN_EXPIRED` | Token has expired |

---

## Rate Limiting

### 8.1 Rate Limits

| Endpoint Category | Rate Limit | Window |
|-------------------|------------|--------|
| Read operations | 1000 req/min | Per user |
| Write operations | 100 req/min | Per user |
| Real-time commands | 1000 cmd/sec | Per embodiment |
| Emergency stop | Unlimited | - |

### 8.2 Rate Limit Headers

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 950
X-RateLimit-Reset: 1704110460
```

### 8.3 Rate Limit Exceeded Response

```json
{
  "success": false,
  "error": {
    "code": "RATE_LIMIT_EXCEEDED",
    "message": "Too many requests",
    "retry_after": 30
  }
}
```

---

## SDK Examples

### 9.1 TypeScript - Basic Control

```typescript
import { WiaEmbodiment, ControlMode, SafetyMode } from 'wia-embodiment';

async function main() {
  const client = new WiaEmbodiment({
    baseUrl: 'https://api.wia.live/embodiment/v1',
    safetyEnabled: true
  });

  // Authenticate
  await client.authenticate({
    apiKey: 'wia_emb_xxxxxxxx'
  });

  // Connect to embodiment
  await client.connect({
    embodimentId: 'emb-001',
    controlMode: ControlMode.POSITION,
    safetyMode: SafetyMode.COLLABORATIVE
  });

  // Subscribe to sensor data
  const subscription = client.subscribeSensors((data) => {
    console.log('Sensor update:', data.timestamp);
  });

  // Send position command
  const result = await client.sendCommand({
    actuatorId: 'right_shoulder_pitch',
    commandType: 'position',
    targetValue: 0.8,
    durationMs: 1000
  });

  console.log('Command result:', result.status);

  // Handle safety events
  client.on('safety_event', (event) => {
    if (event.severity === 'critical') {
      client.emergencyStop();
    }
  });

  // Cleanup
  subscription.unsubscribe();
  await client.disconnect();
}

main().catch(console.error);
```

### 9.2 Python - Motion Sequence

```python
import asyncio
from wia_embodiment import WiaEmbodiment, ControlMode, SafetyMode

async def main():
    client = WiaEmbodiment(
        base_url='https://api.wia.live/embodiment/v1',
        safety_enabled=True
    )

    # Authenticate
    await client.authenticate(api_key='wia_emb_xxxxxxxx')

    # Connect
    await client.connect(
        embodiment_id='emb-001',
        control_mode=ControlMode.POSITION,
        safety_mode=SafetyMode.COLLABORATIVE
    )

    # Define motion sequence
    motion_sequence = [
        {'joint': 'right_shoulder_pitch', 'target': 0.5, 'duration': 500},
        {'joint': 'right_elbow_pitch', 'target': 1.2, 'duration': 500},
        {'joint': 'right_wrist_rotation', 'target': 0.0, 'duration': 300},
    ]

    # Execute sequence
    for motion in motion_sequence:
        result = await client.send_command(
            actuator_id=motion['joint'],
            command_type='position',
            target_value=motion['target'],
            duration_ms=motion['duration']
        )
        print(f"Motion {motion['joint']}: {result.status}")
        await asyncio.sleep(motion['duration'] / 1000)

    # Get final state
    state = await client.get_state()
    print(f"Final position: {state.joints}")

    await client.disconnect()

asyncio.run(main())
```

### 9.3 Real-Time Control Loop

```typescript
import { WiaEmbodiment, ControlMode } from 'wia-embodiment';

async function realTimeControl() {
  const client = new WiaEmbodiment({
    controlRate: 100  // 100 Hz control loop
  });

  await client.connect({
    embodimentId: 'emb-001',
    controlMode: ControlMode.VELOCITY
  });

  // Real-time control loop
  const controlLoop = setInterval(async () => {
    // Get current sensor data
    const sensors = await client.getSensorData();

    // Compute control output (example: proportional control)
    const targetPosition = 1.0;
    const currentPosition = sensors.joints[0].position;
    const error = targetPosition - currentPosition;
    const velocityCommand = error * 2.0;  // Kp = 2.0

    // Send velocity command
    await client.sendCommand({
      actuatorId: 'right_shoulder_pitch',
      commandType: 'velocity',
      targetValue: velocityCommand
    });
  }, 10);  // 100 Hz

  // Stop after 5 seconds
  setTimeout(() => {
    clearInterval(controlLoop);
    client.emergencyStop();
    client.disconnect();
  }, 5000);
}
```

---

## References

### Related Standards

- [WIA AI Embodiment Data Format (Phase 1)](/spec/PHASE-1-DATA-FORMAT.md)
- [WIA AI Motor Control Standard](/ai-motor-control/)
- [WIA AI Robot Interface Standard](/ai-robot-interface/)

### External References

- ROS2 Control Framework
- IEEE 1872 - Standard for Robot Map Data
- ISO 10218 - Robots and Robotic Devices Safety

---

<div align="center">

**WIA AI Embodiment API Interface Standard v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA Standards Committee**

**MIT License**

</div>
