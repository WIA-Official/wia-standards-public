# WIA-AMR Phase 3: Protocol Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-01-15

---

## 1. Overview

Phase 3 of the WIA-AMR standard defines real-time communication protocols for AMR systems. This includes MQTT messaging, WebSocket connections, and VDA 5050 compatibility.

### 1.1 Protocol Selection

| Protocol | Use Case | Characteristics |
|----------|----------|-----------------|
| MQTT | Robot-to-FMS communication | Lightweight, pub/sub, QoS levels |
| WebSocket | Browser dashboards | Full-duplex, real-time |
| VDA 5050 | Industry standard compliance | AGV/AMR interoperability |

### 1.2 Design Principles

- **Real-time**: Low latency for control and monitoring
- **Reliability**: Message delivery guarantees via QoS
- **Scalability**: Support for large robot fleets
- **Compatibility**: VDA 5050 alignment

---

## 2. MQTT Protocol

### 2.1 Connection Parameters

| Parameter | Value |
|-----------|-------|
| Protocol | MQTT 3.1.1 or 5.0 |
| Port | 1883 (TCP), 8883 (TLS) |
| Keep Alive | 30 seconds |
| Clean Session | false (for persistent subscriptions) |

### 2.2 Authentication

```
Username: {client_id}
Password: {api_key or jwt_token}
```

### 2.3 Topic Structure

```
/wia/amr/{version}/{category}/{identifier}/{type}
```

**Topic Hierarchy:**

```
/wia/amr/v1/
├── robots/
│   └── {robotId}/
│       ├── state              # Robot publishes state
│       ├── telemetry          # High-frequency sensor data
│       ├── events             # Robot events
│       ├── commands           # FMS sends commands
│       └── commands/ack       # Robot acknowledges
├── fleet/
│   ├── status                 # Fleet-wide status
│   └── alerts                 # Fleet alerts
├── traffic/
│   ├── reservation/request    # Zone reservation requests
│   └── reservation/response   # Zone reservation responses
└── system/
    ├── heartbeat              # System health
    └── config                 # Configuration updates
```

### 2.4 Quality of Service (QoS)

| QoS Level | Guarantee | Use Case |
|-----------|-----------|----------|
| 0 | At most once | Telemetry data (loss acceptable) |
| 1 | At least once | State updates |
| 2 | Exactly once | Critical commands, transactions |

### 2.5 Message Formats

#### Robot State Message

**Topic:** `/wia/amr/v1/robots/{robotId}/state`
**QoS:** 1
**Retain:** true

```json
{
  "robotId": "amr-001",
  "timestamp": "2025-01-15T10:30:00.123Z",
  "position": {
    "x": 45.32,
    "y": 28.91,
    "theta": 1.234,
    "mapId": "warehouse-01"
  },
  "velocity": {
    "linear": 0.85,
    "angular": 0.12
  },
  "battery": {
    "level": 72,
    "charging": false
  },
  "operatingState": "NAVIGATING",
  "safetyState": "SAFE",
  "currentTaskId": "task-12345",
  "errors": []
}
```

#### Command Message

**Topic:** `/wia/amr/v1/robots/{robotId}/commands`
**QoS:** 2

```json
{
  "commandId": "cmd-67890",
  "commandType": "NAVIGATE",
  "timestamp": "2025-01-15T10:30:01.000Z",
  "destination": {
    "x": 100.0,
    "y": 50.0,
    "theta": 0
  },
  "parameters": {
    "maxSpeed": 1.5,
    "priority": "HIGH"
  }
}
```

#### Command Acknowledgment

**Topic:** `/wia/amr/v1/robots/{robotId}/commands/ack`
**QoS:** 2

```json
{
  "commandId": "cmd-67890",
  "status": "ACCEPTED",
  "timestamp": "2025-01-15T10:30:01.050Z",
  "estimatedDuration": 45,
  "errorCode": null,
  "errorMessage": null
}
```

#### Event Message

**Topic:** `/wia/amr/v1/robots/{robotId}/events`
**QoS:** 1

```json
{
  "eventId": "evt-001",
  "eventType": "TASK_COMPLETED",
  "timestamp": "2025-01-15T10:35:00Z",
  "robotId": "amr-001",
  "data": {
    "taskId": "task-12345",
    "duration": 180,
    "distanceTraveled": 45.6
  }
}
```

### 2.6 Retained Messages

The following topics SHOULD use retained messages:

- `/wia/amr/v1/robots/{robotId}/state` - Last known state
- `/wia/amr/v1/fleet/status` - Fleet status
- `/wia/amr/v1/system/config` - System configuration

### 2.7 Last Will and Testament (LWT)

Robots MUST configure LWT for connection loss detection:

**Topic:** `/wia/amr/v1/robots/{robotId}/connection`
**Payload:**

```json
{
  "robotId": "amr-001",
  "status": "DISCONNECTED",
  "timestamp": "2025-01-15T10:30:00Z"
}
```

---

## 3. VDA 5050 Compatibility

### 3.1 Overview

VDA 5050 is the German Association of the Automotive Industry (VDA) standard for AGV/AMR communication. WIA-AMR maintains full compatibility with VDA 5050.

### 3.2 Topic Mapping

| VDA 5050 Topic | WIA-AMR Topic |
|----------------|---------------|
| `{interface}/{version}/{manufacturer}/{serialNumber}/order` | `/wia/amr/v1/robots/{robotId}/order` |
| `{interface}/{version}/{manufacturer}/{serialNumber}/state` | `/wia/amr/v1/robots/{robotId}/state` |
| `{interface}/{version}/{manufacturer}/{serialNumber}/visualization` | `/wia/amr/v1/robots/{robotId}/telemetry` |
| `{interface}/{version}/{manufacturer}/{serialNumber}/connection` | `/wia/amr/v1/robots/{robotId}/connection` |

### 3.3 Order Message (VDA 5050 Compatible)

```json
{
  "headerId": 1234,
  "timestamp": "2025-01-15T10:30:00.000Z",
  "version": "2.0.0",
  "manufacturer": "WIA",
  "serialNumber": "amr-001",
  "orderId": "order-001",
  "orderUpdateId": 0,
  "nodes": [
    {
      "nodeId": "node-1",
      "sequenceId": 0,
      "released": true,
      "nodePosition": {
        "x": 0.0,
        "y": 0.0,
        "theta": 0.0,
        "mapId": "warehouse-01",
        "allowedDeviationXY": 0.5,
        "allowedDeviationTheta": 0.1
      },
      "actions": []
    },
    {
      "nodeId": "node-2",
      "sequenceId": 2,
      "released": true,
      "nodePosition": {
        "x": 10.0,
        "y": 0.0,
        "theta": 0.0,
        "mapId": "warehouse-01"
      },
      "actions": [
        {
          "actionId": "action-001",
          "actionType": "pick",
          "actionDescription": "Pick item",
          "blockingType": "HARD",
          "actionParameters": [
            { "key": "stationType", "value": "rack" }
          ]
        }
      ]
    }
  ],
  "edges": [
    {
      "edgeId": "edge-1-2",
      "sequenceId": 1,
      "released": true,
      "startNodeId": "node-1",
      "endNodeId": "node-2",
      "maxSpeed": 1.5,
      "maxRotationSpeed": 0.5,
      "orientation": 0.0,
      "direction": "straight",
      "actions": []
    }
  ]
}
```

### 3.4 State Message (VDA 5050 Compatible)

```json
{
  "headerId": 5678,
  "timestamp": "2025-01-15T10:30:05.000Z",
  "version": "2.0.0",
  "manufacturer": "WIA",
  "serialNumber": "amr-001",
  "orderId": "order-001",
  "orderUpdateId": 0,
  "lastNodeId": "node-1",
  "lastNodeSequenceId": 0,
  "driving": true,
  "paused": false,
  "newBaseRequest": false,
  "distanceSinceLastNode": 5.2,
  "nodeStates": [],
  "edgeStates": [
    {
      "edgeId": "edge-1-2",
      "sequenceId": 1,
      "released": true
    }
  ],
  "agvPosition": {
    "x": 5.0,
    "y": 0.0,
    "theta": 0.0,
    "mapId": "warehouse-01",
    "positionInitialized": true,
    "localizationScore": 0.95
  },
  "velocity": {
    "vx": 1.2,
    "vy": 0.0,
    "omega": 0.0
  },
  "loads": [],
  "actionStates": [],
  "batteryState": {
    "batteryCharge": 72.5,
    "batteryVoltage": 48.2,
    "batteryHealth": 95,
    "charging": false,
    "reach": 3600
  },
  "operatingMode": "AUTOMATIC",
  "errors": [],
  "information": [],
  "safetyState": {
    "eStop": "NONE",
    "fieldViolation": false
  }
}
```

### 3.5 VDA 5050 Adapter

WIA-AMR provides adapters for bidirectional conversion:

```typescript
import { VDA5050Adapter } from '@wia/amr-sdk';

const adapter = new VDA5050Adapter();

// Convert WIA-AMR Task to VDA 5050 Order
const order = adapter.taskToOrder(wiaTask);

// Convert VDA 5050 State to WIA-AMR RobotState
const robotState = adapter.stateToRobotState(vdaState);
```

---

## 4. WebSocket Protocol

### 4.1 Connection

```javascript
const ws = new WebSocket('wss://api.example.com/ws/v1');
```

### 4.2 Authentication

Send authentication immediately after connection:

```json
{
  "type": "AUTH",
  "token": "Bearer eyJhbGciOiJSUzI1NiIs..."
}
```

Response:

```json
{
  "type": "AUTH_RESPONSE",
  "status": "SUCCESS",
  "sessionId": "sess-12345"
}
```

### 4.3 Subscription

```json
{
  "type": "SUBSCRIBE",
  "channels": [
    "robots.*.state",
    "fleet.alerts",
    "tasks.*.status"
  ]
}
```

### 4.4 Message Types

| Type | Direction | Description |
|------|-----------|-------------|
| AUTH | Client → Server | Authentication |
| SUBSCRIBE | Client → Server | Subscribe to channels |
| UNSUBSCRIBE | Client → Server | Unsubscribe |
| COMMAND | Client → Server | Send command |
| ROBOT_STATE | Server → Client | Robot state update |
| TASK_UPDATE | Server → Client | Task status change |
| ALERT | Server → Client | System alert |
| PING/PONG | Bidirectional | Keep-alive |

### 4.5 Robot State Streaming

```json
{
  "type": "ROBOT_STATE",
  "timestamp": "2025-01-15T10:30:00.123Z",
  "robotId": "amr-001",
  "data": {
    "position": { "x": 45.32, "y": 28.91, "theta": 1.234 },
    "velocity": { "linear": 0.85, "angular": 0.12 },
    "operatingState": "NAVIGATING",
    "batteryLevel": 72
  }
}
```

---

## 5. Multi-Robot Coordination

### 5.1 Traffic Management

WIA-AMR supports centralized traffic management for collision avoidance.

#### Zone Reservation Request

**Topic:** `/wia/amr/v1/traffic/reservation/request`

```json
{
  "requestId": "res-001",
  "robotId": "amr-001",
  "timestamp": "2025-01-15T10:30:00Z",
  "zones": ["zone-A1", "zone-A2"],
  "estimatedDuration": 30,
  "priority": 80,
  "currentPosition": { "x": 10.0, "y": 20.0 }
}
```

#### Zone Reservation Response

**Topic:** `/wia/amr/v1/robots/{robotId}/traffic/reservation/response`

```json
{
  "requestId": "res-001",
  "status": "GRANTED",
  "timestamp": "2025-01-15T10:30:00.050Z",
  "validUntil": "2025-01-15T10:30:30Z",
  "zones": ["zone-A1", "zone-A2"],
  "waitingRobots": []
}
```

#### Zone Release

**Topic:** `/wia/amr/v1/traffic/reservation/release`

```json
{
  "robotId": "amr-001",
  "zones": ["zone-A1"],
  "timestamp": "2025-01-15T10:30:15Z"
}
```

### 5.2 Priority Levels

| Priority | Range | Description |
|----------|-------|-------------|
| CRITICAL | 90-100 | Emergency, safety |
| HIGH | 70-89 | Urgent tasks |
| NORMAL | 30-69 | Standard operations |
| LOW | 0-29 | Background tasks |

---

## 6. Connection Management

### 6.1 Reconnection Strategy

```
Initial delay: 1 second
Max delay: 60 seconds
Backoff multiplier: 2
Max attempts: unlimited
```

### 6.2 Heartbeat

- **Interval:** 30 seconds
- **Timeout:** 60 seconds (2 missed heartbeats)

### 6.3 Session Recovery

After reconnection, clients SHOULD:

1. Re-authenticate
2. Re-subscribe to topics
3. Request missed messages (if supported)
4. Sync current state

---

## 7. Security

### 7.1 Transport Security

- TLS 1.2 or higher required for production
- Certificate validation mandatory
- Client certificates supported (mTLS)

### 7.2 Message Security

- Signed messages for critical commands (optional)
- Encrypted payloads for sensitive data (optional)

### 7.3 Access Control

Topic-level access control:

| Role | Allowed Topics |
|------|----------------|
| robot | Own robot topics, traffic requests |
| operator | All robot topics (read), commands |
| admin | All topics |
| system | Integration topics |

---

## 8. Message Sequencing

### 8.1 Sequence Numbers

All stateful messages include sequence numbers:

```json
{
  "headerId": 12345,
  "timestamp": "2025-01-15T10:30:00Z",
  ...
}
```

### 8.2 Duplicate Detection

Receivers SHOULD track received sequence numbers and ignore duplicates.

### 8.3 Ordering

Messages with the same robotId SHOULD be processed in order.

---

## 9. References

- MQTT 5.0 Specification: https://docs.oasis-open.org/mqtt/mqtt/v5.0/mqtt-v5.0.html
- VDA 5050: https://github.com/VDA5050/VDA5050
- WebSocket Protocol: https://tools.ietf.org/html/rfc6455

---

© 2025 WIA Standards | 弘益人間 · Benefit All Humanity
