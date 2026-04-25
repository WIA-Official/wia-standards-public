# WIA Upcycling Standard
## Phase 3: Protocol Specification
### Version 1.0.0

---

## 1. Introduction

Phase 3 defines real-time communication protocols for robot control, material flow management, sensor data streaming, and safety monitoring. WebSocket provides primary transport with fallback options.

### 1.1 Real-Time Requirements

- **Continuous Communication:** Persistent bidirectional data flow
- **Low Latency:** <10ms for critical commands
- **High Frequency:** 10-100 Hz update rates
- **Reliable Delivery:** Guaranteed delivery for critical messages
- **Ordered Delivery:** Commands execute in correct sequence

### 1.2 Protocol Stack

| Protocol | Use Case | Frequency | Latency |
|----------|----------|-----------|---------|
| WebSocket | Primary real-time control | 10-100 Hz | <10ms |
| MQTT | Sensor data streaming | 1-50 Hz | <50ms |
| gRPC | High-performance control | 100+ Hz | <5ms |
| UDP | Emergency shutdown | As needed | <1ms |

## 2. WebSocket Foundation

### 2.1 Connection Establishment

```javascript
const ws = new WebSocket('wss://printer.example.com:8443/wia/v1/control');

ws.onopen = function(event) {
    ws.send(JSON.stringify({
        type: 'AUTH',
        token: 'Bearer ...'
    }));
};

ws.onmessage = function(event) {
    const message = JSON.parse(event.data);
    handleMessage(message);
};
```

### 2.2 Message Format

All WebSocket messages follow standard format:

```json
{
  "type": "MESSAGE_TYPE",
  "timestamp": "ISO 8601",
  "sequenceNumber": "Integer",
  "payload": {}
}
```

### 2.3 Message Types

| Type | Direction | Purpose | QoS |
|------|-----------|---------|-----|
| AUTH | Client → Server | Authentication | High |
| COMMAND | Client → Server | Control commands | High |
| STATUS | Server → Client | System status | Medium |
| SENSOR | Server → Client | Sensor data | Low |
| EVENT | Server → Client | Significant events | Medium |
| HEARTBEAT | Bidirectional | Connection alive | High |
| ACK | Bidirectional | Acknowledgment | High |
| ERROR | Bidirectional | Error notifications | High |

## 3. Robot Control Protocol

### 3.1 Motion Commands

```json
{
  "type": "COMMAND",
  "timestamp": "2025-03-12T10:30:45.123Z",
  "sequenceNumber": 12345,
  "payload": {
    "command": "MOVE",
    "target": {
      "position": {"x": 5000.0, "y": 2500.0, "z": 120.0, "unit": "mm"},
      "velocity": {"linear": 100.0, "unit": "mm/s"},
      "acceleration": {"value": 500.0, "unit": "mm/s²"}
    },
    "materialFlow": {
      "enabled": true,
      "rate": 1000.0,
      "unit": "mm³/s"
    },
    "coordinateSystem": "absolute"
  }
}
```

### 3.2 Path Streaming

```json
{
  "type": "COMMAND",
  "payload": {
    "command": "PATH_SEGMENT",
    "pathId": "layer-025-perimeter",
    "segmentNumber": 157,
    "points": [
      {"x": 5000.0, "y": 2500.0, "z": 120.0},
      {"x": 5100.0, "y": 2500.0, "z": 120.0}
    ],
    "velocity": 100.0,
    "materialFlow": 1000.0,
    "blending": {
      "enabled": true,
      "radius": 5.0
    }
  }
}
```

### 3.3 Position Feedback

```json
{
  "type": "STATUS",
  "payload": {
    "robot": {
      "position": {"x": 5075.3, "y": 2501.2, "z": 120.0},
      "velocity": {"linear": 98.5, "angular": 0.0},
      "status": "moving",
      "bufferFill": 0.75
    }
  }
}
```

## 4. Material Flow Management

### 4.1 Flow Control

```json
{
  "type": "COMMAND",
  "payload": {
    "command": "SET_MATERIAL_FLOW",
    "flowRate": {"target": 1200.0, "unit": "mm³/s", "rampTime": 0.5},
    "pressure": {"target": 1.25, "max": 1.5, "unit": "MPa"},
    "temperature": {"target": 25.0, "tolerance": 2.0, "unit": "celsius"}
  }
}
```

### 4.2 Material Monitoring

```json
{
  "type": "SENSOR",
  "payload": {
    "sensor": "material-flow",
    "measurements": {
      "flowRate": {"value": 1195.3, "unit": "mm³/s"},
      "pressure": {"value": 1.23, "unit": "MPa"},
      "temperature": {"value": 24.8, "unit": "celsius"},
      "viscosity": {"value": 45.2, "unit": "Pa·s"}
    },
    "status": "normal"
  }
}
```

## 5. Sensor Data Streaming

### 5.1 Sensor Types

| Sensor | Measurements | Rate | Purpose |
|--------|-------------|------|---------|
| Position | X, Y, Z coordinates | 100 Hz | Motion feedback |
| Material Flow | Flow, pressure, temp | 50 Hz | Material control |
| Laser Scanner | Layer geometry | 10 Hz | Quality verification |
| Camera | Visual | 30 FPS | Monitoring |
| Environmental | Temp, humidity, wind | 1 Hz | Conditions |
| Vibration | Acceleration | 1000 Hz | Equipment health |

### 5.2 Laser Scanner Data

```json
{
  "type": "SENSOR",
  "payload": {
    "sensor": "laser-scanner",
    "scan": {
      "layerNumber": 25,
      "points": [[x, y, z], ...],
      "analysis": {
        "meanHeight": 120.0,
        "stdDeviation": 0.5,
        "withinTolerance": true
      }
    }
  }
}
```

## 6. Safety Monitoring

### 6.1 Safety Zones

```json
{
  "type": "COMMAND",
  "payload": {
    "command": "DEFINE_SAFETY_ZONE",
    "zoneId": "exclusion-001",
    "type": "exclusion",
    "geometry": {
      "type": "cylinder",
      "center": {"x": 5000, "y": 3000, "z": 0},
      "radius": 500,
      "height": 3000
    },
    "action": "emergency-stop"
  }
}
```

### 6.2 Emergency Stop

```json
{
  "type": "COMMAND",
  "payload": {
    "command": "EMERGENCY_STOP",
    "reason": "safety-zone-violation",
    "priority": "critical"
  }
}
```

**Response (within 5ms):**
```json
{
  "type": "STATUS",
  "payload": {
    "system": "emergency-stopped",
    "allMotionCeased": true,
    "materialFlowStopped": true
  }
}
```

## 7. Event Notifications

### 7.1 Event Types

| Event | Severity | Description |
|-------|----------|-------------|
| LAYER_STARTED | Info | New layer begun |
| LAYER_COMPLETED | Info | Layer finished |
| MATERIAL_LOW | Warning | Low inventory |
| QUALITY_ISSUE | Warning | Quality out of tolerance |
| EQUIPMENT_FAULT | Error | Equipment malfunction |
| SAFETY_VIOLATION | Critical | Safety trigger |

### 7.2 Event Format

```json
{
  "type": "EVENT",
  "payload": {
    "event": "LAYER_COMPLETED",
    "severity": "info",
    "data": {
      "layerNumber": 25,
      "duration": 660.123,
      "materialUsed": 125.5,
      "quality": {"overallScore": 96.3}
    }
  }
}
```

## 8. Connection Management

### 8.1 Heartbeat

```json
{
  "type": "HEARTBEAT",
  "timestamp": "2025-03-12T10:30:46.123Z",
  "sequenceNumber": 20001
}
```

**Server Response:**
```json
{
  "type": "HEARTBEAT",
  "timestamp": "2025-03-12T10:30:46.125Z",
  "payload": {
    "latency": 2.0,
    "serverLoad": 0.35
  }
}
```

### 8.2 Reconnection

**Exponential Backoff:**
- Attempt 1: 1 second delay
- Attempt 2: 2 seconds delay
- Attempt 3: 5 seconds delay
- Attempt 4+: 10 seconds delay
- Maximum 10 attempts

### 8.3 State Resynchronization

```json
{
  "type": "COMMAND",
  "payload": {"command": "SYNC_STATE"}
}
```

**Response:**
```json
{
  "type": "STATUS",
  "payload": {
    "fullState": {
      "job": {...},
      "robot": {...},
      "material": {...},
      "safety": {...}
    }
  }
}
```

## 9. Quality of Service

### 9.1 QoS Levels

| Message Type | QoS | Guarantee |
|--------------|-----|-----------|
| EMERGENCY_STOP | Highest | Guaranteed, ordered, acknowledged |
| COMMAND | High | Guaranteed, acknowledged |
| STATUS | Medium | Best effort, latest value |
| SENSOR | Low | Best effort, drops okay |

### 9.2 Acknowledgment

```json
{
  "type": "ACK",
  "payload": {
    "acknowledgedSequence": 12346,
    "status": "accepted"
  }
}
```

## 10. Performance Optimization

### 10.1 Message Compression

```javascript
ws = new WebSocket('wss://...', {
    perMessageDeflate: true
});
```

### 10.2 Batching

```json
{
  "type": "SENSOR_BATCH",
  "payload": {
    "sensors": [
      {"sensor": "position", "data": {...}},
      {"sensor": "material-flow", "data": {...}}
    ]
  }
}
```

### 10.3 Delta Updates

```json
{
  "type": "STATUS_DELTA",
  "payload": {
    "changes": {
      "robot.position.x": 5125.8,
      "material.flowRate": 1005.2
    }
  }
}
```

## 11. Compliance Checklist

**Basic Compliance:**
- ☐ WebSocket support
- ☐ All message types
- ☐ Authentication
- ☐ Heartbeat mechanism

**Advanced Compliance:**
- ☐ Robot control
- ☐ Material flow management
- ☐ Sensor streaming
- ☐ Safety protocols
- ☐ Emergency stop (< 5ms)
- ☐ State resynchronization
- ☐ QoS levels

**Full Compliance (adds):**
- ☐ MQTT support
- ☐ gRPC support
- ☐ Message compression
- ☐ Delta updates
- ☐ Advanced optimization

---

**Document Information:**
- **Version:** 1.0.0
- **Status:** Published
- **Date:** 2025-01-15
- **Depends on:** Phase 1 v1.0.0, Phase 2 v1.0.0
- **License:** CC BY 4.0

**弘益人間 (Benefit All Humanity)**

© 2025 SmileStory Inc. / WIA

---

## Conformance Test Battery (Phase 3)

A platform claiming Phase 3 conformance demonstrates the following at commissioning and reproduces them on each periodic re-certification.

### Process Calibration Test

The platform documents the calibration of its primary processing operations (sorting, cleaning, sizing, recompounding, additive manufacturing where applicable) against the calibration material set declared in the manifest. Calibration drift between commissioning and re-certification is reported; sustained drift triggers process re-tuning before re-certification proceeds.

### Material Provenance Test

A sampled set of intake material lots is traced through the platform from intake to outbound product. The trace is reconstructed from audit records alone — without operator narration. A platform whose audit records do not support end-to-end reconstruction is non-conformant in provenance.

### Quality Reproducibility Test

Independent samples drawn from the same outbound product lot are tested for the declared quality attributes. Variance within the lot must be within the declared bound. Variance exceeding the bound is itself a finding; a deployment that has tightened its bound below what its process can deliver must re-tune the bound or re-tune the process.

### Energy & Resource Accounting

The deployment publishes per-lot energy and water consumption derived from sub-meter data. The accounting must reconcile against utility-meter data within the declared tolerance. Reconciliation failure is a non-conformance: an upcycling claim that does not survive its own resource accounting is unsupported.

### Inter-Standard Dependencies

Phase 3 depends on:

- WIA-SEC-017 (Security Audit) for tamper-evident operational logs.
- The WIA waste-management family for upstream material-classification interoperability.
- The WIA recycling family where the boundary between upcycling and recycling is process-dependent.

A break in any dependency invalidates the Phase 3 conformance claim until repaired.

### Reproducibility & Public Trust

A deployment's upcycling claim ultimately rests on whether independent verifiers can reproduce its quality results from its published methods. The conformance test battery is the path to that reproducibility; the lifecycle audit is the evidence of it. Together they sustain the public trust the WIA Upcycling claim is asking for.
