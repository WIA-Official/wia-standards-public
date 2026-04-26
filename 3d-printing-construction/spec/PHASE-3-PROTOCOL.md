# WIA 3D Printing Construction Standard
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

## Annex E — Implementation Notes for PHASE-3-PROTOCOL

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-3-PROTOCOL.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-3-PROTOCOL. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-3-protocol/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-3-PROTOCOL with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-3-PROTOCOL does not require bespoke
auditor tooling.

---

**Document Information:**
- **Version:** 1.0.0
- **Status:** Published
- **Date:** 2025-01-15
- **Depends on:** Phase 1 v1.0.0, Phase 2 v1.0.0
- **License:** CC BY 4.0

**弘益人間 (Benefit All Humanity)**

© 2025 SmileStory Inc. / WIA
