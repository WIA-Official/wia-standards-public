# WIA-FARMING_ROBOT: PHASE 3 - Protocol Specification

**Version:** 1.0
**Status:** Active
**Last Updated:** 2026-01-12

---

## Overview

This specification defines communication protocols, safety procedures, and operational workflows for agricultural robotics systems to ensure safe, reliable, and efficient operation.

---

## 1. Communication Protocols

### 1.1 Primary Protocol: MQTT

**Use Case:** Telemetry, status updates, sensor data

```
Topic Structure:
wia/farming/{farmId}/{robotId}/{messageType}

Examples:
wia/farming/farm-001/robot-550e/telemetry
wia/farming/farm-001/robot-550e/status
wia/farming/farm-001/robot-550e/alerts
```

**QoS Levels:**
- QoS 0: Telemetry data (high frequency, loss acceptable)
- QoS 1: Status updates (at least once delivery)
- QoS 2: Control commands (exactly once delivery)

**Message Format:**
```json
{
  "timestamp": "2026-01-12T10:30:00.123Z",
  "robotId": "550e8400-e29b-41d4-a716-446655440000",
  "messageType": "TELEMETRY",
  "payload": {
    "position": {...},
    "status": "WORKING",
    "battery": 87.5
  },
  "signature": "sha256_hash"
}
```

### 1.2 Secondary Protocol: WebSocket

**Use Case:** Real-time control, bi-directional communication

**Connection URL:**
```
wss://realtime.farming-robot.wia.org/v1/robots/{robotId}
```

**Heartbeat:** Every 5 seconds
**Timeout:** 30 seconds without heartbeat = connection lost

### 1.3 Tertiary Protocol: HTTP/2

**Use Case:** Configuration updates, firmware downloads, bulk data transfer

**Features:**
- Server push for firmware updates
- Multiplexing for parallel requests
- Header compression (HPACK)

---

## 2. Safety Protocols

### 2.1 Emergency Stop (E-STOP)

**Trigger Conditions:**
1. Human detected within 5-meter safety zone
2. Obstacle detection failure
3. Manual emergency button pressed
4. Communication loss >30 seconds
5. Critical system failure
6. Path deviation >2 meters

**E-STOP Procedure:**
```
1. IMMEDIATE: Cut power to all actuators (within 100ms)
2. ACTIVATE: Engage mechanical brakes
3. BROADCAST: Send E-STOP alert to all systems
4. LOG: Record event with full sensor state
5. AWAIT: Manual inspection and reset required
```

**E-STOP Message:**
```json
{
  "type": "EMERGENCY_STOP",
  "robotId": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": "2026-01-12T10:30:01.234Z",
  "reason": "HUMAN_DETECTED",
  "position": {"lat": 42.3601, "lon": -71.0589},
  "severity": "CRITICAL",
  "sensorData": {...}
}
```

### 2.2 Collision Avoidance

**Detection Range:**
- Primary: LiDAR 0-50 meters
- Secondary: Camera vision 0-100 meters
- Tertiary: Radar 0-200 meters

**Response Times:**
- 0-5m: Emergency stop (100ms)
- 5-15m: Slow to 50% speed (500ms)
- 15-30m: Slow to 75% speed (1000ms)
- 30-50m: Monitor and alert

**Obstacle Classification:**
```typescript
interface ObstacleResponse {
  type: ObstacleType;
  distance: number;
  action: "STOP" | "SLOW" | "AVOID" | "MONITOR";
  avoidancePath?: PathSegment[];
  estimatedImpactTime?: number;
}
```

### 2.3 Geofencing

**Virtual Boundaries:**
```json
{
  "fieldId": "field-001",
  "boundary": {
    "type": "Polygon",
    "coordinates": [...]
  },
  "bufferZone": 3.0,
  "action": "SOFT_STOP",
  "alerts": ["OPERATOR", "FLEET_MANAGER"]
}
```

**Boundary Violations:**
- **Approach (buffer zone):** Warning + slow to 50%
- **Breach:** Auto-stop + alert + log
- **Override:** Requires operator authorization

---

## 3. Task Execution Protocol

### 3.1 Task Lifecycle

```
[CREATED] → [VALIDATED] → [SCHEDULED] → [DISPATCHED] →
[ACCEPTED] → [IN_PROGRESS] → [PAUSED] → [RESUMED] →
[COMPLETED] | [FAILED] | [CANCELLED]
```

### 3.2 Task Validation Checklist

```typescript
interface TaskValidation {
  robotAvailable: boolean;
  batteryLevel: number;          // Must be >20% + task requirement
  weatherConditions: boolean;    // Check rain, wind, temperature
  fieldAccessible: boolean;
  pathPlanValid: boolean;
  equipmentReady: boolean;       // Seed, fertilizer, etc.
  safetyChecks: boolean;
  estimatedTime: number;
}
```

### 3.3 Progress Reporting

**Frequency:** Every 10 seconds during active work

```json
{
  "taskId": "task-2026-001",
  "timestamp": "2026-01-12T14:15:30Z",
  "progress": {
    "completionPercentage": 67.5,
    "areaCovered": 27.5,
    "areaRemaining": 13.2,
    "timeElapsed": 2.5,
    "timeRemaining": 1.2,
    "currentSegment": 32,
    "totalSegments": 47
  },
  "performance": {
    "workRate": 6.2,
    "efficiency": 94.7,
    "fuelRemaining": 45.3
  }
}
```

---

## 4. Sensor Fusion Protocol

### 4.1 Multi-Sensor Integration

**Sensor Priority:**
1. **Primary:** RTK-GNSS (positioning)
2. **Secondary:** LiDAR (obstacle detection)
3. **Tertiary:** Camera (crop analysis)
4. **Quaternary:** Radar (long-range detection)
5. **Quinary:** IMU (orientation, acceleration)

**Fusion Algorithm:**
```
1. Acquire data from all sensors (synchronized)
2. Validate data quality (outlier detection)
3. Apply Kalman filter for position estimation
4. Cross-validate with multiple sensors
5. Confidence scoring (0-100%)
6. Decision making based on fused data
```

### 4.2 Data Synchronization

**Timestamp Requirements:**
- All sensors synchronized to GPS time
- Precision: Millisecond resolution
- Max drift: ±10ms

**Synchronization Message:**
```json
{
  "syncId": "sync-001",
  "timestamp": "2026-01-12T10:30:00.123Z",
  "sensors": {
    "lidar": {"offset": 2, "quality": 98.5},
    "camera1": {"offset": 5, "quality": 95.2},
    "camera2": {"offset": 3, "quality": 97.1},
    "radar": {"offset": 1, "quality": 99.0},
    "imu": {"offset": 0, "quality": 100.0}
  }
}
```

---

## 5. Fleet Coordination Protocol

### 5.1 Multi-Robot Cooperation

**Coordination Modes:**
- **INDEPENDENT:** Each robot operates separately
- **COORDINATED:** Robots avoid conflicts, share resources
- **COLLABORATIVE:** Robots work together on single task

**Conflict Resolution:**
```typescript
interface Conflict {
  type: "PATH_CROSSING" | "RESOURCE" | "FIELD_ACCESS";
  robots: string[];
  resolution: "PRIORITY" | "NEGOTIATION" | "SEQUENTIAL";
  winner?: string;
}

// Priority Rules:
// 1. Emergency > Normal operation
// 2. Time-sensitive > Regular
// 3. Higher efficiency robot gets priority
// 4. First-come-first-served
```

### 5.2 Resource Sharing

**Shared Resources:**
- Charging stations
- Refill stations (seed, fertilizer, water)
- Field access gates
- Data processing (cloud compute)

**Reservation Protocol:**
```json
{
  "resourceId": "charger-001",
  "requestedBy": "robot-550e",
  "requestTime": "2026-01-12T14:00:00Z",
  "duration": 3600,
  "priority": "MEDIUM",
  "status": "RESERVED"
}
```

---

## 6. Data Security Protocol

### 6.1 Authentication

**Robot Authentication:**
```
1. X.509 Certificate (installed at manufacturing)
2. JWT Token (renewed every 24 hours)
3. API Key (for API access)
4. Hardware Security Module (HSM) for key storage
```

**Message Signing:**
```javascript
const signature = HMAC_SHA256(
  payload + timestamp + nonce,
  secret_key
);
```

### 6.2 Encryption

**Transport Layer:**
- TLS 1.3 for HTTP/WebSocket
- DTLS 1.3 for UDP-based protocols

**Data at Rest:**
- AES-256-GCM for sensor logs
- Key rotation every 90 days

### 6.3 Audit Logging

**Logged Events:**
- All control commands
- Configuration changes
- Emergency stops
- Boundary violations
- Communication failures

```json
{
  "eventId": "evt-001",
  "timestamp": "2026-01-12T10:30:00Z",
  "robotId": "550e8400-e29b-41d4-a716-446655440000",
  "eventType": "CONTROL_COMMAND",
  "user": "operator-john",
  "action": "PAUSE_TASK",
  "taskId": "task-2026-001",
  "ipAddress": "192.168.1.100",
  "result": "SUCCESS"
}
```

---

## 7. Maintenance Protocol

### 7.1 Predictive Maintenance

**Monitored Parameters:**
- Engine hours
- Distance traveled
- Vibration patterns
- Temperature anomalies
- Oil quality (for ICE engines)
- Battery health (for electric)

**Alert Thresholds:**
```typescript
interface MaintenanceAlert {
  component: string;
  severity: "INFO" | "WARNING" | "CRITICAL";
  metric: string;
  currentValue: number;
  threshold: number;
  recommendedAction: string;
  estimatedTimeToFailure?: number;
}
```

### 7.2 Scheduled Maintenance

```json
{
  "maintenanceSchedule": {
    "daily": ["Battery check", "Visual inspection"],
    "weekly": ["Sensor calibration", "Filter replacement"],
    "monthly": ["Oil change", "Tire pressure"],
    "quarterly": ["Full system diagnostic", "Firmware update"],
    "annual": ["Certification renewal", "Hardware upgrade"]
  }
}
```

---

## 8. Weather Response Protocol

### 8.1 Weather Monitoring

**Data Sources:**
- Local weather station
- National weather service API
- On-board sensors (temperature, humidity, wind)

**Critical Thresholds:**
```typescript
interface WeatherLimits {
  windSpeed: 40;           // km/h
  rainfall: 5;             // mm/h
  temperature: [-5, 45];   // Celsius
  visibility: 100;         // meters
  soilMoisture: [20, 80];  // percentage
}
```

### 8.2 Weather Response Actions

```
IF wind > 40 km/h THEN
  PAUSE spraying operations
  CONTINUE harvesting (with caution)
END IF

IF rain > 5 mm/h THEN
  PAUSE all field operations
  RETURN to shelter
END IF

IF temperature < -5°C OR > 45°C THEN
  STOP all operations
  PROTECT equipment
END IF
```

---

## Protocol Version Management

**Current Version:** 1.0
**Backward Compatibility:** 2 major versions
**Deprecation Notice:** 6 months minimum

**Version Negotiation:**
```json
{
  "supportedVersions": ["1.0", "0.9", "0.8"],
  "preferredVersion": "1.0",
  "requiredFeatures": ["RTK", "LiDAR", "E-STOP"]
}
```

---

**弘益人間 (Benefit All Humanity)**
© 2026 WIA (World Industry Association)
