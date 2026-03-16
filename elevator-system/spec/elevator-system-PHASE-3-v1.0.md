# WIA Elevator System Standard
## Phase 3: Safety & Communication Protocols v1.0

**Status:** APPROVED  
**Date:** 2025-12-26  
**弘益人間** · Benefit All Humanity

---

## 1. Introduction

Phase 3 defines safety interlocks, communication protocols, and emergency procedures for WIA-compliant elevator systems. This ensures EN 81-20/50 compliance, secure data transmission, and reliable emergency response.

### 1.1 Scope

- Digital safety protocols
- MQTT messaging
- CoAP for IoT devices
- TLS/DTLS encryption
- Emergency communication
- Fire service protocols
- Certificate management

---

## 2. Safety Interlock Protocols

### 2.1 Door Safety Interlock

**Digital Safety Message:**
```json
{
  "type": "SAFETY_INTERLOCK",
  "component": "DOOR",
  "timestamp": "2025-12-26T10:30:00Z",
  "checks": {
    "doorLockVerified": true,
    "obstacleDetection": "CLEAR",
    "reopenSensor": "ACTIVE",
    "forceLimit": "WITHIN_SPEC"
  },
  "result": "SAFE_TO_OPERATE"
}
```

**Safety Rules:**
1. Elevator MUST NOT move if `doorLockVerified = false`
2. Doors MUST reopen if `obstacleDetection = OBSTRUCTION`
3. Force limit MUST NOT exceed 150N (EN 81-20 clause 5.3.6.2)

### 2.2 Overload Protection

```json
{
  "type": "SAFETY_INTERLOCK",
  "component": "LOAD",
  "maxWeight": 1000,
  "currentWeight": 1050,
  "overloadThreshold": 950,
  "result": "OVERLOAD_DETECTED",
  "action": "PREVENT_DOOR_CLOSE"
}
```

### 2.3 Overspeed Governor

```json
{
  "type": "SAFETY_INTERLOCK",
  "component": "OVERSPEED",
  "ratedSpeed": 2.5,
  "currentSpeed": 2.8,
  "tripSpeed": 3.0,
  "result": "APPROACHING_TRIP",
  "action": "REDUCE_SPEED"
}
```

---

## 3. MQTT Protocol Specification

### 3.1 Topic Structure

```
wia/elevator/{buildingId}/{elevatorId}/{category}/{subcategory}
```

**Examples:**
- `wia/elevator/BLD-2025/ELV-001/status` - Status updates
- `wia/elevator/BLD-2025/ELV-001/telemetry/sensors` - Sensor data
- `wia/elevator/BLD-2025/dispatch/request` - Dispatch requests
- `wia/elevator/BLD-2025/ELV-001/maintenance/alert` - Maintenance alerts

### 3.2 QoS Levels

- **QoS 0 (At most once):** Telemetry data (non-critical)
- **QoS 1 (At least once):** Status updates, events
- **QoS 2 (Exactly once):** Control commands, safety messages

### 3.3 Message Format

**Status Update:**
```json
{
  "topic": "wia/elevator/BLD-2025/ELV-001/status",
  "qos": 1,
  "retain": true,
  "payload": {
    "elevatorId": "ELV-001",
    "timestamp": "2025-12-26T10:30:00Z",
    "status": {
      "currentFloor": 5,
      "direction": "UP",
      "doorStatus": "CLOSED"
    }
  }
}
```

**Control Command:**
```json
{
  "topic": "wia/elevator/BLD-2025/ELV-001/command",
  "qos": 2,
  "payload": {
    "commandId": "CMD-12345",
    "command": "GO_TO_FLOOR",
    "parameters": {
      "targetFloor": 10
    },
    "timestamp": "2025-12-26T10:30:05Z"
  }
}
```

### 3.4 Last Will and Testament (LWT)

Configure LWT for offline detection:
```json
{
  "topic": "wia/elevator/BLD-2025/ELV-001/status/lwt",
  "qos": 1,
  "retain": true,
  "message": {
    "elevatorId": "ELV-001",
    "status": "OFFLINE",
    "timestamp": "2025-12-26T10:30:00Z"
  }
}
```

---

## 4. CoAP Protocol for IoT Sensors

### 4.1 CoAP URIs

```
coap://elevator.local/status
coap://elevator.local/sensors/load
coap://elevator.local/sensors/temperature
```

### 4.2 CoAP Methods

- **GET:** Retrieve sensor data
- **POST:** Update configuration
- **PUT:** Full resource update
- **OBSERVE:** Subscribe to changes

### 4.3 Example Request/Response

**GET Request:**
```
GET coap://elevator.local/sensors/load
Accept: application/json
```

**Response:**
```
2.05 Content
Content-Format: application/json

{
  "sensorId": "LOAD-001",
  "elevatorId": "ELV-001",
  "weight": 640,
  "unit": "kg",
  "timestamp": "2025-12-26T10:30:00Z"
}
```

### 4.4 DTLS Security

All CoAP communications MUST use DTLS 1.3:
```
coaps://elevator.local/sensors/load
```

---

## 5. TLS/DTLS Encryption

### 5.1 TLS Requirements

- **Minimum version:** TLS 1.3
- **Cipher suites:** 
  - `TLS_AES_256_GCM_SHA384`
  - `TLS_CHACHA20_POLY1305_SHA256`
- **Certificate validation:** REQUIRED
- **Mutual TLS:** RECOMMENDED

### 5.2 Certificate Requirements

**X.509 v3 Certificate:**
```
Subject: CN=ELV-001, O=Building Management, C=US
Issuer: CN=WIA Certificate Authority
Valid From: 2025-01-01T00:00:00Z
Valid To: 2027-01-01T00:00:00Z
Key Usage: Digital Signature, Key Encipherment
Extended Key Usage: Server Authentication, Client Authentication
```

### 5.3 Certificate Pinning

Pin WIA root CA certificate:
```
SHA256:a1b2c3d4e5f6...
```

---

## 6. Emergency Communication Protocols

### 6.1 Emergency Stop

**Emergency stop message (QoS 2):**
```json
{
  "type": "EMERGENCY_STOP",
  "elevatorId": "ELV-001",
  "timestamp": "2025-12-26T10:31:00Z",
  "triggeredBy": "PASSENGER_BUTTON",
  "location": {
    "floor": 7,
    "position": 21.5
  },
  "actions": [
    "STOP_MOVEMENT",
    "ACTIVATE_ALARM",
    "NOTIFY_EMERGENCY_SERVICES",
    "ENABLE_TWO_WAY_COMM"
  ]
}
```

### 6.2 Two-Way Emergency Communication

**Protocol:** VoIP over TLS
```json
{
  "type": "EMERGENCY_CALL",
  "elevatorId": "ELV-001",
  "callId": "EMERG-2025-001",
  "codec": "G.711",
  "sampleRate": 8000,
  "videoEnabled": true,
  "recipientList": [
    "emergency-center@building.com",
    "security@building.com"
  ]
}
```

---

## 7. Fire Service Operation Mode

### 7.1 Fire Alarm Integration

**Fire alarm trigger:**
```json
{
  "type": "FIRE_ALARM_ACTIVATION",
  "buildingId": "BLD-2025",
  "timestamp": "2025-12-26T10:35:00Z",
  "alarmZone": "FLOOR-5",
  "severity": "CRITICAL",
  "elevatorActions": {
    "allElevators": "RECALL_TO_DESIGNATED_FLOOR",
    "recallFloor": 1,
    "disableNormalOperation": true,
    "enableFiremanService": true
  }
}
```

### 7.2 Fireman Service Control

**Digital fireman interface:**
```json
{
  "mode": "FIREMAN_SERVICE_PHASE_II",
  "elevatorId": "ELV-001",
  "controlMethod": "DIGITAL_PANEL",
  "authentication": {
    "keyType": "FIRE_DEPARTMENT_KEY",
    "keyId": "FD-KEY-2025-123"
  },
  "manualControl": {
    "targetFloor": 5,
    "doorControl": "MANUAL",
    "carLightsOn": true
  }
}
```

---

## 8. Earthquake Protocol

### 8.1 Seismic Sensor Integration

**Earthquake detection:**
```json
{
  "type": "SEISMIC_EVENT",
  "buildingId": "BLD-2025",
  "timestamp": "2025-12-26T10:40:00Z",
  "magnitude": 5.2,
  "intensity": "MODERATE",
  "elevatorActions": {
    "allElevators": "STOP_AT_NEAREST_FLOOR",
    "openDoors": true,
    "disableOperation": true,
    "awaitInspection": true
  }
}
```

---

## 9. Compliance Testing

### 9.1 Safety Interlock Tests

1. ✅ Door interlock prevents movement
2. ✅ Overload prevents door closing
3. ✅ Overspeed triggers emergency brake
4. ✅ Emergency stop halts operation

### 9.2 Protocol Tests

1. ✅ MQTT QoS levels enforced
2. ✅ TLS 1.3 encryption verified
3. ✅ Certificate validation working
4. ✅ Emergency communication functional

---

**© 2025 WIA - World Certification Industry Association**  
**MIT License**  
**弘益人間 · Benefit All Humanity**
