# WIA-AGRI-028: Aeroponics Standard
## Phase 3: Protocol Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-01-01

---

## 1. Overview

This specification defines communication protocols for aeroponic IoT devices, including MQTT messaging, WebSocket streams, and real-time control systems for mist-based agriculture.

### 1.1 Supported Protocols

- **MQTT v5.0**: Primary protocol for device-to-cloud communication
- **WebSocket**: Real-time bidirectional streaming
- **CoAP**: Lightweight protocol for constrained devices
- **HTTP/REST**: Standard API access

---

## 2. MQTT Protocol

### 2.1 Connection Parameters

```json
{
  "broker": "mqtt.aeroponics.wiastandards.com",
  "port": 8883,
  "protocol": "mqtts",
  "clientId": "aero-{systemId}-{chamberId}",
  "username": "{systemId}",
  "password": "{api_key}",
  "keepAlive": 60,
  "cleanSession": false,
  "qos": 1
}
```

### 2.2 Topic Structure

**Topic Pattern:**
```
aero/{systemId}/{component}/{action}
```

**Examples:**
```
aero/AERO-SEOUL-001/misting/status
aero/AERO-SEOUL-001/sensors/data
aero/AERO-SEOUL-001/nutrients/update
aero/AERO-SEOUL-001/alerts/critical
aero/AERO-SEOUL-001/control/command
```

### 2.3 Mist Cycle Messages

**Topic:** `aero/{systemId}/misting/cycle`

**Publish:** Device publishes after each mist cycle

```json
{
  "type": "mist_cycle_completed",
  "systemId": "AERO-SEOUL-001",
  "chamber": 3,
  "timestamp": "2025-01-01T14:30:00Z",
  "cycle": {
    "cycleId": "CYCLE-20250101-143000",
    "interval": 180,
    "duration": 5,
    "pressure": 85,
    "volume": 125,
    "dropletSize": 25,
    "nozzlesActive": 24,
    "success": true
  },
  "performance": {
    "coverage": 98.5,
    "efficiency": 97.8,
    "energyUsed": 0.15
  },
  "nextCycle": "2025-01-01T14:33:00Z"
}
```

### 2.4 Sensor Data Messages

**Topic:** `aero/{systemId}/sensors/data`

**Publish:** Real-time sensor readings (every 5 minutes)

```json
{
  "type": "sensor_update",
  "systemId": "AERO-SEOUL-001",
  "chamber": 3,
  "timestamp": "2025-01-01T14:30:00Z",
  "sensors": {
    "airTemperature": 22.5,
    "rootZoneTemp": 20.0,
    "humidity": 70,
    "oxygenLevel": 21.5,
    "co2Level": 1100,
    "ec": 1.8,
    "ph": 6.1,
    "pressure": 85,
    "dissolvedOxygen": 8.5
  },
  "status": "normal"
}
```

### 2.5 Control Commands

**Topic:** `aero/{systemId}/control/command`

**Subscribe:** Device receives control commands

```json
{
  "type": "control_command",
  "commandId": "CMD-20250101-143500",
  "command": "adjust_misting",
  "timestamp": "2025-01-01T14:35:00Z",
  "target": {
    "chamber": 3,
    "component": "misting_system"
  },
  "parameters": {
    "interval": 150,
    "duration": 6,
    "pressure": 90
  },
  "priority": "high",
  "expiry": "2025-01-01T15:00:00Z"
}
```

**Response Topic:** `aero/{systemId}/control/response`

```json
{
  "type": "command_response",
  "commandId": "CMD-20250101-143500",
  "status": "executed",
  "timestamp": "2025-01-01T14:35:05Z",
  "result": {
    "success": true,
    "appliedValues": {
      "interval": 150,
      "duration": 6,
      "pressure": 90
    },
    "nextCycle": "2025-01-01T14:37:30Z"
  }
}
```

### 2.6 Alert Messages

**Topic:** `aero/{systemId}/alerts/{severity}`

**Severities:** `info`, `warning`, `error`, `critical`

```json
{
  "type": "alert",
  "alertId": "ALERT-20250101-001",
  "severity": "warning",
  "timestamp": "2025-01-01T14:25:00Z",
  "systemId": "AERO-SEOUL-001",
  "chamber": 3,
  "category": "misting_pressure",
  "message": "Misting pressure below optimal range",
  "details": {
    "currentValue": 65,
    "threshold": 75,
    "unit": "PSI",
    "component": "high_pressure_pump"
  },
  "recommendations": [
    "Check pump performance",
    "Inspect pressure regulator",
    "Verify nozzle flow rates"
  ],
  "autoResolved": false
}
```

### 2.7 Nutrient Management Messages

**Topic:** `aero/{systemId}/nutrients/status`

```json
{
  "type": "nutrient_status",
  "systemId": "AERO-SEOUL-001",
  "timestamp": "2025-01-01T14:30:00Z",
  "solution": {
    "ec": 1.8,
    "ph": 6.1,
    "temperature": 20,
    "dissolvedOxygen": 8.5
  },
  "reservoir": {
    "level": 85,
    "volume": 850,
    "capacity": 1000,
    "daysRemaining": 12
  },
  "nutrients": {
    "nitrogen": 180,
    "phosphorus": 60,
    "potassium": 220
  },
  "action": "none"
}
```

---

## 3. WebSocket Protocol

### 3.1 Connection

**Endpoint:**
```
wss://stream.aeroponics.wiastandards.com/v1/stream
```

**Authentication:**
```json
{
  "type": "auth",
  "apiKey": "{API_KEY}",
  "systemId": "AERO-SEOUL-001"
}
```

### 3.2 Real-Time Streaming

**Subscribe to Events:**
```json
{
  "type": "subscribe",
  "channels": [
    "misting.cycles",
    "sensors.realtime",
    "alerts.all",
    "root_zone.monitoring"
  ]
}
```

**Streaming Data:**
```json
{
  "type": "stream",
  "channel": "sensors.realtime",
  "timestamp": "2025-01-01T14:30:00Z",
  "data": {
    "systemId": "AERO-SEOUL-001",
    "chamber": 3,
    "temperature": 22.5,
    "humidity": 70,
    "pressure": 85,
    "ec": 1.8,
    "ph": 6.1
  }
}
```

### 3.3 Heartbeat

```json
{
  "type": "ping",
  "timestamp": "2025-01-01T14:30:00Z"
}
```

**Response:**
```json
{
  "type": "pong",
  "timestamp": "2025-01-01T14:30:00Z",
  "latency": 15
}
```

---

## 4. CoAP Protocol

### 4.1 Endpoints

**Base URI:** `coap://coap.aeroponics.wiastandards.com:5683`

**Resources:**
```
/aero/{systemId}/misting
/aero/{systemId}/sensors
/aero/{systemId}/nutrients
/aero/{systemId}/status
```

### 4.2 GET Request Example

```
GET coap://coap.aeroponics.wiastandards.com:5683/aero/AERO-SEOUL-001/sensors
```

**Response:**
```json
{
  "systemId": "AERO-SEOUL-001",
  "temperature": 22.5,
  "humidity": 70,
  "ec": 1.8,
  "ph": 6.1,
  "pressure": 85
}
```

### 4.3 POST Request Example

```
POST coap://coap.aeroponics.wiastandards.com:5683/aero/AERO-SEOUL-001/misting
Content-Format: application/json

{
  "interval": 150,
  "duration": 6
}
```

---

## 5. Data Encoding

### 5.1 JSON (Default)

Standard UTF-8 encoded JSON for maximum compatibility.

### 5.2 MessagePack

Compact binary format for bandwidth-constrained devices.

```python
import msgpack

# Encode
data = {"temperature": 22.5, "humidity": 70}
packed = msgpack.packb(data)

# Decode
unpacked = msgpack.unpackb(packed)
```

### 5.3 Protocol Buffers

High-performance serialization for high-frequency data.

```protobuf
syntax = "proto3";

message SensorData {
  string system_id = 1;
  int32 chamber = 2;
  double temperature = 3;
  double humidity = 4;
  double ec = 5;
  double ph = 6;
  int32 pressure = 7;
  int64 timestamp = 8;
}
```

---

## 6. Security

### 6.1 TLS/SSL

All connections MUST use TLS 1.3 or higher.

**MQTT:** Port 8883 (MQTTS)
**WebSocket:** WSS (TLS)
**CoAP:** DTLS

### 6.2 Authentication

**API Key:**
```
X-API-Key: {API_KEY}
```

**Token Authentication:**
```
Authorization: Bearer {JWT_TOKEN}
```

### 6.3 Message Integrity

All messages include HMAC-SHA256 signature:

```json
{
  "data": {...},
  "signature": "3a5f8b9c2d...",
  "timestamp": "2025-01-01T14:30:00Z"
}
```

---

## 7. QoS Levels

### 7.1 MQTT QoS

| Level | Description | Use Case |
|-------|-------------|----------|
| 0 | At most once | Non-critical sensors |
| 1 | At least once | Standard telemetry |
| 2 | Exactly once | Critical commands |

### 7.2 Message Priorities

**Critical:** Misting failures, pump errors
**High:** Nutrient alerts, pressure warnings
**Medium:** Sensor updates, cycle completions
**Low:** Analytics, statistics

---

## 8. Offline & Recovery

### 8.1 Buffering

Devices MUST buffer messages during offline periods:
- **Max Buffer Size:** 1000 messages
- **Max Buffer Time:** 24 hours
- **Priority:** Critical messages first

### 8.2 Reconnection Strategy

```javascript
const reconnect = {
  initialDelay: 1000,      // 1 second
  maxDelay: 60000,         // 1 minute
  multiplier: 2,           // Exponential backoff
  maxAttempts: 10
};
```

### 8.3 State Synchronization

After reconnection:
1. Publish current system state
2. Request missed commands
3. Resume normal operations

---

## 9. Integration Examples

### 9.1 MQTT Client (JavaScript)

```javascript
const mqtt = require('mqtt');

const client = mqtt.connect('mqtts://mqtt.aeroponics.wiastandards.com:8883', {
  clientId: 'aero-AERO-SEOUL-001-3',
  username: 'AERO-SEOUL-001',
  password: process.env.API_KEY,
  qos: 1
});

// Subscribe to control commands
client.subscribe('aero/AERO-SEOUL-001/control/command');

// Publish mist cycle completion
client.publish('aero/AERO-SEOUL-001/misting/cycle', JSON.stringify({
  type: 'mist_cycle_completed',
  systemId: 'AERO-SEOUL-001',
  chamber: 3,
  cycle: {
    duration: 5,
    pressure: 85,
    success: true
  }
}));

// Handle incoming commands
client.on('message', (topic, message) => {
  const command = JSON.parse(message.toString());
  handleControlCommand(command);
});
```

### 9.2 WebSocket Client (Python)

```python
import asyncio
import websockets
import json

async def stream_sensors():
    uri = "wss://stream.aeroponics.wiastandards.com/v1/stream"

    async with websockets.connect(uri) as websocket:
        # Authenticate
        await websocket.send(json.dumps({
            "type": "auth",
            "apiKey": os.environ['API_KEY'],
            "systemId": "AERO-SEOUL-001"
        }))

        # Subscribe
        await websocket.send(json.dumps({
            "type": "subscribe",
            "channels": ["sensors.realtime"]
        }))

        # Stream data
        while True:
            data = await websocket.recv()
            message = json.loads(data)
            print(f"Sensor update: {message}")

asyncio.run(stream_sensors())
```

---

## 10. Best Practices

1. **Use QoS 1** for standard telemetry (at-least-once delivery)
2. **Use QoS 2** for critical commands (exactly-once delivery)
3. **Enable TLS/SSL** for all connections
4. **Implement reconnection** with exponential backoff
5. **Buffer messages** during offline periods
6. **Validate timestamps** to detect replay attacks
7. **Monitor latency** and adjust accordingly
8. **Use retained messages** for system state
9. **Implement heartbeat** for connection health
10. **Log all protocol errors** for debugging

---

**Document History:**
- v1.0.0 (2025-01-01): Initial release

© 2025 WIA Standards · MIT License
弘益人間 (Benefit All Humanity)
