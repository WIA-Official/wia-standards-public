# WIA-AGRI-018: Vertical Farming Standard
## Phase 3: Protocol Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-01-01

---

## 1. Overview

This specification defines communication protocols for vertical farming IoT devices, including MQTT messaging, WebSocket streaming, CoAP lightweight protocol, and OPC UA for industrial integration.

### 1.1 Design Principles

- **Real-time**: Sub-second latency for critical controls
- **Reliable**: Message delivery guarantees for safety-critical operations
- **Lightweight**: Optimized for resource-constrained IoT devices
- **Secure**: End-to-end encryption and device authentication
- **Interoperable**: Standards-based protocols (MQTT, WebSocket, CoAP, OPC UA)

---

## 2. MQTT Protocol

### 2.1 MQTT Configuration

**Broker Settings:**
```
Host: mqtt.verticalfarm.io
Port: 8883 (TLS/SSL)
Port: 1883 (Non-TLS, dev only)
Protocol: MQTT 3.1.1 / MQTT 5.0
QoS: 0, 1, or 2 (configurable)
Keep-Alive: 60 seconds
```

**Authentication:**
```
Username: farmId (e.g., VF-SEOUL-001)
Password: Device-specific API token
Client ID: {farmId}-{deviceType}-{deviceId}
```

### 2.2 Topic Structure

**Hierarchical Topic Naming:**
```
farm/{farmId}/tier/{tierNum}/sensor/{sensorType}
farm/{farmId}/tier/{tierNum}/actuator/{actuatorType}
farm/{farmId}/tier/{tierNum}/alert/{alertType}
farm/{farmId}/system/{systemType}
farm/{farmId}/analytics/{metricType}
```

**Examples:**
```
farm/VF-SEOUL-001/tier/3/sensor/temperature
farm/VF-SEOUL-001/tier/3/sensor/humidity
farm/VF-SEOUL-001/tier/3/actuator/hvac
farm/VF-SEOUL-001/tier/3/alert/critical
farm/VF-SEOUL-001/system/power
```

### 2.3 Sensor Data Publishing

**Topic:** `farm/{farmId}/tier/{tierNum}/sensor/{sensorType}`

**Payload (JSON):**
```json
{
  "deviceId": "TEMP-SENSOR-T3-01",
  "timestamp": "2025-01-01T10:30:45.123Z",
  "value": 22.5,
  "unit": "celsius",
  "status": "NORMAL",
  "quality": 100,
  "battery": 85
}
```

**Publishing Frequency:**
- Temperature/Humidity: Every 30 seconds
- CO2: Every 60 seconds
- pH/EC: Every 120 seconds
- Light sensors: Every 60 seconds

**QoS Levels:**
- QoS 0: Non-critical metrics (light intensity)
- QoS 1: Standard sensors (temperature, humidity)
- QoS 2: Critical safety data (pH, EC, alerts)

### 2.4 Control Commands

**Topic:** `farm/{farmId}/tier/{tierNum}/actuator/{actuatorType}/cmd`

**Payload Example - HVAC Control:**
```json
{
  "commandId": "CMD-2025-001",
  "timestamp": "2025-01-01T10:35:00Z",
  "action": "SET_TEMPERATURE",
  "parameters": {
    "targetTemperature": 23.0,
    "rampRate": 0.5,
    "priority": "HIGH"
  },
  "requester": "automation-controller",
  "expiresAt": "2025-01-01T10:40:00Z"
}
```

**Response Topic:** `farm/{farmId}/tier/{tierNum}/actuator/{actuatorType}/status`

**Response Payload:**
```json
{
  "commandId": "CMD-2025-001",
  "timestamp": "2025-01-01T10:35:05Z",
  "status": "EXECUTING",
  "currentTemperature": 22.5,
  "targetTemperature": 23.0,
  "estimatedCompletion": "2025-01-01T10:45:00Z",
  "error": null
}
```

### 2.5 Alert Publishing

**Topic:** `farm/{farmId}/tier/{tierNum}/alert/{severity}`

**Payload:**
```json
{
  "alertId": "ALERT-2025-001",
  "timestamp": "2025-01-01T14:22:15Z",
  "severity": "CRITICAL",
  "category": "ENVIRONMENTAL",
  "type": "TEMPERATURE_HIGH",
  "message": "Temperature exceeding safe limits",
  "tier": 3,
  "zone": "A",
  "currentValue": 28.5,
  "threshold": 27.0,
  "affectedBatches": ["BATCH-2025-001"],
  "recommendations": [
    "Increase cooling immediately",
    "Check HVAC system",
    "Reduce LED intensity"
  ],
  "autoActions": ["HVAC_MAX_COOLING"],
  "requiresAcknowledgment": true
}
```

**Severity Levels:**
- `info` - Informational messages
- `warning` - Attention needed
- `critical` - Immediate action required
- `emergency` - System failure, crop at risk

### 2.6 Last Will and Testament (LWT)

**Configuration:**
```json
{
  "topic": "farm/VF-SEOUL-001/tier/3/sensor/temperature/status",
  "payload": {
    "deviceId": "TEMP-SENSOR-T3-01",
    "status": "OFFLINE",
    "timestamp": "2025-01-01T10:30:00Z"
  },
  "qos": 1,
  "retain": true
}
```

**Purpose:** Notify system when device disconnects unexpectedly

---

## 3. WebSocket Protocol

### 3.1 Connection

**Endpoint:** `wss://stream.verticalfarm.io/v1/ws`

**Authentication:**
```javascript
const ws = new WebSocket('wss://stream.verticalfarm.io/v1/ws');

ws.onopen = () => {
  ws.send(JSON.stringify({
    type: 'authenticate',
    token: 'your_access_token',
    farmId: 'VF-SEOUL-001'
  }));
};
```

### 3.2 Subscribe to Data Streams

**Subscribe Message:**
```json
{
  "type": "subscribe",
  "streams": [
    {
      "farmId": "VF-SEOUL-001",
      "tier": 3,
      "dataType": "environment",
      "parameters": ["temperature", "humidity", "co2"]
    },
    {
      "farmId": "VF-SEOUL-001",
      "tier": 3,
      "dataType": "alerts",
      "severity": ["WARNING", "CRITICAL"]
    }
  ]
}
```

**Subscription Confirmation:**
```json
{
  "type": "subscribed",
  "streamId": "STREAM-001",
  "message": "Successfully subscribed to 2 data streams"
}
```

### 3.3 Real-time Data Messages

**Environmental Update:**
```json
{
  "type": "environment_update",
  "streamId": "STREAM-001",
  "timestamp": "2025-01-01T10:30:45.123Z",
  "farmId": "VF-SEOUL-001",
  "tier": 3,
  "data": {
    "temperature": 22.5,
    "humidity": 65,
    "co2": 1200
  }
}
```

**Alert Notification:**
```json
{
  "type": "alert",
  "streamId": "STREAM-001",
  "timestamp": "2025-01-01T14:22:15Z",
  "severity": "CRITICAL",
  "alertId": "ALERT-2025-001",
  "message": "Temperature exceeding safe limits",
  "data": {
    "tier": 3,
    "currentValue": 28.5,
    "threshold": 27.0
  }
}
```

### 3.4 Heartbeat & Keep-Alive

**Client Ping (every 30 seconds):**
```json
{
  "type": "ping",
  "timestamp": "2025-01-01T10:30:00Z"
}
```

**Server Pong:**
```json
{
  "type": "pong",
  "timestamp": "2025-01-01T10:30:00Z",
  "serverTime": "2025-01-01T10:30:00.123Z"
}
```

---

## 4. CoAP Protocol (Lightweight IoT)

### 4.1 CoAP Overview

**Use Case:** Battery-powered sensors with limited bandwidth

**Endpoint:** `coap://coap.verticalfarm.io:5683`

**Security:** DTLS (CoAP over DTLS on port 5684)

### 4.2 Resource URIs

**Sensor Reading:**
```
coap://coap.verticalfarm.io/farm/VF-SEOUL-001/tier/3/temp
```

**CoAP Request:**
```
GET coap://coap.verticalfarm.io/farm/VF-SEOUL-001/tier/3/temp
Accept: application/json
```

**CoAP Response:**
```
2.05 Content
Content-Format: application/json

{
  "v": 22.5,
  "u": "C",
  "t": 1640995845
}
```

**Compact Payload (CBOR):**
```cbor
{
  "v": 22.5,  // value
  "u": "C",   // unit
  "t": 1640995845,  // timestamp
  "b": 85,    // battery %
  "q": 100    // quality %
}
```

### 4.3 Observe Pattern (Pub/Sub)

**Observe Request:**
```
GET coap://coap.verticalfarm.io/farm/VF-SEOUL-001/tier/3/temp
Observe: 0
```

**Periodic Notifications:**
Server sends updates when temperature changes by >0.5°C or every 60 seconds.

```
2.05 Content
Observe: 1
Content-Format: application/json

{"v": 22.5, "t": 1640995845}
```

---

## 5. OPC UA Protocol (Industrial Integration)

### 5.1 OPC UA Overview

**Use Case:** Integration with industrial building management systems (BMS)

**Endpoint:** `opc.tcp://opcua.verticalfarm.io:4840`

**Security Mode:** Sign & Encrypt (recommended)

### 5.2 Information Model

**Node Structure:**
```
Root
└── VF-SEOUL-001 (FarmObject)
    ├── Infrastructure
    │   ├── TotalArea
    │   ├── NumberOfTiers
    │   └── Capacity
    ├── Tier3 (TierObject)
    │   ├── Environment
    │   │   ├── Temperature (AnalogItem)
    │   │   ├── Humidity (AnalogItem)
    │   │   └── CO2 (AnalogItem)
    │   ├── Lighting
    │   │   ├── Intensity (AnalogItem)
    │   │   └── Photoperiod (TwoStateVariable)
    │   └── WaterSystem
    │       ├── pH (AnalogItem)
    │       ├── EC (AnalogItem)
    │       └── FlowRate (AnalogItem)
    └── Alerts (FolderType)
        └── ActiveAlerts (BaseEventType[])
```

### 5.3 Reading Values

**Read Temperature:**
```
NodeId: ns=2;s=VF-SEOUL-001.Tier3.Environment.Temperature
```

**Returned Value:**
```
Value: 22.5
Quality: Good
Timestamp: 2025-01-01T10:30:45.123Z
```

### 5.4 Subscriptions

**Create Monitored Item:**
```
NodeId: ns=2;s=VF-SEOUL-001.Tier3.Environment.Temperature
SamplingInterval: 1000ms (1 second)
QueueSize: 10
DiscardOldest: true
```

**Data Change Notification:**
```
MonitoredItemId: 1234
Value: 22.6
Quality: Good
SourceTimestamp: 2025-01-01T10:31:00.456Z
```

### 5.5 Methods (Control Commands)

**Set Temperature Setpoint:**
```
Object: ns=2;s=VF-SEOUL-001.Tier3.Environment
Method: SetTemperatureSetpoint
Input: [23.0, 1.5]  // [targetTemp, tolerance]
Output: [true, "Setpoint updated successfully"]
```

---

## 6. HTTP/REST Protocol

### 6.1 Device Registration

**Endpoint:** `POST /api/v1/devices/register`

**Request:**
```json
{
  "farmId": "VF-SEOUL-001",
  "deviceType": "TEMPERATURE_SENSOR",
  "deviceId": "TEMP-T3-01",
  "tier": 3,
  "zone": "A",
  "manufacturer": "SensorTech Inc",
  "model": "ST-TEMP-200",
  "firmwareVersion": "1.2.5"
}
```

**Response:**
```json
{
  "success": true,
  "deviceToken": "device_token_here",
  "mqttConfig": {
    "broker": "mqtt.verticalfarm.io",
    "port": 8883,
    "clientId": "VF-SEOUL-001-TEMP-T3-01",
    "topics": {
      "publish": "farm/VF-SEOUL-001/tier/3/sensor/temperature",
      "subscribe": "farm/VF-SEOUL-001/tier/3/sensor/temperature/cmd"
    }
  }
}
```

### 6.2 Device Heartbeat

**Endpoint:** `POST /api/v1/devices/heartbeat`

**Request:**
```json
{
  "deviceId": "TEMP-T3-01",
  "timestamp": "2025-01-01T10:30:00Z",
  "status": "ONLINE",
  "battery": 85,
  "signalStrength": -65,
  "memoryUsage": 45,
  "uptime": 345600
}
```

---

## 7. Message Encoding

### 7.1 JSON (Default)

**Advantages:**
- Human-readable
- Wide support
- Easy debugging

**Disadvantages:**
- Larger payload size
- Higher parsing overhead

### 7.2 CBOR (Compact Binary)

**Use Case:** Battery-powered sensors, low-bandwidth networks

**Advantages:**
- 40-60% smaller than JSON
- Fast encoding/decoding
- Binary efficiency

**Example:**
```
JSON: {"v":22.5,"u":"C","t":1640995845} (38 bytes)
CBOR: A36176F94640617561436174... (24 bytes)
```

### 7.3 Protocol Buffers

**Use Case:** High-frequency data streams

**Schema Definition (.proto):**
```protobuf
message SensorReading {
  string device_id = 1;
  int64 timestamp = 2;
  float value = 3;
  string unit = 4;
  SensorStatus status = 5;
}

enum SensorStatus {
  NORMAL = 0;
  WARNING = 1;
  CRITICAL = 2;
  ERROR = 3;
}
```

---

## 8. Security Protocols

### 8.1 TLS/SSL Encryption

**MQTT over TLS:**
```
Port: 8883
Protocol: TLS 1.2 / TLS 1.3
Cipher Suites: AES-256-GCM, CHACHA20-POLY1305
Certificate: X.509 (Let's Encrypt)
```

**WebSocket Secure (WSS):**
```
Protocol: WSS (WebSocket over TLS)
Port: 443
Same TLS configuration as HTTPS
```

### 8.2 Device Authentication

**X.509 Client Certificates:**
```
Subject: CN=VF-SEOUL-001-TEMP-T3-01
Issuer: CN=WIA Vertical Farming CA
Valid From: 2025-01-01
Valid To: 2026-01-01
Key Usage: Digital Signature, Key Encipherment
```

**JWT Tokens (Alternative):**
```json
{
  "iss": "wia-vertical-farming",
  "sub": "VF-SEOUL-001-TEMP-T3-01",
  "iat": 1640995845,
  "exp": 1672531845,
  "farmId": "VF-SEOUL-001",
  "deviceType": "TEMPERATURE_SENSOR",
  "permissions": ["PUBLISH_SENSOR_DATA"]
}
```

### 8.3 Message Signing

**HMAC-SHA256 Signature:**
```
Payload: {"v":22.5,"t":1640995845}
Secret: shared_device_secret
Signature: sha256=abc123def456...
```

**Verification:**
```
Header: X-Signature: sha256=abc123def456...
Server validates signature before processing
```

---

## 9. Quality of Service (QoS)

### 9.1 MQTT QoS Levels

**QoS 0 - At Most Once:**
- Use: Non-critical data (light intensity)
- Guarantee: Best effort, no retry
- Performance: Fastest, lowest overhead

**QoS 1 - At Least Once:**
- Use: Standard sensor data
- Guarantee: Delivery confirmed, possible duplicates
- Performance: Balanced

**QoS 2 - Exactly Once:**
- Use: Critical controls, alerts, pH/EC
- Guarantee: Exactly one delivery
- Performance: Slowest, highest overhead

### 9.2 Message Priority

**High Priority:**
- Safety alerts
- Critical environmental controls
- System failures

**Normal Priority:**
- Standard sensor readings
- Crop observations
- Analytics data

**Low Priority:**
- Historical data sync
- Bulk exports
- Diagnostic logs

---

## 10. Protocol Selection Guide

| Protocol | Use Case | Pros | Cons |
|----------|----------|------|------|
| MQTT | IoT sensors, real-time data | Lightweight, reliable, pub/sub | Requires broker |
| WebSocket | Live dashboards, streaming | Full-duplex, low latency | Connection overhead |
| CoAP | Battery devices, constrained networks | Very lightweight, UDP-based | Less reliable |
| OPC UA | Industrial integration, BMS | Standardized, secure, structured | Complex setup |
| HTTP/REST | Device management, bulk operations | Universal, simple | Higher overhead |

---

## 11. Message Flow Examples

### 11.1 Normal Sensor Reading

```
1. Sensor measures temperature: 22.5°C
2. Device publishes to MQTT:
   Topic: farm/VF-SEOUL-001/tier/3/sensor/temperature
   QoS: 1
3. MQTT broker receives and confirms (PUBACK)
4. Broker distributes to subscribers
5. Backend stores in time-series database
6. WebSocket clients receive real-time update
7. Alert engine checks thresholds (no alert needed)
```

### 11.2 Critical Alert Flow

```
1. Temperature sensor reads: 28.5°C (threshold: 27°C)
2. Device publishes alert:
   Topic: farm/VF-SEOUL-001/tier/3/alert/critical
   QoS: 2
3. Backend receives alert
4. Alert engine triggers automation:
   - Send command to HVAC (max cooling)
   - Reduce LED intensity
5. Notifications sent:
   - WebSocket to dashboard (instant)
   - Email to supervisors
   - SMS to on-call staff
6. System waits for acknowledgment
7. Temperature normalizes, alert resolves
```

---

## 12. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01-01 | Initial protocol specification |

---

**© 2025 WIA Standards · MIT License**
**弘益人間 (Benefit All Humanity)**
