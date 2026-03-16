# WIA-AGRI-023: Deep Sea Aquaculture Standard
## Phase 3: Protocol Specification

**Version:** 1.0
**Status:** Complete
**Last Updated:** 2025-12-26
**Standard ID:** WIA-AGRI-023

---

## 1. Overview

This specification defines the communication protocols, messaging standards, and data exchange mechanisms for deep-sea aquaculture systems. It ensures reliable, real-time data transmission between sensors, control systems, cloud platforms, and third-party integrations.

### 1.1 Protocol Stack

```
┌─────────────────────────────────────┐
│   Application Layer (APIs)          │
├─────────────────────────────────────┤
│   Transport Layer (MQTT, WebSocket) │
├─────────────────────────────────────┤
│   Network Layer (TCP/IP, LoRaWAN)   │
├─────────────────────────────────────┤
│   Physical Layer (4G/5G, Satellite) │
└─────────────────────────────────────┘
```

---

## 2. Primary Communication Protocols

### 2.1 MQTT (Message Queuing Telemetry Transport)

**Use Case:** Real-time sensor data streaming, alerts, and device control

**Configuration:**
- **Version:** MQTT 3.1.1 or 5.0
- **QoS Level:**
  - QoS 0: Non-critical sensor readings
  - QoS 1: Critical environmental data, alerts
  - QoS 2: Control commands, harvest records
- **Keep-Alive:** 60 seconds
- **Clean Session:** False (persistent sessions)

**Topic Structure:**

```
wia/aquaculture/{farmId}/{cageId}/{dataType}/{sensorId}

Examples:
wia/aquaculture/DSA-FARM-USA-001/CAGE-01/sensors/ENV-SENSOR-001
wia/aquaculture/DSA-FARM-USA-001/CAGE-01/alerts/oxygen-low
wia/aquaculture/DSA-FARM-USA-001/CAGE-01/control/feeding
```

**Message Format:**

```json
{
  "messageId": "msg-123456789",
  "timestamp": "2025-12-26T10:30:00Z",
  "farmId": "DSA-FARM-USA-001",
  "cageId": "CAGE-01",
  "sensorId": "ENV-SENSOR-001",
  "type": "sensor-data",
  "payload": {
    "waterTemperature": {"value": 18.5, "unit": "celsius"},
    "salinity": {"value": 35.2, "unit": "ppt"},
    "dissolvedOxygen": {"value": 7.8, "unit": "mg/L"}
  },
  "metadata": {
    "batteryLevel": 87,
    "signalStrength": -72
  }
}
```

**MQTT Broker Requirements:**
- SSL/TLS encryption (TLS 1.2+)
- Client authentication (username/password or certificates)
- Message retention: 7 days for critical topics
- Maximum message size: 256KB

**Example MQTT Client (Python):**

```python
import paho.mqtt.client as mqtt
import json

def on_connect(client, userdata, flags, rc):
    client.subscribe("wia/aquaculture/DSA-FARM-USA-001/+/sensors/+")

def on_message(client, userdata, msg):
    data = json.loads(msg.payload)
    print(f"Received: {data}")

client = mqtt.Client()
client.username_pw_set("username", "password")
client.tls_set(ca_certs="/path/to/ca.crt")
client.on_connect = on_connect
client.on_message = on_message
client.connect("mqtt.wia-aquaculture.org", 8883, 60)
client.loop_forever()
```

---

### 2.2 WebSocket

**Use Case:** Real-time dashboard updates, live video feeds, bidirectional communication

**Connection URL:**

```
wss://stream.wia-aquaculture.org/v1/farms/{farmId}/stream
```

**Authentication:**

```json
{
  "action": "authenticate",
  "token": "Bearer eyJhbGc..."
}
```

**Subscription:**

```json
{
  "action": "subscribe",
  "channels": [
    "sensors",
    "alerts",
    "feeding",
    "fish-health"
  ],
  "cageIds": ["CAGE-01", "CAGE-02"]
}
```

**Message Format:**

```json
{
  "messageId": "ws-123456789",
  "channel": "sensors",
  "timestamp": "2025-12-26T10:30:00Z",
  "data": {
    "farmId": "DSA-FARM-USA-001",
    "cageId": "CAGE-01",
    "measurements": {...}
  }
}
```

**Example WebSocket Client (JavaScript):**

```javascript
const ws = new WebSocket('wss://stream.wia-aquaculture.org/v1/farms/DSA-FARM-USA-001/stream');

ws.onopen = () => {
  ws.send(JSON.stringify({
    action: 'authenticate',
    token: 'Bearer eyJhbGc...'
  }));

  ws.send(JSON.stringify({
    action: 'subscribe',
    channels: ['sensors', 'alerts']
  }));
};

ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log('Received:', data);
};
```

---

### 2.3 HTTP/REST

**Use Case:** API access, batch data upload, third-party integrations

**Base URL:**

```
https://api.wia-aquaculture.org/v1
```

**Request Format:**

```http
POST /api/v1/farms/DSA-FARM-USA-001/sensors
Content-Type: application/json
Authorization: Bearer eyJhbGc...

{
  "sensorId": "ENV-SENSOR-001",
  "timestamp": "2025-12-26T10:30:00Z",
  "measurements": {...}
}
```

**Response Format:**

```json
{
  "status": "success",
  "data": {
    "recordId": "REC-20251226-001234",
    "timestamp": "2025-12-26T10:30:00Z"
  },
  "metadata": {
    "requestId": "req-987654321",
    "processingTime": 45
  }
}
```

---

### 2.4 LoRaWAN (Long Range Wide Area Network)

**Use Case:** Remote offshore sites with limited connectivity, low-power sensors

**Configuration:**
- **Frequency Plans:** EU868, US915, AS923
- **Data Rate:** SF7-SF12 (adaptive)
- **Maximum Payload:** 51 bytes (SF7) to 242 bytes (SF12)
- **Uplink Interval:** 5-60 minutes (configurable)

**LoRaWAN Device Profile:**

```json
{
  "deviceEUI": "70B3D57ED0001234",
  "applicationEUI": "WIA-AQUA-APP-001",
  "deviceClass": "A",
  "activationType": "OTAA",
  "region": "US915"
}
```

**Payload Format (Cayenne LPP):**

```
Channel 1: Temperature Sensor (Type 0x67)
01 67 00 B9     → Ch:1, Type:Temp, Value:18.5°C

Channel 2: Analog Input - Salinity (Type 0x02)
02 02 0E 00     → Ch:2, Type:Analog, Value:35.2 ppt

Channel 3: Analog Input - Oxygen (Type 0x02)
03 02 03 0E     → Ch:3, Type:Analog, Value:7.8 mg/L
```

**Decoding Function (JavaScript):**

```javascript
function Decoder(bytes, port) {
  return {
    waterTemp: ((bytes[1] << 8) | bytes[2]) / 10,
    salinity: ((bytes[4] << 8) | bytes[5]) / 10,
    oxygen: ((bytes[7] << 8) | bytes[8]) / 10
  };
}
```

---

## 3. Satellite Communication

**Use Case:** Ultra-remote deep-sea farms beyond cellular/LoRaWAN coverage

**Supported Systems:**
- Iridium (global coverage)
- Globalstar
- Inmarsat

**Message Format (Short Burst Data):**

```
Header: 0xWIA1
Payload: Binary compressed sensor data
Max Size: 340 bytes
Frequency: Hourly
```

**Compression:** Protocol Buffers (protobuf) for efficient data transmission

---

## 4. Data Streaming and Time-Series

### 4.1 Time-Series Database Integration

**Recommended Databases:**
- InfluxDB
- TimescaleDB
- Apache Cassandra

**InfluxDB Line Protocol:**

```
aquaculture,farmId=DSA-FARM-USA-001,cageId=CAGE-01,sensor=ENV-SENSOR-001 waterTemp=18.5,salinity=35.2,oxygen=7.8 1640520000000000000
```

### 4.2 Apache Kafka Integration

**Topic Naming:**

```
wia.aquaculture.sensor-data
wia.aquaculture.alerts
wia.aquaculture.feeding-events
wia.aquaculture.harvest-records
```

**Message Format (Avro Schema):**

```json
{
  "namespace": "org.wia.aquaculture",
  "type": "record",
  "name": "SensorReading",
  "fields": [
    {"name": "farmId", "type": "string"},
    {"name": "sensorId", "type": "string"},
    {"name": "timestamp", "type": "long", "logicalType": "timestamp-millis"},
    {"name": "waterTemp", "type": "float"},
    {"name": "salinity", "type": "float"},
    {"name": "oxygen", "type": "float"}
  ]
}
```

---

## 5. Security Protocols

### 5.1 Transport Layer Security (TLS)

**Requirements:**
- TLS 1.2 or higher
- Certificate-based authentication
- Perfect Forward Secrecy (PFS)
- Cipher suites: AES-256-GCM, ChaCha20-Poly1305

### 5.2 Authentication

**OAuth 2.0 Flow:**

```
1. Client → Authorization Server: Request access token
2. Authorization Server → Client: Return access token
3. Client → API: Request with Bearer token
4. API → Client: Return data
```

**API Key Authentication:**

```http
X-API-Key: wia_live_sk_abcdef123456
```

### 5.3 Data Encryption

**At Rest:**
- AES-256 encryption
- Key management: AWS KMS, Azure Key Vault, or HashiCorp Vault

**In Transit:**
- TLS 1.2+ for all HTTP/WebSocket connections
- DTLS for LoRaWAN end-to-end encryption

---

## 6. Quality of Service (QoS)

### 6.1 Message Priority Levels

| Priority | Use Case | Delivery Guarantee | Latency |
|----------|----------|-------------------|---------|
| Critical | Oxygen alerts, equipment failure | Guaranteed, duplicate OK | < 1 second |
| High | Feeding commands, disease alerts | Guaranteed, exactly once | < 5 seconds |
| Normal | Sensor readings | At most once | < 30 seconds |
| Low | Historical data sync | Best effort | < 5 minutes |

### 6.2 Reliability Mechanisms

**Acknowledgments:**

```json
{
  "messageId": "msg-123456789",
  "ack": true,
  "receivedAt": "2025-12-26T10:30:01Z",
  "status": "processed"
}
```

**Retry Policy:**

```
Attempt 1: Immediate
Attempt 2: +5 seconds
Attempt 3: +30 seconds
Attempt 4: +5 minutes
Max attempts: 4
```

---

## 7. Protocol Interoperability

### 7.1 Protocol Translation Gateway

Converts between different protocols for legacy system integration:

```
LoRaWAN → MQTT → HTTP REST API
Satellite → Kafka → WebSocket
Modbus RTU → MQTT → Cloud Platform
```

### 7.2 OPC UA (Open Platform Communications Unified Architecture)

For industrial equipment integration:

```
Endpoint: opc.tcp://farm-plc.local:4840
Namespace: http://wia.org/aquaculture/
Nodes:
  - FarmId
  - CageTemperature
  - FeedingSystem
  - OxygenLevel
```

---

## 8. Bandwidth Optimization

### 8.1 Data Compression

**For Low-bandwidth Links:**
- Protocol Buffers: 70-80% size reduction
- MessagePack: 60-70% size reduction
- gzip: 50-60% size reduction

### 8.2 Adaptive Sampling

```json
{
  "normalConditions": {
    "samplingRate": 300,
    "unit": "seconds"
  },
  "abnormalConditions": {
    "samplingRate": 30,
    "unit": "seconds"
  },
  "criticalConditions": {
    "samplingRate": 5,
    "unit": "seconds"
  }
}
```

---

## 9. Edge Computing Protocols

### 9.1 Edge Processing

Local data aggregation and filtering at farm edge devices:

```python
# Edge device pseudo-code
def process_sensor_data(raw_data):
    # Local filtering
    if is_abnormal(raw_data):
        send_to_cloud(raw_data)  # High priority
    else:
        aggregate_locally(raw_data)
        if time_to_sync():
            send_aggregated_data()  # Low priority
```

### 9.2 MQTT Sparkplug B

Industrial IoT protocol for edge devices:

```
Topic: spBv1.0/wia-aquaculture/DDATA/farm-001/cage-01
Payload: Protobuf-encoded metrics
```

---

## 10. Protocol Testing and Validation

### 10.1 Test Vectors

**MQTT Message Test:**

```json
{
  "topic": "wia/aquaculture/TEST-FARM-001/CAGE-01/sensors/TEST-001",
  "qos": 1,
  "retain": false,
  "payload": {
    "messageId": "test-msg-001",
    "timestamp": "2025-12-26T10:30:00Z",
    "waterTemperature": {"value": 18.5, "unit": "celsius"}
  }
}
```

### 10.2 Protocol Compliance Checklist

- [ ] TLS 1.2+ encryption verified
- [ ] Authentication working (OAuth 2.0 / API Key)
- [ ] MQTT QoS levels tested (0, 1, 2)
- [ ] WebSocket reconnection handling
- [ ] LoRaWAN OTAA activation successful
- [ ] Message serialization/deserialization validated
- [ ] Error handling and retry logic tested
- [ ] Bandwidth usage within limits
- [ ] Latency requirements met

---

## 11. Protocol Migration Path

### 11.1 Version Compatibility

```json
{
  "protocolVersion": "1.0",
  "supportedVersions": ["1.0", "1.1"],
  "deprecatedVersions": [],
  "backwardCompatible": true
}
```

### 11.2 Graceful Degradation

If primary protocol fails:

```
Primary: MQTT over 5G
Fallback 1: HTTP REST over 4G
Fallback 2: LoRaWAN
Fallback 3: Satellite (critical alerts only)
```

---

## 12. Monitoring and Diagnostics

### 12.1 Protocol Health Metrics

```json
{
  "protocolHealth": {
    "mqtt": {
      "status": "healthy",
      "messageRate": 45,
      "messageRateUnit": "messages/minute",
      "errorRate": 0.001,
      "lastError": null
    },
    "websocket": {
      "status": "healthy",
      "activeConnections": 12,
      "averageLatency": 45,
      "latencyUnit": "ms"
    }
  }
}
```

---

**弘益人間 (Hongik Ingan) - Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
