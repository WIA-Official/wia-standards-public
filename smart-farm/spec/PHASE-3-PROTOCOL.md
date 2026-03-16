# WIA-AGRI-001: Smart Farm Standard
## Phase 3 - Protocol Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-01-01

---

## 1. Overview

This specification defines communication protocols for smart farm IoT devices, sensor networks, automation controllers, and edge computing systems.

### 1.1 Protocol Stack

```
┌─────────────────────────────────────────┐
│   Application Layer (MQTT, HTTP, CoAP) │
├─────────────────────────────────────────┤
│   Security Layer (TLS 1.3, DTLS 1.2)   │
├─────────────────────────────────────────┤
│   Transport Layer (TCP, UDP)            │
├─────────────────────────────────────────┤
│   Network Layer (IPv4, IPv6, 6LoWPAN)  │
├─────────────────────────────────────────┤
│   Link Layer (WiFi, LoRa, Zigbee, BLE) │
└─────────────────────────────────────────┘
```

---

## 2. MQTT Protocol (Primary)

### 2.1 Broker Configuration

**Recommended Brokers:**
- Mosquitto (open-source)
- HiveMQ (enterprise)
- AWS IoT Core (cloud)
- Azure IoT Hub (cloud)

**Connection Parameters:**
```yaml
host: mqtt.wiastandards.com
port: 8883 (TLS) / 1883 (non-TLS)
protocol: MQTT v5.0 (fallback to v3.1.1)
keepalive: 60 seconds
clean_session: false (for persistent sessions)
qos: 1 (at least once delivery)
```

### 2.2 Topic Hierarchy

```
wia/smart-farm/{farmId}/{domain}/{action}/{resource}
```

**Examples:**
```
wia/smart-farm/farm-001/sensors/data/soil-001
wia/smart-farm/farm-001/sensors/status/soil-001
wia/smart-farm/farm-001/automation/command/irrigation
wia/smart-farm/farm-001/automation/status/irrigation
wia/smart-farm/farm-001/alerts/weather/frost
wia/smart-farm/farm-001/analytics/yield/crop-001
```

### 2.3 Topic Structure

| Level | Description | Example |
|-------|-------------|---------|
| 1 | Prefix | `wia` |
| 2 | Standard | `smart-farm` |
| 3 | Farm ID | `farm-001` |
| 4 | Domain | `sensors`, `automation`, `alerts`, `analytics` |
| 5 | Action | `data`, `command`, `status`, `config` |
| 6 | Resource | `soil-001`, `irrigation`, `frost` |

### 2.4 Publish: Sensor Data

**Topic:** `wia/smart-farm/{farmId}/sensors/data/{sensorId}`
**QoS:** 1
**Retain:** false
**Payload:**

```json
{
  "sensorId": "soil-001",
  "timestamp": 1704096600000,
  "measurements": {
    "soil": {
      "moisture": 68.5,
      "temperature": 22.3,
      "pH": 6.4
    }
  },
  "battery": 85,
  "quality": "good"
}
```

### 2.5 Subscribe: Automation Commands

**Topic:** `wia/smart-farm/{farmId}/automation/command/+`
**QoS:** 1
**Payload:**

```json
{
  "commandId": "cmd-001",
  "timestamp": 1704096600000,
  "action": "irrigation",
  "parameters": {
    "zoneId": "greenhouse-A",
    "duration": 15,
    "waterSource": "rainwater"
  },
  "priority": "normal"
}
```

### 2.6 Response: Command Acknowledgment

**Topic:** `wia/smart-farm/{farmId}/automation/status/{resource}`
**QoS:** 1
**Payload:**

```json
{
  "commandId": "cmd-001",
  "timestamp": 1704096605000,
  "status": "completed",
  "result": {
    "duration": 15,
    "volumeUsed": 450,
    "energyConsumed": 0.75
  }
}
```

### 2.7 Last Will and Testament (LWT)

```json
{
  "topic": "wia/smart-farm/farm-001/sensors/status/soil-001",
  "payload": {
    "sensorId": "soil-001",
    "status": "offline",
    "timestamp": 1704096600000
  },
  "qos": 1,
  "retain": true
}
```

---

## 3. HTTP/REST Protocol (Secondary)

### 3.1 Use Cases

- Web dashboards
- Mobile applications
- Third-party integrations
- Batch data uploads

### 3.2 Endpoints

See [PHASE-2-API-INTERFACE.md](./PHASE-2-API-INTERFACE.md) for complete REST API specification.

### 3.3 WebSocket for Real-Time Updates

**Connection:** `wss://api.wiastandards.com/v1/smart-farm/ws`

**Authentication:**
```json
{
  "type": "auth",
  "token": "your_jwt_token"
}
```

**Subscribe to Farm Updates:**
```json
{
  "type": "subscribe",
  "farmId": "farm-001",
  "topics": ["sensors", "alerts", "automation"]
}
```

**Receive Updates:**
```json
{
  "type": "sensor.data",
  "farmId": "farm-001",
  "sensorId": "soil-001",
  "data": { ... }
}
```

---

## 4. CoAP Protocol (Constrained Devices)

### 4.1 Overview

- **RFC 7252** - Constrained Application Protocol
- **Port:** 5684 (DTLS) / 5683 (non-secure)
- **Transport:** UDP
- **Use Case:** Battery-powered sensors, low-bandwidth networks

### 4.2 Resource Discovery

**Request:**
```
GET coap://farm-gateway.local/.well-known/core
```

**Response:**
```
</sensors/soil-001>;ct=50;rt="sensor.soil",
</sensors/air-001>;ct=50;rt="sensor.air",
</automation/irrigation>;ct=50;rt="actuator.irrigation"
```

### 4.3 Get Sensor Data

**Request:**
```
GET coap://farm-gateway.local/sensors/soil-001
```

**Response (CoAP Content Format 50 - application/json):**
```json
{
  "moisture": 68.5,
  "temperature": 22.3,
  "pH": 6.4
}
```

### 4.4 Observe (Pub/Sub Pattern)

**Request:**
```
GET coap://farm-gateway.local/sensors/soil-001
Observe: 0
```

**Notifications (sent automatically when value changes):**
```
2.05 Content
Observe: 1
Payload: {"moisture": 68.2, ...}

2.05 Content
Observe: 2
Payload: {"moisture": 67.8, ...}
```

---

## 5. LoRaWAN Protocol (Long-Range Communication)

### 5.1 Overview

- **Frequency:** 868 MHz (EU), 915 MHz (US), 923 MHz (Asia)
- **Range:** 2-15 km (rural), 1-5 km (urban)
- **Data Rate:** 0.3 - 50 kbps
- **Use Case:** Remote farms, solar-powered sensors

### 5.2 Network Architecture

```
Sensor Nodes → LoRa Gateway → Network Server → Application Server
```

### 5.3 Device Configuration

**Device EUI:** `70B3D5499XXXXXXX` (unique identifier)
**Application EUI:** `00000000000000XX`
**Application Key:** `XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX`

### 5.4 Uplink Message (Sensor → Server)

**FPort:** 1 (sensor data)
**Payload (bytes):**

```
[01][2A 5C][15 8F][19 00][00 55]
 │    │     │      │      │
 │    │     │      │      └─ Battery (85%)
 │    │     │      └──────── pH (6.4)
 │    │     └──────────────── Temperature (22.3°C)
 │    └─────────────────────── Moisture (68.5%)
 └──────────────────────────── Sensor Type (0x01 = soil)
```

**Decoding Function (JavaScript):**
```javascript
function decode(bytes) {
  return {
    sensorType: bytes[0],
    moisture: ((bytes[1] << 8) | bytes[2]) / 100,
    temperature: ((bytes[3] << 8) | bytes[4]) / 100,
    pH: ((bytes[5] << 8) | bytes[6]) / 100,
    battery: bytes[7]
  };
}
```

### 5.5 Downlink Message (Server → Sensor)

**FPort:** 2 (commands)
**Payload:**

```
[01][00 0F]
 │    │
 │    └─ Duration (15 minutes)
 └────── Command (0x01 = start irrigation)
```

---

## 6. Modbus Protocol (Industrial Devices)

### 6.1 Overview

- **Use Case:** Irrigation controllers, HVAC systems, pumps
- **Variants:** Modbus RTU (serial), Modbus TCP (Ethernet)

### 6.2 Modbus TCP

**Connection:** TCP port 502
**Addressing:** IP address + Unit ID

**Read Holding Registers (Function Code 0x03):**
```
Request:
[Transaction ID][Protocol ID][Length][Unit ID][Function Code][Start Address][Quantity]
[00 01][00 00][00 06][01][03][00 00][00 0A]

Response:
[Transaction ID][Protocol ID][Length][Unit ID][Function Code][Byte Count][Data...]
[00 01][00 00][00 17][01][03][14][data 20 bytes]
```

### 6.3 Register Map Example (Irrigation Controller)

| Address | Description | Type | Range | Unit |
|---------|-------------|------|-------|------|
| 40001 | Soil Moisture | Read | 0-100 | % |
| 40002 | Water Flow Rate | Read | 0-1000 | L/min |
| 40003 | Pump Status | Read | 0-1 | 0=off, 1=on |
| 40101 | Irrigation Duration | Write | 0-120 | min |
| 40102 | Target Moisture | Write | 0-100 | % |

---

## 7. BLE Protocol (Bluetooth Low Energy)

### 7.1 Overview

- **Use Case:** Short-range sensor reading (< 100m), mobile apps
- **Version:** Bluetooth 5.0+
- **Power:** Ultra-low (years on coin cell battery)

### 7.2 GATT Service Definition

**Service UUID:** `0000181A-0000-1000-8000-00805F9B34FB` (Environmental Sensing)

**Characteristics:**

| UUID | Property | Description |
|------|----------|-------------|
| `2A6E` | Read/Notify | Temperature |
| `2A6F` | Read/Notify | Humidity |
| `2A76` | Read/Notify | UV Index |
| `CUSTOM-001` | Read/Notify | Soil Moisture |
| `CUSTOM-002` | Read/Notify | Soil pH |

### 7.3 Advertisement Packet

```
Flags: [0x06] (LE General Discoverable, BR/EDR Not Supported)
Complete Local Name: "WIA-Soil-001"
Service UUID: 181A
Manufacturer Data: [CompanyID][SensorType][BatteryLevel]
```

### 7.4 Reading Sensor Data (JavaScript/Web Bluetooth)

```javascript
const device = await navigator.bluetooth.requestDevice({
  filters: [{ services: ['0000181a-0000-1000-8000-00805f9b34fb'] }]
});

const server = await device.gatt.connect();
const service = await server.getPrimaryService('0000181a-0000-1000-8000-00805f9b34fb');
const characteristic = await service.getCharacteristic('2a6e'); // Temperature

const value = await characteristic.readValue();
const temperature = value.getUint16(0, true) / 100; // °C
```

---

## 8. Security

### 8.1 TLS/DTLS Configuration

**Minimum Version:** TLS 1.2 (prefer TLS 1.3)
**Cipher Suites (Recommended):**
```
TLS_ECDHE_RSA_WITH_AES_128_GCM_SHA256
TLS_ECDHE_RSA_WITH_AES_256_GCM_SHA384
TLS_ECDHE_ECDSA_WITH_AES_128_GCM_SHA256
```

**Certificate Requirements:**
- RSA 2048-bit or ECDSA P-256
- Valid for max 2 years
- Include SubjectAltName (SAN)

### 8.2 Device Authentication

**Method 1: X.509 Certificates**
```
Device Certificate → Intermediate CA → Root CA
```

**Method 2: Pre-Shared Keys (PSK)**
```
PSK Identity: device-soil-001
PSK: hex-encoded-256-bit-key
```

**Method 3: Token-Based (MQTT)**
```
Username: device-soil-001
Password: JWT token
```

### 8.3 Message Encryption

**MQTT over TLS:**
- Broker certificate validation
- Client certificate (mutual TLS)
- Encrypted payload (optional, application-level)

**LoRaWAN:**
- Network Session Key (NwkSKey)
- Application Session Key (AppSKey)
- AES-128 encryption

---

## 9. Quality of Service (QoS)

### 9.1 MQTT QoS Levels

| QoS | Name | Use Case | Delivery Guarantee |
|-----|------|----------|-------------------|
| 0 | At most once | Non-critical sensor data | No guarantee |
| 1 | At least once | Regular sensor readings | Duplicate possible |
| 2 | Exactly once | Critical commands, billing data | No duplicates |

### 9.2 Recommended QoS by Message Type

| Message Type | QoS | Rationale |
|--------------|-----|-----------|
| Sensor data (periodic) | 0 or 1 | Next reading coming soon |
| Sensor data (event) | 1 | Important but idempotent |
| Alerts | 1 | Must be delivered |
| Automation commands | 1 or 2 | Avoid duplicate actions |
| Status updates | 1 | Track command completion |
| Configuration | 2 | Ensure proper setup |

---

## 10. Data Compression

### 10.1 JSON Compression

**Method:** gzip
**Savings:** 60-70% for typical sensor data

**Example (Python):**
```python
import gzip
import json

data = {"sensorId": "soil-001", "moisture": 68.5, ...}
compressed = gzip.compress(json.dumps(data).encode('utf-8'))
```

### 10.2 Binary Encoding (CBOR)

**Savings:** 30-50% vs JSON
**Library:** `cbor2` (Python), `cbor` (JavaScript)

**Example:**
```javascript
const cbor = require('cbor');
const data = {sensorId: "soil-001", moisture: 68.5};
const encoded = cbor.encode(data);
```

### 10.3 Protocol Buffers

**Schema (`sensor.proto`):**
```protobuf
syntax = "proto3";

message SensorData {
  string sensor_id = 1;
  int64 timestamp = 2;
  SoilMeasurements soil = 3;
}

message SoilMeasurements {
  float moisture = 1;
  float temperature = 2;
  float ph = 3;
}
```

---

## 11. Protocol Selection Guide

| Scenario | Recommended Protocol | Alternative |
|----------|---------------------|-------------|
| WiFi-enabled sensors | MQTT over TLS | HTTP/REST |
| Battery-powered sensors | CoAP over DTLS | LoRaWAN |
| Remote farms (no internet) | LoRaWAN | Satellite IoT |
| Industrial equipment | Modbus TCP | OPC UA |
| Mobile app integration | BLE + HTTP | MQTT over WebSocket |
| Real-time dashboards | WebSocket | Server-Sent Events |

---

## 12. Network Topologies

### 12.1 Star Topology (Most Common)

```
        Gateway/Hub
        /    |    \
       /     |     \
   Sensor Sensor Sensor
```

**Pros:** Simple, centralized control
**Cons:** Single point of failure

### 12.2 Mesh Topology (Zigbee, Thread)

```
Sensor ← → Sensor ← → Sensor
  ↕          ↕          ↕
Sensor ← → Gateway ← → Sensor
```

**Pros:** Self-healing, extended range
**Cons:** Complex routing, higher latency

### 12.3 Hybrid Topology

```
LoRa Sensors → LoRa Gateway ┐
WiFi Sensors → WiFi Gateway ├→ Cloud Platform
BLE Sensors → BLE Gateway   ┘
```

---

## 13. Edge Computing Protocol

### 13.1 Edge Device Communication

**Technology:** gRPC (for low-latency, high-throughput)
**Use Case:** Local AI inference, data preprocessing

**Service Definition (`edge.proto`):**
```protobuf
service EdgeAnalytics {
  rpc ProcessSensorData(SensorDataBatch) returns (AnalyticsResult);
  rpc DetectDisease(ImageData) returns (DiseaseDetection);
  rpc OptimizeIrrigation(IrrigationRequest) returns (IrrigationSchedule);
}
```

### 13.2 Edge-to-Cloud Sync

**Protocol:** MQTT-SN (MQTT for Sensor Networks) or HTTP/2
**Frequency:** Configurable (real-time, hourly, daily)
**Data:** Aggregated sensor readings, analytics results, alerts

---

## 14. Testing & Debugging

### 14.1 MQTT Testing Tools

- **MQTT Explorer** (GUI client)
- **Mosquitto CLI** (`mosquitto_sub`, `mosquitto_pub`)
- **MQTT.fx** (Java-based client)

**Example Commands:**
```bash
# Subscribe to all farm topics
mosquitto_sub -h mqtt.wiastandards.com -p 8883 \
  --cafile ca.crt -t "wia/smart-farm/farm-001/#" -v

# Publish sensor data
mosquitto_pub -h mqtt.wiastandards.com -p 8883 \
  --cafile ca.crt -t "wia/smart-farm/farm-001/sensors/data/soil-001" \
  -m '{"moisture": 68.5}'
```

### 14.2 Packet Capture

```bash
# Capture MQTT traffic
tcpdump -i eth0 -A 'tcp port 1883' -w mqtt.pcap

# Analyze with Wireshark
wireshark mqtt.pcap
```

---

## 15. Implementation Checklist

- [ ] Choose primary protocol (MQTT recommended)
- [ ] Set up MQTT broker (Mosquitto, HiveMQ)
- [ ] Define topic hierarchy
- [ ] Implement TLS/DTLS encryption
- [ ] Configure device authentication (X.509, PSK, JWT)
- [ ] Set up QoS policies
- [ ] Implement data compression (gzip, CBOR)
- [ ] Configure Last Will and Testament
- [ ] Set up edge gateway (for protocol translation)
- [ ] Implement WebSocket for real-time dashboards
- [ ] Test with MQTT clients
- [ ] Monitor protocol performance (latency, packet loss)

---

**Previous Phase:** [PHASE-2-API-INTERFACE.md](./PHASE-2-API-INTERFACE.md)
**Next Phase:** [PHASE-4-INTEGRATION.md](./PHASE-4-INTEGRATION.md)

---

© 2025 WIA (World Certification Industry Association)
弘益人間 · Benefit All Humanity
