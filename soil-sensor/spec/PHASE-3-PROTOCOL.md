# WIA-AGRI-005 PHASE 3: Communication Protocol

## Overview
Phase 3 defines the communication protocols for WIA-AGRI-005 Soil Sensor Standard, enabling reliable data transmission from sensors in agricultural fields to cloud platforms through LoRaWAN, MQTT, HTTP/HTTPS, and other protocols.

## Protocol Stack

### Supported Protocols

1. **LoRaWAN** - Long-range, low-power wireless for remote fields
2. **MQTT** - Lightweight pub/sub messaging for connected sensors
3. **HTTP/HTTPS** - RESTful communication for standard deployments
4. **CoAP** - Constrained Application Protocol for IoT devices
5. **Modbus RTU/TCP** - Industrial automation protocol
6. **NB-IoT** - Narrowband IoT cellular communication

## 1. LoRaWAN Protocol

### Overview
LoRaWAN is ideal for soil sensors in remote agricultural fields where traditional connectivity is unavailable. Provides long-range (2-15km) with low power consumption.

### Network Architecture
```
[Soil Sensor] --LoRa--> [Gateway] --IP--> [Network Server] --HTTPS--> [Application Server]
```

### Device Configuration

#### Device Profile
```json
{
  "deviceEUI": "70B3D57ED0000001",
  "appEUI": "70B3D57ED0001000",
  "appKey": "2B7E151628AED2A6ABF7158809CF4F3C",
  "deviceClass": "A",
  "region": "EU868",
  "activationType": "OTAA"
}
```

#### LoRaWAN Classes
- **Class A (Required)**: Bi-directional, lowest power
  - Uplink: Sensor sends data when needed
  - Downlink: Only after uplink (2 receive windows)
  - Battery life: Up to 10 years

- **Class B (Optional)**: Scheduled receive windows
  - Periodic downlink opportunities
  - Time-synchronized beacons
  - Battery life: 2-5 years

- **Class C (Optional)**: Continuous listening
  - Always ready for downlink
  - Highest power consumption
  - Battery life: Requires mains power

### Data Encoding

#### Uplink Message Format (Soil Sensor)
```
Byte 0: Message Type (0x01 = Sensor Reading)
Byte 1-2: Nitrogen (mg/kg) - uint16, Big Endian
Byte 3-4: Phosphorus (mg/kg) - uint16, Big Endian
Byte 5-6: Potassium (mg/kg) - uint16, Big Endian
Byte 7: pH (scaled: pH * 10) - uint8
Byte 8: EC (scaled: EC * 10) - uint8
Byte 9: Moisture (%) - uint8
Byte 10: Temperature (°C + 50) - uint8
Byte 11: Battery (%) - uint8
```

**Example**:
```
01 0055 0023 0096 44 0C 23 48 57
```
Decoded:
- Nitrogen: 85 mg/kg
- Phosphorus: 35 mg/kg
- Potassium: 150 mg/kg
- pH: 6.8
- EC: 1.2 dS/m
- Moisture: 35%
- Temperature: 22°C
- Battery: 87%

#### Downlink Message Format (Configuration)
```
Byte 0: Command Type
  0x01 = Set Interval
  0x02 = Request Calibration
  0x03 = Reset Device
Byte 1-N: Command Parameters
```

### Transmission Strategy

#### Adaptive Data Rate (ADR)
- Start with SF12 (longest range, lowest data rate)
- Network server adjusts SF based on link quality
- Target: SF7-SF9 for optimal battery life

#### Transmission Schedule
```json
{
  "normalMode": {
    "interval": 900,
    "spreadingFactor": "SF7",
    "transmitPower": 14
  },
  "powerSaveMode": {
    "interval": 3600,
    "spreadingFactor": "SF9",
    "transmitPower": 10
  },
  "alertMode": {
    "interval": 300,
    "spreadingFactor": "SF7",
    "transmitPower": 14
  }
}
```

### Security
- **AES-128 Encryption**: All LoRaWAN payloads
- **Network Session Key (NwkSKey)**: Network layer security
- **Application Session Key (AppSKey)**: Application layer security
- **Over-The-Air Activation (OTAA)**: Secure device joining

## 2. MQTT Protocol

### Overview
MQTT provides lightweight publish/subscribe messaging for soil sensors with reliable connectivity (Wi-Fi, Ethernet, 4G).

### Broker Configuration
```json
{
  "broker": "mqtt.wia-agri.org",
  "port": 8883,
  "protocol": "mqtts",
  "clientId": "SOIL-001",
  "username": "soil-sensor",
  "password": "***",
  "keepAlive": 60,
  "cleanSession": false,
  "qos": 1,
  "tlsVersion": "TLSv1.3"
}
```

### Topic Structure

#### Publishing (Sensor → Cloud)
```
wia/agri/soil/{sensorId}/data
wia/agri/soil/{sensorId}/status
wia/agri/soil/{sensorId}/alert
wia/agri/soil/{sensorId}/calibration
```

#### Subscribing (Cloud → Sensor)
```
wia/agri/soil/{sensorId}/command
wia/agri/soil/{sensorId}/config
wia/agri/soil/{sensorId}/firmware
```

### Message Format

#### Data Topic
```json
Topic: wia/agri/soil/SOIL-001/data
QoS: 1
Retain: false

Payload:
{
  "sensorId": "SOIL-001",
  "timestamp": "2025-12-26T10:00:00Z",
  "npk": {
    "nitrogen": 85,
    "phosphorus": 35,
    "potassium": 150
  },
  "ph": 6.8,
  "ec": 1.2,
  "moisture": 35,
  "temperature": 22,
  "battery": 87,
  "standard": "WIA-AGRI-005"
}
```

#### Status Topic
```json
Topic: wia/agri/soil/SOIL-001/status
QoS: 1
Retain: true

Payload:
{
  "sensorId": "SOIL-001",
  "status": "online",
  "lastSeen": "2025-12-26T10:00:00Z",
  "uptime": 86400,
  "firmware": "1.2.3",
  "signalStrength": -75
}
```

#### Alert Topic
```json
Topic: wia/agri/soil/SOIL-001/alert
QoS: 2
Retain: false

Payload:
{
  "sensorId": "SOIL-001",
  "alertType": "moisture_low",
  "severity": "warning",
  "value": 25,
  "threshold": 30,
  "timestamp": "2025-12-26T10:00:00Z"
}
```

### Last Will and Testament (LWT)
```json
{
  "topic": "wia/agri/soil/SOIL-001/status",
  "payload": {
    "sensorId": "SOIL-001",
    "status": "offline",
    "reason": "connection_lost"
  },
  "qos": 1,
  "retain": true
}
```

## 3. HTTP/HTTPS Protocol

### RESTful Endpoints

#### Submit Reading
```http
POST https://api.wia-agri.org/v1/soil-sensor/readings
Content-Type: application/json
Authorization: Bearer <token>

{
  "sensorId": "SOIL-001",
  "timestamp": "2025-12-26T10:00:00Z",
  "npk": { "nitrogen": 85, "phosphorus": 35, "potassium": 150 },
  "ph": 6.8,
  "ec": 1.2,
  "moisture": 35,
  "temperature": 22
}
```

#### Batch Submission
```http
POST https://api.wia-agri.org/v1/soil-sensor/readings/batch
Content-Type: application/json
Authorization: Bearer <token>

{
  "readings": [
    { "sensorId": "SOIL-001", ... },
    { "sensorId": "SOIL-002", ... }
  ]
}
```

### Compression
- **Gzip**: Recommended for batch submissions
- **Header**: `Content-Encoding: gzip`

### Retry Strategy
```javascript
const retryConfig = {
  maxRetries: 3,
  backoff: 'exponential',
  initialDelay: 1000,  // 1 second
  maxDelay: 30000,     // 30 seconds
  retryOn: [408, 429, 500, 502, 503, 504]
};
```

## 4. CoAP Protocol

### Overview
Constrained Application Protocol for resource-limited devices.

### Endpoint
```
coaps://coap.wia-agri.org:5684/soil-sensor/readings
```

### Message Format
```
POST coaps://coap.wia-agri.org:5684/soil-sensor/readings
Content-Format: application/json (50)
DTLS 1.2 Encryption

{
  "sensorId": "SOIL-001",
  "n": 85, "p": 35, "k": 150,
  "ph": 6.8, "m": 35, "t": 22
}
```

### Observe Pattern
```
GET coaps://coap.wia-agri.org:5684/soil-sensor/SOIL-001?observe=1
```

## 5. Modbus Protocol

### RTU Configuration
```
Baud Rate: 9600 bps
Data Bits: 8
Parity: None
Stop Bits: 1
Slave Address: 1-247
```

### Register Map

| Address | Register Type | Description | Unit | Scale |
|---------|--------------|-------------|------|-------|
| 0x0000 | Holding | Nitrogen | mg/kg | 1 |
| 0x0001 | Holding | Phosphorus | mg/kg | 1 |
| 0x0002 | Holding | Potassium | mg/kg | 1 |
| 0x0003 | Holding | pH | pH | 0.1 |
| 0x0004 | Holding | EC | dS/m | 0.1 |
| 0x0005 | Holding | Moisture | % | 1 |
| 0x0006 | Holding | Temperature | °C | 0.1 |
| 0x0007 | Input | Battery | % | 1 |
| 0x0008 | Input | Status | bitmap | - |

### Read Command
```
Request: [Slave][0x03][Start Addr Hi][Start Addr Lo][Num Regs Hi][Num Regs Lo][CRC Lo][CRC Hi]
Example: 01 03 00 00 00 07 04 08

Response: [Slave][0x03][Byte Count][Data...][CRC Lo][CRC Hi]
Example: 01 03 0E 00 55 00 23 00 96 00 44 00 0C 00 23 00 48 F2 A1
```

## Security Considerations

### Transport Layer Security
- **TLS 1.3**: Required for HTTPS and MQTTS
- **DTLS 1.2**: Required for CoAP
- **AES-128**: LoRaWAN encryption

### Authentication
```json
{
  "method": "oauth2",
  "tokenEndpoint": "https://auth.wia-agri.org/token",
  "clientId": "soil-sensor-001",
  "clientSecret": "***",
  "scope": "soil-sensor:write"
}
```

### Certificate Pinning
```json
{
  "pinnedCertificates": [
    "sha256/AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA="
  ],
  "validationMode": "strict"
}
```

## Network Resilience

### Offline Buffering
```json
{
  "bufferSize": 1000,
  "storageType": "flash",
  "retentionPolicy": "fifo",
  "syncStrategy": "batch"
}
```

### Connection Management
```javascript
// Automatic reconnection
const connectionConfig = {
  autoReconnect: true,
  reconnectInterval: 5000,
  maxReconnectAttempts: 10,
  connectionTimeout: 30000
};
```

## Performance Optimization

### Payload Compression
- **LoRaWAN**: Binary encoding (12 bytes)
- **MQTT**: JSON with optional gzip
- **HTTP**: JSON with gzip
- **CoAP**: CBOR encoding

### Batching Strategy
```json
{
  "batchSize": 10,
  "batchInterval": 3600,
  "flushOnAlert": true,
  "maxBatchAge": 86400
}
```

## 弘益人間 (Benefit All Humanity)
Phase 3 ensures reliable, secure, and efficient communication from soil sensors to agricultural systems worldwide, enabling farmers to make data-driven decisions regardless of connectivity constraints.

---
© 2025 SmileStory Inc. / WIA
