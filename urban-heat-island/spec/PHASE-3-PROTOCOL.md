# WIA Urban Heat Island Response Protocol Standard
## Phase 3 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #EF4444 (Red - ENE Category)

---

## Table of Contents

1. [Overview](#overview)
2. [Communication Protocols](#communication-protocols)
3. [Data Exchange Formats](#data-exchange-formats)
4. [Real-time Streaming](#real-time-streaming)
5. [Security Protocols](#security-protocols)
6. [Interoperability Standards](#interoperability-standards)

---

## Overview

### 1.1 Purpose

Define standardized communication protocols for urban heat island monitoring systems, ensuring interoperability between sensors, platforms, and stakeholders.

### 1.2 Protocol Stack

```
┌─────────────────────────────────────┐
│   Application Layer (HTTP/MQTT)    │
├─────────────────────────────────────┤
│   Transport Layer (TLS/DTLS)       │
├─────────────────────────────────────┤
│   Network Layer (IPv4/IPv6)        │
├─────────────────────────────────────┤
│   Data Link Layer (WiFi/Ethernet)  │
└─────────────────────────────────────┘
```

---

## Communication Protocols

### 2.1 HTTP/HTTPS REST API

**Use Case**: Client-server communication, data queries

```http
POST /api/v1/heat/sensors/data HTTP/1.1
Host: api.wia.live
Authorization: Bearer {API_KEY}
Content-Type: application/json

{
  "sensorId": "SENSOR-001",
  "timestamp": "2025-07-15T14:00:00Z",
  "measurements": {
    "temperature": {"value": 35.5, "unit": "celsius"}
  }
}
```

**Features**:
- Request/response model
- Stateless
- Caching support
- HTTPS encryption

### 2.2 MQTT Protocol

**Use Case**: IoT sensor data streaming, low-bandwidth networks

```
Topic Structure:
  wia/heat/{areaId}/sensors/{sensorId}/data
  wia/heat/{areaId}/alerts
  wia/heat/{areaId}/status

Example:
  Topic: wia/heat/AREA-SEOUL-GANGNAM/sensors/SENSOR-001/data
  Payload: {
    "timestamp": "2025-07-15T14:00:00Z",
    "temperature": 35.5,
    "humidity": 60
  }
```

**MQTT Configuration**:
```json
{
  "broker": "mqtt.wia.live",
  "port": 8883,
  "protocol": "mqtts",
  "qos": 1,
  "retain": false,
  "keepalive": 60
}
```

**QoS Levels**:
- QoS 0: At most once (status updates)
- QoS 1: At least once (sensor data)
- QoS 2: Exactly once (critical alerts)

### 2.3 WebSocket Protocol

**Use Case**: Real-time bidirectional communication, dashboards

```javascript
// Connect
const ws = new WebSocket('wss://api.wia.live/heat/stream');

// Subscribe to area
ws.send(JSON.stringify({
  "action": "subscribe",
  "areaId": "AREA-SEOUL-GANGNAM",
  "dataTypes": ["temperature", "alerts"]
}));

// Receive updates
ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log('Temperature update:', data);
};
```

**Message Format**:
```json
{
  "type": "temperature_update",
  "areaId": "AREA-SEOUL-GANGNAM",
  "timestamp": "2025-07-15T14:00:00Z",
  "data": {
    "temperature": 35.5,
    "heatIslandIntensity": 7.0
  }
}
```

### 2.4 CoAP Protocol

**Use Case**: Constrained IoT devices, low-power sensors

```
coap://api.wia.live/heat/sensors/SENSOR-001/data

Request:
  POST coap://api.wia.live/heat/sensors/SENSOR-001/data
  Content-Format: application/json
  Payload: {"temp":35.5,"hum":60}

Response:
  2.01 Created
  Payload: {"recordId":"REC-2025-001"}
```

---

## Data Exchange Formats

### 3.1 JSON (Default)

```json
{
  "sensorId": "SENSOR-001",
  "timestamp": "2025-07-15T14:00:00Z",
  "measurements": {
    "temperature": {"value": 35.5, "unit": "celsius"},
    "humidity": {"value": 60, "unit": "percent"}
  }
}
```

### 3.2 CBOR (Compact Binary)

**Use Case**: Low-bandwidth IoT devices

```
CBOR Encoding:
a3                                 # map(3)
   67                              # text(7)
      73656e736f724964             # "sensorId"
   6b                              # text(11)
      53454e534f522d303031         # "SENSOR-001"
   69                              # text(9)
      74696d657374616d70           # "timestamp"
   ...
```

### 3.3 Protocol Buffers

**Use Case**: High-performance data serialization

```protobuf
syntax = "proto3";

message SensorData {
  string sensor_id = 1;
  int64 timestamp = 2;
  Measurements measurements = 3;
}

message Measurements {
  double temperature = 1;
  double humidity = 2;
  string unit = 3;
}
```

---

## Real-time Streaming

### 4.1 Server-Sent Events (SSE)

```javascript
const eventSource = new EventSource(
  'https://api.wia.live/heat/stream/AREA-SEOUL-GANGNAM'
);

eventSource.addEventListener('temperature', (event) => {
  const data = JSON.parse(event.data);
  console.log('Temperature:', data.value);
});
```

**Event Types**:
```
event: temperature
data: {"value":35.5,"timestamp":"2025-07-15T14:00:00Z"}

event: alert
data: {"severity":"critical","message":"Heat island intensity exceeds threshold"}
```

### 4.2 gRPC Streaming

```protobuf
service HeatMonitoring {
  rpc StreamTemperature(AreaRequest) returns (stream TemperatureData);
}
```

```javascript
const call = client.streamTemperature({areaId: 'AREA-SEOUL-GANGNAM'});

call.on('data', (data) => {
  console.log('Temperature:', data.temperature);
});
```

---

## Security Protocols

### 5.1 TLS/SSL Encryption

**Requirements**:
- TLS 1.2+ required
- TLS 1.3 recommended
- Strong cipher suites only

```
Cipher Suites:
  TLS_ECDHE_RSA_WITH_AES_256_GCM_SHA384
  TLS_ECDHE_RSA_WITH_AES_128_GCM_SHA256
  TLS_AES_256_GCM_SHA384
```

### 5.2 Authentication

**API Key Authentication**:
```http
Authorization: Bearer EXAMPLE_API_KEY_REPLACE_ME...
```

**JWT Token**:
```http
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

**Token Payload**:
```json
{
  "sub": "sensor_001",
  "iss": "wia.live",
  "exp": 1642262400,
  "scope": ["heat:read", "heat:write"]
}
```

### 5.3 Data Integrity

**Message Signing**:
```json
{
  "data": {
    "sensorId": "SENSOR-001",
    "temperature": 35.5
  },
  "signature": "sha256:abc123...",
  "timestamp": "2025-07-15T14:00:00Z"
}
```

**HMAC Verification**:
```python
import hmac
import hashlib

message = json.dumps(data, sort_keys=True)
signature = hmac.new(
    secret_key.encode(),
    message.encode(),
    hashlib.sha256
).hexdigest()
```

---

## Interoperability Standards

### 6.1 Smart City Integration

**NGSI-LD Context Broker**:
```json
{
  "id": "urn:ngsi-ld:HeatSensor:SENSOR-001",
  "type": "HeatSensor",
  "temperature": {
    "type": "Property",
    "value": 35.5,
    "unitCode": "CEL",
    "observedAt": "2025-07-15T14:00:00Z"
  },
  "@context": [
    "https://uri.etsi.org/ngsi-ld/v1/ngsi-ld-core-context.jsonld"
  ]
}
```

### 6.2 BACnet Integration

**BACnet Object**:
```
Object Type: Analog Input
Object Identifier: 1
Object Name: "Heat Sensor 001"
Present Value: 35.5 °C
Units: degrees-celsius
Status Flags: {0,0,0,0}
```

### 6.3 OPC UA Integration

**Node Definition**:
```xml
<UAVariable NodeId="ns=2;s=SENSOR-001.Temperature"
            BrowseName="Temperature"
            DataType="Double">
  <DisplayName>Sensor 001 Temperature</DisplayName>
  <Description>Air temperature in Celsius</Description>
</UAVariable>
```

### 6.4 SensorThings API

```json
{
  "@iot.id": 1,
  "@iot.selfLink": "https://api.wia.live/sensorthings/v1.1/Datastreams(1)",
  "name": "Heat Sensor 001 Temperature",
  "description": "Air temperature measurement",
  "observationType": "http://www.opengis.net/def/observationType/OGC-OM/2.0/OM_Measurement",
  "unitOfMeasurement": {
    "name": "degree Celsius",
    "symbol": "°C",
    "definition": "http://www.qudt.org/qudt/owl/1.0.0/unit/Instances.html#DegreeCelsius"
  }
}
```

---

<div align="center">

**WIA Urban Heat Island Response Protocol v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA**

**MIT License**

</div>

*Generated under WIA Standards Phase 3 governance — 弘益人間 Benefit All Humanity.*
