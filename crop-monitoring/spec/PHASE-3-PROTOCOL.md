# WIA Crop Monitoring Protocol Standard
## Phase 3 Specification

---

**Version**: 1.0.0
**Status**: Complete
**Date**: 2025-01
**Standard ID**: WIA-AGRI-006
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #84CC16 (Lime - AGRI)

---

## Table of Contents

1. [Overview](#overview)
2. [Communication Protocols](#communication-protocols)
3. [IoT Sensor Protocol](#iot-sensor-protocol)
4. [Camera Streaming Protocol](#camera-streaming-protocol)
5. [Data Synchronization](#data-synchronization)
6. [Security & Encryption](#security--encryption)
7. [Network Requirements](#network-requirements)
8. [Protocol Examples](#protocol-examples)
9. [Version History](#version-history)

---

## Overview

### 1.1 Purpose

The WIA Crop Monitoring Protocol Standard defines communication protocols for IoT sensors, cameras, drones, and agricultural equipment to stream real-time crop data. This ensures reliable, secure, and efficient data transmission from field to cloud.

**Core Protocols**:
- **MQTT** for lightweight IoT sensor messaging
- **WebSocket** for real-time camera streaming
- **CoAP** for constrained devices (low power)
- **HTTP/2** for high-bandwidth data uploads
- **LoRaWAN** for long-range, low-power sensors

### 1.2 Protocol Stack

```
┌─────────────────────────────────────┐
│   Application Layer                 │
│   (Crop Data Format - Phase 1)      │
├─────────────────────────────────────┤
│   Transport Layer                   │
│   MQTT | WebSocket | CoAP | HTTP/2  │
├─────────────────────────────────────┤
│   Network Layer                     │
│   IPv4/IPv6 | LoRaWAN               │
├─────────────────────────────────────┤
│   Physical Layer                    │
│   WiFi | 4G/5G | Ethernet | LoRa    │
└─────────────────────────────────────┘
```

### 1.3 Design Principles

1. **Low Bandwidth**: Optimized for rural areas with limited connectivity
2. **Reliability**: Auto-reconnect and offline buffering
3. **Real-Time**: Sub-second latency for critical alerts
4. **Security**: End-to-end encryption (TLS 1.3)
5. **Scalability**: Support 10,000+ sensors per farm

---

## Communication Protocols

### 2.1 Protocol Selection Matrix

| Use Case | Protocol | Bandwidth | Power | Range |
|----------|----------|-----------|-------|-------|
| Soil sensors | MQTT/CoAP | Low | Low | Medium |
| Weather stations | MQTT | Medium | Medium | Medium |
| Fixed cameras | WebSocket | High | High | Short |
| Drone imaging | HTTP/2 | Very High | High | Variable |
| Remote fields | LoRaWAN | Very Low | Very Low | Long |

### 2.2 MQTT for IoT Sensors

**Connection:**
```
Broker: mqtt.crop-monitoring.wiastandards.com
Port: 8883 (TLS)
Protocol: MQTT 3.1.1 / MQTT 5.0
QoS: 1 (At least once delivery)
```

**Topic Structure:**
```
wia/crop/{farmId}/{fieldId}/{sensorType}/{sensorId}

Examples:
wia/crop/FARM-KR-12345/FIELD-A-01/soil-moisture/SM-001
wia/crop/FARM-KR-12345/FIELD-A-01/temperature/TEMP-005
```

**Message Payload (JSON):**
```json
{
  "sensorId": "SM-001",
  "timestamp": "2025-06-15T10:30:00Z",
  "measurements": {
    "soilMoisture": {"value": 85, "unit": "%"},
    "soilTemperature": {"value": 22.5, "unit": "°C"}
  },
  "battery": 87,
  "signalStrength": -65
}
```

### 2.3 WebSocket for Camera Streaming

**Connection:**
```
wss://stream.crop-monitoring.wiastandards.com/v1/camera/{cameraId}
Protocol: WebSocket (RFC 6455)
Compression: permessage-deflate
```

**Handshake:**
```http
GET /v1/camera/CAM-001 HTTP/1.1
Host: stream.crop-monitoring.wiastandards.com
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==
Sec-WebSocket-Version: 13
Authorization: Bearer {api_key}
```

**Frame Format:**
```json
{
  "type": "video-frame",
  "cameraId": "CAM-001",
  "timestamp": "2025-06-15T10:30:00.123Z",
  "sequenceNumber": 12345,
  "codec": "h264",
  "resolution": "1920x1080",
  "fps": 30,
  "data": "base64-encoded-frame-data"
}
```

### 2.4 CoAP for Low-Power Devices

**Endpoint:**
```
coaps://coap.crop-monitoring.wiastandards.com:5684/sensors
Method: POST
```

**Request:**
```
POST /sensors/SM-001/data
Content-Format: application/json

{
  "moisture": 85,
  "temp": 22.5,
  "ts": 1625097600
}
```

**Response:**
```
2.01 Created
Location-Path: /sensors/SM-001/data/1625097600
```

### 2.5 LoRaWAN for Remote Fields

**Configuration:**
```
Frequency: 915 MHz (US) / 868 MHz (EU) / 920 MHz (Asia)
Data Rate: DR0-DR5 (250 bps - 5.5 kbps)
Adaptive Data Rate (ADR): Enabled
Confirmed Uplinks: For critical data only
```

**Payload (Compact Binary):**
```
[Sensor ID: 2 bytes][Timestamp: 4 bytes][Moisture: 1 byte][Temp: 2 bytes]

Example:
0x00 0x01 0x60 0xB9 0x4A 0x80 0x55 0x0E 0x1A
```

---

## IoT Sensor Protocol

### 3.1 Sensor Registration

**MQTT Topic:**
```
wia/crop/register
```

**Registration Message:**
```json
{
  "sensorId": "SM-001",
  "farmId": "FARM-KR-12345",
  "sensorType": "soil-moisture",
  "location": {"latitude": 37.5665, "longitude": 126.9780},
  "capabilities": ["moisture", "temperature", "ec"],
  "transmissionInterval": 300,
  "protocol": "MQTT",
  "firmwareVersion": "1.2.3"
}
```

**Response:**
```json
{
  "status": "registered",
  "sensorId": "SM-001",
  "assignedTopic": "wia/crop/FARM-KR-12345/FIELD-A-01/soil-moisture/SM-001",
  "config": {
    "transmissionInterval": 300,
    "qos": 1,
    "retain": false
  }
}
```

### 3.2 Periodic Data Transmission

**Transmission Schedule:**
- Normal mode: Every 5 minutes
- Alert mode: Every 30 seconds
- Sleep mode: Every 30 minutes

**Data Message (MQTT):**
```json
{
  "sensorId": "SM-001",
  "timestamp": "2025-06-15T10:30:00Z",
  "measurements": {
    "soilMoisture": {"value": 85, "unit": "%"},
    "soilTemperature": {"value": 22.5, "unit": "°C"},
    "electricalConductivity": {"value": 1.2, "unit": "dS/m"}
  },
  "battery": 87,
  "signalStrength": -65
}
```

### 3.3 Alert Protocol

**Critical Alert (QoS 2):**
```json
{
  "type": "alert",
  "severity": "critical",
  "sensorId": "SM-001",
  "timestamp": "2025-06-15T10:30:00Z",
  "alert": {
    "code": "SOIL_MOISTURE_LOW",
    "message": "Soil moisture below critical threshold",
    "value": 25,
    "threshold": 30
  }
}
```

---

## Camera Streaming Protocol

### 4.1 Video Stream Configuration

**Stream Setup (WebSocket):**
```json
{
  "action": "start-stream",
  "cameraId": "CAM-001",
  "config": {
    "resolution": "1920x1080",
    "fps": 30,
    "codec": "h264",
    "bitrate": 2000000,
    "keyFrameInterval": 60
  }
}
```

### 4.2 AI Frame Analysis Request

**Analysis Request:**
```json
{
  "action": "analyze-frame",
  "cameraId": "CAM-001",
  "timestamp": "2025-06-15T10:30:00Z",
  "analysisTypes": ["disease-detection", "pest-detection", "growth-stage"]
}
```

**Analysis Response:**
```json
{
  "type": "analysis-result",
  "timestamp": "2025-06-15T10:30:00Z",
  "results": {
    "diseaseDetection": [
      {
        "disease": "Late Blight",
        "confidence": 0.87,
        "boundingBox": {"x": 100, "y": 150, "w": 200, "h": 180}
      }
    ],
    "growthStage": "BBCH-51",
    "healthScore": 72
  }
}
```

### 4.3 Drone Video Upload

**HTTP/2 Multipart Upload:**
```http
POST /upload/drone-footage
Authorization: Bearer {api_key}
Content-Type: multipart/form-data

--boundary
Content-Disposition: form-data; name="metadata"

{
  "droneId": "DRONE-001",
  "flightId": "FLIGHT-20250615-001",
  "fieldId": "FIELD-A-01",
  "timestamp": "2025-06-15T10:30:00Z"
}

--boundary
Content-Disposition: form-data; name="video"; filename="flight.mp4"
Content-Type: video/mp4

[Binary video data]
--boundary--
```

---

## Data Synchronization

### 5.1 Offline Buffering

**Local Storage (SQLite):**
```sql
CREATE TABLE sensor_buffer (
  id INTEGER PRIMARY KEY,
  sensor_id TEXT,
  timestamp TEXT,
  payload TEXT,
  retry_count INTEGER DEFAULT 0,
  synced BOOLEAN DEFAULT 0
);
```

**Sync Process:**
1. Attempt to send data via MQTT
2. If connection fails, store in local buffer
3. Retry every 5 minutes with exponential backoff
4. Once connected, upload all buffered data
5. Delete from buffer after successful upload

### 5.2 Time Synchronization

**NTP Configuration:**
```
NTP Servers:
- time.crop-monitoring.wiastandards.com
- pool.ntp.org

Sync Interval: Every 1 hour
Drift Tolerance: ±1 second
```

**Time Sync Message (MQTT):**
```json
{
  "action": "time-sync",
  "sensorId": "SM-001",
  "localTime": "2025-06-15T10:30:00Z",
  "requestTimestamp": 1625097600000
}
```

**Response:**
```json
{
  "action": "time-sync-response",
  "serverTime": "2025-06-15T10:30:01.234Z",
  "responseTimestamp": 1625097601234,
  "drift": 1234
}
```

---

## Security & Encryption

### 6.1 Transport Layer Security

**TLS 1.3 Configuration:**
```
Cipher Suites:
- TLS_AES_128_GCM_SHA256
- TLS_AES_256_GCM_SHA384
- TLS_CHACHA20_POLY1305_SHA256

Certificate Validation: Required
Client Certificates: Optional (for enterprise)
```

### 6.2 Message Authentication

**HMAC Signature:**
```python
import hmac
import hashlib

message = json.dumps(payload)
secret_key = "sensor_secret_key"
signature = hmac.new(
    secret_key.encode(),
    message.encode(),
    hashlib.sha256
).hexdigest()

mqtt_publish(topic, {
    "payload": payload,
    "signature": signature
})
```

### 6.3 Data Encryption

**AES-256 Encryption (Sensitive Data):**
```javascript
const crypto = require('crypto');

function encryptData(data, key) {
  const iv = crypto.randomBytes(16);
  const cipher = crypto.createCipheriv('aes-256-gcm', key, iv);

  let encrypted = cipher.update(JSON.stringify(data), 'utf8', 'hex');
  encrypted += cipher.final('hex');

  return {
    encrypted: encrypted,
    iv: iv.toString('hex'),
    authTag: cipher.getAuthTag().toString('hex')
  };
}
```

---

## Network Requirements

### 6.1 Bandwidth Requirements

| Device Type | Upstream | Downstream | Latency |
|-------------|----------|------------|---------|
| Soil Sensor | 1 kbps | 0.5 kbps | < 5 sec |
| Weather Station | 5 kbps | 1 kbps | < 2 sec |
| Fixed Camera | 2 Mbps | 100 kbps | < 500 ms |
| Drone | 10 Mbps | 1 Mbps | < 1 sec |

### 6.2 Connection Reliability

**Auto-Reconnect Logic:**
```javascript
let reconnectAttempts = 0;
const maxReconnectAttempts = 10;

function connectMQTT() {
  client.on('error', (err) => {
    console.error('Connection error:', err);
    reconnect();
  });

  client.on('close', () => {
    reconnect();
  });
}

function reconnect() {
  if (reconnectAttempts < maxReconnectAttempts) {
    const delay = Math.min(1000 * Math.pow(2, reconnectAttempts), 60000);
    setTimeout(() => {
      reconnectAttempts++;
      connectMQTT();
    }, delay);
  }
}
```

---

## Protocol Examples

### 7.1 Complete MQTT Example (Node.js)

```javascript
const mqtt = require('mqtt');

const client = mqtt.connect('mqtts://mqtt.crop-monitoring.wiastandards.com:8883', {
  clientId: 'SM-001',
  username: 'FARM-KR-12345',
  password: 'wia_api_key_1234567890abcdef',
  clean: true,
  reconnectPeriod: 5000,
  connectTimeout: 30000
});

client.on('connect', () => {
  console.log('Connected to WIA Crop Monitoring MQTT Broker');

  // Subscribe to commands
  client.subscribe('wia/crop/FARM-KR-12345/FIELD-A-01/commands/SM-001');

  // Publish sensor data every 5 minutes
  setInterval(() => {
    const data = {
      sensorId: 'SM-001',
      timestamp: new Date().toISOString(),
      measurements: {
        soilMoisture: { value: Math.random() * 50 + 50, unit: '%' },
        soilTemperature: { value: Math.random() * 10 + 20, unit: '°C' }
      },
      battery: 87,
      signalStrength: -65
    };

    client.publish(
      'wia/crop/FARM-KR-12345/FIELD-A-01/soil-moisture/SM-001',
      JSON.stringify(data),
      { qos: 1 }
    );
  }, 300000);
});
```

### 7.2 WebSocket Camera Stream (Python)

```python
import asyncio
import websockets
import json
import base64

async def stream_camera():
    uri = "wss://stream.crop-monitoring.wiastandards.com/v1/camera/CAM-001"
    headers = {"Authorization": "Bearer wia_api_key_1234567890abcdef"}

    async with websockets.connect(uri, extra_headers=headers) as websocket:
        # Start streaming
        await websocket.send(json.dumps({
            "action": "start-stream",
            "config": {
                "resolution": "1920x1080",
                "fps": 30,
                "codec": "h264"
            }
        }))

        while True:
            # Capture frame (simulated)
            frame_data = capture_frame()

            message = {
                "type": "video-frame",
                "timestamp": datetime.utcnow().isoformat() + "Z",
                "data": base64.b64encode(frame_data).decode()
            }

            await websocket.send(json.dumps(message))
            await asyncio.sleep(1/30)  # 30 fps

asyncio.run(stream_camera())
```

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial protocol specification |

---

**Philosophy**: 弘益人間 (Benefit All Humanity)
**License**: MIT
**Contact**: protocol-support@wiastandards.com
**Documentation**: https://docs.crop-monitoring.wiastandards.com/protocol
