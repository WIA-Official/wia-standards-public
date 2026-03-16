# WIA Green Infrastructure Communication Protocol
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
3. [IoT Device Integration](#iot-device-integration)
4. [Data Transmission](#data-transmission)
5. [Security](#security)
6. [Real-time Monitoring](#real-time-monitoring)
7. [Examples](#examples)

---

## Overview

### 1.1 Purpose

The WIA Green Infrastructure Communication Protocol defines standards for IoT sensor networks, real-time monitoring, and data exchange between green infrastructure systems and management platforms.

### 1.2 Supported Protocols

| Protocol | Use Case | Port |
|----------|----------|------|
| HTTPS/REST | API communication | 443 |
| MQTT | IoT sensor data | 1883/8883 |
| WebSocket | Real-time updates | 443 |
| CoAP | Constrained devices | 5683 |

---

## Communication Protocols

### 2.1 MQTT for IoT Sensors

#### Topic Structure

```
wia/green-infrastructure/{infrastructure_id}/{sensor_type}/{action}
```

**Examples:**
```
wia/green-infrastructure/GI-2025-001/soil_moisture/data
wia/green-infrastructure/GI-2025-001/temperature/data
wia/green-infrastructure/GI-2025-001/flow_meter/data
wia/green-infrastructure/GI-2025-001/status/alert
```

#### Message Format

```json
{
  "infrastructureId": "GI-2025-001",
  "sensorId": "SENSOR-GI-001",
  "type": "soil_moisture",
  "data": {
    "moisture": 65,
    "temperature": 22.5,
    "unit": "celsius"
  },
  "timestamp": "2025-01-15T10:30:00Z",
  "quality": "excellent"
}
```

#### QoS Levels

| Level | Description | Use Case |
|-------|-------------|----------|
| 0 | At most once | Non-critical data |
| 1 | At least once | Standard sensor readings |
| 2 | Exactly once | Critical alerts, commands |

### 2.2 WebSocket for Real-time Updates

#### Connection

```javascript
const ws = new WebSocket('wss://api.wia.live/green-infrastructure/v1/stream');

ws.onopen = () => {
  ws.send(JSON.stringify({
    action: 'subscribe',
    infrastructureId: 'GI-2025-001',
    dataTypes: ['soil_moisture', 'temperature', 'alerts']
  }));
};

ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log('Received:', data);
};
```

#### Message Types

**Sensor Update:**
```json
{
  "type": "sensor_update",
  "infrastructureId": "GI-2025-001",
  "sensorId": "SENSOR-GI-001",
  "data": {
    "moisture": 65,
    "temperature": 22.5
  },
  "timestamp": "2025-01-15T10:30:00Z"
}
```

**Alert Notification:**
```json
{
  "type": "alert",
  "infrastructureId": "GI-2025-001",
  "severity": "warning",
  "message": "Soil moisture below threshold",
  "timestamp": "2025-01-15T10:30:00Z"
}
```

### 2.3 CoAP for Constrained Devices

```
coap://api.wia.live:5683/infrastructure/GI-2025-001/sensor
```

**Request:**
```
POST /infrastructure/GI-2025-001/sensor
Content-Format: application/json

{"moisture": 65, "temperature": 22.5}
```

---

## IoT Device Integration

### 3.1 Device Registration

```http
POST /api/v1/devices/register
```

**Request:**
```json
{
  "deviceId": "SENSOR-GI-001",
  "type": "soil_moisture",
  "infrastructureId": "GI-2025-001",
  "location": "zone_a",
  "manufacturer": "Green Sensors Inc.",
  "model": "SM-3000",
  "firmware": "v2.1.0"
}
```

### 3.2 Device Configuration

**MQTT Configuration:**
```json
{
  "broker": "mqtt.wia.live",
  "port": 8883,
  "username": "device_SENSOR-GI-001",
  "password": "encrypted_password",
  "clientId": "SENSOR-GI-001",
  "keepAlive": 60,
  "qos": 1
}
```

### 3.3 Data Collection Schedule

| Sensor Type | Frequency | Battery Impact |
|-------------|-----------|----------------|
| Soil Moisture | 15 minutes | Low |
| Temperature | 5 minutes | Low |
| Flow Meter | Real-time (on-change) | Medium |
| Vegetation Health | Daily | Very Low |
| Water Quality | 1 hour | Medium |

---

## Data Transmission

### 4.1 Batch Transmission

For battery-powered devices:

```json
{
  "deviceId": "SENSOR-GI-001",
  "readings": [
    {
      "timestamp": "2025-01-15T10:00:00Z",
      "moisture": 65,
      "temperature": 22.5
    },
    {
      "timestamp": "2025-01-15T10:15:00Z",
      "moisture": 64,
      "temperature": 22.6
    },
    {
      "timestamp": "2025-01-15T10:30:00Z",
      "moisture": 63,
      "temperature": 22.7
    }
  ],
  "battery": 85
}
```

### 4.2 Data Compression

For limited bandwidth:

```json
{
  "d": "SENSOR-GI-001",
  "r": [
    {"t": "2025-01-15T10:00:00Z", "m": 65, "temp": 22.5},
    {"t": "2025-01-15T10:15:00Z", "m": 64, "temp": 22.6}
  ],
  "b": 85
}
```

### 4.3 Error Recovery

**Retry Logic:**
- Attempt 1: Immediate
- Attempt 2: After 30 seconds
- Attempt 3: After 2 minutes
- Attempt 4: After 5 minutes
- Fallback: Store locally and sync when connected

---

## Security

### 5.1 Authentication

#### Device Certificates (TLS)

```
Device Certificate: device-cert.pem
Private Key: device-key.pem
CA Certificate: ca-cert.pem
```

#### API Keys

```http
X-API-Key: wia_gi_12345678901234567890
X-Device-ID: SENSOR-GI-001
```

### 5.2 Encryption

| Protocol | Encryption |
|----------|------------|
| HTTPS | TLS 1.3 |
| MQTT | TLS 1.2+ |
| WebSocket | WSS (TLS) |
| CoAP | DTLS 1.2 |

### 5.3 Data Integrity

**Message Signing:**
```json
{
  "data": {...},
  "signature": "sha256_hmac_signature",
  "timestamp": "2025-01-15T10:30:00Z"
}
```

---

## Real-time Monitoring

### 6.1 Alert System

**Alert Levels:**
| Level | Trigger | Response Time |
|-------|---------|---------------|
| Critical | System failure | Immediate |
| Warning | Threshold exceeded | < 5 minutes |
| Info | Status change | < 15 minutes |

**Alert Message:**
```json
{
  "alertId": "ALERT-001",
  "infrastructureId": "GI-2025-001",
  "severity": "warning",
  "type": "low_moisture",
  "message": "Soil moisture below 40% in zone_a",
  "sensorId": "SENSOR-GI-001",
  "value": 38,
  "threshold": 40,
  "timestamp": "2025-01-15T10:30:00Z",
  "actions": ["notify_admin", "trigger_irrigation"]
}
```

### 6.2 Dashboard Updates

**Subscription:**
```json
{
  "action": "subscribe",
  "infrastructureIds": ["GI-2025-001", "GI-2025-002"],
  "metrics": ["all"],
  "updateFrequency": "real-time"
}
```

**Update Message:**
```json
{
  "type": "dashboard_update",
  "infrastructureId": "GI-2025-001",
  "metrics": {
    "soilMoisture": 65,
    "temperature": 22.5,
    "flowRate": 15.5,
    "vegetationHealth": "good"
  },
  "timestamp": "2025-01-15T10:30:00Z"
}
```

---

## Examples

### 7.1 MQTT Sensor Publishing

```python
import paho.mqtt.client as mqtt
import json

client = mqtt.Client(client_id="SENSOR-GI-001")
client.username_pw_set("device_username", "device_password")
client.tls_set(ca_certs="ca-cert.pem")
client.connect("mqtt.wia.live", 8883, 60)

topic = "wia/green-infrastructure/GI-2025-001/soil_moisture/data"
payload = {
    "sensorId": "SENSOR-GI-001",
    "moisture": 65,
    "temperature": 22.5,
    "timestamp": "2025-01-15T10:30:00Z"
}

client.publish(topic, json.dumps(payload), qos=1)
```

### 7.2 WebSocket Subscription

```javascript
const ws = new WebSocket('wss://api.wia.live/green-infrastructure/v1/stream');

ws.onopen = () => {
  ws.send(JSON.stringify({
    action: 'subscribe',
    infrastructureId: 'GI-2025-001',
    dataTypes: ['soil_moisture', 'temperature', 'alerts']
  }));
};

ws.onmessage = (event) => {
  const data = JSON.parse(event.data);

  if (data.type === 'alert') {
    console.log('Alert:', data.message);
    // Trigger notification
  } else if (data.type === 'sensor_update') {
    console.log('Sensor Update:', data.data);
    // Update dashboard
  }
};
```

### 7.3 REST API with Polling

```javascript
async function pollSensorData(infrastructureId) {
  const response = await fetch(
    `https://api.wia.live/green-infrastructure/v1/infrastructure/${infrastructureId}/readings`,
    {
      headers: {
        'Authorization': 'Bearer YOUR_TOKEN'
      }
    }
  );

  const data = await response.json();
  return data;
}

// Poll every 30 seconds
setInterval(() => {
  pollSensorData('GI-2025-001')
    .then(data => console.log(data));
}, 30000);
```

---

<div align="center">

**WIA Green Infrastructure Protocol v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA**

**MIT License**

</div>
