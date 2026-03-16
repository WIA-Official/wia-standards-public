# WIA-AGRI-031: Urban Farming Standard
## Phase 3: Protocol Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-01-01

---

## 1. Overview

This specification defines real-time communication protocols for urban farming systems, including sensor data streaming, event notifications, and inter-farm communication.

### 1.1 Supported Protocols

- **MQTT**: Lightweight IoT messaging for sensors
- **WebSocket**: Real-time bidirectional communication
- **CoAP**: Constrained Application Protocol for IoT devices
- **Webhooks**: HTTP-based event notifications
- **SSE**: Server-Sent Events for real-time updates

---

## 2. MQTT Protocol

### 2.1 Connection Parameters

```
Broker: mqtt.urbanfarming.example.com
Port: 1883 (TCP), 8883 (TLS)
Protocol: MQTT v3.1.1 or v5.0
QoS Levels: 0, 1, 2 (configurable)
Keep Alive: 60 seconds
```

### 2.2 Topic Structure

```
urbanfarming/{farmId}/{dataType}/{subType}
```

**Examples:**
```
urbanfarming/UF-NYC-001/sensor/soil-moisture
urbanfarming/UF-NYC-001/event/harvest
urbanfarming/UF-NYC-001/alert/watering
urbanfarming/UF-NYC-001/status/system
```

### 2.3 Sensor Data Publishing

**Topic:** `urbanfarming/{farmId}/sensor/{sensorType}`

**Payload:**
```json
{
  "farmId": "UF-NYC-001",
  "sensorId": "SENSOR-001",
  "sensorType": "SOIL_MOISTURE",
  "location": "PLOT_A_01",
  "timestamp": "2025-06-15T12:00:00Z",
  "readings": {
    "value": 68,
    "unit": "percent",
    "status": "OPTIMAL"
  },
  "battery": 85,
  "signalStrength": -45
}
```

**QoS:** 1 (At least once delivery)
**Retain:** false

### 2.4 Event Publishing

**Topic:** `urbanfarming/{farmId}/event/{eventType}`

**Harvest Event:**
```json
{
  "eventType": "HARVEST",
  "farmId": "UF-NYC-001",
  "plotId": "UF-NYC-001-P01",
  "timestamp": "2025-07-20T16:30:00Z",
  "data": {
    "harvestId": "HRV-2025-001234",
    "crop": "Tomatoes",
    "quantity": 12.5,
    "unit": "kg",
    "gardenerId": "G-2345"
  }
}
```

**Planting Event:**
```json
{
  "eventType": "PLANTING",
  "farmId": "UF-NYC-001",
  "plotId": "UF-NYC-001-P01",
  "timestamp": "2025-05-01T10:00:00Z",
  "data": {
    "crop": "Tomatoes",
    "variety": "Heirloom Beefsteak",
    "plantCount": 6,
    "gardenerId": "G-2345"
  }
}
```

### 2.5 Alert Publishing

**Topic:** `urbanfarming/{farmId}/alert/{alertType}`

**Watering Alert:**
```json
{
  "alertType": "WATERING_NEEDED",
  "farmId": "UF-NYC-001",
  "plotId": "UF-NYC-001-P03",
  "timestamp": "2025-06-15T14:00:00Z",
  "severity": "MEDIUM",
  "message": "Soil moisture below optimal level",
  "data": {
    "currentMoisture": 45,
    "optimalMoisture": 60,
    "sensorId": "SENSOR-003"
  },
  "actionRequired": true
}
```

**Pest Alert:**
```json
{
  "alertType": "PEST_DETECTION",
  "farmId": "UF-NYC-001",
  "plotId": "UF-NYC-001-P05",
  "timestamp": "2025-06-16T09:00:00Z",
  "severity": "HIGH",
  "message": "Aphid infestation detected",
  "data": {
    "pestType": "APHIDS",
    "severity": "MODERATE",
    "affectedCrops": ["Lettuce", "Kale"]
  },
  "actionRequired": true,
  "recommendedAction": "Apply neem oil or introduce ladybugs"
}
```

### 2.6 Subscription Patterns

**Subscribe to all farm data:**
```
urbanfarming/UF-NYC-001/#
```

**Subscribe to all sensor data:**
```
urbanfarming/+/sensor/#
```

**Subscribe to specific sensor type:**
```
urbanfarming/+/sensor/soil-moisture
```

**Subscribe to all alerts:**
```
urbanfarming/+/alert/#
```

---

## 3. WebSocket Protocol

### 3.1 Connection

```javascript
const ws = new WebSocket('wss://ws.urbanfarming.example.com/v1');

ws.onopen = () => {
  ws.send(JSON.stringify({
    type: 'authenticate',
    token: 'YOUR_API_TOKEN'
  }));
};
```

### 3.2 Authentication Message

```json
{
  "type": "authenticate",
  "token": "eyJhbGciOiJIUzI1NiIs...",
  "farmId": "UF-NYC-001"
}
```

**Response:**
```json
{
  "type": "auth_success",
  "message": "Authentication successful",
  "sessionId": "WS-SESSION-12345"
}
```

### 3.3 Subscribe to Farm Updates

**Request:**
```json
{
  "type": "subscribe",
  "channels": [
    "farm:UF-NYC-001",
    "sensors:UF-NYC-001",
    "events:UF-NYC-001"
  ]
}
```

**Response:**
```json
{
  "type": "subscription_confirmed",
  "channels": [
    "farm:UF-NYC-001",
    "sensors:UF-NYC-001",
    "events:UF-NYC-001"
  ]
}
```

### 3.4 Real-time Data Messages

**Sensor Update:**
```json
{
  "type": "sensor_update",
  "channel": "sensors:UF-NYC-001",
  "timestamp": "2025-06-15T12:00:00Z",
  "data": {
    "sensorId": "SENSOR-001",
    "sensorType": "SOIL_MOISTURE",
    "value": 68,
    "status": "OPTIMAL"
  }
}
```

**Harvest Event:**
```json
{
  "type": "event",
  "channel": "events:UF-NYC-001",
  "eventType": "HARVEST",
  "timestamp": "2025-07-20T16:30:00Z",
  "data": {
    "harvestId": "HRV-2025-001234",
    "crop": "Tomatoes",
    "quantity": 12.5
  }
}
```

### 3.5 Heartbeat

**Client → Server (every 30 seconds):**
```json
{
  "type": "ping"
}
```

**Server → Client:**
```json
{
  "type": "pong",
  "timestamp": "2025-06-15T12:00:00Z"
}
```

---

## 4. CoAP Protocol

### 4.1 Resource Structure

```
coap://coap.urbanfarming.example.com/
  ├── farms/
  │   └── {farmId}/
  │       ├── sensors/
  │       ├── plots/
  │       └── status
  └── observe/
      └── {farmId}/
          └── sensors/{sensorId}
```

### 4.2 GET Request

```
coap://coap.urbanfarming.example.com/farms/UF-NYC-001/sensors
```

**Response:**
```json
{
  "farmId": "UF-NYC-001",
  "sensors": [
    {
      "sensorId": "SENSOR-001",
      "type": "SOIL_MOISTURE",
      "status": "ACTIVE",
      "lastReading": "2025-06-15T12:00:00Z"
    }
  ]
}
```

### 4.3 POST Sensor Data

```
POST coap://coap.urbanfarming.example.com/farms/UF-NYC-001/sensors/data
```

**Payload:**
```json
{
  "sensorId": "SENSOR-001",
  "readings": {
    "soilMoisture": 68,
    "temperature": 22.0
  },
  "timestamp": "2025-06-15T12:00:00Z"
}
```

**Response:**
```
2.01 Created
Location: /farms/UF-NYC-001/sensors/data/12345
```

### 4.4 Observe Pattern

```
GET coap://coap.urbanfarming.example.com/observe/UF-NYC-001/sensors/SENSOR-001
Observe: 0
```

**Notifications:**
```json
{
  "sensorId": "SENSOR-001",
  "value": 68,
  "timestamp": "2025-06-15T12:00:00Z"
}
```

---

## 5. Webhook Protocol

### 5.1 Event Types

- `farm.created`
- `farm.updated`
- `plot.assigned`
- `plot.released`
- `harvest.recorded`
- `member.added`
- `member.removed`
- `alert.triggered`
- `sensor.offline`
- `sensor.online`

### 5.2 Webhook Payload

**HTTP POST to configured URL:**

**Headers:**
```
Content-Type: application/json
X-WIA-Event: harvest.recorded
X-WIA-Signature: sha256=abc123...
X-WIA-Delivery: uuid-12345
```

**Body:**
```json
{
  "event": "harvest.recorded",
  "farmId": "UF-NYC-001",
  "timestamp": "2025-07-20T16:30:00Z",
  "data": {
    "harvestId": "HRV-2025-001234",
    "plotId": "UF-NYC-001-P01",
    "crop": "Tomatoes",
    "quantity": 12.5,
    "unit": "kg",
    "gardenerId": "G-2345"
  }
}
```

### 5.3 Signature Verification

**Pseudocode:**
```python
import hmac
import hashlib

def verify_signature(payload, signature, secret):
    expected = hmac.new(
        secret.encode(),
        payload.encode(),
        hashlib.sha256
    ).hexdigest()
    return f"sha256={expected}" == signature
```

### 5.4 Retry Logic

- Initial attempt: Immediate
- Retry 1: After 1 minute
- Retry 2: After 5 minutes
- Retry 3: After 15 minutes
- Retry 4: After 1 hour
- Maximum: 5 attempts

---

## 6. Server-Sent Events (SSE)

### 6.1 Connection

```javascript
const eventSource = new EventSource(
  'https://sse.urbanfarming.example.com/v1/farms/UF-NYC-001?token=YOUR_TOKEN'
);
```

### 6.2 Event Streams

**Sensor Updates:**
```
event: sensor_update
data: {"sensorId":"SENSOR-001","value":68,"timestamp":"2025-06-15T12:00:00Z"}

event: sensor_update
data: {"sensorId":"SENSOR-002","value":22.5,"timestamp":"2025-06-15T12:00:05Z"}
```

**Harvest Events:**
```
event: harvest
data: {"harvestId":"HRV-001","crop":"Tomatoes","quantity":12.5}
```

**Heartbeat:**
```
event: heartbeat
data: {"timestamp":"2025-06-15T12:00:00Z"}
```

### 6.3 Client Implementation

```javascript
eventSource.addEventListener('sensor_update', (e) => {
  const data = JSON.parse(e.data);
  console.log('Sensor update:', data);
});

eventSource.addEventListener('harvest', (e) => {
  const data = JSON.parse(e.data);
  console.log('Harvest recorded:', data);
});

eventSource.onerror = (error) => {
  console.error('SSE connection error:', error);
};
```

---

## 7. Protocol Selection Guide

| Use Case | Recommended Protocol | Reason |
|----------|---------------------|--------|
| IoT sensors | MQTT or CoAP | Lightweight, low bandwidth |
| Web dashboards | WebSocket or SSE | Real-time browser updates |
| Mobile apps | WebSocket | Bidirectional, efficient |
| Event notifications | Webhooks | Simple integration |
| Constrained devices | CoAP | Minimal overhead |
| Server monitoring | SSE | One-way data stream |

---

## 8. Security Considerations

### 8.1 Encryption

- **MQTT**: Use TLS (port 8883)
- **WebSocket**: Use WSS (TLS)
- **CoAP**: Use DTLS
- **Webhooks**: HTTPS only
- **SSE**: HTTPS only

### 8.2 Authentication

All protocols require authentication:
- API tokens
- OAuth 2.0 tokens
- Client certificates (for MQTT/CoAP)

### 8.3 Data Validation

- Validate all incoming messages
- Check timestamps for freshness
- Reject malformed data
- Rate limiting per client

---

## 9. Quality of Service (QoS)

### 9.1 MQTT QoS Levels

- **QoS 0**: Sensor data (fire and forget)
- **QoS 1**: Events and alerts (at least once)
- **QoS 2**: Critical commands (exactly once)

### 9.2 Message Retention

- Sensor data: Not retained
- Events: Retained for 24 hours
- Alerts: Retained for 7 days
- System status: Retained (last value)

---

## 10. Example Implementations

### 10.1 MQTT Publisher (Python)

```python
import paho.mqtt.client as mqtt
import json

client = mqtt.Client()
client.username_pw_set("api_key", "YOUR_API_KEY")
client.connect("mqtt.urbanfarming.example.com", 1883, 60)

data = {
    "farmId": "UF-NYC-001",
    "sensorId": "SENSOR-001",
    "readings": {"soilMoisture": 68},
    "timestamp": "2025-06-15T12:00:00Z"
}

client.publish(
    "urbanfarming/UF-NYC-001/sensor/soil-moisture",
    json.dumps(data),
    qos=1
)
```

### 10.2 WebSocket Client (JavaScript)

```javascript
const ws = new WebSocket('wss://ws.urbanfarming.example.com/v1');

ws.onopen = () => {
  ws.send(JSON.stringify({
    type: 'authenticate',
    token: 'YOUR_TOKEN'
  }));

  ws.send(JSON.stringify({
    type: 'subscribe',
    channels: ['farm:UF-NYC-001']
  }));
};

ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log('Received:', data);
};
```

---

**Previous Phase:** [PHASE-2-API-INTERFACE.md](PHASE-2-API-INTERFACE.md)
**Next Phase:** [PHASE-4-INTEGRATION.md](PHASE-4-INTEGRATION.md)

© 2025 WIA - World Certification Industry Association
弘益人間 · Benefit All Humanity
