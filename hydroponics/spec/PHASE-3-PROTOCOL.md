# WIA-AGRI-027: Hydroponics Standard
## Phase 3: Communication Protocols

**Version:** 1.0
**Status:** Active
**Last Updated:** 2025-12-26
**Emoji:** 💧

---

## 1. Overview

This document defines the communication protocols for the WIA-AGRI-027 Hydroponics Standard. These protocols ensure reliable, secure, and efficient data exchange between sensors, controllers, gateways, and cloud platforms.

### 1.1 Philosophy

弘益人間 (홍익인간) - Benefit All Humanity

Our protocols are designed for reliability, scalability, and interoperability across diverse hardware and software ecosystems.

### 1.2 Protocol Stack

```
┌─────────────────────────────────────┐
│     Application Layer (REST/MQTT)   │
├─────────────────────────────────────┤
│     Security Layer (TLS/DTLS)       │
├─────────────────────────────────────┤
│     Transport Layer (TCP/UDP)       │
├─────────────────────────────────────┤
│     Network Layer (IPv4/IPv6)       │
├─────────────────────────────────────┤
│     Physical Layer (WiFi/Ethernet)  │
└─────────────────────────────────────┘
```

---

## 2. Primary Protocols

### 2.1 MQTT (Message Queuing Telemetry Transport)

**Purpose:** Real-time sensor data streaming and device control

**Advantages:**
- Lightweight and efficient
- Publish/Subscribe model
- QoS (Quality of Service) levels
- Low bandwidth usage
- Ideal for IoT devices

**Configuration:**

```yaml
mqtt:
  broker: mqtt.wia.org
  port: 8883 (TLS)
  protocol: MQTTv5
  keepalive: 60
  clean_session: true
  qos: 1
```

**Topic Structure:**

```
wia/agri027/{facility_id}/{system_id}/{device_type}/{device_id}
```

**Examples:**

```
wia/agri027/farm-001/hydro-sys-001/sensors/ph-001
wia/agri027/farm-001/hydro-sys-001/sensors/ec-001
wia/agri027/farm-001/hydro-sys-001/actuators/pump-001
wia/agri027/farm-001/hydro-sys-001/alerts
```

### 2.2 MQTT Topic Hierarchy

**Sensor Data Publishing:**

```
Topic: wia/agri027/{facility_id}/{system_id}/sensors/{sensor_type}
Payload: {
  "deviceId": "SENSOR-PH-001",
  "timestamp": "2025-12-26T10:30:00Z",
  "value": 6.2,
  "unit": "pH",
  "quality": "excellent"
}
```

**Control Commands:**

```
Topic: wia/agri027/{facility_id}/{system_id}/control/{device_id}
Payload: {
  "command": "start",
  "duration": 300,
  "priority": "high"
}
```

**Alert Notifications:**

```
Topic: wia/agri027/{facility_id}/{system_id}/alerts
Payload: {
  "alertId": "550e8400-e29b-41d4-a716-446655440000",
  "severity": "warning",
  "parameter": "pH",
  "currentValue": 7.2,
  "message": "pH level exceeds threshold"
}
```

### 2.3 QoS Levels

| QoS | Name | Description | Use Case |
|-----|------|-------------|----------|
| 0 | At most once | Fire and forget | Non-critical telemetry |
| 1 | At least once | Acknowledged delivery | Sensor readings |
| 2 | Exactly once | Guaranteed delivery | Control commands, alerts |

**Recommended QoS:**

- Sensor readings: QoS 1
- Control commands: QoS 2
- Alerts: QoS 2
- Status updates: QoS 1

---

## 3. WebSocket Protocol

**Purpose:** Real-time bidirectional communication for dashboards and monitoring applications

**Endpoint:**

```
wss://ws.wia.org/agri027/v1/stream
```

**Connection Handshake:**

```javascript
const ws = new WebSocket('wss://ws.wia.org/agri027/v1/stream');

ws.onopen = function() {
  // Authenticate
  ws.send(JSON.stringify({
    type: 'auth',
    token: 'YOUR_API_KEY'
  }));

  // Subscribe to system
  ws.send(JSON.stringify({
    type: 'subscribe',
    systemId: 'HYDRO-SYS-001',
    channels: ['sensors', 'alerts', 'control']
  }));
};

ws.onmessage = function(event) {
  const data = JSON.parse(event.data);
  console.log('Received:', data);
};
```

**Message Types:**

```json
// Authentication
{
  "type": "auth",
  "token": "YOUR_API_KEY"
}

// Subscribe
{
  "type": "subscribe",
  "systemId": "HYDRO-SYS-001",
  "channels": ["sensors", "alerts"]
}

// Unsubscribe
{
  "type": "unsubscribe",
  "systemId": "HYDRO-SYS-001"
}

// Sensor Data Event
{
  "type": "sensor_data",
  "systemId": "HYDRO-SYS-001",
  "deviceId": "SENSOR-PH-001",
  "timestamp": "2025-12-26T10:30:00Z",
  "value": 6.2,
  "unit": "pH"
}

// Alert Event
{
  "type": "alert",
  "systemId": "HYDRO-SYS-001",
  "alertId": "550e8400-e29b-41d4-a716-446655440000",
  "severity": "warning",
  "message": "pH level exceeds threshold"
}
```

---

## 4. HTTP/REST Protocol

**Purpose:** CRUD operations, configuration management, and batch queries

**Base URL:**

```
https://api.wia.org/agri027/v1
```

**HTTP Methods:**

- `GET`: Retrieve resources
- `POST`: Create new resources
- `PUT`: Update existing resources
- `PATCH`: Partial updates
- `DELETE`: Remove resources

**Request Headers:**

```http
Authorization: Bearer YOUR_API_KEY
Content-Type: application/json
Accept: application/json
X-Request-ID: unique-request-id
```

**Response Headers:**

```http
Content-Type: application/json
X-RateLimit-Limit: 100
X-RateLimit-Remaining: 95
X-RateLimit-Reset: 1640520000
X-Request-ID: unique-request-id
```

**Example Request:**

```bash
curl -X GET https://api.wia.org/agri027/v1/system/HYDRO-SYS-001/sensors/current \
  -H "Authorization: Bearer EXAMPLE_API_KEY_REPLACE_ME" \
  -H "Content-Type: application/json"
```

---

## 5. CoAP (Constrained Application Protocol)

**Purpose:** Lightweight protocol for resource-constrained devices

**Benefits:**
- Low overhead
- UDP-based (optional DTLS for security)
- RESTful design
- Ideal for battery-powered sensors

**Endpoint:**

```
coaps://coap.wia.org:5684/agri027/v1
```

**Example Request:**

```bash
coap-client -m get coaps://coap.wia.org:5684/agri027/v1/system/HYDRO-SYS-001/sensors
```

**CoAP Methods:**

- `GET`: Retrieve sensor data
- `POST`: Send sensor readings
- `PUT`: Update configuration
- `OBSERVE`: Subscribe to resource updates

**Observe Example:**

```bash
coap-client -m get -s 60 coaps://coap.wia.org:5684/agri027/v1/system/HYDRO-SYS-001/sensors/ph-001
```

---

## 6. Modbus Protocol (Legacy Support)

**Purpose:** Integration with industrial sensors and controllers

**Supported Variants:**
- Modbus TCP/IP
- Modbus RTU (over RS-485)

**Configuration:**

```yaml
modbus:
  mode: TCP
  host: 192.168.1.100
  port: 502
  slave_id: 1
  timeout: 3
```

**Register Mapping:**

| Address | Type | Parameter | Unit | Scale |
|---------|------|-----------|------|-------|
| 40001 | Holding | pH Value | pH | 0.01 |
| 40002 | Holding | EC Value | mS/cm | 0.01 |
| 40003 | Holding | Temperature | °C | 0.1 |
| 40004 | Holding | DO Value | mg/L | 0.1 |
| 40005 | Holding | Water Level | % | 1 |

**Reading Example (Python):**

```python
from pymodbus.client import ModbusTcpClient

client = ModbusTcpClient('192.168.1.100', port=502)
client.connect()

# Read pH value from register 40001
result = client.read_holding_registers(address=40001, count=1, slave=1)
ph_value = result.registers[0] / 100.0  # Scale by 0.01
print(f"pH: {ph_value}")

client.close()
```

---

## 7. Security Protocols

### 7.1 TLS/SSL (Transport Layer Security)

**Requirements:**
- TLS 1.2 or higher
- Strong cipher suites
- Certificate validation
- Perfect Forward Secrecy (PFS)

**Recommended Cipher Suites:**

```
TLS_ECDHE_RSA_WITH_AES_256_GCM_SHA384
TLS_ECDHE_RSA_WITH_AES_128_GCM_SHA256
TLS_ECDHE_ECDSA_WITH_AES_256_GCM_SHA384
```

### 7.2 DTLS (Datagram Transport Layer Security)

For UDP-based protocols (CoAP):

```yaml
dtls:
  version: 1.2
  cipher_suite: TLS_ECDHE_ECDSA_WITH_AES_128_CCM_8
  certificate: /path/to/cert.pem
  private_key: /path/to/key.pem
```

### 7.3 Authentication Mechanisms

**API Key Authentication:**

```http
Authorization: Bearer EXAMPLE_API_KEY_REPLACE_ME
```

**JWT Token Authentication:**

```http
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

**Client Certificate Authentication:**

```bash
curl --cert client-cert.pem --key client-key.pem \
  https://api.wia.org/agri027/v1/system/status
```

---

## 8. Data Compression

### 8.1 HTTP Compression

**Supported Encodings:**

```http
Accept-Encoding: gzip, deflate, br
```

**Server Response:**

```http
Content-Encoding: gzip
```

### 8.2 MQTT Payload Compression

For bandwidth-constrained environments:

**MessagePack:**

```python
import msgpack

data = {
  "deviceId": "SENSOR-PH-001",
  "value": 6.2,
  "timestamp": "2025-12-26T10:30:00Z"
}

# Compress
payload = msgpack.packb(data)
mqtt_client.publish("wia/agri027/farm-001/sensors/ph", payload)

# Decompress
received_data = msgpack.unpackb(payload)
```

---

## 9. Protocol Selection Guidelines

### 9.1 Decision Matrix

| Use Case | Recommended Protocol | Reason |
|----------|---------------------|--------|
| Real-time sensor streaming | MQTT | Low latency, efficient |
| Dashboard monitoring | WebSocket | Bidirectional, real-time |
| Configuration management | REST/HTTP | Standard, widely supported |
| Battery-powered sensors | CoAP | Low power, lightweight |
| Industrial integration | Modbus | Legacy compatibility |
| Mobile applications | REST/HTTP + WebSocket | Flexibility, real-time updates |

### 9.2 Bandwidth Comparison

| Protocol | Typical Bandwidth | Overhead | Best For |
|----------|------------------|----------|----------|
| MQTT | Low (2 bytes + payload) | Minimal | IoT sensors |
| WebSocket | Low-Medium | Low | Real-time apps |
| HTTP/REST | Medium-High | Moderate | CRUD operations |
| CoAP | Very Low | Minimal | Constrained devices |
| Modbus | Low | Low | Industrial sensors |

---

## 10. Reliability & Fault Tolerance

### 10.1 Retry Mechanisms

**Exponential Backoff:**

```python
import time

def send_with_retry(publish_func, max_retries=3):
    for attempt in range(max_retries):
        try:
            publish_func()
            return True
        except Exception as e:
            wait_time = 2 ** attempt  # Exponential backoff
            print(f"Retry {attempt + 1} after {wait_time}s")
            time.sleep(wait_time)
    return False
```

### 10.2 Message Queuing

**Persistent Sessions (MQTT):**

```python
mqtt_client.connect("mqtt.wia.org", 8883, keepalive=60)
mqtt_client.publish(
    "wia/agri027/sensors/ph",
    payload,
    qos=1,
    retain=True  # Retain last known good value
)
```

### 10.3 Heartbeat Mechanism

**Keep-Alive Messages:**

```json
{
  "type": "heartbeat",
  "deviceId": "SENSOR-PH-001",
  "timestamp": "2025-12-26T10:30:00Z",
  "status": "online"
}
```

**Interval:** Every 60 seconds

---

## 11. Protocol Implementation Examples

### 11.1 Python MQTT Client

```python
import paho.mqtt.client as mqtt
import json
from datetime import datetime

def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    client.subscribe("wia/agri027/farm-001/hydro-sys-001/sensors/#")

def on_message(client, userdata, msg):
    data = json.loads(msg.payload.decode())
    print(f"Received: {data}")

client = mqtt.Client()
client.username_pw_set("api_key", "YOUR_API_KEY")
client.tls_set()  # Enable TLS
client.on_connect = on_connect
client.on_message = on_message

client.connect("mqtt.wia.org", 8883, 60)
client.loop_forever()
```

### 11.2 JavaScript WebSocket Client

```javascript
const WebSocket = require('ws');

const ws = new WebSocket('wss://ws.wia.org/agri027/v1/stream');

ws.on('open', function open() {
  // Authenticate
  ws.send(JSON.stringify({
    type: 'auth',
    token: 'YOUR_API_KEY'
  }));

  // Subscribe
  ws.send(JSON.stringify({
    type: 'subscribe',
    systemId: 'HYDRO-SYS-001',
    channels: ['sensors', 'alerts']
  }));
});

ws.on('message', function incoming(data) {
  const message = JSON.parse(data);
  console.log('Received:', message);

  if (message.type === 'sensor_data') {
    console.log(`${message.deviceId}: ${message.value} ${message.unit}`);
  }
});

ws.on('error', function error(err) {
  console.error('WebSocket error:', err);
});
```

---

## 12. Performance Optimization

### 12.1 Connection Pooling

Reuse connections to reduce overhead:

```python
from requests.adapters import HTTPAdapter
from requests.packages.urllib3.util.retry import Retry
import requests

session = requests.Session()
retry = Retry(total=3, backoff_factor=1)
adapter = HTTPAdapter(max_retries=retry, pool_connections=10, pool_maxsize=20)
session.mount('https://', adapter)

response = session.get('https://api.wia.org/agri027/v1/system/status')
```

### 12.2 Batch Operations

Reduce API calls by batching:

```json
POST /api/v1/sensors/batch

{
  "readings": [
    {"deviceId": "SENSOR-PH-001", "value": 6.2, "timestamp": "2025-12-26T10:30:00Z"},
    {"deviceId": "SENSOR-EC-001", "value": 2.1, "timestamp": "2025-12-26T10:30:00Z"},
    {"deviceId": "SENSOR-TEMP-001", "value": 21.5, "timestamp": "2025-12-26T10:30:00Z"}
  ]
}
```

---

## 13. Monitoring & Diagnostics

### 13.1 Protocol Health Metrics

Monitor these metrics for each protocol:

- Connection uptime
- Message delivery rate
- Average latency
- Error rate
- Retry count

### 13.2 Diagnostic Tools

**MQTT:**

```bash
mosquitto_sub -h mqtt.wia.org -p 8883 -t "wia/agri027/#" -u api_key -P YOUR_API_KEY --cafile ca.crt
```

**WebSocket:**

```bash
wscat -c wss://ws.wia.org/agri027/v1/stream
```

**REST API:**

```bash
curl -X GET https://api.wia.org/agri027/v1/health \
  -H "Authorization: Bearer YOUR_API_KEY"
```

---

## 14. Conclusion

The WIA-AGRI-027 Phase 3 communication protocols provide a comprehensive framework for reliable, secure, and efficient data exchange in hydroponic systems. By supporting multiple protocols, we ensure compatibility with diverse hardware and software ecosystems.

---

**Document Information:**

- **Standard ID:** WIA-AGRI-027
- **Phase:** 3 - Communication Protocols
- **Status:** Active
- **Maintained by:** WIA (World Certification Industry Association)
- **Contact:** protocols@wia.org

弘益人間 (홍익인간) - Benefit All Humanity

© 2025 SmileStory Inc. / WIA
