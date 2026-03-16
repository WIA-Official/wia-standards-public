# WIA-AGRI-020: Agricultural Data Exchange Standard
## Phase 3: Communication Protocol Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-12-26

---

## 1. Overview

This specification defines communication protocols, transport mechanisms, and data streaming standards for agricultural data exchange across heterogeneous systems.

### 1.1 Protocol Objectives

- **Reliability**: Guaranteed delivery for critical agricultural data
- **Efficiency**: Optimized for bandwidth-constrained rural networks
- **Real-time**: Sub-second latency for time-sensitive operations
- **Scalability**: Support from single sensors to large-scale deployments
- **Interoperability**: Compatible with existing IoT and agricultural platforms

---

## 2. Transport Protocols

### 2.1 HTTP/HTTPS (RESTful)

**Use Cases:**
- API integrations
- Web applications
- Mobile apps
- Manual data submissions

**Configuration:**
```yaml
protocol: https
endpoint: https://api.wia-agri.org/v1
port: 443
tls: 1.3
compression: gzip
timeout: 30000
retryAttempts: 3
```

**Request Example:**
```http
POST /api/v1/data/submit HTTP/1.1
Host: api.wia-agri.org
Content-Type: application/json
Content-Encoding: gzip
X-API-Key: wia_live_abc123def456ghi789
User-Agent: WIA-AGRI-Client/1.0

{compressed JSON payload}
```

**Response Headers:**
```http
HTTP/1.1 201 Created
Content-Type: application/json
X-RateLimit-Remaining: 999
Cache-Control: no-cache
```

### 2.2 MQTT (IoT Messaging)

**Use Cases:**
- IoT sensor networks
- Real-time telemetry
- Edge device communication
- Low-power sensors

**Broker Configuration:**
```yaml
protocol: mqtt
broker: mqtt.wia-agri.org
port: 1883 (TCP) / 8883 (TLS)
websockets: 9001 (ws) / 9002 (wss)
qos: 1
keepAlive: 60
cleanSession: false
```

**Topic Structure:**
```
wia-agri/
├── {farmId}/
│   ├── sensors/
│   │   ├── soil/{deviceId}/data
│   │   ├── weather/{deviceId}/data
│   │   └── crop/{deviceId}/data
│   ├── equipment/
│   │   └── {equipmentId}/telemetry
│   └── alerts/
│       └── {alertType}
```

**Publish Example:**
```python
import paho.mqtt.client as mqtt

client = mqtt.Client(client_id="SENSOR-SOIL-001")
client.username_pw_set("farmId", "api_token")
client.tls_set()  # Enable TLS
client.connect("mqtt.wia-agri.org", 8883, 60)

topic = "wia-agri/FARM-2025-001/sensors/soil/SENSOR-SOIL-001/data"
payload = {
    "timestamp": "2025-12-26T10:30:00.000Z",
    "readings": {
        "soilMoisture": {"value": 45.5}
    }
}

client.publish(topic, json.dumps(payload), qos=1)
```

**Subscribe Example:**
```python
def on_message(client, userdata, msg):
    data = json.loads(msg.payload)
    print(f"Received from {msg.topic}: {data}")

client.subscribe("wia-agri/FARM-2025-001/sensors/#")
client.on_message = on_message
client.loop_forever()
```

**QoS Levels:**
- **QoS 0**: At most once (weather updates, non-critical)
- **QoS 1**: At least once (sensor readings, recommended)
- **QoS 2**: Exactly once (irrigation commands, critical)

### 2.3 WebSocket (Real-Time Streaming)

**Use Cases:**
- Real-time dashboards
- Live monitoring
- Bidirectional communication
- Browser-based applications

**Connection:**
```javascript
const ws = new WebSocket('wss://stream.wia-agri.org/v1/stream');

ws.onopen = () => {
  ws.send(JSON.stringify({
    action: 'authenticate',
    token: 'wia_live_abc123def456ghi789'
  }));

  ws.send(JSON.stringify({
    action: 'subscribe',
    farmId: 'FARM-2025-001',
    channels: ['sensors', 'alerts']
  }));
};

ws.onmessage = (event) => {
  const message = JSON.parse(event.data);
  handleRealtimeData(message);
};

ws.onerror = (error) => {
  console.error('WebSocket error:', error);
};

ws.onclose = () => {
  console.log('WebSocket connection closed');
  setTimeout(reconnect, 5000);
};
```

**Message Types:**
```json
{
  "type": "data",
  "timestamp": "2025-12-26T10:30:00.000Z",
  "channel": "sensors",
  "payload": {
    "dataType": "SOIL_SENSOR",
    "readings": {...}
  }
}
```

```json
{
  "type": "alert",
  "severity": "HIGH",
  "message": "Soil moisture below threshold",
  "fieldId": "FIELD-NORTH-01"
}
```

```json
{
  "type": "ping",
  "timestamp": "2025-12-26T10:30:00.000Z"
}
```

### 2.4 CoAP (Constrained Application Protocol)

**Use Cases:**
- Battery-powered sensors
- Low-bandwidth networks
- Edge devices with limited resources

**Configuration:**
```yaml
protocol: coap
endpoint: coap://coap.wia-agri.org
port: 5683 (UDP) / 5684 (DTLS)
blockSize: 1024
maxRetransmit: 4
ackTimeout: 2000
```

**Request Example:**
```
CON POST coap://coap.wia-agri.org/data/submit
Token: 0x7d38
Content-Format: application/json

{"farmId":"FARM-2025-001","readings":{...}}
```

**Response:**
```
ACK 2.01 Created
Token: 0x7d38
Content-Format: application/json

{"status":"success","dataId":"DATA-20251226-001"}
```

---

## 3. Data Encoding & Serialization

### 3.1 JSON (Default)

**Advantages:**
- Human-readable
- Universal support
- Easy debugging

**Example:**
```json
{
  "timestamp": "2025-12-26T10:30:00.000Z",
  "readings": {
    "soilMoisture": {"value": 45.5},
    "temperature": {"value": 24.5}
  }
}
```

**Size:** ~120 bytes (uncompressed)

### 3.2 Protocol Buffers (High Volume)

**Advantages:**
- 3-10x smaller than JSON
- Fast serialization
- Strongly typed

**Schema Definition (.proto):**
```protobuf
syntax = "proto3";

message SensorReading {
  string data_id = 1;
  int64 timestamp = 2;
  string farm_id = 3;
  string data_type = 4;

  message Reading {
    double value = 1;
    string unit = 2;
  }

  map<string, Reading> readings = 5;
}
```

**Binary Size:** ~35 bytes (70% reduction)

### 3.3 MessagePack (Compact JSON)

**Advantages:**
- Binary JSON alternative
- Smaller than JSON
- Easy migration from JSON

**Example:**
```javascript
const msgpack = require('msgpack-lite');

const data = {
  timestamp: "2025-12-26T10:30:00.000Z",
  readings: {
    soilMoisture: {value: 45.5}
  }
};

const encoded = msgpack.encode(data);  // Binary format
const decoded = msgpack.decode(encoded);
```

**Size:** ~80 bytes (33% reduction from JSON)

### 3.4 CBOR (Concise Binary Object Representation)

**Advantages:**
- Self-describing
- Extensible
- IoT-optimized

**Use with CoAP protocol for maximum efficiency**

---

## 4. Compression

### 4.1 HTTP Compression

**Supported Algorithms:**
```http
Accept-Encoding: gzip, br
```

**Response:**
```http
HTTP/1.1 200 OK
Content-Encoding: gzip
Content-Type: application/json
```

**Compression Ratios:**
- GZIP: 70-80% reduction
- Brotli: 75-85% reduction

### 4.2 MQTT Compression

Enable at broker level or use compressed payloads:
```python
import gzip
import json

data = {"readings": {...}}
compressed = gzip.compress(json.dumps(data).encode())
client.publish(topic, compressed, qos=1)
```

---

## 5. Security Protocols

### 5.1 TLS/SSL Encryption

**Minimum Requirements:**
- TLS 1.2 or higher (TLS 1.3 recommended)
- Strong cipher suites (AES-256-GCM)
- Valid SSL certificates (Let's Encrypt)

**Configuration:**
```yaml
tls:
  version: 1.3
  ciphers:
    - TLS_AES_256_GCM_SHA384
    - TLS_CHACHA20_POLY1305_SHA256
  verifyPeer: true
  verifyHostname: true
```

### 5.2 DTLS (for CoAP)

Datagram TLS for UDP-based CoAP:
```yaml
dtls:
  version: 1.2
  ciphers: [TLS_ECDHE_ECDSA_WITH_AES_128_CCM_8]
  psk: true  # Pre-shared keys
```

### 5.3 Message Authentication

**HMAC Signing:**
```python
import hmac
import hashlib

def sign_message(message, secret_key):
    signature = hmac.new(
        secret_key.encode(),
        message.encode(),
        hashlib.sha256
    ).hexdigest()
    return signature

# Add signature to message
message['signature'] = sign_message(json.dumps(message), api_secret)
```

**Verification:**
```python
def verify_signature(message, signature, secret_key):
    expected = sign_message(json.dumps(message), secret_key)
    return hmac.compare_digest(expected, signature)
```

---

## 6. Connection Management

### 6.1 Reconnection Strategy

**Exponential Backoff:**
```python
import time

def reconnect_with_backoff(max_attempts=10):
    attempt = 0
    delay = 1  # Start with 1 second

    while attempt < max_attempts:
        try:
            connect()
            return True
        except ConnectionError:
            attempt += 1
            wait_time = min(delay * (2 ** attempt), 300)  # Max 5 min
            time.sleep(wait_time)

    return False
```

### 6.2 Keep-Alive

**MQTT:**
```yaml
keepAlive: 60  # Send PINGREQ every 60 seconds
```

**WebSocket:**
```javascript
setInterval(() => {
  if (ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({type: 'ping'}));
  }
}, 30000);  // Ping every 30 seconds
```

### 6.3 Heartbeat Messages

```json
{
  "type": "heartbeat",
  "deviceId": "SENSOR-SOIL-001",
  "timestamp": "2025-12-26T10:30:00.000Z",
  "status": "online",
  "batteryLevel": 78,
  "signalStrength": -65
}
```

---

## 7. Flow Control & Congestion

### 7.1 Buffering Strategy

**Edge Device Buffer:**
```javascript
const buffer = [];
const MAX_BUFFER_SIZE = 1000;

function bufferReading(reading) {
  if (buffer.length >= MAX_BUFFER_SIZE) {
    buffer.shift();  // Remove oldest
  }
  buffer.push(reading);
}

function flushBuffer() {
  if (isConnected() && buffer.length > 0) {
    sendBatch(buffer);
    buffer.length = 0;
  }
}
```

### 7.2 Rate Limiting (Client-Side)

```python
import time
from collections import deque

class RateLimiter:
    def __init__(self, max_calls, period):
        self.max_calls = max_calls
        self.period = period
        self.calls = deque()

    def allow_request(self):
        now = time.time()

        # Remove old calls outside the period
        while self.calls and self.calls[0] < now - self.period:
            self.calls.popleft()

        if len(self.calls) < self.max_calls:
            self.calls.append(now)
            return True
        return False

limiter = RateLimiter(max_calls=10, period=60)  # 10 calls/min

if limiter.allow_request():
    send_data()
else:
    buffer_data()
```

---

## 8. Protocol Selection Guide

| Protocol | Latency | Bandwidth | Power | Complexity | Best For |
|----------|---------|-----------|-------|------------|----------|
| **HTTPS** | Medium | High | High | Low | APIs, web apps |
| **MQTT** | Low | Low | Low | Medium | IoT sensors |
| **WebSocket** | Very Low | Medium | Medium | Medium | Real-time dashboards |
| **CoAP** | Low | Very Low | Very Low | High | Battery sensors |

---

## 9. Offline Operation

### 9.1 Store-and-Forward

```javascript
class OfflineStorage {
  constructor() {
    this.queue = [];
  }

  async save(data) {
    this.queue.push(data);
    await this.persistToLocalStorage();
  }

  async sync() {
    while (this.queue.length > 0 && isOnline()) {
      const data = this.queue.shift();
      try {
        await sendToServer(data);
      } catch (error) {
        this.queue.unshift(data);  // Put back on failure
        break;
      }
    }
  }
}
```

### 9.2 Conflict Resolution

**Last-Write-Wins:**
```javascript
function resolveConflict(local, remote) {
  return local.timestamp > remote.timestamp ? local : remote;
}
```

**Custom Merge:**
```javascript
function mergeReadings(local, remote) {
  return {
    ...remote,
    readings: {
      ...remote.readings,
      ...local.readings
    }
  };
}
```

---

## 10. Monitoring & Diagnostics

### 10.1 Protocol Metrics

```json
{
  "metrics": {
    "protocol": "mqtt",
    "messagesPublished": 1245,
    "messagesReceived": 892,
    "bytesTransmitted": 45678,
    "averageLatency": 24,
    "connectionUptime": 86400,
    "reconnectCount": 0,
    "errorRate": 0.001
  }
}
```

### 10.2 Health Check Endpoint

```http
GET /health HTTP/1.1
```

**Response:**
```json
{
  "status": "healthy",
  "uptime": 86400,
  "protocols": {
    "https": "operational",
    "mqtt": "operational",
    "websocket": "operational"
  },
  "timestamp": "2025-12-26T10:30:00.000Z"
}
```

---

## 11. Implementation Checklist

- [ ] Select appropriate protocol(s) for use case
- [ ] Implement TLS/DTLS encryption
- [ ] Configure reconnection with exponential backoff
- [ ] Add keep-alive/heartbeat mechanisms
- [ ] Implement client-side rate limiting
- [ ] Add offline storage and sync
- [ ] Set up monitoring and metrics
- [ ] Test protocol failover scenarios
- [ ] Optimize compression and encoding
- [ ] Document protocol configuration

---

**Next Phase:** [PHASE-4-INTEGRATION.md](./PHASE-4-INTEGRATION.md)

---

© 2025 WIA Standards · MIT License
弘益人間 · Benefit All Humanity
