# WIA Ocean Plastic Track Protocol Standard
## Phase 3 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #84CC16 (Lime)

---

## Table of Contents

1. [Overview](#overview)
2. [Transport Layer](#transport-layer)
3. [Message Format](#message-format)
4. [Connection Lifecycle](#connection-lifecycle)
5. [IoT Device Protocol](#iot-device-protocol)
6. [Security](#security)
7. [Quality of Service](#quality-of-service)
8. [Error Handling](#error-handling)
9. [Version History](#version-history)

---

## Overview

### 1.1 Purpose

The WIA Ocean Plastic Track Protocol Standard defines communication protocols for real-time data exchange between IoT collection devices, field operators, recycling facilities, and central tracking systems, ensuring reliable and secure data transmission across diverse network conditions.

**Core Objectives**:
- Enable real-time tracking of plastic collection in remote ocean locations
- Support low-bandwidth IoT devices with intermittent connectivity
- Ensure data integrity and security during transmission
- Provide fallback mechanisms for offline operation

### 1.2 Protocol Stack

```
┌─────────────────────────────────────┐
│     Application Layer               │
│  (Batch Data, Sensor Readings)      │
├─────────────────────────────────────┤
│     Message Layer                   │
│  (JSON, Protocol Buffers, CBOR)     │
├─────────────────────────────────────┤
│     Transport Layer                 │
│  (MQTT, WebSocket, HTTP/2)          │
├─────────────────────────────────────┤
│     Security Layer                  │
│  (TLS 1.3, mTLS, Device Auth)       │
├─────────────────────────────────────┤
│     Network Layer                   │
│  (TCP/IP, 4G/5G, Satellite)         │
└─────────────────────────────────────┘
```

### 1.3 Supported Protocols

| Protocol | Use Case | Network | Priority |
|----------|----------|---------|----------|
| MQTT 5.0 | IoT sensors, real-time tracking | 4G/5G/Satellite | High |
| WebSocket | Web/mobile apps, dashboards | WiFi/4G/5G | High |
| HTTP/2 | API integration, batch uploads | All | Medium |
| CoAP | Low-power IoT devices | LoRaWAN/NB-IoT | Medium |
| Satellite | Remote ocean locations | Iridium/Starlink | Low |

---

## Transport Layer

### 2.1 MQTT for IoT Devices

#### 2.1.1 Connection Configuration

```json
{
  "broker": {
    "host": "mqtt.wia.live",
    "port": 8883,
    "protocol": "mqtts",
    "keepAlive": 60,
    "cleanSession": false,
    "clientId": "IOT-DEV-{deviceId}"
  },
  "authentication": {
    "username": "{deviceId}",
    "password": "{deviceToken}"
  },
  "tls": {
    "version": "1.3",
    "caCert": "/path/to/ca.crt",
    "clientCert": "/path/to/client.crt",
    "clientKey": "/path/to/client.key"
  }
}
```

#### 2.1.2 Topic Structure

```
wia/ocean-plastic-track/{version}/{deviceId}/{messageType}

Examples:
- wia/ocean-plastic-track/v1/IOT-DEV-001/sensor
- wia/ocean-plastic-track/v1/IOT-DEV-001/batch
- wia/ocean-plastic-track/v1/IOT-DEV-001/status
- wia/ocean-plastic-track/v1/IOT-DEV-001/command
```

**Topic Patterns:**

| Topic | QoS | Retained | Purpose |
|-------|-----|----------|---------|
| `{prefix}/sensor` | 1 | false | Sensor data streams |
| `{prefix}/batch` | 2 | false | Batch creation/updates |
| `{prefix}/status` | 1 | true | Device status heartbeat |
| `{prefix}/command` | 2 | false | Remote commands |
| `{prefix}/response` | 1 | false | Command responses |

#### 2.1.3 MQTT Connection Example

```python
import paho.mqtt.client as mqtt
import json
import ssl

# MQTT Configuration
BROKER = "mqtt.wia.live"
PORT = 8883
DEVICE_ID = "IOT-DEV-001"
DEVICE_TOKEN = "device_token_here"

# Callbacks
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected successfully")
        # Subscribe to command topic
        client.subscribe(f"wia/ocean-plastic-track/v1/{DEVICE_ID}/command", qos=2)
    else:
        print(f"Connection failed with code {rc}")

def on_message(client, userdata, msg):
    print(f"Received: {msg.topic} - {msg.payload.decode()}")
    # Process command
    handle_command(json.loads(msg.payload))

def on_publish(client, userdata, mid):
    print(f"Message {mid} published")

# Create client
client = mqtt.Client(
    client_id=f"IOT-DEV-{DEVICE_ID}",
    clean_session=False,
    protocol=mqtt.MQTTv5
)

# Set authentication
client.username_pw_set(DEVICE_ID, DEVICE_TOKEN)

# Configure TLS
client.tls_set(
    ca_certs="/path/to/ca.crt",
    certfile="/path/to/client.crt",
    keyfile="/path/to/client.key",
    tls_version=ssl.PROTOCOL_TLSv1_3
)

# Set callbacks
client.on_connect = on_connect
client.on_message = on_message
client.on_publish = on_publish

# Connect
client.connect(BROKER, PORT, keepalive=60)

# Publish sensor data
sensor_data = {
    "timestamp": "2025-01-15T10:30:00Z",
    "sensors": [
        {
            "type": "gps",
            "data": {
                "latitude": 37.5665,
                "longitude": 126.9780,
                "accuracy": 5.0
            }
        }
    ]
}

client.publish(
    f"wia/ocean-plastic-track/v1/{DEVICE_ID}/sensor",
    json.dumps(sensor_data),
    qos=1
)

# Start loop
client.loop_forever()
```

#### 2.1.4 MQTT Message Format

```json
{
  "header": {
    "version": "1.0.0",
    "messageId": "msg-abc123",
    "timestamp": "2025-01-15T10:30:00Z",
    "deviceId": "IOT-DEV-001",
    "messageType": "sensor_data"
  },
  "payload": {
    "sensors": [
      {
        "type": "gps",
        "data": {
          "latitude": 37.5665,
          "longitude": 126.9780
        }
      }
    ]
  },
  "signature": "sha256:abc123..."
}
```

### 2.2 WebSocket for Real-time Applications

#### 2.2.1 WebSocket Connection

```javascript
// Client-side WebSocket connection
const ws = new WebSocket('wss://ws.wia.live/ocean-plastic-track/v1');

ws.onopen = () => {
  console.log('WebSocket connected');

  // Authenticate
  ws.send(JSON.stringify({
    type: 'auth',
    token: 'your_jwt_token_here'
  }));

  // Subscribe to batch updates
  ws.send(JSON.stringify({
    type: 'subscribe',
    channels: ['batch.updates', 'device.status']
  }));
};

ws.onmessage = (event) => {
  const message = JSON.parse(event.data);
  console.log('Received:', message);

  switch (message.type) {
    case 'batch.created':
      handleBatchCreated(message.data);
      break;
    case 'batch.status_changed':
      handleStatusChange(message.data);
      break;
    case 'device.status':
      updateDeviceStatus(message.data);
      break;
  }
};

ws.onerror = (error) => {
  console.error('WebSocket error:', error);
};

ws.onclose = (event) => {
  console.log('WebSocket closed:', event.code, event.reason);
  // Implement reconnection logic
  setTimeout(() => reconnect(), 5000);
};

// Send heartbeat
setInterval(() => {
  ws.send(JSON.stringify({ type: 'ping' }));
}, 30000);
```

#### 2.2.2 WebSocket Message Types

```json
// Authentication
{
  "type": "auth",
  "token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9..."
}

// Subscribe to channels
{
  "type": "subscribe",
  "channels": ["batch.updates", "device.IOT-DEV-001"]
}

// Batch update notification
{
  "type": "batch.status_changed",
  "timestamp": "2025-01-15T10:30:00Z",
  "data": {
    "batchId": "BATCH-2025-000001",
    "previousStatus": "collected",
    "currentStatus": "transported"
  }
}

// Device status update
{
  "type": "device.status",
  "timestamp": "2025-01-15T10:30:00Z",
  "data": {
    "deviceId": "IOT-DEV-001",
    "status": "active",
    "location": {
      "latitude": 37.5665,
      "longitude": 126.9780
    },
    "battery": 85
  }
}

// Heartbeat
{
  "type": "ping",
  "timestamp": "2025-01-15T10:30:00Z"
}

// Response
{
  "type": "pong",
  "timestamp": "2025-01-15T10:30:01Z"
}
```

### 2.3 HTTP/2 for Batch Operations

#### 2.3.1 HTTP/2 Configuration

```
Protocol: HTTP/2 over TLS 1.3
Server: h2.wia.live
Port: 443
Features:
  - Multiplexing: Multiple requests on single connection
  - Server Push: Proactive resource delivery
  - Header Compression: HPACK compression
  - Stream Prioritization: Priority-based delivery
```

#### 2.3.2 HTTP/2 Example

```javascript
const http2 = require('http2');
const fs = require('fs');

const client = http2.connect('https://h2.wia.live', {
  ca: fs.readFileSync('ca.crt')
});

client.on('error', (err) => console.error(err));

// Create batch request
const req = client.request({
  ':method': 'POST',
  ':path': '/ocean-plastic-track/v1/batches',
  'authorization': 'Bearer YOUR_TOKEN',
  'content-type': 'application/json'
});

req.setEncoding('utf8');

let data = '';
req.on('data', (chunk) => { data += chunk; });
req.on('end', () => {
  console.log('Response:', JSON.parse(data));
  client.close();
});

req.write(JSON.stringify({
  collection: { /* ... */ },
  material: { /* ... */ }
}));
req.end();
```

### 2.4 CoAP for Low-Power Devices

#### 2.4.1 CoAP Configuration

```
Protocol: CoAP (RFC 7252)
Transport: UDP
Port: 5684 (CoAPS)
DTLS: 1.3

Endpoints:
  coaps://coap.wia.live:5684/v1/batches
  coaps://coap.wia.live:5684/v1/sensors
```

#### 2.4.2 CoAP Example

```python
from aiocoap import *

async def send_sensor_data():
    protocol = await Context.create_client_context()

    payload = cbor2.dumps({
        "deviceId": "IOT-DEV-001",
        "sensors": [{
            "type": "gps",
            "data": {"lat": 37.5665, "lon": 126.9780}
        }]
    })

    request = Message(
        code=POST,
        uri='coaps://coap.wia.live:5684/v1/sensors',
        payload=payload
    )

    response = await protocol.request(request).response
    print(f'Result: {response.code}')
```

---

## Message Format

### 3.1 JSON (Default Format)

```json
{
  "header": {
    "version": "1.0.0",
    "messageId": "msg-abc123",
    "timestamp": "2025-01-15T10:30:00Z",
    "deviceId": "IOT-DEV-001",
    "messageType": "batch_create",
    "compression": "none",
    "encoding": "utf-8"
  },
  "payload": {
    "collection": { /* ... */ },
    "material": { /* ... */ }
  },
  "checksum": "crc32:abc123",
  "signature": "sha256:def456"
}
```

### 3.2 Protocol Buffers (Binary Format)

```protobuf
syntax = "proto3";

package wia.ocean_plastic_track.v1;

message SensorData {
  string message_id = 1;
  string device_id = 2;
  int64 timestamp = 3;

  repeated Sensor sensors = 4;
}

message Sensor {
  string sensor_id = 1;
  string type = 2;

  oneof data {
    GPSData gps = 3;
    EnvironmentalData environmental = 4;
    WeightData weight = 5;
  }
}

message GPSData {
  double latitude = 1;
  double longitude = 2;
  double altitude = 3;
  double accuracy = 4;
  double speed = 5;
  double heading = 6;
}

message EnvironmentalData {
  double water_temperature = 1;
  double salinity = 2;
  double ph = 3;
  double turbidity = 4;
}

message WeightData {
  double current_weight = 1;
  double max_capacity = 2;
  double percent_full = 3;
}
```

**Usage Example:**

```python
import sensor_pb2

# Create sensor data
sensor_data = sensor_pb2.SensorData()
sensor_data.message_id = "msg-abc123"
sensor_data.device_id = "IOT-DEV-001"
sensor_data.timestamp = int(time.time())

# Add GPS sensor
gps_sensor = sensor_data.sensors.add()
gps_sensor.sensor_id = "GPS-001"
gps_sensor.type = "gps"
gps_sensor.gps.latitude = 37.5665
gps_sensor.gps.longitude = 126.9780

# Serialize to binary
binary_data = sensor_data.SerializeToString()

# Send via MQTT
client.publish(topic, binary_data, qos=1)
```

### 3.3 CBOR (Concise Binary Object Representation)

```python
import cbor2

# Create message
message = {
    "header": {
        "v": "1.0.0",
        "mid": "msg-abc123",
        "ts": 1642253400,
        "did": "IOT-DEV-001"
    },
    "payload": {
        "sensors": [{
            "t": "gps",
            "d": {"lat": 37.5665, "lon": 126.9780}
        }]
    }
}

# Encode to CBOR
cbor_data = cbor2.dumps(message)
print(f"JSON size: {len(json.dumps(message))} bytes")
print(f"CBOR size: {len(cbor_data)} bytes")

# Decode
decoded = cbor2.loads(cbor_data)
```

**Size Comparison:**

| Format | Size (bytes) | Encoding Time | Use Case |
|--------|--------------|---------------|----------|
| JSON | 450 | Fast | Human-readable, debugging |
| Protocol Buffers | 180 | Medium | Efficient, typed |
| CBOR | 210 | Fast | Compact, simple |
| MessagePack | 200 | Fast | Alternative to CBOR |

---

## Connection Lifecycle

### 4.1 Device Registration

```
1. Device Initialization
   ↓
2. Generate Device Credentials
   ↓
3. Register with Server
   ↓
4. Receive Device Token
   ↓
5. Store Token Securely
   ↓
6. Ready for Operation
```

**Registration Flow:**

```http
POST /v1/devices/register
Content-Type: application/json

{
  "deviceId": "IOT-DEV-001",
  "deviceType": "vessel_tracker",
  "publicKey": "ed25519:abc123...",
  "organizationId": "ORG-CLEANUP-001"
}

Response:
{
  "deviceId": "IOT-DEV-001",
  "deviceToken": "dt_abc123...",
  "mqttBroker": "mqtt.wia.live",
  "mqttPort": 8883,
  "certificateChain": "-----BEGIN CERTIFICATE-----\n..."
}
```

### 4.2 Connection Establishment

```
1. TCP Connection
   ↓
2. TLS Handshake (1.3)
   ↓
3. MQTT CONNECT (with auth)
   ↓
4. Server CONNACK
   ↓
5. Subscribe to Topics
   ↓
6. Publish Status (retained)
   ↓
7. Active Connection
```

**MQTT Connect Packet:**

```python
connect_packet = {
    "clientId": "IOT-DEV-001",
    "keepAlive": 60,
    "cleanSession": False,
    "username": "IOT-DEV-001",
    "password": "device_token",
    "will": {
        "topic": "wia/ocean-plastic-track/v1/IOT-DEV-001/status",
        "message": '{"status": "offline"}',
        "qos": 1,
        "retained": True
    }
}
```

### 4.3 Heartbeat & Keep-Alive

```python
import time
import threading

class DeviceHeartbeat:
    def __init__(self, client, device_id, interval=30):
        self.client = client
        self.device_id = device_id
        self.interval = interval
        self.running = False

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._heartbeat_loop)
        self.thread.start()

    def _heartbeat_loop(self):
        while self.running:
            status = {
                "deviceId": self.device_id,
                "status": "active",
                "timestamp": time.time(),
                "battery": get_battery_level(),
                "signal": get_signal_strength()
            }

            self.client.publish(
                f"wia/ocean-plastic-track/v1/{self.device_id}/status",
                json.dumps(status),
                qos=1,
                retain=True
            )

            time.sleep(self.interval)

    def stop(self):
        self.running = False
        self.thread.join()
```

### 4.4 Disconnection & Reconnection

```python
import time
import random

class ReconnectHandler:
    def __init__(self, client, max_retries=10):
        self.client = client
        self.max_retries = max_retries
        self.retry_count = 0

    def on_disconnect(self, client, userdata, rc):
        if rc != 0:
            print(f"Unexpected disconnect: {rc}")
            self.reconnect()

    def reconnect(self):
        while self.retry_count < self.max_retries:
            try:
                # Exponential backoff with jitter
                wait_time = min(300, (2 ** self.retry_count) + random.uniform(0, 1))
                print(f"Reconnecting in {wait_time}s... (attempt {self.retry_count + 1})")
                time.sleep(wait_time)

                self.client.reconnect()
                self.retry_count = 0
                print("Reconnected successfully")
                return
            except Exception as e:
                print(f"Reconnection failed: {e}")
                self.retry_count += 1

        print("Max retries reached. Giving up.")

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("Connected")
            self.retry_count = 0
            # Re-subscribe to topics
            self.resubscribe()

    def resubscribe(self):
        topics = [
            ("wia/ocean-plastic-track/v1/IOT-DEV-001/command", 2),
            ("wia/ocean-plastic-track/v1/broadcasts", 1)
        ]
        for topic, qos in topics:
            self.client.subscribe(topic, qos)
```

### 4.5 Graceful Shutdown

```python
def graceful_shutdown(client, device_id):
    # Publish offline status
    offline_status = {
        "deviceId": device_id,
        "status": "offline",
        "timestamp": time.time(),
        "reason": "shutdown"
    }

    client.publish(
        f"wia/ocean-plastic-track/v1/{device_id}/status",
        json.dumps(offline_status),
        qos=1,
        retain=True
    )

    # Wait for message to be sent
    time.sleep(0.5)

    # Disconnect
    client.disconnect()

    # Wait for disconnect to complete
    client.loop_stop()

    print("Shutdown complete")
```

---

## IoT Device Protocol

### 5.1 Device States

```
┌──────────────┐
│ UNREGISTERED │
└──────┬───────┘
       │ register()
       ↓
┌──────────────┐
│  REGISTERED  │
└──────┬───────┘
       │ connect()
       ↓
┌──────────────┐
│  CONNECTING  │
└──────┬───────┘
       │ CONNACK
       ↓
┌──────────────┐
│    ACTIVE    │ ←──────┐
└──────┬───────┘        │
       │                │ reconnect()
       ├─ publish()     │
       ├─ subscribe()   │
       │                │
       ↓                │
┌──────────────┐        │
│    IDLE      │────────┘
└──────┬───────┘
       │ disconnect()
       ↓
┌──────────────┐
│   OFFLINE    │
└──────────────┘
```

### 5.2 Sensor Data Collection

```python
class SensorCollector:
    def __init__(self, client, device_id):
        self.client = client
        self.device_id = device_id
        self.sensors = []

    def add_sensor(self, sensor):
        self.sensors.append(sensor)

    def collect_and_publish(self):
        sensor_data = {
            "timestamp": time.time(),
            "sensors": []
        }

        for sensor in self.sensors:
            try:
                reading = sensor.read()
                sensor_data["sensors"].append({
                    "sensorId": sensor.id,
                    "type": sensor.type,
                    "data": reading,
                    "status": "active"
                })
            except Exception as e:
                print(f"Sensor {sensor.id} error: {e}")
                sensor_data["sensors"].append({
                    "sensorId": sensor.id,
                    "type": sensor.type,
                    "status": "error",
                    "error": str(e)
                })

        self.publish(sensor_data)

    def publish(self, data):
        topic = f"wia/ocean-plastic-track/v1/{self.device_id}/sensor"
        self.client.publish(topic, json.dumps(data), qos=1)
```

### 5.3 Offline Buffering

```python
import queue
import sqlite3

class OfflineBuffer:
    def __init__(self, db_path='offline_buffer.db', max_size=1000):
        self.db_path = db_path
        self.max_size = max_size
        self.init_db()

    def init_db(self):
        conn = sqlite3.connect(self.db_path)
        conn.execute('''
            CREATE TABLE IF NOT EXISTS buffer (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp REAL,
                topic TEXT,
                payload TEXT,
                qos INTEGER,
                retry_count INTEGER DEFAULT 0
            )
        ''')
        conn.commit()
        conn.close()

    def add(self, topic, payload, qos=1):
        conn = sqlite3.connect(self.db_path)
        conn.execute(
            'INSERT INTO buffer (timestamp, topic, payload, qos) VALUES (?, ?, ?, ?)',
            (time.time(), topic, payload, qos)
        )
        conn.commit()

        # Limit buffer size
        conn.execute(
            'DELETE FROM buffer WHERE id IN (SELECT id FROM buffer ORDER BY timestamp ASC LIMIT -1 OFFSET ?)',
            (self.max_size,)
        )
        conn.commit()
        conn.close()

    def get_pending(self, limit=100):
        conn = sqlite3.connect(self.db_path)
        cursor = conn.execute(
            'SELECT id, topic, payload, qos FROM buffer ORDER BY timestamp ASC LIMIT ?',
            (limit,)
        )
        messages = cursor.fetchall()
        conn.close()
        return messages

    def remove(self, message_id):
        conn = sqlite3.connect(self.db_path)
        conn.execute('DELETE FROM buffer WHERE id = ?', (message_id,))
        conn.commit()
        conn.close()

    def flush(self, client):
        """Send all buffered messages"""
        messages = self.get_pending()
        for msg_id, topic, payload, qos in messages:
            try:
                client.publish(topic, payload, qos=qos)
                self.remove(msg_id)
                print(f"Flushed message {msg_id}")
            except Exception as e:
                print(f"Failed to flush {msg_id}: {e}")
                break  # Stop if connection fails
```

---

## Security

### 6.1 Transport Layer Security

**TLS 1.3 Configuration:**

```
Protocol: TLS 1.3
Cipher Suites:
  - TLS_AES_256_GCM_SHA384
  - TLS_AES_128_GCM_SHA256
  - TLS_CHACHA20_POLY1305_SHA256

Certificate Validation:
  - Server certificate validation required
  - Client certificate for mTLS (optional)
  - Certificate pinning for critical devices
```

### 6.2 Device Authentication

```python
import jwt
import time

def create_device_jwt(device_id, private_key):
    payload = {
        "sub": device_id,
        "iat": int(time.time()),
        "exp": int(time.time()) + 86400,  # 24 hours
        "iss": "wia-ocean-plastic-track",
        "device": True
    }

    token = jwt.encode(payload, private_key, algorithm='ES256')
    return token

def verify_device_jwt(token, public_key):
    try:
        payload = jwt.decode(token, public_key, algorithms=['ES256'])
        return payload
    except jwt.ExpiredSignatureError:
        raise Exception("Token expired")
    except jwt.InvalidTokenError:
        raise Exception("Invalid token")
```

### 6.3 Message Signing

```python
import hmac
import hashlib

def sign_message(message, secret_key):
    """Sign message with HMAC-SHA256"""
    message_bytes = json.dumps(message, sort_keys=True).encode()
    signature = hmac.new(
        secret_key.encode(),
        message_bytes,
        hashlib.sha256
    ).hexdigest()

    return {
        "message": message,
        "signature": f"sha256:{signature}"
    }

def verify_signature(signed_message, secret_key):
    """Verify message signature"""
    message = signed_message["message"]
    expected_signature = signed_message["signature"].replace("sha256:", "")

    message_bytes = json.dumps(message, sort_keys=True).encode()
    calculated_signature = hmac.new(
        secret_key.encode(),
        message_bytes,
        hashlib.sha256
    ).hexdigest()

    return hmac.compare_digest(expected_signature, calculated_signature)
```

### 6.4 End-to-End Encryption

```python
from cryptography.hazmat.primitives.asymmetric import x25519
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.ciphers.aead import ChaCha20Poly1305

class E2EEncryption:
    def __init__(self):
        self.private_key = x25519.X25519PrivateKey.generate()
        self.public_key = self.private_key.public_key()

    def encrypt(self, plaintext, recipient_public_key):
        # Perform ECDH key exchange
        shared_secret = self.private_key.exchange(recipient_public_key)

        # Derive encryption key
        cipher = ChaCha20Poly1305(shared_secret[:32])

        # Encrypt
        nonce = os.urandom(12)
        ciphertext = cipher.encrypt(nonce, plaintext.encode(), None)

        return {
            "nonce": nonce.hex(),
            "ciphertext": ciphertext.hex(),
            "senderPublicKey": self.public_key.public_bytes(
                encoding=serialization.Encoding.Raw,
                format=serialization.PublicFormat.Raw
            ).hex()
        }

    def decrypt(self, encrypted_data, sender_public_key):
        # Perform ECDH key exchange
        shared_secret = self.private_key.exchange(sender_public_key)

        # Decrypt
        cipher = ChaCha20Poly1305(shared_secret[:32])
        plaintext = cipher.decrypt(
            bytes.fromhex(encrypted_data["nonce"]),
            bytes.fromhex(encrypted_data["ciphertext"]),
            None
        )

        return plaintext.decode()
```

---

## Quality of Service

### 7.1 MQTT QoS Levels

| QoS | Description | Use Case | Delivery Guarantee |
|-----|-------------|----------|-------------------|
| 0 | At most once | Non-critical sensor data | Best effort |
| 1 | At least once | Regular sensor data | Acknowledged |
| 2 | Exactly once | Batch creation, critical commands | Guaranteed |

### 7.2 Message Prioritization

```python
class MessagePriority:
    CRITICAL = 0   # Safety, emergency shutdown
    HIGH = 1       # Batch creation, alerts
    NORMAL = 2     # Regular sensor data
    LOW = 3        # Status updates, heartbeat

def publish_with_priority(client, topic, message, priority):
    mqtt_properties = {
        'user_property': [
            ('priority', str(priority)),
            ('timestamp', str(time.time()))
        ]
    }

    client.publish(
        topic,
        json.dumps(message),
        qos=get_qos_for_priority(priority),
        properties=mqtt_properties
    )

def get_qos_for_priority(priority):
    if priority == MessagePriority.CRITICAL:
        return 2
    elif priority == MessagePriority.HIGH:
        return 2
    elif priority == MessagePriority.NORMAL:
        return 1
    else:
        return 0
```

---

## Error Handling

### 8.1 Error Codes

| Code | Description | Recovery Action |
|------|-------------|----------------|
| `CONN_FAILED` | Connection failed | Retry with backoff |
| `AUTH_FAILED` | Authentication failed | Renew token |
| `PUB_FAILED` | Publish failed | Buffer and retry |
| `SUB_FAILED` | Subscribe failed | Retry subscription |
| `TIMEOUT` | Operation timeout | Retry or buffer |
| `INVALID_MSG` | Invalid message format | Log and discard |

### 8.2 Retry Strategy

```python
def exponential_backoff_retry(func, max_retries=5):
    for attempt in range(max_retries):
        try:
            return func()
        except Exception as e:
            if attempt == max_retries - 1:
                raise
            wait_time = min(300, (2 ** attempt) + random.uniform(0, 1))
            print(f"Retry {attempt + 1} after {wait_time}s: {e}")
            time.sleep(wait_time)
```

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial release |

---

<div align="center">

**WIA Ocean Plastic Track Standard v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA**

**MIT License**

</div>
