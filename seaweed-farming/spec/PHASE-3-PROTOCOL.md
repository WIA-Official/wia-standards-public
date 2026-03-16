# WIA-AGRI-024: Seaweed Farming Standard
## Phase 3 - Communication Protocol Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-01-01

---

## 1. Overview

This specification defines communication protocols for seaweed farming systems, optimized for marine environments with intermittent connectivity, low bandwidth, and harsh conditions.

### 1.1 Design Principles

- **Marine-Resilient**: Tolerates network interruptions and high latency
- **Bandwidth-Efficient**: Optimized for satellite and cellular marine networks
- **Energy-Conscious**: Low power consumption for battery-operated sensors
- **Secure-by-Default**: End-to-end encryption and authentication
- **Offline-Capable**: Local buffering and synchronization

---

## 2. Protocol Stack

### 2.1 Layer Architecture

```
┌─────────────────────────────────────────────┐
│  Application Layer                          │
│  - Seaweed Farm Protocol (SFP)              │
│  - RESTful HTTP/2, WebSocket, MQTT         │
├─────────────────────────────────────────────┤
│  Presentation Layer                         │
│  - JSON, Protocol Buffers, MessagePack     │
│  - GZIP, Brotli compression                │
├─────────────────────────────────────────────┤
│  Session Layer                              │
│  - OAuth 2.0, JWT tokens                   │
│  - Session resumption                       │
├─────────────────────────────────────────────┤
│  Transport Layer                            │
│  - TLS 1.3, DTLS 1.3                       │
│  - TCP (reliable), UDP (real-time)         │
├─────────────────────────────────────────────┤
│  Network Layer                              │
│  - IPv4, IPv6                              │
│  - CoAP for constrained devices            │
├─────────────────────────────────────────────┤
│  Physical Layer                             │
│  - WiFi, 4G/5G, Satellite (Starlink, etc.) │
│  - LoRaWAN for long-range sensors          │
└─────────────────────────────────────────────┘
```

---

## 3. Message Types

### 3.1 Core Message Types

#### GROWTH_UPDATE

Real-time growth monitoring data from farm sensors.

**Message Structure:**

```json
{
  "messageType": "GROWTH_UPDATE",
  "version": "1.0",
  "messageId": "msg_67890xyz",
  "timestamp": "2025-01-15T14:30:00Z",
  "farmId": "550e8400-e29b-41d4-a716-446655440000",
  "payload": {
    "lineId": "LINE-025",
    "speciesId": "kelp-saccharina-latissima",
    "biomass": 12.5,
    "length": 230,
    "healthScore": 95
  },
  "qos": 1,
  "ttl": 3600
}
```

#### WATER_QUALITY

Water parameter measurements from environmental sensors.

**Message Structure:**

```json
{
  "messageType": "WATER_QUALITY",
  "version": "1.0",
  "messageId": "msg_12345abc",
  "timestamp": "2025-01-15T14:30:00Z",
  "farmId": "550e8400-e29b-41d4-a716-446655440000",
  "payload": {
    "sensorId": "SENSOR-WQ-001",
    "depth": 5,
    "temperature": 15.5,
    "salinity": 32.5,
    "pH": 8.1,
    "nitrate": 15.2,
    "phosphate": 1.8,
    "dissolvedOxygen": 8.5
  },
  "qos": 1,
  "ttl": 900
}
```

#### HARVEST_COMMAND

Command to trigger or schedule harvest operations.

**Message Structure:**

```json
{
  "messageType": "HARVEST_COMMAND",
  "version": "1.0",
  "messageId": "msg_cmd_001",
  "timestamp": "2025-01-15T14:30:00Z",
  "farmId": "550e8400-e29b-41d4-a716-446655440000",
  "payload": {
    "command": "schedule_harvest",
    "scheduledDate": "2025-03-15T07:00:00Z",
    "zoneIds": ["ZONE-A", "ZONE-B"],
    "method": "mechanical",
    "parameters": {
      "expectedYield": 5000,
      "cutHeight": 0.3
    }
  },
  "qos": 2,
  "requiresAck": true
}
```

#### CARBON_REPORT

Carbon sequestration data and credit generation.

**Message Structure:**

```json
{
  "messageType": "CARBON_REPORT",
  "version": "1.0",
  "messageId": "msg_carbon_001",
  "timestamp": "2025-01-15T14:30:00Z",
  "farmId": "550e8400-e29b-41d4-a716-446655440000",
  "payload": {
    "reportingPeriod": {
      "start": "2024-01-01T00:00:00Z",
      "end": "2024-12-31T23:59:59Z"
    },
    "totalCO2": 67850,
    "carbonCredits": 67.85,
    "verificationStatus": "verified"
  },
  "qos": 2,
  "ttl": 86400
}
```

#### ALERT

Critical system alerts and warnings.

**Message Structure:**

```json
{
  "messageType": "ALERT",
  "version": "1.0",
  "messageId": "msg_alert_001",
  "timestamp": "2025-01-15T14:30:00Z",
  "farmId": "550e8400-e29b-41d4-a716-446655440000",
  "payload": {
    "severity": "warning",
    "category": "environmental",
    "alertType": "temperature_high",
    "value": 18.5,
    "threshold": 18.0,
    "zoneId": "ZONE-C",
    "message": "Water temperature exceeds optimal range",
    "actionRequired": true,
    "recommendations": [
      "Increase water circulation",
      "Monitor for heat stress"
    ]
  },
  "qos": 2,
  "requiresAck": true,
  "priority": "high"
}
```

---

## 4. Transport Protocols

### 4.1 MQTT for Real-time Messaging

**Connection Parameters:**

```
Protocol: MQTT v5.0
Port: 8883 (TLS), 1883 (plain - dev only)
Keep-alive: 60 seconds
Clean session: false (persistent sessions)
```

**Topic Structure:**

```
wia/seaweed/{farmId}/growth/+
wia/seaweed/{farmId}/water-quality/+
wia/seaweed/{farmId}/harvest/commands
wia/seaweed/{farmId}/carbon/reports
wia/seaweed/{farmId}/alerts/+
wia/seaweed/{farmId}/status
```

**QoS Levels:**

- **QoS 0 (At most once)**: Non-critical telemetry
- **QoS 1 (At least once)**: Standard sensor data
- **QoS 2 (Exactly once)**: Commands, harvest records, carbon reports

**Example MQTT Publish:**

```python
import paho.mqtt.client as mqtt

client = mqtt.Client(client_id="farm_sensor_001", protocol=mqtt.MQTTv5)
client.tls_set(ca_certs="/path/to/ca.crt")
client.username_pw_set("farm_001", "api_key_here")
client.connect("mqtt.wia-seaweed.org", 8883, 60)

message = {
    "messageType": "WATER_QUALITY",
    "timestamp": "2025-01-15T14:30:00Z",
    "farmId": "550e8400-e29b-41d4-a716-446655440000",
    "payload": {
        "temperature": 15.5,
        "salinity": 32.5,
        "pH": 8.1
    }
}

client.publish(
    "wia/seaweed/550e8400-e29b-41d4-a716-446655440000/water-quality/SENSOR-WQ-001",
    json.dumps(message),
    qos=1
)
```

### 4.2 HTTP/2 for Request-Response

**Base URL:**

```
https://api.wia-seaweed.org/v1
```

**Headers:**

```http
POST /farms/{farmId}/growth HTTP/2
Host: api.wia-seaweed.org
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
Content-Type: application/json
Accept: application/json
X-Request-ID: req_abc123xyz
User-Agent: WIA-Seaweed-Sensor/1.0
```

**Request:**

```json
{
  "lineId": "LINE-025",
  "timestamp": "2025-01-15T14:30:00Z",
  "biomass": 12.5,
  "length": 230,
  "healthScore": 95
}
```

**Response:**

```http
HTTP/2 201 Created
Content-Type: application/json
X-Request-ID: req_abc123xyz
X-RateLimit-Remaining: 998

{
  "measurementId": "meas_xyz789",
  "status": "accepted",
  "processedAt": "2025-01-15T14:30:01Z"
}
```

### 4.3 WebSocket for Bidirectional Streaming

**Connection:**

```javascript
const ws = new WebSocket('wss://stream.wia-seaweed.org/v1/farms/550e8400-e29b-41d4-a716-446655440000');

ws.onopen = () => {
  ws.send(JSON.stringify({
    action: 'subscribe',
    channels: ['water-quality', 'growth', 'alerts']
  }));
};

ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log('Received:', data);
};
```

**Message Format:**

```json
{
  "event": "water-quality",
  "farmId": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": "2025-01-15T14:30:00Z",
  "data": {
    "temperature": 15.5,
    "salinity": 32.5,
    "pH": 8.1
  }
}
```

### 4.4 CoAP for Constrained Devices

**Endpoint:**

```
coaps://coap.wia-seaweed.org:5684/farms/{farmId}/water-quality
```

**Request:**

```
POST coaps://coap.wia-seaweed.org:5684/farms/550e8400/water-quality
Content-Format: application/cbor
Payload: CBOR-encoded sensor data
```

**Response:**

```
2.01 Created
Content-Format: application/cbor
Payload: {messageId: "msg_12345"}
```

---

## 5. Security Protocols

### 5.1 Authentication

**OAuth 2.0 Flow:**

```http
POST /oauth/token HTTP/1.1
Host: auth.wia-seaweed.org
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id=farm_550e8400
&client_secret=secret_abc123xyz
&scope=farm:read farm:write sensor:write
```

**Response:**

```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "farm:read farm:write sensor:write"
}
```

### 5.2 Message Encryption

**End-to-End Encryption:**

- Algorithm: AES-256-GCM
- Key Exchange: ECDH (Curve25519)
- Message Authentication: HMAC-SHA256

**Encrypted Message Format:**

```json
{
  "messageType": "ENCRYPTED",
  "version": "1.0",
  "messageId": "msg_enc_001",
  "encryptionAlgorithm": "AES-256-GCM",
  "keyId": "key_farm_001",
  "iv": "base64-encoded-iv",
  "ciphertext": "base64-encoded-encrypted-payload",
  "tag": "base64-encoded-auth-tag"
}
```

### 5.3 Message Signing

**Digital Signatures:**

```json
{
  "messageType": "SIGNED_MESSAGE",
  "version": "1.0",
  "messageId": "msg_sign_001",
  "payload": { /* original message */ },
  "signature": {
    "algorithm": "RS256",
    "keyId": "did:wia:farm_550e8400#key-1",
    "value": "base64-encoded-signature"
  }
}
```

---

## 6. Offline Operation

### 6.1 Local Buffering

**Buffer Management:**

```typescript
interface MessageBuffer {
  farmId: string;
  messages: Message[];
  maxSize: number;
  compression: boolean;
  retentionPolicy: {
    maxAge: number; // seconds
    maxMessages: number;
  };
}

class OfflineBuffer {
  async addMessage(message: Message): Promise<void> {
    // Add to local storage
    await localDB.messages.add(message);
  }

  async sync(): Promise<void> {
    // Upload buffered messages when connection restored
    const messages = await localDB.messages.getAll();
    await api.batch.upload(messages);
    await localDB.messages.clear();
  }
}
```

### 6.2 Synchronization Protocol

**Sync Request:**

```json
{
  "action": "sync",
  "farmId": "550e8400-e29b-41d4-a716-446655440000",
  "lastSync": "2025-01-15T12:00:00Z",
  "bufferedMessages": 127,
  "priority": "high"
}
```

**Sync Response:**

```json
{
  "status": "accepted",
  "messagesReceived": 127,
  "messagesProcessed": 125,
  "messagesFailed": 2,
  "nextSync": "2025-01-15T16:00:00Z",
  "updates": [
    {
      "type": "config_update",
      "data": { /* new configuration */ }
    }
  ]
}
```

---

## 7. Network Optimization

### 7.1 Compression

**Payload Compression:**

- GZIP: General-purpose compression
- Brotli: Web content (better compression)
- LZ4: Real-time data (faster)

**Compressed Message:**

```http
POST /farms/550e8400/growth HTTP/2
Content-Encoding: gzip
Content-Type: application/json

[gzip-compressed-payload]
```

### 7.2 Binary Protocols

**Protocol Buffers:**

```protobuf
syntax = "proto3";

message WaterQuality {
  string sensor_id = 1;
  int64 timestamp = 2;
  float temperature = 3;
  float salinity = 4;
  float ph = 5;
  float nitrate = 6;
  float phosphate = 7;
}
```

**MessagePack:**

```python
import msgpack

data = {
    "temperature": 15.5,
    "salinity": 32.5,
    "pH": 8.1
}

packed = msgpack.packb(data)
# Smaller size than JSON
```

### 7.3 Delta Encoding

**Send only changes:**

```json
{
  "messageType": "DELTA_UPDATE",
  "baseMessageId": "msg_12345",
  "timestamp": "2025-01-15T14:30:00Z",
  "changes": {
    "temperature": 15.5,  // Changed
    "salinity": 32.5      // Changed
    // pH, nitrate unchanged - not sent
  }
}
```

---

## 8. Quality of Service

### 8.1 Message Priority

| Priority | QoS | Use Case | Latency |
|----------|-----|----------|---------|
| Critical | 2 | Alerts, Commands | < 1s |
| High | 2 | Harvest data, Carbon reports | < 5s |
| Normal | 1 | Growth data, Water quality | < 30s |
| Low | 0 | Historical data, Analytics | < 5m |

### 8.2 Retry Policy

```typescript
interface RetryPolicy {
  maxAttempts: 5;
  backoffStrategy: 'exponential';
  initialDelay: 1000;  // ms
  maxDelay: 60000;     // ms
  jitter: true;
}

async function sendWithRetry(message: Message, policy: RetryPolicy) {
  for (let attempt = 1; attempt <= policy.maxAttempts; attempt++) {
    try {
      await send(message);
      return;
    } catch (error) {
      if (attempt === policy.maxAttempts) throw error;

      const delay = Math.min(
        policy.initialDelay * Math.pow(2, attempt - 1),
        policy.maxDelay
      );

      await sleep(delay + (policy.jitter ? Math.random() * 1000 : 0));
    }
  }
}
```

---

## 9. Protocol Extensions

### 9.1 Custom Headers

```json
{
  "messageType": "GROWTH_UPDATE",
  "headers": {
    "X-Farm-Region": "Pacific-Northwest",
    "X-Species-Type": "kelp",
    "X-Device-Type": "underwater-camera",
    "X-Batch-Size": "10"
  },
  "payload": { /* ... */ }
}
```

### 9.2 Vendor Extensions

```json
{
  "messageType": "GROWTH_UPDATE",
  "vendorExtensions": {
    "acme-farm-systems": {
      "cameraModel": "UW-4K-PRO",
      "aiModelVersion": "v2.5.1",
      "confidence": 0.95
    }
  },
  "payload": { /* ... */ }
}
```

---

## 10. Monitoring and Diagnostics

### 10.1 Heartbeat Protocol

```json
{
  "messageType": "HEARTBEAT",
  "version": "1.0",
  "timestamp": "2025-01-15T14:30:00Z",
  "farmId": "550e8400-e29b-41d4-a716-446655440000",
  "deviceId": "SENSOR-WQ-001",
  "status": {
    "online": true,
    "batteryLevel": 85,
    "signalStrength": -72,
    "uptime": 86400,
    "lastDataSent": "2025-01-15T14:29:00Z"
  }
}
```

### 10.2 Network Diagnostics

```json
{
  "messageType": "DIAGNOSTICS",
  "version": "1.0",
  "timestamp": "2025-01-15T14:30:00Z",
  "farmId": "550e8400-e29b-41d4-a716-446655440000",
  "networkMetrics": {
    "latency": 120,           // ms
    "packetLoss": 0.02,       // 2%
    "bandwidth": 1500000,     // bps
    "connectionType": "4G",
    "provider": "Marine Networks Inc"
  }
}
```

---

## 11. Implementation Guidelines

### 11.1 Client Implementation Checklist

- ✅ Support MQTT v5.0 with QoS 1 minimum
- ✅ Implement exponential backoff retry
- ✅ Local message buffering (min 1000 messages)
- ✅ TLS 1.3 for all connections
- ✅ OAuth 2.0 token refresh
- ✅ Compression support (GZIP minimum)
- ✅ Heartbeat every 60 seconds
- ✅ Graceful disconnection handling

### 11.2 Server Implementation Checklist

- ✅ MQTT broker with clustering support
- ✅ Message persistence (7 days minimum)
- ✅ Rate limiting per client
- ✅ Message deduplication
- ✅ Dead letter queue for failed messages
- ✅ Metrics and monitoring
- ✅ Multi-region deployment
- ✅ Load balancing

---

**© 2025 WIA Standards | 弘益人間 · Benefit All Humanity**
