# WIA-AGRI-014: Agricultural Supply Chain Standard
## Phase 3: Protocol Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-01-01

---

## 1. Overview

This specification defines communication protocols for agricultural supply chain systems, including IoT sensor protocols, blockchain integration, and real-time data streaming.

### 1.1 Protocol Stack

```
┌─────────────────────────────────┐
│   Application Layer (REST API)  │
├─────────────────────────────────┤
│   Message Protocol (MQTT/CoAP)  │
├─────────────────────────────────┤
│   Security Layer (TLS 1.3)      │
├─────────────────────────────────┤
│   Transport Layer (TCP/UDP)     │
├─────────────────────────────────┤
│   Network Layer (IPv4/IPv6)     │
└─────────────────────────────────┘
```

---

## 2. IoT Sensor Communication

### 2.1 MQTT Protocol

**Broker Configuration:**
```
mqtt://iot.wia-supply-chain.org:1883 (Standard)
mqtts://iot.wia-supply-chain.org:8883 (TLS)
```

**Topic Structure:**
```
wia/supply-chain/{shipmentId}/{sensorType}
wia/supply-chain/{shipmentId}/temperature
wia/supply-chain/{shipmentId}/humidity
wia/supply-chain/{shipmentId}/gps
wia/supply-chain/{shipmentId}/alerts
```

**Message Format:**
```json
{
  "deviceId": "SENSOR-TEMP-001",
  "timestamp": "2025-01-01T12:30:45.123Z",
  "data": {
    "temperature": 4.2,
    "unit": "CELSIUS"
  },
  "metadata": {
    "batteryLevel": 85,
    "signalStrength": 75
  }
}
```

**QoS Levels:**
- QoS 0: Sensor heartbeat messages
- QoS 1: Temperature/humidity readings (at least once delivery)
- QoS 2: Alert messages (exactly once delivery)

### 2.2 CoAP Protocol

For constrained devices with limited bandwidth:

```
coap://iot.wia-supply-chain.org:5683
```

**Request:**
```
CON POST /shipment/SHIP-2025-001/temperature
Content-Format: application/json
Payload: {"temperature": 4.2}
```

**Response:**
```
ACK 2.04 Changed
```

### 2.3 LoRaWAN Integration

For long-range, low-power sensors:

**Device Configuration:**
```json
{
  "devEUI": "0018B20000000001",
  "appEUI": "70B3D57ED0000000",
  "appKey": "00112233445566778899AABBCCDDEEFF",
  "dataRate": "SF7BW125",
  "frequency": 868100000
}
```

**Uplink Payload (Temperature Sensor):**
```
Port: 1
Payload (hex): 01 A6 04 B2
Decoded: {temperature: 4.2, humidity: 75}
```

---

## 3. Blockchain Protocol

### 3.1 Hyperledger Fabric Integration

**Network Configuration:**
```yaml
name: wia-supply-chain
version: 1.0.0
channels:
  - agricultural-supply:
      orderers:
        - orderer.wia.org
      peers:
        - peer0.farm.wia.org
        - peer0.distributor.wia.org
        - peer0.retailer.wia.org
```

**Chaincode Functions:**

**Record Shipment:**
```javascript
async function recordShipment(ctx, shipmentData) {
  const shipmentId = shipmentData.shipmentId;
  const key = ctx.stub.createCompositeKey('shipment', [shipmentId]);

  const shipment = {
    shipmentId: shipmentData.shipmentId,
    productId: shipmentData.productInfo.productId,
    timestamp: new Date().toISOString(),
    hash: sha256(JSON.stringify(shipmentData))
  };

  await ctx.stub.putState(key, Buffer.from(JSON.stringify(shipment)));
  return shipment;
}
```

**Verify Provenance:**
```javascript
async function verifyProvenance(ctx, productId) {
  const iterator = await ctx.stub.getStateByPartialCompositeKey('provenance', [productId]);
  const history = [];

  while (true) {
    const result = await iterator.next();
    if (result.done) break;

    const record = JSON.parse(result.value.value.toString());
    history.push(record);
  }

  return {
    productId: productId,
    verified: true,
    history: history
  };
}
```

### 3.2 Smart Contract Events

**Shipment Created Event:**
```javascript
ctx.stub.setEvent('ShipmentCreated', Buffer.from(JSON.stringify({
  shipmentId: 'SHIP-2025-001',
  timestamp: '2025-01-01T10:00:00Z'
})));
```

**Cold Chain Violation Event:**
```javascript
ctx.stub.setEvent('ColdChainViolation', Buffer.from(JSON.stringify({
  shipmentId: 'SHIP-2025-001',
  severity: 'HIGH',
  temperature: 12.5,
  threshold: 8.0
})));
```

---

## 4. Real-Time Data Streaming

### 4.1 WebSocket Protocol

**Connection:**
```javascript
const ws = new WebSocket('wss://stream.wia-supply-chain.org/shipments/SHIP-2025-001');

ws.onopen = () => {
  console.log('Connected to shipment stream');
};

ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log('Shipment update:', data);
};
```

**Message Types:**

**Location Update:**
```json
{
  "type": "location_update",
  "shipmentId": "SHIP-2025-001",
  "timestamp": "2025-01-01T12:30:45Z",
  "location": {
    "latitude": 37.1234,
    "longitude": 127.5678
  }
}
```

**Cold Chain Update:**
```json
{
  "type": "cold_chain_update",
  "shipmentId": "SHIP-2025-001",
  "temperature": 4.2,
  "humidity": 75,
  "status": "NORMAL"
}
```

### 4.2 Server-Sent Events (SSE)

For one-way streaming from server:

```javascript
const eventSource = new EventSource('https://api.wia-supply-chain.org/v1/stream/SHIP-2025-001');

eventSource.addEventListener('temperature', (event) => {
  const data = JSON.parse(event.data);
  console.log('Temperature:', data.value);
});

eventSource.addEventListener('alert', (event) => {
  const data = JSON.parse(event.data);
  console.log('Alert:', data.message);
});
```

---

## 5. Data Synchronization Protocol

### 5.1 Offline-First Sync

For mobile apps with intermittent connectivity:

**Sync Request:**
```json
{
  "deviceId": "MOBILE-001",
  "lastSyncTimestamp": "2025-01-01T10:00:00Z",
  "changes": [
    {
      "type": "UPDATE",
      "entity": "shipment",
      "id": "SHIP-2025-001",
      "field": "status",
      "value": "DELIVERED",
      "timestamp": "2025-01-01T18:00:00Z"
    }
  ]
}
```

**Sync Response:**
```json
{
  "syncTimestamp": "2025-01-01T18:05:00Z",
  "serverChanges": [
    {
      "entity": "cold_chain",
      "id": "SHIP-2025-002",
      "data": {...},
      "timestamp": "2025-01-01T17:00:00Z"
    }
  ],
  "conflicts": []
}
```

### 5.2 Conflict Resolution

**Conflict Detection:**
```json
{
  "conflictId": "CONFLICT-001",
  "entity": "shipment",
  "id": "SHIP-2025-001",
  "field": "status",
  "clientValue": "DELIVERED",
  "clientTimestamp": "2025-01-01T18:00:00Z",
  "serverValue": "IN_TRANSIT",
  "serverTimestamp": "2025-01-01T18:01:00Z"
}
```

**Resolution Strategy:**
- Last-Write-Wins (default)
- Server-Wins (for critical fields)
- Manual Resolution (for conflicts)

---

## 6. Security Protocols

### 6.1 TLS/SSL Configuration

**Minimum TLS Version:** TLS 1.3

**Cipher Suites:**
```
TLS_AES_256_GCM_SHA384
TLS_CHACHA20_POLY1305_SHA256
TLS_AES_128_GCM_SHA256
```

**Certificate Pinning:**
```javascript
const options = {
  ca: fs.readFileSync('ca-cert.pem'),
  checkServerIdentity: (host, cert) => {
    const expectedFingerprint = 'AA:BB:CC:DD:EE:FF...';
    const actualFingerprint = cert.fingerprint256;
    if (expectedFingerprint !== actualFingerprint) {
      throw new Error('Certificate fingerprint mismatch');
    }
  }
};
```

### 6.2 Message Signing

**HMAC-SHA256 Signature:**
```javascript
function signMessage(message, secret) {
  const hmac = crypto.createHmac('sha256', secret);
  hmac.update(JSON.stringify(message));
  return hmac.digest('hex');
}

const message = { shipmentId: 'SHIP-2025-001', status: 'DELIVERED' };
const signature = signMessage(message, 'your_secret_key');

// Send with signature
{
  "data": message,
  "signature": signature
}
```

**Verification:**
```javascript
function verifyMessage(receivedMessage, receivedSignature, secret) {
  const expectedSignature = signMessage(receivedMessage, secret);
  return expectedSignature === receivedSignature;
}
```

---

## 7. Error Handling & Retry Protocol

### 7.1 Exponential Backoff

```javascript
async function sendWithRetry(data, maxRetries = 3) {
  for (let i = 0; i < maxRetries; i++) {
    try {
      return await sendData(data);
    } catch (error) {
      if (i === maxRetries - 1) throw error;
      const delay = Math.pow(2, i) * 1000; // 1s, 2s, 4s
      await sleep(delay);
    }
  }
}
```

### 7.2 Circuit Breaker Pattern

```javascript
class CircuitBreaker {
  constructor(threshold = 5, timeout = 60000) {
    this.failureCount = 0;
    this.threshold = threshold;
    this.timeout = timeout;
    this.state = 'CLOSED';
  }

  async execute(fn) {
    if (this.state === 'OPEN') {
      throw new Error('Circuit breaker is OPEN');
    }

    try {
      const result = await fn();
      this.onSuccess();
      return result;
    } catch (error) {
      this.onFailure();
      throw error;
    }
  }

  onSuccess() {
    this.failureCount = 0;
    this.state = 'CLOSED';
  }

  onFailure() {
    this.failureCount++;
    if (this.failureCount >= this.threshold) {
      this.state = 'OPEN';
      setTimeout(() => {
        this.state = 'HALF_OPEN';
        this.failureCount = 0;
      }, this.timeout);
    }
  }
}
```

---

## 8. Performance Optimization

### 8.1 Message Compression

**Gzip Compression:**
```http
Content-Encoding: gzip
Accept-Encoding: gzip, deflate
```

### 8.2 Batch Processing

**Batch Sensor Data:**
```json
{
  "batchId": "BATCH-001",
  "shipmentId": "SHIP-2025-001",
  "readings": [
    { "timestamp": "2025-01-01T12:00:00Z", "temperature": 4.1 },
    { "timestamp": "2025-01-01T12:01:00Z", "temperature": 4.2 },
    { "timestamp": "2025-01-01T12:02:00Z", "temperature": 4.1 }
  ]
}
```

---

**Next Phase:** [Phase 4: Integration](PHASE-4-INTEGRATION.md)

---

**弘益人間 (Benefit All Humanity)**
*WIA - World Certification Industry Association*
*© 2025 MIT License*
