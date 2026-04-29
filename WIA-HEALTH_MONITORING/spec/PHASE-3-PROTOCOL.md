# WIA-HEALTH_MONITORING: Phase 3 - Protocol Specification

**Version**: 1.0
**Status**: Draft
**Last Updated**: 2026-01-12
**Authors**: WIA Technical Committee

---

## 1. Overview

This document defines the communication protocols, data synchronization strategies, and operational procedures for health monitoring systems.

### 1.1 Protocol Objectives

1. **Reliable Data Transmission**: Ensure no data loss during transmission
2. **Efficient Synchronization**: Minimize battery and bandwidth usage
3. **Real-Time Support**: Low-latency critical alert delivery
4. **Offline Resilience**: Function without continuous connectivity
5. **Interoperability**: Work across diverse device ecosystems

---

## 2. Communication Protocols

### 2.1 Protocol Stack

```
┌─────────────────────────────────────┐
│   Application Layer (WIA-HEALTH)    │
├─────────────────────────────────────┤
│   Transport Layer (HTTPS, WSS)      │
├─────────────────────────────────────┤
│   Network Layer (TCP/IP)            │
├─────────────────────────────────────┤
│   Link Layer (Wi-Fi, BLE, Cellular) │
└─────────────────────────────────────┘
```

### 2.2 Transport Protocols

#### 2.2.1 HTTPS (HTTP/2 or HTTP/3)

**Primary Use**: Bulk data upload, API requests

```
Protocol: HTTP/2 over TLS 1.3
Port: 443
Features:
  - Multiplexing
  - Server push
  - Header compression
  - Binary framing
```

**Connection Establishment:**
```http
GET /v1/health HTTP/2
Host: api.wia-health.org
Authorization: Bearer <token>
User-Agent: WIA-HealthMonitor/1.0
Accept: application/json
Accept-Encoding: gzip, br
```

#### 2.2.2 WebSocket Secure (WSS)

**Primary Use**: Real-time streaming, bidirectional communication

```
Protocol: WebSocket over TLS 1.3
Port: 443
Ping Interval: 30 seconds
Reconnect Strategy: Exponential backoff (1s, 2s, 4s, 8s, 16s, max 60s)
```

**Connection Flow:**
```javascript
// 1. Establish connection
ws = new WebSocket('wss://stream.wia-health.org/v1/stream');

// 2. Authenticate
ws.send({
  type: 'AUTH',
  token: 'bearer-token'
});

// 3. Subscribe
ws.send({
  type: 'SUBSCRIBE',
  channels: ['metrics', 'alerts'],
  metric_types: ['HEART_RATE', 'BLOOD_GLUCOSE']
});

// 4. Receive data
ws.onmessage = (event) => {
  const message = JSON.parse(event.data);
  handleMessage(message);
};

// 5. Keep-alive
setInterval(() => {
  ws.send({ type: 'PING' });
}, 30000);
```

#### 2.2.3 MQTT (Optional for IoT Devices)

**Primary Use**: Constrained devices, IoT sensors

```
Protocol: MQTT 5.0 over TLS
Port: 8883
QoS Levels:
  - QoS 0: At most once (non-critical metrics)
  - QoS 1: At least once (standard metrics)
  - QoS 2: Exactly once (critical alerts)
Keep Alive: 60 seconds
```

**Topic Structure:**
```
wia/health/{user_id}/metrics/{metric_type}
wia/health/{user_id}/alerts
wia/health/{user_id}/commands
```

**Example Publish:**
```python
import paho.mqtt.client as mqtt

client = mqtt.Client(protocol=mqtt.MQTTv5)
client.tls_set(ca_certs='ca.pem')
client.username_pw_set('device-id', 'api-key')
client.connect('mqtt.wia-health.org', 8883)

# Publish heart rate
client.publish(
    'wia/health/user-123/metrics/HEART_RATE',
    payload='{"bpm": 72, "timestamp": "2026-01-12T16:00:00Z"}',
    qos=1
)
```

#### 2.2.4 Bluetooth Low Energy (BLE)

**Primary Use**: Device-to-phone communication

```
Protocol: Bluetooth 5.0+
Profile: Health Device Profile (HDP) / Generic Attribute Profile (GATT)
Max Range: 100m (Bluetooth 5.0)
Power: Ultra-low energy
```

**GATT Service Structure:**
```
WIA Health Monitoring Service (UUID: 0000180D-0000-1000-8000-00805F9B34FB)
├─ Heart Rate Measurement (UUID: 00002A37-0000-1000-8000-00805F9B34FB)
├─ Blood Glucose Measurement (UUID: 00002A18-0000-1000-8000-00805F9B34FB)
├─ SpO2 Measurement (UUID: 00002A5F-0000-1000-8000-00805F9B34FB)
└─ Device Configuration (UUID: custom)
```

---

## 3. Data Synchronization

### 3.1 Synchronization Strategies

#### 3.1.1 Continuous Sync (Real-Time)

For critical metrics requiring immediate transmission:

```
Metrics: BLOOD_GLUCOSE (CGM), ECG, CRITICAL_ALERTS
Frequency: Immediate (0-5 second delay)
Transport: WebSocket or MQTT QoS 2
Battery Impact: High
Use Case: Critical monitoring, hospital settings
```

**Implementation:**
```javascript
class RealtimeSync {
  constructor(ws) {
    this.ws = ws;
    this.buffer = [];
  }

  async sendMetric(metric) {
    if (this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify({
        type: 'METRIC',
        data: metric
      }));
    } else {
      // Buffer for retry
      this.buffer.push(metric);
      await this.reconnect();
    }
  }
}
```

#### 3.1.2 Periodic Sync (Scheduled)

For non-critical metrics:

```
Metrics: HEART_RATE, STEPS, SLEEP
Frequency: Every 5-15 minutes
Transport: HTTPS batch upload
Battery Impact: Medium
Use Case: Fitness tracking, general wellness
```

**Implementation:**
```javascript
class PeriodicSync {
  constructor(interval = 300000) { // 5 minutes
    this.interval = interval;
    this.buffer = [];
    this.start();
  }

  start() {
    setInterval(async () => {
      if (this.buffer.length > 0) {
        await this.flush();
      }
    }, this.interval);
  }

  addMetric(metric) {
    this.buffer.push(metric);
  }

  async flush() {
    const batch = this.buffer.splice(0, this.buffer.length);
    await api.post('/metrics/batch', { metrics: batch });
  }
}
```

#### 3.1.3 Opportunistic Sync (Wi-Fi Only)

For bulk data:

```
Metrics: HIGH_RESOLUTION_ECG, DETAILED_SLEEP_ANALYSIS
Frequency: When Wi-Fi available
Transport: HTTPS with compression
Battery Impact: Low
Use Case: Detailed analysis, research data
```

**Implementation:**
```javascript
class OpportunisticSync {
  constructor() {
    this.pendingData = [];
    this.listenForWiFi();
  }

  listenForWiFi() {
    navigator.connection.addEventListener('change', async () => {
      if (navigator.connection.type === 'wifi') {
        await this.syncAll();
      }
    });
  }

  async syncAll() {
    for (const data of this.pendingData) {
      await api.post('/metrics', data);
    }
    this.pendingData = [];
  }
}
```

### 3.2 Conflict Resolution

When data conflicts occur (e.g., manual entry vs. device data):

```
Resolution Strategy:
1. Latest timestamp wins (Last-Write-Wins)
2. Device data preferred over manual entry
3. Higher quality score preferred
4. Manual override flag respected
```

**Example:**
```javascript
function resolveConflict(metric1, metric2) {
  // Rule 1: Manual override flag
  if (metric1.manual_override) return metric1;
  if (metric2.manual_override) return metric2;

  // Rule 2: Higher quality score
  if (metric1.quality_score !== metric2.quality_score) {
    return metric1.quality_score > metric2.quality_score ? metric1 : metric2;
  }

  // Rule 3: Device data preferred
  if (metric1.source === 'DEVICE' && metric2.source === 'MANUAL') {
    return metric1;
  }

  // Rule 4: Latest timestamp wins
  return new Date(metric1.timestamp) > new Date(metric2.timestamp)
    ? metric1 : metric2;
}
```

### 3.3 Delta Sync

Only sync changed data:

```json
{
  "sync_type": "DELTA",
  "last_sync_timestamp": "2026-01-12T15:00:00Z",
  "changes": [
    {
      "operation": "CREATE",
      "metric_id": "new-uuid-1",
      "data": { /* metric data */ }
    },
    {
      "operation": "UPDATE",
      "metric_id": "existing-uuid-2",
      "changes": {
        "quality_score": 0.95
      }
    },
    {
      "operation": "DELETE",
      "metric_id": "deleted-uuid-3"
    }
  ],
  "next_sync_token": "sync-token-abc123"
}
```

---

## 4. Error Handling and Retry

### 4.1 Retry Strategy

```
Algorithm: Exponential Backoff with Jitter

Base Delay: 1 second
Max Delay: 60 seconds
Max Attempts: 5
Jitter: ±25%

Delay Calculation:
delay = min(base_delay * 2^attempt + random_jitter, max_delay)
```

**Implementation:**
```javascript
async function retryWithBackoff(fn, maxAttempts = 5) {
  let attempt = 0;
  const baseDelay = 1000;
  const maxDelay = 60000;

  while (attempt < maxAttempts) {
    try {
      return await fn();
    } catch (error) {
      attempt++;
      if (attempt >= maxAttempts) throw error;

      const delay = Math.min(
        baseDelay * Math.pow(2, attempt) + (Math.random() - 0.5) * 500,
        maxDelay
      );
      await sleep(delay);
    }
  }
}
```

### 4.2 Circuit Breaker

Prevent cascading failures:

```javascript
class CircuitBreaker {
  constructor(threshold = 5, timeout = 60000) {
    this.failureCount = 0;
    this.threshold = threshold;
    this.timeout = timeout;
    this.state = 'CLOSED'; // CLOSED, OPEN, HALF_OPEN
    this.nextAttempt = null;
  }

  async call(fn) {
    if (this.state === 'OPEN') {
      if (Date.now() < this.nextAttempt) {
        throw new Error('Circuit breaker is OPEN');
      }
      this.state = 'HALF_OPEN';
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
      this.nextAttempt = Date.now() + this.timeout;
    }
  }
}
```

### 4.3 Offline Queue

Store data when network unavailable:

```javascript
class OfflineQueue {
  constructor() {
    this.queue = [];
    this.maxSize = 10000; // Max 10k metrics
    this.storage = localStorage; // or IndexedDB for larger capacity
  }

  enqueue(metric) {
    if (this.queue.length >= this.maxSize) {
      // Remove oldest if full
      this.queue.shift();
    }
    this.queue.push(metric);
    this.persist();
  }

  async flush() {
    while (this.queue.length > 0 && navigator.onLine) {
      const batch = this.queue.splice(0, 100); // Process 100 at a time
      try {
        await api.post('/metrics/batch', { metrics: batch });
        this.persist();
      } catch (error) {
        // Re-add to queue on failure
        this.queue.unshift(...batch);
        break;
      }
    }
  }

  persist() {
    this.storage.setItem('offline_queue', JSON.stringify(this.queue));
  }

  restore() {
    const stored = this.storage.getItem('offline_queue');
    if (stored) {
      this.queue = JSON.parse(stored);
    }
  }
}
```

---

## 5. Security Protocols

### 5.1 Authentication Flow

```
┌──────────┐                ┌────────────┐               ┌──────────┐
│  Device  │                │  Gateway   │               │   API    │
└────┬─────┘                └─────┬──────┘               └────┬─────┘
     │                            │                           │
     │ 1. Device Certificate      │                           │
     ├───────────────────────────>│                           │
     │                            │                           │
     │                            │ 2. Authenticate Device    │
     │                            ├──────────────────────────>│
     │                            │                           │
     │                            │ 3. Access Token           │
     │                            │<──────────────────────────┤
     │ 4. Token                   │                           │
     │<───────────────────────────┤                           │
     │                            │                           │
     │ 5. API Request + Token     │                           │
     ├───────────────────────────────────────────────────────>│
     │                            │                           │
     │ 6. Response                │                           │
     │<───────────────────────────────────────────────────────┤
```

### 5.2 End-to-End Encryption

For sensitive metrics (glucose, ECG):

```javascript
class E2EEncryption {
  constructor(publicKey, privateKey) {
    this.publicKey = publicKey;
    this.privateKey = privateKey;
  }

  async encrypt(data) {
    // Generate symmetric key
    const symmetricKey = await crypto.subtle.generateKey(
      { name: 'AES-GCM', length: 256 },
      true,
      ['encrypt', 'decrypt']
    );

    // Encrypt data with symmetric key
    const iv = crypto.getRandomValues(new Uint8Array(12));
    const encryptedData = await crypto.subtle.encrypt(
      { name: 'AES-GCM', iv },
      symmetricKey,
      new TextEncoder().encode(JSON.stringify(data))
    );

    // Encrypt symmetric key with public key
    const exportedKey = await crypto.subtle.exportKey('raw', symmetricKey);
    const encryptedKey = await crypto.subtle.encrypt(
      { name: 'RSA-OAEP' },
      this.publicKey,
      exportedKey
    );

    return {
      encryptedData: arrayBufferToBase64(encryptedData),
      encryptedKey: arrayBufferToBase64(encryptedKey),
      iv: arrayBufferToBase64(iv)
    };
  }

  async decrypt(encrypted) {
    // Decrypt symmetric key
    const symmetricKeyData = await crypto.subtle.decrypt(
      { name: 'RSA-OAEP' },
      this.privateKey,
      base64ToArrayBuffer(encrypted.encryptedKey)
    );

    // Import symmetric key
    const symmetricKey = await crypto.subtle.importKey(
      'raw',
      symmetricKeyData,
      { name: 'AES-GCM' },
      false,
      ['decrypt']
    );

    // Decrypt data
    const decryptedData = await crypto.subtle.decrypt(
      {
        name: 'AES-GCM',
        iv: base64ToArrayBuffer(encrypted.iv)
      },
      symmetricKey,
      base64ToArrayBuffer(encrypted.encryptedData)
    );

    return JSON.parse(new TextDecoder().decode(decryptedData));
  }
}
```

### 5.3 Data Integrity

Verify data hasn't been tampered with:

```javascript
async function signMetric(metric, privateKey) {
  const data = JSON.stringify(metric);
  const signature = await crypto.subtle.sign(
    { name: 'RSASSA-PKCS1-v1_5' },
    privateKey,
    new TextEncoder().encode(data)
  );

  return {
    ...metric,
    signature: arrayBufferToBase64(signature)
  };
}

async function verifyMetric(signedMetric, publicKey) {
  const { signature, ...metric } = signedMetric;
  const data = JSON.stringify(metric);

  const isValid = await crypto.subtle.verify(
    { name: 'RSASSA-PKCS1-v1_5' },
    publicKey,
    base64ToArrayBuffer(signature),
    new TextEncoder().encode(data)
  );

  return isValid;
}
```

---

## 6. Quality of Service (QoS)

### 6.1 Priority Levels

| Priority | Metric Types | Max Latency | Retry Policy |
|----------|--------------|-------------|--------------|
| CRITICAL | ECG anomaly, Low glucose alert | <5 seconds | Aggressive (1s backoff) |
| HIGH | CGM readings, Blood pressure | <30 seconds | Standard (exponential) |
| MEDIUM | Heart rate, SpO2 | <2 minutes | Standard |
| LOW | Steps, Sleep summary | <1 hour | Relaxed (Wi-Fi preferred) |

### 6.2 Bandwidth Optimization

```javascript
class BandwidthOptimizer {
  constructor() {
    this.compressionEnabled = true;
    this.samplingRate = 'AUTO'; // FULL, AUTO, REDUCED
  }

  optimize(metrics, connectionType) {
    if (connectionType === 'cellular-2g' || connectionType === 'cellular-3g') {
      // Reduce sampling rate
      metrics = this.downsample(metrics, 0.5);
    }

    if (this.compressionEnabled) {
      metrics = this.compress(metrics);
    }

    return metrics;
  }

  downsample(metrics, factor) {
    return metrics.filter((_, index) => index % Math.ceil(1/factor) === 0);
  }

  compress(data) {
    // Use gzip or brotli compression
    return pako.gzip(JSON.stringify(data));
  }
}
```

---

## 7. Time Synchronization

### 7.1 NTP Sync

Ensure accurate timestamps:

```javascript
class TimeSync {
  constructor() {
    this.offset = 0;
    this.syncInterval = 3600000; // Sync every hour
    this.sync();
  }

  async sync() {
    const t1 = Date.now();
    const serverTime = await api.get('/time');
    const t2 = Date.now();

    this.offset = serverTime.timestamp - (t1 + t2) / 2;

    setTimeout(() => this.sync(), this.syncInterval);
  }

  now() {
    return Date.now() + this.offset;
  }

  toISO() {
    return new Date(this.now()).toISOString();
  }
}
```

---

## 8. Protocol State Machine

```
┌─────────────┐
│   INITIAL   │
└──────┬──────┘
       │
       │ Connect
       ▼
┌─────────────┐        Disconnect
│  CONNECTING ├───────────────────┐
└──────┬──────┘                   │
       │                          │
       │ Connected                │
       ▼                          ▼
┌─────────────┐              ┌─────────────┐
│   CONNECTED ├─────────────>│ DISCONNECTED│
└──────┬──────┘  Timeout     └──────┬──────┘
       │         Error               │
       │                             │
       │ Authenticate                │ Retry
       ▼                             │
┌─────────────┐                     │
│AUTHENTICATED├<────────────────────┘
└──────┬──────┘
       │
       │ Subscribe
       ▼
┌─────────────┐
│   STREAMING │
└─────────────┘
```

---

## 9. Monitoring and Observability

### 9.1 Protocol Metrics

Track protocol health:

```javascript
class ProtocolMetrics {
  constructor() {
    this.metrics = {
      messagesReceived: 0,
      messagesSent: 0,
      bytesSent: 0,
      bytesReceived: 0,
      errors: 0,
      reconnects: 0,
      avgLatency: 0
    };
  }

  recordMessage(direction, size, latency) {
    if (direction === 'SENT') {
      this.metrics.messagesSent++;
      this.metrics.bytesSent += size;
    } else {
      this.metrics.messagesReceived++;
      this.metrics.bytesReceived += size;
    }

    if (latency) {
      this.updateAvgLatency(latency);
    }
  }

  recordError(error) {
    this.metrics.errors++;
    console.error('Protocol error:', error);
  }

  recordReconnect() {
    this.metrics.reconnects++;
  }

  getMetrics() {
    return { ...this.metrics };
  }
}
```

---

## 10. Compliance and Regulations

### 10.1 HIPAA Compliance

- All transmissions encrypted (TLS 1.3+)
- Audit logs for all data access
- Data retention policies enforced
- Secure device pairing

### 10.2 GDPR Compliance

- Right to deletion supported
- Data portability (export API)
- Consent management
- Data minimization

### 10.3 Medical Device Regulations

- FDA Class II compliance (if applicable)
- CE marking (EU)
- ISO 13485 quality management
- IEC 62304 software lifecycle

---

**弘益人間 (홍익인간)** - Benefit All Humanity

© 2026 WIA (World Certification Industry Association)
