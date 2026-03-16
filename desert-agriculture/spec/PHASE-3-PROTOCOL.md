# WIA-AGRI-021: Desert Agriculture
## Phase 3 - Protocol Specification

**Version:** 1.0
**Status:** Official Standard
**Last Updated:** 2025-12-26
**Standard ID:** WIA-AGRI-021

---

## 1. Overview

This specification defines the communication protocols, security measures, and operational procedures for the WIA-AGRI-021 Desert Agriculture standard. It ensures reliable, secure, and efficient data exchange between sensors, control systems, and management platforms in harsh desert environments.

### 1.1 Protocol Stack

```
┌─────────────────────────────────────┐
│     Application Layer (HTTP/WSS)   │
├─────────────────────────────────────┤
│     Security Layer (TLS 1.3)       │
├─────────────────────────────────────┤
│     Transport Layer (TCP/UDP)      │
├─────────────────────────────────────┤
│     Network Layer (IPv4/IPv6)      │
├─────────────────────────────────────┤
│     Data Link Layer (Ethernet/WiFi)│
└─────────────────────────────────────┘
```

### 1.2 Supported Protocols

| Protocol | Purpose | Port | Encryption |
|----------|---------|------|------------|
| HTTPS | REST API | 443 | TLS 1.3 |
| WSS | Real-time data | 443 | TLS 1.3 |
| MQTT | IoT messaging | 8883 | TLS 1.3 |
| CoAP | Lightweight IoT | 5684 | DTLS 1.3 |
| NTP | Time sync | 123 | - |

---

## 2. Communication Protocols

### 2.1 HTTP/HTTPS Protocol

#### 2.1.1 Request Format

All HTTP requests must follow standard HTTP/1.1 or HTTP/2 specifications:

```http
POST /api/v1/sensors/data HTTP/1.1
Host: api.wia-agri-021.org
X-API-Key: your_api_key
Content-Type: application/json
Content-Length: 256
User-Agent: WIA-AGRI-Client/1.0

{
  "farmId": "farm_sahara_001",
  "data": { ... }
}
```

#### 2.1.2 Response Headers

Mandatory response headers:

```http
HTTP/1.1 200 OK
Content-Type: application/json
X-Request-ID: req_abc123def456
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 847
X-RateLimit-Reset: 1735214400
Cache-Control: no-cache
Date: Thu, 26 Dec 2025 10:30:00 GMT
```

#### 2.1.3 Compression

Support for gzip and deflate compression:

```http
Accept-Encoding: gzip, deflate
Content-Encoding: gzip
```

### 2.2 WebSocket Protocol

#### 2.2.1 Connection Establishment

```javascript
// Client connects to WebSocket endpoint
const ws = new WebSocket('wss://ws.wia-agri-021.org/v1/stream');

// Authentication message (must be first)
ws.onopen = () => {
  ws.send(JSON.stringify({
    type: 'auth',
    apiKey: 'your_api_key',
    version: '1.0'
  }));
};
```

#### 2.2.2 Message Format

All WebSocket messages use JSON:

```json
{
  "type": "message_type",
  "id": "unique_message_id",
  "timestamp": "2025-12-26T10:30:00Z",
  "data": {
    /* Message payload */
  }
}
```

#### 2.2.3 Message Types

| Type | Direction | Purpose |
|------|-----------|---------|
| auth | Client → Server | Authentication |
| auth_success | Server → Client | Auth confirmation |
| subscribe | Client → Server | Subscribe to data |
| unsubscribe | Client → Server | Unsubscribe from data |
| data | Server → Client | Sensor data update |
| command | Client → Server | Control command |
| ping | Bidirectional | Keep-alive |
| pong | Bidirectional | Keep-alive response |
| error | Server → Client | Error notification |

#### 2.2.4 Heartbeat Protocol

```json
// Client sends ping every 30 seconds
{
  "type": "ping",
  "timestamp": "2025-12-26T10:30:00Z"
}

// Server responds with pong
{
  "type": "pong",
  "timestamp": "2025-12-26T10:30:01Z"
}
```

#### 2.2.5 Reconnection Strategy

```javascript
class WIAWebSocket {
  constructor(url, apiKey) {
    this.url = url;
    this.apiKey = apiKey;
    this.reconnectDelay = 1000; // Start with 1 second
    this.maxReconnectDelay = 60000; // Max 1 minute
    this.connect();
  }

  connect() {
    this.ws = new WebSocket(this.url);

    this.ws.onopen = () => {
      this.reconnectDelay = 1000; // Reset delay on successful connect
      this.authenticate();
    };

    this.ws.onclose = () => {
      // Exponential backoff
      setTimeout(() => this.connect(), this.reconnectDelay);
      this.reconnectDelay = Math.min(
        this.reconnectDelay * 2,
        this.maxReconnectDelay
      );
    };
  }
}
```

### 2.3 MQTT Protocol

#### 2.3.1 Connection Parameters

```javascript
const mqtt = require('mqtt');

const client = mqtt.connect('mqtts://mqtt.wia-agri-021.org:8883', {
  clientId: 'farm_sahara_001_client',
  username: 'farm_sahara_001',
  password: 'api_key_here',
  protocol: 'mqtts',
  keepalive: 60,
  clean: true,
  reconnectPeriod: 5000,
  connectTimeout: 30000
});
```

#### 2.3.2 Topic Structure

```
wia-agri-021/{farmId}/{deviceType}/{deviceId}/{dataType}

Examples:
wia-agri-021/farm_sahara_001/sensor/temp_001/temperature
wia-agri-021/farm_sahara_001/controller/irr_001/command
wia-agri-021/farm_sahara_001/alert/system/critical
```

#### 2.3.3 QoS Levels

| QoS | Description | Use Case |
|-----|-------------|----------|
| 0 | At most once | Non-critical sensor data |
| 1 | At least once | Standard sensor readings |
| 2 | Exactly once | Control commands, alerts |

#### 2.3.4 Message Retention

```javascript
// Publish with retention
client.publish(
  'wia-agri-021/farm_sahara_001/sensor/temp_001/temperature',
  JSON.stringify(data),
  { qos: 1, retain: true }
);
```

### 2.4 CoAP Protocol

For resource-constrained devices:

```
coaps://coap.wia-agri-021.org:5684/sensors/data

Method: POST
Content-Format: application/json
Payload: {"sensorId":"temp_001","value":38.5}
```

---

## 3. Security Protocols

### 3.1 TLS/SSL Configuration

#### 3.1.1 Minimum Requirements

- Protocol: TLS 1.3 (TLS 1.2 minimum)
- Certificate: Valid X.509 certificate
- Key Exchange: ECDHE
- Cipher Suites: AES-256-GCM, ChaCha20-Poly1305
- Hash: SHA-384 or SHA-512

#### 3.1.2 Certificate Pinning

```javascript
const https = require('https');
const fs = require('fs');

const options = {
  hostname: 'api.wia-agri-021.org',
  port: 443,
  path: '/v1/sensors/data',
  method: 'GET',
  ca: fs.readFileSync('wia-agri-021-ca.pem'),
  checkServerIdentity: (host, cert) => {
    // Verify certificate fingerprint
    const fingerprint = cert.fingerprint256;
    const expectedFingerprint = 'AA:BB:CC:...';

    if (fingerprint !== expectedFingerprint) {
      throw new Error('Certificate pinning failed');
    }
  }
};
```

### 3.2 API Authentication

#### 3.2.1 API Key Format

```
wia_agri_021_{environment}_{random_32_chars}

Examples:
wia_agri_021_prod_a1b2c3d4e5f6g7h8i9j0k1l2m3n4o5p6
wia_agri_021_test_x9y8z7w6v5u4t3s2r1q0p9o8n7m6l5k4
```

#### 3.2.2 Key Rotation

```javascript
// Automatic key rotation every 90 days
const rotateApiKey = async (farmId) => {
  // Generate new key
  const newKey = await generateApiKey(farmId);

  // Update in database with overlap period
  await updateApiKey(farmId, newKey, {
    activateAt: Date.now() + 86400000, // 24 hours
    deprecateOldAt: Date.now() + 604800000 // 7 days
  });

  // Notify administrators
  await sendKeyRotationNotification(farmId, newKey);
};
```

### 3.3 Data Encryption

#### 3.3.1 Encryption at Rest

```
Algorithm: AES-256-GCM
Key Management: AWS KMS / Azure Key Vault
Key Rotation: Every 90 days
Backup Encryption: AES-256-CBC
```

#### 3.3.2 Encryption in Transit

All data transmission must use:
- TLS 1.3 for HTTPS/WSS
- DTLS 1.3 for CoAP
- TLS 1.3 for MQTT

#### 3.3.3 Field-Level Encryption

For sensitive data fields:

```json
{
  "farmId": "farm_sahara_001",
  "location": {
    "encrypted": true,
    "algorithm": "AES-256-GCM",
    "data": "encrypted_base64_string",
    "iv": "initialization_vector",
    "tag": "authentication_tag"
  }
}
```

### 3.4 Access Control

#### 3.4.1 Role-Based Access Control (RBAC)

| Role | Permissions |
|------|-------------|
| Owner | Full access to all resources |
| Admin | Manage farm, zones, schedules |
| Operator | Start/stop irrigation, view data |
| Viewer | Read-only access to data |
| API | Programmatic access only |

#### 3.4.2 Permission Matrix

| Resource | Owner | Admin | Operator | Viewer | API |
|----------|-------|-------|----------|--------|-----|
| Read Sensors | ✓ | ✓ | ✓ | ✓ | ✓ |
| Control Irrigation | ✓ | ✓ | ✓ | ✗ | ✓ |
| Manage Schedules | ✓ | ✓ | ✗ | ✗ | ✓ |
| Configure Farm | ✓ | ✓ | ✗ | ✗ | ✗ |
| Manage Users | ✓ | ✗ | ✗ | ✗ | ✗ |

---

## 4. Data Synchronization Protocol

### 4.1 Clock Synchronization

All devices must synchronize with NTP servers:

```
Primary: time.wia-agri-021.org
Secondary: pool.ntp.org
Sync Interval: 3600 seconds (1 hour)
Max Clock Drift: ±100ms
```

### 4.2 Data Consistency

#### 4.2.1 Eventually Consistent Model

```javascript
// Client-side buffering for offline scenarios
class DataBuffer {
  constructor() {
    this.buffer = [];
    this.maxSize = 10000;
  }

  async add(data) {
    data.bufferedAt = Date.now();
    this.buffer.push(data);

    if (this.isOnline()) {
      await this.flush();
    }
  }

  async flush() {
    while (this.buffer.length > 0) {
      const batch = this.buffer.splice(0, 100);
      await this.sendBatch(batch);
    }
  }
}
```

#### 4.2.2 Conflict Resolution

```javascript
// Last-Write-Wins (LWW) strategy
const resolveConflict = (local, remote) => {
  if (remote.timestamp > local.timestamp) {
    return remote;
  }
  return local;
};
```

### 4.3 Data Aggregation

#### 4.3.1 Edge Computing

```javascript
// Aggregate sensor data at edge before transmission
class EdgeAggregator {
  constructor(interval = 300000) { // 5 minutes
    this.interval = interval;
    this.measurements = [];
  }

  add(measurement) {
    this.measurements.push(measurement);
  }

  aggregate() {
    return {
      timestamp: Date.now(),
      count: this.measurements.length,
      mean: this.calculateMean(),
      min: Math.min(...this.measurements),
      max: Math.max(...this.measurements),
      stddev: this.calculateStdDev()
    };
  }
}
```

---

## 5. Error Handling Protocol

### 5.1 Retry Strategy

#### 5.1.1 Exponential Backoff

```javascript
class RetryHandler {
  constructor() {
    this.maxRetries = 5;
    this.baseDelay = 1000;
    this.maxDelay = 60000;
  }

  async executeWithRetry(fn, attempt = 0) {
    try {
      return await fn();
    } catch (error) {
      if (attempt >= this.maxRetries) {
        throw error;
      }

      const delay = Math.min(
        this.baseDelay * Math.pow(2, attempt),
        this.maxDelay
      );

      // Add jitter to prevent thundering herd
      const jitter = Math.random() * 1000;

      await this.sleep(delay + jitter);
      return this.executeWithRetry(fn, attempt + 1);
    }
  }
}
```

#### 5.1.2 Circuit Breaker Pattern

```javascript
class CircuitBreaker {
  constructor(threshold = 5, timeout = 60000) {
    this.failureThreshold = threshold;
    this.timeout = timeout;
    this.failures = 0;
    this.state = 'CLOSED'; // CLOSED, OPEN, HALF_OPEN
    this.nextAttempt = Date.now();
  }

  async execute(fn) {
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
    this.failures = 0;
    this.state = 'CLOSED';
  }

  onFailure() {
    this.failures++;
    if (this.failures >= this.failureThreshold) {
      this.state = 'OPEN';
      this.nextAttempt = Date.now() + this.timeout;
    }
  }
}
```

### 5.2 Error Reporting

```json
{
  "error": {
    "code": "SENSOR_READ_FAILURE",
    "message": "Failed to read sensor data",
    "severity": "high",
    "timestamp": "2025-12-26T10:30:00Z",
    "context": {
      "sensorId": "sensor_temp_001",
      "farmId": "farm_sahara_001",
      "attemptCount": 3,
      "lastError": "Timeout after 5000ms"
    },
    "stackTrace": "Error stack trace...",
    "requestId": "req_abc123"
  }
}
```

---

## 6. Quality of Service (QoS)

### 6.1 Service Level Agreements

| Service | Uptime | Response Time | Throughput |
|---------|--------|---------------|------------|
| API | 99.9% | < 200ms (p95) | 10K req/s |
| WebSocket | 99.5% | < 50ms (p95) | 100K msg/s |
| Data Ingestion | 99.9% | < 100ms | 1M points/s |
| Alert Delivery | 99.99% | < 5s | N/A |

### 6.2 Performance Monitoring

```javascript
// Request performance tracking
const trackRequest = (endpoint, method, duration, status) => {
  metrics.record({
    metric: 'api.request.duration',
    value: duration,
    tags: {
      endpoint,
      method,
      status,
      region: 'us-east-1'
    }
  });

  // Alert if p95 exceeds threshold
  if (duration > 200 && percentile === 95) {
    alerts.trigger('SLOW_API_RESPONSE', {
      endpoint,
      duration,
      threshold: 200
    });
  }
};
```

### 6.3 Rate Limiting

```javascript
// Token bucket algorithm
class RateLimiter {
  constructor(capacity, refillRate) {
    this.capacity = capacity;
    this.tokens = capacity;
    this.refillRate = refillRate; // tokens per second
    this.lastRefill = Date.now();
  }

  async consume(tokens = 1) {
    this.refill();

    if (this.tokens >= tokens) {
      this.tokens -= tokens;
      return true;
    }

    throw new Error('Rate limit exceeded');
  }

  refill() {
    const now = Date.now();
    const elapsed = (now - this.lastRefill) / 1000;
    const tokensToAdd = elapsed * this.refillRate;

    this.tokens = Math.min(
      this.capacity,
      this.tokens + tokensToAdd
    );
    this.lastRefill = now;
  }
}
```

---

## 7. Monitoring and Logging Protocol

### 7.1 Structured Logging

```json
{
  "timestamp": "2025-12-26T10:30:00.123Z",
  "level": "INFO",
  "service": "wia-agri-021-api",
  "version": "1.0.0",
  "requestId": "req_abc123",
  "farmId": "farm_sahara_001",
  "event": "irrigation_started",
  "details": {
    "zoneId": "zone_a1",
    "duration": 1800,
    "flowRate": 15
  },
  "performance": {
    "duration_ms": 45,
    "cpu_usage": 12.5,
    "memory_mb": 256
  }
}
```

### 7.2 Metrics Collection

```javascript
// Prometheus-compatible metrics
const metrics = {
  // Counters
  'api_requests_total': counter({
    help: 'Total API requests',
    labelNames: ['endpoint', 'method', 'status']
  }),

  // Gauges
  'active_sensors': gauge({
    help: 'Number of active sensors',
    labelNames: ['farmId', 'type']
  }),

  // Histograms
  'request_duration_seconds': histogram({
    help: 'Request duration in seconds',
    labelNames: ['endpoint'],
    buckets: [0.1, 0.5, 1, 2, 5]
  })
};
```

### 7.3 Health Checks

```http
GET /health HTTP/1.1
Host: api.wia-agri-021.org

Response:
{
  "status": "healthy",
  "version": "1.0.0",
  "timestamp": "2025-12-26T10:30:00Z",
  "checks": {
    "database": "healthy",
    "cache": "healthy",
    "queue": "healthy",
    "storage": "healthy"
  },
  "metrics": {
    "uptime": 8640000,
    "requestsPerSecond": 1250,
    "errorRate": 0.001
  }
}
```

---

## 8. Compliance and Standards

This protocol specification complies with:

- **RFC 7230-7237**: HTTP/1.1
- **RFC 7540**: HTTP/2
- **RFC 6455**: WebSocket Protocol
- **RFC 3629**: MQTT Protocol
- **RFC 7252**: CoAP Protocol
- **RFC 8446**: TLS 1.3
- **ISO/IEC 27001**: Information Security
- **GDPR**: Data Protection
- **ISO 50001**: Energy Management

---

**弘益人間 (홍익인간) · Benefit All Humanity**

© 2025 SmileStory Inc. / WIA · All Rights Reserved
