# WIA-DATA-011: PHASE 3 - Protocol Specification

**Version:** 1.0
**Status:** Active
**Last Updated:** 2025-12-26

## Overview

This specification defines the real-time protocols for data visualization, including WebSocket connections, streaming updates, and event-driven architectures compliant with WIA-DATA-011.

## Table of Contents

1. [WebSocket Protocol](#websocket-protocol)
2. [Server-Sent Events (SSE)](#server-sent-events-sse)
3. [Real-Time Data Streaming](#real-time-data-streaming)
4. [Message Format](#message-format)
5. [Connection Management](#connection-management)
6. [Security](#security)

---

## 1. WebSocket Protocol

### 1.1 Connection Establishment

**Endpoint:**

```
wss://api.example.com/v1/viz/stream
```

**Connection Parameters:**

```javascript
const ws = new WebSocket('wss://api.example.com/v1/viz/stream', {
  headers: {
    'Authorization': 'Bearer YOUR_API_KEY'
  }
});
```

### 1.2 Message Types

#### Client to Server

**Subscribe to Dataset**

```json
{
  "type": "subscribe",
  "channel": "dataset",
  "dataset_id": "sales-2025",
  "options": {
    "sample_rate": 1000,
    "fields": ["timestamp", "value"],
    "filter": "value > 100"
  }
}
```

**Unsubscribe**

```json
{
  "type": "unsubscribe",
  "channel": "dataset",
  "dataset_id": "sales-2025"
}
```

**Ping**

```json
{
  "type": "ping",
  "timestamp": 1735210200000
}
```

#### Server to Client

**Data Update**

```json
{
  "type": "data_update",
  "channel": "dataset",
  "dataset_id": "sales-2025",
  "timestamp": 1735210200000,
  "data": [
    { "timestamp": "2025-12-26T10:30:00Z", "value": 150 }
  ]
}
```

**Aggregated Update**

```json
{
  "type": "aggregated_update",
  "channel": "dataset",
  "dataset_id": "sales-2025",
  "interval": "1m",
  "timestamp": 1735210200000,
  "metrics": {
    "count": 100,
    "sum": 15000,
    "avg": 150,
    "min": 100,
    "max": 200,
    "stddev": 25
  }
}
```

**Pong**

```json
{
  "type": "pong",
  "timestamp": 1735210200000
}
```

**Error**

```json
{
  "type": "error",
  "code": "SUBSCRIPTION_ERROR",
  "message": "Dataset not found",
  "dataset_id": "invalid-id"
}
```

### 1.3 Subscription Channels

| Channel | Description | Example |
|---------|-------------|---------|
| `dataset` | Raw data updates | Real-time sensor data |
| `aggregated` | Pre-aggregated metrics | Minutely averages |
| `visualization` | Visualization state changes | Chart configuration updates |
| `insights` | AI-generated insights | Anomaly detection alerts |

---

## 2. Server-Sent Events (SSE)

### 2.1 Endpoint

```
GET /v1/viz/stream/sse?dataset_id=sales-2025
```

### 2.2 Event Stream Format

```
event: data_update
data: {"timestamp": "2025-12-26T10:30:00Z", "value": 150}

event: aggregated_update
data: {"interval": "1m", "metrics": {"count": 100, "avg": 150}}

event: heartbeat
data: {"timestamp": 1735210200000}
```

### 2.3 Client Implementation

```javascript
const eventSource = new EventSource(
  '/v1/viz/stream/sse?dataset_id=sales-2025',
  {
    headers: {
      'Authorization': 'Bearer YOUR_API_KEY'
    }
  }
);

eventSource.addEventListener('data_update', (event) => {
  const data = JSON.parse(event.data);
  updateVisualization(data);
});

eventSource.addEventListener('error', (event) => {
  console.error('SSE Error:', event);
  // Implement reconnection logic
});
```

---

## 3. Real-Time Data Streaming

### 3.1 Streaming Modes

| Mode | Description | Use Case |
|------|-------------|----------|
| `raw` | Send every data point | High-frequency trading, IoT sensors |
| `sampled` | Sample at fixed rate | Reduce bandwidth for high-volume data |
| `aggregated` | Pre-aggregate before sending | Dashboard metrics, KPIs |
| `delta` | Send only changes | State synchronization |

### 3.2 Configuration

```json
{
  "type": "subscribe",
  "channel": "dataset",
  "dataset_id": "sensor-data",
  "streaming_config": {
    "mode": "sampled",
    "sample_rate": 1000,
    "buffer_size": 100,
    "compression": "gzip",
    "format": "json"
  }
}
```

### 3.3 Backpressure Handling

```json
{
  "type": "backpressure_config",
  "strategy": "drop_oldest",
  "buffer_size": 1000,
  "drop_threshold": 0.9
}
```

**Strategies:**

- `drop_oldest`: Drop oldest messages when buffer full
- `drop_newest`: Drop newest messages
- `throttle`: Slow down source
- `pause`: Pause streaming until buffer cleared

---

## 4. Message Format

### 4.1 Base Message Structure

All messages MUST conform to:

```json
{
  "type": "message_type",
  "timestamp": 1735210200000,
  "version": "1.0",
  "payload": {
    ...
  }
}
```

### 4.2 Binary Message Format

For high-performance scenarios, use binary format (MessagePack, Protocol Buffers):

**MessagePack Example:**

```javascript
import msgpack from 'msgpack-lite';

const message = {
  type: 'data_update',
  data: [...]
};

const binary = msgpack.encode(message);
ws.send(binary);
```

### 4.3 Compression

Supported compression algorithms:

| Algorithm | Compression | Speed | Bandwidth Savings |
|-----------|-------------|-------|-------------------|
| `gzip` | High | Medium | 70-80% |
| `deflate` | High | Medium | 70-80% |
| `brotli` | Very High | Slow | 75-85% |
| `lz4` | Medium | Very Fast | 50-60% |

**Enable compression:**

```json
{
  "type": "subscribe",
  "compression": "gzip"
}
```

---

## 5. Connection Management

### 5.1 Heartbeat/Ping-Pong

**Client Ping Interval:** 30 seconds

```javascript
setInterval(() => {
  ws.send(JSON.stringify({
    type: 'ping',
    timestamp: Date.now()
  }));
}, 30000);
```

**Server Pong Timeout:** 60 seconds

If no pong received within 60s, client should reconnect.

### 5.2 Reconnection Strategy

**Exponential Backoff:**

```javascript
class ReconnectingWebSocket {
  constructor(url) {
    this.url = url;
    this.reconnectDelay = 1000;
    this.maxReconnectDelay = 30000;
    this.reconnectAttempts = 0;
    this.connect();
  }

  connect() {
    this.ws = new WebSocket(this.url);

    this.ws.onclose = () => {
      const delay = Math.min(
        this.reconnectDelay * Math.pow(2, this.reconnectAttempts),
        this.maxReconnectDelay
      );

      console.log(`Reconnecting in ${delay}ms...`);

      setTimeout(() => {
        this.reconnectAttempts++;
        this.connect();
      }, delay);
    };

    this.ws.onopen = () => {
      console.log('Connected');
      this.reconnectAttempts = 0;
      this.reconnectDelay = 1000;
    };
  }
}
```

### 5.3 Connection Lifecycle

```
[Client]                [Server]
   |                       |
   |--- Connect ---------->|
   |<-- Acknowledge -------|
   |                       |
   |--- Subscribe -------->|
   |<-- Subscribed --------|
   |                       |
   |<== Data Stream =======|
   |                       |
   |--- Ping ------------->|
   |<-- Pong --------------|
   |                       |
   |--- Unsubscribe ------>|
   |<-- Unsubscribed ------|
   |                       |
   |--- Disconnect ------->|
   |<-- Close -------------|
```

---

## 6. Security

### 6.1 Authentication

**Token-based:**

```javascript
const ws = new WebSocket('wss://api.example.com/v1/viz/stream', {
  headers: {
    'Authorization': 'Bearer YOUR_JWT_TOKEN'
  }
});
```

**Query Parameter (less secure):**

```
wss://api.example.com/v1/viz/stream?token=YOUR_TOKEN
```

### 6.2 Encryption

- All connections MUST use TLS 1.2+ (`wss://`, not `ws://`)
- Certificate validation MUST be enforced
- Perfect Forward Secrecy (PFS) recommended

### 6.3 Rate Limiting

**Connection Limits:**

| Tier | Concurrent Connections | Messages/Second |
|------|----------------------|-----------------|
| Free | 5 | 10 |
| Pro | 50 | 100 |
| Enterprise | 500 | 1000 |

**Exceeded Limit Response:**

```json
{
  "type": "error",
  "code": "RATE_LIMIT_EXCEEDED",
  "message": "Too many connections",
  "max_connections": 5,
  "current_connections": 6
}
```

### 6.4 Message Signing

Optional message signing for critical data:

```json
{
  "type": "data_update",
  "data": [...],
  "signature": "HMAC-SHA256 signature",
  "nonce": "unique-nonce"
}
```

**Verification:**

```javascript
const hmac = crypto.createHmac('sha256', SECRET_KEY);
hmac.update(JSON.stringify(message.data) + message.nonce);
const signature = hmac.digest('hex');

if (signature !== message.signature) {
  throw new Error('Invalid signature');
}
```

---

## 7. Protocol Extensions

### 7.1 Binary Data Transfer

For large datasets, use binary frames:

```
[Header: 16 bytes]
[Length: 4 bytes]
[Payload: variable]
```

### 7.2 Multiplexing

Multiple subscriptions over single connection:

```json
{
  "type": "subscribe",
  "subscriptions": [
    { "channel": "dataset", "dataset_id": "sales-2025" },
    { "channel": "dataset", "dataset_id": "traffic-2025" },
    { "channel": "insights", "dataset_id": "sales-2025" }
  ]
}
```

---

## Appendix A: Message Type Reference

| Type | Direction | Description |
|------|-----------|-------------|
| `subscribe` | C→S | Subscribe to channel |
| `unsubscribe` | C→S | Unsubscribe from channel |
| `ping` | C→S | Heartbeat request |
| `pong` | S→C | Heartbeat response |
| `data_update` | S→C | New data points |
| `aggregated_update` | S→C | Aggregated metrics |
| `error` | S→C | Error message |
| `subscribed` | S→C | Subscription confirmed |
| `unsubscribed` | S→C | Unsubscription confirmed |

**C→S**: Client to Server
**S→C**: Server to Client

---

## Appendix B: Error Codes

| Code | Description |
|------|-------------|
| `UNAUTHORIZED` | Invalid or missing authentication |
| `SUBSCRIPTION_ERROR` | Failed to subscribe |
| `DATASET_NOT_FOUND` | Requested dataset doesn't exist |
| `RATE_LIMIT_EXCEEDED` | Too many connections or messages |
| `INVALID_MESSAGE` | Malformed message |
| `INTERNAL_ERROR` | Server error |

---

**© 2025 WIA (World Certification Industry Association)**
**Standard:** WIA-DATA-011
**Phase:** 3 - Protocol Specification
