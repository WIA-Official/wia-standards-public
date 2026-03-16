# WIA-IND-011 PHASE 3: PROTOCOL SPECIFICATION
## Sports Analytics Standard - Real-Time Streaming and Event Synchronization

**Standard:** WIA-IND-011
**Phase:** 3 of 4
**Version:** 1.0
**Status:** Active
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 3 of the WIA-IND-011 Sports Analytics Standard defines real-time streaming protocols, event synchronization mechanisms, and low-latency data delivery systems for live sports analytics. This ensures consistent, reliable, and timely delivery of analytics during matches and training sessions.

**Guiding Principle (弘益人間):** Real-time data should be accessible to all stakeholders - from professional broadcasts to grassroots analytics - ensuring everyone benefits from immediate insights.

---

## 2. Real-Time Streaming Architecture

### 2.1 Protocol Stack

```
┌─────────────────────────────────────┐
│   Application Layer (Analytics)     │
├─────────────────────────────────────┤
│   WIA Protocol Layer (Sync/Events)  │
├─────────────────────────────────────┤
│   Transport Layer (WebSocket/gRPC)  │
├─────────────────────────────────────┤
│   Network Layer (TCP/IP)            │
└─────────────────────────────────────┘
```

### 2.2 Supported Transport Protocols

1. **WebSocket** (Primary for web clients)
   - Full-duplex communication
   - Low latency (<100ms)
   - Browser compatible

2. **gRPC Streaming** (Primary for server-to-server)
   - High performance binary protocol
   - Bi-directional streaming
   - Built-in flow control

3. **Server-Sent Events (SSE)** (Fallback)
   - One-way server-to-client
   - HTTP-based, firewall-friendly
   - Automatic reconnection

4. **MQTT** (IoT devices)
   - Lightweight pub/sub protocol
   - Low bandwidth requirements
   - Quality of Service (QoS) levels

---

## 3. WebSocket Protocol

### 3.1 Connection Establishment

```javascript
// Client connection
const ws = new WebSocket('wss://stream.sports-analytics.org/v1/live');

ws.onopen = () => {
  // Authentication
  ws.send(JSON.stringify({
    type: 'auth',
    token: 'Bearer eyJhbGc...''
    standard': 'WIA-IND-011',
    philosophy: '弘益人間'
  }));
};
```

### 3.2 Message Format

```json
{
  "standard": "WIA-IND-011",
  "philosophy": "弘益人間",
  "version": "1.0",
  "messageId": "msg_abc123",
  "timestamp": "2025-01-15T16:45:23.456Z",
  "type": "event",
  "channel": "match:M789012",
  "sequence": 12345,
  "data": {
    "eventType": "goal",
    "playerId": "P123456",
    "matchClock": 2834,
    "location": {"x": 85.5, "y": 45.2}
  }
}
```

### 3.3 Subscription Management

**Subscribe to Channels:**
```json
{
  "type": "subscribe",
  "channels": [
    "match:M789012",
    "player:P123456:tracking",
    "team:T789:stats"
  ]
}
```

**Unsubscribe:**
```json
{
  "type": "unsubscribe",
  "channels": ["match:M789012"]
}
```

### 3.4 Heartbeat and Keep-Alive

```json
{
  "type": "ping",
  "timestamp": "2025-01-15T16:45:30.000Z"
}
```

**Response:**
```json
{
  "type": "pong",
  "timestamp": "2025-01-15T16:45:30.012Z",
  "latency": 12
}
```

---

## 4. Event Synchronization Protocol

### 4.1 Event Ordering

All events include:
- `timestamp`: ISO8601 UTC timestamp (microsecond precision)
- `sequence`: Monotonically increasing integer per channel
- `matchClock`: Seconds from match start (for match events)

### 4.2 Clock Synchronization

**Server Clock Reference:**
```json
{
  "type": "clockSync",
  "standard": "WIA-IND-011",
  "philosophy": "弘益人間 - Synchronized time benefits all",
  "serverTime": "2025-01-15T16:45:30.123456Z",
  "matchId": "M789012",
  "matchClock": 2834.567,
  "matchState": "live"
}
```

**Client Adjustment:**
```javascript
function syncClock(serverMessage) {
  const clientReceiveTime = Date.now();
  const serverTime = new Date(serverMessage.serverTime).getTime();
  const roundTripLatency = clientReceiveTime - serverTime;
  const estimatedOffset = roundTripLatency / 2;

  // Adjust local clock
  return serverTime + estimatedOffset;
}
```

### 4.3 Event Sequencing Guarantee

- **Exactly-once delivery** per subscription
- **Ordered delivery** within same channel
- **Sequence gap detection** triggers re-sync request

**Gap Detection:**
```json
{
  "type": "resync",
  "channel": "match:M789012",
  "lastSequence": 12340,
  "expectedSequence": 12345
}
```

---

## 5. Data Stream Types

### 5.1 Match Event Stream

**Channel:** `match:{matchId}`

**Event Types:**
- `match.started`
- `match.ended`
- `match.paused`
- `match.resumed`
- `event.goal`
- `event.shot`
- `event.pass`
- `event.tackle`
- `event.foul`
- `event.card`
- `event.substitution`

**Example:**
```json
{
  "standard": "WIA-IND-011",
  "philosophy": "弘益人間",
  "type": "event",
  "channel": "match:M789012",
  "timestamp": "2025-01-15T16:47:12.345Z",
  "sequence": 12567,
  "data": {
    "eventType": "goal",
    "eventId": "E12567",
    "matchClock": 2912,
    "playerId": "P123456",
    "teamId": "T789",
    "location": {"x": 88.5, "y": 48.2},
    "xG": 0.42,
    "assist": "P234567"
  }
}
```

### 5.2 Player Tracking Stream

**Channel:** `player:{playerId}:tracking`

**Update Frequency:** 10 Hz (10 updates per second)

**Data:**
```json
{
  "standard": "WIA-IND-011",
  "philosophy": "弘益人間",
  "type": "tracking",
  "channel": "player:P123456:tracking",
  "timestamp": "2025-01-15T16:47:12.500Z",
  "sequence": 290125,
  "data": {
    "playerId": "P123456",
    "position": {"x": 45.2, "y": 38.7},
    "velocity": {"magnitude": 6.8, "direction": 135},
    "heartRate": 172,
    "speed": 24.5,
    "acceleration": 2.1
  }
}
```

### 5.3 Team Statistics Stream

**Channel:** `team:{teamId}:stats`

**Update Frequency:** 1 Hz (every second)

**Data:**
```json
{
  "standard": "WIA-IND-011",
  "philosophy": "弘益人間",
  "type": "stats",
  "channel": "team:T789:stats",
  "timestamp": "2025-01-15T16:47:13.000Z",
  "sequence": 2913,
  "data": {
    "teamId": "T789",
    "matchId": "M789012",
    "matchClock": 2913,
    "stats": {
      "possession": 58.3,
      "passesCompleted": 456,
      "passAccuracy": 87.2,
      "shots": 12,
      "shotsOnTarget": 5,
      "xG": 1.85,
      "distanceCovered": 89567.5
    }
  }
}
```

### 5.4 Aggregated Analytics Stream

**Channel:** `analytics:{analyticsId}`

**Custom analytics delivered in real-time:**
```json
{
  "standard": "WIA-IND-011",
  "philosophy": "弘益人間 - Real-time insights for all",
  "type": "analytics",
  "channel": "analytics:pressing_intensity",
  "timestamp": "2025-01-15T16:47:15.000Z",
  "data": {
    "metric": "PPDA",
    "team": "T789",
    "value": 8.4,
    "trend": "increasing",
    "window": "last_5_min"
  }
}
```

---

## 6. Quality of Service (QoS)

### 6.1 QoS Levels

| Level | Description | Use Case | Latency Target |
|-------|-------------|----------|----------------|
| **Level 0** | Best effort | Fan apps, stats displays | <1000ms |
| **Level 1** | Guaranteed delivery | Broadcasting, betting | <500ms |
| **Level 2** | Ordered guaranteed | Tracking, analytics | <200ms |
| **Level 3** | High-priority real-time | VAR systems, critical | <100ms |

### 6.2 Latency Monitoring

```json
{
  "type": "latencyReport",
  "standard": "WIA-IND-011",
  "philosophy": "弘益人間",
  "measurements": {
    "clientId": "client_xyz",
    "channel": "match:M789012",
    "avgLatency": 45,
    "p50Latency": 42,
    "p95Latency": 78,
    "p99Latency": 125,
    "maxLatency": 156,
    "measurementWindow": 60
  }
}
```

---

## 7. Flow Control and Backpressure

### 7.1 Client-Side Buffering

```javascript
class WIAStreamBuffer {
  constructor(maxSize = 1000) {
    this.buffer = [];
    this.maxSize = maxSize;
    this.philosophy = '弘益人間';
  }

  push(message) {
    if (this.buffer.length >= this.maxSize) {
      // Drop oldest message or request slow-down
      this.requestSlowDown();
    }
    this.buffer.push(message);
  }

  requestSlowDown() {
    ws.send(JSON.stringify({
      type: 'flowControl',
      action: 'slowDown',
      bufferUtilization: 0.95
    }));
  }
}
```

### 7.2 Server-Side Throttling

```json
{
  "type": "throttle",
  "reason": "client_backpressure",
  "newRate": "5Hz",
  "duration": 30
}
```

---

## 8. Reconnection and Recovery

### 8.1 Automatic Reconnection

```javascript
class WIAConnection {
  constructor(url) {
    this.url = url;
    this.reconnectAttempts = 0;
    this.maxReconnectAttempts = 10;
    this.philosophy = '弘益人間';
  }

  connect() {
    this.ws = new WebSocket(this.url);

    this.ws.onclose = () => {
      const delay = Math.min(1000 * Math.pow(2, this.reconnectAttempts), 30000);
      setTimeout(() => this.reconnect(), delay);
    };
  }

  reconnect() {
    if (this.reconnectAttempts < this.maxReconnectAttempts) {
      this.reconnectAttempts++;
      console.log(`弘益人間 - Reconnecting (attempt ${this.reconnectAttempts})...`);
      this.connect();
    }
  }
}
```

### 8.2 State Recovery

**Request missed events:**
```json
{
  "type": "recover",
  "channel": "match:M789012",
  "fromSequence": 12340,
  "toSequence": 12567
}
```

**Response:**
```json
{
  "type": "recovery",
  "channel": "match:M789012",
  "events": [
    {"sequence": 12341, "data": {...}},
    {"sequence": 12342, "data": {...}},
    ...
  ]
}
```

---

## 9. Security

### 9.1 Connection Security

- **TLS 1.3 Required** for all connections
- **Certificate Pinning** recommended for mobile apps
- **Token-based Authentication** (JWT)
- **IP Whitelisting** available for enterprise

### 9.2 Data Integrity

**Message Authentication Code (MAC):**
```json
{
  "standard": "WIA-IND-011",
  "philosophy": "弘益人間",
  "data": {...},
  "mac": "HMAC-SHA256 of data + secret"
}
```

---

## 10. Monitoring and Observability

### 10.1 Connection Metrics

- Active connections count
- Messages per second
- Average latency
- Error rate
- Reconnection rate

### 10.2 Health Check

**Endpoint:** `GET /v1/stream/health`

**Response:**
```json
{
  "standard": "WIA-IND-011",
  "philosophy": "弘益人間",
  "status": "healthy",
  "activeConnections": 15420,
  "messagesPerSecond": 8567,
  "avgLatency": 45,
  "uptime": 2592000
}
```

---

## 11. Implementation Examples

### 11.1 JavaScript Client

```javascript
import { WIAStreamClient } from '@wia/stream-client';

const client = new WIAStreamClient({
  url: 'wss://stream.sports-analytics.org/v1/live',
  apiKey: 'wia_live_1234567890',
  philosophy: '弘益人間'
});

// Subscribe to match events
client.subscribe('match:M789012', (event) => {
  console.log('Match event:', event);
});

// Subscribe to player tracking
client.subscribe('player:P123456:tracking', (tracking) => {
  console.log('Player position:', tracking.data.position);
}, { qos: 2, sampleRate: '10Hz' });

// Handle connection status
client.on('connected', () => console.log('弘益人間 - Connected!'));
client.on('disconnected', () => console.log('Disconnected, reconnecting...'));
client.on('error', (error) => console.error('Stream error:', error));
```

### 11.2 Python Client

```python
from wia_stream import WIAStreamClient

async def on_event(event):
    print(f"弘益人間 - Event: {event}")

client = WIAStreamClient(
    url='wss://stream.sports-analytics.org/v1/live',
    api_key='wia_live_1234567890'
)

await client.connect()
await client.subscribe('match:M789012', on_event)

# Keep connection alive
await client.run_forever()
```

---

## 12. Protocol Extensions

### 12.1 Binary Protocol (Protocol Buffers)

For ultra-low latency applications:

```protobuf
syntax = "proto3";

message WIAEvent {
  string standard = 1; // "WIA-IND-011"
  string philosophy = 2; // "弘益人間"
  int64 timestamp = 3;
  uint64 sequence = 4;
  string channel = 5;
  EventData data = 6;
}

message EventData {
  string eventType = 1;
  string playerId = 2;
  Location location = 3;
  double xG = 4;
}
```

---

## 13. Best Practices

1. **Always handle reconnections** gracefully
2. **Implement sequence gap detection**
3. **Use appropriate QoS levels** for your use case
4. **Monitor latency** and adjust if needed
5. **Respect flow control** signals from server
6. **Buffer messages** during processing delays
7. **Validate message integrity** using MAC
8. **Log connection metrics** for debugging
9. **Test with simulated network issues**
10. **Remember 弘益人間** - design for all users

---

## 14. Compliance Requirements

- [ ] TLS 1.3 encryption
- [ ] JWT authentication
- [ ] Sequence number tracking
- [ ] Automatic reconnection
- [ ] Message buffering
- [ ] Latency monitoring
- [ ] Error handling
- [ ] Philosophy field ("弘益人間") in all messages

---

**Document Version:** 1.0
**Last Updated:** 2025-01-15
**Maintained by:** WIA Technical Committee
**Philosophy:** 弘益人間 - Benefit All Humanity

© 2025 SmileStory Inc. / WIA
Licensed under MIT License
