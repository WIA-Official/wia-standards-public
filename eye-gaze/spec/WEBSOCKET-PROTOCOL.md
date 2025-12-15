# WIA Eye Gaze Standard - WebSocket Protocol Specification

**Version**: 1.0.0
**Phase**: 3 - Real-time Communication Protocol
**弘益人間** - 널리 인간을 이롭게

---

## 1. Overview

This specification defines the WebSocket-based real-time streaming protocol for the WIA Eye Gaze Standard. It enables low-latency gaze data transmission between eye tracker services and client applications.

### 1.1 Design Goals

- **Low Latency**: < 10ms transmission delay
- **High Throughput**: Support 60-120Hz sampling rates
- **Bidirectional**: Server pushes data, client sends control commands
- **Interoperable**: JSON-based messages with optional binary mode

---

## 2. Connection

### 2.1 Endpoint

```
WebSocket URL: ws://{host}:{port}/wia-eye-gaze/v1/stream
Default: ws://localhost:8765/wia-eye-gaze/v1/stream
Secure: wss://{host}:{port}/wia-eye-gaze/v1/stream
```

### 2.2 Connection Handshake

```http
GET /wia-eye-gaze/v1/stream HTTP/1.1
Host: localhost:8765
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==
Sec-WebSocket-Protocol: wia-eye-gaze-v1
Sec-WebSocket-Version: 13
```

### 2.3 Subprotocol

- Protocol Name: `wia-eye-gaze-v1`
- Content Type: `application/json` (default) or `application/octet-stream` (binary mode)

---

## 3. Message Format

### 3.1 Base Message Structure

```typescript
interface WiaMessage {
  type: MessageType;
  timestamp: number;      // Unix timestamp in milliseconds
  sequence?: number;      // Optional sequence number for ordering
}

type MessageType =
  | 'gaze_data'           // Server → Client: Gaze points
  | 'gaze_event'          // Server → Client: Gaze events
  | 'control'             // Client → Server: Control commands
  | 'status'              // Bidirectional: Status updates
  | 'error'               // Server → Client: Error messages
  | 'ping'                // Client → Server: Keep-alive
  | 'pong';               // Server → Client: Keep-alive response
```

---

## 4. Server → Client Messages

### 4.1 Gaze Data Stream

Primary message for real-time gaze data transmission.

```typescript
interface GazeDataMessage {
  type: 'gaze_data';
  timestamp: number;
  sequence: number;
  frequency: number;        // Current streaming frequency (Hz)
  points: GazePointData[];  // Batch of gaze points
}

interface GazePointData {
  t: number;     // Timestamp (ms)
  x: number;     // Normalized X (0.0-1.0)
  y: number;     // Normalized Y (0.0-1.0)
  c: number;     // Confidence (0.0-1.0)
  v: boolean;    // Valid flag
  le?: EyeData;  // Left eye (optional)
  re?: EyeData;  // Right eye (optional)
}

interface EyeData {
  x: number;     // Gaze X
  y: number;     // Gaze Y
  p?: number;    // Pupil diameter (mm)
  o?: number;    // Eye openness (0.0-1.0)
}
```

**Example:**
```json
{
  "type": "gaze_data",
  "timestamp": 1704067200000,
  "sequence": 12345,
  "frequency": 60,
  "points": [
    {
      "t": 1704067200000,
      "x": 0.5234,
      "y": 0.4821,
      "c": 0.95,
      "v": true,
      "le": { "x": 0.5200, "y": 0.4820, "p": 3.5, "o": 0.92 },
      "re": { "x": 0.5268, "y": 0.4822, "p": 3.6, "o": 0.91 }
    }
  ]
}
```

### 4.2 Gaze Event

```typescript
interface GazeEventMessage {
  type: 'gaze_event';
  timestamp: number;
  event: {
    eventType: GazeEventType;
    eventId: string;
    duration?: number;
    position?: { x: number; y: number };
    target?: GazeTarget;
  };
}

type GazeEventType =
  | 'FIXATION_START'
  | 'FIXATION_END'
  | 'SACCADE_START'
  | 'SACCADE_END'
  | 'BLINK'
  | 'DWELL_START'
  | 'DWELL_PROGRESS'
  | 'DWELL_COMPLETE'
  | 'DWELL_CANCEL'
  | 'GAZE_ENTER'
  | 'GAZE_EXIT';
```

**Example:**
```json
{
  "type": "gaze_event",
  "timestamp": 1704067200500,
  "event": {
    "eventType": "DWELL_COMPLETE",
    "eventId": "evt-001",
    "duration": 800,
    "position": { "x": 0.5, "y": 0.5 },
    "target": {
      "elementId": "btn-ok",
      "label": "OK Button"
    }
  }
}
```

### 4.3 Status Update

```typescript
interface StatusMessage {
  type: 'status';
  timestamp: number;
  status: {
    connected: boolean;
    tracking: boolean;
    calibrated: boolean;
    state: 'disconnected' | 'connecting' | 'connected' | 'calibrating' | 'tracking' | 'error';
    device?: {
      vendor: string;
      model: string;
      firmware: string;
    };
    metrics?: {
      samplingRate: number;
      latency: number;
      packetsDropped: number;
    };
  };
}
```

### 4.4 Error Message

```typescript
interface ErrorMessage {
  type: 'error';
  timestamp: number;
  error: {
    code: ErrorCode;
    message: string;
    details?: Record<string, unknown>;
    recoverable: boolean;
  };
}

type ErrorCode =
  | 'DEVICE_NOT_FOUND'
  | 'DEVICE_DISCONNECTED'
  | 'CALIBRATION_FAILED'
  | 'TRACKING_ERROR'
  | 'INVALID_MESSAGE'
  | 'RATE_LIMITED'
  | 'INTERNAL_ERROR';
```

---

## 5. Client → Server Messages

### 5.1 Control Commands

```typescript
interface ControlMessage {
  type: 'control';
  timestamp: number;
  requestId?: string;       // For response correlation
  action: ControlAction;
  params?: Record<string, unknown>;
}

type ControlAction =
  | 'start'           // Start tracking
  | 'stop'            // Stop tracking
  | 'pause'           // Pause streaming (tracker continues)
  | 'resume'          // Resume streaming
  | 'calibrate'       // Start calibration
  | 'set_frequency'   // Change streaming frequency
  | 'set_format'      // Switch to binary format
  | 'subscribe'       // Subscribe to specific events
  | 'unsubscribe';    // Unsubscribe from events
```

**Examples:**

Start tracking:
```json
{
  "type": "control",
  "timestamp": 1704067200000,
  "requestId": "req-001",
  "action": "start"
}
```

Set streaming frequency:
```json
{
  "type": "control",
  "timestamp": 1704067200000,
  "action": "set_frequency",
  "params": { "frequency": 120 }
}
```

Subscribe to events:
```json
{
  "type": "control",
  "timestamp": 1704067200000,
  "action": "subscribe",
  "params": {
    "events": ["FIXATION_START", "FIXATION_END", "DWELL_COMPLETE"]
  }
}
```

### 5.2 Ping/Pong (Keep-Alive)

```typescript
// Client → Server
interface PingMessage {
  type: 'ping';
  timestamp: number;
}

// Server → Client
interface PongMessage {
  type: 'pong';
  timestamp: number;
  serverTime: number;
}
```

---

## 6. Binary Mode

For high-performance scenarios, binary mode reduces bandwidth by 7x.

### 6.1 Enabling Binary Mode

```json
{
  "type": "control",
  "action": "set_format",
  "params": { "format": "binary" }
}
```

### 6.2 Binary Frame Format

See `BINARY-FORMAT.md` for detailed binary protocol specification.

```
┌────────────────────────────────────────────────────────────┐
│ Header (8 bytes)                                           │
├──────────┬──────────┬──────────┬──────────────────────────┤
│ Magic(2) │ Version  │ Type(1)  │ Payload Length (4)       │
│ 0x5749   │ (1)      │          │                          │
├──────────┴──────────┴──────────┴──────────────────────────┤
│ Payload (variable length)                                  │
└────────────────────────────────────────────────────────────┘
```

---

## 7. Flow Control

### 7.1 Backpressure

If client cannot keep up with data rate:

1. Client sends `pause` command
2. Server buffers data (max 1 second)
3. Client sends `resume` when ready
4. Server sends buffered data, then live data

### 7.2 Rate Limiting

Server may throttle clients exceeding limits:

| Limit | Default | Description |
|-------|---------|-------------|
| Max connections per IP | 10 | Concurrent WebSocket connections |
| Max message rate | 100/sec | Client → Server messages |
| Max data rate | 10 MB/sec | Total throughput per connection |

---

## 8. Reconnection

### 8.1 Automatic Reconnection

Clients SHOULD implement exponential backoff:

```typescript
const reconnect = async (attempt: number) => {
  const delay = Math.min(1000 * Math.pow(2, attempt), 30000);
  await sleep(delay);
  return connect();
};
```

### 8.2 Session Resumption

```json
{
  "type": "control",
  "action": "resume_session",
  "params": {
    "sessionId": "session-uuid",
    "lastSequence": 12345
  }
}
```

---

## 9. Security

### 9.1 Authentication

Optional token-based authentication:

```
ws://localhost:8765/wia-eye-gaze/v1/stream?token=<jwt>
```

### 9.2 TLS

Production deployments SHOULD use `wss://` (WebSocket Secure).

### 9.3 Origin Validation

Server SHOULD validate `Origin` header for browser clients.

---

## 10. TypeScript Client Example

```typescript
import { WiaGazeClient } from '@anthropics/wia-eye-gaze';

const client = new WiaGazeClient({
  url: 'ws://localhost:8765/wia-eye-gaze/v1/stream',
  autoReconnect: true,
});

client.on('gaze', (points) => {
  for (const point of points) {
    console.log(`Gaze: (${point.x.toFixed(3)}, ${point.y.toFixed(3)})`);
  }
});

client.on('event', (event) => {
  if (event.eventType === 'DWELL_COMPLETE') {
    console.log(`Selected: ${event.target?.label}`);
  }
});

await client.connect();
await client.start();
```

---

## 11. Latency Budget

| Stage | Max Latency | Notes |
|-------|-------------|-------|
| Eye → Tracker Hardware | 16ms | 60Hz capture |
| Tracker → Server Process | 5ms | USB/internal |
| Server Processing | 2ms | Filtering, events |
| WebSocket Transmission | 3ms | Local network |
| **Total** | **< 26ms** | Within 50ms budget |

---

## Appendix A: Message Type Codes

| Type | Code | Direction |
|------|------|-----------|
| gaze_data | 0x01 | S → C |
| gaze_event | 0x02 | S → C |
| control | 0x03 | C → S |
| status | 0x04 | Both |
| error | 0x05 | S → C |
| ping | 0x06 | C → S |
| pong | 0x07 | S → C |

---

**弘益人間** - 널리 인간을 이롭게
