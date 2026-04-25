# WIA-IND-004 Phase 3: Protocol Standard
## Real-Time Communication Protocols

**Version:** 1.0.0
**Status:** Active
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## Overview

Phase 3 defines communication protocols for real-time beauty technology interactions including AR try-on sessions, device synchronization, and live consultations.

## Core Protocol Principles

1. **Real-Time**: Low-latency bidirectional communication
2. **Event-Driven**: Message-based architecture
3. **Stateful**: Session management for continuous interactions
4. **Secure**: Encrypted WebSocket connections (WSS)
5. **Resilient**: Automatic reconnection and error recovery

---

## 1. AR Try-On Protocol

### Protocol Overview

WebSocket-based protocol for augmented reality makeup try-on sessions.

### Connection

```
wss://api.beauty.example.com/v1/ar-tryon
Authorization: Bearer <token>
```

### Message Format

All messages follow this structure:

```json
{
  "messageId": "uuid",
  "timestamp": "ISO 8601",
  "type": "MESSAGE_TYPE",
  "sessionId": "string",
  "payload": { /* type-specific data */ }
}
```

### Session Lifecycle

#### 1. SESSION_INIT

**Client → Server**

```json
{
  "type": "SESSION_INIT",
  "payload": {
    "userId": "user-123",
    "category": "lipstick|eyeshadow|foundation|blush|hair-color",
    "deviceCapabilities": {
      "camera": "front|back",
      "resolution": "1920x1080",
      "fps": 30
    },
    "renderEngine": "WebGL 2.0|Metal|DirectX"
  }
}
```

**Server → Client**

```json
{
  "type": "SESSION_STARTED",
  "sessionId": "ar-session-550e8400",
  "payload": {
    "sessionId": "ar-session-550e8400",
    "configuration": {
      "faceTrackingModel": "MediaPipe Face Mesh",
      "landmarkCount": 478,
      "targetFPS": 30
    }
  }
}
```

#### 2. FACE_DETECTION

**Client → Server** (continuous stream)

```json
{
  "type": "FACE_FRAME",
  "payload": {
    "frameId": "integer",
    "landmarks": [
      {"x": 0.5, "y": 0.3, "z": 0.1},
      /* 478 landmarks */
    ],
    "pose": {
      "pitch": -5.2,
      "yaw": 10.3,
      "roll": 2.1
    },
    "confidence": 0.96
  }
}
```

**Server → Client**

```json
{
  "type": "FACE_DETECTED",
  "payload": {
    "detected": true,
    "quality": "excellent|good|fair|poor",
    "lighting": "natural|indoor|low|bright",
    "warnings": ["face_too_dark", "extreme_angle"]
  }
}
```

#### 3. PRODUCT_APPLY

**Client → Server**

```json
{
  "type": "PRODUCT_APPLY",
  "payload": {
    "productType": "lipstick",
    "productId": "LP-001",
    "color": "#FF6B9D",
    "opacity": 0.85,
    "finish": "matte|glossy|satin",
    "blendMode": "multiply|screen|overlay"
  }
}
```

**Server → Client**

```json
{
  "type": "RENDER_INSTRUCTIONS",
  "payload": {
    "productType": "lipstick",
    "targetRegion": {
      "type": "lips",
      "landmarks": [61, 62, 63, /* ... */],
      "mask": "base64_encoded_mask"
    },
    "colorTransform": {
      "baseColor": "#FF6B9D",
      "highlightMultiplier": 1.2,
      "shadowMultiplier": 0.8
    },
    "textureUrl": "https://cdn.example.com/textures/matte.png",
    "lightingAdjustment": {
      "brightness": 5,
      "contrast": 3,
      "warmth": 10
    }
  }
}
```

#### 4. RENDER_UPDATE

**Client → Server** (performance metrics)

```json
{
  "type": "RENDER_METRICS",
  "payload": {
    "fps": 28,
    "latency": 45,
    "droppedFrames": 2,
    "cpuUsage": 35,
    "gpuUsage": 60
  }
}
```

#### 5. SCREENSHOT

**Client → Server**

```json
{
  "type": "CAPTURE_SCREENSHOT",
  "payload": {
    "includeWatermark": true,
    "resolution": "original|1080p|720p"
  }
}
```

**Server → Client**

```json
{
  "type": "SCREENSHOT_READY",
  "payload": {
    "imageUrl": "https://cdn.example.com/screenshots/abc123.jpg",
    "expiresAt": "ISO 8601",
    "productList": ["LP-001", "ES-045"]
  }
}
```

#### 6. SESSION_END

**Client → Server**

```json
{
  "type": "SESSION_END",
  "payload": {
    "reason": "user_quit|screenshot_taken|timeout",
    "duration": 125
  }
}
```

---

## 2. Device Sync Protocol

### Protocol Overview

Bluetooth Low Energy (BLE) GATT protocol for smart beauty devices.

### Service UUID

```
WIA_BEAUTY_SERVICE_UUID: "00001000-0000-1000-8000-00805f9b34fb"
```

### Characteristics

#### Device Info (Read)

**UUID:** `00001001-0000-1000-8000-00805f9b34fb`

**Value:**
```
{
  "deviceType": "cleansing-brush",
  "model": "CB-2025",
  "firmwareVersion": "2.1.4",
  "serialNumber": "CB2025-001234",
  "batteryLevel": 85
}
```

#### Usage Data (Read/Notify)

**UUID:** `00001002-0000-1000-8000-00805f9b34fb`

**Value:**
```
{
  "timestamp": "ISO 8601",
  "session": {
    "startTime": "ISO 8601",
    "duration": 60,
    "mode": "sensitive|normal|deep",
    "speed": 75,
    "pressure": 65
  }
}
```

#### Settings (Read/Write)

**UUID:** `00001003-0000-1000-8000-00805f9b34fb`

**Value:**
```
{
  "defaultMode": "sensitive",
  "defaultSpeed": 50,
  "timerEnabled": true,
  "timerDuration": 60,
  "hapticFeedback": true
}
```

#### Command (Write)

**UUID:** `00001004-0000-1000-8000-00805f9b34fb`

**Commands:**
```
START_SESSION
STOP_SESSION
SYNC_DATA
UPDATE_SETTINGS
REQUEST_STATUS
FACTORY_RESET
```

### Connection Flow

1. **Discovery**: Scan for devices advertising WIA_BEAUTY_SERVICE_UUID
2. **Pairing**: Secure pairing with PIN or button press
3. **Service Enumeration**: Read device info characteristic
4. **Subscription**: Enable notifications on usage data
5. **Initial Sync**: Read historical usage data
6. **Continuous Monitoring**: Receive real-time updates
7. **Disconnection**: Graceful shutdown

---

## 3. Live Consultation Protocol

### Protocol Overview

Video consultation protocol for real-time beauty consultations.

### Signaling

WebRTC signaling via WebSocket:

```
wss://api.beauty.example.com/v1/consultation
```

### Session Flow

#### 1. Create Consultation

**HTTP POST** `/v1/consultations`

```json
{
  "consultantId": "expert-123",
  "userId": "user-456",
  "type": "skin-analysis|product-recommendation|application-tutorial",
  "scheduledTime": "ISO 8601"
}
```

**Response:**
```json
{
  "consultationId": "consult-789",
  "signalingUrl": "wss://...",
  "iceServers": [
    {"urls": "stun:stun.example.com:3478"},
    {"urls": "turn:turn.example.com:3478", "credential": "..."}
  ]
}
```

#### 2. WebRTC Offer/Answer

**Client → Server** (Offer)

```json
{
  "type": "WEBRTC_OFFER",
  "consultationId": "consult-789",
  "payload": {
    "sdp": "v=0\r\no=- ... ",
    "type": "offer"
  }
}
```

**Server → Client** (Answer)

```json
{
  "type": "WEBRTC_ANSWER",
  "payload": {
    "sdp": "v=0\r\no=- ... ",
    "type": "answer"
  }
}
```

#### 3. ICE Candidates

```json
{
  "type": "ICE_CANDIDATE",
  "payload": {
    "candidate": "candidate:...",
    "sdpMid": "0",
    "sdpMLineIndex": 0
  }
}
```

#### 4. Annotation Overlay

**Consultant → Client**

```json
{
  "type": "ANNOTATION",
  "payload": {
    "tool": "circle|arrow|highlight|text",
    "coordinates": {"x": 0.5, "y": 0.3},
    "color": "#FF0000",
    "text": "Focus on this area",
    "duration": 5000
  }
}
```

#### 5. Product Share

**Consultant → Client**

```json
{
  "type": "PRODUCT_SHARE",
  "payload": {
    "productId": "BT-001",
    "action": "show_details|add_to_recommendation"
  }
}
```

---

## 4. Data Streaming Protocol

### Protocol Overview

Server-Sent Events (SSE) for real-time updates.

### Connection

```
GET /v1/stream/updates?userId=user-123
Accept: text/event-stream
Authorization: Bearer <token>
```

### Event Types

#### Product Update

```
event: product.updated
data: {"productId": "BT-001", "field": "price", "newValue": 79.99}
```

#### Skin Analysis Complete

```
event: skin.analysis.completed
data: {"analysisId": "analysis-abc", "overallScore": 78}
```

#### Recommendation Ready

```
event: recommendation.ready
data: {"recommendationId": "rec-123", "productCount": 5}
```

#### Device Connected

```
event: device.connected
data: {"deviceId": "dev-456", "deviceType": "cleansing-brush"}
```

### Heartbeat

```
event: heartbeat
data: {"timestamp": "2025-12-27T10:30:00Z"}
```

---

## Error Handling

### Error Message Format

```json
{
  "type": "ERROR",
  "payload": {
    "code": "FACE_NOT_DETECTED|CONNECTION_LOST|DEVICE_DISCONNECTED",
    "message": "Human-readable error message",
    "severity": "warning|error|critical",
    "recoverable": true,
    "suggestedAction": "Improve lighting and try again"
  }
}
```

### Reconnection Strategy

1. Initial disconnect: Wait 1 second
2. First retry: Wait 2 seconds
3. Second retry: Wait 4 seconds
4. Exponential backoff: Max 30 seconds
5. Give up after: 5 minutes

---

## Security

### Encryption

- WebSocket: TLS 1.3 (WSS)
- BLE: AES-128-CCM encryption
- WebRTC: DTLS-SRTP

### Authentication

- WebSocket: JWT in connection URL
- BLE: Secure pairing with PIN
- WebRTC: TURN credentials

### Data Validation

- Validate all incoming messages against schema
- Sanitize user inputs
- Rate limit message frequency
- Detect and block malicious patterns

---

## Performance Requirements

| Metric | Requirement |
|--------|-------------|
| AR FPS | ≥ 25 |
| AR Latency | ≤ 50ms |
| Video Quality | 720p @ 30fps min |
| Audio Quality | 48kHz, opus codec |
| BLE Notification Rate | 1-5 Hz |
| SSE Reconnect Time | ≤ 3 seconds |

---

## Compliance

Phase 3 compliant implementations must:
1. Implement WebSocket-based AR protocol
2. Support BLE GATT characteristics
3. Implement WebRTC for consultations
4. Use SSE for real-time updates
5. Encrypt all communications
6. Handle errors gracefully with reconnection
7. Meet performance requirements
8. Pass WIA protocol certification tests

---

**Maintained by:** WIA (World Certification Industry Association)
**Last Updated:** 2025-12-27

弘益人間 · Benefit All Humanity

---

## Annex A — Conformance Tier Matrix

WIA conformance for WIA-IND-004-beauty-tech is evaluated across three tiers:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references published standards alongside this document. Implementers SHOULD review the relevant standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- ISO/IEC 17065:2012 — Conformity assessment — Requirements for bodies certifying products, processes and services
- ISO/IEC 27001:2022 — Information security management systems
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- W3C PROV-DM — provenance data model

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/WIA-IND-004-beauty-tech/api/` — TypeScript SDK skeleton
- `wia-standards/standards/WIA-IND-004-beauty-tech/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/WIA-IND-004-beauty-tech/simulator/` — interactive browser-based simulator for the PHASE protocol

Each reference artifact ships with an MIT license and is intended as a starting point, not as a production-grade implementation.

### C.2 Test Vectors

A normative set of request/response pairs covering the schemas defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector in both serialization (request) and deserialization (response) directions, and MUST publish a signed report identifying which vectors were exercised.

Test vectors are versioned independently of the PHASE document; refer to the `test-vectors/` directory for the active version.

---

## Annex D — Open Questions and Future Work

This PHASE document captures the consensus position at v1.0. The following items are tracked for future minor or major revisions:

1. **Schema versioning policy** — formal MUST/SHOULD rules for backward-compatible vs. breaking changes between minor releases.
2. **Privacy-preserving aggregation** — guidance on differential privacy and secure aggregation patterns for telemetry channels, without prescribing a specific algorithm.
3. **Multilingual error catalogs** — localization strategy for the standard error codes defined in this PHASE.
4. **Long-term retention** — alignment with sectoral retention regulations across jurisdictions.
5. **Sustainability disclosure** — optional fields for energy and emissions reporting tied to operations covered by this PHASE.

Items in this annex are non-normative. Comments and proposals are accepted via the GitHub issues tracker on the WIA-Official organization.

---
