# WIA-IND-013: E-Sports Platform Standard
## Phase 3: Real-Time Protocol Specification
### 弘益人間 - Broadly Benefiting Humanity

**Version:** 1.0.0
**Status:** Final
**Last Updated:** 2025-01-15

---

## Table of Contents

1. [Introduction](#introduction)
2. [WebSocket Protocol](#websocket-protocol)
3. [Real-Time Match Sync](#real-time-match-sync)
4. [Streaming Protocols](#streaming-protocols)
5. [Event Broadcasting](#event-broadcasting)
6. [Chat Protocols](#chat-protocols)
7. [State Synchronization](#state-synchronization)

---

## Introduction

Phase 3 establishes real-time communication protocols for live match broadcasting, spectator features, and synchronized game state updates. WebSocket-based protocols ensure low-latency data transmission for competitive events.

### Design Principles

- **Low Latency:** Sub-500ms target for critical events
- **Reliability:** Guaranteed message delivery for important events
- **Scalability:** Support millions of concurrent connections
- **Efficiency:** Minimal bandwidth usage
- **Philosophy:** 弘益人間 through accessible real-time data

---

## WebSocket Protocol

### Connection Establishment

```javascript
const ws = new WebSocket('wss://realtime.platform.com/wia-ind-013/v1');

ws.onopen = () => {
  // Authenticate
  ws.send(JSON.stringify({
    type: 'auth',
    token: 'Bearer eyJhbGciOiJSUzI1NiIs...'
  }));
};
```

### Authentication Message

```json
{
  "type": "auth",
  "token": "Bearer {access_token}",
  "clientId": "client-abc123",
  "version": "1.0.0"
}
```

**Response:**
```json
{
  "type": "auth_success",
  "sessionId": "session-xyz789",
  "expiresIn": 3600,
  "capabilities": [
    "match_updates",
    "chat",
    "predictions"
  ]
}
```

### Heartbeat/Ping-Pong

Client sends ping every 30 seconds:
```json
{"type": "ping", "timestamp": 1642694400}
```

Server responds:
```json
{"type": "pong", "timestamp": 1642694401}
```

### Subscription Model

```json
{
  "type": "subscribe",
  "channels": [
    "match:match-2025-001",
    "tournament:tournament-wc-2025",
    "chat:global"
  ]
}
```

**Response:**
```json
{
  "type": "subscribed",
  "channels": [
    "match:match-2025-001",
    "tournament:tournament-wc-2025",
    "chat:global"
  ]
}
```

---

## Real-Time Match Sync

### Game State Updates

```json
{
  "type": "game_state",
  "matchId": "match-2025-001",
  "timestamp": "2025-01-15T18:25:30.123Z",
  "gameTime": 1530,
  "state": {
    "score": {
      "team_blue": 12,
      "team_red": 8
    },
    "objectives": {
      "dragons": {"blue": 2, "red": 1},
      "towers": {"blue": 6, "red": 3},
      "baron": {"blue": 0, "red": 0}
    },
    "players": [
      {
        "playerId": "player-faker",
        "team": "blue",
        "champion": "Azir",
        "alive": true,
        "position": {"x": 1245, "y": 3421},
        "stats": {
          "kills": 5,
          "deaths": 1,
          "assists": 7,
          "gold": 12450,
          "cs": 234
        }
      }
    ]
  },
  "sequence": 15234
}
```

### Event Notifications

```json
{
  "type": "game_event",
  "matchId": "match-2025-001",
  "eventId": "event-12345",
  "timestamp": "2025-01-15T18:25:45.678Z",
  "gameTime": 1545,
  "eventType": "kill",
  "priority": "high",
  "data": {
    "killer": "player-faker",
    "victim": "player-chovy",
    "assists": ["player-keria", "player-zeus"],
    "position": {"x": 1200, "y": 3400},
    "killStreak": 3
  },
  "sequence": 15235
}
```

### Event Types

| Type | Priority | Description |
|------|----------|-------------|
| `kill` | high | Player elimination |
| `first_blood` | critical | First kill |
| `multi_kill` | high | Multiple kills |
| `objective_taken` | critical | Dragon/Baron/Tower |
| `ace` | critical | Team wipe |
| `item_purchased` | low | Equipment buy |
| `ability_upgrade` | low | Skill level up |
| `game_pause` | critical | Match paused |
| `game_resume` | critical | Match resumed |
| `game_end` | critical | Match concluded |

---

## Streaming Protocols

### Video Stream Metadata

```json
{
  "type": "stream_metadata",
  "matchId": "match-2025-001",
  "streams": [
    {
      "streamId": "stream-primary",
      "type": "primary_broadcast",
      "url": "https://cdn.platform.com/live/match-2025-001/primary.m3u8",
      "quality": [
        {"resolution": "1080p60", "bitrate": 6000},
        {"resolution": "720p60", "bitrate": 3000},
        {"resolution": "480p30", "bitrate": 1500}
      ],
      "latency": "low",
      "protocol": "HLS"
    },
    {
      "streamId": "stream-player-faker",
      "type": "player_pov",
      "playerId": "player-faker",
      "url": "https://cdn.platform.com/live/match-2025-001/faker.m3u8",
      "quality": [
        {"resolution": "1080p60", "bitrate": 6000}
      ]
    }
  ]
}
```

### WebRTC Signaling

For ultra-low latency WebRTC streams:

```json
{
  "type": "webrtc_offer",
  "streamId": "stream-primary",
  "sdp": "v=0\r\no=- 123456789 2 IN IP4 127.0.0.1\r\n...",
  "iceServers": [
    {"urls": "stun:stun.platform.com:3478"},
    {
      "urls": "turn:turn.platform.com:3478",
      "username": "user123",
      "credential": "pass456"
    }
  ]
}
```

**Client Response:**
```json
{
  "type": "webrtc_answer",
  "streamId": "stream-primary",
  "sdp": "v=0\r\no=- 987654321 2 IN IP4 192.168.1.1\r\n..."
}
```

### Stream Quality Adaptation

```json
{
  "type": "quality_change",
  "streamId": "stream-primary",
  "quality": "720p60",
  "reason": "bandwidth_limitation",
  "measuredBandwidth": 2500
}
```

---

## Event Broadcasting

### Tournament Updates

```json
{
  "type": "tournament_update",
  "tournamentId": "tournament-wc-2025",
  "updateType": "match_completed",
  "data": {
    "matchId": "match-qf1",
    "result": {
      "winner": "team-t1",
      "score": "3-1"
    },
    "bracketUpdates": {
      "nextMatch": "match-sf1",
      "advancingTeam": "team-t1"
    }
  }
}
```

### Bracket Updates

```json
{
  "type": "bracket_update",
  "tournamentId": "tournament-wc-2025",
  "roundId": "semifinals",
  "matches": [
    {
      "matchId": "match-sf1",
      "participants": [
        {"teamId": "team-t1", "seed": 1},
        {"teamId": "team-drx", "seed": 4}
      ],
      "scheduledTime": "2025-11-17T18:00:00Z"
    }
  ]
}
```

---

## Chat Protocols

### Chat Message Format

```json
{
  "type": "chat_message",
  "messageId": "msg-abc123",
  "channel": "match:match-2025-001",
  "sender": {
    "userId": "user-12345",
    "username": "Spectator123",
    "badges": ["verified", "subscriber"],
    "color": "#F59E0B"
  },
  "content": {
    "text": "Amazing play by Faker!",
    "emotes": [
      {
        "id": "emote-pogchamp",
        "name": "PogChamp",
        "position": [20, 28]
      }
    ]
  },
  "timestamp": "2025-01-15T18:30:00.123Z",
  "metadata": {
    "language": "en",
    "filtered": false
  }
}
```

### Moderation Actions

```json
{
  "type": "chat_moderation",
  "action": "timeout",
  "targetUserId": "user-spam123",
  "moderatorId": "mod-abc",
  "duration": 600,
  "reason": "spam",
  "timestamp": "2025-01-15T18:30:15Z"
}
```

### Chat Commands

```json
{
  "type": "chat_command",
  "command": "predict",
  "args": ["team-t1"],
  "userId": "user-12345",
  "channel": "match:match-2025-001"
}
```

---

## State Synchronization

### Snapshot and Delta Updates

Initial snapshot:
```json
{
  "type": "state_snapshot",
  "matchId": "match-2025-001",
  "timestamp": "2025-01-15T18:25:00Z",
  "gameTime": 1500,
  "fullState": {
    "score": {"blue": 10, "red": 7},
    "players": [ /* full player states */ ],
    "objectives": { /* full objectives */ }
  },
  "sequence": 15000
}
```

Delta update:
```json
{
  "type": "state_delta",
  "matchId": "match-2025-001",
  "timestamp": "2025-01-15T18:25:05Z",
  "gameTime": 1505,
  "changes": [
    {
      "path": "score.blue",
      "operation": "set",
      "value": 11
    },
    {
      "path": "players[0].stats.gold",
      "operation": "increment",
      "value": 350
    }
  ],
  "sequence": 15001
}
```

### Conflict Resolution

```json
{
  "type": "state_conflict",
  "matchId": "match-2025-001",
  "conflictId": "conflict-xyz",
  "clientSequence": 15005,
  "serverSequence": 15008,
  "resolution": "server_wins",
  "message": "Client state outdated. Resync required."
}
```

---

## Error Handling

### Protocol Errors

```json
{
  "type": "error",
  "code": "invalid_subscription",
  "message": "Channel 'match:invalid-id' does not exist",
  "timestamp": "2025-01-15T18:00:00Z",
  "recoverable": true,
  "suggestion": "Verify matchId and try again"
}
```

### Connection Issues

```json
{
  "type": "connection_warning",
  "severity": "medium",
  "issue": "high_latency",
  "details": {
    "currentLatency": 850,
    "thresholdLatency": 500,
    "packetsLost": 3
  },
  "recommendation": "Consider reducing video quality"
}
```

---

## Quality of Service

### Latency Requirements

| Event Type | Target Latency | Max Latency |
|------------|----------------|-------------|
| Critical Events | 100ms | 300ms |
| Game State | 200ms | 500ms |
| Chat Messages | 500ms | 1000ms |
| Statistics | 1000ms | 3000ms |

### Bandwidth Optimization

- Binary protocols for high-frequency updates
- Message compression (gzip, brotli)
- Delta updates instead of full snapshots
- Priority queues for critical events
- Rate limiting for non-critical data

---

## Philosophy: 弘益人間

Real-time protocols embody "Broadly Benefiting Humanity" through:

- **Low Barriers:** WebSocket support in all browsers
- **Efficiency:** Minimal bandwidth for mobile users
- **Reliability:** Graceful degradation under poor conditions
- **Accessibility:** Text-based protocols for compatibility
- **Openness:** Fully documented for any implementation

---

## Implementation Guide

### Client-Side Example

```javascript
class WIARealtimeClient {
  constructor(matchId, token) {
    this.matchId = matchId;
    this.token = token;
    this.ws = null;
    this.sequence = 0;
  }

  connect() {
    this.ws = new WebSocket('wss://realtime.platform.com/wia-ind-013/v1');

    this.ws.onopen = () => {
      this.authenticate();
      this.subscribe();
    };

    this.ws.onmessage = (event) => {
      const message = JSON.parse(event.data);
      this.handleMessage(message);
    };
  }

  authenticate() {
    this.send({
      type: 'auth',
      token: this.token
    });
  }

  subscribe() {
    this.send({
      type: 'subscribe',
      channels: [`match:${this.matchId}`]
    });
  }

  handleMessage(message) {
    switch(message.type) {
      case 'game_state':
        this.onGameState(message);
        break;
      case 'game_event':
        this.onGameEvent(message);
        break;
    }
  }

  send(message) {
    this.ws.send(JSON.stringify(message));
  }
}
```

---

## References

- [WebSocket Protocol RFC 6455](https://tools.ietf.org/html/rfc6455)
- [WebRTC Specification](https://webrtc.org/)
- [Server-Sent Events](https://html.spec.whatwg.org/multipage/server-sent-events.html)
- [WIA-IND-013 Phase 4: Integration](./PHASE-4-INTEGRATION.md)

---

**© 2025 SmileStory Inc. / WIA**
**弘익人間 (홍익인간) · Benefit All Humanity**

---

## Annex A — Conformance Tier Matrix

WIA conformance for WIA-IND-013-esports is evaluated across three tiers:

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

- `wia-standards/standards/WIA-IND-013-esports/api/` — TypeScript SDK skeleton
- `wia-standards/standards/WIA-IND-013-esports/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/WIA-IND-013-esports/simulator/` — interactive browser-based simulator for the PHASE protocol

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
