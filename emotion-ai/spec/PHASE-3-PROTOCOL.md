# WIA Emotion AI Standard - Phase 3: Streaming Protocol Specification

> **Version**: 1.0.0
> **Status**: Stable
> **Last Updated**: 2025

---

## 1. Overview

Phase 3 defines the WebSocket-based streaming protocol for real-time emotion analysis. This specification ensures low-latency, continuous emotion data streaming for applications requiring immediate feedback.

### 1.1 Design Principles

- **Low Latency**: Minimize end-to-end delay
- **Reliability**: Handle disconnections gracefully
- **Scalability**: Support high concurrent connections
- **Security**: Encrypted transport with authentication

---

## 2. Connection Management

### 2.1 WebSocket URL

```
wss://stream.{provider}.com/wia/emotion/v1/stream
```

### 2.2 Connection Request

```http
GET /wia/emotion/v1/stream HTTP/1.1
Host: stream.provider.com
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==
Sec-WebSocket-Version: 13
Authorization: Bearer {access_token}
X-WIA-Version: 1.0.0
```

### 2.3 Handshake Process

| Step | Direction | Message |
|------|-----------|---------|
| 1 | Client → Server | WebSocket upgrade request |
| 2 | Server → Client | 101 Switching Protocols |
| 3 | Client → Server | session.start |
| 4 | Server → Client | session.started |
| 5 | Bidirectional | Data streaming begins |

---

## 3. Session Management

### 3.1 Session Start

**Client → Server**:
```json
{
    "type": "session.start",
    "session_id": "client-generated-uuid",
    "timestamp": "2025-01-15T14:30:00.000Z",
    "config": {
        "modalities": ["facial", "voice"],
        "facial_config": {
            "frame_rate": 15,
            "detect_action_units": true,
            "detect_micro_expressions": true,
            "resolution": "640x480"
        },
        "voice_config": {
            "sample_rate": 16000,
            "encoding": "LINEAR16",
            "language": "ko-KR"
        },
        "output_config": {
            "include_dimensional": true,
            "include_raw_scores": false,
            "confidence_threshold": 0.5,
            "emit_interval_ms": 100
        }
    }
}
```

**Server → Client**:
```json
{
    "type": "session.started",
    "session_id": "server-assigned-session-id",
    "timestamp": "2025-01-15T14:30:00.050Z",
    "config_applied": {
        "modalities": ["facial", "voice"],
        "max_frame_rate": 15,
        "session_timeout_seconds": 3600
    },
    "server_info": {
        "version": "1.0.0",
        "region": "ap-northeast-2"
    }
}
```

### 3.2 Session End

**Client → Server**:
```json
{
    "type": "session.end",
    "timestamp": "2025-01-15T15:00:00.000Z",
    "reason": "user_initiated",
    "request_summary": true
}
```

**Server → Client**:
```json
{
    "type": "session.ended",
    "session_id": "session-12345",
    "timestamp": "2025-01-15T15:00:00.100Z",
    "duration_seconds": 1800,
    "summary": {
        "total_frames_processed": 27000,
        "total_events_emitted": 18000,
        "dominant_emotion": "neutral",
        "average_engagement": 0.68,
        "average_valence": 0.35
    }
}
```

---

## 4. Message Types

### 4.1 Message Type Reference

| Type | Direction | Description |
|------|-----------|-------------|
| session.start | C → S | Initialize session |
| session.started | S → C | Session confirmation |
| data.frame | C → S | Video/audio frame |
| emotion.event | S → C | Emotion analysis result |
| emotion.aggregate | S → C | Aggregated summary |
| heartbeat.ping | Bidirectional | Keep-alive ping |
| heartbeat.pong | Bidirectional | Ping response |
| session.end | C → S | End session request |
| session.ended | S → C | Session end confirmation |
| error | S → C | Error notification |
| config.update | C → S | Update configuration |
| config.updated | S → C | Configuration update confirmation |

### 4.2 Data Frame Messages

**Video Frame**:
```json
{
    "type": "data.frame",
    "modality": "facial",
    "timestamp": "2025-01-15T14:30:01.000Z",
    "sequence": 150,
    "data": {
        "frame": "{base64_encoded_frame}",
        "format": "jpeg",
        "resolution": "640x480"
    }
}
```

**Audio Chunk**:
```json
{
    "type": "data.frame",
    "modality": "voice",
    "timestamp": "2025-01-15T14:30:01.000Z",
    "sequence": 450,
    "data": {
        "audio": "{base64_encoded_audio_chunk}",
        "encoding": "LINEAR16",
        "duration_ms": 100
    }
}
```

### 4.3 Emotion Events

**Emotion Event**:
```json
{
    "type": "emotion.event",
    "event_id": "evt-12345-abcde",
    "timestamp": "2025-01-15T14:30:01.100Z",
    "sequence": 150,
    "latency_ms": 85,
    "emotions": [
        {
            "category": "happiness",
            "intensity": 0.78,
            "confidence": 0.91
        }
    ],
    "action_units": [
        {
            "au": "AU6",
            "intensity": 0.72,
            "confidence": 0.89
        },
        {
            "au": "AU12",
            "intensity": 0.81,
            "confidence": 0.93
        }
    ],
    "dimensional": {
        "valence": 0.72,
        "arousal": 0.55
    },
    "source_modality": "facial",
    "face_detected": true,
    "micro_expression": false
}
```

**Aggregate Event** (every 5 seconds):
```json
{
    "type": "emotion.aggregate",
    "timestamp": "2025-01-15T14:30:05.000Z",
    "window_start": "2025-01-15T14:30:00.000Z",
    "window_end": "2025-01-15T14:30:05.000Z",
    "frame_count": 75,
    "aggregate": {
        "dominant_emotion": "happiness",
        "emotion_distribution": {
            "happiness": 0.65,
            "neutral": 0.25,
            "surprise": 0.10
        },
        "average_valence": 0.68,
        "average_arousal": 0.52,
        "emotional_stability": 0.85,
        "engagement_score": 0.72
    },
    "modality_quality": {
        "facial": {
            "average_confidence": 0.88,
            "frames_processed": 75,
            "frames_dropped": 0
        },
        "voice": {
            "average_confidence": 0.82,
            "segments_processed": 50
        }
    }
}
```

---

## 5. Frame Rate and Latency

### 5.1 Frame Rate Recommendations

| Modality | Minimum | Recommended | Maximum |
|----------|---------|-------------|---------|
| Facial Analysis | 5 fps | 15 fps | 30 fps |
| Micro-expressions | 25 fps | 30 fps | 60 fps |
| Voice Analysis | 100ms chunks | 100ms chunks | 50ms chunks |

### 5.2 Latency Requirements

| Use Case | Max Latency | Target Latency |
|----------|-------------|----------------|
| Real-time video call | 200ms | 100ms |
| Gaming/Interaction | 100ms | 50ms |
| Driver monitoring | 150ms | 75ms |
| Customer service | 500ms | 250ms |
| Recorded analysis | N/A | Non-real-time |

### 5.3 Quality vs Latency Modes

**Low Latency Mode**:
```json
{
    "config": {
        "mode": "low_latency",
        "facial_config": {
            "frame_rate": 15,
            "detect_action_units": false,
            "detect_micro_expressions": false,
            "lightweight_model": true
        },
        "output_config": {
            "emit_interval_ms": 50
        }
    }
}
```

**High Quality Mode**:
```json
{
    "config": {
        "mode": "high_quality",
        "facial_config": {
            "frame_rate": 30,
            "detect_action_units": true,
            "detect_micro_expressions": true,
            "high_precision_model": true
        },
        "output_config": {
            "emit_interval_ms": 200
        }
    }
}
```

---

## 6. Heartbeat Protocol

### 6.1 Ping/Pong Messages

**Client → Server**:
```json
{
    "type": "heartbeat.ping",
    "timestamp": "2025-01-15T14:30:30.000Z",
    "client_sequence": 500
}
```

**Server → Client**:
```json
{
    "type": "heartbeat.pong",
    "timestamp": "2025-01-15T14:30:30.050Z",
    "client_sequence": 500,
    "server_sequence": 495,
    "latency_ms": 50
}
```

### 6.2 Heartbeat Configuration

| Parameter | Default | Description |
|-----------|---------|-------------|
| interval | 30 seconds | Ping frequency |
| timeout | 60 seconds | Max time without response |
| max_missed | 2 | Missed pings before disconnect |

---

## 7. Security

### 7.1 Transport Security

| Requirement | Specification |
|-------------|---------------|
| Encryption | TLS 1.3 required (minimum TLS 1.2) |
| WebSocket | wss:// protocol only |
| Certificate | Trusted CA issued |
| Cipher Suites | AES-256-GCM, CHACHA20-POLY1305 |

### 7.2 Authentication

**JWT Token Structure**:
```json
{
    "header": {
        "alg": "RS256",
        "typ": "JWT"
    },
    "payload": {
        "sub": "user-12345",
        "iss": "https://auth.provider.com",
        "aud": "wia-emotion-stream",
        "iat": 1705329000,
        "exp": 1705332600,
        "scope": ["stream:read", "stream:write"],
        "session_limit": 5,
        "rate_limit": {
            "frames_per_second": 30,
            "max_concurrent_sessions": 3
        }
    }
}
```

### 7.3 Session Security

```json
{
    "security": {
        "encryption": {
            "enable_payload_encryption": true,
            "algorithm": "AES-256-GCM"
        },
        "consent": {
            "user_consent_token": "{signed_consent_token}",
            "consent_scope": ["emotion_analysis", "data_storage"],
            "consent_timestamp": "2025-01-15T14:25:00.000Z"
        },
        "data_handling": {
            "store_raw_data": false,
            "anonymize_results": true,
            "retention_period": "session_only"
        }
    }
}
```

---

## 8. Error Handling

### 8.1 Error Message Format

```json
{
    "type": "error",
    "error_code": "PROCESSING_FAILED",
    "message": "Frame processing failed",
    "timestamp": "2025-01-15T14:30:05.000Z",
    "details": {
        "sequence": 155,
        "modality": "facial",
        "reason": "face_not_detected"
    },
    "severity": "warning",
    "recoverable": true,
    "action": "skip_frame"
}
```

### 8.2 Error Codes

| Error Code | Severity | Description | Action |
|------------|----------|-------------|--------|
| AUTH_EXPIRED | critical | Token expired | Re-authenticate |
| SESSION_TIMEOUT | critical | Session timed out | Restart session |
| RATE_LIMITED | warning | Rate limit exceeded | Reduce frame rate |
| PROCESSING_FAILED | warning | Frame processing failed | Skip frame |
| MODALITY_UNAVAILABLE | warning | Modality unavailable | Use other modality |
| QUALITY_LOW | info | Low input quality | Improve quality |
| SERVER_OVERLOAD | critical | Server overloaded | Wait and reconnect |

### 8.3 Reconnection Protocol

```json
{
    "reconnection_policy": {
        "max_attempts": 5,
        "backoff_strategy": "exponential",
        "initial_delay_ms": 1000,
        "max_delay_ms": 30000,
        "backoff_multiplier": 2.0,
        "jitter": 0.1
    },
    "session_resumption": {
        "enabled": true,
        "resume_token": "{session_resume_token}",
        "resume_from_sequence": 155,
        "max_gap_seconds": 30
    }
}
```

---

## 9. Connection States

### 9.1 State Diagram

```
┌────────────┐
│ connecting │◄────────────────────────────┐
└─────┬──────┘                             │
      │                                    │
      ▼                                    │
┌────────────────┐                         │
│ authenticating │                         │
└───────┬────────┘                         │
        │                                  │
        ▼                                  │
┌─────────────┐                            │
│ initializing│                            │
└──────┬──────┘                            │
       │                                   │
       ▼                                   │
┌───────────┐    pause     ┌────────┐     │
│ streaming │◄────────────►│ paused │     │
└─────┬─────┘    resume    └────────┘     │
      │                                    │
      │ error                              │
      ▼                                    │
┌──────────────┐                           │
│ reconnecting │───────────────────────────┘
└──────┬───────┘
       │ max_attempts
       ▼
┌─────────┐
│ closing │
└────┬────┘
     │
     ▼
┌────────┐
│ closed │
└────────┘
```

### 9.2 State Descriptions

| State | Description |
|-------|-------------|
| connecting | WebSocket connection in progress |
| authenticating | Authentication in progress |
| initializing | Session initialization |
| streaming | Active data streaming |
| paused | Streaming temporarily paused |
| reconnecting | Attempting to reconnect |
| closing | Session ending |
| closed | Connection terminated |

---

## 10. Binary Protocol (Optional)

### 10.1 Binary Frame Format

For high-performance scenarios, binary frames can be used:

```
┌──────────────────────────────────────────────────────┐
│ Byte 0-1  │ Byte 2-5  │ Byte 6-9  │ Byte 10+        │
├───────────┼───────────┼───────────┼─────────────────┤
│ Frame     │ Sequence  │ Timestamp │ Payload         │
│ Type (2B) │ Number(4B)│ (4B)      │ (Variable)      │
└──────────────────────────────────────────────────────┘
```

### 10.2 Frame Types (Binary)

| Type Code | Description |
|-----------|-------------|
| 0x0001 | Video frame |
| 0x0002 | Audio chunk |
| 0x0003 | Emotion event |
| 0x0004 | Heartbeat |
| 0x0005 | Control message |

---

## 11. SDK Reference

### 11.1 JavaScript/TypeScript

```typescript
import { EmotionStream } from '@wia/emotion-ai';

const stream = new EmotionStream({
    url: 'wss://stream.provider.com/wia/emotion/v1/stream',
    token: 'your-access-token',
    modalities: ['facial', 'voice']
});

stream.on('emotion', (event) => {
    console.log('Emotion:', event.emotions[0].category);
});

stream.on('error', (error) => {
    console.error('Error:', error.message);
});

await stream.start();
stream.sendFrame(videoFrame);
```

### 11.2 Python

```python
from wia_emotion_ai import EmotionStream

stream = EmotionStream(
    url='wss://stream.provider.com/wia/emotion/v1/stream',
    token='your-access-token',
    modalities=['facial', 'voice']
)

@stream.on('emotion')
def handle_emotion(event):
    print(f"Emotion: {event['emotions'][0]['category']}")

stream.start()
stream.send_frame(video_frame)
```

---

**홍익인간 (弘益人間)**: "Benefit all humanity"

The WIA Emotion AI Standard belongs to humanity. Free forever.

---

**Copyright 2025 SmileStory Inc. / WIA**
MIT License
