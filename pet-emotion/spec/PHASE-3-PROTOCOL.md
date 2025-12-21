# WIA Pet Emotion Communication Protocol
## Phase 3 Specification

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-12-18
**Primary Color**: #F59E0B (Amber)

---

## Table of Contents

1. [Overview](#overview)
2. [Terminology](#terminology)
3. [Protocol Architecture](#protocol-architecture)
4. [Message Format](#message-format)
5. [Message Types](#message-types)
6. [Connection Management](#connection-management)
7. [Streaming Protocol](#streaming-protocol)
8. [Security](#security)
9. [Transport Layers](#transport-layers)
10. [Error Handling](#error-handling)

---

## 1. Overview

### 1.1 Purpose

This document defines the communication protocol for WIA Pet Emotion Standard Phase 3, enabling standardized communication between emotion detection systems, sensors, applications, and cloud services.

**Core Objectives**:
- Real-time emotion data streaming
- Multi-device synchronization
- Cloud-edge coordination
- Low-latency communication
- Secure data transmission

### 1.2 Scope

| Component | Description |
|-----------|-------------|
| **Message Format** | JSON-based protocol messages |
| **Transport** | WebSocket, MQTT, HTTP, BLE |
| **Security** | TLS, authentication, encryption |
| **Streaming** | Real-time emotion event streaming |
| **Sync** | Multi-device state synchronization |

### 1.3 Related Documents

| Document | Description |
|----------|-------------|
| PHASE-1-DATA-FORMAT.md | Emotion data structures |
| PHASE-2-API-INTERFACE.md | Programming interfaces |
| PHASE-4-INTEGRATION.md | Ecosystem integration |

---

## 2. Terminology

| Term | Definition |
|------|------------|
| **Client** | Device/app requesting emotion detection |
| **Server** | Service providing emotion detection |
| **Edge Device** | Local processing device (camera, wearable) |
| **Cloud Service** | Remote processing/storage service |
| **Message** | Protocol data unit |
| **Stream** | Continuous message flow |
| **Session** | Connected client-server interaction |

---

## 3. Protocol Architecture

### 3.1 System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                        Cloud Layer                           │
│            (Storage, Analytics, ML Training)                 │
└────────────────────┬────────────────────────────────────────┘
                     │ HTTPS/MQTT
                     ▼
┌─────────────────────────────────────────────────────────────┐
│                     Gateway/Hub Layer                        │
│           (Aggregation, Routing, Preprocessing)              │
└────────────────────┬────────────────────────────────────────┘
                     │ WebSocket/MQTT
                     ▼
┌─────────────────────────────────────────────────────────────┐
│                      Edge Layer                              │
│        (Cameras, Wearables, Local Processing)                │
└────────────────────┬────────────────────────────────────────┘
                     │ BLE/USB/Local
                     ▼
┌─────────────────────────────────────────────────────────────┐
│                    Sensor Layer                              │
│              (Accelerometer, Heart Rate, Mic)                │
└─────────────────────────────────────────────────────────────┘
```

### 3.2 Protocol Stack

```
┌─────────────────────────────────────────┐
│         Application Layer               │
│     (Emotion Detection Logic)           │
├─────────────────────────────────────────┤
│         Protocol Layer                  │
│   (WIA Pet Emotion Messages)            │
├─────────────────────────────────────────┤
│        Transport Layer                  │
│  (WebSocket / MQTT / HTTP / BLE)        │
├─────────────────────────────────────────┤
│         Security Layer                  │
│      (TLS / Encryption)                 │
├─────────────────────────────────────────┤
│         Network Layer                   │
│         (TCP/IP / BLE)                  │
└─────────────────────────────────────────┘
```

---

## 4. Message Format

### 4.1 Base Message Structure

All WIA Pet Emotion protocol messages follow this structure:

```json
{
  "protocol": "wia-pet-emotion",
  "version": "1.0.0",
  "messageId": "uuid-v4-string",
  "timestamp": 1702915200000,
  "type": "message_type",
  "payload": {},
  "metadata": {}
}
```

### 4.2 Field Definitions

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `protocol` | string | Yes | Protocol identifier ("wia-pet-emotion") |
| `version` | string | Yes | Protocol version (SemVer) |
| `messageId` | string | Yes | Unique message ID (UUID v4) |
| `timestamp` | number | Yes | Unix timestamp (milliseconds) |
| `type` | string | Yes | Message type identifier |
| `payload` | object | Yes | Message-specific data |
| `metadata` | object | No | Optional metadata |

### 4.3 JSON Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wiastandards.com/schemas/pet-emotion-protocol/v1/message.schema.json",
  "title": "WIA Pet Emotion Protocol Message",
  "type": "object",
  "required": ["protocol", "version", "messageId", "timestamp", "type", "payload"],
  "properties": {
    "protocol": {
      "type": "string",
      "const": "wia-pet-emotion"
    },
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$"
    },
    "messageId": {
      "type": "string",
      "format": "uuid"
    },
    "timestamp": {
      "type": "integer",
      "minimum": 0
    },
    "type": {
      "type": "string",
      "enum": [
        "connect", "connect_ack", "disconnect", "disconnect_ack",
        "emotion_data", "emotion_request", "emotion_response",
        "sensor_data", "sensor_command", "sensor_status",
        "stream_start", "stream_stop", "stream_data",
        "sync_request", "sync_response",
        "alert", "event",
        "error", "ping", "pong"
      ]
    },
    "payload": {
      "type": "object"
    },
    "metadata": {
      "type": "object",
      "properties": {
        "source": { "type": "string" },
        "destination": { "type": "string" },
        "priority": { "type": "integer", "minimum": 0, "maximum": 9 },
        "correlation_id": { "type": "string" }
      }
    }
  }
}
```

---

## 5. Message Types

### 5.1 Message Type Overview

| Category | Message Types | Direction |
|----------|---------------|-----------|
| Connection | `connect`, `connect_ack`, `disconnect`, `disconnect_ack` | Both |
| Emotion Data | `emotion_data`, `emotion_request`, `emotion_response` | Both |
| Sensor | `sensor_data`, `sensor_command`, `sensor_status` | Both |
| Streaming | `stream_start`, `stream_stop`, `stream_data` | Both |
| Sync | `sync_request`, `sync_response` | Both |
| Alerts | `alert`, `event` | Server → Client |
| Control | `ping`, `pong`, `error` | Both |

### 5.2 Connection Messages

#### 5.2.1 connect

Establish a connection to the emotion detection service.

```json
{
  "protocol": "wia-pet-emotion",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": 1702915200000,
  "type": "connect",
  "payload": {
    "clientId": "mobile-app-12345",
    "clientType": "mobile_app",
    "clientVersion": "2.1.0",
    "capabilities": {
      "sensors": ["camera", "microphone"],
      "models": ["emotion-detect-v2"],
      "streaming": true,
      "offline_mode": true
    },
    "pets": [
      {
        "petId": "PET-001",
        "species": "dog",
        "name": "Max"
      }
    ],
    "preferences": {
      "detection_frequency": 30,
      "data_compression": true,
      "alert_enabled": true
    }
  },
  "metadata": {
    "source": "mobile-app",
    "priority": 5
  }
}
```

**Payload Fields**:

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `clientId` | string | Yes | Unique client identifier |
| `clientType` | string | Yes | Client type (mobile_app, web_app, edge_device) |
| `clientVersion` | string | No | Client software version |
| `capabilities` | object | Yes | Client capabilities |
| `pets` | array | Yes | List of pets to monitor |
| `preferences` | object | No | Connection preferences |

#### 5.2.2 connect_ack

Connection acknowledgment from server.

```json
{
  "protocol": "wia-pet-emotion",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440001",
  "timestamp": 1702915200050,
  "type": "connect_ack",
  "payload": {
    "success": true,
    "sessionId": "SESSION-abc123",
    "serverId": "emotion-server-01",
    "serverVersion": "1.5.2",
    "capabilities": {
      "max_pets": 10,
      "streaming": true,
      "models_available": ["emotion-detect-v2", "emotion-detect-v3"],
      "cloud_storage": true
    },
    "config": {
      "heartbeat_interval": 30000,
      "max_message_size": 1048576,
      "compression_enabled": true
    },
    "session_info": {
      "expires_at": 1702958400000,
      "renewable": true
    }
  }
}
```

#### 5.2.3 disconnect

Graceful connection termination.

```json
{
  "protocol": "wia-pet-emotion",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440010",
  "timestamp": 1702918800000,
  "type": "disconnect",
  "payload": {
    "reason": "user_logout",
    "message": "User logged out",
    "reconnect_allowed": true
  }
}
```

### 5.3 Emotion Data Messages

#### 5.3.1 emotion_request

Request emotion detection.

```json
{
  "protocol": "wia-pet-emotion",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440020",
  "timestamp": 1702915210000,
  "type": "emotion_request",
  "payload": {
    "petId": "PET-001",
    "requestId": "REQ-001",
    "inputs": {
      "image": {
        "encoding": "base64",
        "format": "jpeg",
        "data": "base64-encoded-image-data",
        "width": 1920,
        "height": 1080
      },
      "audio": {
        "encoding": "base64",
        "format": "wav",
        "data": "base64-encoded-audio-data",
        "sample_rate": 44100,
        "channels": 1,
        "duration": 1000
      },
      "sensors": {
        "heart_rate": 95,
        "activity_level": 0.75,
        "temperature": 38.5
      },
      "context": {
        "location": "backyard",
        "social_setting": "alone",
        "time_of_day": "afternoon"
      }
    },
    "options": {
      "model": "emotion-detect-v2",
      "return_raw_predictions": true,
      "include_attention_map": true,
      "min_confidence": 0.7
    }
  }
}
```

#### 5.3.2 emotion_response

Response with detected emotion.

```json
{
  "protocol": "wia-pet-emotion",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440021",
  "timestamp": 1702915210145,
  "type": "emotion_response",
  "payload": {
    "petId": "PET-001",
    "requestId": "REQ-001",
    "success": true,
    "result": {
      "emotion": "happy",
      "intensity": {
        "intensity": 0.85,
        "level": "high",
        "confidence": 0.92
      },
      "dimensions": {
        "valence": 0.8,
        "arousal": 0.7,
        "dominance": 0.6
      },
      "confidence": 0.92,
      "predictions": [
        { "emotion": "happy", "probability": 0.89, "confidence": 0.92 },
        { "emotion": "excited", "probability": 0.08, "confidence": 0.85 },
        { "emotion": "playful", "probability": 0.03, "confidence": 0.72 }
      ],
      "factors": [
        { "source": "visual", "contribution": 0.65, "confidence": 0.91 },
        { "source": "audio", "contribution": 0.25, "confidence": 0.88 },
        { "source": "sensor", "contribution": 0.10, "confidence": 0.75 }
      ],
      "model": {
        "id": "emotion-detect-v2",
        "version": "2.1.0",
        "processing_time": 145
      }
    }
  },
  "metadata": {
    "correlation_id": "REQ-001"
  }
}
```

#### 5.3.3 emotion_data

Unsolicited emotion state update (push notification).

```json
{
  "protocol": "wia-pet-emotion",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440030",
  "timestamp": 1702915220000,
  "type": "emotion_data",
  "payload": {
    "petId": "PET-001",
    "emotion_state": {
      "emotion": "anxious",
      "intensity": 0.72,
      "confidence": 0.88,
      "dimensions": {
        "valence": -0.6,
        "arousal": 0.85,
        "dominance": 0.3
      }
    },
    "change_detected": true,
    "previous_emotion": "calm",
    "transition_duration": 5000,
    "trigger": {
      "type": "environmental",
      "description": "Loud noise detected",
      "confidence": 0.75
    }
  }
}
```

### 5.4 Sensor Messages

#### 5.4.1 sensor_data

Raw sensor data transmission.

```json
{
  "protocol": "wia-pet-emotion",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440040",
  "timestamp": 1702915230000,
  "type": "sensor_data",
  "payload": {
    "petId": "PET-001",
    "sensorId": "COLLAR-001",
    "sensorType": "wearable",
    "data": {
      "heart_rate": {
        "bpm": 95,
        "variability": 45,
        "timestamp": 1702915230000
      },
      "accelerometer": {
        "x": 0.05,
        "y": 0.12,
        "z": 9.81,
        "magnitude": 9.82,
        "timestamp": 1702915230000
      },
      "temperature": {
        "value": 38.5,
        "unit": "celsius",
        "timestamp": 1702915230000
      },
      "gps": {
        "latitude": 37.7749,
        "longitude": -122.4194,
        "accuracy": 5.0,
        "timestamp": 1702915230000
      }
    },
    "battery_level": 78,
    "signal_strength": -65
  }
}
```

#### 5.4.2 sensor_command

Command to control sensor.

```json
{
  "protocol": "wia-pet-emotion",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440041",
  "timestamp": 1702915240000,
  "type": "sensor_command",
  "payload": {
    "petId": "PET-001",
    "sensorId": "COLLAR-001",
    "command": "set_sampling_rate",
    "parameters": {
      "rate": 60,
      "sensors": ["heart_rate", "accelerometer"]
    },
    "execute_at": "immediate"
  }
}
```

#### 5.4.3 sensor_status

Sensor status update.

```json
{
  "protocol": "wia-pet-emotion",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440042",
  "timestamp": 1702915250000,
  "type": "sensor_status",
  "payload": {
    "petId": "PET-001",
    "sensorId": "COLLAR-001",
    "status": "active",
    "connection_quality": 0.95,
    "battery_level": 78,
    "last_data_received": 1702915249500,
    "errors": [],
    "warnings": ["Battery below 80%"]
  }
}
```

### 5.5 Streaming Messages

#### 5.5.1 stream_start

Start continuous emotion streaming.

```json
{
  "protocol": "wia-pet-emotion",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440050",
  "timestamp": 1702915260000,
  "type": "stream_start",
  "payload": {
    "petId": "PET-001",
    "streamId": "STREAM-001",
    "stream_type": "emotion_continuous",
    "options": {
      "frequency": 30,
      "sources": ["camera", "wearable"],
      "min_confidence": 0.7,
      "compression": true,
      "buffer_size": 100
    },
    "filters": {
      "emotions": null,
      "min_intensity": 0.3
    }
  }
}
```

#### 5.5.2 stream_data

Streaming emotion data packet.

```json
{
  "protocol": "wia-pet-emotion",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440051",
  "timestamp": 1702915260033,
  "type": "stream_data",
  "payload": {
    "streamId": "STREAM-001",
    "sequence": 1,
    "petId": "PET-001",
    "data": {
      "emotion": "happy",
      "intensity": 0.85,
      "confidence": 0.92,
      "valence": 0.8,
      "arousal": 0.7
    },
    "delta": {
      "emotion_changed": false,
      "intensity_change": 0.02
    }
  }
}
```

#### 5.5.3 stream_stop

Stop emotion streaming.

```json
{
  "protocol": "wia-pet-emotion",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440052",
  "timestamp": 1702915320000,
  "type": "stream_stop",
  "payload": {
    "streamId": "STREAM-001",
    "reason": "user_request",
    "statistics": {
      "total_samples": 1800,
      "duration": 60000,
      "average_confidence": 0.88,
      "emotion_changes": 3
    }
  }
}
```

### 5.6 Synchronization Messages

#### 5.6.1 sync_request

Request state synchronization.

```json
{
  "protocol": "wia-pet-emotion",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440060",
  "timestamp": 1702915330000,
  "type": "sync_request",
  "payload": {
    "clientId": "mobile-app-12345",
    "petIds": ["PET-001", "PET-002"],
    "sync_items": ["current_emotion", "daily_summary", "alerts"],
    "since": 1702828800000,
    "include_deleted": false
  }
}
```

#### 5.6.2 sync_response

State synchronization response.

```json
{
  "protocol": "wia-pet-emotion",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440061",
  "timestamp": 1702915330150,
  "type": "sync_response",
  "payload": {
    "success": true,
    "sync_timestamp": 1702915330150,
    "pets": [
      {
        "petId": "PET-001",
        "current_emotion": {
          "emotion": "happy",
          "intensity": 0.85,
          "timestamp": 1702915325000
        },
        "daily_summary": {
          "date": "2025-12-18",
          "dominant_emotion": "content",
          "average_valence": 0.65
        },
        "alerts": []
      }
    ],
    "conflicts": [],
    "next_sync_token": "SYNC-TOKEN-12345"
  }
}
```

### 5.7 Alert and Event Messages

#### 5.7.1 alert

High-priority alert notification.

```json
{
  "protocol": "wia-pet-emotion",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440070",
  "timestamp": 1702915340000,
  "type": "alert",
  "payload": {
    "alertId": "ALERT-001",
    "petId": "PET-001",
    "alert_type": "high_stress",
    "severity": "high",
    "description": "Pet showing signs of high stress for extended period",
    "details": {
      "stress_level": 0.88,
      "duration": 180000,
      "current_emotion": "anxious",
      "triggers": ["loud_noise", "unfamiliar_person"]
    },
    "recommended_action": "Check environment and provide comfort",
    "expires_at": 1702918940000,
    "requires_acknowledgment": true
  },
  "metadata": {
    "priority": 9
  }
}
```

#### 5.7.2 event

General event notification.

```json
{
  "protocol": "wia-pet-emotion",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440071",
  "timestamp": 1702915350000,
  "type": "event",
  "payload": {
    "eventId": "EVENT-001",
    "event_type": "social_interaction",
    "petIds": ["PET-001", "PET-002"],
    "data": {
      "interaction_type": "play",
      "duration": 120000,
      "quality": {
        "valence": "positive",
        "intensity": 0.87,
        "reciprocity": 0.92
      }
    },
    "timestamp": 1702915350000
  }
}
```

### 5.8 Control Messages

#### 5.8.1 ping / pong

Connection keep-alive.

```json
{
  "protocol": "wia-pet-emotion",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440080",
  "timestamp": 1702915360000,
  "type": "ping",
  "payload": {
    "echo_data": "test123"
  }
}
```

```json
{
  "protocol": "wia-pet-emotion",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440081",
  "timestamp": 1702915360005,
  "type": "pong",
  "payload": {
    "echo_data": "test123",
    "server_time": 1702915360005
  }
}
```

#### 5.8.2 error

Error notification.

```json
{
  "protocol": "wia-pet-emotion",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440082",
  "timestamp": 1702915370000,
  "type": "error",
  "payload": {
    "error_code": "DETECTION_FAILED",
    "error_message": "Insufficient image quality for emotion detection",
    "details": {
      "image_quality_score": 0.35,
      "min_required": 0.60,
      "issues": ["low_resolution", "poor_lighting"]
    },
    "recoverable": true,
    "suggested_action": "Improve lighting and try again",
    "reference_message_id": "550e8400-e29b-41d4-a716-446655440020"
  }
}
```

---

## 6. Connection Management

### 6.1 Connection Lifecycle

```
Client                                  Server
  │                                       │
  ├────── connect ──────────────────────►│
  │                                       │
  │◄────── connect_ack ───────────────────┤
  │                                       │
  │                                       │
  ├────── emotion_request ──────────────►│
  │                                       │
  │◄────── emotion_response ──────────────┤
  │                                       │
  │                                       │
  ├────── ping ──────────────────────────►│
  │                                       │
  │◄────── pong ───────────────────────────┤
  │                                       │
  │                                       │
  ├────── disconnect ────────────────────►│
  │                                       │
  │◄────── disconnect_ack ─────────────────┤
  │                                       │
```

### 6.2 Heartbeat Mechanism

```typescript
interface HeartbeatConfig {
  enabled: boolean;
  interval: number;          // milliseconds
  timeout: number;           // milliseconds
  max_missed: number;        // consecutive misses before disconnect
}

// Default configuration
const DEFAULT_HEARTBEAT: HeartbeatConfig = {
  enabled: true,
  interval: 30000,          // 30 seconds
  timeout: 5000,            // 5 seconds
  max_missed: 3             // 3 consecutive misses
};
```

### 6.3 Reconnection Strategy

```typescript
interface ReconnectionConfig {
  enabled: boolean;
  initial_delay: number;     // milliseconds
  max_delay: number;         // milliseconds
  backoff_multiplier: number;
  max_attempts: number;
  jitter: boolean;
}

// Exponential backoff with jitter
function calculateReconnectDelay(attempt: number, config: ReconnectionConfig): number {
  let delay = Math.min(
    config.initial_delay * Math.pow(config.backoff_multiplier, attempt),
    config.max_delay
  );

  if (config.jitter) {
    delay = delay * (0.5 + Math.random() * 0.5);
  }

  return delay;
}
```

---

## 7. Streaming Protocol

### 7.1 Stream Types

| Stream Type | Description | Frequency | Payload |
|-------------|-------------|-----------|---------|
| `emotion_continuous` | Continuous emotion state | 1-60 Hz | EmotionState |
| `sensor_raw` | Raw sensor data | Variable | SensorData |
| `sensor_processed` | Processed sensor data | 1-30 Hz | ProcessedSensorData |
| `event_stream` | Event notifications | Event-driven | Event |
| `alert_stream` | Alert notifications | Event-driven | Alert |

### 7.2 Stream Quality of Service

```typescript
enum StreamQoS {
  AT_MOST_ONCE = 0,      // Fire and forget
  AT_LEAST_ONCE = 1,     // Guaranteed delivery, may duplicate
  EXACTLY_ONCE = 2       // Guaranteed delivery, no duplicates
}

interface StreamOptions {
  qos: StreamQoS;
  buffer_size: number;
  compression: boolean;
  encryption: boolean;
  priority: number;       // 0-9
}
```

### 7.3 Flow Control

```typescript
interface FlowControl {
  enabled: boolean;
  window_size: number;    // Maximum unacknowledged messages
  rate_limit: number;     // Messages per second
  burst_size: number;     // Maximum burst size
}

// Flow control message
interface FlowControlMessage {
  type: 'flow_control';
  action: 'pause' | 'resume' | 'adjust';
  parameters?: {
    new_rate?: number;
    window_size?: number;
  };
}
```

### 7.4 Stream Buffering

```typescript
interface BufferStrategy {
  type: 'ring' | 'fifo' | 'priority';
  size: number;
  overflow_policy: 'drop_oldest' | 'drop_newest' | 'block';
  persistence: 'memory' | 'disk' | 'hybrid';
}
```

---

## 8. Security

### 8.1 Authentication

#### 8.1.1 API Key Authentication

```json
{
  "protocol": "wia-pet-emotion",
  "version": "1.0.0",
  "messageId": "...",
  "timestamp": 1702915400000,
  "type": "connect",
  "payload": {
    "clientId": "mobile-app-12345",
    "auth": {
      "method": "api_key",
      "api_key": "wpe_1234567890abcdef",
      "api_secret": "secret_abcdefghijklmnop"
    }
  }
}
```

#### 8.1.2 OAuth 2.0 Authentication

```json
{
  "protocol": "wia-pet-emotion",
  "version": "1.0.0",
  "messageId": "...",
  "timestamp": 1702915400000,
  "type": "connect",
  "payload": {
    "clientId": "mobile-app-12345",
    "auth": {
      "method": "oauth2",
      "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
      "token_type": "Bearer",
      "expires_in": 3600
    }
  }
}
```

#### 8.1.3 Device Certificate Authentication

```json
{
  "protocol": "wia-pet-emotion",
  "version": "1.0.0",
  "messageId": "...",
  "timestamp": 1702915400000,
  "type": "connect",
  "payload": {
    "clientId": "edge-device-001",
    "auth": {
      "method": "certificate",
      "device_id": "DEV-001",
      "certificate_thumbprint": "SHA256:1234567890abcdef..."
    }
  }
}
```

### 8.2 Encryption

#### 8.2.1 Transport Layer Security

```
All connections MUST use TLS 1.2 or higher:
- WebSocket: wss://
- HTTPS: https://
- MQTT: mqtts://

Recommended cipher suites:
- TLS_ECDHE_RSA_WITH_AES_256_GCM_SHA384
- TLS_ECDHE_RSA_WITH_AES_128_GCM_SHA256
```

#### 8.2.2 End-to-End Encryption

```typescript
interface E2EEncryption {
  enabled: boolean;
  algorithm: 'AES-256-GCM' | 'ChaCha20-Poly1305';
  key_exchange: 'ECDH' | 'RSA';

  // Encrypted payload
  encrypted_data: string;    // Base64 encoded
  iv: string;                // Initialization vector
  auth_tag: string;          // Authentication tag
  key_id: string;            // Key identifier
}
```

### 8.3 Message Signing

```typescript
interface MessageSignature {
  algorithm: 'HMAC-SHA256' | 'RSA-SHA256' | 'ECDSA-SHA256';
  signature: string;         // Base64 encoded signature
  signing_key_id: string;    // Key identifier
  signed_fields: string[];   // Fields included in signature
}

// Add signature to metadata
{
  "metadata": {
    "signature": {
      "algorithm": "HMAC-SHA256",
      "signature": "base64-encoded-signature",
      "signing_key_id": "KEY-001",
      "signed_fields": ["messageId", "timestamp", "type", "payload"]
    }
  }
}
```

---

## 9. Transport Layers

### 9.1 WebSocket Transport

```typescript
// WebSocket endpoint
const WS_ENDPOINT = 'wss://api.wia-pet-emotion.com/v1/ws';

// Connection
const ws = new WebSocket(WS_ENDPOINT);

ws.on('open', () => {
  // Send connect message
  ws.send(JSON.stringify(connectMessage));
});

ws.on('message', (data) => {
  const message = JSON.parse(data);
  handleMessage(message);
});

ws.on('close', (code, reason) => {
  console.log('Connection closed:', code, reason);
  attemptReconnect();
});

// Heartbeat
setInterval(() => {
  ws.send(JSON.stringify(pingMessage));
}, 30000);
```

### 9.2 MQTT Transport

```typescript
// MQTT broker
const MQTT_BROKER = 'mqtts://mqtt.wia-pet-emotion.com:8883';

// Topics
const TOPICS = {
  // Publishing
  emotion_request: `pet-emotion/pet/{petId}/request`,
  sensor_data: `pet-emotion/pet/{petId}/sensor`,

  // Subscribing
  emotion_response: `pet-emotion/pet/{petId}/response`,
  emotion_stream: `pet-emotion/pet/{petId}/stream`,
  alerts: `pet-emotion/pet/{petId}/alerts`,
  events: `pet-emotion/pet/{petId}/events`
};

// Connect
const client = mqtt.connect(MQTT_BROKER, {
  clientId: 'mobile-app-12345',
  username: 'user',
  password: 'pass',
  keepalive: 60,
  clean: true,
  reconnectPeriod: 1000
});

// Subscribe
client.subscribe(TOPICS.emotion_response);
client.subscribe(TOPICS.alerts);

// Publish
client.publish(
  TOPICS.emotion_request,
  JSON.stringify(emotionRequestMessage),
  { qos: 1, retain: false }
);
```

### 9.3 HTTP/REST Transport

```typescript
// Base URL
const API_BASE = 'https://api.wia-pet-emotion.com/v1';

// Request emotion detection
const response = await fetch(`${API_BASE}/pets/${petId}/detect`, {
  method: 'POST',
  headers: {
    'Content-Type': 'application/json',
    'Authorization': `Bearer ${accessToken}`,
    'X-API-Version': '1.0.0'
  },
  body: JSON.stringify(emotionRequest)
});

const result = await response.json();
```

### 9.4 Bluetooth LE Transport

```typescript
// BLE Service UUID
const SERVICE_UUID = '0000fe00-0000-1000-8000-00805f9b34fb';

// Characteristics
const CHARACTERISTICS = {
  emotion_data: '0000fe01-0000-1000-8000-00805f9b34fb',
  sensor_data: '0000fe02-0000-1000-8000-00805f9b34fb',
  control: '0000fe03-0000-1000-8000-00805f9b34fb'
};

// Connect to device
const device = await navigator.bluetooth.requestDevice({
  filters: [{ services: [SERVICE_UUID] }]
});

const server = await device.gatt.connect();
const service = await server.getPrimaryService(SERVICE_UUID);

// Subscribe to emotion data
const emotionChar = await service.getCharacteristic(CHARACTERISTICS.emotion_data);
await emotionChar.startNotifications();
emotionChar.addEventListener('characteristicvaluechanged', (event) => {
  const data = parseEmotionData(event.target.value);
  handleEmotionUpdate(data);
});
```

---

## 10. Error Handling

### 10.1 Error Codes

| Code | Category | Description |
|------|----------|-------------|
| 1000 | Protocol | Invalid protocol version |
| 1001 | Protocol | Malformed message |
| 1002 | Protocol | Unsupported message type |
| 2000 | Authentication | Authentication failed |
| 2001 | Authentication | Invalid credentials |
| 2002 | Authentication | Token expired |
| 2003 | Authentication | Insufficient permissions |
| 3000 | Connection | Connection timeout |
| 3001 | Connection | Connection refused |
| 3002 | Connection | Session expired |
| 4000 | Detection | Detection failed |
| 4001 | Detection | Insufficient data quality |
| 4002 | Detection | Model not available |
| 5000 | Sensor | Sensor error |
| 5001 | Sensor | Sensor not found |
| 5002 | Sensor | Sensor offline |
| 6000 | Data | Data validation failed |
| 6001 | Data | Data corruption detected |
| 9000 | Internal | Internal server error |

### 10.2 Error Message Format

```json
{
  "protocol": "wia-pet-emotion",
  "version": "1.0.0",
  "messageId": "...",
  "timestamp": 1702915500000,
  "type": "error",
  "payload": {
    "error_code": 4001,
    "error_category": "Detection",
    "error_message": "Insufficient data quality for emotion detection",
    "details": {
      "image_quality_score": 0.35,
      "min_required": 0.60,
      "issues": ["low_resolution", "poor_lighting"]
    },
    "recoverable": true,
    "suggested_action": "Improve lighting and ensure pet is clearly visible",
    "reference_message_id": "...",
    "help_url": "https://docs.wia-pet-emotion.com/errors/4001"
  }
}
```

### 10.3 Retry Strategy

```typescript
interface RetryConfig {
  max_retries: number;
  initial_delay: number;
  max_delay: number;
  backoff_multiplier: number;
  retryable_errors: number[];  // Error codes that can be retried
}

async function sendWithRetry(
  message: any,
  config: RetryConfig
): Promise<any> {
  let attempt = 0;

  while (attempt < config.max_retries) {
    try {
      return await send(message);
    } catch (error) {
      if (!config.retryable_errors.includes(error.code)) {
        throw error;
      }

      const delay = Math.min(
        config.initial_delay * Math.pow(config.backoff_multiplier, attempt),
        config.max_delay
      );

      await sleep(delay);
      attempt++;
    }
  }

  throw new Error('Max retries exceeded');
}
```

---

## 11. Performance Specifications

| Metric | Requirement | Target |
|--------|-------------|--------|
| Message Latency (p50) | < 100ms | < 50ms |
| Message Latency (p99) | < 500ms | < 200ms |
| Throughput | > 1000 msg/s | > 5000 msg/s |
| Connection Establishment | < 2s | < 1s |
| Reconnection Time | < 5s | < 2s |
| Stream Latency | < 100ms | < 33ms (30Hz) |
| Heartbeat Overhead | < 1% bandwidth | < 0.5% bandwidth |

---

## 12. Compliance and Standards

| Standard | Compliance |
|----------|------------|
| RFC 6455 | WebSocket Protocol |
| RFC 7692 | WebSocket Compression |
| MQTT 5.0 | MQTT Protocol |
| TLS 1.3 | Transport Security |
| OAuth 2.0 | Authentication |
| JSON Schema | Message Validation |

---

**Document ID**: WIA-PET-EMOTION-PHASE3-001
**Version**: 1.0.0
**Last Updated**: 2025-12-18
**Copyright**: © 2025 WIA - MIT License

---
**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
