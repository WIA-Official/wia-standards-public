# WIA BCI Communication Protocol Specification

**Phase 3: Communication Protocol Standard**

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01

---

## 1. Overview

### 1.1 Purpose

WIA BCI Communication Protocol은 BCI 기기와 소프트웨어 간의 실시간 데이터 전송을 위한 통신 프로토콜입니다. 이 프로토콜은 다양한 전송 계층(WebSocket, TCP, LSL 등)에서 동일한 메시지 형식을 사용하여 상호운용성을 보장합니다.

### 1.2 Design Goals

1. **Transport Agnostic**: 전송 계층과 독립적인 메시지 형식
2. **Real-time**: 저지연 실시간 데이터 스트리밍
3. **Reliable**: 연결 관리 및 재연결 메커니즘
4. **Compatible**: Phase 1/2와 완전 호환
5. **Extensible**: 새로운 메시지 타입 추가 용이

### 1.3 Protocol Stack

```
┌─────────────────────────────────────┐
│        Application Layer            │
│   (WiaBci, Signal Processing)       │
├─────────────────────────────────────┤
│        Protocol Layer               │
│   (Message Format, Handlers)        │
├─────────────────────────────────────┤
│        Transport Layer              │
│   (WebSocket, TCP, LSL, BLE)        │
├─────────────────────────────────────┤
│        Network Layer                │
│   (TCP/IP, UDP, Serial)             │
└─────────────────────────────────────┘
```

---

## 2. Terminology

| Term | Definition |
|------|------------|
| **Client** | BCI 데이터를 수신하는 애플리케이션 |
| **Server** | BCI 데이터를 송신하는 기기/소프트웨어 |
| **Message** | 프로토콜 메시지 단위 |
| **Payload** | 메시지 내용 (Signal, Command 등) |
| **Transport** | 전송 계층 어댑터 |
| **Session** | 연결 세션 |

---

## 3. Message Format

### 3.1 Base Message Structure

모든 메시지는 다음 JSON 구조를 따릅니다:

```json
{
  "protocol": "wia-bci",
  "version": "1.0.0",
  "messageId": "uuid-v4",
  "timestamp": 1702483200000,
  "type": "message_type",
  "payload": {}
}
```

### 3.2 Message Fields

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `protocol` | string | Yes | 프로토콜 식별자 ("wia-bci") |
| `version` | string | Yes | 프로토콜 버전 (semver) |
| `messageId` | string | Yes | 고유 메시지 ID (UUID v4) |
| `timestamp` | number | Yes | Unix timestamp (milliseconds) |
| `type` | string | Yes | 메시지 유형 |
| `payload` | object | Yes | 메시지 내용 |
| `sequence` | number | No | 메시지 순서 번호 |
| `sessionId` | string | No | 세션 식별자 |

### 3.3 Message Types

| Type | Direction | Description |
|------|-----------|-------------|
| `connect` | Client → Server | 연결 요청 |
| `connect_ack` | Server → Client | 연결 응답 |
| `disconnect` | Both | 연결 종료 |
| `start_stream` | Client → Server | 스트리밍 시작 요청 |
| `stop_stream` | Client → Server | 스트리밍 중지 요청 |
| `stream_ack` | Server → Client | 스트리밍 응답 |
| `signal` | Server → Client | 신호 데이터 (Phase 1) |
| `signal_batch` | Server → Client | 신호 데이터 배치 |
| `marker` | Both | 이벤트 마커 |
| `command` | Client → Server | 명령 전송 |
| `command_ack` | Server → Client | 명령 응답 |
| `config` | Both | 설정 변경 |
| `status` | Server → Client | 상태 업데이트 |
| `error` | Both | 에러 메시지 |
| `ping` | Client → Server | 연결 확인 |
| `pong` | Server → Client | 연결 확인 응답 |

---

## 4. Message Definitions

### 4.1 Connect

연결 요청 메시지:

```json
{
  "protocol": "wia-bci",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": 1702483200000,
  "type": "connect",
  "payload": {
    "clientId": "app-12345",
    "clientName": "My BCI App",
    "clientVersion": "1.0.0",
    "capabilities": ["eeg", "emg", "markers"],
    "options": {
      "samplingRate": 250,
      "channels": ["Fp1", "Fp2", "C3", "C4"],
      "compression": false,
      "binaryMode": false
    }
  }
}
```

### 4.2 Connect Ack

연결 응답 메시지:

```json
{
  "protocol": "wia-bci",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440001",
  "timestamp": 1702483200100,
  "type": "connect_ack",
  "payload": {
    "sessionId": "session-67890",
    "status": "connected",
    "serverInfo": {
      "name": "OpenBCI Server",
      "version": "2.0.0"
    },
    "deviceInfo": {
      "type": "eeg_headset",
      "manufacturer": "OpenBCI",
      "model": "Cyton",
      "channels": 8,
      "samplingRate": 250
    },
    "negotiated": {
      "samplingRate": 250,
      "channels": ["Fp1", "Fp2", "C3", "C4", "O1", "O2", "P3", "P4"],
      "compression": false
    }
  }
}
```

### 4.3 Signal

신호 데이터 메시지 (Phase 1 호환):

```json
{
  "protocol": "wia-bci",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440002",
  "timestamp": 1702483200200,
  "type": "signal",
  "sequence": 1,
  "payload": {
    "sampleIndex": 1000,
    "timestamp": 1702483200200,
    "channels": [0, 1, 2, 3, 4, 5, 6, 7],
    "data": [12.5, -8.3, 15.2, -3.1, 22.7, -11.4, 8.9, -5.6]
  }
}
```

### 4.4 Signal Batch

배치 신호 데이터 (효율적인 전송):

```json
{
  "protocol": "wia-bci",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440003",
  "timestamp": 1702483200300,
  "type": "signal_batch",
  "sequence": 2,
  "payload": {
    "startSampleIndex": 1000,
    "sampleCount": 10,
    "channels": [0, 1, 2, 3, 4, 5, 6, 7],
    "data": [
      [12.5, -8.3, 15.2, -3.1, 22.7, -11.4, 8.9, -5.6],
      [13.1, -7.9, 14.8, -2.8, 23.1, -10.9, 9.2, -5.2],
      ...
    ],
    "timestamps": [1702483200200, 1702483200204, ...]
  }
}
```

### 4.5 Marker

이벤트 마커 메시지:

```json
{
  "protocol": "wia-bci",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440004",
  "timestamp": 1702483200400,
  "type": "marker",
  "payload": {
    "sampleIndex": 1050,
    "code": 10,
    "label": "stimulus_onset",
    "value": "left_hand",
    "duration": 4000
  }
}
```

### 4.6 Command

명령 메시지:

```json
{
  "protocol": "wia-bci",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440005",
  "timestamp": 1702483200500,
  "type": "command",
  "payload": {
    "command": "set_sampling_rate",
    "params": {
      "rate": 500
    }
  }
}
```

### 4.7 Error

에러 메시지:

```json
{
  "protocol": "wia-bci",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440006",
  "timestamp": 1702483200600,
  "type": "error",
  "payload": {
    "code": 2001,
    "name": "DEVICE_NOT_FOUND",
    "message": "BCI device not found",
    "recoverable": false,
    "details": {
      "searchedDevices": ["COM3", "COM4"]
    }
  }
}
```

### 4.8 Ping/Pong

연결 유지 메시지:

```json
// Ping
{
  "protocol": "wia-bci",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440007",
  "timestamp": 1702483200700,
  "type": "ping",
  "payload": {}
}

// Pong
{
  "protocol": "wia-bci",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440008",
  "timestamp": 1702483200710,
  "type": "pong",
  "payload": {
    "serverTime": 1702483200710
  }
}
```

---

## 5. Connection Management

### 5.1 Connection State Machine

```
                    ┌─────────────────┐
                    │  DISCONNECTED   │
                    └────────┬────────┘
                             │ connect()
                             ▼
                    ┌─────────────────┐
         timeout    │   CONNECTING    │
        ┌──────────│                 │
        │          └────────┬────────┘
        │                   │ connect_ack
        │                   ▼
        │          ┌─────────────────┐
        │          │    CONNECTED    │◄────────┐
        │          └────────┬────────┘         │
        │                   │                  │
        │           error / │                  │ success
        │         disconnect│                  │
        │                   ▼                  │
        │          ┌─────────────────┐         │
        └─────────►│  RECONNECTING   ├─────────┘
                   └────────┬────────┘
                            │ max retries
                            ▼
                   ┌─────────────────┐
                   │     ERROR       │
                   └─────────────────┘
```

### 5.2 Connection Lifecycle

1. **DISCONNECTED**: 초기 상태
2. **CONNECTING**: 연결 시도 중
3. **CONNECTED**: 연결됨, 스트리밍 가능
4. **RECONNECTING**: 재연결 시도 중
5. **ERROR**: 복구 불가능한 에러

### 5.3 Heartbeat

연결 유지를 위한 ping/pong 메커니즘:

- **Ping Interval**: 30초 (기본값)
- **Pong Timeout**: 10초
- **Max Missed**: 3회

```typescript
// Heartbeat configuration
interface HeartbeatConfig {
  pingInterval: number;    // 30000ms
  pongTimeout: number;     // 10000ms
  maxMissedPongs: number;  // 3
}
```

### 5.4 Reconnection Strategy

자동 재연결 설정:

```typescript
interface ReconnectConfig {
  enabled: boolean;
  maxAttempts: number;       // 5
  initialDelay: number;      // 1000ms
  maxDelay: number;          // 30000ms
  backoffMultiplier: number; // 2
}

// Delay calculation: min(initialDelay * (backoffMultiplier ^ attempt), maxDelay)
```

---

## 6. Error Handling

### 6.1 Error Codes

| Code | Name | Description | Recoverable |
|------|------|-------------|-------------|
| 1000 | CONNECTION_CLOSED | 정상 종료 | N/A |
| 1001 | CONNECTION_LOST | 연결 끊김 | Yes |
| 1002 | CONNECTION_TIMEOUT | 연결 시간 초과 | Yes |
| 1003 | PROTOCOL_ERROR | 프로토콜 오류 | No |
| 1004 | VERSION_MISMATCH | 버전 불일치 | No |
| 2001 | DEVICE_NOT_FOUND | 기기 없음 | No |
| 2002 | DEVICE_BUSY | 기기 사용 중 | Yes |
| 2003 | DEVICE_ERROR | 기기 오류 | Yes |
| 2004 | STREAM_ERROR | 스트리밍 오류 | Yes |
| 3001 | INVALID_MESSAGE | 잘못된 메시지 | No |
| 3002 | INVALID_PAYLOAD | 잘못된 페이로드 | No |
| 3003 | UNSUPPORTED_TYPE | 지원하지 않는 타입 | No |
| 4001 | AUTH_FAILED | 인증 실패 | No |
| 4002 | PERMISSION_DENIED | 권한 없음 | No |

### 6.2 Error Response

에러 발생 시 즉시 error 메시지 전송:

```json
{
  "type": "error",
  "payload": {
    "code": 2003,
    "name": "DEVICE_ERROR",
    "message": "Electrode contact lost on channel C3",
    "recoverable": true,
    "details": {
      "channel": "C3",
      "impedance": "Infinity"
    }
  }
}
```

---

## 7. Transport Layer

### 7.1 Transport Interface

```typescript
interface ITransport {
  // Connection
  connect(url: string, options?: TransportOptions): Promise<void>;
  disconnect(): Promise<void>;
  isConnected(): boolean;

  // Messaging
  send(message: BciMessage): Promise<void>;
  onMessage(handler: (message: BciMessage) => void): void;

  // Events
  onOpen(handler: () => void): void;
  onClose(handler: (code: number, reason: string) => void): void;
  onError(handler: (error: Error) => void): void;
}
```

### 7.2 WebSocket Transport

기본 전송 계층:

```typescript
// Connection URL format
ws://host:port/wia-bci
wss://host:port/wia-bci

// Default port: 9876

// Subprotocol
Sec-WebSocket-Protocol: wia-bci-v1
```

**WebSocket 설정**:
- Binary Type: arraybuffer (binary mode) / text (JSON mode)
- Ping Interval: 30s
- Max Message Size: 1MB

### 7.3 TCP Transport

고성능 로컬 전송:

```typescript
// Connection format
tcp://host:port

// Default port: 9877

// Message framing
[4 bytes: length][payload]
```

### 7.4 LSL Bridge

Lab Streaming Layer 호환:

```typescript
// Stream info
Name: "WIA-BCI"
Type: "EEG"
Channel Count: N
Nominal Rate: 250
Channel Format: float32
Source ID: "wia-bci-<device-id>"

// Marker stream
Name: "WIA-BCI-Markers"
Type: "Markers"
```

### 7.5 Mock Transport

개발/테스트용:

```typescript
const transport = new MockTransport({
  latency: 10,           // Simulated latency (ms)
  dropRate: 0.01,        // Message drop rate (1%)
  disconnectChance: 0.001 // Random disconnect chance
});
```

---

## 8. Binary Mode

### 8.1 Overview

성능이 중요한 경우 바이너리 모드 사용:

```typescript
// Enable in connect options
{
  "options": {
    "binaryMode": true
  }
}
```

### 8.2 Binary Message Format

```
┌──────────────────────────────────────────────────┐
│ Header (16 bytes)                                │
├──────────────────────────────────────────────────┤
│ [0-3]   Magic Number (0x57494142 = "WIAB")       │
│ [4-5]   Version (uint16, 0x0100 = 1.0)           │
│ [6-7]   Message Type (uint16)                    │
│ [8-11]  Sequence Number (uint32)                 │
│ [12-15] Payload Length (uint32)                  │
├──────────────────────────────────────────────────┤
│ Payload (variable length)                        │
└──────────────────────────────────────────────────┘
```

### 8.3 Signal Binary Format

```
┌──────────────────────────────────────────────────┐
│ Signal Header (16 bytes)                         │
├──────────────────────────────────────────────────┤
│ [0-7]   Timestamp (int64, microseconds)          │
│ [8-11]  Sample Index (uint32)                    │
│ [12-13] Channel Count (uint16)                   │
│ [14-15] Reserved                                 │
├──────────────────────────────────────────────────┤
│ Channel Data (channels × 4 bytes)                │
│ [float32 × channel_count]                        │
└──────────────────────────────────────────────────┘
```

---

## 9. Security

### 9.1 Transport Security

- **WebSocket**: WSS (TLS) 권장
- **TCP**: TLS 옵션 제공
- **Local**: 로컬호스트만 허용

### 9.2 Authentication (Optional)

```json
{
  "type": "connect",
  "payload": {
    "auth": {
      "type": "token",
      "token": "jwt-token-here"
    }
  }
}
```

### 9.3 Data Encryption

민감 데이터 암호화 옵션:

```json
{
  "options": {
    "encryption": {
      "enabled": true,
      "algorithm": "AES-256-GCM"
    }
  }
}
```

---

## 10. Examples

### 10.1 Complete Session Example

```
Client                          Server
  │                               │
  │──────── connect ─────────────>│
  │                               │
  │<─────── connect_ack ──────────│
  │                               │
  │──────── start_stream ────────>│
  │                               │
  │<─────── stream_ack ───────────│
  │                               │
  │<─────── signal ───────────────│
  │<─────── signal ───────────────│
  │<─────── signal ───────────────│
  │                               │
  │──────── marker ──────────────>│
  │                               │
  │<─────── signal ───────────────│
  │<─────── signal ───────────────│
  │                               │
  │──────── ping ────────────────>│
  │<─────── pong ─────────────────│
  │                               │
  │──────── stop_stream ─────────>│
  │                               │
  │<─────── stream_ack ───────────│
  │                               │
  │──────── disconnect ──────────>│
  │                               │
```

### 10.2 Error Recovery Example

```
Client                          Server
  │                               │
  │<─────── signal ───────────────│
  │                               │
  │         (connection lost)     │
  │                               │
  │──────── connect ─────────────>│  (reconnect attempt 1)
  │         (timeout)             │
  │                               │
  │──────── connect ─────────────>│  (reconnect attempt 2)
  │                               │
  │<─────── connect_ack ──────────│
  │                               │
  │──────── start_stream ────────>│
  │                               │
  │<─────── stream_ack ───────────│
  │                               │
  │<─────── signal ───────────────│  (resume streaming)
  │                               │
```

---

## 11. JSON Schema

### 11.1 Message Schema Location

```
/spec/schemas/protocol/
├── message.schema.json
├── connect.schema.json
├── signal.schema.json
├── marker.schema.json
├── error.schema.json
└── ...
```

### 11.2 Base Message Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.live/schemas/bci/protocol/message.schema.json",
  "title": "WIA BCI Protocol Message",
  "type": "object",
  "required": ["protocol", "version", "messageId", "timestamp", "type", "payload"],
  "properties": {
    "protocol": {
      "type": "string",
      "const": "wia-bci"
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
        "connect", "connect_ack", "disconnect",
        "start_stream", "stop_stream", "stream_ack",
        "signal", "signal_batch", "marker",
        "command", "command_ack", "config", "status",
        "error", "ping", "pong"
      ]
    },
    "payload": {
      "type": "object"
    },
    "sequence": {
      "type": "integer",
      "minimum": 0
    },
    "sessionId": {
      "type": "string"
    }
  }
}
```

---

## 12. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial specification |

---

**Document Version**: 1.0.0
**Last Updated**: 2025-01-XX
**Author**: WIA BCI Working Group

---

弘益人間 - *Benefit All Humanity*
