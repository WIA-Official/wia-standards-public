# Phase 3: Communication Protocol Specification
# WIA Climate 통신 프로토콜 표준

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-12-14
**Author**: Claude Code (Opus 4.5)

---

## 목차 (Table of Contents)

1. [개요](#1-개요)
2. [설계 원칙](#2-설계-원칙)
3. [메시지 형식](#3-메시지-형식)
4. [메시지 유형](#4-메시지-유형)
5. [연결 관리](#5-연결-관리)
6. [에러 처리](#6-에러-처리)
7. [전송 계층](#7-전송-계층)
8. [보안](#8-보안)
9. [구현 가이드](#9-구현-가이드)

---

## 1. 개요

### 1.1 목적

WIA Climate Protocol은 기후/환경 센서와 시스템 간 통신을 위한 애플리케이션 레벨 프로토콜입니다. Phase 1에서 정의한 데이터 형식을 페이로드로 사용하며, 다양한 전송 계층(WebSocket, MQTT, HTTP)을 지원합니다.

### 1.2 범위

- 메시지 형식 정의
- 연결 생명주기 관리
- 명령/응답 패턴
- 에러 처리
- 다중 전송 방식 지원

### 1.3 아키텍처

```
┌─────────────────────────────────────────────────────────────────┐
│                        Application                               │
│                   (Climate Monitoring App)                       │
├─────────────────────────────────────────────────────────────────┤
│                    WIA Climate Protocol                          │
│              (Message Format & Semantics)                        │
├─────────────────────────────────────────────────────────────────┤
│              Phase 1 Data Format (Payload)                       │
│          (ClimateMessage with typed data)                        │
├────────────┬─────────────┬─────────────┬────────────────────────┤
│ WebSocket  │    MQTT     │    HTTP     │    Serial              │
│ Transport  │  Transport  │  Transport  │   Transport            │
├────────────┴─────────────┴─────────────┴────────────────────────┤
│                      Network Layer                               │
│                     (TCP/UDP/Serial)                             │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. 설계 원칙

### 2.1 핵심 원칙

| 원칙 | 설명 |
|------|------|
| **트랜스포트 독립성** | 메시지 형식은 전송 계층과 분리되어 동작 |
| **Phase 1/2 호환** | 기존 데이터 형식을 페이로드로 그대로 사용 |
| **경량 설계** | IoT 센서 환경을 고려한 최소 오버헤드 |
| **확장 가능** | 새로운 메시지 타입 및 전송 방식 추가 용이 |
| **양방향 통신** | 요청/응답 및 스트리밍 모두 지원 |

### 2.2 호환성

- Phase 1 데이터 형식과 100% 호환
- Phase 2 Rust API의 `ClimateMessage`를 페이로드로 사용
- 기존 JSON Schema로 페이로드 검증 가능

---

## 3. 메시지 형식

### 3.1 프로토콜 메시지 구조

모든 WIA Climate 프로토콜 메시지는 다음 형식을 따릅니다:

```json
{
  "protocol": "wia-climate",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": 1702483200000,
  "type": "data",
  "payload": { ... },
  "meta": { ... }
}
```

### 3.2 필드 정의

| 필드 | 타입 | 필수 | 설명 |
|------|------|------|------|
| `protocol` | string | Yes | 프로토콜 식별자. 항상 `"wia-climate"` |
| `version` | string | Yes | 프로토콜 버전. SemVer 형식 (예: `"1.0.0"`) |
| `messageId` | string | Yes | 메시지 고유 식별자. UUID v4 형식 |
| `timestamp` | integer | Yes | 메시지 생성 시각. Unix timestamp (milliseconds) |
| `type` | string | Yes | 메시지 유형. [메시지 유형](#4-메시지-유형) 참조 |
| `payload` | object | Conditional | 메시지 본문. 타입에 따라 필수 여부 결정 |
| `meta` | object | No | 추가 메타데이터 |

### 3.3 메시지 ID 생성

```
messageId 형식: UUID v4
예시: "550e8400-e29b-41d4-a716-446655440000"

용도:
- 요청/응답 매칭
- 중복 메시지 감지
- 로깅 및 추적
```

### 3.4 타임스탬프 규칙

- Unix timestamp (milliseconds since 1970-01-01 00:00:00 UTC)
- 항상 UTC 기준
- 메시지 생성 시점 기록

---

## 4. 메시지 유형

### 4.1 메시지 유형 목록

| Type | 방향 | 설명 | Payload |
|------|-----|------|---------|
| `connect` | C → S | 연결 요청 | ConnectPayload |
| `connect_ack` | S → C | 연결 응답 | ConnectAckPayload |
| `disconnect` | Both | 연결 종료 | DisconnectPayload (optional) |
| `data` | S → C | 센서 데이터 | Phase 1 ClimateMessage |
| `command` | C → S | 명령 전송 | CommandPayload |
| `command_ack` | S → C | 명령 응답 | CommandAckPayload |
| `subscribe` | C → S | 데이터 구독 | SubscribePayload |
| `subscribe_ack` | S → C | 구독 응답 | SubscribeAckPayload |
| `unsubscribe` | C → S | 구독 해제 | UnsubscribePayload |
| `error` | Both | 에러 메시지 | ErrorPayload |
| `ping` | C → S | 연결 확인 | (none) |
| `pong` | S → C | 연결 확인 응답 | (none) |

방향: C = Client, S = Server

### 4.2 connect

클라이언트가 서버에 연결을 요청합니다.

```json
{
  "protocol": "wia-climate",
  "version": "1.0.0",
  "messageId": "...",
  "timestamp": 1702483200000,
  "type": "connect",
  "payload": {
    "clientId": "client-001",
    "clientType": "sensor",
    "capabilities": ["carbon_capture", "vertical_farming"],
    "auth": {
      "method": "api_key",
      "token": "..."
    }
  }
}
```

**ConnectPayload**:

| 필드 | 타입 | 필수 | 설명 |
|------|------|------|------|
| `clientId` | string | Yes | 클라이언트 고유 식별자 |
| `clientType` | string | No | 클라이언트 유형 (`sensor`, `gateway`, `dashboard`, `service`) |
| `capabilities` | string[] | No | 지원하는 데이터 타입 목록 |
| `auth` | object | No | 인증 정보 |

### 4.3 connect_ack

서버가 연결 요청에 응답합니다.

```json
{
  "protocol": "wia-climate",
  "version": "1.0.0",
  "messageId": "...",
  "timestamp": 1702483200000,
  "type": "connect_ack",
  "payload": {
    "success": true,
    "sessionId": "session-12345",
    "serverInfo": {
      "name": "WIA Climate Server",
      "version": "1.0.0"
    },
    "keepAliveInterval": 30000
  }
}
```

**ConnectAckPayload**:

| 필드 | 타입 | 필수 | 설명 |
|------|------|------|------|
| `success` | boolean | Yes | 연결 성공 여부 |
| `sessionId` | string | Conditional | 세션 ID (성공 시) |
| `serverInfo` | object | No | 서버 정보 |
| `keepAliveInterval` | integer | No | Keep-alive 주기 (ms) |
| `error` | ErrorPayload | Conditional | 에러 정보 (실패 시) |

### 4.4 disconnect

연결을 종료합니다.

```json
{
  "protocol": "wia-climate",
  "version": "1.0.0",
  "messageId": "...",
  "timestamp": 1702483200000,
  "type": "disconnect",
  "payload": {
    "reason": "normal",
    "message": "Client shutdown"
  }
}
```

**DisconnectPayload** (optional):

| 필드 | 타입 | 필수 | 설명 |
|------|------|------|------|
| `reason` | string | No | 종료 사유 코드 |
| `message` | string | No | 사람이 읽을 수 있는 메시지 |

### 4.5 data

센서 데이터를 전송합니다. Payload는 Phase 1의 `ClimateMessage` 형식입니다.

```json
{
  "protocol": "wia-climate",
  "version": "1.0.0",
  "messageId": "...",
  "timestamp": 1702483200000,
  "type": "data",
  "payload": {
    "version": "1.0.0",
    "type": "carbon_capture",
    "timestamp": {
      "unix_ms": 1702483200000,
      "iso8601": "2024-12-14T12:00:00.000Z"
    },
    "location": {
      "latitude": 64.0,
      "longitude": -21.0
    },
    "device": {
      "manufacturer": "Climeworks",
      "model": "Orca DAC"
    },
    "data": {
      "technology": "dac",
      "capture_rate_kg_per_hour": 125.5
    }
  }
}
```

### 4.6 command

클라이언트가 서버/장치에 명령을 전송합니다.

```json
{
  "protocol": "wia-climate",
  "version": "1.0.0",
  "messageId": "cmd-001",
  "timestamp": 1702483200000,
  "type": "command",
  "payload": {
    "targetId": "device-orca-001",
    "action": "set_capture_rate",
    "parameters": {
      "rate_kg_per_hour": 150.0
    },
    "timeout": 5000
  }
}
```

**CommandPayload**:

| 필드 | 타입 | 필수 | 설명 |
|------|------|------|------|
| `targetId` | string | Yes | 대상 장치/서비스 ID |
| `action` | string | Yes | 명령 액션 이름 |
| `parameters` | object | No | 명령 파라미터 |
| `timeout` | integer | No | 타임아웃 (ms) |

### 4.7 command_ack

명령 실행 결과를 응답합니다.

```json
{
  "protocol": "wia-climate",
  "version": "1.0.0",
  "messageId": "...",
  "timestamp": 1702483200000,
  "type": "command_ack",
  "payload": {
    "commandId": "cmd-001",
    "success": true,
    "result": {
      "new_rate_kg_per_hour": 150.0
    },
    "executionTime": 250
  }
}
```

**CommandAckPayload**:

| 필드 | 타입 | 필수 | 설명 |
|------|------|------|------|
| `commandId` | string | Yes | 원본 명령의 messageId |
| `success` | boolean | Yes | 명령 성공 여부 |
| `result` | object | No | 실행 결과 (성공 시) |
| `error` | ErrorPayload | Conditional | 에러 정보 (실패 시) |
| `executionTime` | integer | No | 실행 시간 (ms) |

### 4.8 subscribe

데이터 스트림을 구독합니다.

```json
{
  "protocol": "wia-climate",
  "version": "1.0.0",
  "messageId": "...",
  "timestamp": 1702483200000,
  "type": "subscribe",
  "payload": {
    "topics": [
      {
        "pattern": "carbon_capture/*",
        "filter": {
          "location.latitude": { "gte": 60.0 }
        }
      }
    ],
    "qos": 1
  }
}
```

**SubscribePayload**:

| 필드 | 타입 | 필수 | 설명 |
|------|------|------|------|
| `topics` | TopicSubscription[] | Yes | 구독할 토픽 목록 |
| `qos` | integer | No | QoS 레벨 (0, 1, 2). 기본값: 0 |

**TopicSubscription**:

| 필드 | 타입 | 필수 | 설명 |
|------|------|------|------|
| `pattern` | string | Yes | 토픽 패턴 (와일드카드 지원: `*`, `**`) |
| `filter` | object | No | 데이터 필터 조건 |

### 4.9 subscribe_ack

구독 요청에 응답합니다.

```json
{
  "protocol": "wia-climate",
  "version": "1.0.0",
  "messageId": "...",
  "timestamp": 1702483200000,
  "type": "subscribe_ack",
  "payload": {
    "subscriptionId": "sub-12345",
    "topics": ["carbon_capture/*"],
    "success": true
  }
}
```

### 4.10 unsubscribe

구독을 해제합니다.

```json
{
  "protocol": "wia-climate",
  "version": "1.0.0",
  "messageId": "...",
  "timestamp": 1702483200000,
  "type": "unsubscribe",
  "payload": {
    "subscriptionId": "sub-12345"
  }
}
```

### 4.11 error

에러 메시지를 전송합니다.

```json
{
  "protocol": "wia-climate",
  "version": "1.0.0",
  "messageId": "...",
  "timestamp": 1702483200000,
  "type": "error",
  "payload": {
    "code": "VALIDATION_ERROR",
    "message": "Invalid latitude value: must be between -90 and 90",
    "details": {
      "field": "payload.location.latitude",
      "value": 100.0
    },
    "relatedMessageId": "original-msg-id"
  }
}
```

### 4.12 ping / pong

연결 상태를 확인합니다.

```json
// ping (Client → Server)
{
  "protocol": "wia-climate",
  "version": "1.0.0",
  "messageId": "...",
  "timestamp": 1702483200000,
  "type": "ping"
}

// pong (Server → Client)
{
  "protocol": "wia-climate",
  "version": "1.0.0",
  "messageId": "...",
  "timestamp": 1702483200001,
  "type": "pong"
}
```

---

## 5. 연결 관리

### 5.1 연결 생명주기

```
┌──────────┐    connect      ┌──────────────┐
│          │ ───────────────>│              │
│  INIT    │                 │  CONNECTING  │
│          │<────────────────│              │
└──────────┘   connect_ack   └──────┬───────┘
                                    │ success
                                    ▼
              ┌─────────────────────────────────────┐
              │                                     │
              │            CONNECTED                │
              │                                     │
              │  ┌─────────┐     ┌─────────┐       │
              │  │subscribe│     │  data   │       │
              │  │ command │ <-> │  ping   │       │
              │  │   ...   │     │  pong   │       │
              │  └─────────┘     └─────────┘       │
              │                                     │
              └──────────────────┬──────────────────┘
                                 │ disconnect / error
                                 ▼
                          ┌──────────────┐
                          │ DISCONNECTED │
                          └──────────────┘
```

### 5.2 Keep-Alive

- 클라이언트는 `connect_ack`의 `keepAliveInterval`에 따라 `ping` 전송
- 서버는 `pong`으로 응답
- 지정된 시간 내 응답 없으면 연결 끊김으로 간주

### 5.3 재연결 정책

```
Initial delay: 1000ms
Max delay: 30000ms
Multiplier: 2x
Jitter: ±20%

예시 시퀀스:
  시도 1: 1000ms ± 200ms
  시도 2: 2000ms ± 400ms
  시도 3: 4000ms ± 800ms
  시도 4: 8000ms ± 1600ms
  시도 5: 16000ms ± 3200ms
  시도 6+: 30000ms ± 6000ms
```

---

## 6. 에러 처리

### 6.1 에러 코드

| 코드 | 설명 |
|------|------|
| `PROTOCOL_ERROR` | 프로토콜 형식 오류 |
| `VERSION_MISMATCH` | 프로토콜 버전 불일치 |
| `AUTH_FAILED` | 인증 실패 |
| `AUTH_EXPIRED` | 인증 만료 |
| `PERMISSION_DENIED` | 권한 없음 |
| `VALIDATION_ERROR` | 데이터 검증 실패 |
| `NOT_FOUND` | 리소스 없음 |
| `CONFLICT` | 충돌 (예: 중복 구독) |
| `RATE_LIMITED` | 요청 제한 초과 |
| `TIMEOUT` | 타임아웃 |
| `INTERNAL_ERROR` | 서버 내부 오류 |
| `SERVICE_UNAVAILABLE` | 서비스 이용 불가 |

### 6.2 ErrorPayload

```json
{
  "code": "VALIDATION_ERROR",
  "message": "Human-readable error description",
  "details": {
    "field": "payload.data.capture_rate_kg_per_hour",
    "constraint": "must be positive",
    "value": -10.0
  },
  "relatedMessageId": "original-message-uuid",
  "retryable": true,
  "retryAfter": 5000
}
```

| 필드 | 타입 | 필수 | 설명 |
|------|------|------|------|
| `code` | string | Yes | 에러 코드 |
| `message` | string | Yes | 사람이 읽을 수 있는 메시지 |
| `details` | object | No | 상세 에러 정보 |
| `relatedMessageId` | string | No | 관련 메시지 ID |
| `retryable` | boolean | No | 재시도 가능 여부 |
| `retryAfter` | integer | No | 재시도 대기 시간 (ms) |

---

## 7. 전송 계층

### 7.1 WebSocket Transport

**연결 설정**:
```
URL: wss://api.example.com/wia-climate/v1/ws
Subprotocol: wia-climate-v1

Headers:
  Authorization: Bearer <token>
  X-Client-Id: <client-id>
```

**메시지 전송**:
- 텍스트 프레임으로 JSON 메시지 전송
- 바이너리 프레임은 사용하지 않음

### 7.2 MQTT Transport

**토픽 구조**:
```
wia/climate/{version}/{clientId}/{messageType}
wia/climate/{version}/{clientId}/data/{dataType}
wia/climate/{version}/broadcast/{topic}

예시:
  wia/climate/v1/client-001/data/carbon_capture
  wia/climate/v1/client-001/command
  wia/climate/v1/broadcast/alerts
```

**QoS 매핑**:

| 메시지 타입 | 권장 QoS |
|------------|---------|
| data (streaming) | 0 |
| command | 2 |
| command_ack | 1 |
| subscribe | 1 |
| error | 1 |

### 7.3 HTTP Transport

**엔드포인트**:
```
POST /wia-climate/v1/messages
  - 단일 메시지 전송

GET /wia-climate/v1/data?type=carbon_capture&limit=100
  - 데이터 조회

POST /wia-climate/v1/commands
  - 명령 전송

GET /wia-climate/v1/commands/{commandId}/status
  - 명령 상태 조회
```

**Long Polling** (실시간 대안):
```
GET /wia-climate/v1/stream?subscriptionId=...&timeout=30000
```

### 7.4 Serial Transport

로컬 센서 연결용:

```
Baud rate: 115200
Data bits: 8
Stop bits: 1
Parity: None

프레임 형식:
  [STX (0x02)] [LENGTH (2 bytes)] [JSON payload] [CRC16] [ETX (0x03)]
```

---

## 8. 보안

### 8.1 전송 암호화

- WebSocket: WSS (TLS 1.2+)
- MQTT: MQTTS (TLS 1.2+)
- HTTP: HTTPS (TLS 1.2+)

### 8.2 인증 방식

**API Key**:
```json
{
  "auth": {
    "method": "api_key",
    "token": "wia_climate_api_key_..."
  }
}
```

**JWT**:
```json
{
  "auth": {
    "method": "jwt",
    "token": "eyJhbGciOiJIUzI1NiIs..."
  }
}
```

### 8.3 메시지 서명 (선택)

```json
{
  "protocol": "wia-climate",
  "version": "1.0.0",
  "messageId": "...",
  "timestamp": 1702483200000,
  "type": "data",
  "payload": { ... },
  "meta": {
    "signature": {
      "algorithm": "HMAC-SHA256",
      "value": "base64-encoded-signature"
    }
  }
}
```

---

## 9. 구현 가이드

### 9.1 Rust 모듈 구조

```
src/
├── protocol/
│   ├── mod.rs           # 모듈 진입점
│   ├── message.rs       # ProtocolMessage 타입
│   ├── message_types.rs # 메시지 유형별 페이로드
│   ├── builder.rs       # 메시지 빌더
│   └── handler.rs       # 메시지 핸들러 트레잇
└── transport/
    ├── mod.rs           # 모듈 진입점
    ├── websocket.rs     # WebSocket 구현
    ├── mqtt.rs          # MQTT 구현 (향후)
    └── mock.rs          # 테스트용 Mock
```

### 9.2 핵심 타입 예시

```rust
use serde::{Deserialize, Serialize};
use uuid::Uuid;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProtocolMessage {
    pub protocol: String,  // "wia-climate"
    pub version: String,   // "1.0.0"
    #[serde(rename = "messageId")]
    pub message_id: String,
    pub timestamp: i64,
    #[serde(rename = "type")]
    pub message_type: MessageType,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub payload: Option<serde_json::Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub meta: Option<MessageMeta>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MessageType {
    Connect,
    ConnectAck,
    Disconnect,
    Data,
    Command,
    CommandAck,
    Subscribe,
    SubscribeAck,
    Unsubscribe,
    Error,
    Ping,
    Pong,
}
```

### 9.3 메시지 핸들러

```rust
#[async_trait]
pub trait MessageHandler: Send + Sync {
    async fn on_connect(&self, msg: &ConnectPayload) -> Result<ConnectAckPayload>;
    async fn on_data(&self, msg: &ClimateMessage) -> Result<()>;
    async fn on_command(&self, msg: &CommandPayload) -> Result<CommandAckPayload>;
    async fn on_subscribe(&self, msg: &SubscribePayload) -> Result<SubscribeAckPayload>;
    async fn on_error(&self, msg: &ErrorPayload) -> Result<()>;
}
```

### 9.4 Transport 추상화

```rust
#[async_trait]
pub trait Transport: Send + Sync {
    async fn connect(&mut self, url: &str) -> Result<()>;
    async fn disconnect(&mut self) -> Result<()>;
    async fn send(&self, message: &ProtocolMessage) -> Result<()>;
    async fn receive(&mut self) -> Result<ProtocolMessage>;
    fn is_connected(&self) -> bool;
}
```

---

## Appendix A: JSON Schema

프로토콜 메시지 검증을 위한 JSON Schema는 `/spec/schemas/protocol-message.schema.json`에 정의됩니다.

## Appendix B: 버전 히스토리

| 버전 | 날짜 | 변경 사항 |
|------|------|----------|
| 1.0.0 | 2025-12-14 | 최초 릴리스 |

---

弘益人間 - Benefit All Humanity 🌍
