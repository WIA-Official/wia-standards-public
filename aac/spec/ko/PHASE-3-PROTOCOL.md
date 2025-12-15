# WIA AAC Standard - Phase 3: Communication Protocol
# 통신 프로토콜 표준

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-12

---

## 1. 개요 (Overview)

### 1.1 목적 (Purpose)

본 문서는 WIA AAC 표준의 Phase 3으로, AAC 센서와 소프트웨어 간 통신을 위한 프로토콜을 정의합니다.

This document defines the communication protocol for WIA AAC Standard Phase 3, enabling standardized communication between AAC sensors and software.

### 1.2 범위 (Scope)

- 메시지 형식 (Message Format)
- 메시지 유형 (Message Types)
- 연결 관리 (Connection Management)
- 에러 처리 (Error Handling)
- 전송 계층 (Transport Layer)

### 1.3 연관 문서 (Related Documents)

| 문서 | 설명 |
|-----|------|
| PHASE-1-SIGNAL-FORMAT.md | 센서 신호 형식 |
| PHASE-2-API-INTERFACE.md | API 인터페이스 |
| RESEARCH-PHASE-3.md | 사전 조사 결과 |

---

## 2. 용어 정의 (Terminology)

| 용어 | 정의 |
|-----|------|
| **Client** | AAC 소프트웨어 (WIA AAC API 사용) |
| **Server** | 센서 제공자 (센서 드라이버/브릿지) |
| **Message** | 클라이언트-서버 간 전송 단위 |
| **Payload** | 메시지의 데이터 본문 |
| **Transport** | 메시지 전송 계층 (WebSocket, USB 등) |
| **Session** | 연결된 클라이언트-서버 통신 세션 |

---

## 3. 프로토콜 아키텍처 (Protocol Architecture)

```
┌──────────────────────────────────────────────────────────────┐
│                    Application Layer                          │
│                    (WIA AAC API)                              │
├──────────────────────────────────────────────────────────────┤
│                    Protocol Layer                             │
│            (Message Format, Handlers)                         │
├──────────────────────────────────────────────────────────────┤
│                    Transport Layer                            │
│         (WebSocket / USB HID / Bluetooth LE)                  │
└──────────────────────────────────────────────────────────────┘
```

---

## 4. 메시지 형식 (Message Format)

### 4.1 기본 구조 (Base Structure)

모든 WIA AAC 메시지는 다음 구조를 따릅니다:

```json
{
  "protocol": "wia-aac",
  "version": "1.0.0",
  "messageId": "uuid-v4-string",
  "timestamp": 1702483200000,
  "type": "message_type",
  "payload": {}
}
```

### 4.2 필드 정의 (Field Definitions)

| 필드 | 타입 | 필수 | 설명 |
|-----|-----|:---:|------|
| `protocol` | string | Y | 프로토콜 식별자 ("wia-aac") |
| `version` | string | Y | 프로토콜 버전 (SemVer) |
| `messageId` | string | Y | 고유 메시지 ID (UUID v4) |
| `timestamp` | number | Y | Unix timestamp (milliseconds) |
| `type` | string | Y | 메시지 유형 |
| `payload` | object | Y | 메시지 데이터 (유형별 상이) |

### 4.3 JSON Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.live/aac/protocol/v1/message.schema.json",
  "title": "WIA AAC Protocol Message",
  "type": "object",
  "required": ["protocol", "version", "messageId", "timestamp", "type", "payload"],
  "properties": {
    "protocol": {
      "type": "string",
      "const": "wia-aac"
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
        "signal", "command", "command_ack",
        "subscribe", "subscribe_ack", "unsubscribe", "unsubscribe_ack",
        "error", "ping", "pong"
      ]
    },
    "payload": {
      "type": "object"
    }
  }
}
```

---

## 5. 메시지 유형 (Message Types)

### 5.1 메시지 유형 목록

| Type | 방향 | 설명 |
|------|-----|------|
| `connect` | C → S | 연결 요청 |
| `connect_ack` | S → C | 연결 응답 |
| `disconnect` | Both | 연결 종료 요청 |
| `disconnect_ack` | Both | 연결 종료 응답 |
| `signal` | S → C | 센서 신호 (Phase 1 형식) |
| `command` | C → S | 센서 명령 |
| `command_ack` | S → C | 명령 응답 |
| `subscribe` | C → S | 데이터 스트림 구독 |
| `subscribe_ack` | S → C | 구독 응답 |
| `unsubscribe` | C → S | 구독 해제 |
| `unsubscribe_ack` | S → C | 구독 해제 응답 |
| `error` | Both | 에러 메시지 |
| `ping` | C → S | 연결 확인 |
| `pong` | S → C | 연결 확인 응답 |

### 5.2 연결 메시지 (Connection Messages)

#### 5.2.1 connect

클라이언트가 서버에 연결을 요청합니다.

```json
{
  "protocol": "wia-aac",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": 1702483200000,
  "type": "connect",
  "payload": {
    "clientId": "my-aac-app-12345",
    "clientName": "My AAC Application",
    "clientVersion": "2.1.0",
    "capabilities": ["eye_tracker", "switch", "muscle_sensor"],
    "options": {
      "signalRate": 60,
      "compression": false,
      "heartbeatInterval": 30000
    }
  }
}
```

**Payload 필드:**

| 필드 | 타입 | 필수 | 설명 |
|-----|-----|:---:|------|
| `clientId` | string | Y | 클라이언트 고유 ID |
| `clientName` | string | N | 클라이언트 이름 |
| `clientVersion` | string | N | 클라이언트 버전 |
| `capabilities` | string[] | N | 지원 센서 유형 |
| `options` | object | N | 연결 옵션 |

#### 5.2.2 connect_ack

서버가 연결 요청에 응답합니다.

```json
{
  "protocol": "wia-aac",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440001",
  "timestamp": 1702483200050,
  "type": "connect_ack",
  "payload": {
    "success": true,
    "sessionId": "session-abc123",
    "serverName": "WIA AAC Sensor Bridge",
    "serverVersion": "1.0.0",
    "availableSensors": [
      {
        "type": "eye_tracker",
        "manufacturer": "Tobii",
        "model": "Eye Tracker 5",
        "deviceId": "device-001"
      }
    ],
    "config": {
      "maxSignalRate": 120,
      "heartbeatInterval": 30000
    }
  }
}
```

#### 5.2.3 disconnect

연결 종료를 요청합니다.

```json
{
  "protocol": "wia-aac",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440010",
  "timestamp": 1702483500000,
  "type": "disconnect",
  "payload": {
    "reason": "user_request",
    "message": "User closed the application"
  }
}
```

### 5.3 신호 메시지 (Signal Message)

#### 5.3.1 signal

센서 신호를 전송합니다. payload는 Phase 1 Signal Format을 따릅니다.

```json
{
  "protocol": "wia-aac",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440100",
  "timestamp": 1702483200100,
  "type": "signal",
  "payload": {
    "$schema": "https://wia.live/aac/signal/v1/schema.json",
    "version": "1.0.0",
    "type": "eye_tracker",
    "timestamp": {
      "unix_ms": 1702483200100,
      "iso": "2024-12-13T12:00:00.100Z"
    },
    "sequence": 1,
    "device": {
      "manufacturer": "Tobii",
      "model": "Eye Tracker 5",
      "serial": "ET5-12345"
    },
    "data": {
      "gaze_point": { "x": 0.45, "y": 0.32 },
      "fixation": {
        "active": true,
        "duration_ms": 850,
        "target_id": "key_A"
      }
    },
    "meta": {
      "confidence": 0.95
    }
  }
}
```

### 5.4 구독 메시지 (Subscription Messages)

#### 5.4.1 subscribe

특정 센서의 데이터 스트림을 구독합니다.

```json
{
  "protocol": "wia-aac",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440020",
  "timestamp": 1702483200200,
  "type": "subscribe",
  "payload": {
    "streams": ["eye_tracker", "switch"],
    "deviceFilter": {
      "type": "eye_tracker",
      "manufacturer": "Tobii"
    },
    "options": {
      "signalRate": 60,
      "includeRaw": false
    }
  }
}
```

#### 5.4.2 subscribe_ack

```json
{
  "protocol": "wia-aac",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440021",
  "timestamp": 1702483200250,
  "type": "subscribe_ack",
  "payload": {
    "success": true,
    "subscriptionId": "sub-xyz789",
    "activeStreams": ["eye_tracker"],
    "actualSignalRate": 60
  }
}
```

### 5.5 명령 메시지 (Command Messages)

#### 5.5.1 command

센서에 명령을 전송합니다.

```json
{
  "protocol": "wia-aac",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440030",
  "timestamp": 1702483200300,
  "type": "command",
  "payload": {
    "command": "calibrate",
    "target": "eye_tracker",
    "parameters": {
      "points": 5,
      "mode": "automatic"
    }
  }
}
```

**지원 명령:**

| 명령 | 대상 | 설명 |
|-----|-----|------|
| `calibrate` | 센서 | 센서 캘리브레이션 |
| `configure` | 센서 | 센서 설정 변경 |
| `reset` | 센서 | 센서 리셋 |
| `get_status` | 센서 | 상태 조회 |
| `get_config` | 센서 | 설정 조회 |

#### 5.5.2 command_ack

```json
{
  "protocol": "wia-aac",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440031",
  "timestamp": 1702483200350,
  "type": "command_ack",
  "payload": {
    "success": true,
    "command": "calibrate",
    "result": {
      "accuracy": 0.95,
      "message": "Calibration completed successfully"
    }
  }
}
```

### 5.6 에러 메시지 (Error Message)

```json
{
  "protocol": "wia-aac",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440099",
  "timestamp": 1702483200400,
  "type": "error",
  "payload": {
    "code": 2001,
    "name": "SENSOR_NOT_FOUND",
    "message": "The requested sensor is not connected",
    "recoverable": true,
    "details": {
      "requestedType": "eye_tracker",
      "availableTypes": ["switch"]
    },
    "relatedMessageId": "550e8400-e29b-41d4-a716-446655440020"
  }
}
```

### 5.7 하트비트 메시지 (Heartbeat Messages)

#### 5.7.1 ping

```json
{
  "protocol": "wia-aac",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440050",
  "timestamp": 1702483230000,
  "type": "ping",
  "payload": {
    "sequence": 1
  }
}
```

#### 5.7.2 pong

```json
{
  "protocol": "wia-aac",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440051",
  "timestamp": 1702483230005,
  "type": "pong",
  "payload": {
    "sequence": 1,
    "latency_ms": 5
  }
}
```

---

## 6. 연결 관리 (Connection Management)

### 6.1 연결 상태 (Connection States)

```
                          ┌─────────────────┐
                          │  DISCONNECTED   │
                          └────────┬────────┘
                                   │ connect()
                                   ▼
                          ┌─────────────────┐
                          │   CONNECTING    │
                          └────────┬────────┘
                                   │ connect_ack (success)
                    ┌──────────────┼──────────────┐
                    │              ▼              │
                    │     ┌─────────────────┐     │
         error/     │     │   CONNECTED     │     │ connect_ack
         timeout    │     └────────┬────────┘     │ (failure)
                    │              │              │
                    │              │ error/       │
                    │              │ disconnect   │
                    │              ▼              │
                    │     ┌─────────────────┐     │
                    │     │  RECONNECTING   │     │
                    │     └────────┬────────┘     │
                    │              │              │
                    │   success    │   max retry  │
                    │   ┌──────────┘   exceeded   │
                    │   │              │          │
                    │   │              ▼          │
                    │   │     ┌─────────────────┐ │
                    │   │     │     ERROR       │ │
                    │   │     └────────┬────────┘ │
                    │   │              │          │
                    └───┼──────────────┼──────────┘
                        │              │
                        │              ▼
                        │     ┌─────────────────┐
                        └────►│  DISCONNECTED   │
                              └─────────────────┘
```

### 6.2 연결 시퀀스 (Connection Sequence)

```
Client                                           Server
   │                                                │
   │ ─────────── TCP/WebSocket Connect ──────────► │
   │                                                │
   │ ◄─────────── Connection Established ───────── │
   │                                                │
   │ ─────────────── connect ───────────────────► │
   │                                                │
   │ ◄────────────── connect_ack ────────────────  │
   │                                                │
   │ ─────────────── subscribe ─────────────────► │
   │                                                │
   │ ◄────────────── subscribe_ack ──────────────  │
   │                                                │
   │ ◄──────────────── signal ───────────────────  │
   │ ◄──────────────── signal ───────────────────  │
   │ ◄──────────────── signal ───────────────────  │
   │                  ...                           │
```

### 6.3 재연결 정책 (Reconnection Policy)

| 설정 | 기본값 | 설명 |
|-----|-------|------|
| `autoReconnect` | true | 자동 재연결 활성화 |
| `reconnectInterval` | 1000ms | 재연결 시도 간격 |
| `maxReconnectAttempts` | 5 | 최대 재연결 시도 |
| `reconnectBackoff` | exponential | 백오프 전략 |
| `maxReconnectInterval` | 30000ms | 최대 재연결 간격 |

**지수 백오프 (Exponential Backoff):**
```
attempt 1: 1000ms
attempt 2: 2000ms
attempt 3: 4000ms
attempt 4: 8000ms
attempt 5: 16000ms (max 30000ms)
```

### 6.4 하트비트 (Heartbeat)

- 기본 간격: 30초
- 타임아웃: 10초 (응답 없음 시 연결 끊김 처리)
- 3회 연속 실패 시 재연결 시도

---

## 7. 에러 처리 (Error Handling)

### 7.1 에러 코드 (Error Codes)

#### 연결 에러 (1xxx)

| 코드 | 이름 | 설명 |
|-----|------|------|
| 1000 | `CONNECTION_CLOSED` | 정상 연결 종료 |
| 1001 | `CONNECTION_LOST` | 연결 끊김 |
| 1002 | `CONNECTION_TIMEOUT` | 연결 시간 초과 |
| 1003 | `PROTOCOL_ERROR` | 프로토콜 오류 |
| 1004 | `VERSION_MISMATCH` | 버전 불일치 |
| 1005 | `INVALID_MESSAGE` | 잘못된 메시지 형식 |

#### 센서 에러 (2xxx)

| 코드 | 이름 | 설명 |
|-----|------|------|
| 2001 | `SENSOR_NOT_FOUND` | 센서 없음 |
| 2002 | `SENSOR_BUSY` | 센서 사용 중 |
| 2003 | `SENSOR_ERROR` | 센서 내부 오류 |
| 2004 | `SENSOR_DISCONNECTED` | 센서 연결 끊김 |
| 2005 | `CALIBRATION_REQUIRED` | 캘리브레이션 필요 |
| 2006 | `CALIBRATION_FAILED` | 캘리브레이션 실패 |

#### 인증 에러 (3xxx)

| 코드 | 이름 | 설명 |
|-----|------|------|
| 3001 | `AUTH_REQUIRED` | 인증 필요 |
| 3002 | `AUTH_FAILED` | 인증 실패 |
| 3003 | `PERMISSION_DENIED` | 권한 없음 |
| 3004 | `SESSION_EXPIRED` | 세션 만료 |

#### 구독 에러 (4xxx)

| 코드 | 이름 | 설명 |
|-----|------|------|
| 4001 | `SUBSCRIPTION_FAILED` | 구독 실패 |
| 4002 | `STREAM_NOT_AVAILABLE` | 스트림 없음 |
| 4003 | `RATE_LIMIT_EXCEEDED` | 속도 제한 초과 |

### 7.2 에러 복구 (Error Recovery)

| recoverable | 처리 방식 |
|-------------|----------|
| `true` | 재시도 또는 대체 동작 가능 |
| `false` | 사용자 개입 필요 |

---

## 8. 전송 계층 (Transport Layer)

### 8.1 WebSocket (Primary)

**연결 URL 형식:**
```
ws://host:port/wia-aac
wss://host:port/wia-aac  (TLS)
```

**기본 포트:** 8765

**서브프로토콜:** `wia-aac-v1`

**예시:**
```javascript
const ws = new WebSocket('ws://localhost:8765/wia-aac', 'wia-aac-v1');
```

### 8.2 USB HID (Secondary)

USB HID를 통한 통신 시 메시지를 청크로 분할:

```
┌────────────────────────────────────────┐
│ HID Report                              │
├────────────────────────────────────────┤
│ Report ID (1 byte)                     │
│ Sequence (1 byte)                      │
│ Flags (1 byte): [FIRST|LAST|MORE]      │
│ Length (2 bytes)                       │
│ Data (59 bytes max)                    │
└────────────────────────────────────────┘
```

### 8.3 Bluetooth LE (Tertiary)

GATT 서비스 UUID: `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`

특성 (Characteristics):
- TX: `6E400002-B5A3-F393-E0A9-E50E24DCCA9E` (Write)
- RX: `6E400003-B5A3-F393-E0A9-E50E24DCCA9E` (Notify)

### 8.4 전송 추상화 (Transport Abstraction)

```typescript
interface ITransport {
  // 연결 관리
  connect(url: string): Promise<void>;
  disconnect(): Promise<void>;
  isConnected(): boolean;

  // 메시지 전송/수신
  send(message: WiaAacMessage): Promise<void>;
  onMessage(handler: (message: WiaAacMessage) => void): void;

  // 이벤트
  onOpen(handler: () => void): void;
  onClose(handler: (reason: string) => void): void;
  onError(handler: (error: Error) => void): void;
}
```

---

## 9. 보안 (Security)

### 9.1 전송 보안

- **WebSocket**: WSS (TLS 1.2+) 권장
- **USB**: OS 수준 권한 관리
- **Bluetooth**: BLE 페어링 및 암호화

### 9.2 인증 (Authentication)

선택적 토큰 기반 인증:

```json
{
  "type": "connect",
  "payload": {
    "clientId": "my-app",
    "authToken": "eyJhbGciOiJIUzI1NiIs..."
  }
}
```

### 9.3 권한 (Authorization)

센서별 접근 권한 제어:

```json
{
  "type": "connect_ack",
  "payload": {
    "permissions": {
      "eye_tracker": ["read", "calibrate"],
      "switch": ["read"],
      "brain_interface": ["read", "configure"]
    }
  }
}
```

---

## 10. 예제 (Examples)

### 10.1 전체 연결 플로우 (TypeScript)

```typescript
import { WiaAac, WebSocketTransport } from 'wia-aac';

async function main() {
  const aac = new WiaAac();
  const transport = new WebSocketTransport();

  // 전송 계층 연결
  await transport.connect('ws://localhost:8765/wia-aac');

  // 프로토콜 연결
  await aac.connect({
    transport,
    clientId: 'my-app',
    capabilities: ['eye_tracker', 'switch']
  });

  // 신호 구독
  await aac.subscribe({
    streams: ['eye_tracker'],
    signalRate: 60
  });

  // 신호 수신
  aac.on('signal', (signal) => {
    console.log('Gaze:', signal.data.gaze_point);
  });

  // 종료
  await aac.disconnect();
}
```

### 10.2 전체 연결 플로우 (Python)

```python
import asyncio
from wia_aac import WiaAac, WebSocketTransport

async def main():
    aac = WiaAac()
    transport = WebSocketTransport()

    # 전송 계층 연결
    await transport.connect('ws://localhost:8765/wia-aac')

    # 프로토콜 연결
    await aac.connect(
        transport=transport,
        client_id='my-app',
        capabilities=['eye_tracker', 'switch']
    )

    # 신호 구독
    await aac.subscribe(
        streams=['eye_tracker'],
        signal_rate=60
    )

    # 신호 수신
    def on_signal(signal):
        print('Gaze:', signal.data.gaze_point)

    aac.on('signal', on_signal)

    # 실행
    await asyncio.sleep(10)

    # 종료
    await aac.disconnect()

asyncio.run(main())
```

---

## 11. 버전 관리 (Versioning)

### 11.1 프로토콜 버전

- SemVer 형식: `MAJOR.MINOR.PATCH`
- `MAJOR`: 호환성 깨지는 변경
- `MINOR`: 하위 호환 기능 추가
- `PATCH`: 버그 수정

### 11.2 버전 협상

1. 클라이언트가 지원 버전 전송
2. 서버가 호환 버전 확인
3. 불일치 시 `VERSION_MISMATCH` 에러

---

## 12. 참고문헌 (References)

1. RFC 6455 - The WebSocket Protocol
2. USB HID Specification 1.11
3. Bluetooth HOGP 1.0
4. JSON Schema draft-07
5. WIA AAC Standard Phase 1 - Signal Format
6. WIA AAC Standard Phase 2 - API Interface

---

<div align="center">

**WIA AAC Standard - Phase 3**

Communication Protocol v1.0.0

---

**弘益人間** - 널리 인간을 이롭게

</div>
