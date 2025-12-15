# Phase 3: Communication Protocol
## Claude Code 작업 프롬프트

---

**Phase**: 3 of 4
**목표**: AAC 센서와 소프트웨어 간 통신 프로토콜 표준화
**난이도**: ★★★★☆
**예상 작업량**: 스펙 문서 1개 + Protocol 구현 + 예제

---

## 🎯 Phase 3 목표

### 핵심 질문
```
"Phase 1에서 Signal Format을 정의하고,
 Phase 2에서 API Interface를 만들었다.

 이제 센서와 소프트웨어가 실제로 어떻게 통신할 것인가?
 WebSocket? USB? Bluetooth? Serial?

 모든 통신 방식에서 동일한 메시지 형식을 사용할 수 있을까?"
```

### 목표
```
AAC 센서와 소프트웨어 간 통신을 위한
전송 계층(Transport Layer) 프로토콜을 정의한다.

- 메시지 형식 (Message Format)
- 연결 관리 (Connection Management)
- 에러 처리 (Error Handling)
- 재연결 (Reconnection)
- 다중 전송 방식 지원 (WebSocket, USB, Bluetooth, Serial)
```

---

## 📋 Phase 1 & 2 결과물 활용

| 이전 Phase 산출물 | Phase 3 활용 |
|-----------------|-------------|
| Phase 1: Signal Format | 메시지 페이로드 (payload) |
| Phase 2: API Interface | 메시지 핸들러 연동 |
| JSON Schema | 메시지 검증 |

---

## 📋 사전 조사 (웹서치 필수)

### 1단계: 통신 프로토콜 조사

| 프로토콜 | 조사 대상 | 웹서치 키워드 |
|---------|----------|--------------|
| **WebSocket** | 실시간 양방향 통신 | "WebSocket protocol RFC 6455" |
| **USB HID** | 하드웨어 장치 통신 | "USB HID protocol specification" |
| **Bluetooth LE** | 저전력 무선 통신 | "Bluetooth Low Energy GATT profile" |
| **Serial** | 레거시 장치 | "serial port communication protocol" |

### 2단계: 기존 AAC 통신 방식 조사

| 제품/프로젝트 | 조사 내용 | 웹서치 키워드 |
|-------------|----------|--------------|
| **Tobii SDK** | 통신 방식 | "Tobii eye tracker communication protocol" |
| **OpenBCI** | 데이터 스트리밍 | "OpenBCI data streaming protocol" |
| **Intel ACAT** | 센서 연결 | "Intel ACAT sensor communication" |
| **Emotiv** | BCI 통신 | "Emotiv headset protocol" |

### 3단계: 조사 결과 정리

조사 후 `/spec/RESEARCH-PHASE-3.md`에 다음을 정리:

```markdown
# Phase 3 사전 조사 결과

## 1. 전송 프로토콜 비교

### WebSocket
- 장점: [조사 내용]
- 단점: [조사 내용]
- AAC 적용: [분석]

### USB HID
- 장점: [조사 내용]
- 단점: [조사 내용]
- AAC 적용: [분석]

### Bluetooth LE
- 장점: [조사 내용]
- 단점: [조사 내용]
- AAC 적용: [분석]

## 2. 기존 AAC 제품 통신 방식

### Tobii
- 통신 방식: [조사 내용]
- 메시지 형식: [조사 내용]

### OpenBCI
- 통신 방식: [조사 내용]
- 메시지 형식: [조사 내용]

## 3. 결론
- 권장 통신 방식: [제안]
- 메시지 프로토콜 설계 방향: [제안]
```

---

## 🏗️ 프로토콜 설계

### 1. 메시지 형식 (Message Format)

#### 기본 메시지 구조
```json
{
  "protocol": "wia-aac",
  "version": "1.0.0",
  "messageId": "uuid-v4",
  "timestamp": 1702483200000,
  "type": "메시지 유형",
  "payload": {
    "메시지 데이터"
  }
}
```

#### 메시지 유형 (Message Types)

| Type | 방향 | 설명 |
|------|-----|------|
| `connect` | Client → Server | 연결 요청 |
| `connect_ack` | Server → Client | 연결 응답 |
| `disconnect` | Both | 연결 종료 |
| `signal` | Server → Client | 센서 신호 (Phase 1 형식) |
| `command` | Client → Server | 명령 전송 |
| `command_ack` | Server → Client | 명령 응답 |
| `error` | Both | 에러 메시지 |
| `ping` | Client → Server | 연결 확인 |
| `pong` | Server → Client | 연결 확인 응답 |

#### 연결 메시지 예시
```json
{
  "protocol": "wia-aac",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": 1702483200000,
  "type": "connect",
  "payload": {
    "clientId": "app-12345",
    "clientName": "My AAC App",
    "capabilities": ["eye_tracker", "switch"],
    "options": {
      "signalRate": 60,
      "compression": false
    }
  }
}
```

#### 신호 메시지 예시 (Phase 1 Signal을 payload로)
```json
{
  "protocol": "wia-aac",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440001",
  "timestamp": 1702483200100,
  "type": "signal",
  "payload": {
    "$schema": "https://wia.live/aac/signal/v1/schema.json",
    "version": "1.0.0",
    "type": "eye_tracker",
    "timestamp": {...},
    "data": {...}
  }
}
```

### 2. 연결 상태 관리 (Connection State Machine)

```
┌─────────────┐
│ DISCONNECTED│
└──────┬──────┘
       │ connect()
       ▼
┌─────────────┐
│ CONNECTING  │
└──────┬──────┘
       │ connect_ack received
       ▼
┌─────────────┐
│  CONNECTED  │◄──────┐
└──────┬──────┘       │
       │              │ reconnect
       │ error/       │
       │ disconnect   │
       ▼              │
┌─────────────┐       │
│ RECONNECTING├───────┘
└──────┬──────┘
       │ max retries exceeded
       ▼
┌─────────────┐
│   ERROR     │
└─────────────┘
```

### 3. 에러 코드 (Error Codes)

| 코드 | 이름 | 설명 |
|-----|------|------|
| `1000` | `CONNECTION_CLOSED` | 정상 종료 |
| `1001` | `CONNECTION_LOST` | 연결 끊김 |
| `1002` | `PROTOCOL_ERROR` | 프로토콜 오류 |
| `1003` | `UNSUPPORTED_TYPE` | 지원하지 않는 메시지 유형 |
| `2001` | `SENSOR_NOT_FOUND` | 센서 없음 |
| `2002` | `SENSOR_BUSY` | 센서 사용 중 |
| `2003` | `SENSOR_ERROR` | 센서 오류 |
| `3001` | `AUTH_FAILED` | 인증 실패 |
| `3002` | `PERMISSION_DENIED` | 권한 없음 |

### 4. 전송 계층 어댑터 (Transport Adapters)

각 전송 방식에 대한 추상화 계층:

```typescript
interface ITransportAdapter {
  connect(url: string): Promise<void>;
  disconnect(): Promise<void>;
  send(message: WiaAacMessage): Promise<void>;
  onMessage(handler: MessageHandler): void;
  onError(handler: ErrorHandler): void;
  onClose(handler: CloseHandler): void;
  isConnected(): boolean;
}
```

---

## 📁 산출물 목록

Phase 3 완료 시 다음 파일을 생성해야 합니다:

### 1. 조사 문서
```
/spec/RESEARCH-PHASE-3.md
```

### 2. 표준 스펙 문서
```
/spec/PHASE-3-PROTOCOL.md

내용:
1. 개요 (Overview)
2. 용어 정의 (Terminology)
3. 메시지 형식 (Message Format)
4. 메시지 유형 (Message Types)
5. 연결 관리 (Connection Management)
6. 에러 처리 (Error Handling)
7. 전송 계층 (Transport Layer)
   - WebSocket
   - USB HID
   - Bluetooth LE
   - Serial
8. 보안 (Security)
9. 예제 (Examples)
10. 참고문헌 (References)
```

### 3. TypeScript Protocol 구현
```
/api/typescript/src/
├── protocol/
│   ├── index.ts
│   ├── message.ts           # 메시지 타입 정의
│   ├── MessageBuilder.ts    # 메시지 생성
│   ├── MessageParser.ts     # 메시지 파싱
│   └── ProtocolHandler.ts   # 프로토콜 처리
├── transport/
│   ├── index.ts
│   ├── ITransport.ts        # 전송 인터페이스
│   ├── WebSocketTransport.ts
│   ├── MockTransport.ts     # 테스트용
│   └── TransportFactory.ts
└── ...
```

### 4. Python Protocol 구현
```
/api/python/wia_aac/
├── protocol/
│   ├── __init__.py
│   ├── message.py
│   ├── message_builder.py
│   ├── message_parser.py
│   └── protocol_handler.py
├── transport/
│   ├── __init__.py
│   ├── base_transport.py
│   ├── websocket_transport.py
│   ├── mock_transport.py
│   └── transport_factory.py
└── ...
```

### 5. 예제 코드
```
/examples/protocol/
├── typescript/
│   ├── websocket-client.ts
│   ├── websocket-server.ts
│   └── mock-sensor.ts
└── python/
    ├── websocket_client.py
    ├── websocket_server.py
    └── mock_sensor.py
```

---

## ✅ 완료 체크리스트

Phase 3 완료 전 확인:

```
□ 웹서치로 통신 프로토콜 조사 완료
□ /spec/RESEARCH-PHASE-3.md 작성 완료
□ /spec/PHASE-3-PROTOCOL.md 작성 완료
□ 메시지 형식 JSON Schema 정의 완료
□ TypeScript protocol/transport 구현 완료
□ Python protocol/transport 구현 완료
□ WebSocket 구현 완료
□ 단위 테스트 작성 완료
□ 테스트 통과
□ WebSocket 클라이언트/서버 예제 완료
□ README 업데이트 (Phase 3 완료 표시)
```

---

## 🔄 작업 순서

```
1. 웹서치로 통신 프로토콜 조사
   ↓
2. /spec/RESEARCH-PHASE-3.md 작성
   ↓
3. 프로토콜 설계
   ↓
4. /spec/PHASE-3-PROTOCOL.md 작성
   ↓
5. 메시지 형식 JSON Schema 작성
   ↓
6. TypeScript protocol 구현
   ↓
7. TypeScript transport 구현 (WebSocket)
   ↓
8. Python protocol 구현
   ↓
9. Python transport 구현 (WebSocket)
   ↓
10. 테스트 작성 및 실행
   ↓
11. 예제 코드 작성
   ↓
12. 완료 체크리스트 확인
   ↓
13. Phase 4 시작 가능
```

---

## 💡 설계 가이드라인

### DO (해야 할 것)

```
✅ Phase 1 Signal Format을 메시지 payload로 사용
✅ Phase 2 API와 연동 가능하도록 설계
✅ 전송 계층 추상화 (다양한 전송 방식 지원)
✅ 재연결 로직 포함
✅ 메시지 순서 보장 (sequence number)
✅ 하트비트 (ping/pong) 포함
```

### DON'T (하지 말 것)

```
❌ 특정 전송 방식에만 종속되는 설계
❌ 바이너리 전용 프로토콜 (JSON 기반 유지)
❌ Phase 1/2 형식과 불일치
❌ 에러 처리 없는 happy path만 구현
```

---

## 🚀 작업 시작

이제 Phase 3 작업을 시작하세요.

첫 번째 단계: **웹서치로 AAC 센서 통신 프로토콜 조사**

```
검색 키워드: "Tobii eye tracker SDK communication protocol WebSocket"
```

화이팅! 🤟

---

<div align="center">

**Phase 3 of 4**

Communication Protocol

</div>
