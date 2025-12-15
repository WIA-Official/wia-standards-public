# Phase 3: Communication Protocol
## Claude Code 작업 프롬프트

---

**Standard**: WIA Climate
**Phase**: 3 of 4
**목표**: Climate 센서/시스템 간 통신 프로토콜 표준화
**난이도**: ★★★★☆

---

## 🎯 Phase 3 목표

### 핵심 질문
```
"Phase 1에서 Data Format을 정의하고,
 Phase 2에서 Rust API를 만들었다.

 이제 기후 센서와 시스템이 실제로 어떻게 통신할 것인가?
 WebSocket? MQTT? HTTP? Serial?

 모든 통신 방식에서 동일한 메시지 형식을 사용할 수 있을까?"
```

### 목표
```
기후/환경 센서와 시스템 간 통신을 위한
전송 계층(Transport Layer) 프로토콜을 정의한다.

- 메시지 형식 (Message Format)
- 연결 관리 (Connection Management)
- 에러 처리 (Error Handling)
- 재연결 (Reconnection)
- 다중 전송 방식 지원 (WebSocket, MQTT, HTTP, Serial)
```

---

## 📋 Phase 1 & 2 결과물 활용

| 이전 Phase 산출물 | Phase 3 활용 |
|-----------------|-------------|
| Phase 1: Data Format | 메시지 페이로드 (payload) |
| Phase 2: Rust API | 메시지 핸들러 연동 |
| JSON Schema | 메시지 검증 |

---

## 📋 사전 조사 (웹서치 필수)

### 1단계: 통신 프로토콜 조사

| 프로토콜 | 조사 대상 | 웹서치 키워드 |
|---------|----------|--------------|
| **WebSocket** | 실시간 양방향 통신 | "WebSocket protocol RFC 6455" |
| **MQTT** | IoT 센서 통신 | "MQTT protocol IoT climate sensor" |
| **HTTP/REST** | 요청/응답 방식 | "REST API climate data exchange" |
| **OPC UA** | 산업 자동화 | "OPC UA climate monitoring" |

### 2단계: 기존 Climate 시스템 통신 방식 조사

| 시스템/프로토콜 | 조사 내용 | 웹서치 키워드 |
|---------------|----------|--------------|
| **OGC SensorThings API** | IoT 센서 표준 | "OGC SensorThings API climate" |
| **CMIP6/ESGF** | 기후 모델 데이터 | "CMIP6 data access protocol" |
| **WMO BUFR/GRIB** | 기상 데이터 | "WMO data exchange protocol" |
| **OpenBCI** | 센서 스트리밍 | "OpenBCI streaming protocol" |

---

## 🏗️ 프로토콜 설계

### 1. 메시지 형식 (Message Format)

```json
{
  "protocol": "wia-climate",
  "version": "1.0.0",
  "messageId": "uuid-v4",
  "timestamp": 1702483200000,
  "type": "메시지 유형",
  "payload": {
    "메시지 데이터 (Phase 1 형식)"
  }
}
```

### 2. 메시지 유형 (Message Types)

| Type | 방향 | 설명 |
|------|-----|------|
| `connect` | Client → Server | 연결 요청 |
| `connect_ack` | Server → Client | 연결 응답 |
| `disconnect` | Both | 연결 종료 |
| `data` | Server → Client | 센서 데이터 (Phase 1 형식) |
| `command` | Client → Server | 명령 전송 |
| `command_ack` | Server → Client | 명령 응답 |
| `subscribe` | Client → Server | 데이터 구독 |
| `unsubscribe` | Client → Server | 구독 해제 |
| `error` | Both | 에러 메시지 |
| `ping` | Client → Server | 연결 확인 |
| `pong` | Server → Client | 연결 확인 응답 |

---

## 📁 산출물 목록

### 1. 조사 문서
```
/spec/RESEARCH-PHASE-3.md
```

### 2. 표준 스펙 문서
```
/spec/PHASE-3-PROTOCOL.md
```

### 3. Rust Protocol 구현
```
/api/rust/src/
├── protocol/
│   ├── mod.rs
│   ├── message.rs
│   ├── message_builder.rs
│   └── handler.rs
└── transport/
    ├── mod.rs
    ├── websocket.rs
    └── mock.rs
```

### 4. 예제 코드
```
/api/rust/examples/
├── websocket_client.rs
└── websocket_server.rs
```

---

## ✅ 완료 체크리스트

```
□ 웹서치로 통신 프로토콜 조사 완료
□ /spec/RESEARCH-PHASE-3.md 작성 완료
□ /spec/PHASE-3-PROTOCOL.md 작성 완료
□ 메시지 형식 JSON Schema 정의 완료
□ Rust protocol 구현 완료
□ Rust transport (WebSocket) 구현 완료
□ 테스트 작성 및 통과
□ 예제 코드 작성 완료
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
6. Rust protocol 구현
   ↓
7. Rust transport 구현 (WebSocket)
   ↓
8. 테스트 작성 및 실행
   ↓
9. 예제 코드 작성
   ↓
10. 완료 체크리스트 확인
```

---

弘益人間 🤟🌍
