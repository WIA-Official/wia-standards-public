# Phase 3: Communication Protocol
## Claude Code 작업 프롬프트

---

**Standard**: WIA Space
**Phase**: 3 of 4
**목표**: 우주 기술 시스템 간 통신 프로토콜 표준화
**난이도**: ★★★★☆
**예상 작업량**: 스펙 문서 1개 + Protocol 구현 + 예제

---

## 🎯 Phase 3 목표

### 핵심 질문
```
"Phase 1에서 Data Format을 정의하고,
 Phase 2에서 API Interface를 만들었다.

 이제 우주 시스템들이 실제로 어떻게 통신할 것인가?

 - 지구-화성 간 통신 지연 (4~24분)?
 - 심우주 탐사 미션의 데이터 전송?
 - 위성 간 통신 (Inter-satellite link)?

 모든 통신 방식에서 동일한 메시지 형식을 사용할 수 있을까?"
```

### 목표
```
우주 기술 시스템 간 통신을 위한
WIA Space Protocol (WSP)을 정의한다.

- 메시지 형식 (Message Format)
- 연결 관리 (Connection Management)
- 에러 처리 (Error Handling)
- 지연 허용 네트워킹 (Delay-Tolerant Networking)
- 다중 전송 방식 지원 (TCP, UDP, DTN)
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

### 1단계: 우주 통신 프로토콜 조사

| 프로토콜 | 조사 대상 | 웹서치 키워드 |
|---------|----------|--------------|
| **CCSDS** | 국제 우주 데이터 시스템 | "CCSDS space communication protocol" |
| **DSN** | NASA 심우주 네트워크 | "NASA Deep Space Network protocol" |
| **DTN** | 지연 허용 네트워킹 | "Delay Tolerant Networking space" |
| **ISL** | 위성간 링크 | "Inter-satellite link protocol Starlink" |

### 2단계: 기존 우주 미션 통신 방식 조사

| 미션/시스템 | 조사 내용 | 웹서치 키워드 |
|------------|----------|--------------|
| **Mars Rover** | 화성 탐사선 통신 | "Mars rover communication delay protocol" |
| **Starlink** | 위성 메시 네트워크 | "Starlink inter-satellite laser link" |
| **Artemis** | 달 탐사 통신 | "Artemis lunar communication system" |
| **Voyager** | 심우주 통신 | "Voyager deep space communication" |

### 3단계: 조사 결과 정리

조사 후 `/spec/RESEARCH-PHASE-3.md`에 다음을 정리:

```markdown
# Phase 3 사전 조사 결과

## 1. 우주 통신 프로토콜 비교

### CCSDS (Consultative Committee for Space Data Systems)
- 개요: [조사 내용]
- 특징: [조사 내용]
- WIA Space 적용: [분석]

### DTN (Delay-Tolerant Networking)
- 개요: [조사 내용]
- Bundle Protocol: [조사 내용]
- WIA Space 적용: [분석]

## 2. 기존 우주 미션 통신 분석

### Mars Communication
- 통신 지연: [조사 내용]
- 프로토콜: [조사 내용]

### Satellite Networks
- 통신 방식: [조사 내용]
- 메시지 형식: [조사 내용]

## 3. 결론
- 권장 프로토콜 아키텍처: [제안]
- 메시지 형식 설계 방향: [제안]
```

---

## 🏗️ 프로토콜 설계

### 1. 메시지 형식 (Message Format)

#### 기본 메시지 구조
```json
{
  "protocol": "wia-space",
  "version": "1.0.0",
  "messageId": "uuid-v4",
  "timestamp": 1702483200000,
  "type": "메시지 유형",
  "source": {
    "id": "송신자 ID",
    "type": "endpoint 유형",
    "location": "좌표/위치"
  },
  "destination": {
    "id": "수신자 ID",
    "type": "endpoint 유형"
  },
  "priority": "normal|high|critical",
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
| `telemetry` | Server → Client | 텔레메트리 데이터 |
| `command` | Client → Server | 명령 전송 |
| `command_ack` | Server → Client | 명령 응답 |
| `data` | Both | 일반 데이터 전송 |
| `error` | Both | 에러 메시지 |
| `ping` | Client → Server | 연결 확인 |
| `pong` | Server → Client | 연결 확인 응답 |

#### 엔드포인트 유형 (Endpoint Types)

| Type | 설명 | 예시 |
|------|------|------|
| `ground_station` | 지상국 | NASA DSN, ESA ESTRACK |
| `spacecraft` | 우주선 | Mars Rover, Satellite |
| `satellite` | 인공위성 | Communication Relay |
| `habitat` | 우주 기지 | ISS, Lunar Gateway |
| `simulation` | 시뮬레이션 | 테스트 환경 |

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

| 코드 범위 | 카테고리 | 설명 |
|----------|---------|------|
| `1xxx` | Connection | 연결 관련 에러 |
| `2xxx` | Protocol | 프로토콜 오류 |
| `3xxx` | Transmission | 전송 오류 |
| `4xxx` | Payload | 데이터 오류 |
| `5xxx` | System | 시스템 오류 |

### 4. 전송 계층 어댑터 (Transport Adapters)

각 전송 방식에 대한 추상화 계층:

```rust
#[async_trait]
pub trait Transport: Send + Sync {
    fn transport_type(&self) -> TransportType;
    async fn connect(&mut self, config: &TransportConfig) -> Result<(), TransportError>;
    async fn disconnect(&mut self) -> Result<(), TransportError>;
    async fn send(&self, message: &WspMessage) -> Result<(), TransportError>;
    async fn receive(&self) -> Result<WspMessage, TransportError>;
    fn is_connected(&self) -> bool;
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
   - TCP/IP
   - DTN (Bundle Protocol)
   - Mock (테스트용)
8. 지연 시뮬레이션 (Latency Simulation)
9. 보안 (Security)
10. 예제 (Examples)
11. 참고문헌 (References)
```

### 3. JSON Schema
```
/spec/schemas/
├── wsp-message.schema.json    # 프로토콜 메시지 스키마
└── wsp-error.schema.json      # 에러 메시지 스키마
```

### 4. Rust Protocol 구현
```
/api/rust/src/
├── protocol/
│   ├── mod.rs
│   ├── message.rs           # 메시지 타입 정의
│   ├── builder.rs           # 메시지 생성
│   ├── handler.rs           # 프로토콜 처리
│   └── error.rs             # 에러 타입
├── transport/
│   ├── mod.rs
│   ├── base.rs              # 전송 인터페이스
│   ├── mock.rs              # 테스트용
│   └── latency.rs           # 지연 시뮬레이션
└── ...
```

### 5. 예제 코드
```
/api/rust/examples/
├── protocol_demo.rs         # 프로토콜 데모
└── latency_simulation.rs    # 지연 시뮬레이션 예제
```

---

## ✅ 완료 체크리스트

Phase 3 완료 전 확인:

```
□ 웹서치로 우주 통신 프로토콜 조사 완료
□ /spec/RESEARCH-PHASE-3.md 작성 완료
□ /spec/PHASE-3-PROTOCOL.md 작성 완료
□ 메시지 형식 JSON Schema 정의 완료
□ Rust protocol 모듈 구현 완료
□ Rust transport 모듈 구현 완료
□ MockTransport 테스트용 구현 완료
□ 지연 시뮬레이션 구현 완료
□ 단위 테스트 작성 완료
□ 테스트 통과
□ 예제 코드 완료
□ README 업데이트 (Phase 3 완료 표시)
```

---

## 🔄 작업 순서

```
1. 웹서치로 우주 통신 프로토콜 조사
   ↓
2. /spec/RESEARCH-PHASE-3.md 작성
   ↓
3. 프로토콜 설계
   ↓
4. /spec/PHASE-3-PROTOCOL.md 작성
   ↓
5. 메시지 형식 JSON Schema 작성
   ↓
6. Rust protocol 모듈 구현
   ↓
7. Rust transport 모듈 구현 (Mock)
   ↓
8. 지연 시뮬레이션 구현
   ↓
9. 테스트 작성 및 실행
   ↓
10. 예제 코드 작성
   ↓
11. 완료 체크리스트 확인
   ↓
12. Phase 4 시작 가능
```

---

## 💡 설계 가이드라인

### DO (해야 할 것)

```
✅ Phase 1 Data Format을 메시지 payload로 사용
✅ Phase 2 Rust API와 연동 가능하도록 설계
✅ 전송 계층 추상화 (다양한 전송 방식 지원)
✅ 지연 허용 설계 (DTN 개념 적용)
✅ 메시지 순서 보장 (sequence number)
✅ 하트비트 (ping/pong) 포함
✅ 지연 시뮬레이션 기능 포함
```

### DON'T (하지 말 것)

```
❌ 특정 전송 방식에만 종속되는 설계
❌ 실시간 통신만 고려 (우주는 지연이 기본)
❌ Phase 1/2 형식과 불일치
❌ 에러 처리 없는 happy path만 구현
```

---

## 🚀 작업 시작

이제 Phase 3 작업을 시작하세요.

첫 번째 단계: **웹서치로 우주 통신 프로토콜 조사**

```
검색 키워드: "CCSDS space communication protocol specification"
```

화이팅! 🚀

---

<div align="center">

**Phase 3 of 4**

WIA Space Protocol (WSP)

Communication for the Stars 🌟

</div>
