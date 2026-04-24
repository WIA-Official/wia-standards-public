# Phase 3: Protocol Specification
## Claude Code 작업 프롬프트

---

**Standard**: WIA Auto
**Phase**: 3 of 4
**목표**: 통신 프로토콜 및 메시지 플로우 표준화

---

## 🎯 목표

시스템 간 통신 프로토콜을 정의하고 구현합니다.

---

## 🔗 통신 아키텍처

```
┌─────────────────┐      ┌─────────────────┐
│  Passenger App  │ ←──► │  Fleet Server   │
└────────┬────────┘      └────────┬────────┘
         │                        │
         │    WebSocket/gRPC      │
         │                        │
         └───────────┬────────────┘
                     │
              ┌──────▼──────┐
              │   Vehicle   │
              │   System    │
              └──────┬──────┘
                     │
         ┌───────────┴───────────┐
         │                       │
   ┌─────▼─────┐          ┌─────▼─────┐
   │  Support  │          │ Emergency │
   │  Center   │          │ Services  │
   └───────────┘          └───────────┘
```

---

## 📡 프로토콜 정의

### REST API
```
POST   /api/v1/trips           # 여행 요청
GET    /api/v1/trips/{id}      # 여행 상태 조회
DELETE /api/v1/trips/{id}      # 여행 취소
POST   /api/v1/profiles        # 프로필 생성
GET    /api/v1/profiles/{id}   # 프로필 조회
GET    /api/v1/vehicles        # 차량 목록
POST   /api/v1/emergency       # 긴급 상황 보고
```

### WebSocket Events
```
trip.status.updated      # 여행 상태 변경
vehicle.location.updated # 차량 위치 업데이트
vehicle.eta.updated      # 도착 예정 시간 변경
securement.status.changed # 고정장치 상태 변경
emergency.alert          # 긴급 알림
hmi.command              # HMI 명령
```

### gRPC Services (실시간)
```protobuf
service VehicleControl {
  rpc StreamLocation(Empty) returns (stream Location);
  rpc StreamSecurement(Empty) returns (stream SecurementStatus);
  rpc SendHmiCommand(HmiCommand) returns (Response);
  rpc EmergencyStop(EmergencyRequest) returns (Response);
}
```

---

## 🔐 보안 요구사항

```
1. TLS 1.3 필수
2. OAuth 2.0 / JWT 인증
3. 메시지 서명 (HMAC-SHA256)
4. 개인정보 암호화 (AES-256-GCM)
5. Rate limiting
6. Audit logging
```

---

## 📦 산출물

```
/auto/
├── spec/
│   ├── PHASE-3-PROTOCOL.md      # 프로토콜 명세
│   └── schemas/
│       ├── openapi.yaml         # REST API 스펙
│       └── grpc/
│           └── vehicle.proto    # gRPC 정의
├── api/rust/src/
│   ├── protocol/
│   │   ├── mod.rs
│   │   ├── rest.rs              # REST 클라이언트
│   │   ├── websocket.rs         # WebSocket 핸들러
│   │   └── grpc.rs              # gRPC 클라이언트
│   └── security/
│       ├── mod.rs
│       ├── auth.rs              # 인증
│       └── crypto.rs            # 암호화
└── examples/
    └── protocol_demo.rs
```

---

## 🔄 작업 순서

```
1. 프로토콜 명세 작성 (PHASE-3-PROTOCOL.md)
2. OpenAPI 스펙 생성 (openapi.yaml)
3. gRPC 정의 (vehicle.proto)
4. Rust protocol 모듈 구현
5. 보안 모듈 구현
6. 테스트 작성
7. 예제 코드 작성
```

---

## ✅ 완료 체크리스트

```
□ 프로토콜 명세 문서
□ OpenAPI 스펙
□ gRPC 정의
□ REST 클라이언트 구현
□ WebSocket 핸들러 구현
□ 인증/암호화 모듈
□ 테스트 통과
□ 예제 코드
```

---

弘益人間 🚗♿
