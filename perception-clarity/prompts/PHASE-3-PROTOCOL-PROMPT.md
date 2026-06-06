# Phase 3: Protocol Specification
## Claude Code 작업 프롬프트

---

**Standard**: WIA Perception Clarity
**Phase**: 3 of 4
**목표**: 인식 명료도 보고 통신 프로토콜 및 메시지 플로우 표준화

---

## 🎯 목표

에이전트 ↔ 플릿 ↔ 안전 시스템 간 인식 명료도 보고 프로토콜을 정의하고 구현합니다. VDA5050 메시지 모델을 본보기로 삼습니다.

---

## 📋 웹서치 키워드

```
sensor contamination, lens soiling autonomous, lidar window cleaning, camera occlusion detection, VDA5050, MTF degradation
```

---

## 🔗 통신 아키텍처

```
┌─────────────────┐      ┌─────────────────┐
│  Physical Agent │ ←──► │  Fleet Server   │
│ (veh/robot/AMR) │      └────────┬────────┘
└────────┬────────┘               │
         │     WebSocket/MQTT     │
         │     (VDA5050-style)    │
         └───────────┬────────────┘
                     │
              ┌──────▼──────┐
              │   Safety    │
              │  Supervisor │
              └─────────────┘
```

---

## 📡 프로토콜 정의

### REST API
```
POST   /api/v1/clarity/reports        # SensorClarityReport 송출
GET    /api/v1/clarity/agents/{id}    # 에이전트 최신 명료도 조회
GET    /api/v1/clarity/sensors/{id}   # 센서 단위 PCI/상태 조회
POST   /api/v1/clarity/conformance    # 적합성 선언 등록
```

### WebSocket / MQTT Events
```
clarity.report.published   # 명료도 보고 송출
sensor.state.changed       # 상태 전이 (clear↔degraded↔obstructed↔blind)
sensor.contaminant.detected# 오염 감지
safe.action.triggered      # 안전 동작 트리거 (감속·우회·세척·safe-state)
```

---

## 🔐 보안 요구사항

```
1. TLS 1.3 / DTLS 필수
2. OAuth 2.0 / JWT 인증
3. 메시지 서명 (HMAC-SHA256)
4. agentId 등 PII 최소 수집·접근 통제
5. Rate limiting
6. Audit logging
```

---

## 📦 산출물

```
/perception-clarity/
├── spec/
│   ├── PHASE-3-PROTOCOL.md      # 프로토콜 명세
│   └── schemas/
│       └── openapi.yaml         # REST API 스펙
├── api/rust/src/
│   ├── protocol/
│   │   ├── mod.rs
│   │   ├── rest.rs              # REST 클라이언트
│   │   └── websocket.rs         # WebSocket/MQTT 핸들러
│   └── security/
│       ├── mod.rs
│       └── auth.rs              # 인증/서명
└── examples/
    └── protocol_demo.rs
```

---

## 🔄 작업 순서

```
1. 프로토콜 명세 작성 (PHASE-3-PROTOCOL.md)
2. OpenAPI 스펙 생성 (openapi.yaml)
3. Rust protocol 모듈 구현
4. 보안 모듈 구현
5. 테스트 작성
6. 예제 코드 작성
```

---

## ✅ 완료 체크리스트

```
□ 프로토콜 명세 문서
□ OpenAPI 스펙
□ REST 클라이언트 구현
□ WebSocket/MQTT 핸들러 구현
□ 인증/서명 모듈
□ 테스트 통과
□ 예제 코드
```

---

弘益人間 🤟
