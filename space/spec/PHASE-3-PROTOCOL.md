# WIA Space Protocol (WSP) Specification

**Version**: 1.0.0
**Status**: Draft
**Phase**: 3 of 4

---

## 1. 개요 (Overview)

WIA Space Protocol (WSP)은 우주 미션 시스템 간 통신을 위한 표준 프로토콜입니다. 지상 관제, 우주선, 시뮬레이터 간의 데이터 교환을 정의합니다.

### 1.1 목표

- Phase 1 데이터 형식을 메시지 페이로드로 활용
- Phase 2 API와 원활한 연동
- 다중 전송 계층 지원 (WebSocket, TCP, DTN)
- 심우주 통신 지연 처리

### 1.2 범위

```
┌─────────────────────────────────────────────────────────────┐
│                    WIA Space Protocol                        │
├─────────────────────────────────────────────────────────────┤
│  Mission Control ◄─────► Spacecraft ◄─────► Ground Station  │
│                                                              │
│  ┌──────────┐    ┌──────────┐    ┌──────────┐               │
│  │Simulator │    │ Planner  │    │ Monitor  │               │
│  └──────────┘    └──────────┘    └──────────┘               │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. 용어 정의 (Terminology)

| 용어 | 정의 |
|-----|------|
| **WSP** | WIA Space Protocol |
| **Message** | 프로토콜 기본 데이터 단위 |
| **Payload** | 메시지 내 실제 데이터 |
| **Session** | 연결 세션 |
| **Bundle** | DTN 전송 단위 |
| **Latency** | 통신 지연 시간 |
| **TRL** | Technology Readiness Level |

---

## 3. 메시지 형식 (Message Format)

### 3.1 기본 메시지 구조

```json
{
  "protocol": "wia-space",
  "version": "1.0.0",
  "messageId": "uuid-v4",
  "timestamp": 1702483200000,
  "sequence": 1,
  "type": "메시지 유형",
  "source": {
    "id": "source-id",
    "type": "ground_station | spacecraft | simulator"
  },
  "destination": {
    "id": "destination-id",
    "type": "ground_station | spacecraft | simulator"
  },
  "payload": {
    // 메시지 데이터
  },
  "metadata": {
    "priority": "critical | high | normal | low",
    "ttl": 86400,
    "compression": "none | gzip | lz4",
    "encryption": "none | aes-256-gcm"
  }
}
```

### 3.2 필드 설명

| 필드 | 타입 | 필수 | 설명 |
|-----|------|-----|------|
| `protocol` | string | ✓ | 프로토콜 식별자 ("wia-space") |
| `version` | string | ✓ | 프로토콜 버전 (semver) |
| `messageId` | string | ✓ | 고유 메시지 ID (UUID v4) |
| `timestamp` | number | ✓ | Unix 타임스탬프 (밀리초) |
| `sequence` | number | ✓ | 시퀀스 번호 |
| `type` | string | ✓ | 메시지 유형 |
| `source` | object | ✓ | 송신자 정보 |
| `destination` | object | ✓ | 수신자 정보 |
| `payload` | object | ✓ | 메시지 페이로드 |
| `metadata` | object | | 메타데이터 (선택) |

---

## 4. 메시지 유형 (Message Types)

### 4.1 연결 관리 메시지

| Type | 방향 | 설명 |
|------|-----|------|
| `connect` | Client → Server | 연결 요청 |
| `connect_ack` | Server → Client | 연결 응답 |
| `disconnect` | Both | 연결 종료 |
| `ping` | Client → Server | 연결 확인 |
| `pong` | Server → Client | 연결 확인 응답 |

### 4.2 미션 데이터 메시지

| Type | 방향 | 설명 |
|------|-----|------|
| `telemetry` | Spacecraft → Ground | 텔레메트리 데이터 |
| `command` | Ground → Spacecraft | 명령 전송 |
| `command_ack` | Spacecraft → Ground | 명령 확인 |
| `mission_update` | Both | 미션 상태 업데이트 |
| `simulation_data` | Simulator → Client | 시뮬레이션 데이터 |

### 4.3 기술 사양 메시지

| Type | 방향 | 설명 |
|------|-----|------|
| `spec_update` | Both | 기술 사양 업데이트 (Phase 1 형식) |
| `validation_request` | Client → Server | 사양 검증 요청 |
| `validation_result` | Server → Client | 검증 결과 |

### 4.4 에러 메시지

| Type | 방향 | 설명 |
|------|-----|------|
| `error` | Both | 에러 메시지 |
| `warning` | Both | 경고 메시지 |

---

## 5. 메시지 예시

### 5.1 연결 요청

```json
{
  "protocol": "wia-space",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": 1702483200000,
  "sequence": 1,
  "type": "connect",
  "source": {
    "id": "ground-station-01",
    "type": "ground_station"
  },
  "destination": {
    "id": "mission-control",
    "type": "ground_station"
  },
  "payload": {
    "clientName": "Goldstone DSN",
    "capabilities": ["telemetry", "command", "tracking"],
    "supportedTechnologies": [
      "dyson_sphere",
      "mars_terraforming",
      "asteroid_mining"
    ],
    "options": {
      "dataRate": 1000000,
      "compression": true,
      "encryption": true
    }
  }
}
```

### 5.2 텔레메트리 데이터

```json
{
  "protocol": "wia-space",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440001",
  "timestamp": 1702483200100,
  "sequence": 42,
  "type": "telemetry",
  "source": {
    "id": "mars-orbiter-01",
    "type": "spacecraft"
  },
  "destination": {
    "id": "mission-control",
    "type": "ground_station"
  },
  "payload": {
    "missionId": "mars-terraform-2035",
    "subsystem": "atmospheric_processor",
    "readings": {
      "co2_concentration": 0.953,
      "temperature_k": 210.5,
      "pressure_pa": 636,
      "processing_rate_kg_per_day": 1500
    },
    "status": "nominal",
    "timestamp": {
      "mission_elapsed_time_s": 86400,
      "utc": "2035-03-15T12:00:00Z"
    }
  },
  "metadata": {
    "priority": "normal",
    "ttl": 3600
  }
}
```

### 5.3 기술 사양 업데이트 (Phase 1 형식 포함)

```json
{
  "protocol": "wia-space",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440002",
  "timestamp": 1702483200200,
  "sequence": 100,
  "type": "spec_update",
  "source": {
    "id": "design-system",
    "type": "simulator"
  },
  "destination": {
    "id": "mission-planner",
    "type": "ground_station"
  },
  "payload": {
    "$schema": "https://wia.live/space/v1/dyson-sphere.schema.json",
    "version": "1.0.0",
    "projectId": "dyson-alpha-centauri",
    "technology": "dyson_sphere",
    "trl": 2,
    "spec": {
      "type": "swarm",
      "targetStar": {
        "name": "Alpha Centauri A",
        "type": "G2V",
        "luminosity": 1.519
      },
      "structure": {
        "collectorCount": 1000000000,
        "totalMass_kg": 1e22,
        "captureEfficiency": 0.40
      },
      "powerOutput_watts": 5.8e26
    }
  }
}
```

### 5.4 에러 메시지

```json
{
  "protocol": "wia-space",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440003",
  "timestamp": 1702483200300,
  "sequence": 101,
  "type": "error",
  "source": {
    "id": "mission-control",
    "type": "ground_station"
  },
  "destination": {
    "id": "ground-station-01",
    "type": "ground_station"
  },
  "payload": {
    "code": 2001,
    "name": "SPACECRAFT_UNREACHABLE",
    "message": "Unable to establish contact with mars-orbiter-01",
    "details": {
      "lastContact": "2035-03-15T11:45:00Z",
      "expectedDelay_s": 1200,
      "actualDelay_s": 3600
    },
    "recoverable": true,
    "retryAfter_s": 300
  }
}
```

---

## 6. 연결 관리 (Connection Management)

### 6.1 연결 상태 머신

```
┌─────────────────┐
│  DISCONNECTED   │
└────────┬────────┘
         │ connect()
         ▼
┌─────────────────┐
│   CONNECTING    │
└────────┬────────┘
         │ connect_ack received
         ▼
┌─────────────────┐
│   CONNECTED     │◄──────────────┐
└────────┬────────┘               │
         │                        │ reconnect success
         │ error/timeout          │
         ▼                        │
┌─────────────────┐               │
│  RECONNECTING   ├───────────────┘
└────────┬────────┘
         │ max retries exceeded
         ▼
┌─────────────────┐
│     ERROR       │
└─────────────────┘
```

### 6.2 연결 상태

| 상태 | 설명 |
|-----|------|
| `DISCONNECTED` | 연결되지 않음 |
| `CONNECTING` | 연결 시도 중 |
| `CONNECTED` | 연결됨 |
| `RECONNECTING` | 재연결 시도 중 |
| `ERROR` | 복구 불가 에러 |

### 6.3 하트비트

- **주기**: 30초 (지구 근방), 5분 (심우주)
- **타임아웃**: 3회 실패 시 재연결 시도
- **적응형**: 통신 지연에 따라 동적 조절

```json
{
  "type": "ping",
  "payload": {
    "sentAt": 1702483200000,
    "expectedLatency_ms": 1200000
  }
}
```

```json
{
  "type": "pong",
  "payload": {
    "pingId": "550e8400-e29b-41d4-a716-446655440004",
    "receivedAt": 1702484400000,
    "processedAt": 1702484400005
  }
}
```

---

## 7. 에러 처리 (Error Handling)

### 7.1 에러 코드

#### 연결 에러 (1xxx)

| 코드 | 이름 | 설명 |
|-----|------|------|
| `1000` | `CONNECTION_CLOSED` | 정상 종료 |
| `1001` | `CONNECTION_LOST` | 연결 끊김 |
| `1002` | `CONNECTION_TIMEOUT` | 연결 시간 초과 |
| `1003` | `PROTOCOL_ERROR` | 프로토콜 오류 |
| `1004` | `VERSION_MISMATCH` | 버전 불일치 |

#### 통신 에러 (2xxx)

| 코드 | 이름 | 설명 |
|-----|------|------|
| `2001` | `SPACECRAFT_UNREACHABLE` | 우주선 연결 불가 |
| `2002` | `GROUND_STATION_BUSY` | 지상국 사용 중 |
| `2003` | `LINK_DEGRADED` | 통신 품질 저하 |
| `2004` | `SIGNAL_LOST` | 신호 손실 |
| `2005` | `LATENCY_EXCEEDED` | 지연 시간 초과 |

#### 데이터 에러 (3xxx)

| 코드 | 이름 | 설명 |
|-----|------|------|
| `3001` | `INVALID_MESSAGE` | 잘못된 메시지 형식 |
| `3002` | `PAYLOAD_TOO_LARGE` | 페이로드 크기 초과 |
| `3003` | `CHECKSUM_FAILED` | 체크섬 오류 |
| `3004` | `SEQUENCE_GAP` | 시퀀스 누락 |
| `3005` | `DUPLICATE_MESSAGE` | 중복 메시지 |

#### 미션 에러 (4xxx)

| 코드 | 이름 | 설명 |
|-----|------|------|
| `4001` | `MISSION_NOT_FOUND` | 미션 없음 |
| `4002` | `INVALID_COMMAND` | 잘못된 명령 |
| `4003` | `COMMAND_REJECTED` | 명령 거부됨 |
| `4004` | `SUBSYSTEM_OFFLINE` | 서브시스템 오프라인 |
| `4005` | `RESOURCE_EXHAUSTED` | 자원 부족 |

#### 인증 에러 (5xxx)

| 코드 | 이름 | 설명 |
|-----|------|------|
| `5001` | `AUTH_REQUIRED` | 인증 필요 |
| `5002` | `AUTH_FAILED` | 인증 실패 |
| `5003` | `PERMISSION_DENIED` | 권한 없음 |
| `5004` | `SESSION_EXPIRED` | 세션 만료 |

### 7.2 에러 복구 전략

| 에러 유형 | 전략 |
|---------|------|
| 연결 에러 | 지수 백오프 재시도 (2s, 4s, 8s, 16s, 32s) |
| 통신 에러 | 대체 채널 시도, DTN 번들 저장 |
| 데이터 에러 | 재전송 요청 |
| 미션 에러 | 운영자 개입 요청 |
| 인증 에러 | 재인증 |

---

## 8. 전송 계층 (Transport Layer)

### 8.1 전송 어댑터 인터페이스

```rust
#[async_trait]
pub trait Transport: Send + Sync {
    /// 연결
    async fn connect(&mut self, endpoint: &str) -> Result<(), TransportError>;

    /// 연결 종료
    async fn disconnect(&mut self) -> Result<(), TransportError>;

    /// 메시지 전송
    async fn send(&self, message: &WspMessage) -> Result<(), TransportError>;

    /// 메시지 수신
    async fn receive(&mut self) -> Result<WspMessage, TransportError>;

    /// 연결 상태
    fn is_connected(&self) -> bool;

    /// 전송 유형
    fn transport_type(&self) -> TransportType;
}
```

### 8.2 지원 전송 방식

#### WebSocket

- **용도**: 개발, 테스트, 실시간 시뮬레이션
- **포트**: 8080 (ws://), 8443 (wss://)
- **특징**: 양방향, 저지연

```
ws://mission-control.example.com:8080/wsp
wss://mission-control.example.com:8443/wsp
```

#### TCP

- **용도**: 지상 네트워크, 미션 관제
- **포트**: 9090
- **특징**: 신뢰성, 순서 보장

```
tcp://192.168.1.100:9090
```

#### DTN Bundle

- **용도**: 심우주 통신
- **특징**: 지연 허용, 저장 후 전달

```
dtn://mars-relay.nasa.gov/wsp
```

### 8.3 지연 시뮬레이션

다양한 우주 환경의 통신 지연을 시뮬레이션:

| 환경 | 편도 지연 | 왕복 지연 |
|-----|----------|----------|
| LEO | 0.02초 | 0.04초 |
| GEO | 0.12초 | 0.24초 |
| 달 | 1.3초 | 2.6초 |
| 화성 (최근) | 3분 | 6분 |
| 화성 (최원) | 22분 | 44분 |
| 목성 | 35-52분 | 70-104분 |
| 토성 | 70-90분 | 140-180분 |

```json
{
  "simulation": {
    "target": "mars",
    "position": "conjunction",
    "oneWayDelay_s": 1320,
    "jitter_s": 5,
    "packetLoss": 0.001
  }
}
```

---

## 9. 보안 (Security)

### 9.1 인증

- **세션 토큰**: JWT 기반
- **API 키**: 정적 키 (테스트용)
- **인증서**: X.509 (프로덕션)

### 9.2 암호화

| 계층 | 방식 |
|-----|------|
| 전송 계층 | TLS 1.3 |
| 메시지 | AES-256-GCM |
| 서명 | Ed25519 |

### 9.3 권한 관리

| 역할 | 권한 |
|-----|------|
| `observer` | 읽기 전용 |
| `operator` | 명령 전송 |
| `controller` | 전체 제어 |
| `admin` | 시스템 관리 |

---

## 10. 메시지 흐름 예시

### 10.1 미션 시작

```
Ground Station          Mission Control          Spacecraft
     │                        │                       │
     │──── connect ──────────►│                       │
     │◄─── connect_ack ───────│                       │
     │                        │                       │
     │──── mission_start ────►│                       │
     │                        │──── command ─────────►│
     │                        │                       │ (delay)
     │                        │◄─── command_ack ──────│
     │◄─── mission_update ────│                       │
     │                        │                       │
     │                        │◄─── telemetry ────────│
     │◄─── telemetry ─────────│                       │
```

### 10.2 심우주 통신 (DTN)

```
Ground Station          Relay Satellite          Mars Orbiter
     │                        │                       │
     │──── bundle ───────────►│                       │
     │                        │ (store)               │
     │                        │                       │
     │                        │ (wait for contact)    │
     │                        │                       │
     │                        │──── bundle ──────────►│
     │                        │                       │ (process)
     │                        │◄─── bundle ack ───────│
     │◄─── bundle ack ────────│                       │
```

---

## 11. 확장성 (Extensibility)

### 11.1 커스텀 메시지 타입

```json
{
  "type": "x-custom/my-telemetry",
  "payload": {
    "customField": "value"
  }
}
```

### 11.2 메타데이터 확장

```json
{
  "metadata": {
    "x-mission-phase": "cruise",
    "x-priority-override": true
  }
}
```

---

## 12. 구현 체크리스트

### 필수 구현

- [ ] 메시지 직렬화/역직렬화
- [ ] 연결 상태 관리
- [ ] 메시지 시퀀싱
- [ ] 기본 에러 처리
- [ ] WebSocket 전송

### 선택 구현

- [ ] TCP 전송
- [ ] DTN Bundle 전송
- [ ] 지연 시뮬레이션
- [ ] 메시지 압축
- [ ] 메시지 암호화

---

## 13. 참조 (References)

- [CCSDS Space Packet Protocol](https://ccsds.org/Pubs/133x0b2e1.pdf)
- [RFC 9171 - Bundle Protocol Version 7](https://www.rfc-editor.org/rfc/rfc9171)
- [WebSocket Protocol RFC 6455](https://www.rfc-editor.org/rfc/rfc6455)
- [WIA Space Standard Phase 1](./PHASE-1-DATA-FORMAT.md)
- [WIA Space Standard Phase 2 Rust API](../api/rust/README.md)

---

*WIA Space Protocol Specification v1.0.0*
*Phase 3 of 4*
