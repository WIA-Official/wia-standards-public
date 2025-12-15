# WIA Biotech Communication Protocol Specification

**Phase 3: Communication Protocol Standard**

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-12

---

## 1. Overview

### 1.1 Purpose

WIA Biotech Communication Protocol은 바이오테크 시스템 간 데이터 교환을 위한 통합 통신 프로토콜입니다. 이 프로토콜은 연구소, 바이오파운드리, LIMS, 분석 플랫폼 간의 상호운용성을 보장하고, 실시간 데이터 스트리밍과 비동기 작업을 지원합니다.

### 1.2 Scope

- **In Scope**:
  - 메시지 형식 및 타입 정의
  - 연결 관리 (Connection Management)
  - 에러 처리 (Error Handling)
  - 다중 전송 계층 (WebSocket, REST)
  - 보안 및 인증

- **Out of Scope** (Phase 3):
  - 특정 장비 드라이버 (Phase 4)
  - 클라우드 통합 (Phase 4)

### 1.3 Design Principles

1. **Transport Agnostic**: 전송 계층 독립적 설계
2. **FHIR Compatible**: HL7 FHIR Genomics 호환
3. **Phase 1/2 Integration**: 기존 데이터 형식 및 API 활용
4. **Security First**: 보안 우선 설계
5. **Extensible**: 확장 가능한 메시지 구조

---

## 2. Terminology

| Term | Definition |
|------|------------|
| **Client** | 데이터를 요청하거나 명령을 보내는 시스템 |
| **Server** | 데이터를 제공하거나 명령을 처리하는 시스템 |
| **Message** | 클라이언트-서버 간 교환되는 데이터 단위 |
| **Payload** | 메시지 내 실제 데이터 (Phase 1 형식) |
| **Transport** | 메시지 전송을 담당하는 계층 |
| **Session** | 인증된 연결 상태 |

---

## 3. Message Format

### 3.1 Base Message Structure

모든 메시지는 다음 기본 구조를 따릅니다:

```json
{
  "protocol": "wia-bio",
  "version": "1.0.0",
  "message_id": "uuid-v4",
  "timestamp": "2025-12-15T10:30:00.000Z",
  "message_type": "string",
  "payload": {}
}
```

### 3.2 Field Definitions

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `protocol` | string | Yes | 프로토콜 식별자 ("wia-bio") |
| `version` | string | Yes | 프로토콜 버전 (semver) |
| `message_id` | string | Yes | UUID v4 메시지 ID |
| `timestamp` | string | Yes | ISO 8601 타임스탬프 |
| `message_type` | string | Yes | 메시지 유형 |
| `payload` | object | Yes | 메시지 데이터 |
| `correlation_id` | string | No | 요청-응답 연결 ID |
| `session_id` | string | No | 세션 식별자 |

### 3.3 Message Types

#### 3.3.1 Connection Messages

| Type | Direction | Description |
|------|-----------|-------------|
| `connect` | Client → Server | 연결 요청 |
| `connect_ack` | Server → Client | 연결 승인 |
| `disconnect` | Both | 연결 종료 |
| `ping` | Client → Server | 연결 확인 |
| `pong` | Server → Client | 연결 확인 응답 |

#### 3.3.2 Data Messages

| Type | Direction | Description |
|------|-----------|-------------|
| `sequence` | Both | 시퀀스 데이터 (Phase 1) |
| `experiment` | Both | CRISPR 실험 데이터 |
| `structure` | Both | 단백질 구조 데이터 |
| `part` | Both | 합성생물학 부품 |
| `result` | Server → Client | 분석 결과 |

#### 3.3.3 Command Messages

| Type | Direction | Description |
|------|-----------|-------------|
| `command` | Client → Server | 명령 요청 |
| `command_ack` | Server → Client | 명령 응답 |
| `subscribe` | Client → Server | 데이터 구독 |
| `unsubscribe` | Client → Server | 구독 해제 |

#### 3.3.4 Error Messages

| Type | Direction | Description |
|------|-----------|-------------|
| `error` | Both | 에러 메시지 |

---

## 4. Message Definitions

### 4.1 Connect Message

```json
{
  "protocol": "wia-bio",
  "version": "1.0.0",
  "message_id": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": "2025-12-15T10:30:00.000Z",
  "message_type": "connect",
  "payload": {
    "client_id": "app-12345",
    "client_name": "My Biotech App",
    "client_version": "1.0.0",
    "capabilities": ["sequence", "crispr", "structure"],
    "auth": {
      "type": "bearer",
      "token": "eyJhbGciOiJIUzI1NiIs..."
    },
    "options": {
      "compression": false,
      "binary_payload": false
    }
  }
}
```

### 4.2 Connect Acknowledgment

```json
{
  "protocol": "wia-bio",
  "version": "1.0.0",
  "message_id": "550e8400-e29b-41d4-a716-446655440001",
  "timestamp": "2025-12-15T10:30:00.100Z",
  "message_type": "connect_ack",
  "correlation_id": "550e8400-e29b-41d4-a716-446655440000",
  "payload": {
    "session_id": "sess-abc123",
    "server_name": "WIA Bio Server",
    "server_version": "1.0.0",
    "capabilities": ["sequence", "crispr", "structure", "streaming"],
    "heartbeat_interval_ms": 30000,
    "session_timeout_ms": 300000
  }
}
```

### 4.3 Sequence Data Message

Phase 1 Sequence를 페이로드로 포함:

```json
{
  "protocol": "wia-bio",
  "version": "1.0.0",
  "message_id": "550e8400-e29b-41d4-a716-446655440002",
  "timestamp": "2025-12-15T10:30:01.000Z",
  "message_type": "sequence",
  "session_id": "sess-abc123",
  "payload": {
    "$schema": "https://wia.live/schemas/bio/sequence.schema.json",
    "sequence_id": "seq-001",
    "sequence_type": "dna",
    "length_bp": 2500,
    "sequence_info": {
      "name": "Target Gene ABC",
      "organism": "Homo sapiens"
    }
  }
}
```

### 4.4 Command Message

```json
{
  "protocol": "wia-bio",
  "version": "1.0.0",
  "message_id": "550e8400-e29b-41d4-a716-446655440003",
  "timestamp": "2025-12-15T10:30:02.000Z",
  "message_type": "command",
  "session_id": "sess-abc123",
  "payload": {
    "command": "analyze_sequence",
    "params": {
      "sequence_id": "seq-001",
      "analysis_type": "gc_content"
    }
  }
}
```

### 4.5 Subscribe Message

```json
{
  "protocol": "wia-bio",
  "version": "1.0.0",
  "message_id": "550e8400-e29b-41d4-a716-446655440004",
  "timestamp": "2025-12-15T10:30:03.000Z",
  "message_type": "subscribe",
  "session_id": "sess-abc123",
  "payload": {
    "subscription_id": "sub-001",
    "topic": "experiment_results",
    "filters": {
      "experiment_id": "crispr-exp-001"
    }
  }
}
```

### 4.6 Error Message

```json
{
  "protocol": "wia-bio",
  "version": "1.0.0",
  "message_id": "550e8400-e29b-41d4-a716-446655440005",
  "timestamp": "2025-12-15T10:30:04.000Z",
  "message_type": "error",
  "correlation_id": "550e8400-e29b-41d4-a716-446655440003",
  "payload": {
    "code": 2001,
    "error": "RESOURCE_NOT_FOUND",
    "message": "Sequence seq-001 not found",
    "details": {
      "resource_type": "sequence",
      "resource_id": "seq-001"
    }
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
│   CONNECTING    │
└────────┬────────┘
         │ connect_ack received
         ▼
┌─────────────────┐
│    CONNECTED    │◄──────────┐
└────────┬────────┘           │
         │                    │ reconnected
         │ error/timeout      │
         ▼                    │
┌─────────────────┐           │
│  RECONNECTING   ├───────────┘
└────────┬────────┘
         │ max retries exceeded
         ▼
┌─────────────────┐
│     FAILED      │
└─────────────────┘
```

### 5.2 Heartbeat

- 클라이언트는 `heartbeat_interval_ms` 간격으로 `ping` 전송
- 서버는 `pong`으로 응답
- 3회 연속 실패 시 연결 종료로 간주

### 5.3 Session Management

| Parameter | Default | Description |
|-----------|---------|-------------|
| `heartbeat_interval_ms` | 30000 | 하트비트 간격 |
| `session_timeout_ms` | 300000 | 세션 타임아웃 |
| `reconnect_attempts` | 5 | 재연결 시도 횟수 |
| `reconnect_delay_ms` | 1000 | 재연결 대기 시간 (지수 백오프) |

---

## 6. Error Handling

### 6.1 Error Codes

#### Connection Errors (1xxx)

| Code | Name | Description |
|------|------|-------------|
| 1000 | `CONNECTION_CLOSED` | 정상 종료 |
| 1001 | `CONNECTION_LOST` | 연결 끊김 |
| 1002 | `PROTOCOL_ERROR` | 프로토콜 오류 |
| 1003 | `UNSUPPORTED_VERSION` | 지원하지 않는 버전 |
| 1004 | `SESSION_EXPIRED` | 세션 만료 |

#### Resource Errors (2xxx)

| Code | Name | Description |
|------|------|-------------|
| 2001 | `RESOURCE_NOT_FOUND` | 리소스 없음 |
| 2002 | `RESOURCE_EXISTS` | 리소스 이미 존재 |
| 2003 | `INVALID_RESOURCE` | 잘못된 리소스 |
| 2004 | `RESOURCE_LOCKED` | 리소스 잠금 |

#### Authentication Errors (3xxx)

| Code | Name | Description |
|------|------|-------------|
| 3001 | `AUTH_REQUIRED` | 인증 필요 |
| 3002 | `AUTH_FAILED` | 인증 실패 |
| 3003 | `TOKEN_EXPIRED` | 토큰 만료 |
| 3004 | `PERMISSION_DENIED` | 권한 없음 |

#### Validation Errors (4xxx)

| Code | Name | Description |
|------|------|-------------|
| 4001 | `INVALID_MESSAGE` | 잘못된 메시지 |
| 4002 | `INVALID_PAYLOAD` | 잘못된 페이로드 |
| 4003 | `MISSING_FIELD` | 필수 필드 누락 |
| 4004 | `INVALID_TYPE` | 잘못된 타입 |

#### Server Errors (5xxx)

| Code | Name | Description |
|------|------|-------------|
| 5001 | `INTERNAL_ERROR` | 내부 오류 |
| 5002 | `SERVICE_UNAVAILABLE` | 서비스 불가 |
| 5003 | `TIMEOUT` | 타임아웃 |

---

## 7. Transport Layer

### 7.1 Transport Interface

모든 전송 계층은 다음 인터페이스를 구현합니다:

```rust
pub trait Transport: Send + Sync {
    async fn connect(&mut self, url: &str) -> Result<()>;
    async fn disconnect(&mut self) -> Result<()>;
    async fn send(&self, message: &BioMessage) -> Result<()>;
    async fn receive(&self) -> Result<BioMessage>;
    fn is_connected(&self) -> bool;
    fn transport_type(&self) -> TransportType;
}
```

### 7.2 WebSocket Transport

**URL Format:**
```
wss://server.example.com/wia-bio/v1
ws://localhost:8080/wia-bio/v1  (개발용)
```

**특징:**
- 양방향 실시간 통신
- 자동 재연결 지원
- 압축 옵션 (permessage-deflate)

### 7.3 REST Transport

**Base URL:**
```
https://server.example.com/api/wia-bio/v1
```

**Endpoints:**

| Method | Path | Description |
|--------|------|-------------|
| POST | `/connect` | 세션 생성 |
| DELETE | `/disconnect` | 세션 종료 |
| POST | `/messages` | 메시지 전송 |
| GET | `/messages` | 메시지 수신 (Long Polling) |
| POST | `/sequences` | 시퀀스 업로드 |
| GET | `/sequences/{id}` | 시퀀스 조회 |

---

## 8. Security

### 8.1 Transport Security

- **Required**: TLS 1.2 이상 (TLS 1.3 권장)
- **Cipher Suites**: AEAD 암호화 필수

### 8.2 Authentication

#### Bearer Token (JWT)
```json
{
  "auth": {
    "type": "bearer",
    "token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9..."
  }
}
```

#### API Key
```json
{
  "auth": {
    "type": "api_key",
    "key": "wia-bio-api-key-12345"
  }
}
```

### 8.3 Authorization

역할 기반 접근 제어 (RBAC):

| Role | Permissions |
|------|-------------|
| `reader` | 데이터 조회 |
| `writer` | 데이터 생성/수정 |
| `admin` | 전체 권한 |
| `analyzer` | 분석 실행 |

### 8.4 Audit Logging

모든 작업은 감사 로그로 기록:

```json
{
  "timestamp": "2025-12-15T10:30:00.000Z",
  "session_id": "sess-abc123",
  "user_id": "user-001",
  "action": "create_sequence",
  "resource_id": "seq-001",
  "ip_address": "192.168.1.100",
  "result": "success"
}
```

---

## 9. FHIR Compatibility

### 9.1 FHIR Genomics Mapping

WIA Biotech 메시지를 FHIR 리소스로 변환:

| WIA Type | FHIR Resource |
|----------|---------------|
| Sequence | MolecularDefinition |
| CrisprExperiment | GenomicStudy |
| ProteinStructure | MolecularDefinition |
| CrisprResults | Observation |

### 9.2 FHIR Bundle Support

```json
{
  "message_type": "fhir_bundle",
  "payload": {
    "resourceType": "Bundle",
    "type": "transaction",
    "entry": [...]
  }
}
```

---

## 10. Examples

### 10.1 Complete Session Flow

```
Client                          Server
  │                               │
  │──── connect ─────────────────>│
  │                               │
  │<──── connect_ack ─────────────│
  │                               │
  │──── sequence (data) ─────────>│
  │                               │
  │<──── command_ack ─────────────│
  │                               │
  │──── subscribe ───────────────>│
  │                               │
  │<──── result (streaming) ──────│
  │<──── result (streaming) ──────│
  │                               │
  │──── ping ────────────────────>│
  │<──── pong ────────────────────│
  │                               │
  │──── disconnect ──────────────>│
  │                               │
```

### 10.2 Error Handling Flow

```
Client                          Server
  │                               │
  │──── command (invalid) ───────>│
  │                               │
  │<──── error (4002) ────────────│
  │                               │
  │──── command (corrected) ─────>│
  │                               │
  │<──── command_ack ─────────────│
  │                               │
```

---

## 11. Schema Files

JSON Schema 파일:

- `schemas/wia-bio-message.schema.json`
- `schemas/wia-bio-connect.schema.json`
- `schemas/wia-bio-error.schema.json`

Online: `https://wia.live/schemas/bio/protocol/`

---

## 12. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-12 | Initial specification |

---

## 13. References

- [HL7 FHIR Genomics](https://www.hl7.org/fhir/genomics.html)
- [GA4GH htsget Protocol](https://samtools.github.io/hts-specs/htsget.html)
- [WebSocket Protocol RFC 6455](https://tools.ietf.org/html/rfc6455)
- [JSON Schema](https://json-schema.org/)

---

**Document Version**: 1.0.0
**Last Updated**: 2025-12
**Author**: WIA Biotech Working Group

---

弘益人間 - *Benefit All Humanity*
