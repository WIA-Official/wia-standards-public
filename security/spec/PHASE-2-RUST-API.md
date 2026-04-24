# WIA Security — Phase 2: SDK & REST API Reference

---

**Version**: 1.0.0
**Status**: Draft → Published
**Date**: 2026-04-25
**Authors**: WIA (World Industry Authentication Association) / SmileStory Inc.
**License**: MIT

---

## 목차 (Table of Contents)

1. [개요](#1-개요)
2. [설계 원칙](#2-설계-원칙)
3. [기본 URL과 버전 관리](#3-기본-url과-버전-관리)
4. [인증 체계](#4-인증-체계)
5. [공통 규약](#5-공통-규약)
6. [응답 형식 표준](#6-응답-형식-표준)
7. [리소스 엔드포인트](#7-리소스-엔드포인트)
8. [상세 API 스펙](#8-상세-api-스펙)
9. [다언어 SDK](#9-다언어-sdk)
10. [에러 코드](#10-에러-코드)
11. [속도 제한과 배압](#11-속도-제한과-배압)
12. [OpenAPI 3.1 스펙 발췌](#12-openapi-31-스펙-발췌)
13. [WebSocket 저지연 스트림](#13-websocket-저지연-스트림)
14. [보안 고려사항](#14-보안-고려사항)
15. [참고문헌](#15-참고문헌)

---

## 1. 개요

### 1.1 목적

WIA Security REST API + SDK는 Phase 1에서 정의한 보안 이벤트 형식(알림·탐지·정책 평가·감사 로그)을 **도구 중립적**으로 교환하기 위한 계약입니다. 단일 SaaS가 아니라, SIEM·EDR·IAM·WAF·CNAPP 간에 동일한 형식으로 신호를 주고받을 수 있도록 설계되었습니다.

### 1.2 범위

**포함**:

- 보안 이벤트(alert·finding·incident·policy decision·audit log)의 CRUD
- 위험 점수(risk score) 계산·조회
- 탐지 규칙(rule) 관리와 시뮬레이션
- 자산 인벤토리(asset)와 소유자(ownership)
- 취약점(vulnerability) 및 CVE 매핑
- 인증·인가 이벤트와 MFA 트리거
- Bearer JWT · mTLS · API Key 인증

**제외 (다른 Phase)**:

- 바이너리 포렌식 아티팩트 대용량 업로드 → Phase 3 (스트리밍·청크)
- 엔드포인트 에이전트 배포 → Phase 4 (패키징)
- 크로스 도메인 연합(federation) → Phase 4 (IdP 연계)

### 1.3 레이어 관계

```
┌──────────────────────────────────────────────────────────────┐
│        SIEM · SOAR · EDR · CNAPP · IdP · Audit Dashboard      │
├──────────────────────────────────────────────────────────────┤
│          Phase 2 REST API + SDK (this specification)          │
│     (resources · envelopes · SDK bindings · consent logs)     │
├──────────────────────────────────────────────────────────────┤
│              Phase 1 Data Format (Payload)                    │
│     (Alert · Finding · Incident · AuditLog · PolicyDecision)  │
├──────────────────────────────────────────────────────────────┤
│               HTTP/2 · mTLS 1.3 · WSS                         │
└──────────────────────────────────────────────────────────────┘
```

### 1.4 핵심 용어

| 용어 | 정의 |
|---|---|
| **Event** | Phase 1 정의 메시지 인스턴스 (alert·finding·incident 등) |
| **Alert** | 즉각 대응이 필요한 탐지 결과 |
| **Finding** | 스캔·컴플라이언스 점검으로 발견된 정적 상태 |
| **Incident** | 하나 이상의 alert/finding을 묶은 대응 단위 |
| **Policy Decision** | 접근 요청에 대한 허용/차단 판단 |
| **Asset** | 보호 대상 (서버·컨테이너·계정·데이터셋 등) |
| **Rule** | 이벤트를 만들어내는 탐지 논리 |
| **Risk Score** | 자산 단위로 계산된 0–100 복합 점수 |

---

## 2. 설계 원칙

1. **Zero Trust Friendly** — 모든 요청은 강한 인증을 전제로 한다. 기본 교차망 접근은 mTLS + JWT 이중 검증.
2. **Event-First** — 리소스의 상태보다 이벤트 스트림이 1차. 상태는 이벤트 집계로 파생된다.
3. **Idempotent Critical Paths** — 대응(quarantine·isolate) 호출은 `Idempotency-Key` 필수.
4. **Immutable Audit** — 모든 쓰기는 append-only 감사 로그에 해시 체인으로 남는다.
5. **Structured Failures** — 4xx 응답은 기계 해석 가능한 `code`와 `remediation_hint`를 포함.
6. **Vendor-Neutral** — 스키마·스코프·감사 형식은 특정 벤더 모델을 가정하지 않는다. STIX 2.1·OpenCybersecurityFramework(OCSF)와 상호 매핑 문서를 부속으로 제공.

---

## 3. 기본 URL과 버전 관리

```
https://{host}/{version}/{resource}[/{id}][/{subresource}]

예:
https://api.wia.live/v1/events
https://api.wia.live/v1/incidents/inc_01H.../timeline
https://api.wia.live/v1/assets/ast_01H.../risk
```

`GET /v1/capabilities`로 서버가 지원하는 기능 플래그(mTLS · stream · SOAR · MITRE mapping 등)를 공개.

---

## 4. 인증 체계

### 4.1 Bearer JWT (기본)

`Authorization: Bearer <JWT>`. JWT 클레임 중 `iss` · `sub` · `aud` · `exp` · `iat` 필수, `jti`·`scope` 강력 권장. 토큰 수명은 **10분 이내** 권장, 갱신은 OAuth 2.0 refresh token (RFC 6749).

### 4.2 mTLS (상호 TLS)

엄격한 배치에서는 mTLS를 강제. 서버는 클라이언트 인증서 체인을 검증 후, `subject` DN을 OAuth subject와 함께 바인딩하여 토큰 도용 재사용을 막는다(RFC 8705 — OAuth Mutual-TLS Certificate-Bound Access Tokens).

### 4.3 Service API Key

서비스 간 호출 전용. `X-WIA-API-Key: <key>`. 32바이트 이상 엔트로피, 서버는 argon2 해시 저장, 90일 로테이션.

### 4.4 스코프

| Scope | 의미 |
|---|---|
| `event:read` / `event:write` | 이벤트 CRUD |
| `incident:manage` | 인시던트 전이·할당 |
| `response:execute` | 격리·차단 액션 실행 |
| `rule:read` / `rule:write` | 탐지 규칙 관리 |
| `asset:read` / `asset:write` | 자산 인벤토리 |
| `audit:read` | 감사 로그 조회 |
| `admin:tenant` | 테넌트 관리 |

`response:execute`는 반드시 **2인 승인(two-person rule)** 기반으로 활성되는 단기 상승 토큰으로만 부여할 것을 권장.

### 4.5 미인증·권한부족

| 상황 | 응답 |
|---|---|
| 헤더 없음 | `401` + `WWW-Authenticate: Bearer realm="wia-sec"` |
| 토큰 무효 | `401` + `error.code="invalid_token"` |
| mTLS 미제출 (요구 시) | `495` (Nginx 준용) 또는 `401` |
| 스코프 부족 | `403` + `error.code="insufficient_scope"` |

---

## 5. 공통 규약

### 5.1 HTTP 메서드

| 메서드 | 의미 | 멱등 |
|---|---|:-:|
| `GET` | 조회 | ✓ |
| `POST` | 생성·액션 | ✗ |
| `PUT` | 전체 대체 | ✓ |
| `PATCH` | 부분 수정 (JSON Merge Patch, RFC 7396) | ✗ |
| `DELETE` | 삭제·취소 | ✓ |

### 5.2 요청 헤더

| 헤더 | 필수 | 비고 |
|---|:-:|---|
| `Authorization` | ✓ | Bearer JWT |
| `Content-Type` | 본문 있을 때 | `application/json; charset=utf-8` |
| `X-Request-ID` | - | UUID v4 |
| `Idempotency-Key` | 권장 | 24시간 중복 방지 |
| `X-Tenant-ID` | - | 멀티테넌트 강제 |
| `X-Signed-Action` | 대응 액션에 권장 | 서명 페이로드 (§14.2) |

### 5.3 응답 헤더

| 헤더 | 값 |
|---|---|
| `Content-Type` | `application/json; charset=utf-8` |
| `X-Request-ID` | 반사 또는 서버 생성 |
| `ETag` | 리소스 버전 |
| `X-RateLimit-Limit/Remaining/Reset` | §11 |
| `Content-Digest` | RFC 9530 digest (무결성) |

### 5.4 페이지네이션

커서 기본. `limit` 최대 1000 (이벤트는 대량이므로 일반보다 큼).

### 5.5 필터

공통 필터: `severity`, `status`, `asset_id`, `rule_id`, `timestamp[gte|lte]`, `tags`, `mitre_attack_technique`, `cvss_base_score[gte]`.

### 5.6 조건부 요청·멱등성

- `If-Match` / `If-None-Match`.
- 대응 액션(`POST /incidents/{id}:respond`)은 24시간 `Idempotency-Key` 캐시.

---

## 6. 응답 형식 표준

### 6.1 Envelope

**성공**:

```json
{
    "data": { /* or [...] */ },
    "pagination": { /* 컬렉션만 */ },
    "meta": {
        "request_id": "req_01H...",
        "generated_at": "2026-04-25T20:00:00Z",
        "audit_log_hash": "sha256:abcd...",
        "rule_revision": 142
    }
}
```

**실패**:

```json
{
    "error": {
        "code": "policy_violation",
        "message": "Requested action blocked by policy 'no_cross_region_isolate'.",
        "errors": [
            { "policy_id": "pol_01H...", "rule": "no_cross_region_isolate" }
        ],
        "remediation_hint": "Request approval from a Region Admin or invoke from same-region token.",
        "doc_url": "https://wia.live/security/errors/policy_violation"
    }
}
```

### 6.2 리소스 ID (ULID + 접두어)

| 리소스 | 접두어 |
|---|---|
| Event | `evt_` |
| Alert | `alt_` |
| Finding | `fnd_` |
| Incident | `inc_` |
| Asset | `ast_` |
| Rule | `rul_` |
| Policy | `pol_` |
| Audit entry | `aud_` |

### 6.3 시각

RFC 3339 UTC. 이벤트 내부는 Phase 1의 `{unix_ms, iso8601}` 이중 표기.

---

## 7. 리소스 엔드포인트

| # | Method | Path | 설명 |
|:-:|---|---|---|
| 1 | `GET`/`POST` | `/v1/events` | 이벤트 목록·수집 |
| 2 | `GET` | `/v1/events/{id}` | 이벤트 상세 |
| 3 | `POST` | `/v1/events:batch` | 최대 10000건 일괄 수집 |
| 4 | `GET`/`POST` | `/v1/incidents` | 인시던트 |
| 5 | `PATCH` | `/v1/incidents/{id}` | 상태·할당·태그 수정 |
| 6 | `POST` | `/v1/incidents/{id}:respond` | 대응 액션 실행 |
| 7 | `GET` | `/v1/incidents/{id}/timeline` | 관련 이벤트·코멘트 타임라인 |
| 8 | `GET`/`POST` | `/v1/assets` | 자산 인벤토리 |
| 9 | `GET` | `/v1/assets/{id}/risk` | 위험 점수 계산 결과 |
| 10 | `GET`/`POST` | `/v1/rules` | 탐지 규칙 |
| 11 | `POST` | `/v1/rules/{id}:simulate` | 샘플 입력에 대한 규칙 시뮬레이션 |
| 12 | `GET` | `/v1/audit` | 감사 로그 |
| 13 | `POST` | `/v1/auth/mfa:challenge` | MFA 트리거 |
| 14 | `GET` | `/v1/capabilities` | 서버 지원 플래그 |
| 15 | `GET` | `/v1/health` / `/v1/openapi.json` | 진단·스펙 |

---

## 8. 상세 API 스펙

### 8.1 이벤트 수집

```
POST /v1/events
Authorization: Bearer <token-with-event:write>
Content-Type: application/json
Idempotency-Key: ...

{
    "version": "1.0.0",
    "type": "alert",
    "timestamp": { "unix_ms": 1714089600000, "iso8601": "2026-04-25T20:00:00Z" },
    "source": { "source_type": "edr", "name": "Falcon Sensor", "version": "7.20" },
    "data": {
        "alert_id": "EDR-2026-042133",
        "title": "Suspicious LOLBAS invocation",
        "category": "defense_evasion",
        "severity": "high",
        "mitre_attack": { "tactic": "TA0005", "technique": "T1218" },
        "asset_ref": { "id": "ast_01H...", "hostname": "prod-01" },
        "raw_evidence_ref": "s3://wia-sec/evidence/2026/04/25/EDR-2026-042133.cbor"
    },
    "meta": { "confidence": 0.91, "operator": "auto" }
}
```

응답 `201` + `evt_` ID, `ETag: W/"1"`.

### 8.2 일괄 수집 (배치)

```
POST /v1/events:batch
Content-Type: application/x-ndjson

{...event1...}
{...event2...}
...
```

응답 `207 Multi-Status`:

```json
{
    "data": [
        { "index": 0, "status": 201, "id": "evt_01H..." },
        { "index": 1, "status": 422, "error": { "code": "validation_failed", "message": "..." } }
    ],
    "meta": { "total": 2, "succeeded": 1, "failed": 1 }
}
```

### 8.3 인시던트 대응

```
POST /v1/incidents/inc_01H.../:respond
Authorization: Bearer <short-lived-elevated-token>
Content-Type: application/json
X-Signed-Action: eyJhbGciOiJFZERTQSJ9...   # JWS over request body
Idempotency-Key: ...

{
    "action": "isolate_host",
    "target": { "asset_id": "ast_01H..." },
    "parameters": { "network_scope": "production", "duration_minutes": 60 },
    "approval_refs": [ "apr_01H...", "apr_02H..." ],
    "reason_code": "suspected_c2_beacon"
}
```

응답 `202 Accepted` (작업 ID) + WebSocket `incident.action_state`로 진행 수신.

`approval_refs`는 두 명 이상의 승인자가 사전에 생성한 approval 레코드를 가리킨다. 고위험 액션(`isolate_host`, `block_account`, `revoke_cert`)은 기본적으로 2인 승인 필수.

### 8.4 위험 점수

```
GET /v1/assets/ast_01H.../risk
```

응답:

```json
{
    "data": {
        "asset_id": "ast_01H...",
        "score": 73,
        "max_score": 100,
        "band": "high",
        "breakdown": {
            "vulnerability": 32,
            "exposure": 18,
            "asset_criticality": 15,
            "threat_intelligence": 8
        },
        "top_contributors": [
            { "kind": "vulnerability", "cve": "CVE-2026-12345", "cvss": 9.1 },
            { "kind": "exposure", "port": 3389, "network": "public" }
        ],
        "computed_at": "2026-04-25T19:55:12Z"
    },
    "meta": { "rule_revision": 142 }
}
```

### 8.5 규칙 시뮬레이션

```
POST /v1/rules/rul_01H.../:simulate
Content-Type: application/json

{
    "sample_events": [
        { "type": "auth.login_failed", "data": { "user": "alice", "src_ip": "198.51.100.24" } },
        { "type": "auth.login_failed", "data": { "user": "alice", "src_ip": "198.51.100.24" } },
        { "type": "auth.login_failed", "data": { "user": "alice", "src_ip": "198.51.100.24" } }
    ]
}
```

응답:

```json
{
    "data": {
        "would_trigger": true,
        "matches": [
            { "reason": "3 failed logins in 30s from same IP", "confidence": 0.98 }
        ],
        "would_generate": {
            "alert_category": "credential_access",
            "severity": "medium"
        }
    }
}
```

### 8.6 감사 로그

```
GET /v1/audit?actor=usr_01H...&action=incident.respond&timestamp[gte]=2026-04-20
```

엔트리는 `prev_hash` + `payload_hash` 체인. 외부 검증자(`cli/verify-audit`) 제공.

---

## 9. 다언어 SDK

Phase 1의 JSON Schema로부터 자동 생성되는 공식 SDK 3종. 모든 SDK는 동일한 이름 공간 규약(`wia.security.*`)을 따른다.

### 9.1 Rust (`api/rust/`)

```rust
use wia_security::{Client, Event, EventType, Severity};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let client = Client::builder()
        .base_url("https://api.wia.live/v1")
        .bearer_token(std::env::var("WIA_TOKEN")?)
        .build()?;

    let evt = Event::alert()
        .title("Suspicious LOLBAS invocation")
        .severity(Severity::High)
        .mitre("T1218")
        .asset_hostname("prod-01")
        .build()?;

    let saved = client.events().create(evt).await?;
    println!("saved: {}", saved.id);
    Ok(())
}
```

### 9.2 Python (`api/python/`)

```python
from wia_security import Client, EventBuilder, Severity

client = Client(
    base_url="https://api.wia.live/v1",
    token=os.environ["WIA_TOKEN"],
)

event = (
    EventBuilder.alert()
    .title("Suspicious LOLBAS invocation")
    .severity(Severity.HIGH)
    .mitre("T1218")
    .asset_hostname("prod-01")
    .build()
)

saved = client.events.create(event)
print(f"saved: {saved.id}")
```

### 9.3 TypeScript (`api/typescript/`)

```typescript
import { Client, EventBuilder } from '@wia/security';

const client = new Client({
    baseUrl: 'https://api.wia.live/v1',
    token: process.env.WIA_TOKEN!,
});

const event = EventBuilder.alert({
    title: 'Suspicious LOLBAS invocation',
    severity: 'high',
    mitre: { technique: 'T1218' },
    asset: { hostname: 'prod-01' },
});

const saved = await client.events.create(event);
console.log('saved:', saved.id);
```

### 9.4 공통 패턴

모든 SDK는 다음을 보장:

- Phase 1 스키마 검증을 **클라이언트에서 선행** (서버 왕복 줄이기)
- 자동 `Idempotency-Key` 생성 (`ulid` 기반)
- Exponential backoff + jitter 재시도 (5xx·429 한정, 멱등 요청만)
- `tracing` / `OpenTelemetry` span 자동 주입
- 비밀 문자열 마스킹 (로그 `Authorization`, `X-WIA-API-Key`)

---

## 10. 에러 코드

| HTTP | `error.code` | 의미 |
|---|---|---|
| 400 | `invalid_request` | 구문 오류 |
| 401 | `invalid_token` · `token_expired` · `mtls_required` | 인증 |
| 403 | `insufficient_scope` · `two_person_required` · `policy_violation` | 권한·정책 |
| 404 | `resource_not_found` | 없음 |
| 409 | `idempotency_key_conflict` · `incident_state_conflict` | 충돌 |
| 412 | `etag_mismatch` | 낙관적 잠금 실패 |
| 413 | `payload_too_large` | 배치·첨부 크기 초과 |
| 422 | `validation_failed` · `rule_compilation_error` | 스키마·규칙 정합 |
| 429 | `rate_limit_exceeded` · `backpressure_active` | 속도·배압 |
| 451 | `jurisdictional_restriction` | 법적 제약 |
| 500 | `server_error` | 일반 |
| 503 | `service_unavailable` · `ingest_saturated` | 가용성 |

---

## 11. 속도 제한과 배압

### 11.1 기본 한도

| 인증 | 일반 | 이벤트 수집 |
|---|---|---|
| Bearer (사용자) | 600/min | 1200/min |
| Service API Key | 1200/min | 6000/min |
| 미인증 (health만) | 60/min | N/A |

### 11.2 배압(Backpressure)

수집 엔드포인트가 포화되면 `429` 대신 `503 service_unavailable` + `Retry-After`를 반환할 수 있다. 응답 헤더 `X-Backpressure-Strategy: drop_low_severity_info` 로 서버가 취하는 전략을 알린다. 클라이언트는 자체 큐에서 선별적 재전송을 수행해야 한다.

---

## 12. OpenAPI 3.1 스펙 발췌

```yaml
openapi: 3.1.0
info:
  title: WIA Security API
  version: 1.0.0
  license: { name: MIT }
servers:
  - url: https://api.wia.live/v1
paths:
  /events:
    get:
      summary: List security events
      parameters:
        - name: severity
          in: query
          schema:
            type: string
            enum: [info, low, medium, high, critical]
        - name: timestamp[gte]
          in: query
          schema: { type: string, format: date-time }
      responses:
        '200':
          description: OK
          content:
            application/json:
              schema: { $ref: '#/components/schemas/EventList' }
    post:
      summary: Create a security event
      requestBody:
        required: true
        content:
          application/json:
            schema: { $ref: '#/components/schemas/EventMessage' }
      responses:
        '201':
          description: Created
  /incidents/{id}:respond:
    post:
      summary: Execute a response action on an incident
      parameters:
        - name: id
          in: path
          required: true
          schema: { type: string }
      requestBody:
        required: true
        content:
          application/json:
            schema: { $ref: '#/components/schemas/ResponseActionRequest' }
      responses:
        '202':
          description: Accepted
components:
  securitySchemes:
    bearerAuth: { type: http, scheme: bearer, bearerFormat: JWT }
    mtls: { type: mutualTLS }
  schemas:
    EventMessage:
      $ref: 'https://wia.live/security/data/v1/event.schema.json'
security:
  - bearerAuth: []
  - mtls: []
```

---

## 13. WebSocket 저지연 스트림

```
GET /v1/ws/events
Authorization: Bearer <token-with-event:read>
Sec-WebSocket-Protocol: wia-security-v1
```

서버→클라이언트 메시지:

| `type` | 요약 |
|---|---|
| `event.created` | 신규 이벤트 |
| `incident.state_change` | 상태 전이 |
| `incident.action_state` | 대응 액션 진행/결과 |
| `rule.compiled` | 규칙 컴파일 결과 브로드캐스트 |

하트비트 30초, 타임아웃 60초. 클라이언트는 `subscribe` 메시지로 도메인 필터를 적용한다.

---

## 14. 보안 고려사항

### 14.1 Replay 방지

`Idempotency-Key` 외에도, 대응 액션 호출에는 단기 `nonce` 클레임이 포함된 JWT가 강력 권장된다. 서버는 동일 `jti` + `nonce` 조합을 60초 내에 거부한다.

### 14.2 서명된 액션 (X-Signed-Action)

고위험 대응 요청은 본문 전체에 대해 EdDSA/Ed25519로 서명된 JWS를 별도 헤더로 함께 보낸다. 서명 키는 HSM 기반으로 관리되며, 서버는 서명 검증 실패 시 `401 signature_verification_failed`.

### 14.3 민감 필드 최소화

클라이언트 SDK는 이벤트 페이로드에서 passphrase·플레인텍스트 비밀 값이 발견되면 자동 마스킹 후 서버로 전송한다. 서버는 이미 마스킹된 표식(`"***redacted***"`)을 감사 로그에도 그대로 저장해 원문 재구성을 방지한다.

### 14.4 CORS (관리 UI)

보안 콘솔은 브라우저 기반 접근을 허용할 때, 쿠키 대신 짧은 수명의 Bearer를 사용하고 `SameSite=Lax`·`Secure` 플래그를 강제한다. 민감 API는 CORS를 `Access-Control-Allow-Origin` 화이트리스트에서만 허용.

### 14.5 사고 발생 시 감사 보존

데이터 보존 정책이 "사건 종료 후 90일"로 설정된 테넌트에서도, `legal_hold=true` 인시던트와 연관된 모든 이벤트·감사 엔트리는 보존 기간 연장이 자동 적용된다. 해제는 법무·보안 공동 승인이 있어야 한다.

---

## 15. 참고문헌

- **NIST SP 800-61 Rev. 3** — Computer Security Incident Handling Guide
- **MITRE ATT&CK** — https://attack.mitre.org/
- **OCSF v1.x** — Open Cybersecurity Schema Framework
- **STIX 2.1** — Structured Threat Information eXpression
- **RFC 9110** — HTTP Semantics
- **RFC 7519** — JSON Web Token
- **RFC 8705** — OAuth Mutual-TLS
- **RFC 7636** — OAuth 2.0 PKCE
- **RFC 9530** — Content-Digest
- **RFC 7396** — JSON Merge Patch
- WIA Security Phase 1 (`spec/PHASE-1-DATA-FORMAT.md`)
- WIA Security Phase 3 (`spec/PHASE-3-PROTOCOL.md`)
- WIA Security Phase 4 (`spec/PHASE-4-INTEGRATION.md`)

---

**弘益人間 · Benefit All Humanity**
