# WIA CareBot — Phase 2: REST API Interface

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
4. [인증과 권한](#4-인증과-권한)
5. [공통 규약](#5-공통-규약)
6. [응답 형식 표준](#6-응답-형식-표준)
7. [리소스 엔드포인트](#7-리소스-엔드포인트)
8. [상세 API 스펙](#8-상세-api-스펙)
9. [에러 코드](#9-에러-코드)
10. [속도 제한](#10-속도-제한)
11. [실시간 WebSocket](#11-실시간-websocket)
12. [OpenAPI 3.1 스펙 발췌](#12-openapi-31-스펙-발췌)
13. [클라이언트 예제](#13-클라이언트-예제)
14. [프라이버시·동의 특별 조항](#14-프라이버시동의-특별-조항)
15. [참고문헌](#15-참고문헌)

---

## 1. 개요

### 1.1 목적

WIA CareBot REST API는 고령자·장애인·만성 질환자를 돌보는 소프트 서비스 로봇과 서드파티 시스템(의료기관 EHR, 가족 알림 앱, 지자체 사회복지 플랫폼)을 **표준화된 방식으로 연결**하기 위한 인터페이스입니다. Phase 1에서 정의한 데이터 형식(수신자 프로필, 건강 측정, 감정 상태, 일상 루틴, 안전 이벤트, 약물 복약 알림 등)을 HTTP 자원으로 노출합니다.

### 1.2 범위

**포함**:

- 수신자(recipient) 등록·프로필 관리
- 건강 측정값(혈압·혈당·심박·수면·낙상 징후) 기록·조회
- 감정/인지 상태 스냅샷
- 일상 루틴(기상·식사·복약·운동·수면) 관리
- 알림(notification) 송출과 확인
- 안전 이벤트(낙상·무응답·가스·문 개방)
- 대화 로그 (요약 수준)
- 디바이스(IoT 센서·웨어러블) 페어링

**제외 (다른 Phase)**:

- 로컬 센서 저지연 스트림 → Phase 3
- 로봇-로봇 다자간 협업 → Phase 4

### 1.3 윤리적 기반

본 API는 다음 전제 위에서 설계되었습니다:

1. **수신자 주권**: 자신의 데이터에 대한 최종 결정권은 돌봄 수신자(혹은 지정 대리인)에게 있다.
2. **비가시적 감시 금지**: 카메라·마이크 기반 관찰은 반드시 동의 + UI/LED로 활성 상태를 시각/청각적으로 알려야 한다.
3. **최소 저장 원칙**: 원시 음성·영상은 디바이스에서 특징 벡터로 가공 후 서버 전송. 원시 데이터 서버 보존은 기본 OFF.
4. **사람의 결정권 보존**: 약물 투여·외출 제어 등 위험 액션은 로봇이 단독 수행하지 않는다. API는 "건의"까지만 수행하고 보호자 확인을 요구한다.

### 1.4 레이어 관계

```
┌──────────────────────────────────────────────────────────────┐
│      CareBot App · EHR Integration · Family Dashboard         │
├──────────────────────────────────────────────────────────────┤
│               Phase 2 REST API (this specification)           │
│       (resources · envelopes · auth · consent audit)          │
├──────────────────────────────────────────────────────────────┤
│              Phase 1 Data Format (Payload)                    │
├──────────────────────────────────────────────────────────────┤
│                   HTTP/1.1 · HTTP/2 · WSS                     │
│                    (TLS 1.2+ REQUIRED)                        │
└──────────────────────────────────────────────────────────────┘
```

---

## 2. 설계 원칙

1. **Consent-First** — 모든 리소스는 `consent_state` 메타를 포함하며, 미동의 상태에서 쓰기 요청은 422를 반환한다.
2. **Resource-Oriented** — URI는 명사, 메서드는 동사.
3. **Stateless** — 토큰 인증 매 요청.
4. **Idempotent for Actions** — 알림 송출·약물 알람 설정과 같이 중복이 피해를 줄 수 있는 요청은 `Idempotency-Key` 필수.
5. **Observable** — 쓰기 요청은 감사 로그 자동 기록. 보호자가 언제든 조회 가능.
6. **Backward-Compatible Versioning** — 메이저는 URL, 마이너는 비파괴적 추가.

---

## 3. 기본 URL과 버전 관리

```
https://{host}/{version}/{resource}[/{id}][/{subresource}]

예:
https://api.wia.live/v1/recipients
https://api.wia.live/v1/recipients/rcp_01H.../health-events
https://api.wia.live/v1/robots/rbt_01H.../routines
```

서버는 `GET /v1/capabilities`로 자신이 지원하는 기능 플래그를 공개한다(`health_monitoring`, `fall_detection`, `medication_reminder`, `emotion_sensing` 등).

---

## 4. 인증과 권한

### 4.1 OAuth 2.0 (사람 사용자)

돌봄 수신자 본인, 보호자, 요양보호사용. Authorization Code + PKCE (RFC 7636). 토큰은 JWT 형식 권장 (`iss`, `sub`, `aud`, `exp`, `iat` 필수).

### 4.2 디바이스 토큰 (로봇·센서)

로봇이 서버와 통신할 때 사용. 디바이스 등록 시 발급되는 long-lived 토큰(90일) + 단기 access token(1시간) 체계. 분실 시 소유자 동의 없이 재발급 불가.

### 4.3 스코프

| Scope | 의미 |
|---|---|
| `recipient:read` / `recipient:write` | 수신자 프로필 |
| `health:read` / `health:write` | 건강 데이터 |
| `emotion:read` | 감정 스냅샷 (write는 로봇 전용) |
| `routine:read` / `routine:write` | 일상 루틴 |
| `notification:send` | 알림 송출 |
| `safety:read` / `safety:ack` | 안전 이벤트 조회·확인 |
| `audit:read` | 감사 로그 조회 (보호자 전용) |

### 4.4 동의 기반 접근 제어 (CBAC)

OAuth 스코프만으로는 부족하다. 수신자가 각 **데이터 카테고리**마다 개별 동의 상태를 가지며, 서버는 매 요청에서 다음을 평가한다:

1. 호출자가 해당 스코프를 가졌는가?
2. 수신자가 해당 데이터 카테고리에 대해 호출자(또는 호출자 역할)에게 동의했는가?

둘 중 하나라도 거짓 → `403` + `error.code="consent_not_granted"`.

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
| `Authorization` | ✓ | `Bearer <token>` |
| `Accept` / `Content-Type` | 기본 `application/json; charset=utf-8` | |
| `X-Request-ID` | - | UUID v4 |
| `Idempotency-Key` | POST 권장 | 24시간 중복 방지 |
| `X-Client-Locale` | - | BCP 47. 알림 텍스트·TTS 언어 결정 |
| `X-Acting-On-Behalf-Of` | - | 보호자가 수신자 대신 행동할 때, 수신자 ID. 동의 검사에 사용 |

### 5.3 페이지네이션

커서 기본(§WIA Climate Phase 2와 동일). `limit` 기본 50, 최대 500.

### 5.4 필터링과 정렬

```
GET /v1/recipients/rcp_01H.../health-events
   ?metric=blood_pressure&timestamp[gte]=2026-04-01T00:00:00Z&sort=-timestamp
```

공통 필터: `timestamp[gte|lte]`, `severity`, `kind`, `device.serial`.

### 5.5 조건부 요청·멱등성

- `ETag` / `If-Match` / `If-None-Match` 지원.
- POST는 `Idempotency-Key` 24시간 캐시.

### 5.6 실시간 구독

장시간 폴링 대신 WebSocket(§11) 또는 Server-Sent Events `GET /v1/recipients/{id}/events:stream`를 사용.

---

## 6. 응답 형식 표준

### 6.1 Envelope

```json
{
    "data": { /* 또는 [...] */ },
    "pagination": { /* 컬렉션만 */ },
    "meta": {
        "request_id": "req_01H...",
        "generated_at": "2026-04-25T20:00:00Z",
        "consent_state": "granted",
        "consent_expires_at": "2026-10-25T00:00:00Z"
    }
}
```

`meta.consent_state`는 요청 당시 활성 동의 상태를 스냅샷 반영한다 (`granted` · `revoked` · `expired` · `pending`).

### 6.2 리소스 ID

| 리소스 | 접두어 |
|---|---|
| Recipient | `rcp_` |
| Robot | `rbt_` |
| Health Event | `hev_` |
| Safety Event | `sev_` |
| Notification | `ntf_` |
| Routine | `rtn_` |
| Consent Record | `cns_` |

### 6.3 시각·좌표

시각은 RFC 3339 UTC. 수신자 거주지 좌표는 프라이버시를 위해 조회 시 **100m 격자 버킷**으로만 반환되며, 정확한 좌표가 필요한 요청(응급 이송)은 별도 스코프(`location:precise`) + 사유 로그를 남긴다.

---

## 7. 리소스 엔드포인트

| # | Method | Path | 설명 |
|:-:|---|---|---|
| 1 | `GET`/`POST` | `/v1/recipients` | 수신자 목록·등록 |
| 2 | `GET`/`PATCH` | `/v1/recipients/{id}` | 프로필 |
| 3 | `GET`/`POST` | `/v1/recipients/{id}/consents` | 동의 레코드 |
| 4 | `DELETE` | `/v1/recipients/{id}/consents/{cid}` | 동의 철회 (즉시) |
| 5 | `GET`/`POST` | `/v1/recipients/{id}/health-events` | 건강 이벤트 |
| 6 | `GET` | `/v1/recipients/{id}/emotion-snapshots` | 감정 스냅샷 (7일 보존 기본) |
| 7 | `GET`/`PUT` | `/v1/recipients/{id}/routines` | 일상 루틴 |
| 8 | `POST` | `/v1/recipients/{id}/notifications` | 알림 송출 |
| 9 | `GET`/`POST` | `/v1/recipients/{id}/safety-events` | 안전 이벤트 |
| 10 | `POST` | `/v1/recipients/{id}/safety-events/{sid}:ack` | 안전 이벤트 확인 |
| 11 | `GET`/`POST` | `/v1/robots` | 로봇 목록·페어링 |
| 12 | `GET` | `/v1/audit?recipient_id=...` | 감사 로그 |
| 13 | `GET` | `/v1/capabilities` | 서버 기능 플래그 |
| 14 | `GET` | `/v1/health` | 라이브니스 |
| 15 | `GET` | `/v1/openapi.json` | OpenAPI 3.1 |

---

## 8. 상세 API 스펙

### 8.1 수신자 등록

```
POST /v1/recipients
Authorization: Bearer <token-with-recipient:write>
Content-Type: application/json
Idempotency-Key: ...

{
    "display_name": "김영자",
    "birth_year": 1941,
    "gender": "female",
    "conditions": ["hypertension", "mild_cognitive_impairment"],
    "primary_language": "ko-KR",
    "guardians": [
        { "name": "김철수", "relationship": "son", "phone_e164": "+821012345678" }
    ],
    "residence_grid_100m": "37.5665,126.9780",
    "robot_paired": null
}
```

응답 `201 Created` (리소스 + `ETag: W/"1"`).

### 8.2 동의 관리

```
POST /v1/recipients/rcp_01H.../consents
Content-Type: application/json

{
    "category": "health.blood_pressure",
    "granted_to_role": "family_guardian",
    "granted_to_subject_ids": ["usr_01H..."],
    "effective_from": "2026-04-25T00:00:00Z",
    "expires_at": "2027-04-25T00:00:00Z",
    "method": "signed_in_app",
    "evidence_hash": "sha256:8c7f..."
}
```

응답 `201 Created` + 감사 로그에 불변 엔트리.

철회는 `DELETE /v1/recipients/.../consents/{cid}` — **즉시 효과**, 이후 해당 카테고리 접근은 403.

### 8.3 건강 이벤트 기록 (로봇·웨어러블)

```
POST /v1/recipients/rcp_01H.../health-events
Authorization: Bearer <device-token>
Content-Type: application/json
Idempotency-Key: ...

{
    "version": "1.0.0",
    "type": "health.blood_pressure",
    "timestamp": { "unix_ms": 1714089600000, "iso8601": "2026-04-25T20:00:00Z" },
    "device": { "manufacturer": "Omron", "model": "BP-7350", "serial": "OM73-001122" },
    "data": {
        "systolic_mmhg": 142,
        "diastolic_mmhg": 89,
        "pulse_bpm": 76,
        "measurement_posture": "sitting",
        "cuff_position": "left_upper_arm"
    },
    "meta": { "quality_score": 0.94, "operator": "self" }
}
```

서버는 Phase 1 JSON Schema로 검증 후 `201`. 수치가 수신자의 임계치(profile에서 설정한 `alert_if_systolic_gt_150`)를 넘으면 자동으로 `notification:send` 트리거.

### 8.4 알림 송출

```
POST /v1/recipients/rcp_01H.../notifications
Authorization: Bearer <token-with-notification:send>
Content-Type: application/json
Idempotency-Key: ...

{
    "kind": "medication_reminder",
    "severity": "info",
    "scheduled_at": "2026-04-26T08:00:00Z",
    "channels": ["robot_voice", "robot_screen", "family_push"],
    "title": { "ko-KR": "아침 혈압약 드실 시간이에요", "en-US": "Time for your morning blood pressure med" },
    "body": { "ko-KR": "왼쪽 파란 병, 1알입니다", "en-US": "Left blue bottle, 1 pill" },
    "confirm_required": true,
    "expires_at": "2026-04-26T10:00:00Z"
}
```

응답 `201` + `ntf_` ID. WebSocket `notification.state`로 확인 상태 추적.

### 8.5 안전 이벤트

```
POST /v1/recipients/rcp_01H.../safety-events
Authorization: Bearer <robot-device-token>
Content-Type: application/json

{
    "kind": "fall_detected",
    "severity": "high",
    "detected_at": "2026-04-25T20:01:12Z",
    "confidence": 0.92,
    "sensor_fusion": ["imu", "radar"],
    "free_text": "거실 남서쪽에서 급격한 하강 감지 후 40초 무응답"
}
```

응답 `201` + 즉시 보호자에게 알림 fan-out. 로봇은 수신자에게 음성 질의 `괜찮으세요?`를 병행하며, 60초 무응답 시 `severity=critical`로 에스컬레이션.

수신자가 괜찮은 경우 보호자나 본인이 `POST .../safety-events/{sid}:ack`로 확인. 확인 시각·주체는 감사 로그에 영구 기록.

### 8.6 감사 로그

```
GET /v1/audit?recipient_id=rcp_01H...&kind=consent_change,health_event_written&limit=100
Authorization: Bearer <token-with-audit:read>
```

응답: 시간 역순. 엔트리는 `prev_hash` + `payload_hash`로 해시 체인. 무결성 외부 검증 가능.

---

## 9. 에러 코드

| HTTP | `error.code` | 의미 |
|---|---|---|
| 400 | `invalid_request` | 구문 |
| 401 | `invalid_token` · `token_expired` | 인증 |
| 403 | `insufficient_scope` · `consent_not_granted` · `consent_revoked` | 권한·동의 |
| 404 | `resource_not_found` | 없음 |
| 409 | `idempotency_key_conflict` · `consent_conflict` | 충돌 |
| 412 | `etag_mismatch` | 낙관적 잠금 |
| 422 | `validation_failed` · `condition_threshold_exceeded` (감사 기록용) | 스키마·정합성 |
| 429 | `rate_limit_exceeded` | §10 |
| 451 | `legal_hold` | 법적 보존 요구 |
| 500 | `server_error` | 일반 |
| 503 | `service_unavailable` | 가용성 |

`consent_not_granted` 응답은 반드시 `doc_url`과 **동의 갱신을 시작할 수 있는 URL**(`errors[0].resolve_url`)을 포함해야 한다.

---

## 10. 속도 제한

| 호출자 | 일반 | 알림·안전 이벤트 |
|---|---|---|
| User OAuth | 300/min | 60/min |
| Robot Device | 1200/min | 120/min |
| API Key (3rd party) | 600/min | 30/min |

응답 헤더: `X-RateLimit-*`, `Retry-After`.

---

## 11. 실시간 WebSocket

```
GET /v1/ws/recipients/rcp_01H.../stream
Authorization: Bearer <token>
Sec-WebSocket-Protocol: wia-carebot-v1
```

서버→클라이언트 메시지:

| `type` | 요약 |
|---|---|
| `health.new` | 새 건강 이벤트 |
| `safety.declared` | 안전 이벤트 발생 |
| `safety.ack` | 확인됨 |
| `notification.delivered` / `notification.confirmed` | 알림 상태 |
| `consent.changed` | 동의 상태 변화 |
| `robot.availability` | 로봇 온·오프 |

클라이언트→서버:

- `subscribe` (categories 필터), `unsubscribe`, `ack` (중복 방지용)

하트비트: 30초 ping/pong, 60초 미응답 → `1008`.

---

## 12. OpenAPI 3.1 스펙 발췌

```yaml
openapi: 3.1.0
info:
  title: WIA CareBot API
  version: 1.0.0
  license: { name: MIT }
servers:
  - url: https://api.wia.live/v1
paths:
  /recipients/{id}/health-events:
    get:
      summary: List health events for a recipient
      parameters:
        - name: id
          in: path
          required: true
          schema: { type: string, pattern: "^rcp_[0-9A-HJKMNP-TV-Z]{26}$" }
        - name: metric
          in: query
          schema:
            type: string
            enum: [blood_pressure, blood_glucose, heart_rate, sleep, oxygen_saturation, body_weight]
        - name: timestamp[gte]
          in: query
          schema: { type: string, format: date-time }
      responses:
        '200':
          description: OK
          content:
            application/json:
              schema: { $ref: '#/components/schemas/HealthEventList' }
    post:
      summary: Record a health event
      requestBody:
        required: true
        content:
          application/json:
            schema: { $ref: '#/components/schemas/HealthEventMessage' }
      responses:
        '201':
          description: Created
          headers:
            Location: { schema: { type: string } }
components:
  securitySchemes:
    bearerAuth: { type: http, scheme: bearer, bearerFormat: JWT }
security: [ { bearerAuth: [] } ]
```

---

## 13. 클라이언트 예제

### 13.1 curl

```bash
# 혈압 이벤트 기록
curl -sS -X POST "https://api.wia.live/v1/recipients/$RCP/health-events" \
     -H "Authorization: Bearer $DEVICE_TOKEN" \
     -H "Content-Type: application/json" \
     -H "Idempotency-Key: $(uuidgen)" \
     -d '{
         "version":"1.0.0",
         "type":"health.blood_pressure",
         "timestamp":{"unix_ms":1714089600000,"iso8601":"2026-04-25T20:00:00Z"},
         "device":{"manufacturer":"Omron","model":"BP-7350","serial":"OM73-001122"},
         "data":{"systolic_mmhg":142,"diastolic_mmhg":89,"pulse_bpm":76,"measurement_posture":"sitting"},
         "meta":{"quality_score":0.94,"operator":"self"}
     }'
```

### 13.2 JavaScript (fetch)

```javascript
const BASE = 'https://api.wia.live/v1';

async function recordHealthEvent(recipientId, event) {
    const res = await fetch(`${BASE}/recipients/${recipientId}/health-events`, {
        method: 'POST',
        headers: {
            'Authorization': `Bearer ${process.env.DEVICE_TOKEN}`,
            'Content-Type': 'application/json',
            'Idempotency-Key': crypto.randomUUID(),
            'X-Acting-On-Behalf-Of': recipientId,
        },
        body: JSON.stringify(event),
    });
    if (res.status !== 201) {
        const err = await res.json().catch(() => ({}));
        if (err?.error?.code === 'consent_not_granted') {
            promptRenewConsent(err.error.errors?.[0]?.resolve_url);
            return null;
        }
        throw new Error(`HTTP ${res.status}: ${err?.error?.message ?? res.statusText}`);
    }
    return (await res.json()).data;
}
```

### 13.3 Python

```python
import os, uuid, requests

BASE = "https://api.wia.live/v1"

def record_health_event(recipient_id, event):
    headers = {
        "Authorization": f"Bearer {os.environ['DEVICE_TOKEN']}",
        "Content-Type": "application/json",
        "Idempotency-Key": str(uuid.uuid4()),
    }
    r = requests.post(f"{BASE}/recipients/{recipient_id}/health-events",
                      headers=headers, json=event, timeout=15)
    if r.status_code == 403 and r.json().get("error", {}).get("code") == "consent_not_granted":
        raise PermissionError("Consent required. Resolve URL: " +
                              r.json()["error"]["errors"][0].get("resolve_url", ""))
    r.raise_for_status()
    return r.json()["data"]
```

### 13.4 WebSocket

```javascript
const ws = new WebSocket(`wss://api.wia.live/v1/ws/recipients/${RCP}/stream`, 'wia-carebot-v1');
ws.addEventListener('open', () => {
    ws.send(JSON.stringify({ type: 'auth', token: process.env.USER_TOKEN }));
    ws.send(JSON.stringify({ type: 'subscribe', categories: ['safety.declared', 'notification.confirmed'] }));
});
ws.addEventListener('message', (evt) => {
    const msg = JSON.parse(evt.data);
    if (msg.type === 'safety.declared' && msg.data.severity === 'high') {
        triggerFamilyAlarm(msg.data);
    }
});
```

---

## 14. 프라이버시·동의 특별 조항

### 14.1 최소 저장

카메라·마이크에서 유도된 특징 벡터는 서버에 저장하되, 원시 오디오/비디오는 **기본적으로 로봇 내부 24시간 보존** 후 자동 삭제. 법적 분쟁 대비 원본 보존은 별도 `legal_hold` 플래그로만 가능하며, 이는 감사 로그에 명시된다.

### 14.2 데이터 이동성 (GDPR 20조 유사)

수신자(또는 정당한 대리인)는 `GET /v1/recipients/{id}/export?format=ndjson` 으로 자신에 관한 전 데이터를 기계가 읽을 수 있는 형식으로 내려받을 수 있다. 이 엔드포인트는 항상 동의 없이 가능하며(수신자 본인 토큰일 때), 내보내기에는 감사 로그를 남긴다.

### 14.3 삭제권 (GDPR 17조 유사)

수신자 전체 프로필 삭제는 `DELETE /v1/recipients/{id}?reason=...`로 요청. 요청 시 30일 유예(철회 가능) 후 영구 삭제. 안전·의료 기록 중 법정 보관 기간이 있는 항목은 삭제 대신 "redacted" 상태로 전환한다.

### 14.4 보호자의 열람 범위

보호자는 수신자 명시 동의가 있는 카테고리만 볼 수 있다. 보호자가 동의 범위 밖을 쿼리하면 해당 항목은 `null` 또는 `redacted` 표식으로 반환되며, 403 대신 본문의 `data[n].state = "not_consented"` 필드로 알린다. 이는 보호자가 "동의 더 요청하기" 플로우를 유도할 수 있게 하기 위함이다.

### 14.5 비가시 감시 방지

로봇이 능동적으로 관찰(카메라·마이크) 중이면, `GET /v1/robots/{id}/observation-state` 응답에 `currently_sensing: true`와 사유가 실시간 반영되며, 로봇의 물리적 LED/사운드로도 표시된다. 이 둘이 불일치하는 상태는 로봇 이상으로 간주되어 안전 이벤트가 자동 생성된다.

---

## 15. 참고문헌

- **ISO 13482:2014** — Robots and robotic devices — Safety requirements for personal care robots
- **IEC 62304** — Medical device software — Software life cycle processes
- **HL7 FHIR R5** — Health data exchange (observation·medication resources)
- **RFC 9110** — HTTP Semantics
- **RFC 7636** — OAuth 2.0 PKCE
- **RFC 7519** — JSON Web Token
- **RFC 7396** — JSON Merge Patch
- **GDPR Articles 7, 17, 20** — Consent, Erasure, Data Portability
- **WIA CareBot Phase 1** (`spec/PHASE-1-DATA-FORMAT.md`)
- **WIA CareBot Phase 3** (`spec/PHASE-3-COMMUNICATION.md`)
- **WIA CareBot Phase 4** (`spec/PHASE-4-INTEGRATION.md`)

---

**弘益人間 · Benefit All Humanity**
