# WIA Autonomous Vehicle Accessibility — Phase 2: REST API Interface

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
9. [에러 코드](#9-에러-코드)
10. [속도 제한](#10-속도-제한)
11. [실시간 WebSocket](#11-실시간-websocket)
12. [OpenAPI 3.1 스펙](#12-openapi-31-스펙)
13. [클라이언트 예제](#13-클라이언트-예제)
14. [접근성 관점의 API 설계 특수사항](#14-접근성-관점의-api-설계-특수사항)
15. [참고문헌](#15-참고문헌)

---

## 1. 개요

### 1.1 목적

WIA Autonomous Vehicle Accessibility REST API는 Phase 1에서 정의한 데이터 형식(승객 접근성 프로파일·차량 기능·탑승 요청·HMI 구성 등)을 HTTP 기반 동기식 인터페이스로 노출하기 위한 표준입니다. 본 스펙은 자율주행차 운영 플랫폼과 이용자 측 클라이언트(장애인 이동 앱, MaaS 통합 플랫폼, 지자체 대시보드)가 상호 운용 가능하게 하는 계약을 정의합니다.

### 1.2 범위

**포함**:

- 승객·차량·탑승·접근성 프로파일의 CRUD
- 경로 제안 및 탑승 견적
- HMI 설정 동기화
- 휠체어 고정 상태 조회
- 비상 상황 보고·에스컬레이션
- Bearer JWT / OAuth 2.0 PKCE 인증

**제외 (다른 Phase 참조)**:

- 센서 융합 레벨 원시 스트림 → Phase 3
- 차량-인프라(V2I) 저지연 통신 → Phase 3
- 차량 펌웨어 OTA 배포 → Phase 4

### 1.3 레이어 관계

```
┌──────────────────────────────────────────────────────────────┐
│               Rider App · Dispatcher · MaaS Plugin            │
├──────────────────────────────────────────────────────────────┤
│          Phase 2 REST API (this specification)                │
│       (resources · auth · envelopes · WebSocket session)      │
├──────────────────────────────────────────────────────────────┤
│              Phase 1 Data Format (Payload)                    │
│   (AccessibilityProfile, VehicleFeatures, RideRequest, ...)   │
├──────────────────────────────────────────────────────────────┤
│                   HTTP/1.1 · HTTP/2 · WSS                     │
│                    (TLS 1.2+ REQUIRED)                        │
└──────────────────────────────────────────────────────────────┘
```

### 1.4 핵심 용어

| 용어 | 정의 |
|---|---|
| **Rider** | 차량을 이용하는 사람. 접근성 프로필을 소유 |
| **Vehicle** | 자율주행 차량. SAE L3~L5, 접근성 기능 속성 집합 |
| **Ride** | 특정 Rider가 특정 Vehicle로 출발지→도착지 이동하는 1회 행위 |
| **Accessibility Profile** | Rider의 장애 유형·보조기구·선호 HMI 설정 |
| **Securement** | 휠체어 고정 상태 (SAE J2094, 4점/6점, 토크값) |
| **HMI Preset** | 음성·자막·햅틱·UI 콘트라스트 등 차량 내부 HMI 구성 |

---

## 2. 설계 원칙

1. **Accessibility-First** — 모든 리소스는 접근성 속성을 1급 시민으로 취급한다. 장애가 있는 Rider에게 맞출 수 없는 차량은 매칭 단계에서 제외되며, 그 사유가 응답에 포함되어야 한다.
2. **Resource-Oriented** — URI는 명사. 동사는 메서드에.
3. **Stateless** — 서버 세션 없음. 인증은 매 요청 토큰으로.
4. **Idempotent Critical Paths** — 결제·탑승 확정과 같이 중복 실행이 피해를 주는 요청은 `Idempotency-Key` 필수.
5. **Defensive on Safety** — 비상 관련 엔드포인트(§8.5)는 자체 격리된 인증 스코프(`ride:emergency`)와 별도 속도 제한을 갖는다.
6. **Backward-Compatible Versioning** — 메이저는 URL(`/v1`, `/v2`), 마이너는 비파괴적 추가.

---

## 3. 기본 URL과 버전 관리

```
https://{host}/{version}/{resource}[/{id}][/{subresource}]

예:
https://api.wia.live/v1/vehicles
https://api.wia.live/v1/riders/rdr_01H.../profile
https://api.wia.live/v1/rides/rd_01H.../securement
```

본 스펙 기준 `{version}=v1`. 서버는 `GET /v1/capabilities`로 자신이 지원하는 마이너 기능 플래그를 공개해야 한다.

---

## 4. 인증 체계

### 4.1 OAuth 2.0 Authorization Code + PKCE (RFC 7636)

모바일·웹 클라이언트의 기본 인증. 장애가 있는 사용자의 앱에서 사용하므로 스크린 리더·보이스 오버 흐름을 깨지 않도록 리다이렉트 횟수와 팝업을 최소화해야 한다.

권장 시퀀스:

1. 클라이언트 생성: `code_verifier`(43–128자 ASCII) · `code_challenge = BASE64URL(SHA-256(code_verifier))`.
2. `GET /oauth/authorize?response_type=code&client_id=...&code_challenge=...&code_challenge_method=S256&scope=ride:read+ride:write+profile:read&redirect_uri=...`
3. 사용자 동의 후 authorization code 수령.
4. `POST /oauth/token` with `grant_type=authorization_code`, `code`, `code_verifier`.
5. Access token(1시간) + Refresh token(90일) 수령.

### 4.2 Service-to-Service API Key

디스패처, 지자체 MaaS, 차량 게이트웨이가 사용. `X-WIA-API-Key` 헤더로 전달. 32바이트 이상 엔트로피, 서버는 argon2 해시 저장.

### 4.3 스코프

| Scope | 의미 |
|---|---|
| `profile:read` / `profile:write` | 접근성 프로필 |
| `vehicle:read` | 차량 목록·상세 |
| `ride:read` / `ride:write` | 탑승 조회·요청 |
| `ride:emergency` | 비상 엔드포인트 (별도 감사) |
| `dispatch:admin` | 디스패처 전용 (차량 등록·제외) |

### 4.4 미인증·권한부족

| 상황 | 응답 |
|---|---|
| 헤더 없음 | `401` + `WWW-Authenticate: Bearer realm="wia-auto"` |
| 토큰 서명 오류 | `401` + `error.code="invalid_token"` |
| 토큰 만료 | `401` + `error.code="token_expired"` |
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
| `Authorization` | ✓ | `Bearer <token>` |
| `Accept` | - | 기본 `application/json` |
| `Content-Type` | 본문 있을 때 ✓ | `application/json; charset=utf-8` |
| `X-Request-ID` | - | UUID v4. 서버가 누락 시 생성 |
| `Idempotency-Key` | POST 강력 권장 | 24시간 중복 방지 |
| `X-Client-Locale` | - | BCP 47 (예: `ko-KR`, `en-US`). HMI preset 기본값·음성 안내 언어 결정 |
| `X-Assistive-Tech` | - | `screen_reader`, `switch_access`, `voice_control` 등. 서버가 응답 구조를 AT 친화적으로 조정 |

### 5.3 응답 헤더

| 헤더 | 값 |
|---|---|
| `Content-Type` | `application/json; charset=utf-8` |
| `X-Request-ID` | 반사 또는 생성 |
| `X-RateLimit-Limit/Remaining/Reset` | §10 |
| `ETag` | 리소스 조회 시 버전 |
| `X-Matched-Accessibility-Gap` | 매칭 결과가 사용자 프로필과 완전히 일치하지 않을 때, 부족 항목 목록 (§14.2) |

### 5.4 페이지네이션 (커서)

```
GET /v1/vehicles?limit=50&cursor=eyJ0IjoxNzI...
```

| 파라미터 | 기본 | 최대 |
|---|---|---|
| `limit` | 50 | 200 |
| `cursor` | - | - |

응답:

```json
{
    "data": [ /* 항목 */ ],
    "pagination": { "limit": 50, "next_cursor": "eyJ...", "has_more": true }
}
```

### 5.5 필터링과 정렬

```
GET /v1/vehicles?features.wheelchair_ramp=true&features.securement_points[gte]=4&sort=-proximity_m
```

서버는 다음 공통 필터를 지원해야 한다: `location[bbox]`, `features.*`, `status`, `sae_level`, `updated_at[gte|lte]`.

### 5.6 조건부 요청 (ETag)

```
GET /v1/rides/rd_01H... → 200 OK, ETag: W/"7"
GET /v1/rides/rd_01H... (If-None-Match: W/"7") → 304 Not Modified
PATCH /v1/rides/rd_01H... (If-Match: W/"7") → 200 or 412
```

### 5.7 멱등성

```
POST /v1/rides
Idempotency-Key: 8a8c5f2e-7db7-4f9a-9c2d-b4f1d1a2c3e4
```

동일 키 + 동일 본문 → 24시간 내 캐시된 응답.
동일 키 + 다른 본문 → `409` + `error.code="idempotency_key_conflict"`.

---

## 6. 응답 형식 표준

### 6.1 Envelope

**성공**:

```json
{
    "data": { /* 또는 [...] */ },
    "pagination": { /* 컬렉션 응답에만 */ },
    "meta": {
        "request_id": "req_01H...",
        "generated_at": "2026-04-25T20:00:00Z",
        "server_version": "1.3.4"
    }
}
```

**실패**:

```json
{
    "error": {
        "code": "accessibility_mismatch",
        "message": "Requested vehicle does not meet rider's securement_points requirement.",
        "errors": [
            {"path": "vehicle.features.securement_points", "required": 6, "actual": 4}
        ],
        "doc_url": "https://wia.live/auto/errors/accessibility_mismatch"
    },
    "meta": { "request_id": "req_01H...", "generated_at": "2026-04-25T20:00:00Z" }
}
```

### 6.2 리소스 ID

ULID + 접두어:

| 리소스 | 접두어 |
|---|---|
| Rider | `rdr_` |
| Vehicle | `veh_` |
| Ride | `rd_` |
| Profile | `prf_` |
| Route | `rt_` |
| Emergency Incident | `em_` |

### 6.3 시각·좌표

시각: RFC 3339 UTC (`2026-04-25T20:00:00Z`) 또는 Phase 1의 `{unix_ms, iso8601}` 이중 표현.
좌표: WGS84 (`EPSG:4326`). 고정밀이 필요한 픽업 지점은 `ETRS89` 변형 보조 필드 허용.

---

## 7. 리소스 엔드포인트

| # | Method | Path | 설명 |
|:-:|---|---|---|
| 1 | `GET` | `/v1/vehicles` | 가용 차량 목록 (필터·정렬) |
| 2 | `GET` | `/v1/vehicles/{id}` | 차량 상세 |
| 3 | `POST` | `/v1/vehicles/{id}:reserve` | 차량 임시 홀드 (5분) |
| 4 | `GET` | `/v1/riders/{id}` | 라이더 기본 정보 |
| 5 | `GET` / `PUT` / `PATCH` | `/v1/riders/{id}/profile` | 접근성 프로필 |
| 6 | `POST` | `/v1/rides` | 탑승 요청 생성 |
| 7 | `GET` | `/v1/rides/{id}` | 탑승 상태 |
| 8 | `PATCH` | `/v1/rides/{id}` | 픽업 지점·선호도 수정 |
| 9 | `DELETE` | `/v1/rides/{id}` | 탑승 취소 (요금 정책 §8.4 적용) |
| 10 | `GET` | `/v1/rides/{id}/securement` | 휠체어 고정 상태 |
| 11 | `POST` | `/v1/rides/{id}/hmi:apply-preset` | HMI 프리셋 적용 |
| 12 | `POST` | `/v1/rides/{id}:emergency` | 비상 보고 (별도 스코프) |
| 13 | `GET` | `/v1/capabilities` | 서버 지원 플래그 |
| 14 | `GET` | `/v1/health` | 라이브니스·레디니스 |
| 15 | `GET` | `/v1/openapi.json` | OpenAPI 3.1 문서 |

---

## 8. 상세 API 스펙

### 8.1 가용 차량 조회

```
GET /v1/vehicles
   ?location[lat]=37.5665&location[lon]=126.9780&radius_m=2000
   &features.wheelchair_ramp=true
   &features.securement_points[gte]=4
   &sae_level[gte]=4
   &sort=-proximity_m
   &limit=20
Authorization: Bearer <token>
X-Client-Locale: ko-KR
```

응답 `200 OK`:

```json
{
    "data": [
        {
            "id": "veh_01H9XKZ2P1QWE7R8T6Y4U3I2OP",
            "model": "SmileStory Atlas L4",
            "sae_level": 4,
            "location": {
                "latitude": 37.56731,
                "longitude": 126.97921,
                "heading_deg": 87.2,
                "updated_at": "2026-04-25T19:59:58Z"
            },
            "features": {
                "wheelchair_ramp": true,
                "ramp_slope_deg_max": 7.0,
                "securement_points": 6,
                "securement_standard": "SAE_J2094",
                "audio_induction_loop": true,
                "braille_control_panel": true,
                "voice_control_languages": ["ko-KR", "en-US", "ja-JP"],
                "service_animal_accommodated": true,
                "oxygen_friendly": true,
                "kneel_system": true,
                "interior_luminance_lux_min": 150,
                "interior_luminance_lux_max": 500
            },
            "availability": {
                "status": "idle",
                "queue_length": 0,
                "eta_minutes": 3
            },
            "proximity_m": 180.4
        }
    ],
    "pagination": { "limit": 20, "next_cursor": null, "has_more": false },
    "meta": { "request_id": "req_01H...", "generated_at": "2026-04-25T20:00:00Z" }
}
```

### 8.2 접근성 프로필 갱신

```
PATCH /v1/riders/rdr_01H.../profile
Authorization: Bearer <token>
Content-Type: application/merge-patch+json
If-Match: W/"3"

{
    "disability_types": ["mobility_wheelchair_power"],
    "assistive_devices": [
        { "type": "power_wheelchair", "weight_kg": 135, "length_mm": 1200, "width_mm": 680 }
    ],
    "hmi_preferences": {
        "voice_language": "ko-KR",
        "voice_gender_preference": "any",
        "caption_font_size_pt": 24,
        "caption_contrast_ratio_min": 7.0,
        "haptic_enabled": true,
        "screen_reader_announcements_verbosity": "detailed"
    },
    "transport_notes": "출입구는 항상 측면 (rear-lift 사용 불가)"
}
```

응답 `200 OK` (갱신된 프로필 반환, `ETag` 증가).

### 8.3 탑승 요청 생성

```
POST /v1/rides
Authorization: Bearer <token>
Content-Type: application/json
Idempotency-Key: 8a8c5f2e-7db7-4f9a-9c2d-b4f1d1a2c3e4

{
    "rider_id": "rdr_01H...",
    "pickup": {
        "location": { "latitude": 37.5665, "longitude": 126.9780 },
        "address_text": "서울시 종로구 종로1가 1",
        "landmark_voice_cue": "세종문화회관 북측 장애인 승하차구역"
    },
    "dropoff": {
        "location": { "latitude": 37.5512, "longitude": 126.9882 },
        "address_text": "서울특별시 중구 명동길 26"
    },
    "vehicle_hard_requirements": {
        "securement_points_min": 6,
        "wheelchair_ramp_required": true,
        "voice_control_language": "ko-KR"
    },
    "vehicle_soft_preferences": {
        "service_animal_accommodated": true,
        "interior_luminance_lux_min": 200
    },
    "priority": "standard",
    "metadata": { "booked_via": "WIA-Move-App-2.3.1" }
}
```

응답 `201 Created` (탑승 할당 결과, 차량 ID·ETA·결제 견적).

### 8.4 탑승 취소 정책

| 시점 | 취소 수수료 |
|---|---|
| 차량 배차 전 | 0 |
| 배차 완료 ~ 픽업 5분 전 | 운임의 10% |
| 픽업 5분 이내 또는 이후 | 운임의 30% + 이동 비용 |

의료·법률 예외(응급 호출, 지연의 귀책이 플랫폼) 시 `X-Cancellation-Exception: medical|platform_fault` 헤더로 수수료 면제 요청.

### 8.5 비상 엔드포인트

```
POST /v1/rides/rd_01H.../:emergency
Authorization: Bearer <token-with-ride-emergency-scope>
Content-Type: application/json
X-Geolocation: 37.5605,126.9789

{
    "kind": "medical_distress",
    "severity": "high",
    "reporter": "rider",
    "free_text": "탑승자가 의식을 잃은 것 같음",
    "device_audio_available": true
}
```

응답: `202 Accepted` + `em_` 접두 ID. 서버는 즉시 (a) 가장 가까운 응급 디스패처에 전달, (b) 차량을 안전 정차 모드로, (c) 감사 로그에 변경 불가능한 엔트리를 기록한다.

비상 엔드포인트에는 별도 속도 제한(`10 req/min/token`)과 의무 감사(§14.4)가 적용된다.

### 8.6 HMI 프리셋 적용

```
POST /v1/rides/rd_01H.../hmi:apply-preset
Content-Type: application/json

{
    "preset": "high_contrast_voice_first",
    "overrides": {
        "caption_font_size_pt": 28,
        "audio_description_enabled": true
    }
}
```

차량 내 HMI는 WebSocket(§11) 이벤트 `hmi.state` 또는 다음 GET `/v1/rides/{id}`로 새 상태를 확인할 수 있다.

---

## 9. 에러 코드

| HTTP | `error.code` | 의미 |
|---|---|---|
| 400 | `invalid_request` | 구문 오류 |
| 401 | `invalid_token` · `token_expired` | 인증 실패 |
| 403 | `insufficient_scope` · `permission_denied` | 권한 |
| 404 | `resource_not_found` | 없음 |
| 409 | `idempotency_key_conflict` · `vehicle_reservation_conflict` | 충돌 |
| 412 | `etag_mismatch` | 낙관적 잠금 실패 |
| 422 | `validation_failed` · `accessibility_mismatch` | 스키마·정합성 |
| 429 | `rate_limit_exceeded` | §10 |
| 451 | `jurisdictional_restriction` | 해당 도시가 서비스 미제공 |
| 500 | `server_error` | 일반 |
| 503 | `service_unavailable` · `vehicle_fleet_offline` | 가용성 |

`accessibility_mismatch`(422)는 항상 `errors[].required`와 `errors[].actual`을 포함해 클라이언트가 사용자에게 무엇이 부족한지 설명할 수 있게 한다.

---

## 10. 속도 제한

| 인증 | 일반 엔드포인트 | 비상 엔드포인트 |
|---|---|---|
| OAuth User | 300 req/min | 10 req/min |
| API Key | 600 req/min | 30 req/min |
| 미인증 (health만) | 60 req/min | N/A |

응답 헤더: `X-RateLimit-Limit/Remaining/Reset`, `Retry-After`.

---

## 11. 실시간 WebSocket

탑승 진행 상황(차량 위치·ETA·HMI 상태·비상 플래그)을 저지연으로 수신.

```
GET /v1/ws/rides/rd_01H... 
Authorization: Bearer <token>
Sec-WebSocket-Protocol: wia-auto-v1
```

서버→클라이언트 메시지 종류:

| `type` | 페이로드 요약 |
|---|---|
| `ride.status_change` | `status`, `reason`, `eta_minutes` |
| `vehicle.location` | `lat`, `lon`, `heading_deg`, `speed_kmh` |
| `securement.state` | `is_locked`, `torque_nm`, `points_engaged` |
| `hmi.state` | 현재 적용 preset + overrides 스냅샷 |
| `emergency.declared` | `em_` 비상 레코드 ID |

하트비트: 15초 간격 `ping` / `pong`. 30초 내 미응답 시 연결 종료 `1008`.

---

## 12. OpenAPI 3.1 스펙

전체 문서는 `spec/openapi.yaml` 및 `GET /v1/openapi.json`. 발췌:

```yaml
openapi: 3.1.0
info:
  title: WIA Autonomous Vehicle Accessibility API
  version: 1.0.0
  license: { name: MIT }
servers:
  - url: https://api.wia.live/v1
paths:
  /vehicles:
    get:
      summary: List available vehicles
      parameters:
        - name: location[lat]
          in: query
          schema: { type: number, format: double }
        - name: location[lon]
          in: query
          schema: { type: number, format: double }
        - name: radius_m
          in: query
          schema: { type: integer, minimum: 100, maximum: 20000, default: 2000 }
        - name: features.securement_points[gte]
          in: query
          schema: { type: integer, minimum: 0, maximum: 8 }
      responses:
        '200':
          description: OK
          content:
            application/json:
              schema: { $ref: '#/components/schemas/VehicleList' }
  /rides:
    post:
      summary: Request a ride
      requestBody:
        required: true
        content:
          application/json:
            schema: { $ref: '#/components/schemas/RideRequest' }
      responses:
        '201':
          description: Created
          headers:
            Location: { schema: { type: string } }
components:
  securitySchemes:
    bearerAuth: { type: http, scheme: bearer, bearerFormat: JWT }
    apiKeyAuth: { type: apiKey, in: header, name: X-WIA-API-Key }
security: [ { bearerAuth: [] }, { apiKeyAuth: [] } ]
```

---

## 13. 클라이언트 예제

### 13.1 curl

```bash
# 차량 조회
curl -sS "https://api.wia.live/v1/vehicles?location[lat]=37.5665&location[lon]=126.9780&radius_m=2000&features.wheelchair_ramp=true&features.securement_points[gte]=6" \
     -H "Authorization: Bearer $WIA_TOKEN" | jq '.data[0]'

# 탑승 요청
curl -sS -X POST "https://api.wia.live/v1/rides" \
     -H "Authorization: Bearer $WIA_TOKEN" \
     -H "Content-Type: application/json" \
     -H "Idempotency-Key: $(uuidgen)" \
     -d @ride-request.json
```

### 13.2 JavaScript (fetch)

```javascript
const BASE = 'https://api.wia.live/v1';

async function findWheelchairVehicles({ lat, lon, radius = 2000, points = 4 }) {
    const q = new URLSearchParams({
        'location[lat]': lat,
        'location[lon]': lon,
        'radius_m': String(radius),
        'features.wheelchair_ramp': 'true',
        'features.securement_points[gte]': String(points),
        'sort': '-proximity_m',
    });
    const res = await fetch(`${BASE}/vehicles?${q}`, {
        headers: {
            'Authorization': `Bearer ${process.env.WIA_TOKEN}`,
            'X-Client-Locale': 'ko-KR',
            'X-Assistive-Tech': 'screen_reader',
        },
    });
    if (!res.ok) {
        const err = await res.json().catch(() => ({}));
        throw new Error(`HTTP ${res.status}: ${err?.error?.message ?? res.statusText}`);
    }
    return (await res.json()).data;
}

async function requestRide(body) {
    const res = await fetch(`${BASE}/rides`, {
        method: 'POST',
        headers: {
            'Authorization': `Bearer ${process.env.WIA_TOKEN}`,
            'Content-Type': 'application/json',
            'Idempotency-Key': crypto.randomUUID(),
        },
        body: JSON.stringify(body),
    });
    if (res.status !== 201) {
        const err = await res.json().catch(() => ({}));
        throw new Error(`HTTP ${res.status}: ${err?.error?.message ?? res.statusText}`);
    }
    return (await res.json()).data;
}
```

### 13.3 Python

```python
import os, uuid, requests

BASE = "https://api.wia.live/v1"
H = {
    "Authorization": f"Bearer {os.environ['WIA_TOKEN']}",
    "X-Client-Locale": "ko-KR",
    "Accept": "application/json",
}

def find_vehicles(lat, lon, radius_m=2000, securement_min=4):
    params = {
        "location[lat]": lat, "location[lon]": lon,
        "radius_m": radius_m,
        "features.wheelchair_ramp": "true",
        "features.securement_points[gte]": securement_min,
        "sort": "-proximity_m",
    }
    r = requests.get(f"{BASE}/vehicles", headers=H, params=params, timeout=15)
    r.raise_for_status()
    return r.json()["data"]

def request_ride(payload):
    headers = {**H, "Content-Type": "application/json", "Idempotency-Key": str(uuid.uuid4())}
    r = requests.post(f"{BASE}/rides", headers=headers, json=payload, timeout=30)
    r.raise_for_status()
    return r.json()["data"]
```

### 13.4 WebSocket (JS)

```javascript
const ws = new WebSocket(`wss://api.wia.live/v1/ws/rides/${rideId}`, 'wia-auto-v1');
ws.addEventListener('open', () => {
    ws.send(JSON.stringify({ type: 'auth', token: process.env.WIA_TOKEN }));
});
ws.addEventListener('message', (evt) => {
    const msg = JSON.parse(evt.data);
    if (msg.type === 'vehicle.location') {
        updateMapPin(msg.data);
    } else if (msg.type === 'emergency.declared') {
        alertEmergencyUI(msg.data.em_id);
    }
});
```

---

## 14. 접근성 관점의 API 설계 특수사항

### 14.1 하드 요구와 소프트 선호의 구분

`vehicle_hard_requirements`는 하나라도 미충족 시 차량 매칭 실패(422). `vehicle_soft_preferences`는 매칭 스코어에만 영향을 주고, 최상의 매칭도 미충족일 수 있다. 클라이언트 UI는 이 둘을 시각적으로 분리해야 한다.

### 14.2 부분 매칭 설명 헤더

`X-Matched-Accessibility-Gap` 헤더는 차량이 소프트 선호 중 어느 것을 충족하지 못했는지 구조화된 값으로 알린다:

```
X-Matched-Accessibility-Gap: service_animal_accommodated=false; interior_luminance_lux_min=achieved(120) < preferred(200)
```

이를 바탕으로 클라이언트는 사용자에게 "이 차량은 반려견 동반은 어렵습니다" 같은 안내를 TTS로 전달할 수 있다.

### 14.3 오디오 설명(AD) 동기화

HMI 프리셋으로 `audio_description_enabled=true`가 활성화된 경우, 차량은 주요 주행 이벤트(차선 변경·정차·문 개폐)마다 오디오 설명을 TTS로 내보낸다. 본 API의 `hmi.state` WebSocket 이벤트에 `ad_last_spoken_at` 타임스탬프가 포함되어 클라이언트 자막과 동기화할 수 있다.

### 14.4 감사 로그 불변성

비상 엔드포인트와 프로필 PATCH는 append-only 감사 저장소에 해시 체인으로 기록한다. `GET /v1/rides/{id}/audit` (스코프 `ride:read` + `dispatch:admin`)로 조회 가능. 각 엔트리는 `prev_hash`·`payload_hash`를 포함해 무결성 검증을 외부에서 수행할 수 있다.

### 14.5 언어·스크립트 일관성

API 응답의 자유 텍스트 필드(`address_text`, `landmark_voice_cue` 등)는 요청 시 `X-Client-Locale` 헤더 또는 `?locale=` 파라미터 값을 우선한다. 해당 locale로 기록된 텍스트가 없으면 차량 주행 지역의 기본 locale로 폴백하며, 이때 응답 `meta.locale_used`에 실제 사용된 locale을 표기한다.

---

## 15. 참고문헌

- **SAE J3016** — Taxonomy and Definitions for Terms Related to Driving Automation Systems
- **SAE J2094** — Wheelchair Tiedown and Occupant Restraint Systems
- **49 CFR Part 38** — ADA Accessibility Specifications for Transportation Vehicles
- **RFC 9110** — HTTP Semantics
- **RFC 7636** — OAuth 2.0 Proof Key for Code Exchange (PKCE)
- **RFC 7519** — JSON Web Token (JWT)
- **RFC 7396** — JSON Merge Patch
- **RFC 3339** — Date and Time
- **OpenAPI 3.1.0** — OpenAPI Initiative
- **ULID** — https://github.com/ulid/spec
- WIA Autonomous Vehicle Accessibility Phase 1 (`spec/PHASE-1-DATA-FORMAT.md`)
- WIA Autonomous Vehicle Accessibility Phase 3 (`spec/PHASE-3-PROTOCOL.md`)
- WIA Autonomous Vehicle Accessibility Phase 4 (`spec/PHASE-4-INTEGRATION.md`)

---

**弘益人間 · Benefit All Humanity**
