# Phase 2: REST API Interface Specification
# WIA Climate REST API 표준

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2026-04-25
**Authors**: WIA (World Industry Authentication Association) / SmileStory Inc.
**License**: MIT

---

## 목차 (Table of Contents)

1. [개요](#1-개요)
2. [설계 원칙](#2-설계-원칙)
3. [버전 관리와 기본 URL](#3-버전-관리와-기본-url)
4. [인증과 인가](#4-인증과-인가)
5. [공통 규약](#5-공통-규약)
6. [응답 형식 표준](#6-응답-형식-표준)
7. [엔드포인트 목록](#7-엔드포인트-목록)
8. [도메인별 상세 스펙](#8-도메인별-상세-스펙)
9. [에러 코드](#9-에러-코드)
10. [속도 제한](#10-속도-제한)
11. [OpenAPI 3.1 스펙](#11-openapi-31-스펙)
12. [클라이언트 예제](#12-클라이언트-예제)
13. [CORS와 브라우저 호환성](#13-cors와-브라우저-호환성)
14. [준수 수준](#14-준수-수준)
15. [참고문헌](#15-참고문헌)

---

## 1. 개요

### 1.1 목적

WIA Climate REST API는 Phase 1에서 정의한 기후/환경 데이터 형식을 HTTP 기반으로 송수신하기 위한 표준 인터페이스입니다. 본 스펙은 서버 구현자와 클라이언트 구현자가 상호운용 가능한 방식으로 데이터를 주고받기 위해 따라야 할 엔드포인트, 요청/응답 규약, 에러 처리, 인증 체계를 정의합니다.

### 1.2 범위

본 스펙이 다루는 것:

- HTTP/1.1 및 HTTP/2 기반 동기식 요청/응답
- JSON 페이로드 (`Content-Type: application/json; charset=utf-8`)
- 6개 도메인(Phase 1 §1.2 참조)에 대한 CRUD 및 조회 엔드포인트
- Bearer Token / API Key 인증
- 표준 페이지네이션, 필터링, 정렬
- 멱등성과 캐싱 규약

본 스펙이 다루지 않는 것 (다른 Phase 참조):

- WebSocket/MQTT 기반 비동기 스트림 → Phase 3
- 파일 시스템/아카이브 형식 → Phase 4
- 데이터 내부 구조 → Phase 1

### 1.3 레이어 관계

```
┌─────────────────────────────────────────────────────────────────┐
│                     Client Application                           │
│                (Web · Mobile · Embedded · SDK)                   │
├─────────────────────────────────────────────────────────────────┤
│              Phase 2 REST API (this specification)               │
│           (HTTP verbs · resources · envelopes · auth)            │
├─────────────────────────────────────────────────────────────────┤
│               Phase 1 Data Format (Payload body)                 │
│            (ClimateMessage with validated schema)                │
├─────────────────────────────────────────────────────────────────┤
│                    HTTP/1.1 or HTTP/2                            │
│                  (TLS 1.2+ REQUIRED in prod)                     │
└─────────────────────────────────────────────────────────────────┘
```

REST API는 하나의 **게이트웨이**로서 Phase 1 데이터를 CRUD 형태로 노출하며, Phase 3 스트리밍 세션의 초기 인증·구독 설정을 위한 REST Fallback도 제공합니다.

### 1.4 핵심 용어

| 용어 | 정의 |
|------|------|
| **Resource** | URI로 식별되는 개별 데이터 객체 (예: `/v1/carbon-capture/evt_01H9…`) |
| **Collection** | 동종 Resource의 집합 (예: `/v1/carbon-capture`) |
| **Envelope** | 모든 응답이 공통으로 갖는 바깥 구조 (§6.1) |
| **Idempotency Key** | 동일 요청을 여러 번 보내도 한 번만 반영되게 하는 클라이언트 토큰 |
| **ETag** | 리소스 버전 태그, 캐싱 및 낙관적 동시성 제어에 사용 |

---

## 2. 설계 원칙

1. **Resource-Oriented** — 모든 도메인은 집합 리소스(collection)와 개별 리소스(item)로 표현된다. 동사는 URL에 등장하지 않는다.
2. **HTTP 표준 준수** — RFC 7230–7235, RFC 9110, RFC 9111을 따른다. 상태 코드는 `4xx`는 클라이언트, `5xx`는 서버.
3. **JSON Only (in·out)** — 요청/응답 본문은 UTF-8 JSON. 이진 데이터는 `data:` URL 또는 별도 업로드 엔드포인트(`/v1/ingest/bulk`) 경유.
4. **Stateless** — 서버는 세션을 저장하지 않는다. 모든 요청은 자기 서술적(self-descriptive)이어야 한다.
5. **Cacheable** — 읽기 요청은 `ETag`와 `Cache-Control`을 반환하여 조건부 요청(`If-None-Match`)을 지원.
6. **Backward-Compatible Versioning** — 메이저 버전은 URL 경로(`/v1`, `/v2`), 마이너/패치는 동일 경로에서 비파괴적으로 추가.
7. **Fail Fast with Context** — 오류 응답은 기계가 분기할 수 있는 `code`와 사람이 읽을 수 있는 `message`, 그리고 필드별 상세(`errors[]`)를 모두 포함.

---

## 3. 버전 관리와 기본 URL

### 3.1 URL 구조

```
https://{host}/{version}/{domain}[/{id}]

예시:
https://api.wia.live/v1/carbon-capture
https://api.wia.live/v1/carbon-capture/evt_01H9XKZ2P1QWE7R8T6Y4U3I2OP
https://api.wia.live/v1/ocean-cleanup?region=pacific&limit=50
```

- `{host}`: 구현체가 정하는 도메인. 프로덕션에서는 **TLS 필수**.
- `{version}`: 본 스펙 기준 `v1`. 메이저 변경 시에만 증가.
- `{domain}`: kebab-case. Phase 1 §1.2의 6개 도메인과 1:1 대응.

### 3.2 도메인 매핑

| Phase 1 type | URL segment |
|---|---|
| `carbon_capture` | `/v1/carbon-capture` |
| `weather_control` | `/v1/weather-control` |
| `geoengineering` | `/v1/geoengineering` |
| `vertical_farming` | `/v1/vertical-farming` |
| `ocean_cleanup` | `/v1/ocean-cleanup` |
| `climate_model` | `/v1/climate-model` |

### 3.3 버전 호환성 정책

- **추가는 허용**: 새 필드, 새 엔드포인트, 새 쿼리 파라미터는 기존 클라이언트를 깨지 않는다.
- **제거/형식 변경은 금지**: 필드 제거, 타입 변경, 필수 여부 변경은 새 메이저 버전(`v2`)에서만.
- **`Deprecation` 헤더**: 폐기 예정 엔드포인트는 최소 **180일 전** 응답에 `Deprecation: true`와 `Sunset: <RFC 7231 date>`를 포함해야 한다.

---

## 4. 인증과 인가

### 4.1 지원 인증 방식

본 스펙은 두 가지 인증 방식을 정의한다. 서버는 **최소 하나 이상**을 구현해야 하며, 둘 다 지원을 권장한다.

#### 4.1.1 Bearer Token (RECOMMENDED)

```
Authorization: Bearer <jwt_or_opaque_token>
```

- 토큰은 JWT(RFC 7519) 또는 서버 발급 불투명 토큰.
- JWT 사용 시 다음 클레임을 포함해야 한다: `iss`, `sub`, `aud`, `exp`, `iat`.
- 토큰 수명(`exp` - `iat`)은 **1시간 이내** 권장. 갱신은 OAuth 2.0 refresh token(RFC 6749)에 따른다.

#### 4.1.2 API Key

```
X-WIA-API-Key: <key_string>
```

- 기계 간 통신용. 사용자 컨텍스트가 없는 디바이스/배치 작업에 적합.
- 키는 최소 **32 바이트 엔트로피**. 서버는 해시 저장(bcrypt/argon2).

### 4.2 권한 범위 (Scopes)

| Scope | 의미 |
|---|---|
| `climate:read` | 모든 도메인 GET 허용 |
| `climate:write` | POST/PUT/PATCH 허용 |
| `climate:delete` | DELETE 허용 |
| `climate:admin` | 통계·감사 로그 접근 |
| `{domain}:read` / `{domain}:write` | 도메인별 세분화 (예: `carbon-capture:write`) |

권한 부족 시 `403 Forbidden` + `error.code = "permission_denied"`.

### 4.3 미인증 요청 처리

| 상황 | 응답 |
|---|---|
| `Authorization` 헤더 없음 | `401` + `WWW-Authenticate: Bearer realm="wia-climate"` |
| 토큰 서명 검증 실패 | `401` + `error.code = "invalid_token"` |
| 토큰 만료 | `401` + `error.code = "token_expired"` |
| 권한 부족 | `403` + `error.code = "permission_denied"` |

---

## 5. 공통 규약

### 5.1 HTTP 메서드 의미

| 메서드 | 의미 | 멱등 | 캐시 가능 |
|---|---|:-:|:-:|
| `GET` | 리소스 조회 | ✓ | ✓ |
| `POST` | 리소스 생성 또는 액션 트리거 | ✗ | ✗ |
| `PUT` | 리소스 전체 대체 | ✓ | ✗ |
| `PATCH` | 리소스 부분 수정 (RFC 7396 JSON Merge Patch) | ✗ | ✗ |
| `DELETE` | 리소스 삭제 | ✓ | ✗ |

### 5.2 필수 요청 헤더

| 헤더 | 값 | 비고 |
|---|---|---|
| `Authorization` | `Bearer <token>` | §4.1.1 |
| `Accept` | `application/json` | 생략 시 기본값 |
| `Content-Type` | `application/json; charset=utf-8` | 본문 있을 때 필수 |
| `X-Request-ID` | UUID v4 | OPTIONAL. 서버가 생성·반사 가능 |
| `Idempotency-Key` | 임의 문자열 (≤ 255자) | POST에 강력 권장, 24시간 중복 방지 |

### 5.3 필수 응답 헤더

| 헤더 | 값 |
|---|---|
| `Content-Type` | `application/json; charset=utf-8` |
| `X-Request-ID` | 요청 ID 반사 (또는 서버 생성) |
| `X-RateLimit-Limit` | 현재 창의 최대 요청 수 |
| `X-RateLimit-Remaining` | 남은 요청 수 |
| `X-RateLimit-Reset` | 리셋까지 초 (RFC 7231 delta-seconds) |
| `ETag` | GET·PUT·PATCH 응답에서 리소스 버전 |

### 5.4 페이지네이션

본 스펙은 **커서 기반(cursor)** 페이지네이션을 기본으로, **오프셋(offset)**을 대안으로 허용한다.

#### 5.4.1 커서 기반 (권장)

```
GET /v1/carbon-capture?limit=50&cursor=eyJ0IjoxNzAyNDY4ODAwMDAwfQ
```

| 파라미터 | 타입 | 기본 | 최대 | 설명 |
|---|---|---|---|---|
| `limit` | integer | 50 | 500 | 반환할 항목 수 |
| `cursor` | string | - | - | 이전 응답의 `pagination.next_cursor` |

응답:

```json
{
    "data": [ /* 항목들 */ ],
    "pagination": {
        "limit": 50,
        "next_cursor": "eyJ0IjoxNzAyNDcwNDAwMDAwfQ",
        "has_more": true
    }
}
```

커서는 **불투명 문자열**로 취급한다. 클라이언트는 파싱하지 않는다.

#### 5.4.2 오프셋 (대안)

```
GET /v1/carbon-capture?limit=50&offset=100
```

오프셋 방식은 대용량 컬렉션에서 성능이 저하되므로, 서버는 오프셋 상한을 두고 초과 시 `400`과 함께 커서 사용을 권고해야 한다.

### 5.5 필터링

쿼리 파라미터는 `필드=값` 형식. 복수 값은 콤마. 범위는 `필드[gte]=값&필드[lte]=값`.

```
GET /v1/carbon-capture?technology=dac,post_combustion&capture_rate_kg_per_hour[gte]=100
```

서버는 필수적으로 다음 공통 필터를 지원해야 한다:

| 필터 | 적용 필드 |
|---|---|
| `timestamp[gte]`, `timestamp[lte]` | Phase 1 `timestamp.unix_ms` |
| `location[bbox]=minLon,minLat,maxLon,maxLat` | Phase 1 `location` |
| `device.serial` | Phase 1 `device.serial` |
| `meta.quality_score[gte]` | Phase 1 `meta.quality_score` |

### 5.6 정렬

```
GET /v1/carbon-capture?sort=-timestamp.unix_ms,meta.quality_score
```

- `-` 접두: 내림차순. 없으면 오름차순.
- 기본 정렬: `-timestamp.unix_ms` (최신 우선).
- 최대 **3개 필드**까지 복합 정렬 지원.

### 5.7 필드 선택 (Sparse Fieldsets)

네트워크 최적화가 필요할 때:

```
GET /v1/carbon-capture?fields=timestamp,data.capture_rate_kg_per_hour
```

서버는 요청된 경로만 응답 `data`에 포함한다. 필수 시스템 필드(`id`, `version`)는 항상 포함.

### 5.8 조건부 요청 (ETag)

```
GET /v1/carbon-capture/evt_01H9XKZ2P1
ETag: "W/\"abc123\""

# 클라이언트 재요청 시
GET /v1/carbon-capture/evt_01H9XKZ2P1
If-None-Match: "W/\"abc123\""

# 변경 없음
304 Not Modified
```

PUT/PATCH에도 `If-Match`를 지원하여 낙관적 동시성 제어 수행:

```
PATCH /v1/carbon-capture/evt_01H9XKZ2P1
If-Match: "W/\"abc123\""

# ETag 불일치
412 Precondition Failed
```

### 5.9 멱등성 (Idempotency)

POST·비멱등 작업에 `Idempotency-Key` 헤더 지원 필수. 서버는 **24시간** 동안 동일 키+동일 요청 해시에 대해 캐시된 응답을 반환한다.

```
POST /v1/carbon-capture
Idempotency-Key: 8a8c5f2e-…-b4f1
Content-Type: application/json

{ /* ClimateMessage */ }
```

동일 키 + 다른 요청 본문 → `409 Conflict` + `error.code = "idempotency_key_conflict"`.

---

## 6. 응답 형식 표준

### 6.1 Envelope

모든 응답은 다음 envelope을 따른다.

**성공 (2xx)**:

```json
{
    "data": { /* 또는 [ … ] */ },
    "pagination": { /* 컬렉션 응답에만 */ },
    "meta": {
        "request_id": "req_01H9…",
        "generated_at": "2026-04-25T09:15:30.123Z",
        "server_version": "1.3.2"
    }
}
```

**실패 (4xx, 5xx)**:

```json
{
    "error": {
        "code": "validation_failed",
        "message": "One or more fields failed validation.",
        "errors": [
            {
                "path": "data.capture_rate_kg_per_hour",
                "code": "out_of_range",
                "message": "must be >= 0"
            }
        ],
        "doc_url": "https://wia.live/climate/errors/validation_failed"
    },
    "meta": {
        "request_id": "req_01H9…",
        "generated_at": "2026-04-25T09:15:30.123Z"
    }
}
```

### 6.2 리소스 ID 포맷

신규 생성 리소스의 `id` 필드는 **ULID**(Crockford Base32, 26자) 권장. 접두어로 도메인 약어를 사용한다.

| 도메인 | 접두어 | 예시 |
|---|---|---|
| carbon-capture | `cc_` | `cc_01H9XKZ2P1QWE7R8T6Y4U3I2OP` |
| weather-control | `wc_` | `wc_01H9XKZ2P1QWE7R8T6Y4U3I2OP` |
| geoengineering | `ge_` | `ge_01H9XKZ2P1QWE7R8T6Y4U3I2OP` |
| vertical-farming | `vf_` | `vf_01H9XKZ2P1QWE7R8T6Y4U3I2OP` |
| ocean-cleanup | `oc_` | `oc_01H9XKZ2P1QWE7R8T6Y4U3I2OP` |
| climate-model | `cm_` | `cm_01H9XKZ2P1QWE7R8T6Y4U3I2OP` |

### 6.3 타임스탬프

응답 내 모든 시각은 **RFC 3339** ISO 8601 문자열 + `Z` 접미사 (UTC)로 표현한다. Phase 1 데이터 페이로드 내부는 PHASE-1 §3.2의 `{unix_ms, iso8601}` 이중 표현을 유지한다.

---

## 7. 엔드포인트 목록

모든 도메인은 아래 8개 표준 오퍼레이션을 동일한 패턴으로 제공한다. 도메인별 확장은 §8에서 정의.

| # | Method | Path | 설명 | 성공 코드 |
|---|---|---|---|---|
| 1 | `GET` | `/v1/{domain}` | 목록 조회 (필터·페이지) | `200` |
| 2 | `POST` | `/v1/{domain}` | 신규 생성 | `201` |
| 3 | `GET` | `/v1/{domain}/{id}` | 단일 조회 | `200` / `304` |
| 4 | `PUT` | `/v1/{domain}/{id}` | 전체 대체 | `200` |
| 5 | `PATCH` | `/v1/{domain}/{id}` | 부분 수정 (JSON Merge Patch) | `200` |
| 6 | `DELETE` | `/v1/{domain}/{id}` | 삭제 (soft by default) | `204` |
| 7 | `POST` | `/v1/{domain}:batch` | 최대 1000건 일괄 생성 | `207` |
| 8 | `GET` | `/v1/{domain}/stats` | 집계 통계 | `200` |

또한 도메인 횡단 엔드포인트:

| # | Method | Path | 설명 |
|---|---|---|---|
| A | `POST` | `/v1/ingest/bulk` | 이종 도메인 혼합 일괄 업로드 (NDJSON, 최대 100 MB) |
| B | `GET` | `/v1/search?q=...` | 전 도메인 전문 검색 |
| C | `GET` | `/v1/health` | 라이브니스/레디니스 (인증 불요) |
| D | `GET` | `/v1/openapi.json` | 현재 구현의 OpenAPI 3.1 스펙 반환 |

---

## 8. 도메인별 상세 스펙

도메인 8개 오퍼레이션의 스키마는 Phase 1 §4의 JSON Schema를 그대로 참조한다. 본 섹션에서는 **carbon-capture를 참조 구현**으로 완전히 서술하고, 나머지 도메인은 차이점만 명시한다.

### 8.1 Carbon Capture 참조 구현

#### 8.1.1 리스트 조회

```
GET /v1/carbon-capture?technology=dac&timestamp[gte]=2026-01-01T00:00:00Z&limit=100
Authorization: Bearer <token>
Accept: application/json
```

응답 `200 OK`:

```json
{
    "data": [
        {
            "id": "cc_01H9XKZ2P1QWE7R8T6Y4U3I2OP",
            "version": "1.0.0",
            "type": "carbon_capture",
            "timestamp": {
                "unix_ms": 1702468800000,
                "iso8601": "2026-01-14T00:00:00Z"
            },
            "location": {
                "latitude": 37.5665,
                "longitude": 126.9780,
                "altitude_m": 38.0,
                "crs": "EPSG:4326"
            },
            "device": {
                "manufacturer": "SmileStory",
                "model": "DAC-100",
                "serial": "DAC100-0042"
            },
            "data": {
                "technology": "dac",
                "capture_rate_kg_per_hour": 125.5,
                "co2_concentration_ppm": 420.3,
                "co2_purity_percentage": 99.2,
                "energy_consumption_kwh": 340.0
            },
            "meta": {
                "quality_score": 0.98,
                "uncertainty": 0.02,
                "source": "primary_sensor"
            }
        }
    ],
    "pagination": {
        "limit": 100,
        "next_cursor": "eyJ0IjoxNzAyNTU1MjAwMDAwfQ",
        "has_more": true
    },
    "meta": {
        "request_id": "req_01H9…",
        "generated_at": "2026-04-25T09:15:30.123Z",
        "server_version": "1.3.2"
    }
}
```

#### 8.1.2 생성

```
POST /v1/carbon-capture
Authorization: Bearer <token>
Content-Type: application/json; charset=utf-8
Idempotency-Key: 8a8c5f2e-7db7-4f9a-9c2d-b4f1d1a2c3e4

{
    "version": "1.0.0",
    "type": "carbon_capture",
    "timestamp": { "unix_ms": 1702468800000, "iso8601": "2026-01-14T00:00:00Z" },
    "location": { "latitude": 37.5665, "longitude": 126.9780, "altitude_m": 38.0, "crs": "EPSG:4326" },
    "device": { "manufacturer": "SmileStory", "model": "DAC-100", "serial": "DAC100-0042" },
    "data": {
        "technology": "dac",
        "capture_rate_kg_per_hour": 125.5,
        "co2_concentration_ppm": 420.3
    },
    "meta": { "quality_score": 0.98, "uncertainty": 0.02, "source": "primary_sensor" }
}
```

응답 `201 Created`:

```
Location: /v1/carbon-capture/cc_01H9XKZ2P1QWE7R8T6Y4U3I2OP
ETag: W/"1"

{ "data": { /* 생성된 리소스, id 포함 */ }, "meta": { … } }
```

#### 8.1.3 단일 조회 / 수정 / 삭제

```
GET    /v1/carbon-capture/cc_01H9XKZ2P1QWE7R8T6Y4U3I2OP
PATCH  /v1/carbon-capture/cc_01H9XKZ2P1QWE7R8T6Y4U3I2OP   (Content-Type: application/merge-patch+json)
PUT    /v1/carbon-capture/cc_01H9XKZ2P1QWE7R8T6Y4U3I2OP
DELETE /v1/carbon-capture/cc_01H9XKZ2P1QWE7R8T6Y4U3I2OP
```

DELETE는 기본적으로 **soft delete**이며, `?hard=true` 파라미터로 물리 삭제를 요청할 수 있다. 물리 삭제는 `climate:delete` + `climate:admin` 두 스코프 모두 필요.

#### 8.1.4 일괄 생성

```
POST /v1/carbon-capture:batch
Content-Type: application/json

{
    "items": [ /* ClimateMessage 배열, 최대 1000 */ ]
}
```

응답 `207 Multi-Status`:

```json
{
    "data": [
        { "index": 0, "status": 201, "id": "cc_01H9…", "etag": "W/\"1\"" },
        { "index": 1, "status": 422, "error": { "code": "validation_failed", "message": "…" } }
    ],
    "meta": { "total": 2, "succeeded": 1, "failed": 1, "request_id": "req_…" }
}
```

#### 8.1.5 집계 통계

```
GET /v1/carbon-capture/stats?group_by=technology&interval=day&timestamp[gte]=2026-01-01T00:00:00Z
```

응답:

```json
{
    "data": {
        "group_by": "technology",
        "interval": "day",
        "buckets": [
            {
                "key": "dac",
                "time_start": "2026-01-01T00:00:00Z",
                "time_end": "2026-01-02T00:00:00Z",
                "count": 1440,
                "sum_capture_rate_kg_per_hour": 180720.0,
                "avg_capture_rate_kg_per_hour": 125.5
            }
        ]
    }
}
```

지원 `interval`: `minute`, `hour`, `day`, `week`, `month`.

### 8.2 Weather Control

carbon-capture와 동일한 8개 오퍼레이션. 차이:

- `data` 스키마는 Phase 1 `weather_control` 데이터를 따른다 (`technique`, `target_area_km2`, `duration_minutes` 등).
- 민감 도메인이므로 POST/PUT/DELETE는 `climate:admin` 스코프 필수.
- 통계 `group_by`에 `technique`가 추가된다.

### 8.3 Geoengineering

- `data` 스키마: `approach`, `deployment_status`, `risk_level` 등.
- `deployment_status = "active"`인 레코드 수정/삭제는 **이중 승인** 필요 (§4.2 `climate:admin` + `X-Confirmation-Token` 헤더). 확인 토큰은 별도 `POST /v1/geoengineering/{id}:request-confirmation`로 발급.

### 8.4 Vertical Farming

- `data` 스키마: `crop_type`, `growth_stage`, `light_spectrum` 등.
- 추가 필터: `crop_type`, `growth_stage[gte]`.
- 고빈도 데이터이므로 리스트 엔드포인트는 기본 `limit=100`, 최대 `1000`.

### 8.5 Ocean Cleanup

- `data` 스키마: `cleanup_zone`, `plastic_collected_kg`, `marine_life_impact_score` 등.
- 좌표 필터 `location[polygon]=lon1,lat1;lon2,lat2;…`으로 임의 다각형 영역 조회 지원.

### 8.6 Climate Model

- `data` 스키마: `model_name`, `scenario`, `time_horizon_years`, `temperature_change_celsius` 등.
- 대용량 배열을 포함할 수 있으므로, 단일 조회 응답은 `?expand=false`(기본)일 때 메타데이터만, `?expand=true`일 때 전체 배열을 반환.

---

## 9. 에러 코드

### 9.1 HTTP 상태 매핑

| 상태 | 의미 | 예 |
|---|---|---|
| `400 Bad Request` | 잘못된 요청 구문 | 잘못된 JSON, 쿼리 형식 |
| `401 Unauthorized` | 미인증 | 토큰 없음·만료·서명 오류 |
| `403 Forbidden` | 권한 부족 | 스코프 부족, 소유자 아님 |
| `404 Not Found` | 리소스 없음 | 존재하지 않는 `id` |
| `405 Method Not Allowed` | 허용되지 않은 메서드 | `/v1/health`에 POST 등 |
| `409 Conflict` | 충돌 | idempotency key 불일치 |
| `412 Precondition Failed` | ETag 불일치 | `If-Match` 실패 |
| `413 Content Too Large` | 본문 초과 | batch > 1000, body > 10 MB |
| `415 Unsupported Media Type` | 잘못된 Content-Type | XML 등 |
| `422 Unprocessable Entity` | 스키마 위반 | Phase 1 스키마 검증 실패 |
| `429 Too Many Requests` | 속도 제한 | §10 |
| `500 Internal Server Error` | 서버 오류 | 일반 예외 |
| `503 Service Unavailable` | 일시 장애 | 유지보수, 과부하 |
| `504 Gateway Timeout` | 타임아웃 | 업스트림 미응답 |

### 9.2 응답 `error.code` 목록

| 코드 | 상태 | 의미 |
|---|---|---|
| `invalid_request` | 400 | 구문 오류 |
| `invalid_token` | 401 | 토큰 검증 실패 |
| `token_expired` | 401 | 토큰 만료 |
| `permission_denied` | 403 | 스코프 부족 |
| `resource_not_found` | 404 | id 존재 안 함 |
| `method_not_allowed` | 405 | 메서드 불허 |
| `idempotency_key_conflict` | 409 | 동일 키, 다른 본문 |
| `etag_mismatch` | 412 | 낙관적 잠금 실패 |
| `payload_too_large` | 413 | 크기 초과 |
| `unsupported_media_type` | 415 | Content-Type 불가 |
| `validation_failed` | 422 | 스키마 필드 오류 |
| `rate_limit_exceeded` | 429 | 속도 제한 |
| `server_error` | 500 | 내부 오류 |
| `service_unavailable` | 503 | 일시 장애 |

---

## 10. 속도 제한

### 10.1 기본 한도

| 인증 | 한도 | 창 |
|---|---|---|
| API Key | 600 req/min | slideing window |
| Bearer (user) | 300 req/min | sliding window |
| 미인증 (`/v1/health`만) | 60 req/min | IP 기준 |

### 10.2 응답 헤더

```
X-RateLimit-Limit: 600
X-RateLimit-Remaining: 572
X-RateLimit-Reset: 34
```

### 10.3 초과 시 동작

`429 Too Many Requests` + `Retry-After: <seconds>` 헤더.

```json
{
    "error": {
        "code": "rate_limit_exceeded",
        "message": "API rate limit exceeded. Retry after 34 seconds.",
        "doc_url": "https://wia.live/climate/errors/rate_limit_exceeded"
    }
}
```

---

## 11. OpenAPI 3.1 스펙

서버는 자신의 구현을 서술하는 OpenAPI 3.1 문서를 `GET /v1/openapi.json`으로 반드시 제공해야 한다. 아래는 carbon-capture 발췌. 전체 문서는 `spec/schemas/openapi-climate-v1.yaml`에 참조 구현이 포함된다.

```yaml
openapi: 3.1.0
info:
  title: WIA Climate REST API
  version: 1.0.0
  license:
    name: MIT
servers:
  - url: https://api.wia.live/v1
paths:
  /carbon-capture:
    get:
      summary: List carbon capture records
      operationId: listCarbonCapture
      parameters:
        - $ref: '#/components/parameters/Limit'
        - $ref: '#/components/parameters/Cursor'
        - name: technology
          in: query
          schema:
            type: string
            enum: [dac, post_combustion, pre_combustion, oxy_fuel, bioenergy_ccs]
      responses:
        '200':
          description: OK
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/CarbonCaptureList'
    post:
      summary: Create a carbon capture record
      operationId: createCarbonCapture
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/CarbonCaptureMessage'
      responses:
        '201':
          description: Created
          headers:
            Location:
              schema:
                type: string
components:
  securitySchemes:
    bearerAuth:
      type: http
      scheme: bearer
      bearerFormat: JWT
    apiKeyAuth:
      type: apiKey
      in: header
      name: X-WIA-API-Key
  parameters:
    Limit:
      name: limit
      in: query
      schema:
        type: integer
        minimum: 1
        maximum: 500
        default: 50
    Cursor:
      name: cursor
      in: query
      schema:
        type: string
  schemas:
    CarbonCaptureMessage:
      $ref: 'https://wia.live/climate/data/v1/carbon-capture.schema.json'
    CarbonCaptureList:
      type: object
      required: [data, meta]
      properties:
        data:
          type: array
          items:
            $ref: '#/components/schemas/CarbonCaptureMessage'
        pagination:
          $ref: '#/components/schemas/Pagination'
        meta:
          $ref: '#/components/schemas/Meta'
security:
  - bearerAuth: []
  - apiKeyAuth: []
```

---

## 12. 클라이언트 예제

아래 예제는 **복사-붙여넣기로 실행 가능**하며, `$API_TOKEN` 환경 변수에 유효한 Bearer 토큰이 설정되어 있다고 가정한다.

### 12.1 curl

```bash
# 리스트 조회
curl -sS "https://api.wia.live/v1/carbon-capture?limit=10&technology=dac" \
     -H "Authorization: Bearer $API_TOKEN" \
     -H "Accept: application/json" | jq '.data[0]'

# 신규 생성
curl -sS -X POST "https://api.wia.live/v1/carbon-capture" \
     -H "Authorization: Bearer $API_TOKEN" \
     -H "Content-Type: application/json; charset=utf-8" \
     -H "Idempotency-Key: $(uuidgen)" \
     -d '{
         "version": "1.0.0",
         "type": "carbon_capture",
         "timestamp": { "unix_ms": 1702468800000, "iso8601": "2026-01-14T00:00:00Z" },
         "location": { "latitude": 37.5665, "longitude": 126.9780, "altitude_m": 38.0, "crs": "EPSG:4326" },
         "device": { "manufacturer": "SmileStory", "model": "DAC-100", "serial": "DAC100-0042" },
         "data": { "technology": "dac", "capture_rate_kg_per_hour": 125.5, "co2_concentration_ppm": 420.3 },
         "meta": { "quality_score": 0.98, "uncertainty": 0.02, "source": "primary_sensor" }
     }'
```

### 12.2 JavaScript (fetch, Node 20+ / 모던 브라우저)

```javascript
const BASE = 'https://api.wia.live/v1';
const TOKEN = process.env.API_TOKEN;

async function listCarbonCapture({ technology, limit = 50, cursor } = {}) {
    const q = new URLSearchParams();
    if (technology) q.set('technology', technology);
    q.set('limit', String(limit));
    if (cursor) q.set('cursor', cursor);

    const res = await fetch(`${BASE}/carbon-capture?${q}`, {
        headers: {
            'Authorization': `Bearer ${TOKEN}`,
            'Accept': 'application/json',
        },
    });
    if (!res.ok) {
        const err = await res.json().catch(() => ({}));
        throw new Error(`HTTP ${res.status}: ${err?.error?.message ?? res.statusText}`);
    }
    return res.json();
}

async function createCarbonCapture(message) {
    const res = await fetch(`${BASE}/carbon-capture`, {
        method: 'POST',
        headers: {
            'Authorization': `Bearer ${TOKEN}`,
            'Content-Type': 'application/json; charset=utf-8',
            'Idempotency-Key': crypto.randomUUID(),
        },
        body: JSON.stringify(message),
    });
    if (res.status !== 201) {
        const err = await res.json().catch(() => ({}));
        throw new Error(`HTTP ${res.status}: ${err?.error?.message ?? res.statusText}`);
    }
    return res.json();
}

// 사용 예
listCarbonCapture({ technology: 'dac', limit: 10 })
    .then(r => console.log(r.data.length, 'records'))
    .catch(console.error);
```

### 12.3 Python 3.10+ (requests)

```python
import os
import uuid
import requests

BASE = "https://api.wia.live/v1"
HEADERS = {
    "Authorization": f"Bearer {os.environ['API_TOKEN']}",
    "Accept": "application/json",
}

def list_carbon_capture(technology=None, limit=50, cursor=None):
    params = {"limit": limit}
    if technology:
        params["technology"] = technology
    if cursor:
        params["cursor"] = cursor
    r = requests.get(f"{BASE}/carbon-capture", headers=HEADERS, params=params, timeout=30)
    r.raise_for_status()
    return r.json()

def create_carbon_capture(message: dict):
    headers = {
        **HEADERS,
        "Content-Type": "application/json; charset=utf-8",
        "Idempotency-Key": str(uuid.uuid4()),
    }
    r = requests.post(f"{BASE}/carbon-capture", headers=headers, json=message, timeout=30)
    r.raise_for_status()
    return r.json()

if __name__ == "__main__":
    result = list_carbon_capture(technology="dac", limit=10)
    print(f"{len(result['data'])} records; has_more={result['pagination']['has_more']}")
```

### 12.4 페이지네이션 반복 (cursor)

```python
def iter_all(technology=None, page_size=500):
    cursor = None
    while True:
        page = list_carbon_capture(technology=technology, limit=page_size, cursor=cursor)
        for item in page["data"]:
            yield item
        if not page["pagination"]["has_more"]:
            return
        cursor = page["pagination"]["next_cursor"]
```

---

## 13. CORS와 브라우저 호환성

### 13.1 CORS 헤더 (서버 구현 요구사항)

브라우저 클라이언트를 허용하는 서버는 다음 헤더를 반드시 제공해야 한다:

```
Access-Control-Allow-Origin: <Origin 반사 또는 *>
Access-Control-Allow-Methods: GET, POST, PUT, PATCH, DELETE, OPTIONS
Access-Control-Allow-Headers: Authorization, Content-Type, Idempotency-Key, If-Match, If-None-Match, X-Request-ID, X-WIA-API-Key
Access-Control-Expose-Headers: ETag, Location, X-Request-ID, X-RateLimit-Limit, X-RateLimit-Remaining, X-RateLimit-Reset, Retry-After, Deprecation, Sunset
Access-Control-Max-Age: 86400
```

자격 증명을 포함한 요청(`credentials: "include"`)을 허용할 경우 `Access-Control-Allow-Origin`은 와일드카드 사용 불가, 반드시 요청한 Origin을 그대로 반사해야 한다.

### 13.2 프리플라이트

비단순 요청(PUT/PATCH/DELETE, 커스텀 헤더 포함)은 `OPTIONS` 프리플라이트를 먼저 받는다. 서버는 `OPTIONS`에 인증을 요구하지 않는다.

---

## 14. 준수 수준

본 스펙을 구현한다고 주장하는 서버는 아래 요구사항을 만족해야 한다. 각 항목은 RFC 2119 키워드로 구분.

### 14.1 MUST (필수)

- 6개 도메인 중 최소 1개 이상의 7개 기본 엔드포인트(표 §7 #1–#7) 구현
- §6의 envelope 형식 준수
- §4.1 중 최소 한 가지 인증 방식 지원
- §10의 속도 제한 헤더 반환
- `GET /v1/health` 및 `GET /v1/openapi.json` 무조건 제공

### 14.2 SHOULD (권장)

- 6개 도메인 모두 구현
- ETag 및 조건부 요청 지원
- Idempotency-Key 지원
- 일괄(`:batch`) 및 통계(`/stats`) 엔드포인트 제공

### 14.3 MAY (선택)

- 전문 검색(`/v1/search`) 지원
- Webhook 콜백 (`/v1/webhooks`) 제공 — 별도 부속 스펙으로 정의 예정
- GraphQL 레이어 — Phase 2.5로 분리

### 14.4 적합성 테스트

`cli/climate-conformance` 도구로 대상 서버를 검증할 수 있다. 테스트 스위트는 다음을 포함한다:

1. 스모크: 모든 MUST 엔드포인트 2xx 응답
2. 계약: OpenAPI 문서와 실제 응답 형식 일치
3. 경계: 잘못된 입력에 올바른 4xx 응답
4. 일관성: Phase 1 스키마 검증 통과
5. 성능: p99 지연 < 500ms (리스트 50건 기준)

---

## 15. 참고문헌

- **RFC 9110** — HTTP Semantics
- **RFC 9111** — HTTP Caching
- **RFC 7519** — JSON Web Token (JWT)
- **RFC 6749** — OAuth 2.0 Authorization Framework
- **RFC 7396** — JSON Merge Patch
- **RFC 7232** — Conditional Requests (ETag, If-Match, If-None-Match)
- **RFC 3339** — Date and Time on the Internet
- **OpenAPI Specification 3.1.0** — OpenAPI Initiative
- **ULID** — https://github.com/ulid/spec
- WIA Climate Phase 1: Data Format (`spec/PHASE-1-DATA-FORMAT.md`)
- WIA Climate Phase 3: Communication Protocol (`spec/PHASE-3-PROTOCOL.md`)
- WIA Climate Phase 4: Integration (`spec/PHASE-4-INTEGRATION.md`)

---

**弘益人間 · Benefit All Humanity**
