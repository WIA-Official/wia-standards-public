# WIA 냉동보존 API 인터페이스 표준
## 2단계 명세

---

**버전**: 1.0.0
**상태**: Draft
**작성일**: 2025-01
**작성자**: WIA 표준 위원회
**라이선스**: MIT
**Primary Color**: #06B6D4 (Cyan)

---

## 목차

1. [개요](#개요)
2. [인증](#인증)
3. [기본 URL 구조](#기본-url-구조)
4. [엔드포인트](#엔드포인트)
5. [요청/응답 형식](#요청응답-형식)
6. [에러 처리](#에러-처리)
7. [속도 제한](#속도-제한)
8. [SDK 예제](#sdk-예제)
9. [웹훅](#웹훅)

---

## 개요

### 1.1 목적

WIA 냉동보존 API 인터페이스 표준은 냉동보존 기록 관리, 저장 조건 모니터링, 시설 간 데이터 교환을 위한 RESTful API를 정의합니다.

**핵심 목표**:
- 보존 데이터에 대한 안전한 프로그래밍 방식 접근 제공
- 저장 조건의 실시간 모니터링 지원
- 시설 간 데이터 전송 촉진
- 포괄적인 로깅을 통한 감사 준수 보장

### 1.2 API 설계 원칙

| 원칙 | 설명 |
|------|------|
| RESTful | 리소스 지향 아키텍처 |
| 버전 관리 | URL 경로에 API 버전 포함 |
| 보안 | OAuth 2.0 + JWT 인증 |
| 일관성 | 균일한 응답 구조 |
| 페이지네이션 | 커서 기반 페이지네이션 |

### 1.3 지원 작업

| 작업 | HTTP 메서드 | 설명 |
|------|-------------|------|
| 생성 | POST | 새 기록 생성 |
| 조회 | GET | 기록 검색 |
| 수정 | PATCH | 부분 업데이트 |
| 삭제 | DELETE | 소프트 삭제 (보관) |
| 목록 | GET | 페이지네이션된 목록 |

---

## 인증

### 2.1 API 키 인증

서버 간 통신용:

```http
GET /api/v1/subjects
Authorization: Bearer <api_key>
X-Facility-ID: FAC-KR-001
```

### 2.2 OAuth 2.0 플로우

사용자 대면 애플리케이션용:

```
┌──────────┐                               ┌──────────┐
│  클라이언트│                               │   인증   │
│    앱    │                               │  서버    │
└────┬─────┘                               └────┬─────┘
     │                                          │
     │  1. 인증 요청                            │
     │  ─────────────────────────────────────► │
     │                                          │
     │  2. 인증 코드                            │
     │  ◄───────────────────────────────────── │
     │                                          │
     │  3. 토큰 요청 (코드 + 시크릿)              │
     │  ─────────────────────────────────────► │
     │                                          │
     │  4. 액세스 토큰 + 리프레시 토큰            │
     │  ◄───────────────────────────────────── │
```

#### 토큰 요청

```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=authorization_code
&code=AUTH_CODE
&client_id=CLIENT_ID
&client_secret=CLIENT_SECRET
&redirect_uri=https://app.example.com/callback
```

#### 토큰 응답

```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIs...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "refresh_token": "dGhpcyBpcyBhIHJlZnJlc2g...",
  "scope": "read write"
}
```

### 2.3 스코프

| 스코프 | 설명 |
|--------|------|
| `read` | 기록에 대한 읽기 전용 접근 |
| `write` | 기록 생성 및 수정 |
| `admin` | 관리 작업 |
| `transfer` | 시설 간 이송 |
| `quality` | 품질 평가 보고서 |

---

## 기본 URL 구조

### 3.1 URL 형식

```
https://api.wia.live/cryo-preservation/v1/{resource}
```

### 3.2 환경별 URL

| 환경 | 기본 URL |
|------|----------|
| 프로덕션 | `https://api.wia.live/cryo-preservation/v1` |
| 스테이징 | `https://staging-api.wia.live/cryo-preservation/v1` |
| 개발 | `https://dev-api.wia.live/cryo-preservation/v1` |

---

## 엔드포인트

### 4.1 대상자 관리

#### 대상자 생성

```http
POST /api/v1/subjects
Content-Type: application/json
Authorization: Bearer <token>

{
  "anonymizedId": "ANON-550e8400",
  "consentId": "CONSENT-2024-1234",
  "demographics": {
    "birthYear": 1950,
    "biologicalSex": "male",
    "bloodType": "A+"
  }
}
```

**응답** `201 Created`
```json
{
  "success": true,
  "data": {
    "id": "SUBJ-2025-001",
    "anonymizedId": "ANON-550e8400",
    "consentId": "CONSENT-2024-1234",
    "demographics": {
      "birthYear": 1950,
      "biologicalSex": "male",
      "bloodType": "A+"
    },
    "createdAt": "2025-01-15T10:30:00Z",
    "updatedAt": "2025-01-15T10:30:00Z"
  }
}
```

#### 대상자 조회

```http
GET /api/v1/subjects/{subjectId}
Authorization: Bearer <token>
```

**응답** `200 OK`
```json
{
  "success": true,
  "data": {
    "id": "SUBJ-2025-001",
    "anonymizedId": "ANON-550e8400",
    "consentId": "CONSENT-2024-1234",
    "preservationStatus": "long_term_storage",
    "facilityId": "FAC-KR-001",
    "createdAt": "2025-01-15T10:30:00Z",
    "updatedAt": "2025-01-15T10:30:00Z"
  }
}
```

#### 대상자 목록

```http
GET /api/v1/subjects?status=long_term_storage&limit=20&cursor=abc123
Authorization: Bearer <token>
```

**응답** `200 OK`
```json
{
  "success": true,
  "data": [
    {
      "id": "SUBJ-2025-001",
      "anonymizedId": "ANON-550e8400",
      "preservationStatus": "long_term_storage"
    }
  ],
  "pagination": {
    "cursor": "def456",
    "hasMore": true,
    "total": 150
  }
}
```

### 4.2 보존 기록

#### 보존 기록 생성

```http
POST /api/v1/subjects/{subjectId}/preservation
Content-Type: application/json
Authorization: Bearer <token>

{
  "preservationType": "whole_body",
  "timeline": {
    "pronouncement": "2025-01-15T08:00:00Z",
    "stabilization_start": "2025-01-15T08:15:00Z"
  },
  "perfusion": {
    "cryoprotectant": "M22",
    "concentration": 0.7,
    "volume_liters": 15.5
  }
}
```

#### 보존 상태 업데이트

```http
PATCH /api/v1/subjects/{subjectId}/preservation/{recordId}
Content-Type: application/json
Authorization: Bearer <token>

{
  "status": "long_term_storage",
  "timeline": {
    "storage_start": "2025-01-16T03:00:00Z"
  },
  "storage": {
    "container_id": "DEW-KR-001",
    "position": "A1-05"
  }
}
```

### 4.3 저장 모니터링

#### 현재 저장 조건 조회

```http
GET /api/v1/storage/containers/{containerId}/conditions
Authorization: Bearer <token>
```

**응답** `200 OK`
```json
{
  "success": true,
  "data": {
    "containerId": "DEW-KR-001",
    "timestamp": "2025-01-20T10:00:00Z",
    "temperature": {
      "current": -196.2,
      "unit": "celsius",
      "status": "normal"
    },
    "liquidNitrogen": {
      "level": 0.85,
      "estimatedDaysRemaining": 45
    },
    "vacuum": {
      "pressure": 1.2e-6,
      "unit": "mbar",
      "status": "normal"
    },
    "alerts": []
  }
}
```

### 4.4 품질 보고서

#### 품질 보고서 제출

```http
POST /api/v1/subjects/{subjectId}/quality-reports
Content-Type: application/json
Authorization: Bearer <token>

{
  "assessmentType": "monthly_review",
  "quality": {
    "vitrification_score": 0.92,
    "tissue_integrity": 0.88,
    "cpa_distribution": 0.95
  },
  "inspector": "STAFF-002",
  "notes": "모든 지표가 허용 범위 내"
}
```

### 4.5 시설 간 이송

#### 이송 시작

```http
POST /api/v1/transfers
Content-Type: application/json
Authorization: Bearer <token>

{
  "subjectId": "SUBJ-2025-001",
  "sourceFacility": "FAC-KR-001",
  "destinationFacility": "FAC-US-001",
  "reason": "family_request",
  "scheduledDate": "2025-02-15",
  "transportMethod": "cryogenic_dewar"
}
```

---

## 요청/응답 형식

### 5.1 표준 응답 구조

#### 성공 응답

```json
{
  "success": true,
  "data": { },
  "meta": {
    "requestId": "req-123456",
    "timestamp": "2025-01-20T10:00:00Z",
    "version": "1.0.0"
  }
}
```

#### 에러 응답

```json
{
  "success": false,
  "error": {
    "code": "ERR_VALIDATION_FAILED",
    "message": "검증 실패",
    "details": [
      {
        "field": "perfusion.concentration",
        "message": "값은 0.0과 1.0 사이여야 합니다"
      }
    ]
  },
  "meta": {
    "requestId": "req-123456",
    "timestamp": "2025-01-20T10:00:00Z"
  }
}
```

### 5.2 공통 헤더

| 헤더 | 설명 | 필수 |
|------|------|------|
| `Authorization` | Bearer 토큰 | 예 |
| `Content-Type` | `application/json` | 예 (POST/PATCH) |
| `X-Request-ID` | 클라이언트 요청 ID | 아니오 |
| `X-Facility-ID` | 시설 식별자 | 조건부 |
| `Accept-Language` | 응답 언어 | 아니오 |

---

## 에러 처리

### 6.1 HTTP 상태 코드

| 코드 | 설명 | 사용 |
|------|------|------|
| 200 | OK | 성공적인 GET/PATCH |
| 201 | Created | 성공적인 POST |
| 204 | No Content | 성공적인 DELETE |
| 400 | Bad Request | 잘못된 입력 |
| 401 | Unauthorized | 인증 실패 |
| 403 | Forbidden | 권한 부족 |
| 404 | Not Found | 리소스 없음 |
| 409 | Conflict | 리소스 충돌 |
| 422 | Unprocessable | 검증 실패 |
| 429 | Too Many Requests | 속도 제한 초과 |
| 500 | Internal Error | 서버 오류 |

### 6.2 에러 코드

| 코드 | 설명 |
|------|------|
| `ERR_AUTHENTICATION_FAILED` | 잘못된 자격 증명 |
| `ERR_AUTHORIZATION_FAILED` | 권한 부족 |
| `ERR_VALIDATION_FAILED` | 입력 검증 오류 |
| `ERR_RESOURCE_NOT_FOUND` | 리소스가 존재하지 않음 |
| `ERR_RESOURCE_CONFLICT` | 중복 또는 충돌 |
| `ERR_RATE_LIMIT_EXCEEDED` | 요청이 너무 많음 |
| `ERR_INTERNAL_ERROR` | 서버 오류 |

---

## 속도 제한

### 7.1 제한

| 등급 | 요청/분 | 요청/시간 |
|------|---------|-----------|
| 기본 | 60 | 1,000 |
| 표준 | 300 | 10,000 |
| 엔터프라이즈 | 1,000 | 100,000 |

### 7.2 속도 제한 헤더

```http
X-RateLimit-Limit: 300
X-RateLimit-Remaining: 299
X-RateLimit-Reset: 1704067200
```

---

## SDK 예제

### 8.1 TypeScript/JavaScript

```typescript
import { WiaCryoClient } from '@wia/cryo-preservation';

const client = new WiaCryoClient({
  apiKey: 'your-api-key',
  facilityId: 'FAC-KR-001',
  environment: 'production'
});

// 대상자 생성
const subject = await client.subjects.create({
  anonymizedId: 'ANON-550e8400',
  consentId: 'CONSENT-2024-1234',
  demographics: {
    birthYear: 1950,
    biologicalSex: 'male',
    bloodType: 'A+'
  }
});

// 보존 기록 조회
const preservation = await client.preservation.get(subject.id);

// 저장 조건 모니터링
const conditions = await client.storage.getConditions('DEW-KR-001');

// 알림 구독
client.alerts.subscribe('DEW-KR-001', (alert) => {
  console.log('알림 수신:', alert);
});
```

### 8.2 Python

```python
from wia_cryo import WiaCryoClient

client = WiaCryoClient(
    api_key='your-api-key',
    facility_id='FAC-KR-001',
    environment='production'
)

# 대상자 생성
subject = client.subjects.create(
    anonymized_id='ANON-550e8400',
    consent_id='CONSENT-2024-1234',
    demographics={
        'birth_year': 1950,
        'biological_sex': 'male',
        'blood_type': 'A+'
    }
)

# 보존 기록 조회
preservation = client.preservation.get(subject.id)

# 저장 조건 모니터링
conditions = client.storage.get_conditions('DEW-KR-001')

# 품질 보고서 제출
report = client.quality.submit_report(
    subject_id=subject.id,
    assessment_type='monthly_review',
    quality={
        'vitrification_score': 0.92,
        'tissue_integrity': 0.88
    }
)
```

### 8.3 cURL 예제

```bash
# 대상자 생성
curl -X POST https://api.wia.live/cryo-preservation/v1/subjects \
  -H "Authorization: Bearer $API_KEY" \
  -H "Content-Type: application/json" \
  -d '{
    "anonymizedId": "ANON-550e8400",
    "consentId": "CONSENT-2024-1234"
  }'

# 저장 조건 조회
curl https://api.wia.live/cryo-preservation/v1/storage/containers/DEW-KR-001/conditions \
  -H "Authorization: Bearer $API_KEY"
```

---

## 웹훅

### 9.1 웹훅 이벤트

| 이벤트 | 설명 |
|--------|------|
| `preservation.status_changed` | 보존 상태 업데이트 |
| `storage.alert` | 저장 조건 알림 |
| `transfer.initiated` | 이송 요청 생성 |
| `transfer.completed` | 이송 완료 |
| `quality.report_submitted` | 품질 보고서 제출 |

### 9.2 웹훅 페이로드

```json
{
  "event": "storage.alert",
  "timestamp": "2025-01-20T10:00:00Z",
  "data": {
    "containerId": "DEW-KR-001",
    "alertType": "temperature_variance",
    "severity": "warning",
    "message": "온도 편차가 임계값을 초과함",
    "currentValue": -195.0,
    "threshold": -195.5
  },
  "signature": "sha256=..."
}
```

---

<div align="center">

**WIA 냉동보존 API 인터페이스 표준 v1.0.0**

**弘益人間 (홍익인간)** - 널리 인간을 이롭게

---

**© 2025 WIA 표준 위원회**

**MIT License**

</div>
