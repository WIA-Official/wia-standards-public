# WIA 냉동인간 소생 API 인터페이스 표준
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
6. [오류 처리](#오류-처리)
7. [속도 제한](#속도-제한)
8. [SDK 예제](#sdk-예제)
9. [웹훅](#웹훅)

---

## 개요

### 1.1 목적

WIA 냉동인간 소생 API 인터페이스 표준은 소생 절차 관리, 실시간 환자 상태 모니터링, 의료진 조정, 의료 정보 시스템과의 통합을 위한 RESTful API를 정의합니다.

**핵심 목표**:
- 소생 절차 데이터에 대한 안전한 프로그래밍 방식 액세스 제공
- 소생 중 환자 생체 징후의 실시간 모니터링 지원
- 전자 건강 기록(EHR) 및 FHIR 시스템과의 통합 지원
- 다학제 의료진 조정 촉진
- 의료 데이터 보호 규정 준수 보장(HIPAA, GDPR)

### 1.2 API 설계 원칙

| 원칙 | 설명 |
|------|------|
| RESTful | 리소스 지향 아키텍처 |
| 버전 관리 | URL 경로의 API 버전 관리 |
| 보안 | HIPAA 준수를 포함한 OAuth 2.0 + JWT 인증 |
| 일관성 | 모든 엔드포인트에 걸친 균일한 응답 구조 |
| 실시간 | 실시간 모니터링을 위한 WebSocket 지원 |

### 1.3 지원 작업

| 작업 | HTTP 메소드 | 설명 |
|------|------------|------|
| 생성 | POST | 소생 절차 시작 |
| 읽기 | GET | 소생 기록 및 모니터링 데이터 검색 |
| 업데이트 | PATCH | 절차 상태 및 생체 징후 업데이트 |
| 스트리밍 | WebSocket | 실시간 생체 징후 스트리밍 |
| 통합 | POST | EHR/FHIR 시스템으로 내보내기 |

---

## 인증

### 2.1 API 키 인증

서버 간 통신의 경우:

```http
GET /api/v1/revivals
Authorization: Bearer <api_key>
X-Facility-ID: FAC-KR-REVIVAL-001
X-Medical-License: MED-KR-2025-001
```

### 2.2 OAuth 2.0 플로우

의료 전문가 애플리케이션의 경우:

```
┌──────────────┐                           ┌──────────────┐
│   의료       │                           │     인증     │
│ 애플리케이션  │                           │    서버      │
└──────┬───────┘                           └──────┬───────┘
       │                                          │
       │  1. 인증 요청 (HIPAA 범위)                 │
       │  ───────────────────────────────────────►│
       │                                          │
       │  2. 인증 코드                             │
       │  ◄───────────────────────────────────────│
       │                                          │
       │  3. 토큰 요청 (코드 + 시크릿)               │
       │  ───────────────────────────────────────►│
       │                                          │
       │  4. 액세스 토큰 + 리프레시 토큰             │
       │  ◄───────────────────────────────────────│
       │                                          │
```

#### 토큰 요청

```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=authorization_code
&code=AUTH_CODE
&client_id=MEDICAL_APP_ID
&client_secret=CLIENT_SECRET
&redirect_uri=https://medical-app.example.com/callback
```

#### 토큰 응답

```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIs...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "refresh_token": "dGhpcyBpcyBhIHJlZnJlc2g...",
  "scope": "revival:read revival:write monitoring:stream"
}
```

### 2.3 범위

| 범위 | 설명 |
|------|------|
| `revival:read` | 소생 절차 기록 읽기 |
| `revival:write` | 소생 절차 생성 및 업데이트 |
| `monitoring:read` | 환자 모니터링 데이터 읽기 |
| `monitoring:stream` | 실시간 생체 징후 스트리밍 |
| `protocol:manage` | 소생 프로토콜 관리 |
| `integration:write` | 의료 시스템으로 내보내기 |
| `admin` | 관리 작업 |

---

## 기본 URL 구조

### 3.1 URL 형식

```
https://api.wia.live/cryo-revival/v1/{resource}
```

### 3.2 환경 URL

| 환경 | 기본 URL |
|------|---------|
| 프로덕션 | `https://api.wia.live/cryo-revival/v1` |
| 스테이징 | `https://staging-api.wia.live/cryo-revival/v1` |
| 개발 | `https://dev-api.wia.live/cryo-revival/v1` |

---

## 엔드포인트

### 4.1 소생 절차 관리

#### 소생 절차 시작

```http
POST /api/v1/revivals
Content-Type: application/json
Authorization: Bearer <token>

{
  "subjectId": "SUBJ-2025-001",
  "preservationRecordId": "PRES-2024-001",
  "revivalType": "full_body",
  "protocol": "standard_v1",
  "medicalTeam": {
    "leadPhysician": "DR-001",
    "neurosurgeon": "DR-002",
    "intensivist": "DR-003",
    "nurses": ["RN-001", "RN-002"]
  },
  "scheduledStart": "2025-01-15T08:00:00Z"
}
```

**응답** `201 Created`
```json
{
  "success": true,
  "data": {
    "revivalId": "REV-2025-001",
    "subjectId": "SUBJ-2025-001",
    "status": "pending",
    "scheduledStart": "2025-01-15T08:00:00Z",
    "facilityId": "FAC-KR-REVIVAL-001",
    "createdAt": "2025-01-14T10:00:00Z"
  }
}
```

#### 소생 절차 조회

```http
GET /api/v1/revivals/{revivalId}
Authorization: Bearer <token>
```

**응답** `200 OK`
```json
{
  "success": true,
  "data": {
    "revivalId": "REV-2025-001",
    "subjectId": "SUBJ-2025-001",
    "revivalType": "full_body",
    "status": "in_progress",
    "timeline": {
      "revival_initiated": "2025-01-15T08:00:00Z",
      "warming_start": "2025-01-15T08:15:00Z",
      "current_phase": "perfusion_reversal"
    },
    "currentVitals": {
      "heart_rate": 72,
      "blood_pressure": { "systolic": 120, "diastolic": 80 },
      "body_temperature": 37.0
    },
    "medicalTeam": {
      "leadPhysician": "DR-001",
      "onDuty": ["DR-001", "DR-002", "RN-001"]
    },
    "updatedAt": "2025-01-15T18:30:00Z"
  }
}
```

#### 소생 상태 업데이트

```http
PATCH /api/v1/revivals/{revivalId}
Content-Type: application/json
Authorization: Bearer <token>

{
  "status": "successful",
  "timeline": {
    "consciousness_restored": "2025-01-16T06:00:00Z"
  },
  "successCriteria": {
    "cardiac_function_restored": true,
    "respiratory_function_restored": true,
    "neurological_activity_present": true,
    "consciousness_level": "alert",
    "overall_success_score": 0.88
  },
  "notes": "환자가 완전히 반응하고 모든 생체 징후가 안정적입니다"
}
```

**응답** `200 OK`
```json
{
  "success": true,
  "data": {
    "revivalId": "REV-2025-001",
    "status": "successful",
    "updatedAt": "2025-01-16T06:00:00Z"
  }
}
```

#### 소생 절차 목록

```http
GET /api/v1/revivals?status=in_progress&facilityId=FAC-KR-REVIVAL-001&limit=20
Authorization: Bearer <token>
```

**응답** `200 OK`
```json
{
  "success": true,
  "data": [
    {
      "revivalId": "REV-2025-001",
      "subjectId": "SUBJ-2025-001",
      "status": "in_progress",
      "currentPhase": "perfusion_reversal"
    },
    {
      "revivalId": "REV-2025-002",
      "subjectId": "SUBJ-2025-002",
      "status": "warming",
      "currentPhase": "warming"
    }
  ],
  "pagination": {
    "cursor": "eyJpZCI6MX0=",
    "hasMore": false,
    "total": 2
  }
}
```

### 4.2 실시간 모니터링

#### 생체 징후 업데이트 제출

```http
POST /api/v1/revivals/{revivalId}/vitals
Content-Type: application/json
Authorization: Bearer <token>

{
  "timestamp": "2025-01-15T19:00:00Z",
  "heart_rate": 72,
  "blood_pressure": {
    "systolic": 120,
    "diastolic": 80
  },
  "respiratory_rate": 16,
  "body_temperature": 37.0,
  "oxygen_saturation": 98,
  "neurologicalStatus": {
    "glasgow_coma_scale": 10,
    "pupil_response": "sluggish"
  }
}
```

**응답** `201 Created`
```json
{
  "success": true,
  "data": {
    "vitalSignsId": "VS-2025-001-12345",
    "revivalId": "REV-2025-001",
    "timestamp": "2025-01-15T19:00:00Z",
    "recordedAt": "2025-01-15T19:00:05Z"
  }
}
```

#### 생체 징후 이력 조회

```http
GET /api/v1/revivals/{revivalId}/vitals?from=2025-01-15T08:00:00Z&to=2025-01-15T20:00:00Z
Authorization: Bearer <token>
```

**응답** `200 OK`
```json
{
  "success": true,
  "data": {
    "revivalId": "REV-2025-001",
    "period": {
      "from": "2025-01-15T08:00:00Z",
      "to": "2025-01-15T20:00:00Z"
    },
    "readings": [
      {
        "timestamp": "2025-01-15T08:00:00Z",
        "heart_rate": 0,
        "body_temperature": -196.0
      },
      {
        "timestamp": "2025-01-15T19:00:00Z",
        "heart_rate": 72,
        "body_temperature": 37.0,
        "glasgow_coma_scale": 10
      }
    ],
    "statistics": {
      "heart_rate_avg": 58,
      "temperature_trend": "increasing",
      "critical_events": 0
    }
  }
}
```

#### 생체 징후 스트리밍 (WebSocket)

```javascript
ws://ws.wia.live/cryo-revival/v1/stream/{revivalId}?token={jwt_token}
```

**메시지 형식:**
```json
{
  "type": "vital_signs_update",
  "revivalId": "REV-2025-001",
  "timestamp": "2025-01-15T19:00:00Z",
  "data": {
    "heart_rate": 72,
    "blood_pressure": { "systolic": 120, "diastolic": 80 },
    "respiratory_rate": 16,
    "body_temperature": 37.0,
    "oxygen_saturation": 98
  }
}
```

### 4.3 프로토콜 관리

#### 사용 가능한 프로토콜 조회

```http
GET /api/v1/protocols?revivalType=full_body
Authorization: Bearer <token>
```

**응답** `200 OK`
```json
{
  "success": true,
  "data": [
    {
      "protocolId": "PROTO-STD-V1",
      "name": "표준 전신 소생 v1.0",
      "revivalType": "full_body",
      "version": "1.0",
      "approvedBy": "WIA 의료 위원회",
      "successRate": 0.78,
      "phases": ["warming", "perfusion_reversal", "cardiac_restoration", "neurological_restoration"]
    },
    {
      "protocolId": "PROTO-RAPID-V1",
      "name": "급속 소생 프로토콜 v1.0",
      "revivalType": "full_body",
      "version": "1.0",
      "approvedBy": "WIA 의료 위원회",
      "successRate": 0.65,
      "phases": ["rapid_warming", "simultaneous_perfusion", "integrated_restoration"]
    }
  ]
}
```

#### 프로토콜 상세 조회

```http
GET /api/v1/protocols/{protocolId}
Authorization: Bearer <token>
```

**응답** `200 OK`
```json
{
  "success": true,
  "data": {
    "protocolId": "PROTO-STD-V1",
    "name": "표준 전신 소생 v1.0",
    "description": "가장 높은 성공률을 가진 증거 기반 전신 소생 프로토콜",
    "revivalType": "full_body",
    "version": "1.0",
    "phases": [
      {
        "name": "warming",
        "duration_hours": 6,
        "temperature_range": [-196, 37],
        "monitoring_frequency_minutes": 5,
        "success_criteria": {
          "temperature_target": 37.0,
          "temperature_variance_max": 0.5
        }
      },
      {
        "name": "perfusion_reversal",
        "duration_hours": 3.5,
        "flow_rate_ml_per_minute": 500,
        "success_criteria": {
          "cpa_removal_percentage": 0.95
        }
      }
    ],
    "contraindications": [
      "심각한 조직 손상 (무결성 < 0.5)",
      "불완전한 보존 기록"
    ],
    "equipment_required": [
      "제어된 가온 챔버",
      "관류 시스템",
      "고급 생명 유지 장치"
    ]
  }
}
```

### 4.4 의료진 조정

#### 팀 배정 조회

```http
GET /api/v1/revivals/{revivalId}/team
Authorization: Bearer <token>
```

**응답** `200 OK`
```json
{
  "success": true,
  "data": {
    "revivalId": "REV-2025-001",
    "team": {
      "leadPhysician": {
        "id": "DR-001",
        "name": "김민준 박사",
        "specialty": "냉동의학",
        "status": "on_duty"
      },
      "neurosurgeon": {
        "id": "DR-002",
        "name": "박지우 박사",
        "specialty": "신경외과",
        "status": "on_duty"
      },
      "intensivist": {
        "id": "DR-003",
        "name": "이서연 박사",
        "specialty": "중환자의학",
        "status": "on_call"
      },
      "nurses": [
        { "id": "RN-001", "name": "최 간호사", "status": "on_duty" },
        { "id": "RN-002", "name": "정 간호사", "status": "on_duty" }
      ]
    }
  }
}
```

### 4.5 의료 시스템 통합

#### FHIR로 내보내기

```http
POST /api/v1/revivals/{revivalId}/export/fhir
Authorization: Bearer <token>

{
  "targetSystem": "hospital-ehr-001",
  "resourceTypes": ["Patient", "Procedure", "Observation"],
  "includeVitals": true
}
```

**응답** `200 OK`
```json
{
  "success": true,
  "data": {
    "exportId": "EXPORT-2025-001",
    "fhirBundle": {
      "resourceType": "Bundle",
      "type": "transaction",
      "entry": [
        {
          "resource": {
            "resourceType": "Patient",
            "id": "SUBJ-2025-001",
            "identifier": [
              {
                "system": "https://wia.live/cryo-revival",
                "value": "SUBJ-2025-001"
              }
            ]
          }
        },
        {
          "resource": {
            "resourceType": "Procedure",
            "id": "REV-2025-001",
            "status": "completed",
            "code": {
              "coding": [
                {
                  "system": "https://wia.live/procedure-codes",
                  "code": "cryo-revival-full-body",
                  "display": "냉동인간 소생 - 전신"
                }
              ]
            }
          }
        }
      ]
    }
  }
}
```

### 4.6 결과 평가

#### 결과 평가 제출

```http
POST /api/v1/revivals/{revivalId}/outcomes
Content-Type: application/json
Authorization: Bearer <token>

{
  "assessmentDate": "2025-01-20T10:00:00Z",
  "assessor": "DR-001",
  "timepoint": "day_5_post_revival",
  "neurologicalAssessment": {
    "glasgow_coma_scale": 15,
    "cognitive_function": "normal",
    "memory": {
      "short_term": "intact",
      "long_term": "partial",
      "episodic": "recovering"
    },
    "motor_function": "normal",
    "sensory_function": "normal"
  },
  "organFunction": {
    "cardiac": 0.95,
    "respiratory": 0.92,
    "renal": 0.88,
    "hepatic": 0.90
  },
  "overallOutcome": "excellent",
  "notes": "환자는 탁월한 회복을 보이고 있습니다. 기억 통합이 진행 중입니다."
}
```

**응답** `201 Created`
```json
{
  "success": true,
  "data": {
    "outcomeId": "OUT-2025-001",
    "revivalId": "REV-2025-001",
    "assessmentDate": "2025-01-20T10:00:00Z",
    "overallOutcome": "excellent"
  }
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

#### 오류 응답

```json
{
  "success": false,
  "error": {
    "code": "ERR_MEDICAL_VALIDATION_FAILED",
    "message": "의료 검증 실패",
    "details": [
      {
        "field": "vitalSigns.heart_rate",
        "message": "심박수는 0에서 300 사이여야 합니다"
      }
    ]
  },
  "meta": {
    "requestId": "req-123456",
    "timestamp": "2025-01-20T10:00:00Z"
  }
}
```

### 5.2 페이지네이션

```json
{
  "success": true,
  "data": [],
  "pagination": {
    "cursor": "eyJpZCI6MTIzfQ==",
    "hasMore": true,
    "limit": 20,
    "total": 150
  }
}
```

### 5.3 공통 헤더

| 헤더 | 설명 | 필수 |
|------|------|------|
| `Authorization` | Bearer 토큰 | 예 |
| `Content-Type` | `application/json` | 예 (POST/PATCH) |
| `X-Request-ID` | 클라이언트 요청 ID | 아니오 |
| `X-Facility-ID` | 시설 식별자 | 조건부 |
| `X-Medical-License` | 의료 면허 번호 | 예 |
| `Accept-Language` | 응답 언어 | 아니오 |

---

## 오류 처리

### 6.1 HTTP 상태 코드

| 코드 | 설명 | 사용 |
|------|------|------|
| 200 | OK | 성공적인 GET/PATCH |
| 201 | Created | 성공적인 POST |
| 204 | No Content | 성공적인 DELETE |
| 400 | Bad Request | 잘못된 입력 |
| 401 | Unauthorized | 인증 실패 |
| 403 | Forbidden | 권한 부족 |
| 404 | Not Found | 리소스를 찾을 수 없음 |
| 409 | Conflict | 리소스 충돌 |
| 422 | Unprocessable | 의료 검증 실패 |
| 429 | Too Many Requests | 속도 제한 초과 |
| 500 | Internal Error | 서버 오류 |
| 503 | Service Unavailable | 유지보수 모드 |

### 6.2 오류 코드

| 코드 | 설명 |
|------|------|
| `ERR_AUTHENTICATION_FAILED` | 잘못된 자격 증명 |
| `ERR_AUTHORIZATION_FAILED` | 권한 부족 |
| `ERR_MEDICAL_VALIDATION_FAILED` | 의료 데이터 검증 오류 |
| `ERR_PROTOCOL_VIOLATION` | 소생 프로토콜 위반 |
| `ERR_RESOURCE_NOT_FOUND` | 리소스가 존재하지 않음 |
| `ERR_VITAL_SIGNS_CRITICAL` | 중요한 생체 징후 감지 |
| `ERR_TEAM_UNAVAILABLE` | 필수 의료진 구성원 없음 |
| `ERR_INTEGRATION_FAILED` | 의료 시스템 통합 오류 |

---

## 속도 제한

### 7.1 제한

| 등급 | 요청/분 | 요청/시간 |
|------|---------|----------|
| 기본 | 60 | 1,000 |
| 의료 전문가 | 300 | 10,000 |
| 병원 시스템 | 1,000 | 100,000 |
| 응급 | 무제한 | 무제한 |

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
import { WiaRevivalClient } from '@wia/cryo-revival';

const client = new WiaRevivalClient({
  apiKey: 'your-api-key',
  facilityId: 'FAC-KR-REVIVAL-001',
  medicalLicense: 'MED-KR-2025-001',
  environment: 'production'
});

// 소생 시작
const revival = await client.revivals.create({
  subjectId: 'SUBJ-2025-001',
  preservationRecordId: 'PRES-2024-001',
  revivalType: 'full_body',
  protocol: 'standard_v1'
});

// 실시간 생체 징후 스트리밍
client.monitoring.streamVitals(revival.revivalId, (vitals) => {
  console.log('심박수:', vitals.heart_rate);
  console.log('체온:', vitals.body_temperature);

  if (vitals.heart_rate > 150) {
    client.alerts.send({
      severity: 'critical',
      message: '빈맥 감지됨'
    });
  }
});

// FHIR로 내보내기
await client.integration.exportToFHIR(revival.revivalId, {
  targetSystem: 'hospital-ehr-001',
  resourceTypes: ['Patient', 'Procedure', 'Observation']
});
```

### 8.2 Python

```python
from wia_revival import WiaRevivalClient

client = WiaRevivalClient(
    api_key='your-api-key',
    facility_id='FAC-KR-REVIVAL-001',
    medical_license='MED-KR-2025-001',
    environment='production'
)

# 소생 시작
revival = client.revivals.create(
    subject_id='SUBJ-2025-001',
    preservation_record_id='PRES-2024-001',
    revival_type='full_body',
    protocol='standard_v1'
)

# 생체 징후 업데이트
client.monitoring.submit_vitals(
    revival_id=revival.revival_id,
    heart_rate=72,
    blood_pressure={'systolic': 120, 'diastolic': 80},
    body_temperature=37.0,
    oxygen_saturation=98
)

# 프로토콜 세부 정보 가져오기
protocol = client.protocols.get('PROTO-STD-V1')
print(f"프로토콜 성공률: {protocol.success_rate}")

# 결과 평가 제출
outcome = client.outcomes.submit(
    revival_id=revival.revival_id,
    assessment_date='2025-01-20T10:00:00Z',
    neurological_assessment={
        'glasgow_coma_scale': 15,
        'cognitive_function': 'normal'
    },
    overall_outcome='excellent'
)
```

---

## 웹훅

### 9.1 웹훅 이벤트

| 이벤트 | 설명 |
|-------|------|
| `revival.initiated` | 소생 절차 시작됨 |
| `revival.status_changed` | 소생 상태 업데이트됨 |
| `revival.completed` | 소생 절차 완료됨 |
| `vitals.critical` | 중요한 생체 징후 감지됨 |
| `consciousness.restored` | 환자 의식 회복됨 |
| `protocol.violation` | 프로토콜 준수 문제 |
| `integration.synced` | 데이터가 EHR에 동기화됨 |

### 9.2 웹훅 페이로드

```json
{
  "event": "vitals.critical",
  "timestamp": "2025-01-15T19:30:00Z",
  "data": {
    "revivalId": "REV-2025-001",
    "alertType": "heart_rate_critical",
    "severity": "critical",
    "message": "심박수가 임계값 이상으로 상승했습니다",
    "currentValue": 165,
    "threshold": 150,
    "vitalSigns": {
      "heart_rate": 165,
      "blood_pressure": { "systolic": 145, "diastolic": 95 },
      "timestamp": "2025-01-15T19:30:00Z"
    }
  },
  "signature": "sha256=..."
}
```

---

<div align="center">

**WIA 냉동인간 소생 API 인터페이스 표준 v1.0.0**

**弘益人間 (홍익인간)** - 모든 인류에게 이익을

---

**© 2025 WIA 표준 위원회**

**MIT 라이선스**

</div>
