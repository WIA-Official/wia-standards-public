# WIA Pet Welfare Global API 인터페이스 표준
## Phase 2 사양서

---

**버전**: 1.0.0
**상태**: Draft
**날짜**: 2025-12-18
**작성자**: WIA Standards Committee
**라이선스**: MIT
**Primary Color**: #F59E0B (Amber)

---

## 목차

1. [개요](#개요)
2. [인증](#인증)
3. [API 엔드포인트](#api-엔드포인트)
4. [요청/응답 형식](#요청응답-형식)
5. [오류 처리](#오류-처리)
6. [속도 제한](#속도-제한)
7. [웹훅](#웹훅)
8. [코드 예제](#코드-예제)
9. [API 참조](#api-참조)

---

## 개요

### 1.1 목적

WIA Pet Welfare Global API 인터페이스 표준은 동물 복지 시스템, 보호소, 구조 조직, 정부 기관 및 국경 간 조정 플랫폼을 위한 RESTful API 엔드포인트, 인증 메커니즘, 데이터 교환 프로토콜 및 통합 패턴을 정의합니다.

**핵심 목표**:
- 동물 복지 데이터에 대한 표준화된 API 액세스 제공
- 실시간 업데이트 및 알림 지원
- 국경 간 데이터 교환 및 입양 워크플로우 지원
- 기존 보호소 관리 시스템과의 통합 촉진
- 안전하고 승인된 데이터 액세스 보장
- 학대 신고 및 추적 시스템 활성화

### 1.2 API 아키텍처

| 컴포넌트 | 기술 | 설명 |
|-----------|------|------|
| Protocol | HTTPS/REST | 보안 HTTP를 통한 RESTful API |
| Format | JSON | 모든 요청 및 응답은 JSON 형식 |
| Authentication | OAuth 2.0 + API Keys | 토큰 기반 인증 |
| Versioning | URI Versioning | URL 경로에 버전 포함 (/v1/) |
| Rate Limiting | Token Bucket | 기본 티어 시간당 1000 요청 |

### 1.3 기본 URL 구조

```
https://api.wia.org/pet-welfare-global/v1/{resource}
```

**환경**:
- Production: `https://api.wia.org/pet-welfare-global/v1/`
- Staging: `https://api-staging.wia.org/pet-welfare-global/v1/`
- Sandbox: `https://api-sandbox.wia.org/pet-welfare-global/v1/`

---

## 인증

### 2.1 인증 방법

| 방법 | 사용 사례 | 보안 수준 |
|------|-----------|----------|
| OAuth 2.0 | 사용자 위임 액세스 | High |
| API Key | 서비스 간 통신 | Medium |
| JWT Token | 세션 관리 | High |
| Client Credentials | 서버 애플리케이션 | High |

### 2.2 OAuth 2.0 플로우

```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id=your_client_id
&client_secret=your_client_secret
&scope=animal:read animal:write shelter:manage
```

**응답**:
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "animal:read animal:write shelter:manage"
}
```

### 2.3 API Key 인증

```http
GET /animals/WIA-PET-2025-000001
Authorization: Bearer YOUR_API_KEY
X-WIA-Organization-ID: WIA-ORG-US-001234
```

### 2.4 권한 스코프

| 스코프 | 액세스 레벨 | 설명 |
|-------|------------|------|
| `animal:read` | Read | 동물 프로필 및 데이터 조회 |
| `animal:write` | Write | 동물 기록 생성 및 업데이트 |
| `animal:delete` | Delete | 동물 기록 삭제 |
| `shelter:read` | Read | 보호소 정보 조회 |
| `shelter:manage` | Admin | 보호소 운영 관리 |
| `adoption:process` | Write | 입양 신청 처리 |
| `transport:manage` | Write | 국제 운송 관리 |
| `abuse:report` | Write | 학대 신고 제출 |
| `abuse:investigate` | Admin | 조사 데이터 액세스 |
| `health:read` | Read | 건강 기록 조회 |
| `health:write` | Write | 의료 기록 업데이트 |
| `admin:full` | Super Admin | 전체 시스템 액세스 |

---

## API 엔드포인트

### 3.1 동물 관리 엔드포인트

#### 3.1.1 동물 프로필 생성

```http
POST /animals
Authorization: Bearer {token}
Content-Type: application/json
```

**요청 본문**:
```json
{
  "identifiers": {
    "microchip_id": "900123456789012",
    "passport_number": "PET-US-2025-001"
  },
  "basic_info": {
    "name": "Max",
    "species": "canis_lupus_familiaris",
    "breed": "Golden Retriever",
    "sex": "male",
    "date_of_birth": "2023-06-15"
  },
  "current_status": {
    "status": "shelter_care",
    "organization_id": "WIA-ORG-US-001234"
  }
}
```

**응답** (201 Created):
```json
{
  "success": true,
  "data": {
    "wia_id": "WIA-PET-2025-000001",
    "created_at": "2025-12-18T10:00:00Z",
    "animal_profile": {
      "wia_id": "WIA-PET-2025-000001",
      "identifiers": {
        "microchip_id": "900123456789012"
      },
      "basic_info": {
        "name": "Max",
        "species": "canis_lupus_familiaris"
      }
    }
  },
  "links": {
    "self": "/animals/WIA-PET-2025-000001",
    "health_passport": "/animals/WIA-PET-2025-000001/health-passport",
    "welfare_assessments": "/animals/WIA-PET-2025-000001/welfare-assessments"
  }
}
```

#### 3.1.2 동물 프로필 조회

```http
GET /animals/{animal_id}
Authorization: Bearer {token}
```

**응답** (200 OK):
```json
{
  "success": true,
  "data": {
    "animal_profile": {
      "wia_id": "WIA-PET-2025-000001",
      "identifiers": {
        "microchip_id": "900123456789012"
      },
      "basic_info": {
        "name": "Max",
        "species": "canis_lupus_familiaris",
        "breed": "Golden Retriever"
      },
      "welfare_scores": {
        "overall_score": 87,
        "last_assessed": "2025-12-18T14:00:00Z"
      }
    }
  }
}
```

#### 3.1.3 동물 프로필 업데이트

```http
PATCH /animals/{animal_id}
Authorization: Bearer {token}
Content-Type: application/json
```

**요청 본문**:
```json
{
  "current_status": {
    "status": "adopted",
    "adoption_date": "2025-12-20T10:00:00Z"
  },
  "welfare_scores": {
    "overall_score": 92
  }
}
```

**응답** (200 OK):
```json
{
  "success": true,
  "data": {
    "wia_id": "WIA-PET-2025-000001",
    "updated_at": "2025-12-20T10:00:00Z"
  }
}
```

#### 3.1.4 동물 검색

```http
GET /animals/search
Authorization: Bearer {token}
```

**쿼리 파라미터**:

| 파라미터 | 타입 | 설명 |
|----------|------|------|
| `species` | string | 종별 필터 |
| `breed` | string | 품종별 필터 |
| `status` | string | 현재 상태별 필터 |
| `organization_id` | string | 조직별 필터 |
| `min_welfare_score` | integer | 최소 복지 점수 |
| `available_for_adoption` | boolean | 입양 가능 여부 |
| `country_code` | string | ISO 국가 코드 |
| `age_min` | integer | 최소 나이 (월) |
| `age_max` | integer | 최대 나이 (월) |
| `page` | integer | 페이지 번호 (기본값: 1) |
| `limit` | integer | 페이지당 결과 (기본값: 20, 최대: 100) |

**예제 요청**:
```http
GET /animals/search?species=canis_lupus_familiaris&status=shelter_care&available_for_adoption=true&page=1&limit=20
```

**응답** (200 OK):
```json
{
  "success": true,
  "data": {
    "animals": [
      {
        "wia_id": "WIA-PET-2025-000001",
        "name": "Max",
        "species": "canis_lupus_familiaris",
        "breed": "Golden Retriever",
        "welfare_score": 87,
        "available_for_adoption": true
      }
    ],
    "pagination": {
      "page": 1,
      "limit": 20,
      "total_results": 156,
      "total_pages": 8
    }
  },
  "links": {
    "self": "/animals/search?page=1&limit=20",
    "next": "/animals/search?page=2&limit=20",
    "last": "/animals/search?page=8&limit=20"
  }
}
```

### 3.2 건강 여권 엔드포인트

#### 3.2.1 건강 여권 생성

```http
POST /animals/{animal_id}/health-passport
Authorization: Bearer {token}
Content-Type: application/json
```

**요청 본문**:
```json
{
  "issued_date": "2025-03-20T10:00:00Z",
  "issuing_authority": {
    "organization": "California Department of Food and Agriculture",
    "country": "US",
    "veterinarian": {
      "name": "Dr. Sarah Johnson",
      "license_number": "CA-VET-12345"
    }
  }
}
```

**응답** (201 Created):
```json
{
  "success": true,
  "data": {
    "passport_id": "PET-US-2025-001",
    "animal_id": "WIA-PET-2025-000001",
    "created_at": "2025-03-20T10:00:00Z"
  }
}
```

#### 3.2.2 예방접종 기록 추가

```http
POST /animals/{animal_id}/health-passport/vaccinations
Authorization: Bearer {token}
Content-Type: application/json
```

**요청 본문**:
```json
{
  "vaccine_name": "Rabies",
  "vaccine_type": "killed_virus",
  "manufacturer": "Merial",
  "batch_number": "RAB-2025-0312",
  "date_administered": "2025-03-20T11:00:00Z",
  "expiry_date": "2028-03-20",
  "administered_by": "Dr. Sarah Johnson",
  "dose_ml": 1.0
}
```

**응답** (201 Created):
```json
{
  "success": true,
  "data": {
    "vaccine_id": "VAC-2025-001",
    "animal_id": "WIA-PET-2025-000001",
    "vaccine_name": "Rabies",
    "date_administered": "2025-03-20T11:00:00Z",
    "next_due_date": "2028-03-20"
  }
}
```

### 3.3 입양 관리 엔드포인트

#### 3.3.1 입양 신청 제출

```http
POST /adoptions/applications
Authorization: Bearer {token}
Content-Type: application/json
```

**요청 본문**:
```json
{
  "animal_id": "WIA-PET-2025-000001",
  "applicant": {
    "name": "Emily Wilson",
    "email": "emily.wilson@email.com",
    "phone": "+44-20-7946-1234",
    "address": {
      "street": "789 Pet Lane",
      "city": "London",
      "country_code": "GB",
      "postal_code": "E1 7AA"
    }
  },
  "household_info": {
    "adults": 2,
    "children": 1,
    "children_ages": [8],
    "other_pets": true,
    "other_pets_details": "One cat, age 5"
  },
  "adoption_type": "international"
}
```

**응답** (201 Created):
```json
{
  "success": true,
  "data": {
    "application_id": "APP-2025-001",
    "animal_id": "WIA-PET-2025-000001",
    "status": "pending_review",
    "submitted_at": "2025-12-18T10:00:00Z"
  }
}
```

#### 3.3.2 입양 신청 조회

```http
GET /adoptions/applications/{application_id}
Authorization: Bearer {token}
```

**응답** (200 OK):
```json
{
  "success": true,
  "data": {
    "application": {
      "application_id": "APP-2025-001",
      "animal_id": "WIA-PET-2025-000001",
      "status": "approved",
      "applicant": {
        "name": "Emily Wilson"
      },
      "submitted_at": "2025-12-18T10:00:00Z",
      "reviewed_at": "2025-12-19T14:00:00Z"
    }
  }
}
```

### 3.4 운송 관리 엔드포인트

#### 3.4.1 운송 매니페스트 생성

```http
POST /transport/manifests
Authorization: Bearer {token}
Content-Type: application/json
```

**요청 본문**:
```json
{
  "animal_id": "WIA-PET-2025-000001",
  "transport_type": "international_adoption",
  "origin": {
    "country_code": "US",
    "organization_id": "WIA-ORG-US-001234",
    "departure_date": "2025-12-20T08:00:00Z"
  },
  "destination": {
    "country_code": "GB",
    "organization_id": "WIA-ORG-GB-005678",
    "expected_arrival": "2025-12-21T14:00:00Z"
  },
  "route": [
    {
      "leg_number": 1,
      "departure_airport": "SFO",
      "arrival_airport": "JFK",
      "flight_number": "AA100"
    }
  ]
}
```

**응답** (201 Created):
```json
{
  "success": true,
  "data": {
    "transport_id": "WIA-TRN-2025-009012",
    "animal_id": "WIA-PET-2025-000001",
    "status": "scheduled",
    "created_at": "2025-12-18T10:00:00Z"
  }
}
```

### 3.5 학대 신고 엔드포인트

#### 3.5.1 학대 신고 제출

```http
POST /abuse/reports
Authorization: Bearer {token}
Content-Type: application/json
```

**요청 본문**:
```json
{
  "incident_date": "2025-11-15T10:00:00Z",
  "incident_type": "neglect",
  "severity_level": "moderate",
  "reporter": {
    "reporter_type": "citizen",
    "anonymous": false,
    "name": "John Smith",
    "contact": {
      "phone": "+1-415-555-9876",
      "email": "john.smith@email.com"
    }
  },
  "location": {
    "address": "456 Oak Street, San Francisco, CA 94102",
    "country_code": "US"
  },
  "incident_details": {
    "description": "Dog found chained outside without shelter, food, or water in freezing temperatures.",
    "visible_injuries": true
  }
}
```

**응답** (201 Created):
```json
{
  "success": true,
  "data": {
    "incident_id": "WIA-INC-2025-005678",
    "status": "pending_review",
    "reported_at": "2025-11-15T14:30:00Z",
    "case_number": "Case assigned upon review"
  }
}
```

### 3.6 복지 평가 엔드포인트

#### 3.6.1 복지 평가 생성

```http
POST /animals/{animal_id}/welfare-assessments
Authorization: Bearer {token}
Content-Type: application/json
```

**요청 본문**:
```json
{
  "assessment_date": "2025-12-18T14:00:00Z",
  "assessed_by": "Dr. Sarah Johnson, DVM",
  "protocol_version": "WIA-WELFARE-ASSESSMENT-v1.0",
  "scores": {
    "physical_health": {
      "score": 90,
      "observations": "Excellent body condition"
    },
    "mental_wellbeing": {
      "score": 85,
      "observations": "Alert and engaged"
    },
    "social_behavior": {
      "score": 88,
      "observations": "Friendly with people and dogs"
    },
    "environmental_enrichment": {
      "score": 82,
      "observations": "Good housing, adequate exercise"
    },
    "care_standards": {
      "score": 90,
      "observations": "Excellent care from staff"
    }
  }
}
```

**응답** (201 Created):
```json
{
  "success": true,
  "data": {
    "assessment_id": "ASSESS-2025-001",
    "animal_id": "WIA-PET-2025-000001",
    "overall_score": 87,
    "assessed_at": "2025-12-18T14:00:00Z",
    "next_assessment_due": "2026-01-18T14:00:00Z"
  }
}
```

### 3.7 조직 관리 엔드포인트

#### 3.7.1 조직 등록

```http
POST /organizations
Authorization: Bearer {token}
Content-Type: application/json
```

**요청 본문**:
```json
{
  "basic_info": {
    "name": "Happy Paws Animal Shelter",
    "organization_type": "nonprofit_shelter",
    "country_code": "US"
  },
  "contact": {
    "primary_address": {
      "street": "123 Animal Way",
      "city": "San Francisco",
      "region": "California",
      "postal_code": "94102",
      "country_code": "US"
    },
    "phone": "+1-415-555-0123",
    "email": "info@happypaws.org"
  },
  "operations": {
    "services_provided": [
      "animal_shelter",
      "adoption_services"
    ]
  }
}
```

**응답** (201 Created):
```json
{
  "success": true,
  "data": {
    "organization_id": "WIA-ORG-US-001234",
    "created_at": "2025-01-01T10:00:00Z",
    "verification_status": "pending"
  }
}
```

### 3.8 마이크로칩 레지스트리 엔드포인트

#### 3.8.1 마이크로칩 등록

```http
POST /microchips/register
Authorization: Bearer {token}
Content-Type: application/json
```

**요청 본문**:
```json
{
  "microchip_id": "900123456789012",
  "animal_id": "WIA-PET-2025-000001",
  "implant_date": "2024-01-10T10:00:00Z",
  "implant_location": "between_shoulder_blades",
  "veterinarian": {
    "name": "Dr. Michael Chen",
    "license_number": "CA-VET-67890"
  },
  "owner": {
    "name": "Emily Wilson",
    "phone": "+44-20-7946-1234",
    "email": "emily.wilson@email.com"
  }
}
```

**응답** (201 Created):
```json
{
  "success": true,
  "data": {
    "microchip_id": "900123456789012",
    "registration_status": "active",
    "registered_at": "2025-12-18T10:00:00Z"
  }
}
```

#### 3.8.2 마이크로칩 조회

```http
GET /microchips/{microchip_id}
Authorization: Bearer {token}
```

**응답** (200 OK):
```json
{
  "success": true,
  "data": {
    "microchip_id": "900123456789012",
    "animal_id": "WIA-PET-2025-000001",
    "registration_status": "active",
    "animal_info": {
      "name": "Max",
      "species": "canis_lupus_familiaris",
      "breed": "Golden Retriever"
    },
    "owner": {
      "name": "Emily Wilson",
      "contact_allowed": true
    }
  }
}
```

### 3.9 통계 및 보고 엔드포인트

#### 3.9.1 조직 통계 조회

```http
GET /organizations/{organization_id}/statistics
Authorization: Bearer {token}
```

**쿼리 파라미터**:

| 파라미터 | 타입 | 설명 |
|----------|------|------|
| `start_date` | timestamp | 통계 시작 날짜 |
| `end_date` | timestamp | 통계 종료 날짜 |
| `period` | string | 집계 기간 (day, week, month, year) |

**응답** (200 OK):
```json
{
  "success": true,
  "data": {
    "period": {
      "start_date": "2025-01-01T00:00:00Z",
      "end_date": "2025-12-18T23:59:59Z"
    },
    "statistics": {
      "total_intakes": 450,
      "total_adoptions": 380,
      "current_population": 127,
      "adoption_rate": 84.4,
      "average_stay_days": 28,
      "welfare_score_average": 86,
      "species_breakdown": {
        "dogs": 68,
        "cats": 52,
        "other": 7
      },
      "intake_sources": {
        "stray": 180,
        "owner_surrender": 120,
        "transfer": 90,
        "seized": 60
      }
    }
  }
}
```

#### 3.9.2 글로벌 복지 통계 조회

```http
GET /statistics/global-welfare
Authorization: Bearer {token}
```

**응답** (200 OK):
```json
{
  "success": true,
  "data": {
    "global_statistics": {
      "total_animals_tracked": 1250000,
      "countries_participating": 45,
      "organizations_registered": 8500,
      "average_welfare_score": 78,
      "cross_border_adoptions": 12500,
      "abuse_reports": {
        "total": 8900,
        "resolved": 7200,
        "under_investigation": 1500,
        "pending": 200
      },
      "top_welfare_countries": [
        {
          "country_code": "NL",
          "average_score": 92,
          "rank": 1
        },
        {
          "country_code": "CH",
          "average_score": 91,
          "rank": 2
        }
      ]
    }
  }
}
```

---

## 요청/응답 형식

### 4.1 표준 요청 헤더

| 헤더 | 필수 | 설명 |
|------|------|------|
| `Authorization` | Yes | Bearer 토큰 또는 API 키 |
| `Content-Type` | Yes (POST/PATCH) | application/json |
| `X-WIA-Organization-ID` | Conditional | 조직 컨텍스트 |
| `X-WIA-Request-ID` | Optional | 멱등성 키 |
| `Accept-Language` | Optional | 선호 언어 (en, ko, es 등) |

### 4.2 표준 응답 형식

**성공 응답**:
```json
{
  "success": true,
  "data": {},
  "meta": {
    "request_id": "req_abc123",
    "timestamp": "2025-12-18T10:00:00Z",
    "version": "1.0.0"
  },
  "links": {
    "self": "/resource/id"
  }
}
```

**오류 응답**:
```json
{
  "success": false,
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Invalid microchip ID format",
    "details": [
      {
        "field": "microchip_id",
        "issue": "Must be 15 digits"
      }
    ]
  },
  "meta": {
    "request_id": "req_abc123",
    "timestamp": "2025-12-18T10:00:00Z"
  }
}
```

---

## 오류 처리

### 5.1 HTTP 상태 코드

| 코드 | 의미 | 사용 |
|------|------|------|
| 200 | OK | 성공적인 GET/PATCH 요청 |
| 201 | Created | 성공적인 POST 요청 |
| 204 | No Content | 성공적인 DELETE 요청 |
| 400 | Bad Request | 잘못된 요청 데이터 |
| 401 | Unauthorized | 인증 누락 또는 잘못됨 |
| 403 | Forbidden | 권한 부족 |
| 404 | Not Found | 리소스가 존재하지 않음 |
| 409 | Conflict | 중복 또는 충돌하는 데이터 |
| 422 | Unprocessable Entity | 유효성 검증 오류 |
| 429 | Too Many Requests | 속도 제한 초과 |
| 500 | Internal Server Error | 서버 오류 |
| 503 | Service Unavailable | 일시적 중단 |

### 5.2 오류 코드

| 코드 | 설명 |
|------|------|
| `AUTHENTICATION_FAILED` | 잘못된 자격 증명 |
| `AUTHORIZATION_FAILED` | 권한 부족 |
| `VALIDATION_ERROR` | 요청 데이터 유효성 검증 실패 |
| `RESOURCE_NOT_FOUND` | 요청한 리소스가 존재하지 않음 |
| `DUPLICATE_RESOURCE` | 리소스가 이미 존재함 |
| `RATE_LIMIT_EXCEEDED` | 너무 많은 요청 |
| `MICROCHIP_ALREADY_REGISTERED` | 마이크로칩이 이미 시스템에 등록됨 |
| `ANIMAL_NOT_AVAILABLE` | 입양 가능한 동물이 아님 |
| `INVALID_HEALTH_CERTIFICATE` | 건강 증명서 유효성 검증 실패 |
| `TRANSPORT_PERMISSION_DENIED` | 목적지 국가로 운송 불가 |

---

## 속도 제한

### 6.1 속도 제한 티어

| 티어 | 시간당 요청 | 일일 요청 | Burst |
|------|-------------|-----------|-------|
| Free | 100 | 1,000 | 10 |
| Basic | 1,000 | 10,000 | 50 |
| Professional | 10,000 | 100,000 | 200 |
| Enterprise | Unlimited | Unlimited | 1,000 |

### 6.2 속도 제한 헤더

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 987
X-RateLimit-Reset: 1703001600
X-RateLimit-Retry-After: 1800
```

---

## 웹훅

### 7.1 웹훅 이벤트

| 이벤트 | 트리거 |
|--------|--------|
| `animal.created` | 새 동물 프로필 생성됨 |
| `animal.updated` | 동물 프로필 업데이트됨 |
| `animal.adopted` | 동물 입양됨 |
| `adoption.application.submitted` | 새 입양 신청 |
| `adoption.application.approved` | 입양 승인됨 |
| `transport.status.updated` | 운송 상태 변경됨 |
| `abuse.report.submitted` | 새 학대 신고 |
| `welfare.assessment.completed` | 복지 평가 완료됨 |
| `health.vaccination.added` | 예방접종 기록됨 |

### 7.2 웹훅 설정

```http
POST /webhooks
Authorization: Bearer {token}
Content-Type: application/json
```

**요청**:
```json
{
  "url": "https://your-server.com/webhooks/wia",
  "events": [
    "animal.adopted",
    "abuse.report.submitted"
  ],
  "secret": "your_webhook_secret"
}
```

**응답**:
```json
{
  "success": true,
  "data": {
    "webhook_id": "wh_abc123",
    "url": "https://your-server.com/webhooks/wia",
    "events": ["animal.adopted", "abuse.report.submitted"],
    "created_at": "2025-12-18T10:00:00Z"
  }
}
```

### 7.3 웹훅 페이로드

```json
{
  "event": "animal.adopted",
  "timestamp": "2025-12-20T10:00:00Z",
  "data": {
    "animal_id": "WIA-PET-2025-000001",
    "adoption_contract_id": "ADOPT-2025-001",
    "adopter": {
      "name": "Emily Wilson",
      "country_code": "GB"
    }
  },
  "signature": "sha256_signature_here"
}
```

---

## 코드 예제

### 예제 1: 동물 프로필 생성 (Python)

```python
import requests
import json

API_BASE = "https://api.wia.org/pet-welfare-global/v1"
API_KEY = "your_api_key_here"

headers = {
    "Authorization": f"Bearer {API_KEY}",
    "Content-Type": "application/json",
    "X-WIA-Organization-ID": "WIA-ORG-US-001234"
}

animal_data = {
    "identifiers": {
        "microchip_id": "900123456789012"
    },
    "basic_info": {
        "name": "Max",
        "species": "canis_lupus_familiaris",
        "breed": "Golden Retriever",
        "sex": "male",
        "date_of_birth": "2023-06-15"
    },
    "current_status": {
        "status": "shelter_care",
        "organization_id": "WIA-ORG-US-001234"
    }
}

response = requests.post(
    f"{API_BASE}/animals",
    headers=headers,
    json=animal_data
)

if response.status_code == 201:
    animal = response.json()
    print(f"동물 생성됨: {animal['data']['wia_id']}")
else:
    print(f"오류: {response.json()['error']['message']}")
```

### 예제 2: 사용 가능한 동물 검색 (JavaScript)

```javascript
const API_BASE = 'https://api.wia.org/pet-welfare-global/v1';
const API_KEY = 'your_api_key_here';

async function searchAnimals(filters) {
  const params = new URLSearchParams(filters);

  const response = await fetch(`${API_BASE}/animals/search?${params}`, {
    method: 'GET',
    headers: {
      'Authorization': `Bearer ${API_KEY}`,
      'Content-Type': 'application/json'
    }
  });

  if (!response.ok) {
    throw new Error(`HTTP error! status: ${response.status}`);
  }

  const data = await response.json();
  return data.data.animals;
}

// 입양 가능한 개 검색
searchAnimals({
  species: 'canis_lupus_familiaris',
  status: 'shelter_care',
  available_for_adoption: true,
  page: 1,
  limit: 20
}).then(animals => {
  console.log(`${animals.length}마리의 입양 가능한 개를 찾았습니다`);
  animals.forEach(animal => {
    console.log(`- ${animal.name} (${animal.breed}): 점수 ${animal.welfare_score}`);
  });
});
```

### 예제 3: 학대 신고 제출 (cURL)

```bash
curl -X POST https://api.wia.org/pet-welfare-global/v1/abuse/reports \
  -H "Authorization: Bearer your_api_key_here" \
  -H "Content-Type: application/json" \
  -d '{
    "incident_date": "2025-11-15T10:00:00Z",
    "incident_type": "neglect",
    "severity_level": "moderate",
    "reporter": {
      "reporter_type": "citizen",
      "anonymous": false,
      "name": "John Smith",
      "contact": {
        "phone": "+1-415-555-9876"
      }
    },
    "location": {
      "address": "456 Oak Street, San Francisco, CA 94102",
      "country_code": "US"
    },
    "incident_details": {
      "description": "Dog found chained outside without shelter",
      "visible_injuries": true
    }
  }'
```

### 예제 4: 웹훅 핸들러 (Node.js/Express)

```javascript
const express = require('express');
const crypto = require('crypto');

const app = express();
app.use(express.json());

const WEBHOOK_SECRET = 'your_webhook_secret';

function verifyWebhookSignature(payload, signature) {
  const hmac = crypto.createHmac('sha256', WEBHOOK_SECRET);
  const digest = hmac.update(JSON.stringify(payload)).digest('hex');
  return crypto.timingSafeEqual(
    Buffer.from(signature),
    Buffer.from(digest)
  );
}

app.post('/webhooks/wia', (req, res) => {
  const signature = req.headers['x-wia-signature'];

  if (!verifyWebhookSignature(req.body, signature)) {
    return res.status(401).send('Invalid signature');
  }

  const { event, data } = req.body;

  switch (event) {
    case 'animal.adopted':
      console.log(`동물 ${data.animal_id}가 입양되었습니다!`);
      // 직원에게 알림 전송
      break;

    case 'abuse.report.submitted':
      console.log(`새 학대 신고: ${data.incident_id}`);
      // 조사팀에 경고
      break;

    case 'transport.status.updated':
      console.log(`운송 ${data.transport_id}: ${data.status}`);
      // 추적 시스템 업데이트
      break;
  }

  res.status(200).send('Webhook received');
});

app.listen(3000, () => {
  console.log('웹훅 서버가 포트 3000에서 실행 중입니다');
});
```

### 예제 5: 복지 평가 제출 (Ruby)

```ruby
require 'net/http'
require 'json'

class WIAWelfareAssessment
  BASE_URL = 'https://api.wia.org/pet-welfare-global/v1'

  def initialize(api_key)
    @api_key = api_key
  end

  def submit_assessment(animal_id, scores)
    uri = URI("#{BASE_URL}/animals/#{animal_id}/welfare-assessments")

    request = Net::HTTP::Post.new(uri)
    request['Authorization'] = "Bearer #{@api_key}"
    request['Content-Type'] = 'application/json'

    request.body = {
      assessment_date: Time.now.iso8601,
      assessed_by: "Dr. Sarah Johnson, DVM",
      protocol_version: "WIA-WELFARE-ASSESSMENT-v1.0",
      scores: scores
    }.to_json

    response = Net::HTTP.start(uri.hostname, uri.port, use_ssl: true) do |http|
      http.request(request)
    end

    JSON.parse(response.body)
  end
end

# 사용 예시
assessment = WIAWelfareAssessment.new('your_api_key')

scores = {
  physical_health: {
    score: 90,
    observations: "Excellent body condition"
  },
  mental_wellbeing: {
    score: 85,
    observations: "Alert and engaged"
  },
  social_behavior: {
    score: 88,
    observations: "Friendly with people and dogs"
  },
  environmental_enrichment: {
    score: 82,
    observations: "Good housing, adequate exercise"
  },
  care_standards: {
    score: 90,
    observations: "Excellent care from staff"
  }
}

result = assessment.submit_assessment('WIA-PET-2025-000001', scores)
puts "평가 제출됨: #{result['data']['assessment_id']}"
puts "전체 점수: #{result['data']['overall_score']}"
```

---

## API 참조

### 9.1 전체 엔드포인트 요약

| 엔드포인트 | 메서드 | 설명 |
|----------|--------|------|
| `/animals` | POST | 동물 프로필 생성 |
| `/animals/{id}` | GET | 동물 프로필 조회 |
| `/animals/{id}` | PATCH | 동물 프로필 업데이트 |
| `/animals/{id}` | DELETE | 동물 프로필 삭제 |
| `/animals/search` | GET | 동물 검색 |
| `/animals/{id}/health-passport` | POST | 건강 여권 생성 |
| `/animals/{id}/health-passport` | GET | 건강 여권 조회 |
| `/animals/{id}/health-passport/vaccinations` | POST | 예방접종 추가 |
| `/animals/{id}/welfare-assessments` | POST | 복지 평가 생성 |
| `/animals/{id}/welfare-assessments` | GET | 평가 이력 조회 |
| `/adoptions/applications` | POST | 입양 신청 제출 |
| `/adoptions/applications/{id}` | GET | 입양 신청 조회 |
| `/adoptions/applications/{id}/approve` | POST | 입양 승인 |
| `/transport/manifests` | POST | 운송 매니페스트 생성 |
| `/transport/manifests/{id}` | GET | 운송 매니페스트 조회 |
| `/transport/manifests/{id}/status` | PATCH | 운송 상태 업데이트 |
| `/abuse/reports` | POST | 학대 신고 제출 |
| `/abuse/reports/{id}` | GET | 학대 신고 조회 |
| `/abuse/reports/{id}/investigation` | PATCH | 조사 업데이트 |
| `/organizations` | POST | 조직 등록 |
| `/organizations/{id}` | GET | 조직 프로필 조회 |
| `/organizations/{id}/capacity` | PATCH | 수용 능력 업데이트 |
| `/organizations/{id}/statistics` | GET | 조직 통계 조회 |
| `/microchips/register` | POST | 마이크로칩 등록 |
| `/microchips/{id}` | GET | 마이크로칩 조회 |
| `/statistics/global-welfare` | GET | 글로벌 통계 조회 |
| `/webhooks` | POST | 웹훅 설정 |

---

**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
