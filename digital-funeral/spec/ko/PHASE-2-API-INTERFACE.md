# WIA 디지털 장례 표준 - Phase 2: API Interface 명세서

**버전:** 1.0.0
**상태:** Draft
**날짜:** 2025-12-18
**Primary Color:** #64748B (Slate)
**시리즈:** Digital Death Services

---

## 목차

1. [소개](#소개)
2. [API 아키텍처](#api-아키텍처)
3. [인증 및 권한 부여](#인증-및-권한-부여)
4. [고인 관리 API](#고인-관리-api)
5. [장례 서비스 API](#장례-서비스-api)
6. [손님 및 참석자 API](#손님-및-참석자-api)
7. [추모 및 헌사 API](#추모-및-헌사-api)
8. [금융 거래 API](#금융-거래-api)
9. [묘지 및 안치 API](#묘지-및-안치-api)
10. [스트리밍 및 미디어 API](#스트리밍-및-미디어-api)
11. [알림 API](#알림-api)
12. [오류 처리](#오류-처리)
13. [요청 제한 및 할당량](#요청-제한-및-할당량)
14. [구현 예제](#구현-예제)

---

## 1. 소개

### 1.1 목적

본 명세서는 디지털 장례 서비스를 위한 RESTful API interface를 정의하여, 장례 서비스 제공업체, 추모 플랫폼, 스트리밍 서비스 및 타사 애플리케이션 간의 표준화된 통합을 가능하게 합니다. API는 사전 장례 계획부터 사후 추모관 유지까지 장례 서비스의 전체 수명 주기 관리를 지원합니다.

### 1.2 API 설계 원칙

- **RESTful Architecture**: 표준 HTTP 메서드를 사용한 리소스 지향 URL
- **JSON Format**: 모든 요청 및 응답은 JSON 인코딩 사용
- **Versioning**: URL 경로에 API 버전 포함 (예: `/v1/`)
- **Idempotency**: 중요한 작업의 안전한 재시도
- **Pagination**: 목록 endpoint의 일관된 페이지네이션
- **Security**: 역할 기반 액세스 제어를 통한 OAuth 2.0 인증
- **Privacy**: 개인정보 보호 기본 설정을 존중하는 데이터 액세스 제어

### 1.3 Base URL 구조

```
Production: https://api.funeral-service.example.com/v1
Staging: https://api-staging.funeral-service.example.com/v1
Sandbox: https://api-sandbox.funeral-service.example.com/v1
```

---

## 2. API 아키텍처

### 2.1 HTTP 메서드

| 메서드 | 용도 | Idempotent |
|-------|------|-----------|
| GET | 리소스 검색 | Yes |
| POST | 새 리소스 생성 | No |
| PUT | 전체 리소스 교체 | Yes |
| PATCH | 리소스 부분 업데이트 | No |
| DELETE | 리소스 제거 | Yes |

### 2.2 표준 응답 형식

```json
{
  "success": true,
  "data": {},
  "metadata": {
    "timestamp": "2025-12-18T10:30:00Z",
    "requestId": "req_1234567890",
    "version": "1.0.0"
  },
  "pagination": {
    "page": 1,
    "pageSize": 20,
    "totalPages": 5,
    "totalCount": 98
  }
}
```

### 2.3 오류 응답 형식

```json
{
  "success": false,
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "dateOfDeath의 날짜 형식이 잘못되었습니다",
    "details": [
      {
        "field": "dateOfDeath",
        "issue": "ISO 8601 형식이어야 합니다"
      }
    ],
    "requestId": "req_1234567890",
    "timestamp": "2025-12-18T10:30:00Z"
  }
}
```

### 2.4 공통 Query Parameter

| Parameter | 타입 | 설명 | 예제 |
|-----------|-----|------|-----|
| page | integer | 페이지 번호 (1부터 시작) | `?page=2` |
| pageSize | integer | 페이지당 항목 수 (최대 100) | `?pageSize=50` |
| sort | string | 정렬 필드 및 방향 | `?sort=createdAt:desc` |
| filter | string | 필터 조건 | `?filter=status:published` |
| include | string | 포함할 관련 리소스 | `?include=services,photos` |
| fields | string | 반환할 특정 필드 | `?fields=id,name,dateOfDeath` |

---

## 3. 인증 및 권한 부여

### 3.1 OAuth 2.0 인증

```http
POST /v1/auth/token
Content-Type: application/json

{
  "grant_type": "client_credentials",
  "client_id": "your_client_id",
  "client_secret": "your_client_secret",
  "scope": "funeral.read funeral.write"
}
```

**응답:**
```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "funeral.read funeral.write"
}
```

### 3.2 Authorization Header

```http
GET /v1/deceased/550e8400-e29b-41d4-a716-446655440000
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

### 3.3 API Scope

| Scope | 설명 | 액세스 수준 |
|-------|-----|-----------|
| funeral.read | 장례 서비스 데이터 읽기 | 읽기 전용 |
| funeral.write | 장례 데이터 생성 및 수정 | 읽기/쓰기 |
| funeral.admin | 관리 작업 | 전체 액세스 |
| memorial.read | 추모 및 헌사 데이터 읽기 | 읽기 전용 |
| memorial.write | 헌사 및 조문 게시 | 읽기/쓰기 |
| financial.read | 금융 거래 보기 | 읽기 전용 |
| financial.write | 결제 및 기부 처리 | 읽기/쓰기 |
| streaming.manage | 라이브 스트리밍 제어 | 읽기/쓰기 |

### 3.4 역할 기반 액세스 제어

| 역할 | 권한 | 일반 사용자 |
|-----|------|-----------|
| family_admin | 고인 및 서비스에 대한 전체 액세스 | 주요 가족 연락처 |
| family_member | 읽기 액세스, 제한된 쓰기 | 가족 구성원 |
| funeral_director | 서비스 관리, 재무 보기 | 장례식장 직원 |
| guest | 공개 정보 보기, 조문 게시 | 서비스 참석자 |
| service_provider | 특정 서비스 데이터 액세스 | 화환 배달, 케이터링 |

---

## 4. 고인 관리 API

### 4.1 고인 기록 생성

```http
POST /v1/deceased
Authorization: Bearer {token}
Content-Type: application/json

{
  "legalName": {
    "prefix": "Dr.",
    "firstName": "Margaret",
    "middleName": "Anne",
    "lastName": "Johnson",
    "suffix": "PhD",
    "preferredName": "Maggie"
  },
  "dateOfBirth": "1945-03-15",
  "dateOfDeath": "2025-12-10T08:30:00Z",
  "placeOfBirth": {
    "city": "Boston",
    "state": "Massachusetts",
    "country": "US"
  },
  "privacy": {
    "level": "public",
    "showDateOfBirth": true,
    "showFullBiography": true
  }
}
```

**응답: 201 Created**
```json
{
  "success": true,
  "data": {
    "id": "550e8400-e29b-41d4-a716-446655440000",
    "legalName": {
      "firstName": "Margaret",
      "lastName": "Johnson",
      "preferredName": "Maggie"
    },
    "dateOfBirth": "1945-03-15",
    "dateOfDeath": "2025-12-10T08:30:00Z",
    "metadata": {
      "createdAt": "2025-12-18T10:30:00Z",
      "verificationStatus": "pending"
    }
  }
}
```

### 4.2 고인 세부 정보 조회

```http
GET /v1/deceased/{deceasedId}
Authorization: Bearer {token}
```

**Query Parameters:**
- `include`: 관련 리소스 (예: `services,photos,tributes`)
- `fields`: 반환할 특정 필드

**응답: 200 OK**

### 4.3 고인 정보 업데이트

```http
PATCH /v1/deceased/{deceasedId}
Authorization: Bearer {token}
Content-Type: application/json

{
  "biography": {
    "full": "업데이트된 생애 이야기...",
    "achievements": [
      "포틀랜드 주립대학교 명예교수",
      "미국 시에 관한 세 권의 저서 저자"
    ]
  }
}
```

**응답: 200 OK**

### 4.4 고인 목록 조회

```http
GET /v1/deceased?page=1&pageSize=20&sort=dateOfDeath:desc
Authorization: Bearer {token}
```

### 4.5 고인 사진 업로드

```http
POST /v1/deceased/{deceasedId}/photos
Authorization: Bearer {token}
Content-Type: multipart/form-data

file: [바이너리 이미지 데이터]
caption: "퇴임식에서의 존슨 박사님"
yearTaken: 2010
isPrimary: true
```

**응답: 201 Created**

### 4.6 고인 사진 삭제

```http
DELETE /v1/deceased/{deceasedId}/photos/{photoId}
Authorization: Bearer {token}
```

**응답: 204 No Content**

---

## 5. 장례 서비스 API

### 5.1 장례 서비스 생성

```http
POST /v1/services
Authorization: Bearer {token}
Content-Type: application/json

{
  "deceasedId": "550e8400-e29b-41d4-a716-446655440000",
  "serviceType": "memorial",
  "religiousTradition": "non-religious",
  "startDateTime": "2025-12-18T14:00:00-08:00",
  "endDateTime": "2025-12-18T16:00:00-08:00",
  "timezone": "America/Los_Angeles",
  "venue": {
    "type": "funeral-home",
    "name": "Peaceful Rest Memorial Chapel",
    "address": {
      "street": "1234 Oak Street",
      "city": "Portland",
      "state": "Oregon",
      "postalCode": "97201",
      "country": "US"
    },
    "capacity": 150
  },
  "virtualService": {
    "enabled": true,
    "platform": "custom",
    "recordingEnabled": true,
    "chatEnabled": true
  },
  "rsvpRequired": true,
  "rsvpDeadline": "2025-12-16T23:59:59-08:00"
}
```

**응답: 201 Created**
```json
{
  "success": true,
  "data": {
    "id": "660e8400-e29b-41d4-a716-446655440001",
    "deceasedId": "550e8400-e29b-41d4-a716-446655440000",
    "serviceType": "memorial",
    "startDateTime": "2025-12-18T14:00:00-08:00",
    "status": "planned",
    "virtualService": {
      "enabled": true,
      "streamingUrl": "https://stream.example.com/service/660e8400",
      "accessCode": "MAGGIE2025"
    }
  }
}
```

### 5.2 서비스 세부 정보 조회

```http
GET /v1/services/{serviceId}
Authorization: Bearer {token}
```

### 5.3 서비스 정보 업데이트

```http
PATCH /v1/services/{serviceId}
Authorization: Bearer {token}
Content-Type: application/json

{
  "status": "confirmed",
  "expectedAttendance": 120,
  "program": [
    {
      "order": 1,
      "type": "music",
      "title": "전주곡",
      "duration": 10,
      "performer": "Emily Chen, pianist"
    }
  ]
}
```

### 5.4 고인별 서비스 목록 조회

```http
GET /v1/deceased/{deceasedId}/services
Authorization: Bearer {token}
```

### 5.5 서비스 취소

```http
POST /v1/services/{serviceId}/cancel
Authorization: Bearer {token}
Content-Type: application/json

{
  "reason": "기상 조건 - 일정 변경",
  "notifyAttendees": true
}
```

### 5.6 사전 장례 계획 생성

```http
POST /v1/preneed-plans
Authorization: Bearer {token}
Content-Type: application/json

{
  "planHolderId": "bb0e8400-e29b-41d4-a716-446655440006",
  "servicePreferences": {
    "serviceType": ["memorial", "celebration-of-life"],
    "religiousTradition": "non-religious",
    "musicSelections": [
      "What a Wonderful World - Louis Armstrong"
    ]
  },
  "dispositionPreferences": {
    "method": "cremation",
    "cemetery": "Green Valley Cemetery"
  },
  "contacts": {
    "primaryContact": {
      "name": "김지영",
      "relationship": "daughter",
      "phone": "+82-10-1234-5678",
      "email": "jiyoung.kim@email.com"
    }
  }
}
```

### 5.7 사전 장례 계획 조회

```http
GET /v1/preneed-plans/{planId}
Authorization: Bearer {token}
```

### 5.8 사전 장례 계획 업데이트

```http
PATCH /v1/preneed-plans/{planId}
Authorization: Bearer {token}
Content-Type: application/json

{
  "financialPlan": {
    "amountPaid": 5100000
  },
  "status": "active"
}
```

---

## 6. 손님 및 참석자 API

### 6.1 RSVP 제출

```http
POST /v1/services/{serviceId}/rsvp
Content-Type: application/json

{
  "attendeeInfo": {
    "name": "김민수",
    "email": "minsoo.kim@email.com",
    "phone": "+82-10-1234-5678",
    "relationship": "former student"
  },
  "response": "attending",
  "attendanceType": "in-person",
  "numberOfGuests": 2,
  "guestNames": ["김민수", "이영희"],
  "dietaryRestrictions": ["vegetarian"]
}
```

**응답: 201 Created**
```json
{
  "success": true,
  "data": {
    "id": "770e8400-e29b-41d4-a716-446655440002",
    "serviceId": "660e8400-e29b-41d4-a716-446655440001",
    "response": "attending",
    "confirmationCode": "RSVP-KM-2025",
    "submittedAt": "2025-12-14T10:30:00Z"
  }
}
```

### 6.2 RSVP 업데이트

```http
PATCH /v1/services/{serviceId}/rsvp/{rsvpId}
Content-Type: application/json

{
  "response": "not-attending",
  "numberOfGuests": 0
}
```

### 6.3 RSVP 세부 정보 조회

```http
GET /v1/services/{serviceId}/rsvp/{rsvpId}
```

### 6.4 서비스 RSVP 목록 조회

```http
GET /v1/services/{serviceId}/rsvp?filter=response:attending
Authorization: Bearer {token}
```

**응답: 200 OK**
```json
{
  "success": true,
  "data": [
    {
      "id": "770e8400-e29b-41d4-a716-446655440002",
      "attendeeInfo": {
        "name": "김민수",
        "relationship": "former student"
      },
      "response": "attending",
      "numberOfGuests": 2
    }
  ],
  "metadata": {
    "summary": {
      "attending": 45,
      "notAttending": 12,
      "maybe": 3,
      "virtualOnly": 18
    }
  }
}
```

### 6.5 참석 요약 조회

```http
GET /v1/services/{serviceId}/attendance-summary
Authorization: Bearer {token}
```

**응답: 200 OK**
```json
{
  "success": true,
  "data": {
    "totalResponses": 78,
    "attending": 45,
    "notAttending": 12,
    "maybe": 3,
    "virtualOnly": 18,
    "totalGuests": 89,
    "inPerson": 71,
    "virtual": 18
  }
}
```

---

## 7. 추모 및 헌사 API

### 7.1 조문 메시지 게시

```http
POST /v1/deceased/{deceasedId}/condolences
Content-Type: application/json

{
  "authorName": "박지연",
  "authorEmail": "jiyeon.park@email.com",
  "relationship": "former student",
  "message": "존슨 박사님은 제가 만난 가장 영감을 주는 교수님이셨습니다...",
  "isPublic": true
}
```

**응답: 201 Created**
```json
{
  "success": true,
  "data": {
    "id": "880e8400-e29b-41d4-a716-446655440003",
    "authorName": "박지연",
    "message": "존슨 박사님은 제가 만난 가장 영감을 주는...",
    "moderationStatus": "pending",
    "postedAt": "2025-12-12T16:45:00Z"
  }
}
```

### 7.2 조문 메시지 목록 조회

```http
GET /v1/deceased/{deceasedId}/condolences?page=1&pageSize=20&sort=postedAt:desc
```

**응답: 200 OK**

### 7.3 조문 메시지 승인

```http
PATCH /v1/condolences/{condolenceId}/moderate
Authorization: Bearer {token}
Content-Type: application/json

{
  "moderationStatus": "approved"
}
```

### 7.4 조문 메시지 삭제

```http
DELETE /v1/condolences/{condolenceId}
Authorization: Bearer {token}
```

### 7.5 추모 헌사 생성

```http
POST /v1/deceased/{deceasedId}/tributes
Authorization: Bearer {token}
Content-Type: application/json

{
  "type": "photo-album",
  "title": "세월을 통한 추억",
  "content": {
    "text": "존슨 박사님의 생애를 기념하는 사진 모음",
    "mediaUrls": [
      "https://cdn.example.com/tributes/photo1.jpg",
      "https://cdn.example.com/tributes/photo2.jpg"
    ]
  },
  "author": {
    "name": "존슨 가족",
    "relationship": "family"
  },
  "isPublic": true
}
```

### 7.6 헌사 세부 정보 조회

```http
GET /v1/tributes/{tributeId}
```

### 7.7 헌사 목록 조회

```http
GET /v1/deceased/{deceasedId}/tributes?type=video-montage
```

---

## 8. 금융 거래 API

### 8.1 기부 처리

```http
POST /v1/deceased/{deceasedId}/donations
Content-Type: application/json

{
  "amount": 100000,
  "currency": "KRW",
  "donorInfo": {
    "name": "김철수",
    "email": "chulsoo.kim@email.com",
    "isAnonymous": false
  },
  "recipient": {
    "type": "charity",
    "name": "포틀랜드 공공도서관 재단",
    "taxId": "12-3456789"
  },
  "message": "문맹 퇴치에 헌신한 존슨 박사님을 기리며",
  "paymentMethod": "credit-card",
  "paymentDetails": {
    "token": "tok_visa_1234567890"
  }
}
```

**응답: 201 Created**
```json
{
  "success": true,
  "data": {
    "id": "990e8400-e29b-41d4-a716-446655440004",
    "amount": 100000,
    "currency": "KRW",
    "status": "completed",
    "transactionId": "txn_1234567890",
    "receiptUrl": "https://receipts.example.com/990e8400",
    "donatedAt": "2025-12-13T09:15:00Z"
  }
}
```

### 8.2 기부 세부 정보 조회

```http
GET /v1/donations/{donationId}
Authorization: Bearer {token}
```

### 8.3 기부 목록 조회

```http
GET /v1/deceased/{deceasedId}/donations?page=1&pageSize=20
Authorization: Bearer {token}
```

**응답: 200 OK**
```json
{
  "success": true,
  "data": [
    {
      "id": "990e8400-e29b-41d4-a716-446655440004",
      "amount": 100000,
      "currency": "KRW",
      "donorInfo": {
        "name": "김철수",
        "isAnonymous": false
      },
      "donatedAt": "2025-12-13T09:15:00Z"
    }
  ],
  "metadata": {
    "totalDonations": 47,
    "totalAmount": 4750000,
    "currency": "KRW"
  }
}
```

### 8.4 화환 기록

```http
POST /v1/services/{serviceId}/flowers
Authorization: Bearer {token}
Content-Type: application/json

{
  "type": "standing-spray",
  "description": "흰 장미와 백합",
  "florist": {
    "name": "서울 꽃집",
    "phone": "+82-2-1234-5678",
    "orderNumber": "FLW-12345"
  },
  "sender": {
    "name": "ABC 회사",
    "relationship": "employer"
  },
  "cardMessage": "존슨 박사님을 사랑으로 추모하며",
  "deliveryDate": "2025-12-18",
  "deliveryLocation": "funeral-home"
}
```

### 8.5 화환 목록 조회

```http
GET /v1/services/{serviceId}/flowers
Authorization: Bearer {token}
```

---

## 9. 묘지 및 안치 API

### 9.1 안치 기록 생성

```http
POST /v1/interments
Authorization: Bearer {token}
Content-Type: application/json

{
  "deceasedId": "550e8400-e29b-41d4-a716-446655440000",
  "cemetery": {
    "name": "River View Memorial Park",
    "address": {
      "street": "5678 Cemetery Lane",
      "city": "Portland",
      "state": "Oregon",
      "postalCode": "97202",
      "country": "US"
    }
  },
  "plotLocation": {
    "section": "Garden of Memories",
    "lot": "42",
    "grave": "3"
  },
  "intermentType": "cremains",
  "intermentDate": "2025-12-20T11:00:00-08:00",
  "perpetualCare": true
}
```

### 9.2 안치 세부 정보 조회

```http
GET /v1/interments/{intermentId}
```

### 9.3 비석 정보 업데이트

```http
PATCH /v1/interments/{intermentId}/monument
Authorization: Bearer {token}
Content-Type: application/json

{
  "monument": {
    "type": "marker",
    "material": "granite",
    "inscription": "Dr. Margaret Anne Johnson\n1945 - 2025\n사랑하는 아내, 어머니, 선생님",
    "unveilingDate": "2026-12-10"
  }
}
```

### 9.4 묘지 기록 검색

```http
GET /v1/cemeteries/{cemeteryId}/search?lastName=Johnson&section=Garden
Authorization: Bearer {token}
```

---

## 10. 스트리밍 및 미디어 API

### 10.1 라이브 스트림 시작

```http
POST /v1/services/{serviceId}/stream/start
Authorization: Bearer {token}
Content-Type: application/json

{
  "platform": "custom",
  "quality": "1080p",
  "enableChat": true,
  "enableRecording": true
}
```

**응답: 200 OK**
```json
{
  "success": true,
  "data": {
    "streamId": "stream_660e8400",
    "streamUrl": "https://stream.example.com/service/660e8400",
    "streamKey": "live_sk_1234567890",
    "viewerUrl": "https://watch.example.com/660e8400",
    "status": "live",
    "startedAt": "2025-12-18T14:00:00Z"
  }
}
```

### 10.2 라이브 스트림 중지

```http
POST /v1/services/{serviceId}/stream/stop
Authorization: Bearer {token}
```

**응답: 200 OK**
```json
{
  "success": true,
  "data": {
    "streamId": "stream_660e8400",
    "status": "ended",
    "endedAt": "2025-12-18T16:00:00Z",
    "duration": 7200,
    "viewerCount": 156,
    "recordingUrl": "https://cdn.example.com/recordings/660e8400.mp4"
  }
}
```

### 10.3 스트림 상태 조회

```http
GET /v1/services/{serviceId}/stream/status
Authorization: Bearer {token}
```

**응답: 200 OK**
```json
{
  "success": true,
  "data": {
    "streamId": "stream_660e8400",
    "status": "live",
    "currentViewers": 78,
    "peakViewers": 92,
    "duration": 3600,
    "quality": "1080p"
  }
}
```

### 10.4 스트림 분석 조회

```http
GET /v1/services/{serviceId}/stream/analytics
Authorization: Bearer {token}
```

**응답: 200 OK**
```json
{
  "success": true,
  "data": {
    "totalViews": 156,
    "uniqueViewers": 134,
    "averageWatchTime": 5400,
    "peakViewers": 92,
    "viewersByLocation": {
      "KR": 120,
      "US": 15,
      "CA": 8,
      "other": 13
    },
    "chatMessages": 47
  }
}
```

### 10.5 서비스 녹화본 업로드

```http
POST /v1/services/{serviceId}/recordings
Authorization: Bearer {token}
Content-Type: multipart/form-data

file: [바이너리 비디오 데이터]
title: "마가렛 존슨 박사님 추모식"
description: "추모식 전체 녹화본"
```

### 10.6 녹화본 세부 정보 조회

```http
GET /v1/recordings/{recordingId}
```

---

## 11. 알림 API

### 11.1 서비스 알림 전송

```http
POST /v1/services/{serviceId}/notifications/reminder
Authorization: Bearer {token}
Content-Type: application/json

{
  "recipients": "all-rsvp",
  "channel": ["email", "sms"],
  "message": {
    "subject": "알림: 내일 추모식이 있습니다",
    "body": "마가렛 존슨 박사님 추모식에 대한 알림입니다..."
  },
  "scheduledFor": "2025-12-17T09:00:00Z"
}
```

### 11.2 서비스 업데이트 전송

```http
POST /v1/services/{serviceId}/notifications/update
Authorization: Bearer {token}
Content-Type: application/json

{
  "recipients": "all-rsvp",
  "updateType": "time-change",
  "message": {
    "subject": "서비스 시간 변경",
    "body": "서비스 시간이 오후 3시로 변경되었습니다..."
  }
}
```

### 11.3 알림 구독

```http
POST /v1/deceased/{deceasedId}/notifications/subscribe
Content-Type: application/json

{
  "email": "user@example.com",
  "phone": "+82-10-1234-5678",
  "preferences": {
    "serviceUpdates": true,
    "newTributes": true,
    "anniversaryReminders": true
  }
}
```

### 11.4 알림 구독 취소

```http
DELETE /v1/notifications/unsubscribe/{subscriptionId}
```

---

## 12. 오류 처리

### 12.1 오류 코드

| 코드 | HTTP 상태 | 설명 |
|-----|----------|-----|
| VALIDATION_ERROR | 400 | 요청 검증 실패 |
| AUTHENTICATION_ERROR | 401 | 잘못되거나 누락된 인증 |
| AUTHORIZATION_ERROR | 403 | 권한 부족 |
| NOT_FOUND | 404 | 리소스를 찾을 수 없음 |
| CONFLICT | 409 | 리소스 충돌 (예: 중복) |
| RATE_LIMIT_EXCEEDED | 429 | 요청 횟수 초과 |
| INTERNAL_ERROR | 500 | 서버 오류 |
| SERVICE_UNAVAILABLE | 503 | 일시적인 서비스 중단 |

### 12.2 오류 응답 예제

**검증 오류:**
```json
{
  "success": false,
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "잘못된 요청 데이터",
    "details": [
      {
        "field": "dateOfDeath",
        "issue": "ISO 8601 형식이어야 합니다"
      },
      {
        "field": "privacy.level",
        "issue": "public, family-only, private 중 하나여야 합니다"
      }
    ],
    "requestId": "req_1234567890"
  }
}
```

---

## 13. 요청 제한 및 할당량

### 13.1 요청 제한

| 등급 | 분당 요청 | 시간당 요청 | 일일 요청 |
|-----|---------|-----------|---------|
| Free | 60 | 1,000 | 10,000 |
| Basic | 300 | 10,000 | 100,000 |
| Professional | 1,000 | 50,000 | 500,000 |
| Enterprise | 맞춤형 | 맞춤형 | 맞춤형 |

### 13.2 요청 제한 Header

```http
HTTP/1.1 200 OK
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 847
X-RateLimit-Reset: 1734529200
Retry-After: 3600
```

---

## 14. 구현 예제

### 14.1 전체 워크플로: 장례 서비스 생성

```javascript
// 1단계: 인증
const authResponse = await fetch('https://api.funeral-service.example.com/v1/auth/token', {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({
    grant_type: 'client_credentials',
    client_id: 'your_client_id',
    client_secret: 'your_client_secret',
    scope: 'funeral.write'
  })
});
const { access_token } = await authResponse.json();

// 2단계: 고인 기록 생성
const deceasedResponse = await fetch('https://api.funeral-service.example.com/v1/deceased', {
  method: 'POST',
  headers: {
    'Authorization': `Bearer ${access_token}`,
    'Content-Type': 'application/json'
  },
  body: JSON.stringify({
    legalName: {
      firstName: 'Margaret',
      lastName: 'Johnson'
    },
    dateOfBirth: '1945-03-15',
    dateOfDeath: '2025-12-10T08:30:00Z',
    privacy: { level: 'public' }
  })
});
const { data: deceased } = await deceasedResponse.json();

// 3단계: 장례 서비스 생성
const serviceResponse = await fetch('https://api.funeral-service.example.com/v1/services', {
  method: 'POST',
  headers: {
    'Authorization': `Bearer ${access_token}`,
    'Content-Type': 'application/json'
  },
  body: JSON.stringify({
    deceasedId: deceased.id,
    serviceType: 'memorial',
    startDateTime: '2025-12-18T14:00:00-08:00',
    virtualService: { enabled: true }
  })
});
const { data: service } = await serviceResponse.json();

console.log('서비스 생성됨:', service.id);
console.log('스트리밍 URL:', service.virtualService.streamingUrl);
```

### 14.2 공개 손님 RSVP 제출

```javascript
// 공개 RSVP에는 인증이 필요하지 않음
const rsvpResponse = await fetch(
  `https://api.funeral-service.example.com/v1/services/${serviceId}/rsvp`,
  {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      attendeeInfo: {
        name: '김민수',
        email: 'minsoo.kim@email.com',
        relationship: 'former student'
      },
      response: 'attending',
      numberOfGuests: 2
    })
  }
);

const { data: rsvp } = await rsvpResponse.json();
console.log('RSVP 확인됨:', rsvp.confirmationCode);
```

### 14.3 추모 기부 처리

```javascript
const donationResponse = await fetch(
  `https://api.funeral-service.example.com/v1/deceased/${deceasedId}/donations`,
  {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      amount: 100000,
      currency: 'KRW',
      donorInfo: {
        name: '김철수',
        email: 'chulsoo.kim@email.com'
      },
      recipient: {
        type: 'charity',
        name: '포틀랜드 공공도서관 재단'
      },
      message: '존슨 박사님을 기리며',
      paymentMethod: 'credit-card',
      paymentDetails: {
        token: 'tok_visa_1234567890'
      }
    })
  }
);

const { data: donation } = await donationResponse.json();
console.log('기부 처리됨:', donation.transactionId);
console.log('영수증 URL:', donation.receiptUrl);
```

---

## 결론

이 Phase 2 명세서는 디지털 장례 서비스를 위한 포괄적인 RESTful API를 정의합니다. API는 고인 관리, 장례 서비스, 손님 참석, 추모관, 금융 거래 및 라이브 스트리밍을 위한 완전한 기능을 제공합니다. 모든 endpoint는 인증, 오류 처리 및 데이터 형식에 대한 일관된 패턴을 따릅니다.

**다음 단계:** Phase 3은 실시간 통신, webhook 및 이벤트 스트리밍을 위한 protocol 명세를 정의합니다.

---

**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
