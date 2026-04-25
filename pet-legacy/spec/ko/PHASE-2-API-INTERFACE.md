# WIA PET-LEGACY PHASE 2: API 인터페이스 명세서

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-18
**Primary Color:** #F59E0B (Amber - PET Series)

---

## 목차

1. [개요](#개요)
2. [API 아키텍처](#api-아키텍처)
3. [인증 및 권한 부여](#인증-및-권한-부여)
4. [추모 프로필 엔드포인트](#추모-프로필-엔드포인트)
5. [미디어 관리 엔드포인트](#미디어-관리-엔드포인트)
6. [타임라인 이벤트 엔드포인트](#타임라인-이벤트-엔드포인트)
7. [가족 구성원 엔드포인트](#가족-구성원-엔드포인트)
8. [검색 및 탐색 엔드포인트](#검색-및-탐색-엔드포인트)
9. [AI 및 분석 엔드포인트](#ai-및-분석-엔드포인트)
10. [웹훅 통합](#웹훅-통합)
11. [오류 처리](#오류-처리)
12. [속도 제한 및 할당량](#속도-제한-및-할당량)

---

## 1. 개요

### 1.1 목적

WIA PET-LEGACY Phase 2 명세서는 디지털 반려동물 추모관 관리를 위한 포괄적인 REST API 인터페이스를 정의합니다. 이러한 API는 보안, 프라이버시 및 데이터 무결성을 유지하면서 추모 플랫폼, 동물병원 시스템, 반려동물 묘지 및 타사 애플리케이션 간의 원활한 통합을 가능하게 합니다.

### 1.2 API 설계 원칙

| 원칙 | 설명 | 구현 |
|------|------|------|
| **RESTful** | REST 아키텍처 제약 조건 따르기 | 리소스 기반 URL, HTTP 동사 |
| **버전 관리** | 여러 API 버전 지원 | URL 경로 버전 관리 (/v1/, /v2/) |
| **무상태** | 각 요청에 필요한 모든 정보 포함 | JWT 토큰, 서버 측 세션 없음 |
| **캐시 가능** | 효율적인 캐싱 전략 활성화 | ETag, Cache-Control 헤더 |
| **보안** | 민감한 추모 데이터 보호 | OAuth 2.0, HTTPS만, 암호화 |
| **문서화** | 포괄적인 OpenAPI 문서 | Swagger/OpenAPI 3.0 명세 |

### 1.3 기본 URL 구조

```
프로덕션:  https://api.petlegacy.wia.org/v1
스테이징:  https://api-staging.petlegacy.wia.org/v1
개발:      https://api-dev.petlegacy.wia.org/v1
```

### 1.4 지원되는 형식

| 형식 | Content-Type | 사용 |
|------|-------------|------|
| JSON | application/json | 기본 요청/응답 형식 |
| JSON-LD | application/ld+json | 시맨틱 웹용 링크드 데이터 |
| XML | application/xml | 레거시 시스템 호환성 |
| MessagePack | application/msgpack | 고성능 바이너리 형식 |

---

## 2. API 아키텍처

### 2.1 시스템 아키텍처

```
┌─────────────────┐
│  클라이언트 앱   │
│  웹, 모바일      │
└────────┬────────┘
         │
         ▼
┌─────────────────────────────────────┐
│     API Gateway (Kong/AWS)          │
│  - 속도 제한                         │
│  - 인증                              │
│  - 요청 라우팅                       │
└────────┬────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────┐
│     로드 밸런서                       │
└────────┬────────────────────────────┘
         │
    ┌────┴────┐
    ▼         ▼
┌────────┐ ┌────────┐
│  API   │ │  API   │
│ 서버   │ │ 서버   │
│  (N)   │ │  (N)   │
└───┬────┘ └───┬────┘
    │          │
    └────┬─────┘
         ▼
┌─────────────────────────────────────┐
│         마이크로서비스                │
│  ┌──────────┐  ┌─────────────┐      │
│  │ 추모관   │  │   미디어    │      │
│  │ 서비스   │  │   서비스    │      │
│  └──────────┘  └─────────────┘      │
│  ┌──────────┐  ┌─────────────┐      │
│  │타임라인  │  │   사용자    │      │
│  │ 서비스   │  │   서비스    │      │
│  └──────────┘  └─────────────┘      │
└─────────────────────────────────────┘
```

### 2.2 응답 봉투

모든 API 응답은 일관된 봉투 구조를 따릅니다:

```json
{
  "success": true,
  "data": {
    // 응답 페이로드
  },
  "metadata": {
    "requestId": "req_7x9kp3m2n5",
    "timestamp": "2024-12-18T14:22:00Z",
    "version": "1.0.0",
    "processingTime": 234
  },
  "pagination": {
    "page": 1,
    "perPage": 20,
    "total": 150,
    "totalPages": 8,
    "hasNext": true,
    "hasPrevious": false
  },
  "links": {
    "self": "/v1/memorials?page=1",
    "next": "/v1/memorials?page=2",
    "previous": null,
    "first": "/v1/memorials?page=1",
    "last": "/v1/memorials?page=8"
  }
}
```

---

## 3. 인증 및 권한 부여

### 3.1 OAuth 2.0 흐름

```
┌────────┐                                   ┌─────────────┐
│클라이언트│                                   │ 인증 서버   │
└───┬────┘                                   └──────┬──────┘
    │                                               │
    │  1. 권한 부여 요청                            │
    │──────────────────────────────────────────────>│
    │                                               │
    │  2. 사용자 로그인 및 동의                     │
    │<──────────────────────────────────────────────│
    │                                               │
    │  3. 권한 부여 코드                            │
    │<──────────────────────────────────────────────│
    │                                               │
    │  4. 토큰으로 코드 교환                        │
    │──────────────────────────────────────────────>│
    │                                               │
    │  5. Access Token + Refresh Token             │
    │<──────────────────────────────────────────────│
```

### 3.2 인증 엔드포인트

#### 3.2.1 Access Token 획득

```http
POST /v1/auth/token
Content-Type: application/json

{
  "grant_type": "authorization_code",
  "code": "AUTH_CODE_HERE",
  "client_id": "your_client_id",
  "client_secret": "your_client_secret",
  "redirect_uri": "https://yourapp.com/callback"
}
```

**응답:**
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "refresh_token": "def502004f8c7e8e3b2d1a9c...",
  "scope": "memorial:read memorial:write media:upload"
}
```

### 3.3 권한 범위

| Scope | 설명 | 접근 수준 |
|-------|------|---------|
| `memorial:read` | 추모 프로필 보기 | 읽기 전용 |
| `memorial:write` | 추모관 생성 및 업데이트 | 읽기-쓰기 |
| `memorial:delete` | 추모 프로필 삭제 | 전체 |
| `media:read` | 미디어 자산 보기 | 읽기 전용 |
| `media:upload` | 사진/비디오 업로드 | 쓰기 |
| `media:delete` | 미디어 자산 삭제 | 전체 |
| `timeline:read` | 타임라인 이벤트 보기 | 읽기 전용 |
| `timeline:write` | 이벤트 생성/업데이트 | 읽기-쓰기 |
| `family:read` | 가족 구성원 보기 | 읽기 전용 |
| `family:manage` | 구성원 접근 관리 | 관리자 |
| `admin:all` | 전체 관리자 접근 | 슈퍼 관리자 |

---

## 4. 추모 프로필 엔드포인트

### 4.1 추모 프로필 생성

```http
POST /v1/memorials
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "petIdentity": {
    "name": "맥스",
    "species": "dog",
    "breed": "골든 리트리버",
    "gender": "male",
    "birthDate": "2012-05-20",
    "passingDate": "2024-03-10"
  },
  "guardianInfo": {
    "userId": "660e8400-e29b-41d4-a716-446655440001",
    "name": "사라 존슨",
    "role": "owner"
  },
  "memorialStatus": {
    "status": "draft",
    "isPublic": false
  }
}
```

**응답 (201 Created):**
```json
{
  "success": true,
  "data": {
    "profileId": "550e8400-e29b-41d4-a716-446655440000",
    "petIdentity": {
      "name": "맥스",
      "species": "dog",
      "breed": "골든 리트리버",
      "gender": "male",
      "birthDate": "2012-05-20",
      "passingDate": "2024-03-10"
    },
    "createdAt": "2024-12-18T14:22:00Z",
    "memorialUrl": "https://petlegacy.wia.org/memorial/550e8400"
  }
}
```

### 4.2 추모 프로필 조회

```http
GET /v1/memorials/{memorialId}
Authorization: Bearer {access_token}
```

**쿼리 파라미터:**
| 파라미터 | 타입 | 설명 | 기본값 |
|---------|-----|------|-------|
| `include` | string | 쉼표로 구분된 관계 (timeline,media,family) | null |
| `fields` | string | 반환할 특정 필드 | all |

### 4.3 추모 프로필 업데이트

```http
PATCH /v1/memorials/{memorialId}
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "memorialStatus": {
    "status": "active",
    "isPublic": true
  },
  "memorialCustomization": {
    "epitaph": "영원히 우리 마음속에. 누구든 바랄 수 있는 최고의 친구.",
    "themeColor": "#F59E0B"
  }
}
```

### 4.4 추모 프로필 삭제

```http
DELETE /v1/memorials/{memorialId}
Authorization: Bearer {access_token}
```

**쿼리 파라미터:**
| 파라미터 | 타입 | 설명 | 필수 |
|---------|-----|------|-----|
| `confirm` | boolean | 삭제 확인 | 예 |
| `deleteMedia` | boolean | 모든 미디어 자산도 삭제 | 아니오 |

### 4.5 추모 프로필 목록

```http
GET /v1/memorials
Authorization: Bearer {access_token}
```

**쿼리 파라미터:**
| 파라미터 | 타입 | 설명 | 기본값 |
|---------|-----|------|-------|
| `page` | integer | 페이지 번호 | 1 |
| `perPage` | integer | 페이지당 항목 (최대 100) | 20 |
| `status` | string | 상태별 필터 | all |
| `species` | string | 종별 필터 | all |
| `sort` | string | 정렬 필드 (createdAt, name, passingDate) | createdAt |
| `order` | string | 정렬 순서 (asc, desc) | desc |

### 4.6 추모 데이터 내보내기

```http
POST /v1/memorials/{memorialId}/export
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "format": "json",
  "includeMedia": true,
  "compression": "zip",
  "password": "optional_password"
}
```

**응답 (202 Accepted):**
```json
{
  "success": true,
  "data": {
    "exportId": "exp_9k2m5n7p8q",
    "status": "processing",
    "estimatedSize": "2.3 GB",
    "estimatedTime": "15분",
    "statusUrl": "/v1/exports/exp_9k2m5n7p8q"
  }
}
```

---

## 5. 미디어 관리 엔드포인트

### 5.1 미디어 자산 업로드

```http
POST /v1/memorials/{memorialId}/media
Authorization: Bearer {access_token}
Content-Type: multipart/form-data

--boundary
Content-Disposition: form-data; name="file"; filename="max-photo.jpg"
Content-Type: image/jpeg

[이진 데이터]
--boundary
Content-Disposition: form-data; name="metadata"

{
  "title": "해변의 맥스",
  "description": "처음 바다를 본 날",
  "capturedAt": "2015-07-20T14:30:00Z",
  "tags": ["해변", "여름", "행복"],
  "privacyLevel": "public"
}
--boundary--
```

**응답 (201 Created):**
```json
{
  "success": true,
  "data": {
    "assetId": "770e8400-e29b-41d4-a716-446655440002",
    "type": "photo",
    "url": "https://cdn.petlegacy.wia.org/assets/max-beach.jpg",
    "thumbnailUrl": "https://cdn.petlegacy.wia.org/thumbs/max-beach-thumb.jpg",
    "processingStatus": "processing",
    "uploadedAt": "2024-12-18T14:35:00Z"
  }
}
```

### 5.2 미디어 자산 조회

```http
GET /v1/memorials/{memorialId}/media/{assetId}
Authorization: Bearer {access_token}
```

### 5.3 미디어 자산 목록

```http
GET /v1/memorials/{memorialId}/media
Authorization: Bearer {access_token}
```

**쿼리 파라미터:**
| 파라미터 | 타입 | 설명 | 기본값 |
|---------|-----|------|-------|
| `type` | string | 유형별 필터 (photo, video, audio) | all |
| `page` | integer | 페이지 번호 | 1 |
| `perPage` | integer | 페이지당 항목 | 50 |
| `sort` | string | 정렬 기준 (capturedAt, uploadedAt) | capturedAt |
| `order` | string | 정렬 순서 (asc, desc) | desc |
| `tags` | string | 태그별 필터 (쉼표 구분) | null |

### 5.4 미디어 메타데이터 업데이트

```http
PATCH /v1/memorials/{memorialId}/media/{assetId}
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "title": "맥스의 첫 해변 모험",
  "description": "더 자세한 내용으로 업데이트된 설명",
  "tags": ["해변", "여름", "행복", "캘리포니아"],
  "privacyLevel": "public",
  "isFeatured": true
}
```

### 5.5 미디어 자산 삭제

```http
DELETE /v1/memorials/{memorialId}/media/{assetId}
Authorization: Bearer {access_token}
```

### 5.6 대량 미디어 업로드

```http
POST /v1/memorials/{memorialId}/media/bulk
Authorization: Bearer {access_token}
Content-Type: multipart/form-data

{
  "files": [
    {"file": "photo1.jpg", "capturedAt": "2020-01-15"},
    {"file": "photo2.jpg", "capturedAt": "2020-01-16"},
    {"file": "video1.mp4", "capturedAt": "2020-01-17"}
  ],
  "defaultPrivacy": "family_only",
  "defaultTags": ["휴가", "2020"]
}
```

---

## 6. 타임라인 이벤트 엔드포인트

### 6.1 타임라인 이벤트 생성

```http
POST /v1/memorials/{memorialId}/timeline
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "eventType": "celebration",
  "date": "2020-05-20T15:00:00Z",
  "title": "맥스의 8번째 생일",
  "description": "강아지 전용 케이크와 많은 간식으로 축하했어요!",
  "location": {
    "placeName": "우리 집 뒷마당",
    "city": "포틀랜드",
    "state": "OR"
  },
  "attachedMedia": [
    "770e8400-e29b-41d4-a716-446655440002",
    "770e8400-e29b-41d4-a716-446655440005"
  ],
  "celebrationInfo": {
    "occasionType": "birthday",
    "gifts": ["새 장난감", "간식"]
  }
}
```

### 6.2 타임라인 이벤트 조회

```http
GET /v1/memorials/{memorialId}/timeline/{eventId}
Authorization: Bearer {access_token}
```

### 6.3 타임라인 이벤트 목록

```http
GET /v1/memorials/{memorialId}/timeline
Authorization: Bearer {access_token}
```

**쿼리 파라미터:**
| 파라미터 | 타입 | 설명 | 기본값 |
|---------|-----|------|-------|
| `eventType` | string | 이벤트 유형별 필터 | all |
| `startDate` | date | 이 날짜 이후의 이벤트 | null |
| `endDate` | date | 이 날짜 이전의 이벤트 | null |
| `sort` | string | 날짜별 정렬 | desc |

### 6.4 타임라인 이벤트 업데이트

```http
PATCH /v1/memorials/{memorialId}/timeline/{eventId}
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "description": "더 많은 추억으로 업데이트된 설명",
  "attachedMedia": [
    "770e8400-e29b-41d4-a716-446655440002",
    "770e8400-e29b-41d4-a716-446655440005",
    "770e8400-e29b-41d4-a716-446655440007"
  ]
}
```

### 6.5 타임라인 이벤트 삭제

```http
DELETE /v1/memorials/{memorialId}/timeline/{eventId}
Authorization: Bearer {access_token}
```

---

## 7. 가족 구성원 엔드포인트

### 7.1 가족 구성원 초대

```http
POST /v1/memorials/{memorialId}/family/invite
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "email": "michael.j@example.com",
  "role": "contributor",
  "relationshipToPet": "공동 소유자",
  "personalMessage": "안녕 마이클, 맥스의 추모관을 함께 만들어주길 바래.",
  "permissions": {
    "canView": true,
    "canComment": true,
    "canUpload": true,
    "canEdit": true
  }
}
```

**응답 (201 Created):**
```json
{
  "success": true,
  "data": {
    "invitationId": "inv_2k5m7n9p3q",
    "email": "michael.j@example.com",
    "role": "contributor",
    "status": "pending",
    "invitedAt": "2024-12-18T15:00:00Z",
    "expiresAt": "2025-01-17T15:00:00Z",
    "invitationUrl": "https://petlegacy.wia.org/invite/inv_2k5m7n9p3q"
  }
}
```

### 7.2 가족 구성원 목록

```http
GET /v1/memorials/{memorialId}/family
Authorization: Bearer {access_token}
```

### 7.3 가족 구성원 권한 업데이트

```http
PATCH /v1/memorials/{memorialId}/family/{memberId}
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "role": "contributor",
  "permissions": {
    "canUpload": true,
    "canEdit": false,
    "canDelete": false
  }
}
```

### 7.4 가족 구성원 제거

```http
DELETE /v1/memorials/{memorialId}/family/{memberId}
Authorization: Bearer {access_token}
```

---

## 8. 검색 및 탐색 엔드포인트

### 8.1 추모관 검색

```http
GET /v1/search/memorials
Authorization: Bearer {access_token}
```

**쿼리 파라미터:**
| 파라미터 | 타입 | 설명 |
|---------|-----|------|
| `q` | string | 검색 쿼리 |
| `species` | string | 종별 필터 |
| `breed` | string | 품종별 필터 |
| `location` | string | 지리적 위치 |
| `yearPassed` | integer | 별세 연도 |
| `isPublic` | boolean | 공개 추모관만 |

### 8.2 공개 추모관 탐색

```http
GET /v1/discover/memorials
Authorization: Bearer {access_token}
```

**쿼리 파라미터:**
| 파라미터 | 타입 | 설명 |
|---------|-----|------|
| `category` | string | trending, recent, popular |
| `timeframe` | string | day, week, month, year |

### 8.3 미디어 검색

```http
GET /v1/search/media
Authorization: Bearer {access_token}
```

**쿼리 파라미터:**
| 파라미터 | 타입 | 설명 |
|---------|-----|------|
| `q` | string | 제목/설명에서 검색 |
| `tags` | string | 태그별 필터 |
| `type` | string | photo, video, audio |
| `dateRange` | string | 날짜 범위 필터 |

---

## 9. AI 및 분석 엔드포인트

### 9.1 추억 편집 생성

```http
POST /v1/memorials/{memorialId}/ai/compile
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "compilationType": "video_slideshow",
  "includePhotos": true,
  "includeVideos": false,
  "musicPreference": "gentle",
  "duration": 180,
  "theme": "celebration_of_life"
}
```

**응답 (202 Accepted):**
```json
{
  "success": true,
  "data": {
    "compilationId": "comp_3k7m9n2p5q",
    "status": "processing",
    "estimatedTime": "5분",
    "statusUrl": "/v1/ai/compilations/comp_3k7m9n2p5q"
  }
}
```

### 9.2 추모관 요약 생성

```http
POST /v1/memorials/{memorialId}/ai/summarize
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "summaryType": "life_story",
  "tone": "warm_and_loving",
  "length": "medium",
  "includeMilestones": true,
  "includePersonality": true
}
```

### 9.3 추모관 분석 조회

```http
GET /v1/memorials/{memorialId}/analytics
Authorization: Bearer {access_token}
```

**쿼리 파라미터:**
| 파라미터 | 타입 | 설명 |
|---------|-----|------|
| `metric` | string | views, tributes, shares, engagement |
| `period` | string | day, week, month, year, all |
| `granularity` | string | hourly, daily, weekly, monthly |

### 9.4 AI 사진 향상

```http
POST /v1/memorials/{memorialId}/media/{assetId}/enhance
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "enhancements": ["color_correction", "sharpening", "noise_reduction"],
  "preserveOriginal": true
}
```

### 9.5 AI 태그 제안

```http
POST /v1/memorials/{memorialId}/media/{assetId}/ai/tags
Authorization: Bearer {access_token}
```

**응답 (200 OK):**
```json
{
  "success": true,
  "data": {
    "assetId": "770e8400-e29b-41d4-a716-446655440002",
    "suggestedTags": [
      {"tag": "해변", "confidence": 0.98},
      {"tag": "바다", "confidence": 0.95},
      {"tag": "여름", "confidence": 0.89},
      {"tag": "골든_리트리버", "confidence": 0.99}
    ],
    "detectedObjects": ["개", "물", "모래", "하늘"],
    "sceneType": "outdoor_beach"
  }
}
```

---

## 10. 웹훅 통합

### 10.1 웹훅 구성

```http
POST /v1/webhooks
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "url": "https://yourapp.com/webhooks/petlegacy",
  "events": [
    "memorial.created",
    "memorial.updated",
    "media.uploaded",
    "timeline.event_created",
    "family.member_joined"
  ],
  "secret": "your_webhook_secret"
}
```

### 10.2 웹훅 이벤트

| 이벤트 | 설명 | 페이로드 |
|-------|------|---------|
| `memorial.created` | 새 추모관 생성 | 전체 추모관 객체 |
| `memorial.updated` | 추모관 수정 | 변경된 필드 + 전체 객체 |
| `memorial.deleted` | 추모관 삭제 | 추모관 ID만 |
| `media.uploaded` | 새 미디어 자산 추가 | 미디어 객체 |
| `media.processed` | 미디어 처리 완료 | URL이 포함된 미디어 객체 |
| `timeline.event_created` | 타임라인 이벤트 추가 | 이벤트 객체 |
| `family.member_invited` | 새 구성원 초대 | 초대 객체 |
| `family.member_joined` | 구성원이 초대 수락 | 구성원 객체 |

### 10.3 웹훅 페이로드 예시

```json
{
  "eventId": "evt_9k2m5n7p8q",
  "eventType": "media.uploaded",
  "timestamp": "2024-12-18T16:05:00Z",
  "apiVersion": "1.0.0",
  "data": {
    "memorialId": "550e8400-e29b-41d4-a716-446655440000",
    "assetId": "770e8400-e29b-41d4-a716-446655440002",
    "type": "photo",
    "uploadedBy": "660e8400-e29b-41d4-a716-446655440001",
    "uploadedAt": "2024-12-18T16:05:00Z"
  }
}
```

---

## 11. 오류 처리

### 11.1 오류 응답 형식

```json
{
  "success": false,
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "잘못된 요청 데이터",
    "details": [
      {
        "field": "petIdentity.birthDate",
        "issue": "출생일은 미래 날짜일 수 없습니다",
        "rejectedValue": "2025-12-25"
      }
    ],
    "requestId": "req_7x9kp3m2n5",
    "timestamp": "2024-12-18T16:10:00Z",
    "documentation": "https://docs.petlegacy.wia.org/errors/VALIDATION_ERROR"
  }
}
```

### 11.2 HTTP 상태 코드

| 코드 | 상태 | 설명 |
|------|------|------|
| 200 | OK | 요청 성공 |
| 201 | Created | 리소스 생성 성공 |
| 202 | Accepted | 처리를 위해 요청 수락됨 |
| 204 | No Content | 응답 본문 없이 성공 |
| 400 | Bad Request | 잘못된 요청 데이터 |
| 401 | Unauthorized | 인증 누락 또는 잘못됨 |
| 403 | Forbidden | 권한 부족 |
| 404 | Not Found | 리소스를 찾을 수 없음 |
| 409 | Conflict | 리소스 충돌 (중복) |
| 422 | Unprocessable Entity | 검증 실패 |
| 429 | Too Many Requests | 속도 제한 초과 |
| 500 | Internal Server Error | 서버 오류 |
| 503 | Service Unavailable | 임시 서비스 중단 |

### 11.3 일반적인 오류 코드

| 오류 코드 | HTTP 상태 | 설명 |
|----------|-----------|------|
| `AUTHENTICATION_REQUIRED` | 401 | 인증이 제공되지 않음 |
| `INVALID_TOKEN` | 401 | 토큰이 만료되었거나 잘못됨 |
| `INSUFFICIENT_PERMISSIONS` | 403 | 필요한 권한 누락 |
| `RESOURCE_NOT_FOUND` | 404 | 요청한 리소스가 존재하지 않음 |
| `VALIDATION_ERROR` | 422 | 요청 검증 실패 |
| `RATE_LIMIT_EXCEEDED` | 429 | 요청이 너무 많음 |
| `MEDIA_TOO_LARGE` | 413 | 파일이 크기 제한 초과 |
| `UNSUPPORTED_FORMAT` | 415 | 파일 형식이 지원되지 않음 |
| `QUOTA_EXCEEDED` | 403 | 계정 할당량 제한 도달 |

---

## 12. 속도 제한 및 할당량

### 12.1 속도 제한

| 등급 | 요청/분 | 요청/시간 | 요청/일 |
|------|--------|----------|--------|
| **무료** | 60 | 1,000 | 10,000 |
| **기본** | 300 | 10,000 | 100,000 |
| **프로** | 1,000 | 50,000 | 500,000 |
| **엔터프라이즈** | 맞춤 | 맞춤 | 맞춤 |

### 12.2 속도 제한 헤더

```http
HTTP/1.1 200 OK
X-RateLimit-Limit: 60
X-RateLimit-Remaining: 45
X-RateLimit-Reset: 1703347260
Retry-After: 30
```

### 12.3 저장 공간 할당량

| 등급 | 총 저장 공간 | 최대 파일 크기 | 최대 추모관 수 |
|------|------------|--------------|--------------|
| **무료** | 5 GB | 50 MB | 3 |
| **기본** | 50 GB | 100 MB | 25 |
| **프로** | 500 GB | 500 MB | 무제한 |
| **엔터프라이즈** | 맞춤 | 맞춤 | 무제한 |

---

## 부록 A: 코드 예시

### 예시 1: Node.js SDK 사용

```javascript
const PetLegacy = require('@wia/pet-legacy');

const client = new PetLegacy({
  apiKey: 'your_api_key',
  environment: 'production'
});

// 추모관 생성
async function createMemorial() {
  try {
    const memorial = await client.memorials.create({
      petIdentity: {
        name: '맥스',
        species: 'dog',
        breed: '골든 리트리버',
        birthDate: '2012-05-20',
        passingDate: '2024-03-10'
      }
    });

    console.log('추모관 생성됨:', memorial.profileId);
    return memorial;
  } catch (error) {
    console.error('오류:', error.message);
  }
}

// 사진 업로드
async function uploadPhoto(memorialId, filePath) {
  const photo = await client.media.upload(memorialId, {
    file: fs.createReadStream(filePath),
    title: '해변의 맥스',
    tags: ['해변', '여름']
  });

  return photo;
}
```

### 예시 2: Python SDK 사용

```python
from wia_pet_legacy import PetLegacyClient

client = PetLegacyClient(api_key='your_api_key')

# 추모관 생성
memorial = client.memorials.create({
    'petIdentity': {
        'name': '맥스',
        'species': 'dog',
        'breed': '골든 리트리버',
        'birthDate': '2012-05-20',
        'passingDate': '2024-03-10'
    }
})

print(f"추모관 생성됨: {memorial.profile_id}")

# 미디어 업로드
with open('max-photo.jpg', 'rb') as photo_file:
    media = client.media.upload(
        memorial_id=memorial.profile_id,
        file=photo_file,
        metadata={
            'title': '해변의 맥스',
            'tags': ['해변', '여름']
        }
    )
```

### 예시 3: cURL 예시

```bash
# 추모관 생성
curl -X POST https://api.petlegacy.wia.org/v1/memorials \
  -H "Authorization: Bearer YOUR_TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "petIdentity": {
      "name": "맥스",
      "species": "dog",
      "breed": "골든 리트리버",
      "birthDate": "2012-05-20",
      "passingDate": "2024-03-10"
    }
  }'

# 사진 업로드
curl -X POST https://api.petlegacy.wia.org/v1/memorials/MEMORIAL_ID/media \
  -H "Authorization: Bearer YOUR_TOKEN" \
  -F "file=@max-photo.jpg" \
  -F 'metadata={"title":"해변의 맥스","tags":["해변","여름"]}'
```

---

**弘益人間 (홍익인간)** - 모든 인류에 이로움
© 2025 WIA
MIT License
