# Phase 2: Pet Care Robot API 인터페이스 사양

## WIA-PET-CARE-ROBOT API 표준

**Version**: 1.0.0
**Date**: 2025-12-18
**Status**: Draft
**Standard ID**: WIA-PET-CARE-ROBOT-PHASE2-001
**Primary Color**: #F59E0B (Amber)

---

## 1. 개요

### 1.1 목적

이 사양은 펫 케어 로봇 시스템을 위한 RESTful API 인터페이스를 정의하며, 로봇, 클라우드 서비스, 모바일 애플리케이션 및 스마트 홈 플랫폼 간의 표준화된 통신을 가능하게 합니다. API는 펫 케어 생태계 전반에 걸쳐 실시간 제어, 스케줄링, 모니터링 및 통합을 지원합니다.

**API 설계 원칙**:
- JSON 페이로드를 사용한 RESTful 아키텍처
- OAuth 2.0 인증 및 권한 부여
- 속도 제한 및 할당량 관리
- 실시간 이벤트를 위한 Webhook 지원
- 하위 호환성을 위한 버전 관리 엔드포인트
- 포괄적인 오류 처리
- 검색 가능성을 위한 HATEOAS 지원

### 1.2 기본 URL 구조

```
Production:  https://api.petcare.wia.org/v1
Staging:     https://api-staging.petcare.wia.org/v1
Development: https://api-dev.petcare.wia.org/v1
```

### 1.3 API 엔드포인트 개요

| 카테고리 | 엔드포인트 | 설명 |
|----------|-----------|------|
| **로봇 관리** | 5개 엔드포인트 | 장치 등록, 상태, 구성 |
| **급식 작업** | 6개 엔드포인트 | 일정, 분배, 이력, 영양 |
| **놀이 & 운동** | 5개 엔드포인트 | 세션, 제어, 분석 |
| **건강 모니터링** | 4개 엔드포인트 | 관찰, 경고, 추세 |
| **반려동물 프로필** | 5개 엔드포인트 | 반려동물 관리, 식별 |
| **스케줄링** | 4개 엔드포인트 | 자동화 루틴, 캘린더 |
| **스마트 홈** | 3개 엔드포인트 | 통합, 음성 명령 |
| **미디어** | 3개 엔드포인트 | 카메라, 녹화, 스트리밍 |
| **분석** | 4개 엔드포인트 | 보고서, 인사이트, 예측 |
| **Webhooks** | 3개 엔드포인트 | 이벤트 구독 |

---

## 2. 인증 및 권한 부여

### 2.1 OAuth 2.0 플로우

```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id=YOUR_CLIENT_ID
&client_secret=YOUR_CLIENT_SECRET
&scope=robot:read robot:write pet:read pet:write
```

**응답:**
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "robot:read robot:write pet:read pet:write",
  "refresh_token": "def50200..."
}
```

### 2.2 API 키 인증

```http
GET /api/v1/robots
Authorization: Bearer YOUR_ACCESS_TOKEN
X-API-Key: your-api-key-here
```

### 2.3 권한 범위

| 범위 | 설명 | 작업 |
|------|------|------|
| `robot:read` | 로봇 상태 및 데이터 보기 | GET 작업 |
| `robot:write` | 로봇 작업 제어 | POST, PUT, PATCH |
| `robot:admin` | 전체 로봇 관리 | DELETE를 포함한 모든 작업 |
| `pet:read` | 반려동물 프로필 및 데이터 보기 | GET 반려동물 정보 |
| `pet:write` | 반려동물 프로필 관리 | 반려동물 생성/업데이트 |
| `feeding:execute` | 급식 작업 트리거 | 수동 급식 |
| `play:execute` | 놀이 세션 제어 | 놀이 시작/중지 |
| `health:read` | 건강 데이터 액세스 | 건강 관찰 보기 |
| `analytics:read` | 보고서 및 인사이트 액세스 | 분석 엔드포인트 |
| `webhook:manage` | Webhook 구독 관리 | Webhook 작업 |

---

## 3. 로봇 관리 엔드포인트

### 3.1 로봇 목록 조회

```http
GET /api/v1/robots
```

**쿼리 매개변수:**
| 매개변수 | 유형 | 설명 | 필수 |
|----------|------|------|------|
| `status` | string | 작동 상태로 필터링 | 아니오 |
| `deviceType` | string | 장치 유형으로 필터링 | 아니오 |
| `location` | string | 위치/방으로 필터링 | 아니오 |
| `page` | integer | 페이지 번호 (기본값: 1) | 아니오 |
| `limit` | integer | 페이지당 항목 수 (기본값: 20, 최대: 100) | 아니오 |

**응답 (200 OK):**
```json
{
  "data": [
    {
      "robotId": "PCR-ABC123456789",
      "deviceType": "multi_function",
      "manufacturer": {
        "name": "PetTech Pro",
        "model": "AutoCare 3000",
        "serialNumber": "SN-2025-001234"
      },
      "status": {
        "operational": "online",
        "batteryLevel": 85,
        "foodLevel": 65,
        "waterLevel": 80
      },
      "location": {
        "room": "Living Room",
        "zone": "Pet Area"
      },
      "capabilities": {
        "feeding": true,
        "watering": true,
        "play": true,
        "monitoring": true
      },
      "lastSeen": "2025-12-18T10:30:00Z"
    }
  ],
  "pagination": {
    "page": 1,
    "limit": 20,
    "total": 3,
    "pages": 1
  },
  "links": {
    "self": "/api/v1/robots?page=1",
    "first": "/api/v1/robots?page=1",
    "last": "/api/v1/robots?page=1"
  }
}
```

### 3.2 로봇 세부 정보 조회

이 엔드포인트는 특정 로봇의 상세 정보를 반환합니다. 센서 상태, 기능, 제조업체 정보 및 현재 작동 상태를 포함합니다.

```http
GET /api/v1/robots/{robotId}
```

### 3.3 새 로봇 등록

새 펫 케어 로봇을 시스템에 등록합니다. 초기 설정 및 구성을 위해 사용됩니다.

```http
POST /api/v1/robots
Content-Type: application/json
```

**요청 본문:**
```json
{
  "deviceType": "multi_function",
  "serialNumber": "SN-2025-001234",
  "location": {
    "room": "Living Room",
    "zone": "Pet Area"
  },
  "setupCode": "SETUP-123456"
}
```

### 3.4 로봇 구성 업데이트

로봇 설정 및 구성을 업데이트합니다.

```http
PATCH /api/v1/robots/{robotId}
```

### 3.5 로봇 상태 조회

로봇의 실시간 상태를 조회합니다. 배터리 수준, 공급품 수준 및 경고가 포함됩니다.

```http
GET /api/v1/robots/{robotId}/status
```

---

## 4. 급식 작업 엔드포인트

### 4.1 급식 예약

정기적인 급식 일정을 생성합니다.

```http
POST /api/v1/feeding/schedule
Content-Type: application/json
```

**요청 본문:**
```json
{
  "robotId": "PCR-ABC123456789",
  "petId": "PET-DOG001",
  "schedule": {
    "name": "아침 식사",
    "time": "07:00:00",
    "timezone": "Asia/Seoul",
    "recurring": true,
    "daysOfWeek": ["MON", "TUE", "WED", "THU", "FRI", "SAT", "SUN"]
  },
  "portion": {
    "size": 150,
    "unit": "grams",
    "foodType": "dry_kibble"
  },
  "enabled": true
}
```

### 4.2 수동 급식 분배

즉시 음식을 분배합니다.

```http
POST /api/v1/feeding/dispense
```

### 4.3 급식 이력 조회

과거 급식 이벤트를 조회합니다.

```http
GET /api/v1/feeding/history
```

**쿼리 매개변수:**
- `robotId`: 로봇으로 필터링
- `petId`: 반려동물로 필터링
- `startDate`: 시작 날짜
- `endDate`: 종료 날짜
- `feedingType`: 유형으로 필터링

### 4.4 급식 일정 조회

모든 예약된 급식을 조회합니다.

```http
GET /api/v1/feeding/schedules
```

### 4.5 급식 일정 업데이트

기존 급식 일정을 수정합니다.

```http
PATCH /api/v1/feeding/schedules/{scheduleId}
```

### 4.6 급식 일정 삭제

급식 일정을 제거합니다.

```http
DELETE /api/v1/feeding/schedules/{scheduleId}
```

---

## 5. 놀이 및 운동 엔드포인트

### 5.1 놀이 세션 시작

새 놀이 세션을 시작합니다.

```http
POST /api/v1/play/sessions
```

**요청 본문:**
```json
{
  "robotId": "PCR-ABC123456789",
  "petId": "PET-DOG001",
  "playType": "laser_chase",
  "duration": 900,
  "intensity": "adaptive",
  "settings": {
    "speed": "medium",
    "pattern": "random",
    "restIntervals": true
  }
}
```

### 5.2 놀이 세션 상태 조회

진행 중인 놀이 세션의 상태를 모니터링합니다.

```http
GET /api/v1/play/sessions/{sessionId}
```

### 5.3 놀이 세션 중지

활성 놀이 세션을 종료합니다.

```http
POST /api/v1/play/sessions/{sessionId}/stop
```

### 5.4 놀이 이력 조회

과거 놀이 세션 기록을 조회합니다.

```http
GET /api/v1/play/history
```

### 5.5 놀이 세션 예약

정기적인 놀이 시간을 예약합니다.

```http
POST /api/v1/play/schedule
```

---

## 6. 건강 모니터링 엔드포인트

### 6.1 건강 관찰 기록

새 건강 관찰을 기록합니다.

```http
POST /api/v1/health/observations
```

### 6.2 건강 관찰 조회

저장된 건강 데이터를 조회합니다.

```http
GET /api/v1/health/observations
```

### 6.3 건강 경고 조회

활성 건강 경고를 조회합니다.

```http
GET /api/v1/health/alerts
```

### 6.4 건강 경고 확인

건강 경고를 확인합니다.

```http
POST /api/v1/health/alerts/{alertId}/acknowledge
```

---

## 7. 반려동물 프로필 엔드포인트

### 7.1 반려동물 프로필 생성

새 반려동물을 시스템에 추가합니다.

```http
POST /api/v1/pets
```

**요청 본문:**
```json
{
  "name": "맥스",
  "species": "dog",
  "breed": "골든 리트리버",
  "birthDate": "2020-03-15",
  "gender": "male",
  "neutered": true,
  "identification": {
    "microchipId": "982000123456789",
    "rfidTag": "RFID-MAX-001"
  },
  "dietaryRequirements": {
    "foodType": ["dry_kibble"],
    "portionSize": 400,
    "feedingFrequency": 2,
    "allergies": ["chicken"],
    "restrictions": []
  }
}
```

### 7.2 반려동물 프로필 조회

반려동물의 상세 정보를 조회합니다.

```http
GET /api/v1/pets/{petId}
```

### 7.3 반려동물 목록 조회

모든 등록된 반려동물을 나열합니다.

```http
GET /api/v1/pets
```

### 7.4 반려동물 프로필 업데이트

반려동물 정보를 수정합니다.

```http
PATCH /api/v1/pets/{petId}
```

### 7.5 반려동물 프로필 삭제

반려동물을 시스템에서 제거합니다.

```http
DELETE /api/v1/pets/{petId}
```

---

## 8. 스케줄링 엔드포인트

### 8.1 루틴 생성

자동화된 케어 루틴을 만듭니다.

```http
POST /api/v1/schedules/routines
```

### 8.2 루틴 목록 조회

모든 루틴을 조회합니다.

```http
GET /api/v1/schedules/routines
```

### 8.3 캘린더 보기 조회

예정된 이벤트의 캘린더 보기를 가져옵니다.

```http
GET /api/v1/schedules/calendar
```

### 8.4 루틴 업데이트

기존 루틴을 수정합니다.

```http
PATCH /api/v1/schedules/routines/{routineId}
```

---

## 9. 스마트 홈 통합 엔드포인트

### 9.1 음성 비서 명령 조회

지원되는 음성 명령을 조회합니다.

```http
GET /api/v1/integrations/voice/commands
```

### 9.2 스마트 홈 통합 구성

스마트 홈 플랫폼과 연결합니다.

```http
POST /api/v1/integrations/smarthome
```

### 9.3 통합 상태 조회

활성 통합 상태를 확인합니다.

```http
GET /api/v1/integrations
```

---

## 10. 미디어 엔드포인트

### 10.1 라이브 스트림 조회

카메라 라이브 스트림에 액세스합니다.

```http
GET /api/v1/media/stream/{robotId}
```

### 10.2 녹화 목록 조회

저장된 비디오 녹화를 나열합니다.

```http
GET /api/v1/media/recordings
```

### 10.3 스냅샷 캡처

현재 카메라 이미지를 캡처합니다.

```http
POST /api/v1/media/snapshot
```

---

## 11. 분석 엔드포인트

### 11.1 활동 보고서 조회

반려동물 활동 통계를 조회합니다.

```http
GET /api/v1/analytics/activity
```

**쿼리 매개변수:**
- `petId`: 반려동물 ID (필수)
- `period`: 기간 (day/week/month)
- `startDate`: 시작 날짜
- `endDate`: 종료 날짜

### 11.2 영양 보고서 조회

급식 및 영양 분석을 조회합니다.

```http
GET /api/v1/analytics/nutrition
```

### 11.3 행동 인사이트 조회

행동 패턴 분석을 조회합니다.

```http
GET /api/v1/analytics/behavior
```

### 11.4 예측 분석 조회

AI 기반 예측을 조회합니다.

```http
GET /api/v1/analytics/predictions
```

---

## 12. Webhook 엔드포인트

### 12.1 Webhook 구독 생성

이벤트 알림을 구독합니다.

```http
POST /api/v1/webhooks
```

**요청 본문:**
```json
{
  "url": "https://your-server.com/webhooks/petcare",
  "events": [
    "feeding.completed",
    "play.started",
    "play.completed",
    "health.alert.created",
    "robot.status.changed",
    "supply.low"
  ],
  "secret": "your-webhook-secret",
  "enabled": true
}
```

### 12.2 Webhook 목록 조회

모든 활성 webhook을 조회합니다.

```http
GET /api/v1/webhooks
```

### 12.3 Webhook 삭제

webhook 구독을 제거합니다.

```http
DELETE /api/v1/webhooks/{webhookId}
```

---

## 13. 오류 처리

### 13.1 오류 응답 형식

```json
{
  "error": {
    "code": "ROBOT_NOT_FOUND",
    "message": "ID가 'PCR-INVALID123'인 로봇을 찾을 수 없습니다",
    "statusCode": 404,
    "timestamp": "2025-12-18T14:40:00Z",
    "requestId": "req-abc123",
    "details": {
      "robotId": "PCR-INVALID123",
      "suggestion": "로봇 ID 형식 확인: PCR-[A-Z0-9]{12}"
    }
  }
}
```

### 13.2 오류 코드

| HTTP 상태 | 오류 코드 | 설명 | 해결 방법 |
|-----------|----------|------|----------|
| 400 | `INVALID_REQUEST` | 잘못된 요청 데이터 | 요청 형식 확인 |
| 400 | `VALIDATION_ERROR` | 필드 검증 실패 | 필드 요구 사항 검토 |
| 401 | `UNAUTHORIZED` | 인증 누락 또는 무효 | 유효한 자격 증명 제공 |
| 403 | `FORBIDDEN` | 권한 부족 | 범위 권한 확인 |
| 404 | `ROBOT_NOT_FOUND` | 로봇이 존재하지 않음 | 로봇 ID 확인 |
| 404 | `PET_NOT_FOUND` | 반려동물 프로필을 찾을 수 없음 | 반려동물 ID 확인 |
| 409 | `ROBOT_BUSY` | 로봇이 현재 작업 실행 중 | 대기 또는 현재 작업 취소 |
| 409 | `SCHEDULE_CONFLICT` | 시간대가 이미 예약됨 | 다른 시간 선택 |
| 422 | `SUPPLY_EMPTY` | 음식 또는 물 고갈 | 공급품 보충 |
| 429 | `RATE_LIMIT_EXCEEDED` | 요청이 너무 많음 | 요청 빈도 감소 |
| 500 | `INTERNAL_ERROR` | 서버 오류 | 재시도 또는 지원 문의 |
| 503 | `SERVICE_UNAVAILABLE` | 서비스 일시적으로 다운 | 나중에 재시도 |

---

## 14. 속도 제한

### 14.1 속도 제한 헤더

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 995
X-RateLimit-Reset: 1703001600
```

### 14.2 속도 제한 등급

| 등급 | 요청/시간 | 버스트 | 사용 사례 |
|------|----------|--------|----------|
| **무료** | 100 | 10 | 개인 사용 |
| **기본** | 1,000 | 50 | 소규모 가정 |
| **프로** | 10,000 | 200 | 다중 장치 가정 |
| **엔터프라이즈** | 100,000 | 1,000 | 상업용 애플리케이션 |

---

## 15. Webhook 이벤트 페이로드

### 15.1 급식 완료 이벤트

```json
{
  "event": "feeding.completed",
  "timestamp": "2025-12-18T15:00:00Z",
  "data": {
    "eventId": "FEED-20251218-001",
    "robotId": "PCR-ABC123456789",
    "petId": "PET-DOG001",
    "feedingType": "scheduled",
    "portionSize": 150,
    "dispensed": true,
    "consumed": {
      "detected": true,
      "estimatedAmount": 148
    }
  }
}
```

### 15.2 건강 경고 이벤트

```json
{
  "event": "health.alert.created",
  "timestamp": "2025-12-18T15:05:00Z",
  "data": {
    "alertId": "ALERT-HEALTH-002",
    "petId": "PET-DOG001",
    "alertType": "weight_change",
    "severity": "medium",
    "message": "지난 주 동안 체중이 5% 증가했습니다",
    "recommendation": "급식 분량 및 활동 수준을 검토하세요"
  }
}
```

### 15.3 공급품 부족 이벤트

```json
{
  "event": "supply.low",
  "timestamp": "2025-12-18T15:10:00Z",
  "data": {
    "robotId": "PCR-ABC123456789",
    "supplyType": "food",
    "currentLevel": 18,
    "threshold": 20,
    "estimatedDepletion": "2025-12-19T18:00:00Z",
    "message": "음식 공급이 20% 미만입니다 - 보충 권장"
  }
}
```

---

**弘益人間 (홍익인간)** - 인류와 모든 생명체의 이익을 위하여
© 2025 WIA
MIT License
