# 5장: API 인터페이스 사양

## 개요

이 장은 WIA-IND-012 규격 시스템을 위한 RESTful API 엔드포인트 및 WebSocket 인터페이스를 정의합니다.

---

## 5.1 API 설계 원칙

### 5.1.1 REST 아키텍처

**핵심 원칙:**
- **리소스 기반 URL**: `/api/v1/activities/{id}`
- **HTTP 메서드**: GET, POST, PUT, DELETE, PATCH
- **무상태**: 각 요청에 필요한 모든 정보 포함
- **JSON 형식**: 요청 및 응답 본문
- **버전 관리**: 안정성을 위한 `/api/v1/`

### 5.1.2 HTTP 상태 코드

```
성공 코드:
200 OK              - 성공적인 GET, PUT, PATCH
201 Created         - 성공적인 POST
204 No Content      - 성공적인 DELETE

클라이언트 오류 코드:
400 Bad Request     - 잘못된 요청 형식
401 Unauthorized    - 인증 누락 또는 무효
403 Forbidden       - 인증되었지만 권한 없음
404 Not Found       - 리소스가 존재하지 않음
429 Too Many Requests - 속도 제한 초과

서버 오류 코드:
500 Internal Server Error - 예상치 못한 서버 오류
503 Service Unavailable - 유지보수 또는 과부하
```

### 5.1.3 인증

**OAuth 2.0 흐름:**
```http
POST /oauth/token HTTP/1.1
Host: api.example.com
Content-Type: application/x-www-form-urlencoded

grant_type=authorization_code&
code=AUTHORIZATION_CODE&
client_id=CLIENT_ID&
client_secret=CLIENT_SECRET
```

**응답:**
```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "refresh_token": "tGzv3JOkF0XG5Qx2TlKWIA"
}
```

**액세스 토큰 사용:**
```http
GET /api/v1/activities HTTP/1.1
Host: api.example.com
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

---

## 5.2 활동 엔드포인트

### 5.2.1 활동 생성

**요청:**
```http
POST /api/v1/activities HTTP/1.1
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "type": "running",
  "startTime": "2025-12-27T07:00:00Z",
  "endTime": "2025-12-27T07:32:15Z",
  "distance": 8000,
  "calories": 680
}
```

**응답 (201 Created):**
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "userId": "user_12345",
  "type": "running",
  "startTime": "2025-12-27T07:00:00Z",
  "duration": 1935,
  "distance": 8000,
  "calories": 680
}
```

### 5.2.2 활동 조회

**요청:**
```http
GET /api/v1/activities/550e8400-e29b-41d4-a716-446655440000 HTTP/1.1
Authorization: Bearer {access_token}
```

**응답 (200 OK):**
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "type": "running",
  "distance": 8000,
  "calories": 680,
  "heartRate": {
    "avg": 155,
    "max": 178,
    "zones": {
      "zone1": 0,
      "zone2": 5.2,
      "zone3": 18.5,
      "zone4": 8.3,
      "zone5": 0
    }
  }
}
```

### 5.2.3 활동 목록

**요청:**
```http
GET /api/v1/activities?startDate=2025-12-01&type=running&limit=20 HTTP/1.1
Authorization: Bearer {access_token}
```

**쿼리 매개변수:**
- `startDate`: ISO 8601 날짜 (선택)
- `endDate`: ISO 8601 날짜 (선택)
- `type`: 활동 유형 필터 (선택)
- `limit`: 페이지당 결과 (기본: 20, 최대: 100)
- `offset`: 페이지네이션 오프셋 (기본: 0)

**응답 (200 OK):**
```json
{
  "data": [
    {
      "id": "550e8400-e29b-41d4-a716-446655440000",
      "type": "running",
      "distance": 8000,
      "calories": 680
    }
  ],
  "pagination": {
    "total": 45,
    "limit": 20,
    "offset": 0,
    "hasMore": true
  }
}
```

---

## 5.3 운동 엔드포인트

### 5.3.1 운동 생성

**요청:**
```http
POST /api/v1/workouts HTTP/1.1
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "type": "interval_training",
  "name": "5x1000m 인터벌",
  "startTime": "2025-12-27T17:00:00Z",
  "intervals": [
    {
      "number": 1,
      "type": "warmup",
      "duration": 600
    },
    {
      "number": 2,
      "type": "work",
      "duration": 240,
      "distance": 1000
    }
  ]
}
```

---

## 5.4 지표 엔드포인트

### 5.4.1 심박수 데이터 조회

**요청:**
```http
GET /api/v1/metrics/heart-rate?startTime=2025-12-27T07:00:00Z HTTP/1.1
Authorization: Bearer {access_token}
```

**응답 (200 OK):**
```json
{
  "userId": "user_12345",
  "startTime": "2025-12-27T07:00:00Z",
  "readings": [
    {"timestamp": "2025-12-27T07:00:00Z", "bpm": 142},
    {"timestamp": "2025-12-27T07:00:05Z", "bpm": 145}
  ],
  "avgBpm": 155,
  "maxBpm": 178,
  "minBpm": 142
}
```

### 5.4.2 걸음 수 데이터 조회

**요청:**
```http
GET /api/v1/metrics/steps?date=2025-12-27 HTTP/1.1
Authorization: Bearer {access_token}
```

**응답 (200 OK):**
```json
{
  "date": "2025-12-27",
  "totalSteps": 12547,
  "goal": 10000,
  "progress": 125.47
}
```

### 5.4.3 칼로리 데이터 조회

**요청:**
```http
GET /api/v1/metrics/calories?date=2025-12-27 HTTP/1.1
Authorization: Bearer {access_token}
```

**응답 (200 OK):**
```json
{
  "date": "2025-12-27",
  "totalCalories": 2847,
  "breakdown": {
    "bmr": 1685,
    "active": 1052,
    "epoc": 110
  }
}
```

---

## 5.5 목표 엔드포인트

### 5.5.1 목표 생성

**요청:**
```http
POST /api/v1/goals HTTP/1.1
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "type": "daily_steps",
  "name": "일일 만보",
  "target": 10000,
  "unit": "걸음",
  "period": "daily"
}
```

**응답 (201 Created):**
```json
{
  "id": "goal_880b2133",
  "type": "daily_steps",
  "target": 10000,
  "current": 12547,
  "progress": 125.47,
  "status": "active"
}
```

---

## 5.6 요약 엔드포인트

### 5.6.1 일일 요약

**요청:**
```http
GET /api/v1/summary/daily?date=2025-12-27 HTTP/1.1
Authorization: Bearer {access_token}
```

**응답 (200 OK):**
```json
{
  "date": "2025-12-27",
  "steps": 12547,
  "distance": 9.8,
  "calories": 2847,
  "activeMinutes": 87,
  "workouts": 2
}
```

---

## 5.7 WebSocket 실시간 스트리밍

### 5.7.1 심박수 스트림

**연결:**
```javascript
const ws = new WebSocket('wss://api.example.com/v1/stream/heart-rate');

ws.onopen = () => {
  ws.send(JSON.stringify({
    type: 'authenticate',
    token: 'access_token_here'
  }));
};

ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log('심박수:', data.bpm);
};
```

**서버 메시지:**
```json
{
  "type": "heartRateUpdate",
  "timestamp": "2025-12-27T07:15:30Z",
  "bpm": 155,
  "zone": 3
}
```

---

## 5.8 속도 제한

### 5.8.1 속도 제한 헤더

**응답 헤더:**
```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 995
X-RateLimit-Reset: 1735290000
```

### 5.8.2 속도 제한 티어

```
무료 티어:
- 시간당 1,000 요청
- 일당 10,000 요청

표준 티어:
- 시간당 5,000 요청
- 일당 50,000 요청

프리미엄 티어:
- 시간당 20,000 요청
- 일당 200,000 요청
```

---

## 5.9 오류 처리

### 5.9.1 오류 응답 형식

```json
{
  "error": {
    "code": "ERROR_CODE",
    "message": "사람이 읽을 수 있는 오류 메시지",
    "details": {
      "field": "추가 컨텍스트"
    }
  }
}
```

### 5.9.2 일반 오류 코드

```
AUTHENTICATION_FAILED      - 잘못된 자격 증명
AUTHORIZATION_FAILED       - 권한 불충분
RESOURCE_NOT_FOUND        - 요청한 리소스가 존재하지 않음
VALIDATION_ERROR          - 요청 검증 실패
RATE_LIMIT_EXCEEDED       - 너무 많은 요청
INTERNAL_ERROR            - 서버 오류
```

---

## 핵심 요점

✓ RESTful API는 표준 HTTP 규칙 및 상태 코드 준수

✓ OAuth 2.0은 안전한 인증 및 권한 부여 제공

✓ 활동, 운동, 지표, 목표, 요약을 다루는 포괄적인 엔드포인트

✓ WebSocket은 실시간 심박수 및 활동 스트리밍 지원

✓ 속도 제한은 API 인프라를 보호하고 공정한 사용 보장

✓ 상세한 오류 코드 및 메시지로 일관된 오류 처리

✓ 모든 응답은 데이터 스키마와 일치하는 표준화된 JSON 형식 사용

---

**다음:** [6장: 프로토콜 사양 →](06-protocol.md)

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
