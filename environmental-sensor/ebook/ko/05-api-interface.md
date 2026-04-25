# 제5장: API 인터페이스

## Phase 2: 환경 센서 데이터를 위한 RESTful API

---

## 학습 목표

이 장을 마치면 다음을 수행할 수 있습니다:

1. RESTful API 설계 원칙과 URL 구조 이해하기
2. 센서 탐색, 등록 및 세부 정보 검색 API 사용하기
3. 단일 및 일괄 데이터 제출 엔드포인트 구현하기
4. 최신 및 과거 센서 데이터 쿼리하기
5. WebSocket 및 Server-Sent Events를 위한 실시간 스트리밍 설정하기
6. API 키, OAuth 2.0, RBAC를 사용한 인증 구성하기
7. 오류 처리 및 속도 제한 모범 사례 적용하기

---

## 5.1 RESTful API 설계 원칙

WIA-ENE-027 Phase 2 사양은 업계 모범 사례를 따르는 RESTful API를 정의합니다.

### URL 구조

```
https://{host}/api/v{version}/{resource}
```

**예:**
```
https://api.example.com/api/v1/sensors
https://api.example.com/api/v1/sensors/ENV-AIR-001/data
https://api.example.com/api/v1/sensors/ENV-AIR-001/data/latest
```

### HTTP 메서드

| 메서드 | 목적 | 멱등성 | 안전 |
|--------|---------|------------|------|
| GET | 리소스 검색 | 예 | 예 |
| POST | 리소스 생성, 데이터 제출 | 아니오 | 아니오 |
| PUT | 리소스 업데이트 | 예 | 아니오 |
| DELETE | 리소스 제거 | 예 | 아니오 |

### 표준 응답 코드

| 코드 | 의미 | 사용 사례 |
|------|---------|----------|
| 200 | OK | 성공적인 요청 |
| 201 | Created | 리소스 성공적으로 생성됨 |
| 400 | Bad Request | 잘못된 요청 데이터 |
| 401 | Unauthorized | 인증 필요 |
| 403 | Forbidden | 권한 부족 |
| 404 | Not Found | 리소스가 존재하지 않음 |
| 429 | Too Many Requests | 속도 제한 초과 |
| 500 | Internal Server Error | 서버 오류 |

---

## 5.2 센서 탐색 및 등록

### 모든 센서 나열

```http
GET /api/v1/sensors
```

**쿼리 매개변수:**
- `type`: 센서 유형으로 필터링 (`air_quality`, `water_quality`, `soil`, `meteorological`)
- `location`: 지리적 경계 상자 `lat1,lon1,lat2,lon2`
- `status`: 상태로 필터링 (`active`, `inactive`, `maintenance`)
- `limit`: 최대 결과 (기본값 100, 최대 1000)
- `offset`: 페이지네이션 오프셋

**예제 요청:**
```http
GET /api/v1/sensors?type=air_quality&status=active&limit=50
Authorization: Bearer YOUR_API_KEY
```

**응답 (200 OK):**
```json
{
  "total": 1547,
  "count": 50,
  "offset": 0,
  "sensors": [
    {
      "deviceId": "ENV-AIR-001",
      "type": "air_quality",
      "location": {
        "latitude": 37.5665,
        "longitude": 126.9780,
        "altitude": 38.5
      },
      "status": "active",
      "lastUpdate": "2025-01-09T10:30:00Z",
      "capabilities": ["pm2_5", "pm10", "temperature", "humidity"]
    },
    {
      "deviceId": "ENV-AIR-002",
      "type": "air_quality",
      "location": {
        "latitude": 37.5700,
        "longitude": 126.9800,
        "altitude": 42.0
      },
      "status": "active",
      "lastUpdate": "2025-01-09T10:32:00Z",
      "capabilities": ["pm2_5", "pm10", "co2", "voc"]
    }
  ],
  "links": {
    "self": "/api/v1/sensors?type=air_quality&status=active&limit=50&offset=0",
    "next": "/api/v1/sensors?type=air_quality&status=active&limit=50&offset=50"
  }
}
```

### 센서 세부 정보 가져오기

```http
GET /api/v1/sensors/{deviceId}
```

**예제 요청:**
```http
GET /api/v1/sensors/ENV-AIR-001
Authorization: Bearer YOUR_API_KEY
```

**응답 (200 OK):**
```json
{
  "deviceId": "ENV-AIR-001",
  "type": "air_quality",
  "manufacturer": "AirSense Corp",
  "model": "AS-3000",
  "firmware": "v2.4.1",
  "location": {
    "latitude": 37.5665,
    "longitude": 126.9780,
    "altitude": 38.5,
    "accuracy": 10.0
  },
  "status": "active",
  "installDate": "2024-06-15T09:00:00Z",
  "lastCalibration": "2024-11-15T09:00:00Z",
  "nextCalibration": "2025-05-15T09:00:00Z",
  "capabilities": {
    "pm2_5": {
      "range": [0, 500],
      "unit": "μg/m³",
      "accuracy": 2.0,
      "method": "laser_scattering"
    },
    "pm10": {
      "range": [0, 1000],
      "unit": "μg/m³",
      "accuracy": 4.5,
      "method": "laser_scattering"
    },
    "temperature": {
      "range": [-40, 85],
      "unit": "°C",
      "accuracy": 0.3
    }
  },
  "metadata": {
    "owner": "Seoul Metropolitan Government",
    "contact": "env@seoul.go.kr",
    "notes": "Downtown monitoring station"
  }
}
```

### 새 센서 등록

```http
POST /api/v1/sensors
```

**요청 본문:**
```json
{
  "deviceId": "ENV-AIR-NEW-001",
  "type": "air_quality",
  "manufacturer": "AirSense Corp",
  "model": "AS-3000",
  "firmware": "v2.4.1",
  "location": {
    "latitude": 37.5650,
    "longitude": 126.9750,
    "altitude": 40.0
  },
  "capabilities": ["pm2_5", "pm10", "temperature", "humidity"]
}
```

**응답 (201 Created):**
```json
{
  "deviceId": "ENV-AIR-NEW-001",
  "status": "registered",
  "registrationDate": "2025-01-09T10:45:00Z",
  "message": "Sensor registered successfully"
}
```

---

## 5.3 데이터 제출 엔드포인트

### 센서 데이터 제출

```http
POST /api/v1/sensors/{deviceId}/data
```

**요청 본문 (WIA-ENE-027 Phase 1 형식):**
```json
{
  "version": "1.0.0",
  "standard": "WIA-ENE-027",
  "deviceId": "ENV-AIR-001",
  "timestamp": "2025-01-09T10:30:00.000Z",
  "sensorType": "air_quality",
  "readings": {
    "pm2_5": {
      "value": 15.3,
      "unit": "μg/m³",
      "method": "laser_scattering"
    },
    "pm10": {
      "value": 22.8,
      "unit": "μg/m³",
      "method": "laser_scattering"
    }
  },
  "quality": {
    "overall": "good",
    "flags": []
  }
}
```

**응답 (201 Created):**
```json
{
  "status": "accepted",
  "timestamp": "2025-01-09T10:30:00.000Z",
  "dataId": "data-abc123",
  "validation": {
    "passed": true,
    "warnings": []
  }
}
```

**오류 응답 (400 Bad Request):**
```json
{
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Invalid data format",
    "details": {
      "field": "readings.pm2_5.value",
      "issue": "Value 1500 exceeds maximum range of 500"
    },
    "timestamp": "2025-01-09T10:30:00Z"
  }
}
```

### 일괄 데이터 제출

```http
POST /api/v1/sensors/{deviceId}/data/batch
```

**요청 본문:**
```json
{
  "data": [
    {
      "timestamp": "2025-01-09T10:00:00.000Z",
      "readings": {"pm2_5": {"value": 14.5, "unit": "μg/m³"}}
    },
    {
      "timestamp": "2025-01-09T10:05:00.000Z",
      "readings": {"pm2_5": {"value": 15.1, "unit": "μg/m³"}}
    },
    {
      "timestamp": "2025-01-09T10:10:00.000Z",
      "readings": {"pm2_5": {"value": 15.8, "unit": "μg/m³"}}
    }
  ]
}
```

---

## 5.4 데이터 검색 및 쿼리

### 최신 읽기 가져오기

```http
GET /api/v1/sensors/{deviceId}/data/latest
```

**응답 (200 OK):**
```json
{
  "version": "1.0.0",
  "standard": "WIA-ENE-027",
  "deviceId": "ENV-AIR-001",
  "timestamp": "2025-01-09T10:30:00.000Z",
  "sensorType": "air_quality",
  "readings": {
    "pm2_5": {"value": 15.3, "unit": "μg/m³"},
    "pm10": {"value": 22.8, "unit": "μg/m³"}
  }
}
```

### 과거 데이터 가져오기

```http
GET /api/v1/sensors/{deviceId}/data
```

**쿼리 매개변수:**
- `start`: ISO 8601 시작 타임스탬프 (필수)
- `end`: ISO 8601 종료 타임스탬프 (필수)
- `parameters`: 쉼표로 구분된 목록 (예: `pm2_5,temperature`)
- `aggregation`: `none` | `hourly` | `daily` | `weekly` | `monthly`
- `format`: `json` | `csv` | `xml`
- `limit`: 최대 레코드 (기본값 1000)
- `offset`: 페이지네이션 오프셋

**예제 요청:**
```http
GET /api/v1/sensors/ENV-AIR-001/data?start=2025-01-08T00:00:00Z&end=2025-01-09T00:00:00Z&aggregation=hourly&parameters=pm2_5
```

**응답 (200 OK):**
```json
{
  "deviceId": "ENV-AIR-001",
  "start": "2025-01-08T00:00:00Z",
  "end": "2025-01-09T00:00:00Z",
  "aggregation": "hourly",
  "count": 24,
  "data": [
    {
      "timestamp": "2025-01-08T00:00:00Z",
      "pm2_5": {
        "mean": 12.5,
        "min": 8.2,
        "max": 18.3,
        "stddev": 2.1,
        "count": 12
      }
    },
    {
      "timestamp": "2025-01-08T01:00:00Z",
      "pm2_5": {
        "mean": 14.2,
        "min": 10.5,
        "max": 19.8,
        "stddev": 2.5,
        "count": 12
      }
    }
  ]
}
```

### 대량 데이터 검색

```http
POST /api/v1/data/bulk
```

**요청 본문:**
```json
{
  "sensorIds": ["ENV-AIR-001", "ENV-AIR-002", "ENV-AIR-003"],
  "start": "2025-01-09T00:00:00Z",
  "end": "2025-01-09T12:00:00Z",
  "parameters": ["pm2_5", "temperature"]
}
```

---

## 5.5 실시간 스트리밍 인터페이스

### WebSocket 연결

**연결 URL:**
```
wss://api.example.com/v1/stream
```

**구독 메시지:**
```json
{
  "action": "subscribe",
  "sensors": ["ENV-AIR-001", "ENV-AIR-002"],
  "parameters": ["pm2_5", "temperature"]
}
```

**데이터 메시지:**
```json
{
  "type": "measurement",
  "deviceId": "ENV-AIR-001",
  "timestamp": "2025-01-09T10:30:00Z",
  "readings": {
    "pm2_5": {"value": 15.3, "unit": "μg/m³"}
  }
}
```

### Server-Sent Events

```http
GET /api/v1/sensors/{deviceId}/stream
Accept: text/event-stream
```

**응답 스트림:**
```
event: measurement
data: {"timestamp":"2025-01-09T10:30:00Z","pm2_5":15.3}

event: measurement
data: {"timestamp":"2025-01-09T10:35:00Z","pm2_5":15.5}

event: heartbeat
data: {"timestamp":"2025-01-09T10:36:00Z"}
```

---

## 5.6 인증 및 권한 부여

### API 키

```http
GET /api/v1/sensors
Authorization: Bearer YOUR_API_KEY
```

### OAuth 2.0

```http
GET /api/v1/sensors
Authorization: Bearer {access_token}
```

### 역할 기반 액세스 제어

| 역할 | 권한 |
|------|-------------|
| Public Viewer | 공개 센서 데이터 읽기 |
| Data Consumer | 모든 센서 데이터 읽기 |
| Sensor Operator | 데이터 읽기, 데이터 제출, 구성 업데이트 |
| Administrator | 전체 액세스 |

---

## 5.7 오류 처리 및 속도 제한

### 오류 응답 형식

```json
{
  "error": {
    "code": "SENSOR_NOT_FOUND",
    "message": "Sensor with ID 'ENV-AIR-999' not found",
    "details": {
      "requestedId": "ENV-AIR-999"
    },
    "timestamp": "2025-01-09T10:30:00Z",
    "requestId": "req_abc123"
  }
}
```

### 속도 제한 헤더

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 987
X-RateLimit-Reset: 1704067200
```

### 속도 제한 티어

| 티어 | 시간당 요청 | 버스트 제한 |
|------|---------------|-------------|
| Free | 1,000 | 100/분 |
| Standard | 10,000 | 500/분 |
| Professional | 100,000 | 2,000/분 |
| Enterprise | 무제한 | 맞춤형 |

---

## 5.8 복습 문제 및 핵심 요점

### 복습 문제

1. 50개 센서가 있는 수질 모니터링 시스템을 위한 API 엔드포인트를 설계하세요. 어떤 엔드포인트를 구현하시겠습니까?

2. 센서가 5분마다 데이터를 제출합니다. 월간 API 호출을 계산하고 적절한 속도 제한 티어를 권장하세요.

3. 10개 센서에서 지난 7일 동안 시간당 집계된 PM2.5 데이터를 검색하는 쿼리를 생성하세요.

4. 최대 범위 500을 초과하는 2000 μg/m³의 PM2.5 값을 제출하는 센서에 대한 오류 처리를 설계하세요.

### 핵심 요점

1. **RESTful 설계**: 표준 HTTP 메서드 및 URL 구조로 직관적인 API 사용이 가능합니다.

2. **데이터 제출**: POST /sensors/{id}/data 엔드포인트는 Phase 1 형식 데이터를 수락합니다.

3. **쿼리**: 유연한 쿼리 매개변수(시간 범위, 집계, 매개변수)로 다양한 사용 사례가 가능합니다.

4. **실시간 스트리밍**: WebSocket 및 SSE 인터페이스로 실시간 데이터 액세스가 가능합니다.

5. **인증**: API 키, OAuth 2.0 및 RBAC로 다양한 보안 요구사항을 지원합니다.

6. **속도 제한**: 계층화된 제한으로 남용을 방지하면서 합법적인 대량 사용을 지원합니다.

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity

**다음 장: [제6장: 통신 프로토콜](06-protocol.md)**
