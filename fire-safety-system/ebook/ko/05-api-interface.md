# 5장: API 인터페이스 설계 (2단계)

## 학습 목표

이 장을 마치면 다음을 수행할 수 있습니다:

1. RESTful API 설계 원칙을 이해하고 소방 안전 시스템에 적용
2. 장치 관리, 경보 관리 및 시스템 상태 API 엔드포인트 사용
3. JWT 기반 인증 구현 및 역할 기반 액세스 제어 구성
4. WebSocket 연결을 통한 실시간 이벤트 스트리밍 통합
5. 표준 HTTP 상태 코드와 오류 처리 패턴 적용

---

## 개요

WIA 표준의 2단계는 소프트웨어 애플리케이션, 빌딩 관리 시스템 및 타사 서비스가 소방 안전 인프라와 균일하게 상호 작용할 수 있도록 하는 포괄적인 API 사양을 정의합니다. 이 장에서는 RESTful API 설계, WebSocket 이벤트 스트리밍, 인증 메커니즘 및 구현 모범 사례를 자세히 설명합니다.

---

## API 아키텍처

### RESTful API 설계 원칙

WIA 표준은 입증된 효과와 광범위한 채택을 위해 RESTful (Representational State Transfer) 아키텍처를 채택합니다:

```
REST 원칙 적용:

1. 리소스 기반 URL
   ✓ /api/v1/devices/{id}
   ✗ /api/v1/getDevice?id=123

2. 표준 HTTP 메서드
   ✓ GET, POST, PUT, DELETE
   ✗ URL의 사용자 정의 동사

3. 무상태 통신
   ✓ 각 요청에 필요한 모든 정보 포함
   ✗ 서버 측 세션 상태

4. JSON 표현
   ✓ Content-Type: application/json
   ✗ 사용자 정의 이진 형식

5. HATEOAS (선택사항)
   ✓ 관련 리소스에 대한 링크
```

### API 기본 URL 구조

```
형식: https://{host}:{port}/api/{version}/

예제:
https://panel.building.com:8443/api/v1/
https://192.168.1.100:443/api/v1/
https://fire-safety-api.example.com/api/v1/

구성 요소:
- 프로토콜: HTTPS (TLS 1.3 필수)
- 호스트: 패널 IP 또는 도메인 이름
- 포트: 443 (표준 HTTPS) 또는 사용자 정의
- API 접두사: /api/
- 버전: /v1/ (현재 버전)
```

---

## 장치 관리 API

### 모든 장치 나열

**엔드포인트:** `GET /api/v1/devices`

**설명:** 시스템의 모든 장치 목록 검색

**쿼리 매개변수:**
```
type       : 장치 유형별 필터링 (smoke, heat, flame, co, multi)
status     : 상태별 필터링 (normal, alarm, trouble, disabled)
building   : 건물 식별자별 필터링
floor      : 층 번호별 필터링
zone       : 구역 식별자별 필터링
limit      : 페이지당 결과 수 (기본값: 100, 최대: 1000)
offset     : 페이지네이션 오프셋 (기본값: 0)
```

**요청 예제:**
```http
GET /api/v1/devices?type=smoke&status=normal&limit=50 HTTP/1.1
Host: panel.building.com
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
Accept: application/json
```

**응답 (200 OK):**
```json
{
  "total": 237,
  "limit": 50,
  "offset": 0,
  "devices": [
    {
      "sensorId": "550e8400-e29b-41d4-a716-446655440000",
      "type": "smoke",
      "location": {
        "building": "Main Tower",
        "floor": 12,
        "zone": "East Wing E12-A"
      },
      "status": "normal",
      "readings": {
        "value": 0.8,
        "unit": "OD/meter",
        "timestamp": "2025-12-27T14:32:15Z"
      },
      "metadata": {
        "manufacturer": "SafetyTech Inc",
        "model": "ST-5000",
        "firmwareVersion": "2.4.1"
      }
    },
    {
      "sensorId": "7c9e6679-7425-40de-944b-e07fc1f90ae7",
      "type": "smoke",
      "location": {
        "building": "Main Tower",
        "floor": 11,
        "zone": "West Wing W11-B"
      },
      "status": "normal",
      "readings": {
        "value": 1.2,
        "unit": "OD/meter",
        "timestamp": "2025-12-27T14:32:18Z"
      },
      "metadata": {
        "manufacturer": "SafetyTech Inc",
        "model": "ST-5000",
        "firmwareVersion": "2.4.1"
      }
    }
  ]
}
```

### 장치 세부 정보 가져오기

**엔드포인트:** `GET /api/v1/devices/{id}`

**설명:** 특정 장치에 대한 자세한 정보 검색

**경로 매개변수:**
```
id : 장치 UUID
```

**요청 예제:**
```http
GET /api/v1/devices/550e8400-e29b-41d4-a716-446655440000 HTTP/1.1
Host: panel.building.com
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
Accept: application/json
```

**응답 (200 OK):**
```json
{
  "sensorId": "550e8400-e29b-41d4-a716-446655440000",
  "type": "smoke",
  "location": {
    "building": "Main Tower",
    "floor": 12,
    "zone": "East Wing E12-A",
    "coordinates": {
      "x": 45.2,
      "y": 23.8,
      "z": 3.5
    }
  },
  "status": "normal",
  "readings": {
    "value": 0.8,
    "unit": "OD/meter",
    "timestamp": "2025-12-27T14:32:15Z"
  },
  "metadata": {
    "manufacturer": "SafetyTech Inc",
    "model": "ST-5000",
    "firmwareVersion": "2.4.1",
    "installationDate": "2024-06-15T10:00:00Z",
    "lastMaintenance": "2025-10-12T09:30:00Z",
    "serialNumber": "ST5K-2024-061567",
    "certifications": ["UL-268", "FM-3260", "EN-54-7"]
  },
  "history": {
    "totalAlarms": 3,
    "lastAlarm": "2025-11-08T09:15:30Z",
    "totalTroubles": 1,
    "lastTrouble": "2025-09-22T14:42:10Z",
    "uptime": 99.97
  }
}
```

**응답 (404 Not Found):**
```json
{
  "error": "DeviceNotFound",
  "message": "ID가 550e8400-e29b-41d4-a716-446655440000인 장치를 찾을 수 없습니다",
  "timestamp": "2025-12-27T14:35:20Z"
}
```

### 장치 구성 업데이트

**엔드포인트:** `PUT /api/v1/devices/{id}`

**설명:** 장치 구성 매개변수 업데이트

**필요한 역할:** 기술자 또는 관리자

**요청 예제:**
```http
PUT /api/v1/devices/550e8400-e29b-41d4-a716-446655440000 HTTP/1.1
Host: panel.building.com
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
Content-Type: application/json

{
  "location": {
    "building": "Main Tower",
    "floor": 12,
    "zone": "East Wing E12-B"
  },
  "sensitivity": "high",
  "alarmThreshold": 3.5
}
```

**응답 (200 OK):**
```json
{
  "message": "장치 구성이 성공적으로 업데이트되었습니다",
  "deviceId": "550e8400-e29b-41d4-a716-446655440000",
  "updatedFields": ["location.zone", "sensitivity", "alarmThreshold"],
  "timestamp": "2025-12-27T14:40:15Z"
}
```

### 장치 테스트

**엔드포인트:** `POST /api/v1/devices/{id}/test`

**설명:** 장치 자가 테스트 시작

**필요한 역할:** 기술자 또는 관리자

**요청 예제:**
```http
POST /api/v1/devices/550e8400-e29b-41d4-a716-446655440000/test HTTP/1.1
Host: panel.building.com
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
Content-Type: application/json

{
  "testType": "functional",
  "silenceNotifications": true
}
```

**응답 (202 Accepted):**
```json
{
  "message": "장치 테스트가 시작되었습니다",
  "deviceId": "550e8400-e29b-41d4-a716-446655440000",
  "testId": "test-9a8b7c6d-5e4f-3a2b-1c0d-9e8f7a6b5c4d",
  "estimatedDuration": 30,
  "status": "in_progress",
  "timestamp": "2025-12-27T14:45:00Z"
}
```

### 장치 제거

**엔드포인트:** `DELETE /api/v1/devices/{id}`

**설명:** 시스템에서 장치 제거

**필요한 역할:** 관리자

**요청 예제:**
```http
DELETE /api/v1/devices/550e8400-e29b-41d4-a716-446655440000 HTTP/1.1
Host: panel.building.com
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

**응답 (200 OK):**
```json
{
  "message": "장치가 성공적으로 제거되었습니다",
  "deviceId": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": "2025-12-27T14:50:00Z"
}
```

---

## 경보 관리 API

### 활성 경보 나열

**엔드포인트:** `GET /api/v1/alarms`

**설명:** 경보 이벤트 목록 검색

**쿼리 매개변수:**
```
status     : 상태별 필터링 (active, acknowledged, resolved)
type       : 유형별 필터링 (fire, supervisory, trouble, test)
priority   : 우선순위별 필터링 (critical, high, medium, low)
building   : 건물별 필터링
floor      : 층별 필터링
startDate  : 이 타임스탬프 이후 이벤트 필터링 (ISO 8601)
endDate    : 이 타임스탬프 이전 이벤트 필터링 (ISO 8601)
limit      : 페이지당 결과 수 (기본값: 100, 최대: 1000)
offset     : 페이지네이션 오프셋
```

**요청 예제:**
```http
GET /api/v1/alarms?status=active&priority=critical HTTP/1.1
Host: panel.building.com
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
Accept: application/json
```

**응답 (200 OK):**
```json
{
  "total": 2,
  "limit": 100,
  "offset": 0,
  "alarms": [
    {
      "eventId": "9f4e2c8a-5d3b-4a7e-9c1f-8e2d4a6c3b5f",
      "eventType": "fire",
      "priority": "critical",
      "source": {
        "deviceId": "550e8400-e29b-41d4-a716-446655440000",
        "deviceType": "smoke_detector",
        "location": {
          "building": "Main Tower",
          "floor": 12,
          "zone": "East Wing E12-A"
        }
      },
      "timestamp": "2025-12-27T14:32:15Z",
      "description": "12층 동쪽 구역 E12-A에서 연기 감지",
      "status": "active",
      "duration": 180
    },
    {
      "eventId": "1a2b3c4d-5e6f-4a5b-8c9d-0e1f2a3b4c5d",
      "eventType": "fire",
      "priority": "critical",
      "source": {
        "deviceId": "7c9e6679-7425-40de-944b-e07fc1f90ae7",
        "deviceType": "heat_detector",
        "location": {
          "building": "West Annex",
          "floor": 3,
          "zone": "Storage Room W3-S2"
        }
      },
      "timestamp": "2025-12-27T14:35:42Z",
      "description": "창고에서 고온 감지",
      "status": "active",
      "duration": 47
    }
  ]
}
```

### 경보 세부 정보 가져오기

**엔드포인트:** `GET /api/v1/alarms/{id}`

**설명:** 특정 경보에 대한 자세한 정보 검색

**응답 (200 OK):**
```json
{
  "eventId": "9f4e2c8a-5d3b-4a7e-9c1f-8e2d4a6c3b5f",
  "eventType": "fire",
  "priority": "critical",
  "source": {
    "deviceId": "550e8400-e29b-41d4-a716-446655440000",
    "deviceType": "smoke_detector",
    "location": {
      "building": "Main Tower",
      "floor": 12,
      "zone": "East Wing E12-A"
    }
  },
  "timestamp": "2025-12-27T14:32:15Z",
  "description": "12층 동쪽 구역 E12-A에서 연기 감지",
  "notifications": [
    "panel",
    "monitoring-center",
    "emergency-services",
    "building-management"
  ],
  "status": "active",
  "actions": [
    {
      "type": "notification_sent",
      "target": "panel",
      "timestamp": "2025-12-27T14:32:16Z"
    },
    {
      "type": "notification_sent",
      "target": "monitoring-center",
      "timestamp": "2025-12-27T14:32:17Z"
    },
    {
      "type": "emergency_services_notified",
      "timestamp": "2025-12-27T14:32:18Z"
    }
  ],
  "relatedDevices": [
    "550e8400-e29b-41d4-a716-446655440000",
    "660f9511-f3ac-52e5-b827-557766551111"
  ]
}
```

### 경보 확인

**엔드포인트:** `POST /api/v1/alarms/{id}/acknowledge`

**설명:** 활성 경보 확인

**필요한 역할:** 운영자, 기술자 또는 관리자

**요청 예제:**
```http
POST /api/v1/alarms/9f4e2c8a-5d3b-4a7e-9c1f-8e2d4a6c3b5f/acknowledge HTTP/1.1
Host: panel.building.com
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
Content-Type: application/json

{
  "notes": "소방서 출동 중. 12층 대피 중."
}
```

**응답 (200 OK):**
```json
{
  "message": "경보가 성공적으로 확인되었습니다",
  "eventId": "9f4e2c8a-5d3b-4a7e-9c1f-8e2d4a6c3b5f",
  "acknowledgedBy": "operator-john-smith",
  "acknowledgedAt": "2025-12-27T14:35:20Z",
  "timestamp": "2025-12-27T14:35:20Z"
}
```

### 경보 알림 음소거

**엔드포인트:** `POST /api/v1/alarms/{id}/silence`

**설명:** 경보 상태를 유지하면서 청각/시각 알림 음소거

**필요한 역할:** 운영자, 기술자 또는 관리자

**요청 예제:**
```http
POST /api/v1/alarms/9f4e2c8a-5d3b-4a7e-9c1f-8e2d4a6c3b5f/silence HTTP/1.1
Host: panel.building.com
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
Content-Type: application/json

{
  "duration": 300,
  "reason": "소방서 현장에서 조사 중"
}
```

**응답 (200 OK):**
```json
{
  "message": "경보 알림이 음소거되었습니다",
  "eventId": "9f4e2c8a-5d3b-4a7e-9c1f-8e2d4a6c3b5f",
  "silencedBy": "operator-john-smith",
  "silencedAt": "2025-12-27T14:40:00Z",
  "silenceDuration": 300,
  "autoResumeAt": "2025-12-27T14:45:00Z"
}
```

---

## 시스템 상태 API

### 전체 시스템 상태 가져오기

**엔드포인트:** `GET /api/v1/status`

**설명:** 전체 시스템 상태 및 통계 검색

**응답 (200 OK):**
```json
{
  "systemStatus": "operational",
  "timestamp": "2025-12-27T14:50:00Z",
  "uptime": 2592000,
  "statistics": {
    "totalDevices": 2847,
    "devicesNormal": 2840,
    "devicesAlarm": 2,
    "devicesTrouble": 5,
    "devicesDisabled": 0,
    "activeAlarms": 2,
    "acknowledgedAlarms": 3,
    "resolvedAlarmsToday": 12
  },
  "panels": {
    "total": 8,
    "online": 8,
    "offline": 0
  },
  "network": {
    "status": "healthy",
    "latency": 12,
    "packetLoss": 0.02
  },
  "power": {
    "status": "ac_power",
    "batteryCharge": 100,
    "batteryHealth": "good"
  }
}
```

### 제어 패널 상태 가져오기

**엔드포인트:** `GET /api/v1/status/panels`

**설명:** 모든 제어 패널의 상태 검색

**응답 (200 OK):**
```json
{
  "panels": [
    {
      "panelId": "panel-001",
      "name": "Main Tower North",
      "status": "operational",
      "location": {
        "building": "Main Tower",
        "floor": 1,
        "room": "Security Office"
      },
      "connectedDevices": 856,
      "activeAlarms": 1,
      "uptime": 2592000,
      "firmwareVersion": "4.2.1",
      "lastCommunication": "2025-12-27T14:50:00Z"
    },
    {
      "panelId": "panel-002",
      "name": "Main Tower South",
      "status": "operational",
      "location": {
        "building": "Main Tower",
        "floor": 1,
        "room": "Electrical Room"
      },
      "connectedDevices": 743,
      "activeAlarms": 0,
      "uptime": 2592000,
      "firmwareVersion": "4.2.1",
      "lastCommunication": "2025-12-27T14:49:58Z"
    }
  ]
}
```

---

## 인증 및 권한 부여

### JWT 인증

**토큰 요청:**
```http
POST /api/v1/auth/login HTTP/1.1
Host: panel.building.com
Content-Type: application/json

{
  "username": "john.smith",
  "password": "SecurePassword123!",
  "mfaCode": "123456"
}
```

**토큰 응답:**
```json
{
  "accessToken": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJzdWIiOiJqb2huLnNtaXRoIiwicm9sZSI6Im9wZXJhdG9yIiwiaWF0IjoxNzM1MzE1MjAwLCJleHAiOjE3MzUzMTg4MDB9.signature",
  "refreshToken": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "expiresIn": 3600,
  "tokenType": "Bearer"
}
```

**토큰 사용:**
```http
GET /api/v1/devices HTTP/1.1
Host: panel.building.com
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

### 역할 기반 액세스 제어

```
역할 계층 및 권한:

┌─────────────────────────────────────────────┐
│ 관리자 (전체 액세스)                        │
│ ✓ 모든 기술자 권한                          │
│ ✓ 사용자 관리                               │
│ ✓ 시스템 구성                               │
│ ✓ 장치 추가/제거                            │
│ ✓ 감사 로그 액세스                          │
├─────────────────────────────────────────────┤
│ 기술자                                      │
│ ✓ 모든 운영자 권한                          │
│ ✓ 장치 구성                                 │
│ ✓ 장치 테스트                               │
│ ✓ 유지보수 작업                             │
├─────────────────────────────────────────────┤
│ 운영자                                      │
│ ✓ 모든 뷰어 권한                            │
│ ✓ 경보 확인                                 │
│ ✓ 알림 음소거                               │
│ ✓ 이벤트에 메모 추가                        │
├─────────────────────────────────────────────┤
│ 뷰어 (읽기 전용)                            │
│ ✓ 장치 상태 보기                            │
│ ✓ 경보 이벤트 보기                          │
│ ✓ 시스템 상태 보기                          │
│ ✓ 보고서 다운로드                           │
└─────────────────────────────────────────────┘
```

---

## WebSocket 실시간 이벤트

### 연결 설정

```javascript
// 이벤트 스트림에 연결
const ws = new WebSocket('wss://panel.building.com/api/v1/events');

// 인증 포함
ws.onopen = () => {
  ws.send(JSON.stringify({
    type: 'authenticate',
    token: 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...'
  }));
};

// 인증 응답 처리
ws.onmessage = (event) => {
  const data = JSON.parse(event.data);

  if (data.type === 'authenticated') {
    console.log('이벤트 스트림에 연결됨');
  }
};
```

### 이벤트 유형

**경보 트리거됨:**
```json
{
  "eventType": "alarm.triggered",
  "timestamp": "2025-12-27T14:32:15Z",
  "data": {
    "eventId": "9f4e2c8a-5d3b-4a7e-9c1f-8e2d4a6c3b5f",
    "alarmType": "fire",
    "priority": "critical",
    "source": {
      "deviceId": "550e8400-e29b-41d4-a716-446655440000",
      "location": {
        "building": "Main Tower",
        "floor": 12,
        "zone": "East Wing E12-A"
      }
    }
  }
}
```

**장치 상태 변경됨:**
```json
{
  "eventType": "device.status.changed",
  "timestamp": "2025-12-27T14:38:20Z",
  "data": {
    "deviceId": "a1b2c3d4-e5f6-4a5b-8c9d-0e1f2a3b4c5d",
    "previousStatus": "normal",
    "newStatus": "trouble",
    "reason": "통신 실패"
  }
}
```

---

## 오류 처리

### 표준 오류 응답 형식

```json
{
  "error": "ErrorCode",
  "message": "사람이 읽을 수 있는 오류 설명",
  "details": {
    "field": "specificField",
    "reason": "상세한 설명"
  },
  "timestamp": "2025-12-27T14:50:00Z",
  "requestId": "req-1234567890"
}
```

### 일반적인 HTTP 상태 코드

```
200 OK                  - 성공적인 요청
201 Created             - 리소스가 성공적으로 생성됨
202 Accepted            - 요청이 수락됨, 비동기 처리 중
204 No Content          - 성공, 응답 본문 없음
400 Bad Request         - 잘못된 요청 형식
401 Unauthorized        - 인증 필요/실패
403 Forbidden           - 권한 부족
404 Not Found           - 리소스를 찾을 수 없음
409 Conflict            - 현재 상태와 요청 충돌
429 Too Many Requests   - 속도 제한 초과
500 Internal Server Error - 서버 오류
503 Service Unavailable - 일시적 서비스 중단
```

---

## 핵심 요점

1. **RESTful API 설계**는 모든 소방 안전 시스템에 대해 보편적이고 예측 가능한 인터페이스를 제공합니다.

2. **장치 및 경보 관리** 엔드포인트는 완전한 시스템 제어 및 모니터링을 가능하게 합니다.

3. **역할 기반 액세스 제어**는 적절한 권한 수준을 통해 보안을 보장합니다.

4. **WebSocket 이벤트**는 최소 대기 시간으로 실시간 업데이트를 제공합니다.

5. **표준 오류 처리**는 강력한 클라이언트 구현을 촉진합니다.

---

## 복습 문제

1. RESTful API 설계의 핵심 원칙은 무엇인가요?
2. 장치 관리를 위한 주요 HTTP 메서드와 엔드포인트는 무엇인가요?
3. JWT 인증은 어떻게 작동하며 토큰은 어떻게 사용되나요?
4. 네 가지 역할 수준과 각각의 권한은 무엇인가요?
5. WebSocket 이벤트가 RESTful API보다 실시간 업데이트에 더 나은 이유는 무엇인가요?

---

## 다음 단계

6장에서는 소방 안전 시스템의 신뢰할 수 있고 안전하며 적시의 작동을 보장하는 통신 프로토콜을 정의하는 3단계를 탐구합니다.

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
