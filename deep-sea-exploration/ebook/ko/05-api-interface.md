# 제5장: API 인터페이스 사양 (Phase 2)

## 수중 차량 제어 및 데이터 접근을 위한 RESTful 및 실시간 API

---

## 5.1 API 아키텍처 개요

### 설계 철학

WIA 심해 탐사 API는 리소스 기반 작업에 REST 원칙을 따르고 실시간 스트리밍에 WebSocket을 사용합니다. 아키텍처는 다음을 우선시합니다:

- **무상태성**: 각 요청이 모든 필요한 정보 포함
- **리소스 지향**: 데이터 접근을 위한 명확한 URL 패턴
- **일관성**: 엔드포인트 전반에 걸쳐 균일한 응답 형식
- **실시간 기능**: 원격측정 및 제어를 위한 WebSocket 스트림
- **보안**: 모든 레이어에서 인증 및 권한 부여

### API 기본 URL 구조

```
https://{호스트}/api/v{버전}/{리소스}

예시:
https://vehicle.kiost.ac.kr/api/v1/telemetry
https://data.noaa.gov/api/v1/bathymetry
https://rov-control.ship.local/api/v1/commands
```

### 응답 형식

모든 응답은 일관된 구조의 JSON을 사용합니다:

**성공 응답**:
```json
{
  "success": true,
  "data": { ... },
  "meta": {
    "timestamp": "2025-01-15T14:30:00.000Z",
    "requestId": "req-abc123",
    "processingTime": 45
  }
}
```

**오류 응답**:
```json
{
  "success": false,
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "잘못된 깊이 값: 0에서 11000 사이여야 합니다",
    "field": "depth",
    "details": { ... }
  },
  "meta": {
    "timestamp": "2025-01-15T14:30:00.000Z",
    "requestId": "req-abc123"
  }
}
```

### HTTP 상태 코드

| 코드 | 의미 | 사용 |
|------|------|------|
| 200 | OK | 성공적인 GET, PUT |
| 201 | Created | 성공적인 POST |
| 204 | No Content | 성공적인 DELETE |
| 400 | Bad Request | 검증 오류 |
| 401 | Unauthorized | 인증 필요 |
| 403 | Forbidden | 권한 부족 |
| 404 | Not Found | 리소스가 존재하지 않음 |
| 429 | Too Many Requests | 속도 제한 초과 |
| 500 | Internal Error | 서버 오류 |

---

## 5.2 차량 제어 엔드포인트

### 명령 실행

**POST /api/v1/commands**

차량에서 명령 실행:

```json
// 요청
{
  "commandType": "THRUSTER_CONTROL",
  "target": "ALL",
  "parameters": {
    "mode": "STATION_KEEP",
    "heading": 180.0,
    "altitude": 5.0
  },
  "priority": "NORMAL",
  "timeout": 30000
}

// 응답
{
  "success": true,
  "data": {
    "commandId": "cmd-2025-01-15-001",
    "status": "ACCEPTED",
    "estimatedExecutionTime": 2500,
    "queuePosition": 0
  }
}
```

### 명령 유형

| 명령 유형 | 설명 | 매개변수 |
|----------|------|----------|
| THRUSTER_CONTROL | 추진 제어 | mode, heading, speed, depth |
| LIGHTS_CONTROL | 조명 제어 | intensity, target |
| CAMERA_CONTROL | 카메라 작동 | action, zoom, pan, tilt |
| MANIPULATOR_CONTROL | 암 작동 | arm, action, position |
| SAMPLING | 샘플 수집 | sampler, action |
| NAVIGATION | 웨이포인트 항법 | waypoint, speed |
| EMERGENCY | 비상 절차 | action |

### 명령 상태

**GET /api/v1/commands/{commandId}**

```json
{
  "success": true,
  "data": {
    "commandId": "cmd-2025-01-15-001",
    "commandType": "THRUSTER_CONTROL",
    "status": "COMPLETED",
    "submittedAt": "2025-01-15T14:30:00.000Z",
    "completedAt": "2025-01-15T14:30:02.100Z",
    "result": {
      "success": true,
      "message": "정지 유지 설정됨"
    }
  }
}
```

---

## 5.3 원격측정 스트리밍 (WebSocket)

### 연결 설정

```javascript
const ws = new WebSocket('wss://vehicle.example.com/api/v1/telemetry/stream');

ws.onopen = () => {
  // 채널 구독
  ws.send(JSON.stringify({
    action: 'subscribe',
    channels: ['navigation', 'environment', 'systems'],
    rate: 1,  // Hz
    format: 'COMPACT'
  }));
};

ws.onmessage = (event) => {
  const message = JSON.parse(event.data);
  handleTelemetry(message);
};
```

### 구독 메시지

```json
{
  "action": "subscribe",
  "channels": ["navigation", "environment", "systems", "alerts"],
  "rate": 1,
  "format": "FULL",
  "filter": {
    "systems": ["thrusters", "cameras"]
  }
}
```

### 원격측정 채널

| 채널 | 데이터 내용 | 일반적인 속도 |
|------|-----------|-------------|
| navigation | 위치, 방향, 속도 | 1-10 Hz |
| environment | CTD, 수류 | 0.1-1 Hz |
| systems | 스러스터, 카메라, 전력 상태 | 1 Hz |
| alerts | 경고 및 오류 메시지 | 이벤트 기반 |
| video | 비디오 스트림 메타데이터 | 프레임 속도 |
| commands | 명령 상태 업데이트 | 이벤트 기반 |

### 원격측정 메시지 형식

```json
{
  "channel": "navigation",
  "timestamp": "2025-01-15T14:30:00.123Z",
  "sequenceNumber": 12345,
  "data": {
    "position": {
      "latitude": 36.7977,
      "longitude": -121.8472,
      "depth": 3547.2,
      "altitude": 12.5
    },
    "orientation": {
      "heading": 187.5,
      "pitch": -2.3,
      "roll": 1.7
    },
    "velocity": {
      "forward": 0.5,
      "lateral": 0.1,
      "vertical": -0.02
    }
  }
}
```

---

## 5.4 데이터 쿼리 및 내보내기

### 원격측정 히스토리

**GET /api/v1/telemetry**

쿼리 매개변수:
- `start`: 시작 타임스탬프 (ISO8601)
- `end`: 종료 타임스탬프 (ISO8601)
- `channels`: 쉼표로 구분된 채널 목록
- `resolution`: 간략화 계수 (1, 10, 60, 300)
- `format`: json, csv, netcdf
- `limit`: 최대 레코드 (기본 1000)
- `offset`: 페이지네이션 오프셋

```
GET /api/v1/telemetry?start=2025-01-15T10:00:00Z&end=2025-01-15T14:00:00Z&channels=navigation,environment&resolution=60&format=json
```

### 수심측량 데이터

**GET /api/v1/bathymetry**

```
GET /api/v1/bathymetry?bbox=-121.85,-121.84,36.79,36.80&resolution=1&format=geotiff
```

응답:
```json
{
  "data": {
    "type": "grid",
    "boundingBox": {...},
    "resolution": 1.0,
    "crs": "EPSG:4326",
    "downloadUrl": "/api/v1/downloads/bathy-abc123.tiff",
    "expiresAt": "2025-01-15T15:30:00.000Z",
    "size": 52428800
  }
}
```

### 샘플 레코드

**GET /api/v1/samples**

```json
{
  "data": {
    "total": 145,
    "page": 1,
    "pageSize": 20,
    "samples": [
      {
        "sampleId": "SAMPLE-2025-01-15-001",
        "sampleType": "BIOLOGICAL",
        "taxon": "Riftia pachyptila",
        "location": {...},
        "collectedAt": "2025-01-15T14:30:00.000Z"
      }
    ]
  }
}
```

**POST /api/v1/samples**

새 샘플 레코드 생성:
```json
{
  "sampleType": "BIOLOGICAL",
  "location": {
    "latitude": 36.7977,
    "longitude": -121.8472,
    "depth": 3547.2
  },
  "specimen": {
    "taxonCandidate": "미확인 튜브웜",
    "description": "신종, 붉은 색상"
  }
}
```

---

## 5.5 미션 관리 API

### 미션 CRUD 작업

**GET /api/v1/missions**
```json
{
  "data": {
    "missions": [
      {
        "missionId": "MISSION-2025-01-15-001",
        "name": "열수 조사 알파",
        "status": "IN_PROGRESS",
        "startedAt": "2025-01-15T10:00:00.000Z",
        "vehicle": "ROV-HEMIRE-001",
        "progress": 65
      }
    ]
  }
}
```

**POST /api/v1/missions**
```json
{
  "name": "열수 조사 베타",
  "description": "분출구 필드 후속 조사",
  "vehicle": "ROV-HEMIRE-001",
  "waypoints": [
    {"latitude": 36.7977, "longitude": -121.8472, "depth": 3500, "action": "SURVEY"},
    {"latitude": 36.7980, "longitude": -121.8470, "depth": 3520, "action": "SAMPLE"}
  ],
  "parameters": {
    "maxDepth": 3600,
    "surveySpeed": 0.5
  }
}
```

### 미션 제어

**POST /api/v1/missions/{missionId}/control**
```json
{
  "action": "PAUSE"  // START, PAUSE, RESUME, ABORT
}
```

---

## 5.6 인증 및 권한 부여

### 인증 방법

**API 키 인증**:
```
Authorization: ApiKey EXAMPLE_API_KEY_REPLACE_ME
```

**JWT 베어러 토큰**:
```
Authorization: Bearer eyJhbGciOiJIUzI1NiIs...
```

### 권한 수준

| 역할 | 권한 |
|------|------|
| OBSERVER | 원격측정 읽기, 데이터 쿼리 |
| SCIENTIST | Observer + 샘플/주석 생성 |
| PILOT | Scientist + 차량 제어 |
| SUPERVISOR | Pilot + 미션 관리 |
| ADMIN | 전체 접근 + 사용자 관리 |

### 역할 기반 접근 제어

```json
{
  "user": {
    "id": "user-12345",
    "email": "scientist@kiost.ac.kr",
    "roles": ["SCIENTIST"],
    "permissions": [
      "telemetry:read",
      "data:read",
      "samples:create",
      "annotations:create"
    ]
  }
}
```

---

## 5.7 속도 제한 및 할당량

### 속도 제한

| 엔드포인트 범주 | 제한 | 창 |
|---------------|------|-----|
| 원격측정 읽기 | 100 req/sec | IP당 |
| 데이터 쿼리 | 60 req/min | API 키당 |
| 명령 | 10 req/sec | 차량당 |
| 대량 내보내기 | 10 req/hour | API 키당 |

### 속도 제한 헤더

```http
X-RateLimit-Limit: 100
X-RateLimit-Remaining: 87
X-RateLimit-Reset: 1705329600
```

---

## 5.8 SDK 참조

### TypeScript SDK

```typescript
import { WIADeepSeaClient } from '@wia/deep-sea-sdk';

const client = new WIADeepSeaClient({
  baseUrl: 'https://vehicle.example.com/api/v1',
  apiKey: 'EXAMPLE_API_KEY_REPLACE_ME'
});

// 현재 원격측정 가져오기
const telemetry = await client.telemetry.getCurrent();
console.log(`깊이: ${telemetry.position.depth}m`);

// 실시간 업데이트 구독
client.telemetry.subscribe({
  channels: ['navigation', 'environment'],
  rate: 1,
  onMessage: (data) => {
    console.log('원격측정 업데이트:', data);
  }
});

// 명령 전송
const result = await client.commands.execute({
  commandType: 'LIGHTS_CONTROL',
  parameters: { intensity: 75 }
});
```

### Python SDK

```python
from wia_deepsea import DeepSeaClient

client = DeepSeaClient(
    base_url='https://vehicle.example.com/api/v1',
    api_key='EXAMPLE_API_KEY_REPLACE_ME'
)

# 현재 원격측정 가져오기
telemetry = client.telemetry.get_current()
print(f"깊이: {telemetry.position.depth}m")

# 샘플 쿼리
samples = client.samples.list(
    sample_type='BIOLOGICAL',
    start_date='2025-01-15',
    limit=50
)

# 샘플 레코드 생성
new_sample = client.samples.create(
    sample_type='GEOLOGICAL',
    location={'latitude': 36.7977, 'longitude': -121.8472, 'depth': 3547},
    specimen={'rock_type': 'BASALT'}
)
```

---

## 장 요약

WIA 심해 탐사 표준의 Phase 2는 수중 차량 및 해양학 데이터 시스템과 상호작용하기 위한 포괄적인 API 사양을 제공합니다. RESTful 설계는 광범위한 호환성을 보장하고, WebSocket 스트리밍은 실시간 원격측정 및 제어를 가능하게 합니다.

인증, 속도 제한, 역할 기반 접근 제어는 안전한 운영을 보장하고, TypeScript 및 Python의 SDK 구현은 개발자의 통합을 단순화합니다.

---

## 핵심 요점

1. **리소스에는 REST, 실시간 스트리밍에는 WebSocket** 원격측정
2. **success/error 구조를 가진 일관된 응답 형식**
3. **역할 기반 접근 제어**가 차량 운영 보호
4. **속도 제한이 남용 방지**하면서 합법적 사용 허용
5. **TypeScript와 Python SDK**가 개발 가속화

---

## 복습 질문

1. 새 샘플 레코드를 생성하기 위해 어떤 HTTP 메서드를 사용해야 합니까?
2. WebSocket 원격측정 구독은 어떻게 작동합니까?
3. 차량 명령을 실행하려면 어떤 역할이 필요합니까?
4. 특정 영역의 수심측량 데이터를 쿼리하는 API 요청을 설계하세요.
5. 속도 제한은 API 클라이언트에 어떻게 전달됩니까?

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · 널리 인간을 이롭게 하라
