# 제5장: API 인터페이스 및 통합

## Phase 2: RESTful API, WebSocket 스트리밍 및 SDK 구현

---

## 5.1 API 아키텍처 개요

### 설계 철학

WIA 배송 드론 API는 리소스 기반 작업에 REST 원칙을, 실시간 스트리밍에 WebSocket을 따릅니다. 아키텍처는 다음을 우선시합니다:

- **단순성**: 직관적인 리소스 명명 및 HTTP 시맨틱
- **일관성**: 균일한 응답 형식 및 오류 처리
- **보안**: 인증, 권한 부여 및 암호화
- **성능**: 효율적인 페이징, 캐싱 및 스트리밍
- **확장성**: 버전 관리 및 하위 호환성

### 기본 URL 구조

```
https://{environment}.api.wia.com/v{version}/{resource}

환경:
  - production: api.wia.com
  - staging: staging.api.wia.com
  - sandbox: sandbox.api.wia.com

예시:
  https://api.wia.com/v1/missions
  https://api.wia.com/v1/drones/WIA-DRN-X1-0042/telemetry
  https://staging.api.wia.com/v1/flight-plans
```

### 응답 형식

**성공 응답**:
```json
{
  "success": true,
  "data": {
    "missionId": "MSN-20250101-1234",
    "status": "SCHEDULED",
    ...
  },
  "meta": {
    "requestId": "req-abc123",
    "timestamp": "2025-01-01T10:00:00.000Z",
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
    "message": "픽업 위치 좌표가 유효하지 않습니다",
    "field": "pickup.location.latitude",
    "details": {
      "received": 200.5,
      "constraint": "위도는 -90에서 90 사이여야 합니다"
    }
  },
  "meta": {
    "requestId": "req-abc123",
    "timestamp": "2025-01-01T10:00:00.000Z"
  }
}
```

### HTTP 상태 코드

| 코드 | 의미 | 사용 사례 |
|------|------|----------|
| 200 | OK | 성공적인 GET, PUT |
| 201 | Created | 성공적인 POST |
| 202 | Accepted | 비동기 작업 시작됨 |
| 204 | No Content | 성공적인 DELETE |
| 400 | Bad Request | 유효성 검사 오류 |
| 401 | Unauthorized | 인증 누락/무효 |
| 403 | Forbidden | 권한 부족 |
| 404 | Not Found | 리소스 없음 |
| 409 | Conflict | 리소스 상태 충돌 |
| 429 | Too Many Requests | 속도 제한 초과 |
| 500 | Internal Error | 서버 오류 |

---

## 5.2 미션 관리 API

### 미션 생성

**POST /api/v1/missions**

새로운 배송 미션 생성:

```json
// 요청
{
  "pickup": {
    "location": {
      "latitude": 37.5665,
      "longitude": 126.9780,
      "altitude": 0
    },
    "address": "서울시 종로구 세종대로 123",
    "contact": {
      "name": "김영희",
      "phone": "+82-10-1234-5678",
      "email": "kim@example.com"
    },
    "instructions": "초인종 누르시면, 패키지는 리셉션에 있습니다"
  },
  "dropoff": {
    "location": {
      "latitude": 37.5012,
      "longitude": 127.0396,
      "altitude": 0
    },
    "address": "서울시 강남구 테헤란로 456",
    "contact": {
      "name": "이철수",
      "phone": "+82-10-9876-5432",
      "email": "lee@example.com"
    },
    "instructions": "현관문 앞에 놓아주세요",
    "deliveryWindow": {
      "start": "2025-01-01T10:00:00Z",
      "end": "2025-01-01T12:00:00Z"
    }
  },
  "package": {
    "weight": 2.5,
    "dimensions": {
      "length": 30,
      "width": 20,
      "height": 15
    },
    "fragile": false,
    "value": 150000
  },
  "priority": "STANDARD",
  "scheduledTime": "2025-01-01T10:00:00Z"
}

// 응답 (201 Created)
{
  "success": true,
  "data": {
    "missionId": "MSN-20250101-1234",
    "status": "SCHEDULED",
    "pickup": { ... },
    "dropoff": { ... },
    "package": { ... },
    "scheduling": {
      "scheduledPickup": "2025-01-01T10:00:00Z",
      "estimatedDelivery": "2025-01-01T10:18:00Z",
      "estimatedFlightTime": 12
    },
    "drone": {
      "assigned": "WIA-DRN-X1-0042",
      "name": "팔콘-42",
      "class": "LIGHT"
    },
    "tracking": {
      "url": "https://track.wia.com/MSN-20250101-1234",
      "code": "ABCD1234"
    },
    "createdAt": "2025-01-01T09:30:00.000Z"
  }
}
```

### 미션 상태 조회

**GET /api/v1/missions/{missionId}**

```json
// 응답
{
  "success": true,
  "data": {
    "missionId": "MSN-20250101-1234",
    "status": "IN_FLIGHT",
    "phase": "EN_ROUTE_DROPOFF",
    "progress": 65,
    "currentPosition": {
      "latitude": 37.5300,
      "longitude": 127.0100,
      "altitude": 100,
      "heading": 45,
      "speed": 18.5
    },
    "timing": {
      "scheduledPickup": "2025-01-01T10:00:00Z",
      "actualPickup": "2025-01-01T10:02:00Z",
      "estimatedDelivery": "2025-01-01T10:14:00Z",
      "eta": 180
    },
    "battery": {
      "current": 72,
      "estimatedAtDelivery": 58
    },
    "weather": {
      "temperature": 18,
      "windSpeed": 5.2,
      "conditions": "CLEAR"
    }
  }
}
```

### 미션 목록 조회

**GET /api/v1/missions**

쿼리 파라미터:
- `status`: 상태로 필터 (PENDING, SCHEDULED, IN_FLIGHT, COMPLETED, CANCELLED)
- `droneId`: 할당된 드론으로 필터
- `startDate`: 날짜 이후 미션
- `endDate`: 날짜 이전 미션
- `limit`: 페이지당 결과 (기본 20, 최대 100)
- `offset`: 페이지네이션 오프셋

```
GET /api/v1/missions?status=COMPLETED&startDate=2025-01-01&limit=50
```

### 미션 취소

**POST /api/v1/missions/{missionId}/cancel**

```json
// 요청
{
  "reason": "고객 요청으로 취소",
  "code": "CUSTOMER_REQUEST"
}

// 응답
{
  "success": true,
  "data": {
    "missionId": "MSN-20250101-1234",
    "status": "CANCELLED",
    "cancellation": {
      "reason": "고객 요청으로 취소",
      "code": "CUSTOMER_REQUEST",
      "cancelledAt": "2025-01-01T09:45:00.000Z",
      "refundEligible": true
    }
  }
}
```

---

## 5.3 드론 관리 API

### 드론 등록

**POST /api/v1/drones**

```json
// 요청
{
  "serialNumber": "SN-2025-00042",
  "manufacturer": "WIA 에어로스페이스",
  "model": "팔콘 X1",
  "class": "LIGHT",
  "specifications": {
    "mtow": 8.5,
    "maxPayload": 3.0,
    "maxRange": 15,
    "maxSpeed": 22,
    "maxFlightTime": 35,
    "batteryCapacity": 10000
  },
  "certifications": {
    "remoteId": "RID-2025-00042",
    "typeApproval": "MOLIT-TC-2024-0042",
    "insurancePolicy": "POL-2025-001234"
  },
  "homeBase": {
    "latitude": 37.5665,
    "longitude": 126.9780,
    "name": "서울 물류 센터"
  }
}

// 응답 (201 Created)
{
  "success": true,
  "data": {
    "droneId": "WIA-DRN-X1-0042",
    "status": "REGISTERED",
    "serialNumber": "SN-2025-00042",
    "name": "팔콘-42",
    ...
  }
}
```

### 드론 상태 조회

**GET /api/v1/drones/{droneId}**

```json
// 응답
{
  "success": true,
  "data": {
    "droneId": "WIA-DRN-X1-0042",
    "name": "팔콘-42",
    "status": "AVAILABLE",
    "operational": true,
    "location": {
      "latitude": 37.5665,
      "longitude": 126.9780,
      "altitude": 0,
      "updatedAt": "2025-01-01T09:55:00.000Z"
    },
    "battery": {
      "level": 100,
      "health": 95,
      "cycleCount": 142
    },
    "maintenance": {
      "lastInspection": "2025-01-01",
      "nextInspection": "2025-01-15",
      "flightHours": 245.5,
      "totalFlights": 892
    },
    "currentMission": null,
    "todayStats": {
      "flights": 5,
      "distance": 42.5,
      "deliveries": 5,
      "flightTime": 1.2
    }
  }
}
```

### 드론 명령

**POST /api/v1/drones/{droneId}/commands**

```json
// 요청: 홈 복귀
{
  "command": "RETURN_TO_HOME",
  "priority": "HIGH",
  "parameters": {}
}

// 요청: 위치로 이동
{
  "command": "GOTO_POSITION",
  "priority": "NORMAL",
  "parameters": {
    "latitude": 37.5400,
    "longitude": 127.0000,
    "altitude": 50,
    "speed": 10
  }
}

// 응답
{
  "success": true,
  "data": {
    "commandId": "CMD-20250101-5678",
    "status": "ACCEPTED",
    "droneId": "WIA-DRN-X1-0042",
    "command": "RETURN_TO_HOME",
    "acceptedAt": "2025-01-01T10:05:00.000Z",
    "estimatedCompletion": "2025-01-01T10:12:00.000Z"
  }
}
```

---

## 5.4 실시간 스트리밍 (WebSocket)

### 연결 수립

```javascript
const ws = new WebSocket('wss://api.wia.com/v1/stream');

ws.onopen = () => {
  // 인증
  ws.send(JSON.stringify({
    type: 'AUTH',
    token: 'Bearer eyJhbGciOiJIUzI1NiIs...'
  }));

  // 채널 구독
  ws.send(JSON.stringify({
    type: 'SUBSCRIBE',
    channels: [
      { type: 'mission', id: 'MSN-20250101-1234' },
      { type: 'drone', id: 'WIA-DRN-X1-0042' },
      { type: 'fleet', id: 'FLEET-001' }
    ],
    options: {
      telemetryRate: 1,  // Hz
      includeVideo: false
    }
  }));
};

ws.onmessage = (event) => {
  const message = JSON.parse(event.data);
  handleStreamMessage(message);
};

function handleStreamMessage(message) {
  switch (message.type) {
    case 'TELEMETRY':
      updateDronePosition(message.data);
      break;
    case 'STATUS':
      updateDroneStatus(message.data);
      break;
    case 'EVENT':
      handleEvent(message.data);
      break;
    case 'MISSION_UPDATE':
      updateMissionStatus(message.data);
      break;
  }
}
```

### 스트림 메시지 타입

**원격측정 스트림**:
```json
{
  "type": "TELEMETRY",
  "channel": "drone",
  "id": "WIA-DRN-X1-0042",
  "timestamp": "2025-01-01T10:05:30.123Z",
  "data": {
    "position": {
      "latitude": 37.5300,
      "longitude": 127.0100,
      "altitude": 100
    },
    "velocity": {
      "groundSpeed": 18.5,
      "verticalSpeed": 0.2
    },
    "attitude": {
      "heading": 45,
      "roll": 2.1,
      "pitch": 5.3
    },
    "battery": 72
  }
}
```

**미션 업데이트 스트림**:
```json
{
  "type": "MISSION_UPDATE",
  "channel": "mission",
  "id": "MSN-20250101-1234",
  "timestamp": "2025-01-01T10:05:00.000Z",
  "data": {
    "previousStatus": "PICKUP",
    "currentStatus": "EN_ROUTE",
    "progress": 15,
    "eta": 780,
    "event": "DEPARTED_PICKUP"
  }
}
```

**이벤트 스트림**:
```json
{
  "type": "EVENT",
  "channel": "drone",
  "id": "WIA-DRN-X1-0042",
  "timestamp": "2025-01-01T10:06:30.000Z",
  "data": {
    "eventType": "WARNING",
    "code": "WIND_ADVISORY",
    "severity": "MEDIUM",
    "message": "풍속 증가, 상황 모니터링 중",
    "details": {
      "currentWindSpeed": 8.5,
      "threshold": 10
    }
  }
}
```

---

## 5.5 함대 관리 API

### 함대 상태 조회

**GET /api/v1/fleets/{fleetId}**

```json
// 응답
{
  "success": true,
  "data": {
    "fleetId": "FLEET-001",
    "name": "수도권 함대",
    "summary": {
      "totalDrones": 25,
      "available": 18,
      "inFlight": 5,
      "charging": 2,
      "maintenance": 0
    },
    "drones": [
      {
        "droneId": "WIA-DRN-X1-0042",
        "status": "IN_FLIGHT",
        "currentMission": "MSN-20250101-1234",
        "battery": 72
      },
      ...
    ],
    "todayMetrics": {
      "totalFlights": 142,
      "totalDeliveries": 140,
      "successRate": 98.6,
      "totalDistance": 1250.5,
      "totalFlightTime": 42.3,
      "averageDeliveryTime": 12.5
    }
  }
}
```

### 함대 분석

**GET /api/v1/fleets/{fleetId}/analytics**

쿼리 파라미터:
- `period`: DAY, WEEK, MONTH, YEAR
- `startDate`: 기간 시작
- `endDate`: 기간 종료
- `metrics`: 쉼표로 구분된 메트릭 이름

```json
// 응답
{
  "success": true,
  "data": {
    "period": {
      "start": "2025-01-01",
      "end": "2025-01-31"
    },
    "metrics": {
      "deliveries": {
        "total": 4250,
        "successful": 4180,
        "failed": 35,
        "cancelled": 35,
        "successRate": 98.35
      },
      "flights": {
        "total": 4300,
        "totalDistance": 52500,
        "totalFlightTime": 1850,
        "averageFlightTime": 25.8
      },
      "efficiency": {
        "deliveriesPerDrone": 170,
        "utilizationRate": 72.5,
        "energyPerDelivery": 0.85
      },
      "reliability": {
        "mtbf": 125.5,
        "downtime": 2.3,
        "incidentRate": 0.12
      }
    },
    "trends": {
      "daily": [
        { "date": "2025-01-01", "deliveries": 142, "successRate": 98.5 },
        { "date": "2025-01-02", "deliveries": 138, "successRate": 99.2 },
        ...
      ]
    }
  }
}
```

---

## 5.6 인증 및 보안

### API 키 인증

```http
GET /api/v1/missions HTTP/1.1
Host: api.wia.com
Authorization: ApiKey EXAMPLE_API_KEY_REPLACE_ME
```

### JWT Bearer 토큰

```http
POST /api/v1/auth/token HTTP/1.1
Host: api.wia.com
Content-Type: application/json

{
  "clientId": "client_abc123",
  "clientSecret": "secret_xyz789",
  "scope": "missions:read missions:write drones:read"
}

// 응답
{
  "access_token": "eyJhbGciOiJIUzI1NiIs...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "missions:read missions:write drones:read"
}
```

### 권한 범위

| 범위 | 접근 수준 |
|------|----------|
| missions:read | 미션 데이터 조회 |
| missions:write | 미션 생성/수정 |
| drones:read | 드론 데이터 조회 |
| drones:control | 드론 명령 전송 |
| fleet:manage | 함대 관리 접근 |
| admin | 전체 관리 접근 |

### 속도 제한

```http
HTTP/1.1 200 OK
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 998
X-RateLimit-Reset: 1704110400
```

| 엔드포인트 카테고리 | 제한 | 시간 창 |
|-------------------|------|---------|
| 읽기 작업 | 1000/분 | API 키당 |
| 쓰기 작업 | 100/분 | API 키당 |
| 스트리밍 연결 | 10 | 동시 |
| 드론 명령 | 60/분 | 드론당 |

---

## 5.7 SDK 구현

### TypeScript/JavaScript SDK

```typescript
import { WIADeliveryClient } from '@wia/delivery-sdk';

// 클라이언트 초기화
const client = new WIADeliveryClient({
  apiKey: 'EXAMPLE_API_KEY_REPLACE_ME',
  environment: 'production'
});

// 미션 생성
async function createDeliveryMission() {
  const mission = await client.missions.create({
    pickup: {
      location: { latitude: 37.5665, longitude: 126.9780 },
      address: '서울시 종로구 세종대로 123',
      contact: { name: '김영희', phone: '+82-10-1234-5678' }
    },
    dropoff: {
      location: { latitude: 37.5012, longitude: 127.0396 },
      address: '서울시 강남구 테헤란로 456',
      contact: { name: '이철수', phone: '+82-10-9876-5432' }
    },
    package: {
      weight: 2.5,
      dimensions: { length: 30, width: 20, height: 15 }
    },
    priority: 'STANDARD'
  });

  console.log(`미션 생성됨: ${mission.missionId}`);
  return mission;
}

// 실시간 미션 추적
async function trackMission(missionId: string) {
  const stream = client.missions.track(missionId);

  stream.on('position', (data) => {
    console.log(`위치: ${data.latitude}, ${data.longitude}`);
  });

  stream.on('status', (data) => {
    console.log(`상태: ${data.status}, 진행률: ${data.progress}%`);
  });

  stream.on('delivered', (data) => {
    console.log(`배송 완료: ${data.timestamp}`);
    stream.close();
  });
}

// 함대 상태 조회
async function getFleetStatus() {
  const fleet = await client.fleets.get('FLEET-001');

  console.log(`총 드론: ${fleet.summary.totalDrones}`);
  console.log(`가용: ${fleet.summary.available}`);
  console.log(`비행 중: ${fleet.summary.inFlight}`);

  return fleet;
}
```

### Python SDK

```python
from wia_delivery import DeliveryClient

# 클라이언트 초기화
client = DeliveryClient(
    api_key='EXAMPLE_API_KEY_REPLACE_ME',
    environment='production'
)

# 미션 생성
def create_delivery_mission():
    mission = client.missions.create(
        pickup={
            'location': {'latitude': 37.5665, 'longitude': 126.9780},
            'address': '서울시 종로구 세종대로 123',
            'contact': {'name': '김영희', 'phone': '+82-10-1234-5678'}
        },
        dropoff={
            'location': {'latitude': 37.5012, 'longitude': 127.0396},
            'address': '서울시 강남구 테헤란로 456',
            'contact': {'name': '이철수', 'phone': '+82-10-9876-5432'}
        },
        package={
            'weight': 2.5,
            'dimensions': {'length': 30, 'width': 20, 'height': 15}
        },
        priority='STANDARD'
    )

    print(f"미션 생성됨: {mission.mission_id}")
    return mission

# 실시간 미션 추적
def track_mission(mission_id: str):
    for update in client.missions.track(mission_id):
        if update.type == 'position':
            print(f"위치: {update.latitude}, {update.longitude}")
        elif update.type == 'status':
            print(f"상태: {update.status}, 진행률: {update.progress}%")
        elif update.type == 'delivered':
            print(f"배송 완료: {update.timestamp}")
            break

# 분석 조회
def get_fleet_analytics():
    analytics = client.fleets.analytics(
        fleet_id='FLEET-001',
        period='MONTH',
        start_date='2025-01-01',
        end_date='2025-01-31'
    )

    print(f"총 배송: {analytics.metrics.deliveries.total}")
    print(f"성공률: {analytics.metrics.deliveries.success_rate}%")

    return analytics
```

---

## 한국 특화 API 확장

### K-드론 시스템 연동 API

```http
POST /api/v1/kdrone/flight-approval HTTP/1.1
Host: api.wia.com
Content-Type: application/json

{
  "flightPlanId": "FP-20250101-1234",
  "pilotLicense": "DRP-1-2025-00567",
  "droneRegistration": "K-DRN-2025-001234",
  "requestedTime": {
    "start": "2025-01-01T10:00:00Z",
    "end": "2025-01-01T12:00:00Z"
  },
  "area": {
    "type": "Polygon",
    "coordinates": [[...]]
  },
  "purpose": "DELIVERY"
}

// 응답
{
  "success": true,
  "data": {
    "approvalNumber": "FA-20250101-1234",
    "status": "APPROVED",
    "validFrom": "2025-01-01T10:00:00Z",
    "validTo": "2025-01-01T12:00:00Z",
    "restrictions": [],
    "notifyBefore": 30
  }
}
```

### 한국 배송 서비스 통합

```typescript
// 배달의민족 연동 예시
const baeminIntegration = await client.integrations.connect({
  provider: 'BAEMIN',
  credentials: {
    apiKey: 'baemin_api_key',
    storeId: 'STORE-001'
  },
  webhooks: {
    orderReceived: 'https://your-server.com/baemin/orders',
    deliveryUpdate: 'https://your-server.com/baemin/updates'
  }
});

// 쿠팡 연동 예시
const coupangIntegration = await client.integrations.connect({
  provider: 'COUPANG',
  credentials: {
    accessKey: 'coupang_access_key',
    secretKey: 'coupang_secret_key'
  }
});
```

---

## 장 요약

WIA 배송 드론 API는 드론 배송 운영의 모든 측면을 관리하기 위한 포괄적인 인터페이스를 제공합니다. RESTful 설계는 직관적인 리소스 관리를 가능하게 하고, WebSocket 스트리밍은 미션 진행 및 드론 상태에 대한 실시간 가시성을 제공합니다.

미션 관리 API는 생성부터 완료까지 전체 배송 생명 주기를 지원하며, 풍부한 상태 추적 및 이벤트 알림을 제공합니다. 드론 관리는 등록, 모니터링 및 명령/제어 작업을 가능하게 합니다. 함대 관리 API는 운영 최적화를 위한 집계된 뷰 및 분석을 제공합니다.

API 키 및 JWT 토큰을 통한 인증은 보안 접근을 보장하며, 세분화된 권한 범위로 적절한 접근 제어가 가능합니다. 속도 제한은 시스템 리소스를 보호하면서 공정한 사용을 보장합니다. TypeScript 및 Python SDK는 개발자의 통합을 단순화합니다.

---

## 핵심 요약

1. **리소스용 RESTful API, 실시간용 WebSocket** 스트리밍
2. **일관된 응답 형식** (성공/오류 구조)
3. **포괄적인 미션 생명 주기** 관리 (생성부터 완료까지)
4. **WebSocket 구독을 통한 실시간 추적**
5. **TypeScript 및 Python SDK 구현**으로 통합 간소화

---

## 복습 문제

1. 미션이 성공적으로 생성되었음을 나타내는 HTTP 상태 코드는?
2. 배송 미션에 대한 실시간 위치 추적을 어떻게 구현하겠습니까?
3. 일괄 미션 생성(다중 배송)을 위한 API 엔드포인트를 설계하시오.
4. 서버 간 통합에 적합한 인증 방법은?
5. 속도 제한이 API를 남용으로부터 어떻게 보호합니까?

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · 널리 인간을 이롭게 하라
