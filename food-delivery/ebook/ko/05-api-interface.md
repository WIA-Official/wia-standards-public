# 제5장: API 인터페이스 사양

---

## 학습 목표

이 장을 마치면 다음을 수행할 수 있습니다:

1. WIA-IND-009 표준의 RESTful API 엔드포인트 구조 이해
2. 인증 및 권한 부여 메커니즘 구현
3. 주문, 드라이버, 경로 관련 API 엔드포인트 활용
4. WebSocket을 통한 실시간 추적 구현
5. 웹훅 알림 시스템 구성
6. API 오류 처리 및 속도 제한 관리

---

## 5.1 개요

WIA-IND-009 표준은 실시간 기능을 위한 WebSocket 지원과 함께 포괄적인 REST API를 정의합니다. 이 장에서는 엔드포인트, 요청/응답 형식, 인증 및 오류 처리를 포함한 완전한 API 사양을 제공합니다.

### API 설계 원칙

1. **RESTful**: 표준 HTTP 메서드 (GET, POST, PATCH, DELETE)
2. **JSON**: 모든 데이터는 JSON 형식
3. **버전 관리**: URL 기반 버전 관리 (/v1/)
4. **무상태성**: 각 요청에 필요한 모든 정보 포함
5. **HATEOAS**: 검색 가능성을 위한 하이퍼미디어 링크
6. **멱등성**: 요청 재시도 안전

---

## 5.2 기본 구성

### 5.2.1 기본 URL

```
프로덕션: https://api.wia-ind-009.com/v1
샌드박스: https://sandbox.wia-ind-009.com/v1
```

### 5.2.2 요청 헤더

```
필수 헤더:
  Authorization: Bearer <jwt_token>
  Content-Type: application/json
  Accept: application/json

선택적 헤더:
  X-Request-ID: <uuid>           (추적용)
  X-Client-Version: <version>    (앱 버전)
  Accept-Language: ko-KR         (현지화)
```

### 5.2.3 응답 형식

**성공 응답:**
```json
{
  "data": {
    ...응답 데이터...
  },
  "meta": {
    "requestId": "req_abc123",
    "timestamp": "2025-01-15T18:00:00Z",
    "version": "1.0.0"
  }
}
```

**오류 응답:**
```json
{
  "error": {
    "code": "INVALID_INPUT",
    "message": "배달 위치가 필요합니다",
    "details": {
      "field": "deliveryLocation",
      "constraint": "required"
    }
  },
  "meta": {
    "requestId": "req_abc123",
    "timestamp": "2025-01-15T18:00:00Z"
  }
}
```

---

## 5.3 인증

### 5.3.1 로그인

**엔드포인트:** `POST /auth/login`

**요청:**
```json
{
  "email": "customer@example.com",
  "password": "secure_password_123"
}
```

**응답:**
```json
{
  "data": {
    "token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
    "refreshToken": "refresh_abc123xyz",
    "expiresIn": 86400,
    "user": {
      "id": "user_123",
      "email": "customer@example.com",
      "role": "customer",
      "name": "Jane Doe"
    }
  }
}
```

### 5.3.2 토큰 갱신

**엔드포인트:** `POST /auth/refresh`

**요청:**
```json
{
  "refreshToken": "refresh_abc123xyz"
}
```

**응답:**
```json
{
  "data": {
    "token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
    "expiresIn": 86400
  }
}
```

### 5.3.3 JWT 구조

```json
{
  "sub": "user_123",
  "role": "customer",
  "permissions": [
    "order:create",
    "order:read",
    "order:update"
  ],
  "exp": 1737057600,
  "iat": 1736971200
}
```

**JWT 페이로드 설명:**
- `sub`: 사용자 고유 식별자
- `role`: 사용자 역할 (customer, driver, restaurant, admin)
- `permissions`: 세분화된 권한 목록
- `exp`: 만료 시간 (Unix 타임스탬프)
- `iat`: 발급 시간 (Unix 타임스탬프)

---

## 5.4 주문 엔드포인트

### 5.4.1 주문 생성

**엔드포인트:** `POST /orders`

**요청:**
```json
{
  "restaurantId": "rest_789",
  "items": [
    {
      "itemId": "item_pizza_001",
      "quantity": 1,
      "modifiers": [
        {"id": "mod_extra_cheese", "quantity": 1}
      ]
    }
  ],
  "deliveryLocation": {
    "latitude": 37.7858,
    "longitude": -122.4068,
    "address": "456 Mission St, Apt 12B, San Francisco, CA 94105",
    "addressLine2": "Apt 12B",
    "deliveryInstructions": "초인종을 눌러주세요"
  },
  "paymentMethodId": "pm_card_visa_1234",
  "contactlessDelivery": true,
  "specialInstructions": "냅킨 추가로 부탁합니다",
  "scheduledFor": "2025-01-15T19:00:00Z",
  "tip": 500
}
```

**응답:** `201 Created`
```json
{
  "data": {
    "id": "order_123",
    "confirmationCode": "ABC123",
    "status": "pending",
    "items": [...],
    "subtotal": 2598,
    "tax": 234,
    "deliveryFee": 499,
    "serviceFee": 390,
    "tip": 500,
    "total": 4221,
    "estimatedDelivery": "2025-01-15T19:35:00Z",
    "createdAt": "2025-01-15T18:00:00Z"
  },
  "links": {
    "self": "/orders/order_123",
    "tracking": "/orders/order_123/tracking",
    "cancel": "/orders/order_123"
  }
}
```

**필드 설명:**
- `confirmationCode`: 고객 참조용 6자리 확인 코드
- `subtotal`: 세금 및 수수료 제외 품목 총액 (센트)
- `tax`: 판매세 (센트)
- `deliveryFee`: 배달 수수료 (센트)
- `serviceFee`: 플랫폼 서비스 수수료 (센트)
- `tip`: 드라이버 팁 (센트)
- `total`: 최종 총액 (센트)

### 5.4.2 주문 조회

**엔드포인트:** `GET /orders/:orderId`

**응답:** `200 OK`
```json
{
  "data": {
    "id": "order_123",
    "confirmationCode": "ABC123",
    "status": "in_transit",
    "restaurant": {
      "id": "rest_789",
      "name": "Luigi's Pizza",
      "phone": "+14155551234",
      "address": "123 Market St, San Francisco, CA"
    },
    "driver": {
      "id": "drv_321",
      "name": "John D.",
      "phone": "+14155555678",
      "photo": "https://cdn.example.com/drivers/drv_321.jpg",
      "rating": 4.87,
      "vehicleType": "ebike"
    },
    "items": [...],
    "subtotal": 2598,
    "total": 4221,
    "estimatedDelivery": "2025-01-15T19:35:00Z",
    "actualDelivery": null,
    "statusHistory": [...]
  }
}
```

**상태 히스토리 형식:**
```json
"statusHistory": [
  {
    "status": "pending",
    "timestamp": "2025-01-15T18:00:00Z",
    "message": "주문이 접수되었습니다"
  },
  {
    "status": "confirmed",
    "timestamp": "2025-01-15T18:02:00Z",
    "message": "레스토랑에서 주문을 확인했습니다"
  },
  {
    "status": "preparing",
    "timestamp": "2025-01-15T18:05:00Z",
    "message": "음식을 준비 중입니다"
  }
]
```

### 5.4.3 주문 목록 조회

**엔드포인트:** `GET /orders`

**쿼리 매개변수:**
```
?status=in_transit          상태별 필터링
&restaurant=rest_789        레스토랑별 필터링
&customer=cust_456          고객별 필터링
&from=2025-01-01            날짜 범위 시작
&to=2025-01-31              날짜 범위 종료
&limit=20                   페이지당 결과 수 (기본값: 20, 최대: 100)
&offset=0                   페이지네이션 오프셋
&sort=-createdAt            정렬 필드 (- 는 내림차순)
```

**응답:** `200 OK`
```json
{
  "data": [
    {...주문 1...},
    {...주문 2...}
  ],
  "meta": {
    "total": 150,
    "limit": 20,
    "offset": 0,
    "hasMore": true
  },
  "links": {
    "self": "/orders?limit=20&offset=0",
    "next": "/orders?limit=20&offset=20",
    "prev": null
  }
}
```

**페이지네이션 예제:**
```
1페이지: GET /orders?limit=20&offset=0
2페이지: GET /orders?limit=20&offset=20
3페이지: GET /orders?limit=20&offset=40
```

### 5.4.4 주문 수정

**엔드포인트:** `PATCH /orders/:orderId`

**요청:**
```json
{
  "specialInstructions": "문 앞에 놓아주세요",
  "tip": 700
}
```

**응답:** `200 OK`
```json
{
  "data": {
    "id": "order_123",
    "specialInstructions": "문 앞에 놓아주세요",
    "tip": 700,
    "total": 4421,
    "updatedAt": "2025-01-15T18:15:00Z"
  }
}
```

**수정 가능한 필드 (상태에 따라):**
- `specialInstructions`: 언제든지 (delivered 전까지)
- `tip`: assigned 상태 전까지
- `deliveryLocation`: preparing 상태 전까지
- `contactlessDelivery`: in_transit 전까지

### 5.4.5 주문 취소

**엔드포인트:** `DELETE /orders/:orderId`

**요청:**
```json
{
  "reason": "마음이 바뀌었습니다",
  "cancelledBy": "customer"
}
```

**응답:** `200 OK`
```json
{
  "data": {
    "id": "order_123",
    "status": "cancelled",
    "cancelledAt": "2025-01-15T18:20:00Z",
    "refundAmount": 4221,
    "refundStatus": "pending"
  }
}
```

**취소 정책:**
- **pending/confirmed**: 전액 환불
- **preparing**: 75% 환불 (준비 시작 수수료)
- **ready/assigned**: 50% 환불 (음식 준비 완료)
- **picked_up 이후**: 환불 불가 (고객 서비스 검토 필요)

### 5.4.6 주문 추적 조회

**엔드포인트:** `GET /orders/:orderId/tracking`

**응답:** `200 OK`
```json
{
  "data": {
    "orderId": "order_123",
    "status": "in_transit",
    "driver": {
      "location": {
        "latitude": 37.7820,
        "longitude": -122.4100,
        "heading": 45,
        "speed": 22,
        "accuracy": 10,
        "timestamp": "2025-01-15T18:30:00Z"
      }
    },
    "route": {
      "currentStop": 2,
      "totalStops": 3,
      "distanceRemaining": 1.5,
      "estimatedArrival": "2025-01-15T18:40:00Z"
    },
    "temperature": {
      "current": 62.5,
      "target": 60,
      "inCompliance": true,
      "lastReading": "2025-01-15T18:30:00Z"
    }
  }
}
```

**위치 정보 설명:**
- `heading`: 진행 방향 (0-360도, 0=북쪽)
- `speed`: 속도 (km/h)
- `accuracy`: GPS 정확도 (미터)
- `distanceRemaining`: 남은 거리 (킬로미터)

### 5.4.7 온도 이력 조회

**엔드포인트:** `GET /orders/:orderId/temperature`

**응답:** `200 OK`
```json
{
  "data": {
    "orderId": "order_123",
    "readings": [
      {
        "timestamp": "2025-01-15T18:20:00Z",
        "temperature": 65.2,
        "humidity": 45,
        "inCompliance": true
      },
      {
        "timestamp": "2025-01-15T18:21:00Z",
        "temperature": 63.8,
        "humidity": 46,
        "inCompliance": true
      }
    ],
    "summary": {
      "avgTemperature": 63.5,
      "minTemperature": 60.1,
      "maxTemperature": 65.2,
      "complianceRate": 1.0,
      "alerts": []
    }
  }
}
```

**온도 요구사항:**
- **hot (뜨거운 음식)**: 60°C 이상
- **cold (차가운 음식)**: 4°C 이하
- **frozen (냉동 음식)**: -15°C 이하
- **ambient (상온 음식)**: 15-25°C

---

## 5.5 드라이버 엔드포인트

### 5.5.1 드라이버 등록

**엔드포인트:** `POST /drivers`

**요청:**
```json
{
  "firstName": "홍",
  "lastName": "길동",
  "email": "hong@example.com",
  "phone": "+821012345678",
  "dateOfBirth": "1990-05-15",
  "vehicleType": "ebike",
  "vehicleMake": "Rad Power",
  "vehicleModel": "RadRunner",
  "vehicleYear": 2024,
  "licensePlate": "서울12가3456",
  "licenseNumber": "12-345678-90",
  "licenseExpiration": "2028-05-15"
}
```

**응답:** `201 Created`
```json
{
  "data": {
    "id": "drv_321",
    "status": "pending_verification",
    "backgroundCheckStatus": "pending",
    "joinedAt": "2025-01-15T18:00:00Z"
  }
}
```

**등록 프로세스:**
1. 신청서 제출
2. 신원 조회 (2-5일)
3. 교육 이수 (4시간)
4. 장비 확인
5. 계정 활성화

### 5.5.2 드라이버 프로필 조회

**엔드포인트:** `GET /drivers/:driverId`

**응답:** `200 OK`
```json
{
  "data": {
    "id": "drv_321",
    "firstName": "홍",
    "lastName": "길동",
    "rating": 4.87,
    "totalDeliveries": 1523,
    "completionRate": 0.984,
    "onTimeRate": 0.923,
    "vehicleType": "ebike",
    "equipment": {
      "hasHotBag": true,
      "hasColdBag": true,
      "hasTemperatureSensor": true
    },
    "tier": "gold"
  }
}
```

**드라이버 등급 시스템:**
- **Bronze**: 0-99 배달
- **Silver**: 100-499 배달
- **Gold**: 500-1999 배달
- **Platinum**: 2000+ 배달, 평점 4.8+

### 5.5.3 드라이버 위치 업데이트

**엔드포인트:** `POST /drivers/:driverId/location`

**요청:**
```json
{
  "latitude": 37.7820,
  "longitude": -122.4100,
  "accuracy": 10,
  "heading": 45,
  "speed": 22,
  "timestamp": "2025-01-15T18:30:00Z"
}
```

**응답:** `200 OK`
```json
{
  "data": {
    "driverId": "drv_321",
    "location": {
      "latitude": 37.7820,
      "longitude": -122.4100,
      "timestamp": "2025-01-15T18:30:00Z"
    },
    "nearbyOrders": [
      {
        "orderId": "order_456",
        "distance": 0.8,
        "estimatedEarnings": 12.50
      }
    ]
  }
}
```

**위치 업데이트 빈도:**
- **온라인 대기 중**: 60초마다
- **픽업 중**: 30초마다
- **배달 중**: 10초마다

### 5.5.4 드라이버 지표 조회

**엔드포인트:** `GET /drivers/:driverId/metrics`

**쿼리 매개변수:**
```
?period=week                today|week|month|all
&from=2025-01-01
&to=2025-01-31
```

**응답:** `200 OK`
```json
{
  "data": {
    "period": {
      "start": "2025-01-08T00:00:00Z",
      "end": "2025-01-15T23:59:59Z"
    },
    "efficiency": {
      "ordersPerHour": 2.8,
      "avgDeliveryTime": 23,
      "avgDistancePerOrder": 3.2,
      "utilizationRate": 0.75
    },
    "quality": {
      "onTimeDeliveryRate": 0.923,
      "customerRating": 4.87,
      "orderAccuracy": 0.995,
      "temperatureCompliance": 0.978
    },
    "earnings": {
      "total": 85320,
      "deliveryFees": 65000,
      "tips": 18000,
      "bonuses": 2320,
      "avgPerHour": 24.50,
      "avgPerDelivery": 9.20
    }
  }
}
```

**지표 설명:**
- `ordersPerHour`: 시간당 완료 주문 수
- `avgDeliveryTime`: 평균 배달 시간 (분)
- `utilizationRate`: 활성 시간 대비 배달 시간 비율
- `onTimeDeliveryRate`: 정시 배달 비율
- `temperatureCompliance`: 온도 규정 준수율

---

## 5.6 경로 엔드포인트

### 5.6.1 단일 경로 계산

**엔드포인트:** `POST /routes/calculate`

**요청:**
```json
{
  "pickup": {
    "latitude": 37.7749,
    "longitude": -122.4194
  },
  "delivery": {
    "latitude": 37.7858,
    "longitude": -122.4068
  },
  "vehicleType": "ebike",
  "departureTime": "2025-01-15T18:30:00Z"
}
```

**응답:** `200 OK`
```json
{
  "data": {
    "distance": 2.3,
    "duration": 12,
    "eta": "2025-01-15T18:42:00Z",
    "waypoints": [
      {"latitude": 37.7749, "longitude": -122.4194},
      {"latitude": 37.7780, "longitude": -122.4150},
      {"latitude": 37.7858, "longitude": -122.4068}
    ],
    "encodedPolyline": "a~l~Fjk~uOwHJy@P",
    "instructions": [
      "Market St를 따라 북쪽으로 진행",
      "Mission St에서 우회전",
      "목적지 도착"
    ],
    "trafficConditions": "moderate"
  }
}
```

**교통 상황:**
- `light`: 원활 (기준 시간)
- `moderate`: 보통 (기준 시간 × 1.5)
- `heavy`: 정체 (기준 시간 × 2.0)
- `severe`: 심각한 정체 (기준 시간 × 3.0)

### 5.6.2 다중 정류장 경로 최적화

**엔드포인트:** `POST /routes/optimize`

**요청:**
```json
{
  "driverId": "drv_321",
  "orders": [
    {
      "orderId": "order_123",
      "pickup": {"latitude": 37.7749, "longitude": -122.4194},
      "delivery": {"latitude": 37.7858, "longitude": -122.4068},
      "deliveryWindow": {
        "start": "2025-01-15T18:30:00Z",
        "end": "2025-01-15T19:00:00Z"
      }
    },
    {
      "orderId": "order_124",
      "pickup": {"latitude": 37.7755, "longitude": -122.4185},
      "delivery": {"latitude": 37.7865, "longitude": -122.4055},
      "deliveryWindow": {
        "start": "2025-01-15T18:45:00Z",
        "end": "2025-01-15T19:15:00Z"
      }
    }
  ],
  "constraints": {
    "maxDuration": 60,
    "maxDistance": 15
  }
}
```

**응답:** `200 OK`
```json
{
  "data": {
    "routeId": "route_555",
    "stops": [
      {
        "sequence": 1,
        "type": "pickup",
        "orderId": "order_123",
        "location": {...},
        "estimatedArrival": "2025-01-15T18:35:00Z",
        "serviceDuration": 5
      },
      {
        "sequence": 2,
        "type": "pickup",
        "orderId": "order_124",
        "location": {...},
        "estimatedArrival": "2025-01-15T18:42:00Z",
        "serviceDuration": 5
      },
      {
        "sequence": 3,
        "type": "delivery",
        "orderId": "order_123",
        "location": {...},
        "estimatedArrival": "2025-01-15T18:55:00Z",
        "serviceDuration": 3
      },
      {
        "sequence": 4,
        "type": "delivery",
        "orderId": "order_124",
        "location": {...},
        "estimatedArrival": "2025-01-15T19:00:00Z",
        "serviceDuration": 3
      }
    ],
    "totalDistance": 4.8,
    "totalDuration": 25,
    "optimizationTime": 342,
    "algorithm": "2-opt",
    "allConstraintsMet": true
  }
}
```

**최적화 알고리즘:**
- **2-opt**: 소규모 (2-10개 정류장)
- **Genetic**: 중규모 (10-50개 정류장)
- **Cluster-first, route-second**: 대규모 (50+ 정류장)

---

## 5.7 WebSocket 실시간 추적

### 5.7.1 연결

**URL:** `wss://ws.wia-ind-009.com/tracking`

**인증:**
```javascript
const socket = new WebSocket('wss://ws.wia-ind-009.com/tracking');

socket.onopen = () => {
  socket.send(JSON.stringify({
    action: 'authenticate',
    token: 'Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...'
  }));
};

socket.onmessage = (event) => {
  const message = JSON.parse(event.data);
  console.log('수신:', message);
};

socket.onerror = (error) => {
  console.error('WebSocket 오류:', error);
};

socket.onclose = () => {
  console.log('WebSocket 연결 종료');
  // 재연결 로직
};
```

### 5.7.2 주문 업데이트 구독

**클라이언트 → 서버:**
```json
{
  "action": "subscribe",
  "orderId": "order_123",
  "userId": "user_456"
}
```

**서버 → 클라이언트 (확인):**
```json
{
  "event": "subscribed",
  "orderId": "order_123",
  "timestamp": "2025-01-15T18:30:00Z"
}
```

### 5.7.3 위치 업데이트

**서버 → 클라이언트 (10-30초마다):**
```json
{
  "event": "location_update",
  "orderId": "order_123",
  "timestamp": "2025-01-15T18:30:15Z",
  "data": {
    "driver": {
      "location": {
        "latitude": 37.7820,
        "longitude": -122.4100,
        "heading": 45,
        "speed": 22
      }
    },
    "eta": "2025-01-15T18:45:00Z",
    "distanceRemaining": 1.8,
    "status": "in_transit"
  }
}
```

### 5.7.4 상태 업데이트

**서버 → 클라이언트:**
```json
{
  "event": "status_changed",
  "orderId": "order_123",
  "timestamp": "2025-01-15T18:35:00Z",
  "data": {
    "oldStatus": "assigned",
    "newStatus": "picked_up",
    "message": "드라이버가 주문을 픽업했습니다"
  }
}
```

### 5.7.5 온도 경고

**서버 → 클라이언트:**
```json
{
  "event": "temperature_alert",
  "orderId": "order_123",
  "timestamp": "2025-01-15T18:32:00Z",
  "data": {
    "level": "warning",
    "temperature": 57.5,
    "threshold": 60,
    "message": "음식 온도가 하한선에 접근하고 있습니다"
  }
}
```

**경고 레벨:**
- `info`: 정보 (정상 범위)
- `warning`: 주의 (임계값 접근)
- `critical`: 위험 (임계값 초과)
- `severe`: 심각 (장기간 위반)

### 5.7.6 ETA 업데이트

**서버 → 클라이언트 (배송 중 2분마다):**
```json
{
  "event": "eta_updated",
  "orderId": "order_123",
  "timestamp": "2025-01-15T18:36:00Z",
  "data": {
    "previousEta": "2025-01-15T18:45:00Z",
    "currentEta": "2025-01-15T18:42:00Z",
    "reason": "traffic_improved",
    "confidence": 0.95
  }
}
```

**ETA 변경 사유:**
- `traffic_improved`: 교통 상황 개선
- `traffic_worsened`: 교통 정체
- `route_changed`: 경로 변경
- `driver_delayed`: 드라이버 지연

---

## 5.8 웹훅 알림

### 5.8.1 웹훅 등록

**엔드포인트:** `POST /webhooks`

**요청:**
```json
{
  "url": "https://your-server.com/webhooks/wia-ind-009",
  "events": [
    "order.created",
    "order.confirmed",
    "order.delivered",
    "order.cancelled",
    "order.temperature_alert"
  ],
  "secret": "your_webhook_secret_key"
}
```

**응답:** `201 Created`
```json
{
  "data": {
    "webhookId": "webhook_999",
    "url": "https://your-server.com/webhooks/wia-ind-009",
    "events": [...],
    "status": "active",
    "createdAt": "2025-01-15T18:00:00Z"
  }
}
```

**사용 가능한 이벤트:**
- `order.created`: 주문 생성
- `order.confirmed`: 레스토랑 확인
- `order.preparing`: 음식 준비 중
- `order.ready`: 픽업 준비 완료
- `order.assigned`: 드라이버 배정
- `order.picked_up`: 픽업 완료
- `order.in_transit`: 배달 중
- `order.delivered`: 배달 완료
- `order.cancelled`: 주문 취소
- `order.temperature_alert`: 온도 경고

### 5.8.2 웹훅 페이로드

**서버에서 POST 요청:**
```json
{
  "webhookId": "webhook_999",
  "event": "order.delivered",
  "timestamp": "2025-01-15T19:00:00Z",
  "data": {
    "orderId": "order_123",
    "status": "delivered",
    "deliveryTime": "2025-01-15T19:00:00Z",
    "proofOfDelivery": {
      "photo": "https://cdn.example.com/proof_123.jpg",
      "signature": null,
      "location": {
        "latitude": 37.7858,
        "longitude": -122.4068
      }
    }
  },
  "signature": "sha256=5d41402abc4b2a76b9719d911017c592"
}
```

**헤더:**
```
X-WIA-Signature: sha256=5d41402abc4b2a76b9719d911017c592
X-WIA-Event: order.delivered
X-WIA-Timestamp: 1737057600
```

### 5.8.3 웹훅 검증

**Python 예제:**
```python
import hmac
import hashlib

def verify_webhook(payload, signature, secret):
    """웹훅 서명 검증"""

    expected_signature = hmac.new(
        secret.encode(),
        payload.encode(),
        hashlib.sha256
    ).hexdigest()

    return hmac.compare_digest(
        f"sha256={expected_signature}",
        signature
    )

# 사용 예제
@app.route('/webhooks/wia-ind-009', methods=['POST'])
def handle_webhook():
    payload = request.get_data(as_text=True)
    signature = request.headers.get('X-WIA-Signature')

    if not verify_webhook(payload, signature, WEBHOOK_SECRET):
        return jsonify({'error': '유효하지 않은 서명'}), 401

    data = json.loads(payload)
    # 이벤트 처리

    return jsonify({'success': True}), 200
```

**Node.js 예제:**
```javascript
const crypto = require('crypto');

function verifyWebhook(payload, signature, secret) {
  const expectedSignature = crypto
    .createHmac('sha256', secret)
    .update(payload)
    .digest('hex');

  return crypto.timingSafeEqual(
    Buffer.from(`sha256=${expectedSignature}`),
    Buffer.from(signature)
  );
}

app.post('/webhooks/wia-ind-009', (req, res) => {
  const payload = JSON.stringify(req.body);
  const signature = req.headers['x-wia-signature'];

  if (!verifyWebhook(payload, signature, WEBHOOK_SECRET)) {
    return res.status(401).json({ error: '유효하지 않은 서명' });
  }

  // 이벤트 처리

  res.json({ success: true });
});
```

---

## 5.9 오류 처리

### 5.9.1 HTTP 상태 코드

```
200 OK                  성공
201 Created             리소스 생성됨
204 No Content          응답 본문 없이 성공
400 Bad Request         잘못된 입력
401 Unauthorized        인증 누락 또는 유효하지 않음
403 Forbidden           유효한 인증이지만 권한 부족
404 Not Found           리소스 존재하지 않음
409 Conflict            리소스가 이미 존재하거나 상태 충돌
422 Unprocessable       유효한 입력이지만 비즈니스 로직 실패
429 Too Many Requests   속도 제한 초과
500 Internal Error      서버 오류
503 Service Unavailable 일시적 장애
```

### 5.9.2 오류 코드

```typescript
enum ErrorCode {
  // 인증
  INVALID_TOKEN = 'INVALID_TOKEN',
  EXPIRED_TOKEN = 'EXPIRED_TOKEN',
  INSUFFICIENT_PERMISSIONS = 'INSUFFICIENT_PERMISSIONS',

  // 검증
  INVALID_INPUT = 'INVALID_INPUT',
  MISSING_FIELD = 'MISSING_FIELD',
  INVALID_FORMAT = 'INVALID_FORMAT',
  OUT_OF_RANGE = 'OUT_OF_RANGE',

  // 비즈니스 로직
  RESTAURANT_UNAVAILABLE = 'RESTAURANT_UNAVAILABLE',
  OUT_OF_DELIVERY_RANGE = 'OUT_OF_DELIVERY_RANGE',
  NO_DRIVERS_AVAILABLE = 'NO_DRIVERS_AVAILABLE',
  PAYMENT_FAILED = 'PAYMENT_FAILED',

  // 리소스
  ORDER_NOT_FOUND = 'ORDER_NOT_FOUND',
  DRIVER_NOT_FOUND = 'DRIVER_NOT_FOUND',
  CANNOT_CANCEL = 'CANNOT_CANCEL',

  // 시스템
  INTERNAL_ERROR = 'INTERNAL_ERROR',
  SERVICE_UNAVAILABLE = 'SERVICE_UNAVAILABLE',
  RATE_LIMIT_EXCEEDED = 'RATE_LIMIT_EXCEEDED'
}
```

### 5.9.3 오류 응답 예제

```json
{
  "error": {
    "code": "OUT_OF_DELIVERY_RANGE",
    "message": "배달 주소가 서비스 지역 밖에 있습니다",
    "details": {
      "address": "456 Mission St, San Francisco, CA",
      "distance": 25.5,
      "maxDistance": 15,
      "unit": "km"
    },
    "hint": "더 가까운 레스토랑을 시도하거나 고객 지원에 문의하세요"
  },
  "meta": {
    "requestId": "req_abc123",
    "timestamp": "2025-01-15T18:00:00Z",
    "documentation": "https://docs.wia-ind-009.com/errors/OUT_OF_DELIVERY_RANGE"
  }
}
```

**오류 처리 모범 사례:**

1. **클라이언트 측:**
```typescript
async function createOrder(orderData: OrderRequest) {
  try {
    const response = await fetch('/orders', {
      method: 'POST',
      headers: {
        'Authorization': `Bearer ${token}`,
        'Content-Type': 'application/json'
      },
      body: JSON.stringify(orderData)
    });

    if (!response.ok) {
      const error = await response.json();

      // 특정 오류 처리
      switch (error.error.code) {
        case 'OUT_OF_DELIVERY_RANGE':
          showError('배달 불가 지역입니다');
          break;
        case 'PAYMENT_FAILED':
          showError('결제에 실패했습니다');
          break;
        default:
          showError('주문 생성 실패');
      }

      throw new Error(error.error.message);
    }

    return await response.json();
  } catch (error) {
    console.error('주문 오류:', error);
    throw error;
  }
}
```

---

## 5.10 속도 제한

### 5.10.1 속도 제한

```
등급              요청/분    버스트    WebSocket 연결
─────────────────────────────────────────────────────
Standard          100       200       10
Premium           1,000     2,000     50
Enterprise        10,000    20,000    500
```

**등급별 설명:**
- **Standard**: 소규모 운영 (1-5개 레스토랑)
- **Premium**: 중규모 운영 (5-50개 레스토랑)
- **Enterprise**: 대규모 운영 (50+ 레스토랑)

### 5.10.2 속도 제한 헤더

**응답 헤더:**
```
X-RateLimit-Limit: 100
X-RateLimit-Remaining: 87
X-RateLimit-Reset: 1737057600
Retry-After: 60
```

**헤더 설명:**
- `X-RateLimit-Limit`: 분당 최대 요청 수
- `X-RateLimit-Remaining`: 남은 요청 수
- `X-RateLimit-Reset`: 제한 재설정 시간 (Unix 타임스탬프)
- `Retry-After`: 재시도까지 대기 시간 (초)

### 5.10.3 속도 제한 초과 응답

**HTTP 429:**
```json
{
  "error": {
    "code": "RATE_LIMIT_EXCEEDED",
    "message": "너무 많은 요청입니다. 60초 후에 다시 시도하세요.",
    "details": {
      "limit": 100,
      "window": "minute",
      "retryAfter": 60
    }
  }
}
```

**속도 제한 처리 예제:**
```typescript
async function makeApiRequest(url: string, options: RequestInit) {
  const response = await fetch(url, options);

  if (response.status === 429) {
    const retryAfter = parseInt(
      response.headers.get('Retry-After') || '60'
    );

    console.log(`속도 제한 초과. ${retryAfter}초 후 재시도...`);

    await new Promise(resolve =>
      setTimeout(resolve, retryAfter * 1000)
    );

    // 재시도
    return makeApiRequest(url, options);
  }

  return response;
}
```

---

## 5.11 요약

WIA-IND-009 API는 다음을 제공합니다:

- **RESTful 엔드포인트**: 주문, 드라이버, 경로에 대한 완전한 CRUD 작업
- **WebSocket**: 초 단위 업데이트의 실시간 추적
- **웹훅**: 이벤트 기반 알림
- **인증**: JWT 기반 보안 액세스
- **속도 제한**: 버스트 허용이 있는 계층별 제한
- **오류 처리**: 포괄적인 오류 코드 및 메시지

**API 통합 체크리스트:**
- [ ] API 키 및 인증 설정
- [ ] 오류 처리 구현
- [ ] 속도 제한 관리
- [ ] WebSocket 재연결 로직
- [ ] 웹훅 서명 검증
- [ ] 로깅 및 모니터링
- [ ] 테스트 환경 설정
- [ ] 프로덕션 배포

---

## 복습 문제

1. **인증**: JWT 토큰의 주요 구성 요소는 무엇이며, 각각의 역할은 무엇인가요?

2. **주문 생성**: 새 주문을 생성할 때 필수 필드는 무엇인가요? 각 필드의 유효성 검사 규칙은?

3. **실시간 추적**: WebSocket과 REST API의 차이점은 무엇이며, 각각 언제 사용해야 하나요?

4. **웹훅**: 웹훅 페이로드의 서명을 검증하는 이유는 무엇이며, 어떻게 구현하나요?

5. **오류 처리**: `422 Unprocessable Entity`와 `400 Bad Request`의 차이점은 무엇인가요?

6. **속도 제한**: 속도 제한에 도달했을 때 클라이언트는 어떻게 대응해야 하나요?

7. **경로 최적화**: 다중 정류장 경로 최적화가 필요한 경우는 언제이며, 어떤 제약 조건을 고려해야 하나요?

8. **온도 모니터링**: 온도 이력 API를 사용하여 식품 안전 규정 준수를 어떻게 검증하나요?

---

**다음 장**: [제6장: 프로토콜 및 알고리즘 →](06-protocol.md)

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
