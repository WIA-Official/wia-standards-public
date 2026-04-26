# 4장: 데이터 형식 및 모델

---

## 학습 목표

이 장을 마치면 다음을 수행할 수 있습니다:

- WIA-IND-009 표준에서 사용되는 모든 핵심 데이터 구조를 이해하고 구현합니다
- 주문, 배달원, 경로, 온도 읽기 등의 완전한 엔티티 모델을 설계합니다
- JSON 및 TypeScript 인터페이스를 사용하여 타입 안전성을 보장합니다
- ISO 8601, E.164 등 표준 형식을 적용합니다
- 데이터 무결성을 위한 검증 규칙을 구현합니다
- 향후 확장을 위한 메타데이터 패턴을 활용합니다

---

## 4.1 개요

이 장은 WIA-IND-009 표준에서 사용되는 모든 데이터 구조에 대한 상세한 사양을 제공합니다. 모든 데이터는 JSON 형식으로 교환되며 구현 가이드를 위한 강력한 타입의 TypeScript 인터페이스가 제공됩니다.

### 설계 원칙

1. **명확성**: 필드 이름은 자명함
2. **일관성**: 모든 엔티티에 걸쳐 동일한 패턴
3. **확장성**: 향후 추가를 위한 메타데이터 필드
4. **검증**: 명확한 제약 조건 및 형식
5. **상호 운용성**: 표준 형식 (ISO 8601, E.164 등)

---

## 4.2 주문 엔티티

### 4.2.1 완전한 주문 구조

```typescript
interface Order {
  // ========== 식별 ==========
  id: string;                       // UUID v4
  externalId?: string;              // 파트너 시스템 참조
  confirmationCode: string;         // 사람이 읽을 수 있는 (예: "ABC123")

  // ========== 당사자 ==========
  restaurantId: string;             // UUID
  customerId: string;               // UUID
  driverId?: string;                // UUID (나중에 배정됨)

  // ========== 품목 ==========
  items: OrderItem[];
  subtotal: number;                 // USD 센트 (2500 = $25.00)
  tax: number;                      // USD 센트
  deliveryFee: number;              // USD 센트
  serviceFee: number;               // USD 센트
  smallOrderFee: number;            // USD 센트
  discount: number;                 // USD 센트 (음수)
  tip: number;                      // USD 센트
  total: number;                    // USD 센트

  // ========== 위치 ==========
  pickupLocation: Location;
  deliveryLocation: Location;

  // ========== 타이밍 ==========
  createdAt: Date;                  // ISO 8601
  confirmedAt?: Date;
  preparedAt?: Date;
  readyAt?: Date;
  assignedAt?: Date;
  pickedUpAt?: Date;
  deliveredAt?: Date;
  estimatedDelivery: Date;          // 고객에게 약속
  actualDelivery?: Date;
  scheduledFor?: Date;              // 예약 주문용
  deliveryWindow?: TimeWindow;

  // ========== 상태 ==========
  status: OrderStatus;
  statusHistory: StatusChange[];

  // ========== 물류 ==========
  route?: Route;
  distance: number;                 // 킬로미터
  duration: number;                 // 분 (추정)
  actualDuration?: number;          // 분 (실제)
  temperatureRequirement: TemperatureType;
  vehicleType?: VehicleType;

  // ========== 선호 사항 ==========
  contactlessDelivery: boolean;
  utensils: boolean;
  specialInstructions?: string;     // 최대 500자
  deliveryInstructions?: string;    // 주소 관련

  // ========== 품질 ==========
  temperatureLogs?: TemperatureReading[];
  photos?: OrderPhoto[];
  rating?: OrderRating;
  issues?: OrderIssue[];

  // ========== 준수 ==========
  cancelledBy?: 'customer' | 'restaurant' | 'driver' | 'system';
  cancellationReason?: string;
  refundAmount?: number;            // USD 센트
  refundReason?: string;

  // ========== 메타데이터 ==========
  metadata?: Record<string, any>;   // 확장성
  version: number;                  // 낙관적 잠금용
}
```

### 4.2.2 주문 품목

```typescript
interface OrderItem {
  id: string;                       // UUID
  externalId?: string;              // 메뉴 항목 ID
  name: string;
  description?: string;
  quantity: number;                 // > 0이어야 함
  unitPrice: number;                // USD 센트
  totalPrice: number;               // quantity × unitPrice
  temperature: TemperatureType;

  // 사용자 지정
  modifiers?: ItemModifier[];
  specialRequests?: string;
  allergens?: string[];

  // 메타데이터
  sku?: string;
  category?: string;
  image?: string;                   // URL
}

interface ItemModifier {
  id: string;
  name: string;                     // "치즈 추가", "양파 빼기"
  price: number;                    // USD 센트 (0 가능)
  quantity: number;                 // 보통 1
}

type TemperatureType = 'hot' | 'cold' | 'ambient' | 'frozen';
```

### 4.2.3 위치

```typescript
interface Location {
  // 좌표
  latitude: number;                 // -90 ~ 90
  longitude: number;                // -180 ~ 180
  accuracy?: number;                // 미터

  // 주소
  address: string;                  // 완전한 형식의 주소
  addressLine1?: string;            // 도로명 주소
  addressLine2?: string;            // 아파트/호수
  city: string;
  state?: string;                   // 주/도
  postalCode?: string;
  country: string;                  // ISO 3166-1 alpha-2 (US, CA 등)

  // 세부 정보
  locationType?: 'restaurant' | 'residence' | 'office' | 'hotel' | 'other';
  parkingNotes?: string;
  accessCode?: string;              // 건물 출입 코드
  buzzerCode?: string;

  // 검증
  verified: boolean;                // 지오코딩 성공
  placeId?: string;                 // Google Places ID
}
```

### 4.2.4 주문 상태

```typescript
type OrderStatus =
  | 'pending'         // 주문 생성됨, 확인 대기 중
  | 'confirmed'       // 결제 승인됨, 음식점에 알림
  | 'preparing'       // 음식점이 음식 준비 중
  | 'ready'           // 음식이 픽업 준비됨
  | 'assigned'        // 배달원 배정됨
  | 'picked_up'       // 배달원이 음식 수령함
  | 'in_transit'      // 배달원이 고객으로 이동 중
  | 'arriving'        // 배달원이 2분 이내 도착
  | 'delivered'       // 음식 배달됨 (확인 대기)
  | 'completed'       // 주문 완료, 결제 캡처됨
  | 'cancelled'       // 주문 취소됨
  | 'failed';         // 배달 실패

interface StatusChange {
  from: OrderStatus;
  to: OrderStatus;
  timestamp: Date;
  actor: string;                    // 사용자/배달원/시스템 ID
  reason?: string;
  location?: Location;
}
```

### 4.2.5 주문 JSON 예시

```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "confirmationCode": "ABC123",
  "restaurantId": "rest_789",
  "customerId": "cust_456",
  "driverId": "drv_321",

  "items": [
    {
      "id": "item_1",
      "name": "Margherita Pizza",
      "quantity": 1,
      "unitPrice": 1499,
      "totalPrice": 1499,
      "temperature": "hot",
      "modifiers": [
        {
          "id": "mod_1",
          "name": "Extra Cheese",
          "price": 200,
          "quantity": 1
        }
      ]
    },
    {
      "id": "item_2",
      "name": "Caesar Salad",
      "quantity": 1,
      "unitPrice": 899,
      "totalPrice": 899,
      "temperature": "cold"
    }
  ],

  "subtotal": 2598,
  "tax": 234,
  "deliveryFee": 499,
  "serviceFee": 390,
  "smallOrderFee": 0,
  "discount": -300,
  "tip": 500,
  "total": 3921,

  "pickupLocation": {
    "latitude": 37.7749,
    "longitude": -122.4194,
    "address": "123 Market St, San Francisco, CA 94103",
    "city": "San Francisco",
    "state": "CA",
    "postalCode": "94103",
    "country": "US",
    "locationType": "restaurant",
    "verified": true
  },

  "deliveryLocation": {
    "latitude": 37.7858,
    "longitude": -122.4068,
    "address": "456 Mission St, Apt 12B, San Francisco, CA 94105",
    "addressLine1": "456 Mission St",
    "addressLine2": "Apt 12B",
    "city": "San Francisco",
    "state": "CA",
    "postalCode": "94105",
    "country": "US",
    "locationType": "residence",
    "buzzerCode": "12B",
    "verified": true
  },

  "createdAt": "2025-01-15T18:00:00Z",
  "confirmedAt": "2025-01-15T18:00:15Z",
  "estimatedDelivery": "2025-01-15T18:45:00Z",

  "status": "in_transit",
  "statusHistory": [
    {
      "from": "pending",
      "to": "confirmed",
      "timestamp": "2025-01-15T18:00:15Z",
      "actor": "system"
    },
    {
      "from": "confirmed",
      "to": "preparing",
      "timestamp": "2025-01-15T18:05:00Z",
      "actor": "rest_789"
    }
  ],

  "distance": 2.3,
  "duration": 25,
  "temperatureRequirement": "hot",
  "contactlessDelivery": true,
  "utensils": false,
  "specialInstructions": "Please ring doorbell",

  "version": 5
}
```

---

## 4.3 배달원 엔티티

### 4.3.1 완전한 배달원 구조

```typescript
interface Driver {
  // ========== 식별 ==========
  id: string;                       // UUID
  externalId?: string;

  // ========== 개인 정보 ==========
  firstName: string;
  lastName: string;
  email: string;                    // 고유
  phone: string;                    // E.164 형식: +14155551234
  photo?: string;                   // URL
  dateOfBirth: Date;

  // ========== 차량 ==========
  vehicleType: VehicleType;
  vehicleMake?: string;             // "Honda"
  vehicleModel?: string;            // "Civic"
  vehicleYear?: number;             // 2020
  vehicleColor?: string;            // "Silver"
  licensePlate?: string;            // "ABC1234"

  // ========== 상태 ==========
  status: DriverStatus;
  isOnline: boolean;
  isAvailable: boolean;             // 온라인이지만 활성 배달 중이 아님

  // ========== 위치 ==========
  location: Location;
  heading?: number;                 // 도 0-359 (0 = 북쪽)
  speed?: number;                   // km/h
  lastLocationUpdate: Date;

  // ========== 성과 ==========
  rating: number;                   // 0-5 (예: 4.87)
  totalDeliveries: number;
  completionRate: number;           // 0-1 (0.98 = 98%)
  onTimeRate: number;               // 0-1
  cancellationRate: number;         // 0-1
  acceptanceRate: number;           // 0-1
  responseTime: number;             // 주문 수락까지 평균 초

  // ========== 장비 ==========
  equipment: DriverEquipment;

  // ========== 용량 ==========
  maxOrders: number;                // 동시 주문
  currentOrders: string[];          // 주문 ID
  maxWeight: number;                // 킬로그램
  maxVolume: number;                // 리터

  // ========== 수입 ==========
  earnings: DriverEarnings;

  // ========== 검증 ==========
  backgroundCheckStatus: 'pending' | 'approved' | 'rejected' | 'expired';
  backgroundCheckDate?: Date;
  licenseVerified: boolean;
  licenseNumber?: string;
  licenseExpiration?: Date;
  insuranceVerified: boolean;
  insurancePolicyNumber?: string;
  insuranceExpiration?: Date;
  foodSafetyCertified: boolean;
  foodSafetyCertDate?: Date;

  // ========== 계정 ==========
  accountStatus: 'active' | 'inactive' | 'suspended' | 'deactivated';
  suspensionReason?: string;
  tier: 'bronze' | 'silver' | 'gold' | 'platinum';
  joinedAt: Date;
  lastActiveAt: Date;

  // ========== 선호 사항 ==========
  preferredZones?: string[];        // 구역 ID
  maxDistance: number;              // km (이동할 의향이 있는)
  workingHours?: TimeWindow[];

  // ========== 메타데이터 ==========
  metadata?: Record<string, any>;
  version: number;
}

type VehicleType =
  | 'bike'           // 자전거
  | 'ebike'          // 전기 자전거
  | 'scooter'        // 모터 스쿠터
  | 'motorcycle'     // 오토바이
  | 'car';           // 자동차

type DriverStatus =
  | 'offline'        // 근무하지 않음
  | 'online'         // 주문 가능
  | 'assigned'       // 주문 배정됨, 픽업으로 이동 중
  | 'picking_up'     // 음식점에 있음
  | 'in_transit'     // 배달 중
  | 'delivering'     // 고객 위치에 있음
  | 'break';         // 휴식 중

interface DriverEquipment {
  hasHotBag: boolean;
  hasColdBag: boolean;
  hasFrozenStorage: boolean;
  hasTemperatureSensor: boolean;
  hasInsulatedContainer: boolean;
  hasSmartphone: boolean;
  smartphoneModel?: string;

  // 가방 사양
  hotBagCapacity?: number;          // 리터
  coldBagCapacity?: number;         // 리터
}

interface DriverEarnings {
  today: number;                    // USD 센트
  week: number;
  month: number;
  allTime: number;

  // 분석
  deliveryFees: number;             // 기본 + 거리 + 시간
  tips: number;
  bonuses: number;                  // 피크 시간, 퀘스트 등

  // 공제
  fees: number;                     // 플랫폼 수수료
  adjustments: number;              // 수정, 벌금
}
```

### 4.3.2 배달원 지표

```typescript
interface DriverMetrics {
  // 효율성
  ordersPerHour: number;            // 평균
  avgDeliveryTime: number;          // 분
  avgDistancePerOrder: number;      // 킬로미터
  utilizationRate: number;          // 활성 시간 / 온라인 시간

  // 품질
  onTimeDeliveryRate: number;       // 0-1
  customerRating: number;           // 0-5
  orderAccuracy: number;            // 0-1
  temperatureCompliance: number;    // 0-1

  // 신뢰성
  completionRate: number;           // 0-1
  cancellationRate: number;         // 0-1
  acceptanceRate: number;           // 0-1
  responseTime: number;             // 초

  // 수입
  totalEarnings: number;            // USD 센트
  avgEarningsPerHour: number;
  avgEarningsPerDelivery: number;

  // 기간
  period: TimePeriod;
}

interface TimePeriod {
  start: Date;
  end: Date;
}
```

### 4.3.3 배달원 JSON 예시

```json
{
  "id": "drv_321",
  "firstName": "John",
  "lastName": "Doe",
  "email": "john.doe@example.com",
  "phone": "+14155551234",
  "photo": "https://cdn.example.com/drivers/drv_321.jpg",

  "vehicleType": "ebike",
  "vehicleMake": "Rad Power",
  "vehicleModel": "RadRunner",
  "vehicleYear": 2024,
  "vehicleColor": "Black",

  "status": "in_transit",
  "isOnline": true,
  "isAvailable": false,

  "location": {
    "latitude": 37.7800,
    "longitude": -122.4150,
    "accuracy": 10,
    "address": "789 Howard St, San Francisco, CA 94103",
    "city": "San Francisco",
    "state": "CA",
    "country": "US",
    "verified": true
  },
  "heading": 45,
  "speed": 22,
  "lastLocationUpdate": "2025-01-15T18:30:00Z",

  "rating": 4.87,
  "totalDeliveries": 1523,
  "completionRate": 0.984,
  "onTimeRate": 0.923,
  "cancellationRate": 0.016,
  "acceptanceRate": 0.89,
  "responseTime": 12,

  "equipment": {
    "hasHotBag": true,
    "hasColdBag": true,
    "hasFrozenStorage": false,
    "hasTemperatureSensor": true,
    "hasInsulatedContainer": true,
    "hasSmartphone": true,
    "smartphoneModel": "iPhone 14",
    "hotBagCapacity": 40,
    "coldBagCapacity": 30
  },

  "maxOrders": 3,
  "currentOrders": ["order_123"],
  "maxWeight": 15,
  "maxVolume": 60,

  "earnings": {
    "today": 12450,
    "week": 85320,
    "month": 342180,
    "allTime": 4567890,
    "deliveryFees": 3200000,
    "tips": 1200000,
    "bonuses": 180000,
    "fees": 12110,
    "adjustments": 0
  },

  "backgroundCheckStatus": "approved",
  "backgroundCheckDate": "2024-06-15T00:00:00Z",
  "licenseVerified": true,
  "insuranceVerified": true,
  "foodSafetyCertified": true,
  "foodSafetyCertDate": "2024-07-01T00:00:00Z",

  "accountStatus": "active",
  "tier": "gold",
  "joinedAt": "2023-03-10T00:00:00Z",
  "lastActiveAt": "2025-01-15T18:30:00Z",

  "maxDistance": 15,

  "version": 42
}
```

---

## 4.4 경로 엔티티

### 4.4.1 경로 구조

```typescript
interface Route {
  id: string;                       // UUID
  driverId: string;
  orders: string[];                 // 주문 ID

  // 경로 세부 정보
  stops: RouteStop[];
  totalDistance: number;            // 킬로미터
  totalDuration: number;            // 분
  optimizationAlgorithm: string;    // "2-opt", "nearest-neighbor"
  optimizationTime: number;         // 계산에 걸린 밀리초

  // 경유지 (상세 경로)
  waypoints: GeoPoint[];
  encodedPolyline?: string;         // Google polyline 인코딩

  // 성능
  estimatedCost: number;            // USD 센트 (연료, 시간)
  fuelConsumption?: number;         // 리터
  co2Emissions?: number;            // 킬로그램

  // 상태
  status: 'planned' | 'active' | 'completed' | 'cancelled';
  startedAt?: Date;
  completedAt?: Date;

  // 편차
  deviations: RouteDeviation[];
  reroutes: number;                 // 동적 재경로 수

  // 메타데이터
  metadata?: Record<string, any>;
}

interface RouteStop {
  sequence: number;                 // 1, 2, 3, ...
  type: 'pickup' | 'delivery';
  orderId: string;
  location: Location;

  // 타이밍
  estimatedArrival: Date;
  actualArrival?: Date;
  estimatedDeparture: Date;
  actualDeparture?: Date;
  serviceDuration: number;          // 분

  // 상태
  completed: boolean;
  skipped: boolean;
  notes?: string;
}

interface GeoPoint {
  latitude: number;
  longitude: number;
  timestamp?: Date;
}

interface RouteDeviation {
  timestamp: Date;
  location: Location;
  reason: string;                   // "traffic", "driver_deviation", "customer_unavailable"
  distanceOff: number;              // 계획된 경로에서 미터
  timeImpact: number;               // 추가된 분
}
```

### 4.4.2 경로 JSON 예시

```json
{
  "id": "route_555",
  "driverId": "drv_321",
  "orders": ["order_123", "order_124"],

  "stops": [
    {
      "sequence": 1,
      "type": "pickup",
      "orderId": "order_123",
      "location": {
        "latitude": 37.7749,
        "longitude": -122.4194,
        "address": "123 Market St, San Francisco, CA"
      },
      "estimatedArrival": "2025-01-15T18:10:00Z",
      "actualArrival": "2025-01-15T18:11:30Z",
      "estimatedDeparture": "2025-01-15T18:15:00Z",
      "actualDeparture": "2025-01-15T18:14:00Z",
      "serviceDuration": 5,
      "completed": true
    },
    {
      "sequence": 2,
      "type": "pickup",
      "orderId": "order_124",
      "location": {
        "latitude": 37.7755,
        "longitude": -122.4185,
        "address": "150 Market St, San Francisco, CA"
      },
      "estimatedArrival": "2025-01-15T18:17:00Z",
      "serviceDuration": 5,
      "completed": false
    },
    {
      "sequence": 3,
      "type": "delivery",
      "orderId": "order_123",
      "location": {
        "latitude": 37.7858,
        "longitude": -122.4068,
        "address": "456 Mission St, San Francisco, CA"
      },
      "estimatedArrival": "2025-01-15T18:30:00Z",
      "serviceDuration": 3,
      "completed": false
    },
    {
      "sequence": 4,
      "type": "delivery",
      "orderId": "order_124",
      "location": {
        "latitude": 37.7865,
        "longitude": -122.4055,
        "address": "500 Mission St, San Francisco, CA"
      },
      "estimatedArrival": "2025-01-15T18:35:00Z",
      "serviceDuration": 3,
      "completed": false
    }
  ],

  "totalDistance": 4.8,
  "totalDuration": 25,
  "optimizationAlgorithm": "2-opt",
  "optimizationTime": 342,

  "waypoints": [
    {"latitude": 37.7749, "longitude": -122.4194},
    {"latitude": 37.7755, "longitude": -122.4192},
    {"latitude": 37.7760, "longitude": -122.4180}
  ],

  "estimatedCost": 850,
  "fuelConsumption": 0.3,
  "co2Emissions": 0.8,

  "status": "active",
  "startedAt": "2025-01-15T18:10:00Z",

  "deviations": [],
  "reroutes": 0
}
```

---

## 4.5 온도 모니터링

### 4.5.1 온도 읽기

```typescript
interface TemperatureReading {
  id: string;                       // UUID
  timestamp: Date;                  // ISO 8601

  // 연결
  orderId: string;
  driverId: string;
  sensorId: string;

  // 측정
  temperature: number;              // 섭씨
  humidity?: number;                // 백분율 0-100
  batteryLevel: number;             // 백분율 0-100

  // 위치
  location: Location;

  // 상태
  inCompliance: boolean;
  alertLevel?: 'warning' | 'critical' | 'severe';

  // 메타데이터
  sensorType: 'bluetooth' | 'wifi' | 'cellular';
  accuracy: number;                 // ±도
}

interface TemperatureAlert {
  id: string;
  timestamp: Date;
  orderId: string;
  driverId: string;
  level: 'warning' | 'critical' | 'severe';
  temperature: number;              // 섭씨
  threshold: number;                // 섭씨
  duration: number;                 // 준수하지 않은 초
  action: string;                   // 취한 조치
  resolved: boolean;
  resolvedAt?: Date;
}
```

### 4.5.2 온도 읽기 JSON 예시

```json
{
  "id": "temp_999",
  "timestamp": "2025-01-15T18:25:00Z",
  "orderId": "order_123",
  "driverId": "drv_321",
  "sensorId": "sensor_ABC123",

  "temperature": 62.5,
  "humidity": 45,
  "batteryLevel": 78,

  "location": {
    "latitude": 37.7820,
    "longitude": -122.4100,
    "accuracy": 15
  },

  "inCompliance": true,
  "sensorType": "bluetooth",
  "accuracy": 0.5
}
```

---

## 4.6 검증 규칙

### 4.6.1 주문 검증

```typescript
class OrderValidator {
  validateOrder(order: Order): ValidationResult {
    const errors: string[] = [];

    // 필수 필드
    if (!order.restaurantId) errors.push("restaurantId is required");
    if (!order.customerId) errors.push("customerId is required");
    if (!order.items || order.items.length === 0) {
      errors.push("At least one item is required");
    }

    // 품목 검증
    order.items.forEach((item, index) => {
      if (item.quantity <= 0) {
        errors.push(`Item ${index}: quantity must be > 0`);
      }
      if (item.unitPrice < 0) {
        errors.push(`Item ${index}: unitPrice cannot be negative`);
      }
    });

    // 위치 검증
    if (!this.isValidLatitude(order.deliveryLocation.latitude)) {
      errors.push("Invalid delivery latitude");
    }
    if (!this.isValidLongitude(order.deliveryLocation.longitude)) {
      errors.push("Invalid delivery longitude");
    }

    // 재무 검증
    const calculatedSubtotal = order.items.reduce(
      (sum, item) => sum + item.totalPrice, 0
    );
    if (Math.abs(calculatedSubtotal - order.subtotal) > 1) {
      errors.push("Subtotal mismatch");
    }

    return {
      valid: errors.length === 0,
      errors
    };
  }

  private isValidLatitude(lat: number): boolean {
    return lat >= -90 && lat <= 90;
  }

  private isValidLongitude(lng: number): boolean {
    return lng >= -180 && lng <= 180;
  }
}
```

---

## 4.7 요약

이 장은 모든 핵심 데이터 구조를 정의했습니다:
- **주문**: 품목, 위치, 상태가 있는 완전한 주문 수명 주기
- **배달원**: 프로필, 장비, 성과, 수입
- **경로**: 편차가 있는 최적화된 다중 정류장 경로
- **온도**: 경고가 있는 실시간 모니터링

모든 구조는 다음을 사용합니다:
- **JSON** 데이터 교환용
- **TypeScript** 타입 안전성용
- **표준 형식** (ISO 8601, E.164, UUID)
- **명확한 검증** 규칙
- **확장성** 메타데이터 필드를 통한

---

## 복습 질문

1. 주문 엔티티의 세 가지 주요 섹션(식별, 당사자, 품목 등)을 나열하고 각각에 포함된 필드 유형을 설명하십시오.

2. OrderStatus에 사용 가능한 모든 값을 순서대로 나열하고 각 상태가 배달 프로세스의 어느 단계를 나타내는지 설명하십시오.

3. TemperatureType의 네 가지 가능한 값은 무엇이며 각각은 언제 사용됩니까?

4. 배달원 엔티티에서 장비 정보를 추적하는 것이 왜 중요한지 설명하십시오.

5. Location 인터페이스에서 verified 필드의 목적은 무엇입니까?

6. 경로 엔티티가 편차(deviations)를 어떻게 추적하고 이 정보가 시스템 개선에 왜 유용한지 설명하십시오.

7. OrderValidator 클래스에서 수행하는 검증 유형을 나열하십시오. 재무 검증이 중요한 이유는 무엇입니까?

8. 온도 읽기에서 배터리 수준을 추적하는 것이 왜 중요한지 설명하십시오.

9. 메타데이터 필드가 모든 주요 엔티티에 포함된 이유는 무엇이며 이것이 향후 확장성에 어떻게 도움이 됩니까?

10. VehicleType과 vehicleMake/vehicleModel의 차이점은 무엇이며 각각이 시스템에서 어떻게 사용됩니까?

---

**다음 장**: [5장: API 인터페이스 사양 →](05-api-interface.md)

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
