# 3장: WIA 표준 개요

---

## 3.1 아키텍처 개요

WIA-IND-009 표준은 음식 배달 시스템을 위한 모듈식 확장 가능한 아키텍처를 정의합니다. 이 장에서는 시스템 구성 요소, 데이터 흐름 및 설계 결정에 대한 포괄적인 개요를 제공합니다.

### 고수준 아키텍처

```
┌─────────────────────────────────────────────────────────────┐
│                    고객 애플리케이션                          │
│              (웹, iOS, Android, 음성, 채팅)                  │
│                                                              │
│  기능: 탐색, 주문, 추적, 결제, 평가, 지원                     │
└────────────────────────┬────────────────────────────────────┘
                         │ HTTPS/WSS
                         │
┌────────────────────────▼────────────────────────────────────┐
│                   WIA-IND-009 API 게이트웨이                 │
│                                                              │
│  • 인증 및 권한 부여(JWT)                                     │
│  • 속도 제한 및 조절                                          │
│  • 요청 검증                                                  │
│  • API 버전 관리                                             │
│  • 로드 밸런싱                                                │
└─────┬──────────┬──────────┬──────────┬──────────┬──────────┘
      │          │          │          │          │
┌─────▼────┐ ┌──▼──────┐ ┌─▼────────┐ ┌▼─────────┐ ┌▼────────┐
│  주문    │ │ 배달원  │ │  경로    │ │   온도   │ │ 결제    │
│  서비스  │ │ 서비스  │ │ 최적화   │ │  모니터  │ │ 서비스  │
│          │ │         │ │          │ │          │ │         │
│ • 생성   │ │• 프로필 │ │• 단일    │ │• IoT     │ │• 청구   │
│ • 업데이트│ │• 배정  │ │• 다중    │ │• 경고    │ │• 환불   │
│ • 추적   │ │• 추적  │ │• 동적    │ │• 로그    │ │• 분할   │
│ • 취소   │ │• 지표  │ │• TSP     │ │• HACCP   │ │• 정산   │
└─────┬────┘ └──┬──────┘ └─┬────────┘ └┬─────────┘ └┬────────┘
      │          │          │          │          │
┌─────▼──────────▼──────────▼──────────▼──────────▼──────────┐
│                   데이터 및 분석 계층                         │
│                                                              │
│  • PostgreSQL: 핵심 데이터(주문, 배달원, 고객)               │
│  • Redis: 캐싱, 세션, 실시간 데이터                          │
│  • TimescaleDB: 시계열(온도, 위치)                           │
│  • Elasticsearch: 검색, 로그, 분석                           │
│  • Kafka: 이벤트 스트리밍 및 메시지 큐                       │
└────────────────────────┬────────────────────────────────────┘
                         │
┌────────────────────────▼────────────────────────────────────┐
│                  외부 통합                                    │
│                                                              │
│  • 레스토랑 POS(Toast, Square, Clover)                       │
│  • 지도 서비스(Google Maps, Mapbox)                          │
│  • 결제 게이트웨이(Stripe, Square, PayPal)                   │
│  • 날씨 및 교통 API                                           │
│  • IoT 플랫폼(AWS IoT, Azure IoT Hub)                        │
│  • SMS/이메일(Twilio, SendGrid)                              │
│  • 분석(Segment, Mixpanel)                                   │
└─────────────────────────────────────────────────────────────┘
```

---

## 3.2 핵심 서비스

### 3.2.1 주문 서비스

**책임:**
- 주문 생성 및 검증
- 주문 수명 주기 관리
- 상태 전환
- 알림 조율
- 비즈니스 로직 시행

**주요 기능:**
```typescript
interface OrderService {
  // 주문 관리
  createOrder(request: OrderRequest): Promise<Order>;
  getOrder(orderId: string): Promise<Order>;
  updateOrder(orderId: string, update: OrderUpdate): Promise<Order>;
  cancelOrder(orderId: string, reason: string): Promise<void>;

  // 쿼리
  listOrders(filters: OrderFilters): Promise<Order[]>;
  searchOrders(query: string): Promise<Order[]>;

  // 상태 관리
  updateStatus(orderId: string, status: OrderStatus): Promise<void>;
  getStatusHistory(orderId: string): Promise<StatusChange[]>;

  // 추적
  getTrackingInfo(orderId: string): Promise<TrackingInfo>;
  subscribeToUpdates(orderId: string): Promise<EventStream>;
}
```

**상태 머신:**
```
PENDING → CONFIRMED → PREPARING → READY → ASSIGNED
  ↓
CANCELLED

ASSIGNED → PICKED_UP → IN_TRANSIT → ARRIVING → DELIVERED
  ↓                                              ↓
CANCELLED (부분 환불)                        COMPLETED
  ↓
FAILED (전액 환불)
```

### 3.2.2 배달원 서비스

**책임:**
- 배달원 등록 및 확인
- 상태 및 위치 관리
- 성능 추적
- 배정 매칭
- 수입 계산

**주요 기능:**
```typescript
interface DriverService {
  // 배달원 관리
  registerDriver(application: DriverApplication): Promise<Driver>;
  getDriver(driverId: string): Promise<Driver>;
  updateDriver(driverId: string, update: DriverUpdate): Promise<Driver>;
  verifyDriver(driverId: string, verification: Verification): Promise<void>;

  // 상태 및 위치
  updateStatus(driverId: string, status: DriverStatus): Promise<void>;
  updateLocation(driverId: string, location: Location): Promise<void>;
  getLocation(driverId: string): Promise<Location>;

  // 배정
  findAvailableDrivers(criteria: DriverCriteria): Promise<Driver[]>;
  assignOrder(driverId: string, orderId: string): Promise<Assignment>;

  // 성능
  getMetrics(driverId: string, period: TimePeriod): Promise<DriverMetrics>;
  getRating(driverId: string): Promise<number>;

  // 수입
  calculateEarnings(driverId: string, orderId: string): Promise<Earnings>;
  getEarningsHistory(driverId: string, period: TimePeriod): Promise<Earnings[]>;
}
```

**배달원 배정 알고리즘:**
```python
def assign_driver(order: Order) -> Optional[Driver]:
    """
    점수 기반 배달원 배정

    요소:
    1. 픽업까지의 거리(40% 가중치)
    2. 배달원 평점(25% 가중치)
    3. 완료율(20% 가중치)
    4. 현재 일괄 크기(10% 가중치)
    5. 장비 능력(5% 가중치)
    """

    # 반경 내 사용 가능한 배달원 가져오기
    available = find_available_drivers(
        location=order.pickup_location,
        radius=10_km,
        vehicle_compatible=order.vehicle_requirements
    )

    # 각 배달원에 대한 점수 계산
    scores = []
    for driver in available:
        distance_score = 1.0 - (driver.distance / 10_km)  # 0-1
        rating_score = driver.rating / 5.0  # 0-1
        completion_score = driver.completion_rate  # 0-1
        batch_score = 1.0 - (len(driver.active_orders) / driver.max_orders)
        equipment_score = 1.0 if driver.has_required_equipment(order) else 0.5

        total_score = (
            0.40 * distance_score +
            0.25 * rating_score +
            0.20 * completion_score +
            0.10 * batch_score +
            0.05 * equipment_score
        )

        scores.append((driver, total_score))

    # 최고 점수 배달원 반환
    if scores:
        scores.sort(key=lambda x: x[1], reverse=True)
        return scores[0][0]

    return None  # 사용 가능한 배달원 없음
```

### 3.2.3 경로 최적화

**책임:**
- 단일 정류장 경로 계산
- 다중 정류장 경로 최적화(TSP)
- 동적 재경로
- ETA 계산 및 업데이트
- 교통 및 날씨 통합

**주요 기능:**
```typescript
interface RouteOptimizer {
  // 단일 경로
  calculateRoute(
    pickup: Location,
    delivery: Location,
    vehicle: VehicleType
  ): Promise<Route>;

  // 다중 정류장 최적화
  optimizeMultiStop(
    stops: Stop[],
    constraints: RouteConstraints
  ): Promise<Route>;

  // 동적 업데이트
  recalculateRoute(
    routeId: string,
    currentLocation: Location,
    conditions: TrafficConditions
  ): Promise<Route>;

  // ETA
  calculateETA(
    route: Route,
    currentLocation: Location
  ): Promise<Date>;

  updateETA(routeId: string): Promise<Date>;
}
```

**최적화 알고리즘(TSP용 2-Opt):**
```python
def optimize_route(stops: List[Stop], constraints: Constraints) -> Route:
    """
    다중 정류장 경로 최적화를 위한 2-Opt 알고리즘

    시간 복잡도: 반복당 O(n²)
    공간 복잡도: O(n)
    일반적인 실행 시간: 10개 정류장에 대해 <500ms
    """

    # 최근접 이웃을 사용한 초기 경로
    route = nearest_neighbor_route(stops)

    # 제약 조건 시행(배달 전 픽업)
    route = enforce_precedence_constraints(route)

    # 2-Opt 개선
    improved = True
    max_iterations = 1000
    iteration = 0

    while improved and iteration < max_iterations:
        improved = False
        iteration += 1

        for i in range(len(route) - 1):
            for j in range(i + 2, len(route)):
                # 엣지 교환 시도
                new_route = two_opt_swap(route, i, j)

                # 유효하고 더 나은지 확인
                if (satisfies_constraints(new_route, constraints) and
                    route_cost(new_route) < route_cost(route)):
                    route = new_route
                    improved = True
                    break

            if improved:
                break

    return route

def route_cost(route: List[Stop]) -> float:
    """
    총 비용 계산: 이동 시간 + 벌금
    """
    total_time = 0
    current_time = now()
    penalty = 0

    for i in range(len(route) - 1):
        # 이동 시간
        travel = calculate_travel_time(route[i], route[i+1])
        total_time += travel
        current_time += travel

        # 정류장에서 서비스 시간
        total_time += route[i+1].service_duration
        current_time += route[i+1].service_duration

        # 지연 벌금
        if current_time > route[i+1].delivery_window.end:
            delay = (current_time - route[i+1].delivery_window.end).seconds
            penalty += 1000 * delay  # 지연된 초당 1000초 벌금

    return total_time + penalty
```

### 3.2.4 온도 모니터

**책임:**
- IoT 센서 데이터 수집
- 실시간 온도 분석
- 경고 생성 및 에스컬레이션
- 규정 준수 로깅
- HACCP 보고

**주요 기능:**
```typescript
interface TemperatureMonitor {
  // 데이터 수집
  recordReading(reading: TemperatureReading): Promise<void>;
  getReadings(orderId: string): Promise<TemperatureReading[]>;

  // 분석
  analyzeCompliance(orderId: string): Promise<ComplianceReport>;
  checkThresholds(reading: TemperatureReading): Promise<Alert[]>;

  // 경고
  sendAlert(alert: TemperatureAlert): Promise<void>;
  getAlerts(orderId: string): Promise<Alert[]>;

  // 보고
  generateComplianceReport(period: TimePeriod): Promise<Report>;
  exportHACCPLog(orderId: string): Promise<HACCPLog>;
}
```

**온도 모니터링 흐름:**
```
IoT 센서(60초마다) → 블루투스 → 배달원 전화
                                          ↓
                                     모바일 앱
                                          ↓
                                   HTTPS POST /temperature
                                          ↓
                              온도 모니터 서비스
                                          ↓
                        ┌─────────────────┼─────────────────┐
                        ↓                 ↓                 ↓
                  TimescaleDB      경고 엔진          분석
                  (저장)          (임계값)          (추세)
                        ↓                 ↓                 ↓
                  규정 준수        배달원 앱         대시보드
                  로그            알림
```

**경고 로직:**
```python
def check_temperature_alert(reading: TemperatureReading, order: Order) -> Optional[Alert]:
    """
    온도 측정값이 경고를 필요로 하는지 확인
    """

    temp = reading.temperature
    requirements = order.temperature_requirements

    # 뜨거운 음식 확인
    if requirements.type == 'hot':
        if temp < 60 and temp >= 55:
            return Alert(level='WARNING', message='온도 하강')
        elif temp < 55:
            return Alert(level='CRITICAL', message='안전 온도 미만')

    # 차가운 음식 확인
    elif requirements.type == 'cold':
        if temp > 4 and temp <= 8:
            return Alert(level='WARNING', message='온도 상승')
        elif temp > 8:
            return Alert(level='CRITICAL', message='안전 온도 초과')

    # 냉동 음식 확인
    elif requirements.type == 'frozen':
        if temp > -15:
            return Alert(level='CRITICAL', message='해동 감지')

    return None  # 온도 정상
```

### 3.2.5 결제 서비스

**책임:**
- 결제 승인 및 캡처
- 환불 처리
- 분할 결제(고객 → 플랫폼 → 레스토랑/배달원)
- 거래 로깅
- PCI DSS 규정 준수

**주요 기능:**
```typescript
interface PaymentService {
  // 승인
  authorizePayment(orderId: string, amount: number): Promise<Authorization>;

  // 캡처
  capturePayment(authorizationId: string): Promise<Transaction>;

  // 환불
  refundPayment(
    transactionId: string,
    amount: number,
    reason: string
  ): Promise<Refund>;

  // 정산
  splitPayment(
    transactionId: string,
    splits: PaymentSplit[]
  ): Promise<void>;

  // 기록
  getTransactionHistory(orderId: string): Promise<Transaction[]>;
}
```

**결제 흐름:**
```
주문 생성 → 결제 승인($42.50)
                ↓
         [카드 보류]
                ↓
주문 배달 → 결제 캡처
                ↓
         ┌──────┴──────┬──────────┐
         ↓             ↓          ↓
    레스토랑       플랫폼      배달원
    ($25.00)       ($8.50)     ($9.00)
    [음식의 90%]   [수수료]    [배달+팁]
```

---

## 3.3 데이터 흐름 패턴

### 3.3.1 주문 생성 흐름

```
1. 고객이 주문 제출
   POST /orders
   {
     restaurantId, items, deliveryLocation, paymentMethod
   }
   ↓
2. API 게이트웨이가 요청 검증
   - 인증 확인
   - 속도 제한 확인
   - 입력 검증
   ↓
3. 주문 서비스가 처리
   - 레스토랑 검증(활성, 서비스 지역 내)
   - 항목 검증(사용 가능, 정확한 가격)
   - 배달 검증(범위 내, 주소 유효)
   - 비용 계산(소계, 수수료, 세금)
   ↓
4. 결제 서비스가 승인
   - 카드의 총 금액 승인
   - 자금 보류(아직 캡처되지 않음)
   ↓
5. 주문 서비스가 주문 생성
   - 데이터베이스에 삽입
   - 상태: PENDING → CONFIRMED
   ↓
6. 알림 전송
   - 고객: 주문 확인됨
   - 레스토랑: 새 주문 수신됨
   ↓
7. 배달원 배정 트리거
   - 사용 가능한 배달원 찾기
   - 점수 계산
   - 최고 배달원에게 배정
   ↓
8. 배달원 수락
   - 상태: CONFIRMED → ASSIGNED
   - 모든 당사자에게 알림 전송
```

### 3.3.2 실시간 추적 흐름

```
WebSocket 연결:
고객 앱 ←──WebSocket──→ 추적 서비스

배달원 위치 업데이트:
배달원 전화(10초마다 GPS)
   ↓
모바일 앱
   ↓
POST /drivers/{id}/location
   {latitude, longitude, heading, speed}
   ↓
배달원 서비스
   ↓
Redis(현재 위치 캐시)
   ↓
WebSocket 서버(구독자에게 푸시)
   ↓
고객 앱(지도 업데이트)

ETA 업데이트(2분마다):
현재 위치 + 교통 데이터
   ↓
경로 최적화
   ↓
새 ETA 계산
   ↓
WebSocket을 통해 푸시
   ↓
고객 앱(표시 업데이트)
```

### 3.3.3 온도 모니터링 흐름

```
IoT 센서(BLE) → 배달원 전화(60초마다)
                         ↓
                   모바일 앱
                         ↓
        POST /temperature-readings
        {
          orderId, sensorId, temperature,
          timestamp, location
        }
                         ↓
              온도 모니터 서비스
                         ↓
        ┌────────────────┼────────────────┐
        ↓                ↓                ↓
   TimescaleDB    경고 엔진     실시간 대시보드
   (영구)        (임계값)     (WebSocket 푸시)
        ↓                ↓
   규정 준수      CRITICAL인 경우:
   보고          - 배달원에게 알림
                - 지원팀에게 알림
                - 주문 플래그
                - 심각한 경우 자동 환불
```

---

## 3.4 기술 스택

### 3.4.1 필수 구성 요소

**백엔드:**
- RESTful API 프레임워크(Express.js, FastAPI, Spring Boot 등)
- WebSocket 서버(Socket.io, ws 등)
- 데이터베이스: PostgreSQL 또는 호환 RDBMS
- 캐시: Redis 또는 호환
- 메시지 큐: Kafka, RabbitMQ, AWS SQS 등

**시계열 데이터(온도, 위치):**
- TimescaleDB(PostgreSQL 확장)
- InfluxDB
- AWS Timestream
- Google Cloud Bigtable

**검색 및 분석:**
- Elasticsearch(선택 사항이지만 권장)
- PostgreSQL 전체 텍스트 검색(최소)

**인증:**
- JWT(JSON Web Tokens)
- 타사 통합을 위한 OAuth 2.0

**API:**
- 지도: Google Maps, Mapbox, HERE 또는 OpenStreetMap
- 결제: Stripe, Square, Braintree, PayPal
- 통신: Twilio(SMS), SendGrid(이메일)
- 날씨: OpenWeatherMap, Weather.com API

### 3.4.2 선택적 구성 요소

**고급 기능:**
- 유연한 쿼리를 위한 GraphQL
- 서비스 간 통신을 위한 gRPC
- 빅 데이터 분석을 위한 Apache Spark
- ML 모델을 위한 TensorFlow/PyTorch

**인프라:**
- 컨테이너 오케스트레이션을 위한 Kubernetes
- 코드형 인프라를 위한 Terraform
- 컨테이너화를 위한 Docker
- CI/CD: GitHub Actions, GitLab CI, Jenkins

### 3.4.3 배포 아키텍처

**소규모(단일 도시, <1000 주문/일):**
```
단일 서버:
- API + WebSocket 서버
- PostgreSQL 데이터베이스
- Redis 캐시
단일 머신 또는 소규모 클러스터
```

**중간 규모(여러 도시, 10K 주문/일):**
```
로드 밸런서
   ↓
API 서버(3개 이상 인스턴스)
   ↓
애플리케이션 서버(5개 이상 인스턴스)
- 주문 서비스
- 배달원 서비스
- 경로 최적화
- 온도 모니터
   ↓
데이터 계층
- PostgreSQL(기본 + 복제본)
- Redis 클러스터
- TimescaleDB
   ↓
메시지 큐(Kafka 클러스터)
```

**대규모(국가/글로벌, 100K+ 주문/일):**
```
CDN + 로드 밸런서(글로벌)
   ↓
API 게이트웨이(지리적으로 분산)
   ↓
마이크로서비스(자동 확장)
- 주문 서비스(20개 이상 인스턴스)
- 배달원 서비스(10개 이상 인스턴스)
- 경로 최적화(5개 이상 인스턴스)
- 결제 서비스(10개 이상 인스턴스)
- 온도 모니터(5개 이상 인스턴스)
   ↓
데이터 계층(분산)
- PostgreSQL(샤딩, 다중 지역)
- Redis(클러스터링, 지리적 복제)
- TimescaleDB(파티셔닝)
- Elasticsearch 클러스터
   ↓
메시지 큐(Kafka 다중 지역)
   ↓
분석 및 ML 파이프라인
- 데이터 웨어하우스(Snowflake, BigQuery)
- ML 모델(SageMaker, Vertex AI)
```

---

## 3.5 확장성 고려 사항

### 3.5.1 수직 확장 한계

**단일 서버 용량:**
- 시간당 100-500 주문
- 1,000-5,000 동시 WebSocket 연결
- 초당 10,000 데이터베이스 쿼리

**수평 확장 시기:**
- 일관된 >70% CPU 사용량
- 데이터베이스 연결 풀 소진
- 느린 쿼리 시간(>100ms p95)
- WebSocket 연결 한계 도달

### 3.5.2 수평 확장 전략

**무상태 서비스:**
- API 서버: N개 인스턴스로 확장
- 라운드 로빈 또는 최소 연결로 로드 밸런싱
- Redis의 세션 데이터(공유 상태)

**데이터베이스 확장:**
- 쿼리 로드를 위한 읽기 복제본
- 쓰기 로드를 위한 샤딩(도시/지역별)
- 연결 풀링(PgBouncer)

**캐시 전략:**
- 고가용성을 위한 Redis 클러스터
- 자주 액세스되는 데이터 캐시:
  - 배달원 위치(60초 TTL)
  - 레스토랑 메뉴(5분 TTL)
  - 주문 상태(30초 TTL)

**WebSocket 확장:**
- 서버 간 메시징을 위해 Redis pub/sub 사용
- 연결 유지를 위한 스티키 세션
- 로드 밸런서로 수평 확장

### 3.5.3 성능 목표

**응답 시간 SLA:**
```
엔드포인트                    P50      P95      P99
──────────────────────────────────────────────────
GET /orders/:id           <50ms    <100ms   <200ms
POST /orders              <100ms   <300ms   <500ms
GET /tracking             <50ms    <100ms   <150ms
POST /routes/optimize     <500ms   <2s      <5s
WebSocket 메시지          <100ms   <200ms   <500ms
```

**처리량 목표:**
```
시간당 10,000 주문 피크 = 초당 2.8 주문
× 주문당 10 API 호출 = 최소 초당 28 요청
3배 안전 마진 포함 = 필요한 초당 84 요청 용량
```

**가용성 목표:**
```
99.9% 가동 시간 = 월 43분 다운타임
- 중복 서버
- 데이터베이스 복제
- 다중 AZ 배포
- 자동 페일오버
```

---

## 3.6 보안 아키텍처

### 3.6.1 인증 흐름

```
1. 사용자가 자격 증명으로 로그인
   POST /auth/login
   {email, password}
   ↓
2. 자격 증명 확인
   - 비밀번호 해시(bcrypt)
   - 데이터베이스와 비교
   ↓
3. JWT 토큰 생성
   {
     sub: user_id,
     role: 'customer|driver|restaurant|admin',
     permissions: [...],
     exp: timestamp + 24h
   }
   ↓
4. 클라이언트에 토큰 반환
   ↓
5. 클라이언트가 후속 요청에 포함
   Authorization: Bearer <token>
   ↓
6. API 게이트웨이가 토큰 확인
   - 서명 유효?
   - 만료되지 않음?
   - 필요한 권한이 있음?
```

### 3.6.2 데이터 암호화

**전송 중:**
- 모든 HTTPS 연결에 대한 TLS 1.3
- 실시간 업데이트를 위한 WSS(보안 WebSocket)
- 모바일 앱의 인증서 고정

**저장 중:**
- 데이터베이스에 대한 AES-256 암호화
- PII에 대한 별도 암호화 키
- 결제 데이터: PCI DSS 레벨 1 준수

### 3.6.3 API 보안

**속도 제한:**
```
표준 티어: 분당 100 요청
프리미엄 티어: 분당 1,000 요청
엔터프라이즈 티어: 분당 10,000 요청

WebSocket: 사용자당 최대 10 연결
```

**입력 검증:**
- 유효한 문자 화이트리스트
- 문자열의 길이 제한
- 숫자의 범위 검증
- SQL 인젝션 방지(매개변수화된 쿼리)
- XSS 방지(HTML 삭제)

**요청 서명(웹훅용):**
```
X-WIA-Signature: HMAC-SHA256(secret, timestamp + body)
X-WIA-Timestamp: Unix 타임스탬프
타임스탬프 > 5분인 경우 거부(재생 공격 방지)
```

---

## 3.7 모니터링 및 관찰성

### 3.7.1 로깅

**구조화된 로깅(JSON 형식):**
```json
{
  "timestamp": "2025-01-15T14:30:00Z",
  "level": "INFO",
  "service": "order-service",
  "trace_id": "abc123",
  "user_id": "user_456",
  "order_id": "order_789",
  "message": "주문이 성공적으로 생성되었습니다",
  "duration_ms": 145,
  "status_code": 201
}
```

**로그 집계:**
- 저장 및 검색을 위한 Elasticsearch
- 시각화를 위한 Kibana
- 보존: 30일 핫, 90일 아카이브

### 3.7.2 지표

**추적할 주요 지표:**
```
비즈니스 지표:
- 시간당 주문
- 시간당 수익
- 평균 주문 금액
- 고객 획득 비용

성능 지표:
- API 응답 시간(p50, p95, p99)
- 데이터베이스 쿼리 시간
- 캐시 적중률
- 오류율

운영 지표:
- 활성 배달원
- 사용 가능한 배달원
- 평균 배달 시간
- 온도 준수율
- 정시 배달률

인프라 지표:
- CPU 사용량
- 메모리 사용량
- 디스크 I/O
- 네트워크 처리량
```

**도구:**
- 지표 수집을 위한 Prometheus
- 대시보드를 위한 Grafana
- 경고를 위한 PagerDuty

### 3.7.3 추적

**분산 추적:**
- 서비스 간 요청 추적
- 병목 현상 식별
- 성능 문제 디버그

**예시 추적:**
```
주문 생성(총: 456ms)
├─ API 게이트웨이(5ms)
├─ 주문 서비스(145ms)
│  ├─ 레스토랑 검증(45ms)
│  ├─ 항목 검증(30ms)
│  ├─ 비용 계산(20ms)
│  └─ 주문 생성(50ms)
├─ 결제 서비스(180ms)
│  └─ 청구 승인(175ms)
├─ 배달원 서비스(120ms)
│  ├─ 배달원 찾기(80ms)
│  ├─ 점수 계산(25ms)
│  └─ 배달원 배정(15ms)
└─ 알림 서비스(6ms)
```

**도구:**
- 추적을 위한 Jaeger 또는 Zipkin
- 계측을 위한 OpenTelemetry

---

## 3.8 요약

WIA-IND-009 아키텍처는 다음을 제공합니다:

1. **모듈식 설계**: 별도로 확장할 수 있는 독립적인 서비스
2. **실시간 기능**: 추적을 위한 WebSocket, 서브초 업데이트
3. **확장성**: 단일 레스토랑에서 글로벌 플랫폼까지
4. **안정성**: 중복성, 페일오버, 모니터링
5. **보안**: 암호화, 인증, 입력 검증
6. **성능**: 캐싱, 최적화, 효율적인 알고리즘
7. **관찰성**: 로깅, 지표, 추적

이 아키텍처는 식품 안전, 배달원 복지 및 고객 만족을 유지하면서 전체 음식 배달 수명 주기를 지원합니다.

---

**다음 장**: [4장: 데이터 형식 및 모델 →](04-data-format.md)

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
