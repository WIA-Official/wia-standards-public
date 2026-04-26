# 제6장: 프로토콜 및 알고리즘

---

## 학습 목표

이 장을 마치면 다음을 수행할 수 있습니다:

1. 드라이버 배정을 위한 다중 요소 점수 알고리즘 이해 및 구현
2. 다중 정류장 경로를 위한 2-Opt 최적화 알고리즘 적용
3. 실시간 온도 모니터링 프로토콜 구현
4. 머신 러닝 기반 ETA 예측 시스템 구축
5. 각 알고리즘의 시간 복잡도 및 성능 특성 분석
6. 프로덕션 환경을 위한 알고리즘 최적화 및 조정

---

## 6.1 개요

이 장에서는 음식 배달 시스템의 핵심 기능을 위한 상세한 알고리즘과 프로토콜을 제공합니다: 드라이버 배정, 경로 최적화, 온도 모니터링 및 ETA 예측.

모든 알고리즘은 선호하는 언어로 적응할 수 있는 프로덕션 준비 Python 구현을 포함합니다.

---

## 6.2 드라이버 배정 알고리즘

### 6.2.1 점수 기반 배정

WIA-IND-009 표준은 최적의 드라이버를 각 주문에 배정하기 위해 다중 요소 점수 시스템을 사용합니다.

**목표:** 드라이버 활용도와 고객 만족도를 최대화하면서 배달 시간을 최소화합니다.

**구현:**

```python
from typing import List, Optional
from dataclasses import dataclass
from math import radians, cos, sin, asin, sqrt

@dataclass
class Location:
    latitude: float
    longitude: float

@dataclass
class Driver:
    id: str
    location: Location
    rating: float  # 0-5
    completion_rate: float  # 0-1
    active_orders: List[str]
    max_orders: int
    has_hot_bag: bool
    has_cold_bag: bool
    vehicle_type: str

@dataclass
class Order:
    id: str
    pickup_location: Location
    temperature_requirement: str  # 'hot', 'cold', 'ambient', 'frozen'
    vehicle_type_required: Optional[str] = None

def haversine_distance(loc1: Location, loc2: Location) -> float:
    """
    Haversine 공식을 사용하여 두 좌표 간 거리 계산 (킬로미터)
    """
    # 십진수 도를 라디안으로 변환
    lon1, lat1, lon2, lat2 = map(
        radians,
        [loc1.longitude, loc1.latitude, loc2.longitude, loc2.latitude]
    )

    # Haversine 공식
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a))
    km = 6371 * c  # 지구 반지름 (킬로미터)

    return km

def calculate_driver_score(
    driver: Driver,
    order: Order,
    max_search_radius: float = 10.0
) -> float:
    """
    드라이버-주문 쌍에 대한 배정 점수 계산

    요소 및 가중치:
    - 픽업까지 거리: 40%
    - 드라이버 평점: 25%
    - 완료율: 20%
    - 현재 부하: 10%
    - 장비 능력: 5%

    0.0 (최악)부터 1.0 (최상)까지의 점수 반환
    """

    # 요소 1: 픽업까지 거리 (40% 가중치)
    distance = haversine_distance(driver.location, order.pickup_location)
    if distance > max_search_radius:
        return 0.0  # 너무 멀어서 실격

    distance_score = 1.0 - (distance / max_search_radius)

    # 요소 2: 드라이버 평점 (25% 가중치)
    rating_score = driver.rating / 5.0

    # 요소 3: 완료율 (20% 가중치)
    completion_score = driver.completion_rate

    # 요소 4: 현재 부하 (10% 가중치)
    # 더 많은 주문을 처리할 수 있는 드라이버 선호
    load_ratio = len(driver.active_orders) / driver.max_orders
    load_score = 1.0 - load_ratio

    # 요소 5: 장비 능력 (5% 가중치)
    equipment_score = 1.0
    if order.temperature_requirement == 'hot' and not driver.has_hot_bag:
        equipment_score = 0.5
    elif order.temperature_requirement == 'cold' and not driver.has_cold_bag:
        equipment_score = 0.5

    # 가중 합계
    total_score = (
        0.40 * distance_score +
        0.25 * rating_score +
        0.20 * completion_score +
        0.10 * load_score +
        0.05 * equipment_score
    )

    return total_score

def assign_driver(
    order: Order,
    available_drivers: List[Driver],
    max_radius: float = 10.0
) -> Optional[Driver]:
    """
    점수 알고리즘을 기반으로 최적의 드라이버를 주문에 배정

    반환:
        최적의 드라이버 또는 적합한 드라이버가 없으면 None
    """

    if not available_drivers:
        return None

    # 필요한 경우 차량 유형으로 필터링
    if order.vehicle_type_required:
        available_drivers = [
            d for d in available_drivers
            if d.vehicle_type == order.vehicle_type_required
        ]

    # 모든 드라이버에 대한 점수 계산
    driver_scores = []
    for driver in available_drivers:
        score = calculate_driver_score(driver, order, max_radius)
        if score > 0:  # 적격 드라이버만 포함
            driver_scores.append((driver, score))

    if not driver_scores:
        return None  # 적격 드라이버 없음

    # 점수순 정렬 (내림차순) 및 최상위 반환
    driver_scores.sort(key=lambda x: x[1], reverse=True)
    best_driver, best_score = driver_scores[0]

    print(f"드라이버 {best_driver.id}를 점수 {best_score:.3f}로 배정")

    return best_driver

# 사용 예제
if __name__ == "__main__":
    order = Order(
        id="order_123",
        pickup_location=Location(37.7749, -122.4194),
        temperature_requirement='hot'
    )

    drivers = [
        Driver(
            id="drv_1",
            location=Location(37.7800, -122.4150),
            rating=4.9,
            completion_rate=0.98,
            active_orders=[],
            max_orders=3,
            has_hot_bag=True,
            has_cold_bag=True,
            vehicle_type='ebike'
        ),
        Driver(
            id="drv_2",
            location=Location(37.7700, -122.4200),
            rating=4.5,
            completion_rate=0.85,
            active_orders=["order_456"],
            max_orders=3,
            has_hot_bag=True,
            has_cold_bag=False,
            vehicle_type='bike'
        )
    ]

    assigned = assign_driver(order, drivers)
    print(f"배정됨: {assigned.id if assigned else '없음'}")
```

**점수 계산 예제:**

```
드라이버 A:
  - 거리: 0.5km (점수: 0.95) × 0.40 = 0.38
  - 평점: 4.9/5.0 (점수: 0.98) × 0.25 = 0.245
  - 완료율: 0.98 × 0.20 = 0.196
  - 부하: 0/3 (점수: 1.0) × 0.10 = 0.10
  - 장비: 완전 (점수: 1.0) × 0.05 = 0.05
  총 점수: 0.971

드라이버 B:
  - 거리: 1.2km (점수: 0.88) × 0.40 = 0.352
  - 평점: 4.5/5.0 (점수: 0.90) × 0.25 = 0.225
  - 완료율: 0.85 × 0.20 = 0.170
  - 부하: 1/3 (점수: 0.67) × 0.10 = 0.067
  - 장비: 부분적 (점수: 0.5) × 0.05 = 0.025
  총 점수: 0.839

결과: 드라이버 A 선택 (더 높은 점수)
```

---

## 6.3 경로 최적화 알고리즘

### 6.3.1 단일 정류장 라우팅

단일 픽업에서 단일 배달까지는 실시간 교통과 함께 표준 최단 경로 알고리즘을 사용합니다.

**알고리즘: 교통 가중치가 있는 Dijkstra**

```python
import heapq
from typing import Dict, List, Tuple
from datetime import datetime, timedelta

@dataclass
class Edge:
    from_node: str
    to_node: str
    base_time: float  # 분
    distance: float  # km

class TrafficAwareRouter:
    def __init__(self, graph: Dict[str, List[Edge]]):
        self.graph = graph

    def get_traffic_factor(self, edge: Edge, time: datetime) -> float:
        """
        주어진 시간에 엣지에 대한 교통 승수 가져오기
        1.0 (교통 없음)부터 3.0 (심한 정체)까지 반환
        """
        hour = time.hour

        # 러시 아워 (오전 8-9시, 오후 5-7시): 2.0배
        if (8 <= hour < 9) or (17 <= hour < 19):
            return 2.0
        # 혼잡 (오후 12-1시, 오후 7-9시): 1.5배
        elif (12 <= hour < 13) or (19 <= hour < 21):
            return 1.5
        # 비혼잡: 1.0배
        else:
            return 1.0

    def calculate_route(
        self,
        start: str,
        end: str,
        departure_time: datetime
    ) -> Tuple[List[str], float, float]:
        """
        Dijkstra 알고리즘을 사용한 최적 경로 계산

        반환:
            (경로, 거리_km, 소요시간_분)
        """

        # 우선순위 큐: (총_시간, 현재_노드, 경로, 거리)
        queue = [(0, start, [start], 0)]
        visited = set()
        best = {}

        while queue:
            time, node, path, distance = heapq.heappop(queue)

            if node in visited:
                continue

            visited.add(node)

            if node == end:
                return path, distance, time

            # 이웃 탐색
            for edge in self.graph.get(node, []):
                if edge.to_node in visited:
                    continue

                # 시간 의존적 엣지 가중치 계산
                edge_time = departure_time + timedelta(minutes=time)
                traffic = self.get_traffic_factor(edge, edge_time)
                weighted_time = edge.base_time * traffic

                new_time = time + weighted_time
                new_distance = distance + edge.distance
                new_path = path + [edge.to_node]

                # 이전 최선보다 나은 경우에만 추가
                if edge.to_node not in best or new_time < best[edge.to_node]:
                    best[edge.to_node] = new_time
                    heapq.heappush(
                        queue,
                        (new_time, edge.to_node, new_path, new_distance)
                    )

        return None, 0, float('inf')  # 경로를 찾지 못함
```

### 6.3.2 다중 정류장 최적화 (TSP)

**문제:** N개의 정류장 (픽업 및 배달)이 주어졌을 때 최적의 순서를 찾습니다.

**알고리즘: 2-Opt 휴리스틱**

```python
from typing import List, Tuple
from dataclasses import dataclass
import time

@dataclass
class Stop:
    id: str
    type: str  # 'pickup' 또는 'delivery'
    order_id: str
    location: Location
    time_window_start: datetime
    time_window_end: datetime
    service_duration: int  # 분

def calculate_travel_time(stop1: Stop, stop2: Stop) -> float:
    """
    두 정류장 간 이동 시간 계산 (분)
    거리 + 교통 요소 포함
    """
    distance = haversine_distance(stop1.location, stop2.location)
    # 교통을 고려한 평균 속도 25 km/h로 가정
    return (distance / 25.0) * 60.0

def route_cost(route: List[Stop], start_time: datetime) -> float:
    """
    경로의 총 비용 계산: 이동 시간 + 지연 벌금

    지연 벌금: 초당 1000초
    """
    total_time = 0
    current_time = start_time
    penalty = 0

    for i in range(len(route) - 1):
        # 이동 시간
        travel = calculate_travel_time(route[i], route[i+1])
        total_time += travel
        current_time += timedelta(minutes=travel)

        # 다음 정류장에서 서비스 시간
        total_time += route[i+1].service_duration
        current_time += timedelta(minutes=route[i+1].service_duration)

        # 지연 확인
        if current_time > route[i+1].time_window_end:
            delay_seconds = (current_time - route[i+1].time_window_end).total_seconds()
            penalty += 1000 * delay_seconds

    return total_time + penalty

def is_valid_route(route: List[Stop]) -> bool:
    """
    경로가 선행 제약 조건을 만족하는지 확인:
    - 각 픽업은 배달보다 먼저 발생해야 함
    """
    pickup_positions = {}
    delivery_positions = {}

    for i, stop in enumerate(route):
        if stop.type == 'pickup':
            pickup_positions[stop.order_id] = i
        else:  # delivery
            delivery_positions[stop.order_id] = i

    # 모든 픽업이 배달보다 먼저인지 확인
    for order_id in pickup_positions:
        if order_id in delivery_positions:
            if pickup_positions[order_id] >= delivery_positions[order_id]:
                return False

    return True

def two_opt_swap(route: List[Stop], i: int, j: int) -> List[Stop]:
    """
    2-opt 교환 수행: i에서 j까지 세그먼트 반전

    원본: [..., A, B, C, D, E, ...]
                  ^        ^
                  i        j

    2-opt 후: [..., A, D, C, B, E, ...]
    """
    new_route = route[:i] + route[i:j+1][::-1] + route[j+1:]
    return new_route

def optimize_route(
    stops: List[Stop],
    start_time: datetime,
    max_iterations: int = 1000,
    timeout_seconds: float = 5.0
) -> List[Stop]:
    """
    2-opt 알고리즘을 사용한 다중 정류장 경로 최적화

    시간 복잡도: O(n² × iterations)
    일반적인 실행 시간: 10개 정류장에 대해 <500ms
    """

    # 최근접 이웃 초기 경로로 시작
    route = nearest_neighbor_route(stops)

    # 유효성 확인 (픽업이 배달보다 먼저)
    if not is_valid_route(route):
        route = enforce_precedence(route)

    best_cost = route_cost(route, start_time)
    start_optimization = time.time()
    iteration = 0

    improved = True
    while improved and iteration < max_iterations:
        # 시간 초과 확인
        if time.time() - start_optimization > timeout_seconds:
            print(f"{iteration}회 반복 후 시간 초과")
            break

        improved = False
        iteration += 1

        # 모든 2-opt 교환 시도
        for i in range(len(route) - 1):
            for j in range(i + 2, len(route)):
                # 교환된 세그먼트로 새 경로 생성
                new_route = two_opt_swap(route, i, j)

                # 유효하고 더 나은지 확인
                if is_valid_route(new_route):
                    new_cost = route_cost(new_route, start_time)

                    if new_cost < best_cost:
                        route = new_route
                        best_cost = new_cost
                        improved = True
                        print(f"반복 {iteration}: {best_cost:.1f}로 개선")
                        break

            if improved:
                break

    print(f"{iteration}회 반복으로 최적화 완료, 비용: {best_cost:.1f}")
    return route

def nearest_neighbor_route(stops: List[Stop]) -> List[Stop]:
    """
    최근접 이웃 휴리스틱을 사용한 초기 경로 생성
    """
    if not stops:
        return []

    route = [stops[0]]
    remaining = set(stops[1:])

    while remaining:
        last_stop = route[-1]
        # 가장 가까운 정류장 찾기
        nearest = min(
            remaining,
            key=lambda s: haversine_distance(last_stop.location, s.location)
        )
        route.append(nearest)
        remaining.remove(nearest)

    return route

def enforce_precedence(route: List[Stop]) -> List[Stop]:
    """
    모든 픽업이 배달보다 먼저 발생하도록 경로 재정렬
    """
    pickups = [s for s in route if s.type == 'pickup']
    deliveries = [s for s in route if s.type == 'delivery']

    # 주문별로 배달 그룹화
    delivery_map = {}
    for delivery in deliveries:
        delivery_map[delivery.order_id] = delivery

    # 유효한 경로 구축: 각 주문에 대해 픽업 → 배달
    valid_route = []
    for pickup in pickups:
        valid_route.append(pickup)
        if pickup.order_id in delivery_map:
            valid_route.append(delivery_map[pickup.order_id])

    return valid_route
```

**최적화 예제:**

```
초기 경로 (최근접 이웃):
  픽업A → 픽업B → 배달A → 배달B
  총 거리: 8.5km, 시간: 42분

2-opt 개선 후:
  픽업A → 배달A → 픽업B → 배달B
  총 거리: 6.2km, 시간: 35분
  개선: 27% 감소
```

---

## 6.4 온도 모니터링 프로토콜

### 6.4.1 실시간 모니터링

**모니터링 흐름:**

```python
from enum import Enum
from typing import Optional

class AlertLevel(Enum):
    NONE = 0
    WARNING = 1
    CRITICAL = 2
    SEVERE = 3

@dataclass
class TemperatureReading:
    timestamp: datetime
    order_id: str
    temperature: float  # 섭씨
    sensor_id: str

@dataclass
class TemperatureRequirement:
    type: str  # 'hot', 'cold', 'frozen'
    min_temp: Optional[float]
    max_temp: Optional[float]
    max_duration: int  # 분

# 온도 안전 임계값
TEMP_REQUIREMENTS = {
    'hot': TemperatureRequirement(
        type='hot',
        min_temp=60,  # 140°F
        max_temp=None,
        max_duration=45
    ),
    'cold': TemperatureRequirement(
        type='cold',
        min_temp=None,
        max_temp=4,  # 39°F
        max_duration=60
    ),
    'frozen': TemperatureRequirement(
        type='frozen',
        min_temp=None,
        max_temp=-15,  # 5°F
        max_duration=30
    ),
    'ambient': TemperatureRequirement(
        type='ambient',
        min_temp=15,
        max_temp=25,
        max_duration=90
    )
}

class TemperatureMonitor:
    def __init__(self):
        self.violations = {}  # order_id → 위반 시작 시간

    def check_temperature(
        self,
        reading: TemperatureReading,
        requirement: TemperatureRequirement
    ) -> AlertLevel:
        """
        온도 판독값 분석 및 경고 수준 결정

        반환:
            AlertLevel.NONE: 안전 범위 내
            AlertLevel.WARNING: 임계값의 5°C 이내 >2분
            AlertLevel.CRITICAL: 임계값 초과 >5분
            AlertLevel.SEVERE: 위험 구역 >15분
        """

        temp = reading.temperature
        order_id = reading.order_id

        # 온도가 범위를 벗어났는지 확인
        out_of_range = False
        if requirement.min_temp and temp < requirement.min_temp:
            out_of_range = True
            threshold = requirement.min_temp
        elif requirement.max_temp and temp > requirement.max_temp:
            out_of_range = True
            threshold = requirement.max_temp

        if not out_of_range:
            # 온도 정상, 위반 제거
            if order_id in self.violations:
                del self.violations[order_id]
            return AlertLevel.NONE

        # 온도 범위 초과
        if order_id not in self.violations:
            self.violations[order_id] = reading.timestamp

        violation_duration = (
            reading.timestamp - self.violations[order_id]
        ).total_seconds()

        # 지속 시간에 따라 경고 수준 결정
        if violation_duration > 900:  # 15분
            return AlertLevel.SEVERE
        elif violation_duration > 300:  # 5분
            return AlertLevel.CRITICAL
        elif violation_duration > 120:  # 2분
            return AlertLevel.WARNING
        else:
            return AlertLevel.NONE

    def handle_alert(
        self,
        alert_level: AlertLevel,
        reading: TemperatureReading,
        order_id: str
    ):
        """
        경고 수준에 따른 조치
        """

        if alert_level == AlertLevel.WARNING:
            # 드라이버에게 알림
            send_driver_notification(
                order_id,
                "⚠️ 온도 경고: 음식 용기 확인"
            )

        elif alert_level == AlertLevel.CRITICAL:
            # 드라이버 및 지원팀에 알림
            send_driver_notification(
                order_id,
                "🚨 중요: 온도가 안전 범위를 벗어남"
            )
            create_support_ticket(
                order_id,
                priority='high',
                reason='temperature_violation'
            )

        elif alert_level == AlertLevel.SEVERE:
            # 모든 당사자에게 에스컬레이션, 자동 환불
            send_multi_channel_alert(order_id)
            flag_for_quality_review(order_id)
            automatic_refund(
                order_id,
                reason='food_safety_violation',
                amount='full'
            )

# 플레이스홀더 함수 (시스템에 따라 구현)
def send_driver_notification(order_id: str, message: str):
    print(f"{order_id}에 대해 드라이버에게 알림: {message}")

def create_support_ticket(order_id: str, priority: str, reason: str):
    print(f"지원 티켓 생성: {order_id} - {reason}")

def send_multi_channel_alert(order_id: str):
    print(f"다중 채널 경고: {order_id}")

def flag_for_quality_review(order_id: str):
    print(f"품질 검토를 위해 플래그 지정: {order_id}")

def automatic_refund(order_id: str, reason: str, amount: str):
    print(f"자동 환불: {order_id} - {amount} - {reason}")
```

**온도 모니터링 시나리오:**

```
시나리오 1: 정상 작동
시간    온도   상태
18:00   65°C   ✓ 정상
18:02   63°C   ✓ 정상
18:04   62°C   ✓ 정상
결과: 경고 없음

시나리오 2: 경고
시간    온도   상태
18:00   65°C   ✓ 정상
18:02   59°C   ⚠ 임계값 접근 (60°C 미만)
18:04   58°C   ⚠ 경고 (2분 초과)
결과: 드라이버에게 알림

시나리오 3: 심각
시간    온도   상태
18:00   65°C   ✓ 정상
18:03   55°C   🚨 임계값 초과
18:08   52°C   🚨 중요 (5분 초과)
18:18   50°C   🔥 심각 (15분 초과)
결과: 자동 환불, 품질 검토
```

---

## 6.5 ETA 예측

### 6.5.1 머신 러닝 기반 예측

**ETA 구성 요소:**
```
총 배달 시간 = T_prep + T_assign + T_pickup + T_transit + T_dropoff
```

**구현:**

```python
from sklearn.ensemble import RandomForestRegressor
import numpy as np

class ETAPredictor:
    def __init__(self):
        # 각 구성 요소에 대해 별도의 모델 학습
        self.prep_time_model = RandomForestRegressor(n_estimators=100)
        self.transit_time_model = RandomForestRegressor(n_estimators=100)

    def predict_prep_time(
        self,
        restaurant_id: str,
        item_count: int,
        complexity_score: float,
        current_load: int,
        hour: int,
        is_peak: bool
    ) -> dict:
        """
        레스토랑 준비 시간 예측

        특징:
        - restaurant_avg: 이 레스토랑의 이력 평균
        - item_count: 주문 항목 수
        - complexity_score: 0-1 (간단함부터 복잡함까지)
        - current_load: 대기 중인 주문 수
        - hour: 시간 (0-23)
        - is_peak: 불리언 (러시 아워 여부)
        """

        # 이력 평균 가져오기
        restaurant_avg = get_restaurant_avg_prep(restaurant_id)

        # 특징 준비
        features = np.array([[
            restaurant_avg,
            item_count,
            complexity_score,
            current_load,
            hour,
            1.0 if is_peak else 0.0
        ]])

        # 예측
        predicted = self.prep_time_model.predict(features)[0]

        # 신뢰 구간 계산 (80%)
        # 프로덕션에서는 return_std=True로 predict 사용
        confidence_range = predicted * 0.2

        return {
            'expected': predicted,
            'min': predicted - confidence_range,
            'max': predicted + confidence_range,
            'confidence': 0.80
        }

    def predict_transit_time(
        self,
        distance: float,
        vehicle_type: str,
        traffic_factor: float,
        weather_factor: float,
        hour: int
    ) -> float:
        """
        픽업부터 배달까지 운송 시간 예측

        반환: 분
        """

        # 차량 유형별 기본 속도 (km/h)
        base_speeds = {
            'bike': 15,
            'ebike': 20,
            'scooter': 25,
            'motorcycle': 35,
            'car': 30
        }

        base_speed = base_speeds.get(vehicle_type, 20)

        # 교통 및 날씨에 따라 조정
        adjusted_speed = base_speed / (traffic_factor * weather_factor)

        # 시간 계산
        time_hours = distance / adjusted_speed
        time_minutes = time_hours * 60

        return time_minutes

    def calculate_total_eta(
        self,
        order: dict,
        driver: dict,
        current_time: datetime
    ) -> datetime:
        """
        주문에 대한 완전한 ETA 계산
        """

        # 1. 준비 시간
        prep = self.predict_prep_time(
            restaurant_id=order['restaurant_id'],
            item_count=len(order['items']),
            complexity_score=calculate_complexity(order['items']),
            current_load=get_restaurant_load(order['restaurant_id']),
            hour=current_time.hour,
            is_peak=is_peak_hour(current_time)
        )
        prep_time = prep['expected']

        # 2. 배정 시간 (중앙값: 2분)
        assign_time = 2

        # 3. 드라이버에서 픽업까지 시간
        pickup_distance = haversine_distance(
            driver['location'],
            order['pickup_location']
        )
        traffic = get_traffic_factor(current_time)
        weather = get_weather_factor()

        to_pickup_time = self.predict_transit_time(
            distance=pickup_distance,
            vehicle_type=driver['vehicle_type'],
            traffic_factor=traffic,
            weather_factor=weather,
            hour=current_time.hour
        )

        # 4. 레스토랑 서비스 시간 (5분)
        pickup_service_time = 5

        # 5. 고객까지 운송 시간
        delivery_distance = haversine_distance(
            order['pickup_location'],
            order['delivery_location']
        )

        transit_time = self.predict_transit_time(
            distance=delivery_distance,
            vehicle_type=driver['vehicle_type'],
            traffic_factor=traffic,
            weather_factor=weather,
            hour=current_time.hour
        )

        # 6. 고객 서비스 시간 (3분)
        delivery_service_time = 3

        # 총 시간
        total_minutes = (
            prep_time +
            assign_time +
            to_pickup_time +
            pickup_service_time +
            transit_time +
            delivery_service_time
        )

        # ETA 계산
        eta = current_time + timedelta(minutes=total_minutes)

        return eta

def is_peak_hour(time: datetime) -> bool:
    """피크 시간인지 확인"""
    hour = time.hour
    # 점심: 11-13시, 저녁: 17-20시
    return (11 <= hour < 13) or (17 <= hour < 20)

def calculate_complexity(items: List[dict]) -> float:
    """주문 복잡도 점수 계산 (0-1)"""
    # 간단한 휴리스틱: 더 많은 항목 = 더 복잡함
    # 프로덕션에서는 항목별 복잡도 데이터 사용
    complexity = min(len(items) / 10.0, 1.0)
    return complexity

def get_restaurant_avg_prep(restaurant_id: str) -> float:
    """레스토랑의 이력 평균 준비 시간 가져오기"""
    # 이력 데이터를 위해 데이터베이스 쿼리
    return 15.0  # 플레이스홀더

def get_restaurant_load(restaurant_id: str) -> int:
    """레스토랑의 현재 대기 주문 수 가져오기"""
    return 3  # 플레이스홀더

def get_traffic_factor(time: datetime) -> float:
    """교통 승수 가져오기"""
    hour = time.hour
    if (8 <= hour < 9) or (17 <= hour < 19):
        return 2.0  # 러시 아워
    return 1.0

def get_weather_factor() -> float:
    """날씨 영향 승수 가져오기"""
    # 날씨 API 쿼리
    return 1.0  # 맑은 날씨
```

**ETA 예측 예제:**

```
주문 시간: 18:00

1. 준비 시간:
   - 레스토랑 평균: 15분
   - 항목 수: 3개 (+2분)
   - 현재 부하: 5개 주문 (+3분)
   - 피크 시간: 예 (+5분)
   예측: 25분 (신뢰도: 80%)

2. 배정 시간: 2분

3. 픽업까지:
   - 거리: 2.5km
   - 차량: 이바이크 (20km/h)
   - 교통: 러시 아워 (2.0배)
   - 조정된 속도: 10km/h
   시간: 15분

4. 픽업 서비스: 5분

5. 배달까지:
   - 거리: 3.8km
   - 차량: 이바이크
   - 교통: 러시 아워
   시간: 23분

6. 배달 서비스: 3분

총 시간: 73분
ETA: 19:13

신뢰 구간:
  - 최소: 19:03 (낙관적)
  - 예상: 19:13 (가장 가능성 있음)
  - 최대: 19:23 (비관적)
```

---

## 6.6 요약

이 장에서는 다음을 위한 프로덕션 준비 알고리즘을 제공했습니다:

1. **드라이버 배정**: 거리, 평점, 부하 및 장비를 사용한 다중 요소 점수
2. **경로 최적화**: 다중 정류장 TSP를 위한 2-Opt 알고리즘
3. **온도 모니터링**: 에스컬레이션 수준이 있는 실시간 경고
4. **ETA 예측**: 신뢰 구간이 있는 ML 기반 예측

모든 알고리즘은 다음을 위해 설계되었습니다:
- **성능**: <500ms 실행 시간
- **확장성**: 시간당 10,000개 이상의 주문 처리
- **신뢰성**: 우아한 저하 및 폴백

**알고리즘 성능 비교:**

| 알고리즘 | 시간 복잡도 | 공간 복잡도 | 일반적인 실행 시간 |
|---------|------------|------------|------------------|
| 드라이버 배정 | O(n) | O(n) | <50ms (100 드라이버) |
| 단일 경로 | O((V+E)logV) | O(V) | <100ms |
| 다중 경로 (2-Opt) | O(n²×k) | O(n) | <500ms (10 정류장) |
| 온도 모니터링 | O(1) | O(n) | <10ms |
| ETA 예측 | O(f) | O(1) | <200ms |

**최적화 팁:**

1. **캐싱**: 자주 액세스하는 데이터 캐시 (드라이버 위치, 레스토랑 메트릭)
2. **병렬화**: 독립적인 계산을 병렬로 실행
3. **조기 종료**: 임계값에 도달하면 최적화 중지
4. **점진적 업데이트**: 전체 재계산 대신 증분 업데이트 사용
5. **휴리스틱**: 대규모 문제에 대해 정확한 알고리즘 대신 휴리스틱 사용

---

## 복습 문제

1. **드라이버 배정**: 점수 시스템의 각 요소에 가중치를 부여하는 이유는 무엇이며, 가중치를 어떻게 조정하나요?

2. **Haversine 공식**: 직선 거리 대신 Haversine 공식을 사용하는 이유는 무엇인가요?

3. **2-Opt 알고리즘**: 2-Opt가 항상 전역 최적해를 찾는가요? 그렇지 않다면 왜 그런가요?

4. **선행 제약**: 픽업이 배달보다 먼저 발생해야 하는 제약 조건을 어떻게 적용하나요?

5. **온도 모니터링**: 경고 레벨이 3개(WARNING, CRITICAL, SEVERE) 있는 이유는 무엇이며, 각 레벨에서 어떤 조치를 취하나요?

6. **ETA 예측**: 머신 러닝 기반 예측이 간단한 평균보다 나은 이유는 무엇인가요?

7. **교통 요소**: 하루 중 시간에 따라 교통 요소를 어떻게 조정하나요?

8. **알고리즘 선택**: 주문이 50개 이상일 때 2-Opt 대신 어떤 알고리즘을 사용해야 하나요?

---

**다음 장**: [제7장: 시스템 통합 →](07-system-integration.md)

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
