# 제6장: 비행 제어 및 항법 프로토콜

## Phase 3: 공기역학, 제어 시스템 및 자율 항법

---

## 6.1 비행 역학 기초

### 멀티로터에 작용하는 힘

멀티로터 드론은 비행 중 네 가지 주요 힘을 경험합니다:

```
                    양력 (L)
                      ↑
                      │
                      │
    항력 (D) ←────── [드론] ──────→ 추력 (T)
                      │
                      │
                      ↓
                   중량 (W)

평형 조건:
- 호버: L = W, T = D = 0
- 전진 비행: L ≈ W, T > D
- 상승: L > W
- 하강: L < W
```

### 양력 및 추력 방정식

**총 양력 (멀티로터)**:

$$L = n \times T_{rotor}$$

여기서:
- $L$ = 총 양력 (N)
- $n$ = 로터 수
- $T_{rotor}$ = 로터당 추력 (N)

**개별 로터 추력**:

$$T = C_T \times \rho \times A \times (\omega \times r)^2$$

여기서:
- $C_T$ = 추력 계수 (일반적으로 0.01-0.015)
- $\rho$ = 공기 밀도 (해수면에서 1.225 kg/m³)
- $A$ = 로터 디스크 면적 (m²)
- $\omega$ = 각속도 (rad/s)
- $r$ = 로터 반경 (m)

```python
def calculate_rotor_thrust(ct, rho, diameter, rpm):
    """
    단일 로터가 생성하는 추력 계산.

    Args:
        ct: 추력 계수 (일반적으로 0.012)
        rho: 공기 밀도 (kg/m³)
        diameter: 프로펠러 직경 (m)
        rpm: 분당 회전수

    Returns:
        뉴턴 단위 추력
    """
    import math

    radius = diameter / 2
    area = math.pi * radius ** 2
    omega = rpm * 2 * math.pi / 60  # rad/s로 변환

    thrust = ct * rho * area * (omega * radius) ** 2

    return thrust

# 예시: 5000 RPM에서 15" 프로펠러
thrust = calculate_rotor_thrust(
    ct=0.012,
    rho=1.225,
    diameter=0.381,  # 15 인치
    rpm=5000
)
print(f"단일 로터 추력: {thrust:.2f} N ({thrust/9.81:.2f} kg)")
```

### 전력 요구사항

**호버 전력**:

$$P_{hover} = \frac{(mg)^{3/2}}{\sqrt{2\rho A_{total}}}$$

여기서:
- $m$ = 총 질량 (kg)
- $g$ = 중력 가속도 (9.81 m/s²)
- $A_{total}$ = 총 로터 디스크 면적 (m²)

**전진 비행 전력**:

$$P_{forward} = P_{induced} + P_{profile} + P_{parasite}$$

여기서:
- $P_{induced}$ = 양력 생성 전력
- $P_{profile}$ = 로터 항력 극복 전력
- $P_{parasite}$ = 기체 항력 극복 전력

```python
def calculate_hover_power(mass_kg, num_rotors, rotor_diameter_m, rho=1.225):
    """
    호버링에 필요한 전력 계산.

    Returns:
        와트 단위 전력
    """
    import math

    # 총 중량
    weight = mass_kg * 9.81

    # 총 로터 디스크 면적
    rotor_area = num_rotors * math.pi * (rotor_diameter_m / 2) ** 2

    # 이상적 호버 전력
    p_ideal = weight ** 1.5 / math.sqrt(2 * rho * rotor_area)

    # 효율 손실 고려 (일반적으로 60-70% 효율)
    efficiency = 0.65
    p_actual = p_ideal / efficiency

    return p_actual

# 예시: 15" 프롭을 가진 8kg 헥사콥터
power = calculate_hover_power(
    mass_kg=8,
    num_rotors=6,
    rotor_diameter_m=0.381
)
print(f"호버 전력: {power:.0f} W")
```

### 항력

**항력 방정식**:

$$D = \frac{1}{2} \times \rho \times v^2 \times A_f \times C_D$$

여기서:
- $v$ = 대기속도 (m/s)
- $A_f$ = 정면 면적 (m²)
- $C_D$ = 항력 계수 (멀티로터에서 1.0-1.5)

| 구성 | 일반적인 $C_D$ |
|------|----------------|
| 쿼드콥터 (노출) | 1.3-1.5 |
| 헥사콥터 (밀폐) | 1.0-1.2 |
| 유선형 배송 | 0.8-1.0 |
| 고정익 VTOL | 0.3-0.5 |

---

## 6.2 안정화 및 제어 시스템

### 제어 계층

```
┌─────────────────────────────────────────────────────────────────┐
│                    미션 플래너                                    │
│  (웨이포인트, 목표, 제약)                                         │
├─────────────────────────────────────────────────────────────────┤
│                    항법 컨트롤러                                   │
│  (경로 추적, 위치 제어)                                           │
│  루프 율: 10-50 Hz                                               │
├─────────────────────────────────────────────────────────────────┤
│                    자세 컨트롤러                                   │
│  (롤, 피치, 요 제어)                                              │
│  루프 율: 250-500 Hz                                             │
├─────────────────────────────────────────────────────────────────┤
│                    레이트 컨트롤러                                 │
│  (각속도 제어)                                                    │
│  루프 율: 500-1000 Hz                                            │
├─────────────────────────────────────────────────────────────────┤
│                    모터 믹서                                      │
│  (모터 명령)                                                      │
└─────────────────────────────────────────────────────────────────┘
```

### PID 컨트롤러 구현

**표준 PID 공식**:

$$u(t) = K_p e(t) + K_i \int_{0}^{t} e(\tau) d\tau + K_d \frac{de(t)}{dt}$$

```python
class PIDController:
    """
    드론 안정화를 위한 PID 컨트롤러.
    """

    def __init__(self, kp: float, ki: float, kd: float,
                 integral_limit: float = 100.0,
                 output_limit: float = 1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.output_limit = output_limit

        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time = None

    def compute(self, setpoint: float, measured: float,
                current_time: float) -> float:
        """
        PID 출력 계산.

        Args:
            setpoint: 목표값
            measured: 현재 측정값
            current_time: 현재 타임스탬프

        Returns:
            제어 출력
        """
        error = setpoint - measured

        # 시간 델타
        if self.previous_time is None:
            dt = 0.01  # 첫 호출 시 100Hz 가정
        else:
            dt = current_time - self.previous_time

        # 비례 항
        p_term = self.kp * error

        # 적분 항 (안티와인드업 포함)
        self.integral += error * dt
        self.integral = max(-self.integral_limit,
                           min(self.integral_limit, self.integral))
        i_term = self.ki * self.integral

        # 미분 항
        if dt > 0:
            derivative = (error - self.previous_error) / dt
        else:
            derivative = 0
        d_term = self.kd * derivative

        # 제한이 있는 총 출력
        output = p_term + i_term + d_term
        output = max(-self.output_limit,
                    min(self.output_limit, output))

        # 다음 반복을 위해 저장
        self.previous_error = error
        self.previous_time = current_time

        return output

    def reset(self):
        """컨트롤러 상태 초기화."""
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time = None


# 자세 제어를 위한 일반적인 PID 게인
attitude_controllers = {
    'roll': PIDController(kp=4.5, ki=0.02, kd=0.18),
    'pitch': PIDController(kp=4.5, ki=0.02, kd=0.18),
    'yaw': PIDController(kp=3.0, ki=0.01, kd=0.10)
}
```

### 모터 믹싱

X 구성 헥사콥터용:

```python
def hexacopter_mixer(throttle, roll, pitch, yaw):
    """
    헥사콥터 X 구성을 위한 제어 입력을 모터 명령으로 믹싱.

    모터 배치 (위에서 본 모습):
          1     2
           \   /
        6   [X]   3
           /   \
          5     4

    Returns:
        6개 모터 명령 리스트 (0.0-1.0)
    """
    # 모터 믹싱 매트릭스
    # 각 행: [스로틀, 롤, 피치, 요]
    mix = [
        [1.0, -0.5,  0.866,  1.0],  # 모터 1 (전방-우측)
        [1.0,  0.5,  0.866, -1.0],  # 모터 2 (전방-좌측)
        [1.0,  1.0,  0.0,    1.0],  # 모터 3 (우측)
        [1.0,  0.5, -0.866, -1.0],  # 모터 4 (후방-좌측)
        [1.0, -0.5, -0.866,  1.0],  # 모터 5 (후방-우측)
        [1.0, -1.0,  0.0,   -1.0],  # 모터 6 (좌측)
    ]

    motors = []
    for i in range(6):
        cmd = (mix[i][0] * throttle +
               mix[i][1] * roll +
               mix[i][2] * pitch +
               mix[i][3] * yaw)
        # 유효 범위로 제한
        cmd = max(0.0, min(1.0, cmd))
        motors.append(cmd)

    return motors
```

### 센서 융합 (확장 칼만 필터)

```python
import numpy as np

class EKFStateEstimator:
    """
    드론 상태 추정을 위한 확장 칼만 필터.
    IMU, GPS, 기압계 데이터 융합.
    """

    def __init__(self):
        # 상태: [x, y, z, vx, vy, vz, roll, pitch, yaw]
        self.state = np.zeros(9)

        # 상태 공분산
        self.P = np.eye(9) * 0.1

        # 프로세스 노이즈
        self.Q = np.diag([0.1, 0.1, 0.1, 0.5, 0.5, 0.5, 0.01, 0.01, 0.01])

        # GPS 측정 노이즈
        self.R_gps = np.diag([2.0, 2.0, 3.0])

        # 기압계 측정 노이즈
        self.R_baro = np.array([[1.0]])

    def predict(self, accel, gyro, dt):
        """
        IMU 데이터를 사용한 예측 단계.
        """
        # 자세 업데이트
        self.state[6:9] += gyro * dt

        # 회전 행렬 (간소화)
        roll, pitch, yaw = self.state[6:9]
        c_roll, s_roll = np.cos(roll), np.sin(roll)
        c_pitch, s_pitch = np.cos(pitch), np.sin(pitch)
        c_yaw, s_yaw = np.cos(yaw), np.sin(yaw)

        # 가속도를 세계 좌표계로 회전
        R = np.array([
            [c_yaw*c_pitch, c_yaw*s_pitch*s_roll - s_yaw*c_roll, c_yaw*s_pitch*c_roll + s_yaw*s_roll],
            [s_yaw*c_pitch, s_yaw*s_pitch*s_roll + c_yaw*c_roll, s_yaw*s_pitch*c_roll - c_yaw*s_roll],
            [-s_pitch, c_pitch*s_roll, c_pitch*c_roll]
        ])

        world_accel = R @ accel - np.array([0, 0, 9.81])

        # 속도 업데이트
        self.state[3:6] += world_accel * dt

        # 위치 업데이트
        self.state[0:3] += self.state[3:6] * dt + 0.5 * world_accel * dt**2

        # 공분산 업데이트
        F = self._get_jacobian(dt)
        self.P = F @ self.P @ F.T + self.Q

    def update_gps(self, lat, lon, alt, hdop):
        """
        GPS 측정으로 업데이트.
        """
        # 위경도를 로컬 프레임으로 변환 (간소화)
        gps_pos = self._gps_to_local(lat, lon, alt)

        # 측정 행렬 (위치만)
        H = np.zeros((3, 9))
        H[0, 0] = H[1, 1] = H[2, 2] = 1.0

        # HDOP에 기반한 동적 측정 노이즈
        R = self.R_gps * hdop

        # 칼만 이득
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        # 혁신
        y = gps_pos - self.state[0:3]

        # 상태 업데이트
        self.state += K @ y

        # 공분산 업데이트
        self.P = (np.eye(9) - K @ H) @ self.P

    def update_barometer(self, altitude):
        """
        기압계 측정으로 업데이트.
        """
        H = np.zeros((1, 9))
        H[0, 2] = 1.0

        S = H @ self.P @ H.T + self.R_baro
        K = self.P @ H.T / S[0, 0]

        y = altitude - self.state[2]
        self.state += K.flatten() * y

        self.P = (np.eye(9) - np.outer(K, H)) @ self.P
```

---

## 6.3 경로 계획 및 항법

### A* 경로 계획

```python
import heapq
from typing import List, Tuple, Set

class AStarPathPlanner:
    """
    3D 드론 항법을 위한 A* 경로 계획.
    """

    def __init__(self, grid_resolution: float = 10.0):
        self.resolution = grid_resolution
        self.obstacles: Set[Tuple[int, int, int]] = set()

    def add_obstacle(self, x: float, y: float, z: float, radius: float):
        """지도에 원통형 장애물 추가."""
        # 그리드 셀로 변환
        cells_radius = int(radius / self.resolution) + 1
        cx = int(x / self.resolution)
        cy = int(y / self.resolution)
        cz = int(z / self.resolution)

        for dx in range(-cells_radius, cells_radius + 1):
            for dy in range(-cells_radius, cells_radius + 1):
                for dz in range(-cells_radius, cells_radius + 1):
                    if dx*dx + dy*dy <= cells_radius*cells_radius:
                        self.obstacles.add((cx + dx, cy + dy, cz + dz))

    def plan(self, start: Tuple[float, float, float],
             goal: Tuple[float, float, float]) -> List[Tuple[float, float, float]]:
        """
        A*를 사용하여 시작에서 목표까지 경로 찾기.

        Returns:
            웨이포인트 리스트 (x, y, z)
        """
        # 그리드로 변환
        start_cell = self._to_cell(start)
        goal_cell = self._to_cell(goal)

        # 우선순위 큐: (f_score, g_score, cell, path)
        open_set = [(self._heuristic(start_cell, goal_cell), 0, start_cell, [start_cell])]
        closed_set = set()

        while open_set:
            f, g, current, path = heapq.heappop(open_set)

            if current == goal_cell:
                # 경로를 세계 좌표로 다시 변환
                return [self._to_world(cell) for cell in path]

            if current in closed_set:
                continue
            closed_set.add(current)

            # 이웃 탐색 (6-연결 그리드)
            for neighbor in self._get_neighbors(current):
                if neighbor in closed_set or neighbor in self.obstacles:
                    continue

                # 이웃까지의 비용
                new_g = g + self._distance(current, neighbor)
                new_f = new_g + self._heuristic(neighbor, goal_cell)

                heapq.heappush(open_set, (new_f, new_g, neighbor, path + [neighbor]))

        return []  # 경로 없음

    def _to_cell(self, pos: Tuple[float, float, float]) -> Tuple[int, int, int]:
        return (int(pos[0] / self.resolution),
                int(pos[1] / self.resolution),
                int(pos[2] / self.resolution))

    def _to_world(self, cell: Tuple[int, int, int]) -> Tuple[float, float, float]:
        return (cell[0] * self.resolution + self.resolution / 2,
                cell[1] * self.resolution + self.resolution / 2,
                cell[2] * self.resolution + self.resolution / 2)

    def _heuristic(self, a: Tuple[int, int, int], b: Tuple[int, int, int]) -> float:
        # 고도 페널티가 있는 유클리드 거리
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        dz = b[2] - a[2]
        return ((dx*dx + dy*dy + 1.5 * dz*dz) ** 0.5) * self.resolution

    def _distance(self, a: Tuple[int, int, int], b: Tuple[int, int, int]) -> float:
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        dz = b[2] - a[2]
        return ((dx*dx + dy*dy + dz*dz) ** 0.5) * self.resolution

    def _get_neighbors(self, cell: Tuple[int, int, int]) -> List[Tuple[int, int, int]]:
        x, y, z = cell
        return [
            (x+1, y, z), (x-1, y, z),
            (x, y+1, z), (x, y-1, z),
            (x, y, z+1), (x, y, z-1)
        ]
```

### 동적 윈도우 접근법 (장애물 회피)

```python
class DynamicWindowApproach:
    """
    실시간 장애물 회피를 위한 동적 윈도우 접근법.
    """

    def __init__(self, max_speed: float = 20.0,
                 max_yaw_rate: float = 1.0,
                 dt: float = 0.1):
        self.max_speed = max_speed
        self.max_yaw_rate = max_yaw_rate
        self.dt = dt

        # 목적 함수 가중치
        self.alpha = 0.3  # 방향
        self.beta = 0.5   # 여유
        self.gamma = 0.2  # 속도

    def compute_velocity(self, current_state, goal, obstacles):
        """
        장애물을 고려한 최적 속도 계산.

        Args:
            current_state: (x, y, vx, vy, heading)
            goal: (x, y)
            obstacles: (x, y, radius) 리스트

        Returns:
            (velocity, yaw_rate)
        """
        x, y, vx, vy, heading = current_state
        current_speed = (vx*vx + vy*vy) ** 0.5

        # 동적 윈도우 (도달 가능한 속도)
        min_v = max(0, current_speed - 2.0 * self.dt)
        max_v = min(self.max_speed, current_speed + 2.0 * self.dt)
        min_w = -self.max_yaw_rate
        max_w = self.max_yaw_rate

        best_score = -float('inf')
        best_v, best_w = 0, 0

        # 속도 샘플링
        for v in np.linspace(min_v, max_v, 10):
            for w in np.linspace(min_w, max_w, 10):
                # 궤적 시뮬레이션
                traj = self._simulate(x, y, heading, v, w)

                # 충돌 확인
                if self._check_collision(traj, obstacles):
                    continue

                # 목적 함수 계산
                heading_score = self._heading_objective(traj[-1], goal)
                clearance_score = self._clearance_objective(traj, obstacles)
                velocity_score = v / self.max_speed

                score = (self.alpha * heading_score +
                        self.beta * clearance_score +
                        self.gamma * velocity_score)

                if score > best_score:
                    best_score = score
                    best_v, best_w = v, w

        return best_v, best_w

    def _simulate(self, x, y, heading, v, w, steps=10):
        """주어진 속도에 대한 궤적 시뮬레이션."""
        trajectory = [(x, y)]
        for _ in range(steps):
            heading += w * self.dt
            x += v * np.cos(heading) * self.dt
            y += v * np.sin(heading) * self.dt
            trajectory.append((x, y))
        return trajectory
```

### 정밀 착륙

```python
class PrecisionLandingController:
    """
    AprilTag를 사용한 정밀 착륙을 위한 시각 서보잉.
    """

    def __init__(self, camera_matrix, tag_size: float = 0.5):
        self.camera_matrix = camera_matrix
        self.tag_size = tag_size

        # 착륙 단계
        self.phases = ['APPROACH', 'ALIGN', 'DESCEND', 'FINAL', 'TOUCHDOWN']
        self.current_phase = 'APPROACH'

        # 착륙용 PID 컨트롤러
        self.x_controller = PIDController(kp=0.5, ki=0.01, kd=0.1)
        self.y_controller = PIDController(kp=0.5, ki=0.01, kd=0.1)

    def process_frame(self, frame, current_altitude):
        """
        카메라 프레임 처리 및 착륙 명령 계산.

        Returns:
            (vx, vy, vz, detected)
        """
        # AprilTag 감지
        tag = self._detect_tag(frame)

        if tag is None:
            return 0, 0, 0, False

        # 상대 위치 추정
        rel_x, rel_y = self._estimate_position(tag, current_altitude)

        # 상태 머신
        if self.current_phase == 'APPROACH' and current_altitude < 10:
            self.current_phase = 'ALIGN'

        elif self.current_phase == 'ALIGN':
            error = (rel_x**2 + rel_y**2) ** 0.5
            if error < 0.5 and current_altitude > 3:
                self.current_phase = 'DESCEND'

        elif self.current_phase == 'DESCEND':
            if current_altitude < 3:
                self.current_phase = 'FINAL'

        elif self.current_phase == 'FINAL':
            if current_altitude < 0.5:
                self.current_phase = 'TOUCHDOWN'

        # 단계에 따른 속도 계산
        vx = -self.x_controller.compute(0, rel_x, time.time())
        vy = -self.y_controller.compute(0, rel_y, time.time())

        descent_rates = {
            'APPROACH': 0,
            'ALIGN': 0,
            'DESCEND': -0.3,
            'FINAL': -0.1,
            'TOUCHDOWN': 0
        }
        vz = descent_rates[self.current_phase]

        return vx, vy, vz, True
```

---

## 6.4 안전 및 비상 프로토콜

### 비행 모드 상태 머신

```python
from enum import Enum, auto

class FlightMode(Enum):
    DISARMED = auto()      # 해제
    ARMED = auto()         # 아밍
    TAKEOFF = auto()       # 이륙
    HOVER = auto()         # 호버
    WAYPOINT = auto()      # 웨이포인트
    RETURN_TO_HOME = auto() # 홈 복귀
    LAND = auto()          # 착륙
    EMERGENCY = auto()     # 비상

class FlightStateMachine:
    """
    안전 전환이 있는 비행 모드 상태 머신.
    """

    def __init__(self):
        self.mode = FlightMode.DISARMED
        self.previous_mode = None

        # 유효한 전환 정의
        self.valid_transitions = {
            FlightMode.DISARMED: [FlightMode.ARMED],
            FlightMode.ARMED: [FlightMode.TAKEOFF, FlightMode.DISARMED],
            FlightMode.TAKEOFF: [FlightMode.HOVER, FlightMode.EMERGENCY],
            FlightMode.HOVER: [FlightMode.WAYPOINT, FlightMode.RETURN_TO_HOME,
                              FlightMode.LAND, FlightMode.EMERGENCY],
            FlightMode.WAYPOINT: [FlightMode.HOVER, FlightMode.RETURN_TO_HOME,
                                  FlightMode.EMERGENCY],
            FlightMode.RETURN_TO_HOME: [FlightMode.HOVER, FlightMode.LAND,
                                        FlightMode.EMERGENCY],
            FlightMode.LAND: [FlightMode.DISARMED, FlightMode.EMERGENCY],
            FlightMode.EMERGENCY: [FlightMode.DISARMED]
        }

    def transition(self, new_mode: FlightMode) -> bool:
        """
        새 비행 모드로 전환 시도.

        Returns:
            전환 성공 시 True
        """
        if new_mode in self.valid_transitions.get(self.mode, []):
            self.previous_mode = self.mode
            self.mode = new_mode
            return True

        # 비상은 모든 상태에서 진입 가능
        if new_mode == FlightMode.EMERGENCY:
            self.previous_mode = self.mode
            self.mode = new_mode
            return True

        return False
```

### 비상 절차

```python
class EmergencyHandler:
    """
    비행 중 비상 상황 처리.
    """

    def __init__(self, drone_controller, home_position):
        self.controller = drone_controller
        self.home = home_position

    def handle_gps_loss(self):
        """GPS 신호 두절 절차."""
        # 1. 광학 흐름/시각 오도메트리로 전환
        self.controller.enable_visual_odometry()

        # 2. 고도를 10m AGL로 낮춤
        self.controller.set_altitude(10)

        # 3. 제자리 호버
        self.controller.hold_position()

        # 4. GPS 복구 대기 (60초 타임아웃)
        start_time = time.time()
        while time.time() - start_time < 60:
            if self.controller.gps_available():
                self.controller.resume_mission()
                return True
            time.sleep(1)

        # 5. 복구 안되면 비상 착륙
        self.controller.emergency_land()
        return False

    def handle_low_battery(self, battery_level: int):
        """저배터리 절차."""
        if battery_level <= 30 and battery_level > 20:
            # 경고 - RTH 제안
            self.controller.send_warning("LOW_BATTERY_WARNING")

        elif battery_level <= 20 and battery_level > 10:
            # 자동 RTH
            self.controller.send_warning("LOW_BATTERY_RTH")
            self.controller.return_to_home()

        elif battery_level <= 10 and battery_level > 5:
            # 가장 가까운 안전 위치에 비상 착륙
            safe_site = self.find_nearest_safe_landing()
            self.controller.land_at(safe_site)

        elif battery_level <= 5:
            # 즉시 비상 착륙
            self.controller.emergency_land()

    def handle_motor_failure(self, failed_motor: int, motor_count: int):
        """모터 고장 절차."""
        if motor_count >= 6:  # 헥사콥터 이상
            # 단일 모터 고장 보상 가능
            self.controller.disable_motor(failed_motor)
            self.controller.reconfigure_mixer()
            self.controller.reduce_performance()

            # 비상 착륙으로 이동
            safe_site = self.find_nearest_safe_landing()
            self.controller.land_at(safe_site)
        else:
            # 쿼드콥터 - 보상 불가
            if self.controller.has_parachute():
                self.controller.deploy_parachute()
            else:
                # 제어된 추락 시도
                self.controller.cut_motors()
                self.controller.broadcast_emergency()

    def handle_communication_loss(self):
        """통신 두절 절차."""
        # 1. 현재 미션 10초 계속
        time.sleep(10)

        # 2. 통신 복구 확인
        if self.controller.communication_available():
            return

        # 3. 복귀 중단 실행
        self.controller.climb_to_safe_altitude()
        self.controller.return_via_preplanned_route()
```

---

## 한국 운영 특화 사항

### K-드론 시스템 연동 프로토콜

```python
class KDroneSystemIntegration:
    """K-드론 시스템과의 연동 프로토콜."""

    def __init__(self, registration_number: str):
        self.registration = registration_number
        self.kdrone_api = "https://api.kdrone.go.kr/v1"

    async def submit_flight_plan(self, flight_plan: dict) -> dict:
        """K-드론 시스템에 비행계획 제출."""
        return await self._post("/flight-plans", {
            "registration": self.registration,
            "plan": flight_plan,
            "purpose": "DELIVERY"
        })

    async def report_position(self, position: dict):
        """실시간 위치 보고 (1초 간격)."""
        await self._post("/tracking", {
            "registration": self.registration,
            "position": position,
            "timestamp": datetime.utcnow().isoformat()
        })
```

### 한국 공역 고려사항

| 구역 | 제한 | 조치 |
|------|------|------|
| P-73 (청와대) | 비행 금지 | 절대 진입 금지 |
| 공항 반경 9.3km | 허가 필요 | 사전 승인 |
| 군 훈련구역 | 시간별 제한 | 동적 회피 |
| 국립공원 | 환경 보호 | 대체 경로 |

---

## 장 요약

비행 제어 및 항법은 배송 드론 운영의 기술적 핵심입니다. 비행 역학(양력, 추력, 항력, 전력 요구사항)을 이해하면 적절한 시스템 설계와 성능 예측이 가능합니다. 안정화 시스템은 모든 조건에서 안정적인 비행을 유지하기 위해 신중하게 튜닝된 캐스케이드 PID 컨트롤러를 사용합니다.

확장 칼만 필터링을 통한 센서 융합은 IMU, GPS, 기압계 데이터를 결합하여 정확한 상태 추정을 제공합니다. A*와 같은 경로 계획 알고리즘은 최적 경로를 찾고, 동적 윈도우 접근법은 실시간 장애물 회피를 가능하게 합니다. 시각 서보잉을 사용한 정밀 착륙은 정확한 패키지 배송을 보장합니다.

안전 시스템은 GPS 두절, 저배터리, 모터 고장, 통신 두절에 대한 포괄적인 비상 절차를 구현하여 고장 모드에서도 드론이 예측 가능하고 안전하게 동작하도록 보장합니다.

---

## 핵심 요약

1. **호버 전력은 질량^1.5에 비례**하여 페이로드 효율이 중요
2. **캐스케이드 PID 제어**로 자세와 위치 조절 분리
3. **확장 칼만 필터**로 다중 센서 융합하여 견고한 추정
4. **고도 페널티가 있는 A* 경로 계획**으로 에너지 소비 최적화
5. **비상 절차**는 모든 신뢰할 수 있는 고장 모드 처리 필수

---

## 복습 문제

1. 6x 18" 프로펠러를 가진 12kg 드론의 호버 전력을 계산하시오.
2. 롤/피치 제어의 일반적인 PID 게인은 무엇이며, 왜 요와 다릅니까?
3. EKF에서 자력계에 대한 측정 업데이트 단계를 구현하시오.
4. 동적 윈도우 접근법이 장애물 회피에서 A*와 어떻게 다릅니까?
5. 낙하산 전개 결정 로직을 위한 상태 머신을 설계하시오.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · 널리 인간을 이롭게 하라
