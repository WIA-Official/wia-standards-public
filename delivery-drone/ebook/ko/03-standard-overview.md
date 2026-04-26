# 제3장: 표준 아키텍처 및 프레임워크

## WIA-AUTO-017 설계 철학, 분류 체계 및 시스템 요구사항

---

## 3.1 WIA 표준 설계 철학

### 핵심 원칙

WIA-AUTO-017 표준은 모든 기술적 결정을 안내하는 5가지 기본 원칙을 기반으로 구축되었습니다:

**1. 안전 최우선**

모든 사양은 효율성이나 비용보다 안전을 우선시합니다:
- 중요 기능에 대한 이중화 시스템
- 모든 고장 모드에서의 페일세이프 동작
- 보수적인 운영 한계
- 포괄적인 비상 절차

**2. 상호운용성**

서로 다른 제조업체의 시스템이 함께 작동해야 합니다:
- 공통 데이터 형식 및 API
- 표준 통신 프로토콜
- 호환 가능한 UTM 통합
- 공유된 안전 규약

**3. 확장성**

표준은 단일 드론 운영자부터 대륙 규모 함대까지 지원합니다:
- 모듈형 아키텍처
- 계층적 관리
- 성능 등급
- 성장 지향적 설계

**4. 접근성**

弘益人間의 철학은 모두에게 이로운 기술을 요구합니다:
- 라이선스 비용 없는 개방형 표준
- 참조 구현 제공
- 다양한 운영 환경 지원
- 농촌 및 소외 지역 고려

**5. 지속가능성**

환경적 책임이 전반에 걸쳐 통합되어 있습니다:
- 에너지 효율 최적화
- 배터리 수명 주기 관리
- 소음 최소화
- 폐기물 재활용 지침

### 표준 구조

```
WIA-AUTO-017 표준 아키텍처:

┌────────────────────────────────────────────────────────────────┐
│                     애플리케이션 레이어                         │
│  (배송 관리, 함대 운영, 고객 인터페이스)                        │
├────────────────────────────────────────────────────────────────┤
│                     API 레이어 (Phase 2)                       │
│  (REST API, WebSocket, SDK)                                    │
├────────────────────────────────────────────────────────────────┤
│                     프로토콜 레이어 (Phase 3)                   │
│  (비행 제어, 항법, 통신)                                       │
├────────────────────────────────────────────────────────────────┤
│                     데이터 형식 레이어 (Phase 1)                │
│  (메시지, 웨이포인트, 원격측정, 비행 로그)                      │
├────────────────────────────────────────────────────────────────┤
│                     통합 레이어 (Phase 4)                       │
│  (UTM, 규제, 지상 시스템)                                      │
└────────────────────────────────────────────────────────────────┘
```

---

## 3.2 드론 분류 시스템

### 중량 기반 분류

표준은 최대이륙중량(MTOW)을 기준으로 4가지 중량 등급을 정의합니다:

#### 마이크로 등급 (0-2 kg)

```yaml
micro_class:
  mtow_range: "0-2 kg"
  payload_capacity: "0.1-0.5 kg"
  typical_range: "1-3 km"
  flight_time: "10-20분"
  max_speed: "15 m/s"
  use_cases:
    - 문서 및 소형 우편물
    - 경량 의약품
    - 열쇠 및 소형 물품
    - 도심 단거리 배송
  regulatory_category: "저위험"
  certification: "대부분 관할권에서 자기 선언"
```

**기술 사양**:

| 매개변수 | 최소 | 일반 | 최대 |
|----------|------|------|------|
| 모터 수 | 4 | 4 | 6 |
| 모터 출력 | 30W | 50W | 100W |
| 배터리 용량 | 1,000 mAh | 2,500 mAh | 4,000 mAh |
| GPS 정확도 | 3m | 1m | 0.5m |
| 풍속 허용 | 5 m/s | 8 m/s | 10 m/s |

#### 경량 등급 (2-10 kg)

```yaml
light_class:
  mtow_range: "2-10 kg"
  payload_capacity: "0.5-3 kg"
  typical_range: "3-10 km"
  flight_time: "20-35분"
  max_speed: "20 m/s"
  use_cases:
    - 전자상거래 패키지
    - 음식 배달
    - 약국 배송
    - 소형 전자제품
  regulatory_category: "중위험"
  certification: "특별 인가 필요"
```

**기술 사양**:

| 매개변수 | 최소 | 일반 | 최대 |
|----------|------|------|------|
| 모터 수 | 4 | 6 | 8 |
| 모터 출력 | 100W | 200W | 400W |
| 배터리 용량 | 5,000 mAh | 10,000 mAh | 16,000 mAh |
| GPS 정확도 | 2m | 0.5m | 0.1m (RTK) |
| 풍속 허용 | 8 m/s | 10 m/s | 12 m/s |

#### 중형 등급 (10-25 kg)

```yaml
medium_class:
  mtow_range: "10-25 kg"
  payload_capacity: "3-8 kg"
  typical_range: "10-30 km"
  flight_time: "30-50분"
  max_speed: "25 m/s"
  use_cases:
    - 대형 패키지
    - 식료품 배송
    - 의료 물자
    - 비상 장비
  regulatory_category: "고위험"
  certification: "완전 형식 인증 권장"
```

**기술 사양**:

| 매개변수 | 최소 | 일반 | 최대 |
|----------|------|------|------|
| 모터 수 | 6 | 8 | 8 |
| 모터 출력 | 400W | 600W | 1000W |
| 배터리 용량 | 16,000 mAh | 30,000 mAh | 50,000 mAh |
| GPS 정확도 | 1m | 0.2m | 0.05m (RTK) |
| 풍속 허용 | 10 m/s | 12 m/s | 15 m/s |

#### 대형 등급 (25-150 kg)

```yaml
heavy_class:
  mtow_range: "25-150 kg"
  payload_capacity: "8-50 kg"
  typical_range: "30-100 km"
  flight_time: "45-90분"
  max_speed: "30 m/s"
  use_cases:
    - 대형 화물
    - 재난 구호 물자
    - 농촌 지역 배송
    - 산업 장비
  regulatory_category: "최고 위험"
  certification: "완전 형식 및 생산 인증 필수"
  parachute: "필수"
```

### 능력 등급

중량 외에도 드론은 능력에 따라 분류됩니다:

| 등급 | 자율성 | BVLOS | 기상 | 야간 |
|------|--------|-------|------|------|
| 기본 | 수동/보조 | 불가 | 맑음 전용 | 불가 |
| 표준 | 웨이포인트 항법 | 제한적 | 약한 조건 | 조명 있으면 가능 |
| 고급 | 완전 자율 | 가능 | 보통 조건 | 완전 가능 |
| 엔터프라이즈 | 다중 기체 조정 | 가능 | 악조건 | 완전 가능 |

---

## 3.3 추진 시스템

### 멀티로터 구성

```
쿼드콥터 (4개 모터):         헥사콥터 (6개 모터):
     M1    M2                    M1      M2
       \  /                        \    /
        \/                          \  /
        /\                      M6---()---M3
       /  \                         /  \
     M3    M4                      /    \
                                 M5      M4

이중화: 없음                   이중화: 1개 모터 고장 시 비행 가능
적합: 마이크로/경량 등급       적합: 경량/중형 등급


옥토콥터 (8개 모터):          동축 옥토 (8개 모터, 4개 암):
  M1    M2                           M1/M5
    \  /                               |
 M8--\/--M3                        M8/M4---M2/M6
    /\                                 |
  M7    M4                           M7/M3
    \  /
 M6--\/--M5

이중화: 2개 모터 고장 시 비행 가능
적합: 중형/대형 등급
```

### 모터 사양

```python
class MotorSpecification:
    """배송 드론용 표준 모터 사양."""

    def __init__(self, kv_rating, max_current, efficiency):
        self.kv = kv_rating  # 볼트당 RPM
        self.max_current = max_current  # 암페어
        self.efficiency = efficiency  # 0-1

    def calculate_thrust(self, voltage, propeller_pitch, propeller_diameter):
        """
        모터와 프로펠러 사양을 기반으로 추력 추정.
        계획 목적의 단순화 모델.
        """
        rpm = self.kv * voltage
        # 추력 계수 근사
        ct = 0.012 * (propeller_pitch / propeller_diameter)
        # 해수면 공기 밀도
        rho = 1.225
        # 프로펠러 디스크 면적
        area = 3.14159 * (propeller_diameter / 2) ** 2

        # 단순화된 추력 계산
        thrust = ct * rho * area * (rpm / 60 * propeller_pitch) ** 2

        return thrust * self.efficiency

# 예시: 일반적인 중형 등급 모터
motor = MotorSpecification(kv_rating=320, max_current=40, efficiency=0.85)
thrust = motor.calculate_thrust(voltage=50, propeller_pitch=0.15, propeller_diameter=0.38)
print(f"추정 추력: {thrust:.1f} N")
```

### 추진 시스템 요구사항

| 매개변수 | 마이크로 | 경량 | 중형 | 대형 |
|----------|----------|------|------|------|
| 추력/중량 비율 | >1.5 | >1.8 | >2.0 | >2.2 |
| 모터 이중화 | 선택 | 권장 | 필수 | 필수 |
| ESC 이중화 | 불필요 | 선택 | 권장 | 필수 |
| 프로펠러 타입 | 고정 | 접이식 | 접이식 | 접이식 |

---

## 3.4 전력 시스템

### 배터리 사양

#### 리튬 폴리머 (LiPo) 요구사항

```yaml
battery_requirements:
  chemistry: "리튬 폴리머 (LiPo)"
  nominal_voltage: "셀당 3.7V"
  max_charge_voltage: "셀당 4.2V"
  min_discharge_voltage: "셀당 3.3V"
  c_rating_minimum: "연속 방전 10C"
  temperature_range:
    storage: "-20°C ~ 45°C"
    charge: "5°C ~ 45°C"
    discharge: "0°C ~ 55°C"
  cycle_life: "80% 용량까지 >300 사이클"
  protection:
    - 과전류
    - 과전압
    - 저전압
    - 과온도
    - 단락
```

#### 배터리 용량 산정

```python
def size_battery(flight_time_min, hover_power_w, cruise_power_w,
                 hover_ratio=0.3, reserve_ratio=0.25):
    """
    배송 미션에 필요한 배터리 용량 계산.

    Args:
        flight_time_min: 목표 비행 시간 (분)
        hover_power_w: 호버링 시 전력 소비 (W)
        cruise_power_w: 순항 시 전력 소비 (W)
        hover_ratio: 호버링 비율 (0-1)
        reserve_ratio: 안전 예비 (0-1)
    """
    # 평균 전력 소비
    avg_power = hover_ratio * hover_power_w + (1 - hover_ratio) * cruise_power_w

    # 필요 에너지 (Wh)
    energy_required = avg_power * (flight_time_min / 60)

    # 예비 추가
    total_energy = energy_required / (1 - reserve_ratio)

    # 일반적인 배터리 전압 (6S = 22.2V 공칭)
    voltage = 22.2

    # 필요 용량 (Ah)
    capacity_ah = total_energy / voltage

    return {
        "energy_wh": total_energy,
        "capacity_ah": capacity_ah,
        "capacity_mah": capacity_ah * 1000
    }

# 예시: 경량 등급 드론
result = size_battery(
    flight_time_min=30,
    hover_power_w=600,
    cruise_power_w=400,
    hover_ratio=0.3,
    reserve_ratio=0.25
)
print(f"필요 배터리: {result['capacity_mah']:.0f} mAh ({result['energy_wh']:.0f} Wh)")
```

### 전력 분배

```
전력 분배 아키텍처:

        ┌─────────────────────────────────────────────┐
        │              배터리 팩                        │
        │         (22.2V 6S / 44.4V 12S)              │
        └────────────────────┬────────────────────────┘
                             │
                    ┌────────┴────────┐
                    │  메인 전원 버스  │
                    └────────┬────────┘
                             │
        ┌────────────────────┼────────────────────┐
        │                    │                    │
   ┌────┴────┐         ┌─────┴─────┐        ┌────┴────┐
   │  모터   │         │  5V/12V   │        │  이중화  │
   │  ESC    │         │  레귤레이터│        │  전원   │
   └────┬────┘         └─────┬─────┘        └────┬────┘
        │                    │                    │
   M1 M2 M3...         ┌─────┴─────┐        ┌────┴────┐
                       │           │        │ 비행    │
                  ┌────┴────┐ ┌────┴────┐   │컨트롤러 │
                  │  센서   │ │ 페이로드│   │ (백업)  │
                  │GPS,IMU  │ │  해제   │   └─────────┘
                  └─────────┘ └─────────┘
```

---

## 3.5 센서 세트

### 필수 센서

| 센서 | 목적 | 정확도 | 업데이트 율 |
|------|------|--------|-------------|
| IMU (6축) | 자세 추정 | ±0.5° | 400+ Hz |
| GNSS 수신기 | 위치 | <2m (단독) | 5-10 Hz |
| 기압계 | 고도 | ±1m | 50 Hz |
| 자력계 | 방위 | ±2° | 100 Hz |

### 권장 센서

| 센서 | 목적 | 범위 | 비고 |
|------|------|------|------|
| RTK GNSS | 정밀 위치 | N/A | <2cm 정확도 |
| LiDAR | 장애물 감지 | 30-100m | 단일 또는 다중 빔 |
| 스테레오 카메라 | 시각 항법 | 10-30m | 깊이 인식 |
| 초음파 | 지면 근접 | 0.5-5m | 착륙 보조 |
| 광학 흐름 | 속도 추정 | 0.5-5m | GPS 백업 |
| ADS-B 수신기 | 항공기 감지 | N/A | 충돌 회피 |

### 센서 융합 아키텍처

```python
class SensorFusion:
    """
    드론 상태 추정을 위한 확장 칼만 필터.
    GPS, IMU, 기압계 데이터 융합.
    """

    def __init__(self):
        # 상태 벡터: [x, y, z, vx, vy, vz, roll, pitch, yaw]
        self.state = np.zeros(9)
        self.covariance = np.eye(9) * 0.1

    def predict(self, imu_data, dt):
        """
        IMU 데이터를 사용한 예측 단계.
        """
        # 가속도 및 각속도 추출
        accel = imu_data['acceleration']
        gyro = imu_data['angular_velocity']

        # 자세 업데이트
        self.state[6:9] += gyro * dt

        # 가속도를 세계 좌표계로 회전
        R = self._rotation_matrix(self.state[6], self.state[7], self.state[8])
        world_accel = R @ accel - np.array([0, 0, 9.81])

        # 속도 업데이트
        self.state[3:6] += world_accel * dt

        # 위치 업데이트
        self.state[0:3] += self.state[3:6] * dt

        # 공분산 업데이트
        self._update_covariance_predict(dt)

    def update_gps(self, gps_data):
        """
        GPS 위치를 사용한 업데이트 단계.
        """
        # 측정: [lat, lon, alt] -> [x, y, z]
        measurement = self._gps_to_local(gps_data)

        # 칼만 이득 계산
        H = np.zeros((3, 9))
        H[0, 0] = H[1, 1] = H[2, 2] = 1

        R = np.diag([gps_data['hdop'] * 2, gps_data['hdop'] * 2, gps_data['vdop'] * 3])

        self._kalman_update(measurement, H, R)

    def get_position(self):
        return self.state[0:3]

    def get_velocity(self):
        return self.state[3:6]

    def get_attitude(self):
        return self.state[6:9]
```

---

## 3.6 통신 아키텍처

### 통신 요구사항

| 링크 | 목적 | 범위 | 지연 | 이중화 |
|------|------|------|------|--------|
| 지휘 및 제어 | 비행 제어 | BVLOS | <100ms | 필수 |
| 원격측정 | 상태 보고 | BVLOS | <500ms | 권장 |
| 비디오 | 상황 인식 | BVLOS | <1s | 선택 |
| UTM | 교통 관리 | 전국 | <5s | 필수 |
| Remote ID | 공개 식별 | 1km | <1s | 필수 |

### 통신 스택

```
통신 프로토콜 스택:

┌─────────────────────────────────────────────────────────────┐
│                    애플리케이션 레이어                        │
│  (미션 제어, 원격측정, 명령)                                  │
├─────────────────────────────────────────────────────────────┤
│                    보안 레이어                                │
│  (TLS 1.3, AES-256 암호화, 인증)                             │
├─────────────────────────────────────────────────────────────┤
│                    전송 레이어                                │
│  (명령은 TCP, 원격측정은 UDP, 이벤트는 MQTT)                  │
├─────────────────────────────────────────────────────────────┤
│                    네트워크 레이어                            │
│  (IPv4/IPv6, 모바일 IP)                                      │
├─────────────────────────────────────────────────────────────┤
│                    링크 레이어                                │
│  주: LTE/5G | 백업: 900 MHz | 비상: 위성                     │
└─────────────────────────────────────────────────────────────┘
```

### 링크 버짓 계산

```python
def calculate_link_budget(frequency_mhz, distance_km, tx_power_dbm,
                          tx_antenna_gain_db, rx_antenna_gain_db,
                          rx_sensitivity_dbm):
    """
    통신 링크 버짓 계산.

    Returns:
        링크 마진 (dB) (양수 = 작동, 음수 = 실패)
    """
    import math

    # 자유 공간 경로 손실 (dB)
    fspl = 20 * math.log10(distance_km) + 20 * math.log10(frequency_mhz) + 32.44

    # 추가 손실 (대기, 커넥터 등)
    misc_loss = 3  # dB

    # 총 경로 손실
    total_loss = fspl + misc_loss

    # 수신 전력
    rx_power = tx_power_dbm + tx_antenna_gain_db - total_loss + rx_antenna_gain_db

    # 링크 마진
    margin = rx_power - rx_sensitivity_dbm

    return {
        "fspl_db": fspl,
        "total_loss_db": total_loss,
        "rx_power_dbm": rx_power,
        "margin_db": margin,
        "status": "OK" if margin > 6 else "MARGINAL" if margin > 0 else "FAILED"
    }

# 예시: 10 km에서 900 MHz 라디오 링크
result = calculate_link_budget(
    frequency_mhz=915,
    distance_km=10,
    tx_power_dbm=30,      # 1W 송신기
    tx_antenna_gain_db=3,
    rx_antenna_gain_db=3,
    rx_sensitivity_dbm=-110
)
print(f"링크 마진: {result['margin_db']:.1f} dB ({result['status']})")
```

---

## 3.7 안전 시스템

### 이중화 요구사항

| 시스템 | 마이크로 | 경량 | 중형 | 대형 |
|--------|----------|------|------|------|
| 비행 컨트롤러 | 단일 | 단일 | 이중 | 삼중 |
| IMU | 단일 | 이중 | 삼중 | 삼중 |
| GNSS | 단일 | 이중 | 이중 | 삼중 |
| 모터 (유효) | 4 | 6+ | 6+ | 8+ |
| 전원 | 단일 | 단일 | 이중 | 이중 |
| 통신 | 이중 | 이중 | 삼중 | 삼중 |

### 낙하산 시스템

MTOW 10 kg 초과 드론에 필수:

```yaml
parachute_specification:
  deployment_time: "<2초"
  descent_rate: "최대 중량에서 <5 m/s"
  opening_altitude: "안전 전개를 위해 >30m"
  trigger_conditions:
    - 자세 제어 상실
    - 다중 모터 고장
    - 구조적 고장 감지
    - 조종사 수동 명령
  testing:
    - 인증 전 10회 성공적인 전개
    - 연간 재포장 필요
    - 매 비행 전 기능 테스트
```

### 지오펜싱

```python
class Geofence:
    """
    배송 드론용 지오펜싱 시행.
    """

    PRIORITY_CRITICAL = 1  # 공항, 군사 - 하드 경계
    PRIORITY_HIGH = 2      # 학교, 병원 - 인가 시 허용
    PRIORITY_MEDIUM = 3    # 공원, 경기장 - 시간적
    PRIORITY_LOW = 4       # 주거 지역 - 고도 제한

    def __init__(self):
        self.zones = []

    def add_zone(self, zone):
        self.zones.append(zone)

    def check_position(self, lat, lon, alt):
        """
        위치가 지오펜스를 위반하는지 확인.

        Returns:
            (allowed: bool, action: str, zone: Zone)
        """
        for zone in sorted(self.zones, key=lambda z: z.priority):
            if zone.contains(lat, lon, alt):
                if zone.priority == self.PRIORITY_CRITICAL:
                    return False, "EMERGENCY_LAND", zone
                elif zone.priority == self.PRIORITY_HIGH:
                    if not zone.is_authorized():
                        return False, "RETURN_TO_HOME", zone
                elif zone.priority == self.PRIORITY_MEDIUM:
                    if zone.is_active():
                        return False, "REROUTE", zone
                elif zone.priority == self.PRIORITY_LOW:
                    if alt < zone.min_altitude:
                        return True, "CLIMB", zone

        return True, None, None
```

---

## 한국 특화 사항

### K-드론 시스템 통합

한국에서 배송 드론 운영 시 K-드론 시스템과의 통합이 필수입니다:

| 요구사항 | 설명 |
|----------|------|
| 등록 | 250g 이상 드론 의무 등록 |
| 비행계획 | 사전 비행계획 제출 필수 |
| 실시간 추적 | 비행 중 위치 보고 |
| Remote ID | 브로드캐스트 식별 |

### 국내 주요 배송 드론 업체

| 업체 | 특징 | 적용 등급 |
|------|------|----------|
| 한국항공우주연구원(KARI) | 연구 개발 선도 | 전 등급 |
| 배달의민족 | 음식 배달 시범 | 경량 |
| 쿠팡 | 물류 배송 | 중형 |
| 대한통운 | 도서산간 배송 | 중형/대형 |

---

## 장 요약

WIA-AUTO-017 표준은 안전, 상호운용성, 확장성, 접근성, 지속가능성의 원칙을 기반으로 배송 드론 시스템을 위한 포괄적인 프레임워크를 제공합니다. 분류 시스템은 소형 패키지용 마이크로 등급 드론부터 대형 화물 및 재난 구호용 대형 기체까지 포괄합니다.

추진 시스템은 단순한 쿼드콥터부터 이중화된 옥토콥터 구성까지, 모터 성능과 신뢰성에 대한 명확한 사양을 제공합니다. 전력 시스템은 배터리 안전과 미션 요구사항에 적합한 용량 산정을 강조합니다. 센서 세트는 필수 구성요소(IMU, GNSS, 기압계, 자력계)와 고급 자율성을 위한 권장 추가 장치를 결합합니다.

통신 아키텍처는 적절한 보안과 함께 이중화된 연결성을 제공하며, 이중화 요구사항, 낙하산, 지오펜싱을 포함한 안전 시스템은 모든 조건에서 안정적인 운영을 보장합니다.

---

## 핵심 요약

1. **4가지 중량 등급** (마이크로, 경량, 중형, 대형)은 각각 다른 요구사항을 가짐
2. **이중화는 위험에 비례**: 더 무거운 드론은 더 많은 모터와 이중 비행 컨트롤러 필요
3. **센서 융합**은 IMU, GPS, 기압계를 결합하여 정확한 상태 추정
4. **통신 이중화** (셀룰러 + 라디오 + 위성)로 BVLOS 신뢰성 보장
5. **MTOW 10 kg 초과 드론**에 낙하산 필수

---

## 복습 문제

1. 중형 등급 배송 드론의 최소 추력/중량 비율은?
2. 평균 500W 전력 소비로 25분 비행을 위한 배터리 사양을 설계하시오.
3. 경량 등급 드론에서 필수 센서와 권장 센서는?
4. LTE(1800 MHz)를 사용한 5 km 배송 미션의 링크 버짓을 계산하시오.
5. 각 우선순위 레벨에 대한 지오펜싱 조치 요구사항은?

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · 널리 인간을 이롭게 하라
