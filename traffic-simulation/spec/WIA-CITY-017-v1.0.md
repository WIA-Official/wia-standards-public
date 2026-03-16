# WIA-CITY-017: 교통 시뮬레이션 표준 v1.0 🚦

> **弘益人間 (홍익인간) - 널리 인간을 이롭게 하라**

**표준 ID:** WIA-CITY-017
**버전:** 1.0.0
**발행일:** 2025-12-25
**상태:** 활성
**카테고리:** 스마트 시티 (CITY)

---

## 목차

1. [개요](#1-개요)
2. [적용 범위](#2-적용-범위)
3. [용어 정의](#3-용어-정의)
4. [교통량 분석 및 예측](#4-교통량-분석-및-예측)
5. [신호등 최적화](#5-신호등-최적화)
6. [차량 흐름 시뮬레이션](#6-차량-흐름-시뮬레이션)
7. [보행자 동선 분석](#7-보행자-동선-분석)
8. [대중교통 시뮬레이션](#8-대중교통-시뮬레이션)
9. [교통 혼잡 예측](#9-교통-혼잡-예측)
10. [사고 영향 분석](#10-사고-영향-분석)
11. [교통 정책 시뮬레이션](#11-교통-정책-시뮬레이션)
12. [데이터 모델](#12-데이터-모델)
13. [API 명세](#13-api-명세)
14. [보안 및 개인정보보호](#14-보안-및-개인정보보호)
15. [성과 지표 (KPI)](#15-성과-지표-kpi)

---

## 1. 개요

### 1.1 목적

WIA-CITY-017 교통 시뮬레이션 표준은 도시 교통 시스템의 현재 상태 분석, 미래 예측, 정책 평가를 위한 국제 표준입니다.

교통 시뮬레이션은 실제 도로망에서 차량, 보행자, 대중교통의 움직임을 컴퓨터 모델로 재현하여 교통 혼잡, 사고 위험, 신호 최적화 등을 분석합니다.

### 1.2 핵심 원칙

- **정확성 (Accuracy)**: 실제 교통 상황을 정밀하게 재현
- **확장성 (Scalability)**: 교차로부터 도시 전체까지 유연한 범위
- **실시간성 (Real-time)**: 실시간 교통 데이터 통합
- **예측성 (Predictive)**: AI 기반 교통량 예측
- **정책 평가 (Policy Evaluation)**: 교통 정책의 사전 평가
- **개방형 표준 (Open Standards)**: 다양한 시뮬레이션 엔진 간 호환
- **데이터 주도 (Data-driven)**: 실제 센서 데이터 기반 검증

### 1.3 적용 대상

- 도시 교통 관리 센터
- 교통 정책 입안 기관
- 도로 설계 및 건설 업체
- 자율주행 차량 개발사
- 대중교통 운영 기관
- 스마트 시티 솔루션 제공사
- 교통 연구 기관 및 대학

---

## 2. 적용 범위

### 2.1 시뮬레이션 유형

본 표준은 다음 시뮬레이션 유형에 적용됩니다:

#### 미시적 시뮬레이션 (Microscopic Simulation)
- 개별 차량의 움직임 추적
- 차량 추종 모델 (Car-following model)
- 차선 변경 모델 (Lane-changing model)
- 초 단위 시간 해상도

#### 중시적 시뮬레이션 (Mesoscopic Simulation)
- 차량 그룹 단위 모델링
- 링크-노드 네트워크 기반
- 분 단위 시간 해상도
- 대규모 네트워크에 적합

#### 거시적 시뮬레이션 (Macroscopic Simulation)
- 교통 흐름을 유체로 모델링
- 교통량-밀도-속도 관계
- 시간 단위 해상도
- 전략적 계획에 적합

### 2.2 분석 영역

- **도로 네트워크**: 고속도로, 간선도로, 이면도로
- **교차로**: 신호 교차로, 회전 교차로
- **대중교통**: 버스, 지하철, 트램
- **보행자**: 보행로, 횡단보도
- **주차**: 노상 주차, 주차장
- **특수 구역**: 학교 구역, 상업 지구

### 2.3 시뮬레이션 시나리오

- 현황 분석 (Base scenario)
- 미래 예측 (Future scenario)
- 정책 평가 (Policy scenario)
- 사고 영향 (Incident scenario)
- 이벤트 영향 (Event scenario)
- 최악 상황 (Worst-case scenario)

---

## 3. 용어 정의

### 3.1 교통 측정 단위

| 용어 | 정의 | 단위 |
|------|------|------|
| **교통량 (Volume)** | 단위 시간당 통과 차량 수 | 대/시간 (vph) |
| **밀도 (Density)** | 단위 거리당 차량 수 | 대/km (vpk) |
| **속도 (Speed)** | 차량 이동 속도 | km/h |
| **점유율 (Occupancy)** | 차량이 차지하는 시간 비율 | % |
| **지체 (Delay)** | 자유 흐름 대비 추가 시간 | 초/대 |
| **큐 길이 (Queue Length)** | 대기 차량 줄의 길이 | m 또는 대 |
| **서비스 수준 (LOS)** | 교통 흐름의 질적 평가 | A-F |

### 3.2 시뮬레이션 용어

| 용어 | 정의 |
|------|------|
| **Time Step** | 시뮬레이션 시간 간격 (0.1-1초) |
| **Warm-up Period** | 초기 안정화 기간 (15-30분) |
| **Seed** | 난수 생성 시드 값 |
| **Calibration** | 실제 데이터와 일치하도록 조정 |
| **Validation** | 독립 데이터셋으로 검증 |
| **GEH Statistic** | 교통량 비교 통계량 |

### 3.3 차량 유형

- **승용차 (Passenger Car)**: PCU = 1.0
- **버스 (Bus)**: PCU = 2.0-3.0
- **트럭 (Truck)**: PCU = 1.5-2.5
- **오토바이 (Motorcycle)**: PCU = 0.3-0.5
- **자전거 (Bicycle)**: PCU = 0.2

---

## 4. 교통량 분석 및 예측

### 4.1 교통량 수집

#### 검지기 유형

| 검지기 | 측정 항목 | 정확도 | 설치 위치 |
|--------|----------|--------|-----------|
| 루프 검지기 (Loop Detector) | 교통량, 속도, 점유율 | 95-98% | 도로 매설 |
| 영상 검지기 (Video Detector) | 교통량, 속도, 차종 | 90-95% | 상부 구조물 |
| 레이더 검지기 (Radar Detector) | 속도, 거리 | 95-98% | 노변 |
| 블루투스 (Bluetooth) | 통행 시간 | 85-90% | 노변 |
| GPS 프로브 (Probe Vehicle) | 속도, 경로 | 80-90% | 차량 탑재 |

#### 데이터 수집 주기

```json
{
  "realtime": {
    "interval": "1-5 minutes",
    "granularity": "lane-level",
    "coverage": "critical corridors"
  },
  "historical": {
    "interval": "15 minutes",
    "granularity": "link-level",
    "coverage": "entire network"
  },
  "origin_destination": {
    "interval": "daily/weekly",
    "method": "bluetooth/license_plate",
    "sample_rate": "5-20%"
  }
}
```

### 4.2 교통량 예측

#### 단기 예측 (Short-term Forecasting)

**시간 범위**: 5분 - 1시간

**방법론**:
- ARIMA (자기회귀 이동평균)
- Kalman Filter
- Neural Networks (LSTM, GRU)
- Support Vector Regression

**입력 데이터**:
- 과거 교통량 (15분-2시간)
- 요일/시간대
- 날씨 정보
- 이벤트 정보

**정확도 목표**:
- MAPE (Mean Absolute Percentage Error) < 10%
- R² > 0.85

#### 중기 예측 (Medium-term Forecasting)

**시간 범위**: 1시간 - 24시간

**방법론**:
- Time-series Analysis
- Machine Learning (Random Forest, XGBoost)
- Deep Learning (CNN-LSTM)

**입력 데이터**:
- 과거 패턴 (동일 요일/시간대)
- 기상 예보
- 이벤트 일정
- 공휴일 정보

#### 장기 예측 (Long-term Forecasting)

**시간 범위**: 1년 - 20년

**방법론**:
- 4-Step Model (통행 생성, 분포, 수단 선택, 경로 배정)
- Activity-based Model
- Land-use Transport Interaction Model

**입력 데이터**:
- 인구 통계 변화
- 토지 이용 계획
- 교통 인프라 개발 계획
- 경제 성장률

### 4.3 O-D 매트릭스 추정

**기원-목적지 (Origin-Destination) 매트릭스**:

```
        | Zone 1 | Zone 2 | Zone 3 | ... | Zone N |
--------|--------|--------|--------|-----|--------|
Zone 1  |   0    |  150   |   80   | ... |   30   |
Zone 2  |  120   |   0    |  200   | ... |   50   |
Zone 3  |   70   |  180   |   0    | ... |   40   |
...     |  ...   |  ...   |  ...   | ... |  ...   |
Zone N  |   25   |   45   |   35   | ... |   0    |
```

**추정 방법**:
1. **직접 조사**: 가구 통행 조사, 노변 인터뷰
2. **간접 추정**: 교통량 기반 역산 (Matrix Estimation)
3. **프로브 데이터**: GPS, 통신사 데이터

**검증 기준**:
- Link count vs. Observed: GEH < 5 for 85% of links
- Screen line flow accuracy: ±5%

---

## 5. 신호등 최적화

### 5.1 신호 제어 유형

#### 고정 신호 (Fixed-time Signal)

**특징**:
- 사전 설정된 고정 주기
- 시간대별 신호 시간표 (TOD, Time of Day)
- 간단하고 예측 가능

**최적화 방법**:
- Webster's Method
- Synchro/SimTraffic
- TRANSYT

**신호 파라미터**:
```json
{
  "cycle_length": 90,        // 초
  "green_time": {
    "phase_1": 35,           // 남-북 직진
    "phase_2": 25,           // 동-서 직진
    "phase_3": 15,           // 좌회전
    "phase_4": 10            // 보행자
  },
  "amber_time": 3,           // 초
  "all_red_time": 2,         // 초
  "offset": 10               // 초 (상류 교차로 대비)
}
```

#### 감응 신호 (Actuated Signal)

**특징**:
- 차량 검지기 기반 실시간 조정
- 최소/최대 녹색 시간 설정
- 교통량 변동에 적응

**제어 로직**:
1. **Vehicle Detection**: 차량 도착 감지
2. **Gap-out**: 차량 간격 > Threshold → 신호 변경
3. **Max-out**: 최대 녹색 시간 도달 → 강제 변경

#### 적응 신호 (Adaptive Signal)

**특징**:
- 실시간 교통 패턴 학습
- 네트워크 전체 최적화
- AI/ML 기반 예측 제어

**시스템 예시**:
- SCATS (Sydney Coordinated Adaptive Traffic System)
- SCOOT (Split Cycle Offset Optimization Technique)
- InSync
- Surtrac

**최적화 목표**:
```python
minimize:
  Total_Delay = Σ (Vehicle_Delay + Pedestrian_Delay + Bicycle_Delay)

subject to:
  - Minimum Green Time ≥ 7 seconds
  - Maximum Cycle Length ≤ 150 seconds
  - Pedestrian Crossing Time ≥ (Crossing_Distance / 1.2 m/s)
  - Queue Length < Storage Capacity
```

### 5.2 신호 협조 (Signal Coordination)

**Green Wave (녹색 물결)**:

```
Distance (m)
  ↑
  |  Traffic Signal 3 ━━━━━━━━━━━━●━━━━
  |                            ╱
  |  Traffic Signal 2 ━━━━━━●━━━━━━━━━━━
  |                     ╱
  |  Traffic Signal 1 ●━━━━━━━━━━━━━━━━━
  |__________________|___|___|___|___|___→ Time (s)
                     0  30  60  90 120
```

**Offset 계산**:
```
Offset = (Distance / Speed) mod Cycle_Length

예시:
  Distance = 400m
  Speed = 50 km/h = 13.89 m/s
  Cycle = 90s

  Offset = (400 / 13.89) mod 90
        = 28.8 mod 90
        ≈ 29 seconds
```

### 5.3 성능 지표

| 지표 | 정의 | 목표 |
|------|------|------|
| **Average Delay** | 차량당 평균 지체 | < 30초 |
| **Queue Length** | 최대 대기 차량 수 | < 도로 저장 용량 |
| **Level of Service** | 서비스 수준 | LOS C 이상 |
| **Stops per Vehicle** | 차량당 정지 횟수 | < 1.5회 |
| **Fuel Consumption** | 연료 소비량 | -20% (최적화 후) |
| **Emissions** | CO2 배출량 | -15% (최적화 후) |

---

## 6. 차량 흐름 시뮬레이션

### 6.1 차량 추종 모델 (Car-following Model)

#### Wiedemann Model (미시적)

**기본 방정식**:
```
a(t) = f(Δv, Δx, v)

where:
  a(t) = acceleration at time t
  Δv = speed difference (leader - follower)
  Δx = spacing (distance between vehicles)
  v = follower's speed
```

**상태 변화**:
1. **Free Driving**: 충분한 간격, 원하는 속도 유지
2. **Approaching**: 선행 차량에 접근
3. **Following**: 선행 차량 속도 추종
4. **Braking**: 급감속

#### Gipps Model

**안전 거리 기반**:
```python
def gipps_model(v_n, v_n_minus_1, x_n, x_n_minus_1, tau):
    """
    v_n: 추종 차량 속도
    v_n_minus_1: 선행 차량 속도
    x_n: 추종 차량 위치
    x_n_minus_1: 선행 차량 위치
    tau: 반응 시간
    """
    # Free flow acceleration
    v_free = v_n + 2.5 * a_max * tau * (1 - v_n / v_desired)

    # Safe speed
    b = -b_max  # comfortable deceleration
    s = x_n_minus_1 - x_n - L  # spacing
    v_safe = b * tau + sqrt((b * tau)**2 - b * (2 * s - v_n * tau - v_n_minus_1**2 / b_hat))

    return min(v_free, v_safe)
```

#### Intelligent Driver Model (IDM)

**가속도 함수**:
```
a = a_max * [1 - (v/v_0)^δ - (s*/s)²]

s* = s_0 + v*T + (v*Δv) / (2*√(a_max * b))

where:
  a_max = 최대 가속도 (1.5 m/s²)
  v_0 = 원하는 속도 (120 km/h)
  δ = 가속 지수 (4)
  s_0 = 최소 간격 (2m)
  T = 안전 시간 간격 (1.5s)
  b = 편안한 감속도 (2 m/s²)
  Δv = 상대 속도
  s = 실제 간격
```

### 6.2 차선 변경 모델 (Lane-changing Model)

#### MOBIL (Minimizing Overall Braking Induced by Lane changes)

**차선 변경 조건**:
```
1. Safety Criterion (안전성):
   a_n_new (new follower's acceleration) > -b_safe

2. Incentive Criterion (유인성):
   a_c_new - a_c_old + p * (a_n_new - a_n_old + a_o_new - a_o_old) > a_threshold

where:
  a_c = changing vehicle's acceleration
  a_n = new follower's acceleration
  a_o = old follower's acceleration
  p = politeness factor (0-1)
  a_threshold = lane-changing threshold
  b_safe = safe deceleration
```

#### 차선 변경 유형

1. **Mandatory Lane Change (필수)**:
   - 회전 또는 진출을 위한 차선 변경
   - 도로 종료 전 차선 변경
   - 높은 우선순위

2. **Discretionary Lane Change (선택)**:
   - 더 빠른 속도를 위한 차선 변경
   - 느린 차량 추월
   - 낮은 우선순위

### 6.3 교통 흐름 이론

#### 기본 관계식 (Fundamental Diagram)

```
q = k * v

where:
  q = flow (veh/h)
  k = density (veh/km)
  v = speed (km/h)
```

**Greenshields Model**:
```
v = v_f * (1 - k/k_j)

where:
  v_f = free-flow speed (120 km/h)
  k_j = jam density (140 veh/km)
```

**임계 값**:
- **Critical Density (k_c)**: 최대 교통량 발생 밀도
- **Critical Speed (v_c)**: 최대 교통량 발생 속도
- **Capacity (q_max)**: 최대 교통량

```
k_c = k_j / 2 = 70 veh/km
v_c = v_f / 2 = 60 km/h
q_max = k_c * v_c = 4,200 veh/h
```

#### 서비스 수준 (Level of Service)

| LOS | 밀도 (veh/km/lane) | v/c 비 | 설명 |
|-----|-------------------|--------|------|
| A | 0-7 | 0.00-0.35 | 자유 흐름 |
| B | 7-11 | 0.35-0.54 | 안정적 흐름 |
| C | 11-16 | 0.54-0.77 | 안정적 흐름 (제한적) |
| D | 16-22 | 0.77-0.90 | 불안정 흐름 접근 |
| E | 22-28 | 0.90-1.00 | 불안정 흐름 |
| F | >28 | >1.00 | 정체 (breakdown) |

---

## 7. 보행자 동선 분석

### 7.1 보행자 행동 모델

#### Social Force Model (사회력 모델)

**기본 방정식**:
```
m_i * dv_i/dt = F_i^0 + Σ F_ij + Σ F_iW + ξ_i

where:
  F_i^0 = desired force (목적지로의 힘)
  F_ij = repulsive force from other pedestrians
  F_iW = repulsive force from walls/obstacles
  ξ_i = random fluctuation
```

**Desired Force**:
```
F_i^0 = (v_i^0 * e_i^0 - v_i) / τ_i

where:
  v_i^0 = desired speed (1.34 m/s average)
  e_i^0 = desired direction
  v_i = current velocity
  τ_i = relaxation time (0.5s)
```

**Repulsive Force (from pedestrians)**:
```
F_ij = A_i * exp((r_ij - d_ij) / B_i) * n_ij

where:
  A_i = interaction strength (2000 N)
  B_i = range parameter (0.08 m)
  r_ij = sum of radii
  d_ij = distance between centers
  n_ij = normalized direction
```

### 7.2 보행자 속도

| 상황 | 평균 속도 | 범위 |
|------|----------|------|
| 자유 보행 | 1.34 m/s | 0.8-1.8 m/s |
| 혼잡 | 0.7 m/s | 0.3-1.0 m/s |
| 계단 상행 | 0.6 m/s | 0.4-0.8 m/s |
| 계단 하행 | 0.7 m/s | 0.5-0.9 m/s |
| 노인 | 0.9 m/s | 0.5-1.2 m/s |
| 장애인 | 0.7 m/s | 0.3-1.0 m/s |

### 7.3 횡단보도 시뮬레이션

**횡단 시간 계산**:
```
T_crossing = W / v_p + T_startup

where:
  W = 횡단보도 폭 (m)
  v_p = 보행 속도 (1.2 m/s - 안전마진 포함)
  T_startup = 출발 지연 (3-4초)
```

**보행자 신호 시간**:
```json
{
  "pedestrian_signal": {
    "walk": 7,                    // 초 (녹색 보행자)
    "flashing_dont_walk": 12,     // 초 (깜빡이는 적색 보행자)
    "dont_walk": 70,              // 초 (적색 보행자)
    "total_cycle": 90             // 초
  },
  "clearance_interval": {
    "formula": "crossing_distance / 1.0",  // m/s
    "minimum": 4                            // 초
  }
}
```

### 7.4 보행자 밀도 및 LOS

| LOS | 밀도 (ped/m²) | 공간 (m²/ped) | 흐름 특성 |
|-----|--------------|--------------|----------|
| A | 0.00-0.31 | >3.25 | 자유 선택 속도 및 경로 |
| B | 0.31-0.43 | 2.30-3.25 | 정상 보행 속도 |
| C | 0.43-0.72 | 1.39-2.30 | 제한적 속도 |
| D | 0.72-1.08 | 0.93-1.39 | 느린 속도, 자주 충돌 |
| E | 1.08-2.17 | 0.46-0.93 | 매우 느림, 빈번한 접촉 |
| F | >2.17 | <0.46 | 정체, 이동 불가 |

---

## 8. 대중교통 시뮬레이션

### 8.1 버스 시뮬레이션

#### 버스 정류장 모델

**정차 시간**:
```
T_dwell = T_open + n_board * t_board + n_alight * t_alight + T_close

where:
  T_open = 문 열림 시간 (2초)
  n_board = 승차 인원
  t_board = 인당 승차 시간 (2-3초)
  n_alight = 하차 인원
  t_alight = 인당 하차 시간 (1-2초)
  T_close = 문 닫힘 시간 (2초)
```

**버스 연착 모델**:
```python
def bus_delay(schedule, actual, factors):
    """
    버스 연착 계산
    """
    delay = actual - schedule

    # 지체 요인
    traffic_delay = factors['congestion_level'] * factors['link_length'] / factors['avg_speed']
    signal_delay = factors['red_signals'] * factors['avg_red_time']
    passenger_delay = factors['passenger_count'] * factors['avg_boarding_time']

    total_delay = traffic_delay + signal_delay + passenger_delay

    return {
        'total_delay': total_delay,
        'on_time_performance': 1 if delay < 300 else 0,  # 5분 기준
        'breakdown': {
            'traffic': traffic_delay,
            'signal': signal_delay,
            'passenger': passenger_delay
        }
    }
```

#### 버스 우선 신호 (Transit Signal Priority, TSP)

**전략**:
1. **Green Extension**: 녹색 신호 연장 (0-20초)
2. **Red Truncation**: 적색 신호 단축 (0-15초)
3. **Phase Insertion**: 버스 전용 신호 삽입

**활성화 조건**:
```json
{
  "activation_criteria": {
    "delay_threshold": 180,        // 초 (3분 이상 지연)
    "occupancy_threshold": 0.7,    // 70% 이상 탑승
    "headway_threshold": 600,      // 초 (10분 이상 배차 간격)
    "priority_window": 300         // m (신호 300m 전 감지)
  }
}
```

### 8.2 지하철 시뮬레이션

#### 열차 운행 모델

**운행 시간**:
```
T_total = Σ T_running + Σ T_dwell + T_terminal

where:
  T_running = 역간 주행 시간
  T_dwell = 역 정차 시간
  T_terminal = 종착역 대기 시간
```

**배차 간격 최적화**:
```
Headway = (60 / Frequency) * 60  // seconds

예시:
  Peak: 2-3분 간격 → Frequency = 20-30 trains/hour
  Off-peak: 5-10분 간격 → Frequency = 6-12 trains/hour
```

#### 용량 분석

**차량 용량**:
```json
{
  "subway_car": {
    "seated_capacity": 48,
    "standing_capacity": 160,
    "total_capacity": 208,
    "crush_capacity": 250,
    "cars_per_train": 6,
    "train_capacity": 1248
  }
}
```

**역 용량**:
```
Platform_Capacity = (Platform_Length / 20) * 1000  // persons

Stairway_Capacity = Width * 80  // persons/minute/meter

Gate_Capacity = 25-30  // persons/minute/gate
```

### 8.3 환승 시뮬레이션

**환승 시간**:
```
T_transfer = T_walk + T_wait + T_stairs

where:
  T_walk = walking distance / 1.2 m/s
  T_wait = headway / 2 (average)
  T_stairs = vertical_distance / 0.5 m/s
```

**환승 저항 (Transfer Penalty)**:
```
Perceived_Time = Actual_Time + Transfer_Penalty

Transfer_Penalty = 5-10 minutes (subjective)
```

---

## 9. 교통 혼잡 예측

### 9.1 혼잡 지표

#### 교통 혼잡 지수 (Traffic Congestion Index)

```
TCI = (T_actual - T_free) / T_free * 100

where:
  T_actual = 실제 통행 시간
  T_free = 자유 흐름 통행 시간

분류:
  TCI < 20%: 원활
  20% ≤ TCI < 50%: 서행
  50% ≤ TCI < 100%: 혼잡
  TCI ≥ 100%: 정체
```

#### TTI (Travel Time Index)

```
TTI = T_peak / T_free

예시:
  TTI = 1.3 → 피크 시간이 자유 흐름보다 30% 더 소요
```

#### PTI (Planning Time Index)

```
PTI = T_95th_percentile / T_free

예시:
  PTI = 2.5 → 정시 도착을 위해 2.5배 시간 필요 (95% 신뢰도)
```

### 9.2 혼잡 예측 모델

#### Machine Learning 기반 예측

**입력 특성**:
```json
{
  "temporal_features": {
    "hour_of_day": 17,
    "day_of_week": 5,
    "month": 12,
    "is_holiday": false,
    "is_school_day": true
  },
  "traffic_features": {
    "current_speed": 35,
    "current_volume": 1800,
    "upstream_speed": 40,
    "downstream_speed": 30,
    "occupancy": 0.65
  },
  "weather_features": {
    "temperature": 5,
    "precipitation": 0.5,
    "visibility": 8,
    "wind_speed": 15
  },
  "event_features": {
    "special_event": false,
    "construction": true,
    "incident": false
  }
}
```

**모델 예시**:
1. **Random Forest**: 비선형 관계 포착
2. **XGBoost**: 고성능 예측
3. **LSTM**: 시계열 패턴 학습
4. **Transformer**: 장기 의존성 학습

**성능 지표**:
```python
metrics = {
    "MAE": 5.2,          # km/h
    "RMSE": 7.8,         # km/h
    "MAPE": 12.5,        # %
    "R²": 0.87
}
```

### 9.3 병목 구간 식별

**병목 강도 (Bottleneck Intensity)**:
```
BI = (q_upstream - q_bottleneck) / q_capacity

where:
  q_upstream = 상류 교통량
  q_bottleneck = 병목 교통량
  q_capacity = 용량
```

**재발성 vs 비재발성 혼잡**:

| 유형 | 원인 | 예측 가능성 | 비율 |
|------|------|------------|------|
| 재발성 | 용량 부족, 수요 초과 | 높음 | 40-50% |
| 비재발성 | 사고, 공사, 날씨, 이벤트 | 중간-낮음 | 50-60% |

---

## 10. 사고 영향 분석

### 10.1 사고 시나리오

#### 사고 유형별 영향

| 사고 유형 | 차단 차선 | 용량 감소 | 평균 처리 시간 |
|----------|----------|----------|---------------|
| 경미한 접촉 | 0-1 | 10-30% | 15-30분 |
| 일반 사고 | 1-2 | 40-60% | 30-90분 |
| 중대 사고 | 2+ | 70-90% | 90-180분 |
| 전복 | All | 100% | 180-300분 |
| 위험물 | All | 100% | 300+분 |

#### 용량 감소 모델

```
Q_incident = Q_normal * (1 - ∏ RF_i)

where:
  RF_i = reduction factor for each condition

  RF_lane_closure = 0.40 (1 lane closed on 3-lane highway)
  RF_rubbernecking = 0.10 (구경꾼 효과)
  RF_weather = 0.15 (악천후)
  RF_shoulder = 0.05 (갓길 이용 불가)
```

### 10.2 파급 효과 분석

**공간적 파급**:
```
L_queue = (Demand - Reduced_Capacity) * Duration / Jam_Density

예시:
  Demand = 2000 veh/h
  Reduced_Capacity = 800 veh/h
  Duration = 1 hour
  Jam_Density = 140 veh/km

  L_queue = (2000 - 800) * 1 / 140 = 8.6 km
```

**시간적 파급**:
```
Recovery_Time = Incident_Duration * Recovery_Ratio

Recovery_Ratio = 1.5-3.0 (사고 시간의 1.5-3배)
```

### 10.3 우회 경로 분석

**동적 교통 배정**:
```python
def dynamic_traffic_assignment(network, incident):
    """
    사고 발생 시 동적 경로 재배정
    """
    # 1. 사고 링크 용량 감소
    network.update_capacity(incident.link, incident.capacity_reduction)

    # 2. 최단 경로 재계산 (Dijkstra/A*)
    for origin in network.origins:
        for destination in network.destinations:
            new_path = network.shortest_path(origin, destination)

    # 3. 교통량 재배정 (User Equilibrium)
    converged = False
    while not converged:
        for od_pair in network.od_pairs:
            flows = network.assign_traffic(od_pair)
        converged = check_convergence(flows)

    return network.analyze_impacts()
```

---

## 11. 교통 정책 시뮬레이션

### 11.1 정책 유형

#### 도로 용량 확장

**분석 항목**:
- 차로 추가 효과
- 교차로 개선 효과
- BRT (Bus Rapid Transit) 도입 효과

**Before-After 비교**:
```json
{
  "before": {
    "lanes": 2,
    "capacity": 1800,
    "avg_speed": 35,
    "LOS": "D"
  },
  "after": {
    "lanes": 3,
    "capacity": 2700,
    "avg_speed": 55,
    "LOS": "B"
  },
  "improvement": {
    "capacity_increase": "50%",
    "speed_increase": "57%",
    "LOS_improvement": "D→B"
  }
}
```

#### 대중교통 개선

**정책 옵션**:
1. 배차 간격 단축 (10분 → 5분)
2. 버스 전용 차로 신설
3. 환승 센터 구축
4. 요금 인하 (10-20%)

**전환 효과 모델**:
```
Modal_Shift = f(Time_Saving, Cost_Saving, Comfort_Improvement)

Logit Model:
  P_transit = exp(U_transit) / (exp(U_transit) + exp(U_car))

  U_transit = β_time * Time + β_cost * Cost + β_comfort * Comfort + ε
```

#### 교통 수요 관리 (TDM)

**정책 도구**:
- 혼잡 통행료 (Congestion Pricing)
- 주차 요금 인상
- 카풀 우대 차로 (HOV Lane)
- 재택근무 장려
- 탄력 근무제

**혼잡 통행료 효과**:
```
Demand_Reduction = α * ln(1 + Toll/VOT)

where:
  α = price elasticity (-0.3 to -0.5)
  Toll = 통행료 (원)
  VOT = Value of Time (시간 가치, 15,000원/시간)
```

### 11.2 환경 영향 평가

#### 배출량 계산

**CO2 배출 모델**:
```
CO2 = Fuel_Consumption * 2.31  // kg CO2 per liter

Fuel_Consumption = f(Speed, Acceleration, Load)

MOVES Model (EPA):
  Emission_Rate = a + b*v + c*v² + d*a + e*a²

  where v = speed, a = acceleration
```

**배출량 비교**:
```json
{
  "baseline_scenario": {
    "total_vmt": 1000000,        // vehicle-miles traveled
    "avg_speed": 35,
    "CO2_emissions": 450,        // tons/day
    "NOx_emissions": 2.5,        // tons/day
    "PM2.5_emissions": 0.15      // tons/day
  },
  "improved_scenario": {
    "total_vmt": 950000,
    "avg_speed": 50,
    "CO2_emissions": 380,        // -15.6%
    "NOx_emissions": 2.0,        // -20%
    "PM2.5_emissions": 0.12      // -20%
  }
}
```

### 11.3 비용-편익 분석

**편익 항목**:
```
Total_Benefit = Travel_Time_Savings +
                Vehicle_Operating_Cost_Savings +
                Accident_Reduction_Benefit +
                Emission_Reduction_Benefit

Travel_Time_Savings = ΔTime * Traffic_Volume * VOT * 365 * Analysis_Period
```

**비용 항목**:
```
Total_Cost = Construction_Cost +
             Maintenance_Cost +
             Operating_Cost

Annualized_Cost = Total_Cost / Analysis_Period + Annual_O&M
```

**경제성 지표**:
```
BCR (Benefit-Cost Ratio) = PV(Benefits) / PV(Costs)
NPV (Net Present Value) = PV(Benefits) - PV(Costs)
IRR (Internal Rate of Return) = discount rate where NPV = 0

Criteria:
  BCR > 1.0: economically viable
  NPV > 0: profitable
  IRR > discount rate: acceptable
```

---

## 12. 데이터 모델

### 12.1 네트워크 데이터

**Link (도로 구간)**:
```json
{
  "linkId": "link-001",
  "name": "세종대로",
  "from_node": "node-100",
  "to_node": "node-101",
  "length": 500,
  "lanes": 4,
  "speed_limit": 60,
  "capacity": 1800,
  "link_type": "arterial",
  "coordinates": [
    [126.9780, 37.5665],
    [126.9785, 37.5670]
  ]
}
```

**Node (교차로)**:
```json
{
  "nodeId": "node-100",
  "name": "광화문 교차로",
  "node_type": "signalized",
  "signal_id": "signal-050",
  "coordinates": [126.9780, 37.5665],
  "elevation": 15.5
}
```

### 12.2 차량 데이터

```typescript
interface Vehicle {
  vehicleId: string;
  type: 'car' | 'bus' | 'truck' | 'motorcycle';
  origin: string;
  destination: string;
  route: string[];

  current_state: {
    link: string;
    lane: number;
    position: number;        // meters from start
    speed: number;           // km/h
    acceleration: number;    // m/s²
    timestamp: string;
  };

  characteristics: {
    length: number;          // m
    width: number;           // m
    max_acceleration: number;  // m/s²
    max_deceleration: number;  // m/s²
    desired_speed: number;   // km/h
  };
}
```

### 12.3 시뮬레이션 설정

```json
{
  "simulation": {
    "id": "sim-20251225-001",
    "name": "Gangnam Peak Hour Analysis",
    "description": "분석 평일 오후 6시 강남역 일대 교통 현황",

    "time_settings": {
      "start_time": "2025-12-25T18:00:00",
      "end_time": "2025-12-25T20:00:00",
      "time_step": 0.1,
      "warmup_period": 900
    },

    "network": {
      "area": "Gangnam District",
      "nodes": 150,
      "links": 400,
      "signals": 45
    },

    "demand": {
      "type": "od_matrix",
      "format": "csv",
      "file": "gangnam_od_matrix.csv",
      "scaling_factor": 1.0
    },

    "parameters": {
      "car_following_model": "Wiedemann99",
      "lane_change_model": "MOBIL",
      "route_choice": "dynamic",
      "seed": 42
    },

    "output": {
      "interval": 300,
      "metrics": ["speed", "volume", "density", "delay"],
      "format": "json",
      "aggregate_level": "link"
    }
  }
}
```

---

## 13. API 명세

### 13.1 Simulation API

#### POST /api/v1/simulations

시뮬레이션 생성

**Request**:
```json
{
  "name": "Test Simulation",
  "network_id": "network-001",
  "scenario": "peak_hour",
  "start_time": "2025-12-25T18:00:00",
  "duration": 7200,
  "parameters": {
    "time_step": 0.1,
    "seed": 42
  }
}
```

**Response**:
```json
{
  "simulation_id": "sim-123",
  "status": "queued",
  "created_at": "2025-12-25T10:00:00Z"
}
```

#### GET /api/v1/simulations/{id}

시뮬레이션 상태 조회

**Response**:
```json
{
  "simulation_id": "sim-123",
  "status": "running",
  "progress": 45.5,
  "start_time": "2025-12-25T10:01:00Z",
  "estimated_completion": "2025-12-25T10:15:00Z"
}
```

#### GET /api/v1/simulations/{id}/results

시뮬레이션 결과 조회

**Query Parameters**:
- `metric`: speed, volume, density, delay
- `aggregation`: link, zone, network
- `interval`: 300, 600, 900 (seconds)

**Response**:
```json
{
  "simulation_id": "sim-123",
  "results": {
    "network_performance": {
      "avg_speed": 35.5,
      "total_delay": 12500,
      "total_vmt": 50000,
      "LOS": "C"
    },
    "link_performance": [
      {
        "link_id": "link-001",
        "avg_speed": 40,
        "volume": 1800,
        "density": 45,
        "LOS": "C"
      }
    ]
  }
}
```

### 13.2 Traffic Data API

#### GET /api/v1/traffic/realtime

실시간 교통 데이터

**Response**:
```json
{
  "timestamp": "2025-12-25T18:30:00Z",
  "links": [
    {
      "link_id": "link-001",
      "speed": 35,
      "volume": 1800,
      "occupancy": 0.65,
      "incident": false
    }
  ]
}
```

### 13.3 Signal API

#### PUT /api/v1/signals/{id}/timing

신호 시간 업데이트

**Request**:
```json
{
  "cycle_length": 90,
  "phases": [
    {
      "phase_id": 1,
      "green_time": 35,
      "yellow_time": 3,
      "all_red": 2
    }
  ]
}
```

---

## 14. 보안 및 개인정보보호

### 14.1 데이터 익명화

**차량 추적 데이터**:
- 차량 ID 해싱
- 위치 정보 일반화 (100m 그리드)
- 통행 시작/종료 지점 삭제

**개인정보 보호 기법**:
- K-anonymity (k ≥ 5)
- Differential Privacy (ε ≤ 1.0)
- Data Aggregation (최소 15분 단위)

### 14.2 접근 제어

**역할 기반 접근 제어 (RBAC)**:
```json
{
  "roles": {
    "admin": ["read", "write", "delete", "simulate"],
    "analyst": ["read", "simulate"],
    "viewer": ["read"]
  }
}
```

---

## 15. 성과 지표 (KPI)

### 15.1 교통 효율성

| KPI | 측정 방법 | 목표 |
|-----|----------|------|
| 평균 통행 속도 | km/h | >40 |
| 평균 지체 | 초/대 | <30 |
| 네트워크 평균 LOS | A-F | C 이상 |
| V/C 비율 | - | <0.85 |

### 15.2 환경 영향

| KPI | 측정 방법 | 목표 |
|-----|----------|------|
| CO2 배출량 | tons/day | -20% |
| 연료 소비량 | liters/day | -15% |
| 공회전 시간 | hours/day | -25% |

### 15.3 시뮬레이션 정확도

| KPI | 측정 방법 | 목표 |
|-----|----------|------|
| GEH < 5 | % of links | >85% |
| MAPE (속도) | % | <15% |
| RMSE (교통량) | veh/h | <200 |

---

**弘益人間 (홍익인간) - Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
