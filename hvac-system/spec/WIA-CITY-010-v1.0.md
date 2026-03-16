# WIA-CITY-010: 냉난방 시스템 표준 v1.0 ❄️

> **弘益人間 (홍익인간) - 널리 인간을 이롭게 하라**

## 문서 정보

- **표준 번호**: WIA-CITY-010
- **표준 명칭**: HVAC System Standard (냉난방 시스템 표준)
- **버전**: 1.0
- **상태**: Active
- **발행일**: 2025-12-25
- **발행처**: WIA (World Certification Industry Association)
- **카테고리**: CITY (도시 인프라)
- **라이선스**: MIT

---

## 목차

1. [개요](#1-개요)
2. [적용 범위](#2-적용-범위)
3. [참조 표준](#3-참조-표준)
4. [용어 정의](#4-용어-정의)
5. [HVAC 시스템 유형](#5-hvac-시스템-유형)
6. [존 관리](#6-존-관리)
7. [온도 및 습도 제어](#7-온도-및-습도-제어)
8. [공기질 통합](#8-공기질-통합)
9. [에너지 효율](#9-에너지-효율)
10. [프로토콜 통합](#10-프로토콜-통합)
11. [예지 정비](#11-예지-정비)
12. [데이터 모델](#12-데이터-모델)
13. [API 명세](#13-api-명세)
14. [보안 요구사항](#14-보안-요구사항)
15. [구현 가이드](#15-구현-가이드)

---

## 1. 개요

### 1.1 목적

WIA-CITY-010은 건물 냉난방 시스템(HVAC)의 모니터링, 제어, 최적화를 위한 포괄적 표준을 정의합니다. 이 표준은 다양한 HVAC 시스템 유형, 제어 프로토콜, 에너지 관리를 통합하여 효율적이고 쾌적한 실내 환경을 제공합니다.

### 1.2 배경

건물 에너지 소비의 40-60%가 HVAC 시스템에서 발생합니다. 스마트 빌딩 시대에 HVAC 시스템의 통합 관리는 다음을 가능하게 합니다:

- **에너지 절감**: 20-40% 에너지 소비 감소
- **쾌적성 향상**: 정밀한 온습도 제어
- **예방 정비**: 장비 수명 20-30% 연장
- **통합 관리**: 단일 플랫폼에서 다중 시스템 관리

### 1.3 핵심 가치

**弘益人間 (홍익인간)** 철학에 따라, 이 표준은:

- 모든 건물 거주자에게 쾌적한 환경 제공
- 에너지 효율을 통한 환경 보호
- 표준화를 통한 시스템 상호운용성 보장
- 지속가능한 건물 운영 지원

---

## 2. 적용 범위

### 2.1 대상 시스템

이 표준은 다음 HVAC 시스템에 적용됩니다:

- **분리형 에어컨** (Split AC)
- **멀티 시스템** (Multi-Split)
- **VRF/VRV** (Variable Refrigerant Flow)
- **칠러 시스템** (Chiller-based)
- **히트펌프** (Heat Pump)
- **공조기** (AHU - Air Handling Unit)
- **팬코일 유닛** (FCU - Fan Coil Unit)
- **복사 냉난방** (Radiant Heating/Cooling)
- **지열 시스템** (Geothermal)

### 2.2 대상 건물

- 주거용 건물 (아파트, 주택)
- 상업용 건물 (오피스, 쇼핑몰)
- 산업 시설
- 병원 및 의료 시설
- 교육 시설 (학교, 대학)
- 데이터센터
- 호텔 및 숙박 시설

### 2.3 제외 사항

다음은 이 표준의 범위에서 제외됩니다:

- HVAC 설비의 기계적 설계
- 전기 배선 및 설치
- 건축 구조 설계

---

## 3. 참조 표준

### 3.1 국제 표준

- **ASHRAE 90.1**: Energy Standard for Buildings
- **ASHRAE 55**: Thermal Environmental Conditions
- **ISO 16484**: Building Automation and Control Systems (BACS)
- **ISO 50001**: Energy Management Systems
- **BACnet (ISO 16484-5)**: Building Automation Protocol
- **Modbus**: Industrial Communication Protocol
- **EN 15232**: Energy Performance of Buildings (EPBD)

### 3.2 WIA 표준

- **WIA-ENE-027**: Indoor Air Quality (실내 공기질)
- **WIA-CITY-001**: Smart Building Platform (스마트 빌딩)
- **WIA-ENE-001**: Energy Monitoring (에너지 모니터링)
- **WIA-INTENT**: Intent Expression Standard

### 3.3 산업 표준

- AHRI 550/590: Performance Rating of Water-Chilling Packages
- SEER/EER: Seasonal Energy Efficiency Ratio
- COP: Coefficient of Performance

---

## 4. 용어 정의

### 4.1 기본 용어

| 용어 | 정의 |
|------|------|
| **HVAC** | Heating, Ventilation, and Air Conditioning (난방, 환기, 냉방) |
| **존 (Zone)** | 독립적으로 제어되는 공간 단위 |
| **설정온도 (Setpoint)** | 목표 온도 값 |
| **데드밴드 (Deadband)** | 냉난방이 작동하지 않는 온도 범위 |
| **VAV** | Variable Air Volume (가변 풍량) |
| **CAV** | Constant Air Volume (정풍량) |
| **BMS** | Building Management System |

### 4.2 시스템 용어

| 용어 | 정의 |
|------|------|
| **VRF** | Variable Refrigerant Flow (가변 냉매 유량) |
| **AHU** | Air Handling Unit (공조기) |
| **FCU** | Fan Coil Unit (팬코일 유닛) |
| **칠러 (Chiller)** | 냉각수 생성 장치 |
| **보일러 (Boiler)** | 온수/증기 생성 장치 |
| **냉각탑 (Cooling Tower)** | 냉각수 냉각 장치 |

### 4.3 성능 지표

| 용어 | 정의 |
|------|------|
| **COP** | Coefficient of Performance (성능계수) |
| **SEER** | Seasonal Energy Efficiency Ratio (계절 에너지 효율) |
| **EER** | Energy Efficiency Ratio (에너지 효율비) |
| **IPLV** | Integrated Part Load Value (부분부하 효율) |
| **PUE** | Power Usage Effectiveness (전력 사용 효율) |

---

## 5. HVAC 시스템 유형

### 5.1 분리형 에어컨 (Split AC)

#### 특징
- 실내기와 실외기로 분리
- 단일 공간 냉난방
- 주거 및 소규모 상업 공간

#### 제어 파라미터
```yaml
system_type: SPLIT_AC
components:
  - indoor_unit:
      model: "IU-2024"
      capacity_btu: 12000
      capacity_kw: 3.5
  - outdoor_unit:
      model: "OU-2024"
      compressor_type: "INVERTER"

control:
  mode: [COOLING, HEATING, AUTO, DRY, FAN_ONLY]
  temperature_range: [16, 30]  # °C
  fan_speed: [AUTO, LOW, MEDIUM, HIGH, TURBO]
  swing: [AUTO, FIXED, HORIZONTAL, VERTICAL]
```

### 5.2 VRF 시스템

#### 특징
- 하나의 실외기에 다수의 실내기
- 개별 존 제어
- 동시 냉난방 가능
- 높은 에너지 효율

#### 시스템 구성
```yaml
system_type: VRF
capacity_total_kw: 50.0
outdoor_units:
  - unit_id: "ODU-01"
    capacity_kw: 50.0
    compressor_type: "INVERTER_SCROLL"
    refrigerant: "R410A"

indoor_units:
  - unit_id: "IDU-01"
    type: "CEILING_CASSETTE"
    zone_id: "ZONE-101"
    capacity_kw: 5.6
  - unit_id: "IDU-02"
    type: "WALL_MOUNTED"
    zone_id: "ZONE-102"
    capacity_kw: 3.5
  - unit_id: "IDU-03"
    type: "DUCTED"
    zone_id: "ZONE-103"
    capacity_kw: 7.1

features:
  - simultaneous_heating_cooling: true
  - heat_recovery: true
  - individual_zone_control: true
```

### 5.3 칠러 시스템

#### 특징
- 중대형 건물용
- 냉각수/온수 순환 방식
- FCU, AHU와 연계

#### 시스템 구성
```yaml
system_type: CHILLER
chiller:
  chiller_id: "CH-01"
  type: "WATER_COOLED"
  compressor_type: "CENTRIFUGAL"
  capacity_ton: 500
  capacity_kw: 1758
  refrigerant: "R134a"
  cop_rated: 6.5
  iplv: 7.2

cooling_tower:
  tower_id: "CT-01"
  type: "OPEN_CIRCUIT"
  capacity_ton: 600
  fan_control: "VFD"

pumps:
  - pump_id: "CHWP-01"
    type: "CHILLED_WATER"
    flow_rate_lpm: 5000
    head_m: 40
    vfd_enabled: true
  - pump_id: "CWP-01"
    type: "CONDENSER_WATER"
    flow_rate_lpm: 6000
    head_m: 35
    vfd_enabled: true

distribution:
  primary_loop:
    supply_temp_c: 7
    return_temp_c: 12
    delta_t: 5
  secondary_loop:
    supply_temp_c: 8
    return_temp_c: 13
```

### 5.4 히트펌프

#### 특징
- 냉난방 겸용
- 높은 에너지 효율
- 지열/공기열/수열 방식

#### 타입
```yaml
heat_pump_types:
  air_source:
    cop_heating: 3.5-4.5
    cop_cooling: 3.0-4.0
    temp_range: [-15, 43]  # °C

  ground_source:
    cop_heating: 4.0-5.0
    cop_cooling: 4.5-5.5
    temp_range: [-5, 35]
    loop_type: [VERTICAL, HORIZONTAL, POND]

  water_source:
    cop_heating: 4.5-5.5
    cop_cooling: 5.0-6.0
    source: [LAKE, RIVER, WELL, SEA]
```

### 5.5 공조기 (AHU)

#### 특징
- 외기 처리 및 순환
- 필터링, 냉각, 가열, 가습
- VAV/CAV 제어

#### 구성
```yaml
system_type: AHU
ahu_id: "AHU-01"
capacity_cmh: 10000  # m³/h
capacity_cfm: 5886

components:
  filters:
    pre_filter: "G4"
    main_filter: "F7"
    final_filter: "H13_HEPA"

  cooling_coil:
    type: "CHILLED_WATER"
    rows: 6
    capacity_kw: 150

  heating_coil:
    type: "HOT_WATER"
    rows: 4
    capacity_kw: 100

  humidifier:
    type: "STEAM"
    capacity_kg_h: 50

  supply_fan:
    type: "CENTRIFUGAL"
    motor_kw: 15
    vfd_enabled: true

  return_fan:
    type: "CENTRIFUGAL"
    motor_kw: 11
    vfd_enabled: true

control:
  type: "VAV"
  supply_temp_control: "RESET"
  supply_temp_setpoint_c: 14
  reset_schedule:
    outdoor_temp_c: [10, 15, 20, 25, 30]
    supply_temp_c: [18, 16, 14, 12, 10]

  ventilation:
    mode: "DEMAND_CONTROLLED"
    co2_setpoint_ppm: 800
    min_outdoor_air_percent: 20
    economizer_enabled: true
```

---

## 6. 존 관리

### 6.1 존 정의

존(Zone)은 독립적으로 제어되는 공간 단위입니다.

```yaml
zone:
  zone_id: "ZONE-301"
  name: "3층 회의실 A"
  floor: 3
  building: "본관"

  geometry:
    area_m2: 45.0
    volume_m3: 121.5
    ceiling_height_m: 2.7

  occupancy:
    max_occupancy: 12
    occupancy_sensor: true

  thermal:
    design_cooling_load_kw: 5.4
    design_heating_load_kw: 4.2
    thermal_mass: "MEDIUM"

  equipment:
    - type: "VRF_INDOOR_UNIT"
      unit_id: "IDU-301"
    - type: "THERMOSTAT"
      device_id: "TSTAT-301"
    - type: "CO2_SENSOR"
      device_id: "CO2-301"
```

### 6.2 멀티존 제어

#### 존 우선순위
```yaml
zone_priority:
  critical:
    - "서버실"
    - "수술실"
    - "클린룸"
  high:
    - "회의실"
    - "사무실"
  normal:
    - "복도"
    - "로비"
  low:
    - "창고"
    - "주차장"
```

#### 존간 상호작용
```yaml
zone_interaction:
  - source_zone: "ZONE-301"
    adjacent_zones:
      - zone_id: "ZONE-302"
        wall_type: "PARTITION"
        u_value: 0.5
      - zone_id: "CORRIDOR-3F"
        door_type: "GLASS"
        air_transfer: true
```

### 6.3 스케줄링

```yaml
zone_schedule:
  zone_id: "ZONE-301"
  schedules:
    - name: "평일 근무"
      days: [MON, TUE, WED, THU, FRI]
      periods:
        - start: "07:00"
          end: "09:00"
          mode: "PREHEAT/PRECOOL"
          temp_setpoint_c: 22

        - start: "09:00"
          end: "18:00"
          mode: "OCCUPIED"
          temp_setpoint_c: 22
          humidity_setpoint_percent: 50

        - start: "18:00"
          end: "22:00"
          mode: "SETBACK"
          temp_setpoint_c: 25

        - start: "22:00"
          end: "07:00"
          mode: "UNOCCUPIED"
          temp_setpoint_c: 28

    - name: "주말"
      days: [SAT, SUN]
      periods:
        - start: "00:00"
          end: "24:00"
          mode: "UNOCCUPIED"
          temp_setpoint_c: 28
```

---

## 7. 온도 및 습도 제어

### 7.1 온도 제어

#### 설정온도 범위
```yaml
temperature_control:
  cooling:
    min_setpoint_c: 18
    max_setpoint_c: 28
    default_setpoint_c: 24
    deadband_c: 2

  heating:
    min_setpoint_c: 16
    max_setpoint_c: 26
    default_setpoint_c: 22
    deadband_c: 2

  comfort_range:
    summer:
      optimal: [23, 26]  # °C
      acceptable: [20, 28]
    winter:
      optimal: [20, 23]
      acceptable: [18, 25]
```

#### 제어 알고리즘
```yaml
control_algorithm:
  type: "PID"
  parameters:
    proportional_gain: 0.5
    integral_time_s: 300
    derivative_time_s: 60

  anti_windup:
    enabled: true
    max_integral: 100

  output:
    type: "MODULATING"
    range: [0, 100]  # %
    update_interval_s: 10
```

### 7.2 습도 제어

```yaml
humidity_control:
  enabled: true

  setpoints:
    min_rh_percent: 40
    max_rh_percent: 60
    optimal_rh_percent: 50

  equipment:
    humidifier:
      type: "STEAM"
      capacity_kg_h: 50
      modulation: [0, 100]  # %

    dehumidifier:
      type: "COOLING_COIL_REHEAT"
      enabled: true

  control:
    priority: "SECONDARY"  # Temperature has priority
    deadband_percent: 5
    max_deviation_percent: 10
```

### 7.3 온습도 통합 제어

```yaml
integrated_control:
  psychrometric_control:
    enabled: true
    control_parameter: "ENTHALPY"  # or DEWPOINT

  sequential_control:
    - condition: "temp_high"
      action: "INCREASE_COOLING"

    - condition: "temp_low"
      action: "INCREASE_HEATING"

    - condition: "humidity_high"
      actions:
        - "INCREASE_COOLING"
        - "ENABLE_REHEAT"

    - condition: "humidity_low"
      action: "ACTIVATE_HUMIDIFIER"
```

---

## 8. 공기질 통합

### 8.1 공기질 파라미터

HVAC 시스템은 WIA-ENE-027 실내 공기질 표준과 통합됩니다.

```yaml
air_quality_integration:
  monitored_parameters:
    - parameter: "CO2"
      threshold_ppm: 1000
      action: "INCREASE_VENTILATION"

    - parameter: "PM2.5"
      threshold_ugm3: 35
      action: "INCREASE_FILTRATION"

    - parameter: "VOC"
      threshold_ppb: 500
      action: "INCREASE_OUTDOOR_AIR"

    - parameter: "HUMIDITY"
      threshold_percent: 60
      action: "ACTIVATE_DEHUMIDIFICATION"
```

### 8.2 환기 제어

```yaml
ventilation_control:
  mode: "DEMAND_CONTROLLED_VENTILATION"

  outdoor_air:
    min_flow_percent: 20
    max_flow_percent: 100

  co2_based:
    enabled: true
    setpoint_ppm: 800
    control_range_ppm: [400, 1200]

  economizer:
    enabled: true
    type: "DIFFERENTIAL_ENTHALPY"
    lockout_temp_c: 21

  strategies:
    - name: "Night Purge"
      enabled: true
      schedule: "02:00-06:00"
      outdoor_temp_range_c: [15, 25]

    - name: "Pre-Occupancy Flush"
      enabled: true
      duration_minutes: 30
      outdoor_air_percent: 100
```

### 8.3 필터 관리

```yaml
filter_management:
  filters:
    - filter_id: "PRE-FILTER-01"
      type: "G4"
      efficiency_percent: 90
      pressure_drop_initial_pa: 50
      pressure_drop_final_pa: 200

    - filter_id: "MAIN-FILTER-01"
      type: "F7"
      efficiency_percent: 85
      pressure_drop_initial_pa: 100
      pressure_drop_final_pa: 300

    - filter_id: "HEPA-FILTER-01"
      type: "H13"
      efficiency_percent: 99.95
      pressure_drop_initial_pa: 200
      pressure_drop_final_pa: 500

  replacement:
    criteria:
      - "PRESSURE_DROP"
      - "TIME_BASED"
      - "PARTICLE_COUNT"

    pressure_threshold_pa: 250
    time_threshold_hours: 2160  # 3 months

  monitoring:
    pressure_sensor: true
    particle_counter: true
    alert_threshold_days: 7
```

---

## 9. 에너지 효율

### 9.1 성능 지표

#### COP (Coefficient of Performance)
```yaml
cop_measurement:
  definition: "출력 / 입력"

  cooling_mode:
    calculation: "냉방능력_kw / 소비전력_kw"
    typical_range: [2.5, 4.5]
    excellent: ">4.0"

  heating_mode:
    calculation: "난방능력_kw / 소비전력_kw"
    typical_range: [3.0, 5.0]
    excellent: ">4.5"
```

#### SEER (Seasonal Energy Efficiency Ratio)
```yaml
seer_rating:
  definition: "계절별 냉방 총량 / 소비 전력 총량"
  unit: "BTU/Wh"

  ratings:
    minimum: 13
    standard: 14-16
    high_efficiency: 17-20
    ultra_high: ">20"

  calculation_conditions:
    outdoor_temp_range_f: [65, 104]
    test_conditions:
      - temp_f: 82
        weight_percent: 28
      - temp_f: 92
        weight_percent: 45
      - temp_f: 102
        weight_percent: 27
```

### 9.2 에너지 절감 전략

```yaml
energy_saving_strategies:
  - strategy: "Optimal Start/Stop"
    description: "건물 열 특성 학습하여 최적 시작/종료 시간 결정"
    potential_savings_percent: 10-20

  - strategy: "Demand Response"
    description: "전력 피크 시간대 부하 감소"
    potential_savings_percent: 15-25

  - strategy: "Free Cooling"
    description: "외기 온도 낮을 때 칠러 대신 외기 사용"
    potential_savings_percent: 20-40

  - strategy: "Variable Speed Control"
    description: "팬, 펌프 인버터 제어"
    potential_savings_percent: 30-50

  - strategy: "Heat Recovery"
    description: "배기 열 회수"
    potential_savings_percent: 15-30

  - strategy: "Occupancy-based Control"
    description: "재실 감지 기반 제어"
    potential_savings_percent: 20-35
```

### 9.3 에너지 모니터링

```yaml
energy_monitoring:
  metering:
    - meter_id: "EM-HVAC-01"
      type: "ELECTRIC"
      measurement:
        - "ACTIVE_POWER_KW"
        - "REACTIVE_POWER_KVAR"
        - "POWER_FACTOR"
        - "ENERGY_KWH"
      interval_s: 60

  kpi:
    - metric: "EUI"
      name: "Energy Use Intensity"
      unit: "kWh/m²/year"
      target: "<120"

    - metric: "Cooling_Specific_Power"
      unit: "kW/ton"
      target: "<0.6"

    - metric: "PUE"
      name: "Power Usage Effectiveness"
      calculation: "Total_Power / IT_Power"
      target: "<1.5"

  reporting:
    frequency: "DAILY"
    benchmarking: true
    alerts:
      - threshold_exceeded
      - anomaly_detected
      - target_missed
```

---

## 10. 프로토콜 통합

### 10.1 BACnet

```yaml
bacnet_integration:
  protocol_version: "1.24"
  device_profile: "BACnet Application Specific Controller (B-ASC)"

  objects:
    - object_type: "ANALOG_INPUT"
      instances:
        - id: 1
          name: "Zone Temperature"
          units: "DEGREES_CELSIUS"

    - object_type: "ANALOG_OUTPUT"
      instances:
        - id: 1
          name: "Cooling Valve Position"
          units: "PERCENT"

    - object_type: "ANALOG_VALUE"
      instances:
        - id: 1
          name: "Temperature Setpoint"
          units: "DEGREES_CELSIUS"

    - object_type: "BINARY_INPUT"
      instances:
        - id: 1
          name: "Occupancy Status"

    - object_type: "BINARY_OUTPUT"
      instances:
        - id: 1
          name: "Fan Enable"

  services:
    - "ReadProperty"
    - "WriteProperty"
    - "SubscribeCOV"
    - "GetAlarmSummary"
    - "GetEventInformation"

  network:
    transport: "BACnet/IP"
    port: 47808
    bbmd_enabled: true
```

### 10.2 Modbus

```yaml
modbus_integration:
  protocol: "Modbus TCP"
  port: 502

  register_map:
    holding_registers:
      - address: 40001
        name: "Zone_Temp_Setpoint"
        unit: "°C"
        scale: 0.1
        writable: true

      - address: 40002
        name: "Operating_Mode"
        values:
          0: "OFF"
          1: "COOLING"
          2: "HEATING"
          3: "AUTO"
        writable: true

    input_registers:
      - address: 30001
        name: "Zone_Temperature"
        unit: "°C"
        scale: 0.1

      - address: 30002
        name: "Zone_Humidity"
        unit: "%RH"
        scale: 0.1

      - address: 30003
        name: "Power_Consumption"
        unit: "kW"
        scale: 0.01

    coils:
      - address: 00001
        name: "System_Enable"
        writable: true

    discrete_inputs:
      - address: 10001
        name: "Alarm_Status"
```

### 10.3 KNX

```yaml
knx_integration:
  protocol_version: "KNX/IP"

  group_addresses:
    - ga: "1/1/1"
      dpt: "DPT_Switch"
      name: "HVAC_On_Off"

    - ga: "1/1/2"
      dpt: "DPT_Value_Temp"
      name: "Zone_Temperature"

    - ga: "1/1/3"
      dpt: "DPT_Scaling"
      name: "Fan_Speed_Percent"

  telegram_handling:
    read: true
    write: true
    respond: true
```

### 10.4 LonWorks

```yaml
lonworks_integration:
  protocol: "LonTalk"

  network_variables:
    - nv_name: "nviSpaceTemp"
      snvt: "SNVT_temp_p"
      direction: "INPUT"

    - nv_name: "nvoSpaceTemp"
      snvt: "SNVT_temp_p"
      direction: "OUTPUT"

    - nv_name: "nviSetpoint"
      snvt: "SNVT_temp_setpt"
      direction: "INPUT"
```

---

## 11. 예지 정비

### 11.1 상태 기반 모니터링

```yaml
condition_monitoring:
  parameters:
    - name: "Compressor Vibration"
      sensor_type: "ACCELEROMETER"
      threshold_mm_s: 7.1
      alert_level: "CRITICAL"

    - name: "Bearing Temperature"
      sensor_type: "TEMPERATURE"
      threshold_c: 75
      alert_level: "WARNING"

    - name: "Refrigerant Pressure"
      sensor_type: "PRESSURE"
      range_bar: [15, 25]
      alert_level: "CRITICAL"

    - name: "Oil Level"
      sensor_type: "LEVEL"
      min_percent: 30
      alert_level: "WARNING"

    - name: "Motor Current"
      sensor_type: "CURRENT"
      baseline_tolerance_percent: 20
      alert_level: "WARNING"
```

### 11.2 고장 예측

```yaml
fault_prediction:
  algorithms:
    - type: "MACHINE_LEARNING"
      model: "LSTM"
      training_data_months: 12
      prediction_horizon_days: 30

  failure_modes:
    - component: "Compressor"
      indicators:
        - "Increased vibration"
        - "Higher discharge temp"
        - "Lower efficiency"
      mtbf_hours: 50000

    - component: "Fan Motor"
      indicators:
        - "Bearing noise"
        - "Higher current"
        - "Reduced airflow"
      mtbf_hours: 40000

    - component: "Refrigerant Leak"
      indicators:
        - "Low pressure"
        - "Reduced cooling"
        - "Frost formation"
      detection_method: "PRESSURE_MONITORING"
```

### 11.3 정비 스케줄링

```yaml
maintenance_schedule:
  preventive:
    - task: "Filter Replacement"
      frequency_months: 3
      duration_hours: 1

    - task: "Coil Cleaning"
      frequency_months: 6
      duration_hours: 4

    - task: "Refrigerant Check"
      frequency_months: 12
      duration_hours: 2

    - task: "Electrical Inspection"
      frequency_months: 12
      duration_hours: 3

  predictive:
    trigger: "CONDITION_BASED"
    notification_advance_days: 7
    work_order_generation: "AUTOMATIC"

  optimization:
    grouping: true
    minimize_downtime: true
    cost_optimization: true
```

---

## 12. 데이터 모델

### 12.1 시스템 엔티티

```typescript
// 데이터 모델은 api/typescript/src/types.ts에 정의됨
// 여기서는 주요 구조만 설명

HVACSystem {
  system_id: string
  system_type: SystemType
  location: Location
  capacity: Capacity
  components: Component[]
  zones: Zone[]
  status: SystemStatus
  performance: PerformanceMetrics
  energy: EnergyData
  maintenance: MaintenanceRecord
}
```

### 12.2 실시간 데이터

```yaml
realtime_data:
  timestamp: "ISO8601"
  sampling_rate_s: 60

  measurements:
    temperatures:
      - zone_id
      - current_c
      - setpoint_c
      - deviation_c

    humidity:
      - zone_id
      - current_percent
      - setpoint_percent

    power:
      - equipment_id
      - power_kw
      - energy_kwh

    air_quality:
      - zone_id
      - co2_ppm
      - pm25_ugm3

  status:
    - equipment_id
    - operating_mode
    - run_status
    - alarm_status
```

---

## 13. API 명세

### 13.1 시스템 등록

```http
POST /api/v1/hvac/systems/register
Content-Type: application/json

{
  "system": {
    "system_id": "HVAC-BLDG-A-01",
    "system_type": "VRF",
    "building_id": "BLDG-A",
    "capacity_kw": 50.0
  }
}

Response: 201 Created
{
  "success": true,
  "data": {
    "system_id": "HVAC-BLDG-A-01",
    "registration_date": "2025-12-25T10:00:00Z",
    "dashboard_url": "https://hvac.wia.org/systems/HVAC-BLDG-A-01"
  }
}
```

### 13.2 온도 제어

```http
PUT /api/v1/hvac/zones/{zone_id}/setpoint
Content-Type: application/json

{
  "temperature_c": 22.0,
  "mode": "AUTO",
  "fan_speed": "AUTO"
}

Response: 200 OK
{
  "success": true,
  "data": {
    "zone_id": "ZONE-301",
    "setpoint_updated": "2025-12-25T10:05:00Z",
    "effective_immediately": true
  }
}
```

### 13.3 상태 조회

```http
GET /api/v1/hvac/systems/{system_id}/status

Response: 200 OK
{
  "success": true,
  "data": {
    "system_id": "HVAC-BLDG-A-01",
    "timestamp": "2025-12-25T10:10:00Z",
    "overall_status": "RUNNING",
    "zones": [
      {
        "zone_id": "ZONE-301",
        "temperature_c": 22.5,
        "setpoint_c": 22.0,
        "humidity_percent": 52,
        "mode": "COOLING",
        "comfort_level": "OPTIMAL"
      }
    ],
    "energy": {
      "current_power_kw": 35.2,
      "daily_energy_kwh": 456.8
    }
  }
}
```

---

## 14. 보안 요구사항

### 14.1 인증 및 권한

```yaml
authentication:
  method: "OAuth2.0"
  token_expiry_hours: 24

authorization:
  roles:
    - role: "SYSTEM_ADMIN"
      permissions:
        - "system_configure"
        - "user_manage"
        - "all_read"
        - "all_write"

    - role: "FACILITY_MANAGER"
      permissions:
        - "system_monitor"
        - "setpoint_adjust"
        - "schedule_modify"

    - role: "OPERATOR"
      permissions:
        - "system_monitor"
        - "setpoint_adjust_limited"

    - role: "OCCUPANT"
      permissions:
        - "zone_read"
        - "comfort_vote"
```

### 14.2 데이터 보호

```yaml
data_protection:
  encryption:
    at_rest: "AES-256"
    in_transit: "TLS 1.3"

  privacy:
    pii_handling: "GDPR_COMPLIANT"
    data_retention_days: 730
    anonymization: true
```

---

## 15. 구현 가이드

### 15.1 시스템 통합 단계

1. **현황 조사**
   - 기존 HVAC 시스템 파악
   - 제어 프로토콜 확인
   - 센서 및 액추에이터 목록 작성

2. **설계**
   - 존 구성 설계
   - 제어 전략 수립
   - 통신 아키텍처 설계

3. **구현**
   - 게이트웨이 설치
   - 센서 추가/교체
   - 제어 로직 프로그래밍

4. **시운전**
   - 통신 테스트
   - 제어 로직 검증
   - 성능 측정

5. **최적화**
   - 데이터 분석
   - 제어 파라미터 튜닝
   - 에너지 절감 검증

### 15.2 권장 하드웨어

```yaml
hardware_recommendations:
  controller:
    - "Siemens PXC Series"
    - "Honeywell Excel 10"
    - "Johnson Controls FX Series"
    - "Tridium JACE"

  sensors:
    temperature:
      accuracy: "±0.3°C"
      range: "0-50°C"

    humidity:
      accuracy: "±2% RH"
      range: "0-100% RH"

    co2:
      accuracy: "±50 ppm"
      range: "0-2000 ppm"

  actuators:
    valve:
      type: "MODULATING"
      control_signal: "0-10V or 4-20mA"

    damper:
      type: "PROPORTIONAL"
      actuator: "24VAC"
```

### 15.3 베스트 프랙티스

1. **제어 전략**
   - 재실 기반 제어 활용
   - 외기 이코노마이저 적극 활용
   - 야간 냉방/난방 Setback

2. **에너지 관리**
   - 실시간 모니터링
   - 벤치마킹
   - 지속적 개선

3. **유지보수**
   - 예방 정비 스케줄 준수
   - 필터 정기 교체
   - 데이터 기반 진단

---

## 부록 A: 참조 구현

TypeScript SDK 및 CLI 도구는 다음 위치에 있습니다:

- **API SDK**: `api/typescript/`
- **CLI 도구**: `cli/hvac-system.sh`
- **설치 스크립트**: `install.sh`

## 부록 B: 용례

실제 구현 사례는 README.md를 참조하십시오.

---

## 문서 이력

| 버전 | 날짜 | 변경 내용 | 작성자 |
|------|------|-----------|--------|
| 1.0 | 2025-12-25 | 초기 버전 | WIA Standards Committee |

---

## 라이선스

© 2025 SmileStory Inc. / WIA - World Certification Industry Association

이 표준은 MIT 라이선스 하에 배포됩니다.

---

## 연락처

- **웹사이트**: https://wia-official.org
- **이메일**: standards@wia-official.org
- **GitHub**: https://github.com/WIA-Official/wia-standards

---

**弘益人間 (홍익인간) - 널리 인간을 이롭게 하라**

*효율적인 HVAC, 쾌적한 환경, 지속가능한 미래*
