# WIA-CITY-011: 빌딩 에너지 관리 표준 v1.0 ⚡

> **弘益人間 (홍익인간) - 널리 인간을 이롭게 하라**

## 문서 정보

- **표준 ID**: WIA-CITY-011
- **표준명**: Building Energy Management Standard (빌딩 에너지 관리 표준)
- **버전**: 1.0.0
- **발행일**: 2025-12-25
- **상태**: 정식 (Active)
- **카테고리**: CITY (건축/도시)
- **라이선스**: MIT License
- **발행 기관**: WIA (World Certification Industry Association)

---

## 목차

1. [개요](#1-개요)
2. [적용 범위](#2-적용-범위)
3. [용어 정의](#3-용어-정의)
4. [시스템 아키텍처](#4-시스템-아키텍처)
5. [데이터 형식](#5-데이터-형식)
6. [BEMS 구성 요소](#6-bems-구성-요소)
7. [에너지 계측](#7-에너지-계측)
8. [부하 관리](#8-부하-관리)
9. [신재생 에너지 통합](#9-신재생-에너지-통합)
10. [탄소 배출 관리](#10-탄소-배출-관리)
11. [인증 및 벤치마킹](#11-인증-및-벤치마킹)
12. [API 명세](#12-api-명세)
13. [보안 요구사항](#13-보안-요구사항)
14. [적합성 평가](#14-적합성-평가)

---

## 1. 개요

### 1.1 목적

WIA-CITY-011 빌딩 에너지 관리 표준은 건물의 에너지 소비를 체계적으로 모니터링, 분석, 최적화하기 위한 통합 표준입니다. 본 표준은 Building Energy Management System (BEMS)의 데이터 형식, 통신 프로토콜, 제어 로직, 성능 지표를 정의합니다.

### 1.2 배경

- 건물 부문은 전 세계 에너지 소비의 약 40%, 온실가스 배출의 33% 차지
- 스마트 빌딩 기술과 AI 기반 에너지 관리 시스템의 급속한 발전
- LEED, BREEAM 등 녹색 건축 인증 제도의 확산
- 넷제로 건물에 대한 정책적 요구사항 증가
- 에너지 비용 절감 및 운영 효율성 개선 필요성

### 1.3 적용 효과

- **에너지 절감**: 기존 대비 20-40% 에너지 소비 감축
- **비용 절감**: 연간 운영 비용 15-30% 절감
- **탄소 감축**: 건물 운영 탄소 배출 30-50% 감축
- **쾌적성 향상**: 실내 환경 품질 개선 (온도, 습도, 공기질)
- **자산 가치**: 건물 가치 5-15% 상승

### 1.4 핵심 기능

1. **실시간 모니터링**: 전력, 가스, 수도, 냉난방 에너지 사용량 15분 단위 계측
2. **AI 기반 최적화**: 머신러닝을 활용한 에너지 사용 패턴 예측 및 최적화
3. **수요 반응**: 전력망 상황에 따른 자동 부하 조절
4. **피크 절삭**: 최대 전력 수요 시간대 부하 분산
5. **재생 에너지 통합**: 태양광, ESS, 지열 등 신재생 에너지 통합 관리
6. **탄소 발자국**: 실시간 탄소 배출량 산정 및 리포팅
7. **에너지 벤치마킹**: 유사 건물 대비 에너지 효율 비교
8. **인증 지원**: LEED, BREEAM, G-SEED 등 녹색 건축 인증 데이터 제공

---

## 2. 적용 범위

### 2.1 건물 유형

- **상업용 건물**: 오피스, 쇼핑몰, 호텔, 병원
- **주거용 건물**: 아파트, 빌라, 단독주택
- **산업용 건물**: 공장, 창고, 물류센터
- **공공 건물**: 관공서, 학교, 도서관, 박물관
- **특수 목적 건물**: 데이터센터, 연구소, 공항

### 2.2 건물 규모

- **소형 건물**: 연면적 1,000m² 미만
- **중형 건물**: 연면적 1,000-10,000m²
- **대형 건물**: 연면적 10,000-100,000m²
- **초대형 건물**: 연면적 100,000m² 이상

### 2.3 시스템 범위

- HVAC (냉난방, 환기)
- 조명
- 급탕
- 전력 (콘센트 부하)
- 승강기
- 급수/배수
- 재생 에너지 (태양광, ESS, 지열)
- 건물 자동화 시스템 (BAS)

---

## 3. 용어 정의

### 3.1 핵심 용어

- **BEMS (Building Energy Management System)**: 빌딩 에너지 관리 시스템
- **EUI (Energy Use Intensity)**: 에너지 사용 집약도 (kWh/m²/년)
- **Peak Demand**: 최대 전력 수요 (kW)
- **Load Factor**: 부하율 = 평균 부하 / 최대 부하
- **Demand Response (DR)**: 수요 반응 - 전력망 상황에 따른 부하 조절
- **Peak Shaving**: 피크 절삭 - 최대 전력 수요 시간대 부하 감축
- **ESS (Energy Storage System)**: 에너지 저장 장치
- **PV (Photovoltaic)**: 태양광 발전
- **GHP (Geothermal Heat Pump)**: 지열 히트펌프
- **VRF (Variable Refrigerant Flow)**: 가변 냉매 유량 시스템

### 3.2 성능 지표

- **COP (Coefficient of Performance)**: 성능 계수
- **SEER (Seasonal Energy Efficiency Ratio)**: 계절 에너지 효율 비율
- **HSPF (Heating Seasonal Performance Factor)**: 난방 계절 성능 계수
- **PUE (Power Usage Effectiveness)**: 전력 사용 효율 (데이터센터)
- **Carbon Intensity**: 탄소 집약도 (kg CO₂e/kWh)

---

## 4. 시스템 아키텍처

### 4.1 계층 구조

```
┌─────────────────────────────────────────────────────────────┐
│              Layer 5: 클라우드/분석 플랫폼                     │
│   (빅데이터 분석, AI/ML, 대시보드, 리포팅, 외부 연동)         │
└─────────────────────────────────────────────────────────────┘
                           ↕ HTTPS/REST API
┌─────────────────────────────────────────────────────────────┐
│              Layer 4: BEMS 중앙 서버                          │
│   (데이터 수집, 처리, 저장, 제어 로직, 스케줄링)              │
└─────────────────────────────────────────────────────────────┘
                           ↕ BACnet/Modbus/MQTT
┌─────────────────────────────────────────────────────────────┐
│              Layer 3: 게이트웨이/컨트롤러                     │
│   (데이터 집계, 프로토콜 변환, 로컬 제어)                     │
└─────────────────────────────────────────────────────────────┘
                           ↕ BACnet/Modbus/RS-485
┌─────────────────────────────────────────────────────────────┐
│              Layer 2: 필드 컨트롤러                           │
│   (VAV, FCU, AHU, 보일러, 칠러 제어기)                       │
└─────────────────────────────────────────────────────────────┘
                           ↕ Analog/Digital I/O
┌─────────────────────────────────────────────────────────────┐
│              Layer 1: 센서 및 액추에이터                      │
│   (온도, 습도, CO₂, 조도, 전력, 유량, 밸브, 댐퍼)            │
└─────────────────────────────────────────────────────────────┘
```

### 4.2 통신 프로토콜

| 프로토콜 | 용도 | 특징 |
|---------|------|------|
| BACnet | 빌딩 자동화 표준 | ISO 16484-5, ASHRAE 135 |
| Modbus | 산업용 필드버스 | RTU/TCP 지원 |
| MQTT | IoT 메시징 | 경량, Pub/Sub 모델 |
| REST API | 클라우드 연동 | HTTP/HTTPS, JSON |
| OPC UA | 산업 자동화 | 보안, 정보 모델링 |

### 4.3 데이터 흐름

```
센서 → 필드 컨트롤러 → 게이트웨이 → BEMS 서버 → 클라우드
                                     ↓
                                 로컬 제어
                                     ↓
                            액추에이터 → 기계 설비
```

---

## 5. 데이터 형식

### 5.1 빌딩 정보 스키마

```json
{
  "buildingId": "BLD-SEOUL-001",
  "name": "그린타워",
  "type": "commercial-office",
  "location": {
    "address": "서울특별시 강남구 테헤란로 123",
    "coordinates": {
      "latitude": 37.5665,
      "longitude": 126.9780
    },
    "timezone": "Asia/Seoul",
    "climateZone": "4A"
  },
  "specifications": {
    "totalFloorArea": 50000,
    "grossFloorArea": 65000,
    "floors": {
      "aboveGround": 25,
      "underground": 5
    },
    "occupancy": 2000,
    "constructionYear": 2020,
    "certifications": ["LEED Gold", "G-SEED 1등급"]
  },
  "energySystems": {
    "hvac": {
      "heatingSource": "district-heating",
      "coolingSource": "electric-chiller",
      "ventilationType": "mechanical-with-hrv"
    },
    "lighting": {
      "type": "LED",
      "controlSystem": "daylight-harvesting"
    },
    "renewable": {
      "solar": {
        "installed": true,
        "capacity": 500,
        "type": "rooftop-pv"
      },
      "ess": {
        "installed": true,
        "capacity": 1000,
        "type": "lithium-ion"
      }
    }
  }
}
```

### 5.2 에너지 계측 데이터

```json
{
  "meterId": "MTR-ELEC-001",
  "buildingId": "BLD-SEOUL-001",
  "meterType": "electricity",
  "timestamp": "2025-12-25T14:30:00+09:00",
  "interval": "15min",
  "readings": {
    "activePower": 850.5,
    "reactivePower": 120.3,
    "apparentPower": 858.9,
    "powerFactor": 0.99,
    "voltage": {
      "phaseA": 220.5,
      "phaseB": 221.2,
      "phaseC": 220.8
    },
    "current": {
      "phaseA": 1250.5,
      "phaseB": 1248.3,
      "phaseC": 1251.0
    },
    "frequency": 60.0,
    "energy": {
      "cumulative": 1250000.5,
      "interval": 212.5
    }
  },
  "breakdown": {
    "hvac": 450.2,
    "lighting": 180.5,
    "plugLoad": 150.8,
    "elevators": 40.0,
    "others": 29.0
  },
  "quality": {
    "accuracy": 0.5,
    "dataValid": true,
    "estimatedData": false
  }
}
```

### 5.3 HVAC 운전 데이터

```json
{
  "systemId": "HVAC-AHU-001",
  "buildingId": "BLD-SEOUL-001",
  "systemType": "air-handling-unit",
  "timestamp": "2025-12-25T14:30:00+09:00",
  "operationalStatus": "running",
  "controlMode": "auto",
  "airflow": {
    "supplyAirflow": 10000,
    "returnAirflow": 9500,
    "outdoorAirflow": 2000,
    "exhaustAirflow": 1500
  },
  "temperature": {
    "supplyAir": 18.5,
    "returnAir": 24.2,
    "outdoorAir": 12.5,
    "mixedAir": 20.1
  },
  "humidity": {
    "supplyAir": 45.0,
    "returnAir": 50.5,
    "outdoorAir": 35.0
  },
  "pressure": {
    "staticPressure": 250.0,
    "differentialPressure": 180.0
  },
  "components": {
    "fan": {
      "status": "running",
      "speed": 75.0,
      "power": 15.5
    },
    "heatingCoil": {
      "status": "off",
      "valvePosition": 0.0,
      "output": 0.0
    },
    "coolingCoil": {
      "status": "on",
      "valvePosition": 45.0,
      "output": 85.5
    },
    "filter": {
      "status": "normal",
      "pressureDrop": 120.0,
      "replacementDue": "2026-03-25"
    }
  },
  "energyConsumption": {
    "current": 15.5,
    "daily": 250.8,
    "monthly": 7500.0
  }
}
```

---

## 6. BEMS 구성 요소

### 6.1 데이터 수집 시스템

**기능**:
- 센서 데이터 실시간 수집 (15분 간격)
- 다중 프로토콜 지원 (BACnet, Modbus, MQTT)
- 데이터 검증 및 필터링
- 손실 데이터 보간

**요구사항**:
- 최소 수집 주기: 15분
- 데이터 정확도: ±2% 이내
- 통신 신뢰성: 99.9% 이상
- 버퍼 용량: 24시간 이상

### 6.2 에너지 분석 엔진

**분석 기능**:
1. **기본 통계**: 평균, 최대, 최소, 표준편차
2. **추세 분석**: 시간대별, 요일별, 월별 패턴
3. **이상 탐지**: 비정상 에너지 사용 패턴 감지
4. **예측 모델**: 단기(1일), 중기(7일), 장기(30일) 예측
5. **회귀 분석**: 외기 온도, 재실자 수 등 변수와 상관관계

**성능 지표**:
- **EUI (Energy Use Intensity)**: kWh/m²/년
- **부하율 (Load Factor)**: 평균 부하 / 최대 부하
- **피크 수요 (Peak Demand)**: kW
- **탄소 배출량 (Carbon Emissions)**: kg CO₂e

### 6.3 최적화 제어

**제어 전략**:

1. **스케줄 제어**
   - 재실 스케줄 기반 냉난방 제어
   - 조명 자동 제어 (재실 센서, 주광 센서)
   - 야간/주말 최소 운전 모드

2. **최적 기동/정지 (Optimal Start/Stop)**
   - 예측 제어로 운전 시간 최소화
   - 건물 열관성 고려

3. **외기 냉방 (Free Cooling)**
   - 외기 온도가 낮을 때 냉방 부하 감소

4. **동시사용률 제어**
   - 설비 순차 기동으로 피크 수요 감축

5. **수요 반응 (Demand Response)**
   - 전력망 신호에 따라 부하 자동 조절
   - 피크 시간대 ESS 방전, 비피크 시간대 충전

### 6.4 사용자 인터페이스

**대시보드 구성**:
- 실시간 에너지 사용량
- 금일/금주/금월 사용량 및 비용
- 시스템 운전 현황
- 알람 및 이벤트
- 탄소 배출량
- 에너지 효율 지표
- 벤치마킹 비교

**알람 관리**:
- 높은 에너지 사용
- 설비 고장
- 센서 이상
- 목표치 초과
- 유지보수 알림

---

## 7. 에너지 계측

### 7.1 계측 항목

| 에너지원 | 계측 항목 | 단위 | 최소 정확도 |
|---------|----------|------|------------|
| 전력 | 유효전력, 무효전력, 역률 | kW, kvar, PF | Class 1.0 |
| 가스 | 순간 유량, 누적 사용량 | m³/h, m³ | ±2% |
| 지역난방 | 공급/환수 온도, 유량 | °C, m³/h | ±3% |
| 수도 | 순간 유량, 누적 사용량 | m³/h, m³ | ±2% |
| 냉수 | 공급/환수 온도, 유량 | °C, m³/h | ±3% |
| 온수 | 공급/환수 온도, 유량 | °C, m³/h | ±3% |

### 7.2 계측 위치

**전력 계측**:
- 수전반 (총 전력 소비)
- 주요 부하별 분전반 (HVAC, 조명, 콘센트, 승강기)
- 재생 에너지 발전량 (태양광)
- ESS 충방전량

**열량 계측**:
- 지역난방 공급점
- 칠러 1차/2차측
- 보일러 급탕
- 층별/용도별 분배

### 7.3 데이터 로깅

- **실시간 데이터**: 15분 간격
- **시간 데이터**: 1시간 평균/최대/최소
- **일일 데이터**: 24시간 누적/평균
- **월별 데이터**: 30일 누적/평균
- **보관 기간**: 최소 3년

---

## 8. 부하 관리

### 8.1 피크 절삭 (Peak Shaving)

**목표**: 최대 전력 수요 15-30% 감축

**전략**:
1. ESS 방전으로 피크 시간 전력 공급
2. 비필수 부하 순차 차단
3. 냉난방 설정 온도 임시 조정 (±1-2°C)
4. 조명 밝기 감소 (70-80% 수준)
5. 승강기 일부 운휴

**제어 로직**:
```
IF 현재전력 > 목표피크 - 100kW THEN
  1. ESS 방전 시작 (최대 200kW)
  2. 냉난방 설정온도 조정
  3. 현재전력 > 목표피크 THEN
     비필수 부하 순차 차단
  END IF
END IF
```

### 8.2 수요 반응 (Demand Response)

**DR 신호 수신**:
- OpenADR 2.0b 프로토콜 지원
- 한전 DR 시장 연동

**대응 수준**:
- **Level 1 (5-10% 감축)**: 냉난방 설정온도 조정
- **Level 2 (10-20% 감축)**: ESS 방전, 조명 밝기 조정
- **Level 3 (20-30% 감축)**: 비필수 부하 차단

**보상 메커니즘**:
- 부하 감축량 × 감축 시간 × 단가 = 보상금

### 8.3 부하 예측

**예측 모델**:
- 시계열 분석 (ARIMA, Prophet)
- 머신러닝 (Random Forest, XGBoost)
- 딥러닝 (LSTM, Transformer)

**입력 변수**:
- 과거 부하 패턴
- 외기 온도, 습도, 일사량
- 요일, 시간, 공휴일
- 재실자 수
- 특별 이벤트

**성능 목표**:
- 1일 예측 정확도: ±10% 이내
- 7일 예측 정확도: ±15% 이내

---

## 9. 신재생 에너지 통합

### 9.1 태양광 발전 (PV)

**시스템 구성**:
```json
{
  "pvSystemId": "PV-ROOF-001",
  "capacity": 500,
  "installationType": "rooftop",
  "panelCount": 1000,
  "panelWattage": 500,
  "inverterType": "string-inverter",
  "inverterCount": 10,
  "inverterCapacity": 50,
  "azimuth": 180,
  "tilt": 30
}
```

**실시간 모니터링**:
- 발전량 (kW)
- 누적 발전량 (kWh)
- 일사량 (W/m²)
- 모듈 온도 (°C)
- 인버터 효율 (%)
- 성능비 (PR, Performance Ratio)

**성능 지표**:
- PR (Performance Ratio) ≥ 80%
- 발전량 예측 정확도 ±15%

### 9.2 에너지 저장 장치 (ESS)

**시스템 사양**:
```json
{
  "essId": "ESS-001",
  "capacity": 1000,
  "power": 500,
  "batteryType": "lithium-ion",
  "cycleLife": 5000,
  "efficiency": 95,
  "dod": 80
}
```

**운영 모드**:
1. **피크 절삭**: 피크 시간 방전, 비피크 시간 충전
2. **자가 소비**: 태양광 발전 저장 후 사용
3. **주파수 조정**: 전력망 주파수 안정화 지원
4. **비상 전원**: 정전 시 중요 부하 백업

**제어 로직**:
```
- 09:00-12:00: 태양광 발전 충전 (SOC 80%까지)
- 12:00-16:00: 피크 시간대 방전 (최대 500kW)
- 16:00-21:00: 심야 요금 충전 (SOC 80%까지)
- 21:00-09:00: 대기 모드 (SOC 20-80% 유지)
```

### 9.3 지열 히트펌프 (GHP)

**시스템 정보**:
```json
{
  "ghpId": "GHP-001",
  "capacity": 1000,
  "heatPumpType": "water-to-water",
  "groundLoopType": "vertical-borehole",
  "boreholeCount": 50,
  "boreholeDepth": 150,
  "copHeating": 4.5,
  "copCooling": 5.5
}
```

**성능 모니터링**:
- 히트펌프 입출구 온도
- 지중 온도
- 유량
- COP (실시간, 계절 평균)
- 에너지 소비량

---

## 10. 탄소 배출 관리

### 10.1 탄소 배출 계산

**전력 소비 배출**:
```
CO₂ 배출량 (kg) = 전력 소비량 (kWh) × 배출 계수 (kg CO₂/kWh)
```

**한국 전력 배출 계수**: 0.4594 kg CO₂/kWh (2023년 기준)

**가스 소비 배출**:
```
CO₂ 배출량 (kg) = 가스 소비량 (m³) × 2.23 kg CO₂/m³
```

**지역난방 배출**:
```
CO₂ 배출량 (kg) = 열 소비량 (MJ) × 0.067 kg CO₂/MJ
```

### 10.2 탄소 발자국 리포팅

**일일 리포트**:
```json
{
  "reportDate": "2025-12-25",
  "buildingId": "BLD-SEOUL-001",
  "emissions": {
    "electricity": 2500.5,
    "gas": 1200.0,
    "districtHeating": 800.0,
    "total": 4500.5
  },
  "offsetByRenewable": {
    "solar": 500.5,
    "net": 4000.0
  },
  "intensity": {
    "perSquareMeter": 0.08,
    "perOccupant": 2.0
  },
  "target": {
    "daily": 4200.0,
    "achievement": 95.2
  }
}
```

### 10.3 넷제로 달성 경로

**단계별 목표**:
1. **Phase 1 (2025-2027)**: 에너지 효율 개선 → 30% 감축
2. **Phase 2 (2027-2030)**: 재생 에너지 확대 → 60% 감축
3. **Phase 3 (2030-2035)**: 100% 재생 에너지 → 넷제로 달성

---

## 11. 인증 및 벤치마킹

### 11.1 LEED 인증

**에너지 성능 크레딧**:
- EA Credit 1: Optimize Energy Performance (최대 18점)
- EA Credit 3: Advanced Energy Metering (1점)
- EA Credit 6: Renewable Energy Production (최대 3점)

**데이터 요구사항**:
- 12개월 연속 에너지 소비 데이터
- 에너지 모델링 결과
- 미터 데이터 (15분 간격)

### 11.2 BREEAM 인증

**에너지 섹션 (Ene)**:
- Ene 01: Energy efficiency (15%)
- Ene 02: Energy monitoring (2%)
- Ene 04: Low carbon design (3%)
- Ene 05: Energy efficient cold storage (3%)

### 11.3 에너지 벤치마킹

**비교 지표**:
| 건물 유형 | EUI 중간값 | 상위 25% | 하위 25% |
|----------|-----------|---------|---------|
| 오피스 | 150 kWh/m²/년 | 100 | 200 |
| 쇼핑몰 | 250 kWh/m²/년 | 180 | 350 |
| 호텔 | 300 kWh/m²/년 | 220 | 400 |
| 병원 | 400 kWh/m²/년 | 300 | 550 |
| 학교 | 120 kWh/m²/년 | 80 | 180 |

**벤치마킹 API**:
```
GET /api/v1/benchmarking/compare
{
  "buildingType": "commercial-office",
  "floorArea": 50000,
  "annualEnergy": 7500000,
  "location": "Seoul"
}

Response:
{
  "yourEUI": 150,
  "medianEUI": 150,
  "percentile": 50,
  "rank": "Average",
  "potentialSavings": 1250000,
  "recommendations": [...]
}
```

---

## 12. API 명세

### 12.1 인증

**API Key 발급**:
```
POST /api/v1/auth/register
{
  "organizationName": "그린빌딩 주식회사",
  "email": "admin@greenbuilding.com",
  "buildingId": "BLD-SEOUL-001"
}

Response:
{
  "apiKey": "wia_city011_xxxxxxxxxx",
  "expiresAt": "2026-12-25T00:00:00Z"
}
```

**헤더**:
```
Authorization: Bearer {apiKey}
Content-Type: application/json
X-WIA-Standard: CITY-011
X-WIA-Version: 1.0.0
```

### 12.2 실시간 데이터 조회

**전력 사용량 조회**:
```
GET /api/v1/energy/electricity?buildingId=BLD-SEOUL-001&interval=15min
```

**HVAC 운전 데이터**:
```
GET /api/v1/hvac/status?buildingId=BLD-SEOUL-001&systemId=HVAC-AHU-001
```

### 12.3 제어 명령

**냉난방 설정 온도 변경**:
```
POST /api/v1/hvac/setpoint
{
  "buildingId": "BLD-SEOUL-001",
  "zoneId": "ZONE-FL5-EAST",
  "mode": "cooling",
  "setpoint": 26.0,
  "duration": 3600
}
```

**조명 제어**:
```
POST /api/v1/lighting/control
{
  "buildingId": "BLD-SEOUL-001",
  "lightingGroupId": "LG-FL3-OFFICE",
  "dimLevel": 70,
  "schedule": {
    "startTime": "08:00",
    "endTime": "18:00"
  }
}
```

### 12.4 분석 및 리포트

**에너지 소비 분석**:
```
GET /api/v1/analytics/energy-consumption
  ?buildingId=BLD-SEOUL-001
  &startDate=2025-12-01
  &endDate=2025-12-31
  &aggregation=daily
```

**탄소 배출 리포트**:
```
GET /api/v1/analytics/carbon-emissions
  ?buildingId=BLD-SEOUL-001
  &period=monthly
```

---

## 13. 보안 요구사항

### 13.1 네트워크 보안

- **물리적 분리**: OT 네트워크와 IT 네트워크 분리
- **방화벽**: BEMS 서버 및 필드 디바이스 보호
- **VPN**: 원격 접속 시 암호화된 VPN 터널 사용
- **IDS/IPS**: 침입 탐지 및 차단 시스템

### 13.2 데이터 보안

- **암호화**: 전송 데이터 TLS 1.3 이상, 저장 데이터 AES-256
- **인증**: API Key + OAuth 2.0
- **권한 관리**: RBAC (Role-Based Access Control)
- **감사 로그**: 모든 제어 명령 및 접근 기록

### 13.3 사이버 보안

- **정기 점검**: 분기별 보안 취약점 스캔
- **패치 관리**: 월별 보안 패치 적용
- **백업**: 일일 자동 백업, 원격지 저장
- **재해 복구**: RPO 24시간, RTO 4시간

---

## 14. 적합성 평가

### 14.1 필수 요구사항

- [ ] 전력, 가스, 수도 계측 (15분 간격)
- [ ] 에너지 사용량 3년 이상 저장
- [ ] 실시간 모니터링 대시보드
- [ ] 알람 및 이벤트 관리
- [ ] 에너지 분석 리포트 (일/주/월/년)
- [ ] HVAC 자동 제어 기능
- [ ] API 인터페이스 제공
- [ ] 데이터 보안 (암호화, 인증)

### 14.2 권장 요구사항

- [ ] AI 기반 에너지 예측
- [ ] 피크 절삭 기능
- [ ] 수요 반응 연동
- [ ] 재생 에너지 통합
- [ ] ESS 제어
- [ ] 탄소 배출 추적
- [ ] LEED/BREEAM 데이터 제공
- [ ] 모바일 앱

### 14.3 성능 기준

| 지표 | 목표 |
|------|------|
| 데이터 수집 주기 | ≤ 15분 |
| 데이터 정확도 | ≥ 98% |
| 시스템 가용성 | ≥ 99.5% |
| 제어 응답 시간 | ≤ 5초 |
| 에너지 절감 | ≥ 20% (기존 대비) |
| 피크 수요 감축 | ≥ 15% |

---

## 부록 A: 센서 사양

### A.1 전력 계측기

- **정확도**: Class 1.0 (IEC 62053-22)
- **통신**: Modbus RTU/TCP, BACnet
- **계측 항목**: 유효전력, 무효전력, 역률, 전압, 전류, 주파수, 고조파

### A.2 온도 센서

- **측정 범위**: -20°C ~ 60°C
- **정확도**: ±0.5°C
- **응답 시간**: ≤ 30초

### A.3 습도 센서

- **측정 범위**: 0 ~ 100% RH
- **정확도**: ±3% RH
- **응답 시간**: ≤ 60초

### A.4 CO₂ 센서

- **측정 범위**: 0 ~ 2000 ppm
- **정확도**: ±50 ppm ±3%
- **보정 주기**: 12개월

---

## 부록 B: 참조 표준

- ISO 50001: 에너지 경영 시스템
- ISO 52000 시리즈: 건물 에너지 성능
- ASHRAE 90.1: 에너지 표준
- ASHRAE 135: BACnet 통신 프로토콜
- IEC 61850: 전력 시스템 통신
- OpenADR 2.0b: 수요 반응 통신
- IEEE 1547: 분산 에너지 자원 연계

---

## 부록 C: 용례 (Use Cases)

### C.1 오피스 빌딩 에너지 관리

**건물 정보**:
- 연면적: 50,000m²
- 층수: 지상 25층, 지하 5층
- 재실자: 2,000명
- 연간 에너지 비용: 10억 원

**적용 효과**:
- 에너지 소비 25% 감축
- 연간 2.5억 원 절감
- 탄소 배출 500톤 감축

### C.2 쇼핑몰 BEMS

**건물 정보**:
- 연면적: 100,000m²
- 영업 시간: 10:00-22:00
- 연간 방문객: 1,000만 명

**적용 효과**:
- 냉난방 에너지 30% 감축
- 조명 에너지 40% 감축 (LED + 주광 센서)
- 피크 수요 20% 감축 (ESS 활용)

---

## 부록 D: 변경 이력

| 버전 | 날짜 | 변경 내용 |
|------|------|----------|
| 1.0.0 | 2025-12-25 | 초판 발행 |

---

**문서 끝**

---

**발행 기관**: WIA (World Certification Industry Association)
**라이선스**: MIT License
**문의**: standards@wia.org
**홈페이지**: https://wia.org/standards/city-011

**弘益人間 (홍익인간) - 널리 인간을 이롭게 하라**
