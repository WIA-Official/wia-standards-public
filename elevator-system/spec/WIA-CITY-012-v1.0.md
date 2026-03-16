# WIA-CITY-012: 엘리베이터 시스템 표준 v1.0 🛗

> **弘益人間 (홍익인간) - 널리 인간을 이롭게 하라**

**표준 ID:** WIA-CITY-012
**버전:** 1.0.0
**발행일:** 2025-12-25
**상태:** 활성
**카테고리:** 스마트시티 (CITY)

---

## 목차

1. [개요](#1-개요)
2. [적용 범위](#2-적용-범위)
3. [용어 정의](#3-용어-정의)
4. [엘리베이터 유형](#4-엘리베이터-유형)
5. [군관제 시스템](#5-군관제-시스템)
6. [목적층 예약 시스템](#6-목적층-예약-시스템)
7. [교통 분석 및 최적화](#7-교통-분석-및-최적화)
8. [에너지 재생 시스템](#8-에너지-재생-시스템)
9. [안전 시스템](#9-안전-시스템)
10. [접근성 기능](#10-접근성-기능)
11. [예측 정비](#11-예측-정비)
12. [IoT 센서 통합](#12-iot-센서-통합)
13. [성과 지표 (KPI)](#13-성과-지표-kpi)
14. [보안 및 개인정보보호](#14-보안-및-개인정보보호)
15. [인증 요구사항](#15-인증-요구사항)

---

## 1. 개요

### 1.1 목적

WIA-CITY-012 엘리베이터 시스템 표준은 지능형 엘리베이터 관제, 에너지 효율, 안전, 접근성을 통합한 스마트빌딩 수직 운송 시스템의 국제 표준입니다.

### 1.2 핵심 원칙

- **효율성 (Efficiency)**: AI 기반 교통 흐름 최적화
- **안전성 (Safety)**: 다층 안전 시스템 및 실시간 모니터링
- **접근성 (Accessibility)**: 모든 사용자를 위한 포용적 설계
- **지속가능성 (Sustainability)**: 에너지 재생 및 저전력 운영
- **신뢰성 (Reliability)**: 99.5% 가동률 목표
- **투명성 (Transparency)**: 실시간 상태 정보 공개
- **예방성 (Predictiveness)**: AI 기반 고장 예측 및 예방

### 1.3 적용 대상

- 고층 빌딩 및 마천루
- 스마트 오피스 빌딩
- 주거용 아파트 단지
- 병원 및 의료 시설
- 쇼핑몰 및 상업 시설
- 호텔 및 리조트
- 교통 허브 (역사, 공항)
- 산업 및 물류 시설

---

## 2. 적용 범위

### 2.1 엘리베이터 유형

- **견인식 (Traction)**: 로프/케이블 구동 (고층용)
- **유압식 (Hydraulic)**: 유압 실린더 구동 (저층용)
- **기계실 없는 엘리베이터 (MRL - Machine Room Less)**: 컴팩트 설계
- **고속 엘리베이터**: 속도 > 5m/s (초고층 빌딩)
- **더블 덱 엘리베이터**: 2층 동시 서비스
- **화물용 엘리베이터**: 대형 화물 운송
- **소형 화물 엘리베이터 (Dumbwaiter)**: 음식/서류 운송

### 2.2 건물 규모

- **저층 건물**: 7층 이하
- **중층 건물**: 8-20층
- **고층 건물**: 21-50층
- **초고층 건물**: 51층 이상

### 2.3 운영 모드

- **일반 운행**: 정상 운영
- **출퇴근 모드**: 피크 타임 최적화
- **야간 모드**: 절전 운영
- **비상 모드**: 재난 대응
- **정비 모드**: 유지보수
- **VIP 모드**: 전용 서비스

---

## 3. 용어 정의

### 3.1 기본 용어

- **카 (Car)**: 승객을 태우는 엘리베이터 객실
- **샤프트 (Shaft)**: 엘리베이터 승강로
- **홀 (Hall)**: 각 층의 엘리베이터 대기 공간
- **도어 (Door)**: 카 도어 및 홀 도어
- **랜딩 (Landing)**: 각 층 정차 위치
- **호이스트웨이 (Hoistway)**: 엘리베이터 수직 이동 공간

### 3.2 기술 용어

- **군관제 (Group Control)**: 여러 대의 엘리베이터 통합 제어
- **목적층 예약 (Destination Dispatch)**: 홀에서 목적층 미리 입력
- **평균 대기 시간 (AWT - Average Waiting Time)**: 호출 후 도착까지 평균 시간
- **평균 이동 시간 (ATT - Average Travel Time)**: 탑승 후 목적층까지 시간
- **처리 용량 (Handling Capacity)**: 5분간 운송 가능한 승객 수 (%)
- **왕복 시간 (RTT - Round Trip Time)**: 한 바퀴 순환 시간

### 3.3 성능 용어

- **정격 속도 (Rated Speed)**: 최대 운행 속도 (m/s)
- **정격 하중 (Rated Load)**: 최대 탑승 인원/중량
- **가속도 (Acceleration)**: 출발/정지 가속도 (m/s²)
- **저크 (Jerk)**: 가속도 변화율 (m/s³) - 승차감 지표
- **에너지 재생률 (Regeneration Rate)**: 회생 에너지 비율

---

## 4. 엘리베이터 유형

### 4.1 견인식 엘리베이터 (Traction Elevator)

```typescript
interface TractionElevator {
  type: 'traction';
  driveType: 'geared' | 'gearless';

  // 성능 사양
  maxSpeed: number;         // m/s (최대 10+ m/s)
  maxLoad: number;          // kg (일반: 1,000-2,000kg)
  maxTravel: number;        // m (최대 500+ m)

  // 로프 시스템
  rope: {
    material: 'steel' | 'carbon-fiber';
    diameter: number;       // mm
    count: number;          // 로프 개수
    safetyFactor: number;   // 안전율 (일반: 12:1)
  };

  // 견인 시스템
  traction: {
    sheaveType: 'single-wrap' | 'double-wrap';
    sheaveDiameter: number; // mm
    motorPower: number;     // kW
    efficiency: number;     // %
  };

  // 제동 시스템
  brake: {
    type: 'electromagnetic' | 'disc';
    redundancy: boolean;
    emergencyBrake: boolean;
  };
}
```

**적용:**
- 고층/초고층 빌딩
- 높은 속도와 효율 요구
- 장거리 수직 운송

### 4.2 유압식 엘리베이터 (Hydraulic Elevator)

```typescript
interface HydraulicElevator {
  type: 'hydraulic';
  driveType: 'direct' | 'indirect' | 'roped-hydraulic';

  // 성능 사양
  maxSpeed: number;         // m/s (일반: 0.5-1.5 m/s)
  maxLoad: number;          // kg (대형 화물용 가능)
  maxTravel: number;        // m (일반: < 20m)

  // 유압 시스템
  hydraulic: {
    cylinderType: 'telescopic' | 'single-stage';
    cylinderDiameter: number;    // mm
    workingPressure: number;     // bar
    fluid: 'mineral-oil' | 'bio-degradable';
    pumpPower: number;           // kW
  };

  // 밸브 시스템
  valve: {
    type: 'solenoid' | 'proportional';
    flowControl: boolean;
    pressureRelief: boolean;
  };

  // 안전 장치
  safety: {
    overloadProtection: boolean;
    antiDrop: boolean;           // 낙하 방지
    emergencyLowering: boolean;  // 비상 하강
  };
}
```

**적용:**
- 저층 건물 (7층 이하)
- 화물 운송
- 유지보수 접근이 용이한 환경

### 4.3 기계실 없는 엘리베이터 (MRL)

```typescript
interface MRLElevator {
  type: 'mrl';
  baseType: 'traction';

  // 컴팩트 설계
  installation: {
    noMachineRoom: true;
    motorLocation: 'shaft-top' | 'shaft-side';
    controllerLocation: 'shaft' | 'landing';
    spaceRequired: 'minimal';     // 공간 절약
  };

  // 모터 시스템
  motor: {
    type: 'permanent-magnet' | 'gearless';
    efficiency: number;           // % (>90%)
    compactDesign: true;
  };

  // 성능
  maxSpeed: number;               // m/s (일반: < 4 m/s)
  maxTravel: number;              // m (일반: < 75m)

  // 이점
  advantages: {
    spaceSaving: boolean;
    energyEfficient: boolean;
    lowerCost: boolean;
    quietOperation: boolean;
  };
}
```

**적용:**
- 중층 건물
- 공간 제약이 있는 건물
- 에너지 효율 중시

---

## 5. 군관제 시스템

### 5.1 군관제 개요

군관제는 빌딩 내 여러 대의 엘리베이터를 통합 제어하여 대기 시간 최소화, 에너지 효율 극대화, 교통 흐름 최적화를 달성합니다.

### 5.2 군관제 알고리즘

```typescript
interface GroupControlSystem {
  // 기본 정보
  buildingId: string;
  elevatorGroup: {
    groupId: string;
    elevators: string[];          // 엘리베이터 ID 목록
    totalCapacity: number;        // 총 처리 용량
    serviceZones: number[];       // 서비스 층
  };

  // 제어 알고리즘
  algorithm: {
    type: 'conventional' | 'ai-optimized' | 'fuzzy-logic' | 'genetic-algorithm';
    optimization: {
      minimizeWaitingTime: boolean;
      minimizeEnergy: boolean;
      balanceLoad: boolean;
      predictTraffic: boolean;
    };
  };

  // 호출 할당
  callAllocation: {
    strategy: 'nearest-car' | 'sector-allocation' | 'dynamic-sectoring';
    considerLoad: boolean;
    considerDirection: boolean;
    considerWaitingPassengers: number;
  };

  // 교통 패턴 학습
  trafficLearning: {
    enabled: boolean;
    historicalData: boolean;
    realTimeAdaptation: boolean;
    peakHourDetection: boolean;
  };
}
```

### 5.3 군관제 모드

| 모드 | 시간대 | 특징 | 최적화 목표 |
|------|--------|------|-------------|
| **아침 출근** | 07:00-09:00 | 1층→상층 집중 | 상향 교통 처리 |
| **점심 시간** | 12:00-14:00 | 중간층 이동 | 분산 교통 처리 |
| **저녁 퇴근** | 17:00-19:00 | 상층→1층 집중 | 하향 교통 처리 |
| **야간/주말** | 기타 시간 | 저빈도 사용 | 에너지 절약 |
| **비상 상황** | 재난 시 | 대피 우선 | 안전한 대피 |

### 5.4 성능 지표

```typescript
interface GroupControlMetrics {
  // 대기 시간
  waitingTime: {
    average: number;              // 초 (목표: < 30초)
    percentile95: number;         // 초 (목표: < 60초)
    maximum: number;              // 초
  };

  // 이동 시간
  travelTime: {
    average: number;              // 초
    percentile95: number;         // 초
  };

  // 처리 용량
  handlingCapacity: {
    current: number;              // %
    peak: number;                 // % (목표: > 15%)
  };

  // 효율성
  efficiency: {
    carUtilization: number;       // % (평균 탑승률)
    emptyRuns: number;            // % (공차 운행 비율)
    energyPerTrip: number;        // kWh
  };
}
```

---

## 6. 목적층 예약 시스템

### 6.1 목적층 예약 개요

목적층 예약 시스템(Destination Dispatch)은 승객이 홀에서 목적층을 미리 입력하면 AI가 최적의 카를 배정하는 첨단 시스템입니다.

### 6.2 시스템 구조

```typescript
interface DestinationDispatchSystem {
  // 입력 인터페이스
  inputDevices: {
    touchScreen: boolean;
    keypad: boolean;
    voiceControl: boolean;
    mobileApp: boolean;
    accessCard: boolean;          // RFID/NFC
  };

  // 목적층 그룹화
  destinationGrouping: {
    enabled: boolean;
    groupSize: number;            // 동일 목적층 그룹
    maxWaitTime: number;          // 초 (그룹 대기 시간)
  };

  // 카 배정 알고리즘
  carAssignment: {
    criteria: {
      minimizeWaitTime: boolean;
      minimizeEnergyUse: boolean;
      balanceLoad: boolean;
      minimizeStops: number;
    };
    optimization: 'real-time' | 'predictive';
  };

  // 승객 안내
  guidance: {
    displayAssignedCar: string;   // 배정된 카 번호
    estimatedWaitTime: number;    // 초
    visualIndicators: boolean;
    audioAnnouncements: boolean;
  };
}
```

### 6.3 목적층 예약 장점

- **대기 시간 30-40% 감소**
- **에너지 사용 20-30% 절감**
- **처리 용량 25-30% 증가**
- **홀 혼잡도 감소**
- **승객 경험 향상**

### 6.4 구현 예시

```typescript
interface DestinationRequest {
  requestId: string;
  timestamp: string;
  hall: {
    floor: number;
    location: string;
  };
  destination: number;              // 목적층
  passengerCount: number;           // 예상 승객 수
  priority: 'normal' | 'vip' | 'emergency';
  accessibility: {
    wheelchair: boolean;
    visuallyImpaired: boolean;
    hearingImpaired: boolean;
  };
}

interface CarAssignment {
  assignmentId: string;
  carId: string;
  carNumber: number;                // 표시용 카 번호
  estimatedArrival: number;         // 초
  estimatedTravelTime: number;      // 초
  stops: number[];                  // 경유 층
  capacity: {
    current: number;
    available: number;
  };
}
```

---

## 7. 교통 분석 및 최적화

### 7.1 교통 패턴 분석

```typescript
interface TrafficAnalysis {
  // 실시간 분석
  realTime: {
    currentDemand: number;          // 호출/분
    activePassengers: number;
    queueLength: number[];          // 각 층 대기 인원
    trafficDirection: 'up' | 'down' | 'balanced';
  };

  // 역사 데이터 분석
  historical: {
    dailyPattern: {
      hourly: number[];             // 시간대별 수요
      peakHours: string[];
      lowTrafficHours: string[];
    };
    weeklyPattern: {
      weekday: number[];
      weekend: number[];
    };
    seasonalTrends: boolean;
  };

  // 예측 모델
  prediction: {
    algorithm: 'lstm' | 'arima' | 'prophet';
    horizonMinutes: number;         // 예측 시간 범위
    accuracy: number;               // %
    nextPeakTime: string;
  };
}
```

### 7.2 대기 시간 최적화

```typescript
interface WaitingTimeOptimization {
  // 목표
  targets: {
    averageWaitTime: number;        // 초 (목표: < 30)
    maxWaitTime: number;            // 초 (목표: < 60)
    serviceLevel: number;           // % (목표: 95%)
  };

  // 전략
  strategies: {
    carParking: {
      enabled: boolean;
      parkingFloors: number[];      // 대기 위치
      adaptiveParkingEnabled: boolean;
    };
    anticipatoryControl: {
      enabled: boolean;
      predictionHorizon: number;    // 분
    };
    loadBalancing: {
      enabled: boolean;
      redistributionThreshold: number;
    };
  };

  // 실시간 조정
  realTimeAdjustment: {
    demandSensing: boolean;
    dynamicSectoring: boolean;
    expressService: boolean;        // 특급 서비스
  };
}
```

### 7.3 교통 흐름 시뮬레이션

AI 기반 시뮬레이션으로 다양한 시나리오 테스트:

- **피크 타임 시뮬레이션**: 최대 부하 테스트
- **비상 대피 시뮬레이션**: 재난 대응 검증
- **정비 시나리오**: 일부 카 운행 중단 시 대응
- **이벤트 시나리오**: 대규모 행사 시 대응

---

## 8. 에너지 재생 시스템

### 8.1 에너지 재생 개요

엘리베이터 하강 및 경부하 상승 시 발생하는 에너지를 회수하여 건물 전력망에 환원합니다.

### 8.2 재생 시스템 구조

```typescript
interface EnergyRegenerationSystem {
  // 재생 타입
  type: 'passive' | 'active';

  // 재생 방식
  regeneration: {
    method: 'grid-feedback' | 'battery-storage' | 'capacitor-storage';
    efficiency: number;             // % (70-90%)
    maxPower: number;               // kW
  };

  // 배터리 저장 (선택)
  batteryStorage?: {
    capacity: number;               // kWh
    type: 'lithium-ion' | 'supercapacitor';
    chargeRate: number;             // kW
    dischargeRate: number;          // kW
    cycleLife: number;
  };

  // 에너지 측정
  energyMetering: {
    consumedEnergy: number;         // kWh
    regeneratedEnergy: number;      // kWh
    netEnergy: number;              // kWh
    regenerationRate: number;       // %
  };

  // 성능 지표
  performance: {
    dailyRegeneration: number;      // kWh/일
    annualSavings: number;          // KRW/년
    co2Reduction: number;           // kg CO₂/년
  };
}
```

### 8.3 에너지 절감 전략

| 전략 | 설명 | 절감 효과 |
|------|------|-----------|
| **LED 조명** | 카 내부 및 홀 LED 조명 | 5-10% |
| **대기 모드** | 저사용 시간대 절전 | 10-15% |
| **AI 최적화** | 운행 경로 및 속도 최적화 | 15-20% |
| **재생 제동** | 에너지 회수 및 재사용 | 20-30% |
| **가변 주파수 드라이브** | 모터 속도 정밀 제어 | 10-15% |

**총 에너지 절감 가능량: 40-60%**

---

## 9. 안전 시스템

### 9.1 다층 안전 시스템

```typescript
interface ElevatorSafetySystem {
  // 기계적 안전 장치
  mechanical: {
    safetyGear: {
      type: 'instantaneous' | 'progressive';
      activationSpeed: number;      // m/s
      redundancy: boolean;
    };
    overspeedGovernor: {
      trippingSpeed: number;        // m/s
      testFrequency: string;        // '매월'
    };
    buffers: {
      type: 'spring' | 'oil';
      stroke: number;               // mm
    };
    emergencyBrake: boolean;
  };

  // 전기적 안전 장치
  electrical: {
    doorSafetyEdge: boolean;        // 도어 끼임 방지
    lightCurtain: boolean;          // 적외선 감지
    overloadSensor: boolean;
    emergencyLighting: boolean;
    emergencyAlarm: boolean;
    emergencyPhone: boolean;
  };

  // 센서 시스템
  sensors: {
    positionSensors: boolean;
    loadSensors: boolean;
    vibrationSensors: boolean;
    temperatureSensors: boolean;
    smokeSensors: boolean;
  };

  // 모니터링
  monitoring: {
    realTimeMonitoring: boolean;
    remoteMonitoring: boolean;
    faultDetection: boolean;
    predictiveMaintenance: boolean;
  };
}
```

### 9.2 비상 대응 프로토콜

```typescript
interface EmergencyProtocol {
  // 화재 비상
  fireEmergency: {
    action: 'recall-to-evacuation-floor';
    evacuationFloor: number;        // 대피층 (일반: 1층)
    doorAction: 'open-and-disable';
    elevatorService: 'disabled';
  };

  // 지진 비상
  earthquakeEmergency: {
    detection: 'seismic-sensor';
    action: 'controlled-stop-at-nearest-floor';
    doorAction: 'open';
    aftershockProtocol: boolean;
  };

  // 정전 비상
  powerOutage: {
    backupPower: boolean;
    action: 'controlled-descent-to-nearest-floor';
    emergencyLighting: boolean;
    communication: 'battery-powered';
  };

  // 갇힘 사고
  entrapment: {
    detection: 'automatic';
    communication: 'two-way-intercom';
    monitoring: '24/7-control-center';
    rescueTime: number;             // 분 (목표: < 30)
  };
}
```

### 9.3 안전 인증 및 규정

- **EN 81-1/81-2**: 유럽 안전 규정
- **ASME A17.1**: 미국 안전 코드
- **ISO 8100**: 국제 승강기 안전 기준
- **KS B 6900**: 한국 산업 규격
- **승강기 안전관리법**: 한국 법규

---

## 10. 접근성 기능

### 10.1 장애인 접근성

```typescript
interface AccessibilityFeatures {
  // 휠체어 사용자
  wheelchair: {
    minimumCarWidth: number;        // mm (1,100mm 이상)
    minimumCarDepth: number;        // mm (1,400mm 이상)
    handrails: boolean;
    rearViewMirror: boolean;
    lowButtonHeight: number;        // mm (900-1,200mm)
  };

  // 시각 장애인
  visuallyImpaired: {
    brailleButtons: boolean;
    voiceAnnouncements: boolean;
    audioFloorIndicators: boolean;
    tactileFloorIndicators: boolean;
    highContrastButtons: boolean;
  };

  // 청각 장애인
  hearingImpaired: {
    visualAlarms: boolean;
    textDisplay: boolean;
    vibrationAlerts: boolean;
  };

  // 기타 편의 기능
  convenience: {
    automaticDoors: boolean;
    extendedDoorOpenTime: boolean;
    largePrintButtons: boolean;
    colorCodedButtons: boolean;
  };
}
```

### 10.2 유니버설 디자인 원칙

- **공평한 사용**: 누구나 동등하게 사용 가능
- **사용의 유연성**: 다양한 방법으로 사용 가능
- **간단하고 직관적**: 쉬운 이해와 사용
- **인지 가능한 정보**: 시각, 청각, 촉각 정보 제공
- **오류에 대한 포용력**: 실수 방지 및 복구 용이
- **적은 물리적 노력**: 최소한의 힘으로 사용
- **접근과 사용을 위한 크기와 공간**: 충분한 공간 확보

---

## 11. 예측 정비

### 11.1 예측 정비 시스템

```typescript
interface PredictiveMaintenanceSystem {
  // IoT 센서 데이터
  sensorData: {
    vibration: number[];            // 진동 패턴
    temperature: number[];          // 온도 변화
    current: number[];              // 전류 소비
    noise: number[];                // 소음 레벨
    doorCycles: number;             // 도어 개폐 횟수
    brakePadWear: number;           // 브레이크 마모도
    ropeCondition: number;          // 로프 상태
  };

  // AI 분석
  aiAnalysis: {
    algorithm: 'random-forest' | 'lstm' | 'isolation-forest';
    anomalyDetection: boolean;
    failurePrediction: {
      component: string;
      probability: number;          // %
      estimatedFailureDate: string;
      recommendedAction: string;
    };
  };

  // 정비 스케줄
  maintenanceSchedule: {
    type: 'predictive' | 'preventive' | 'corrective';
    scheduledDate: string;
    priority: 'critical' | 'high' | 'medium' | 'low';
    estimatedDuration: number;      // 시간
    requiredParts: string[];
  };

  // 성과
  performance: {
    mtbf: number;                   // 평균 고장 간격 (시간)
    mttr: number;                   // 평균 수리 시간 (시간)
    availabilityRate: number;       // % (목표: 99.5%)
    maintenanceCostReduction: number; // %
  };
}
```

### 11.2 정비 항목 및 주기

| 항목 | 점검 주기 | 예측 정비 적용 |
|------|-----------|----------------|
| **로프/케이블** | 월 1회 | ✓ 마모 패턴 분석 |
| **도어 시스템** | 월 1회 | ✓ 개폐 사이클 모니터링 |
| **브레이크** | 분기 1회 | ✓ 마모 센서 |
| **모터/구동부** | 분기 1회 | ✓ 진동/온도 분석 |
| **안전 장치** | 월 1회 | ✓ 동작 테스트 |
| **제어 시스템** | 월 1회 | ✓ 로그 분석 |
| **비상 통신** | 월 1회 | ✓ 연결 상태 |

---

## 12. IoT 센서 통합

### 12.1 센서 네트워크

```typescript
interface IoTSensorNetwork {
  // 센서 유형
  sensors: {
    // 운행 센서
    operational: {
      positionEncoder: boolean;     // 위치 추적
      speedSensor: boolean;         // 속도 측정
      loadCell: boolean;            // 하중 측정
      accelerometer: boolean;       // 가속도 측정
    };

    // 상태 센서
    condition: {
      vibrationSensor: boolean;
      temperatureSensor: boolean;
      currentSensor: boolean;
      noiseSensor: boolean;
    };

    // 안전 센서
    safety: {
      doorSensor: boolean;
      smokeSensor: boolean;
      seismicSensor: boolean;
      waterDetector: boolean;
    };

    // 환경 센서
    environmental: {
      co2Sensor: boolean;
      humiditySensor: boolean;
      lightSensor: boolean;
    };
  };

  // 통신 프로토콜
  communication: {
    protocol: 'mqtt' | 'modbus' | 'bacnet' | 'opcua';
    frequency: number;              // Hz (데이터 전송 주기)
    encryption: boolean;
    edgeComputing: boolean;         // 엣지 처리
  };

  // 데이터 관리
  dataManagement: {
    storage: 'cloud' | 'on-premise' | 'hybrid';
    retention: number;              // 일 (보관 기간)
    realTimeProcessing: boolean;
    batchProcessing: boolean;
  };
}
```

### 12.2 스마트빌딩 통합

```typescript
interface SmartBuildingIntegration {
  // BMS 연동
  buildingManagementSystem: {
    protocol: 'bacnet' | 'lonworks';
    hvacIntegration: boolean;       // 공조 연동
    lightingControl: boolean;       // 조명 제어
    securitySystem: boolean;        // 보안 연동
    fireAlarmSystem: boolean;       // 화재 경보 연동
  };

  // 에너지 관리
  energyManagement: {
    demandResponse: boolean;        // 수요 반응
    loadShifting: boolean;          // 부하 이동
    peakShaving: boolean;           // 피크 절감
  };

  // 사용자 경험
  userExperience: {
    mobileApp: boolean;
    smartCardIntegration: boolean;
    voiceControl: boolean;
    personalizedService: boolean;
  };
}
```

---

## 13. 성과 지표 (KPI)

### 13.1 운영 효율성 KPI

```typescript
interface OperationalKPIs {
  // 대기 시간
  waitingTime: {
    average: number;                // 초 (목표: < 30)
    peak95Percentile: number;       // 초 (목표: < 60)
    offPeak95Percentile: number;    // 초 (목표: < 40)
  };

  // 이동 시간
  travelTime: {
    average: number;                // 초
    percentile95: number;           // 초
  };

  // 처리 용량
  handlingCapacity: {
    peakHour: number;               // % (목표: > 15%)
    averageHour: number;            // %
  };

  // 가동률
  availability: {
    uptime: number;                 // % (목표: > 99.5%)
    mtbf: number;                   // 시간 (평균 고장 간격)
    mttr: number;                   // 시간 (평균 수리 시간)
  };
}
```

### 13.2 에너지 효율성 KPI

```typescript
interface EnergyKPIs {
  // 에너지 소비
  consumption: {
    dailyTotal: number;             // kWh/일
    perTripAverage: number;         // kWh/운행
    comparedToBaseline: number;     // % (절감률)
  };

  // 재생 에너지
  regeneration: {
    dailyRegeneration: number;      // kWh/일
    regenerationRate: number;       // % (목표: > 20%)
    annualSavings: number;          // KRW/년
  };

  // 환경 영향
  environmental: {
    co2Emissions: number;           // kg CO₂/년
    co2Reduction: number;           // %
    energyRating: string;           // A+ ~ E
  };
}
```

### 13.3 안전 및 만족도 KPI

```typescript
interface SafetyAndSatisfactionKPIs {
  // 안전 지표
  safety: {
    incidentCount: number;          // 건/년 (목표: 0)
    entrapmentCount: number;        // 건/년 (목표: < 2)
    averageRescueTime: number;      // 분 (목표: < 30)
    safetyInspectionPass: number;   // % (목표: 100%)
  };

  // 승객 만족도
  satisfaction: {
    overallRating: number;          // /5 (목표: > 4.0)
    waitTimeRating: number;         // /5
    comfortRating: number;          // /5
    cleanlinessRating: number;      // /5
    accessibilityRating: number;    // /5
  };

  // 정비 품질
  maintenance: {
    scheduledComplianceRate: number; // % (목표: > 95%)
    predictiveAccuracy: number;      // % (예측 정확도)
    costPerElevator: number;         // KRW/년
    sparepartInventoryTurnover: number;
  };
}
```

---

## 14. 보안 및 개인정보보호

### 14.1 보안 시스템

```typescript
interface SecuritySystem {
  // 접근 제어
  accessControl: {
    cardReader: boolean;            // RFID/NFC
    biometric: boolean;             // 생체 인식
    pinCode: boolean;
    timeBasedAccess: boolean;       // 시간대별 제어
    floorRestriction: boolean;      // 층별 권한
  };

  // 감시 시스템
  surveillance: {
    cctv: boolean;                  // 카 내부 CCTV
    recordingRetention: number;     // 일 (보관 기간)
    liveMonitoring: boolean;
    motionDetection: boolean;
  };

  // 사이버 보안
  cybersecurity: {
    networkEncryption: 'tls-1.3';
    authentication: 'oauth-2.0';
    firmwareVerification: boolean;
    intrusionDetection: boolean;
    regularSecurityAudit: boolean;
  };

  // 개인정보 보호
  privacy: {
    gdprCompliance: boolean;
    dataMinimization: boolean;
    anonymization: boolean;
    consentManagement: boolean;
    rightToDeletion: boolean;
  };
}
```

### 14.2 데이터 보안

- **전송 암호화**: TLS 1.3
- **저장 암호화**: AES-256
- **접근 권한**: RBAC (역할 기반 접근 제어)
- **감사 로그**: 모든 시스템 접근 기록
- **침입 탐지**: 실시간 모니터링
- **정기 보안 감사**: 분기 1회

---

## 15. 인증 요구사항

### 15.1 필수 인증

- **안전 인증**
  - EN 81-1/81-2 (유럽)
  - ASME A17.1 (미국)
  - ISO 8100 (국제)
  - KS B 6900 (한국)

- **품질 인증**
  - ISO 9001 (품질 경영)
  - ISO 14001 (환경 경영)
  - ISO 45001 (안전보건 경영)

- **에너지 인증**
  - ISO 25745-1/2/3 (에너지 성능)
  - LEED 기여 (그린빌딩)
  - BREEAM 인증

### 15.2 접근성 인증

- **ADA 준수** (미국 장애인법)
- **EN 81-70** (유럽 접근성 규정)
- **장애인·노인·임산부 등의 편의증진 보장에 관한 법률** (한국)

### 15.3 WIA 인증 프로세스

```typescript
interface WIACertification {
  // 1단계: 설계 검토
  designReview: {
    specification: boolean;
    safetyAnalysis: boolean;
    energyEfficiency: boolean;
    accessibility: boolean;
  };

  // 2단계: 제조 감사
  manufacturingAudit: {
    qualityControl: boolean;
    materialVerification: boolean;
    processCompliance: boolean;
  };

  // 3단계: 성능 시험
  performanceTest: {
    safetyTest: boolean;
    loadTest: boolean;
    speedTest: boolean;
    energyTest: boolean;
    noiseTest: boolean;
  };

  // 4단계: 현장 검증
  fieldValidation: {
    installationInspection: boolean;
    commissioning: boolean;
    performanceMonitoring: number;  // 일 (모니터링 기간)
  };

  // 5단계: 인증 발급
  certification: {
    certificationLevel: 'gold' | 'silver' | 'bronze';
    validityPeriod: number;         // 년
    auditFrequency: string;         // '연 1회'
  };
}
```

---

## 부록 A: 데이터 포맷 예시

### A.1 엘리베이터 상태 메시지

```json
{
  "elevatorId": "BLDG-A-ELV-01",
  "timestamp": "2025-12-25T10:30:00Z",
  "type": "traction",
  "status": {
    "operational": "running",
    "currentFloor": 15,
    "direction": "up",
    "speed": 2.5,
    "load": {
      "current": 450,
      "rated": 1000,
      "percentage": 45
    }
  },
  "performance": {
    "waitingTime": {
      "average": 25,
      "last10Calls": [20, 18, 30, 25, 22, 28, 24, 26, 19, 31]
    },
    "energyConsumption": {
      "current": 8.5,
      "today": 45.2,
      "regenerated": 12.3
    }
  },
  "safety": {
    "doorStatus": "closed",
    "brakeStatus": "normal",
    "safetyCircuit": "normal",
    "emergencyStop": false
  },
  "maintenance": {
    "lastService": "2025-12-15",
    "nextService": "2026-01-15",
    "alerts": []
  }
}
```

### A.2 호출 요청 메시지

```json
{
  "callId": "CALL-20251225-001234",
  "timestamp": "2025-12-25T10:30:15Z",
  "type": "destination-dispatch",
  "request": {
    "fromFloor": 1,
    "toFloor": 25,
    "passengerCount": 2,
    "accessibility": {
      "wheelchair": false,
      "visuallyImpaired": false
    },
    "priority": "normal"
  },
  "assignment": {
    "assignedCar": "BLDG-A-ELV-03",
    "estimatedArrival": 35,
    "estimatedTravelTime": 45
  }
}
```

---

## 부록 B: API 엔드포인트

### B.1 엘리베이터 제어 API

- `GET /api/v1/elevators` - 엘리베이터 목록
- `GET /api/v1/elevators/{id}` - 특정 엘리베이터 상태
- `POST /api/v1/elevators/{id}/call` - 호출 요청
- `PUT /api/v1/elevators/{id}/mode` - 운영 모드 변경
- `GET /api/v1/elevators/{id}/status` - 실시간 상태

### B.2 군관제 API

- `GET /api/v1/group-control/status` - 군관제 상태
- `PUT /api/v1/group-control/algorithm` - 알고리즘 설정
- `GET /api/v1/group-control/metrics` - 성능 지표
- `POST /api/v1/group-control/optimize` - 최적화 실행

### B.3 정비 관리 API

- `GET /api/v1/maintenance/schedule` - 정비 일정
- `POST /api/v1/maintenance/alert` - 정비 알림
- `GET /api/v1/maintenance/history` - 정비 이력
- `GET /api/v1/maintenance/prediction` - 예측 정비

---

## 변경 이력

### Version 1.0.0 (2025-12-25)

- 초판 발행
- 완전한 엘리베이터 시스템 표준 정의
- 군관제, 목적층 예약, 예측 정비 포함
- IoT 센서 통합 및 에너지 재생 시스템 정의

---

## 라이선스

이 표준은 [MIT License](https://opensource.org/licenses/MIT) 하에 배포됩니다.

---

## 弘익人間 (홍익인간) · 널리 인간을 이롭게 하라

WIA-CITY-012 엘리베이터 시스템 표준은 弘益人間(홍익인간)의 정신을 구현합니다. 지능형 엘리베이터 시스템을 통해 모든 사람이 안전하고 효율적이며 편리하게 수직 이동할 수 있도록 보장합니다.

안전, 효율, 접근성, 지속가능성을 통해 스마트빌딩과 스마트시티의 미래를 함께 만들어갑니다.

**함께, 우리는 더 나은 도시 환경을 만들어갑니다.**

---

© 2025 SmileStory Inc. / WIA
**弘益人間 (홍익인간) · Benefit All Humanity**
