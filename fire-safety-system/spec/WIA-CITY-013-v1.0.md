# WIA-CITY-013: 화재 안전 시스템 표준 🔥

> **Version:** 1.0.0
> **발행일:** 2025-12-25
> **상태:** Active
> **弘益人間 (홍익인간) - 널리 인간을 이롭게 하라**

---

## 목차

1. [개요](#1-개요)
2. [표준의 목적](#2-표준의-목적)
3. [적용 범위](#3-적용-범위)
4. [용어 정의](#4-용어-정의)
5. [화재 감지 센서](#5-화재-감지-센서)
6. [스프링클러 시스템](#6-스프링클러-시스템)
7. [화재 경보 및 알림](#7-화재-경보-및-알림)
8. [대피 경로 관리](#8-대피-경로-관리)
9. [소화 장비](#9-소화-장비)
10. [방화 구획](#10-방화-구획)
11. [시스템 연동](#11-시스템-연동)
12. [데이터 형식 표준](#12-데이터-형식-표준)
13. [API 인터페이스](#13-api-인터페이스)
14. [보안 및 인증](#14-보안-및-인증)
15. [성능 요구사항](#15-성능-요구사항)
16. [부록](#16-부록)

---

## 1. 개요

### 1.1 배경

화재는 전 세계적으로 매년 수백만 건 발생하며, 인명 피해와 재산 손실을 초래합니다. 2024년 기준:

- **전 세계 화재 발생**: 약 700만 건/년
- **사망자**: 약 30만 명/년
- **재산 피해**: 약 $3,000억/년
- **부상자**: 약 150만 명/년

### 1.2 문제점

현재 화재 안전 시스템의 주요 문제점:

1. **감지 지연**: 화재 초기 단계 감지 실패로 인한 확산
2. **경보 부족**: 취약 계층(청각 장애인, 외국인)에 대한 접근성 부족
3. **대피 혼란**: 명확한 대피 경로 안내 부재
4. **시스템 분리**: 감지, 경보, 소화 시스템의 비통합
5. **유지보수 소홀**: 정기 점검 및 관리 미흡

### 1.3 WIA-CITY-013의 가치

이 표준은 다음을 제공합니다:

- ✅ **다층 감지**: 연기, 열, 화염, CO 등 다중 센서 활용
- ✅ **즉각 경보**: 음향, 시각, 문자 등 다양한 경보 수단
- ✅ **자동 대응**: 스프링클러 자동 작동, HVAC 제어
- ✅ **안전 대피**: 실시간 최적 경로 안내
- ✅ **완전 통합**: 모든 시스템의 유기적 연동

---

## 2. 표준의 목적

### 2.1 핵심 목표

1. **생명 보호**: 화재로부터 인명 피해 최소화
2. **재산 보호**: 화재로 인한 재산 손실 감소
3. **조기 감지**: 화재 초기 단계에서 신속한 감지
4. **신속 대응**: 자동화된 소화 및 대피 시스템
5. **상호운용성**: 다양한 시스템 간 통합 및 연동

### 2.2 기대 효과

- **생명**: 화재 사망률 50% 감소
- **재산**: 재산 피해 60% 감소
- **대응**: 초기 대응 시간 5분 → 2분 단축
- **부상**: 화재 부상자 70% 감소
- **비용**: 보험료 20-30% 절감

---

## 3. 적용 범위

### 3.1 대상 건물

이 표준은 다음 건물에 적용됩니다:

#### 주거 시설
- 아파트, 빌라, 연립주택
- 기숙사, 호텔, 모텔
- 노인 요양 시설, 장애인 시설

#### 상업 시설
- 사무실 빌딩, 쇼핑몰
- 음식점, 카페
- 극장, 공연장

#### 공공 시설
- 학교, 대학교
- 병원, 보건소
- 도서관, 박물관

#### 산업 시설
- 공장, 창고
- 데이터센터
- 발전소

### 3.2 제외 대상

- 단독 주택 (별도 기준 적용)
- 이동식 구조물
- 임시 건축물 (6개월 미만)

---

## 4. 용어 정의

### 4.1 일반 용어

**화재 (Fire)**
- 인간의 의도와 무관하게 발생하거나 확산되어 소화의 필요가 있는 연소 현상

**화재 등급 (Fire Class)**
- A급: 일반 화재 (목재, 종이, 섬유)
- B급: 유류 화재 (휘발유, 오일)
- C급: 전기 화재
- D급: 금속 화재
- K급: 주방 화재 (식용유)

**방화 (Fire Prevention)**
- 화재 발생을 사전에 방지하는 모든 활동

**진화 (Fire Suppression)**
- 발생한 화재를 소화하는 활동

### 4.2 기술 용어

**연기 농도 (Smoke Density)**
- 단위 체적당 연기 입자의 농도, ppm 또는 obscuration/ft로 측정

**열 방출률 (Heat Release Rate, HRR)**
- 단위 시간당 화재에서 방출되는 열량, kW 단위

**플래시오버 (Flashover)**
- 화재가 급격히 확산되어 실내 전체가 동시 착화되는 현상

**백드래프트 (Backdraft)**
- 산소가 부족한 밀폐 공간에 갑자기 산소가 공급되어 폭발적 연소가 발생하는 현상

**FRR (Fire Resistance Rating)**
- 화재에 견딜 수 있는 시간, 30분/60분/90분/120분 등급

---

## 5. 화재 감지 센서

### 5.1 연기 감지기

#### 5.1.1 광전식 연기 감지기

**원리**
- 광원에서 나온 빛이 연기 입자에 산란되어 수광부에 도달하는 양을 측정
- 산란광 방식 (forward scatter)과 차광 방식 (obscuration) 존재

**성능 기준**
```json
{
  "sensitivity": {
    "min": "0.5% obscuration/ft",
    "max": "4.0% obscuration/ft"
  },
  "responseTime": {
    "target": "< 30 seconds",
    "excellent": "< 15 seconds"
  },
  "coverage": {
    "ceiling_height_3m": "60 m²/detector",
    "ceiling_height_6m": "40 m²/detector"
  }
}
```

**적용 장소**
- 침실, 거실, 복도
- 느린 연소 화재에 효과적
- 먼지가 적은 환경

#### 5.1.2 이온화식 연기 감지기

**원리**
- 방사성 물질(Am-241)로 공기를 이온화
- 연기 입자가 이온화된 공기의 전류를 감소시킴을 감지

**성능 기준**
```json
{
  "sensitivity": {
    "min": "1.0% obscuration/ft",
    "max": "4.0% obscuration/ft"
  },
  "responseTime": {
    "target": "< 20 seconds",
    "excellent": "< 10 seconds"
  },
  "coverage": "60 m²/detector"
}
```

**적용 장소**
- 주방, 보일러실
- 빠른 연소 화재에 효과적
- 환기가 잘 되는 공간

#### 5.1.3 아날로그 어드레서블 연기 감지기

**특징**
- 연기 농도를 아날로그 값으로 전송
- 오염도 보상 기능
- 자동 감도 조정

**데이터 모델**
```typescript
interface SmokeDetector {
  sensorId: string;
  type: 'PHOTOELECTRIC' | 'IONIZATION' | 'ANALOG_ADDRESSABLE';
  location: {
    building: string;
    floor: string;
    zone: string;
    coordinates: { lat: number; lon: number };
  };
  readings: {
    smokeLevel_ppm: number;
    visibility_m: number;
    trend: 'INCREASING' | 'STABLE' | 'DECREASING';
  };
  status: 'NORMAL' | 'ALERT' | 'WARNING' | 'FAULT';
  batteryLevel_percent?: number;
  lastMaintenance: string; // ISO 8601
  nextMaintenance: string; // ISO 8601
}
```

### 5.2 열 감지기

#### 5.2.1 정온식 열 감지기

**원리**
- 설정된 온도에 도달하면 작동
- 바이메탈, 퓨즈블 링크 방식

**등급**
| 등급 | 작동 온도 | 색상 코드 | 적용 장소 |
|------|----------|----------|-----------|
| 보통 | 57-77°C | 주황색 | 일반 사무실, 주거 |
| 중간 | 79-107°C | 빨간색 | 주방, 세탁실 |
| 고온 | 121-149°C | 노란색 | 보일러실, 건조실 |
| 초고온 | 163-260°C | 파란색 | 용광로 근처 |

#### 5.2.2 차동식 열 감지기

**원리**
- 온도 상승률을 감지
- 일반적으로 8-10°C/분 이상 상승 시 작동

**성능 기준**
```json
{
  "rateOfRise": {
    "threshold": "8-10°C/min",
    "measurement_interval": "1 second"
  },
  "responseTime": "< 60 seconds",
  "coverage": "40 m²/detector"
}
```

**데이터 모델**
```typescript
interface HeatDetector {
  sensorId: string;
  type: 'FIXED_TEMPERATURE' | 'RATE_OF_RISE' | 'COMBINATION';
  location: Location;
  readings: {
    temperature_C: number;
    threshold_C: number;
    rateOfRise_C_per_min?: number;
  };
  status: SensorStatus;
  calibrationDate: string;
  nextCalibration: string;
}
```

### 5.3 화염 감지기

#### 5.3.1 UV 화염 감지기

**원리**
- 185-260 nm 파장의 자외선 감지
- 화염의 UV 복사 검출

**성능**
- 응답 시간: < 0.1초
- 감지 범위: 10-30m
- 시야각: 90-120°

#### 5.3.2 IR 화염 감지기

**원리**
- 4.3-4.4 μm 파장의 적외선 감지
- 화염의 깜빡임 주파수 분석 (1-20 Hz)

**성능**
- 응답 시간: < 5초
- 감지 범위: 15-50m
- 시야각: 90-180°

#### 5.3.3 UV/IR 복합 감지기

**특징**
- UV와 IR 동시 감지로 오경보 최소화
- AND 로직: 두 조건 모두 만족 시 경보

**데이터 모델**
```typescript
interface FlameDetector {
  sensorId: string;
  type: 'UV' | 'IR' | 'UV_IR' | 'TRIPLE_IR';
  location: Location;
  readings: {
    uvIntensity?: number;
    irIntensity?: number;
    flickerFrequency_Hz?: number;
    flameDetected: boolean;
  };
  detectionRange_m: number;
  fieldOfView_degrees: number;
  status: SensorStatus;
}
```

### 5.4 일산화탄소 감지기

#### 5.4.1 전기화학식 CO 감지기

**원리**
- CO가 전극에서 산화되면서 발생하는 전류 측정
- 전류의 크기가 CO 농도에 비례

**경보 기준**
| CO 농도 (ppm) | 노출 시간 | 조치 |
|--------------|----------|------|
| 30 | 30일 | 장기 노출 경고 |
| 50 | 8시간 | 경보 |
| 100 | 90분 | 경보 |
| 200 | 35분 | 즉시 경보 |
| 400 | 즉시 | 긴급 경보 + 대피 |

**데이터 모델**
```typescript
interface CODetector {
  sensorId: string;
  type: 'ELECTROCHEMICAL' | 'METAL_OXIDE' | 'BIOMIMETIC';
  location: Location;
  readings: {
    coLevel_ppm: number;
    threshold_ppm: number;
    peakLevel_ppm?: number;
    exposureTime_minutes?: number;
  };
  status: SensorStatus;
  sensorLifeRemaining_days: number;
}
```

### 5.5 센서 배치 기준

#### 5.5.1 천장 높이별 배치

**3m 이하**
- 연기 감지기: 60 m²/개
- 열 감지기: 40 m²/개

**3-6m**
- 연기 감지기: 40 m²/개
- 열 감지기: 30 m²/개

**6-9m**
- 연기 감지기: 30 m²/개 (고감도)
- 열 감지기: 사용 불가 (연기 감지기 권장)

#### 5.5.2 특수 공간

**복도**
- 15m 간격
- 막다른 곳에서 7.5m 이내

**계단실**
- 각 층마다 최소 1개
- 최상층에 추가 설치

**엘리베이터 샤프트**
- 최상부 및 최하부
- 승강기 기계실

---

## 6. 스프링클러 시스템

### 6.1 습식 스프링클러 시스템

#### 6.1.1 구성 요소

**급수원**
- 상수도 직결 또는 고가 수조
- 최소 용량: 전체 헤드 개수의 30% × 60분

**가압 펌프**
- 주펌프 + 예비 펌프 (자동 전환)
- 압력: 최소 7 bar
- 유량: 최대 방호 구역 요구량의 150%

**배관**
- 주배관: 강관 또는 CPVC (내화)
- 지름: 수리 계산으로 결정
- 경사: 1/250 이상 (배수 고려)

**스프링클러 헤드**
```typescript
interface SprinklerHead {
  headId: string;
  type: 'PENDANT' | 'UPRIGHT' | 'SIDEWALL' | 'CONCEALED';
  responseType: 'QUICK' | 'STANDARD' | 'SPECIAL';
  temperatureRating_C: 57 | 68 | 79 | 93 | 107 | 121 | 141 | 182;
  kFactor: 80 | 115 | 160 | 200 | 240; // L/min/√bar
  coverageArea_m2: number;
  location: Location;
  installDate: string;
  lastInspection: string;
}
```

#### 6.1.2 설계 기준

**방호 면적**
| 건물 용도 | 위험도 | 방수 밀도 (mm/min) | 방호 면적 (m²) |
|----------|--------|-------------------|---------------|
| 주거 | 경위험 | 2.5 | 84 |
| 사무실 | 경위험 | 5.0 | 232 |
| 상업 | 중위험 | 7.5 | 186 |
| 창고 | 고위험 | 12.5 | 139 |

**작동 헤드 개수**
- 경위험: 4개
- 중위험: 6개
- 고위험: 12개

**방수량 계산**
```
Q = K × √P
여기서:
Q = 유량 (L/min)
K = K-Factor (80, 115, 160, 200, 240)
P = 압력 (bar)
```

### 6.2 건식 스프링클러 시스템

#### 6.2.1 작동 원리

1. 배관에 압축 공기 또는 질소 충전 (0.7-4 bar)
2. 화재 시 스프링클러 헤드 개방
3. 공기 배출로 압력 저하
4. 건식 밸브 개방
5. 물 방출 (지연 시간: 30-60초)

#### 6.2.2 적용

**필수 적용**
- 동결 위험 구역 (0°C 이하)
- 비난방 창고, 주차장
- 냉동 창고

**데이터 모델**
```typescript
interface DryPipeSystem {
  systemId: string;
  airPressure_bar: number;
  waterPressure_bar: number;
  valveStatus: 'NORMAL' | 'TRIPPED' | 'FAULT';
  pipeLength_m: number;
  estimatedDelayTime_sec: number;
  lowAirPressureAlarm: boolean;
  supervisorySignals: {
    lowAirPressure: boolean;
    valveOpen: boolean;
    compressorFault: boolean;
  };
}
```

### 6.3 준비작동식 시스템

#### 6.3.1 이중 연동 방식

**Single Interlock**
- 화재 감지기 작동 → 밸브 개방
- 스프링클러 헤드 개방 → 방수

**Double Interlock**
- 화재 감지기 작동 + 스프링클러 헤드 개방 → 방수
- 오작동 방지 극대화

#### 6.3.2 적용

- 데이터센터
- 서버실
- 박물관, 미술관
- 도서관, 기록 보관소
- 고가 장비 시설

### 6.4 일제살수식 시스템

#### 6.4.1 특징

- 모든 헤드가 개방형 (오리피스만 존재)
- 델루지 밸브로 일괄 제어
- 화재 감지기 작동 시 전체 구역 일제 방수

#### 6.4.2 적용

- 항공기 격납고
- 화학 공장
- 유류 저장 시설
- 변압기실

**방수 밀도**
- 표준: 10 mm/min
- 고위험: 20 mm/min 이상

---

## 7. 화재 경보 및 알림

### 7.1 음향 경보

#### 7.1.1 사이렌

**성능 기준**
```json
{
  "soundLevel": {
    "min": "75 dBA @ 3m",
    "max": "120 dBA",
    "background_plus": "15 dBA"
  },
  "frequency": {
    "min": "500 Hz",
    "max": "2500 Hz",
    "sweep": true
  },
  "pattern": {
    "temporal_3": "0.5s on, 0.5s off, 0.5s on, 0.5s off, 0.5s on, 1.5s off"
  }
}
```

#### 7.1.2 벨

**성능 기준**
- 음압: 최소 75 dBA @ 3m
- 펄스: 1초 온, 1초 오프
- 주파수: 400-600 Hz

#### 7.1.3 배치 기준

- 복도: 30m 간격
- 실내: 모든 위치에서 청취 가능
- 수면 공간: 75 dBA @ 침대 위치
- 소음 환경: 배경 소음 + 15 dBA 이상

### 7.2 시각 경보

#### 7.2.1 스트로브 라이트

**성능 기준**
```typescript
interface VisualAlarm {
  type: 'STROBE' | 'LED_PANEL';
  flashRate_Hz: 1 | 2; // 1-2 Hz (60-120 fpm)
  intensity_cd: number; // 15-177 candela
  color: 'CLEAR' | 'RED' | 'AMBER';
  coverage_m2: number;
  location: Location;
  powerBackup: boolean;
  batteryDuration_hours: number;
}
```

**광도 기준 (방 크기별)**
| 방 크기 (m²) | 최소 광도 (cd) |
|-------------|---------------|
| < 10 | 15 |
| 10-20 | 30 |
| 20-40 | 60 |
| 40-80 | 95 |
| > 80 | 110-177 |

#### 7.2.2 배치 기준

- 복도: 30m 간격
- 대공간: 벽면에서 15m 이내
- 높이: 2.0-2.4m (바닥에서)
- 시야 확보: 모든 위치에서 최소 1개 이상 보임

### 7.3 음성 경보

#### 7.3.1 시스템 구성

**마이크로폰/앰프**
- 소방대원 수동 방송
- 사전 녹음 메시지 자동 재생

**스피커**
- 음압: 최소 75 dBA @ 3m
- 주파수 응답: 250-4,000 Hz
- 명료도: 0.7 이상 (CIS)

**메시지 예시 (한국어)**
```
"화재가 발생했습니다. 침착하게 가장 가까운 비상구로 대피하시기 바랍니다.
엘리베이터는 절대 이용하지 마십시오.
계단을 이용하여 건물 밖으로 나가십시오."
```

**메시지 예시 (영어)**
```
"Fire alarm activated. Please evacuate calmly using the nearest exit.
Do not use elevators. Use stairs to exit the building."
```

#### 7.3.2 다국어 지원

- 한국어 (기본)
- 영어
- 중국어
- 일본어
- 기타 (건물 특성에 따라)

### 7.4 문자 알림

#### 7.4.1 알림 채널

**SMS**
```json
{
  "recipients": ["건물 관리자", "소방 안전 담당자", "경비실"],
  "message": "화재 경보 작동 - [건물명] [위치] - 즉시 확인 요망",
  "priority": "URGENT",
  "deliveryTime_sec": "< 30"
}
```

**모바일 앱 푸시**
```typescript
interface PushNotification {
  title: string;
  body: string;
  priority: 'HIGH' | 'URGENT';
  sound: 'ALERT' | 'CRITICAL';
  data: {
    incidentId: string;
    location: Location;
    severity: 'WARNING' | 'CRITICAL' | 'EMERGENCY';
    actions: ['ACKNOWLEDGE', 'RESPOND', 'EVACUATE'];
  };
}
```

**이메일**
- 상세 정보 포함
- 첨부: 센서 데이터, 영상 (CCTV)
- 수신: 관리자, 소방서, 보험사

---

## 8. 대피 경로 관리

### 8.1 비상구

#### 8.1.1 출구 종류

**주 출구 (Main Exit)**
- 폭: 최소 1.8m
- 수용 능력: 100명/분
- 일상적 출입구 겸용

**비상구 (Emergency Exit)**
- 폭: 최소 0.9m (1인용), 1.4m (2인용)
- 수용 능력: 50명/분 (0.9m), 100명/분 (1.4m)
- 화재 시 전용

**화재 대피 계단 (Fire Escape Stair)**
- 내화 구조: 최소 2시간
- 폭: 최소 1.1m
- 방연 설비
- 비상 조명

#### 8.1.2 비상구 요구사항

**개구부 크기**
- 최소 높이: 2.0m
- 최소 폭: 0.9m
- 최소 면적: 1.8 m²

**문 (Door)**
- 개방 방향: 대피 방향 (밖으로)
- 잠금 장치: 내부에서 항상 개방 가능
- 패닉 바 (Panic Bar): 5 kg 이하 힘으로 개방
- 자동 폐쇄: 방화문은 자동 폐쇄 (화재 시)

**유도등 (Exit Sign)**
```typescript
interface ExitSign {
  signId: string;
  type: 'ILLUMINATED' | 'SELF_LUMINOUS';
  location: Location;
  message: string; // "EXIT", "비상구", "EXIT 出口"
  color: 'GREEN' | 'RED'; // 한국: 녹색, 일부 국가: 적색
  brightness_cd_m2: number; // 최소 50 cd/m²
  visibility_m: number; // 최소 30m
  powerBackup: {
    batteryType: 'LEAD_ACID' | 'LITHIUM' | 'NICAD';
    duration_hours: number; // 최소 90분
  };
  lastTest: string;
  nextTest: string;
}
```

### 8.2 대피 경로 설계

#### 8.2.1 이동 거리

**허용 이동 거리**
| 건물 용도 | 스프링클러 X | 스프링클러 O |
|----------|-------------|-------------|
| 주거 | 30m | 45m |
| 사무실 | 45m | 60m |
| 상업 | 40m | 55m |
| 고위험 | 23m | 30m |

#### 8.2.2 통행 능력

**계단 폭별 수용 능력**
| 계단 폭 (m) | 수용 인원 (명/분) |
|-----------|------------------|
| 0.9 | 40 |
| 1.1 | 60 |
| 1.4 | 100 |
| 1.8 | 140 |

**복도 폭**
- 최소: 1.2m (양방향 통행)
- 권장: 1.8m (원활한 통행)
- 고밀도: 2.4m 이상

#### 8.2.3 대피 시간 계산

**공식**
```
T_total = T_detection + T_alarm + T_decision + T_movement

여기서:
T_detection = 화재 감지 시간 (목표: < 30초)
T_alarm = 경보 전파 시간 (목표: < 10초)
T_decision = 의사 결정 시간 (평균: 30-60초)
T_movement = 이동 시간 (거리 / 속도)

이동 속도:
- 복도: 1.2 m/s
- 계단 하향: 0.6 m/s
- 계단 상향: 0.5 m/s
```

### 8.3 대피 경로 안내 시스템

#### 8.3.1 동적 경로 안내

**실시간 경로 계산**
```typescript
interface DynamicEvacuationRoute {
  routeId: string;
  startLocation: Location;
  endLocation: Location;
  waypoints: Location[];
  distance_m: number;
  estimatedTime_sec: number;
  congestionLevel: 'LOW' | 'MEDIUM' | 'HIGH';
  hazards: Array<{
    type: 'FIRE' | 'SMOKE' | 'BLOCKED';
    location: Location;
    severity: 'LOW' | 'HIGH';
  }>;
  alternativeRoutes: string[]; // 대체 경로 IDs
  accessibility: {
    wheelchairAccessible: boolean;
    elevatorRequired: boolean;
    rampAvailable: boolean;
  };
}
```

#### 8.3.2 디지털 사이니지

**디스플레이 정보**
- 현재 위치
- 최적 대피 방향 (화살표)
- 출구까지 거리
- 예상 소요 시간
- 위험 구역 표시

**기술 사양**
- 크기: 최소 32인치
- 해상도: Full HD (1920×1080)
- 밝기: 500 nits 이상
- 응답 시간: < 1초
- 전원: UPS 백업 (2시간)

---

## 9. 소화 장비

### 9.1 소화기

#### 9.1.1 종류 및 적용

**물 소화기 (Water Extinguisher)**
```typescript
interface WaterExtinguisher {
  type: 'WATER_JET' | 'WATER_SPRAY' | 'WATER_MIST';
  capacity_L: 6 | 9;
  range_m: 10;
  dischargeTime_sec: 40;
  applicableFireClass: ['CLASS_A'];
  weight_kg: 10;
  testPressure_bar: 25;
}
```

**포말 소화기 (Foam Extinguisher)**
- 적용: A급, B급
- 용량: 6L, 9L
- 포 팽창비: 10:1
- 방사 거리: 6-8m

**분말 소화기 (Dry Powder Extinguisher)**
```typescript
interface DryPowderExtinguisher {
  type: 'ABC' | 'BC' | 'D';
  capacity_kg: 3 | 5 | 10;
  agent: 'MONOAMMONIUM_PHOSPHATE' | 'SODIUM_BICARBONATE' | 'POTASSIUM_BICARBONATE';
  applicableFireClass: ['CLASS_A', 'CLASS_B', 'CLASS_C'];
  range_m: 5;
  dischargeTime_sec: 15;
}
```

**CO₂ 소화기 (Carbon Dioxide Extinguisher)**
- 적용: B급, C급 (전기 화재)
- 용량: 2kg, 5kg
- 방사 거리: 1-2m (근거리)
- 질식 효과

**청정 소화기 (Clean Agent Extinguisher)**
- 적용: A급, B급, C급
- 약제: Halon 대체제 (FM-200, Novec 1230)
- 전기 전도성: 없음
- 잔류물: 없음

#### 9.1.2 배치 기준

**개수**
- A급 위험: 200 m² 당 1개
- B급 위험: 75 m² 당 1개

**이동 거리**
- 일반: 최대 23m
- 고위험: 최대 15m

**설치 높이**
- 최대 중량 18kg 이하: 바닥에서 1.5m 이내
- 중량 18kg 초과: 바닥에서 1.0m 이내

#### 9.1.3 점검 및 관리

**월간 점검 (육안)**
- 위치 및 접근성
- 압력 게이지 (녹색 범위)
- 물리적 손상
- 봉인 상태

**연간 점검 (전문가)**
- 내부 검사
- 압력 테스트
- 약제 교체 (필요 시)
- 호스/노즐 점검

**데이터 모델**
```typescript
interface ExtinguisherInspection {
  inspectionId: string;
  extinguisherId: string;
  inspectionDate: string;
  inspector: string;
  checklist: {
    locationAccessible: boolean;
    pressureGaugeNormal: boolean;
    sealIntact: boolean;
    physicalDamageNone: boolean;
    signageVisible: boolean;
  };
  maintenanceActions: Array<{
    action: 'RECHARGED' | 'REPLACED' | 'REPAIRED' | 'REMOVED';
    reason: string;
    date: string;
  }>;
  nextInspection: string;
  passed: boolean;
}
```

### 9.2 소화전

#### 9.2.1 실내 소화전

**구성**
- 호스: 15m 또는 25m
- 노즐: 직사/무상 겸용 (13mm, 16mm)
- 밸브: 각도 밸브 또는 볼 밸브
- 호스 릴 또는 호스 랙

**수압 및 유량**
```json
{
  "pressure": {
    "min_bar": 3.5,
    "nominal_bar": 5.0,
    "max_bar": 7.0
  },
  "flowRate": {
    "13mm_nozzle_lpm": 130,
    "16mm_nozzle_lpm": 190
  },
  "simultaneousOperation": 2
}
```

**배치**
- 보행 거리: 최대 40m
- 설치 높이: 바닥에서 1.5m (호스 연결구 중심)

#### 9.2.2 옥외 소화전

**지상식**
- 연결구: 65mm (2개)
- 높이: 지면에서 60cm
- 색상: 빨강 또는 노랑
- 동결 방지

**지하식**
- 맨홀 덮개
- 연결구: 65mm 또는 100mm
- 표시: "소화전" 노면 표시

**수압 및 유량**
```json
{
  "pressure_bar": 4.5,
  "flowRate_lpm": 500,
  "simultaneousOperation": 3
}
```

---

## 10. 방화 구획

### 10.1 방화문

#### 10.1.1 내화 성능

**FRR-30 (30분 내화)**
- 구조: 철재 또는 목재 (내화 처리)
- 적용: 일반 사무실 간 칸막이
- 시험: ISO 834 또는 KS F 2845

**FRR-60 (60분 내화)**
- 구조: 철재 + 단열재
- 적용: 계단실, 복도 구획
- 두께: 최소 50mm

**FRR-90 (90분 내화)**
- 구조: 철재 + 고단열재
- 적용: 수직 샤프트, 위험물 저장실
- 두께: 최소 70mm

**FRR-120 (120분 내화)**
- 구조: 철재 + 특수 단열재
- 적용: 특별 방호 구역
- 두께: 최소 90mm

#### 10.1.2 자동 폐쇄 장치

**유압식 도어 클로저**
```typescript
interface DoorCloser {
  type: 'HYDRAULIC' | 'PNEUMATIC' | 'SPRING';
  closingForce_N: number; // 65-85 N 권장
  closingSpeed: {
    initial: number; // mm/s
    final: number; // mm/s (마지막 10cm 빠르게)
  };
  holdOpenAngle_degrees?: number;
  adjustable: boolean;
}
```

**전자기식 도어 홀더**
```typescript
interface MagneticDoorHolder {
  holdingForce_N: number; // 200-400 N
  releaseSignal: 'FIRE_ALARM' | 'MANUAL' | 'POWER_FAILURE';
  fallbackMode: 'FAIL_SAFE'; // 정전 시 자동 해제
  integration: {
    firePanelConnected: boolean;
    bmsConnected: boolean;
  };
}
```

#### 10.1.3 데이터 모델

```typescript
interface FireDoor {
  doorId: string;
  location: Location;
  fireRating: 'FRR_30' | 'FRR_60' | 'FRR_90' | 'FRR_120';
  type: 'SINGLE_SWING' | 'DOUBLE_SWING' | 'SLIDING' | 'ROLLING';
  status: 'OPEN' | 'CLOSED' | 'AUTOMATIC' | 'FAULT';
  doorCloser: {
    type: string;
    functional: boolean;
    lastService: string;
  };
  holdOpenDevice?: {
    active: boolean;
    releaseCondition: string[];
  };
  sensors: {
    positionSensor: boolean;
    obstructionSensor: boolean;
  };
  inspection: {
    lastInspection: string;
    nextInspection: string;
    defects: string[];
  };
}
```

### 10.2 방화벽

#### 10.2.1 내화 성능

**2시간 내화 (일반)**
- 콘크리트 블록: 200mm
- 철근 콘크리트: 150mm
- 경량 내화 보드: 이중 레이어

**3-4시간 내화 (특수)**
- 철근 콘크리트: 200mm+
- 특수 내화 벽돌

#### 10.2.2 구조적 요구사항

**독립성**
- 바닥부터 지붕까지 연속
- 천장 위 공간도 차단
- 외벽 관통 시 60cm 이상 돌출

**개구부**
- 최소화
- 방화문 또는 방화 댐퍼로 차단

### 10.3 방화 구획

#### 10.3.1 구획 면적

**건물 용도별**
| 용도 | 스프링클러 X | 스프링클러 O |
|------|-------------|-------------|
| 주거 | 500 m² | 1,000 m² |
| 사무실 | 1,000 m² | 3,000 m² |
| 판매시설 | 500 m² | 1,500 m² |
| 창고 | 1,000 m² | 2,000 m² |

#### 10.3.2 수직 샤프트

**계단실**
- 완전 밀폐
- FRR-120 구조
- 각 층 방화문 (FRR-60)

**엘리베이터 샤프트**
- FRR-120 구조
- 기계실 방화 구획
- 각 층 승강장 방화문

**배관/덕트 샤프트**
- FRR-120 구조
- 관통부 방화 충전재
- 방화 댐퍼

---

## 11. 시스템 연동

### 11.1 HVAC 제어

#### 11.1.1 화재 시 동작 시나리오

**배기 팬 (Exhaust Fan)**
```typescript
interface ExhaustFanControl {
  fanId: string;
  zone: string;
  fireMode: {
    status: 'FIRE_DETECTED';
    action: 'START' | 'STOP';
    speed_percent: number;
    damperPosition: 'OPEN' | 'CLOSED';
  };
  normalMode: {
    status: 'NORMAL';
    action: 'AUTO';
    schedule: string;
  };
}
```

**화재 구역**
- 배기 팬: 100% 가동 (연기 배출)
- 급기 팬: 정지
- 순환 팬: 정지

**인접 구역**
- 배기 팬: 정지 (연기 확산 방지)
- 급기 팬: 정지
- 방화 댐퍼: 폐쇄

**대피 경로 (계단실, 복도)**
- 급기 팬: 가동 (양압 유지, 연기 유입 방지)
- 압력: +50 Pa (상대압)

#### 11.1.2 방화 댐퍼

**종류**
- 동적 방화 댐퍼: FRR-60, 120°C에서 폐쇄
- 정적 방화 댐퍼: FRR-120, 화재 신호로 폐쇄

**작동 방식**
```typescript
interface FireDamper {
  damperId: string;
  location: Location;
  type: 'DYNAMIC' | 'STATIC' | 'SMOKE';
  fireRating: 'FRR_60' | 'FRR_120';
  actuator: {
    type: 'FUSIBLE_LINK' | 'ELECTRIC' | 'PNEUMATIC';
    activationTemp_C?: number;
    closeTime_sec: number; // < 15초
  };
  position: 'OPEN' | 'CLOSED' | 'MODULATING';
  integrated: {
    hvacSystem: string;
    firePanelConnected: boolean;
  };
}
```

#### 11.1.3 연기 배출 시스템

**자연 배연**
- 배연창: 바닥 면적의 2-5%
- 개방: 자동 (화재 감지 시)
- 위치: 천장 또는 상부 벽

**기계 배연**
- 배연 팬 용량: 6-10 ACH (공기 교환 횟수/시간)
- 배연구: 화재 구역 천장
- 급기구: 하부 (바닥 1/2 이하)

### 11.2 엘리베이터 제어

#### 11.2.1 일반 엘리베이터 화재 모드

**Phase 1: 리콜 (Recall)**
1. 모든 호출 취소
2. 현재 층에서 문 즉시 개방 (10초)
3. 지정 층으로 이동 (보통 1층 또는 출구 층)
4. 도착 후 문 개방 유지
5. 서비스 정지 (소방 키 대기)

**Phase 2: 소방 운전**
- 소방 키 삽입 후 작동
- 수동 운전 모드
- 문 개방 시간 연장
- 홀 버튼 무효

```typescript
interface ElevatorFireControl {
  elevatorId: string;
  mode: 'NORMAL' | 'PHASE_1_RECALL' | 'PHASE_2_FIREFIGHTER';
  currentFloor: string;
  recallFloor: string;
  doorStatus: 'OPEN' | 'CLOSED' | 'OPENING' | 'CLOSING';
  fireSignals: {
    lobbyDetector: boolean;
    hoistway Detector: boolean;
    machineRoomDetector: boolean;
  };
  waterFlowSwitch: boolean;
  emergencyPower: boolean;
}
```

#### 11.2.2 소방 전용 엘리베이터

**요구사항**
- 독립 전원 (비상 발전기)
- 방수 기능
- 소방대원 전용 로비
- 통신 설비 (양방향 인터콤)

**성능**
- 용량: 최소 1,135 kg (15인)
- 속도: 최소 1 m/s
- 정원: 최소 15명

### 11.3 출입 통제 시스템

#### 11.3.1 전자 도어락

**화재 시 동작**
```typescript
interface AccessControlFireMode {
  doorId: string;
  normalMode: {
    lockStatus: 'LOCKED' | 'UNLOCKED';
    accessMethod: 'CARD' | 'PIN' | 'BIOMETRIC';
  };
  fireMode: {
    triggered: boolean;
    lockStatus: 'UNLOCKED'; // 모든 출구 해제
    direction: {
      egress: 'FREE'; // 내부 → 외부 자유
      ingress: 'CONTROLLED'; // 외부 → 내부 차단
    };
    overridePossible: boolean; // 소방 키로 재설정 가능
  };
}
```

#### 11.3.2 턴스타일/게이트

**화재 시 동작**
- 자동 개방 (양방향)
- 알람 해제
- 비상구 경로 확보

---

## 12. 데이터 형식 표준

### 12.1 JSON Schema

#### 12.1.1 화재 감지 이벤트

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "FireDetectionEvent",
  "type": "object",
  "required": ["eventId", "timestamp", "sensorId", "location", "readings"],
  "properties": {
    "eventId": {
      "type": "string",
      "pattern": "^FIRE-[0-9]{4}-[0-9]{6}$",
      "description": "고유 이벤트 ID (예: FIRE-2025-000001)"
    },
    "timestamp": {
      "type": "string",
      "format": "date-time",
      "description": "ISO 8601 timestamp"
    },
    "sensorId": {
      "type": "string",
      "description": "센서 ID"
    },
    "sensorType": {
      "type": "string",
      "enum": ["SMOKE", "HEAT", "FLAME", "CO", "MULTI"]
    },
    "location": {
      "type": "object",
      "required": ["building", "floor", "zone"],
      "properties": {
        "building": {"type": "string"},
        "floor": {"type": "string"},
        "zone": {"type": "string"},
        "coordinates": {
          "type": "object",
          "properties": {
            "lat": {"type": "number"},
            "lon": {"type": "number"}
          }
        }
      }
    },
    "readings": {
      "type": "object",
      "properties": {
        "temperature_C": {"type": "number"},
        "smokeLevel_ppm": {"type": "number"},
        "coLevel_ppm": {"type": "number"},
        "flameDetected": {"type": "boolean"}
      }
    },
    "alarmTriggered": {"type": "boolean"}
  }
}
```

### 12.2 XML Schema

#### 12.2.1 화재 사고 보고

```xml
<?xml version="1.0" encoding="UTF-8"?>
<xs:schema xmlns:xs="http://www.w3.org/2001/XMLSchema">
  <xs:element name="FireIncidentReport">
    <xs:complexType>
      <xs:sequence>
        <xs:element name="incidentId" type="xs:string"/>
        <xs:element name="detectedAt" type="xs:dateTime"/>
        <xs:element name="location" type="LocationType"/>
        <xs:element name="severity" type="SeverityType"/>
        <xs:element name="detectedBy" type="xs:string" maxOccurs="unbounded"/>
        <xs:element name="response" type="ResponseType"/>
        <xs:element name="damage" type="DamageAssessmentType"/>
      </xs:sequence>
    </xs:complexType>
  </xs:element>

  <xs:simpleType name="SeverityType">
    <xs:restriction base="xs:string">
      <xs:enumeration value="MINOR"/>
      <xs:enumeration value="MODERATE"/>
      <xs:enumeration value="MAJOR"/>
      <xs:enumeration value="CATASTROPHIC"/>
    </xs:restriction>
  </xs:simpleType>
</xs:schema>
```

---

## 13. API 인터페이스

### 13.1 RESTful API

#### 13.1.1 Base URL

```
Production: https://api.wia.org/city-013/v1
Sandbox: https://sandbox-api.wia.org/city-013/v1
```

#### 13.1.2 인증

**API Key**
```http
Authorization: Bearer YOUR_API_KEY
```

**OAuth 2.0**
```http
Authorization: Bearer ACCESS_TOKEN
```

#### 13.1.3 엔드포인트

**화재 감지**
```http
POST /fire/detect
Content-Type: application/json

{
  "sensorId": "SMOKE-FL2-A001",
  "sensorType": "SMOKE",
  "location": {...},
  "readings": {...}
}

Response: 201 Created
{
  "success": true,
  "data": {
    "eventId": "FIRE-2025-000123",
    "timestamp": "2025-12-25T10:30:00Z",
    "severity": "CRITICAL"
  }
}
```

**스프링클러 작동**
```http
POST /sprinklers/activate
Content-Type: application/json

{
  "systemId": "SPR-001",
  "zoneId": "ZONE-A"
}

Response: 200 OK
{
  "success": true,
  "data": {
    "systemId": "SPR-001",
    "activated": true,
    "activatedHeads": ["SPR-A-001", "SPR-A-002"],
    "timestamp": "2025-12-25T10:30:15Z"
  }
}
```

**대피 경로 조회**
```http
POST /evacuation/route
Content-Type: application/json

{
  "startLocation": {"floor": "3F", "zone": "B"},
  "accessible": true
}

Response: 200 OK
{
  "success": true,
  "data": {
    "routeId": "ROUTE-3FB-001",
    "distance_m": 45,
    "estimatedTime_sec": 90,
    "exits": [...]
  }
}
```

### 13.2 WebSocket API

#### 13.2.1 실시간 이벤트 스트리밍

**연결**
```
wss://api.wia.org/city-013/v1/events/subscribe?types=FIRE_DETECTED,ALARM_TRIGGERED
```

**메시지 포맷**
```json
{
  "type": "FIRE_DETECTED",
  "timestamp": "2025-12-25T10:30:00Z",
  "data": {
    "eventId": "FIRE-2025-000123",
    "sensorId": "SMOKE-FL2-A001",
    "location": {...},
    "severity": "CRITICAL"
  }
}
```

---

## 14. 보안 및 인증

### 14.1 API 보안

**HTTPS 필수**
- TLS 1.2 이상
- 인증서: SHA-256 이상

**인증 방법**
- API Key (간단한 통합)
- OAuth 2.0 (높은 보안)
- JWT (Stateless 인증)

**Rate Limiting**
- 일반: 1,000 requests/hour
- 긴급 (화재 감지): 무제한

### 14.2 데이터 암호화

**전송 중 (In Transit)**
- TLS 1.3
- 암호화 스위트: AES-256-GCM

**저장 시 (At Rest)**
- AES-256 암호화
- 키 관리: AWS KMS, Azure Key Vault

---

## 15. 성능 요구사항

### 15.1 응답 시간

| 작업 | 목표 | 우수 |
|------|------|------|
| 화재 감지 → 경보 | < 10초 | < 5초 |
| API 응답 시간 | < 500ms | < 200ms |
| 대피 경로 계산 | < 2초 | < 1초 |
| 스프링클러 작동 | < 30초 | < 15초 |

### 15.2 가용성

**시스템 가용성**
- 목표: 99.9% (연간 8.76시간 다운타임)
- 우수: 99.99% (연간 52.6분 다운타임)

**센서 가용성**
- 목표: > 99%
- 우수: > 99.9%

---

## 16. 부록

### 16.1 참조 표준

**국제 표준**
- NFPA 72: Fire Alarm Systems
- NFPA 13: Sprinkler Systems
- ISO 7240: Fire Detection and Alarm Systems
- EN 54: Fire Detection and Alarm Systems

**국내 표준**
- KS F 2845: 방화문의 내화 시험 방법
- 소방시설법
- 건축법 (방화 구획)

### 16.2 용어 약어

| 약어 | 전체 이름 |
|------|----------|
| HVAC | Heating, Ventilation, and Air Conditioning |
| FRR | Fire Resistance Rating |
| CO | Carbon Monoxide |
| UV | Ultraviolet |
| IR | Infrared |
| ACH | Air Changes per Hour |
| UPS | Uninterruptible Power Supply |
| BMS | Building Management System |

### 16.3 변경 이력

| 버전 | 날짜 | 변경 내용 |
|------|------|-----------|
| 1.0.0 | 2025-12-25 | 초기 버전 발행 |

---

**弘益人間 (홍익인간) - 널리 인간을 이롭게 하라**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
