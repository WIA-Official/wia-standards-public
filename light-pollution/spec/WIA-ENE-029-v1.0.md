# WIA-ENE-029: 빛 공해 관리 표준 v1.0 💡

> **弘益人間 (홍익인간) - 널리 인간을 이롭게 하라**

**표준 ID:** WIA-ENE-029
**버전:** 1.0.0
**발행일:** 2025-12-25
**상태:** 활성
**카테고리:** 에너지/환경 (ENE)

---

## 목차

1. [개요](#1-개요)
2. [적용 범위](#2-적용-범위)
3. [용어 정의](#3-용어-정의)
4. [빛 공해 분류](#4-빛-공해-분류)
5. [측정 기준 및 방법](#5-측정-기준-및-방법)
6. [어두운 하늘 구역 (Dark Sky Zones)](#6-어두운-하늘-구역-dark-sky-zones)
7. [조명 기준 및 규정](#7-조명-기준-및-규정)
8. [데이터 모델](#8-데이터-모델)
9. [모니터링 및 보고](#9-모니터링-및-보고)
10. [영향 평가](#10-영향-평가)
11. [완화 및 개선 조치](#11-완화-및-개선-조치)
12. [성과 지표 (KPI)](#12-성과-지표-kpi)
13. [통합 및 상호운용성](#13-통합-및-상호운용성)
14. [보안 및 개인정보보호](#14-보안-및-개인정보보호)
15. [인증 요구사항](#15-인증-요구사항)

---

## 1. 개요

### 1.1 목적

WIA-ENE-029 빛 공해 관리 표준은 인공 조명으로 인한 환경 오염을 체계적으로 관리하고, 어두운 밤하늘을 보호하며, 인간과 생태계의 건강을 증진하기 위한 국제 표준입니다.

빛 공해는 과도하거나 부적절한 인공 조명으로 인해 발생하는 환경 문제로, 천문 관측 방해, 생태계 교란, 인간 건강 악화, 에너지 낭비를 초래합니다.

### 1.2 핵심 원칙

- **필요한 곳에만 (Light Only Where Needed)**: 조명이 필요한 곳에만 빛을 제공
- **필요한 시간에만 (Light Only When Needed)**: 필요한 시간대에만 조명 운영
- **적정 밝기 (Appropriate Brightness)**: 과도하지 않은 적정 조도 유지
- **하향 조사 (Downward Direction)**: 빛을 아래로만 향하게 설계
- **따뜻한 색온도 (Warm Color Temperature)**: 낮은 색온도의 조명 사용
- **차폐 설계 (Full Cutoff)**: 상향광 차단 설계
- **어두운 하늘 보호 (Dark Sky Preservation)**: 밤하늘의 어둠 보존

### 1.3 적용 대상

- 지방자치단체 및 도시 계획 부서
- 조명 설계자 및 제조업체
- 건축 및 도시 개발 사업자
- 천문대 및 과학 연구 기관
- 자연 보호 구역 관리자
- 야생동물 서식지 관리자
- 공중 보건 기관

---

## 2. 적용 범위

### 2.1 빛 공해 유형

본 표준은 다음 빛 공해 유형에 적용됩니다:

- **하늘 빛공해 (Sky Glow)**: 대기 중 산란으로 인한 야간 하늘 밝아짐
- **빛 침입 (Light Trespass)**: 원하지 않는 곳으로의 빛 유입
- **눈부심 (Glare)**: 과도한 밝기나 대비로 인한 시각적 불편
- **과잉 조명 (Over-illumination)**: 필요 이상의 과도한 밝기
- **빛 혼란 (Light Clutter)**: 과도하고 혼란스러운 조명 그룹

### 2.2 조명 유형

- **가로등 (Street Lights)**
- **보안등 (Security Lights)**
- **건축 조명 (Architectural Lighting)**
- **광고판 조명 (Billboard Lighting)**
- **스포츠 시설 조명 (Sports Facility Lighting)**
- **장식 조명 (Decorative Lighting)**
- **상업 조명 (Commercial Lighting)**

### 2.3 관리 단계

- 조명 계획 및 설계
- 조명 기구 선정 및 설치
- 운영 및 유지 관리
- 모니터링 및 측정
- 규정 준수 검증
- 개선 및 업그레이드

---

## 3. 용어 정의

### 3.1 광학 측정 단위

- **조도 (Illuminance)**: 표면에 도달하는 빛의 양, 단위: lux (lx)
- **휘도 (Luminance)**: 발광 표면의 밝기, 단위: cd/m² (칸델라/제곱미터)
- **광속 (Luminous Flux)**: 광원이 방출하는 총 빛의 양, 단위: lm (루멘)
- **광도 (Luminous Intensity)**: 특정 방향의 빛의 세기, 단위: cd (칸델라)

### 3.2 천문학적 측정

- **하늘 밝기 (Sky Brightness)**: 야간 하늘의 밝기, 단위: mag/arcsec² (등급/제곱초각)
- **천정 휘도 (Zenith Luminance)**: 머리 위 하늘의 휘도
- **Bortle Scale**: 하늘의 어두운 정도를 나타내는 9단계 척도 (1=가장 어두움, 9=가장 밝음)
- **SQM (Sky Quality Meter)**: 하늘 밝기 측정 장비

### 3.3 조명 특성

- **색온도 (Color Temperature)**: 빛의 색상 특성, 단위: K (켈빈)
- **연색지수 (CRI, Color Rendering Index)**: 빛의 색재현성, 범위: 0-100
- **상향광비 (Upward Light Ratio)**: 수평선 위로 방출되는 빛의 비율 (%)
- **차단각 (Cutoff Angle)**: 빛이 차단되기 시작하는 각도

### 3.4 생물학적 용어

- **멜라토닌 (Melatonin)**: 수면을 조절하는 호르몬
- **일주기 리듬 (Circadian Rhythm)**: 24시간 주기의 생체 리듬
- **멜라놉신 (Melanopsin)**: 청색광에 민감한 망막 광수용체
- **멜라놉신 가중 조도 (Melanopic Lux)**: 일주기 리듬에 영향을 주는 유효 조도

---

## 4. 빛 공해 분류

### 4.1 하늘 빛공해 (Sky Glow)

인공 조명이 대기 입자에 산란되어 야간 하늘을 밝게 만드는 현상

#### 4.1.1 심각도 분류

| Bortle 척도 | 하늘 밝기 (mag/arcsec²) | 육안 별 수 | 상태 |
|------------|------------------------|-----------|------|
| 1 | > 21.7 | 5,000+ | 우수한 어두운 하늘 |
| 2 | 21.5-21.7 | 4,000 | 일반적인 어두운 하늘 |
| 3 | 21.3-21.5 | 2,500 | 시골 하늘 |
| 4 | 20.5-21.3 | 1,000 | 시골-교외 경계 |
| 5 | 19.5-20.5 | 500 | 교외 하늘 |
| 6 | 18.5-19.5 | 250 | 밝은 교외 하늘 |
| 7 | 18.0-18.5 | 100 | 교외-도심 경계 |
| 8 | 17.0-18.0 | 50 | 도시 하늘 |
| 9 | < 17.0 | 10 | 도심 하늘 |

#### 4.1.2 측정 방법

```typescript
interface SkyGlowMeasurement {
  skyBrightness: {
    value: number;           // mag/arcsec²
    bortle: BortleScale;     // 1-9
    sqm: number;             // SQM 측정값
  };
  zenithLuminance: number;   // cd/m²
  visibleStars: number;      // 육안 관측 별 개수
  affectedArea: number;      // 영향 범위 (km²)
}
```

### 4.2 빛 침입 (Light Trespass)

원하지 않는 곳으로 빛이 침투하는 현상

#### 4.2.1 허용 기준

| 구역 | 창문 수직 조도 (lux) | 침실 수평 조도 (lux) |
|------|-------------------|-------------------|
| E1 (천연 어두움) | 0.1 | 0.05 |
| E2 (어두움) | 0.5 | 0.25 |
| E3 (시골) | 1.0 | 0.5 |
| E4 (교외) | 2.0 | 1.0 |
| E5 (도시) | 5.0 | 2.0 |

#### 4.2.2 측정 및 평가

```typescript
interface LightTrespassAssessment {
  source: {
    location: Coordinates;
    fixtureType: string;
    power: number;           // W
  };
  target: {
    location: Coordinates;
    type: 'window' | 'bedroom' | 'property';
  };
  measurement: {
    illuminance: number;     // lux
    angle: number;           // degrees
    distance: number;        // meters
  };
  compliance: boolean;
  severity: PollutionSeverity;
}
```

### 4.3 눈부심 (Glare)

과도한 밝기나 명암 대비로 인한 시각적 불편함

#### 4.3.1 눈부심 유형

- **능력 저하 눈부심 (Disability Glare)**: 시야 감소, 안전 위험
- **불쾌 눈부심 (Discomfort Glare)**: 시각적 불편함
- **실명 눈부심 (Blinding Glare)**: 일시적 시력 상실

#### 4.3.2 평가 기준

| 대비율 | 휘도 차이 (cd/m²) | 눈부심 정도 |
|--------|----------------|----------|
| < 3:1 | < 500 | 허용 |
| 3:1 - 10:1 | 500-2,000 | 불쾌 |
| 10:1 - 40:1 | 2,000-10,000 | 능력 저하 |
| > 40:1 | > 10,000 | 실명 위험 |

---

## 5. 측정 기준 및 방법

### 5.1 조도 측정 (Illuminance)

#### 5.1.1 측정 장비

- 조도계 (Lux Meter)
- 정확도: ±5% (Class A) 또는 ±10% (Class B)
- 코사인 보정 (Cosine Correction) 필수
- 색 보정 (Color Correction) 권장

#### 5.1.2 측정 위치

- **수평 조도**: 지면 또는 작업 표면 기준
- **수직 조도**: 창문, 벽면 기준
- **측정 높이**: 지면에서 0.75m (작업면) 또는 1.5m (보행자)

#### 5.1.3 측정 조건

- 측정 시간: 완전한 어둠 (일몰 후 1시간 이상)
- 날씨: 맑은 날, 구름 없음
- 달: 신월 또는 삭(0-25% 밝기) 선호
- 주변 광원: 측정 대상 외 광원 차단

### 5.2 하늘 밝기 측정 (Sky Brightness)

#### 5.2.1 SQM (Sky Quality Meter) 사용

```
측정 절차:
1. 천정(zenith, 머리 위) 방향 측정
2. 방위별 측정 (N, E, S, W, 45° 고도)
3. 최소 10회 반복 측정 후 평균
4. 측정값 기록: mag/arcsec²
```

#### 5.2.2 데이터 기록

```typescript
interface SkyBrightnessMeasurement {
  timestamp: string;              // ISO 8601
  location: {
    latitude: number;
    longitude: number;
    altitude: number;             // m (해발)
  };
  measurements: {
    zenith: number;               // mag/arcsec²
    north: number;
    east: number;
    south: number;
    west: number;
  };
  conditions: {
    cloudCover: number;           // %
    moonPhase: number;            // 0-1
    visibility: number;           // km
    temperature: number;          // °C
    humidity: number;             // %
  };
  bortle: BortleScale;
}
```

### 5.3 색온도 및 스펙트럼 측정

#### 5.3.1 분광계 (Spectrometer) 사용

- 파장 범위: 380-780nm (가시광선)
- 청색광 방출량 측정: 400-500nm 파장 적분
- 멜라놉신 가중 조도 계산

#### 5.3.2 청색광 비율 계산

```
청색광 비율 (%) = (400-500nm 광속 / 총 광속) × 100

멜라놉신 가중 조도 (melanopic lux) = ∫ S(λ) × M(λ) × dλ
여기서:
- S(λ): 스펙트럼 분포
- M(λ): 멜라놉신 감도 함수
```

---

## 6. 어두운 하늘 구역 (Dark Sky Zones)

### 6.1 구역 분류

IDA (International Dark-Sky Association) 기준 준수

#### 6.1.1 E1 - 본질적으로 어두운 구역

**특성:**
- 국립공원, 천문대, 자연 보호 구역
- 인공 조명 최소화
- 야생동물 서식지 우선

**조명 제한:**
- 최대 수직 조도: 0.1 lux
- 상향광비: 0% (완전 차폐)
- 색온도: ≤ 2200K (호박색)
- 소등 시간: 22:00-06:00 (필수)

#### 6.1.2 E2 - 어두운 구역

**특성:**
- 농촌 지역, 저밀도 주거 지역
- 최소한의 공공 조명

**조명 제한:**
- 최대 수직 조도: 1 lux
- 상향광비: 0% (완전 차폐)
- 색온도: ≤ 2700K (따뜻한 백색)
- 소등 시간: 23:00-06:00 (권장)

#### 6.1.3 E3 - 시골 구역

**특성:**
- 일반 시골 지역
- 중밀도 주거 및 상업 지역

**조명 제한:**
- 최대 수직 조도: 2 lux
- 상향광비: ≤ 5% (완전 차단)
- 색온도: ≤ 3000K (따뜻한 백색)
- 소등 시간: 없음

#### 6.1.4 E4 - 교외 구역

**특성:**
- 교외 주거 및 상업 지역
- 일반적인 도시 외곽 지역

**조명 제한:**
- 최대 수직 조도: 5 lux
- 상향광비: ≤ 10% (차단)
- 색온도: ≤ 4000K (중성 백색)
- 소등 시간: 없음

#### 6.1.5 E5 - 도시 구역

**특성:**
- 도심 상업 및 업무 지역
- 고밀도 지역

**조명 제한:**
- 최대 수직 조도: 10 lux
- 상향광비: ≤ 15% (반차단)
- 색온도: ≤ 5000K (주광색)
- 소등 시간: 없음 (심야 조광 권장)

### 6.2 구역별 조명 설계 가이드

```typescript
interface ZoneDesignGuidelines {
  zone: DarkSkyZone;

  // 조도 기준
  lighting: {
    maxVerticalIlluminance: number;    // lux
    maxHorizontalIlluminance: number;  // lux
    recommendedAverage: number;        // lux
  };

  // 기술 요구사항
  technical: {
    maxUpwardLightRatio: number;       // %
    maxColorTemperature: number;       // K
    minShielding: ShieldingType;
    cutoffAngle: number;               // degrees
  };

  // 운영 요구사항
  operational: {
    curfewRequired: boolean;
    curfewHours: CurfewHours | null;
    dimmingRequired: boolean;
    motionSensorRecommended: boolean;
  };

  // 에너지 효율
  efficiency: {
    minEfficacy: number;               // lm/W
    maxPowerDensity: number;           // W/m²
    smartControlsRequired: boolean;
  };
}
```

---

## 7. 조명 기준 및 규정

### 7.1 조명 기구 요구사항

#### 7.1.1 차폐 (Shielding) 기준

| 차폐 유형 | 90° 상향광 | 80° 상향광 | 적용 구역 |
|---------|----------|----------|---------|
| 완전 차폐 (Fully Shielded) | 0% | 0% | E1, E2 |
| 완전 차단 (Fully Cutoff) | 0% | ≤ 2.5% | E2, E3 |
| 차단 (Cutoff) | 0% | ≤ 5% | E3, E4 |
| 반차단 (Semi-Cutoff) | ≤ 5% | ≤ 20% | E4, E5 |
| 비차단 (Non-Cutoff) | 제한 없음 | 제한 없음 | 사용 금지 |

#### 7.1.2 색온도 제한

```
E1 구역: ≤ 2200K (호박색, Amber)
E2 구역: ≤ 2700K (따뜻한 백색, Warm White)
E3 구역: ≤ 3000K (따뜻한 백색)
E4 구역: ≤ 4000K (중성 백색, Neutral White)
E5 구역: ≤ 5000K (주광색, Daylight)

청색광 비율: < 15% (모든 구역)
```

#### 7.1.3 효율성 기준

- LED 기구: ≥ 80 lm/W
- 조광 기능: 권장 (E1-E3 필수)
- 수명: ≥ 50,000 시간 (L70)
- 연색지수: CRI ≥ 70

### 7.2 조명 유형별 기준

#### 7.2.1 가로등

```typescript
interface StreetLightStandards {
  // 조도 기준
  illuminance: {
    average: number;                   // lux
    uniformity: number;                // Emin/Eavg
    maxVertical: number;               // lux (건물 벽면)
  };

  // 설치 기준
  installation: {
    maxHeight: number;                 // m
    spacing: number;                   // m
    mountingAngle: number;             // degrees (0-15°)
    shielding: ShieldingType;
  };

  // 제어
  controls: {
    dimming: {
      enabled: boolean;
      schedule: DimmingSchedule[];
      minLevel: number;                // % (50% 권장)
    };
    adaptiveLighting: boolean;
    centralControl: boolean;
  };
}

interface DimmingSchedule {
  timeStart: string;                   // HH:MM
  timeEnd: string;                     // HH:MM
  level: number;                       // % (0-100)
  condition?: 'traffic_low' | 'pedestrian_low' | 'always';
}
```

#### 7.2.2 보안등

- 동작 감지 센서 필수
- 최대 조명 시간: 5분
- 조사 범위: 재산 경계 내로 제한
- 최대 높이: 4m

#### 7.2.3 광고판 조명

- 상향 조사 금지 (E1-E3 구역)
- 소등 시간 준수
- 밝기 조절: 주변 조도에 따라 자동 조정
- 애니메이션/점멸: 금지 또는 제한

#### 7.2.4 스포츠 시설 조명

- 사용 시간 외 소등
- 완전 차단 기구 사용
- 스필 라이트(spill light) 최소화: 경기장 경계 외 < 10 lux
- 상향광: < 5%

---

## 8. 데이터 모델

### 8.1 측정 이벤트

```typescript
interface LightPollutionMeasurement {
  // 기본 정보
  measurementId: string;
  timestamp: string;                   // ISO 8601
  location: {
    address: string;
    coordinates: {
      latitude: number;
      longitude: number;
      altitude: number;                // m
    };
    zone: DarkSkyZone;
  };

  // 측정 데이터
  illuminance?: {
    horizontal: number;                // lux
    vertical: number;                  // lux
    unit: 'lux';
  };

  luminance?: {
    value: number;                     // cd/m²
    backgroundValue: number;
    contrastRatio: number;
  };

  skyBrightness?: {
    zenith: number;                    // mag/arcsec²
    bortle: BortleScale;
    sqm: number;
    visibleStars: number;
  };

  colorTemperature?: {
    value: number;                     // K
    blueLightPercentage: number;       // %
    melanopicLux: number;
  };

  // 오염 평가
  pollution: {
    skyGlow?: {
      severity: PollutionSeverity;
      affectedArea: number;            // km²
    };
    lightTrespass?: {
      severity: PollutionSeverity;
      targetType: string;
      distance: number;                // m
    };
    glare?: {
      type: GlareType;
      severity: PollutionSeverity;
    };
  };

  // 메타데이터
  metadata: {
    deviceId: string;
    deviceType: string;
    operator: string;
    weather: WeatherConditions;
    calibrationDate: string;
    dataQuality: number;               // 0-100
    verified: boolean;
  };
}
```

### 8.2 조명 기구 정보

```typescript
interface LightingFixture {
  // 기본 정보
  fixtureId: string;
  name: string;
  type: FixtureType;
  location: Location;

  // 기술 사양
  specifications: {
    manufacturer: string;
    model: string;
    wattage: number;                   // W
    lumens: number;                    // lm
    efficacy: number;                  // lm/W
    colorTemperature: number;          // K
    cri: number;                       // 0-100

    // 광학 특성
    beamAngle: number;                 // degrees
    cutoffAngle: number;               // degrees
    shielding: ShieldingType;
    upwardLightRatio: number;          // %

    // 수명
    ratedLife: number;                 // hours
    l70: number;                       // hours
  };

  // 설치 정보
  installation: {
    installDate: string;
    height: number;                    // m
    tilt: number;                      // degrees
    orientation: number;               // degrees (방위각)
    pole: {
      height: number;                  // m
      type: string;
    };
  };

  // 제어 시스템
  controls: {
    dimming: boolean;
    dimmingSchedule?: DimmingSchedule[];
    motionSensor: boolean;
    lightSensor: boolean;
    timer: boolean;
    centralControl: boolean;
    smartControl: boolean;
  };

  // 규정 준수
  compliance: {
    compliant: boolean;
    zone: DarkSkyZone;
    violations: ComplianceViolation[];
    certifications: string[];          // IDA, DarkSky 등
    lastInspection: string;
    nextInspection: string;
  };

  // 운영 데이터
  operation: {
    status: 'active' | 'inactive' | 'maintenance' | 'faulty';
    operatingHours: number;            // 누적 운영 시간
    energyConsumption: number;         // kWh
    maintenanceHistory: MaintenanceRecord[];
  };
}
```

### 8.3 규정 위반 사항

```typescript
interface ComplianceViolation {
  violationId: string;
  fixtureId: string;
  timestamp: string;

  violationType: ViolationType;
  severity: PollutionSeverity;

  details: {
    parameter: string;                 // 위반 항목
    measuredValue: number;
    limitValue: number;
    unit: string;
    exceedancePercentage: number;      // %
  };

  impact: {
    affectedArea: string;
    affectedPopulation?: number;
    wildlifeImpact?: boolean;
    astronomyImpact?: boolean;
  };

  resolution: {
    resolved: boolean;
    resolutionDate?: string;
    resolutionMethod?: string;
    verifiedBy?: string;
  };

  enforcement: {
    warning: boolean;
    fine?: number;                     // 원
    deadline?: string;
    legalAction?: boolean;
  };
}
```

---

## 9. 모니터링 및 보고

### 9.1 실시간 모니터링

#### 9.1.1 IoT 센서 네트워크

```typescript
interface LightMonitoringSensor {
  sensorId: string;
  location: Coordinates;
  zone: DarkSkyZone;

  sensors: {
    luxMeter: boolean;
    sqmMeter: boolean;
    spectrometer: boolean;
    camera: boolean;
  };

  measurements: {
    illuminance: number;               // lux
    skyBrightness: number;             // mag/arcsec²
    colorTemperature: number;          // K
    timestamp: string;
  };

  status: {
    online: boolean;
    batteryLevel: number;              // %
    lastCalibration: string;
    nextCalibration: string;
    dataQuality: number;               // 0-100
  };

  communication: {
    protocol: 'LoRaWAN' | 'NB-IoT' | 'WiFi' | '4G';
    updateInterval: number;            // seconds
    dataTransmission: 'realtime' | 'batch';
  };
}
```

#### 9.1.2 자동 경보 시스템

```typescript
interface AlertTrigger {
  parameter: 'illuminance' | 'sky_brightness' | 'color_temp' | 'upward_light';
  threshold: number;
  condition: '>' | '<' | '==' | '!=' | '>=' | '<=';
  duration: number;                    // seconds (지속 시간)
  severity: PollutionSeverity;

  actions: {
    notification: boolean;
    email: string[];
    sms: string[];
    autoReport: boolean;
    escalation: boolean;
  };
}
```

### 9.2 보고 체계

#### 9.2.1 정기 보고

- **일일 보고**: 센서 데이터 요약
- **주간 보고**: 평균 측정값, 위반 사항
- **월간 보고**: 통계 분석, KPI
- **분기 보고**: 추세 분석, 개선 현황
- **연간 보고**: 종합 평가, 정책 권고

#### 9.2.2 보고서 포맷

```typescript
interface LightPollutionReport {
  reportId: string;
  reportType: ReportType;
  period: {
    start: string;
    end: string;
  };
  region: string;

  // 요약
  summary: {
    totalMeasurements: number;
    averageSkyBrightness: number;      // mag/arcsec²
    averageBortle: number;
    complianceRate: number;            // %
    violationCount: number;
  };

  // 측정 통계
  statistics: {
    illuminance: {
      min: number;
      max: number;
      average: number;
      median: number;
      stdDev: number;
    };
    skyBrightness: {
      min: number;
      max: number;
      average: number;
    };
    colorTemperature: {
      average: number;
      warmLightPercentage: number;     // < 3000K
      coolLightPercentage: number;     // > 4000K
    };
  };

  // 구역별 분석
  byZone: {
    zone: DarkSkyZone;
    fixtureCount: number;
    averageBortle: number;
    complianceRate: number;
    violationCount: number;
    improvements: string[];
  }[];

  // 영향 평가
  impacts: {
    wildlifeAffected: number;
    astronomyImpact: 'none' | 'low' | 'moderate' | 'high' | 'severe';
    healthRisk: 'low' | 'moderate' | 'high';
    energyWaste: number;               // kWh
    co2Emissions: number;              // kg CO2
  };

  // 권고 사항
  recommendations: string[];

  // 메타데이터
  metadata: {
    generatedBy: string;
    generatedAt: string;
    version: string;
    dataQuality: number;               // 0-100
  };
}
```

---

## 10. 영향 평가

### 10.1 야생동물 영향

#### 10.1.1 영향 유형

```typescript
enum WildlifeImpactType {
  NAVIGATION_DISRUPTION = 'navigation_disruption',        // 철새, 바다거북 방향 감각
  BREEDING_DISRUPTION = 'breeding_disruption',            // 야행성 동물 번식 방해
  FEEDING_DISRUPTION = 'feeding_disruption',              // 섭식 행동 변화
  MIGRATION_DISRUPTION = 'migration_disruption',          // 이동 경로 변화
  PREDATION_INCREASE = 'predation_increase',              // 포식 위험 증가
  HABITAT_AVOIDANCE = 'habitat_avoidance',                // 서식지 회피
  CIRCADIAN_DISRUPTION = 'circadian_disruption',          // 생체 리듬 교란
  ATTRACTION_TO_LIGHT = 'attraction_to_light',            // 빛에 이끌림 (곤충)
  LIGHT_AVOIDANCE = 'light_avoidance',                    // 빛 회피 (박쥐)
}
```

#### 10.1.2 취약 종 보호

| 종 분류 | 취약성 | 보호 조치 |
|--------|-------|----------|
| 철새 (Migratory Birds) | 높음 | 이동 시기 소등, 빨간색 조명 |
| 바다거북 (Sea Turtles) | 매우 높음 | 해안 조명 차단, 호박색 조명 |
| 야행성 포유류 (Nocturnal Mammals) | 중간 | 동작 감지 조명, 낮은 조도 |
| 박쥐 (Bats) | 높음 | 따뜻한 색온도, 소등 시간 |
| 곤충 (Insects) | 매우 높음 | 호박색 조명, UV 차단 |
| 양서류 (Amphibians) | 높음 | 서식지 조명 최소화 |

### 10.2 인간 건강 영향

#### 10.2.1 일주기 리듬 교란

```typescript
interface CircadianRhythmAssessment {
  exposure: {
    melanopicLux: number;
    duration: number;                  // minutes
    timeOfDay: string;                 // HH:MM
    frequency: 'daily' | 'frequent' | 'occasional';
  };

  effects: {
    melatoninSuppression: number;      // % (0-100)
    sleepDelayEstimate: number;        // minutes
    sleepQualityImpact: 'none' | 'mild' | 'moderate' | 'severe';
  };

  riskFactors: HealthRiskFactor[];

  recommendations: {
    maxExposureDuration: number;       // minutes
    recommendedColorTemp: number;      // K
    dimmingRecommended: boolean;
    blueBlockerGlasses: boolean;
  };
}
```

#### 10.2.2 건강 위험 요소

- **수면 장애**: 멜라토닌 억제로 인한 수면 시작 지연, 수면 질 저하
- **일주기 리듬 불일치**: 생체 시계와 환경 사이클 불일치
- **암 위험 증가**: 멜라토닌 감소로 인한 유방암, 전립선암 위험
- **대사 장애**: 당뇨병, 비만 위험 증가
- **기분 장애**: 우울증, 불안증
- **인지 기능 저하**: 집중력, 기억력 감소

### 10.3 천문 관측 영향

#### 10.3.1 관측 조건 평가

```typescript
interface AstronomyImpactAssessment {
  location: {
    coordinates: Coordinates;
    nearestObservatory: string;
    distance: number;                  // km
  };

  skyQuality: {
    bortle: BortleScale;
    limitingMagnitude: number;         // 육안 관측 한계 등급
    skyBrightness: number;             // mag/arcsec²
    transparency: number;              // % (0-100)
  };

  impact: {
    severity: 'none' | 'minimal' | 'moderate' | 'severe' | 'extreme';
    affectedInstruments: string[];
    observableObjects: {
      messierObjects: number;          // 관측 가능한 메시에 천체 수
      galaxies: boolean;
      nebulae: boolean;
      milkyWay: boolean;
    };
  };

  recommendations: string[];
}
```

### 10.4 에너지 및 경제 영향

#### 10.4.1 에너지 낭비 평가

```typescript
interface EnergyWasteAssessment {
  fixtures: {
    total: number;
    totalWattage: number;              // W
    upwardLightWattage: number;        // W
    unnecessaryWattage: number;        // W (과조명)
  };

  annualConsumption: {
    total: number;                     // kWh/year
    wasted: number;                    // kWh/year
    wastePercentage: number;           // %
    potentialSavings: number;          // kWh/year
  };

  cost: {
    annualEnergyCost: number;          // 원/year
    wastedCost: number;                // 원/year
    potentialSavings: number;          // 원/year
    investmentForUpgrade: number;      // 원
    roi: number;                       // years (투자 회수 기간)
  };

  emissions: {
    annualCO2: number;                 // kg CO2/year
    wastedCO2: number;                 // kg CO2/year
    potentialReduction: number;        // kg CO2/year
  };
}
```

---

## 11. 완화 및 개선 조치

### 11.1 조명 개선 전략

#### 11.1.1 즉시 조치 (0-6개월)

1. **과조명 감소**
   - 불필요한 조명 제거
   - 조도 감소 (50-70% 수준으로)
   - 타이머 설치

2. **차폐 추가**
   - 기존 기구에 차폐물 추가
   - 각도 조정 (하향 조사)
   - 반사판 설치

3. **운영 시간 조정**
   - 소등 시간 설정
   - 심야 조광 (50% 이하)
   - 동작 감지 센서 추가

#### 11.1.2 단기 조치 (6-12개월)

1. **색온도 개선**
   - 고온(>4000K) LED → 저온(≤3000K) LED 교체
   - 호박색 필터 추가
   - 따뜻한 색온도 조명 우선 설치

2. **스마트 제어 시스템**
   - 중앙 제어 시스템 구축
   - 조광 스케줄 자동화
   - 적응형 조명 (교통량/보행자 기반)

3. **모니터링 시스템**
   - SQM 센서 설치
   - 조도계 네트워크 구축
   - 실시간 데이터 대시보드

#### 11.1.3 장기 조치 (1-3년)

1. **전면 교체**
   - 완전 차단 기구로 교체
   - 고효율 LED (>100 lm/W)
   - IDA/DarkSky 인증 제품 사용

2. **도시 계획 통합**
   - 조명 마스터 플랜 수립
   - 어두운 하늘 구역 지정
   - 건축 조명 가이드라인

3. **정책 및 규제**
   - 빛 공해 방지 조례 제정
   - 신규 개발 프로젝트 기준 강화
   - 인증 및 검증 시스템

### 11.2 설계 베스트 프랙티스

#### 11.2.1 5가지 원칙

```
1. 유용성 (Useful)
   - 조명이 필요한 곳에만 빛을 제공
   - 목표 지역 외 조명 최소화

2. 목표 조명 (Targeted)
   - 정확한 광학 제어
   - 빔 각도 최적화
   - 스필 라이트 방지

3. 낮은 조도 (Low Light Levels)
   - 과조명 방지
   - 적정 조도 유지
   - 눈의 적응 고려

4. 통제 (Controlled)
   - 필요한 시간에만 점등
   - 조광 및 타이머 활용
   - 동작 감지 센서

5. 따뜻한 색상 (Warm Colors)
   - 색온도 ≤ 3000K
   - 청색광 최소화
   - 호박색/따뜻한 백색 선호
```

#### 11.2.2 조명 설계 체크리스트

```markdown
☐ 조명이 정말 필요한가?
☐ 자연광 또는 반사광으로 대체 가능한가?
☐ 조도 레벨이 적정한가?
☐ 완전 차단(Full Cutoff) 기구인가?
☐ 색온도가 3000K 이하인가?
☐ 상향광이 없는가?
☐ 동작 감지 센서가 있는가?
☐ 타이머 또는 조광 기능이 있는가?
☐ 빛이 목표 지역 외로 누출되지 않는가?
☐ 에너지 효율이 80 lm/W 이상인가?
☐ 구역 규정을 준수하는가?
☐ 야생동물 영향이 평가되었는가?
```

### 11.3 커뮤니티 참여

#### 11.3.1 교육 및 인식 제고

- 공개 워크숍 및 세미나
- 학교 교육 프로그램
- 별 관측 행사
- 빛 공해 지도 제작 및 공개
- 소셜 미디어 캠페인

#### 11.3.2 시민 과학 프로그램

```typescript
interface CitizenScienceProgram {
  activities: {
    skyBrightnessMeasurement: boolean;      // Globe at Night, Loss of the Night
    fixtureMapping: boolean;                // 조명 기구 위치 기록
    wildlifeObservation: boolean;           // 야행성 동물 관찰
    photoDocumentation: boolean;            // 빛 공해 사진 수집
  };

  tools: {
    mobileApps: string[];                   // 측정 앱 목록
    sqmLoanProgram: boolean;                // SQM 대여 프로그램
    onlineTraining: boolean;
  };

  dataSubmission: {
    platform: string;
    dataFormat: 'csv' | 'json' | 'api';
    quality_control: boolean;
    publicDatabase: boolean;
  };
}
```

---

## 12. 성과 지표 (KPI)

### 12.1 환경 성과 지표

| KPI | 목표값 | 측정 단위 | 산정 방식 |
|-----|--------|----------|----------|
| 평균 하늘 밝기 | Bortle ≤ 4 | mag/arcsec² | 센서 네트워크 평균 |
| 어두운 하늘 면적 | 30% 증가 | km² | Bortle ≤ 3 지역 |
| 규정 준수율 | 95% 이상 | % | (준수 기구 / 총 기구) × 100 |
| 에너지 절감 | 40% 감소 | kWh/year | 개선 전후 비교 |
| CO2 감축 | 10,000톤/년 | 톤 CO2 | 에너지 절감 × 배출 계수 |

### 12.2 생태 성과 지표

- **야생동물 서식지 회복**: 빛 공해 감소 지역 수
- **철새 충돌 감소**: 연간 충돌 사고 건수
- **곤충 개체수 회복**: 야행성 곤충 다양성 지수
- **생물 다양성 증가**: 야행성 종 관측 빈도

### 12.3 사회 성과 지표

- **별 관측 가능 지역**: 육안으로 은하수 관측 가능 지역 비율
- **수면 질 개선**: 주민 설문 조사 (주관적 수면 질)
- **민원 감소**: 빛 공해 관련 민원 건수
- **인식 제고**: 빛 공해 인지도 (설문)

### 12.4 경제 성과 지표

- **에너지 비용 절감**: 연간 전기 요금 절감액
- **유지 보수 비용 감소**: LED 교체로 인한 유지비 절감
- **관광 수익 증가**: 어두운 하늘 관광 수익
- **ROI**: 투자 대비 수익률

---

## 13. 통합 및 상호운용성

### 13.1 WIA 생태계 통합

#### 13.1.1 연계 표준

- **WIA-ENE-001 (Climate)**: 조명 에너지 사용으로 인한 탄소 배출 산정
- **WIA-ENE-007 (Smart Grid)**: 스마트 그리드 기반 조명 제어
- **WIA-SOCIAL**: 시민 참여 및 교육 프로그램
- **WIA-BLOCKCHAIN**: 조명 기구 이력 추적
- **WIA-ENE-BIO (Biodiversity)**: 생물 다양성 영향 평가 연계

#### 13.1.2 API 엔드포인트

```
# 측정
POST   /api/v1/measurements                # 측정 데이터 제출
GET    /api/v1/measurements/{id}            # 측정 조회
GET    /api/v1/measurements/sky-brightness  # 하늘 밝기 조회

# 조명 기구
POST   /api/v1/fixtures                     # 기구 등록
GET    /api/v1/fixtures/{id}                # 기구 조회
PUT    /api/v1/fixtures/{id}                # 기구 업데이트
GET    /api/v1/fixtures/{id}/compliance     # 규정 준수 확인

# 구역 관리
GET    /api/v1/zones/{zone}/limits          # 구역 제한 조회
GET    /api/v1/zones/classify               # 위치 기반 구역 분류

# 경보 및 모니터링
POST   /api/v1/alerts                       # 경보 생성
GET    /api/v1/alerts                       # 경보 목록
POST   /api/v1/alerts/{id}/acknowledge      # 경보 확인
POST   /api/v1/alerts/{id}/resolve          # 경보 해결

# 분석 및 보고
GET    /api/v1/analytics/statistics         # 통계 조회
POST   /api/v1/analytics/report             # 보고서 생성
GET    /api/v1/analytics/compliance         # 규정 준수 대시보드

# 위반 관리
POST   /api/v1/violations                   # 위반 보고
GET    /api/v1/violations                   # 위반 목록
```

### 13.2 데이터 교환 포맷

- **기본 포맷**: JSON (UTF-8)
- **대용량 데이터**: CSV, Parquet, HDF5
- **실시간 스트림**: MQTT, WebSocket
- **지리 데이터**: GeoJSON
- **이미지**: FITS (천문 이미지), JPEG/PNG

### 13.3 상호운용성 요구사항

- ISO 8601 (날짜 및 시간) 준수
- WGS84 (좌표계) 사용
- OAuth 2.0 (인증) 지원
- RESTful API 설계 원칙 준수
- OpenAPI 3.0 스펙 문서화
- MQTT v5.0 (IoT 센서 통신)

---

## 14. 보안 및 개인정보보호

### 14.1 데이터 보안

#### 14.1.1 보안 등급

| 데이터 유형 | 보안 등급 | 암호화 | 접근 통제 |
|------------|----------|--------|----------|
| 측정 데이터 | Public | 불필요 | 공개 |
| 조명 기구 위치 | Public | 불필요 | 공개 |
| 개인 재산 정보 | Confidential | AES-256 | 권한자만 |
| 위반 사항 (개인) | Confidential | AES-256 | 엄격 제한 |

#### 14.1.2 보안 조치

- 전송 중 암호화: TLS 1.3 이상
- API 인증: Bearer Token, API Key
- 접근 로그: 모든 API 호출 기록
- 데이터 무결성: HMAC-SHA256 서명
- 백업: 일일 백업 및 30일 보관

### 14.2 개인정보보호

- **최소 수집 원칙**: 필수 정보만 수집
- **익명화 처리**: 개인 식별 정보 제거
- **위치 정보 보호**: 정확한 주소 대신 대략적 지역 공개
- **동의 기반 수집**: 명시적 동의 후 수집
- **삭제 권리**: 정보주체의 삭제 요청 처리

---

## 15. 인증 요구사항

### 15.1 시설 인증

#### 15.1.1 인증 등급

**Bronze (기본)**
- 규정 준수율 80% 이상
- 소등 시간 운영 (해당 구역)
- 연간 에너지 10% 절감

**Silver (우수)**
- 규정 준수율 90% 이상
- 완전 차단 기구 60% 이상
- 색온도 ≤ 3000K (80% 이상)
- 연간 에너지 25% 절감

**Gold (최우수)**
- 규정 준수율 95% 이상
- 완전 차단 기구 80% 이상
- 색온도 ≤ 3000K (100%)
- 스마트 제어 시스템
- 연간 에너지 40% 절감

**Platinum (특급)**
- 규정 준수율 100%
- 완전 차단 기구 100%
- 호박색 조명 (≤ 2700K)
- AI 기반 적응형 조명
- 연간 에너지 50% 절감
- 어두운 하늘 보호 구역 지정

#### 15.1.2 국제 인증 통합

- **IDA (International Dark-Sky Association)**: Dark Sky Place 인증
- **DarkSky International**: 어두운 하늘 공동체 인증
- **LEED**: 빛 공해 크레딧
- **BREEAM**: 외부 조명 크레딧

### 15.2 조명 기구 인증

#### 15.2.1 인증 기준

```typescript
interface FixtureCertificationCriteria {
  photometric: {
    fullyCutoff: boolean;              // 완전 차단
    maxUpwardLight: number;            // % (< 2.5%)
    uvRating: number;                  // UV 방출 등급 (낮을수록 좋음)
  };

  electrical: {
    minEfficacy: number;               // lm/W (≥ 80)
    minPowerFactor: number;            // ≥ 0.9
    minL70: number;                    // hours (≥ 50,000)
  };

  environmental: {
    maxColorTemperature: number;       // K (≤ 3000)
    blueLightHazard: 'RG0' | 'RG1';   // Risk Group 0 or 1
    flickerFree: boolean;              // 플리커 없음
  };

  controls: {
    dimmingCapable: boolean;
    motionSensor: boolean;
    smartControlReady: boolean;
  };

  sustainability: {
    recyclable: boolean;
    hazardousMaterials: boolean;       // false (없음)
    energyStarRated: boolean;
  };
}
```

### 15.3 인증 프로세스

1. **신청 및 서류 제출** (1주)
   - 조명 재고 목록
   - 측정 데이터 (최근 6개월)
   - 운영 계획서

2. **현장 실사** (3일)
   - 조명 기구 검사
   - 조도 및 하늘 밝기 측정
   - 운영 시스템 검증

3. **데이터 검증** (2주)
   - 측정 데이터 분석
   - 규정 준수 확인
   - 영향 평가

4. **전문가 심사** (2주)
   - 설계 검토
   - 개선 권고
   - 등급 결정

5. **인증서 발급**
   - 인증서 및 로고 제공
   - 웹사이트 등록
   - 홍보 지원

6. **사후 관리** (연간)
   - 연간 감사
   - 데이터 제출
   - 개선 이행 확인

---

## 부록

### A. 측정 장비 목록

#### A.1 조도계 (Lux Meter)
- **정확도**: Class A (±5%) 또는 Class B (±10%)
- **측정 범위**: 0.01 - 200,000 lux
- **주요 제조사**: Konica Minolta, GOSSEN, Sekonic

#### A.2 SQM (Sky Quality Meter)
- **측정 범위**: 15-22 mag/arcsec²
- **정확도**: ±0.1 mag/arcsec²
- **제조사**: Unihedron, Diffraction Limited

#### A.3 분광계 (Spectrometer)
- **파장 범위**: 380-780 nm
- **해상도**: 1-10 nm
- **제조사**: Ocean Optics, Stellarnet

### B. 조명 설계 예시

[구역별 조명 설계 사례 10개 이상]

### C. 샘플 데이터셋

[실제 측정 데이터 JSON 예시]

### D. API 레퍼런스

[전체 API 엔드포인트 상세 문서]

### E. 용어 사전

[150개 이상의 전문 용어 정의]

### F. 법규 및 정책 사례

[국가별 빛 공해 방지 법규 요약]

---

## 개정 이력

| 버전 | 날짜 | 주요 변경사항 |
|------|------|--------------|
| 1.0.0 | 2025-12-25 | 초판 발행 |

---

## 라이선스

© 2025 WIA (World Certification Industry Association)
이 표준은 Creative Commons Attribution 4.0 International License 하에 배포됩니다.

---

## 연락처

**WIA 표준 사무국**
웹사이트: https://wiastandards.com
이메일: standards@wia-official.org
GitHub: https://github.com/WIA-Official/wia-standards

---

## 弘益人間 (홍익인간) · 널리 인간을 이롭게 하라

빛은 인류 문명의 상징이지만, 과도한 인공 조명은 밤하늘의 어둠을 잃게 하고 생태계를 교란하며 인간 건강을 해칩니다. WIA-ENE-029 표준은 지속가능한 조명 관리를 통해 어두운 밤하늘을 보호하고, 에너지를 절약하며, 인류와 자연이 조화롭게 공존하는 미래를 만들어갑니다.

개방형 표준, 투명한 프로토콜, 협력적 개발을 통해 빛 공해 관리가 인류 전체와 지구 생태계의 공동선에 기여하도록 보장합니다.

**함께, 우리는 별이 빛나는 밤하늘을 되찾습니다.**

---

© 2025 SmileStory Inc. / WIA
**弘益人間 (홍익인간) · Benefit All Humanity**
