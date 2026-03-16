# WIA-ENE-028: 소음 공해 표준 v1.0 🔊

> **弘益人間 (홍익인간) - 널리 인간을 이롭게 하라**

**표준 ID:** WIA-ENE-028
**버전:** 1.0.0
**발행일:** 2025-12-25
**상태:** 활성
**카테고리:** 에너지/환경 (ENE)

---

## 목차

1. [개요](#1-개요)
2. [적용 범위](#2-적용-범위)
3. [용어 정의](#3-용어-정의)
4. [소음 측정 기준](#4-소음-측정-기준)
5. [데이터 모델](#5-데이터-모델)
6. [소음원 분류](#6-소음원-분류)
7. [규제 기준](#7-규제-기준)
8. [건강 영향 평가](#8-건강-영향-평가)
9. [모니터링 네트워크](#9-모니터링-네트워크)
10. [저감 대책](#10-저감-대책)
11. [성과 지표 (KPI)](#11-성과-지표-kpi)
12. [통합 및 상호운용성](#12-통합-및-상호운용성)
13. [보안 및 개인정보보호](#13-보안-및-개인정보보호)
14. [인증 요구사항](#14-인증-요구사항)

---

## 1. 개요

### 1.1 목적

WIA-ENE-028 소음 공해 표준은 환경 소음을 체계적으로 측정, 모니터링, 관리하기 위한 국제 표준입니다. 본 표준은 소음으로부터 시민의 건강과 삶의 질을 보호하고, 정온한 도시 환경을 조성하는 것을 목표로 합니다.

### 1.2 핵심 원칙

- **건강 우선 (Health First)**: 인체 영향 기반 규제 기준
- **과학적 측정 (Scientific Measurement)**: 정확하고 표준화된 측정 방법
- **실시간 모니터링 (Real-time Monitoring)**: 지속적인 소음 감시
- **투명성 (Transparency)**: 소음 정보 공개 및 시민 참여
- **예방 원칙 (Prevention)**: 사전 예방적 소음 관리
- **공평성 (Equity)**: 모든 지역에 동일한 정온 환경 보장
- **지속가능성 (Sustainability)**: 장기적 소음 저감 정책

### 1.3 적용 대상

- 지방자치단체 환경부서
- 소음 모니터링 기관
- 건설 및 공사 현장
- 공항 및 항공 관련 시설
- 도로 및 철도 운영 기관
- 산업 시설 및 공장
- 스마트시티 환경 관리 시스템
- 연구 기관 및 학계

---

## 2. 적용 범위

### 2.1 소음 유형

본 표준은 다음 소음 유형에 적용됩니다:

- **교통소음**: 도로, 철도, 항공기 소음
- **산업소음**: 공장, 제조 시설 소음
- **건설소음**: 공사 현장, 중장비 소음
- **생활소음**: 주거지역, 상업시설 소음
- **환경소음**: 통합 환경 배경 소음
- **임펄스소음**: 폭발, 충격 등 순간 소음
- **저주파소음**: 20Hz 이하 저주파 진동

### 2.2 측정 영역

- 주거지역
- 상업지역
- 공업지역
- 교육 및 의료시설 인근
- 공원 및 녹지
- 정온시설 보호구역
- 도로변 및 철도변

---

## 3. 용어 정의

### 3.1 기본 용어

- **소음 (Noise)**: 원하지 않는 불쾌한 소리
- **음압 레벨 (Sound Pressure Level, SPL)**: 소리의 크기를 나타내는 물리량 (dB)
- **등가소음도 (Leq)**: 일정 시간 동안의 평균 소음 레벨
- **최대소음도 (Lmax)**: 측정 기간 중 최대 소음 레벨
- **배경소음 (Background Noise)**: 특정 음원이 없을 때의 주변 소음
- **정온시설 (Quiet Zone)**: 소음으로부터 보호가 필요한 시설

### 3.2 측정 단위

- **dB (Decibel)**: 소리의 크기 단위
- **dBA**: A-가중 데시벨 (인간 청각 특성 반영)
- **dBC**: C-가중 데시벨 (저주파 포함)
- **dBZ**: Z-가중 데시벨 (무가중, 전 주파수)
- **Hz (Hertz)**: 주파수 단위
- **Pa (Pascal)**: 음압 단위

### 3.3 시간 지표

- **Ld (Day)**: 주간 소음도 (07:00-18:00)
- **Le (Evening)**: 저녁 소음도 (18:00-22:00)
- **Ln (Night)**: 야간 소음도 (22:00-07:00)
- **Lden**: 주야간 등가소음도 (Day-Evening-Night)
- **L10, L50, L90**: 누적백분율소음도

---

## 4. 소음 측정 기준

### 4.1 측정 장비 기준

#### 4.1.1 소음계 등급

| 등급 | 정확도 | 용도 | 국제 표준 |
|------|--------|------|----------|
| Class 0 | ±0.7 dB | 실험실 표준 | IEC 61672-1 |
| Class 1 | ±1.0 dB | 정밀 측정 (규제용) | IEC 61672-1 |
| Class 2 | ±1.5 dB | 일반 측정 (조사용) | IEC 61672-1 |
| Class 3 | ±2.5 dB | 간이 측정 (참고용) | - |

#### 4.1.2 필수 기능

- 주파수 가중: A, C, Z 가중
- 시간 가중: Fast (125ms), Slow (1s), Impulse
- 주파수 분석: 1/1 옥타브 밴드, 1/3 옥타브 밴드
- 통계 분석: Leq, Lmax, Lmin, L10, L50, L90
- 데이터 로깅: 최소 1초 간격 저장
- 검교정 기능: 자동 교정 기록

### 4.2 측정 방법

#### 4.2.1 측정 위치

- **높이**: 지면으로부터 1.2m ~ 1.5m
- **거리**: 반사면으로부터 최소 3.5m 이격
- **방향**: 주 소음원 방향
- **환경**: 강우, 강풍 시 측정 금지 (풍속 5m/s 이상)

#### 4.2.2 측정 시간

| 측정 유형 | 측정 시간 | 비고 |
|----------|----------|------|
| 순간 측정 | 5분 이상 | 변동이 적은 정상 소음 |
| 단기 측정 | 1시간 | 교통소음 등 |
| 장기 측정 | 24시간 이상 | 환경 소음 평가 |
| 연속 모니터링 | 7일 ~ 연중 | 상시 측정소 |

### 4.3 주파수 대역

#### 4.3.1 옥타브 밴드 중심 주파수 (Hz)

```
31.5, 63, 125, 250, 500, 1000, 2000, 4000, 8000, 16000
```

#### 4.3.2 1/3 옥타브 밴드 (상세 분석용)

```
25, 31.5, 40, 50, 63, 80, 100, 125, 160, 200, 250, 315, 400, 500,
630, 800, 1000, 1250, 1600, 2000, 2500, 3150, 4000, 5000, 6300,
8000, 10000, 12500, 16000, 20000
```

---

## 5. 데이터 모델

### 5.1 소음 측정 데이터

```typescript
interface NoiseMeasurement {
  // 기본 정보
  measurementId: string;              // 측정 ID
  timestamp: string;                  // ISO 8601 형식
  stationId: string;                  // 측정소 ID

  // 위치 정보
  location: {
    address: string;                  // 주소
    coordinates: {
      latitude: number;               // 위도
      longitude: number;              // 경도
    };
    elevation: number;                // 고도 (m)
    zoneType: string;                 // 지역 유형
  };

  // 소음 레벨
  levels: {
    dBA: {
      leq: number;                    // 등가소음도 (dBA)
      lmax: number;                   // 최대소음도 (dBA)
      lmin: number;                   // 최소소음도 (dBA)
      l10: number;                    // 10% 초과 레벨
      l50: number;                    // 50% 초과 레벨 (중앙값)
      l90: number;                    // 90% 초과 레벨 (배경소음)
    };
    dBC: {
      leq: number;                    // C-가중 등가소음도
      lmax: number;                   // C-가중 최대소음도
    };
    dBZ?: {
      leq: number;                    // 무가중 등가소음도
      lmax: number;                   // 무가중 최대소음도
    };
  };

  // 주파수 분석
  frequencyAnalysis?: {
    octaveBand: {                     // 옥타브 밴드
      freq: number;                   // 중심 주파수 (Hz)
      level: number;                  // 음압 레벨 (dB)
    }[];
    oneThirdOctave?: {                // 1/3 옥타브 밴드
      freq: number;
      level: number;
    }[];
  };

  // 환경 조건
  environmental: {
    temperature: number;              // 온도 (°C)
    humidity: number;                 // 습도 (%)
    windSpeed: number;                // 풍속 (m/s)
    windDirection: number;            // 풍향 (도)
    precipitation: boolean;           // 강수 여부
  };

  // 메타데이터
  metadata: {
    instrumentId: string;             // 측정 장비 ID
    calibrationDate: string;          // 교정 일자
    dataQuality: number;              // 데이터 품질 (0-100)
    flags: string[];                  // 플래그 (예: 'rain', 'wind')
  };
}
```

### 5.2 소음원 정보

```typescript
interface NoiseSource {
  sourceId: string;                   // 소음원 ID
  name: string;                       // 소음원 명칭
  category: NoiseSourceCategory;      // 카테고리

  // 위치
  location: {
    address: string;
    coordinates: { latitude: number; longitude: number; };
    area: number;                     // 영향 범위 (m²)
  };

  // 소음 특성
  characteristics: {
    typical_dBA: number;              // 전형적 소음도 (dBA)
    peak_dBA: number;                 // 최고 소음도 (dBA)
    frequency_range: {
      low: number;                    // 저주파 (Hz)
      high: number;                   // 고주파 (Hz)
    };
    pattern: 'continuous' | 'intermittent' | 'impulse';  // 소음 패턴
  };

  // 운영 시간
  operatingHours: {
    weekday: { start: string; end: string; };
    weekend: { start: string; end: string; };
    exceptions: string[];             // 예외 사항
  };

  // 저감 조치
  mitigationMeasures: {
    type: string;                     // 저감 조치 유형
    effectiveness: number;            // 효과 (dB 감소)
    installedDate: string;            // 설치 일자
  }[];

  // 규제 정보
  compliance: {
    limitApplicable: number;          // 적용 기준 (dBA)
    currentLevel: number;             // 현재 레벨 (dBA)
    isCompliant: boolean;             // 준수 여부
    lastInspection: string;           // 최근 검사일
  };
}
```

### 5.3 모니터링 스테이션

```typescript
interface MonitoringStation {
  stationId: string;                  // 측정소 ID
  name: string;                       // 측정소 명칭
  type: 'permanent' | 'temporary' | 'mobile';

  // 위치 정보
  location: {
    address: string;
    coordinates: { latitude: number; longitude: number; };
    elevation: number;                // 해발 고도 (m)
    zoneType: string;                 // 지역 유형
  };

  // 측정 장비
  equipment: {
    instrumentId: string;             // 장비 ID
    manufacturer: string;             // 제조사
    model: string;                    // 모델명
    class: 0 | 1 | 2 | 3;            // 정확도 등급
    calibrationDue: string;           // 다음 교정일
    installDate: string;              // 설치 일자
  };

  // 운영 정보
  operation: {
    status: 'active' | 'inactive' | 'maintenance';
    startDate: string;                // 운영 시작일
    measurementInterval: number;      // 측정 주기 (초)
    dataTransmission: 'realtime' | 'batch';
    powerSource: 'grid' | 'solar' | 'battery';
  };

  // 통신
  communication: {
    protocol: 'mqtt' | 'http' | '4g' | 'wifi';
    endpoint: string;                 // 데이터 전송 엔드포인트
    lastTransmission: string;         // 최근 전송 시각
  };

  // 주변 환경
  surroundings: {
    nearestRoad: number;              // 최근접 도로 거리 (m)
    nearestBuilding: number;          // 최근접 건물 거리 (m)
    landUse: string;                  // 토지 이용
    populationDensity: number;        // 인구 밀도 (명/km²)
  };
}
```

### 5.4 건강 영향 평가

```typescript
interface HealthImpactAssessment {
  assessmentId: string;               // 평가 ID
  period: {
    start: string;
    end: string;
  };
  region: string;                     // 평가 지역

  // 노출 평가
  exposure: {
    population: number;               // 총 인구
    exposedPopulation: {
      above55dB: number;              // 55 dB 이상 노출 인구
      above65dB: number;              // 65 dB 이상 노출 인구
      above75dB: number;              // 75 dB 이상 노출 인구
    };
    averageExposure_dBA: number;      // 평균 노출 소음도
  };

  // 건강 영향
  healthEffects: {
    sleepDisturbance: {
      affectedPopulation: number;     // 영향 받는 인구
      percentage: number;              // 비율 (%)
    };
    annoyance: {
      highlyAnnoyed: number;          // 심각한 불쾌감 인구
      percentage: number;              // 비율 (%)
    };
    cardiovascular: {
      attributableCases: number;      // 귀인 가능 사례 수
      relativeRisk: number;           // 상대 위험도
    };
    cognitiveImpairment: {
      affectedChildren: number;       // 영향 받는 아동 수
      learningDelay: number;          // 학습 지연 (%)
    };
  };

  // 경제적 영향
  economicImpact: {
    healthcareCost: number;           // 의료 비용 (원)
    productivityLoss: number;         // 생산성 손실 (원)
    propertyValueLoss: number;        // 부동산 가치 하락 (원)
    totalCost: number;                // 총 사회적 비용 (원)
  };

  // 권고사항
  recommendations: string[];
}
```

### 5.5 시간대별 소음도

```typescript
interface TimePeriodNoise {
  date: string;                       // 날짜
  location: string;                   // 위치

  // 시간대별 소음도
  periods: {
    day: {                            // 주간 (07:00-18:00)
      leq: number;
      lmax: number;
      exceedances: number;            // 기준 초과 횟수
    };
    evening: {                        // 저녁 (18:00-22:00)
      leq: number;
      lmax: number;
      exceedances: number;
    };
    night: {                          // 야간 (22:00-07:00)
      leq: number;
      lmax: number;
      exceedances: number;
    };
  };

  // 통합 지표
  lden: number;                       // 주야간 등가소음도
  lnight: number;                     // 야간 소음도

  // 준수 여부
  compliance: {
    dayLimit: number;
    eveningLimit: number;
    nightLimit: number;
    isCompliant: boolean;
    violations: string[];
  };
}
```

---

## 6. 소음원 분류

### 6.1 교통 소음

#### 6.1.1 도로 교통

| 차량 유형 | 전형적 소음도 | 최고 소음도 | 주파수 특성 |
|----------|-------------|-----------|-----------|
| 승용차 (50km/h) | 65-70 dBA | 75 dBA | 500-2000 Hz |
| 대형 트럭 (50km/h) | 75-80 dBA | 85 dBA | 125-1000 Hz |
| 오토바이 (50km/h) | 75-85 dBA | 90 dBA | 1000-4000 Hz |
| 버스 (50km/h) | 70-75 dBA | 80 dBA | 250-1000 Hz |
| 전기차 (50km/h) | 60-65 dBA | 70 dBA | 500-2000 Hz |

#### 6.1.2 철도

| 열차 유형 | 전형적 소음도 | 최고 소음도 | 비고 |
|----------|-------------|-----------|------|
| 고속철도 (300km/h) | 85-90 dBA | 95 dBA | 통과 시 |
| 일반 열차 (100km/h) | 75-80 dBA | 85 dBA | 통과 시 |
| 지하철 | 70-75 dBA | 80 dBA | 역 인근 |
| 화물 열차 | 80-85 dBA | 90 dBA | 통과 시 |

#### 6.1.3 항공기

| 항공기 유형 | 이륙 소음도 | 착륙 소음도 | 영향 범위 |
|-----------|-----------|-----------|----------|
| 대형 여객기 | 90-100 dBA | 85-95 dBA | 10km |
| 중형 여객기 | 85-95 dBA | 80-90 dBA | 7km |
| 소형 항공기 | 75-85 dBA | 70-80 dBA | 3km |
| 헬리콥터 | 80-95 dBA | 80-95 dBA | 5km |

### 6.2 산업 소음

| 산업 유형 | 소음 범위 | 주요 소음원 | 특성 |
|----------|----------|-----------|------|
| 중공업 | 80-100 dBA | 단조, 프레스, 용접 | 임펄스 |
| 제조업 | 70-90 dBA | 기계 가동, 조립 라인 | 연속 |
| 건자재 | 85-105 dBA | 파쇄, 분쇄, 믹싱 | 연속 + 임펄스 |
| 섬유/의류 | 75-85 dBA | 방직기, 재봉틀 | 연속 |
| 식품 가공 | 70-80 dBA | 포장, 냉동 설비 | 연속 |

### 6.3 건설 소음

| 장비 유형 | 소음도 (7m 거리) | 주파수 | 사용 시간대 |
|----------|----------------|--------|-----------|
| 항타기 | 95-105 dBA | 저주파 | 제한적 |
| 굴착기 | 80-90 dBA | 중저주파 | 주간 |
| 불도저 | 85-95 dBA | 중저주파 | 주간 |
| 콘크리트 믹서 | 75-85 dBA | 중주파 | 주간 |
| 절단기 | 90-100 dBA | 고주파 | 주간 제한 |
| 천공기 | 95-105 dBA | 고주파 | 주간 제한 |

### 6.4 생활 소음

| 소음원 | 소음도 | 시간 특성 | 주요 불만 |
|--------|--------|----------|----------|
| 층간 소음 | 40-60 dBA | 야간 집중 | 수면 방해 |
| 상업 시설 | 60-75 dBA | 영업시간 | 대화 방해 |
| 냉난방 설비 | 45-65 dBA | 연중 | 지속적 불쾌감 |
| 애완동물 | 60-80 dBA | 불규칙 | 돌발 소음 |
| 공동주택 설비 | 50-70 dBA | 생활시간 | 반복 노출 |

---

## 7. 규제 기준

### 7.1 지역별 소음 환경 기준

#### 7.1.1 대한민국 환경부 기준

| 지역 구분 | 주간<br>(07:00-18:00) | 저녁<br>(18:00-22:00) | 야간<br>(22:00-07:00) |
|----------|---------------------|---------------------|---------------------|
| **일반주거지역** | 55 dBA | 50 dBA | 45 dBA |
| **도로변지역** | 70 dBA | 65 dBA | 60 dBA |
| **상업 · 공업지역** | 65 dBA | 60 dBA | 55 dBA |
| **녹지지역** | 50 dBA | 45 dBA | 40 dBA |
| **학교 · 병원** | 50 dBA | 45 dBA | 40 dBA |

#### 7.1.2 WHO 야간 소음 가이드라인

| 건강 영향 | 야간 평균 (Lnight) | 최대 소음 (Lmax) |
|----------|------------------|----------------|
| 수면 방해 없음 | < 30 dBA | < 45 dBA |
| 경미한 수면 방해 | 30-40 dBA | 45-60 dBA |
| 중간 수면 방해 | 40-55 dBA | 60-70 dBA |
| 심각한 건강 영향 | > 55 dBA | > 70 dBA |

#### 7.1.3 EU 환경 소음 지침 (END)

| 지표 | 위험 수준 | 기준값 |
|------|----------|--------|
| Lden | 주의 (Attention) | 55 dBA |
| Lden | 경고 (Alert) | 65 dBA |
| Lden | 위험 (Critical) | 75 dBA |
| Lnight | 주의 | 45 dBA |
| Lnight | 경고 | 55 dBA |
| Lnight | 위험 | 65 dBA |

### 7.2 특정 소음원 규제

#### 7.2.1 건설 소음 규제

| 시간대 | 허용 소음도 | 비고 |
|--------|-----------|------|
| 주간 (08:00-18:00) | 70 dBA | 일반 작업 가능 |
| 저녁 (18:00-22:00) | 65 dBA | 제한 작업 |
| 야간 (22:00-08:00) | 작업 금지 | 긴급 공사만 허가 |
| 일요일/공휴일 | 작업 금지 | 예외적 허가 필요 |

#### 7.2.2 항공기 소음 규제

| 구역 | WECPNL | 용도 제한 |
|------|--------|----------|
| 1종 구역 | ≥ 95 | 주거 금지 |
| 2종 구역 | 90-95 | 학교, 병원 금지 |
| 3종 구역 | 85-90 | 방음 시설 필수 |

**WECPNL (Weighted Equivalent Continuous Perceived Noise Level)**: 항공기 소음 단위

#### 7.2.3 공장 소음 배출 기준

| 지역 | 주간 | 야간 |
|------|------|------|
| 주거 인접 | 60 dBA | 50 dBA |
| 상업 지역 | 65 dBA | 55 dBA |
| 공업 지역 | 70 dBA | 60 dBA |

### 7.3 정온시설 보호 구역

#### 7.3.1 정온시설 정의

- 학교 (도서관 포함)
- 종합병원 및 요양시설
- 공공도서관
- 어린이집 및 유치원
- 경로당

#### 7.3.2 보호 구역 기준

| 시설 | 보호 거리 | 주간 기준 | 야간 기준 |
|------|----------|----------|----------|
| 학교 | 직선거리 50m | 50 dBA | 45 dBA |
| 병원 | 직선거리 50m | 45 dBA | 40 dBA |
| 도서관 | 직선거리 50m | 45 dBA | 40 dBA |

---

## 8. 건강 영향 평가

### 8.1 소음 노출과 건강 영향

#### 8.1.1 청력 손상

| 노출 레벨 | 노출 시간 | 영향 |
|----------|----------|------|
| 85 dBA | 8시간/일 | 청력 손실 위험 시작 |
| 90 dBA | 4시간/일 | 중간 위험 |
| 95 dBA | 2시간/일 | 높은 위험 |
| 100 dBA | 1시간/일 | 매우 높은 위험 |
| 115 dBA | 15분 | 즉각적 손상 위험 |
| 120 dBA | - | 통증 역치 |

#### 8.1.2 비청각적 영향

| 건강 영향 | 임계 소음도 | 주요 증상 |
|----------|-----------|----------|
| 수면 방해 | 40 dBA (야간) | 수면 단계 변화, 각성 |
| 심혈관 질환 | 55 dBA (Lden) | 고혈압, 심근경색 위험 증가 |
| 인지 기능 저하 | 50 dBA (학교) | 학습 능력 감소, 집중력 저하 |
| 스트레스 | 60 dBA | 코르티솔 증가, 심리적 불안 |
| 불쾌감 (Annoyance) | 55 dBA (Lden) | 삶의 질 저하 |

#### 8.1.3 취약 집단

- **어린이**: 인지 발달 영향, 학습 능력 저하
- **노인**: 청력 손실 가속화, 낙상 위험 증가
- **임산부**: 태아 발달 영향, 조산 위험
- **심혈관 질환자**: 증상 악화
- **수면 장애 환자**: 증상 심화

### 8.2 용량-반응 관계

#### 8.2.1 심각한 불쾌감 (Highly Annoyed)

```
HA(%) = 78.9270 - 3.1172 × Lden + 0.0244 × Lden²
```

예시:
- Lden 50 dBA → HA 약 10%
- Lden 60 dBA → HA 약 20%
- Lden 70 dBA → HA 약 35%

#### 8.2.2 수면 방해

```
HSD(%) = 20.8 - 1.05 × Lnight + 0.01486 × Lnight²
```

예시:
- Lnight 40 dBA → HSD 약 2%
- Lnight 50 dBA → HSD 약 5%
- Lnight 60 dBA → HSD 약 12%

---

## 9. 모니터링 네트워크

### 9.1 측정소 배치 기준

#### 9.1.1 인구 기준

| 도시 인구 | 최소 측정소 수 | 권장 측정소 수 |
|----------|--------------|--------------|
| < 50만 | 5개 | 10개 |
| 50만 ~ 100만 | 10개 | 20개 |
| 100만 ~ 300만 | 20개 | 40개 |
| > 300만 | 40개 | 인구 10만 당 1개 |

#### 9.1.2 면적 기준

- **도심지**: 4km² 당 1개소
- **주거지역**: 10km² 당 1개소
- **공업지역**: 중요 시설별 1개소

#### 9.1.3 특수 위치

- 주요 도로변 (교통량 > 50,000대/일)
- 철도역 및 고속철도 인근
- 공항 주변 (소음 영향권)
- 대형 공사 현장
- 정온시설 보호구역

### 9.2 데이터 전송 및 저장

#### 9.2.1 실시간 전송

```typescript
interface RealtimeDataStream {
  protocol: 'MQTT' | 'WebSocket';
  topic: string;                    // 예: "noise/station/ST001"
  qos: 0 | 1 | 2;                   // MQTT QoS
  frequency: number;                // 전송 주기 (초)
  compression: boolean;             // 데이터 압축 여부
}
```

#### 9.2.2 데이터 저장 주기

| 데이터 유형 | 저장 주기 | 보관 기간 |
|------------|----------|----------|
| 원시 데이터 (Raw) | 1초 | 1개월 |
| 1분 평균 | 1분 | 1년 |
| 1시간 평균 | 1시간 | 5년 |
| 일일 통계 | 1일 | 영구 |

### 9.3 품질 관리

#### 9.3.1 자동 품질 검사

- 레벨 범위 검사: 30 dBA ~ 120 dBA
- 변동률 검사: 급격한 변화 감지
- 통신 상태 검사: 데이터 단절 감지
- 센서 이상 검사: 고장 패턴 인식

#### 9.3.2 정기 교정

- **Class 1 소음계**: 6개월마다 교정
- **Class 2 소음계**: 1년마다 교정
- **현장 점검**: 월 1회
- **음향 교정기**: 음압 레벨 94 dB 또는 114 dB

---

## 10. 저감 대책

### 10.1 발생원 대책

#### 10.1.1 교통 소음 저감

| 대책 | 저감 효과 | 비용 | 적용 |
|------|----------|------|------|
| 저소음 포장 (배수성 아스팔트) | 3-5 dB | 중 | 도로 |
| 방음벽 설치 | 5-15 dB | 고 | 도로/철도 |
| 속도 제한 (10km/h 감소) | 1-2 dB | 저 | 도로 |
| 전기차 전환 | 5-10 dB | - | 차량 |
| 철도 레일 연마 | 3-5 dB | 중 | 철도 |

#### 10.1.2 건설 소음 저감

- 저소음 공법 적용
- 방음 가설 펜스 설치
- 작업 시간 제한 (야간 금지)
- 장비 소음기 부착
- 진동 및 충격 최소화

#### 10.1.3 산업 소음 저감

- 소음원 격리 (차음실)
- 흡음재 설치
- 방음 덮개 및 인클로저
- 장비 유지보수 강화
- 저소음 장비로 교체

### 10.2 전파 경로 대책

#### 10.2.1 방음벽

| 방음벽 유형 | 투과 손실 | 높이 | 재료 |
|-----------|----------|------|------|
| 흡음형 | 10-15 dB | 3-6m | 글라스울 + 타공판 |
| 반사형 | 15-20 dB | 3-6m | 콘크리트, 금속 |
| 투명형 | 10-15 dB | 2-4m | 아크릴, 강화유리 |
| 복합형 | 15-25 dB | 4-8m | 흡음 + 반사 조합 |

#### 10.2.2 완충 녹지대

- **폭 10m**: 3 dB 저감
- **폭 30m**: 5-7 dB 저감
- **폭 100m**: 10-12 dB 저감

#### 10.2.3 건물 배치

- 소음원으로부터 이격 (거리 2배 → -6 dB)
- 건물 차폐 효과 활용
- 창문 배치 최적화

### 10.3 수음점 대책

#### 10.3.1 건물 방음

| 대책 | 차음 성능 | 비용 |
|------|----------|------|
| 이중창 설치 | 25-35 dB | 중 |
| 방음문 교체 | 15-25 dB | 중 |
| 외벽 단열 강화 | 10-15 dB | 고 |
| 환기구 방음 처리 | 10-20 dB | 저 |

#### 10.3.2 개인 보호구

| 보호구 유형 | 차음 성능 | 사용 환경 |
|-----------|----------|----------|
| 귀마개 (foam) | 25-30 dB | 85 dBA 이상 |
| 귀덮개 | 20-35 dB | 90 dBA 이상 |
| 복합형 (귀마개+귀덮개) | 35-40 dB | 100 dBA 이상 |

---

## 11. 성과 지표 (KPI)

### 11.1 환경 성과 지표

| KPI | 목표값 | 측정 단위 | 산정 방식 |
|-----|--------|----------|----------|
| 기준 초과율 감소 | 50% 감소 | % | (기준 초과 측정소 / 전체 측정소) × 100 |
| 평균 소음도 감소 | 5 dB 감소 | dBA | 전체 측정소 Lden 평균 |
| 야간 소음 개선 | 3 dB 감소 | dBA | 야간 Lnight 평균 |
| 조용한 구역 확대 | 20% 증가 | km² | 50 dBA 이하 지역 면적 |

### 11.2 건강 성과 지표

- **고소음 노출 인구 감소**: 65 dBA 이상 노출 인구 30% 감소
- **수면 방해 감소**: 수면 방해 호소율 20% 감소
- **불쾌감 감소**: 심각한 불쾌감 호소율 25% 감소
- **건강 비용 절감**: 소음 관련 의료 비용 10% 절감

### 11.3 관리 성과 지표

- **측정소 가동률**: 95% 이상
- **데이터 품질**: 유효 데이터 비율 98% 이상
- **민원 처리율**: 접수 후 7일 이내 100% 처리
- **규제 준수율**: 소음 배출 시설 95% 이상 기준 준수

---

## 12. 통합 및 상호운용성

### 12.1 WIA 생태계 통합

#### 12.1.1 연계 표준

- **WIA-ENE-027 (Air Quality)**: 대기질-소음 통합 모니터링
- **WIA-ENE-001 (Climate)**: 기후 데이터 연계
- **WIA-SMARTHOME**: 실내 소음 관리
- **WIA-HEALTH**: 건강 영향 평가 연계
- **WIA-SOCIAL**: 시민 참여 플랫폼

#### 12.1.2 API 엔드포인트

```
GET    /api/v1/noise/measurement/{id}       # 측정 데이터 조회
POST   /api/v1/noise/measurement            # 측정 데이터 등록
GET    /api/v1/noise/station/{id}           # 측정소 정보 조회
GET    /api/v1/noise/realtime               # 실시간 데이터 스트림
GET    /api/v1/noise/heatmap                # 소음 지도 데이터
GET    /api/v1/noise/source/{id}            # 소음원 정보
POST   /api/v1/noise/complaint              # 민원 접수
GET    /api/v1/noise/analytics/health       # 건강 영향 분석
GET    /api/v1/noise/compliance             # 준수 여부 조회
```

### 12.2 데이터 교환 포맷

- **기본 포맷**: JSON (UTF-8)
- **실시간 스트림**: MQTT (QoS 1 또는 2)
- **대용량 데이터**: HDF5, NetCDF
- **GIS 연동**: GeoJSON, WMS, WFS

### 12.3 상호운용성 표준

- ISO 1996-1:2016 (소음 측정 기본)
- ISO 1996-2:2017 (환경 소음 평가)
- IEC 61672-1:2013 (소음계 표준)
- ANSI S1.4 (미국 소음계 표준)
- ISO 9613 (소음 감쇠 계산)

---

## 13. 보안 및 개인정보보호

### 13.1 데이터 보안

#### 13.1.1 보안 등급

| 데이터 유형 | 보안 등급 | 암호화 | 접근 통제 |
|------------|----------|--------|----------|
| 측정 데이터 | Public | 불필요 | 공개 API |
| 측정소 위치 | Public | 불필요 | 공개 |
| 민원 정보 | Confidential | AES-256 | 권한자만 |
| 개인 건강 정보 | Highly Confidential | End-to-End | 최소 권한 |

#### 13.1.2 보안 조치

- 전송 중 암호화: TLS 1.3
- API 인증: OAuth 2.0 + JWT
- 침입 탐지: 실시간 이상 패턴 감지
- 감사 로그: 모든 접근 기록
- 백업: 일일 백업, 30일 보관

### 13.2 개인정보보호

- **익명화**: 측정 데이터에 개인 식별 정보 제거
- **동의 기반**: 건강 정보 수집 시 명시적 동의
- **최소 수집**: 필수 정보만 수집
- **보유 기한**: 법적 요구사항 충족 (3년)
- **삭제 권리**: 정보주체의 삭제 요청 처리

---

## 14. 인증 요구사항

### 14.1 측정소 인증

#### 14.1.1 인증 등급

- **Bronze**: 기본 요구사항 충족 (Class 2 장비, 월 1회 점검)
- **Silver**: Class 1 장비, 실시간 전송, 품질 관리
- **Gold**: Class 1 장비, 주파수 분석, 통합 시스템
- **Platinum**: Class 0/1 장비, 연구급 데이터, AI 분석

#### 14.1.2 인증 프로세스

1. 신청 및 서류 제출 (장비 사양, 설치 위치)
2. 현장 실사 (2일)
3. 데이터 검증 (30일 시험 운영)
4. 전문가 심사 (2주)
5. 인증서 발급
6. 정기 사후 관리 (6개월마다)

### 14.2 운영자 교육

- **기본 과정**: 소음 측정 기초 (8시간)
- **전문 과정**: 주파수 분석, 소음 평가 (16시간)
- **관리자 과정**: 네트워크 운영, 데이터 관리 (24시간)
- **재교육**: 2년마다 갱신 (4시간)

### 14.3 장비 검교정

- **음향 교정기**: 연 1회 국가 표준 연계 교정
- **소음계**: Class에 따라 6개월~1년 교정
- **교정 기록**: 블록체인 기반 위변조 방지
- **현장 교정**: 측정 전후 음향 교정기로 검증

---

## 부록

### A. 주파수 가중 곡선

[A, C, Z 가중 곡선 그래프 및 수식]

### B. 소음 지도 작성 가이드

[GIS 기반 소음 지도 제작 방법론]

### C. 민원 처리 프로세스

[소음 민원 접수부터 해결까지 워크플로우]

### D. 샘플 데이터셋

[실제 측정 데이터 JSON 예시]

### E. API 레퍼런스

[전체 API 엔드포인트 상세 문서]

### F. 용어 사전

[200개 이상의 소음 관련 전문 용어]

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

**弘益人間 (홍익인간) · 널리 인간을 이롭게 하라**

조용한 환경은 인간의 기본적인 권리입니다. WIA-ENE-028 표준은 소음으로부터 모든 사람을 보호하고, 건강하고 평화로운 환경을 만드는 데 기여합니다.

**Together, we create a quieter world for all humanity.**

---
