# WIA-CITY-017: 교통 시뮬레이션 표준 🚦

> **홍익인간 (弘益人間) (홍익인간) - 널리 인간을 이롭게 하라**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Version](https://img.shields.io/badge/Version-1.0-blue.svg)](https://github.com/WIA-Official/wia-standards)
[![Standard](https://img.shields.io/badge/Standard-WIA--CITY--017-red.svg)](https://wia.org/standards/city-017)

## 개요

WIA-CITY-017 교통 시뮬레이션 표준은 도시 교통 시스템의 현재 상태 분석, 미래 예측, 정책 평가를 위한 국제 표준입니다.

### 주요 기능

- 🚗 **차량 흐름 시뮬레이션**: 미시적, 중시적, 거시적 모델 지원
- 🚦 **신호등 최적화**: 고정, 감응, 적응 신호 제어
- 👥 **보행자 동선 분석**: Social Force Model 기반
- 🚌 **대중교통 시뮬레이션**: 버스, 지하철, 환승 분석
- 📊 **교통량 예측**: AI/ML 기반 단기/중기/장기 예측
- 🚨 **사고 영향 분석**: 실시간 우회 경로 및 지체 분석
- 🔴 **혼잡 예측**: TCI, TTI, PTI 지수 기반
- 📍 **O-D 매트릭스**: 기원-목적지 통행량 분석
- 🌐 **개방형 표준**: 다양한 시뮬레이션 엔진 간 호환

## 빠른 시작

### 1. 설치

```bash
# 저장소 클론
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/traffic-simulation

# 의존성 설치
./install.sh
```

### 2. TypeScript SDK 사용

```bash
# npm으로 설치
npm install @wia/city-traffic-simulation

# 또는 yarn
yarn add @wia/city-traffic-simulation
```

```typescript
import { TrafficSimulationSDK } from '@wia/city-traffic-simulation';

const sdk = new TrafficSimulationSDK({
  apiKey: 'your-api-key',
  endpoint: 'https://api.wia.org/city-017/v1'
});

// 시뮬레이션 생성
const simulation = await sdk.simulation.create({
  name: '강남역 출근 시간대 분석',
  network: { networkId: 'network-gangnam' },
  timeSettings: {
    startTime: '2025-12-26T07:00:00',
    endTime: '2025-12-26T09:00:00',
    timeStep: 0.5,
    warmupPeriod: 900
  },
  demand: {
    odMatrixId: 'od-peak-morning',
    scalingFactor: 1.0
  },
  models: {
    carFollowing: {
      model: 'idm',
      minGap: 2.0,
      timeHeadway: 1.5
    },
    laneChanging: {
      model: 'mobil',
      anticipationDistance: 200
    },
    routeChoice: 'dynamic'
  },
  output: {
    interval: 300,
    metrics: ['speed', 'volume', 'density', 'delay'],
    aggregationLevel: 'link'
  }
});

// 시뮬레이션 실행 및 대기
const results = await sdk.simulation.runAndWait(simulation, 5000);

console.log('Network Performance:', results.data.networkPerformance);
console.log('Average Speed:', results.data.networkPerformance.avgSpeed, 'km/h');
console.log('Total Delay:', results.data.networkPerformance.totalDelay, 'veh-hours');
```

### 3. CLI 도구 사용

```bash
# API 키 설정
export WIA_TRAFFIC_SIM_API_KEY="your-api-key"

# 시뮬레이션 생성
./cli/traffic-simulation.sh create-simulation \
  "강남 출근 시간대" \
  network-gangnam \
  "2025-12-26T07:00:00" \
  7200

# 실시간 교통 데이터 조회
./cli/traffic-simulation.sh traffic-realtime

# 특정 링크 교통 데이터
./cli/traffic-simulation.sh link-traffic link-001

# 신호 최적화
./cli/traffic-simulation.sh optimize-signals signal-001,signal-002,signal-003

# 교통량 예측
./cli/traffic-simulation.sh forecast-traffic link-001 short

# 혼잡 예측
./cli/traffic-simulation.sh predict-congestion "2025-12-26T18:00:00" 3600
```

## 주요 기능 상세

### 1. 시뮬레이션 유형

#### 미시적 시뮬레이션 (Microscopic Simulation)

개별 차량의 움직임을 초 단위로 추적합니다.

**적용 모델**:
- **차량 추종 모델**: IDM (Intelligent Driver Model), Wiedemann, Gipps
- **차선 변경 모델**: MOBIL, Gipps
- **시간 해상도**: 0.1-1.0초

**사용 사례**:
- 교차로 상세 분석
- 신호 최적화
- 교통 안전 분석
- 자율주행 시뮬레이션

**예시 코드**:
```typescript
// 미시적 시뮬레이션 설정
const microConfig = {
  models: {
    carFollowing: {
      model: 'idm' as const,
      minGap: 2.0,                    // m
      timeHeadway: 1.5,               // s
      idm: {
        delta: 4,                     // acceleration exponent
        a: 1.5,                       // max acceleration (m/s²)
        b: 2.0                        // comfortable deceleration (m/s²)
      }
    },
    laneChanging: {
      model: 'mobil' as const,
      mobil: {
        politeness: 0.5,              // 0-1
        threshold: 0.1,               // m/s²
        safeDeceleration: 4.0         // m/s²
      },
      anticipationDistance: 200       // m
    }
  },
  timeSettings: {
    timeStep: 0.5                     // 0.5초 간격
  }
};
```

#### 중시적 시뮬레이션 (Mesoscopic Simulation)

차량 그룹 단위로 모델링하여 대규모 네트워크 분석에 적합합니다.

**특징**:
- 링크-노드 네트워크 기반
- 분 단위 시간 해상도
- 도시 전체 네트워크 분석

**사용 사례**:
- 도시 전체 교통 분석
- 대규모 인프라 계획
- 통행 배정

#### 거시적 시뮬레이션 (Macroscopic Simulation)

교통 흐름을 유체로 모델링합니다.

**특징**:
- 교통량-밀도-속도 관계식
- 시간 단위 해상도
- 전략적 계획

**사용 사례**:
- 장기 교통 수요 예측
- 정책 평가
- 네트워크 용량 분석

### 2. 신호등 최적화

#### 고정 신호 (Fixed-time Signal)

사전 설정된 고정 주기로 운영됩니다.

```typescript
const fixedSignal: SignalTimingPlan = {
  planId: 'plan-001',
  name: '평일 오전 피크',
  signalId: 'signal-gangnam-01',
  cycleLength: 90,                    // 초
  offset: 10,                         // 초
  phases: [
    {
      phaseId: 1,
      name: '남-북 직진',
      movements: [
        {
          fromLink: 'link-001',
          toLink: 'link-002',
          type: 'through',
          protected: true
        }
      ],
      timing: {
        greenTime: 35,
        yellowTime: 3,
        allRedTime: 2
      }
    },
    {
      phaseId: 2,
      name: '동-서 직진',
      movements: [/* ... */],
      timing: {
        greenTime: 25,
        yellowTime: 3,
        allRedTime: 2
      }
    },
    {
      phaseId: 3,
      name: '좌회전',
      movements: [/* ... */],
      timing: {
        greenTime: 15,
        yellowTime: 3,
        allRedTime: 2
      }
    }
  ]
};
```

#### 감응 신호 (Actuated Signal)

차량 검지기 기반으로 실시간 조정됩니다.

```typescript
const actuatedSignal: SignalTimingPlan = {
  planId: 'plan-actuated-001',
  name: '감응 신호',
  signalId: 'signal-002',
  cycleLength: 120,                   // 최대 주기
  offset: 0,
  phases: [
    {
      phaseId: 1,
      name: '주도로',
      movements: [/* ... */],
      timing: {
        greenTime: 40,                // 초기 녹색 시간
        yellowTime: 3,
        allRedTime: 2,
        minGreenTime: 7,              // 최소 녹색 시간
        maxGreenTime: 60              // 최대 녹색 시간
      },
      detectors: ['det-001', 'det-002']
    }
  ]
};
```

#### 신호 최적화 실행

```typescript
// 여러 신호등 동시 최적화
const optimization = await sdk.signals.optimize(
  ['signal-001', 'signal-002', 'signal-003'],
  {
    objective: 'minimize_delay',      // 또는 'maximize_throughput', 'green_wave'
    constraints: {
      minCycleLength: 60,
      maxCycleLength: 120,
      minGreenTime: 7,
      pedestrianCrossingTime: 15
    }
  }
);

console.log('최적화 결과:');
console.log('- 지체 감소:', optimization.data.improvement.delayReduction, '%');
console.log('- 처리량 증가:', optimization.data.improvement.throughputIncrease, '%');

// 최적화된 타이밍 적용
for (const timing of optimization.data.optimizedTimings) {
  await sdk.signals.updateTiming(timing.signalId, timing);
}
```

### 3. 교통량 예측

#### 단기 예측 (5분 - 1시간)

```typescript
// LSTM 기반 단기 예측
const shortTermForecast = await sdk.forecasting.getForecast('link-001', {
  horizon: 'short',
  forecastTime: '2025-12-26T08:00:00'
});

console.log('단기 예측 (다음 1시간):');
for (const pred of shortTermForecast.data.predictions) {
  console.log(`${pred.timestamp}: ${pred.speed} km/h, ${pred.volume} veh/h`);
}

// 정확도 지표
const accuracy = shortTermForecast.data.metadata.accuracy;
console.log(`MAE: ${accuracy.mae} km/h`);
console.log(`MAPE: ${accuracy.mape}%`);
console.log(`R²: ${accuracy.r2}`);
```

#### 중기 예측 (1시간 - 24시간)

```typescript
// 날씨, 이벤트 정보 포함
const mediumTermForecast = await sdk.forecasting.getForecast('link-001', {
  horizon: 'medium',
  forecastTime: '2025-12-26T18:00:00'
});
```

#### 혼잡 예측

```typescript
// 특정 시간의 네트워크 전체 혼잡 예측
const congestionPrediction = await sdk.forecasting.predictCongestion({
  bbox: {
    minLat: 37.49,
    minLon: 127.01,
    maxLat: 37.52,
    maxLon: 127.06
  },
  time: '2025-12-26T18:00:00',
  duration: 3600                      // 1시간
});

console.log('혼잡 예측:');
for (const pred of congestionPrediction.data.predictions) {
  if (pred.congestionLevel === 'congested' || pred.congestionLevel === 'jammed') {
    console.log(`링크 ${pred.linkId}: ${pred.congestionLevel} (확률: ${pred.probability})`);
  }
}
```

### 4. 사고 영향 분석

#### 사고 생성 및 분석

```typescript
// 사고 시나리오 생성
const incident = await sdk.incidents.create({
  type: 'accident',
  location: {
    linkId: 'link-gangnam-main-001',
    position: 500,                    // m from link start
    coordinates: {
      latitude: 37.4979,
      longitude: 127.0276
    }
  },
  impact: {
    lanesBlocked: [1, 2],             // 2개 차로 차단
    capacityReduction: 60,            // 60% 용량 감소
    speedReduction: 50                // 50% 속도 감소
  },
  time: {
    startTime: new Date().toISOString(),
    duration: 3600                    // 1시간 (예상)
  },
  severity: 'major'
});

console.log('사고 생성:', incident.data.incidentId);

// 사고 영향 분석
const impact = await sdk.incidents.analyzeImpact(incident.data.incidentId);

console.log('사고 영향 분석:');
console.log('- 대기 행렬 길이:', impact.data.spatialImpact.queueLength, 'm');
console.log('- 영향 받은 링크 수:', impact.data.spatialImpact.affectedLinks.length);
console.log('- 총 지체 시간:', impact.data.metrics.totalDelayHours, 'veh-hours');
console.log('- 영향 받은 차량 수:', impact.data.metrics.affectedVehicles);

// 우회 경로
console.log('\n우회 경로:');
for (const detour of impact.data.spatialImpact.detourRoutes) {
  console.log(`경로 ${detour.routeId}:`);
  console.log(`  - 거리: ${detour.distance} m`);
  console.log(`  - 소요 시간: ${detour.travelTime} 초`);
  console.log(`  - 이용 차량: ${detour.vehiclesUsingRoute}`);
}
```

### 5. 대중교통 시뮬레이션

#### 버스 노선 성능 분석

```typescript
// 버스 노선 성능 조회
const busPerformance = await sdk.transit.getPerformance('route-bus-1001', {
  start: '2025-12-26T07:00:00',
  end: '2025-12-26T09:00:00'
});

console.log('버스 노선 성능:');
console.log('- 정시 운행률:', busPerformance.data.onTimePerformance * 100, '%');
console.log('- 평균 지연:', busPerformance.data.avgDelay, '초');
console.log('- 평균 속도:', busPerformance.data.avgSpeed, 'km/h');
console.log('- 평균 탑승률:', busPerformance.data.passengerLoad * 100, '%');

// 노선의 모든 버스 추적
const vehicles = await sdk.transit.listVehiclesOnRoute('route-bus-1001');

console.log('\n운행 중인 버스:');
for (const vehicle of vehicles.data.vehicles) {
  console.log(`버스 ${vehicle.vehicleId}:`);
  console.log(`  - 현재 위치: ${vehicle.currentState.linkId}`);
  console.log(`  - 탑승 인원: ${vehicle.currentState.passengers}/${vehicle.capacity.total}`);
  console.log(`  - 다음 정류장: ${vehicle.currentState.nextStop}`);
  console.log(`  - 지연: ${vehicle.currentState.delay} 초`);
}
```

### 6. 혼잡 분석

#### 현재 혼잡 상황

```typescript
// 실시간 혼잡 데이터
const currentCongestion = await sdk.congestion.getCurrent({
  bbox: {
    minLat: 37.49,
    minLon: 127.01,
    maxLat: 37.52,
    maxLon: 127.06
  }
});

console.log('현재 혼잡 상황:');
for (const congestion of currentCongestion.data.congestion) {
  if (congestion.level === 'congested' || congestion.level === 'jammed') {
    console.log(`링크 ${congestion.linkId}:`);
    console.log(`  - 혼잡도: ${congestion.level}`);
    console.log(`  - TCI: ${congestion.indices.tci.toFixed(1)}%`);
    console.log(`  - TTI: ${congestion.indices.tti.toFixed(2)}`);
    console.log(`  - 유형: ${congestion.characteristics.type}`);
  }
}
```

#### 병목 구간 식별

```typescript
// 재발성 병목 구간 식별
const bottlenecks = await sdk.congestion.identifyBottlenecks({
  threshold: 0.3,                     // 30% 이상 강도
  minDuration: 1800                   // 최소 30분
});

console.log('병목 구간:');
for (const bottleneck of bottlenecks.data.bottlenecks) {
  console.log(`링크 ${bottleneck.linkId}:`);
  console.log(`  - 병목 강도: ${bottleneck.intensity.toFixed(2)}`);
  console.log(`  - 발생 빈도: ${bottleneck.frequency}회/일`);
  console.log(`  - 평균 지속 시간: ${bottleneck.avgDuration} 초`);
}
```

### 7. O-D 매트릭스 분석

#### O-D 매트릭스 생성

```typescript
// 기원-목적지 매트릭스 생성
const odMatrix = await sdk.od.createMatrix({
  name: '평일 아침 피크 O-D',
  timeOfDay: {
    startTime: '07:00',
    endTime: '09:00'
  },
  zones: ['zone-gangnam', 'zone-seocho', 'zone-songpa', 'zone-jamsil'],
  trips: [
    {
      origin: 'zone-gangnam',
      destination: 'zone-seocho',
      trips: 1500,
      mode: 'car'
    },
    {
      origin: 'zone-gangnam',
      destination: 'zone-songpa',
      trips: 800,
      mode: 'car'
    },
    // ... more trips
  ],
  metadata: {
    source: 'household_survey_2025',
    date: '2025-12-01',
    scalingFactor: 1.2
  }
});

console.log('O-D 매트릭스 생성:', odMatrix.data.matrixId);
```

#### 교통량 기반 O-D 추정

```typescript
// 링크 교통량 데이터로부터 O-D 매트릭스 추정
const estimation = await sdk.od.estimateFromCounts({
  linkCounts: [
    { linkId: 'link-001', count: 1800 },
    { linkId: 'link-002', count: 1500 },
    { linkId: 'link-003', count: 2200 },
    // ... more counts
  ],
  seedMatrix: 'od-base-2024'          // 시드 매트릭스 (옵션)
});

console.log('추정된 O-D 매트릭스:', estimation.data.matrixId);
console.log('추정 정확도:', estimation.data.accuracy * 100, '%');
```

## API 레퍼런스

### Simulation API

#### 시뮬레이션 생성
```typescript
await sdk.simulation.create(config: SimulationConfig): Promise<{ simulationId: string }>
```

#### 시뮬레이션 시작
```typescript
await sdk.simulation.start(simulationId: string): Promise<{ success: boolean }>
```

#### 시뮬레이션 상태 조회
```typescript
await sdk.simulation.getStatus(simulationId: string): Promise<SimulationState>
```

#### 시뮬레이션 결과 조회
```typescript
await sdk.simulation.getResults(simulationId: string): Promise<SimulationResults>
```

#### 시뮬레이션 실행 및 대기
```typescript
await sdk.simulation.runAndWait(
  config: SimulationConfig,
  pollInterval?: number
): Promise<SimulationResults>
```

### Traffic API

#### 실시간 교통 데이터
```typescript
await sdk.traffic.getRealtime(params?: {
  linkIds?: string[];
  bbox?: BoundingBox;
}): Promise<{ states: TrafficState[] }>
```

#### 링크 교통 데이터
```typescript
await sdk.traffic.getLinkTraffic(
  linkId: string,
  params?: { start?: string; end?: string; interval?: number }
): Promise<{ states: TrafficState[] }>
```

### Signals API

#### 신호 최적화
```typescript
await sdk.signals.optimize(
  signalIds: string[],
  params?: { objective?: string; constraints?: any }
): Promise<{
  optimizedTimings: SignalTimingPlan[];
  improvement: { delayReduction: number; throughputIncrease: number };
}>
```

### Forecasting API

#### 교통량 예측
```typescript
await sdk.forecasting.getForecast(
  linkId: string,
  params: { horizon: 'short' | 'medium' | 'long'; forecastTime?: string }
): Promise<TrafficForecast>
```

#### 혼잡 예측
```typescript
await sdk.forecasting.predictCongestion(params: {
  bbox?: BoundingBox;
  linkIds?: string[];
  time: string;
  duration?: number;
}): Promise<{ predictions: CongestionPrediction[] }>
```

## 성능 지표

### 교통 효율성 KPI

| 지표 | 목표 값 | 설명 |
|------|---------|------|
| 평균 통행 속도 | > 40 km/h | 네트워크 평균 속도 |
| 평균 지체 | < 30 초/대 | 차량당 평균 지체 시간 |
| 네트워크 평균 LOS | C 이상 | 서비스 수준 |
| V/C 비율 | < 0.85 | 교통량/용량 비율 |

### 시뮬레이션 정확도

| 지표 | 목표 값 | 설명 |
|------|---------|------|
| GEH < 5 | > 85% of links | 교통량 비교 통계량 |
| MAPE (속도) | < 15% | 평균 절대 백분율 오차 |
| RMSE (교통량) | < 200 veh/h | 평균 제곱근 오차 |

## 시뮬레이션 모델

### 차량 추종 모델

#### IDM (Intelligent Driver Model)
```
가속도 = a_max * [1 - (v/v_0)^4 - (s*/s)²]

s* = s_0 + v*T + (v*Δv) / (2*√(a_max * b))
```

#### Wiedemann Model
- 자유 주행, 접근, 추종, 감속 4개 상태
- PTV VISSIM 기본 모델

### 차선 변경 모델

#### MOBIL
- 안전성 조건: 뒤차의 감속 < 안전 감속도
- 유인성 조건: 자신과 주변 차량의 이득 고려
- 정중함 계수 (politeness factor) 적용

## 환경 변수

```bash
# API 설정
export WIA_TRAFFIC_SIM_ENDPOINT="https://api.wia.org/city-017/v1"
export WIA_TRAFFIC_SIM_API_KEY="your-api-key"

# 로깅 (선택)
export LOG_LEVEL="info"
```

## 자주 묻는 질문

### Q: 시뮬레이션은 얼마나 정확한가요?
A: 검증된 시뮬레이션의 경우 실제 교통량 대비 GEH < 5를 85% 이상 링크에서 달성합니다. 속도 예측은 MAPE < 15% 수준입니다.

### Q: 시뮬레이션 실행 시간은?
A: 네트워크 크기와 시뮬레이션 기간에 따라 다릅니다. 일반적으로 실제 시간 대비 1:10 ~ 1:100 비율입니다 (예: 2시간 시뮬레이션 → 1-12분 실행).

### Q: 자율주행 차량 시뮬레이션도 가능한가요?
A: 네, 차량 특성에서 ACC (Adaptive Cruise Control), CACC (Cooperative ACC) 모델을 지원합니다.

### Q: 실시간 교통 데이터 통합은?
A: 루프 검지기, 영상 검지기, GPS 프로브 데이터 등 다양한 소스의 실시간 데이터를 통합할 수 있습니다.

### Q: 다른 시뮬레이터와 호환되나요?
A: WIA 표준을 따르는 모든 시뮬레이터와 호환됩니다. SUMO, VISSIM, Aimsun 등과 데이터 교환 가능합니다.

## 라이선스

MIT License - 자유롭게 사용, 수정, 배포 가능합니다.

## 지원

- 📖 문서: https://wia.org/standards/city-017
- 💬 커뮤니티: https://github.com/WIA-Official/wia-standards
- 📧 이메일: standards@wia-official.org

## 기여

기여를 환영합니다! Pull Request를 보내주세요.

---

**홍익인간 (弘益人間) (홍익인간) - Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
