# WIA-CITY-016: 도시 계획 표준 🏙️

> **홍익인간 (弘益人間) (홍익인간) - 널리 인간을 이롭게 하라**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Version](https://img.shields.io/badge/Version-1.0-blue.svg)](https://github.com/WIA-Official/wia-standards)
[![Standard](https://img.shields.io/badge/Standard-WIA--CITY--016-red.svg)](https://wia.org/standards/city-016)

## 개요

WIA-CITY-016 도시 계획 표준은 지속 가능하고 효율적이며 인간 중심적인 도시 개발을 위한 국제 표준입니다.

### 주요 기능

- 🏞️ **토지 이용 계획**: 용도 지역, 건폐율, 용적률 관리
- 🏙️ **용도 지역 관리**: 주거, 상업, 공업 지역 규제
- 🌳 **녹지 및 공원 계획**: 1인당 녹지 면적, 접근성 관리
- 🛣️ **교통 인프라**: 도로, 대중교통, 보행/자전거 네트워크
- 💧 **상하수도 인프라**: 정수장, 하수처리장, 배관 시스템
- ⚡ **에너지 인프라**: 전력망, 신재생 에너지 통합
- 👥 **인구 밀도 분석**: 밀도 계산, 인구 예측
- 🌆 **도시 성장 시뮬레이션**: 시나리오 기반 성장 예측
- 📊 **환경 영향 평가**: 대기질, 수질, 소음 분석
- 🔍 **접근성 분석**: 대중교통, 편의시설 접근성

## 빠른 시작

### 1. 설치

```bash
# 저장소 클론
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/urban-planning

# 의존성 설치
./install.sh
```

### 2. TypeScript SDK 사용

```bash
# npm으로 설치
npm install @wia/city-urban-planning

# 또는 yarn
yarn add @wia/city-urban-planning
```

```typescript
import { UrbanPlanningSDK } from '@wia/city-urban-planning';

const sdk = new UrbanPlanningSDK({
  apiKey: 'your-api-key',
  endpoint: 'https://api.wia.org/city-016/v1'
});

// 토지 이용 조회
const landUse = await sdk.landUse.get('parcel-001');

// 용도 지역 호환성 확인
const compatibility = await sdk.zoning.checkCompatibility(
  'residential_low',
  'commercial_central'
);

// 인구 밀도 계산
const density = await sdk.population.calculateDensity({
  zoneId: 'zone-001',
  type: 'gross'
});

// 도시 성장 시뮬레이션
const simulation = await sdk.simulation.simulateGrowth({
  startYear: 2025,
  endYear: 2050,
  timeStep: 1,
  scenario: 'compact',
  populationGrowthRate: 1.5,
  economicGrowthRate: 2.0,
  constraints: {}
});

// 접근성 분석
const accessibility = await sdk.analysis.analyzeAccessibility('zone-001');
```

### 3. CLI 도구 사용

```bash
# API 키 설정
export WIA_URBAN_PLANNING_API_KEY="your-api-key"

# 토지 이용 목록 조회
./cli/urban-planning.sh list-land-use

# 용도 지역 호환성 확인
./cli/urban-planning.sh check-compatibility residential_low commercial_central

# 인구 밀도 계산
./cli/urban-planning.sh calculate-density zone-001

# 도시 성장 시뮬레이션
./cli/urban-planning.sh simulate-growth 2025 2050 compact

# 접근성 분석
./cli/urban-planning.sh analyze-accessibility zone-001
```

## 주요 기능 상세

### 1. 토지 이용 계획

#### 토지 이용 분류

**주거 용지 (Residential)**:
- 저밀도 주거: 건폐율 ≤40%, 용적률 ≤100%, 1-2층
- 중밀도 주거: 건폐율 ≤50%, 용적률 100-200%, 3-5층
- 고밀도 주거: 건폐율 ≤60%, 용적률 200-500%, 6-30층

**상업 용지 (Commercial)**:
- 근린 상업: 건폐율 ≤60%, 용적률 150-300%, 1-5층
- 중심 상업: 건폐율 ≤70%, 용적률 300-800%, 5-20층

**공업 용지 (Industrial)**:
- 경공업: 건폐율 ≤60%, 용적률 100-200%
- 중공업: 건폐율 ≤70%, 용적률 150-300%

#### 용도 혼합 (Mixed-Use Development)

**수직 혼합**:
- 1층: 상업 시설
- 2-5층: 오피스
- 6층 이상: 주거

**수평 혼합**:
- 블록 단위 용도 배치
- 주거-상업-업무 근접 배치
- 보행 거리 최소화

### 2. 용도 지역 관리

#### 주거 지역

**제1종 전용주거지역**:
- 건폐율: ≤50%, 용적률: ≤100%
- 허용 용도: 단독주택, 공동주택, 근린생활시설
- 금지 용도: 공장, 위험물 저장소, 유흥 시설

**제2종 일반주거지역**:
- 건폐율: ≤60%, 용적률: 150-250%
- 허용 용도: 모든 주거 시설, 근린상업시설, 교육·복지시설

**제3종 일반주거지역**:
- 건폐율: ≤60%, 용적률: 200-400%
- 허용 용도: 모든 주거 및 상업 시설, 업무 시설

#### 상업 지역

**중심상업지역**:
- 건폐율: ≤70%, 용적률: 400-1,000%
- 허용 용도: 백화점, 오피스 빌딩, 호텔, 문화 시설

**근린상업지역**:
- 건폐율: ≤60%, 용적률: 200-400%
- 허용 용도: 슈퍼마켓, 음식점, 병원, 주거 시설

#### 공업 지역

**전용공업지역**:
- 건폐율: ≤70%, 용적률: 200-400%
- 허용 용도: 제조 공장, 물류 창고, 연구개발시설
- 금지 용도: 주거 시설, 학교, 대규모 상업 시설

**일반공업지역**:
- 건폐율: ≤60%, 용적률: 150-300%
- 허용 용도: 경공업 공장, 첨단 제조업, 창업 지원 시설

### 3. 건축물 높이 및 밀도 규제

#### 높이 규제

| 용도 지역 | 최대 높이 | 최대 층수 |
|----------|----------|----------|
| 저층주거 | 15m | 4층 |
| 중층주거 | 35m | 10층 |
| 고층주거 | 100m | 30층 |
| 근린상업 | 50m | 15층 |
| 중심상업 | 제한없음 | 제한없음 |
| 업무지역 | 200m | 60층 |
| 공업지역 | 50m | 15층 |

#### 사선 제한

**북측 사선 제한**:
- 인접 대지 경계로부터 높이의 1/2 이격
- 목적: 일조권 보호

**도로 사선 제한**:
- 도로 폭의 1.5배 높이에서 45° 사선
- 목적: 스카이라인 보호

#### 보너스 용적률

- 공개공지 제공: +10-20%
- 친환경 건축: +5-15%
- 문화 시설 설치: +5-10%
- 대중교통 연계: +10-30%

### 4. 녹지 및 공원 계획

#### 녹지 위계

**광역 녹지 (Regional Green)**:
- 규모: 100ha 이상
- 기능: 생태 보전, 레크리에이션
- 접근성: 차량 30분 이내

**도시 공원 (Urban Park)**:
- 규모: 10-100ha
- 기능: 시민 휴식, 문화 활동
- 접근성: 자전거 20분 이내

**근린 공원 (Neighborhood Park)**:
- 규모: 1-10ha
- 기능: 일상 휴식, 놀이터
- 접근성: 도보 10분 이내

**소공원 (Pocket Park)**:
- 규모: 0.1-1ha
- 기능: 소규모 휴게 공간
- 접근성: 도보 5분 이내

#### 녹지 기준

| 용도 지역 | 최소 녹지율 | 권장 녹지율 |
|----------|-----------|-----------|
| 저층주거 | 30% | 40% |
| 중층주거 | 25% | 35% |
| 고층주거 | 20% | 30% |
| 상업지역 | 15% | 25% |
| 업무지역 | 20% | 30% |
| 공업지역 | 15% | 25% |

#### 1인당 공원 면적

- **WHO 권장 기준**: 9m²/인
- **우수 도시 기준**: 15-20m²/인
- **최소 기준**: 6m²/인

### 5. 교통 인프라 계획

#### 도로 위계

**고속도로 (Highway)**:
- 폭: 25-50m, 차선: 4-8차선
- 속도: 80-120 km/h, 간격: 10-20km

**간선도로 (Arterial Road)**:
- 폭: 20-35m, 차선: 4-6차선
- 속도: 60-80 km/h, 간격: 2-5km

**집산도로 (Collector Road)**:
- 폭: 15-25m, 차선: 2-4차선
- 속도: 40-60 km/h, 간격: 500m-2km

**국지도로 (Local Road)**:
- 폭: 8-15m, 차선: 1-2차선
- 속도: 30-40 km/h, 간격: 100-500m

#### 대중교통

**지하철 (Metro)**:
- 역 간격: 800m-1,500m
- 역세권: 반경 500m (도보 10분)
- 배차 간격: 첨두 3분, 비첨두 6분

**경전철 (Light Rail)**:
- 역 간격: 500m-1,000m
- 역세권: 반경 400m
- 배차 간격: 첨두 5분, 비첨두 10분

**BRT (Bus Rapid Transit)**:
- 정류장 간격: 400m-800m
- 전용 차로 운영
- 배차 간격: 첨두 5분, 비첨두 10분

#### TOD (Transit-Oriented Development)

**반경 200m (Core Zone)**:
- 용적률: 500-1,000%
- 용도: 상업, 업무 중심
- 주차: 최소화 (0.5대/100m²)

**반경 500m (Secondary Zone)**:
- 용적률: 300-500%
- 용도: 혼합 용도
- 주차: 저감 (0.8대/100m²)

**반경 800m (Tertiary Zone)**:
- 용적률: 200-300%
- 용도: 주거 중심
- 주차: 일반 기준

#### 보행 및 자전거

**보도 폭원**:
- 주요 상업 지역: 5-10m
- 일반 도로: 3-5m
- 주거 지역: 2-3m

**자전거 도로**:
- 독립 전용도로: 3-4m (양방향)
- 전용차로: 1.5-2m (편도)
- 공유 도로: 차량 속도 제한 30 km/h

### 6. 상하수도 인프라

#### 상수도 시스템

**수요 예측**:
- 주거: 200-300 L/일·인
- 상업: 50-100 L/일·m²
- 공업: 100-500 L/일·m²

**시설 계획**:
- 정수장: 일 최대 수요의 120%
- 배수지: 일 평균 수요의 12시간분
- 배수관: 최소 구경 75mm (주택), 150mm (간선)

#### 하수도 시스템

**하수 발생량**:
- 오수: 상수도 사용량의 80-90%
- 우수: Q = C × I × A

**하수 처리**:
- 처리 공법: 고도 처리 (질소, 인 제거)
- 처리 수준: 방류수 BOD < 10 mg/L

#### 빗물 관리 (LID)

**투수성 포장**:
- 보도, 주차장: 투수 블록
- 침투량: 50-80%

**빗물 정원 (Rain Garden)**:
- 규모: 배수 면적의 5-10%
- 침투: 24시간 이내 배수

### 7. 에너지 및 통신 인프라

#### 전력 시스템

**전력 원단위**:

| 용도 | 전력 원단위 | 피크 계수 |
|------|------------|----------|
| 주거 | 30-50 kWh/m²·년 | 1.5 |
| 상업 | 150-250 kWh/m²·년 | 2.0 |
| 업무 | 100-200 kWh/m²·년 | 1.8 |
| 공업 | 200-500 kWh/m²·년 | 1.3 |

**배전 계획**:
- 변전소: 지역 수요의 130%
- 배전선: 22.9 kV (1차), 220/380 V (2차)
- 지중화율: 도심 100%, 주거 80%

#### 신재생 에너지

**건물 일체형 태양광 (BIPV)**:
- 설치 비율: 옥상 면적의 50%
- 발전 용량: 100-200 W/m²

**지역 냉난방**:
- 열병합 발전 (CHP): 효율 80-90%
- 공급: 지역 난방 + 전력

### 8. 인구 밀도 분석

#### 인구 추정

```
인구 = 세대수 × 세대당 인구
세대수 = (주거 용지 면적 × 용적률 × 주거 비율) / 평균 주호 면적
```

**세대당 인구**:
- 신도시: 2.5-3.0인
- 기성 시가지: 2.2-2.5인

#### 밀도 기준

**양호한 주거 환경**:
- 순밀도: 100-200 호/ha
- 인구 밀도: 250-500 명/ha
- 총밀도: 50-100 호/ha

### 9. 도시 성장 시뮬레이션

#### 시뮬레이션 모델

**셀룰러 오토마타 (Cellular Automata)**:
```
P(개발) = f(접근성, 인접도, 경사, 용도규제)
```

**에이전트 기반 모델 (Agent-Based Model)**:
- 가구 (주거지 선택)
- 기업 (입지 선택)
- 개발업자 (개발 결정)

#### 시나리오

**BAU (Business As Usual)**:
- 현재 추세 지속

**컴팩트 시티 (Compact City)**:
- 고밀도 개발
- 대중교통 중심

**분산 개발 (Sprawl)**:
- 저밀도 확산
- 자동차 중심

### 10. 환경 영향 평가

#### 환경 지표

**대기질**:
- PM2.5, PM10, NO2, SO2

**수질**:
- BOD, COD, SS, T-N, T-P

**소음**:
- 주간: < 60 dB(A)
- 야간: < 50 dB(A)

## API 엔드포인트

### 토지 이용 관리

```
GET    /api/v1/land-use              # 토지 이용 목록
GET    /api/v1/land-use/{id}         # 토지 이용 조회
POST   /api/v1/land-use              # 토지 이용 생성
PUT    /api/v1/land-use/{id}         # 토지 이용 업데이트
DELETE /api/v1/land-use/{id}         # 토지 이용 삭제
POST   /api/v1/land-use/analyze      # 토지 이용 분석
```

### 용도 지역 관리

```
GET    /api/v1/zoning                # 용도 지역 목록
GET    /api/v1/zoning/{id}           # 용도 지역 조회
POST   /api/v1/zoning                # 용도 지역 생성
PUT    /api/v1/zoning/{id}           # 용도 지역 업데이트
POST   /api/v1/zoning/compatibility  # 호환성 분석
POST   /api/v1/zoning/{id}/capacity  # 개발 용량 계산
```

### 녹지 및 공원

```
GET    /api/v1/green-space           # 녹지 목록
GET    /api/v1/green-space/{id}      # 녹지 조회
POST   /api/v1/green-space           # 녹지 생성
POST   /api/v1/green-space/per-capita      # 1인당 녹지 면적
POST   /api/v1/green-space/accessibility   # 접근성 분석
```

### 인프라

```
GET    /api/v1/infrastructure/roads          # 도로 네트워크
GET    /api/v1/infrastructure/transit        # 대중교통
POST   /api/v1/infrastructure/plan           # 인프라 계획
POST   /api/v1/infrastructure/capacity       # 용량 분석
```

### 인구 및 밀도

```
GET    /api/v1/population/{id}       # 인구 데이터 조회
GET    /api/v1/population            # 인구 데이터 목록
POST   /api/v1/population/density    # 밀도 계산
POST   /api/v1/population/project    # 인구 예측
POST   /api/v1/population/{id}/demographics  # 인구통계 분석
```

### 도시 계획

```
GET    /api/v1/plans                 # 계획 목록
GET    /api/v1/plans/{id}            # 계획 조회
POST   /api/v1/plans                 # 계획 생성
PUT    /api/v1/plans/{id}            # 계획 업데이트
DELETE /api/v1/plans/{id}            # 계획 삭제
GET    /api/v1/plans/{id}/status     # 계획 진행 상황
```

### 시뮬레이션

```
POST   /api/v1/simulation/growth           # 성장 시뮬레이션
GET    /api/v1/simulation/{id}             # 시뮬레이션 결과
POST   /api/v1/simulation/traffic          # 교통 시뮬레이션
POST   /api/v1/simulation/environmental    # 환경 영향 시뮬레이션
POST   /api/v1/simulation/compare          # 시나리오 비교
```

### 분석

```
POST   /api/v1/analysis/accessibility       # 접근성 분석
POST   /api/v1/analysis/density             # 밀도 분석
POST   /api/v1/analysis/land-use-efficiency # 토지 이용 효율성
POST   /api/v1/analysis/sustainability      # 지속가능성 분석
POST   /api/v1/analysis/equity              # 형평성 분석
```

## 디렉토리 구조

```
urban-planning/
├── spec/
│   └── WIA-CITY-016-v1.0.md          # 상세 스펙
├── api/
│   └── typescript/
│       ├── src/
│       │   ├── types.ts              # 타입 정의
│       │   └── index.ts              # SDK
│       └── package.json
├── cli/
│   └── urban-planning.sh             # CLI 도구
├── README.md
└── install.sh
```

## 통합 예제

### 1. Node.js 백엔드

```typescript
import { UrbanPlanningSDK } from '@wia/city-urban-planning';

const sdk = new UrbanPlanningSDK({
  apiKey: process.env.WIA_API_KEY,
  endpoint: 'https://api.wia.org/city-016/v1'
});

// Express 라우트
app.post('/api/zoning/compatibility', async (req, res) => {
  const { zone1, zone2 } = req.body;
  const result = await sdk.zoning.checkCompatibility(zone1, zone2);
  res.json(result);
});

app.post('/api/simulation/growth', async (req, res) => {
  const params = req.body;
  const result = await sdk.simulation.simulateGrowth(params);
  res.json(result);
});
```

### 2. React 프론트엔드

```tsx
import { UrbanPlanningSDK } from '@wia/city-urban-planning';
import { useState, useEffect } from 'react';

function UrbanPlanDashboard() {
  const [plans, setPlans] = useState([]);
  const sdk = new UrbanPlanningSDK({
    apiKey: 'your-key',
    endpoint: 'https://api.wia.org/city-016/v1'
  });

  useEffect(() => {
    sdk.plans.list().then((res) => {
      setPlans(res.data.items);
    });
  }, []);

  const runSimulation = async (planId) => {
    const result = await sdk.simulation.simulateGrowth({
      startYear: 2025,
      endYear: 2050,
      timeStep: 5,
      scenario: 'compact',
      populationGrowthRate: 1.5,
      economicGrowthRate: 2.0,
      constraints: {}
    });
    console.log(result);
  };

  return (
    <div>
      <h1>도시 계획 대시보드</h1>
      {plans.map((plan) => (
        <div key={plan.planId}>
          <h2>{plan.name}</h2>
          <p>상태: {plan.status}</p>
          <button onClick={() => runSimulation(plan.planId)}>
            시뮬레이션 실행
          </button>
        </div>
      ))}
    </div>
  );
}
```

### 3. GIS 통합 (Leaflet)

```typescript
import L from 'leaflet';
import { UrbanPlanningSDK } from '@wia/city-urban-planning';

const sdk = new UrbanPlanningSDK({
  apiKey: 'your-key',
  endpoint: 'https://api.wia.org/city-016/v1'
});

// 지도 초기화
const map = L.map('map').setView([37.5665, 126.9780], 13);

// 용도 지역 레이어
const zoningLayer = async () => {
  const zones = await sdk.zoning.list();

  zones.data.items.forEach((zone) => {
    const polygon = L.polygon(zone.boundary, {
      color: getColorByZoning(zone.type),
      fillOpacity: 0.3
    });

    polygon.bindPopup(`
      <b>${zone.name}</b><br>
      유형: ${zone.type}<br>
      면적: ${zone.totalArea.toLocaleString()}m²<br>
      용적률: ${zone.regulation.far}%
    `);

    polygon.addTo(map);
  });
};

zoningLayer();
```

## 성과 지표 (KPI)

### 지속가능성

- 녹지율: > 30%
- 1인당 공원 면적: > 9 m²/인
- 신재생 에너지 비율: > 20%
- 탄소 배출량: < 5 톤CO2/인·년

### 이동성

- 대중교통 분담률: > 50%
- 평균 통근 시간: < 45분
- 역세권 거주 인구: > 60%
- 보행 친화성: > 70점

### 삶의 질

- 도보권 편의시설 접근성: > 80%
- 주거 만족도: > 7/10
- 공기질 우수일: > 250일/년
- 1인당 생활공간: > 25m²

### 경제성

- 고용률: > 65%
- 실업률: < 5%
- 혁신 클러스터: > 3개
- 창업 생존율: > 60%

## 보안 및 개인정보보호

### 인증 및 권한

- OAuth 2.0 인증
- 역할 기반 접근 제어 (RBAC)
- API 키 관리

### 데이터 보안

- 전송 중 암호화: TLS 1.3
- 저장 암호화: AES-256
- 접근 로그 기록

### 개인정보보호

- GDPR 준수
- 데이터 최소화
- 익명화 처리
- 사용자 동의 기반 수집

## 문서

- [상세 스펙](./spec/WIA-CITY-016-v1.0.md): 전체 기술 사양
- [API 레퍼런스](https://api.wia.org/city-016/docs): OpenAPI 3.0 문서
- [SDK 문서](./api/typescript/README.md): TypeScript SDK 가이드
- [CLI 가이드](./cli/README.md): 명령줄 도구 사용법

## 관련 표준

- **WIA-CITY-009 (Smart Lighting)**: 스마트 조명 시스템
- **WIA-CITY-010 (HVAC)**: 냉난방 시스템
- **WIA-CITY-015 (Traffic Simulation)**: 교통 시뮬레이션
- **WIA-ENE-007 (Smart Grid)**: 스마트 그리드
- **WIA-ENE-016 (Air Quality)**: 대기질 관리

## 라이선스

© 2025 WIA (World Certification Industry Association)

이 표준은 MIT License 하에 배포됩니다.

## 기여

기여를 환영합니다! Pull Request를 보내주세요.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 연락처

**WIA 표준 사무국**
- 웹사이트: https://wiastandards.com
- 이메일: standards@wia-official.org
- GitHub: https://github.com/WIA-Official/wia-standards

## 홍익인간 (弘益人間) (홍익인간) · 널리 인간을 이롭게 하라

도시 계획은 단순히 건물과 도로를 배치하는 기술을 넘어, 사람들이 더 나은 삶을 살 수 있는 공간을 만드는 일입니다.

WIA-CITY-016 표준은 개방형 표준, 투명한 프로세스, 협력적 개발을 통해 도시 계획 기술이 인류 전체의 공동선에 기여하도록 보장합니다.

**함께, 우리는 지속 가능하고 포용적이며 회복탄력적인 도시를 만듭니다.** 🏙️

---

© 2025 SmileStory Inc. / WIA
**홍익인간 (弘益人間) (홍익인간) · Benefit All Humanity**
