# WIA-CLIMATE Phase 3: Adaptation (기후 적응)
# Climate Adaptation Framework Standard

**Version**: 1.0.0
**Status**: Draft
**Last Updated**: 2025-01-01

---

## 1. 개요 (Overview)

### 1.1 목적

WIA-CLIMATE-ADAPTATION 표준은 기후변화 영향에 대한 적응 계획 수립, 리스크 평가, 조기경보 시스템을 위한 통합 프레임워크를 제공합니다.

**핵심 목표**:
- 기후 리스크 평가 프레임워크
- 조기경보시스템 데이터 표준
- 적응 조치 효과성 평가
- 기후 이주 예측 모델

### 1.2 기후 적응 vs 완화

```
┌─────────────────────────────────────────────────────────────┐
│                  기후 대응: 적응 vs 완화                      │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  완화 (Mitigation)               적응 (Adaptation)          │
│  ──────────────────              ──────────────────          │
│  원인 해결                        결과 대응                   │
│  배출량 감축                      영향 최소화                 │
│  글로벌 효과                      지역 효과                   │
│  장기 (수십 년)                   단기~중기                   │
│                                                             │
│         완화                      적응                       │
│           │                        │                        │
│           ▼                        ▼                        │
│   ┌──────────────┐        ┌──────────────┐                 │
│   │ 배출량 감소   │        │ 피해 최소화   │                 │
│   │              │        │              │                 │
│   │  MRV 표준    │        │  적응 표준    │                 │
│   │  탄소시장     │        │  리스크 평가  │                 │
│   └──────────────┘        └──────────────┘                 │
│           │                        │                        │
│           └────────┬───────────────┘                        │
│                    ▼                                        │
│            ┌──────────────┐                                 │
│            │ 기후 회복력   │                                 │
│            │ (Resilience) │                                 │
│            └──────────────┘                                 │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. 기후 리스크 평가 (Climate Risk Assessment)

### 2.1 리스크 공식

```
┌─────────────────────────────────────────────────────────────┐
│                    기후 리스크 공식 (IPCC)                    │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│   Risk = f(Hazard, Exposure, Vulnerability)                 │
│                                                             │
│   위험 = 위해요소 × 노출 × 취약성 / 적응역량                 │
│                                                             │
│   ┌─────────────────────────────────────────────────────┐   │
│   │                                                     │   │
│   │         Hazard          위해요소                    │   │
│   │        (기후현상)        ├── 홍수                   │   │
│   │            │             ├── 가뭄                   │   │
│   │            ▼             ├── 폭염                   │   │
│   │   ┌───────────────┐     └── 해수면 상승            │   │
│   │   │               │                                 │   │
│   │   │     RISK      │                                 │   │
│   │   │               │                                 │   │
│   │   └───────────────┘                                 │   │
│   │      ▲         ▲                                    │   │
│   │      │         │                                    │   │
│   │  Exposure  Vulnerability    노출        취약성       │   │
│   │  (노출)    (취약성)       인구/자산    사회/경제적   │   │
│   │                                                     │   │
│   └─────────────────────────────────────────────────────┘   │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 2.2 기후 위해요소 (Climate Hazards)

| 위해요소 | 유형 | 주요 지표 | 시나리오 영향 |
|----------|------|-----------|--------------|
| 하천 홍수 | 급성 | 강수량, 수위 | SSP5-8.5: +40% 빈도 |
| 해안 홍수 | 급성 | 해수면, 폭풍해일 | +1m 해수면 (2100) |
| 가뭄 | 만성 | SPI, 토양 수분 | 가뭄 면적 2배 |
| 폭염 | 급성 | 열대야, 최고온도 | +4°C (2100) |
| 산불 | 급성 | 건조도, 풍속 | 위험일 +50% |
| 태풍 | 급성 | 최대풍속, 강수량 | 강도 증가 |
| 해수면 상승 | 만성 | 연간 상승률 | 1m (SSP5-8.5) |

### 2.3 리스크 평가 데이터 스키마

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://wia.org/schemas/climate/risk-assessment.json",
  "title": "WIA Climate Risk Assessment",
  "type": "object",
  "required": ["riskId", "location", "hazard", "scenario", "timeHorizon"],
  "properties": {
    "riskId": { "type": "string" },
    "location": { "$ref": "#/$defs/geoLocation" },
    "hazard": {
      "type": "string",
      "enum": [
        "FLOOD_RIVERINE", "FLOOD_COASTAL", "FLOOD_FLASH",
        "DROUGHT", "HEAT_WAVE", "COLD_WAVE",
        "WILDFIRE", "TROPICAL_CYCLONE",
        "SEA_LEVEL_RISE", "OCEAN_ACIDIFICATION"
      ]
    },
    "exposure": { "$ref": "#/$defs/exposure" },
    "vulnerability": { "$ref": "#/$defs/vulnerability" },
    "adaptiveCapacity": { "type": "number", "minimum": 0, "maximum": 1 },
    "riskScore": { "type": "number", "minimum": 0, "maximum": 100 },
    "timeHorizon": { "enum": ["2030", "2050", "2100"] },
    "scenario": { "enum": ["SSP1-1.9", "SSP1-2.6", "SSP2-4.5", "SSP3-7.0", "SSP5-8.5"] },
    "confidence": { "enum": ["LOW", "MEDIUM", "HIGH", "VERY_HIGH"] },
    "impactAssessment": { "$ref": "#/$defs/impact" }
  }
}
```

---

## 3. 기후 시나리오 (Climate Scenarios)

### 3.1 SSP-RCP 매트릭스

```
┌─────────────────────────────────────────────────────────────┐
│                    SSP-RCP 시나리오 매트릭스                  │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│   복사강제력 (W/m²)                                          │
│   8.5 ├────────────────────────────────────┐ SSP5-8.5      │
│       │                                    │ 화석연료 의존   │
│   7.0 ├─────────────────────────┐          │ +4~5°C         │
│       │                         │ SSP3-7.0 │                │
│   4.5 ├────────────────┐        │ 지역 경쟁 │                │
│       │                │ SSP2-4.5│ +3~4°C  │                │
│   2.6 ├───────┐        │ 중간 경로 │         │                │
│       │       │ SSP1-2.6│ +2~3°C │          │                │
│   1.9 ├───────┤ 지속가능│        │          │                │
│       │ SSP1-1.9│ +1.5°C│        │          │                │
│       └───────┴────────┴────────┴──────────┘                │
│       낮음                                 높음              │
│           사회경제적 완화 도전 ──────▶                       │
│                                                             │
│   WIA-CLIMATE 기본 시나리오: SSP2-4.5 (가장 현실적)          │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 3.2 시나리오별 영향 예측

| 시나리오 | 2050 온도 | 2100 온도 | 해수면 | 극한 기상 |
|----------|----------|----------|--------|----------|
| SSP1-1.9 | +1.5°C | +1.4°C | +0.3m | +10% |
| SSP1-2.6 | +1.7°C | +1.8°C | +0.4m | +20% |
| SSP2-4.5 | +2.0°C | +2.7°C | +0.6m | +40% |
| SSP3-7.0 | +2.1°C | +3.6°C | +0.7m | +60% |
| SSP5-8.5 | +2.4°C | +4.4°C | +1.0m | +100% |

---

## 4. 적응 조치 (Adaptation Measures)

### 4.1 적응 조치 분류

```
┌─────────────────────────────────────────────────────────────┐
│                    적응 조치 분류 체계                        │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  1. 인프라 (Infrastructure)                                  │
│     ├── 홍수 방어 시설 (제방, 수문)                          │
│     ├── 해안 보호 (해벽, 사구 복원)                          │
│     ├── 배수 시스템 업그레이드                               │
│     └── 내열 건축 표준                                       │
│                                                             │
│  2. 자연기반해법 (Nature-based Solutions)                    │
│     ├── 맹그로브 복원                                        │
│     ├── 도시 녹지                                            │
│     ├── 습지 보전                                            │
│     └── 유역 관리                                            │
│                                                             │
│  3. 사회적 적응 (Social)                                     │
│     ├── 조기경보시스템                                       │
│     ├── 기후 회복력 교육                                     │
│     ├── 대피 계획                                            │
│     └── 커뮤니티 기반 적응                                   │
│                                                             │
│  4. 제도적 적응 (Institutional)                              │
│     ├── 토지 이용 계획                                       │
│     ├── 건축 규정 강화                                       │
│     ├── 보험 제도                                            │
│     └── 기후 거버넌스                                        │
│                                                             │
│  5. 기술적 적응 (Technological)                              │
│     ├── 내열 작물 품종                                       │
│     ├── 담수화 기술                                          │
│     ├── 냉방 효율화                                          │
│     └── 기후 정보 시스템                                     │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 4.2 적응 조치 효과성

| 조치 | 대상 위해요소 | 효과성 | 비용 ($/혜택인) | TRL |
|------|-------------|--------|----------------|-----|
| 맹그로브 복원 | 해안홍수, 태풍 | 60% | $50 | 8 |
| 조기경보시스템 | 다중 위해 | 65% | $5 | 9 |
| 내열 작물 | 폭염, 가뭄 | 55% | $30 | 7 |
| 해안 제방 | 해수면상승 | 80% | $5,000 | 9 |
| 도시 녹지 | 폭염, 홍수 | 50% | $200 | 9 |
| 담수화 | 가뭄 | 85% | $10,000 | 9 |

### 4.3 적응 조치 데이터 스키마

```json
{
  "$id": "https://wia.org/schemas/climate/adaptation-measure.json",
  "title": "WIA Adaptation Measure",
  "type": "object",
  "properties": {
    "measureId": { "type": "string" },
    "name": { "type": "string" },
    "description": { "type": "string" },
    "category": {
      "enum": ["INFRASTRUCTURE", "ECOSYSTEM_BASED", "SOCIAL", "INSTITUTIONAL", "TECHNOLOGICAL"]
    },
    "targetHazards": {
      "type": "array",
      "items": { "type": "string" }
    },
    "cost": {
      "type": "object",
      "properties": {
        "capital": { "$ref": "#/$defs/money" },
        "operational": { "$ref": "#/$defs/money" },
        "benefitCostRatio": { "type": "number" }
      }
    },
    "effectiveness": {
      "type": "number",
      "minimum": 0,
      "maximum": 100,
      "description": "Risk reduction percentage"
    },
    "implementationTime": {
      "type": "integer",
      "description": "Months to implement"
    },
    "lifespan": {
      "type": "integer",
      "description": "Years of effectiveness"
    },
    "cobenefits": {
      "type": "array",
      "items": { "$ref": "#/$defs/cobenefit" }
    },
    "maladaptationRisk": {
      "type": "number",
      "minimum": 0,
      "maximum": 100
    },
    "technologyReadiness": {
      "type": "integer",
      "minimum": 1,
      "maximum": 9
    },
    "scalability": {
      "enum": ["LOCAL", "REGIONAL", "NATIONAL", "GLOBAL"]
    }
  }
}
```

---

## 5. 조기경보시스템 (Early Warning System)

### 5.1 다중위해 조기경보 구조

```
┌─────────────────────────────────────────────────────────────┐
│              다중위해 조기경보시스템 (MHEWS)                  │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  1. 위험 지식 (Risk Knowledge)                               │
│     ├── 과거 재해 데이터                                     │
│     ├── 취약성 평가                                          │
│     └── 위험 지도                                            │
│          │                                                  │
│          ▼                                                  │
│  2. 모니터링 & 예보 (Detection & Forecasting)                │
│     ┌─────────────────────────────────────────────────┐    │
│     │  ┌─────┐  ┌─────┐  ┌─────┐  ┌─────┐           │    │
│     │  │위성  │  │레이더│  │센서  │  │수문  │           │    │
│     │  └──┬──┘  └──┬──┘  └──┬──┘  └──┬──┘           │    │
│     │     └────────┴────────┴────────┘               │    │
│     │                  │                              │    │
│     │          ┌───────▼───────┐                     │    │
│     │          │   AI 예보    │                      │    │
│     │          │   시스템     │                      │    │
│     │          └───────────────┘                     │    │
│     └─────────────────────────────────────────────────┘    │
│          │                                                  │
│          ▼                                                  │
│  3. 경보 전파 (Warning Dissemination)                        │
│     ┌─────────────────────────────────────────────────┐    │
│     │  ┌─────┐  ┌─────┐  ┌─────┐  ┌─────┐           │    │
│     │  │SMS  │  │사이렌│  │방송  │  │앱   │           │    │
│     │  └─────┘  └─────┘  └─────┘  └─────┘           │    │
│     └─────────────────────────────────────────────────┘    │
│          │                                                  │
│          ▼                                                  │
│  4. 대응 역량 (Response Capability)                          │
│     ├── 대피 계획                                            │
│     ├── 대피소 운영                                          │
│     ├── 응급 대응                                            │
│     └── 복구 절차                                            │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 5.2 경보 수준

| 수준 | 색상 | 의미 | 조치 |
|------|------|------|------|
| WATCH | 🟡 황색 | 위험 가능성 | 상황 주시, 준비 |
| WARNING | 🟠 주황 | 위험 예상 | 대비 행동, 대피 준비 |
| EMERGENCY | 🔴 적색 | 위험 임박/발생 | 즉시 대피, 생명 보호 |

### 5.3 조기경보 API

```typescript
interface EarlyWarningAPI {
  // 경보 시스템 생성
  createSystem(
    hazard: ClimateHazard,
    coverage: GeoLocation[],
    channels: string[]
  ): EarlyWarningSystem;

  // 경보 발령
  issueAlert(
    systemId: string,
    severity: 'WATCH' | 'WARNING' | 'EMERGENCY',
    details: string,
    duration: number
  ): Alert;

  // 경보 해제
  cancelAlert(alertId: string, reason: string): void;

  // 경보 이력 조회
  getAlertHistory(
    systemId: string,
    period: DateRange
  ): Alert[];

  // 실시간 모니터링
  subscribeAlerts(
    location: GeoLocation,
    radius: number,
    callback: (alert: Alert) => void
  ): Subscription;
}
```

---

## 6. 기후 이주 (Climate Migration)

### 6.1 기후 이주 예측 모델

```
┌─────────────────────────────────────────────────────────────┐
│                    기후 이주 예측 모델                        │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  입력 변수                                                   │
│  ├── 기후 시나리오 (SSP)                                     │
│  ├── 위해요소 (해수면, 가뭄, 홍수)                           │
│  ├── 인구 분포                                               │
│  ├── 사회경제적 요인                                         │
│  └── 적응 역량                                               │
│                                                             │
│  모델 구조                                                   │
│  ┌─────────────────────────────────────────────────────┐   │
│  │                                                     │   │
│  │   기후 영향 → 생계 영향 → 이주 결정 → 목적지 선택   │   │
│  │       │          │           │           │          │   │
│  │       ▼          ▼           ▼           ▼          │   │
│  │   ┌─────┐    ┌─────┐    ┌─────┐    ┌─────┐        │   │
│  │   │물리 │    │경제 │    │행동 │    │공간 │        │   │
│  │   │모델 │    │모델 │    │모델 │    │모델 │        │   │
│  │   └─────┘    └─────┘    └─────┘    └─────┘        │   │
│  │                                                     │   │
│  └─────────────────────────────────────────────────────┘   │
│                                                             │
│  예측 결과 (2050년)                                          │
│  ├── SSP1-2.6: 1.4억 명 국내 이주                           │
│  ├── SSP2-4.5: 2.2억 명 국내 이주                           │
│  └── SSP5-8.5: 3.5억 명 국내 이주                           │
│                                                             │
│  고위험 지역                                                 │
│  ├── 방글라데시 해안                                         │
│  ├── 사하라 이남 아프리카                                    │
│  ├── 동남아시아 삼각주                                       │
│  └── 태평양 도서국                                           │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 6.2 취약 지역 지원

```typescript
interface ClimateRefugeeSupport {
  // 이주 예측
  projectDisplacement(
    region: string,
    risks: ClimateRisk[],
    population: number,
    scenario: ClimateScenario,
    timeHorizon: TimeHorizon
  ): ClimateRefugee;

  // 목적지 식별
  identifyDestinations(
    originRegion: string,
    drivers: ClimateHazard[]
  ): string[];

  // 지원 필요 평가
  assessSupportNeeds(
    displacement: ClimateRefugee
  ): SupportNeeds;

  // 조기경보 연계
  linkToEarlyWarning(
    region: string,
    threshold: number
  ): void;
}

interface SupportNeeds {
  immediate: string[];      // 긴급 지원
  shortTerm: string[];      // 단기 지원 (1년)
  longTerm: string[];       // 장기 지원 (5년+)
  estimatedCost: Money;
  internationalAssistance: boolean;
}
```

---

## 7. 적응 금융 (Adaptation Finance)

### 7.1 적응 금융 흐름

```
┌─────────────────────────────────────────────────────────────┐
│                    적응 금융 흐름                            │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  재원                                                        │
│  ├── 공공 (약 $300억/년)                                    │
│  │   ├── 녹색기후기금 (GCF)                                 │
│  │   ├── 적응기금 (AF)                                      │
│  │   ├── GEF                                                │
│  │   └── 양자 ODA                                           │
│  │                                                          │
│  └── 민간 (성장 필요)                                        │
│      ├── 기후 채권                                           │
│      ├── 보험                                                │
│      ├── ESG 투자                                            │
│      └── 블렌디드 파이낸스                                   │
│                                                             │
│  격차 분석                                                   │
│  ┌─────────────────────────────────────────────────────┐   │
│  │                                                     │   │
│  │  필요: $1,400-3,000억/년 (2030)                     │   │
│  │  현재: ~$300억/년                                   │   │
│  │  격차: $1,100-2,700억/년                            │   │
│  │                                                     │   │
│  │  ████████████████████████████░░░░░░░ 현재 21%       │   │
│  │                                                     │   │
│  └─────────────────────────────────────────────────────┘   │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 7.2 적응 프로젝트 평가

```typescript
interface AdaptationProjectEvaluation {
  // 비용-편익 분석
  conductCBA(
    project: AdaptationMeasure,
    risks: ClimateRisk[],
    discountRate: number,
    timeHorizon: number
  ): CBAResult;

  // 공동편익 평가
  assessCobenefits(
    project: AdaptationMeasure
  ): CoBenefit[];

  // 부적응 위험 평가
  assessMaladaptation(
    project: AdaptationMeasure
  ): MaladaptationRisk;

  // 확장성 평가
  assessScalability(
    project: AdaptationMeasure,
    targetRegions: string[]
  ): ScalabilityAssessment;
}

interface CBAResult {
  netPresentValue: Money;
  benefitCostRatio: number;
  internalRateOfReturn: number;
  paybackPeriod: number;
  avoidedLosses: Money;
  uncertainty: number;
}
```

---

## 8. 구현 가이드 (Implementation Guide)

### 8.1 리스크 평가 예시

```typescript
import { ClimateRiskAssessment, AdaptationPlanner } from '@wia/climate';

// 1. 리스크 평가 초기화
const riskAssessment = new ClimateRiskAssessment();

// 2. 서울 지역 홍수 리스크 평가
const seoulFloodRisk = riskAssessment.assessRisk(
  { latitude: 37.5665, longitude: 126.9780 },  // 서울
  'FLOOD_RIVERINE',
  'SSP2-4.5',
  '2050',
  {
    population: 10000000,
    assets: { amount: 500000000000, currency: 'USD' },
    infrastructure: [
      { type: 'TRANSPORTATION', quantity: 500, value: { amount: 50000000000, currency: 'USD' }, criticality: 'HIGH' }
    ],
    ecosystems: []
  },
  {
    overall: 45,
    sensitivity: 50,
    adaptiveCapacity: 60,
    socialVulnerability: 30,
    economicVulnerability: 40,
    environmentalVulnerability: 50
  }
);

console.log(`Risk Score: ${seoulFloodRisk.riskScore}/100`);
console.log(`Confidence: ${seoulFloodRisk.confidence}`);
console.log(`Economic Loss: $${seoulFloodRisk.impactAssessment.economicLoss.amount.toLocaleString()}`);

// 3. 시나리오 비교
const comparison = riskAssessment.compareScenarios(
  { latitude: 37.5665, longitude: 126.9780 },
  'FLOOD_RIVERINE',
  '2050',
  exposure,
  vulnerability
);

console.log(`Risk Range: ${comparison.riskRange.min} - ${comparison.riskRange.max}`);
```

### 8.2 적응 계획 수립

```typescript
// 4. 적응 계획 수립
const planner = new AdaptationPlanner();

// 추천 적응 조치
const recommendations = planner.getRecommendations(
  seoulFloodRisk,
  { amount: 100000000, currency: 'USD' }
);

console.log('Recommended adaptations:');
recommendations.forEach(rec => {
  console.log(`- ${rec.measure.name}: Priority ${rec.priority.toFixed(1)}`);
  console.log(`  Effectiveness: ${rec.measure.effectiveness}%`);
  console.log(`  Cost: $${rec.measure.cost.total.amount.toLocaleString()}`);
});

// 종합 적응 계획
const plan = planner.createPlan(
  [seoulFloodRisk],
  { amount: 500000000, currency: 'USD' },
  60  // 5년
);

console.log(`Plan includes ${plan.selectedMeasures.length} measures`);
console.log(`Expected risk reduction: ${plan.expectedRiskReduction.toFixed(1)}%`);
console.log(`Co-benefits: ${plan.cobenefits.map(c => c.type).join(', ')}`);
```

### 8.3 조기경보 시스템

```typescript
import { EarlyWarningService } from '@wia/climate';

// 5. 조기경보 시스템 구축
const ews = new EarlyWarningService();

const system = ews.createSystem(
  'FLOOD_RIVERINE',
  [{ latitude: 37.5665, longitude: 126.9780 }],
  ['SMS', 'App notification', 'Sirens', 'Radio']
);

console.log(`Lead time: ${system.leadTime} hours`);
console.log(`Accuracy: ${system.accuracy}%`);

// 경보 발령
const alert = ews.generateAlert(
  system,
  'WARNING',
  'Heavy rainfall expected in next 24 hours. Flooding possible in Han River basin.'
);

console.log(`Alert issued: ${alert.severity}`);
console.log(`Actions: ${alert.actions.join(', ')}`);
```

---

## 9. 연관 표준

- WIA-CLIMATE-MRV (Phase 1)
- WIA-CLIMATE-CARBON-MARKET (Phase 2)
- WIA-CLIMATE-FINANCE (Phase 4)
- IPCC AR6 WGII (Impacts, Adaptation, Vulnerability)
- UNFCCC National Adaptation Programmes of Action
- Sendai Framework for Disaster Risk Reduction
- ISO 14090:2019 Adaptation to Climate Change

---

**弘益人間 (홍익인간)** - 기후 적응으로 가장 취약한 이들을 보호합니다.

© 2025 WIA (World Certification Industry Association)
