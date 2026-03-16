# WIA-CLIMATE Phase 4: Finance (기후 금융)
# Climate Finance Integration Standard

**Version**: 1.0.0
**Status**: Draft
**Last Updated**: 2025-01-01

---

## 1. 개요 (Overview)

### 1.1 목적

WIA-CLIMATE-FINANCE 표준은 기후 금융 생태계의 표준화와 투명성을 위한 통합 프레임워크를 제공합니다.

**핵심 목표**:
- 녹색채권 분류체계 (Taxonomy) 통합
- TCFD 보고 표준 구현
- ESG 데이터 상호운용성
- 좌초자산 평가 표준

### 1.2 기후 금융 현황

```
┌─────────────────────────────────────────────────────────────┐
│                    글로벌 기후 금융 현황                      │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  녹색채권 시장 (2024)                                        │
│  ├── 누적 발행: $2.5조                                       │
│  ├── 연간 발행: $5,000억                                     │
│  └── 성장률: 연 20%+                                         │
│                                                             │
│  ESG 자산                                                    │
│  ├── 글로벌 ESG 자산: $40조+                                 │
│  ├── ESG ETF: $5,000억                                       │
│  └── 지속가능 펀드: 5,000개+                                 │
│                                                             │
│  기후 금융 격차                                               │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  필요: $4-6조/년 (Net Zero by 2050)                  │   │
│  │  현재: $1.3조/년                                     │   │
│  │  격차: $3-5조/년                                     │   │
│  │                                                     │   │
│  │  ██████░░░░░░░░░░░░░░░░░░░░░░░░░ 현재 25%           │   │
│  └─────────────────────────────────────────────────────┘   │
│                                                             │
│  주요 과제                                                   │
│  ├── 그린워싱: 녹색 정의 불일치                              │
│  ├── 데이터: ESG 데이터 품질 및 비교가능성                   │
│  ├── 분류체계: 국가별 상이한 기준                            │
│  └── 공시: TCFD/ISSB 이행 격차                              │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. 녹색채권 표준 (Green Bond Standard)

### 2.1 분류체계 (Taxonomy) 비교

| 분류체계 | 지역 | 목표 수 | 특징 |
|----------|------|--------|------|
| EU Taxonomy | EU | 6 | 가장 상세, Do No Significant Harm |
| CBI Taxonomy | 글로벌 | - | 과학 기반, 섹터별 기준 |
| 한국 녹색분류체계 | 한국 | 6 | EU와 유사, K-택소노미 |
| 중국 녹색채권목록 | 중국 | 6 | 전환 포함, 청정석탄 제외 (2021) |
| ASEAN Taxonomy | 동남아 | 4 | 전환 활동 중심 |

### 2.2 녹색채권 스키마

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://wia.org/schemas/climate/green-bond.json",
  "title": "WIA Green Bond",
  "type": "object",
  "required": ["isin", "issuer", "taxonomy", "useOfProceeds"],
  "properties": {
    "isin": {
      "type": "string",
      "pattern": "^[A-Z]{2}[A-Z0-9]{9}[0-9]$"
    },
    "issuer": { "$ref": "#/$defs/organization" },
    "issueDate": { "type": "string", "format": "date" },
    "maturityDate": { "type": "string", "format": "date" },
    "couponRate": { "type": "number" },
    "principal": { "$ref": "#/$defs/money" },
    "useOfProceeds": {
      "type": "array",
      "items": {
        "enum": [
          "RENEWABLE_ENERGY",
          "ENERGY_EFFICIENCY",
          "GREEN_BUILDINGS",
          "CLEAN_TRANSPORTATION",
          "SUSTAINABLE_WATER",
          "POLLUTION_PREVENTION",
          "BIODIVERSITY",
          "CIRCULAR_ECONOMY"
        ]
      }
    },
    "taxonomy": {
      "enum": ["EU_TAXONOMY", "CBI", "ASEAN_TAXONOMY", "CHINA_TAXONOMY", "KOREA_TAXONOMY", "ICMA_GBP"]
    },
    "impactMetrics": {
      "type": "array",
      "items": { "$ref": "#/$defs/impactMetric" }
    },
    "tcfdAligned": { "type": "boolean" },
    "secondPartyOpinion": { "$ref": "#/$defs/spo" },
    "externalReview": { "$ref": "#/$defs/review" }
  }
}
```

### 2.3 EU Taxonomy 정렬 검사

```
┌─────────────────────────────────────────────────────────────┐
│                EU Taxonomy 정렬 프로세스                     │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  Step 1: 환경 목표 기여 확인                                 │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  6대 환경 목표                                       │   │
│  │  1. 기후변화 완화 ✓                                  │   │
│  │  2. 기후변화 적응 ✓                                  │   │
│  │  3. 물 및 해양자원                                    │   │
│  │  4. 순환경제                                          │   │
│  │  5. 오염 방지                                         │   │
│  │  6. 생물다양성                                        │   │
│  └─────────────────────────────────────────────────────┘   │
│                           │                                 │
│                           ▼                                 │
│  Step 2: DNSH (Do No Significant Harm) 검증                 │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  각 목표에 심각한 해를 끼치지 않음 확인               │   │
│  │  ├── 기후 완화 활동 → 적응에 해 없음 ✓               │   │
│  │  ├── 수자원에 해 없음 ✓                              │   │
│  │  ├── 순환경제에 해 없음 ✓                            │   │
│  │  └── ...                                             │   │
│  └─────────────────────────────────────────────────────┘   │
│                           │                                 │
│                           ▼                                 │
│  Step 3: 최소 사회적 안전장치 (Minimum Safeguards)          │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  ├── OECD 다국적기업 가이드라인 준수                  │   │
│  │  ├── UN 기업과 인권 이행지침                         │   │
│  │  ├── ILO 핵심 협약                                   │   │
│  │  └── 국제인권장전                                     │   │
│  └─────────────────────────────────────────────────────┘   │
│                           │                                 │
│                           ▼                                 │
│  Step 4: 기술 심사 기준 (Technical Screening Criteria)      │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  섹터별 정량 기준 충족                                │   │
│  │  예: 발전 → 100g CO2e/kWh 미만                       │   │
│  │  예: 건물 → NZEB 기준 충족                           │   │
│  └─────────────────────────────────────────────────────┘   │
│                           │                                 │
│                           ▼                                 │
│                    ✅ Taxonomy Aligned                      │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 3. TCFD 표준 (TCFD Standard)

### 3.1 TCFD 4대 축

```
┌─────────────────────────────────────────────────────────────┐
│                    TCFD 권고안 구조                          │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌───────────────────────────────────────────────────┐     │
│  │                    GOVERNANCE                      │     │
│  │               이사회/경영진 감독                    │     │
│  └───────────────────────────────────────────────────┘     │
│                          │                                  │
│           ┌──────────────┼──────────────┐                  │
│           ▼              ▼              ▼                  │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐          │
│  │  STRATEGY   │ │    RISK     │ │   METRICS   │          │
│  │             │ │ MANAGEMENT  │ │  & TARGETS  │          │
│  │  전략적     │ │             │ │             │          │
│  │  영향 분석  │ │  리스크     │ │  배출량     │          │
│  │             │ │  식별/관리  │ │  목표/KPI   │          │
│  └─────────────┘ └─────────────┘ └─────────────┘          │
│                                                             │
│  각 축별 권고 사항                                           │
│  ┌─────────────────────────────────────────────────────┐   │
│  │ Governance                                          │   │
│  │ a) 이사회의 기후 리스크/기회 감독                     │   │
│  │ b) 경영진의 기후 리스크/기회 평가 및 관리 역할        │   │
│  ├─────────────────────────────────────────────────────┤   │
│  │ Strategy                                            │   │
│  │ a) 단기/중기/장기 기후 리스크 및 기회                 │   │
│  │ b) 조직의 사업, 전략, 재무계획에 미치는 영향          │   │
│  │ c) 시나리오 분석 포함 전략의 회복력                   │   │
│  ├─────────────────────────────────────────────────────┤   │
│  │ Risk Management                                     │   │
│  │ a) 기후 리스크 식별 및 평가 프로세스                  │   │
│  │ b) 기후 리스크 관리 프로세스                         │   │
│  │ c) 전사 리스크 관리와의 통합                         │   │
│  ├─────────────────────────────────────────────────────┤   │
│  │ Metrics & Targets                                   │   │
│  │ a) 기후 리스크/기회 평가에 사용되는 지표              │   │
│  │ b) Scope 1, 2, 3 온실가스 배출량                     │   │
│  │ c) 기후 리스크/기회 관리 목표 및 성과                 │   │
│  └─────────────────────────────────────────────────────┘   │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 3.2 TCFD 준수 수준

| 수준 | 설명 | 요건 | 점수 |
|------|------|------|------|
| NOT_ALIGNED | 미준수 | TCFD 공시 없음 | 0-40 |
| PARTIAL | 부분 준수 | 일부 권고 이행 | 40-60 |
| SUBSTANTIAL | 상당 준수 | 대부분 권고 이행 | 60-80 |
| FULL | 완전 준수 | 모든 권고 + 시나리오 분석 | 80-100 |

### 3.3 TCFD 스키마

```json
{
  "$id": "https://wia.org/schemas/climate/tcfd-disclosure.json",
  "title": "WIA TCFD Disclosure",
  "type": "object",
  "properties": {
    "organizationId": { "type": "string" },
    "reportingYear": { "type": "integer" },
    "governance": {
      "type": "object",
      "properties": {
        "boardOversight": { "type": "string" },
        "managementRole": { "type": "string" },
        "committeesResponsible": { "type": "array", "items": { "type": "string" } },
        "reportingFrequency": { "type": "string" },
        "climateExpertise": { "type": "boolean" }
      }
    },
    "strategy": {
      "type": "object",
      "properties": {
        "climateRisksIdentified": { "type": "array" },
        "climateOpportunities": { "type": "array" },
        "businessImpact": { "type": "string" },
        "resilienceStrategy": { "type": "string" },
        "transitionPlan": { "type": "string" }
      }
    },
    "riskManagement": {
      "type": "object",
      "properties": {
        "identificationProcess": { "type": "string" },
        "assessmentProcess": { "type": "string" },
        "managementProcess": { "type": "string" },
        "integrationWithERM": { "type": "boolean" }
      }
    },
    "metricsTargets": {
      "type": "object",
      "properties": {
        "scope1Emissions": { "type": "number" },
        "scope2Emissions": { "type": "number" },
        "scope3Emissions": { "type": "number" },
        "emissionsIntensity": { "type": "number" },
        "targets": { "type": "array", "items": { "$ref": "#/$defs/climateTarget" } }
      }
    },
    "scenarioAnalysis": {
      "type": "array",
      "items": { "$ref": "#/$defs/scenarioResult" }
    }
  }
}
```

---

## 4. ESG 데이터 표준 (ESG Data Standard)

### 4.1 ESG 프레임워크 통합

```
┌─────────────────────────────────────────────────────────────┐
│                    ESG 프레임워크 매핑                       │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  WIA-CLIMATE ESG ──────┬─────────────────────────────────   │
│                        │                                    │
│           ┌────────────┼────────────┐                      │
│           ▼            ▼            ▼                      │
│      ┌─────────┐ ┌─────────┐ ┌─────────┐                  │
│      │  ISSB   │ │   GRI   │ │  SASB   │                  │
│      │ S1/S2   │ │Standards│ │Standards│                  │
│      └─────────┘ └─────────┘ └─────────┘                  │
│           │            │            │                      │
│           └────────────┴────────────┘                      │
│                        │                                    │
│                        ▼                                    │
│      ┌─────────────────────────────────────────────────┐   │
│      │              통합 ESG 데이터 모델               │   │
│      │                                                 │   │
│      │  Environmental  │  Social      │  Governance    │   │
│      │  ──────────────│─────────────│──────────────   │   │
│      │  GHG 배출      │  다양성     │  이사회 독립성  │   │
│      │  에너지 사용   │  안전      │  윤리 경영     │   │
│      │  물 사용       │  인권      │  보상 정책     │   │
│      │  폐기물       │  지역사회   │  리스크 관리   │   │
│      └─────────────────────────────────────────────────┘   │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 4.2 ESG 등급 체계

| 등급 | 점수 | 설명 |
|------|------|------|
| AAA | 80+ | 업계 최고, 모범 사례 |
| AA | 70-79 | 우수, 대부분 기준 충족 |
| A | 60-69 | 평균 이상 |
| BBB | 50-59 | 평균 |
| BB | 40-49 | 평균 이하 |
| B | 30-39 | 미흡 |
| CCC | <30 | 심각한 개선 필요 |

### 4.3 ESG 데이터 API

```typescript
interface ESGDataAPI {
  // ESG 점수 계산
  calculateScore(data: ESGData): ESGScoreResult;

  // 벤치마크 비교
  benchmark(
    target: ESGData,
    peers: ESGData[]
  ): ESGBenchmark;

  // 시계열 분석
  analyzeTrend(
    organizationId: string,
    periods: DateRange[]
  ): ESGTrend;

  // 데이터 품질 평가
  assessDataQuality(data: ESGData): DataQualityScore;

  // 표준 변환
  convertToGRI(data: ESGData): GRIReport;
  convertToSASB(data: ESGData): SASBReport;
  convertToISSB(data: ESGData): ISSBReport;
}
```

---

## 5. 전환 금융 (Transition Finance)

### 5.1 전환 금융 정의

```
┌─────────────────────────────────────────────────────────────┐
│                    전환 금융 스펙트럼                         │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  녹색 ◄─────────────────────────────────────────► 갈색     │
│                                                             │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐       │
│  │ 순수    │  │ 전환    │  │ 지속가능│  │ 일반    │       │
│  │ 녹색    │  │ 금융    │  │ 연계    │  │ 금융    │       │
│  │         │  │         │  │ (SLB)   │  │         │       │
│  └─────────┘  └─────────┘  └─────────┘  └─────────┘       │
│       │            │            │            │              │
│       ▼            ▼            ▼            ▼              │
│  재생에너지    저탄소 철강   배출 감축     기존 사업        │
│  녹색건물      청정 시멘트   목표 연계                      │
│  친환경 운송   가스 전환     KPI 달성시                     │
│                              금리 혜택                      │
│                                                             │
│  전환 금융의 필요성                                          │
│  ├── 고탄소 산업의 탈탄소화 지원                            │
│  ├── 갈색 → 녹색 전환 경로 제공                            │
│  ├── 좌초자산 위험 관리                                     │
│  └── 공정 전환 (Just Transition) 지원                      │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 5.2 전환 계획 요건

```typescript
interface TransitionPlan {
  baselineYear: number;
  netZeroYear: number;          // 2050 이전 권장
  pathway: TransitionPathway[]; // 연도별 경로
  capitalExpenditure: Money;    // 전환 투자
  assumptions: string[];
}

interface TransitionPathway {
  year: number;
  emissions: number;            // tCO2e
  reduction: number;            // % vs baseline
  milestones: string[];
}

// 전환 계획 검증
function validateTransitionPlan(plan: TransitionPlan): ValidationResult {
  const checks = [
    // 1. Net Zero 목표 연도
    plan.netZeroYear <= 2050,

    // 2. 과학 기반 경로
    isAlignedWithScienceBasedPathway(plan.pathway),

    // 3. 중간 목표
    hasInterimTargets(plan.pathway, [2025, 2030, 2040]),

    // 4. CapEx 투입
    plan.capitalExpenditure.amount > 0,

    // 5. 일관된 감축
    isConsistentReduction(plan.pathway)
  ];

  return {
    valid: checks.every(c => c),
    score: checks.filter(c => c).length / checks.length * 100
  };
}
```

---

## 6. 좌초자산 (Stranded Assets)

### 6.1 좌초자산 유형

```
┌─────────────────────────────────────────────────────────────┐
│                    좌초자산 리스크 분석                       │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  좌초자산 정의                                               │
│  경제적 수명 완료 전 가치 하락/상실되는 자산                  │
│                                                             │
│  유형별 위험도                                               │
│  ┌─────────────────────────────────────────────────────┐   │
│  │                                                     │   │
│  │  화석연료 매장량  ████████████████████████  90%     │   │
│  │  석탄 발전소      ███████████████████████   85%     │   │
│  │  정유시설         ██████████████████       70%     │   │
│  │  가스 파이프라인  █████████████           55%     │   │
│  │  ICE 자동차 공장  ████████████████        60%     │   │
│  │  고탄소 부동산    ████████                30%     │   │
│  │                                                     │   │
│  └─────────────────────────────────────────────────────┘   │
│                                                             │
│  시나리오별 좌초 위험                                        │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  시나리오    │  석탄  │  석유  │  가스  │  총액    │   │
│  │ ───────────│───────│───────│───────│────────── │   │
│  │  SSP1-1.9  │  100% │   80% │   60% │  $100조   │   │
│  │  SSP1-2.6  │   90% │   70% │   50% │   $80조   │   │
│  │  SSP2-4.5  │   70% │   50% │   30% │   $50조   │   │
│  │  SSP5-8.5  │   30% │   20% │   10% │   $20조   │   │
│  └─────────────────────────────────────────────────────┘   │
│                                                             │
│  완화 전략                                                   │
│  ├── 조기 퇴역 및 대체 투자                                  │
│  ├── 자산 전환 (예: 석탄→수소)                              │
│  ├── 포트폴리오 다각화                                       │
│  └── 전환 금융 활용                                          │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 6.2 좌초자산 평가 API

```typescript
interface StrandedAssetAPI {
  // 개별 자산 평가
  assessRisk(
    assetType: StrandedAssetType,
    currentValue: Money,
    scenario: ClimateScenario,
    timeHorizon: TimeHorizon
  ): StrandedAsset;

  // 포트폴리오 분석
  analyzePortfolio(assets: StrandedAsset[]): PortfolioAnalysis;

  // 시나리오 스트레스 테스트
  stressTest(
    portfolio: StrandedAsset[],
    scenarios: ClimateScenario[]
  ): StressTestResult[];

  // 완화 옵션 추천
  suggestMitigations(asset: StrandedAsset): string[];

  // 전환 경로 모델링
  modelTransition(
    asset: StrandedAsset,
    targetState: string,
    budget: Money
  ): TransitionPathway;
}

// 사용 예시
const assetService = new StrandedAssetService();

// 석탄 발전소 평가
const coalPlant = assetService.assessRisk(
  'COAL_POWER_PLANT',
  { amount: 500000000, currency: 'USD' },
  'SSP2-4.5',
  '2050'
);

console.log(`Writedown risk: ${coalPlant.writedownRisk}%`);
console.log(`Projected value: $${coalPlant.projectedValue.amount.toLocaleString()}`);
console.log(`Mitigations: ${coalPlant.mitigationOptions.join(', ')}`);
```

---

## 7. 구현 가이드 (Implementation Guide)

### 7.1 녹색채권 스크리닝

```typescript
import { GreenBondService } from '@wia/climate';

const bondService = new GreenBondService();

// 녹색채권 정보
const greenBond: GreenBond = {
  isin: 'XS1234567890',
  issuer: { id: 'ORG-001', name: 'Korea Electric Power', country: 'KR', sector: 'ENERGY' },
  issueDate: new Date('2024-01-15'),
  maturityDate: new Date('2034-01-15'),
  couponRate: 3.5,
  principal: { amount: 500000000, currency: 'USD' },
  useOfProceeds: ['RENEWABLE_ENERGY', 'ENERGY_EFFICIENCY'],
  taxonomy: 'EU_TAXONOMY',
  impactMetrics: [
    { category: 'GHG', metric: 'CO2 avoided', unit: 'tCO2e', baseline: 0, target: 100000 },
    { category: 'Energy', metric: 'Renewable capacity', unit: 'MW', baseline: 0, target: 200 }
  ],
  tcfdAligned: true,
  secondPartyOpinion: {
    name: 'Sustainalytics',
    opinion: 'POSITIVE',
    reportDate: new Date('2024-01-01'),
    reportUrl: 'https://...',
    alignment: ['EU_TAXONOMY', 'CBI']
  },
  externalReview: {
    reviewType: 'SECOND_PARTY_OPINION',
    provider: 'Sustainalytics',
    date: new Date('2024-01-01'),
    result: 'Aligned',
    reportUrl: 'https://...'
  },
  greenBondFramework: 'https://...'
};

// 1. 택소노미 스크리닝
const screening = bondService.screenBond(greenBond);
console.log(`Aligned: ${screening.aligned}`);
console.log(`Score: ${screening.overallScore}/100`);

// 2. EU Taxonomy 분석
const euAnalysis = bondService.analyzeEUTaxonomy(greenBond);
console.log(`Taxonomy eligible: ${euAnalysis.taxonomyEligibleShare * 100}%`);
console.log(`DNSH compliant: ${euAnalysis.dnshCompliant}`);

// 3. 임팩트 계산
const impact = bondService.calculateImpact(greenBond, {
  start: new Date('2024-01-01'),
  end: new Date('2024-12-31')
});
console.log(`Carbon impact: ${impact.carbonImpact.amount} tCO2e avoided`);
```

### 7.2 TCFD 평가

```typescript
import { TCFDService } from '@wia/climate';

const tcfdService = new TCFDService();

// TCFD 공시 평가
const disclosure: TCFDDisclosure = {
  organizationId: 'ORG-001',
  reportingYear: 2024,
  governance: {
    boardOversight: 'Climate committee meets quarterly...',
    managementRole: 'CSO reports to CEO on climate...',
    committeesResponsible: ['Climate Committee', 'Risk Committee'],
    reportingFrequency: 'Quarterly',
    climateExpertise: true
  },
  strategy: {
    climateRisksIdentified: ['POLICY_LEGAL', 'TECHNOLOGY', 'MARKET', 'ACUTE_PHYSICAL'],
    climateOpportunities: [
      { category: 'PRODUCTS_SERVICES', description: 'EV charging infrastructure', financialImpact: '$50M revenue', timeframe: 'MEDIUM' }
    ],
    businessImpact: 'Transition to renewable energy...',
    resilienceStrategy: 'Diversified energy portfolio...',
    transitionPlan: 'Net zero by 2040...'
  },
  riskManagement: {
    identificationProcess: 'Annual climate risk assessment...',
    assessmentProcess: 'Scenario analysis using TCFD...',
    managementProcess: 'Integration with ERM...',
    integrationWithERM: true
  },
  metricsTargets: {
    scope1Emissions: 50000,
    scope2Emissions: 30000,
    scope3Emissions: 200000,
    emissionsIntensity: 0.5,
    energyConsumption: 1000000,
    renewableEnergyShare: 40,
    waterUsage: 500000,
    targets: [
      {
        targetType: 'NET_ZERO',
        scope: ['SCOPE1', 'SCOPE2'],
        baseYear: 2020,
        targetYear: 2040,
        reduction: 100,
        sbtiValidated: true,
        progress: 25
      }
    ]
  },
  scenarioAnalysis: [
    {
      scenario: 'SSP1-2.6',
      timeHorizon: '2050',
      assumptions: ['Carbon price $150/t', '100% renewable grid'],
      physicalRiskImpact: { amount: -10000000, currency: 'USD' },
      transitionRiskImpact: { amount: -50000000, currency: 'USD' },
      opportunities: { amount: 100000000, currency: 'USD' },
      netImpact: { amount: 40000000, currency: 'USD' },
      resilienceConclusion: 'Strategy resilient under 2°C scenario'
    }
  ],
  complianceLevel: 'SUBSTANTIAL'
};

const assessment = tcfdService.assessDisclosure(disclosure);
console.log(`TCFD Compliance: ${assessment.complianceLevel}`);
console.log(`Overall Score: ${assessment.overallScore}/100`);
console.log(`Recommendations:`);
assessment.recommendations.forEach(r => console.log(`  - ${r}`));
```

### 7.3 포트폴리오 좌초자산 분석

```typescript
import { StrandedAssetService } from '@wia/climate';

const strandedService = new StrandedAssetService();

// 포트폴리오 자산
const assets = [
  strandedService.assessRisk('COAL_POWER_PLANT', { amount: 200000000, currency: 'USD' }, 'SSP2-4.5', '2050'),
  strandedService.assessRisk('GAS_PIPELINE', { amount: 300000000, currency: 'USD' }, 'SSP2-4.5', '2050'),
  strandedService.assessRisk('OIL_REFINERY', { amount: 400000000, currency: 'USD' }, 'SSP2-4.5', '2050'),
];

// 포트폴리오 분석
const analysis = strandedService.analyzePortfolio(assets);

console.log(`Total at risk: $${analysis.totalAtRisk.amount.toLocaleString()}`);
console.log(`Risk percentage: ${analysis.atRiskPercentage.toFixed(1)}%`);
console.log(`Recommendations:`);
analysis.recommendations.forEach(r => console.log(`  - ${r}`));
```

---

## 8. ISSB 통합 (IFRS S1/S2)

### 8.1 ISSB 표준 개요

```
┌─────────────────────────────────────────────────────────────┐
│                    ISSB 표준 구조                            │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  IFRS S1: 일반 지속가능성 공시                               │
│  ├── 모든 지속가능성 관련 리스크/기회 공시                   │
│  ├── 산업 특화 지표 (SASB 기반)                             │
│  └── 중대성 기반 접근                                        │
│                                                             │
│  IFRS S2: 기후 관련 공시                                     │
│  ├── TCFD 4대 축 기반                                        │
│  ├── Scope 1, 2, 3 배출량 필수                              │
│  ├── 시나리오 분석 (1.5°C 포함)                             │
│  └── 전환 계획 공시                                          │
│                                                             │
│  WIA-CLIMATE와 ISSB 매핑                                     │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  WIA-CLIMATE          │  IFRS S2                    │   │
│  │ ─────────────────────│──────────────────────────── │   │
│  │  GHGEmission          │  Climate-related metrics    │   │
│  │  TCFDDisclosure       │  Core content               │   │
│  │  ClimateRisk          │  Climate-related risks      │   │
│  │  TransitionPlan       │  Transition plans           │   │
│  │  ScenarioAnalysis     │  Climate resilience         │   │
│  └─────────────────────────────────────────────────────┘   │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 9. 연관 표준

- WIA-CLIMATE-MRV (Phase 1)
- WIA-CLIMATE-CARBON-MARKET (Phase 2)
- WIA-CLIMATE-ADAPTATION (Phase 3)
- TCFD Recommendations (2017)
- IFRS S1/S2 (ISSB 2023)
- EU Taxonomy Regulation
- ICMA Green Bond Principles
- Climate Bonds Standard

---

**弘益人間 (홍익인간)** - 투명한 기후 금융으로 지속가능한 미래를 만듭니다.

© 2025 WIA (World Certification Industry Association)
