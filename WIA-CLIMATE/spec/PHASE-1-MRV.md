# WIA-CLIMATE Phase 1: MRV (측정-보고-검증)
# Measurement, Reporting, and Verification Standard

**Version**: 1.0.0
**Status**: Draft
**Last Updated**: 2025-01-01

---

## 1. 개요 (Overview)

### 1.1 목적

WIA-CLIMATE-MRV 표준은 온실가스 배출량의 측정, 보고, 검증을 위한 통합 프레임워크를 제공합니다.

**핵심 목표**:
- Scope 1/2/3 배출량 측정 표준화
- GHG Protocol, ISO 14064 호환성
- 블록체인 기반 투명한 검증
- AI 활용 배출량 추정

### 1.2 적용 범위

```
┌─────────────────────────────────────────────────────────────┐
│                    WIA-CLIMATE-MRV 범위                      │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  Scope 1: 직접 배출                                         │
│  ├── 고정 연소 (보일러, 발전기)                              │
│  ├── 이동 연소 (차량, 항공기)                                │
│  ├── 공정 배출 (화학반응)                                    │
│  └── 누출 배출 (냉매, 가스)                                  │
│                                                             │
│  Scope 2: 간접 배출 (에너지)                                 │
│  ├── 구매 전력                                              │
│  ├── 구매 열/스팀                                           │
│  └── 구매 냉방                                              │
│                                                             │
│  Scope 3: 가치사슬 배출                                      │
│  ├── 상류: 구매품, 운송, 출장, 통근                          │
│  └── 하류: 제품 사용, 폐기, 투자                             │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. 데이터 스키마 (Data Schema)

### 2.1 GHG Emission JSON Schema

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://wia.org/schemas/climate/ghg-emission.json",
  "title": "WIA GHG Emission Report",
  "type": "object",
  "required": ["facilityId", "reportingPeriod", "scope1", "scope2", "methodology"],
  "properties": {
    "facilityId": {
      "type": "string",
      "description": "Unique facility identifier"
    },
    "reportingPeriod": {
      "type": "object",
      "properties": {
        "start": { "type": "string", "format": "date" },
        "end": { "type": "string", "format": "date" }
      },
      "required": ["start", "end"]
    },
    "scope1": { "$ref": "#/$defs/emissionData" },
    "scope2": { "$ref": "#/$defs/emissionData" },
    "scope3": { "$ref": "#/$defs/emissionData" },
    "methodology": {
      "type": "string",
      "enum": ["GHG_PROTOCOL", "ISO_14064", "ISO_14067", "EPA_GHG", "IPCC_GUIDELINES", "EU_ETS_MRV"]
    },
    "verificationStatus": { "$ref": "#/$defs/verification" },
    "uncertainty": {
      "type": "number",
      "minimum": 0,
      "maximum": 100,
      "description": "Uncertainty percentage"
    }
  },
  "$defs": {
    "emissionData": {
      "type": "object",
      "properties": {
        "total": { "type": "number", "description": "Total tCO2e" },
        "breakdown": {
          "type": "array",
          "items": { "$ref": "#/$defs/emissionBreakdown" }
        },
        "dataQuality": { "$ref": "#/$defs/dataQuality" },
        "calculationMethod": {
          "type": "string",
          "enum": ["DIRECT_MEASUREMENT", "MASS_BALANCE", "EMISSION_FACTORS", "ENGINEERING_ESTIMATES", "HYBRID"]
        }
      },
      "required": ["total", "breakdown"]
    },
    "emissionBreakdown": {
      "type": "object",
      "properties": {
        "category": { "type": "string" },
        "source": { "type": "string" },
        "gasType": {
          "type": "string",
          "enum": ["CO2", "CH4", "N2O", "HFCs", "PFCs", "SF6", "NF3"]
        },
        "quantity": { "type": "number" },
        "activityData": { "type": "number" },
        "emissionFactor": { "type": "number" },
        "gwp": { "type": "number" }
      },
      "required": ["category", "gasType", "quantity"]
    },
    "dataQuality": {
      "type": "object",
      "properties": {
        "overall": { "type": "number", "minimum": 0, "maximum": 100 },
        "completeness": { "type": "number" },
        "consistency": { "type": "number" },
        "accuracy": { "type": "number" },
        "timeliness": { "type": "number" },
        "transparency": { "type": "number" }
      }
    },
    "verification": {
      "type": "object",
      "properties": {
        "level": {
          "type": "string",
          "enum": ["SELF_REPORTED", "INTERNAL_AUDIT", "THIRD_PARTY", "ACCREDITED", "BLOCKCHAIN_VERIFIED"]
        },
        "verifier": { "type": "object" },
        "timestamp": { "type": "string", "format": "date-time" },
        "hash": { "type": "string" }
      },
      "required": ["level", "timestamp"]
    }
  }
}
```

---

## 3. 배출 계수 (Emission Factors)

### 3.1 에너지 배출 계수

| 연료 유형 | 단위 | CO2 (kg) | CH4 (g) | N2O (g) | 출처 |
|-----------|------|----------|---------|---------|------|
| 천연가스 | m³ | 2.02 | 40 | 1 | IPCC 2019 |
| 경유 | L | 2.68 | 1 | 1 | IPCC 2019 |
| 휘발유 | L | 2.31 | 1 | 1 | IPCC 2019 |
| 석탄 | kg | 2.42 | 30 | 2 | IPCC 2019 |
| LPG | kg | 2.98 | 1 | 1 | IPCC 2019 |

### 3.2 전력 계통 배출 계수 (2023)

| 국가/지역 | 계수 (kg CO2e/kWh) | 출처 |
|-----------|-------------------|------|
| 글로벌 평균 | 0.475 | IEA 2023 |
| 대한민국 | 0.459 | 환경부 2023 |
| 미국 | 0.417 | EPA 2023 |
| EU | 0.276 | EEA 2023 |
| 중국 | 0.581 | CEC 2023 |
| 일본 | 0.468 | MOE 2023 |

### 3.3 GWP (지구온난화지수)

| 가스 | AR5 100년 GWP | AR6 100년 GWP |
|------|---------------|---------------|
| CO2 | 1 | 1 |
| CH4 | 28 | 27.9 |
| N2O | 265 | 273 |
| SF6 | 23,500 | 25,200 |
| NF3 | 16,100 | 17,400 |

---

## 4. 측정 방법론 (Measurement Methodology)

### 4.1 직접 측정 (Tier 3)

```
┌─────────────────────────────────────────────────────────────┐
│                    직접 측정 시스템                          │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│   연속 배출 모니터링 시스템 (CEMS)                           │
│   ┌─────────┐    ┌─────────┐    ┌─────────┐                │
│   │ 가스     │───▶│ 분석기   │───▶│ 데이터   │               │
│   │ 샘플링   │    │ CO2/CH4 │    │ 로거    │                │
│   └─────────┘    └─────────┘    └─────────┘                │
│        │              │              │                      │
│        ▼              ▼              ▼                      │
│   ┌─────────┐    ┌─────────┐    ┌─────────┐                │
│   │ 유량계   │    │ 온도/압력 │    │ WIA MRV │               │
│   │         │    │ 보정    │    │ API     │                │
│   └─────────┘    └─────────┘    └─────────┘                │
│                                                             │
│   정확도: ±2%                                               │
│   불확실성: <5%                                             │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 4.2 활동 데이터 기반 (Tier 2)

```typescript
// 배출량 = 활동 데이터 × 배출 계수 × GWP
emission = activityData * emissionFactor * gwp;

// 예시: 천연가스 연소
const naturalGasEmission = {
  activityData: 10000,           // m³
  emissionFactor: 2.02,          // kg CO2/m³
  gwp: 1,                        // CO2
  result: 10000 * 2.02 * 1       // = 20,200 kg CO2e
};
```

### 4.3 AI 기반 추정 (Tier 1+)

```
┌─────────────────────────────────────────────────────────────┐
│                    AI 배출량 추정 시스템                     │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│   입력 데이터                                                │
│   ├── 위성 이미지 (열적외선, 메탄 감지)                       │
│   ├── IoT 센서 데이터                                        │
│   ├── 산업 활동 지표                                         │
│   └── 기상 데이터                                            │
│                                                             │
│   AI 모델                                                    │
│   ┌─────────────────────────────────────────────────────┐   │
│   │  Satellite ML     NLP Scope3     Anomaly Detection │   │
│   │  ┌─────┐          ┌─────┐        ┌─────┐          │   │
│   │  │ CNN │          │ LLM │        │ VAE │          │   │
│   │  └─────┘          └─────┘        └─────┘          │   │
│   │     │                │               │             │   │
│   │     └────────────────┼───────────────┘             │   │
│   │                      ▼                             │   │
│   │              ┌───────────┐                         │   │
│   │              │ Ensemble  │                         │   │
│   │              └───────────┘                         │   │
│   └─────────────────────────────────────────────────────┘   │
│                                                             │
│   출력: 배출량 추정치 + 불확실성 + 신뢰도                     │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 5. 보고 표준 (Reporting Standard)

### 5.1 보고 주기

| 보고 유형 | 주기 | 제출 기한 | 대상 |
|-----------|------|-----------|------|
| 월간 보고 | 매월 | 익월 15일 | 고배출 시설 |
| 분기 보고 | 분기 | 분기 종료 후 30일 | 중간 시설 |
| 연간 보고 | 연간 | 익년 3월 31일 | 모든 시설 |
| 실시간 | 연속 | 실시간 | CEMS 보유 시설 |

### 5.2 보고 양식

```
┌─────────────────────────────────────────────────────────────┐
│                WIA-CLIMATE GHG 보고서                        │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  1. 조직 정보                                                │
│     ├── 법인명                                               │
│     ├── 시설 정보                                            │
│     └── 산업 분류 (ISIC/NAICS)                               │
│                                                             │
│  2. 보고 기간                                                │
│     ├── 시작일                                               │
│     ├── 종료일                                               │
│     └── 보고 유형                                            │
│                                                             │
│  3. 배출량 요약                                              │
│     ├── Scope 1: ______ tCO2e                               │
│     ├── Scope 2: ______ tCO2e                               │
│     ├── Scope 3: ______ tCO2e                               │
│     └── 총 배출량: ______ tCO2e                              │
│                                                             │
│  4. 배출량 상세                                              │
│     ├── 카테고리별 분류                                       │
│     ├── 온실가스별 분류                                       │
│     └── 시설별 분류                                          │
│                                                             │
│  5. 데이터 품질                                              │
│     ├── 산정 방법론                                          │
│     ├── 불확실성                                             │
│     └── 데이터 소스                                          │
│                                                             │
│  6. 검증 정보                                                │
│     ├── 검증 수준                                            │
│     ├── 검증 기관                                            │
│     └── 검증 결과                                            │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 6. 검증 프레임워크 (Verification Framework)

### 6.1 검증 수준

| 수준 | 설명 | 요건 | 신뢰도 |
|------|------|------|--------|
| Level 1 | 자체 보고 | 내부 검토 | 60% |
| Level 2 | 내부 감사 | 독립적 내부 팀 | 75% |
| Level 3 | 제3자 검증 | 외부 검증기관 | 90% |
| Level 4 | 인증 검증 | 인증 기관 | 95% |
| Level 5 | 블록체인 검증 | 분산 합의 | 99% |

### 6.2 블록체인 MRV

```
┌─────────────────────────────────────────────────────────────┐
│                  블록체인 MRV 아키텍처                       │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│   데이터 소스                                                │
│   ┌─────┐  ┌─────┐  ┌─────┐  ┌─────┐                       │
│   │CEMS │  │위성  │  │IoT  │  │수동  │                       │
│   └──┬──┘  └──┬──┘  └──┬──┘  └──┬──┘                       │
│      │        │        │        │                          │
│      └────────┴────────┴────────┘                          │
│                   │                                         │
│                   ▼                                         │
│   ┌─────────────────────────────────────────────────────┐  │
│   │              오라클 레이어 (Oracle Layer)            │  │
│   │  ┌─────────┐  ┌─────────┐  ┌─────────┐             │  │
│   │  │Chainlink│  │ API3    │  │ Band    │             │  │
│   │  └─────────┘  └─────────┘  └─────────┘             │  │
│   └─────────────────────────────────────────────────────┘  │
│                   │                                         │
│                   ▼                                         │
│   ┌─────────────────────────────────────────────────────┐  │
│   │              스마트 컨트랙트 (Smart Contract)        │  │
│   │                                                     │  │
│   │  function verifyEmission(                           │  │
│   │    bytes32 dataHash,                               │  │
│   │    uint256 emission,                               │  │
│   │    address[] oracles                               │  │
│   │  ) external returns (bool) {                       │  │
│   │    require(oracleConsensus(oracles, dataHash));    │  │
│   │    emit EmissionVerified(msg.sender, emission);    │  │
│   │    return true;                                    │  │
│   │  }                                                 │  │
│   │                                                     │  │
│   └─────────────────────────────────────────────────────┘  │
│                   │                                         │
│                   ▼                                         │
│   ┌─────────────────────────────────────────────────────┐  │
│   │              블록체인 네트워크                        │  │
│   │  ┌─────────┐  ┌─────────┐  ┌─────────┐             │  │
│   │  │Polygon  │  │Hedera   │  │Celo     │             │  │
│   │  │PoS      │  │Hashgraph│  │PoS      │             │  │
│   │  └─────────┘  └─────────┘  └─────────┘             │  │
│   └─────────────────────────────────────────────────────┘  │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 7. 국제 표준 호환성 (International Compatibility)

### 7.1 표준 매핑

| WIA-MRV 필드 | GHG Protocol | ISO 14064-1 | EU ETS MRV |
|--------------|--------------|-------------|------------|
| facilityId | OrganizationalBoundary | 3.1 Organization | InstallationId |
| scope1 | DirectEmissions | 3.5 Direct GHG | AnnualEmissions |
| scope2 | EnergyIndirect | 3.6 Energy indirect | - |
| scope3 | OtherIndirect | 3.7 Other indirect | - |
| methodology | CalculationApproach | 4.3 Quantification | CalculationMethod |
| verification | Assurance | Clause 7 Verification | VerificationReport |

### 7.2 API 상호운용성

```typescript
// 표준 변환 예시
import { WIAtoGHGProtocol, WIAtoISO14064 } from '@wia/climate';

const wiaReport: GHGEmission = { /* WIA format */ };

// GHG Protocol 형식으로 변환
const ghgProtocolFormat = WIAtoGHGProtocol(wiaReport);

// ISO 14064-1 형식으로 변환
const isoFormat = WIAtoISO14064(wiaReport);

// EU ETS MRV 형식으로 변환
const euEtsFormat = WIAtoEUETS(wiaReport);
```

---

## 8. 구현 가이드 (Implementation Guide)

### 8.1 Quick Start

```typescript
import { MRVCalculator, VerificationService } from '@wia/climate';

// 1. MRV 계산기 초기화
const mrv = new MRVCalculator();

// 2. Scope 1 배출량 계산
const scope1 = mrv.calculateScope1([
  {
    category: 'natural_gas',
    source: 'combustion',
    quantity: 10000,
    unit: 'm3',
    dataSource: 'DIRECT_METER'
  },
  {
    category: 'diesel',
    source: 'combustion',
    quantity: 5000,
    unit: 'L',
    dataSource: 'INVOICE'
  }
]);

// 3. Scope 2 배출량 계산
const scope2 = mrv.calculateScope2({
  consumption: 500000,  // kWh
  region: 'KR',
  fromInvoice: true
});

// 4. 전체 보고서 생성
const report = mrv.createEmissionReport(
  facility,
  { start: new Date('2024-01-01'), end: new Date('2024-12-31') },
  scope1Activities,
  scope2Data,
  scope3Categories,
  'GHG_PROTOCOL'
);

// 5. 자체 검증
const verification = new VerificationService();
const result = verification.selfVerify(report);

console.log(`Total emissions: ${report.scope1.total + report.scope2.total + report.scope3.total} tCO2e`);
console.log(`Verification: ${result.overallResult}`);
```

### 8.2 고급 사용법

```typescript
// AI 기반 배출량 추정
const aiEstimator = new AIEmissionEstimator();

// 위성 이미지로 추정
const estimate = await aiEstimator.estimateFromSatellite(
  'FACILITY-001',
  'https://satellite.example.com/image.tif',
  'POWER_PLANT'
);

// 이상치 탐지
const anomaly = aiEstimator.detectAnomalies(
  'FACILITY-001',
  historicalEmissions,
  currentEmission
);

if (anomaly) {
  console.warn(`Anomaly detected: ${anomaly.description}`);
}

// 블록체인 검증
const proof = await verification.generateBlockchainProof(report, {
  network: 'POLYGON',
  smartContract: '0x...',
  oracleSources: [/* ... */],
  consensusType: 'PROOF_OF_STAKE'
});

console.log(`Blockchain proof: ${proof.hash}`);
```

---

## 9. 부록 (Appendix)

### 9.1 산업별 배출 계수

전체 배출 계수 데이터베이스는 WIA-CLIMATE SDK에 포함되어 있습니다.

### 9.2 관련 표준

- GHG Protocol Corporate Standard
- ISO 14064-1:2018
- ISO 14064-3:2019 (Verification)
- ISO 14067:2018 (Carbon Footprint)
- EU ETS MRV Regulation
- EPA Greenhouse Gas Reporting Rule

### 9.3 참고 문헌

1. IPCC 2019 Refinement to the 2006 IPCC Guidelines
2. GHG Protocol: A Corporate Accounting and Reporting Standard
3. ISO 14064-1:2018 Specification with guidance
4. EU Commission Delegated Regulation (EU) 2019/331

---

**弘益人間 (홍익인간)** - 투명한 배출량 측정으로 인류의 기후 대응에 기여합니다.

© 2025 WIA (World Certification Industry Association)
