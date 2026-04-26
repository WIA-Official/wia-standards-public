# 제2장: 패션 산업의 현재 과제

## 학습 목표

이 장을 마치면 다음을 이해하게 됩니다:
- 전통적인 패션 생산의 환경적 영향
- 반품 및 사이즈 불일치 문제
- 패션 공급망의 낭비
- 사회적 및 노동 문제
- 기술이 이러한 문제를 해결하는 방법

---

## 2.1 환경 위기

패션 산업은 세계에서 가장 큰 오염원 중 하나입니다:

### 탄소 배출

```
패션 산업 연간 영향 (2024년):
┌────────────────────────────────────┐
│ 총 배출량: 12억 톤                  │
│ CO₂e/년 (전 세계의 2-8%)            │
│                                    │
│ 세부 내역:                          │
│ • 소재 생산:           45%          │
│ • 제조:               25%          │
│ • 운송:               10%          │
│ • 소매 운영:           5%          │
│ • 소비자 사용:         15%          │
└────────────────────────────────────┘

비교:
프랑스 + 독일 합계 = 11억 톤
패션 산업 = 12억 톤
```

**계산 예시:**

```typescript
interface GarmentCarbonFootprint {
  material: number;        // 소재
  manufacturing: number;   // 제조
  transport: number;       // 운송
  use: number;            // 사용
  endOfLife: number;      // 폐기
}

// 전통적인 패스트 패션 드레스
const fastFashionDress: GarmentCarbonFootprint = {
  material: 3.5,          // 폴리에스터: 0.5 kg × 7.0 kg CO₂e/kg
  manufacturing: 2.3,     // 복잡한 생산
  transport: 1.5,         // 항공 운송
  use: 24.0,              // 100회 세탁, 뜨거운 물, 건조기 사용
  endOfLife: 0.5          // 매립지 (메탄 배출)
};
const total = Object.values(fastFashionDress).reduce((a, b) => a + b, 0);
// 총합: 31.8 kg CO₂e

// 지속가능한 유기농 면 드레스
const sustainableDress: GarmentCarbonFootprint = {
  material: 0.63,         // 유기농 면: 0.3 kg × 2.1 kg CO₂e/kg
  manufacturing: 1.0,     // 재생 에너지 사용 윤리적 공장
  transport: 0.03,        // 해상 운송
  use: 7.5,               // 50회 세탁, 찬물, 자연 건조
  endOfLife: -1.0         // 기부/재활용 (탄소 크레딧)
};
const sustainableTotal = Object.values(sustainableDress).reduce((a, b) => a + b, 0);
// 총합: 8.16 kg CO₂e

// 절감량: 31.8 - 8.16 = 23.64 kg CO₂e (74% 감소)
```

### 물 소비량

| 소재 | 물 사용량 (L/kg) | 예시: 면 티셔츠 1장 (0.25 kg) |
|----------|-------------------:|-------------------------------------:|
| 일반 면 | 10,000 L | **2,500 L** |
| 유기농 면 | 7,000 L | 1,750 L |
| 폴리에스터 | 1,000 L | 250 L |
| 재활용 폴리에스터 | 500 L | 125 L |
| 리넨 | 2,500 L | 625 L |
| 텐셀 | 500 L | 125 L |

**2,500리터**는 다음과 같습니다:
- 한 사람의 **3.5년**치 식수
- **욕조 20개** 분량의 물
- **샤워 50회** 분량

### 화학 오염

```typescript
interface ChemicalImpact {
  dyeingProcess: {
    waterUsed: number;          // 직물 kg당 리터
    chemicalsUsed: string[];    // 사용된 화학물질
    toxicWaste: number;         // 직물 kg당 kg
    waterPolluted: number;      // % 처리됨 vs. 미처리
  };
}

const conventionalDyeing: ChemicalImpact = {
  dyeingProcess: {
    waterUsed: 100,              // kg당 100L
    chemicalsUsed: [
      '중금속 (크롬, 납)',
      '아조 염료',
      '포름알데히드',
      '표백제',
      '유연제'
    ],
    toxicWaste: 0.2,             // 20%가 독성 폐기물
    waterPolluted: 80            // 일부 지역에서 80% 미처리 배출
  }
};

// 전 세계 산업용 수질 오염의 20%가 섬유 염색에서 발생
```

---

## 2.2 반품 문제

전자상거래 패션은 대규모 반품 문제를 안고 있습니다:

### 반품 통계

```
업계 평균 반품률:
┌─────────────────────────────────────┐
│ 전체 패션:            25-30%        │
│ 온라인 패션:          30-40%        │
│ 신발:                35-45%        │
│ 정장:                40-50%        │
│                                     │
│ 매장 내:              8-10%         │
│ (비교용)                            │
└─────────────────────────────────────┘

반품 사유 (온라인 패션):
┌─────────────────────────┬─────────┐
│ 사유                    │ %       │
├─────────────────────────┼─────────┤
│ 잘못된 사이즈/핏        │ 60%     │
│ 다르게 보임             │ 20%     │
│ 품질 문제               │ 10%     │
│ 마음이 바뀜             │ 8%      │
│ 기타                    │ 2%      │
└─────────────────────────┴─────────┘
```

### 반품 비용

```typescript
interface ReturnCosts {
  // 반품당
  logistics: number;        // 역방향 배송
  processing: number;       // 창고 처리
  refurbishment: number;    // 세척, 재포장
  markdown: number;         // 할인 판매
  disposal: number;         // 판매 불가능한 경우

  // 환경적 비용
  carbonEmissions: number;  // 반품당 kg CO₂e
  packaging: number;        // 추가 포장재
}

const averageReturn: ReturnCosts = {
  logistics: 8.50,          // $8.50 역방향 배송
  processing: 3.00,         // $3.00 처리
  refurbishment: 2.50,      // $2.50 세척/재포장
  markdown: 12.00,          // $12.00 평균 가격 인하
  disposal: 1.00,           // $1.00 폐기 (반품의 25%)

  carbonEmissions: 2.5,     // 왕복 2.5 kg CO₂e
  packaging: 0.15           // 150g 추가 포장재
};

const totalCostPerReturn = Object.values(averageReturn)
  .slice(0, 5)
  .reduce((a, b) => a + b, 0);
// 총 비용: 반품당 $27.00

// 다음과 같은 소매업체의 경우:
// - 연간 매출 $100M
// - 반품률 30%
// - 평균 주문 금액 $75
//
// 반품 = ($100M / $75) × 0.30 = 400,000건
// 비용 = 400,000 × $27 = $10.8M/년
// 탄소 = 400,000 × 2.5 kg = 1,000톤 CO₂e
```

### 반품의 환경 영향

```
연간 반품 주기:
┌────────────────────────────────────┐
│ 고객 → 창고 → 고객                 │
│    └──────────┬──────────┘         │
│            반품                     │
│                                    │
│ 미국 패션 반품 (2024년):            │
│ • 50억 개 품목 반품                │
│ • 1,250만 톤 CO₂e                 │
│ • 연간 270만 대 자동차와 동일      │
│                                    │
│ 폐기:                              │
│ • 25%가 매립지로                   │
│ • 12억 5천만 개 품목 폐기          │
└────────────────────────────────────┘
```

---

## 2.3 사이즈 불일치

### 사이즈 차트 문제

브랜드마다 사이즈가 크게 다릅니다:

```typescript
interface BrandSizing {
  brand: string;
  size: string;
  measurements: {
    chest: number;  // cm
    waist: number;  // cm
    hips: number;   // cm
  };
}

// 다른 브랜드의 같은 "사이즈 M":
const sizeMComparison: BrandSizing[] = [
  {
    brand: '브랜드 A (럭셔리)',
    size: 'M',
    measurements: { chest: 96, waist: 78, hips: 100 }
  },
  {
    brand: '브랜드 B (패스트 패션)',
    size: 'M',
    measurements: { chest: 92, waist: 74, hips: 96 }
  },
  {
    brand: '브랜드 C (애슬레틱)',
    size: 'M',
    measurements: { chest: 100, waist: 82, hips: 102 }
  },
  {
    brand: '브랜드 D (지속가능)',
    size: 'M',
    measurements: { chest: 94, waist: 76, hips: 98 }
  }
];

// 가슴둘레 범위: 92-100 cm (8cm 차이!)
// 가슴둘레 94cm인 사람은 브랜드에 따라 S, M 또는 L일 수 있음
```

### 허영 사이즈 트렌드

```
시간에 따른 사이즈 인플레이션 (여성 미국 사이즈 8):
┌─────────────────────────────────────────┐
│ 연도    허리 사이즈 (인치)               │
├─────────────────────────────────────────┤
│ 1950년대 24-25"  ████░░░░░░              │
│ 1970년대 26-27"  █████░░░░░              │
│ 1990년대 28-29"  ██████░░░░              │
│ 2000년대 30-31"  ███████░░░              │
│ 2020년대 32-33"  ████████░░              │
└─────────────────────────────────────────┘

같은 숫자 사이즈 = 70년 동안 8인치 더 커짐!
```

### 신체 다양성 문제

전통적인 사이즈 차트는 다음을 고려하지 못합니다:

```typescript
interface BodyDiversity {
  // 키 변화
  heightRange: {
    petite: number;    // <160 cm
    regular: number;   // 160-173 cm
    tall: number;      // >173 cm
  };

  // 체형 비율
  bodyShapes: string[]; // ['pear', 'hourglass', 'apple', 'rectangle', 'inverted_triangle']

  // 지역별 차이
  regionalAverages: {
    region: string;
    averageHeight: number;
    averageChest: number;
    averageWaist: number;
  }[];

  // 연령 관련 변화
  ageGroups: string[]; // 나이에 따라 체형 비율 변화
}

// 예시: 원사이즈가 모두에게 맞지 않는 이유
const diversityExample: BodyDiversity = {
  heightRange: {
    petite: 155,  // 5'1"
    regular: 165, // 5'5"
    tall: 178     // 5'10"
  },

  bodyShapes: [
    'pear',       // 엉덩이 >> 가슴
    'hourglass',  // 가슴 ≈ 엉덩이, 작은 허리
    'apple',      // 복부가 풍만
    'rectangle',  // 일자형
    'inverted_triangle' // 넓은 어깨, 좁은 엉덩이
  ],

  regionalAverages: [
    { region: '동아시아', averageHeight: 162, averageChest: 84, averageWaist: 68 },
    { region: '북미', averageHeight: 165, averageChest: 92, averageWaist: 76 },
    { region: '북유럽', averageHeight: 168, averageChest: 90, averageWaist: 72 }
  ],

  ageGroups: [
    '18-25', '26-35', '36-45', '46-55', '56-65', '65+'
  ]
};

// "사이즈 M"이 이 모든 변화를 고려할 수 없습니다!
```

---

## 2.4 공급망 전반의 낭비

### 소비자 이전 낭비

```
디자인에서 생산까지의 낭비:
┌────────────────────────────────────┐
│ 물리적 샘플링:                      │
│ • 컬렉션당 100-200개 샘플           │
│ • 70-90%는 생산되지 않음            │
│ • 샘플당 14 kg 직물 낭비            │
│                                    │
│ 패턴 재단:                          │
│ • 평균 직물 활용률: 75%             │
│ • 재단 과정에서 25% 낭비            │
│ • 불규칙한 모양 = 더 많은 낭비      │
│                                    │
│ 과잉 생산:                          │
│ • 판매량보다 30% 더 많이 생산       │
│ • 미판매 재고 폐기                  │
│ • 연간 1,280만 톤 매립              │
└────────────────────────────────────┘
```

**디지털 솔루션:**

```typescript
interface WasteReduction {
  traditional: {
    physicalSamples: number;      // 100개 샘플
    fabricPerSample: number;      // 1.5 kg
    sampleWaste: number;          // 70% × 100 × 1.5 = 105 kg
    cuttingEfficiency: number;    // 75%
    cuttingWaste: number;         // 직물의 25%
  };

  withDigitalTech: {
    virtualSamples: number;       // 가상 100개, 물리적 30개
    fabricPerSample: number;      // 1.5 kg
    sampleWaste: number;          // 30 × 1.5 = 45 kg
    cuttingEfficiency: number;    // 85% (최적화된 패턴)
    cuttingWaste: number;         // 직물의 15%
  };

  savings: {
    sampleWaste: number;          // 105 - 45 = 60 kg (57% 감소)
    cuttingWaste: number;         // 10% 효율 향상
    carbonSaved: number;          // kg CO₂e
  };
}

const wasteReduction: WasteReduction = {
  traditional: {
    physicalSamples: 100,
    fabricPerSample: 1.5,
    sampleWaste: 105,
    cuttingEfficiency: 75,
    cuttingWaste: 25
  },

  withDigitalTech: {
    virtualSamples: 100,    // 모두 가상으로 시작
    fabricPerSample: 1.5,
    sampleWaste: 45,        // 물리적 샘플 30개만
    cuttingEfficiency: 85,  // AI 최적화 패턴
    cuttingWaste: 15
  },

  savings: {
    sampleWaste: 60,        // 57% 감소
    cuttingWaste: 10,       // 40% 상대 감소 (25%에서 15%로)
    carbonSaved: 354        // 컬렉션당 kg CO₂e 절감
  }
};
```

### 소비자 이후 낭비

```
소비자 행동:
┌────────────────────────────────────┐
│ 평균 의류 수명:                     │
│ • 패스트 패션: 5-7회 착용           │
│ • 중가 브랜드: 20-30회 착용         │
│ • 고품질: 50-100회 착용             │
│                                    │
│ 수명 종료:                          │
│ • 매립:     73%                    │
│ • 재활용:   12%                    │
│ • 기부:     15%                    │
│                                    │
│ 전 세계 섬유 폐기물:                │
│ • 연간 9,200만 톤                  │
│ • 매초 쓰레기 트럭 1대              │
│   매립지로                          │
└────────────────────────────────────┘
```

---

## 2.5 사회 및 노동 문제

### 공급망 불투명성

```
전통적인 공급망:
┌─────────────────────────────────────────────┐
│ 브랜드 → 대리인 → 공장 → 하청업체 → ?        │
│                                             │
│ 문제점:                                      │
│ • 알 수 없는 근무 조건                       │
│ • 불명확한 임금 수준                         │
│ • Tier 1 이후 가시성 없음                   │
│ • 아동 노동 위험                             │
│ • 안전 위반                                  │
└─────────────────────────────────────────────┘

블록체인을 통한 투명성:
┌─────────────────────────────────────────────┐
│ 면 농장 → 방직 공장 → 제조 공장 →           │
│ → 창고 → 소매                                │
│                                             │
│ 각 단계가 블록체인에 기록됨:                 │
│ ✓ 근로자 임금 검증됨                        │
│ ✓ 인증서 확인됨                             │
│ ✓ 안전 감사 기록됨                          │
│ ✓ 화학물질 사용 추적됨                      │
└─────────────────────────────────────────────┘
```

### 노동 통계

```typescript
interface LaborIssues {
  workers: {
    total: number;              // 전 세계 7,500만 명
    women: number;              // 80%가 여성
    wages: {
      livingWage: number;       // 기본 필요를 위한 필수 임금
      actualWage: number;       // 대부분이 받는 임금
      gap: number;              // 생활 임금보다 낮은 비율
    };
  };

  workingConditions: {
    averageHoursPerWeek: number;
    overtimeUnpaid: number;     // 무급 초과 근무 %
    safetyCertified: number;    // 인증된 공장 %
    accidents: number;          // 연간 사고
  };
}

const globalFashionLabor: LaborIssues = {
  workers: {
    total: 75_000_000,
    women: 60_000_000,  // 80%
    wages: {
      livingWage: 400,    // 월 $400 필요
      actualWage: 200,    // 월 평균 $200
      gap: 50             // 생활 임금보다 50% 낮음
    }
  },

  workingConditions: {
    averageHoursPerWeek: 60,    // 표준 40시간 vs.
    overtimeUnpaid: 40,         // 40% 무급 초과 근무
    safetyCertified: 35,        // 35%만 인증됨
    accidents: 5000             // 연간 심각한 사고
  }
};
```

---

## 2.6 기술이 이러한 문제를 해결하는 방법

### 문제-솔루션 매트릭스

| 문제 | 기술 솔루션 | 영향 |
|-----------|---------------------|--------|
| **높은 탄소 배출** | 가상 샘플링, 디지털 디자인 | 샘플 70% 감소 |
| **수질 오염** | 지속가능 소재 데이터베이스 | 정보에 입각한 소재 선택 |
| **높은 반품률** | 가상 착용, 사이즈 AI | 반품 35-45% 감소 |
| **사이즈 불일치** | 범용 사이즈 표준 | 92% 사이즈 정확도 |
| **생산 낭비** | AI 패턴 최적화 | 소재 10-15% 절약 |
| **공급망 불투명성** | 블록체인 추적 | 100% 추적 가능성 |
| **노동 위반** | 스마트 계약, 감사 | 검증된 공정 임금 |
| **과잉 생산** | 수요 예측 AI | 과잉 재고 30% 감소 |

### 정량화된 영향

```typescript
interface TechnologyImpact {
  environmental: {
    carbonReduction: number;     // 절감된 톤 CO₂e
    waterSaved: number;          // 리터
    wasteReduced: number;        // kg
  };

  business: {
    returnsSaved: number;        // $
    timeSaved: number;           // 시장 출시 주 단축
    costReduction: number;       // % 운영 비용
  };

  social: {
    workersProtected: number;    // 공정 임금 검증
    transparencyScore: number;   // 0-100
    consumerTrust: number;       // % 증가
  };
}

// 예시: WIA 표준을 구현하는 중규모 패션 브랜드
const annualImpact: TechnologyImpact = {
  environmental: {
    carbonReduction: 500_000,    // 500톤 CO₂e
    waterSaved: 50_000_000,      // 5,000만 리터
    wasteReduced: 30_000         // 30톤 직물
  },

  business: {
    returnsSaved: 2_000_000,     // $2M 절감
    timeSaved: 5,                // 5주 빠름
    costReduction: 15            // 15% 비용 절감
  },

  social: {
    workersProtected: 2500,      // 검증된 임금을 받는 2,500명 근로자
    transparencyScore: 85,       // 30에서 85로
    consumerTrust: 40            // 신뢰도 40% 증가
  }
};
```

---

## 복습 질문

1. **섬유 염색에서 전 세계 산업용 수질 오염의 몇 퍼센트가 발생합니까?**
   <details>
   <summary>답변</summary>
   전 세계 산업용 수질 오염의 20%가 섬유 염색 및 처리에서 발생합니다.
   </details>

2. **연간 매출 $50M, 반품률 35%, 평균 주문 금액 $60인 소매업체의 총 비용을 계산하세요:**
   <details>
   <summary>답변</summary>
   - 주문 수: $50M / $60 = 833,333
   - 반품: 833,333 × 0.35 = 291,667
   - 반품당 $27 비용: 291,667 × $27 = $7,875,000 (거의 $8M)
   </details>

3. **온라인 패션 반품의 주요 이유는 무엇입니까?**
   <details>
   <summary>답변</summary>
   잘못된 사이즈/핏이 반품의 60%를 차지합니다.
   </details>

4. **가상 샘플링으로 얼마나 많은 직물 낭비를 제거할 수 있습니까?**
   <details>
   <summary>답변</summary>
   물리적 샘플 70% 감소, 샘플 직물 낭비 57% 절감 (예: 컬렉션당 60 kg).
   </details>

5. **현재 섬유 폐기물의 몇 퍼센트가 매립지로 갑니까?**
   <details>
   <summary>답변</summary>
   섬유 폐기물의 73%가 매립지로 가며, 12%만 재활용되고 15%가 기부됩니다.
   </details>

6. **1950년대 이후 미국 여성 사이즈 8은 얼마나 커졌습니까?**
   <details>
   <summary>답변</summary>
   허리 치수로 약 8인치 더 커졌습니다 (24-25"에서 32-33"로), "허영 사이즈" 인플레이션을 보여줍니다.
   </details>

---

## 다음 단계

이러한 문제를 이해하면 WIA 패션 기술 표준이 각 문제에 대한 솔루션을 제공하는 방법을 이해할 수 있습니다. [**제3장: 표준 개요**](03-standard-overview.md)에서 이러한 문제를 해결하기 위해 설계된 포괄적인 아키텍처를 살펴보겠습니다.

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
