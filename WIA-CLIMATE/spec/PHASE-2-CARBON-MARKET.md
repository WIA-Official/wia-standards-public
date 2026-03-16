# WIA-CLIMATE Phase 2: Carbon Market (탄소시장 통합)
# Carbon Market Integration Standard

**Version**: 1.0.0
**Status**: Draft
**Last Updated**: 2025-01-01

---

## 1. 개요 (Overview)

### 1.1 목적

WIA-CLIMATE-CARBON-MARKET 표준은 분산된 글로벌 탄소시장을 연결하고, 이중계산을 방지하며, 투명한 탄소 거래를 가능하게 합니다.

**핵심 목표**:
- EU ETS, 중국 ETS, 자발적 시장 연결
- Article 6 (파리협정) 호환 표준
- 이중계산 방지 메커니즘
- 그린워싱 탐지 알고리즘

### 1.2 글로벌 탄소시장 현황

```
┌─────────────────────────────────────────────────────────────┐
│                    글로벌 탄소시장 지도                       │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  규제시장 (Compliance Markets)                               │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  EU ETS        중국 ETS      한국 ETS     캘리포니아   │   │
│  │  $85/t         $12/t        $20/t       $35/t       │   │
│  │  1.5Gt         4.5Gt        0.6Gt       0.4Gt       │   │
│  └─────────────────────────────────────────────────────┘   │
│                                                             │
│  자발적시장 (Voluntary Markets)                              │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  Verra (VCS)   Gold Standard  ACR        Puro        │   │
│  │  $12/t         $25/t         $15/t      $150/t      │   │
│  │  1.0Gt         0.2Gt         0.1Gt      0.01Gt      │   │
│  └─────────────────────────────────────────────────────┘   │
│                                                             │
│  문제점                                                      │
│  ├── 가격 편차: $5 ~ $150 (30배)                            │
│  ├── 이중계산: NDC 중복 카운팅                               │
│  ├── 그린워싱: 품질 불확실                                   │
│  └── 상호운용성 부재                                         │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. 탄소크레딧 표준 (Carbon Credit Standard)

### 2.1 크레딧 데이터 스키마

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://wia.org/schemas/climate/carbon-credit.json",
  "title": "WIA Carbon Credit",
  "type": "object",
  "required": ["creditId", "registry", "vintage", "projectType", "quantity"],
  "properties": {
    "creditId": {
      "type": "string",
      "pattern": "^[A-Z0-9-]+$",
      "description": "Unique credit identifier"
    },
    "registry": {
      "type": "string",
      "enum": ["EU_ETS", "UK_ETS", "CHINA_ETS", "KOREA_ETS", "VERRA", "GOLD_STANDARD", "ACR", "WIA_REGISTRY"]
    },
    "vintage": {
      "type": "integer",
      "minimum": 2000,
      "maximum": 2100,
      "description": "Credit issuance year"
    },
    "projectType": {
      "type": "string",
      "enum": [
        "RENEWABLE_ENERGY",
        "ENERGY_EFFICIENCY",
        "FORESTRY_AFFORESTATION",
        "FORESTRY_REDD_PLUS",
        "BLUE_CARBON",
        "DIRECT_AIR_CAPTURE",
        "CARBON_CAPTURE_STORAGE",
        "METHANE_CAPTURE"
      ]
    },
    "projectId": { "type": "string" },
    "projectName": { "type": "string" },
    "projectLocation": { "$ref": "#/$defs/geoLocation" },
    "quantity": {
      "type": "number",
      "description": "Quantity in tCO2e"
    },
    "price": { "$ref": "#/$defs/money" },
    "status": {
      "type": "string",
      "enum": ["ISSUED", "LISTED", "TRADED", "RETIRED", "CANCELLED", "SUSPENDED"]
    },
    "article6Eligible": {
      "type": "boolean",
      "description": "Paris Agreement Article 6 compatible"
    },
    "correspondingAdjustment": {
      "type": "boolean",
      "description": "CA applied to prevent double counting"
    },
    "serialNumber": { "type": "string" },
    "issuanceDate": { "type": "string", "format": "date" },
    "expiryDate": { "type": "string", "format": "date" },
    "retirementInfo": { "$ref": "#/$defs/retirement" }
  }
}
```

### 2.2 프로젝트 유형별 요건

| 프로젝트 유형 | 추가성 증명 | 영속성 보장 | 검증 주기 | Article 6 적합 |
|--------------|------------|------------|-----------|---------------|
| 재생에너지 | 재정 분석 | N/A | 연간 | ✅ |
| REDD+ | 기준선 비교 | 버퍼풀 20% | 5년 | ⚠️ 조건부 |
| 조림 | 토지 이용 증명 | 버퍼풀 15% | 5년 | ✅ |
| DAC | 기술 증명 | 지질 저장 | 연간 | ✅ |
| CCS | 기술 증명 | 모니터링 100년 | 연간 | ✅ |
| 블루카본 | 기준선 비교 | 버퍼풀 25% | 5년 | ⚠️ 조건부 |

---

## 3. Article 6 호환성 (Paris Agreement Compliance)

### 3.1 Article 6.2 (양자 협력)

```
┌─────────────────────────────────────────────────────────────┐
│                Article 6.2 ITMOs 거래 구조                   │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│   호스트국 (Host Country)         획득국 (Acquiring Country) │
│   ┌───────────────┐               ┌───────────────┐        │
│   │               │               │               │        │
│   │   NDC: -40%   │──── ITMO ────▶│   NDC: -30%   │        │
│   │               │               │               │        │
│   │   배출: 100Mt │               │   배출: 200Mt │        │
│   │   목표: 60Mt  │               │   목표: 140Mt │        │
│   │               │               │               │        │
│   └───────────────┘               └───────────────┘        │
│          │                               │                  │
│          ▼                               ▼                  │
│   상응조정 (CA)                    크레딧 사용               │
│   NDC에서 5Mt 추가                 NDC에서 5Mt 상쇄          │
│   (60Mt → 65Mt)                   (140Mt - 5Mt = 135Mt)    │
│                                                             │
│   결과: 글로벌 배출량 동일 유지 (이중계산 방지)               │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 3.2 Article 6.4 (메커니즘)

```
┌─────────────────────────────────────────────────────────────┐
│                Article 6.4 메커니즘 구조                     │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│   감독기구 (Supervisory Body)                                │
│   ├── 방법론 승인                                            │
│   ├── 검증기관 인증                                          │
│   ├── 크레딧 발행 승인                                       │
│   └── 분쟁 해결                                              │
│                                                             │
│   A6.4ER (크레딧)                                            │
│   ├── 전 세계 단일 레지스트리                                 │
│   ├── 2% 적응 기금 기여                                      │
│   ├── 5% 취소 (전체 완화 효과)                               │
│   └── 상응조정 의무화                                        │
│                                                             │
│   WIA-CLIMATE 연동                                           │
│   ├── A6.4ER ↔ WIA Credit 변환                              │
│   ├── CA 상태 실시간 추적                                    │
│   └── 이중계산 자동 탐지                                     │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 3.3 상응조정 (Corresponding Adjustment) 확인

```typescript
interface CorrespondingAdjustment {
  creditId: string;
  hostCountry: string;
  acquiringEntity: string;
  quantity: number;
  adjustmentDate: Date;
  hostNDCReference: string;
  status: 'PENDING' | 'APPLIED' | 'VERIFIED';
  registryConfirmation: string;
}

function verifyCA(credit: CarbonCredit): CAVerificationResult {
  // 1. 호스트국 NDC 확인
  const hostNDC = fetchNDC(credit.projectLocation.country);

  // 2. 상응조정 기록 확인
  const caRecord = fetchCARecord(credit.creditId);

  // 3. 이중계산 검사
  const doubleCountingCheck = checkDoubleCount(credit.creditId);

  return {
    valid: caRecord?.status === 'APPLIED' && !doubleCountingCheck.found,
    hostNDC,
    caRecord,
    doubleCountingRisk: doubleCountingCheck
  };
}
```

---

## 4. 이중계산 방지 (Double Counting Prevention)

### 4.1 이중계산 유형

```
┌─────────────────────────────────────────────────────────────┐
│                    이중계산 유형 분류                         │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  1. 이중 발행 (Double Issuance)                              │
│     동일 감축량에 대해 여러 크레딧 발행                        │
│     ┌─────┐                                                 │
│     │프로젝트│ ──▶ 크레딧 A (Verra)                          │
│     │  A   │ ──▶ 크레딧 A' (Gold Standard) ← 중복!          │
│     └─────┘                                                 │
│                                                             │
│  2. 이중 청구 (Double Claiming)                              │
│     동일 크레딧을 여러 주체가 주장                             │
│     ┌─────┐                                                 │
│     │크레딧│ ──▶ 국가 NDC 달성                               │
│     │  A  │ ──▶ 기업 탄소중립 주장 ← 중복!                   │
│     └─────┘                                                 │
│                                                             │
│  3. 이중 사용 (Double Use)                                   │
│     상쇄된 크레딧 재사용                                      │
│     ┌─────┐                                                 │
│     │크레딧│ ──▶ 2023년 상쇄 (Retired)                       │
│     │  A  │ ──▶ 2024년 재판매 ← 사기!                        │
│     └─────┘                                                 │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 4.2 WIA 이중계산 방지 시스템

```
┌─────────────────────────────────────────────────────────────┐
│                WIA Double Counting Prevention                │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│   레이어 1: 고유 식별자                                       │
│   ┌─────────────────────────────────────────────────────┐   │
│   │  WIA-CREDIT-ID = Registry + Project + Serial + Hash  │   │
│   │  예: VERRA-1234-00001-a7b3c9d2                        │   │
│   └─────────────────────────────────────────────────────┘   │
│                                                             │
│   레이어 2: 크로스 레지스트리 검증                            │
│   ┌─────────────────────────────────────────────────────┐   │
│   │  ┌─────┐  ┌─────┐  ┌─────┐  ┌─────┐               │   │
│   │  │Verra│  │ GS  │  │ ACR │  │ETS  │               │   │
│   │  └──┬──┘  └──┬──┘  └──┬──┘  └──┬──┘               │   │
│   │     │        │        │        │                   │   │
│   │     └────────┴────────┴────────┘                   │   │
│   │                   │                                 │   │
│   │          ┌────────▼────────┐                       │   │
│   │          │  WIA Registry   │                       │   │
│   │          │  Aggregator     │                       │   │
│   │          └─────────────────┘                       │   │
│   └─────────────────────────────────────────────────────┘   │
│                                                             │
│   레이어 3: 블록체인 불변 기록                                │
│   ┌─────────────────────────────────────────────────────┐   │
│   │  Block #12345                                       │   │
│   │  ├── Credit ID: VERRA-1234-00001                   │   │
│   │  ├── Status: RETIRED                               │   │
│   │  ├── Retired By: Corp XYZ                          │   │
│   │  ├── Timestamp: 2024-06-15T10:30:00Z               │   │
│   │  └── Hash: 0x7a8b9c...                             │   │
│   └─────────────────────────────────────────────────────┘   │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 5. 그린워싱 탐지 (Greenwashing Detection)

### 5.1 탐지 알고리즘

```typescript
interface GreenwashingIndicator {
  type: GreenwashingType;
  severity: 'LOW' | 'MEDIUM' | 'HIGH' | 'CRITICAL';
  confidence: number;
  evidence: string[];
}

const GREENWASHING_RULES: GreenwashingRule[] = [
  {
    type: 'ADDITIONALITY_CONCERN',
    condition: (credit) =>
      credit.projectType === 'RENEWABLE_ENERGY' &&
      credit.vintage < 2020 &&
      credit.projectLocation.country in DEVELOPED_COUNTRIES,
    severity: 'MEDIUM',
    description: '선진국의 오래된 재생에너지 프로젝트는 추가성 의심'
  },
  {
    type: 'PERMANENCE_RISK',
    condition: (credit) =>
      ['FORESTRY_AFFORESTATION', 'FORESTRY_REDD_PLUS', 'BLUE_CARBON'].includes(credit.projectType),
    severity: 'MEDIUM',
    description: '자연 기반 솔루션의 영속성 리스크'
  },
  {
    type: 'OVER_CREDITING',
    condition: (credit) =>
      credit.vintage < 2015 && credit.quantity > 10000,
    severity: 'HIGH',
    description: '오래된 대량 크레딧은 과대 발행 가능성'
  },
  {
    type: 'BASELINE_MANIPULATION',
    condition: (credit) =>
      credit.projectType === 'FORESTRY_REDD_PLUS' &&
      !credit.thirdPartyVerification,
    severity: 'HIGH',
    description: 'REDD+ 프로젝트의 기준선 조작 위험'
  },
  {
    type: 'DOUBLE_COUNTING',
    condition: (credit) =>
      credit.article6Eligible && !credit.correspondingAdjustment,
    severity: 'CRITICAL',
    description: 'Article 6 크레딧에 상응조정 미적용'
  }
];
```

### 5.2 위험 등급

```
┌─────────────────────────────────────────────────────────────┐
│                    그린워싱 위험 등급                         │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ✅ 등급 A (Low Risk)                                       │
│     ├── 인증 검증기관 검증                                   │
│     ├── 최근 빈티지 (3년 이내)                               │
│     ├── Article 6 호환 + CA 적용                            │
│     └── 위험 점수 < 20                                       │
│                                                             │
│  ⚠️ 등급 B (Medium Risk)                                    │
│     ├── 제3자 검증                                           │
│     ├── 빈티지 3-7년                                         │
│     ├── 일부 위험 지표 존재                                   │
│     └── 위험 점수 20-50                                      │
│                                                             │
│  🟠 등급 C (High Risk)                                       │
│     ├── 검증 수준 낮음                                        │
│     ├── 빈티지 7년 이상                                       │
│     ├── 복수 위험 지표                                        │
│     └── 위험 점수 50-75                                      │
│                                                             │
│  ❌ 등급 D (Critical Risk)                                   │
│     ├── 검증 불충분 또는 없음                                 │
│     ├── 이중계산 의심                                         │
│     ├── 심각한 위험 지표                                      │
│     └── 위험 점수 > 75                                       │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 6. 가격 발견 메커니즘 (Price Discovery)

### 6.1 가격 수렴 모델

```
┌─────────────────────────────────────────────────────────────┐
│                    탄소 가격 수렴 모델                        │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│   현재 상태 (2024)                                           │
│   가격 $/tCO2e                                               │
│   │                                                         │
│   150├──────────────────── Puro (CDR)                       │
│      │                                                      │
│   100├                                                      │
│      │                                                      │
│    85├────────────────── EU ETS                             │
│      │                                                      │
│    50├                                                      │
│      │                                                      │
│    35├────────── California                                 │
│    25├────── Gold Standard                                  │
│    20├──── Korea ETS                                        │
│    12├── China ETS / Verra                                  │
│      │                                                      │
│     0└──────────────────────────────────────────            │
│                                                             │
│   WIA 가격 수렴 전략                                         │
│   ├── 품질 기반 차등화: 고품질 = 높은 가격                    │
│   ├── 투명성 프리미엄: 검증 수준에 따른 가산                  │
│   ├── Article 6 프리미엄: CA 적용 크레딧 우대                 │
│   └── 시장 연결: 차익거래로 가격 수렴 촉진                    │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 6.2 실시간 가격 API

```typescript
interface CarbonPriceAPI {
  // 현재 가격 조회
  getCurrentPrice(registry: CarbonRegistry): Promise<CarbonPrice>;

  // 전체 시장 가격 조회
  getAllPrices(): Promise<Map<CarbonRegistry, CarbonPrice>>;

  // 가격 스프레드 계산
  calculateSpread(reg1: CarbonRegistry, reg2: CarbonRegistry): Promise<number>;

  // 미래 가격 추정
  estimateFuturePrice(registry: CarbonRegistry, years: number): Promise<Money>;

  // 가격 알림 구독
  subscribePriceAlerts(
    registry: CarbonRegistry,
    threshold: number,
    callback: (price: CarbonPrice) => void
  ): Subscription;
}

// 사용 예시
const priceService = new CarbonPriceService();

// EU ETS 현재 가격
const euPrice = await priceService.getCurrentPrice('EU_ETS');
console.log(`EU ETS: $${euPrice.price.amount}/tCO2e`);

// 스프레드 계산
const spread = await priceService.calculateSpread('EU_ETS', 'CHINA_ETS');
console.log(`EU-China spread: $${spread}`);

// 5년 후 가격 추정
const futurePrice = await priceService.estimateFuturePrice('EU_ETS', 5);
console.log(`EU ETS in 2029: $${futurePrice.amount}`);
```

---

## 7. 토큰화 (Tokenization)

### 7.1 탄소 토큰 표준

```
┌─────────────────────────────────────────────────────────────┐
│                    WIA Carbon Token (WIAC)                   │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│   토큰 표준                                                  │
│   ├── ERC-20: 대체 가능 토큰 (일반 탄소)                     │
│   └── ERC-1155: 반대체 가능 (특정 프로젝트)                   │
│                                                             │
│   토큰 구조                                                  │
│   ┌─────────────────────────────────────────────────────┐   │
│   │  {                                                  │   │
│   │    "tokenId": "WIAC-VERRA-2024-001",                │   │
│   │    "standard": "ERC-1155",                          │   │
│   │    "underlyingCredits": 1000,                       │   │
│   │    "registry": "VERRA",                             │   │
│   │    "vintage": 2024,                                 │   │
│   │    "projectType": "FORESTRY_AFFORESTATION",         │   │
│   │    "metadata": {                                    │   │
│   │      "projectName": "Amazon Reforestation",         │   │
│   │      "location": "Brazil",                          │   │
│   │      "verifier": "SCS Global Services"              │   │
│   │    }                                                │   │
│   │  }                                                  │   │
│   └─────────────────────────────────────────────────────┘   │
│                                                             │
│   지원 네트워크                                              │
│   ├── Polygon (주요): 낮은 수수료, 높은 처리량               │
│   ├── Ethereum: 보안성, 유동성                              │
│   ├── Celo: 탄소 중립 블록체인                              │
│   └── Hedera: 기업용, 에너지 효율                           │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 7.2 토큰화 프로세스

```
┌─────────────────────────────────────────────────────────────┐
│                    탄소 토큰화 프로세스                       │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│   1. 크레딧 브릿징 (Credit Bridging)                         │
│      ┌─────────────────────────────────────────────────┐   │
│      │  원본 레지스트리 (Verra)                          │   │
│      │  └── 크레딧 1000t 잠금 (Lock)                     │   │
│      │      └── WIA 검증                                 │   │
│      │          └── 토큰 발행                            │   │
│      └─────────────────────────────────────────────────┘   │
│                                                             │
│   2. 토큰 민팅 (Token Minting)                               │
│      ┌─────────────────────────────────────────────────┐   │
│      │  function mintCarbonToken(                       │   │
│      │    uint256 amount,                               │   │
│      │    bytes32 creditProof,                          │   │
│      │    string memory metadata                        │   │
│      │  ) external onlyBridge {                         │   │
│      │    require(verifyCreditLock(creditProof));       │   │
│      │    _mint(msg.sender, tokenId, amount, "");       │   │
│      │    emit CarbonMinted(tokenId, amount);           │   │
│      │  }                                               │   │
│      └─────────────────────────────────────────────────┘   │
│                                                             │
│   3. 토큰 소각 (Token Burning = Retirement)                  │
│      ┌─────────────────────────────────────────────────┐   │
│      │  function retireCarbon(                          │   │
│      │    uint256 tokenId,                              │   │
│      │    uint256 amount,                               │   │
│      │    string memory beneficiary                     │   │
│      │  ) external {                                    │   │
│      │    _burn(msg.sender, tokenId, amount);           │   │
│      │    emit CarbonRetired(tokenId, amount, beneficiary);│   │
│      │    // 원본 레지스트리에 상쇄 통보                    │   │
│      │    notifyOriginalRegistry(tokenId, amount);      │   │
│      │  }                                               │   │
│      └─────────────────────────────────────────────────┘   │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 8. 거래 API (Trading API)

### 8.1 거래 인터페이스

```typescript
interface CarbonTradingAPI {
  // 크레딧 발행
  issueCredits(params: CreditIssuanceParams): Promise<CarbonCredit[]>;

  // 크레딧 조회
  getCredit(creditId: string): Promise<CarbonCredit>;

  // 크레딧 이전
  transferCredits(
    creditIds: string[],
    from: Party,
    to: Party,
    price: Money
  ): Promise<CarbonTransaction>;

  // 크레딧 상쇄
  retireCredits(
    creditIds: string[],
    retiredBy: Organization,
    purpose: RetirementPurpose,
    beneficiary?: string
  ): Promise<RetirementReceipt>;

  // 그린워싱 검사
  analyzeGreenwashing(creditIds: string[]): Promise<GreenwashingAlert[]>;

  // 포트폴리오 조회
  getPortfolio(organizationId: string): Promise<CreditPortfolio>;

  // 주문 생성
  createOrder(order: CarbonOrder): Promise<OrderConfirmation>;

  // 주문장 조회
  getOrderBook(registry: CarbonRegistry, projectType?: ProjectCategory): Promise<OrderBook>;
}
```

### 8.2 사용 예시

```typescript
import { CarbonRegistryService, GreenwashingDetector } from '@wia/climate';

const registry = new CarbonRegistryService();
const detector = new GreenwashingDetector();

// 1. 크레딧 발행
const credits = registry.issueCredits({
  registry: 'WIA_REGISTRY',
  vintage: 2024,
  projectType: 'FORESTRY_AFFORESTATION',
  projectId: 'WIA-FOREST-001',
  projectName: 'Korea Reforestation Project',
  projectLocation: { latitude: 37.5, longitude: 127.0 },
  quantity: 1000,
  correspondingAdjustment: true
});

// 2. 그린워싱 검사
const alerts = detector.analyzeCredits(credits);
if (alerts.length > 0) {
  console.warn('Greenwashing alerts:', alerts);
}

// 3. 크레딧 이전
const transaction = await registry.transferCredits(
  credits.map(c => c.creditId),
  { organization: seller, role: 'SELLER' },
  { organization: buyer, role: 'BUYER' },
  { amount: 25000, currency: 'USD' }
);

// 4. 크레딧 상쇄
const receipt = await registry.retireCredits(
  credits.map(c => c.creditId),
  retirer,
  'VOLUNTARY_OFFSETTING',
  'Company XYZ Carbon Neutral Claims'
);

console.log(`Retired ${receipt.totalQuantity} tCO2e`);
console.log(`Receipt ID: ${receipt.receiptId}`);
```

---

## 9. 구현 체크리스트

### 9.1 규제시장 연결

- [ ] EU ETS 레지스트리 연동
- [ ] 중국 ETS 레지스트리 연동
- [ ] 한국 ETS 레지스트리 연동
- [ ] 캘리포니아 CCI 연동
- [ ] RGGI 연동

### 9.2 자발적시장 연결

- [ ] Verra VCS 연동
- [ ] Gold Standard 연동
- [ ] American Carbon Registry 연동
- [ ] Climate Action Reserve 연동
- [ ] Puro.earth 연동

### 9.3 Article 6 호환

- [ ] ITMO 추적 시스템
- [ ] 상응조정 모니터링
- [ ] A6.4ER 레지스트리 연결
- [ ] 이중계산 방지 알고리즘

### 9.4 기술 구현

- [ ] 블록체인 스마트 컨트랙트
- [ ] 크로스 레지스트리 API
- [ ] 그린워싱 탐지 ML 모델
- [ ] 실시간 가격 피드

---

**弘益人間 (홍익인간)** - 투명한 탄소시장으로 기후 정의 실현

© 2025 WIA (World Certification Industry Association)
