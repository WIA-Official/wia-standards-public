# 제3장: 데이터 형식

## 자산 표현 및 교환 스키마

### 소개

효과적인 냉동보존 자산 관리는 다양한 자산 유형을 표현하고, 시간에 따른 평가를 추적하며, 신탁 구조를 관리하고, 조직 간 데이터 교환을 용이하게 하기 위한 표준화된 데이터 형식이 필요합니다. 이 장에서는 WIA Cryo-Asset 표준에서 사용되는 핵심 스키마와 데이터 구조를 정의하여 냉동보존 조직, 금융 기관, 법률 서비스 제공업체 및 기술 플랫폼 간의 상호 운용성을 보장합니다.

---

## 3.1 핵심 엔티티 스키마

### 환자 신원 스키마

```typescript
// 자산 관리를 위한 핵심 환자 신원
interface CryoPatient {
  // 고유 식별자
  id: string;  // UUID 형식
  organizationPatientId: string;  // 조직별 ID
  externalIds: ExternalIdentifier[];

  // 법적 신원
  legalIdentity: {
    fullLegalName: string;
    previousNames: string[];
    dateOfBirth: Date;
    dateOfLegalDeath: Date | null;
    placeOfBirth: string;
    citizenship: string[];
    taxIdentifiers: TaxIdentifier[];
    governmentIds: GovernmentId[];
  };

  // 보존 상태
  preservationStatus: {
    status: PreservationStatus;
    preservationType: 'WHOLE_BODY' | 'NEURO' | 'BRAIN_ONLY';
    preservationDate: Date | null;
    preservationLocation: string;
    organizationId: string;
  };

  // 연락처 및 관계
  relationships: {
    emergencyContacts: Contact[];
    legalRepresentatives: LegalRepresentative[];
    familyMembers: FamilyMember[];
    authorizedAgents: AuthorizedAgent[];
  };

  // 문서 참조
  documents: {
    membershipAgreement: DocumentReference;
    lastWill: DocumentReference | null;
    advanceDirective: DocumentReference | null;
    trustDocuments: DocumentReference[];
    identityDocuments: DocumentReference[];
  };

  // 메타데이터
  metadata: {
    createdAt: Date;
    updatedAt: Date;
    dataSource: string;
    verificationLevel: 'UNVERIFIED' | 'BASIC' | 'ENHANCED' | 'FULL';
  };
}

// 보존 상태 열거형
enum PreservationStatus {
  MEMBER_ACTIVE = 'MEMBER_ACTIVE',       // 회원 활성
  MEMBER_STANDBY = 'MEMBER_STANDBY',     // 회원 대기
  MEMBER_SUSPENDED = 'MEMBER_SUSPENDED', // 회원 일시 중지
  IN_TRANSPORT = 'IN_TRANSPORT',         // 운송 중
  PRESERVED = 'PRESERVED',               // 보존됨
  REVIVED = 'REVIVED',                   // 소생됨
  TERMINATED = 'TERMINATED',             // 종료됨
}

// 세금 식별자
interface TaxIdentifier {
  country: string;  // ISO 3166-1 alpha-2
  type: 'SSN' | 'TIN' | 'EIN' | 'ITIN' | 'OTHER';
  value: string;  // 암호화됨
  validFrom: Date;
  validTo: Date | null;
}
```

### 자산 스키마

```typescript
// 포괄적인 자산 표현
interface CryoAsset {
  // 식별
  id: string;  // UUID
  portfolioId: string;
  patientId: string;

  // 분류
  category: AssetCategory;
  subcategory: string;
  assetType: AssetType;

  // 설명
  name: string;
  description: string;
  notes: string;

  // 소유권
  ownership: AssetOwnership;

  // 평가
  acquisitionInfo: AcquisitionInfo;
  currentValuation: Valuation;
  valuationHistory: Valuation[];

  // 위치 및 보관
  location: AssetLocation;
  custody: CustodyInfo;

  // 문서화
  documentation: AssetDocumentation;

  // 블록체인 참조
  blockchainRef: BlockchainReference | null;

  // 상태 및 수명주기
  status: AssetStatus;
  lifecycle: AssetLifecycle;

  // 메타데이터
  metadata: AssetMetadata;
}

// 자산 카테고리
enum AssetCategory {
  FINANCIAL = 'FINANCIAL',                     // 금융
  REAL_PROPERTY = 'REAL_PROPERTY',             // 부동산
  PERSONAL_PROPERTY = 'PERSONAL_PROPERTY',     // 개인 재산
  DIGITAL = 'DIGITAL',                         // 디지털
  INTELLECTUAL_PROPERTY = 'INTELLECTUAL_PROPERTY', // 지적재산
  BUSINESS_INTEREST = 'BUSINESS_INTEREST',     // 사업 지분
  CRYONICS_SPECIFIC = 'CRYONICS_SPECIFIC',     // 냉동보존 전용
}

// 자산 유형 레지스트리
const AssetTypeRegistry = {
  FINANCIAL: {
    BANK_ACCOUNT: {
      subtypes: ['CHECKING', 'SAVINGS', 'MONEY_MARKET', 'CD'],  // 당좌, 저축, MMF, CD
    },
    BROKERAGE: {
      subtypes: ['INDIVIDUAL', 'JOINT', 'MARGIN'],  // 개인, 공동, 마진
    },
    RETIREMENT: {
      subtypes: ['401K', 'IRA', 'ROTH_IRA', 'SEP_IRA', 'PENSION'],  // 퇴직연금
    },
    INSURANCE: {
      subtypes: ['LIFE_WHOLE', 'LIFE_TERM', 'LIFE_UNIVERSAL', 'ANNUITY'],  // 생명보험
    },
    CRYPTOCURRENCY: {
      subtypes: ['BITCOIN', 'ETHEREUM', 'STABLECOIN', 'OTHER'],  // 암호화폐
    },
  },

  REAL_PROPERTY: {
    RESIDENTIAL: {
      subtypes: ['PRIMARY_RESIDENCE', 'VACATION', 'RENTAL'],  // 주거용
    },
    COMMERCIAL: {
      subtypes: ['OFFICE', 'RETAIL', 'INDUSTRIAL', 'MIXED_USE'],  // 상업용
    },
  },

  DIGITAL: {
    ACCOUNT: {
      subtypes: ['EMAIL', 'SOCIAL_MEDIA', 'CLOUD_STORAGE', 'SUBSCRIPTION'],  // 계정
    },
    NFT: {
      subtypes: ['ART', 'COLLECTIBLE', 'UTILITY'],  // NFT
    },
  },

  CRYONICS_SPECIFIC: {
    PATIENT_CARE_FUND: {
      subtypes: ['STORAGE', 'MAINTENANCE', 'EMERGENCY'],  // 환자 케어 펀드
    },
    REVIVAL_FUND: {
      subtypes: ['PROCEDURE', 'REHABILITATION', 'GENERAL'],  // 소생 펀드
    },
  },
};

// 소유권 구조
interface AssetOwnership {
  type: OwnershipType;
  holders: OwnershipHolder[];
  restrictions: OwnershipRestriction[];
  encumbrances: Encumbrance[];
}

enum OwnershipType {
  SOLE = 'SOLE',                         // 단독
  JOINT_TENANCY = 'JOINT_TENANCY',       // 합유
  TENANCY_IN_COMMON = 'TENANCY_IN_COMMON', // 공유
  COMMUNITY_PROPERTY = 'COMMUNITY_PROPERTY', // 부부공유재산
  TRUST = 'TRUST',                       // 신탁
  CORPORATE = 'CORPORATE',               // 법인
  PARTNERSHIP = 'PARTNERSHIP',           // 파트너십
}
```

### 평가 스키마

```typescript
// 자산 평가 구조
interface Valuation {
  id: string;
  assetId: string;

  // 평가 상세
  valuationDate: Date;
  effectiveDate: Date;
  value: number;
  currency: string;  // ISO 4217

  // 방법론
  valuationMethod: ValuationMethod;
  valuationBasis: ValuationBasis;
  valuationSource: ValuationSource;

  // 지원 데이터
  marketData: MarketData | null;
  appraisalData: AppraisalData | null;
  assumptions: ValuationAssumption[];

  // 신뢰도 및 품질
  confidenceLevel: 'HIGH' | 'MEDIUM' | 'LOW';
  qualityScore: number;  // 0-100

  // 감사 추적
  performedBy: string;
  reviewedBy: string | null;
  approvedBy: string | null;
  approvalDate: Date | null;

  // 블록체인 앵커
  blockchainHash: string | null;
}

enum ValuationMethod {
  MARKET_QUOTE = 'MARKET_QUOTE',           // 시장 시세
  RECENT_TRANSACTION = 'RECENT_TRANSACTION', // 최근 거래
  APPRAISAL = 'APPRAISAL',                 // 전문 감정
  DISCOUNTED_CASH_FLOW = 'DISCOUNTED_CASH_FLOW', // DCF 분석
  COMPARABLE_SALES = 'COMPARABLE_SALES',   // 비교 매매
  BOOK_VALUE = 'BOOK_VALUE',               // 장부가
  COST_BASIS = 'COST_BASIS',               // 원가
  ESTIMATED = 'ESTIMATED',                 // 내부 추정
}

enum ValuationBasis {
  FAIR_MARKET_VALUE = 'FAIR_MARKET_VALUE', // 공정시장가치
  LIQUIDATION_VALUE = 'LIQUIDATION_VALUE', // 청산가치
  REPLACEMENT_COST = 'REPLACEMENT_COST',   // 대체비용
  BOOK_VALUE = 'BOOK_VALUE',               // 장부가
  TAX_VALUE = 'TAX_VALUE',                 // 세금 가치
  INSURANCE_VALUE = 'INSURANCE_VALUE',     // 보험 가치
}
```

---

## 3.2 신탁 및 법인 스키마

### 신탁 스키마

```typescript
// 포괄적인 신탁 표현
interface CryonicsTrust {
  // 식별
  id: string;
  externalId: string | null;
  name: string;

  // 유형 및 분류
  trustType: TrustType;
  taxClassification: TaxClassification;

  // 관할권
  jurisdiction: TrustJurisdiction;

  // 당사자
  parties: TrustParties;

  // 조건
  terms: TrustTerms;

  // 자산
  assets: {
    initialFunding: FundingRecord[];
    currentHoldings: AssetHolding[];
    totalValue: number;
    lastValuationDate: Date;
  };

  // 거버넌스
  governance: TrustGovernance;

  // 문서화
  documentation: TrustDocumentation;

  // 상태
  status: TrustStatus;
  statusHistory: StatusChange[];

  // 메타데이터
  metadata: {
    createdAt: Date;
    updatedAt: Date;
    version: number;
    blockchainRef: BlockchainReference | null;
  };
}

enum TrustType {
  // 표준 신탁 유형
  REVOCABLE_LIVING = 'REVOCABLE_LIVING',   // 철회 가능 생전신탁
  IRREVOCABLE = 'IRREVOCABLE',             // 철회 불가능
  TESTAMENTARY = 'TESTAMENTARY',           // 유언 신탁

  // 냉동보존 전용
  PERSONAL_REVIVAL = 'PERSONAL_REVIVAL',   // 개인 소생 신탁
  PATIENT_CARE = 'PATIENT_CARE',           // 환자 케어 신탁
  DYNASTY = 'DYNASTY',                     // 왕조 신탁
  ASSET_PROTECTION = 'ASSET_PROTECTION',   // 자산 보호 신탁

  // 특수 목적
  LIFE_INSURANCE = 'LIFE_INSURANCE',       // 생명보험 신탁
}

// 신탁 당사자
interface TrustParties {
  grantor: TrustParty;              // 위탁자
  trustees: TrusteeInfo[];          // 수탁자
  successorTrustees: TrusteeInfo[]; // 후임 수탁자
  trustProtector: TrustParty | null; // 신탁 보호자
  beneficiaries: Beneficiary[];     // 수익자
  remainderBeneficiaries: Beneficiary[]; // 잔여 수익자
}

// 수익자
interface Beneficiary {
  beneficiaryType: BeneficiaryType;
  party: TrustParty | null;
  patientId: string | null;  // 냉동보존 수익자용
  interestType: 'INCOME' | 'PRINCIPAL' | 'BOTH' | 'CONTINGENT';
  share: number | null;
  conditions: BeneficiaryCondition[];
  distributionStandard: string;
}

enum BeneficiaryType {
  INDIVIDUAL = 'INDIVIDUAL',                     // 개인
  CLASS = 'CLASS',                               // 클래스
  CHARITY = 'CHARITY',                           // 자선단체
  PATIENT_UPON_REVIVAL = 'PATIENT_UPON_REVIVAL', // 소생 시 환자
  CRYONICS_ORGANIZATION = 'CRYONICS_ORGANIZATION', // 냉동보존 조직
}

// 소생 조항
interface RevivalProvisions {
  revivalDefinition: string;           // 소생 정의
  identityVerificationMethod: string;  // 신원 확인 방법
  revivalTriggerProcess: string;       // 소생 트리거 프로세스
  distributionUponRevival: DistributionProvision; // 소생 시 배분
  rehabilitationPeriod: number | null; // 재활 기간
  rehabilitationSupport: string;       // 재활 지원
  failedRevivalProvisions: string;     // 소생 실패 조항
}
```

---

## 3.3 포트폴리오 및 투자 스키마

### 포트폴리오 스키마

```typescript
// 환자 포트폴리오 구조
interface PatientPortfolio {
  id: string;
  patientId: string;
  organizationId: string;

  // 상태
  status: PortfolioStatus;

  // 핵심 펀드
  preservationFund: PreservationFund;  // 보존 펀드
  revivalFund: RevivalFund;            // 소생 펀드

  // 기타 자산
  personalAssets: AssetCategory[];
  trusts: TrustReference[];

  // 거버넌스
  governance: PortfolioGovernance;

  // 성과
  performance: PortfolioPerformance;

  // 평가
  totalValue: number;
  lastValuationDate: Date;
  valuationHistory: PortfolioValuation[];

  // 규정 준수
  compliance: ComplianceStatus;

  // 메타데이터
  metadata: {
    createdAt: Date;
    updatedAt: Date;
    nextReviewDate: Date;
  };
}

// 보존 펀드
interface PreservationFund {
  id: string;
  type: 'PATIENT_CARE_FUND';

  // 목표 및 실제
  targetAmount: number;
  currentAmount: number;
  fundingStatus: 'UNDERFUNDED' | 'ADEQUATE' | 'OVERFUNDED';

  // 구조
  structure: 'INTERNAL' | 'EXTERNAL_TRUST' | 'HYBRID';
  custodian: string;

  // 투자
  holdings: InvestmentHolding[];
  allocationPolicy: AllocationPolicy;
  currentAllocation: AssetAllocation;

  // 현금 흐름
  annualExpenses: number;
  projectedYearsOfFunding: number;
  lastDistribution: Date | null;
}

// 소생 펀드
interface RevivalFund {
  id: string;
  type: 'REVIVAL_FUND';

  // 목표 및 실제
  targetAmount: number | null;  // 오픈엔드일 수 있음
  currentAmount: number;
  projectedRevivalCost: CostProjection;

  // 구조
  structure: 'TRUST' | 'ACCOUNT' | 'INSURANCE' | 'MIXED';
  trustId: string | null;
  custodians: string[];

  // 투자
  holdings: InvestmentHolding[];
  allocationPolicy: AllocationPolicy;
  currentAllocation: AssetAllocation;

  // 성장 예측
  projectedGrowth: GrowthProjection[];
}

// 비용 예측
interface CostProjection {
  estimatedRevivalCost: number;       // 예상 소생 비용
  estimatedRehabilitationCost: number; // 예상 재활 비용
  estimatedLivingSupport: number;     // 예상 생활 지원
  totalProjectedNeed: number;         // 총 예상 필요액
  projectionBasis: string;            // 예측 근거
  projectionDate: Date;               // 예측 날짜
  confidenceInterval: { low: number; mid: number; high: number };
}

// 배분 정책
interface AllocationPolicy {
  id: string;
  name: string;
  type: 'STRATEGIC' | 'TACTICAL' | 'DYNAMIC';

  // 목표 배분
  targetAllocation: AllocationTarget[];

  // 제약
  constraints: AllocationConstraint[];

  // 리밸런싱
  rebalancingPolicy: RebalancingPolicy;

  // 리스크 파라미터
  riskTolerance: 'CONSERVATIVE' | 'MODERATE' | 'AGGRESSIVE';
  maxDrawdown: number;
  volatilityTarget: number | null;

  // 시간 지평
  timeHorizon: 'SHORT' | 'MEDIUM' | 'LONG' | 'PERPETUAL';
  timeHorizonYears: number | null;
}
```

---

## 3.4 거래 및 이벤트 스키마

### 거래 스키마

```typescript
// 금융 거래 표현
interface CryoTransaction {
  id: string;
  portfolioId: string;
  accountId: string;

  // 거래 유형
  type: TransactionType;
  subtype: string | null;

  // 타이밍
  transactionDate: Date;
  settlementDate: Date;
  recordedAt: Date;

  // 금액
  amount: TransactionAmount;
  fees: Fee[];
  taxes: Tax[];
  netAmount: number;

  // 증권 상세 (거래용)
  security: SecurityInfo | null;
  quantity: number | null;
  price: number | null;

  // 상대방
  counterparty: Counterparty | null;

  // 참조
  externalReference: string | null;
  parentTransactionId: string | null;

  // 상태
  status: TransactionStatus;

  // 블록체인
  blockchainRef: BlockchainReference | null;
}

enum TransactionType {
  // 현금 이동
  DEPOSIT = 'DEPOSIT',             // 입금
  WITHDRAWAL = 'WITHDRAWAL',       // 출금
  TRANSFER = 'TRANSFER',           // 이체

  // 거래
  BUY = 'BUY',                     // 매수
  SELL = 'SELL',                   // 매도

  // 수익
  DIVIDEND = 'DIVIDEND',           // 배당
  INTEREST = 'INTEREST',           // 이자
  DISTRIBUTION = 'DISTRIBUTION',   // 배분

  // 신탁 전용
  TRUST_FUNDING = 'TRUST_FUNDING',         // 신탁 자금 조달
  TRUST_DISTRIBUTION = 'TRUST_DISTRIBUTION', // 신탁 배분
  BENEFICIARY_PAYMENT = 'BENEFICIARY_PAYMENT', // 수익자 지급

  // 냉동보존 전용
  MEMBERSHIP_PAYMENT = 'MEMBERSHIP_PAYMENT',   // 회원 결제
  PRESERVATION_FEE = 'PRESERVATION_FEE',       // 보존 수수료
  REVIVAL_EXPENSE = 'REVIVAL_EXPENSE',         // 소생 비용
}
```

---

## 3.5 블록체인 데이터 구조

### 온체인 자산 등록

```typescript
// 자산 등록을 위한 블록체인 데이터 구조
interface BlockchainAssetRecord {
  // 온체인 데이터 (불변)
  assetHash: string;              // 자산 상세의 SHA-256
  registrationTimestamp: number;  // Unix 타임스탬프
  registrar: string;              // 지갑 주소
  organizationId: string;

  // 자산 분류 (온체인)
  assetCategory: number;          // 숫자로서의 열거형
  assetType: number;

  // 평가 앵커
  valuationHash: string;
  valuationTimestamp: number;

  // 오프체인 참조
  ipfsHash: string;               // IPFS의 전체 상세
  encryptionKeyId: string;        // IPFS 콘텐츠 복호화 키
}

// Solidity 컨트랙트 구조
const solidityContract = `
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.19;

contract CryoAssetRegistry {
    struct AssetRecord {
        bytes32 assetHash;
        uint256 registrationTimestamp;
        address registrar;
        bytes32 organizationId;
        uint8 assetCategory;
        uint16 assetType;
        bytes32 currentValuationHash;
        uint256 lastValuationTimestamp;
        string ipfsHash;
        bool isActive;
    }

    mapping(bytes32 => AssetRecord) public assets;
    mapping(bytes32 => ValuationHistory[]) public valuationHistory;

    event AssetRegistered(
        bytes32 indexed assetHash,
        bytes32 indexed organizationId,
        uint256 timestamp
    );

    event ValuationUpdated(
        bytes32 indexed assetHash,
        bytes32 valuationHash,
        uint256 timestamp
    );

    function registerAsset(
        bytes32 _assetHash,
        uint8 _category,
        uint16 _assetType,
        bytes32 _valuationHash,
        string memory _ipfsHash,
        bytes32 _organizationId
    ) external returns (bool) {
        require(assets[_assetHash].registrationTimestamp == 0, "자산 존재");

        assets[_assetHash] = AssetRecord({
            assetHash: _assetHash,
            registrationTimestamp: block.timestamp,
            registrar: msg.sender,
            organizationId: _organizationId,
            assetCategory: _category,
            assetType: _assetType,
            currentValuationHash: _valuationHash,
            lastValuationTimestamp: block.timestamp,
            ipfsHash: _ipfsHash,
            isActive: true
        });

        emit AssetRegistered(_assetHash, _organizationId, block.timestamp);
        return true;
    }

    function getAssetRecord(bytes32 _assetHash)
        external view returns (AssetRecord memory)
    {
        return assets[_assetHash];
    }
}
`;
```

---

## 3.6 데이터 검증 규칙

### 검증 스키마

```typescript
// 핵심 엔티티에 대한 검증 규칙
import { z } from 'zod';

// 환자 검증
const PatientSchema = z.object({
  id: z.string().uuid(),
  organizationPatientId: z.string().min(1).max(50),

  legalIdentity: z.object({
    fullLegalName: z.string().min(2).max(200),
    dateOfBirth: z.date().max(new Date()),
    citizenship: z.array(z.string().length(2)).min(1),  // ISO 3166-1 alpha-2
  }),

  preservationStatus: z.object({
    status: z.nativeEnum(PreservationStatus),
    preservationType: z.enum(['WHOLE_BODY', 'NEURO', 'BRAIN_ONLY']),
    preservationDate: z.date().nullable(),
  }),
});

// 자산 검증
const AssetSchema = z.object({
  id: z.string().uuid(),
  portfolioId: z.string().uuid(),
  patientId: z.string().uuid(),

  category: z.nativeEnum(AssetCategory),
  name: z.string().min(1).max(500),

  ownership: z.object({
    type: z.nativeEnum(OwnershipType),
    holders: z.array(z.object({
      entityType: z.enum(['INDIVIDUAL', 'TRUST', 'CORPORATION']),
      percentage: z.number().min(0).max(100),
    })).min(1),
  }).refine(
    (ownership) => {
      const totalPercentage = ownership.holders.reduce(
        (sum, h) => sum + h.percentage, 0
      );
      return Math.abs(totalPercentage - 100) < 0.01;
    },
    { message: '소유권 비율의 합은 100%여야 합니다' }
  ),

  currentValuation: z.object({
    value: z.number().nonnegative(),
    currency: z.string().length(3),  // ISO 4217
    valuationDate: z.date(),
  }),

  status: z.nativeEnum(AssetStatus),
});

// 커스텀 검증 함수
class DataValidator {
  validatePatient(data: unknown): ValidationResult<CryoPatient> {
    try {
      const parsed = PatientSchema.parse(data);
      return { valid: true, data: parsed as CryoPatient, errors: [] };
    } catch (error) {
      if (error instanceof z.ZodError) {
        return {
          valid: false,
          data: null,
          errors: error.errors.map(e => ({
            path: e.path.join('.'),
            message: e.message,
            code: e.code,
          })),
        };
      }
      throw error;
    }
  }

  // 교차 엔티티 검증
  validatePortfolioIntegrity(portfolio: PatientPortfolio): ValidationResult<void> {
    const errors: ValidationError[] = [];

    // 보존 펀드 적정성 확인
    if (portfolio.preservationFund.currentAmount <
        portfolio.preservationFund.targetAmount * 0.5) {
      errors.push({
        path: 'preservationFund.currentAmount',
        message: '보존 펀드가 목표의 50% 미만입니다',
        code: 'UNDERFUNDED',
      });
    }

    // 거버넌스 요구사항 확인
    if (!portfolio.governance.trustees ||
        portfolio.governance.trustees.length === 0) {
      errors.push({
        path: 'governance.trustees',
        message: '포트폴리오에는 최소 한 명의 수탁자가 있어야 합니다',
        code: 'MISSING_TRUSTEE',
      });
    }

    return {
      valid: errors.length === 0,
      data: undefined,
      errors,
    };
  }
}
```

---

## 장 요약

이 장에서는 냉동보존 자산 관리에 필요한 포괄적인 데이터 형식과 스키마를 정의했습니다:

1. **핵심 엔티티 스키마**: 환자 신원, 자산 표현 및 평가 구조
2. **신탁 스키마**: 냉동보존 전용 조항을 포함한 법인 표현
3. **포트폴리오 스키마**: 투자 보유, 배분 정책 및 성과 추적
4. **거래 스키마**: 금융 거래 및 이벤트 기록
5. **블록체인 구조**: 온체인 자산 등록 및 검증
6. **검증 규칙**: 데이터 무결성 및 일관성 검사

이러한 스키마는 냉동보존 보존에 필요한 확장된 시간 지평에 걸쳐 자산을 신뢰성 있게 추적하고 관리할 수 있는 상호 운용 가능한 냉동보존 자산 관리 시스템 구축의 기반을 제공합니다.

---

*다음 장: API 인터페이스 - Cryo-Asset 플랫폼을 위한 서비스 통합 사양*
