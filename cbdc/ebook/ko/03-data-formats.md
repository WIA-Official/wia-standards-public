# 제3장: CBDC 데이터 모델 및 토큰 표준

## 중앙은행 디지털 화폐 시스템을 위한 종합 데이터 아키텍처

### 3.1 핵심 데이터 모델 아키텍처

WIA-CBDC 데이터 모델은 다양한 CBDC 구현 전반에서 디지털 화폐, 계좌, 거래 및 관련 엔티티를 표현하기 위한 표준화된 기반을 제공합니다.

```typescript
// 핵심 CBDC 데이터 모델
interface WIACBDCDataModel {
  version: '1.0.0';
  namespace: 'wia.cbdc';

  coreEntities: {
    currency: CBDCCurrencyDefinition;  // 통화 정의
    token: CBDCToken;                   // 토큰
    account: CBDCAccount;               // 계좌
    wallet: CBDCWallet;                 // 지갑
    transaction: CBDCTransaction;       // 거래
    participant: CBDCParticipant;       // 참여자
  };

  supportingEntities: {
    identity: DigitalIdentity;          // 디지털 신원
    policy: MonetaryPolicy;             // 통화 정책
    compliance: ComplianceRecord;       // 컴플라이언스 기록
    audit: AuditTrail;                  // 감사 추적
  };
}

// 통화 정의 스키마
interface CBDCCurrencyDefinition {
  // ISO 4217 준수 통화 식별
  currencyCode: string;           // 예: "KRW", "USD", "EUR"
  digitalCurrencyCode: string;    // 예: "eKRW", "eUSD", "e-EUR"
  currencyName: string;
  digitalCurrencyName: string;

  // 발행 기관
  issuer: {
    centralBankId: string;
    centralBankName: string;
    countryCode: string;          // ISO 3166-1 alpha-2
    jurisdiction: string[];
  };

  // 통화 속성
  properties: {
    decimalPlaces: number;        // 일반적으로 0 (원), 2 (달러)
    smallestUnit: number;         // 예: 1 (원), 0.01 (센트)
    symbol: string;               // 예: "₩", "$", "€"
    displayFormat: string;        // 예: "{symbol}{amount}"
  };

  // 화폐 특성
  monetaryProperties: {
    legalTender: boolean;         // 법정 화폐 여부
    interestBearing: boolean;     // 이자 지급 여부
    interestRate?: number;        // 기준점 (basis points)
    holdingLimit?: MonetaryAmount;
    transactionLimit?: MonetaryLimit;
  };

  // 기술적 속성
  technicalProperties: {
    tokenStandard: TokenStandard;
    programmable: boolean;
    offlineCapable: boolean;
    expirationSupport: boolean;
  };

  // 생명주기
  lifecycle: {
    createdAt: ISO8601DateTime;
    effectiveFrom: ISO8601DateTime;
    status: 'DRAFT' | 'ACTIVE' | 'SUSPENDED' | 'RETIRED';
  };
}

// 화폐 금액 (정밀도 처리 포함)
interface MonetaryAmount {
  value: string;                  // 정확한 소수 표현을 위한 문자열
  currency: string;               // 통화 코드
  precision: number;              // 소수점 자릿수

  // 계산된 속성
  readonly valueInSmallestUnit: bigint;
  readonly displayValue: string;
}

class MonetaryAmountImpl implements MonetaryAmount {
  constructor(
    public value: string,
    public currency: string,
    public precision: number = 0  // KRW는 소수점 없음
  ) {}

  get valueInSmallestUnit(): bigint {
    const multiplier = BigInt(10 ** this.precision);
    const [whole, decimal = '0'] = this.value.split('.');
    const paddedDecimal = decimal.padEnd(this.precision, '0').slice(0, this.precision);
    return BigInt(whole) * multiplier + BigInt(paddedDecimal);
  }

  get displayValue(): string {
    return new Intl.NumberFormat('ko-KR', {
      style: 'currency',
      currency: this.currency,
      minimumFractionDigits: this.precision,
      maximumFractionDigits: this.precision
    }).format(parseFloat(this.value));
  }

  add(other: MonetaryAmount): MonetaryAmountImpl {
    if (this.currency !== other.currency) {
      throw new Error('다른 통화는 더할 수 없습니다');
    }
    const sum = this.valueInSmallestUnit + other.valueInSmallestUnit;
    return MonetaryAmountImpl.fromSmallestUnit(sum, this.currency, this.precision);
  }

  subtract(other: MonetaryAmount): MonetaryAmountImpl {
    if (this.currency !== other.currency) {
      throw new Error('다른 통화는 뺄 수 없습니다');
    }
    const diff = this.valueInSmallestUnit - other.valueInSmallestUnit;
    return MonetaryAmountImpl.fromSmallestUnit(diff, this.currency, this.precision);
  }
}
```

### 3.2 토큰 데이터 구조

```typescript
// CBDC 토큰 스키마
interface CBDCToken {
  // 토큰 식별
  tokenId: string;                // 전역 고유 식별자
  serialNumber: string;           // 사람이 읽을 수 있는 일련번호
  version: number;                // 업그레이드를 위한 토큰 버전

  // 가치 표현
  denomination: MonetaryAmount;
  tokenType: TokenType;

  // 소유권
  ownership: {
    currentHolder: string;        // 지갑/계좌 ID
    ownershipHistory: OwnershipRecord[];
    transferRestrictions?: TransferRestriction[];
  };

  // 토큰 상태
  state: {
    status: TokenStatus;
    issuedAt: ISO8601DateTime;
    lastTransferAt?: ISO8601DateTime;
    expiresAt?: ISO8601DateTime;
    lockedUntil?: ISO8601DateTime;
    lockReason?: string;
  };

  // 프로그래머빌리티
  conditions?: TokenCondition[];

  // 암호화 속성
  cryptography: {
    signatureAlgorithm: string;
    issuerSignature: string;
    currentOwnerSignature?: string;
    proofOfOwnership: string;
  };

  // 메타데이터
  metadata: {
    issuanceBatch: string;
    mintingNode: string;
    offlineCapable: boolean;
    privacyLevel: PrivacyLevel;
  };
}

enum TokenType {
  FUNGIBLE = 'FUNGIBLE',              // 표준 화폐 토큰
  NON_FUNGIBLE = 'NON_FUNGIBLE',      // 고유 토큰 (드문 사용)
  SEMI_FUNGIBLE = 'SEMI_FUNGIBLE',    // 조건부 배치
  PURPOSE_BOUND = 'PURPOSE_BOUND'      // 제한 사용 토큰
}

enum TokenStatus {
  ACTIVE = 'ACTIVE',          // 활성
  PENDING = 'PENDING',        // 대기 중
  LOCKED = 'LOCKED',          // 잠김
  EXPIRED = 'EXPIRED',        // 만료됨
  REDEEMED = 'REDEEMED',      // 상환됨
  DESTROYED = 'DESTROYED'     // 파기됨
}

enum PrivacyLevel {
  TRANSPARENT = 'TRANSPARENT',        // 완전 거래 가시성
  PSEUDONYMOUS = 'PSEUDONYMOUS',      // 주소 기반 프라이버시
  CONFIDENTIAL = 'CONFIDENTIAL',      // 금액 숨김
  ANONYMOUS = 'ANONYMOUS'             // 완전 프라이버시 (제한적 사용)
}

// 프로그래머블 머니를 위한 토큰 조건
interface TokenCondition {
  conditionId: string;
  conditionType: ConditionType;
  parameters: Record<string, any>;
  enforced: boolean;
  validFrom?: ISO8601DateTime;
  validUntil?: ISO8601DateTime;
}

enum ConditionType {
  // 시간 기반 조건
  EXPIRATION = 'EXPIRATION',              // 만료
  TIME_LOCK = 'TIME_LOCK',                // 시간 잠금
  VESTING = 'VESTING',                    // 베스팅

  // 사용 조건
  MERCHANT_CATEGORY = 'MERCHANT_CATEGORY', // 가맹점 카테고리
  GEOGRAPHIC_RESTRICTION = 'GEOGRAPHIC_RESTRICTION', // 지역 제한
  PURPOSE_RESTRICTION = 'PURPOSE_RESTRICTION', // 목적 제한

  // 가치 조건
  MINIMUM_SPEND = 'MINIMUM_SPEND',        // 최소 지출
  MAXIMUM_SPEND = 'MAXIMUM_SPEND',        // 최대 지출
  VELOCITY_LIMIT = 'VELOCITY_LIMIT',      // 속도 제한

  // 신원 조건
  KYC_LEVEL_REQUIRED = 'KYC_LEVEL_REQUIRED', // KYC 수준 필요
  AGE_VERIFICATION = 'AGE_VERIFICATION',  // 연령 확인

  // 사용자 정의 조건
  SMART_CONTRACT = 'SMART_CONTRACT'       // 스마트 계약
}

// 토큰 JSON 스키마
const CBDCTokenSchema = {
  $schema: 'http://json-schema.org/draft-07/schema#',
  $id: 'https://wia.org/schemas/cbdc/token/v1',
  title: 'WIA-CBDC 토큰',
  type: 'object',
  required: ['tokenId', 'denomination', 'ownership', 'state', 'cryptography'],
  properties: {
    tokenId: {
      type: 'string',
      format: 'uuid',
      description: '전역 고유 토큰 식별자'
    },
    serialNumber: {
      type: 'string',
      pattern: '^[A-Z0-9]{16,32}$',
      description: '사람이 읽을 수 있는 일련번호'
    },
    denomination: {
      $ref: '#/definitions/MonetaryAmount'
    },
    tokenType: {
      type: 'string',
      enum: ['FUNGIBLE', 'NON_FUNGIBLE', 'SEMI_FUNGIBLE', 'PURPOSE_BOUND']
    }
  }
};
```

### 3.3 계좌 및 지갑 모델

```typescript
// CBDC 계좌 모델
interface CBDCAccount {
  // 계좌 식별
  accountId: string;              // 내부 고유 식별자
  accountNumber: string;          // 외부 계좌 번호
  accountType: AccountType;

  // 계좌 보유자
  holder: {
    participantId: string;
    participantType: ParticipantType;
    identityVerificationLevel: KYCLevel;
  };

  // 잔액 정보
  balances: {
    available: MonetaryAmount;    // 사용 가능
    pending: MonetaryAmount;      // 대기 중
    locked: MonetaryAmount;       // 잠김
    total: MonetaryAmount;        // 총계
  };

  // 계좌 한도
  limits: {
    holdingLimit: MonetaryAmount;           // 보유 한도
    dailyTransactionLimit: MonetaryAmount;  // 일일 거래 한도
    singleTransactionLimit: MonetaryAmount; // 단일 거래 한도
    monthlyTransactionLimit: MonetaryAmount; // 월간 거래 한도
  };

  // 계좌 상태
  status: {
    state: AccountState;
    kycStatus: KYCStatus;
    amlStatus: AMLStatus;
    lastActivityAt: ISO8601DateTime;
    createdAt: ISO8601DateTime;
  };

  // 연결된 엔티티
  linkedWallets: string[];
  linkedBankAccounts?: BankAccountLink[];

  // 설정
  settings: {
    defaultCurrency: string;
    notificationPreferences: NotificationPreferences;
    privacySettings: PrivacySettings;
  };
}

enum AccountType {
  PERSONAL = 'PERSONAL',          // 개인
  BUSINESS = 'BUSINESS',          // 사업자
  GOVERNMENT = 'GOVERNMENT',      // 정부
  INSTITUTIONAL = 'INSTITUTIONAL', // 기관
  RESERVE = 'RESERVE'             // 준비금
}

enum ParticipantType {
  INDIVIDUAL = 'INDIVIDUAL',                  // 개인
  SOLE_PROPRIETOR = 'SOLE_PROPRIETOR',        // 개인사업자
  CORPORATION = 'CORPORATION',                // 법인
  GOVERNMENT_ENTITY = 'GOVERNMENT_ENTITY',    // 정부 기관
  FINANCIAL_INSTITUTION = 'FINANCIAL_INSTITUTION', // 금융기관
  CENTRAL_BANK = 'CENTRAL_BANK'               // 중앙은행
}

enum KYCLevel {
  NONE = 'NONE',                  // 미인증
  BASIC = 'BASIC',                // 전화/이메일만
  STANDARD = 'STANDARD',          // 신분증 확인
  ENHANCED = 'ENHANCED',          // 전체 KYC + 주소
  INSTITUTIONAL = 'INSTITUTIONAL' // 기업 KYC
}

enum AccountState {
  PENDING_ACTIVATION = 'PENDING_ACTIVATION', // 활성화 대기
  ACTIVE = 'ACTIVE',                         // 활성
  SUSPENDED = 'SUSPENDED',                   // 정지
  FROZEN = 'FROZEN',                         // 동결
  CLOSED = 'CLOSED'                          // 폐쇄
}

// CBDC 지갑 모델
interface CBDCWallet {
  walletId: string;
  walletType: WalletType;
  accountId: string;              // 상위 계좌

  // 지갑 식별
  walletAddress: string;          // 공개 주소
  publicKey: string;

  // 지갑 기능
  capabilities: {
    canSend: boolean;             // 송금 가능
    canReceive: boolean;          // 수신 가능
    canOffline: boolean;          // 오프라인 가능
    programmableSupport: boolean; // 프로그래머블 지원
    maxTokens: number;
  };

  // 기기 바인딩 (모바일/하드웨어 지갑용)
  deviceBinding?: {
    deviceId: string;
    deviceType: DeviceType;
    deviceFingerprint: string;
    bindingCreatedAt: ISO8601DateTime;
    lastUsedAt: ISO8601DateTime;
  };

  // 보안
  security: {
    multiSigRequired: boolean;
    multiSigThreshold?: number;
    biometricEnabled: boolean;
    pinEnabled: boolean;
    spendingPassword: boolean;
  };

  // 토큰 보유
  tokens: TokenHolding[];

  // 오프라인 잔액 (오프라인 가능 지갑용)
  offlineState?: {
    offlineBalance: MonetaryAmount;
    lastSyncAt: ISO8601DateTime;
    pendingOfflineTransactions: OfflineTransaction[];
  };
}

enum WalletType {
  MOBILE = 'MOBILE',              // 모바일
  WEB = 'WEB',                    // 웹
  HARDWARE = 'HARDWARE',          // 하드웨어
  CUSTODIAL = 'CUSTODIAL',        // 수탁형
  SMART_CONTRACT = 'SMART_CONTRACT' // 스마트 계약
}

enum DeviceType {
  SMARTPHONE = 'SMARTPHONE',      // 스마트폰
  TABLET = 'TABLET',              // 태블릿
  SMARTWATCH = 'SMARTWATCH',      // 스마트워치
  HARDWARE_WALLET = 'HARDWARE_WALLET', // 하드웨어 지갑
  SIM_CARD = 'SIM_CARD',          // SIM 카드
  SECURE_ELEMENT = 'SECURE_ELEMENT'    // 보안 요소
}
```

### 3.4 거래 데이터 모델

```typescript
// CBDC 거래 모델
interface CBDCTransaction {
  // 거래 식별
  transactionId: string;          // UUID
  referenceNumber: string;        // 사람이 읽을 수 있는 참조 번호
  correlationId?: string;         // 관련 거래용

  // 거래 유형
  type: TransactionType;
  subType?: TransactionSubType;

  // 당사자
  parties: {
    initiator: TransactionParty;
    sender?: TransactionParty;
    receiver?: TransactionParty;
    intermediaries?: TransactionParty[];
  };

  // 가치
  amount: MonetaryAmount;
  fees?: TransactionFee[];

  // 타이밍
  timing: {
    initiatedAt: ISO8601DateTime;
    submittedAt: ISO8601DateTime;
    processedAt?: ISO8601DateTime;
    settledAt?: ISO8601DateTime;
    expiresAt?: ISO8601DateTime;
  };

  // 상태
  status: {
    current: TransactionStatus;
    history: StatusChange[];
  };

  // 실행 세부 정보
  execution: {
    channel: TransactionChannel;
    method: PaymentMethod;
    deviceInfo?: DeviceInfo;
    locationInfo?: LocationInfo;
  };

  // 컴플라이언스
  compliance: {
    amlScreeningResult: ScreeningResult;
    sanctionsCheckResult: ScreeningResult;
    riskScore: number;
    flags: ComplianceFlag[];
  };

  // 기술적 세부 정보
  technical: {
    ledgerEntries: LedgerEntry[];
    signatures: TransactionSignature[];
    proofs?: CryptographicProof[];
    blockReference?: BlockReference;
  };

  // 추가 데이터
  metadata?: {
    description?: string;
    merchantCategory?: string;
    invoiceReference?: string;
    customFields?: Record<string, any>;
  };
}

enum TransactionType {
  // 생명주기 거래
  ISSUANCE = 'ISSUANCE',          // 발행
  REDEMPTION = 'REDEMPTION',      // 상환

  // 전송 거래
  TRANSFER = 'TRANSFER',          // 전송
  PAYMENT = 'PAYMENT',            // 결제

  // 계좌 작업
  DEPOSIT = 'DEPOSIT',            // 입금
  WITHDRAWAL = 'WITHDRAWAL',      // 출금

  // 특수 거래
  FX_CONVERSION = 'FX_CONVERSION', // 외환 전환
  CROSS_BORDER = 'CROSS_BORDER',  // 국경간

  // 프로그래머블 거래
  CONDITIONAL_PAYMENT = 'CONDITIONAL_PAYMENT', // 조건부 결제
  ESCROW = 'ESCROW',              // 에스크로

  // 관리
  ADJUSTMENT = 'ADJUSTMENT',      // 조정
  REVERSAL = 'REVERSAL',          // 취소
  FEE = 'FEE'                     // 수수료
}

enum TransactionSubType {
  // 전송 하위 유형
  P2P = 'P2P',                    // 개인간
  P2M = 'P2M',                    // 개인-가맹점
  P2G = 'P2G',                    // 개인-정부
  G2P = 'G2P',                    // 정부-개인
  B2B = 'B2B',                    // 기업간

  // 결제 하위 유형
  RETAIL_PAYMENT = 'RETAIL_PAYMENT',   // 소매 결제
  BILL_PAYMENT = 'BILL_PAYMENT',       // 공과금 납부
  SALARY = 'SALARY',                   // 급여
  PENSION = 'PENSION',                 // 연금
  SUBSIDY = 'SUBSIDY',                 // 보조금
  TAX_PAYMENT = 'TAX_PAYMENT',         // 세금 납부

  // 국경간 하위 유형
  REMITTANCE = 'REMITTANCE',           // 송금
  TRADE_SETTLEMENT = 'TRADE_SETTLEMENT', // 무역 결제

  // 오프라인
  OFFLINE_TRANSFER = 'OFFLINE_TRANSFER' // 오프라인 전송
}

enum TransactionStatus {
  INITIATED = 'INITIATED',                     // 시작됨
  PENDING_AUTHORIZATION = 'PENDING_AUTHORIZATION', // 승인 대기
  AUTHORIZED = 'AUTHORIZED',                   // 승인됨
  PENDING_SCREENING = 'PENDING_SCREENING',     // 심사 대기
  SCREENED = 'SCREENED',                       // 심사됨
  PENDING_EXECUTION = 'PENDING_EXECUTION',     // 실행 대기
  EXECUTING = 'EXECUTING',                     // 실행 중
  PENDING_SETTLEMENT = 'PENDING_SETTLEMENT',   // 결제 대기
  SETTLED = 'SETTLED',                         // 결제됨
  COMPLETED = 'COMPLETED',                     // 완료됨
  FAILED = 'FAILED',                           // 실패
  CANCELLED = 'CANCELLED',                     // 취소됨
  REVERSED = 'REVERSED',                       // 취소됨
  EXPIRED = 'EXPIRED'                          // 만료됨
}
```

### 3.5 데이터 검증 및 무결성

```typescript
// 데이터 검증 엔진
class CBDCDataValidator {
  private schemas: Map<string, JSONSchema>;
  private customValidators: Map<string, CustomValidator>;

  async validateToken(token: CBDCToken): Promise<ValidationResult> {
    const results: ValidationError[] = [];

    // 스키마 검증
    const schemaResult = this.validateAgainstSchema(token, 'CBDCToken');
    results.push(...schemaResult.errors);

    // 비즈니스 규칙 검증
    results.push(...this.validateTokenBusinessRules(token));

    // 암호화 검증
    results.push(...await this.validateTokenCryptography(token));

    return {
      valid: results.length === 0,
      errors: results,
      warnings: this.getWarnings(token)
    };
  }

  private validateTokenBusinessRules(token: CBDCToken): ValidationError[] {
    const errors: ValidationError[] = [];

    // 액면가 유효성 확인
    if (parseFloat(token.denomination.value) <= 0) {
      errors.push({
        field: 'denomination.value',
        message: '토큰 액면가는 양수여야 합니다',
        code: 'INVALID_DENOMINATION'
      });
    }

    // 만료 확인
    if (token.state.expiresAt) {
      const expirationDate = new Date(token.state.expiresAt);
      if (expirationDate <= new Date()) {
        errors.push({
          field: 'state.expiresAt',
          message: '토큰이 만료되었습니다',
          code: 'TOKEN_EXPIRED'
        });
      }
    }

    // 잠금 상태 확인
    if (token.state.lockedUntil) {
      const lockDate = new Date(token.state.lockedUntil);
      if (lockDate > new Date() && token.state.status === TokenStatus.ACTIVE) {
        errors.push({
          field: 'state.status',
          message: '잠긴 토큰은 ACTIVE 상태가 아니어야 합니다',
          code: 'INVALID_LOCK_STATUS'
        });
      }
    }

    return errors;
  }

  async validateTransaction(
    transaction: CBDCTransaction
  ): Promise<ValidationResult> {
    const results: ValidationError[] = [];

    // 스키마 검증
    const schemaResult = this.validateAgainstSchema(transaction, 'CBDCTransaction');
    results.push(...schemaResult.errors);

    // 당사자 검증
    results.push(...this.validateTransactionParties(transaction));

    // 금액 검증
    results.push(...this.validateTransactionAmount(transaction));

    // 서명 검증
    results.push(...await this.validateTransactionSignatures(transaction));

    // 이중 지출 확인
    results.push(...await this.checkDoubleSpend(transaction));

    return {
      valid: results.length === 0,
      errors: results,
      warnings: []
    };
  }

  private async checkDoubleSpend(
    transaction: CBDCTransaction
  ): Promise<ValidationError[]> {
    const errors: ValidationError[] = [];
    const tokenIds = this.extractTokenIds(transaction);

    for (const tokenId of tokenIds) {
      const token = await this.tokenRepository.findById(tokenId);

      if (!token) {
        errors.push({
          field: 'tokens',
          message: `토큰 ${tokenId}을(를) 찾을 수 없습니다`,
          code: 'TOKEN_NOT_FOUND'
        });
        continue;
      }

      if (token.state.status !== TokenStatus.ACTIVE) {
        errors.push({
          field: 'tokens',
          message: `토큰 ${tokenId}이(가) 활성 상태가 아닙니다 (상태: ${token.state.status})`,
          code: 'TOKEN_NOT_ACTIVE'
        });
      }

      // 소유권 확인
      if (token.ownership.currentHolder !== transaction.parties.sender?.walletId) {
        errors.push({
          field: 'tokens',
          message: `발신자가 토큰 ${tokenId}의 소유자가 아닙니다`,
          code: 'OWNERSHIP_MISMATCH'
        });
      }
    }

    return errors;
  }
}

interface ValidationResult {
  valid: boolean;
  errors: ValidationError[];
  warnings: ValidationWarning[];
}

interface ValidationError {
  field: string;
  message: string;
  code: string;
}
```

### 3.6 요약

WIA-CBDC 데이터 모델이 제공하는 것:

1. **종합적인 토큰 모델**: 프로그래머빌리티를 갖춘 전체 생명주기 지원
2. **유연한 계좌 구조**: KYC 수준에 따른 다계층 계좌
3. **풍부한 거래 모델**: 모든 거래 유형 지원
4. **국경간 지원**: 다자간 CBDC 거래 처리
5. **강력한 검증**: 스키마 및 비즈니스 규칙 시행

---

**WIA-CBDC 데이터 모델**
**버전**: 1.0.0
**최종 업데이트**: 2025

© 2025 WIA (World Interoperability Alliance)
