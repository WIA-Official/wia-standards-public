# Chapter 3: CBDC Data Models and Token Standards

## Comprehensive Data Architecture for Central Bank Digital Currency Systems

### 3.1 Core Data Model Architecture

The WIA-CBDC data model provides a standardized foundation for representing digital currency, accounts, transactions, and related entities across diverse CBDC implementations.

```typescript
// Core CBDC Data Model
interface WIACBDCDataModel {
  version: '1.0.0';
  namespace: 'wia.cbdc';

  coreEntities: {
    currency: CBDCCurrencyDefinition;
    token: CBDCToken;
    account: CBDCAccount;
    wallet: CBDCWallet;
    transaction: CBDCTransaction;
    participant: CBDCParticipant;
  };

  supportingEntities: {
    identity: DigitalIdentity;
    policy: MonetaryPolicy;
    compliance: ComplianceRecord;
    audit: AuditTrail;
  };
}

// Currency Definition Schema
interface CBDCCurrencyDefinition {
  // ISO 4217 compliant currency identification
  currencyCode: string;           // e.g., "USD", "EUR", "CNY"
  digitalCurrencyCode: string;    // e.g., "eUSD", "DCEP", "e-EUR"
  currencyName: string;
  digitalCurrencyName: string;

  // Issuing authority
  issuer: {
    centralBankId: string;
    centralBankName: string;
    countryCode: string;          // ISO 3166-1 alpha-2
    jurisdiction: string[];
  };

  // Currency properties
  properties: {
    decimalPlaces: number;        // Typically 2, some 0 or 3
    smallestUnit: number;         // e.g., 0.01 for cents
    symbol: string;               // e.g., "$", "€", "¥"
    displayFormat: string;        // e.g., "{symbol}{amount}"
  };

  // Monetary characteristics
  monetaryProperties: {
    legalTender: boolean;
    interestBearing: boolean;
    interestRate?: number;        // Basis points
    holdingLimit?: MonetaryAmount;
    transactionLimit?: MonetaryLimit;
  };

  // Technical properties
  technicalProperties: {
    tokenStandard: TokenStandard;
    programmable: boolean;
    offlineCapable: boolean;
    expirationSupport: boolean;
  };

  // Lifecycle
  lifecycle: {
    createdAt: ISO8601DateTime;
    effectiveFrom: ISO8601DateTime;
    status: 'DRAFT' | 'ACTIVE' | 'SUSPENDED' | 'RETIRED';
  };
}

// Token Standard Definition
type TokenStandard =
  | 'WIA-CBDC-TOKEN-V1'
  | 'ISO-24165-COMPLIANT'
  | 'CUSTOM';

// Monetary Amount with precision handling
interface MonetaryAmount {
  value: string;                  // String for precise decimal representation
  currency: string;               // Currency code
  precision: number;              // Decimal places

  // Computed properties
  readonly valueInSmallestUnit: bigint;
  readonly displayValue: string;
}

class MonetaryAmountImpl implements MonetaryAmount {
  constructor(
    public value: string,
    public currency: string,
    public precision: number = 2
  ) {}

  get valueInSmallestUnit(): bigint {
    const multiplier = BigInt(10 ** this.precision);
    const [whole, decimal = '0'] = this.value.split('.');
    const paddedDecimal = decimal.padEnd(this.precision, '0').slice(0, this.precision);
    return BigInt(whole) * multiplier + BigInt(paddedDecimal);
  }

  get displayValue(): string {
    return new Intl.NumberFormat('en-US', {
      style: 'currency',
      currency: this.currency,
      minimumFractionDigits: this.precision,
      maximumFractionDigits: this.precision
    }).format(parseFloat(this.value));
  }

  static fromSmallestUnit(
    value: bigint,
    currency: string,
    precision: number = 2
  ): MonetaryAmountImpl {
    const divisor = BigInt(10 ** precision);
    const whole = value / divisor;
    const decimal = value % divisor;
    const decimalStr = decimal.toString().padStart(precision, '0');
    return new MonetaryAmountImpl(
      `${whole}.${decimalStr}`,
      currency,
      precision
    );
  }

  add(other: MonetaryAmount): MonetaryAmountImpl {
    if (this.currency !== other.currency) {
      throw new Error('Cannot add different currencies');
    }
    const sum = this.valueInSmallestUnit + other.valueInSmallestUnit;
    return MonetaryAmountImpl.fromSmallestUnit(sum, this.currency, this.precision);
  }

  subtract(other: MonetaryAmount): MonetaryAmountImpl {
    if (this.currency !== other.currency) {
      throw new Error('Cannot subtract different currencies');
    }
    const diff = this.valueInSmallestUnit - other.valueInSmallestUnit;
    return MonetaryAmountImpl.fromSmallestUnit(diff, this.currency, this.precision);
  }
}
```

### 3.2 Token Data Structure

```typescript
// CBDC Token Schema
interface CBDCToken {
  // Token identification
  tokenId: string;                // Globally unique identifier
  serialNumber: string;           // Human-readable serial
  version: number;                // Token version for upgrades

  // Value representation
  denomination: MonetaryAmount;
  tokenType: TokenType;

  // Ownership
  ownership: {
    currentHolder: string;        // Wallet/Account ID
    ownershipHistory: OwnershipRecord[];
    transferRestrictions?: TransferRestriction[];
  };

  // Token state
  state: {
    status: TokenStatus;
    issuedAt: ISO8601DateTime;
    lastTransferAt?: ISO8601DateTime;
    expiresAt?: ISO8601DateTime;
    lockedUntil?: ISO8601DateTime;
    lockReason?: string;
  };

  // Programmability
  conditions?: TokenCondition[];

  // Cryptographic properties
  cryptography: {
    signatureAlgorithm: string;
    issuerSignature: string;
    currentOwnerSignature?: string;
    proofOfOwnership: string;
  };

  // Metadata
  metadata: {
    issuanceBatch: string;
    mintingNode: string;
    offlineCapable: boolean;
    privacyLevel: PrivacyLevel;
  };
}

enum TokenType {
  FUNGIBLE = 'FUNGIBLE',              // Standard currency token
  NON_FUNGIBLE = 'NON_FUNGIBLE',      // Unique token (rare use)
  SEMI_FUNGIBLE = 'SEMI_FUNGIBLE',    // Batched with conditions
  PURPOSE_BOUND = 'PURPOSE_BOUND'      // Restricted use token
}

enum TokenStatus {
  ACTIVE = 'ACTIVE',
  PENDING = 'PENDING',
  LOCKED = 'LOCKED',
  EXPIRED = 'EXPIRED',
  REDEEMED = 'REDEEMED',
  DESTROYED = 'DESTROYED'
}

enum PrivacyLevel {
  TRANSPARENT = 'TRANSPARENT',        // Full transaction visibility
  PSEUDONYMOUS = 'PSEUDONYMOUS',      // Address-based privacy
  CONFIDENTIAL = 'CONFIDENTIAL',      // Amount hidden
  ANONYMOUS = 'ANONYMOUS'             // Full privacy (limited use)
}

// Token Condition for Programmable Money
interface TokenCondition {
  conditionId: string;
  conditionType: ConditionType;
  parameters: Record<string, any>;
  enforced: boolean;
  validFrom?: ISO8601DateTime;
  validUntil?: ISO8601DateTime;
}

enum ConditionType {
  // Time-based conditions
  EXPIRATION = 'EXPIRATION',
  TIME_LOCK = 'TIME_LOCK',
  VESTING = 'VESTING',

  // Usage conditions
  MERCHANT_CATEGORY = 'MERCHANT_CATEGORY',
  GEOGRAPHIC_RESTRICTION = 'GEOGRAPHIC_RESTRICTION',
  PURPOSE_RESTRICTION = 'PURPOSE_RESTRICTION',

  // Value conditions
  MINIMUM_SPEND = 'MINIMUM_SPEND',
  MAXIMUM_SPEND = 'MAXIMUM_SPEND',
  VELOCITY_LIMIT = 'VELOCITY_LIMIT',

  // Identity conditions
  KYC_LEVEL_REQUIRED = 'KYC_LEVEL_REQUIRED',
  AGE_VERIFICATION = 'AGE_VERIFICATION',

  // Custom conditions
  SMART_CONTRACT = 'SMART_CONTRACT'
}

// JSON Schema for Token
const CBDCTokenSchema = {
  $schema: 'http://json-schema.org/draft-07/schema#',
  $id: 'https://wia.org/schemas/cbdc/token/v1',
  title: 'WIA-CBDC Token',
  type: 'object',
  required: ['tokenId', 'denomination', 'ownership', 'state', 'cryptography'],
  properties: {
    tokenId: {
      type: 'string',
      format: 'uuid',
      description: 'Globally unique token identifier'
    },
    serialNumber: {
      type: 'string',
      pattern: '^[A-Z0-9]{16,32}$',
      description: 'Human-readable serial number'
    },
    denomination: {
      $ref: '#/definitions/MonetaryAmount'
    },
    tokenType: {
      type: 'string',
      enum: ['FUNGIBLE', 'NON_FUNGIBLE', 'SEMI_FUNGIBLE', 'PURPOSE_BOUND']
    },
    ownership: {
      type: 'object',
      required: ['currentHolder'],
      properties: {
        currentHolder: { type: 'string' },
        ownershipHistory: {
          type: 'array',
          items: { $ref: '#/definitions/OwnershipRecord' }
        }
      }
    },
    state: {
      type: 'object',
      required: ['status', 'issuedAt'],
      properties: {
        status: {
          type: 'string',
          enum: ['ACTIVE', 'PENDING', 'LOCKED', 'EXPIRED', 'REDEEMED', 'DESTROYED']
        },
        issuedAt: { type: 'string', format: 'date-time' },
        expiresAt: { type: 'string', format: 'date-time' }
      }
    },
    conditions: {
      type: 'array',
      items: { $ref: '#/definitions/TokenCondition' }
    },
    cryptography: {
      type: 'object',
      required: ['signatureAlgorithm', 'issuerSignature'],
      properties: {
        signatureAlgorithm: { type: 'string' },
        issuerSignature: { type: 'string' },
        proofOfOwnership: { type: 'string' }
      }
    }
  },
  definitions: {
    MonetaryAmount: {
      type: 'object',
      required: ['value', 'currency'],
      properties: {
        value: { type: 'string', pattern: '^-?\\d+(\\.\\d+)?$' },
        currency: { type: 'string', minLength: 3, maxLength: 4 },
        precision: { type: 'integer', minimum: 0, maximum: 18 }
      }
    },
    OwnershipRecord: {
      type: 'object',
      properties: {
        holder: { type: 'string' },
        acquiredAt: { type: 'string', format: 'date-time' },
        transferredAt: { type: 'string', format: 'date-time' },
        transactionId: { type: 'string' }
      }
    },
    TokenCondition: {
      type: 'object',
      required: ['conditionId', 'conditionType'],
      properties: {
        conditionId: { type: 'string' },
        conditionType: { type: 'string' },
        parameters: { type: 'object' },
        enforced: { type: 'boolean' }
      }
    }
  }
};
```

### 3.3 Account and Wallet Models

```typescript
// CBDC Account Model
interface CBDCAccount {
  // Account identification
  accountId: string;              // Internal unique identifier
  accountNumber: string;          // External account number
  accountType: AccountType;

  // Account holder
  holder: {
    participantId: string;
    participantType: ParticipantType;
    identityVerificationLevel: KYCLevel;
  };

  // Balance information
  balances: {
    available: MonetaryAmount;
    pending: MonetaryAmount;
    locked: MonetaryAmount;
    total: MonetaryAmount;
  };

  // Account limits
  limits: {
    holdingLimit: MonetaryAmount;
    dailyTransactionLimit: MonetaryAmount;
    singleTransactionLimit: MonetaryAmount;
    monthlyTransactionLimit: MonetaryAmount;
  };

  // Account status
  status: {
    state: AccountState;
    kycStatus: KYCStatus;
    amlStatus: AMLStatus;
    lastActivityAt: ISO8601DateTime;
    createdAt: ISO8601DateTime;
  };

  // Linked entities
  linkedWallets: string[];
  linkedBankAccounts?: BankAccountLink[];

  // Settings
  settings: {
    defaultCurrency: string;
    notificationPreferences: NotificationPreferences;
    privacySettings: PrivacySettings;
  };
}

enum AccountType {
  PERSONAL = 'PERSONAL',
  BUSINESS = 'BUSINESS',
  GOVERNMENT = 'GOVERNMENT',
  INSTITUTIONAL = 'INSTITUTIONAL',
  RESERVE = 'RESERVE'
}

enum ParticipantType {
  INDIVIDUAL = 'INDIVIDUAL',
  SOLE_PROPRIETOR = 'SOLE_PROPRIETOR',
  CORPORATION = 'CORPORATION',
  GOVERNMENT_ENTITY = 'GOVERNMENT_ENTITY',
  FINANCIAL_INSTITUTION = 'FINANCIAL_INSTITUTION',
  CENTRAL_BANK = 'CENTRAL_BANK'
}

enum KYCLevel {
  NONE = 'NONE',                  // No verification
  BASIC = 'BASIC',                // Phone/email only
  STANDARD = 'STANDARD',          // ID document verified
  ENHANCED = 'ENHANCED',          // Full KYC with address
  INSTITUTIONAL = 'INSTITUTIONAL' // Corporate KYC
}

enum AccountState {
  PENDING_ACTIVATION = 'PENDING_ACTIVATION',
  ACTIVE = 'ACTIVE',
  SUSPENDED = 'SUSPENDED',
  FROZEN = 'FROZEN',
  CLOSED = 'CLOSED'
}

// CBDC Wallet Model
interface CBDCWallet {
  walletId: string;
  walletType: WalletType;
  accountId: string;              // Parent account

  // Wallet identification
  walletAddress: string;          // Public address
  publicKey: string;

  // Wallet capabilities
  capabilities: {
    canSend: boolean;
    canReceive: boolean;
    canOffline: boolean;
    programmableSupport: boolean;
    maxTokens: number;
  };

  // Device binding (for mobile/hardware wallets)
  deviceBinding?: {
    deviceId: string;
    deviceType: DeviceType;
    deviceFingerprint: string;
    bindingCreatedAt: ISO8601DateTime;
    lastUsedAt: ISO8601DateTime;
  };

  // Security
  security: {
    multiSigRequired: boolean;
    multiSigThreshold?: number;
    biometricEnabled: boolean;
    pinEnabled: boolean;
    spendingPassword: boolean;
  };

  // Token holdings
  tokens: TokenHolding[];

  // Offline balance (for offline-capable wallets)
  offlineState?: {
    offlineBalance: MonetaryAmount;
    lastSyncAt: ISO8601DateTime;
    pendingOfflineTransactions: OfflineTransaction[];
  };
}

enum WalletType {
  MOBILE = 'MOBILE',
  WEB = 'WEB',
  HARDWARE = 'HARDWARE',
  CUSTODIAL = 'CUSTODIAL',
  SMART_CONTRACT = 'SMART_CONTRACT'
}

enum DeviceType {
  SMARTPHONE = 'SMARTPHONE',
  TABLET = 'TABLET',
  SMARTWATCH = 'SMARTWATCH',
  HARDWARE_WALLET = 'HARDWARE_WALLET',
  SIM_CARD = 'SIM_CARD',
  SECURE_ELEMENT = 'SECURE_ELEMENT'
}

interface TokenHolding {
  tokenId: string;
  denomination: MonetaryAmount;
  acquiredAt: ISO8601DateTime;
  conditions?: TokenCondition[];
}
```

### 3.4 Transaction Data Models

```typescript
// CBDC Transaction Model
interface CBDCTransaction {
  // Transaction identification
  transactionId: string;          // UUID
  referenceNumber: string;        // Human-readable reference
  correlationId?: string;         // For related transactions

  // Transaction type
  type: TransactionType;
  subType?: TransactionSubType;

  // Parties
  parties: {
    initiator: TransactionParty;
    sender?: TransactionParty;
    receiver?: TransactionParty;
    intermediaries?: TransactionParty[];
  };

  // Value
  amount: MonetaryAmount;
  fees?: TransactionFee[];

  // Timing
  timing: {
    initiatedAt: ISO8601DateTime;
    submittedAt: ISO8601DateTime;
    processedAt?: ISO8601DateTime;
    settledAt?: ISO8601DateTime;
    expiresAt?: ISO8601DateTime;
  };

  // Status
  status: {
    current: TransactionStatus;
    history: StatusChange[];
  };

  // Execution details
  execution: {
    channel: TransactionChannel;
    method: PaymentMethod;
    deviceInfo?: DeviceInfo;
    locationInfo?: LocationInfo;
  };

  // Compliance
  compliance: {
    amlScreeningResult: ScreeningResult;
    sanctionsCheckResult: ScreeningResult;
    riskScore: number;
    flags: ComplianceFlag[];
  };

  // Technical details
  technical: {
    ledgerEntries: LedgerEntry[];
    signatures: TransactionSignature[];
    proofs?: CryptographicProof[];
    blockReference?: BlockReference;
  };

  // Additional data
  metadata?: {
    description?: string;
    merchantCategory?: string;
    invoiceReference?: string;
    customFields?: Record<string, any>;
  };
}

enum TransactionType {
  // Lifecycle transactions
  ISSUANCE = 'ISSUANCE',
  REDEMPTION = 'REDEMPTION',

  // Transfer transactions
  TRANSFER = 'TRANSFER',
  PAYMENT = 'PAYMENT',

  // Account operations
  DEPOSIT = 'DEPOSIT',
  WITHDRAWAL = 'WITHDRAWAL',

  // Special transactions
  FX_CONVERSION = 'FX_CONVERSION',
  CROSS_BORDER = 'CROSS_BORDER',

  // Programmable transactions
  CONDITIONAL_PAYMENT = 'CONDITIONAL_PAYMENT',
  ESCROW = 'ESCROW',

  // Administrative
  ADJUSTMENT = 'ADJUSTMENT',
  REVERSAL = 'REVERSAL',
  FEE = 'FEE'
}

enum TransactionSubType {
  // Transfer subtypes
  P2P = 'P2P',                    // Person to Person
  P2M = 'P2M',                    // Person to Merchant
  P2G = 'P2G',                    // Person to Government
  G2P = 'G2P',                    // Government to Person
  B2B = 'B2B',                    // Business to Business

  // Payment subtypes
  RETAIL_PAYMENT = 'RETAIL_PAYMENT',
  BILL_PAYMENT = 'BILL_PAYMENT',
  SALARY = 'SALARY',
  PENSION = 'PENSION',
  SUBSIDY = 'SUBSIDY',
  TAX_PAYMENT = 'TAX_PAYMENT',

  // Cross-border subtypes
  REMITTANCE = 'REMITTANCE',
  TRADE_SETTLEMENT = 'TRADE_SETTLEMENT',

  // Offline
  OFFLINE_TRANSFER = 'OFFLINE_TRANSFER'
}

enum TransactionStatus {
  INITIATED = 'INITIATED',
  PENDING_AUTHORIZATION = 'PENDING_AUTHORIZATION',
  AUTHORIZED = 'AUTHORIZED',
  PENDING_SCREENING = 'PENDING_SCREENING',
  SCREENED = 'SCREENED',
  PENDING_EXECUTION = 'PENDING_EXECUTION',
  EXECUTING = 'EXECUTING',
  PENDING_SETTLEMENT = 'PENDING_SETTLEMENT',
  SETTLED = 'SETTLED',
  COMPLETED = 'COMPLETED',
  FAILED = 'FAILED',
  CANCELLED = 'CANCELLED',
  REVERSED = 'REVERSED',
  EXPIRED = 'EXPIRED'
}

interface TransactionParty {
  participantId: string;
  accountId?: string;
  walletId?: string;
  walletAddress?: string;
  participantType: ParticipantType;
  name?: string;                  // May be masked for privacy
  role: PartyRole;
}

enum PartyRole {
  INITIATOR = 'INITIATOR',
  SENDER = 'SENDER',
  RECEIVER = 'RECEIVER',
  AUTHORIZER = 'AUTHORIZER',
  INTERMEDIARY = 'INTERMEDIARY'
}

interface TransactionFee {
  feeType: FeeType;
  amount: MonetaryAmount;
  collector: string;
  description: string;
}

enum FeeType {
  NETWORK_FEE = 'NETWORK_FEE',
  SERVICE_FEE = 'SERVICE_FEE',
  FX_FEE = 'FX_FEE',
  CROSS_BORDER_FEE = 'CROSS_BORDER_FEE',
  INSTANT_FEE = 'INSTANT_FEE'
}

interface LedgerEntry {
  entryId: string;
  accountId: string;
  entryType: 'DEBIT' | 'CREDIT';
  amount: MonetaryAmount;
  balanceBefore: MonetaryAmount;
  balanceAfter: MonetaryAmount;
  timestamp: ISO8601DateTime;
}

interface TransactionSignature {
  signerId: string;
  signerRole: string;
  algorithm: string;
  signature: string;
  timestamp: ISO8601DateTime;
  publicKey: string;
}
```

### 3.5 Issuance and Redemption Models

```typescript
// CBDC Issuance Model
interface CBDCIssuance {
  issuanceId: string;
  issuanceType: IssuanceType;

  // Issuance details
  details: {
    totalAmount: MonetaryAmount;
    tokenDenominations: DenominationBreakdown[];
    numberOfTokens: number;
    batchId: string;
  };

  // Authorization
  authorization: {
    requestedBy: string;
    authorizedBy: string[];
    authorizationLevel: number;    // Multi-sig threshold
    policyReference: string;
  };

  // Recipient
  recipient: {
    participantId: string;
    accountId: string;
    recipientType: ParticipantType;
  };

  // Execution
  execution: {
    status: IssuanceStatus;
    requestedAt: ISO8601DateTime;
    authorizedAt?: ISO8601DateTime;
    executedAt?: ISO8601DateTime;
    mintingNodeId?: string;
  };

  // Monetary policy context
  policyContext: {
    monetaryPolicyReference?: string;
    economicIndicators?: EconomicIndicators;
    issuanceReason: IssuanceReason;
    notes?: string;
  };

  // Audit trail
  audit: {
    createdTokenIds: string[];
    ledgerTransactionIds: string[];
    signatures: AuthorizationSignature[];
  };
}

enum IssuanceType {
  INITIAL_ISSUANCE = 'INITIAL_ISSUANCE',
  MONETARY_POLICY = 'MONETARY_POLICY',
  EMERGENCY_ISSUANCE = 'EMERGENCY_ISSUANCE',
  REPLACEMENT = 'REPLACEMENT',
  INTERBANK_DISTRIBUTION = 'INTERBANK_DISTRIBUTION'
}

enum IssuanceStatus {
  REQUESTED = 'REQUESTED',
  PENDING_AUTHORIZATION = 'PENDING_AUTHORIZATION',
  AUTHORIZED = 'AUTHORIZED',
  MINTING = 'MINTING',
  DISTRIBUTED = 'DISTRIBUTED',
  COMPLETED = 'COMPLETED',
  REJECTED = 'REJECTED',
  CANCELLED = 'CANCELLED'
}

enum IssuanceReason {
  MONETARY_EXPANSION = 'MONETARY_EXPANSION',
  ECONOMIC_STIMULUS = 'ECONOMIC_STIMULUS',
  BANK_LIQUIDITY = 'BANK_LIQUIDITY',
  MARKET_INTERVENTION = 'MARKET_INTERVENTION',
  DEPOSIT_CONVERSION = 'DEPOSIT_CONVERSION',
  EMERGENCY_RESPONSE = 'EMERGENCY_RESPONSE'
}

interface DenominationBreakdown {
  denomination: MonetaryAmount;
  quantity: number;
  totalValue: MonetaryAmount;
}

// CBDC Redemption Model
interface CBDCRedemption {
  redemptionId: string;
  redemptionType: RedemptionType;

  // Redemption details
  details: {
    totalAmount: MonetaryAmount;
    tokenIds: string[];
    numberOfTokens: number;
  };

  // Requester
  requester: {
    participantId: string;
    accountId: string;
    participantType: ParticipantType;
  };

  // Settlement
  settlement: {
    method: RedemptionMethod;
    destinationAccount?: string;
    destinationBank?: string;
  };

  // Execution
  execution: {
    status: RedemptionStatus;
    requestedAt: ISO8601DateTime;
    processedAt?: ISO8601DateTime;
    settledAt?: ISO8601DateTime;
  };

  // Verification
  verification: {
    tokenValidation: ValidationResult[];
    ownershipProof: string;
    complianceCheck: ComplianceCheckResult;
  };
}

enum RedemptionType {
  STANDARD = 'STANDARD',
  EXPIRED_TOKEN = 'EXPIRED_TOKEN',
  DAMAGED_TOKEN = 'DAMAGED_TOKEN',
  BANK_SETTLEMENT = 'BANK_SETTLEMENT',
  MONETARY_CONTRACTION = 'MONETARY_CONTRACTION'
}

enum RedemptionMethod {
  BANK_DEPOSIT = 'BANK_DEPOSIT',
  CASH = 'CASH',
  INTERBANK_SETTLEMENT = 'INTERBANK_SETTLEMENT',
  DESTRUCTION = 'DESTRUCTION'
}

enum RedemptionStatus {
  REQUESTED = 'REQUESTED',
  VALIDATING = 'VALIDATING',
  APPROVED = 'APPROVED',
  PROCESSING = 'PROCESSING',
  SETTLED = 'SETTLED',
  COMPLETED = 'COMPLETED',
  REJECTED = 'REJECTED'
}
```

### 3.6 Cross-Border Transaction Models

```typescript
// Cross-Border CBDC Transaction
interface CrossBorderCBDCTransaction {
  transactionId: string;
  corridorId: string;             // e.g., "USD-EUR", "CNY-THB"

  // Source details
  source: {
    cbdc: string;                 // Source CBDC code
    amount: MonetaryAmount;
    senderAccount: string;
    originatingCountry: string;
    originatingCentralBank: string;
  };

  // Destination details
  destination: {
    cbdc: string;                 // Destination CBDC code
    amount: MonetaryAmount;
    receiverAccount: string;
    destinationCountry: string;
    destinationCentralBank: string;
  };

  // FX details
  foreignExchange: {
    exchangeRate: number;
    rateSource: string;
    rateTimestamp: ISO8601DateTime;
    spreadBps: number;
    inverseRate: number;
  };

  // Settlement details
  settlement: {
    mechanism: CrossBorderSettlementMechanism;
    platform?: string;            // e.g., "mBridge", "Project Dunbar"
    atomicSettlement: boolean;
    settlementCurrency?: string;
  };

  // Compliance
  crossBorderCompliance: {
    originKYC: boolean;
    destinationKYC: boolean;
    capitalControlsCheck: boolean;
    sanctionsCheck: boolean;
    reportingRequirements: ReportingRequirement[];
  };

  // Timing
  timing: {
    initiatedAt: ISO8601DateTime;
    fxLockedAt?: ISO8601DateTime;
    settlementStartedAt?: ISO8601DateTime;
    completedAt?: ISO8601DateTime;
    valueDate: ISO8601Date;
  };

  status: CrossBorderTransactionStatus;
}

enum CrossBorderSettlementMechanism {
  CORRESPONDENT_BANKING = 'CORRESPONDENT_BANKING',
  MULTI_CBDC_PLATFORM = 'MULTI_CBDC_PLATFORM',
  BILATERAL_LINK = 'BILATERAL_LINK',
  HUB_AND_SPOKE = 'HUB_AND_SPOKE',
  ATOMIC_SWAP = 'ATOMIC_SWAP'
}

enum CrossBorderTransactionStatus {
  INITIATED = 'INITIATED',
  FX_RATE_LOCKED = 'FX_RATE_LOCKED',
  COMPLIANCE_CLEARED = 'COMPLIANCE_CLEARED',
  PENDING_ORIGIN_SETTLEMENT = 'PENDING_ORIGIN_SETTLEMENT',
  ORIGIN_SETTLED = 'ORIGIN_SETTLED',
  PENDING_DESTINATION_SETTLEMENT = 'PENDING_DESTINATION_SETTLEMENT',
  DESTINATION_SETTLED = 'DESTINATION_SETTLED',
  COMPLETED = 'COMPLETED',
  FAILED = 'FAILED',
  REVERSED = 'REVERSED'
}

interface ReportingRequirement {
  jurisdiction: string;
  reportType: string;
  threshold?: MonetaryAmount;
  deadline: string;
  submitted: boolean;
}
```

### 3.7 Data Validation and Integrity

```typescript
// Data Validation Engine
class CBDCDataValidator {
  private schemas: Map<string, JSONSchema>;
  private customValidators: Map<string, CustomValidator>;

  constructor() {
    this.loadSchemas();
    this.registerCustomValidators();
  }

  async validateToken(token: CBDCToken): Promise<ValidationResult> {
    const results: ValidationError[] = [];

    // Schema validation
    const schemaResult = this.validateAgainstSchema(token, 'CBDCToken');
    results.push(...schemaResult.errors);

    // Business rule validation
    results.push(...this.validateTokenBusinessRules(token));

    // Cryptographic validation
    results.push(...await this.validateTokenCryptography(token));

    return {
      valid: results.length === 0,
      errors: results,
      warnings: this.getWarnings(token)
    };
  }

  private validateTokenBusinessRules(token: CBDCToken): ValidationError[] {
    const errors: ValidationError[] = [];

    // Check denomination validity
    if (parseFloat(token.denomination.value) <= 0) {
      errors.push({
        field: 'denomination.value',
        message: 'Token denomination must be positive',
        code: 'INVALID_DENOMINATION'
      });
    }

    // Check expiration
    if (token.state.expiresAt) {
      const expirationDate = new Date(token.state.expiresAt);
      if (expirationDate <= new Date()) {
        errors.push({
          field: 'state.expiresAt',
          message: 'Token has expired',
          code: 'TOKEN_EXPIRED'
        });
      }
    }

    // Check lock status
    if (token.state.lockedUntil) {
      const lockDate = new Date(token.state.lockedUntil);
      if (lockDate > new Date() && token.state.status === TokenStatus.ACTIVE) {
        errors.push({
          field: 'state.status',
          message: 'Locked token should not be ACTIVE',
          code: 'INVALID_LOCK_STATUS'
        });
      }
    }

    // Validate conditions
    if (token.conditions) {
      for (const condition of token.conditions) {
        const conditionErrors = this.validateCondition(condition);
        errors.push(...conditionErrors);
      }
    }

    return errors;
  }

  async validateTransaction(
    transaction: CBDCTransaction
  ): Promise<ValidationResult> {
    const results: ValidationError[] = [];

    // Schema validation
    const schemaResult = this.validateAgainstSchema(transaction, 'CBDCTransaction');
    results.push(...schemaResult.errors);

    // Party validation
    results.push(...this.validateTransactionParties(transaction));

    // Amount validation
    results.push(...this.validateTransactionAmount(transaction));

    // Signature validation
    results.push(...await this.validateTransactionSignatures(transaction));

    // Double-spend check
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

    // Check if tokens have been spent
    const tokenIds = this.extractTokenIds(transaction);

    for (const tokenId of tokenIds) {
      const token = await this.tokenRepository.findById(tokenId);

      if (!token) {
        errors.push({
          field: 'tokens',
          message: `Token ${tokenId} not found`,
          code: 'TOKEN_NOT_FOUND'
        });
        continue;
      }

      if (token.state.status !== TokenStatus.ACTIVE) {
        errors.push({
          field: 'tokens',
          message: `Token ${tokenId} is not active (status: ${token.state.status})`,
          code: 'TOKEN_NOT_ACTIVE'
        });
      }

      // Check ownership
      if (token.ownership.currentHolder !== transaction.parties.sender?.walletId) {
        errors.push({
          field: 'tokens',
          message: `Sender does not own token ${tokenId}`,
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

interface ValidationWarning {
  field: string;
  message: string;
  suggestion?: string;
}
```

### 3.8 Summary

The WIA-CBDC data model provides:

1. **Comprehensive Token Model**: Full lifecycle support with programmability
2. **Flexible Account Structure**: Multi-tier accounts with KYC levels
3. **Rich Transaction Model**: Support for all transaction types
4. **Cross-Border Support**: Multi-CBDC transaction handling
5. **Strong Validation**: Schema and business rule enforcement

---

**WIA-CBDC Data Models**
**Version**: 1.0.0
**Last Updated**: 2025

© 2025 WIA (World Interoperability Alliance)
