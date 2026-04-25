/**
 * WIA-FIN-004 Digital Currency Standard - TypeScript Type Definitions
 * @version 1.0.0
 */

// ==================== Currency Types ====================

export enum CurrencyType {
  FIAT = 'FIAT',
  CBDC = 'CBDC',
  STABLECOIN_FIAT = 'STABLECOIN_FIAT',
  STABLECOIN_CRYPTO = 'STABLECOIN_CRYPTO',
  STABLECOIN_COMMODITY = 'STABLECOIN_COMMODITY',
  STABLECOIN_ALGORITHMIC = 'STABLECOIN_ALGORITHMIC',
  CRYPTOCURRENCY = 'CRYPTOCURRENCY',
  VIRTUAL = 'VIRTUAL',
  CORPORATE = 'CORPORATE',
  E_MONEY = 'E_MONEY'
}

export interface Currency {
  code: string;
  name: string;
  type: CurrencyType;
  symbol: string;
  decimals: number;
  minAmount: string;
  maxAmount?: string;
  blockchain?: {
    network: string;
    chainId: number;
    contractAddress?: string;
    standard?: string;
  };
  issuer?: {
    entity: string;
    jurisdiction: string;
    website?: string;
    regulators?: string[];
  };
  metadata: {
    created: string;
    lastUpdated: string;
    status: 'ACTIVE' | 'DEPRECATED' | 'SUSPENDED';
  };
}

// ==================== Account Types ====================

export enum AccountType {
  PERSONAL = 'PERSONAL',
  BUSINESS = 'BUSINESS',
  SAVINGS = 'SAVINGS',
  CHECKING = 'CHECKING',
  MERCHANT = 'MERCHANT',
  EXCHANGE = 'EXCHANGE',
  CUSTODIAL = 'CUSTODIAL',
  NON_CUSTODIAL = 'NON_CUSTODIAL'
}

export enum AccountStatus {
  ACTIVE = 'ACTIVE',
  PENDING = 'PENDING',
  SUSPENDED = 'SUSPENDED',
  FROZEN = 'FROZEN',
  CLOSED = 'CLOSED'
}

export interface Account {
  id: string;
  type: AccountType;
  status: AccountStatus;
  owner: {
    type: 'INDIVIDUAL' | 'BUSINESS' | 'INSTITUTION';
    id: string;
    name: string;
    email?: string;
    phone?: string;
    address?: Address;
  };
  balances: Balance[];
  limits?: TransactionLimits;
  compliance: ComplianceInfo;
  security: SecuritySettings;
  metadata: {
    created: string;
    lastUpdated: string;
    tags?: string[];
  };
}

export interface Address {
  street: string;
  city: string;
  state?: string;
  postalCode: string;
  country: string;
}

export interface Balance {
  currency: string;
  available: string;
  pending: string;
  locked: string;
  total: string;
}

export interface TransactionLimits {
  dailyTransactionLimit: string;
  monthlyTransactionLimit: string;
  singleTransactionLimit: string;
  currency: string;
}

export interface ComplianceInfo {
  kycLevel: 'BASIC' | 'STANDARD' | 'ENHANCED';
  kycStatus: 'PENDING' | 'VERIFIED' | 'REJECTED' | 'EXPIRED';
  kycDate?: string;
  amlRating: 'LOW' | 'MEDIUM' | 'HIGH';
  pepStatus: boolean;
  sanctionsScreening: {
    status: 'CLEAR' | 'FLAGGED' | 'BLOCKED';
    lastChecked: string;
  };
}

export interface SecuritySettings {
  mfaEnabled: boolean;
  mfaMethods: ('SMS' | 'TOTP' | 'HARDWARE_KEY')[];
  lastLogin?: string;
  ipWhitelist?: string[];
}

// ==================== Transaction Types ====================

export enum TransactionType {
  TRANSFER = 'TRANSFER',
  PAYMENT = 'PAYMENT',
  EXCHANGE = 'EXCHANGE',
  WITHDRAWAL = 'WITHDRAWAL',
  DEPOSIT = 'DEPOSIT',
  REFUND = 'REFUND',
  FEE = 'FEE',
  MINT = 'MINT',
  BURN = 'BURN',
  STAKE = 'STAKE',
  UNSTAKE = 'UNSTAKE',
  REWARD = 'REWARD'
}

export enum TransactionStatus {
  INITIATED = 'INITIATED',
  PENDING = 'PENDING',
  PROCESSING = 'PROCESSING',
  CONFIRMING = 'CONFIRMING',
  COMPLETED = 'COMPLETED',
  FAILED = 'FAILED',
  CANCELLED = 'CANCELLED',
  REVERSED = 'REVERSED',
  FLAGGED = 'FLAGGED',
  BLOCKED = 'BLOCKED'
}

export interface Transaction {
  id: string;
  type: TransactionType;
  status: TransactionStatus;
  timestamp: string;
  sender: PartyIdentifier;
  recipient: PartyIdentifier;
  amount: Amount;
  fee?: Amount;
  exchange?: ExchangeDetails;
  purpose?: string;
  reference?: string;
  blockchain?: BlockchainDetails;
  settlement?: SettlementDetails;
  compliance: TransactionCompliance;
  security?: SecurityDetails;
  metadata: {
    created: string;
    lastUpdated: string;
    completedAt?: string;
    failureReason?: string;
    retryCount?: number;
    tags?: string[];
    extensionData?: Record<string, any>;
  };
}

export interface PartyIdentifier {
  type: 'ACCOUNT' | 'WALLET' | 'ADDRESS';
  identifier: string;
  name?: string;
}

export interface Amount {
  value: string;
  currency: string;
  precision: number;
}

export interface ExchangeDetails {
  fromCurrency: string;
  toCurrency: string;
  rate: string;
  receivedAmount: string;
}

export interface BlockchainDetails {
  network: string;
  txHash?: string;
  blockNumber?: number;
  confirmations?: number;
  gasUsed?: number;
  gasPrice?: string;
}

export interface SettlementDetails {
  method: 'INSTANT' | 'BATCH' | 'DEFERRED';
  settledAt?: string;
  settlementReference?: string;
}

export interface TransactionCompliance {
  amlCheck: 'PASSED' | 'FLAGGED' | 'FAILED';
  amlScore?: number;
  kycVerified: boolean;
  sanctionsCheck: 'PASSED' | 'FLAGGED' | 'FAILED';
  travelRuleData?: TravelRuleData;
  sarFiled: boolean;
  flags?: string[];
}

export interface TravelRuleData {
  originatorName: string;
  originatorAddress: string;
  beneficiaryName: string;
  beneficiaryAddress: string;
}

export interface SecurityDetails {
  signature: string;
  signingKey: string;
  nonce: number;
  ipAddress?: string;
  deviceId?: string;
  geolocation?: {
    latitude: number;
    longitude: number;
    country: string;
  };
}

// ==================== Wallet Types ====================

export enum WalletType {
  HOT_WALLET = 'HOT_WALLET',
  COLD_WALLET = 'COLD_WALLET',
  HARDWARE_WALLET = 'HARDWARE_WALLET',
  PAPER_WALLET = 'PAPER_WALLET',
  CUSTODIAL = 'CUSTODIAL',
  NON_CUSTODIAL = 'NON_CUSTODIAL',
  MULTI_SIG = 'MULTI_SIG',
  SMART_CONTRACT = 'SMART_CONTRACT'
}

export interface Wallet {
  id: string;
  type: WalletType;
  status: 'ACTIVE' | 'SUSPENDED' | 'LOCKED';
  name: string;
  description?: string;
  addresses: WalletAddress[];
  security: WalletSecurity;
  recovery?: RecoveryOptions;
  metadata: {
    created: string;
    lastAccessed: string;
    deviceId?: string;
    appVersion?: string;
  };
}

export interface WalletAddress {
  currency: string;
  address: string;
  derivationPath?: string;
  balance: string;
  lastActivity?: string;
}

export interface WalletSecurity {
  encryptionMethod: string;
  encryptionSalt: string;
  keyDerivation: string;
  backupMethod: string;
  seedPhraseBackedUp: boolean;
  multiSig?: MultiSigConfig;
}

export interface MultiSigConfig {
  enabled: boolean;
  required: number;
  total: number;
  signers: string[];
}

export interface RecoveryOptions {
  seedPhraseWordCount: 12 | 24;
  recoveryQuestions: boolean;
  socialRecovery?: {
    enabled: boolean;
    guardians: string[];
    threshold: number;
  };
}

// ==================== API Request/Response Types ====================

export interface PaymentRequest {
  from: PartyIdentifier;
  to: PartyIdentifier;
  amount: Amount;
  purpose?: string;
  reference?: string;
  metadata?: Record<string, any>;
}

export interface PaymentResponse {
  paymentId: string;
  status: TransactionStatus;
  estimatedCompletion?: string;
  fee?: Amount;
}

export interface ExchangeRequest {
  from: {
    currency: string;
    amount: string;
  };
  to: {
    currency: string;
  };
  accountId: string;
}

export interface ExchangeResponse {
  exchangeId: string;
  fromAmount: string;
  toAmount: string;
  rate: string;
  fee: Amount;
  status: TransactionStatus;
}

export interface BalanceResponse {
  accountId: string;
  balances: Balance[];
  lastUpdated: string;
}

export interface ErrorResponse {
  error: {
    code: string;
    message: string;
    details?: string;
    field?: string;
    timestamp: string;
    requestId: string;
    documentation?: string;
  };
}

// ==================== Configuration ====================

export interface WIAConfig {
  apiKey: string;
  apiUrl?: string;
  timeout?: number;
  retries?: number;
}
