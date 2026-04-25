/**
 * WIA CBDC Standard - Type Definitions
 * Standard: WIA-FIN-005
 * Version: 1.0.0
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 */

// ============================================================================
// Core Enumerations
// ============================================================================

/**
 * CBDC network types
 */
export enum NetworkType {
  MAINNET = 'mainnet',
  TESTNET = 'testnet',
  DEVNET = 'devnet',
  SANDBOX = 'sandbox'
}

/**
 * CBDC implementation models
 */
export enum CBDCModel {
  RETAIL_DIRECT = 'retail_direct',        // Public holds CBDC directly with central bank
  RETAIL_TWO_TIER = 'retail_two_tier',    // Intermediaries serve public
  WHOLESALE = 'wholesale',                 // Only for financial institutions
  HYBRID = 'hybrid'                        // Combination of models
}

/**
 * Transaction types
 */
export enum TransactionType {
  TRANSFER = 'transfer',
  MINT = 'mint',
  BURN = 'burn',
  EXCHANGE = 'exchange',
  CONTRACT = 'contract',
  STAKE = 'stake',
  UNSTAKE = 'unstake'
}

/**
 * Transaction status
 */
export enum TransactionStatus {
  PENDING = 'pending',
  CONFIRMED = 'confirmed',
  FAILED = 'failed',
  REJECTED = 'rejected'
}

/**
 * Privacy levels
 */
export enum PrivacyLevel {
  FULL_PRIVACY = 'full_privacy',          // Tier 1: Zero-knowledge proofs
  CONTROLLED = 'controlled',               // Tier 2: Controlled anonymity
  STANDARD = 'standard',                   // Tier 3: Standard privacy
  TRANSPARENT = 'transparent'              // Tier 4: Full transparency
}

/**
 * Privacy techniques
 */
export enum PrivacyTechnique {
  ZKSNARK = 'zksnark',
  ZKSTARK = 'zkstark',
  RING_SIGNATURE = 'ring_signature',
  CONFIDENTIAL_TX = 'confidential_tx',
  NONE = 'none'
}

/**
 * Certification levels
 */
export enum CertificationLevel {
  COMPATIBLE = 'compatible',
  CERTIFIED = 'certified',
  FULL_COMPLIANCE = 'full_compliance'
}

/**
 * Wallet types
 */
export enum WalletType {
  NON_CUSTODIAL = 'non_custodial',
  CUSTODIAL = 'custodial',
  MULTISIG = 'multisig',
  SMART_CONTRACT = 'smart_contract',
  HARDWARE = 'hardware'
}

// ============================================================================
// Core Interfaces
// ============================================================================

/**
 * Currency information
 */
export interface Currency {
  code: string;           // ISO 4217 or custom code (e.g., "CNY", "EUR")
  cbdcId: string;         // Unique CBDC identifier (e.g., "digital-yuan")
  issuer: string;         // Issuing central bank (e.g., "PBOC", "ECB")
  precision: number;      // Decimal precision (typically 2-8)
}

/**
 * Amount representation
 */
export interface Amount {
  value: string;          // Fixed-point decimal string (never float!)
  precision: number;      // Number of decimal places
}

/**
 * Transaction fee
 */
export interface TransactionFee {
  value: string;
  payer: 'sender' | 'recipient' | 'shared';
}

/**
 * Wallet participant
 */
export interface Participant {
  walletId: string;
  cbdcId?: string;        // For cross-CBDC transactions
  publicKey?: string;
  signature?: string;
  did?: string;           // Decentralized Identifier
}

/**
 * Privacy configuration
 */
export interface PrivacyConfig {
  level: PrivacyLevel;
  technique: PrivacyTechnique;
  proof?: string;         // Zero-knowledge proof data
  reveals?: {
    amount?: boolean;
    sender?: boolean;
    recipient?: boolean;
    timestamp?: boolean;
  };
}

/**
 * Compliance metadata
 */
export interface ComplianceMetadata {
  aml: boolean;
  sanctions: 'checked' | 'pending' | 'failed';
  taxReporting: 'required' | 'optional' | 'none';
  kycLevel?: 'tier1' | 'tier2' | 'tier3' | 'tier4';
}

/**
 * Universal CBDC transaction
 */
export interface CBDCTransaction {
  version: string;
  standard: 'WIA-FIN-005';
  transactionId?: string;
  timestamp: string;      // ISO 8601 format
  currency: Currency;
  type: TransactionType;
  sender: Participant;
  recipient: Participant;
  amount: Amount;
  fee: TransactionFee;
  privacy: PrivacyConfig;
  metadata?: Record<string, any>;
  compliance: ComplianceMetadata;
  lockTime?: number;
  nonce?: number;
  confirmations?: number;
  blockHeight?: number;
  blockHash?: string;
}

/**
 * Wallet configuration
 */
export interface Wallet {
  walletId: string;
  version: string;
  standard: 'WIA-FIN-005';
  cbdcId: string;
  type: WalletType;
  created: string;
  owner: {
    type: 'individual' | 'business' | 'government';
    verificationLevel: 'tier1' | 'tier2' | 'tier3' | 'tier4';
    publicKey: string;
    did?: string;
  };
  balance: {
    available: string;
    pending: string;
    locked: string;
  };
  limits: {
    dailyTransaction: string;
    singleTransaction: string;
    monthlyVolume: string;
  };
  features: {
    offlinePayments: boolean;
    smartContracts: boolean;
    multiSignature: boolean;
    crossBorder: boolean;
  };
  privacy: {
    defaultLevel: PrivacyLevel;
    supportedTechniques: PrivacyTechnique[];
  };
}

/**
 * Smart contract
 */
export interface SmartContract {
  contractId: string;
  cbdcId: string;
  version: string;
  bytecode: string;
  abi: any[];
  creator: string;
  createdAt: string;
  parameters?: Record<string, any>;
  state?: Record<string, any>;
}

/**
 * Cross-border exchange
 */
export interface CrossBorderExchange {
  exchangeId: string;
  sourceCurrency: Currency;
  targetCurrency: Currency;
  sourceAmount: Amount;
  targetAmount: Amount;
  exchangeRate: string;
  mechanism: 'atomic_swap' | 'pvp' | 'liquidity_pool' | 'central_bridge';
  settlementTime: string;
  status: 'initiated' | 'in_progress' | 'settled' | 'failed';
}

/**
 * Payment request
 */
export interface PaymentRequest {
  requestId: string;
  merchant: string;
  amount: Amount;
  currency: Currency;
  acceptedCBDCs: string[];
  expiresAt: string;
  memo?: string;
  qrCode?: string;
}

/**
 * Offline payment token
 */
export interface OfflineToken {
  tokenId: string;
  cbdcId: string;
  amount: Amount;
  validUntil: string;
  maxUses: number;
  usedCount: number;
  secureElement: {
    chipId: string;
    publicKey: string;
  };
}

/**
 * Certification metadata
 */
export interface CertificationMetadata {
  certificationId: string;
  level: CertificationLevel;
  validFrom: string;
  validUntil: string;
  issuer: string;
  scope: string[];
  attestations: {
    securityAudit: boolean;
    interoperabilityTest: boolean;
    performanceBenchmark: boolean;
  };
}

// ============================================================================
// API Request/Response Types
// ============================================================================

/**
 * Create wallet request
 */
export interface CreateWalletRequest {
  cbdcId: string;
  type: WalletType;
  owner: {
    type: 'individual' | 'business' | 'government';
    publicKey: string;
    did?: string;
  };
  features?: {
    offlinePayments?: boolean;
    smartContracts?: boolean;
    multiSignature?: boolean;
    crossBorder?: boolean;
  };
}

/**
 * Transfer request
 */
export interface TransferRequest {
  from: string;
  to: string;
  amount: string;
  currency?: Currency;
  privacy?: PrivacyConfig;
  memo?: string;
  metadata?: Record<string, any>;
}

/**
 * Cross-border transfer request
 */
export interface CrossBorderTransferRequest {
  from: string;
  to: string;
  sourceAmount: string;
  sourceCurrency: Currency;
  targetCurrency: Currency;
  mechanism?: 'atomic_swap' | 'pvp' | 'liquidity_pool' | 'central_bridge';
  maxSlippage?: number;
}

/**
 * Query balance request
 */
export interface QueryBalanceRequest {
  walletId: string;
  cbdcId: string;
}

/**
 * Query transaction request
 */
export interface QueryTransactionRequest {
  transactionId: string;
  cbdcId: string;
}

/**
 * Transaction history request
 */
export interface TransactionHistoryRequest {
  walletId: string;
  cbdcId: string;
  startDate?: string;
  endDate?: string;
  limit?: number;
  offset?: number;
  types?: TransactionType[];
}

/**
 * API response wrapper
 */
export interface APIResponse<T> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
    details?: any;
  };
  metadata?: {
    timestamp: string;
    requestId: string;
    version: string;
  };
}

/**
 * Validation result
 */
export interface ValidationResult {
  valid: boolean;
  errors?: string[];
  warnings?: string[];
}

/**
 * Configuration for CBDC SDK
 */
export interface CBDCConfig {
  cbdcId: string;
  network: NetworkType;
  model: CBDCModel;
  apiEndpoint?: string;
  wsEndpoint?: string;
  privateKey?: string;
  certificationLevel?: CertificationLevel;
  features?: {
    crossBorder?: boolean;
    smartContracts?: boolean;
    offlinePayments?: boolean;
    privacyEnhanced?: boolean;
  };
  limits?: {
    maxTransactionSize?: string;
    dailyLimit?: string;
  };
  debug?: boolean;
}

// ============================================================================
// Event Types
// ============================================================================

/**
 * Transaction event
 */
export interface TransactionEvent {
  type: 'transaction';
  transactionId: string;
  status: TransactionStatus;
  transaction: CBDCTransaction;
  timestamp: string;
}

/**
 * Balance update event
 */
export interface BalanceUpdateEvent {
  type: 'balance_update';
  walletId: string;
  oldBalance: Amount;
  newBalance: Amount;
  reason: string;
  timestamp: string;
}

/**
 * System event
 */
export interface SystemEvent {
  type: 'system';
  eventType: 'maintenance' | 'upgrade' | 'alert' | 'notification';
  severity: 'info' | 'warning' | 'error' | 'critical';
  message: string;
  timestamp: string;
}

export type CBDCEvent = TransactionEvent | BalanceUpdateEvent | SystemEvent;

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  items: T[];
  total: number;
  limit: number;
  offset: number;
  hasMore: boolean;
}

/**
 * Statistics
 */
export interface CBDCStatistics {
  totalSupply: Amount;
  circulatingSupply: Amount;
  totalWallets: number;
  activeWallets24h: number;
  transactionsToday: number;
  avgTransactionSize: Amount;
  networkLoad: number;  // 0-1 representing capacity usage
}
