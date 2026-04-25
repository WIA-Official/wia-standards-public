/**
 * WIA Cryptocurrency Standard - Type Definitions
 * Standard: WIA-FIN-003
 * Version: 1.0.0
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Supported blockchain networks
 */
export enum BlockchainNetwork {
  BITCOIN = 'bitcoin',
  ETHEREUM = 'ethereum',
  LITECOIN = 'litecoin',
  BITCOIN_CASH = 'bitcoin-cash',
  RIPPLE = 'ripple',
  CARDANO = 'cardano',
  POLKADOT = 'polkadot',
  SOLANA = 'solana',
  AVALANCHE = 'avalanche',
  POLYGON = 'polygon',
  BSC = 'binance-smart-chain',
  COSMOS = 'cosmos',
  TEZOS = 'tezos',
  MONERO = 'monero',
  ZCASH = 'zcash'
}

/**
 * Network types (mainnet, testnet, etc.)
 */
export enum NetworkType {
  MAINNET = 'mainnet',
  TESTNET = 'testnet',
  REGTEST = 'regtest',
  DEVNET = 'devnet'
}

/**
 * Transaction types
 */
export enum TransactionType {
  TRANSFER = 'transfer',
  CONTRACT = 'contract',
  STAKE = 'stake',
  UNSTAKE = 'unstake',
  SWAP = 'swap',
  BRIDGE = 'bridge',
  MINT = 'mint',
  BURN = 'burn'
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
 * Address types
 */
export enum AddressType {
  P2PKH = 'p2pkh',           // Pay to Public Key Hash (Bitcoin legacy)
  P2SH = 'p2sh',             // Pay to Script Hash (Bitcoin)
  P2WPKH = 'p2wpkh',         // Pay to Witness Public Key Hash (SegWit)
  P2WSH = 'p2wsh',           // Pay to Witness Script Hash (SegWit)
  BECH32 = 'bech32',         // Native SegWit (Bitcoin)
  ETHEREUM = 'ethereum',      // Ethereum address
  ED25519 = 'ed25519'        // Solana, Cardano
}

// ============================================================================
// Transaction Structures
// ============================================================================

/**
 * Universal transaction format
 */
export interface WIATransaction {
  version: string;
  standard: 'WIA-FIN-003';
  cryptocurrency: string;
  network: NetworkType;
  transactionId?: string;
  transactionType: TransactionType;
  timestamp: string;
  lockTime?: number;
  size?: number;
  weight?: number;
  fee: TransactionFee;
  confirmations?: number;
  blockHeight?: number;
  blockHash?: string;
  signatures: Signature[];
  metadata?: Record<string, any>;
}

/**
 * Bitcoin-style UTXO transaction
 */
export interface UTXOTransaction extends WIATransaction {
  inputs: UTXOInput[];
  outputs: UTXOOutput[];
}

/**
 * Ethereum-style account transaction
 */
export interface AccountTransaction extends WIATransaction {
  from: string;
  to: string;
  value: string;
  data?: string;
  nonce: number;
  gasLimit: string;
  gasPrice?: string;
  maxFeePerGas?: string;
  maxPriorityFeePerGas?: string;
}

/**
 * UTXO input
 */
export interface UTXOInput {
  previousTxId: string;
  outputIndex: number;
  scriptSig: string;
  sequence: number;
  witness?: string[];
  value?: string;
  address?: string;
}

/**
 * UTXO output
 */
export interface UTXOOutput {
  value: string;
  scriptPubKey: string;
  address: string;
  outputIndex: number;
  spent?: boolean;
}

/**
 * Transaction fee
 */
export interface TransactionFee {
  amount: string;
  currency: string;
  feeRate?: string;
  estimatedUSD?: number;
}

/**
 * Digital signature
 */
export interface Signature {
  algorithm: 'ECDSA' | 'Schnorr' | 'EdDSA' | 'BLS';
  publicKey: string;
  signature: string;
  messageHash: string;
}

// ============================================================================
// Wallet Types
// ============================================================================

/**
 * Wallet configuration
 */
export interface WalletConfig {
  blockchain: BlockchainNetwork;
  network: NetworkType;
  addressType?: AddressType;
  derivationPath?: string;
  mnemonic?: string;
  privateKey?: string;
  publicKey?: string;
}

/**
 * Wallet information
 */
export interface Wallet {
  address: string;
  publicKey: string;
  privateKey?: string;
  mnemonic?: string;
  derivationPath?: string;
  addressType: AddressType;
  blockchain: BlockchainNetwork;
  network: NetworkType;
  balance?: Balance;
}

/**
 * Balance information
 */
export interface Balance {
  confirmed: string;
  unconfirmed: string;
  total: string;
  currency: string;
  usdValue?: number;
  lastUpdated: string;
}

/**
 * Multi-signature wallet
 */
export interface MultisigWallet {
  address: string;
  required: number;
  total: number;
  signers: string[];
  blockchain: BlockchainNetwork;
  balance?: Balance;
}

// ============================================================================
// Block Structures
// ============================================================================

/**
 * Block header
 */
export interface BlockHeader {
  version: number;
  previousBlockHash: string;
  merkleRoot: string;
  timestamp: number;
  difficulty: string;
  nonce: number;
  height: number;
}

/**
 * Full block
 */
export interface Block {
  header: BlockHeader;
  transactions: WIATransaction[];
  size: number;
  weight?: number;
  transactionCount: number;
  hash: string;
  confirmations: number;
}

// ============================================================================
// Mining & Consensus
// ============================================================================

/**
 * Consensus mechanism
 */
export enum ConsensusMechanism {
  PROOF_OF_WORK = 'proof-of-work',
  PROOF_OF_STAKE = 'proof-of-stake',
  DELEGATED_PROOF_OF_STAKE = 'delegated-proof-of-stake',
  PROOF_OF_AUTHORITY = 'proof-of-authority',
  BYZANTINE_FAULT_TOLERANCE = 'byzantine-fault-tolerance',
  PROOF_OF_HISTORY = 'proof-of-history'
}

/**
 * Mining information
 */
export interface MiningInfo {
  blockchain: BlockchainNetwork;
  consensus: ConsensusMechanism;
  difficulty: string;
  hashRate: string;
  blockTime: number;
  blockReward: string;
  nextDifficultyAdjustment?: number;
}

/**
 * Staking information
 */
export interface StakingInfo {
  blockchain: BlockchainNetwork;
  totalStaked: string;
  stakingAPY: number;
  minimumStake: string;
  lockPeriod?: number;
  validators?: number;
}

// ============================================================================
// Exchange & Trading
// ============================================================================

/**
 * Exchange order types
 */
export enum OrderType {
  MARKET = 'market',
  LIMIT = 'limit',
  STOP_LOSS = 'stop-loss',
  TAKE_PROFIT = 'take-profit',
  OCO = 'oco'
}

/**
 * Order side
 */
export enum OrderSide {
  BUY = 'buy',
  SELL = 'sell'
}

/**
 * Order status
 */
export enum OrderStatus {
  PENDING = 'pending',
  OPEN = 'open',
  FILLED = 'filled',
  PARTIALLY_FILLED = 'partially-filled',
  CANCELLED = 'cancelled',
  REJECTED = 'rejected'
}

/**
 * Trading pair
 */
export interface TradingPair {
  base: string;
  quote: string;
  symbol: string;
  minAmount?: string;
  maxAmount?: string;
  pricePrecision?: number;
  amountPrecision?: number;
}

/**
 * Market order
 */
export interface MarketOrder {
  type: OrderType.MARKET;
  pair: string;
  side: OrderSide;
  amount: string;
  slippage?: number;
}

/**
 * Limit order
 */
export interface LimitOrder {
  type: OrderType.LIMIT;
  pair: string;
  side: OrderSide;
  amount: string;
  price: string;
  timeInForce?: 'GTC' | 'IOC' | 'FOK';
}

/**
 * Order result
 */
export interface OrderResult {
  orderId: string;
  status: OrderStatus;
  pair: string;
  side: OrderSide;
  type: OrderType;
  amount: string;
  price?: string;
  filled: string;
  remaining: string;
  timestamp: string;
  fee?: TransactionFee;
}

/**
 * Price ticker
 */
export interface PriceTicker {
  symbol: string;
  price: string;
  volume24h: string;
  high24h: string;
  low24h: string;
  change24h: number;
  timestamp: string;
}

// ============================================================================
// Payment & Commerce
// ============================================================================

/**
 * Payment request
 */
export interface PaymentRequest {
  requestId: string;
  merchant: string;
  amount: string;
  currency: string;
  acceptedCryptocurrencies: string[];
  expiresAt: string;
  memo?: string;
  metadata?: Record<string, any>;
}

/**
 * Payment result
 */
export interface PaymentResult {
  requestId: string;
  transactionId: string;
  cryptocurrency: string;
  amount: string;
  status: 'pending' | 'confirmed' | 'failed';
  confirmations: number;
  timestamp: string;
}

/**
 * QR code payment data
 */
export interface QRPaymentData {
  address: string;
  amount?: string;
  cryptocurrency: string;
  label?: string;
  message?: string;
  uri: string;
}

// ============================================================================
// Security & Compliance
// ============================================================================

/**
 * WIA compliance level
 */
export enum ComplianceLevel {
  REGISTERED = 'registered',
  VERIFIED = 'verified',
  CERTIFIED = 'certified'
}

/**
 * WIA compliance metadata
 */
export interface WIACompliance {
  version: string;
  standard: 'WIA-FIN-003';
  level: ComplianceLevel;
  certificationId?: string;
  validUntil?: string;
  scope: string[];
  audits?: Audit[];
}

/**
 * Security audit
 */
export interface Audit {
  auditor: string;
  date: string;
  type: string;
  status: 'passed' | 'failed' | 'pending';
  reportUrl?: string;
}

/**
 * KYC information
 */
export interface KYCInfo {
  level: 'none' | 'basic' | 'verified' | 'enhanced';
  provider?: string;
  verifiedAt?: string;
  expiresAt?: string;
  userId?: string;
}

/**
 * Travel Rule data (FATF)
 */
export interface TravelRuleData {
  originator: {
    name: string;
    accountNumber?: string;
    address: string;
    country: string;
    vasp: string;
  };
  beneficiary: {
    name: string;
    accountNumber?: string;
    address: string;
    country: string;
    vasp: string;
  };
}

// ============================================================================
// Smart Contracts
// ============================================================================

/**
 * Smart contract metadata
 */
export interface SmartContract {
  address: string;
  name: string;
  blockchain: BlockchainNetwork;
  abi?: any[];
  bytecode?: string;
  deployedAt?: string;
  creator?: string;
  verified: boolean;
  audits?: Audit[];
}

/**
 * Contract call
 */
export interface ContractCall {
  contract: string;
  method: string;
  params: any[];
  value?: string;
  gasLimit?: string;
}

// ============================================================================
// SDK Configuration
// ============================================================================

/**
 * SDK configuration
 */
export interface WIACryptocurrencyConfig {
  networks?: {
    [key in BlockchainNetwork]?: NetworkType;
  };
  providers?: {
    [key in BlockchainNetwork]?: string;
  };
  apiKeys?: {
    [exchange: string]: {
      apiKey: string;
      apiSecret: string;
    };
  };
  debug?: boolean;
  timeout?: number;
  retries?: number;
}

/**
 * Transaction options
 */
export interface TransactionOptions {
  feeRate?: 'slow' | 'medium' | 'fast' | number;
  gasPrice?: string;
  gasLimit?: string;
  nonce?: number;
  memo?: string;
  rbf?: boolean; // Replace-By-Fee
  confirmations?: number;
}

/**
 * Transfer parameters
 */
export interface TransferParams {
  blockchain: BlockchainNetwork;
  from: string;
  to: string;
  amount: string;
  options?: TransactionOptions;
  metadata?: Record<string, any>;
}

// ============================================================================
// Analytics & Monitoring
// ============================================================================

/**
 * Portfolio overview
 */
export interface Portfolio {
  totalValue: {
    usd: number;
    btc: number;
    eth: number;
  };
  assets: PortfolioAsset[];
  performance: Performance;
  lastUpdated: string;
}

/**
 * Portfolio asset
 */
export interface PortfolioAsset {
  symbol: string;
  name: string;
  amount: string;
  value: number;
  percentage: number;
  change24h: number;
  blockchain?: BlockchainNetwork;
}

/**
 * Performance metrics
 */
export interface Performance {
  day: number;
  week: number;
  month: number;
  year: number;
  allTime: number;
}

/**
 * Transaction history entry
 */
export interface TransactionHistoryEntry {
  transactionId: string;
  blockchain: BlockchainNetwork;
  type: TransactionType;
  from: string;
  to: string;
  amount: string;
  fee: string;
  status: TransactionStatus;
  timestamp: string;
  confirmations: number;
  usdValue?: number;
}

/**
 * Price alert
 */
export interface PriceAlert {
  alertId: string;
  cryptocurrency: string;
  condition: 'price_above' | 'price_below' | 'volume_spike' | 'percent_change';
  threshold: number;
  notification: {
    channels: ('email' | 'sms' | 'push')[];
    message?: string;
  };
  active: boolean;
}

// ============================================================================
// Events
// ============================================================================

/**
 * Transaction event
 */
export interface TransactionEvent {
  type: 'pending' | 'confirmed' | 'failed';
  transaction: WIATransaction;
  confirmations?: number;
}

/**
 * Block event
 */
export interface BlockEvent {
  type: 'new_block';
  block: Block;
  blockchain: BlockchainNetwork;
}

/**
 * Price event
 */
export interface PriceEvent {
  type: 'price_update';
  cryptocurrency: string;
  price: string;
  change24h: number;
  timestamp: string;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA Cryptocurrency Error
 */
export class WIACryptocurrencyError extends Error {
  constructor(
    message: string,
    public code: string,
    public details?: any
  ) {
    super(message);
    this.name = 'WIACryptocurrencyError';
  }
}

/**
 * Error codes
 */
export enum ErrorCode {
  INVALID_ADDRESS = 'INVALID_ADDRESS',
  INSUFFICIENT_BALANCE = 'INSUFFICIENT_BALANCE',
  INVALID_AMOUNT = 'INVALID_AMOUNT',
  NETWORK_ERROR = 'NETWORK_ERROR',
  TRANSACTION_FAILED = 'TRANSACTION_FAILED',
  INVALID_SIGNATURE = 'INVALID_SIGNATURE',
  UNSUPPORTED_BLOCKCHAIN = 'UNSUPPORTED_BLOCKCHAIN',
  INVALID_PRIVATE_KEY = 'INVALID_PRIVATE_KEY',
  API_ERROR = 'API_ERROR',
  RATE_LIMIT_EXCEEDED = 'RATE_LIMIT_EXCEEDED',
  COMPLIANCE_ERROR = 'COMPLIANCE_ERROR'
}

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Pagination parameters
 */
export interface PaginationParams {
  page?: number;
  limit?: number;
  offset?: number;
}

/**
 * Date range filter
 */
export interface DateRangeFilter {
  startDate?: string;
  endDate?: string;
}

/**
 * Sort order
 */
export type SortOrder = 'asc' | 'desc';

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
    version: string;
  };
}
