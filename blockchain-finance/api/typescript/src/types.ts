/**
 * WIA Blockchain Finance Standard - TypeScript Type Definitions
 *
 * Comprehensive type definitions for blockchain transactions, tokens, and DeFi operations
 *
 * @module @wia/blockchain-finance/types
 * @license MIT
 */

// ============================================================================
// Core Blockchain Types
// ============================================================================

/**
 * Supported blockchain networks
 */
export enum BlockchainNetwork {
  ETHEREUM = 'ethereum',
  POLYGON = 'polygon',
  BSC = 'bsc',
  AVALANCHE = 'avalanche',
  ARBITRUM = 'arbitrum',
  OPTIMISM = 'optimism',
  SOLANA = 'solana',
  POLKADOT = 'polkadot',
  COSMOS = 'cosmos',
  NEAR = 'near',
}

/**
 * Transaction status
 */
export enum TransactionStatus {
  PENDING = 'pending',
  CONFIRMED = 'confirmed',
  FAILED = 'failed',
  CANCELLED = 'cancelled',
}

/**
 * Base blockchain transaction
 */
export interface BlockchainTransaction {
  /** Transaction hash */
  hash: string;
  /** Sender address */
  from: string;
  /** Recipient address */
  to: string;
  /** Amount in wei/smallest unit */
  value: string;
  /** Gas price in wei */
  gasPrice?: string;
  /** Gas limit */
  gasLimit?: string;
  /** Transaction nonce */
  nonce: number;
  /** Transaction data/input */
  data?: string;
  /** Chain ID */
  chainId: number;
  /** Block number */
  blockNumber?: number;
  /** Transaction status */
  status: TransactionStatus;
  /** Timestamp */
  timestamp: number;
}

// ============================================================================
// Token Standards (ERC-20, ERC-721, ERC-1155)
// ============================================================================

/**
 * Token standard types
 */
export enum TokenStandard {
  ERC20 = 'ERC20',
  ERC721 = 'ERC721',
  ERC1155 = 'ERC1155',
  BEP20 = 'BEP20',
  SPL = 'SPL',
}

/**
 * ERC-20 Fungible Token
 */
export interface ERC20Token {
  /** Token standard */
  standard: TokenStandard.ERC20;
  /** Contract address */
  address: string;
  /** Token name */
  name: string;
  /** Token symbol */
  symbol: string;
  /** Number of decimals */
  decimals: number;
  /** Total supply */
  totalSupply: string;
  /** Network */
  network: BlockchainNetwork;
}

/**
 * ERC-20 Transfer
 */
export interface ERC20Transfer {
  /** Token contract address */
  tokenAddress: string;
  /** Sender address */
  from: string;
  /** Recipient address */
  to: string;
  /** Amount in smallest unit */
  amount: string;
  /** Transaction hash */
  txHash?: string;
}

/**
 * ERC-721 Non-Fungible Token (NFT)
 */
export interface ERC721Token {
  /** Token standard */
  standard: TokenStandard.ERC721;
  /** Contract address */
  address: string;
  /** Token ID */
  tokenId: string;
  /** Token name */
  name: string;
  /** Token symbol */
  symbol: string;
  /** Token URI (metadata) */
  tokenURI: string;
  /** Owner address */
  owner: string;
  /** Network */
  network: BlockchainNetwork;
}

/**
 * ERC-721 Transfer
 */
export interface ERC721Transfer {
  /** Token contract address */
  tokenAddress: string;
  /** Sender address */
  from: string;
  /** Recipient address */
  to: string;
  /** Token ID */
  tokenId: string;
  /** Transaction hash */
  txHash?: string;
}

/**
 * ERC-1155 Multi-Token
 */
export interface ERC1155Token {
  /** Token standard */
  standard: TokenStandard.ERC1155;
  /** Contract address */
  address: string;
  /** Token ID */
  tokenId: string;
  /** Amount/balance */
  amount: string;
  /** Token URI (metadata) */
  uri: string;
  /** Owner address */
  owner: string;
  /** Network */
  network: BlockchainNetwork;
}

/**
 * ERC-1155 Transfer
 */
export interface ERC1155Transfer {
  /** Token contract address */
  tokenAddress: string;
  /** Sender address */
  from: string;
  /** Recipient address */
  to: string;
  /** Token ID */
  tokenId: string;
  /** Amount to transfer */
  amount: string;
  /** Transaction hash */
  txHash?: string;
}

// ============================================================================
// DeFi Operations
// ============================================================================

/**
 * DeFi operation types
 */
export enum DeFiOperationType {
  SWAP = 'swap',
  STAKE = 'stake',
  UNSTAKE = 'unstake',
  LEND = 'lend',
  BORROW = 'borrow',
  FARM = 'farm',
  BRIDGE = 'bridge',
  LIQUIDITY_ADD = 'liquidity_add',
  LIQUIDITY_REMOVE = 'liquidity_remove',
}

/**
 * Token swap operation
 */
export interface SwapOperation {
  /** Operation type */
  type: DeFiOperationType.SWAP;
  /** DEX protocol (Uniswap, PancakeSwap, etc.) */
  protocol: string;
  /** Input token */
  tokenIn: string;
  /** Output token */
  tokenOut: string;
  /** Input amount */
  amountIn: string;
  /** Minimum output amount (slippage) */
  amountOutMin: string;
  /** Swap path (for multi-hop swaps) */
  path: string[];
  /** Deadline timestamp */
  deadline: number;
  /** Recipient address */
  recipient: string;
}

/**
 * Staking operation
 */
export interface StakeOperation {
  /** Operation type */
  type: DeFiOperationType.STAKE | DeFiOperationType.UNSTAKE;
  /** Staking protocol */
  protocol: string;
  /** Token to stake */
  token: string;
  /** Amount to stake/unstake */
  amount: string;
  /** Validator address (for PoS) */
  validator?: string;
  /** Lock period in seconds */
  lockPeriod?: number;
  /** APY (Annual Percentage Yield) */
  apy?: number;
}

/**
 * Lending operation
 */
export interface LendOperation {
  /** Operation type */
  type: DeFiOperationType.LEND | DeFiOperationType.BORROW;
  /** Lending protocol (Aave, Compound, etc.) */
  protocol: string;
  /** Asset to lend/borrow */
  asset: string;
  /** Amount */
  amount: string;
  /** Interest rate (APR) */
  interestRate: number;
  /** Collateral (for borrowing) */
  collateral?: {
    asset: string;
    amount: string;
  };
}

/**
 * Liquidity pool operation
 */
export interface LiquidityPoolOperation {
  /** Operation type */
  type: DeFiOperationType.LIQUIDITY_ADD | DeFiOperationType.LIQUIDITY_REMOVE;
  /** DEX protocol */
  protocol: string;
  /** Token A */
  tokenA: string;
  /** Token B */
  tokenB: string;
  /** Amount A */
  amountA: string;
  /** Amount B */
  amountB: string;
  /** Pool share (LP tokens) */
  lpTokens?: string;
  /** Minimum amounts (slippage protection) */
  minAmounts?: {
    tokenA: string;
    tokenB: string;
  };
}

/**
 * Cross-chain bridge operation
 */
export interface BridgeOperation {
  /** Operation type */
  type: DeFiOperationType.BRIDGE;
  /** Bridge protocol */
  protocol: string;
  /** Source chain */
  sourceChain: BlockchainNetwork;
  /** Destination chain */
  destinationChain: BlockchainNetwork;
  /** Token to bridge */
  token: string;
  /** Amount */
  amount: string;
  /** Recipient address on destination chain */
  recipient: string;
  /** Bridge fee */
  fee?: string;
}

// ============================================================================
// WIA Compliance Types
// ============================================================================

/**
 * WIA compliance level
 */
export enum ComplianceLevel {
  BASIC = 'basic',
  STANDARD = 'standard',
  ADVANCED = 'advanced',
  CERTIFIED = 'certified',
}

/**
 * WIA compliance metadata
 */
export interface WIACompliance {
  /** Compliance level */
  level: ComplianceLevel;
  /** Standard version */
  version: string;
  /** Certification ID */
  certificationId?: string;
  /** Certification date */
  certificationDate?: string;
  /** Audited by */
  auditedBy?: string[];
  /** Security score (0-100) */
  securityScore?: number;
}

/**
 * WIA transaction wrapper with compliance metadata
 */
export interface WIATransaction extends BlockchainTransaction {
  /** WIA compliance metadata */
  wiaCompliance: WIACompliance;
  /** KYC/AML compliance */
  kycCompliant?: boolean;
  /** Regulatory flags */
  regulatoryFlags?: string[];
}

/**
 * Smart contract metadata
 */
export interface SmartContractMetadata {
  /** Contract address */
  address: string;
  /** Contract name */
  name: string;
  /** ABI hash */
  abiHash: string;
  /** Deployment date */
  deployedAt: string;
  /** Creator address */
  creator: string;
  /** Source code verification */
  verified: boolean;
  /** Audit reports */
  audits?: {
    auditor: string;
    date: string;
    reportUrl: string;
  }[];
  /** WIA compliance */
  wiaCompliance?: WIACompliance;
}

// ============================================================================
// Wallet and Signature Types
// ============================================================================

/**
 * Wallet types
 */
export enum WalletType {
  METAMASK = 'metamask',
  WALLETCONNECT = 'walletconnect',
  COINBASE = 'coinbase',
  HARDWARE = 'hardware',
  CUSTODIAL = 'custodial',
}

/**
 * Wallet information
 */
export interface WalletInfo {
  /** Wallet type */
  type: WalletType;
  /** Wallet address */
  address: string;
  /** Connected network */
  network: BlockchainNetwork;
  /** Balance (native token) */
  balance?: string;
  /** Public key */
  publicKey?: string;
}

/**
 * Signature request
 */
export interface SignatureRequest {
  /** Message to sign */
  message: string;
  /** Signer address */
  signer: string;
  /** Message type (personal, typed data, etc.) */
  messageType: 'personal' | 'typed';
  /** Domain (for EIP-712) */
  domain?: any;
  /** Types (for EIP-712) */
  types?: any;
}

/**
 * Signature result
 */
export interface SignatureResult {
  /** Signature */
  signature: string;
  /** Signer address */
  signer: string;
  /** Recovery ID */
  v: number;
  /** R component */
  r: string;
  /** S component */
  s: string;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA error codes
 */
export enum WIAErrorCode {
  INVALID_ADDRESS = 'INVALID_ADDRESS',
  INSUFFICIENT_BALANCE = 'INSUFFICIENT_BALANCE',
  TRANSACTION_FAILED = 'TRANSACTION_FAILED',
  NETWORK_ERROR = 'NETWORK_ERROR',
  INVALID_SIGNATURE = 'INVALID_SIGNATURE',
  CONTRACT_ERROR = 'CONTRACT_ERROR',
  COMPLIANCE_ERROR = 'COMPLIANCE_ERROR',
  BRIDGE_ERROR = 'BRIDGE_ERROR',
}

/**
 * WIA error
 */
export class WIAError extends Error {
  constructor(
    public code: WIAErrorCode,
    message: string,
    public details?: any
  ) {
    super(message);
    this.name = 'WIAError';
  }
}
