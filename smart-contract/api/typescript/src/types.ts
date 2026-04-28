/**
 * WIA-FIN-007 Smart Contract SDK Type Definitions
 * @packageDocumentation
 */

import { BigNumber, ethers } from 'ethers';

// ============================================================================
// Core Types
// ============================================================================

export type ChainId = number;
export type Address = string;
export type Hash = string;

export type SupportedChain =
  | 'ethereum'
  | 'polygon'
  | 'arbitrum'
  | 'optimism'
  | 'bsc'
  | 'avalanche';

export type GasStrategy = 'slow' | 'standard' | 'fast' | 'instant';

// ============================================================================
// Contract Metadata
// ============================================================================

export interface WIAContractMetadata {
  wiaVersion: string;
  contractInfo: ContractInfo;
  security: SecurityConfig;
  deployment: DeploymentConfig;
  features: ContractFeatures;
  gasOptimization: GasOptimizationConfig;
  standards: StandardsConfig;
  dependencies: Record<string, string>;
}

export interface ContractInfo {
  name: string;
  type: string;
  version: string;
  description: string;
  author: string;
  license: string;
  repository?: string;
  documentation?: string;
}

export interface SecurityConfig {
  audited: boolean;
  auditor?: string;
  auditDate?: string;
  auditReport?: string;
  vulnerabilities: string[];
  securityLevel: 'low' | 'medium' | 'high';
  reentrancyGuard: boolean;
  overflowProtection: boolean;
  accessControl: AccessControl;
  pausable: boolean;
  timelocked: boolean;
}

export interface AccessControl {
  type: string;
  roles: string[];
}

export interface DeploymentConfig {
  networks: NetworkDeployment[];
  upgradeable: boolean;
  upgradePattern?: string;
  proxyAddress?: Address;
  implementationAddress?: Address;
  adminAddress?: Address;
  initializer?: string;
  constructorArgs?: any[];
}

export interface NetworkDeployment {
  chainId: ChainId;
  name: SupportedChain;
  address: Address;
  deployedAt: string;
  blockNumber?: number;
  transactionHash?: Hash;
}

export interface ContractFeatures {
  mintable: boolean;
  burnable: boolean;
  pausable: boolean;
  snapshot: boolean;
  permit: boolean;
  flashMint: boolean;
  voting: boolean;
}

export interface GasOptimizationConfig {
  level: 'low' | 'medium' | 'high';
  techniques: string[];
  estimatedDeployGas: number;
  estimatedTransferGas: number;
  benchmarks: Record<string, GasBenchmark>;
}

export interface GasBenchmark {
  min: number;
  avg: number;
  max: number;
}

export interface StandardsConfig {
  implements: string[];
  compatible: string[];
}

// ============================================================================
// SDK Configuration
// ============================================================================

export interface WIAContractConfig {
  address?: Address;
  addresses?: Record<ChainId, Address>;
  abi: ethers.InterfaceAbi;
  provider?: ethers.Provider;
  signer?: ethers.Signer;
  chain?: SupportedChain | ChainId;
  defaultChain?: SupportedChain | ChainId;
  analytics?: AnalyticsConfig;
}

export interface AnalyticsConfig {
  enabled: boolean;
  endpoint?: string;
  apiKey?: string;
}

// ============================================================================
// Transaction Types
// ============================================================================

export interface TxOptions {
  gasLimit?: BigNumber;
  maxPriorityFeePerGas?: BigNumber;
  maxFeePerGas?: BigNumber;
  nonce?: number;
  value?: BigNumber;
  maxSlippage?: number;
}

export interface TransactionResponse {
  hash: Hash;
  from: Address;
  to: Address;
  value: BigNumber;
  gasLimit: BigNumber;
  maxPriorityFeePerGas?: BigNumber;
  maxFeePerGas?: BigNumber;
  nonce: number;
  data: string;
  chainId: ChainId;
  wait(confirmations?: number): Promise<TransactionReceipt>;
}

export interface TransactionReceipt {
  transactionHash: Hash;
  blockNumber: number;
  blockHash: Hash;
  from: Address;
  to: Address;
  gasUsed: BigNumber;
  effectiveGasPrice: BigNumber;
  status: number;
  logs: Log[];
  events?: Event[];
}

export interface Log {
  address: Address;
  topics: string[];
  data: string;
  blockNumber: number;
  transactionHash: Hash;
  logIndex: number;
}

export interface Event {
  event: string;
  args: any[];
  address: Address;
  blockNumber: number;
  transactionHash: Hash;
}

// ============================================================================
// Event Handling
// ============================================================================

export type EventListener = (...args: any[]) => void;

export interface EventFilter {
  address?: Address;
  topics?: (string | string[])[];
  fromBlock?: number | 'latest';
  toBlock?: number | 'latest';
}

export interface PaginationOptions {
  eventName: string;
  fromBlock: number;
  toBlock: number | 'latest';
  pageSize: number;
  filter?: Record<string, any>;
}

export interface PaginatedEvents {
  events: Event[];
  hasMore: boolean;
  nextPage?: number;
}

export interface StreamOptions {
  fromBlock: number | 'latest';
  filter?: Record<string, any>;
}

// ============================================================================
// Multi-Call
// ============================================================================

export interface MulticallRequest {
  method: string;
  args: any[];
}

export interface MulticallResponse {
  success: boolean;
  returnData: any;
  error?: Error;
}

// ============================================================================
// Gas Estimation
// ============================================================================

export interface GasEstimate {
  gasLimit: BigNumber;
  maxPriorityFeePerGas: BigNumber;
  maxFeePerGas: BigNumber;
  estimatedCost: BigNumber;
  strategy: GasStrategy;
}

// ============================================================================
// Wallet Types
// ============================================================================

export type WalletType =
  | 'metamask'
  | 'walletconnect'
  | 'coinbase'
  | 'trust'
  | 'ledger'
  | 'trezor'
  | 'safe';

export interface WalletConnection {
  type: WalletType;
  address: Address;
  chainId: ChainId;
  isConnected: boolean;
  isHardwareWallet: boolean;
}

export interface WalletConnectorConfig {
  supportedWallets: WalletType[];
  supportedChains: ChainId[];
  appMetadata: AppMetadata;
}

export interface AppMetadata {
  name: string;
  description: string;
  url: string;
  icons: string[];
}

// ============================================================================
// Error Types
// ============================================================================

export class WIAContractError extends Error {
  constructor(
    message: string,
    public code: string,
    public details?: any
  ) {
    super(message);
    this.name = 'WIAContractError';
  }
}

export class WIAInsufficientBalanceError extends WIAContractError {
  constructor(
    public account: Address,
    public requested: BigNumber,
    public available: BigNumber
  ) {
    super(
      `Insufficient balance: requested ${requested.toString()}, available ${available.toString()}`,
      'INSUFFICIENT_BALANCE',
      { account, requested, available }
    );
    this.name = 'WIAInsufficientBalanceError';
  }
}

export class WIAUnauthorizedError extends WIAContractError {
  constructor(
    public caller: Address,
    public requiredRole: string
  ) {
    super(
      `Unauthorized: ${caller} requires role ${requiredRole}`,
      'UNAUTHORIZED',
      { caller, requiredRole }
    );
    this.name = 'WIAUnauthorizedError';
  }
}

export class WIAContractPausedError extends WIAContractError {
  constructor() {
    super('Contract is paused', 'CONTRACT_PAUSED');
    this.name = 'WIAContractPausedError';
  }
}

export class WIAInvalidAddressError extends WIAContractError {
  constructor(
    public address: Address,
    public reason: string
  ) {
    super(
      `Invalid address ${address}: ${reason}`,
      'INVALID_ADDRESS',
      { address, reason }
    );
    this.name = 'WIAInvalidAddressError';
  }
}

// ============================================================================
// Deployment Types
// ============================================================================

export interface DeploymentOptions {
  contract: string;
  bytecode: string;
  constructorArgs?: any[];
  salt?: string;
  chains: SupportedChain[];
  initializeData?: Record<string, any>;
  verify?: boolean;
}

export interface DeploymentResult {
  addresses: Record<SupportedChain, Address>;
  allAddressesMatch: boolean;
  deploymentInfo: Record<SupportedChain, NetworkDeployment>;
}

// ============================================================================
// Analytics Types
// ============================================================================

export interface ContractMetrics {
  totalTransactions: number;
  avgGasUsed: number;
  successRate: number;
  avgConfirmationTime: number;
  uniqueUsers: number;
}

// ============================================================================
// Permit (EIP-2612)
// ============================================================================

export interface PermitSignature {
  v: number;
  r: string;
  s: string;
  deadline: number;
}

export interface TypedData {
  domain: {
    name: string;
    version: string;
    chainId: ChainId;
    verifyingContract: Address;
  };
  types: Record<string, Array<{ name: string; type: string }>>;
  value: Record<string, any>;
}
