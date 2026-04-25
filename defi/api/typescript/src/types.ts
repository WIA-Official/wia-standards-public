/**
 * WIA-FIN-006 DeFi Standard TypeScript Type Definitions
 * @version 1.0.0
 * @standard WIA-FIN-006
 */

// ========================================
// Common Types
// ========================================

export interface WIAMetadata {
  version: string;
  standard: 'WIA-FIN-006';
  timestamp: string;
  requestId?: string;
}

export interface ChainInfo {
  name: string;
  chainId: number;
  network: 'mainnet' | 'testnet' | 'regtest';
}

// ========================================
// Token Types
// ========================================

export interface TokenMetadata {
  address: string;
  symbol: string;
  name: string;
  decimals: number;
  totalSupply?: string;
  circulatingSupply?: string;
  maxSupply?: string;
  price?: TokenPrice;
  holders?: number;
  transfers24h?: number;
}

export interface TokenPrice {
  usd: number;
  btc?: number;
  eth?: number;
  change24h: number;
  change7d?: number;
  marketCap: number;
  volume24h: number;
}

// ========================================
// Pool Types
// ========================================

export type PoolType = 'AMM' | 'Lending' | 'Staking' | 'Farming';
export type AMMType = 'ConstantProduct' | 'ConcentratedLiquidity' | 'StableSwap' | 'Weighted';

export interface PoolToken {
  address: string;
  symbol: string;
  name: string;
  decimals: number;
  reserve: string;
  weight?: number;
}

export interface PoolMetrics {
  tvl: number;
  tvlChange24h?: number;
  volume24h: number;
  volume7d?: number;
  fees24h: number;
  feeRate: number;
  apy: number;
  apr?: number;
  lpTokenSupply?: string;
  lpTokenPrice?: number;
  utilization?: number;
}

export interface LiquidityPool {
  version: string;
  standard: 'WIA-FIN-006';
  poolType: PoolType;
  poolId: string;
  protocol: string;
  chain: ChainInfo;
  contractAddress: string;
  tokens: PoolToken[];
  metrics: PoolMetrics;
  timestamp: string;
  metadata?: PoolMetadata;
}

export interface PoolMetadata {
  createdAt?: string;
  createdBlock?: number;
  lastUpdated?: string;
  audits?: AuditInfo[];
}

export interface AuditInfo {
  auditor: string;
  date: string;
  score?: number;
  url?: string;
}

// ========================================
// AMM Types
// ========================================

export interface SwapQuote {
  amountIn: string;
  amountOut: string;
  amountOutMin: string;
  priceImpact: number;
  route: SwapRoute[];
  estimatedGas: string;
  gasCostUSD: string;
  deadline?: number;
}

export interface SwapRoute {
  protocol: string;
  poolAddress: string;
  fee: number;
  tokenIn: string;
  tokenOut: string;
  amountIn: string;
  amountOut: string;
}

export interface SwapTransaction {
  to: string;
  data: string;
  value: string;
  gasLimit: string;
}

// ========================================
// Lending Types
// ========================================

export interface LendingMarket {
  version: string;
  standard: 'WIA-FIN-006';
  protocolType: 'Lending';
  protocol: string;
  marketId: string;
  chain: ChainInfo;
  assets: LendingAsset[];
  totalMarketSize: number;
  totalBorrowed: number;
  totalAvailable: number;
  timestamp: string;
}

export interface LendingAsset {
  asset: string;
  symbol: string;
  aToken?: string;
  stableDebtToken?: string;
  variableDebtToken?: string;
  totalSupplied: string;
  totalBorrowed: string;
  supplyAPY: number;
  variableBorrowAPY: number;
  stableBorrowAPY?: number;
  utilizationRate: number;
  liquidityRate: number;
  availableLiquidity: string;
  ltv: number;
  liquidationThreshold: number;
  liquidationBonus: number;
  reserveFactor: number;
  isActive: boolean;
  isFrozen: boolean;
  borrowingEnabled: boolean;
  stableBorrowingEnabled?: boolean;
}

export interface UserLendingPosition {
  version: string;
  standard: 'WIA-FIN-006';
  positionType: 'Lending';
  user: string;
  protocol: string;
  chain: ChainInfo;
  supplied: PositionAsset[];
  borrowed: PositionAsset[];
  healthFactor: number;
  totalCollateralUSD: number;
  totalBorrowedUSD: number;
  availableBorrowsUSD: number;
  currentLTV: number;
  liquidationThreshold: number;
  timestamp: string;
}

export interface PositionAsset {
  asset: string;
  symbol: string;
  amount: string;
  amountUSD: number;
  apy: number;
  isCollateral?: boolean;
  rateMode?: 'stable' | 'variable';
}

// ========================================
// Yield Farming Types
// ========================================

export type FarmType = 'LiquidityMining' | 'SingleStaking' | 'VaultStrategy';

export interface YieldFarm {
  version: string;
  standard: 'WIA-FIN-006';
  farmType: FarmType;
  farmId: string;
  protocol: string;
  chain: ChainInfo;
  stakingToken: TokenMetadata;
  rewardTokens: RewardToken[];
  metrics: FarmMetrics;
  duration?: FarmDuration;
  timestamp: string;
}

export interface RewardToken {
  address: string;
  symbol: string;
  emissionRate: string;
  emissionPeriod: 'per_block' | 'per_second' | 'per_day';
}

export interface FarmMetrics {
  totalStaked: string;
  totalStakedUSD: number;
  apy: number;
  apr: number;
  dailyRewards: string;
  tvl: number;
}

export interface FarmDuration {
  startBlock?: number;
  endBlock?: number;
  startTime?: string;
  endTime?: string;
}

export interface UserFarmPosition {
  version: string;
  standard: 'WIA-FIN-006';
  positionType: 'YieldFarming';
  user: string;
  farmId: string;
  protocol: string;
  staked: StakedInfo;
  rewards: RewardsInfo;
  performance: PerformanceMetrics;
  timestamp: string;
}

export interface StakedInfo {
  amount: string;
  amountUSD: number;
  stakedAt: string;
  stakedBlock?: number;
}

export interface RewardsInfo {
  pending: PendingReward[];
  claimed: ClaimedReward[];
}

export interface PendingReward {
  token: string;
  amount: string;
  amountUSD: number;
}

export interface ClaimedReward {
  token: string;
  amount: string;
  amountUSD: number;
  claimedAt: string;
}

export interface PerformanceMetrics {
  apy: number;
  dailyReward: string;
  projectedYearlyReward: string;
}

// ========================================
// Flash Loan Types
// ========================================

export interface FlashLoanRequest {
  version: string;
  standard: 'WIA-FIN-006';
  transactionType: 'FlashLoan';
  protocol: string;
  chain: ChainInfo;
  borrower: string;
  loans: FlashLoan[];
  execution: FlashLoanExecution;
  result?: FlashLoanResult;
  transactionHash?: string;
  blockNumber?: number;
  timestamp: string;
}

export interface FlashLoan {
  asset: string;
  symbol: string;
  amount: string;
  fee: string;
  feeRate: number;
  repayAmount: string;
}

export interface FlashLoanExecution {
  mode: string;
  target: string;
  params: string;
  operations: string[];
}

export interface FlashLoanResult {
  success: boolean;
  gasUsed: number;
  gasPrice: string;
  profit?: {
    asset: string;
    amount: string;
    amountUSD: number;
  };
}

// ========================================
// Governance Types
// ========================================

export type ProposalStatus =
  | 'Active'
  | 'Pending'
  | 'Succeeded'
  | 'Defeated'
  | 'Queued'
  | 'Executed'
  | 'Canceled'
  | 'Expired';

export type VoteSupport = 'For' | 'Against' | 'Abstain';

export interface GovernanceProposal {
  version: string;
  standard: 'WIA-FIN-006';
  governanceType: 'DAO';
  protocol: string;
  proposalId: string;
  title: string;
  description: string;
  proposer: string;
  status: ProposalStatus;
  voting: VotingInfo;
  execution?: ExecutionInfo;
  timestamp: string;
}

export interface VotingInfo {
  startBlock: number;
  endBlock: number;
  startTime: string;
  endTime: string;
  quorum: string;
  votesFor: string;
  votesAgainst: string;
  votesAbstain: string;
  totalVotes: string;
  quorumReached: boolean;
  currentResult: VoteSupport;
}

export interface ExecutionInfo {
  eta: string;
  targets: string[];
  values: string[];
  signatures: string[];
  calldatas: string[];
}

export interface UserVote {
  version: string;
  standard: 'WIA-FIN-006';
  voteType: 'GovernanceVote';
  proposalId: string;
  voter: string;
  support: VoteSupport;
  votingPower: string;
  reason?: string;
  transactionHash: string;
  blockNumber: number;
  timestamp: string;
}

// ========================================
// Oracle Types
// ========================================

export type OracleType = 'Chainlink' | 'TWAP' | 'Custom';

export interface PriceOracle {
  version: string;
  standard: 'WIA-FIN-006';
  oracleType: OracleType;
  feedAddress?: string;
  pair: string;
  chain: ChainInfo;
  latestRound?: ChainlinkRound;
  twapPrice?: TWAPPrice;
  timestamp: string;
}

export interface ChainlinkRound {
  roundId: string;
  answer: string;
  decimals: number;
  price: number;
  updatedAt: string;
  answeredInRound: string;
}

export interface TWAPPrice {
  period: number;
  price: number;
  token0: string;
  token1: string;
}

// ========================================
// API Response Types
// ========================================

export interface APIResponse<T> {
  success: boolean;
  data?: T;
  error?: APIError;
  meta: APIMetadata;
}

export interface APIError {
  code: string;
  message: string;
  details?: any;
}

export interface APIMetadata {
  timestamp: string;
  requestId: string;
}

export interface PaginatedResponse<T> {
  items: T[];
  pagination: PaginationInfo;
}

export interface PaginationInfo {
  hasMore: boolean;
  cursor?: string;
  total?: number;
}

// ========================================
// SDK Configuration
// ========================================

export interface WIADeFiConfig {
  apiKey: string;
  chainId?: number;
  provider?: any;
  baseURL?: string;
  timeout?: number;
}
