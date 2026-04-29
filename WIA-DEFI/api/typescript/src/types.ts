/**
 * WIA-DEFI TypeScript SDK - Type Definitions
 * Version: 1.0.0
 * Philosophy: 弘益人間 (Benefit All Humanity)
 */

/**
 * Protocol information
 */
export interface Protocol {
  protocolId: string;
  name: string;
  version: string;
  chainId: number;
  tvl: bigint;
  tvlChange24h: number;
  totalBorrowed: bigint;
  totalSupplied: bigint;
  utilization: number;
  markets: number;
  activeUsers24h: number;
  volume24h: bigint;
  deployedAt: Date;
  contracts: ProtocolContracts;
  links?: ProtocolLinks;
}

export interface ProtocolContracts {
  pool: string;
  router?: string;
  factory?: string;
  oracle?: string;
  [key: string]: string | undefined;
}

export interface ProtocolLinks {
  website?: string;
  docs?: string;
  github?: string;
  discord?: string;
  twitter?: string;
}

/**
 * Token information
 */
export interface Token {
  address: string;
  symbol: string;
  name: string;
  decimals: number;
  priceUsd: number;
  volume24h?: bigint;
  totalSupply?: bigint;
  logoUri?: string;
}

/**
 * Liquidity Pool
 */
export interface LiquidityPool {
  poolId: string;
  protocol: string;
  chainId: number;
  token0: Token;
  token1: Token;
  reserve0: bigint;
  reserve1: bigint;
  tvl: bigint;
  volume24h: bigint;
  fees24h: bigint;
  apr: number;
  feeTier: number;
  liquidity: bigint;
  sqrtPriceX96: bigint;
  tick: number;
  hooks?: string[];
  createdAt: Date;
}

/**
 * User positions
 */
export type Position = LiquidityPosition | LendingPosition | StakingPosition;

export interface BasePosition {
  positionId: string;
  type: 'liquidity' | 'lending' | 'staking';
  protocol: string;
  owner: string;
  valueUsd: number;
  createdAt: Date;
}

export interface LiquidityPosition extends BasePosition {
  type: 'liquidity';
  poolId: string;
  token0: string;
  token1: string;
  liquidity: bigint;
  amount0: bigint;
  amount1: bigint;
  tickLower: number;
  tickUpper: number;
  unclaimedFees0: bigint;
  unclaimedFees1: bigint;
  inRange: boolean;
}

export interface LendingPosition extends BasePosition {
  type: 'lending';
  asset: string;
  supplied: bigint;
  borrowed: bigint;
  suppliedUsd: number;
  borrowedUsd: number;
  apy: number;
  healthFactor?: number;
  rewards?: Reward[];
}

export interface StakingPosition extends BasePosition {
  type: 'staking';
  asset: string;
  staked: bigint;
  stakedUsd: number;
  rewards: bigint;
  rewardsUsd: number;
  apr: number;
}

export interface Reward {
  token: string;
  amount: bigint;
  valueUsd: number;
}

/**
 * Swap quote
 */
export interface SwapQuote {
  quoteId: string;
  chainId: number;
  tokenIn: Token;
  tokenOut: Token;
  amountIn: bigint;
  amountOut: bigint;
  priceImpact: number;
  route: RouteStep[];
  estimatedGas: number;
  gasPriceGwei: number;
  gasCostUsd: number;
  expiresAt: Date;
  transaction: TransactionData;
}

export interface RouteStep {
  protocol: string;
  poolId: string;
  tokenIn: string;
  tokenOut: string;
  percentage: number;
}

export interface TransactionData {
  to: string;
  data: string;
  value: string;
  gasLimit?: number;
}

/**
 * Swap transaction
 */
export interface Swap {
  id: string;
  pool: string;
  sender: string;
  recipient: string;
  amount0: bigint;
  amount1: bigint;
  amountUsd: number;
  sqrtPriceX96: bigint;
  tick: number;
  timestamp: Date;
  transactionHash: string;
  logIndex: number;
}

/**
 * Lending operations
 */
export interface LendingMarket {
  marketId: string;
  protocol: string;
  asset: Token;
  totalSupplied: bigint;
  totalBorrowed: bigint;
  supplyApy: number;
  borrowApy: number;
  utilizationRate: number;
  collateralFactor: number;
  liquidationThreshold: number;
  liquidationPenalty: number;
}

export interface BorrowParams {
  asset: string;
  amount: bigint;
  interestRateMode: 'stable' | 'variable';
}

export interface SupplyParams {
  asset: string;
  amount: bigint;
  onBehalfOf?: string;
}

/**
 * Staking operations
 */
export interface StakingPool {
  poolId: string;
  protocol: string;
  asset: Token;
  rewardToken: Token;
  totalStaked: bigint;
  apr: number;
  lockPeriod?: number; // in seconds
  minStake?: bigint;
  maxStake?: bigint;
}

/**
 * Query parameters
 */
export interface PoolsQueryParams {
  protocol?: string;
  chain?: number;
  token0?: string;
  token1?: string;
  minTvl?: number;
  sortBy?: 'tvl' | 'volume24h' | 'apr' | 'createdAt';
  order?: 'asc' | 'desc';
  limit?: number;
  offset?: number;
}

export interface SwapQuoteParams {
  chainId: number;
  tokenIn: string;
  tokenOut: string;
  amountIn: string;
  slippageTolerance: number;
  recipient?: string;
  deadline?: number;
}

export interface PositionsQueryParams {
  address: string;
  protocols?: string[];
  types?: Array<'liquidity' | 'lending' | 'staking'>;
  minValueUsd?: number;
}

/**
 * API Configuration
 */
export interface SDKConfig {
  apiKey: string;
  network: 'mainnet' | 'sepolia' | 'arbitrum' | 'optimism' | 'base';
  baseURL?: string;
  timeout?: number;
  retries?: number;
}

/**
 * API Response types
 */
export interface APIResponse<T> {
  data: T;
  success: boolean;
  timestamp: Date;
}

export interface PaginatedResponse<T> {
  data: T[];
  total: number;
  limit: number;
  offset: number;
}

export interface APIError {
  code: string;
  message: string;
  details?: Record<string, any>;
}

/**
 * WebSocket types
 */
export interface StreamOptions {
  poolIds?: string[];
  minAmountUsd?: number;
  onUpdate: (data: any) => void;
  onError?: (error: Error) => void;
}

export interface PoolUpdateEvent {
  type: 'pool_update';
  timestamp: Date;
  data: {
    poolId: string;
    reserve0: bigint;
    reserve1: bigint;
    sqrtPriceX96: bigint;
    tick: number;
  };
}

export interface SwapEvent {
  type: 'swap';
  timestamp: Date;
  data: Swap;
}

export type StreamEvent = PoolUpdateEvent | SwapEvent;

/**
 * Chain configurations
 */
export interface ChainConfig {
  chainId: number;
  name: string;
  rpcUrl: string;
  explorer: string;
  nativeCurrency: {
    name: string;
    symbol: string;
    decimals: number;
  };
}

export const SUPPORTED_CHAINS: Record<string, ChainConfig> = {
  mainnet: {
    chainId: 1,
    name: 'Ethereum Mainnet',
    rpcUrl: 'https://eth-mainnet.g.alchemy.com/v2/',
    explorer: 'https://etherscan.io',
    nativeCurrency: {
      name: 'Ether',
      symbol: 'ETH',
      decimals: 18,
    },
  },
  sepolia: {
    chainId: 11155111,
    name: 'Sepolia Testnet',
    rpcUrl: 'https://eth-sepolia.g.alchemy.com/v2/',
    explorer: 'https://sepolia.etherscan.io',
    nativeCurrency: {
      name: 'Sepolia Ether',
      symbol: 'SEP',
      decimals: 18,
    },
  },
  arbitrum: {
    chainId: 42161,
    name: 'Arbitrum One',
    rpcUrl: 'https://arb-mainnet.g.alchemy.com/v2/',
    explorer: 'https://arbiscan.io',
    nativeCurrency: {
      name: 'Ether',
      symbol: 'ETH',
      decimals: 18,
    },
  },
  optimism: {
    chainId: 10,
    name: 'Optimism',
    rpcUrl: 'https://opt-mainnet.g.alchemy.com/v2/',
    explorer: 'https://optimistic.etherscan.io',
    nativeCurrency: {
      name: 'Ether',
      symbol: 'ETH',
      decimals: 18,
    },
  },
  base: {
    chainId: 8453,
    name: 'Base',
    rpcUrl: 'https://base-mainnet.g.alchemy.com/v2/',
    explorer: 'https://basescan.org',
    nativeCurrency: {
      name: 'Ether',
      symbol: 'ETH',
      decimals: 18,
    },
  },
};

/**
 * Utility types
 */
export type Address = `0x${string}`;
export type Hex = `0x${string}`;
export type BigIntish = bigint | string | number;
