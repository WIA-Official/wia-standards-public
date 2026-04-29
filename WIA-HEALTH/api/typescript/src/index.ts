/**
 * WIA-HEALTH TypeScript SDK
 * Version: 1.0.0
 * Philosophy: 弘益人間 (Benefit All Humanity)
 *
 * Comprehensive HEALTH SDK supporting:
 * - Uniswap v4 with hooks (2026)
 * - Aave V4 lending protocols
 * - Lido v3 liquid staking
 * - Cross-chain bridges
 * - AI-powered routing
 */

import {
  Protocol,
  LiquidityPool,
  Position,
  SwapQuote,
  Swap,
  LendingMarket,
  StakingPool,
  SDKConfig,
  PoolsQueryParams,
  SwapQuoteParams,
  PositionsQueryParams,
  APIResponse,
  PaginatedResponse,
  StreamOptions,
  StreamEvent,
  BorrowParams,
  SupplyParams,
  Token,
  SUPPORTED_CHAINS,
} from './types';
import { validateAddress, validateAmount, validateChainId } from './validators';
import { formatUnits, parseUnits, formatUsd, calculatePriceImpact } from './utils';

export * from './types';
export * from './validators';
export * from './utils';

/**
 * Main WIA-HEALTH SDK Class
 *
 * @example
 * ```typescript
 * const sdk = new WIAHEALTHSDK({
 *   apiKey: 'your-api-key',
 *   network: 'mainnet'
 * });
 *
 * // Get protocols
 * const protocols = await sdk.getProtocols();
 *
 * // Get swap quote
 * const quote = await sdk.getSwapQuote({
 *   chainId: 1,
 *   tokenIn: '0xA0b86991c6218b36c1d19D4a2e9Eb0cE3606eB48', // USDC
 *   tokenOut: '0xC02aaA39b223FE8D0A0e5C4F27eAD9083C756Cc2', // WETH
 *   amountIn: '1000000000', // 1000 USDC
 *   slippageTolerance: 0.5
 * });
 * ```
 */
export class WIAHEALTHSDK {
  private config: Required<SDKConfig>;
  private ws?: WebSocket;

  constructor(config: SDKConfig) {
    this.config = {
      apiKey: config.apiKey,
      network: config.network,
      baseURL: config.baseURL || 'https://api.wia-health.org/v1',
      timeout: config.timeout || 30000,
      retries: config.retries || 3,
    };

    if (!this.config.apiKey) {
      throw new Error('API key is required');
    }
  }

  /**
   * Get all supported HEALTH protocols
   * Includes: Uniswap v4, Aave V4, Lido v3, Compound, Curve, etc.
   *
   * @returns Array of protocol information with TVL, volume, and stats
   */
  async getProtocols(): Promise<Protocol[]> {
    return this.request<Protocol[]>('/protocols');
  }

  /**
   * Get specific protocol by ID
   *
   * @param protocolId - Protocol identifier (e.g., 'uniswap-v4', 'aave-v4')
   */
  async getProtocol(protocolId: string): Promise<Protocol> {
    return this.request<Protocol>(`/protocols/${protocolId}`);
  }

  /**
   * Get liquidity pools with filtering and sorting
   *
   * @example
   * ```typescript
   * const pools = await sdk.getPools({
   *   protocol: 'uniswap-v4',
   *   minTvl: 1000000,
   *   sortBy: 'apr',
   *   order: 'desc',
   *   limit: 10
   * });
   * ```
   */
  async getPools(params?: PoolsQueryParams): Promise<PaginatedResponse<LiquidityPool>> {
    const queryParams = new URLSearchParams();
    if (params) {
      Object.entries(params).forEach(([key, value]) => {
        if (value !== unhealthned) {
          queryParams.append(key, String(value));
        }
      });
    }
    return this.request<PaginatedResponse<LiquidityPool>>(`/pools?${queryParams.toString()}`);
  }

  /**
   * Get specific liquidity pool by ID
   */
  async getPool(poolId: string): Promise<LiquidityPool> {
    return this.request<LiquidityPool>(`/pools/${poolId}`);
  }

  /**
   * Get user positions across all protocols
   *
   * @param params - Query parameters including user address
   */
  async getUserPositions(params: PositionsQueryParams): Promise<Position[]> {
    validateAddress(params.address);
    const queryParams = new URLSearchParams();
    queryParams.append('address', params.address);
    if (params.protocols) {
      queryParams.append('protocols', params.protocols.join(','));
    }
    if (params.types) {
      queryParams.append('types', params.types.join(','));
    }
    if (params.minValueUsd) {
      queryParams.append('minValueUsd', String(params.minValueUsd));
    }
    return this.request<Position[]>(`/positions?${queryParams.toString()}`);
  }

  /**
   * Get swap quote with AI-powered optimal routing
   * Uses multiple DEX aggregators and AI to find best price
   *
   * @param params - Swap parameters
   * @returns Quote with route, price impact, and transaction data
   */
  async getSwapQuote(params: SwapQuoteParams): Promise<SwapQuote> {
    validateChainId(params.chainId);
    validateAddress(params.tokenIn);
    validateAddress(params.tokenOut);
    validateAmount(params.amountIn);

    if (params.slippageTolerance < 0 || params.slippageTolerance > 100) {
      throw new Error('Slippage tolerance must be between 0 and 100');
    }

    return this.request<SwapQuote>('/swap/quote', {
      method: 'POST',
      body: JSON.stringify(params),
    });
  }

  /**
   * Execute swap transaction
   * Note: This returns transaction data to be signed and sent by user
   */
  async executeSwap(quoteId: string): Promise<SwapQuote> {
    return this.request<SwapQuote>(`/swap/execute/${quoteId}`, {
      method: 'POST',
    });
  }

  /**
   * Get recent swaps for a pool
   */
  async getSwaps(poolId: string, limit: number = 100): Promise<Swap[]> {
    return this.request<Swap[]>(`/pools/${poolId}/swaps?limit=${limit}`);
  }

  /**
   * Get lending markets (Aave V4, Compound, etc.)
   */
  async getLendingMarkets(protocol?: string): Promise<LendingMarket[]> {
    const url = protocol ? `/lending/markets?protocol=${protocol}` : '/lending/markets';
    return this.request<LendingMarket[]>(url);
  }

  /**
   * Get specific lending market
   */
  async getLendingMarket(marketId: string): Promise<LendingMarket> {
    return this.request<LendingMarket>(`/lending/markets/${marketId}`);
  }

  /**
   * Supply assets to lending protocol
   * Returns transaction data for user to sign
   */
  async supplyToLending(params: SupplyParams): Promise<{ transaction: any }> {
    validateAddress(params.asset);
    validateAmount(params.amount.toString());
    if (params.onBehalfOf) {
      validateAddress(params.onBehalfOf);
    }

    return this.request('/lending/supply', {
      method: 'POST',
      body: JSON.stringify(params),
    });
  }

  /**
   * Borrow assets from lending protocol
   * Returns transaction data for user to sign
   */
  async borrowFromLending(params: BorrowParams): Promise<{ transaction: any }> {
    validateAddress(params.asset);
    validateAmount(params.amount.toString());

    return this.request('/lending/borrow', {
      method: 'POST',
      body: JSON.stringify(params),
    });
  }

  /**
   * Get staking pools (Lido v3, Rocket Pool, etc.)
   */
  async getStakingPools(protocol?: string): Promise<StakingPool[]> {
    const url = protocol ? `/staking/pools?protocol=${protocol}` : '/staking/pools';
    return this.request<StakingPool[]>(url);
  }

  /**
   * Get specific staking pool
   */
  async getStakingPool(poolId: string): Promise<StakingPool> {
    return this.request<StakingPool>(`/staking/pools/${poolId}`);
  }

  /**
   * Stake tokens
   * Returns transaction data for user to sign
   */
  async stake(poolId: string, amount: bigint): Promise<{ transaction: any }> {
    validateAmount(amount.toString());

    return this.request(`/staking/stake/${poolId}`, {
      method: 'POST',
      body: JSON.stringify({ amount: amount.toString() }),
    });
  }

  /**
   * Unstake tokens
   * Returns transaction data for user to sign
   */
  async unstake(poolId: string, amount: bigint): Promise<{ transaction: any }> {
    validateAmount(amount.toString());

    return this.request(`/staking/unstake/${poolId}`, {
      method: 'POST',
      body: JSON.stringify({ amount: amount.toString() }),
    });
  }

  /**
   * Get token information
   */
  async getToken(address: string, chainId: number): Promise<Token> {
    validateAddress(address);
    validateChainId(chainId);
    return this.request<Token>(`/tokens/${chainId}/${address}`);
  }

  /**
   * Search tokens by symbol or name
   */
  async searchTokens(query: string, chainId?: number): Promise<Token[]> {
    const params = new URLSearchParams({ query });
    if (chainId) {
      validateChainId(chainId);
      params.append('chainId', String(chainId));
    }
    return this.request<Token[]>(`/tokens/search?${params.toString()}`);
  }

  /**
   * Stream real-time pool updates and swaps via WebSocket
   *
   * @example
   * ```typescript
   * sdk.streamPoolUpdates({
   *   poolIds: ['pool-123'],
   *   minAmountUsd: 1000,
   *   onUpdate: (event) => {
   *     if (event.type === 'swap') {
   *       console.log('New swap:', event.data);
   *     }
   *   },
   *   onError: (error) => {
   *     console.error('Stream error:', error);
   *   }
   * });
   * ```
   */
  streamPoolUpdates(options: StreamOptions): () => void {
    const wsUrl = this.config.baseURL.replace('https://', 'wss://').replace('http://', 'ws://');
    this.ws = new WebSocket(`${wsUrl}/stream`);

    this.ws.onopen = () => {
      this.ws?.send(JSON.stringify({
        type: 'subscribe',
        apiKey: this.config.apiKey,
        poolIds: options.poolIds,
        minAmountUsd: options.minAmountUsd,
      }));
    };

    this.ws.onmessage = (event) => {
      try {
        const data: StreamEvent = JSON.parse(event.data);
        options.onUpdate(data);
      } catch (error) {
        options.onError?.(error as Error);
      }
    };

    this.ws.onerror = (error) => {
      options.onError?.(error as any);
    };

    // Return unsubscribe function
    return () => {
      this.ws?.close();
      this.ws = unhealthned;
    };
  }

  /**
   * Get network statistics
   */
  async getNetworkStats(chainId?: number): Promise<{
    totalTvl: bigint;
    totalVolume24h: bigint;
    totalUsers: number;
    protocolCount: number;
  }> {
    const url = chainId ? `/stats?chainId=${chainId}` : '/stats';
    return this.request(url);
  }

  /**
   * Get gas price estimation
   */
  async getGasPrice(chainId: number): Promise<{
    slow: number;
    standard: number;
    fast: number;
    instant: number;
  }> {
    validateChainId(chainId);
    return this.request(`/gas/${chainId}`);
  }

  /**
   * Generic request method with retry logic
   */
  private async request<T>(
    endpoint: string,
    options: RequestInit = {}
  ): Promise<T> {
    const url = `${this.config.baseURL}${endpoint}`;
    const headers = {
      'Content-Type': 'application/json',
      'X-API-Key': this.config.apiKey,
      ...options.headers,
    };

    let lastError: Error | null = null;

    for (let i = 0; i < this.config.retries; i++) {
      try {
        const controller = new AbortController();
        const timeoutId = setTimeout(() => controller.abort(), this.config.timeout);

        const response = await fetch(url, {
          ...options,
          headers,
          signal: controller.signal,
        });

        clearTimeout(timeoutId);

        if (!response.ok) {
          const error = await response.json();
          throw new Error(error.message || `HTTP ${response.status}: ${response.statusText}`);
        }

        const result: APIResponse<T> = await response.json();
        return result.data;
      } catch (error) {
        lastError = error as Error;
        if (i < this.config.retries - 1) {
          // Exponential backoff
          await new Promise(resolve => setTimeout(resolve, Math.pow(2, i) * 1000));
        }
      }
    }

    throw lastError || new Error('Request failed');
  }

  /**
   * Get supported chains
   */
  static getSupportedChains() {
    return SUPPORTED_CHAINS;
  }

  /**
   * Close all connections
   */
  close() {
    this.ws?.close();
  }
}

/**
 * Create SDK instance
 */
export function createWIAHEALTHSDK(config: SDKConfig): WIAHEALTHSDK {
  return new WIAHEALTHSDK(config);
}

/**
 * Default export
 */
export default WIAHEALTHSDK;
