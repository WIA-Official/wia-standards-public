/**
 * WIA-FIN-006 DeFi Standard SDK
 * @version 1.0.0
 * @standard WIA-FIN-006
 */

import axios, { AxiosInstance } from 'axios';
import { ethers } from 'ethers';
import * as Types from './types';

export * from './types';

const DEFAULT_BASE_URL = 'https://api.wiastandards.com/defi/v1';
const DEFAULT_TIMEOUT = 30000;

/**
 * Main SDK class for WIA DeFi Standard
 */
export class WIADeFi {
  private api: AxiosInstance;
  private config: Types.WIADeFiConfig;
  private provider?: ethers.Provider;

  /**
   * Initialize WIA DeFi SDK
   * @param config SDK configuration
   */
  constructor(config: Types.WIADeFiConfig) {
    this.config = config;

    this.api = axios.create({
      baseURL: config.baseURL || DEFAULT_BASE_URL,
      timeout: config.timeout || DEFAULT_TIMEOUT,
      headers: {
        'Content-Type': 'application/json',
        'X-API-Key': config.apiKey,
        'X-Chain-Id': config.chainId?.toString() || '1'
      }
    });

    if (config.provider) {
      this.provider = config.provider;
    }
  }

  // ========================================
  // Pool Methods
  // ========================================

  /**
   * Get list of liquidity pools
   * @param params Query parameters
   * @returns List of pools
   */
  async getPools(params?: {
    chainId?: number;
    protocol?: string;
    type?: Types.PoolType;
    minTVL?: number;
    limit?: number;
    cursor?: string;
  }): Promise<Types.APIResponse<Types.PaginatedResponse<Types.LiquidityPool>>> {
    const response = await this.api.get('/pools', { params });
    return response.data;
  }

  /**
   * Get specific pool details
   * @param poolId Pool identifier
   * @returns Pool details
   */
  async getPool(poolId: string): Promise<Types.APIResponse<{ pool: Types.LiquidityPool }>> {
    const response = await this.api.get(`/pools/${poolId}`);
    return response.data;
  }

  /**
   * Get pool historical data
   * @param poolId Pool identifier
   * @param params Query parameters
   * @returns Historical data
   */
  async getPoolHistory(
    poolId: string,
    params: {
      interval: '1h' | '4h' | '1d' | '1w';
      from: string;
      to: string;
    }
  ): Promise<Types.APIResponse<{ history: any[] }>> {
    const response = await this.api.get(`/pools/${poolId}/history`, { params });
    return response.data;
  }

  // ========================================
  // Swap Methods
  // ========================================

  /**
   * Get swap quote
   * @param params Swap parameters
   * @returns Swap quote
   */
  async getSwapQuote(params: {
    chainId: number;
    tokenIn: string;
    tokenOut: string;
    amountIn: string;
    slippageTolerance?: number;
    protocols?: string[];
  }): Promise<Types.APIResponse<{ quote: Types.SwapQuote }>> {
    const response = await this.api.post('/swap/quote', params);
    return response.data;
  }

  /**
   * Execute swap transaction
   * @param params Swap execution parameters
   * @returns Transaction data
   */
  async executeSwap(params: {
    quote: Types.SwapQuote;
    from: string;
    deadline?: number;
  }): Promise<Types.APIResponse<{ transaction: Types.SwapTransaction }>> {
    const response = await this.api.post('/swap/execute', params);
    return response.data;
  }

  /**
   * Get swap history for address
   * @param address Wallet address
   * @param params Query parameters
   * @returns Swap history
   */
  async getSwapHistory(
    address: string,
    params?: {
      chainId?: number;
      limit?: number;
    }
  ): Promise<Types.APIResponse<{ swaps: any[] }>> {
    const response = await this.api.get('/swap/history', {
      params: { ...params, address }
    });
    return response.data;
  }

  // ========================================
  // Lending Methods
  // ========================================

  /**
   * Get lending markets
   * @param params Query parameters
   * @returns Lending markets
   */
  async getLendingMarkets(params?: {
    chainId?: number;
    protocol?: string;
  }): Promise<Types.APIResponse<{ markets: Types.LendingMarket[] }>> {
    const response = await this.api.get('/lending/markets', { params });
    return response.data;
  }

  /**
   * Get user lending position
   * @param address User address
   * @param params Query parameters
   * @returns User position
   */
  async getLendingPosition(
    address: string,
    params?: {
      chainId?: number;
      protocol?: string;
    }
  ): Promise<Types.APIResponse<{ position: Types.UserLendingPosition }>> {
    const response = await this.api.get(`/lending/positions/${address}`, { params });
    return response.data;
  }

  /**
   * Supply asset to lending protocol
   * @param params Supply parameters
   * @returns Transaction data
   */
  async supplyAsset(params: {
    chainId: number;
    protocol: string;
    asset: string;
    amount: string;
    from: string;
  }): Promise<Types.APIResponse<{ transaction: Types.SwapTransaction }>> {
    const response = await this.api.post('/lending/supply', params);
    return response.data;
  }

  /**
   * Borrow asset from lending protocol
   * @param params Borrow parameters
   * @returns Transaction data
   */
  async borrowAsset(params: {
    chainId: number;
    protocol: string;
    asset: string;
    amount: string;
    rateMode: 'stable' | 'variable';
    from: string;
  }): Promise<Types.APIResponse<{ transaction: Types.SwapTransaction }>> {
    const response = await this.api.post('/lending/borrow', params);
    return response.data;
  }

  /**
   * Repay borrowed asset
   * @param params Repay parameters
   * @returns Transaction data
   */
  async repayAsset(params: {
    chainId: number;
    protocol: string;
    asset: string;
    amount: string;
    rateMode: 'stable' | 'variable';
    from: string;
  }): Promise<Types.APIResponse<{ transaction: Types.SwapTransaction }>> {
    const response = await this.api.post('/lending/repay', params);
    return response.data;
  }

  // ========================================
  // Yield Farming Methods
  // ========================================

  /**
   * Get yield farms
   * @param params Query parameters
   * @returns Yield farms
   */
  async getFarms(params?: {
    chainId?: number;
    protocol?: string;
    minAPY?: number;
    type?: Types.FarmType;
  }): Promise<Types.APIResponse<{ farms: Types.YieldFarm[] }>> {
    const response = await this.api.get('/farming/farms', { params });
    return response.data;
  }

  /**
   * Stake tokens in farm
   * @param params Stake parameters
   * @returns Transaction data
   */
  async stakeFarm(params: {
    chainId: number;
    farmId: string;
    amount: string;
    from: string;
  }): Promise<Types.APIResponse<{ transaction: Types.SwapTransaction }>> {
    const response = await this.api.post('/farming/stake', params);
    return response.data;
  }

  /**
   * Claim farm rewards
   * @param params Claim parameters
   * @returns Transaction data
   */
  async claimRewards(params: {
    chainId: number;
    farmId: string;
    from: string;
  }): Promise<Types.APIResponse<{ transaction: Types.SwapTransaction }>> {
    const response = await this.api.post('/farming/claim', params);
    return response.data;
  }

  /**
   * Unstake tokens from farm
   * @param params Unstake parameters
   * @returns Transaction data
   */
  async unstakeFarm(params: {
    chainId: number;
    farmId: string;
    amount: string;
    from: string;
  }): Promise<Types.APIResponse<{ transaction: Types.SwapTransaction }>> {
    const response = await this.api.post('/farming/unstake', params);
    return response.data;
  }

  // ========================================
  // Flash Loan Methods
  // ========================================

  /**
   * Get flash loan providers
   * @returns Flash loan providers
   */
  async getFlashLoanProviders(): Promise<Types.APIResponse<{ providers: any[] }>> {
    const response = await this.api.get('/flashloan/providers');
    return response.data;
  }

  /**
   * Simulate flash loan
   * @param params Flash loan parameters
   * @returns Simulation result
   */
  async simulateFlashLoan(params: {
    chainId: number;
    protocol: string;
    loans: Array<{ asset: string; amount: string }>;
    operations: any[];
  }): Promise<Types.APIResponse<{ simulation: any }>> {
    const response = await this.api.post('/flashloan/simulate', params);
    return response.data;
  }

  // ========================================
  // Governance Methods
  // ========================================

  /**
   * Get governance proposals
   * @param params Query parameters
   * @returns Proposals
   */
  async getProposals(params?: {
    protocol?: string;
    status?: Types.ProposalStatus;
  }): Promise<Types.APIResponse<{ proposals: Types.GovernanceProposal[] }>> {
    const response = await this.api.get('/governance/proposals', { params });
    return response.data;
  }

  /**
   * Get proposal details
   * @param proposalId Proposal ID
   * @returns Proposal details
   */
  async getProposal(
    proposalId: string
  ): Promise<Types.APIResponse<{ proposal: Types.GovernanceProposal }>> {
    const response = await this.api.get(`/governance/proposals/${proposalId}`);
    return response.data;
  }

  /**
   * Cast vote on proposal
   * @param params Vote parameters
   * @returns Transaction data
   */
  async castVote(params: {
    proposalId: string;
    support: Types.VoteSupport;
    from: string;
    reason?: string;
  }): Promise<Types.APIResponse<{ transaction: Types.SwapTransaction }>> {
    const response = await this.api.post('/governance/vote', params);
    return response.data;
  }

  // ========================================
  // Portfolio Methods
  // ========================================

  /**
   * Get user portfolio
   * @param address User address
   * @param params Query parameters
   * @returns Portfolio positions
   */
  async getPortfolio(
    address: string,
    params?: {
      chains?: number[];
      protocols?: string[];
      includeRewards?: boolean;
    }
  ): Promise<Types.APIResponse<{ positions: any[] }>> {
    const response = await this.api.get(`/portfolio/${address}`, { params });
    return response.data;
  }

  /**
   * Get user net worth
   * @param address User address
   * @returns Net worth data
   */
  async getNetWorth(
    address: string
  ): Promise<Types.APIResponse<{ netWorth: any }>> {
    const response = await this.api.get(`/portfolio/${address}/networth`);
    return response.data;
  }

  // ========================================
  // Utility Methods
  // ========================================

  /**
   * Get supported networks
   * @returns Network list
   */
  async getNetworks(): Promise<Types.APIResponse<{ networks: Types.ChainInfo[] }>> {
    const response = await this.api.get('/networks');
    return response.data;
  }

  /**
   * Set chain ID for subsequent requests
   * @param chainId Chain ID
   */
  setChainId(chainId: number): void {
    this.config.chainId = chainId;
    this.api.defaults.headers['X-Chain-Id'] = chainId.toString();
  }

  /**
   * Get current configuration
   * @returns Current config
   */
  getConfig(): Types.WIADeFiConfig {
    return { ...this.config };
  }
}

/**
 * Create WIA DeFi SDK instance
 * @param config SDK configuration
 * @returns SDK instance
 */
export function createWIADeFi(config: Types.WIADeFiConfig): WIADeFi {
  return new WIADeFi(config);
}

export default WIADeFi;
