/**
 * WIA Wealth Management Standard - TypeScript SDK
 *
 * @module @wia/wealth-management
 * @version 1.0.0
 * @license MIT
 *
 * 弘益人間 · Benefit All Humanity
 */

import axios, { AxiosInstance, AxiosError } from 'axios';
import {
  SDKConfig,
  APIResponse,
  PaginationParams,
  PaginatedResponse,
  User,
  Portfolio,
  Asset,
  Transaction,
  Analytics,
  TaxReport,
  TaxLossOpportunity,
  FinancialGoal,
  RetirementPlan,
  Quote,
  WIAError,
  ErrorCode,
  AssetType,
  TransactionType
} from './types';

/**
 * WIA Wealth Management SDK
 *
 * Main SDK class for interacting with the WIA Wealth Management API
 *
 * @example
 * ```typescript
 * const sdk = new WealthManagementSDK({
 *   apiKey: 'your-api-key',
 *   environment: 'production'
 * });
 *
 * const portfolio = await sdk.getPortfolio('port_123');
 * console.log(portfolio.totalValue);
 * ```
 */
export class WealthManagementSDK {
  private client: AxiosInstance;
  private config: SDKConfig;

  /**
   * Create a new SDK instance
   *
   * @param config - SDK configuration
   */
  constructor(config: SDKConfig) {
    this.config = config;

    const baseURLs = {
      development: 'https://dev-api.wia.org/v1',
      staging: 'https://staging-api.wia.org/v1',
      production: 'https://api.wia.org/v1'
    };

    this.client = axios.create({
      baseURL: config.baseUrl || baseURLs[config.environment || 'production'],
      timeout: config.timeout || 30000,
      headers: {
        'Authorization': `Bearer ${config.apiKey}`,
        'Content-Type': 'application/json',
        'X-WIA-SDK-Version': '1.0.0'
      }
    });

    // Response interceptor for error handling
    this.client.interceptors.response.use(
      (response) => response,
      (error: AxiosError) => {
        throw this.handleError(error);
      }
    );
  }

  // ============================================================================
  // User Management
  // ============================================================================

  /**
   * Get current user information
   *
   * @returns User information
   */
  async getCurrentUser(): Promise<User> {
    const response = await this.client.get<APIResponse<User>>('/user');
    return response.data.data;
  }

  /**
   * Update user information
   *
   * @param updates - User fields to update
   * @returns Updated user information
   */
  async updateUser(updates: Partial<User>): Promise<User> {
    const response = await this.client.put<APIResponse<User>>('/user', updates);
    return response.data.data;
  }

  // ============================================================================
  // Portfolio Management
  // ============================================================================

  /**
   * Get all portfolios for the current user
   *
   * @param params - Pagination parameters
   * @returns List of portfolios
   */
  async getPortfolios(params?: PaginationParams): Promise<PaginatedResponse<Portfolio>> {
    const response = await this.client.get<APIResponse<PaginatedResponse<Portfolio>>>(
      '/portfolios',
      { params }
    );
    return response.data.data;
  }

  /**
   * Get a specific portfolio by ID
   *
   * @param portfolioId - Portfolio ID
   * @returns Portfolio details
   */
  async getPortfolio(portfolioId: string): Promise<Portfolio> {
    const response = await this.client.get<APIResponse<Portfolio>>(
      `/portfolios/${portfolioId}`
    );
    return response.data.data;
  }

  /**
   * Create a new portfolio
   *
   * @param portfolio - Portfolio data
   * @returns Created portfolio
   */
  async createPortfolio(portfolio: {
    name: string;
    description?: string;
    currency?: string;
  }): Promise<Portfolio> {
    const response = await this.client.post<APIResponse<Portfolio>>(
      '/portfolios',
      portfolio
    );
    return response.data.data;
  }

  /**
   * Update a portfolio
   *
   * @param portfolioId - Portfolio ID
   * @param updates - Portfolio fields to update
   * @returns Updated portfolio
   */
  async updatePortfolio(
    portfolioId: string,
    updates: Partial<Portfolio>
  ): Promise<Portfolio> {
    const response = await this.client.put<APIResponse<Portfolio>>(
      `/portfolios/${portfolioId}`,
      updates
    );
    return response.data.data;
  }

  /**
   * Delete a portfolio
   *
   * @param portfolioId - Portfolio ID
   * @returns Success status
   */
  async deletePortfolio(portfolioId: string): Promise<boolean> {
    const response = await this.client.delete<APIResponse<{ success: boolean }>>(
      `/portfolios/${portfolioId}`
    );
    return response.data.data.success;
  }

  // ============================================================================
  // Asset Management
  // ============================================================================

  /**
   * Get all assets in a portfolio
   *
   * @param portfolioId - Portfolio ID
   * @returns List of assets
   */
  async getAssets(portfolioId: string): Promise<Asset[]> {
    const response = await this.client.get<APIResponse<Asset[]>>(
      `/portfolios/${portfolioId}/assets`
    );
    return response.data.data;
  }

  /**
   * Get a specific asset
   *
   * @param assetId - Asset ID
   * @returns Asset details
   */
  async getAsset(assetId: string): Promise<Asset> {
    const response = await this.client.get<APIResponse<Asset>>(`/assets/${assetId}`);
    return response.data.data;
  }

  /**
   * Add a new asset to a portfolio
   *
   * @param asset - Asset data
   * @returns Created asset
   */
  async addAsset(asset: {
    portfolioId: string;
    type: AssetType;
    symbol?: string;
    name: string;
    quantity: number;
    costBasis: number;
  }): Promise<Asset> {
    const response = await this.client.post<APIResponse<Asset>>('/assets', asset);
    return response.data.data;
  }

  /**
   * Update an asset
   *
   * @param assetId - Asset ID
   * @param updates - Asset fields to update
   * @returns Updated asset
   */
  async updateAsset(assetId: string, updates: Partial<Asset>): Promise<Asset> {
    const response = await this.client.put<APIResponse<Asset>>(
      `/assets/${assetId}`,
      updates
    );
    return response.data.data;
  }

  /**
   * Remove an asset from a portfolio
   *
   * @param assetId - Asset ID
   * @returns Success status
   */
  async removeAsset(assetId: string): Promise<boolean> {
    const response = await this.client.delete<APIResponse<{ success: boolean }>>(
      `/assets/${assetId}`
    );
    return response.data.data.success;
  }

  /**
   * Get current market valuation for an asset
   *
   * @param assetId - Asset ID
   * @returns Updated asset with current market value
   */
  async getAssetValuation(assetId: string): Promise<Asset> {
    const response = await this.client.get<APIResponse<Asset>>(
      `/assets/${assetId}/valuation`
    );
    return response.data.data;
  }

  // ============================================================================
  // Transaction Management
  // ============================================================================

  /**
   * Get transaction history
   *
   * @param portfolioId - Portfolio ID
   * @param params - Pagination and filter parameters
   * @returns List of transactions
   */
  async getTransactions(
    portfolioId: string,
    params?: PaginationParams & { type?: TransactionType }
  ): Promise<PaginatedResponse<Transaction>> {
    const response = await this.client.get<APIResponse<PaginatedResponse<Transaction>>>(
      `/portfolios/${portfolioId}/transactions`,
      { params }
    );
    return response.data.data;
  }

  /**
   * Record a new transaction
   *
   * @param transaction - Transaction data
   * @returns Created transaction
   */
  async recordTransaction(transaction: Omit<Transaction, 'id'>): Promise<Transaction> {
    const response = await this.client.post<APIResponse<Transaction>>(
      '/transactions',
      transaction
    );
    return response.data.data;
  }

  // ============================================================================
  // Analytics & Performance
  // ============================================================================

  /**
   * Get portfolio analytics
   *
   * @param portfolioId - Portfolio ID
   * @param period - Analysis period (ytd, 1m, 3m, 1y, etc.)
   * @returns Portfolio analytics
   */
  async getAnalytics(params: {
    portfolioId: string;
    period: string;
  }): Promise<Analytics> {
    const response = await this.client.get<APIResponse<Analytics>>(
      `/analytics/portfolio/${params.portfolioId}`,
      { params: { period: params.period } }
    );
    return response.data.data;
  }

  /**
   * Optimize portfolio allocation
   *
   * @param portfolioId - Portfolio ID
   * @param constraints - Optimization constraints
   * @returns Recommended rebalancing trades
   */
  async optimizePortfolio(
    portfolioId: string,
    constraints?: {
      targetReturn?: number;
      maxRisk?: number;
      constraints?: Record<string, any>;
    }
  ): Promise<{
    currentAllocation: Record<string, number>;
    optimizedAllocation: Record<string, number>;
    recommendedTrades: Array<{
      assetId: string;
      action: 'buy' | 'sell';
      quantity: number;
      amount: number;
    }>;
  }> {
    const response = await this.client.post<APIResponse<any>>(
      `/analytics/optimize/${portfolioId}`,
      constraints
    );
    return response.data.data;
  }

  // ============================================================================
  // Tax Services
  // ============================================================================

  /**
   * Get tax report for a specific year
   *
   * @param year - Tax year
   * @returns Tax report
   */
  async getTaxReport(year: number): Promise<TaxReport> {
    const response = await this.client.get<APIResponse<TaxReport>>(
      `/tax/reports/${year}`
    );
    return response.data.data;
  }

  /**
   * Get tax-loss harvesting opportunities
   *
   * @param portfolioId - Portfolio ID (optional)
   * @returns List of tax-loss harvesting opportunities
   */
  async getTaxHarvestOpportunities(
    portfolioId?: string
  ): Promise<TaxLossOpportunity[]> {
    const response = await this.client.get<APIResponse<TaxLossOpportunity[]>>(
      '/tax/harvest-opportunities',
      { params: { portfolioId } }
    );
    return response.data.data;
  }

  /**
   * Execute tax-loss harvesting
   *
   * @param opportunityId - Opportunity ID
   * @returns Transaction record
   */
  async executeTaxHarvest(opportunityId: string): Promise<Transaction> {
    const response = await this.client.post<APIResponse<Transaction>>(
      `/tax/execute-harvest/${opportunityId}`
    );
    return response.data.data;
  }

  // ============================================================================
  // Financial Planning
  // ============================================================================

  /**
   * Get all financial goals
   *
   * @returns List of financial goals
   */
  async getFinancialGoals(): Promise<FinancialGoal[]> {
    const response = await this.client.get<APIResponse<FinancialGoal[]>>('/goals');
    return response.data.data;
  }

  /**
   * Create a new financial goal
   *
   * @param goal - Goal data
   * @returns Created goal
   */
  async createFinancialGoal(goal: {
    name: string;
    description?: string;
    targetAmount: number;
    targetDate: Date;
  }): Promise<FinancialGoal> {
    const response = await this.client.post<APIResponse<FinancialGoal>>('/goals', goal);
    return response.data.data;
  }

  /**
   * Get retirement plan
   *
   * @returns Retirement plan details
   */
  async getRetirementPlan(): Promise<RetirementPlan> {
    const response = await this.client.get<APIResponse<RetirementPlan>>(
      '/planning/retirement'
    );
    return response.data.data;
  }

  /**
   * Update retirement plan parameters
   *
   * @param updates - Plan parameters to update
   * @returns Updated retirement plan
   */
  async updateRetirementPlan(updates: Partial<RetirementPlan>): Promise<RetirementPlan> {
    const response = await this.client.put<APIResponse<RetirementPlan>>(
      '/planning/retirement',
      updates
    );
    return response.data.data;
  }

  // ============================================================================
  // Market Data
  // ============================================================================

  /**
   * Get real-time quote for a symbol
   *
   * @param symbol - Asset symbol
   * @returns Market quote
   */
  async getQuote(symbol: string): Promise<Quote> {
    const response = await this.client.get<APIResponse<Quote>>(
      `/market/quote/${symbol}`
    );
    return response.data.data;
  }

  /**
   * Get quotes for multiple symbols
   *
   * @param symbols - Array of asset symbols
   * @returns Array of market quotes
   */
  async getQuotes(symbols: string[]): Promise<Quote[]> {
    const response = await this.client.post<APIResponse<Quote[]>>('/market/quotes', {
      symbols
    });
    return response.data.data;
  }

  // ============================================================================
  // Error Handling
  // ============================================================================

  /**
   * Handle API errors and convert to WIAError
   *
   * @param error - Axios error
   * @returns WIAError instance
   */
  private handleError(error: AxiosError): WIAError {
    if (error.response) {
      const status = error.response.status;
      const data: any = error.response.data;

      let code: ErrorCode;
      switch (status) {
        case 401:
          code = ErrorCode.UNAUTHORIZED;
          break;
        case 403:
          code = ErrorCode.FORBIDDEN;
          break;
        case 404:
          code = ErrorCode.NOT_FOUND;
          break;
        case 400:
          code = ErrorCode.VALIDATION_ERROR;
          break;
        case 429:
          code = ErrorCode.RATE_LIMIT_EXCEEDED;
          break;
        case 503:
          code = ErrorCode.SERVICE_UNAVAILABLE;
          break;
        default:
          code = ErrorCode.INTERNAL_ERROR;
      }

      return new WIAError(
        code,
        data.error || error.message,
        status,
        data.details
      );
    } else if (error.request) {
      return new WIAError(
        ErrorCode.SERVICE_UNAVAILABLE,
        'No response received from server',
        undefined,
        { request: error.request }
      );
    } else {
      return new WIAError(
        ErrorCode.INTERNAL_ERROR,
        error.message,
        undefined,
        { originalError: error }
      );
    }
  }
}

// Export all types
export * from './types';

// Default export
export default WealthManagementSDK;

/**
 * Create SDK instance helper function
 *
 * @param config - SDK configuration
 * @returns WealthManagementSDK instance
 */
export function createWealthManagementSDK(config: SDKConfig): WealthManagementSDK {
  return new WealthManagementSDK(config);
}
