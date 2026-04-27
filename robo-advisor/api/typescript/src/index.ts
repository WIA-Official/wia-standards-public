/**
 * WIA-FIN-016 Robo-Advisor Standard - TypeScript SDK
 * Version: 1.0.0
 *
 * © 2025 WIA (World Certification Industry Association)
 * 弘益人間 · Benefit All Humanity
 */

import axios, { AxiosInstance, AxiosError } from 'axios';
import * as Types from './types';

export * from './types';

export class RoboAdvisorClient {
  private client: AxiosInstance;
  private config: Types.RoboAdvisorConfig;

  constructor(config: Types.RoboAdvisorConfig) {
    this.config = {
      baseURL: config.baseURL || 'https://api.wiastandards.com',
      timeout: config.timeout || 30000,
      retries: config.retries || 3,
      ...config
    };

    this.client = axios.create({
      baseURL: this.config.baseURL,
      timeout: this.config.timeout,
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${this.config.apiKey}`,
        'X-WIA-Standard': 'FIN-016',
        'X-WIA-Version': '1.0.0'
      }
    });

    // Add response interceptor for error handling
    this.client.interceptors.response.use(
      response => response,
      error => this.handleError(error)
    );
  }

  private handleError(error: AxiosError): never {
    if (error.response) {
      throw new Types.RoboAdvisorError(
        error.response.data?.message || 'API request failed',
        error.response.data?.code || 'UNKNOWN_ERROR',
        error.response.status,
        error.response.data
      );
    } else if (error.request) {
      throw new Types.RoboAdvisorError(
        'No response received from server',
        'NETWORK_ERROR'
      );
    } else {
      throw new Types.RoboAdvisorError(
        error.message,
        'REQUEST_ERROR'
      );
    }
  }

  // Portfolio Management Methods

  /**
   * Create a new portfolio
   */
  async createPortfolio(
    request: Types.CreatePortfolioRequest
  ): Promise<Types.CreatePortfolioResponse> {
    const response = await this.client.post('/api/v1/portfolios', request);
    return response.data;
  }

  /**
   * Get portfolio by ID
   */
  async getPortfolio(portfolioId: Types.PortfolioId): Promise<Types.Portfolio> {
    const response = await this.client.get(`/api/v1/portfolios/${portfolioId}`);
    return response.data;
  }

  /**
   * List all portfolios for a user
   */
  async listPortfolios(userId: Types.UserId): Promise<Types.Portfolio[]> {
    const response = await this.client.get('/api/v1/portfolios', {
      params: { userId }
    });
    return response.data;
  }

  /**
   * Update portfolio allocation
   */
  async updateAllocation(
    portfolioId: Types.PortfolioId,
    allocation: Types.UpdateAllocationRequest
  ): Promise<void> {
    await this.client.put(
      `/api/v1/portfolios/${portfolioId}/allocation`,
      allocation
    );
  }

  /**
   * Get portfolio holdings
   */
  async getHoldings(portfolioId: Types.PortfolioId): Promise<Types.Holding[]> {
    const response = await this.client.get(
      `/api/v1/portfolios/${portfolioId}/holdings`
    );
    return response.data;
  }

  /**
   * Get portfolio performance
   */
  async getPerformance(portfolioId: Types.PortfolioId): Promise<Types.Performance> {
    const response = await this.client.get(
      `/api/v1/portfolios/${portfolioId}/performance`
    );
    return response.data;
  }

  /**
   * Close portfolio
   */
  async closePortfolio(portfolioId: Types.PortfolioId): Promise<void> {
    await this.client.delete(`/api/v1/portfolios/${portfolioId}`);
  }

  // Risk Assessment Methods

  /**
   * Submit risk assessment questionnaire
   */
  async submitRiskAssessment(
    request: Types.SubmitRiskAssessmentRequest
  ): Promise<Types.RiskAssessmentResponse> {
    const response = await this.client.post('/api/v1/risk-assessment', request);
    return response.data;
  }

  /**
   * Get risk assessment for user
   */
  async getRiskAssessment(userId: Types.UserId): Promise<Types.RiskAssessment> {
    const response = await this.client.get(`/api/v1/risk-assessment/${userId}`);
    return response.data;
  }

  // Rebalancing Methods

  /**
   * Preview rebalancing for a portfolio
   */
  async previewRebalance(
    portfolioId: Types.PortfolioId
  ): Promise<Types.RebalancePreview> {
    const response = await this.client.get(
      `/api/v1/portfolios/${portfolioId}/rebalance/preview`
    );
    return response.data;
  }

  /**
   * Execute rebalancing
   */
  async executeRebalance(
    portfolioId: Types.PortfolioId
  ): Promise<Types.RebalanceExecuteResponse> {
    const response = await this.client.post(
      `/api/v1/portfolios/${portfolioId}/rebalance`
    );
    return response.data;
  }

  /**
   * Get rebalance event by ID
   */
  async getRebalanceEvent(
    rebalanceId: string
  ): Promise<Types.RebalanceEvent> {
    const response = await this.client.get(`/api/v1/rebalance/${rebalanceId}`);
    return response.data;
  }

  /**
   * List rebalance events for a portfolio
   */
  async listRebalanceEvents(
    portfolioId: Types.PortfolioId,
    limit: number = 10
  ): Promise<Types.RebalanceEvent[]> {
    const response = await this.client.get(
      `/api/v1/portfolios/${portfolioId}/rebalance/history`,
      { params: { limit } }
    );
    return response.data;
  }

  // Market Data Methods

  /**
   * Get current price for a security
   */
  async getSecurityPrice(symbol: string): Promise<number> {
    const response = await this.client.get(`/api/v1/market-data/${symbol}`);
    return response.data.price;
  }

  /**
   * Get historical prices for a security
   */
  async getHistoricalPrices(
    symbol: string,
    startDate: string,
    endDate: string
  ): Promise<Array<{ date: string; price: number }>> {
    const response = await this.client.get(
      `/api/v1/market-data/${symbol}/history`,
      { params: { startDate, endDate } }
    );
    return response.data;
  }

  // Utility Methods

  /**
   * Calculate recommended allocation based on risk score
   */
  static calculateAllocation(riskScore: number): Types.Allocation {
    const stockAllocation = Math.min(90, Math.max(10, riskScore));
    const bondAllocation = 100 - stockAllocation;

    return {
      stocks: stockAllocation,
      bonds: bondAllocation,
      cash: 0,
      alternatives: 0
    };
  }

  /**
   * Calculate Sharpe ratio
   */
  static calculateSharpe(
    portfolioReturn: number,
    riskFreeRate: number,
    stdDev: number
  ): number {
    return (portfolioReturn - riskFreeRate) / stdDev;
  }

  /**
   * Check if rebalancing is needed
   */
  static needsRebalancing(
    currentAllocation: Types.Allocation,
    targetAllocation: Types.Allocation,
    threshold: number = 5
  ): boolean {
    const stockDrift = Math.abs(currentAllocation.stocks - targetAllocation.stocks);
    const bondDrift = Math.abs(currentAllocation.bonds - targetAllocation.bonds);

    return stockDrift >= threshold || bondDrift >= threshold;
  }
}

// Export default instance creator
export function createClient(config: Types.RoboAdvisorConfig): RoboAdvisorClient {
  return new RoboAdvisorClient(config);
}

// Example usage
export const example = async () => {
  const client = createClient({
    apiKey: 'your_api_key_here',
    baseURL: 'https://api.wiastandards.com'
  });

  // Create portfolio
  const portfolio = await client.createPortfolio({
    userId: 'user_123',
    accountType: Types.AccountType.TAXABLE,
    initialDeposit: 10000,
    riskScore: 70
  });

  console.log('Portfolio created:', portfolio);

  // Get portfolio details
  const details = await client.getPortfolio(portfolio.portfolioId);
  console.log('Portfolio details:', details);

  // Preview rebalancing
  const preview = await client.previewRebalance(portfolio.portfolioId);
  console.log('Rebalance preview:', preview);

  if (preview.rebalanceNeeded) {
    // Execute rebalancing
    const result = await client.executeRebalance(portfolio.portfolioId);
    console.log('Rebalance executed:', result);
  }
};

export default RoboAdvisorClient;
