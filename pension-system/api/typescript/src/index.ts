/**
 * WIA-SOC-018 Pension System Standard - TypeScript SDK
 *
 * @packageDocumentation
 * @module wia-soc-018
 * @version 1.0.0
 * @license MIT
 *
 * 弘益人間 (Hongik Ingan) - Benefit All Humanity
 */

import {
  PensionMember,
  PensionContribution,
  PensionBenefitCalculation,
  PensionFundAllocation,
  ApiResponse,
  PaginatedResponse,
  CrossBorderTransferRequest,
  TransferStatus,
  MemberStatus,
  SchemeType,
  AssetClass
} from './types';

export * from './types';

/**
 * Configuration options for WIA-SOC-018 Client
 */
export interface ClientConfig {
  baseUrl: string;
  apiKey: string;
  timeout?: number;
  retryAttempts?: number;
}

/**
 * WIA-SOC-018 Pension System API Client
 *
 * @example
 * ```typescript
 * const client = new WIAPensionClient({
 *   baseUrl: 'https://api.pension-provider.com',
 *   apiKey: 'your-api-key'
 * });
 *
 * const member = await client.getMember('member-uuid');
 * ```
 */
export class WIAPensionClient {
  private config: ClientConfig;
  private readonly DEFAULT_TIMEOUT = 30000;
  private readonly DEFAULT_RETRY_ATTEMPTS = 3;

  constructor(config: ClientConfig) {
    this.config = {
      ...config,
      timeout: config.timeout || this.DEFAULT_TIMEOUT,
      retryAttempts: config.retryAttempts || this.DEFAULT_RETRY_ATTEMPTS
    };
  }

  /**
   * Make HTTP request to API
   */
  private async request<T>(
    method: string,
    endpoint: string,
    data?: any
  ): Promise<ApiResponse<T>> {
    const url = `${this.config.baseUrl}/wia/soc-018/v1${endpoint}`;
    const headers = {
      'Authorization': `Bearer ${this.config.apiKey}`,
      'Content-Type': 'application/json',
      'X-API-Version': '1.0.0'
    };

    // Implement actual HTTP request logic here
    // This is a placeholder implementation
    throw new Error('HTTP client not implemented - use fetch or axios');
  }

  // ==================== Member Management ====================

  /**
   * Retrieve member information
   * @param memberId - Unique member identifier
   */
  async getMember(memberId: string): Promise<ApiResponse<PensionMember>> {
    return this.request<PensionMember>('GET', `/members/${memberId}`);
  }

  /**
   * Create new member
   * @param member - Member data
   */
  async createMember(member: Partial<PensionMember>): Promise<ApiResponse<PensionMember>> {
    return this.request<PensionMember>('POST', '/members', member);
  }

  /**
   * Update member information
   * @param memberId - Unique member identifier
   * @param updates - Partial member data to update
   */
  async updateMember(
    memberId: string,
    updates: Partial<PensionMember>
  ): Promise<ApiResponse<PensionMember>> {
    return this.request<PensionMember>('PATCH', `/members/${memberId}`, updates);
  }

  // ==================== Contribution Management ====================

  /**
   * Submit contribution record
   * @param contribution - Contribution data
   */
  async createContribution(
    contribution: Partial<PensionContribution>
  ): Promise<ApiResponse<PensionContribution>> {
    return this.request<PensionContribution>('POST', '/contributions', contribution);
  }

  /**
   * Get contribution history for member
   * @param memberId - Unique member identifier
   * @param startDate - Filter start date (ISO8601)
   * @param endDate - Filter end date (ISO8601)
   * @param page - Page number
   * @param limit - Items per page
   */
  async getContributions(
    memberId: string,
    options?: {
      startDate?: string;
      endDate?: string;
      page?: number;
      limit?: number;
    }
  ): Promise<ApiResponse<PaginatedResponse<PensionContribution>>> {
    const params = new URLSearchParams();
    if (options?.startDate) params.append('startDate', options.startDate);
    if (options?.endDate) params.append('endDate', options.endDate);
    if (options?.page) params.append('page', options.page.toString());
    if (options?.limit) params.append('limit', options.limit.toString());

    const query = params.toString() ? `?${params.toString()}` : '';
    return this.request<PaginatedResponse<PensionContribution>>(
      'GET',
      `/contributions/member/${memberId}${query}`
    );
  }

  /**
   * Submit batch contributions
   * @param contributions - Array of contribution data
   */
  async createContributionBatch(
    contributions: Partial<PensionContribution>[]
  ): Promise<ApiResponse<{ contributionIds: string[] }>> {
    return this.request('POST', '/contributions/batch', { contributions });
  }

  // ==================== Benefit Calculation ====================

  /**
   * Calculate projected pension benefits
   * @param memberId - Unique member identifier
   * @param retirementAge - Planned retirement age
   * @param scenario - Calculation scenario
   */
  async calculateBenefits(
    memberId: string,
    retirementAge: number,
    scenario?: 'standard' | 'optimistic' | 'pessimistic'
  ): Promise<ApiResponse<PensionBenefitCalculation>> {
    return this.request<PensionBenefitCalculation>('POST', '/benefits/calculate', {
      memberId,
      retirementAge,
      scenario: scenario || 'standard',
      calculationDate: new Date().toISOString()
    });
  }

  /**
   * Get benefit estimates and history
   * @param memberId - Unique member identifier
   */
  async getBenefits(memberId: string): Promise<ApiResponse<PensionBenefitCalculation[]>> {
    return this.request<PensionBenefitCalculation[]>('GET', `/benefits/member/${memberId}`);
  }

  // ==================== Fund Management ====================

  /**
   * Get current fund allocation
   * @param memberId - Unique member identifier
   */
  async getFundAllocation(memberId: string): Promise<ApiResponse<PensionFundAllocation>> {
    return this.request<PensionFundAllocation>('GET', `/funds/allocation/${memberId}`);
  }

  /**
   * Update fund allocation preferences
   * @param memberId - Unique member identifier
   * @param allocation - New allocation preferences
   */
  async updateFundAllocation(
    memberId: string,
    allocation: Partial<PensionFundAllocation>
  ): Promise<ApiResponse<PensionFundAllocation>> {
    return this.request<PensionFundAllocation>(
      'PUT',
      `/funds/allocation/${memberId}`,
      allocation
    );
  }

  /**
   * Get fund performance metrics
   */
  async getFundPerformance(): Promise<ApiResponse<any>> {
    return this.request('GET', '/funds/performance');
  }

  // ==================== Cross-Border Portability ====================

  /**
   * Initiate cross-border pension transfer
   * @param transferRequest - Transfer request data
   */
  async initiateTransfer(
    transferRequest: CrossBorderTransferRequest
  ): Promise<ApiResponse<{ transferId: string }>> {
    return this.request('POST', '/portability/initiate', transferRequest);
  }

  /**
   * Check transfer status
   * @param transferId - Unique transfer identifier
   */
  async getTransferStatus(transferId: string): Promise<ApiResponse<TransferStatus>> {
    return this.request<TransferStatus>('GET', `/portability/status/${transferId}`);
  }

  /**
   * List available social security treaties
   */
  async getTreaties(): Promise<ApiResponse<any>> {
    return this.request('GET', '/portability/treaties');
  }

  // ==================== Utility Methods ====================

  /**
   * Validate contribution data
   * @param contribution - Contribution to validate
   * @returns Validation result with errors if any
   */
  validateContribution(contribution: Partial<PensionContribution>): {
    valid: boolean;
    errors: string[];
  } {
    const errors: string[] = [];

    if (!contribution.memberId) {
      errors.push('memberId is required');
    }

    if (contribution.contributions) {
      if (
        contribution.contributions.employeeAmount < 0 ||
        contribution.contributions.employerAmount < 0
      ) {
        errors.push('Contribution amounts must be non-negative');
      }

      const total =
        (contribution.contributions.employeeAmount || 0) +
        (contribution.contributions.employerAmount || 0);
      if (Math.abs(total - (contribution.contributions.totalContribution || 0)) > 0.01) {
        errors.push('Total contribution must equal employee + employer amounts');
      }
    }

    return {
      valid: errors.length === 0,
      errors
    };
  }

  /**
   * Calculate replacement rate
   * @param monthlyBenefit - Monthly pension benefit
   * @param finalSalary - Final annual salary
   * @returns Replacement rate as percentage
   */
  calculateReplacementRate(monthlyBenefit: number, finalSalary: number): number {
    const annualBenefit = monthlyBenefit * 12;
    return (annualBenefit / finalSalary) * 100;
  }

  /**
   * Project future value with inflation adjustment
   * @param currentValue - Current value
   * @param years - Number of years
   * @param inflationRate - Annual inflation rate (as decimal)
   */
  projectValueWithInflation(
    currentValue: number,
    years: number,
    inflationRate: number
  ): number {
    return currentValue * Math.pow(1 + inflationRate, years);
  }

  /**
   * Calculate early retirement reduction
   * @param fullRetirementAge - Normal retirement age
   * @param earlyRetirementAge - Actual retirement age
   * @param reductionPerYear - Reduction percentage per year (default: 5%)
   */
  calculateEarlyRetirementReduction(
    fullRetirementAge: number,
    earlyRetirementAge: number,
    reductionPerYear: number = 0.05
  ): number {
    const yearsDifference = fullRetirementAge - earlyRetirementAge;
    if (yearsDifference <= 0) return 0;
    return yearsDifference * reductionPerYear;
  }
}

/**
 * Helper functions for pension calculations
 */
export const PensionHelpers = {
  /**
   * Calculate monthly benefit from annual salary and accrual rate
   */
  calculateMonthlyBenefit(
    finalAverageSalary: number,
    serviceYears: number,
    accrualRate: number
  ): number {
    const annualBenefit = finalAverageSalary * serviceYears * accrualRate;
    return annualBenefit / 12;
  },

  /**
   * Calculate required savings for retirement goal
   */
  calculateRequiredSavings(
    targetMonthlyIncome: number,
    yearsInRetirement: number,
    annualReturnRate: number,
    inflationRate: number
  ): number {
    const monthlyReturnRate = annualReturnRate / 12;
    const monthlyInflationRate = inflationRate / 12;
    const months = yearsInRetirement * 12;

    // Present value of annuity calculation
    const realRate = (monthlyReturnRate - monthlyInflationRate) / (1 + monthlyInflationRate);
    return targetMonthlyIncome * ((1 - Math.pow(1 + realRate, -months)) / realRate);
  },

  /**
   * Validate ISO8601 date string
   */
  isValidISO8601(dateString: string): boolean {
    const iso8601Regex = /^\d{4}-\d{2}-\d{2}(T\d{2}:\d{2}:\d{2}(\.\d{3})?Z?)?$/;
    return iso8601Regex.test(dateString);
  },

  /**
   * Validate currency code (ISO 4217)
   */
  isValidCurrencyCode(code: string): boolean {
    const validCodes = ['USD', 'EUR', 'GBP', 'JPY', 'CNY', 'KRW']; // Add more as needed
    return validCodes.includes(code.toUpperCase());
  }
};

export default WIAPensionClient;
