/**
 * WIA-FIN-003 Micro-Lending Standard SDK
 *
 * 弘益人間 · Benefit All Humanity
 *
 * This SDK provides a comprehensive interface for interacting with
 * WIA-compliant micro-lending platforms.
 *
 * @module @wia/micro-lending
 * @version 1.0.0
 */

import axios, { AxiosInstance, AxiosError } from 'axios';
import {
  SDKConfig,
  Environment,
  Borrower,
  CreateBorrowerRequest,
  Lender,
  CreateLenderRequest,
  LenderPortfolio,
  Loan,
  CreateLoanRequest,
  FundLoanRequest,
  LoanSummary,
  CreditScore,
  Payment,
  MakePaymentRequest,
  LoanMatch,
  InvestmentOpportunity,
  PaginatedResponse,
  ErrorResponse,
  AutoInvestSettings,
  AutoRepaymentSettings,
  LoanStatus
} from './types';

// Export all types
export * from './types';

// ============================================================================
// Constants
// ============================================================================

const DEFAULT_ENDPOINTS = {
  development: 'https://api-dev.wia.org/v1/micro-lending',
  sandbox: 'https://api-sandbox.wia.org/v1/micro-lending',
  production: 'https://api.wia.org/v1/micro-lending'
};

const DEFAULT_TIMEOUT = 30000; // 30 seconds
const DEFAULT_MAX_RETRIES = 3;

// ============================================================================
// SDK Class
// ============================================================================

/**
 * WIA Micro-Lending SDK
 *
 * @example
 * ```typescript
 * import { MicroLendingSDK } from '@wia/micro-lending';
 *
 * const sdk = new MicroLendingSDK({
 *   apiKey: 'your-api-key',
 *   environment: 'production'
 * });
 *
 * const borrower = await sdk.createBorrower({
 *   type: 'individual',
 *   profile: { ... }
 * });
 * ```
 */
export class MicroLendingSDK {
  private client: AxiosInstance;
  private config: Required<SDKConfig>;

  /**
   * Create a new SDK instance
   *
   * @param config - SDK configuration
   */
  constructor(config: SDKConfig) {
    this.config = {
      apiKey: config.apiKey,
      environment: config.environment || Environment.PRODUCTION,
      apiEndpoint: config.apiEndpoint || DEFAULT_ENDPOINTS[config.environment || Environment.PRODUCTION],
      timeout: config.timeout || DEFAULT_TIMEOUT,
      maxRetries: config.maxRetries || DEFAULT_MAX_RETRIES
    };

    this.client = axios.create({
      baseURL: this.config.apiEndpoint,
      timeout: this.config.timeout,
      headers: {
        'Authorization': `Bearer ${this.config.apiKey}`,
        'Content-Type': 'application/json',
        'User-Agent': 'WIA-Micro-Lending-SDK/1.0.0'
      }
    });

    // Add response interceptor for error handling
    this.client.interceptors.response.use(
      response => response,
      error => this.handleError(error)
    );
  }

  // ==========================================================================
  // Borrower Methods
  // ==========================================================================

  /**
   * Create a new borrower
   *
   * @param request - Borrower creation request
   * @returns Created borrower
   *
   * @example
   * ```typescript
   * const borrower = await sdk.createBorrower({
   *   type: 'individual',
   *   profile: {
   *     firstName: 'Jane',
   *     lastName: 'Doe',
   *     email: 'jane@example.com',
   *     phone: '+1234567890',
   *     dateOfBirth: '1990-01-01',
   *     address: { ... }
   *   }
   * });
   * ```
   */
  async createBorrower(request: CreateBorrowerRequest): Promise<Borrower> {
    const response = await this.client.post<Borrower>('/borrowers', request);
    return response.data;
  }

  /**
   * Get borrower by ID
   *
   * @param borrowerId - Borrower UUID
   * @returns Borrower entity
   */
  async getBorrower(borrowerId: string): Promise<Borrower> {
    const response = await this.client.get<Borrower>(`/borrowers/${borrowerId}`);
    return response.data;
  }

  /**
   * Update borrower
   *
   * @param borrowerId - Borrower UUID
   * @param updates - Partial borrower updates
   * @returns Updated borrower
   */
  async updateBorrower(borrowerId: string, updates: Partial<CreateBorrowerRequest>): Promise<Borrower> {
    const response = await this.client.put<Borrower>(`/borrowers/${borrowerId}`, updates);
    return response.data;
  }

  /**
   * Get borrower's credit score
   *
   * @param borrowerId - Borrower UUID
   * @returns Credit score
   *
   * @example
   * ```typescript
   * const creditScore = await sdk.getCreditScore('borrower-uuid');
   * console.log(`Score: ${creditScore.score}/1000`);
   * console.log(`Rating: ${creditScore.rating}`);
   * ```
   */
  async getCreditScore(borrowerId: string): Promise<CreditScore> {
    const response = await this.client.get<CreditScore>(`/borrowers/${borrowerId}/credit-score`);
    return response.data;
  }

  // ==========================================================================
  // Lender Methods
  // ==========================================================================

  /**
   * Create a new lender
   *
   * @param request - Lender creation request
   * @returns Created lender
   *
   * @example
   * ```typescript
   * const lender = await sdk.createLender({
   *   type: 'individual',
   *   profile: { ... },
   *   investmentProfile: {
   *     totalCapital: 50000,
   *     availableCapital: 50000,
   *     investedCapital: 0,
   *     riskTolerance: 'moderate',
   *     preferences: { ... }
   *   }
   * });
   * ```
   */
  async createLender(request: CreateLenderRequest): Promise<Lender> {
    const response = await this.client.post<Lender>('/lenders', request);
    return response.data;
  }

  /**
   * Get lender by ID
   *
   * @param lenderId - Lender UUID
   * @returns Lender entity
   */
  async getLender(lenderId: string): Promise<Lender> {
    const response = await this.client.get<Lender>(`/lenders/${lenderId}`);
    return response.data;
  }

  /**
   * Update lender
   *
   * @param lenderId - Lender UUID
   * @param updates - Partial lender updates
   * @returns Updated lender
   */
  async updateLender(lenderId: string, updates: Partial<CreateLenderRequest>): Promise<Lender> {
    const response = await this.client.put<Lender>(`/lenders/${lenderId}`, updates);
    return response.data;
  }

  /**
   * Get lender's portfolio
   *
   * @param lenderId - Lender UUID
   * @returns Portfolio statistics
   *
   * @example
   * ```typescript
   * const portfolio = await sdk.getPortfolio('lender-uuid');
   * console.log(`Total invested: $${portfolio.totalInvested}`);
   * console.log(`Active loans: ${portfolio.activeLoans}`);
   * console.log(`Average ROI: ${portfolio.averageROI}%`);
   * ```
   */
  async getPortfolio(lenderId: string): Promise<LenderPortfolio> {
    const response = await this.client.get<LenderPortfolio>(`/lenders/${lenderId}/portfolio`);
    return response.data;
  }

  /**
   * Enable auto-invest for a lender
   *
   * @param lenderId - Lender UUID
   * @param settings - Auto-investment settings
   *
   * @example
   * ```typescript
   * await sdk.enableAutoInvest('lender-uuid', {
   *   amountPerLoan: 500,
   *   maxLoansPerDay: 10,
   *   diversificationRules: {
   *     maxPerBorrower: 1000,
   *     maxPerSector: 15000
   *   }
   * });
   * ```
   */
  async enableAutoInvest(lenderId: string, settings: AutoInvestSettings): Promise<void> {
    await this.client.post(`/lenders/${lenderId}/auto-invest`, settings);
  }

  /**
   * Disable auto-invest for a lender
   *
   * @param lenderId - Lender UUID
   */
  async disableAutoInvest(lenderId: string): Promise<void> {
    await this.client.delete(`/lenders/${lenderId}/auto-invest`);
  }

  // ==========================================================================
  // Loan Methods
  // ==========================================================================

  /**
   * Create a new loan
   *
   * @param request - Loan creation request
   * @returns Created loan
   *
   * @example
   * ```typescript
   * const loan = await sdk.createLoan({
   *   borrowerId: 'borrower-uuid',
   *   amount: 5000,
   *   currency: 'USD',
   *   purpose: 'business',
   *   description: 'Working capital for inventory',
   *   term: 12,
   *   interestRate: 0.12
   * });
   * ```
   */
  async createLoan(request: CreateLoanRequest): Promise<Loan> {
    const response = await this.client.post<Loan>('/loans', request);
    return response.data;
  }

  /**
   * Get loan by ID
   *
   * @param loanId - Loan UUID
   * @returns Loan entity
   */
  async getLoan(loanId: string): Promise<Loan> {
    const response = await this.client.get<Loan>(`/loans/${loanId}`);
    return response.data;
  }

  /**
   * Get loan status
   *
   * @param loanId - Loan UUID
   * @returns Loan status
   */
  async getLoanStatus(loanId: string): Promise<{ status: LoanStatus }> {
    const loan = await this.getLoan(loanId);
    return { status: loan.status };
  }

  /**
   * List loans with optional filters
   *
   * @param filters - Query parameters
   * @returns Paginated loan list
   *
   * @example
   * ```typescript
   * const loans = await sdk.listLoans({
   *   status: 'active',
   *   limit: 20,
   *   offset: 0
   * });
   * ```
   */
  async listLoans(filters?: {
    status?: LoanStatus;
    borrowerId?: string;
    limit?: number;
    offset?: number;
  }): Promise<PaginatedResponse<LoanSummary>> {
    const response = await this.client.get<PaginatedResponse<LoanSummary>>('/loans', {
      params: filters
    });
    return response.data;
  }

  /**
   * Fund a loan
   *
   * @param loanId - Loan UUID
   * @param request - Funding request
   *
   * @example
   * ```typescript
   * await sdk.fundLoan('loan-uuid', {
   *   lenderId: 'lender-uuid',
   *   amount: 1000
   * });
   * ```
   */
  async fundLoan(loanId: string, request: FundLoanRequest): Promise<void> {
    await this.client.post(`/loans/${loanId}/fund`, request);
  }

  /**
   * Cancel a loan
   *
   * @param loanId - Loan UUID
   */
  async cancelLoan(loanId: string): Promise<void> {
    await this.client.delete(`/loans/${loanId}`);
  }

  // ==========================================================================
  // Payment Methods
  // ==========================================================================

  /**
   * Make a payment
   *
   * @param request - Payment request
   * @returns Created payment
   *
   * @example
   * ```typescript
   * const payment = await sdk.makePayment({
   *   loanId: 'loan-uuid',
   *   amount: 450,
   *   method: 'bank_transfer',
   *   accountId: 'account-uuid'
   * });
   * ```
   */
  async makePayment(request: MakePaymentRequest): Promise<Payment> {
    const response = await this.client.post<Payment>('/payments', request);
    return response.data;
  }

  /**
   * Get payment by ID
   *
   * @param paymentId - Payment UUID
   * @returns Payment entity
   */
  async getPayment(paymentId: string): Promise<Payment> {
    const response = await this.client.get<Payment>(`/payments/${paymentId}`);
    return response.data;
  }

  /**
   * Get payment status
   *
   * @param paymentId - Payment UUID
   * @returns Payment status
   */
  async getPaymentStatus(paymentId: string): Promise<{ status: string }> {
    const payment = await this.getPayment(paymentId);
    return { status: payment.status };
  }

  /**
   * Setup automatic repayment
   *
   * @param loanId - Loan UUID
   * @param settings - Auto-repayment settings
   *
   * @example
   * ```typescript
   * await sdk.setupAutoRepayment('loan-uuid', {
   *   method: 'bank_transfer',
   *   accountId: 'account-uuid',
   *   frequency: 'monthly',
   *   dayOfMonth: 1
   * });
   * ```
   */
  async setupAutoRepayment(loanId: string, settings: AutoRepaymentSettings): Promise<void> {
    await this.client.post(`/loans/${loanId}/auto-repayment`, settings);
  }

  /**
   * Disable automatic repayment
   *
   * @param loanId - Loan UUID
   */
  async disableAutoRepayment(loanId: string): Promise<void> {
    await this.client.delete(`/loans/${loanId}/auto-repayment`);
  }

  // ==========================================================================
  // Matching Methods
  // ==========================================================================

  /**
   * Get potential lender matches for a loan
   *
   * @param loanId - Loan UUID
   * @returns Array of matches
   *
   * @example
   * ```typescript
   * const matches = await sdk.getMatchesForLoan('loan-uuid');
   * matches.forEach(match => {
   *   console.log(`Lender ${match.lenderId}: ${match.matchScore * 100}% match`);
   * });
   * ```
   */
  async getMatchesForLoan(loanId: string): Promise<LoanMatch[]> {
    const response = await this.client.get<{ matches: LoanMatch[] }>(`/loans/${loanId}/matches`);
    return response.data.matches;
  }

  /**
   * Get investment opportunities for a lender
   *
   * @param lenderId - Lender UUID
   * @returns Array of opportunities
   *
   * @example
   * ```typescript
   * const opportunities = await sdk.getInvestmentOpportunities('lender-uuid');
   * opportunities.forEach(opp => {
   *   console.log(`Loan ${opp.loanId}: ${opp.matchScore * 100}% match`);
   * });
   * ```
   */
  async getInvestmentOpportunities(lenderId: string): Promise<InvestmentOpportunity[]> {
    const response = await this.client.get<{ opportunities: InvestmentOpportunity[] }>(
      `/lenders/${lenderId}/opportunities`
    );
    return response.data.opportunities;
  }

  // ==========================================================================
  // Utility Methods
  // ==========================================================================

  /**
   * Health check
   *
   * @returns API health status
   */
  async healthCheck(): Promise<{ status: string; version: string }> {
    const response = await this.client.get<{ status: string; version: string }>('/health');
    return response.data;
  }

  /**
   * Get API version
   *
   * @returns API version
   */
  async getVersion(): Promise<string> {
    const health = await this.healthCheck();
    return health.version;
  }

  // ==========================================================================
  // Private Methods
  // ==========================================================================

  /**
   * Handle API errors
   *
   * @param error - Axios error
   * @throws Formatted error
   */
  private handleError(error: AxiosError<ErrorResponse>): never {
    if (error.response) {
      // Server responded with error
      const errorData = error.response.data;
      if (errorData?.error) {
        const err = new Error(errorData.error.message);
        (err as any).code = errorData.error.code;
        (err as any).details = errorData.error.details;
        (err as any).statusCode = error.response.status;
        throw err;
      }
    } else if (error.request) {
      // No response received
      throw new Error('No response from server. Please check your connection.');
    }

    // Other errors
    throw error;
  }
}

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * Calculate credit score from factors
 *
 * @param factors - Individual factor values (0-100)
 * @returns Credit score (0-1000)
 *
 * @example
 * ```typescript
 * const score = calculateCreditScore({
 *   paymentHistory: 70,
 *   financialStability: 65,
 *   networkTrust: 80,
 *   incomeVerification: 75,
 *   educationSkills: 60
 * });
 * console.log(score); // 720
 * ```
 */
export function calculateCreditScore(factors: {
  paymentHistory: number;
  financialStability: number;
  networkTrust: number;
  incomeVerification: number;
  educationSkills: number;
}): number {
  return Math.round(
    (factors.paymentHistory * 0.35 +
      factors.financialStability * 0.25 +
      factors.networkTrust * 0.20 +
      factors.incomeVerification * 0.15 +
      factors.educationSkills * 0.05) * 10
  );
}

/**
 * Calculate monthly payment amount
 *
 * @param principal - Loan principal amount
 * @param annualRate - Annual interest rate (decimal, e.g., 0.12)
 * @param termMonths - Loan term in months
 * @returns Monthly payment amount
 *
 * @example
 * ```typescript
 * const payment = calculateMonthlyPayment(5000, 0.12, 12);
 * console.log(`Monthly payment: $${payment.toFixed(2)}`);
 * ```
 */
export function calculateMonthlyPayment(
  principal: number,
  annualRate: number,
  termMonths: number
): number {
  const monthlyRate = annualRate / 12;
  if (monthlyRate === 0) {
    return principal / termMonths;
  }

  const payment =
    (principal * monthlyRate * Math.pow(1 + monthlyRate, termMonths)) /
    (Math.pow(1 + monthlyRate, termMonths) - 1);

  return Math.round(payment * 100) / 100;
}

/**
 * Calculate total interest
 *
 * @param principal - Loan principal amount
 * @param annualRate - Annual interest rate (decimal)
 * @param termMonths - Loan term in months
 * @returns Total interest amount
 */
export function calculateTotalInterest(
  principal: number,
  annualRate: number,
  termMonths: number
): number {
  const monthlyPayment = calculateMonthlyPayment(principal, annualRate, termMonths);
  const totalPayment = monthlyPayment * termMonths;
  return Math.round((totalPayment - principal) * 100) / 100;
}

/**
 * Validate email format
 *
 * @param email - Email address
 * @returns True if valid
 */
export function isValidEmail(email: string): boolean {
  const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
  return emailRegex.test(email);
}

/**
 * Validate phone number (E.164 format)
 *
 * @param phone - Phone number
 * @returns True if valid
 */
export function isValidPhone(phone: string): boolean {
  const phoneRegex = /^\+[1-9]\d{1,14}$/;
  return phoneRegex.test(phone);
}

// ============================================================================
// Default Export
// ============================================================================

export default MicroLendingSDK;
