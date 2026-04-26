/**
 * WIA-FIN-023 Financial Inclusion Standard SDK
 *
 * @version 2.0.0
 * @license MIT
 */

import axios, { AxiosInstance } from 'axios';
import * as CryptoJS from 'crypto-js';
import {
  SDKConfig,
  APIResponse,
  PaginatedResponse,
  User,
  Account,
  Transaction,
  TransactionRequest,
  P2PTransferRequest,
  BillPayment,
  MerchantPayment,
  LoanApplication,
  Loan,
  CreditScore,
  SavingsGoal,
  Investment,
  InsurancePolicy,
  Agent,
  AgentTransaction,
  UsageMetrics,
  ImpactMetrics,
  WebhookPayload,
  KYCLevel,
  UserSegment
} from './types';

export * from './types';

/**
 * Main SDK class for WIA-FIN-023 Financial Inclusion Standard
 */
export class FinancialInclusionSDK {
  private client: AxiosInstance;
  private config: SDKConfig;

  constructor(config: SDKConfig) {
    this.config = {
      environment: 'sandbox',
      baseURL: config.baseURL || this.getDefaultBaseURL(config.environment),
      timeout: config.timeout || 30000,
      retryAttempts: config.retryAttempts || 3,
      locale: config.locale || 'en',
      ...config
    };

    this.client = axios.create({
      baseURL: this.config.baseURL,
      timeout: this.config.timeout,
      headers: {
        'Content-Type': 'application/json',
        'X-API-Key': this.config.apiKey,
        'X-SDK-Version': '2.0.0',
        'Accept-Language': this.config.locale
      }
    });

    this.setupInterceptors();
  }

  private getDefaultBaseURL(env: 'sandbox' | 'production'): string {
    return env === 'production'
      ? 'https://api.wia.org/fin-023/v2'
      : 'https://sandbox.wia.org/fin-023/v2';
  }

  private setupInterceptors(): void {
    // Request interceptor
    this.client.interceptors.request.use(
      (config) => {
        config.headers['X-Request-ID'] = this.generateRequestId();
        config.headers['X-Timestamp'] = new Date().toISOString();
        return config;
      },
      (error) => Promise.reject(error)
    );

    // Response interceptor
    this.client.interceptors.response.use(
      (response) => response,
      async (error) => {
        const { config, response } = error;

        // Retry logic
        if (!config._retry && config._retryCount < this.config.retryAttempts!) {
          config._retry = true;
          config._retryCount = (config._retryCount || 0) + 1;
          return this.client.request(config);
        }

        return Promise.reject(error);
      }
    );
  }

  private generateRequestId(): string {
    return CryptoJS.lib.WordArray.random(16).toString();
  }

  // =========================================================================
  // User & Account Management
  // =========================================================================

  /**
   * Create a new user account
   */
  async createAccount(params: {
    phoneNumber: string;
    firstName: string;
    lastName: string;
    kycLevel: KYCLevel;
    userSegment?: UserSegment;
    email?: string;
    dateOfBirth?: string;
  }): Promise<APIResponse<{ user: User; account: Account }>> {
    const response = await this.client.post('/accounts', params);
    return response.data;
  }

  /**
   * Get user account details
   */
  async getAccount(accountId: string): Promise<APIResponse<Account>> {
    const response = await this.client.get(`/accounts/${accountId}`);
    return response.data;
  }

  /**
   * Get account balance
   */
  async getBalance(accountId: string): Promise<APIResponse<{ balance: number; currency: string }>> {
    const response = await this.client.get(`/accounts/${accountId}/balance`);
    return response.data;
  }

  /**
   * Update KYC level
   */
  async upgradeKYC(userId: string, kycLevel: KYCLevel, documents?: any): Promise<APIResponse<User>> {
    const response = await this.client.post(`/users/${userId}/kyc/upgrade`, {
      kycLevel,
      documents
    });
    return response.data;
  }

  // =========================================================================
  // Transactions
  // =========================================================================

  /**
   * Process a transaction
   */
  async processTransaction(request: TransactionRequest): Promise<APIResponse<Transaction>> {
    const response = await this.client.post('/transactions', request);
    return response.data;
  }

  /**
   * Person-to-person money transfer
   */
  async p2pTransfer(request: P2PTransferRequest): Promise<APIResponse<Transaction>> {
    const response = await this.client.post('/transactions/p2p', request);
    return response.data;
  }

  /**
   * Bill payment
   */
  async payBill(accountId: string, payment: BillPayment, pin: string): Promise<APIResponse<Transaction>> {
    const response = await this.client.post('/transactions/bill-payment', {
      fromAccountId: accountId,
      ...payment,
      pin
    });
    return response.data;
  }

  /**
   * Merchant payment
   */
  async payMerchant(accountId: string, payment: MerchantPayment, pin: string): Promise<APIResponse<Transaction>> {
    const response = await this.client.post('/transactions/merchant-payment', {
      fromAccountId: accountId,
      ...payment,
      pin
    });
    return response.data;
  }

  /**
   * Get transaction history
   */
  async getTransactionHistory(
    accountId: string,
    params?: {
      page?: number;
      pageSize?: number;
      startDate?: Date;
      endDate?: Date;
    }
  ): Promise<APIResponse<PaginatedResponse<Transaction>>> {
    const response = await this.client.get(`/accounts/${accountId}/transactions`, { params });
    return response.data;
  }

  /**
   * Reverse a transaction (within 30 minutes)
   */
  async reverseTransaction(transactionId: string, reason: string): Promise<APIResponse<Transaction>> {
    const response = await this.client.post(`/transactions/${transactionId}/reverse`, { reason });
    return response.data;
  }

  // =========================================================================
  // Credit & Lending
  // =========================================================================

  /**
   * Apply for a loan
   */
  async applyForLoan(application: LoanApplication): Promise<APIResponse<Loan>> {
    const response = await this.client.post('/loans/apply', application);
    return response.data;
  }

  /**
   * Get credit score
   */
  async getCreditScore(userId: string): Promise<APIResponse<CreditScore>> {
    const response = await this.client.get(`/users/${userId}/credit-score`);
    return response.data;
  }

  /**
   * Get loan details
   */
  async getLoan(loanId: string): Promise<APIResponse<Loan>> {
    const response = await this.client.get(`/loans/${loanId}`);
    return response.data;
  }

  /**
   * Make loan repayment
   */
  async repayLoan(loanId: string, amount: number, accountId: string, pin: string): Promise<APIResponse<Transaction>> {
    const response = await this.client.post(`/loans/${loanId}/repay`, {
      amount,
      accountId,
      pin
    });
    return response.data;
  }

  // =========================================================================
  // Savings & Investment
  // =========================================================================

  /**
   * Create savings goal
   */
  async createSavingsGoal(goal: Omit<SavingsGoal, 'id' | 'currentAmount' | 'status'>): Promise<APIResponse<SavingsGoal>> {
    const response = await this.client.post('/savings/goals', goal);
    return response.data;
  }

  /**
   * Contribute to savings goal
   */
  async contributeToSavings(goalId: string, amount: number, accountId: string, pin: string): Promise<APIResponse<Transaction>> {
    const response = await this.client.post(`/savings/goals/${goalId}/contribute`, {
      amount,
      accountId,
      pin
    });
    return response.data;
  }

  /**
   * Create investment
   */
  async invest(investment: Omit<Investment, 'id' | 'returns' | 'purchaseDate'>): Promise<APIResponse<Investment>> {
    const response = await this.client.post('/investments', investment);
    return response.data;
  }

  // =========================================================================
  // Insurance
  // =========================================================================

  /**
   * Purchase insurance policy
   */
  async purchaseInsurance(policy: Omit<InsurancePolicy, 'id' | 'status' | 'startDate' | 'endDate'>): Promise<APIResponse<InsurancePolicy>> {
    const response = await this.client.post('/insurance/purchase', policy);
    return response.data;
  }

  /**
   * Get insurance policies
   */
  async getInsurancePolicies(userId: string): Promise<APIResponse<InsurancePolicy[]>> {
    const response = await this.client.get(`/users/${userId}/insurance`);
    return response.data;
  }

  // =========================================================================
  // Agent Network
  // =========================================================================

  /**
   * Find nearby agents
   */
  async findNearbyAgents(latitude: number, longitude: number, radius: number = 5): Promise<APIResponse<Agent[]>> {
    const response = await this.client.get('/agents/nearby', {
      params: { latitude, longitude, radius }
    });
    return response.data;
  }

  /**
   * Process agent transaction
   */
  async processAgentTransaction(transaction: {
    agentId: string;
    customerId: string;
    service: string;
    amount: number;
    pin: string;
  }): Promise<APIResponse<AgentTransaction>> {
    const response = await this.client.post('/agents/transactions', transaction);
    return response.data;
  }

  // =========================================================================
  // Analytics & Reporting
  // =========================================================================

  /**
   * Get usage metrics
   */
  async getUsageMetrics(startDate: Date, endDate: Date): Promise<APIResponse<UsageMetrics>> {
    const response = await this.client.get('/analytics/usage', {
      params: { startDate, endDate }
    });
    return response.data;
  }

  /**
   * Get impact metrics
   */
  async getImpactMetrics(): Promise<APIResponse<ImpactMetrics>> {
    const response = await this.client.get('/analytics/impact');
    return response.data;
  }

  // =========================================================================
  // Webhooks
  // =========================================================================

  /**
   * Verify webhook signature
   */
  verifyWebhookSignature(payload: WebhookPayload, secret: string): boolean {
    const { signature, ...data } = payload;
    const computedSignature = CryptoJS.HmacSHA256(
      JSON.stringify(data),
      secret
    ).toString();
    return signature === computedSignature;
  }

  /**
   * Register webhook endpoint
   */
  async registerWebhook(url: string, events: string[]): Promise<APIResponse<{ webhookId: string }>> {
    const response = await this.client.post('/webhooks', { url, events });
    return response.data;
  }
}

/**
 * Default export
 */
export default FinancialInclusionSDK;

/**
 * Helper function to create SDK instance
 */
export function createClient(config: SDKConfig): FinancialInclusionSDK {
  return new FinancialInclusionSDK(config);
}
