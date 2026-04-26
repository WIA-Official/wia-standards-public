/**
 * WIA-UNI-004 Economic Integration SDK
 *
 * Official TypeScript SDK for inter-Korean economic integration
 * including trade, investment, joint ventures, and SEZ management.
 *
 * @module @wia/economic-integration-sdk
 * @version 1.0.0
 */

import axios, { AxiosInstance } from 'axios';
import {
  ClientConfig,
  Trade,
  CreateTradeRequest,
  Investment,
  CreateInvestmentRequest,
  JointVenture,
  CreateJointVentureRequest,
  SEZ,
  CreateSEZRequest,
  ComplianceCheck,
  RiskAssessment,
  APIResponse,
} from './types';

export * from './types';

/**
 * Main client for WIA-UNI-004 Economic Integration API
 */
export class EconomicIntegrationClient {
  private client: AxiosInstance;
  private config: ClientConfig;

  constructor(config: ClientConfig) {
    this.config = config;

    const baseURL =
      config.baseURL ||
      (config.environment === 'production'
        ? 'https://api.wia.org/uni-004/v1'
        : 'https://sandbox-api.wia.org/uni-004/v1');

    this.client = axios.create({
      baseURL,
      timeout: config.timeout || 30000,
      headers: {
        'Content-Type': 'application/json',
        Authorization: `Bearer ${config.apiKey}`,
        'X-WIA-SDK-Version': '1.0.0',
      },
    });

    // Add retry logic
    this.client.interceptors.response.use(
      (response) => response,
      async (error) => {
        const retries = config.retries || 3;
        const retryCount = error.config.__retryCount || 0;

        if (retryCount < retries && this.isRetryableError(error)) {
          error.config.__retryCount = retryCount + 1;
          await this.delay(Math.pow(2, retryCount) * 1000);
          return this.client(error.config);
        }

        return Promise.reject(error);
      }
    );
  }

  // ============================================================================
  // Trade Operations
  // ============================================================================

  /**
   * Create a new trade declaration
   */
  async createTrade(request: CreateTradeRequest): Promise<Trade> {
    const response = await this.client.post<APIResponse<Trade>>(
      '/trade/declarations',
      request
    );
    return this.handleResponse(response.data);
  }

  /**
   * Get trade declaration by ID
   */
  async getTrade(tradeId: string): Promise<Trade> {
    const response = await this.client.get<APIResponse<Trade>>(
      `/trade/declarations/${tradeId}`
    );
    return this.handleResponse(response.data);
  }

  /**
   * List all trade declarations
   */
  async listTrades(params?: {
    status?: string;
    limit?: number;
    offset?: number;
  }): Promise<Trade[]> {
    const response = await this.client.get<APIResponse<Trade[]>>(
      '/trade/declarations',
      { params }
    );
    return this.handleResponse(response.data);
  }

  /**
   * Update trade declaration
   */
  async updateTrade(
    tradeId: string,
    updates: Partial<CreateTradeRequest>
  ): Promise<Trade> {
    const response = await this.client.patch<APIResponse<Trade>>(
      `/trade/declarations/${tradeId}`,
      updates
    );
    return this.handleResponse(response.data);
  }

  // ============================================================================
  // Investment Operations
  // ============================================================================

  /**
   * Submit investment application
   */
  async createInvestment(
    request: CreateInvestmentRequest
  ): Promise<Investment> {
    const response = await this.client.post<APIResponse<Investment>>(
      '/investment/applications',
      request
    );
    return this.handleResponse(response.data);
  }

  /**
   * Get investment application by ID
   */
  async getInvestment(investmentId: string): Promise<Investment> {
    const response = await this.client.get<APIResponse<Investment>>(
      `/investment/applications/${investmentId}`
    );
    return this.handleResponse(response.data);
  }

  /**
   * List investment applications
   */
  async listInvestments(params?: {
    status?: string;
    investmentType?: string;
    limit?: number;
    offset?: number;
  }): Promise<Investment[]> {
    const response = await this.client.get<APIResponse<Investment[]>>(
      '/investment/applications',
      { params }
    );
    return this.handleResponse(response.data);
  }

  // ============================================================================
  // Joint Venture Operations
  // ============================================================================

  /**
   * Create joint venture
   */
  async createJointVenture(
    request: CreateJointVentureRequest
  ): Promise<JointVenture> {
    const response = await this.client.post<APIResponse<JointVenture>>(
      '/joint-ventures',
      request
    );
    return this.handleResponse(response.data);
  }

  /**
   * Get joint venture by ID
   */
  async getJointVenture(jvId: string): Promise<JointVenture> {
    const response = await this.client.get<APIResponse<JointVenture>>(
      `/joint-ventures/${jvId}`
    );
    return this.handleResponse(response.data);
  }

  /**
   * List joint ventures
   */
  async listJointVentures(params?: {
    status?: string;
    limit?: number;
    offset?: number;
  }): Promise<JointVenture[]> {
    const response = await this.client.get<APIResponse<JointVenture[]>>(
      '/joint-ventures',
      { params }
    );
    return this.handleResponse(response.data);
  }

  // ============================================================================
  // SEZ Operations
  // ============================================================================

  /**
   * Create Special Economic Zone
   */
  async createSEZ(request: CreateSEZRequest): Promise<SEZ> {
    const response = await this.client.post<APIResponse<SEZ>>('/sez', request);
    return this.handleResponse(response.data);
  }

  /**
   * Get SEZ by ID
   */
  async getSEZ(sezId: string): Promise<SEZ> {
    const response = await this.client.get<APIResponse<SEZ>>(`/sez/${sezId}`);
    return this.handleResponse(response.data);
  }

  /**
   * List all SEZs
   */
  async listSEZs(params?: {
    status?: string;
    sezType?: string;
    limit?: number;
    offset?: number;
  }): Promise<SEZ[]> {
    const response = await this.client.get<APIResponse<SEZ[]>>('/sez', {
      params,
    });
    return this.handleResponse(response.data);
  }

  // ============================================================================
  // Compliance & Risk Operations
  // ============================================================================

  /**
   * Run compliance check
   */
  async runComplianceCheck(params: {
    entityName: string;
    country: string;
    type: 'INDIVIDUAL' | 'COMPANY';
  }): Promise<ComplianceCheck> {
    const response = await this.client.post<APIResponse<ComplianceCheck>>(
      '/compliance/check',
      params
    );
    return this.handleResponse(response.data);
  }

  /**
   * Get risk assessment
   */
  async getRiskAssessment(params: {
    projectType: string;
    investmentAmount: number;
    location: string;
  }): Promise<RiskAssessment> {
    const response = await this.client.post<APIResponse<RiskAssessment>>(
      '/risk/assessment',
      params
    );
    return this.handleResponse(response.data);
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  private handleResponse<T>(response: APIResponse<T>): T {
    if (!response.success) {
      throw new Error(
        response.error?.message || 'API request failed'
      );
    }
    return response.data!;
  }

  private isRetryableError(error: any): boolean {
    return (
      !error.response ||
      error.response.status === 429 ||
      error.response.status >= 500
    );
  }

  private delay(ms: number): Promise<void> {
    return new Promise((resolve) => setTimeout(resolve, ms));
  }
}

/**
 * Create a new Economic Integration client
 */
export function createClient(config: ClientConfig): EconomicIntegrationClient {
  return new EconomicIntegrationClient(config);
}

// Default export
export default EconomicIntegrationClient;
