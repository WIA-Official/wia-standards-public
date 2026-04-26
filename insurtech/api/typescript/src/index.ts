/**
 * WIA-FIN-005 InsurTech Standard - TypeScript SDK
 *
 * @package @wia/insurtech
 * @version 1.0.0
 * @license MIT
 * @author WIA Standards Committee
 */

import axios, { AxiosInstance } from 'axios';
import WebSocket from 'ws';
import type {
  InsurTechClientConfig,
  Policy,
  Claim,
  Quote,
  QuoteRequest,
  RiskProfile,
  UnderwritingAssessment,
  APIResponse,
  ClaimStatus
} from './types';

/**
 * Main client for interacting with WIA-FIN-005 InsurTech APIs
 */
export class InsurTechClient {
  private apiClient: AxiosInstance;
  private config: Required<InsurTechClientConfig>;
  private ws?: WebSocket;

  /**
   * Create a new InsurTech client
   * @param config Client configuration
   */
  constructor(config: InsurTechClientConfig) {
    this.config = {
      apiKey: config.apiKey,
      environment: config.environment || 'production',
      baseURL: config.baseURL || 'https://api.wia.org/v1/insurtech',
      timeout: config.timeout || 30000,
      retryAttempts: config.retryAttempts || 3
    };

    this.apiClient = axios.create({
      baseURL: this.config.baseURL,
      timeout: this.config.timeout,
      headers: {
        'Authorization': `Bearer ${this.config.apiKey}`,
        'Content-Type': 'application/json',
        'X-WIA-Environment': this.config.environment
      }
    });
  }

  /**
   * Generate an insurance quote
   * @param request Quote request parameters
   * @returns Insurance quote
   */
  async generateQuote(request: QuoteRequest): Promise<Quote> {
    const response = await this.apiClient.post<APIResponse<Quote>>(
      '/quotes',
      request
    );

    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to generate quote');
    }

    return response.data.data;
  }

  /**
   * Create a new insurance policy
   * @param quoteId Quote identifier
   * @param paymentMethod Payment method
   * @param startDate Policy start date (ISO 8601)
   * @returns Created policy
   */
  async createPolicy(
    quoteId: string,
    paymentMethod: string,
    startDate?: string
  ): Promise<Policy> {
    const response = await this.apiClient.post<APIResponse<Policy>>(
      '/policies',
      {
        quoteId,
        paymentMethod,
        startDate: startDate || new Date().toISOString()
      }
    );

    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to create policy');
    }

    return response.data.data;
  }

  /**
   * Get policy details
   * @param policyId Policy identifier
   * @returns Policy information
   */
  async getPolicy(policyId: string): Promise<Policy> {
    const response = await this.apiClient.get<APIResponse<Policy>>(
      `/policies/${policyId}`
    );

    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to get policy');
    }

    return response.data.data;
  }

  /**
   * Update policy information
   * @param policyId Policy identifier
   * @param updates Policy updates
   * @returns Updated policy
   */
  async updatePolicy(policyId: string, updates: Partial<Policy>): Promise<Policy> {
    const response = await this.apiClient.put<APIResponse<Policy>>(
      `/policies/${policyId}`,
      updates
    );

    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to update policy');
    }

    return response.data.data;
  }

  /**
   * Cancel policy
   * @param policyId Policy identifier
   * @returns Cancelled policy
   */
  async cancelPolicy(policyId: string): Promise<Policy> {
    const response = await this.apiClient.delete<APIResponse<Policy>>(
      `/policies/${policyId}`
    );

    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to cancel policy');
    }

    return response.data.data;
  }

  /**
   * File a new insurance claim
   * @param claimData Claim information
   * @returns Filed claim
   */
  async fileClaim(claimData: Partial<Claim>): Promise<Claim> {
    const response = await this.apiClient.post<APIResponse<Claim>>(
      '/claims',
      claimData
    );

    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to file claim');
    }

    return response.data.data;
  }

  /**
   * Get claim details
   * @param claimId Claim identifier
   * @returns Claim information
   */
  async getClaim(claimId: string): Promise<Claim> {
    const response = await this.apiClient.get<APIResponse<Claim>>(
      `/claims/${claimId}`
    );

    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to get claim');
    }

    return response.data.data;
  }

  /**
   * Update claim status
   * @param claimId Claim identifier
   * @param status New claim status
   * @returns Updated claim
   */
  async updateClaimStatus(claimId: string, status: ClaimStatus): Promise<Claim> {
    const response = await this.apiClient.patch<APIResponse<Claim>>(
      `/claims/${claimId}/status`,
      { status }
    );

    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to update claim status');
    }

    return response.data.data;
  }

  /**
   * Get customer risk profile
   * @param customerId Customer identifier
   * @returns Risk profile
   */
  async getRiskProfile(customerId: string): Promise<RiskProfile> {
    const response = await this.apiClient.get<APIResponse<RiskProfile>>(
      `/customers/${customerId}/risk-profile`
    );

    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to get risk profile');
    }

    return response.data.data;
  }

  /**
   * Request underwriting assessment
   * @param applicationId Application identifier
   * @returns Underwriting assessment
   */
  async requestUnderwriting(applicationId: string): Promise<UnderwritingAssessment> {
    const response = await this.apiClient.post<APIResponse<UnderwritingAssessment>>(
      '/underwriting',
      { applicationId }
    );

    if (!response.data.success || !response.data.data) {
      throw new Error(response.data.error?.message || 'Failed to request underwriting');
    }

    return response.data.data;
  }

  /**
   * Connect to real-time event stream via WebSocket
   * @param channels Channels to subscribe to
   * @param onEvent Event handler callback
   */
  connectWebSocket(
    channels: string[],
    onEvent: (event: any) => void
  ): void {
    const wsUrl = this.config.baseURL.replace('https://', 'wss://').replace('http://', 'ws://');
    this.ws = new WebSocket(`${wsUrl}/stream`);

    this.ws.on('open', () => {
      this.ws?.send(JSON.stringify({
        action: 'subscribe',
        channels,
        apiKey: this.config.apiKey
      }));
    });

    this.ws.on('message', (data: WebSocket.Data) => {
      try {
        const event = JSON.parse(data.toString());
        onEvent(event);
      } catch (error) {
        console.error('Failed to parse WebSocket message:', error);
      }
    });

    this.ws.on('error', (error) => {
      console.error('WebSocket error:', error);
    });
  }

  /**
   * Disconnect from WebSocket stream
   */
  disconnectWebSocket(): void {
    if (this.ws) {
      this.ws.close();
      this.ws = undefined;
    }
  }
}

/**
 * Risk score calculator utility
 */
export class RiskScoreCalculator {
  /**
   * Calculate transaction risk score
   * @param transaction Transaction data
   * @param customer Customer data
   * @returns Risk score (0-100)
   */
  static calculateTransactionRisk(transaction: any, customer: any): number {
    let score = 50; // Base score

    // Age factor
    if (customer.age < 25) score += 15;
    else if (customer.age > 60) score += 10;
    else score -= 5;

    // Claim history
    const claimCount = customer.claimHistory?.length || 0;
    score += claimCount * 8;

    // Transaction amount
    if (transaction.amount > 100000) score += 10;
    else if (transaction.amount < 10000) score -= 5;

    // Normalize to 0-100
    return Math.max(0, Math.min(100, score));
  }

  /**
   * Determine if claim requires manual review
   * @param riskScore Risk score
   * @param claimAmount Claim amount
   * @returns True if manual review required
   */
  static requiresManualReview(riskScore: number, claimAmount: number): boolean {
    return riskScore > 70 || claimAmount > 50000;
  }

  /**
   * Check if claim should be auto-approved
   * @param riskScore Risk score
   * @param claimAmount Claim amount
   * @returns True if auto-approve
   */
  static shouldAutoApprove(riskScore: number, claimAmount: number): boolean {
    return riskScore < 20 && claimAmount < 5000;
  }
}

// Export all types
export * from './types';

// Default export
export default InsurTechClient;
