/**
 * WIA-FIN-008: Health Insurance Data Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

import EventEmitter from 'eventemitter3';
import {
  WIAConfig,
  APIResponse,
  PaginatedResponse,
  InsuranceMember,
  InsuranceClaim,
  CoverageDetails,
  Provider,
  ClaimStatus,
} from './types';

export * from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface WIAInsuranceConfig extends WIAConfig {
  region?: string;
}

// ============================================================================
// Main SDK Client
// ============================================================================

export class WIAInsuranceClient {
  private config: Required<WIAInsuranceConfig>;
  private eventEmitter = new EventEmitter();

  constructor(config: WIAInsuranceConfig) {
    this.config = {
      endpoint: 'https://api.wia-standards.org/v1/insurance',
      timeout: 60000,
      debug: false,
      region: 'global',
      ...config,
    };

    if (!this.config.apiKey) {
      throw new Error('API key is required');
    }
  }

  // ==========================================================================
  // Member Operations
  // ==========================================================================

  async getMember(memberId: string): Promise<APIResponse<InsuranceMember>> {
    return this.makeRequest('GET', `/members/${memberId}`);
  }

  async searchMembers(query: {
    name?: string;
    policyNumber?: string;
    status?: string;
  }): Promise<PaginatedResponse<InsuranceMember>> {
    const params = new URLSearchParams(query as Record<string, string>);
    return this.makeRequest('GET', `/members?${params}`);
  }

  async updateMember(memberId: string, updates: Partial<InsuranceMember>): Promise<APIResponse<InsuranceMember>> {
    return this.makeRequest('PATCH', `/members/${memberId}`, updates);
  }

  async checkEligibility(memberId: string, serviceDate?: string): Promise<APIResponse<{
    eligible: boolean;
    coverageActive: boolean;
    deductibleMet: boolean;
    remainingDeductible: number;
  }>> {
    const params = serviceDate ? `?serviceDate=${serviceDate}` : '';
    return this.makeRequest('GET', `/members/${memberId}/eligibility${params}`);
  }

  // ==========================================================================
  // Claims Operations
  // ==========================================================================

  async submitClaim(claim: Partial<InsuranceClaim>): Promise<APIResponse<InsuranceClaim>> {
    return this.makeRequest('POST', '/claims', claim);
  }

  async getClaim(claimId: string): Promise<APIResponse<InsuranceClaim>> {
    return this.makeRequest('GET', `/claims/${claimId}`);
  }

  async listClaims(memberId: string, filters?: {
    status?: ClaimStatus;
    startDate?: string;
    endDate?: string;
    page?: number;
    pageSize?: number;
  }): Promise<PaginatedResponse<InsuranceClaim>> {
    const params = new URLSearchParams({ memberId, ...filters as Record<string, string> });
    return this.makeRequest('GET', `/claims?${params}`);
  }

  async updateClaimStatus(claimId: string, status: ClaimStatus, notes?: string): Promise<APIResponse<InsuranceClaim>> {
    return this.makeRequest('PATCH', `/claims/${claimId}/status`, { status, notes });
  }

  async appealClaim(claimId: string, reason: string, documents?: string[]): Promise<APIResponse<{
    appealId: string;
    status: string;
    submittedAt: string;
  }>> {
    return this.makeRequest('POST', `/claims/${claimId}/appeal`, { reason, documents });
  }

  // ==========================================================================
  // Coverage Operations
  // ==========================================================================

  async getCoverage(memberId: string): Promise<APIResponse<CoverageDetails>> {
    return this.makeRequest('GET', `/members/${memberId}/coverage`);
  }

  async checkCoverage(memberId: string, serviceCode: string): Promise<APIResponse<{
    covered: boolean;
    copay: number;
    coinsurance: number;
    priorAuthRequired: boolean;
  }>> {
    return this.makeRequest('GET', `/members/${memberId}/coverage/check?serviceCode=${serviceCode}`);
  }

  // ==========================================================================
  // Provider Operations
  // ==========================================================================

  async searchProviders(query: {
    specialty?: string;
    zipCode?: string;
    radius?: number;
    networkId?: string;
  }): Promise<PaginatedResponse<Provider>> {
    const params = new URLSearchParams(query as Record<string, string>);
    return this.makeRequest('GET', `/providers?${params}`);
  }

  async getProvider(npi: string): Promise<APIResponse<Provider>> {
    return this.makeRequest('GET', `/providers/${npi}`);
  }

  // ==========================================================================
  // Event Handling
  // ==========================================================================

  on(event: 'claimSubmitted' | 'claimUpdated' | 'error', callback: (...args: unknown[]) => void): void {
    this.eventEmitter.on(event, callback);
  }

  off(event: string, callback: (...args: unknown[]) => void): void {
    this.eventEmitter.off(event, callback);
  }

  // ==========================================================================
  // Private Methods
  // ==========================================================================

  private async makeRequest<T>(method: string, path: string, body?: unknown): Promise<T> {
    const url = `${this.config.endpoint}${path}`;

    if (this.config.debug) {
      console.log(`[WIA Insurance] ${method} ${url}`, body);
    }

    try {
      const response = await fetch(url, {
        method,
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${this.config.apiKey}`,
          'X-WIA-Standard': 'FIN-008',
          'X-WIA-Version': '1.0.0',
          'X-WIA-Region': this.config.region,
        },
        body: body ? JSON.stringify(body) : undefined,
      });

      const data = await response.json();

      if (!response.ok) {
        this.eventEmitter.emit('error', data);
      }

      return data;
    } catch (error: unknown) {
      const message = error instanceof Error ? error.message : 'Unknown error';
      this.eventEmitter.emit('error', { code: 'REQUEST_FAILED', message });
      throw error;
    }
  }
}

export function createClient(config: WIAInsuranceConfig): WIAInsuranceClient {
  return new WIAInsuranceClient(config);
}

export default WIAInsuranceClient;
