/**
 * WIA-SOC-019 Healthcare Insurance Standard - TypeScript SDK
 * @version 1.0.0
 * @license MIT
 * @description Complete SDK for healthcare insurance operations including
 * enrollment, claims processing, eligibility verification, provider search,
 * premium calculation, and cross-border healthcare services.
 */

import axios, { AxiosInstance, AxiosError } from 'axios';
import type {
  APIConfig,
  APIError,
  Member,
  EnrollmentRequest,
  EnrollmentResponse,
  Claim,
  ClaimSubmission,
  ClaimSubmissionResponse,
  EligibilityRequest,
  EligibilityResponse,
  BenefitsRequest,
  BenefitsResponse,
  Provider,
  ProviderSearchRequest,
  ProviderSearchResponse,
  PremiumCalculationRequest,
  PremiumCalculationResponse,
  CrossBorderRequest,
  CrossBorderResponse,
  PaginationParams,
  PaginatedResponse,
} from './types';

export * from './types';

/**
 * Main Healthcare Insurance API Client
 */
export class HealthcareInsuranceAPI {
  private client: AxiosInstance;
  private apiKey: string;

  constructor(config: APIConfig) {
    this.apiKey = config.apiKey;

    this.client = axios.create({
      baseURL: config.baseURL || 'https://api.healthcare-insurance.wia.org/v1',
      timeout: config.timeout || 30000,
      headers: {
        'Content-Type': 'application/json',
        'Accept': 'application/json',
        'Authorization': `Bearer ${config.apiKey}`,
      },
    });

    // Request interceptor for logging
    this.client.interceptors.request.use(
      (request) => {
        request.headers['X-Request-ID'] = this.generateRequestId();
        request.headers['X-API-Version'] = '1.0';
        return request;
      },
      (error) => Promise.reject(error)
    );

    // Response interceptor for error handling
    this.client.interceptors.response.use(
      (response) => response,
      (error: AxiosError) => {
        const apiError: APIError = {
          code: error.response?.data?.error?.code || 'UNKNOWN_ERROR',
          message: error.response?.data?.error?.message || error.message,
          details: error.response?.data?.error?.details,
          requestId: error.config?.headers?.['X-Request-ID'] as string,
        };
        return Promise.reject(apiError);
      }
    );
  }

  private generateRequestId(): string {
    return `req-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
  }

  // ===========================
  // Enrollment Operations
  // ===========================

  /**
   * Enroll a new member
   */
  async enrollMember(request: EnrollmentRequest): Promise<EnrollmentResponse> {
    const response = await this.client.post<EnrollmentResponse>('/enrollments', request);
    return response.data;
  }

  /**
   * Get member details by ID
   */
  async getMember(memberId: string): Promise<Member> {
    const response = await this.client.get<Member>(`/members/${memberId}`);
    return response.data;
  }

  /**
   * Update member information
   */
  async updateMember(memberId: string, updates: Partial<Member>): Promise<Member> {
    const response = await this.client.patch<Member>(`/members/${memberId}`, updates);
    return response.data;
  }

  /**
   * Terminate member coverage
   */
  async terminateCoverage(memberId: string, terminationDate: string): Promise<void> {
    await this.client.post(`/members/${memberId}/terminate`, { terminationDate });
  }

  // ===========================
  // Claims Operations
  // ===========================

  /**
   * Submit a new claim
   */
  async submitClaim(claim: ClaimSubmission): Promise<ClaimSubmissionResponse> {
    const response = await this.client.post<ClaimSubmissionResponse>('/claims', claim);
    return response.data;
  }

  /**
   * Get claim details by ID
   */
  async getClaim(claimId: string): Promise<Claim> {
    const response = await this.client.get<Claim>(`/claims/${claimId}`);
    return response.data;
  }

  /**
   * Search claims with filters
   */
  async searchClaims(
    filters: {
      memberId?: string;
      providerId?: string;
      status?: string;
      fromDate?: string;
      toDate?: string;
    },
    pagination?: PaginationParams
  ): Promise<PaginatedResponse<Claim>> {
    const response = await this.client.get<PaginatedResponse<Claim>>('/claims', {
      params: { ...filters, ...pagination },
    });
    return response.data;
  }

  /**
   * Appeal a denied claim
   */
  async appealClaim(
    claimId: string,
    reason: string,
    supportingDocs?: string[]
  ): Promise<{ appealId: string }> {
    const response = await this.client.post<{ appealId: string }>(
      `/claims/${claimId}/appeals`,
      { reason, supportingDocs }
    );
    return response.data;
  }

  // ===========================
  // Eligibility Operations
  // ===========================

  /**
   * Verify member eligibility for a service
   */
  async verifyEligibility(request: EligibilityRequest): Promise<EligibilityResponse> {
    const response = await this.client.post<EligibilityResponse>(
      '/eligibility/verify',
      request
    );
    return response.data;
  }

  /**
   * Check coverage benefits for a service type
   */
  async checkBenefits(request: BenefitsRequest): Promise<BenefitsResponse> {
    const response = await this.client.get<BenefitsResponse>(
      `/members/${request.memberId}/benefits`,
      { params: { serviceType: request.serviceType } }
    );
    return response.data;
  }

  // ===========================
  // Provider Operations
  // ===========================

  /**
   * Search for providers
   */
  async searchProviders(request: ProviderSearchRequest): Promise<ProviderSearchResponse> {
    const response = await this.client.get<ProviderSearchResponse>('/providers/search', {
      params: request,
    });
    return response.data;
  }

  /**
   * Get provider details by ID
   */
  async getProvider(providerId: string): Promise<Provider> {
    const response = await this.client.get<Provider>(`/providers/${providerId}`);
    return response.data;
  }

  /**
   * Check provider network status
   */
  async checkProviderNetwork(
    providerId: string,
    planId: string
  ): Promise<{ inNetwork: boolean; effectiveDate: string }> {
    const response = await this.client.get<{ inNetwork: boolean; effectiveDate: string }>(
      `/providers/${providerId}/network-status`,
      { params: { planId } }
    );
    return response.data;
  }

  // ===========================
  // Premium Operations
  // ===========================

  /**
   * Calculate premium for a member
   */
  async calculatePremium(
    request: PremiumCalculationRequest
  ): Promise<PremiumCalculationResponse> {
    const response = await this.client.post<PremiumCalculationResponse>(
      '/premiums/calculate',
      request
    );
    return response.data;
  }

  /**
   * Get payment history for a member
   */
  async getPaymentHistory(
    memberId: string,
    fromDate: string,
    toDate: string
  ): Promise<{ payments: any[] }> {
    const response = await this.client.get<{ payments: any[] }>(
      `/members/${memberId}/payments`,
      { params: { fromDate, toDate } }
    );
    return response.data;
  }

  // ===========================
  // Cross-Border Operations
  // ===========================

  /**
   * Check cross-border healthcare coverage
   */
  async checkCrossBorderCoverage(
    request: CrossBorderRequest
  ): Promise<CrossBorderResponse> {
    const response = await this.client.post<CrossBorderResponse>(
      '/cross-border/coverage-check',
      request
    );
    return response.data;
  }

  /**
   * Get international network providers
   */
  async getInternationalProviders(
    country: string,
    city?: string
  ): Promise<Provider[]> {
    const response = await this.client.get<{ providers: Provider[] }>(
      '/cross-border/providers',
      { params: { country, city } }
    );
    return response.data.providers;
  }
}

// ===========================
// Convenience Functions
// ===========================

/**
 * Create a new Healthcare Insurance API client instance
 */
export function createClient(apiKey: string, baseURL?: string): HealthcareInsuranceAPI {
  return new HealthcareInsuranceAPI({ apiKey, baseURL });
}

/**
 * Default export
 */
export default HealthcareInsuranceAPI;
