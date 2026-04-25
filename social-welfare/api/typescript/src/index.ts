/**
 * WIA-SOC-017: Social Welfare Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

import {
  ApiResponse,
  BeneficiaryProfile,
  BenefitApplication,
  BenefitPayment,
  CaseRecord,
  FraudAlert,
  WelfareProgram,
  PaginationParams,
} from './types';

export * from './types';

// ============================================================================
// Client Configuration
// ============================================================================

/**
 * Client configuration options
 */
export interface SocialWelfareClientConfig {
  apiKey: string;
  endpoint?: string;
  timeout?: number;
  retries?: number;
}

/**
 * Default configuration
 */
const DEFAULT_CONFIG = {
  endpoint: 'https://api.wia.org/soc-017/v1',
  timeout: 30000,
  retries: 3,
};

// ============================================================================
// Social Welfare Client
// ============================================================================

/**
 * WIA-SOC-017 Social Welfare API Client
 *
 * @example
 * ```typescript
 * const client = new SocialWelfareClient({
 *   apiKey: 'your-api-key'
 * });
 *
 * // Get beneficiary profile
 * const profile = await client.getBeneficiary('BEN-123');
 *
 * // Submit application
 * const application = await client.submitApplication({
 *   beneficiaryId: 'BEN-123',
 *   programId: 'PROG-FOOD-001',
 *   data: { ... }
 * });
 * ```
 */
export class SocialWelfareClient {
  private config: Required<SocialWelfareClientConfig>;

  constructor(config: SocialWelfareClientConfig) {
    this.config = {
      ...DEFAULT_CONFIG,
      ...config,
    };
  }

  /**
   * Make HTTP request
   */
  private async request<T>(
    method: string,
    path: string,
    data?: any
  ): Promise<ApiResponse<T>> {
    const url = `${this.config.endpoint}${path}`;
    const headers: Record<string, string> = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
    };

    try {
      const response = await fetch(url, {
        method,
        headers,
        body: data ? JSON.stringify(data) : undefined,
      });

      const result = await response.json();
      return result as ApiResponse<T>;
    } catch (error: any) {
      return {
        success: false,
        error: {
          code: 'REQUEST_FAILED',
          message: error.message,
          details: error,
        },
      };
    }
  }

  // ==========================================================================
  // Beneficiary Management
  // ==========================================================================

  /**
   * Get beneficiary profile
   */
  async getBeneficiary(beneficiaryId: string): Promise<ApiResponse<BeneficiaryProfile>> {
    return this.request<BeneficiaryProfile>(
      'GET',
      `/beneficiaries/${beneficiaryId}`
    );
  }

  /**
   * Create beneficiary profile
   */
  async createBeneficiary(
    profile: Partial<BeneficiaryProfile>
  ): Promise<ApiResponse<BeneficiaryProfile>> {
    return this.request<BeneficiaryProfile>(
      'POST',
      '/beneficiaries',
      profile
    );
  }

  /**
   * Update beneficiary profile
   */
  async updateBeneficiary(
    beneficiaryId: string,
    updates: Partial<BeneficiaryProfile>
  ): Promise<ApiResponse<BeneficiaryProfile>> {
    return this.request<BeneficiaryProfile>(
      'PUT',
      `/beneficiaries/${beneficiaryId}`,
      updates
    );
  }

  /**
   * List beneficiaries
   */
  async listBeneficiaries(
    params?: PaginationParams
  ): Promise<ApiResponse<BeneficiaryProfile[]>> {
    const queryString = params ? new URLSearchParams(params as any).toString() : '';
    return this.request<BeneficiaryProfile[]>(
      'GET',
      `/beneficiaries${queryString ? '?' + queryString : ''}`
    );
  }

  // ==========================================================================
  // Program Management
  // ==========================================================================

  /**
   * Get welfare program
   */
  async getProgram(programId: string): Promise<ApiResponse<WelfareProgram>> {
    return this.request<WelfareProgram>(
      'GET',
      `/programs/${programId}`
    );
  }

  /**
   * List welfare programs
   */
  async listPrograms(
    params?: PaginationParams
  ): Promise<ApiResponse<WelfareProgram[]>> {
    const queryString = params ? new URLSearchParams(params as any).toString() : '';
    return this.request<WelfareProgram[]>(
      'GET',
      `/programs${queryString ? '?' + queryString : ''}`
    );
  }

  /**
   * Create welfare program
   */
  async createProgram(
    program: Partial<WelfareProgram>
  ): Promise<ApiResponse<WelfareProgram>> {
    return this.request<WelfareProgram>(
      'POST',
      '/programs',
      program
    );
  }

  // ==========================================================================
  // Application Management
  // ==========================================================================

  /**
   * Submit benefit application
   */
  async submitApplication(
    application: Partial<BenefitApplication>
  ): Promise<ApiResponse<BenefitApplication>> {
    return this.request<BenefitApplication>(
      'POST',
      '/applications',
      application
    );
  }

  /**
   * Get application status
   */
  async getApplication(
    applicationId: string
  ): Promise<ApiResponse<BenefitApplication>> {
    return this.request<BenefitApplication>(
      'GET',
      `/applications/${applicationId}`
    );
  }

  /**
   * Update application
   */
  async updateApplication(
    applicationId: string,
    updates: Partial<BenefitApplication>
  ): Promise<ApiResponse<BenefitApplication>> {
    return this.request<BenefitApplication>(
      'PUT',
      `/applications/${applicationId}`,
      updates
    );
  }

  /**
   * List applications
   */
  async listApplications(
    params?: PaginationParams & { beneficiaryId?: string; programId?: string }
  ): Promise<ApiResponse<BenefitApplication[]>> {
    const queryString = params ? new URLSearchParams(params as any).toString() : '';
    return this.request<BenefitApplication[]>(
      'GET',
      `/applications${queryString ? '?' + queryString : ''}`
    );
  }

  /**
   * Withdraw application
   */
  async withdrawApplication(
    applicationId: string
  ): Promise<ApiResponse<BenefitApplication>> {
    return this.request<BenefitApplication>(
      'POST',
      `/applications/${applicationId}/withdraw`
    );
  }

  // ==========================================================================
  // Benefit Distribution
  // ==========================================================================

  /**
   * Get payment details
   */
  async getPayment(paymentId: string): Promise<ApiResponse<BenefitPayment>> {
    return this.request<BenefitPayment>(
      'GET',
      `/payments/${paymentId}`
    );
  }

  /**
   * List payments
   */
  async listPayments(
    params?: PaginationParams & { beneficiaryId?: string }
  ): Promise<ApiResponse<BenefitPayment[]>> {
    const queryString = params ? new URLSearchParams(params as any).toString() : '';
    return this.request<BenefitPayment[]>(
      'GET',
      `/payments${queryString ? '?' + queryString : ''}`
    );
  }

  /**
   * Process payment
   */
  async processPayment(
    payment: Partial<BenefitPayment>
  ): Promise<ApiResponse<BenefitPayment>> {
    return this.request<BenefitPayment>(
      'POST',
      '/payments',
      payment
    );
  }

  // ==========================================================================
  // Case Management
  // ==========================================================================

  /**
   * Get case record
   */
  async getCase(caseId: string): Promise<ApiResponse<CaseRecord>> {
    return this.request<CaseRecord>(
      'GET',
      `/cases/${caseId}`
    );
  }

  /**
   * Create case
   */
  async createCase(
    caseData: Partial<CaseRecord>
  ): Promise<ApiResponse<CaseRecord>> {
    return this.request<CaseRecord>(
      'POST',
      '/cases',
      caseData
    );
  }

  /**
   * Update case
   */
  async updateCase(
    caseId: string,
    updates: Partial<CaseRecord>
  ): Promise<ApiResponse<CaseRecord>> {
    return this.request<CaseRecord>(
      'PUT',
      `/cases/${caseId}`,
      updates
    );
  }

  /**
   * List cases
   */
  async listCases(
    params?: PaginationParams & { beneficiaryId?: string }
  ): Promise<ApiResponse<CaseRecord[]>> {
    const queryString = params ? new URLSearchParams(params as any).toString() : '';
    return this.request<CaseRecord[]>(
      'GET',
      `/cases${queryString ? '?' + queryString : ''}`
    );
  }

  /**
   * Close case
   */
  async closeCase(caseId: string): Promise<ApiResponse<CaseRecord>> {
    return this.request<CaseRecord>(
      'POST',
      `/cases/${caseId}/close`
    );
  }

  // ==========================================================================
  // Fraud Detection
  // ==========================================================================

  /**
   * Get fraud alert
   */
  async getFraudAlert(alertId: string): Promise<ApiResponse<FraudAlert>> {
    return this.request<FraudAlert>(
      'GET',
      `/fraud-alerts/${alertId}`
    );
  }

  /**
   * List fraud alerts
   */
  async listFraudAlerts(
    params?: PaginationParams & { severity?: string }
  ): Promise<ApiResponse<FraudAlert[]>> {
    const queryString = params ? new URLSearchParams(params as any).toString() : '';
    return this.request<FraudAlert[]>(
      'GET',
      `/fraud-alerts${queryString ? '?' + queryString : ''}`
    );
  }

  /**
   * Create fraud alert
   */
  async createFraudAlert(
    alert: Partial<FraudAlert>
  ): Promise<ApiResponse<FraudAlert>> {
    return this.request<FraudAlert>(
      'POST',
      '/fraud-alerts',
      alert
    );
  }

  /**
   * Resolve fraud alert
   */
  async resolveFraudAlert(
    alertId: string,
    resolution: any
  ): Promise<ApiResponse<FraudAlert>> {
    return this.request<FraudAlert>(
      'POST',
      `/fraud-alerts/${alertId}/resolve`,
      resolution
    );
  }

  // ==========================================================================
  // Eligibility Verification
  // ==========================================================================

  /**
   * Check eligibility for a program
   */
  async checkEligibility(
    beneficiaryId: string,
    programId: string
  ): Promise<ApiResponse<any>> {
    return this.request<any>(
      'POST',
      '/eligibility/check',
      { beneficiaryId, programId }
    );
  }

  /**
   * Verify income
   */
  async verifyIncome(
    beneficiaryId: string,
    incomeData: any
  ): Promise<ApiResponse<any>> {
    return this.request<any>(
      'POST',
      `/beneficiaries/${beneficiaryId}/verify-income`,
      incomeData
    );
  }

  // ==========================================================================
  // Reporting & Analytics
  // ==========================================================================

  /**
   * Get program statistics
   */
  async getProgramStatistics(programId: string): Promise<ApiResponse<any>> {
    return this.request<any>(
      'GET',
      `/programs/${programId}/statistics`
    );
  }

  /**
   * Generate report
   */
  async generateReport(reportType: string, params: any): Promise<ApiResponse<any>> {
    return this.request<any>(
      'POST',
      '/reports',
      { type: reportType, ...params }
    );
  }
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Calculate benefit amount
 */
export function calculateBenefit(
  grossIncome: number,
  householdSize: number,
  deductions: number,
  maximumBenefit: number,
  incomeLimit: number
): number {
  const netIncome = Math.max(0, grossIncome - deductions);
  const incomePercentage = netIncome / incomeLimit;
  const reductionFactor = 0.3 * incomePercentage;
  const benefitAmount = maximumBenefit * (1 - reductionFactor);
  return Math.max(0, Math.round(benefitAmount));
}

/**
 * Validate SSN format
 */
export function validateSSN(ssn: string): boolean {
  return /^\d{3}-\d{2}-\d{4}$/.test(ssn);
}

/**
 * Validate email format
 */
export function validateEmail(email: string): boolean {
  return /^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(email);
}

/**
 * Validate phone number
 */
export function validatePhone(phone: string): boolean {
  return /^\+?1?\d{10,}$/.test(phone);
}

/**
 * Calculate age from date of birth
 */
export function calculateAge(dateOfBirth: string): number {
  const dob = new Date(dateOfBirth);
  const today = new Date();
  let age = today.getFullYear() - dob.getFullYear();
  const monthDiff = today.getMonth() - dob.getMonth();

  if (monthDiff < 0 || (monthDiff === 0 && today.getDate() < dob.getDate())) {
    age--;
  }

  return age;
}

/**
 * Format currency
 */
export function formatCurrency(amount: number): string {
  return new Intl.NumberFormat('en-US', {
    style: 'currency',
    currency: 'USD',
  }).format(amount);
}

/**
 * Format date
 */
export function formatDate(date: string | Date): string {
  return new Intl.DateTimeFormat('en-US').format(new Date(date));
}
