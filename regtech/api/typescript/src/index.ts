/**
 * WIA-FIN-004 RegTech Standard - TypeScript SDK
 * @module @wia/regtech
 *
 * Regulatory Technology and Compliance Automation
 * 弘益人間 · Benefit All Humanity
 */

import axios, { AxiosInstance } from 'axios';
import * as types from './types';

export * from './types';

/**
 * Main RegTech client for compliance operations
 */
export class RegTechClient {
  private api: AxiosInstance;
  private config: types.RegTechClientConfig;

  /**
   * Creates a new RegTech client instance
   * @param config - Client configuration
   */
  constructor(config: types.RegTechClientConfig) {
    this.config = {
      baseUrl: 'https://api.wia.org/v1/regtech',
      network: 'mainnet',
      timeout: 30000,
      debug: false,
      ...config,
    };

    this.api = axios.create({
      baseURL: this.config.baseUrl,
      timeout: this.config.timeout,
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${this.config.apiKey}`,
        'X-WIA-Network': this.config.network,
      },
    });

    // Add request interceptor for debugging
    if (this.config.debug) {
      this.api.interceptors.request.use((config) => {
        console.log('[RegTech SDK] Request:', config.method?.toUpperCase(), config.url);
        return config;
      });

      this.api.interceptors.response.use(
        (response) => {
          console.log('[RegTech SDK] Response:', response.status, response.data);
          return response;
        },
        (error) => {
          console.error('[RegTech SDK] Error:', error.response?.status, error.response?.data);
          return Promise.reject(error);
        }
      );
    }
  }

  /**
   * Check compliance for a transaction
   * @param request - Compliance check request
   * @returns Compliance check response
   */
  async checkCompliance(
    request: types.ComplianceCheckRequest
  ): Promise<types.ComplianceCheckResponse> {
    try {
      const response = await this.api.post<types.ComplianceCheckResponse>(
        '/compliance/check',
        request
      );
      return response.data;
    } catch (error: any) {
      throw this.handleError(error);
    }
  }

  /**
   * Verify customer KYC documents
   * @param request - KYC verification request
   * @returns KYC verification response
   */
  async verifyKYC(
    request: types.KYCVerificationRequest
  ): Promise<types.KYCVerificationResponse> {
    try {
      const response = await this.api.post<types.KYCVerificationResponse>(
        '/kyc/verify',
        request
      );
      return response.data;
    } catch (error: any) {
      throw this.handleError(error);
    }
  }

  /**
   * Screen entity against sanctions lists
   * @param request - Sanctions screening request
   * @returns Sanctions screening response
   */
  async screenSanctions(
    request: types.SanctionsScreeningRequest
  ): Promise<types.SanctionsScreeningResponse> {
    try {
      const response = await this.api.post<types.SanctionsScreeningResponse>(
        '/sanctions/screen',
        request
      );
      return response.data;
    } catch (error: any) {
      throw this.handleError(error);
    }
  }

  /**
   * Submit regulatory report
   * @param report - Regulatory report
   * @returns Report submission response
   */
  async submitReport(
    report: types.RegulatoryReport
  ): Promise<types.ReportSubmissionResponse> {
    try {
      const response = await this.api.post<types.ReportSubmissionResponse>(
        `/reports/${report.reportType}`,
        report
      );
      return response.data;
    } catch (error: any) {
      throw this.handleError(error);
    }
  }

  /**
   * Get customer information
   * @param customerId - Customer identifier
   * @returns Customer data
   */
  async getCustomer(customerId: string): Promise<types.Customer> {
    try {
      const response = await this.api.get<types.Customer>(
        `/customers/${customerId}`
      );
      return response.data;
    } catch (error: any) {
      throw this.handleError(error);
    }
  }

  /**
   * Get transaction information
   * @param transactionId - Transaction identifier
   * @returns Transaction data
   */
  async getTransaction(transactionId: string): Promise<types.Transaction> {
    try {
      const response = await this.api.get<types.Transaction>(
        `/transactions/${transactionId}`
      );
      return response.data;
    } catch (error: any) {
      throw this.handleError(error);
    }
  }

  /**
   * Get compliance events
   * @param params - Query parameters
   * @returns Array of compliance events
   */
  async getComplianceEvents(params: {
    startDate: string;
    endDate: string;
    jurisdiction?: types.Jurisdiction;
    eventType?: types.EventType;
    limit?: number;
  }): Promise<types.ComplianceEvent[]> {
    try {
      const response = await this.api.get<types.ComplianceEvent[]>(
        '/compliance/events',
        { params }
      );
      return response.data;
    } catch (error: any) {
      throw this.handleError(error);
    }
  }

  /**
   * Update customer risk profile
   * @param customerId - Customer identifier
   * @param riskProfile - Updated risk profile
   * @returns Updated customer data
   */
  async updateCustomerRisk(
    customerId: string,
    riskProfile: Partial<types.CustomerRiskProfile>
  ): Promise<types.Customer> {
    try {
      const response = await this.api.patch<types.Customer>(
        `/customers/${customerId}/risk`,
        riskProfile
      );
      return response.data;
    } catch (error: any) {
      throw this.handleError(error);
    }
  }

  /**
   * Generate compliance report
   * @param reportType - Type of report to generate
   * @param params - Report parameters
   * @returns Generated report
   */
  async generateReport(
    reportType: types.ReportType,
    params: {
      startDate: string;
      endDate: string;
      jurisdiction: types.Jurisdiction;
      format?: 'json' | 'xml' | 'pdf';
    }
  ): Promise<types.RegulatoryReport> {
    try {
      const response = await this.api.post<types.RegulatoryReport>(
        '/reports/generate',
        {
          reportType,
          ...params,
        }
      );
      return response.data;
    } catch (error: any) {
      throw this.handleError(error);
    }
  }

  /**
   * Handle API errors
   * @param error - Axios error
   * @returns Formatted API error
   */
  private handleError(error: any): types.APIError {
    if (error.response) {
      return {
        code: error.response.data?.code || 'API_ERROR',
        message: error.response.data?.message || error.message,
        details: error.response.data?.details,
        statusCode: error.response.status,
      };
    } else if (error.request) {
      return {
        code: 'NETWORK_ERROR',
        message: 'No response received from server',
        statusCode: 0,
      };
    } else {
      return {
        code: 'CLIENT_ERROR',
        message: error.message,
        statusCode: 0,
      };
    }
  }
}

/**
 * Helper function to create a RegTech client
 * @param config - Client configuration
 * @returns RegTech client instance
 */
export function createRegTechClient(
  config: types.RegTechClientConfig
): RegTechClient {
  return new RegTechClient(config);
}

/**
 * Risk score calculator utility
 */
export class RiskScoreCalculator {
  /**
   * Calculate risk score for a transaction
   * @param transaction - Transaction data
   * @param customer - Customer data
   * @returns Calculated risk score (0-100)
   */
  static calculateTransactionRisk(
    transaction: types.Transaction,
    customer: types.Customer
  ): number {
    let score = 0;

    // Customer risk (0-30 points)
    if (customer.riskProfile.riskLevel === 'high') score += 30;
    else if (customer.riskProfile.riskLevel === 'medium') score += 15;
    else if (customer.riskProfile.riskLevel === 'low') score += 5;

    if (customer.riskProfile.isPEP) score += 20;

    // Amount risk (0-25 points)
    if (transaction.amount > 100000) score += 25;
    else if (transaction.amount > 50000) score += 15;
    else if (transaction.amount > 10000) score += 5;

    // Geographic risk (0-25 points)
    const highRiskCountries = ['AF', 'KP', 'IR', 'SY', 'YE'];
    if (highRiskCountries.includes(transaction.parties.beneficiary.country)) {
      score += 25;
    }

    // Sanctions/adverse media (0-20 points)
    if (customer.riskProfile.sanctionsMatch) score += 20;
    if (customer.riskProfile.adverseMedia && customer.riskProfile.adverseMedia.length > 0) {
      score += 10;
    }

    return Math.min(score, 100);
  }

  /**
   * Determine if SAR filing is required
   * @param riskScore - Risk score
   * @param transaction - Transaction data
   * @returns Whether SAR is required
   */
  static requiresSAR(
    riskScore: number,
    transaction: types.Transaction
  ): boolean {
    return riskScore > 70 || transaction.compliance.alerts.some(
      alert => alert.severity === 'critical'
    );
  }

  /**
   * Determine if CTR filing is required
   * @param transaction - Transaction data
   * @param jurisdiction - Jurisdiction
   * @returns Whether CTR is required
   */
  static requiresCTR(
    transaction: types.Transaction,
    jurisdiction: types.Jurisdiction = 'US'
  ): boolean {
    const thresholds: Record<string, number> = {
      'US': 10000,
      'EU': 10000,
      'UK': 10000,
      'AU': 10000,
    };

    const threshold = thresholds[jurisdiction] || 10000;
    return transaction.amount >= threshold && transaction.currency === 'USD';
  }
}

/**
 * Compliance rules engine
 */
export class ComplianceRules {
  private rules: Map<string, (data: any) => boolean>;

  constructor() {
    this.rules = new Map();
    this.initializeDefaultRules();
  }

  /**
   * Initialize default compliance rules
   */
  private initializeDefaultRules(): void {
    // Structuring detection
    this.addRule('structuring', (transactions: types.Transaction[]) => {
      const last24h = transactions.filter(
        tx => new Date(tx.timestamp).getTime() > Date.now() - 24 * 60 * 60 * 1000
      );
      const total = last24h.reduce((sum, tx) => sum + tx.amount, 0);
      return total >= 10000 && last24h.every(tx => tx.amount < 10000) && last24h.length >= 3;
    });

    // Large transaction
    this.addRule('large_transaction', (transaction: types.Transaction) => {
      return transaction.amount >= 50000;
    });

    // High-risk country
    this.addRule('high_risk_country', (transaction: types.Transaction) => {
      const highRiskCountries = ['AF', 'KP', 'IR', 'SY', 'YE'];
      return highRiskCountries.includes(transaction.parties.beneficiary.country);
    });
  }

  /**
   * Add a custom compliance rule
   * @param name - Rule name
   * @param rule - Rule function
   */
  addRule(name: string, rule: (data: any) => boolean): void {
    this.rules.set(name, rule);
  }

  /**
   * Evaluate a rule
   * @param name - Rule name
   * @param data - Data to evaluate
   * @returns Rule evaluation result
   */
  evaluate(name: string, data: any): boolean {
    const rule = this.rules.get(name);
    if (!rule) {
      throw new Error(`Rule '${name}' not found`);
    }
    return rule(data);
  }

  /**
   * Evaluate all rules
   * @param data - Data to evaluate
   * @returns Map of rule results
   */
  evaluateAll(data: any): Map<string, boolean> {
    const results = new Map<string, boolean>();
    this.rules.forEach((rule, name) => {
      try {
        results.set(name, rule(data));
      } catch (error) {
        results.set(name, false);
      }
    });
    return results;
  }
}

/**
 * Export default client
 */
export default RegTechClient;
