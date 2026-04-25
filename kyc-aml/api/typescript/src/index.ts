/**
 * WIA-FIN-011 KYC/AML TypeScript SDK
 * @version 1.0.0
 */

import axios, { AxiosInstance, AxiosRequestConfig } from 'axios';
import * as types from './types';

export * from './types';

/**
 * Main KYC/AML Client
 */
export class KYCAMLClient {
  private client: AxiosInstance;
  private config: types.APIConfig;

  constructor(config: types.APIConfig) {
    this.config = config;
    
    this.client = axios.create({
      baseURL: config.baseUrl,
      timeout: config.timeout || 30000,
      headers: {
        'Content-Type': 'application/json',
        ...(config.apiKey && { 'X-API-Key': config.apiKey }),
        ...(config.accessToken && { 'Authorization': `Bearer ${config.accessToken}` }),
      },
    });

    // Request interceptor
    this.client.interceptors.request.use(
      (config) => {
        config.headers['X-Request-ID'] = this.generateUUID();
        return config;
      },
      (error) => Promise.reject(error)
    );

    // Response interceptor
    this.client.interceptors.response.use(
      (response) => response,
      async (error) => {
        if (error.response?.status === 401) {
          // Token refresh logic here
        }
        return Promise.reject(error);
      }
    );
  }

  // ==================== Customer Management ====================

  /**
   * Create a new customer
   */
  async createCustomer(
    customer: Partial<types.Person | types.LegalEntity>
  ): Promise<types.APIResponse<{ customerId: string; status: string }>> {
    const response = await this.client.post('/api/v1/customers', customer);
    return response.data;
  }

  /**
   * Get customer by ID
   */
  async getCustomer(customerId: string): Promise<types.APIResponse<types.Person | types.LegalEntity>> {
    const response = await this.client.get(`/api/v1/customers/${customerId}`);
    return response.data;
  }

  /**
   * Update customer information
   */
  async updateCustomer(
    customerId: string,
    updates: Partial<types.Person | types.LegalEntity>
  ): Promise<types.APIResponse<types.Person | types.LegalEntity>> {
    const response = await this.client.patch(`/api/v1/customers/${customerId}`, updates);
    return response.data;
  }

  /**
   * Search customers
   */
  async searchCustomers(
    query: {
      name?: string;
      dateOfBirth?: string;
      country?: types.CountryCode;
      email?: string;
      page?: number;
      limit?: number;
    }
  ): Promise<types.PaginatedResponse<types.Person | types.LegalEntity>> {
    const response = await this.client.get('/api/v1/customers/search', { params: query });
    return response.data;
  }

  // ==================== Customer Due Diligence ====================

  /**
   * Initiate CDD process
   */
  async initiateCDD(
    customerId: string,
    cddType: types.CDDType = 'standard',
    triggerReason: string
  ): Promise<types.APIResponse<{ cddId: string; status: string }>> {
    const response = await this.client.post('/api/v1/cdd', {
      customerId,
      cddType,
      triggerReason,
    });
    return response.data;
  }

  /**
   * Get CDD status
   */
  async getCDDStatus(cddId: string): Promise<types.APIResponse<types.CDDRecord>> {
    const response = await this.client.get(`/api/v1/cdd/${cddId}`);
    return response.data;
  }

  /**
   * Get all CDD records for a customer
   */
  async getCustomerCDDRecords(customerId: string): Promise<types.APIResponse<types.CDDRecord[]>> {
    const response = await this.client.get(`/api/v1/customers/${customerId}/cdd`);
    return response.data;
  }

  // ==================== Document Verification ====================

  /**
   * Upload document for verification
   */
  async uploadDocument(
    customerId: string,
    documentType: types.DocumentType,
    file: File | Buffer,
    metadata?: Record<string, any>
  ): Promise<types.APIResponse<{ documentId: string; status: string }>> {
    const formData = new FormData();
    formData.append('file', file);
    formData.append('documentType', documentType);
    formData.append('customerId', customerId);
    if (metadata) {
      formData.append('metadata', JSON.stringify(metadata));
    }

    const response = await this.client.post('/api/v1/documents', formData, {
      headers: { 'Content-Type': 'multipart/form-data' },
    });
    return response.data;
  }

  /**
   * Verify uploaded document
   */
  async verifyDocument(
    documentId: string
  ): Promise<types.APIResponse<{
    documentId: string;
    verificationStatus: types.VerificationStatus;
    extractedData: Record<string, any>;
    validationChecks: Record<string, string>;
    confidenceScore: number;
  }>> {
    const response = await this.client.post(`/api/v1/documents/${documentId}/verify`);
    return response.data;
  }

  // ==================== Screening ====================

  /**
   * Perform sanctions screening
   */
  async sanctionsScreening(
    data: {
      searchType: 'individual' | 'entity';
      firstName?: string;
      lastName?: string;
      fullName?: string;
      dateOfBirth?: string;
      nationality?: types.CountryCode;
      entityName?: string;
    }
  ): Promise<types.APIResponse<{
    screeningId: string;
    searchDate: Date;
    totalMatches: number;
    clearanceStatus: 'clear' | 'hit' | 'under_review';
    listsSearched: string[];
    matches?: types.ScreeningMatch[];
  }>> {
    const response = await this.client.post('/api/v1/screening/sanctions', data);
    return response.data;
  }

  /**
   * Perform PEP screening
   */
  async pepScreening(
    data: {
      fullName: string;
      dateOfBirth?: string;
      country?: types.CountryCode;
    }
  ): Promise<types.APIResponse<{
    screeningId: string;
    pepStatus: 'not_pep' | 'pep' | 'former_pep';
    matches: any[];
    screeningDate: Date;
  }>> {
    const response = await this.client.post('/api/v1/screening/pep', data);
    return response.data;
  }

  /**
   * Perform adverse media screening
   */
  async adverseMediaScreening(
    customerId: string
  ): Promise<types.APIResponse<{
    screeningId: string;
    articlesFound: number;
    relevantArticles: any[];
    riskIndicator: 'none' | 'low' | 'medium' | 'high';
  }>> {
    const response = await this.client.post('/api/v1/screening/adverse-media', { customerId });
    return response.data;
  }

  // ==================== Risk Assessment ====================

  /**
   * Calculate risk score for a customer
   */
  async calculateRiskScore(
    customerId: string,
    riskFactors: Record<string, any>
  ): Promise<types.APIResponse<types.RiskAssessment>> {
    const response = await this.client.post('/api/v1/risk/assess', {
      customerId,
      riskFactors,
    });
    return response.data;
  }

  /**
   * Get customer risk assessment
   */
  async getCustomerRisk(customerId: string): Promise<types.APIResponse<types.RiskAssessment>> {
    const response = await this.client.get(`/api/v1/customers/${customerId}/risk`);
    return response.data;
  }

  // ==================== Transaction Monitoring ====================

  /**
   * Submit transaction for monitoring
   */
  async submitTransaction(
    transaction: Partial<types.Transaction>
  ): Promise<types.APIResponse<{
    transactionId: string;
    monitoringStatus: string;
    alerts: types.Alert[];
    clearanceStatus: 'approved' | 'pending' | 'blocked';
  }>> {
    const response = await this.client.post('/api/v1/transactions', transaction);
    return response.data;
  }

  /**
   * Get transaction details
   */
  async getTransaction(transactionId: string): Promise<types.APIResponse<types.Transaction>> {
    const response = await this.client.get(`/api/v1/transactions/${transactionId}`);
    return response.data;
  }

  /**
   * Get transaction alerts
   */
  async getTransactionAlerts(
    transactionId: string
  ): Promise<types.APIResponse<{ transactionId: string; alerts: types.Alert[] }>> {
    const response = await this.client.get(`/api/v1/transactions/${transactionId}/alerts`);
    return response.data;
  }

  /**
   * Search transactions
   */
  async searchTransactions(
    query: {
      customerId?: string;
      startDate?: string;
      endDate?: string;
      minAmount?: number;
      maxAmount?: number;
      transactionType?: types.TransactionType;
      page?: number;
      limit?: number;
    }
  ): Promise<types.PaginatedResponse<types.Transaction>> {
    const response = await this.client.get('/api/v1/transactions/search', { params: query });
    return response.data;
  }

  // ==================== SAR Management ====================

  /**
   * Create Suspicious Activity Report
   */
  async createSAR(
    sar: Partial<types.SuspiciousActivityReport>
  ): Promise<types.APIResponse<{ sarId: string; status: 'draft' | 'pending_approval' }>> {
    const response = await this.client.post('/api/v1/sar', sar);
    return response.data;
  }

  /**
   * Get SAR by ID
   */
  async getSAR(sarId: string): Promise<types.APIResponse<types.SuspiciousActivityReport>> {
    const response = await this.client.get(`/api/v1/sar/${sarId}`);
    return response.data;
  }

  /**
   * Update SAR
   */
  async updateSAR(
    sarId: string,
    updates: Partial<types.SuspiciousActivityReport>
  ): Promise<types.APIResponse<types.SuspiciousActivityReport>> {
    const response = await this.client.patch(`/api/v1/sar/${sarId}`, updates);
    return response.data;
  }

  /**
   * Submit SAR to regulatory authority
   */
  async submitSAR(
    sarId: string
  ): Promise<types.APIResponse<{ sarId: string; filingId: string; status: 'filed'; filedAt: Date }>> {
    const response = await this.client.post(`/api/v1/sar/${sarId}/submit`);
    return response.data;
  }

  // ==================== Utility Methods ====================

  /**
   * Generate UUID v4
   */
  private generateUUID(): string {
    return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => {
      const r = (Math.random() * 16) | 0;
      const v = c === 'x' ? r : (r & 0x3) | 0x8;
      return v.toString(16);
    });
  }

  /**
   * Set access token
   */
  setAccessToken(token: string): void {
    this.config.accessToken = token;
    this.client.defaults.headers.common['Authorization'] = `Bearer ${token}`;
  }

  /**
   * Set API key
   */
  setAPIKey(apiKey: string): void {
    this.config.apiKey = apiKey;
    this.client.defaults.headers.common['X-API-Key'] = apiKey;
  }
}

/**
 * Helper functions
 */

/**
 * Validate email format
 */
export function validateEmail(email: string): boolean {
  const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
  return emailRegex.test(email);
}

/**
 * Validate phone number (E.164 format)
 */
export function validatePhoneNumber(phone: string): boolean {
  const phoneRegex = /^\+[1-9]\d{1,14}$/;
  return phoneRegex.test(phone);
}

/**
 * Calculate age from date of birth
 */
export function calculateAge(dateOfBirth: Date): number {
  const today = new Date();
  const birthDate = new Date(dateOfBirth);
  let age = today.getFullYear() - birthDate.getFullYear();
  const monthDiff = today.getMonth() - birthDate.getMonth();
  
  if (monthDiff < 0 || (monthDiff === 0 && today.getDate() < birthDate.getDate())) {
    age--;
  }
  
  return age;
}

/**
 * Format currency
 */
export function formatCurrency(amount: number, currency: types.CurrencyCode): string {
  return new Intl.NumberFormat('en-US', {
    style: 'currency',
    currency: currency,
  }).format(amount);
}

/**
 * Validate risk score (0-100)
 */
export function validateRiskScore(score: number): boolean {
  return score >= 0 && score <= 100;
}

/**
 * Get risk rating from score
 */
export function getRiskRatingFromScore(score: number): types.RiskRating {
  if (score < 30) return 'low';
  if (score < 70) return 'medium';
  return 'high';
}

/**
 * Default export
 */
export default KYCAMLClient;
