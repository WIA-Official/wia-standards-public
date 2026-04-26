/**
 * WIA-FIN-024 ESG Finance Standard SDK
 *
 * @version 2.0.0
 * @license MIT
 */

import axios, { AxiosInstance } from 'axios';
import * as crypto from 'crypto-js';
import {
  ESGScore,
  GreenBond,
  SustainabilityReport,
  CalculateESGScoreRequest,
  CalculateESGScoreResponse,
  IssueGreenBondRequest,
  IssueGreenBondResponse,
  GenerateReportRequest,
  GenerateReportResponse,
  CompanyId,
  BondId,
  ReportId,
  ReportingFramework
} from './types';

export * from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface ESGFinanceConfig {
  apiKey: string;
  environment?: 'sandbox' | 'production';
  baseUrl?: string;
  timeout?: number;
}

const DEFAULT_CONFIG = {
  environment: 'production' as const,
  baseUrl: 'https://api.wia.org/v1/esg',
  timeout: 30000
};

// ============================================================================
// Main SDK Class
// ============================================================================

export class ESGFinanceSDK {
  private config: Required<ESGFinanceConfig>;
  private client: AxiosInstance;

  constructor(config: ESGFinanceConfig) {
    this.config = {
      ...DEFAULT_CONFIG,
      ...config,
      baseUrl: config.baseUrl || DEFAULT_CONFIG.baseUrl
    };

    if (this.config.environment === 'sandbox') {
      this.config.baseUrl = 'https://sandbox-api.wia.org/v1/esg';
    }

    this.client = axios.create({
      baseURL: this.config.baseUrl,
      timeout: this.config.timeout,
      headers: {
        'Authorization': `Bearer ${this.config.apiKey}`,
        'Content-Type': 'application/json',
        'X-WIA-Standard': 'FIN-024',
        'X-WIA-Version': '2.0.0'
      }
    });
  }

  // ==========================================================================
  // ESG Scoring
  // ==========================================================================

  /**
   * Calculate ESG score for a company
   */
  async calculateESGScore(
    request: CalculateESGScoreRequest
  ): Promise<CalculateESGScoreResponse> {
    try {
      const response = await this.client.post<CalculateESGScoreResponse>(
        '/score',
        request
      );
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Get historical ESG scores
   */
  async getESGScoreHistory(
    companyId: CompanyId,
    startPeriod?: string,
    endPeriod?: string
  ): Promise<ESGScore[]> {
    try {
      const response = await this.client.get<ESGScore[]>(
        `/score/${companyId}/history`,
        { params: { startPeriod, endPeriod } }
      );
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Get industry benchmark scores
   */
  async getBenchmarkScores(
    industry: string,
    region?: string
  ): Promise<{ average: number; median: number; top10: number }> {
    try {
      const response = await this.client.get(
        `/benchmark/${industry}`,
        { params: { region } }
      );
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  // ==========================================================================
  // Green Finance
  // ==========================================================================

  /**
   * Issue a green bond
   */
  async issueGreenBond(
    request: IssueGreenBondRequest
  ): Promise<IssueGreenBondResponse> {
    try {
      const response = await this.client.post<IssueGreenBondResponse>(
        '/bonds/green',
        request
      );
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Get green bond details
   */
  async getGreenBond(bondId: BondId): Promise<GreenBond> {
    try {
      const response = await this.client.get<GreenBond>(
        `/bonds/green/${bondId}`
      );
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Track impact metrics for green bond
   */
  async trackImpact(
    bondId: BondId,
    period: string,
    metrics: {
      co2Avoided?: number;
      renewableEnergyGenerated?: number;
      waterSaved?: number;
      wasteDiverted?: number;
      beneficiariesCount?: number;
    }
  ): Promise<void> {
    try {
      await this.client.post(
        `/bonds/green/${bondId}/impact`,
        { period, metrics }
      );
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * List all green bonds for a company
   */
  async listGreenBonds(
    issuer: CompanyId,
    active?: boolean
  ): Promise<GreenBond[]> {
    try {
      const response = await this.client.get<GreenBond[]>(
        `/bonds/green`,
        { params: { issuer, active } }
      );
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  // ==========================================================================
  // Compliance & Reporting
  // ==========================================================================

  /**
   * Check compliance status
   */
  async checkCompliance(
    companyId: CompanyId,
    frameworks?: ReportingFramework[]
  ): Promise<{
    compliant: boolean;
    frameworks: Array<{
      framework: ReportingFramework;
      compliant: boolean;
      gaps: string[];
    }>;
  }> {
    try {
      const response = await this.client.get(
        `/compliance/${companyId}`,
        { params: { frameworks: frameworks?.join(',') } }
      );
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Generate sustainability report
   */
  async generateReport(
    request: GenerateReportRequest
  ): Promise<GenerateReportResponse> {
    try {
      const response = await this.client.post<GenerateReportResponse>(
        '/reports',
        request
      );
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Get report status
   */
  async getReportStatus(reportId: ReportId): Promise<{
    status: 'processing' | 'completed' | 'failed';
    progress: number;
    downloadUrl?: string;
  }> {
    try {
      const response = await this.client.get(`/reports/${reportId}/status`);
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Submit ESG data
   */
  async submitData(
    companyId: CompanyId,
    period: string,
    data: {
      environmental?: any;
      social?: any;
      governance?: any;
    }
  ): Promise<{ success: boolean; dataId: string }> {
    try {
      const response = await this.client.post(
        `/data/${companyId}`,
        { period, ...data }
      );
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  // ==========================================================================
  // Webhooks
  // ==========================================================================

  webhooks = {
    /**
     * Create a webhook subscription
     */
    create: async (config: {
      url: string;
      events: string[];
      secret?: string;
    }): Promise<{ webhookId: string; secret: string }> => {
      try {
        const response = await this.client.post('/webhooks', config);
        return response.data;
      } catch (error) {
        throw this.handleError(error);
      }
    },

    /**
     * Verify webhook signature
     */
    verify: (payload: any, signature: string, secret: string): boolean => {
      const computedSignature = crypto.HmacSHA256(
        JSON.stringify(payload),
        secret
      ).toString();
      return computedSignature === signature;
    },

    /**
     * Delete webhook
     */
    delete: async (webhookId: string): Promise<void> => {
      try {
        await this.client.delete(`/webhooks/${webhookId}`);
      } catch (error) {
        throw this.handleError(error);
      }
    }
  };

  // ==========================================================================
  // Carbon Accounting Helpers
  // ==========================================================================

  carbon = {
    /**
     * Calculate carbon intensity
     */
    calculateIntensity: (
      totalEmissions: number,
      revenue: number
    ): number => {
      return totalEmissions / revenue;
    },

    /**
     * Convert between emission units
     */
    convert: (
      value: number,
      from: 'kg' | 'tons' | 'kWh',
      to: 'kg' | 'tons' | 'tCO2e'
    ): number => {
      const conversions: Record<string, number> = {
        'kg->tons': 0.001,
        'tons->kg': 1000,
        'kWh->tCO2e': 0.000475 // US grid average
      };
      const key = `${from}->${to}`;
      return value * (conversions[key] || 1);
    },

    /**
     * Calculate emissions reduction percentage
     */
    reductionPercentage: (
      baseline: number,
      current: number
    ): number => {
      return ((baseline - current) / baseline) * 100;
    }
  };

  // ==========================================================================
  // Utility Methods
  // ==========================================================================

  /**
   * Validate ESG data completeness
   */
  async validateData(
    companyId: CompanyId,
    period: string
  ): Promise<{
    complete: boolean;
    missingFields: string[];
    warnings: string[];
  }> {
    try {
      const response = await this.client.post(`/validate`, {
        companyId,
        period
      });
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Get ESG rating explanation
   */
  async explainRating(
    companyId: CompanyId,
    period: string
  ): Promise<{
    rating: string;
    strengths: string[];
    weaknesses: string[];
    improvements: string[];
  }> {
    try {
      const response = await this.client.get(
        `/score/${companyId}/${period}/explain`
      );
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  // ==========================================================================
  // Error Handling
  // ==========================================================================

  private handleError(error: any): Error {
    if (axios.isAxiosError(error)) {
      const message = error.response?.data?.message || error.message;
      const status = error.response?.status;
      return new Error(`ESG Finance API Error (${status}): ${message}`);
    }
    return error;
  }
}

// ============================================================================
// Standalone Helper Functions
// ============================================================================

/**
 * Calculate overall ESG score from component scores
 */
export function calculateOverallScore(
  environmentalScore: number,
  socialScore: number,
  governanceScore: number
): number {
  return environmentalScore * 0.35 + socialScore * 0.30 + governanceScore * 0.35;
}

/**
 * Determine ESG rating from score
 */
export function scoreToRating(score: number): string {
  if (score >= 90) return 'AAA';
  if (score >= 80) return 'AA';
  if (score >= 70) return 'A';
  if (score >= 60) return 'BBB';
  if (score >= 50) return 'BB';
  return 'B';
}

/**
 * Validate green bond use of proceeds totals to 100%
 */
export function validateUseOfProceeds(useOfProceeds: Record<string, number>): boolean {
  const total = Object.values(useOfProceeds).reduce((sum, val) => sum + val, 0);
  return Math.abs(total - 1.0) < 0.001; // Allow for floating point rounding
}

export default ESGFinanceSDK;
