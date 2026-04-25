/**
 * WIA-FIN-008 Asset Tokenization Standard
 * TypeScript SDK
 *
 * @version 1.0.0
 * @license MIT
 */

import axios, { AxiosInstance } from 'axios';
import {
  APIConfig,
  APIResponse,
  TokenMetadata,
  Transfer,
  ComplianceCheck,
  Distribution,
  InvestorRecord,
  ValuationInfo,
} from './types';

export * from './types';

export class WIATokenization {
  private client: AxiosInstance;
  private config: APIConfig;

  constructor(config: APIConfig) {
    this.config = {
      baseUrl: 'https://api.wia-fin-008.com',
      environment: 'production',
      timeout: 30000,
      ...config,
    };

    this.client = axios.create({
      baseURL: this.config.baseUrl,
      timeout: this.config.timeout,
      headers: {
        'Authorization': `Bearer ${this.config.apiKey}`,
        'Content-Type': 'application/json',
        'X-WIA-SDK-Version': '1.0.0',
        'X-WIA-Environment': this.config.environment,
      },
    });
  }

  /**
   * Token Service
   */
  public tokens = {
    /**
     * Create a new security token
     */
    create: async (data: Partial<TokenMetadata>): Promise<TokenMetadata> => {
      const response = await this.client.post<APIResponse<TokenMetadata>>(
        '/api/v1/tokens',
        data
      );
      return response.data.data;
    },

    /**
     * Get token details by ID
     */
    get: async (tokenId: string): Promise<TokenMetadata> => {
      const response = await this.client.get<APIResponse<TokenMetadata>>(
        `/api/v1/tokens/${tokenId}`
      );
      return response.data.data;
    },

    /**
     * List all tokens
     */
    list: async (params?: {
      assetClass?: string;
      status?: string;
      page?: number;
      limit?: number;
    }): Promise<{ data: TokenMetadata[]; pagination: any }> => {
      const response = await this.client.get<
        APIResponse<{ data: TokenMetadata[]; pagination: any }>
      >('/api/v1/tokens', { params });
      return response.data.data;
    },

    /**
     * Update token metadata
     */
    update: async (
      tokenId: string,
      data: Partial<TokenMetadata>
    ): Promise<TokenMetadata> => {
      const response = await this.client.put<APIResponse<TokenMetadata>>(
        `/api/v1/tokens/${tokenId}`,
        data
      );
      return response.data.data;
    },
  };

  /**
   * Transfer Service
   */
  public transfers = {
    /**
     * Execute a token transfer
     */
    create: async (data: {
      tokenId: string;
      from: string;
      to: string;
      amount: number;
      memo?: string;
    }): Promise<Transfer> => {
      const response = await this.client.post<APIResponse<Transfer>>(
        '/api/v1/transfers',
        data
      );
      return response.data.data;
    },

    /**
     * Get transfer status
     */
    get: async (transferId: string): Promise<Transfer> => {
      const response = await this.client.get<APIResponse<Transfer>>(
        `/api/v1/transfers/${transferId}`
      );
      return response.data.data;
    },

    /**
     * List token transfers
     */
    list: async (
      tokenId: string,
      params?: { page?: number; limit?: number }
    ): Promise<{ data: Transfer[]; pagination: any }> => {
      const response = await this.client.get<
        APIResponse<{ data: Transfer[]; pagination: any }>
      >(`/api/v1/tokens/${tokenId}/transfers`, { params });
      return response.data.data;
    },
  };

  /**
   * Compliance Service
   */
  public compliance = {
    /**
     * Check if a transfer is compliant
     */
    check: async (data: {
      tokenId: string;
      from: string;
      to: string;
      amount: number;
    }): Promise<ComplianceCheck> => {
      const response = await this.client.post<APIResponse<ComplianceCheck>>(
        '/api/v1/compliance/check',
        data
      );
      return response.data.data;
    },
  };

  /**
   * Investor Service
   */
  public investors = {
    /**
     * Onboard a new investor
     */
    create: async (data: Partial<InvestorRecord>): Promise<InvestorRecord> => {
      const response = await this.client.post<APIResponse<InvestorRecord>>(
        '/api/v1/investors',
        data
      );
      return response.data.data;
    },

    /**
     * Get investor details
     */
    get: async (investorId: string): Promise<InvestorRecord> => {
      const response = await this.client.get<APIResponse<InvestorRecord>>(
        `/api/v1/investors/${investorId}`
      );
      return response.data.data;
    },

    /**
     * Get investor KYC status
     */
    getKYC: async (investorId: string): Promise<any> => {
      const response = await this.client.get<APIResponse<any>>(
        `/api/v1/investors/${investorId}/kyc`
      );
      return response.data.data;
    },

    /**
     * Verify investor (submit KYC documents)
     */
    verify: async (investorId: string, data: any): Promise<any> => {
      const response = await this.client.post<APIResponse<any>>(
        `/api/v1/investors/${investorId}/verify`,
        data
      );
      return response.data.data;
    },
  };

  /**
   * Distribution Service
   */
  public distributions = {
    /**
     * Schedule a new distribution
     */
    create: async (data: Partial<Distribution>): Promise<Distribution> => {
      const response = await this.client.post<APIResponse<Distribution>>(
        '/api/v1/distributions',
        data
      );
      return response.data.data;
    },

    /**
     * Get distribution details
     */
    get: async (distributionId: string): Promise<Distribution> => {
      const response = await this.client.get<APIResponse<Distribution>>(
        `/api/v1/distributions/${distributionId}`
      );
      return response.data.data;
    },

    /**
     * Execute a scheduled distribution
     */
    execute: async (distributionId: string): Promise<Distribution> => {
      const response = await this.client.post<APIResponse<Distribution>>(
        `/api/v1/distributions/${distributionId}/execute`
      );
      return response.data.data;
    },
  };

  /**
   * Valuation Service
   */
  public valuations = {
    /**
     * Submit a new valuation
     */
    create: async (data: Partial<ValuationInfo>): Promise<ValuationInfo> => {
      const response = await this.client.post<APIResponse<ValuationInfo>>(
        '/api/v1/valuations',
        data
      );
      return response.data.data;
    },

    /**
     * Get current valuation
     */
    getCurrent: async (tokenId: string): Promise<ValuationInfo> => {
      const response = await this.client.get<APIResponse<ValuationInfo>>(
        `/api/v1/tokens/${tokenId}/valuation/current`
      );
      return response.data.data;
    },

    /**
     * Get valuation history
     */
    getHistory: async (
      tokenId: string
    ): Promise<{ valuations: ValuationInfo[] }> => {
      const response = await this.client.get<
        APIResponse<{ valuations: ValuationInfo[] }>
      >(`/api/v1/tokens/${tokenId}/valuations`);
      return response.data.data;
    },
  };
}

/**
 * Create a new WIA Tokenization client
 */
export function createClient(config: APIConfig): WIATokenization {
  return new WIATokenization(config);
}

/**
 * Export default instance
 */
export default WIATokenization;
