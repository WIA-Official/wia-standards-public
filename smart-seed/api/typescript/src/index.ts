/**
 * WIA Smart Seed Standard - TypeScript SDK
 * @version 1.0.0
 * @license MIT
 */

import {
  SeedVariety,
  SeedLot,
  GerminationTest,
  SeedCertification,
  DigitalSeedPassport,
  ApiResponse,
  SeedQuery,
} from './types';

/**
 * Configuration options for the Smart Seed API client
 */
export interface SmartSeedConfig {
  /**
   * Base URL for the API endpoint
   */
  baseUrl: string;

  /**
   * API key for authentication
   */
  apiKey?: string;

  /**
   * Request timeout in milliseconds
   * @default 30000
   */
  timeout?: number;
}

/**
 * Smart Seed API Client
 *
 * Provides methods to interact with the WIA Smart Seed standard, enabling
 * seed variety registration, germination testing, seed certification, and
 * digital seed passport management.
 *
 * @example
 * ```typescript
 * const client = new SmartSeedClient({
 *   baseUrl: 'https://api.example.com/seed',
 *   apiKey: 'your-api-key'
 * });
 *
 * // Get seed variety information
 * const variety = await client.getVariety('550e8400-e29b-41d4-a716-446655440000');
 * console.log(variety.data);
 * ```
 */
export class SmartSeedClient {
  private config: SmartSeedConfig;
  private headers: Record<string, string>;

  /**
   * Creates a new Smart Seed API client
   *
   * @param config - Configuration options
   */
  constructor(config: SmartSeedConfig) {
    this.config = {
      timeout: 30000,
      ...config,
    };

    this.headers = {
      'Content-Type': 'application/json',
      'Accept': 'application/json',
    };

    if (config.apiKey) {
      this.headers['Authorization'] = `Bearer ${config.apiKey}`;
    }
  }

  /**
   * Makes an HTTP request to the API
   *
   * @param endpoint - API endpoint path
   * @param options - Fetch options
   * @returns Promise resolving to the response data
   */
  private async request<T>(
    endpoint: string,
    options: RequestInit = {}
  ): Promise<ApiResponse<T>> {
    const url = `${this.config.baseUrl}${endpoint}`;
    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), this.config.timeout);

    try {
      const response = await fetch(url, {
        ...options,
        headers: {
          ...this.headers,
          ...options.headers,
        },
        signal: controller.signal,
      });

      clearTimeout(timeoutId);

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }

      const data = await response.json();
      return {
        success: true,
        data,
        timestamp: new Date().toISOString(),
      };
    } catch (error) {
      clearTimeout(timeoutId);
      return {
        success: false,
        error: error instanceof Error ? error.message : 'Unknown error',
        timestamp: new Date().toISOString(),
      };
    }
  }

  // ==================== Seed Variety Management ====================

  /**
   * Get seed variety information
   *
   * @param varietyId - Unique identifier for the variety
   * @returns Promise resolving to seed variety data
   */
  async getVariety(varietyId: string): Promise<ApiResponse<SeedVariety>> {
    return this.request<SeedVariety>(`/varieties/${varietyId}`);
  }

  /**
   * Register a new seed variety
   *
   * @param variety - Seed variety data to register
   * @returns Promise resolving to registered variety data
   */
  async registerVariety(variety: SeedVariety): Promise<ApiResponse<SeedVariety>> {
    return this.request<SeedVariety>('/varieties', {
      method: 'POST',
      body: JSON.stringify(variety),
    });
  }

  /**
   * Update seed variety information
   *
   * @param varietyId - Variety ID to update
   * @param variety - Updated variety data
   * @returns Promise resolving to updated variety data
   */
  async updateVariety(
    varietyId: string,
    variety: Partial<SeedVariety>
  ): Promise<ApiResponse<SeedVariety>> {
    return this.request<SeedVariety>(`/varieties/${varietyId}`, {
      method: 'PUT',
      body: JSON.stringify(variety),
    });
  }

  /**
   * Delete a seed variety
   *
   * @param varietyId - Variety ID to delete
   * @returns Promise resolving to deletion confirmation
   */
  async deleteVariety(varietyId: string): Promise<ApiResponse<void>> {
    return this.request<void>(`/varieties/${varietyId}`, {
      method: 'DELETE',
    });
  }

  /**
   * List seed varieties with optional filtering
   *
   * @param query - Query parameters for filtering
   * @returns Promise resolving to list of varieties
   */
  async listVarieties(query?: SeedQuery): Promise<ApiResponse<SeedVariety[]>> {
    const params = new URLSearchParams();
    if (query) {
      Object.entries(query).forEach(([key, value]) => {
        if (value !== undefined) {
          params.append(key, String(value));
        }
      });
    }

    const endpoint = `/varieties${params.toString() ? `?${params.toString()}` : ''}`;
    return this.request<SeedVariety[]>(endpoint);
  }

  // ==================== Seed Lot Management ====================

  /**
   * Get seed lot information
   *
   * @param lotId - Unique identifier for the lot
   * @returns Promise resolving to seed lot data
   */
  async getLot(lotId: string): Promise<ApiResponse<SeedLot>> {
    return this.request<SeedLot>(`/lots/${lotId}`);
  }

  /**
   * Create a new seed lot
   *
   * @param lot - Seed lot data to create
   * @returns Promise resolving to created lot data
   */
  async createLot(lot: SeedLot): Promise<ApiResponse<SeedLot>> {
    return this.request<SeedLot>('/lots', {
      method: 'POST',
      body: JSON.stringify(lot),
    });
  }

  /**
   * Update seed lot information
   *
   * @param lotId - Lot ID to update
   * @param lot - Updated lot data
   * @returns Promise resolving to updated lot data
   */
  async updateLot(lotId: string, lot: Partial<SeedLot>): Promise<ApiResponse<SeedLot>> {
    return this.request<SeedLot>(`/lots/${lotId}`, {
      method: 'PUT',
      body: JSON.stringify(lot),
    });
  }

  /**
   * Delete a seed lot
   *
   * @param lotId - Lot ID to delete
   * @returns Promise resolving to deletion confirmation
   */
  async deleteLot(lotId: string): Promise<ApiResponse<void>> {
    return this.request<void>(`/lots/${lotId}`, {
      method: 'DELETE',
    });
  }

  /**
   * List seed lots with optional filtering
   *
   * @param query - Query parameters for filtering
   * @returns Promise resolving to list of lots
   */
  async listLots(query?: SeedQuery): Promise<ApiResponse<SeedLot[]>> {
    const params = new URLSearchParams();
    if (query) {
      Object.entries(query).forEach(([key, value]) => {
        if (value !== undefined) {
          params.append(key, String(value));
        }
      });
    }

    const endpoint = `/lots${params.toString() ? `?${params.toString()}` : ''}`;
    return this.request<SeedLot[]>(endpoint);
  }

  // ==================== Germination Testing ====================

  /**
   * Get germination test results
   *
   * @param testId - Test ID
   * @returns Promise resolving to germination test data
   */
  async getGerminationTest(testId: string): Promise<ApiResponse<GerminationTest>> {
    return this.request<GerminationTest>(`/germination-tests/${testId}`);
  }

  /**
   * Submit germination test results
   *
   * @param test - Germination test data
   * @returns Promise resolving to submitted test data
   */
  async submitGerminationTest(test: GerminationTest): Promise<ApiResponse<GerminationTest>> {
    return this.request<GerminationTest>('/germination-tests', {
      method: 'POST',
      body: JSON.stringify(test),
    });
  }

  /**
   * List germination tests for a lot
   *
   * @param lotId - Lot ID
   * @returns Promise resolving to list of tests
   */
  async listGerminationTests(lotId: string): Promise<ApiResponse<GerminationTest[]>> {
    return this.request<GerminationTest[]>(`/lots/${lotId}/germination-tests`);
  }

  // ==================== Seed Certification ====================

  /**
   * Get seed certification information
   *
   * @param certificateId - Certificate ID
   * @returns Promise resolving to certification data
   */
  async getCertification(certificateId: string): Promise<ApiResponse<SeedCertification>> {
    return this.request<SeedCertification>(`/certifications/${certificateId}`);
  }

  /**
   * Issue a seed certification
   *
   * @param certification - Certification data
   * @returns Promise resolving to issued certification
   */
  async issueCertification(
    certification: SeedCertification
  ): Promise<ApiResponse<SeedCertification>> {
    return this.request<SeedCertification>('/certifications', {
      method: 'POST',
      body: JSON.stringify(certification),
    });
  }

  /**
   * Verify seed certification
   *
   * @param certificateId - Certificate ID to verify
   * @returns Promise resolving to verification result
   */
  async verifyCertification(
    certificateId: string
  ): Promise<ApiResponse<{ valid: boolean; details: SeedCertification }>> {
    return this.request<{ valid: boolean; details: SeedCertification }>(
      `/certifications/${certificateId}/verify`
    );
  }

  // ==================== Digital Seed Passport ====================

  /**
   * Get digital seed passport
   *
   * @param passportId - Passport ID
   * @returns Promise resolving to passport data
   */
  async getPassport(passportId: string): Promise<ApiResponse<DigitalSeedPassport>> {
    return this.request<DigitalSeedPassport>(`/passports/${passportId}`);
  }

  /**
   * Generate digital seed passport
   *
   * @param passport - Passport data
   * @returns Promise resolving to generated passport
   */
  async generatePassport(
    passport: DigitalSeedPassport
  ): Promise<ApiResponse<DigitalSeedPassport>> {
    return this.request<DigitalSeedPassport>('/passports', {
      method: 'POST',
      body: JSON.stringify(passport),
    });
  }

  /**
   * Scan QR code and retrieve passport
   *
   * @param qrCode - QR code data
   * @returns Promise resolving to passport data
   */
  async scanQRCode(qrCode: string): Promise<ApiResponse<DigitalSeedPassport>> {
    return this.request<DigitalSeedPassport>('/passports/scan', {
      method: 'POST',
      body: JSON.stringify({ qrCode }),
    });
  }

  /**
   * Verify seed passport blockchain record
   *
   * @param passportId - Passport ID
   * @returns Promise resolving to blockchain verification result
   */
  async verifyPassportBlockchain(
    passportId: string
  ): Promise<ApiResponse<{ valid: boolean; blockchainRecord: any }>> {
    return this.request<{ valid: boolean; blockchainRecord: any }>(
      `/passports/${passportId}/verify-blockchain`
    );
  }
}

// Export all types
export * from './types';

// Export default client
export default SmartSeedClient;
