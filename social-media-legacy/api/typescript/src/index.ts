/**
 * WIA-LEG-007 Social Media Legacy Standard v2.0
 * TypeScript SDK
 *
 * Philosophy: 弘益人間 (Hongik Ingan) - Benefit All Humanity
 *
 * @packageDocumentation
 */

import axios, { AxiosInstance, AxiosRequestConfig } from 'axios';
import type {
  LegacyConfiguration,
  DeathVerificationRequest,
  DeathVerificationResponse,
  DataExportRequest,
  DataExportResponse,
  MemorialTribute,
  TributeResponse,
  AIBotQuery,
  AIBotResponse,
  APIError,
  APISuccess,
} from './types';

export * from './types';

/**
 * SDK Configuration Options
 */
export interface SDKConfig {
  /**
   * API base URL
   */
  baseURL: string;

  /**
   * API access token
   */
  accessToken: string;

  /**
   * Request timeout in milliseconds
   * @default 30000
   */
  timeout?: number;

  /**
   * Custom headers
   */
  headers?: Record<string, string>;
}

/**
 * WIA Social Media Legacy SDK
 *
 * @example
 * ```typescript
 * const sdk = new WIALegacySDK({
 *   baseURL: 'https://api.platform.com',
 *   accessToken: 'your-access-token'
 * });
 *
 * // Configure legacy settings
 * await sdk.configureLegacy({
 *   userId: 'user123',
 *   version: '2.0',
 *   legacyContacts: [...],
 *   preferences: {...}
 * });
 * ```
 */
export class WIALegacySDK {
  private client: AxiosInstance;

  constructor(config: SDKConfig) {
    this.client = axios.create({
      baseURL: config.baseURL,
      timeout: config.timeout || 30000,
      headers: {
        'Content-Type': 'application/json',
        Authorization: `Bearer ${config.accessToken}`,
        'User-Agent': 'WIA-Legacy-SDK/2.0',
        ...config.headers,
      },
    });

    // Response interceptor for error handling
    this.client.interceptors.response.use(
      (response) => response,
      (error) => {
        if (error.response?.data) {
          const apiError: APIError = error.response.data;
          throw new Error(`API Error: ${apiError.error.message} (${apiError.error.code})`);
        }
        throw error;
      }
    );
  }

  /**
   * Configure Legacy Settings
   *
   * Set up legacy contacts, memorialization preferences, and data export options.
   *
   * @param config - Legacy configuration object
   * @returns Promise resolving to success response
   *
   * @example
   * ```typescript
   * await sdk.configureLegacy({
   *   userId: 'user123',
   *   version: '2.0',
   *   legacyContacts: [
   *     {
   *       contactId: 'contact456',
   *       name: 'Jane Smith',
   *       email: 'jane@example.com',
   *       permissions: ['view', 'download', 'manage_tribute'],
   *       priority: 1
   *     }
   *   ],
   *   preferences: {
   *     action: 'memorialize',
   *     coolingOffPeriod: 30,
   *     dataExport: {
   *       enabled: true,
   *       formats: ['json', 'csv'],
   *       includeMessages: false,
   *       deliverTo: 'legacyContacts'
   *     },
   *     deletionTrigger: {
   *       type: 'time_based',
   *       years: 10
   *     }
   *   }
   * });
   * ```
   */
  async configureLegacy(config: LegacyConfiguration): Promise<APISuccess<LegacyConfiguration>> {
    const response = await this.client.post<APISuccess<LegacyConfiguration>>(
      '/api/v2/legacy/configure',
      config
    );
    return response.data;
  }

  /**
   * Get Current Legacy Configuration
   *
   * Retrieve existing legacy settings for a user.
   *
   * @param userId - User ID
   * @returns Promise resolving to legacy configuration
   */
  async getLegacyConfiguration(userId: string): Promise<APISuccess<LegacyConfiguration>> {
    const response = await this.client.get<APISuccess<LegacyConfiguration>>(
      `/api/v2/legacy/configure/${userId}`
    );
    return response.data;
  }

  /**
   * Verify Death and Initiate Legacy Process
   *
   * Submit death certificate and begin verification process.
   * Triggers 30-90 day cooling-off period.
   *
   * @param request - Death verification request with documents
   * @returns Promise resolving to verification response
   *
   * @example
   * ```typescript
   * const file = new File([certificateData], 'death-cert.pdf');
   * const result = await sdk.verifyDeath({
   *   userId: 'user123',
   *   requestorId: 'contact456',
   *   deathCertificate: file,
   *   relationship: 'spouse',
   *   dateOfDeath: '2025-01-15'
   * });
   * console.log(`Verification ${result.data.verificationId} submitted`);
   * ```
   */
  async verifyDeath(
    request: DeathVerificationRequest
  ): Promise<APISuccess<DeathVerificationResponse>> {
    const formData = new FormData();
    formData.append('userId', request.userId);
    formData.append('requestorId', request.requestorId);
    formData.append('deathCertificate', request.deathCertificate);
    formData.append('relationship', request.relationship);
    formData.append('dateOfDeath', request.dateOfDeath);

    if (request.additionalDocuments) {
      request.additionalDocuments.forEach((doc, index) => {
        formData.append(`additionalDocument${index}`, doc);
      });
    }

    const response = await this.client.post<APISuccess<DeathVerificationResponse>>(
      '/api/v2/legacy/verify-death',
      formData,
      {
        headers: { 'Content-Type': 'multipart/form-data' },
      }
    );
    return response.data;
  }

  /**
   * Get Verification Status
   *
   * Check the current status of a death verification request.
   *
   * @param verificationId - Verification request ID
   * @returns Promise resolving to verification response
   */
  async getVerificationStatus(
    verificationId: string
  ): Promise<APISuccess<DeathVerificationResponse>> {
    const response = await this.client.get<APISuccess<DeathVerificationResponse>>(
      `/api/v2/legacy/verify-death/${verificationId}`
    );
    return response.data;
  }

  /**
   * Request Data Export
   *
   * Generate and download complete data archive in specified format.
   *
   * @param request - Export request parameters
   * @returns Promise resolving to export response with download link
   *
   * @example
   * ```typescript
   * const exportRequest = await sdk.requestDataExport({
   *   userId: 'user123',
   *   format: 'json',
   *   scope: 'all'
   * });
   * console.log(`Export queued: ${exportRequest.data.exportId}`);
   * // Poll for completion
   * const status = await sdk.getExportStatus(exportRequest.data.exportId);
   * if (status.data.status === 'ready') {
   *   console.log(`Download: ${status.data.downloadUrl}`);
   * }
   * ```
   */
  async requestDataExport(request: DataExportRequest): Promise<APISuccess<DataExportResponse>> {
    const response = await this.client.post<APISuccess<DataExportResponse>>(
      '/api/v2/legacy/export',
      request
    );
    return response.data;
  }

  /**
   * Get Export Status
   *
   * Check the status of a data export request.
   *
   * @param exportId - Export request ID
   * @returns Promise resolving to export response
   */
  async getExportStatus(exportId: string): Promise<APISuccess<DataExportResponse>> {
    const response = await this.client.get<APISuccess<DataExportResponse>>(
      `/api/v2/legacy/export/${exportId}`
    );
    return response.data;
  }

  /**
   * Post Memorial Tribute
   *
   * Add a tribute message, photo, or video to a memorial page.
   * Requires legacy contact authorization.
   *
   * @param tribute - Tribute content and metadata
   * @returns Promise resolving to tribute response
   *
   * @example
   * ```typescript
   * await sdk.postTribute({
   *   memorialId: 'memorial123',
   *   content: 'Remembering the wonderful times we shared...',
   *   mediaIds: ['photo1', 'photo2'],
   *   visibility: 'friends'
   * });
   * ```
   */
  async postTribute(tribute: MemorialTribute): Promise<APISuccess<TributeResponse>> {
    const response = await this.client.post<APISuccess<TributeResponse>>(
      '/api/v2/memorial/tribute',
      tribute
    );
    return response.data;
  }

  /**
   * Query AI Memorial Bot
   *
   * Interact with AI chatbot trained on deceased's digital history.
   *
   * @param query - Bot query parameters
   * @returns Promise resolving to bot response
   *
   * @example
   * ```typescript
   * const response = await sdk.queryMemorialBot({
   *   memorialId: 'memorial123',
   *   query: 'Tell me about Mom\'s favorite vacation',
   *   maxLength: 500
   * });
   * console.log(response.data.response);
   * console.log(`Confidence: ${response.data.confidence}`);
   * ```
   */
  async queryMemorialBot(query: AIBotQuery): Promise<APISuccess<AIBotResponse>> {
    const response = await this.client.post<APISuccess<AIBotResponse>>(
      '/api/v2/memorial/aibot/query',
      query
    );
    return response.data;
  }

  /**
   * Get Memorial Page Details
   *
   * Retrieve memorial page information and settings.
   *
   * @param memorialId - Memorial page ID
   * @returns Promise resolving to memorial details
   */
  async getMemorialPage(memorialId: string): Promise<APISuccess<unknown>> {
    const response = await this.client.get<APISuccess<unknown>>(
      `/api/v2/memorial/${memorialId}`
    );
    return response.data;
  }

  /**
   * Update Memorial Settings
   *
   * Modify memorial page visibility, moderation, or other settings.
   * Requires legacy contact authorization with manage_settings permission.
   *
   * @param memorialId - Memorial page ID
   * @param settings - Updated settings
   * @returns Promise resolving to success response
   */
  async updateMemorialSettings(
    memorialId: string,
    settings: Partial<LegacyConfiguration['preferences']>
  ): Promise<APISuccess<unknown>> {
    const response = await this.client.patch<APISuccess<unknown>>(
      `/api/v2/memorial/${memorialId}/settings`,
      settings
    );
    return response.data;
  }
}

/**
 * Create SDK Instance
 *
 * Convenience function for creating SDK instance.
 *
 * @param config - SDK configuration
 * @returns Configured SDK instance
 *
 * @example
 * ```typescript
 * import { createSDK } from '@wia/social-media-legacy';
 *
 * const sdk = createSDK({
 *   baseURL: 'https://api.platform.com',
 *   accessToken: process.env.WIA_ACCESS_TOKEN!
 * });
 * ```
 */
export function createSDK(config: SDKConfig): WIALegacySDK {
  return new WIALegacySDK(config);
}

/**
 * Default export
 */
export default WIALegacySDK;
