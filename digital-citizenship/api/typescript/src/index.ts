/**
 * WIA-SOC-004 Digital Citizenship Standard - TypeScript SDK
 * @module @wia/digital-citizenship
 */

import axios, { AxiosInstance } from 'axios';
import EventEmitter from 'eventemitter3';
import * as crypto from 'crypto';

import {
  ClientConfig,
  DigitalIdentity,
  DigitalRights,
  VerificationRequest,
  VerificationResponse,
  CrossBorderVerification,
  CrossBorderResponse,
  RightsExerciseRequest,
  RightsExerciseResponse,
  ApiResponse,
  PaginatedResponse,
  GlobalStatistics,
  CitizenStatistics,
  PersonalInfo,
  Biometrics,
  Credential,
} from './types';

export * from './types';

/**
 * Digital Citizenship Client
 * Main SDK class for interacting with WIA-SOC-004 Digital Citizenship services
 */
export class DigitalCitizenshipClient extends EventEmitter {
  private client: AxiosInstance;
  private config: ClientConfig;

  constructor(config: ClientConfig) {
    super();
    this.config = config;

    const baseURL = config.baseUrl || this.getDefaultBaseUrl(config.environment, config.region);

    this.client = axios.create({
      baseURL,
      timeout: config.timeout || 30000,
      headers: {
        'Authorization': `Bearer ${config.apiKey}`,
        'Content-Type': 'application/json',
        'X-WIA-Standard': 'SOC-004',
        'X-WIA-Version': '1.0.0',
      },
    });

    // Request interceptor for logging
    if (config.enableLogging) {
      this.client.interceptors.request.use(request => {
        this.log('Request:', request.method?.toUpperCase(), request.url);
        return request;
      });
    }

    // Response interceptor for error handling and retry
    this.client.interceptors.response.use(
      response => response,
      async error => {
        if (error.response?.status === 429) {
          // Rate limit exceeded
          this.emit('rateLimitExceeded', error);
        }
        return Promise.reject(error);
      }
    );
  }

  private getDefaultBaseUrl(environment: string, region?: string): string {
    const regionPrefix = region || 'global';
    const envSuffix = environment === 'production' ? '' : `-${environment}`;
    return `https://api${envSuffix}.wia.org/${regionPrefix}/digital-citizenship/v1`;
  }

  private log(...args: any[]): void {
    if (this.config.enableLogging) {
      console.log('[WIA Digital Citizenship]', ...args);
    }
  }

  // ============================================================================
  // Identity Management Methods
  // ============================================================================

  /**
   * Create a new digital identity
   */
  async createIdentity(params: {
    personalInfo: PersonalInfo;
    biometrics?: Biometrics;
    credentials?: Credential[];
  }): Promise<DigitalIdentity> {
    this.log('Creating digital identity...');

    const response = await this.client.post<ApiResponse<DigitalIdentity>>(
      '/identity',
      params
    );

    if (response.data.success && response.data.data) {
      this.emit('identityCreated', response.data.data);
      return response.data.data;
    }

    throw new Error(response.data.error?.message || 'Failed to create identity');
  }

  /**
   * Get identity by citizen ID
   */
  async getIdentity(citizenId: string): Promise<DigitalIdentity> {
    this.log('Fetching identity:', citizenId);

    const response = await this.client.get<ApiResponse<DigitalIdentity>>(
      `/identity/${citizenId}`
    );

    if (response.data.success && response.data.data) {
      return response.data.data;
    }

    throw new Error(response.data.error?.message || 'Failed to fetch identity');
  }

  /**
   * Update identity information
   */
  async updateIdentity(
    citizenId: string,
    updates: Partial<{
      personalInfo: Partial<PersonalInfo>;
      credentials: Credential[];
      biometrics: Partial<Biometrics>;
    }>
  ): Promise<DigitalIdentity> {
    this.log('Updating identity:', citizenId);

    const response = await this.client.patch<ApiResponse<DigitalIdentity>>(
      `/identity/${citizenId}`,
      updates
    );

    if (response.data.success && response.data.data) {
      this.emit('identityUpdated', response.data.data);
      return response.data.data;
    }

    throw new Error(response.data.error?.message || 'Failed to update identity');
  }

  /**
   * Delete identity (Right to be Forgotten)
   */
  async deleteIdentity(citizenId: string, confirmationToken: string): Promise<void> {
    this.log('Deleting identity:', citizenId);

    const response = await this.client.delete<ApiResponse<void>>(
      `/identity/${citizenId}`,
      {
        headers: {
          'X-Confirmation-Token': confirmationToken,
        },
      }
    );

    if (response.data.success) {
      this.emit('identityDeleted', { citizenId });
      return;
    }

    throw new Error(response.data.error?.message || 'Failed to delete identity');
  }

  // ============================================================================
  // Verification Methods
  // ============================================================================

  /**
   * Verify identity using biometric or other methods
   */
  async verifyIdentity(request: VerificationRequest): Promise<VerificationResponse> {
    this.log('Verifying identity:', request.citizenId);

    // Add timestamp and nonce if not provided
    const enrichedRequest: VerificationRequest = {
      ...request,
      timestamp: request.timestamp || new Date().toISOString(),
      nonce: request.nonce || this.generateNonce(),
    };

    const response = await this.client.post<ApiResponse<VerificationResponse>>(
      '/verify',
      enrichedRequest
    );

    if (response.data.success && response.data.data) {
      const result = response.data.data;
      this.emit('verificationCompleted', result);
      return result;
    }

    throw new Error(response.data.error?.message || 'Verification failed');
  }

  /**
   * Verify identity with zero-knowledge proof
   */
  async verifyWithZKP(params: {
    citizenId: string;
    proofType: 'age' | 'citizenship' | 'credential' | 'financial';
    threshold?: number;
    proof: string; // Zero-knowledge proof data
  }): Promise<{ verified: boolean; confidence: number }> {
    this.log('Verifying with zero-knowledge proof:', params.citizenId);

    const response = await this.client.post<ApiResponse<{ verified: boolean; confidence: number }>>(
      '/verify/zkp',
      params
    );

    if (response.data.success && response.data.data) {
      return response.data.data;
    }

    throw new Error(response.data.error?.message || 'ZKP verification failed');
  }

  // ============================================================================
  // Rights Management Methods
  // ============================================================================

  /**
   * Get digital rights for a citizen
   */
  async getRights(citizenId: string): Promise<DigitalRights> {
    this.log('Fetching rights:', citizenId);

    const response = await this.client.get<ApiResponse<DigitalRights>>(
      `/rights/${citizenId}`
    );

    if (response.data.success && response.data.data) {
      return response.data.data;
    }

    throw new Error(response.data.error?.message || 'Failed to fetch rights');
  }

  /**
   * Exercise privacy rights (e.g., data erasure, portability)
   */
  async exerciseRights(request: RightsExerciseRequest): Promise<RightsExerciseResponse> {
    this.log('Exercising rights:', request.rightType);

    const response = await this.client.post<ApiResponse<RightsExerciseResponse>>(
      '/rights/exercise',
      request
    );

    if (response.data.success && response.data.data) {
      this.emit('rightsExercised', response.data.data);
      return response.data.data;
    }

    throw new Error(response.data.error?.message || 'Failed to exercise rights');
  }

  /**
   * Check rights exercise request status
   */
  async getRightsRequestStatus(requestId: string): Promise<RightsExerciseResponse> {
    this.log('Checking rights request status:', requestId);

    const response = await this.client.get<ApiResponse<RightsExerciseResponse>>(
      `/rights/requests/${requestId}`
    );

    if (response.data.success && response.data.data) {
      return response.data.data;
    }

    throw new Error(response.data.error?.message || 'Failed to fetch request status');
  }

  // ============================================================================
  // Cross-Border Methods
  // ============================================================================

  /**
   * Verify identity for cross-border services
   */
  async verifyCrossBorder(request: CrossBorderVerification): Promise<CrossBorderResponse> {
    this.log('Cross-border verification:', request.sourceCountry, '->', request.targetCountry);

    const response = await this.client.post<ApiResponse<CrossBorderResponse>>(
      '/cross-border/verify',
      {
        ...request,
        timestamp: request.timestamp || new Date().toISOString(),
      }
    );

    if (response.data.success && response.data.data) {
      this.emit('crossBorderVerified', response.data.data);
      return response.data.data;
    }

    throw new Error(response.data.error?.message || 'Cross-border verification failed');
  }

  /**
   * Get list of countries with mutual recognition
   */
  async getMutualRecognitionCountries(): Promise<string[]> {
    this.log('Fetching mutual recognition countries...');

    const response = await this.client.get<ApiResponse<string[]>>(
      '/cross-border/countries'
    );

    if (response.data.success && response.data.data) {
      return response.data.data;
    }

    throw new Error(response.data.error?.message || 'Failed to fetch countries');
  }

  // ============================================================================
  // Credential Methods
  // ============================================================================

  /**
   * Add a new credential to identity
   */
  async addCredential(citizenId: string, credential: Credential): Promise<DigitalIdentity> {
    this.log('Adding credential:', credential.type);

    const response = await this.client.post<ApiResponse<DigitalIdentity>>(
      `/identity/${citizenId}/credentials`,
      credential
    );

    if (response.data.success && response.data.data) {
      this.emit('credentialAdded', { citizenId, credential });
      return response.data.data;
    }

    throw new Error(response.data.error?.message || 'Failed to add credential');
  }

  /**
   * Verify a credential
   */
  async verifyCredential(
    citizenId: string,
    credentialId: string
  ): Promise<{ verified: boolean; status: string }> {
    this.log('Verifying credential:', credentialId);

    const response = await this.client.post<ApiResponse<{ verified: boolean; status: string }>>(
      `/identity/${citizenId}/credentials/${credentialId}/verify`
    );

    if (response.data.success && response.data.data) {
      return response.data.data;
    }

    throw new Error(response.data.error?.message || 'Credential verification failed');
  }

  // ============================================================================
  // Statistics Methods
  // ============================================================================

  /**
   * Get global system statistics
   */
  async getGlobalStatistics(): Promise<GlobalStatistics> {
    this.log('Fetching global statistics...');

    const response = await this.client.get<ApiResponse<GlobalStatistics>>(
      '/statistics/global'
    );

    if (response.data.success && response.data.data) {
      return response.data.data;
    }

    throw new Error(response.data.error?.message || 'Failed to fetch statistics');
  }

  /**
   * Get citizen-specific statistics
   */
  async getCitizenStatistics(citizenId: string): Promise<CitizenStatistics> {
    this.log('Fetching citizen statistics:', citizenId);

    const response = await this.client.get<ApiResponse<CitizenStatistics>>(
      `/statistics/citizen/${citizenId}`
    );

    if (response.data.success && response.data.data) {
      return response.data.data;
    }

    throw new Error(response.data.error?.message || 'Failed to fetch statistics');
  }

  // ============================================================================
  // Utility Methods
  // ============================================================================

  /**
   * Generate a random nonce for verification requests
   */
  private generateNonce(): string {
    return crypto.randomBytes(32).toString('hex');
  }

  /**
   * Health check
   */
  async healthCheck(): Promise<{ status: 'healthy' | 'degraded' | 'down'; uptime: number }> {
    try {
      const response = await this.client.get<ApiResponse<{ status: string; uptime: number }>>(
        '/health'
      );

      if (response.data.success && response.data.data) {
        return response.data.data as { status: 'healthy' | 'degraded' | 'down'; uptime: number };
      }

      return { status: 'down', uptime: 0 };
    } catch (error) {
      return { status: 'down', uptime: 0 };
    }
  }

  /**
   * Validate citizen ID format
   */
  static validateCitizenId(citizenId: string): boolean {
    return /^WIA-[A-Z0-9]{9}$/.test(citizenId);
  }

  /**
   * Generate confirmation token for sensitive operations
   */
  async generateConfirmationToken(citizenId: string, operation: string): Promise<string> {
    const response = await this.client.post<ApiResponse<{ token: string }>>(
      '/auth/confirmation-token',
      { citizenId, operation }
    );

    if (response.data.success && response.data.data) {
      return response.data.data.token;
    }

    throw new Error(response.data.error?.message || 'Failed to generate confirmation token');
  }
}

/**
 * Convenience function to create a client instance
 */
export function createClient(config: ClientConfig): DigitalCitizenshipClient {
  return new DigitalCitizenshipClient(config);
}

/**
 * Default export
 */
export default DigitalCitizenshipClient;
