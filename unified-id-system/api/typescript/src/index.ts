/**
 * WIA-UNI-002 Unified ID System - TypeScript SDK
 *
 * @version 1.0.0
 * @license MIT
 *
 * @example
 * ```typescript
 * import { UnifiedIdClient } from '@wia/unified-id-sdk';
 *
 * const client = new UnifiedIdClient({
 *   apiKey: 'your-api-key',
 *   environment: 'production'
 * });
 *
 * const result = await client.verify({ credential });
 * console.log(result.verified); // true
 * ```
 */

import axios, { AxiosInstance } from 'axios';
import type {
  ClientConfig,
  IssueCredentialRequest,
  IssueCredentialResponse,
  VerifyCredentialRequest,
  VerificationResult,
  GenerateZKProofRequest,
  GenerateZKProofResponse,
  RevokeCredentialRequest,
  RevokeCredentialResponse,
  APIError
} from './types';

export * from './types';

/**
 * Base URL for different environments
 */
const BASE_URLS = {
  production: 'https://api.unified-id.kr/v1',
  staging: 'https://api-staging.unified-id.kr/v1',
  development: 'http://localhost:3000/v1'
};

/**
 * WIA-UNI-002 Unified ID System Client
 */
export class UnifiedIdClient {
  private client: AxiosInstance;
  private apiKey: string;

  /**
   * Create a new Unified ID client
   *
   * @param config - Client configuration
   */
  constructor(config: ClientConfig) {
    this.apiKey = config.apiKey;

    const baseURL = config.baseUrl || BASE_URLS[config.environment || 'production'];

    this.client = axios.create({
      baseURL,
      timeout: config.timeout || 30000,
      headers: {
        'Content-Type': 'application/json',
        'User-Agent': '@wia/unified-id-sdk/1.0.0'
      }
    });

    // Add authorization header to all requests
    this.client.interceptors.request.use((request) => {
      request.headers['Authorization'] = `Bearer ${this.apiKey}`;
      return request;
    });

    // Error handling interceptor
    this.client.interceptors.response.use(
      (response) => response,
      (error) => {
        if (error.response?.data) {
          throw new UnifiedIdError(error.response.data as APIError);
        }
        throw error;
      }
    );
  }

  /**
   * Issue a new unified ID credential
   *
   * @param request - Credential issuance request
   * @returns Issued credential
   *
   * @throws {UnifiedIdError} If issuance fails
   *
   * @example
   * ```typescript
   * const credential = await client.issueCredential({
   *   personalInfo: {
   *     familyName: "Kim",
   *     givenName: "MinJun",
   *     fullName: "Kim MinJun",
   *     birthDate: "1990-05-15"
   *   },
   *   origin: {
   *     region: OriginRegion.SOUTH,
   *     previousId: "900515-1234567",
   *     registrationDate: new Date().toISOString()
   *   },
   *   biometric: {
   *     fingerprintHash: "sha256:...",
   *     facialHash: "sha256:..."
   *   }
   * });
   * ```
   */
  async issueCredential(request: IssueCredentialRequest): Promise<IssueCredentialResponse> {
    const response = await this.client.post<IssueCredentialResponse>('/credentials', request);
    return response.data;
  }

  /**
   * Verify a unified ID credential
   *
   * @param request - Verification request
   * @returns Verification result
   *
   * @example
   * ```typescript
   * const result = await client.verify({
   *   credential: verifiableCredential
   * });
   *
   * if (result.verified) {
   *   console.log('Credential is valid');
   * }
   * ```
   */
  async verify(request: VerifyCredentialRequest): Promise<VerificationResult> {
    const response = await this.client.post<VerificationResult>('/verify', request);
    return response.data;
  }

  /**
   * Generate a zero-knowledge proof
   *
   * @param request - ZK proof generation request
   * @returns Generated proof
   *
   * @example
   * ```typescript
   * const proofResponse = await client.generateZKProof({
   *   unifiedId: "UNI-KR-900515-1234",
   *   proofType: ZKProofType.AGE_OVER_18,
   *   parameters: {
   *     currentDate: new Date().toISOString().split('T')[0]
   *   }
   * });
   * ```
   */
  async generateZKProof(request: GenerateZKProofRequest): Promise<GenerateZKProofResponse> {
    const response = await this.client.post<GenerateZKProofResponse>('/proofs/zk', request);
    return response.data;
  }

  /**
   * Revoke a unified ID credential
   *
   * @param request - Revocation request
   * @returns Revocation confirmation
   *
   * @example
   * ```typescript
   * const result = await client.revoke({
   *   unifiedId: "UNI-KR-900515-1234",
   *   reason: "lost"
   * });
   * ```
   */
  async revoke(request: RevokeCredentialRequest): Promise<RevokeCredentialResponse> {
    const response = await this.client.post<RevokeCredentialResponse>('/revoke', request);
    return response.data;
  }

  /**
   * Get credential by unified ID
   *
   * @param unifiedId - The unified ID
   * @returns The credential
   */
  async getCredential(unifiedId: string): Promise<IssueCredentialResponse> {
    const response = await this.client.get<IssueCredentialResponse>(`/credentials/${unifiedId}`);
    return response.data;
  }
}

/**
 * Custom error class for Unified ID SDK
 */
export class UnifiedIdError extends Error {
  public code: string;
  public timestamp: string;
  public requestId: string;

  constructor(error: APIError) {
    super(error.error.message);
    this.name = 'UnifiedIdError';
    this.code = error.error.code;
    this.timestamp = error.error.timestamp;
    this.requestId = error.error.requestId;
  }
}

/**
 * Utility functions
 */
export const utils = {
  /**
   * Validate unified ID format
   *
   * @param id - The ID to validate
   * @returns True if valid
   */
  isValidUnifiedId(id: string): boolean {
    return /^UNI-KR-\d{6}-\d{4}$/.test(id);
  },

  /**
   * Extract birth date from unified ID
   *
   * @param id - The unified ID
   * @returns Birth date or null if invalid
   */
  extractBirthDate(id: string): Date | null {
    if (!this.isValidUnifiedId(id)) return null;

    const parts = id.split('-');
    const dateStr = parts[2]; // YYMMDD

    const yy = parseInt(dateStr.substring(0, 2));
    const mm = parseInt(dateStr.substring(2, 4));
    const dd = parseInt(dateStr.substring(4, 6));

    // Assume 19xx for yy >= 50, otherwise 20xx
    const year = yy >= 50 ? 1900 + yy : 2000 + yy;

    return new Date(year, mm - 1, dd);
  },

  /**
   * Calculate age from unified ID
   *
   * @param id - The unified ID
   * @param referenceDate - Reference date for age calculation (default: today)
   * @returns Age in years or null if invalid
   */
  calculateAge(id: string, referenceDate: Date = new Date()): number | null {
    const birthDate = this.extractBirthDate(id);
    if (!birthDate) return null;

    let age = referenceDate.getFullYear() - birthDate.getFullYear();
    const monthDiff = referenceDate.getMonth() - birthDate.getMonth();

    if (monthDiff < 0 || (monthDiff === 0 && referenceDate.getDate() < birthDate.getDate())) {
      age--;
    }

    return age;
  }
};

/**
 * Default export
 */
export default UnifiedIdClient;
