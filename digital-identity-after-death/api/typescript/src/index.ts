/**
 * WIA-LEG-010 Digital Identity After Death Standard - TypeScript SDK
 * @module @wia/digital-identity-after-death
 */

import axios, { AxiosInstance } from 'axios';
import { v4 as uuidv4 } from 'uuid';
import * as jose from 'jose';

export * from './types';

import type {
  ClientConfig,
  DeathCertificate,
  VerificationResult,
  ProofOfDeath,
  ExecutorCredential,
  RevocationRequest,
  RevocationResult,
  PersonaConfig,
  Persona,
  MemorialConfig,
  Memorial,
  Tribute,
  AuditLogEntry,
  ApiResponse,
  PaginationParams,
  PaginatedResponse,
} from './types';

/**
 * Main client for WIA-LEG-010 Digital Identity After Death services
 */
export class DigitalIdentityAfterDeathClient {
  private client: AxiosInstance;
  private config: Required<ClientConfig>;

  constructor(config: ClientConfig) {
    this.config = {
      apiKey: config.apiKey,
      baseUrl: config.baseUrl || 'https://api.wia.org/v1/leg-010',
      network: config.network || 'mainnet',
      jurisdiction: config.jurisdiction || 'US',
      timeout: config.timeout || 30000,
      debug: config.debug || false,
    };

    this.client = axios.create({
      baseURL: this.config.baseUrl,
      timeout: this.config.timeout,
      headers: {
        'Content-Type': 'application/json',
        'X-API-Key': this.config.apiKey,
        'X-Network': this.config.network,
        'X-Client-Version': '1.0.0',
      },
    });

    // Add request/response interceptors
    this.setupInterceptors();
  }

  private setupInterceptors(): void {
    // Request interceptor
    this.client.interceptors.request.use(
      (config) => {
        if (this.config.debug) {
          console.log('[WIA-LEG-010] Request:', config.method?.toUpperCase(), config.url);
        }
        return config;
      },
      (error) => {
        if (this.config.debug) {
          console.error('[WIA-LEG-010] Request Error:', error);
        }
        return Promise.reject(error);
      }
    );

    // Response interceptor
    this.client.interceptors.response.use(
      (response) => {
        if (this.config.debug) {
          console.log('[WIA-LEG-010] Response:', response.status, response.config.url);
        }
        return response;
      },
      (error) => {
        if (this.config.debug) {
          console.error('[WIA-LEG-010] Response Error:', error.response?.status, error.message);
        }
        return Promise.reject(this.formatError(error));
      }
    );
  }

  private formatError(error: any): Error {
    if (error.response?.data?.error) {
      return new Error(`WIA-LEG-010 Error: ${error.response.data.error.message}`);
    }
    return error;
  }

  /**
   * Report a death event with certificate data
   */
  async reportDeath(certificate: DeathCertificate): Promise<ApiResponse<VerificationResult>> {
    const response = await this.client.post<ApiResponse<VerificationResult>>(
      '/death/report',
      certificate
    );
    return response.data;
  }

  /**
   * Verify death using certificate ID
   */
  async verifyDeath(certificateId: string, jurisdiction?: string): Promise<ApiResponse<VerificationResult>> {
    const response = await this.client.get<ApiResponse<VerificationResult>>(
      `/death/verify/${certificateId}`,
      {
        params: { jurisdiction: jurisdiction || this.config.jurisdiction },
      }
    );
    return response.data;
  }

  /**
   * Get proof-of-death for a subject
   */
  async getProofOfDeath(subjectId: string): Promise<ApiResponse<ProofOfDeath>> {
    const response = await this.client.get<ApiResponse<ProofOfDeath>>(
      `/death/proof/${subjectId}`
    );
    return response.data;
  }

  /**
   * Get identity status (active, deceased, etc.)
   */
  async getIdentityStatus(subjectId: string): Promise<ApiResponse<{ status: string; verifiedAt?: string }>> {
    const response = await this.client.get<ApiResponse<{ status: string; verifiedAt?: string }>>(
      `/identity/status/${subjectId}`
    );
    return response.data;
  }

  /**
   * Revoke credentials for a deceased person
   */
  async revokeCredentials(request: RevocationRequest): Promise<ApiResponse<RevocationResult>> {
    const response = await this.client.post<ApiResponse<RevocationResult>>(
      '/credentials/revoke',
      request
    );
    return response.data;
  }

  /**
   * Get revocation status for a subject
   */
  async getRevocationStatus(subjectId: string): Promise<ApiResponse<RevocationResult>> {
    const response = await this.client.get<ApiResponse<RevocationResult>>(
      `/credentials/revoke/status/${subjectId}`
    );
    return response.data;
  }

  /**
   * Issue executor access credentials
   */
  async issueExecutorAuth(request: {
    executorId: string;
    deceasedId: string;
    permissions: string[];
    legalDocuments: any[];
    expiresIn?: string;
  }): Promise<ApiResponse<ExecutorCredential & { accessToken: string }>> {
    const response = await this.client.post<ApiResponse<ExecutorCredential & { accessToken: string }>>(
      '/executor/authorize',
      request
    );
    return response.data;
  }

  /**
   * Verify executor authorization
   */
  async verifyExecutorAuth(credentialId: string): Promise<ApiResponse<ExecutorCredential>> {
    const response = await this.client.get<ApiResponse<ExecutorCredential>>(
      `/executor/verify/${credentialId}`
    );
    return response.data;
  }

  /**
   * Revoke executor access
   */
  async revokeExecutorAuth(credentialId: string, reason?: string): Promise<ApiResponse<void>> {
    const response = await this.client.post<ApiResponse<void>>(
      `/executor/revoke/${credentialId}`,
      { reason }
    );
    return response.data;
  }

  /**
   * Create AI persona
   */
  async createPersona(config: PersonaConfig): Promise<ApiResponse<Persona>> {
    const response = await this.client.post<ApiResponse<Persona>>(
      '/persona/create',
      config
    );
    return response.data;
  }

  /**
   * Get AI persona
   */
  async getPersona(personaId: string): Promise<ApiResponse<Persona>> {
    const response = await this.client.get<ApiResponse<Persona>>(
      `/persona/${personaId}`
    );
    return response.data;
  }

  /**
   * Interact with AI persona
   */
  async interactWithPersona(
    personaId: string,
    message: string,
    sessionId?: string
  ): Promise<ApiResponse<{ response: string; sessionId: string }>> {
    const response = await this.client.post<ApiResponse<{ response: string; sessionId: string }>>(
      `/persona/${personaId}/interact`,
      {
        message,
        sessionId: sessionId || uuidv4(),
      }
    );
    return response.data;
  }

  /**
   * Delete AI persona
   */
  async deletePersona(personaId: string): Promise<ApiResponse<void>> {
    const response = await this.client.delete<ApiResponse<void>>(
      `/persona/${personaId}`
    );
    return response.data;
  }

  /**
   * Create digital memorial
   */
  async createMemorial(config: MemorialConfig): Promise<ApiResponse<Memorial>> {
    const response = await this.client.post<ApiResponse<Memorial>>(
      '/memorial/create',
      config
    );
    return response.data;
  }

  /**
   * Get memorial
   */
  async getMemorial(memorialId: string): Promise<ApiResponse<Memorial>> {
    const response = await this.client.get<ApiResponse<Memorial>>(
      `/memorial/${memorialId}`
    );
    return response.data;
  }

  /**
   * Add tribute to memorial
   */
  async addTribute(tribute: Tribute): Promise<ApiResponse<Tribute>> {
    const response = await this.client.post<ApiResponse<Tribute>>(
      '/memorial/tribute',
      tribute
    );
    return response.data;
  }

  /**
   * Get tributes for memorial
   */
  async getTributes(
    memorialId: string,
    pagination?: PaginationParams
  ): Promise<ApiResponse<PaginatedResponse<Tribute>>> {
    const response = await this.client.get<ApiResponse<PaginatedResponse<Tribute>>>(
      `/memorial/${memorialId}/tributes`,
      { params: pagination }
    );
    return response.data;
  }

  /**
   * Get audit logs for deceased person
   */
  async getAuditLogs(
    deceasedId: string,
    pagination?: PaginationParams
  ): Promise<ApiResponse<PaginatedResponse<AuditLogEntry>>> {
    const response = await this.client.get<ApiResponse<PaginatedResponse<AuditLogEntry>>>(
      `/audit/${deceasedId}`,
      { params: pagination }
    );
    return response.data;
  }

  /**
   * Verify audit log integrity
   */
  async verifyAuditIntegrity(deceasedId: string): Promise<ApiResponse<{
    valid: boolean;
    totalEntries: number;
    tampered: boolean;
  }>> {
    const response = await this.client.get<ApiResponse<{
      valid: boolean;
      totalEntries: number;
      tampered: boolean;
    }>>(`/audit/${deceasedId}/verify`);
    return response.data;
  }

  /**
   * Health check
   */
  async healthCheck(): Promise<ApiResponse<{ status: string; timestamp: string }>> {
    const response = await this.client.get<ApiResponse<{ status: string; timestamp: string }>>(
      '/health'
    );
    return response.data;
  }
}

/**
 * Create a new client instance
 */
export function createClient(config: ClientConfig): DigitalIdentityAfterDeathClient {
  return new DigitalIdentityAfterDeathClient(config);
}

/**
 * Utility functions
 */
export const utils = {
  /**
   * Generate UUID v4
   */
  generateId(): string {
    return uuidv4();
  },

  /**
   * Validate jurisdiction code
   */
  validateJurisdiction(code: string): boolean {
    return /^[A-Z]{2}(-[A-Z0-9]{1,3})?$/.test(code);
  },

  /**
   * Format timestamp to ISO 8601
   */
  formatTimestamp(date: Date = new Date()): string {
    return date.toISOString();
  },

  /**
   * Calculate hash (SHA-256)
   */
  async calculateHash(data: string): Promise<string> {
    const encoder = new TextEncoder();
    const dataBuffer = encoder.encode(data);
    const hashBuffer = await crypto.subtle.digest('SHA-256', dataBuffer);
    const hashArray = Array.from(new Uint8Array(hashBuffer));
    const hashHex = hashArray.map(b => b.toString(16).padStart(2, '0')).join('');
    return `sha256:${hashHex}`;
  },

  /**
   * Verify JWT signature (placeholder - uses jose library)
   */
  async verifyJWT(token: string, publicKey: string): Promise<boolean> {
    try {
      const jwk = JSON.parse(publicKey);
      const publicKeyObject = await jose.importJWK(jwk);
      await jose.jwtVerify(token, publicKeyObject);
      return true;
    } catch {
      return false;
    }
  },
};

// Export default
export default DigitalIdentityAfterDeathClient;
