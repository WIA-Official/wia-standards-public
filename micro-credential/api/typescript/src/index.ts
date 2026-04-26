/**
 * WIA-EDU-012 Micro-Credential Standard - TypeScript SDK
 * Version: 2.0.0
 * Philosophy: 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';
import { v4 as uuidv4 } from 'uuid';
import * as jsonld from 'jsonld';

import {
  WIAClientConfig,
  CredentialIssuanceRequest,
  WIAMicroCredential,
  VerificationResult,
  BatchIssuanceRequest,
  BatchIssuanceResponse,
  CredentialSearchParams,
  ComprehensiveLearnerRecord,
  WebhookConfig,
  APIError,
  WIAEvidence,
  WIACompetency,
  OpenBadgeCredential
} from './types';

/**
 * WIA Micro-Credential SDK Client
 */
export class MicroCredentialClient {
  private client: AxiosInstance;
  private apiKey: string;
  private baseURL: string;

  constructor(config: WIAClientConfig) {
    this.apiKey = config.apiKey;
    this.baseURL = config.baseURL || this.getEnvironmentURL(config.environment);

    this.client = axios.create({
      baseURL: this.baseURL,
      timeout: config.timeout || 30000,
      headers: {
        'Authorization': `Bearer ${this.apiKey}`,
        'Content-Type': 'application/json',
        'X-WIA-SDK-Version': '2.0.0'
      }
    });

    // Setup retry logic
    if (config.retries) {
      this.setupRetries(config.retries);
    }
  }

  /**
   * Get API URL based on environment
   */
  private getEnvironmentURL(env?: string): string {
    switch (env) {
      case 'production':
        return 'https://api.wia.org/edu-012/v1';
      case 'staging':
        return 'https://staging-api.wia.org/edu-012/v1';
      case 'development':
        return 'http://localhost:3000/edu-012/v1';
      default:
        return 'https://api.wia.org/edu-012/v1';
    }
  }

  /**
   * Setup automatic retries for failed requests
   */
  private setupRetries(maxRetries: number): void {
    this.client.interceptors.response.use(
      response => response,
      async error => {
        const config = error.config;
        if (!config || !config.retryCount) {
          config.retryCount = 0;
        }

        if (config.retryCount < maxRetries && error.response?.status >= 500) {
          config.retryCount += 1;
          await new Promise(resolve => setTimeout(resolve, 1000 * config.retryCount));
          return this.client(config);
        }

        return Promise.reject(error);
      }
    );
  }

  /**
   * Issue a new micro-credential
   */
  async issue(request: CredentialIssuanceRequest): Promise<WIAMicroCredential> {
    try {
      const credential = this.buildCredential(request);
      const response = await this.client.post('/credentials', credential);
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Issue multiple credentials in batch
   */
  async issueBatch(request: BatchIssuanceRequest): Promise<BatchIssuanceResponse> {
    try {
      const response = await this.client.post('/credentials/batch', request);
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Verify a credential
   */
  async verify(credentialId: string): Promise<VerificationResult> {
    try {
      const response = await this.client.get(`/credentials/${credentialId}/verify`);
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Verify multiple credentials in batch
   */
  async verifyBatch(credentialIds: string[]): Promise<VerificationResult[]> {
    try {
      const response = await this.client.post('/credentials/verify/batch', { credentialIds });
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Retrieve a credential by ID
   */
  async get(credentialId: string): Promise<WIAMicroCredential> {
    try {
      const response = await this.client.get(`/credentials/${credentialId}`);
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Search for credentials
   */
  async search(params: CredentialSearchParams): Promise<WIAMicroCredential[]> {
    try {
      const response = await this.client.get('/credentials', { params });
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Revoke a credential
   */
  async revoke(credentialId: string, reason?: string): Promise<void> {
    try {
      await this.client.post(`/credentials/${credentialId}/revoke`, { reason });
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Export credentials as Comprehensive Learner Record (CLR)
   */
  async exportCLR(recipientId: string): Promise<ComprehensiveLearnerRecord> {
    try {
      const response = await this.client.get('/credentials/export/clr', {
        params: { recipientId }
      });
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Export credential as Open Badges format
   */
  async exportOpenBadge(credentialId: string): Promise<OpenBadgeCredential> {
    try {
      const response = await this.client.get(`/credentials/${credentialId}/export/openbadges`);
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Subscribe to webhook events
   */
  async subscribeWebhook(config: WebhookConfig): Promise<{ webhookId: string }> {
    try {
      const response = await this.client.post('/webhooks', config);
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Get credential statistics
   */
  async getStatistics(): Promise<any> {
    try {
      const response = await this.client.get('/credentials/stats');
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Build credential object from issuance request
   */
  private buildCredential(request: CredentialIssuanceRequest): WIAMicroCredential {
    const credentialId = 'urn:uuid:' + uuidv4();
    const issuanceDate = request.issuanceDate || new Date().toISOString();

    return {
      '@context': [
        'https://w3id.org/openbadges/v3',
        'https://wia.org/context/v1'
      ],
      type: 'OpenBadgeCredential',
      id: credentialId,
      issuer: {
        id: 'https://wia.org/issuers/default',
        type: 'Profile',
        name: 'WIA Issuer'
      },
      issuanceDate,
      ...(request.expirationDate && { expirationDate: request.expirationDate }),
      credentialSubject: {
        id: request.recipientId,
        type: 'AchievementSubject',
        achievement: {
          id: request.achievementId,
          type: 'Achievement',
          name: 'Achievement',
          description: 'WIA Micro-Credential',
          criteria: {
            narrative: 'Demonstrated competency'
          }
        }
      },
      ...(request.competencies && { 'wia:competencies': request.competencies }),
      ...(request.evidence && { 'wia:evidence': request.evidence }),
      ...(request.privacy && { 'wia:privacy': request.privacy }),
      'wia:version': '2.0'
    };
  }

  /**
   * Handle API errors
   */
  private handleError(error: any): Error {
    if (error.response) {
      const apiError: APIError = {
        error: error.response.data?.error || 'API Error',
        message: error.response.data?.message || error.message,
        statusCode: error.response.status,
        details: error.response.data?.details
      };
      return new Error(JSON.stringify(apiError));
    }
    return error;
  }
}

/**
 * Utility Functions
 */

/**
 * Validate credential against Open Badges 3.0 schema
 */
export async function validateCredential(credential: WIAMicroCredential): Promise<boolean> {
  try {
    // Basic validation
    if (!credential['@context']) return false;
    if (credential.type !== 'OpenBadgeCredential') return false;
    if (!credential.id) return false;
    if (!credential.issuer) return false;
    if (!credential.credentialSubject) return false;

    // JSON-LD validation
    await jsonld.expand(credential);
    return true;
  } catch (error) {
    return false;
  }
}

/**
 * Create a competency mapping
 */
export function createCompetency(
  framework: string,
  competencyId: string,
  competencyName: string,
  level: string
): WIACompetency {
  return {
    framework,
    competencyId,
    competencyName,
    proficiencyLevel: level as any,
    assessmentDate: new Date().toISOString()
  };
}

/**
 * Create an evidence attachment
 */
export function createEvidence(
  type: string,
  description: string,
  url?: string
): WIAEvidence {
  return {
    type: type as any,
    description,
    ...(url && { url }),
    date: new Date().toISOString()
  };
}

/**
 * Generate credential ID
 */
export function generateCredentialId(): string {
  return 'urn:uuid:' + uuidv4();
}

// Export all types
export * from './types';

// Default export
export default MicroCredentialClient;

// 弘益人間 - Benefit All Humanity
