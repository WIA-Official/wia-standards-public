/**
 * WIA-EDU-011 Digital Credential Standard - TypeScript SDK
 * Version: 2.0.0
 * Philosophy: 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';
import type {
  SDKConfig,
  VerifiableCredential,
  IssuanceRequest,
  IssuanceResponse,
  VerificationRequest,
  VerificationResponse,
  RevocationRequest,
  RevocationResponse,
  BatchIssuanceRequest,
  BatchIssuanceResponse,
  CredentialAnalytics,
  TrustRegistryEntry,
  DIDDocument,
  CredentialWallet,
  VerifiablePresentation,
} from './types';

export * from './types';

/**
 * Main SDK Client for WIA-EDU-011 Digital Credentials
 */
export class DigitalCredentialClient {
  private axios: AxiosInstance;
  private config: SDKConfig;

  constructor(config: SDKConfig) {
    this.config = {
      baseURL: config.baseURL || 'https://api.wiastandards.com/v1',
      ...config,
    };

    this.axios = axios.create({
      baseURL: this.config.baseURL,
      headers: {
        'Content-Type': 'application/json',
        'X-API-Key': this.config.apiKey || '',
      },
    });
  }

  /**
   * Issue a new credential
   */
  async issueCredential(
    request: IssuanceRequest
  ): Promise<IssuanceResponse> {
    const response = await this.axios.post('/credentials/issue', request);
    return response.data;
  }

  /**
   * Verify a credential
   */
  async verifyCredential(
    request: VerificationRequest
  ): Promise<VerificationResponse> {
    const response = await this.axios.post('/credentials/verify', request);
    return response.data;
  }

  /**
   * Revoke a credential
   */
  async revokeCredential(
    request: RevocationRequest
  ): Promise<RevocationResponse> {
    const response = await this.axios.post('/credentials/revoke', request);
    return response.data;
  }

  /**
   * Get credential status
   */
  async getCredentialStatus(credentialId: string): Promise<any> {
    const response = await this.axios.get(`/credentials/${credentialId}/status`);
    return response.data;
  }

  /**
   * Batch issue credentials
   */
  async batchIssueCredentials(
    request: BatchIssuanceRequest
  ): Promise<BatchIssuanceResponse> {
    const response = await this.axios.post('/credentials/batch/issue', request);
    return response.data;
  }

  /**
   * Get credential by ID
   */
  async getCredential(credentialId: string): Promise<VerifiableCredential> {
    const response = await this.axios.get(`/credentials/${credentialId}`);
    return response.data;
  }

  /**
   * List credentials for an issuer
   */
  async listCredentials(params?: {
    limit?: number;
    offset?: number;
    status?: string;
  }): Promise<VerifiableCredential[]> {
    const response = await this.axios.get('/credentials', { params });
    return response.data;
  }

  /**
   * Get analytics data
   */
  async getAnalytics(params?: {
    startDate?: string;
    endDate?: string;
  }): Promise<CredentialAnalytics> {
    const response = await this.axios.get('/analytics', { params });
    return response.data;
  }
}

/**
 * Credential Issuer Client
 */
export class CredentialIssuer {
  private client: DigitalCredentialClient;

  constructor(config: SDKConfig) {
    if (!config.did || !config.privateKey) {
      throw new Error('DID and private key required for issuer');
    }
    this.client = new DigitalCredentialClient(config);
  }

  /**
   * Issue a credential
   */
  async issueCredential(
    request: IssuanceRequest
  ): Promise<IssuanceResponse> {
    return this.client.issueCredential(request);
  }

  /**
   * Issue multiple credentials in batch
   */
  async batchIssue(
    credentials: IssuanceRequest[]
  ): Promise<BatchIssuanceResponse> {
    return this.client.batchIssueCredentials({
      credentials,
      blockchainBatch: true,
    });
  }

  /**
   * Revoke a credential
   */
  async revokeCredential(
    credentialId: string,
    reason: string
  ): Promise<RevocationResponse> {
    return this.client.revokeCredential({ credentialId, reason });
  }

  /**
   * Get issuance statistics
   */
  async getStatistics(): Promise<CredentialAnalytics> {
    return this.client.getAnalytics();
  }
}

/**
 * Credential Verifier Client
 */
export class CredentialVerifier {
  private client: DigitalCredentialClient;

  constructor(config: SDKConfig) {
    this.client = new DigitalCredentialClient(config);
  }

  /**
   * Verify a credential
   */
  async verify(
    credential: VerifiableCredential | string,
    options?: {
      checkRevocation?: boolean;
      checkExpiration?: boolean;
      trustRegistry?: string[];
    }
  ): Promise<VerificationResponse> {
    return this.client.verifyCredential({
      credential,
      ...options,
    });
  }

  /**
   * Verify multiple credentials
   */
  async verifyBatch(
    credentials: (VerifiableCredential | string)[]
  ): Promise<VerificationResponse[]> {
    const promises = credentials.map((credential) =>
      this.verify(credential)
    );
    return Promise.all(promises);
  }

  /**
   * Quick verification (signature only)
   */
  async quickVerify(
    credential: VerifiableCredential | string
  ): Promise<boolean> {
    const result = await this.verify(credential, {
      checkRevocation: false,
      checkExpiration: false,
    });
    return result.verified;
  }
}

/**
 * DID Manager for creating and resolving DIDs
 */
export class DIDManager {
  private axios: AxiosInstance;

  constructor(baseURL?: string) {
    this.axios = axios.create({
      baseURL: baseURL || 'https://api.wiastandards.com/v1',
    });
  }

  /**
   * Create a new DID
   */
  async createDID(params: {
    method?: string;
    type?: string;
    organization?: string;
  }): Promise<{ did: string; privateKey: string; document: DIDDocument }> {
    const response = await this.axios.post('/did/create', params);
    return response.data;
  }

  /**
   * Resolve a DID to its DID Document
   */
  async resolveDID(did: string): Promise<DIDDocument> {
    const response = await this.axios.get(`/did/${encodeURIComponent(did)}`);
    return response.data;
  }

  /**
   * Register DID in trust registry
   */
  async registerDID(did: string, metadata: any): Promise<void> {
    await this.axios.post('/did/register', { did, metadata });
  }

  /**
   * Update DID document
   */
  async updateDID(did: string, document: DIDDocument): Promise<void> {
    await this.axios.put(`/did/${encodeURIComponent(did)}`, document);
  }
}

/**
 * Trust Registry Client
 */
export class TrustRegistry {
  private axios: AxiosInstance;

  constructor(baseURL?: string) {
    this.axios = axios.create({
      baseURL: baseURL || 'https://api.wiastandards.com/v1',
    });
  }

  /**
   * Lookup an issuer in trust registry
   */
  async lookupIssuer(did: string): Promise<TrustRegistryEntry> {
    const response = await this.axios.get(
      `/registry/issuers/${encodeURIComponent(did)}`
    );
    return response.data;
  }

  /**
   * List all trusted issuers
   */
  async listIssuers(params?: {
    country?: string;
    type?: string;
    limit?: number;
  }): Promise<TrustRegistryEntry[]> {
    const response = await this.axios.get('/registry/issuers', { params });
    return response.data;
  }

  /**
   * Verify if an issuer is trusted
   */
  async isTrusted(did: string): Promise<boolean> {
    try {
      const entry = await this.lookupIssuer(did);
      return entry.status === 'active' && entry.accredited;
    } catch {
      return false;
    }
  }
}

/**
 * Simple in-memory credential wallet implementation
 */
export class SimpleWallet implements CredentialWallet {
  private credentials: Map<string, VerifiableCredential> = new Map();

  async addCredential(credential: VerifiableCredential): Promise<void> {
    this.credentials.set(credential.id, credential);
  }

  async getCredential(id: string): Promise<VerifiableCredential | null> {
    return this.credentials.get(id) || null;
  }

  async listCredentials(): Promise<VerifiableCredential[]> {
    return Array.from(this.credentials.values());
  }

  async removeCredential(id: string): Promise<void> {
    this.credentials.delete(id);
  }

  async presentCredential(
    id: string,
    selectiveDisclosure?: string[]
  ): Promise<VerifiablePresentation> {
    const credential = await this.getCredential(id);
    if (!credential) {
      throw new Error('Credential not found');
    }

    // In real implementation, would apply selective disclosure
    return {
      '@context': ['https://www.w3.org/2018/credentials/v1'],
      type: ['VerifiablePresentation'],
      verifiableCredential: [credential],
      proof: {
        type: 'Ed25519Signature2020',
        created: new Date().toISOString(),
        verificationMethod: '',
        proofPurpose: 'authentication',
        proofValue: '',
      },
    };
  }
}

/**
 * Utility functions
 */
export const utils = {
  /**
   * Generate QR code data URL from credential
   */
  generateQRCode(credential: VerifiableCredential): string {
    const data = JSON.stringify(credential);
    // In real implementation, would generate actual QR code
    return `data:text/plain;base64,${Buffer.from(data).toString('base64')}`;
  },

  /**
   * Validate credential structure
   */
  validateCredential(credential: any): boolean {
    return !!(
      credential['@context'] &&
      credential.id &&
      credential.type &&
      credential.issuer &&
      credential.issuanceDate &&
      credential.credentialSubject &&
      credential.proof
    );
  },

  /**
   * Extract achievement from credential
   */
  extractAchievement(credential: VerifiableCredential): any {
    return credential.credentialSubject.achievement;
  },

  /**
   * Format credential for display
   */
  formatForDisplay(credential: VerifiableCredential): string {
    const subject = credential.credentialSubject;
    const achievement = subject.achievement;
    if (!achievement) return 'Unknown Credential';

    return `${achievement.name} - ${credential.issuer.name} (${new Date(
      credential.issuanceDate
    ).getFullYear()})`;
  },
};

/**
 * Default export
 */
export default {
  DigitalCredentialClient,
  CredentialIssuer,
  CredentialVerifier,
  DIDManager,
  TrustRegistry,
  SimpleWallet,
  utils,
};
