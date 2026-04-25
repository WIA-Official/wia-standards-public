/**
 * WIA Digital Identity Standard (WIA-FIN-010)
 * TypeScript SDK - Main Implementation
 *
 * @version 1.0.0
 * @license MIT
 */

import {
  WIAIdentityConfig,
  DIDParams,
  DIDDocument,
  DIDResolutionResult,
  IssueCredentialParams,
  VerifiableCredential,
  VerificationResult,
  PresentationParams,
  VerifiablePresentation,
  BiometricEnrollParams,
  BiometricVerifyParams,
  BiometricVerifyResult,
  AgeProofParams,
  ZeroKnowledgeProof,
  KYCParams,
  KYCSession,
  KYCStatusResult,
  WalletParams,
  WalletInfo,
  StoredCredential,
  WIAIdentityError,
} from './types';

/**
 * Main SDK class for WIA Digital Identity Standard
 */
export class WIAIdentity {
  private config: WIAIdentityConfig;
  private baseUrl: string;

  constructor(config: WIAIdentityConfig = {}) {
    this.config = {
      baseUrl: 'https://api.identity.wia.live',
      resolver: 'https://resolver.identity.wia.live',
      debug: false,
      ...config,
    };
    this.baseUrl = this.config.baseUrl!;
  }

  // ========================================================================
  // DID Operations
  // ========================================================================

  /**
   * Create a new Decentralized Identifier (DID)
   */
  async createDID(params: DIDParams): Promise<DIDDocument> {
    const response = await this.post('/did/create', params);
    return response.didDocument;
  }

  /**
   * Resolve a DID to its DID Document
   */
  async resolveDID(did: string): Promise<DIDResolutionResult> {
    const resolverUrl = `${this.config.resolver}/1.0/identifiers/${encodeURIComponent(did)}`;
    const response = await fetch(resolverUrl);

    if (!response.ok) {
      throw new WIAIdentityError(
        `Failed to resolve DID: ${did}`,
        'E2001',
        response.status
      );
    }

    return await response.json();
  }

  /**
   * Update a DID Document
   */
  async updateDID(did: string, updates: Partial<DIDDocument>): Promise<DIDDocument> {
    const response = await this.put(`/did/${encodeURIComponent(did)}`, updates);
    return response.didDocument;
  }

  /**
   * Deactivate a DID
   */
  async deactivateDID(did: string): Promise<void> {
    await this.delete(`/did/${encodeURIComponent(did)}`);
  }

  // ========================================================================
  // Credential Operations
  // ========================================================================

  /**
   * Issue a Verifiable Credential
   */
  async issueCredential(params: IssueCredentialParams): Promise<VerifiableCredential> {
    const response = await this.post('/credentials/issue', params);
    return response.verifiableCredential;
  }

  /**
   * Verify a Verifiable Credential
   */
  async verifyCredential(
    credential: VerifiableCredential,
    options?: {
      checkStatus?: boolean;
      checkExpiration?: boolean;
      checkIssuerTrust?: boolean;
    }
  ): Promise<VerificationResult> {
    const response = await this.post('/credentials/verify', {
      verifiableCredential: credential,
      options: options || {},
    });
    return response;
  }

  /**
   * Revoke a Verifiable Credential
   */
  async revokeCredential(credentialId: string): Promise<void> {
    await this.post('/credentials/revoke', { credentialId });
  }

  /**
   * Check revocation status of a credential
   */
  async checkRevocationStatus(credential: VerifiableCredential): Promise<boolean> {
    if (!credential.credentialStatus) {
      return false; // Not revoked if no status
    }

    const response = await this.get(
      `/credentials/status/${credential.credentialStatus.id}`
    );
    return response.revoked;
  }

  // ========================================================================
  // Presentation Operations
  // ========================================================================

  /**
   * Create a Verifiable Presentation
   */
  async createPresentation(params: PresentationParams): Promise<VerifiablePresentation> {
    const response = await this.post('/presentations/create', params);
    return response.verifiablePresentation;
  }

  /**
   * Verify a Verifiable Presentation
   */
  async verifyPresentation(
    presentation: VerifiablePresentation,
    challenge: string,
    domain?: string
  ): Promise<VerificationResult> {
    const response = await this.post('/presentations/verify', {
      verifiablePresentation: presentation,
      challenge,
      domain,
    });
    return response;
  }

  // ========================================================================
  // Biometric Operations
  // ========================================================================

  /**
   * Enroll biometric data for a DID
   */
  async enrollBiometric(params: BiometricEnrollParams): Promise<{ templateId: string }> {
    const response = await this.post('/biometrics/enroll', params);
    return { templateId: response.templateId };
  }

  /**
   * Verify biometric data against enrolled template
   */
  async verifyBiometric(params: BiometricVerifyParams): Promise<BiometricVerifyResult> {
    const response = await this.post('/biometrics/verify', params);
    return {
      verified: response.verified,
      confidence: response.confidence,
      matchScore: response.matchScore,
    };
  }

  // ========================================================================
  // Zero-Knowledge Proof Operations
  // ========================================================================

  /**
   * Generate a zero-knowledge age proof
   */
  async generateAgeProof(params: AgeProofParams): Promise<ZeroKnowledgeProof> {
    const response = await this.post('/zkp/age/generate', params);
    return response.proof;
  }

  /**
   * Verify a zero-knowledge proof
   */
  async verifyZKProof(proof: ZeroKnowledgeProof): Promise<boolean> {
    const response = await this.post('/zkp/verify', { proof });
    return response.verified;
  }

  // ========================================================================
  // eKYC Operations
  // ========================================================================

  /**
   * Initiate KYC verification process
   */
  async initiateKYC(params: KYCParams): Promise<KYCSession> {
    const response = await this.post('/kyc/initiate', params);
    return {
      sessionId: response.sessionId,
      uploadUrl: response.uploadUrl,
      expiresAt: response.expiresAt,
    };
  }

  /**
   * Submit KYC documents
   */
  async submitKYCDocuments(
    sessionId: string,
    documents: { document: File; selfie: File }
  ): Promise<void> {
    const formData = new FormData();
    formData.append('document', documents.document);
    formData.append('selfie', documents.selfie);

    await fetch(`${this.baseUrl}/kyc/${sessionId}/documents`, {
      method: 'POST',
      headers: this.getHeaders(true), // Skip Content-Type for FormData
      body: formData,
    });
  }

  /**
   * Get KYC verification status
   */
  async getKYCStatus(sessionId: string): Promise<KYCStatusResult> {
    const response = await this.get(`/kyc/${sessionId}/status`);
    return response;
  }

  // ========================================================================
  // Wallet Operations
  // ========================================================================

  /**
   * Create a new digital identity wallet
   */
  async createWallet(params: WalletParams): Promise<WalletInfo> {
    const response = await this.post('/wallet/create', params);
    return {
      walletId: response.walletId,
      recoveryPhrase: response.recoveryPhrase,
      backupUrl: response.backupUrl,
    };
  }

  /**
   * Store a credential in wallet
   */
  async storeCredential(
    walletId: string,
    credential: VerifiableCredential,
    tags: string[] = []
  ): Promise<{ credentialId: string }> {
    const response = await this.post(`/wallet/${walletId}/credentials`, {
      credential,
      tags,
    });
    return { credentialId: response.credentialId };
  }

  /**
   * List credentials in wallet
   */
  async listCredentials(
    walletId: string,
    filters?: { type?: string; issuer?: string; tags?: string[] }
  ): Promise<StoredCredential[]> {
    const queryParams = new URLSearchParams();
    if (filters?.type) queryParams.append('type', filters.type);
    if (filters?.issuer) queryParams.append('issuer', filters.issuer);
    if (filters?.tags) queryParams.append('tags', filters.tags.join(','));

    const url = `/wallet/${walletId}/credentials${queryParams.toString() ? '?' + queryParams : ''}`;
    const response = await this.get(url);
    return response.credentials;
  }

  /**
   * Delete a credential from wallet
   */
  async deleteCredential(walletId: string, credentialId: string): Promise<void> {
    await this.delete(`/wallet/${walletId}/credentials/${credentialId}`);
  }

  // ========================================================================
  // Helper Methods
  // ========================================================================

  private async post(endpoint: string, data: any): Promise<any> {
    return this.request('POST', endpoint, data);
  }

  private async get(endpoint: string): Promise<any> {
    return this.request('GET', endpoint);
  }

  private async put(endpoint: string, data: any): Promise<any> {
    return this.request('PUT', endpoint, data);
  }

  private async delete(endpoint: string): Promise<any> {
    return this.request('DELETE', endpoint);
  }

  private async request(method: string, endpoint: string, data?: any): Promise<any> {
    const url = `${this.baseUrl}${endpoint}`;

    if (this.config.debug) {
      console.log(`[WIA Identity] ${method} ${url}`, data);
    }

    const response = await fetch(url, {
      method,
      headers: this.getHeaders(),
      body: data ? JSON.stringify(data) : undefined,
    });

    const result = await response.json();

    if (!response.ok) {
      throw new WIAIdentityError(
        result.error_description || 'API request failed',
        result.error_code || 'UNKNOWN',
        response.status
      );
    }

    return result;
  }

  private getHeaders(skipContentType = false): Record<string, string> {
    const headers: Record<string, string> = {};

    if (!skipContentType) {
      headers['Content-Type'] = 'application/json';
    }

    if (this.config.apiKey) {
      headers['Authorization'] = `Bearer ${this.config.apiKey}`;
    }

    return headers;
  }
}

// ========================================================================
// Re-export types
// ========================================================================

export * from './types';

// ========================================================================
// Default export
// ========================================================================

export default WIAIdentity;
