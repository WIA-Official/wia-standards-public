/**
 * WIA Secure Enclave - TypeScript SDK
 * Main Client Implementation
 *
 * @version 1.0.0
 * @license Apache-2.0
 * @author WIA Technical Committee
 *
 * Philosophy: 弘益人間 (Hongik Ingan) - Benefit All Humanity
 */

import {
  WiaSecureEnclaveConfig,
  HSMInfo,
  TEEInfo,
  CreateTEERequest,
  KeyAttributes,
  CreateKeyRequest,
  ImportKeyRequest,
  ExportKeyRequest,
  KeyRotationPolicy,
  EncryptRequest,
  EncryptResponse,
  DecryptRequest,
  DecryptResponse,
  SignRequest,
  SignResponse,
  VerifyRequest,
  VerifyResponse,
  AttestationRequest,
  AttestationResponse,
  VerifyAttestationRequest,
  SecureBootStatus,
  UpdateSecureBootPolicyRequest,
  AccessPolicy,
  CreateAccessPolicyRequest,
  AccessRequest,
  AccessDecision,
  AuditEvent,
  AuditQuery,
  AuditLogConfig,
  PaginatedResponse,
  ApiErrorResponse,
} from './types';

export * from './types';

/**
 * Main WIA Secure Enclave Client Class
 */
export class WiaSecureEnclave {
  private config: Required<WiaSecureEnclaveConfig>;
  private headers: Record<string, string>;
  private eventHandlers: Map<string, Set<Function>>;

  constructor(config: WiaSecureEnclaveConfig) {
    this.config = {
      apiUrl: config.apiUrl.replace(/\/$/, ''),
      apiKey: config.apiKey || '',
      accessToken: config.accessToken || '',
      hsmId: config.hsmId || '',
      timeout: config.timeout || 30000,
      retries: config.retries || 3,
      enableAuditLogging: config.enableAuditLogging !== false,
      headers: config.headers || {},
    };

    this.headers = {
      'Content-Type': 'application/json',
      ...this.config.headers,
    };

    if (this.config.apiKey) {
      this.headers['X-API-Key'] = this.config.apiKey;
    }

    if (this.config.accessToken) {
      this.headers['Authorization'] = `Bearer ${this.config.accessToken}`;
    }

    if (this.config.hsmId) {
      this.headers['X-HSM-ID'] = this.config.hsmId;
    }

    this.eventHandlers = new Map();
  }

  /**
   * Set access token for authenticated requests
   */
  setAccessToken(token: string): void {
    this.config.accessToken = token;
    this.headers['Authorization'] = `Bearer ${token}`;
  }

  /**
   * Set HSM ID for requests
   */
  setHSM(hsmId: string): void {
    this.config.hsmId = hsmId;
    this.headers['X-HSM-ID'] = hsmId;
  }

  /**
   * Make HTTP request with retry logic
   */
  private async request<T>(
    method: string,
    path: string,
    body?: any,
    customHeaders?: Record<string, string>
  ): Promise<T> {
    const url = `${this.config.apiUrl}${path}`;
    const headers = { ...this.headers, ...customHeaders };

    let lastError: Error | null = null;

    for (let attempt = 0; attempt < this.config.retries; attempt++) {
      try {
        const response = await fetch(url, {
          method,
          headers,
          body: body ? JSON.stringify(body) : undefined,
          signal: AbortSignal.timeout(this.config.timeout),
        });

        if (!response.ok) {
          const errorData: ApiErrorResponse = await response.json();
          throw new WiaSecureEnclaveError(
            errorData.error.code,
            errorData.error.message,
            response.status,
            errorData.error
          );
        }

        if (response.status === 204) {
          return {} as T;
        }

        return await response.json();
      } catch (error) {
        lastError = error as Error;

        if (error instanceof WiaSecureEnclaveError && error.statusCode < 500 && error.statusCode !== 429) {
          throw error;
        }

        if (attempt < this.config.retries - 1) {
          await new Promise((resolve) => setTimeout(resolve, Math.pow(2, attempt) * 1000));
        }
      }
    }

    throw lastError;
  }

  // ============================================================================
  // HSM Management Methods
  // ============================================================================

  /**
   * Get HSM information
   */
  async getHSMInfo(hsmId?: string): Promise<HSMInfo> {
    const id = hsmId || this.config.hsmId;
    return this.request<HSMInfo>('GET', `/v1/hsm/${id}`);
  }

  /**
   * List all available HSMs
   */
  async listHSMs(): Promise<HSMInfo[]> {
    const response = await this.request<{ hsms: HSMInfo[] }>('GET', '/v1/hsm');
    return response.hsms;
  }

  // ============================================================================
  // TEE Management Methods
  // ============================================================================

  /**
   * Create a new Trusted Execution Environment
   */
  async createTEE(request: CreateTEERequest): Promise<TEEInfo> {
    const result = await this.request<TEEInfo>('POST', '/v1/tee', request);
    this.emit('tee:created', result);
    return result;
  }

  /**
   * Get TEE information
   */
  async getTEE(teeId: string): Promise<TEEInfo> {
    return this.request<TEEInfo>('GET', `/v1/tee/${teeId}`);
  }

  /**
   * List all TEEs
   */
  async listTEEs(): Promise<TEEInfo[]> {
    const response = await this.request<{ tees: TEEInfo[] }>('GET', '/v1/tee');
    return response.tees;
  }

  /**
   * Terminate a TEE
   */
  async terminateTEE(teeId: string): Promise<void> {
    await this.request<void>('DELETE', `/v1/tee/${teeId}`);
    this.emit('tee:terminated', { teeId });
  }

  // ============================================================================
  // Key Management Methods
  // ============================================================================

  /**
   * Generate a new cryptographic key
   */
  async generateKey(request: CreateKeyRequest): Promise<KeyAttributes> {
    const result = await this.request<KeyAttributes>('POST', '/v1/keys', request);
    this.emit('key:created', result);
    return result;
  }

  /**
   * Import an existing key
   */
  async importKey(request: ImportKeyRequest): Promise<KeyAttributes> {
    const result = await this.request<KeyAttributes>('POST', '/v1/keys/import', request);
    this.emit('key:imported', result);
    return result;
  }

  /**
   * Export a key
   */
  async exportKey(request: ExportKeyRequest): Promise<ArrayBuffer> {
    const result = await this.request<{ key_material: string }>('POST', '/v1/keys/export', request);
    this.emit('key:exported', { keyId: request.key_id });
    return Buffer.from(result.key_material, 'base64');
  }

  /**
   * Get key attributes
   */
  async getKey(keyId: string): Promise<KeyAttributes> {
    return this.request<KeyAttributes>('GET', `/v1/keys/${keyId}`);
  }

  /**
   * List all keys
   */
  async listKeys(query?: { status?: string; key_type?: string }): Promise<KeyAttributes[]> {
    const params = new URLSearchParams();
    if (query) {
      Object.entries(query).forEach(([key, value]) => {
        if (value !== undefined) {
          params.append(key, String(value));
        }
      });
    }

    const path = params.toString() ? `/v1/keys?${params}` : '/v1/keys';
    const response = await this.request<{ keys: KeyAttributes[] }>('GET', path);
    return response.keys;
  }

  /**
   * Delete a key
   */
  async deleteKey(keyId: string): Promise<void> {
    await this.request<void>('DELETE', `/v1/keys/${keyId}`);
    this.emit('key:deleted', { keyId });
  }

  /**
   * Rotate a key
   */
  async rotateKey(keyId: string): Promise<KeyAttributes> {
    const result = await this.request<KeyAttributes>('POST', `/v1/keys/${keyId}/rotate`);
    this.emit('key:rotated', result);
    return result;
  }

  /**
   * Set key rotation policy
   */
  async setKeyRotationPolicy(keyId: string, policy: KeyRotationPolicy): Promise<void> {
    await this.request<void>('PUT', `/v1/keys/${keyId}/rotation-policy`, policy);
  }

  /**
   * Get key rotation policy
   */
  async getKeyRotationPolicy(keyId: string): Promise<KeyRotationPolicy> {
    return this.request<KeyRotationPolicy>('GET', `/v1/keys/${keyId}/rotation-policy`);
  }

  // ============================================================================
  // Cryptographic Operations
  // ============================================================================

  /**
   * Encrypt data
   */
  async encrypt(request: EncryptRequest): Promise<EncryptResponse> {
    const result = await this.request<EncryptResponse>('POST', '/v1/crypto/encrypt', request);
    this.emit('crypto:encrypt', { keyId: request.key_id });
    return result;
  }

  /**
   * Decrypt data
   */
  async decrypt(request: DecryptRequest): Promise<DecryptResponse> {
    const result = await this.request<DecryptResponse>('POST', '/v1/crypto/decrypt', request);
    this.emit('crypto:decrypt', { keyId: request.key_id });
    return result;
  }

  /**
   * Sign data
   */
  async sign(request: SignRequest): Promise<SignResponse> {
    const result = await this.request<SignResponse>('POST', '/v1/crypto/sign', request);
    this.emit('crypto:sign', { keyId: request.key_id });
    return result;
  }

  /**
   * Verify signature
   */
  async verify(request: VerifyRequest): Promise<VerifyResponse> {
    const result = await this.request<VerifyResponse>('POST', '/v1/crypto/verify', request);
    this.emit('crypto:verify', { keyId: request.key_id, valid: result.valid });
    return result;
  }

  // ============================================================================
  // Attestation Methods
  // ============================================================================

  /**
   * Request attestation from a TEE
   */
  async requestAttestation(request: AttestationRequest): Promise<AttestationResponse> {
    const result = await this.request<AttestationResponse>('POST', '/v1/attestation/request', request);
    this.emit('attestation:requested', { teeId: request.tee_id });
    return result;
  }

  /**
   * Verify attestation evidence
   */
  async verifyAttestation(request: VerifyAttestationRequest): Promise<AttestationResponse> {
    const result = await this.request<AttestationResponse>('POST', '/v1/attestation/verify', request);
    this.emit('attestation:verified', { status: result.status });
    return result;
  }

  // ============================================================================
  // Secure Boot Methods
  // ============================================================================

  /**
   * Get secure boot status
   */
  async getSecureBootStatus(): Promise<SecureBootStatus> {
    return this.request<SecureBootStatus>('GET', '/v1/secure-boot/status');
  }

  /**
   * Update secure boot policy
   */
  async updateSecureBootPolicy(request: UpdateSecureBootPolicyRequest): Promise<void> {
    await this.request<void>('PUT', '/v1/secure-boot/policy', request);
    this.emit('secure-boot:policy-updated', request);
  }

  /**
   * Verify boot measurements
   */
  async verifyBootMeasurements(): Promise<{ verified: boolean; violations?: string[] }> {
    return this.request<{ verified: boolean; violations?: string[] }>('POST', '/v1/secure-boot/verify');
  }

  // ============================================================================
  // Access Control Methods
  // ============================================================================

  /**
   * Create access policy
   */
  async createAccessPolicy(request: CreateAccessPolicyRequest): Promise<AccessPolicy> {
    const result = await this.request<AccessPolicy>('POST', '/v1/access/policies', request);
    this.emit('access:policy-created', result);
    return result;
  }

  /**
   * List access policies
   */
  async listAccessPolicies(): Promise<AccessPolicy[]> {
    const response = await this.request<{ policies: AccessPolicy[] }>('GET', '/v1/access/policies');
    return response.policies;
  }

  /**
   * Get access policy
   */
  async getAccessPolicy(policyId: string): Promise<AccessPolicy> {
    return this.request<AccessPolicy>('GET', `/v1/access/policies/${policyId}`);
  }

  /**
   * Update access policy
   */
  async updateAccessPolicy(policyId: string, updates: Partial<CreateAccessPolicyRequest>): Promise<AccessPolicy> {
    return this.request<AccessPolicy>('PATCH', `/v1/access/policies/${policyId}`, updates);
  }

  /**
   * Delete access policy
   */
  async deleteAccessPolicy(policyId: string): Promise<void> {
    await this.request<void>('DELETE', `/v1/access/policies/${policyId}`);
    this.emit('access:policy-deleted', { policyId });
  }

  /**
   * Check access permission
   */
  async checkAccess(request: AccessRequest): Promise<AccessDecision> {
    const result = await this.request<AccessDecision>('POST', '/v1/access/check', request);
    this.emit('access:checked', { allowed: result.allowed });
    return result;
  }

  // ============================================================================
  // Audit Logging Methods
  // ============================================================================

  /**
   * Query audit logs
   */
  async queryAuditLogs(query: AuditQuery): Promise<PaginatedResponse<AuditEvent>> {
    const params = new URLSearchParams();
    Object.entries(query).forEach(([key, value]) => {
      if (value !== undefined) {
        params.append(key, String(value));
      }
    });

    const path = `/v1/audit/events?${params}`;
    return this.request<PaginatedResponse<AuditEvent>>('GET', path);
  }

  /**
   * Get audit log configuration
   */
  async getAuditConfig(): Promise<AuditLogConfig> {
    return this.request<AuditLogConfig>('GET', '/v1/audit/config');
  }

  /**
   * Update audit log configuration
   */
  async updateAuditConfig(config: Partial<AuditLogConfig>): Promise<AuditLogConfig> {
    return this.request<AuditLogConfig>('PUT', '/v1/audit/config', config);
  }

  // ============================================================================
  // Event Handling
  // ============================================================================

  /**
   * Register event handler
   */
  on(event: string, handler: Function): void {
    if (!this.eventHandlers.has(event)) {
      this.eventHandlers.set(event, new Set());
    }
    this.eventHandlers.get(event)!.add(handler);
  }

  /**
   * Unregister event handler
   */
  off(event: string, handler: Function): void {
    const handlers = this.eventHandlers.get(event);
    if (handlers) {
      handlers.delete(handler);
    }
  }

  /**
   * Emit event
   */
  private emit(event: string, data?: any): void {
    const handlers = this.eventHandlers.get(event);
    if (handlers) {
      handlers.forEach((handler) => {
        try {
          handler(data);
        } catch (error) {
          console.error(`Error in event handler for ${event}:`, error);
        }
      });
    }
  }
}

/**
 * Custom Error Class for WIA Secure Enclave API Errors
 */
export class WiaSecureEnclaveError extends Error {
  public readonly code: string;
  public readonly statusCode: number;
  public readonly details?: any;

  constructor(code: string, message: string, statusCode: number, details?: any) {
    super(message);
    this.name = 'WiaSecureEnclaveError';
    this.code = code;
    this.statusCode = statusCode;
    this.details = details;

    if (Error.captureStackTrace) {
      Error.captureStackTrace(this, WiaSecureEnclaveError);
    }
  }
}

/**
 * Factory function to create WiaSecureEnclave client
 */
export function createSecureEnclaveClient(config: WiaSecureEnclaveConfig): WiaSecureEnclave {
  return new WiaSecureEnclave(config);
}

/**
 * Default export
 */
export default WiaSecureEnclave;
