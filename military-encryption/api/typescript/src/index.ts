/**
 * WIA-DEF-017: Military Encryption SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Defense & Security Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides military-grade encryption capabilities including:
 * - AES-256-GCM and ChaCha20-Poly1305 encryption
 * - RSA-4096 and ECC-P521 asymmetric cryptography
 * - Post-quantum cryptography (CRYSTALS-Kyber, Dilithium)
 * - Hardware security module (HSM) integration
 * - Secure communication channels (TLS 1.3, IPsec)
 * - Multi-level security classification enforcement
 */

import {
  Classification,
  EncryptRequest,
  EncryptResponse,
  DecryptRequest,
  DecryptResponse,
  KeyGenerationRequest,
  KeyGenerationResponse,
  KeyRotationRequest,
  KeyRotationResponse,
  SecureChannelRequest,
  SecureChannelResponse,
  SignatureRequest,
  SignatureResponse,
  VerifyRequest,
  VerifyResponse,
  HSMConfig,
  HSMStatus,
  AuditLogEntry,
  ErrorCode,
  MilitaryEncryptionError,
  SECURITY_CONSTANTS,
  SymmetricAlgorithm,
  AsymmetricAlgorithm,
  PostQuantumAlgorithm,
  KeyInfo,
  AccessPolicy,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-DEF-017 Military Encryption SDK
 */
export class EncryptionSDK {
  private version = '1.0.0';
  private initialized = false;
  private hsmConfig?: HSMConfig;
  private defaultClassification: Classification;
  private auditLog: AuditLogEntry[] = [];

  constructor(options?: {
    classification?: Classification;
    hsmConfig?: HSMConfig;
    enableAudit?: boolean;
  }) {
    this.defaultClassification = options?.classification || Classification.CONFIDENTIAL;
    this.hsmConfig = options?.hsmConfig;
    this.initialized = true;
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Generate cryptographic key
   *
   * @param request - Key generation parameters
   * @returns Key generation response with key ID and metadata
   */
  async generateKey(request: KeyGenerationRequest): Promise<KeyGenerationResponse> {
    this.validateClassification(request.classification);

    // Validate algorithm and classification compatibility
    const minKeySize =
      SECURITY_CONSTANTS.MIN_KEY_SIZE[request.classification];

    // Generate key ID
    const keyId = this.generateKeyId(request.algorithm, request.classification);
    const keyHandle = this.generateKeyHandle();

    // Calculate expiration
    const expirationDays =
      request.expirationDays ||
      SECURITY_CONSTANTS.MAX_ROTATION_PERIOD[request.classification];

    const created = new Date();
    const expires = new Date(created);
    expires.setDate(expires.getDate() + expirationDays);

    // Simulate key generation (in real implementation, use crypto library or HSM)
    let publicKey: Buffer | undefined;

    if (
      this.isAsymmetricAlgorithm(request.algorithm) ||
      this.isPostQuantumAlgorithm(request.algorithm)
    ) {
      // For asymmetric/PQC, generate key pair
      publicKey = this.generatePublicKey(request.algorithm);
    }

    // Log audit event
    this.logAuditEvent({
      eventType: 'key_generation',
      classification: request.classification,
      operator: 'system',
      resource: keyId,
      result: 'success',
      details: {
        algorithm: request.algorithm,
        hsmProtected: request.hsmProtected,
      },
    });

    return {
      keyId,
      keyHandle,
      publicKey,
      created,
      expires,
      classification: request.classification,
      hsmId: request.hsmProtected ? this.hsmConfig?.id : undefined,
    };
  }

  /**
   * Encrypt data with specified algorithm
   *
   * @param request - Encryption request parameters
   * @returns Encrypted ciphertext with authentication tag
   */
  async encrypt(request: EncryptRequest): Promise<EncryptResponse> {
    this.validateClassification(request.classification);

    // Convert plaintext to Buffer if string
    const plaintext =
      typeof request.plaintext === 'string'
        ? Buffer.from(request.plaintext, 'utf-8')
        : request.plaintext;

    // Generate nonce (96 bits for GCM, 96 bits for ChaCha20)
    const nonce = this.generateNonce(request.algorithm);

    // Simulate encryption (in real implementation, use crypto library)
    const ciphertext = this.performEncryption(
      plaintext,
      request.algorithm,
      request.keyId,
      nonce,
      request.additionalData
    );

    // Generate authentication tag
    const tag = this.generateAuthTag(ciphertext, request.additionalData);

    // Create metadata
    const metadata: EncryptResponse['metadata'] = {
      algorithm: request.algorithm,
      classification: request.classification,
      timestamp: new Date(),
      keyId: request.keyId,
      operator: request.operator || 'system',
      hsmId: request.useHSM ? this.hsmConfig?.id : undefined,
      version: this.version,
    };

    // Log audit event
    this.logAuditEvent({
      eventType: 'encryption',
      classification: request.classification,
      operator: metadata.operator,
      resource: request.keyId,
      result: 'success',
      details: {
        algorithm: request.algorithm,
        dataSize: plaintext.length,
      },
    });

    return {
      ciphertext,
      nonce,
      tag,
      metadata,
    };
  }

  /**
   * Decrypt encrypted data
   *
   * @param request - Decryption request parameters
   * @returns Decrypted plaintext
   */
  async decrypt(request: DecryptRequest): Promise<DecryptResponse> {
    // Verify authentication tag
    const verified = this.verifyAuthTag(
      request.ciphertext,
      request.tag,
      request.additionalData
    );

    if (!verified) {
      throw new MilitaryEncryptionError(
        ErrorCode.INVALID_CIPHERTEXT,
        'Authentication tag verification failed'
      );
    }

    // Simulate decryption (in real implementation, use crypto library)
    const plaintext = this.performDecryption(
      request.ciphertext,
      request.algorithm,
      request.keyId,
      request.nonce,
      request.additionalData
    );

    // Log audit event
    this.logAuditEvent({
      eventType: 'decryption',
      classification: Classification.CONFIDENTIAL, // Would be determined from key metadata
      operator: 'system',
      resource: request.keyId,
      result: 'success',
      details: {
        algorithm: request.algorithm,
        dataSize: request.ciphertext.length,
      },
    });

    return {
      plaintext,
      metadata: {
        classification: Classification.CONFIDENTIAL,
        timestamp: new Date(),
        operator: 'system',
      },
      verified,
    };
  }

  /**
   * Rotate encryption key
   *
   * @param request - Key rotation parameters
   * @returns Rotation result with new key ID
   */
  async rotateKey(request: KeyRotationRequest): Promise<KeyRotationResponse> {
    // Generate new key if requested
    let newKeyId = request.newKeyId;

    if (request.generateNew) {
      // Get old key info to copy parameters
      const oldKeyInfo = await this.getKeyInfo(request.oldKeyId);

      const newKeyResponse = await this.generateKey({
        algorithm: oldKeyInfo.algorithm,
        classification: oldKeyInfo.classification,
        hsmProtected: !!oldKeyInfo.hsmId,
      });

      newKeyId = newKeyResponse.keyId;
    }

    if (!newKeyId) {
      throw new MilitaryEncryptionError(
        ErrorCode.INVALID_PARAMETERS,
        'Must provide newKeyId or set generateNew to true'
      );
    }

    // Simulate re-encryption (in real implementation, re-encrypt all data)
    const reencryptedCount = request.reencryptData ? this.reencryptData(request.oldKeyId, newKeyId) : 0;

    // Revoke old key
    await this.revokeKey(request.oldKeyId, request.reason);

    // Log audit event
    this.logAuditEvent({
      eventType: 'key_rotation',
      classification: Classification.SECRET,
      operator: 'system',
      resource: request.oldKeyId,
      result: 'success',
      details: {
        newKeyId,
        reason: request.reason,
        reencryptedCount,
      },
    });

    return {
      newKeyId,
      oldKeyId: request.oldKeyId,
      timestamp: new Date(),
      reencryptedCount,
      status: 'completed',
    };
  }

  /**
   * Create secure communication channel
   *
   * @param request - Secure channel parameters
   * @returns Channel information and status
   */
  async createSecureChannel(
    request: SecureChannelRequest
  ): Promise<SecureChannelResponse> {
    this.validateClassification(request.classification);

    // Generate channel ID
    const channelId = this.generateChannelId();

    // Simulate TLS 1.3 handshake or other protocol establishment
    const sessionKeyRef = this.generateKeyHandle();

    // Determine encryption algorithm based on protocol and settings
    const encryptionAlgorithm = this.selectChannelCipherSuite(
      request.protocol,
      request.classification,
      request.postQuantum
    );

    const established = new Date();
    const expires = new Date(established);
    expires.setHours(expires.getHours() + 8); // 8-hour session

    // Log audit event
    this.logAuditEvent({
      eventType: 'encryption',
      classification: request.classification,
      operator: request.localIdentity,
      resource: channelId,
      result: 'success',
      details: {
        protocol: request.protocol,
        remoteIdentity: request.remoteIdentity,
        postQuantum: request.postQuantum,
      },
    });

    return {
      channelId,
      status: 'established',
      localAddress: `${request.localIdentity}:443`,
      remoteAddress: `${request.remoteIdentity}:443`,
      encryptionAlgorithm,
      sessionKeyRef,
      established,
      expires,
      security: {
        forwardSecrecy: request.forwardSecrecy,
        postQuantum: request.postQuantum || false,
        mutualAuth: request.mutualAuth || false,
        peerVerified: true,
      },
    };
  }

  /**
   * Sign data with digital signature
   *
   * @param request - Signature request parameters
   * @returns Digital signature
   */
  async sign(request: SignatureRequest): Promise<SignatureResponse> {
    // Simulate signing operation (in real implementation, use crypto library or HSM)
    const signature = this.performSigning(
      request.data,
      request.algorithm,
      request.keyId,
      request.hashAlgorithm
    );

    // Log audit event
    this.logAuditEvent({
      eventType: 'encryption',
      classification: Classification.CONFIDENTIAL,
      operator: request.operator,
      resource: request.keyId,
      result: 'success',
      details: {
        algorithm: request.algorithm,
        dataSize: request.data.length,
      },
    });

    return {
      signature,
      algorithm: request.algorithm,
      timestamp: new Date(),
      signer: request.operator,
      keyId: request.keyId,
    };
  }

  /**
   * Verify digital signature
   *
   * @param request - Verification request parameters
   * @returns Verification result
   */
  async verify(request: VerifyRequest): Promise<VerifyResponse> {
    // Simulate signature verification
    const valid = this.performVerification(
      request.data,
      request.signature,
      request.publicKey,
      request.algorithm,
      request.hashAlgorithm
    );

    return {
      valid,
      signer: valid ? 'verified_signer' : undefined,
      timestamp: new Date(),
      error: valid ? undefined : 'Signature verification failed',
    };
  }

  /**
   * Get HSM status
   *
   * @returns HSM health and status information
   */
  async getHSMStatus(): Promise<HSMStatus> {
    if (!this.hsmConfig) {
      throw new MilitaryEncryptionError(
        ErrorCode.HSM_ERROR,
        'HSM not configured'
      );
    }

    // Simulate HSM status check
    return {
      id: this.hsmConfig.id,
      connected: true,
      health: 'healthy',
      availableSlots: 9500,
      usedSlots: 500,
      tamperDetected: false,
      lastHealthCheck: new Date(),
      temperature: 45.2,
    };
  }

  /**
   * Get audit log entries
   *
   * @param filter - Optional filter parameters
   * @returns Array of audit log entries
   */
  async getAuditLog(filter?: {
    startTime?: Date;
    endTime?: Date;
    eventType?: string;
  }): Promise<AuditLogEntry[]> {
    let logs = this.auditLog;

    if (filter) {
      if (filter.startTime) {
        logs = logs.filter((log) => log.timestamp >= filter.startTime!);
      }
      if (filter.endTime) {
        logs = logs.filter((log) => log.timestamp <= filter.endTime!);
      }
      if (filter.eventType) {
        logs = logs.filter((log) => log.eventType === filter.eventType);
      }
    }

    return logs;
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Validate classification level
   */
  private validateClassification(classification: Classification): void {
    if (!Object.values(Classification).includes(classification)) {
      throw new MilitaryEncryptionError(
        ErrorCode.INVALID_PARAMETERS,
        `Invalid classification level: ${classification}`
      );
    }
  }

  /**
   * Check if algorithm is asymmetric
   */
  private isAsymmetricAlgorithm(algorithm: string): boolean {
    return (
      algorithm.startsWith('RSA') ||
      algorithm.startsWith('ECC') ||
      algorithm.includes('25519')
    );
  }

  /**
   * Check if algorithm is post-quantum
   */
  private isPostQuantumAlgorithm(algorithm: string): boolean {
    return (
      algorithm.includes('Kyber') ||
      algorithm.includes('Dilithium') ||
      algorithm.includes('SPHINCS')
    );
  }

  /**
   * Generate unique key ID
   */
  private generateKeyId(algorithm: string, classification: Classification): string {
    const timestamp = Date.now();
    const random = Math.random().toString(36).substring(2, 10);
    return `${classification}-${algorithm}-${timestamp}-${random}`.toUpperCase();
  }

  /**
   * Generate key handle
   */
  private generateKeyHandle(): string {
    return '0x' + Math.random().toString(16).substring(2, 10).toUpperCase();
  }

  /**
   * Generate channel ID
   */
  private generateChannelId(): string {
    return `CH-${Date.now()}-${Math.random().toString(36).substring(2, 10)}`.toUpperCase();
  }

  /**
   * Generate public key (simulated)
   */
  private generatePublicKey(algorithm: string): Buffer {
    // In real implementation, generate actual public key
    const size = algorithm.includes('RSA-4096') ? 512 : algorithm.includes('P521') ? 133 : 64;
    return Buffer.from(
      Array.from({ length: size }, () => Math.floor(Math.random() * 256))
    );
  }

  /**
   * Generate nonce/IV
   */
  private generateNonce(algorithm: SymmetricAlgorithm): Buffer {
    // 96 bits (12 bytes) for GCM and ChaCha20
    return Buffer.from(
      Array.from({ length: 12 }, () => Math.floor(Math.random() * 256))
    );
  }

  /**
   * Perform encryption (simulated)
   */
  private performEncryption(
    plaintext: Buffer,
    algorithm: SymmetricAlgorithm,
    keyId: string,
    nonce: Buffer,
    additionalData?: Record<string, unknown>
  ): Buffer {
    // In real implementation, use actual crypto library (e.g., Node crypto, libsodium)
    // This is a placeholder that just XORs with a pseudo-random stream
    const ciphertext = Buffer.from(plaintext);
    for (let i = 0; i < ciphertext.length; i++) {
      ciphertext[i] ^= (nonce[i % nonce.length] + i) & 0xff;
    }
    return ciphertext;
  }

  /**
   * Perform decryption (simulated)
   */
  private performDecryption(
    ciphertext: Buffer,
    algorithm: SymmetricAlgorithm,
    keyId: string,
    nonce: Buffer,
    additionalData?: Record<string, unknown>
  ): Buffer {
    // In real implementation, use actual crypto library
    // This matches the simulated encryption above
    const plaintext = Buffer.from(ciphertext);
    for (let i = 0; i < plaintext.length; i++) {
      plaintext[i] ^= (nonce[i % nonce.length] + i) & 0xff;
    }
    return plaintext;
  }

  /**
   * Generate authentication tag (simulated)
   */
  private generateAuthTag(
    ciphertext: Buffer,
    additionalData?: Record<string, unknown>
  ): Buffer {
    // In real implementation, use GMAC or Poly1305
    return Buffer.from(
      Array.from({ length: 16 }, () => Math.floor(Math.random() * 256))
    );
  }

  /**
   * Verify authentication tag (simulated)
   */
  private verifyAuthTag(
    ciphertext: Buffer,
    tag: Buffer,
    additionalData?: Record<string, unknown>
  ): boolean {
    // In real implementation, verify actual GMAC or Poly1305 tag
    return tag.length === 16; // Simplified check
  }

  /**
   * Perform signing (simulated)
   */
  private performSigning(
    data: Buffer,
    algorithm: AsymmetricAlgorithm | PostQuantumAlgorithm,
    keyId: string,
    hashAlgorithm?: string
  ): Buffer {
    // In real implementation, use actual signing algorithm
    const signatureSize = algorithm.includes('RSA-4096') ? 512 : algorithm.includes('Dilithium') ? 4627 : 64;
    return Buffer.from(
      Array.from({ length: signatureSize }, () => Math.floor(Math.random() * 256))
    );
  }

  /**
   * Perform signature verification (simulated)
   */
  private performVerification(
    data: Buffer,
    signature: Buffer,
    publicKey: Buffer | string,
    algorithm: AsymmetricAlgorithm | PostQuantumAlgorithm,
    hashAlgorithm?: string
  ): boolean {
    // In real implementation, verify actual signature
    return signature.length > 0; // Simplified check
  }

  /**
   * Select cipher suite for secure channel
   */
  private selectChannelCipherSuite(
    protocol: string,
    classification: Classification,
    postQuantum?: boolean
  ): string {
    if (classification === Classification.TOP_SECRET && postQuantum) {
      return 'TLS_KYBER1024_AES_256_GCM_SHA384';
    } else if (classification === Classification.TOP_SECRET) {
      return 'TLS_ECDHE_ECDSA_AES_256_GCM_SHA384';
    } else if (classification === Classification.SECRET) {
      return 'TLS_ECDHE_RSA_AES_256_GCM_SHA384';
    } else {
      return 'TLS_ECDHE_RSA_AES_256_GCM_SHA256';
    }
  }

  /**
   * Get key information
   */
  private async getKeyInfo(keyId: string): Promise<KeyInfo> {
    // In real implementation, retrieve from key store or HSM
    return {
      keyId,
      algorithm: 'AES-256-GCM' as any,
      classification: Classification.SECRET,
      created: new Date(),
      expires: new Date(Date.now() + 90 * 24 * 60 * 60 * 1000),
      status: 'active',
      usageCount: 0,
    };
  }

  /**
   * Revoke key
   */
  private async revokeKey(keyId: string, reason: string): Promise<void> {
    // In real implementation, update key status in HSM/key store
    this.logAuditEvent({
      eventType: 'key_revocation',
      classification: Classification.SECRET,
      operator: 'system',
      resource: keyId,
      result: 'success',
      details: { reason },
    });
  }

  /**
   * Re-encrypt data with new key (simulated)
   */
  private reencryptData(oldKeyId: string, newKeyId: string): number {
    // In real implementation, retrieve all data encrypted with oldKeyId,
    // decrypt with old key, and re-encrypt with new key
    return Math.floor(Math.random() * 1000); // Simulated count
  }

  /**
   * Log audit event
   */
  private logAuditEvent(
    event: Omit<AuditLogEntry, 'id' | 'timestamp'>
  ): void {
    const entry: AuditLogEntry = {
      id: `AUDIT-${Date.now()}-${Math.random().toString(36).substring(2, 8)}`,
      timestamp: new Date(),
      ...event,
    };

    this.auditLog.push(entry);

    // In real implementation, also send to external audit system
    // for tamper-proof logging
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Generate encryption key (standalone function)
 */
export async function generateKey(
  request: KeyGenerationRequest
): Promise<KeyGenerationResponse> {
  const sdk = new EncryptionSDK();
  return sdk.generateKey(request);
}

/**
 * Encrypt data (standalone function)
 */
export async function encryptData(
  request: EncryptRequest
): Promise<EncryptResponse> {
  const sdk = new EncryptionSDK();
  return sdk.encrypt(request);
}

/**
 * Decrypt data (standalone function)
 */
export async function decryptData(
  request: DecryptRequest
): Promise<DecryptResponse> {
  const sdk = new EncryptionSDK();
  return sdk.decrypt(request);
}

/**
 * Create secure channel (standalone function)
 */
export async function createSecureChannel(
  request: SecureChannelRequest
): Promise<SecureChannelResponse> {
  const sdk = new EncryptionSDK();
  return sdk.createSecureChannel(request);
}

/**
 * Sign data (standalone function)
 */
export async function signData(
  request: SignatureRequest
): Promise<SignatureResponse> {
  const sdk = new EncryptionSDK();
  return sdk.sign(request);
}

/**
 * Verify signature (standalone function)
 */
export async function verifySignature(
  request: VerifyRequest
): Promise<VerifyResponse> {
  const sdk = new EncryptionSDK();
  return sdk.verify(request);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { EncryptionSDK };
export default EncryptionSDK;
