/**
 * WIA-DEF-017: Military Encryption - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Defense & Security Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Classification and Security Types
// ============================================================================

/**
 * Security classification levels
 */
export enum Classification {
  TOP_SECRET = 'TOP_SECRET',
  SECRET = 'SECRET',
  CONFIDENTIAL = 'CONFIDENTIAL',
  RESTRICTED = 'RESTRICTED',
  UNCLASSIFIED = 'UNCLASSIFIED',
}

/**
 * Security clearance levels
 */
export interface SecurityClearance {
  level: Classification;
  compartments: string[];
  expiresAt: Date;
  grantedBy: string;
  specialAccess?: string[];
}

// ============================================================================
// Cryptographic Algorithm Types
// ============================================================================

/**
 * Symmetric encryption algorithms
 */
export type SymmetricAlgorithm =
  | 'AES-128-GCM'
  | 'AES-256-GCM'
  | 'AES-256-CTR'
  | 'ChaCha20-Poly1305'
  | 'Camellia-256-GCM';

/**
 * Asymmetric encryption algorithms
 */
export type AsymmetricAlgorithm =
  | 'RSA-2048'
  | 'RSA-4096'
  | 'ECC-P256'
  | 'ECC-P384'
  | 'ECC-P521'
  | 'X25519'
  | 'Ed25519';

/**
 * Post-quantum cryptographic algorithms
 */
export type PostQuantumAlgorithm =
  | 'CRYSTALS-Kyber-512'
  | 'CRYSTALS-Kyber-768'
  | 'CRYSTALS-Kyber-1024'
  | 'CRYSTALS-Dilithium2'
  | 'CRYSTALS-Dilithium3'
  | 'CRYSTALS-Dilithium5'
  | 'SPHINCS+-128f'
  | 'SPHINCS+-256f'
  | 'NTRU-HPS-2048-509';

/**
 * Hash functions
 */
export type HashAlgorithm =
  | 'SHA-256'
  | 'SHA-384'
  | 'SHA-512'
  | 'SHA3-256'
  | 'SHA3-512'
  | 'BLAKE3';

/**
 * All supported algorithms
 */
export type Algorithm =
  | SymmetricAlgorithm
  | AsymmetricAlgorithm
  | PostQuantumAlgorithm;

// ============================================================================
// Encryption Operations
// ============================================================================

/**
 * Encryption request parameters
 */
export interface EncryptRequest {
  /** Data to encrypt */
  plaintext: Buffer | string;

  /** Encryption algorithm */
  algorithm: SymmetricAlgorithm;

  /** Key identifier or handle */
  keyId: string;

  /** Classification level of data */
  classification: Classification;

  /** Additional authenticated data (AAD) */
  additionalData?: Record<string, unknown>;

  /** Use HSM for encryption */
  useHSM?: boolean;

  /** Operator performing encryption */
  operator?: string;
}

/**
 * Encryption response
 */
export interface EncryptResponse {
  /** Encrypted ciphertext */
  ciphertext: Buffer;

  /** Nonce/IV used for encryption */
  nonce: Buffer;

  /** Authentication tag */
  tag: Buffer;

  /** Metadata about encryption operation */
  metadata: EncryptionMetadata;
}

/**
 * Encryption metadata
 */
export interface EncryptionMetadata {
  /** Algorithm used */
  algorithm: string;

  /** Classification level */
  classification: Classification;

  /** Timestamp of encryption */
  timestamp: Date;

  /** Key identifier */
  keyId: string;

  /** Operator who performed encryption */
  operator: string;

  /** HSM identifier (if used) */
  hsmId?: string;

  /** Version of encryption standard */
  version: string;
}

/**
 * Decryption request parameters
 */
export interface DecryptRequest {
  /** Encrypted ciphertext */
  ciphertext: Buffer;

  /** Nonce/IV used during encryption */
  nonce: Buffer;

  /** Authentication tag */
  tag: Buffer;

  /** Key identifier or handle */
  keyId: string;

  /** Algorithm used for encryption */
  algorithm: SymmetricAlgorithm;

  /** Additional authenticated data (must match encryption) */
  additionalData?: Record<string, unknown>;

  /** Use HSM for decryption */
  useHSM?: boolean;
}

/**
 * Decryption response
 */
export interface DecryptResponse {
  /** Decrypted plaintext */
  plaintext: Buffer;

  /** Metadata about decryption operation */
  metadata: {
    classification: Classification;
    timestamp: Date;
    operator: string;
  };

  /** Verification status */
  verified: boolean;
}

// ============================================================================
// Key Management
// ============================================================================

/**
 * Key generation parameters
 */
export interface KeyGenerationRequest {
  /** Cryptographic algorithm */
  algorithm: Algorithm;

  /** Classification level */
  classification: Classification;

  /** Store key in HSM */
  hsmProtected: boolean;

  /** Key access policy */
  accessPolicy?: AccessPolicy;

  /** Key expiration period in days */
  expirationDays?: number;

  /** Additional key metadata */
  metadata?: Record<string, unknown>;
}

/**
 * Key generation response
 */
export interface KeyGenerationResponse {
  /** Unique key identifier */
  keyId: string;

  /** Key handle for operations */
  keyHandle: string;

  /** Public key (for asymmetric algorithms) */
  publicKey?: Buffer;

  /** Key creation timestamp */
  created: Date;

  /** Key expiration timestamp */
  expires: Date;

  /** Classification level */
  classification: Classification;

  /** HSM identifier (if stored in HSM) */
  hsmId?: string;
}

/**
 * Access policy for keys
 */
export interface AccessPolicy {
  /** List of authorized user IDs */
  authorizedUsers: string[];

  /** Number of approvals required */
  requiredApprovals: number;

  /** Time restrictions for key usage */
  timeRestrictions?: 'business_hours' | 'always' | 'custom';

  /** Location restrictions */
  locationRestrictions?: string[];

  /** Allowed operations */
  allowedOperations: KeyOperation[];

  /** Maximum usage count */
  maxUsageCount?: number;
}

/**
 * Key operations
 */
export type KeyOperation =
  | 'encrypt'
  | 'decrypt'
  | 'sign'
  | 'verify'
  | 'wrap'
  | 'unwrap'
  | 'derive'
  | 'export';

/**
 * Key information
 */
export interface KeyInfo {
  /** Key identifier */
  keyId: string;

  /** Algorithm */
  algorithm: Algorithm;

  /** Classification */
  classification: Classification;

  /** Creation date */
  created: Date;

  /** Expiration date */
  expires: Date;

  /** Current status */
  status: 'active' | 'expired' | 'revoked' | 'compromised';

  /** Usage count */
  usageCount: number;

  /** Last used timestamp */
  lastUsed?: Date;

  /** HSM location */
  hsmId?: string;
}

/**
 * Key rotation parameters
 */
export interface KeyRotationRequest {
  /** Current key ID */
  oldKeyId: string;

  /** Generate new key with same parameters */
  generateNew?: boolean;

  /** New key ID (if not generating) */
  newKeyId?: string;

  /** Re-encrypt all data */
  reencryptData?: boolean;

  /** Rotation reason */
  reason: 'scheduled' | 'compromise' | 'policy' | 'manual';
}

/**
 * Key rotation response
 */
export interface KeyRotationResponse {
  /** New key ID */
  newKeyId: string;

  /** Old key ID (revoked) */
  oldKeyId: string;

  /** Rotation timestamp */
  timestamp: Date;

  /** Re-encrypted data count */
  reencryptedCount?: number;

  /** Rotation status */
  status: 'completed' | 'in_progress' | 'failed';
}

// ============================================================================
// Hardware Security Module (HSM)
// ============================================================================

/**
 * HSM configuration
 */
export interface HSMConfig {
  /** HSM identifier */
  id: string;

  /** HSM type/vendor */
  type: 'Thales' | 'Gemalto' | 'Utimaco' | 'AWS-CloudHSM' | 'Generic';

  /** FIPS compliance level */
  fipsLevel: '140-2-L3' | '140-2-L4' | '140-3-L3' | '140-3-L4';

  /** Connection endpoint */
  endpoint: string;

  /** Authentication credentials */
  credentials: {
    partition: string;
    username: string;
    password?: string;
    certificatePath?: string;
  };

  /** Network configuration */
  network?: {
    ipAddress: string;
    port: number;
    useTLS: boolean;
  };
}

/**
 * HSM status
 */
export interface HSMStatus {
  /** HSM identifier */
  id: string;

  /** Connection status */
  connected: boolean;

  /** Health status */
  health: 'healthy' | 'degraded' | 'critical' | 'offline';

  /** Available key slots */
  availableSlots: number;

  /** Used key slots */
  usedSlots: number;

  /** Tamper status */
  tamperDetected: boolean;

  /** Last health check */
  lastHealthCheck: Date;

  /** Temperature (if available) */
  temperature?: number;
}

/**
 * HSM operation request
 */
export interface HSMOperationRequest {
  /** HSM identifier */
  hsmId: string;

  /** Operation type */
  operation: 'generate' | 'encrypt' | 'decrypt' | 'sign' | 'verify' | 'delete';

  /** Operation parameters */
  parameters: Record<string, unknown>;

  /** Operator authorization */
  authorization: {
    operatorId: string;
    authToken: string;
  };
}

// ============================================================================
// Secure Communication Channels
// ============================================================================

/**
 * Secure channel protocols
 */
export type ChannelProtocol = 'TLS-1.3' | 'DTLS-1.3' | 'IPsec' | 'WireGuard' | 'Custom';

/**
 * Secure channel request
 */
export interface SecureChannelRequest {
  /** Local identity */
  localIdentity: string;

  /** Remote identity */
  remoteIdentity: string;

  /** Protocol to use */
  protocol: ChannelProtocol;

  /** Cipher suite */
  cipherSuite?: string;

  /** Enable post-quantum cryptography */
  postQuantum?: boolean;

  /** Require forward secrecy */
  forwardSecrecy: boolean;

  /** Mutual authentication required */
  mutualAuth?: boolean;

  /** Classification level */
  classification: Classification;
}

/**
 * Secure channel response
 */
export interface SecureChannelResponse {
  /** Channel identifier */
  channelId: string;

  /** Connection status */
  status: 'established' | 'failed' | 'pending' | 'closed';

  /** Local address */
  localAddress: string;

  /** Remote address */
  remoteAddress: string;

  /** Negotiated encryption algorithm */
  encryptionAlgorithm: string;

  /** Session key reference (not actual key) */
  sessionKeyRef: string;

  /** Established timestamp */
  established?: Date;

  /** Expiration timestamp */
  expires?: Date;

  /** Security properties */
  security: {
    forwardSecrecy: boolean;
    postQuantum: boolean;
    mutualAuth: boolean;
    peerVerified: boolean;
  };
}

/**
 * Channel message
 */
export interface ChannelMessage {
  /** Channel identifier */
  channelId: string;

  /** Message payload */
  payload: Buffer;

  /** Message sequence number */
  sequence: number;

  /** Timestamp */
  timestamp: Date;

  /** Classification */
  classification: Classification;
}

// ============================================================================
// Digital Signatures
// ============================================================================

/**
 * Signature request
 */
export interface SignatureRequest {
  /** Data to sign */
  data: Buffer;

  /** Signing key ID */
  keyId: string;

  /** Signature algorithm */
  algorithm: AsymmetricAlgorithm | PostQuantumAlgorithm;

  /** Hash algorithm (for classical signatures) */
  hashAlgorithm?: HashAlgorithm;

  /** Use HSM for signing */
  useHSM?: boolean;

  /** Operator performing signature */
  operator: string;
}

/**
 * Signature response
 */
export interface SignatureResponse {
  /** Digital signature */
  signature: Buffer;

  /** Algorithm used */
  algorithm: string;

  /** Timestamp of signing */
  timestamp: Date;

  /** Signer identity */
  signer: string;

  /** Key ID used */
  keyId: string;
}

/**
 * Signature verification request
 */
export interface VerifyRequest {
  /** Original data */
  data: Buffer;

  /** Signature to verify */
  signature: Buffer;

  /** Public key or key ID */
  publicKey: Buffer | string;

  /** Algorithm used */
  algorithm: AsymmetricAlgorithm | PostQuantumAlgorithm;

  /** Hash algorithm (for classical signatures) */
  hashAlgorithm?: HashAlgorithm;
}

/**
 * Signature verification response
 */
export interface VerifyResponse {
  /** Verification result */
  valid: boolean;

  /** Signer identity (if verified) */
  signer?: string;

  /** Timestamp of verification */
  timestamp: Date;

  /** Error message (if invalid) */
  error?: string;
}

// ============================================================================
// Key Derivation
// ============================================================================

/**
 * Key derivation request
 */
export interface KeyDerivationRequest {
  /** Master key ID */
  masterKeyId: string;

  /** Derivation algorithm */
  algorithm: 'HKDF-SHA256' | 'HKDF-SHA512' | 'PBKDF2' | 'Argon2id';

  /** Salt value */
  salt?: Buffer;

  /** Context information */
  info: string;

  /** Desired output length in bits */
  outputLength: number;

  /** Additional parameters (for PBKDF2, Argon2id) */
  parameters?: {
    iterations?: number;
    memory?: number;
    parallelism?: number;
  };
}

/**
 * Key derivation response
 */
export interface KeyDerivationResponse {
  /** Derived key ID */
  derivedKeyId: string;

  /** Derived key (if not stored in HSM) */
  derivedKey?: Buffer;

  /** Derivation timestamp */
  timestamp: Date;

  /** Classification (inherited from master) */
  classification: Classification;
}

// ============================================================================
// Certificate Management
// ============================================================================

/**
 * X.509 certificate
 */
export interface Certificate {
  /** Certificate in PEM or DER format */
  data: Buffer;

  /** Certificate format */
  format: 'PEM' | 'DER';

  /** Subject DN */
  subject: string;

  /** Issuer DN */
  issuer: string;

  /** Serial number */
  serialNumber: string;

  /** Valid from date */
  validFrom: Date;

  /** Valid to date */
  validTo: Date;

  /** Public key */
  publicKey: Buffer;

  /** Signature algorithm */
  signatureAlgorithm: string;

  /** Extensions */
  extensions?: Record<string, unknown>;
}

/**
 * Certificate signing request
 */
export interface CertificateSigningRequest {
  /** Subject information */
  subject: {
    commonName: string;
    organization?: string;
    organizationalUnit?: string;
    country?: string;
  };

  /** Key ID for certificate */
  keyId: string;

  /** Validity period in days */
  validityDays: number;

  /** Certificate extensions */
  extensions?: Record<string, unknown>;

  /** Classification level */
  classification: Classification;
}

// ============================================================================
// Audit and Logging
// ============================================================================

/**
 * Audit log entry
 */
export interface AuditLogEntry {
  /** Unique log entry ID */
  id: string;

  /** Timestamp */
  timestamp: Date;

  /** Event type */
  eventType:
    | 'key_generation'
    | 'encryption'
    | 'decryption'
    | 'key_rotation'
    | 'key_revocation'
    | 'authentication'
    | 'authorization_failure'
    | 'hsm_access'
    | 'configuration_change';

  /** Classification level */
  classification: Classification;

  /** Operator/user */
  operator: string;

  /** Resource accessed */
  resource: string;

  /** Operation result */
  result: 'success' | 'failure' | 'warning';

  /** Additional details */
  details: Record<string, unknown>;

  /** Source IP address */
  sourceIp?: string;

  /** Geographic location */
  location?: string;
}

/**
 * Audit query parameters
 */
export interface AuditQuery {
  /** Start timestamp */
  startTime: Date;

  /** End timestamp */
  endTime: Date;

  /** Filter by event type */
  eventType?: string[];

  /** Filter by operator */
  operator?: string;

  /** Filter by classification */
  classification?: Classification[];

  /** Filter by result */
  result?: ('success' | 'failure' | 'warning')[];

  /** Maximum results */
  limit?: number;

  /** Offset for pagination */
  offset?: number;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-DEF-017 error codes
 */
export enum ErrorCode {
  KEY_NOT_FOUND = 'E001',
  INSUFFICIENT_PERMISSIONS = 'E002',
  HSM_ERROR = 'E003',
  INVALID_CIPHERTEXT = 'E004',
  AUTHENTICATION_FAILED = 'E005',
  KEY_EXPIRED = 'E006',
  CLASSIFICATION_MISMATCH = 'E007',
  CRYPTOGRAPHIC_FAILURE = 'E008',
  INVALID_ALGORITHM = 'E009',
  INVALID_PARAMETERS = 'E010',
  TAMPER_DETECTED = 'E011',
  POLICY_VIOLATION = 'E012',
}

/**
 * Military encryption error
 */
export class MilitaryEncryptionError extends Error {
  constructor(
    public code: ErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'MilitaryEncryptionError';
  }
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Security constants and limits
 */
export const SECURITY_CONSTANTS = {
  /** Minimum key sizes by classification */
  MIN_KEY_SIZE: {
    TOP_SECRET: 256,
    SECRET: 256,
    CONFIDENTIAL: 256,
    RESTRICTED: 128,
    UNCLASSIFIED: 128,
  },

  /** Maximum key rotation periods (days) */
  MAX_ROTATION_PERIOD: {
    TOP_SECRET: 30,
    SECRET: 90,
    CONFIDENTIAL: 180,
    RESTRICTED: 365,
    UNCLASSIFIED: 365,
  },

  /** Required authentication factors */
  AUTH_FACTORS_REQUIRED: {
    TOP_SECRET: 3,
    SECRET: 2,
    CONFIDENTIAL: 2,
    RESTRICTED: 1,
    UNCLASSIFIED: 1,
  },

  /** HSM requirements */
  HSM_FIPS_LEVEL: {
    TOP_SECRET: '140-3-L4',
    SECRET: '140-2-L3',
    CONFIDENTIAL: '140-2-L2',
    RESTRICTED: 'none',
    UNCLASSIFIED: 'none',
  },
} as const;

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Result type for operations that can fail
 */
export type Result<T, E = MilitaryEncryptionError> =
  | { success: true; data: T }
  | { success: false; error: E };

/**
 * Async result type
 */
export type AsyncResult<T, E = MilitaryEncryptionError> = Promise<Result<T, E>>;

// ============================================================================
// Export All Types
// ============================================================================

export type {
  // Core types
  SecurityClearance,

  // Algorithms
  Algorithm,
  SymmetricAlgorithm,
  AsymmetricAlgorithm,
  PostQuantumAlgorithm,
  HashAlgorithm,

  // Encryption
  EncryptRequest,
  EncryptResponse,
  EncryptionMetadata,
  DecryptRequest,
  DecryptResponse,

  // Key management
  KeyGenerationRequest,
  KeyGenerationResponse,
  AccessPolicy,
  KeyOperation,
  KeyInfo,
  KeyRotationRequest,
  KeyRotationResponse,

  // HSM
  HSMConfig,
  HSMStatus,
  HSMOperationRequest,

  // Channels
  ChannelProtocol,
  SecureChannelRequest,
  SecureChannelResponse,
  ChannelMessage,

  // Signatures
  SignatureRequest,
  SignatureResponse,
  VerifyRequest,
  VerifyResponse,

  // Key derivation
  KeyDerivationRequest,
  KeyDerivationResponse,

  // Certificates
  Certificate,
  CertificateSigningRequest,

  // Audit
  AuditLogEntry,
  AuditQuery,
};

export { Classification, ErrorCode, MilitaryEncryptionError, SECURITY_CONSTANTS };
