/**
 * WIA Digital Time Capsule Standard - Type Definitions
 * Version: 1.0
 *
 * 弘益人間 · Benefit All Humanity
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Unique identifier for a time capsule
 * Format: TC-YYYY-NNN-XXX
 */
export type CapsuleId = string;

/**
 * Content identifier
 */
export type ContentId = string;

/**
 * Status of a time capsule
 */
export enum CapsuleStatus {
  DRAFT = 'draft',
  SEALING = 'sealing',
  SEALED = 'sealed',
  UNLOCKED = 'unlocked',
  ARCHIVED = 'archived'
}

/**
 * Content type classification
 */
export enum ContentType {
  IMAGE = 'image',
  VIDEO = 'video',
  DOCUMENT = 'document',
  AUDIO = 'audio',
  OTHER = 'other'
}

/**
 * Storage backend types
 */
export enum StorageType {
  IPFS = 'ipfs',
  S3 = 's3',
  AZURE = 'azure',
  GCS = 'gcs',
  LOCAL = 'local'
}

/**
 * Blockchain network types
 */
export enum BlockchainNetwork {
  ETHEREUM = 'ethereum',
  POLYGON = 'polygon',
  ARBITRUM = 'arbitrum',
  OPTIMISM = 'optimism'
}

// ============================================================================
// Time Capsule
// ============================================================================

/**
 * Creator information
 */
export interface Creator {
  /** Full name of the creator */
  name: string;
  /** Email address */
  email: string;
  /** Ed25519 public key (base64 encoded) */
  publicKey?: string;
  /** Additional metadata */
  metadata?: Record<string, any>;
}

/**
 * Encryption configuration
 */
export interface EncryptionConfig {
  /** Whether encryption is enabled */
  enabled: boolean;
  /** Encryption algorithm (default: AES-256-GCM) */
  algorithm: 'AES-256-GCM' | 'ChaCha20-Poly1305';
  /** Encrypted master key (base64 encoded) */
  encryptedKey?: string;
  /** Initialization vector (base64 encoded) */
  iv?: string;
}

/**
 * Digital signature
 */
export interface DigitalSignature {
  /** Signature algorithm */
  algorithm: 'Ed25519' | 'ECDSA';
  /** Public key (base64 encoded) */
  publicKey: string;
  /** Signature (base64 encoded) */
  signature: string;
  /** Timestamp when signed */
  timestamp: Date;
}

/**
 * Security configuration
 */
export interface SecurityConfig {
  /** Whether content is encrypted */
  encrypted: boolean;
  /** Encryption algorithm */
  algorithm: string;
  /** Digital signature */
  signature?: DigitalSignature;
  /** Content hash (SHA3-256) */
  contentHash: string;
  /** Authentication tag for AES-GCM */
  authTag?: string;
}

/**
 * Early access configuration
 */
export interface EarlyAccessConfig {
  /** Whether early access is enabled */
  enabled: boolean;
  /** Whether approval is required */
  requiresApproval: boolean;
  /** List of approver email addresses */
  approvers: string[];
  /** Reason for early access (if requested) */
  reason?: string;
  /** Whether to maintain audit log */
  auditLog: boolean;
}

/**
 * Multi-party authorization configuration
 */
export interface MultiPartyAuth {
  /** Whether multi-party auth is required */
  required: boolean;
  /** Minimum number of parties required (M) */
  threshold: number;
  /** Total number of parties (N) */
  totalParties: number;
  /** Party identifiers (public keys or emails) */
  parties: string[];
}

/**
 * Access control configuration
 */
export interface AccessControl {
  /** Unlock date and time */
  unlockDate: Date;
  /** Timezone for unlock */
  timezone?: string;
  /** Whether to automatically unlock */
  automaticUnlock: boolean;
  /** Early access configuration */
  earlyAccess?: EarlyAccessConfig;
  /** Multi-party authorization */
  multiParty?: MultiPartyAuth;
}

/**
 * Content metadata
 */
export interface ContentMetadata {
  /** Description of the content */
  description?: string;
  /** Tags for categorization */
  tags?: string[];
  /** Creation timestamp */
  created?: Date;
  /** Original filename */
  originalFilename?: string;
  /** MIME type */
  mimeType?: string;
  /** Custom metadata */
  custom?: Record<string, any>;
}

/**
 * Individual content item
 */
export interface Content {
  /** Unique content identifier */
  id: ContentId;
  /** Filename */
  filename: string;
  /** Content type */
  type: ContentType;
  /** File size in bytes */
  size: number;
  /** File format/extension */
  format: string;
  /** SHA3-256 checksum */
  checksum: string;
  /** Content metadata */
  metadata: ContentMetadata;
  /** Whether this content is encrypted */
  encrypted: boolean;
  /** Encrypted data (base64 encoded) */
  encryptedData?: string;
  /** Storage location */
  storageLocation?: string;
}

/**
 * Storage location information
 */
export interface StorageLocation {
  /** Storage type */
  type: StorageType;
  /** Location URI or hash */
  location: string;
  /** Timestamp when stored */
  timestamp: Date;
  /** Additional storage metadata */
  metadata?: Record<string, any>;
}

/**
 * Blockchain information
 */
export interface BlockchainInfo {
  /** Blockchain network */
  network: BlockchainNetwork;
  /** Transaction hash */
  transactionHash: string;
  /** Block number */
  blockNumber: number;
  /** Block timestamp */
  timestamp: Date;
  /** Contract address (if applicable) */
  contractAddress?: string;
}

/**
 * Format migration rule
 */
export interface MigrationRule {
  /** Source format */
  sourceFormat: string;
  /** Target format */
  targetFormat: string;
  /** Whether to preserve original */
  preserveOriginal: boolean;
  /** Quality setting */
  quality: 'lossless' | 'high' | 'medium' | 'low';
  /** Migration triggers */
  triggers?: {
    formatDeprecated?: boolean;
    betterFormatAvailable?: boolean;
  };
}

/**
 * Format migration policy
 */
export interface MigrationPolicy {
  /** Whether migration is enabled */
  enabled: boolean;
  /** Check interval */
  checkInterval: 'yearly' | 'monthly' | 'quarterly';
  /** Migration rules */
  rules: MigrationRule[];
  /** Last check timestamp */
  lastCheck?: Date;
  /** Next check timestamp */
  nextCheck?: Date;
}

/**
 * Complete time capsule
 */
export interface TimeCapsule {
  /** Capsule ID */
  id: CapsuleId;
  /** Version of the WIA-TC format */
  version: string;
  /** Capsule title */
  title: string;
  /** Description */
  description?: string;
  /** Creation timestamp */
  created: Date;
  /** Last modified timestamp */
  modified: Date;
  /** Creator information */
  creator: Creator;
  /** Current status */
  status: CapsuleStatus;
  /** Security configuration */
  security: SecurityConfig;
  /** Access control */
  access: AccessControl;
  /** Content items */
  contents: Content[];
  /** Storage locations */
  storage: {
    primary: StorageLocation;
    backups: StorageLocation[];
  };
  /** Blockchain information */
  blockchain?: BlockchainInfo;
  /** Migration policy */
  migrationPolicy?: MigrationPolicy;
  /** Custom metadata */
  metadata?: Record<string, any>;
}

// ============================================================================
// API Request/Response Types
// ============================================================================

/**
 * Configuration for creating a time capsule
 */
export interface CreateCapsuleRequest {
  /** Capsule title */
  title: string;
  /** Description */
  description?: string;
  /** Unlock date */
  unlockDate: Date;
  /** Creator information */
  creator: Creator;
  /** Encryption configuration */
  encryption?: EncryptionConfig;
  /** Access control */
  access?: Partial<AccessControl>;
  /** Custom metadata */
  metadata?: Record<string, any>;
}

/**
 * Response when creating a capsule
 */
export interface CreateCapsuleResponse {
  /** Capsule ID */
  id: CapsuleId;
  /** Status */
  status: CapsuleStatus;
  /** Creation timestamp */
  created: Date;
  /** API endpoint for this capsule */
  endpoint: string;
}

/**
 * Configuration for adding content
 */
export interface AddContentRequest {
  /** File data (Buffer or Blob) */
  file: Buffer | Blob;
  /** Content type */
  type: ContentType;
  /** Metadata */
  metadata?: ContentMetadata;
}

/**
 * Response when adding content
 */
export interface AddContentResponse {
  /** Content ID */
  contentId: ContentId;
  /** Checksum */
  checksum: string;
  /** File size */
  size: number;
  /** Upload timestamp */
  uploaded: Date;
}

/**
 * Configuration for sealing a capsule
 */
export interface SealCapsuleRequest {
  /** Whether to create digital signature */
  signature: boolean;
  /** Whether to store hash on blockchain */
  blockchain: boolean;
  /** Storage backends to use */
  storage: StorageType[];
  /** Migration policy */
  migrationPolicy?: MigrationPolicy;
}

/**
 * Response when sealing a capsule
 */
export interface SealCapsuleResponse {
  /** Capsule ID */
  id: CapsuleId;
  /** New status */
  status: CapsuleStatus;
  /** Unlock date */
  unlockDate: Date;
  /** IPFS hash (if using IPFS) */
  ipfsHash?: string;
  /** Blockchain transaction hash */
  blockchainTx?: string;
  /** Storage locations */
  storageLocations: StorageLocation[];
  /** Seal timestamp */
  sealed: Date;
}

/**
 * Capsule status information
 */
export interface CapsuleStatusResponse {
  /** Capsule ID */
  id: CapsuleId;
  /** Title */
  title: string;
  /** Status */
  status: CapsuleStatus;
  /** Unlock date */
  unlockDate: Date;
  /** Days until unlock */
  daysUntilUnlock: number;
  /** Content count */
  contentCount: number;
  /** Total size in bytes */
  totalSize: number;
  /** Whether accessible */
  accessible: boolean;
}

/**
 * Early access request
 */
export interface EarlyAccessRequest {
  /** Capsule ID */
  capsuleId: CapsuleId;
  /** Requester information */
  requester: {
    name: string;
    email: string;
  };
  /** Reason for early access */
  reason: string;
  /** Requested duration (in days) */
  duration?: number;
}

/**
 * Schedule information
 */
export interface Schedule {
  /** Capsule ID */
  capsuleId: CapsuleId;
  /** Creation date */
  created: Date;
  /** Seal date */
  sealed?: Date;
  /** Unlock date */
  unlockDate: Date;
  /** Days remaining */
  daysRemaining: number;
  /** Next integrity check */
  nextIntegrityCheck?: Date;
  /** Next migration check */
  nextMigrationCheck?: Date;
}

// ============================================================================
// SDK Configuration
// ============================================================================

/**
 * SDK configuration options
 */
export interface SDKConfig {
  /** API endpoint */
  endpoint?: string;
  /** API key for authentication */
  apiKey?: string;
  /** Request timeout in milliseconds */
  timeout?: number;
  /** Whether to use encryption by default */
  defaultEncryption?: boolean;
  /** Default storage backends */
  defaultStorage?: StorageType[];
  /** Whether to use blockchain by default */
  defaultBlockchain?: boolean;
}

/**
 * Verification result
 */
export interface VerificationResult {
  /** Whether verification passed */
  valid: boolean;
  /** Content hash verification */
  hashValid: boolean;
  /** Signature verification */
  signatureValid: boolean;
  /** Integrity check result */
  integrityValid: boolean;
  /** Verification timestamp */
  verifiedAt: Date;
  /** Error messages (if any) */
  errors?: string[];
}

// ============================================================================
// Events
// ============================================================================

/**
 * Event types
 */
export enum EventType {
  CREATED = 'created',
  CONTENT_ADDED = 'content_added',
  SEALED = 'sealed',
  UNLOCKED = 'unlocked',
  ACCESSED = 'accessed',
  EARLY_ACCESS_REQUESTED = 'early_access_requested',
  EARLY_ACCESS_APPROVED = 'early_access_approved',
  INTEGRITY_CHECK = 'integrity_check',
  MIGRATION_PERFORMED = 'migration_performed'
}

/**
 * Event data
 */
export interface CapsuleEvent {
  /** Event type */
  type: EventType;
  /** Capsule ID */
  capsuleId: CapsuleId;
  /** Timestamp */
  timestamp: Date;
  /** Actor (user/system) */
  actor?: string;
  /** Event details */
  details: Record<string, any>;
}

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Pagination parameters
 */
export interface PaginationParams {
  /** Page number (1-indexed) */
  page: number;
  /** Items per page */
  limit: number;
  /** Sort field */
  sortBy?: string;
  /** Sort order */
  sortOrder?: 'asc' | 'desc';
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  /** Data items */
  data: T[];
  /** Pagination metadata */
  pagination: {
    page: number;
    limit: number;
    total: number;
    totalPages: number;
  };
}

/**
 * Error response
 */
export interface ErrorResponse {
  /** Error code */
  code: string;
  /** Error message */
  message: string;
  /** Additional details */
  details?: Record<string, any>;
  /** Timestamp */
  timestamp: Date;
}
