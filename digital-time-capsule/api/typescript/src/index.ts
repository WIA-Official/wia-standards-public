/**
 * WIA Digital Time Capsule Standard - TypeScript SDK
 * Version: 1.0
 *
 * 弘益人間 · Benefit All Humanity
 *
 * This SDK provides a comprehensive interface for creating, managing,
 * and accessing digital time capsules according to the WIA-LEG-001 standard.
 */

import axios, { AxiosInstance } from 'axios';
import { createHash, randomBytes } from 'crypto';
import * as nacl from 'tweetnacl';
import * as util from 'tweetnacl-util';

// Import all types
export * from './types';

import {
  CapsuleId,
  ContentId,
  TimeCapsule,
  CapsuleStatus,
  ContentType,
  StorageType,
  CreateCapsuleRequest,
  CreateCapsuleResponse,
  AddContentRequest,
  AddContentResponse,
  SealCapsuleRequest,
  SealCapsuleResponse,
  CapsuleStatusResponse,
  SDKConfig,
  VerificationResult,
  Content,
  AccessControl,
  MigrationPolicy,
  EarlyAccessRequest,
  Schedule,
} from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

const DEFAULT_CONFIG: Required<SDKConfig> = {
  endpoint: 'https://api.wia.org/timecapsule/v1',
  apiKey: '',
  timeout: 30000,
  defaultEncryption: true,
  defaultStorage: [StorageType.IPFS, StorageType.S3],
  defaultBlockchain: true,
};

// ============================================================================
// Time Capsule SDK
// ============================================================================

/**
 * Main SDK class for interacting with WIA Digital Time Capsule API
 */
export class TimeCapsuleSDK {
  private config: Required<SDKConfig>;
  private client: AxiosInstance;

  /**
   * Initialize the SDK
   * @param config - Configuration options
   */
  constructor(config: SDKConfig = {}) {
    this.config = { ...DEFAULT_CONFIG, ...config };

    this.client = axios.create({
      baseURL: this.config.endpoint,
      timeout: this.config.timeout,
      headers: {
        'Content-Type': 'application/json',
        'X-WIA-Version': '1.0',
        ...(this.config.apiKey && { Authorization: `Bearer ${this.config.apiKey}` }),
      },
    });
  }

  /**
   * Create a new time capsule
   * @param request - Capsule creation parameters
   * @returns Capsule instance
   */
  async create(request: CreateCapsuleRequest): Promise<TimeCapsuleInstance> {
    const response = await this.client.post<CreateCapsuleResponse>('/capsules', {
      ...request,
      encryption: request.encryption ?? {
        enabled: this.config.defaultEncryption,
        algorithm: 'AES-256-GCM',
      },
    });

    return new TimeCapsuleInstance(response.data.id, this);
  }

  /**
   * Get an existing time capsule
   * @param id - Capsule ID
   * @returns Capsule instance
   */
  async get(id: CapsuleId): Promise<TimeCapsuleInstance> {
    return new TimeCapsuleInstance(id, this);
  }

  /**
   * List all time capsules
   * @param page - Page number (1-indexed)
   * @param limit - Items per page
   * @returns List of capsule instances
   */
  async list(page: number = 1, limit: number = 20): Promise<TimeCapsuleInstance[]> {
    const response = await this.client.get('/capsules', {
      params: { page, limit },
    });

    return response.data.data.map(
      (capsule: any) => new TimeCapsuleInstance(capsule.id, this)
    );
  }

  /**
   * Verify capsule integrity
   * @param id - Capsule ID
   * @returns Verification result
   */
  async verify(id: CapsuleId): Promise<VerificationResult> {
    const response = await this.client.post<VerificationResult>(
      `/capsules/${id}/verify`
    );
    return response.data;
  }

  /**
   * Get internal HTTP client
   * @internal
   */
  getClient(): AxiosInstance {
    return this.client;
  }

  /**
   * Get configuration
   * @internal
   */
  getConfig(): Required<SDKConfig> {
    return this.config;
  }
}

// ============================================================================
// Time Capsule Instance
// ============================================================================

/**
 * Represents a single time capsule instance
 */
export class TimeCapsuleInstance {
  private id: CapsuleId;
  private sdk: TimeCapsuleSDK;
  private cachedData: TimeCapsule | null = null;

  /**
   * @internal
   */
  constructor(id: CapsuleId, sdk: TimeCapsuleSDK) {
    this.id = id;
    this.sdk = sdk;
  }

  /**
   * Get capsule ID
   */
  getId(): CapsuleId {
    return this.id;
  }

  /**
   * Fetch capsule details
   * @param refresh - Force refresh from server
   * @returns Complete capsule data
   */
  async fetch(refresh: boolean = false): Promise<TimeCapsule> {
    if (!refresh && this.cachedData) {
      return this.cachedData;
    }

    const response = await this.sdk.getClient().get<TimeCapsule>(`/capsules/${this.id}`);
    this.cachedData = response.data;
    return response.data;
  }

  /**
   * Get capsule status
   * @returns Status information
   */
  async getStatus(): Promise<CapsuleStatusResponse> {
    const response = await this.sdk.getClient().get<CapsuleStatusResponse>(
      `/capsules/${this.id}/status`
    );
    return response.data;
  }

  /**
   * Add a file to the capsule
   * @param file - File data (Buffer or Blob)
   * @param options - Content options
   * @returns Content information
   */
  async addFile(
    file: Buffer | Blob,
    options: {
      type: ContentType;
      filename?: string;
      description?: string;
      tags?: string[];
    }
  ): Promise<AddContentResponse> {
    const formData = new FormData();
    formData.append('file', file instanceof Buffer ? new Blob([file]) : file);
    formData.append('type', options.type);

    if (options.filename) formData.append('filename', options.filename);
    if (options.description) formData.append('description', options.description);
    if (options.tags) formData.append('tags', JSON.stringify(options.tags));

    const response = await this.sdk.getClient().post<AddContentResponse>(
      `/capsules/${this.id}/contents`,
      formData,
      {
        headers: { 'Content-Type': 'multipart/form-data' },
      }
    );

    this.cachedData = null; // Invalidate cache
    return response.data;
  }

  /**
   * Add multiple files from a directory
   * @param files - Array of files
   * @param type - Content type for all files
   * @returns Array of content information
   */
  async addFiles(
    files: Array<{ file: Buffer | Blob; filename: string }>,
    type: ContentType
  ): Promise<AddContentResponse[]> {
    const results: AddContentResponse[] = [];

    for (const { file, filename } of files) {
      const result = await this.addFile(file, { type, filename });
      results.push(result);
    }

    return results;
  }

  /**
   * List all contents in the capsule
   * @returns Array of content items
   */
  async listContents(): Promise<Content[]> {
    const capsule = await this.fetch();
    return capsule.contents;
  }

  /**
   * Remove content from capsule (only works if not sealed)
   * @param contentId - Content ID to remove
   */
  async removeContent(contentId: ContentId): Promise<void> {
    await this.sdk.getClient().delete(`/capsules/${this.id}/contents/${contentId}`);
    this.cachedData = null; // Invalidate cache
  }

  /**
   * Set access control configuration
   * @param access - Access control settings
   */
  async setAccessControl(access: Partial<AccessControl>): Promise<void> {
    await this.sdk.getClient().patch(`/capsules/${this.id}/access`, access);
    this.cachedData = null;
  }

  /**
   * Set migration policy
   * @param policy - Migration policy configuration
   */
  async setMigrationPolicy(policy: MigrationPolicy): Promise<void> {
    await this.sdk.getClient().patch(`/capsules/${this.id}/migration`, policy);
    this.cachedData = null;
  }

  /**
   * Set custom metadata
   * @param metadata - Custom metadata object
   */
  async setMetadata(metadata: Record<string, any>): Promise<void> {
    await this.sdk.getClient().patch(`/capsules/${this.id}/metadata`, { metadata });
    this.cachedData = null;
  }

  /**
   * Seal the time capsule
   * @param options - Sealing options
   * @returns Seal result
   */
  async seal(options?: Partial<SealCapsuleRequest>): Promise<SealCapsuleResponse> {
    const config = this.sdk.getConfig();

    const request: SealCapsuleRequest = {
      signature: true,
      blockchain: options?.blockchain ?? config.defaultBlockchain,
      storage: options?.storage ?? config.defaultStorage,
      migrationPolicy: options?.migrationPolicy,
    };

    const response = await this.sdk.getClient().post<SealCapsuleResponse>(
      `/capsules/${this.id}/seal`,
      request
    );

    this.cachedData = null;
    return response.data;
  }

  /**
   * Request early access to the capsule
   * @param request - Early access request details
   */
  async requestEarlyAccess(request: Omit<EarlyAccessRequest, 'capsuleId'>): Promise<void> {
    await this.sdk.getClient().post(`/capsules/${this.id}/early-access`, {
      ...request,
      capsuleId: this.id,
    });
  }

  /**
   * Approve early access request (for approvers)
   * @param requestId - Early access request ID
   * @param approved - Whether to approve
   * @param reason - Reason for decision
   */
  async approveEarlyAccess(
    requestId: string,
    approved: boolean,
    reason?: string
  ): Promise<void> {
    await this.sdk.getClient().post(`/capsules/${this.id}/early-access/${requestId}/approve`, {
      approved,
      reason,
    });
  }

  /**
   * Access capsule contents (only if unlocked or approved early access)
   * @param contentId - Optional specific content ID
   * @returns Content data
   */
  async access(contentId?: ContentId): Promise<Buffer | Content[]> {
    if (contentId) {
      const response = await this.sdk.getClient().get(
        `/capsules/${this.id}/contents/${contentId}/download`,
        { responseType: 'arraybuffer' }
      );
      return Buffer.from(response.data);
    } else {
      const response = await this.sdk.getClient().get<Content[]>(
        `/capsules/${this.id}/contents/all`
      );
      return response.data;
    }
  }

  /**
   * Get schedule information
   * @returns Schedule details
   */
  async getSchedule(): Promise<Schedule> {
    const response = await this.sdk.getClient().get<Schedule>(
      `/capsules/${this.id}/schedule`
    );
    return response.data;
  }

  /**
   * Verify capsule integrity
   * @returns Verification result
   */
  async verify(): Promise<VerificationResult> {
    return this.sdk.verify(this.id);
  }

  /**
   * Delete the capsule (only if not sealed)
   */
  async delete(): Promise<void> {
    await this.sdk.getClient().delete(`/capsules/${this.id}`);
    this.cachedData = null;
  }
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Calculate SHA3-256 hash of data
 * @param data - Data to hash
 * @returns Hash as hex string
 */
export function calculateHash(data: Buffer | string): string {
  const hash = createHash('sha3-256');
  hash.update(data);
  return hash.digest('hex');
}

/**
 * Generate Ed25519 key pair
 * @returns Key pair with public and private keys
 */
export function generateKeyPair(): {
  publicKey: string;
  privateKey: string;
} {
  const keyPair = nacl.sign.keyPair();
  return {
    publicKey: util.encodeBase64(keyPair.publicKey),
    privateKey: util.encodeBase64(keyPair.secretKey),
  };
}

/**
 * Sign data with Ed25519 private key
 * @param data - Data to sign
 * @param privateKey - Private key (base64 encoded)
 * @returns Signature (base64 encoded)
 */
export function signData(data: Buffer | string, privateKey: string): string {
  const dataBuffer = Buffer.isBuffer(data) ? data : Buffer.from(data);
  const privateKeyBytes = util.decodeBase64(privateKey);
  const signature = nacl.sign.detached(dataBuffer, privateKeyBytes);
  return util.encodeBase64(signature);
}

/**
 * Verify signature with Ed25519 public key
 * @param data - Original data
 * @param signature - Signature (base64 encoded)
 * @param publicKey - Public key (base64 encoded)
 * @returns Whether signature is valid
 */
export function verifySignature(
  data: Buffer | string,
  signature: string,
  publicKey: string
): boolean {
  const dataBuffer = Buffer.isBuffer(data) ? data : Buffer.from(data);
  const signatureBytes = util.decodeBase64(signature);
  const publicKeyBytes = util.decodeBase64(publicKey);
  return nacl.sign.detached.verify(dataBuffer, signatureBytes, publicKeyBytes);
}

/**
 * Generate random encryption key
 * @param length - Key length in bytes (default: 32 for AES-256)
 * @returns Key as base64 string
 */
export function generateEncryptionKey(length: number = 32): string {
  return randomBytes(length).toString('base64');
}

/**
 * Calculate days until a future date
 * @param date - Future date
 * @returns Number of days
 */
export function daysUntil(date: Date): number {
  const now = new Date();
  const diff = date.getTime() - now.getTime();
  return Math.ceil(diff / (1000 * 60 * 60 * 24));
}

/**
 * Format bytes to human-readable size
 * @param bytes - Size in bytes
 * @returns Formatted string (e.g., "1.5 MB")
 */
export function formatBytes(bytes: number): string {
  const units = ['B', 'KB', 'MB', 'GB', 'TB'];
  let size = bytes;
  let unitIndex = 0;

  while (size >= 1024 && unitIndex < units.length - 1) {
    size /= 1024;
    unitIndex++;
  }

  return `${size.toFixed(1)} ${units[unitIndex]}`;
}

/**
 * Validate capsule ID format
 * @param id - Capsule ID to validate
 * @returns Whether ID is valid
 */
export function isValidCapsuleId(id: string): boolean {
  const pattern = /^TC-\d{4}-\d{3}-[A-Z0-9]{3}$/;
  return pattern.test(id);
}

/**
 * Generate a new capsule ID
 * @param year - Year (default: current year)
 * @returns New capsule ID
 */
export function generateCapsuleId(year?: number): CapsuleId {
  const y = year ?? new Date().getFullYear();
  const seq = Math.floor(Math.random() * 1000)
    .toString()
    .padStart(3, '0');
  const random = randomBytes(2).toString('hex').toUpperCase().substring(0, 3);
  return `TC-${y}-${seq}-${random}`;
}

// ============================================================================
// Export default SDK
// ============================================================================

export default TimeCapsuleSDK;

/**
 * Example usage:
 *
 * ```typescript
 * import TimeCapsuleSDK from '@wia/digital-time-capsule';
 *
 * const sdk = new TimeCapsuleSDK({
 *   apiKey: 'your-api-key'
 * });
 *
 * // Create a new time capsule
 * const capsule = await sdk.create({
 *   title: 'Family Memories 2025',
 *   description: 'Photos and letters from our family reunion',
 *   unlockDate: new Date('2050-01-01'),
 *   creator: {
 *     name: 'John Smith',
 *     email: 'john@example.com'
 *   }
 * });
 *
 * // Add content
 * await capsule.addFile(photoBuffer, {
 *   type: ContentType.IMAGE,
 *   filename: 'reunion.jpg',
 *   description: 'Family reunion photo'
 * });
 *
 * // Seal the capsule
 * const result = await capsule.seal({
 *   blockchain: true,
 *   storage: [StorageType.IPFS, StorageType.S3]
 * });
 *
 * console.log('Capsule sealed:', result.id);
 * console.log('IPFS hash:', result.ipfsHash);
 * console.log('Unlock date:', result.unlockDate);
 * ```
 */
