/**
 * WIA-LEG-008: Data Portability Standard - TypeScript SDK
 * @version 1.0.0
 * @license MIT
 */

import {
  DataPortabilityPackage,
  ExportConfig,
  ExportResult,
  ImportConfig,
  ImportResult,
  ValidationResult,
  TransferConfig,
  TransferResult,
  ConsentRecord,
  SDKConfig,
  EncryptionMetadata
} from './types';

/**
 * Main SDK class for WIA-LEG-008 Data Portability
 */
export class DataPortabilitySDK {
  private config: SDKConfig;
  private apiEndpoint: string;

  constructor(config: SDKConfig = {}) {
    this.config = config;
    this.apiEndpoint = config.apiEndpoint || 'https://api.wiastandards.com/leg-008/v1';
  }

  /**
   * Export all data for a deceased user
   */
  async exportAllData(config: ExportConfig): Promise<ExportResult> {
    try {
      const response = await fetch(`${this.apiEndpoint}/export/initiate`, {
        method: 'POST',
        headers: this.getHeaders(),
        body: JSON.stringify(config)
      });

      if (!response.ok) {
        throw new Error(`Export failed: ${response.statusText}`);
      }

      const result: ExportResult = await response.json();
      
      // Wait for export to complete if needed
      if (result.status === 'in_progress') {
        return await this.waitForExportCompletion(result.export_id);
      }

      return result;
    } catch (error) {
      throw new Error(`Export all data failed: ${error.message}`);
    }
  }

  /**
   * Create export package
   */
  async createExport(config: ExportConfig): Promise<DataPortabilityPackage> {
    const result = await this.exportAllData(config);
    
    if (result.status !== 'completed' || !result.dpp) {
      throw new Error('Export did not complete successfully');
    }

    return result.dpp;
  }

  /**
   * Save export to file
   */
  async saveExport(dpp: DataPortabilityPackage, filepath: string): Promise<void> {
    const json = JSON.stringify(dpp, null, 2);
    
    // In browser environment
    if (typeof window !== 'undefined') {
      const blob = new Blob([json], { type: 'application/json' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = filepath;
      a.click();
      URL.revokeObjectURL(url);
    }
    // In Node.js environment
    else if (typeof require !== 'undefined') {
      const fs = require('fs').promises;
      await fs.writeFile(filepath, json, 'utf-8');
    }
  }

  /**
   * Initiate cross-platform transfer
   */
  async initiateTransfer(config: TransferConfig): Promise<TransferResult> {
    try {
      const response = await fetch(`${this.apiEndpoint}/transfer/initiate`, {
        method: 'POST',
        headers: this.getHeaders(),
        body: JSON.stringify(config)
      });

      if (!response.ok) {
        throw new Error(`Transfer failed: ${response.statusText}`);
      }

      return await response.json();
    } catch (error) {
      throw new Error(`Initiate transfer failed: ${error.message}`);
    }
  }

  /**
   * Verify executor consent
   */
  async verifyConsent(
    deceasedId: string,
    executorId: string,
    requestedPermissions: string[]
  ): Promise<{ valid: boolean; consent_id?: string; errors?: string[] }> {
    try {
      const response = await fetch(`${this.apiEndpoint}/consent/verify`, {
        method: 'POST',
        headers: this.getHeaders(),
        body: JSON.stringify({
          deceased_id: deceasedId,
          executor_id: executorId,
          requested_permissions: requestedPermissions
        })
      });

      if (!response.ok) {
        return {
          valid: false,
          errors: [`Verification failed: ${response.statusText}`]
        };
      }

      return await response.json();
    } catch (error) {
      return {
        valid: false,
        errors: [error.message]
      };
    }
  }

  /**
   * Validate data portability package
   */
  async validateDPP(dpp: DataPortabilityPackage): Promise<ValidationResult> {
    const errors: string[] = [];
    const warnings: string[] = [];

    // Check required fields
    if (!dpp['@context']) {
      errors.push('Missing @context field');
    }
    if (dpp['@context'] !== 'https://schema.wiastandards.com/leg-008/v1') {
      errors.push('Invalid @context value');
    }
    if (!dpp.version || dpp.version !== '1.0') {
      errors.push('Invalid or missing version');
    }
    if (!dpp.deceased || !dpp.deceased.id) {
      errors.push('Missing deceased information');
    }
    if (!dpp.executor || !dpp.executor.id) {
      errors.push('Missing executor information');
    }
    if (!dpp.data_inventory) {
      errors.push('Missing data inventory');
    }

    // Check encryption if present
    if (dpp.encryption) {
      if (!dpp.encryption.algorithm || !dpp.encryption.key_derivation) {
        errors.push('Invalid encryption metadata');
      }
    }

    // Check metadata
    if (!dpp.metadata || !dpp.metadata.checksum) {
      warnings.push('Missing or incomplete metadata');
    }

    // Verify checksum if present
    if (dpp.metadata?.checksum) {
      const calculatedChecksum = await this.calculateChecksum(dpp);
      if (dpp.metadata.checksum !== calculatedChecksum) {
        errors.push('Checksum verification failed');
      }
    }

    return {
      valid: errors.length === 0,
      format_valid: true,
      schema_valid: errors.length === 0,
      encryption_valid: dpp.encryption ? errors.length === 0 : true,
      errors: errors.length > 0 ? errors : undefined,
      warnings: warnings.length > 0 ? warnings : undefined
    };
  }

  /**
   * Encrypt data portability package
   */
  async encryptDPP(
    dpp: DataPortabilityPackage,
    password: string,
    algorithm: 'AES-256-GCM' | 'ChaCha20-Poly1305' = 'AES-256-GCM'
  ): Promise<any> {
    try {
      // Generate salt
      const salt = crypto.getRandomValues(new Uint8Array(32));
      
      // Derive key from password
      const key = await this.deriveKey(password, salt);
      
      // Generate IV
      const iv = crypto.getRandomValues(new Uint8Array(12));
      
      // Encrypt
      const plaintext = JSON.stringify(dpp);
      const encoder = new TextEncoder();
      const data = encoder.encode(plaintext);
      
      const ciphertext = await crypto.subtle.encrypt(
        {
          name: 'AES-GCM',
          iv: iv
        },
        key,
        data
      );

      return {
        version: '1.0',
        encryption: {
          algorithm: algorithm,
          key_derivation: 'PBKDF2',
          iterations: 100000,
          salt: this.arrayBufferToBase64(salt),
          iv: this.arrayBufferToBase64(iv)
        },
        ciphertext: this.arrayBufferToBase64(ciphertext)
      };
    } catch (error) {
      throw new Error(`Encryption failed: ${error.message}`);
    }
  }

  /**
   * Decrypt data portability package
   */
  async decryptDPP(encrypted: any, password: string): Promise<DataPortabilityPackage> {
    try {
      // Decode parameters
      const salt = this.base64ToArrayBuffer(encrypted.encryption.salt);
      const iv = this.base64ToArrayBuffer(encrypted.encryption.iv);
      const ciphertext = this.base64ToArrayBuffer(encrypted.ciphertext);
      
      // Derive key
      const key = await this.deriveKey(password, new Uint8Array(salt));
      
      // Decrypt
      const plaintext = await crypto.subtle.decrypt(
        {
          name: 'AES-GCM',
          iv: new Uint8Array(iv)
        },
        key,
        ciphertext
      );
      
      const decoder = new TextDecoder();
      const json = decoder.decode(plaintext);
      
      return JSON.parse(json);
    } catch (error) {
      throw new Error(`Decryption failed: ${error.message}`);
    }
  }

  /**
   * Generate data inventory
   */
  async generateInventory(deceasedId: string): Promise<any> {
    try {
      const response = await fetch(`${this.apiEndpoint}/inventory/generate`, {
        method: 'POST',
        headers: this.getHeaders(),
        body: JSON.stringify({ deceased_id: deceasedId })
      });

      if (!response.ok) {
        throw new Error(`Inventory generation failed: ${response.statusText}`);
      }

      return await response.json();
    } catch (error) {
      throw new Error(`Generate inventory failed: ${error.message}`);
    }
  }

  /**
   * Import data to platform
   */
  async importData(config: ImportConfig): Promise<ImportResult> {
    try {
      // First validate
      const validation = await this.validateDPP(config.dpp);
      if (!validation.valid) {
        return {
          import_id: '',
          status: 'failed',
          validation_results: validation,
          error: 'Validation failed'
        };
      }

      // Initiate import
      const response = await fetch(`${this.apiEndpoint}/import/initiate`, {
        method: 'POST',
        headers: this.getHeaders(),
        body: JSON.stringify(config)
      });

      if (!response.ok) {
        throw new Error(`Import failed: ${response.statusText}`);
      }

      return await response.json();
    } catch (error) {
      throw new Error(`Import data failed: ${error.message}`);
    }
  }

  // Private helper methods

  private getHeaders(): HeadersInit {
    const headers: HeadersInit = {
      'Content-Type': 'application/json',
      'User-Agent': 'WIA-LEG-008-SDK/1.0.0'
    };

    if (this.config.apiKey) {
      headers['Authorization'] = `Bearer ${this.config.apiKey}`;
    }

    if (this.config.executorCredentials) {
      headers['X-Executor-ID'] = this.config.executorCredentials.id;
    }

    return headers;
  }

  private async waitForExportCompletion(exportId: string, maxWaitMs: number = 300000): Promise<ExportResult> {
    const startTime = Date.now();
    
    while (Date.now() - startTime < maxWaitMs) {
      const status = await this.checkExportStatus(exportId);
      
      if (status.status === 'completed' || status.status === 'failed') {
        return status;
      }
      
      // Wait 2 seconds before checking again
      await new Promise(resolve => setTimeout(resolve, 2000));
    }
    
    throw new Error('Export timeout');
  }

  private async checkExportStatus(exportId: string): Promise<ExportResult> {
    const response = await fetch(`${this.apiEndpoint}/export/${exportId}/status`, {
      headers: this.getHeaders()
    });
    
    if (!response.ok) {
      throw new Error(`Status check failed: ${response.statusText}`);
    }
    
    return await response.json();
  }

  private async calculateChecksum(dpp: DataPortabilityPackage): Promise<string> {
    const json = JSON.stringify(dpp);
    const encoder = new TextEncoder();
    const data = encoder.encode(json);
    const hash = await crypto.subtle.digest('SHA-256', data);
    const hashArray = Array.from(new Uint8Array(hash));
    const hashHex = hashArray.map(b => b.toString(16).padStart(2, '0')).join('');
    return `sha256:${hashHex}`;
  }

  private async deriveKey(password: string, salt: Uint8Array): Promise<CryptoKey> {
    const encoder = new TextEncoder();
    const passwordKey = await crypto.subtle.importKey(
      'raw',
      encoder.encode(password),
      'PBKDF2',
      false,
      ['deriveBits', 'deriveKey']
    );
    
    return await crypto.subtle.deriveKey(
      {
        name: 'PBKDF2',
        salt: salt,
        iterations: 100000,
        hash: 'SHA-256'
      },
      passwordKey,
      { name: 'AES-GCM', length: 256 },
      false,
      ['encrypt', 'decrypt']
    );
  }

  private arrayBufferToBase64(buffer: ArrayBuffer): string {
    const bytes = new Uint8Array(buffer);
    let binary = '';
    for (let i = 0; i < bytes.byteLength; i++) {
      binary += String.fromCharCode(bytes[i]);
    }
    return btoa(binary);
  }

  private base64ToArrayBuffer(base64: string): ArrayBuffer {
    const binary = atob(base64);
    const bytes = new Uint8Array(binary.length);
    for (let i = 0; i < binary.length; i++) {
      bytes[i] = binary.charCodeAt(i);
    }
    return bytes.buffer;
  }
}

/**
 * Data Portability Manager - High-level API
 */
export class DataPortabilityManager {
  private sdk: DataPortabilitySDK;
  private deceasedId: string;
  private executorId: string;

  constructor(config: { deceasedId: string; executorId: string; apiKey?: string }) {
    this.deceasedId = config.deceasedId;
    this.executorId = config.executorId;
    this.sdk = new DataPortabilitySDK({
      deceasedId: config.deceasedId,
      apiKey: config.apiKey
    });
  }

  async createExport(config: Partial<ExportConfig>): Promise<DataPortabilityPackage> {
    const fullConfig: ExportConfig = {
      deceased_id: this.deceasedId,
      executor_id: this.executorId,
      categories: config.categories || ['all'],
      format: config.format || 'json-ld',
      encryption: config.encryption
    };

    return await this.sdk.createExport(fullConfig);
  }

  async initiateTransfer(config: Partial<TransferConfig>): Promise<TransferResult> {
    const fullConfig: TransferConfig = {
      source: config.source || '',
      destination: config.destination || '',
      dataTypes: config.dataTypes || [],
      executorConsent: config.executorConsent !== false
    };

    return await this.sdk.initiateTransfer(fullConfig);
  }

  async verifyConsent(permissions: string[]): Promise<boolean> {
    const result = await this.sdk.verifyConsent(
      this.deceasedId,
      this.executorId,
      permissions
    );
    return result.valid;
  }
}

// Export helper functions
export * from './types';

export {
  createExportPackage,
  initiateTransfer,
  verifyConsent
} from './helpers';

/**
 * Create a data portability package
 */
export async function createDataPortabilityPackage(
  config: ExportConfig
): Promise<DataPortabilityPackage> {
  const sdk = new DataPortabilitySDK();
  return await sdk.createExport(config);
}

/**
 * Validate a data portability package
 */
export async function validateDataPortabilityPackage(
  dpp: DataPortabilityPackage
): Promise<ValidationResult> {
  const sdk = new DataPortabilitySDK();
  return await sdk.validateDPP(dpp);
}
