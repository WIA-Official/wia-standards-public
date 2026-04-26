/**
 * WIA-MED-012: Healthcare Blockchain Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

import EventEmitter from 'eventemitter3';
import {
  WIAConfig,
  APIResponse,
  PaginatedResponse,
  Block,
  Transaction,
  HealthRecord,
  ConsentRecord,
  TransactionType,
  RecordStatus,
} from './types';

export * from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface WIABlockchainConfig extends WIAConfig {
  networkId?: string;
  walletAddress?: string;
}

// ============================================================================
// Main SDK Client
// ============================================================================

export class WIAHealthcareBlockchainClient {
  private config: Required<WIABlockchainConfig>;
  private eventEmitter = new EventEmitter();

  constructor(config: WIABlockchainConfig) {
    this.config = {
      endpoint: 'https://api.wia-standards.org/v1/blockchain',
      timeout: 60000,
      debug: false,
      networkId: 'mainnet',
      walletAddress: '',
      ...config,
    };

    if (!this.config.apiKey) {
      throw new Error('API key is required');
    }
  }

  // ==========================================================================
  // Health Record Operations
  // ==========================================================================

  async createHealthRecord(record: Omit<HealthRecord, 'recordId' | 'createdAt' | 'updatedAt'>): Promise<APIResponse<HealthRecord>> {
    return this.makeRequest('POST', '/records', record);
  }

  async getHealthRecord(recordId: string): Promise<APIResponse<HealthRecord>> {
    return this.makeRequest('GET', `/records/${recordId}`);
  }

  async updateHealthRecord(recordId: string, updates: Partial<HealthRecord>): Promise<APIResponse<HealthRecord>> {
    return this.makeRequest('PATCH', `/records/${recordId}`, updates);
  }

  async getPatientRecords(patientId: string, filters?: {
    category?: string;
    status?: RecordStatus;
    startDate?: string;
    endDate?: string;
  }): Promise<PaginatedResponse<HealthRecord>> {
    const params = new URLSearchParams({ patientId, ...filters as Record<string, string> });
    return this.makeRequest('GET', `/records?${params}`);
  }

  async verifyRecord(recordId: string): Promise<APIResponse<{
    valid: boolean;
    hash: string;
    blockNumber: number;
    verifiedAt: string;
  }>> {
    return this.makeRequest('GET', `/records/${recordId}/verify`);
  }

  // ==========================================================================
  // Consent Operations
  // ==========================================================================

  async grantConsent(consent: Omit<ConsentRecord, 'consentId' | 'grantedAt'>): Promise<APIResponse<ConsentRecord>> {
    return this.makeRequest('POST', '/consents', consent);
  }

  async revokeConsent(consentId: string): Promise<APIResponse<ConsentRecord>> {
    return this.makeRequest('DELETE', `/consents/${consentId}`);
  }

  async checkConsent(patientId: string, requesterId: string, dataCategory: string): Promise<APIResponse<{
    hasConsent: boolean;
    consentId?: string;
    expiresAt?: string;
    permissions: string[];
  }>> {
    const params = new URLSearchParams({ patientId, requesterId, dataCategory });
    return this.makeRequest('GET', `/consents/check?${params}`);
  }

  async listConsents(patientId: string): Promise<PaginatedResponse<ConsentRecord>> {
    return this.makeRequest('GET', `/consents?patientId=${patientId}`);
  }

  // ==========================================================================
  // Transaction Operations
  // ==========================================================================

  async submitTransaction(transaction: Omit<Transaction, 'transactionId' | 'timestamp' | 'blockNumber'>): Promise<APIResponse<Transaction>> {
    return this.makeRequest('POST', '/transactions', transaction);
  }

  async getTransaction(transactionId: string): Promise<APIResponse<Transaction>> {
    return this.makeRequest('GET', `/transactions/${transactionId}`);
  }

  async listTransactions(filters?: {
    type?: TransactionType;
    from?: string;
    to?: string;
    startBlock?: number;
    endBlock?: number;
  }): Promise<PaginatedResponse<Transaction>> {
    const params = new URLSearchParams(filters as Record<string, string>);
    return this.makeRequest('GET', `/transactions?${params}`);
  }

  // ==========================================================================
  // Block Operations
  // ==========================================================================

  async getBlock(blockNumber: number): Promise<APIResponse<Block>> {
    return this.makeRequest('GET', `/blocks/${blockNumber}`);
  }

  async getLatestBlock(): Promise<APIResponse<Block>> {
    return this.makeRequest('GET', '/blocks/latest');
  }

  // ==========================================================================
  // Utility Methods
  // ==========================================================================

  async hashData(data: unknown): Promise<string> {
    const encoder = new TextEncoder();
    const dataBuffer = encoder.encode(JSON.stringify(data));
    const hashBuffer = await crypto.subtle.digest('SHA-256', dataBuffer);
    const hashArray = Array.from(new Uint8Array(hashBuffer));
    return hashArray.map(b => b.toString(16).padStart(2, '0')).join('');
  }

  on(event: 'recordCreated' | 'consentGranted' | 'transactionConfirmed' | 'error', callback: (...args: unknown[]) => void): void {
    this.eventEmitter.on(event, callback);
  }

  private async makeRequest<T>(method: string, path: string, body?: unknown): Promise<T> {
    const url = `${this.config.endpoint}${path}`;

    if (this.config.debug) {
      console.log(`[WIA Blockchain] ${method} ${url}`, body);
    }

    try {
      const response = await fetch(url, {
        method,
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${this.config.apiKey}`,
          'X-WIA-Standard': 'MED-012',
          'X-WIA-Version': '1.0.0',
          'X-WIA-Network': this.config.networkId,
        },
        body: body ? JSON.stringify(body) : undefined,
      });

      return await response.json();
    } catch (error: unknown) {
      const message = error instanceof Error ? error.message : 'Unknown error';
      this.eventEmitter.emit('error', { code: 'REQUEST_FAILED', message });
      throw error;
    }
  }
}

export function createClient(config: WIABlockchainConfig): WIAHealthcareBlockchainClient {
  return new WIAHealthcareBlockchainClient(config);
}

export default WIAHealthcareBlockchainClient;
