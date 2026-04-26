/**
 * WIA-UNI-014: Legal System Harmonization SDK
 *
 * Official TypeScript/JavaScript SDK for the WIA-UNI-014 standard
 *
 * @example
 * ```typescript
 * import { WIALegalClient } from '@wia/legal-system-harmonization';
 *
 * const client = new WIALegalClient({
 *   apiKey: process.env.WIA_API_KEY,
 *   environment: 'production'
 * });
 *
 * // Get legal document
 * const document = await client.documents.get('WIA-UNI-014-LEGAL-abc123');
 *
 * // Search for laws
 * const results = await client.search({
 *   query: 'property rights',
 *   type: 'law',
 *   jurisdiction: 'unified'
 * });
 * ```
 *
 * @version 1.0.0
 * @license MIT
 */

import axios, { AxiosInstance } from 'axios';
import * as Types from './types';

export * from './types';

const DEFAULT_BASE_URL = 'https://api.wiastandards.com/uni-014/v1';
const SANDBOX_BASE_URL = 'https://sandbox-api.wiastandards.com/uni-014/v1';

/**
 * Main client for WIA-UNI-014 Legal System Harmonization API
 */
export class WIALegalClient {
  private http: AxiosInstance;
  public documents: DocumentsAPI;
  public properties: PropertiesAPI;
  public contracts: ContractsAPI;
  public courts: CourtsAPI;

  constructor(config: Types.ClientConfig) {
    const baseURL = config.baseURL ||
      (config.environment === 'sandbox' ? SANDBOX_BASE_URL : DEFAULT_BASE_URL);

    this.http = axios.create({
      baseURL,
      timeout: config.timeout || 30000,
      headers: {
        'Authorization': `Bearer ${config.apiKey}`,
        'Content-Type': 'application/json',
        'User-Agent': 'WIA-UNI-014-SDK/1.0.0'
      }
    });

    // Initialize API modules
    this.documents = new DocumentsAPI(this.http);
    this.properties = new PropertiesAPI(this.http);
    this.contracts = new ContractsAPI(this.http);
    this.courts = new CourtsAPI(this.http);
  }

  /**
   * Search for legal documents
   */
  async search(options: Types.SearchOptions): Promise<Types.SearchResponse> {
    const response = await this.http.get('/documents', { params: options });
    return response.data;
  }
}

/**
 * Documents API
 */
class DocumentsAPI {
  constructor(private http: AxiosInstance) {}

  /**
   * Get a legal document by ID
   */
  async get(id: string): Promise<Types.LegalDocument> {
    const response = await this.http.get(`/documents/${id}`);
    return response.data;
  }

  /**
   * List legal documents
   */
  async list(options: Types.SearchOptions = {}): Promise<Types.SearchResponse> {
    const response = await this.http.get('/documents', { params: options });
    return response.data;
  }

  /**
   * Get a specific law
   */
  async getLaw(id: string): Promise<Types.LegalDocument> {
    const response = await this.http.get(`/laws/${id}`);
    return response.data;
  }

  /**
   * Get a court judgment
   */
  async getJudgment(id: string): Promise<Types.JudgmentDocument> {
    const response = await this.http.get(`/judgments/${id}`);
    return response.data;
  }
}

/**
 * Properties API
 */
class PropertiesAPI {
  constructor(private http: AxiosInstance) {}

  /**
   * Get property details
   */
  async get(id: string): Promise<Types.PropertyDocument> {
    const response = await this.http.get(`/properties/${id}`);
    return response.data;
  }

  /**
   * Get property owner
   */
  async getOwner(id: string): Promise<Types.Ownership> {
    const response = await this.http.get(`/properties/${id}/owner`);
    return response.data;
  }

  /**
   * Get property history
   */
  async getHistory(id: string): Promise<Types.PropertyHistoryEntry[]> {
    const response = await this.http.get(`/properties/${id}/history`);
    return response.data;
  }

  /**
   * Search properties
   */
  async search(query: string, options: { limit?: number; offset?: number } = {}): Promise<Types.SearchResponse> {
    const response = await this.http.get('/properties/search', {
      params: { q: query, ...options }
    });
    return response.data;
  }

  /**
   * Initiate property transfer
   */
  async initiateTransfer(request: Types.PropertyTransferRequest): Promise<{ transactionId: string }> {
    const response = await this.http.post(`/properties/${request.propertyId}/transfer`, request);
    return response.data;
  }
}

/**
 * Contracts API
 */
class ContractsAPI {
  constructor(private http: AxiosInstance) {}

  /**
   * Create a new contract
   */
  async create(contract: Types.ContractCreateRequest): Promise<Types.ContractDocument> {
    const response = await this.http.post('/contracts', contract);
    return response.data;
  }

  /**
   * Get contract by ID
   */
  async get(id: string): Promise<Types.ContractDocument> {
    const response = await this.http.get(`/contracts/${id}`);
    return response.data;
  }

  /**
   * Check contract validity
   */
  async checkValidity(id: string): Promise<{
    valid: boolean;
    status: string;
    compliance: {
      northKoreaCompliant: boolean;
      southKoreaCompliant: boolean;
      issues: string[];
    };
  }> {
    const response = await this.http.get(`/contracts/${id}/validity`);
    return response.data;
  }

  /**
   * List contracts
   */
  async list(options: { limit?: number; offset?: number } = {}): Promise<Types.SearchResponse> {
    const response = await this.http.get('/contracts', { params: options });
    return response.data;
  }
}

/**
 * Courts API
 */
class CourtsAPI {
  constructor(private http: AxiosInstance) {}

  /**
   * File a new case
   */
  async file(filing: {
    courtId: string;
    caseType: string;
    parties: {
      plaintiffs: string[];
      defendants: string[];
    };
    documents: string[];
    filingFee?: { amount: number; paymentMethod: string };
  }): Promise<{ caseId: string; caseNumber: string }> {
    const response = await this.http.post(`/courts/${filing.courtId}/filings`, filing);
    return response.data;
  }

  /**
   * Get case status
   */
  async getCaseStatus(caseId: string): Promise<{
    caseNumber: string;
    status: string;
    filingDate: string;
    nextHearing?: string;
  }> {
    const response = await this.http.get(`/courts/cases/${caseId}/status`);
    return response.data;
  }

  /**
   * Get court information
   */
  async getCourt(courtId: string): Promise<Types.Court> {
    const response = await this.http.get(`/courts/${courtId}`);
    return response.data;
  }
}

/**
 * Helper functions
 */
export class WIALegalHelpers {
  /**
   * Generate WIA-UNI-014 document ID
   */
  static generateDocumentId(type: 'LEGAL' | 'PROP' | 'CONTRACT'): string {
    const uuid = crypto.randomUUID();
    return `WIA-UNI-014-${type}-${uuid}`;
  }

  /**
   * Validate document structure
   */
  static validateDocument(document: Types.LegalDocument): { valid: boolean; errors: string[] } {
    const errors: string[] = [];

    if (document.standard !== 'WIA-UNI-014') {
      errors.push('Invalid standard identifier');
    }

    if (!document.document.id || !document.document.id.startsWith('WIA-UNI-014')) {
      errors.push('Invalid document ID format');
    }

    if (!document.document.metadata.title) {
      errors.push('Document title is required');
    }

    if (!document.document.metadata.issueDate) {
      errors.push('Issue date is required');
    }

    return {
      valid: errors.length === 0,
      errors
    };
  }

  /**
   * Check if two jurisdictions are compatible
   */
  static areJurisdictionsCompatible(j1: Types.JurisdictionSystem, j2: Types.JurisdictionSystem): boolean {
    if (j1 === 'unified' || j2 === 'unified') return true;
    return j1 === j2;
  }
}

// Default export
export default WIALegalClient;
