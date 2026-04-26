/**
 * WIA-UNI-003: Family Reunion Data Standard - TypeScript SDK
 * 
 * @module @wia/family-reunion-sdk
 * @version 1.0.0
 */

import axios, { AxiosInstance } from 'axios';
import * as Types from './types';

export * from './types';

export class FamilyReunionClient {
  private client: AxiosInstance;
  private apiKey: string;

  constructor(config: Types.ClientConfig) {
    this.apiKey = config.apiKey;
    
    const baseURL = config.baseURL || this.getBaseURL(config.environment, config.region);
    
    this.client = axios.create({
      baseURL,
      headers: {
        'Authorization': `Bearer ${this.apiKey}`,
        'Content-Type': 'application/json',
        'X-WIA-Standard': 'UNI-003',
        'X-WIA-Version': '1.0.0'
      },
      timeout: 30000
    });
  }

  private getBaseURL(env?: string, region?: string): string {
    const environment = env || 'production';
    const reg = region || 'global';
    
    if (environment === 'sandbox') {
      return 'https://sandbox-api.wia.org/family-reunion';
    }
    
    return `https://api-${reg}.wia.org/family-reunion`;
  }

  /**
   * Search for family members across databases
   */
  async search(
    criteria: Types.SearchCriteria,
    options?: Types.SearchOptions
  ): Promise<Types.SearchResponse> {
    const response = await this.client.post('/api/v1/search', {
      searchCriteria: criteria,
      searchOptions: options || {
        fuzzyMatching: true,
        maxResults: 50,
        confidenceThreshold: 70
      }
    });
    
    return response.data;
  }

  /**
   * Submit DNA profile for matching
   */
  async submitDNA(profile: Omit<Types.DNAProfile, 'dnaProfileId'>): Promise<Types.DNAProfile> {
    const response = await this.client.post('/api/v1/dna/submit', profile);
    return response.data;
  }

  /**
   * Get DNA matches for a given profile
   */
  async getDNAMatches(
    dnaProfileId: string,
    matchThreshold?: number,
    relationshipTypes?: string[]
  ): Promise<Types.DNAMatchResponse> {
    const response = await this.client.post('/api/v1/dna/match', {
      dnaProfileId,
      matchThreshold: matchThreshold || 7, // centiMorgans
      relationshipTypes: relationshipTypes || ['PARENT', 'SIBLING', 'COUSIN']
    });
    
    return response.data;
  }

  /**
   * Upload and analyze photo
   */
  async analyzePhoto(
    imageData: string,
    metadata: Partial<Types.PhotoMetadata>
  ): Promise<Types.PhotoAnalysisResponse> {
    const response = await this.client.post('/api/v1/photo/analyze', {
      imageData,
      metadata,
      enhanceQuality: true
    });
    
    return response.data;
  }

  /**
   * Request reunion with matched person
   */
  async requestReunion(request: Types.ReunionRequest): Promise<Types.ReunionResponse> {
    const response = await this.client.post('/api/v1/reunion/request', request);
    return response.data;
  }

  /**
   * Get person details by ID
   */
  async getPerson(personId: string): Promise<Types.Person> {
    const response = await this.client.get(`/api/v1/person/${personId}`);
    return response.data;
  }

  /**
   * Update privacy settings
   */
  async updatePrivacySettings(
    personId: string,
    settings: Types.PrivacySettings
  ): Promise<void> {
    await this.client.put(`/api/v1/person/${personId}/privacy`, settings);
  }

  /**
   * Request data deletion (GDPR right to be forgotten)
   */
  async requestDataDeletion(
    personId: string,
    confirmDeletion: boolean,
    reason?: string
  ): Promise<{ status: string; deletionDate: string }> {
    const response = await this.client.post(`/api/v1/person/${personId}/delete`, {
      confirmDeletion,
      reason
    });
    
    return response.data;
  }

  /**
   * Get support resources by location
   */
  async getSupportResources(
    location: string,
    language: string,
    serviceType?: 'EMOTIONAL_SUPPORT' | 'LEGAL_AID' | 'TRANSLATION' | 'TRAVEL'
  ): Promise<Array<{
    organization: string;
    phone: string;
    services: string[];
    address: string;
  }>> {
    const response = await this.client.get('/api/v1/support/resources', {
      params: { location, language, serviceType }
    });
    
    return response.data;
  }
}

export default FamilyReunionClient;
