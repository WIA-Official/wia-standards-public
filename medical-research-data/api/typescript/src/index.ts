/**
 * WIA-MED-019: Medical Research Data Standard - TypeScript SDK
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
  ResearchStudy,
  Researcher,
  Participant,
  Dataset,
  DataAccessRequest,
  StudyStatus,
  StudyPhase,
  ConsentStatus,
} from './types';

export * from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface WIAResearchDataConfig extends WIAConfig {
  institutionId?: string;
}

// ============================================================================
// Main SDK Client
// ============================================================================

export class WIAResearchDataClient {
  private config: Required<WIAResearchDataConfig>;
  private eventEmitter = new EventEmitter();

  constructor(config: WIAResearchDataConfig) {
    this.config = {
      endpoint: 'https://api.wia-standards.org/v1/research',
      timeout: 60000,
      debug: false,
      institutionId: '',
      ...config,
    };

    if (!this.config.apiKey) {
      throw new Error('API key is required');
    }
  }

  // ==========================================================================
  // Study Operations
  // ==========================================================================

  async getStudy(studyId: string): Promise<APIResponse<ResearchStudy>> {
    return this.makeRequest('GET', `/studies/${studyId}`);
  }

  async listStudies(filters?: {
    status?: StudyStatus;
    phase?: StudyPhase;
    institution?: string;
  }): Promise<PaginatedResponse<ResearchStudy>> {
    const params = new URLSearchParams(filters as Record<string, string>);
    return this.makeRequest('GET', `/studies?${params}`);
  }

  async createStudy(study: Omit<ResearchStudy, 'studyId'>): Promise<APIResponse<ResearchStudy>> {
    return this.makeRequest('POST', '/studies', study);
  }

  async updateStudy(studyId: string, updates: Partial<ResearchStudy>): Promise<APIResponse<ResearchStudy>> {
    return this.makeRequest('PATCH', `/studies/${studyId}`, updates);
  }

  async updateStudyStatus(studyId: string, status: StudyStatus): Promise<APIResponse<ResearchStudy>> {
    return this.makeRequest('PATCH', `/studies/${studyId}/status`, { status });
  }

  // ==========================================================================
  // Researcher Operations
  // ==========================================================================

  async getResearcher(researcherId: string): Promise<APIResponse<Researcher>> {
    return this.makeRequest('GET', `/researchers/${researcherId}`);
  }

  async addResearcherToStudy(studyId: string, researcherId: string, role: string): Promise<APIResponse<void>> {
    return this.makeRequest('POST', `/studies/${studyId}/researchers`, { researcherId, role });
  }

  async listStudyResearchers(studyId: string): Promise<PaginatedResponse<Researcher>> {
    return this.makeRequest('GET', `/studies/${studyId}/researchers`);
  }

  // ==========================================================================
  // Participant Operations
  // ==========================================================================

  async enrollParticipant(studyId: string, participant: Omit<Participant, 'participantId' | 'enrollmentDate'>): Promise<APIResponse<Participant>> {
    return this.makeRequest('POST', `/studies/${studyId}/participants`, participant);
  }

  async getParticipant(studyId: string, participantId: string): Promise<APIResponse<Participant>> {
    return this.makeRequest('GET', `/studies/${studyId}/participants/${participantId}`);
  }

  async listParticipants(studyId: string, filters?: {
    status?: string;
    consentStatus?: ConsentStatus;
  }): Promise<PaginatedResponse<Participant>> {
    const params = new URLSearchParams(filters as Record<string, string>);
    return this.makeRequest('GET', `/studies/${studyId}/participants?${params}`);
  }

  async updateParticipantConsent(studyId: string, participantId: string, consentStatus: ConsentStatus): Promise<APIResponse<Participant>> {
    return this.makeRequest('PATCH', `/studies/${studyId}/participants/${participantId}/consent`, { consentStatus });
  }

  async withdrawParticipant(studyId: string, participantId: string, reason: string): Promise<APIResponse<Participant>> {
    return this.makeRequest('POST', `/studies/${studyId}/participants/${participantId}/withdraw`, { reason });
  }

  // ==========================================================================
  // Dataset Operations
  // ==========================================================================

  async getDataset(datasetId: string): Promise<APIResponse<Dataset>> {
    return this.makeRequest('GET', `/datasets/${datasetId}`);
  }

  async listDatasets(studyId: string): Promise<PaginatedResponse<Dataset>> {
    return this.makeRequest('GET', `/studies/${studyId}/datasets`);
  }

  async createDataset(studyId: string, dataset: Omit<Dataset, 'datasetId' | 'createdAt' | 'updatedAt'>): Promise<APIResponse<Dataset>> {
    return this.makeRequest('POST', `/studies/${studyId}/datasets`, dataset);
  }

  async updateDataset(datasetId: string, updates: Partial<Dataset>): Promise<APIResponse<Dataset>> {
    return this.makeRequest('PATCH', `/datasets/${datasetId}`, updates);
  }

  // ==========================================================================
  // Data Access Operations
  // ==========================================================================

  async requestDataAccess(request: Omit<DataAccessRequest, 'requestId' | 'submittedAt'>): Promise<APIResponse<DataAccessRequest>> {
    return this.makeRequest('POST', '/data-access-requests', request);
  }

  async getAccessRequest(requestId: string): Promise<APIResponse<DataAccessRequest>> {
    return this.makeRequest('GET', `/data-access-requests/${requestId}`);
  }

  async listAccessRequests(datasetId: string): Promise<PaginatedResponse<DataAccessRequest>> {
    return this.makeRequest('GET', `/datasets/${datasetId}/access-requests`);
  }

  async approveAccessRequest(requestId: string, approvedBy: string): Promise<APIResponse<DataAccessRequest>> {
    return this.makeRequest('POST', `/data-access-requests/${requestId}/approve`, { approvedBy });
  }

  async denyAccessRequest(requestId: string, reason: string): Promise<APIResponse<DataAccessRequest>> {
    return this.makeRequest('POST', `/data-access-requests/${requestId}/deny`, { reason });
  }

  // ==========================================================================
  // Event Handling
  // ==========================================================================

  on(event: 'studyCreated' | 'participantEnrolled' | 'datasetCreated' | 'accessApproved' | 'error', callback: (...args: unknown[]) => void): void {
    this.eventEmitter.on(event, callback);
  }

  // ==========================================================================
  // Private Methods
  // ==========================================================================

  private async makeRequest<T>(method: string, path: string, body?: unknown): Promise<T> {
    const url = `${this.config.endpoint}${path}`;

    if (this.config.debug) {
      console.log(`[WIA Research Data] ${method} ${url}`, body);
    }

    try {
      const response = await fetch(url, {
        method,
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${this.config.apiKey}`,
          'X-WIA-Standard': 'MED-019',
          'X-WIA-Version': '1.0.0',
          ...(this.config.institutionId && { 'X-Institution-ID': this.config.institutionId }),
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

export function createClient(config: WIAResearchDataConfig): WIAResearchDataClient {
  return new WIAResearchDataClient(config);
}

export default WIAResearchDataClient;
