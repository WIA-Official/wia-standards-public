/**
 * WIA-SOC-005: Civic Participation Standard - TypeScript SDK
 *
 * @module @wia/civic-participation
 * @description Official TypeScript SDK for WIA Civic Participation Standard
 *
 * @example
 * ```typescript
 * import { CivicParticipation } from '@wia/civic-participation';
 *
 * const civic = new CivicParticipation({
 *   apiKey: 'your-api-key',
 *   network: 'mainnet'
 * });
 *
 * // Create a vote
 * const vote = await civic.createVote({
 *   title: 'Community Park Renovation',
 *   description: 'Should we renovate the central park?',
 *   type: 'single',
 *   options: [{ text: 'Yes' }, { text: 'No' }],
 *   schedule: {
 *     startDate: new Date('2025-01-01'),
 *     endDate: new Date('2025-01-31')
 *   }
 * });
 * ```
 */

import axios, { AxiosInstance, AxiosError } from 'axios';
import WebSocket from 'ws';
import type {
  CivicParticipationConfig,
  Vote,
  CreateVoteRequest,
  Ballot,
  CastBallotRequest,
  BallotReceipt,
  VoteResults,
  Petition,
  CreatePetitionRequest,
  SignPetitionRequest,
  SignatureReceipt,
  Consultation,
  CreateConsultationRequest,
  AddCommentRequest,
  ConsultationComment,
  BudgetProposal,
  CreateBudgetProposalRequest,
  VoteBudgetProposalRequest,
  ApiResponse,
  PaginatedResponse,
  PaginationParams,
  FilterParams,
  CivicEvent,
  EventType,
} from './types';

export * from './types';

/**
 * Main SDK class for WIA Civic Participation
 */
export class CivicParticipation {
  private config: Required<CivicParticipationConfig>;
  private httpClient: AxiosInstance;
  private wsClient?: WebSocket;
  private eventHandlers: Map<EventType, Set<(event: CivicEvent) => void>>;

  /**
   * Initialize the Civic Participation SDK
   *
   * @param config - Configuration options
   */
  constructor(config: CivicParticipationConfig) {
    this.config = this.mergeConfig(config);
    this.httpClient = this.createHttpClient();
    this.eventHandlers = new Map();

    if (this.config.websocket.enabled) {
      this.initWebSocket();
    }
  }

  // ============================================================================
  // Configuration
  // ============================================================================

  private mergeConfig(config: CivicParticipationConfig): Required<CivicParticipationConfig> {
    return {
      apiKey: config.apiKey,
      baseUrl: config.baseUrl || 'https://api.wia.org/civic-participation',
      network: config.network || 'mainnet',
      timeout: config.timeout || 30000,
      retries: config.retries || 3,
      blockchain: {
        enabled: config.blockchain?.enabled ?? true,
        provider: config.blockchain?.provider || 'https://mainnet.infura.io',
        contractAddress: config.blockchain?.contractAddress || '0x...',
      },
      websocket: {
        enabled: config.websocket?.enabled ?? false,
        url: config.websocket?.url || 'wss://api.wia.org/civic-participation/ws',
      },
    };
  }

  private createHttpClient(): AxiosInstance {
    return axios.create({
      baseURL: this.config.baseUrl,
      timeout: this.config.timeout,
      headers: {
        'Content-Type': 'application/json',
        Authorization: `Bearer ${this.config.apiKey}`,
        'X-WIA-Network': this.config.network,
      },
    });
  }

  // ============================================================================
  // Voting Methods
  // ============================================================================

  /**
   * Create a new vote
   *
   * @param request - Vote creation parameters
   * @returns Created vote object
   *
   * @example
   * ```typescript
   * const vote = await civic.createVote({
   *   title: 'Community Park Renovation',
   *   description: 'Should we renovate the central park?',
   *   type: 'single',
   *   options: [
   *     { text: 'Yes' },
   *     { text: 'No' },
   *     { text: 'Need more information' }
   *   ],
   *   schedule: {
   *     startDate: new Date('2025-01-01'),
   *     endDate: new Date('2025-01-31')
   *   }
   * });
   * ```
   */
  async createVote(request: CreateVoteRequest): Promise<Vote> {
    const response = await this.httpClient.post<ApiResponse<Vote>>('/api/v1/votes', request);
    return this.handleResponse(response.data);
  }

  /**
   * Get vote by ID
   *
   * @param voteId - Vote identifier
   * @returns Vote object
   */
  async getVote(voteId: string): Promise<Vote> {
    const response = await this.httpClient.get<ApiResponse<Vote>>(`/api/v1/votes/${voteId}`);
    return this.handleResponse(response.data);
  }

  /**
   * List votes with optional filters and pagination
   *
   * @param params - Pagination and filter parameters
   * @returns Paginated list of votes
   */
  async listVotes(
    params?: PaginationParams & FilterParams
  ): Promise<PaginatedResponse<Vote>> {
    const response = await this.httpClient.get<ApiResponse<PaginatedResponse<Vote>>>(
      '/api/v1/votes',
      { params }
    );
    return this.handleResponse(response.data);
  }

  /**
   * Cast a ballot in a vote
   *
   * @param voteId - Vote identifier
   * @param request - Ballot casting parameters
   * @returns Ballot receipt
   *
   * @example
   * ```typescript
   * const receipt = await civic.castBallot('vote-123', {
   *   selection: 0, // Vote for option 0
   *   voterToken: 'anonymous-token-xyz'
   * });
   * console.log('Ballot cast! Receipt:', receipt.verificationCode);
   * ```
   */
  async castBallot(voteId: string, request: CastBallotRequest): Promise<BallotReceipt> {
    const response = await this.httpClient.post<ApiResponse<BallotReceipt>>(
      `/api/v1/votes/${voteId}/ballots`,
      request
    );
    return this.handleResponse(response.data);
  }

  /**
   * Get vote results
   *
   * @param voteId - Vote identifier
   * @returns Vote results with counts and blockchain hash
   */
  async getVoteResults(voteId: string): Promise<VoteResults> {
    const response = await this.httpClient.get<ApiResponse<VoteResults>>(
      `/api/v1/votes/${voteId}/results`
    );
    return this.handleResponse(response.data);
  }

  /**
   * Close a vote (for authorized users only)
   *
   * @param voteId - Vote identifier
   * @returns Updated vote object
   */
  async closeVote(voteId: string): Promise<Vote> {
    const response = await this.httpClient.post<ApiResponse<Vote>>(
      `/api/v1/votes/${voteId}/close`
    );
    return this.handleResponse(response.data);
  }

  // ============================================================================
  // Petition Methods
  // ============================================================================

  /**
   * Create a new petition
   *
   * @param request - Petition creation parameters
   * @returns Created petition object
   *
   * @example
   * ```typescript
   * const petition = await civic.createPetition({
   *   title: 'Improve Public Transportation',
   *   description: 'We need better bus routes...',
   *   category: 'transportation',
   *   objectives: ['Add 3 new bus routes', 'Increase frequency'],
   *   signatureGoal: 1000,
   *   recipient: {
   *     name: 'City Council',
   *     organization: 'City of Springfield'
   *   }
   * });
   * ```
   */
  async createPetition(request: CreatePetitionRequest): Promise<Petition> {
    const response = await this.httpClient.post<ApiResponse<Petition>>(
      '/api/v1/petitions',
      request
    );
    return this.handleResponse(response.data);
  }

  /**
   * Get petition by ID
   *
   * @param petitionId - Petition identifier
   * @returns Petition object
   */
  async getPetition(petitionId: string): Promise<Petition> {
    const response = await this.httpClient.get<ApiResponse<Petition>>(
      `/api/v1/petitions/${petitionId}`
    );
    return this.handleResponse(response.data);
  }

  /**
   * List petitions with optional filters and pagination
   *
   * @param params - Pagination and filter parameters
   * @returns Paginated list of petitions
   */
  async listPetitions(
    params?: PaginationParams & FilterParams
  ): Promise<PaginatedResponse<Petition>> {
    const response = await this.httpClient.get<ApiResponse<PaginatedResponse<Petition>>>(
      '/api/v1/petitions',
      { params }
    );
    return this.handleResponse(response.data);
  }

  /**
   * Sign a petition
   *
   * @param petitionId - Petition identifier
   * @param request - Signature parameters
   * @returns Signature receipt
   *
   * @example
   * ```typescript
   * const receipt = await civic.signPetition('petition-456', {
   *   signerToken: 'verified-token-abc',
   *   email: 'user@example.com',
   *   comment: 'I fully support this initiative!'
   * });
   * ```
   */
  async signPetition(
    petitionId: string,
    request: SignPetitionRequest
  ): Promise<SignatureReceipt> {
    const response = await this.httpClient.post<ApiResponse<SignatureReceipt>>(
      `/api/v1/petitions/${petitionId}/signatures`,
      request
    );
    return this.handleResponse(response.data);
  }

  // ============================================================================
  // Consultation Methods
  // ============================================================================

  /**
   * Create a new public consultation
   *
   * @param request - Consultation creation parameters
   * @returns Created consultation object
   *
   * @example
   * ```typescript
   * const consultation = await civic.createConsultation({
   *   topic: 'City Climate Action Plan',
   *   description: 'Help us develop climate strategy...',
   *   category: 'environment',
   *   schedule: {
   *     phases: [
   *       {
   *         name: 'Information Phase',
   *         type: 'information',
   *         startDate: new Date('2025-02-01'),
   *         endDate: new Date('2025-02-15')
   *       },
   *       {
   *         name: 'Discussion Phase',
   *         type: 'discussion',
   *         startDate: new Date('2025-02-16'),
   *         endDate: new Date('2025-03-15')
   *       }
   *     ]
   *   }
   * });
   * ```
   */
  async createConsultation(request: CreateConsultationRequest): Promise<Consultation> {
    const response = await this.httpClient.post<ApiResponse<Consultation>>(
      '/api/v1/consultations',
      request
    );
    return this.handleResponse(response.data);
  }

  /**
   * Get consultation by ID
   *
   * @param consultationId - Consultation identifier
   * @returns Consultation object
   */
  async getConsultation(consultationId: string): Promise<Consultation> {
    const response = await this.httpClient.get<ApiResponse<Consultation>>(
      `/api/v1/consultations/${consultationId}`
    );
    return this.handleResponse(response.data);
  }

  /**
   * List consultations with optional filters and pagination
   *
   * @param params - Pagination and filter parameters
   * @returns Paginated list of consultations
   */
  async listConsultations(
    params?: PaginationParams & FilterParams
  ): Promise<PaginatedResponse<Consultation>> {
    const response = await this.httpClient.get<ApiResponse<PaginatedResponse<Consultation>>>(
      '/api/v1/consultations',
      { params }
    );
    return this.handleResponse(response.data);
  }

  /**
   * Add a comment to a consultation
   *
   * @param consultationId - Consultation identifier
   * @param request - Comment parameters
   * @returns Created comment object
   */
  async addConsultationComment(
    consultationId: string,
    request: AddCommentRequest
  ): Promise<ConsultationComment> {
    const response = await this.httpClient.post<ApiResponse<ConsultationComment>>(
      `/api/v1/consultations/${consultationId}/comments`,
      request
    );
    return this.handleResponse(response.data);
  }

  /**
   * Get comments for a consultation
   *
   * @param consultationId - Consultation identifier
   * @param params - Pagination parameters
   * @returns Paginated list of comments
   */
  async getConsultationComments(
    consultationId: string,
    params?: PaginationParams
  ): Promise<PaginatedResponse<ConsultationComment>> {
    const response = await this.httpClient.get<
      ApiResponse<PaginatedResponse<ConsultationComment>>
    >(`/api/v1/consultations/${consultationId}/comments`, { params });
    return this.handleResponse(response.data);
  }

  // ============================================================================
  // Participatory Budgeting Methods
  // ============================================================================

  /**
   * Create a budget proposal
   *
   * @param request - Budget proposal creation parameters
   * @returns Created budget proposal object
   *
   * @example
   * ```typescript
   * const proposal = await civic.createBudgetProposal({
   *   projectName: 'Community Center Renovation',
   *   description: 'Modernize the community center...',
   *   category: 'infrastructure',
   *   location: {
   *     address: '123 Main St',
   *     coordinates: { lat: 37.7749, lng: -122.4194 }
   *   },
   *   budget: {
   *     requested: 5000000 // $50,000 in cents
   *   },
   *   timeline: {
   *     proposedStart: new Date('2025-06-01'),
   *     proposedEnd: new Date('2025-12-31')
   *   }
   * });
   * ```
   */
  async createBudgetProposal(request: CreateBudgetProposalRequest): Promise<BudgetProposal> {
    const response = await this.httpClient.post<ApiResponse<BudgetProposal>>(
      '/api/v1/budgets/proposals',
      request
    );
    return this.handleResponse(response.data);
  }

  /**
   * Get budget proposal by ID
   *
   * @param proposalId - Budget proposal identifier
   * @returns Budget proposal object
   */
  async getBudgetProposal(proposalId: string): Promise<BudgetProposal> {
    const response = await this.httpClient.get<ApiResponse<BudgetProposal>>(
      `/api/v1/budgets/proposals/${proposalId}`
    );
    return this.handleResponse(response.data);
  }

  /**
   * List budget proposals with optional filters and pagination
   *
   * @param params - Pagination and filter parameters
   * @returns Paginated list of budget proposals
   */
  async listBudgetProposals(
    params?: PaginationParams & FilterParams
  ): Promise<PaginatedResponse<BudgetProposal>> {
    const response = await this.httpClient.get<ApiResponse<PaginatedResponse<BudgetProposal>>>(
      '/api/v1/budgets/proposals',
      { params }
    );
    return this.handleResponse(response.data);
  }

  /**
   * Vote for a budget proposal
   *
   * @param proposalId - Budget proposal identifier
   * @param request - Vote parameters
   * @returns Updated budget proposal object
   */
  async voteBudgetProposal(
    proposalId: string,
    request: VoteBudgetProposalRequest
  ): Promise<BudgetProposal> {
    const response = await this.httpClient.post<ApiResponse<BudgetProposal>>(
      `/api/v1/budgets/proposals/${proposalId}/vote`,
      request
    );
    return this.handleResponse(response.data);
  }

  // ============================================================================
  // WebSocket / Real-time Events
  // ============================================================================

  private initWebSocket(): void {
    if (!this.config.websocket.url) return;

    this.wsClient = new WebSocket(this.config.websocket.url, {
      headers: {
        Authorization: `Bearer ${this.config.apiKey}`,
      },
    });

    this.wsClient.on('open', () => {
      console.log('[WIA] WebSocket connected');
    });

    this.wsClient.on('message', (data: string) => {
      try {
        const event: CivicEvent = JSON.parse(data);
        this.handleEvent(event);
      } catch (error) {
        console.error('[WIA] Failed to parse WebSocket message:', error);
      }
    });

    this.wsClient.on('error', (error) => {
      console.error('[WIA] WebSocket error:', error);
    });

    this.wsClient.on('close', () => {
      console.log('[WIA] WebSocket disconnected');
    });
  }

  private handleEvent(event: CivicEvent): void {
    const handlers = this.eventHandlers.get(event.type);
    if (handlers) {
      handlers.forEach((handler) => handler(event));
    }
  }

  /**
   * Subscribe to real-time events
   *
   * @param eventType - Type of event to listen for
   * @param handler - Event handler function
   *
   * @example
   * ```typescript
   * civic.on('vote_cast', (event) => {
   *   console.log('New vote cast:', event.data);
   * });
   * ```
   */
  on(eventType: EventType, handler: (event: CivicEvent) => void): void {
    if (!this.eventHandlers.has(eventType)) {
      this.eventHandlers.set(eventType, new Set());
    }
    this.eventHandlers.get(eventType)!.add(handler);

    // Subscribe via WebSocket
    if (this.wsClient && this.wsClient.readyState === WebSocket.OPEN) {
      this.wsClient.send(
        JSON.stringify({
          action: 'subscribe',
          eventType,
        })
      );
    }
  }

  /**
   * Unsubscribe from real-time events
   *
   * @param eventType - Type of event to stop listening for
   * @param handler - Event handler function to remove
   */
  off(eventType: EventType, handler: (event: CivicEvent) => void): void {
    const handlers = this.eventHandlers.get(eventType);
    if (handlers) {
      handlers.delete(handler);
    }

    // Unsubscribe via WebSocket
    if (this.wsClient && this.wsClient.readyState === WebSocket.OPEN) {
      this.wsClient.send(
        JSON.stringify({
          action: 'unsubscribe',
          eventType,
        })
      );
    }
  }

  // ============================================================================
  // Utility Methods
  // ============================================================================

  private handleResponse<T>(response: ApiResponse<T>): T {
    if (!response.success || !response.data) {
      throw new Error(
        response.error?.message || 'API request failed'
      );
    }
    return response.data;
  }

  /**
   * Close connections and cleanup resources
   */
  async disconnect(): Promise<void> {
    if (this.wsClient) {
      this.wsClient.close();
      this.wsClient = undefined;
    }
  }
}

/**
 * Create a new Civic Participation client
 *
 * @param config - Configuration options
 * @returns CivicParticipation instance
 *
 * @example
 * ```typescript
 * import { createCivicParticipation } from '@wia/civic-participation';
 *
 * const civic = createCivicParticipation({
 *   apiKey: process.env.WIA_API_KEY!,
 *   network: 'mainnet'
 * });
 * ```
 */
export function createCivicParticipation(
  config: CivicParticipationConfig
): CivicParticipation {
  return new CivicParticipation(config);
}

// Default export
export default CivicParticipation;
