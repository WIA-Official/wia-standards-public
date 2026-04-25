/**
 * WIA-AGING TypeScript SDK
 * @version 1.0.0
 * @license MIT
 * @description SDK for interacting with the WIA-AGING Standard API
 *
 * 弘益人間 (Benefit All Humanity)
 */

import {
  WiaAgingConfig,
  AgingProfile,
  ProfileSummary,
  CreateProfileRequest,
  AssessmentResult,
  CreateAssessmentRequest,
  Biomarker,
  Intervention,
  PaginatedResponse,
  PaginationParams,
  ApiError,
  ProtocolMessage,
  BiomarkerUpdateEvent,
  BiomarkerAck,
  Alert,
  BiologicalAge,
  BiologicalAgeMethod,
} from './types';

// Re-export all types
export * from './types';

// ============================================================================
// Constants
// ============================================================================

const DEFAULT_CONFIG: Partial<WiaAgingConfig> = {
  environment: 'production',
  timeout: 30000,
  retry: {
    maxRetries: 3,
    baseDelay: 1000,
  },
};

const BASE_URLS = {
  production: 'https://api.aging.wia.org/v1',
  sandbox: 'https://sandbox.aging.wia.org/v1',
};

const WS_URLS = {
  production: 'wss://stream.aging.wia.org/v1/ws',
  sandbox: 'wss://sandbox-stream.aging.wia.org/v1/ws',
};

// ============================================================================
// HTTP Client
// ============================================================================

class HttpClient {
  private baseUrl: string;
  private apiKey: string;
  private timeout: number;

  constructor(config: WiaAgingConfig) {
    this.baseUrl = config.baseUrl || BASE_URLS[config.environment || 'production'];
    this.apiKey = config.apiKey;
    this.timeout = config.timeout || DEFAULT_CONFIG.timeout!;
  }

  private async request<T>(
    method: string,
    path: string,
    body?: unknown
  ): Promise<T> {
    const url = `${this.baseUrl}${path}`;
    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), this.timeout);

    try {
      const response = await fetch(url, {
        method,
        headers: {
          'Authorization': `Bearer ${this.apiKey}`,
          'Content-Type': 'application/json',
          'Accept': 'application/json',
        },
        body: body ? JSON.stringify(body) : undefined,
        signal: controller.signal,
      });

      clearTimeout(timeoutId);

      if (!response.ok) {
        const error = await response.json() as { error: ApiError };
        throw new WiaAgingError(error.error);
      }

      return response.json() as Promise<T>;
    } catch (error) {
      clearTimeout(timeoutId);
      if (error instanceof WiaAgingError) throw error;
      throw new WiaAgingError({
        code: 'NETWORK_ERROR',
        message: error instanceof Error ? error.message : 'Network request failed',
        requestId: 'unknown',
      });
    }
  }

  get<T>(path: string): Promise<T> {
    return this.request<T>('GET', path);
  }

  post<T>(path: string, body: unknown): Promise<T> {
    return this.request<T>('POST', path, body);
  }

  put<T>(path: string, body: unknown): Promise<T> {
    return this.request<T>('PUT', path, body);
  }

  delete<T>(path: string): Promise<T> {
    return this.request<T>('DELETE', path);
  }
}

// ============================================================================
// Error Classes
// ============================================================================

/**
 * Custom error class for WIA-AGING SDK errors
 */
export class WiaAgingError extends Error {
  code: string;
  details?: unknown;
  requestId: string;

  constructor(error: ApiError) {
    super(error.message);
    this.name = 'WiaAgingError';
    this.code = error.code;
    this.details = error.details;
    this.requestId = error.requestId;
  }
}

// ============================================================================
// Profiles API
// ============================================================================

class ProfilesApi {
  constructor(private http: HttpClient) {}

  /**
   * List all profiles for the authenticated user
   */
  async list(params?: PaginationParams): Promise<PaginatedResponse<ProfileSummary>> {
    const query = new URLSearchParams();
    if (params?.limit) query.set('limit', params.limit.toString());
    if (params?.offset) query.set('offset', params.offset.toString());
    const queryString = query.toString();
    return this.http.get(`/profiles${queryString ? `?${queryString}` : ''}`);
  }

  /**
   * Get a specific profile by ID
   */
  async get(profileId: string): Promise<AgingProfile> {
    return this.http.get(`/profiles/${profileId}`);
  }

  /**
   * Create a new profile
   */
  async create(data: CreateProfileRequest): Promise<ProfileSummary> {
    return this.http.post('/profiles', data);
  }

  /**
   * Update an existing profile
   */
  async update(profileId: string, data: Partial<CreateProfileRequest>): Promise<ProfileSummary> {
    return this.http.put(`/profiles/${profileId}`, data);
  }

  /**
   * Delete a profile
   */
  async delete(profileId: string): Promise<{ success: boolean }> {
    return this.http.delete(`/profiles/${profileId}`);
  }
}

// ============================================================================
// Assessments API
// ============================================================================

class AssessmentsApi {
  constructor(private http: HttpClient) {}

  /**
   * Create a new biological age assessment
   */
  async create(profileId: string, data: CreateAssessmentRequest): Promise<AssessmentResult> {
    return this.http.post(`/profiles/${profileId}/assessments`, data);
  }

  /**
   * List assessments for a profile
   */
  async list(
    profileId: string,
    params?: PaginationParams
  ): Promise<PaginatedResponse<AssessmentResult>> {
    const query = new URLSearchParams();
    if (params?.limit) query.set('limit', params.limit.toString());
    if (params?.offset) query.set('offset', params.offset.toString());
    const queryString = query.toString();
    return this.http.get(`/profiles/${profileId}/assessments${queryString ? `?${queryString}` : ''}`);
  }

  /**
   * Get a specific assessment
   */
  async get(profileId: string, assessmentId: string): Promise<AssessmentResult> {
    return this.http.get(`/profiles/${profileId}/assessments/${assessmentId}`);
  }
}

// ============================================================================
// Biomarkers API
// ============================================================================

interface BiomarkerCatalogItem {
  code: string;
  name: string;
  unit: string;
  category: string;
  referenceRange: { low: number; high: number };
}

class BiomarkersApi {
  constructor(private http: HttpClient) {}

  /**
   * Submit biomarkers for a profile
   */
  async submit(
    profileId: string,
    biomarkers: Omit<Biomarker, 'status'>[],
    source?: string
  ): Promise<{ success: boolean; count: number }> {
    return this.http.post('/biomarkers/batch', {
      profileId,
      biomarkers,
      source,
    });
  }

  /**
   * Get the biomarker catalog
   */
  async catalog(): Promise<BiomarkerCatalogItem[]> {
    return this.http.get('/biomarkers/catalog');
  }

  /**
   * Get biomarkers for a profile
   */
  async list(
    profileId: string,
    params?: PaginationParams & { category?: string }
  ): Promise<PaginatedResponse<Biomarker>> {
    const query = new URLSearchParams();
    if (params?.limit) query.set('limit', params.limit.toString());
    if (params?.offset) query.set('offset', params.offset.toString());
    if (params?.category) query.set('category', params.category);
    const queryString = query.toString();
    return this.http.get(`/profiles/${profileId}/biomarkers${queryString ? `?${queryString}` : ''}`);
  }
}

// ============================================================================
// Interventions API
// ============================================================================

class InterventionsApi {
  constructor(private http: HttpClient) {}

  /**
   * Create a new intervention
   */
  async create(
    profileId: string,
    data: Omit<Intervention, 'id' | 'active'>
  ): Promise<Intervention> {
    return this.http.post(`/profiles/${profileId}/interventions`, data);
  }

  /**
   * List interventions for a profile
   */
  async list(profileId: string): Promise<Intervention[]> {
    return this.http.get(`/profiles/${profileId}/interventions`);
  }

  /**
   * Update an intervention
   */
  async update(
    profileId: string,
    interventionId: string,
    data: Partial<Intervention>
  ): Promise<Intervention> {
    return this.http.put(`/profiles/${profileId}/interventions/${interventionId}`, data);
  }

  /**
   * Delete an intervention
   */
  async delete(
    profileId: string,
    interventionId: string
  ): Promise<{ success: boolean }> {
    return this.http.delete(`/profiles/${profileId}/interventions/${interventionId}`);
  }
}

// ============================================================================
// Streaming Client
// ============================================================================

type StreamEventHandler = (event: ProtocolMessage) => void;
type StreamErrorHandler = (error: Error) => void;

class StreamingClient {
  private ws: WebSocket | null = null;
  private url: string;
  private apiKey: string;
  private handlers: Map<string, StreamEventHandler[]> = new Map();
  private errorHandler: StreamErrorHandler | null = null;
  private reconnectAttempts = 0;
  private maxReconnectAttempts = 5;

  constructor(config: WiaAgingConfig) {
    this.url = WS_URLS[config.environment || 'production'];
    this.apiKey = config.apiKey;
  }

  /**
   * Connect to the streaming service
   */
  connect(): Promise<void> {
    return new Promise((resolve, reject) => {
      this.ws = new WebSocket(this.url, ['wia-aging-v1']);

      this.ws.onopen = () => {
        // Send authentication
        this.send({
          header: {
            version: 'WIA-AGING/1.0',
            messageId: `auth_${Date.now()}`,
            type: 'auth.request',
            timestamp: new Date().toISOString(),
          },
          payload: { token: this.apiKey },
        });
        this.reconnectAttempts = 0;
        resolve();
      };

      this.ws.onerror = (event) => {
        const error = new Error('WebSocket connection error');
        if (this.errorHandler) this.errorHandler(error);
        reject(error);
      };

      this.ws.onmessage = (event) => {
        try {
          const message = JSON.parse(event.data) as ProtocolMessage;
          const handlers = this.handlers.get(message.header.type) || [];
          handlers.forEach((handler) => handler(message));

          // Also emit to '*' handlers
          const allHandlers = this.handlers.get('*') || [];
          allHandlers.forEach((handler) => handler(message));
        } catch (error) {
          if (this.errorHandler) {
            this.errorHandler(error instanceof Error ? error : new Error('Parse error'));
          }
        }
      };

      this.ws.onclose = () => {
        if (this.reconnectAttempts < this.maxReconnectAttempts) {
          this.reconnectAttempts++;
          setTimeout(() => this.connect(), 1000 * Math.pow(2, this.reconnectAttempts));
        }
      };
    });
  }

  /**
   * Disconnect from the streaming service
   */
  disconnect(): void {
    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }
  }

  /**
   * Send a message
   */
  send(message: ProtocolMessage): void {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(message));
    } else {
      throw new Error('WebSocket is not connected');
    }
  }

  /**
   * Subscribe to biomarker updates
   */
  sendBiomarkerUpdate(event: BiomarkerUpdateEvent): void {
    this.send({
      header: {
        version: 'WIA-AGING/1.0',
        messageId: `bio_${Date.now()}`,
        type: 'biomarker.update',
        timestamp: new Date().toISOString(),
      },
      payload: event,
    });
  }

  /**
   * Register an event handler
   */
  on(eventType: string, handler: StreamEventHandler): void {
    const handlers = this.handlers.get(eventType) || [];
    handlers.push(handler);
    this.handlers.set(eventType, handlers);
  }

  /**
   * Remove an event handler
   */
  off(eventType: string, handler: StreamEventHandler): void {
    const handlers = this.handlers.get(eventType) || [];
    const index = handlers.indexOf(handler);
    if (index > -1) {
      handlers.splice(index, 1);
      this.handlers.set(eventType, handlers);
    }
  }

  /**
   * Set error handler
   */
  onError(handler: StreamErrorHandler): void {
    this.errorHandler = handler;
  }
}

// ============================================================================
// Biological Age Calculator (Local)
// ============================================================================

/**
 * Calculate biological age using Levine's Phenotypic Age formula
 * This is a simplified local implementation for quick estimates
 */
export function calculatePhenotypicAge(
  chronologicalAge: number,
  biomarkers: {
    albumin?: number;      // g/dL
    creatinine?: number;   // mg/dL
    glucose?: number;      // mg/dL
    crp?: number;          // mg/L (log-transformed)
    lymphocyte?: number;   // %
    mcv?: number;          // fL
    rdw?: number;          // %
    alkalinePhosphatase?: number;  // U/L
    wbc?: number;          // 10^3 cells/µL
  }
): BiologicalAge {
  // Simplified Phenotypic Age calculation
  let phenoAge = chronologicalAge;

  if (biomarkers.albumin !== undefined) {
    phenoAge += (4.5 - biomarkers.albumin) * 5;
  }
  if (biomarkers.creatinine !== undefined) {
    phenoAge += (biomarkers.creatinine - 0.9) * 8;
  }
  if (biomarkers.glucose !== undefined) {
    phenoAge += (biomarkers.glucose - 90) * 0.05;
  }
  if (biomarkers.crp !== undefined) {
    phenoAge += biomarkers.crp * 0.8;
  }
  if (biomarkers.lymphocyte !== undefined) {
    phenoAge -= (biomarkers.lymphocyte - 25) * 0.3;
  }

  phenoAge = Math.round(phenoAge * 10) / 10;

  return {
    value: phenoAge,
    method: 'phenotypic-levine' as BiologicalAgeMethod,
    confidence: 0.85,
    ageDifference: Math.round((phenoAge - chronologicalAge) * 10) / 10,
    agingRate: Math.round((phenoAge / chronologicalAge) * 100) / 100,
    timestamp: new Date().toISOString(),
  };
}

// ============================================================================
// Main Client
// ============================================================================

/**
 * WIA-AGING SDK Client
 *
 * @example
 * ```typescript
 * import { WiaAgingClient } from '@wia/aging-sdk';
 *
 * const client = new WiaAgingClient({
 *   apiKey: 'your-api-key',
 *   environment: 'production'
 * });
 *
 * // Create an assessment
 * const assessment = await client.assessments.create('profile_id', {
 *   method: 'phenotypic-levine',
 *   biomarkers: [
 *     { code: 'WIA-AGE-001', value: 0.8, unit: 'mg/L', timestamp: new Date().toISOString() }
 *   ]
 * });
 *
 * console.log(`Biological Age: ${assessment.biologicalAge.value}`);
 * ```
 */
export class WiaAgingClient {
  private http: HttpClient;

  /** Profiles API */
  public profiles: ProfilesApi;

  /** Assessments API */
  public assessments: AssessmentsApi;

  /** Biomarkers API */
  public biomarkers: BiomarkersApi;

  /** Interventions API */
  public interventions: InterventionsApi;

  /** Streaming client for real-time data */
  public streaming: StreamingClient;

  constructor(config: WiaAgingConfig) {
    const mergedConfig = { ...DEFAULT_CONFIG, ...config } as WiaAgingConfig;
    this.http = new HttpClient(mergedConfig);
    this.profiles = new ProfilesApi(this.http);
    this.assessments = new AssessmentsApi(this.http);
    this.biomarkers = new BiomarkersApi(this.http);
    this.interventions = new InterventionsApi(this.http);
    this.streaming = new StreamingClient(mergedConfig);
  }
}

// Default export
export default WiaAgingClient;
