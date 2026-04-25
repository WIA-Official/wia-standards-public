/**
 * WIA-SOIL-MICROBIOME TypeScript SDK
 * @version 1.0.0
 * @license MIT
 * @description SDK for interacting with the WIA-SOIL-MICROBIOME Standard API
 *
 * 弘益人間 (Benefit All Humanity)
 */

import {
  WiaSoilMicrobiomeConfig,
  SoilMicrobiomeReport,
  SampleSummary,
  SubmitSampleRequest,
  MicrobiomeProfile,
  AnalysisRequest,
  SoilHealthIndex,
  CalculateHealthIndexRequest,
  CarbonSequestration,
  Intervention,
  InterventionType,
  PaginatedResponse,
  PaginationParams,
  ApiError,
  ProtocolMessage,
  Alert,
  SoilSample,
  DiversityIndex,
} from './types';

// Re-export all types
export * from './types';

// ============================================================================
// Constants
// ============================================================================

const DEFAULT_CONFIG: Partial<WiaSoilMicrobiomeConfig> = {
  environment: 'production',
  timeout: 30000,
  retry: {
    maxRetries: 3,
    baseDelay: 1000,
  },
};

const BASE_URLS = {
  production: 'https://api.soil-microbiome.wia.org/v1',
  sandbox: 'https://sandbox.soil-microbiome.wia.org/v1',
};

const WS_URLS = {
  production: 'wss://stream.soil-microbiome.wia.org/v1/ws',
  sandbox: 'wss://sandbox-stream.soil-microbiome.wia.org/v1/ws',
};

// ============================================================================
// HTTP Client
// ============================================================================

class HttpClient {
  private baseUrl: string;
  private apiKey: string;
  private timeout: number;

  constructor(config: WiaSoilMicrobiomeConfig) {
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
        throw new WiaSoilMicrobiomeError(error.error);
      }

      return response.json() as Promise<T>;
    } catch (error) {
      clearTimeout(timeoutId);
      if (error instanceof WiaSoilMicrobiomeError) throw error;
      throw new WiaSoilMicrobiomeError({
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
 * Custom error class for WIA-SOIL-MICROBIOME SDK errors
 */
export class WiaSoilMicrobiomeError extends Error {
  code: string;
  details?: unknown;
  requestId: string;

  constructor(error: ApiError) {
    super(error.message);
    this.name = 'WiaSoilMicrobiomeError';
    this.code = error.code;
    this.details = error.details;
    this.requestId = error.requestId;
  }
}

// ============================================================================
// Samples API
// ============================================================================

class SamplesApi {
  constructor(private http: HttpClient) {}

  /**
   * List all samples
   */
  async list(params?: PaginationParams): Promise<PaginatedResponse<SampleSummary>> {
    const query = new URLSearchParams();
    if (params?.limit) query.set('limit', params.limit.toString());
    if (params?.offset) query.set('offset', params.offset.toString());
    const queryString = query.toString();
    return this.http.get(`/samples${queryString ? `?${queryString}` : ''}`);
  }

  /**
   * Get a specific sample by ID
   */
  async get(sampleId: string): Promise<SoilSample> {
    return this.http.get(`/samples/${sampleId}`);
  }

  /**
   * Submit a new soil sample
   */
  async submit(data: SubmitSampleRequest): Promise<SampleSummary> {
    return this.http.post('/samples', data);
  }

  /**
   * Update an existing sample
   */
  async update(sampleId: string, data: Partial<SoilSample>): Promise<SampleSummary> {
    return this.http.put(`/samples/${sampleId}`, data);
  }

  /**
   * Delete a sample
   */
  async delete(sampleId: string): Promise<{ success: boolean }> {
    return this.http.delete(`/samples/${sampleId}`);
  }

  /**
   * Get sample analysis status
   */
  async getStatus(sampleId: string): Promise<{
    status: 'pending' | 'processing' | 'completed' | 'failed';
    progress?: number;
    estimatedCompletion?: string;
  }> {
    return this.http.get(`/samples/${sampleId}/status`);
  }
}

// ============================================================================
// Microbiome API
// ============================================================================

class MicrobiomeApi {
  constructor(private http: HttpClient) {}

  /**
   * Get microbiome profile for a sample
   */
  async getProfile(sampleId: string): Promise<MicrobiomeProfile> {
    return this.http.get(`/samples/${sampleId}/microbiome`);
  }

  /**
   * Request new analysis
   */
  async requestAnalysis(data: AnalysisRequest): Promise<{
    analysisId: string;
    status: string;
    estimatedCompletion: string;
  }> {
    return this.http.post('/microbiome/analyze', data);
  }

  /**
   * Get diversity metrics for a sample
   */
  async getDiversity(sampleId: string): Promise<DiversityIndex> {
    return this.http.get(`/samples/${sampleId}/microbiome/diversity`);
  }

  /**
   * Compare multiple samples
   */
  async compare(sampleIds: string[]): Promise<{
    betaDiversity: Record<string, number>;
    similarities: Array<{
      sample1: string;
      sample2: string;
      similarity: number;
    }>;
  }> {
    return this.http.post('/microbiome/compare', { sampleIds });
  }

  /**
   * Get functional group analysis
   */
  async getFunctionalGroups(sampleId: string): Promise<{
    groups: Array<{
      name: string;
      abundance: number;
      genes: string[];
    }>;
  }> {
    return this.http.get(`/samples/${sampleId}/microbiome/functional-groups`);
  }
}

// ============================================================================
// Health Index API
// ============================================================================

class HealthIndexApi {
  constructor(private http: HttpClient) {}

  /**
   * Calculate soil health index
   */
  async calculate(data: CalculateHealthIndexRequest): Promise<SoilHealthIndex> {
    return this.http.post('/health-index/calculate', data);
  }

  /**
   * Get health index for a sample
   */
  async get(sampleId: string): Promise<SoilHealthIndex> {
    return this.http.get(`/samples/${sampleId}/health-index`);
  }

  /**
   * Get health trends over time
   */
  async getTrends(
    sampleIds: string[],
    params?: { startDate?: string; endDate?: string }
  ): Promise<{
    trends: Array<{
      date: string;
      score: number;
      status: string;
    }>;
  }> {
    const query = new URLSearchParams();
    if (params?.startDate) query.set('startDate', params.startDate);
    if (params?.endDate) query.set('endDate', params.endDate);
    const queryString = query.toString();
    return this.http.post(
      `/health-index/trends${queryString ? `?${queryString}` : ''}`,
      { sampleIds }
    );
  }
}

// ============================================================================
// Carbon API
// ============================================================================

class CarbonApi {
  constructor(private http: HttpClient) {}

  /**
   * Submit carbon sequestration data
   */
  async submitSequestration(
    sampleId: string,
    data: Omit<CarbonSequestration, 'netChange' | 'annualRate'>
  ): Promise<CarbonSequestration> {
    return this.http.post(`/samples/${sampleId}/carbon`, data);
  }

  /**
   * Get carbon sequestration data
   */
  async getSequestration(sampleId: string): Promise<CarbonSequestration> {
    return this.http.get(`/samples/${sampleId}/carbon`);
  }

  /**
   * Calculate carbon credits
   */
  async calculateCredits(sampleId: string, params: {
    startDate: string;
    endDate: string;
    area: number; // hectares
  }): Promise<{
    totalCredits: number;
    annualCredits: number;
    confidence: number;
    methodology: string;
  }> {
    return this.http.post(`/samples/${sampleId}/carbon/credits`, params);
  }

  /**
   * Get carbon trends across multiple samples
   */
  async getTrends(params: {
    sampleIds: string[];
    startDate?: string;
    endDate?: string;
  }): Promise<{
    trends: Array<{
      date: string;
      carbonLevel: number;
      sequestrationRate: number;
    }>;
  }> {
    return this.http.post('/carbon/trends', params);
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
    sampleId: string,
    data: Omit<Intervention, 'id' | 'active'>
  ): Promise<Intervention> {
    return this.http.post(`/samples/${sampleId}/interventions`, data);
  }

  /**
   * List interventions for a sample
   */
  async list(sampleId: string): Promise<Intervention[]> {
    return this.http.get(`/samples/${sampleId}/interventions`);
  }

  /**
   * Update an intervention
   */
  async update(
    sampleId: string,
    interventionId: string,
    data: Partial<Intervention>
  ): Promise<Intervention> {
    return this.http.put(`/samples/${sampleId}/interventions/${interventionId}`, data);
  }

  /**
   * Delete an intervention
   */
  async delete(
    sampleId: string,
    interventionId: string
  ): Promise<{ success: boolean }> {
    return this.http.delete(`/samples/${sampleId}/interventions/${interventionId}`);
  }

  /**
   * Get intervention effectiveness
   */
  async getEffectiveness(
    sampleId: string,
    interventionId: string
  ): Promise<{
    intervention: Intervention;
    beforeMetrics: SoilHealthIndex;
    afterMetrics: SoilHealthIndex;
    improvement: number;
  }> {
    return this.http.get(`/samples/${sampleId}/interventions/${interventionId}/effectiveness`);
  }
}

// ============================================================================
// Reports API
// ============================================================================

class ReportsApi {
  constructor(private http: HttpClient) {}

  /**
   * Generate complete soil microbiome report
   */
  async generate(sampleId: string): Promise<SoilMicrobiomeReport> {
    return this.http.post('/reports/generate', { sampleId });
  }

  /**
   * Get existing report
   */
  async get(reportId: string): Promise<SoilMicrobiomeReport> {
    return this.http.get(`/reports/${reportId}`);
  }

  /**
   * List reports
   */
  async list(params?: PaginationParams & {
    sampleId?: string;
    startDate?: string;
    endDate?: string;
  }): Promise<PaginatedResponse<{
    id: string;
    sampleId: string;
    createdAt: string;
    status: string;
  }>> {
    const query = new URLSearchParams();
    if (params?.limit) query.set('limit', params.limit.toString());
    if (params?.offset) query.set('offset', params.offset.toString());
    if (params?.sampleId) query.set('sampleId', params.sampleId);
    if (params?.startDate) query.set('startDate', params.startDate);
    if (params?.endDate) query.set('endDate', params.endDate);
    const queryString = query.toString();
    return this.http.get(`/reports${queryString ? `?${queryString}` : ''}`);
  }

  /**
   * Export report in different formats
   */
  async export(reportId: string, format: 'pdf' | 'json' | 'csv'): Promise<Blob> {
    const response = await fetch(
      `${this.http['baseUrl']}/reports/${reportId}/export?format=${format}`,
      {
        headers: {
          'Authorization': `Bearer ${this.http['apiKey']}`,
        },
      }
    );
    return response.blob();
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

  constructor(config: WiaSoilMicrobiomeConfig) {
    this.url = WS_URLS[config.environment || 'production'];
    this.apiKey = config.apiKey;
  }

  /**
   * Connect to the streaming service
   */
  connect(): Promise<void> {
    return new Promise((resolve, reject) => {
      this.ws = new WebSocket(this.url, ['wia-soil-microbiome-v1']);

      this.ws.onopen = () => {
        // Send authentication
        this.send({
          header: {
            version: 'WIA-SOIL-MICROBIOME/1.0',
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
   * Subscribe to sample updates
   */
  subscribeSample(sampleId: string): void {
    this.send({
      header: {
        version: 'WIA-SOIL-MICROBIOME/1.0',
        messageId: `sub_${Date.now()}`,
        type: 'subscription.add',
        timestamp: new Date().toISOString(),
      },
      payload: { sampleId },
    });
  }

  /**
   * Unsubscribe from sample updates
   */
  unsubscribeSample(sampleId: string): void {
    this.send({
      header: {
        version: 'WIA-SOIL-MICROBIOME/1.0',
        messageId: `unsub_${Date.now()}`,
        type: 'subscription.remove',
        timestamp: new Date().toISOString(),
      },
      payload: { sampleId },
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
// Local Calculations
// ============================================================================

/**
 * Calculate Shannon diversity index
 */
export function calculateShannonIndex(abundances: number[]): number {
  const total = abundances.reduce((sum, a) => sum + a, 0);
  if (total === 0) return 0;

  const proportions = abundances.map(a => a / total);
  return -proportions.reduce((sum, p) => {
    if (p === 0) return sum;
    return sum + (p * Math.log(p));
  }, 0);
}

/**
 * Calculate Simpson diversity index
 */
export function calculateSimpsonIndex(abundances: number[]): number {
  const total = abundances.reduce((sum, a) => sum + a, 0);
  if (total === 0) return 0;

  const proportions = abundances.map(a => a / total);
  return 1 - proportions.reduce((sum, p) => sum + (p * p), 0);
}

/**
 * Calculate species richness
 */
export function calculateRichness(abundances: number[]): number {
  return abundances.filter(a => a > 0).length;
}

// ============================================================================
// Main Client
// ============================================================================

/**
 * WIA-SOIL-MICROBIOME SDK Client
 *
 * @example
 * ```typescript
 * import { WiaSoilMicrobiomeClient } from '@wia/soil-microbiome-sdk';
 *
 * const client = new WiaSoilMicrobiomeClient({
 *   apiKey: 'your-api-key',
 *   environment: 'production'
 * });
 *
 * // Submit a sample
 * const sample = await client.samples.submit({
 *   sample: {
 *     collectionDate: new Date().toISOString(),
 *     location: { latitude: 37.7749, longitude: -122.4194 },
 *     soilType: 'agricultural',
 *     depth: 'topsoil-0-15cm'
 *   },
 *   analysisMethods: ['16s-rrna-sequencing']
 * });
 *
 * console.log(`Sample ID: ${sample.id}`);
 * ```
 */
export class WiaSoilMicrobiomeClient {
  private http: HttpClient;

  /** Samples API */
  public samples: SamplesApi;

  /** Microbiome API */
  public microbiome: MicrobiomeApi;

  /** Health Index API */
  public healthIndex: HealthIndexApi;

  /** Carbon API */
  public carbon: CarbonApi;

  /** Interventions API */
  public interventions: InterventionsApi;

  /** Reports API */
  public reports: ReportsApi;

  /** Streaming client for real-time data */
  public streaming: StreamingClient;

  constructor(config: WiaSoilMicrobiomeConfig) {
    const mergedConfig = { ...DEFAULT_CONFIG, ...config } as WiaSoilMicrobiomeConfig;
    this.http = new HttpClient(mergedConfig);
    this.samples = new SamplesApi(this.http);
    this.microbiome = new MicrobiomeApi(this.http);
    this.healthIndex = new HealthIndexApi(this.http);
    this.carbon = new CarbonApi(this.http);
    this.interventions = new InterventionsApi(this.http);
    this.reports = new ReportsApi(this.http);
    this.streaming = new StreamingClient(mergedConfig);
  }
}

// Default export
export default WiaSoilMicrobiomeClient;
