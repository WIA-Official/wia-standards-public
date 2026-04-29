/**
 * WIA-ORGAN-SHORTAGE TypeScript SDK
 * @version 1.0.0
 * @license MIT
 * @description SDK for interacting with the WIA-ORGAN-SHORTAGE Standard API
 *
 * 弘益人間 (Benefit All Humanity)
 */

import {
  WiaOrganShortageConfig,
  OrganProfile,
  PatientSummary,
  CreatePatientRequest,
  EligibilityRequest,
  EligibilityResult,
  MatchRequest,
  MatchResult,
  BioprintOrderRequest,
  BioprintOrder,
  PaginatedResponse,
  PaginationParams,
  ApiError,
  ProtocolMessage,
  OrganType,
} from './types';

export * from './types';

// ============================================================================
// Constants
// ============================================================================

const DEFAULT_CONFIG: Partial<WiaOrganShortageConfig> = {
  environment: 'production',
  timeout: 30000,
  retry: {
    maxRetries: 3,
    baseDelay: 1000,
  },
};

const BASE_URLS = {
  production: 'https://api.organ-shortage.wia.org/v1',
  sandbox: 'https://sandbox.organ-shortage.wia.org/v1',
};

const WS_URLS = {
  production: 'wss://stream.organ-shortage.wia.org/v1/ws',
  sandbox: 'wss://sandbox-stream.organ-shortage.wia.org/v1/ws',
};

// ============================================================================
// HTTP Client
// ============================================================================

class HttpClient {
  private baseUrl: string;
  private apiKey: string;
  private timeout: number;

  constructor(config: WiaOrganShortageConfig) {
    this.baseUrl = config.baseUrl || BASE_URLS[config.environment || 'production'];
    this.apiKey = config.apiKey;
    this.timeout = config.timeout || DEFAULT_CONFIG.timeout!;
  }

  private async request<T>(method: string, path: string, body?: unknown): Promise<T> {
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
        throw new WiaOrganShortageError(error.error);
      }

      return response.json() as Promise<T>;
    } catch (error) {
      clearTimeout(timeoutId);
      if (error instanceof WiaOrganShortageError) throw error;
      throw new WiaOrganShortageError({
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

export class WiaOrganShortageError extends Error {
  code: string;
  details?: Record<string, unknown>;
  requestId: string;

  constructor(error: ApiError) {
    super(error.message);
    this.name = 'WiaOrganShortageError';
    this.code = error.code;
    this.details = error.details;
    this.requestId = error.requestId;
  }
}

// ============================================================================
// Patients API
// ============================================================================

class PatientsApi {
  constructor(private http: HttpClient) {}

  async list(params?: PaginationParams & { organ?: OrganType; urgency?: string }): Promise<PaginatedResponse<PatientSummary>> {
    const query = new URLSearchParams();
    if (params?.limit) query.set('limit', params.limit.toString());
    if (params?.offset) query.set('offset', params.offset.toString());
    if (params?.organ) query.set('organ', params.organ);
    if (params?.urgency) query.set('urgency', params.urgency);
    const queryString = query.toString();
    return this.http.get(`/waitlist/patients${queryString ? `?${queryString}` : ''}`);
  }

  async get(patientId: string): Promise<OrganProfile> {
    return this.http.get(`/waitlist/patients/${patientId}`);
  }

  async create(data: CreatePatientRequest): Promise<PatientSummary> {
    return this.http.post('/waitlist/patients', data);
  }

  async update(patientId: string, data: Partial<CreatePatientRequest>): Promise<PatientSummary> {
    return this.http.put(`/waitlist/patients/${patientId}`, data);
  }

  async delete(patientId: string): Promise<{ success: boolean }> {
    return this.http.delete(`/waitlist/patients/${patientId}`);
  }
}

// ============================================================================
// Alternatives API
// ============================================================================

class AlternativesApi {
  constructor(private http: HttpClient) {}

  async evaluateXenotransplant(data: EligibilityRequest): Promise<EligibilityResult> {
    return this.http.post('/xenotransplant/eligibility', data);
  }

  async listTrials(organ?: OrganType): Promise<{ trials: Array<{ id: string; sponsor: string; organ: string; phase: string; status: string }> }> {
    const query = organ ? `?organ=${organ}` : '';
    return this.http.get(`/xenotransplant/trials${query}`);
  }

  async evaluate(patientId: string): Promise<{
    xenotransplant: { eligible: boolean; reasons: string[] };
    bioprinted: { eligible: boolean; estimatedAvailability: string };
    traditional: { livingDonor: boolean; waitlistPosition: number };
  }> {
    return this.http.get(`/alternatives/${patientId}/evaluate`);
  }
}

// ============================================================================
// Bioprint API
// ============================================================================

class BioprintApi {
  constructor(private http: HttpClient) {}

  async order(data: BioprintOrderRequest): Promise<BioprintOrder> {
    return this.http.post('/bioprint/order', data);
  }

  async status(orderId: string): Promise<BioprintOrder> {
    return this.http.get(`/bioprint/status/${orderId}`);
  }

  async cancel(orderId: string): Promise<{ success: boolean }> {
    return this.http.delete(`/bioprint/orders/${orderId}`);
  }
}

// ============================================================================
// Matching API
// ============================================================================

class MatchingApi {
  constructor(private http: HttpClient) {}

  async run(data: MatchRequest): Promise<MatchResult> {
    return this.http.post('/match', data);
  }

  async getWaitlistStatus(): Promise<{
    totalWaiting: number;
    byOrgan: Record<OrganType, number>;
    dailyDeaths: number;
    timestamp: string;
  }> {
    return this.http.get('/waitlist/status');
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

  constructor(config: WiaOrganShortageConfig) {
    this.url = WS_URLS[config.environment || 'production'];
    this.apiKey = config.apiKey;
  }

  connect(): Promise<void> {
    return new Promise((resolve, reject) => {
      this.ws = new WebSocket(this.url, ['wia-organ-shortage-v1']);

      this.ws.onopen = () => {
        this.send({
          header: {
            version: 'WIA-ORGAN-SHORTAGE/1.0',
            messageId: `auth_${Date.now()}`,
            type: 'auth.request' as any,
            timestamp: new Date().toISOString(),
          },
          payload: { token: this.apiKey },
        });
        this.reconnectAttempts = 0;
        resolve();
      };

      this.ws.onerror = () => {
        const error = new Error('WebSocket connection error');
        if (this.errorHandler) this.errorHandler(error);
        reject(error);
      };

      this.ws.onmessage = (event) => {
        try {
          const message = JSON.parse(event.data) as ProtocolMessage;
          const handlers = this.handlers.get(message.header.type) || [];
          handlers.forEach((handler) => handler(message));

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

  disconnect(): void {
    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }
  }

  send(message: ProtocolMessage): void {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(message));
    } else {
      throw new Error('WebSocket is not connected');
    }
  }

  subscribe(events: string[], filters?: { organTypes?: OrganType[]; regions?: string[] }): void {
    this.send({
      header: {
        version: 'WIA-ORGAN-SHORTAGE/1.0',
        messageId: `sub_${Date.now()}`,
        type: 'subscription.add' as any,
        timestamp: new Date().toISOString(),
      },
      payload: { events, filters },
    });
  }

  on(eventType: string, handler: StreamEventHandler): void {
    const handlers = this.handlers.get(eventType) || [];
    handlers.push(handler);
    this.handlers.set(eventType, handlers);
  }

  off(eventType: string, handler: StreamEventHandler): void {
    const handlers = this.handlers.get(eventType) || [];
    const index = handlers.indexOf(handler);
    if (index > -1) {
      handlers.splice(index, 1);
      this.handlers.set(eventType, handlers);
    }
  }

  onError(handler: StreamErrorHandler): void {
    this.errorHandler = handler;
  }
}

// ============================================================================
// Main Client
// ============================================================================

/**
 * WIA-ORGAN-SHORTAGE SDK Client
 *
 * @example
 * ```typescript
 * import { WiaOrganShortageClient } from '@wia/organ-shortage-sdk';
 *
 * const client = new WiaOrganShortageClient({
 *   apiKey: 'your-api-key',
 *   environment: 'production'
 * });
 *
 * // Check xenotransplant eligibility
 * const eligibility = await client.alternatives.evaluateXenotransplant({
 *   patientId: 'patient_123',
 *   organNeeded: 'kidney',
 *   praPercent: 65,
 *   waitingYears: 3
 * });
 *
 * console.log(`Eligible: ${eligibility.eligible}`);
 * console.log(`Score: ${eligibility.score}`);
 * ```
 */
export class WiaOrganShortageClient {
  private http: HttpClient;

  public patients: PatientsApi;
  public alternatives: AlternativesApi;
  public bioprint: BioprintApi;
  public matching: MatchingApi;
  public streaming: StreamingClient;

  constructor(config: WiaOrganShortageConfig) {
    const mergedConfig = { ...DEFAULT_CONFIG, ...config } as WiaOrganShortageConfig;
    this.http = new HttpClient(mergedConfig);
    this.patients = new PatientsApi(this.http);
    this.alternatives = new AlternativesApi(this.http);
    this.bioprint = new BioprintApi(this.http);
    this.matching = new MatchingApi(this.http);
    this.streaming = new StreamingClient(mergedConfig);
  }
}

export default WiaOrganShortageClient;
