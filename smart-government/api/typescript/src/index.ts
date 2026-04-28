/**
 * WIA Smart Government Standard (WIA-SOC-006)
 * TypeScript SDK Implementation
 *
 * @version 1.0.0
 * @standard WIA-SOC-006
 * @philosophy 弘益人間 (홍익인간) · Benefit All Humanity
 */

import axios, { AxiosInstance, AxiosError } from 'axios';
import type {
  SmartGovernmentConfig,
  CitizenProfile,
  ServiceRequest,
  ServiceCatalogItem,
  ChatSession,
  AIAssistantConfig,
  AIResponse,
  DashboardMetrics,
  PredictionRequest,
  PredictionResult,
  Integration,
  SmartCityEvent,
  InterAgencyRequest,
  APIResponse,
  PaginatedResponse,
  Webhook,
  WebhookEvent,
  RequestStatus,
  ServiceType,
  ServiceCategory,
} from './types';
import { SmartGovernmentError } from './types';

/**
 * Main SDK class for WIA Smart Government Standard
 */
export class SmartGovernment {
  private client: AxiosInstance;
  private config: SmartGovernmentConfig;

  constructor(config: SmartGovernmentConfig) {
    this.config = {
      baseUrl: config.baseUrl || 'https://api.gov.example/v1',
      timeout: config.timeout || 30000,
      retryAttempts: config.retryAttempts || 3,
      enableLogging: config.enableLogging || false,
      ...config,
    };

    this.client = axios.create({
      baseURL: this.config.baseUrl,
      timeout: this.config.timeout,
      headers: {
        'Authorization': `Bearer ${this.config.apiKey}`,
        'Content-Type': 'application/json',
        'X-WIA-Standard': 'SOC-006',
        'X-WIA-Version': '1.0.0',
      },
    });

    this.setupInterceptors();
  }

  /**
   * Setup request/response interceptors
   */
  private setupInterceptors(): void {
    // Request interceptor
    this.client.interceptors.request.use(
      (config) => {
        if (this.config.enableLogging) {
          console.log(`[SmartGov] ${config.method?.toUpperCase()} ${config.url}`);
        }
        return config;
      },
      (error) => {
        return Promise.reject(this.handleError(error));
      }
    );

    // Response interceptor
    this.client.interceptors.response.use(
      (response) => {
        if (this.config.enableLogging) {
          console.log(`[SmartGov] Response:`, response.status);
        }
        return response;
      },
      async (error) => {
        if (this.config.retryAttempts && error.config && !error.config.__retryCount) {
          error.config.__retryCount = 0;
        }

        if (
          error.config &&
          error.config.__retryCount < (this.config.retryAttempts || 0) &&
          this.shouldRetry(error)
        ) {
          error.config.__retryCount++;
          await this.delay(1000 * error.config.__retryCount);
          return this.client.request(error.config);
        }

        return Promise.reject(this.handleError(error));
      }
    );
  }

  /**
   * Determine if request should be retried
   */
  private shouldRetry(error: AxiosError): boolean {
    return (
      error.response?.status === 429 || // Rate limit
      error.response?.status === 503 || // Service unavailable
      error.code === 'ECONNABORTED' ||
      error.code === 'ETIMEDOUT'
    );
  }

  /**
   * Delay helper for retry logic
   */
  private delay(ms: number): Promise<void> {
    return new Promise((resolve) => setTimeout(resolve, ms));
  }

  /**
   * Handle and transform errors
   */
  private handleError(error: any): SmartGovernmentError {
    if (error instanceof SmartGovernmentError) {
      return error;
    }

    if (axios.isAxiosError(error)) {
      const message = error.response?.data?.error?.message || error.message;
      const code = error.response?.data?.error?.code || 'UNKNOWN_ERROR';
      const statusCode = error.response?.status;
      const details = error.response?.data?.error?.details;

      return new SmartGovernmentError(message, code, statusCode, details);
    }

    return new SmartGovernmentError(error.message || 'Unknown error', 'UNKNOWN_ERROR');
  }

  // ========================================================================
  // Citizen Management
  // ========================================================================

  /**
   * Get citizen profile
   */
  async getCitizen(citizenId: string): Promise<CitizenProfile> {
    const response = await this.client.get<APIResponse<CitizenProfile>>(`/citizens/${citizenId}`);
    if (!response.data.success || !response.data.data) {
      throw new SmartGovernmentError('Failed to fetch citizen profile', 'FETCH_ERROR');
    }
    return response.data.data;
  }

  /**
   * Create or update citizen profile
   */
  async updateCitizen(citizenId: string, profile: Partial<CitizenProfile>): Promise<CitizenProfile> {
    const response = await this.client.put<APIResponse<CitizenProfile>>(
      `/citizens/${citizenId}`,
      profile
    );
    if (!response.data.success || !response.data.data) {
      throw new SmartGovernmentError('Failed to update citizen profile', 'UPDATE_ERROR');
    }
    return response.data.data;
  }

  /**
   * Verify citizen identity
   */
  async verifyCitizen(
    citizenId: string,
    method: 'biometric' | 'document' | 'blockchain',
    data: any
  ): Promise<{ verified: boolean; trustLevel: string }> {
    const response = await this.client.post<APIResponse<any>>(`/citizens/${citizenId}/verify`, {
      method,
      data,
    });
    if (!response.data.success || !response.data.data) {
      throw new SmartGovernmentError('Verification failed', 'VERIFICATION_ERROR');
    }
    return response.data.data;
  }

  // ========================================================================
  // Service Management
  // ========================================================================

  /**
   * Get service catalog
   */
  async getServiceCatalog(filters?: {
    category?: ServiceCategory;
    language?: string;
  }): Promise<ServiceCatalogItem[]> {
    const response = await this.client.get<APIResponse<ServiceCatalogItem[]>>('/services/catalog', {
      params: filters,
    });
    if (!response.data.success || !response.data.data) {
      throw new SmartGovernmentError('Failed to fetch service catalog', 'FETCH_ERROR');
    }
    return response.data.data;
  }

  /**
   * Create service request
   */
  async createServiceRequest(request: {
    serviceType: ServiceType;
    formData: Record<string, any>;
    documents?: Array<{ type: string; url: string; name: string }>;
    priority?: string;
  }): Promise<ServiceRequest> {
    const response = await this.client.post<APIResponse<ServiceRequest>>(
      '/services/requests',
      request
    );
    if (!response.data.success || !response.data.data) {
      throw new SmartGovernmentError('Failed to create service request', 'CREATE_ERROR');
    }
    return response.data.data;
  }

  /**
   * Get service request status
   */
  async getServiceRequest(requestId: string): Promise<ServiceRequest> {
    const response = await this.client.get<APIResponse<ServiceRequest>>(
      `/services/requests/${requestId}`
    );
    if (!response.data.success || !response.data.data) {
      throw new SmartGovernmentError('Failed to fetch service request', 'FETCH_ERROR');
    }
    return response.data.data;
  }

  /**
   * List service requests for a citizen
   */
  async listServiceRequests(
    citizenId: string,
    filters?: {
      status?: RequestStatus;
      serviceType?: ServiceType;
      page?: number;
      pageSize?: number;
    }
  ): Promise<PaginatedResponse<ServiceRequest>> {
    const response = await this.client.get<PaginatedResponse<ServiceRequest>>(
      '/services/requests',
      {
        params: { citizenId, ...filters },
      }
    );
    if (!response.data.success) {
      throw new SmartGovernmentError('Failed to fetch service requests', 'FETCH_ERROR');
    }
    return response.data;
  }

  /**
   * Update service request
   */
  async updateServiceRequest(
    requestId: string,
    updates: Partial<ServiceRequest>
  ): Promise<ServiceRequest> {
    const response = await this.client.patch<APIResponse<ServiceRequest>>(
      `/services/requests/${requestId}`,
      updates
    );
    if (!response.data.success || !response.data.data) {
      throw new SmartGovernmentError('Failed to update service request', 'UPDATE_ERROR');
    }
    return response.data.data;
  }

  /**
   * Cancel service request
   */
  async cancelServiceRequest(requestId: string, reason?: string): Promise<void> {
    const response = await this.client.post<APIResponse<void>>(
      `/services/requests/${requestId}/cancel`,
      { reason }
    );
    if (!response.data.success) {
      throw new SmartGovernmentError('Failed to cancel service request', 'CANCEL_ERROR');
    }
  }

  // ========================================================================
  // AI Assistant
  // ========================================================================

  /**
   * Create AI assistant instance
   */
  async createAIAssistant(config: AIAssistantConfig): Promise<AIAssistant> {
    return new AIAssistant(this.client, config);
  }

  /**
   * Create chat session
   */
  async createChatSession(): Promise<ChatSession> {
    const response = await this.client.post<APIResponse<ChatSession>>('/ai/sessions');
    if (!response.data.success || !response.data.data) {
      throw new SmartGovernmentError('Failed to create chat session', 'CREATE_ERROR');
    }
    return response.data.data;
  }

  /**
   * Send message to AI assistant
   */
  async sendMessage(sessionId: string, message: string, language?: string): Promise<AIResponse> {
    const response = await this.client.post<APIResponse<AIResponse>>(
      `/ai/sessions/${sessionId}/messages`,
      { message, language }
    );
    if (!response.data.success || !response.data.data) {
      throw new SmartGovernmentError('Failed to send message', 'SEND_ERROR');
    }
    return response.data.data;
  }

  /**
   * Get chat session history
   */
  async getChatSession(sessionId: string): Promise<ChatSession> {
    const response = await this.client.get<APIResponse<ChatSession>>(`/ai/sessions/${sessionId}`);
    if (!response.data.success || !response.data.data) {
      throw new SmartGovernmentError('Failed to fetch chat session', 'FETCH_ERROR');
    }
    return response.data.data;
  }

  // ========================================================================
  // Analytics
  // ========================================================================

  /**
   * Get dashboard metrics
   */
  async getDashboardMetrics(params?: {
    department?: string;
    period?: string;
    startDate?: string;
    endDate?: string;
  }): Promise<DashboardMetrics> {
    const response = await this.client.get<APIResponse<DashboardMetrics>>('/analytics/metrics', {
      params,
    });
    if (!response.data.success || !response.data.data) {
      throw new SmartGovernmentError('Failed to fetch metrics', 'FETCH_ERROR');
    }
    return response.data.data;
  }

  /**
   * Get predictive analytics
   */
  async getPredictions(request: PredictionRequest): Promise<PredictionResult[]> {
    const response = await this.client.post<APIResponse<PredictionResult[]>>(
      '/analytics/predict',
      request
    );
    if (!response.data.success || !response.data.data) {
      throw new SmartGovernmentError('Failed to get predictions', 'PREDICTION_ERROR');
    }
    return response.data.data;
  }

  /**
   * Track custom analytics event
   */
  async trackEvent(eventData: {
    eventType: string;
    data: Record<string, any>;
    tags?: string[];
  }): Promise<void> {
    const response = await this.client.post<APIResponse<void>>('/analytics/events', eventData);
    if (!response.data.success) {
      throw new SmartGovernmentError('Failed to track event', 'TRACK_ERROR');
    }
  }

  // ========================================================================
  // Integration Management
  // ========================================================================

  /**
   * List all integrations
   */
  async listIntegrations(): Promise<Integration[]> {
    const response = await this.client.get<APIResponse<Integration[]>>('/integrations');
    if (!response.data.success || !response.data.data) {
      throw new SmartGovernmentError('Failed to fetch integrations', 'FETCH_ERROR');
    }
    return response.data.data;
  }

  /**
   * Get integration details
   */
  async getIntegration(integrationId: string): Promise<Integration> {
    const response = await this.client.get<APIResponse<Integration>>(
      `/integrations/${integrationId}`
    );
    if (!response.data.success || !response.data.data) {
      throw new SmartGovernmentError('Failed to fetch integration', 'FETCH_ERROR');
    }
    return response.data.data;
  }

  /**
   * Test integration health
   */
  async testIntegration(integrationId: string): Promise<{ healthy: boolean; responseTime: number }> {
    const response = await this.client.post<APIResponse<any>>(
      `/integrations/${integrationId}/test`
    );
    if (!response.data.success || !response.data.data) {
      throw new SmartGovernmentError('Integration test failed', 'TEST_ERROR');
    }
    return response.data.data;
  }

  /**
   * Submit inter-agency data request
   */
  async requestInterAgencyData(request: Partial<InterAgencyRequest>): Promise<InterAgencyRequest> {
    const response = await this.client.post<APIResponse<InterAgencyRequest>>(
      '/integrations/inter-agency',
      request
    );
    if (!response.data.success || !response.data.data) {
      throw new SmartGovernmentError('Failed to submit inter-agency request', 'REQUEST_ERROR');
    }
    return response.data.data;
  }

  // ========================================================================
  // Webhook Management
  // ========================================================================

  /**
   * Create webhook
   */
  async createWebhook(webhook: {
    url: string;
    events: WebhookEvent[];
    secret?: string;
  }): Promise<Webhook> {
    const response = await this.client.post<APIResponse<Webhook>>('/webhooks', webhook);
    if (!response.data.success || !response.data.data) {
      throw new SmartGovernmentError('Failed to create webhook', 'CREATE_ERROR');
    }
    return response.data.data;
  }

  /**
   * List webhooks
   */
  async listWebhooks(): Promise<Webhook[]> {
    const response = await this.client.get<APIResponse<Webhook[]>>('/webhooks');
    if (!response.data.success || !response.data.data) {
      throw new SmartGovernmentError('Failed to fetch webhooks', 'FETCH_ERROR');
    }
    return response.data.data;
  }

  /**
   * Delete webhook
   */
  async deleteWebhook(webhookId: string): Promise<void> {
    const response = await this.client.delete<APIResponse<void>>(`/webhooks/${webhookId}`);
    if (!response.data.success) {
      throw new SmartGovernmentError('Failed to delete webhook', 'DELETE_ERROR');
    }
  }
}

/**
 * AI Assistant helper class
 */
export class AIAssistant {
  private client: AxiosInstance;
  private config: AIAssistantConfig;
  private sessionId?: string;

  constructor(client: AxiosInstance, config: AIAssistantConfig) {
    this.client = client;
    this.config = config;
  }

  /**
   * Initialize assistant session
   */
  async start(): Promise<string> {
    const response = await this.client.post<APIResponse<ChatSession>>('/ai/sessions', {
      config: this.config,
    });
    if (!response.data.success || !response.data.data) {
      throw new SmartGovernmentError('Failed to start AI assistant', 'START_ERROR');
    }
    this.sessionId = response.data.data.sessionId;
    return this.sessionId;
  }

  /**
   * Process user query
   */
  async processQuery(query: string, language?: string): Promise<string> {
    if (!this.sessionId) {
      await this.start();
    }

    const response = await this.client.post<APIResponse<AIResponse>>(
      `/ai/sessions/${this.sessionId}/messages`,
      {
        message: query,
        language: language || this.config.languages[0],
      }
    );

    if (!response.data.success || !response.data.data) {
      throw new SmartGovernmentError('Failed to process query', 'QUERY_ERROR');
    }

    return response.data.data.response;
  }

  /**
   * Get conversation history
   */
  async getHistory(): Promise<ChatSession> {
    if (!this.sessionId) {
      throw new SmartGovernmentError('No active session', 'NO_SESSION');
    }

    const response = await this.client.get<APIResponse<ChatSession>>(
      `/ai/sessions/${this.sessionId}`
    );

    if (!response.data.success || !response.data.data) {
      throw new SmartGovernmentError('Failed to fetch history', 'FETCH_ERROR');
    }

    return response.data.data;
  }

  /**
   * End assistant session
   */
  async end(): Promise<void> {
    if (this.sessionId) {
      await this.client.delete(`/ai/sessions/${this.sessionId}`);
      this.sessionId = undefined;
    }
  }
}

// Export all types
export * from './types';

// Export default instance creator
export default SmartGovernment;
