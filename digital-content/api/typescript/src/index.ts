/**
 * WIA-EDU-019 Digital Content Standard - TypeScript SDK
 * Version: 2.0.0
 *
 * Philosophy: 弘益人間 (Benefit All Humanity)
 *
 * @packageDocumentation
 */

import axios, { AxiosInstance, AxiosError } from 'axios';
import type {
  ClientConfig,
  Content,
  ListContentRequest,
  ListContentResponse,
  CreateContentRequest,
  UpdateContentRequest,
  SearchRequest,
  SearchResponse,
  TrackingEvent,
  AnalyticsData,
  SyncRequest,
  SyncResponse,
  WebhookConfig,
  APIError,
  Certification,
  CertificationRequest,
} from './types';

// Re-export all types
export * from './types';

/**
 * Main client for interacting with WIA-EDU-019 Digital Content API
 */
export class DigitalContentClient {
  private client: AxiosInstance;
  private config: ClientConfig;

  /**
   * Creates a new Digital Content API client
   *
   * @param config - Client configuration
   *
   * @example
   * ```typescript
   * const client = new DigitalContentClient({
   *   baseURL: 'https://api.example.com/wia/v1',
   *   accessToken: 'your-access-token',
   *   timeout: 30000
   * });
   * ```
   */
  constructor(config: ClientConfig) {
    this.config = {
      timeout: 30000,
      retries: 3,
      enableLogging: false,
      ...config,
    };

    this.client = axios.create({
      baseURL: this.config.baseURL,
      timeout: this.config.timeout,
      headers: {
        'Content-Type': 'application/json',
        'X-WIA-Standard': 'WIA-EDU-019',
        'X-WIA-Version': '2.0.0',
        ...(this.config.accessToken && {
          'Authorization': `Bearer ${this.config.accessToken}`,
        }),
        ...(this.config.apiKey && {
          'X-API-Key': this.config.apiKey,
        }),
      },
    });

    this.setupInterceptors();
  }

  /**
   * Sets up request/response interceptors
   * @private
   */
  private setupInterceptors(): void {
    // Request interceptor
    this.client.interceptors.request.use(
      (config) => {
        if (this.config.enableLogging) {
          console.log(`[WIA] ${config.method?.toUpperCase()} ${config.url}`);
        }
        return config;
      },
      (error) => Promise.reject(error)
    );

    // Response interceptor
    this.client.interceptors.response.use(
      (response) => response,
      async (error: AxiosError) => {
        if (this.config.enableLogging) {
          console.error('[WIA] Error:', error.message);
        }
        return Promise.reject(this.handleError(error));
      }
    );
  }

  /**
   * Handles API errors and transforms them to APIError format
   * @private
   */
  private handleError(error: AxiosError): APIError {
    if (error.response?.data) {
      return error.response.data as APIError;
    }

    return {
      code: error.code || 'UNKNOWN_ERROR',
      message: error.message || 'An unknown error occurred',
    };
  }

  // ============================================================================
  // Content Management
  // ============================================================================

  /**
   * Lists content with optional filtering and pagination
   *
   * @param params - List parameters
   * @returns Promise resolving to list of content
   *
   * @example
   * ```typescript
   * const response = await client.listContent({
   *   limit: 20,
   *   type: 'video',
   *   language: 'en'
   * });
   *
   * console.log(`Found ${response.total} items`);
   * response.items.forEach(item => {
   *   console.log(item.title);
   * });
   * ```
   */
  async listContent(params?: ListContentRequest): Promise<ListContentResponse> {
    const response = await this.client.get<ListContentResponse>('/content', { params });
    return response.data;
  }

  /**
   * Gets detailed information about specific content
   *
   * @param id - Content ID
   * @returns Promise resolving to content details
   *
   * @example
   * ```typescript
   * const content = await client.getContent('content-123');
   * console.log(content.title);
   * console.log(content.accessibility.captions);
   * ```
   */
  async getContent(id: string): Promise<Content> {
    const response = await this.client.get<Content>(`/content/${id}`);
    return response.data;
  }

  /**
   * Creates new content
   *
   * @param data - Content creation data
   * @returns Promise resolving to created content
   *
   * @example
   * ```typescript
   * const content = await client.createContent({
   *   title: 'My Educational Video',
   *   description: 'A comprehensive guide...',
   *   type: 'video',
   *   language: 'en',
   *   fileUrl: 'https://example.com/video.mp4',
   *   metadata: {
   *     creator: 'Jane Smith',
   *     subject: ['Education', 'Technology'],
   *     wcagLevel: 'AA'
   *   },
   *   accessibility: {
   *     captions: ['en', 'ko'],
   *     audioDescription: true,
   *     transcript: true
   *   },
   *   license: {
   *     type: 'CC BY',
   *     allowsCommercial: true,
   *     allowsDerivatives: true
   *   }
   * });
   * ```
   */
  async createContent(data: CreateContentRequest): Promise<Content> {
    const response = await this.client.post<Content>('/content', data);
    return response.data;
  }

  /**
   * Updates existing content
   *
   * @param id - Content ID
   * @param data - Update data
   * @returns Promise resolving to updated content
   *
   * @example
   * ```typescript
   * const updated = await client.updateContent('content-123', {
   *   title: 'Updated Title',
   *   description: 'Updated description...'
   * });
   * ```
   */
  async updateContent(id: string, data: UpdateContentRequest): Promise<Content> {
    const response = await this.client.put<Content>(`/content/${id}`, data);
    return response.data;
  }

  /**
   * Deletes content
   *
   * @param id - Content ID
   * @returns Promise resolving when deletion is complete
   *
   * @example
   * ```typescript
   * await client.deleteContent('content-123');
   * console.log('Content deleted');
   * ```
   */
  async deleteContent(id: string): Promise<void> {
    await this.client.delete(`/content/${id}`);
  }

  // ============================================================================
  // Search
  // ============================================================================

  /**
   * Searches for content
   *
   * @param request - Search request
   * @returns Promise resolving to search results
   *
   * @example
   * ```typescript
   * const results = await client.search({
   *   query: 'digital content standards',
   *   filters: {
   *     type: ['video', 'interactive'],
   *     language: ['en', 'ko'],
   *     wcagLevel: ['AA']
   *   },
   *   sort: {
   *     field: 'relevance',
   *     order: 'desc'
   *   },
   *   limit: 20
   * });
   *
   * results.results.forEach(result => {
   *   console.log(`${result.title} (score: ${result.score})`);
   * });
   * ```
   */
  async search(request: SearchRequest): Promise<SearchResponse> {
    const response = await this.client.post<SearchResponse>('/search', request);
    return response.data;
  }

  // ============================================================================
  // Analytics
  // ============================================================================

  /**
   * Tracks content usage event
   *
   * @param event - Tracking event data
   * @returns Promise resolving when event is tracked
   *
   * @example
   * ```typescript
   * await client.trackEvent({
   *   contentId: 'content-123',
   *   userId: 'user-456',
   *   event: 'view',
   *   timestamp: new Date().toISOString(),
   *   metadata: {
   *     duration: 450,
   *     completion: 0.5,
   *     device: 'mobile'
   *   }
   * });
   * ```
   */
  async trackEvent(event: TrackingEvent): Promise<void> {
    await this.client.post('/analytics/track', event);
  }

  /**
   * Gets analytics data for content
   *
   * @param contentId - Content ID
   * @param period - Time period (e.g., '2025-01', '2025-Q1')
   * @returns Promise resolving to analytics data
   *
   * @example
   * ```typescript
   * const analytics = await client.getAnalytics('content-123', '2025-01');
   * console.log(`Views: ${analytics.views}`);
   * console.log(`Unique users: ${analytics.uniqueUsers}`);
   * console.log(`Completion rate: ${analytics.completionRate * 100}%`);
   * ```
   */
  async getAnalytics(contentId: string, period?: string): Promise<AnalyticsData> {
    const response = await this.client.get<AnalyticsData>(`/analytics/content/${contentId}`, {
      params: { period },
    });
    return response.data;
  }

  // ============================================================================
  // Synchronization
  // ============================================================================

  /**
   * Synchronizes content across devices
   *
   * @param request - Sync request
   * @returns Promise resolving to sync response with updates
   *
   * @example
   * ```typescript
   * const syncResult = await client.sync({
   *   deviceId: 'device-123',
   *   lastSyncTimestamp: '2025-01-20T10:00:00Z',
   *   localContent: [
   *     {
   *       id: 'content-123',
   *       version: '1.0.0',
   *       checksum: 'abc123...'
   *     }
   *   ]
   * });
   *
   * syncResult.updates.forEach(update => {
   *   if (update.action === 'update') {
   *     console.log(`Update ${update.id} to ${update.version}`);
   *   }
   * });
   * ```
   */
  async sync(request: SyncRequest): Promise<SyncResponse> {
    const response = await this.client.post<SyncResponse>('/sync', request);
    return response.data;
  }

  // ============================================================================
  // Certification
  // ============================================================================

  /**
   * Requests content certification
   *
   * @param request - Certification request
   * @returns Promise resolving to certification details
   *
   * @example
   * ```typescript
   * const certification = await client.requestCertification({
   *   contentId: 'content-123',
   *   targetLevel: 'Full Compliance',
   *   applicant: {
   *     organization: 'Example Publishing',
   *     contact: 'cert@example.com'
   *   },
   *   documentation: {
   *     technicalSpecs: 'https://example.com/specs.pdf',
   *     accessibilityReport: 'https://example.com/a11y.pdf'
   *   }
   * });
   * ```
   */
  async requestCertification(request: CertificationRequest): Promise<Certification> {
    const response = await this.client.post<Certification>('/certification/request', request);
    return response.data;
  }

  /**
   * Verifies content certification
   *
   * @param certificationId - Certification ID
   * @returns Promise resolving to certification details
   *
   * @example
   * ```typescript
   * const cert = await client.verifyCertification('WIA-CERT-2025-001234');
   * console.log(`Level: ${cert.level}`);
   * console.log(`Valid until: ${cert.expirationDate}`);
   * ```
   */
  async verifyCertification(certificationId: string): Promise<Certification> {
    const response = await this.client.get<Certification>(`/certification/verify/${certificationId}`);
    return response.data;
  }

  // ============================================================================
  // Webhooks
  // ============================================================================

  /**
   * Registers a webhook
   *
   * @param config - Webhook configuration
   * @returns Promise resolving to webhook ID
   *
   * @example
   * ```typescript
   * const webhookId = await client.registerWebhook({
   *   url: 'https://your-app.com/webhook',
   *   events: ['content.created', 'content.updated'],
   *   secret: 'your-webhook-secret'
   * });
   * ```
   */
  async registerWebhook(config: WebhookConfig): Promise<string> {
    const response = await this.client.post<{ id: string }>('/webhooks', config);
    return response.data.id;
  }

  /**
   * Deletes a webhook
   *
   * @param webhookId - Webhook ID
   * @returns Promise resolving when webhook is deleted
   *
   * @example
   * ```typescript
   * await client.deleteWebhook('webhook-123');
   * ```
   */
  async deleteWebhook(webhookId: string): Promise<void> {
    await this.client.delete(`/webhooks/${webhookId}`);
  }
}

/**
 * Default export
 */
export default DigitalContentClient;
