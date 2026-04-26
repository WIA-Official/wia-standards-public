/**
 * WIA-SOC-003 E-Government Standard API
 * TypeScript Implementation
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @packageDocumentation
 * @module wia-soc-003
 * @license MIT
 * @author WIA / SmileStory Inc.
 */

import * as types from './types';

// Re-export all types
export * from './types';

/**
 * Main API client for WIA-SOC-003 e-government services
 */
export class WiaEGovernment {
  private baseUrl: string;
  private apiKey: string;
  private token?: string;
  private timeout: number;

  constructor(options: types.ApiClientOptions) {
    const env = options.environment || 'production';
    this.baseUrl = options.endpoint ||
      (env === 'production'
        ? `https://api.egov.${options.countryCode.toLowerCase()}/v1`
        : `https://sandbox-api.egov.${options.countryCode.toLowerCase()}/v1`);
    this.apiKey = options.apiKey;
    this.timeout = options.timeout || 30000;
  }

  // ========================================================================
  // Authentication Methods
  // ========================================================================

  /**
   * Authenticate a citizen
   */
  async authenticateCitizen(
    request: types.AuthenticationRequest
  ): Promise<types.AuthenticationResponse> {
    const response = await this.fetch<types.AuthenticationResponse>('/auth/citizen', {
      method: 'POST',
      body: JSON.stringify(request)
    });

    if (response.success && response.data) {
      this.token = response.data.token;
      return response.data;
    }

    throw new Error(response.error?.message || 'Authentication failed');
  }

  /**
   * Refresh authentication token
   */
  async refreshToken(refreshToken: string): Promise<string> {
    const response = await this.fetch<{ token: string; expiresIn: number }>('/auth/refresh', {
      method: 'POST',
      body: JSON.stringify({ refreshToken })
    });

    if (response.success && response.data) {
      this.token = response.data.token;
      return response.data.token;
    }

    throw new Error('Token refresh failed');
  }

  /**
   * Logout current session
   */
  async logout(): Promise<boolean> {
    if (!this.token) {
      throw new Error('Not authenticated');
    }

    const response = await this.fetch<{ success: boolean }>('/auth/logout', {
      method: 'POST',
      body: JSON.stringify({ token: this.token })
    });

    if (response.success) {
      this.token = undefined;
      return true;
    }

    return false;
  }

  // ========================================================================
  // Citizen Management Methods
  // ========================================================================

  /**
   * Get citizen profile
   */
  async getCitizenProfile(citizenId: string): Promise<types.CitizenIdentity> {
    this.requireAuth();

    const response = await this.fetch<types.CitizenIdentity>(`/citizen/${citizenId}`);

    if (response.success && response.data) {
      return response.data;
    }

    throw new Error(response.error?.message || 'Failed to get citizen profile');
  }

  /**
   * Update citizen profile
   */
  async updateCitizenProfile(
    citizenId: string,
    updates: Partial<types.CitizenIdentity>
  ): Promise<types.CitizenIdentity> {
    this.requireAuth();

    const response = await this.fetch<types.CitizenIdentity>(`/citizen/${citizenId}`, {
      method: 'PUT',
      body: JSON.stringify(updates)
    });

    if (response.success && response.data) {
      return response.data;
    }

    throw new Error(response.error?.message || 'Failed to update profile');
  }

  // ========================================================================
  // Service Request Methods
  // ========================================================================

  /**
   * List available services
   */
  async listAvailableServices(options?: {
    category?: types.ServiceCategory;
    language?: types.LanguageCode;
    page?: number;
    limit?: number;
  }): Promise<types.PaginatedResponse<types.GovernmentService>> {
    const params = new URLSearchParams();
    if (options?.category) params.append('category', options.category);
    if (options?.language) params.append('language', options.language);
    if (options?.page) params.append('page', options.page.toString());
    if (options?.limit) params.append('limit', options.limit.toString());

    const url = `/services/available${params.toString() ? '?' + params.toString() : ''}`;
    const response = await this.fetch<types.PaginatedResponse<types.GovernmentService>>(url);

    if (response.success && response.data) {
      return response.data;
    }

    throw new Error(response.error?.message || 'Failed to list services');
  }

  /**
   * Submit a service request
   */
  async submitServiceRequest(
    params: types.ServiceRequestParams
  ): Promise<types.ServiceRequest> {
    this.requireAuth();

    const response = await this.fetch<{ request: types.ServiceRequest }>('/services/request', {
      method: 'POST',
      body: JSON.stringify(params)
    });

    if (response.success && response.data) {
      return response.data.request;
    }

    throw new Error(response.error?.message || 'Failed to submit request');
  }

  /**
   * Get service request status
   */
  async getRequestStatus(requestId: string): Promise<types.ServiceRequest> {
    this.requireAuth();

    const response = await this.fetch<types.ServiceRequest>(`/services/request/${requestId}`);

    if (response.success && response.data) {
      return response.data;
    }

    throw new Error(response.error?.message || 'Failed to get request status');
  }

  /**
   * Cancel a service request
   */
  async cancelServiceRequest(
    requestId: string,
    reason?: string
  ): Promise<boolean> {
    this.requireAuth();

    const response = await this.fetch<{ success: boolean }>(`/services/request/${requestId}`, {
      method: 'DELETE',
      body: JSON.stringify({ reason })
    });

    return response.success || false;
  }

  // ========================================================================
  // Document Management Methods
  // ========================================================================

  /**
   * Upload a document
   */
  async uploadDocument(
    params: types.DocumentUploadParams
  ): Promise<types.GovernmentDocument> {
    this.requireAuth();

    const formData = new FormData();
    formData.append('file', params.file);
    formData.append('type', params.type);
    if (params.subtype) formData.append('subtype', params.subtype);
    if (params.metadata) formData.append('metadata', JSON.stringify(params.metadata));
    if (params.encryption !== undefined) formData.append('encryption', String(params.encryption));

    const response = await this.fetch<{ document: types.GovernmentDocument }>('/documents/upload', {
      method: 'POST',
      body: formData,
      headers: {} // Let browser set Content-Type for multipart
    });

    if (response.success && response.data) {
      return response.data.document;
    }

    throw new Error(response.error?.message || 'Failed to upload document');
  }

  /**
   * Get document metadata
   */
  async getDocument(documentId: string): Promise<types.GovernmentDocument> {
    this.requireAuth();

    const response = await this.fetch<types.GovernmentDocument>(`/documents/${documentId}`);

    if (response.success && response.data) {
      return response.data;
    }

    throw new Error(response.error?.message || 'Failed to get document');
  }

  /**
   * Download a document
   */
  async downloadDocument(documentId: string): Promise<Blob> {
    this.requireAuth();

    const url = `${this.baseUrl}/documents/${documentId}/download`;
    const headers = this.getHeaders();

    const response = await fetch(url, { headers });

    if (!response.ok) {
      throw new Error(`Failed to download document: ${response.statusText}`);
    }

    return await response.blob();
  }

  /**
   * Verify a document
   */
  async verifyDocument(documentId: string): Promise<{
    valid: boolean;
    issuedBy: string;
    issuedDate: string;
    verificationMethod: string;
  }> {
    const response = await this.fetch<{
      documentId: string;
      valid: boolean;
      issuedBy: string;
      issuedDate: string;
      verificationMethod: string;
    }>(`/documents/${documentId}/verify`);

    if (response.success && response.data) {
      return response.data;
    }

    throw new Error(response.error?.message || 'Failed to verify document');
  }

  // ========================================================================
  // Payment Methods
  // ========================================================================

  /**
   * Initiate a payment
   */
  async initiatePayment(params: types.PaymentParams): Promise<types.Payment> {
    this.requireAuth();

    const response = await this.fetch<types.Payment>('/payment', {
      method: 'POST',
      body: JSON.stringify(params)
    });

    if (response.success && response.data) {
      return response.data;
    }

    throw new Error(response.error?.message || 'Failed to initiate payment');
  }

  /**
   * Get payment status
   */
  async getPaymentStatus(paymentId: string): Promise<types.Payment> {
    this.requireAuth();

    const response = await this.fetch<types.Payment>(`/payment/${paymentId}`);

    if (response.success && response.data) {
      return response.data;
    }

    throw new Error(response.error?.message || 'Failed to get payment status');
  }

  // ========================================================================
  // Notification Methods
  // ========================================================================

  /**
   * Subscribe to notifications
   */
  async subscribeToNotifications(
    subscription: types.NotificationSubscription
  ): Promise<{ subscriptionId: string }> {
    this.requireAuth();

    const response = await this.fetch<{ subscriptionId: string }>('/notifications/subscribe', {
      method: 'POST',
      body: JSON.stringify(subscription)
    });

    if (response.success && response.data) {
      return response.data;
    }

    throw new Error(response.error?.message || 'Failed to subscribe to notifications');
  }

  /**
   * Subscribe to real-time updates for a request
   */
  subscribeToUpdates(
    requestId: string,
    callback: (notification: types.Notification) => void
  ): () => void {
    this.requireAuth();

    const ws = new WebSocket(`${this.baseUrl.replace('http', 'ws')}/ws`);

    ws.onopen = () => {
      ws.send(JSON.stringify({
        type: 'auth',
        token: this.token
      }));

      ws.send(JSON.stringify({
        type: 'subscribe',
        requestId: requestId
      }));
    };

    ws.onmessage = (event) => {
      const notification: types.Notification = JSON.parse(event.data);
      callback(notification);
    };

    ws.onerror = (error) => {
      console.error('WebSocket error:', error);
    };

    // Return unsubscribe function
    return () => {
      ws.close();
    };
  }

  // ========================================================================
  // Analytics Methods
  // ========================================================================

  /**
   * Get public analytics data
   */
  async getPublicAnalytics(period?: {
    start: string;
    end: string;
  }): Promise<types.PublicAnalytics> {
    const params = new URLSearchParams();
    if (period?.start) params.append('start', period.start);
    if (period?.end) params.append('end', period.end);

    const url = `/analytics/public${params.toString() ? '?' + params.toString() : ''}`;
    const response = await this.fetch<types.PublicAnalytics>(url);

    if (response.success && response.data) {
      return response.data;
    }

    throw new Error(response.error?.message || 'Failed to get analytics');
  }

  // ========================================================================
  // Helper Methods
  // ========================================================================

  /**
   * Make an API request
   */
  private async fetch<T>(
    path: string,
    options: RequestInit = {}
  ): Promise<types.ApiResponse<T>> {
    const url = `${this.baseUrl}${path}`;
    const headers = this.getHeaders(options.headers);

    try {
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), this.timeout);

      const response = await fetch(url, {
        ...options,
        headers,
        signal: controller.signal
      });

      clearTimeout(timeoutId);

      const data = await response.json();

      if (!response.ok) {
        return {
          success: false,
          error: data.error || {
            code: `HTTP_${response.status}`,
            message: response.statusText
          }
        };
      }

      return {
        success: true,
        data: data
      };
    } catch (error) {
      return {
        success: false,
        error: {
          code: 'NETWORK_ERROR',
          message: error instanceof Error ? error.message : 'Unknown error'
        }
      };
    }
  }

  /**
   * Get request headers
   */
  private getHeaders(additionalHeaders?: HeadersInit): Record<string, string> {
    const headers: Record<string, string> = {
      'Content-Type': 'application/json',
      'X-API-Key': this.apiKey,
      'X-Request-ID': this.generateRequestId(),
      ...additionalHeaders as Record<string, string>
    };

    if (this.token) {
      headers['Authorization'] = `Bearer ${this.token}`;
    }

    return headers;
  }

  /**
   * Generate unique request ID
   */
  private generateRequestId(): string {
    return `${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
  }

  /**
   * Require authentication
   */
  private requireAuth(): void {
    if (!this.token) {
      throw new Error('Authentication required. Please call authenticateCitizen() first.');
    }
  }
}

/**
 * Create a new WIA E-Government API client
 */
export function createClient(options: types.ApiClientOptions): WiaEGovernment {
  return new WiaEGovernment(options);
}

/**
 * Default export
 */
export default WiaEGovernment;
