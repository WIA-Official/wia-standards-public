/**
 * WIA-SOC-006 Disaster Management System Standard API
 * TypeScript Implementation
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @packageDocumentation
 * @module wia-soc-006
 * @license MIT
 * @author WIA / SmileStory Inc.
 */

import * as types from './types';

export * from './types';

export class WiaDisasterManagement {
  private baseUrl: string;
  private token?: string;
  private apiKey?: string;
  private timeout: number;

  constructor(options: types.ApiClientOptions) {
    const protocol = options.protocol || 'https';
    const port = options.port || 443;
    this.baseUrl = `${protocol}://${options.host}:${port}/api/v1`;
    this.token = options.token;
    this.apiKey = options.apiKey;
    this.timeout = options.timeout || 5000;
  }

  // =========================================================================
  // Event Management
  // =========================================================================

  async createEvent(event: Omit<types.DisasterEvent, 'eventId'>): Promise<types.DisasterEvent> {
    return await this.fetch('/events', {
      method: 'POST',
      body: JSON.stringify(event)
    });
  }

  async getEvent(eventId: string): Promise<types.DisasterEvent> {
    return await this.fetch(`/events/${eventId}`);
  }

  async updateEvent(eventId: string, updates: Partial<types.DisasterEvent>): Promise<types.CommandResponse> {
    return await this.fetch(`/events/${eventId}`, {
      method: 'PATCH',
      body: JSON.stringify(updates)
    });
  }

  async listEvents(filters?: {
    status?: types.EventStatus;
    type?: types.DisasterType;
    severity_min?: number;
    severity_max?: number;
    bbox?: string;
    limit?: number;
    offset?: number;
  }): Promise<types.PaginatedResponse<types.DisasterEvent>> {
    const params = new URLSearchParams(filters as any);
    return await this.fetch(`/events?${params}`);
  }

  // =========================================================================
  // Alert Management
  // =========================================================================

  async issueAlert(alert: Omit<types.AlertMessage, 'alertId' | 'timestamp'>): Promise<types.AlertMessage> {
    return await this.fetch('/alerts', {
      method: 'POST',
      body: JSON.stringify(alert)
    });
  }

  async getAlert(alertId: string): Promise<types.AlertMessage> {
    return await this.fetch(`/alerts/${alertId}`);
  }

  async cancelAlert(alertId: string): Promise<types.CommandResponse> {
    return await this.fetch(`/alerts/${alertId}`, {
      method: 'DELETE'
    });
  }

  // =========================================================================
  // Resource Management
  // =========================================================================

  async registerResource(resource: Omit<types.EmergencyResource, 'resourceId'>): Promise<types.EmergencyResource> {
    return await this.fetch('/resources', {
      method: 'POST',
      body: JSON.stringify(resource)
    });
  }

  async getResource(resourceId: string): Promise<types.EmergencyResource> {
    return await this.fetch(`/resources/${resourceId}`);
  }

  async listResources(filters?: {
    type?: types.ResourceType;
    status?: types.ResourceStatus;
    agency?: string;
  }): Promise<types.PaginatedResponse<types.EmergencyResource>> {
    const params = new URLSearchParams(filters as any);
    return await this.fetch(`/resources?${params}`);
  }

  async deployResource(deployment: Omit<types.ResourceDeployment, 'deploymentId' | 'deployedAt'>): Promise<types.ResourceDeployment> {
    return await this.fetch('/deployments', {
      method: 'POST',
      body: JSON.stringify(deployment)
    });
  }

  async updateDeployment(deploymentId: string, updates: Partial<types.ResourceDeployment>): Promise<types.CommandResponse> {
    return await this.fetch(`/deployments/${deploymentId}`, {
      method: 'PATCH',
      body: JSON.stringify(updates)
    });
  }

  async recallResource(deploymentId: string): Promise<types.CommandResponse> {
    return await this.fetch(`/deployments/${deploymentId}/recall`, {
      method: 'POST'
    });
  }

  // =========================================================================
  // Evacuation Management
  // =========================================================================

  async issueEvacuation(order: Omit<types.EvacuationOrder, 'orderId'>): Promise<types.EvacuationOrder> {
    return await this.fetch('/evacuations', {
      method: 'POST',
      body: JSON.stringify(order)
    });
  }

  async getEvacuation(orderId: string): Promise<types.EvacuationOrder> {
    return await this.fetch(`/evacuations/${orderId}`);
  }

  async getEvacuationStatus(orderId: string): Promise<any> {
    return await this.fetch(`/evacuations/${orderId}/status`);
  }

  // =========================================================================
  // Damage Assessment
  // =========================================================================

  async submitAssessment(assessment: Omit<types.DamageAssessment, 'assessmentId'>): Promise<types.DamageAssessment> {
    return await this.fetch('/assessments', {
      method: 'POST',
      body: JSON.stringify(assessment)
    });
  }

  async getAssessmentSummary(eventId: string): Promise<any> {
    return await this.fetch(`/events/${eventId}/assessments/summary`);
  }

  // =========================================================================
  // Communication
  // =========================================================================

  async sendMessage(message: Omit<types.InterAgencyMessage, 'messageId' | 'timestamp'>): Promise<types.InterAgencyMessage> {
    return await this.fetch('/messages', {
      method: 'POST',
      body: JSON.stringify(message)
    });
  }

  async getMessages(filters?: {
    unread?: boolean;
    priority?: types.MessagePriority;
  }): Promise<types.PaginatedResponse<types.InterAgencyMessage>> {
    const params = new URLSearchParams(filters as any);
    return await this.fetch(`/messages?${params}`);
  }

  // =========================================================================
  // Incident Logs
  // =========================================================================

  async logIncident(log: Omit<types.IncidentLog, 'logId'>): Promise<types.IncidentLog> {
    return await this.fetch('/logs', {
      method: 'POST',
      body: JSON.stringify(log)
    });
  }

  async getLogs(eventId: string, filters?: {
    severity?: types.LogSeverity;
    category?: types.LogCategory;
  }): Promise<types.PaginatedResponse<types.IncidentLog>> {
    const params = new URLSearchParams({ eventId, ...(filters as any) });
    return await this.fetch(`/logs?${params}`);
  }

  // =========================================================================
  // Geospatial Search
  // =========================================================================

  async geoSearch(point: types.Coordinates, radius: number, types?: string[]): Promise<any> {
    return await this.fetch('/search/geo', {
      method: 'POST',
      body: JSON.stringify({ point, radius, types })
    });
  }

  // =========================================================================
  // Statistics
  // =========================================================================

  async getStats(eventId?: string, period?: string): Promise<any> {
    const params = new URLSearchParams({ eventId, period } as any);
    return await this.fetch(`/stats?${params}`);
  }

  // =========================================================================
  // Real-Time WebSocket
  // =========================================================================

  subscribeToEvents(callback: types.EventCallback, options?: types.WebSocketOptions): () => void {
    const wsUrl = this.baseUrl.replace(/^http/, 'ws').replace('/api/v1', '/ws');
    const ws = new WebSocket(wsUrl);

    ws.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        callback(data);
      } catch (error) {
        console.error('Failed to parse WebSocket message:', error);
      }
    };

    ws.onopen = () => {
      ws.send(JSON.stringify({
        action: 'subscribe',
        channels: ['events', 'alerts', 'deployments']
      }));
    };

    if (options?.reconnect) {
      let reconnectAttempts = 0;
      ws.onclose = () => {
        if (reconnectAttempts < (options.maxReconnectAttempts || Infinity)) {
          setTimeout(() => {
            reconnectAttempts++;
            this.subscribeToEvents(callback, options);
          }, options.reconnectInterval || 5000);
        }
      };
    }

    return () => ws.close();
  }

  // =========================================================================
  // Internal Helpers
  // =========================================================================

  private async fetch(endpoint: string, options: RequestInit = {}): Promise<any> {
    const headers: HeadersInit = {
      'Content-Type': 'application/json',
      ...options.headers
    };

    if (this.token) {
      headers['Authorization'] = `Bearer ${this.token}`;
    }
    if (this.apiKey) {
      headers['X-API-Key'] = this.apiKey;
    }

    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), this.timeout);

    try {
      const response = await fetch(`${this.baseUrl}${endpoint}`, {
        ...options,
        headers,
        signal: controller.signal
      });

      clearTimeout(timeoutId);

      if (!response.ok) {
        const error = await response.json();
        throw new Error(error.error?.message || `HTTP ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      clearTimeout(timeoutId);
      throw error;
    }
  }
}

export function createClient(options: types.ApiClientOptions): WiaDisasterManagement {
  return new WiaDisasterManagement(options);
}

export const VERSION = '1.0.0';
export default WiaDisasterManagement;
