/**
 * WIA-UNI-015: Peace Monitoring Standard
 * TypeScript SDK Implementation
 * Version: 1.0.0
 */

import axios, { AxiosInstance, AxiosError } from 'axios';
import WebSocket from 'ws';
import {
  PeaceMonitoringConfig,
  MonitoringEvent,
  MonitoringEventsQuery,
  MonitoringEventsResponse,
  ArmsInventory,
  ArmsInventoryQuery,
  ArmsInventoryResponse,
  DMZSensorData,
  SensorDataQuery,
  SensorDataResponse,
  VerificationRequest,
  ThreatAssessment,
  ConfidenceBuildingMeasure,
  APIResponse,
  WebSocketMessage,
  WebSocketSubscription,
  WebSocketAuth
} from './types';

export * from './types';

/**
 * Main Peace Monitoring Client
 */
export class PeaceMonitoringClient {
  private client: AxiosInstance;
  private config: Required<PeaceMonitoringConfig>;
  private ws?: WebSocket;

  constructor(config: PeaceMonitoringConfig) {
    this.config = {
      baseURL: config.baseURL || this.getBaseURL(config.environment || 'production'),
      apiKey: config.apiKey,
      environment: config.environment || 'production',
      timeout: config.timeout || 30000,
      region: config.region || 'global'
    };

    this.client = axios.create({
      baseURL: this.config.baseURL,
      timeout: this.config.timeout,
      headers: {
        'Authorization': `Bearer ${this.config.apiKey}`,
        'Content-Type': 'application/json',
        'X-WIA-Standard': 'WIA-UNI-015',
        'X-WIA-Version': '1.0.0'
      }
    });

    this.setupInterceptors();
  }

  private getBaseURL(environment: string): string {
    const urls = {
      production: 'https://api.wia.org/peace-monitoring/v1',
      staging: 'https://staging-api.wia.org/peace-monitoring/v1',
      development: 'https://dev-api.wia.org/peace-monitoring/v1'
    };
    return urls[environment as keyof typeof urls] || urls.production;
  }

  private setupInterceptors(): void {
    this.client.interceptors.response.use(
      response => response,
      error => this.handleError(error)
    );
  }

  private handleError(error: AxiosError): never {
    if (error.response) {
      const apiError = error.response.data as APIResponse;
      throw new Error(apiError.error?.message || 'API request failed');
    } else if (error.request) {
      throw new Error('No response received from server');
    } else {
      throw new Error(error.message);
    }
  }

  // ============================================================================
  // Monitoring Events API
  // ============================================================================

  /**
   * Get monitoring events with filtering and pagination
   */
  async getMonitoringEvents(query?: MonitoringEventsQuery): Promise<MonitoringEventsResponse> {
    const response = await this.client.get<MonitoringEventsResponse>('/monitoring/events', {
      params: query
    });
    return response.data;
  }

  /**
   * Get a specific monitoring event by ID
   */
  async getMonitoringEvent(monitoringId: string): Promise<MonitoringEvent> {
    const response = await this.client.get<MonitoringEvent>(`/monitoring/events/${monitoringId}`);
    return response.data;
  }

  /**
   * Submit a new monitoring event
   */
  async createMonitoringEvent(event: Partial<MonitoringEvent>): Promise<{ monitoringId: string; status: string; timestamp: string }> {
    const response = await this.client.post('/monitoring/events', event);
    return response.data;
  }

  /**
   * Update an existing monitoring event
   */
  async updateMonitoringEvent(monitoringId: string, updates: Partial<MonitoringEvent>): Promise<MonitoringEvent> {
    const response = await this.client.patch(`/monitoring/events/${monitoringId}`, updates);
    return response.data;
  }

  // ============================================================================
  // Arms Inventory API
  // ============================================================================

  /**
   * Get arms inventory declarations
   */
  async getArmsInventory(query?: ArmsInventoryQuery): Promise<ArmsInventoryResponse> {
    const response = await this.client.get<ArmsInventoryResponse>('/arms/inventory', {
      params: query
    });
    return response.data;
  }

  /**
   * Submit arms inventory declaration
   */
  async submitArmsInventory(inventory: Partial<ArmsInventory>): Promise<{ inventoryId: string }> {
    const response = await this.client.post('/arms/inventory', inventory);
    return response.data;
  }

  /**
   * Get specific arms inventory by ID
   */
  async getArmsInventoryById(inventoryId: string): Promise<ArmsInventory> {
    const response = await this.client.get<ArmsInventory>(`/arms/inventory/${inventoryId}`);
    return response.data;
  }

  // ============================================================================
  // DMZ Sensors API
  // ============================================================================

  /**
   * Get sensor data with filtering
   */
  async getSensorData(query?: SensorDataQuery): Promise<SensorDataResponse> {
    const response = await this.client.get<SensorDataResponse>('/sensors/readings', {
      params: query
    });
    return response.data;
  }

  /**
   * Submit sensor data (for authorized sensor systems)
   */
  async submitSensorData(sensorData: Partial<DMZSensorData>): Promise<{ sensorDataId: string }> {
    const response = await this.client.post('/sensors/readings', sensorData);
    return response.data;
  }

  /**
   * Get specific sensor by ID
   */
  async getSensor(sensorId: string): Promise<DMZSensorData> {
    const response = await this.client.get<DMZSensorData>(`/sensors/${sensorId}`);
    return response.data;
  }

  // ============================================================================
  // Verification API
  // ============================================================================

  /**
   * Create a verification request
   */
  async createVerificationRequest(request: Partial<VerificationRequest>): Promise<{
    requestId: string;
    status: string;
    expectedResponseBy: string;
    trackingUrl: string;
  }> {
    const response = await this.client.post('/verification/requests', request);
    return response.data;
  }

  /**
   * Get verification request status
   */
  async getVerificationRequest(requestId: string): Promise<VerificationRequest> {
    const response = await this.client.get<VerificationRequest>(`/verification/requests/${requestId}`);
    return response.data;
  }

  /**
   * List verification requests
   */
  async listVerificationRequests(params?: {
    status?: string;
    requestedBy?: string;
    page?: number;
    limit?: number;
  }): Promise<{ data: VerificationRequest[]; pagination: any }> {
    const response = await this.client.get('/verification/requests', { params });
    return response.data;
  }

  // ============================================================================
  // Confidence Building Measures API
  // ============================================================================

  /**
   * Get CBM activities
   */
  async getCBMActivities(params?: {
    type?: string;
    status?: string;
    page?: number;
    limit?: number;
  }): Promise<{ data: ConfidenceBuildingMeasure[]; pagination: any }> {
    const response = await this.client.get('/cbm/activities', { params });
    return response.data;
  }

  /**
   * Create CBM activity
   */
  async createCBMActivity(cbm: Partial<ConfidenceBuildingMeasure>): Promise<{ cbmId: string }> {
    const response = await this.client.post('/cbm/activities', cbm);
    return response.data;
  }

  // ============================================================================
  // Analytics & Assessment API
  // ============================================================================

  /**
   * Calculate threat assessment
   */
  async calculateThreatAssessment(params: {
    troopCount: number;
    heavyWeapons: number;
    distanceDMZ: number;
    incidents: number;
  }): Promise<ThreatAssessment> {
    const response = await this.client.post<ThreatAssessment>('/analytics/threat-assessment', params);
    return response.data;
  }

  /**
   * Get compliance summary
   */
  async getComplianceSummary(params?: {
    party?: string;
    startDate?: string;
    endDate?: string;
  }): Promise<any> {
    const response = await this.client.get('/analytics/compliance-summary', { params });
    return response.data;
  }

  // ============================================================================
  // WebSocket Real-time Streaming
  // ============================================================================

  /**
   * Connect to real-time event stream
   */
  connectStream(options: {
    channels: string[];
    onMessage: (message: WebSocketMessage) => void;
    onError?: (error: Error) => void;
    onClose?: () => void;
  }): void {
    const wsUrl = this.config.baseURL.replace('https://', 'wss://').replace('http://', 'ws://') + '/stream';
    this.ws = new WebSocket(wsUrl);

    this.ws.on('open', () => {
      // Authenticate
      const authMessage: WebSocketAuth = {
        type: 'authenticate',
        token: this.config.apiKey
      };
      this.ws?.send(JSON.stringify(authMessage));

      // Subscribe to channels
      const subscribeMessage: WebSocketSubscription = {
        type: 'subscribe',
        channels: options.channels
      };
      this.ws?.send(JSON.stringify(subscribeMessage));
    });

    this.ws.on('message', (data: WebSocket.Data) => {
      try {
        const message = JSON.parse(data.toString()) as WebSocketMessage;
        options.onMessage(message);
      } catch (error) {
        options.onError?.(error as Error);
      }
    });

    this.ws.on('error', (error) => {
      options.onError?.(error);
    });

    this.ws.on('close', () => {
      options.onClose?.();
    });
  }

  /**
   * Disconnect from real-time stream
   */
  disconnectStream(): void {
    if (this.ws) {
      this.ws.close();
      this.ws = undefined;
    }
  }

  // ============================================================================
  // Utility Methods
  // ============================================================================

  /**
   * Health check
   */
  async healthCheck(): Promise<{ status: string; timestamp: string }> {
    const response = await this.client.get('/health');
    return response.data;
  }

  /**
   * Get API info
   */
  async getAPIInfo(): Promise<{
    version: string;
    standard: string;
    environment: string;
    rateLimit: any;
  }> {
    const response = await this.client.get('/info');
    return response.data;
  }
}

/**
 * Helper function to create a peace monitoring client
 */
export function createPeaceMonitoringClient(config: PeaceMonitoringConfig): PeaceMonitoringClient {
  return new PeaceMonitoringClient(config);
}

/**
 * Default export
 */
export default PeaceMonitoringClient;
