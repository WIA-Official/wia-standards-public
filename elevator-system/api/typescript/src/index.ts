/**
 * WIA Elevator System Standard - TypeScript SDK
 * Main SDK Implementation
 * 
 * @package @wia/elevator-sdk
 * @version 1.0.0
 * @license MIT
 * 弘益人間 · Benefit All Humanity
 */

import type {
  ElevatorStatus,
  SensorData,
  ElevatorEvent,
  ElevatorAlarm,
  MaintenanceRecord,
  TrafficMetrics,
  EnergyMetrics,
  DispatchRequest,
  DispatchResponse,
  CommandRequest,
  CommandResponse,
  Building,
  APIError,
  WebSocketMessage,
  SubscriptionRequest,
  WIAElevatorClientConfig,
} from './types';

export * from './types';

/**
 * WIA Elevator System SDK Client
 */
export class WIAElevatorClient {
  private apiKey: string;
  private baseUrl: string;
  private timeout: number;
  private retryAttempts: number;
  private websocket: WebSocket | null = null;

  constructor(config: WIAElevatorClientConfig) {
    this.apiKey = config.apiKey || '';
    this.baseUrl = config.baseUrl || 'https://api.wiastandards.com';
    this.timeout = config.timeout || 30000;
    this.retryAttempts = config.retryAttempts || 3;

    if (!this.apiKey) {
      throw new Error('API key is required');
    }
  }

  // ============================================================================
  // Elevator Status & Telemetry
  // ============================================================================

  /**
   * Get current elevator status
   */
  async getStatus(elevatorId: string): Promise<ElevatorStatus> {
    return this.request<ElevatorStatus>(`/api/v1/elevator/${elevatorId}/status`);
  }

  /**
   * Get elevator sensor telemetry
   */
  async getTelemetry(
    elevatorId: string,
    start?: string,
    end?: string,
    sensors?: string[]
  ): Promise<SensorData[]> {
    const params = new URLSearchParams();
    if (start) params.append('start', start);
    if (end) params.append('end', end);
    if (sensors) params.append('sensors', sensors.join(','));

    return this.request<SensorData[]>(
      `/api/v1/elevator/${elevatorId}/telemetry?${params.toString()}`
    );
  }

  /**
   * Get elevator events
   */
  async getEvents(
    elevatorId: string,
    start?: string,
    end?: string
  ): Promise<ElevatorEvent[]> {
    const params = new URLSearchParams();
    if (start) params.append('start', start);
    if (end) params.append('end', end);

    return this.request<ElevatorEvent[]>(
      `/api/v1/elevator/${elevatorId}/events?${params.toString()}`
    );
  }

  /**
   * Get elevator alarms
   */
  async getAlarms(elevatorId: string): Promise<ElevatorAlarm[]> {
    return this.request<ElevatorAlarm[]>(`/api/v1/elevator/${elevatorId}/alarms`);
  }

  /**
   * Acknowledge alarm
   */
  async acknowledgeAlarm(alarmId: string, acknowledgedBy: string): Promise<void> {
    await this.request<void>(`/api/v1/alarm/${alarmId}/acknowledge`, {
      method: 'POST',
      body: JSON.stringify({ acknowledgedBy }),
    });
  }

  // ============================================================================
  // Dispatch & Control
  // ============================================================================

  /**
   * Request elevator dispatch
   */
  async dispatch(request: DispatchRequest): Promise<DispatchResponse> {
    return this.request<DispatchResponse>('/api/v1/elevator/dispatch', {
      method: 'POST',
      body: JSON.stringify(request),
    });
  }

  /**
   * Send control command
   */
  async sendCommand(
    elevatorId: string,
    command: CommandRequest
  ): Promise<CommandResponse> {
    return this.request<CommandResponse>(
      `/api/v1/elevator/${elevatorId}/command`,
      {
        method: 'PUT',
        body: JSON.stringify(command),
      }
    );
  }

  // ============================================================================
  // Building & Fleet Management
  // ============================================================================

  /**
   * Get building elevators
   */
  async getBuildingElevators(buildingId: string): Promise<Building> {
    return this.request<Building>(`/api/v1/building/${buildingId}/elevators`);
  }

  /**
   * Get traffic metrics
   */
  async getTrafficMetrics(
    elevatorId: string,
    start: string,
    end: string
  ): Promise<TrafficMetrics> {
    const params = new URLSearchParams({ start, end });
    return this.request<TrafficMetrics>(
      `/api/v1/elevator/${elevatorId}/traffic?${params.toString()}`
    );
  }

  /**
   * Get energy metrics
   */
  async getEnergyMetrics(
    elevatorId: string,
    start: string,
    end: string
  ): Promise<EnergyMetrics> {
    const params = new URLSearchParams({ start, end });
    return this.request<EnergyMetrics>(
      `/api/v1/elevator/${elevatorId}/energy?${params.toString()}`
    );
  }

  // ============================================================================
  // Maintenance
  // ============================================================================

  /**
   * Get maintenance records
   */
  async getMaintenanceRecords(elevatorId: string): Promise<MaintenanceRecord[]> {
    return this.request<MaintenanceRecord[]>(
      `/api/v1/elevator/${elevatorId}/maintenance`
    );
  }

  /**
   * Create maintenance record
   */
  async createMaintenanceRecord(
    elevatorId: string,
    record: Omit<MaintenanceRecord, 'maintenanceId'>
  ): Promise<MaintenanceRecord> {
    return this.request<MaintenanceRecord>(
      `/api/v1/elevator/${elevatorId}/maintenance`,
      {
        method: 'POST',
        body: JSON.stringify(record),
      }
    );
  }

  // ============================================================================
  // WebSocket Real-Time
  // ============================================================================

  /**
   * Connect to WebSocket for real-time updates
   */
  connectWebSocket(elevatorId: string, onMessage: (msg: WebSocketMessage) => void): void {
    const wsUrl = `${this.baseUrl.replace('https://', 'wss://').replace('http://', 'ws://')}/ws/elevator/${elevatorId}`;
    
    this.websocket = new WebSocket(wsUrl);
    
    this.websocket.onopen = () => {
      console.log(`WebSocket connected to ${elevatorId}`);
    };

    this.websocket.onmessage = (event) => {
      try {
        const message: WebSocketMessage = JSON.parse(event.data);
        onMessage(message);
      } catch (error) {
        console.error('Failed to parse WebSocket message:', error);
      }
    };

    this.websocket.onerror = (error) => {
      console.error('WebSocket error:', error);
    };

    this.websocket.onclose = () => {
      console.log('WebSocket disconnected');
    };
  }

  /**
   * Subscribe to specific data streams
   */
  subscribe(subscription: SubscriptionRequest): void {
    if (!this.websocket || this.websocket.readyState !== WebSocket.OPEN) {
      throw new Error('WebSocket not connected');
    }

    this.websocket.send(JSON.stringify(subscription));
  }

  /**
   * Disconnect WebSocket
   */
  disconnectWebSocket(): void {
    if (this.websocket) {
      this.websocket.close();
      this.websocket = null;
    }
  }

  // ============================================================================
  // Private Helper Methods
  // ============================================================================

  private async request<T>(
    endpoint: string,
    options: RequestInit = {}
  ): Promise<T> {
    const url = `${this.baseUrl}${endpoint}`;
    const headers = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.apiKey}`,
      ...options.headers,
    };

    let lastError: Error | null = null;

    for (let attempt = 0; attempt < this.retryAttempts; attempt++) {
      try {
        const controller = new AbortController();
        const timeoutId = setTimeout(() => controller.abort(), this.timeout);

        const response = await fetch(url, {
          ...options,
          headers,
          signal: controller.signal,
        });

        clearTimeout(timeoutId);

        if (!response.ok) {
          const error: APIError = await response.json();
          throw new Error(`API Error: ${error.error.code} - ${error.error.message}`);
        }

        // Handle empty responses (e.g., 204 No Content)
        if (response.status === 204) {
          return undefined as any;
        }

        return await response.json();
      } catch (error) {
        lastError = error as Error;
        if (attempt < this.retryAttempts - 1) {
          // Exponential backoff
          await this.delay(Math.pow(2, attempt) * 1000);
        }
      }
    }

    throw lastError || new Error('Request failed');
  }

  private delay(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}

/**
 * Create WIA Elevator Client
 */
export function createClient(config: WIAElevatorClientConfig): WIAElevatorClient {
  return new WIAElevatorClient(config);
}

/**
 * Default export
 */
export default WIAElevatorClient;
