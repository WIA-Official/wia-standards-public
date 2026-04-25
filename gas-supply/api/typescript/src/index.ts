/**
 * WIA-SOC-011: Gas Supply Standard - TypeScript SDK
 * Version: 1.0.0
 *
 * © 2025 World Certification Industry Association
 * License: MIT
 */

import axios, { AxiosInstance, AxiosError } from 'axios';
import WebSocket from 'ws';
import {
  GasSupplyClientConfig,
  Pipeline,
  ListPipelinesParams,
  ListPipelinesResponse,
  Measurement,
  MeasurementQueryParams,
  HistoricalDataParams,
  TimeSeriesData,
  GasComposition,
  Event,
  ControlCommand,
  CommandResponse,
  WebSocketSubscription,
  GasSupplyError,
  APIError,
} from './types';

export * from './types';

/**
 * Main client for WIA-SOC-011 Gas Supply API
 */
export class GasSupplyClient {
  private axios: AxiosInstance;
  private wsConnection?: WebSocket;

  constructor(private config: GasSupplyClientConfig) {
    this.axios = axios.create({
      baseURL: config.baseUrl,
      timeout: config.timeout || 30000,
      headers: {
        'Content-Type': 'application/json',
        'User-Agent': '@wia/gas-supply-sdk/1.0.0',
        ...(config.apiKey && { 'X-API-Key': config.apiKey }),
        ...(config.accessToken && { Authorization: `Bearer ${config.accessToken}` }),
      },
    });

    // Add response interceptor for error handling
    this.axios.interceptors.response.use(
      (response) => response,
      (error: AxiosError<APIError>) => {
        if (error.response?.data) {
          const apiError = error.response.data;
          throw new GasSupplyError(
            apiError.message,
            apiError.code,
            apiError.details,
            apiError.correlationId
          );
        }
        throw error;
      }
    );
  }

  // ========================================================================
  // Pipeline Methods
  // ========================================================================

  /**
   * List pipelines with optional filters
   */
  async listPipelines(params?: ListPipelinesParams): Promise<ListPipelinesResponse> {
    const response = await this.axios.get<ListPipelinesResponse>('/pipelines', { params });
    return response.data;
  }

  /**
   * Get detailed information about a specific pipeline
   */
  async getPipeline(pipelineId: string): Promise<Pipeline> {
    const response = await this.axios.get<Pipeline>(`/pipelines/${pipelineId}`);
    return response.data;
  }

  /**
   * Create a new pipeline
   */
  async createPipeline(pipeline: Omit<Pipeline, 'pipelineId'>): Promise<Pipeline> {
    const response = await this.axios.post<Pipeline>('/pipelines', pipeline);
    return response.data;
  }

  /**
   * Update an existing pipeline
   */
  async updatePipeline(pipelineId: string, updates: Partial<Pipeline>): Promise<Pipeline> {
    const response = await this.axios.put<Pipeline>(`/pipelines/${pipelineId}`, updates);
    return response.data;
  }

  /**
   * Delete a pipeline
   */
  async deletePipeline(pipelineId: string): Promise<void> {
    await this.axios.delete(`/pipelines/${pipelineId}`);
  }

  // ========================================================================
  // Measurement Methods
  // ========================================================================

  /**
   * Get real-time measurements
   */
  async getRealtimeMeasurements(params?: MeasurementQueryParams): Promise<Measurement[]> {
    const response = await this.axios.get<{ measurements: Measurement[] }>(
      '/measurements/realtime',
      { params }
    );
    return response.data.measurements;
  }

  /**
   * Get latest measurement for a specific asset
   */
  async getLatestMeasurement(assetId: string, type: string): Promise<Measurement> {
    const response = await this.axios.get<Measurement>(
      `/assets/${assetId}/measurements/latest`,
      { params: { type } }
    );
    return response.data;
  }

  /**
   * Get historical time series data
   */
  async getHistoricalData(params: HistoricalDataParams): Promise<TimeSeriesData> {
    const response = await this.axios.get<TimeSeriesData>('/measurements/history', { params });
    return response.data;
  }

  // ========================================================================
  // Gas Quality Methods
  // ========================================================================

  /**
   * Get gas composition at a location
   */
  async getGasComposition(location: string, timestamp?: string): Promise<GasComposition> {
    const response = await this.axios.get<GasComposition>('/gas-quality/composition', {
      params: { location, timestamp },
    });
    return response.data;
  }

  // ========================================================================
  // Equipment Control Methods
  // ========================================================================

  /**
   * Send command to compressor
   */
  async controlCompressor(compressorId: string, command: ControlCommand): Promise<CommandResponse> {
    const response = await this.axios.post<CommandResponse>(
      `/equipment/compressors/${compressorId}/commands`,
      command
    );
    return response.data;
  }

  /**
   * Send command to valve
   */
  async controlValve(valveId: string, command: ControlCommand): Promise<CommandResponse> {
    const response = await this.axios.post<CommandResponse>(
      `/equipment/valves/${valveId}/commands`,
      command
    );
    return response.data;
  }

  /**
   * Get command status
   */
  async getCommandStatus(commandId: string): Promise<CommandResponse> {
    const response = await this.axios.get<CommandResponse>(`/commands/${commandId}/status`);
    return response.data;
  }

  // ========================================================================
  // Event and Alarm Methods
  // ========================================================================

  /**
   * Get events with optional filters
   */
  async getEvents(params?: {
    severity?: string[];
    category?: string[];
    since?: string;
  }): Promise<Event[]> {
    const response = await this.axios.get<{ events: Event[] }>('/events', { params });
    return response.data.events;
  }

  /**
   * Acknowledge an event/alarm
   */
  async acknowledgeEvent(
    eventId: string,
    acknowledgedBy: string,
    notes?: string
  ): Promise<Event> {
    const response = await this.axios.post<Event>(`/events/${eventId}/acknowledge`, {
      acknowledgedBy,
      notes,
    });
    return response.data;
  }

  // ========================================================================
  // WebSocket Real-time Streaming
  // ========================================================================

  /**
   * Connect to WebSocket for real-time updates
   */
  connectWebSocket(
    subscription: WebSocketSubscription,
    onMessage: (data: any) => void,
    onError?: (error: Error) => void
  ): void {
    const wsUrl = this.config.baseUrl.replace(/^http/, 'ws') + '/stream';

    this.wsConnection = new WebSocket(wsUrl, {
      headers: {
        ...(this.config.apiKey && { 'X-API-Key': this.config.apiKey }),
        ...(this.config.accessToken && { Authorization: `Bearer ${this.config.accessToken}` }),
      },
    });

    this.wsConnection.on('open', () => {
      this.wsConnection?.send(
        JSON.stringify({
          action: 'subscribe',
          channels: subscription.channels,
        })
      );
    });

    this.wsConnection.on('message', (data: WebSocket.Data) => {
      try {
        const parsed = JSON.parse(data.toString());
        onMessage(parsed);
      } catch (error) {
        if (onError) {
          onError(error as Error);
        }
      }
    });

    this.wsConnection.on('error', (error) => {
      if (onError) {
        onError(error);
      }
    });
  }

  /**
   * Disconnect WebSocket
   */
  disconnectWebSocket(): void {
    if (this.wsConnection) {
      this.wsConnection.close();
      this.wsConnection = undefined;
    }
  }

  // ========================================================================
  // Batch Operations
  // ========================================================================

  /**
   * Execute multiple operations in a single request
   */
  async batch(operations: Array<{ method: string; path: string; body?: any }>): Promise<any[]> {
    const response = await this.axios.post<{ results: any[] }>('/batch', { operations });
    return response.data.results;
  }

  // ========================================================================
  // Utility Methods
  // ========================================================================

  /**
   * Check API health
   */
  async healthCheck(): Promise<{ status: string; timestamp: string }> {
    const response = await this.axios.get<{ status: string; timestamp: string }>('/health');
    return response.data;
  }

  /**
   * Get API version information
   */
  async getVersion(): Promise<{ version: string; standard: string }> {
    const response = await this.axios.get<{ version: string; standard: string }>('/version');
    return response.data;
  }
}

/**
 * Create a new Gas Supply API client
 */
export function createClient(config: GasSupplyClientConfig): GasSupplyClient {
  return new GasSupplyClient(config);
}

/**
 * Default export
 */
export default {
  GasSupplyClient,
  createClient,
};
