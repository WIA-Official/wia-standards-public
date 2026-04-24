# Chapter 4: API Interface Specifications

**弘益人間 (Benefit All Humanity)**

---

## Overview

This chapter defines comprehensive API interfaces for the WIA Cryo Preservation Standard. The APIs support REST, GraphQL, and WebSocket protocols to enable real-time specimen tracking, temperature monitoring, protocol management, and system integration.

---

## REST API Specification

### Base Configuration

```typescript
/**
 * WIA Cryo Preservation REST API
 * Complete API client implementation
 */

import axios, { AxiosInstance, AxiosRequestConfig } from 'axios';
import { z } from 'zod';
import {
  Specimen,
  PreservationProtocol,
  StorageTank,
  TemperatureMonitoring,
  ThawingRecord,
  AuditLog
} from './schemas';

/**
 * API configuration schema
 */
export const ApiConfigSchema = z.object({
  baseUrl: z.string().url(),
  apiKey: z.string().optional(),
  timeout: z.number().default(30000),
  retryAttempts: z.number().default(3),
  retryDelay: z.number().default(1000),
  enableLogging: z.boolean().default(false)
});

export type ApiConfig = z.infer<typeof ApiConfigSchema>;

/**
 * API response wrapper
 */
export const ApiResponseSchema = <T extends z.ZodTypeAny>(dataSchema: T) =>
  z.object({
    success: z.boolean(),
    data: dataSchema.optional(),
    error: z
      .object({
        code: z.string(),
        message: z.string(),
        details: z.any().optional()
      })
      .optional(),
    metadata: z
      .object({
        timestamp: z.string(),
        requestId: z.string(),
        version: z.string()
      })
      .optional()
  });

/**
 * Paginated response schema
 */
export const PaginatedResponseSchema = <T extends z.ZodTypeAny>(itemSchema: T) =>
  z.object({
    items: z.array(itemSchema),
    pagination: z.object({
      page: z.number(),
      pageSize: z.number(),
      totalItems: z.number(),
      totalPages: z.number(),
      hasNext: z.boolean(),
      hasPrevious: z.boolean()
    }),
    sorting: z
      .object({
        field: z.string(),
        order: z.enum(['asc', 'desc'])
      })
      .optional(),
    filters: z.record(z.any()).optional()
  });

/**
 * Main API client
 */
export class CryoPreservationApiClient {
  private client: AxiosInstance;
  private config: ApiConfig;

  constructor(config: ApiConfig) {
    this.config = ApiConfigSchema.parse(config);
    this.client = axios.create({
      baseURL: this.config.baseUrl,
      timeout: this.config.timeout,
      headers: {
        'Content-Type': 'application/json',
        ...(this.config.apiKey && { Authorization: `Bearer ${this.config.apiKey}` })
      }
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
          console.log(`[API Request] ${config.method?.toUpperCase()} ${config.url}`);
        }
        return config;
      },
      (error) => {
        console.error('[API Request Error]', error);
        return Promise.reject(error);
      }
    );

    // Response interceptor
    this.client.interceptors.response.use(
      (response) => {
        if (this.config.enableLogging) {
          console.log(`[API Response] ${response.status} ${response.config.url}`);
        }
        return response;
      },
      async (error) => {
        const config = error.config as AxiosRequestConfig & { _retry?: number };

        // Retry logic
        if (!config._retry) {
          config._retry = 0;
        }

        if (config._retry < this.config.retryAttempts) {
          config._retry++;
          await new Promise((resolve) => setTimeout(resolve, this.config.retryDelay));
          return this.client(config);
        }

        console.error('[API Response Error]', error);
        return Promise.reject(error);
      }
    );
  }

  // ==================== Specimen Endpoints ====================

  /**
   * Create a new specimen
   */
  async createSpecimen(specimen: Specimen): Promise<Specimen> {
    const response = await this.client.post('/api/v1/specimens', specimen);
    return response.data.data;
  }

  /**
   * Get specimen by ID
   */
  async getSpecimen(specimenId: string): Promise<Specimen> {
    const response = await this.client.get(`/api/v1/specimens/${specimenId}`);
    return response.data.data;
  }

  /**
   * Update specimen
   */
  async updateSpecimen(specimenId: string, updates: Partial<Specimen>): Promise<Specimen> {
    const response = await this.client.patch(`/api/v1/specimens/${specimenId}`, updates);
    return response.data.data;
  }

  /**
   * Delete specimen
   */
  async deleteSpecimen(specimenId: string): Promise<void> {
    await this.client.delete(`/api/v1/specimens/${specimenId}`);
  }

  /**
   * Search specimens with filters
   */
  async searchSpecimens(params: {
    category?: string;
    status?: string;
    donorId?: string;
    tankId?: string;
    dateFrom?: Date;
    dateTo?: Date;
    page?: number;
    pageSize?: number;
    sortBy?: string;
    sortOrder?: 'asc' | 'desc';
  }): Promise<{
    items: Specimen[];
    pagination: {
      page: number;
      pageSize: number;
      totalItems: number;
      totalPages: number;
    };
  }> {
    const response = await this.client.get('/api/v1/specimens/search', { params });
    return response.data.data;
  }

  /**
   * Get specimens by barcode
   */
  async getSpecimenByBarcode(barcode: string): Promise<Specimen> {
    const response = await this.client.get('/api/v1/specimens/barcode/' + barcode);
    return response.data.data;
  }

  /**
   * Get specimens by RFID tag
   */
  async getSpecimenByRFID(rfidTag: string): Promise<Specimen> {
    const response = await this.client.get('/api/v1/specimens/rfid/' + rfidTag);
    return response.data.data;
  }

  /**
   * Batch create specimens
   */
  async createSpecimenBatch(specimens: Specimen[]): Promise<{
    successful: Specimen[];
    failed: Array<{ specimen: Specimen; error: string }>;
  }> {
    const response = await this.client.post('/api/v1/specimens/batch', { specimens });
    return response.data.data;
  }

  /**
   * Move specimen to new location
   */
  async moveSpecimen(
    specimenId: string,
    newLocation: {
      tankId: string;
      canister: string;
      goblet?: string;
      position: string;
    }
  ): Promise<Specimen> {
    const response = await this.client.post(`/api/v1/specimens/${specimenId}/move`, newLocation);
    return response.data.data;
  }

  /**
   * Get specimen chain of custody
   */
  async getChainOfCustody(specimenId: string): Promise<any[]> {
    const response = await this.client.get(`/api/v1/specimens/${specimenId}/chain-of-custody`);
    return response.data.data;
  }

  /**
   * Export specimens to CSV/Excel
   */
  async exportSpecimens(
    format: 'csv' | 'excel',
    filters?: Record<string, any>
  ): Promise<Blob> {
    const response = await this.client.post(
      '/api/v1/specimens/export',
      { format, filters },
      { responseType: 'blob' }
    );
    return response.data;
  }

  // ==================== Protocol Endpoints ====================

  /**
   * Create preservation protocol
   */
  async createProtocol(protocol: PreservationProtocol): Promise<PreservationProtocol> {
    const response = await this.client.post('/api/v1/protocols', protocol);
    return response.data.data;
  }

  /**
   * Get protocol by ID
   */
  async getProtocol(protocolId: string): Promise<PreservationProtocol> {
    const response = await this.client.get(`/api/v1/protocols/${protocolId}`);
    return response.data.data;
  }

  /**
   * List all active protocols
   */
  async listProtocols(params?: {
    category?: string;
    status?: string;
    page?: number;
    pageSize?: number;
  }): Promise<{
    items: PreservationProtocol[];
    pagination: any;
  }> {
    const response = await this.client.get('/api/v1/protocols', { params });
    return response.data.data;
  }

  /**
   * Update protocol
   */
  async updateProtocol(
    protocolId: string,
    updates: Partial<PreservationProtocol>
  ): Promise<PreservationProtocol> {
    const response = await this.client.patch(`/api/v1/protocols/${protocolId}`, updates);
    return response.data.data;
  }

  /**
   * Find optimal protocol for specimen
   */
  async findOptimalProtocol(params: {
    category: string;
    subcategory: string;
    volume?: number;
    donorAge?: number;
  }): Promise<PreservationProtocol> {
    const response = await this.client.post('/api/v1/protocols/find-optimal', params);
    return response.data.data;
  }

  // ==================== Storage Tank Endpoints ====================

  /**
   * Register new storage tank
   */
  async registerTank(tank: StorageTank): Promise<StorageTank> {
    const response = await this.client.post('/api/v1/tanks', tank);
    return response.data.data;
  }

  /**
   * Get tank by ID
   */
  async getTank(tankId: string): Promise<StorageTank> {
    const response = await this.client.get(`/api/v1/tanks/${tankId}`);
    return response.data.data;
  }

  /**
   * List all tanks
   */
  async listTanks(params?: {
    facilityId?: string;
    type?: string;
    status?: string;
  }): Promise<StorageTank[]> {
    const response = await this.client.get('/api/v1/tanks', { params });
    return response.data.data;
  }

  /**
   * Update tank status
   */
  async updateTankStatus(
    tankId: string,
    status: {
      currentTemperature: number;
      liquidNitrogenLevel: number;
      currentLoad: number;
    }
  ): Promise<StorageTank> {
    const response = await this.client.patch(`/api/v1/tanks/${tankId}/status`, status);
    return response.data.data;
  }

  /**
   * Get tank inventory
   */
  async getTankInventory(tankId: string): Promise<{
    tankId: string;
    totalCapacity: number;
    currentLoad: number;
    utilizationRate: number;
    specimens: Array<{
      specimenId: string;
      barcode: string;
      location: string;
      category: string;
    }>;
  }> {
    const response = await this.client.get(`/api/v1/tanks/${tankId}/inventory`);
    return response.data.data;
  }

  /**
   * Get available storage locations in tank
   */
  async getAvailableLocations(tankId: string): Promise<
    Array<{
      canister: string;
      goblet?: string;
      position: string;
      available: boolean;
    }>
  > {
    const response = await this.client.get(`/api/v1/tanks/${tankId}/available-locations`);
    return response.data.data;
  }

  // ==================== Temperature Monitoring Endpoints ====================

  /**
   * Get current temperature readings
   */
  async getCurrentTemperature(tankId: string): Promise<{
    tankId: string;
    readings: Array<{
      sensorId: string;
      location: string;
      temperature: number;
      timestamp: Date;
    }>;
  }> {
    const response = await this.client.get(`/api/v1/monitoring/tanks/${tankId}/current`);
    return response.data.data;
  }

  /**
   * Get temperature history
   */
  async getTemperatureHistory(
    tankId: string,
    params: {
      startDate: Date;
      endDate: Date;
      interval?: 'minute' | 'hour' | 'day';
    }
  ): Promise<TemperatureMonitoring> {
    const response = await this.client.get(`/api/v1/monitoring/tanks/${tankId}/history`, {
      params: {
        startDate: params.startDate.toISOString(),
        endDate: params.endDate.toISOString(),
        interval: params.interval
      }
    });
    return response.data.data;
  }

  /**
   * Get active alarms
   */
  async getActiveAlarms(facilityId?: string): Promise<
    Array<{
      alarmId: string;
      tankId: string;
      type: string;
      severity: string;
      message: string;
      triggeredAt: Date;
    }>
  > {
    const response = await this.client.get('/api/v1/monitoring/alarms', {
      params: { facilityId }
    });
    return response.data.data;
  }

  /**
   * Acknowledge alarm
   */
  async acknowledgeAlarm(
    alarmId: string,
    acknowledgement: {
      acknowledgedBy: string;
      notes?: string;
    }
  ): Promise<void> {
    await this.client.post(`/api/v1/monitoring/alarms/${alarmId}/acknowledge`, acknowledgement);
  }

  /**
   * Get temperature excursions
   */
  async getTemperatureExcursions(params: {
    tankId?: string;
    startDate: Date;
    endDate: Date;
    minDuration?: number; // minutes
  }): Promise<
    Array<{
      excursionId: string;
      tankId: string;
      startTime: Date;
      endTime: Date;
      peakDeviation: number;
      duration: number;
      affectedSpecimens: string[];
    }>
  > {
    const response = await this.client.get('/api/v1/monitoring/excursions', {
      params: {
        ...params,
        startDate: params.startDate.toISOString(),
        endDate: params.endDate.toISOString()
      }
    });
    return response.data.data;
  }

  // ==================== Thawing Endpoints ====================

  /**
   * Create thawing record
   */
  async createThawingRecord(record: ThawingRecord): Promise<ThawingRecord> {
    const response = await this.client.post('/api/v1/thawing', record);
    return response.data.data;
  }

  /**
   * Get thawing record
   */
  async getThawingRecord(thawingId: string): Promise<ThawingRecord> {
    const response = await this.client.get(`/api/v1/thawing/${thawingId}`);
    return response.data.data;
  }

  /**
   * Get thawing records for specimen
   */
  async getSpecimenThawingHistory(specimenId: string): Promise<ThawingRecord[]> {
    const response = await this.client.get(`/api/v1/specimens/${specimenId}/thawing-history`);
    return response.data.data;
  }

  // ==================== Audit Log Endpoints ====================

  /**
   * Get audit logs
   */
  async getAuditLogs(params: {
    resourceType?: string;
    resourceId?: string;
    actionType?: string;
    userId?: string;
    startDate?: Date;
    endDate?: Date;
    page?: number;
    pageSize?: number;
  }): Promise<{
    items: AuditLog[];
    pagination: any;
  }> {
    const response = await this.client.get('/api/v1/audit-logs', {
      params: {
        ...params,
        startDate: params.startDate?.toISOString(),
        endDate: params.endDate?.toISOString()
      }
    });
    return response.data.data;
  }

  /**
   * Export audit logs
   */
  async exportAuditLogs(
    format: 'csv' | 'excel' | 'pdf',
    filters: Record<string, any>
  ): Promise<Blob> {
    const response = await this.client.post(
      '/api/v1/audit-logs/export',
      { format, filters },
      { responseType: 'blob' }
    );
    return response.data;
  }

  // ==================== Report Endpoints ====================

  /**
   * Generate specimen inventory report
   */
  async generateInventoryReport(params: {
    facilityId?: string;
    tankId?: string;
    category?: string;
    format: 'pdf' | 'excel' | 'html';
  }): Promise<Blob> {
    const response = await this.client.post('/api/v1/reports/inventory', params, {
      responseType: 'blob'
    });
    return response.data;
  }

  /**
   * Generate quality control report
   */
  async generateQualityReport(params: {
    startDate: Date;
    endDate: Date;
    facilityId?: string;
    format: 'pdf' | 'excel';
  }): Promise<Blob> {
    const response = await this.client.post(
      '/api/v1/reports/quality',
      {
        ...params,
        startDate: params.startDate.toISOString(),
        endDate: params.endDate.toISOString()
      },
      { responseType: 'blob' }
    );
    return response.data;
  }

  /**
   * Generate compliance report
   */
  async generateComplianceReport(params: {
    year: number;
    quarter?: number;
    facilityId?: string;
    format: 'pdf' | 'excel';
  }): Promise<Blob> {
    const response = await this.client.post('/api/v1/reports/compliance', params, {
      responseType: 'blob'
    });
    return response.data;
  }
}
```

---

## GraphQL API Specification

```typescript
/**
 * GraphQL API for flexible querying
 */

import { GraphQLClient, gql } from 'graphql-request';

export class CryoPreservationGraphQLClient {
  private client: GraphQLClient;

  constructor(endpoint: string, apiKey?: string) {
    this.client = new GraphQLClient(endpoint, {
      headers: {
        ...(apiKey && { Authorization: `Bearer ${apiKey}` })
      }
    });
  }

  /**
   * Query specimen with custom fields
   */
  async querySpecimen(specimenId: string, fields: string[]): Promise<any> {
    const query = gql`
      query GetSpecimen($specimenId: ID!) {
        specimen(id: $specimenId) {
          ${fields.join('\n')}
        }
      }
    `;

    const data = await this.client.request(query, { specimenId });
    return data.specimen;
  }

  /**
   * Search specimens with complex filters
   */
  async searchSpecimens(filter: {
    category?: string[];
    status?: string[];
    qualityGrade?: string[];
    donorAgeLt?: number;
    donorAgeGt?: number;
    viabilityGte?: number;
    freezeDateAfter?: Date;
    freezeDateBefore?: Date;
  }): Promise<any[]> {
    const query = gql`
      query SearchSpecimens($filter: SpecimenFilterInput!) {
        specimens(filter: $filter) {
          specimenId
          barcode
          category
          subcategory
          donor {
            demographics {
              age
              sex
            }
          }
          physicalProperties {
            viability {
              percentage
            }
          }
          quality {
            overallGrade
            qualityScore
          }
          storage {
            tank {
              tankId
              tankName
            }
            location {
              canister
              position
            }
          }
          status
          metadata {
            createdAt
          }
        }
      }
    `;

    const data = await this.client.request(query, { filter });
    return data.specimens;
  }

  /**
   * Get tank with nested specimen data
   */
  async getTankWithSpecimens(tankId: string): Promise<any> {
    const query = gql`
      query GetTank($tankId: ID!) {
        tank(id: $tankId) {
          tankId
          tankName
          specifications {
            type
            capacity {
              total
              specimenCapacity
            }
          }
          status {
            currentTemperature {
              value
              unit
            }
            liquidNitrogenLevel
            currentLoad
            utilizationRate
          }
          organization {
            canisters {
              canisterId
              currentLoad
              goblets {
                gobletId
                specimenIds
                specimens {
                  specimenId
                  barcode
                  category
                  status
                }
              }
            }
          }
        }
      }
    `;

    const data = await this.client.request(query, { tankId });
    return data.tank;
  }

  /**
   * Get temperature monitoring with statistics
   */
  async getTemperatureMonitoring(
    tankId: string,
    startDate: Date,
    endDate: Date
  ): Promise<any> {
    const query = gql`
      query GetTemperatureMonitoring($tankId: ID!, $startDate: DateTime!, $endDate: DateTime!) {
        temperatureMonitoring(tankId: $tankId, startDate: $startDate, endDate: $endDate) {
          monitoringId
          readings {
            timestamp
            temperature {
              value
              unit
            }
            sensorLocation
          }
          statistics {
            temperatureStats {
              mean
              min
              max
              standardDeviation
            }
            excursions {
              excursionId
              startTime
              endTime
              peakDeviation
              duration
              affectedSpecimens {
                specimenId
                barcode
              }
            }
          }
        }
      }
    `;

    const data = await this.client.request(query, {
      tankId,
      startDate: startDate.toISOString(),
      endDate: endDate.toISOString()
    });
    return data.temperatureMonitoring;
  }

  /**
   * Create specimen mutation
   */
  async createSpecimen(specimen: any): Promise<any> {
    const mutation = gql`
      mutation CreateSpecimen($input: SpecimenInput!) {
        createSpecimen(input: $input) {
          specimenId
          barcode
          category
          status
          metadata {
            createdAt
          }
        }
      }
    `;

    const data = await this.client.request(mutation, { input: specimen });
    return data.createSpecimen;
  }

  /**
   * Update specimen mutation
   */
  async updateSpecimen(specimenId: string, updates: any): Promise<any> {
    const mutation = gql`
      mutation UpdateSpecimen($specimenId: ID!, $updates: SpecimenUpdateInput!) {
        updateSpecimen(id: $specimenId, updates: $updates) {
          specimenId
          barcode
          status
          metadata {
            updatedAt
          }
        }
      }
    `;

    const data = await this.client.request(mutation, { specimenId, updates });
    return data.updateSpecimen;
  }

  /**
   * Subscribe to temperature updates
   */
  subscribeToTemperatureUpdates(tankId: string, callback: (data: any) => void): void {
    const subscription = gql`
      subscription OnTemperatureUpdate($tankId: ID!) {
        temperatureUpdated(tankId: $tankId) {
          tankId
          reading {
            timestamp
            temperature {
              value
              unit
            }
            sensorId
            sensorLocation
          }
        }
      }
    `;

    // Note: This is a simplified example. In practice, you'd use a WebSocket client
    console.log('Subscription query:', subscription);
    console.log('Tank ID:', tankId);
    // Implementation depends on your GraphQL subscription setup
  }

  /**
   * Subscribe to alarm notifications
   */
  subscribeToAlarms(facilityId: string, callback: (alarm: any) => void): void {
    const subscription = gql`
      subscription OnAlarm($facilityId: ID!) {
        alarmTriggered(facilityId: $facilityId) {
          alarmId
          tankId
          type
          severity
          message
          triggeredAt
          value
          threshold
        }
      }
    `;

    console.log('Alarm subscription query:', subscription);
    console.log('Facility ID:', facilityId);
    // Implementation depends on your GraphQL subscription setup
  }
}
```

---

## WebSocket API for Real-Time Updates

```typescript
/**
 * WebSocket client for real-time monitoring
 */

export interface WebSocketMessage {
  type: string;
  payload: any;
  timestamp: Date;
}

export class CryoPreservationWebSocketClient {
  private ws: WebSocket | null = null;
  private reconnectAttempts = 0;
  private maxReconnectAttempts = 5;
  private reconnectDelay = 1000;
  private eventHandlers: Map<string, Array<(payload: any) => void>> = new Map();

  constructor(private url: string, private apiKey?: string) {}

  /**
   * Connect to WebSocket server
   */
  connect(): Promise<void> {
    return new Promise((resolve, reject) => {
      const wsUrl = this.apiKey ? `${this.url}?token=${this.apiKey}` : this.url;
      this.ws = new WebSocket(wsUrl);

      this.ws.onopen = () => {
        console.log('[WebSocket] Connected');
        this.reconnectAttempts = 0;
        resolve();
      };

      this.ws.onmessage = (event) => {
        try {
          const message: WebSocketMessage = JSON.parse(event.data);
          this.handleMessage(message);
        } catch (error) {
          console.error('[WebSocket] Failed to parse message:', error);
        }
      };

      this.ws.onerror = (error) => {
        console.error('[WebSocket] Error:', error);
        reject(error);
      };

      this.ws.onclose = () => {
        console.log('[WebSocket] Disconnected');
        this.attemptReconnect();
      };
    });
  }

  /**
   * Disconnect from WebSocket server
   */
  disconnect(): void {
    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }
  }

  /**
   * Attempt to reconnect
   */
  private attemptReconnect(): void {
    if (this.reconnectAttempts < this.maxReconnectAttempts) {
      this.reconnectAttempts++;
      console.log(
        `[WebSocket] Reconnecting... Attempt ${this.reconnectAttempts}/${this.maxReconnectAttempts}`
      );

      setTimeout(() => {
        this.connect().catch((error) => {
          console.error('[WebSocket] Reconnection failed:', error);
        });
      }, this.reconnectDelay * this.reconnectAttempts);
    } else {
      console.error('[WebSocket] Max reconnection attempts reached');
    }
  }

  /**
   * Handle incoming message
   */
  private handleMessage(message: WebSocketMessage): void {
    const handlers = this.eventHandlers.get(message.type);
    if (handlers) {
      handlers.forEach((handler) => handler(message.payload));
    }
  }

  /**
   * Subscribe to event
   */
  on(eventType: string, handler: (payload: any) => void): void {
    if (!this.eventHandlers.has(eventType)) {
      this.eventHandlers.set(eventType, []);
    }
    this.eventHandlers.get(eventType)!.push(handler);
  }

  /**
   * Unsubscribe from event
   */
  off(eventType: string, handler: (payload: any) => void): void {
    const handlers = this.eventHandlers.get(eventType);
    if (handlers) {
      const index = handlers.indexOf(handler);
      if (index > -1) {
        handlers.splice(index, 1);
      }
    }
  }

  /**
   * Send message to server
   */
  send(type: string, payload: any): void {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      const message: WebSocketMessage = {
        type,
        payload,
        timestamp: new Date()
      };
      this.ws.send(JSON.stringify(message));
    } else {
      console.error('[WebSocket] Cannot send message: not connected');
    }
  }

  /**
   * Subscribe to temperature updates for a tank
   */
  subscribeToTankTemperature(tankId: string): void {
    this.send('subscribe', {
      channel: 'temperature',
      tankId
    });
  }

  /**
   * Unsubscribe from temperature updates
   */
  unsubscribeFromTankTemperature(tankId: string): void {
    this.send('unsubscribe', {
      channel: 'temperature',
      tankId
    });
  }

  /**
   * Subscribe to all alarms
   */
  subscribeToAlarms(facilityId?: string): void {
    this.send('subscribe', {
      channel: 'alarms',
      facilityId
    });
  }

  /**
   * Subscribe to specimen changes
   */
  subscribeToSpecimenChanges(specimenId: string): void {
    this.send('subscribe', {
      channel: 'specimen',
      specimenId
    });
  }
}
```

---

## Complete Usage Example

```typescript
/**
 * Complete example demonstrating all API clients
 */

async function demonstrateCryoPreservationAPIs() {
  // ==================== REST API ====================
  console.log('=== REST API Demo ===\n');

  const restClient = new CryoPreservationApiClient({
    baseUrl: 'https://api.cryobank.example.com',
    apiKey: 'your-api-key-here',
    timeout: 30000,
    retryAttempts: 3,
    enableLogging: true
  });

  // Create a specimen
  const newSpecimen = await restClient.createSpecimen({
    specimenId: crypto.randomUUID(),
    barcode: 'CRYO-2025-001234',
    category: 'GAMETES',
    subcategory: 'SPERM',
    // ... other fields
  } as any);

  console.log('Created specimen:', newSpecimen.specimenId);

  // Search specimens
  const searchResults = await restClient.searchSpecimens({
    category: 'GAMETES',
    status: 'ACTIVE',
    page: 1,
    pageSize: 20
  });

  console.log(`Found ${searchResults.pagination.totalItems} specimens`);

  // Get temperature monitoring
  const temperature = await restClient.getCurrentTemperature('TANK-001');
  console.log('Current temperature:', temperature.readings);

  // ==================== GraphQL API ====================
  console.log('\n=== GraphQL API Demo ===\n');

  const graphqlClient = new CryoPreservationGraphQLClient(
    'https://api.cryobank.example.com/graphql',
    'your-api-key-here'
  );

  // Query specimen with specific fields
  const specimen = await graphqlClient.querySpecimen('specimen-uuid', [
    'specimenId',
    'barcode',
    'category',
    'quality { overallGrade qualityScore }',
    'storage { tank { tankName } }'
  ]);

  console.log('Specimen:', specimen);

  // Complex search
  const specimens = await graphqlClient.searchSpecimens({
    category: ['GAMETES', 'EMBRYOS'],
    qualityGrade: ['EXCELLENT', 'GOOD'],
    viabilityGte: 80
  });

  console.log(`Found ${specimens.length} high-quality specimens`);

  // ==================== WebSocket API ====================
  console.log('\n=== WebSocket API Demo ===\n');

  const wsClient = new CryoPreservationWebSocketClient(
    'wss://api.cryobank.example.com/ws',
    'your-api-key-here'
  );

  // Connect
  await wsClient.connect();

  // Subscribe to temperature updates
  wsClient.on('temperature', (data) => {
    console.log('[Real-time] Temperature update:', data);

    if (data.temperature.value > -150) {
      console.warn('⚠️ Temperature above safe threshold!');
    }
  });

  wsClient.subscribeToTankTemperature('TANK-001');

  // Subscribe to alarms
  wsClient.on('alarm', (alarm) => {
    console.error('🚨 ALARM:', alarm.message);

    if (alarm.severity === 'CRITICAL') {
      console.error('!!! CRITICAL ALARM - IMMEDIATE ACTION REQUIRED !!!');
      // Trigger emergency notification
    }
  });

  wsClient.subscribeToAlarms();

  // Subscribe to specimen changes
  wsClient.on('specimen', (data) => {
    console.log('[Real-time] Specimen changed:', data);
  });

  wsClient.subscribeToSpecimenChanges(newSpecimen.specimenId);

  // Keep connection alive for 60 seconds
  await new Promise((resolve) => setTimeout(resolve, 60000));

  // Cleanup
  wsClient.disconnect();
}

// Run the demo
demonstrateCryoPreservationAPIs().catch(console.error);
```

---

## API Error Handling

```typescript
/**
 * Comprehensive error handling for API clients
 */

export class CryoApiError extends Error {
  constructor(
    public code: string,
    message: string,
    public status?: number,
    public details?: any
  ) {
    super(message);
    this.name = 'CryoApiError';
  }
}

export class ApiErrorHandler {
  /**
   * Handle API error
   */
  static handleError(error: any): never {
    if (error.response) {
      // HTTP error response
      const status = error.response.status;
      const data = error.response.data;

      switch (status) {
        case 400:
          throw new CryoApiError('BAD_REQUEST', 'Invalid request parameters', status, data);
        case 401:
          throw new CryoApiError('UNAUTHORIZED', 'Authentication required', status, data);
        case 403:
          throw new CryoApiError('FORBIDDEN', 'Access denied', status, data);
        case 404:
          throw new CryoApiError('NOT_FOUND', 'Resource not found', status, data);
        case 409:
          throw new CryoApiError('CONFLICT', 'Resource conflict', status, data);
        case 422:
          throw new CryoApiError('VALIDATION_ERROR', 'Validation failed', status, data);
        case 429:
          throw new CryoApiError('RATE_LIMIT', 'Too many requests', status, data);
        case 500:
          throw new CryoApiError('SERVER_ERROR', 'Internal server error', status, data);
        case 503:
          throw new CryoApiError('SERVICE_UNAVAILABLE', 'Service temporarily unavailable', status, data);
        default:
          throw new CryoApiError('UNKNOWN_ERROR', `HTTP error ${status}`, status, data);
      }
    } else if (error.request) {
      // No response received
      throw new CryoApiError('NETWORK_ERROR', 'No response from server', undefined, error);
    } else {
      // Other errors
      throw new CryoApiError('CLIENT_ERROR', error.message, undefined, error);
    }
  }

  /**
   * Retry with exponential backoff
   */
  static async retryWithBackoff<T>(
    fn: () => Promise<T>,
    maxAttempts: number = 3,
    initialDelay: number = 1000
  ): Promise<T> {
    let lastError: any;

    for (let attempt = 0; attempt < maxAttempts; attempt++) {
      try {
        return await fn();
      } catch (error) {
        lastError = error;

        if (attempt < maxAttempts - 1) {
          const delay = initialDelay * Math.pow(2, attempt);
          console.log(`Retry attempt ${attempt + 1}/${maxAttempts} after ${delay}ms`);
          await new Promise((resolve) => setTimeout(resolve, delay));
        }
      }
    }

    throw lastError;
  }
}
```

---

## Summary

This chapter provides comprehensive API interfaces for the WIA Cryo Preservation Standard:

- **REST API**: Full CRUD operations for specimens, protocols, tanks, and monitoring
- **GraphQL API**: Flexible querying with custom fields and complex filters
- **WebSocket API**: Real-time temperature monitoring and alarm notifications
- **Error Handling**: Robust error management with retry logic
- **Type Safety**: Full TypeScript support with Zod validation

All APIs support:
- Authentication via API keys
- Pagination for large datasets
- Filtering and sorting
- Export functionality
- Real-time updates
- Comprehensive error handling

---

**弘益人間 (Benefit All Humanity)**

*Standardized APIs enable seamless integration, real-time monitoring, and global collaboration in cryopreservation technology.*
