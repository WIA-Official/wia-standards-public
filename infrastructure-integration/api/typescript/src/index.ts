/**
 * WIA-UNI-005 Infrastructure Integration Standard
 * TypeScript SDK
 *
 * @version 1.0.0
 * @license MIT
 */

import axios, { AxiosInstance, AxiosRequestConfig } from 'axios';
import {
  InfrastructureProject,
  InfrastructureAsset,
  SensorReading,
  MaintenanceRecord,
  EnvironmentalReading,
  StatusUpdate,
  APIResponse,
  PaginatedResponse,
  QueryParams,
  APIError,
} from './types';

/**
 * SDK Configuration options
 */
export interface WIAInfrastructureConfig {
  /**
   * API base URL
   */
  baseURL: string;

  /**
   * OAuth 2.0 access token
   */
  accessToken: string;

  /**
   * Region identifier
   */
  region?: string;

  /**
   * API version
   */
  apiVersion?: string;

  /**
   * Request timeout in milliseconds
   */
  timeout?: number;
}

/**
 * Main SDK class for WIA-UNI-005 Infrastructure Integration
 */
export class WIAInfrastructureSDK {
  private client: AxiosInstance;
  private config: WIAInfrastructureConfig;

  /**
   * Create a new SDK instance
   * @param config - SDK configuration
   */
  constructor(config: WIAInfrastructureConfig) {
    this.config = {
      apiVersion: 'v1',
      region: 'default',
      timeout: 30000,
      ...config,
    };

    this.client = axios.create({
      baseURL: `${this.config.baseURL}/${this.config.region}/${this.config.apiVersion}`,
      timeout: this.config.timeout,
      headers: {
        'Authorization': `Bearer ${this.config.accessToken}`,
        'Content-Type': 'application/json',
        'Accept': 'application/json',
      },
    });

    // Add response interceptor for error handling
    this.client.interceptors.response.use(
      (response) => response,
      (error) => {
        if (error.response?.data) {
          throw error.response.data as APIError;
        }
        throw error;
      }
    );
  }

  // ==================== Project Management ====================

  /**
   * Create a new infrastructure project
   * @param project - Project data
   * @returns Created project with ID and links
   */
  async createProject(
    project: Omit<InfrastructureProject, 'id' | '@context'>
  ): Promise<APIResponse<InfrastructureProject>> {
    const response = await this.client.post<APIResponse<InfrastructureProject>>(
      '/projects',
      {
        '@context': 'https://wiastandards.com/contexts/uni-005/v1',
        ...project,
      }
    );
    return response.data;
  }

  /**
   * Get a project by ID
   * @param projectId - Project identifier
   * @returns Project data
   */
  async getProject(projectId: string): Promise<APIResponse<InfrastructureProject>> {
    const response = await this.client.get<APIResponse<InfrastructureProject>>(
      `/projects/${projectId}`
    );
    return response.data;
  }

  /**
   * Query projects with filters
   * @param params - Query parameters
   * @returns Paginated project list
   */
  async queryProjects(
    params?: QueryParams
  ): Promise<PaginatedResponse<InfrastructureProject>> {
    const response = await this.client.get<PaginatedResponse<InfrastructureProject>>(
      '/projects',
      { params }
    );
    return response.data;
  }

  /**
   * Update a project
   * @param projectId - Project identifier
   * @param updates - Partial project updates
   * @returns Updated project
   */
  async updateProject(
    projectId: string,
    updates: Partial<InfrastructureProject>
  ): Promise<APIResponse<InfrastructureProject>> {
    const response = await this.client.patch<APIResponse<InfrastructureProject>>(
      `/projects/${projectId}`,
      updates
    );
    return response.data;
  }

  /**
   * Delete a project
   * @param projectId - Project identifier
   */
  async deleteProject(projectId: string): Promise<void> {
    await this.client.delete(`/projects/${projectId}`);
  }

  // ==================== Asset Registry ====================

  /**
   * Register a new infrastructure asset
   * @param asset - Asset data
   * @returns Registered asset with ID
   */
  async registerAsset(
    asset: Omit<InfrastructureAsset, 'id' | '@context'>
  ): Promise<APIResponse<InfrastructureAsset>> {
    const response = await this.client.post<APIResponse<InfrastructureAsset>>(
      '/assets',
      {
        '@context': 'https://wiastandards.com/contexts/uni-005/v1',
        ...asset,
      }
    );
    return response.data;
  }

  /**
   * Get an asset by ID
   * @param assetId - Asset identifier
   * @returns Asset data
   */
  async getAsset(assetId: string): Promise<APIResponse<InfrastructureAsset>> {
    const response = await this.client.get<APIResponse<InfrastructureAsset>>(
      `/assets/${assetId}`
    );
    return response.data;
  }

  /**
   * Query assets with filters
   * @param params - Query parameters
   * @returns Paginated asset list
   */
  async queryAssets(
    params?: QueryParams
  ): Promise<PaginatedResponse<InfrastructureAsset>> {
    const response = await this.client.get<PaginatedResponse<InfrastructureAsset>>(
      '/assets',
      { params }
    );
    return response.data;
  }

  /**
   * Update asset status
   * @param assetId - Asset identifier
   * @param status - New operational status
   * @param reason - Reason for status change
   * @returns Updated asset
   */
  async updateAssetStatus(
    assetId: string,
    status: string,
    reason?: string
  ): Promise<APIResponse<InfrastructureAsset>> {
    const response = await this.client.patch<APIResponse<InfrastructureAsset>>(
      `/assets/${assetId}/status`,
      { operationalStatus: status, reason }
    );
    return response.data;
  }

  // ==================== Real-time Monitoring ====================

  /**
   * Get latest sensor reading
   * @param sensorId - Sensor identifier
   * @returns Latest sensor reading
   */
  async getSensorReading(sensorId: string): Promise<APIResponse<SensorReading>> {
    const response = await this.client.get<APIResponse<SensorReading>>(
      `/monitoring/sensors/${sensorId}/latest`
    );
    return response.data;
  }

  /**
   * Get historical sensor data
   * @param sensorId - Sensor identifier
   * @param startDate - Start of time range (ISO 8601)
   * @param endDate - End of time range (ISO 8601)
   * @param aggregation - Aggregation interval (e.g., "1h", "1d")
   * @returns Historical sensor readings
   */
  async getSensorHistory(
    sensorId: string,
    startDate: string,
    endDate: string,
    aggregation?: string
  ): Promise<PaginatedResponse<SensorReading>> {
    const response = await this.client.get<PaginatedResponse<SensorReading>>(
      `/monitoring/sensors/${sensorId}/history`,
      {
        params: { start: startDate, end: endDate, aggregation },
      }
    );
    return response.data;
  }

  /**
   * Submit sensor reading
   * @param reading - Sensor reading data
   * @returns Confirmation
   */
  async submitSensorReading(
    reading: Omit<SensorReading, 'id' | '@context'>
  ): Promise<APIResponse<SensorReading>> {
    const response = await this.client.post<APIResponse<SensorReading>>(
      '/monitoring/sensors/readings',
      {
        '@context': 'https://wiastandards.com/contexts/uni-005/v1',
        ...reading,
      }
    );
    return response.data;
  }

  /**
   * Get asset status
   * @param assetId - Asset identifier
   * @returns Current status
   */
  async getAssetStatus(assetId: string): Promise<APIResponse<StatusUpdate>> {
    const response = await this.client.get<APIResponse<StatusUpdate>>(
      `/monitoring/assets/${assetId}/status`
    );
    return response.data;
  }

  // ==================== Maintenance ====================

  /**
   * Schedule maintenance
   * @param record - Maintenance schedule data
   * @returns Scheduled maintenance
   */
  async scheduleMaintenance(
    record: Partial<MaintenanceRecord>
  ): Promise<APIResponse<MaintenanceRecord>> {
    const response = await this.client.post<APIResponse<MaintenanceRecord>>(
      '/maintenance/schedules',
      record
    );
    return response.data;
  }

  /**
   * Record completed maintenance
   * @param record - Maintenance record data
   * @returns Created maintenance record
   */
  async recordMaintenance(
    record: Omit<MaintenanceRecord, 'id' | '@context'>
  ): Promise<APIResponse<MaintenanceRecord>> {
    const response = await this.client.post<APIResponse<MaintenanceRecord>>(
      '/maintenance/records',
      {
        '@context': 'https://wiastandards.com/contexts/uni-005/v1',
        ...record,
      }
    );
    return response.data;
  }

  /**
   * Get maintenance history for an asset
   * @param assetId - Asset identifier
   * @param params - Query parameters
   * @returns Maintenance history
   */
  async getMaintenanceHistory(
    assetId: string,
    params?: QueryParams
  ): Promise<PaginatedResponse<MaintenanceRecord>> {
    const response = await this.client.get<PaginatedResponse<MaintenanceRecord>>(
      `/maintenance/assets/${assetId}/history`,
      { params }
    );
    return response.data;
  }

  // ==================== Environmental Monitoring ====================

  /**
   * Submit environmental reading
   * @param reading - Environmental data
   * @returns Confirmation
   */
  async submitEnvironmentalReading(
    reading: Omit<EnvironmentalReading, 'id' | '@context'>
  ): Promise<APIResponse<EnvironmentalReading>> {
    const response = await this.client.post<APIResponse<EnvironmentalReading>>(
      '/environmental/readings',
      {
        '@context': 'https://wiastandards.com/contexts/uni-005/v1',
        ...reading,
      }
    );
    return response.data;
  }

  /**
   * Get environmental data for a location
   * @param stationId - Monitoring station identifier
   * @param params - Query parameters
   * @returns Environmental readings
   */
  async getEnvironmentalData(
    stationId: string,
    params?: QueryParams
  ): Promise<PaginatedResponse<EnvironmentalReading>> {
    const response = await this.client.get<PaginatedResponse<EnvironmentalReading>>(
      `/environmental/stations/${stationId}/readings`,
      { params }
    );
    return response.data;
  }

  // ==================== Webhooks ====================

  /**
   * Create webhook subscription
   * @param url - Webhook URL
   * @param events - Event types to subscribe to
   * @param secret - Webhook secret for signature verification
   * @returns Webhook subscription details
   */
  async createWebhook(
    url: string,
    events: string[],
    secret: string
  ): Promise<APIResponse<any>> {
    const response = await this.client.post<APIResponse<any>>('/webhooks', {
      url,
      events,
      secret,
    });
    return response.data;
  }

  /**
   * Delete webhook subscription
   * @param webhookId - Webhook identifier
   */
  async deleteWebhook(webhookId: string): Promise<void> {
    await this.client.delete(`/webhooks/${webhookId}`);
  }
}

// Export types
export * from './types';

// Export SDK as default
export default WIAInfrastructureSDK;
