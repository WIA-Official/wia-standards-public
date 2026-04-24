/**
 * WIA-AGRI-028 Agricultural Data Exchange Standard - TypeScript SDK
 * @version 1.0.0
 * @standard WIA-AGRI-028
 * @license MIT
 */

import axios, { AxiosInstance } from 'axios';
import * as Types from './types';

export * from './types';

/**
 * Main SDK client for WIA-AGRI-028 Agricultural Data Exchange Standard
 */
export class WIAAgriDataExchangeClient {
  private client: AxiosInstance;
  private config: Types.ClientConfig;

  constructor(config: Types.ClientConfig) {
    this.config = config;
    this.client = axios.create({
      baseURL: config.baseURL,
      timeout: config.timeout || 30000,
      headers: {
        'Content-Type': 'application/json',
        'X-WIA-Standard': 'WIA-AGRI-028',
        'X-WIA-Version': '1.0.0',
        ...(config.apiKey && { Authorization: `Bearer ${config.apiKey}` }),
      },
    });
  }

  // ========================================================================
  // Data Source Management
  // ========================================================================

  /**
   * Register a new data source
   */
  async registerDataSource(source: Omit<Types.DataSource, 'sourceId' | 'createdAt' | 'updatedAt'>): Promise<Types.DataSource> {
    const response = await this.client.post('/api/v1/sources', source);
    return response.data;
  }

  /**
   * Get data source by ID
   */
  async getDataSource(sourceId: string): Promise<Types.DataSource> {
    const response = await this.client.get(`/api/v1/sources/${sourceId}`);
    return response.data;
  }

  /**
   * Update data source
   */
  async updateDataSource(sourceId: string, updates: Partial<Types.DataSource>): Promise<Types.DataSource> {
    const response = await this.client.put(`/api/v1/sources/${sourceId}`, updates);
    return response.data;
  }

  /**
   * Delete data source
   */
  async deleteDataSource(sourceId: string): Promise<void> {
    await this.client.delete(`/api/v1/sources/${sourceId}`);
  }

  /**
   * List data sources with filters
   */
  async listDataSources(filters?: {
    type?: Types.DataType;
    providerId?: string;
    region?: string;
    accessLevel?: Types.AccessLevel;
  }): Promise<Types.DataSource[]> {
    const response = await this.client.get('/api/v1/sources', { params: filters });
    return response.data;
  }

  /**
   * Search data sources
   */
  async searchDataSources(query: {
    keywords?: string[];
    dataTypes?: Types.DataType[];
    coverage?: Types.GeographicCoverage;
    minQuality?: Types.DataQuality;
  }): Promise<Types.DataSource[]> {
    const response = await this.client.post('/api/v1/sources/search', query);
    return response.data;
  }

  // ========================================================================
  // Data Exchange Management
  // ========================================================================

  /**
   * Create data exchange subscription
   */
  async createExchange(exchange: Omit<Types.DataExchange, 'exchangeId' | 'status' | 'statistics' | 'createdAt' | 'updatedAt'>): Promise<Types.DataExchange> {
    const response = await this.client.post('/api/v1/exchanges', exchange);
    return response.data;
  }

  /**
   * Get data exchange by ID
   */
  async getExchange(exchangeId: string): Promise<Types.DataExchange> {
    const response = await this.client.get(`/api/v1/exchanges/${exchangeId}`);
    return response.data;
  }

  /**
   * Update data exchange
   */
  async updateExchange(exchangeId: string, updates: Partial<Types.DataExchange>): Promise<Types.DataExchange> {
    const response = await this.client.put(`/api/v1/exchanges/${exchangeId}`, updates);
    return response.data;
  }

  /**
   * Cancel data exchange
   */
  async cancelExchange(exchangeId: string): Promise<void> {
    await this.client.post(`/api/v1/exchanges/${exchangeId}/cancel`);
  }

  /**
   * List exchanges for a consumer
   */
  async listExchanges(consumerId: string): Promise<Types.DataExchange[]> {
    const response = await this.client.get(`/api/v1/consumers/${consumerId}/exchanges`);
    return response.data;
  }

  /**
   * Get exchange statistics
   */
  async getExchangeStatistics(exchangeId: string): Promise<Types.ExchangeStatistics> {
    const response = await this.client.get(`/api/v1/exchanges/${exchangeId}/statistics`);
    return response.data;
  }

  // ========================================================================
  // Data Records
  // ========================================================================

  /**
   * Publish data record
   */
  async publishRecord(sourceId: string, record: Omit<Types.DataRecord, 'recordId'>): Promise<Types.DataRecord> {
    const response = await this.client.post(`/api/v1/sources/${sourceId}/records`, record);
    return response.data;
  }

  /**
   * Publish multiple records (batch)
   */
  async publishRecordsBatch(sourceId: string, records: Omit<Types.DataRecord, 'recordId'>[]): Promise<Types.DataRecord[]> {
    const response = await this.client.post(`/api/v1/sources/${sourceId}/records/batch`, { records });
    return response.data;
  }

  /**
   * Get data records
   */
  async getRecords(
    sourceId: string,
    options?: {
      startDate?: string;
      endDate?: string;
      limit?: number;
      offset?: number;
    }
  ): Promise<Types.DataRecord[]> {
    const response = await this.client.get(`/api/v1/sources/${sourceId}/records`, {
      params: options,
    });
    return response.data;
  }

  /**
   * Query data records with filters
   */
  async queryRecords(
    sourceId: string,
    filter: Types.DataFilter,
    options?: { limit?: number; offset?: number }
  ): Promise<Types.DataRecord[]> {
    const response = await this.client.post(`/api/v1/sources/${sourceId}/records/query`, {
      filter,
      ...options,
    });
    return response.data;
  }

  /**
   * Get latest record
   */
  async getLatestRecord(sourceId: string): Promise<Types.DataRecord> {
    const response = await this.client.get(`/api/v1/sources/${sourceId}/records/latest`);
    return response.data;
  }

  // ========================================================================
  // Weather Data
  // ========================================================================

  /**
   * Publish weather data
   */
  async publishWeatherData(sourceId: string, data: Types.WeatherData): Promise<void> {
    await this.client.post(`/api/v1/sources/${sourceId}/weather`, data);
  }

  /**
   * Get weather data
   */
  async getWeatherData(
    sourceId: string,
    location: Types.GeoCoordinates,
    options?: { startDate?: string; endDate?: string }
  ): Promise<Types.WeatherData[]> {
    const response = await this.client.get(`/api/v1/sources/${sourceId}/weather`, {
      params: { ...location, ...options },
    });
    return response.data;
  }

  /**
   * Get weather forecast
   */
  async getWeatherForecast(
    sourceId: string,
    location: Types.GeoCoordinates,
    days: number
  ): Promise<Types.WeatherForecast[]> {
    const response = await this.client.get(`/api/v1/sources/${sourceId}/weather/forecast`, {
      params: { ...location, days },
    });
    return response.data;
  }

  // ========================================================================
  // Soil Data
  // ========================================================================

  /**
   * Publish soil data
   */
  async publishSoilData(sourceId: string, data: Types.SoilData): Promise<void> {
    await this.client.post(`/api/v1/sources/${sourceId}/soil`, data);
  }

  /**
   * Get soil data
   */
  async getSoilData(
    sourceId: string,
    location: Types.GeoCoordinates,
    options?: { startDate?: string; endDate?: string }
  ): Promise<Types.SoilData[]> {
    const response = await this.client.get(`/api/v1/sources/${sourceId}/soil`, {
      params: { ...location, ...options },
    });
    return response.data;
  }

  /**
   * Get soil analysis
   */
  async getSoilAnalysis(sourceId: string, location: Types.GeoCoordinates): Promise<Types.SoilData> {
    const response = await this.client.get(`/api/v1/sources/${sourceId}/soil/analysis`, {
      params: location,
    });
    return response.data;
  }

  // ========================================================================
  // Crop Data
  // ========================================================================

  /**
   * Publish crop data
   */
  async publishCropData(sourceId: string, data: Types.CropData): Promise<void> {
    await this.client.post(`/api/v1/sources/${sourceId}/crop`, data);
  }

  /**
   * Get crop data
   */
  async getCropData(
    sourceId: string,
    options?: {
      cropType?: string;
      startDate?: string;
      endDate?: string;
    }
  ): Promise<Types.CropData[]> {
    const response = await this.client.get(`/api/v1/sources/${sourceId}/crop`, {
      params: options,
    });
    return response.data;
  }

  /**
   * Get crop health assessment
   */
  async getCropHealth(sourceId: string, location: Types.GeoCoordinates): Promise<Types.CropHealth> {
    const response = await this.client.get(`/api/v1/sources/${sourceId}/crop/health`, {
      params: location,
    });
    return response.data;
  }

  // ========================================================================
  // Market Data
  // ========================================================================

  /**
   * Publish market data
   */
  async publishMarketData(sourceId: string, data: Types.MarketData): Promise<void> {
    await this.client.post(`/api/v1/sources/${sourceId}/market`, data);
  }

  /**
   * Get market data
   */
  async getMarketData(
    sourceId: string,
    commodity: string,
    options?: { startDate?: string; endDate?: string }
  ): Promise<Types.MarketData[]> {
    const response = await this.client.get(`/api/v1/sources/${sourceId}/market`, {
      params: { commodity, ...options },
    });
    return response.data;
  }

  /**
   * Get price forecast
   */
  async getPriceForecast(
    sourceId: string,
    commodity: string,
    days: number
  ): Promise<Types.PriceForecast[]> {
    const response = await this.client.get(`/api/v1/sources/${sourceId}/market/forecast`, {
      params: { commodity, days },
    });
    return response.data;
  }

  // ========================================================================
  // Quality Management
  // ========================================================================

  /**
   * Assess data quality
   */
  async assessDataQuality(sourceId: string): Promise<Types.QualityMetrics> {
    const response = await this.client.post(`/api/v1/sources/${sourceId}/quality/assess`);
    return response.data;
  }

  /**
   * Get quality metrics
   */
  async getQualityMetrics(sourceId: string): Promise<Types.QualityMetrics> {
    const response = await this.client.get(`/api/v1/sources/${sourceId}/quality`);
    return response.data;
  }

  /**
   * Update quality metrics
   */
  async updateQualityMetrics(sourceId: string, metrics: Types.QualityMetrics): Promise<void> {
    await this.client.put(`/api/v1/sources/${sourceId}/quality`, metrics);
  }

  // ========================================================================
  // Schema Management
  // ========================================================================

  /**
   * Register data schema
   */
  async registerSchema(schema: Types.DataSchema): Promise<Types.DataSchema> {
    const response = await this.client.post('/api/v1/schemas', schema);
    return response.data;
  }

  /**
   * Get schema by ID
   */
  async getSchema(schemaId: string): Promise<Types.DataSchema> {
    const response = await this.client.get(`/api/v1/schemas/${schemaId}`);
    return response.data;
  }

  /**
   * Validate data against schema
   */
  async validateData(schemaId: string, data: any): Promise<{
    valid: boolean;
    errors?: string[];
  }> {
    const response = await this.client.post(`/api/v1/schemas/${schemaId}/validate`, { data });
    return response.data;
  }
}

/**
 * Factory function to create WIA Agricultural Data Exchange client
 */
export function createClient(config: Types.ClientConfig): WIAAgriDataExchangeClient {
  return new WIAAgriDataExchangeClient(config);
}

/**
 * Default export
 */
export default {
  createClient,
  WIAAgriDataExchangeClient,
};
