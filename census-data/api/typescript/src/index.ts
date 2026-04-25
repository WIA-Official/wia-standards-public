/**
 * WIA-SOC-016 Census Data Standard - TypeScript SDK
 *
 * @packageDocumentation
 * @module @wia/census-data-sdk
 */

import axios, { AxiosInstance, AxiosError } from 'axios';
import type {
  CensusAPIConfig,
  CensusResponse,
  CensusError,
  PopulationStatistics,
  PopulationQueryParams,
  PaginatedResponse,
  Person,
  Household,
  Dwelling,
} from './types';

export * from './types';

/**
 * Default API configuration
 */
const DEFAULT_CONFIG = {
  baseURL: 'https://api.census.wia.org/v1',
  timeout: 30000,
  retry: true,
  maxRetries: 3,
};

/**
 * Main Census API client class
 *
 * @example
 * ```typescript
 * import { CensusAPI } from '@wia/census-data-sdk';
 *
 * const api = new CensusAPI({
 *   apiKey: 'your-api-key'
 * });
 *
 * const population = await api.getPopulation({
 *   geoCode: 'USA-CA',
 *   year: 2025
 * });
 * ```
 */
export class CensusAPI {
  private client: AxiosInstance;
  private config: Required<CensusAPIConfig>;

  /**
   * Creates a new Census API client
   *
   * @param config - API configuration options
   */
  constructor(config: CensusAPIConfig) {
    this.config = { ...DEFAULT_CONFIG, ...config };

    this.client = axios.create({
      baseURL: this.config.baseURL,
      timeout: this.config.timeout,
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${this.config.apiKey}`,
      },
    });

    // Add response interceptor for error handling
    this.client.interceptors.response.use(
      (response) => response,
      (error: AxiosError<CensusError>) => {
        if (this.config.retry && this.shouldRetry(error)) {
          return this.retryRequest(error);
        }
        return Promise.reject(this.handleError(error));
      }
    );
  }

  /**
   * Get population statistics for a geographic area
   *
   * @param params - Query parameters
   * @returns Population statistics
   *
   * @example
   * ```typescript
   * const stats = await api.getPopulation({
   *   geoCode: 'USA-CA',
   *   year: 2025,
   *   ageGroup: '25-34'
   * });
   * console.log(stats.data.total); // Total population
   * ```
   */
  async getPopulation(
    params: PopulationQueryParams = {}
  ): Promise<CensusResponse<PopulationStatistics>> {
    const { geoCode, ...queryParams } = params;
    const endpoint = geoCode
      ? `/population/${geoCode}`
      : '/population';

    const response = await this.client.get(endpoint, {
      params: queryParams,
    });

    return response.data;
  }

  /**
   * Get population statistics for a specific geographic area
   *
   * @param geoCode - Geographic code (e.g., 'USA-CA')
   * @param year - Census year (optional)
   * @returns Population statistics
   */
  async getPopulationByGeo(
    geoCode: string,
    year?: number
  ): Promise<CensusResponse<PopulationStatistics>> {
    return this.getPopulation({ geoCode, year });
  }

  /**
   * Get demographic breakdown data
   *
   * @param geoCode - Geographic code
   * @param params - Additional query parameters
   * @returns Demographic data
   */
  async getDemographics(
    geoCode?: string,
    params?: Record<string, any>
  ): Promise<CensusResponse<any>> {
    const endpoint = geoCode
      ? `/demographics/${geoCode}`
      : '/demographics';

    const response = await this.client.get(endpoint, { params });
    return response.data;
  }

  /**
   * Get housing statistics
   *
   * @param geoCode - Geographic code
   * @param params - Additional query parameters
   * @returns Housing data
   */
  async getHousing(
    geoCode?: string,
    params?: Record<string, any>
  ): Promise<CensusResponse<any>> {
    const endpoint = geoCode
      ? `/housing/${geoCode}`
      : '/housing';

    const response = await this.client.get(endpoint, { params });
    return response.data;
  }

  /**
   * Get economic indicators
   *
   * @param geoCode - Geographic code
   * @param params - Additional query parameters
   * @returns Economic data
   */
  async getEconomy(
    geoCode?: string,
    params?: Record<string, any>
  ): Promise<CensusResponse<any>> {
    const endpoint = geoCode
      ? `/economy/${geoCode}`
      : '/economy';

    const response = await this.client.get(endpoint, { params });
    return response.data;
  }

  /**
   * Get time series data for a variable
   *
   * @param variable - Variable name
   * @param params - Query parameters
   * @returns Time series data
   */
  async getTimeSeries(
    variable: string,
    params?: {
      geoCode?: string;
      startYear?: number;
      endYear?: number;
      frequency?: 'annual' | 'intercensal';
    }
  ): Promise<CensusResponse<any>> {
    const response = await this.client.get(`/timeseries/${variable}`, {
      params,
    });
    return response.data;
  }

  /**
   * Search for geographic areas
   *
   * @param query - Search query
   * @param params - Additional parameters
   * @returns List of matching geographic areas
   */
  async searchGeography(
    query: string,
    params?: { level?: string; limit?: number }
  ): Promise<PaginatedResponse<any>> {
    const response = await this.client.get('/geography/search', {
      params: { q: query, ...params },
    });
    return response.data;
  }

  /**
   * Get metadata for a variable
   *
   * @param variableId - Variable identifier
   * @returns Variable metadata
   */
  async getVariableMetadata(variableId: string): Promise<any> {
    const response = await this.client.get(`/metadata/variables/${variableId}`);
    return response.data;
  }

  /**
   * List available datasets
   *
   * @param params - Query parameters
   * @returns List of datasets
   */
  async listDatasets(params?: {
    year?: number;
    type?: string;
    page?: number;
    perPage?: number;
  }): Promise<PaginatedResponse<any>> {
    const response = await this.client.get('/datasets', { params });
    return response.data;
  }

  /**
   * Get data quality indicators
   *
   * @param geoCode - Geographic code
   * @param year - Census year
   * @returns Data quality metadata
   */
  async getDataQuality(geoCode: string, year: number): Promise<any> {
    const response = await this.client.get(`/quality/${geoCode}`, {
      params: { year },
    });
    return response.data;
  }

  /**
   * Custom query with flexible parameters
   *
   * @param endpoint - API endpoint
   * @param params - Query parameters
   * @returns API response
   */
  async query(
    endpoint: string,
    params?: Record<string, any>
  ): Promise<any> {
    const response = await this.client.get(endpoint, { params });
    return response.data;
  }

  /**
   * Check if an error should be retried
   */
  private shouldRetry(error: AxiosError): boolean {
    if (!error.response) {
      return true; // Network errors
    }

    const status = error.response.status;
    return status === 429 || status >= 500;
  }

  /**
   * Retry a failed request
   */
  private async retryRequest(
    error: AxiosError,
    retryCount = 0
  ): Promise<any> {
    if (retryCount >= this.config.maxRetries) {
      return Promise.reject(this.handleError(error));
    }

    // Exponential backoff
    const delay = Math.min(1000 * Math.pow(2, retryCount), 10000);
    await new Promise((resolve) => setTimeout(resolve, delay));

    try {
      const config = error.config;
      if (!config) {
        throw error;
      }
      return await this.client.request(config);
    } catch (retryError) {
      if (axios.isAxiosError(retryError)) {
        return this.retryRequest(retryError, retryCount + 1);
      }
      throw retryError;
    }
  }

  /**
   * Handle API errors
   */
  private handleError(error: AxiosError<CensusError>): Error {
    if (error.response?.data) {
      const censusError = error.response.data;
      const message = `Census API Error: ${censusError.message} (${censusError.code})`;
      const err = new Error(message);
      (err as any).code = censusError.code;
      (err as any).status = censusError.status;
      (err as any).details = censusError.details;
      return err;
    }

    return new Error(
      error.message || 'An unknown error occurred while accessing Census API'
    );
  }
}

/**
 * Create a new Census API client instance
 *
 * @param config - API configuration
 * @returns Census API client
 *
 * @example
 * ```typescript
 * const api = createClient({ apiKey: 'your-api-key' });
 * ```
 */
export function createClient(config: CensusAPIConfig): CensusAPI {
  return new CensusAPI(config);
}

/**
 * Default export
 */
export default CensusAPI;
