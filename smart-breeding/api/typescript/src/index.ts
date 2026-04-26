/**
 * WIA Smart Breeding Data Format Standard - TypeScript SDK
 * @version 1.0.0
 * @license MIT
 */

import {
  BreedingData,
  Individual,
  GenotypingData,
  PhenotypeData,
  BreedingValue,
  GeneticDiversity,
  ApiResponse,
  BreedingDataQuery,
} from './types';

/**
 * Configuration options for the Smart Breeding API client
 */
export interface SmartBreedingConfig {
  /**
   * Base URL for the API endpoint
   */
  baseUrl: string;

  /**
   * API key for authentication
   */
  apiKey?: string;

  /**
   * Request timeout in milliseconds
   * @default 30000
   */
  timeout?: number;
}

/**
 * Smart Breeding API Client
 *
 * Provides methods to interact with the WIA Smart Breeding data format standard,
 * enabling genomic selection, phenotyping, breeding value estimation, and genetic
 * diversity management.
 *
 * @example
 * ```typescript
 * const client = new SmartBreedingClient({
 *   baseUrl: 'https://api.example.com/breeding',
 *   apiKey: 'your-api-key'
 * });
 *
 * // Get individual breeding data
 * const individual = await client.getIndividual('BULL-2025-001');
 * console.log(individual.data);
 * ```
 */
export class SmartBreedingClient {
  private config: SmartBreedingConfig;
  private headers: Record<string, string>;

  /**
   * Creates a new Smart Breeding API client
   *
   * @param config - Configuration options
   */
  constructor(config: SmartBreedingConfig) {
    this.config = {
      timeout: 30000,
      ...config,
    };

    this.headers = {
      'Content-Type': 'application/json',
      'Accept': 'application/json',
    };

    if (config.apiKey) {
      this.headers['Authorization'] = `Bearer ${config.apiKey}`;
    }
  }

  /**
   * Makes an HTTP request to the API
   *
   * @param endpoint - API endpoint path
   * @param options - Fetch options
   * @returns Promise resolving to the response data
   */
  private async request<T>(
    endpoint: string,
    options: RequestInit = {}
  ): Promise<ApiResponse<T>> {
    const url = `${this.config.baseUrl}${endpoint}`;
    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), this.config.timeout);

    try {
      const response = await fetch(url, {
        ...options,
        headers: {
          ...this.headers,
          ...options.headers,
        },
        signal: controller.signal,
      });

      clearTimeout(timeoutId);

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }

      const data = await response.json();
      return {
        success: true,
        data,
        timestamp: new Date().toISOString(),
      };
    } catch (error) {
      clearTimeout(timeoutId);
      return {
        success: false,
        error: error instanceof Error ? error.message : 'Unknown error',
        timestamp: new Date().toISOString(),
      };
    }
  }

  // ==================== Individual Management ====================

  /**
   * Get breeding data for a specific individual
   *
   * @param individualId - Unique identifier for the individual
   * @returns Promise resolving to individual breeding data
   *
   * @example
   * ```typescript
   * const result = await client.getIndividual('BULL-2025-001');
   * if (result.success) {
   *   console.log(result.data);
   * }
   * ```
   */
  async getIndividual(individualId: string): Promise<ApiResponse<Individual>> {
    return this.request<Individual>(`/individuals/${individualId}`);
  }

  /**
   * Create a new individual breeding record
   *
   * @param individual - Individual data to create
   * @returns Promise resolving to created individual data
   */
  async createIndividual(individual: Individual): Promise<ApiResponse<Individual>> {
    return this.request<Individual>('/individuals', {
      method: 'POST',
      body: JSON.stringify(individual),
    });
  }

  /**
   * Update an existing individual breeding record
   *
   * @param individualId - Individual ID to update
   * @param individual - Updated individual data
   * @returns Promise resolving to updated individual data
   */
  async updateIndividual(
    individualId: string,
    individual: Partial<Individual>
  ): Promise<ApiResponse<Individual>> {
    return this.request<Individual>(`/individuals/${individualId}`, {
      method: 'PUT',
      body: JSON.stringify(individual),
    });
  }

  /**
   * Delete an individual breeding record
   *
   * @param individualId - Individual ID to delete
   * @returns Promise resolving to deletion confirmation
   */
  async deleteIndividual(individualId: string): Promise<ApiResponse<void>> {
    return this.request<void>(`/individuals/${individualId}`, {
      method: 'DELETE',
    });
  }

  /**
   * List individuals with optional filtering
   *
   * @param query - Query parameters for filtering
   * @returns Promise resolving to list of individuals
   */
  async listIndividuals(query?: BreedingDataQuery): Promise<ApiResponse<Individual[]>> {
    const params = new URLSearchParams();
    if (query) {
      Object.entries(query).forEach(([key, value]) => {
        if (value !== undefined) {
          params.append(key, String(value));
        }
      });
    }

    const endpoint = `/individuals${params.toString() ? `?${params.toString()}` : ''}`;
    return this.request<Individual[]>(endpoint);
  }

  // ==================== Genomic Data ====================

  /**
   * Get genomic data for an individual
   *
   * @param individualId - Individual ID
   * @returns Promise resolving to genotyping data
   */
  async getGenomicData(individualId: string): Promise<ApiResponse<GenotypingData>> {
    return this.request<GenotypingData>(`/genomic/${individualId}`);
  }

  /**
   * Upload genomic data for an individual
   *
   * @param data - Genotyping data to upload
   * @returns Promise resolving to uploaded data
   */
  async uploadGenomicData(data: GenotypingData): Promise<ApiResponse<GenotypingData>> {
    return this.request<GenotypingData>('/genomic', {
      method: 'POST',
      body: JSON.stringify(data),
    });
  }

  // ==================== Phenotype Data ====================

  /**
   * Get phenotype data for an individual
   *
   * @param individualId - Individual ID
   * @returns Promise resolving to phenotype data
   */
  async getPhenotypeData(individualId: string): Promise<ApiResponse<PhenotypeData>> {
    return this.request<PhenotypeData>(`/phenotype/${individualId}`);
  }

  /**
   * Add phenotype measurements for an individual
   *
   * @param data - Phenotype data to add
   * @returns Promise resolving to added data
   */
  async addPhenotypeData(data: PhenotypeData): Promise<ApiResponse<PhenotypeData>> {
    return this.request<PhenotypeData>('/phenotype', {
      method: 'POST',
      body: JSON.stringify(data),
    });
  }

  // ==================== Breeding Values ====================

  /**
   * Get breeding values for an individual
   *
   * @param individualId - Individual ID
   * @returns Promise resolving to breeding values
   */
  async getBreedingValues(individualId: string): Promise<ApiResponse<BreedingValue>> {
    return this.request<BreedingValue>(`/breeding-values/${individualId}`);
  }

  /**
   * Calculate breeding values for an individual
   *
   * @param individualId - Individual ID
   * @param method - Method to use (BLUP or GBLUP)
   * @returns Promise resolving to calculated breeding values
   */
  async calculateBreedingValues(
    individualId: string,
    method: 'BLUP' | 'GBLUP' = 'GBLUP'
  ): Promise<ApiResponse<BreedingValue>> {
    return this.request<BreedingValue>('/breeding-values/calculate', {
      method: 'POST',
      body: JSON.stringify({ individual_id: individualId, method }),
    });
  }

  // ==================== Genetic Diversity ====================

  /**
   * Get genetic diversity metrics for a population
   *
   * @param populationName - Population name
   * @returns Promise resolving to genetic diversity data
   */
  async getGeneticDiversity(populationName: string): Promise<ApiResponse<GeneticDiversity>> {
    return this.request<GeneticDiversity>(`/diversity/${populationName}`);
  }

  /**
   * Calculate genetic diversity for a population
   *
   * @param populationName - Population name
   * @param individualIds - List of individual IDs in the population
   * @returns Promise resolving to calculated diversity metrics
   */
  async calculateGeneticDiversity(
    populationName: string,
    individualIds: string[]
  ): Promise<ApiResponse<GeneticDiversity>> {
    return this.request<GeneticDiversity>('/diversity/calculate', {
      method: 'POST',
      body: JSON.stringify({ population_name: populationName, individual_ids: individualIds }),
    });
  }

  // ==================== Bulk Operations ====================

  /**
   * Export breeding data in JSON format
   *
   * @param query - Query parameters for filtering
   * @returns Promise resolving to breeding data
   */
  async exportData(query?: BreedingDataQuery): Promise<ApiResponse<BreedingData>> {
    const params = new URLSearchParams();
    if (query) {
      Object.entries(query).forEach(([key, value]) => {
        if (value !== undefined) {
          params.append(key, String(value));
        }
      });
    }

    const endpoint = `/export${params.toString() ? `?${params.toString()}` : ''}`;
    return this.request<BreedingData>(endpoint);
  }

  /**
   * Import breeding data from JSON
   *
   * @param data - Breeding data to import
   * @returns Promise resolving to import confirmation
   */
  async importData(data: BreedingData): Promise<ApiResponse<{ imported: number }>> {
    return this.request<{ imported: number }>('/import', {
      method: 'POST',
      body: JSON.stringify(data),
    });
  }
}

// Export all types
export * from './types';

// Export default client
export default SmartBreedingClient;
