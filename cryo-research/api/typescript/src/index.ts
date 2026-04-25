/**
 * WIA-CRYO-010 TypeScript SDK
 * Version: 2.0.0
 *
 * 弘益人間 (Hongik Ingan) - Benefit All Humanity
 *
 * @packageDocumentation
 */

import axios, { AxiosInstance, AxiosError } from 'axios';
import {
  ExperimentData,
  ClinicalExperimentData,
  ClientConfig,
  ValidationResult,
  SubmitResponse,
  SearchQuery,
  SearchResult,
} from './types';

// Re-export all types
export * from './types';

/**
 * Main client for interacting with WIA-CRYO-010 API
 *
 * @example
 * ```typescript
 * const client = new CryoResearchClient({
 *   apiKey: process.env.WIA_API_KEY,
 *   endpoint: 'https://api.wia.org/cryo-010'
 * });
 *
 * const experiment: ExperimentData = {
 *   standard: 'WIA-CRYO-010',
 *   version: '2.0',
 *   experiment: {
 *     id: 'CRYO-2025-001',
 *     type: 'cell',
 *     title: 'MSC Cryopreservation Study',
 *     date: new Date().toISOString(),
 *     researcher: {
 *       name: 'Dr. Jane Smith',
 *       orcid: '0000-0001-2345-6789',
 *       institution: 'Research Institute',
 *       email: 'jane@institute.org'
 *     }
 *   },
 *   // ... rest of data
 * };
 *
 * const result = await client.submitExperiment(experiment);
 * console.log('Experiment ID:', result.experiment_id);
 * ```
 */
export class CryoResearchClient {
  private client: AxiosInstance;
  private config: Required<ClientConfig>;

  /**
   * Creates a new CryoResearchClient instance
   *
   * @param config - Client configuration
   */
  constructor(config: ClientConfig = {}) {
    this.config = {
      apiKey: config.apiKey || '',
      endpoint: config.endpoint || 'https://api.wia.org/cryo-010',
      version: config.version || '2.0',
      timeout: config.timeout || 30000,
      retries: config.retries || 3,
    };

    this.client = axios.create({
      baseURL: this.config.endpoint,
      timeout: this.config.timeout,
      headers: {
        'Content-Type': 'application/json',
        'X-WIA-Standard': 'CRYO-010',
        'X-WIA-Version': this.config.version,
        ...(this.config.apiKey && {
          Authorization: `Bearer ${this.config.apiKey}`,
        }),
      },
    });

    // Add retry logic
    this.setupRetryLogic();
  }

  /**
   * Setup automatic retry logic for failed requests
   */
  private setupRetryLogic(): void {
    this.client.interceptors.response.use(
      (response) => response,
      async (error: AxiosError) => {
        const config = error.config;
        if (!config || !config.headers) {
          return Promise.reject(error);
        }

        const retryCount = (config.headers['x-retry-count'] as number) || 0;

        if (retryCount < this.config.retries && this.isRetryableError(error)) {
          config.headers['x-retry-count'] = retryCount + 1;

          // Exponential backoff
          const delay = Math.pow(2, retryCount) * 1000;
          await new Promise((resolve) => setTimeout(resolve, delay));

          return this.client(config);
        }

        return Promise.reject(error);
      }
    );
  }

  /**
   * Determine if an error is retryable
   */
  private isRetryableError(error: AxiosError): boolean {
    if (!error.response) {
      return true; // Network errors are retryable
    }

    const status = error.response.status;
    return status === 429 || status >= 500; // Rate limit or server errors
  }

  /**
   * Validate experiment data against WIA-CRYO-010 schema
   *
   * @param data - Experiment data to validate
   * @returns Validation result
   */
  async validateExperiment(
    data: ExperimentData | ClinicalExperimentData
  ): Promise<ValidationResult> {
    try {
      const response = await this.client.post('/validate', data);
      return response.data;
    } catch (error) {
      if (axios.isAxiosError(error) && error.response) {
        return {
          valid: false,
          errors: [
            {
              field: 'general',
              message: error.response.data.message || 'Validation failed',
              severity: 'error',
            },
          ],
        };
      }
      throw error;
    }
  }

  /**
   * Submit a new experiment to the repository
   *
   * @param data - Experiment data
   * @returns Submit response with experiment ID and validation results
   */
  async submitExperiment(
    data: ExperimentData | ClinicalExperimentData
  ): Promise<SubmitResponse> {
    try {
      const response = await this.client.post('/experiments', data);
      return response.data;
    } catch (error) {
      if (axios.isAxiosError(error)) {
        throw new Error(
          `Failed to submit experiment: ${error.response?.data.message || error.message}`
        );
      }
      throw error;
    }
  }

  /**
   * Retrieve a specific experiment by ID
   *
   * @param id - Experiment ID (format: CRYO-YYYY-NNN)
   * @returns Experiment data
   */
  async getExperiment(
    id: string
  ): Promise<ExperimentData | ClinicalExperimentData> {
    try {
      const response = await this.client.get(`/experiments/${id}`);
      return response.data;
    } catch (error) {
      if (axios.isAxiosError(error)) {
        throw new Error(
          `Failed to retrieve experiment: ${error.response?.data.message || error.message}`
        );
      }
      throw error;
    }
  }

  /**
   * Search experiments with filters
   *
   * @param query - Search query parameters
   * @returns Search results
   */
  async searchExperiments(query: SearchQuery): Promise<SearchResult> {
    try {
      const response = await this.client.get('/experiments/search', {
        params: query,
      });
      return response.data;
    } catch (error) {
      if (axios.isAxiosError(error)) {
        throw new Error(
          `Failed to search experiments: ${error.response?.data.message || error.message}`
        );
      }
      throw error;
    }
  }

  /**
   * Update an existing experiment
   *
   * @param id - Experiment ID
   * @param data - Updated experiment data
   * @returns Update response
   */
  async updateExperiment(
    id: string,
    data: Partial<ExperimentData | ClinicalExperimentData>
  ): Promise<SubmitResponse> {
    try {
      const response = await this.client.put(`/experiments/${id}`, data);
      return response.data;
    } catch (error) {
      if (axios.isAxiosError(error)) {
        throw new Error(
          `Failed to update experiment: ${error.response?.data.message || error.message}`
        );
      }
      throw error;
    }
  }

  /**
   * Calculate viability metrics
   *
   * @param totalCells - Total cell count
   * @param viableCells - Viable cell count
   * @returns Viability percentage and related metrics
   */
  calculateViability(totalCells: number, viableCells: number): {
    viability_percentage: number;
    dead_cells: number;
    survival_rate: number;
  } {
    const deadCells = totalCells - viableCells;
    const viabilityPercentage = (viableCells / totalCells) * 100;
    const survivalRate = viabilityPercentage / 100;

    return {
      viability_percentage: Math.round(viabilityPercentage * 100) / 100,
      dead_cells: deadCells,
      survival_rate: Math.round(survivalRate * 10000) / 10000,
    };
  }

  /**
   * Validate experiment ID format
   *
   * @param id - Experiment ID to validate
   * @returns True if valid format
   */
  isValidExperimentId(id: string): boolean {
    const pattern = /^CRYO-\d{4}-\d{3}$/;
    return pattern.test(id);
  }

  /**
   * Validate ORCID format
   *
   * @param orcid - ORCID to validate
   * @returns True if valid format
   */
  isValidOrcid(orcid: string): boolean {
    const pattern = /^\d{4}-\d{4}-\d{4}-\d{3}[0-9X]$/;
    return pattern.test(orcid);
  }

  /**
   * Generate a new experiment ID
   *
   * @param year - Year (defaults to current year)
   * @param sequence - Sequence number (defaults to 1)
   * @returns Generated experiment ID
   */
  generateExperimentId(year?: number, sequence?: number): string {
    const currentYear = year || new Date().getFullYear();
    const seqNumber = sequence || 1;
    const paddedSeq = seqNumber.toString().padStart(3, '0');
    return `CRYO-${currentYear}-${paddedSeq}`;
  }

  /**
   * Validate cryoprotectant composition (must sum to 100%)
   *
   * @param components - Array of cryoprotectant components
   * @returns True if composition is valid
   */
  validateCryoprotectantComposition(
    components: Array<{ concentration: number }>
  ): { valid: boolean; total: number } {
    const total = components.reduce((sum, c) => sum + c.concentration, 0);
    const valid = Math.abs(total - 100) < 0.01; // Allow for floating point precision
    return { valid, total: Math.round(total * 100) / 100 };
  }

  /**
   * Get API health status
   *
   * @returns Health status
   */
  async getHealth(): Promise<{
    status: 'healthy' | 'degraded' | 'down';
    version: string;
    timestamp: string;
  }> {
    try {
      const response = await this.client.get('/health');
      return response.data;
    } catch (error) {
      return {
        status: 'down',
        version: this.config.version,
        timestamp: new Date().toISOString(),
      };
    }
  }
}

/**
 * Default export
 */
export default CryoResearchClient;

/**
 * Create a client instance with configuration
 *
 * @param config - Client configuration
 * @returns CryoResearchClient instance
 */
export function createClient(config: ClientConfig = {}): CryoResearchClient {
  return new CryoResearchClient(config);
}
