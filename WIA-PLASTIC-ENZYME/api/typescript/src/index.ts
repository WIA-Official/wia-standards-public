/**
 * WIA-PLASTIC-ENZYME SDK
 *
 * Official SDK for enzymatic plastic degradation standard
 *
 * @version 1.0.0
 * @license MIT
 * @copyright 2025 WIA - World Certification Industry Association
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance, AxiosError } from 'axios';
import {
  ClientConfig,
  ApiError,
  EnzymeProfile,
  EnzymeLibraryParams,
  EnzymeLibraryResponse,
  PlasticInput,
  EnzymeMatchParams,
  EnzymeMatchResponse,
  DegradationProcess,
  PredictionResponse,
  OptimizationRequest,
  OptimizationResponse,
  QualityMetrics,
  PlasticIdentifyRequest,
  PlasticIdentifyResponse,
  PaginatedResponse,
} from './types';

// Re-export all types
export * from './types';

/**
 * Default API configuration
 */
const DEFAULT_CONFIG: Partial<ClientConfig> = {
  baseUrl: 'https://api.wiastandards.com/plastic-enzyme/v1',
  environment: 'production',
  timeout: 30000,
};

/**
 * WIA Plastic Enzyme SDK Client
 *
 * @example
 * ```typescript
 * import { WiaPlasticEnzyme } from '@wia/plastic-enzyme-sdk';
 *
 * const client = new WiaPlasticEnzyme({
 *   apiKey: 'your-api-key'
 * });
 *
 * // Match enzymes to plastic
 * const matches = await client.matchEnzymes({
 *   type: 'PET',
 *   weight_kg: 100,
 *   crystallinity_percent: 25
 * });
 * ```
 */
export class WiaPlasticEnzyme {
  private client: AxiosInstance;
  private config: ClientConfig;

  constructor(config: ClientConfig) {
    this.config = { ...DEFAULT_CONFIG, ...config } as ClientConfig;

    const baseURL =
      this.config.environment === 'sandbox'
        ? 'https://sandbox.api.wiastandards.com/plastic-enzyme/v1'
        : this.config.baseUrl;

    this.client = axios.create({
      baseURL,
      timeout: this.config.timeout,
      headers: {
        'Content-Type': 'application/json',
        'X-API-Key': this.config.apiKey,
        'X-WIA-Standard': 'PLASTIC-ENZYME',
        'X-WIA-Version': '1.0.0',
      },
    });

    // Response interceptor for error handling
    this.client.interceptors.response.use(
      (response) => response,
      (error: AxiosError<ApiError>) => {
        if (error.response?.data) {
          const apiError = error.response.data;
          throw new WiaApiError(
            apiError.message || 'Unknown error',
            apiError.code || 'UNKNOWN_ERROR',
            apiError.details,
            apiError.request_id
          );
        }
        throw error;
      }
    );
  }

  // ============================================================================
  // Enzyme Library Methods
  // ============================================================================

  /**
   * List enzymes from the library
   *
   * @param params - Query parameters for filtering
   * @returns Paginated list of enzymes
   *
   * @example
   * ```typescript
   * const enzymes = await client.listEnzymes({
   *   classification: 'PETase',
   *   min_temperature: 40,
   *   max_temperature: 60
   * });
   * ```
   */
  async listEnzymes(params?: EnzymeLibraryParams): Promise<EnzymeLibraryResponse> {
    const response = await this.client.get('/enzymes', { params });
    return response.data;
  }

  /**
   * Get detailed enzyme profile by ID
   *
   * @param enzymeId - The enzyme identifier
   * @returns Complete enzyme profile
   *
   * @example
   * ```typescript
   * const enzyme = await client.getEnzyme('petase-is');
   * console.log(enzyme.kinetics.kcat_km);
   * ```
   */
  async getEnzyme(enzymeId: string): Promise<EnzymeProfile> {
    const response = await this.client.get(`/enzymes/${enzymeId}`);
    return response.data;
  }

  /**
   * Search enzymes by criteria
   *
   * @param query - Search query string
   * @param params - Additional filter parameters
   * @returns Matching enzymes
   */
  async searchEnzymes(
    query: string,
    params?: EnzymeLibraryParams
  ): Promise<EnzymeLibraryResponse> {
    const response = await this.client.get('/enzymes/search', {
      params: { q: query, ...params },
    });
    return response.data;
  }

  // ============================================================================
  // Plastic Identification Methods
  // ============================================================================

  /**
   * Identify plastic type from spectral data or image
   *
   * @param request - Identification request with spectral or image data
   * @returns Plastic identification result with confidence
   *
   * @example
   * ```typescript
   * const result = await client.identifyPlastic({
   *   method: 'ftir_spectrum',
   *   data: {
   *     wavenumbers: [...],
   *     absorbance: [...]
   *   }
   * });
   * console.log(`Identified: ${result.plastic_type} (${result.confidence * 100}%)`);
   * ```
   */
  async identifyPlastic(request: PlasticIdentifyRequest): Promise<PlasticIdentifyResponse> {
    const response = await this.client.post('/plastic/identify', request);
    return response.data;
  }

  // ============================================================================
  // Enzyme Matching Methods
  // ============================================================================

  /**
   * Match optimal enzymes for a plastic input
   *
   * @param plastic - Plastic input specification
   * @param params - Optional matching parameters
   * @returns Ranked enzyme recommendations
   *
   * @example
   * ```typescript
   * const match = await client.matchEnzymes(
   *   { type: 'PET', weight_kg: 100, crystallinity_percent: 25 },
   *   { temperature: 50, ph: 8.0, target_efficiency: 0.95 }
   * );
   *
   * console.log('Top enzyme:', match.recommendations[0].name);
   * console.log('Cocktail:', match.cocktail_suggestion);
   * ```
   */
  async matchEnzymes(
    plastic: PlasticInput,
    params?: EnzymeMatchParams
  ): Promise<EnzymeMatchResponse> {
    const response = await this.client.post('/match', {
      plastic,
      ...params,
    });
    return response.data;
  }

  // ============================================================================
  // Degradation Prediction Methods
  // ============================================================================

  /**
   * Predict degradation outcomes for a process
   *
   * @param process - Degradation process specification
   * @returns Prediction with uncertainty estimates and time course
   *
   * @example
   * ```typescript
   * const prediction = await client.predictDegradation({
   *   process_id: 'test-001',
   *   plastic_input: { type: 'PET', weight_kg: 100 },
   *   enzyme_cocktail: [
   *     { enzyme_id: 'turbopetase-v2', concentration_mg_g: 3 },
   *     { enzyme_id: 'mhetase-is', concentration_mg_g: 1.5 }
   *   ],
   *   conditions: {
   *     temperature_c: 50,
   *     ph: 8.0,
   *     duration_hours: 48
   *   }
   * });
   *
   * console.log(`Expected degradation: ${prediction.predictions.degradation_percent.mean}%`);
   * ```
   */
  async predictDegradation(process: DegradationProcess): Promise<PredictionResponse> {
    const response = await this.client.post('/predict', process);
    return response.data;
  }

  /**
   * Get time course prediction for degradation
   *
   * @param process - Degradation process specification
   * @param intervalHours - Time interval for predictions (default: 1)
   * @returns Array of time-degradation points
   */
  async getTimeCourse(
    process: DegradationProcess,
    intervalHours: number = 1
  ): Promise<Array<{ hour: number; degradation: number }>> {
    const response = await this.client.post('/predict/timecourse', {
      ...process,
      interval_hours: intervalHours,
    });
    return response.data.time_course;
  }

  // ============================================================================
  // Optimization Methods
  // ============================================================================

  /**
   * Optimize degradation conditions
   *
   * @param request - Optimization request with constraints
   * @returns Optimal conditions and predicted outcomes
   *
   * @example
   * ```typescript
   * const optimized = await client.optimizeConditions({
   *   plastic: { type: 'PET', weight_kg: 1000, crystallinity_percent: 30 },
   *   constraints: {
   *     max_temperature_c: 60,
   *     max_time_hours: 48,
   *     target_efficiency: 0.95
   *   },
   *   optimization_goal: 'cost'
   * });
   *
   * console.log('Optimal temp:', optimized.optimal_conditions.temperature_c);
   * console.log('Enzyme cocktail:', optimized.enzyme_cocktail);
   * ```
   */
  async optimizeConditions(request: OptimizationRequest): Promise<OptimizationResponse> {
    const response = await this.client.post('/optimize', request);
    return response.data;
  }

  // ============================================================================
  // Process Management Methods
  // ============================================================================

  /**
   * Record a new degradation process
   *
   * @param process - Process data to record
   * @returns Created process with ID
   */
  async createProcess(
    process: Omit<DegradationProcess, 'process_id'>
  ): Promise<DegradationProcess> {
    const response = await this.client.post('/processes', process);
    return response.data;
  }

  /**
   * Get process by ID
   *
   * @param processId - The process identifier
   * @returns Process details
   */
  async getProcess(processId: string): Promise<DegradationProcess> {
    const response = await this.client.get(`/processes/${processId}`);
    return response.data;
  }

  /**
   * List processes with optional filtering
   *
   * @param params - Query parameters
   * @returns Paginated list of processes
   */
  async listProcesses(params?: {
    facility_id?: string;
    status?: string;
    from_date?: string;
    to_date?: string;
    page?: number;
    per_page?: number;
  }): Promise<PaginatedResponse<DegradationProcess>> {
    const response = await this.client.get('/processes', { params });
    return response.data;
  }

  /**
   * Update process output data
   *
   * @param processId - The process identifier
   * @param output - Output data to update
   * @returns Updated process
   */
  async updateProcessOutput(
    processId: string,
    output: DegradationProcess['output']
  ): Promise<DegradationProcess> {
    const response = await this.client.patch(`/processes/${processId}/output`, output);
    return response.data;
  }

  // ============================================================================
  // Quality Metrics Methods
  // ============================================================================

  /**
   * Submit quality metrics for a batch
   *
   * @param metrics - Quality metrics data
   * @returns Recorded quality metrics with certification status
   *
   * @example
   * ```typescript
   * const quality = await client.submitQualityMetrics({
   *   batch_id: 'BATCH-2025-001',
   *   measurement_date: '2025-01-15',
   *   monomer_analysis: {
   *     tpa: {
   *       concentration_mM: 850,
   *       purity_percent: 99.2,
   *       color_hazen: 15
   *     },
   *     eg: {
   *       concentration_mM: 420,
   *       purity_percent: 99.5
   *     }
   *   }
   * });
   * ```
   */
  async submitQualityMetrics(metrics: QualityMetrics): Promise<QualityMetrics> {
    const response = await this.client.post('/quality', metrics);
    return response.data;
  }

  /**
   * Get quality metrics for a batch
   *
   * @param batchId - The batch identifier
   * @returns Quality metrics
   */
  async getQualityMetrics(batchId: string): Promise<QualityMetrics> {
    const response = await this.client.get(`/quality/${batchId}`);
    return response.data;
  }

  /**
   * Check grade eligibility for a batch
   *
   * @param batchId - The batch identifier
   * @returns Grade qualification results
   */
  async checkGradeEligibility(
    batchId: string
  ): Promise<{
    food_contact_eligible: boolean;
    bottle_grade_eligible: boolean;
    textile_grade_eligible: boolean;
    issues?: string[];
  }> {
    const response = await this.client.get(`/quality/${batchId}/eligibility`);
    return response.data;
  }

  // ============================================================================
  // Certification Methods
  // ============================================================================

  /**
   * Request WIA certification for a batch
   *
   * @param batchId - The batch identifier
   * @param grade - Target certification grade
   * @returns Certification request status
   */
  async requestCertification(
    batchId: string,
    grade: 'food-contact' | 'bottle-grade' | 'textile-grade' | 'industrial'
  ): Promise<{
    request_id: string;
    status: 'pending' | 'approved' | 'rejected';
    certification_id?: string;
    issues?: string[];
  }> {
    const response = await this.client.post('/certifications/request', {
      batch_id: batchId,
      grade,
    });
    return response.data;
  }

  /**
   * Get certification status
   *
   * @param requestId - The certification request ID
   * @returns Certification details
   */
  async getCertificationStatus(requestId: string): Promise<{
    request_id: string;
    status: 'pending' | 'approved' | 'rejected';
    certification_id?: string;
    grade?: string;
    issued_at?: string;
    expires_at?: string;
  }> {
    const response = await this.client.get(`/certifications/${requestId}`);
    return response.data;
  }

  // ============================================================================
  // Utility Methods
  // ============================================================================

  /**
   * Calculate theoretical yield for a plastic input
   *
   * @param plastic - Plastic input specification
   * @returns Theoretical yields
   */
  async calculateTheoreticalYield(plastic: PlasticInput): Promise<{
    tpa_yield_kg: number;
    eg_yield_kg: number;
    theoretical_efficiency: number;
  }> {
    const response = await this.client.post('/utils/theoretical-yield', plastic);
    return response.data;
  }

  /**
   * Validate enzyme compatibility
   *
   * @param enzymes - Array of enzyme IDs
   * @param conditions - Reaction conditions
   * @returns Compatibility assessment
   */
  async validateCompatibility(
    enzymes: string[],
    conditions: { temperature_c: number; ph: number }
  ): Promise<{
    compatible: boolean;
    issues?: string[];
    recommendations?: string[];
  }> {
    const response = await this.client.post('/utils/compatibility', {
      enzymes,
      conditions,
    });
    return response.data;
  }

  /**
   * Get SDK and API version information
   *
   * @returns Version information
   */
  async getVersion(): Promise<{
    api_version: string;
    sdk_version: string;
    wia_standard_version: string;
  }> {
    const response = await this.client.get('/version');
    return {
      ...response.data,
      sdk_version: '1.0.0',
    };
  }

  /**
   * Health check for API connectivity
   *
   * @returns Health status
   */
  async healthCheck(): Promise<{ status: 'healthy' | 'degraded' | 'unhealthy' }> {
    const response = await this.client.get('/health');
    return response.data;
  }
}

/**
 * Custom error class for WIA API errors
 */
export class WiaApiError extends Error {
  constructor(
    message: string,
    public code: string,
    public details?: Record<string, unknown>,
    public requestId?: string
  ) {
    super(message);
    this.name = 'WiaApiError';
  }
}

/**
 * Create a new WIA Plastic Enzyme client
 *
 * @param config - Client configuration
 * @returns Configured client instance
 *
 * @example
 * ```typescript
 * import { createClient } from '@wia/plastic-enzyme-sdk';
 *
 * const client = createClient({
 *   apiKey: process.env.WIA_API_KEY!,
 *   environment: 'sandbox'
 * });
 * ```
 */
export function createClient(config: ClientConfig): WiaPlasticEnzyme {
  return new WiaPlasticEnzyme(config);
}

// Default export
export default WiaPlasticEnzyme;
