/**
 * WIA-PROTEIN-DYNAMICS SDK
 *
 * TypeScript SDK for protein dynamics standardization
 *
 * @version 1.0.0
 * @license MIT
 * @philosophy 弘益人間 (Benefit All Humanity)
 *
 * @example
 * ```typescript
 * import { WIAProteinDynamics } from '@wia/protein-dynamics';
 *
 * const client = new WIAProteinDynamics({
 *   apiKey: 'your-api-key'
 * });
 *
 * // Fetch protein dynamics profile
 * const profile = await client.getProteinDynamics('P00533');
 *
 * // Generate conformational ensemble
 * const job = await client.generateEnsemble({
 *   protein_id: 'P00533',
 *   method: 'alphaflow'
 * });
 * ```
 */

import type {
  ProteinDynamicsProfile,
  ConformationalEnsemble,
  DrugBindingProfile,
  AllostericNetwork,
  SimulationProtocol,
  ProteinDynamicsQuery,
  EnsembleGenerationRequest,
  BindingPredictionRequest,
  IdentifierMapping,
  SyncStatus,
  APIResponse,
  ProteinIdType,
  Ligand,
  BindingSite,
} from './types';

// Re-export all types
export * from './types';

// ============================================================================
// Configuration
// ============================================================================

/**
 * SDK configuration options
 */
export interface WIAClientConfig {
  /** API key for authentication */
  apiKey?: string;
  /** Base URL for API (default: https://api.wia-standards.org/protein-dynamics) */
  baseUrl?: string;
  /** Request timeout in milliseconds (default: 30000) */
  timeout?: number;
  /** Retry failed requests (default: true) */
  retry?: boolean;
  /** Maximum retry attempts (default: 3) */
  maxRetries?: number;
}

/**
 * Default configuration
 */
const DEFAULT_CONFIG: Required<WIAClientConfig> = {
  apiKey: '',
  baseUrl: 'https://api.wia-standards.org/protein-dynamics/v1',
  timeout: 30000,
  retry: true,
  maxRetries: 3,
};

// ============================================================================
// HTTP Client
// ============================================================================

/**
 * HTTP request options
 */
interface RequestOptions {
  method: 'GET' | 'POST' | 'PUT' | 'DELETE';
  path: string;
  body?: unknown;
  params?: Record<string, string | number | boolean | undefined>;
}

/**
 * Simple HTTP client for API requests
 */
class HttpClient {
  private config: Required<WIAClientConfig>;

  constructor(config: Required<WIAClientConfig>) {
    this.config = config;
  }

  async request<T>(options: RequestOptions): Promise<APIResponse<T>> {
    const url = this.buildUrl(options.path, options.params);
    const headers: Record<string, string> = {
      'Content-Type': 'application/json',
      'Accept': 'application/json',
    };

    if (this.config.apiKey) {
      headers['Authorization'] = `Bearer ${this.config.apiKey}`;
    }

    const fetchOptions: RequestInit = {
      method: options.method,
      headers,
      body: options.body ? JSON.stringify(options.body) : undefined,
    };

    let lastError: Error | null = null;
    const maxAttempts = this.config.retry ? this.config.maxRetries : 1;

    for (let attempt = 1; attempt <= maxAttempts; attempt++) {
      try {
        const controller = new AbortController();
        const timeoutId = setTimeout(() => controller.abort(), this.config.timeout);

        const response = await fetch(url, {
          ...fetchOptions,
          signal: controller.signal,
        });

        clearTimeout(timeoutId);

        const data = await response.json();

        if (!response.ok) {
          return {
            success: false,
            error: {
              code: `HTTP_${response.status}`,
              message: data.message || response.statusText,
              details: data,
            },
          };
        }

        return {
          success: true,
          data: data as T,
          pagination: data.pagination,
        };
      } catch (error) {
        lastError = error as Error;
        if (attempt < maxAttempts) {
          await this.delay(Math.pow(2, attempt) * 1000);
        }
      }
    }

    return {
      success: false,
      error: {
        code: 'NETWORK_ERROR',
        message: lastError?.message || 'Network request failed',
      },
    };
  }

  private buildUrl(path: string, params?: Record<string, string | number | boolean | undefined>): string {
    const url = new URL(`${this.config.baseUrl}${path}`);
    if (params) {
      Object.entries(params).forEach(([key, value]) => {
        if (value !== undefined) {
          url.searchParams.append(key, String(value));
        }
      });
    }
    return url.toString();
  }

  private delay(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}

// ============================================================================
// Main SDK Client
// ============================================================================

/**
 * WIA Protein Dynamics SDK Client
 *
 * Main client for interacting with the WIA-PROTEIN-DYNAMICS API
 */
export class WIAProteinDynamics {
  private http: HttpClient;
  private config: Required<WIAClientConfig>;

  constructor(config: WIAClientConfig = {}) {
    this.config = { ...DEFAULT_CONFIG, ...config };
    this.http = new HttpClient(this.config);
  }

  // ==========================================================================
  // Protein Dynamics Profile
  // ==========================================================================

  /**
   * Get protein dynamics profile by ID
   *
   * @param proteinId - UniProt, PDB, or AlphaFold ID
   * @param options - Query options
   * @returns Protein dynamics profile
   *
   * @example
   * ```typescript
   * const profile = await client.getProteinDynamics('P00533');
   * console.log(profile.conformational_ensemble?.states.length);
   * ```
   */
  async getProteinDynamics(
    proteinId: string,
    options?: {
      include_ensemble?: boolean;
      include_binding?: boolean;
      include_allosteric?: boolean;
    }
  ): Promise<APIResponse<ProteinDynamicsProfile>> {
    return this.http.request<ProteinDynamicsProfile>({
      method: 'GET',
      path: `/proteins/${proteinId}`,
      params: options,
    });
  }

  /**
   * Search protein dynamics profiles
   *
   * @param query - Search query parameters
   * @returns List of matching profiles
   */
  async searchProteins(
    query: ProteinDynamicsQuery
  ): Promise<APIResponse<ProteinDynamicsProfile[]>> {
    return this.http.request<ProteinDynamicsProfile[]>({
      method: 'GET',
      path: '/proteins',
      params: query as Record<string, string | number | boolean>,
    });
  }

  // ==========================================================================
  // Conformational Ensemble
  // ==========================================================================

  /**
   * Get conformational ensemble for a protein
   *
   * @param proteinId - Protein identifier
   * @returns Conformational ensemble data
   */
  async getEnsemble(proteinId: string): Promise<APIResponse<ConformationalEnsemble>> {
    return this.http.request<ConformationalEnsemble>({
      method: 'GET',
      path: `/proteins/${proteinId}/ensemble`,
    });
  }

  /**
   * Request ensemble generation (async job)
   *
   * @param request - Ensemble generation parameters
   * @returns Job information with status tracking URL
   */
  async generateEnsemble(
    request: EnsembleGenerationRequest
  ): Promise<APIResponse<{ job_id: string; status_url: string }>> {
    return this.http.request<{ job_id: string; status_url: string }>({
      method: 'POST',
      path: '/ensemble/generate',
      body: request,
    });
  }

  /**
   * Get ensemble generation job status
   *
   * @param jobId - Job identifier
   * @returns Job status and progress
   */
  async getJobStatus(
    jobId: string
  ): Promise<APIResponse<{
    job_id: string;
    status: 'queued' | 'running' | 'completed' | 'failed';
    progress?: number;
    result_url?: string;
    error?: string;
  }>> {
    return this.http.request({
      method: 'GET',
      path: `/jobs/${jobId}`,
    });
  }

  // ==========================================================================
  // Drug Binding
  // ==========================================================================

  /**
   * Get drug binding data for a protein
   *
   * @param proteinId - Protein identifier
   * @param ligandId - Optional ligand filter (ChEMBL ID)
   * @returns Drug binding profiles
   */
  async getBindingData(
    proteinId: string,
    ligandId?: string
  ): Promise<APIResponse<DrugBindingProfile[]>> {
    return this.http.request<DrugBindingProfile[]>({
      method: 'GET',
      path: `/proteins/${proteinId}/binding`,
      params: { ligand_id: ligandId },
    });
  }

  /**
   * Predict binding affinity for a ligand
   *
   * @param request - Binding prediction request
   * @returns Predicted binding profile
   */
  async predictBinding(
    request: BindingPredictionRequest
  ): Promise<APIResponse<DrugBindingProfile>> {
    return this.http.request<DrugBindingProfile>({
      method: 'POST',
      path: '/binding/predict',
      body: request,
    });
  }

  /**
   * Find binding sites (including cryptic sites)
   *
   * @param proteinId - Protein identifier
   * @param options - Search options
   * @returns List of binding sites
   */
  async findBindingSites(
    proteinId: string,
    options?: {
      include_cryptic?: boolean;
      min_druggability?: number;
    }
  ): Promise<APIResponse<BindingSite[]>> {
    return this.http.request<BindingSite[]>({
      method: 'GET',
      path: `/proteins/${proteinId}/binding-sites`,
      params: options,
    });
  }

  // ==========================================================================
  // Allosteric Network
  // ==========================================================================

  /**
   * Get allosteric network analysis
   *
   * @param proteinId - Protein identifier
   * @returns Allosteric network data
   */
  async getAllostericNetwork(proteinId: string): Promise<APIResponse<AllostericNetwork>> {
    return this.http.request<AllostericNetwork>({
      method: 'GET',
      path: `/proteins/${proteinId}/allosteric`,
    });
  }

  /**
   * Find communication pathways between residues
   *
   * @param proteinId - Protein identifier
   * @param sourceResidues - Source residue numbers
   * @param targetResidues - Target residue numbers
   * @returns Communication pathways
   */
  async findPathways(
    proteinId: string,
    sourceResidues: number[],
    targetResidues: number[]
  ): Promise<APIResponse<AllostericNetwork['pathways']>> {
    return this.http.request({
      method: 'POST',
      path: `/proteins/${proteinId}/allosteric/pathways`,
      body: { source_residues: sourceResidues, target_residues: targetResidues },
    });
  }

  // ==========================================================================
  // Simulation Protocols
  // ==========================================================================

  /**
   * List available simulation protocols
   *
   * @returns Available protocols
   */
  async listProtocols(): Promise<APIResponse<SimulationProtocol[]>> {
    return this.http.request<SimulationProtocol[]>({
      method: 'GET',
      path: '/protocols',
    });
  }

  /**
   * Get protocol details
   *
   * @param protocolId - Protocol identifier (e.g., 'WIA-PD-ENS-001')
   * @returns Protocol specification
   */
  async getProtocol(protocolId: string): Promise<APIResponse<SimulationProtocol>> {
    return this.http.request<SimulationProtocol>({
      method: 'GET',
      path: `/protocols/${protocolId}`,
    });
  }

  /**
   * Run a simulation protocol
   *
   * @param protocolId - Protocol to run
   * @param proteinId - Target protein
   * @param parameters - Optional parameter overrides
   * @returns Job information
   */
  async runProtocol(
    protocolId: string,
    proteinId: string,
    parameters?: Record<string, unknown>
  ): Promise<APIResponse<{ job_id: string; status_url: string }>> {
    return this.http.request({
      method: 'POST',
      path: `/protocols/${protocolId}/run`,
      body: { protein_id: proteinId, parameters },
    });
  }

  // ==========================================================================
  // Integration Services
  // ==========================================================================

  /**
   * Map identifiers across databases
   *
   * @param sourceId - Source identifier
   * @param sourceType - Source type (e.g., 'uniprot')
   * @param targetTypes - Target types to map to
   * @returns Identifier mappings
   */
  async mapIdentifiers(
    sourceId: string,
    sourceType: ProteinIdType,
    targetTypes: ProteinIdType[]
  ): Promise<APIResponse<IdentifierMapping>> {
    return this.http.request<IdentifierMapping>({
      method: 'GET',
      path: '/integration/map',
      params: {
        source_id: sourceId,
        source_type: sourceType,
        target_types: targetTypes.join(','),
      },
    });
  }

  /**
   * Check synchronization status with external databases
   *
   * @param proteinId - Protein identifier
   * @returns Sync status for each database
   */
  async checkSyncStatus(proteinId: string): Promise<APIResponse<SyncStatus>> {
    return this.http.request<SyncStatus>({
      method: 'GET',
      path: `/integration/sync-status/${proteinId}`,
    });
  }

  /**
   * Fetch data from PDB
   *
   * @param pdbId - PDB identifier
   * @returns Structure with dynamics hints from B-factors
   */
  async fetchFromPDB(pdbId: string): Promise<APIResponse<ProteinDynamicsProfile>> {
    return this.http.request<ProteinDynamicsProfile>({
      method: 'GET',
      path: `/integration/pdb/${pdbId}`,
    });
  }

  /**
   * Fetch data from AlphaFold DB
   *
   * @param uniprotId - UniProt identifier
   * @returns AlphaFold prediction with confidence-based flexibility
   */
  async fetchFromAlphaFold(uniprotId: string): Promise<APIResponse<ProteinDynamicsProfile>> {
    return this.http.request<ProteinDynamicsProfile>({
      method: 'GET',
      path: `/integration/alphafold/${uniprotId}`,
    });
  }

  /**
   * Search compounds from ChEMBL
   *
   * @param proteinId - Target protein UniProt ID
   * @param options - Search options
   * @returns List of compounds with binding data
   */
  async searchChEMBL(
    proteinId: string,
    options?: {
      activity_type?: 'IC50' | 'Ki' | 'Kd';
      max_value_nM?: number;
    }
  ): Promise<APIResponse<Ligand[]>> {
    return this.http.request<Ligand[]>({
      method: 'GET',
      path: `/integration/chembl/search`,
      params: { target: proteinId, ...options },
    });
  }

  // ==========================================================================
  // Utilities
  // ==========================================================================

  /**
   * Convert between file formats
   *
   * @param sourceFormat - Input format
   * @param targetFormat - Output format
   * @param data - Input data (base64 encoded for binary)
   * @returns Converted data
   */
  async convert(
    sourceFormat: 'pdb' | 'mmcif' | 'xtc' | 'wia',
    targetFormat: 'pdb' | 'mmcif' | 'wia',
    data: string
  ): Promise<APIResponse<{ data: string }>> {
    return this.http.request({
      method: 'POST',
      path: '/convert',
      body: { source_format: sourceFormat, target_format: targetFormat, data },
    });
  }

  /**
   * Validate WIA-PROTEIN-DYNAMICS JSON
   *
   * @param data - JSON data to validate
   * @returns Validation result with any errors
   */
  async validate(
    data: unknown
  ): Promise<APIResponse<{ valid: boolean; errors?: string[] }>> {
    return this.http.request({
      method: 'POST',
      path: '/validate',
      body: data,
    });
  }

  /**
   * Get API health status
   *
   * @returns API health information
   */
  async health(): Promise<APIResponse<{
    status: 'healthy' | 'degraded' | 'down';
    version: string;
    uptime_seconds: number;
  }>> {
    return this.http.request({
      method: 'GET',
      path: '/health',
    });
  }
}

// ============================================================================
// Factory Function
// ============================================================================

/**
 * Create a new WIA Protein Dynamics client
 *
 * @param config - Client configuration
 * @returns Configured client instance
 *
 * @example
 * ```typescript
 * const client = createClient({ apiKey: 'your-key' });
 * ```
 */
export function createClient(config?: WIAClientConfig): WIAProteinDynamics {
  return new WIAProteinDynamics(config);
}

// Default export
export default WIAProteinDynamics;
