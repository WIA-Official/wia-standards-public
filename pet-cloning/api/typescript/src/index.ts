/**
 * WIA-PET-009 Pet Cloning Standard - TypeScript SDK
 * Version: 2.0.0
 * Philosophy: 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance, AxiosRequestConfig } from 'axios';
import { v4 as uuidv4 } from 'uuid';

import * as Types from './types';

export * from './types';

// ============================================================================
// Main SDK Client
// ============================================================================

export class PetCloningClient {
  private api: AxiosInstance;
  private config: Types.SDKConfig;

  constructor(config: Types.SDKConfig) {
    this.config = {
      baseURL: 'https://api.wia-pet-009.org/v2',
      timeout: 30000,
      retryAttempts: 3,
      debug: false,
      ...config,
    };

    this.api = axios.create({
      baseURL: this.config.baseURL,
      timeout: this.config.timeout,
      headers: {
        'Authorization': `Bearer ${this.config.apiKey}`,
        'Content-Type': 'application/json',
        'X-WIA-Standard': 'PET-009',
        'X-WIA-Version': '2.0',
      },
    });

    // Add request interceptor for debugging
    if (this.config.debug) {
      this.api.interceptors.request.use((config) => {
        console.log('[WIA-PET-009 SDK] Request:', config.method?.toUpperCase(), config.url);
        return config;
      });

      this.api.interceptors.response.use(
        (response) => {
          console.log('[WIA-PET-009 SDK] Response:', response.status, response.config.url);
          return response;
        },
        (error) => {
          console.error('[WIA-PET-009 SDK] Error:', error.message);
          return Promise.reject(error);
        }
      );
    }
  }

  // ============================================================================
  // Samples API
  // ============================================================================

  public samples = {
    /**
     * Create a new genetic sample record
     */
    create: async (sample: Omit<Types.GeneticSample, 'id'>): Promise<Types.APIResponse<Types.GeneticSample>> => {
      const response = await this.api.post('/samples', sample);
      return response.data;
    },

    /**
     * Get sample by ID
     */
    get: async (sampleId: string): Promise<Types.APIResponse<Types.GeneticSample>> => {
      const response = await this.api.get(`/samples/${sampleId}`);
      return response.data;
    },

    /**
     * List samples for a pet
     */
    listByPet: async (petId: string): Promise<Types.APIResponse<Types.GeneticSample[]>> => {
      const response = await this.api.get(`/samples?petId=${petId}`);
      return response.data;
    },

    /**
     * Update sample quality metrics
     */
    updateQuality: async (
      sampleId: string,
      metrics: Partial<Pick<Types.GeneticSample, 'viability' | 'dnaIntegrity' | 'chromosomeCount' | 'contaminationCheck'>>
    ): Promise<Types.APIResponse<Types.GeneticSample>> => {
      const response = await this.api.patch(`/samples/${sampleId}/quality`, metrics);
      return response.data;
    },

    /**
     * Update sample storage location
     */
    updateStorage: async (
      sampleId: string,
      storage: Partial<Pick<Types.GeneticSample, 'storageLocation' | 'storageTemperature' | 'cryopreservationDate'>>
    ): Promise<Types.APIResponse<Types.GeneticSample>> => {
      const response = await this.api.patch(`/samples/${sampleId}/storage`, storage);
      return response.data;
    },

    /**
     * Mark sample as used
     */
    markUsed: async (sampleId: string, usedFor: string): Promise<Types.APIResponse<Types.GeneticSample>> => {
      const response = await this.api.post(`/samples/${sampleId}/use`, { usedFor });
      return response.data;
    },
  };

  // ============================================================================
  // Pets API
  // ============================================================================

  public pets = {
    /**
     * Register a new pet profile
     */
    create: async (pet: Omit<Types.PetProfile, 'id'>): Promise<Types.APIResponse<Types.PetProfile>> => {
      const response = await this.api.post('/pets', pet);
      return response.data;
    },

    /**
     * Get pet by ID
     */
    get: async (petId: string): Promise<Types.APIResponse<Types.PetProfile>> => {
      const response = await this.api.get(`/pets/${petId}`);
      return response.data;
    },

    /**
     * Update pet profile
     */
    update: async (petId: string, updates: Partial<Types.PetProfile>): Promise<Types.APIResponse<Types.PetProfile>> => {
      const response = await this.api.patch(`/pets/${petId}`, updates);
      return response.data;
    },

    /**
     * Add health record
     */
    addHealthRecord: async (petId: string, record: Types.HealthRecord): Promise<Types.APIResponse<Types.PetProfile>> => {
      const response = await this.api.post(`/pets/${petId}/health`, record);
      return response.data;
    },
  };

  // ============================================================================
  // Cloning Requests API
  // ============================================================================

  public cloning = {
    /**
     * Submit a new cloning request
     */
    create: async (request: Omit<Types.CloningRequest, 'id' | 'status' | 'requestDate'>): Promise<Types.APIResponse<Types.CloningRequest>> => {
      const response = await this.api.post('/cloning/requests', request);
      return response.data;
    },

    /**
     * Get cloning request by ID
     */
    get: async (requestId: string): Promise<Types.APIResponse<Types.CloningRequest>> => {
      const response = await this.api.get(`/cloning/requests/${requestId}`);
      return response.data;
    },

    /**
     * Get cloning request status
     */
    getStatus: async (requestId: string): Promise<Types.APIResponse<{ status: Types.ProcedureStatus; progress: number }>> => {
      const response = await this.api.get(`/cloning/requests/${requestId}/status`);
      return response.data;
    },

    /**
     * List cloning requests for a pet
     */
    listByPet: async (petId: string): Promise<Types.APIResponse<Types.CloningRequest[]>> => {
      const response = await this.api.get(`/cloning/requests?petId=${petId}`);
      return response.data;
    },

    /**
     * Cancel a cloning request
     */
    cancel: async (requestId: string, reason: string): Promise<Types.APIResponse<Types.CloningRequest>> => {
      const response = await this.api.post(`/cloning/requests/${requestId}/cancel`, { reason });
      return response.data;
    },
  };

  // ============================================================================
  // SCNT Procedures API
  // ============================================================================

  public procedures = {
    /**
     * Record a new SCNT procedure
     */
    create: async (procedure: Omit<Types.SCNTProcedure, 'id'>): Promise<Types.APIResponse<Types.SCNTProcedure>> => {
      const response = await this.api.post('/procedures/scnt', procedure);
      return response.data;
    },

    /**
     * Get procedure by ID
     */
    get: async (procedureId: string): Promise<Types.APIResponse<Types.SCNTProcedure>> => {
      const response = await this.api.get(`/procedures/scnt/${procedureId}`);
      return response.data;
    },

    /**
     * Get procedures for a cloning request
     */
    listByRequest: async (requestId: string): Promise<Types.APIResponse<Types.SCNTProcedure[]>> => {
      const response = await this.api.get(`/procedures/scnt?requestId=${requestId}`);
      return response.data;
    },
  };

  // ============================================================================
  // Embryos API
  // ============================================================================

  public embryos = {
    /**
     * Create embryo records
     */
    createBatch: async (embryos: Omit<Types.Embryo, 'id'>[]): Promise<Types.APIResponse<Types.Embryo[]>> => {
      const response = await this.api.post('/embryos/batch', { embryos });
      return response.data;
    },

    /**
     * Get embryo by ID
     */
    get: async (embryoId: string): Promise<Types.APIResponse<Types.Embryo>> => {
      const response = await this.api.get(`/embryos/${embryoId}`);
      return response.data;
    },

    /**
     * Add development checkpoint
     */
    addCheckpoint: async (embryoId: string, checkpoint: Types.DevelopmentCheckpoint): Promise<Types.APIResponse<Types.Embryo>> => {
      const response = await this.api.post(`/embryos/${embryoId}/checkpoints`, checkpoint);
      return response.data;
    },

    /**
     * Update embryo stage
     */
    updateStage: async (embryoId: string, stage: Types.EmbryoStage): Promise<Types.APIResponse<Types.Embryo>> => {
      const response = await this.api.patch(`/embryos/${embryoId}/stage`, { stage });
      return response.data;
    },

    /**
     * Grade blastocyst
     */
    gradeBlastocyst: async (embryoId: string, grade: Types.BlastocystGrade): Promise<Types.APIResponse<Types.Embryo>> => {
      const response = await this.api.patch(`/embryos/${embryoId}/grade`, { grade });
      return response.data;
    },

    /**
     * List embryos by procedure
     */
    listByProcedure: async (procedureId: string): Promise<Types.APIResponse<Types.Embryo[]>> => {
      const response = await this.api.get(`/embryos?procedureId=${procedureId}`);
      return response.data;
    },
  };

  // ============================================================================
  // Embryo Transfer API
  // ============================================================================

  public transfers = {
    /**
     * Record embryo transfer
     */
    create: async (transfer: Omit<Types.EmbryoTransfer, 'id'>): Promise<Types.APIResponse<Types.EmbryoTransfer>> => {
      const response = await this.api.post('/transfers', transfer);
      return response.data;
    },

    /**
     * Get transfer by ID
     */
    get: async (transferId: string): Promise<Types.APIResponse<Types.EmbryoTransfer>> => {
      const response = await this.api.get(`/transfers/${transferId}`);
      return response.data;
    },

    /**
     * Update pregnancy status
     */
    updatePregnancy: async (transferId: string, established: boolean, pregnancyId?: string): Promise<Types.APIResponse<Types.EmbryoTransfer>> => {
      const response = await this.api.patch(`/transfers/${transferId}/pregnancy`, {
        pregnancyEstablished: established,
        pregnancyId,
      });
      return response.data;
    },
  };

  // ============================================================================
  // Pregnancy API
  // ============================================================================

  public pregnancies = {
    /**
     * Create pregnancy record
     */
    create: async (pregnancy: Omit<Types.Pregnancy, 'id' | 'status' | 'checkups'>): Promise<Types.APIResponse<Types.Pregnancy>> => {
      const response = await this.api.post('/pregnancies', pregnancy);
      return response.data;
    },

    /**
     * Get pregnancy by ID
     */
    get: async (pregnancyId: string): Promise<Types.APIResponse<Types.Pregnancy>> => {
      const response = await this.api.get(`/pregnancies/${pregnancyId}`);
      return response.data;
    },

    /**
     * Add pregnancy checkup
     */
    addCheckup: async (pregnancyId: string, checkup: Types.PregnancyCheckup): Promise<Types.APIResponse<Types.Pregnancy>> => {
      const response = await this.api.post(`/pregnancies/${pregnancyId}/checkups`, checkup);
      return response.data;
    },

    /**
     * Report complication
     */
    reportComplication: async (pregnancyId: string, complication: Types.PregnancyComplication): Promise<Types.APIResponse<Types.Pregnancy>> => {
      const response = await this.api.post(`/pregnancies/${pregnancyId}/complications`, complication);
      return response.data;
    },

    /**
     * Update pregnancy status
     */
    updateStatus: async (pregnancyId: string, status: Types.PregnancyStatus): Promise<Types.APIResponse<Types.Pregnancy>> => {
      const response = await this.api.patch(`/pregnancies/${pregnancyId}/status`, { status });
      return response.data;
    },
  };

  // ============================================================================
  // Clones API
  // ============================================================================

  public clones = {
    /**
     * Register newborn clone
     */
    create: async (clone: Omit<Types.Clone, 'id' | 'alive'>): Promise<Types.APIResponse<Types.Clone>> => {
      const response = await this.api.post('/clones', clone);
      return response.data;
    },

    /**
     * Get clone by ID
     */
    get: async (cloneId: string): Promise<Types.APIResponse<Types.Clone>> => {
      const response = await this.api.get(`/clones/${cloneId}`);
      return response.data;
    },

    /**
     * Add neonatal checkup
     */
    addCheckup: async (cloneId: string, checkup: Types.NeonatalCheckup): Promise<Types.APIResponse<Types.Clone>> => {
      const response = await this.api.post(`/clones/${cloneId}/checkups`, checkup);
      return response.data;
    },

    /**
     * Record milestone
     */
    addMilestone: async (cloneId: string, milestone: Types.DevelopmentMilestone): Promise<Types.APIResponse<Types.Clone>> => {
      const response = await this.api.post(`/clones/${cloneId}/milestones`, milestone);
      return response.data;
    },

    /**
     * Add health record
     */
    addHealthRecord: async (cloneId: string, record: Types.HealthRecord): Promise<Types.APIResponse<Types.Clone>> => {
      const response = await this.api.post(`/clones/${cloneId}/health`, record);
      return response.data;
    },

    /**
     * Get clones for original pet
     */
    listByOriginal: async (originalPetId: string): Promise<Types.APIResponse<Types.Clone[]>> => {
      const response = await this.api.get(`/clones?originalPetId=${originalPetId}`);
      return response.data;
    },
  };

  // ============================================================================
  // Algorithms API
  // ============================================================================

  public algorithms = {
    /**
     * Predict cloning success rate
     */
    predictSuccess: async (params: {
      sampleId?: string;
      viability?: number;
      dnaIntegrity?: number;
      passageNumber?: number;
      species?: Types.Species;
      technicianExperience?: number;
      labQuality?: number;
      oocyteQuality?: Types.OocyteGrade;
    }): Promise<Types.APIResponse<Types.SuccessPrediction>> => {
      const response = await this.api.post('/algorithms/predict-success', params);
      return response.data;
    },

    /**
     * Calculate blastocyst formation probability
     */
    predictBlastocyst: async (params: {
      fusionRate: number;
      cellViability: number;
      oocyteGrade: Types.OocyteGrade;
      species: Types.Species;
    }): Promise<Types.APIResponse<{ probability: number; confidence: number }>> => {
      const response = await this.api.post('/algorithms/predict-blastocyst', params);
      return response.data;
    },

    /**
     * Recommend optimal transfer timing
     */
    recommendTransferTiming: async (params: {
      embryoAge: number; // hours
      embryoStage: Types.EmbryoStage;
      embryoGrade?: Types.BlastocystGrade;
      surrogateDay: number; // day of cycle
      species: Types.Species;
    }): Promise<Types.APIResponse<{ recommendedHours: number; confidence: number; reasoning: string }>> => {
      const response = await this.api.post('/algorithms/transfer-timing', params);
      return response.data;
    },
  };

  // ============================================================================
  // Protocols API
  // ============================================================================

  public protocols = {
    /**
     * Get current protocol version
     */
    getCurrent: async (): Promise<Types.APIResponse<{ version: string; protocols: Record<string, any> }>> => {
      const response = await this.api.get('/protocols/current');
      return response.data;
    },

    /**
     * Get species-specific protocol
     */
    getBySpecies: async (species: Types.Species): Promise<Types.APIResponse<any>> => {
      const response = await this.api.get(`/protocols/species/${species}`);
      return response.data;
    },

    /**
     * Get procedure-specific protocol
     */
    getByProcedure: async (procedure: string): Promise<Types.APIResponse<any>> => {
      const response = await this.api.get(`/protocols/procedure/${procedure}`);
      return response.data;
    },
  };

  // ============================================================================
  // Quality Control API
  // ============================================================================

  public qualityControl = {
    /**
     * Submit QC result
     */
    submit: async (result: Omit<Types.QualityControlResult, 'id'>): Promise<Types.APIResponse<Types.QualityControlResult>> => {
      const response = await this.api.post('/qc/results', result);
      return response.data;
    },

    /**
     * Get QC results
     */
    list: async (filters?: {
      startDate?: Date;
      endDate?: Date;
      type?: string;
      passed?: boolean;
    }): Promise<Types.PaginatedResponse<Types.QualityControlResult>> => {
      const params = new URLSearchParams();
      if (filters?.startDate) params.append('startDate', filters.startDate.toISOString());
      if (filters?.endDate) params.append('endDate', filters.endDate.toISOString());
      if (filters?.type) params.append('type', filters.type);
      if (filters?.passed !== undefined) params.append('passed', filters.passed.toString());

      const response = await this.api.get(`/qc/results?${params.toString()}`);
      return response.data;
    },

    /**
     * Get QC statistics
     */
    getStatistics: async (period: 'week' | 'month' | 'quarter' | 'year'): Promise<Types.APIResponse<any>> => {
      const response = await this.api.get(`/qc/statistics?period=${period}`);
      return response.data;
    },
  };

  // ============================================================================
  // Utility Methods
  // ============================================================================

  /**
   * Check API health
   */
  public async healthCheck(): Promise<boolean> {
    try {
      const response = await this.api.get('/health');
      return response.status === 200;
    } catch {
      return false;
    }
  }

  /**
   * Get API version
   */
  public async getVersion(): Promise<string> {
    const response = await this.api.get('/version');
    return response.data.version;
  }

  /**
   * Test API connection with credentials
   */
  public async testConnection(): Promise<Types.APIResponse<{ message: string; authenticated: boolean }>> {
    const response = await this.api.get('/auth/test');
    return response.data;
  }
}

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * Calculate success rate based on multiple factors
 */
export function calculateSuccessRate(params: {
  viability: number;
  dnaIntegrity: number;
  passageNumber: number;
  species: Types.Species;
  experience: number;
  labQuality: number;
}): number {
  const baseRate = 0.75;
  const viabilityFactor = params.viability / 100;
  const integrityFactor = params.dnaIntegrity;
  const passageFactor = Math.max(0.5, 1 - params.passageNumber * 0.03);

  const speciesFactors: Record<Types.Species, number> = {
    [Types.Species.DOG]: 1.0,
    [Types.Species.CAT]: 0.85,
    [Types.Species.HORSE]: 0.70,
  };
  const speciesFactor = speciesFactors[params.species] || 0.8;

  const experienceFactor = Math.min(1.2, 0.8 + params.experience * 0.02);
  const labFactor = params.labQuality / 5;

  const successRate = baseRate * viabilityFactor * integrityFactor * passageFactor * speciesFactor * experienceFactor * labFactor;

  return Math.round(successRate * 100 * 100) / 100; // Round to 2 decimal places
}

/**
 * Validate genetic sample quality
 */
export function validateSampleQuality(sample: Partial<Types.GeneticSample>): { valid: boolean; issues: string[] } {
  const issues: string[] = [];

  if (sample.viability !== undefined && sample.viability < 95) {
    issues.push(`Cell viability below threshold: ${sample.viability}% (minimum 95%)`);
  }

  if (sample.dnaIntegrity !== undefined && sample.dnaIntegrity < 0.9) {
    issues.push(`DNA integrity below threshold: ${sample.dnaIntegrity} (minimum 0.90)`);
  }

  if (sample.passageNumber !== undefined && sample.passageNumber > 10) {
    issues.push(`Passage number too high: P${sample.passageNumber} (recommended <10)`);
  }

  if (sample.contaminationCheck && sample.contaminationCheck.percentage > 0.1) {
    issues.push(`Contamination detected: ${sample.contaminationCheck.percentage}% (maximum 0.1%)`);
  }

  return {
    valid: issues.length === 0,
    issues,
  };
}

/**
 * Format embryo age as human-readable string
 */
export function formatEmbryoAge(hoursPostActivation: number): string {
  if (hoursPostActivation < 24) {
    return `${hoursPostActivation} hours`;
  }

  const days = Math.floor(hoursPostActivation / 24);
  const hours = hoursPostActivation % 24;

  if (hours === 0) {
    return `${days} day${days !== 1 ? 's' : ''}`;
  }

  return `${days} day${days !== 1 ? 's' : ''}, ${hours} hour${hours !== 1 ? 's' : ''}`;
}

/**
 * Determine if embryo transfer timing is optimal
 */
export function isOptimalTransferTiming(embryoStage: Types.EmbryoStage, surrogateDay: number): boolean {
  // Blastocyst-stage embryos (day 6-7) should be transferred into day 5-7 surrogates
  const blastocystStages: Types.EmbryoStage[] = ['blastocyst', 'expanded_blastocyst', 'hatching_blastocyst'];

  if (blastocystStages.includes(embryoStage)) {
    return surrogateDay >= 5 && surrogateDay <= 7;
  }

  return false;
}

export default PetCloningClient;
