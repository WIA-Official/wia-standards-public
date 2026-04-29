/**
 * WIA-NANOTECHNOLOGY TypeScript SDK
 *
 * Version: 1.0.0
 * Philosophy: 弘익人間 (Benefit All Humanity)
 *
 * Comprehensive SDK for nanomaterial data management, synthesis planning,
 * characterization analysis, simulation orchestration, and safety compliance.
 */

import axios, { AxiosInstance, AxiosError } from 'axios';
import { v4 as uuidv4 } from 'uuid';
import * as Types from './types';

// Re-export all types
export * from './types';

// ============================================================================
// Configuration
// ============================================================================

export interface WIANanoConfig {
  baseURL: string;
  apiKey?: string;
  timeout?: number;
  retryAttempts?: number;
}

const DEFAULT_CONFIG: Partial<WIANanoConfig> = {
  baseURL: 'https://api.wia-nanotechnology.org/v1',
  timeout: 30000,
  retryAttempts: 3
};

// ============================================================================
// Main SDK Class
// ============================================================================

export class WIANanotechnology {
  private client: AxiosInstance;
  private config: WIANanoConfig;

  constructor(config: Partial<WIANanoConfig> = {}) {
    this.config = { ...DEFAULT_CONFIG, ...config } as WIANanoConfig;

    this.client = axios.create({
      baseURL: this.config.baseURL,
      timeout: this.config.timeout,
      headers: {
        'Content-Type': 'application/json',
        ...(this.config.apiKey && { Authorization: `Bearer ${this.config.apiKey}` })
      }
    });

    // Response interceptor for error handling
    this.client.interceptors.response.use(
      response => response,
      error => this.handleError(error)
    );
  }

  // ==========================================================================
  // Error Handling
  // ==========================================================================

  private handleError(error: AxiosError): Promise<never> {
    if (error.response) {
      const apiError: Types.APIError = error.response.data as Types.APIError;
      throw new Error(`API Error (${apiError.code}): ${apiError.message}`);
    } else if (error.request) {
      throw new Error('No response from server');
    } else {
      throw new Error(`Request error: ${error.message}`);
    }
  }

  // ==========================================================================
  // Material Management
  // ==========================================================================

  /**
   * Create a new nanomaterial record
   */
  async createMaterial(
    material: Omit<Types.Material, 'materialId' | 'metadata'>
  ): Promise<Types.Material> {
    const materialId = uuidv4();
    const now = new Date().toISOString();

    const fullMaterial: Types.Material = {
      ...material,
      materialId,
      metadata: {
        createdAt: now,
        updatedAt: now,
        source: 'WIA-NANO SDK'
      }
    };

    const response = await this.client.post<Types.Material>('/materials', fullMaterial);
    return response.data;
  }

  /**
   * Get material by ID
   */
  async getMaterial(materialId: string): Promise<Types.Material> {
    const response = await this.client.get<Types.Material>(`/materials/${materialId}`);
    return response.data;
  }

  /**
   * Search materials with filters
   */
  async searchMaterials(
    filters: Types.SearchFilters,
    page: number = 1,
    limit: number = 20
  ): Promise<Types.SearchResults<Types.Material>> {
    const response = await this.client.get<Types.SearchResults<Types.Material>>(
      '/materials/search',
      {
        params: {
          ...filters,
          elements: filters.elements?.join(','),
          properties: filters.properties ? JSON.stringify(filters.properties) : undefined,
          page,
          limit
        }
      }
    );
    return response.data;
  }

  /**
   * Update material properties
   */
  async updateMaterial(
    materialId: string,
    updates: Partial<Omit<Types.Material, 'materialId' | 'metadata'>>
  ): Promise<Types.Material> {
    const response = await this.client.patch<Types.Material>(
      `/materials/${materialId}`,
      updates
    );
    return response.data;
  }

  /**
   * Delete material
   */
  async deleteMaterial(materialId: string): Promise<void> {
    await this.client.delete(`/materials/${materialId}`);
  }

  /**
   * Batch create materials
   */
  async batchCreateMaterials(
    materials: Array<Omit<Types.Material, 'materialId' | 'metadata'>>
  ): Promise<{ created: number; failed: number; results: any[] }> {
    const response = await this.client.post('/materials/batch', { materials });
    return response.data;
  }

  // ==========================================================================
  // Characterization Management
  // ==========================================================================

  /**
   * Upload characterization results
   */
  async uploadCharacterization(
    materialId: string,
    technique: Types.CharacterizationTechnique,
    results: any,
    metadata: {
      operator: string;
      instrument: Types.Instrument;
      conditions?: Types.CharacterizationConditions;
    }
  ): Promise<Types.Characterization> {
    const characterizationId = uuidv4();
    const timestamp = new Date().toISOString();

    const characterization: Types.Characterization = {
      characterizationId,
      materialId,
      technique,
      timestamp,
      operator: metadata.operator,
      instrument: metadata.instrument,
      conditions: metadata.conditions,
      results
    };

    const response = await this.client.post<Types.Characterization>(
      '/characterization',
      characterization
    );
    return response.data;
  }

  /**
   * Get characterization results by ID
   */
  async getCharacterization(characterizationId: string): Promise<Types.Characterization> {
    const response = await this.client.get<Types.Characterization>(
      `/characterization/${characterizationId}`
    );
    return response.data;
  }

  /**
   * List all characterizations for a material
   */
  async listCharacterizations(
    materialId: string,
    technique?: Types.CharacterizationTechnique,
    page: number = 1,
    limit: number = 20
  ): Promise<Types.SearchResults<Types.Characterization>> {
    const response = await this.client.get<Types.SearchResults<Types.Characterization>>(
      `/materials/${materialId}/characterizations`,
      {
        params: { technique, page, limit }
      }
    );
    return response.data;
  }

  /**
   * Analyze characterization data (e.g., particle size distribution)
   */
  async analyzeCharacterization(
    characterizationId: string,
    analysisType: 'particle_size' | 'phase_identification' | 'roughness' | 'composition',
    parameters?: Record<string, any>
  ): Promise<any> {
    const response = await this.client.post(
      `/characterization/${characterizationId}/analyze`,
      {
        analysisType,
        parameters
      }
    );
    return response.data;
  }

  // ==========================================================================
  // Synthesis Planning and Recording
  // ==========================================================================

  /**
   * Create a synthesis plan
   */
  async createSynthesisPlan(
    targetMaterialId: string,
    method: Types.SynthesisMethod,
    constraints?: {
      maxTemperature?: number;
      availablePrecursors?: string[];
      targetYield?: number;
    }
  ): Promise<any> {
    const response = await this.client.post('/synthesis/plan', {
      targetMaterialId,
      method,
      constraints
    });
    return response.data;
  }

  /**
   * Record synthesis execution
   */
  async recordSynthesis(
    synthesis: Omit<Types.SynthesisRecord, 'synthesisId' | 'timestamp'>
  ): Promise<Types.SynthesisRecord> {
    const synthesisId = uuidv4();
    const timestamp = new Date().toISOString();

    const fullSynthesis: Types.SynthesisRecord = {
      ...synthesis,
      synthesisId,
      timestamp
    };

    const response = await this.client.post<Types.SynthesisRecord>(
      '/synthesis/execute',
      fullSynthesis
    );
    return response.data;
  }

  /**
   * Get synthesis history for a material
   */
  async getSynthesisHistory(materialId: string): Promise<Types.SynthesisRecord[]> {
    const response = await this.client.get<{ syntheses: Types.SynthesisRecord[] }>(
      `/materials/${materialId}/synthesis`
    );
    return response.data.syntheses;
  }

  // ==========================================================================
  // Simulation Management
  // ==========================================================================

  /**
   * Submit a simulation job
   */
  async submitSimulation(
    type: Types.SimulationType,
    system: Types.SimulationSystem,
    parameters: Types.SimulationParameters,
    computeResources?: Types.ComputeResources
  ): Promise<Types.Simulation> {
    const simulationId = uuidv4();

    const simulation = {
      simulationId,
      type,
      system,
      parameters,
      computeResources
    };

    const response = await this.client.post<Types.Simulation>('/simulation', simulation);
    return response.data;
  }

  /**
   * Get simulation status
   */
  async getSimulationStatus(simulationId: string): Promise<Types.Simulation> {
    const response = await this.client.get<Types.Simulation>(`/simulation/${simulationId}`);
    return response.data;
  }

  /**
   * Get simulation results
   */
  async getSimulationResults(simulationId: string): Promise<Types.SimulationResults> {
    const response = await this.client.get<{ results: Types.SimulationResults }>(
      `/simulation/${simulationId}/results`
    );
    return response.data.results;
  }

  /**
   * Cancel a simulation
   */
  async cancelSimulation(simulationId: string): Promise<void> {
    await this.client.delete(`/simulation/${simulationId}`);
  }

  /**
   * Wait for simulation to complete (with polling)
   */
  async waitForSimulation(
    simulationId: string,
    pollInterval: number = 5000,
    maxWaitTime: number = 3600000
  ): Promise<Types.Simulation> {
    const startTime = Date.now();

    while (Date.now() - startTime < maxWaitTime) {
      const simulation = await this.getSimulationStatus(simulationId);

      if (simulation.status === Types.SimulationStatus.Completed) {
        return simulation;
      } else if (simulation.status === Types.SimulationStatus.Failed) {
        throw new Error('Simulation failed');
      } else if (simulation.status === Types.SimulationStatus.Cancelled) {
        throw new Error('Simulation was cancelled');
      }

      await new Promise(resolve => setTimeout(resolve, pollInterval));
    }

    throw new Error('Simulation timeout');
  }

  // ==========================================================================
  // Property Prediction
  // ==========================================================================

  /**
   * Predict material properties using ML or DFT
   */
  async predictProperties(
    materialId: string,
    properties: string[],
    method: Types.PredictionMethod = Types.PredictionMethod.ML
  ): Promise<Types.PredictionResults> {
    const response = await this.client.post<Types.PredictionResults>(
      '/properties/predict',
      {
        materialId,
        properties,
        method
      }
    );
    return response.data;
  }

  // ==========================================================================
  // Safety and Compliance
  // ==========================================================================

  /**
   * Get safety information for a material
   */
  async getSafetyInformation(materialId: string): Promise<Types.SafetyData> {
    const response = await this.client.get<Types.SafetyData>(
      `/materials/${materialId}/safety`
    );
    return response.data;
  }

  /**
   * Check regulatory compliance
   */
  async checkCompliance(
    materialId: string,
    jurisdiction: 'US' | 'EU' | 'ISO',
    regulations: string[]
  ): Promise<Types.ComplianceCheck> {
    const response = await this.client.post<Types.ComplianceCheck>(
      '/compliance/check',
      {
        materialId,
        jurisdiction,
        regulations
      }
    );
    return response.data;
  }

  // ==========================================================================
  // Utility Methods
  // ==========================================================================

  /**
   * Calculate particle size distribution from microscopy images
   */
  calculateParticleSizeDistribution(
    particles: Array<{ diameter: number }>
  ): Types.ParticleSizeDistribution {
    const diameters = particles.map(p => p.diameter).sort((a, b) => a - b);
    const n = diameters.length;

    const mean = diameters.reduce((sum, d) => sum + d, 0) / n;
    const median = n % 2 === 0
      ? (diameters[n / 2 - 1] + diameters[n / 2]) / 2
      : diameters[Math.floor(n / 2)];

    const variance = diameters.reduce((sum, d) => sum + Math.pow(d - mean, 2), 0) / n;
    const standardDeviation = Math.sqrt(variance);

    // Create histogram (10 bins)
    const min = Math.min(...diameters);
    const max = Math.max(...diameters);
    const binWidth = (max - min) / 10;
    const histogram: Types.HistogramBin[] = [];

    for (let i = 0; i < 10; i++) {
      const binCenter = min + (i + 0.5) * binWidth;
      const binMin = min + i * binWidth;
      const binMax = min + (i + 1) * binWidth;
      const count = diameters.filter(d => d >= binMin && d < binMax).length;
      histogram.push({ binCenter, count });
    }

    return {
      mean,
      median,
      standardDeviation,
      histogram
    };
  }

  /**
   * Calculate crystallite size using Scherrer equation
   */
  calculateCrystalliteSize(
    lambda: number,
    beta: number,
    theta: number,
    K: number = 0.9
  ): number {
    // Scherrer equation: D = Kλ / (β cos θ)
    // lambda in Angstrom, beta in radians, theta in radians
    const betaRad = (beta * Math.PI) / 180;
    const thetaRad = (theta * Math.PI) / 180;
    return (K * lambda) / (betaRad * Math.cos(thetaRad));
  }

  /**
   * Validate material composition (elements sum to 100%)
   */
  validateComposition(composition: Types.Composition): boolean {
    const totalPercentage = composition.elements.reduce(
      (sum, el) => sum + el.percentage,
      0
    );
    return Math.abs(totalPercentage - 100) < 0.01;
  }

  /**
   * Generate material formula from composition
   */
  generateFormula(elements: Types.Element[]): string {
    // Normalize to smallest integer ratios
    const percentages = elements.map(el => el.percentage);
    const minPercentage = Math.min(...percentages);
    const ratios = percentages.map(p => p / minPercentage);

    // Round to nearest integer (within tolerance)
    const integerRatios = ratios.map(r => {
      const rounded = Math.round(r);
      return Math.abs(r - rounded) < 0.1 ? rounded : r.toFixed(2);
    });

    // Build formula string
    return elements
      .map((el, i) => {
        const ratio = integerRatios[i];
        return ratio === 1 || ratio === '1' ? el.symbol : `${el.symbol}${ratio}`;
      })
      .join('');
  }

  /**
   * Convert between units
   */
  convertUnit(value: number, fromUnit: string, toUnit: string): number {
    const conversions: Record<string, Record<string, number>> = {
      nm: { μm: 0.001, mm: 0.000001, m: 1e-9 },
      μm: { nm: 1000, mm: 0.001, m: 1e-6 },
      mm: { nm: 1e6, μm: 1000, m: 0.001 },
      m: { nm: 1e9, μm: 1e6, mm: 1000 },
      C: { K: (c: number) => c + 273.15, F: (c: number) => (c * 9) / 5 + 32 },
      K: { C: (k: number) => k - 273.15, F: (k: number) => ((k - 273.15) * 9) / 5 + 32 },
      F: { C: (f: number) => ((f - 32) * 5) / 9, K: (f: number) => ((f - 32) * 5) / 9 + 273.15 }
    };

    if (fromUnit === toUnit) return value;

    if (conversions[fromUnit] && conversions[fromUnit][toUnit]) {
      const conversion = conversions[fromUnit][toUnit];
      return typeof conversion === 'function' ? conversion(value) : value * conversion;
    }

    throw new Error(`Conversion from ${fromUnit} to ${toUnit} not supported`);
  }

  /**
   * Estimate surface area from particle size (assuming spherical particles)
   */
  estimateSurfaceArea(
    diameter: number,
    density: number,
    unit: 'nm' | 'μm' = 'nm'
  ): number {
    // Convert to meters
    const diameterM = unit === 'nm' ? diameter * 1e-9 : diameter * 1e-6;

    // Surface area of sphere: 4πr²
    const radius = diameterM / 2;
    const surfaceAreaM2 = 4 * Math.PI * radius * radius;

    // Volume of sphere: (4/3)πr³
    const volumeM3 = (4 / 3) * Math.PI * Math.pow(radius, 3);

    // Mass in kg (density in g/cm³ = kg/L = 1000 kg/m³)
    const massKg = volumeM3 * density * 1000;

    // Surface area per gram (m²/g)
    return surfaceAreaM2 / (massKg * 1000);
  }

  /**
   * Generate synthesis recommendations based on material type
   */
  getSynthesisRecommendations(materialType: Types.MaterialType): {
    recommendedMethods: Types.SynthesisMethod[];
    considerations: string[];
  } {
    const recommendations: Record<
      Types.MaterialType,
      { recommendedMethods: Types.SynthesisMethod[]; considerations: string[] }
    > = {
      [Types.MaterialType.CNT]: {
        recommendedMethods: [
          Types.SynthesisMethod.CVD,
          Types.SynthesisMethod.PVD,
          Types.SynthesisMethod.Other
        ],
        considerations: [
          'CVD is most common for high-quality CNTs',
          'Requires metal catalyst (Fe, Ni, Co)',
          'Temperature range: 700-1000°C',
          'Carbon source: CH4, C2H2, or CO'
        ]
      },
      [Types.MaterialType.Graphene]: {
        recommendedMethods: [
          Types.SynthesisMethod.CVD,
          Types.SynthesisMethod.MBE,
          Types.SynthesisMethod.Lithography
        ],
        considerations: [
          'CVD on Cu foil is most common',
          'Requires transfer process for applications',
          'Temperature: 900-1100°C',
          'Can also use liquid-phase exfoliation'
        ]
      },
      [Types.MaterialType.QuantumDot]: {
        recommendedMethods: [
          Types.SynthesisMethod.SolGel,
          Types.SynthesisMethod.Hydrothermal,
          Types.SynthesisMethod.Coprecipitation
        ],
        considerations: [
          'Size control is critical for optical properties',
          'Surface passivation needed',
          'Quantum confinement effects below 10 nm',
          'Colloidal synthesis provides good size control'
        ]
      },
      [Types.MaterialType.Nanoparticle]: {
        recommendedMethods: [
          Types.SynthesisMethod.SolGel,
          Types.SynthesisMethod.Coprecipitation,
          Types.SynthesisMethod.Hydrothermal,
          Types.SynthesisMethod.BallMilling
        ],
        considerations: [
          'Method depends on material composition',
          'Surfactants can control size and shape',
          'Post-synthesis calcination may be needed',
          'Consider scalability for applications'
        ]
      },
      [Types.MaterialType.Nanocomposite]: {
        recommendedMethods: [
          Types.SynthesisMethod.SolGel,
          Types.SynthesisMethod.Other
        ],
        considerations: [
          'Mixing at nanoscale is challenging',
          'Interface compatibility is important',
          'May require multi-step synthesis',
          'Processing affects final properties'
        ]
      },
      [Types.MaterialType.Nanofilm]: {
        recommendedMethods: [
          Types.SynthesisMethod.PVD,
          Types.SynthesisMethod.CVD,
          Types.SynthesisMethod.SolGel
        ],
        considerations: [
          'Substrate preparation is critical',
          'Thickness control via deposition parameters',
          'Adhesion to substrate important',
          'May require annealing'
        ]
      },
      [Types.MaterialType.Nanowire]: {
        recommendedMethods: [
          Types.SynthesisMethod.CVD,
          Types.SynthesisMethod.Electrochemical,
          Types.SynthesisMethod.SelfAssembly
        ],
        considerations: [
          'VLS (Vapor-Liquid-Solid) mechanism for CVD',
          'Template-assisted growth for alignment',
          'Aspect ratio control is important',
          'Contact formation for devices'
        ]
      },
      [Types.MaterialType.Nanotube]: {
        recommendedMethods: [
          Types.SynthesisMethod.CVD,
          Types.SynthesisMethod.Hydrothermal,
          Types.SynthesisMethod.Other
        ],
        considerations: [
          'Similar to CNT for carbon-based',
          'Other materials may use hydrothermal',
          'Hollow structure requires templating or etching',
          'Chirality control challenging'
        ]
      },
      [Types.MaterialType.Fullerene]: {
        recommendedMethods: [Types.SynthesisMethod.PVD, Types.SynthesisMethod.Other],
        considerations: [
          'Arc discharge or laser ablation common',
          'Separation and purification needed',
          'C60 and C70 most common',
          'Functionalization expands applications'
        ]
      },
      [Types.MaterialType.Nanosheet]: {
        recommendedMethods: [
          Types.SynthesisMethod.Lithography,
          Types.SynthesisMethod.CVD,
          Types.SynthesisMethod.Other
        ],
        considerations: [
          'Exfoliation from layered materials',
          'CVD for 2D materials like MoS2',
          'Layer thickness control critical',
          'Stacking for heterostructures'
        ]
      }
    };

    return recommendations[materialType] || {
      recommendedMethods: [],
      considerations: ['No specific recommendations available']
    };
  }
}

// ============================================================================
// Convenience Functions
// ============================================================================

/**
 * Create a WIA Nanotechnology SDK instance
 */
export function createClient(config: Partial<WIANanoConfig> = {}): WIANanotechnology {
  return new WIANanotechnology(config);
}

/**
 * Quick material search
 */
export async function searchMaterials(
  filters: Types.SearchFilters,
  config?: Partial<WIANanoConfig>
): Promise<Types.Material[]> {
  const client = createClient(config);
  const results = await client.searchMaterials(filters);
  return results.items;
}

/**
 * Quick property prediction
 */
export async function predictProperties(
  materialId: string,
  properties: string[],
  config?: Partial<WIANanoConfig>
): Promise<Types.PredictionResults> {
  const client = createClient(config);
  return await client.predictProperties(materialId, properties);
}

// ============================================================================
// Default Export
// ============================================================================

export default WIANanotechnology;
