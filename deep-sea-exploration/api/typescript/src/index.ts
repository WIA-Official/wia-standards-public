/**
 * WIA-ENE-040: Deep Sea Exploration Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// Export all types
export * from './types';

import {
  Expedition,
  DiveLog,
  OrganismObservation,
  BiologicalSample,
  HydrothermalVent,
  RockSample,
  SedimentCore,
  MultibeamSurvey,
  EnvironmentalDNASample,
  IsobaricSampler,
  HOVSpecification,
  ROVSpecification,
  AUVSpecification,
  LanderSpecification,
  ApiResponse,
  PaginatedResponse,
  PaginationParams,
  DateRangeFilter,
  DepthRangeFilter,
  GeographicFilter,
  SubmersibleType,
  DepthZone,
  HabitatType,
} from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface DeepSeaExplorationSDKConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  retries?: number;
  debug?: boolean;
}

// ============================================================================
// SDK Client
// ============================================================================

export class DeepSeaExplorationSDK {
  private config: Required<DeepSeaExplorationSDKConfig>;
  private headers: Record<string, string>;

  constructor(config: DeepSeaExplorationSDKConfig) {
    this.config = {
      timeout: 30000,
      retries: 3,
      debug: false,
      ...config,
    };

    this.headers = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'X-WIA-Standard': 'ENE-040',
      'X-WIA-Version': '1.0.0',
    };
  }

  // ==========================================================================
  // Expedition Management APIs
  // ==========================================================================

  /**
   * List expeditions
   */
  async listExpeditions(
    params?: PaginationParams & { region?: string; year?: number }
  ): Promise<ApiResponse<PaginatedResponse<Expedition>>> {
    const queryParams = new URLSearchParams();
    if (params) {
      Object.entries(params).forEach(([key, value]) => {
        if (value !== undefined) {
          queryParams.append(key, String(value));
        }
      });
    }
    return this.get<PaginatedResponse<Expedition>>(
      `/api/v1/expeditions?${queryParams.toString()}`
    );
  }

  /**
   * Get expedition details
   */
  async getExpedition(expeditionId: string): Promise<ApiResponse<Expedition>> {
    return this.get<Expedition>(`/api/v1/expeditions/${expeditionId}`);
  }

  /**
   * Create new expedition
   */
  async createExpedition(
    expedition: Omit<Expedition, 'expeditionId' | 'metadata'>
  ): Promise<ApiResponse<Expedition>> {
    return this.post<Expedition>('/api/v1/expeditions', expedition);
  }

  /**
   * Update expedition
   */
  async updateExpedition(
    expeditionId: string,
    updates: Partial<Expedition>
  ): Promise<ApiResponse<Expedition>> {
    return this.put<Expedition>(`/api/v1/expeditions/${expeditionId}`, updates);
  }

  /**
   * Delete expedition
   */
  async deleteExpedition(expeditionId: string): Promise<ApiResponse<void>> {
    return this.delete<void>(`/api/v1/expeditions/${expeditionId}`);
  }

  // ==========================================================================
  // Dive Operations APIs
  // ==========================================================================

  /**
   * List dives for an expedition
   */
  async listDives(
    expeditionId: string,
    params?: PaginationParams
  ): Promise<ApiResponse<PaginatedResponse<DiveLog>>> {
    const queryParams = new URLSearchParams();
    if (params) {
      Object.entries(params).forEach(([key, value]) => {
        if (value !== undefined) {
          queryParams.append(key, String(value));
        }
      });
    }
    return this.get<PaginatedResponse<DiveLog>>(
      `/api/v1/expeditions/${expeditionId}/dives?${queryParams.toString()}`
    );
  }

  /**
   * Get dive details
   */
  async getDive(diveId: string): Promise<ApiResponse<DiveLog>> {
    return this.get<DiveLog>(`/api/v1/dives/${diveId}`);
  }

  /**
   * Create dive log
   */
  async createDive(
    dive: Omit<DiveLog, 'diveId'>
  ): Promise<ApiResponse<DiveLog>> {
    return this.post<DiveLog>('/api/v1/dives', dive);
  }

  /**
   * Update dive log
   */
  async updateDive(
    diveId: string,
    updates: Partial<DiveLog>
  ): Promise<ApiResponse<DiveLog>> {
    return this.put<DiveLog>(`/api/v1/dives/${diveId}`, updates);
  }

  /**
   * Get dive track (navigation)
   */
  async getDiveTrack(diveId: string): Promise<ApiResponse<any>> {
    return this.get<any>(`/api/v1/dives/${diveId}/track`);
  }

  // ==========================================================================
  // Biological Observation APIs
  // ==========================================================================

  /**
   * List organism observations
   */
  async listObservations(
    filters?: {
      diveId?: string;
      depthZone?: DepthZone;
      habitat?: HabitatType;
      taxon?: string;
      minDepth?: number;
      maxDepth?: number;
    } & PaginationParams
  ): Promise<ApiResponse<PaginatedResponse<OrganismObservation>>> {
    const queryParams = new URLSearchParams();
    if (filters) {
      Object.entries(filters).forEach(([key, value]) => {
        if (value !== undefined) {
          queryParams.append(key, String(value));
        }
      });
    }
    return this.get<PaginatedResponse<OrganismObservation>>(
      `/api/v1/observations?${queryParams.toString()}`
    );
  }

  /**
   * Get observation details
   */
  async getObservation(observationId: string): Promise<ApiResponse<OrganismObservation>> {
    return this.get<OrganismObservation>(`/api/v1/observations/${observationId}`);
  }

  /**
   * Submit organism observation
   */
  async submitObservation(
    observation: Omit<OrganismObservation, 'observationId'>
  ): Promise<ApiResponse<OrganismObservation>> {
    return this.post<OrganismObservation>('/api/v1/observations', observation);
  }

  /**
   * Update observation
   */
  async updateObservation(
    observationId: string,
    updates: Partial<OrganismObservation>
  ): Promise<ApiResponse<OrganismObservation>> {
    return this.put<OrganismObservation>(
      `/api/v1/observations/${observationId}`,
      updates
    );
  }

  /**
   * Get observations for a dive
   */
  async getDiveObservations(diveId: string): Promise<ApiResponse<OrganismObservation[]>> {
    return this.get<OrganismObservation[]>(`/api/v1/dives/${diveId}/observations`);
  }

  // ==========================================================================
  // Sample Management APIs
  // ==========================================================================

  /**
   * List biological samples
   */
  async listSamples(
    filters?: {
      expeditionId?: string;
      sampleType?: string;
      availability?: string;
    } & PaginationParams
  ): Promise<ApiResponse<PaginatedResponse<BiologicalSample>>> {
    const queryParams = new URLSearchParams();
    if (filters) {
      Object.entries(filters).forEach(([key, value]) => {
        if (value !== undefined) {
          queryParams.append(key, String(value));
        }
      });
    }
    return this.get<PaginatedResponse<BiologicalSample>>(
      `/api/v1/samples?${queryParams.toString()}`
    );
  }

  /**
   * Get sample details
   */
  async getSample(sampleId: string): Promise<ApiResponse<BiologicalSample>> {
    return this.get<BiologicalSample>(`/api/v1/samples/${sampleId}`);
  }

  /**
   * Register sample
   */
  async registerSample(
    sample: Omit<BiologicalSample, 'sampleId'>
  ): Promise<ApiResponse<BiologicalSample>> {
    return this.post<BiologicalSample>('/api/v1/samples', sample);
  }

  /**
   * Update sample
   */
  async updateSample(
    sampleId: string,
    updates: Partial<BiologicalSample>
  ): Promise<ApiResponse<BiologicalSample>> {
    return this.put<BiologicalSample>(`/api/v1/samples/${sampleId}`, updates);
  }

  /**
   * Request sample access
   */
  async requestSample(
    sampleId: string,
    request: {
      requester: string;
      institution: string;
      purpose: string;
    }
  ): Promise<ApiResponse<any>> {
    return this.post<any>(`/api/v1/samples/${sampleId}/request`, request);
  }

  /**
   * Get available samples
   */
  async getAvailableSamples(
    filters?: PaginationParams
  ): Promise<ApiResponse<PaginatedResponse<BiologicalSample>>> {
    const queryParams = new URLSearchParams({ availability: 'available' });
    if (filters) {
      Object.entries(filters).forEach(([key, value]) => {
        if (value !== undefined) {
          queryParams.append(key, String(value));
        }
      });
    }
    return this.get<PaginatedResponse<BiologicalSample>>(
      `/api/v1/samples/available?${queryParams.toString()}`
    );
  }

  // ==========================================================================
  // Hydrothermal Vent APIs
  // ==========================================================================

  /**
   * List hydrothermal vents
   */
  async listVents(
    filters?: {
      region?: string;
      ventType?: string;
      minTemp?: number;
      maxTemp?: number;
    } & PaginationParams
  ): Promise<ApiResponse<PaginatedResponse<HydrothermalVent>>> {
    const queryParams = new URLSearchParams();
    if (filters) {
      Object.entries(filters).forEach(([key, value]) => {
        if (value !== undefined) {
          queryParams.append(key, String(value));
        }
      });
    }
    return this.get<PaginatedResponse<HydrothermalVent>>(
      `/api/v1/vents?${queryParams.toString()}`
    );
  }

  /**
   * Get vent details
   */
  async getVent(ventId: string): Promise<ApiResponse<HydrothermalVent>> {
    return this.get<HydrothermalVent>(`/api/v1/vents/${ventId}`);
  }

  /**
   * Register hydrothermal vent
   */
  async registerVent(
    vent: Omit<HydrothermalVent, 'ventId'>
  ): Promise<ApiResponse<HydrothermalVent>> {
    return this.post<HydrothermalVent>('/api/v1/vents', vent);
  }

  /**
   * Update vent information
   */
  async updateVent(
    ventId: string,
    updates: Partial<HydrothermalVent>
  ): Promise<ApiResponse<HydrothermalVent>> {
    return this.put<HydrothermalVent>(`/api/v1/vents/${ventId}`, updates);
  }

  // ==========================================================================
  // Geological Sample APIs
  // ==========================================================================

  /**
   * Register rock sample
   */
  async registerRockSample(
    sample: Omit<RockSample, 'sampleId'>
  ): Promise<ApiResponse<RockSample>> {
    return this.post<RockSample>('/api/v1/geological/rocks', sample);
  }

  /**
   * Get rock sample
   */
  async getRockSample(sampleId: string): Promise<ApiResponse<RockSample>> {
    return this.get<RockSample>(`/api/v1/geological/rocks/${sampleId}`);
  }

  /**
   * Register sediment core
   */
  async registerSedimentCore(
    core: Omit<SedimentCore, 'coreId'>
  ): Promise<ApiResponse<SedimentCore>> {
    return this.post<SedimentCore>('/api/v1/geological/cores', core);
  }

  /**
   * Get sediment core
   */
  async getSedimentCore(coreId: string): Promise<ApiResponse<SedimentCore>> {
    return this.get<SedimentCore>(`/api/v1/geological/cores/${coreId}`);
  }

  // ==========================================================================
  // Bathymetry APIs
  // ==========================================================================

  /**
   * Upload multibeam survey data
   */
  async uploadBathymetry(
    survey: Omit<MultibeamSurvey, 'surveyId'>
  ): Promise<ApiResponse<MultibeamSurvey>> {
    return this.post<MultibeamSurvey>('/api/v1/bathymetry/upload', survey);
  }

  /**
   * Get bathymetry survey
   */
  async getBathymetrySurvey(surveyId: string): Promise<ApiResponse<MultibeamSurvey>> {
    return this.get<MultibeamSurvey>(`/api/v1/bathymetry/${surveyId}`);
  }

  /**
   * Get bathymetry grid data
   */
  async getBathymetryGrid(
    bbox: { minLon: number; minLat: number; maxLon: number; maxLat: number },
    resolution: string
  ): Promise<ApiResponse<any>> {
    const queryParams = new URLSearchParams({
      bbox: `${bbox.minLon},${bbox.minLat},${bbox.maxLon},${bbox.maxLat}`,
      resolution,
    });
    return this.get<any>(`/api/v1/bathymetry/grid?${queryParams.toString()}`);
  }

  /**
   * Get 3D model
   */
  async get3DModel(surveyId: string): Promise<ApiResponse<any>> {
    return this.get<any>(`/api/v1/bathymetry/${surveyId}/3d-model`);
  }

  // ==========================================================================
  // Environmental DNA APIs
  // ==========================================================================

  /**
   * Register eDNA sample
   */
  async registerEDNASample(
    sample: Omit<EnvironmentalDNASample, 'sampleId'>
  ): Promise<ApiResponse<EnvironmentalDNASample>> {
    return this.post<EnvironmentalDNASample>('/api/v1/edna', sample);
  }

  /**
   * Get eDNA sample
   */
  async getEDNASample(sampleId: string): Promise<ApiResponse<EnvironmentalDNASample>> {
    return this.get<EnvironmentalDNASample>(`/api/v1/edna/${sampleId}`);
  }

  /**
   * Submit eDNA analysis results
   */
  async submitEDNAAnalysis(
    sampleId: string,
    analysis: any
  ): Promise<ApiResponse<EnvironmentalDNASample>> {
    return this.put<EnvironmentalDNASample>(
      `/api/v1/edna/${sampleId}/analysis`,
      analysis
    );
  }

  // ==========================================================================
  // Submersible Management APIs
  // ==========================================================================

  /**
   * Register submersible vehicle
   */
  async registerSubmersible(
    submersible:
      | Omit<HOVSpecification, 'vehicleId'>
      | Omit<ROVSpecification, 'vehicleId'>
      | Omit<AUVSpecification, 'vehicleId'>
      | Omit<LanderSpecification, 'landerId'>
  ): Promise<ApiResponse<any>> {
    return this.post<any>('/api/v1/submersibles', submersible);
  }

  /**
   * Get submersible details
   */
  async getSubmersible(vehicleId: string): Promise<ApiResponse<any>> {
    return this.get<any>(`/api/v1/submersibles/${vehicleId}`);
  }

  /**
   * List submersibles
   */
  async listSubmersibles(
    filters?: { type?: SubmersibleType } & PaginationParams
  ): Promise<ApiResponse<PaginatedResponse<any>>> {
    const queryParams = new URLSearchParams();
    if (filters) {
      Object.entries(filters).forEach(([key, value]) => {
        if (value !== undefined) {
          queryParams.append(key, String(value));
        }
      });
    }
    return this.get<PaginatedResponse<any>>(
      `/api/v1/submersibles?${queryParams.toString()}`
    );
  }

  // ==========================================================================
  // Search & Filter APIs
  // ==========================================================================

  /**
   * Search across all data types
   */
  async search(
    query: string,
    filters?: {
      type?: 'expedition' | 'dive' | 'observation' | 'sample' | 'vent';
      dateRange?: DateRangeFilter;
      depthRange?: DepthRangeFilter;
      geographic?: GeographicFilter;
    }
  ): Promise<ApiResponse<any>> {
    const body = { query, ...filters };
    return this.post<any>('/api/v1/search', body);
  }

  /**
   * Get statistics
   */
  async getStatistics(): Promise<ApiResponse<any>> {
    return this.get<any>('/api/v1/statistics');
  }

  // ==========================================================================
  // HTTP Helper Methods
  // ==========================================================================

  private async request<T>(
    method: string,
    path: string,
    body?: any
  ): Promise<ApiResponse<T>> {
    const url = `${this.config.endpoint}${path}`;

    if (this.config.debug) {
      console.log(`[WIA-ENE-040] ${method} ${url}`, body || '');
    }

    try {
      const response = await fetch(url, {
        method,
        headers: this.headers,
        body: body ? JSON.stringify(body) : undefined,
        signal: AbortSignal.timeout(this.config.timeout),
      });

      const data = await response.json();

      if (!response.ok) {
        return {
          success: false,
          error: {
            code: `HTTP_${response.status}`,
            message: data.message || response.statusText,
            details: data,
          },
          metadata: {
            timestamp: new Date().toISOString(),
            version: '1.0.0',
          },
        };
      }

      return {
        success: true,
        data: data as T,
        metadata: {
          timestamp: new Date().toISOString(),
          version: '1.0.0',
        },
      };
    } catch (error) {
      if (this.config.debug) {
        console.error('[WIA-ENE-040] Request failed:', error);
      }

      return {
        success: false,
        error: {
          code: 'NETWORK_ERROR',
          message: error instanceof Error ? error.message : 'Unknown error',
          details: error,
        },
        metadata: {
          timestamp: new Date().toISOString(),
          version: '1.0.0',
        },
      };
    }
  }

  private async get<T>(path: string): Promise<ApiResponse<T>> {
    return this.request<T>('GET', path);
  }

  private async post<T>(path: string, body: any): Promise<ApiResponse<T>> {
    return this.request<T>('POST', path, body);
  }

  private async put<T>(path: string, body: any): Promise<ApiResponse<T>> {
    return this.request<T>('PUT', path, body);
  }

  private async delete<T>(path: string): Promise<ApiResponse<T>> {
    return this.request<T>('DELETE', path);
  }
}

// ============================================================================
// Convenience Functions
// ============================================================================

/**
 * Calculate pressure from depth
 */
export function calculatePressure(depthMeters: number): {
  atm: number;
  bar: number;
  mpa: number;
  psi: number;
} {
  const atm = 1 + depthMeters / 10;
  const bar = atm * 1.01325;
  const mpa = depthMeters * 0.1 * 1025 * 9.81 / 1e6; // ρ * g * h (seawater density ~1025 kg/m³)
  const psi = atm * 14.696;

  return { atm, bar, mpa, psi };
}

/**
 * Determine depth zone from depth
 */
export function getDepthZone(depthMeters: number): DepthZone {
  if (depthMeters < 200) return DepthZone.EPIPELAGIC;
  if (depthMeters < 1000) return DepthZone.MESOPELAGIC;
  if (depthMeters < 4000) return DepthZone.BATHYPELAGIC;
  if (depthMeters < 6000) return DepthZone.ABYSSOPELAGIC;
  return DepthZone.HADOPELAGIC;
}

/**
 * Calculate descent/ascent time
 */
export function calculateDescentTime(
  depthMeters: number,
  descentRate: number = 1000 / 60 // 1000m/hr = ~16.67 m/min
): number {
  return depthMeters / descentRate; // minutes
}

/**
 * Validate coordinates
 */
export function isValidCoordinates(lat: number, lon: number): boolean {
  return lat >= -90 && lat <= 90 && lon >= -180 && lon <= 180;
}

/**
 * Calculate distance between two coordinates (Haversine formula)
 */
export function calculateDistance(
  lat1: number,
  lon1: number,
  lat2: number,
  lon2: number
): number {
  const R = 6371; // Earth radius in km
  const dLat = ((lat2 - lat1) * Math.PI) / 180;
  const dLon = ((lon2 - lon1) * Math.PI) / 180;
  const a =
    Math.sin(dLat / 2) * Math.sin(dLat / 2) +
    Math.cos((lat1 * Math.PI) / 180) *
      Math.cos((lat2 * Math.PI) / 180) *
      Math.sin(dLon / 2) *
      Math.sin(dLon / 2);
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
  return R * c; // Distance in km
}

/**
 * Validate submersible type
 */
export function isValidSubmersibleType(type: string): type is SubmersibleType {
  return Object.values(SubmersibleType).includes(type as SubmersibleType);
}

/**
 * Validate depth zone
 */
export function isValidDepthZone(zone: string): zone is DepthZone {
  return Object.values(DepthZone).includes(zone as DepthZone);
}

/**
 * Validate habitat type
 */
export function isValidHabitatType(habitat: string): habitat is HabitatType {
  return Object.values(HabitatType).includes(habitat as HabitatType);
}

// ============================================================================
// Default Export
// ============================================================================

export default DeepSeaExplorationSDK;
