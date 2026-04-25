/**
 * WIA-ENE-030: Biodiversity Index Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// Export all types
export * from './types';

import {
  SpeciesObservation,
  BiodiversitySurvey,
  DiversityIndices,
  HabitatAssessment,
  DNABarcoding,
  EDNASample,
  EDNAMetabarcoding,
  BiodiversityReport,
  KPIDashboard,
  ApiResponse,
  PaginatedResponse,
  PaginationParams,
  DateRangeFilter,
  SpeciesFilter,
  LocationFilter,
  ObserverCertification,
  IUCNCategory,
  HabitatType,
  EcosystemType,
  SurveyType,
  GeneRegion,
  CertificationLevel,
  SpeciesComposition,
} from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface BiodiversitySDKConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  retries?: number;
  debug?: boolean;
}

// ============================================================================
// SDK Client
// ============================================================================

export class BiodiversitySDK {
  private config: Required<BiodiversitySDKConfig>;
  private headers: Record<string, string>;

  constructor(config: BiodiversitySDKConfig) {
    this.config = {
      timeout: 30000,
      retries: 3,
      debug: false,
      ...config,
    };

    this.headers = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'X-WIA-Standard': 'ENE-030',
      'X-WIA-Version': '1.0.0',
    };
  }

  // ==========================================================================
  // Observation APIs
  // ==========================================================================

  /**
   * Create a new species observation
   */
  async createObservation(
    observation: Omit<SpeciesObservation, 'observationId' | 'timestamp' | 'metadata'>
  ): Promise<ApiResponse<SpeciesObservation>> {
    return this.post<SpeciesObservation>('/api/v1/observation/create', observation);
  }

  /**
   * Get observation by ID
   */
  async getObservation(observationId: string): Promise<ApiResponse<SpeciesObservation>> {
    return this.get<SpeciesObservation>(`/api/v1/observation/${observationId}`);
  }

  /**
   * Update observation
   */
  async updateObservation(
    observationId: string,
    updates: Partial<SpeciesObservation>
  ): Promise<ApiResponse<SpeciesObservation>> {
    return this.put<SpeciesObservation>(`/api/v1/observation/${observationId}`, updates);
  }

  /**
   * Delete observation
   */
  async deleteObservation(observationId: string): Promise<ApiResponse<void>> {
    return this.delete<void>(`/api/v1/observation/${observationId}`);
  }

  /**
   * List observations with filters
   */
  async listObservations(params?: {
    pagination?: PaginationParams;
    dateRange?: DateRangeFilter;
    species?: SpeciesFilter;
    location?: LocationFilter;
  }): Promise<ApiResponse<PaginatedResponse<SpeciesObservation>>> {
    const queryParams = new URLSearchParams();

    if (params?.pagination) {
      Object.entries(params.pagination).forEach(([key, value]) => {
        if (value !== undefined) queryParams.append(key, String(value));
      });
    }

    if (params?.dateRange) {
      queryParams.append('startDate', params.dateRange.startDate);
      queryParams.append('endDate', params.dateRange.endDate);
    }

    if (params?.species) {
      Object.entries(params.species).forEach(([key, value]) => {
        if (value !== undefined) queryParams.append(key, String(value));
      });
    }

    if (params?.location) {
      Object.entries(params.location).forEach(([key, value]) => {
        if (value !== undefined && key !== 'boundingBox') {
          queryParams.append(key, String(value));
        }
      });

      if (params.location.boundingBox) {
        queryParams.append('bbox', JSON.stringify(params.location.boundingBox));
      }
    }

    return this.get<PaginatedResponse<SpeciesObservation>>(
      `/api/v1/observations?${queryParams.toString()}`
    );
  }

  // ==========================================================================
  // Survey APIs
  // ==========================================================================

  /**
   * Create a new biodiversity survey
   */
  async createSurvey(
    survey: Omit<BiodiversitySurvey, 'surveyId' | 'status'>
  ): Promise<ApiResponse<BiodiversitySurvey>> {
    return this.post<BiodiversitySurvey>('/api/v1/survey/create', survey);
  }

  /**
   * Get survey by ID
   */
  async getSurvey(surveyId: string): Promise<ApiResponse<BiodiversitySurvey>> {
    return this.get<BiodiversitySurvey>(`/api/v1/survey/${surveyId}`);
  }

  /**
   * Get survey results
   */
  async getSurveyResults(surveyId: string): Promise<ApiResponse<BiodiversitySurvey['results']>> {
    return this.get<BiodiversitySurvey['results']>(`/api/v1/survey/${surveyId}/results`);
  }

  /**
   * Add observation to survey
   */
  async addObservationToSurvey(
    surveyId: string,
    observationId: string
  ): Promise<ApiResponse<void>> {
    return this.post<void>(`/api/v1/survey/${surveyId}/observation`, { observationId });
  }

  /**
   * Update survey status
   */
  async updateSurveyStatus(
    surveyId: string,
    status: BiodiversitySurvey['status']
  ): Promise<ApiResponse<BiodiversitySurvey>> {
    return this.put<BiodiversitySurvey>(`/api/v1/survey/${surveyId}/status`, { status });
  }

  // ==========================================================================
  // Diversity Analysis APIs
  // ==========================================================================

  /**
   * Calculate diversity indices
   */
  async calculateDiversity(params: {
    surveyId?: string;
    observations?: SpeciesComposition[];
  }): Promise<ApiResponse<DiversityIndices>> {
    return this.post<DiversityIndices>('/api/v1/diversity/calculate', params);
  }

  /**
   * Get diversity indices by survey ID
   */
  async getDiversityIndices(surveyId: string): Promise<ApiResponse<DiversityIndices>> {
    return this.get<DiversityIndices>(`/api/v1/diversity/${surveyId}`);
  }

  /**
   * Get diversity trends over time
   */
  async getDiversityTrends(params: {
    siteName: string;
    dateRange: DateRangeFilter;
    metric: 'shannonIndex' | 'simpsonDiversity' | 'speciesRichness';
  }): Promise<ApiResponse<any>> {
    const queryParams = new URLSearchParams({
      siteName: params.siteName,
      startDate: params.dateRange.startDate,
      endDate: params.dateRange.endDate,
      metric: params.metric,
    });

    return this.get<any>(`/api/v1/diversity/trends?${queryParams.toString()}`);
  }

  // ==========================================================================
  // Species Information APIs
  // ==========================================================================

  /**
   * Search for species
   */
  async searchSpecies(query: string): Promise<ApiResponse<any[]>> {
    const queryParams = new URLSearchParams({ q: query });
    return this.get<any[]>(`/api/v1/species/search?${queryParams.toString()}`);
  }

  /**
   * Get species details
   */
  async getSpeciesInfo(scientificName: string): Promise<ApiResponse<any>> {
    return this.get<any>(`/api/v1/species/${encodeURIComponent(scientificName)}`);
  }

  /**
   * List endangered species
   */
  async listEndangeredSpecies(params?: {
    region?: string;
    category?: IUCNCategory;
  }): Promise<ApiResponse<any[]>> {
    const queryParams = new URLSearchParams();
    if (params?.region) queryParams.append('region', params.region);
    if (params?.category) queryParams.append('category', params.category);

    return this.get<any[]>(
      `/api/v1/species/endangered${queryParams.toString() ? '?' + queryParams.toString() : ''}`
    );
  }

  /**
   * List endemic species
   */
  async listEndemicSpecies(region: string): Promise<ApiResponse<any[]>> {
    const queryParams = new URLSearchParams({ region });
    return this.get<any[]>(`/api/v1/species/endemic?${queryParams.toString()}`);
  }

  // ==========================================================================
  // Habitat Assessment APIs
  // ==========================================================================

  /**
   * Assess habitat quality
   */
  async assessHabitat(
    assessment: Omit<HabitatAssessment, 'assessmentId' | 'assessmentDate'>
  ): Promise<ApiResponse<HabitatAssessment>> {
    return this.post<HabitatAssessment>('/api/v1/habitat/assess', assessment);
  }

  /**
   * Get habitat assessment
   */
  async getHabitatAssessment(siteId: string): Promise<ApiResponse<HabitatAssessment>> {
    return this.get<HabitatAssessment>(`/api/v1/habitat/${siteId}`);
  }

  /**
   * Get habitat quality analysis
   */
  async getHabitatQuality(params: {
    habitatType?: HabitatType;
    region?: string;
  }): Promise<ApiResponse<any>> {
    const queryParams = new URLSearchParams();
    if (params.habitatType) queryParams.append('habitatType', params.habitatType);
    if (params.region) queryParams.append('region', params.region);

    return this.get<any>(
      `/api/v1/habitat/quality${queryParams.toString() ? '?' + queryParams.toString() : ''}`
    );
  }

  // ==========================================================================
  // eDNA & Molecular APIs
  // ==========================================================================

  /**
   * Submit DNA barcoding analysis
   */
  async submitDNABarcoding(
    data: Omit<DNABarcoding, 'matchedSpecies' | 'topMatch'>
  ): Promise<ApiResponse<DNABarcoding>> {
    return this.post<DNABarcoding>('/api/v1/dna/barcode', data);
  }

  /**
   * Submit eDNA sample
   */
  async submitEDNASample(
    sample: EDNASample
  ): Promise<ApiResponse<EDNASample>> {
    return this.post<EDNASample>('/api/v1/edna/submit', sample);
  }

  /**
   * Get eDNA analysis results
   */
  async getEDNAResults(sampleId: string): Promise<ApiResponse<EDNAMetabarcoding>> {
    return this.get<EDNAMetabarcoding>(`/api/v1/edna/${sampleId}/results`);
  }

  /**
   * Request metabarcoding analysis
   */
  async requestMetabarcoding(params: {
    sampleId: string;
    geneRegion: GeneRegion;
  }): Promise<ApiResponse<any>> {
    return this.post<any>('/api/v1/edna/metabarcoding', params);
  }

  // ==========================================================================
  // Reporting & Analytics APIs
  // ==========================================================================

  /**
   * Generate biodiversity report
   */
  async generateReport(params: {
    reportType: 'quarterly' | 'annual' | 'comprehensive';
    region: string;
    period: DateRangeFilter;
  }): Promise<ApiResponse<BiodiversityReport>> {
    return this.post<BiodiversityReport>('/api/v1/report/generate', params);
  }

  /**
   * Get report by ID
   */
  async getReport(reportId: string): Promise<ApiResponse<BiodiversityReport>> {
    return this.get<BiodiversityReport>(`/api/v1/report/${reportId}`);
  }

  /**
   * Get KPI dashboard
   */
  async getKPIDashboard(params?: {
    region?: string;
    date?: string;
  }): Promise<ApiResponse<KPIDashboard>> {
    const queryParams = new URLSearchParams();
    if (params?.region) queryParams.append('region', params.region);
    if (params?.date) queryParams.append('date', params.date);

    return this.get<KPIDashboard>(
      `/api/v1/analytics/kpi${queryParams.toString() ? '?' + queryParams.toString() : ''}`
    );
  }

  // ==========================================================================
  // Certification APIs
  // ==========================================================================

  /**
   * Get observer certification
   */
  async getObserverCertification(observerId: string): Promise<ApiResponse<ObserverCertification>> {
    return this.get<ObserverCertification>(`/api/v1/certification/${observerId}`);
  }

  /**
   * Apply for certification
   */
  async applyCertification(params: {
    observerId: string;
    level: CertificationLevel;
    specialization?: string[];
  }): Promise<ApiResponse<ObserverCertification>> {
    return this.post<ObserverCertification>('/api/v1/certification/apply', params);
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
      console.log(`[WIA-ENE-030] ${method} ${url}`, body || '');
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
        console.error('[WIA-ENE-030] Request failed:', error);
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
// Utility Functions
// ============================================================================

/**
 * Calculate Shannon diversity index
 */
export function calculateShannonIndex(composition: SpeciesComposition[]): number {
  const total = composition.reduce((sum, sp) => sum + sp.count, 0);
  if (total === 0) return 0;

  return -1 * composition.reduce((sum, sp) => {
    const pi = sp.count / total;
    return sum + (pi > 0 ? pi * Math.log(pi) : 0);
  }, 0);
}

/**
 * Calculate Shannon equitability
 */
export function calculateShannonEquitability(
  shannonIndex: number,
  speciesRichness: number
): number {
  if (speciesRichness <= 1) return 0;
  return shannonIndex / Math.log(speciesRichness);
}

/**
 * Calculate Simpson index
 */
export function calculateSimpsonIndex(composition: SpeciesComposition[]): number {
  const total = composition.reduce((sum, sp) => sum + sp.count, 0);
  if (total === 0) return 0;

  return composition.reduce((sum, sp) => {
    const pi = sp.count / total;
    return sum + (pi * pi);
  }, 0);
}

/**
 * Calculate Simpson diversity (1-D)
 */
export function calculateSimpsonDiversity(simpsonIndex: number): number {
  return 1 - simpsonIndex;
}

/**
 * Calculate Margalef richness
 */
export function calculateMargalefRichness(
  speciesRichness: number,
  totalIndividuals: number
): number {
  if (totalIndividuals <= 1) return 0;
  return (speciesRichness - 1) / Math.log(totalIndividuals);
}

/**
 * Calculate Berger-Parker dominance index
 */
export function calculateBergerParkerIndex(composition: SpeciesComposition[]): number {
  const total = composition.reduce((sum, sp) => sum + sp.count, 0);
  if (total === 0) return 0;

  const maxCount = Math.max(...composition.map(sp => sp.count));
  return maxCount / total;
}

/**
 * Calculate all diversity indices at once
 */
export function calculateAllIndices(
  composition: SpeciesComposition[]
): Partial<DiversityIndices> {
  const speciesRichness = composition.length;
  const totalIndividuals = composition.reduce((sum, sp) => sum + sp.count, 0);

  const shannonIndex = calculateShannonIndex(composition);
  const shannonEquitability = calculateShannonEquitability(shannonIndex, speciesRichness);
  const simpsonIndex = calculateSimpsonIndex(composition);
  const simpsonDiversity = calculateSimpsonDiversity(simpsonIndex);
  const bergerParkerIndex = calculateBergerParkerIndex(composition);
  const margalefRichness = calculateMargalefRichness(speciesRichness, totalIndividuals);

  return {
    speciesRichness,
    totalIndividuals,
    shannonIndex,
    shannonEquitability,
    simpsonIndex,
    simpsonDiversity,
    bergerParkerIndex,
    margalefRichness,
  };
}

/**
 * Validate scientific name format
 */
export function isValidScientificName(name: string): boolean {
  // Basic check: should be at least two words (genus + species)
  const parts = name.trim().split(/\s+/);
  return parts.length >= 2 && /^[A-Z][a-z]+$/.test(parts[0]);
}

/**
 * Check if species is threatened (VU, EN, or CR)
 */
export function isThreatened(iucnCategory?: IUCNCategory): boolean {
  return [IUCNCategory.VU, IUCNCategory.EN, IUCNCategory.CR].includes(
    iucnCategory as IUCNCategory
  );
}

/**
 * Calculate habitat suitability index (HSI)
 */
export function calculateHSI(assessment: HabitatAssessment): number {
  // Simplified HSI calculation
  const integrityWeight = 0.4;
  const conditionWeight = 0.3;
  const vegetationWeight = 0.3;

  const integrityScore = assessment.quality.integrityScore / 100;
  const conditionScore = assessment.quality.conditionScore / 100;
  const vegetationScore = assessment.vegetation.canopyCover / 100;

  return (
    integrityScore * integrityWeight +
    conditionScore * conditionWeight +
    vegetationScore * vegetationWeight
  );
}

// ============================================================================
// Default Export
// ============================================================================

export default BiodiversitySDK;
