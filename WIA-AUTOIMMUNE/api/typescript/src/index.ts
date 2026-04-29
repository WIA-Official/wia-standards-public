/**
 * WIA-AUTOIMMUNE SDK
 * Treg-Microbiome Axis Autoimmune Disease Standard
 *
 * @version 1.0.0
 * @license MIT
 * 弘益人間 (홍익인간) - Benefit All Humanity
 */

import {
  ClientConfig,
  Environment,
  DiseaseType,
  AssessmentRequest,
  AssessmentResponse,
  AutoimmuneProfile,
  TregStatusResponse,
  MicrobiomeResponse,
  MicrobiomeSampleRequest,
  SampleSubmissionResponse,
  TreatmentRecommendations,
  TreatmentRecommendationRequest,
  DiseaseActivityResponse,
  DiseaseActivity,
  FlarePrediction,
  FlarePredictionRequest,
  RemissionAssessment,
  RemissionAssessmentRequest,
  TMASScore,
  WiaAutoImmuneError,
} from './types';

export * from './types';

// ============================================================================
// Constants
// ============================================================================

const BASE_URLS: Record<Environment, string> = {
  production: 'https://api.wia.live/autoimmune/v1',
  staging: 'https://api-staging.wia.live/autoimmune/v1',
  development: 'http://localhost:8080/autoimmune/v1',
};

const DEFAULT_TIMEOUT = 30000;
const DEFAULT_RETRIES = 3;

// ============================================================================
// HTTP Client
// ============================================================================

interface RequestOptions {
  method: 'GET' | 'POST' | 'PUT' | 'DELETE' | 'PATCH';
  path: string;
  body?: unknown;
  query?: Record<string, string | number | boolean | undefined>;
}

class HttpClient {
  private baseUrl: string;
  private apiKey: string;
  private clientId?: string;
  private timeout: number;
  private retries: number;

  constructor(config: ClientConfig) {
    this.baseUrl = config.baseUrl ?? BASE_URLS[config.environment ?? 'production'];
    this.apiKey = config.apiKey;
    this.clientId = config.clientId;
    this.timeout = config.timeout ?? DEFAULT_TIMEOUT;
    this.retries = config.retries ?? DEFAULT_RETRIES;
  }

  async request<T>(options: RequestOptions): Promise<T> {
    const url = this.buildUrl(options.path, options.query);
    const headers: Record<string, string> = {
      'Authorization': `Bearer ${this.apiKey}`,
      'Content-Type': 'application/json',
      'Accept': 'application/json',
    };

    if (this.clientId) {
      headers['X-WIA-Client-ID'] = this.clientId;
    }

    let lastError: Error | null = null;
    for (let attempt = 0; attempt <= this.retries; attempt++) {
      try {
        const controller = new AbortController();
        const timeoutId = setTimeout(() => controller.abort(), this.timeout);

        const response = await fetch(url, {
          method: options.method,
          headers,
          body: options.body ? JSON.stringify(options.body) : undefined,
          signal: controller.signal,
        });

        clearTimeout(timeoutId);

        if (!response.ok) {
          const error = await response.json() as WiaAutoImmuneError;
          throw new WiaAutoImmuneApiError(error.code, error.message, error.details);
        }

        return await response.json() as T;
      } catch (error) {
        lastError = error as Error;
        if (attempt < this.retries && this.shouldRetry(error)) {
          await this.delay(Math.pow(2, attempt) * 1000);
          continue;
        }
        throw error;
      }
    }
    throw lastError;
  }

  private buildUrl(path: string, query?: Record<string, string | number | boolean | undefined>): string {
    const url = new URL(path, this.baseUrl);
    if (query) {
      Object.entries(query).forEach(([key, value]) => {
        if (value !== undefined) {
          url.searchParams.append(key, String(value));
        }
      });
    }
    return url.toString();
  }

  private shouldRetry(error: unknown): boolean {
    if (error instanceof WiaAutoImmuneApiError) {
      return error.code.startsWith('5'); // Server errors
    }
    return error instanceof Error && error.name === 'AbortError';
  }

  private delay(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}

// ============================================================================
// Error Class
// ============================================================================

export class WiaAutoImmuneApiError extends Error {
  constructor(
    public code: string,
    message: string,
    public details?: unknown
  ) {
    super(message);
    this.name = 'WiaAutoImmuneApiError';
  }
}

// ============================================================================
// Resource Classes
// ============================================================================

class AssessmentsResource {
  constructor(private client: HttpClient) {}

  /**
   * Create a comprehensive autoimmune assessment
   */
  async create(request: AssessmentRequest): Promise<AssessmentResponse> {
    return this.client.request<AssessmentResponse>({
      method: 'POST',
      path: '/assess',
      body: request,
    });
  }

  /**
   * Get assessment by ID
   */
  async get(assessmentId: string): Promise<AssessmentResponse> {
    return this.client.request<AssessmentResponse>({
      method: 'GET',
      path: `/assessments/${assessmentId}`,
    });
  }

  /**
   * List assessments for a patient
   */
  async list(patientId: string, options?: { limit?: number; offset?: number }): Promise<AssessmentResponse[]> {
    return this.client.request<AssessmentResponse[]>({
      method: 'GET',
      path: `/profiles/${patientId}/assessments`,
      query: options,
    });
  }
}

class ProfilesResource {
  constructor(private client: HttpClient) {}

  /**
   * Get patient autoimmune profile
   */
  async get(
    patientId: string,
    options?: { include?: ('treg' | 'microbiome' | 'history' | 'treatments')[] }
  ): Promise<AutoimmuneProfile> {
    return this.client.request<AutoimmuneProfile>({
      method: 'GET',
      path: `/profiles/${patientId}`,
      query: options?.include ? { include: options.include.join(',') } : undefined,
    });
  }

  /**
   * Calculate TMAS (Treg-Microbiome Axis Score)
   */
  async calculateTMAS(patientId: string): Promise<TMASScore> {
    return this.client.request<TMASScore>({
      method: 'GET',
      path: `/profiles/${patientId}/tmas`,
    });
  }
}

class TregResource {
  constructor(private client: HttpClient) {}

  /**
   * Get Treg functional status
   */
  async getStatus(
    patientId: string,
    options?: { dateFrom?: string; dateTo?: string }
  ): Promise<TregStatusResponse> {
    return this.client.request<TregStatusResponse>({
      method: 'GET',
      path: `/treg-status/${patientId}`,
      query: {
        date_from: options?.dateFrom,
        date_to: options?.dateTo,
      },
    });
  }

  /**
   * Get Treg history over time
   */
  async getHistory(patientId: string, months: number = 12): Promise<TregStatusResponse[]> {
    return this.client.request<TregStatusResponse[]>({
      method: 'GET',
      path: `/treg-status/${patientId}/history`,
      query: { months },
    });
  }
}

class MicrobiomeResource {
  constructor(private client: HttpClient) {}

  /**
   * Get microbiome analysis
   */
  async get(
    patientId: string,
    options?: { includeTaxa?: boolean; scfaFocus?: boolean }
  ): Promise<MicrobiomeResponse> {
    return this.client.request<MicrobiomeResponse>({
      method: 'GET',
      path: `/microbiome/${patientId}`,
      query: {
        include_taxa: options?.includeTaxa,
        scfa_focus: options?.scfaFocus,
      },
    });
  }

  /**
   * Submit new microbiome sample
   */
  async submitSample(
    patientId: string,
    sample: MicrobiomeSampleRequest
  ): Promise<SampleSubmissionResponse> {
    return this.client.request<SampleSubmissionResponse>({
      method: 'POST',
      path: `/microbiome/${patientId}`,
      body: sample,
    });
  }

  /**
   * Get microbiome history
   */
  async getHistory(patientId: string): Promise<MicrobiomeResponse[]> {
    return this.client.request<MicrobiomeResponse[]>({
      method: 'GET',
      path: `/microbiome/${patientId}/history`,
    });
  }
}

class TreatmentResource {
  constructor(private client: HttpClient) {}

  /**
   * Get personalized treatment recommendations
   */
  async getRecommendations(request: TreatmentRecommendationRequest): Promise<TreatmentRecommendations> {
    return this.client.request<TreatmentRecommendations>({
      method: 'POST',
      path: '/treatment/recommend',
      body: request,
    });
  }
}

class DiseaseActivityResource {
  constructor(private client: HttpClient) {}

  /**
   * Get disease activity scores
   */
  async get(patientId: string, diseaseType?: DiseaseType): Promise<DiseaseActivityResponse> {
    return this.client.request<DiseaseActivityResponse>({
      method: 'GET',
      path: `/disease-activity/${patientId}`,
      query: diseaseType ? { disease_type: diseaseType } : undefined,
    });
  }

  /**
   * Record new disease activity measurement
   */
  async record(patientId: string, activity: DiseaseActivity): Promise<DiseaseActivityResponse> {
    return this.client.request<DiseaseActivityResponse>({
      method: 'POST',
      path: `/disease-activity/${patientId}`,
      body: activity,
    });
  }
}

class PredictionResource {
  constructor(private client: HttpClient) {}

  /**
   * Predict flare risk
   */
  async predictFlare(request: FlarePredictionRequest): Promise<FlarePrediction> {
    return this.client.request<FlarePrediction>({
      method: 'POST',
      path: '/flare/predict',
      body: request,
    });
  }

  /**
   * Assess remission probability
   */
  async assessRemission(request: RemissionAssessmentRequest): Promise<RemissionAssessment> {
    return this.client.request<RemissionAssessment>({
      method: 'POST',
      path: '/remission/assess',
      body: request,
    });
  }
}

// ============================================================================
// Main Client
// ============================================================================

export class WiaAutoImmuneClient {
  private client: HttpClient;

  public readonly assessments: AssessmentsResource;
  public readonly profiles: ProfilesResource;
  public readonly treg: TregResource;
  public readonly microbiome: MicrobiomeResource;
  public readonly treatment: TreatmentResource;
  public readonly diseaseActivity: DiseaseActivityResource;
  public readonly predictions: PredictionResource;

  constructor(config: ClientConfig) {
    this.client = new HttpClient(config);

    this.assessments = new AssessmentsResource(this.client);
    this.profiles = new ProfilesResource(this.client);
    this.treg = new TregResource(this.client);
    this.microbiome = new MicrobiomeResource(this.client);
    this.treatment = new TreatmentResource(this.client);
    this.diseaseActivity = new DiseaseActivityResource(this.client);
    this.predictions = new PredictionResource(this.client);
  }
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Calculate TMAS score from raw data
 */
export function calculateTMAS(
  tregFunction: number,
  dysbiosisScore: number,
  diseaseActivityNormalized: number
): TMASScore {
  const tregComponent = tregFunction;
  const microbiomeComponent = 100 - dysbiosisScore;
  const activityComponent = 100 - diseaseActivityNormalized;

  const total = (tregComponent * 0.4) + (microbiomeComponent * 0.4) + (activityComponent * 0.2);

  let interpretation: TMASScore['interpretation'];
  if (total >= 80) interpretation = 'excellent';
  else if (total >= 60) interpretation = 'good';
  else if (total >= 40) interpretation = 'impaired';
  else if (total >= 20) interpretation = 'poor';
  else interpretation = 'critical';

  return {
    total: Math.round(total * 10) / 10,
    treg_component: Math.round(tregComponent * 10) / 10,
    microbiome_component: Math.round(microbiomeComponent * 10) / 10,
    activity_component: Math.round(activityComponent * 10) / 10,
    interpretation,
    calculated_at: new Date().toISOString(),
  };
}

/**
 * Assess flare risk level from probability
 */
export function assessFlareRisk(probability: number): 'low' | 'moderate' | 'high' | 'very_high' {
  if (probability < 30) return 'low';
  if (probability < 50) return 'moderate';
  if (probability < 70) return 'high';
  return 'very_high';
}

/**
 * Normalize disease activity score to 0-100 scale
 */
export function normalizeDiseaseActivity(diseaseType: DiseaseType, score: number): number {
  const ranges: Record<DiseaseType, { min: number; max: number }> = {
    RA: { min: 0, max: 10 }, // DAS28
    SLE: { min: 0, max: 105 }, // SLEDAI
    MS: { min: 0, max: 10 }, // EDSS
    T1D: { min: 4, max: 14 }, // HbA1c
    IBD: { min: 0, max: 450 }, // CDAI
    PSO: { min: 0, max: 72 }, // PASI
    HT: { min: 0, max: 100 }, // Custom
    GD: { min: 0, max: 100 }, // Custom
  };

  const range = ranges[diseaseType];
  return Math.min(100, Math.max(0, ((score - range.min) / (range.max - range.min)) * 100));
}

// ============================================================================
// Default Export
// ============================================================================

export default WiaAutoImmuneClient;
