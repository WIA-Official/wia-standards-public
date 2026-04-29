/**
 * WIA-CHRONIC-PAIN SDK
 * Neuroplasticity Reversal Chronic Pain Standard
 *
 * @version 1.0.0
 * @license MIT
 * 弘益人間 (홍익인간) - Benefit All Humanity
 */

import {
  ClientConfig,
  Environment,
  PainType,
  AssessmentRequest,
  AssessmentResponse,
  ChronicPainProfile,
  SensitizationResponse,
  NeuromodulationPlanRequest,
  NeuromodulationPlan,
  TreatmentRecommendations,
  NeuroplasticityIndex,
  OpioidRiskResponse,
  TaperPlan,
  ChronificationPrediction,
  WiaChronicPainError,
} from './types';

export * from './types';

// ============================================================================
// Constants
// ============================================================================

const BASE_URLS: Record<Environment, string> = {
  production: 'https://api.wia.live/chronic-pain/v1',
  staging: 'https://api-staging.wia.live/chronic-pain/v1',
  development: 'http://localhost:8080/chronic-pain/v1',
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
          const error = await response.json() as WiaChronicPainError;
          throw new WiaChronicPainApiError(error.code, error.message, error.details);
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
    if (error instanceof WiaChronicPainApiError) {
      return error.code.startsWith('5');
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

export class WiaChronicPainApiError extends Error {
  constructor(
    public code: string,
    message: string,
    public details?: unknown
  ) {
    super(message);
    this.name = 'WiaChronicPainApiError';
  }
}

// ============================================================================
// Resource Classes
// ============================================================================

class AssessmentsResource {
  constructor(private client: HttpClient) {}

  async create(request: AssessmentRequest): Promise<AssessmentResponse> {
    return this.client.request<AssessmentResponse>({
      method: 'POST',
      path: '/assess',
      body: request,
    });
  }

  async get(assessmentId: string): Promise<AssessmentResponse> {
    return this.client.request<AssessmentResponse>({
      method: 'GET',
      path: `/assessments/${assessmentId}`,
    });
  }

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

  async get(
    patientId: string,
    options?: { include?: ('sensitization' | 'neuroimaging' | 'psychosocial' | 'treatments')[] }
  ): Promise<ChronicPainProfile> {
    return this.client.request<ChronicPainProfile>({
      method: 'GET',
      path: `/profiles/${patientId}`,
      query: options?.include ? { include: options.include.join(',') } : undefined,
    });
  }
}

class SensitizationResource {
  constructor(private client: HttpClient) {}

  async get(patientId: string): Promise<SensitizationResponse> {
    return this.client.request<SensitizationResponse>({
      method: 'GET',
      path: `/sensitization/${patientId}`,
    });
  }

  async getHistory(patientId: string, months: number = 12): Promise<SensitizationResponse[]> {
    return this.client.request<SensitizationResponse[]>({
      method: 'GET',
      path: `/sensitization/${patientId}/history`,
      query: { months },
    });
  }
}

class NeuromodulationResource {
  constructor(private client: HttpClient) {}

  async createPlan(request: NeuromodulationPlanRequest): Promise<NeuromodulationPlan> {
    return this.client.request<NeuromodulationPlan>({
      method: 'POST',
      path: '/neuromodulation/plan',
      body: request,
    });
  }

  async getPlan(planId: string): Promise<NeuromodulationPlan> {
    return this.client.request<NeuromodulationPlan>({
      method: 'GET',
      path: `/neuromodulation/plan/${planId}`,
    });
  }
}

class TreatmentResource {
  constructor(private client: HttpClient) {}

  async getRecommendations(patientId: string): Promise<TreatmentRecommendations> {
    return this.client.request<TreatmentRecommendations>({
      method: 'POST',
      path: '/treatment/recommend',
      body: { patient_id: patientId },
    });
  }
}

class NeuroplasticityResource {
  constructor(private client: HttpClient) {}

  async getIndex(patientId: string): Promise<NeuroplasticityIndex> {
    return this.client.request<NeuroplasticityIndex>({
      method: 'GET',
      path: `/neuroplasticity/index/${patientId}`,
    });
  }
}

class OpioidResource {
  constructor(private client: HttpClient) {}

  async getRisk(patientId: string): Promise<OpioidRiskResponse> {
    return this.client.request<OpioidRiskResponse>({
      method: 'GET',
      path: `/opioid-risk/${patientId}`,
    });
  }

  async createTaperPlan(patientId: string, targetMme?: number): Promise<TaperPlan> {
    return this.client.request<TaperPlan>({
      method: 'POST',
      path: '/opioid-taper/plan',
      body: { patient_id: patientId, target_mme: targetMme },
    });
  }
}

class PredictionResource {
  constructor(private client: HttpClient) {}

  async predictChronification(patientId: string): Promise<ChronificationPrediction> {
    return this.client.request<ChronificationPrediction>({
      method: 'POST',
      path: '/predict/chronification',
      body: { patient_id: patientId },
    });
  }

  async predictResponse(patientId: string, treatment: string): Promise<{
    treatment: string;
    predicted_response: 'none' | 'partial' | 'good' | 'excellent';
    confidence: number;
  }> {
    return this.client.request({
      method: 'POST',
      path: '/predict/response',
      body: { patient_id: patientId, treatment },
    });
  }
}

// ============================================================================
// Main Client
// ============================================================================

export class WiaChronicPainClient {
  private client: HttpClient;

  public readonly assessments: AssessmentsResource;
  public readonly profiles: ProfilesResource;
  public readonly sensitization: SensitizationResource;
  public readonly neuromodulation: NeuromodulationResource;
  public readonly treatment: TreatmentResource;
  public readonly neuroplasticity: NeuroplasticityResource;
  public readonly opioid: OpioidResource;
  public readonly predictions: PredictionResource;

  constructor(config: ClientConfig) {
    this.client = new HttpClient(config);

    this.assessments = new AssessmentsResource(this.client);
    this.profiles = new ProfilesResource(this.client);
    this.sensitization = new SensitizationResource(this.client);
    this.neuromodulation = new NeuromodulationResource(this.client);
    this.treatment = new TreatmentResource(this.client);
    this.neuroplasticity = new NeuroplasticityResource(this.client);
    this.opioid = new OpioidResource(this.client);
    this.predictions = new PredictionResource(this.client);
  }
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Calculate Neuroplasticity Reversal Index (NRI)
 */
export function calculateNRI(
  csScore: number, // Central sensitization (0-100)
  psychosocialScore: number, // 0-100
  structuralScore: number, // 0-100
  durationMonths: number
): number {
  // Duration factor: longer duration = lower reversal potential
  const durationFactor = Math.min(100, (durationMonths / 120) * 100);

  const nri = 100 - (
    (csScore * 0.35) +
    (psychosocialScore * 0.25) +
    (structuralScore * 0.25) +
    (durationFactor * 0.15)
  );

  return Math.max(0, Math.min(100, Math.round(nri * 10) / 10));
}

/**
 * Interpret NRI score
 */
export function interpretNRI(nri: number): {
  level: 'excellent' | 'good' | 'moderate' | 'limited' | 'challenging';
  description: string;
} {
  if (nri >= 80) {
    return { level: 'excellent', description: 'Excellent reversal potential (early intervention window)' };
  } else if (nri >= 60) {
    return { level: 'good', description: 'Good reversal potential (standard protocols)' };
  } else if (nri >= 40) {
    return { level: 'moderate', description: 'Moderate potential (intensive intervention needed)' };
  } else if (nri >= 20) {
    return { level: 'limited', description: 'Limited potential (prolonged multimodal approach)' };
  } else {
    return { level: 'challenging', description: 'Challenging (focus on function over cure)' };
  }
}

/**
 * Calculate CSI interpretation
 */
export function interpretCSI(score: number): {
  level: 'subclinical' | 'mild' | 'moderate' | 'severe' | 'extreme';
  centralSensitizationLikely: boolean;
} {
  if (score < 25) {
    return { level: 'subclinical', centralSensitizationLikely: false };
  } else if (score < 40) {
    return { level: 'mild', centralSensitizationLikely: false };
  } else if (score < 50) {
    return { level: 'moderate', centralSensitizationLikely: true };
  } else if (score < 60) {
    return { level: 'severe', centralSensitizationLikely: true };
  } else {
    return { level: 'extreme', centralSensitizationLikely: true };
  }
}

/**
 * Calculate MME (Morphine Milligram Equivalents)
 */
export function calculateMME(
  medication: string,
  dailyDose: number
): number {
  const conversionFactors: Record<string, number> = {
    morphine: 1,
    codeine: 0.15,
    hydrocodone: 1,
    oxycodone: 1.5,
    hydromorphone: 4,
    oxymorphone: 3,
    methadone_1_20: 4, // For doses 1-20mg/day
    methadone_21_40: 8, // For doses 21-40mg/day
    methadone_41_60: 10, // For doses 41-60mg/day
    fentanyl_patch: 2.4, // per mcg/hr
    tramadol: 0.1,
    tapentadol: 0.4,
    buprenorphine: 12.6, // Not directly comparable
  };

  const factor = conversionFactors[medication.toLowerCase()] || 1;
  return Math.round(dailyDose * factor * 10) / 10;
}

/**
 * Assess opioid risk level
 */
export function assessOpioidRisk(mme: number, ortScore: number): 'low' | 'moderate' | 'high' {
  if (mme >= 90 || ortScore >= 8) {
    return 'high';
  } else if (mme >= 50 || ortScore >= 4) {
    return 'moderate';
  }
  return 'low';
}

/**
 * Classify pain type based on characteristics
 */
export function classifyPainType(
  hasNerveInjury: boolean,
  csiScore: number,
  painWidespread: boolean,
  proportionalToTissue: boolean
): PainType {
  if (hasNerveInjury && csiScore < 40) {
    return 'neuropathic';
  }
  if (csiScore >= 40 || painWidespread) {
    if (hasNerveInjury) {
      return 'mixed';
    }
    return 'nociplastic';
  }
  if (proportionalToTissue) {
    return 'nociceptive';
  }
  return 'mixed';
}

// ============================================================================
// Default Export
// ============================================================================

export default WiaChronicPainClient;
