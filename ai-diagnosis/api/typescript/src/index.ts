/**
 * WIA-MED-018: AI Diagnosis Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @packageDocumentation
 * @module wia-ai-diagnosis
 * @license MIT
 * @author WIA / SmileStory Inc.
 * @version 1.0.0
 */

import {
  DiagnosisResult,
  AIModel,
  InferenceRequest,
  InferenceResponse,
  DiseaseType,
  ModelMetrics,
  ExplainabilityReport,
  ValidationResult,
  ClinicalContext,
  ImageInput,
  PaginatedResponse,
} from './types';

export * from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface AIDiagnosisConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  retries?: number;
  debug?: boolean;
}

// ============================================================================
// SDK Client
// ============================================================================

export class AIDiagnosisClient {
  private config: Required<AIDiagnosisConfig>;
  private headers: Record<string, string>;

  constructor(config: AIDiagnosisConfig) {
    this.config = {
      timeout: 60000,
      retries: 3,
      debug: false,
      ...config,
    };

    this.headers = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'X-WIA-Standard': 'MED-018',
      'X-WIA-Version': '1.0.0',
    };
  }

  // ==========================================================================
  // Diagnosis APIs
  // ==========================================================================

  async diagnose(request: InferenceRequest): Promise<InferenceResponse> {
    return this.post<InferenceResponse>('/api/v1/diagnose', request);
  }

  async getDiagnosis(diagnosisId: string): Promise<DiagnosisResult> {
    return this.get<DiagnosisResult>(`/api/v1/diagnoses/${diagnosisId}`);
  }

  async listDiagnoses(patientId: string): Promise<DiagnosisResult[]> {
    return this.get<DiagnosisResult[]>(`/api/v1/patients/${patientId}/diagnoses`);
  }

  async submitFeedback(diagnosisId: string, feedback: { correct: boolean; notes?: string }): Promise<void> {
    await this.post<void>(`/api/v1/diagnoses/${diagnosisId}/feedback`, feedback);
  }

  // ==========================================================================
  // Model APIs
  // ==========================================================================

  async listModels(): Promise<AIModel[]> {
    return this.get<AIModel[]>('/api/v1/models');
  }

  async getModel(modelId: string): Promise<AIModel> {
    return this.get<AIModel>(`/api/v1/models/${modelId}`);
  }

  async getModelMetrics(modelId: string): Promise<ModelMetrics> {
    return this.get<ModelMetrics>(`/api/v1/models/${modelId}/metrics`);
  }

  async validateModel(modelId: string, testData: InferenceRequest[]): Promise<ValidationResult> {
    return this.post<ValidationResult>(`/api/v1/models/${modelId}/validate`, { testData });
  }

  // ==========================================================================
  // Image Analysis APIs
  // ==========================================================================

  async analyzeImage(image: ImageInput, context?: ClinicalContext): Promise<InferenceResponse> {
    return this.post<InferenceResponse>('/api/v1/analyze/image', { image, context });
  }

  async analyzeRadiology(image: ImageInput, modality: string): Promise<InferenceResponse> {
    return this.post<InferenceResponse>('/api/v1/analyze/radiology', { image, modality });
  }

  async analyzePathology(image: ImageInput): Promise<InferenceResponse> {
    return this.post<InferenceResponse>('/api/v1/analyze/pathology', { image });
  }

  async analyzeDermatology(image: ImageInput): Promise<InferenceResponse> {
    return this.post<InferenceResponse>('/api/v1/analyze/dermatology', { image });
  }

  // ==========================================================================
  // Explainability APIs
  // ==========================================================================

  async getExplanation(diagnosisId: string): Promise<ExplainabilityReport> {
    return this.get<ExplainabilityReport>(`/api/v1/diagnoses/${diagnosisId}/explain`);
  }

  async getFeatureImportance(diagnosisId: string): Promise<{ features: Array<{ name: string; importance: number }> }> {
    return this.get<{ features: Array<{ name: string; importance: number }> }>(`/api/v1/diagnoses/${diagnosisId}/features`);
  }

  async getAttentionMap(diagnosisId: string): Promise<{ heatmap: string }> {
    return this.get<{ heatmap: string }>(`/api/v1/diagnoses/${diagnosisId}/attention`);
  }

  // ==========================================================================
  // Validation APIs
  // ==========================================================================

  validateDiagnosis(diagnosis: DiagnosisResult): { valid: boolean; errors: string[] } {
    const errors: string[] = [];

    if (!diagnosis.diagnosis_id) errors.push('Missing diagnosis ID');
    if (diagnosis.confidence < 0 || diagnosis.confidence > 1) errors.push('Invalid confidence score');
    if (!diagnosis.disease_type) errors.push('Missing disease type');
    if (!diagnosis.model_id) errors.push('Missing model ID');

    return { valid: errors.length === 0, errors };
  }

  calculateConfidenceLevel(confidence: number): 'high' | 'medium' | 'low' {
    if (confidence >= 0.85) return 'high';
    if (confidence >= 0.6) return 'medium';
    return 'low';
  }

  // ==========================================================================
  // Utility Methods
  // ==========================================================================

  static generateDiagnosisId(): string {
    return 'DX-' + Date.now() + '-' + Math.random().toString(36).substr(2, 9).toUpperCase();
  }

  static generateRequestId(): string {
    return 'REQ-' + Date.now() + '-' + Math.random().toString(36).substr(2, 6).toUpperCase();
  }

  // ==========================================================================
  // HTTP Methods
  // ==========================================================================

  private async get<T>(path: string): Promise<T> {
    return this.request<T>('GET', path);
  }

  private async post<T>(path: string, data: unknown): Promise<T> {
    return this.request<T>('POST', path, data);
  }

  private async request<T>(method: string, path: string, data?: unknown): Promise<T> {
    const url = `${this.config.endpoint}${path}`;
    if (this.config.debug) {
      console.log(`[WIA AI Diagnosis] ${method} ${url}`);
    }

    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), this.config.timeout);

    try {
      const response = await fetch(url, {
        method,
        headers: this.headers,
        body: data ? JSON.stringify(data) : undefined,
        signal: controller.signal,
      });
      clearTimeout(timeoutId);

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }

      return response.json();
    } catch (error) {
      clearTimeout(timeoutId);
      throw error;
    }
  }
}

// ============================================================================
// Factory Function
// ============================================================================

export function createClient(config: AIDiagnosisConfig): AIDiagnosisClient {
  return new AIDiagnosisClient(config);
}

// ============================================================================
// Constants
// ============================================================================

export const VERSION = '1.0.0';

export const MODALITIES = {
  XRAY: 'xray',
  CT: 'ct',
  MRI: 'mri',
  ULTRASOUND: 'ultrasound',
  MAMMOGRAPHY: 'mammography',
  PET: 'pet',
} as const;

export const DISEASE_CATEGORIES = {
  ONCOLOGY: 'oncology',
  CARDIOLOGY: 'cardiology',
  NEUROLOGY: 'neurology',
  PULMONOLOGY: 'pulmonology',
  DERMATOLOGY: 'dermatology',
  OPHTHALMOLOGY: 'ophthalmology',
} as const;

export const CONFIDENCE_THRESHOLDS = {
  HIGH: 0.85,
  MEDIUM: 0.6,
  LOW: 0.4,
} as const;

export default AIDiagnosisClient;
