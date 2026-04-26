/**
 * WIA-MED-015: Clinical Decision Support Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @packageDocumentation
 * @module wia-clinical-decision-support
 * @license MIT
 * @author WIA / SmileStory Inc.
 * @version 1.0.0
 */

export * from './types';

import {
  PatientInfo,
  InteractionCheckRequest,
  InteractionCheckResponse,
  DiagnosticSuggestionRequest,
  DiagnosticSuggestionResponse,
  TreatmentRecommendationRequest,
  TreatmentRecommendationResponse,
  RiskAssessmentRequest,
  RiskAssessmentResponse,
  APIResponse,
  ClinicalAlert,
  AlertSeverity,
  GuidelineReference,
  DrugInfo,
  LabResult,
  PaginatedResponse,
} from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface CDSConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  retries?: number;
  debug?: boolean;
}

// ============================================================================
// SDK Client
// ============================================================================

export class ClinicalDecisionSupportClient {
  private config: Required<CDSConfig>;
  private headers: Record<string, string>;

  constructor(config: CDSConfig) {
    this.config = {
      timeout: 30000,
      retries: 3,
      debug: false,
      ...config,
    };

    this.headers = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'X-WIA-Standard': 'MED-015',
      'X-WIA-Version': '1.0.0',
    };
  }

  // ==========================================================================
  // Interaction Check APIs
  // ==========================================================================

  async checkDrugInteractions(request: InteractionCheckRequest): Promise<APIResponse<InteractionCheckResponse>> {
    return this.post<InteractionCheckResponse>('/api/v1/interactions/check', request);
  }

  async getDrugInfo(drugCode: string): Promise<APIResponse<DrugInfo>> {
    return this.get<DrugInfo>(`/api/v1/drugs/${drugCode}`);
  }

  async searchDrugs(query: string): Promise<APIResponse<DrugInfo[]>> {
    return this.get<DrugInfo[]>(`/api/v1/drugs?q=${encodeURIComponent(query)}`);
  }

  // ==========================================================================
  // Diagnostic APIs
  // ==========================================================================

  async suggestDiagnosis(request: DiagnosticSuggestionRequest): Promise<APIResponse<DiagnosticSuggestionResponse>> {
    return this.post<DiagnosticSuggestionResponse>('/api/v1/diagnosis/suggest', request);
  }

  async getDifferentialDiagnosis(symptoms: string[]): Promise<APIResponse<DiagnosticSuggestionResponse>> {
    return this.post<DiagnosticSuggestionResponse>('/api/v1/diagnosis/differential', { symptoms });
  }

  // ==========================================================================
  // Treatment APIs
  // ==========================================================================

  async recommendTreatment(request: TreatmentRecommendationRequest): Promise<APIResponse<TreatmentRecommendationResponse>> {
    return this.post<TreatmentRecommendationResponse>('/api/v1/treatment/recommend', request);
  }

  async getTreatmentGuidelines(conditionCode: string): Promise<APIResponse<GuidelineReference[]>> {
    return this.get<GuidelineReference[]>(`/api/v1/guidelines/${conditionCode}`);
  }

  // ==========================================================================
  // Risk Assessment APIs
  // ==========================================================================

  async assessRisk(request: RiskAssessmentRequest): Promise<APIResponse<RiskAssessmentResponse>> {
    return this.post<RiskAssessmentResponse>('/api/v1/risk/assess', request);
  }

  async calculateScore(scoreType: string, parameters: Record<string, unknown>): Promise<APIResponse<{ score: number; interpretation: string }>> {
    return this.post<{ score: number; interpretation: string }>(`/api/v1/scores/${scoreType}`, parameters);
  }

  // ==========================================================================
  // Alert APIs
  // ==========================================================================

  async getPatientAlerts(patientId: string): Promise<APIResponse<ClinicalAlert[]>> {
    return this.get<ClinicalAlert[]>(`/api/v1/patients/${patientId}/alerts`);
  }

  async acknowledgeAlert(alertId: string): Promise<APIResponse<ClinicalAlert>> {
    return this.post<ClinicalAlert>(`/api/v1/alerts/${alertId}/acknowledge`, {});
  }

  async dismissAlert(alertId: string, reason: string): Promise<APIResponse<ClinicalAlert>> {
    return this.post<ClinicalAlert>(`/api/v1/alerts/${alertId}/dismiss`, { reason });
  }

  // ==========================================================================
  // Lab Result APIs
  // ==========================================================================

  async interpretLabResult(result: LabResult): Promise<APIResponse<{ interpretation: string; alerts: ClinicalAlert[] }>> {
    return this.post<{ interpretation: string; alerts: ClinicalAlert[] }>('/api/v1/labs/interpret', result);
  }

  async checkCriticalValues(results: LabResult[]): Promise<APIResponse<ClinicalAlert[]>> {
    return this.post<ClinicalAlert[]>('/api/v1/labs/critical-check', { results });
  }

  // ==========================================================================
  // Utility Methods
  // ==========================================================================

  calculateInteractionSeverity(drug1: string, drug2: string): AlertSeverity {
    const criticalPairs = [
      ['warfarin', 'aspirin'],
      ['digoxin', 'amiodarone'],
      ['methotrexate', 'nsaid'],
      ['lithium', 'ibuprofen'],
      ['ssri', 'maoi'],
    ];

    const normalized1 = drug1.toLowerCase();
    const normalized2 = drug2.toLowerCase();

    const isCritical = criticalPairs.some(pair =>
      (pair[0] === normalized1 && pair[1] === normalized2) ||
      (pair[0] === normalized2 && pair[1] === normalized1)
    );

    return isCritical ? AlertSeverity.CRITICAL : AlertSeverity.MEDIUM;
  }

  // ==========================================================================
  // HTTP Methods
  // ==========================================================================

  private async get<T>(path: string): Promise<APIResponse<T>> {
    return this.request<T>('GET', path);
  }

  private async post<T>(path: string, data: unknown): Promise<APIResponse<T>> {
    return this.request<T>('POST', path, data);
  }

  private async request<T>(path: string, method: string, data?: unknown): Promise<APIResponse<T>> {
    const url = `${this.config.endpoint}${path}`;
    if (this.config.debug) {
      console.log(`[WIA CDS] ${method} ${url}`);
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
      return response.json();
    } catch (error) {
      clearTimeout(timeoutId);
      return {
        success: false,
        error: error instanceof Error ? error.message : 'Request failed',
        timestamp: new Date().toISOString(),
      };
    }
  }
}

// ============================================================================
// Factory Function
// ============================================================================

export function createClient(config: CDSConfig): ClinicalDecisionSupportClient {
  return new ClinicalDecisionSupportClient(config);
}

// ============================================================================
// Constants
// ============================================================================

export const VERSION = '1.0.0';

export const ALERT_SEVERITIES = {
  CRITICAL: AlertSeverity.CRITICAL,
  HIGH: AlertSeverity.HIGH,
  MEDIUM: AlertSeverity.MEDIUM,
  LOW: AlertSeverity.LOW,
  INFO: AlertSeverity.INFO,
} as const;

export const CLINICAL_DOMAINS = {
  CARDIOLOGY: 'cardiology',
  ONCOLOGY: 'oncology',
  INFECTIOUS_DISEASE: 'infectious-disease',
  NEPHROLOGY: 'nephrology',
  NEUROLOGY: 'neurology',
} as const;

export default ClinicalDecisionSupportClient;
