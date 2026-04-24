/**
 * WIA-MENTAL-HEALTH SDK
 * Mental Health Monitoring Standard Implementation
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Mental Health Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

import {
  MentalHealthAssessment,
  MentalHealthIndex,
  Biomarkers,
  Neuroimaging,
  DigitalPhenotype,
  TreatmentProtocol,
  TreatmentSession,
  PsychedelicSession,
  DigitalTherapeuticSession,
  CrisisScreening,
  SafetyPlan,
  Consent,
  MentalHealthConfig,
  AssessmentRequest,
  AssessmentResponse,
  TreatmentRecommendation,
  ValidationResult,
  ExportOptions,
  Severity,
  CrisisLevel,
} from './types';

export * from './types';

// ============================================================================
// Constants
// ============================================================================

const DEFAULT_ENDPOINT = 'https://api.wia.org/mental-health/v1';
const VERSION = '1.0.0';

/**
 * PHQ-9 scoring thresholds
 */
export const PHQ9_THRESHOLDS = {
  none: { min: 0, max: 4 },
  mild: { min: 5, max: 9 },
  moderate: { min: 10, max: 14 },
  moderatelySevere: { min: 15, max: 19 },
  severe: { min: 20, max: 27 },
} as const;

/**
 * GAD-7 scoring thresholds
 */
export const GAD7_THRESHOLDS = {
  none: { min: 0, max: 4 },
  mild: { min: 5, max: 9 },
  moderate: { min: 10, max: 14 },
  severe: { min: 15, max: 21 },
} as const;

// ============================================================================
// Score Calculation Functions
// ============================================================================

/**
 * Normalize a raw score to 0.0-1.0 range
 */
export function normalizeScore(rawScore: number, maxScore: number): number {
  return Math.max(0, Math.min(1, rawScore / maxScore));
}

/**
 * Calculate PHQ-9 severity from raw score
 */
export function calculatePHQ9Severity(rawScore: number): Severity {
  if (rawScore <= 4) return 'none';
  if (rawScore <= 9) return 'mild';
  if (rawScore <= 14) return 'moderate';
  if (rawScore <= 19) return 'moderately-severe';
  return 'severe';
}

/**
 * Calculate GAD-7 severity from raw score
 */
export function calculateGAD7Severity(rawScore: number): Severity {
  if (rawScore <= 4) return 'none';
  if (rawScore <= 9) return 'mild';
  if (rawScore <= 14) return 'moderate';
  return 'severe';
}

/**
 * Calculate mental health index from assessment responses
 */
export function calculateMentalHealthIndex(
  phq9Responses: number[],
  gad7Responses: number[]
): Partial<MentalHealthIndex> {
  const phq9Raw = phq9Responses.reduce((sum, r) => sum + r, 0);
  const gad7Raw = gad7Responses.reduce((sum, r) => sum + r, 0);

  return {
    depressionScore: {
      value: normalizeScore(phq9Raw, 27),
      rawScore: phq9Raw,
      severity: calculatePHQ9Severity(phq9Raw),
      instrument: 'PHQ-9',
    },
    anxietyScore: {
      value: normalizeScore(gad7Raw, 21),
      rawScore: gad7Raw,
      severity: calculateGAD7Severity(gad7Raw),
      instrument: 'GAD-7',
    },
  };
}

// ============================================================================
// Crisis Assessment Functions
// ============================================================================

/**
 * Determine crisis level from screening responses
 */
export function determineCrisisLevel(
  suicideIdeation: number,
  selfHarmUrges: number,
  substanceIntoxication: boolean,
  psychosisSymptoms: boolean
): CrisisLevel {
  // Imminent risk indicators
  if (suicideIdeation >= 3 || psychosisSymptoms) {
    return 'imminent';
  }

  // High risk
  if (suicideIdeation >= 2 || (selfHarmUrges >= 2 && substanceIntoxication)) {
    return 'high';
  }

  // Moderate risk
  if (suicideIdeation >= 1 || selfHarmUrges >= 2) {
    return 'moderate';
  }

  // Low risk
  if (selfHarmUrges >= 1 || substanceIntoxication) {
    return 'low';
  }

  return 'none';
}

/**
 * Get recommended action based on crisis level
 */
export function getRecommendedCrisisAction(
  crisisLevel: CrisisLevel
): CrisisScreening['recommendedAction'] {
  switch (crisisLevel) {
    case 'imminent':
      return 'emergency_services';
    case 'high':
      return 'crisis_line';
    case 'moderate':
      return 'safety_plan';
    default:
      return 'continue_care';
  }
}

// ============================================================================
// Validation Functions
// ============================================================================

/**
 * Validate mental health assessment data
 */
export function validateAssessment(assessment: Partial<MentalHealthAssessment>): ValidationResult {
  const errors: string[] = [];
  const warnings: string[] = [];

  if (!assessment.id) {
    errors.push('Assessment ID is required');
  }

  if (!assessment.subject?.id) {
    errors.push('Subject ID is required');
  }

  if (!assessment.subject?.consentStatus) {
    errors.push('Consent status must be verified');
  }

  if (assessment.mentalHealthIndex) {
    const mhi = assessment.mentalHealthIndex;

    // Validate score ranges
    const scoreFields = ['depressionScore', 'anxietyScore', 'wellbeingScore', 'resilienceScore'] as const;
    for (const field of scoreFields) {
      if (mhi[field]) {
        if (mhi[field].value < 0 || mhi[field].value > 1) {
          errors.push(`${field}.value must be between 0.0 and 1.0`);
        }
      }
    }
  }

  if (!assessment.timestamp) {
    warnings.push('Timestamp not provided, will use current time');
  }

  return {
    isValid: errors.length === 0,
    errors,
    warnings,
  };
}

/**
 * Validate biomarker data
 */
export function validateBiomarkers(biomarkers: Partial<Biomarkers>): ValidationResult {
  const errors: string[] = [];
  const warnings: string[] = [];

  if (!biomarkers.collectionDate) {
    errors.push('Collection date is required');
  }

  if (!biomarkers.source) {
    errors.push('Data source is required');
  }

  // Validate HRV ranges
  if (biomarkers.heartRateVariability) {
    const hrv = biomarkers.heartRateVariability;
    if (hrv.rmssd && hrv.rmssd.value < 0) {
      errors.push('HRV RMSSD cannot be negative');
    }
    if (hrv.coherenceScore && (hrv.coherenceScore < 0 || hrv.coherenceScore > 1)) {
      errors.push('HRV coherence score must be between 0.0 and 1.0');
    }
  }

  // Validate sleep metrics
  if (biomarkers.sleepMetrics) {
    const sleep = biomarkers.sleepMetrics;
    if (sleep.sleepEfficiency && (sleep.sleepEfficiency < 0 || sleep.sleepEfficiency > 1)) {
      errors.push('Sleep efficiency must be between 0.0 and 1.0');
    }
  }

  return {
    isValid: errors.length === 0,
    errors,
    warnings,
  };
}

/**
 * Validate psychedelic session safety protocol
 */
export function validatePsychedelicSafety(session: Partial<PsychedelicSession>): ValidationResult {
  const errors: string[] = [];
  const warnings: string[] = [];

  if (!session.safetyProtocol?.screeningCompleted) {
    errors.push('Screening must be completed before psychedelic session');
  }

  if (!session.safetyProtocol?.contraindicationsCleared) {
    errors.push('Contraindications must be cleared');
  }

  if (!session.safetyProtocol?.emergencyProtocol) {
    errors.push('Emergency protocol must be documented');
  }

  if (session.phase === 'dosing' && !session.preparationSession?.sessionsCompleted) {
    warnings.push('Preparation sessions should be completed before dosing');
  }

  if (session.dosingSession && !session.safetyProtocol?.followUpScheduled) {
    warnings.push('Follow-up sessions should be scheduled after dosing');
  }

  return {
    isValid: errors.length === 0,
    errors,
    warnings,
  };
}

// ============================================================================
// SDK Client Class
// ============================================================================

/**
 * WIA Mental Health SDK Client
 */
export class MentalHealthClient {
  private config: MentalHealthConfig;
  private endpoint: string;

  constructor(config: MentalHealthConfig = {}) {
    this.config = config;
    this.endpoint = config.endpoint || DEFAULT_ENDPOINT;
  }

  /**
   * Submit a mental health assessment
   */
  async submitAssessment(request: AssessmentRequest): Promise<AssessmentResponse> {
    const validation = this.validateRequest(request);
    if (!validation.isValid) {
      throw new Error(`Invalid assessment request: ${validation.errors.join(', ')}`);
    }

    const response = await this.fetch('/assessments', {
      method: 'POST',
      body: JSON.stringify(request),
    });

    return response as AssessmentResponse;
  }

  /**
   * Get assessment history for a subject
   */
  async getAssessmentHistory(
    subjectId: string,
    options?: { startDate?: Date; endDate?: Date; limit?: number }
  ): Promise<MentalHealthAssessment[]> {
    const params = new URLSearchParams();
    if (options?.startDate) params.set('start_date', options.startDate.toISOString());
    if (options?.endDate) params.set('end_date', options.endDate.toISOString());
    if (options?.limit) params.set('limit', options.limit.toString());

    const response = await this.fetch(`/assessments/history/${subjectId}?${params}`);
    return response.assessments as MentalHealthAssessment[];
  }

  /**
   * Submit crisis screening
   */
  async submitCrisisScreen(screening: Omit<CrisisScreening, 'crisisLevel' | 'recommendedAction' | 'resources'>): Promise<CrisisScreening> {
    const crisisLevel = determineCrisisLevel(
      screening.responses.suicideIdeation,
      screening.responses.selfHarmUrges,
      screening.responses.substanceIntoxication,
      screening.responses.psychosisSymptoms
    );

    const response = await this.fetch('/assessments/crisis-screen', {
      method: 'POST',
      body: JSON.stringify({
        ...screening,
        crisisLevel,
        recommendedAction: getRecommendedCrisisAction(crisisLevel),
      }),
    });

    return response as CrisisScreening;
  }

  /**
   * Get treatment recommendations
   */
  async getTreatmentRecommendations(
    subjectId: string,
    preferences?: Record<string, unknown>
  ): Promise<TreatmentRecommendation[]> {
    const response = await this.fetch('/treatments/recommend', {
      method: 'POST',
      body: JSON.stringify({ subjectId, preferences }),
    });

    return response.recommendations as TreatmentRecommendation[];
  }

  /**
   * Log a treatment session
   */
  async logTreatmentSession(session: TreatmentSession): Promise<{ sessionId: string }> {
    const response = await this.fetch('/treatments/sessions', {
      method: 'POST',
      body: JSON.stringify(session),
    });

    return response as { sessionId: string };
  }

  /**
   * Submit biomarker data
   */
  async submitBiomarkers(subjectId: string, biomarkers: Biomarkers): Promise<{ recordId: string }> {
    const validation = validateBiomarkers(biomarkers);
    if (!validation.isValid) {
      throw new Error(`Invalid biomarker data: ${validation.errors.join(', ')}`);
    }

    const response = await this.fetch('/biomarkers', {
      method: 'POST',
      body: JSON.stringify({ subjectId, ...biomarkers }),
    });

    return response as { recordId: string };
  }

  /**
   * Create or update safety plan
   */
  async saveSafetyPlan(plan: SafetyPlan): Promise<{ planId: string }> {
    const response = await this.fetch('/safety-plans', {
      method: 'POST',
      body: JSON.stringify(plan),
    });

    return response as { planId: string };
  }

  /**
   * Export data in specified format
   */
  async exportData(
    subjectId: string,
    options: ExportOptions
  ): Promise<Blob | Record<string, unknown>> {
    const response = await this.fetch(`/export/${subjectId}`, {
      method: 'POST',
      body: JSON.stringify(options),
    });

    return response;
  }

  private validateRequest(request: AssessmentRequest): ValidationResult {
    const errors: string[] = [];

    if (!request.subjectId) {
      errors.push('Subject ID is required');
    }

    if (!request.instruments || request.instruments.length === 0) {
      errors.push('At least one assessment instrument is required');
    }

    if (!request.responses || Object.keys(request.responses).length === 0) {
      errors.push('Assessment responses are required');
    }

    return {
      isValid: errors.length === 0,
      errors,
      warnings: [],
    };
  }

  private async fetch(path: string, options: RequestInit = {}): Promise<unknown> {
    const url = `${this.endpoint}${path}`;
    const headers: Record<string, string> = {
      'Content-Type': 'application/json',
      'X-WIA-Standard': `MENTAL-HEALTH-v${VERSION}`,
      ...(options.headers as Record<string, string>),
    };

    if (this.config.apiKey) {
      headers['Authorization'] = `Bearer ${this.config.apiKey}`;
    }

    const response = await fetch(url, {
      ...options,
      headers,
    });

    if (!response.ok) {
      throw new Error(`API request failed: ${response.status} ${response.statusText}`);
    }

    return response.json();
  }
}

// ============================================================================
// Factory Functions
// ============================================================================

/**
 * Create a new Mental Health SDK client
 */
export function createClient(config?: MentalHealthConfig): MentalHealthClient {
  return new MentalHealthClient(config);
}

/**
 * Create an empty safety plan template
 */
export function createSafetyPlanTemplate(subjectId: string): SafetyPlan {
  return {
    id: crypto.randomUUID(),
    subjectId,
    warningSignsIdentified: [],
    copingStrategies: [],
    socialSupports: [],
    professionalContacts: [
      {
        provider: '988 Suicide & Crisis Lifeline',
        phone: '988',
        crisisLine: '988',
      },
    ],
    meansRestriction: {
      lethalMeansIdentified: [],
      restrictionPlan: '',
    },
    reasonsForLiving: [],
    lastReviewed: new Date(),
  };
}

// ============================================================================
// Default Export
// ============================================================================

export default {
  createClient,
  createSafetyPlanTemplate,
  calculateMentalHealthIndex,
  determineCrisisLevel,
  getRecommendedCrisisAction,
  validateAssessment,
  validateBiomarkers,
  validatePsychedelicSafety,
  normalizeScore,
  calculatePHQ9Severity,
  calculateGAD7Severity,
  PHQ9_THRESHOLDS,
  GAD7_THRESHOLDS,
  VERSION,
};
