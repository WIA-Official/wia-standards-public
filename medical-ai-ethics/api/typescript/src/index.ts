/**
 * WIA Medical AI Ethics Standard - TypeScript SDK
 * Version: 1.0
 *
 * Philosophy: 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * This SDK provides a unified interface for ethical AI review in healthcare,
 * ensuring medical AI systems meet ethics standards for patient safety and fairness.
 *
 * © 2025 SmileStory Inc. / WIA
 */

import EventEmitter from 'eventemitter3';
import {
  WIAMedicalAIEthicsConfig,
  APIResponse,
  CreateReviewRequest,
  CreateReviewResponse,
  SubmitReviewRequest,
  SubmitReviewResponse,
  GetReviewRequest,
  GetReviewResponse,
  ListReviewsRequest,
  ListReviewsResponse,
  EthicsReview,
  EthicsScore,
  BiasMetrics,
  TransparencyAssessment,
  PrivacyAssessment,
  MonitoringEvent,
  AuditLogEntry,
  IntegrationStatus,
  ComplianceCertificate,
} from './types';

// Re-export all types
export * from './types';

// ============================================================================
// WIA Medical AI Ethics Client - Main SDK Class
// ============================================================================

/**
 * Main WIA Medical AI Ethics Client
 *
 * Provides methods for:
 * - Creating and managing ethics reviews
 * - Submitting ethics assessments
 * - Calculating ethics scores
 * - Monitoring AI models for ethical compliance
 * - Generating compliance certificates
 *
 * @example
 * ```typescript
 * const client = new WIAMedicalAIEthicsClient({
 *   apiKey: 'wia_sk_abc123'
 * });
 *
 * const review = await client.createReview({
 *   model_id: 'diagnosis_model_v1',
 *   type: 'pre-deployment',
 *   domain: 'diagnosis',
 *   risk_level: 'high',
 *   principles: ['beneficence', 'non-maleficence', 'justice']
 * });
 *
 * console.log(review.data.review.id);
 * ```
 */
export class WIAMedicalAIEthicsClient {
  private config: Required<WIAMedicalAIEthicsConfig>;
  private eventEmitter = new EventEmitter();

  constructor(config: WIAMedicalAIEthicsConfig) {
    this.config = {
      baseURL: 'https://api.wia-medical-ai-ethics.org/v1',
      timeout: 60000,
      debug: false,
      headers: {},
      ...config,
    };

    if (!this.config.apiKey) {
      throw new Error('API key is required');
    }

    if (this.config.debug) {
      console.log('[WIA Medical AI Ethics] Initialized with config:', {
        baseURL: this.config.baseURL,
      });
    }
  }

  // ==========================================================================
  // Ethics Review Operations
  // ==========================================================================

  /**
   * Create a new ethics review
   *
   * @param request - Review creation parameters
   * @returns Promise resolving to the created review
   *
   * @example
   * ```typescript
   * const review = await client.createReview({
   *   model_id: 'cancer_detection_v2',
   *   type: 'pre-deployment',
   *   domain: 'diagnosis',
   *   risk_level: 'critical',
   *   principles: ['beneficence', 'non-maleficence', 'justice', 'transparency']
   * });
   * ```
   */
  async createReview(
    request: CreateReviewRequest
  ): Promise<APIResponse<CreateReviewResponse>> {
    if (this.config.debug) {
      console.log('[WIA Medical AI Ethics] Creating ethics review:', request);
    }

    const response = await this.makeRequest<CreateReviewResponse>(
      'POST',
      '/reviews',
      request
    );

    if (response.success && this.config.debug) {
      console.log('[WIA Medical AI Ethics] Review created:', response.data?.review.id);
    }

    return response;
  }

  /**
   * Submit an ethics review with assessment
   *
   * @param request - Review submission with assessments
   * @returns Promise resolving to submission result
   *
   * @example
   * ```typescript
   * const result = await client.submitReview({
   *   review_id: 'REVIEW-ABC123',
   *   comments: 'Model demonstrates strong ethical compliance',
   *   recommendation: 'approve',
   *   priority: 'high',
   *   principles_assessment: {
   *     beneficence: { compliant: true, score: 95 },
   *     non_maleficence: { compliant: true, score: 92 },
   *     autonomy: { compliant: true, score: 88 },
   *     justice: { compliant: true, score: 91 }
   *   }
   * });
   * ```
   */
  async submitReview(
    request: SubmitReviewRequest
  ): Promise<APIResponse<SubmitReviewResponse>> {
    if (this.config.debug) {
      console.log('[WIA Medical AI Ethics] Submitting review:', request.review_id);
    }

    const response = await this.makeRequest<SubmitReviewResponse>(
      'POST',
      `/reviews/${request.review_id}/submit`,
      request
    );

    if (response.success) {
      this.eventEmitter.emit('review:submitted', response.data);
    }

    return response;
  }

  /**
   * Get an ethics review by ID
   *
   * @param reviewId - Review identifier
   * @returns Promise resolving to the review details
   */
  async getReview(reviewId: string): Promise<APIResponse<GetReviewResponse>> {
    if (this.config.debug) {
      console.log('[WIA Medical AI Ethics] Fetching review:', reviewId);
    }

    return await this.makeRequest<GetReviewResponse>('GET', `/reviews/${reviewId}`);
  }

  /**
   * List ethics reviews with optional filters
   *
   * @param request - Filter parameters
   * @returns Promise resolving to list of reviews
   */
  async listReviews(
    request?: ListReviewsRequest
  ): Promise<APIResponse<ListReviewsResponse>> {
    if (this.config.debug) {
      console.log('[WIA Medical AI Ethics] Listing reviews with filters:', request);
    }

    const queryParams = request ? new URLSearchParams(request as any).toString() : '';
    const url = `/reviews${queryParams ? '?' + queryParams : ''}`;

    return await this.makeRequest<ListReviewsResponse>('GET', url);
  }

  // ==========================================================================
  // Ethics Scoring & Assessment
  // ==========================================================================

  /**
   * Calculate ethics score for a model
   *
   * @param modelId - Model identifier
   * @param assessments - Assessment data
   * @returns Promise resolving to ethics score
   */
  async calculateEthicsScore(
    modelId: string,
    assessments: {
      bias_metrics: BiasMetrics;
      transparency: TransparencyAssessment;
      privacy: PrivacyAssessment;
    }
  ): Promise<APIResponse<EthicsScore>> {
    if (this.config.debug) {
      console.log('[WIA Medical AI Ethics] Calculating ethics score for:', modelId);
    }

    return await this.makeRequest<EthicsScore>('POST', '/scores/calculate', {
      model_id: modelId,
      ...assessments,
    });
  }

  /**
   * Evaluate bias metrics for a model
   *
   * @param modelId - Model identifier
   * @param testData - Test dataset information
   * @returns Promise resolving to bias metrics
   */
  async evaluateBias(
    modelId: string,
    testData: {
      dataset: string;
      demographics: string[];
    }
  ): Promise<APIResponse<BiasMetrics>> {
    if (this.config.debug) {
      console.log('[WIA Medical AI Ethics] Evaluating bias for:', modelId);
    }

    return await this.makeRequest<BiasMetrics>('POST', '/bias/evaluate', {
      model_id: modelId,
      ...testData,
    });
  }

  // ==========================================================================
  // Monitoring & Auditing
  // ==========================================================================

  /**
   * Start real-time monitoring for a model
   *
   * @param modelId - Model identifier
   * @param callback - Event callback function
   */
  startMonitoring(
    modelId: string,
    callback: (event: MonitoringEvent) => void
  ): void {
    if (this.config.debug) {
      console.log('[WIA Medical AI Ethics] Starting monitoring for:', modelId);
    }

    // In production, this would establish WebSocket connection
    this.eventEmitter.on(`monitoring:${modelId}`, callback);

    // Simulate monitoring events for demo
    const interval = setInterval(() => {
      const event: MonitoringEvent = {
        event_id: 'EVT-' + Date.now(),
        timestamp: new Date().toISOString(),
        model_id: modelId,
        event_type: 'bias_detected',
        severity: 'low',
        details: {
          metric: 'demographic_parity',
          value: 0.89,
        },
      };
      this.eventEmitter.emit(`monitoring:${modelId}`, event);
    }, 30000);

    // Store interval for cleanup
    (this as any)[`interval_${modelId}`] = interval;
  }

  /**
   * Stop monitoring for a model
   *
   * @param modelId - Model identifier
   */
  stopMonitoring(modelId: string): void {
    if (this.config.debug) {
      console.log('[WIA Medical AI Ethics] Stopping monitoring for:', modelId);
    }

    this.eventEmitter.removeAllListeners(`monitoring:${modelId}`);

    const interval = (this as any)[`interval_${modelId}`];
    if (interval) {
      clearInterval(interval);
      delete (this as any)[`interval_${modelId}`];
    }
  }

  /**
   * Get audit logs for a review
   *
   * @param reviewId - Review identifier
   * @returns Promise resolving to audit logs
   */
  async getAuditLogs(reviewId: string): Promise<APIResponse<AuditLogEntry[]>> {
    if (this.config.debug) {
      console.log('[WIA Medical AI Ethics] Fetching audit logs for:', reviewId);
    }

    return await this.makeRequest<AuditLogEntry[]>(
      'GET',
      `/reviews/${reviewId}/audit-logs`
    );
  }

  // ==========================================================================
  // Integration & Certification
  // ==========================================================================

  /**
   * Test integration with healthcare systems
   *
   * @returns Promise resolving to integration status
   */
  async testIntegration(): Promise<APIResponse<IntegrationStatus[]>> {
    if (this.config.debug) {
      console.log('[WIA Medical AI Ethics] Testing system integration');
    }

    return await this.makeRequest<IntegrationStatus[]>('GET', '/integration/status');
  }

  /**
   * Generate compliance certificate
   *
   * @param modelId - Model identifier
   * @returns Promise resolving to certificate
   */
  async generateCertificate(
    modelId: string
  ): Promise<APIResponse<ComplianceCertificate>> {
    if (this.config.debug) {
      console.log('[WIA Medical AI Ethics] Generating certificate for:', modelId);
    }

    return await this.makeRequest<ComplianceCertificate>(
      'POST',
      '/certificates/generate',
      { model_id: modelId }
    );
  }

  /**
   * Verify a compliance certificate
   *
   * @param certificateId - Certificate identifier
   * @returns Promise resolving to verification result
   */
  async verifyCertificate(
    certificateId: string
  ): Promise<APIResponse<{ valid: boolean; certificate?: ComplianceCertificate }>> {
    if (this.config.debug) {
      console.log('[WIA Medical AI Ethics] Verifying certificate:', certificateId);
    }

    return await this.makeRequest<{ valid: boolean; certificate?: ComplianceCertificate }>(
      'GET',
      `/certificates/${certificateId}/verify`
    );
  }

  // ==========================================================================
  // Event Handling
  // ==========================================================================

  /**
   * Register event listener
   *
   * @param event - Event name
   * @param callback - Event callback
   */
  on(event: string, callback: (...args: any[]) => void): void {
    this.eventEmitter.on(event, callback);
  }

  /**
   * Unregister event listener
   *
   * @param event - Event name
   * @param callback - Event callback
   */
  off(event: string, callback: (...args: any[]) => void): void {
    this.eventEmitter.off(event, callback);
  }

  // ==========================================================================
  // Internal Methods
  // ==========================================================================

  /**
   * Make HTTP request to API
   */
  private async makeRequest<T>(
    method: string,
    path: string,
    body?: any
  ): Promise<APIResponse<T>> {
    const url = `${this.config.baseURL}${path}`;

    if (this.config.debug) {
      console.log(`[WIA Medical AI Ethics] ${method} ${url}`, body);
    }

    try {
      // In production, this would make actual HTTP request
      // For now, return mock response
      return {
        success: true,
        data: body as T,
        metadata: {
          request_id: 'req_' + Date.now(),
          timestamp: new Date().toISOString(),
          version: '1.0',
        },
      };
    } catch (error: any) {
      if (this.config.debug) {
        console.error('[WIA Medical AI Ethics] Request failed:', error);
      }

      return {
        success: false,
        error: {
          code: 'REQUEST_FAILED',
          message: error.message || 'Unknown error',
          details: error,
        },
      };
    }
  }
}

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * Validate ethics review data
 */
export function validateReview(review: EthicsReview): {
  valid: boolean;
  errors: string[];
} {
  const errors: string[] = [];

  if (!review.id) errors.push('Review ID is required');
  if (!review.model_id) errors.push('Model ID is required');
  if (!review.type) errors.push('Review type is required');
  if (!review.domain) errors.push('Medical domain is required');
  if (!review.risk_level) errors.push('Risk level is required');

  return {
    valid: errors.length === 0,
    errors,
  };
}

/**
 * Calculate overall ethics score from components
 */
export function calculateOverallScore(scores: {
  bias_fairness: number;
  transparency: number;
  privacy: number;
  clinical_safety: number;
  accountability: number;
}): number {
  const weights = {
    bias_fairness: 0.25,
    transparency: 0.20,
    privacy: 0.20,
    clinical_safety: 0.25,
    accountability: 0.10,
  };

  return Math.round(
    scores.bias_fairness * weights.bias_fairness +
      scores.transparency * weights.transparency +
      scores.privacy * weights.privacy +
      scores.clinical_safety * weights.clinical_safety +
      scores.accountability * weights.accountability
  );
}
