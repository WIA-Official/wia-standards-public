/**
 * WIA Medical AI Ethics Standard - TypeScript Types
 * Version: 1.0
 *
 * Philosophy: 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * This module defines TypeScript types for the WIA Medical AI Ethics Standard,
 * enabling ethical AI development in healthcare.
 *
 * © 2025 SmileStory Inc. / WIA
 */

// ============================================================================
// Core Types (Phase 1: Data Format)
// ============================================================================

/**
 * Review type enumeration
 */
export type ReviewType =
  | 'pre-deployment'
  | 'periodic'
  | 'incident'
  | 'audit'
  | 'post-market';

/**
 * Medical domain enumeration
 */
export type MedicalDomain =
  | 'diagnosis'
  | 'treatment'
  | 'prognosis'
  | 'imaging'
  | 'monitoring'
  | 'surgery'
  | 'research';

/**
 * Risk level enumeration
 */
export type RiskLevel = 'low' | 'medium' | 'high' | 'critical';

/**
 * Ethics principle enumeration
 */
export type EthicsPrinciple =
  | 'beneficence'
  | 'non-maleficence'
  | 'autonomy'
  | 'justice'
  | 'transparency'
  | 'privacy'
  | 'accountability';

/**
 * Review status enumeration
 */
export type ReviewStatus =
  | 'draft'
  | 'submitted'
  | 'under_review'
  | 'approved'
  | 'conditional'
  | 'rejected'
  | 'expired';

/**
 * Certification level enumeration
 */
export type CertificationLevel = 'bronze' | 'silver' | 'gold' | 'platinum';

/**
 * Ethics review metadata
 */
export interface EthicsReview {
  /** Unique review identifier */
  id: string;
  /** AI model being reviewed */
  model_id: string;
  /** Review type */
  type: ReviewType;
  /** Medical domain */
  domain: MedicalDomain;
  /** Risk level assessment */
  risk_level: RiskLevel;
  /** Review status */
  status: ReviewStatus;
  /** ISO 8601 timestamp */
  timestamp: string;
  /** Reviewer information */
  reviewer?: ReviewerInfo;
  /** Review metadata */
  metadata?: Record<string, any>;
}

/**
 * Reviewer information
 */
export interface ReviewerInfo {
  /** Reviewer ID */
  id: string;
  /** Reviewer name */
  name?: string;
  /** Reviewer credentials */
  credentials?: string[];
  /** Organization */
  organization?: string;
}

/**
 * Ethics principles assessment
 */
export interface PrinciplesAssessment {
  /** Beneficence (doing good) */
  beneficence: {
    compliant: boolean;
    score: number;
    notes?: string;
  };
  /** Non-maleficence (do no harm) */
  non_maleficence: {
    compliant: boolean;
    score: number;
    notes?: string;
  };
  /** Autonomy (patient choice) */
  autonomy: {
    compliant: boolean;
    score: number;
    notes?: string;
  };
  /** Justice (fairness) */
  justice: {
    compliant: boolean;
    score: number;
    notes?: string;
  };
  /** Transparency (explainability) */
  transparency?: {
    compliant: boolean;
    score: number;
    notes?: string;
  };
  /** Privacy (data protection) */
  privacy?: {
    compliant: boolean;
    score: number;
    notes?: string;
  };
  /** Accountability */
  accountability?: {
    compliant: boolean;
    score: number;
    notes?: string;
  };
}

// ============================================================================
// Bias & Fairness Assessment (Phase 2: Algorithms)
// ============================================================================

/**
 * Bias metrics
 */
export interface BiasMetrics {
  /** Demographic parity score (0-1) */
  demographic_parity: number;
  /** Equal opportunity score (0-1) */
  equal_opportunity: number;
  /** Calibration score (0-1) */
  calibration: number;
  /** Overall fairness score (0-100) */
  overall_fairness_score: number;
  /** Detailed metrics */
  detailed_metrics?: Record<string, any>;
}

/**
 * Transparency assessment
 */
export interface TransparencyAssessment {
  /** Explainability score (0-100) */
  explainability_score: number;
  /** Model interpretability */
  interpretability: 'low' | 'medium' | 'high';
  /** Documentation completeness (0-100) */
  documentation_completeness: number;
  /** Clinical validation status */
  clinical_validation: boolean;
}

/**
 * Privacy assessment
 */
export interface PrivacyAssessment {
  /** Privacy protection score (0-100) */
  privacy_score: number;
  /** Data anonymization status */
  anonymization: boolean;
  /** Encryption status */
  encryption: boolean;
  /** Consent mechanism */
  consent_mechanism: 'explicit' | 'implicit' | 'none';
  /** HIPAA compliance */
  hipaa_compliant?: boolean;
  /** GDPR compliance */
  gdpr_compliant?: boolean;
}

/**
 * Overall ethics score
 */
export interface EthicsScore {
  /** Overall score (0-100) */
  overall_score: number;
  /** Score breakdown */
  breakdown: {
    bias_fairness: number;
    transparency: number;
    privacy: number;
    clinical_safety: number;
    accountability: number;
  };
  /** Certification level */
  certification_level: CertificationLevel;
  /** Pass/fail recommendation */
  recommendation: 'approved' | 'conditional' | 'needs_improvement' | 'rejected';
}

// ============================================================================
// API Interfaces (Phase 2: API)
// ============================================================================

/**
 * Create review request
 */
export interface CreateReviewRequest {
  model_id: string;
  type: ReviewType;
  domain: MedicalDomain;
  risk_level: RiskLevel;
  principles: EthicsPrinciple[];
  reviewer_id?: string;
  metadata?: Record<string, any>;
}

/**
 * Create review response
 */
export interface CreateReviewResponse {
  success: boolean;
  review: EthicsReview;
  message?: string;
}

/**
 * Submit review request
 */
export interface SubmitReviewRequest {
  review_id: string;
  comments: string;
  recommendation: 'approve' | 'conditional' | 'revise' | 'reject';
  priority: 'low' | 'medium' | 'high' | 'urgent';
  principles_assessment: PrinciplesAssessment;
  bias_metrics?: BiasMetrics;
  transparency_assessment?: TransparencyAssessment;
  privacy_assessment?: PrivacyAssessment;
}

/**
 * Submit review response
 */
export interface SubmitReviewResponse {
  success: boolean;
  review_id: string;
  status: ReviewStatus;
  ethics_score?: EthicsScore;
  message?: string;
}

/**
 * Get review request
 */
export interface GetReviewRequest {
  review_id: string;
}

/**
 * Get review response
 */
export interface GetReviewResponse {
  success: boolean;
  review: EthicsReview;
  ethics_score?: EthicsScore;
  message?: string;
}

/**
 * List reviews request
 */
export interface ListReviewsRequest {
  model_id?: string;
  status?: ReviewStatus;
  domain?: MedicalDomain;
  risk_level?: RiskLevel;
  limit?: number;
  offset?: number;
}

/**
 * List reviews response
 */
export interface ListReviewsResponse {
  success: boolean;
  reviews: EthicsReview[];
  total: number;
  limit: number;
  offset: number;
  message?: string;
}

// ============================================================================
// Protocol (Phase 3: Real-time Monitoring)
// ============================================================================

/**
 * Monitoring event
 */
export interface MonitoringEvent {
  event_id: string;
  timestamp: string;
  model_id: string;
  event_type: 'bias_detected' | 'fairness_issue' | 'privacy_breach' | 'safety_alert';
  severity: 'low' | 'medium' | 'high' | 'critical';
  details: Record<string, any>;
}

/**
 * Audit log entry
 */
export interface AuditLogEntry {
  log_id: string;
  timestamp: string;
  review_id: string;
  action: string;
  actor_id: string;
  details?: Record<string, any>;
}

// ============================================================================
// Integration (Phase 4: System Integration)
// ============================================================================

/**
 * Integration status
 */
export interface IntegrationStatus {
  endpoint: string;
  system: string;
  status: 'connected' | 'pending' | 'failed';
  last_check: string;
  latency_ms?: number;
}

/**
 * Compliance certificate
 */
export interface ComplianceCertificate {
  certificate_id: string;
  model_id: string;
  certification_level: CertificationLevel;
  issue_date: string;
  expiry_date: string;
  ethics_score: EthicsScore;
  qr_code?: string;
  verification_url?: string;
}

// ============================================================================
// SDK Configuration
// ============================================================================

/**
 * WIA Medical AI Ethics client configuration
 */
export interface WIAMedicalAIEthicsConfig {
  /** API key */
  apiKey: string;
  /** Base URL for the API */
  baseURL?: string;
  /** Request timeout in milliseconds */
  timeout?: number;
  /** Enable debug logging */
  debug?: boolean;
  /** Custom headers */
  headers?: Record<string, string>;
}

/**
 * API response wrapper
 */
export interface APIResponse<T> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
    details?: any;
  };
  metadata?: {
    request_id: string;
    timestamp: string;
    version: string;
  };
}
