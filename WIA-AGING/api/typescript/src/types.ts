/**
 * WIA-AGING TypeScript Type Definitions
 * @version 1.0.0
 * @license MIT
 * @description Type definitions for the WIA-AGING Standard
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Gender options for subject demographics
 */
export type Gender = 'male' | 'female' | 'other' | 'prefer-not-to-say';

/**
 * Biological age calculation methods
 */
export type BiologicalAgeMethod =
  | 'epigenetic-horvath'
  | 'epigenetic-hannum'
  | 'epigenetic-grimage'
  | 'epigenetic-phenoage'
  | 'phenotypic-levine'
  | 'telomere-length'
  | 'transcriptomic'
  | 'proteomic'
  | 'composite';

/**
 * Biomarker categories
 */
export type BiomarkerCategory =
  | 'inflammatory'
  | 'metabolic'
  | 'organ-function'
  | 'hematological'
  | 'epigenetic'
  | 'telomere'
  | 'hormonal'
  | 'oxidative-stress';

/**
 * Biomarker status based on reference ranges
 */
export type BiomarkerStatus =
  | 'normal'
  | 'low'
  | 'high'
  | 'critical-low'
  | 'critical-high';

/**
 * Intervention types for longevity tracking
 */
export type InterventionType =
  | 'supplement'
  | 'medication'
  | 'exercise'
  | 'diet'
  | 'therapy'
  | 'lifestyle'
  | 'procedure';

/**
 * Message types for real-time protocol
 */
export type MessageType =
  | 'auth.request'
  | 'auth.response'
  | 'biomarker.update'
  | 'biomarker.ack'
  | 'assessment.request'
  | 'assessment.result'
  | 'subscription.add'
  | 'subscription.remove'
  | 'alert.notify'
  | 'heartbeat.ping'
  | 'heartbeat.pong'
  | 'error';

/**
 * Priority levels for messages
 */
export type MessagePriority = 'critical' | 'high' | 'normal' | 'low';

/**
 * Alert levels
 */
export type AlertLevel = 'info' | 'warning' | 'critical';

// ============================================================================
// Subject & Profile Types
// ============================================================================

/**
 * Subject (individual being assessed)
 */
export interface Subject {
  /** Decentralized Identifier (DID) */
  id: string;
  /** Chronological age in years */
  chronologicalAge: number;
  /** Date of birth (optional for privacy) */
  dateOfBirth?: string;
  /** Gender */
  gender?: Gender;
  /** Self-reported ethnicity (optional) */
  ethnicity?: string;
}

/**
 * Laboratory information
 */
export interface Laboratory {
  /** Laboratory identifier */
  id: string;
  /** Laboratory name */
  name: string;
  /** Certification information */
  certification?: string;
  /** Laboratory location */
  location?: string;
}

/**
 * Biological age calculation result
 */
export interface BiologicalAge {
  /** Calculated biological age in years */
  value: number;
  /** Method used for calculation */
  method: BiologicalAgeMethod;
  /** Confidence score (0-1) */
  confidence?: number;
  /** Difference from chronological age */
  ageDifference?: number;
  /** Rate of aging (1.0 = normal) */
  agingRate?: number;
  /** Timestamp of calculation */
  timestamp: string;
  /** Laboratory that performed the analysis */
  laboratory?: Laboratory;
}

/**
 * Reference range for biomarkers
 */
export interface ReferenceRange {
  /** Lower bound */
  low: number;
  /** Upper bound */
  high: number;
  /** Whether range is age-adjusted */
  ageAdjusted?: boolean;
}

/**
 * Biomarker measurement
 */
export interface Biomarker {
  /** LOINC code or WIA biomarker code */
  code: string;
  /** Human-readable name */
  name?: string;
  /** Measured value */
  value: number;
  /** UCUM unit code */
  unit: string;
  /** Reference range */
  referenceRange?: ReferenceRange;
  /** Status based on reference range */
  status?: BiomarkerStatus;
  /** Timestamp of measurement */
  timestamp: string;
  /** Biomarker category */
  category?: BiomarkerCategory;
  /** Source of the measurement */
  source?: string;
}

/**
 * General health metric
 */
export interface HealthMetric {
  /** Metric identifier */
  id: string;
  /** Metric name */
  name: string;
  /** Metric value */
  value: number;
  /** Unit of measurement */
  unit: string;
  /** Timestamp */
  timestamp: string;
  /** Source device or system */
  source?: string;
}

/**
 * Longevity intervention record
 */
export interface Intervention {
  /** Intervention identifier */
  id: string;
  /** Type of intervention */
  type: InterventionType;
  /** Name of intervention */
  name: string;
  /** Dosage (if applicable) */
  dosage?: string;
  /** Frequency */
  frequency?: string;
  /** Start date */
  startDate: string;
  /** End date (if applicable) */
  endDate?: string;
  /** Notes */
  notes?: string;
  /** Active status */
  active: boolean;
}

/**
 * Metadata for profiles and assessments
 */
export interface Metadata {
  /** Standard name */
  standard: string;
  /** Standard version */
  version: string;
  /** Philosophy */
  philosophy: string;
  /** Created timestamp */
  createdAt?: string;
  /** Updated timestamp */
  updatedAt?: string;
  /** Additional custom fields */
  [key: string]: unknown;
}

/**
 * Complete aging profile
 */
export interface AgingProfile {
  /** JSON-LD context */
  '@context': string[];
  /** Profile types */
  type: string[];
  /** Unique identifier (URN) */
  id: string;
  /** Issuer DID */
  issuer?: string;
  /** Issuance date */
  issuanceDate?: string;
  /** Subject information */
  subject: Subject;
  /** Assessment date */
  assessmentDate: string;
  /** Biological age calculation */
  biologicalAge?: BiologicalAge;
  /** Biomarker measurements */
  biomarkers?: Biomarker[];
  /** Health metrics */
  healthMetrics?: HealthMetric[];
  /** Interventions */
  interventions?: Intervention[];
  /** Metadata */
  metadata?: Metadata;
}

// ============================================================================
// API Types
// ============================================================================

/**
 * Pagination parameters
 */
export interface PaginationParams {
  /** Number of items per page */
  limit?: number;
  /** Offset for pagination */
  offset?: number;
}

/**
 * Pagination response
 */
export interface PaginationResponse {
  /** Total number of items */
  total: number;
  /** Current limit */
  limit: number;
  /** Current offset */
  offset: number;
  /** Whether there are more items */
  hasMore: boolean;
}

/**
 * Paginated response wrapper
 */
export interface PaginatedResponse<T> {
  /** Data items */
  data: T[];
  /** Pagination info */
  pagination: PaginationResponse;
}

/**
 * API error details
 */
export interface ApiErrorDetails {
  /** Field that caused the error */
  field?: string;
  /** Invalid value */
  value?: unknown;
  /** Constraint that was violated */
  constraint?: string;
}

/**
 * API error response
 */
export interface ApiError {
  /** Error code */
  code: string;
  /** Error message */
  message: string;
  /** Error details */
  details?: ApiErrorDetails;
  /** Request ID for tracking */
  requestId: string;
}

/**
 * Assessment creation request
 */
export interface CreateAssessmentRequest {
  /** Calculation method */
  method: BiologicalAgeMethod;
  /** Biomarker data */
  biomarkers: Omit<Biomarker, 'status'>[];
}

/**
 * Assessment result with recommendations
 */
export interface AssessmentResult {
  /** Assessment identifier */
  id: string;
  /** Profile identifier */
  profileId: string;
  /** Biological age result */
  biologicalAge: BiologicalAge;
  /** Health score (0-100) */
  healthScore: number;
  /** Interpretation text */
  interpretation: string;
  /** Recommendations */
  recommendations: string[];
  /** Created timestamp */
  createdAt: string;
}

/**
 * Profile creation request
 */
export interface CreateProfileRequest {
  /** Subject information */
  subject: Omit<Subject, 'id'>;
}

/**
 * Profile summary response
 */
export interface ProfileSummary {
  /** Profile identifier */
  id: string;
  /** Subject identifier */
  subjectId: string;
  /** Chronological age */
  chronologicalAge: number;
  /** Latest biological age */
  biologicalAge?: number;
  /** Created timestamp */
  createdAt: string;
  /** Last assessment date */
  lastAssessmentAt?: string;
}

// ============================================================================
// Protocol Types
// ============================================================================

/**
 * Message header
 */
export interface MessageHeader {
  /** Protocol version */
  version: string;
  /** Unique message identifier */
  messageId: string;
  /** Message type */
  type: MessageType;
  /** Timestamp */
  timestamp: string;
  /** Message source */
  source?: {
    id: string;
    type: string;
  };
  /** Message destination */
  destination?: {
    id: string;
    type: string;
  };
  /** Priority level */
  priority?: MessagePriority;
}

/**
 * Security information for messages
 */
export interface MessageSecurity {
  /** Signature */
  signature?: string;
  /** Encryption method */
  encryption?: 'none' | 'AES-256-GCM';
  /** Key identifier (for encrypted messages) */
  keyId?: string;
  /** Initialization vector */
  iv?: string;
}

/**
 * Protocol message
 */
export interface ProtocolMessage<T = unknown> {
  /** Message header */
  header: MessageHeader;
  /** Message payload */
  payload: T;
  /** Security information */
  security?: MessageSecurity;
}

/**
 * Alert notification
 */
export interface Alert {
  /** Alert level */
  level: AlertLevel;
  /** Alert message */
  message: string;
  /** Related biomarker code */
  biomarkerCode?: string;
  /** Alert timestamp */
  timestamp: string;
}

/**
 * Biomarker update event
 */
export interface BiomarkerUpdateEvent {
  /** Profile identifier */
  profileId: string;
  /** Biomarkers to update */
  biomarkers: Biomarker[];
}

/**
 * Biomarker acknowledgment
 */
export interface BiomarkerAck {
  /** Event identifier */
  eventId: string;
  /** Whether biomarkers were accepted */
  accepted: boolean;
  /** Status message */
  status: string;
  /** Any alerts triggered */
  alerts?: Alert[];
}

// ============================================================================
// SDK Configuration
// ============================================================================

/**
 * SDK configuration options
 */
export interface WiaAgingConfig {
  /** API key or access token */
  apiKey: string;
  /** Environment (production or sandbox) */
  environment?: 'production' | 'sandbox';
  /** Base URL override */
  baseUrl?: string;
  /** Request timeout in milliseconds */
  timeout?: number;
  /** Retry configuration */
  retry?: {
    /** Maximum number of retries */
    maxRetries: number;
    /** Base delay between retries */
    baseDelay: number;
  };
}

/**
 * Webhook event types
 */
export type WebhookEventType =
  | 'assessment.completed'
  | 'profile.updated'
  | 'biomarker.alert'
  | 'intervention.reminder';

/**
 * Webhook payload
 */
export interface WebhookPayload<T = unknown> {
  /** Event identifier */
  id: string;
  /** Event type */
  type: WebhookEventType;
  /** Event timestamp */
  timestamp: string;
  /** Event data */
  data: T;
  /** Signature for verification */
  signature: string;
}

// ============================================================================
// Exports
// ============================================================================

export default {
  // Re-export all types for convenience
};
