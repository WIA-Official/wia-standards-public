/**
 * WIA-CORE-002: Universal Consent - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Privacy & Compliance Working Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Enums
// ============================================================================

/**
 * Consent status values
 */
export enum ConsentStatus {
  PENDING = 'pending',
  GRANTED = 'granted',
  DENIED = 'denied',
  REVOKED = 'revoked',
  EXPIRED = 'expired',
  RENEWED = 'renewed',
}

/**
 * GDPR legal bases for data processing (Article 6)
 */
export enum LegalBasis {
  CONSENT = 'consent',              // Art. 6(1)(a) - Explicit consent
  CONTRACT = 'contract',            // Art. 6(1)(b) - Contractual necessity
  LEGAL_OBLIGATION = 'legal',       // Art. 6(1)(c) - Legal requirement
  VITAL_INTERESTS = 'vital',        // Art. 6(1)(d) - Protection of vital interests
  PUBLIC_TASK = 'public',           // Art. 6(1)(e) - Public interest task
  LEGITIMATE_INTEREST = 'legitimate' // Art. 6(1)(f) - Legitimate interests
}

/**
 * Consent event types
 */
export enum ConsentEventType {
  REQUESTED = 'requested',
  GRANTED = 'granted',
  DENIED = 'denied',
  REVOKED = 'revoked',
  RENEWED = 'renewed',
  EXPIRED = 'expired',
  UPDATED = 'updated',
  CHECKED = 'checked',
}

/**
 * Standard consent purposes
 */
export enum ConsentPurpose {
  // Essential
  SERVICE_DELIVERY = 'service_delivery',
  SECURITY = 'security',
  LEGAL_COMPLIANCE = 'legal_compliance',
  FRAUD_PREVENTION = 'fraud_prevention',

  // Functional
  ANALYTICS = 'analytics',
  PERSONALIZATION = 'personalization',
  PERFORMANCE_MONITORING = 'performance_monitoring',
  DEBUGGING = 'debugging',

  // Marketing
  EMAIL_MARKETING = 'email_marketing',
  SMS_MARKETING = 'sms_marketing',
  PUSH_NOTIFICATIONS = 'push_notifications',
  DIRECT_MAIL = 'direct_mail',

  // Social
  SOCIAL_MEDIA_SHARING = 'social_media_sharing',
  USER_CONTENT_SHARING = 'user_content_sharing',
  SOCIAL_LOGIN = 'social_login',

  // Third Party
  ADVERTISING = 'advertising',
  PARTNER_SHARING = 'partner_sharing',
  DATA_SALES = 'data_sales',
  RESEARCH = 'research',

  // CCPA Specific
  CCPA_DO_NOT_SELL = 'ccpa_do_not_sell',
}

/**
 * Data categories for scope definition
 */
export enum DataCategory {
  // Personal Info
  NAME = 'name',
  EMAIL = 'email',
  PHONE = 'phone',
  ADDRESS = 'address',
  BIRTHDATE = 'birthdate',

  // Identifiers
  USER_ID = 'user_id',
  DEVICE_ID = 'device_id',
  IP_ADDRESS = 'ip_address',
  COOKIE_ID = 'cookie_id',

  // Usage Data
  BROWSING_HISTORY = 'browsing_history',
  SEARCH_QUERIES = 'search_queries',
  CLICK_STREAM = 'click_stream',
  SESSION_DURATION = 'session_duration',

  // Preferences
  LANGUAGE = 'language',
  TIMEZONE = 'timezone',
  NOTIFICATION_SETTINGS = 'notification_settings',
  DISPLAY_PREFERENCES = 'display_preferences',

  // Sensitive
  HEALTH_DATA = 'health_data',
  FINANCIAL_DATA = 'financial_data',
  BIOMETRIC_DATA = 'biometric_data',
  POLITICAL_OPINIONS = 'political_opinions',
  RELIGIOUS_BELIEFS = 'religious_beliefs',
}

// ============================================================================
// Core Types
// ============================================================================

/**
 * Main consent record
 */
export interface ConsentRecord {
  // Identifiers
  consentId: string;
  userId: string;
  receiptId: string;

  // Consent details
  purpose: string;
  status: ConsentStatus;
  scope: string[];

  // Legal
  legalBasis: LegalBasis;
  jurisdiction: string;
  version: string;

  // Temporal
  grantedAt?: Date;
  revokedAt?: Date;
  expiresAt?: Date;
  lastChecked?: Date;

  // Provenance
  language: string;
  noticeText: string;
  userAgent?: string;
  ipAddress?: string;
  geoLocation?: string;

  // Verification
  signature: string;
  signatureAlgorithm: string;

  // History
  history: ConsentEvent[];
}

/**
 * Consent event in audit trail
 */
export interface ConsentEvent {
  eventId: string;
  eventType: ConsentEventType;
  timestamp: Date;
  actor: string;
  reason?: string;
  previousStatus?: ConsentStatus;
  newStatus: ConsentStatus;
  metadata?: Record<string, unknown>;
}

/**
 * Purpose definition
 */
export interface PurposeDefinition {
  code: string;
  category: string;
  required: boolean;
  legalBasis: LegalBasis;
  dataCategories: string[];
  retention: string;
  description: Record<string, string>;
  processors: string[];
  jurisdictions: string[];
  policyUrl?: string;
  version: string;
}

/**
 * Consent notice shown to user
 */
export interface ConsentNotice {
  title: string;
  description: string;
  processingDetails: string;
  retention: string;
  rights: string;
  policyUrl?: string;
  processors?: string[];
}

/**
 * Consent receipt for verification
 */
export interface ConsentReceipt {
  receiptId: string;
  consentId: string;
  userId: string;
  purpose: string;
  status: ConsentStatus;
  grantedAt: Date;
  expiresAt?: Date;
  scope: string[];
  legalBasis: LegalBasis;
  jurisdiction: string;
  notice: ConsentNotice;
  signature: string;
  receiptUrl: string;
  qrCode?: string;
}

// ============================================================================
// Request/Response Types
// ============================================================================

/**
 * Request consent from user
 */
export interface ConsentRequest {
  requestId?: string;
  userId: string;
  purpose: string;
  scope: string[];
  legalBasis?: LegalBasis;
  jurisdiction: string;
  language?: string;
  metadata?: {
    requestSource?: string;
    requestContext?: string;
    [key: string]: unknown;
  };
}

/**
 * Consent request response
 */
export interface ConsentRequestResponse {
  requestId: string;
  userId: string;
  purpose: string;
  notice: ConsentNotice;
  expiresAt?: Date;
  metadata?: Record<string, unknown>;
}

/**
 * Grant consent
 */
export interface ConsentGrant {
  requestId: string;
  userId: string;
  accepted: boolean;
  metadata?: {
    userAgent?: string;
    ipAddress?: string;
    geoLocation?: string;
    [key: string]: unknown;
  };
}

/**
 * Consent grant response
 */
export interface ConsentGrantResponse {
  consentId: string;
  status: ConsentStatus;
  grantedAt: Date;
  expiresAt?: Date;
  receiptId: string;
  receiptUrl: string;
  signature: string;
}

/**
 * Check consent status
 */
export interface ConsentCheck {
  userId: string;
  purpose: string;
  scope?: string[];
  jurisdiction?: string;
}

/**
 * Consent check response
 */
export interface ConsentCheckResponse {
  hasConsent: boolean;
  consentId?: string;
  status?: ConsentStatus;
  grantedAt?: Date;
  expiresAt?: Date;
  scope?: string[];
  reason?: string;
}

/**
 * Revoke consent
 */
export interface ConsentRevocation {
  consentId?: string;
  userId: string;
  purpose?: string;
  reason: string;
  deleteData?: boolean;
  effectiveImmediately?: boolean;
  cascadeToProcessors?: boolean;
}

/**
 * Consent revocation response
 */
export interface ConsentRevocationResponse {
  revocationId: string;
  consentId: string;
  status: ConsentStatus;
  revokedAt: Date;
  propagated: boolean;
  dataDeleted?: boolean;
}

/**
 * List consents for user
 */
export interface ConsentListRequest {
  userId: string;
  status?: ConsentStatus[];
  purpose?: string[];
  jurisdiction?: string;
  limit?: number;
  offset?: number;
}

/**
 * Consent list response
 */
export interface ConsentListResponse {
  consents: ConsentSummary[];
  total: number;
  limit: number;
  offset: number;
}

/**
 * Consent summary (for listing)
 */
export interface ConsentSummary {
  consentId: string;
  purpose: string;
  status: ConsentStatus;
  grantedAt?: Date;
  expiresAt?: Date;
  scope: string[];
}

/**
 * Export consent history
 */
export interface ConsentExport {
  userId: string;
  format: 'json' | 'csv' | 'pdf';
  includeHistory?: boolean;
  includeReceipts?: boolean;
}

/**
 * Consent export response
 */
export interface ConsentExportResponse {
  exportId: string;
  format: string;
  downloadUrl: string;
  expiresAt: Date;
  size: number;
}

// ============================================================================
// Propagation Types
// ============================================================================

/**
 * Consent propagation event
 */
export interface ConsentPropagationEvent {
  eventId: string;
  eventType: ConsentEventType;
  consentId: string;
  userId: string;
  purpose: string;
  status: ConsentStatus;
  timestamp: Date;
  metadata?: Record<string, unknown>;
}

/**
 * Propagation configuration
 */
export interface PropagationConfig {
  enabled: boolean;
  targets: PropagationTarget[];
  timeout: number;
  retries: number;
  retryBackoff: number;
}

/**
 * Propagation target
 */
export interface PropagationTarget {
  id: string;
  name: string;
  type: 'webhook' | 'api' | 'queue' | 'database';
  endpoint: string;
  authentication?: {
    type: 'api_key' | 'oauth' | 'jwt';
    credentials: Record<string, string>;
  };
  enabled: boolean;
  priority: number;
}

/**
 * Propagation result
 */
export interface PropagationResult {
  targetId: string;
  success: boolean;
  timestamp: Date;
  latency: number;
  error?: string;
  retries: number;
}

// ============================================================================
// Compliance Types
// ============================================================================

/**
 * GDPR data subject rights request
 */
export interface DataSubjectRequest {
  requestId: string;
  userId: string;
  requestType: 'access' | 'rectification' | 'erasure' | 'restriction' | 'portability' | 'objection';
  purpose?: string;
  reason?: string;
  requestedAt: Date;
  status: 'pending' | 'processing' | 'completed' | 'rejected';
}

/**
 * Compliance report
 */
export interface ComplianceReport {
  reportId: string;
  reportType: 'gdpr_article_30' | 'ccpa_consumer_rights' | 'consent_coverage' | 'revocation_rates';
  generatedAt: Date;
  period: {
    startDate: Date;
    endDate: Date;
  };
  jurisdiction?: string;
  data: Record<string, unknown>;
}

/**
 * Audit log entry
 */
export interface AuditLogEntry {
  auditId: string;
  timestamp: Date;
  action: string;
  actor: string;
  consentId?: string;
  userId?: string;
  purpose?: string;
  previousState?: unknown;
  newState?: unknown;
  metadata?: Record<string, unknown>;
  signature: string;
}

// ============================================================================
// Configuration Types
// ============================================================================

/**
 * Consent manager configuration
 */
export interface ConsentManagerConfig {
  // Storage
  storage: 'memory' | 'encrypted' | 'database';
  encryptionKey?: string;

  // Jurisdiction
  defaultJurisdiction: string;
  defaultLanguage: string;

  // Retention
  retentionPolicies?: {
    activeConsents?: string;    // ISO 8601 duration or 'indefinite'
    revokedConsents?: string;   // Default: P3Y (3 years)
    auditLogs?: string;         // Default: P7Y (7 years)
  };

  // Propagation
  propagation?: PropagationConfig;

  // Security
  signatureAlgorithm?: string;  // Default: SHA256-RSA
  encryptAtRest?: boolean;      // Default: true
  encryptInTransit?: boolean;   // Default: true

  // Validation
  strictValidation?: boolean;   // Default: true
  requireExplicitConsent?: boolean; // Default: true

  // Notifications
  sendReceipts?: boolean;       // Default: true
  sendRenewalReminders?: boolean; // Default: true
  reminderDays?: number;        // Default: 30
}

/**
 * Validation result
 */
export interface ValidationResult {
  valid: boolean;
  errors: ValidationError[];
  warnings: ValidationWarning[];
}

/**
 * Validation error
 */
export interface ValidationError {
  code: string;
  message: string;
  field?: string;
  value?: unknown;
}

/**
 * Validation warning
 */
export interface ValidationWarning {
  code: string;
  message: string;
  field?: string;
  value?: unknown;
}

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Result type for operations that can fail
 */
export type Result<T, E = Error> =
  | { success: true; data: T }
  | { success: false; error: E };

/**
 * Async result type
 */
export type AsyncResult<T, E = Error> = Promise<Result<T, E>>;

/**
 * Pagination parameters
 */
export interface PaginationParams {
  limit?: number;
  offset?: number;
  sortBy?: string;
  sortOrder?: 'asc' | 'desc';
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  items: T[];
  total: number;
  limit: number;
  offset: number;
  hasMore: boolean;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-CORE-002 error codes
 */
export enum ConsentErrorCode {
  INVALID_REQUEST = 'C001',
  USER_NOT_FOUND = 'C002',
  CONSENT_NOT_FOUND = 'C003',
  PURPOSE_NOT_DEFINED = 'C004',
  INVALID_JURISDICTION = 'C005',
  CONSENT_EXPIRED = 'C006',
  ALREADY_GRANTED = 'C007',
  ALREADY_REVOKED = 'C008',
  PROPAGATION_FAILED = 'C009',
  SIGNATURE_INVALID = 'C010',
  ENCRYPTION_FAILED = 'C011',
  VALIDATION_FAILED = 'C012',
}

/**
 * Consent error
 */
export class ConsentError extends Error {
  constructor(
    public code: ConsentErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'ConsentError';
  }
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Default configuration values
 */
export const DEFAULT_CONFIG: Partial<ConsentManagerConfig> = {
  storage: 'encrypted',
  defaultLanguage: 'en',
  signatureAlgorithm: 'SHA256-RSA',
  encryptAtRest: true,
  encryptInTransit: true,
  strictValidation: true,
  requireExplicitConsent: true,
  sendReceipts: true,
  sendRenewalReminders: true,
  reminderDays: 30,
  retentionPolicies: {
    activeConsents: 'indefinite',
    revokedConsents: 'P3Y',
    auditLogs: 'P7Y',
  },
};

/**
 * Standard jurisdictions
 */
export const JURISDICTIONS = {
  EU: 'EU',           // European Union (GDPR)
  US_CA: 'US-CA',     // California (CCPA)
  CA: 'CA',           // Canada (PIPEDA)
  BR: 'BR',           // Brazil (LGPD)
  CN: 'CN',           // China (PIPL)
  JP: 'JP',           // Japan (APPI)
  GB: 'GB',           // United Kingdom (UK GDPR)
  AU: 'AU',           // Australia (Privacy Act)
} as const;

/**
 * Language codes (ISO 639-1)
 */
export const LANGUAGES = {
  EN: 'en',           // English
  KO: 'ko',           // Korean
  ES: 'es',           // Spanish
  DE: 'de',           // German
  FR: 'fr',           // French
  JA: 'ja',           // Japanese
  ZH: 'zh',           // Chinese
  PT: 'pt',           // Portuguese
} as const;

// ============================================================================
// Export All
// ============================================================================

export type {
  ConsentRecord,
  ConsentEvent,
  PurposeDefinition,
  ConsentNotice,
  ConsentReceipt,
  ConsentRequest,
  ConsentRequestResponse,
  ConsentGrant,
  ConsentGrantResponse,
  ConsentCheck,
  ConsentCheckResponse,
  ConsentRevocation,
  ConsentRevocationResponse,
  ConsentListRequest,
  ConsentListResponse,
  ConsentSummary,
  ConsentExport,
  ConsentExportResponse,
  ConsentPropagationEvent,
  PropagationConfig,
  PropagationTarget,
  PropagationResult,
  DataSubjectRequest,
  ComplianceReport,
  AuditLogEntry,
  ConsentManagerConfig,
  ValidationResult,
  ValidationError,
  ValidationWarning,
  PaginationParams,
  PaginatedResponse,
};

export {
  ConsentStatus,
  LegalBasis,
  ConsentEventType,
  ConsentPurpose,
  DataCategory,
  ConsentErrorCode,
  ConsentError,
  DEFAULT_CONFIG,
  JURISDICTIONS,
  LANGUAGES,
};
