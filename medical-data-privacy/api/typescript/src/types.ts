/**
 * WIA-MED-017 Medical Data Privacy Standard - Type Definitions
 * @version 1.0.0
 */

// ============================================================================
// Core Types
// ============================================================================

export type SensitivityLevel = 'public' | 'internal' | 'confidential' | 'restricted';
export type DataClassification = 'phi' | 'pii' | 'sensitive' | 'general';
export type EncryptionAlgorithm = 'AES-256-GCM' | 'AES-128-GCM' | 'ChaCha20-Poly1305';
export type ConsentStatus = 'granted' | 'withdrawn' | 'expired' | 'pending';
export type AccessLevel = 'read' | 'write' | 'admin' | 'audit';
export type BreachSeverity = 'low' | 'medium' | 'high' | 'critical';
export type ComplianceStandard = 'HIPAA' | 'GDPR' | 'HITECH' | 'ISO27001' | 'ISO27701';

// ============================================================================
// Protected Health Information (PHI)
// ============================================================================

export interface PHIData {
  // Core Identity
  id: string;
  patientId: string;

  // HIPAA 18 Identifiers (de-identification tracking)
  identifiers: {
    names?: boolean;
    geographicSubdivisions?: boolean;
    dates?: boolean;
    telephoneNumbers?: boolean;
    faxNumbers?: boolean;
    emailAddresses?: boolean;
    socialSecurityNumbers?: boolean;
    medicalRecordNumbers?: boolean;
    healthPlanNumbers?: boolean;
    accountNumbers?: boolean;
    certificateNumbers?: boolean;
    vehicleIdentifiers?: boolean;
    deviceIdentifiers?: boolean;
    urls?: boolean;
    ipAddresses?: boolean;
    biometricIdentifiers?: boolean;
    facePhotographicImages?: boolean;
    otherUniqueIdentifiers?: boolean;
  };

  // Data Classification
  classification: DataClassification;
  sensitivityLevel: SensitivityLevel;

  // Encryption
  encryption: EncryptionConfig;

  // Access Control
  accessControl: AccessControlPolicy;

  // Consent
  consent?: ConsentRecord;

  // Metadata
  createdAt: string;
  updatedAt: string;
  expiresAt?: string;
}

export interface EncryptionConfig {
  algorithm: EncryptionAlgorithm;
  keyId: string;
  keyRotationDate: string;
  nextRotationDate: string;
  isEncryptedAtRest: boolean;
  isEncryptedInTransit: boolean;
  tlsVersion?: string;
}

// ============================================================================
// Access Control
// ============================================================================

export interface AccessControlPolicy {
  id: string;
  resourceId: string;

  // RBAC
  roles: Role[];

  // Permissions
  permissions: Permission[];

  // MFA Requirement
  requireMFA: boolean;

  // Session Management
  sessionTimeout: number; // minutes

  // IP Restrictions
  allowedIPs?: string[];

  // Time-based Access
  accessSchedule?: AccessSchedule;
}

export interface Role {
  id: string;
  name: string;
  level: AccessLevel;
  permissions: string[];
}

export interface Permission {
  action: 'read' | 'write' | 'delete' | 'share' | 'export';
  resource: string;
  conditions?: PermissionCondition[];
}

export interface PermissionCondition {
  type: 'time' | 'location' | 'device' | 'network';
  value: string;
}

export interface AccessSchedule {
  timezone: string;
  allowedDays: number[]; // 0-6 (Sunday-Saturday)
  allowedHours: { start: string; end: string }[];
}

// ============================================================================
// Consent Management
// ============================================================================

export interface ConsentRecord {
  id: string;
  patientId: string;

  // GDPR Requirements
  isFreely: boolean;
  isSpecific: boolean;
  isInformed: boolean;
  isUnambiguous: boolean;

  // Consent Details
  purposes: ConsentPurpose[];
  grantedAt: string;
  withdrawnAt?: string;
  expiresAt?: string;
  status: ConsentStatus;

  // Audit Trail
  auditLog: ConsentAuditEntry[];

  // Granular Options
  dataTypes: string[];
  processingActivities: string[];
  thirdPartySharing: boolean;
  internationalTransfer: boolean;
}

export interface ConsentPurpose {
  id: string;
  name: string;
  description: string;
  required: boolean;
  granted: boolean;
}

export interface ConsentAuditEntry {
  timestamp: string;
  action: 'grant' | 'withdraw' | 'modify' | 'renew';
  userId?: string;
  ipAddress?: string;
  userAgent?: string;
}

// ============================================================================
// De-identification
// ============================================================================

export interface DeIdentificationConfig {
  method: 'safe_harbor' | 'expert_determination';

  // k-anonymity
  kAnonymity?: {
    k: number; // recommended: k >= 5
    quasiIdentifiers: string[];
  };

  // Differential Privacy
  differentialPrivacy?: {
    epsilon: number; // recommended: ε <= 1.0
    delta?: number;
    mechanism: 'laplace' | 'gaussian' | 'exponential';
  };

  // Suppression Rules
  suppressionRules?: SuppressionRule[];

  // Generalization Rules
  generalizationRules?: GeneralizationRule[];
}

export interface SuppressionRule {
  field: string;
  condition?: string;
  replacement: string;
}

export interface GeneralizationRule {
  field: string;
  method: 'range' | 'category' | 'hierarchy';
  parameters: any;
}

// ============================================================================
// Audit Logging
// ============================================================================

export interface AuditLog {
  id: string;

  // Event Details
  timestamp: string;
  eventType: AuditEventType;
  resource: string;
  resourceId: string;

  // User Information
  userId?: string;
  userRole?: string;

  // Action Details
  action: string;
  result: 'success' | 'failure' | 'partial';

  // Network Information
  ipAddress?: string;
  userAgent?: string;
  location?: GeoLocation;

  // Data Access
  dataAccessed?: {
    fields: string[];
    recordCount: number;
    query?: string;
  };

  // Changes
  changes?: {
    before?: any;
    after?: any;
  };

  // Tamper-proof
  hash: string;
  previousHash?: string;

  // Retention
  retentionDate: string; // 6 years minimum (HIPAA)
}

export type AuditEventType =
  | 'access'
  | 'create'
  | 'update'
  | 'delete'
  | 'export'
  | 'share'
  | 'consent_grant'
  | 'consent_withdraw'
  | 'login'
  | 'logout'
  | 'failed_login'
  | 'permission_change'
  | 'encryption_key_rotation'
  | 'breach_detection';

export interface GeoLocation {
  country?: string;
  region?: string;
  city?: string;
  latitude?: number;
  longitude?: number;
}

// ============================================================================
// Breach Notification
// ============================================================================

export interface BreachIncident {
  id: string;

  // Incident Details
  detectedAt: string;
  reportedAt?: string;
  resolvedAt?: string;

  // Classification
  severity: BreachSeverity;
  type: BreachType;

  // Affected Data
  affectedRecords: number;
  affectedIndividuals: number;
  dataTypes: string[];

  // Description
  description: string;
  cause?: string;

  // Impact Assessment
  impactAssessment: {
    likelihood: 'low' | 'medium' | 'high';
    harm: 'minimal' | 'moderate' | 'substantial';
    recommendation: string;
  };

  // Notification Requirements
  notifications: BreachNotification[];

  // Remediation
  remediationSteps: string[];
  preventiveMeasures: string[];

  // Compliance
  hipaaNotificationDeadline?: string; // 60 days
  gdprNotificationDeadline?: string; // 72 hours
}

export type BreachType =
  | 'unauthorized_access'
  | 'theft'
  | 'loss'
  | 'improper_disposal'
  | 'hacking'
  | 'malware'
  | 'ransomware'
  | 'insider_threat'
  | 'accidental_disclosure'
  | 'other';

export interface BreachNotification {
  recipient: 'individual' | 'authority' | 'media';
  authority?: 'HHS' | 'ICO' | 'CNIL' | 'other';
  sentAt?: string;
  deadline: string;
  method: 'email' | 'mail' | 'phone' | 'website' | 'media';
  status: 'pending' | 'sent' | 'acknowledged';
}

// ============================================================================
// Cross-Border Transfer
// ============================================================================

export interface CrossBorderTransfer {
  id: string;

  // Transfer Details
  fromCountry: string;
  toCountry: string;
  dataTypes: string[];

  // Legal Basis (GDPR Chapter V)
  legalMechanism: TransferMechanism;

  // Assessment
  transferImpactAssessment?: TransferImpactAssessment;

  // Safeguards
  additionalSafeguards: string[];

  // Documentation
  agreements: string[]; // SCCs, BCRs, etc.

  // Approval
  approvedBy?: string;
  approvedAt?: string;
  reviewDate: string;
}

export type TransferMechanism =
  | 'adequacy_decision'
  | 'standard_contractual_clauses'
  | 'binding_corporate_rules'
  | 'approved_code_of_conduct'
  | 'approved_certification'
  | 'explicit_consent'
  | 'contractual_necessity'
  | 'derogations';

export interface TransferImpactAssessment {
  riskLevel: 'low' | 'medium' | 'high';
  governmentAccess: boolean;
  surveillanceLaws: string[];
  legalRemedies: string[];
  additionalMeasures: string[];
  completedAt: string;
  completedBy: string;
}

// ============================================================================
// API Request/Response Types
// ============================================================================

export interface EncryptDataRequest {
  data: any;
  algorithm?: EncryptionAlgorithm;
  keyId?: string;
}

export interface EncryptDataResponse {
  encryptedData: string;
  keyId: string;
  algorithm: EncryptionAlgorithm;
  iv: string;
}

export interface DeIdentifyRequest {
  data: any;
  config: DeIdentificationConfig;
}

export interface DeIdentifyResponse {
  deidentifiedData: any;
  suppressedFields: string[];
  generalizedFields: string[];
  qualityMetrics: {
    informationLoss: number;
    kAnonymity?: number;
    epsilonUsed?: number;
  };
}

export interface CreateConsentRequest {
  patientId: string;
  purposes: ConsentPurpose[];
  dataTypes: string[];
  processingActivities: string[];
  thirdPartySharing?: boolean;
  internationalTransfer?: boolean;
  expiresAt?: string;
}

export interface CreateConsentResponse {
  consentId: string;
  status: ConsentStatus;
  grantedAt: string;
  expiresAt?: string;
}

export interface AccessRequestParams {
  userId: string;
  resourceId: string;
  action: string;
  context?: {
    ipAddress?: string;
    userAgent?: string;
    mfaVerified?: boolean;
  };
}

export interface AccessRequestResponse {
  granted: boolean;
  reason?: string;
  conditions?: string[];
  auditLogId: string;
}

export interface ReportBreachRequest {
  type: BreachType;
  severity: BreachSeverity;
  description: string;
  affectedRecords: number;
  affectedIndividuals: number;
  dataTypes: string[];
  detectedAt: string;
}

export interface ReportBreachResponse {
  incidentId: string;
  hipaaDeadline: string;
  gdprDeadline: string;
  recommendedActions: string[];
}

export interface ComplianceCheckRequest {
  standard: ComplianceStandard;
  scope: string[];
}

export interface ComplianceCheckResponse {
  compliant: boolean;
  standard: ComplianceStandard;
  score: number;
  findings: ComplianceFinding[];
  recommendations: string[];
}

export interface ComplianceFinding {
  requirement: string;
  status: 'pass' | 'fail' | 'warning';
  description: string;
  evidence?: string;
}

// ============================================================================
// SDK Configuration
// ============================================================================

export interface SDKConfig {
  apiKey: string;
  environment: 'production' | 'staging' | 'development';
  baseUrl?: string;
  timeout?: number;
  retryAttempts?: number;

  // Security
  enableEncryption?: boolean;
  kmsProvider?: 'aws' | 'azure' | 'gcp' | 'hsm';

  // Compliance
  complianceMode?: ComplianceStandard[];
}

// ============================================================================
// Error Types
// ============================================================================

export class MedicalDataPrivacyError extends Error {
  constructor(
    message: string,
    public code: string,
    public statusCode?: number,
    public details?: any
  ) {
    super(message);
    this.name = 'MedicalDataPrivacyError';
  }
}

export interface APIErrorResponse {
  error: {
    code: string;
    message: string;
    details?: any;
  };
}
