/**
 * WIA Digital Erasure Standard - TypeScript Type Definitions
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

export interface WIADigitalErasure {
  standard: 'WIA-DIGITAL-ERASURE';
  version: string;
  request: ErasureRequest;
  subject: DataSubject;
  scope: ErasureScope;
  verification: VerificationConfig;
  execution: ExecutionPlan;
  audit: AuditConfig;
  extensions?: Record<string, unknown>;
}

export interface ErasureRequest {
  id: string;
  type: ErasureType;
  status: ErasureStatus;
  priority: Priority;
  createdAt: string;
  updatedAt?: string;
  deadline?: string;
  legalBasis: LegalBasis;
  requestor: RequestorInfo;
}

export type ErasureType = 'right-to-be-forgotten' | 'account-deletion' | 'data-portability-erasure' | 'consent-withdrawal' | 'legal-requirement';
export type ErasureStatus = 'pending' | 'verified' | 'in-progress' | 'completed' | 'rejected' | 'appealed';
export type Priority = 'low' | 'normal' | 'high' | 'urgent';

export interface LegalBasis {
  regulation: 'gdpr' | 'ccpa' | 'lgpd' | 'pipeda' | 'other';
  article?: string;
  justification: string;
}

export interface RequestorInfo {
  type: 'data-subject' | 'legal-representative' | 'regulator';
  name?: string;
  email?: string;
  verificationMethod: VerificationMethod;
}

export type VerificationMethod = 'email' | 'government-id' | 'two-factor' | 'notarized';

export interface DataSubject {
  id: string;
  identifiers: SubjectIdentifier[];
  accounts: AccountReference[];
  dataCategories: DataCategory[];
}

export interface SubjectIdentifier {
  type: 'email' | 'phone' | 'user-id' | 'ssn' | 'passport' | 'custom';
  value: string;
  verified: boolean;
}

export interface AccountReference {
  service: string;
  accountId: string;
  status: 'active' | 'suspended' | 'deleted';
  dataTypes: string[];
}

export type DataCategory = 'personal' | 'sensitive' | 'biometric' | 'financial' | 'health' | 'location' | 'behavioral' | 'communication';

// ============================================================================
// Scope Types
// ============================================================================

export interface ErasureScope {
  systems: SystemScope[];
  dataTypes: DataTypeScope[];
  timeRange?: TimeRange;
  exclusions?: Exclusion[];
  backups: BackupPolicy;
  thirdParties?: ThirdPartyScope[];
}

export interface SystemScope {
  systemId: string;
  name: string;
  type: SystemType;
  dataLocations: DataLocation[];
  retentionOverride?: boolean;
}

export type SystemType = 'database' | 'file-storage' | 'data-warehouse' | 'backup' | 'log' | 'cache' | 'third-party';

export interface DataLocation {
  type: 'table' | 'collection' | 'file' | 'object' | 'index';
  path: string;
  identifierField: string;
}

export interface DataTypeScope {
  category: DataCategory;
  included: boolean;
  retentionRequired?: boolean;
  retentionReason?: string;
}

export interface TimeRange {
  from?: string;
  to?: string;
  includeHistorical: boolean;
}

export interface Exclusion {
  reason: ExclusionReason;
  dataType: string;
  legalBasis: string;
  retentionPeriod?: number;
}

export type ExclusionReason = 'legal-hold' | 'regulatory-requirement' | 'legitimate-interest' | 'contractual-obligation' | 'public-interest';

export interface BackupPolicy {
  includeBackups: boolean;
  retentionPeriod?: number;
  anonymizationOption?: boolean;
}

export interface ThirdPartyScope {
  vendorId: string;
  vendorName: string;
  dataShared: string[];
  notificationRequired: boolean;
  deletionConfirmation: boolean;
}

// ============================================================================
// Verification Types
// ============================================================================

export interface VerificationConfig {
  steps: VerificationStep[];
  requiredLevel: VerificationLevel;
  timeoutHours: number;
  maxAttempts: number;
}

export interface VerificationStep {
  type: VerificationStepType;
  status: 'pending' | 'passed' | 'failed';
  completedAt?: string;
  details?: Record<string, unknown>;
}

export type VerificationStepType = 'identity' | 'ownership' | 'authorization' | 'consent';
export type VerificationLevel = 'basic' | 'standard' | 'enhanced' | 'maximum';

// ============================================================================
// Execution Types
// ============================================================================

export interface ExecutionPlan {
  phases: ExecutionPhase[];
  rollbackEnabled: boolean;
  dryRunCompleted?: boolean;
  estimatedDuration: number;
  parallelization: boolean;
}

export interface ExecutionPhase {
  id: string;
  name: string;
  order: number;
  status: PhaseStatus;
  tasks: ErasureTask[];
  startedAt?: string;
  completedAt?: string;
}

export type PhaseStatus = 'pending' | 'running' | 'completed' | 'failed' | 'rolled-back';

export interface ErasureTask {
  id: string;
  systemId: string;
  action: ErasureAction;
  status: TaskStatus;
  recordsAffected?: number;
  error?: string;
}

export type ErasureAction = 'delete' | 'anonymize' | 'pseudonymize' | 'encrypt' | 'archive';
export type TaskStatus = 'pending' | 'running' | 'completed' | 'failed' | 'skipped';

// ============================================================================
// Audit Types
// ============================================================================

export interface AuditConfig {
  enabled: boolean;
  retention: number;
  immutable: boolean;
  events: AuditEvent[];
  certificate?: ErasureCertificate;
}

export interface AuditEvent {
  id: string;
  timestamp: string;
  action: string;
  actor: string;
  details: Record<string, unknown>;
  hash?: string;
}

export interface ErasureCertificate {
  id: string;
  issuedAt: string;
  scope: string;
  systemsAffected: string[];
  recordsDeleted: number;
  signedBy: string;
  signature: string;
}

// ============================================================================
// API Types
// ============================================================================

export interface APIConfig {
  baseURL: string;
  apiKey?: string;
  timeout?: number;
}

export interface ErasureResponse {
  id: string;
  status: ErasureStatus;
  subjectId: string;
  createdAt: string;
  completedAt?: string;
  links: { self: string };
}

export interface ValidationResult {
  valid: boolean;
  errors?: { path: string; message: string }[];
}

export interface PaginatedResponse<T> {
  data: T[];
  pagination: { total: number; limit: number; offset: number; hasMore: boolean };
}
